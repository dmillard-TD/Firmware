/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file bmp388.cpp
 * Driver for the BMP388 barometric pressure sensor connected via I2C TODO or SPI.
 */

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <px4_getopt.h>
#include <px4_log.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>
#include <board_config.h>
#include "bmp388.h"

#include <lib/cdev/CDev.hpp>
#include <drivers/drv_baro.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>


enum BMP388_BUS {
	BMP388_BUS_ALL = 0,
	BMP388_BUS_I2C_INTERNAL,
	BMP388_BUS_I2C_EXTERNAL,
	BMP388_BUS_SPI_INTERNAL,
	BMP388_BUS_SPI_EXTERNAL
};

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

/*
 * BMP388 internal constants and data structures.
 */

class BMP388 : public cdev::CDev
{
public:
	BMP388(bmp388::IBMP388 *interface, const char *path);
	virtual ~BMP388();

	virtual int		init();

	virtual ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

private:
	bmp388::IBMP388	*_interface;

	bool			_running;

	uint8_t			_curr_ctrl;

	struct work_s		_work;
	unsigned		_report_ticks; // 0 - no cycling, otherwise period of sending a report
	unsigned		_max_mesure_ticks; //ticks needed to measure

	ringbuffer::RingBuffer	*_reports;

	bool			_collect_phase;

	orb_advert_t		_baro_topic;
	int			_orb_class_instance;
	int			_class_instance;

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_perf;
	perf_counter_t		_comms_errors;

	struct bmp388::calibration_s *_cal; //stored calibration constants
	struct bmp388::fcalibration_s _fcal; //pre processed calibration constants

	float			_P; /* in Pa */
	float			_T; /* in K */


	/* periodic execution helpers */
	void			start_cycle();
	void			stop_cycle();
	void			cycle(); //main execution
	static void		cycle_trampoline(void *arg);

	int		measure(); //start measure
	int		collect(); //get results and publish
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int bmp388_main(int argc, char *argv[]);

BMP388::BMP388(bmp388::IBMP388 *interface, const char *path) :
	CDev(path),
	_interface(interface),
	_running(false),
	_report_ticks(0),
	_reports(nullptr),
	_collect_phase(false),
	_baro_topic(nullptr),
	_orb_class_instance(-1),
	_class_instance(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "bmp388_read")),
	_measure_perf(perf_alloc(PC_ELAPSED, "bmp388_measure")),
	_comms_errors(perf_alloc(PC_COUNT, "bmp388_comms_errors"))
{
	// work_cancel in stop_cycle called from the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

BMP388::~BMP388()
{
	/* make sure we are truly inactive */
	stop_cycle();

	if (_class_instance != -1) {
		unregister_class_devname(BARO_BASE_DEVICE_PATH, _class_instance);
	}

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_baro_topic != nullptr) {
		orb_unadvertise(_baro_topic);
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);

	delete _interface;
}

int
BMP388::init()
{
	int ret = CDev::init();

	if (ret != OK) {
		PX4_ERR("CDev init failed");
		return ret;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_baro_s));

	if (_reports == nullptr) {
		PX4_ERR("can't get memory for reports");
		ret = -ENOMEM;
		return ret;
	}

	/* register alternate interfaces if we have to */
	_class_instance = register_class_devname(BARO_BASE_DEVICE_PATH);

	/* reset sensor */
	_interface->set_reg(BMP388_VALUE_RESET, BMP388_ADDR_CMD);
	usleep(10000);

	/* check  id*/
	if (_interface->get_reg(BMP388_ADDR_ID) != BMP388_VALUE_ID) {
		PX4_WARN("id of your baro is not: 0x%02x", BMP388_VALUE_ID);
		return -EIO;
	}

	/* set config, recommended settings */
	_curr_ctrl = BMP388_CTRL_MODE_P | BMP388_CTRL_MODE_T;

	/* set oversampling */
	_interface->set_reg(BMP388_CTRL_P16 | BMP388_CTRL_T2, BMP388_ADDR_OSR);

	_max_mesure_ticks = USEC2TICK(BMP388_MT_INIT + BMP388_MT * (16 - 1 + 2 - 1));

	/* configure IIR filter */
	_interface->set_reg(BMP388_CONFIG_F15, BMP388_ADDR_CONFIG);


	/* get calibration and pre process them*/
	_cal = _interface->get_calibration(BMP388_ADDR_CAL);

	_fcal.t1 = _cal->t1 / powf(2, -8);
	_fcal.t2 = _cal->t2 / powf(2, 30);
	_fcal.t3 = _cal->t3 / powf(2, 48);

	_fcal.p1 = (_cal->p1 - powf(2, 14)) / powf(2, 20);
	_fcal.p2 = (_cal->p2 - powf(2, 14)) / powf(2, 29);
	_fcal.p3 = _cal->p3 / powf(2, 32);

	_fcal.p4 = _cal->p4 / powf(2, 37);
	_fcal.p5 = _cal->p5 / powf(2, -3);
	_fcal.p6 = _cal->p6 / powf(2, 6);

	_fcal.p7 = _cal->p7 / powf(2, 8);
	_fcal.p8 = _cal->p8 / powf(2, 15);
	_fcal.p9 = _cal->p9 / powf(2, 48);

	_fcal.p10 = _cal->p10 / powf(2, 48);
	_fcal.p11 = _cal->p11 / powf(2, 65);

	/* do a first measurement cycle to populate reports with valid data */
	sensor_baro_s brp;
	_reports->flush();

	if (measure()) {
		return -EIO;
	}

	usleep(TICK2USEC(_max_mesure_ticks));

	if (collect()) {
		return -EIO;
	}

	_reports->get(&brp);

	_baro_topic = orb_advertise_multi(ORB_ID(sensor_baro), &brp,
					  &_orb_class_instance, _interface->is_external() ? ORB_PRIO_HIGH : ORB_PRIO_DEFAULT);

	if (_baro_topic == nullptr) {
		PX4_WARN("failed to create sensor_baro publication");
		return -ENOMEM;
	}

	return OK;

}

ssize_t
BMP388::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(sensor_baro_s);
	sensor_baro_s *brp = reinterpret_cast<sensor_baro_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_report_ticks > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(brp)) {
				ret += sizeof(*brp);
				brp++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */

	_reports->flush();

	if (measure()) {
		return -EIO;
	}

	usleep(TICK2USEC(_max_mesure_ticks));

	if (collect()) {
		return -EIO;
	}

	if (_reports->get(brp)) { //get new generated report
		ret = sizeof(*brp);
	}

	return ret;
}

int
BMP388::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {

			unsigned ticks = 0;

			switch (arg) {

			case 0:
				return -EINVAL;

			case SENSOR_POLLRATE_DEFAULT:
				ticks = _max_mesure_ticks;

			/* FALLTHROUGH */
			default: {
					if (ticks == 0) {
						ticks = USEC2TICK(USEC_PER_SEC / arg);
					}

					/* do we need to start internal polling? */
					bool want_start = (_report_ticks == 0);

					/* check against maximum rate */
					if (ticks < _max_mesure_ticks) {
						return -EINVAL;
					}

					_report_ticks = ticks;

					if (want_start) {
						start_cycle();
					}

					return OK;
				}
			}

			break;
		}

	case SENSORIOCRESET:
		/*
		 * Since we are initialized, we do not need to do anything, since the
		 * PROM is correctly read and the part does not need to be configured.
		 */
		return OK;

	default:
		break;
	}

	return CDev::ioctl(filp, cmd, arg);
}

void
BMP388::start_cycle()
{

	/* reset the report ring and state machine */
	_collect_phase = false;
	_running = true;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&BMP388::cycle_trampoline, this, 1);
}

void
BMP388::stop_cycle()
{
	_running = false;
	work_cancel(HPWORK, &_work);
}

void
BMP388::cycle_trampoline(void *arg)
{
	BMP388 *dev = reinterpret_cast<BMP388 *>(arg);

	dev->cycle();
}

void
BMP388::cycle()
{
	if (_collect_phase) {
		collect();
		unsigned wait_gap = _report_ticks - _max_mesure_ticks;

		if ((wait_gap != 0) && (_running)) {
			work_queue(HPWORK, &_work, (worker_t)&BMP388::cycle_trampoline, this,
				   wait_gap); //need to wait some time before new measurement
			return;
		}

	}

	measure();

	if (_running) {
		work_queue(HPWORK, &_work, (worker_t)&BMP388::cycle_trampoline, this, _max_mesure_ticks);
	}

}

int
BMP388::measure()
{
	_collect_phase = true;

	perf_begin(_measure_perf);

	/* start measure */
	int ret = _interface->set_reg(_curr_ctrl | BMP388_CTRL_MODE_FORCE, BMP388_ADDR_PWR_CTRL);

	if (ret != OK) {
		perf_count(_comms_errors);
		perf_cancel(_measure_perf);
		return -EIO;
	}

	perf_end(_measure_perf);

	return OK;
}

int
BMP388::collect()
{
	_collect_phase = false;

	perf_begin(_sample_perf);

	sensor_baro_s report;
	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	report.timestamp = hrt_absolute_time();
	report.error_count = perf_event_count(_comms_errors);

	bmp388::data_s *data = _interface->get_data(BMP388_ADDR_DATA);

	if (data == nullptr) {
		perf_count(_comms_errors);
		perf_cancel(_sample_perf);
		return -EIO;
	}

	//convert data to number 20 bit
	uint32_t p_raw =  data->p_msb << 16 | data->p_lsb << 8 | data->p_xlsb;
	uint32_t t_raw =  data->t_msb << 16 | data->t_lsb << 8 | data->t_xlsb;

	// Temperature
	float pd1 = (float) t_raw - _fcal.t1;
	float pd2 = (pd1 * _fcal.t2);
	_T = pd2 + (pd1 * pd1) * _fcal.t3;

	// Pressure
	pd1 = _fcal.p6 * _T;
	pd2 = _fcal.p7 * (_T * _T);
	float pd3 = _fcal.p8 * (_T * _T * _T);
	float po1 = _fcal.p5 + pd1 + pd2 + pd3;

	pd1 = _fcal.p2 * _T;
	pd2 = _fcal.p3 * (_T * _T);
	pd3 = _fcal.p4 * (_T * _T * _T);
	float po2 = p_raw * (_fcal.p1 + pd1 + pd2 + pd3);

	pd1 = p_raw * p_raw;
	pd2 = _fcal.p9 + (_fcal.p10 * _T);
	pd3 = pd1 * pd2;
	float pd4 = pd3 + ((p_raw * p_raw * p_raw) * _fcal.p11);
	_P = po1 + po2 + pd4;


	report.temperature = _T;
	report.pressure = _P / 100.0f; // to mbar

	/* publish it */
	orb_publish(ORB_ID(sensor_baro), _baro_topic, &report);

	_reports->force(&report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	perf_end(_sample_perf);

	return OK;
}

void
BMP388::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u us \n", _report_ticks * USEC_PER_TICK);
	_reports->print_info("report queue");

	sensor_baro_s brp = {};
	_reports->get(&brp);
	print_message(brp);
}

/**
 * Local functions in support of the shell command.
 */
namespace bmp388
{

/*
  list of supported bus configurations
 */
struct bmp388_bus_option {
	enum BMP388_BUS busid;
	const char *devpath;
	BMP388_constructor interface_constructor;
	uint8_t busnum;
	uint32_t device;
	bool external;
	BMP388 *dev;
} bus_options[] = {
#if defined(PX4_SPIDEV_EXT_BARO) && defined(PX4_SPI_BUS_EXT)
	{ BMP388_BUS_SPI_EXTERNAL, "/dev/bmp388_spi_ext", &bmp388_spi_interface, PX4_SPI_BUS_EXT, PX4_SPIDEV_EXT_BARO, true, NULL },
#endif
#if defined(PX4_SPIDEV_BARO)
#  if defined(PX4_SPIDEV_BARO_BUS)
	{ BMP388_BUS_SPI_INTERNAL, "/dev/bmp388_spi_int", &bmp388_spi_interface, PX4_SPIDEV_BARO_BUS, PX4_SPIDEV_BARO, false, NULL },
#  else
	{ BMP388_BUS_SPI_INTERNAL, "/dev/bmp388_spi_int", &bmp388_spi_interface, PX4_SPI_BUS_SENSORS, PX4_SPIDEV_BARO, false, NULL },
#  endif
#endif
#if defined(PX4_I2C_BUS_ONBOARD) && defined(PX4_I2C_OBDEV_BMP388)
	{ BMP388_BUS_I2C_INTERNAL, "/dev/bmp388_i2c_int", &bmp388_i2c_interface, PX4_I2C_BUS_ONBOARD, PX4_I2C_OBDEV_BMP388, false, NULL },
#endif
#if defined(PX4_I2C_BUS_EXPANSION) && defined(PX4_I2C_EXT_OBDEV_BMP388)
	{ BMP388_BUS_I2C_EXTERNAL, "/dev/bmp388_i2c_ext", &bmp388_i2c_interface, PX4_I2C_BUS_EXPANSION, PX4_I2C_EXT_OBDEV_BMP388, true, NULL },
#endif
#if defined(PX4_SPI_BUS_EXTERNAL1)
	{ BMP388_BUS_SPI_EXTERNAL, "/dev/bmp388_spi_ext", &bmp388_spi_interface, PX4_SPI_BUS_EXTERNAL1, PX4_SPIDEV_EXTERNAL1_1, true, NULL },
#endif

};
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

bool	start_bus(struct bmp388_bus_option &bus);
struct	bmp388_bus_option &find_bus(enum BMP388_BUS busid);
void	start(enum BMP388_BUS busid);
void	test(enum BMP388_BUS busid);
void	reset(enum BMP388_BUS busid);
void	info();
void	usage();


/**
 * Start the driver.
 */
bool
start_bus(struct bmp388_bus_option &bus)
{
	if (bus.dev != nullptr) {
		PX4_ERR("bus option already started");
		exit(1);
	}

	bmp388::IBMP388 *interface = bus.interface_constructor(bus.busnum, bus.device, bus.external);

	if (interface->init() != OK) {
		delete interface;
		PX4_WARN("no device on bus %u", (unsigned)bus.busid);
		return false;
	}

	bus.dev = new BMP388(interface, bus.devpath);

	if (bus.dev == nullptr) {
		return false;
	}

	if (OK != bus.dev->init()) {
		delete bus.dev;
		bus.dev = nullptr;
		return false;
	}

	int fd = open(bus.devpath, O_RDONLY);

	/* set the poll rate to default, starts automatic data collection */
	if (fd == -1) {
		PX4_ERR("can't open baro device");
		exit(1);
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		close(fd);
		PX4_ERR("failed setting default poll rate");
		exit(1);
	}

	close(fd);
	return true;
}


/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * is either successfully up and running or failed to start.
 */
void
start(enum BMP388_BUS busid)
{
	uint8_t i;
	bool started = false;

	for (i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == BMP388_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (busid != BMP388_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}

		started |= start_bus(bus_options[i]);
	}

	if (!started) {
		PX4_ERR("driver start failed");
		exit(1);
	}

	// one or more drivers started OK
	exit(0);
}


/**
 * find a bus structure for a busid
 */
struct bmp388_bus_option &find_bus(enum BMP388_BUS busid)
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == BMP388_BUS_ALL ||
		     busid == bus_options[i].busid) && bus_options[i].dev != NULL) {
			return bus_options[i];
		}
	}

	exit(1);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test(enum BMP388_BUS busid)
{
	struct bmp388_bus_option &bus = find_bus(busid);
	sensor_baro_s report;
	ssize_t sz;
	int ret;

	int fd;

	fd = open(bus.devpath, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("open failed (try 'bmp388 start' if the driver is not running)");
		exit(1);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		PX4_ERR("immediate read failed");
		exit(1);
	}

	print_message(report);

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			PX4_ERR("timed out waiting for sensor data");
			exit(1);
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			PX4_ERR("periodic read failed");
			exit(1);
		}

		print_message(report);
	}

	close(fd);
	PX4_ERR("PASS");
	exit(0);
}

/**
 * Reset the driver.
 */
void
reset(enum BMP388_BUS busid)
{
	struct bmp388_bus_option &bus = find_bus(busid);
	int fd;

	fd = open(bus.devpath, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("failed ");
		exit(1);
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		PX4_ERR("driver reset failed");
		exit(1);
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("driver poll restart failed");
		exit(1);
	}

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		struct bmp388_bus_option &bus = bus_options[i];

		if (bus.dev != nullptr) {
			PX4_WARN("%s", bus.devpath);
			bus.dev->print_info();
		}
	}

	exit(0);
}

void
usage()
{
	PX4_WARN("missing command: try 'start', 'info', 'test', 'test2', 'reset'");
	PX4_WARN("options:");
	PX4_WARN("    -X    (external I2C bus TODO)");
	PX4_WARN("    -I    (internal I2C bus TODO)");
	PX4_WARN("    -S    (external SPI bus)");
	PX4_WARN("    -s    (internal SPI bus)");
}

} // namespace

int
bmp388_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	enum BMP388_BUS busid = BMP388_BUS_ALL;

	while ((ch = px4_getopt(argc, argv, "XISs", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			busid = BMP388_BUS_I2C_EXTERNAL;
			break;

		case 'I':
			busid = BMP388_BUS_I2C_INTERNAL;
			break;

		case 'S':
			busid = BMP388_BUS_SPI_EXTERNAL;
			break;

		case 's':
			busid = BMP388_BUS_SPI_INTERNAL;
			break;

		default:
			bmp388::usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		bmp388::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		bmp388::start(busid);
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		bmp388::test(busid);
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		bmp388::reset(busid);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		bmp388::info();
	}

	PX4_ERR("unrecognized command, try 'start', 'test', 'reset' or 'info'");
	return -1;
}
