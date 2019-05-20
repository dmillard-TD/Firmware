/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "BMI088_accel.hpp"

#include <ecl/geo/geo.h>

/*
  list of registers that will be checked in check_registers(). Note
  that ADDR_WHO_AM_I must be first in the list.
 */
const uint8_t BMI088_accel::_checked_registers[BMI088_ACCEL_NUM_CHECKED_REGISTERS] = {    BMI088_ACC_CHIP_ID,
											  BMI088_ACC_CONF,
											  BMI088_ACC_RANGE,
										     };

BMI088_accel::BMI088_accel(int bus, const char *path_accel, uint32_t device, enum Rotation rotation) :
	BMI088("BMI088_ACCEL", path_accel, bus, device, SPIDEV_MODE3, BMI088_BUS_SPEED, rotation),
	_sample_perf(perf_alloc(PC_ELAPSED, "bmi088_accel_read")),
	_measure_interval(perf_alloc(PC_INTERVAL, "bmi088_accel_measure_interval")),
	_bad_transfers(perf_alloc(PC_COUNT, "bmi088_accel_bad_transfers")),
	_bad_registers(perf_alloc(PC_COUNT, "bmi088_accel_bad_registers")),
	_duplicates(perf_alloc(PC_COUNT, "bmi088_accel_duplicates")),
	_accel_reports(nullptr),
	_accel_scale{},
	_accel_range_scale(0.0f),
	_accel_range_m_s2(0.0f),
	_accel_topic(nullptr),
	_accel_orb_class_instance(-1),
	_accel_class_instance(-1),
	_accel_filter_x(BMI088_ACCEL_DEFAULT_RATE, BMI088_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_y(BMI088_ACCEL_DEFAULT_RATE, BMI088_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_z(BMI088_ACCEL_DEFAULT_RATE, BMI088_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_int(1000000 / BMI088_ACCEL_MAX_PUBLISH_RATE),
	_last_temperature(0),
	_got_duplicate(false)
{
	_device_id.devid_s.devtype = DRV_ACC_DEVTYPE_BMI088;

	// default accel scale factors
	_accel_scale.x_offset = 0;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0;
	_accel_scale.z_scale  = 1.0f;

	memset(&_call, 0, sizeof(_call));
}

BMI088_accel::~BMI088_accel()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_accel_reports != nullptr) {
		delete _accel_reports;
	}

	if (_accel_class_instance != -1) {
		unregister_class_devname(ACCEL_BASE_DEVICE_PATH, _accel_class_instance);
	}

	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_measure_interval);
	perf_free(_bad_transfers);
	perf_free(_bad_registers);
	perf_free(_duplicates);
}

int
BMI088_accel::init()
{
	/* do SPI init (and probe) first */
	int ret = SPI::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		return ret;
	}

	/* allocate basic report buffers */
	_accel_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_accel_s));

	if (_accel_reports == nullptr) {
		return -ENOMEM;
	}

	ret = reset();

	if (ret != OK) {
		return ret;
	}

	/* Initialize offsets and scales */
	_accel_scale.x_offset = 0;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0;
	_accel_scale.z_scale  = 1.0f;

	_accel_class_instance = register_class_devname(ACCEL_BASE_DEVICE_PATH);

	param_t accel_cut_ph = param_find("IMU_ACCEL_CUTOFF");
	float accel_cut = BMI088_ACCEL_DEFAULT_DRIVER_FILTER_FREQ;

	if (accel_cut_ph != PARAM_INVALID && (param_get(accel_cut_ph, &accel_cut) == PX4_OK)) {

		_accel_filter_x.set_cutoff_frequency(BMI088_ACCEL_DEFAULT_RATE, accel_cut);
		_accel_filter_y.set_cutoff_frequency(BMI088_ACCEL_DEFAULT_RATE, accel_cut);
		_accel_filter_z.set_cutoff_frequency(BMI088_ACCEL_DEFAULT_RATE, accel_cut);
	}

	measure();

	/* advertise sensor topic, measure manually to initialize valid report */
	sensor_accel_s arp;
	_accel_reports->get(&arp);

	/* measurement will have generated a report, publish */
	_accel_topic = orb_advertise_multi(ORB_ID(sensor_accel), &arp,
					   &_accel_orb_class_instance, (external()) ? ORB_PRIO_MAX - 1 : ORB_PRIO_HIGH - 1);

	if (_accel_topic == nullptr) {
		warnx("ADVERT FAIL");
	}

	return ret;
}

int BMI088_accel::reset()
{
	write_reg(BMI088_ACC_SOFTRESET, BMI088_SOFT_RESET);//Soft-reset
	up_udelay(5000);

	write_checked_reg(BMI088_ACC_CONF, BMI088_BWP_NORMAL | BMI088_ODR_1600); //Write accel bandwidth and OS rate
	write_checked_reg(BMI088_ACC_RANGE, BMI088_ACCEL_RANGE_3_G); //Write range
	//write_checked_reg(BMI088_ACC_INT_EN_1, BMI088_ACC_DRDY_INT_EN); //Enable DRDY interrupt
	//write_checked_reg(BMI088_ACC_INT_MAP_1, BMI088_ACC_DRDY_INT1); //Map DRDY interrupt on pin INT1

	set_accel_range(BMI088_ACCEL_DEFAULT_RANGE_G);//set accel range

	//Enable Accelerometer in normal mode
	write_reg(BMI088_ACC_PWR_CTRL, BMI088_PWR_CTRL_ON); // Switch on accelerometer
	write_reg(BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE);
	up_udelay(1000);

	uint8_t retries = 10;

	while (retries--) {
		bool all_ok = true;

		for (uint8_t i = 0; i < BMI088_ACCEL_NUM_CHECKED_REGISTERS; i++) {
			if (read_reg(_checked_registers[i]) != _checked_values[i]) {
				write_reg(_checked_registers[i], _checked_values[i]);
				all_ok = false;
			}
		}

		if (all_ok) {
			break;
		}
	}

	return OK;
}

int
BMI088_accel::probe()
{
	/* look for device ID */
	/* first access will enable the SPI port */
	_whoami = read_reg(BMI088_ACC_CHIP_ID);

	/* now actually read the device ID register */
	_whoami = read_reg(BMI088_ACC_CHIP_ID);

	// verify product revision
	switch (_whoami) {
	case BMI088_ACC_WHO_AM_I:
		memset(_checked_values, 0, sizeof(_checked_values));
		memset(_checked_bad, 0, sizeof(_checked_bad));
		_checked_values[0] = _whoami;
		_checked_bad[0] = _whoami;
		return OK;
	}
	DEVICE_DEBUG("unexpected whoami 0x%02x", _whoami);
	return -EIO;
}

ssize_t
BMI088_accel::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(sensor_accel_s);

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	if (_call_interval == 0) {
		_accel_reports->flush();
		measure();
	}

	/* if no data, error (we could block here) */
	if (_accel_reports->empty()) {
		return -EAGAIN;
	}

	/* copy reports out of our buffer to the caller */
	sensor_accel_s *arp = reinterpret_cast<sensor_accel_s *>(buffer);
	int transferred = 0;

	while (count--) {
		if (!_accel_reports->get(arp)) {
			break;
		}

		transferred++;
		arp++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(sensor_accel_s));
}

int
BMI088_accel::self_test()
{
	if (perf_event_count(_sample_perf) == 0) {
		measure();
	}

	/* return 0 on success, 1 else */
	return (perf_event_count(_sample_perf) > 0) ? 0 : 1;
}

/*
  deliberately trigger an error in the sensor to trigger recovery
 */
void
BMI088_accel::test_error()
{
	write_reg(BMI088_ACC_SOFTRESET, BMI088_SOFT_RESET);
	::printf("error triggered\n");
	print_registers();
}

int
BMI088_accel::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCRESET:
		return reset();

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default polling rate */
			case SENSOR_POLLRATE_DEFAULT:
				// Polling at the highest frequency. We may get duplicate values on the sensors
				return ioctl(filp, SENSORIOCSPOLLRATE, BMI088_ACCEL_DEFAULT_RATE);

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_call_interval == 0);

					/* convert hz to hrt interval via microseconds */
					unsigned ticks = 1000000 / arg;

					/* check against maximum rate */
					if (ticks < 1000) {
						return -EINVAL;
					}

					// adjust filters
					float cutoff_freq_hz = _accel_filter_x.get_cutoff_freq();
					float sample_rate = 1.0e6f / ticks;

					_accel_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
					_accel_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
					_accel_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz);

					/* update interval for next measurement */
					_call_interval = ticks;

					/*
					  set call interval faster than the sample time. We
					  then detect when we have duplicate samples and reject
					  them. This prevents aliasing due to a beat between the
					  stm32 clock and the bmi088 clock
					 */
					_call.period = _call_interval - BMI088_TIMER_REDUCTION;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case ACCELIOCSSCALE: {
			/* copy scale, but only if off by a few percent */
			struct accel_calibration_s *s = (struct accel_calibration_s *) arg;
			float sum = s->x_scale + s->y_scale + s->z_scale;

			if (sum > 2.0f && sum < 4.0f) {
				memcpy(&_accel_scale, s, sizeof(_accel_scale));
				return OK;

			} else {
				return -EINVAL;
			}
		}

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

void
BMI088_accel::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_checked_reg(reg, val);
}

void
BMI088_accel::write_checked_reg(unsigned reg, uint8_t value)
{
	write_reg(reg, value);

	for (uint8_t i = 0; i < BMI088_ACCEL_NUM_CHECKED_REGISTERS; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
			_checked_bad[i] = value;
		}
	}
}


/* TODO this function needs to be rewritten */
int
BMI088_accel::set_accel_range(unsigned max_g)
{
	uint8_t setbits = 0;
	uint8_t clearbits = BMI088_ACCEL_RANGE_3_G | BMI088_ACCEL_RANGE_12_G;
	float lsb_per_g;
	float max_accel_g;

	if (max_g == 0) {
		max_g = 16;
	}

	if (max_g <= 2) {
		max_accel_g = 2;
		setbits |= BMI088_ACCEL_RANGE_3_G;
		lsb_per_g = 1024;

	} else if (max_g <= 4) {
		max_accel_g = 4;
		setbits |= BMI088_ACCEL_RANGE_6_G;
		lsb_per_g = 512;

	} else if (max_g <= 8) {
		max_accel_g = 8;
		setbits |= BMI088_ACCEL_RANGE_12_G;
		lsb_per_g = 256;

	} else if (max_g <= 16) {
		max_accel_g = 16;
		setbits |= BMI088_ACCEL_RANGE_24_G;
		lsb_per_g = 128;

	} else {
		return -EINVAL;
	}

	_accel_range_scale = (CONSTANTS_ONE_G / lsb_per_g);
	_accel_range_m_s2 = max_accel_g * CONSTANTS_ONE_G;

	modify_reg(BMI088_ACC_RANGE, clearbits, setbits);

	return OK;
}

void
BMI088_accel::start()
{
	/* make sure we are stopped first */
	stop();

	/* discard any stale data in the buffers */
	_accel_reports->flush();

	/* start polling at the specified rate */
	hrt_call_every(&_call,
		       1000,
		       _call_interval - BMI088_TIMER_REDUCTION,
		       (hrt_callout)&BMI088_accel::measure_trampoline, this);
	reset();
}

void
BMI088_accel::stop()
{
	hrt_cancel(&_call);
}

void
BMI088_accel::measure_trampoline(void *arg)
{
	BMI088_accel *dev = reinterpret_cast<BMI088_accel *>(arg);

	/* make another measurement */
	dev->measure();
}

void
BMI088_accel::check_registers(void)
{
	uint8_t v;

	if ((v = read_reg(_checked_registers[_checked_next])) !=
	    _checked_values[_checked_next]) {
		_checked_bad[_checked_next] = v;

		/*
		  if we get the wrong value then we know the SPI bus
		  or sensor is very sick. We set _register_wait to 20
		  and wait until we have seen 20 good values in a row
		  before we consider the sensor to be OK again.
		 */
		perf_count(_bad_registers);

		/*
		  try to fix the bad register value. We only try to
		  fix one per loop to prevent a bad sensor hogging the
		  bus.
		 */
		if (_register_wait == 0 || _checked_next == 0) {
			// if the product_id is wrong then reset the
			// sensor completely
			write_reg(BMI088_ACC_SOFTRESET, BMI088_SOFT_RESET);
			_reset_wait = hrt_absolute_time() + 10000;
			_checked_next = 0;

		} else {
			write_reg(_checked_registers[_checked_next], _checked_values[_checked_next]);
			// waiting 3ms between register writes seems
			// to raise the chance of the sensor
			// recovering considerably
			_reset_wait = hrt_absolute_time() + 3000;
		}

		_register_wait = 20;
	}

	_checked_next = (_checked_next + 1) % BMI088_ACCEL_NUM_CHECKED_REGISTERS;
}

void
BMI088_accel::measure()
{
	perf_count(_measure_interval);

	uint8_t index = 0, accel_data[8];
	uint16_t lsb, msb, msblsb;
	uint8_t status_x, status_y, status_z;

	if (hrt_absolute_time() < _reset_wait) {
		// we're waiting for a reset to complete
		return;
	}

	struct Report {
		int16_t     accel_x;
		int16_t     accel_y;
		int16_t     accel_z;
		int16_t     temp;
	} report;

	/* start measuring */
	perf_begin(_sample_perf);

	/*
	 * Fetch the full set of measurements from the BMI088 in one pass.
	 */
	accel_data[index] = BMI088_ACC_X_L | DIR_READ;

	if (OK != transfer(accel_data, accel_data, sizeof(accel_data))) {
		return;
	}

	check_registers();

	/* Extracting accel data from the read data */
	index = 1;
	lsb = (uint16_t)accel_data[index++];
	status_x = (lsb & BMI088_NEW_DATA_MASK);
	msb = (uint16_t)accel_data[index++];
	msblsb = (msb << 8) | lsb;
	report.accel_x = ((int16_t)msblsb >> 4); /* Data in X axis */

	lsb = (uint16_t)accel_data[index++];
	status_y = (lsb & BMI088_NEW_DATA_MASK);
	msb = (uint16_t)accel_data[index++];
	msblsb = (msb << 8) | lsb;
	report.accel_y = ((int16_t)msblsb >> 4); /* Data in Y axis */

	lsb = (uint16_t)accel_data[index++];
	status_z = (lsb & BMI088_NEW_DATA_MASK);
	msb = (uint16_t)accel_data[index++];
	msblsb = (msb << 8) | lsb;
	report.accel_z = ((int16_t)msblsb >> 4); /* Data in Z axis */

	// Byte
	report.temp = accel_data[index++] << 8;
	report.temp += accel_data[index] / 32;

	// Checking the status of new data
	if ((!status_x) || (!status_y) || (!status_z)) {
		perf_end(_sample_perf);
		perf_count(_duplicates);
		_got_duplicate = true;
		return;
	}


	_got_duplicate = false;

	if (report.accel_x == 0 &&
	    report.accel_y == 0 &&
	    report.accel_z == 0 &&
	    report.temp == 0) {
		// all zero data - probably a SPI bus error
		perf_count(_bad_transfers);
		perf_end(_sample_perf);
		// note that we don't call reset() here as a reset()
		// costs 20ms with interrupts disabled. That means if
		// the bmi088 accel does go bad it would cause a FMU failure,
		// regardless of whether another sensor is available,
		return;
	}

	if (_register_wait != 0) {
		// we are waiting for some good transfers before using
		// the sensor again, but don't return any data yet
		_register_wait--;
		return;
	}

	/*
	 * Report buffers.
	 */
	sensor_accel_s arb;

	arb.timestamp = hrt_absolute_time();

	// report the error count as the sum of the number of bad
	// transfers and bad register reads. This allows the higher
	// level code to decide if it should use this sensor based on
	// whether it has had failures
	arb.error_count = perf_event_count(_bad_transfers) + perf_event_count(_bad_registers);

	/*
	 * 1) Scale raw value to SI units using scaling from datasheet.
	 * 2) Subtract static offset (in SI units)
	 * 3) Scale the statically calibrated values with a linear
	 *    dynamically obtained factor
	 *
	 * Note: the static sensor offset is the number the sensor outputs
	 *   at a nominally 'zero' input. Therefore the offset has to
	 *   be subtracted.
	 *
	 */

	arb.x_raw = report.accel_x;
	arb.y_raw = report.accel_y;
	arb.z_raw = report.accel_z;

	float xraw_f = report.accel_x;
	float yraw_f = report.accel_y;
	float zraw_f = report.accel_z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_in_new = ((xraw_f * _accel_range_scale) - _accel_scale.x_offset) * _accel_scale.x_scale;
	float y_in_new = ((yraw_f * _accel_range_scale) - _accel_scale.y_offset) * _accel_scale.y_scale;
	float z_in_new = ((zraw_f * _accel_range_scale) - _accel_scale.z_offset) * _accel_scale.z_scale;

	arb.x = _accel_filter_x.apply(x_in_new);
	arb.y = _accel_filter_y.apply(y_in_new);
	arb.z = _accel_filter_z.apply(z_in_new);

	matrix::Vector3f aval(x_in_new, y_in_new, z_in_new);
	matrix::Vector3f aval_integrated;

	bool accel_notify = _accel_int.put(arb.timestamp, aval, aval_integrated, arb.integral_dt);
	arb.x_integral = aval_integrated(0);
	arb.y_integral = aval_integrated(1);
	arb.z_integral = aval_integrated(2);

	arb.scaling = _accel_range_scale;

	printf("measure: calculating temperature()\n");
	/*
	 * Temperature is reported as eleven-bit 2’s complement sensor temperature value
	 * with 0.125 °C/LSB sensitivity and an offset of 23.0 °C
	 */
	_last_temperature = (report.temp * 0.125f) + 23.0f;
	arb.temperature = _last_temperature;

	arb.device_id = _device_id.devid;

	_accel_reports->force(&arb);

	/* notify anyone waiting for data */
	if (accel_notify) {
		poll_notify(POLLIN);
	}

	if (accel_notify && !(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(sensor_accel), _accel_topic, &arb);
	}

	/* stop measuring */
	perf_end(_sample_perf);
}

void
BMI088_accel::print_info()
{
	PX4_INFO("Accel");

	perf_print_counter(_sample_perf);
	perf_print_counter(_measure_interval);
	perf_print_counter(_bad_transfers);
	perf_print_counter(_bad_registers);
	perf_print_counter(_duplicates);

	_accel_reports->print_info("accel queue");
	::printf("checked_next: %u\n", _checked_next);

	for (uint8_t i = 0; i < BMI088_ACCEL_NUM_CHECKED_REGISTERS; i++) {
		uint8_t v = read_reg(_checked_registers[i]);

		if (v != _checked_values[i]) {
			::printf("reg %02x:%02x should be %02x\n",
				 (unsigned)_checked_registers[i],
				 (unsigned)v,
				 (unsigned)_checked_values[i]);
		}

		if (v != _checked_bad[i]) {
			::printf("reg %02x:%02x was bad %02x\n",
				 (unsigned)_checked_registers[i],
				 (unsigned)v,
				 (unsigned)_checked_bad[i]);
		}
	}

	::printf("temperature: %.1f\n", (double)_last_temperature);
	printf("\n");
}

void
BMI088_accel::print_registers()
{
	uint8_t index = 0;
	printf("BMI088 accel registers\n");

	uint8_t reg = _checked_registers[index++];
	uint8_t v = read_reg(reg);
	printf("Accel Chip Id: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = read_reg(reg);
	printf("Accel Bw: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = read_reg(reg);
	printf("Accel Range: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	printf("\n");
}
