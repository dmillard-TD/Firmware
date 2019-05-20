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

#pragma once

#include "BMI088.hpp"

#include <drivers/device/integrator.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/spi.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_hrt.h>
#include <lib/conversion/rotation.h>
#include <lib/perf/perf_counter.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <px4_config.h>
#include <systemlib/conversions.h>

#define BMI088_DEVICE_PATH_ACCEL	"/dev/bmi088_accel"
#define BMI088_DEVICE_PATH_ACCEL_EXT	"/dev/bmi088_accel_ext"

// BMI088 Accel registers
#define BMI088_ACC_CHIP_ID          0x00
#define BMI088_ACC_ERR_REG          0x02
#define BMI088_ACC_STATUS           0x03
#define BMI088_ACC_X_L              0x12
#define BMI088_ACC_X_H              0x13
#define BMI088_ACC_Y_L              0x14
#define BMI088_ACC_Y_H              0x15
#define BMI088_ACC_Z_L              0x16
#define BMI088_ACC_Z_H              0x17
#define BMI088_SENSORTIME_0         0x18
#define BMI088_SENSORTIME_1         0x19
#define BMI088_SENSORTIME_2         0x1A
#define BMI088_ACC_INT_STAT_1       0x1D
#define BMI088_ACC_TEMP_H           0x22
#define BMI088_ACC_TEMP_L           0x23
#define BMI088_ACC_CONF             0x40
#define BMI088_ACC_RANGE            0x41
#define BMI088_INT1_IO_CTRL         0x53
#define BMI088_INT2_IO_CTRL         0x54
#define BMI088_INT_MAP_DATA         0x58
#define BMI088_ACC_SELF_TEST        0x6D
#define BMI088_ACC_PWR_CONF         0x7C
#define BMI088_ACC_PWR_CTRL         0x7D
#define BMI088_ACC_SOFTRESET        0x7E

/*  Old defines
#define BMI088_ACC_INT_STATUS_0     0x09
#define BMI088_ACC_INT_STATUS_1     0x0A
#define BMI088_ACC_INT_STATUS_2     0x0B
#define BMI088_ACC_INT_STATUS_3     0x0C
#define BMI088_ACC_FIFO_STATUS      0x0E
#define BMI088_ACC_RANGE            0x0F
#define BMI088_ACC_BW               0x10
#define BMI088_ACC_PMU_LPW          0x11
#define BMI088_ACC_PMU_LOW_POWER    0x12
#define BMI088_ACC_DATA_CTRL        0x13
#define BMI088_ACC_SOFTRESET        0x14
#define BMI088_ACC_INT_EN_0         0x16
#define BMI088_ACC_INT_EN_1         0x17
#define BMI088_ACC_INT_EN_2         0x18
#define BMI088_ACC_INT_MAP_0        0x19
#define BMI088_ACC_INT_MAP_1        0x1A
#define BMI088_ACC_INT_MAP_2        0x1B
#define BMI088_ACC_INT_SRC          0x1E
#define BMI088_ACC_INT_OUT_CTRL     0x20
#define BMI088_ACC_INT_LATCH        0x21
#define BMI088_ACC_INT_LH_0         0x22
#define BMI088_ACC_INT_LH_1         0x23
#define BMI088_ACC_INT_LH_2         0x24
#define BMI088_ACC_INT_LH_3         0x25
#define BMI088_ACC_INT_LH_4         0x26
#define BMI088_ACC_INT_MOT_0        0x27
#define BMI088_ACC_INT_MOT_1        0x28
#define BMI088_ACC_INT_MOT_2        0x29
#define BMI088_ACC_INT_TAP_0        0x2A
#define BMI088_ACC_INT_TAP_1        0x2B
#define BMI088_ACC_INT_ORIE_0       0x2C
#define BMI088_ACC_INT_ORIE_1       0x2D
#define BMI088_ACC_INT_FLAT_0       0x2E
#define BMI088_ACC_INT_FLAT_1       0x2F
#define BMI088_ACC_FIFO_CONFIG_0    0x30
#define BMI088_ACC_SELF_TEST        0x32
#define BMI088_ACC_EEPROM_CTRL      0x33
#define BMI088_ACC_SERIAL_CTRL      0x34
#define BMI088_ACC_OFFSET_CTRL      0x36
#define BMI088_ACC_OFC_SETTING      0x37
#define BMI088_ACC_OFFSET_X         0x38
#define BMI088_ACC_OFFSET_Y         0x39
#define BMI088_ACC_OFFSET_Z         0x3A
#define BMI088_ACC_TRIM_GPO         0x3B
#define BMI088_ACC_TRIM_GP1         0x3C
#define BMI088_ACC_FIFO_CONFIG_1    0x3E
#define BMI088_ACC_FIFO_DATA        0x3F
*/

// BMI088 Accelerometer Chip-Id
#define BMI088_ACC_WHO_AM_I         0x1E

// Used to switch accelerometer on/off
#define BMI088_PWR_CTRL_ON          0x04
#define BMI088_PWR_CTRL_OFF         0x00

// Output data rate     0x40
#define BMI088_ODR_12p5             0x05
#define BMI088_ODR_25               0x06
#define BMI088_ODR_50               0x07
#define BMI088_ODR_100              0x08
#define BMI088_ODR_200              0x09
#define BMI088_ODR_400              0x0A
#define BMI088_ODR_800              0x0B
#define BMI088_ODR_1600             0x0C

// Define oversampling and filter bandwidth  0x40
#define BMI088_BWP_OSR4             (0x08<<4)
#define BMI088_BWP_OSR2             (0x09<<4)
#define BMI088_BWP_NORMAL           (0x0A<<4)

// BMI088 power modes    0x7C
#define BMI088_ACC_PWR_ACTIVE       0x00
#define BMI088_ACC_PWR_SUSPEND      0x03

//BMI088_ACC_RANGE        0x41
#define BMI088_ACCEL_RANGE_3_G      0x00
#define BMI088_ACCEL_RANGE_6_G      0x01
#define BMI088_ACCEL_RANGE_12_G     0x02
#define BMI088_ACCEL_RANGE_24_G     0x03

//BMI088_ACC_INT_EN_1      0x17
#define BMI088_ACC_DRDY_INT_EN      (1<<4)

//BMI088_ACC_INT_MAP_1         0x1A
#define BMI088_ACC_DRDY_INT1        (1<<0)

// Default and Max values
#define BMI088_ACCEL_DEFAULT_RANGE_G	0x02 
#define BMI088_ACCEL_DEFAULT_RATE       1000
#define BMI088_ACCEL_MAX_RATE           1000
#define BMI088_ACCEL_MAX_PUBLISH_RATE   280

#define BMI088_ACCEL_DEFAULT_DRIVER_FILTER_FREQ 50

/* Mask definitions for ACCD_X_LSB, ACCD_Y_LSB and ACCD_Z_LSB Register */
#define BMI088_NEW_DATA_MASK                 0x01

class BMI088_accel : public BMI088
{
public:
	BMI088_accel(int bus, const char *path_accel, uint32_t device, enum Rotation rotation);
	virtual ~BMI088_accel();

	virtual int     init();

	virtual ssize_t     read(struct file *filp, char *buffer, size_t buflen);
	virtual int     ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void            print_info();

	void            print_registers();

	// deliberately cause a sensor error
	void            test_error();

protected:

	virtual int     probe();

private:

	perf_counter_t      _sample_perf;
	perf_counter_t      _measure_interval;
	perf_counter_t      _bad_transfers;
	perf_counter_t      _bad_registers;
	perf_counter_t      _duplicates;

	ringbuffer::RingBuffer  *_accel_reports;

	struct accel_calibration_s  _accel_scale;
	float           _accel_range_scale;
	float           _accel_range_m_s2;

	orb_advert_t        _accel_topic;
	int         _accel_orb_class_instance;
	int         _accel_class_instance;

	math::LowPassFilter2p   _accel_filter_x;
	math::LowPassFilter2p   _accel_filter_y;
	math::LowPassFilter2p   _accel_filter_z;

	Integrator      _accel_int;

	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset
#define BMI088_ACCEL_NUM_CHECKED_REGISTERS 3
	static const uint8_t    _checked_registers[BMI088_ACCEL_NUM_CHECKED_REGISTERS];
	uint8_t         _checked_values[BMI088_ACCEL_NUM_CHECKED_REGISTERS];
	uint8_t         _checked_bad[BMI088_ACCEL_NUM_CHECKED_REGISTERS];

	// last temperature reading for print_info()
	float           _last_temperature;

	bool            _got_duplicate;

	/**
	 * Start automatic measurement.
	 */
	void            start();

	/**
	 * Stop automatic measurement.
	 */
	void            stop();

	/**
	 * Reset chip.
	 *
	 * Resets the chip and measurements ranges, but not scale and offset.
	 */
	int         reset();

	/**
	 * Static trampoline from the hrt_call context; because we don't have a
	 * generic hrt wrapper yet.
	 *
	 * Called by the HRT in interrupt context at the specified rate if
	 * automatic polling is enabled.
	 *
	 * @param arg       Instance pointer for the driver that is polling.
	 */
	static void     measure_trampoline(void *arg);

	/**
	 * Fetch measurements from the sensor and update the report buffers.
	 */
	void            measure();

	/**
	 * Modify a register in the BMI088_accel
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg       The register to modify.
	 * @param clearbits Bits in the register to clear.
	 * @param setbits   Bits in the register to set.
	 */
	void            modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Write a register in the BMI088_accel, updating _checked_values
	 *
	 * @param reg       The register to write.
	 * @param value     The new value to write.
	 */
	void            write_checked_reg(unsigned reg, uint8_t value);

	/**
	 * Set the BMI088_accel measurement range.
	 *
	 * @param max_g     The maximum G value the range must support.
	 * @return      OK if the value can be supported, -EINVAL otherwise.
	 */
	int         set_accel_range(unsigned max_g);

	/**
	 * Measurement self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int             self_test();

	/*
	 * check that key registers still have the right value
	 */
	void check_registers(void);

	/* do not allow to copy this class due to pointer data members */
	BMI088_accel(const BMI088_accel &);
	BMI088_accel operator=(const BMI088_accel &);

};
