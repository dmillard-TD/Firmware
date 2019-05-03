/****************************************************************************
 *
 *   Copyright (C) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file bmp388.h
 *
 * Shared defines for the bmp388 driver.
 */
#pragma once

#define BMP388_ADDR_CAL		0x31	/* address of 12x 2 bytes calibration data */
#define BMP388_ADDR_DATA	0x04	/* address of 2x 3 bytes p-t data */

#define BMP388_ADDR_CMD		0x7E	/* command */
#define BMP388_ADDR_CONFIG	0x1F	/* configuration */
#define BMP388_ADDR_ODR		0x1D	/* output data rate select */
#define BMP388_ADDR_OSR		0x1C	/* oversampling select */
#define BMP388_ADDR_PWR_CTRL	0x1B	/* power control */
#define BMP388_ADDR_IF_CTRL	0x1A	/* interface control */
#define BMP388_ADDR_INT_CTRL	0x19	/* interrupt control */
#define BMP388_ADDR_FIFO_CFG	0x17	/* address of 2 bytes FIFO config */
#define BMP388_ADDR_STATUS	0x03	/* state */
#define BMP388_ADDR_ERR_REG	0x02	/* error register */
#define BMP388_ADDR_ID		0x00	/* id */

#define BMP388_VALUE_ID		0x50	/* chip id */
#define BMP388_VALUE_RESET	0xB6	/* soft reset */


#define BMP388_STATUS_MEASURING	(1<<3)	/* if in process of measure */
#define BMP388_STATUS_COPING	(1<<0)	/* if in process of data copy */

#define BMP388_CTRL_P1		(0x0<<0)		/* no oversample */
#define BMP388_CTRL_P2		(0x1<<0)
#define BMP388_CTRL_P4		(0x2<<0)
#define BMP388_CTRL_P8		(0x3<<0)
#define BMP388_CTRL_P16		(0x4<<0)
#define BMP388_CTRL_P32		(0x5<<0)

#define BMP388_CTRL_T1		(0x0<<3)		/* no oversample */
#define BMP388_CTRL_T2		(0x1<<3)
#define BMP388_CTRL_T3		(0x2<<3)
#define BMP388_CTRL_T8		(0x3<<3)
#define BMP388_CTRL_T16		(0x4<<3)
#define BMP388_CTRL_T32		(0x5<<3)

#define BMP388_CONFIG_F0	(0x0<<1)		/* no filter */
#define BMP388_CONFIG_F1	(0x1<<1)
#define BMP388_CONFIG_F3	(0x2<<1)
#define BMP388_CONFIG_F7	(0x3<<1)
#define BMP388_CONFIG_F15	(0x4<<1)
#define BMP388_CONFIG_F31	(0x5<<1)
#define BMP388_CONFIG_F63	(0x6<<1)
#define BMP388_CONFIG_F127	(0x7<<1)

#define BMP388_CTRL_MODE_P	(0x1<<0)
#define BMP388_CTRL_MODE_T	(0x1<<1)

#define BMP388_CTRL_MODE_SLEEP	(0x0<<4)
#define BMP388_CTRL_MODE_FORCE	(0x1<<4)		/* on demand, goes to sleep after */
#define BMP388_CTRL_MODE_NORMAL	(0x3<<4)

#define BMP388_MT_INIT		6400	/* max measure time of initial p + t in us */
#define BMP388_MT		2300	/* max measure time of p or t in us */

namespace bmp388
{

#pragma pack(push,1)
struct calibration_s {
	uint16_t t1;
	uint16_t t2;
	int8_t t3;

	int16_t p1;
	int16_t p2;
	int8_t p3;
	int8_t p4;
	uint16_t p5;
	uint16_t p6;
	int8_t p7;
	int8_t p8;
	int16_t p9;
	int8_t p10;
	int8_t p11;
}; //calibration data

struct data_s {
	uint8_t p_xlsb;
	uint8_t p_lsb;
	uint8_t p_msb;

	uint8_t t_xlsb;
	uint8_t t_lsb;
	uint8_t t_msb;
}; // data
#pragma pack(pop)

struct fcalibration_s {
	float t1;
	float t2;
	float t3;

	float p1;
	float p2;
	float p3;
	float p4;
	float p5;
	float p6;
	float p7;
	float p8;
	float p9;
	float p10;
	float p11;
};

class IBMP388
{
public:
	virtual ~IBMP388() = default;

	virtual bool is_external() = 0;
	virtual int init() = 0;

	// read reg value
	virtual uint8_t get_reg(uint8_t addr) = 0;

	// write reg value
	virtual int set_reg(uint8_t value, uint8_t addr) = 0;

	// bulk read of data into buffer, return same pointer
	virtual bmp388::data_s *get_data(uint8_t addr) = 0;

	// bulk read of calibration data into buffer, return same pointer
	virtual bmp388::calibration_s *get_calibration(uint8_t addr) = 0;

};

} /* namespace */


/* interface factories */
extern bmp388::IBMP388 *bmp388_spi_interface(uint8_t busnum, uint32_t device, bool external);
extern bmp388::IBMP388 *bmp388_i2c_interface(uint8_t busnum, uint32_t device, bool external);
typedef bmp388::IBMP388 *(*BMP388_constructor)(uint8_t, uint32_t, bool);
