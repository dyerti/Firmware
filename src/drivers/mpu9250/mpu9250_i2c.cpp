/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file MPU9250_I2C.cpp
 *
 * I2C interface for MPU9250
 */

/* XXX trim includes */
#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <unistd.h>

#include <arch/board/board.h>

#include <drivers/device/i2c.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_device.h>

#include "mpu9250.h"

#include "board_config.h"

#ifdef PX4_I2C_OBDEV_MPU9250

#define MPU9250_ADDRESS			PX4_I2C_OBDEV_MPU9250

device::Device *MPU9250_I2C_interface(int bus);

class MPU9250_I2C : public device::I2C
{
public:
	MPU9250_I2C(int bus);
	virtual ~MPU9250_I2C();

	virtual int	init();
	virtual int	read(unsigned address, void *data, unsigned count);
	virtual int	write(unsigned address, void *data, unsigned count);

	virtual int	ioctl(unsigned operation, unsigned &arg);

protected:
	virtual int	probe();

};

device::Device *
MPU9250_I2C_interface(int bus)
{
	return new MPU9250_I2C(bus);
}

MPU9250_I2C::MPU9250_I2C(int bus) :
	I2C("MPU9250_I2C", nullptr, bus, MPU9250_ADDRESS, 400000)
{
}

MPU9250_I2C::~MPU9250_I2C()
{
}

int
MPU9250_I2C::init()
{
	/* this will call probe() */
	return I2C::init();
}

int
MPU9250_I2C::ioctl(unsigned operation, unsigned &arg)
{
	int ret;

	switch (operation) {

	case MAGIOCGEXTERNAL:
// On PX4v1 the MAG can be on an internal I2C
// On everything else its always external
#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1
		if (_bus == PX4_I2C_BUS_EXPANSION) {
			return 1;

		} else {
			return 0;
		}

#else
		return 1;
#endif

	case DEVIOCGDEVICEID:
		return CDev::ioctl(nullptr, operation, arg);

	default:
		ret = -EINVAL;
	}

	return ret;
}

int
MPU9250_I2C::probe()
{
	uint8_t id;

	_retries = 10;

	if (read(ADDR_WHO_AM_I, &id, 1)) {
		DEVICE_DEBUG("read_reg fail");
		return -EIO;
	}

	_retries = 2;

	if (id != ID_WHO_AM_I) {
		DEVICE_DEBUG("ID byte mismatch (%02x)", id);
		return -EIO;
	}

	return OK;
}

int
MPU9250_I2C::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], count + 1, nullptr, 0);
}

int
MPU9250_I2C::read(unsigned address, void *data, unsigned count)
{
	uint8_t cmd = address;
	return transfer(&cmd, 1, (uint8_t *)data, count);
}

#endif /* PX4_I2C_OBDEV_MPU9250 */
