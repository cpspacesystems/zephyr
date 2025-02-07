/*
 * Copyright (c) 2023 Trackunit Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bmi323.h"
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

static int bosch_bmi323_i2c_read_words(const void *context, uint8_t offset, uint16_t *words,
				       uint16_t words_count)
{
	const struct i2c_dt_spec *i2c = (const struct i2c_dt_spec *)context;
	uint8_t address;
	uint8_t rxbuf[2 + (words_count * 2)];
	struct i2c_msg msg[3];
	int ret;

	address = offset & 0x7F;

	msg[0].buf = &address;
	msg[0].len = sizeof(address);
	msg[0].flags = I2C_MSG_WRITE;

	msg[1].buf = rxbuf;
	msg[1].len = 2 + (words_count * 2);
	msg[1].flags = I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP;

	ret = i2c_transfer_dt(i2c, msg, 2);
	
	// memcpy((uint8_t *)words, rxbuf + 2, words_count * 2);
	memcpy((uint8_t *)words, rxbuf + 2, words_count * 2);

	k_usleep(2);

	return ret;
}

static int bosch_bmi323_i2c_write_words(const void *context, uint8_t offset, uint16_t *words,
					uint16_t words_count)
{
	const struct i2c_dt_spec *i2c = (const struct i2c_dt_spec *)context;
	uint8_t address;
	struct i2c_msg msg[2];
	int ret;

	address = offset & 0x7F;

	msg[0].buf = &address;
	msg[0].len = 1;
	msg[0].flags = I2C_MSG_WRITE;

	msg[1].buf = (uint8_t *)words;
	msg[1].len = (words_count * 2);
	msg[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	ret = i2c_transfer_dt(i2c, msg, 2);

	k_usleep(2);

	return ret;
}

static int bosch_bmi323_i2c_init(const void *context)
{
	const struct i2c_dt_spec *i2c = (const struct i2c_dt_spec *)context;
	int ret;

	if (i2c_is_ready_dt(i2c) == false) {
		return -ENODEV;
	}

	uint16_t cmd[1] = {(uint16_t)0xDEAF};
	ret = bosch_bmi323_i2c_write_words(i2c, 0x7E, cmd, 1);

	if (ret < 0) {
		return ret;
	}

	k_usleep(1500);

	ret = bosch_bmi323_i2c_read_words(i2c, 0x00, cmd, 1);

	if ((cmd[0] & 0x00FF) != 0x43) {
		return -ENODEV;
	}

	if (ret < 0) {
		return ret;
	}

	k_usleep(1500);

	return 0;
}

const struct bosch_bmi323_bus_api bosch_bmi323_i2c_bus_api = {
	.read_words = bosch_bmi323_i2c_read_words,
	.write_words = bosch_bmi323_i2c_write_words,
	.init = bosch_bmi323_i2c_init};
