/*
 * Copyright (c) 2024 Florian Weber <Florian.Weber@live.de>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MLX90397_MLX90397_H_
#define ZEPHYR_DRIVERS_SENSOR_MLX90397_MLX90397_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/rtio/rtio.h>

#include "mlx90397_reg.h"

/*
 * Time it takes to start-up the device and switch to powerdown mode (after powercycle or soft
 * reset)
 */
#define MLX90397_STARTUP_TIME_US 400

/* Conversion values */
#define MLX90397_25_RANGE_MICRO_GAUSS_PER_BIT       INT64_C(7500)
#define MLX90397_50_RANGE_MICRO_GAUSS_PER_BIT       INT64_C(15000)
#define MLX90397_100_RANGE_MICRO_GAUSS_PER_BIT       INT64_C(30000)
#define MLX90397_200_RANGE_MICRO_GAUSS_PER_BIT       INT64_C(60000)
#define MLX90397_MICRO_CELSIUS_PER_BIT                INT64_C(20000)
static const int64_t mlx90397_range_val[4] = {
	MLX90397_25_RANGE_MICRO_GAUSS_PER_BIT,
	MLX90397_50_RANGE_MICRO_GAUSS_PER_BIT,
	MLX90397_100_RANGE_MICRO_GAUSS_PER_BIT,
	MLX90397_200_RANGE_MICRO_GAUSS_PER_BIT
};

/* values for setting SENSOR_ATTR_FULL_SCALE */
#define MLX90397_ATTR_FS_HIGH_G INT32_C(500)
#define MLX90397_ATTR_FS_LOW_G  INT32_C(250)

struct mlx90397_data {
	struct __packed {
		uint8_t stat1;
		uint8_t x_l;
		uint8_t x_h;
		uint8_t y_l;
		uint8_t y_h;
		uint8_t z_l;
		uint8_t z_h;
		uint8_t stat2;
		uint8_t temp_l;
		uint8_t temp_h;
	} sample;
	enum sensor_channel channel;
	enum mlx90397_reg_range_xy_val range_xy_val;
	enum mlx90397_reg_range_z_val range_z_val;
	int32_t measurement_time_us;
	struct __packed {
		uint8_t ctrl1;
		uint8_t ctrl2;
		uint8_t ctrl3;
		uint8_t ctrl4;
	} ctrl_reg_values;
	bool initialized;
#ifdef CONFIG_SENSOR_ASYNC_API
	struct {
		struct rtio_iodev_sqe *iodev_sqe;
		uint64_t timestamp;
		enum mlx90397_reg_range_xy_val range_xy_val;
		enum mlx90397_reg_range_z_val range_z_val;
	} work_ctx;
	struct k_work_delayable async_fetch_work;
	const struct device *dev;
#endif
};

struct mlx90397_config {
	struct i2c_dt_spec i2c;
};

int mlx90397_sample_fetch_internal(const struct device *dev, enum sensor_channel chan);
int mlx90397_trigger_measurement_internal(const struct device *dev, enum sensor_channel chan);

/* RTIO types and defines */
#ifdef CONFIG_SENSOR_ASYNC_API

/* shift value to use. */
// #define MLX90397_SHIFT_MAGN_HIGH_SENSITIVITY (6)
#define MLX90397_SHIFT_MAGN_50_RANGE       (9)
#define MLX90397_SHIFT_TEMP                  (10)

void mlx90397_async_fetch(struct k_work *work);

struct mlx90397_decoder_header {
	uint64_t timestamp;
	enum mlx90397_reg_range_xy_val range_xy_val;
	enum mlx90397_reg_range_z_val range_z_val;
};

struct mlx90397_encoded_data {
	struct mlx90397_decoder_header header;
	int16_t readings[4];
};

int mlx90397_get_decoder(const struct device *dev, const struct sensor_decoder_api **decoder);
void mlx90397_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe);
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_MLX90397_MLX90397_H_ */
