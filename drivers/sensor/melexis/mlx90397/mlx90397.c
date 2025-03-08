/*
 * Copyright (c) 2024 Florian Weber <Florian.Weber@live.de>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT melexis_mlx90397

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_data_types.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor/mlx90397.h>

#include "mlx90397.h"
#include "mlx90397_reg.h"

LOG_MODULE_REGISTER(MLX90397, CONFIG_SENSOR_LOG_LEVEL);

/*
 * conversion time of one single axis for different filter orders if osr is enabled the value has
 * to be doubled
 */
static const int32_t MLX90397_CONVERSION_TIME_US_AXIS[] = {111, 170,  270,  490,
							   910, 1770, 3470, 6890};

/* conversion time per axis dependent of filter order */

/*
 * DSP Time per measurement first index:temperature sensor count ( 0..1 ) second index:
 * magnetic axis count (0..3) in us
 */
static const int32_t MLX90397_DSP_TIME_US[2][4] = {{0, 27, 50, 73}, {20, 63, 86, 110}};

static void mlx90397_update_measurement_Time_us(struct mlx90397_data *data)
{
	int32_t en_x = FIELD_GET(MLX90397_CTRL1_X_EN, data->ctrl_reg_values.ctrl1);
	int32_t en_y = FIELD_GET(MLX90397_CTRL1_Y_EN, data->ctrl_reg_values.ctrl1);
	int32_t en_z = FIELD_GET(MLX90397_CTRL1_Z_EN, data->ctrl_reg_values.ctrl1);
	int32_t en_temp = FIELD_GET(MLX90397_CTRL4_T_EN, data->ctrl_reg_values.ctrl4);
	int32_t filter_hall_xy =
		FIELD_GET(MLX90397_CTRL3_DIG_FILT_HALL_XY, data->ctrl_reg_values.ctrl3);
	int32_t filter_hall_z =
		FIELD_GET(MLX90397_CTRL4_DIG_FILT_HALL_Z, data->ctrl_reg_values.ctrl4);
	int32_t filter_temp = FIELD_GET(MLX90397_CTRL3_DIG_FILT_TEMP, data->ctrl_reg_values.ctrl3);
	int32_t osr_temp = FIELD_GET(MLX90397_CTRL3_OSR_TEMP, data->ctrl_reg_values.ctrl3);
	int32_t osr_hall = FIELD_GET(MLX90397_CTRL3_OSR_HALL, data->ctrl_reg_values.ctrl3);

	int32_t conversion_time_us =
		(osr_hall + 1) * ((en_x + en_y) * MLX90397_CONVERSION_TIME_US_AXIS[filter_hall_xy] +
				  en_z * MLX90397_CONVERSION_TIME_US_AXIS[filter_hall_z]) +
		(osr_temp + 1) * en_temp * MLX90397_CONVERSION_TIME_US_AXIS[filter_temp];
	int32_t dsp_time_us = MLX90397_DSP_TIME_US[en_temp][en_x + en_y + en_z];

	/*
	 * adding 5% tolerance from datasheet
	 */
	data->measurement_time_us = (conversion_time_us + dsp_time_us) * 105 / 100;
}

static void mlx90397_convert_magn(int64_t scale, struct sensor_value *val, uint8_t sample_l,
				  uint8_t sample_h)
{
	int64_t conv_val = ((int16_t)((uint16_t)sample_l | (uint16_t)(sample_h << 8))) * scale;

	val->val1 = conv_val / 1000000;               /* G */
	val->val2 = conv_val - (val->val1 * 1000000); /* uG */
}

static void mlx90397_convert_temp(struct sensor_value *val, uint8_t sample_l, uint8_t sample_h)
{
	int64_t conv_val = sample_l | (sample_h << 8) * MLX90397_MICRO_CELSIUS_PER_BIT;

	val->val1 = conv_val / 1000000;                 /* C */
	val->val2 = (conv_val - (val->val1 * 1000000)); /* uC */
}

/*
 * The user has to take care about that the requested channel was fetched before. Else the data will
 * be random. Magnetic Flux Density is in Gauss, Temperature in Celsius
 */
static int mlx90397_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
	struct mlx90397_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_MAGN_X: {
		mlx90397_convert_magn(mlx90397_range_val[data->range_xy_val], val, data->sample.x_l, data->sample.x_h);
	} break;
	case SENSOR_CHAN_MAGN_Y: {
		mlx90397_convert_magn(mlx90397_range_val[data->range_xy_val], val, data->sample.y_l, data->sample.y_h);
	} break;
	case SENSOR_CHAN_MAGN_Z: {
		mlx90397_convert_magn(mlx90397_range_val[data->range_z_val], val, data->sample.z_l, data->sample.z_h);
	} break;
	case SENSOR_CHAN_AMBIENT_TEMP: {
		mlx90397_convert_temp(val, data->sample.temp_l, data->sample.temp_h);
	} break;
	case SENSOR_CHAN_MAGN_XYZ: {
		mlx90397_convert_magn(mlx90397_range_val[data->range_xy_val], val, data->sample.x_l, data->sample.x_h);
		mlx90397_convert_magn(mlx90397_range_val[data->range_xy_val], val + 1, data->sample.y_l,
				      data->sample.y_h);
		mlx90397_convert_magn(mlx90397_range_val[data->range_z_val], val + 2, data->sample.z_l,
				      data->sample.z_h);
	} break;
	case SENSOR_CHAN_ALL: {
		mlx90397_convert_magn(mlx90397_range_val[data->range_xy_val], val, data->sample.x_l, data->sample.x_h);
		mlx90397_convert_magn(mlx90397_range_val[data->range_xy_val], val + 1, data->sample.y_l,
				      data->sample.y_h);
		mlx90397_convert_magn(mlx90397_range_val[data->range_z_val], val + 2, data->sample.z_l,
				      data->sample.z_h);
		mlx90397_convert_temp(val + 3, data->sample.temp_l, data->sample.temp_h);
	} break;
	default: {
		LOG_DBG("Invalid channel %d", chan);
		return -ENOTSUP;
	}
	}
	return 0;
}

/**
 * update a register on the device and @param old_value as well
 */
static inline int mlx90397_update_register(const struct device *dev, const uint8_t reg_addr,
					   const uint8_t new_val, uint8_t *old_value)
{
	const struct mlx90397_config *cfg = dev->config;

	if (new_val != *old_value) {
		*old_value = new_val;
		return i2c_reg_write_byte_dt(&cfg->i2c, reg_addr, new_val);
	}
	return 0;
}

static inline int mlx90397_sync_config_val(const struct device *dev)
{
	struct mlx90397_data *data = dev->data;
	uint8_t updated_ctrl2;

	updated_ctrl2 = MLX90397_FIELD_MOD(MLX90397_CTRL2_RANGE_XY, data->range_xy_val,
					   data->ctrl_reg_values.ctrl2);
	updated_ctrl2 =
		MLX90397_FIELD_MOD(MLX90397_CTRL2_RANGE_Z, data->range_z_val, updated_ctrl2);

	return mlx90397_update_register(dev, MLX90397_REG_CTRL2, updated_ctrl2,
					&data->ctrl_reg_values.ctrl2);
	;
}

static inline int mlx90397_fs_set(const struct device *dev, const struct sensor_value *val)
{
	struct mlx90397_data *data = dev->data;

	/* if the requested range is greater the driver switches from HIGH_SENSITIVITY to
	 * HIGH_RANGE
	 */
	if (val->val1 > MLX90397_ATTR_FS_LOW_G) {
		data->range_xy_val = MLX90397_CTRL2_RANGE_SEL_XY_50;
		data->range_z_val = MLX90397_CTRL2_RANGE_SEL_Z_50;
	} else {
		data->range_xy_val = MLX90397_CTRL2_RANGE_SEL_XY_25;
		data->range_z_val = MLX90397_CTRL2_RANGE_SEL_Z_25;
	}

	return mlx90397_sync_config_val(dev);
}

static inline int mlx90397_fs_get(const struct device *dev, struct sensor_value *val)
{
	struct mlx90397_data *data = dev->data;

	val->val2 = 0;
	if (data->range_xy_val == MLX90397_CTRL2_RANGE_SEL_XY_25 &&
	    data->range_z_val == MLX90397_CTRL2_RANGE_SEL_Z_25) {
		val->val1 = MLX90397_ATTR_FS_LOW_G;
	} else {
		val->val1 = MLX90397_ATTR_FS_HIGH_G;
	}

	return 0;
}

/*
 * helper function to get/set attributes if set is 0 it will be a get, otherwise is interpreted
 * as set
 */
static int mlx90397_attr_helper(const struct device *dev, enum sensor_channel chan,
				unsigned int attr, struct sensor_value *val, uint8_t set)
{
	struct mlx90397_data *data = dev->data;

	if (!data->initialized) {
		return -ENODEV;
	}

	switch (attr) {
	case SENSOR_ATTR_FULL_SCALE: {
		if (chan != SENSOR_CHAN_MAGN_XYZ) {
			return -ENOTSUP;
		}

		if (set) {
			return mlx90397_fs_set(dev, val);
		} else {
			return mlx90397_fs_get(dev, val);
		}
	} break;

	case MLX90397_SENSOR_ATTR_MAGN_FILTER_XY: {
		if (set) {
			if (val->val1 > 7 || val->val1 < 0) {
				return -EINVAL;
			}

			return mlx90397_update_register(
				dev, MLX90397_REG_CTRL3,
				MLX90397_FIELD_MOD(MLX90397_CTRL3_DIG_FILT_HALL_XY, val->val1,
						   data->ctrl_reg_values.ctrl3),
				&data->ctrl_reg_values.ctrl3);
		} else {
			val->val1 = FIELD_GET(MLX90397_CTRL3_DIG_FILT_HALL_XY,
					      data->ctrl_reg_values.ctrl3);
			val->val2 = 0;
		}
	} break;
	case MLX90397_SENSOR_ATTR_MAGN_FILTER_Z: {
		if (set) {
			if (val->val1 > 7 || val->val1 < 0) {

				return -EINVAL;
			}

			return mlx90397_update_register(
				dev, MLX90397_REG_CTRL4,
				MLX90397_FIELD_MOD(MLX90397_CTRL4_DIG_FILT_HALL_Z, val->val1,
						   data->ctrl_reg_values.ctrl4),
				&data->ctrl_reg_values.ctrl4);
		} else {
			val->val1 = FIELD_GET(MLX90397_CTRL4_DIG_FILT_HALL_Z,
					      data->ctrl_reg_values.ctrl4);
			val->val2 = 0;
		}
	} break;
	case MLX90397_SENSOR_ATTR_MAGN_OSR: {
		if (set) {
			if (val->val1 > 1 || val->val1 < 0) {

				return -EINVAL;
			}

			return mlx90397_update_register(
				dev, MLX90397_REG_CTRL3,
				MLX90397_FIELD_MOD(MLX90397_CTRL3_OSR_HALL, val->val1,
						   data->ctrl_reg_values.ctrl3),
				&data->ctrl_reg_values.ctrl3);
		} else {
			val->val1 = FIELD_GET(MLX90397_CTRL3_OSR_HALL, data->ctrl_reg_values.ctrl3);
			val->val2 = 0;
		}
	} break;
	case MLX90397_SENSOR_ATTR_TEMP_FILTER: {
		if (set) {
			if (val->val1 > 7 || val->val1 < 0) {
				return -EINVAL;
			}
			return mlx90397_update_register(
				dev, MLX90397_REG_CTRL3,
				MLX90397_FIELD_MOD(MLX90397_CTRL3_DIG_FILT_TEMP, val->val1,
						   data->ctrl_reg_values.ctrl3),
				&data->ctrl_reg_values.ctrl3);
		} else {
			val->val1 = FIELD_GET(MLX90397_CTRL3_DIG_FILT_TEMP,
					      data->ctrl_reg_values.ctrl3);
			val->val2 = 0;
		}
	} break;
	case MLX90397_SENSOR_ATTR_TEMP_OSR: {
		if (set) {
			if (val->val1 > 1 || val->val1 < 0) {
				return -EINVAL;
			}
			return mlx90397_update_register(
				dev, MLX90397_REG_CTRL3,
				MLX90397_FIELD_MOD(MLX90397_CTRL3_OSR_TEMP, val->val1,
						   data->ctrl_reg_values.ctrl3),
				&data->ctrl_reg_values.ctrl3);
		} else {
			val->val1 = FIELD_GET(MLX90397_CTRL3_OSR_TEMP, data->ctrl_reg_values.ctrl3);
			val->val2 = 0;
		}
	} break;
	default: {
		return -ENOTSUP;
	}
	}

	return 0;
}
static int mlx90397_attr_get(const struct device *dev, enum sensor_channel chan,
			     enum sensor_attribute attr, struct sensor_value *val)
{
	return mlx90397_attr_helper(dev, chan, (unsigned int)attr, val, 0);
}

static int mlx90397_attr_set(const struct device *dev, enum sensor_channel chan,
			     enum sensor_attribute attr, const struct sensor_value *val)
{
	/*
	 * must be a copy because val is const
	 */
	struct sensor_value val_copy = *val;
	int rc;

	rc = mlx90397_attr_helper(dev, chan, (unsigned int)attr, &val_copy, 1);
	if (rc == 0) {
		mlx90397_update_measurement_Time_us(dev->data);
	}

	return rc;
}

static inline int mlx90397_check_who_am_i(const struct i2c_dt_spec *i2c)
{
	uint8_t buffer[2];
	int rc;

	rc = i2c_burst_read_dt(i2c, MLX90397_REG_CID, buffer, ARRAY_SIZE(buffer));
	if (rc != 0) {
		LOG_ERR("Failed to read who-am-i register (rc=%d)", rc);
		return -EIO;
	}

	if (buffer[0] != MLX90397_CID || buffer[1] != MLX90397_DID) {
		LOG_ERR("Wrong who-am-i value");
		return -EINVAL;
	}

	return 0;
}

static int mlx90397_write_read_dt(const struct i2c_dt_spec *i2c, uint8_t start_addr,
				  uint8_t *buffer_write, uint8_t *buffer_read, size_t cnt)
{
	int rc;

	rc = i2c_burst_write_dt(i2c, start_addr, buffer_write, cnt);
	if (rc != 0) {
		LOG_ERR("Failed to write %d bytes to register %d (rc=%d)", cnt, start_addr, rc);
		return -EIO;
	}
	rc = i2c_burst_read_dt(i2c, start_addr, buffer_read, cnt);
	if (rc != 0) {
		LOG_ERR("Failed to read %d bytes from register %d (rc=%d)", cnt, start_addr, rc);
		return -EIO;
	}

	return 0;
}

int mlx90397_sample_fetch_internal(const struct device *dev, enum sensor_channel chan)
{
	const struct mlx90397_config *cfg = dev->config;
	struct mlx90397_data *data = dev->data;
	int rc;

	if (!data->initialized) {
		return -ENODEV;
	}

	rc = i2c_burst_read_dt(&cfg->i2c, MLX90397_REG_STAT1, (uint8_t *)&data->sample,
			       MLX90397_REG_TH + 1);
	if (rc != 0) {
		LOG_ERR("Failed to read bytes");
		return rc;
	}

	if (FIELD_GET(MLX90397_STAT1_DRDY, data->sample.stat1) != 1) {
		LOG_ERR("Data was not ready during fetch. In continues mode consider to "
			"adjust "
			"sample frequency");
		return -EIO;
	}
	return 0;
}

int mlx90397_trigger_measurement_internal(const struct device *dev, enum sensor_channel chan)
{
	const struct mlx90397_config *cfg = dev->config;
	struct mlx90397_data *data = dev->data;
	int rc;

	if (!data->initialized) {
		return -ENODEV;
	}

	/*
	 * set single measurement mode as default if not already done
	 */
	if (FIELD_GET(MLX90397_CTRL1_MODE, data->ctrl_reg_values.ctrl1) !=
	    MLX90397_CTRL1_MODE_SINGLE) {
		data->ctrl_reg_values.ctrl1 =
			MLX90397_FIELD_MOD(MLX90397_CTRL1_MODE, MLX90397_CTRL1_MODE_SINGLE,
					   data->ctrl_reg_values.ctrl1);
	}

	/*
	 * change channel bits and update ctrl4 and the measurement time
	 * if the channel is different than during the last measurement
	 */
	if (chan != data->channel) {
		switch (chan) {
		case SENSOR_CHAN_MAGN_X: {
			WRITE_BIT(data->ctrl_reg_values.ctrl1, MLX90397_CTRL1_X_EN_BIT, 1);
			WRITE_BIT(data->ctrl_reg_values.ctrl1, MLX90397_CTRL1_Y_EN_BIT, 0);
			WRITE_BIT(data->ctrl_reg_values.ctrl1, MLX90397_CTRL1_Z_EN_BIT, 0);
			WRITE_BIT(data->ctrl_reg_values.ctrl4, MLX90397_CTRL4_T_EN_BIT, 0);
		} break;
		case SENSOR_CHAN_MAGN_Y: {
			WRITE_BIT(data->ctrl_reg_values.ctrl1, MLX90397_CTRL1_X_EN_BIT, 0);
			WRITE_BIT(data->ctrl_reg_values.ctrl1, MLX90397_CTRL1_Y_EN_BIT, 1);
			WRITE_BIT(data->ctrl_reg_values.ctrl1, MLX90397_CTRL1_Z_EN_BIT, 0);
			WRITE_BIT(data->ctrl_reg_values.ctrl4, MLX90397_CTRL4_T_EN_BIT, 0);
		} break;
		case SENSOR_CHAN_MAGN_Z: {
			WRITE_BIT(data->ctrl_reg_values.ctrl1, MLX90397_CTRL1_X_EN_BIT, 0);
			WRITE_BIT(data->ctrl_reg_values.ctrl1, MLX90397_CTRL1_Y_EN_BIT, 0);
			WRITE_BIT(data->ctrl_reg_values.ctrl1, MLX90397_CTRL1_Z_EN_BIT, 1);
			WRITE_BIT(data->ctrl_reg_values.ctrl4, MLX90397_CTRL4_T_EN_BIT, 0);
		} break;
		case SENSOR_CHAN_MAGN_XYZ: {
			WRITE_BIT(data->ctrl_reg_values.ctrl1, MLX90397_CTRL1_X_EN_BIT, 1);
			WRITE_BIT(data->ctrl_reg_values.ctrl1, MLX90397_CTRL1_Y_EN_BIT, 1);
			WRITE_BIT(data->ctrl_reg_values.ctrl1, MLX90397_CTRL1_Z_EN_BIT, 1);
			WRITE_BIT(data->ctrl_reg_values.ctrl4, MLX90397_CTRL4_T_EN_BIT, 0);
		} break;
		case SENSOR_CHAN_AMBIENT_TEMP: {
			WRITE_BIT(data->ctrl_reg_values.ctrl1, MLX90397_CTRL1_X_EN_BIT, 0);
			WRITE_BIT(data->ctrl_reg_values.ctrl1, MLX90397_CTRL1_Y_EN_BIT, 0);
			WRITE_BIT(data->ctrl_reg_values.ctrl1, MLX90397_CTRL1_Z_EN_BIT, 0);
			WRITE_BIT(data->ctrl_reg_values.ctrl4, MLX90397_CTRL4_T_EN_BIT, 1);
		} break;
		case SENSOR_CHAN_ALL: {
			WRITE_BIT(data->ctrl_reg_values.ctrl1, MLX90397_CTRL1_X_EN_BIT, 1);
			WRITE_BIT(data->ctrl_reg_values.ctrl1, MLX90397_CTRL1_Y_EN_BIT, 1);
			WRITE_BIT(data->ctrl_reg_values.ctrl1, MLX90397_CTRL1_Z_EN_BIT, 1);
			WRITE_BIT(data->ctrl_reg_values.ctrl4, MLX90397_CTRL4_T_EN_BIT, 1);
		} break;
		default: {
			return -ENOTSUP;
		}
		}
		rc = i2c_reg_write_byte_dt(&cfg->i2c, MLX90397_REG_CTRL4,
					   data->ctrl_reg_values.ctrl4);
		if (rc != 0) {
			LOG_ERR("Failed to write ctrl4");
			return rc;
		}

		data->channel = chan;
		mlx90397_update_measurement_Time_us(data);
	}

	rc = i2c_reg_write_byte_dt(&cfg->i2c, MLX90397_REG_CTRL1, data->ctrl_reg_values.ctrl1);
	if (rc != 0) {
		LOG_ERR("Failed to write ctrl1");
	}

	return rc;
}

static int mlx90397_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	int rc;
	struct mlx90397_data *data = dev->data;

	rc = mlx90397_trigger_measurement_internal(dev, chan);
	if (rc != 0) {
		return rc;
	}

	k_usleep(data->measurement_time_us);
	rc = mlx90397_sample_fetch_internal(dev, chan);

	return rc;
}

static int mlx90397_init(const struct device *dev)
{
	const struct mlx90397_config *cfg = dev->config;
	struct mlx90397_data *data = dev->data;
	int rc;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus device not ready");
		return -ENODEV;
	}

	/*
	 * Soft reset the chip
	 */
	rc = i2c_reg_write_byte_dt(&cfg->i2c, MLX90397_REG_RST, MLX90397_RST);
	if (rc != 0) {
		LOG_ERR("Failed to soft reset");
		return -EIO;
	}
	k_usleep(MLX90397_STARTUP_TIME_US);

	/*
	 * check chip ID
	 */
	rc = mlx90397_check_who_am_i(&cfg->i2c);
	if (rc != 0) {
		return rc;
	}

	/*
	 * set all to default and read the settings back
	 */
	rc = mlx90397_write_read_dt(&cfg->i2c, MLX90397_REG_CTRL1,
				    (uint8_t *)&data->ctrl_reg_values.ctrl1,
				    (uint8_t *)&data->ctrl_reg_values.ctrl1, 2);
	if (rc != 0) {
		return rc;
	}

	rc = mlx90397_write_read_dt(&cfg->i2c, MLX90397_REG_CTRL3,
				    (uint8_t *)&data->ctrl_reg_values.ctrl3,
				    (uint8_t *)&data->ctrl_reg_values.ctrl3, 2);
	if (rc != 0) {
		return rc;
	}

	mlx90397_update_measurement_Time_us(data);

#ifdef CONFIG_SENSOR_ASYNC_API
	data->dev = dev;
	/*
	 * init work for fetching after measurement has completed
	 */
	k_work_init_delayable(&data->async_fetch_work, mlx90397_async_fetch);
#endif
	data->initialized = true;
	return 0;
}

static DEVICE_API(sensor, mlx90397_driver_api) = {
	.sample_fetch = mlx90397_sample_fetch,
	.channel_get = mlx90397_channel_get,
	.attr_get = mlx90397_attr_get,
	.attr_set = mlx90397_attr_set,
#ifdef CONFIG_SENSOR_ASYNC_API
	.submit = mlx90397_submit,
	.get_decoder = mlx90397_get_decoder,
#endif
};

#define MLX90397_DEFINE(inst)                                                                      \
	static struct mlx90397_data mlx90397_data_##inst = {                                       \
		.sample = {.x_l = 0,                                                               \
			   .x_h = 0,                                                               \
			   .y_l = 0,                                                               \
			   .y_h = 0,                                                               \
			   .z_l = 0,                                                               \
			   .z_h = 0,                                                               \
			   .temp_l = 0,                                                            \
			   .temp_h = 0},                                                           \
		.channel = SENSOR_CHAN_MAGN_XYZ,                                                   \
		.range_xy_val = FIELD_GET(MLX90397_CTRL2_RANGE_XY, MLX90397_CTRL2_DEFAULT),        \
		.range_z_val = FIELD_GET(MLX90397_CTRL2_RANGE_Z, MLX90397_CTRL2_DEFAULT),          \
		.measurement_time_us = 0,                                                          \
		.ctrl_reg_values = {.ctrl1 = MLX90397_CTRL1_DEFAULT,                               \
				    .ctrl2 = MLX90397_CTRL2_DEFAULT,                               \
				    .ctrl3 = MLX90397_CTRL3_DEFAULT,                               \
				    .ctrl4 = MLX90397_CTRL4_DEFAULT},                              \
		.initialized = false};                                                             \
	static const struct mlx90397_config mlx90397_config_##inst = {                             \
		.i2c = I2C_DT_SPEC_INST_GET(inst)};                                                \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, mlx90397_init, NULL, &mlx90397_data_##inst,             \
				     &mlx90397_config_##inst, POST_KERNEL,                         \
				     CONFIG_SENSOR_INIT_PRIORITY, &mlx90397_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MLX90397_DEFINE)
