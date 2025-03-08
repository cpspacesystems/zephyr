/*
 * Copyright (c) 2024 Florian Weber <Florian.Weber@live.de>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MLX90397_MLX90397_REG_H
#define ZEPHYR_DRIVERS_SENSOR_MLX90397_MLX90397_REG_H

#include <zephyr/sys/util_macro.h>
/* REGISTERS */
/* Status and measurent output */
#define MLX90397_REG_STAT1 0x00
#define MLX90397_REG_BXL   0x01
#define MLX90397_REG_BXH   0x02
#define MLX90397_REG_BYL   0x03
#define MLX90397_REG_BYH   0x04
#define MLX90397_REG_BZL   0x05
#define MLX90397_REG_BZH   0x06
#define MLX90397_REG_STAT2 0x07
#define MLX90397_REG_TL    0x08
#define MLX90397_REG_TH    0x09

/* Who Am I registers */
#define MLX90397_REG_CID 0x0A
#define MLX90397_REG_DID 0x0B

/* Control registers */
#define MLX90397_REG_CTRL1 0x0E
#define MLX90397_REG_CTRL2 0x0F
#define MLX90397_REG_CTRL3 0x14
#define MLX90397_REG_CTRL4 0x15

/* Reset register */
#define MLX90397_REG_RST 0x11

/* Wake On Change registers  */
#define MLX90397_REG_WOC_XL 0x58
#define MLX90397_REG_WOC_XH 0x59
#define MLX90397_REG_WOC_YL 0x5A
#define MLX90397_REG_WOC_YH 0x5B
#define MLX90397_REG_WOC_ZL 0x5C
#define MLX90397_REG_WOC_ZH 0x5D

/* VALUES */
/* STAT1 values RO */
#define MLX90397_STAT1_DRDY    BIT(0)
#define MLX90397_STAT1_DOR     BIT(3)
#define MLX90397_STAT1_RT      BIT(3)
#define MLX90397_STAT1_INT     BIT(4)
#define MLX90397_STAT1_DEFAULT (MLX90397_STAT1_RT)

/* STAT2 values RO */
#define MLX90397_STAT2_HOVF_X  BIT(0)
#define MLX90397_STAT2_HOVF_Y  BIT(1)
#define MLX90397_STAT2_HOVF_Z  BIT(2)
#define MLX90397_STAT2_DOR     BIT(3)
#define MLX90397_STAT2_DEFAULT 0

/* Who-I-Am register values RO */
#define MLX90397_CID 0x97
#define MLX90397_DID 0xaa

/* Write this value to reset Register soft resets the chip RW */
#define MLX90397_RST 0x06

/* CTRL1 values RW */
#define MLX90397_CTRL1_X_EN_BIT    4
#define MLX90397_CTRL1_Y_EN_BIT    5
#define MLX90397_CTRL1_Z_EN_BIT    6
#define MLX90397_CTRL1_MODE        GENMASK(3, 0)
#define MLX90397_CTRL1_MODE_SINGLE 1
#define MLX90397_CTRL1_X_EN        BIT(MLX90397_CTRL1_X_EN_BIT)
#define MLX90397_CTRL1_Y_EN        BIT(MLX90397_CTRL1_Y_EN_BIT)
#define MLX90397_CTRL1_Z_EN        BIT(MLX90397_CTRL1_Z_EN_BIT)
#define MLX90397_CTRL1_PREP(MODE, X_EN, Y_EN, Z_EN)                                          \
	(FIELD_PREP(MLX90397_CTRL1_MODE, MODE) | FIELD_PREP(MLX90397_CTRL1_X_EN, X_EN) |           \
	 FIELD_PREP(MLX90397_CTRL1_Y_EN, Y_EN) | FIELD_PREP(MLX90397_CTRL1_Z_EN, Z_EN))
#define MLX90397_CTRL1_DEFAULT MLX90397_CTRL1_PREP(0, 1, 1, 1)

/* CTRL2 values RW */
enum mlx90397_reg_range_xy_val {
	MLX90397_CTRL2_RANGE_SEL_XY_25 = 0,
	MLX90397_CTRL2_RANGE_SEL_XY_50
};
enum mlx90397_reg_range_z_val {
	MLX90397_CTRL2_RANGE_SEL_Z_25 = 0,
	MLX90397_CTRL2_RANGE_SEL_Z_50,
	MLX90397_CTRL2_RANGE_SEL_Z_100,
	MLX90397_CTRL2_RANGE_SEL_Z_200
};
#define MLX90397_CTRL2_RANGE_XY  BIT(0)
#define MLX90397_CTRL2_RANGE_Z  GENMASK(2, 1)
#define MLX90397_CTRL2_PREP(RANGE_XY, RANGE_Z)                         \
	(FIELD_PREP(MLX90397_CTRL2_RANGE_XY, RANGE_XY) | 		 \
	 FIELD_PREP(MLX90397_CTRL2_RANGE_Z, RANGE_Z))
#define MLX90397_CTRL2_DEFAULT                                                                     \
	MLX90397_CTRL2_PREP(MLX90397_CTRL2_RANGE_SEL_XY_25, MLX90397_CTRL2_RANGE_SEL_Z_25)

/* CTRL3 values RW */
#define MLX90397_CTRL3_DIG_FILT_TEMP    GENMASK(2, 0)
#define MLX90397_CTRL3_DIG_FILT_HALL_XY GENMASK(5, 3)
#define MLX90397_CTRL3_OSR_TEMP         BIT(6)
#define MLX90397_CTRL3_OSR_HALL         BIT(7)
#define MLX90397_CTRL3_PREP(DIG_FILT_TEMP, DIG_FILT_HALL_XY, OSR_TEMP, OSR_HALL)                   \
	(FIELD_PREP(MLX90397_CTRL3_DIG_FILT_TEMP, DIG_FILT_TEMP) |                                 \
	 FIELD_PREP(MLX90397_CTRL3_DIG_FILT_HALL_XY, DIG_FILT_HALL_XY) |                           \
	 FIELD_PREP(MLX90397_CTRL3_OSR_TEMP, OSR_TEMP) |                                           \
	 FIELD_PREP(MLX90397_CTRL3_OSR_HALL, OSR_HALL))
#define MLX90397_CTRL3_DEFAULT MLX90397_CTRL3_PREP(1, 4, 1, 1)

/* CTRL4 values RW BIT(6) has to be always 0 so it is not included here */
#define MLX90397_CTRL4_T_EN_BIT        5
#define MLX90397_CTRL4_DIG_FILT_HALL_Z GENMASK(2, 0)
#define MLX90397_CTRL4_T_EN            BIT(MLX90397_CTRL4_T_EN_BIT)
#define MLX90397_CTRL4_PREP(DIG_FILT_HALL_Z, T_EN)                                        \
	(FIELD_PREP(MLX90397_CTRL4_DIG_FILT_HALL_Z, DIG_FILT_HALL_Z) |                             \
	 FIELD_PREP(MLX90397_CTRL4_T_EN, T_EN) |     \
	 BIT(4) | BIT(7))
#define MLX90397_CTRL4_DEFAULT MLX90397_CTRL4_PREP(5, 0)

/* helper function to modify only one field */
#define MLX90397_FIELD_MOD(mask, new_field_val, val)                                               \
	((val & ~mask) | FIELD_PREP(mask, new_field_val))

#endif /* ZEPHYR_DRIVERS_SENSOR_MLX90397_MLX90397_REG_H */
