/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef __LINUX_MFD_LTC3589_CORE_H_
#define __LINUX_MFD_LTC3589_CORE_H_

#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/regulator/driver.h>

#define LTC3589_SW1 0
#define LTC3589_SW2 1
#define LTC3589_SW3 2
#define LTC3589_SW4 3
#define LTC3589_LDO1 4
#define LTC3589_LDO2 5
#define LTC3589_LDO3 6
#define LTC3589_LDO4 7
#define LTC3589_REG_NUM 8

extern struct i2c_client *ltc3589_client;

struct ltc3589;
struct regulator_init_data;

struct ltc3589_platform_data {
	int (*init)(struct ltc3589 *);
};

struct ltc3589_pmic {
	/* regulator devices */
	struct platform_device *pdev[LTC3589_REG_NUM];
};

struct ltc3589 {
	int rev;		/* chip revision */

	struct device *dev;

	/* device IO */
	struct i2c_client *i2c_client;
	int (*read_dev)(struct ltc3589 *ltc, u8 reg);
	int (*write_dev)(struct ltc3589 *ltc, u8 reg, u8 val);
	u16 *reg_cache;

	/* Client devices */
	struct ltc3589_pmic pmic;
};

int ltc3589_register_regulator(struct ltc3589 *ltc3589, int reg,
			      struct regulator_init_data *initdata);
#endif
