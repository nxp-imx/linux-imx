/*
 * Copyright 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __LINUX_MFD_MC13892_CORE_H_
#define __LINUX_MFD_MC13892_CORE_H_

#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>

#define MC9S08DZ60_LCD 0
#define MC9S08DZ60_WIFI 1
#define MC9S08DZ60_HDD 2
#define MC9S08DZ60_GPS 3
#define MC9S08DZ60_SPKR 4
#define MC9S08DZ60_REG_NUM 5

struct mc9s08dz60;
struct regulator_init_data;

struct mc9s08dz60_platform_data {
	int (*init)(struct mc9s08dz60 *);
};

struct mc9s08dz60_pmic {
	/* regulator devices */
	struct platform_device *pdev[MC9S08DZ60_REG_NUM];
};

struct mc9s08dz60 {
	int rev;		/* chip revision */

	struct device *dev;

	/* device IO */
	union {
		struct i2c_client *i2c_client;
		struct spi_device *spi_device;
	};
	u16 *reg_cache;

	/* Client devices */
	struct mc9s08dz60_pmic pmic;
};

int mc9s08dz60_register_regulator(struct mc9s08dz60 *mc9s08dz60, int reg,
			      struct regulator_init_data *initdata);

#endif
