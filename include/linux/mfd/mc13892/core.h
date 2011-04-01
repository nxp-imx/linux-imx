/*
 * Copyright 2009-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#ifndef __LINUX_MFD_MC13892_CORE_H_
#define __LINUX_MFD_MC13892_CORE_H_

#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/pmic_external.h>

#define MC13892_SW1 0
#define MC13892_SW2 1
#define MC13892_SW3 2
#define MC13892_SW4 3
#define MC13892_SWBST 4
#define MC13892_VIOHI 5
#define MC13892_VPLL 6
#define MC13892_VDIG 7
#define MC13892_VSD 8
#define MC13892_VUSB2 9
#define MC13892_VVIDEO 10
#define MC13892_VAUDIO 11
#define MC13892_VCAM 12
#define MC13892_VGEN1 13
#define MC13892_VGEN2 14
#define MC13892_VGEN3 15
#define MC13892_VUSB 16
#define MC13892_GPO1 17
#define MC13892_GPO2 18
#define MC13892_GPO3 19
#define MC13892_GPO4 20
#define MC13892_PWGT1 21
#define MC13892_PWGT2 22
#define MC13892_REG_NUM 23

struct mc13892;
struct regulator_init_data;

struct mc13892_platform_data {
	int (*init)(struct mc13892 *);
};

struct mc13892_pmic {
	/* regulator devices */
	struct platform_device *pdev[MC13892_REG_NUM];
};

struct mc13892 {
	int rev;		/* chip revision */

	struct device *dev;

	/* device IO */
	union {
		struct i2c_client *i2c_client;
		struct spi_device *spi_device;
	};
	u16 *reg_cache;

	/* Client devices */
	struct mc13892_pmic pmic;
};

int mc13892_register_regulator(struct mc13892 *mc13892, int reg,
			      struct regulator_init_data *initdata);

void *mc13892_alloc_data(struct device *dev);
int mc13892_init_registers(void);
void mc13892_get_revision(pmic_version_t *ver);
#endif
