/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
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

#ifndef __LINUX_MFD_MC34708_CORE_H_
#define __LINUX_MFD_MC34708_CORE_H_

#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/pmic_external.h>
/*!
 * brief PMIC regulators.
 */
#define MC34708_SW1A		0
#define MC34708_SW1B		1
#define MC34708_SW2		2
#define MC34708_SW3		3
#define MC34708_SW4A		4
#define MC34708_SW4B		5
#define MC34708_SW5		6
#define MC34708_SWBST		7
#define MC34708_VPLL		8
#define MC34708_VREFDDR		9
#define MC34708_VUSB		10
#define MC34708_VUSB2		11
#define MC34708_VDAC		12
#define MC34708_VGEN1		13
#define MC34708_VGEN2		14
#define MC34708_REG_NUM		15

struct mc34708;
struct regulator_init_data;

struct mc34708_platform_data {
	int (*init)(struct mc34708 *);
	void *(*pmic_alloc_data)(struct device *dev);
	int (*pmic_init_registers)(void);
	void (*pmic_get_revision)(pmic_version_t *ver);
};

struct mc34708_pmic {
	/* regulator devices */
	struct platform_device *pdev[MC34708_REG_NUM];
};

struct mc34708 {
	int rev;		/* chip revision */

	struct device *dev;

	/* device IO */
	union {
		struct i2c_client *i2c_client;
		struct spi_device *spi_device;
	};
	u16 *reg_cache;

	/* Client devices */
	struct mc34708_pmic pmic;
};

int mc34708_register_regulator(struct mc34708 *mc34708, int reg,
			       struct regulator_init_data *initdata);
void *mc34708_alloc_data(struct device *dev);
int mc34708_init_registers(void);
void mc34708_get_revision(pmic_version_t *ver);

#endif
