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

#ifndef __LINUX_MFD_MC34704_CORE_H_
#define __LINUX_MFD_MC34704_CORE_H_

#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>

#define	MC34704_BKLT	0	/* REG1 5V for Backlight */
#define	MC34704_CPU	1	/* REG2 3.3V for CPU & CPU board peripherals */
#define	MC34704_CORE	2	/* REG3 1.45V for CPU Core */
#define	MC34704_DDR	3	/* REG4 1.8V for DDR */
#define	MC34704_PERS	4	/* REG5 3.3V for Personality board */
#define	MC34704_REG6	5	/* REG6 not used */
#define	MC34704_REG7	6	/* REG7 not used */
#define	MC34704_REG8	7	/* REG8 not used */
#define	MC34704_REG_NUM	8

/* Regulator voltages in mV and dynamic voltage scaling limits in percent */

#define REG1_V_MV		5000
#define REG1_DVS_MIN_PCT	(-10)
#define REG1_DVS_MAX_PCT	10

#define REG2_V_MV		3300
#define REG2_DVS_MIN_PCT	(-20)
#define REG2_DVS_MAX_PCT	17.5

#define REG3_V_MV		1450
#define REG3_DVS_MIN_PCT	(-20)
#define REG3_DVS_MAX_PCT	17.5

#define REG4_V_MV		1800
#define REG4_DVS_MIN_PCT	(-20)
#define REG4_DVS_MAX_PCT	17.5

#define REG5_V_MV		3300
#define REG5_DVS_MIN_PCT	(-20)
#define REG5_DVS_MAX_PCT	17.5

struct mc34704;
struct regulator_init_data;

struct mc34704_platform_data {
	int (*init) (struct mc34704 *);
};

struct mc34704_pmic {
	/* regulator devices */
	struct platform_device *pdev[MC34704_REG_NUM];
};

struct mc34704 {
	int rev;		/* chip revision */

	struct device *dev;

	/* device IO */
	union {
		struct i2c_client *i2c_client;
		struct spi_device *spi_device;
	};
	u16 *reg_cache;

	/* Client devices */
	struct mc34704_pmic pmic;
};

int mc34704_register_regulator(struct mc34704 *mc34704, int reg,
			       struct regulator_init_data *initdata);

#endif
