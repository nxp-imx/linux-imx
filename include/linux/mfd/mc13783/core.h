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

#ifndef __LINUX_MFD_MC13783_CORE_H_
#define __LINUX_MFD_MC13783_CORE_H_

#include <linux/kernel.h>

/*!
 * brief PMIC regulators.
 */
#define MC13783_SW1A 0
#define MC13783_SW1B 1
#define MC13783_SW2A 2
#define MC13783_SW2B 3
#define MC13783_SW3 4
#define MC13783_VAUDIO 5
#define MC13783_VIOHI 6
#define MC13783_VIOLO 7
#define MC13783_VDIG 8
#define MC13783_VGEN 9
#define MC13783_VRFDIG 10
#define MC13783_VRFREF 11
#define MC13783_VRFCP 12
#define MC13783_VSIM 13
#define MC13783_VESIM 14
#define MC13783_VCAM 15
#define MC13783_VRFBG 16
#define MC13783_VVIB 17
#define MC13783_VRF1 18
#define MC13783_VRF2 19
#define MC13783_VMMC1 20
#define MC13783_VMMC2 21
#define MC13783_GPO1 22
#define MC13783_GPO2 23
#define MC13783_GPO3 24
#define MC13783_GPO4 25
#define MC13783_REG_NUM 26

struct mc13783;
struct regulator_init_data;

struct mc13783_pmic {
	/* regulator devices */
	struct platform_device *pdev[MC13783_REG_NUM];
};

struct mc13783 {
	int rev;		/* chip revision */

	struct device *dev;

	u16 *reg_cache;

	/* Client devices */
	struct mc13783_pmic pmic;
};

int mc13783_register_regulator(struct mc13783 *mc13783, int reg,
			       struct regulator_init_data *initdata);

#endif
