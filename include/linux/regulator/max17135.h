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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */
#ifndef __LINUX_REGULATOR_MAX17135_H_
#define __LINUX_REGULATOR_MAX17135_H_

enum {
    /* In alphabetical order */
    MAX17135_DISPLAY, /* virtual master enable */
    MAX17135_GVDD,
    MAX17135_GVEE,
    MAX17135_HVINN,
    MAX17135_HVINP,
    MAX17135_VCOM,
    MAX17135_VNEG,
    MAX17135_VPOS,
    MAX17135_NUM_REGULATORS,
};

/*
 * Declarations
 */
struct regulator_init_data;

struct max17135_platform_data {
	unsigned int gvee_pwrup;
	unsigned int vneg_pwrup;
	unsigned int vpos_pwrup;
	unsigned int gvdd_pwrup;
	unsigned int gvdd_pwrdn;
	unsigned int vpos_pwrdn;
	unsigned int vneg_pwrdn;
	unsigned int gvee_pwrdn;
	int gpio_pmic_pwrgood;
	int gpio_pmic_vcom_ctrl;
	int gpio_pmic_wakeup;
	int gpio_pmic_intr;
	struct regulator_init_data *regulator_init;
};

#endif
