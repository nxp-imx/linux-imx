/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <mach/hardware.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>

struct regulator *cpu_regulator;
struct regulator *soc_regulator;
struct regulator *pu_regulator;
char *gp_reg_id;


void mx5_cpu_regulator_init(void)
{
	cpu_regulator = regulator_get(NULL, gp_reg_id);
	if (IS_ERR(cpu_regulator))
		printk(KERN_ERR "%s: failed to get cpu regulator\n", __func__);
	soc_regulator = pu_regulator = -ENODEV;
}

