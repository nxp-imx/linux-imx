/*
 * Copyright 2008-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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

/*!
 * @file pmic/core/mc13892.c
 * @brief This file contains MC13892 specific PMIC code. This implementaion
 * may differ for each PMIC chip.
 *
 * @ingroup PMIC_CORE
 */

/*
 * Includes
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/mfd/mc13892/core.h>

#include <asm/mach-types.h>
#include <asm/uaccess.h>

#include "pmic.h"

void *mc13892_alloc_data(struct device *dev)
{
	struct mc13892 *mc13892;

	mc13892 = kzalloc(sizeof(struct mc13892), GFP_KERNEL);
	if (mc13892 == NULL)
		return NULL;

	mc13892->dev = dev;

	return (void *)mc13892;
}
EXPORT_SYMBOL(mc13892_alloc_data);

int mc13892_init_registers(void)
{
	CHECK_ERROR(pmic_write(REG_INT_MASK0, 0xFFFFFF));
	CHECK_ERROR(pmic_write(REG_INT_MASK0, 0xFFFFFF));
	CHECK_ERROR(pmic_write(REG_INT_STATUS0, 0xFFFFFF));
	CHECK_ERROR(pmic_write(REG_INT_STATUS1, 0xFFFFFF));
	/* disable auto charge */
	if (machine_is_mx51_3ds())
		CHECK_ERROR(pmic_write(REG_CHARGE, 0xB40003));

	pm_power_off = mc13892_power_off;

	return PMIC_SUCCESS;
}
EXPORT_SYMBOL(mc13892_init_registers);

/*!
 * This function returns the PMIC version in system.
 *
 * @param 	ver	pointer to the pmic_version_t structure
 *
 * @return       This function returns PMIC version.
 */
void mc13892_get_revision(pmic_version_t *ver)
{
	int rev_id = 0;
	int rev1 = 0;
	int rev2 = 0;
	int finid = 0;
	int icid = 0;

	ver->id = PMIC_MC13892;
	pmic_read(REG_IDENTIFICATION, &rev_id);

	rev1 = (rev_id & 0x018) >> 3;
	rev2 = (rev_id & 0x007);
	icid = (rev_id & 0x01C0) >> 6;
	finid = (rev_id & 0x01E00) >> 9;

	ver->revision = ((rev1 * 10) + rev2);
	printk(KERN_INFO "mc13892 Rev %d.%d FinVer %x detected\n", rev1,
	       rev2, finid);
}
EXPORT_SYMBOL(mc13892_get_revision);

void mc13892_power_off(void)
{
	unsigned int value;

	pmic_read_reg(REG_POWER_CTL0, &value, 0xffffff);

	value |= 0x000008;

	pmic_write_reg(REG_POWER_CTL0, value, 0xffffff);
}
