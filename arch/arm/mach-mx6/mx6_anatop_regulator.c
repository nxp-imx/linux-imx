/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*
 * mx6_anatop_regulator.c  --  i.MX6 Driver for Anatop regulators
 */
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/gpio.h>

#include "crm_regs.h"
#include "regs-anadig.h"

extern struct platform_device sgtl5000_vdda_reg_devices;
extern struct platform_device sgtl5000_vddio_reg_devices;
extern struct platform_device sgtl5000_vddd_reg_devices;

static int get_voltage(struct anatop_regulator *sreg)
{
	int uv;
	struct anatop_regulator_data *rdata = sreg->rdata;

	if (sreg->rdata->control_reg) {
		u32 val = (__raw_readl(rdata->control_reg) >>
			   rdata->vol_bit_shift) & rdata->vol_bit_mask;
		uv = rdata->min_voltage + (val - rdata->min_bit_val) * 25000;
		pr_debug("vddio = %d, val=%u\n", uv, val);
		return uv;
	} else {
		pr_debug("Regulator not supported.\n");
		return -ENOTSUPP;
	}
}

static int set_voltage(struct anatop_regulator *sreg, int uv)
{
	u32 val, reg;

	pr_debug("%s: uv %d, min %d, max %d\n", __func__,
		uv, sreg->rdata->min_voltage, sreg->rdata->max_voltage);

	if (uv < sreg->rdata->min_voltage || uv > sreg->rdata->max_voltage)
		return -EINVAL;

	if (sreg->rdata->control_reg) {
		val = sreg->rdata->min_bit_val +
		      (uv - sreg->rdata->min_voltage) / 25000;

		reg = (__raw_readl(sreg->rdata->control_reg) &
			~(sreg->rdata->vol_bit_mask <<
			sreg->rdata->vol_bit_shift));
		pr_debug("%s: calculated val %d\n", __func__, val);
		__raw_writel((val << sreg->rdata->vol_bit_shift) | reg,
			     sreg->rdata->control_reg);
		return 0;
	} else {
		pr_debug("Regulator not supported.\n");
		return -ENOTSUPP;
	}
}

static int enable(struct anatop_regulator *sreg)
{
	return 0;
}

static int disable(struct anatop_regulator *sreg)
{
	return 0;
}

static int is_enabled(struct anatop_regulator *sreg)
{
	return 1;
}

static struct anatop_regulator_data vddpu_data = {
	.name		= "vddpu",
	.set_voltage	= set_voltage,
	.get_voltage	= get_voltage,
	.enable		= enable,
	.disable	= disable,
	.is_enabled	= is_enabled,
	.control_reg	= (u32)(MXC_PLL_BASE + HW_ANADIG_REG_CORE),
	.vol_bit_shift	= 9,
	.vol_bit_mask	= 0x1F,
	.min_bit_val	= 1,
	.min_voltage	= 725000,
	.max_voltage	= 1300000,
};

static struct anatop_regulator_data vddcore_data = {
	.name		= "vddcore",
	.set_voltage	= set_voltage,
	.get_voltage	= get_voltage,
	.enable		= enable,
	.disable	= disable,
	.is_enabled	= is_enabled,
	.control_reg	= (u32)(MXC_PLL_BASE + HW_ANADIG_REG_CORE),
	.vol_bit_shift	= 0,
	.vol_bit_mask	= 0x1F,
	.min_bit_val	= 1,
	.min_voltage	= 725000,
	.max_voltage	= 1300000,
};

static struct anatop_regulator_data vddsoc_data = {
	.name		= "vddsoc",
	.set_voltage	= set_voltage,
	.get_voltage	= get_voltage,
	.enable		= enable,
	.disable	= disable,
	.is_enabled	= is_enabled,
	.control_reg	= (u32)(MXC_PLL_BASE + HW_ANADIG_REG_CORE),
	.vol_bit_shift	= 18,
	.vol_bit_mask	= 0x1F,
	.min_bit_val	= 1,
	.min_voltage	= 725000,
	.max_voltage	= 1300000,
};

static struct anatop_regulator_data vdd2p5_data = {
	.name		= "vdd2p5",
	.set_voltage	= set_voltage,
	.get_voltage	= get_voltage,
	.enable		= enable,
	.disable	= disable,
	.is_enabled	= is_enabled,
	.control_reg	= (u32)(MXC_PLL_BASE + HW_ANADIG_REG_2P5),
	.vol_bit_shift	= 8,
	.vol_bit_mask	= 0x1F,
	.min_bit_val	= 0,
	.min_voltage	= 2000000,
	.max_voltage	= 2775000,
};

static struct anatop_regulator_data vdd1p1_data = {
	.name		= "vdd1p1",
	.set_voltage	= set_voltage,
	.get_voltage	= get_voltage,
	.enable		= enable,
	.disable	= disable,
	.is_enabled	= is_enabled,
	.control_reg	= (u32)(MXC_PLL_BASE + HW_ANADIG_REG_1P1),
	.vol_bit_shift	= 8,
	.vol_bit_mask	= 0x1F,
	.min_bit_val	= 4,
	.min_voltage	= 800000,
	.max_voltage	= 1400000,
};

static struct anatop_regulator_data vdd3p0_data = {
	.name		= "vdd3p0",
	.set_voltage	= set_voltage,
	.get_voltage	= get_voltage,
	.enable		= enable,
	.disable	= disable,
	.is_enabled	= is_enabled,
	.control_reg	= (u32)(MXC_PLL_BASE + HW_ANADIG_REG_3P0),
	.vol_bit_shift	= 8,
	.vol_bit_mask	= 0x1F,
	.min_bit_val	= 7,
	.min_voltage	= 2800000,
	.max_voltage	= 3150000,
};

/* CPU */
static struct regulator_consumer_supply vddcore_consumers[] = {
	{
		.supply = "cpu_vddgp",
	}
};

static struct regulator_init_data vddpu_init = {
	.constraints = {
		.name			= "vddpu",
		.min_uV			= 725000,
		.max_uV			= 1300000,
		.valid_modes_mask	= REGULATOR_MODE_FAST |
					  REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_MODE,
		.always_on		= 1,
	},
	.num_consumer_supplies = 0,
	.consumer_supplies = NULL,
};

static struct regulator_init_data vddcore_init = {
	.constraints = {
		.name			= "vddcore",
		.min_uV			= 725000,
		.max_uV			= 1300000,
		.valid_modes_mask	= REGULATOR_MODE_FAST |
					  REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_MODE,
		.always_on		= 1,
	},
	.num_consumer_supplies = ARRAY_SIZE(vddcore_consumers),
	.consumer_supplies = &vddcore_consumers[0],
};

static struct regulator_init_data vddsoc_init = {
	.constraints = {
		.name			= "vddsoc",
		.min_uV			= 725000,
		.max_uV			= 1300000,
		.valid_modes_mask	= REGULATOR_MODE_FAST |
					  REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_MODE,
		.always_on		= 1,
	},
	.num_consumer_supplies = 0,
	.consumer_supplies = NULL,
};

static struct regulator_init_data vdd2p5_init = {
	.constraints = {
		.name			= "vdd2p5",
		.min_uV			= 2000000,
		.max_uV			= 2775000,
		.valid_modes_mask	= REGULATOR_MODE_FAST |
					  REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_MODE,
		.always_on		= 1,
	},
	.num_consumer_supplies = 0,
	.consumer_supplies = NULL,
};


static struct regulator_init_data vdd1p1_init = {
	.constraints = {
		.name			= "vdd1p1",
		.min_uV			= 800000,
		.max_uV			= 1400000,
		.valid_modes_mask	= REGULATOR_MODE_FAST |
					  REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_MODE,
		.input_uV		= 5000000,
		.always_on		= 1,
	},
	.num_consumer_supplies = 0,
	.consumer_supplies = NULL,
};


static struct regulator_init_data vdd3p0_init = {
	.constraints = {
		.name			= "vdd3p0",
		.min_uV			= 2800000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_FAST |
					  REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_MODE,
		.always_on		= 1,
	},
	.num_consumer_supplies = 0,
	.consumer_supplies = NULL,
};

static struct anatop_regulator vddpu_reg = {
		.rdata = &vddpu_data,
};

static struct anatop_regulator vddcore_reg = {
		.rdata = &vddcore_data,
};

static struct anatop_regulator vddsoc_reg = {
		.rdata = &vddsoc_data,
};

static struct anatop_regulator vdd2p5_reg = {
		.rdata = &vdd2p5_data,
};

static struct anatop_regulator vdd1p1_reg = {
		.rdata = &vdd1p1_data,
};

static struct anatop_regulator vdd3p0_reg = {
		.rdata = &vdd3p0_data,
};

static int __init regulators_init(void)
{
	anatop_register_regulator(&vddpu_reg, ANATOP_VDDPU, &vddpu_init);
	anatop_register_regulator(&vddcore_reg, ANATOP_VDDCORE, &vddcore_init);
	anatop_register_regulator(&vddsoc_reg, ANATOP_VDDSOC, &vddsoc_init);
	anatop_register_regulator(&vdd2p5_reg, ANATOP_VDD2P5, &vdd2p5_init);
	anatop_register_regulator(&vdd1p1_reg, ANATOP_VDD1P1, &vdd1p1_init);
	anatop_register_regulator(&vdd3p0_reg, ANATOP_VDD3P0, &vdd3p0_init);

	return 0;
}
postcore_initcall(regulators_init);
