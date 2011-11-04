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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/mc-pmic.h>
#include <mach/irqs.h>
#include <mach/iomux-mx53.h>

/*
 * Convenience conversion.
 * Here atm, maybe there is somewhere better for this.
 */
#define mV_to_uV(mV) (mV * 1000)
#define uV_to_mV(uV) (uV / 1000)
#define V_to_uV(V) (mV_to_uV(V * 1000))
#define uV_to_V(uV) (uV_to_mV(uV) / 1000)

#define MC34708_I2C_DEVICE_NAME  "mc34708"
/* 7-bit I2C bus slave address */
#define MC34708_I2C_ADDR         (0x08)


static struct regulator_init_data sw1a_init = {
	.constraints = {
		.name = "SW1",
		.min_uV = 650000,
		.max_uV = 1437500,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = 0,
		.always_on = 1,
		.boot_on = 1,
	},
};

static struct regulator_init_data sw1b_init = {
	.constraints = {
		.name = "SW1B",
		.min_uV = 650000,
		.max_uV = 1437500,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = 0,
		.always_on = 1,
		.boot_on = 1,
	},
};

static struct regulator_init_data sw2_init = {
	.constraints = {
		.name = "SW2",
		.min_uV = 650000,
		.max_uV = 1437500,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.always_on = 1,
		.boot_on = 1,
		.initial_state = PM_SUSPEND_MEM,
	}
};

static struct regulator_init_data sw3_init = {
	.constraints = {
		.name = "SW3",
		.min_uV = 650000,
		.max_uV = 1425000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.boot_on = 1,
	}
};

static struct regulator_init_data sw4a_init = {
	.constraints = {
		.name = "SW4A",
		.min_uV = mV_to_uV(1200),
		.max_uV = mV_to_uV(3300),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.always_on = 1,
		.boot_on = 1,
	}
};

static struct regulator_init_data sw4b_init = {
	.constraints = {
		.name = "SW4B",
		.min_uV = mV_to_uV(1200),
		.max_uV = mV_to_uV(3300),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.always_on = 1,
		.boot_on = 1,
	}
};

static struct regulator_init_data sw5_init = {
	.constraints = {
		.name = "SW5",
		.min_uV = mV_to_uV(1200),
		.max_uV = mV_to_uV(1975),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.always_on = 1,
		.boot_on = 1,
	}
};

static struct regulator_init_data vrefddr_init = {
	.constraints = {
		.name = "VREFDDR",
		.always_on = 1,
		.boot_on = 1,
	}
};

static struct regulator_init_data vusb_init = {
	.constraints = {
		.name = "VUSB",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.boot_on = 1,
		.always_on = 1,
	}
};

static struct regulator_init_data swbst_init = {
	.constraints = {
		.name = "SWBST",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.boot_on = 1,
		.always_on = 1,
	}
};

static struct regulator_init_data vpll_init = {
	.constraints = {
		.name = "VPLL",
		.min_uV = mV_to_uV(1200),
		.max_uV = mV_to_uV(1800),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.boot_on = 1,
	},
};

static struct regulator_init_data vdac_init = {
	.constraints = {
		.name = "VDAC",
		.min_uV = mV_to_uV(2500),
		.max_uV = mV_to_uV(2775),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.boot_on = 1,
		.always_on = 1,
	}
};

static struct regulator_init_data vusb2_init = {
	.constraints = {
		.name = "VUSB2",
		.min_uV = mV_to_uV(2500),
		.max_uV = mV_to_uV(3000),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.boot_on = 1,
		.always_on = 1,
	}
};

static struct regulator_init_data vgen1_init = {
	.constraints = {
		.name = "VGEN1",
		.min_uV = mV_to_uV(1200),
		.max_uV = mV_to_uV(1550),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.always_on = 1,
	}
};

static struct regulator_init_data vgen2_init = {
	.constraints = {
		.name = "VGEN2",
		.min_uV = mV_to_uV(2500),
		.max_uV = mV_to_uV(3300),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.always_on = 1,
	}
};

static struct mc_pmic_regulator_init_data mx53_loco_mc34708_regulators[] = {
	{ .id = MC34708_SW1A,			.init_data = &sw1a_init},
	{ .id = MC34708_SW1B,			.init_data = &sw1b_init},
	{ .id = MC34708_SW2,			.init_data = &sw2_init},
	{ .id = MC34708_SW3,			.init_data = &sw3_init},
	{ .id = MC34708_SW4A,			.init_data = &sw4a_init},
	{ .id = MC34708_SW4B,			.init_data = &sw4b_init},
	{ .id = MC34708_SW5,			.init_data = &sw5_init},
	{ .id = MC34708_SWBST,			.init_data = &swbst_init},
	{ .id = MC34708_VPLL,			.init_data = &vpll_init},
	{ .id = MC34708_VREFDDR,		.init_data = &vrefddr_init},
	{ .id = MC34708_VUSB,			.init_data = &vusb_init},
	{ .id = MC34708_VUSB2,			.init_data = &vusb2_init},
	{ .id = MC34708_VDAC,			.init_data = &vdac_init},
	{ .id = MC34708_VGEN1,			.init_data = &vgen1_init},
	{ .id = MC34708_VGEN2,			.init_data = &vgen2_init},

};


static struct mc_pmic_platform_data mc34708_plat = {
	.flags = MC_PMIC_USE_RTC | MC_PMIC_USE_REGULATOR,
	.num_regulators = ARRAY_SIZE(mx53_loco_mc34708_regulators),
	.regulators = mx53_loco_mc34708_regulators,
};

static struct i2c_board_info __initdata mc34708_i2c_device = {
	I2C_BOARD_INFO(MC34708_I2C_DEVICE_NAME, MC34708_I2C_ADDR),
	.platform_data = &mc34708_plat,
};

int __init mx53_loco_init_mc34708(u32 int_gpio)
{
	mc34708_i2c_device.irq = gpio_to_irq(int_gpio);/*update INT gpio*/
	return i2c_register_board_info(0, &mc34708_i2c_device, 1);
}
