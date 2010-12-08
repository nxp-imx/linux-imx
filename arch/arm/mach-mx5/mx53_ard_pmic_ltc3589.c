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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*
 * mx53_ard_pmic_ltc3589.c  --  i.MX53 ARD Driver for Linear LTC3589
 * PMIC
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/regulator/ltc3589.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/ltc3589/core.h>
#include <mach/iomux-mx53.h>
#include <mach/irqs.h>

#define ARD_PMIC_INT			(4*32 + 7)	/* GPIO_5_7 */

/* CPU */
static struct regulator_consumer_supply sw1_consumers[] = {
	{
		.supply = "cpu_vcc",
	}
};

struct ltc3589;

static struct regulator_init_data sw1_init = {
	.constraints = {
		.name = "SW1",
		.min_uV = 564000,
		.max_uV = 1167000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = 0,
		.always_on = 1,
		.boot_on = 1,
		.initial_state = PM_SUSPEND_MEM,
		.state_mem = {
			.uV = 950000,
			.mode = REGULATOR_MODE_NORMAL,
			.enabled = 1,
		},
	},
	.num_consumer_supplies = ARRAY_SIZE(sw1_consumers),
	.consumer_supplies = sw1_consumers,
};

static struct regulator_init_data sw2_init = {
	.constraints = {
		.name = "SW2",
		.min_uV = 704000,
		.max_uV = 1456000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.always_on = 1,
		.boot_on = 1,
		.initial_state = PM_SUSPEND_MEM,
		.state_mem = {
			.uV = 950000,
			.mode = REGULATOR_MODE_NORMAL,
			.enabled = 1,
		},
	},
};

static struct regulator_init_data sw3_init = {
	.constraints = {
		.name = "SW3",
		.min_uV = 1342000,
		.max_uV = 2775000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.always_on = 1,
		.boot_on = 1,
	},
};

static struct regulator_init_data sw4_init = {
	.constraints = {
		.name = "SW4",
		.apply_uV = 1,
		.boot_on = 1,
	}
};

static struct regulator_init_data ldo1_init = {
	.constraints = {
		.name = "LDO1_STBY",
		.apply_uV = 1,
		.boot_on = 1,
		},
};

static struct regulator_init_data ldo2_init = {
	.constraints = {
		.name = "LDO2",
		.min_uV = 704000,
		.max_uV = 1456000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.always_on = 1,
		.boot_on = 1,
	},
};

static struct regulator_init_data ldo3_init = {
	.constraints = {
			.name = "LDO3",
			.apply_uV = 1,
			.boot_on = 1,
			},
};

static struct regulator_init_data ldo4_init = {
	.constraints = {
			.name = "LDO4",
			.min_uV = 1800000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.always_on = 1,
			.boot_on = 1,
			},
};

static void ltc3589_nop_release(struct device *dev)
{
	/* Nothing */
}

static struct platform_device ltc3589_regulator_device[] = {
	{
	.name = "ltc3589-dev",
	.id = 0,
	.dev = {
		.release = ltc3589_nop_release,
		},
			},
};

static int mx53_ltc3589_init(struct ltc3589 *ltc3589)
{
	int i;

	printk(KERN_INFO "Initializing regulators for ARD\n");
	for (i = 0; i < ARRAY_SIZE(ltc3589_regulator_device); i++) {
		if (platform_device_register(&ltc3589_regulator_device[i]) < 0)
			dev_err(&ltc3589_regulator_device[i].dev,
				"Unable to register LTC3589 device\n");
	}

	ltc3589_register_regulator(ltc3589, LTC3589_SW1, &sw1_init);
	ltc3589_register_regulator(ltc3589, LTC3589_SW2, &sw2_init);
	ltc3589_register_regulator(ltc3589, LTC3589_SW3, &sw3_init);
	ltc3589_register_regulator(ltc3589, LTC3589_SW4, &sw4_init);
	ltc3589_register_regulator(ltc3589, LTC3589_LDO1, &ldo1_init);
	ltc3589_register_regulator(ltc3589, LTC3589_LDO2, &ldo2_init);
	ltc3589_register_regulator(ltc3589, LTC3589_LDO3, &ldo3_init);
	ltc3589_register_regulator(ltc3589, LTC3589_LDO4, &ldo4_init);

	return 0;
}

static struct ltc3589_platform_data __initdata ltc3589_plat = {
	.init = mx53_ltc3589_init,
};

static struct i2c_board_info __initdata ltc3589_i2c_device = {
	I2C_BOARD_INFO("ltc3589", 0x34),
	.irq = IOMUX_TO_IRQ_V3(ARD_PMIC_INT),
	.platform_data = &ltc3589_plat,
};

static __init int mx53_init_i2c(void)
{
	return i2c_register_board_info(1, &ltc3589_i2c_device, 1);
}

subsys_initcall(mx53_init_i2c);

static __init int ltc3589_pmic_init(void)
{
	int i = 0;
	int ret = 0;
	struct regulator *regulator;

	char *ltc3589_global_regulator[] = {
		"SW1",
		"SW2",
		"SW3",
		"SW4",
		"LDO1_STBY",
		"LDO2",
		"LDO3",
		"LDO4",
	};

	while ((i < ARRAY_SIZE(ltc3589_global_regulator)) &&
		!IS_ERR_VALUE(
			(unsigned long)(regulator =
					regulator_get(NULL,
						ltc3589_global_regulator
						[i])))) {
		regulator_enable(regulator);
		i++;
	}

	return ret;
}

late_initcall(ltc3589_pmic_init);

