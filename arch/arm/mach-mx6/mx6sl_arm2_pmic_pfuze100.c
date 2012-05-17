/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/pfuze.h>
#include <mach/irqs.h>

/*
 * Convenience conversion.
 * Here atm, maybe there is somewhere better for this.
 */
#define mV_to_uV(mV) (mV * 1000)
#define uV_to_mV(uV) (uV / 1000)
#define V_to_uV(V) (mV_to_uV(V * 1000))
#define uV_to_V(uV) (uV_to_mV(uV) / 1000)

#define PFUZE100_I2C_DEVICE_NAME  "pfuze100"
/* 7-bit I2C bus slave address */
#define PFUZE100_I2C_ADDR         (0x08)
 /*SWBST*/
#define PFUZE100_SW1ASTANDBY	33
#define PFUZE100_SW1ASTANDBY_STBY_VAL	(0x16)
#define PFUZE100_SW1ASTANDBY_STBY_M	(0x3f<<0)
#define PFUZE100_SW1BSTANDBY   40
#define PFUZE100_SW1BSTANDBY_STBY_VAL  (0x16)
#define PFUZE100_SW1BSTANDBY_STBY_M    (0x3f<<0)
#define PFUZE100_SW1CSTANDBY	47
#define PFUZE100_SW1CSTANDBY_STBY_VAL	(0x16)
#define PFUZE100_SW1CSTANDBY_STBY_M	(0x3f<<0)
#define PFUZE100_SW2STANDBY     54
#define PFUZE100_SW2STANDBY_STBY_VAL    0x0
#define PFUZE100_SW2STANDBY_STBY_M      (0x3f<<0)
#define PFUZE100_SW3ASTANDBY    61
#define PFUZE100_SW3ASTANDBY_STBY_VAL   0x0
#define PFUZE100_SW3ASTANDBY_STBY_M     (0x3f<<0)
#define PFUZE100_SW3BSTANDBY    68
#define PFUZE100_SW3BSTANDBY_STBY_VAL   0x0
#define PFUZE100_SW3BSTANDBY_STBY_M     (0x3f<<0)
#define PFUZE100_SW4STANDBY     75
#define PFUZE100_SW4STANDBY_STBY_VAL    0
#define PFUZE100_SW4STANDBY_STBY_M      (0x3f<<0)
#define PFUZE100_SWBSTCON1	102
#define PFUZE100_SWBSTCON1_SWBSTMOD_VAL	(0x1<<2)
#define PFUZE100_SWBSTCON1_SWBSTMOD_M	(0x3<<2)


static struct regulator_consumer_supply sw2_consumers[] = {
	{
		.supply		= "MICVDD",
		.dev_name	= "1-001a",
	},
	{
		.supply 	= "DBVDD",
		.dev_name	= "1-001a",
	}

};
static struct regulator_consumer_supply sw4_consumers[] = {
       {
	.supply = "AUD_1V8",
	}
};
static struct regulator_consumer_supply swbst_consumers[] = {
       {
	.supply = "SWBST_5V",
	}
};
static struct regulator_consumer_supply vgen1_consumers[] = {
       {
	.supply = "VGEN1_1V5",
	}
};
static struct regulator_consumer_supply vgen2_consumers[] = {
       {
	.supply = "VGEN2_1V5",
	}
};
static struct regulator_consumer_supply vgen4_consumers[] = {
	{
		.supply    = "AVDD",
		.dev_name	= "1-001a",
	},
	{
		.supply    = "DCVDD",
		.dev_name	= "1-001a",
	},
	{
		.supply    = "CPVDD",
		.dev_name	= "1-001a",
	},
	{
		.supply    = "PLLVDD",
		.dev_name	= "1-001a",
	}
};
static struct regulator_consumer_supply vgen5_consumers[] = {
       {
	.supply = "VGEN5_2V8",
	}
};
static struct regulator_consumer_supply vgen6_consumers[] = {
       {
	.supply = "VGEN6_3V3",
	}
};

static struct regulator_init_data sw1a_init = {
	.constraints = {
			.name = "PFUZE100_SW1A",
#ifdef PFUZE100_FIRST_VERSION
			.min_uV = 650000,
			.max_uV = 1437500,
#else
			.min_uV = 300000,
			.max_uV = 1875000,
#endif
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask = 0,
			.boot_on = 1,
			.always_on = 1,
			},
};

static struct regulator_init_data sw1b_init = {
	.constraints = {
			.name = "PFUZE100_SW1B",
			.min_uV = 300000,
			.max_uV = 1875000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask = 0,
			.always_on = 1,
			.boot_on = 1,
			},
};

static struct regulator_init_data sw1c_init = {
	.constraints = {
			.name = "PFUZE100_SW1C",
			.min_uV = 300000,
			.max_uV = 1875000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask = 0,
			.always_on = 1,
			.boot_on = 1,
			},
};

static struct regulator_init_data sw2_init = {
	.constraints = {
			.name = "PFUZE100_SW2",
#if PFUZE100_SW2_VOL6
			.min_uV = 800000,
			.max_uV = 3950000,
#else
			.min_uV = 400000,
			.max_uV = 1975000,
#endif
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask = 0,
			.always_on = 1,
			.boot_on = 1,
			},
	.num_consumer_supplies = ARRAY_SIZE(sw2_consumers),
	.consumer_supplies = sw2_consumers,
};

static struct regulator_init_data sw3a_init = {
	.constraints = {
			.name = "PFUZE100_SW3A",
#if PFUZE100_SW3_VOL6
			.min_uV = 800000,
			.max_uV = 3950000,
#else
			.min_uV = 400000,
			.max_uV = 1975000,
#endif
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask = 0,
			.always_on = 1,
			.boot_on = 1,
			},
};

static struct regulator_init_data sw3b_init = {
	.constraints = {
			.name = "PFUZE100_SW3B",
#if PFUZE100_SW3_VOL6
			.min_uV = 800000,
			.max_uV = 3950000,
#else
			.min_uV = 400000,
			.max_uV = 1975000,
#endif
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask = 0,
			.always_on = 1,
			.boot_on = 1,
			},
};

static struct regulator_init_data sw4_init = {
	.constraints = {
			.name = "PFUZE100_SW4",
#if PFUZE100_SW4_VOL6
			.min_uV = 800000,
			.max_uV = 3950000,
#else
			.min_uV = 400000,
			.max_uV = 1975000,
#endif
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask = 0,
			},
	.num_consumer_supplies = ARRAY_SIZE(sw4_consumers),
	.consumer_supplies = sw4_consumers,
};

static struct regulator_init_data swbst_init = {
	.constraints = {
			.name = "PFUZE100_SWBST",
			.min_uV = 5000000,
			.max_uV = 5150000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask = 0,
			.always_on = 1,
			.boot_on = 1,
			},
	.num_consumer_supplies = ARRAY_SIZE(swbst_consumers),
	.consumer_supplies = swbst_consumers,
};

static struct regulator_init_data vsnvs_init = {
	.constraints = {
			.name = "PFUZE100_VSNVS",
			.min_uV = 1200000,
			.max_uV = 3000000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask = 0,
			.always_on = 1,
			.boot_on = 1,
			},
};

static struct regulator_init_data vrefddr_init = {
	.constraints = {
			.name = "PFUZE100_VREFDDR",
			.always_on = 1,
			.boot_on = 1,
			},
};

static struct regulator_init_data vgen1_init = {
	.constraints = {
			.name = "PFUZE100_VGEN1",
#ifdef PFUZE100_FIRST_VERSION
			.min_uV = 1200000,
			.max_uV = 1550000,
#else
			.min_uV = 800000,
			.max_uV = 1550000,
#endif
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
			.valid_modes_mask = 0,
			.always_on = 0,
			.boot_on = 0,
			},
	.num_consumer_supplies = ARRAY_SIZE(vgen1_consumers),
	.consumer_supplies = vgen1_consumers,
};

static struct regulator_init_data vgen2_init = {
	.constraints = {
			.name = "PFUZE100_VGEN2",
#ifdef PFUZE100_FIRST_VERSION
			.min_uV = 1200000,
			.max_uV = 1550000,
#else
			.min_uV = 800000,
			.max_uV = 1550000,
#endif
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
			.valid_modes_mask = 0,
			},
	.num_consumer_supplies = ARRAY_SIZE(vgen2_consumers),
	.consumer_supplies = vgen2_consumers,

};

static struct regulator_init_data vgen3_init = {
	.constraints = {
			.name = "PFUZE100_VGEN3",
			.min_uV = 1800000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
			.valid_modes_mask = 0,
			.always_on = 0,
			.boot_on = 0,
			},
};

static struct regulator_init_data vgen4_init = {
	.constraints = {
			.name = "PFUZE100_VGEN4",
			.min_uV = 1800000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
			.valid_modes_mask = 0,
			},
	.num_consumer_supplies = ARRAY_SIZE(vgen4_consumers),
	.consumer_supplies = vgen4_consumers,
};

static struct regulator_init_data vgen5_init = {
	.constraints = {
			.name = "PFUZE100_VGEN5",
			.min_uV = 1800000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
			.valid_modes_mask = 0,
			},
	.num_consumer_supplies = ARRAY_SIZE(vgen5_consumers),
	.consumer_supplies = vgen5_consumers,
};

static struct regulator_init_data vgen6_init = {
	.constraints = {
			.name = "PFUZE100_VGEN6",
			.min_uV = 1800000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
			.valid_modes_mask = 0,
			},
	.num_consumer_supplies = ARRAY_SIZE(vgen6_consumers),
	.consumer_supplies = vgen6_consumers,
};

static int pfuze100_init(struct mc_pfuze *pfuze)
{
	int ret;
	ret = pfuze_reg_rmw(pfuze, PFUZE100_SW1ASTANDBY,
			    PFUZE100_SW1ASTANDBY_STBY_M,
			    PFUZE100_SW1ASTANDBY_STBY_VAL);
	if (ret)
		goto err;
	ret = pfuze_reg_rmw(pfuze, PFUZE100_SW1BSTANDBY,
			    PFUZE100_SW1BSTANDBY_STBY_M,
			    PFUZE100_SW1BSTANDBY_STBY_VAL);
	if (ret)
		goto err;
	ret = pfuze_reg_rmw(pfuze, PFUZE100_SW1CSTANDBY,
			    PFUZE100_SW1CSTANDBY_STBY_M,
			    PFUZE100_SW1CSTANDBY_STBY_VAL);
	if (ret)
		goto err;
	return 0;
err:
	printk(KERN_ERR "pfuze100 init error!\n");
	return -1;
}

static struct pfuze_regulator_init_data mx6q_sabreauto_pfuze100_regulators[] = {
	{.id = PFUZE100_SW1A,	.init_data = &sw1a_init},
	{.id = PFUZE100_SW1B,	.init_data = &sw1b_init},
	{.id = PFUZE100_SW1C,	.init_data = &sw1c_init},
	{.id = PFUZE100_SW2,	.init_data = &sw2_init},
	{.id = PFUZE100_SW3A,	.init_data = &sw3a_init},
	{.id = PFUZE100_SW3B,	.init_data = &sw3b_init},
	{.id = PFUZE100_SW4,	.init_data = &sw4_init},
	{.id = PFUZE100_SWBST,	.init_data = &swbst_init},
	{.id = PFUZE100_VSNVS,	.init_data = &vsnvs_init},
	{.id = PFUZE100_VREFDDR,	.init_data = &vrefddr_init},
	{.id = PFUZE100_VGEN1,	.init_data = &vgen1_init},
	{.id = PFUZE100_VGEN2,	.init_data = &vgen2_init},
	{.id = PFUZE100_VGEN3,	.init_data = &vgen3_init},
	{.id = PFUZE100_VGEN4,	.init_data = &vgen4_init},
	{.id = PFUZE100_VGEN5,	.init_data = &vgen5_init},
	{.id = PFUZE100_VGEN6,	.init_data = &vgen6_init},
};

static struct pfuze_platform_data pfuze100_plat = {
	.flags = PFUZE_USE_REGULATOR,
	.num_regulators = ARRAY_SIZE(mx6q_sabreauto_pfuze100_regulators),
	.regulators = mx6q_sabreauto_pfuze100_regulators,
	.pfuze_init = pfuze100_init,
};

static struct i2c_board_info __initdata pfuze100_i2c_device = {
	I2C_BOARD_INFO(PFUZE100_I2C_DEVICE_NAME, PFUZE100_I2C_ADDR),
	.platform_data = &pfuze100_plat,
};

int __init mx6sl_arm2_init_pfuze100(u32 int_gpio)
{
	if (int_gpio)
		pfuze100_i2c_device.irq = gpio_to_irq(int_gpio); /*update INT gpio */
	return i2c_register_board_info(0, &pfuze100_i2c_device, 1);
}
