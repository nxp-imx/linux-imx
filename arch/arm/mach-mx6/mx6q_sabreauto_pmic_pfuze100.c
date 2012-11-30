/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/io.h>
#include <mach/irqs.h>
#include <mach/system.h>
#include "crm_regs.h"
#include "regs-anadig.h"
#include "cpu_op-mx6.h"

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
#define PFUZE100_DEVICEID		(0x0)
#define PFUZE100_REVID			(0x3)
#define PFUZE100_SW1AMODE		(0x23)
#define PFUZE100_SW1AVOL       32
#define PFUZE100_SW1AVOL_VSEL_M        (0x3f<<0)
#define PFUZE100_SW1CVOL       46
#define PFUZE100_SW1CVOL_VSEL_M        (0x3f<<0)
#define PFUZE100_SW1ACON		36
#define PFUZE100_SW1ACON_SPEED_VAL	(0x1<<6)	/*default */
#define PFUZE100_SW1ACON_SPEED_M	(0x3<<6)
#define PFUZE100_SW1CCON		49
#define PFUZE100_SW1CCON_SPEED_VAL	(0x1<<6)	/*default */
#define PFUZE100_SW1CCON_SPEED_M	(0x3<<6)

extern u32 arm_max_freq;
extern u32 enable_ldo_mode;

static struct regulator_consumer_supply sw1a_consumers[] = {
	{
	 .supply = "VDDCORE",
	 }
};

static struct regulator_consumer_supply sw1c_consumers[] = {
	{
	 .supply = "VDDSOC",
	 }
};

static struct regulator_consumer_supply sw2_consumers[] = {
	{
	 .supply = "P3V0_VDDHIGH_SW2",
	 }
};

static struct regulator_consumer_supply sw3_consumers[] = {
	{
	 .supply = "P1V5_DDR_SW3",
	 }
};

static struct regulator_consumer_supply sw4_consumers[] = {
	{
	 .supply = "P1V8_SW4",
	 }
};

static struct regulator_consumer_supply swbst_consumers[] = {
	{
	 .supply = "P5V0_SWBST",
	 }
};

static struct regulator_consumer_supply vsnvs_consumers[] = {
	{
	 .supply = "P3V0_STBY",
	 }
};

static struct regulator_consumer_supply vgen1_consumers[] = {
	{
	 .supply = "P1V2_VGEN1",
	 }
};

static struct regulator_consumer_supply vgen2_consumers[] = {
	{
	 .supply = "P1V5_VGEN2",
	 }
};

static struct regulator_consumer_supply vgen3_consumers[] = {
	{
	 .supply = "P1V8_VGEN3",
	 }
};

static struct regulator_consumer_supply vgen4_consumers[] = {
	{
	 .supply = "P1V8_VGEN4",
	 }
};

static struct regulator_consumer_supply vgen5_consumers[] = {
	{
	 .supply = "P2V5_VGEN5",
	 }
};

static struct regulator_consumer_supply vgen6_consumers[] = {
	{
	 .supply = "P2V8_VGEN6",
	 }
};

static struct regulator_init_data sw1a_init = {
	.constraints = {
			.name = "SW1A",
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
			.initial_state = PM_SUSPEND_MEM,
			.state_mem = {
				.uV = 975000,/*0.9V+6%*/
				.mode = REGULATOR_MODE_NORMAL,
				.enabled = 1,
			},
	},
	.num_consumer_supplies = ARRAY_SIZE(sw1a_consumers),
	.consumer_supplies = sw1a_consumers,
};

static struct regulator_init_data sw1b_init = {
	.constraints = {
			.name = "SW1B",
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
			.name = "SW1C",
			.min_uV = 300000,
			.max_uV = 1875000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask = 0,
			.always_on = 1,
			.boot_on = 1,
			.initial_state = PM_SUSPEND_MEM,
			.state_mem = {
				.uV = 975000,/*0.9V+6%*/
				.mode = REGULATOR_MODE_NORMAL,
				.enabled = 1,
			},
	},
	.num_consumer_supplies = ARRAY_SIZE(sw1c_consumers),
	.consumer_supplies = sw1c_consumers,
};

static struct regulator_init_data sw2_init = {
	.constraints = {
			.name = "SW2",
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
			.name = "SW3A",
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
	.num_consumer_supplies = ARRAY_SIZE(sw3_consumers),
	.consumer_supplies = sw3_consumers,
};

static struct regulator_init_data sw3b_init = {
	.constraints = {
			.name = "SW3B",
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
			.name = "SW4",
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
			.name = "SWBST",
			.min_uV = 5000000,
			.max_uV = 5150000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask = 0,
			.always_on = 0,
			.boot_on = 0,
			},
	.num_consumer_supplies = ARRAY_SIZE(swbst_consumers),
	.consumer_supplies = swbst_consumers,
};

static struct regulator_init_data vsnvs_init = {
	.constraints = {
			.name = "VSNVS",
			.min_uV = 1200000,
			.max_uV = 3000000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask = 0,
			.always_on = 1,
			.boot_on = 1,
			},
	.num_consumer_supplies = ARRAY_SIZE(vsnvs_consumers),
	.consumer_supplies = vsnvs_consumers,
};

static struct regulator_init_data vrefddr_init = {
	.constraints = {
			.name = "VREFDDR",
			.always_on = 1,
			.boot_on = 1,
			},
};

static struct regulator_init_data vgen1_init = {
	.constraints = {
			.name = "VGEN1",
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
			.name = "VGEN2",
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
			.name = "VGEN3",
			.min_uV = 1800000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
			.valid_modes_mask = 0,
			.always_on = 0,
			.boot_on = 0,
			},
	.num_consumer_supplies = ARRAY_SIZE(vgen3_consumers),
	.consumer_supplies = vgen3_consumers,
};

static struct regulator_init_data vgen4_init = {
	.constraints = {
			.name = "VGEN4",
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
			.name = "VGEN5",
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
			.name = "VGEN6",
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
	int ret, i;
	unsigned char value;

	/*use default mode(ldo bypass) if no param from cmdline*/
	if (enable_ldo_mode == LDO_MODE_DEFAULT)
		enable_ldo_mode = LDO_MODE_ENABLED;

	/*read Device ID*/
	ret = pfuze_reg_read(pfuze, PFUZE100_DEVICEID, &value);
	if (ret)
		goto err;
	if (value != 0x10) {
		printk(KERN_ERR "wrong device id:%x!\n", value);
		goto err;
	}

	/*read Revision ID*/
	ret = pfuze_reg_read(pfuze, PFUZE100_REVID, &value);
	if (ret)
		goto err;
	if (value == 0x10) {
		printk(KERN_WARNING "PF100 1.0 chip found!\n");
	/* workaround ER1 of pfuze1.0: set all buck regulators in PWM mode
	* except SW1C(APS) in normal and  PFM mode in standby.
	*/
		for (i = 0; i < 7; i++) {
			if (i == 2)/*SW1C*/
				value = 0xc;/*normal:APS mode;standby:PFM mode*/
			else
				value = 0xd;/*normal:PWM mode;standby:PFM mode*/
			ret = pfuze_reg_write(pfuze,
					PFUZE100_SW1AMODE + (i * 7),
					value);
			if (ret)
				goto err;
		}

	} else {
	/*set all switches APS in normal and PFM mode in standby*/
		for (i = 0; i < 7; i++) {
			value = 0xc;
			ret = pfuze_reg_write(pfuze,
					PFUZE100_SW1AMODE + (i * 7),
					value);
			if (ret)
				goto err;
		}

	}
	/*use ldo active mode if use 1.2GHz,otherwise use ldo bypass mode*/
	if (arm_max_freq == CPU_AT_1_2GHz) {
			/*VDDARM_IN 1.425*/
		ret = pfuze_reg_rmw(pfuze, PFUZE100_SW1AVOL,
					PFUZE100_SW1AVOL_VSEL_M,
					0x2d);
		if (ret)
			goto err;
		/*VDDSOC_IN 1.425V*/
		ret = pfuze_reg_rmw(pfuze, PFUZE100_SW1CVOL,
					PFUZE100_SW1CVOL_VSEL_M,
					0x2d);
		if (ret)
			goto err;
		enable_ldo_mode = LDO_MODE_ENABLED;
	} else if (enable_ldo_mode == LDO_MODE_BYPASSED) {
		/*decrease VDDARM_IN/VDDSOC_IN,since we will use ldo bypass mode*/
		/*VDDARM_IN 1.3V*/
		ret = pfuze_reg_rmw(pfuze, PFUZE100_SW1AVOL,
					PFUZE100_SW1AVOL_VSEL_M,
					0x28);
		if (ret)
			goto err;
		/*VDDSOC_IN 1.3V*/
		ret = pfuze_reg_rmw(pfuze, PFUZE100_SW1CVOL,
					PFUZE100_SW1CVOL_VSEL_M,
					0x28);
		if (ret)
			goto err;
		/*set SW1AB/1C DVSPEED as 25mV step each 4us,quick than 16us before.*/
		ret = pfuze_reg_rmw(pfuze, PFUZE100_SW1ACON,
				    PFUZE100_SW1ACON_SPEED_M,
				    PFUZE100_SW1ACON_SPEED_VAL);
		if (ret)
			goto err;
		ret = pfuze_reg_rmw(pfuze, PFUZE100_SW1CCON,
				    PFUZE100_SW1CCON_SPEED_M,
				    PFUZE100_SW1CCON_SPEED_VAL);
		if (ret)
			goto err;
	} else if (enable_ldo_mode != LDO_MODE_BYPASSED) {
		/*Increase VDDARM_IN/VDDSOC_IN to 1.375V in ldo active mode*/
		ret = pfuze_reg_rmw(pfuze, PFUZE100_SW1AVOL,
					PFUZE100_SW1AVOL_VSEL_M,
					0x2b);
		if (ret)
			goto err;
		ret = pfuze_reg_rmw(pfuze, PFUZE100_SW1CVOL,
					PFUZE100_SW1CVOL_VSEL_M,
					0x2b);
		if (ret)
			goto err;
	}
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

int __init mx6q_sabreauto_init_pfuze100(u32 int_gpio)
{
	pfuze100_i2c_device.irq = gpio_to_irq(int_gpio); /*update INT gpio */
	return i2c_register_board_info(1, &pfuze100_i2c_device, 1);
}
