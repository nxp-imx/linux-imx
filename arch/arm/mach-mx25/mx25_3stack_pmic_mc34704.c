/*
 * mx25-3stack-pmic-mc34704.c  --  i.MX25 3STACK Driver for MC34704 PMIC
 */
 /*
  * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
  */

 /*
  * The code contained herein is licensed under the GNU General Public
  * License. You may obtain a copy of the GNU General Public License
  * Version 2 or later at the following locations:
  *
  * http://www.opensource.org/licenses/gpl-license.html
  * http://www.gnu.org/copyleft/gpl.html
  */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/pmic_external.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/mc34704/core.h>
#include "iomux.h"

/*
 * Convenience conversion.
 * Here atm, maybe there is somewhere better for this.
 */
#define mV_to_uV(mV) (mV * 1000)
#define uV_to_mV(uV) (uV / 1000)
#define V_to_uV(V) (mV_to_uV(V * 1000))
#define uV_to_V(uV) (uV_to_mV(uV) / 1000)

struct mc34704;

static struct regulator_consumer_supply rcpu_consumers[] = {
	{
		/* sgtl5000 */
		.supply = "VDDA",
		.dev_name = "0-000a",
	},
};

static struct regulator_consumer_supply rddr_consumers[] = {
	{
		/* sgtl5000 */
		.supply = "VDDIO",
		.dev_name = "0-000a",
	},
};

static struct regulator_init_data rbklt_init = {
	.constraints = {
			.name = "REG1_BKLT",
			.min_uV =
			mV_to_uV(REG1_V_MV * (1000 + REG1_DVS_MIN_PCT * 10) /
				 1000),
			.max_uV =
			mV_to_uV(REG1_V_MV * (1000 + REG1_DVS_MAX_PCT * 10) /
				 1000),
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			}
};

static struct regulator_init_data rcpu_init = {
	.constraints = {
			.name = "REG2_CPU",
			.min_uV =
			mV_to_uV(REG2_V_MV * (1000 + REG2_DVS_MIN_PCT * 10) /
				 1000),
			.max_uV =
			mV_to_uV(REG2_V_MV * (1000 + REG2_DVS_MAX_PCT * 10) /
				 1000),
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			},
	.num_consumer_supplies = ARRAY_SIZE(rcpu_consumers),
	.consumer_supplies = rcpu_consumers,
};

static struct regulator_init_data rcore_init = {
	.constraints = {
			.name = "REG3_CORE",
			.min_uV =
			mV_to_uV(REG3_V_MV * (1000 + REG3_DVS_MIN_PCT * 10) /
				 1000),
			.max_uV =
			mV_to_uV(REG3_V_MV * (1000 + REG3_DVS_MAX_PCT * 10) /
				 1000),
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			}
};

static struct regulator_init_data rddr_init = {
	.constraints = {
			.name = "REG4_DDR",
			.min_uV =
			mV_to_uV(REG4_V_MV * (1000 + REG4_DVS_MIN_PCT * 10) /
				 1000),
			.max_uV =
			mV_to_uV(REG4_V_MV * (1000 + REG4_DVS_MAX_PCT * 10) /
				 1000),
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			},
	.num_consumer_supplies = ARRAY_SIZE(rddr_consumers),
	.consumer_supplies = rddr_consumers,
};

static struct regulator_init_data rpers_init = {
	.constraints = {
			.name = "REG5_PERS",
			.min_uV =
			mV_to_uV(REG5_V_MV * (1000 + REG5_DVS_MIN_PCT * 10) /
				 1000),
			.max_uV =
			mV_to_uV(REG5_V_MV * (1000 + REG5_DVS_MAX_PCT * 10) /
				 1000),
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			}
};

static int mc34704_regulator_init(struct mc34704 *mc34704)
{
	mc34704_register_regulator(mc34704, MC34704_BKLT, &rbklt_init);
	mc34704_register_regulator(mc34704, MC34704_CPU, &rcpu_init);
	mc34704_register_regulator(mc34704, MC34704_CORE, &rcore_init);
	mc34704_register_regulator(mc34704, MC34704_DDR, &rddr_init);
	mc34704_register_regulator(mc34704, MC34704_PERS, &rpers_init);

	return 0;
}

static struct mc34704_platform_data mc34704_plat = {
	.init = mc34704_regulator_init,
};

static struct i2c_board_info __initdata mc34704_i2c_device = {
	.type = "mc34704",
	.addr = 0x54,
	.platform_data = &mc34704_plat,
};

int __init mx25_3stack_init_mc34704(void)
{
	return i2c_register_board_info(0, &mc34704_i2c_device, 1);
}
