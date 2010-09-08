/*
 * mx3-3stack-pmic-mc13783.c  --  i.MX3 3STACK Driver for Atlas MC13783 PMIC
 */
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/pmic_external.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/mc13783/core.h>
#include <mach/irqs.h>
#include <mach/iomux-mx3.h>

/*
 * Convenience conversion.
 * Here atm, maybe there is somewhere better for this.
 */
#define mV_to_uV(mV) (mV * 1000)

struct mc13783;

static struct regulator_init_data violo_init = {
	.constraints = {
		.min_uV = mV_to_uV(1200), /* mc13783 allows min of 1200. */
		.max_uV = mV_to_uV(1800), /* mc13783 allows max of 1800. */
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.boot_on = 1,
	}
};

static struct regulator_init_data vdig_init = {
	.constraints = {
		.min_uV = mV_to_uV(1200), /* mc13783 allows min of 1200. */
		.max_uV = mV_to_uV(1800), /* mc13783 allows max of 1800. */
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.boot_on = 1,
	}
};

static struct regulator_init_data vgen_init = {
	.constraints = {
		.min_uV = mV_to_uV(1100), /* mc13783 allows min of 1100. */
		.max_uV = mV_to_uV(2775), /* mc13783 allows max of 2775. */
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.boot_on = 1,
	}
};

static struct regulator_init_data vrfdig_init = {
	.constraints = {
		.min_uV = mV_to_uV(1200), /* mc13783 allows min of 1200. */
		.max_uV = mV_to_uV(1875), /* mc13783 allows max of 1875. */
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.boot_on = 1,
	}
};

static struct regulator_init_data vrfref_init = {
	.constraints = {
		.min_uV = mV_to_uV(2475), /* mc13783 allows min of 2475. */
		.max_uV = mV_to_uV(2775), /* mc13783 allows max of 2775. */
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.boot_on = 1,
	}
};

static struct regulator_init_data vrfcp_init = {
	.constraints = {
		.min_uV = mV_to_uV(2700), /* mc13783 allows min of 2700. */
		.max_uV = mV_to_uV(2775), /* mc13783 allows max of 2775. */
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.boot_on = 1,
	}
};

static struct regulator_init_data vsim_init = {
	.constraints = {
		.min_uV = mV_to_uV(1800), /* mc13783 allows min of 1800. */
		.max_uV = mV_to_uV(2900), /* mc13783 allows max of 2900. */
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	}
};

static struct regulator_init_data vesim_init = {
	.constraints = {
		.min_uV = mV_to_uV(1800), /* mc13783 allows min of 1800. */
		.max_uV = mV_to_uV(2900), /* mc13783 allows max of 2900. */
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	}
};

static struct regulator_init_data vcam_init = {
	.constraints = {
		.min_uV = mV_to_uV(1500), /* mc13783 allows min of 1500. */
		.max_uV = mV_to_uV(3000), /* mc13783 allows max of 3000. */
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	}
};

static struct regulator_init_data vvib_init = {
	.constraints = {
		.min_uV = mV_to_uV(1300), /* mc13783 allows min of 1300. */
		.max_uV = mV_to_uV(3000), /* mc13783 allows max of 3000. */
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	}
};

static struct regulator_init_data vrf_init = {
	.constraints = {
		.min_uV = mV_to_uV(1500), /* mc13783 allows min of 1500. */
		.max_uV = mV_to_uV(2775), /* mc13783 allows max of 2775. */
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.boot_on = 1,
	}
};

static struct regulator_init_data vmmc_init = {
	.constraints = {
		.min_uV = mV_to_uV(1600), /* mc13783 allows min of 1600. */
		.max_uV = mV_to_uV(3000), /* mc13783 allows max of 3000. */
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	}
};

static struct regulator_init_data sw3_init = {
	.constraints = {
		.min_uV = mV_to_uV(5000), /* mc13783 allows min of 5000. */
		.max_uV = mV_to_uV(5500), /* mc13783 allows max of 5500. */
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.boot_on = 1,
	}
};

static struct regulator_init_data sw1_init = {
	.constraints = {
		.min_uV = mV_to_uV(1200), /* mc13783 allows min of 900. */
		.max_uV = mV_to_uV(1600), /* mc13783 allows max of 2200. */
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE
				  | REGULATOR_CHANGE_MODE,
		.valid_modes_mask = REGULATOR_MODE_FAST
				    | REGULATOR_MODE_NORMAL
				    | REGULATOR_MODE_IDLE
				    | REGULATOR_MODE_STANDBY,
		.always_on = 1,
		.boot_on = 1,
		.initial_state = PM_SUSPEND_MEM,
		.state_mem = {
			.uV = mV_to_uV(1250),
			.mode = REGULATOR_MODE_NORMAL,
			.enabled = 1,
		},
	}
};

static struct regulator_init_data sw_init = {
	.constraints = {
		.min_uV = mV_to_uV(1200), /* mc13783 allows min of 900. */
		.max_uV = mV_to_uV(2200), /* mc13783 allows max of 2200. */
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.boot_on = 1,
	}
};

static struct regulator_init_data vaudio_init = {
	.constraints = {
		.boot_on = 1,
	}
};

static struct regulator_init_data viohi_init = {
	.constraints = {
		.boot_on = 1,
	}
};

static struct regulator_init_data gpo1_init = {
	.constraints = {
		.boot_on = 1,
	}
};

static struct regulator_init_data gpo4_init = {
	.constraints = {
	},
};

static struct regulator_init_data gpo_init = {
	.constraints = {
	},
};

static int mc13783_regulator_init(void *data)
{
	struct mc13783 *mc13783 = data;
	unsigned int value;
#if 0
	/*most regulators are controled by standby signal*/
	/*except violo*/
	pmic_read_reg(REG_REGULATOR_MODE_0, &value, 0xffffff);
	value |= 0x492412;
	pmic_write_reg(REG_REGULATOR_MODE_0, value, 0xffffff);
	pmic_read_reg(REG_REGULATOR_MODE_1, &value, 0xffffff);
	value |= 0x492492;
	pmic_write_reg(REG_REGULATOR_MODE_1, value, 0xffffff);
	/*also sw3 is controled by standby signal*/
	pmic_read_reg(REG_SWITCHERS_5, &value, 0xffffff);
	value |= 0x200000;
	pmic_write_reg(REG_SWITCHERS_5, value, 0xffffff);
#endif
	mc13783_register_regulator(mc13783, MC13783_SW1A, &sw1_init);
	mc13783_register_regulator(mc13783, MC13783_SW1B, &sw_init);
	mc13783_register_regulator(mc13783, MC13783_SW2A, &sw_init);
	mc13783_register_regulator(mc13783, MC13783_SW2B, &sw_init);
	mc13783_register_regulator(mc13783, MC13783_SW3, &sw3_init);
	mc13783_register_regulator(mc13783, MC13783_VMMC1, &vmmc_init);
	mc13783_register_regulator(mc13783, MC13783_VMMC2, &vmmc_init);
	mc13783_register_regulator(mc13783, MC13783_VVIB, &vvib_init);
	mc13783_register_regulator(mc13783, MC13783_VIOHI, &viohi_init);
	mc13783_register_regulator(mc13783, MC13783_VIOLO, &violo_init);
	mc13783_register_regulator(mc13783, MC13783_VDIG, &vdig_init);
	mc13783_register_regulator(mc13783, MC13783_VRFDIG, &vrfdig_init);
	mc13783_register_regulator(mc13783, MC13783_VRFREF, &vrfref_init);
	mc13783_register_regulator(mc13783, MC13783_VRFCP, &vrfcp_init);
	mc13783_register_regulator(mc13783, MC13783_VRF1, &vrf_init);
	mc13783_register_regulator(mc13783, MC13783_VRF2, &vrf_init);
	mc13783_register_regulator(mc13783, MC13783_VAUDIO, &vaudio_init);
	mc13783_register_regulator(mc13783, MC13783_VCAM, &vcam_init);
	mc13783_register_regulator(mc13783, MC13783_VGEN, &vgen_init);
	mc13783_register_regulator(mc13783, MC13783_VSIM, &vsim_init);
	mc13783_register_regulator(mc13783, MC13783_VESIM, &vesim_init);
	mc13783_register_regulator(mc13783, MC13783_GPO1, &gpo1_init);

	gpo_init.supply_regulator_dev =
			&(mc13783->pmic.pdev[MC13783_GPO1]->dev);
	mc13783_register_regulator(mc13783, MC13783_GPO2, &gpo_init);
	mc13783_register_regulator(mc13783, MC13783_GPO3, &gpo_init);
	mc13783_register_regulator(mc13783, MC13783_GPO4, &gpo4_init);

	return 0;
}

static struct pmic_platform_data mc13783_plat = {
	.init = mc13783_regulator_init,
	.power_key_irq = IOMUX_TO_IRQ(MX31_PIN_GPIO1_2),
};

static struct spi_board_info __initdata mc13783_spi_device = {
	.modalias = "pmic_spi",
	.irq = IOMUX_TO_IRQ(MX31_PIN_GPIO1_3),
	.max_speed_hz = 4000000,
	.bus_num = 2,
	.platform_data = &mc13783_plat,
	.chip_select = 2,
};

int __init mx3_3stack_init_mc13783(void)
{
	return spi_register_board_info(&mc13783_spi_device, 1);
}
