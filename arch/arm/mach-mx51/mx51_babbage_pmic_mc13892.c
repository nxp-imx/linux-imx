/*
 * mx51-3stack-pmic-mc13892.c  --  i.MX51 3STACK Driver for Atlas MC13892 PMIC
 */
 /*
  * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/mfd/mc13892/core.h>
#include <mach/irqs.h>
#include "iomux.h"

/*
 * Convenience conversion.
 * Here atm, maybe there is somewhere better for this.
 */
#define mV_to_uV(mV) (mV * 1000)
#define uV_to_mV(uV) (uV / 1000)
#define V_to_uV(V) (mV_to_uV(V * 1000))
#define uV_to_V(uV) (uV_to_mV(uV) / 1000)

/* Coin cell charger enable */
#define CIONCHEN_LSH	23
#define CIONCHEN_WID	1
/* Coin cell charger voltage setting */
#define VCOIN_LSH	20
#define VCOIN_WID	3

/* Coin Charger voltage */
#define VCOIN_2_5V	0x0
#define VCOIN_2_7V	0x1
#define VCOIN_2_8V	0x2
#define VCOIN_2_9V	0x3
#define VCOIN_3_0V	0x4
#define VCOIN_3_1V	0x5
#define VCOIN_3_2V	0x6
#define VCOIN_3_3V	0x7

/* Keeps VSRTC and CLK32KMCU on for all states */
#define DRM_LSH 4
#define DRM_WID 1

/* regulator standby mask */
#define GEN1_STBY_MASK		(1 << 1)
#define IOHI_STBY_MASK		(1 << 4)
#define DIG_STBY_MASK		(1 << 10)
#define GEN2_STBY_MASK		(1 << 13)
#define PLL_STBY_MASK		(1 << 16)
#define USB2_STBY_MASK		(1 << 19)

#define GEN3_STBY_MASK		(1 << 1)
#define CAM_STBY_MASK		(1 << 7)
#define VIDEO_STBY_MASK		(1 << 13)
#define AUDIO_STBY_MASK		(1 << 16)
#define SD_STBY_MASK		(1 << 19)

/* 0x92412 */
#define REG_MODE_0_ALL_MASK	(GEN1_STBY_MASK |\
				DIG_STBY_MASK | GEN2_STBY_MASK |\
				PLL_STBY_MASK | USB2_STBY_MASK)
/* 0x92082 */
#define REG_MODE_1_ALL_MASK	(GEN3_STBY_MASK | CAM_STBY_MASK |\
				VIDEO_STBY_MASK | AUDIO_STBY_MASK |\
				SD_STBY_MASK)

/* CPU */
static struct regulator_consumer_supply sw1_consumers[] = {
	{
		.supply = "cpu_vcc",
	}
};

struct mc13892;

static struct regulator_init_data sw1_init = {
	.constraints = {
		.name = "SW1",
		.min_uV = mV_to_uV(600),
		.max_uV = mV_to_uV(1375),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = 0,
		.always_on = 1,
		.boot_on = 1,
		.initial_state = PM_SUSPEND_MEM,
		.state_mem = {
			.uV = 850000,
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
		.min_uV = mV_to_uV(900),
		.max_uV = mV_to_uV(1850),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.always_on = 1,
		.boot_on = 1,
		.initial_state = PM_SUSPEND_MEM,
		.state_mem = {
			.uV = 950000,
			.mode = REGULATOR_MODE_NORMAL,
			.enabled = 1,
		},
	}
};

static struct regulator_init_data sw3_init = {
	.constraints = {
		.name = "SW3",
		.min_uV = mV_to_uV(1100),
		.max_uV = mV_to_uV(1850),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.always_on = 1,
		.boot_on = 1,
	}
};

static struct regulator_init_data sw4_init = {
	.constraints = {
		.name = "SW4",
		.min_uV = mV_to_uV(1100),
		.max_uV = mV_to_uV(1850),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.always_on = 1,
		.boot_on = 1,
	}
};

static struct regulator_init_data viohi_init = {
	.constraints = {
		.name = "VIOHI",
		.boot_on = 1,
	}
};

static struct regulator_init_data vusb_init = {
	.constraints = {
		.name = "VUSB",
		.boot_on = 1,
	}
};

static struct regulator_init_data swbst_init = {
	.constraints = {
		.name = "SWBST",
	}
};

static struct regulator_init_data vdig_init = {
	.constraints = {
		.name = "VDIG",
		.min_uV = mV_to_uV(1050),
		.max_uV = mV_to_uV(1800),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.boot_on = 1,
	}
};

static struct regulator_init_data vpll_init = {
	.constraints = {
		.name = "VPLL",
		.min_uV = mV_to_uV(1050),
		.max_uV = mV_to_uV(1800),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.boot_on = 1,
	}
};

static struct regulator_init_data vusb2_init = {
	.constraints = {
		.name = "VUSB2",
		.min_uV = mV_to_uV(2400),
		.max_uV = mV_to_uV(2775),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.boot_on = 1,
	}
};

static struct regulator_init_data vvideo_init = {
	.constraints = {
		.name = "VVIDEO",
		.min_uV = mV_to_uV(2775),
		.max_uV = mV_to_uV(2775),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.always_on = 1,
		.apply_uV =1,
	}
};

static struct regulator_init_data vaudio_init = {
	.constraints = {
		.name = "VAUDIO",
		.min_uV = mV_to_uV(2300),
		.max_uV = mV_to_uV(3000),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	}
};

static struct regulator_init_data vsd_init = {
	.constraints = {
		.name = "VSD",
		.min_uV = mV_to_uV(1800),
		.max_uV = mV_to_uV(3150),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	}
};

static struct regulator_init_data vcam_init = {
	.constraints = {
		.name = "VCAM",
		.min_uV = mV_to_uV(2500),
		.max_uV = mV_to_uV(3000),
		.valid_ops_mask =
			REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE,
		.valid_modes_mask = REGULATOR_MODE_FAST | REGULATOR_MODE_NORMAL,
	}
};

static struct regulator_init_data vgen1_init = {
	.constraints = {
		.name = "VGEN1",
		.min_uV = mV_to_uV(1200),
		.max_uV = mV_to_uV(3150),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	}
};

static struct regulator_init_data vgen2_init = {
	.constraints = {
		.name = "VGEN2",
		.min_uV = mV_to_uV(1200),
		.max_uV = mV_to_uV(3150),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	}
};

static struct regulator_init_data vgen3_init = {
	.constraints = {
		.name = "VGEN3",
		.min_uV = mV_to_uV(1800),
		.max_uV = mV_to_uV(2900),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	}
};

static struct regulator_init_data gpo1_init = {
	.constraints = {
		.name = "GPO1",
	}
};

static struct regulator_init_data gpo2_init = {
	.constraints = {
		.name = "GPO2",
	}
};

static struct regulator_init_data gpo3_init = {
	.constraints = {
		.name = "GPO3",
	}
};

static struct regulator_init_data gpo4_init = {
	.constraints = {
		.name = "GPO4",
	}
};

static int mc13892_regulator_init(struct mc13892 *mc13892)
{
	unsigned int value, register_mask;
	printk("Initializing regulators for Babbage.\n");
	if (mxc_cpu_is_rev(CHIP_REV_2_0) < 0)
		sw2_init.constraints.state_mem.uV = 1100000;
	else if (mxc_cpu_is_rev(CHIP_REV_2_0) == 1) {
		sw2_init.constraints.state_mem.uV = 1250000;
		sw1_init.constraints.state_mem.uV = 1000000;
	}

	/* enable standby controll for all regulators */
	pmic_read_reg(REG_MODE_0, &value, 0xffffff);
	value |= REG_MODE_0_ALL_MASK;
	pmic_write_reg(REG_MODE_0, value, 0xffffff);

	pmic_read_reg(REG_MODE_1, &value, 0xffffff);
	value |= REG_MODE_1_ALL_MASK;
	pmic_write_reg(REG_MODE_1, value, 0xffffff);

	/* Enable coin cell charger */
	value = BITFVAL(CIONCHEN, 1) | BITFVAL(VCOIN, VCOIN_3_0V);
	register_mask = BITFMASK(CIONCHEN) | BITFMASK(VCOIN);
	pmic_write_reg(REG_POWER_CTL0, value, register_mask);

#if defined(CONFIG_RTC_DRV_MXC_V2) || defined(CONFIG_RTC_DRV_MXC_V2_MODULE)
	value = BITFVAL(DRM, 1);
	register_mask = BITFMASK(DRM);
	pmic_write_reg(REG_POWER_CTL0, value, register_mask);
#endif

	mc13892_register_regulator(mc13892, MC13892_SW1, &sw1_init);
	mc13892_register_regulator(mc13892, MC13892_SW2, &sw2_init);
	mc13892_register_regulator(mc13892, MC13892_SW3, &sw3_init);
	mc13892_register_regulator(mc13892, MC13892_SW4, &sw4_init);
	mc13892_register_regulator(mc13892, MC13892_SWBST, &swbst_init);
	mc13892_register_regulator(mc13892, MC13892_VIOHI, &viohi_init);
	mc13892_register_regulator(mc13892, MC13892_VPLL, &vpll_init);
	mc13892_register_regulator(mc13892, MC13892_VDIG, &vdig_init);
	mc13892_register_regulator(mc13892, MC13892_VSD, &vsd_init);
	mc13892_register_regulator(mc13892, MC13892_VUSB2, &vusb2_init);
	mc13892_register_regulator(mc13892, MC13892_VVIDEO, &vvideo_init);
	mc13892_register_regulator(mc13892, MC13892_VAUDIO, &vaudio_init);
	mc13892_register_regulator(mc13892, MC13892_VCAM, &vcam_init);
	mc13892_register_regulator(mc13892, MC13892_VGEN1, &vgen1_init);
	mc13892_register_regulator(mc13892, MC13892_VGEN2, &vgen2_init);
	mc13892_register_regulator(mc13892, MC13892_VGEN3, &vgen3_init);
	mc13892_register_regulator(mc13892, MC13892_VUSB, &vusb_init);
	mc13892_register_regulator(mc13892, MC13892_GPO1, &gpo1_init);
	mc13892_register_regulator(mc13892, MC13892_GPO2, &gpo2_init);
	mc13892_register_regulator(mc13892, MC13892_GPO3, &gpo3_init);
	mc13892_register_regulator(mc13892, MC13892_GPO4, &gpo4_init);

	return 0;
}

static struct mc13892_platform_data mc13892_plat = {
	.init = mc13892_regulator_init,
};

static struct spi_board_info __initdata mc13892_spi_device = {
	.modalias = "pmic_spi",
	.irq = IOMUX_TO_IRQ(MX51_PIN_GPIO1_8),
	.max_speed_hz = 1000000,	/* max spi SCK clock speed in HZ */
	.bus_num = 1,
	.chip_select = 0,
	.platform_data = &mc13892_plat,
};


int __init mx51_babbage_init_mc13892(void)
{
	return spi_register_board_info(&mc13892_spi_device, 1);
}
