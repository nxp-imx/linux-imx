/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * mx51-babbage-pmic-mc13892.c  --  i.MX51 Babbage Driver for Atlas MC13892 PMIC
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/pmic_external.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/mc13892.h>
#include <mach/irqs.h>
#include <mach/hardware.h>
#include <linux/gpio.h>
#include <mach/iomux-mx51.h>

/*
 * Convenience conversion.
 * Here atm, maybe there is somewhere better for this.
 */
#define mV_to_uV(mV) (mV * 1000)
#define uV_to_mV(uV) (uV / 1000)
#define V_to_uV(V) (mV_to_uV(V * 1000))
#define uV_to_V(uV) (uV_to_mV(uV) / 1000)

/* CPU */
static struct regulator_consumer_supply sw1_consumers[] = {
	{
		.supply = "cpu_vcc",
	}
};

static struct regulator_consumer_supply vdig_consumers[] = {
	{
		/* sgtl5000 */
		.supply = "VDDA",
		.dev_name = "1-000a",
	},
};

static struct regulator_consumer_supply vvideo_consumers[] = {
	{
		/* sgtl5000 */
		.supply = "VDDIO",
		.dev_name = "1-000a",
	},
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
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
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
	}
};

static struct regulator_init_data vdig_init = {
	.constraints = {
		.name = "VDIG",
		.min_uV = mV_to_uV(1650),
		.max_uV = mV_to_uV(1650),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
		.boot_on = 1,
	},
	.num_consumer_supplies = ARRAY_SIZE(vdig_consumers),
	.consumer_supplies = vdig_consumers,
};

static struct regulator_init_data vpll_init = {
	.constraints = {
		.name = "VPLL",
		.min_uV = mV_to_uV(1050),
		.max_uV = mV_to_uV(1800),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
		.boot_on = 1,
		.always_on = 1,
	}
};

static struct regulator_init_data vusb2_init = {
	.constraints = {
		.name = "VUSB2",
		.min_uV = mV_to_uV(2400),
		.max_uV = mV_to_uV(2775),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
		.boot_on = 1,
		.always_on = 1,
	}
};

static struct regulator_init_data vvideo_init = {
	.constraints = {
		.name = "VVIDEO",
		.min_uV = mV_to_uV(2775),
		.max_uV = mV_to_uV(2775),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
		.apply_uV = 1,
	},
	.num_consumer_supplies = ARRAY_SIZE(vvideo_consumers),
	.consumer_supplies = vvideo_consumers,
};

static struct regulator_init_data vaudio_init = {
	.constraints = {
		.name = "VAUDIO",
		.min_uV = mV_to_uV(2300),
		.max_uV = mV_to_uV(3000),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
	}
};

static struct regulator_init_data vsd_init = {
	.constraints = {
		.name = "VSD",
		.min_uV = mV_to_uV(1800),
		.max_uV = mV_to_uV(3150),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
	}
};

static struct regulator_init_data vcam_init = {
	.constraints = {
		.name = "VCAM",
		.min_uV = mV_to_uV(2500),
		.max_uV = mV_to_uV(3000),
		.valid_ops_mask =
			REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE |
			REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_FAST | REGULATOR_MODE_NORMAL,
	}
};

static struct regulator_init_data vgen1_init = {
	.constraints = {
		.name = "VGEN1",
		.min_uV = mV_to_uV(1200),
		.max_uV = mV_to_uV(1200),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
	}
};

static struct regulator_init_data vgen2_init = {
	.constraints = {
		.name = "VGEN2",
		.min_uV = mV_to_uV(1200),
		.max_uV = mV_to_uV(3150),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
		.always_on = 1,
	}
};

static struct regulator_init_data vgen3_init = {
	.constraints = {
		.name = "VGEN3",
		.min_uV = mV_to_uV(1800),
		.max_uV = mV_to_uV(2900),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
		.always_on = 1,
	}
};

static struct regulator_init_data gpo1_init = {
	.constraints = {
		.name = "GPO1",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	}
};

static struct regulator_init_data gpo2_init = {
	.constraints = {
		.name = "GPO2",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	}
};

static struct regulator_init_data gpo3_init = {
	.constraints = {
		.name = "GPO3",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	}
};

static struct regulator_init_data gpo4_init = {
	.constraints = {
		.name = "GPO4",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	}
};

static struct mc13xxx_regulator_init_data mx51_bbg_regulators[] = {
	{ .id = MC13892_SW1,		.init_data =  &sw1_init },
	{ .id = MC13892_SW2,		.init_data =  &sw2_init },
	{ .id = MC13892_SW3,		.init_data =  &sw3_init },
	{ .id = MC13892_SW4,		.init_data =  &sw4_init },
	{ .id = MC13892_SWBST,		.init_data =  &swbst_init },
	{ .id = MC13892_VIOHI,		.init_data =  &viohi_init },
	{ .id = MC13892_VPLL,		.init_data =  &vpll_init },
	{ .id = MC13892_VDIG,		.init_data =  &vdig_init },
	{ .id = MC13892_VSD,		.init_data =  &vsd_init },
	{ .id = MC13892_VUSB2,		.init_data =  &vusb2_init },
	{ .id = MC13892_VVIDEO,		.init_data =  &vvideo_init },
	{ .id = MC13892_VAUDIO,		.init_data =  &vaudio_init },
	{ .id = MC13892_VCAM,		.init_data =  &vcam_init },
	{ .id = MC13892_VGEN1,		.init_data =  &vgen1_init },
	{ .id = MC13892_VGEN2,		.init_data =  &vgen2_init },
	{ .id = MC13892_VGEN3,		.init_data =  &vgen3_init },
	{ .id = MC13892_VUSB,		.init_data =  &vusb_init },
	{ .id = MC13892_GPO1,		.init_data =  &gpo1_init },
	{ .id = MC13892_GPO2,		.init_data =  &gpo2_init },
	{ .id = MC13892_GPO3,		.init_data =  &gpo3_init },
	{ .id = MC13892_GPO4,		.init_data =  &gpo4_init },
};

static struct mc13xxx_platform_data mc13892_plat = {
	.flags = MC13XXX_USE_REGULATOR,
	.num_regulators = ARRAY_SIZE(mx51_bbg_regulators),
	.regulators = mx51_bbg_regulators,
};

#define MC13892_SPI_IRQ		gpio_to_irq(8)
static struct spi_board_info __initdata mc13892_spi_device = {
	.modalias = "mc13892",
	.irq = MC13892_SPI_IRQ,
	.max_speed_hz = 6000000,	/* max spi SCK clock speed in HZ */
	.bus_num = 0,
	.chip_select = 0,
	.platform_data = &mc13892_plat,
};


int __init mx51_babbage_init_mc13892(void)
{
	return spi_register_board_info(&mc13892_spi_device, 1);
}

