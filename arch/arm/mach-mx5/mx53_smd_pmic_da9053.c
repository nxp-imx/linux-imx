/*
 * Copyright (C) 2010-2012 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * mx53_smd_pmic_da9053.c  --  i.MX53 SMD driver for pmic da9053
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/da9052/da9052.h>
#include <linux/mfd/da9052/pm.h>
#include <linux/mfd/da9052/led.h>
#include <linux/mfd/da9052/tsi.h>
#include <mach/irqs.h>
#include <mach/iomux-mx53.h>
#include <mach/gpio.h>

#define DA9052_LDO(max, min, rname, suspend_mv, num_consumers, consumers) \
{\
	.constraints = {\
		.name		= (rname), \
		.max_uV		= (max) * 1000,\
		.min_uV		= (min) * 1000,\
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE\
		|REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,\
		.valid_modes_mask = REGULATOR_MODE_NORMAL,\
		.state_mem = { \
			.uV = suspend_mv * 1000, \
			.mode = REGULATOR_MODE_NORMAL, \
			.enabled = (0 == suspend_mv) ? 0 : 1, \
			.disabled = 0, \
		}, \
	},\
	.num_consumer_supplies = (num_consumers), \
	.consumer_supplies = (consumers), \
}

/* CPU */
static struct regulator_consumer_supply sw1_consumers[] = {
	{
		.supply = "cpu_vddgp",
	}
};

/* OV5642 Camera */
static struct regulator_consumer_supply ld07_consumers[] = {
	{
		.supply = "DA9052_LDO7",
	}
};

static struct regulator_consumer_supply ld09_consumers[] = {
	{
		.supply = "DA9052_LDO9",
	}
};

/* HDMI SII902x regulator */
static struct regulator_consumer_supply ld02_consumers[] = {
	{
		.supply = "DA9052_LDO2",
	}
};

/* currently the suspend_mv here takes no effects for DA9053
preset-voltage have to be done in the latest stage during
suspend*/
static struct regulator_init_data da9052_regulators_init[] = {
	DA9052_LDO(DA9052_LDO1_VOLT_UPPER,
		DA9052_LDO1_VOLT_LOWER, "DA9052_LDO1", 1300, 0, NULL),
	DA9052_LDO(DA9052_LDO2_VOLT_UPPER,
		DA9052_LDO2_VOLT_LOWER, "DA9052_LDO2", 1300,
		ARRAY_SIZE(ld02_consumers), ld02_consumers),
	DA9052_LDO(DA9052_LDO34_VOLT_UPPER,
		DA9052_LDO34_VOLT_LOWER, "DA9052_LDO3", 3300, 0, NULL),
	DA9052_LDO(DA9052_LDO34_VOLT_UPPER,
		DA9052_LDO34_VOLT_LOWER, "DA9052_LDO4", 2775, 0, NULL),
	DA9052_LDO(DA9052_LDO567810_VOLT_UPPER,
		DA9052_LDO567810_VOLT_LOWER, "DA9052_LDO5", 1300, 0, NULL),
	DA9052_LDO(DA9052_LDO567810_VOLT_UPPER,
		DA9052_LDO567810_VOLT_LOWER, "DA9052_LDO6", 1200, 0, NULL),
	DA9052_LDO(DA9052_LDO567810_VOLT_UPPER,
		DA9052_LDO567810_VOLT_LOWER, "DA9052_LDO7", 2750,
		ARRAY_SIZE(ld07_consumers), ld07_consumers),
	DA9052_LDO(DA9052_LDO567810_VOLT_UPPER,
		DA9052_LDO567810_VOLT_LOWER, "DA9052_LDO8", 1800, 0, NULL),
	DA9052_LDO(DA9052_LDO9_VOLT_UPPER,
		DA9052_LDO9_VOLT_LOWER, "DA9052_LDO9", 2500,
		ARRAY_SIZE(ld09_consumers), ld09_consumers),
	DA9052_LDO(DA9052_LDO567810_VOLT_UPPER,
		DA9052_LDO567810_VOLT_LOWER, "DA9052_LDO10", 1200, 0, NULL),

	/* BUCKS */
	DA9052_LDO(DA9052_BUCK_CORE_PRO_VOLT_UPPER,
		DA9052_BUCK_CORE_PRO_VOLT_LOWER, "DA9052_BUCK_CORE", 850,
		ARRAY_SIZE(sw1_consumers), sw1_consumers),
	DA9052_LDO(DA9052_BUCK_CORE_PRO_VOLT_UPPER,
		DA9052_BUCK_CORE_PRO_VOLT_LOWER, "DA9052_BUCK_PRO", 950,
		0, NULL),
	DA9052_LDO(DA9052_BUCK_MEM_VOLT_UPPER,
		DA9052_BUCK_MEM_VOLT_LOWER, "DA9052_BUCK_MEM", 1500, 0, NULL),
	DA9052_LDO(DA9052_BUCK_PERI_VOLT_UPPER,
		DA9052_BUCK_PERI_VOLT_LOWER, "DA9052_BUCK_PERI", 2500, 0, NULL)
};

#ifdef CONFIG_SND_SOC_SGTL5000

static struct regulator_consumer_supply sgtl5000_consumer[] = {
	REGULATOR_SUPPLY("VDDA", NULL),
	REGULATOR_SUPPLY("VDDIO", NULL),
	REGULATOR_SUPPLY("VDDD", NULL),
};

static struct regulator_init_data sgtl5000_reg_initdata = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(sgtl5000_consumer),
	.consumer_supplies = &sgtl5000_consumer[0],
};

static struct fixed_voltage_config sgtl5000_reg_config = {
	.supply_name		= "VDDA",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &sgtl5000_reg_initdata,
};

static struct platform_device sgtl5000_reg_devices = {

	.name	= "reg-fixed-voltage",
	.id	= 1,
	.dev	= {
		.platform_data = &sgtl5000_reg_config,
	},
};
#endif /* CONFIG_SND_SOC_SGTL5000 */

static struct regulator_consumer_supply mx53_smd_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.0"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
};

static struct regulator_init_data mx53_smd_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(mx53_smd_vmmc_consumers),
	.consumer_supplies = mx53_smd_vmmc_consumers,
};

static struct fixed_voltage_config mx53_smd_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &mx53_smd_vmmc_init,
};

static struct platform_device mx53_smd_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &mx53_smd_vmmc_reg_config,
	},
};

static struct regulator_consumer_supply mx53_smd_cpu_vddvpu_consumers[] = {
	REGULATOR_SUPPLY("cpu_vddvpu", NULL),
};

static struct regulator_init_data mx53_smd_cpu_vddvpu_init = {
	.num_consumer_supplies = ARRAY_SIZE(mx53_smd_cpu_vddvpu_consumers),
	.consumer_supplies = mx53_smd_cpu_vddvpu_consumers,
};

static struct fixed_voltage_config mx53_smd_cpu_vddvpu_reg_config = {
	.supply_name		= "cpu_vddvpu",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &mx53_smd_cpu_vddvpu_init,
};

static struct platform_device mx53_smd_cpu_vddvpu_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 2,
	.dev	= {
		.platform_data = &mx53_smd_cpu_vddvpu_reg_config,
	},
};

static struct da9052_tsi_platform_data da9052_tsi = {
	.pen_up_interval = 50,
	.tsi_delay_bit_shift = 6,
	.tsi_skip_bit_shift = 3,
	.num_gpio_tsi_register = 3,
	.tsi_supply_voltage = 2500,
	 /* This is the DA9052 LDO number used for powering the TSI */
	.tsi_ref_source = 9,
	.max_tsi_delay = TSI_DELAY_4SLOTS,
	.max_tsi_skip_slot = TSI_SKIP_330SLOTS,
};

static struct da9052_led_platform_data da9052_gpio_led[] = {
	{
		.id = DA9052_LED_4,
		.name = "LED_GPIO14",
	},
	{
		.id = DA9052_LED_5,
		.name = "LED_GPIO15",
	},
};

static struct da9052_leds_platform_data da9052_gpio_leds = {
	.num_leds = ARRAY_SIZE(da9052_gpio_led),
	.led = da9052_gpio_led,
};


static struct da9052_bat_platform_data da9052_bat = {
	.sw_temp_control_en = 0,
	.monitoring_interval = 500,
	.sw_bat_temp_threshold = 60,
	.sw_junc_temp_threshold = 120,
	.hysteresis_window_size = 1,
	.current_monitoring_window = 10,
	.bat_with_no_resistor = 62,
	.bat_capacity_limit_low = 4,
	.bat_capacity_full = 100,
	.bat_capacity_limit_high = 70,
	.chg_hysteresis_const = 89,
	.hysteresis_reading_interval = 1000,
	.hysteresis_no_of_reading = 10,
	.filter_size = 4,
	.bat_volt_cutoff = 2800,
	.vbat_first_valid_detect_iteration = 3,
};

static void da9052_init_ssc_cache(struct da9052 *da9052)
{
	unsigned char cnt;

	/* First initialize all registers as Non-volatile */
	for (cnt = 0; cnt < DA9052_REG_CNT; cnt++) {
		da9052->ssc_cache[cnt].type = NON_VOLATILE;
		da9052->ssc_cache[cnt].status = INVALID;
		da9052->ssc_cache[cnt].val = 0;
	}

	/* Now selectively set type for all Volatile registers */
	/* Reg 1 - 9 */
	da9052->ssc_cache[DA9052_STATUSA_REG].type = VOLATILE;
	da9052->ssc_cache[DA9052_STATUSB_REG].type = VOLATILE;
	da9052->ssc_cache[DA9052_STATUSC_REG].type = VOLATILE;
	da9052->ssc_cache[DA9052_STATUSD_REG].type = VOLATILE;
	da9052->ssc_cache[DA9052_EVENTA_REG].type = VOLATILE;
	da9052->ssc_cache[DA9052_EVENTB_REG].type = VOLATILE;
	da9052->ssc_cache[DA9052_EVENTC_REG].type = VOLATILE;
	da9052->ssc_cache[DA9052_EVENTD_REG].type = VOLATILE;
	da9052->ssc_cache[DA9052_FAULTLOG_REG].type = VOLATILE;

	/* Reg 15 */
	da9052->ssc_cache[DA9052_CONTROLB_REG].type = VOLATILE;
	/* Reg - 17 */
	da9052->ssc_cache[DA9052_CONTROLD_REG].type = VOLATILE;
	/* Reg - 60 */
	da9052->ssc_cache[DA9052_SUPPLY_REG].type = VOLATILE;
	/* Reg - 62 */
	da9052->ssc_cache[DA9052_CHGBUCK_REG].type = VOLATILE;

	/* Reg 67 - 68 */
	da9052->ssc_cache[DA9052_INPUTCONT_REG].type = VOLATILE;
	da9052->ssc_cache[DA9052_CHGTIME_REG].type = VOLATILE;

	/* Reg - 70 */
	da9052->ssc_cache[DA9052_BOOST_REG].type = VOLATILE;

	/* Reg - 81 */
	da9052->ssc_cache[DA9052_ADCMAN_REG].type = VOLATILE;

	/* Reg - 83 - 85 */
	da9052->ssc_cache[DA9052_ADCRESL_REG].type = VOLATILE;
	da9052->ssc_cache[DA9052_ADCRESH_REG].type = VOLATILE;
	da9052->ssc_cache[DA9052_VDDRES_REG].type = VOLATILE;

	/* Reg - 87 */
	da9052->ssc_cache[DA9052_ICHGAV_REG].type = VOLATILE;

	/* Reg - 90 */
	da9052->ssc_cache[DA9052_TBATRES_REG].type = VOLATILE;

	/* Reg - 95 */
	da9052->ssc_cache[DA9052_ADCIN4RES_REG].type = VOLATILE;

	/* Reg - 98 */
	da9052->ssc_cache[DA9052_ADCIN5RES_REG].type = VOLATILE;

	/* Reg - 101 */
	da9052->ssc_cache[DA9052_ADCIN6RES_REG].type = VOLATILE;

	/* Reg - 104 */
	da9052->ssc_cache[DA9052_TJUNCRES_REG].type = VOLATILE;

	/* Reg 106 - 110 */
	da9052->ssc_cache[DA9052_TSICONTB_REG].type	= VOLATILE;
	da9052->ssc_cache[DA9052_TSIXMSB_REG].type	= VOLATILE;
	da9052->ssc_cache[DA9052_TSIYMSB_REG].type	= VOLATILE;
	da9052->ssc_cache[DA9052_TSILSB_REG].type	= VOLATILE;
	da9052->ssc_cache[DA9052_TSIZMSB_REG].type	= VOLATILE;

	/* Reg 111 - 117 */
	da9052->ssc_cache[DA9052_COUNTS_REG].type	= VOLATILE;
	da9052->ssc_cache[DA9052_COUNTMI_REG].type	= VOLATILE;
	da9052->ssc_cache[DA9052_COUNTH_REG].type	= VOLATILE;
	da9052->ssc_cache[DA9052_COUNTD_REG].type	= VOLATILE;
	da9052->ssc_cache[DA9052_COUNTMO_REG].type	= VOLATILE;
	da9052->ssc_cache[DA9052_COUNTY_REG].type	= VOLATILE;
	da9052->ssc_cache[DA9052_ALARMMI_REG].type	= VOLATILE;

	/* Reg 122 - 125 */
	da9052->ssc_cache[DA9052_SECONDA_REG].type	= VOLATILE;
	da9052->ssc_cache[DA9052_SECONDB_REG].type	= VOLATILE;
	da9052->ssc_cache[DA9052_SECONDC_REG].type	= VOLATILE;
	da9052->ssc_cache[DA9052_SECONDD_REG].type	= VOLATILE;

	/* Following addresses are not assigned to any register */
	da9052->ssc_cache[126].type			= VOLATILE;
	da9052->ssc_cache[127].type			= VOLATILE;
}


#define MX53_SMD_DA9052_IRQ			(6*32 + 11)	/* GPIO7_11 */

static int __init smd_da9052_init(struct da9052 *da9052)
{
	/* Configuring for DA9052 interrupt servce */
	/* s3c_gpio_setpull(DA9052_IRQ_PIN, S3C_GPIO_PULL_UP);*/

	/* Set interrupt as LOW LEVEL interrupt source */
	irq_set_irq_type(gpio_to_irq(MX53_SMD_DA9052_IRQ), IRQF_TRIGGER_LOW);

	da9052_init_ssc_cache(da9052);
#ifdef CONFIG_SND_SOC_SGTL5000
	platform_device_register(&sgtl5000_reg_devices);
#endif
	platform_device_register(&mx53_smd_vmmc_reg_devices);
	platform_device_register(&mx53_smd_cpu_vddvpu_reg_devices);

	return 0;
}

static struct da9052_platform_data __initdata da9052_plat = {
	.init = smd_da9052_init,
	.num_regulators = ARRAY_SIZE(da9052_regulators_init),
	.regulators = da9052_regulators_init,
	.led_data = &da9052_gpio_leds,
	.tsi_data = &da9052_tsi,
	.bat_data = &da9052_bat,
	/* .gpio_base = GPIO_BOARD_START, */
};


static struct i2c_board_info __initdata da9052_i2c_device = {
	I2C_BOARD_INFO(DA9052_SSC_I2C_DEVICE_NAME, DA9052_I2C_ADDR >> 1),
	.irq = gpio_to_irq(MX53_SMD_DA9052_IRQ),
	.platform_data = &da9052_plat,
};

int __init mx53_smd_init_da9052(void)
{
	return i2c_register_board_info(0, &da9052_i2c_device, 1);
}
