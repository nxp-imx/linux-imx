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
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/mfd/mc9s08dz60/core.h>
#include <linux/mfd/mc9s08dz60/pmic.h>
#include <linux/platform_device.h>
#include <linux/pmic_status.h>

/* lcd */
static int mc9s08dz60_lcd_enable(struct regulator_dev *reg)
{
	return pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 6, 1);
}

static int mc9s08dz60_lcd_disable(struct regulator_dev *reg)
{
	return pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 6, 0);
}

static struct regulator_ops mc9s08dz60_lcd_ops = {
	.enable = mc9s08dz60_lcd_enable,
	.disable = mc9s08dz60_lcd_disable,
};

/* wifi */
static int mc9s08dz60_wifi_enable(struct regulator_dev *reg)
{
	return pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 5, 1);
}

static int mc9s08dz60_wifi_disable(struct regulator_dev *reg)
{
	return pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 5, 0);
}

static struct regulator_ops mc9s08dz60_wifi_ops = {
	.enable = mc9s08dz60_wifi_enable,
	.disable = mc9s08dz60_wifi_disable,
};

/* hdd */
static int mc9s08dz60_hdd_enable(struct regulator_dev *reg)
{
	return pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 4, 1);
}

static int mc9s08dz60_hdd_disable(struct regulator_dev *reg)
{
	return pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 4, 0);
}

static struct regulator_ops mc9s08dz60_hdd_ops = {
	.enable = mc9s08dz60_hdd_enable,
	.disable = mc9s08dz60_hdd_disable,
};

/* gps */
static int mc9s08dz60_gps_enable(struct regulator_dev *reg)
{
	return pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_2, 0, 1);
}

static int mc9s08dz60_gps_disable(struct regulator_dev *reg)
{
	return pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_2, 0, 0);
}

static struct regulator_ops mc9s08dz60_gps_ops = {
	.enable = mc9s08dz60_gps_enable,
	.disable = mc9s08dz60_gps_disable,
};

/* speaker */
static int mc9s08dz60_speaker_enable(struct regulator_dev *reg)
{
	return pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 0, 1);
}

static int mc9s08dz60_speaker_disable(struct regulator_dev *reg)
{
	return pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 0, 0);
}

static struct regulator_ops mc9s08dz60_speaker_ops = {
	.enable = mc9s08dz60_speaker_enable,
	.disable = mc9s08dz60_speaker_disable,
};

static struct regulator_desc mc9s08dz60_reg[] = {
	{
		.name = "LCD",
		.id = MC9S08DZ60_LCD,
		.ops = &mc9s08dz60_lcd_ops,
		.irq = 0,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE
	 },
	{
		.name = "WIFI",
		.id = MC9S08DZ60_WIFI,
		.ops = &mc9s08dz60_wifi_ops,
		.irq = 0,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE
	 },
	{
		.name = "HDD",
		.id = MC9S08DZ60_HDD,
		.ops = &mc9s08dz60_hdd_ops,
		.irq = 0,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE
	 },
	{
		.name = "GPS",
		.id = MC9S08DZ60_GPS,
		.ops = &mc9s08dz60_gps_ops,
		.irq = 0,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE
	 },
	{
		.name = "SPKR",
		.id = MC9S08DZ60_SPKR,
		.ops = &mc9s08dz60_speaker_ops,
		.irq = 0,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE
	 },

};

static int mc9s08dz60_regulator_probe(struct platform_device *pdev)
{
	struct regulator_dev *rdev;

	/* register regulator */
	rdev = regulator_register(&mc9s08dz60_reg[pdev->id], &pdev->dev,
				  pdev->dev.platform_data,
				  dev_get_drvdata(&pdev->dev));
	if (IS_ERR(rdev)) {
		dev_err(&pdev->dev, "failed to register %s\n",
			mc9s08dz60_reg[pdev->id].name);
		return PTR_ERR(rdev);
	}

	return 0;
}


static int mc9s08dz60_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);

	regulator_unregister(rdev);

	return 0;
}

int mc9s08dz60_register_regulator(struct mc9s08dz60 *mc9s08dz60, int reg,
			      struct regulator_init_data *initdata)
{
	struct platform_device *pdev;
	int ret;

	if (mc9s08dz60->pmic.pdev[reg])
		return -EBUSY;

	pdev = platform_device_alloc("mc9s08dz60-regu", reg);
	if (!pdev)
		return -ENOMEM;

	mc9s08dz60->pmic.pdev[reg] = pdev;

	initdata->driver_data = mc9s08dz60;

	pdev->dev.platform_data = initdata;
	pdev->dev.parent = mc9s08dz60->dev;
	platform_set_drvdata(pdev, mc9s08dz60);
	ret = platform_device_add(pdev);

	if (ret != 0) {
		dev_err(mc9s08dz60->dev,
			"Failed to register regulator %d: %d\n",
			reg, ret);
		platform_device_del(pdev);
		mc9s08dz60->pmic.pdev[reg] = NULL;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(mc9s08dz60_register_regulator);

static struct platform_driver mc9s08dz60_regulator_driver = {
	.probe = mc9s08dz60_regulator_probe,
	.remove = mc9s08dz60_regulator_remove,
	.driver		= {
		.name	= "mc9s08dz60-regu",
	},
};

static int __init mc9s08dz60_regulator_init(void)
{
	return platform_driver_register(&mc9s08dz60_regulator_driver);
}
subsys_initcall(mc9s08dz60_regulator_init);

static void __exit mc9s08dz60_regulator_exit(void)
{
	platform_driver_unregister(&mc9s08dz60_regulator_driver);
}
module_exit(mc9s08dz60_regulator_exit);


MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MC9S08DZ60 Regulator driver");
MODULE_LICENSE("GPL");
