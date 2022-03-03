// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * GPIO driver for Analog Devices ADP5585 MFD
 *
 * Copyright 2022 NXP
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mfd/adp5585.h>
#include <linux/gpio/driver.h>

#define ADP5585_GPIO_MAX 10

struct adp5585_gpio_dev {
	struct device *parent;
	struct gpio_chip gpio_chip;
	struct mutex lock;
	u8 dat_out[2];
	u8 dir[2];
};

static int adp5585_gpio_reg_read(struct adp5585_gpio_dev *adp5585_gpio, u8 reg, u8 *val)
{
	struct adp5585_dev *adp5585;
	adp5585 = dev_get_drvdata(adp5585_gpio->parent);

	return adp5585->read_reg(adp5585, reg, val);
}

static int adp5585_gpio_reg_write(struct adp5585_gpio_dev *adp5585_gpio, u8 reg, u8 val)
{
	struct adp5585_dev *adp5585;
	adp5585 = dev_get_drvdata(adp5585_gpio->parent);

	return adp5585->write_reg(adp5585, reg, val);
}

static int adp5585_gpio_get_value(struct gpio_chip *chip, unsigned int off)
{
	struct adp5585_gpio_dev *adp5585_gpio;
	unsigned int bank, bit;
	u8 val;

	adp5585_gpio = gpiochip_get_data(chip);
	bank = ADP5585_BANK(off);
	bit = ADP5585_BIT(off);

	mutex_lock(&adp5585_gpio->lock);
	/* There are dedicated registers for GPIO IN/OUT. */
	if (adp5585_gpio->dir[bank] & bit)
		val = adp5585_gpio->dat_out[bank];
	else
		adp5585_gpio_reg_read(adp5585_gpio, ADP5585_GPI_STATUS_A + bank, &val);
	mutex_unlock(&adp5585_gpio->lock);

	return !!(val & bit);
}

static void adp5585_gpio_set_value(struct gpio_chip *chip, unsigned int off, int val)
{
	struct adp5585_gpio_dev *adp5585_gpio;
	unsigned int bank, bit;

	adp5585_gpio = gpiochip_get_data(chip);
	bank = ADP5585_BANK(off);
	bit = ADP5585_BIT(off);

	mutex_lock(&adp5585_gpio->lock);
	if (val)
		adp5585_gpio->dat_out[bank] |= bit;
	else
		adp5585_gpio->dat_out[bank] &= ~bit;

	adp5585_gpio_reg_write(adp5585_gpio, ADP5585_GPO_DATA_OUT_A + bank,
			       adp5585_gpio->dat_out[bank]);
	mutex_unlock(&adp5585_gpio->lock);
}

static int adp5585_gpio_direction_input(struct gpio_chip *chip, unsigned int off)
{
	struct adp5585_gpio_dev *adp5585_gpio;
	unsigned int bank, bit;
	int ret;

	adp5585_gpio = gpiochip_get_data(chip);
	bank = ADP5585_BANK(off);
	bit = ADP5585_BIT(off);

	mutex_lock(&adp5585_gpio->lock);
	adp5585_gpio->dir[bank] &= ~bit;
	ret = adp5585_gpio_reg_write(adp5585_gpio, ADP5585_GPIO_DIRECTION_A + bank,
				     adp5585_gpio->dir[bank]);
	mutex_unlock(&adp5585_gpio->lock);
	return ret;
}

static int adp5585_gpio_direction_output(struct gpio_chip *chip, unsigned int off, int val)
{
	struct adp5585_gpio_dev *adp5585_gpio;
	unsigned int bank, bit;
	int ret;

	adp5585_gpio = gpiochip_get_data(chip);
	bank = ADP5585_BANK(off);
	bit = ADP5585_BIT(off);

	mutex_lock(&adp5585_gpio->lock);
	adp5585_gpio->dir[bank] |= bit;

	if (val)
		adp5585_gpio->dat_out[bank] |= bit;
	else
		adp5585_gpio->dat_out[bank] &= ~bit;

	ret = adp5585_gpio_reg_write(adp5585_gpio, ADP5585_GPO_DATA_OUT_A + bank,
				     adp5585_gpio->dat_out[bank]);
	ret |= adp5585_gpio_reg_write(adp5585_gpio, ADP5585_GPIO_DIRECTION_A + bank,
				      adp5585_gpio->dir[bank]);
	mutex_unlock(&adp5585_gpio->lock);

	return ret;
}

static int adp5585_gpio_probe(struct platform_device *pdev)
{
	struct adp5585_gpio_dev *adp5585_gpio;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct gpio_chip *gc;
	int i;

	adp5585_gpio = devm_kzalloc(&pdev->dev, sizeof(struct adp5585_gpio_dev), GFP_KERNEL);
	if (adp5585_gpio == NULL)
		return -ENOMEM;

	adp5585_gpio->parent = pdev->dev.parent;

	gc = &adp5585_gpio->gpio_chip;
	gc->of_node = np;
	gc->parent = dev;
	gc->direction_input  = adp5585_gpio_direction_input;
	gc->direction_output = adp5585_gpio_direction_output;
	gc->get = adp5585_gpio_get_value;
	gc->set = adp5585_gpio_set_value;
	gc->can_sleep = true;

	gc->base = -1;
	gc->ngpio = ADP5585_GPIO_MAX;
	gc->label = pdev->name;
	gc->owner = THIS_MODULE;

	mutex_init(&adp5585_gpio->lock);

	for (i = 0; i < 2; i++) {
		u8 *dat_out, *dir;
		dat_out = adp5585_gpio->dat_out;
		dir = adp5585_gpio->dir;
		adp5585_gpio_reg_read(adp5585_gpio,
				      ADP5585_GPO_DATA_OUT_A + i, dat_out + i);
		adp5585_gpio_reg_read(adp5585_gpio,
				      ADP5585_GPIO_DIRECTION_A + i, dir + i);
	}

	return devm_gpiochip_add_data(&pdev->dev, &adp5585_gpio->gpio_chip, adp5585_gpio);
}

static const struct of_device_id adp5585_of_match[] = {
	{.compatible = "adp5585-gpio", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, adp5585_of_match);

static struct platform_driver adp5585_gpio_driver = {
	.driver	= {
		.name	= "adp5585-gpio",
		.of_match_table = adp5585_of_match,
	},
	.probe		= adp5585_gpio_probe,
};

module_platform_driver(adp5585_gpio_driver);

MODULE_AUTHOR("Haibo Chen <haibo.chen@nxp.com>");
MODULE_DESCRIPTION("GPIO ADP5585 Driver");
MODULE_LICENSE("GPL");
