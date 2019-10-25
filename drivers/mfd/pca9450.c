/*
 * @file pca9450.c  --  NXP PCA9450 mfd driver
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 * Copyright 2019 NXP.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/gpio.h>
#include <linux/regmap.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/mfd/core.h>
#include <linux/mfd/pca9450.h>

/* @brief pca9450 irq resource */
static struct resource pmic_resources[] = {
	{
		.start	= PCA9450_IRQ,
		.end	= PCA9450_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

/* @brief pca9450 multi function cells */
static struct mfd_cell pca9450_mfd_cells[] = {
	{
		.name = "pca9450-pmic",
		.num_resources = ARRAY_SIZE(pmic_resources),
		.resources = &pmic_resources[0],
	},
};

/* @brief pca9450 irqs */
static const struct regmap_irq pca9450_irqs[] = {
	[PCA9450_IRQ] = {
		.mask = PCA9450_INT_MASK,
		.reg_offset = 0,
	},
};

/* @brief pca9450 irq chip definition */
static struct regmap_irq_chip pca9450_irq_chip = {
	.name = "pca9450",
	.irqs = pca9450_irqs,
	.num_irqs = ARRAY_SIZE(pca9450_irqs),
	.num_regs = 1,
	.irq_reg_stride = 1,
	.status_base = PCA9450_INT1,
	.mask_base = PCA9450_INT1_MSK,
	.mask_invert = true,
};

/*
 * @brief pca9450 irq initialize
 * @param pca9450 pca9450 device to init
 * @param bdinfo platform init data
 * @retval 0 probe success
 * @retval negative error number
 */
static int pca9450_irq_init(struct pca9450 *pca9450,
			    struct pca9450_board *bdinfo)
{
	int irq;
	int ret = 0;

	if (!bdinfo) {
		dev_warn(pca9450->dev, "No interrupt support, no pdata\n");
		return -EINVAL;
	}

	dev_info(pca9450->dev, "gpio_intr = %d\n", bdinfo->gpio_intr);
	irq = gpio_to_irq(bdinfo->gpio_intr);

	pca9450->chip_irq = irq;
	dev_info(pca9450->dev, "chip_irq=%d\n", pca9450->chip_irq);
	ret = regmap_add_irq_chip(pca9450->regmap, pca9450->chip_irq,
		IRQF_ONESHOT | IRQF_TRIGGER_FALLING, bdinfo->irq_base,
		&pca9450_irq_chip, &pca9450->irq_data);
	if (ret < 0)
		dev_warn(pca9450->dev, "Failed to add irq_chip %d\n", ret);

	return ret;
}

/*
 * @brief pca9450 irq initialize
 * @param pca9450 pca9450 device to init
 * @retval 0 probe success
 * @retval negative error number
 */
static int pca9450_irq_exit(struct pca9450 *pca9450)
{
	if (pca9450->chip_irq > 0)
		regmap_del_irq_chip(pca9450->chip_irq, pca9450->irq_data);
	return 0;
}

/*
 * @brief check whether volatile register
 * @param dev kernel device pointer
 * @param reg register index
 */
static bool is_volatile_reg(struct device *dev, unsigned int reg)
{
	/*
	 * Caching all regulator registers.
	 */
	return true;
}

/* @brief regmap configures */
static const struct regmap_config pca9450_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.volatile_reg = is_volatile_reg,
	.max_register = PCA9450_MAX_REGISTER - 1,
	.cache_type = REGCACHE_RBTREE,
};

#ifdef CONFIG_OF
static const struct of_device_id pca9450_of_match[] = {
	{ .compatible = "nxp,pca9450", .data = (void *)0},
	{ },
};
MODULE_DEVICE_TABLE(of, pca9450_of_match);

/*
 * @brief parse device tree data of pca9450
 * @param client client object provided by system
 * @param chip_id return chip id back to caller
 * @return board initialize data
 */
static struct pca9450_board *pca9450_parse_dt(struct i2c_client *client,
						int *chip_id)
{
	struct device_node *np = client->dev.of_node;
	struct pca9450_board *board_info;
	unsigned int prop;
	const struct of_device_id *match;
	int r = 0;

	match = of_match_device(pca9450_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev, "Failed to find matching dt id\n");
		return NULL;
	}

	chip_id  = (int *)match->data;

	board_info = devm_kzalloc(&client->dev, sizeof(*board_info),
			GFP_KERNEL);
	if (!board_info)
		return NULL;

	board_info->gpio_intr = of_get_named_gpio(np, "gpio_intr", 0);
	if (!gpio_is_valid(board_info->gpio_intr)) {
		dev_err(&client->dev, "no pmic intr pin available\n");
		goto err_intr;
	}

	r = of_property_read_u32(np, "irq_base", &prop);
	if (!r)
		board_info->irq_base = prop;
	else
		board_info->irq_base = -1;

	return board_info;

err_intr:
	devm_kfree(&client->dev, board_info);
	return NULL;
}
#endif

/*
 * @brief probe pca9450 device
 * @param i2c client object provided by system
 * @param id chip id
 * @retval 0 probe success
 * @retval negative error number
 */
static int pca9450_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct pca9450 *pca9450;
	struct pca9450_board *pmic_plat_data;
	struct pca9450_board *of_pmic_plat_data = NULL;
	int chip_id = id->driver_data;
	int ret = 0;

	pmic_plat_data = dev_get_platdata(&i2c->dev);

	if (!pmic_plat_data && i2c->dev.of_node) {
		pmic_plat_data = pca9450_parse_dt(i2c, &chip_id);
		of_pmic_plat_data = pmic_plat_data;
	}

	if (!pmic_plat_data)
		return -EINVAL;

	pca9450 = kzalloc(sizeof(struct pca9450), GFP_KERNEL);
	if (pca9450 == NULL)
		return -ENOMEM;

	pca9450->of_plat_data = of_pmic_plat_data;
	i2c_set_clientdata(i2c, pca9450);
	pca9450->dev = &i2c->dev;
	pca9450->i2c_client = i2c;
	pca9450->id = chip_id;
	mutex_init(&pca9450->io_mutex);

	pca9450->regmap = devm_regmap_init_i2c(i2c, &pca9450_regmap_config);
	if (IS_ERR(pca9450->regmap)) {
		ret = PTR_ERR(pca9450->regmap);
		dev_err(&i2c->dev, "regmap initialization failed: %d\n", ret);
		return ret;
	}

	ret = pca9450_reg_read(pca9450, PCA9450_REG_DEV_ID);
	if (ret < 0) {
		dev_err(pca9450->dev, "%s(): Read PCA9450_REG_DEVICE failed!\n",
			__func__);
		goto err;
	}
	dev_info(pca9450->dev, "Device ID=0x%X\n", ret);

	pca9450_irq_init(pca9450, of_pmic_plat_data);

	ret = mfd_add_devices(pca9450->dev, -1,
			      pca9450_mfd_cells, ARRAY_SIZE(pca9450_mfd_cells),
			      NULL, 0,
			      regmap_irq_get_domain(pca9450->irq_data));
	if (ret < 0)
		goto err;

	return ret;

err:
	mfd_remove_devices(pca9450->dev);
	kfree(pca9450);
	return ret;
}

/*
 * @brief remove pca9450 device
 * @param i2c client object provided by system
 * @return 0
 */
static int pca9450_i2c_remove(struct i2c_client *i2c)
{
	struct pca9450 *pca9450 = i2c_get_clientdata(i2c);

	pca9450_irq_exit(pca9450);
	mfd_remove_devices(pca9450->dev);
	kfree(pca9450);

	return 0;
}

static const struct i2c_device_id pca9450_i2c_id[] = {
	{ "pca9450", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pca9450_i2c_id);

static struct i2c_driver pca9450_i2c_driver = {
	.driver = {
		.name = "pca9450",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(pca9450_of_match),
	},
	.probe = pca9450_i2c_probe,
	.remove = pca9450_i2c_remove,
	.id_table = pca9450_i2c_id,
};

static int __init pca9450_i2c_init(void)
{
	return i2c_add_driver(&pca9450_i2c_driver);
}
/* init early so consumer devices can complete system boot */
subsys_initcall(pca9450_i2c_init);

static void __exit pca9450_i2c_exit(void)
{
	i2c_del_driver(&pca9450_i2c_driver);
}
module_exit(pca9450_i2c_exit);

MODULE_AUTHOR("John Lee <john.lee@nxp.com>");
MODULE_DESCRIPTION("PCA9450 chip multi-function driver");
MODULE_LICENSE("GPL");
