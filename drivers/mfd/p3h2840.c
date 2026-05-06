// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025-2026 NXP
 * P3H2X4X i3c hub and regulator device.
 */

#include <linux/i2c.h>
#include <linux/i3c/master.h>
#include <linux/mfd/core.h>
#include <linux/mfd/p3h2840.h>
#include <linux/regmap.h>

static const struct mfd_cell p3h2x4x_devs[] = {
	{
		.name = "p3h2x4x-regulator",
	},
	{
		.name = "p3h2x4x-i3c-hub",
	},
};

static const struct regmap_config p3h2x4x_regmap_config = {
	.reg_bits = P3H2X4X_REG_BITS,
	.val_bits = P3H2X4X_VAL_BITS,
	.max_register = 0xFF,
};

static int p3h2x4x_device_probe_i3c(struct i3c_device *i3cdev)
{
	struct p3h2x4x_dev *p3h2x4x;
	int ret;

	p3h2x4x = devm_kzalloc(&i3cdev->dev, sizeof(*p3h2x4x), GFP_KERNEL);
	if (!p3h2x4x)
		return -ENOMEM;

	i3cdev_set_drvdata(i3cdev, p3h2x4x);

	p3h2x4x->regmap = devm_regmap_init_i3c(i3cdev, &p3h2x4x_regmap_config);
	if (IS_ERR(p3h2x4x->regmap))
		return dev_err_probe(&i3cdev->dev, PTR_ERR(p3h2x4x->regmap),
				     "Failed to register HUB regmap\n");

	p3h2x4x->is_p3h2x4x_in_i3c = true;
	p3h2x4x->i3cdev = i3cdev;

	ret = devm_mfd_add_devices(&i3cdev->dev, PLATFORM_DEVID_AUTO,
				   p3h2x4x_devs, ARRAY_SIZE(p3h2x4x_devs),
				   NULL, 0, NULL);
	if (ret)
		return dev_err_probe(&i3cdev->dev, ret, "Failed to add sub devices\n");

	return 0;
}

static int p3h2x4x_device_probe_i2c(struct i2c_client *client)
{
	struct p3h2x4x_dev *p3h2x4x;
	int ret;

	p3h2x4x = devm_kzalloc(&client->dev, sizeof(*p3h2x4x), GFP_KERNEL);
	if (!p3h2x4x)
		return -ENOMEM;

	i2c_set_clientdata(client, p3h2x4x);

	p3h2x4x->regmap = devm_regmap_init_i2c(client, &p3h2x4x_regmap_config);
	if (IS_ERR(p3h2x4x->regmap))
		return dev_err_probe(&client->dev, PTR_ERR(p3h2x4x->regmap),
				     "Failed to register HUB regmap\n");

	p3h2x4x->is_p3h2x4x_in_i3c = false;

	ret = devm_mfd_add_devices(&client->dev, PLATFORM_DEVID_AUTO,
				   p3h2x4x_devs, ARRAY_SIZE(p3h2x4x_devs),
				   NULL, 0, NULL);
	if (ret)
		return dev_err_probe(&client->dev, ret, "Failed to add sub devices\n");

	return 0;
}

static const struct i3c_device_id p3h2x4x_i3c_ids[] = {
	I3C_CLASS(I3C_DCR_HUB, NULL),
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(i3c, p3h2x4x_i3c_ids);

static const struct i2c_device_id p3h2x4x_i2c_id_table[] = {
	{ "nxp-i3c-hub" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, p3h2x4x_i2c_id_table);

static const struct of_device_id p3h2x4x_i2c_of_match[] = {
	{ .compatible = "nxp,p3h2840", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, p3h2x4x_i2c_of_match);

static struct i3c_driver p3h2x4x_i3c = {
	.driver = {
		.name = "p3h2x4x_i3c_drv",
	},
	.probe = p3h2x4x_device_probe_i3c,
	.id_table = p3h2x4x_i3c_ids,
};

static struct i2c_driver p3h2x4x_i2c = {
	.driver = {
		.name = "p3h2x4x_i2c_drv",
		.of_match_table = p3h2x4x_i2c_of_match,
	},
	.probe =  p3h2x4x_device_probe_i2c,
	.id_table = p3h2x4x_i2c_id_table,
};

module_i3c_i2c_driver(p3h2x4x_i3c, &p3h2x4x_i2c);

MODULE_AUTHOR("Aman Kumar Pandey <aman.kumarpandey@nxp.com>");
MODULE_AUTHOR("Vikash Bansal <vikash.bansal@nxp.com>");
MODULE_AUTHOR("Lakshay Piplani <lakshay.piplani@nxp.com>");
MODULE_DESCRIPTION("NXP P3H2X4X I3C HUB multi function driver");
MODULE_LICENSE("GPL");
