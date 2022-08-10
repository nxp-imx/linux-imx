/* SPDX-License-Identifier: GPL-2.0-or-later */
/* ADP5585 IO Expander, Key controller core driver.
 *
 * Copyright 2022 NXP
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/mfd/core.h>
#include <linux/mfd/adp5585.h>

static const struct mfd_cell adp5585_devs[] = {
	{
		.name = "adp5585-gpio",
		.of_compatible = "adp5585-gpio",
	},
	{
		.name = "adp5585-pwm",
		.of_compatible = "adp5585-pwm",
	},
};

static int adp5585_i2c_read_reg(struct adp5585_dev *adp5585, u8 reg, u8 *val)
{
	struct i2c_client *i2c = adp5585->i2c_client;
	struct i2c_msg xfer[2];
	int ret;

	/* Write register */
	xfer[0].addr = i2c->addr;
	xfer[0].flags = 0;
	xfer[0].len = 1;
	xfer[0].buf = &reg;

	/* Read data */
	xfer[1].addr = i2c->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = 1;
	xfer[1].buf = val;

	ret = i2c_transfer(i2c->adapter, xfer, 2);
	if (ret == 2) {
		return 0;
	} else {
		dev_err(&i2c->dev,
			"Failed to read reg 0x%02x, ret is %d\n",
			reg, ret);
		if ( ret >=0)
			return -EIO;
		else
			return ret;
	}
}

static int adp5585_i2c_write_reg(struct adp5585_dev *adp5585, u8 reg, u8 val)
{
	struct i2c_client *i2c = adp5585->i2c_client;
	u8 msg[2];
	int ret;

	msg[0] = reg;
	msg[1] = val;
	ret = i2c_master_send(i2c, msg, 2);
	if (ret == 2) {
		return 0;
	} else {
		dev_err(&i2c->dev,
			"Failed to write reg 0x%02x, ret is %d\n",
			reg, ret);
		if ( ret >=0)
			return -EIO;
		else
			return ret;
	}
}

static int adp5585_i2c_probe(struct i2c_client *i2c,
				const struct i2c_device_id *id)
{
	struct adp5585_dev *adp5585;
	u8 reg;
	int ret;

	adp5585 = devm_kzalloc(&i2c->dev, sizeof(struct adp5585_dev),
				GFP_KERNEL);
	if (adp5585 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, adp5585);
	adp5585->dev = &i2c->dev;
	adp5585->i2c_client = i2c;
	adp5585->read_reg = adp5585_i2c_read_reg;
	adp5585->write_reg = adp5585_i2c_write_reg;

	ret = adp5585_i2c_read_reg(adp5585, ADP5585_ID, &reg);
	if (ret)
		return ret;

	return devm_mfd_add_devices(adp5585->dev, PLATFORM_DEVID_AUTO,
				    adp5585_devs, ARRAY_SIZE(adp5585_devs),
				    NULL, 0, NULL);
}

static const struct i2c_device_id adp5585_i2c_id[] = {
	{ "adp5585", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, adp5585_i2c_id);

static const struct of_device_id adp5585_of_match[] = {
	{.compatible = "adi,adp5585", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, adp5585_of_match);

static struct i2c_driver adp5585_i2c_driver = {
	.driver = {
		   .name = "adp5585",
		   .of_match_table = of_match_ptr(adp5585_of_match),
	},
	.probe = adp5585_i2c_probe,
	.id_table = adp5585_i2c_id,
};

static int __init adp5585_i2c_init(void)
{
	return i2c_add_driver(&adp5585_i2c_driver);
}
/* init early so consumer devices can complete system boot */
subsys_initcall(adp5585_i2c_init);

static void __exit adp5585_i2c_exit(void)
{
	i2c_del_driver(&adp5585_i2c_driver);
}
module_exit(adp5585_i2c_exit);

MODULE_DESCRIPTION("ADP5585 core driver");
MODULE_AUTHOR("Haibo Chen <haibo.chen@nxp.com>");
MODULE_LICENSE("GPL");
