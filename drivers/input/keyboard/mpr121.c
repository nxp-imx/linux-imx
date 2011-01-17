/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 */

/*
 * mpr121.c - Touchkey driver for Freescale MPR121 Capacitive Touch
 * Sensor Controllor
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c/mpr.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/bitops.h>

struct mpr121_touchkey_data {
	struct i2c_client	*client;
	struct input_dev	*input_dev;
	unsigned int		key_val;
	int			statusbits;
	int			keycount;
	u16			keycodes[MPR121_MAX_KEY_COUNT];
};

struct mpr121_init_register {
	int addr;
	u8 val;
};

static struct mpr121_init_register init_reg_table[] = {
	{MHD_RISING_ADDR,	0x1},
	{NHD_RISING_ADDR,	0x1},
	{NCL_RISING_ADDR,	0x0},
	{FDL_RISING_ADDR,	0x0},
	{MHD_FALLING_ADDR,	0x1},
	{NHD_FALLING_ADDR,	0x1},
	{NCL_FALLING_ADDR,	0xff},
	{FDL_FALLING_ADDR,	0x02},
	{FILTER_CONF_ADDR,	0x04},
	{AFE_CONF_ADDR,		0x0b},
	{AUTO_CONFIG_CTRL_ADDR, 0x0b},
};

static irqreturn_t mpr_touchkey_interrupt(int irq, void *dev_id)
{
	struct mpr121_touchkey_data *data = dev_id;
	struct i2c_client *client = data->client;
	struct input_dev *input = data->input_dev;
	unsigned int key_num, pressed;
	int reg;

	reg = i2c_smbus_read_byte_data(client, ELE_TOUCH_STATUS_1_ADDR);
	if (reg < 0) {
		dev_err(&client->dev, "i2c read error [%d]\n", reg);
		goto out;
	}

	reg <<= 8;
	reg |= i2c_smbus_read_byte_data(client, ELE_TOUCH_STATUS_0_ADDR);
	if (reg < 0) {
		dev_err(&client->dev, "i2c read error [%d]\n", reg);
		goto out;
	}

	reg &= TOUCH_STATUS_MASK;
	/* use old press bit to figure out which bit changed */
	key_num = ffs(reg ^ data->statusbits) - 1;
	/* use the bit check the press status */
	pressed = (reg & (1 << (key_num))) >> key_num;
	data->statusbits = reg;
	data->key_val = data->keycodes[key_num];

	input_report_key(input, data->key_val, pressed);
	input_sync(input);

	dev_dbg(&client->dev, "key %d %d %s\n", key_num, data->key_val,
		pressed ? "pressed" : "released");

out:
	return IRQ_HANDLED;
}

static int mpr121_phys_init(struct mpr121_platform_data *pdata,
			    struct mpr121_touchkey_data *data,
			    struct i2c_client *client)
{
	struct mpr121_init_register *reg;
	unsigned char usl, lsl, tl;
	int i, t, vdd, ret;

	/* setup touch/release threshold for ele0-ele11 */
	for (i = 0; i <= MPR121_MAX_KEY_COUNT; i++) {
		t = ELE0_TOUCH_THRESHOLD_ADDR + (i * 2);
		ret = i2c_smbus_write_byte_data(client, t, TOUCH_THRESHOLD);
		if (ret < 0)
			goto err_i2c_write;
		ret = i2c_smbus_write_byte_data(client, t + 1,
						RELEASE_THRESHOLD);
		if (ret < 0)
			goto err_i2c_write;
	}
	/* setup init register */
	for (i = 0; i < ARRAY_SIZE(init_reg_table); i++) {
		reg = &init_reg_table[i];
		ret = i2c_smbus_write_byte_data(client, reg->addr, reg->val);
		if (ret < 0)
			goto err_i2c_write;
	}
	/* setup auto-register by vdd,the formula please ref:AN3889 */
	vdd = pdata->vdd_uv / 1000;
	usl = ((vdd - 700) * 256) / vdd;
	lsl = (usl * 65) / 100;
	tl = (usl * 90) / 100;
	ret = i2c_smbus_write_byte_data(client, AUTO_CONFIG_USL_ADDR, usl);
	if (ret < 0)
		goto err_i2c_write;
	ret = i2c_smbus_write_byte_data(client, AUTO_CONFIG_LSL_ADDR, lsl);
	if (ret < 0)
		goto err_i2c_write;
	ret = i2c_smbus_write_byte_data(client, AUTO_CONFIG_TL_ADDR, tl);
	if (ret < 0)
		goto err_i2c_write;
	ret = i2c_smbus_write_byte_data(client, ELECTRODE_CONF_ADDR,
					data->keycount);
	if (ret < 0)
		goto err_i2c_write;

	dev_info(&client->dev, "mpr121: config as enable %x of electrode.\n",
		 data->keycount);

	return 0;

err_i2c_write:
	dev_err(&client->dev, "i2c write error[%d]\n", ret);
	return ret;
}

static int __devinit mpr_touchkey_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct mpr121_platform_data *pdata;
	struct mpr121_touchkey_data *data;
	struct input_dev *input_dev;
	int error;
	int i;

	pdata = client->dev.platform_data;
	if (!pdata) {
		dev_err(&client->dev, "no platform data defined\n");
		return -EINVAL;
	}

	data = kzalloc(sizeof(struct mpr121_touchkey_data), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!data || !input_dev) {
		dev_err(&client->dev, "Falied to allocate memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	data->client = client;
	data->input_dev = input_dev;
	data->keycount = pdata->keycount;

	if (data->keycount > MPR121_MAX_KEY_COUNT) {
		dev_err(&client->dev, "Too many key defined\n");
		error = -EINVAL;
		goto err_free_mem;
	}

	error = mpr121_phys_init(pdata, data, client);
	if (error < 0) {
		dev_err(&client->dev, "Failed to init register\n");
		goto err_free_mem;
	}

	i2c_set_clientdata(client, data);

	input_dev->name = "FSL MPR121 Touchkey";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	input_dev->keycode = pdata->matrix;
	input_dev->keycodesize = sizeof(pdata->matrix[0]);
	input_dev->keycodemax = data->keycount;

	for (i = 0; i < input_dev->keycodemax; i++) {
		__set_bit(pdata->matrix[i], input_dev->keybit);
		data->keycodes[i] = pdata->matrix[i];
		input_set_capability(input_dev, EV_KEY, pdata->matrix[i]);
	}
	input_set_drvdata(input_dev, data);

	error = request_threaded_irq(client->irq, NULL,
				     mpr_touchkey_interrupt,
				     IRQF_TRIGGER_FALLING,
				     client->dev.driver->name, data);
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_free_mem;
	}

	error = input_register_device(input_dev);
	if (error)
		goto err_free_irq;
	i2c_set_clientdata(client, data);
	device_init_wakeup(&client->dev, pdata->wakeup);
	dev_info(&client->dev, "Mpr121 touch keyboard init success.\n");
	return 0;

err_free_irq:
	free_irq(client->irq, data);
err_free_mem:
	input_free_device(input_dev);
	kfree(data);
	return error;
}

static int __devexit mpr_touchkey_remove(struct i2c_client *client)
{
	struct mpr121_touchkey_data *data = i2c_get_clientdata(client);

	free_irq(client->irq, data);
	input_unregister_device(data->input_dev);
	kfree(data);

	return 0;
}

#ifdef CONFIG_PM
static int mpr_suspend(struct i2c_client *client, pm_message_t mesg)
{
	if (device_may_wakeup(&client->dev))
		enable_irq_wake(client->irq);

	i2c_smbus_write_byte_data(client, ELECTRODE_CONF_ADDR, 0x00);

	return 0;
}

static int mpr_resume(struct i2c_client *client)
{
	struct mpr121_touchkey_data *data = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev))
		disable_irq_wake(client->irq);

	i2c_smbus_write_byte_data(client, ELECTRODE_CONF_ADDR, data->keycount);

	return 0;
}
#else
static int mpr_suspend(struct i2c_client *client, pm_message_t mesg) {}
static int mpr_resume(struct i2c_client *client) {}
#endif

static const struct i2c_device_id mpr121_id[] = {
	{"mpr121_touchkey", 0},
	{ }
};

static struct i2c_driver mpr_touchkey_driver = {
	.driver = {
		.name = "mpr121",
		.owner = THIS_MODULE,
	},
	.id_table = mpr121_id,
	.probe	= mpr_touchkey_probe,
	.remove = __devexit_p(mpr_touchkey_remove),
	.suspend = mpr_suspend,
	.resume = mpr_resume,
};

static int __init mpr_touchkey_init(void)
{
	return i2c_add_driver(&mpr_touchkey_driver);
}

static void __exit mpr_touchkey_exit(void)
{
	i2c_del_driver(&mpr_touchkey_driver);
}

module_init(mpr_touchkey_init);
module_exit(mpr_touchkey_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Touch Key driver for FSL MPR121 Chip");
