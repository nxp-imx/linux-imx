/*
 * Copyright 2008-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

    /*!
     * @file mc9s08dz60.c
     * @brief Driver for MC9sdz60
     *
     * @ingroup pmic
     */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/proc_fs.h>
#include <linux/i2c.h>
#include <linux/mfd/mc9s08dz60/core.h>

#include <mach/clock.h>
#include <linux/uaccess.h>
#include "mc9s08dz60.h"

/* I2C bus id and device address of mcu */
#define I2C1_BUS	0
#define MC9S08DZ60_I2C_ADDR	0xD2	/* 7bits I2C address */
static struct i2c_client *mc9s08dz60_i2c_client;

int mc9s08dz60_read_reg(u8 reg, u8 *value)
{
	*value = (u8) i2c_smbus_read_byte_data(mc9s08dz60_i2c_client, reg);
	return 0;
}

int mc9s08dz60_write_reg(u8 reg, u8 value)
{
	if (i2c_smbus_write_byte_data(mc9s08dz60_i2c_client, reg, value) < 0)
		return -1;
	return 0;
}

static ssize_t mc9s08dz60_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	unsigned int i;
	u8 value;
	int offset = 7;

	for (i = 0; i < 7; i++) {
		mc9s08dz60_read_reg(i, &value);
		pr_info("reg%02x: %02x\t", i, value);
		mc9s08dz60_read_reg(i + offset, &value);
		pr_info("reg%02x: %02x\t", i + offset, value);
		mc9s08dz60_read_reg(i + offset * 2, &value);
		pr_info("reg%02x: %02x\t", i + offset * 2, value);
		mc9s08dz60_read_reg(i + offset * 3, &value);
		pr_info("reg%02x: %02x\t", i + offset * 3, value);
		mc9s08dz60_read_reg(i + offset * 4, &value);
		pr_info("reg%02x: %02x\t", i + offset * 4, value);
		mc9s08dz60_read_reg(i + offset * 5, &value);
		pr_info("reg%02x: %02x\n", i + offset * 5, value);
	}

	return 0;
}

static ssize_t mc9s08dz60_store(struct device *dev,
			     struct device_attribute *attr, const char *buf,
			     size_t count)
{
	int ret;
	unsigned long reg, new_value;
	u8 value;
	char *p;

	strict_strtoul(buf, 16, &reg);

	p = NULL;
	p = memchr(buf, ' ', count);

	if (p == NULL) {
		mc9s08dz60_read_reg(reg, &value);
		pr_info("reg%02lu: %06x\n", reg, value);
		return count;
	}

	p += 1;

	strict_strtoul(p, 16, &new_value);
	value = new_value;

	ret = mc9s08dz60_write_reg((u8)reg, value);
	if (ret == 0)
		pr_info("write reg%02lx: %06x\n", reg, value);
	else
		pr_info("register update failed\n");

	return count;
}

static struct device_attribute mc9s08dz60_dev_attr = {
	.attr = {
		 .name = "mc9s08dz60_ctl",
		 .mode = S_IRUSR | S_IWUSR,
		 },
	.show = mc9s08dz60_show,
	.store = mc9s08dz60_store,
};


/*!
 * mc9s08dz60 I2C attach function
 *
 * @param adapter            struct i2c_adapter *
 * @return  0
 */
static int mc9s08dz60_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret = 0;
	struct mc9s08dz60 *mc9s08dz60 = NULL;
	struct mc9s08dz60_platform_data *plat_data = client->dev.platform_data;
	pr_info("mc9s08dz60 probing .... \n");

	mc9s08dz60 = kzalloc(sizeof(struct mc9s08dz60), GFP_KERNEL);
	if (mc9s08dz60 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(client, mc9s08dz60);
	mc9s08dz60->dev = &client->dev;
	mc9s08dz60->i2c_client = client;

	if (plat_data && plat_data->init) {
		ret = plat_data->init(mc9s08dz60);
		if (ret != 0)
			return -1;
	}

	ret = device_create_file(&client->dev, &mc9s08dz60_dev_attr);
	if (ret)
		dev_err(&client->dev, "create device file failed!\n");


	mc9s08dz60_i2c_client = client;

	return 0;
}

/*!
 * mc9s08dz60 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  0
 */
static int mc9s08dz60_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id mc9s08dz60_id[] = {
	{ "mc9s08dz60", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, mc9s08dz60_id);

static struct i2c_driver mc9s08dz60_i2c_driver = {
	.driver = {.owner = THIS_MODULE,
		   .name = "mc9s08dz60",
		   },
	.probe = mc9s08dz60_probe,
	.remove = mc9s08dz60_remove,
	.id_table = mc9s08dz60_id,
};

#define SET_BIT_IN_BYTE(byte, pos) (byte |= (0x01 << pos))
#define CLEAR_BIT_IN_BYTE(byte, pos) (byte &= ~(0x01 << pos))

int mc9s08dz60_init(void)
{
	int err;
	err = i2c_add_driver(&mc9s08dz60_i2c_driver);
	return err;
}
void mc9s08dz60_exit(void)
{
	i2c_del_driver(&mc9s08dz60_i2c_driver);
}
