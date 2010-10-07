/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

/*!
 * @file pmic/core/max17135.c
 * @brief This file contains MAX17135 specific PMIC code. This implementaion
 * may differ for each PMIC chip.
 *
 * @ingroup PMIC_CORE
 */

/*
 * Includes
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/uaccess.h>

#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/pmic_status.h>
#include <linux/mfd/max17135.h>
#include <asm/mach-types.h>

struct i2c_client *max17135_client;

static const unsigned short normal_i2c[] = {0x48, I2C_CLIENT_END};

int max17135_reg_read(int reg_num, unsigned int *reg_val)
{
	int result;

	if (max17135_client == NULL)
		return PMIC_ERROR;

	if ((reg_num == REG_MAX17135_EXT_TEMP) ||
		(reg_num == REG_MAX17135_INT_TEMP)) {
		result = i2c_smbus_read_word_data(max17135_client, reg_num);
		if (result < 0) {
			dev_err(&max17135_client->dev,
				"Unable to read MAX17135 register via I2C\n");
			return PMIC_ERROR;
		}
		/* Swap bytes for dword read */
		result = (result >> 8) | ((result & 0xFF) << 8);
	} else {
		result = i2c_smbus_read_byte_data(max17135_client, reg_num);
		if (result < 0) {
			dev_err(&max17135_client->dev,
				"Unable to read MAX17135 register via I2C\n");
			return PMIC_ERROR;
		}
	}

	*reg_val = result;
	return PMIC_SUCCESS;
}

int max17135_reg_write(int reg_num, const unsigned int reg_val)
{
	int result;

	if (max17135_client == NULL)
		return PMIC_ERROR;

	result = i2c_smbus_write_byte_data(max17135_client, reg_num, reg_val);
	if (result < 0) {
		dev_err(&max17135_client->dev,
			"Unable to write MAX17135 register via I2C\n");
		return PMIC_ERROR;
	}

	return PMIC_SUCCESS;
}

static int max17135_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct max17135 *max17135;
	struct max17135_platform_data *pdata = client->dev.platform_data;
	int ret = 0;

	if (!pdata || !pdata->init)
		return -ENODEV;

	/* Create the PMIC data structure */
	max17135 = kzalloc(sizeof(struct max17135), GFP_KERNEL);
	if (max17135 == NULL) {
		kfree(client);
		return -ENOMEM;
	}

	/* Initialize the PMIC data structure */
	i2c_set_clientdata(client, max17135);
	max17135->dev = &client->dev;
	max17135->i2c_client = client;

	max17135_client = client;

	if (pdata && pdata->init) {
		ret = pdata->init(max17135);
		if (ret != 0)
			goto err;
	}

	dev_info(&client->dev, "PMIC MAX17135 for eInk display\n");

	return ret;
err:
	kfree(max17135);

	return ret;
}


static int max17135_remove(struct i2c_client *i2c)
{
	struct max17135 *max17135 = i2c_get_clientdata(i2c);
	int i;

	for (i = 0; i < ARRAY_SIZE(max17135->pdev); i++)
		platform_device_unregister(max17135->pdev[i]);

	kfree(max17135);

	return 0;
}

static int max17135_suspend(struct i2c_client *client, pm_message_t state)
{
	return 0;
}

static int max17135_resume(struct i2c_client *client)
{
	return 0;
}

/* Return 0 if detection is successful, -ENODEV otherwise */
static int max17135_detect(struct i2c_client *client,
			  struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	u8 chip_rev, chip_id;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	/* detection */
	if (i2c_smbus_read_byte_data(client,
		REG_MAX17135_PRODUCT_REV) != 0) {
		dev_err(&adapter->dev,
			"Max17135 PMIC not found!\n");
		return -ENODEV;
	}

	/* identification */
	chip_rev = i2c_smbus_read_byte_data(client,
		 REG_MAX17135_PRODUCT_REV);
	chip_id = i2c_smbus_read_byte_data(client,
		  REG_MAX17135_PRODUCT_ID);

	if (chip_rev != 0x00 || chip_id != 0x4D) { /* identification failed */
		dev_info(&adapter->dev,
		    "Unsupported chip (man_id=0x%02X, "
		    "chip_id=0x%02X).\n", chip_rev, chip_id);
		return -ENODEV;
	}

	strlcpy(info->type, "max17135_sensor", I2C_NAME_SIZE);

	return 0;
}

static const struct i2c_device_id max17135_id[] = {
       { "max17135", 0 },
       { }
};
MODULE_DEVICE_TABLE(i2c, max17135_id);


static struct i2c_driver max17135_driver = {
	.driver = {
		   .name = "max17135",
		   .owner = THIS_MODULE,
	},
	.probe = max17135_probe,
	.remove = max17135_remove,
	.suspend = max17135_suspend,
	.resume = max17135_resume,
	.id_table = max17135_id,
	.detect = max17135_detect,
	.address_list = &normal_i2c,
};

static int __init max17135_init(void)
{
	return i2c_add_driver(&max17135_driver);
}

static void __exit max17135_exit(void)
{
	i2c_del_driver(&max17135_driver);
}

/*
 * Module entry points
 */
subsys_initcall_sync(max17135_init);
module_exit(max17135_exit);
