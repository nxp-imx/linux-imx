/*
 * Copyright(c) 2009 Dialog Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * da9052-i2c.c: I2C SSC (Synchronous Serial Communication) driver for DA9052
 */

#include <linux/device.h>
#include <linux/mfd/core.h>
#include <linux/i2c.h>
#include <linux/mfd/da9052/da9052.h>
#include <linux/mfd/da9052/reg.h>

static struct da9052 *da9052_i2c;

#define I2C_CONNECTED 0

static int da9052_i2c_is_connected(void)
{

        struct da9052_ssc_msg msg;

        //printk("Entered da9052_i2c_is_connected.............\n");

        msg.addr = DA9052_INTERFACE_REG;

        /* Test spi connectivity by performing read of the GPIO_0-1 register */
        if ( 0 != da9052_i2c_read(da9052_i2c, &msg)) {
                printk("da9052_i2c_is_connected - i2c read failed.............\n");
                return -1;
        }
        else {
               printk("da9052_i2c_is_connected - i2c read success..............\n");
                return 0;
        }

}

static int __devinit da9052_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter;
 	// printk("\n\tEntered da9052_i2c_is_probe.............\n");

        da9052_i2c = kzalloc(sizeof(struct da9052), GFP_KERNEL);

        if (!da9052_i2c)
                return -ENOMEM;

	/* Get the bus driver handler */
	adapter = to_i2c_adapter(client->dev.parent);

	/* Check i2c bus driver supports byte data transfer */
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_info(&client->dev,\
		"Error in %s:i2c_check_functionality\n", __func__);
		return -ENODEV;;
	}

	/* Store handle to i2c client */
	da9052_i2c->i2c_client = client;
	da9052_i2c->irq = client->irq;

	da9052_i2c->dev = &client->dev;

	/* Initialize i2c data structure here*/
	da9052_i2c->adapter = adapter;

	/* host i2c driver looks only first 7 bits for the slave address */
	da9052_i2c->slave_addr = DA9052_I2C_ADDR >> 1;

	/* Store the i2c client data */
	i2c_set_clientdata(client, da9052_i2c);

	 /* Validate I2C connectivity */
        if ( I2C_CONNECTED  == da9052_i2c_is_connected()) {
                /* I2C is connected */
                da9052_i2c->connecting_device = I2C;
                if( 0!= da9052_ssc_init(da9052_i2c) )
                        return -ENODEV;
        }
        else {
                return -ENODEV;
        }

        //printk("Exiting da9052_i2c_probe.....\n");

	return 0;
}

static int da9052_i2c_remove(struct i2c_client *client)
{

	struct da9052 *da9052 = i2c_get_clientdata(client);

	mfd_remove_devices(da9052->dev);
	kfree(da9052);
	return 0;
}

int da9052_i2c_write(struct da9052 *da9052, struct da9052_ssc_msg *msg)
{
	struct i2c_msg i2cmsg;
	unsigned char buf[2] = {0};
	int ret = 0;

	/* Copy the ssc msg to local character buffer */
	buf[0] = msg->addr;
	buf[1] = msg->data;

	/*Construct a i2c msg for a da9052 driver ssc message request */
	i2cmsg.addr  = da9052->slave_addr;
	i2cmsg.len   = 2;
	i2cmsg.buf   = buf;

	/* To write the data on I2C set flag to zero */
	i2cmsg.flags = 0;

	/* Start the i2c transfer by calling host i2c driver function */
	ret = i2c_transfer(da9052->adapter, &i2cmsg, 1);

	if (ret < 0) {
		dev_info(&da9052->i2c_client->dev,\
		"_%s:master_xfer Failed!!\n", __func__);
		return ret;
	}

	return 0;
}

int da9052_i2c_read(struct da9052 *da9052, struct da9052_ssc_msg *msg)
{

	/*Get the da9052_i2c client details*/
	unsigned char buf[2] = {0, 0};
	struct i2c_msg i2cmsg[2];
	int ret = 0;

	/* Copy SSC Msg to local character buffer */
	buf[0] = msg->addr;

	/*Construct a i2c msg for a da9052 driver ssc message request */
	i2cmsg[0].addr  = da9052->slave_addr ;
	i2cmsg[0].len   = 1;
	i2cmsg[0].buf   = &buf[0];

	/*To write the data on I2C set flag to zero */
	i2cmsg[0].flags = 0;

	/* Read the data from da9052*/
	/*Construct a i2c msg for a da9052 driver ssc message request */
	i2cmsg[1].addr  = da9052->slave_addr ;
	i2cmsg[1].len   = 1;
	i2cmsg[1].buf   = &buf[1];

	/*To read the data on I2C set flag to I2C_M_RD */
	i2cmsg[1].flags = I2C_M_RD;

	/* Start the i2c transfer by calling host i2c driver function */
	ret = i2c_transfer(da9052->adapter, i2cmsg, 2);
	if (ret < 0) {
		dev_info(&da9052->i2c_client->dev,\
		"2 - %s:master_xfer Failed!!\n", __func__);
		return ret;
	}

	msg->data = *i2cmsg[1].buf;

	return 0;
}

int da9052_i2c_write_many(struct da9052 *da9052,
	struct da9052_ssc_msg *sscmsg, int msg_no)
{

	struct i2c_msg i2cmsg;
	unsigned char data_buf[MAX_READ_WRITE_CNT+1];
	struct da9052_ssc_msg ctrlb_msg;
	struct da9052_ssc_msg *msg_queue = sscmsg;
	int ret = 0;
	/* Flag to check if requested registers are contiguous */
	unsigned char cont_data = 1;
	unsigned char cnt = 0;

	/* Check if requested registers are contiguous */
	for (cnt = 1; cnt < msg_no; cnt++) {
		if ((msg_queue[cnt].addr - msg_queue[cnt-1].addr) != 1) {
			/* Difference is not 1, i.e. non-contiguous registers */
			cont_data = 0;
			break;
		}
	}

	if (cont_data == 0) {
		/* Requested registers are non-contiguous */
		for (cnt = 0; cnt < msg_no; cnt++) {
			ret = da9052->write(da9052, &msg_queue[cnt]);
			if (ret != 0)
				return ret;
		}
		return 0;
	}
	/*
	*  Requested registers are contiguous
	* or PAGE WRITE sequence of I2C transactions is as below
	* (slave_addr + reg_addr + data_1 + data_2 + ...)
	* First read current WRITE MODE via CONTROL_B register of DA9052
	*/
	ctrlb_msg.addr = DA9052_CONTROLB_REG;
	ctrlb_msg.data = 0x0;
	ret = da9052->read(da9052, &ctrlb_msg);

	if (ret != 0)
		return ret;

	/* Check if PAGE WRITE mode is set */
	if (ctrlb_msg.data & DA9052_CONTROLB_WRITEMODE) {
		/* REPEAT WRITE mode is configured */
		/* Now set DA9052 into PAGE WRITE mode */
		ctrlb_msg.data &= ~DA9052_CONTROLB_WRITEMODE;
		ret = da9052->write(da9052, &ctrlb_msg);

		if (ret != 0)
			return ret;
	}

	 /* Put first register address */
	data_buf[0] = msg_queue[0].addr;

	for (cnt = 0; cnt < msg_no; cnt++)
		data_buf[cnt+1] = msg_queue[cnt].data;

	/* Construct a i2c msg for PAGE WRITE */
	i2cmsg.addr  = da9052->slave_addr ;
	/* First register address + all data*/
	i2cmsg.len   = (msg_no + 1);
	i2cmsg.buf   = data_buf;

	/*To write the data on I2C set flag to zero */
	i2cmsg.flags = 0;

	/* Start the i2c transfer by calling host i2c driver function */
	ret = i2c_transfer(da9052->adapter, &i2cmsg, 1);
	if (ret < 0) {
		dev_info(&da9052->i2c_client->dev,\
		"1 - i2c_transfer function falied in [%s]!!!\n", __func__);
		return ret;
	}

	return 0;
}

int da9052_i2c_read_many(struct da9052 *da9052,
	struct da9052_ssc_msg *sscmsg, int msg_no)
{

	struct i2c_msg i2cmsg;
	unsigned char data_buf[MAX_READ_WRITE_CNT];
	struct da9052_ssc_msg *msg_queue = sscmsg;
	int ret = 0;
	/* Flag to check if requested registers are contiguous */
	unsigned char cont_data = 1;
	unsigned char cnt = 0;

	/* Check if requested registers are contiguous */
	for (cnt = 1; cnt < msg_no; cnt++) {
		if ((msg_queue[cnt].addr - msg_queue[cnt-1].addr) != 1) {
			/* Difference is not 1, i.e. non-contiguous registers */
			cont_data = 0;
			break;
		}
	}

	if (cont_data == 0) {
		/* Requested registers are non-contiguous */
		for (cnt = 0; cnt < msg_no; cnt++) {
			ret = da9052->read(da9052, &msg_queue[cnt]);
			if (ret != 0) {
				dev_info(&da9052->i2c_client->dev,\
				"Error in %s", __func__);
				return ret;
			}
		}
		return 0;
	}

	/*
	* We want to perform PAGE READ via I2C
	* For PAGE READ sequence of I2C transactions is as below
	* (slave_addr + reg_addr) + (slave_addr + data_1 + data_2 + ...)
	*/
	/* Copy address of first register */
	data_buf[0] = msg_queue[0].addr;

	/* Construct a i2c msg for first transaction of PAGE READ i.e. write */
	i2cmsg.addr  = da9052->slave_addr ;
	i2cmsg.len   = 1;
	i2cmsg.buf   = data_buf;

	/*To write the data on I2C set flag to zero */
	i2cmsg.flags = 0;

	/* Start the i2c transfer by calling host i2c driver function */
	ret = i2c_transfer(da9052->adapter, &i2cmsg, 1);
	if (ret < 0) {
		dev_info(&da9052->i2c_client->dev,\
		"1 - i2c_transfer function falied in [%s]!!!\n", __func__);
		return ret;
	}

	/* Now Read the data from da9052 */
	/* Construct a i2c msg for second transaction of PAGE READ i.e. read */
	i2cmsg.addr  = da9052->slave_addr ;
	i2cmsg.len   = msg_no;
	i2cmsg.buf   = data_buf;

	/*To read the data on I2C set flag to I2C_M_RD */
	i2cmsg.flags = I2C_M_RD;

	/* Start the i2c transfer by calling host i2c driver function */
	ret = i2c_transfer(da9052->adapter,
		&i2cmsg, 1);
	if (ret < 0) {
		dev_info(&da9052->i2c_client->dev,\
		"2 - i2c_transfer function falied in [%s]!!!\n", __func__);
		return ret;
	}

	/* Gather READ data */
	for (cnt = 0; cnt < msg_no; cnt++)
		sscmsg[cnt].data = data_buf[cnt];

	return 0;
}

static struct i2c_device_id da9052_ssc_id[] = {
	{ DA9052_SSC_I2C_DEVICE_NAME, 0},
	{}
};

static struct i2c_driver da9052_i2c_driver =  {
	.driver = {
		.name	= DA9052_SSC_I2C_DEVICE_NAME,
		.owner	= THIS_MODULE,
	},
	.probe	= da9052_i2c_probe,
	.remove	= da9052_i2c_remove,
	.id_table	= da9052_ssc_id,
};

static int __init da9052_i2c_init(void)
{
        int ret = 0;
       // printk("\n\nEntered da9052_i2c_init................\n\n");
        ret = i2c_add_driver(&da9052_i2c_driver);
        if (ret != 0) {
                printk(KERN_ERR "Unable to register %s\n", DA9052_SSC_I2C_DEVICE_NAME);
                return ret;
        }
        return 0;
}
subsys_initcall(da9052_i2c_init);

static void  __exit da9052_i2c_exit(void)
{
        i2c_del_driver(&da9052_i2c_driver);
}
module_exit(da9052_i2c_exit);

MODULE_AUTHOR("Dialog Semiconductor Ltd <dchen@diasemi.com>");
MODULE_DESCRIPTION("I2C driver for Dialog DA9052 PMIC");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DA9052_SSC_I2C_DEVICE_NAME);
