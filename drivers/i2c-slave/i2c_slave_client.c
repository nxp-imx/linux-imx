/*
 * Copyright 2007-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>

struct i2c_client *i2c_slave_client;
static int i2c_slave_client_probe(struct i2c_client *adapter);
static int i2c_slave_client_remove(struct i2c_client *client);
static struct i2c_driver i2c_slave_client_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "i2c-slave-client",
		   },
	.probe = i2c_slave_client_probe,
	.remove = i2c_slave_client_remove,
};

/*!
 * ov2640 I2C attach function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int i2c_slave_client_probe(struct i2c_client *client)
{
	i2c_slave_client = client;
	return 0;
}

/*!
 * ov2640 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int i2c_slave_client_remove(struct i2c_client *client)
{
	return 0;
}

/*!
 * ov2640 init function
 *
 * @return  Error code indicating success or failure
 */
static __init int i2c_slave_client_init(void)
{
	return i2c_add_driver(&i2c_slave_client_driver);
}

/*!
 * OV2640 cleanup function
 *
 * @return  Error code indicating success or failure
 */
static void __exit i2c_slave_client_clean(void)
{
	i2c_del_driver(&i2c_slave_client_driver);
}

module_init(i2c_slave_client_init);
module_exit(i2c_slave_client_clean);
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("I2c Slave Client Driver");
MODULE_LICENSE("GPL");
