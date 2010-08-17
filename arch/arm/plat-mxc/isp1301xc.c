/*
 * Copyright (C) 2005-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/usb/fsl_xcvr.h>
#include <linux/i2c.h>

#include <mach/arc_otg.h>

/*
 * ISP1301 register addresses,all register of ISP1301
 * is one-byte length register
 */

/* ISP1301: I2C device address */
#define ISP1301_DEV_ADDR 		0x2D

/* ISP 1301 register set*/
#define ISP1301_MODE_REG1_SET		0x04
#define ISP1301_MODE_REG1_CLR		0x05

#define ISP1301_CTRL_REG1_SET		0x06
#define ISP1301_CTRL_REG1_CLR		0x07

#define ISP1301_INT_SRC_REG		0x08
#define ISP1301_INT_LAT_REG_SET		0x0a
#define ISP1301_INT_LAT_REG_CLR		0x0b
#define ISP1301_INT_FALSE_REG_SET	0x0c
#define ISP1301_INT_FALSE_REG_CLR	0x0d
#define ISP1301_INT_TRUE_REG_SET	0x0e
#define ISP1301_INT_TRUE_REG_CLR	0x0f

#define ISP1301_CTRL_REG2_SET		0x10
#define ISP1301_CTRL_REG2_CLR		0x11

#define ISP1301_MODE_REG2_SET		0x12
#define ISP1301_MODE_REG2_CLR		0x13

#define ISP1301_BCD_DEV_REG0		0x14
#define ISP1301_BCD_DEV_REG1		0x15

/* OTG Control register bit description */
#define DP_PULLUP			0x01
#define DM_PULLUP			0x02
#define DP_PULLDOWN			0x04
#define DM_PULLDOWN			0x08
#define ID_PULLDOWN			0x10
#define VBUS_DRV			0x20
#define VBUS_DISCHRG			0x40
#define VBUS_CHRG			0x80

/* Mode Control 1 register bit description */
#define SPEED_REG  			0x01
#define SUSPEND_REG			0x02
#define DAT_SE0				0x04
#define TRANSP_EN			0x08
#define BDIS_ACON_EN			0x10
#define OE_INT_EN			0x20
#define UART_EN				0x40

/* Mode Control 2 register bit description */
#define SPD_SUSP_CTRL			0x02
#define BI_DI	  			0x04

static int isp1301_attach(struct i2c_adapter *adapter);
static int isp1301_detach(struct i2c_client *client);

static struct i2c_driver isp1301_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "isp1301 Client",
		   },
	.attach_adapter = isp1301_attach,
	.detach_client = isp1301_detach,
};

static struct i2c_client isp1301_i2c_client = {
	.name = "isp1301 I2C dev",
	.addr = ISP1301_DEV_ADDR,
	.driver = &isp1301_i2c_driver,
};

static unsigned short normal_i2c[] = { ISP1301_DEV_ADDR, I2C_CLIENT_END };

/* Magic definition of all other variables and things */
I2C_CLIENT_INSMOD;

static int isp1301_detect_client(struct i2c_adapter *adapter, int address,
				 int kind)
{
	isp1301_i2c_client.adapter = adapter;
	if (i2c_attach_client(&isp1301_i2c_client)) {
		isp1301_i2c_client.adapter = NULL;
		printk(KERN_ERR "isp1301_attach: i2c_attach_client failed\n");
		return -1;
	}

	printk(KERN_INFO "isp1301 Detected\n");
	return 0;
}

/*!
 * isp1301 I2C attach function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int isp1301_attach(struct i2c_adapter *adapter)
{
	return i2c_probe(adapter, &addr_data, &isp1301_detect_client);
}

/*!
 * isp1301 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int isp1301_detach(struct i2c_client *client)
{
	int err;

	if (!isp1301_i2c_client.adapter)
		return -1;

	err = i2c_detach_client(&isp1301_i2c_client);
	isp1301_i2c_client.adapter = NULL;

	return err;
}

static void isp1301_init(struct fsl_xcvr_ops *this)
{
	pr_debug("%s\n", __FUNCTION__);

	i2c_add_driver(&isp1301_i2c_driver);
}

static void isp1301_uninit(struct fsl_xcvr_ops *this)
{
	/* DDD do this for host only:*/
	/* disable OTG VBUS */
	i2c_del_driver(&isp1301_i2c_driver);
}

/* Write ISP1301 register*/
static inline void isp1301_write_reg(char reg, char data)
{
	i2c_smbus_write_byte_data(&isp1301_i2c_client, reg, data);
}

/* read ISP1301 register*/
static inline char isp1301_read_reg(char reg)
{
	return i2c_smbus_read_byte_data(&isp1301_i2c_client, reg);
}

/* set ISP1301 as USB host*/
static inline void isp1301_set_serial_host(void)
{
	pr_debug("%s\n", __FUNCTION__);

	isp1301_write_reg(ISP1301_MODE_REG2_CLR, 0xFF);
#if defined(CONFIG_MXC_USB_SB3) || defined(CONFIG_MXC_USB_DB4)
	isp1301_write_reg(ISP1301_MODE_REG2_SET, SPD_SUSP_CTRL | BI_DI);
#else
	isp1301_write_reg(ISP1301_MODE_REG2_SET, SPD_SUSP_CTRL);
#endif

	isp1301_write_reg(ISP1301_MODE_REG1_CLR, 0xFF);
#if defined(CONFIG_MXC_USB_SB3) || defined(CONFIG_MXC_USB_SU6)
	isp1301_write_reg(ISP1301_MODE_REG1_SET, DAT_SE0 | SPEED_REG);
#else
	isp1301_write_reg(ISP1301_MODE_REG1_SET, SPEED_REG);
#endif

	/* configure transceiver for host mode */
	isp1301_write_reg(ISP1301_CTRL_REG1_SET,
			  (VBUS_DRV | DP_PULLDOWN | DM_PULLDOWN));
}

/* set ISP1301 as USB device */
static inline void isp1301_set_serial_dev(void)
{
	pr_debug("%s\n", __FUNCTION__);

	isp1301_write_reg(ISP1301_MODE_REG2_CLR, 0xFF);
#if defined(CONFIG_MXC_USB_SB3) || defined(CONFIG_MXC_USB_DB4)
	isp1301_write_reg(ISP1301_MODE_REG2_SET, SPD_SUSP_CTRL | BI_DI);
#else
	isp1301_write_reg(ISP1301_MODE_REG2_SET, SPD_SUSP_CTRL);
#endif

	isp1301_write_reg(ISP1301_MODE_REG1_CLR, 0xFF);
#if defined(CONFIG_MXC_USB_SB3) || defined(CONFIG_MXC_USB_SU6)
	isp1301_write_reg(ISP1301_MODE_REG1_SET, DAT_SE0 | SPEED_REG);
#else
	isp1301_write_reg(ISP1301_MODE_REG1_SET, SPEED_REG);
#endif

	/* FS mode, DP pull down, DM pull down */
	isp1301_write_reg(ISP1301_CTRL_REG1_SET,
			  (DP_PULLDOWN | DM_PULLDOWN | DP_PULLUP));
}

static void isp1301_set_vbus_power(struct fsl_xcvr_ops *this,
				   struct fsl_usb2_platform_data *pdata, int on)
{
	pr_debug("%s(on=%d)\n", __FUNCTION__, on);
	if (on) {
		/* disable D+ pull-up */
		isp1301_write_reg(ISP1301_CTRL_REG1_CLR, DP_PULLUP);
		/* enable D+ pull-down */
		isp1301_write_reg(ISP1301_CTRL_REG1_SET, DP_PULLDOWN);
		/* turn on Vbus */
		isp1301_write_reg(ISP1301_CTRL_REG1_SET, VBUS_DRV);
	} else {
		/* D+ pull up, D- pull down  */
		isp1301_write_reg(ISP1301_CTRL_REG1_SET,
				  (DP_PULLUP | DM_PULLDOWN));
		/* disable D- pull up, disable D+ pull down */
		isp1301_write_reg(ISP1301_CTRL_REG1_CLR,
				  (DM_PULLUP | DP_PULLDOWN));
	}
}

/*
 * Enable or disable the D+ pullup.
 */
static void isp1301_pullup(int on)
{
	pr_debug("%s(%d)\n", __func__, on);

	if (on)
		isp1301_write_reg(ISP1301_CTRL_REG1_SET, DP_PULLUP);
	else
		isp1301_write_reg(ISP1301_CTRL_REG1_CLR, DP_PULLUP);
}

static struct fsl_xcvr_ops isp1301_ops_otg = {
	.name = "isp1301",
	.xcvr_type = PORTSC_PTS_SERIAL,
	.init = isp1301_init,
	.uninit = isp1301_uninit,
	.set_host = isp1301_set_serial_host,
	.set_device = isp1301_set_serial_dev,
	.set_vbus_power = isp1301_set_vbus_power,
	.pullup = isp1301_pullup,
};

extern void fsl_usb_xcvr_register(struct fsl_xcvr_ops *xcvr_ops);

static int __init isp1301xc_init(void)
{
	pr_debug("%s\n", __FUNCTION__);

	fsl_usb_xcvr_register(&isp1301_ops_otg);

	return 0;
}

extern void fsl_usb_xcvr_unregister(struct fsl_xcvr_ops *xcvr_ops);

static void __exit isp1301xc_exit(void)
{
	fsl_usb_xcvr_unregister(&isp1301_ops_otg);
}

subsys_initcall(isp1301xc_init);
module_exit(isp1301xc_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("isp1301");
MODULE_LICENSE("GPL");
