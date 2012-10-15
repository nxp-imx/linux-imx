/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file adv7280_mipi_csi2_tvin.c
 *
 * @brief Analog Device adv7280 video decoder functions
 *
 * @ingroup Camera
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/fsl_devices.h>

#define ADV7280_SAMPLE_SILICON         0x40
#define ADV7280_PROD_SILICON           0x41
/** ADV7280 register definitions */
#define ADV7280_INPUT_CTL              0x00	/* Input Control */
#define ADV7280_STATUS_1               0x10	/* Status #1 */
#define ADV7280_BRIGHTNESS             0x0a	/* Brightness */
#define ADV7280_IDENT                  0x11	/* IDENT */
#define ADV7280_VSYNC_FIELD_CTL_1      0x31	/* VSYNC Field Control #1 */
#define ADV7280_MANUAL_WIN_CTL         0x3d	/* Manual Window Control */
#define ADV7280_SD_SATURATION_CB       0xe3	/* SD Saturation Cb */
#define ADV7280_SD_SATURATION_CR       0xe4	/* SD Saturation Cr */
#define ADV7280_PM_REG                 0x0f /* Power Management */
#define ADV7280_ADI_CONTROL1           0x0E /* ADI Control 1 */
/* adv7280 power management reset bit */
#define ADV7280_PM_RESET_BIT           0x80
/** adv7280 input video masks */
#define ADV7280_INPUT_VIDEO_MASK       0x70 /* input video mask */
#define ADV7280_PAL_MODE_BIT           0x40 /* PAL mode bit     */
#define ADV7280_PAL_M_MODE_BIT         0x20 /* PAL M mode bit   */
#define ADV7280_NTSC_MODE_BIT          0x00 /* NTSC mode bit    */
#define ADV7280_NTSC_4_43_MODE_BIT     0x10 /* NTSC 4.43 mode bit*/
/** adv7280 voltages */
#define ADV7280_VOLTAGE_ANALOG         2800000
#define ADV7280_VOLTAGE_DIGITAL_CORE   1500000
#define ADV7280_VOLTAGE_DIGITAL_IO     1800000

struct reg_value {
	u8 reg;
	u8 value;
	u8 mask;
	u32 delay_ms;
};

struct adv7280_priv {
	struct fsl_mxc_tvin_platform_data *pdata;
	struct i2c_client *client;
};

/*!
 *  adv7280 initialization register structure
 *  registers are configured based on ADI RM
 *  1st value = Register Address
 *  2nd value = default value
 *  3rd value = Register Mask
 *  4th value = delay time to wait until next command
 */

static struct reg_value adv7280_init_params[] = {
	{0x0F, 0x04, 0x00, 0}, /* exit power down mode, no pwrdwn line available
						   in imx6-AI use i2c powrdwn */
	{0x00, 0x0E, 0x00, 0}, /* input control */
	{0x03, 0x0C, 0x00, 0}, /* enable pixel and sync output drivers */
	{0x04, 0x37, 0x00, 0}, /* enable SFL */
	{0x13, 0x00, 0x00, 0}, /* enable INTRQ output driver */
	{0x17, 0x41, 0x00, 0}, /* select SH1 */
	{0x1D, 0x40, 0x00, 0}, /* enable LCC output driver */
	{0x52, 0xC0, 0x00, 0}, /* ADI recommended*/
	{0xFE, 0xA0, 0x00, 0}, /* set CSI-Tx slave address to 0xA0 */
	{0x59, 0x15, 0x00, 0}, /* GPO control */
};

/*! Read one register from a ADV7280 i2c slave device.
 *  @param *reg     register in the device we wish to access.
 *  @return         0 if success, an error code otherwise.
 */
static inline int adv7280_read_reg(struct adv7280_priv *adv7280, u8 reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(adv7280->client, reg);
	if (ret < 0) {
		dev_dbg(&adv7280->client->dev, "%s:read reg error: reg=%2x\n",
				__func__, reg);
	}

	return ret;
}

/*! Write one register from a ADV7280 i2c slave device.
 *  @param *reg     register in the device we wish to access.
 *  @return         0 if success, an error code otherwise.
 */
static inline int adv7280_write_reg(struct adv7280_priv *adv7280,
		u8 reg, u8 val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(adv7280->client, reg, val);
	if (ret < 0) {
		dev_dbg(&adv7280->client->dev, "%s:write reg error: reg=%2x\n",
				__func__, reg);
	}

	return ret;
}

/*! Write ADV7280 config paramater array
 */
static int adv7280_config(struct adv7280_priv *adv7280,
		struct reg_value *config, int size) {
	int i, ret;

	for (i = 0; i < size; i++) {
		pr_debug("%s[%d]: reg = 0x%02x, value = 0x%02x\n", __func__,
				i, config[i].reg,  config[i].value);
		ret = adv7280_write_reg(adv7280, config[i].reg,
				config[i].value | config[i].mask);
		if (ret < 0) {
			pr_err("%s: write error %x\n", __func__, ret);
			return ret;
		}

		if (config[i].delay_ms)
			msleep(config[i].delay_ms);
	}

	return 0;
}

/*!
 * ADV7280 I2C probe function.
 * Function set in i2c_driver struct.
 * Called by insmod.
 *
 *  @param *adapter	I2C adapter descriptor.
 *  @return		Error code indicating success or failure.
 */
static int adv7280_i2c_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret;
	struct adv7280_priv *adv7280;

	adv7280 = kzalloc(sizeof(struct adv7280_priv), GFP_KERNEL);
	if (!adv7280)
		return -ENOMEM;

	adv7280->client = client;
	adv7280->pdata = client->dev.platform_data;
	i2c_set_clientdata(client, adv7280);

	ret = adv7280_read_reg(adv7280, ADV7280_IDENT);
	if (ret < 0) {
		pr_err("%s: read id error %x\n", __func__, ret);
		goto err;
	}

	if ((ret & ADV7280_SAMPLE_SILICON) != 0x40) {
		pr_err("%s: device ADV7280 not found, ret = 0x%02x\n",
				__func__, ret);
		goto err;
	}

	pr_info("%s: device found, rev_id 0x%02x\n", __func__, ret);
	/* select main register map */
	ret = adv7280_write_reg(adv7280, ADV7280_ADI_CONTROL1, 0x00);
	if (ret < 0) {
		pr_err("%s: write error, select memory map %x\n",
				__func__, ret);
		goto err;
	}

	/* perform a device reset */
	ret = adv7280_write_reg(adv7280, ADV7280_PM_REG,
			ADV7280_PM_RESET_BIT);
	if (ret < 0) {
		pr_err("%s: write error, reset %x\n", __func__, ret);
		goto err;
	}
	/* Wait 5ms reset time spec */
	msleep(5);

	/* Initial device configuration */
	ret = adv7280_config(adv7280, adv7280_init_params,
			ARRAY_SIZE(adv7280_init_params));
	if (ret < 0) {
		pr_err("%s: config device error %x\n", __func__, ret);
		goto err;
	}

	return 0;
err:
	kfree(adv7280);
	return ret;
}

static int adv7280_i2c_remove(struct i2c_client *i2c_client)
{
	struct adv7280_priv *adv7280 = i2c_get_clientdata(i2c_client);

	kfree(adv7280);
	return 0;
}

/*!
 * adv7280_i2c_driver - i2c device identification
 * This structure tells the I2C subsystem how to identify and support a
 * given I2C device type.
 */

static const struct i2c_device_id adv7280_i2c_id[] = {
	{"adv7280", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, adv7280_i2c_id);

static struct i2c_driver adv7280_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "adv7280",
		   },
	.probe = adv7280_i2c_probe,
	.remove = adv7280_i2c_remove,
	.id_table = adv7280_i2c_id,
};

/*!
 * ADV7280 init function.:
 * Called on insmod.
 * @return    Error code indicating success or failure.
 */
static __init int adv7280_init(void)
{
	int err;

	err = i2c_add_driver(&adv7280_i2c_driver);

	if (err < 0)
		pr_err("%s: driver registration failed, error=%d\n",
			__func__, err);

	return err;
}

/*!
 * ADV7280 cleanup function.
 * Called on rmmod.
 * @return   Error code indicating success or failure.
 */
static void __exit adv7280_clean(void)
{
	i2c_del_driver(&adv7280_i2c_driver);
}

module_init(adv7280_init);
module_exit(adv7280_clean);

MODULE_AUTHOR("Freescale Semiconductor");
MODULE_DESCRIPTION("adv7280 video decoder driver");
MODULE_LICENSE("GPL");
