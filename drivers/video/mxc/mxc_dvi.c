/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*!
 * @defgroup Framebuffer Framebuffer Driver for SDC and ADC.
 */

/*!
 * @file mxc_dvi.c
 *
 * @brief MXC DVI driver
 *
 * @ingroup Framebuffer
 */

/*!
 * Include files
 */
#include <linux/i2c.h>
#include <linux/fb.h>
#include <linux/console.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/fsl_devices.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/regulator/consumer.h>
#include <mach/mxc_edid.h>
#include "mxc_dispdrv.h"
#include "../edid.h"

#define MXC_EDID_LENGTH (EDID_LENGTH*4)

#define DISPDRV_DVI	"dvi"

struct mxc_dvi_data {
	struct i2c_client *client;
	struct platform_device *pdev;
	struct mxc_dispdrv_handle *disp_dvi;
	struct delayed_work det_work;
	struct fb_info *fbi;
	struct mxc_edid_cfg edid_cfg;
	u8 cable_plugin;
	u8 edid[MXC_EDID_LENGTH];

	u32 ipu;
	u32 di;
	void (*init)(void);
	int (*update)(void);
	struct regulator *analog_reg;
};

static ssize_t mxc_dvi_show_state(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxc_dvi_data *dvi = dev_get_drvdata(dev);

	if (dvi->cable_plugin == 0)
		strcpy(buf, "plugout\n");
	else
		strcpy(buf, "plugin\n");

	return strlen(buf);
}

static DEVICE_ATTR(cable_state, S_IRUGO, mxc_dvi_show_state, NULL);

static ssize_t mxc_dvi_show_name(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxc_dvi_data *dvi = dev_get_drvdata(dev);

	strcpy(buf, dvi->fbi->fix.id);
	sprintf(buf+strlen(buf), "\n");

	return strlen(buf);
}

static DEVICE_ATTR(fb_name, S_IRUGO, mxc_dvi_show_name, NULL);

static ssize_t mxc_dvi_show_edid(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxc_dvi_data *dvi = dev_get_drvdata(dev);
	int i, j, len = 0;

	for (j = 0; j < MXC_EDID_LENGTH/16; j++) {
		for (i = 0; i < 16; i++)
			len += sprintf(buf+len, "0x%02X ",
					dvi->edid[j*16 + i]);
		len += sprintf(buf+len, "\n");
	}

	return len;
}

static DEVICE_ATTR(edid, S_IRUGO, mxc_dvi_show_edid, NULL);

static void det_worker(struct work_struct *work)
{
	struct delayed_work *delay_work = to_delayed_work(work);
	struct mxc_dvi_data *dvi =
		container_of(delay_work, struct mxc_dvi_data, det_work);
	char event_string[16];
	char *envp[] = { event_string, NULL };

	/* cable connection changes */
	if (dvi->update()) {
		u8 edid_old[MXC_EDID_LENGTH];
		dvi->cable_plugin = 1;
		sprintf(event_string, "EVENT=plugin");

		memcpy(edid_old, dvi->edid, MXC_EDID_LENGTH);

		if (mxc_edid_read(dvi->client->adapter, dvi->client->addr,
				dvi->edid, &dvi->edid_cfg, dvi->fbi) < 0)
			dev_err(&dvi->client->dev,
					"MXC dvi: read edid fail\n");
		else {
			if (!memcmp(edid_old, dvi->edid, MXC_EDID_LENGTH))
				dev_info(&dvi->client->dev,
					"Sii902x: same edid\n");
			else if (dvi->fbi->monspecs.modedb_len > 0) {
				int i;
				const struct fb_videomode *mode;
				struct fb_videomode m;

				fb_destroy_modelist(&dvi->fbi->modelist);

				for (i = 0; i < dvi->fbi->monspecs.modedb_len; i++)
					/*FIXME now we do not support interlaced mode */
					if (!(dvi->fbi->monspecs.modedb[i].vmode & FB_VMODE_INTERLACED))
						fb_add_videomode(&dvi->fbi->monspecs.modedb[i],
								&dvi->fbi->modelist);

				fb_var_to_videomode(&m, &dvi->fbi->var);
				mode = fb_find_nearest_mode(&m,
						&dvi->fbi->modelist);

				fb_videomode_to_var(&dvi->fbi->var, mode);

				dvi->fbi->var.activate |= FB_ACTIVATE_FORCE;
				console_lock();
				dvi->fbi->flags |= FBINFO_MISC_USEREVENT;
				fb_set_var(dvi->fbi, &dvi->fbi->var);
				dvi->fbi->flags &= ~FBINFO_MISC_USEREVENT;
				console_unlock();
			}
		}
	} else {
		dvi->cable_plugin = 0;
		sprintf(event_string, "EVENT=plugout");
	}

	kobject_uevent_env(&dvi->pdev->dev.kobj, KOBJ_CHANGE, envp);
}

static irqreturn_t mxc_dvi_detect_handler(int irq, void *data)
{
	struct mxc_dvi_data *dvi = data;
	schedule_delayed_work(&(dvi->det_work), msecs_to_jiffies(300));
	return IRQ_HANDLED;
}

static int dvi_init(struct mxc_dispdrv_handle *disp,
	struct mxc_dispdrv_setting *setting)
{
	int ret = 0;
	struct mxc_dvi_data *dvi = mxc_dispdrv_getdata(disp);
	struct fsl_mxc_dvi_platform_data *plat = dvi->client->dev.platform_data;

	setting->dev_id = dvi->ipu = plat->ipu_id;
	setting->disp_id = dvi->di = plat->disp_id;
	setting->if_fmt = IPU_PIX_FMT_RGB24;
	dvi->fbi = setting->fbi;
	dvi->init = plat->init;
	dvi->update = plat->update;

	dvi->analog_reg = regulator_get(&dvi->pdev->dev, plat->analog_regulator);
	if (!IS_ERR(dvi->analog_reg)) {
		regulator_set_voltage(dvi->analog_reg, 2775000, 2775000);
		regulator_enable(dvi->analog_reg);
	}

	if (dvi->init)
		dvi->init();

	/* get video mode from edid */
	if (!dvi->update)
		return -EINVAL;
	else {
		bool found = false;

		INIT_LIST_HEAD(&dvi->fbi->modelist);
		if (dvi->update()) {
			dvi->cable_plugin = 1;
			/* try to read edid */
			if (mxc_edid_read(dvi->client->adapter, dvi->client->addr,
					dvi->edid, &dvi->edid_cfg, dvi->fbi) < 0)
				dev_warn(&dvi->client->dev, "Can not read edid\n");
			else if (dvi->fbi->monspecs.modedb_len > 0) {
				int i;
				const struct fb_videomode *mode;
				struct fb_videomode m;

				for (i = 0; i < dvi->fbi->monspecs.modedb_len; i++) {
					/*FIXME now we do not support interlaced mode */
					if (!(dvi->fbi->monspecs.modedb[i].vmode
						& FB_VMODE_INTERLACED))
						fb_add_videomode(
							&dvi->fbi->monspecs.modedb[i],
							&dvi->fbi->modelist);
				}

				fb_find_mode(&dvi->fbi->var, dvi->fbi, setting->dft_mode_str,
						NULL, 0, NULL, setting->default_bpp);

				fb_var_to_videomode(&m, &dvi->fbi->var);
				mode = fb_find_nearest_mode(&m,
						&dvi->fbi->modelist);
				fb_videomode_to_var(&dvi->fbi->var, mode);
				found = 1;
			}
		} else
			dvi->cable_plugin = 0;

		if (!found) {
			ret = fb_find_mode(&dvi->fbi->var, dvi->fbi, setting->dft_mode_str,
					NULL, 0, NULL, setting->default_bpp);
			if (!ret)
				return -EINVAL;
		}
	}

	/* cable detection */
	if (dvi->client->irq) {
		ret = request_irq(dvi->client->irq, mxc_dvi_detect_handler,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"dvi_det", dvi);
		if (ret < 0) {
			dev_warn(&dvi->client->dev,
				"MXC dvi: cound not request det irq %d\n",
				dvi->client->irq);
			goto err;
		} else {
			INIT_DELAYED_WORK(&(dvi->det_work), det_worker);
			ret = device_create_file(&dvi->pdev->dev, &dev_attr_fb_name);
			if (ret < 0)
				dev_warn(&dvi->client->dev,
					"MXC dvi: cound not create sys node for fb name\n");
			ret = device_create_file(&dvi->pdev->dev, &dev_attr_cable_state);
			if (ret < 0)
				dev_warn(&dvi->client->dev,
					"MXC dvi: cound not create sys node for cable state\n");
			ret = device_create_file(&dvi->pdev->dev, &dev_attr_edid);
			if (ret < 0)
				dev_warn(&dvi->client->dev,
					"MXC dvi: cound not create sys node for edid\n");

			dev_set_drvdata(&dvi->pdev->dev, dvi);
		}
	}

err:
	return ret;
}

static void dvi_deinit(struct mxc_dispdrv_handle *disp)
{
	struct mxc_dvi_data *dvi = mxc_dispdrv_getdata(disp);

	if (!IS_ERR(dvi->analog_reg))
		regulator_disable(dvi->analog_reg);

	free_irq(dvi->client->irq, dvi);
}

static struct mxc_dispdrv_driver dvi_drv = {
	.name 	= DISPDRV_DVI,
	.init 	= dvi_init,
	.deinit	= dvi_deinit,
};

static int __devinit mxc_dvi_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct mxc_dvi_data *dvi;
	int ret = 0;

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE | I2C_FUNC_I2C))
		return -ENODEV;

	dvi = kzalloc(sizeof(struct mxc_dvi_data), GFP_KERNEL);
	if (!dvi) {
		ret = -ENOMEM;
		goto alloc_failed;
	}

	dvi->pdev = platform_device_register_simple("mxc_dvi", 0, NULL, 0);
	if (IS_ERR(dvi->pdev)) {
		printk(KERN_ERR
				"Unable to register MXC DVI as a platform device\n");
		ret = PTR_ERR(dvi->pdev);
		goto pdev_reg_failed;
	}

	dvi->client = client;
	dvi->disp_dvi = mxc_dispdrv_register(&dvi_drv);
	mxc_dispdrv_setdata(dvi->disp_dvi, dvi);

	i2c_set_clientdata(client, dvi);

	return ret;

pdev_reg_failed:
	kfree(dvi);
alloc_failed:
	return ret;
}

static int __devexit mxc_dvi_remove(struct i2c_client *client)
{
	struct mxc_dvi_data *dvi = i2c_get_clientdata(client);

	mxc_dispdrv_puthandle(dvi->disp_dvi);
	mxc_dispdrv_unregister(dvi->disp_dvi);
	platform_device_unregister(dvi->pdev);
	kfree(dvi);
	return 0;
}

static const struct i2c_device_id mxc_dvi_id[] = {
	{ "mxc_dvi", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, mxc_dvi_id);

static struct i2c_driver mxc_dvi_i2c_driver = {
	.driver = {
		   .name = "mxc_dvi",
		   },
	.probe = mxc_dvi_probe,
	.remove = mxc_dvi_remove,
	.id_table = mxc_dvi_id,
};

static int __init mxc_dvi_init(void)
{
	return i2c_add_driver(&mxc_dvi_i2c_driver);
}

static void __exit mxc_dvi_exit(void)
{
	i2c_del_driver(&mxc_dvi_i2c_driver);
}

module_init(mxc_dvi_init);
module_exit(mxc_dvi_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC DVI driver");
MODULE_LICENSE("GPL");
