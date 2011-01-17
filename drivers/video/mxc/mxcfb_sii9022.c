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

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*!
 * @defgroup Framebuffer Framebuffer Driver for SDC and ADC.
 */

/*!
 * @file mxcfb_sii9022.c
 *
 * @brief MXC Frame buffer driver for SII9022
 *
 * @ingroup Framebuffer
 */

/*!
 * Include files
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/mxcfb.h>
#include <linux/fsl_devices.h>
#include <linux/interrupt.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/mxc_edid.h>

struct sii9022_data {
	int major;
	struct class *class;
	struct device *dev;
	struct i2c_client *client;
	struct delayed_work det_work;
	struct fb_info *fbi;
	struct mxc_edid_cfg edid_cfg;
	u8 cable_plugin;
	u8 edid[256];
} sii9022;

static void sii9022_poweron(void);
static void sii9022_poweroff(void);
static void (*sii9022_reset) (void);

static __attribute__ ((unused)) void dump_regs(u8 reg, int len)
{
	u8 buf[50];
	int i;

	i2c_smbus_read_i2c_block_data(sii9022.client, reg, len, buf);
	for (i = 0; i < len; i++)
		dev_dbg(&sii9022.client->dev, "reg[0x%02X]: 0x%02X\n",
				i+reg, buf[i]);
}

static ssize_t sii9022_show_state(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	if (sii9022.cable_plugin == 0)
		strcpy(buf, "plugout\n");
	else
		strcpy(buf, "plugin\n");

	return strlen(buf);
}

static DEVICE_ATTR(cable_state, S_IRUGO | S_IWUSR, sii9022_show_state, NULL);

static void sii9022_setup(struct fb_info *fbi)
{
	u16 data[4];
	u32 refresh;
	u8 *tmp;
	int i;

	dev_dbg(&sii9022.client->dev, "SII9022: setup..\n");

	/* Power up */
	i2c_smbus_write_byte_data(sii9022.client, 0x1E, 0x00);

	/* set TPI video mode */
	data[0] = PICOS2KHZ(fbi->var.pixclock) / 10;
	data[2] = fbi->var.hsync_len + fbi->var.left_margin +
		  fbi->var.xres + fbi->var.right_margin;
	data[3] = fbi->var.vsync_len + fbi->var.upper_margin +
		  fbi->var.yres + fbi->var.lower_margin;
	refresh = data[2] * data[3];
	refresh = (PICOS2KHZ(fbi->var.pixclock) * 1000) / refresh;
	data[1] = refresh * 100;
	tmp = (u8 *)data;
	for (i = 0; i < 8; i++)
		i2c_smbus_write_byte_data(sii9022.client, i, tmp[i]);

	/* input bus/pixel: full pixel wide (24bit), rising edge */
	i2c_smbus_write_byte_data(sii9022.client, 0x08, 0x70);
	/* Set input format to RGB */
	i2c_smbus_write_byte_data(sii9022.client, 0x09, 0x00);
	/* set output format to RGB */
	i2c_smbus_write_byte_data(sii9022.client, 0x0A, 0x00);
	/* audio setup */
	i2c_smbus_write_byte_data(sii9022.client, 0x25, 0x00);
	i2c_smbus_write_byte_data(sii9022.client, 0x26, 0x40);
	i2c_smbus_write_byte_data(sii9022.client, 0x27, 0x00);
}

static int sii9022_read_edid(void)
{
	int old, dat, ret, cnt = 100;

	old = i2c_smbus_read_byte_data(sii9022.client, 0x1A);

	i2c_smbus_write_byte_data(sii9022.client, 0x1A, old | 0x4);
	do {
		cnt--;
		msleep(10);
		dat = i2c_smbus_read_byte_data(sii9022.client, 0x1A);
	} while ((!(dat & 0x2)) && cnt);

	if (!cnt) {
		ret = -1;
		goto done;
	}

	i2c_smbus_write_byte_data(sii9022.client, 0x1A, old | 0x06);

	/* edid reading */
	ret = mxc_edid_read(sii9022.client->adapter, sii9022.edid,
				&sii9022.edid_cfg, sii9022.fbi);

	cnt = 100;
	do {
		cnt--;
		i2c_smbus_write_byte_data(sii9022.client, 0x1A, old & ~0x6);
		msleep(10);
		dat = i2c_smbus_read_byte_data(sii9022.client, 0x1A);
	} while ((dat & 0x6) && cnt);

	if (!cnt)
		ret = -1;

done:
	i2c_smbus_write_byte_data(sii9022.client, 0x1A, old);
	return ret;
}

static void det_worker(struct work_struct *work)
{
	int dat;
	char event_string[16];
	char *envp[] = { event_string, NULL };

	dat = i2c_smbus_read_byte_data(sii9022.client, 0x3D);
	if (dat & 0x1) {
		/* cable connection changes */
		if (dat & 0x4) {
			sii9022.cable_plugin = 1;
			sprintf(event_string, "EVENT=plugin");
			if (sii9022_read_edid() < 0)
				dev_err(&sii9022.client->dev,
					"SII9022: read edid fail\n");
			else {
				if (sii9022.fbi->monspecs.modedb_len > 0) {
					int i;

					for (i = 0; i < sii9022.fbi->monspecs.modedb_len; i++)
						fb_add_videomode(&sii9022.fbi->monspecs.modedb[i],
								&sii9022.fbi->modelist);
				}
				sii9022_poweron();
			}
		} else {
			sii9022.cable_plugin = 0;
			sprintf(event_string, "EVENT=plugout");
			sii9022_poweroff();
		}
		kobject_uevent_env(&sii9022.dev->kobj, KOBJ_CHANGE, envp);
	}
	i2c_smbus_write_byte_data(sii9022.client, 0x3D, dat);
}

static irqreturn_t sii9022_detect_handler(int irq, void *data)
{
	schedule_delayed_work(&(sii9022.det_work), msecs_to_jiffies(100));
	return IRQ_HANDLED;
}

static int lcd_fb_event(struct notifier_block *nb, unsigned long val, void *v)
{
	struct fb_event *event = v;
	struct fb_info *fbi = event->info;

	/* assume sii9022 on DI0 only */
	if (strcmp(event->info->fix.id, "DISP3 BG"))
		return 0;

	switch (val) {
	case FB_EVENT_MODE_CHANGE:
		sii9022_setup(fbi);
		break;
	case FB_EVENT_BLANK:
		if (*((int *)event->data) == FB_BLANK_UNBLANK)
			sii9022_poweron();
		else
			sii9022_poweroff();
		break;
	}
	return 0;
}

static struct notifier_block nb = {
	.notifier_call = lcd_fb_event,
};

static int __devinit sii9022_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int i, dat;
	struct mxc_lcd_platform_data *plat = client->dev.platform_data;

	sii9022.client = client;

	if (plat->reset) {
		sii9022_reset = plat->reset;
		sii9022_reset();
	}

	/* Set 9022 in hardware TPI mode on and jump out of D3 state */
	if (i2c_smbus_write_byte_data(sii9022.client, 0xc7, 0x00) < 0) {
		dev_err(&sii9022.client->dev,
			"SII9022: cound not find device\n");
		return -ENODEV;
	}

	/* read device ID */
	for (i = 10; i > 0; i--) {
		dat = i2c_smbus_read_byte_data(sii9022.client, 0x1B);
		printk(KERN_DEBUG "Sii9022: read id = 0x%02X", dat);
		if (dat == 0xb0) {
			dat = i2c_smbus_read_byte_data(sii9022.client, 0x1C);
			printk(KERN_DEBUG "-0x%02X", dat);
			dat = i2c_smbus_read_byte_data(sii9022.client, 0x1D);
			printk(KERN_DEBUG "-0x%02X", dat);
			dat = i2c_smbus_read_byte_data(sii9022.client, 0x30);
			printk(KERN_DEBUG "-0x%02X\n", dat);
			break;
		}
	}
	if (i == 0) {
		dev_err(&sii9022.client->dev,
			"SII9022: cound not find device\n");
		return -ENODEV;
	}

	if (sii9022.client->irq) {
		int ret;

		ret = request_irq(sii9022.client->irq, sii9022_detect_handler,
				IRQF_TRIGGER_FALLING,
				"sii9022_det", &sii9022.client->dev);
		if (ret < 0)
			dev_warn(&sii9022.client->dev,
				"SII9022: cound not request det irq %d\n",
				sii9022.client->irq);
		else {
			/*enable cable hot plug irq*/
			i2c_smbus_write_byte_data(sii9022.client, 0x3c, 0x01);
			INIT_DELAYED_WORK(&(sii9022.det_work), det_worker);
		}
		ret = device_create_file(sii9022.dev, &dev_attr_cable_state);
		if (ret < 0)
			dev_warn(&sii9022.client->dev,
				"SII9022: cound not crate sys node\n");
	}

	fb_register_client(&nb);

	for (i = 0; i < num_registered_fb; i++) {
		/* assume sii9022 on DI0 only */
		if (strcmp(registered_fb[i]->fix.id, "DISP3 BG") == 0) {
			sii9022.fbi = registered_fb[i];

			acquire_console_sem();
			fb_blank(sii9022.fbi, FB_BLANK_POWERDOWN);
			release_console_sem();

			sii9022_setup(sii9022.fbi);

			/* primary display? */
			if (i == 0) {
				acquire_console_sem();
				fb_blank(sii9022.fbi, FB_BLANK_UNBLANK);
				release_console_sem();

				fb_show_logo(sii9022.fbi, 0);
			}
		}
	}

	return 0;
}

static int __devexit sii9022_remove(struct i2c_client *client)
{
	fb_unregister_client(&nb);
	sii9022_poweroff();
	return 0;
}

static int sii9022_suspend(struct i2c_client *client, pm_message_t message)
{
	/*TODO*/
	return 0;
}

static int sii9022_resume(struct i2c_client *client)
{
	/*TODO*/
	return 0;
}

static void sii9022_poweron(void)
{
	/* Turn on DVI or HDMI */
	if (sii9022.edid_cfg.hdmi_cap)
		i2c_smbus_write_byte_data(sii9022.client, 0x1A, 0x01);
	else
		i2c_smbus_write_byte_data(sii9022.client, 0x1A, 0x00);
	return;
}

static void sii9022_poweroff(void)
{
	/* disable tmds before changing resolution */
	if (sii9022.edid_cfg.hdmi_cap)
		i2c_smbus_write_byte_data(sii9022.client, 0x1A, 0x11);
	else
		i2c_smbus_write_byte_data(sii9022.client, 0x1A, 0x10);

	return;
}

static const struct i2c_device_id sii9022_id[] = {
	{ "sii9022", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sii9022_id);

static struct i2c_driver sii9022_i2c_driver = {
	.driver = {
		   .name = "sii9022",
		   },
	.probe = sii9022_probe,
	.remove = sii9022_remove,
	.suspend = sii9022_suspend,
	.resume = sii9022_resume,
	.id_table = sii9022_id,
};

static struct file_operations sii9022_fops = {
	.owner = THIS_MODULE,
};

static int __init sii9022_init(void)
{
	int ret;

	memset(&sii9022, 0, sizeof(sii9022));

	sii9022.major = register_chrdev(0, "sii9022", &sii9022_fops);
	if (sii9022.major < 0) {
		printk(KERN_ERR
			"Unable to register Sii9022 as a char device\n");
		return sii9022.major;
	}

	sii9022.class = class_create(THIS_MODULE, "sii9022");
	if (IS_ERR(sii9022.class)) {
		printk(KERN_ERR "Unable to create class for Sii9022\n");
		ret = PTR_ERR(sii9022.class);
		goto err1;
	}

	sii9022.dev = device_create(sii9022.class, NULL,
			MKDEV(sii9022.major, 0), NULL, "sii9022");

	if (IS_ERR(sii9022.dev)) {
		printk(KERN_ERR "Unable to create class device for sii9022\n");
		ret = PTR_ERR(sii9022.dev);
		goto err2;
	}

	return i2c_add_driver(&sii9022_i2c_driver);
err2:
	class_destroy(sii9022.class);
err1:
	unregister_chrdev(sii9022.major, "sii9022");
	return ret;
}

static void __exit sii9022_exit(void)
{
	i2c_del_driver(&sii9022_i2c_driver);
	device_destroy(sii9022.class, MKDEV(sii9022.major, 0));
	class_destroy(sii9022.class);
	unregister_chrdev(sii9022.major, "sii9022");
}

module_init(sii9022_init);
module_exit(sii9022_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("SI9022 DVI/HDMI driver");
MODULE_LICENSE("GPL");
