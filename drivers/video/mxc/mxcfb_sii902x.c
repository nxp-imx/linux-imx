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
 * @file mxcfb_sii902x.c
 *
 * @brief MXC Frame buffer driver for SII902x
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
#include <linux/regulator/consumer.h>
#include <linux/i2c.h>
#include <linux/mxcfb.h>
#include <linux/fsl_devices.h>
#include <linux/interrupt.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/mxc_edid.h>

#define IPU_DISP_PORT 0
#define SII_EDID_LEN	256
#define MXC_ENABLE	1
#define MXC_DISABLE	2
static int g_enable_hdmi;

struct sii902x_data {
	struct platform_device *pdev;
	struct i2c_client *client;
	struct delayed_work det_work;
	struct fb_info *fbi;
	struct mxc_edid_cfg edid_cfg;
	u8 cable_plugin;
	u8 edid[SII_EDID_LEN];
	bool waiting_for_fb;
} sii902x;

static void sii902x_poweron(void);
static void sii902x_poweroff(void);
static void (*sii902x_reset) (void);

static __attribute__ ((unused)) void dump_regs(u8 reg, int len)
{
	u8 buf[50];
	int i;

	i2c_smbus_read_i2c_block_data(sii902x.client, reg, len, buf);
	for (i = 0; i < len; i++)
		dev_dbg(&sii902x.client->dev, "reg[0x%02X]: 0x%02X\n",
				i+reg, buf[i]);
}

static ssize_t sii902x_show_name(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	strcpy(buf, sii902x.fbi->fix.id);
	sprintf(buf+strlen(buf), "\n");

	return strlen(buf);
}

static DEVICE_ATTR(fb_name, S_IRUGO, sii902x_show_name, NULL);

static ssize_t sii902x_show_state(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	if (sii902x.cable_plugin == 0)
		strcpy(buf, "plugout\n");
	else
		strcpy(buf, "plugin\n");

	return strlen(buf);
}

static DEVICE_ATTR(cable_state, S_IRUGO, sii902x_show_state, NULL);

static ssize_t sii902x_show_edid(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i, j, len = 0;

	for (j = 0; j < SII_EDID_LEN/16; j++) {
		for (i = 0; i < 16; i++)
			len += sprintf(buf+len, "0x%02X ",
					sii902x.edid[j*16 + i]);
		len += sprintf(buf+len, "\n");
	}

	return len;
}

static DEVICE_ATTR(edid, S_IRUGO, sii902x_show_edid, NULL);

static void sii902x_setup(struct fb_info *fbi)
{
	u16 data[4];
	u32 refresh;
	u8 *tmp;
	int i;

	dev_dbg(&sii902x.client->dev, "Sii902x: setup..\n");

	/* Power up */
	i2c_smbus_write_byte_data(sii902x.client, 0x1E, 0x00);

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
		i2c_smbus_write_byte_data(sii902x.client, i, tmp[i]);

	/* input bus/pixel: full pixel wide (24bit), rising edge */
	i2c_smbus_write_byte_data(sii902x.client, 0x08, 0x70);
	/* Set input format to RGB */
	i2c_smbus_write_byte_data(sii902x.client, 0x09, 0x00);
	/* set output format to RGB */
	i2c_smbus_write_byte_data(sii902x.client, 0x0A, 0x00);
	/* audio setup */
	i2c_smbus_write_byte_data(sii902x.client, 0x25, 0x00);
	i2c_smbus_write_byte_data(sii902x.client, 0x26, 0x40);
	i2c_smbus_write_byte_data(sii902x.client, 0x27, 0x00);
}

#ifdef CONFIG_FB_MODE_HELPERS
static int sii902x_read_edid(struct fb_info *fbi)
{
	int old, dat, ret, cnt = 100;
	unsigned short addr = 0x50;

	old = i2c_smbus_read_byte_data(sii902x.client, 0x1A);

	i2c_smbus_write_byte_data(sii902x.client, 0x1A, old | 0x4);
	do {
		cnt--;
		msleep(10);
		dat = i2c_smbus_read_byte_data(sii902x.client, 0x1A);
	} while ((!(dat & 0x2)) && cnt);

	if (!cnt) {
		ret = -1;
		goto done;
	}

	i2c_smbus_write_byte_data(sii902x.client, 0x1A, old | 0x06);

	/* edid reading */
	ret = mxc_edid_read(sii902x.client->adapter, addr,
				sii902x.edid, &sii902x.edid_cfg, fbi);

	cnt = 100;
	do {
		cnt--;
		i2c_smbus_write_byte_data(sii902x.client, 0x1A, old & ~0x6);
		msleep(10);
		dat = i2c_smbus_read_byte_data(sii902x.client, 0x1A);
	} while ((dat & 0x6) && cnt);

	if (!cnt)
		ret = -1;

done:
	i2c_smbus_write_byte_data(sii902x.client, 0x1A, old);
	return ret;
}
#else
static int sii902x_read_edid(struct fb_info *fbi)
{
	return -1;
}
#endif

static void det_worker(struct work_struct *work)
{
	int dat;
	char event_string[16];
	char *envp[] = { event_string, NULL };

	dat = i2c_smbus_read_byte_data(sii902x.client, 0x3D);
	if (dat & 0x1) {
		/* cable connection changes */
		if (dat & 0x4) {
			sii902x.cable_plugin = 1;
			sprintf(event_string, "EVENT=plugin");

			/* make sure fb is powerdown */
			acquire_console_sem();
			fb_blank(sii902x.fbi, FB_BLANK_POWERDOWN);
			release_console_sem();

			if (sii902x_read_edid(sii902x.fbi) < 0)
				dev_err(&sii902x.client->dev,
					"Sii902x: read edid fail\n");
			else {
				if (sii902x.fbi->monspecs.modedb_len > 0) {
					int i;
					const struct fb_videomode *mode;
					struct fb_videomode m;

					fb_destroy_modelist(&sii902x.fbi->modelist);

					for (i = 0; i < sii902x.fbi->monspecs.modedb_len; i++) {
						/*FIXME now we do not support interlaced mode */
						if (!(sii902x.fbi->monspecs.modedb[i].vmode & FB_VMODE_INTERLACED))
							fb_add_videomode(&sii902x.fbi->monspecs.modedb[i],
									&sii902x.fbi->modelist);
					}

					fb_var_to_videomode(&m, &sii902x.fbi->var);
					mode = fb_find_nearest_mode(&m,
							&sii902x.fbi->modelist);

					fb_videomode_to_var(&sii902x.fbi->var, mode);

					sii902x.fbi->var.activate |= FB_ACTIVATE_FORCE;
					acquire_console_sem();
					sii902x.fbi->flags |= FBINFO_MISC_USEREVENT;
					fb_set_var(sii902x.fbi, &sii902x.fbi->var);
					sii902x.fbi->flags &= ~FBINFO_MISC_USEREVENT;
					release_console_sem();
				}

				acquire_console_sem();
				fb_blank(sii902x.fbi, FB_BLANK_UNBLANK);
				release_console_sem();
			}
		} else {
			sii902x.cable_plugin = 0;
			sprintf(event_string, "EVENT=plugout");
			acquire_console_sem();
			fb_blank(sii902x.fbi, FB_BLANK_POWERDOWN);
			release_console_sem();
		}
		kobject_uevent_env(&sii902x.pdev->dev.kobj, KOBJ_CHANGE, envp);
	}
	i2c_smbus_write_byte_data(sii902x.client, 0x3D, dat);
}

static irqreturn_t sii902x_detect_handler(int irq, void *data)
{
	if (sii902x.fbi)
		schedule_delayed_work(&(sii902x.det_work), msecs_to_jiffies(20));
	else
		sii902x.waiting_for_fb = true;

	return IRQ_HANDLED;
}

static int sii902x_fb_event(struct notifier_block *nb, unsigned long val, void *v)
{
	struct fb_event *event = v;
	struct fb_info *fbi = event->info;

	switch (val) {
	case FB_EVENT_FB_REGISTERED:
		if (sii902x.fbi == NULL) {
			sii902x.fbi = fbi;
			if (sii902x.waiting_for_fb)
				det_worker(NULL);
		}
		fb_show_logo(fbi, 0);
		break;
	case FB_EVENT_MODE_CHANGE:
		sii902x_setup(fbi);
		break;
	case FB_EVENT_BLANK:
		if (*((int *)event->data) == FB_BLANK_UNBLANK)
			sii902x_poweron();
		else
			sii902x_poweroff();
		break;
	}
	return 0;
}

static struct notifier_block nb = {
	.notifier_call = sii902x_fb_event,
};

static int __devinit sii902x_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int i, dat, ret;
	struct mxc_lcd_platform_data *plat = client->dev.platform_data;
	struct fb_info edid_fbi;

	if (plat->boot_enable &&
		!g_enable_hdmi)
		g_enable_hdmi = MXC_ENABLE;
	if (!g_enable_hdmi)
		g_enable_hdmi = MXC_DISABLE;

	if (g_enable_hdmi == MXC_DISABLE) {
		printk(KERN_WARNING "By setting, SII driver will not be enabled\n");
		return 0;
	}

	sii902x.client = client;

	/* Claim HDMI pins */
	if (plat->get_pins)
		if (!plat->get_pins())
			return -EACCES;

	if (plat->reset) {
		sii902x_reset = plat->reset;
		sii902x_reset();
	}

	/* Set 902x in hardware TPI mode on and jump out of D3 state */
	if (i2c_smbus_write_byte_data(sii902x.client, 0xc7, 0x00) < 0) {
		dev_err(&sii902x.client->dev,
			"Sii902x: cound not find device\n");
		return -ENODEV;
	}

	/* read device ID */
	for (i = 10; i > 0; i--) {
		dat = i2c_smbus_read_byte_data(sii902x.client, 0x1B);
		printk(KERN_DEBUG "Sii902x: read id = 0x%02X", dat);
		if (dat == 0xb0) {
			dat = i2c_smbus_read_byte_data(sii902x.client, 0x1C);
			printk(KERN_DEBUG "-0x%02X", dat);
			dat = i2c_smbus_read_byte_data(sii902x.client, 0x1D);
			printk(KERN_DEBUG "-0x%02X", dat);
			dat = i2c_smbus_read_byte_data(sii902x.client, 0x30);
			printk(KERN_DEBUG "-0x%02X\n", dat);
			break;
		}
	}
	if (i == 0) {
		dev_err(&sii902x.client->dev,
			"Sii902x: cound not find device\n");
		return -ENODEV;
	}

	/* try to read edid */
	ret = sii902x_read_edid(&edid_fbi);
	if (ret < 0)
		dev_warn(&sii902x.client->dev, "Can not read edid\n");

#if defined(CONFIG_MXC_IPU_V3) && defined(CONFIG_FB_MXC_SYNC_PANEL)
	if (ret >= 0)
		mxcfb_register_mode(IPU_DISP_PORT, edid_fbi.monspecs.modedb,
				edid_fbi.monspecs.modedb_len, MXC_DISP_DDC_DEV);
#endif
#if defined(CONFIG_FB_MXC_ELCDIF_FB)
	if (ret >= 0)
		mxcfb_elcdif_register_mode(edid_fbi.monspecs.modedb,
				edid_fbi.monspecs.modedb_len, MXC_DISP_DDC_DEV);
#endif

	if (sii902x.client->irq) {
		ret = request_irq(sii902x.client->irq, sii902x_detect_handler,
				IRQF_TRIGGER_FALLING,
				"SII902x_det", &sii902x);
		if (ret < 0)
			dev_warn(&sii902x.client->dev,
				"Sii902x: cound not request det irq %d\n",
				sii902x.client->irq);
		else {
			/*enable cable hot plug irq*/
			i2c_smbus_write_byte_data(sii902x.client, 0x3c, 0x01);
			INIT_DELAYED_WORK(&(sii902x.det_work), det_worker);
		}
		ret = device_create_file(&sii902x.pdev->dev, &dev_attr_fb_name);
		if (ret < 0)
			dev_warn(&sii902x.client->dev,
				"Sii902x: cound not create sys node for fb name\n");
		ret = device_create_file(&sii902x.pdev->dev, &dev_attr_cable_state);
		if (ret < 0)
			dev_warn(&sii902x.client->dev,
				"Sii902x: cound not create sys node for cable state\n");
		ret = device_create_file(&sii902x.pdev->dev, &dev_attr_edid);
		if (ret < 0)
			dev_warn(&sii902x.client->dev,
				"Sii902x: cound not create sys node for edid\n");
	}

	fb_register_client(&nb);

	return 0;
}

static int __devexit sii902x_remove(struct i2c_client *client)
{
	struct mxc_lcd_platform_data *plat = sii902x.client->dev.platform_data;

	fb_unregister_client(&nb);
	sii902x_poweroff();

	/* Release HDMI pins */
	if (plat->put_pins)
		plat->put_pins();

	return 0;
}

static int sii902x_suspend(struct i2c_client *client, pm_message_t message)
{
	/*TODO*/
	return 0;
}

static int sii902x_resume(struct i2c_client *client)
{
	/*TODO*/
	return 0;
}

static void sii902x_poweron(void)
{
	struct mxc_lcd_platform_data *plat = sii902x.client->dev.platform_data;

	/* Enable pins to HDMI */
	if (plat->enable_pins)
		plat->enable_pins();

	/* Turn on DVI or HDMI */
	if (sii902x.edid_cfg.hdmi_cap)
		i2c_smbus_write_byte_data(sii902x.client, 0x1A, 0x01);
	else
		i2c_smbus_write_byte_data(sii902x.client, 0x1A, 0x00);
	return;
}

static void sii902x_poweroff(void)
{
	struct mxc_lcd_platform_data *plat = sii902x.client->dev.platform_data;

	/* disable tmds before changing resolution */
	if (sii902x.edid_cfg.hdmi_cap)
		i2c_smbus_write_byte_data(sii902x.client, 0x1A, 0x11);
	else
		i2c_smbus_write_byte_data(sii902x.client, 0x1A, 0x10);

	/* Disable pins to HDMI */
	if (plat->disable_pins)
		plat->disable_pins();

	return;
}

static const struct i2c_device_id sii902x_id[] = {
	{ "sii902x", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sii902x_id);

static struct i2c_driver sii902x_i2c_driver = {
	.driver = {
		   .name = "sii902x",
		   },
	.probe = sii902x_probe,
	.remove = sii902x_remove,
	.suspend = sii902x_suspend,
	.resume = sii902x_resume,
	.id_table = sii902x_id,
};

static int __init sii902x_init(void)
{
	int ret;

	memset(&sii902x, 0, sizeof(sii902x));

	sii902x.pdev = platform_device_register_simple("sii902x", 0, NULL, 0);
	if (IS_ERR(sii902x.pdev)) {
		printk(KERN_ERR
				"Unable to register Sii902x as a platform device\n");
		ret = PTR_ERR(sii902x.pdev);
		goto err;
	}

	return i2c_add_driver(&sii902x_i2c_driver);
err:
	return ret;
}

static void __exit sii902x_exit(void)
{
	i2c_del_driver(&sii902x_i2c_driver);
	platform_device_unregister(sii902x.pdev);
}

static int __init enable_hdmi_setup(char *options)
{
	if (!strcmp(options, "=off"))
		g_enable_hdmi = MXC_DISABLE;
	else
		g_enable_hdmi = MXC_ENABLE;

	return 1;
}
__setup("hdmi", enable_hdmi_setup);

module_init(sii902x_init);
module_exit(sii902x_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("SII902x DVI/HDMI driver");
MODULE_LICENSE("GPL");
