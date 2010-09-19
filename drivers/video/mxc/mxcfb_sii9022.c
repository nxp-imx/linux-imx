/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <asm/mach-types.h>
#include <mach/hardware.h>

static struct i2c_client *sii9022_client;
static void sii9022_poweron(void);
static void sii9022_poweroff(void);
static void (*sii9022_reset) (void);

static __attribute__ ((unused)) void dump_regs(u8 reg, int len)
{
	u8 buf[50];
	int i;

	i2c_smbus_read_i2c_block_data(sii9022_client, reg, len, buf);
	for (i = 0; i < len; i++)
		dev_dbg(&sii9022_client->dev, "reg[0x%02X]: 0x%02X\n",
				i+reg, buf[i]);
}

static void sii9022_setup(struct fb_info *fbi)
{
	u16 data[4];
	u32 refresh;
	u8 *tmp;
	int i;

	dev_dbg(&sii9022_client->dev, "SII9022: setup..\n");

	/* Power up */
	i2c_smbus_write_byte_data(sii9022_client, 0x1E, 0x00);

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
		i2c_smbus_write_byte_data(sii9022_client, i, tmp[i]);

	/* input bus/pixel: full pixel wide (24bit), rising edge */
	i2c_smbus_write_byte_data(sii9022_client, 0x08, 0x70);
	/* Set input format to RGB */
	i2c_smbus_write_byte_data(sii9022_client, 0x09, 0x00);
	/* set output format to RGB */
	i2c_smbus_write_byte_data(sii9022_client, 0x0A, 0x00);
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

	sii9022_client = client;

	if (plat->reset) {
		sii9022_reset = plat->reset;
		sii9022_reset();
	}

	/* Set 9022 in hardware TPI mode on and jump out of D3 state */
	if (i2c_smbus_write_byte_data(sii9022_client, 0xc7, 0x00) < 0) {
		dev_err(&sii9022_client->dev,
			"SII9022: cound not find device\n");
		return -ENODEV;
	}

	/* read device ID */
	for (i = 10; i > 0; i--) {
		dat = i2c_smbus_read_byte_data(sii9022_client, 0x1B);
		printk(KERN_DEBUG "Sii9022: read id = 0x%02X", dat);
		if (dat == 0xb0) {
			dat = i2c_smbus_read_byte_data(sii9022_client, 0x1C);
			printk(KERN_DEBUG "-0x%02X", dat);
			dat = i2c_smbus_read_byte_data(sii9022_client, 0x1D);
			printk(KERN_DEBUG "-0x%02X", dat);
			dat = i2c_smbus_read_byte_data(sii9022_client, 0x30);
			printk(KERN_DEBUG "-0x%02X\n", dat);
			break;
		}
	}
	if (i == 0) {
		dev_err(&sii9022_client->dev,
			"SII9022: cound not find device\n");
		return -ENODEV;
	}

	fb_register_client(&nb);

	for (i = 0; i < num_registered_fb; i++) {
		/* assume sii9022 on DI0 only */
		if ((strcmp(registered_fb[i]->fix.id, "DISP3 BG") == 0)
			&& (i == 0)) {
			struct fb_info *fbi = registered_fb[i];
			acquire_console_sem();
			fb_blank(fbi, FB_BLANK_POWERDOWN);
			release_console_sem();

			sii9022_setup(fbi);

			acquire_console_sem();
			fb_blank(fbi, FB_BLANK_UNBLANK);
			release_console_sem();

			fb_show_logo(fbi, 0);
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
	i2c_smbus_write_byte_data(sii9022_client, 0x1A, 0x01);
	return;
}

static void sii9022_poweroff(void)
{
	/* disable tmds before changing resolution */
	i2c_smbus_write_byte_data(sii9022_client, 0x1A, 0x11);

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

static int __init sii9022_init(void)
{
	return i2c_add_driver(&sii9022_i2c_driver);
}

static void __exit sii9022_exit(void)
{
	i2c_del_driver(&sii9022_i2c_driver);
}

module_init(sii9022_init);
module_exit(sii9022_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("SI9022 DVI/HDMI driver");
MODULE_LICENSE("GPL");
