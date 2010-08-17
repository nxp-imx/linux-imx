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
/*!
 * @defgroup IPU_BL MXC IPU Backlight Driver
 */
/*!
 * @file mxc_ipu_bl.c
 *
 * @brief Backlight Driver for IPU PWM on Freescale MXC/i.MX platforms.
 *
 * This file contains API defined in include/linux/clk.h for setting up and
 * retrieving clocks.
 *
 * Based on Sharp's Corgi Backlight Driver
 *
 * @ingroup IPU_BL
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/ipu.h>

#define MXC_MAX_INTENSITY 	255
#define MXC_DEFAULT_INTENSITY 	127
#define MXC_INTENSITY_OFF 	0

struct mxcbl_dev_data {
	int intensity;
};

static int fb_id;

static int mxcbl_send_intensity(struct backlight_device *bd)
{
	int intensity = bd->props.brightness;
	struct mxcbl_dev_data *devdata = dev_get_drvdata(&bd->dev);

	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		intensity = 0;

	ipu_sdc_set_brightness(intensity);

	devdata->intensity = intensity;
	return 0;
}

static int mxcbl_get_intensity(struct backlight_device *bd)
{
	struct mxcbl_dev_data *devdata = dev_get_drvdata(&bd->dev);
	return devdata->intensity;
}

static int mxcbl_check_fb(struct backlight_device *bldev, struct fb_info *info)
{
	int id = info->fix.id[4] - '0';
	if (id == fb_id) {
		if ((id == 3) && !strcmp(info->fix.id, "DISP3 FG")) {
			return 0;
		}
		return 1;
	}
	return 0;
}

static struct backlight_ops mxcbl_ops = {
	.get_brightness = mxcbl_get_intensity,
	.update_status = mxcbl_send_intensity,
	.check_fb = mxcbl_check_fb,
};

static int __init mxcbl_probe(struct platform_device *pdev)
{
	struct backlight_device *bd;
	struct mxcbl_dev_data *devdata;
	struct backlight_properties props;
	int ret = 0;

	devdata = kzalloc(sizeof(struct mxcbl_dev_data), GFP_KERNEL);
	if (!devdata)
		return -ENOMEM;
	fb_id = (int)pdev->dev.platform_data;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = MXC_MAX_INTENSITY;
	bd = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, devdata,
				       &mxcbl_ops, &props);
	if (IS_ERR(bd)) {
		ret = PTR_ERR(bd);
		goto err0;
	}
	platform_set_drvdata(pdev, bd);

	bd->props.brightness = MXC_DEFAULT_INTENSITY;
	bd->props.power = FB_BLANK_UNBLANK;
	bd->props.fb_blank = FB_BLANK_UNBLANK;
	backlight_update_status(bd);

	printk("MXC Backlight Device %s Initialized.\n", dev_name(&pdev->dev));
	return 0;
      err0:
	kfree(devdata);
	return ret;
}

static int mxcbl_remove(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);

	bd->props.brightness = MXC_INTENSITY_OFF;
	backlight_update_status(bd);

	backlight_device_unregister(bd);

	return 0;
}

static struct platform_driver mxcbl_driver = {
	.probe = mxcbl_probe,
	.remove = mxcbl_remove,
	.driver = {
		   .name = "mxc_ipu_bl",
		   },
};

static int __init mxcbl_init(void)
{
	return platform_driver_register(&mxcbl_driver);
}

static void __exit mxcbl_exit(void)
{
	platform_driver_unregister(&mxcbl_driver);
}

late_initcall(mxcbl_init);
module_exit(mxcbl_exit);

MODULE_DESCRIPTION("Freescale MXC/i.MX IPU PWM Backlight Driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
