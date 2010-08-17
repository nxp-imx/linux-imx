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
 * @defgroup PMIC_BL MXC PMIC Backlight Driver
 */
/*!
 * @file mxc_pmic_bl.c
 *
 * @brief PMIC Backlight Driver for Freescale MXC/i.MX platforms.
 *
 * This file contains API defined in include/linux/clk.h for setting up and
 * retrieving clocks.
 *
 * Based on Sharp's Corgi Backlight Driver
 *
 * @ingroup PMIC_BL
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/pmic_light.h>

#include <mach/pmic_power.h>

#define MXC_MAX_INTENSITY 	255
#define MXC_DEFAULT_INTENSITY 	127
#define MXC_INTENSITY_OFF 	0

struct mxcbl_dev_data {
	int bl_id;
	int intensity;
	struct backlight_ops bl_ops;
};

static int pmic_bl_use_count;
static int main_fb_id;
static int sec_fb_id;

static int mxcbl_send_intensity(struct backlight_device *bd)
{
	int intensity = bd->props.brightness;
	struct mxcbl_dev_data *devdata = dev_get_drvdata(&bd->dev);

	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		intensity = 0;

	intensity = intensity / 16;
	pmic_bklit_set_dutycycle(devdata->bl_id, intensity);

	devdata->intensity = intensity;
	return 0;
}

static int mxcbl_get_intensity(struct backlight_device *bd)
{
	struct mxcbl_dev_data *devdata = dev_get_drvdata(&bd->dev);
	return devdata->intensity;
}

static int mxcbl_check_main_fb(struct fb_info *info)
{
	int id = info->fix.id[4] - '0';

	if (id == main_fb_id) {
		return 1;
	} else {
		return 0;
	}
}

static int mxcbl_check_sec_fb(struct backlight_device *bldev, struct fb_info *info)
{
	int id = info->fix.id[4] - '0';

	if (id == sec_fb_id) {
		return 1;
	} else {
		return 0;
	}
}

static int __init mxcbl_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct backlight_device *bd;
	struct mxcbl_dev_data *devdata;
	struct backlight_properties props;

	devdata = kzalloc(sizeof(struct mxcbl_dev_data), GFP_KERNEL);
	if (!devdata)
		return -ENOMEM;
	devdata->bl_id = pdev->id;

	if (pdev->id == 0) {
		devdata->bl_ops.check_fb = mxcbl_check_main_fb;
		main_fb_id = (int)pdev->dev.platform_data;
	} else {
		devdata->bl_ops.check_fb = mxcbl_check_sec_fb;
		sec_fb_id = (int)pdev->dev.platform_data;
	}

	devdata->bl_ops.get_brightness = mxcbl_get_intensity;
	devdata->bl_ops.update_status = mxcbl_send_intensity,
	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = MXC_MAX_INTENSITY;
	    bd =
	    backlight_device_register(dev_name(&pdev->dev), &pdev->dev, devdata,
				      &devdata->bl_ops, &props);
	if (IS_ERR(bd)) {
		ret = PTR_ERR(bd);
		goto err0;
	}

	platform_set_drvdata(pdev, bd);

	if (pmic_bl_use_count++ == 0) {
		pmic_power_regulator_on(SW_SW3);
		pmic_power_regulator_set_lp_mode(SW_SW3, LOW_POWER_CTRL_BY_PIN);

		pmic_bklit_tcled_master_enable();
		pmic_bklit_enable_edge_slow();
		pmic_bklit_set_cycle_time(0);
	}

	pmic_bklit_set_current(devdata->bl_id, 7);
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

	if (--pmic_bl_use_count == 0) {
		pmic_bklit_tcled_master_disable();

		pmic_power_regulator_off(SW_SW3);
		pmic_power_regulator_set_lp_mode(SW_SW3, LOW_POWER_CTRL_BY_PIN);
	}

	backlight_device_unregister(bd);

	printk("MXC Backlight Driver Unloaded\n");

	return 0;
}

static struct platform_driver mxcbl_driver = {
	.probe = mxcbl_probe,
	.remove = mxcbl_remove,
	.driver = {
		   .name = "mxc_pmic_bl",
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

module_init(mxcbl_init);
module_exit(mxcbl_exit);

MODULE_DESCRIPTION("Freescale MXC/i.MX PMIC Backlight Driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
