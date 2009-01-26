/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/fb.h>
#include <linux/backlight.h>

#include <mach/pmic_light.h>

/*
#define MXC_MAX_INTENSITY 	255
#define MXC_DEFAULT_INTENSITY 	127
*/
/* workaround for atlas hot issue */
#define MXC_MAX_INTENSITY 	128
#define MXC_DEFAULT_INTENSITY 	64

#define MXC_INTENSITY_OFF 	0

static int intensity;

static int mxcbl_set_intensity(struct backlight_device *bd)
{
	int brightness = bd->props.brightness;

	if (bd->props.power != FB_BLANK_UNBLANK)
		brightness = 0;
	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	brightness = brightness / 4;
	mc13892_bklit_set_dutycycle(LIT_MAIN, brightness);

	intensity = brightness;

	return 0;
}

static int mxcbl_get_intensity(struct backlight_device *bd)
{
	return intensity;
}

static int mxcbl_check_fb(struct fb_info *info)
{
	char *id = info->fix.id;

	if (!strcmp(id, "DISP3 BG"))
		return 1;
	else
		return 0;
}

static struct backlight_ops bl_ops;

static int __init mxcbl_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct backlight_device *bd;

	pr_debug("mc13892 backlight start probe\n");

	bl_ops.check_fb = mxcbl_check_fb;
	bl_ops.get_brightness = mxcbl_get_intensity;
	bl_ops.update_status = mxcbl_set_intensity;
	bd = backlight_device_register(pdev->dev.bus_id, &pdev->dev, NULL,
				       &bl_ops);
	if (IS_ERR(bd)) {
		ret = PTR_ERR(bd);
		return ret;
	}

	platform_set_drvdata(pdev, bd);

	/* according to LCD spec, current should be 18mA */
	/* workaround for atlas hot issue, set current 15mA */
	mc13892_bklit_set_current(LIT_MAIN, LIT_CURR_15);
	bd->props.brightness = MXC_DEFAULT_INTENSITY;
	bd->props.max_brightness = MXC_MAX_INTENSITY;
	bd->props.power = FB_BLANK_UNBLANK;
	bd->props.fb_blank = FB_BLANK_UNBLANK;
	backlight_update_status(bd);
	pr_debug("mc13892 backlight probed successfully\n");

	return 0;
}

static int mxcbl_remove(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);

	backlight_device_unregister(bd);

	return 0;
}

static struct platform_driver mxcbl_driver = {
	.probe = mxcbl_probe,
	.remove = mxcbl_remove,
	.driver = {
		   .name = "mxc_mc13892_bl",
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
