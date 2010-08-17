/*
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/fb.h>
#include <linux/backlight.h>

#include <linux/pmic_light.h>
#include <linux/pmic_external.h>

/*
#define MXC_MAX_INTENSITY 	255
#define MXC_DEFAULT_INTENSITY 	127
*/
/* workaround for atlas hot issue */
#define MXC_MAX_INTENSITY 	128
#define MXC_DEFAULT_INTENSITY 	64

#define MXC_INTENSITY_OFF 	0

struct mxcbl_dev_data {
	int intensity;
	int suspend;
};

static int mxcbl_set_intensity(struct backlight_device *bd)
{
	int brightness = bd->props.brightness;
	struct mxcbl_dev_data *devdata = dev_get_drvdata(&bd->dev);

	if (bd->props.power != FB_BLANK_UNBLANK)
		brightness = 0;
	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;
	if (devdata->suspend)
		brightness = 0;

	brightness = brightness / 4;
	mc13892_bklit_set_dutycycle(LIT_MAIN, brightness);
	devdata->intensity = brightness;

	return 0;
}

static int mxcbl_get_intensity(struct backlight_device *bd)
{
	struct mxcbl_dev_data *devdata = dev_get_drvdata(&bd->dev);
	return devdata->intensity;
}

static int mxcbl_check_fb(struct backlight_device *bldev, struct fb_info *info)
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
	struct mxcbl_dev_data *devdata;
	struct backlight_properties props;
	pmic_version_t pmic_version;

	pr_debug("mc13892 backlight start probe\n");

	devdata = kzalloc(sizeof(struct mxcbl_dev_data), GFP_KERNEL);
	if (!devdata)
		return -ENOMEM;

	bl_ops.check_fb = mxcbl_check_fb;
	bl_ops.get_brightness = mxcbl_get_intensity;
	bl_ops.update_status = mxcbl_set_intensity;
	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = MXC_MAX_INTENSITY;
	bd = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, devdata,
				       &bl_ops, &props);
	if (IS_ERR(bd)) {
		ret = PTR_ERR(bd);
		goto err0;
	}

	platform_set_drvdata(pdev, bd);

	/* according to LCD spec, current should be 18mA */
	/* workaround for MC13892 TO1.1 crash issue, set current 6mA */
	pmic_version = pmic_get_version();
	if (pmic_version.revision < 20)
		mc13892_bklit_set_current(LIT_MAIN, LIT_CURR_6);
	else
		mc13892_bklit_set_current(LIT_MAIN, LIT_CURR_18);
	bd->props.brightness = MXC_DEFAULT_INTENSITY;
	bd->props.power = FB_BLANK_UNBLANK;
	bd->props.fb_blank = FB_BLANK_UNBLANK;
	backlight_update_status(bd);
	pr_debug("mc13892 backlight probed successfully\n");
	return 0;

      err0:
	kfree(devdata);
	return ret;
}

static int mxcbl_remove(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);
	struct mxcbl_dev_data *devdata = dev_get_drvdata(&bd->dev);

	kfree(devdata);
	backlight_device_unregister(bd);
	return 0;
}

static int mxcbl_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);
	struct mxcbl_dev_data *devdata = dev_get_drvdata(&bd->dev);

	devdata->suspend = 1;
	backlight_update_status(bd);
	return 0;
}

static int mxcbl_resume(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);
	struct mxcbl_dev_data *devdata = dev_get_drvdata(&bd->dev);

	devdata->suspend = 0;
	backlight_update_status(bd);
	return 0;
}

static struct platform_driver mxcbl_driver = {
	.probe = mxcbl_probe,
	.remove = mxcbl_remove,
	.suspend = mxcbl_suspend,
	.resume = mxcbl_resume,
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
