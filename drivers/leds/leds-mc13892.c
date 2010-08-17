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

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/pmic_light.h>

static void mc13892_led_set(struct led_classdev *led_cdev,
			    enum led_brightness value)
{
	struct platform_device *dev = to_platform_device(led_cdev->dev->parent);
	int led_ch;

	switch (dev->id) {
	case 'r':
		led_ch = LIT_RED;
		break;
	case 'g':
		led_ch = LIT_GREEN;
		break;
	case 'b':
		led_ch = LIT_BLUE;
		break;
	default:
		return;
	}

	/* set current with medium value, in case current is too large */
	mc13892_bklit_set_current(led_ch, LIT_CURR_12);
	/* max duty cycle is 63, brightness needs to be divided by 4 */
	mc13892_bklit_set_dutycycle(led_ch, value / 4);

}

static int mc13892_led_remove(struct platform_device *dev)
{
	struct led_classdev *led_cdev = platform_get_drvdata(dev);

	led_classdev_unregister(led_cdev);
	kfree(led_cdev->name);
	kfree(led_cdev);

	return 0;
}

#define LED_NAME_LEN	16

static int mc13892_led_probe(struct platform_device *dev)
{
	int ret;
	struct led_classdev *led_cdev;
	char *name;

	led_cdev = kzalloc(sizeof(struct led_classdev), GFP_KERNEL);
	if (led_cdev == NULL) {
		dev_err(&dev->dev, "No memory for device\n");
		return -ENOMEM;
	}
	name = kzalloc(LED_NAME_LEN, GFP_KERNEL);
	if (name == NULL) {
		dev_err(&dev->dev, "No memory for device\n");
		ret = -ENOMEM;
		goto exit_err;
	}

	strcpy(name, dev->name);
	ret = strlen(dev->name);
	if (ret > LED_NAME_LEN - 2) {
		dev_err(&dev->dev, "led name is too long\n");
		goto exit_err1;
	}
	name[ret] = dev->id;
	name[ret + 1] = '\0';
	led_cdev->name = name;
	led_cdev->brightness_set = mc13892_led_set;

	ret = led_classdev_register(&dev->dev, led_cdev);
	if (ret < 0) {
		dev_err(&dev->dev, "led_classdev_register failed\n");
		goto exit_err1;
	}

	platform_set_drvdata(dev, led_cdev);

	return 0;
      exit_err1:
	kfree(led_cdev->name);
      exit_err:
	kfree(led_cdev);
	return ret;
}

#ifdef CONFIG_PM
static int mc13892_led_suspend(struct platform_device *dev, pm_message_t state)
{
	struct led_classdev *led_cdev = platform_get_drvdata(dev);

	led_classdev_suspend(led_cdev);
	return 0;
}

static int mc13892_led_resume(struct platform_device *dev)
{
	struct led_classdev *led_cdev = platform_get_drvdata(dev);

	led_classdev_resume(led_cdev);
	return 0;
}
#else
#define mc13892_led_suspend NULL
#define mc13892_led_resume NULL
#endif

static struct platform_driver mc13892_led_driver = {
	.probe = mc13892_led_probe,
	.remove = mc13892_led_remove,
	.suspend = mc13892_led_suspend,
	.resume = mc13892_led_resume,
	.driver = {
		   .name = "pmic_leds",
		   .owner = THIS_MODULE,
		   },
};

static int __init mc13892_led_init(void)
{
	return platform_driver_register(&mc13892_led_driver);
}

static void __exit mc13892_led_exit(void)
{
	platform_driver_unregister(&mc13892_led_driver);
}

module_init(mc13892_led_init);
module_exit(mc13892_led_exit);

MODULE_DESCRIPTION("Led driver for PMIC mc13892");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
