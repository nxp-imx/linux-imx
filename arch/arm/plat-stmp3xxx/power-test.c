/*
 * Power consumption test module
 *
 * Author: Dmitrij Frasenyak <sed@embeddedalley.com>
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

static struct regulator *reg;
static struct regulator *freg;

static struct timer_list pt_timer;
static int timer_delay = 5*60*1000; /* 5min */
static DEFINE_MUTEX(run_mutex);


#define REG_GET() do {\
		if (!reg) {\
			reg = regulator_get(NULL, "power-test-1");\
			if (!reg || IS_ERR(reg)) {\
				reg = NULL ; return -ENODEV;\
			} \
		} \
} while (0);

static void timer_func(unsigned long data)
{
	regulator_set_current_limit(reg, 0, 0);
	mutex_unlock(&run_mutex);
}

static ssize_t pt_mode_set(struct device *d, struct device_attribute *attr,
					const char *buf, size_t size)
{
	REG_GET();
	if (buf[0] == 'f')
		regulator_set_mode(reg, REGULATOR_MODE_FAST);
	else
		regulator_set_mode(reg, REGULATOR_MODE_NORMAL);
	return size;
}

static ssize_t pt_mode_show(struct device *d, struct device_attribute *attr,
			      char *buf)
{
	REG_GET();
	if (regulator_get_mode(reg) == REGULATOR_MODE_FAST)
		return snprintf(buf, 5, "fast\n");
	else
		return snprintf(buf, 7, "normal\n");
}

static ssize_t pt_val_fset(struct device *d, struct device_attribute *attr,
					const char *buf, size_t size)
{
	int i, ret;
	ret = sscanf(buf, "%u", &i);
	if (ret != 1)
		return -EINVAL;

	if (!freg) {
		freg = regulator_get(NULL, "stmp3xxx-bl-1");
		if (!freg || IS_ERR(freg)) {
			freg = NULL ; return -ENODEV;
		}
	}
	regulator_set_mode(freg, REGULATOR_MODE_NORMAL);

	if (!regulator_set_current_limit(freg, i, i))
		printk(KERN_ERR "got backlight reg\n");
	else
		printk(KERN_ERR "failed to get backlight reg");

	return size;
}


static ssize_t pt_val_set(struct device *d, struct device_attribute *attr,
					const char *buf, size_t size)
{
	int i, ret;
	REG_GET();

	ret = sscanf(buf, "%u", &i);
	if (ret != 1)
		return -EINVAL;

	mutex_lock(&run_mutex);
	if (!regulator_set_current_limit(reg, i, i)) {
		mod_timer(&pt_timer,
			  jiffies + msecs_to_jiffies(timer_delay));
		return size;
	} else {
		mutex_unlock(&run_mutex);
		return -EPERM;
	}

}

static ssize_t pt_val_show(struct device *d, struct device_attribute *attr,
			      char *buf)
{
	REG_GET();
	return sprintf(buf, "%d\n", regulator_get_current_limit(reg));
}

static ssize_t pt_timeout_set(struct device *d, struct device_attribute *attr,
					const char *buf, size_t size)
{
	int i, ret;
	REG_GET();

	ret = sscanf(buf, "%u", &i);
	if (ret != 1)
		return -EINVAL;

	mutex_lock(&run_mutex);
	timer_delay = 1000*i ;
	mutex_unlock(&run_mutex);

	return size;
}

static ssize_t pt_timeout_show(struct device *d, struct device_attribute *attr,
			      char *buf)
{
	REG_GET();
	return sprintf(buf, "%d\n", timer_delay);
}

static DEVICE_ATTR(mode, 0644, pt_mode_show, pt_mode_set);
static DEVICE_ATTR(val, 0644, pt_val_show, pt_val_set);
static DEVICE_ATTR(fval, 0644, pt_val_show, pt_val_fset);
static DEVICE_ATTR(timeout, 0644, pt_timeout_show, pt_timeout_set);

static int stmp3xxx_power_test_remove(struct platform_device *pdev)
{
	if (reg)
		regulator_put(reg);

	device_remove_file(&pdev->dev, &dev_attr_mode);
	device_remove_file(&pdev->dev, &dev_attr_val);
	device_remove_file(&pdev->dev, &dev_attr_timeout);
	device_remove_file(&pdev->dev, &dev_attr_fval);
	return 0;
}

static int stmp3xxx_power_test_probe(struct platform_device *pdev)
{
	int ret;
	init_timer(&pt_timer);
	pt_timer.data = 0;
	pt_timer.function = timer_func;

	ret = device_create_file(&pdev->dev, &dev_attr_mode);
	ret |= device_create_file(&pdev->dev, &dev_attr_val);
	ret |= device_create_file(&pdev->dev, &dev_attr_fval);
	ret |= device_create_file(&pdev->dev, &dev_attr_timeout);
	return ret;
}

static struct platform_driver stmp3xxx_power_test_driver = {
	.probe		= stmp3xxx_power_test_probe,
	.remove		= stmp3xxx_power_test_remove,
	.driver		= {
		.name   = "stmp3xxx-power-test",
		.owner	= THIS_MODULE,
	},
};

struct platform_device stmp3xxx_pt = {
	.name			= "stmp3xxx-power-test",
	.id			= -1,
};

static int __init stmp3xxx_power_test_init(void)
{

	platform_device_register(&stmp3xxx_pt);
	return platform_driver_register(&stmp3xxx_power_test_driver);
}

static void __exit stmp3xxx_power_test_exit(void)
{
	platform_driver_unregister(&stmp3xxx_power_test_driver);
	platform_device_unregister(&stmp3xxx_pt);
}

MODULE_AUTHOR("<sed@embeddedalley.com>");
MODULE_DESCRIPTION("Power test driver");
MODULE_LICENSE("GPL");

module_init(stmp3xxx_power_test_init);
module_exit(stmp3xxx_power_test_exit);
