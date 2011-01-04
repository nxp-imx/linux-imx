#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/uaccess.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/watchdog.h>
#include <linux/types.h>
#include <linux/kernel.h>


#include <linux/mfd/da9052/reg.h>
#include <linux/mfd/da9052/da9052.h>
#include <linux/mfd/da9052/wdt.h>

#define DRIVER_NAME "da9052-wdt"

#define	DA9052_STROBING_FILTER_ENABLE		0x0001
#define	DA9052_STROBING_FILTER_DISABLE		0x0002
#define	DA9052_SET_STROBING_MODE_MANUAL		0x0004
#define	DA9052_SET_STROBING_MODE_AUTO		0x0008

#define KERNEL_MODULE				0
#define ENABLE					1
#define DISABLE					0

static u8 sm_strobe_filter_flag = DISABLE;
static u8 sm_strobe_mode_flag = DA9052_STROBE_MANUAL;
static u32 sm_mon_interval = DA9052_ADC_TWDMIN_TIME;
static u8 sm_str_req = DISABLE;
static u8 da9052_sm_scale = DA9052_WDT_DISABLE;
module_param(sm_strobe_filter_flag,  byte, 0);
MODULE_PARM_DESC(sm_strobe_filter_flag,
		"DA9052 SM driver strobe filter flag default = DISABLE");

module_param(sm_strobe_mode_flag, byte, 0);
MODULE_PARM_DESC(sm_strobe_mode_flag,
		"DA9052 SM driver watchdog strobing mode default\
		= DA9052_STROBE_MANUAL");

module_param(da9052_sm_scale, byte, 0);
MODULE_PARM_DESC(da9052_sm_scale,
		"DA9052 SM driver scaling value used to calculate the\
		time for the strobing  filter default = 0");

module_param(sm_str_req, byte, 0);
MODULE_PARM_DESC(sm_str_req,
		"DA9052 SM driver strobe request flag default = DISABLE");

static int nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout,
		"Watchdog cannot be stopped once started (default="
		__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static struct timer_list *monitoring_timer;

struct da9052_wdt {
	struct platform_device *pdev;
	struct da9052 *da9052;
};
static struct miscdevice da9052_wdt_miscdev;
static unsigned long da9052_wdt_users;
static int da9052_wdt_expect_close;

static struct da9052_wdt *get_wdt_da9052(void)
{
	/*return dev_get_drvdata(da9052_wdt_miscdev.parent);*/
	return platform_get_drvdata
			(to_platform_device(da9052_wdt_miscdev.parent));
}

void start_strobing(struct work_struct *work)
{
	struct da9052_ssc_msg msg;
	int ret;
	struct da9052_wdt *wdt = get_wdt_da9052();


	if (NULL == wdt) {
		mod_timer(monitoring_timer, jiffies + sm_mon_interval);
		return;
	}
	msg.addr = DA9052_CONTROLD_REG;
	msg.data = 0;
	da9052_lock(wdt->da9052);
	ret = wdt->da9052->read(wdt->da9052, &msg);
	if (ret) {
		da9052_unlock(wdt->da9052);
		return;
	}
	da9052_unlock(wdt->da9052);

	msg.data = (msg.data | DA9052_CONTROLD_WATCHDOG);
	da9052_lock(wdt->da9052);
	ret = wdt->da9052->write(wdt->da9052, &msg);
	if (ret) {
		da9052_unlock(wdt->da9052);
		return;
	}
	da9052_unlock(wdt->da9052);

	sm_str_req = DISABLE;

	mod_timer(monitoring_timer, jiffies + sm_mon_interval);
	return;
}


void timer_callback(void)
{
	if (((sm_strobe_mode_flag) &&
		(sm_strobe_mode_flag == DA9052_STROBE_MANUAL)) ||
		(sm_strobe_mode_flag == DA9052_STROBE_AUTO)) {
		schedule_work(&strobing_action);
	} else {
		if (sm_strobe_mode_flag == DA9052_STROBE_MANUAL) {
			mod_timer(monitoring_timer, jiffies +
			sm_mon_interval);
		}
	}
}

static int da9052_sm_hw_init(struct da9052_wdt *wdt)
{
	/* Create timer structure */
	monitoring_timer = kzalloc(sizeof(struct timer_list), GFP_KERNEL);
	if (!monitoring_timer)
		return -ENOMEM;

	init_timer(monitoring_timer);
	monitoring_timer->expires = jiffies + sm_mon_interval;
	monitoring_timer->function = (void *)&timer_callback;

	sm_strobe_filter_flag = DA9052_SM_STROBE_CONF;
	sm_strobe_mode_flag = DA9052_STROBE_MANUAL;

	return 0;
}

static int da9052_sm_hw_deinit(struct da9052_wdt *wdt)
{
	struct da9052_ssc_msg msg;
	int ret;

	if (monitoring_timer != NULL)
		del_timer(monitoring_timer);
	kfree(monitoring_timer);

	msg.addr = DA9052_CONTROLD_REG;
	msg.data = 0;

	da9052_lock(wdt->da9052);
	ret = wdt->da9052->read(wdt->da9052, &msg);
	if (ret)
		goto ssc_err;
	da9052_unlock(wdt->da9052);

	msg.data = (msg.data & ~(DA9052_CONTROLD_TWDSCALE));
	da9052_lock(wdt->da9052);
	ret = wdt->da9052->write(wdt->da9052, &msg);
	if (ret)
		goto ssc_err;
	da9052_unlock(wdt->da9052);

	return 0;
ssc_err:
	da9052_unlock(wdt->da9052);
	return -EIO;
}

 s32 da9052_sm_set_strobing_filter(struct da9052_wdt *wdt,
				u8 strobing_filter_state)
 {
	struct da9052_ssc_msg msg;
	int ret = 0;

	msg.addr = DA9052_CONTROLD_REG;
	msg.data = 0;
	da9052_lock(wdt->da9052);
	ret = wdt->da9052->read(wdt->da9052, &msg);
	if (ret)
		goto ssc_err;
	da9052_unlock(wdt->da9052);

	msg.data = (msg.data & DA9052_CONTROLD_TWDSCALE);

	if (strobing_filter_state == ENABLE) {
		sm_strobe_filter_flag = ENABLE;
		if (DA9052_WDT_DISABLE == msg.data) {
			sm_str_req = DISABLE;
			del_timer(monitoring_timer);
			return 0;
		}
		if (DA9052_SCALE_64X == msg.data)
			sm_mon_interval = msecs_to_jiffies(DA9052_X64_WINDOW);
		else if (DA9052_SCALE_32X == msg.data)
			sm_mon_interval = msecs_to_jiffies(DA9052_X32_WINDOW);
		else if (DA9052_SCALE_16X == msg.data)
			sm_mon_interval = msecs_to_jiffies(DA9052_X16_WINDOW);
		else if (DA9052_SCALE_8X == msg.data)
			sm_mon_interval = msecs_to_jiffies(DA9052_X8_WINDOW);
		else if (DA9052_SCALE_4X == msg.data)
			sm_mon_interval = msecs_to_jiffies(DA9052_X4_WINDOW);
		else if (DA9052_SCALE_2X == msg.data)
			sm_mon_interval = msecs_to_jiffies(DA9052_X2_WINDOW);
		else
			sm_mon_interval = msecs_to_jiffies(DA9052_X1_WINDOW);

	} else if (strobing_filter_state == DISABLE) {
		sm_strobe_filter_flag = DISABLE;
		sm_mon_interval = msecs_to_jiffies(DA9052_ADC_TWDMIN_TIME);
		if (DA9052_WDT_DISABLE == msg.data) {
			sm_str_req = DISABLE;
			del_timer(monitoring_timer);
			return 0;
		}
	} else {
		return STROBING_FILTER_ERROR;
	}
	mod_timer(monitoring_timer, jiffies + sm_mon_interval);

	return 0;
ssc_err:
	da9052_unlock(wdt->da9052);
	return -EIO;
}

int da9052_sm_set_strobing_mode(u8 strobing_mode_state)
{
	if (strobing_mode_state == DA9052_STROBE_AUTO)
		sm_strobe_mode_flag = DA9052_STROBE_AUTO;
	else if (strobing_mode_state == DA9052_STROBE_MANUAL)
		sm_strobe_mode_flag = DA9052_STROBE_MANUAL;
	else
		return STROBING_MODE_ERROR;

		return 0;
}

int da9052_sm_strobe_wdt(void)
{
	sm_str_req = ENABLE;
	return 0;
}

 s32 da9052_sm_set_wdt(struct da9052_wdt *wdt, u8 wdt_scaling)
{
	struct da9052_ssc_msg msg;
	int ret = 0;


	if (wdt_scaling > DA9052_SCALE_64X)
		return INVALID_SCALING_VALUE;

	msg.addr = DA9052_CONTROLD_REG;
	msg.data = 0;
	da9052_lock(wdt->da9052);
	ret = wdt->da9052->read(wdt->da9052, &msg);
	if (ret)
		goto ssc_err;
	da9052_unlock(wdt->da9052);

	if (!((DA9052_WDT_DISABLE == (msg.data & DA9052_CONTROLD_TWDSCALE)) &&
	    (DA9052_WDT_DISABLE == wdt_scaling))) {
		msg.data = (msg.data & ~(DA9052_CONTROLD_TWDSCALE));
		msg.addr = DA9052_CONTROLD_REG;


		da9052_lock(wdt->da9052);
		ret = wdt->da9052->write(wdt->da9052, &msg);
		if (ret)
			goto ssc_err;
		da9052_unlock(wdt->da9052);

		msleep(1);
		da9052_lock(wdt->da9052);
		ret = wdt->da9052->read(wdt->da9052, &msg);
		if (ret)
			goto ssc_err;
		da9052_unlock(wdt->da9052);


		msg.data |= wdt_scaling;

		da9052_lock(wdt->da9052);
		ret = wdt->da9052->write(wdt->da9052, &msg);
		if (ret)
			goto ssc_err;
		da9052_unlock(wdt->da9052);

		sm_str_req = DISABLE;
		if (DA9052_WDT_DISABLE == wdt_scaling) {
			del_timer(monitoring_timer);
			return 0;
		}
		if (sm_strobe_filter_flag == ENABLE) {
			if (DA9052_SCALE_64X == wdt_scaling) {
				sm_mon_interval =
					msecs_to_jiffies(DA9052_X64_WINDOW);
			} else if (DA9052_SCALE_32X == wdt_scaling) {
				sm_mon_interval =
					msecs_to_jiffies(DA9052_X32_WINDOW);
			} else if (DA9052_SCALE_16X == wdt_scaling) {
				sm_mon_interval =
					msecs_to_jiffies(DA9052_X16_WINDOW);
			} else if (DA9052_SCALE_8X == wdt_scaling) {
				sm_mon_interval =
					msecs_to_jiffies(DA9052_X8_WINDOW);
			} else if (DA9052_SCALE_4X == wdt_scaling) {
				sm_mon_interval =
					msecs_to_jiffies(DA9052_X4_WINDOW);
			} else if (DA9052_SCALE_2X == wdt_scaling) {
				sm_mon_interval =
					msecs_to_jiffies(DA9052_X2_WINDOW);
			} else {
				sm_mon_interval =
					msecs_to_jiffies(DA9052_X1_WINDOW);
			}
		} else {
			sm_mon_interval = msecs_to_jiffies(
						DA9052_ADC_TWDMIN_TIME);
		}
		mod_timer(monitoring_timer, jiffies + sm_mon_interval);
	}

	return 0;
ssc_err:
	da9052_unlock(wdt->da9052);
	return -EIO;
}

static int da9052_wdt_open(struct inode *inode, struct file *file)
{
	struct da9052_wdt *wdt = get_wdt_da9052();
	int ret;
	printk(KERN_INFO"IN WDT OPEN \n");

	if (!wdt) {
		printk(KERN_INFO"Returning no device\n");
		return -ENODEV;
	}
	printk(KERN_INFO"IN WDT OPEN 1\n");

	if (test_and_set_bit(0, &da9052_wdt_users))
		return -EBUSY;

	ret = da9052_sm_hw_init(wdt);
	if (ret != 0) {
		printk(KERN_ERR "Watchdog hw init failed\n");
		return ret;
	}

	return nonseekable_open(inode, file);
}

static int da9052_wdt_release(struct inode *inode, struct file *file)
{
	struct da9052_wdt *wdt = get_wdt_da9052();

	if (da9052_wdt_expect_close == 42)
		da9052_sm_hw_deinit(wdt);
	else
		da9052_sm_strobe_wdt();
	da9052_wdt_expect_close = 0;
	clear_bit(0, &da9052_wdt_users);
	return 0;
}

static ssize_t da9052_wdt_write(struct file *file,
				const char __user *data, size_t count,
				loff_t *ppos)
{
	size_t i;

	if (count) {
		if (!nowayout) {
			/* In case it was set long ago */
			da9052_wdt_expect_close = 0;
			for (i = 0; i != count; i++) {
				char c;
				if (get_user(c, data + i))
					return -EFAULT;
				if (c == 'V')
					da9052_wdt_expect_close = 42;
			}
		}
		da9052_sm_strobe_wdt();
	}
	return count;
}

static struct watchdog_info da9052_wdt_info = {
	.options	= WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING,
	.identity	= "DA9052_SM Watchdog",
};

static long da9052_wdt_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	struct da9052_wdt *wdt = get_wdt_da9052();
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	unsigned char new_value;

	switch (cmd) {

	case WDIOC_GETSUPPORT:
		return copy_to_user(argp, &da9052_wdt_info,
			sizeof(da9052_wdt_info)) ? -EFAULT : 0;
	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		return put_user(0, p);
	case WDIOC_SETOPTIONS:
		if (get_user(new_value, p))
			return -EFAULT;
		if (new_value & DA9052_STROBING_FILTER_ENABLE)
			da9052_sm_set_strobing_filter(wdt, ENABLE);
		if (new_value & DA9052_STROBING_FILTER_DISABLE)
			da9052_sm_set_strobing_filter(wdt, DISABLE);
		if (new_value & DA9052_SET_STROBING_MODE_MANUAL)
			da9052_sm_set_strobing_mode(DA9052_STROBE_MANUAL);
		if (new_value & DA9052_SET_STROBING_MODE_AUTO)
			da9052_sm_set_strobing_mode(DA9052_STROBE_AUTO);
		return 0;
	case WDIOC_KEEPALIVE:
		if (da9052_sm_strobe_wdt())
			return -EFAULT;
		else
			return 0;
	case WDIOC_SETTIMEOUT:
		if (get_user(new_value, p))
			return -EFAULT;
		da9052_sm_scale = new_value;
		if (da9052_sm_set_wdt(wdt, da9052_sm_scale))
			return -EFAULT;
	case WDIOC_GETTIMEOUT:
		return put_user(sm_mon_interval, p);
	default:
		return -ENOTTY;
	}
	return 0;
}

static const struct file_operations da9052_wdt_fops = {
	.owner          = THIS_MODULE,
	.llseek		= no_llseek,
	.unlocked_ioctl	= da9052_wdt_ioctl,
	.write		= da9052_wdt_write,
	.open           = da9052_wdt_open,
	.release        = da9052_wdt_release,
};

static struct miscdevice da9052_wdt_miscdev = {
	.minor		= 255,
	.name		= "da9052-wdt",
	.fops		= &da9052_wdt_fops,
};

static int __devinit da9052_sm_probe(struct platform_device *pdev)
{
	int ret;
	struct da9052_wdt *wdt;
	struct da9052_ssc_msg msg;

	wdt = kzalloc(sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	wdt->da9052 = dev_get_drvdata(pdev->dev.parent);
	platform_set_drvdata(pdev, wdt);

	msg.addr = DA9052_CONTROLD_REG;
	msg.data = 0;

	da9052_lock(wdt->da9052);
	ret = wdt->da9052->read(wdt->da9052, &msg);
	if (ret) {
		da9052_unlock(wdt->da9052);
		goto err_ssc_comm;
	}
	printk(KERN_INFO"DA9052 SM probe - 0 \n");

	msg.data = (msg.data & ~(DA9052_CONTROLD_TWDSCALE));
	ret = wdt->da9052->write(wdt->da9052, &msg);
	if (ret) {
		da9052_unlock(wdt->da9052);
		goto err_ssc_comm;
	}
	da9052_unlock(wdt->da9052);

	da9052_wdt_miscdev.parent = &pdev->dev;

	ret = misc_register(&da9052_wdt_miscdev);
	if (ret != 0) {
		platform_set_drvdata(pdev, NULL);
		kfree(wdt);
		return -EFAULT;
	}
	return 0;
err_ssc_comm:
	platform_set_drvdata(pdev, NULL);
	kfree(wdt);
	return -EIO;
}

static int __devexit da9052_sm_remove(struct platform_device *dev)
{
	misc_deregister(&da9052_wdt_miscdev);

	return 0;
}

static struct platform_driver da9052_sm_driver = {
	.probe		= da9052_sm_probe,
	.remove		= __devexit_p(da9052_sm_remove),
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init da9052_sm_init(void)
{
	return platform_driver_register(&da9052_sm_driver);
}
module_init(da9052_sm_init);

static void __exit da9052_sm_exit(void)
{
	platform_driver_unregister(&da9052_sm_driver);
}
module_exit(da9052_sm_exit);

MODULE_AUTHOR("David Dajun Chen <dchen@diasemi.com>")
MODULE_DESCRIPTION("DA9052 SM Device Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
