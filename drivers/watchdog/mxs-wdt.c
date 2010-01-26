/*
 * Watchdog driver for Freescale STMP37XX/STMP378X
 *
 * Author: Vitaly Wool <vital@embeddedalley.com>
 *
 * Copyright 2008-2010 Freescale Semiconductor, Inc.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <mach/regs-rtc.h>

#define DEFAULT_HEARTBEAT	19
#define MAX_HEARTBEAT		(0x10000000 >> 6)

/* missing bitmask in headers */
#define BV_RTC_PERSISTENT1_GENERAL__RTC_FORCE_UPDATER	0x80000000

#define WDT_IN_USE		0
#define WDT_OK_TO_CLOSE		1

#define WDOG_COUNTER_RATE	1000 /* 1 kHz clock */

static unsigned long wdt_status;
static int heartbeat = DEFAULT_HEARTBEAT;
static unsigned long boot_status;
static unsigned long wdt_base;
static DEFINE_SPINLOCK(mxs_wdt_io_lock);

static void wdt_enable(u32 value)
{
	spin_lock(&mxs_wdt_io_lock);
	__raw_writel(value, wdt_base + HW_RTC_WATCHDOG);
	__raw_writel(BV_RTC_PERSISTENT1_GENERAL__RTC_FORCE_UPDATER,
		     wdt_base + HW_RTC_PERSISTENT1_SET);
	__raw_writel(BM_RTC_CTRL_WATCHDOGEN, wdt_base + HW_RTC_CTRL_SET);
	spin_unlock(&mxs_wdt_io_lock);
}

static void wdt_disable(void)
{
	spin_lock(&mxs_wdt_io_lock);
	__raw_writel(BV_RTC_PERSISTENT1_GENERAL__RTC_FORCE_UPDATER,
		     wdt_base + HW_RTC_PERSISTENT1_CLR);
	__raw_writel(BM_RTC_CTRL_WATCHDOGEN, wdt_base + HW_RTC_CTRL_CLR);
	spin_unlock(&mxs_wdt_io_lock);
}

static void wdt_ping(void)
{
	wdt_enable(heartbeat * WDOG_COUNTER_RATE);
}

static int mxs_wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(WDT_IN_USE, &wdt_status))
		return -EBUSY;

	clear_bit(WDT_OK_TO_CLOSE, &wdt_status);
	wdt_ping();

	return nonseekable_open(inode, file);
}

static ssize_t mxs_wdt_write(struct file *file, const char __user *data,
	size_t len, loff_t *ppos)
{
	if (len) {
		if (WATCHDOG_NOWAYOUT == 0) {
			size_t i;

			clear_bit(WDT_OK_TO_CLOSE, &wdt_status);

			for (i = 0; i != len; i++) {
				char c;

				if (get_user(c, data + i))
					return -EFAULT;
				if (c == 'V')
					set_bit(WDT_OK_TO_CLOSE, &wdt_status);
			}
		}
		wdt_ping();
	}

	return len;
}

static struct watchdog_info ident = {
	.options	= WDIOF_CARDRESET |
			  WDIOF_MAGICCLOSE |
			  WDIOF_SETTIMEOUT |
			  WDIOF_KEEPALIVEPING,
	.identity	= "MXS Watchdog",
};

static long mxs_wdt_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	int new_heartbeat, opts;
	int ret = -ENOTTY;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		ret = copy_to_user(argp, &ident, sizeof(ident)) ? -EFAULT : 0;
		break;

	case WDIOC_GETSTATUS:
		ret = put_user(0, p);
		break;

	case WDIOC_GETBOOTSTATUS:
		ret = put_user(boot_status, p);
		break;

	case WDIOC_SETOPTIONS:
		if (get_user(opts, p)) {
			ret = -EFAULT;
			break;
		}
		if (opts & WDIOS_DISABLECARD)
			wdt_disable();
		else if (opts & WDIOS_ENABLECARD)
			wdt_ping();
		else {
			pr_debug("%s: unknown option 0x%x\n", __func__, opts);
			ret = -EINVAL;
			break;
		}
		ret = 0;
		break;

	case WDIOC_KEEPALIVE:
		wdt_ping();
		ret = 0;
		break;

	case WDIOC_SETTIMEOUT:
		if (get_user(new_heartbeat, p)) {
			ret = -EFAULT;
			break;
		}
		if (new_heartbeat <= 0 || new_heartbeat > MAX_HEARTBEAT) {
			ret = -EINVAL;
			break;
		}

		heartbeat = new_heartbeat;
		wdt_ping();
		/* Fall through */

	case WDIOC_GETTIMEOUT:
		ret = put_user(heartbeat, p);
		break;
	}
	return ret;
}

static int mxs_wdt_release(struct inode *inode, struct file *file)
{
	int ret = 0;

	if (WATCHDOG_NOWAYOUT == 0) {
		if (!test_bit(WDT_OK_TO_CLOSE, &wdt_status)) {
			wdt_ping();
			pr_debug("%s: Device closed unexpectdly\n", __func__);
			ret = -EINVAL;
		} else {
			wdt_disable();
			clear_bit(WDT_OK_TO_CLOSE, &wdt_status);
		}
	}
	clear_bit(WDT_IN_USE, &wdt_status);

	return ret;
}

static const struct file_operations mxs_wdt_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.write = mxs_wdt_write,
	.unlocked_ioctl = mxs_wdt_ioctl,
	.open = mxs_wdt_open,
	.release = mxs_wdt_release,
};

static struct miscdevice mxs_wdt_miscdev = {
	.minor = WATCHDOG_MINOR,
	.name = "watchdog",
	.fops = &mxs_wdt_fops,
};

static int __devinit mxs_wdt_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;

	if (heartbeat < 1 || heartbeat > MAX_HEARTBEAT)
		heartbeat = DEFAULT_HEARTBEAT;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL)
		return -ENODEV;
	wdt_base = (unsigned long)IO_ADDRESS(res->start);

	boot_status = __raw_readl(wdt_base + HW_RTC_PERSISTENT1) &
			BV_RTC_PERSISTENT1_GENERAL__RTC_FORCE_UPDATER;
	boot_status = !!boot_status;
	__raw_writel(BV_RTC_PERSISTENT1_GENERAL__RTC_FORCE_UPDATER,
		     wdt_base + HW_RTC_PERSISTENT1_CLR);

	wdt_disable();		/* disable for now */

	ret = misc_register(&mxs_wdt_miscdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot register misc device\n");
		return ret;
	}

	printk(KERN_INFO "mxs watchdog: initialized, heartbeat %d sec\n",
		heartbeat);

	return ret;
}

static int __devexit mxs_wdt_remove(struct platform_device *pdev)
{
	misc_deregister(&mxs_wdt_miscdev);
	return 0;
}

#ifdef CONFIG_PM
static int wdt_suspended;
static u32 wdt_saved_time;

static int mxs_wdt_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	if (__raw_readl(wdt_base + HW_RTC_CTRL) & BM_RTC_CTRL_WATCHDOGEN) {
		wdt_saved_time = __raw_readl(wdt_base + HW_RTC_WATCHDOG);
		wdt_disable();
		wdt_suspended = 1;
	}
	return 0;
}

static int mxs_wdt_resume(struct platform_device *pdev)
{
	if (wdt_suspended) {
		wdt_suspended = 0;
		wdt_enable(wdt_saved_time);
	}
	return 0;
}
#else
#define mxs_wdt_suspend	NULL
#define mxs_wdt_resume	NULL
#endif

static struct platform_driver mxs_wdt_driver = {
	.driver = {
		.name = "mxs-wdt",
	},
	.probe = mxs_wdt_probe,
	.remove = __devexit_p(mxs_wdt_remove),
	.suspend = mxs_wdt_suspend,
	.resume = mxs_wdt_resume,
};

static int __init mxs_wdt_init(void)
{
	return platform_driver_register(&mxs_wdt_driver);
}

static void __exit mxs_wdt_exit(void)
{
	return platform_driver_unregister(&mxs_wdt_driver);
}

module_init(mxs_wdt_init);
module_exit(mxs_wdt_exit);

MODULE_DESCRIPTION("MXS Watchdog Driver");
MODULE_LICENSE("GPL");

module_param(heartbeat, int, 0);
MODULE_PARM_DESC(heartbeat,
		 "Watchdog heartbeat period in seconds from 1 to "
		 __MODULE_STRING(MAX_HEARTBEAT) ", default "
		 __MODULE_STRING(DEFAULT_HEARTBEAT));

MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
