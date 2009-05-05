/*
 * Watchdog driver for Freescale STMP37XX/STMP378X
 *
 * Author: Vitaly Wool <vital@embeddedalley.com>
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
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/spinlock.h>
#include <linux/init.h>
#include <linux/uaccess.h>

#include <mach/regs-rtc.h>

#define DEFAULT_HEARTBEAT 19
#define MAX_HEARTBEAT     (0x10000000 >> 6) /* actually 0x100000000 >> 10 */

/* missing bitmask in headers */
#define BV_RTC_PERSISTENT1_GENERAL__RTC_FORCE_UPDATER     0x80000000

#define WDT_IN_USE        0
#define WDT_OK_TO_CLOSE   1
#define WDT_REGION_INITED 2
#define WDT_DEVICE_INITED 3

#define WDOG_COUNTER_RATE	1000 /* 1 kHz clock */

DEFINE_SPINLOCK(io_lock);
static unsigned long wdt_status;
static const int  nowayout = WATCHDOG_NOWAYOUT;
static int heartbeat = DEFAULT_HEARTBEAT;
static unsigned long boot_status;


static void wdt_enable(u32 value)
{
	spin_lock(&io_lock);
	HW_RTC_WATCHDOG_WR(value);
	HW_RTC_CTRL_SET(BM_RTC_CTRL_WATCHDOGEN);
	HW_RTC_PERSISTENT1_SET(BV_RTC_PERSISTENT1_GENERAL__RTC_FORCE_UPDATER);
	spin_unlock(&io_lock);
}

static void wdt_disable(void)
{
	spin_lock(&io_lock);
	HW_RTC_PERSISTENT1_CLR(BV_RTC_PERSISTENT1_GENERAL__RTC_FORCE_UPDATER);
	HW_RTC_CTRL_CLR(BM_RTC_CTRL_WATCHDOGEN);
	spin_unlock(&io_lock);
}

static int stmp3xxx_wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(WDT_IN_USE, &wdt_status))
		return -EBUSY;

	clear_bit(WDT_OK_TO_CLOSE, &wdt_status);

	wdt_enable(heartbeat * WDOG_COUNTER_RATE);

	return nonseekable_open(inode, file);
}

static ssize_t
stmp3xxx_wdt_write(struct file *file, const char *data, size_t len,
		  loff_t *ppos)
{
	if (len) {
		if (!nowayout) {
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
		wdt_enable(heartbeat * WDOG_COUNTER_RATE);
	}

	return len;
}

static struct watchdog_info ident = {
	.options	= WDIOF_CARDRESET |
			  WDIOF_MAGICCLOSE |
			  WDIOF_SETTIMEOUT |
			  WDIOF_KEEPALIVEPING,
	.identity = "STMP37XX Watchdog",
};

static int
stmp3xxx_wdt_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
			unsigned long arg)
{
	int new_heartbeat, opts;
	int ret = -ENOTTY;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		ret = copy_to_user((struct watchdog_info __user *)arg, &ident,
				   sizeof(ident)) ? -EFAULT : 0;
		break;

	case WDIOC_GETSTATUS:
		ret = put_user(0, (int __user *)arg);
		break;

	case WDIOC_GETBOOTSTATUS:
		ret = put_user(boot_status, (int __user *)arg);
		break;

	case WDIOC_SETOPTIONS:
		get_user(opts, (int __user *)arg);
		if (opts & WDIOS_DISABLECARD)
			wdt_disable();
		else if (opts & WDIOS_ENABLECARD)
			wdt_enable(heartbeat * WDOG_COUNTER_RATE);
		else {
			pr_debug("%s: unknown option 0x%x\n", __func__, opts);
			ret = -EINVAL;
		}
		break;

	case WDIOC_SETTIMEOUT:
		get_user(new_heartbeat, (int __user *)arg);
		if (new_heartbeat <= 0 || new_heartbeat > MAX_HEARTBEAT) {
			ret = -EINVAL;
			break;
		}

		heartbeat = new_heartbeat;
		wdt_enable(heartbeat * WDOG_COUNTER_RATE);
		/* Fall through */

	case WDIOC_GETTIMEOUT:
		ret = put_user(heartbeat, (int __user *)arg);
		break;

	case WDIOC_KEEPALIVE:
		wdt_enable(heartbeat * WDOG_COUNTER_RATE);
		ret = 0;
		break;

	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int stmp3xxx_wdt_release(struct inode *inode, struct file *file)
{
	int ret = 0;

	if (!nowayout) {
		if (!test_bit(WDT_OK_TO_CLOSE, &wdt_status)) {
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

static const struct file_operations stmp3xxx_wdt_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.write = stmp3xxx_wdt_write,
	.ioctl = stmp3xxx_wdt_ioctl,
	.open = stmp3xxx_wdt_open,
	.release = stmp3xxx_wdt_release,
};

static struct miscdevice stmp3xxx_wdt_miscdev = {
	.minor = WATCHDOG_MINOR,
	.name = "watchdog",
	.fops = &stmp3xxx_wdt_fops,
};

static int stmp3xxx_wdt_probe(struct platform_device *pdev)
{
	int ret = 0;

	if (heartbeat < 1 || heartbeat > MAX_HEARTBEAT)
		heartbeat = DEFAULT_HEARTBEAT;

	boot_status = HW_RTC_PERSISTENT1_RD() &
			BV_RTC_PERSISTENT1_GENERAL__RTC_FORCE_UPDATER;
	boot_status = !!boot_status;
	HW_RTC_PERSISTENT1_CLR(BV_RTC_PERSISTENT1_GENERAL__RTC_FORCE_UPDATER);

	dev_dbg(&pdev->dev, "STMP37XX Watchdog Timer: heartbeat %d sec\n",
		heartbeat);

	ret = misc_register(&stmp3xxx_wdt_miscdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot register misc device\n");
	} else {
		wdt_disable();		/* disable for now */
		set_bit(WDT_DEVICE_INITED, &wdt_status);
	}

	return ret;
}

static int stmp3xxx_wdt_remove(struct platform_device *pdev)
{
	misc_deregister(&stmp3xxx_wdt_miscdev);
	return 0;
}

#ifdef CONFIG_PM
static int wdt_suspended;
static u32 wdt_saved_time;

static int stmp3xxx_wdt_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	if (HW_RTC_CTRL_RD() & BM_RTC_CTRL_WATCHDOGEN) {
		wdt_suspended = 1;
		wdt_saved_time = HW_RTC_WATCHDOG_RD();
		wdt_disable();
	}
	return 0;
}

static int stmp3xxx_wdt_resume(struct platform_device *pdev)
{
	if (wdt_suspended) {
		wdt_enable(wdt_saved_time);
		wdt_suspended = 0;
	}
	return 0;
}
#else
#define stmp3xxx_wdt_suspend	NULL
#define stmp3xxx_wdt_resume	NULL
#endif

static struct platform_driver platform_wdt_driver = {
	.driver = {
		.name = "stmp3xxx_wdt",
	},
	.probe = stmp3xxx_wdt_probe,
	.remove = __devexit_p(stmp3xxx_wdt_remove),
	.suspend = stmp3xxx_wdt_suspend,
	.resume = stmp3xxx_wdt_resume,
};

static int __init stmp3xxx_wdt_init(void)
{
	return platform_driver_register(&platform_wdt_driver);
}

static void __exit stmp3xxx_wdt_exit(void)
{
	return platform_driver_unregister(&platform_wdt_driver);
}

module_init(stmp3xxx_wdt_init);
module_exit(stmp3xxx_wdt_exit);

MODULE_DESCRIPTION("STMP37XX Watchdog Driver");
MODULE_LICENSE("GPL");

module_param(heartbeat, int, 0);
MODULE_PARM_DESC(heartbeat,
		 "Watchdog heartbeat period in seconds from 1 to "
		 __MODULE_STRING(MAX_HEARTBEAT) ", default "
		 __MODULE_STRING(DEFAULT_HEARTBEAT));

MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
