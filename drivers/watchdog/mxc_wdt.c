/*
 * linux/drivers/char/watchdog/mxc_wdt.c
 *
 * Watchdog driver for FSL MXC. It is based on omap1610_wdt.c
 *
 * Copyright (C) 2004-2011 Freescale Semiconductor, Inc.
 * 2005 (c) MontaVista Software, Inc.

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 *
 * 20051207: <AKuster@mvista.com>
 *	     	Full rewrite based on
 *		linux-2.6.15-rc5/drivers/char/watchdog/omap_wdt.c
 *	     	Add platform resource support
 *
 */

/*!
 * @defgroup WDOG Watchdog Timer (WDOG) Driver
 */
/*!
 * @file mxc_wdt.c
 *
 * @brief Watchdog timer driver
 *
 * @ingroup WDOG
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/uaccess.h>

#include "mxc_wdt.h"

#define DVR_VER "2.0"

#define WDOG_SEC_TO_COUNT(s)  ((s * 2) << 8)
#define WDOG_COUNT_TO_SEC(c)  ((c >> 8) / 2)

static void __iomem  *wdt_base_reg;
static int mxc_wdt_users;
static struct clk *mxc_wdt_clk;

static unsigned timer_margin = TIMER_MARGIN_DEFAULT;
module_param(timer_margin, uint, 0);
MODULE_PARM_DESC(timer_margin, "initial watchdog timeout (in seconds)");

static unsigned dev_num;

static void mxc_wdt_ping(void *base)
{
	/* issue the service sequence instructions */
	__raw_writew(WDT_MAGIC_1, base + MXC_WDT_WSR);
	__raw_writew(WDT_MAGIC_2, base + MXC_WDT_WSR);
}

static void mxc_wdt_config(void *base)
{
	u16 val;

	val = __raw_readw(base + MXC_WDT_WCR);
	val |= 0xFF00 | WCR_WOE_BIT | WCR_WDA_BIT | WCR_SRS_BIT;
	/* enable suspend WDT */
	val |= WCR_WDZST_BIT | WCR_WDBG_BIT;
	/* generate reset if wdog times out */
	val &= ~WCR_WRE_BIT;

	__raw_writew(val, base + MXC_WDT_WCR);
}

static void mxc_wdt_enable(void *base)
{
	u16 val;

	val = __raw_readw(base + MXC_WDT_WCR);
	val |= WCR_WDE_BIT;
	__raw_writew(val, base + MXC_WDT_WCR);
}

static void mxc_wdt_disable(void *base)
{
	/* disable not supported by this chip */
}

static void mxc_wdt_adjust_timeout(unsigned new_timeout)
{
	if (new_timeout < TIMER_MARGIN_MIN)
		new_timeout = TIMER_MARGIN_DEFAULT;
	if (new_timeout > TIMER_MARGIN_MAX)
		new_timeout = TIMER_MARGIN_MAX;
	timer_margin = new_timeout;
}

static u16 mxc_wdt_get_timeout(void *base)
{
	u16 val;

	val = __raw_readw(base + MXC_WDT_WCR);
	return WDOG_COUNT_TO_SEC(val);
}

static u16 mxc_wdt_get_bootreason(void *base)
{
	u16 val;

	val = __raw_readw(base + MXC_WDT_WRSR);
	return val;
}

static void mxc_wdt_set_timeout(void *base)
{
	u16 val;
	val = __raw_readw(base + MXC_WDT_WCR);
	val = (val & 0x00FF) | WDOG_SEC_TO_COUNT(timer_margin);
	__raw_writew(val, base + MXC_WDT_WCR);
	val = __raw_readw(base + MXC_WDT_WCR);
	timer_margin = WDOG_COUNT_TO_SEC(val);
}

/*
 *	Allow only one task to hold it open
 */

static int mxc_wdt_open(struct inode *inode, struct file *file)
{

	if (test_and_set_bit(1, (unsigned long *)&mxc_wdt_users))
		return -EBUSY;

	mxc_wdt_config(wdt_base_reg);
	mxc_wdt_set_timeout(wdt_base_reg);
	mxc_wdt_enable(wdt_base_reg);
	mxc_wdt_ping(wdt_base_reg);

	return 0;
}

static int mxc_wdt_release(struct inode *inode, struct file *file)
{
	/*
	 *      Shut off the timer unless NOWAYOUT is defined.
	 */
#ifndef CONFIG_WATCHDOG_NOWAYOUT
	mxc_wdt_disable(wdt_base_reg);

#else
	printk(KERN_CRIT "mxc_wdt: Unexpected close, not stopping!\n");
#endif
	mxc_wdt_users = 0;
	return 0;
}

static ssize_t
mxc_wdt_write(struct file *file, const char __user *data,
	      size_t len, loff_t *ppos)
{
	/* Refresh LOAD_TIME. */
	if (len)
		mxc_wdt_ping(wdt_base_reg);
	return len;
}

static int
mxc_wdt_ioctl(struct inode *inode, struct file *file,
	      unsigned int cmd, unsigned long arg)
{
	int new_margin;
	int bootr;

	static struct watchdog_info ident = {
		.identity = "MXC Watchdog",
		.options = WDIOF_SETTIMEOUT,
		.firmware_version = 0,
	};

	switch (cmd) {
	default:
		return -ENOIOCTLCMD;
	case WDIOC_GETSUPPORT:
		return copy_to_user((struct watchdog_info __user *)arg, &ident,
				    sizeof(ident));
	case WDIOC_GETSTATUS:
		return put_user(0, (int __user *)arg);
	case WDIOC_GETBOOTSTATUS:
		bootr = mxc_wdt_get_bootreason(wdt_base_reg);
		return put_user(bootr, (int __user *)arg);
	case WDIOC_KEEPALIVE:
		mxc_wdt_ping(wdt_base_reg);
		return 0;
	case WDIOC_SETTIMEOUT:
		if (get_user(new_margin, (int __user *)arg))
			return -EFAULT;

		mxc_wdt_adjust_timeout(new_margin);
		mxc_wdt_disable(wdt_base_reg);
		mxc_wdt_set_timeout(wdt_base_reg);
		mxc_wdt_enable(wdt_base_reg);
		mxc_wdt_ping(wdt_base_reg);
		return 0;

	case WDIOC_GETTIMEOUT:
		mxc_wdt_ping(wdt_base_reg);
		new_margin = mxc_wdt_get_timeout(wdt_base_reg);
		return put_user(new_margin, (int __user *)arg);
	}
}

static struct file_operations mxc_wdt_fops = {
	.owner = THIS_MODULE,
	.write = mxc_wdt_write,
	.ioctl = mxc_wdt_ioctl,
	.open = mxc_wdt_open,
	.release = mxc_wdt_release,
};

static struct miscdevice mxc_wdt_miscdev = {
	.minor = WATCHDOG_MINOR,
	.name = "watchdog",
	.fops = &mxc_wdt_fops
};

static int __devinit mxc_wdt_probe(struct platform_device *pdev)
{
	struct resource *res, *mem;
	int ret;

	/* reserve static register mappings */
	res = platform_get_resource(pdev, IORESOURCE_MEM, dev_num);
	if (!res)
		return -ENOENT;

	mem = request_mem_region(res->start, res->end - res->start + 1,
				 pdev->name);
	if (mem == NULL)
		return -EBUSY;

	platform_set_drvdata(pdev, mem);

	wdt_base_reg = ioremap(res->start, res->end - res->start + 1);
	mxc_wdt_disable(wdt_base_reg);
	mxc_wdt_adjust_timeout(timer_margin);

	mxc_wdt_users = 0;

	mxc_wdt_miscdev.this_device = &pdev->dev;

	mxc_wdt_clk = clk_get(NULL, "wdog_clk");
	clk_enable(mxc_wdt_clk);

	ret = misc_register(&mxc_wdt_miscdev);
	if (ret)
		goto fail;

	pr_info("MXC Watchdog # %d Timer: initial timeout %d sec\n", dev_num,
		timer_margin);

	return 0;

      fail:
	iounmap(wdt_base_reg);
	release_resource(mem);
	pr_info("MXC Watchdog Probe failed\n");
	return ret;
}

static void mxc_wdt_shutdown(struct platform_device *pdev)
{
	mxc_wdt_disable(wdt_base_reg);
	pr_info("MXC Watchdog # %d shutdown\n", dev_num);
}

static int __exit mxc_wdt_remove(struct platform_device *pdev)
{
	struct resource *mem = platform_get_drvdata(pdev);
	misc_deregister(&mxc_wdt_miscdev);
	iounmap(wdt_base_reg);
	release_resource(mem);
	pr_info("MXC Watchdog # %d removed\n", dev_num);
	return 0;
}

#ifdef	CONFIG_PM

/* REVISIT ... not clear this is the best way to handle system suspend; and
 * it's very inappropriate for selective device suspend (e.g. suspending this
 * through sysfs rather than by stopping the watchdog daemon).  Also, this
 * may not play well enough with NOWAYOUT...
 */

static int mxc_wdt_suspend(struct platform_device *pdev, pm_message_t state)
{
	if (mxc_wdt_users) {
		mxc_wdt_disable(wdt_base_reg);
	}
	return 0;
}

static int mxc_wdt_resume(struct platform_device *pdev)
{
	if (mxc_wdt_users) {
		mxc_wdt_enable(wdt_base_reg);
		mxc_wdt_ping(wdt_base_reg);
	}
	return 0;
}

#else
#define	mxc_wdt_suspend	NULL
#define	mxc_wdt_resume		NULL
#endif

static struct platform_driver mxc_wdt_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "mxc_wdt",
		   },
	.probe = mxc_wdt_probe,
	.shutdown = mxc_wdt_shutdown,
	.remove = __exit_p(mxc_wdt_remove),
	.suspend = mxc_wdt_suspend,
	.resume = mxc_wdt_resume,
};

static int __init mxc_wdt_init(void)
{
	pr_info("MXC WatchDog Driver %s\n", DVR_VER);

	if ((timer_margin < TIMER_MARGIN_MIN) ||
	    (timer_margin > TIMER_MARGIN_MAX)) {
		pr_info("MXC watchdog error. wrong timer_margin %d\n",
			timer_margin);
		pr_info("    Range: %d to %d seconds\n", TIMER_MARGIN_MIN,
			TIMER_MARGIN_MAX);
		return -EINVAL;
	}

	return platform_driver_register(&mxc_wdt_driver);
}

static void __exit mxc_wdt_exit(void)
{
	platform_driver_unregister(&mxc_wdt_driver);
	pr_info("MXC WatchDog Driver removed\n");
}

module_init(mxc_wdt_init);
module_exit(mxc_wdt_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
