/*
 * Freescale STMP37XX/STMP378X Real Time Clock driver
 *
 * Copyright (c) 2007 Sigmatel, Inc.
 * Peter Hartley, <peter.hartley@sigmatel.com>
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
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/uaccess.h>

#include <mach/stmp3xxx.h>
#include <mach/hardware.h>
#include <mach/platform.h>
#include <mach/irqs.h>
#include <mach/regs-rtc.h>

struct stmp3xxx_rtc_data {
	struct rtc_device *rtc;
	unsigned irq_count;
};

/* Time read/write */
static int stmp3xxx_rtc_gettime(struct device *dev, struct rtc_time *rtc_tm)
{
	while (__raw_readl(REGS_RTC_BASE + HW_RTC_STAT) & BF(0x80, RTC_STAT_STALE_REGS))
		cpu_relax();

	rtc_time_to_tm(__raw_readl(REGS_RTC_BASE + HW_RTC_SECONDS), rtc_tm);
	return 0;
}

static int stmp3xxx_rtc_settime(struct device *dev, struct rtc_time *rtc_tm)
{
	unsigned long t;
	int rc = rtc_tm_to_time(rtc_tm, &t);

	if (rc == 0) {
		__raw_writel(t, REGS_RTC_BASE + HW_RTC_SECONDS);

		/* The datasheet doesn't say which way round the
		 * NEW_REGS/STALE_REGS bitfields go. In fact it's 0x1=P0,
		 * 0x2=P1, .., 0x20=P5, 0x40=ALARM, 0x80=SECONDS,
		 */
		while (__raw_readl(REGS_RTC_BASE + HW_RTC_STAT) & BF(0x80, RTC_STAT_NEW_REGS))
			cpu_relax();
	}
	return rc;
}

static irqreturn_t stmp3xxx_rtc_interrupt(int irq, void *dev_id)
{
	struct platform_device *pdev = to_platform_device(dev_id);
	struct stmp3xxx_rtc_data *data = platform_get_drvdata(pdev);
	u32 status;
	u32 events = 0;

	status = __raw_readl(REGS_RTC_BASE + HW_RTC_CTRL) &
			(BM_RTC_CTRL_ALARM_IRQ | BM_RTC_CTRL_ONEMSEC_IRQ);
	if (status & BM_RTC_CTRL_ALARM_IRQ) {
		__raw_writel(BM_RTC_CTRL_ALARM_IRQ,
			REGS_RTC_BASE + HW_RTC_CTRL_CLR);
		events |= RTC_AF | RTC_IRQF;
	}
	if (status & BM_RTC_CTRL_ONEMSEC_IRQ) {
		__raw_writel(BM_RTC_CTRL_ONEMSEC_IRQ,
			REGS_RTC_BASE + HW_RTC_CTRL_CLR);
		if (++data->irq_count % 1000 == 0) {
			events |= RTC_UF | RTC_IRQF;
			data->irq_count = 0;
		}
	}

	if (events)
		rtc_update_irq(data->rtc, 1, events);

	return IRQ_HANDLED;
}

static int stmp3xxx_rtc_open(struct device *dev)
{
	int r;

	r = request_irq(IRQ_RTC_ALARM, stmp3xxx_rtc_interrupt,
			IRQF_DISABLED, "RTC alarm", dev);
	if (r) {
		dev_err(dev, "Cannot claim IRQ%d\n", IRQ_RTC_ALARM);
		goto fail_1;
	}
	r = request_irq(IRQ_RTC_1MSEC, stmp3xxx_rtc_interrupt,
			IRQF_DISABLED, "RTC tick", dev);
	if (r) {
		dev_err(dev, "Cannot claim IRQ%d\n", IRQ_RTC_1MSEC);
		goto fail_2;
	}

	return 0;
fail_2:
	free_irq(IRQ_RTC_ALARM, dev);
fail_1:
	return r;
}

static void stmp3xxx_rtc_release(struct device *dev)
{
	__raw_writel(BM_RTC_CTRL_ALARM_IRQ_EN | BM_RTC_CTRL_ONEMSEC_IRQ_EN,
		REGS_RTC_BASE + HW_RTC_CTRL_CLR);
	free_irq(IRQ_RTC_ALARM, dev);
	free_irq(IRQ_RTC_1MSEC, dev);
}

static int stmp3xxx_rtc_ioctl(struct device *dev, unsigned int cmd,
			      unsigned long arg)
{
	struct stmp3xxx_rtc_data *data = dev_get_drvdata(dev);

	switch (cmd) {
	case RTC_AIE_OFF:
		__raw_writel(BM_RTC_PERSISTENT0_ALARM_EN |
				BM_RTC_PERSISTENT0_ALARM_WAKE_EN,
				REGS_RTC_BASE + HW_RTC_PERSISTENT0_CLR);
		__raw_writel(BM_RTC_CTRL_ALARM_IRQ_EN,
				REGS_RTC_BASE + HW_RTC_CTRL_CLR);
		break;
	case RTC_AIE_ON:
		__raw_writel(BM_RTC_PERSISTENT0_ALARM_EN |
				BM_RTC_PERSISTENT0_ALARM_WAKE_EN,
				REGS_RTC_BASE + HW_RTC_PERSISTENT0_SET);

		__raw_writel(BM_RTC_CTRL_ALARM_IRQ_EN,
				REGS_RTC_BASE + HW_RTC_CTRL_SET);
		break;
	case RTC_UIE_ON:
		data->irq_count = 0;
		__raw_writel(BM_RTC_CTRL_ONEMSEC_IRQ_EN,
			REGS_RTC_BASE + HW_RTC_CTRL_SET);
		break;
	case RTC_UIE_OFF:
		__raw_writel(BM_RTC_CTRL_ONEMSEC_IRQ_EN,
			REGS_RTC_BASE + HW_RTC_CTRL_CLR);
		break;
	default:
		return -ENOIOCTLCMD;
	}

	return 0;
}
static int stmp3xxx_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	u32 t = __raw_readl(REGS_RTC_BASE + HW_RTC_ALARM);

	rtc_time_to_tm(t, &alm->time);

	return 0;
}

static int stmp3xxx_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	unsigned long t;

	rtc_tm_to_time(&alm->time, &t);
	__raw_writel(t, REGS_RTC_BASE + HW_RTC_ALARM);
	return 0;
}

static struct rtc_class_ops stmp3xxx_rtc_ops = {
	.open		= stmp3xxx_rtc_open,
	.release	= stmp3xxx_rtc_release,
	.ioctl          = stmp3xxx_rtc_ioctl,
	.read_time	= stmp3xxx_rtc_gettime,
	.set_time	= stmp3xxx_rtc_settime,
	.read_alarm	= stmp3xxx_rtc_read_alarm,
	.set_alarm	= stmp3xxx_rtc_set_alarm,
};

static int stmp3xxx_rtc_remove(struct platform_device *dev)
{
	struct stmp3xxx_rtc_data *rtc_data = platform_get_drvdata(dev);

	if (rtc_data) {
		rtc_device_unregister(rtc_data->rtc);
		kfree(rtc_data);
	}

	return 0;
}

static int stmp3xxx_rtc_probe(struct platform_device *pdev)
{
	u32 hwversion = __raw_readl(REGS_RTC_BASE + HW_RTC_VERSION);
	u32 rtc_stat = __raw_readl(REGS_RTC_BASE + HW_RTC_STAT);
	struct stmp3xxx_rtc_data *rtc_data = kzalloc(sizeof *rtc_data,
						     GFP_KERNEL);

	if ((rtc_stat & BM_RTC_STAT_RTC_PRESENT) == 0)
		return -ENODEV;
	if (!rtc_data)
		return -ENOMEM;

	stmp3xxx_reset_block(REGS_RTC_BASE, 1);

	__raw_writel(BM_RTC_PERSISTENT0_ALARM_EN |
			BM_RTC_PERSISTENT0_ALARM_WAKE_EN |
			BM_RTC_PERSISTENT0_ALARM_WAKE,
		     REGS_RTC_BASE + HW_RTC_PERSISTENT0_CLR);
	__raw_writel(BM_RTC_PERSISTENT0_AUTO_RESTART,
		     REGS_RTC_BASE + HW_RTC_PERSISTENT0_SET);

	printk(KERN_INFO "STMP3xxx RTC driver v1.0 hardware v%u.%u.%u\n",
	       (hwversion >> 24),
	       (hwversion >> 16) & 0xFF,
	       hwversion & 0xFFFF);

	rtc_data->rtc = rtc_device_register(pdev->name, &pdev->dev,
				&stmp3xxx_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc_data->rtc)) {
		kfree(rtc_data);
		return PTR_ERR(rtc_data->rtc);
	}

	platform_set_drvdata(pdev, rtc_data);

	return 0;
}

#ifdef CONFIG_PM
static int stmp3xxx_rtc_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

static int stmp3xxx_rtc_resume(struct platform_device *dev)
{
	stmp3xxx_reset_block(REGS_RTC_BASE, 1);
	__raw_writel(BM_RTC_PERSISTENT0_ALARM_EN |
			BM_RTC_PERSISTENT0_ALARM_WAKE_EN |
			BM_RTC_PERSISTENT0_ALARM_WAKE,
		     REGS_RTC_BASE + HW_RTC_PERSISTENT0_CLR);
	return 0;
}
#else
#define stmp3xxx_rtc_suspend	NULL
#define stmp3xxx_rtc_resume	NULL
#endif

static struct platform_driver stmp3xxx_rtcdrv = {
	.probe		= stmp3xxx_rtc_probe,
	.remove		= stmp3xxx_rtc_remove,
	.suspend	= stmp3xxx_rtc_suspend,
	.resume		= stmp3xxx_rtc_resume,
	.driver		= {
		.name	= "stmp3xxx-rtc",
		.owner	= THIS_MODULE,
	},
};

static int __init stmp3xxx_rtc_init(void)
{
	return platform_driver_register(&stmp3xxx_rtcdrv);
}

static void __exit stmp3xxx_rtc_exit(void)
{
	platform_driver_unregister(&stmp3xxx_rtcdrv);
}

module_init(stmp3xxx_rtc_init);
module_exit(stmp3xxx_rtc_exit);

MODULE_DESCRIPTION("STMP3xxx RTC Driver");
MODULE_AUTHOR("dmitry pervushin <dimka@embeddedalley.com>");
MODULE_LICENSE("GPL");
