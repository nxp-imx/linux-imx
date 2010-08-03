/*
 * Freescale STMP37XX/STMP378X Real Time Clock driver
 *
 * Copyright (c) 2007 Sigmatel, Inc.
 * Peter Hartley, <peter.hartley@sigmatel.com>
 *
 * Copyright 2008-2010 Freescale Semiconductor, Inc.
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
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/uaccess.h>

#include <mach/hardware.h>
#include <mach/regs-rtc.h>

struct mxs_rtc_data {
	struct rtc_device *rtc;
	unsigned int base;
	int irq_alarm;
	int irq_sample;
	unsigned irq_count;
};

/* Time read/write */
static int mxs_rtc_gettime(struct device *pdev, struct rtc_time *rtc_tm)
{
	struct mxs_rtc_data *rtc_data = dev_get_drvdata(pdev);

	while (__raw_readl(rtc_data->base + HW_RTC_STAT) &
			   BF_RTC_STAT_STALE_REGS(0x80))
		cpu_relax();

	rtc_time_to_tm(__raw_readl(rtc_data->base + HW_RTC_SECONDS), rtc_tm);
	return 0;
}

static int mxs_rtc_settime(struct device *pdev, struct rtc_time *rtc_tm)
{
	unsigned long t;
	int rc = rtc_tm_to_time(rtc_tm, &t);
	struct mxs_rtc_data *rtc_data = dev_get_drvdata(pdev);

	if (rc == 0) {
		__raw_writel(t, rtc_data->base + HW_RTC_SECONDS);

		/* The datasheet doesn't say which way round the
		 * NEW_REGS/STALE_REGS bitfields go. In fact it's 0x1=P0,
		 * 0x2=P1, .., 0x20=P5, 0x40=ALARM, 0x80=SECONDS,
		 */
		while (__raw_readl(rtc_data->base + HW_RTC_STAT) &
				   BF_RTC_STAT_NEW_REGS(0x80))
			cpu_relax();
	}
	return rc;
}

static irqreturn_t mxs_rtc_interrupt(int irq, void *dev_id)
{
	struct mxs_rtc_data *rtc_data = dev_get_drvdata(dev_id);
	u32 status;
	u32 events = 0;

	status = __raw_readl(rtc_data->base + HW_RTC_CTRL) &
			(BM_RTC_CTRL_ALARM_IRQ | BM_RTC_CTRL_ONEMSEC_IRQ);
	if (status & BM_RTC_CTRL_ALARM_IRQ) {
		__raw_writel(BM_RTC_CTRL_ALARM_IRQ,
			rtc_data->base + HW_RTC_CTRL_CLR);
		events |= RTC_AF | RTC_IRQF;
	}
	if (status & BM_RTC_CTRL_ONEMSEC_IRQ) {
		__raw_writel(BM_RTC_CTRL_ONEMSEC_IRQ,
			rtc_data->base + HW_RTC_CTRL_CLR);
		if (++rtc_data->irq_count % 1000 == 0) {
			events |= RTC_UF | RTC_IRQF;
			rtc_data->irq_count = 0;
		}
	}

	if (events)
		rtc_update_irq(rtc_data->rtc, 1, events);

	return IRQ_HANDLED;
}

static int mxs_rtc_open(struct device *pdev)
{
	int r;
	struct mxs_rtc_data *rtc_data = dev_get_drvdata(pdev);

	r = request_irq(rtc_data->irq_alarm, mxs_rtc_interrupt,
			IRQF_DISABLED, "RTC alarm", pdev);
	if (r) {
		dev_err(pdev, "Cannot claim IRQ%d\n", rtc_data->irq_alarm);
		goto fail_1;
	}
	r = request_irq(rtc_data->irq_sample, mxs_rtc_interrupt,
			IRQF_DISABLED, "RTC tick", pdev);
	if (r) {
		dev_err(pdev, "Cannot claim IRQ%d\n", rtc_data->irq_sample);
		goto fail_2;
	}

	return 0;
fail_2:
	free_irq(rtc_data->irq_alarm, pdev);
fail_1:
	return r;
}

static void mxs_rtc_release(struct device *pdev)
{
	struct mxs_rtc_data *rtc_data = dev_get_drvdata(pdev);

	__raw_writel(BM_RTC_CTRL_ALARM_IRQ_EN | BM_RTC_CTRL_ONEMSEC_IRQ_EN,
		rtc_data->base + HW_RTC_CTRL_CLR);
	free_irq(rtc_data->irq_alarm, pdev);
	free_irq(rtc_data->irq_sample, pdev);
}

static int mxs_rtc_ioctl(struct device *pdev, unsigned int cmd,
			      unsigned long arg)
{
	struct mxs_rtc_data *rtc_data = dev_get_drvdata(pdev);

	switch (cmd) {
	case RTC_AIE_OFF:
		__raw_writel(BM_RTC_PERSISTENT0_ALARM_EN |
				BM_RTC_PERSISTENT0_ALARM_WAKE_EN,
				rtc_data->base + HW_RTC_PERSISTENT0_CLR);
		__raw_writel(BM_RTC_CTRL_ALARM_IRQ_EN,
				rtc_data->base + HW_RTC_CTRL_CLR);
		break;
	case RTC_AIE_ON:
		__raw_writel(BM_RTC_PERSISTENT0_ALARM_EN |
				BM_RTC_PERSISTENT0_ALARM_WAKE_EN,
				rtc_data->base + HW_RTC_PERSISTENT0_SET);

		__raw_writel(BM_RTC_CTRL_ALARM_IRQ_EN,
				rtc_data->base + HW_RTC_CTRL_SET);
		break;
	case RTC_UIE_ON:
		rtc_data->irq_count = 0;
		__raw_writel(BM_RTC_CTRL_ONEMSEC_IRQ_EN,
			rtc_data->base + HW_RTC_CTRL_SET);
		break;
	case RTC_UIE_OFF:
		__raw_writel(BM_RTC_CTRL_ONEMSEC_IRQ_EN,
			rtc_data->base + HW_RTC_CTRL_CLR);
		break;
	default:
		return -ENOIOCTLCMD;
	}

	return 0;
}
static int mxs_rtc_read_alarm(struct device *pdev, struct rtc_wkalrm *alm)
{
	u32 t;
	struct mxs_rtc_data *rtc_data = dev_get_drvdata(pdev);

	t = __raw_readl(rtc_data->base + HW_RTC_ALARM);
	rtc_time_to_tm(t, &alm->time);
	return 0;
}

static int mxs_rtc_set_alarm(struct device *pdev, struct rtc_wkalrm *alm)
{
	unsigned long t;
	struct mxs_rtc_data *rtc_data = dev_get_drvdata(pdev);

	rtc_tm_to_time(&alm->time, &t);
	__raw_writel(t, rtc_data->base + HW_RTC_ALARM);
	return 0;
}

static struct rtc_class_ops mxs_rtc_ops = {
	.open		= mxs_rtc_open,
	.release	= mxs_rtc_release,
	.ioctl          = mxs_rtc_ioctl,
	.read_time	= mxs_rtc_gettime,
	.set_time	= mxs_rtc_settime,
	.read_alarm	= mxs_rtc_read_alarm,
	.set_alarm	= mxs_rtc_set_alarm,
};

static int mxs_rtc_probe(struct platform_device *pdev)
{
	u32 hwversion;
	u32 rtc_stat;
	struct resource *res;
	struct mxs_rtc_data *rtc_data;

	rtc_data = kzalloc(sizeof(*rtc_data), GFP_KERNEL);

	if (!rtc_data)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		kfree(rtc_data);
		return -ENODEV;
	}
	rtc_data->base = (unsigned int)IO_ADDRESS(res->start);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		kfree(rtc_data);
		return -ENODEV;
	}
	rtc_data->irq_alarm = res->start;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
	if (res == NULL) {
		kfree(rtc_data);
		return -ENODEV;
	}
	rtc_data->irq_sample = res->start;

	__raw_writel(BM_RTC_PERSISTENT0_ALARM_EN |
			BM_RTC_PERSISTENT0_ALARM_WAKE_EN |
			BM_RTC_PERSISTENT0_ALARM_WAKE,
		     rtc_data->base + HW_RTC_PERSISTENT0_CLR);

	hwversion = __raw_readl(rtc_data->base + HW_RTC_VERSION);
	rtc_stat = __raw_readl(rtc_data->base + HW_RTC_STAT);
	printk(KERN_INFO "MXS RTC driver v1.0 hardware v%u.%u.%u\n",
	       (hwversion >> 24),
	       (hwversion >> 16) & 0xFF,
	       hwversion & 0xFFFF);

	rtc_data->rtc = rtc_device_register(pdev->name, &pdev->dev,
				&mxs_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc_data->rtc)) {
		kfree(rtc_data);
		return PTR_ERR(rtc_data->rtc);
	}

	platform_set_drvdata(pdev, rtc_data);

	device_init_wakeup(&pdev->dev, 1);

	return 0;
}

static int mxs_rtc_remove(struct platform_device *dev)
{
	struct mxs_rtc_data *rtc_data = platform_get_drvdata(dev);

	if (rtc_data) {
		rtc_device_unregister(rtc_data->rtc);
		kfree(rtc_data);
	}

	return 0;
}

#ifdef CONFIG_PM
static int mxs_rtc_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

static int mxs_rtc_resume(struct platform_device *dev)
{
	struct mxs_rtc_data *rtc_data = platform_get_drvdata(dev);

	__raw_writel(BM_RTC_PERSISTENT0_ALARM_EN |
			BM_RTC_PERSISTENT0_ALARM_WAKE_EN |
			BM_RTC_PERSISTENT0_ALARM_WAKE,
		     rtc_data->base + HW_RTC_PERSISTENT0_CLR);
	return 0;
}
#else
#define mxs_rtc_suspend	NULL
#define mxs_rtc_resume	NULL
#endif

static struct platform_driver mxs_rtcdrv = {
	.probe		= mxs_rtc_probe,
	.remove		= mxs_rtc_remove,
	.suspend	= mxs_rtc_suspend,
	.resume		= mxs_rtc_resume,
	.driver		= {
		.name	= "mxs-rtc",
		.owner	= THIS_MODULE,
	},
};

static int __init mxs_rtc_init(void)
{
	return platform_driver_register(&mxs_rtcdrv);
}

static void __exit mxs_rtc_exit(void)
{
	platform_driver_unregister(&mxs_rtcdrv);
}

module_init(mxs_rtc_init);
module_exit(mxs_rtc_exit);

MODULE_DESCRIPTION("MXS RTC Driver");
MODULE_AUTHOR("dmitry pervushin <dimka@embeddedalley.com>");
MODULE_LICENSE("GPL");
