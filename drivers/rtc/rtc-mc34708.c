
/*
 * RTC  Driver for Freescale MC34708 PMIC
 *
 * Copyright (C) 2004-2012 Freescale Semiconductor, Inc.
 *
 * Based on mc13xxx rtc  driver :
 * Copyright 2009 Pengutronix, Sascha Hauer <s.hauer@pengutronix.de>
 * Copyright 2009 Pengutronix, Uwe Kleine-Koenig
 * <u.kleine-koenig@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/rtc.h>
#include <linux/mfd/mc-pmic.h>

#define DRIVER_NAME "mc34708-rtc"

#define MC34708_RTCTOD	20
#define MC34708_RTCTODA	21
#define MC34708_RTCDAY	22
#define MC34708_RTCDAYA	23

struct mc34708_rtc {
	struct rtc_device *rtc;
	struct mc_pmic *mc_pmic;
	int valid;
	int hz_alarm;		/*workaround the 1hz_alarm loss interrupt */
};

static int
mc34708_rtc_irq_enable_unlocked(struct device *dev,
				unsigned int enabled, int irq)
{
	struct mc34708_rtc *priv = dev_get_drvdata(dev);
	int (*func) (struct mc_pmic *mc_pmic, int irq);

	if (!priv->valid)
		return -ENODATA;

	func = enabled ? mc_pmic_irq_unmask : mc_pmic_irq_mask;
	return func(priv->mc_pmic, irq);
}

static int
mc34708_rtc_irq_enable(struct device *dev, unsigned int enabled, int irq)
{
	struct mc34708_rtc *priv = dev_get_drvdata(dev);
	int ret;

	mc_pmic_lock(priv->mc_pmic);

	ret = mc34708_rtc_irq_enable_unlocked(dev, enabled, irq);

	mc_pmic_unlock(priv->mc_pmic);

	return ret;
}

static int mc34708_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct mc34708_rtc *priv = dev_get_drvdata(dev);
	unsigned int seconds, days1, days2;
	unsigned long s1970;
	int ret;

	mc_pmic_lock(priv->mc_pmic);

	if (!priv->valid) {
		ret = -ENODATA;
		goto out;
	}

	ret = mc_pmic_reg_read(priv->mc_pmic, MC34708_RTCDAY, &days1);
	if (unlikely(ret))
		goto out;

	ret = mc_pmic_reg_read(priv->mc_pmic, MC34708_RTCTOD, &seconds);
	if (unlikely(ret))
		goto out;

	ret = mc_pmic_reg_read(priv->mc_pmic, MC34708_RTCDAY, &days2);
 out:
	mc_pmic_unlock(priv->mc_pmic);

	if (ret)
		return ret;

	if (days2 == days1 + 1) {
		if (seconds >= 86400 / 2)
			days2 = days1;
		else
			days1 = days2;
	}

	if (days1 != days2)
		return -EIO;

	s1970 = days1 * 86400 + seconds;

	rtc_time_to_tm(s1970, tm);

	return rtc_valid_tm(tm);
}

static int mc34708_rtc_set_mmss(struct device *dev, unsigned long secs)
{
	struct mc34708_rtc *priv = dev_get_drvdata(dev);
	unsigned int seconds, days;
	unsigned int alarmseconds;
	int ret;

	seconds = secs % 86400;
	days = secs / 86400;

	mc_pmic_lock(priv->mc_pmic);

	/*
	 * temporarily invalidate alarm to prevent triggering it when the day is
	 * already updated while the time isn't yet.
	 */
	ret = mc_pmic_reg_read(priv->mc_pmic, MC34708_RTCTODA, &alarmseconds);
	if (unlikely(ret))
		goto out;

	if (alarmseconds < 86400) {
		ret =
		    mc_pmic_reg_write(priv->mc_pmic, MC34708_RTCTODA, 0x1ffff);
		if (unlikely(ret))
			goto out;
	}

	/*
	 * write seconds=0 to prevent a day switch between writing days
	 * and seconds below
	 */
	ret = mc_pmic_reg_write(priv->mc_pmic, MC34708_RTCTOD, 0);
	if (unlikely(ret))
		goto out;

	ret = mc_pmic_reg_write(priv->mc_pmic, MC34708_RTCDAY, days);
	if (unlikely(ret))
		goto out;

	ret = mc_pmic_reg_write(priv->mc_pmic, MC34708_RTCTOD, seconds);
	if (unlikely(ret))
		goto out;

	/* restore alarm */
	if (alarmseconds < 86400) {
		ret =
		    mc_pmic_reg_write(priv->mc_pmic, MC34708_RTCTODA,
				      alarmseconds);
		if (unlikely(ret))
			goto out;
	}

	ret = mc_pmic_irq_ack(priv->mc_pmic, MC_PMIC_IRQ_RTCRST);
	if (unlikely(ret))
		goto out;

	ret = mc_pmic_irq_unmask(priv->mc_pmic, MC_PMIC_IRQ_RTCRST);
 out:
	priv->valid = !ret;

	mc_pmic_unlock(priv->mc_pmic);

	return ret;
}

static int mc34708_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct mc34708_rtc *priv = dev_get_drvdata(dev);
	unsigned seconds, days;
	unsigned long s1970;
	int enabled, pending;
	int ret;

	mc_pmic_lock(priv->mc_pmic);

	ret = mc_pmic_reg_read(priv->mc_pmic, MC34708_RTCTODA, &seconds);
	if (unlikely(ret))
		goto out;
	if (seconds >= 86400) {
		ret = -ENODATA;
		goto out;
	}

	ret = mc_pmic_reg_read(priv->mc_pmic, MC34708_RTCDAY, &days);
	if (unlikely(ret))
		goto out;

	ret = mc_pmic_irq_status(priv->mc_pmic, MC_PMIC_IRQ_TODA,
				 &enabled, &pending);

 out:
	mc_pmic_unlock(priv->mc_pmic);

	if (ret)
		return ret;

	alarm->enabled = enabled;
	alarm->pending = pending;

	s1970 = days * 86400 + seconds;

	rtc_time_to_tm(s1970, &alarm->time);
	dev_dbg(dev, "%s: %lu\n", __func__, s1970);

	return 0;
}

static int mc34708_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct mc34708_rtc *priv = dev_get_drvdata(dev);
	unsigned long s1970;
	unsigned seconds, days, seconds_now, days_now;
	int ret;
	mc_pmic_lock(priv->mc_pmic);
	/* disable alarm to prevent false triggering */
	ret = mc_pmic_reg_write(priv->mc_pmic, MC34708_RTCTODA, 0x1ffff);
	if (unlikely(ret))
		goto out;

	ret = mc_pmic_irq_ack(priv->mc_pmic, MC_PMIC_IRQ_TODA);
	if (unlikely(ret))
		goto out;
	ret = rtc_tm_to_time(&alarm->time, &s1970);
	if (unlikely(ret))
		goto out;

	dev_dbg(dev, "%s: o%2.s %lu\n", __func__, alarm->enabled ? "n" : "ff",
		s1970);

	ret = mc34708_rtc_irq_enable_unlocked(dev, alarm->enabled,
					      MC_PMIC_IRQ_TODA);
	if (unlikely(ret))
		goto out;

	seconds = s1970 % 86400;
	days = s1970 / 86400;

	ret = mc_pmic_reg_read(priv->mc_pmic, MC34708_RTCDAY, &days_now);
	if (unlikely(ret))
		goto out;
	ret = mc_pmic_reg_read(priv->mc_pmic, MC34708_RTCTOD, &seconds_now);
	if (unlikely(ret))
		goto out;
	if ((days_now == days) && (seconds == seconds_now + 1)) {
		priv->hz_alarm = 1;
		/*enable 1hz */
		mc34708_rtc_irq_enable_unlocked(dev, 1, MC_PMIC_IRQ_1HZ);
		dev_dbg(dev, "1HZ_ALARM enabled!\n");
		goto out;
	}
	ret = mc_pmic_reg_write(priv->mc_pmic, MC34708_RTCDAYA, days);
	if (unlikely(ret))
		goto out;
	ret = mc_pmic_reg_write(priv->mc_pmic, MC34708_RTCTODA, seconds);

 out:
	mc_pmic_unlock(priv->mc_pmic);
	return ret;
}

static irqreturn_t mc34708_rtc_alarm_handler(int irq, void *dev)
{
	struct mc34708_rtc *priv = dev;
	struct mc_pmic *mc_pmic = priv->mc_pmic;

	dev_dbg(&priv->rtc->dev, "Alarm\n");

	rtc_update_irq(priv->rtc, 1, RTC_IRQF | RTC_AF);

	mc_pmic_irq_ack(mc_pmic, irq);

	return IRQ_HANDLED;
}

static irqreturn_t mc34708_rtc_update_handler(int irq, void *dev)
{
	struct mc34708_rtc *priv = dev;
	struct mc_pmic *mc_pmic = priv->mc_pmic;

	if (priv->hz_alarm == 1) {	/*replace alarm irq with 1hz irq */
		rtc_update_irq(priv->rtc, 1, RTC_IRQF | RTC_AF);
		priv->hz_alarm = 0;
		mc_pmic_irq_ack(mc_pmic, irq);
		mc_pmic_irq_mask(mc_pmic, irq);	/*disable 1Hz */
		dev_dbg(&priv->rtc->dev, "1HZ_ALARM!\n");

	} else {
		dev_dbg(&priv->rtc->dev, "1HZ\n");
		rtc_update_irq(priv->rtc, 1, RTC_IRQF | RTC_UF);
		mc_pmic_irq_ack(mc_pmic, irq);
	}

	return IRQ_HANDLED;
}

static int
mc34708_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	return mc34708_rtc_irq_enable(dev, enabled, MC_PMIC_IRQ_TODA);
}

static const struct rtc_class_ops mc34708_rtc_ops = {
	.read_time = mc34708_rtc_read_time,
	.set_mmss = mc34708_rtc_set_mmss,
	.read_alarm = mc34708_rtc_read_alarm,
	.set_alarm = mc34708_rtc_set_alarm,
	.alarm_irq_enable = mc34708_rtc_alarm_irq_enable,
};

static irqreturn_t mc34708_rtc_reset_handler(int irq, void *dev)
{
	struct mc34708_rtc *priv = dev;
	struct mc_pmic *mc_pmic = priv->mc_pmic;

	dev_dbg(&priv->rtc->dev, "RTCRST\n");
	priv->valid = 0;

	mc_pmic_irq_mask(mc_pmic, irq);

	return IRQ_HANDLED;
}

static int __devinit mc34708_rtc_probe(struct platform_device *pdev)
{
	int ret;
	struct mc34708_rtc *priv;
	struct mc_pmic *mc_pmic;
	int rtcrst_pending;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	mc_pmic = dev_get_drvdata(pdev->dev.parent);
	priv->mc_pmic = mc_pmic;

	platform_set_drvdata(pdev, priv);

	mc_pmic_lock(mc_pmic);

	ret = mc_pmic_irq_request_nounmask(mc_pmic, MC_PMIC_IRQ_RTCRST,
					   mc34708_rtc_reset_handler,
					   DRIVER_NAME, priv);
	if (ret)
		goto err_reset_irq_request;
	ret = mc_pmic_irq_status(mc_pmic, MC_PMIC_IRQ_RTCRST,
				 NULL, &rtcrst_pending);
	if (ret)
		goto err_reset_irq_status;
	priv->valid = rtcrst_pending;
	if (priv->valid) {
		/*if reset interrupt status have pending clear it */
		ret = mc_pmic_irq_ack(priv->mc_pmic, MC_PMIC_IRQ_RTCRST);
		if (ret)
			goto err_reset_irq_status;
	} else
		priv->valid = 1;
	mc34708_rtc_irq_enable_unlocked(&pdev->dev, 1, MC_PMIC_IRQ_RTCRST);
	priv->hz_alarm = 0;
	ret = mc_pmic_irq_request_nounmask(mc_pmic, MC_PMIC_IRQ_1HZ,
					   mc34708_rtc_update_handler,
					   DRIVER_NAME, priv);
	if (ret)
		goto err_update_irq_request;

	ret = mc_pmic_irq_request_nounmask(mc_pmic, MC_PMIC_IRQ_TODA,
					   mc34708_rtc_alarm_handler,
					   DRIVER_NAME, priv);
	if (ret)
		goto err_alarm_irq_request;

	priv->rtc = rtc_device_register(pdev->name,
					&pdev->dev, &mc34708_rtc_ops,
					THIS_MODULE);
	if (IS_ERR(priv->rtc)) {
		ret = PTR_ERR(priv->rtc);

		mc_pmic_irq_free(mc_pmic, MC_PMIC_IRQ_TODA, priv);
 err_alarm_irq_request:

		mc_pmic_irq_free(mc_pmic, MC_PMIC_IRQ_1HZ, priv);
 err_update_irq_request:

 err_reset_irq_status:

		mc_pmic_irq_free(mc_pmic, MC_PMIC_IRQ_RTCRST, priv);
 err_reset_irq_request:

		platform_set_drvdata(pdev, NULL);
		kfree(priv);
	}

	mc_pmic_unlock(mc_pmic);

	return ret;
}

static int __devexit mc34708_rtc_remove(struct platform_device *pdev)
{
	struct mc34708_rtc *priv = platform_get_drvdata(pdev);

	mc_pmic_lock(priv->mc_pmic);

	rtc_device_unregister(priv->rtc);

	mc_pmic_irq_free(priv->mc_pmic, MC_PMIC_IRQ_TODA, priv);
	mc_pmic_irq_free(priv->mc_pmic, MC_PMIC_IRQ_1HZ, priv);
	mc_pmic_irq_free(priv->mc_pmic, MC_PMIC_IRQ_RTCRST, priv);

	mc_pmic_unlock(priv->mc_pmic);

	platform_set_drvdata(pdev, NULL);

	kfree(priv);

	return 0;
}

const struct platform_device_id mc34708_rtc_idtable[] = {
	{
	 .name = "mc34708-rtc",
	 },

};

static struct platform_driver mc34708_rtc_driver = {
	.id_table = mc34708_rtc_idtable,
	.remove = __devexit_p(mc34708_rtc_remove),
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   },
};

static int __init mc34708_rtc_init(void)
{
	return platform_driver_probe(&mc34708_rtc_driver, &mc34708_rtc_probe);
}

module_init(mc34708_rtc_init);

static void __exit mc34708_rtc_exit(void)
{
	platform_driver_unregister(&mc34708_rtc_driver);
}

module_exit(mc34708_rtc_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("RTC driver for Freescale MC34708 PMIC");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
