/*
 * Copyright(c) 2009 Dialog Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * rtc-da9052.c: RTC driver for DA9052
 */

#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/mfd/da9052/da9052.h>
#include <linux/mfd/da9052/reg.h>
#include <linux/mfd/da9052/rtc.h>

#define DRIVER_NAME "da9052-rtc"
#define ENABLE		1
#define DISABLE		0

struct da9052_rtc {
	struct rtc_device *rtc;
	struct da9052 *da9052;
	struct da9052_eh_nb eh_data;
	unsigned char is_min_alarm;
	unsigned char enable_tick_alarm;
	unsigned char enable_clk_buffer;
	unsigned char set_osc_trim_freq;
};

static int da9052_rtc_enable_alarm(struct da9052 *da9052, unsigned char flag);

void da9052_rtc_notifier(struct da9052_eh_nb *eh_data, unsigned int event)
{
	struct da9052_rtc *rtc =
		container_of(eh_data, struct da9052_rtc, eh_data);
	struct da9052_ssc_msg msg;
	unsigned int ret;

	/* Check the alarm type - TIMER or TICK */
	msg.addr = DA9052_ALARMMI_REG;

	da9052_lock(rtc->da9052);
	ret = rtc->da9052->read(rtc->da9052, &msg);
	if (ret != 0) {
		da9052_unlock(rtc->da9052);
		return;
	}

	da9052_unlock(rtc->da9052);


	if (msg.data & DA9052_ALARMMI_ALARMTYPE) {
		da9052_rtc_enable_alarm(rtc->da9052, 0);
		printk(KERN_INFO "RTC: TIMER ALARM\n");
	} else {
		kobject_uevent(&rtc->rtc->dev.kobj, KOBJ_CHANGE);
		printk(KERN_INFO "RTC: TICK ALARM\n");
	}
}

static int da9052_rtc_validate_parameters(struct rtc_time *rtc_tm)
{

	if (rtc_tm->tm_sec > DA9052_RTC_SECONDS_LIMIT)
		return DA9052_RTC_INVALID_SECONDS;

	if (rtc_tm->tm_min > DA9052_RTC_MINUTES_LIMIT)
		return DA9052_RTC_INVALID_MINUTES;

	if (rtc_tm->tm_hour > DA9052_RTC_HOURS_LIMIT)
		return DA9052_RTC_INVALID_HOURS;

	if (rtc_tm->tm_mday == 0)
		return DA9052_RTC_INVALID_DAYS;

	if ((rtc_tm->tm_mon > DA9052_RTC_MONTHS_LIMIT) ||
	(rtc_tm->tm_mon == 0))
		return DA9052_RTC_INVALID_MONTHS;

	if (rtc_tm->tm_year > DA9052_RTC_YEARS_LIMIT)
		return DA9052_RTC_INVALID_YEARS;

	if ((rtc_tm->tm_mon == FEBRUARY)) {
		if (((rtc_tm->tm_year % 4 == 0) &&
			(rtc_tm->tm_year % 100 != 0)) ||
			(rtc_tm->tm_year % 400 == 0)) {
			if (rtc_tm->tm_mday > 29)
				return DA9052_RTC_INVALID_DAYS;
		} else if (rtc_tm->tm_mday > 28) {
			return DA9052_RTC_INVALID_DAYS;
		}
	}

	if (((rtc_tm->tm_mon == APRIL) || (rtc_tm->tm_mon == JUNE) ||
		(rtc_tm->tm_mon == SEPTEMBER) || (rtc_tm->tm_mon == NOVEMBER))
		&& (rtc_tm->tm_mday == 31)) {
		return DA9052_RTC_INVALID_DAYS;
	}


	return 0;
}

static int da9052_rtc_settime(struct da9052 *da9052, struct rtc_time *rtc_tm)
{

	struct da9052_ssc_msg msg_arr[6];
	int validate_param = 0;
	unsigned char loop_index = 0;
	int ret = 0;


	/* System compatability */
	rtc_tm->tm_year -= 100;
	rtc_tm->tm_mon += 1;

	validate_param = da9052_rtc_validate_parameters(rtc_tm);
	if (validate_param)
		return validate_param;

	msg_arr[loop_index].addr = DA9052_COUNTS_REG;
	msg_arr[loop_index++].data = DA9052_COUNTS_MONITOR | rtc_tm->tm_sec;

	msg_arr[loop_index].addr = DA9052_COUNTMI_REG;
	msg_arr[loop_index].data = 0;
	msg_arr[loop_index++].data = rtc_tm->tm_min;

	msg_arr[loop_index].addr = DA9052_COUNTH_REG;
	msg_arr[loop_index].data = 0;
	msg_arr[loop_index++].data = rtc_tm->tm_hour;

	msg_arr[loop_index].addr = DA9052_COUNTD_REG;
	msg_arr[loop_index].data = 0;
	msg_arr[loop_index++].data = rtc_tm->tm_mday;

	msg_arr[loop_index].addr = DA9052_COUNTMO_REG;
	msg_arr[loop_index].data = 0;
	msg_arr[loop_index++].data = rtc_tm->tm_mon;

	msg_arr[loop_index].addr = DA9052_COUNTY_REG;
	msg_arr[loop_index].data = 0;
	msg_arr[loop_index++].data = rtc_tm->tm_year;

	da9052_lock(da9052);
	ret = da9052->write_many(da9052, msg_arr, loop_index);
	if (ret != 0) {
		da9052_unlock(da9052);
		return ret;
	}

	da9052_unlock(da9052);
	return 0;
}

static int da9052_rtc_gettime(struct da9052 *da9052, struct rtc_time *rtc_tm)
{

	struct da9052_ssc_msg msg[6];
	unsigned char loop_index = 0;
	int validate_param = 0;
	int ret = 0;

	msg[loop_index].data = 0;
	msg[loop_index++].addr = DA9052_COUNTS_REG;

	msg[loop_index].data = 0;
	msg[loop_index++].addr = DA9052_COUNTMI_REG;

	msg[loop_index].data = 0;
	msg[loop_index++].addr = DA9052_COUNTH_REG;

	msg[loop_index].data = 0;
	msg[loop_index++].addr = DA9052_COUNTD_REG;

	msg[loop_index].data = 0;
	msg[loop_index++].addr = DA9052_COUNTMO_REG;

	msg[loop_index].data = 0;
	msg[loop_index++].addr = DA9052_COUNTY_REG;

	da9052_lock(da9052);
	ret = da9052->read_many(da9052, msg, loop_index);
	if (ret != 0) {
		da9052_unlock(da9052);
		return ret;
	}
	da9052_unlock(da9052);

	rtc_tm->tm_year = msg[--loop_index].data & DA9052_COUNTY_COUNTYEAR;
	rtc_tm->tm_mon = msg[--loop_index].data & DA9052_COUNTMO_COUNTMONTH;
	rtc_tm->tm_mday = msg[--loop_index].data & DA9052_COUNTD_COUNTDAY;
	rtc_tm->tm_hour = msg[--loop_index].data & DA9052_COUNTH_COUNTHOUR;
	rtc_tm->tm_min = msg[--loop_index].data & DA9052_COUNTMI_COUNTMIN;
	rtc_tm->tm_sec = msg[--loop_index].data & DA9052_COUNTS_COUNTSEC;

	validate_param = da9052_rtc_validate_parameters(rtc_tm);
	if (validate_param)
		return validate_param;

	/* System compatability */
	rtc_tm->tm_year += 100;
	rtc_tm->tm_mon -= 1;
	return 0;
}

static int da9052_alarm_gettime(struct da9052 *da9052, struct rtc_time *rtc_tm)
{
	struct da9052_ssc_msg msg[5];
	unsigned char loop_index = 0;
	int validate_param = 0;
	int ret = 0;

	msg[loop_index].data = 0;
	msg[loop_index++].addr = DA9052_ALARMMI_REG;

	msg[loop_index].data = 0;
	msg[loop_index++].addr = DA9052_ALARMH_REG;

	msg[loop_index].data = 0;
	msg[loop_index++].addr = DA9052_ALARMD_REG;

	msg[loop_index].data = 0;
	msg[loop_index++].addr = DA9052_ALARMMO_REG;

	msg[loop_index].data = 0;
	msg[loop_index++].addr = DA9052_ALARMY_REG;

	da9052_lock(da9052);
	ret = da9052->read_many(da9052, msg, loop_index);
	if (ret != 0) {
		da9052_unlock(da9052);
		return ret;
	}
	da9052_unlock(da9052);

	rtc_tm->tm_year = msg[--loop_index].data & DA9052_ALARMY_ALARMYEAR;
	rtc_tm->tm_mon = msg[--loop_index].data & DA9052_ALARMMO_ALARMMONTH;
	rtc_tm->tm_mday = msg[--loop_index].data & DA9052_ALARMD_ALARMDAY;
	rtc_tm->tm_hour = msg[--loop_index].data & DA9052_ALARMH_ALARMHOUR;
	rtc_tm->tm_min = msg[--loop_index].data & DA9052_ALARMMI_ALARMMIN;

	validate_param = da9052_rtc_validate_parameters(rtc_tm);
	if (validate_param)
		return validate_param;

	/* System compatability */
	rtc_tm->tm_year += 100;
	rtc_tm->tm_mon -= 1;

	return 0;
}

static int da9052_alarm_settime(struct da9052 *da9052, struct rtc_time *rtc_tm)
{

	struct da9052_ssc_msg msg_arr[5];
	struct da9052_ssc_msg msg;
	int validate_param = 0;
	unsigned char loop_index = 0;
	int ret = 0;

	rtc_tm->tm_sec = 0;

	/* System compatability */
	rtc_tm->tm_year -= 100;
	rtc_tm->tm_mon += 1;

	validate_param = da9052_rtc_validate_parameters(rtc_tm);
	if (validate_param)
		return validate_param;

	msg.addr = DA9052_ALARMMI_REG;
	msg.data = 0;

	da9052_lock(da9052);
	ret = da9052->read(da9052, &msg);
	if (ret != 0) {
		da9052_unlock(da9052);
		return ret;
	}

	msg.data = msg.data & ~(DA9052_ALARMMI_ALARMMIN);
	msg.data |= rtc_tm->tm_min;

	msg_arr[loop_index].addr = DA9052_ALARMMI_REG;
	msg_arr[loop_index].data = 0;
	msg_arr[loop_index++].data = msg.data;

	msg_arr[loop_index].addr = DA9052_ALARMH_REG;
	msg_arr[loop_index].data = 0;
	msg_arr[loop_index++].data = rtc_tm->tm_hour;

	msg_arr[loop_index].addr = DA9052_ALARMD_REG;
	msg_arr[loop_index].data = 0;
	msg_arr[loop_index++].data = rtc_tm->tm_mday;

	msg_arr[loop_index].addr = DA9052_ALARMMO_REG;
	msg_arr[loop_index].data = 0;
	msg_arr[loop_index++].data = rtc_tm->tm_mon;

	msg.addr = DA9052_ALARMY_REG;
	msg.data = 0;
	ret = da9052->read(da9052, &msg);
	if (ret != 0) {
		da9052_unlock(da9052);
		return ret;
	}

	msg.data = msg.data & ~(DA9052_ALARMY_ALARMYEAR);


	msg.data |= rtc_tm->tm_year;
	msg_arr[loop_index].addr = DA9052_ALARMY_REG;
	msg_arr[loop_index].data = 0;
	msg_arr[loop_index++].data = msg.data;

	ret = da9052->write_many(da9052, msg_arr, loop_index);
	if (ret) {
		da9052_unlock(da9052);
		return ret;
	}

	da9052_unlock(da9052);
	return 0;
}

static int da9052_rtc_get_alarm_status(struct da9052 *da9052)
{
	struct da9052_ssc_msg msg;
	int ret = 0;

	msg.addr = DA9052_ALARMY_REG;
	msg.data = 0;
	da9052_lock(da9052);
	ret = da9052->read(da9052, &msg);
	if (ret != 0) {
			da9052_unlock(da9052);
	return ret;
	}

	da9052_unlock(da9052);
	msg.data &= DA9052_ALARMY_ALARMON;

	return (msg.data > 0) ? 1 : 0;
}


static int da9052_rtc_enable_alarm(struct da9052 *da9052, unsigned char flag)
{
	struct da9052_ssc_msg msg;
	int ret = 0;

	msg.addr = DA9052_ALARMY_REG;
	da9052_lock(da9052);
	ret = da9052->read(da9052, &msg);
	if (ret != 0) {
		da9052_unlock(da9052);
		return ret;
	}

	if (flag)
		msg.data = msg.data | DA9052_ALARMY_ALARMON;
	else
		msg.data = msg.data & ~(DA9052_ALARMY_ALARMON);

	ret = da9052->write(da9052, &msg);
	if (ret != 0) {
		da9052_unlock(da9052);
		return ret;
	}
	da9052_unlock(da9052);

	return 0;
}


static ssize_t da9052_rtc_mask_irq(struct da9052 *da9052)
 {
	unsigned char data = 0;
	ssize_t ret = 0;
	struct da9052_ssc_msg ssc_msg;

	ssc_msg.addr = DA9052_IRQMASKA_REG;
	ssc_msg.data = 0;

	da9052_lock(da9052);
	ret = da9052->read(da9052, &ssc_msg);
	if (ret != 0) {
		da9052_unlock(da9052);
		return ret;
	}

	data = ret;
	ssc_msg.data = data |= DA9052_IRQMASKA_MALRAM;

	ret = da9052->write(da9052, &ssc_msg);
	if (ret != 0) {
		da9052_unlock(da9052);
		return ret;
	}

	da9052_unlock(da9052);
	return 0;
}


static ssize_t da9052_rtc_unmask_irq(struct da9052 *da9052)
{
	unsigned char data = 0;
	ssize_t ret = 0;
	struct da9052_ssc_msg ssc_msg;

	ssc_msg.addr =  DA9052_IRQMASKA_REG;
	ssc_msg.data =  0;

	da9052_lock(da9052);
	ret = da9052->read(da9052, &ssc_msg);
	if (ret != 0) {
		da9052_unlock(da9052);
		return ret;
	}

	data = ret;
	ssc_msg.data = data &= ~DA9052_IRQMASKA_MALRAM;

	ret = da9052->write(da9052, &ssc_msg);
	if (ret != 0) {
		da9052_unlock(da9052);
		return ret;
	}

	da9052_unlock(da9052);
	return 0;

}

static int da9052_rtc_class_ops_gettime
			(struct device *dev, struct rtc_time *rtc_tm)
{
	int ret;
	struct da9052 *da9052 = dev_get_drvdata(dev->parent);
	ret = da9052_rtc_gettime(da9052, rtc_tm);
	if (ret)
		return ret;
	return 0;
}


static int da9052_rtc_class_ops_settime(struct device *dev, struct rtc_time *tm)
{
	int ret;
	struct da9052 *da9052 = dev_get_drvdata(dev->parent);
	ret = da9052_rtc_settime(da9052, tm);

	return ret;
}

static int da9052_rtc_readalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	int ret;
	struct rtc_time *tm = &alrm->time;
	struct da9052 *da9052 = dev_get_drvdata(dev->parent);
	ret = da9052_alarm_gettime(da9052, tm);

	if (ret)
		return ret;

	alrm->enabled = da9052_rtc_get_alarm_status(da9052);

	return 0;

}

static int da9052_rtc_setalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	int ret = 0;
	struct rtc_time *tm = &alrm->time;
	struct da9052 *da9052 = dev_get_drvdata(dev->parent);

	ret = da9052_alarm_settime(da9052, tm);

	if (ret)
		return ret;

	ret = da9052_rtc_enable_alarm(da9052, 1);

	return ret;
}

static int da9052_rtc_update_irq_enable(struct device *dev,
		unsigned int enabled)
{
	struct da9052_rtc *priv = dev_get_drvdata(dev);
	int ret = -ENODATA;

	da9052_lock(priv->da9052);

	ret = (enabled ? da9052_rtc_unmask_irq : da9052_rtc_mask_irq)
						(priv->da9052);

	da9052_unlock(priv->da9052);

	return ret;
}

static int da9052_rtc_alarm_irq_enable(struct device *dev,
			unsigned int enabled)
{
	struct da9052_rtc *priv = dev_get_drvdata(dev);

	if (enabled)
		return da9052_rtc_enable_alarm(priv->da9052, enabled);
	else
		return da9052_rtc_enable_alarm(priv->da9052, enabled);
}

static const struct rtc_class_ops da9052_rtc_ops = {
	.read_time	= da9052_rtc_class_ops_gettime,
	.set_time	= da9052_rtc_class_ops_settime,
	.read_alarm	= da9052_rtc_readalarm,
	.set_alarm	= da9052_rtc_setalarm,
#if 0
	.update_irq_enable = da9052_rtc_update_irq_enable,
	.alarm_irq_enable = da9052_rtc_alarm_irq_enable,
#endif
};


static int __devinit da9052_rtc_probe(struct platform_device *pdev)
{
	int ret;
	struct da9052_rtc *priv;
	struct da9052_ssc_msg ssc_msg;
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->da9052 = dev_get_drvdata(pdev->dev.parent);
	platform_set_drvdata(pdev, priv);

	/* Added to support sysfs wakealarm attribute */
	pdev->dev.power.can_wakeup = 1;
	/* Added to support sysfs wakealarm attribute */

	/* Set the EH structure */
	priv->eh_data.eve_type = ALARM_EVE;
	priv->eh_data.call_back = &da9052_rtc_notifier;
	ret = priv->da9052->register_event_notifier(priv->da9052,
		&priv->eh_data);
	if (ret)
		goto err_register_alarm;

	priv->is_min_alarm = 1;
	priv->enable_tick_alarm = 1;
	priv->enable_clk_buffer = 1;
	priv->set_osc_trim_freq = 5;
	/* Enable/Disable TICK Alarm */
	/* Read ALARM YEAR register */
	ssc_msg.addr = DA9052_ALARMY_REG;
	ssc_msg.data = 0;

	da9052_lock(priv->da9052);
	ret = priv->da9052->read(priv->da9052, &ssc_msg);
	if (ret != 0) {
		da9052_unlock(priv->da9052);
		goto err_ssc_comm;
	}

	if (priv->enable_tick_alarm)
		ssc_msg.data = (ssc_msg.data | DA9052_ALARMY_TICKON);
	else
		ssc_msg.data =
		((ssc_msg.data & ~(DA9052_ALARMY_TICKON)));

	ret = priv->da9052->write(priv->da9052, &ssc_msg);
	if (ret != 0) {
		da9052_unlock(priv->da9052);
		goto err_ssc_comm;
	}

	/* Set TICK Alarm to 1 minute or 1 sec */
	/* Read ALARM MINUTES register */
	ssc_msg.addr = DA9052_ALARMMI_REG;
	ssc_msg.data = 0;

	ret = priv->da9052->read(priv->da9052, &ssc_msg);
	if (ret != 0) {
		da9052_unlock(priv->da9052);
		goto err_ssc_comm;
	}

	if (priv->is_min_alarm)
		/* Set 1 minute tick type */
		ssc_msg.data = (ssc_msg.data | DA9052_ALARMMI_TICKTYPE);
	else
		/* Set 1 sec tick type */
		ssc_msg.data = (ssc_msg.data & ~(DA9052_ALARMMI_TICKTYPE));

	ret = priv->da9052->write(priv->da9052, &ssc_msg);
	if (ret != 0) {
		da9052_unlock(priv->da9052);
		goto err_ssc_comm;
	}

	/* Enable/Disable Clock buffer in Power Down Mode */
	ssc_msg.addr = DA9052_PDDIS_REG;
	ssc_msg.data = 0;

	ret = priv->da9052->read(priv->da9052, &ssc_msg);
	if (ret != 0) {
		da9052_unlock(priv->da9052);
		goto err_ssc_comm;
	}

	if (priv->enable_clk_buffer)
		ssc_msg.data = (ssc_msg.data | DA9052_PDDIS_OUT32KPD);
	else
		ssc_msg.data = (ssc_msg.data & ~(DA9052_PDDIS_OUT32KPD));

	ret = priv->da9052->write(priv->da9052, &ssc_msg);
	if (ret != 0) {
		da9052_unlock(priv->da9052);
		goto err_ssc_comm;
	}

	/* Set clock trim frequency value */
	ssc_msg.addr = DA9052_OSCTRIM_REG;
	ssc_msg.data = priv->set_osc_trim_freq;

	ret = priv->da9052->write(priv->da9052, &ssc_msg);
	if (ret != 0) {
		da9052_unlock(priv->da9052);
		goto err_ssc_comm;
	}
	da9052_unlock(priv->da9052);

	priv->rtc = rtc_device_register(pdev->name,
			&pdev->dev, &da9052_rtc_ops, THIS_MODULE);
	if (IS_ERR(priv->rtc)) {
		ret = PTR_ERR(priv->rtc);
		goto err_ssc_comm;
	}
	return 0;

err_ssc_comm:
		priv->da9052->unregister_event_notifier
			(priv->da9052, &priv->eh_data);
err_register_alarm:
		platform_set_drvdata(pdev, NULL);
		kfree(priv);

	return ret;
}

static int __devexit da9052_rtc_remove(struct platform_device *pdev)
{
	struct da9052_rtc *priv = platform_get_drvdata(pdev);
	rtc_device_unregister(priv->rtc);
	da9052_lock(priv->da9052);
	priv->da9052->unregister_event_notifier(priv->da9052, &priv->eh_data);
	da9052_unlock(priv->da9052);
	platform_set_drvdata(pdev, NULL);
	kfree(priv);
	return 0;
}

static struct platform_driver da9052_rtc_driver = {
	.probe = da9052_rtc_probe,
	.remove = __devexit_p(da9052_rtc_remove),
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};


static int __init da9052_rtc_init(void)
{
	return platform_driver_register(&da9052_rtc_driver);
}
module_init(da9052_rtc_init);

static void __exit da9052_rtc_exit(void)
{
	platform_driver_unregister(&da9052_rtc_driver);
}
module_exit(da9052_rtc_exit);

MODULE_AUTHOR("Dialog Semiconductor Ltd <dchen@diasemi.com>");
MODULE_DESCRIPTION("RTC driver for Dialog DA9052 PMIC");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
