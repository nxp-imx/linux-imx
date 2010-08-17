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
#include <linux/rtc.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <linux/pmic_status.h>
#include <linux/pmic_external.h>

#define RTC_TIME_LSH		0
#define RTC_DAY_LSH		0
#define RTCALARM_TIME_LSH	0
#define RTCALARM_DAY_LSH	0

#define RTC_TIME_WID		17
#define RTC_DAY_WID		15
#define RTCALARM_TIME_WID	17
#define RTCALARM_DAY_WID	15

static unsigned long rtc_status;

static int mxc_rtc_open(struct device *dev)
{
	if (test_and_set_bit(1, &rtc_status))
		return -EBUSY;
	return 0;
}

static void mxc_rtc_release(struct device *dev)
{
	clear_bit(1, &rtc_status);
}

static int mxc_rtc_ioctl(struct device *dev, unsigned int cmd,
			 unsigned long arg)
{
	switch (cmd) {
	case RTC_AIE_OFF:
		pr_debug("alarm off\n");
		CHECK_ERROR(pmic_write_reg(REG_RTC_ALARM, 0x100000, 0x100000));
		return 0;
	case RTC_AIE_ON:
		pr_debug("alarm on\n");
		CHECK_ERROR(pmic_write_reg(REG_RTC_ALARM, 0, 0x100000));
		return 0;
	}

	return -ENOIOCTLCMD;
}

static int mxc_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	unsigned int tod_reg_val = 0;
	unsigned int day_reg_val = 0, day_reg_val2;
	unsigned int mask, value;
	unsigned long time;

	do {
		mask = BITFMASK(RTC_DAY);
		CHECK_ERROR(pmic_read_reg(REG_RTC_DAY, &value, mask));
		day_reg_val = BITFEXT(value, RTC_DAY);

		mask = BITFMASK(RTC_TIME);
		CHECK_ERROR(pmic_read_reg(REG_RTC_TIME, &value, mask));
		tod_reg_val = BITFEXT(value, RTC_TIME);

		mask = BITFMASK(RTC_DAY);
		CHECK_ERROR(pmic_read_reg(REG_RTC_DAY, &value, mask));
		day_reg_val2 = BITFEXT(value, RTC_DAY);
	} while (day_reg_val != day_reg_val2);

	time = (unsigned long)((unsigned long)(tod_reg_val &
					       0x0001FFFF) +
			       (unsigned long)(day_reg_val * 86400));

	rtc_time_to_tm(time, tm);

	return 0;
}

static int mxc_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned int tod_reg_val = 0;
	unsigned int day_reg_val, day_reg_val2 = 0;
	unsigned int mask, value;
	unsigned long time;

	if (rtc_valid_tm(tm))
		return -1;

	rtc_tm_to_time(tm, &time);

	tod_reg_val = time % 86400;
	day_reg_val = time / 86400;

	do {
		mask = BITFMASK(RTC_DAY);
		value = BITFVAL(RTC_DAY, day_reg_val);
		CHECK_ERROR(pmic_write_reg(REG_RTC_DAY, value, mask));

		mask = BITFMASK(RTC_TIME);
		value = BITFVAL(RTC_TIME, tod_reg_val);
		CHECK_ERROR(pmic_write_reg(REG_RTC_TIME, value, mask));

		mask = BITFMASK(RTC_DAY);
		CHECK_ERROR(pmic_read_reg(REG_RTC_DAY, &value, mask));
		day_reg_val2 = BITFEXT(value, RTC_DAY);
	} while (day_reg_val != day_reg_val2);

	return 0;
}

static int mxc_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	unsigned int tod_reg_val = 0;
	unsigned int day_reg_val = 0;
	unsigned int mask, value;
	unsigned long time;

	mask = BITFMASK(RTCALARM_TIME);
	CHECK_ERROR(pmic_read_reg(REG_RTC_ALARM, &value, mask));
	tod_reg_val = BITFEXT(value, RTCALARM_TIME);

	mask = BITFMASK(RTCALARM_DAY);
	CHECK_ERROR(pmic_read_reg(REG_RTC_DAY_ALARM, &value, mask));
	day_reg_val = BITFEXT(value, RTCALARM_DAY);

	time = (unsigned long)((unsigned long)(tod_reg_val &
					       0x0001FFFF) +
			       (unsigned long)(day_reg_val * 86400));
	rtc_time_to_tm(time, &(alrm->time));

	return 0;
}

static int mxc_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	unsigned int tod_reg_val = 0;
	unsigned int day_reg_val = 0;
	unsigned int mask, value;
	unsigned long time;

	if (rtc_valid_tm(&alrm->time))
		return -1;

	rtc_tm_to_time(&alrm->time, &time);

	tod_reg_val = time % 86400;
	day_reg_val = time / 86400;

	mask = BITFMASK(RTCALARM_TIME);
	value = BITFVAL(RTCALARM_TIME, tod_reg_val);
	CHECK_ERROR(pmic_write_reg(REG_RTC_ALARM, value, mask));

	mask = BITFMASK(RTCALARM_DAY);
	value = BITFVAL(RTCALARM_DAY, day_reg_val);
	CHECK_ERROR(pmic_write_reg(REG_RTC_DAY_ALARM, value, mask));

	return 0;
}

struct rtc_drv_data {
	struct rtc_device *rtc;
	pmic_event_callback_t event;
};

static struct rtc_class_ops mxc_rtc_ops = {
	.open = mxc_rtc_open,
	.release = mxc_rtc_release,
	.ioctl = mxc_rtc_ioctl,
	.read_time = mxc_rtc_read_time,
	.set_time = mxc_rtc_set_time,
	.read_alarm = mxc_rtc_read_alarm,
	.set_alarm = mxc_rtc_set_alarm,
};

static void mxc_rtc_alarm_int(void *data)
{
	struct rtc_drv_data *pdata = data;

	rtc_update_irq(pdata->rtc, 1, RTC_AF | RTC_IRQF);
}

static int mxc_rtc_probe(struct platform_device *pdev)
{
	struct rtc_drv_data *pdata = NULL;

	printk(KERN_INFO "mc13892 rtc probe start\n");

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);

	if (!pdata)
		return -ENOMEM;

	pdata->event.func = mxc_rtc_alarm_int;
	pdata->event.param = pdata;
	CHECK_ERROR(pmic_event_subscribe(EVENT_TODAI, pdata->event));

	device_init_wakeup(&pdev->dev, 1);
	pdata->rtc = rtc_device_register(pdev->name, &pdev->dev,
					 &mxc_rtc_ops, THIS_MODULE);

	platform_set_drvdata(pdev, pdata);
	if (IS_ERR(pdata->rtc))
		return -1;

	printk(KERN_INFO "mc13892 rtc probe succeed\n");
	return 0;
}

static int __exit mxc_rtc_remove(struct platform_device *pdev)
{
	struct rtc_drv_data *pdata = platform_get_drvdata(pdev);

	rtc_device_unregister(pdata->rtc);
	CHECK_ERROR(pmic_event_unsubscribe(EVENT_TODAI, pdata->event));

	return 0;
}

static struct platform_driver mxc_rtc_driver = {
	.driver = {
		   .name = "pmic_rtc",
		   },
	.probe = mxc_rtc_probe,
	.remove = __exit_p(mxc_rtc_remove),
};

static int __init mxc_rtc_init(void)
{
	return platform_driver_register(&mxc_rtc_driver);
}

static void __exit mxc_rtc_exit(void)
{
	platform_driver_unregister(&mxc_rtc_driver);

}

module_init(mxc_rtc_init);
module_exit(mxc_rtc_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MC13892 Realtime Clock Driver (RTC)");
MODULE_LICENSE("GPL");
