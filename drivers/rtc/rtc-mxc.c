/*
 * Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
/*
 * Implementation based on rtc-ds1553.c
 */

/*!
 * @defgroup RTC Real Time Clock (RTC) Driver
 */
/*!
 * @file rtc-mxc.c
 * @brief Real Time Clock interface
 *
 * This file contains Real Time Clock interface for Linux.
 *
 * @ingroup RTC
 */

#include <linux/rtc.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/uaccess.h>

#include <mach/hardware.h>
#define RTC_INPUT_CLK_32768HZ	(0x00 << 5)
#define RTC_INPUT_CLK_32000HZ	(0x01 << 5)
#define RTC_INPUT_CLK_38400HZ	(0x02 << 5)

#define RTC_SW_BIT      (1 << 0)
#define RTC_ALM_BIT     (1 << 2)
#define RTC_1HZ_BIT     (1 << 4)
#define RTC_2HZ_BIT     (1 << 7)
#define RTC_SAM0_BIT    (1 << 8)
#define RTC_SAM1_BIT    (1 << 9)
#define RTC_SAM2_BIT    (1 << 10)
#define RTC_SAM3_BIT    (1 << 11)
#define RTC_SAM4_BIT    (1 << 12)
#define RTC_SAM5_BIT    (1 << 13)
#define RTC_SAM6_BIT    (1 << 14)
#define RTC_SAM7_BIT    (1 << 15)
#define PIT_ALL_ON      (RTC_2HZ_BIT | RTC_SAM0_BIT | RTC_SAM1_BIT | \
                         RTC_SAM2_BIT | RTC_SAM3_BIT | RTC_SAM4_BIT | \
                         RTC_SAM5_BIT | RTC_SAM6_BIT | RTC_SAM7_BIT)

#define RTC_ENABLE_BIT  (1 << 7)

#define MAX_PIE_NUM     9
#define MAX_PIE_FREQ    512
const u32 PIE_BIT_DEF[MAX_PIE_NUM][2] = {
	{2, RTC_2HZ_BIT},
	{4, RTC_SAM0_BIT},
	{8, RTC_SAM1_BIT},
	{16, RTC_SAM2_BIT},
	{32, RTC_SAM3_BIT},
	{64, RTC_SAM4_BIT},
	{128, RTC_SAM5_BIT},
	{256, RTC_SAM6_BIT},
	{MAX_PIE_FREQ, RTC_SAM7_BIT},
};

/* Those are the bits from a classic RTC we want to mimic */
#define RTC_IRQF                0x80	/* any of the following 3 is active */
#define RTC_PF                  0x40	/* Periodic interrupt */
#define RTC_AF                  0x20	/* Alarm interrupt */
#define RTC_UF                  0x10	/* Update interrupt for 1Hz RTC */

#define MXC_RTC_TIME            0
#define MXC_RTC_ALARM           1

#define RTC_HOURMIN    0x00	/*  32bit rtc hour/min counter reg */
#define RTC_SECOND     0x04	/*  32bit rtc seconds counter reg */
#define RTC_ALRM_HM    0x08	/*  32bit rtc alarm hour/min reg */
#define RTC_ALRM_SEC   0x0C	/*  32bit rtc alarm seconds reg */
#define RTC_RTCCTL     0x10	/*  32bit rtc control reg */
#define RTC_RTCISR     0x14	/*  32bit rtc interrupt status reg */
#define RTC_RTCIENR    0x18	/*  32bit rtc interrupt enable reg */
#define RTC_STPWCH     0x1C	/*  32bit rtc stopwatch min reg */
#define RTC_DAYR       0x20	/*  32bit rtc days counter reg */
#define RTC_DAYALARM   0x24	/*  32bit rtc day alarm reg */
#define RTC_TEST1      0x28	/*  32bit rtc test reg 1 */
#define RTC_TEST2      0x2C	/*  32bit rtc test reg 2 */
#define RTC_TEST3      0x30	/*  32bit rtc test reg 3 */

struct rtc_plat_data {
	struct rtc_device *rtc;
	void __iomem *ioaddr;
	unsigned long baseaddr;
	int irq;
	struct clk *clk;
	unsigned int irqen;
	int alrm_sec;
	int alrm_min;
	int alrm_hour;
	int alrm_mday;
};

/*!
 * @defgroup RTC Real Time Clock (RTC) Driver
 */
/*!
 * @file rtc-mxc.c
 * @brief Real Time Clock interface
 *
 * This file contains Real Time Clock interface for Linux.
 *
 * @ingroup RTC
 */

#if defined(CONFIG_MXC_PMIC_SC55112_RTC) || defined(CONFIG_MXC_MC13783_RTC) ||\
    defined(CONFIG_MXC_MC9SDZ60_RTC)
#include <linux/pmic_rtc.h>
#else
#define pmic_rtc_get_time(args)	MXC_EXTERNAL_RTC_NONE
#define pmic_rtc_set_time(args)	MXC_EXTERNAL_RTC_NONE
#define pmic_rtc_loaded()		0
#endif

#define RTC_VERSION		"1.0"
#define MXC_EXTERNAL_RTC_OK	0
#define MXC_EXTERNAL_RTC_ERR	-1
#define MXC_EXTERNAL_RTC_NONE	-2

/*!
 * This function reads the RTC value from some external source.
 *
 * @param  second       pointer to the returned value in second
 *
 * @return 0 if successful; non-zero otherwise
 */
int get_ext_rtc_time(u32 * second)
{
	int ret = 0;
	struct timeval tmp;
	if (!pmic_rtc_loaded()) {
		return MXC_EXTERNAL_RTC_NONE;
	}

	ret = pmic_rtc_get_time(&tmp);

	if (0 == ret)
		*second = tmp.tv_sec;
	else
		ret = MXC_EXTERNAL_RTC_ERR;

	return ret;
}

/*!
 * This function sets external RTC
 *
 * @param  second       value in second to be set to external RTC
 *
 * @return 0 if successful; non-zero otherwise
 */
int set_ext_rtc_time(u32 second)
{
	int ret = 0;
	struct timeval tmp;

	if (!pmic_rtc_loaded()) {
		return MXC_EXTERNAL_RTC_NONE;
	}

	tmp.tv_sec = second;

	ret = pmic_rtc_set_time(&tmp);

	if (0 != ret)
		ret = MXC_EXTERNAL_RTC_ERR;

	return ret;
}

static u32 rtc_freq = 2;	/* minimun value for PIE */
static unsigned long rtc_status;

static struct rtc_time g_rtc_alarm = {
	.tm_year = 0,
	.tm_mon = 0,
	.tm_mday = 0,
	.tm_hour = 0,
	.tm_mon = 0,
	.tm_sec = 0,
};

static DEFINE_SPINLOCK(rtc_lock);

/*!
 * This function is used to obtain the RTC time or the alarm value in
 * second.
 *
 * @param  time_alarm   use MXC_RTC_TIME for RTC time value; MXC_RTC_ALARM for alarm value
 *
 * @return The RTC time or alarm time in second.
 */
static u32 get_alarm_or_time(struct device *dev, int time_alarm)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtc_plat_data *pdata = platform_get_drvdata(pdev);
	void __iomem *ioaddr = pdata->ioaddr;
	u32 day, hr, min, sec, hr_min;
	if (time_alarm == MXC_RTC_TIME) {
		day = readw(ioaddr + RTC_DAYR);
		hr_min = readw(ioaddr + RTC_HOURMIN);
		sec = readw(ioaddr + RTC_SECOND);
	} else if (time_alarm == MXC_RTC_ALARM) {
		day = readw(ioaddr + RTC_DAYALARM);
		hr_min = (0x0000FFFF) & readw(ioaddr + RTC_ALRM_HM);
		sec = readw(ioaddr + RTC_ALRM_SEC);
	} else {
		panic("wrong value for time_alarm=%d\n", time_alarm);
	}

	hr = hr_min >> 8;
	min = hr_min & 0x00FF;

	return ((((day * 24 + hr) * 60) + min) * 60 + sec);
}

/*!
 * This function sets the RTC alarm value or the time value.
 *
 * @param  time_alarm   the new alarm value to be updated in the RTC
 * @param  time         use MXC_RTC_TIME for RTC time value; MXC_RTC_ALARM for alarm value
 */
static void set_alarm_or_time(struct device *dev, int time_alarm, u32 time)
{
	u32 day, hr, min, sec, temp;
	struct platform_device *pdev = to_platform_device(dev);
	struct rtc_plat_data *pdata = platform_get_drvdata(pdev);
	void __iomem *ioaddr = pdata->ioaddr;
	day = time / 86400;
	time -= day * 86400;
	/* time is within a day now */
	hr = time / 3600;
	time -= hr * 3600;
	/* time is within an hour now */
	min = time / 60;
	sec = time - min * 60;

	temp = (hr << 8) + min;

	if (time_alarm == MXC_RTC_TIME) {
		writew(day, ioaddr + RTC_DAYR);
		writew(sec, ioaddr + RTC_SECOND);
		writew(temp, ioaddr + RTC_HOURMIN);
	} else if (time_alarm == MXC_RTC_ALARM) {
		writew(day, ioaddr + RTC_DAYALARM);
		writew(sec, ioaddr + RTC_ALRM_SEC);
		writew(temp, ioaddr + RTC_ALRM_HM);
	} else {
		panic("wrong value for time_alarm=%d\n", time_alarm);
	}
}

/*!
 * This function updates the RTC alarm registers and then clears all the
 * interrupt status bits.
 *
 * @param  alrm         the new alarm value to be updated in the RTC
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int rtc_update_alarm(struct device *dev, struct rtc_time *alrm)
{
	struct rtc_time alarm_tm, now_tm;
	unsigned long now, time;
	int ret;
	struct platform_device *pdev = to_platform_device(dev);
	struct rtc_plat_data *pdata = platform_get_drvdata(pdev);
	void __iomem *ioaddr = pdata->ioaddr;

	now = get_alarm_or_time(dev, MXC_RTC_TIME);
	rtc_time_to_tm(now, &now_tm);
	alarm_tm.tm_year = now_tm.tm_year;
	alarm_tm.tm_mon = now_tm.tm_mon;
	alarm_tm.tm_mday = now_tm.tm_mday;
	alarm_tm.tm_hour = alrm->tm_hour;
	alarm_tm.tm_min = alrm->tm_min;
	alarm_tm.tm_sec = alrm->tm_sec;
	rtc_tm_to_time(&now_tm, &now);
	rtc_tm_to_time(&alarm_tm, &time);
	if (time < now) {
		time += 60 * 60 * 24;
		rtc_time_to_tm(time, &alarm_tm);
	}
	ret = rtc_tm_to_time(&alarm_tm, &time);

	/* clear all the interrupt status bits */
	writew(readw(ioaddr + RTC_RTCISR), ioaddr + RTC_RTCISR);

	set_alarm_or_time(dev, MXC_RTC_ALARM, time);

	return ret;
}

/*!
 * This function is the RTC interrupt service routine.
 *
 * @param  irq          RTC IRQ number
 * @param  dev_id       device ID which is not used
 *
 * @return IRQ_HANDLED as defined in the include/linux/interrupt.h file.
 */
static irqreturn_t mxc_rtc_interrupt(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct rtc_plat_data *pdata = platform_get_drvdata(pdev);
	void __iomem *ioaddr = pdata->ioaddr;
	u32 status;
	u32 events = 0;
	spin_lock(&rtc_lock);
	status = readw(ioaddr + RTC_RTCISR) & readw(ioaddr + RTC_RTCIENR);
	/* clear interrupt sources */
	writew(status, ioaddr + RTC_RTCISR);

	/* clear alarm interrupt if it has occurred */
	if (status & RTC_ALM_BIT) {
		status &= ~RTC_ALM_BIT;
	}

	/* update irq data & counter */
	if (status & RTC_ALM_BIT) {
		events |= (RTC_AF | RTC_IRQF);
	}
	if (status & RTC_1HZ_BIT) {
		events |= (RTC_UF | RTC_IRQF);
	}
	if (status & PIT_ALL_ON) {
		events |= (RTC_PF | RTC_IRQF);
	}

	if ((status & RTC_ALM_BIT) && rtc_valid_tm(&g_rtc_alarm)) {
		rtc_update_alarm(&pdev->dev, &g_rtc_alarm);
	}

	spin_unlock(&rtc_lock);
	rtc_update_irq(pdata->rtc, 1, events);
	return IRQ_HANDLED;
}

/*!
 * This function is used to open the RTC driver by registering the RTC
 * interrupt service routine.
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int mxc_rtc_open(struct device *dev)
{
	if (test_and_set_bit(1, &rtc_status))
		return -EBUSY;
	return 0;
}

/*!
 * clear all interrupts and release the IRQ
 */
static void mxc_rtc_release(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtc_plat_data *pdata = platform_get_drvdata(pdev);
	void __iomem *ioaddr = pdata->ioaddr;

	spin_lock_irq(&rtc_lock);
	writew(0, ioaddr + RTC_RTCIENR);	/* Disable all rtc interrupts */
	writew(0xFFFFFFFF, ioaddr + RTC_RTCISR);	/* Clear all interrupt status */
	spin_unlock_irq(&rtc_lock);
	rtc_status = 0;
}

/*!
 * This function is used to support some ioctl calls directly.
 * Other ioctl calls are supported indirectly through the
 * arm/common/rtctime.c file.
 *
 * @param  cmd          ioctl command as defined in include/linux/rtc.h
 * @param  arg          value for the ioctl command
 *
 * @return  0 if successful or negative value otherwise.
 */
static int mxc_rtc_ioctl(struct device *dev, unsigned int cmd,
			 unsigned long arg)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtc_plat_data *pdata = platform_get_drvdata(pdev);
	void __iomem *ioaddr = pdata->ioaddr;
	int i;
	switch (cmd) {
	case RTC_PIE_OFF:
		writew((readw(ioaddr + RTC_RTCIENR) & ~PIT_ALL_ON),
		       ioaddr + RTC_RTCIENR);
		return 0;
	case RTC_IRQP_SET:
		if (arg < 2 || arg > MAX_PIE_FREQ || (arg % 2) != 0)
			return -EINVAL;	/* Also make sure a power of 2Hz */
		if ((arg > 64) && (!capable(CAP_SYS_RESOURCE)))
			return -EACCES;
		rtc_freq = arg;
		return 0;
	case RTC_IRQP_READ:
		return put_user(rtc_freq, (u32 *) arg);
	case RTC_PIE_ON:
		for (i = 0; i < MAX_PIE_NUM; i++) {
			if (PIE_BIT_DEF[i][0] == rtc_freq) {
				break;
			}
		}
		if (i == MAX_PIE_NUM) {
			return -EACCES;
		}
		spin_lock_irq(&rtc_lock);
		writew((readw(ioaddr + RTC_RTCIENR) | PIE_BIT_DEF[i][1]),
		       ioaddr + RTC_RTCIENR);
		spin_unlock_irq(&rtc_lock);
		return 0;
	case RTC_AIE_OFF:
		spin_lock_irq(&rtc_lock);
		writew((readw(ioaddr + RTC_RTCIENR) & ~RTC_ALM_BIT),
		       ioaddr + RTC_RTCIENR);
		spin_unlock_irq(&rtc_lock);
		return 0;

	case RTC_AIE_ON:
		spin_lock_irq(&rtc_lock);
		writew((readw(ioaddr + RTC_RTCIENR) | RTC_ALM_BIT),
		       ioaddr + RTC_RTCIENR);
		spin_unlock_irq(&rtc_lock);
		return 0;

	case RTC_UIE_OFF:	/* UIE is for the 1Hz interrupt */
		spin_lock_irq(&rtc_lock);
		writew((readw(ioaddr + RTC_RTCIENR) & ~RTC_1HZ_BIT),
		       ioaddr + RTC_RTCIENR);
		spin_unlock_irq(&rtc_lock);
		return 0;

	case RTC_UIE_ON:
		spin_lock_irq(&rtc_lock);
		writew((readw(ioaddr + RTC_RTCIENR) | RTC_1HZ_BIT),
		       ioaddr + RTC_RTCIENR);
		spin_unlock_irq(&rtc_lock);
		return 0;
	}
	return -ENOIOCTLCMD;
}

/*!
 * This function reads the current RTC time into tm in Gregorian date.
 *
 * @param  tm           contains the RTC time value upon return
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int mxc_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	u32 val;

	/* Avoid roll-over from reading the different registers */
	do {
		val = get_alarm_or_time(dev, MXC_RTC_TIME);
	} while (val != get_alarm_or_time(dev, MXC_RTC_TIME));

	rtc_time_to_tm(val, tm);
	return 0;
}

/*!
 * This function sets the internal RTC time based on tm in Gregorian date.
 *
 * @param  tm           the time value to be set in the RTC
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int mxc_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long time;
	int ret;
	ret = rtc_tm_to_time(tm, &time);
	if (ret != 0) {
		return ret;
	}

	/* Avoid roll-over from reading the different registers */
	do {
		set_alarm_or_time(dev, MXC_RTC_TIME, time);
	} while (time != get_alarm_or_time(dev, MXC_RTC_TIME));

	ret = set_ext_rtc_time(time);

	if (ret != MXC_EXTERNAL_RTC_OK) {
		if (ret == MXC_EXTERNAL_RTC_NONE) {
			pr_info("No external RTC\n");
			ret = 0;
		} else
			pr_info("Failed to set external RTC\n");
	}

	return ret;
}

/*!
 * This function reads the current alarm value into the passed in \b alrm
 * argument. It updates the \b alrm's pending field value based on the whether
 * an alarm interrupt occurs or not.
 *
 * @param  alrm         contains the RTC alarm value upon return
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int mxc_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtc_plat_data *pdata = platform_get_drvdata(pdev);
	void __iomem *ioaddr = pdata->ioaddr;

	rtc_time_to_tm(get_alarm_or_time(dev, MXC_RTC_ALARM), &alrm->time);
	alrm->pending =
	    ((readw(ioaddr + RTC_RTCISR) & RTC_ALM_BIT) != 0) ? 1 : 0;

	return 0;
}

/*!
 * This function sets the RTC alarm based on passed in alrm.
 *
 * @param  alrm         the alarm value to be set in the RTC
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int mxc_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtc_plat_data *pdata = platform_get_drvdata(pdev);
	void __iomem *ioaddr = pdata->ioaddr;
	int ret;

	spin_lock_irq(&rtc_lock);
	if (rtc_valid_tm(&alrm->time)) {
		if (alrm->time.tm_sec > 59 ||
		    alrm->time.tm_hour > 23 || alrm->time.tm_min > 59) {
			ret = -EINVAL;
			goto out;
		}
		ret = rtc_update_alarm(dev, &alrm->time);
	} else {
		if ((ret = rtc_valid_tm(&alrm->time)))
			goto out;
		ret = rtc_update_alarm(dev, &alrm->time);
	}

	if (ret == 0) {
		memcpy(&g_rtc_alarm, &alrm->time, sizeof(struct rtc_time));

		if (alrm->enabled) {
			writew((readw(ioaddr + RTC_RTCIENR) | RTC_ALM_BIT),
			       ioaddr + RTC_RTCIENR);
		} else {
			writew((readw(ioaddr + RTC_RTCIENR) & ~RTC_ALM_BIT),
			       ioaddr + RTC_RTCIENR);
		}
	}
      out:
	spin_unlock_irq(&rtc_lock);

	return ret;
}

/*!
 * This function is used to provide the content for the /proc/driver/rtc
 * file.
 *
 * @param  buf          the buffer to hold the information that the driver wants to write
 *
 * @return  The number of bytes written into the rtc file.
 */
static int mxc_rtc_proc(struct device *dev, struct seq_file *sq)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtc_plat_data *pdata = platform_get_drvdata(pdev);
	void __iomem *ioaddr = pdata->ioaddr;
	char *p = sq->buf;

	p += sprintf(p, "alarm_IRQ\t: %s\n",
		     (((readw(ioaddr + RTC_RTCIENR)) & RTC_ALM_BIT) !=
		      0) ? "yes" : "no");
	p += sprintf(p, "update_IRQ\t: %s\n",
		     (((readw(ioaddr + RTC_RTCIENR)) & RTC_1HZ_BIT) !=
		      0) ? "yes" : "no");
	p += sprintf(p, "periodic_IRQ\t: %s\n",
		     (((readw(ioaddr + RTC_RTCIENR)) & PIT_ALL_ON) !=
		      0) ? "yes" : "no");
	p += sprintf(p, "periodic_freq\t: %d\n", rtc_freq);

	return p - (sq->buf);
}

/*!
 * The RTC driver structure
 */
static struct rtc_class_ops mxc_rtc_ops = {
	.open = mxc_rtc_open,
	.release = mxc_rtc_release,
	.ioctl = mxc_rtc_ioctl,
	.read_time = mxc_rtc_read_time,
	.set_time = mxc_rtc_set_time,
	.read_alarm = mxc_rtc_read_alarm,
	.set_alarm = mxc_rtc_set_alarm,
	.proc = mxc_rtc_proc,
};

/*! MXC RTC Power management control */

static struct timespec mxc_rtc_delta;

static int mxc_rtc_probe(struct platform_device *pdev)
{
	struct clk *clk;
	struct timespec tv;
	struct resource *res;
	struct rtc_device *rtc;
	struct rtc_plat_data *pdata = NULL;
	u32 reg;
	int ret = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->clk = clk_get(&pdev->dev, "rtc_clk");
	clk_enable(pdata->clk);

	pdata->baseaddr = res->start;
	pdata->ioaddr = ((void *)(IO_ADDRESS(pdata->baseaddr)));
	/* Configure and enable the RTC */
	pdata->irq = platform_get_irq(pdev, 0);
	if (pdata->irq >= 0) {
		if (request_irq(pdata->irq, mxc_rtc_interrupt, IRQF_SHARED,
				pdev->name, pdev) < 0) {
			dev_warn(&pdev->dev, "interrupt not available.\n");
			pdata->irq = -1;
		}
	}
	rtc =
	    rtc_device_register(pdev->name, &pdev->dev, &mxc_rtc_ops,
				THIS_MODULE);
	if (IS_ERR(rtc)) {
		ret = PTR_ERR(rtc);
		if (pdata->irq >= 0)
			free_irq(pdata->irq, pdev);
		kfree(pdata);
		return ret;
	}
	pdata->rtc = rtc;
	platform_set_drvdata(pdev, pdata);
	tv.tv_nsec = 0;
	tv.tv_sec = get_alarm_or_time(&pdev->dev, MXC_RTC_TIME);
	clk = clk_get(NULL, "ckil");
	if (clk_get_rate(clk) == 32768)
		reg = RTC_INPUT_CLK_32768HZ;
	else if (clk_get_rate(clk) == 32000)
		reg = RTC_INPUT_CLK_32000HZ;
	else if (clk_get_rate(clk) == 38400)
		reg = RTC_INPUT_CLK_38400HZ;
	else {
		printk(KERN_ALERT "rtc clock is not valid");
		return -EINVAL;
	}
	clk_put(clk);
	reg |= RTC_ENABLE_BIT;
	writew(reg, (pdata->ioaddr + RTC_RTCCTL));
	if (((readw(pdata->ioaddr + RTC_RTCCTL)) & RTC_ENABLE_BIT) == 0) {
		printk(KERN_ALERT "rtc : hardware module can't be enabled!\n");
		return -EPERM;
	}
	printk("Real TIme clock Driver v%s \n", RTC_VERSION);
	return ret;
}

static int __exit mxc_rtc_remove(struct platform_device *pdev)
{
	struct rtc_plat_data *pdata = platform_get_drvdata(pdev);
	rtc_device_unregister(pdata->rtc);
	if (pdata->irq >= 0) {
		free_irq(pdata->irq, pdev);
	}
	clk_disable(pdata->clk);
	clk_put(pdata->clk);
	kfree(pdata);
	mxc_rtc_release(NULL);
	return 0;
}

/*!
 * This function is called to save the system time delta relative to
 * the MXC RTC when enterring a low power state. This time delta is
 * then used on resume to adjust the system time to account for time
 * loss while suspended.
 *
 * @param   pdev  not used
 * @param   state Power state to enter.
 *
 * @return  The function always returns 0.
 */
static int mxc_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct timespec tv;

	/* calculate time delta for suspend */
	/* RTC precision is 1 second; adjust delta for avg 1/2 sec err */
	tv.tv_nsec = NSEC_PER_SEC >> 1;
	tv.tv_sec = get_alarm_or_time(&pdev->dev, MXC_RTC_TIME);
	set_normalized_timespec(&mxc_rtc_delta,
				xtime.tv_sec - tv.tv_sec,
				xtime.tv_nsec - tv.tv_nsec);

	return 0;
}

/*!
 * This function is called to correct the system time based on the
 * current MXC RTC time relative to the time delta saved during
 * suspend.
 *
 * @param   pdev  not used
 *
 * @return  The function always returns 0.
 */
static int mxc_rtc_resume(struct platform_device *pdev)
{
	struct timespec tv;
	struct timespec ts;

	tv.tv_nsec = 0;
	tv.tv_sec = get_alarm_or_time(&pdev->dev, MXC_RTC_TIME);

	/* restore wall clock using delta against this RTC;
	 * adjust again for avg 1/2 second RTC sampling error
	 */
	set_normalized_timespec(&ts,
				tv.tv_sec + mxc_rtc_delta.tv_sec,
				(NSEC_PER_SEC >> 1) + mxc_rtc_delta.tv_nsec);
	do_settimeofday(&ts);

	return 0;
}

/*!
 * Contains pointers to the power management callback functions.
 */
static struct platform_driver mxc_rtc_driver = {
	.driver = {
		   .name = "mxc_rtc",
		   },
	.probe = mxc_rtc_probe,
	.remove = __exit_p(mxc_rtc_remove),
	.suspend = mxc_rtc_suspend,
	.resume = mxc_rtc_resume,
};

/*!
 * This function creates the /proc/driver/rtc file and registers the device RTC
 * in the /dev/misc directory. It also reads the RTC value from external source
 * and setup the internal RTC properly.
 *
 * @return  -1 if RTC is failed to initialize; 0 is successful.
 */
static int __init mxc_rtc_init(void)
{
	return platform_driver_register(&mxc_rtc_driver);
}

/*!
 * This function removes the /proc/driver/rtc file and un-registers the
 * device RTC from the /dev/misc directory.
 */
static void __exit mxc_rtc_exit(void)
{
	platform_driver_unregister(&mxc_rtc_driver);

}

device_initcall_sync(mxc_rtc_init);
module_exit(mxc_rtc_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Realtime Clock Driver (RTC)");
MODULE_LICENSE("GPL");
