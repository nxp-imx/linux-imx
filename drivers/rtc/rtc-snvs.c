/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc.
 */

/*
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
/*
 * Implementation based on rtc-ds1553.c
 */

/*!
 * @defgroup RTC Real Time Clock (RTC) Driver
 */
/*!
 * @file rtc-snvs.c
 * @brief Secure Real Time Clock (SRTC) interface in Freescale's SNVS module
 *
 * This file contains Real Time Clock interface for Linux. The Freescale
 * SNVS module's Low Power (LP) SRTC functionality is utilized in this driver,
 * in non-secure mode.
 *
 * @ingroup RTC
 */

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/rtc.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/sched.h>

/* Register definitions */
#define	SNVS_HPSR	0x14	/* HP Status Register */
#define	SNVS_LPCR	0x38	/* LP Control Register */
#define	SNVS_LPSR	0x4C	/* LP Status Register */
#define	SNVS_LPSRTCMR	0x50	/* LP Secure Real Time Counter MSB Register */
#define	SNVS_LPSRTCLR	0x54	/* LP Secure Real Time Counter LSB Register */
#define	SNVS_LPTAR	0x58	/* LP Time Alarm Register */
#define	SNVS_LPSMCMR	0x5C	/* LP Secure Monotonic Counter MSB Register */
#define	SNVS_LPSMCLR	0x60	/* LP Secure Monotonic Counter LSB Register */
#define	SNVS_LPPGDR	0x64	/* LP Power Glitch Detector Register */
#define	SNVS_LPGPR	0x68	/* LP General Purpose Register */

/* Bit Definitions */
#define	SNVS_HPSR_SSM_ST_MASK	0x00000F00
#define	SNVS_HPSR_SSM_ST_SHIFT	8

#define	SNVS_LPCR_SRTC_ENV	(1 << 0)
#define	SNVS_LPCR_LPTA_EN		(1 << 1)
#define	SNVS_LPCR_LPWUI_EN	(1 << 3)

#define	SNVS_LPCR_ALL_INT_EN (SNVS_LPCR_LPTA_EN | SNVS_LPCR_LPWUI_EN)

#define	SNVS_LPSR_LPTA		(1 << 0)

#define	SNVS_LPPGDR_INIT	0x41736166

/* Other defines */
#define	SSM_ST_CHECK	0x9
#define	SSM_ST_NON_SECURE	0xB
#define	CNTR_TO_SECS_SH	15	/* Converts 47-bit counter to 32-bit seconds */

#define RTC_READ_TIME_47BIT	_IOR('p', 0x20, unsigned long long)
/* blocks until LPSCMR is set, returns difference */
#define RTC_WAIT_TIME_SET	_IOR('p', 0x21, int64_t)

struct rtc_drv_data {
	struct rtc_device *rtc;
	void __iomem *ioaddr;
	unsigned long baseaddr;
	int irq;
	bool irq_enable;
};

static unsigned long rtc_status;

static DEFINE_SPINLOCK(rtc_lock);
DECLARE_COMPLETION(snvs_completion);
static int64_t time_diff;

/*!
 * LP counter register reads should always use this function.
 * This function reads 2 consective times from LP counter register
 * until the 2 values match. This is to avoid reading corrupt
 * value if the counter is in the middle of updating (TKT052983)
 */
static inline u32 rtc_read_lp_counter(void __iomem *counter_reg)
{
	u64 read1, read2;
	u32 counter_sec;

	do {
		/* MSB */
		read1 = __raw_readl(counter_reg);
		read1 <<= 32;
		/* LSB */
		read1 |= __raw_readl(counter_reg + 4);

		/* MSB */
		read2 = __raw_readl(counter_reg);
		read2 <<= 32;
		/* LSB */
		read2 |= __raw_readl(counter_reg + 4);
	} while (read1 != read2);

	/* Convert 47-bit counter to 32-bit raw second count */
	counter_sec = (u32) (read1 >> CNTR_TO_SECS_SH);
	return counter_sec;
}

/*!
 * This function does write synchronization for writes to the lp srtc block.
 * To take care of the asynchronous CKIL clock, all writes from the IP domain
 * will be synchronized to the CKIL domain.
 */
static inline void rtc_write_sync_lp(void __iomem *ioaddr)
{
	unsigned int i, count1, count2, count3;

	/* Wait for 3 CKIL cycles */
	for (i = 0; i < 3; i++) {

		/* TKT052983: Do consective reads of LSB of counter
		 * to ensure integrity
		 */
		do {
			count1 = __raw_readl(ioaddr + SNVS_LPSRTCLR);
			count2 = __raw_readl(ioaddr + SNVS_LPSRTCLR);
		} while (count1 != count2);

		/* Now wait until counter value changes */
		do {
			do {
				count2 = __raw_readl(ioaddr + SNVS_LPSRTCLR);
				count3 = __raw_readl(ioaddr + SNVS_LPSRTCLR);
			} while (count2 != count3);
		} while (count3 == count1);
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
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	void __iomem *ioaddr = pdata->ioaddr;
	struct rtc_time alarm_tm, now_tm;
	unsigned long now, time, lp_cr;
	int ret;

	now = rtc_read_lp_counter(ioaddr + SNVS_LPSRTCMR);
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

	/* Have to clear LPTA_EN before programming new alarm time in LPTAR */
	lp_cr = __raw_readl(ioaddr + SNVS_LPCR);
	__raw_writel(lp_cr & ~SNVS_LPCR_LPTA_EN, ioaddr + SNVS_LPCR);
	rtc_write_sync_lp(ioaddr);

	__raw_writel(time, ioaddr + SNVS_LPTAR);

	/* clear alarm interrupt status bit */
	__raw_writel(SNVS_LPSR_LPTA, ioaddr + SNVS_LPSR);

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
static irqreturn_t snvs_rtc_interrupt(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct rtc_drv_data *pdata = platform_get_drvdata(pdev);
	void __iomem *ioaddr = pdata->ioaddr;
	u32 lp_status, lp_cr;
	u32 events = 0;

	lp_status = __raw_readl(ioaddr + SNVS_LPSR);
	lp_cr = __raw_readl(ioaddr + SNVS_LPCR);

	/* update irq data & counter */
	if (lp_status & SNVS_LPSR_LPTA) {
		if (lp_cr & SNVS_LPCR_LPTA_EN)
			events |= (RTC_AF | RTC_IRQF);

		/* disable further lp alarm interrupts */
		lp_cr &= ~(SNVS_LPCR_LPTA_EN | SNVS_LPCR_LPWUI_EN);
	}

	/* Update interrupt enables */
	__raw_writel(lp_cr, ioaddr + SNVS_LPCR);

	/* If no interrupts are enabled, turn off interrupts in kernel */
	if (((lp_cr & SNVS_LPCR_ALL_INT_EN) == 0) && (pdata->irq_enable)) {
		disable_irq_nosync(pdata->irq);
		pdata->irq_enable = false;
	}

	/* clear interrupt status */
	__raw_writel(lp_status, ioaddr + SNVS_LPSR);

	rtc_write_sync_lp(ioaddr);
	rtc_update_irq(pdata->rtc, 1, events);
	return IRQ_HANDLED;
}

/*!
 * This function is used to open the RTC driver.
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int snvs_rtc_open(struct device *dev)
{
	if (test_and_set_bit(1, &rtc_status))
		return -EBUSY;
	return 0;
}

/*!
 * clear all interrupts and release the IRQ
 */
static void snvs_rtc_release(struct device *dev)
{
	rtc_status = 0;
}

/*!
 * This function reads the current RTC time into tm in Gregorian date.
 *
 * @param  tm           contains the RTC time value upon return
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int snvs_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	void __iomem *ioaddr = pdata->ioaddr;

	rtc_time_to_tm(rtc_read_lp_counter(ioaddr + SNVS_LPSRTCMR), tm);
	return 0;
}

/*!
 * This function sets the internal RTC time based on tm in Gregorian date.
 *
 * @param  tm           the time value to be set in the RTC
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int snvs_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	void __iomem *ioaddr = pdata->ioaddr;
	unsigned long time;
	int ret;
	u32 lp_cr;
	u64 old_time_47bit, new_time_47bit;

	ret = rtc_tm_to_time(tm, &time);
	if (ret != 0)
		return ret;

	old_time_47bit = (((u64) (__raw_readl(ioaddr + SNVS_LPSRTCMR) &
		((0x1 << CNTR_TO_SECS_SH) - 1)) << 32) |
		((u64) __raw_readl(ioaddr + SNVS_LPSRTCLR)));

	/* Disable RTC first */
	lp_cr = __raw_readl(ioaddr + SNVS_LPCR) & ~SNVS_LPCR_SRTC_ENV;
	__raw_writel(lp_cr, ioaddr + SNVS_LPCR);
	while (__raw_readl(ioaddr + SNVS_LPCR) & SNVS_LPCR_SRTC_ENV)
		;

	/* Write 32-bit time to 47-bit timer, leaving 15 LSBs blank */
	__raw_writel(time << CNTR_TO_SECS_SH, ioaddr + SNVS_LPSRTCLR);
	__raw_writel(time >> (32 - CNTR_TO_SECS_SH), ioaddr + SNVS_LPSRTCMR);

	/* Enable RTC again */
	__raw_writel(lp_cr | SNVS_LPCR_SRTC_ENV, ioaddr + SNVS_LPCR);
	while (!(__raw_readl(ioaddr + SNVS_LPCR) & SNVS_LPCR_SRTC_ENV))
		;

	rtc_write_sync_lp(ioaddr);

	new_time_47bit = (((u64) (__raw_readl(ioaddr + SNVS_LPSRTCMR) &
		((0x1 << CNTR_TO_SECS_SH) - 1)) << 32) |
		((u64) __raw_readl(ioaddr + SNVS_LPSRTCLR)));

	time_diff = new_time_47bit - old_time_47bit;

	/* signal all waiting threads that time changed */
	complete_all(&snvs_completion);

	/* allow signalled threads to handle the time change notification */
	schedule();

	/* reinitialize completion variable */
	INIT_COMPLETION(snvs_completion);
	return 0;
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
static int snvs_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	void __iomem *ioaddr = pdata->ioaddr;

	rtc_time_to_tm(__raw_readl(ioaddr + SNVS_LPTAR), &alrm->time);
	alrm->pending =
	    ((__raw_readl(ioaddr + SNVS_LPSR) & SNVS_LPSR_LPTA) != 0) ? 1 : 0;

	return 0;
}

/*!
 * This function sets the RTC alarm based on passed in alrm.
 *
 * @param  alrm         the alarm value to be set in the RTC
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int snvs_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	void __iomem *ioaddr = pdata->ioaddr;
	unsigned long lock_flags = 0;
	u32 lp_cr;
	int ret;

	if (rtc_valid_tm(&alrm->time)) {
		if (alrm->time.tm_sec > 59 ||
		    alrm->time.tm_hour > 23 || alrm->time.tm_min > 59) {
			return -EINVAL;
		}
	}

	spin_lock_irqsave(&rtc_lock, lock_flags);

	ret = rtc_update_alarm(dev, &alrm->time);
	if (ret)
		goto out;

	lp_cr = __raw_readl(ioaddr + SNVS_LPCR);

	if (alrm->enabled)
		lp_cr |= (SNVS_LPCR_LPTA_EN | SNVS_LPCR_LPWUI_EN);
	else
		lp_cr &= ~(SNVS_LPCR_LPTA_EN | SNVS_LPCR_LPWUI_EN);

	if (lp_cr & SNVS_LPCR_ALL_INT_EN) {
		if (!pdata->irq_enable) {
			enable_irq(pdata->irq);
			pdata->irq_enable = true;
		}
	} else {
		if (pdata->irq_enable) {
			disable_irq(pdata->irq);
			pdata->irq_enable = false;
		}
	}

	__raw_writel(lp_cr, ioaddr + SNVS_LPCR);

out:
	rtc_write_sync_lp(ioaddr);
	spin_unlock_irqrestore(&rtc_lock, lock_flags);
	return ret;
}

/*!
 * This function is used to provide the content for the /proc/driver/rtc
 * file.
 *
 * @param  seq  buffer to hold the information that the driver wants to write
 *
 * @return  The number of bytes written into the rtc file.
 */
static int snvs_rtc_proc(struct device *dev, struct seq_file *seq)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	void __iomem *ioaddr = pdata->ioaddr;

	seq_printf(seq, "alarm_IRQ\t: %s\n",
		   (((__raw_readl(ioaddr + SNVS_LPCR)) & SNVS_LPCR_LPTA_EN) !=
		    0) ? "yes" : "no");

	return 0;
}

static int snvs_rtc_alarm_irq_enable(struct device *dev, unsigned int enable)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	void __iomem *ioaddr = pdata->ioaddr;
	u32 lp_cr;
	unsigned long lock_flags = 0;

	spin_lock_irqsave(&rtc_lock, lock_flags);

	if (enable) {
		if (!pdata->irq_enable) {
			enable_irq(pdata->irq);
			pdata->irq_enable = true;
		}
		lp_cr = __raw_readl(ioaddr + SNVS_LPCR);
		lp_cr |= (SNVS_LPCR_LPTA_EN | SNVS_LPCR_LPWUI_EN);
		__raw_writel(lp_cr, ioaddr + SNVS_LPCR);
	} else {
		lp_cr = __raw_readl(ioaddr + SNVS_LPCR);
		lp_cr &= ~(SNVS_LPCR_LPTA_EN | SNVS_LPCR_LPWUI_EN);
		if (((lp_cr & SNVS_LPCR_ALL_INT_EN) == 0)
		    && (pdata->irq_enable)) {
			disable_irq(pdata->irq);
			pdata->irq_enable = false;
		}
		__raw_writel(lp_cr, ioaddr + SNVS_LPCR);
	}

	rtc_write_sync_lp(ioaddr);
	spin_unlock_irqrestore(&rtc_lock, lock_flags);
	return 0;
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
static int snvs_rtc_ioctl(struct device *dev, unsigned int cmd,
			 unsigned long arg)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	void __iomem *ioaddr = pdata->ioaddr;
	u64 time_47bit;
	int retVal;

	switch (cmd) {
	case RTC_READ_TIME_47BIT:
		time_47bit = (((u64) (__raw_readl(ioaddr + SNVS_LPSRTCMR) &
			((0x1 << CNTR_TO_SECS_SH) - 1)) << 32) |
			((u64) __raw_readl(ioaddr + SNVS_LPSRTCLR)));

		if (arg && copy_to_user((u64 *) arg, &time_47bit, sizeof(u64)))
			return -EFAULT;

		return 0;

	/* This IOCTL to be used by processes to be notified of time changes */
	case RTC_WAIT_TIME_SET:
		/* don't block without releasing mutex first */
		mutex_unlock(&pdata->rtc->ops_lock);

		/* sleep until awakened by SRTC driver when LPSCMR is changed */
		wait_for_completion(&snvs_completion);

		/* relock mutex because rtc_dev_ioctl will unlock again */
		retVal = mutex_lock_interruptible(&pdata->rtc->ops_lock);

		/* copy the new time difference = new time - previous time
		  * to the user param. The difference is a signed value */
		if (arg && copy_to_user((int64_t *) arg, &time_diff,
			sizeof(int64_t)))
			return -EFAULT;

		return retVal;

	}

	return -ENOIOCTLCMD;
}
/*!
 * The RTC driver structure
 */
static struct rtc_class_ops snvs_rtc_ops = {
	.open = snvs_rtc_open,
	.release = snvs_rtc_release,
	.read_time = snvs_rtc_read_time,
	.set_time = snvs_rtc_set_time,
	.read_alarm = snvs_rtc_read_alarm,
	.set_alarm = snvs_rtc_set_alarm,
	.proc = snvs_rtc_proc,
	.ioctl = snvs_rtc_ioctl,
	.alarm_irq_enable = snvs_rtc_alarm_irq_enable,
};

/*! SNVS RTC Power management control */
static int snvs_rtc_probe(struct platform_device *pdev)
{
	struct timespec tv;
	struct resource *res;
	struct rtc_device *rtc;
	struct rtc_drv_data *pdata = NULL;
	void __iomem *ioaddr;
	u32 lp_cr;
	int ret = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->baseaddr = res->start;
	pdata->ioaddr = ioremap(pdata->baseaddr, 0xC00);
	ioaddr = pdata->ioaddr;
	pdata->irq = platform_get_irq(pdev, 0);
	platform_set_drvdata(pdev, pdata);


	/* Added to support sysfs wakealarm attribute */
	pdev->dev.power.can_wakeup = 1;

	/* initialize glitch detect */
	__raw_writel(SNVS_LPPGDR_INIT, ioaddr + SNVS_LPPGDR);
	udelay(100);

	/* clear lp interrupt status */
	__raw_writel(0xFFFFFFFF, ioaddr + SNVS_LPSR);

	/* Enable RTC */
	lp_cr = __raw_readl(ioaddr + SNVS_LPCR);
	if ((lp_cr & SNVS_LPCR_SRTC_ENV) == 0)
		__raw_writel(lp_cr | SNVS_LPCR_SRTC_ENV, ioaddr + SNVS_LPCR);

	udelay(100);

	__raw_writel(0xFFFFFFFF, ioaddr + SNVS_LPSR);
	udelay(100);

	if (pdata->irq >= 0) {
		if (request_irq(pdata->irq, snvs_rtc_interrupt, IRQF_SHARED,
				pdev->name, pdev) < 0) {
			dev_warn(&pdev->dev, "interrupt not available.\n");
			pdata->irq = -1;
		} else {
			disable_irq(pdata->irq);
			pdata->irq_enable = false;
		}
	}

	rtc = rtc_device_register(pdev->name, &pdev->dev,
				  &snvs_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc)) {
		ret = PTR_ERR(rtc);
		goto err_out;
	}

	pdata->rtc = rtc;

	tv.tv_nsec = 0;
	tv.tv_sec = rtc_read_lp_counter(ioaddr + SNVS_LPSRTCMR);

	/* Remove can_wakeup flag to add common power wakeup interface */
	pdev->dev.power.can_wakeup = 0;

	/* By default, devices should wakeup if they can */
	/* So snvs is set as "should wakeup" as it can */
	device_init_wakeup(&pdev->dev, 1);

	return ret;

err_out:
	iounmap(ioaddr);
	if (pdata->irq >= 0)
		free_irq(pdata->irq, pdev);
	kfree(pdata);
	return ret;
}

static int __exit snvs_rtc_remove(struct platform_device *pdev)
{
	struct rtc_drv_data *pdata = platform_get_drvdata(pdev);
	rtc_device_unregister(pdata->rtc);
	if (pdata->irq >= 0)
		free_irq(pdata->irq, pdev);

	kfree(pdata);
	return 0;
}

/*!
 * This function is called to save the system time delta relative to
 * the SNVS RTC when enterring a low power state. This time delta is
 * then used on resume to adjust the system time to account for time
 * loss while suspended.
 *
 * @param   pdev  not used
 * @param   state Power state to enter.
 *
 * @return  The function always returns 0.
 */
static int snvs_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct rtc_drv_data *pdata = platform_get_drvdata(pdev);

	if (device_may_wakeup(&pdev->dev)) {
		enable_irq_wake(pdata->irq);
	} else {
		if (pdata->irq_enable)
			disable_irq(pdata->irq);
	}

	return 0;
}

/*!
 * This function is called to correct the system time based on the
 * current SNVS RTC time relative to the time delta saved during
 * suspend.
 *
 * @param   pdev  not used
 *
 * @return  The function always returns 0.
 */
static int snvs_rtc_resume(struct platform_device *pdev)
{
	struct rtc_drv_data *pdata = platform_get_drvdata(pdev);

	if (device_may_wakeup(&pdev->dev)) {
		disable_irq_wake(pdata->irq);
	} else {
		if (pdata->irq_enable)
			enable_irq(pdata->irq);
	}

	return 0;
}

/*!
 * Contains pointers to the power management callback functions.
 */
static struct platform_driver snvs_rtc_driver = {
	.driver = {
		   .name = "snvs_rtc",
		   },
	.probe = snvs_rtc_probe,
	.remove = __exit_p(snvs_rtc_remove),
	.suspend = snvs_rtc_suspend,
	.resume = snvs_rtc_resume,
};

/*!
 * This function creates the /proc/driver/rtc file and registers the device RTC
 * in the /dev/misc directory. It also reads the RTC value from external source
 * and setup the internal RTC properly.
 *
 * @return  -1 if RTC is failed to initialize; 0 is successful.
 */
static int __init snvs_rtc_init(void)
{
	return platform_driver_register(&snvs_rtc_driver);
}

/*!
 * This function removes the /proc/driver/rtc file and un-registers the
 * device RTC from the /dev/misc directory.
 */
static void __exit snvs_rtc_exit(void)
{
	platform_driver_unregister(&snvs_rtc_driver);

}

module_init(snvs_rtc_init);
module_exit(snvs_rtc_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("SNVS Realtime Clock Driver (RTC)");
MODULE_LICENSE("GPL");
