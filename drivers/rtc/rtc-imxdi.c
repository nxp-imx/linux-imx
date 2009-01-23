/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/* based on rtc-mc13892.c */

/*
 * This driver uses the 47-bit 32 kHz counter in the Freescale DryIce block
 * to implement a Linux RTC. Times and alarms are truncated to seconds.
 * Since the RTC framework performs API locking via rtc->ops_lock the
 * only simultaneous accesses we need to deal with is updating DryIce
 * registers while servicing an alarm.
 *
 * Note that reading the DSR (DryIce Status Register) automatically clears
 * the WCF (Write Complete Flag). All DryIce writes are synchronized to the
 * LP (Low Power) domain and set the WCF upon completion. Writes to the
 * DIER (DryIce Interrupt Enable Register) are the only exception. These
 * occur at normal bus speeds and do not set WCF.  Periodic interrupts are
 * not supported by the hardware.
 */

/* #define DEBUG */
/* #define DI_DEBUG_REGIO */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/workqueue.h>

/* DryIce Register Definitions */

#define DTCMR     0x00           /* Time Counter MSB Reg */
#define DTCLR     0x04           /* Time Counter LSB Reg */

#define DCAMR     0x08           /* Clock Alarm MSB Reg */
#define DCALR     0x0c           /* Clock Alarm LSB Reg */
#define DCAMR_UNSET  0xFFFFFFFF  /* doomsday - 1 sec */

#define DCR       0x10           /* Control Reg */
#define DCR_TCE   (1 << 3)       /* Time Counter Enable */

#define DSR       0x14           /* Status Reg */
#define DSR_WBF   (1 << 10)      /* Write Busy Flag */
#define DSR_WNF   (1 << 9)       /* Write Next Flag */
#define DSR_WCF   (1 << 8)       /* Write Complete Flag */
#define DSR_WEF   (1 << 7)       /* Write Error Flag */
#define DSR_CAF   (1 << 4)       /* Clock Alarm Flag */
#define DSR_NVF   (1 << 1)       /* Non-Valid Flag */
#define DSR_SVF   (1 << 0)       /* Security Violation Flag */

#define DIER      0x18           /* Interrupt Enable Reg */
#define DIER_WNIE (1 << 9)       /* Write Next Interrupt Enable */
#define DIER_WCIE (1 << 8)       /* Write Complete Interrupt Enable */
#define DIER_WEIE (1 << 7)       /* Write Error Interrupt Enable */
#define DIER_CAIE (1 << 4)       /* Clock Alarm Interrupt Enable */

#ifndef DI_DEBUG_REGIO
/* dryice read register */
#define di_read(pdata, reg)  __raw_readl((pdata)->ioaddr + (reg))

/* dryice write register */
#define di_write(pdata, val, reg)  __raw_writel((val), (pdata)->ioaddr + (reg))
#else
/* dryice read register - debug version */
static inline u32 di_read(struct rtc_drv_data *pdata, int reg)
{
	u32 val = __raw_readl(pdata->ioaddr + reg);
	pr_info("di_read(0x%02x) = 0x%08x\n", reg, val);
	return val;
}

/* dryice write register - debug version */
static inline void di_write(struct rtc_drv_data *pdata, u32 val, int reg)
{
	printk(KERN_INFO "di_write(0x%08x, 0x%02x)\n", val, reg);
	__raw_writel(val, pdata->ioaddr + reg);
}
#endif

/*
 * dryice write register with wait and error handling.
 * all registers, except for DIER, should use this method.
 */
#define di_write_wait_err(pdata, val, reg, rc, label) \
		do { \
			if (di_write_wait((pdata), (val), (reg))) { \
				rc = -EIO; \
				goto label; \
			} \
		} while (0)

struct rtc_drv_data {
	struct platform_device *pdev;  /* pointer to platform dev */
	struct rtc_device *rtc;        /* pointer to rtc struct */
	unsigned long baseaddr;        /* physical bass address */
	void __iomem *ioaddr;          /* virtual base address */
	int size;                      /* size of register region */
	int irq;                       /* dryice normal irq */
	struct clk *clk;               /* dryice clock control */
	u32 dsr;                       /* copy of dsr reg from isr */
	spinlock_t irq_lock;           /* irq resource lock */
	wait_queue_head_t write_wait;  /* write-complete queue */
	struct mutex write_mutex;      /* force reg writes to be sequential */
	struct work_struct work;       /* schedule alarm work */
};

/*
 * enable a dryice interrupt
 */
static inline void di_int_enable(struct rtc_drv_data *pdata, u32 intr)
{
	unsigned long flags;

	spin_lock_irqsave(&pdata->irq_lock, flags);
	di_write(pdata, di_read(pdata, DIER) | intr, DIER);
	spin_unlock_irqrestore(&pdata->irq_lock, flags);
}

/*
 * disable a dryice interrupt
 */
static inline void di_int_disable(struct rtc_drv_data *pdata, u32 intr)
{
	unsigned long flags;

	spin_lock_irqsave(&pdata->irq_lock, flags);
	di_write(pdata, di_read(pdata, DIER) & ~intr, DIER);
	spin_unlock_irqrestore(&pdata->irq_lock, flags);
}

/*
 * This function attempts to clear the dryice write-error flag.
 *
 * A dryice write error is similar to a bus fault and should not occur in
 * normal operation.  Clearing the flag requires another write, so the root
 * cause of the problem may need to be fixed before the flag can be cleared.
 */
static void clear_write_error(struct rtc_drv_data *pdata)
{
	int cnt;

	dev_warn(&pdata->pdev->dev, "WARNING: Register write error!\n");

	for (;;) {
		/* clear the write error flag */
		di_write(pdata, DSR_WEF, DSR);

		/* wait for it to take effect */
		for (cnt = 0; cnt < 100; cnt++) {
			if ((di_read(pdata, DSR) & DSR_WEF) == 0)
				return;
			udelay(10);
		}
		dev_err(&pdata->pdev->dev,
			"ERROR: Cannot clear write-error flag!\n");
	}
}

/*
 * Write a dryice register and wait until it completes.
 *
 * This function uses interrupts to determine when the
 * write has completed.
 */
static int di_write_wait(struct rtc_drv_data *pdata, u32 val, int reg)
{
	int ret;
	int rc = 0;

	/* serialize register writes */
	mutex_lock(&pdata->write_mutex);

	/* enable the write-complete interrupt */
	di_int_enable(pdata, DIER_WCIE);

	pdata->dsr = 0;

	/* do the register write */
	di_write(pdata, val, reg);

	/* wait for the write to finish */
	ret = wait_event_interruptible_timeout(pdata->write_wait,
					       pdata->dsr & (DSR_WCF | DSR_WEF),
					       1 * HZ);
	if (ret == 0)
		dev_warn(&pdata->pdev->dev, "Write-wait timeout\n");

	/* check for write error */
	if (pdata->dsr & DSR_WEF) {
		clear_write_error(pdata);
		rc = -EIO;
	}
	mutex_unlock(&pdata->write_mutex);
	return rc;
}

/*
 * rtc device ioctl
 *
 * The rtc framework handles the basic rtc ioctls on behalf
 * of the driver by calling the functions registered in the
 * rtc_ops structure.
 */
static int dryice_rtc_ioctl(struct device *dev, unsigned int cmd,
			    unsigned long arg)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);

	dev_dbg(dev, "%s(0x%x)\n", __func__, cmd);
	switch (cmd) {
	case RTC_AIE_OFF:  /* alarm disable */
		di_int_disable(pdata, DIER_CAIE);
		return 0;

	case RTC_AIE_ON:  /* alarm enable */
		di_int_enable(pdata, DIER_CAIE);
		return 0;
	}
	return -ENOIOCTLCMD;
}

/*
 * read the seconds portion of the current time from the dryice time counter
 */
static int dryice_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	unsigned long now;

	dev_dbg(dev, "%s\n", __func__);
	now = di_read(pdata, DTCMR);
	rtc_time_to_tm(now, tm);

	return 0;
}

/*
 * set the seconds portion of dryice time counter and clear the
 * fractional part.
 */
static int dryice_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	unsigned long now;
	int rc;

	dev_dbg(dev, "%s\n", __func__);
	rc = rtc_tm_to_time(tm, &now);
	if (rc == 0) {
		/* zero the fractional part first */
		di_write_wait_err(pdata, 0, DTCLR, rc, err);
		di_write_wait_err(pdata, now, DTCMR, rc, err);
	}
err:
	return rc;
}

/*
 * read the seconds portion of the alarm register.
 * the fractional part of the alarm register is always zero.
 */
static int dryice_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	u32 dcamr;

	dev_dbg(dev, "%s\n", __func__);
	dcamr = di_read(pdata, DCAMR);
	rtc_time_to_tm(dcamr, &alarm->time);

	/* alarm is enabled if the interrupt is enabled */
	alarm->enabled = (di_read(pdata, DIER) & DIER_CAIE) != 0;

	/* don't allow the DSR read to mess up DSR_WCF */
	mutex_lock(&pdata->write_mutex);

	/* alarm is pending if the alarm flag is set */
	alarm->pending = (di_read(pdata, DSR) & DSR_CAF) != 0;

	mutex_unlock(&pdata->write_mutex);

	return 0;
}

/*
 * set the seconds portion of dryice alarm register
 */
static int dryice_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	unsigned long now;
	unsigned long alarm_time;
	int rc;

	dev_dbg(dev, "%s\n", __func__);
	rc = rtc_tm_to_time(&alarm->time, &alarm_time);
	if (rc)
		return rc;

	/* don't allow setting alarm in the past */
	now = di_read(pdata, DTCMR);
	if (alarm_time < now)
		return -EINVAL;

	/* write the new alarm time */
	di_write_wait_err(pdata, (u32)alarm_time, DCAMR, rc, err);

	if (alarm->enabled)
		di_int_enable(pdata, DIER_CAIE);  /* enable alarm intr */
	else
		di_int_disable(pdata, DIER_CAIE); /* disable alarm intr */
err:
	return rc;
}

static struct rtc_class_ops dryice_rtc_ops = {
	.ioctl = dryice_rtc_ioctl,
	.read_time = dryice_rtc_read_time,
	.set_time = dryice_rtc_set_time,
	.read_alarm = dryice_rtc_read_alarm,
	.set_alarm = dryice_rtc_set_alarm,
};

/*
 * dryice "normal" interrupt handler
 */
static irqreturn_t dryice_norm_irq(int irq, void *dev_id)
{
	struct rtc_drv_data *pdata = dev_id;
	u32 dsr, dier;
	irqreturn_t rc = IRQ_NONE;

	/* DSR_WCF clears itself on DSR read */
	dsr = di_read(pdata, DSR);
	dier = di_read(pdata, DIER);

	/* handle write complete and write error cases */
	if ((dier & DIER_WCIE) && (dsr & (DSR_WCF | DSR_WEF))) {
		/* mask the interrupt */
		di_int_disable(pdata, DIER_WCIE);

		/* save the dsr value for the wait queue */
		pdata->dsr |= dsr;

		wake_up_interruptible(&pdata->write_wait);
		rc = IRQ_HANDLED;
	}

	/* handle the alarm case */
	if ((dier & DIER_CAIE) && (dsr & DSR_CAF)) {
		/* mask the interrupt */
		di_int_disable(pdata, DIER_CAIE);

		/* finish alarm in user context */
		schedule_work(&pdata->work);
		rc = IRQ_HANDLED;
	}
	return rc;
}

/*
 * post the alarm event from user context so it can sleep
 * on the write completion.
 */
static void dryice_work(struct work_struct *work)
{
	struct rtc_drv_data *pdata = container_of(work, struct rtc_drv_data,
						  work);
	int rc;

	/* dismiss the interrupt (ignore error) */
	di_write_wait_err(pdata, DSR_CAF, DSR, rc, err);
err:
	/*
	 * pass the alarm event to the rtc framework. note that
	 * rtc_update_irq expects to be called with interrupts off.
	 */
	local_irq_disable();
	rtc_update_irq(pdata->rtc, 1, RTC_AF | RTC_IRQF);
	local_irq_enable();
}

/*
 * probe for dryice rtc device
 */
static int dryice_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device *rtc;
	struct resource *res;
	struct rtc_drv_data *pdata = NULL;
	void __iomem *ioaddr = NULL;
	int rc = 0;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->pdev = pdev;
	pdata->irq = -1;
	pdata->size = res->end - res->start + 1;

	if (!request_mem_region(res->start, pdata->size, pdev->name)) {
		rc = -EBUSY;
		goto err;
	}
	pdata->baseaddr = res->start;
	ioaddr = ioremap(pdata->baseaddr, pdata->size);
	if (!ioaddr) {
		rc = -ENOMEM;
		goto err;
	}
	pdata->ioaddr = ioaddr;
	pdata->irq = platform_get_irq(pdev, 0);

	init_waitqueue_head(&pdata->write_wait);

	INIT_WORK(&pdata->work, dryice_work);

	mutex_init(&pdata->write_mutex);

	pdata->clk = clk_get(NULL, "dryice_clk");
	clk_enable(pdata->clk);

	if (pdata->irq >= 0) {
		if (request_irq(pdata->irq, dryice_norm_irq, IRQF_DISABLED,
				pdev->name, pdata) < 0) {
			dev_warn(&pdev->dev, "interrupt not available.\n");
			pdata->irq = -1;
			goto err;
		}
	}

	/*
	 * Initialize dryice hardware
	 */

	/* put dryice into valid state */
	if (di_read(pdata, DSR) & DSR_NVF)
		di_write_wait_err(pdata, DSR_NVF | DSR_SVF, DSR, rc, err);

	/* mask alarm interrupt */
	di_int_disable(pdata, DIER_CAIE);

	/* initialize alarm */
	di_write_wait_err(pdata, DCAMR_UNSET, DCAMR, rc, err);
	di_write_wait_err(pdata, 0, DCALR, rc, err);

	/* clear alarm flag */
	if (di_read(pdata, DSR) & DSR_CAF)
		di_write_wait_err(pdata, DSR_CAF, DSR, rc, err);

	/* the timer won't count if it has never been written to */
	if (!di_read(pdata, DTCMR))
		di_write_wait_err(pdata, 0, DTCMR, rc, err);

	/* start keeping time */
	if (!(di_read(pdata, DCR) & DCR_TCE))
		di_write_wait_err(pdata, di_read(pdata, DCR) | DCR_TCE, DCR,
				  rc, err);

	rtc = rtc_device_register(pdev->name, &pdev->dev,
				  &dryice_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc)) {
		rc = PTR_ERR(rtc);
		goto err;
	}
	pdata->rtc = rtc;
	platform_set_drvdata(pdev, pdata);

	return 0;
err:
	if (pdata->rtc)
		rtc_device_unregister(pdata->rtc);

	if (pdata->irq >= 0)
		free_irq(pdata->irq, pdata);

	if (pdata->clk) {
		clk_disable(pdata->clk);
		clk_put(pdata->clk);
	}

	if (pdata->ioaddr)
		iounmap(pdata->ioaddr);

	if (pdata->baseaddr)
		release_mem_region(pdata->baseaddr, pdata->size);

	kfree(pdata);

	return rc;
}

static int __exit dryice_rtc_remove(struct platform_device *pdev)
{
	struct rtc_drv_data *pdata = platform_get_drvdata(pdev);

	flush_scheduled_work();

	if (pdata->rtc)
		rtc_device_unregister(pdata->rtc);

	/* mask alarm interrupt */
	di_int_disable(pdata, DIER_CAIE);

	if (pdata->irq >= 0)
		free_irq(pdata->irq, pdata);

	if (pdata->clk) {
		clk_disable(pdata->clk);
		clk_put(pdata->clk);
	}

	if (pdata->ioaddr)
		iounmap(pdata->ioaddr);

	if (pdata->baseaddr)
		release_mem_region(pdata->baseaddr, pdata->size);

	kfree(pdata);

	return 0;
}

static struct platform_driver dryice_rtc_driver = {
	.driver = {
		   .name = "imxdi_rtc",
		   .owner = THIS_MODULE,
		   },
	.probe = dryice_rtc_probe,
	.remove = __exit_p(dryice_rtc_remove),
};

static int __init dryice_rtc_init(void)
{
	pr_info("IMXDI Realtime Clock Driver (RTC)\n");
	return platform_driver_register(&dryice_rtc_driver);
}

static void __exit dryice_rtc_exit(void)
{
	platform_driver_unregister(&dryice_rtc_driver);
}

module_init(dryice_rtc_init);
module_exit(dryice_rtc_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("IMXDI Realtime Clock Driver (RTC)");
MODULE_LICENSE("GPL");
