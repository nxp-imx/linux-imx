/*
 * System timer for Freescale STMP37XX/STMP378X
 *
 * Embedded Alley Solutions, Inc <source@embeddedalley.com>
 *
 * Copyright 2009-2010 Freescale Semiconductor, Inc.
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

#include <asm/mach/time.h>
#include <mach/hardware.h>
#include <mach/device.h>
#include <mach/regs-timrot.h>

#ifndef HW_TIMROT_TIMCOUNTn
#define HW_TIMROT_TIMCOUNTn HW_TIMROT_RUNNING_COUNTn
#endif
static struct mxs_sys_timer *online_timer;

static irqreturn_t
mxs_nomatch_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *c = dev_id;

	/* timer 0 */
	if (__raw_readl(online_timer->base + HW_TIMROT_TIMCTRLn(0)) &
			BM_TIMROT_TIMCTRLn_IRQ) {

		__raw_writel(BM_TIMROT_TIMCTRLn_IRQ,
				online_timer->base + HW_TIMROT_TIMCTRLn_CLR(0));
		c->event_handler(c);
	}

	/* timer 1 */
	else if (__raw_readl(online_timer->base + HW_TIMROT_TIMCTRLn(1))
			& BM_TIMROT_TIMCTRLn_IRQ) {
		__raw_writel(BM_TIMROT_TIMCTRLn_IRQ,
				online_timer->base + HW_TIMROT_TIMCTRLn_CLR(1));
		__raw_writel(BM_TIMROT_TIMCTRLn_IRQ_EN,
				online_timer->base + HW_TIMROT_TIMCTRLn_CLR(1));
		__raw_writel(0xFFFF,
				online_timer->base + HW_TIMROT_TIMCOUNTn(1));
	}

	return IRQ_HANDLED;
}

static cycle_t mxs_nomatch_clock_read(struct clocksource *cs)
{
	return ~((__raw_readl(online_timer->base + HW_TIMROT_TIMCOUNTn(1))
				& 0xFFFF0000) >> 16);
}

static int
mxs_nomatch_timrot_set_next_event(unsigned long delta,
		struct clock_event_device *dev)
{
	/* reload the timer */
	__raw_writel(delta, online_timer->base + HW_TIMROT_TIMCOUNTn(0));
	return 0;
}

static void
mxs_nomatch_timrot_set_mode(enum clock_event_mode mode,
		struct clock_event_device *dev)
{
}

static struct clock_event_device ckevt_timrot = {
	.name		= "timrot",
	.features	= CLOCK_EVT_FEAT_ONESHOT,
	.shift		= 32,
	.set_next_event	= mxs_nomatch_timrot_set_next_event,
	.set_mode	= mxs_nomatch_timrot_set_mode,
};

static struct clocksource cksrc_mxs_nomatch = {
	.name           = "mxs clock source",
	.rating         = 250,
	.read           = mxs_nomatch_clock_read,
	.mask           = CLOCKSOURCE_MASK(16),
	.shift          = 10,
	.flags			= CLOCK_SOURCE_IS_CONTINUOUS,
};

static struct irqaction mxs_nomatch_timer_irq = {
	.name		= "mxs_nomatch_timer",
	.flags		= IRQF_DISABLED | IRQF_TIMER,
	.handler	= mxs_nomatch_timer_interrupt,
	.dev_id		= &ckevt_timrot,
};


/*
 * Set up timer interrupt, and return the current time in seconds.
 */
void mxs_nomatch_timer_init(struct mxs_sys_timer *timer)
{

	if (online_timer)
		return;

	online_timer = timer;

	cksrc_mxs_nomatch.mult = clocksource_hz2mult(clk_get_rate(timer->clk),
				cksrc_mxs_nomatch.shift);
	ckevt_timrot.mult = div_sc(clk_get_rate(timer->clk), NSEC_PER_SEC,
				ckevt_timrot.shift);
	ckevt_timrot.min_delta_ns = clockevent_delta2ns(2, &ckevt_timrot);
	ckevt_timrot.max_delta_ns = clockevent_delta2ns(0xFFF, &ckevt_timrot);
	ckevt_timrot.cpumask = cpumask_of(0);

	/* clear two timers */
	__raw_writel(0, online_timer->base + HW_TIMROT_TIMCOUNTn(0));
	__raw_writel(0, online_timer->base + HW_TIMROT_TIMCOUNTn(1));

	/* configure them */
	__raw_writel(
		(8 << BP_TIMROT_TIMCTRLn_SELECT) |  /* 32 kHz */
		BM_TIMROT_TIMCTRLn_RELOAD |
		BM_TIMROT_TIMCTRLn_UPDATE |
		BM_TIMROT_TIMCTRLn_IRQ_EN,
			online_timer->base + HW_TIMROT_TIMCTRLn(0));
	__raw_writel(
		(8 << BP_TIMROT_TIMCTRLn_SELECT) |  /* 32 kHz */
		BM_TIMROT_TIMCTRLn_RELOAD |
		BM_TIMROT_TIMCTRLn_UPDATE |
		BM_TIMROT_TIMCTRLn_IRQ_EN,
			online_timer->base + HW_TIMROT_TIMCTRLn(1));

	__raw_writel(clk_get_rate(timer->clk) / HZ - 1,
			online_timer->base + HW_TIMROT_TIMCOUNTn(0));
	__raw_writel(0xFFFF, online_timer->base + HW_TIMROT_TIMCOUNTn(1));

	setup_irq(IRQ_TIMER0, &mxs_nomatch_timer_irq);

	clocksource_register(&cksrc_mxs_nomatch);
	clockevents_register_device(&ckevt_timrot);
}

#ifdef CONFIG_PM

void mxs_nomatch_suspend_timer(void)
{
	__raw_writel(BM_TIMROT_TIMCTRLn_IRQ_EN | BM_TIMROT_TIMCTRLn_IRQ,
			online_timer->base  + HW_TIMROT_TIMCTRLn_CLR(0));
	__raw_writel(BM_TIMROT_ROTCTRL_CLKGATE,
			online_timer->base  + HW_TIMROT_ROTCTRL_SET);
}

void mxs_nomatch_resume_timer(void)
{
	__raw_writel(BM_TIMROT_ROTCTRL_SFTRST | BM_TIMROT_ROTCTRL_CLKGATE,
			online_timer->base  + HW_TIMROT_ROTCTRL_CLR);
	__raw_writel(
		8 << BP_TIMROT_TIMCTRLn_SELECT |  /* 32 kHz */
		BM_TIMROT_TIMCTRLn_RELOAD |
		BM_TIMROT_TIMCTRLn_UPDATE |
		BM_TIMROT_TIMCTRLn_IRQ_EN,
			online_timer->base  + HW_TIMROT_TIMCTRLn(0));
	__raw_writel(
		8 << BP_TIMROT_TIMCTRLn_SELECT |  /* 32 kHz */
		BM_TIMROT_TIMCTRLn_RELOAD |
		BM_TIMROT_TIMCTRLn_UPDATE |
		BM_TIMROT_TIMCTRLn_IRQ_EN,
			online_timer->base  + HW_TIMROT_TIMCTRLn(1));
	__raw_writel(clk_get_rate(online_timer->clk) / HZ - 1,
			online_timer->base  + HW_TIMROT_TIMCOUNTn(0));
	__raw_writel(0xFFFF, online_timer->base  + HW_TIMROT_TIMCOUNTn(1));
}

#else

#define mxs_nomatch_suspend_timer	NULL
#define	mxs_nomatch_resume_timer	NULL

#endif	/* CONFIG_PM */
