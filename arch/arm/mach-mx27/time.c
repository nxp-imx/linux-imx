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

/* System Timer Interrupt reconfigured to run in free-run mode.
 * Author: Vitaly Wool
 * Copyright 2004 MontaVista Software Inc.
 */

/*!
 * @defgroup Timers_MX27 OS Tick Timer
 * @ingroup MSL_MX27
 */
/*!
 * @file mach-mx27/time.c
 * @brief This file contains OS tick timer implementation.
 *
 * This file contains OS tick timer implementation.
 *
 * @ingroup Timers_MX27
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/mtd/xip.h>
#include <mach/hardware.h>
#include <asm/mach/time.h>

/*!
 * GPT register address definitions
 */
#define GPT_BASE_ADDR		(IO_ADDRESS(GPT1_BASE_ADDR))
#define MXC_GPT_GPTCR		(GPT_BASE_ADDR + 0x00)
#define MXC_GPT_GPTPR		(GPT_BASE_ADDR + 0x04)
#define MXC_GPT_GPTOCR1		(GPT_BASE_ADDR + 0x08)
#define MXC_GPT_GPTICR1		(GPT_BASE_ADDR + 0x0C)
#define MXC_GPT_GPTCNT		(GPT_BASE_ADDR + 0x10)
#define MXC_GPT_GPTSR		(GPT_BASE_ADDR + 0x14)

/*!
 * GPT Control register bit definitions
 */
#define GPTCR_COMPEN			(1 << 4)
#define GPTCR_SWR			(1 << 15)
#define GPTCR_FRR			(1 << 8)

#define GPTCR_CLKSRC_SHIFT		1
#define GPTCR_CLKSRC_MASK		(7 << GPTCR_CLKSRC_SHIFT)
#define GPTCR_CLKSRC_NOCLOCK		(0 << GPTCR_CLKSRC_SHIFT)
#define GPTCR_CLKSRC_HIGHFREQ		(1 << GPTCR_CLKSRC_SHIFT)
#define GPTCR_CLKSRC_CLKIN		(3 << GPTCR_CLKSRC_SHIFT)
#define GPTCR_CLKSRC_CLK32K		(4 << GPTCR_CLKSRC_SHIFT)

#define GPTCR_ENABLE			(1 << 0)

#define	GPTSR_OF1			(1 << 0)

extern unsigned long clk_early_get_timer_rate(void);

static int mxc_gpt_set_next_event(unsigned long cycles,
				  struct clock_event_device *evt)
{
	unsigned long now, expires;
	u32 reg;

	now = __raw_readl(MXC_GPT_GPTCNT);
	expires = now + cycles;
	__raw_writel(expires, MXC_GPT_GPTOCR1);
	__raw_writel(GPTSR_OF1, MXC_GPT_GPTSR);

	/* enable interrupt */
	reg = __raw_readl(MXC_GPT_GPTCR);
	reg |= GPTCR_COMPEN;
	__raw_writel(reg, MXC_GPT_GPTCR);

	return 0;
}

static void mxc_gpt_set_mode(enum clock_event_mode mode,
			     struct clock_event_device *evt)
{
	u32 reg;
	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		panic("MXC GPT: CLOCK_EVT_MODE_PERIODIC not supported\n");
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		break;
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		/* Disable interrupts */
		reg = __raw_readl(MXC_GPT_GPTCR);
		reg &= ~GPTCR_COMPEN;
		__raw_writel(reg, MXC_GPT_GPTCR);
		break;
	case CLOCK_EVT_MODE_RESUME:
		break;
	}
}

static struct clock_event_device gpt_clockevent = {
	.name = "mxc_gpt",
	.features = CLOCK_EVT_FEAT_ONESHOT,
	.rating = 300,
	.shift = 32,
	.set_next_event = mxc_gpt_set_next_event,
	.set_mode = mxc_gpt_set_mode,
};

/*!
 * This is the timer interrupt service routine to do required tasks.
 * It also services the WDOG timer at the frequency of twice per WDOG
 * timeout value. For example, if the WDOG's timeout value is 4 (2
 * seconds since the WDOG runs at 0.5Hz), it will be serviced once
 * every 2/2=1 second.
 *
 * @param  irq          GPT interrupt source number (not used)
 * @param  dev_id       this parameter is not used
 * @return always returns \b IRQ_HANDLED as defined in
 *         include/linux/interrupt.h.
 */
static irqreturn_t mxc_timer_interrupt(int irq, void *dev_id)
{
	unsigned int gptsr;
	u32 reg;

	gptsr = __raw_readl(MXC_GPT_GPTSR);
	if (gptsr & GPTSR_OF1) {
		/* Disable interrupt */
		reg = __raw_readl(MXC_GPT_GPTCR);
		reg &= ~GPTCR_COMPEN;
		__raw_writel(reg, MXC_GPT_GPTCR);
		/* Clear interrupt */
		__raw_writel(GPTSR_OF1, MXC_GPT_GPTSR);

		gpt_clockevent.event_handler(&gpt_clockevent);
	}

	return IRQ_HANDLED;
}

/*!
 * The clockevents timer interrupt structure.
 */
static struct irqaction timer_irq = {
	.name = "gpt-irq",
	.flags = IRQF_DISABLED,
	.handler = mxc_timer_interrupt,
};

static cycle_t __xipram mxc_gpt_read(void)
{
	return __raw_readl(MXC_GPT_GPTCNT);
}

static struct clocksource gpt_clocksrc = {
	.name = "mxc_gpt",
	.rating = 300,
	.read = mxc_gpt_read,
	.mask = CLOCKSOURCE_MASK(32),
	.shift = 24,
	.flags = CLOCK_SOURCE_IS_CONTINUOUS | CLOCK_SOURCE_VALID_FOR_HRES,
};

/*!
 * This function is used to initialize the GPT as a clocksource and clockevent.
 * It is called by the start_kernel() during system startup.
 */
void __init mxc_init_time(void)
{
	int ret;
	unsigned long rate;
	u32 reg, div;

	/* Reset GPT */
	__raw_writel(GPTCR_SWR, MXC_GPT_GPTCR);
	while ((__raw_readl(MXC_GPT_GPTCR) & GPTCR_SWR) != 0)
		mb();

	/* Normal clk api are not yet initialized, so use early verion */
	rate = clk_early_get_timer_rate();
	if (rate == 0)
		panic("MXC GPT: Can't get timer clock rate\n");

#ifdef CLOCK_TICK_RATE
	div = rate / CLOCK_TICK_RATE;
	WARN_ON((div * CLOCK_TICK_RATE) != rate);
#else				/* Hopefully CLOCK_TICK_RATE will go away soon */
	div = 1;
	while ((rate / div) > 20000000) {
		div++;
	}
#endif
	rate /= div;
	__raw_writel(div - 1, MXC_GPT_GPTPR);

	reg = GPTCR_FRR | GPTCR_CLKSRC_HIGHFREQ | GPTCR_ENABLE;
	__raw_writel(reg, MXC_GPT_GPTCR);

	gpt_clocksrc.mult = clocksource_hz2mult(rate, gpt_clocksrc.shift);
	ret = clocksource_register(&gpt_clocksrc);
	if (ret < 0) {
		goto err;
	}

	gpt_clockevent.mult = div_sc(rate, NSEC_PER_SEC, gpt_clockevent.shift);
	gpt_clockevent.max_delta_ns = clockevent_delta2ns(-1, &gpt_clockevent);
	gpt_clockevent.min_delta_ns = clockevent_delta2ns(1, &gpt_clockevent);

	gpt_clockevent.cpumask = cpumask_of_cpu(0);
	clockevents_register_device(&gpt_clockevent);

	ret = setup_irq(MXC_INT_GPT, &timer_irq);
	if (ret < 0) {
		goto err;
	}

	pr_info("MXC GPT timer initialized, rate = %lu\n", rate);
	return;
      err:
	panic("Unable to initialize timer\n");
}

struct sys_timer mxc_timer = {
	.init = mxc_init_time,
};
