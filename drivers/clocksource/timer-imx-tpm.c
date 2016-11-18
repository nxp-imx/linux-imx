/*
 * Copyright 2016 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/delay.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/sched_clock.h>

#define TPM_GLOBAL	0x8

#define TPM_SC		0x10
#define TPM_CNT		0x14
#define TPM_MOD		0x18
#define TPM_STATUS	0x1c
#define TPM_C0SC	0x20
#define TPM_C0V		0x24

#define TPM_STATUS_CH0F	0x1

#define TPM_C0SC_MSA	0x4

static void __iomem *timer_base;
static struct clock_event_device clockevent_tpm;

static inline void tpm_timer_disable(void)
{
	unsigned int val;

	val = __raw_readl(timer_base + TPM_C0SC);
	val &= ~(0x5 << TPM_C0SC_MSA);
	__raw_writel(val, timer_base + TPM_C0SC);
}

static inline void tpm_timer_enable(void)
{
	unsigned int val;

	val = __raw_readl(timer_base + TPM_C0SC);
	val |= (0x5 << TPM_C0SC_MSA);
	__raw_writel(val, timer_base + TPM_C0SC);
}

static inline void tpm_irq_acknowledge(void)
{
	__raw_writel(1, timer_base + TPM_STATUS);
}

static struct delay_timer tpm_delay_timer;

static unsigned long tpm_read_current_timer(void)
{
	return __raw_readl(timer_base + TPM_CNT);
}

static u64 notrace tpm_read_sched_clock(void)
{
	return __raw_readl(timer_base + TPM_CNT);
}

static int __init tpm_clocksource_init(unsigned long rate)
{
	tpm_delay_timer.read_current_timer = &tpm_read_current_timer;
	tpm_delay_timer.freq = rate;
	register_current_timer_delay(&tpm_delay_timer);

	sched_clock_register(tpm_read_sched_clock, 32, rate);
	return clocksource_mmio_init(timer_base + TPM_CNT, "imx-tpm",
		 rate, 200, 32, clocksource_mmio_readl_up);
}

static int tpm_set_next_event(unsigned long delta,
				struct clock_event_device *evt)
{
	unsigned long tmp;

	tmp = __raw_readl(timer_base + TPM_CNT) + delta;
	__raw_writel(tmp, timer_base + TPM_C0V);

	return 0;
}

static int tpm_set_state_oneshot(struct clock_event_device *evt)
{
	/* enable timer */
	tpm_timer_enable();

	return 0;
}

static int tpm_set_state_shutdown(struct clock_event_device *evt)
{
	/* disable timer */
	tpm_timer_disable();

	return 0;
}

static irqreturn_t tpm_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &clockevent_tpm;

	tpm_irq_acknowledge();

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct clock_event_device clockevent_tpm = {
	.name			= "i.MX7ULP tpm timer",
	.features		= CLOCK_EVT_FEAT_ONESHOT,
	.set_state_oneshot	= tpm_set_state_oneshot,
	.set_next_event		= tpm_set_next_event,
	.set_state_shutdown	= tpm_set_state_shutdown,
	.rating			= 200,
};

static struct irqaction tpm_timer_irq = {
	.name		= "i.MX7ULP tpm timer",
	.flags		= IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= tpm_timer_interrupt,
	.dev_id		= &clockevent_tpm,
};

static int __init tpm_clockevent_init(unsigned long rate, int irq)
{
	/* init the channel */
	setup_irq(irq, &tpm_timer_irq);

	clockevent_tpm.cpumask = cpumask_of(0);
	clockevent_tpm.irq = irq;
	clockevents_config_and_register(&clockevent_tpm,
		rate, 2, 0xfffffffe);

	return 0;
}

static int __init tpm_timer_init(struct device_node *np)
{
	struct clk *clk;
	uint32_t val;
	int irq;

	timer_base = of_iomap(np, 0);
	BUG_ON(!timer_base);

	irq = irq_of_parse_and_map(np, 0);

	clk = of_clk_get(np, 0);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	/* clock shoube be enabled before access to the timer registers */
	clk_prepare_enable(clk);

	/* Initialize tpm module to a known state(counter disabled). */
	__raw_writel(0, timer_base + TPM_SC);
	__raw_writel(0, timer_base + TPM_CNT);
	__raw_writel(0, timer_base + TPM_C0SC);

	/* set the prescale div, div by 8 = 3MHz */
	__raw_writel(0xb, timer_base + TPM_SC);

	/* set the MOD register to 0xffffffff for free running counter */
	__raw_writel(0xffffffff, timer_base + TPM_MOD);

	tpm_clocksource_init(clk_get_rate(clk) / 8);
	tpm_clockevent_init(clk_get_rate(clk) / 8, irq);

	val = __raw_readl(timer_base);

	return 0;
}
CLOCKSOURCE_OF_DECLARE(imx7ulp, "fsl,imx7ulp-tpm", tpm_timer_init);

