// SPDX-License-Identifier: GPL-2.0+
//
// Copyright 2017-2019 NXP

#include <linux/interrupt.h>
#include <linux/clockchips.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "timer-of.h"

#define CMP_OFFSET	0x10000
#define RD_OFFSET	0x20000

#define CNTCV_LO	0x8
#define CNTCV_HI	0xc
#define CNTCV_LO_IMX95	(RD_OFFSET + 0x8)
#define CNTCV_HI_IMX95	(RD_OFFSET + 0xc)
#define CMPCV_LO	(CMP_OFFSET + 0x20)
#define CMPCV_HI	(CMP_OFFSET + 0x24)
#define CMPCR		(CMP_OFFSET + 0x2c)

#define SYS_CTR_EN		0x1
#define SYS_CTR_IRQ_MASK	0x2

#define SYS_CTR_CLK_DIV		0x3

#define SYSCTRL_IMX95		BIT(0)

static void __iomem *sys_ctr_base __ro_after_init;
static u32 cmpcr __ro_after_init;
static u32 sysctr_flag __ro_after_init;
static u32 cntcv_hi = CNTCV_HI;
static u32 cntcv_lo = CNTCV_LO;
static struct timer_of to_sysctr;

static inline bool sysctr_is_imx95(void)
{
	return sysctr_flag & SYSCTRL_IMX95 ? true : false;
}

static void sysctr_timer_read_write(void __iomem *addr, u32 mask, u32 val, int count)
{
	u32 i = 0;

	while ((readl(addr) & mask) != val) {
		writel(val, addr);
		count--;
		if (count <= 0) {
			pr_err("%s:%x:%x write failed, retry: %u\n",
			       __func__, (uint32_t)(addr - sys_ctr_base), val, i++);
			count = 1000;
		}
	}
}

static void sysctr_timer_enable(bool enable)
{
	u32 val;

	val = enable ? cmpcr | SYS_CTR_EN : cmpcr;
	writel(val, sys_ctr_base + CMPCR);

	if (!sysctr_is_imx95())
		return;

	sysctr_timer_read_write(sys_ctr_base + CMPCR, val, val, 1000);
}

static void sysctr_irq_acknowledge(void)
{
	/*
	 * clear the enable bit(EN =0) will clear
	 * the status bit(ISTAT = 0), then the interrupt
	 * signal will be negated(acknowledged).
	 */
	sysctr_timer_enable(false);
}

static inline u64 sysctr_read_counter(void)
{
	u32 cnt_hi, tmp_hi, cnt_lo;

	do {
		cnt_hi = readl_relaxed(sys_ctr_base + cntcv_hi);
		cnt_lo = readl_relaxed(sys_ctr_base + cntcv_lo);
		tmp_hi = readl_relaxed(sys_ctr_base + cntcv_hi);
	} while (tmp_hi != cnt_hi);

	return  ((u64) cnt_hi << 32) | cnt_lo;
}

static int sysctr_set_next_event(unsigned long delta,
				 struct clock_event_device *evt)
{
	u32 cmp_hi, cmp_lo;
	u64 next;

	sysctr_timer_enable(false);

	next = sysctr_read_counter();

	next += delta;

	cmp_hi = (next >> 32) & 0x00fffff;
	cmp_lo = next & 0xffffffff;

	writel_relaxed(cmp_hi, sys_ctr_base + CMPCV_HI);
	writel_relaxed(cmp_lo, sys_ctr_base + CMPCV_LO);

	if (sysctr_is_imx95())
		disable_irq_nosync(to_sysctr.clkevt.irq);

	sysctr_timer_enable(true);

	if (!sysctr_is_imx95())
		return 0;

	sysctr_timer_read_write(sys_ctr_base + CMPCV_HI, GENMASK(31, 0), cmp_hi, 1000);
	sysctr_timer_read_write(sys_ctr_base + CMPCV_LO, GENMASK(31, 0), cmp_lo, 1000);

	enable_irq(to_sysctr.clkevt.irq);

	return 0;
}

static int sysctr_set_state_oneshot(struct clock_event_device *evt)
{
	return 0;
}

static int sysctr_set_state_shutdown(struct clock_event_device *evt)
{
	sysctr_timer_enable(false);

	return 0;
}

static irqreturn_t sysctr_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;

	sysctr_irq_acknowledge();

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct timer_of to_sysctr = {
	.flags = TIMER_OF_IRQ | TIMER_OF_CLOCK | TIMER_OF_BASE,
	.clkevt = {
		.name			= "i.MX system counter timer",
		.features		= CLOCK_EVT_FEAT_ONESHOT |
						CLOCK_EVT_FEAT_DYNIRQ,
		.set_state_oneshot	= sysctr_set_state_oneshot,
		.set_next_event		= sysctr_set_next_event,
		.set_state_shutdown	= sysctr_set_state_shutdown,
		.rating			= 200,
	},
	.of_irq = {
		.handler		= sysctr_timer_interrupt,
		.flags			= IRQF_TIMER,
	},
	.of_clk = {
		.name = "per",
	},
};

static void __init sysctr_clockevent_init(void)
{
	to_sysctr.clkevt.cpumask = cpu_possible_mask;

	clockevents_config_and_register(&to_sysctr.clkevt,
					timer_of_rate(&to_sysctr),
					0xff, 0x7fffffff);
}

static int sysctr_timer_init(struct device_node *np)
{
	int ret = 0;

	ret = timer_of_init(np, &to_sysctr);
	if (ret)
		return ret;

	if (!of_property_read_bool(np, "nxp,no-divider")) {
		/* system counter clock is divided by 3 internally */
		to_sysctr.of_clk.rate /= SYS_CTR_CLK_DIV;
	}

	if (of_device_is_compatible(np, "nxp,imx95-sysctr-timer")) {
		cntcv_hi = CNTCV_HI_IMX95;
		cntcv_lo = CNTCV_LO_IMX95;
		sysctr_flag |= SYSCTRL_IMX95;
	}

	sys_ctr_base = timer_of_base(&to_sysctr);
	cmpcr = readl(sys_ctr_base + CMPCR);
	cmpcr &= ~SYS_CTR_EN;

	sysctr_clockevent_init();

	return 0;
}
#ifdef MODULE
static int sysctr_timer_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;

	return sysctr_timer_init(np);
}

static const struct of_device_id sysctr_timer_match_table[] = {
	{ .compatible = "nxp,sysctr-timer" },
	{ .compatible = "nxp,imx95-sysctr-timer" },
	{ }
};
MODULE_DEVICE_TABLE(of, sysctr_timer_match_table);

static struct platform_driver sysctr_timer_driver = {
	.probe		= sysctr_timer_probe,
	.driver		= {
		.name	= "sysctr-timer",
		.of_match_table = sysctr_timer_match_table,
	},
};
module_platform_driver(sysctr_timer_driver);

#else
TIMER_OF_DECLARE(sysctr_timer, "nxp,sysctr-timer", sysctr_timer_init);
#endif

MODULE_LICENSE("GPL");
