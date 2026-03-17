// SPDX-License-Identifier: GPL-2.0+
//
// Copyright 2016 Freescale Semiconductor, Inc.
// Copyright 2017, 2026 NXP

#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#define TPM_PARAM			0x4
#define TPM_PARAM_WIDTH_SHIFT		16
#define TPM_PARAM_WIDTH_MASK		(0xff << 16)
#define TPM_SC				0x10
#define TPM_SC_CMOD_INC_PER_CNT		(0x1 << 3)
#define TPM_SC_CMOD_DIV_DEFAULT		0x3
#define TPM_SC_CMOD_DIV_MAX		0x7
#define TPM_SC_TOF_MASK			(0x1 << 7)
#define TPM_CNT				0x14
#define TPM_MOD				0x18
#define TPM_STATUS			0x1c
#define TPM_STATUS_CH0F			BIT(0)
#define TPM_C0SC			0x20
#define TPM_C0SC_CHIE			BIT(6)
#define TPM_C0SC_MODE_SHIFT		2
#define TPM_C0SC_MODE_MASK		0x3c
#define TPM_C0SC_MODE_SW_COMPARE	0x4
#define TPM_C0SC_CHF_MASK		(0x1 << 7)
#define TPM_C0V				0x24

/* Timer rating values */
#define TPM_RATING_32BIT		200
#define TPM_RATING_16BIT		150

/**
 * struct tpm_timer - TPM timer instance data
 * @base: Base address of the TPM timer registers
 * @ipg: IPG clock (bus clock)
 * @per: Peripheral clock (functional clock)
 * @rate: Effective timer rate after prescaler
 * @counter_width: Counter width in bits (16 or 32)
 * @clkevt: Clock event device
 * @clksrc: Clock source device
 */
struct tpm_timer {
	void __iomem		*base;
	struct clk		*ipg;
	struct clk		*per;
	unsigned long		rate;
	int			counter_width;
	struct clock_event_device clkevt;
	struct clocksource	clksrc;
};

static inline int tpm_get_rating(int counter_width)
{
	return (counter_width == 0x20) ? TPM_RATING_32BIT : TPM_RATING_16BIT;
}

static inline void tpm_timer_disable(struct tpm_timer *tpm)
{
	unsigned int val;

	/* channel disable */
	val = readl(tpm->base + TPM_C0SC);
	val &= ~(TPM_C0SC_MODE_MASK | TPM_C0SC_CHIE);
	writel(val, tpm->base + TPM_C0SC);
}

static inline void tpm_timer_enable(struct tpm_timer *tpm)
{
	unsigned int val;

	/* channel enabled in sw compare mode */
	val = readl(tpm->base + TPM_C0SC);
	val |= (TPM_C0SC_MODE_SW_COMPARE << TPM_C0SC_MODE_SHIFT) |
	       TPM_C0SC_CHIE;
	writel(val, tpm->base + TPM_C0SC);
}

static inline void tpm_irq_acknowledge(struct tpm_timer *tpm)
{
	writel(TPM_STATUS_CH0F, tpm->base + TPM_STATUS);
}

static inline unsigned long tpm_read_counter(struct tpm_timer *tpm)
{
	return readl(tpm->base + TPM_CNT);
}

static int tpm_set_next_event(unsigned long delta,
			       struct clock_event_device *evt)
{
	struct tpm_timer *tpm = container_of(evt, struct tpm_timer, clkevt);
	unsigned long next, prev, now;

	prev = tpm_read_counter(tpm);
	next = prev + delta;
	writel(next, tpm->base + TPM_C0V);
	now = tpm_read_counter(tpm);

	/*
	 * Need to wait CNT increase at least 1 cycle to make sure
	 * the C0V has been updated into HW.
	 */
	if ((next & 0xffffffff) != readl(tpm->base + TPM_C0V))
		while (now == tpm_read_counter(tpm))
			;

	/*
	 * NOTE: We observed in a very small probability, the bus fabric
	 * contention between GPU and A7 may results a few cycles delay
	 * of writing CNT registers which may cause the min_delta event got
	 * missed, so we need add a ETIME check here in case it happened.
	 */
	return (now - prev) >= delta ? -ETIME : 0;
}

static int tpm_set_state_oneshot(struct clock_event_device *evt)
{
	struct tpm_timer *tpm = container_of(evt, struct tpm_timer, clkevt);

	tpm_timer_enable(tpm);
	return 0;
}

static int tpm_set_state_shutdown(struct clock_event_device *evt)
{
	struct tpm_timer *tpm = container_of(evt, struct tpm_timer, clkevt);

	tpm_timer_disable(tpm);
	return 0;
}

static irqreturn_t tpm_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;
	struct tpm_timer *tpm = container_of(evt, struct tpm_timer, clkevt);

	tpm_irq_acknowledge(tpm);

	if (evt->event_handler)
		evt->event_handler(evt);

	return IRQ_HANDLED;
}

static u64 tpm_clocksource_read(struct clocksource *cs)
{
	struct tpm_timer *tpm = container_of(cs, struct tpm_timer, clksrc);
	return readl(tpm->base + TPM_CNT);
}

static int tpm_clocksource_init(struct tpm_timer *tpm)
{
	struct clocksource *cs = &tpm->clksrc;

	cs->name	= "imx-tpm";
	cs->rating	= tpm_get_rating(tpm->counter_width);
	cs->read	= tpm_clocksource_read;
	cs->mask	= CLOCKSOURCE_MASK(tpm->counter_width);
	cs->flags	= CLOCK_SOURCE_IS_CONTINUOUS;

	return clocksource_register_hz(cs, tpm->rate);
}

static void tpm_clockevent_init(struct tpm_timer *tpm)
{
	struct clock_event_device *evt = &tpm->clkevt;

	evt->name		= "i.MX TPM Timer";
	evt->features		= CLOCK_EVT_FEAT_ONESHOT | CLOCK_EVT_FEAT_DYNIRQ;
	evt->set_state_shutdown	= tpm_set_state_shutdown;
	evt->set_state_oneshot	= tpm_set_state_oneshot;
	evt->set_next_event	= tpm_set_next_event;
	evt->rating		= tpm_get_rating(tpm->counter_width);
	evt->cpumask		= cpu_possible_mask;

	clockevents_config_and_register(evt, tpm->rate, 300,
					GENMASK(tpm->counter_width - 1, 1));
}

static int tpm_timer_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tpm_timer *tpm;
	int ret, irq;
	u32 param;

	tpm = devm_kzalloc(dev, sizeof(*tpm), GFP_KERNEL);
	if (!tpm)
		return -ENOMEM;

	tpm->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(tpm->base))
		return PTR_ERR(tpm->base);

	tpm->ipg = devm_clk_get(dev, "ipg");
	if (IS_ERR(tpm->ipg))
		return dev_err_probe(dev, PTR_ERR(tpm->ipg),
				     "failed to get ipg clock\n");

	tpm->per = devm_clk_get(dev, "per");
	if (IS_ERR(tpm->per))
		return dev_err_probe(dev, PTR_ERR(tpm->per),
				     "failed to get per clock\n");

	/* Enable IPG clock */
	ret = clk_prepare_enable(tpm->ipg);
	if (ret) {
		dev_err(dev, "failed to enable ipg clock: %d\n", ret);
		return ret;
	}

	/* Enable PER clock */
	ret = clk_prepare_enable(tpm->per);
	if (ret) {
		dev_err(dev, "failed to enable per clock: %d\n", ret);
		goto err_disable_ipg;
	}

	tpm->rate = clk_get_rate(tpm->per);
	if (!tpm->rate) {
		dev_err(dev, "invalid clock rate\n");
		ret = -EINVAL;
		goto err_disable_per;
	}

	param = readl(tpm->base + TPM_PARAM);
	tpm->counter_width = (param & TPM_PARAM_WIDTH_MASK) >> TPM_PARAM_WIDTH_SHIFT;

	/*
	 * Initialize tpm module to a known state
	 * 1) Counter disabled
	 * 2) TPM counter operates in up counting mode
	 * 3) Timer Overflow Interrupt disabled
	 * 4) Channel0 disabled
	 * 5) DMA transfers disabled
	 */
	writel(0, tpm->base + TPM_SC);
	/* TOF is W1C */
	writel(TPM_SC_TOF_MASK, tpm->base + TPM_SC);
	writel(0, tpm->base + TPM_CNT);
	/* CHF is W1C */
	writel(TPM_C0SC_CHF_MASK, tpm->base + TPM_C0SC);

	/*
	 * increase per cnt,
	 * div 8 for 32-bit counter and div 128 for 16-bit counter
	 */
	writel(TPM_SC_CMOD_INC_PER_CNT |
		(tpm->counter_width == 0x20 ?
		TPM_SC_CMOD_DIV_DEFAULT : TPM_SC_CMOD_DIV_MAX),
		tpm->base + TPM_SC);

	/* set MOD register to maximum for free running mode */
	writel(GENMASK(tpm->counter_width - 1, 0), tpm->base + TPM_MOD);

	/* Adjust rate based on prescaler */
	tpm->rate >>= (tpm->counter_width == 0x20 ? 3 : 7);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		ret = irq;
		goto err_disable_per;
	}

	ret = devm_request_irq(dev, irq, tpm_timer_interrupt,
			       IRQF_TIMER, dev_name(dev), &tpm->clkevt);
	if (ret) {
		dev_err(dev, "failed to request IRQ: %d\n", ret);
		goto err_disable_per;
	}

	tpm_clockevent_init(tpm);

	ret = tpm_clocksource_init(tpm);
	if (ret) {
		dev_err(dev, "failed to init clocksource: %d\n", ret);
		goto err_disable_per;
	}

	platform_set_drvdata(pdev, tpm);

	dev_info(dev, "TPM timer initialized (width=%d bits, rate=%lu Hz, rating=%d)\n",
		 tpm->counter_width, tpm->rate, tpm_get_rating(tpm->counter_width));

	return 0;

err_disable_per:
	clk_disable_unprepare(tpm->per);
err_disable_ipg:
	clk_disable_unprepare(tpm->ipg);
	return ret;
}

static void tpm_timer_remove(struct platform_device *pdev)
{
	struct tpm_timer *tpm = platform_get_drvdata(pdev);

	tpm_timer_disable(tpm);
	clk_disable_unprepare(tpm->per);
	clk_disable_unprepare(tpm->ipg);
}

static const struct of_device_id tpm_timer_of_match[] = {
	{ .compatible = "fsl,imx7ulp-tpm" },
	{ }
};
MODULE_DEVICE_TABLE(of, tpm_timer_of_match);

static struct platform_driver tpm_timer_driver = {
	.probe		= tpm_timer_probe,
	.remove		= tpm_timer_remove,
	.driver		= {
		.name	= "imx-tpm-timer",
		.of_match_table = tpm_timer_of_match,
	},
};
module_platform_driver(tpm_timer_driver);

MODULE_DESCRIPTION("i.MX TPM Timer Driver");
MODULE_LICENSE("GPL");