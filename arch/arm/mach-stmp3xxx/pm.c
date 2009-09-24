/*
 * Static Power Management support for Freescale STMP37XX/STMP378X
 *
 * Author: Vitaly Wool <vital@embeddedalley.com>
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/suspend.h>
#include <linux/rtc.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/kthread.h>

#include <asm/dma.h>
#include <asm/cacheflush.h>
#include <asm/mach-types.h>

#include <asm/mach/time.h>

#include <mach/platform.h>

#include <mach/regs-icoll.h>
#include <mach/regs-rtc.h>
#include <mach/regs-clkctrl.h>
#include <mach/regs-pinctrl.h>
#include <mach/regs-power.h>
#include <mach/regs-gpmi.h>
#include <mach/regs-pwm.h>
#include <mach/regs-usbctrl.h>
#include <mach/regs-apbh.h>
#include <mach/regs-apbx.h>
#include <mach/regs-rtc.h>
#include <mach/regs-dram.h>
#include <mach/regs-emi.h>
#include <mach/regs-digctl.h>

#include "clock.h"
#include "sleep.h"
#include "common.h"
#define PENDING_IRQ_RETRY 100
static void *saved_sram;
static int saved_sleep_state;

#define WAIT_DC_OK_CYCLES 24000
#define WAIT_CYCLE(n) for (i = 0; i < n; i++);
#define LOWER_VDDIO 10
#define LOWER_VDDA 9
#define LOWER_VDDD 0xa
#define MAX_POWEROFF_CODE_SIZE (6 * 1024)

static void stmp378x_standby(void)
{
	int i;
	u32 reg_vddd, reg_vdda, reg_vddio;

	/* DDR EnterSelfrefreshMode */
	HW_DRAM_CTL08_SET(BM_DRAM_CTL08_SREFRESH);

	/* Gating EMI CLock */
	HW_CLKCTRL_EMI_SET(BM_CLKCTRL_EMI_CLKGATE);

	/* Disable PLL */
	HW_CLKCTRL_PLLCTRL0_CLR(BM_CLKCTRL_PLLCTRL0_POWER);

	/* Reduce the VDDIO (3.050 volt) */
	reg_vddio = HW_POWER_VDDIOCTRL_RD();
	HW_POWER_VDDIOCTRL_WR(HW_POWER_VDDIOCTRL_RD() |
		BM_POWER_VDDIOCTRL_BO_OFFSET);
	HW_POWER_VDDIOCTRL_WR(
		(HW_POWER_VDDIOCTRL_RD() &
		~BM_POWER_VDDIOCTRL_TRG)|LOWER_VDDIO);
	WAIT_CYCLE(WAIT_DC_OK_CYCLES)

	while (!(HW_POWER_STS_RD() & BM_POWER_STS_DC_OK))
		;

	/* Reduce VDDA 1.725volt */
	reg_vdda = HW_POWER_VDDACTRL_RD();
	HW_POWER_VDDACTRL_WR(HW_POWER_VDDACTRL_RD() |
			BM_POWER_VDDACTRL_BO_OFFSET);
	HW_POWER_VDDACTRL_WR((HW_POWER_VDDACTRL_RD() &
			~BM_POWER_VDDACTRL_TRG) | LOWER_VDDA);
	WAIT_CYCLE(WAIT_DC_OK_CYCLES)

	/* wait for DC_OK */
	while (!(HW_POWER_STS_RD() & BM_POWER_STS_DC_OK))
		;

	/* Reduce VDDD 1.000 volt */
	reg_vddd = HW_POWER_VDDDCTRL_RD();
	HW_POWER_VDDDCTRL_WR(HW_POWER_VDDDCTRL_RD() |
			BM_POWER_VDDDCTRL_BO_OFFSET);
	HW_POWER_VDDDCTRL_WR((HW_POWER_VDDDCTRL_RD() &
		~BM_POWER_VDDDCTRL_TRG) | LOWER_VDDD);

	WAIT_CYCLE(WAIT_DC_OK_CYCLES)

	while (!(HW_POWER_STS_RD() & BM_POWER_STS_DC_OK))
		;

	/* optimize the DCDC loop gain */
	HW_POWER_LOOPCTRL_WR((HW_POWER_LOOPCTRL_RD() &
			~BM_POWER_LOOPCTRL_EN_RCSCALE));
	HW_POWER_LOOPCTRL_WR((HW_POWER_LOOPCTRL_RD() &
			~BM_POWER_LOOPCTRL_DC_R) |
			(2<<BP_POWER_LOOPCTRL_DC_R));

	/* half the fets */
	HW_POWER_MINPWR_SET(BM_POWER_MINPWR_HALF_FETS);

	HW_POWER_LOOPCTRL_CLR(BM_POWER_LOOPCTRL_CM_HYST_THRESH);
	HW_POWER_LOOPCTRL_CLR(BM_POWER_LOOPCTRL_EN_CM_HYST);
	HW_POWER_LOOPCTRL_CLR(BM_POWER_LOOPCTRL_EN_DF_HYST);
	/* enable PFM */
	HW_POWER_LOOPCTRL_SET(BM_POWER_LOOPCTRL_HYST_SIGN);
	HW_POWER_MINPWR_SET(BM_POWER_MINPWR_EN_DC_PFM);

	HW_CLKCTRL_CPU_SET(BM_CLKCTRL_CPU_INTERRUPT_WAIT);
	/* Power off ... */
	asm("mcr     p15, 0, r2, c7, c0, 4");
	HW_CLKCTRL_CPU_CLR(BM_CLKCTRL_CPU_INTERRUPT_WAIT);

	/* restore the DCDC parameter */

	HW_POWER_MINPWR_CLR(BM_POWER_MINPWR_EN_DC_PFM);
	HW_POWER_LOOPCTRL_CLR(BM_POWER_LOOPCTRL_HYST_SIGN);
	HW_POWER_LOOPCTRL_SET(BM_POWER_LOOPCTRL_EN_DF_HYST);
	HW_POWER_LOOPCTRL_SET(BM_POWER_LOOPCTRL_EN_CM_HYST);
	HW_POWER_LOOPCTRL_SET(BM_POWER_LOOPCTRL_CM_HYST_THRESH);

	HW_POWER_LOOPCTRL_WR((HW_POWER_LOOPCTRL_RD() &
				~BM_POWER_LOOPCTRL_DC_R) |
				(2<<BP_POWER_LOOPCTRL_DC_R));
	HW_POWER_LOOPCTRL_WR((HW_POWER_LOOPCTRL_RD() &
				~BM_POWER_LOOPCTRL_EN_RCSCALE) |
				(3 << BP_POWER_LOOPCTRL_EN_RCSCALE));


	/* Restore VDDD */
	HW_POWER_VDDDCTRL_WR(reg_vddd);

	WAIT_CYCLE(WAIT_DC_OK_CYCLES)
	while (!(HW_POWER_STS_RD() & BM_POWER_STS_DC_OK))
		;

	HW_POWER_VDDACTRL_WR(reg_vdda);
	WAIT_CYCLE(WAIT_DC_OK_CYCLES)
	while (!(HW_POWER_STS_RD() & BM_POWER_STS_DC_OK))
		;

	HW_POWER_VDDIOCTRL_WR(reg_vddio);
	WAIT_CYCLE(WAIT_DC_OK_CYCLES)
	while (!(HW_POWER_STS_RD() & BM_POWER_STS_DC_OK))
		;


	/* Enable PLL */
	HW_CLKCTRL_PLLCTRL0_SET(BM_CLKCTRL_PLLCTRL0_POWER);
	/* Ungating EMI CLock */
	HW_CLKCTRL_EMI_CLR(BM_CLKCTRL_EMI_CLKGATE);

	/* LeaveSelfrefreshMode */
	HW_DRAM_CTL08_CLR(BM_DRAM_CTL08_SREFRESH);
	WAIT_CYCLE(WAIT_DC_OK_CYCLES)
}

static inline void do_standby(void)
{
	void (*stmp37xx_cpu_standby_ptr) (void);
	struct clk *cpu_clk;
	struct clk *osc_clk;
	struct clk *pll_clk;
	struct clk *hbus_clk;
	struct clk *cpu_parent = NULL;
	int cpu_rate = 0;
	int hbus_rate = 0;
	int i, pending_irq;
	u32 reg_clkctrl_clkseq, reg_clkctrl_xtal;

	/*
	 * 1) switch clock domains from PLL to 24MHz
	 * 2) lower voltage (TODO)
	 * 3) switch EMI to 24MHz and turn PLL off (done in sleep.S)
	 */

	/* save portion of SRAM to be used by suspend function. */
	memcpy(saved_sram, (void *)STMP3XXX_OCRAM_VA_BASE,
			stmp_standby_alloc_sz);

	/* make sure SRAM copy gets physically written into SDRAM.
	 * SDRAM will be placed into self-refresh during power down
	 */
	flush_cache_all();

	/* copy suspend function into SRAM */
	memcpy((void *)STMP3XXX_OCRAM_VA_BASE, (void*)stmp378x_standby,
			MAX_POWEROFF_CODE_SIZE);

	/* now switch the CPU to ref_xtal */
	cpu_clk = clk_get(NULL, "cpu");
	osc_clk = clk_get(NULL, "osc_24M");
	pll_clk = clk_get(NULL, "pll");
	hbus_clk = clk_get(NULL, "hclk");

	if (!IS_ERR(cpu_clk) && !IS_ERR(osc_clk)) {
		cpu_rate = clk_get_rate(cpu_clk);
		cpu_parent = cpu_clk->parent;
		hbus_rate = clk_get_rate(hbus_clk);
		clk_set_parent(cpu_clk, osc_clk);
	}

	local_irq_disable();
	local_fiq_disable();

	stmp3xxx_dma_suspend();
	stmp3xxx_suspend_timer();

	HW_POWER_CTRL_SET(BM_POWER_CTRL_ENIRQ_PSWITCH);
	HW_ICOLL_INTERRUPTn_SET(IRQ_VDD5V, BM_ICOLL_INTERRUPTn_ENABLE);

	/* clear pending interrupt, if any */
	for (i = 0; i < PENDING_IRQ_RETRY; i++) {
		pending_irq = HW_ICOLL_STAT_RD() & 0x7f;
		if (pending_irq == 0x7f)
			break;
		pr_info("irqn = %u\n", pending_irq);
		/* Tell ICOLL to release IRQ line */
		HW_ICOLL_VECTOR_WR(0x0);
		/* ACK current interrupt */
		HW_ICOLL_LEVELACK_WR(BV_ICOLL_LEVELACK_IRQLEVELACK__LEVEL0);
		/* Barrier */
		(void) HW_ICOLL_STAT_RD();
	}

	reg_clkctrl_clkseq = HW_CLKCTRL_CLKSEQ_RD();

	HW_CLKCTRL_CLKSEQ_SET(BM_CLKCTRL_CLKSEQ_BYPASS_ETM |
	BM_CLKCTRL_CLKSEQ_BYPASS_SSP |
	BM_CLKCTRL_CLKSEQ_BYPASS_GPMI |
	BM_CLKCTRL_CLKSEQ_BYPASS_IR |
	BM_CLKCTRL_CLKSEQ_BYPASS_PIX|
	BM_CLKCTRL_CLKSEQ_BYPASS_SAIF);

	reg_clkctrl_xtal = HW_CLKCTRL_XTAL_RD();

	HW_CLKCTRL_XTAL_SET(BM_CLKCTRL_XTAL_FILT_CLK24M_GATE |
	BM_CLKCTRL_XTAL_PWM_CLK24M_GATE  |
	BM_CLKCTRL_XTAL_DRI_CLK24M_GATE);

	/* do suspend */
	stmp37xx_cpu_standby_ptr = (void *)STMP3XXX_OCRAM_VA_BASE;

	stmp37xx_cpu_standby_ptr();

	HW_CLKCTRL_CLKSEQ_WR(reg_clkctrl_clkseq);
	HW_CLKCTRL_XTAL_WR(reg_clkctrl_xtal);

	pr_info("wakeup irq source = %d\n", HW_ICOLL_STAT_RD());
	saved_sleep_state = 0;  /* waking from standby */
	HW_POWER_CTRL_CLR(BM_POWER_CTRL_PSWITCH_IRQ);
	stmp3xxx_resume_timer();
	stmp3xxx_dma_resume();

	local_fiq_enable();
	local_irq_enable();

	if (cpu_parent) {
		clk_set_parent(cpu_clk, cpu_parent);
		clk_set_rate(cpu_clk, cpu_rate);
		clk_set_rate(hbus_clk, hbus_rate);
	}

	clk_put(hbus_clk);
	clk_put(pll_clk);
	clk_put(osc_clk);
	clk_put(cpu_clk);

	/* restoring portion of SRAM that was used by suspend function */
	memcpy((void *)STMP3XXX_OCRAM_VA_BASE, saved_sram,
			stmp_standby_alloc_sz);
}

static u32 clk_regs[] = {
		HW_CLKCTRL_PLLCTRL0_ADDR,
		HW_CLKCTRL_XTAL_ADDR,
		HW_CLKCTRL_PIX_ADDR,
		HW_CLKCTRL_SSP_ADDR,
		HW_CLKCTRL_GPMI_ADDR,
		HW_CLKCTRL_FRAC_ADDR,
		HW_CLKCTRL_CLKSEQ_ADDR,
};

static noinline void do_mem(void)
{
	void (*stmp37xx_cpu_suspend_ptr) (u32);
	struct sleep_data saved_context;
	int i;
	struct clk *cpu_clk;
	struct clk *osc_clk;
	struct clk *pll_clk;
	struct clk *hbus_clk;
	int cpu_rate = 0;
	int hbus_rate = 0;

	saved_context.fingerprint = SLEEP_DATA_FINGERPRINT;

	saved_context.old_c00 = __raw_readl(0xC0000000);
	saved_context.old_c04 = __raw_readl(0xC0000004);
	__raw_writel((u32)&saved_context, (void *)0xC0000000);

	local_irq_disable();
	local_fiq_disable();

	stmp3xxx_dma_suspend();
	stmp3xxx_suspend_timer();

	/* clocks */
	for (i = 0; i < ARRAY_SIZE(clk_regs); i++)
		saved_context.clks[i] =
				__raw_readl(clk_regs[i]);

	/* interrupt collector */
	saved_context.icoll_ctrl = HW_ICOLL_CTRL_RD();
	if (machine_is_stmp37xx()) {
#ifdef CONFIG_MACH_STMP37XX
		for (i = 0; i < 16; i++)
			saved_context.icoll.prio[i] = HW_ICOLL_PRIORITYn_RD(i);
#endif
	} else if (machine_is_stmp378x()) {
#ifdef CONFIG_MACH_STMP378X
		for (i = 0; i < 128; i++)
			saved_context.icoll.intr[i] = HW_ICOLL_INTERRUPTn_RD(i);
#endif
	}

	/* save pinmux state */
	for (i = 0; i < 0x100; i++)
		saved_context.pinmux[i] =
				__raw_readl(REGS_PINCTRL_BASE + (i<<4));

	cpu_clk = clk_get(NULL, "cpu");
	osc_clk = clk_get(NULL, "osc_24M");
	pll_clk = clk_get(NULL, "pll");
	hbus_clk = clk_get(NULL, "hclk");

	cpu_rate = clk_get_rate(cpu_clk);
	hbus_rate = clk_get_rate(hbus_clk);

	/* save portion of SRAM to be used by suspend function. */
	memcpy(saved_sram, (void *)STMP3XXX_OCRAM_VA_BASE, stmp_s2ram_alloc_sz);

	/* set the PERSISTENT_SLEEP_BIT for bootloader */
	HW_RTC_PERSISTENT1_SET(1 << 10); /* XXX: temp */

	/*
	 * make sure SRAM copy gets physically written into SDRAM.
	 * SDRAM will be placed into self-refresh during power down
	 */
	flush_cache_all();

	/*copy suspend function into SRAM */
	memcpy((void *)STMP3XXX_OCRAM_VA_BASE, stmp37xx_cpu_suspend,
			stmp_s2ram_alloc_sz);

	/* do suspend */
	stmp37xx_cpu_suspend_ptr = (void *)STMP3XXX_OCRAM_VA_BASE;
	stmp37xx_cpu_suspend_ptr(0);

	saved_sleep_state = 1;	/* waking from non-standby state */

	/* restoring portion of SRAM that was used by suspend function */
	memcpy((void *)STMP3XXX_OCRAM_VA_BASE, saved_sram, stmp_s2ram_alloc_sz);

	/* clocks */
	for (i = 0; i < ARRAY_SIZE(clk_regs); i++)
		__raw_writel(saved_context.clks[i],
			clk_regs[i]);

	/* interrupt collector */
	HW_ICOLL_CTRL_WR(saved_context.icoll_ctrl);
	if (machine_is_stmp37xx()) {
#ifdef CONFIG_MACH_STMP37XX
		for (i = 0; i < 16; i++)
			HW_ICOLL_PRIORITYn_WR(i, saved_context.icoll.prio[i]);
#endif
	} else if (machine_is_stmp378x()) {
#ifdef CONFIG_MACH_STMP378X
		for (i = 0; i < 128; i++)
			HW_ICOLL_INTERRUPTn_WR(i, saved_context.icoll.intr[i]);
#endif
	}

	/* restore pinmux state */
	for (i = 0; i < 0x100; i++)
		__raw_writel(saved_context.pinmux[i],
				REGS_PINCTRL_BASE + (i<<4));

	clk_set_parent(cpu_clk, cpu_clk->parent);
	clk_set_rate(cpu_clk, cpu_rate);
	clk_set_rate(hbus_clk, hbus_rate);

	__raw_writel(saved_context.old_c00, 0xC0000000);
	__raw_writel(saved_context.old_c04, 0xC0000004);

	clk_put(hbus_clk);
	clk_put(pll_clk);
	clk_put(osc_clk);
	clk_put(cpu_clk);

	stmp3xxx_resume_timer();
	stmp3xxx_dma_resume();

	local_fiq_enable();
	local_irq_enable();
}

static int stmp37xx_pm_enter(suspend_state_t state)
{
	switch (state) {
	case PM_SUSPEND_STANDBY:
		do_standby();
		break;
	case PM_SUSPEND_MEM:
		do_mem();
		break;
	}
	return 0;
}

static int stmp37xx_pm_valid(suspend_state_t state)
{
	return (state == PM_SUSPEND_STANDBY) ||
	       (state == PM_SUSPEND_MEM);
}

static suspend_state_t saved_state;

static int stmp37xx_pm_begin(suspend_state_t state)
{
	saved_state = state;
	return 0;
}

static void stmp37xx_pm_end(void)
{
	/*XXX: Nothing to do */
}

suspend_state_t stmp37xx_pm_get_target(void)
{
	return saved_state;
}
EXPORT_SYMBOL(stmp37xx_pm_get_target);

/**
 * stmp37xx_pm_get_sleep_state - get sleep state we waking from
 *
 * returns boolean: 0 if waking up from standby, 1 otherwise
 */
int stmp37xx_pm_sleep_was_deep(void)
{
	return saved_sleep_state;
}
EXPORT_SYMBOL(stmp37xx_pm_sleep_was_deep);

static struct platform_suspend_ops stmp37xx_suspend_ops = {
	.enter	= stmp37xx_pm_enter,
	.valid	= stmp37xx_pm_valid,
	.begin	= stmp37xx_pm_begin,
	.end	= stmp37xx_pm_end,
};

void stmp37xx_pm_idle(void)
{
	local_irq_disable();
	local_fiq_disable();
	if (need_resched()) {
		local_fiq_enable();
		local_irq_enable();
		return;
	}

	HW_CLKCTRL_CPU_SET(1<<12);
	__asm__ __volatile__ ("mcr	p15, 0, r0, c7, c0, 4");

	local_fiq_enable();
	local_irq_enable();
}

static void stmp37xx_pm_power_off(void)
{
	HW_POWER_RESET_WR((0x3e77 << 16) | 1);
}

struct stmp37xx_pswitch_state {
	int dev_running;
};

static DECLARE_COMPLETION(suspend_request);

static int suspend_thread_fn(void *data)
{
	while (1) {
		wait_for_completion(&suspend_request);
		pm_suspend(PM_SUSPEND_STANDBY);
	}
	return 0;
}

static struct stmp37xx_pswitch_state pswitch_state = {
	.dev_running = 0,
};

static irqreturn_t pswitch_interrupt(int irq, void *dev)
{
	int pin_value, i;

	/* check if irq by pswitch */
	if (!(HW_POWER_CTRL_RD() & BM_POWER_CTRL_PSWITCH_IRQ))
		return IRQ_HANDLED;
	for (i = 0; i < 3000; i++) {
		pin_value = HW_POWER_STS_RD() &
			BF_POWER_STS_PSWITCH(0x1);
		if (pin_value == 0)
			break;
		mdelay(1);
	}
	if (i < 3000) {
		pr_info("pswitch goto suspend\n");
		complete(&suspend_request);
	} else {
		pr_info("release pswitch to power down\n");
		for (i = 0; i < 5000; i++) {
			pin_value = HW_POWER_STS_RD() &
				BF_POWER_STS_PSWITCH(0x1);
			if (pin_value == 0)
				break;
			mdelay(1);
		}
		pr_info("pswitch power down\n");
		stmp37xx_pm_power_off();
	}
	HW_POWER_CTRL_CLR(BM_POWER_CTRL_PSWITCH_IRQ);
	return IRQ_HANDLED;
}

static struct irqaction pswitch_irq = {
	.name		= "pswitch",
	.flags		= IRQF_DISABLED | IRQF_SHARED,
	.handler	= pswitch_interrupt,
	.dev_id		= &pswitch_state,
};

static void init_pswitch(void)
{
	kthread_run(suspend_thread_fn, NULL, "pswitch");
	HW_POWER_CTRL_CLR(BM_POWER_CTRL_PSWITCH_IRQ);
	HW_POWER_CTRL_SET(BM_POWER_CTRL_POLARITY_PSWITCH |
		BM_POWER_CTRL_ENIRQ_PSWITCH);
	HW_POWER_CTRL_CLR(BM_POWER_CTRL_PSWITCH_IRQ);
	setup_irq(IRQ_VDD5V, &pswitch_irq);
}

static int __init stmp37xx_pm_init(void)
{
	saved_sram = kmalloc(0x4000, GFP_ATOMIC);
	if (!saved_sram) {
		printk(KERN_ERR
		 "PM Suspend: can't allocate memory to save portion of SRAM\n");
		return -ENOMEM;
	}

	pm_power_off = stmp37xx_pm_power_off;
	pm_idle = stmp37xx_pm_idle;
	suspend_set_ops(&stmp37xx_suspend_ops);
	init_pswitch();
	return 0;
}

late_initcall(stmp37xx_pm_init);
