/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 *
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


#include <linux/suspend.h>
#include <linux/rtc.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kthread.h>
#include <linux/slab.h>

#include <asm/cacheflush.h>
#include <asm/mach-types.h>

#include <asm/mach/time.h>

#include <mach/hardware.h>
#include <mach/dma.h>
#include <mach/regs-rtc.h>
#include "regs-clkctrl.h"
#include "regs-pinctrl.h"
#include <mach/regs-power.h>
#include <mach/regs-pwm.h>
#include <mach/regs-rtc.h>
#include <mach/../../regs-icoll.h>
#include "regs-dram.h"

#include "sleep.h"

#define PENDING_IRQ_RETRY 100
static void *saved_sram;
static int saved_sleep_state;

#define WAIT_DC_OK_CYCLES 24000
#define WAIT_CYCLE(n) for (i = 0; i < n; i++);
#define LOWER_VDDIO 10
#define LOWER_VDDA 9
#define LOWER_VDDD 8
#define MAX_POWEROFF_CODE_SIZE (6 * 1024)
#define REGS_CLKCTRL_BASE IO_ADDRESS(CLKCTRL_PHYS_ADDR)

static void mx23_standby(void)
{
	int i;
	u32 reg_vddd, reg_vdda, reg_vddio;
	/* DDR EnterSelfrefreshMode */
	__raw_writel(
	BF_DRAM_CTL16_LOWPOWER_AUTO_ENABLE(0x2) |
	__raw_readl(IO_ADDRESS(DRAM_PHYS_ADDR)
		+ HW_DRAM_CTL16),
	IO_ADDRESS(DRAM_PHYS_ADDR) + HW_DRAM_CTL16);

	__raw_writel(
	BF_DRAM_CTL16_LOWPOWER_CONTROL(0x2) |
	__raw_readl(IO_ADDRESS(DRAM_PHYS_ADDR)
		+ HW_DRAM_CTL16),
	IO_ADDRESS(DRAM_PHYS_ADDR) + HW_DRAM_CTL16);
	/* Gating EMI CLock */
	__raw_writel(BM_CLKCTRL_EMI_CLKGATE |
			__raw_readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_EMI),
			REGS_CLKCTRL_BASE + HW_CLKCTRL_EMI);

	/* Disable PLL */
	__raw_writel(BM_CLKCTRL_PLLCTRL0_POWER,
		REGS_CLKCTRL_BASE + HW_CLKCTRL_PLLCTRL0_CLR);

	/* Reduce the VDDIO (3.050 volt) */
	reg_vddio = __raw_readl(REGS_POWER_BASE + HW_POWER_VDDIOCTRL);
	__raw_writel(reg_vddio | BM_POWER_VDDIOCTRL_BO_OFFSET,
		REGS_POWER_BASE + HW_POWER_VDDIOCTRL);
	__raw_writel((__raw_readl(REGS_POWER_BASE + HW_POWER_VDDIOCTRL) &
		~BM_POWER_VDDIOCTRL_TRG) | LOWER_VDDIO,
		REGS_POWER_BASE + HW_POWER_VDDIOCTRL);
	WAIT_CYCLE(WAIT_DC_OK_CYCLES)

	while (!(__raw_readl(REGS_POWER_BASE + HW_POWER_STS) &
		BM_POWER_STS_DC_OK))
		;

	/* Reduce VDDA 1.725volt */
	reg_vdda = __raw_readl(REGS_POWER_BASE + HW_POWER_VDDACTRL);
	__raw_writel(reg_vdda | BM_POWER_VDDACTRL_BO_OFFSET,
		REGS_POWER_BASE + HW_POWER_VDDACTRL);
	__raw_writel((__raw_readl(REGS_POWER_BASE + HW_POWER_VDDACTRL) &
		~BM_POWER_VDDACTRL_TRG) | LOWER_VDDA,
		REGS_POWER_BASE + HW_POWER_VDDACTRL);
	WAIT_CYCLE(WAIT_DC_OK_CYCLES)

	/* wait for DC_OK */
	while (!(__raw_readl(REGS_POWER_BASE + HW_POWER_STS) &
		BM_POWER_STS_DC_OK))
		;

	/* Reduce VDDD 1.000 volt */
	reg_vddd = __raw_readl(REGS_POWER_BASE + HW_POWER_VDDDCTRL);
	__raw_writel(reg_vddd | BM_POWER_VDDDCTRL_BO_OFFSET,
		REGS_POWER_BASE + HW_POWER_VDDDCTRL);
	__raw_writel((__raw_readl(REGS_POWER_BASE + HW_POWER_VDDDCTRL) &
		~BM_POWER_VDDDCTRL_TRG) | LOWER_VDDD,
		REGS_POWER_BASE + HW_POWER_VDDDCTRL);
	WAIT_CYCLE(WAIT_DC_OK_CYCLES)

	while (!(__raw_readl(REGS_POWER_BASE + HW_POWER_STS) &
		BM_POWER_STS_DC_OK))
		;

	/* optimize the DCDC loop gain */
	__raw_writel((__raw_readl(REGS_POWER_BASE + HW_POWER_LOOPCTRL) &
		~BM_POWER_LOOPCTRL_EN_RCSCALE),
		REGS_POWER_BASE + HW_POWER_LOOPCTRL);
	__raw_writel((__raw_readl(REGS_POWER_BASE + HW_POWER_LOOPCTRL) &
		~BM_POWER_LOOPCTRL_DC_R) |
		(2<<BP_POWER_LOOPCTRL_DC_R),
		REGS_POWER_BASE + HW_POWER_LOOPCTRL);

	/* half the fets */
	__raw_writel(BM_POWER_MINPWR_HALF_FETS,
		REGS_POWER_BASE + HW_POWER_MINPWR_SET);
	__raw_writel(BM_POWER_MINPWR_DOUBLE_FETS,
		REGS_POWER_BASE + HW_POWER_MINPWR_CLR);

	__raw_writel(BM_POWER_LOOPCTRL_CM_HYST_THRESH,
			REGS_POWER_BASE + HW_POWER_LOOPCTRL_CLR);
	__raw_writel(BM_POWER_LOOPCTRL_EN_CM_HYST,
			REGS_POWER_BASE + HW_POWER_LOOPCTRL_CLR);
	__raw_writel(BM_POWER_LOOPCTRL_EN_DF_HYST,
			REGS_POWER_BASE + HW_POWER_LOOPCTRL_CLR);


	/* enable PFM */
	__raw_writel(BM_POWER_LOOPCTRL_HYST_SIGN,
		REGS_POWER_BASE + HW_POWER_LOOPCTRL_SET);
	__raw_writel(BM_POWER_MINPWR_EN_DC_PFM,
		REGS_POWER_BASE + HW_POWER_MINPWR_SET);

	__raw_writel(BM_CLKCTRL_CPU_INTERRUPT_WAIT,
		REGS_CLKCTRL_BASE + HW_CLKCTRL_CPU_SET);
	/* Power off ... */
	asm("mcr     p15, 0, r2, c7, c0, 4");
	__raw_writel(BM_CLKCTRL_CPU_INTERRUPT_WAIT,
			REGS_CLKCTRL_BASE + HW_CLKCTRL_CPU_CLR);

	/* Enable PLL */
	__raw_writel(BM_CLKCTRL_PLLCTRL0_POWER,
		REGS_CLKCTRL_BASE + HW_CLKCTRL_PLLCTRL0_SET);

	/* restore the DCDC parameter */

	__raw_writel(BM_POWER_MINPWR_EN_DC_PFM,
			REGS_POWER_BASE + HW_POWER_MINPWR_CLR);
	__raw_writel(BM_POWER_LOOPCTRL_HYST_SIGN,
			REGS_POWER_BASE + HW_POWER_LOOPCTRL_CLR);
	__raw_writel(BM_POWER_LOOPCTRL_EN_DF_HYST,
			REGS_POWER_BASE + HW_POWER_LOOPCTRL_SET);
	__raw_writel(BM_POWER_LOOPCTRL_EN_CM_HYST,
			REGS_POWER_BASE + HW_POWER_LOOPCTRL_SET);
	__raw_writel(BM_POWER_LOOPCTRL_CM_HYST_THRESH,
			REGS_POWER_BASE + HW_POWER_LOOPCTRL_SET);

	__raw_writel((__raw_readl(REGS_POWER_BASE + HW_POWER_LOOPCTRL) &
		~BM_POWER_LOOPCTRL_DC_R) |
		(2<<BP_POWER_LOOPCTRL_DC_R),
		REGS_POWER_BASE + HW_POWER_LOOPCTRL);
	__raw_writel((__raw_readl(REGS_POWER_BASE + HW_POWER_LOOPCTRL) &
		~BM_POWER_LOOPCTRL_EN_RCSCALE) |
		(3 << BP_POWER_LOOPCTRL_EN_RCSCALE),
		REGS_POWER_BASE + HW_POWER_LOOPCTRL);

	/* double the fets */
	__raw_writel(BM_POWER_MINPWR_HALF_FETS,
		REGS_POWER_BASE + HW_POWER_MINPWR_CLR);
	__raw_writel(BM_POWER_MINPWR_DOUBLE_FETS,
		REGS_POWER_BASE + HW_POWER_MINPWR_SET);


	/* Restore VDDD */
	__raw_writel(reg_vddd, REGS_POWER_BASE + HW_POWER_VDDDCTRL);

	WAIT_CYCLE(WAIT_DC_OK_CYCLES)
	while (!(__raw_readl(REGS_POWER_BASE + HW_POWER_STS) &
		BM_POWER_STS_DC_OK))
		;

	__raw_writel(reg_vdda, REGS_POWER_BASE + HW_POWER_VDDACTRL);
	WAIT_CYCLE(WAIT_DC_OK_CYCLES)
	while (!(__raw_readl(REGS_POWER_BASE + HW_POWER_STS) &
		BM_POWER_STS_DC_OK))
		;

	__raw_writel(reg_vddio, REGS_POWER_BASE + HW_POWER_VDDIOCTRL);
	WAIT_CYCLE(WAIT_DC_OK_CYCLES)
	while (!(__raw_readl(REGS_POWER_BASE + HW_POWER_STS) &
		BM_POWER_STS_DC_OK))
		;


	/* Ungating EMI CLock */
	__raw_writel(~BM_CLKCTRL_EMI_CLKGATE &
			__raw_readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_EMI),
			REGS_CLKCTRL_BASE + HW_CLKCTRL_EMI);

	/* LeaveSelfrefreshMode */
	__raw_writel(
	(~BF_DRAM_CTL16_LOWPOWER_CONTROL(0x2)) &
	__raw_readl(IO_ADDRESS(DRAM_PHYS_ADDR)
		+ HW_DRAM_CTL16),
	IO_ADDRESS(DRAM_PHYS_ADDR) + HW_DRAM_CTL16);

	__raw_writel(
	(~BF_DRAM_CTL16_LOWPOWER_AUTO_ENABLE(0x2)) &
	__raw_readl(IO_ADDRESS(DRAM_PHYS_ADDR)
		+ HW_DRAM_CTL16),
	IO_ADDRESS(DRAM_PHYS_ADDR) + HW_DRAM_CTL16);
}

static inline void do_standby(void)
{
	void (*mx23_cpu_standby_ptr) (void);
	struct clk *cpu_clk;
	struct clk *osc_clk;
	struct clk *pll_clk;
	struct clk *hbus_clk;
	struct clk *cpu_parent = NULL;
	int cpu_rate = 0;
	int hbus_rate = 0;
	u32 reg_clkctrl_clkseq, reg_clkctrl_xtal;
	unsigned long iram_phy_addr;
	void *iram_virtual_addr;

	/*
	 * 1) switch clock domains from PLL to 24MHz
	 * 2) lower voltage (TODO)
	 * 3) switch EMI to 24MHz and turn PLL off (done in sleep.S)
	 */


	/* make sure SRAM copy gets physically written into SDRAM.
	 * SDRAM will be placed into self-refresh during power down
	 */
	flush_cache_all();
	iram_virtual_addr = iram_alloc(MAX_POWEROFF_CODE_SIZE, &iram_phy_addr);
	if (iram_virtual_addr == NULL) {
		pr_info("can not get iram for suspend\n");
		return;
	}

	/* copy suspend function into SRAM */
	memcpy(iram_virtual_addr, mx23_standby,
			MAX_POWEROFF_CODE_SIZE);

	/* now switch the CPU to ref_xtal */
	cpu_clk = clk_get(NULL, "cpu");
	osc_clk = clk_get(NULL, "ref_xtal");
	pll_clk = clk_get(NULL, "pll.0");
	hbus_clk = clk_get(NULL, "h");

	if (!IS_ERR(cpu_clk) && !IS_ERR(osc_clk)) {
		cpu_rate = clk_get_rate(cpu_clk);
		cpu_parent = clk_get_parent(cpu_clk);
		hbus_rate = clk_get_rate(hbus_clk);
		clk_set_parent(cpu_clk, osc_clk);
	}

	local_fiq_disable();
	mxs_nomatch_suspend_timer();

	__raw_writel(BM_POWER_CTRL_ENIRQ_PSWITCH,
		REGS_POWER_BASE + HW_POWER_CTRL_SET);

	reg_clkctrl_clkseq = __raw_readl(REGS_CLKCTRL_BASE +
		HW_CLKCTRL_CLKSEQ);

	__raw_writel(BM_CLKCTRL_CLKSEQ_BYPASS_ETM |
		BM_CLKCTRL_CLKSEQ_BYPASS_SSP |
		BM_CLKCTRL_CLKSEQ_BYPASS_GPMI |
		BM_CLKCTRL_CLKSEQ_BYPASS_IR |
		BM_CLKCTRL_CLKSEQ_BYPASS_PIX |
		BM_CLKCTRL_CLKSEQ_BYPASS_SAIF,
		REGS_CLKCTRL_BASE + HW_CLKCTRL_CLKSEQ_SET);

	reg_clkctrl_xtal = __raw_readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_XTAL);

	__raw_writel(reg_clkctrl_xtal | BM_CLKCTRL_XTAL_FILT_CLK24M_GATE |
		BM_CLKCTRL_XTAL_PWM_CLK24M_GATE |
		BM_CLKCTRL_XTAL_DRI_CLK24M_GATE,
		REGS_CLKCTRL_BASE + HW_CLKCTRL_XTAL);

	/* do suspend */
	mx23_cpu_standby_ptr = iram_virtual_addr;
	mx23_cpu_standby_ptr();

	__raw_writel(reg_clkctrl_clkseq, REGS_CLKCTRL_BASE + HW_CLKCTRL_CLKSEQ);
	__raw_writel(reg_clkctrl_xtal, REGS_CLKCTRL_BASE + HW_CLKCTRL_XTAL);

	saved_sleep_state = 0;  /* waking from standby */
	__raw_writel(BM_POWER_CTRL_PSWITCH_IRQ,
		REGS_POWER_BASE + HW_POWER_CTRL_CLR);
	mxs_nomatch_resume_timer();

	local_fiq_enable();

	if (cpu_parent) {
		clk_set_parent(cpu_clk, cpu_parent);
		clk_set_rate(cpu_clk, cpu_rate);
		clk_set_rate(hbus_clk, hbus_rate);
	}

	clk_put(hbus_clk);
	clk_put(pll_clk);
	clk_put(osc_clk);
	clk_put(cpu_clk);

	iram_free(iram_phy_addr, MAX_POWEROFF_CODE_SIZE);
}

static u32 clk_regs[] = {
		HW_CLKCTRL_PLLCTRL0,
		HW_CLKCTRL_XTAL,
		HW_CLKCTRL_PIX,
		HW_CLKCTRL_SSP,
		HW_CLKCTRL_GPMI,
		HW_CLKCTRL_FRAC,
		HW_CLKCTRL_CLKSEQ,
};

static noinline void do_mem(void)
{
	unsigned long iram_phy_addr;
	void *iram_virtual_addr;
	void (*mx23_cpu_suspend_ptr) (u32);
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

	iram_virtual_addr = iram_alloc(MAX_POWEROFF_CODE_SIZE, &iram_phy_addr);
	if (iram_virtual_addr == NULL) {
		pr_info("can not get iram for suspend\n");
		return;
	}

	local_irq_disable();
	local_fiq_disable();

	/* mxs_dma_suspend(); */
	mxs_nomatch_suspend_timer();

	/* clocks */
	for (i = 0; i < ARRAY_SIZE(clk_regs); i++)
		saved_context.clks[i] =
				__raw_readl(clk_regs[i]);

	/* interrupt collector */
	/*
	saved_context.icoll_ctrl =
		__raw_readl(REGS_ICOLL_BASE + HW_ICOLL_CTRL);
	if (machine_is_mx23()) {
#ifdef CONFIG_MACH_MX23
		for (i = 0; i < 16; i++)
			saved_context.icoll.prio[i] =
			__raw_readl(REGS_ICOLL_BASE + HW_ICOLL_PRIORITYn(i));
#endif
	} else if (machine_is_mx23()) {
#ifdef CONFIG_MACH_STMP378X
		for (i = 0; i < 128; i++)
			saved_context.icoll.intr[i] =
			__raw_readl(REGS_ICOLL_BASE + HW_ICOLL_INTERRUPTn(i));
#endif
	}
	*/

	/* save pinmux state */
	for (i = 0; i < 0x100; i++)
		saved_context.pinmux[i] =
			__raw_readl(IO_ADDRESS(PINCTRL_PHYS_ADDR) + (i<<4));

	cpu_clk = clk_get(NULL, "cpu");
	osc_clk = clk_get(NULL, "osc_24M");
	pll_clk = clk_get(NULL, "pll");
	hbus_clk = clk_get(NULL, "hclk");

	cpu_rate = clk_get_rate(cpu_clk);
	hbus_rate = clk_get_rate(hbus_clk);


	/* set the PERSISTENT_SLEEP_BIT for bootloader */
	__raw_writel(1 << 10,
		IO_ADDRESS(RTC_PHYS_ADDR) + HW_RTC_PERSISTENT1_SET);
	/* XXX: temp */

	/*
	 * make sure SRAM copy gets physically written into SDRAM.
	 * SDRAM will be placed into self-refresh during power down
	 */
	flush_cache_all();

	/*copy suspend function into SRAM */
	memcpy(iram_virtual_addr, mx23_cpu_suspend,
		MAX_POWEROFF_CODE_SIZE);

	/* do suspend */
	mx23_cpu_suspend_ptr = (void *)MX23_OCRAM_BASE;
	mx23_cpu_suspend_ptr(0);

	saved_sleep_state = 1;	/* waking from non-standby state */


	/* clocks */
	for (i = 0; i < ARRAY_SIZE(clk_regs); i++)
		__raw_writel(saved_context.clks[i],
			clk_regs[i]);

	/* interrupt collector */
/*
	__raw_writel(saved_context.icoll_ctrl, REGS_ICOLL_BASE + HW_ICOLL_CTRL);
	if (machine_is_mx23()) {
#ifdef CONFIG_MACH_MX23
		for (i = 0; i < 16; i++)
			__raw_writel(saved_context.icoll.prio[i],
			REGS_ICOLL_BASE + HW_ICOLL_PRIORITYn(i));
#endif
	} else if (machine_is_mx23()) {
#ifdef CONFIG_MACH_STMP378X
		for (i = 0; i < 128; i++)
			__raw_writel(saved_context.icoll.intr[i],
			REGS_ICOLL_BASE + HW_ICOLL_INTERRUPTn(i));
#endif
	}
*/

	/* restore pinmux state */
	for (i = 0; i < 0x100; i++)
		__raw_writel(saved_context.pinmux[i],
				IO_ADDRESS(PINCTRL_PHYS_ADDR) + (i<<4));

	clk_set_rate(cpu_clk, cpu_rate);
	clk_set_rate(hbus_clk, hbus_rate);

	__raw_writel(saved_context.old_c00, 0xC0000000);
	__raw_writel(saved_context.old_c04, 0xC0000004);

	clk_put(hbus_clk);
	clk_put(pll_clk);
	clk_put(osc_clk);
	clk_put(cpu_clk);

	mxs_nomatch_resume_timer();
	/* mxs_dma_resume(); */

	iram_free(iram_phy_addr, MAX_POWEROFF_CODE_SIZE);
	local_fiq_enable();
	local_irq_enable();
}

static int mx23_pm_enter(suspend_state_t state)
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

static int mx23_pm_valid(suspend_state_t state)
{
	return (state == PM_SUSPEND_STANDBY) ||
	       (state == PM_SUSPEND_MEM);
}

static suspend_state_t saved_state;

static int mx23_pm_begin(suspend_state_t state)
{
	saved_state = state;
	return 0;
}

static void mx23_pm_end(void)
{
	/*XXX: Nothing to do */
}

suspend_state_t mx23_pm_get_target(void)
{
	return saved_state;
}
EXPORT_SYMBOL(mx23_pm_get_target);

/**
 * mx23_pm_get_sleep_state - get sleep state we waking from
 *
 * returns boolean: 0 if waking up from standby, 1 otherwise
 */
int mx23_pm_sleep_was_deep(void)
{
	return saved_sleep_state;
}
EXPORT_SYMBOL(mx23_pm_sleep_was_deep);

static struct platform_suspend_ops mx23_suspend_ops = {
	.enter	= mx23_pm_enter,
	.valid	= mx23_pm_valid,
	.begin	= mx23_pm_begin,
	.end	= mx23_pm_end,
};

void mx23_pm_idle(void)
{
	local_irq_disable();
	local_fiq_disable();
	if (need_resched()) {
		local_fiq_enable();
		local_irq_enable();
		return;
	}

	__raw_writel(1<<12, REGS_CLKCTRL_BASE + HW_CLKCTRL_CPU_SET);
	__asm__ __volatile__ ("mcr	p15, 0, r0, c7, c0, 4");

	local_fiq_enable();
	local_irq_enable();
}

static void mx23_pm_power_off(void)
{
	__raw_writel((0x3e77 << 16) | 1, REGS_POWER_BASE + HW_POWER_RESET);
}

struct mx23_pswitch_state {
	int dev_running;
};

static DECLARE_COMPLETION(suspend_request);

static int suspend_thread_fn(void *data)
{
	while (1) {
		wait_for_completion_interruptible(&suspend_request);
		pm_suspend(PM_SUSPEND_STANDBY);
	}
	return 0;
}

static struct mx23_pswitch_state pswitch_state = {
	.dev_running = 0,
};

static irqreturn_t pswitch_interrupt(int irq, void *dev)
{
	int pin_value, i;

	/* check if irq by pswitch */
	if (!(__raw_readl(REGS_POWER_BASE + HW_POWER_CTRL) &
		BM_POWER_CTRL_PSWITCH_IRQ))
		return IRQ_HANDLED;
	for (i = 0; i < 3000; i++) {
		pin_value = __raw_readl(REGS_POWER_BASE + HW_POWER_STS) &
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
			pin_value = __raw_readl(REGS_POWER_BASE + HW_POWER_STS)
				& BF_POWER_STS_PSWITCH(0x1);
			if (pin_value == 0)
				break;
			mdelay(1);
		}
		pr_info("pswitch power down\n");
		mx23_pm_power_off();
	}
	__raw_writel(BM_POWER_CTRL_PSWITCH_IRQ,
		REGS_POWER_BASE + HW_POWER_CTRL_CLR);
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
	__raw_writel(BM_POWER_CTRL_PSWITCH_IRQ,
		REGS_POWER_BASE + HW_POWER_CTRL_CLR);
	__raw_writel(BM_POWER_CTRL_POLARITY_PSWITCH |
		BM_POWER_CTRL_ENIRQ_PSWITCH,
		REGS_POWER_BASE + HW_POWER_CTRL_SET);
	__raw_writel(BM_POWER_CTRL_PSWITCH_IRQ,
		REGS_POWER_BASE + HW_POWER_CTRL_CLR);
	setup_irq(IRQ_VDD5V, &pswitch_irq);
}

static int __init mx23_pm_init(void)
{
	saved_sram = kmalloc(0x4000, GFP_ATOMIC);
	if (!saved_sram) {
		printk(KERN_ERR
		 "PM Suspend: can't allocate memory to save portion of SRAM\n");
		return -ENOMEM;
	}

	pm_power_off = mx23_pm_power_off;
	pm_idle = mx23_pm_idle;
	suspend_set_ops(&mx23_suspend_ops);
	init_pswitch();
	return 0;
}

late_initcall(mx23_pm_init);
