/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
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
#include <mach/regs-power.h>
#include <mach/regs-pwm.h>
#include <mach/regs-rtc.h>
#include <mach/../../regs-icoll.h>
#include "regs-dram.h"
#include "mx28_pins.h"
#include "mx28evk.h"

#include "sleep.h"

#define PENDING_IRQ_RETRY 100
static void *saved_sram;
static int saved_sleep_state;

#define WAIT_DC_OK_CYCLES 24000
#define WAIT_CYCLE(n) for (i = 0; i < n; i++);
#define LOWER_VDDIO 10
#define LOWER_VDDA 9
#define LOWER_VDDD 0xa
#define MAX_POWEROFF_CODE_SIZE (6 * 1024)
#define REGS_CLKCTRL_BASE IO_ADDRESS(CLKCTRL_PHYS_ADDR)
#define dbgc(ch) __raw_writel(ch, IO_ADDRESS(0x80074000));
inline void dbgnum(u32 num)
{
	dbgc((num / 1000) + '0');
	dbgc(((num%1000) / 100) + '0');
	dbgc(((num%100) / 10) + '0');
	dbgc((num%10) + '0');
	dbgc('\n');
}

static inline void do_standby(void)
{
	void (*mx28_cpu_standby_ptr) (void);
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
	int wakeupirq;
	mx28evk_enet_io_lowerpower_enter();
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
	memcpy(iram_virtual_addr, mx28_cpu_standby, mx28_standby_alloc_sz);

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
	} else
		pr_err("fail to get cpu clk\n");
	if (cpu_rate == 261818000)
		clk_set_rate(hbus_clk, 8727267);
	local_fiq_disable();

	__raw_writel(BM_POWER_CTRL_ENIRQ_PSWITCH,
		REGS_POWER_BASE + HW_POWER_CTRL_SET);

	reg_clkctrl_clkseq = __raw_readl(REGS_CLKCTRL_BASE +
		HW_CLKCTRL_CLKSEQ);

	reg_clkctrl_xtal = __raw_readl(REGS_CLKCTRL_BASE + HW_CLKCTRL_XTAL);


	/* do suspend */
	mx28_cpu_standby_ptr = iram_virtual_addr;

	mx28_cpu_standby_ptr();

	wakeupirq = __raw_readl(IO_ADDRESS(ICOLL_PHYS_ADDR) + HW_ICOLL_STAT);

	pr_info("wakeup irq = %d\n", wakeupirq);

	__raw_writel(reg_clkctrl_clkseq, REGS_CLKCTRL_BASE + HW_CLKCTRL_CLKSEQ);
	__raw_writel(reg_clkctrl_xtal, REGS_CLKCTRL_BASE + HW_CLKCTRL_XTAL);
	saved_sleep_state = 0;  /* waking from standby */
	__raw_writel(BM_POWER_CTRL_PSWITCH_IRQ,
		REGS_POWER_BASE + HW_POWER_CTRL_CLR);
	__raw_writel(BM_POWER_CTRL_ENIRQ_PSWITCH,
		REGS_POWER_BASE + HW_POWER_CTRL_SET);

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
	mx28evk_enet_io_lowerpower_exit();
}

static noinline void do_mem(void)
{
	/* TODO */
}

static int mx28_pm_enter(suspend_state_t state)
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

static int mx28_pm_valid(suspend_state_t state)
{
	return (state == PM_SUSPEND_STANDBY) ||
	       (state == PM_SUSPEND_MEM);
}

static suspend_state_t saved_state;

static int mx28_pm_begin(suspend_state_t state)
{
	saved_state = state;
	return 0;
}

static void mx28_pm_recover(void)
{
	/*
	 * The PSWITCH interrupt is enabled at do_standby, if the deivce
	 * suspend failed, the enable operation will not be executed, in that
	 * case, the POWER key will not be active again.
	 */
	__raw_writel(BM_POWER_CTRL_PSWITCH_IRQ,
		REGS_POWER_BASE + HW_POWER_CTRL_CLR);
	__raw_writel(BM_POWER_CTRL_ENIRQ_PSWITCH,
		REGS_POWER_BASE + HW_POWER_CTRL_SET);
}

static void mx28_pm_end(void)
{
	/*XXX: Nothing to do */
}

suspend_state_t mx28_pm_get_target(void)
{
	return saved_state;
}
EXPORT_SYMBOL(mx28_pm_get_target);

/**
 * mx28_pm_get_sleep_state - get sleep state we waking from
 *
 * returns boolean: 0 if waking up from standby, 1 otherwise
 */
int mx28_pm_sleep_was_deep(void)
{
	return saved_sleep_state;
}
EXPORT_SYMBOL(mx28_pm_sleep_was_deep);

static struct platform_suspend_ops mx28_suspend_ops = {
	.enter	= mx28_pm_enter,
	.valid	= mx28_pm_valid,
	.begin	= mx28_pm_begin,
	.end	= mx28_pm_end,
	.recover = mx28_pm_recover,
};

void mx28_pm_idle(void)
{
	local_irq_disable();
	local_fiq_disable();
	if (need_resched()) {
		local_fiq_enable();
		local_irq_enable();
		return;
	}

	__raw_writel(BM_CLKCTRL_CPU_INTERRUPT_WAIT,
		REGS_CLKCTRL_BASE + HW_CLKCTRL_CPU_SET);
	__asm__ __volatile__ ("mcr	p15, 0, r0, c7, c0, 4");

	local_fiq_enable();
	local_irq_enable();
}

static void mx28_pm_power_off(void)
{
	__raw_writel(BF_POWER_RESET_UNLOCK(0x3e77) | BM_POWER_RESET_PWD,
		REGS_POWER_BASE + HW_POWER_RESET);
}

struct mx28_pswitch_state {
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

static struct mx28_pswitch_state pswitch_state = {
	.dev_running = 0,
};

#define PSWITCH_POWER_DOWN_DELAY 30
static 	struct delayed_work pswitch_work;
static void pswitch_check_work(struct work_struct *work)
{
	int pin_value, i;
	for (i = 0; i < PSWITCH_POWER_DOWN_DELAY; i++) {
		pin_value = __raw_readl(REGS_POWER_BASE + HW_POWER_STS) &
			BF_POWER_STS_PSWITCH(0x1);
		if (pin_value == 0)
			break;
		msleep(100);
	}
	if (i < PSWITCH_POWER_DOWN_DELAY) {
		pr_info("pswitch goto suspend\n");
		complete(&suspend_request);
	} else {
		pr_info("release pswitch to power down\n");
		for (i = 0; i < 500; i++) {
			pin_value = __raw_readl(REGS_POWER_BASE + HW_POWER_STS)
				& BF_POWER_STS_PSWITCH(0x1);
			if (pin_value == 0)
				break;
			msleep(10);
		}
		pr_info("pswitch power down\n");
		mx28_pm_power_off();
	}
	__raw_writel(BM_POWER_CTRL_PSWITCH_IRQ,
		REGS_POWER_BASE + HW_POWER_CTRL_CLR);
	__raw_writel(BM_POWER_CTRL_PSWITCH_IRQ,
		REGS_POWER_BASE + HW_POWER_CTRL_CLR);
}


static irqreturn_t pswitch_interrupt(int irq, void *dev)
{

	/* check if irq by pswitch */
	if (!(__raw_readl(REGS_POWER_BASE + HW_POWER_CTRL) &
		BM_POWER_CTRL_PSWITCH_IRQ))
		return IRQ_HANDLED;
	__raw_writel(BM_POWER_CTRL_ENIRQ_PSWITCH,
		REGS_POWER_BASE + HW_POWER_CTRL_CLR);
	schedule_delayed_work(&pswitch_work, 1);
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
	INIT_DELAYED_WORK(&pswitch_work, pswitch_check_work);
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

static int __init mx28_pm_init(void)
{
	saved_sram = kmalloc(0x4000, GFP_ATOMIC);
	if (!saved_sram) {
		printk(KERN_ERR
		 "PM Suspend: can't allocate memory to save portion of SRAM\n");
		return -ENOMEM;
	}

	pm_power_off = mx28_pm_power_off;
	pm_idle = mx28_pm_idle;
	suspend_set_ops(&mx28_suspend_ops);
	init_pswitch();
	return 0;
}

late_initcall(mx28_pm_init);
