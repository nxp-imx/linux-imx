/*
 * Copyright (C) 2011-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*!
 * @file busfreq-imx6.c
 *
 * @brief A common API for the Freescale Semiconductor iMX6 Busfreq API
 *
 * The APIs are for setting bus frequency to different values based on the
 * highest freqeuncy requested.
 *
 * @ingroup PM
 */

#include <asm/cacheflush.h>
#include <asm/fncpy.h>
#include <asm/io.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>
#include <asm/tlb.h>
#include <linux/busfreq-imx6.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/reboot.h>
#include <linux/regulator/consumer.h>
#include <linux/sched.h>
#include <linux/suspend.h>
#include "clk.h"
#include "hardware.h"
#include "common.h"

#define LPAPM_CLK		24000000
#define DDR3_AUDIO_CLK		50000000
#define LPDDR2_AUDIO_CLK	100000000

#define	MMDC_MDMISC_DDR_TYPE_DDR3	0
#define	MMDC_MDMISC_DDR_TYPE_LPDDR2	1

static int ddr_type;
int high_bus_freq_mode;
int med_bus_freq_mode;
int audio_bus_freq_mode;
int low_bus_freq_mode;
int ultra_low_bus_freq_mode;
unsigned int ddr_med_rate;
unsigned int ddr_normal_rate;
unsigned long ddr_freq_change_total_size;
unsigned long ddr_freq_change_iram_base;
unsigned long ddr_freq_change_iram_phys;

static int bus_freq_scaling_initialized;
static struct device *busfreq_dev;
static int busfreq_suspended;
static u32 org_arm_rate;
static int bus_freq_scaling_is_active;
static int high_bus_count, med_bus_count, audio_bus_count, low_bus_count;
static unsigned int ddr_low_rate;

extern unsigned long iram_tlb_phys_addr;
extern int unsigned long iram_tlb_base_addr;

extern int init_mmdc_lpddr2_settings(struct platform_device *dev);
extern int init_mmdc_ddr3_settings_imx6q(struct platform_device *dev);
extern int init_mmdc_ddr3_settings_imx6sx(struct platform_device *dev);
extern int update_ddr_freq_imx6q(int ddr_rate);
extern int update_ddr_freq_imx6sx(int ddr_rate);
extern int update_lpddr2_freq(int ddr_rate);

DEFINE_MUTEX(bus_freq_mutex);

static struct clk *mmdc_clk;
static struct clk *pll2_400;
static struct clk *periph_clk;
static struct clk *periph_pre_clk;
static struct clk *periph_clk2_sel;
static struct clk *periph_clk2;
static struct clk *osc_clk;
static struct clk *cpu_clk;
static struct clk *pll3;
static struct clk *pll2;
static struct clk *pll2_bus;
static struct clk *pll2_bypass_src;
static struct clk *pll2_bypass;
static struct clk *pll2_200;
static struct clk *pll1_sys;
static struct clk *periph2_clk;
static struct clk *ocram_clk;
static struct clk *ahb_clk;
static struct clk *pll1_sw_clk;
static struct clk *periph2_pre_clk;
static struct clk *periph2_clk2_sel;
static struct clk *periph2_clk2;
static struct clk *step_clk;
static struct clk *axi_alt_sel_clk;
static struct clk *axi_sel_clk;
static struct clk *pll3_pfd1_540m;
static struct clk *m4_clk;
static struct clk *pll1;
static struct clk *pll1_bypass;
static struct clk *pll1_bypass_src;

static u32 pll2_org_rate;
static struct delayed_work low_bus_freq_handler;
static struct delayed_work bus_freq_daemon;

static RAW_NOTIFIER_HEAD(busfreq_notifier_chain);

static int busfreq_notify(enum busfreq_event event)
{
	int ret;

	ret = raw_notifier_call_chain(&busfreq_notifier_chain, event, NULL);

	return notifier_to_errno(ret);
}

int register_busfreq_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_register(&busfreq_notifier_chain, nb);
}
EXPORT_SYMBOL(register_busfreq_notifier);

int unregister_busfreq_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_unregister(&busfreq_notifier_chain, nb);
}
EXPORT_SYMBOL(unregister_busfreq_notifier);

static bool check_m4_sleep(void)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(500);

	while (imx_gpc_is_m4_sleeping() == 0)
		if (time_after(jiffies, timeout))
			return false;
	return  true;
}

static void enter_lpm_imx6sx(void)
{
	if (imx_src_is_m4_enabled())
		if (!check_m4_sleep())
			pr_err("M4 is NOT in sleep!!!\n");

	/* set periph_clk2 to source from OSC for periph */
	imx_clk_set_parent(periph_clk2_sel, osc_clk);
	imx_clk_set_parent(periph_clk, periph_clk2);
	/* set ahb/ocram to 24MHz */
	imx_clk_set_rate(ahb_clk, LPAPM_CLK);
	imx_clk_set_rate(ocram_clk, LPAPM_CLK);

	if (audio_bus_count) {
		/* Need to ensure that PLL2_PFD_400M is kept ON. */
		clk_prepare_enable(pll2_400);
		if (ddr_type == MMDC_MDMISC_DDR_TYPE_DDR3)
			update_ddr_freq_imx6sx(DDR3_AUDIO_CLK);
		else if (ddr_type == MMDC_MDMISC_DDR_TYPE_LPDDR2)
			update_lpddr2_freq(LPDDR2_AUDIO_CLK);
		imx_clk_set_parent(periph2_clk2_sel, pll3);
		imx_clk_set_parent(periph2_pre_clk, pll2_400);
		imx_clk_set_parent(periph2_clk, periph2_pre_clk);
		/*
		 * As periph2_clk's parent is not changed from
		 * high mode to audio mode, so clk framework
		 * will not update its children's freq, but we
		 * change the mmdc's podf in asm code, so here
		 * need to update mmdc rate to make sure clk
		 * tree is right, although it will not do any
		 * change to hardware.
		 */
		if (high_bus_freq_mode) {
			if (ddr_type == MMDC_MDMISC_DDR_TYPE_DDR3)
				imx_clk_set_rate(mmdc_clk, DDR3_AUDIO_CLK);
			else if (ddr_type == MMDC_MDMISC_DDR_TYPE_LPDDR2)
				imx_clk_set_rate(mmdc_clk, LPDDR2_AUDIO_CLK);
		}
		audio_bus_freq_mode = 1;
		low_bus_freq_mode = 0;
	} else {
		if (ddr_type == MMDC_MDMISC_DDR_TYPE_DDR3)
			update_ddr_freq_imx6sx(LPAPM_CLK);
		else if (ddr_type == MMDC_MDMISC_DDR_TYPE_LPDDR2)
			update_lpddr2_freq(LPAPM_CLK);
		imx_clk_set_parent(periph2_clk2_sel, osc_clk);
		imx_clk_set_parent(periph2_clk, periph2_clk2);

		if (audio_bus_freq_mode)
			clk_disable_unprepare(pll2_400);
		low_bus_freq_mode = 1;
		audio_bus_freq_mode = 0;
	}
}

static void exit_lpm_imx6sx(void)
{
	clk_prepare_enable(pll2_400);

	/*
	 * lower ahb/ocram's freq first to avoid too high
	 * freq during parent switch from OSC to pll3.
	 */
	imx_clk_set_rate(ahb_clk, LPAPM_CLK / 3);
	imx_clk_set_rate(ocram_clk, LPAPM_CLK / 2);
	/* set periph_clk2 to pll3 */
	imx_clk_set_parent(periph_clk2_sel, pll3);
	/* set periph clk to from pll2_400 */
	imx_clk_set_parent(periph_pre_clk, pll2_400);
	imx_clk_set_parent(periph_clk, periph_pre_clk);

	if (ddr_type == MMDC_MDMISC_DDR_TYPE_DDR3)
		update_ddr_freq_imx6sx(ddr_normal_rate);
	else if (ddr_type == MMDC_MDMISC_DDR_TYPE_LPDDR2)
		update_lpddr2_freq(ddr_normal_rate);
	/* correct parent info after ddr freq change in asm code */
	imx_clk_set_parent(periph2_pre_clk, pll2_400);
	imx_clk_set_parent(periph2_clk, periph2_pre_clk);
	imx_clk_set_parent(periph2_clk2_sel, pll3);

	/*
	 * As periph2_clk's parent is not changed from
	 * audio mode to high mode, so clk framework
	 * will not update its children's freq, but we
	 * change the mmdc's podf in asm code, so here
	 * need to update mmdc rate to make sure clk
	 * tree is right, although it will not do any
	 * change to hardware.
	 */
	if (audio_bus_freq_mode)
		imx_clk_set_rate(mmdc_clk, ddr_normal_rate);

	clk_disable_unprepare(pll2_400);

	if (audio_bus_freq_mode)
		clk_disable_unprepare(pll2_400);
}

static void enter_lpm_imx6sl(void)
{
	if (high_bus_freq_mode) {
		pll2_org_rate = clk_get_rate(pll2_bus);
		/* Set periph_clk to be sourced from OSC_CLK */
		imx_clk_set_parent(periph_clk2_sel, osc_clk);
		imx_clk_set_parent(periph_clk, periph_clk2);
		/* Ensure AHB/AXI clks are at 24MHz. */
		imx_clk_set_rate(ahb_clk, LPAPM_CLK);
		imx_clk_set_rate(ocram_clk, LPAPM_CLK);
	}
	if (audio_bus_count) {
		/* Set AHB to 8MHz to lower pwer.*/
		imx_clk_set_rate(ahb_clk, LPAPM_CLK / 3);

		/* Set up DDR to 100MHz. */
		update_lpddr2_freq(LPDDR2_AUDIO_CLK);

		/* Fix the clock tree in kernel */
		imx_clk_set_parent(periph2_pre_clk, pll2_200);
		imx_clk_set_parent(periph2_clk, periph2_pre_clk);

		if (low_bus_freq_mode || ultra_low_bus_freq_mode) {
			/*
			 * Fix the clock tree in kernel, make sure
			 * pll2_bypass is updated as it is
			 * sourced from PLL2.
			 */
			imx_clk_set_parent(pll2_bypass, pll2);
			/*
			 * Swtich ARM to run off PLL2_PFD2_400MHz
			 * since DDR is anyway at 100MHz.
			 */
			imx_clk_set_parent(step_clk, pll2_400);
			imx_clk_set_parent(pll1_sw_clk, step_clk);
			/*
			  * Need to ensure that PLL1 is bypassed and enabled
			  * before ARM-PODF is set.
			  */
			clk_set_parent(pll1_bypass, pll1_bypass_src);

			/*
			 * Ensure that the clock will be
			 * at original speed.
			 */
			imx_clk_set_rate(cpu_clk, org_arm_rate);
		}
		low_bus_freq_mode = 0;
		ultra_low_bus_freq_mode = 0;
		audio_bus_freq_mode = 1;
	} else {
		u32 arm_div, pll1_rate;
		org_arm_rate = clk_get_rate(cpu_clk);
		if (low_bus_freq_mode && low_bus_count == 0) {
			/*
			 * We are already in DDR @ 24MHz state, but
			 * no one but ARM needs the DDR. In this case,
			 * we can lower the DDR freq to 1MHz when ARM
			 * enters WFI in this state. Keep track of this state.
			 */
			ultra_low_bus_freq_mode = 1;
			low_bus_freq_mode = 0;
			audio_bus_freq_mode = 0;
		} else {
			if (!ultra_low_bus_freq_mode && !low_bus_freq_mode) {
				/*
				 * Anyway, make sure the AHB is running at 24MHz
				 * in low_bus_freq_mode.
				 */
				if (audio_bus_freq_mode)
					imx_clk_set_rate(ahb_clk, LPAPM_CLK);
				/*
				 * Set DDR to 24MHz.
				 * Since we are going to bypass PLL2,
				 * we need to move ARM clk off PLL2_PFD2
				 * to PLL1. Make sure the PLL1 is running
				 * at the lowest possible freq.
				 * To work well with CPUFREQ we want to ensure that
				 * the CPU freq does not change, so attempt to
				 * get a freq as close to 396MHz as possible.
				 */
				imx_clk_set_rate(pll1,
					clk_round_rate(pll1, (org_arm_rate * 2)));
				pll1_rate = clk_get_rate(pll1);
				arm_div = pll1_rate / org_arm_rate;
				if (pll1_rate / arm_div > org_arm_rate)
					arm_div++;
				/*
				  * Need to ensure that PLL1 is bypassed and enabled
				  * before ARM-PODF is set.
				  */
				clk_set_parent(pll1_bypass, pll1);
				/*
				 * Ensure ARM CLK is lower before
				 * changing the parent.
				 */
				imx_clk_set_rate(cpu_clk, org_arm_rate / arm_div);
				/* Now set the ARM clk parent to PLL1_SYS. */
				imx_clk_set_parent(pll1_sw_clk, pll1_sys);

				/*
				 * Set STEP_CLK back to OSC to save power and
				 * also to maintain the parent.The WFI iram code
				 * will switch step_clk to osc, but the clock API
				 * is not aware of the change and when a new request
				 * to change the step_clk parent to pll2_pfd2_400M
				 * is requested sometime later, the change is ignored.
				 */
				imx_clk_set_parent(step_clk, osc_clk);
				/* Now set DDR to 24MHz. */
				update_lpddr2_freq(LPAPM_CLK);

				/*
				 * Fix the clock tree in kernel.
				 * Make sure PLL2 rate is updated as it gets
				 * bypassed in the DDR freq change code.
				 */
				imx_clk_set_parent(pll2_bypass, pll2_bypass_src);
				imx_clk_set_parent(periph2_clk2_sel, pll2_bus);
				imx_clk_set_parent(periph2_clk, periph2_clk2);

			}
			if (low_bus_count == 0) {
				ultra_low_bus_freq_mode = 1;
				low_bus_freq_mode = 0;
			} else {
				ultra_low_bus_freq_mode = 0;
				low_bus_freq_mode = 1;
			}
			audio_bus_freq_mode = 0;
		}
	}
}

static void exit_lpm_imx6sl(void)
{
	/* Change DDR freq in IRAM. */
	update_lpddr2_freq(ddr_normal_rate);

	/*
	 * Fix the clock tree in kernel.
	 * Make sure PLL2 rate is updated as it gets
	 * un-bypassed in the DDR freq change code.
	 */
	imx_clk_set_parent(pll2_bypass, pll2);
	imx_clk_set_parent(periph2_pre_clk, pll2_400);
	imx_clk_set_parent(periph2_clk, periph2_pre_clk);

	/* Ensure that periph_clk is sourced from PLL2_400. */
	imx_clk_set_parent(periph_pre_clk, pll2_400);
	/*
	 * Before switching the perhiph_clk, ensure that the
	 * AHB/AXI will not be too fast.
	 */
	imx_clk_set_rate(ahb_clk, LPAPM_CLK / 3);
	imx_clk_set_rate(ocram_clk, LPAPM_CLK / 2);
	imx_clk_set_parent(periph_clk, periph_pre_clk);

	if (low_bus_freq_mode || ultra_low_bus_freq_mode) {
		/* Move ARM from PLL1_SW_CLK to PLL2_400. */
		imx_clk_set_parent(step_clk, pll2_400);
		imx_clk_set_parent(pll1_sw_clk, step_clk);
		/*
		  * Need to ensure that PLL1 is bypassed and enabled
		  * before ARM-PODF is set.
		  */
		clk_set_parent(pll1_bypass, pll1_bypass_src);
		imx_clk_set_rate(cpu_clk, org_arm_rate);
		ultra_low_bus_freq_mode = 0;
	}
}

static void reduce_bus_freq(void)
{
	clk_prepare_enable(pll3);
	if (audio_bus_count && (low_bus_freq_mode || ultra_low_bus_freq_mode))
		busfreq_notify(LOW_BUSFREQ_EXIT);
	else if (!audio_bus_count)
		busfreq_notify(LOW_BUSFREQ_ENTER);
	if (cpu_is_imx6sl())
		enter_lpm_imx6sl();
	else if (cpu_is_imx6sx())
		enter_lpm_imx6sx();
	else {
		if (cpu_is_imx6dl())
			/* Set axi to periph_clk */
			imx_clk_set_parent(axi_sel_clk, periph_clk);

		if (audio_bus_count) {
			/* Need to ensure that PLL2_PFD_400M is kept ON. */
			clk_prepare_enable(pll2_400);
			update_ddr_freq_imx6q(DDR3_AUDIO_CLK);
			/* Make sure periph clk's parent also got updated */
			imx_clk_set_parent(periph_clk2_sel, pll3);
			imx_clk_set_parent(periph_pre_clk, pll2_200);
			imx_clk_set_parent(periph_clk, periph_pre_clk);
			audio_bus_freq_mode = 1;
			low_bus_freq_mode = 0;
		} else {
			update_ddr_freq_imx6q(LPAPM_CLK);
			/* Make sure periph clk's parent also got updated */
			imx_clk_set_parent(periph_clk2_sel, osc_clk);
			/* Set periph_clk parent to OSC via periph_clk2_sel */
			imx_clk_set_parent(periph_clk, periph_clk2);
			if (audio_bus_freq_mode)
				clk_disable_unprepare(pll2_400);
			low_bus_freq_mode = 1;
			audio_bus_freq_mode = 0;
		}
	}
	clk_disable_unprepare(pll3);

	med_bus_freq_mode = 0;
	high_bus_freq_mode = 0;

	if (audio_bus_freq_mode)
		dev_dbg(busfreq_dev, "Bus freq set to audio mode. Count:\
			high %d, med %d, audio %d\n",
			high_bus_count, med_bus_count, audio_bus_count);
	if (low_bus_freq_mode)
		dev_dbg(busfreq_dev, "Bus freq set to low mode. Count:\
			high %d, med %d, audio %d\n",
			high_bus_count, med_bus_count, audio_bus_count);
}

static void reduce_bus_freq_handler(struct work_struct *work)
{
	mutex_lock(&bus_freq_mutex);

	reduce_bus_freq();

	mutex_unlock(&bus_freq_mutex);
}

/*
 * Set the DDR, AHB to 24MHz.
 * This mode will be activated only when none of the modules that
 * need a higher DDR or AHB frequency are active.
 */
int set_low_bus_freq(void)
{
	if (busfreq_suspended)
		return 0;

	if (!bus_freq_scaling_initialized || !bus_freq_scaling_is_active)
		return 0;

	/*
	 * Check to see if we need to got from
	 * low bus freq mode to audio bus freq mode.
	 * If so, the change needs to be done immediately.
	 */
	if (audio_bus_count && (low_bus_freq_mode || ultra_low_bus_freq_mode))
		reduce_bus_freq();
	else
		/*
		 * Don't lower the frequency immediately. Instead
		 * scheduled a delayed work and drop the freq if
		 * the conditions still remain the same.
		 */
		schedule_delayed_work(&low_bus_freq_handler,
					usecs_to_jiffies(3000000));
	return 0;
}

/*
 * Set the DDR to either 528MHz or 400MHz for iMX6qd
 * or 400MHz for iMX6dl.
 */
static int set_high_bus_freq(int high_bus_freq)
{
	struct clk *periph_clk_parent;

	if (bus_freq_scaling_initialized && bus_freq_scaling_is_active)
		cancel_delayed_work_sync(&low_bus_freq_handler);

	if (busfreq_suspended)
		return 0;

	if (cpu_is_imx6q())
		periph_clk_parent = pll2_bus;
	else
		periph_clk_parent = pll2_400;

	if (!bus_freq_scaling_initialized || !bus_freq_scaling_is_active)
		return 0;

	if (high_bus_freq_mode)
		return 0;

	/* medium bus freq is only supported for MX6DQ */
	if (med_bus_freq_mode && !high_bus_freq)
		return 0;

	if (low_bus_freq_mode || ultra_low_bus_freq_mode)
		busfreq_notify(LOW_BUSFREQ_EXIT);

	clk_prepare_enable(pll3);
	if (cpu_is_imx6sl())
		exit_lpm_imx6sl();
	else if (cpu_is_imx6sx())
		exit_lpm_imx6sx();
	else {
		if (high_bus_freq) {
			clk_prepare_enable(pll2_400);
			update_ddr_freq_imx6q(ddr_normal_rate);
			/* Make sure periph clk's parent also got updated */
			imx_clk_set_parent(periph_clk2_sel, pll3);
			imx_clk_set_parent(periph_pre_clk, periph_clk_parent);
			imx_clk_set_parent(periph_clk, periph_pre_clk);
			if (cpu_is_imx6dl()) {
				/* Set axi to pll3_pfd1_540m */
				imx_clk_set_parent(axi_alt_sel_clk, pll3_pfd1_540m);
				imx_clk_set_parent(axi_sel_clk, axi_alt_sel_clk);
			}
			clk_disable_unprepare(pll2_400);
		} else {
			update_ddr_freq_imx6q(ddr_med_rate);
			/* Make sure periph clk's parent also got updated */
			imx_clk_set_parent(periph_clk2_sel, pll3);
			imx_clk_set_parent(periph_pre_clk, pll2_400);
			imx_clk_set_parent(periph_clk, periph_pre_clk);
		}
		if (audio_bus_freq_mode)
			clk_disable_unprepare(pll2_400);
	}

	high_bus_freq_mode = 1;
	med_bus_freq_mode = 0;
	low_bus_freq_mode = 0;
	audio_bus_freq_mode = 0;

	clk_disable_unprepare(pll3);
	if (high_bus_freq_mode)
		dev_dbg(busfreq_dev, "Bus freq set to high mode. Count:\
			high %d, med %d, audio %d\n",
			high_bus_count, med_bus_count, audio_bus_count);
	if (med_bus_freq_mode)
		dev_dbg(busfreq_dev, "Bus freq set to med mode. Count:\
			high %d, med %d, audio %d\n",
			high_bus_count, med_bus_count, audio_bus_count);

	return 0;
}

void request_bus_freq(enum bus_freq_mode mode)
{
	mutex_lock(&bus_freq_mutex);

	if (mode == BUS_FREQ_HIGH)
		high_bus_count++;
	else if (mode == BUS_FREQ_MED)
		med_bus_count++;
	else if (mode == BUS_FREQ_AUDIO)
		audio_bus_count++;
	else if (mode == BUS_FREQ_LOW)
		low_bus_count++;

	if (busfreq_suspended || !bus_freq_scaling_initialized ||
		!bus_freq_scaling_is_active) {
		mutex_unlock(&bus_freq_mutex);
		return;
	}
	cancel_delayed_work_sync(&low_bus_freq_handler);

	if (cpu_is_imx6dl() || cpu_is_imx6sx()) {
		/* No support for medium setpoint on i.MX6DL and i.MX6SX. */
		if (mode == BUS_FREQ_MED) {
			high_bus_count++;
			mode = BUS_FREQ_HIGH;
		}
	}

	if ((mode == BUS_FREQ_HIGH) && (!high_bus_freq_mode)) {
		set_high_bus_freq(1);
		mutex_unlock(&bus_freq_mutex);
		return;
	}

	if ((mode == BUS_FREQ_MED) && (!high_bus_freq_mode) &&
		(!med_bus_freq_mode)) {
		set_high_bus_freq(0);
		mutex_unlock(&bus_freq_mutex);
		return;
	}
	if ((mode == BUS_FREQ_AUDIO) && (!high_bus_freq_mode) &&
		(!med_bus_freq_mode) && (!audio_bus_freq_mode)) {
		set_low_bus_freq();
		mutex_unlock(&bus_freq_mutex);
		return;
	}
	mutex_unlock(&bus_freq_mutex);
	return;
}
EXPORT_SYMBOL(request_bus_freq);

void release_bus_freq(enum bus_freq_mode mode)
{
	mutex_lock(&bus_freq_mutex);

	if (mode == BUS_FREQ_HIGH) {
		if (high_bus_count == 0) {
			dev_err(busfreq_dev, "high bus count mismatch!\n");
			dump_stack();
			mutex_unlock(&bus_freq_mutex);
			return;
		}
		high_bus_count--;
	} else if (mode == BUS_FREQ_MED) {
		if (med_bus_count == 0) {
			dev_err(busfreq_dev, "med bus count mismatch!\n");
			dump_stack();
			mutex_unlock(&bus_freq_mutex);
			return;
		}
		med_bus_count--;
	} else if (mode == BUS_FREQ_AUDIO) {
		if (audio_bus_count == 0) {
			dev_err(busfreq_dev, "audio bus count mismatch!\n");
			dump_stack();
			mutex_unlock(&bus_freq_mutex);
			return;
		}
		audio_bus_count--;
	} else if (mode == BUS_FREQ_LOW) {
		if (low_bus_count == 0) {
			dev_err(busfreq_dev, "low bus count mismatch!\n");
			dump_stack();
			mutex_unlock(&bus_freq_mutex);
			return;
		}
		low_bus_count--;
	}

	if (busfreq_suspended || !bus_freq_scaling_initialized ||
		!bus_freq_scaling_is_active) {
		mutex_unlock(&bus_freq_mutex);
		return;
	}

	if (cpu_is_imx6dl() || cpu_is_imx6sx()) {
		/* No support for medium setpoint on i.MX6DL and i.MX6SX. */
		if (mode == BUS_FREQ_MED) {
			high_bus_count--;
			mode = BUS_FREQ_HIGH;
		}
	}

	if ((!audio_bus_freq_mode) && (high_bus_count == 0) &&
		(med_bus_count == 0) && (audio_bus_count != 0)) {
		set_low_bus_freq();
		mutex_unlock(&bus_freq_mutex);
		return;
	}
	if ((!low_bus_freq_mode) && (high_bus_count == 0) &&
		(med_bus_count == 0) && (audio_bus_count == 0) &&
		(low_bus_count != 0)) {
		set_low_bus_freq();
		mutex_unlock(&bus_freq_mutex);
		return;
	}
	if ((!ultra_low_bus_freq_mode) && (high_bus_count == 0) &&
		(med_bus_count == 0) && (audio_bus_count == 0) &&
		(low_bus_count == 0)) {
		set_low_bus_freq();
		mutex_unlock(&bus_freq_mutex);
		return;
	}

	mutex_unlock(&bus_freq_mutex);
	return;
}
EXPORT_SYMBOL(release_bus_freq);

static struct map_desc ddr_iram_io_desc __initdata = {
	/* .virtual and .pfn are run-time assigned */
	.length		= SZ_1M,
	.type		= MT_MEMORY_RWX_NONCACHED,
};

const static char *ddr_freq_iram_match[] __initconst = {
	"fsl,ddr-lpm-sram",
	NULL
};

static int __init imx6_dt_find_ddr_sram(unsigned long node,
		const char *uname, int depth, void *data)
{
	unsigned long ddr_iram_addr;
	__be32 *prop;

	if (of_flat_dt_match(node, ddr_freq_iram_match)) {
		unsigned long len;
		prop = of_get_flat_dt_prop(node, "reg", &len);
		if (prop == NULL || len != (sizeof(unsigned long) * 2))
			return EINVAL;
		ddr_iram_addr = be32_to_cpu(prop[0]);
		ddr_freq_change_total_size = be32_to_cpu(prop[1]);
		ddr_freq_change_iram_phys = ddr_iram_addr;

		/* Make sure ddr_freq_change_iram_phys is 8 byte aligned. */
		if ((uintptr_t)(ddr_freq_change_iram_phys) & (FNCPY_ALIGN - 1))
			ddr_freq_change_iram_phys += FNCPY_ALIGN - ((uintptr_t)ddr_freq_change_iram_phys % (FNCPY_ALIGN));
	}
	return 0;
}

void __init imx6_busfreq_map_io(void)
{
	/*
	 * Get the address of IRAM to be used by the ddr frequency
	 * change code from the device tree.
	 */
	WARN_ON(of_scan_flat_dt(imx6_dt_find_ddr_sram, NULL));

	if (ddr_freq_change_iram_phys) {
		ddr_freq_change_iram_base = IMX_IO_P2V(ddr_freq_change_iram_phys);
		if ((iram_tlb_phys_addr & 0xFFF00000) != (ddr_freq_change_iram_phys & 0xFFF00000)) {
			/* We need to create a 1M page table entry. */
			ddr_iram_io_desc.virtual = IMX_IO_P2V(ddr_freq_change_iram_phys & 0xFFF00000);
			ddr_iram_io_desc.pfn = __phys_to_pfn(ddr_freq_change_iram_phys & 0xFFF00000);
			iotable_init(&ddr_iram_io_desc, 1);
		}
		memset((void *)ddr_freq_change_iram_base, 0, ddr_freq_change_total_size);
	}
}

static void bus_freq_daemon_handler(struct work_struct *work)
{
	mutex_lock(&bus_freq_mutex);
	if ((!low_bus_freq_mode) && (!ultra_low_bus_freq_mode) && (high_bus_count == 0) &&
		(med_bus_count == 0) && (audio_bus_count == 0))
		set_low_bus_freq();
	mutex_unlock(&bus_freq_mutex);
}

static ssize_t bus_freq_scaling_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (bus_freq_scaling_is_active)
		return sprintf(buf, "Bus frequency scaling is enabled\n");
	else
		return sprintf(buf, "Bus frequency scaling is disabled\n");
}

static ssize_t bus_freq_scaling_enable_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	if (strncmp(buf, "1", 1) == 0) {
		bus_freq_scaling_is_active = 1;
		set_high_bus_freq(1);
		/*
		 * We set bus freq to highest at the beginning,
		 * so we use this daemon thread to make sure system
		 * can enter low bus mode if
		 * there is no high bus request pending
		 */
		schedule_delayed_work(&bus_freq_daemon,
			usecs_to_jiffies(5000000));
	} else if (strncmp(buf, "0", 1) == 0) {
		if (bus_freq_scaling_is_active)
			set_high_bus_freq(1);
		bus_freq_scaling_is_active = 0;
	}
	return size;
}

static int bus_freq_pm_notify(struct notifier_block *nb, unsigned long event,
	void *dummy)
{
	mutex_lock(&bus_freq_mutex);

	if (event == PM_SUSPEND_PREPARE) {
		high_bus_count++;
		set_high_bus_freq(1);
		busfreq_suspended = 1;
	} else if (event == PM_POST_SUSPEND) {
		busfreq_suspended = 0;
		high_bus_count--;
		schedule_delayed_work(&bus_freq_daemon,
			usecs_to_jiffies(5000000));
	}

	mutex_unlock(&bus_freq_mutex);

	return NOTIFY_OK;
}

static int busfreq_reboot_notifier_event(struct notifier_block *this,
						 unsigned long event, void *ptr)
{
	/* System is rebooting. Set the system into high_bus_freq_mode. */
	request_bus_freq(BUS_FREQ_HIGH);

	return 0;
}

static struct notifier_block imx_bus_freq_pm_notifier = {
	.notifier_call = bus_freq_pm_notify,
};

static struct notifier_block imx_busfreq_reboot_notifier = {
	.notifier_call = busfreq_reboot_notifier_event,
};


static DEVICE_ATTR(enable, 0644, bus_freq_scaling_enable_show,
			bus_freq_scaling_enable_store);

/*!
 * This is the probe routine for the bus frequency driver.
 *
 * @param   pdev   The platform device structure
 *
 * @return         The function returns 0 on success
 *
 */

static int busfreq_probe(struct platform_device *pdev)
{
	u32 err;

	busfreq_dev = &pdev->dev;

	/* Return if no IRAM space is allocated for ddr freq change code. */
	if (!ddr_freq_change_iram_base)
		return ENOMEM;

	pll2_400 = devm_clk_get(&pdev->dev, "pll2_pfd2_396m");
	if (IS_ERR(pll2_400)) {
		dev_err(busfreq_dev, "%s: failed to get pll2_pfd2_396m\n",
		__func__);
		return PTR_ERR(pll2_400);
	}

	pll2_200 = devm_clk_get(&pdev->dev, "pll2_198m");
	if (IS_ERR(pll2_200)) {
		dev_err(busfreq_dev, "%s: failed to get pll2_198m\n",
			__func__);
		return PTR_ERR(pll2_200);
	}

	pll2_bus = devm_clk_get(&pdev->dev, "pll2_bus");
	if (IS_ERR(pll2_bus)) {
		dev_err(busfreq_dev, "%s: failed to get pll2_bus\n",
			__func__);
		return PTR_ERR(pll2_bus);
	}

	cpu_clk = devm_clk_get(&pdev->dev, "arm");
	if (IS_ERR(cpu_clk)) {
		dev_err(busfreq_dev, "%s: failed to get cpu_clk\n",
			__func__);
		return PTR_ERR(cpu_clk);
	}

	pll3 = devm_clk_get(&pdev->dev, "pll3_usb_otg");
	if (IS_ERR(pll3)) {
		dev_err(busfreq_dev, "%s: failed to get pll3_usb_otg\n",
			__func__);
		return PTR_ERR(pll3);
	}

	periph_clk = devm_clk_get(&pdev->dev, "periph");
	if (IS_ERR(periph_clk)) {
		dev_err(busfreq_dev, "%s: failed to get periph\n",
			__func__);
		return PTR_ERR(periph_clk);
	}

	periph_pre_clk = devm_clk_get(&pdev->dev, "periph_pre");
	if (IS_ERR(periph_pre_clk)) {
		dev_err(busfreq_dev, "%s: failed to get periph_pre\n",
			__func__);
		return PTR_ERR(periph_pre_clk);
	}

	periph_clk2 = devm_clk_get(&pdev->dev, "periph_clk2");
	if (IS_ERR(periph_clk2)) {
		dev_err(busfreq_dev, "%s: failed to get periph_clk2\n",
			__func__);
		return PTR_ERR(periph_clk2);
	}

	periph_clk2_sel = devm_clk_get(&pdev->dev, "periph_clk2_sel");
	if (IS_ERR(periph_clk2_sel)) {
		dev_err(busfreq_dev, "%s: failed to get periph_clk2_sel\n",
			__func__);
		return PTR_ERR(periph_clk2_sel);
	}

	osc_clk = devm_clk_get(&pdev->dev, "osc");
	if (IS_ERR(osc_clk)) {
		dev_err(busfreq_dev, "%s: failed to get osc_clk\n",
			__func__);
		return PTR_ERR(osc_clk);
	}

	if (cpu_is_imx6dl()) {
		axi_alt_sel_clk = devm_clk_get(&pdev->dev, "axi_alt_sel");
		if (IS_ERR(axi_alt_sel_clk)) {
			dev_err(busfreq_dev, "%s: failed to get axi_alt_sel_clk\n",
				__func__);
			return PTR_ERR(axi_alt_sel_clk);
		}

		axi_sel_clk = devm_clk_get(&pdev->dev, "axi_sel");
		if (IS_ERR(axi_sel_clk)) {
			dev_err(busfreq_dev, "%s: failed to get axi_sel_clk\n",
				__func__);
			return PTR_ERR(axi_sel_clk);
		}

		pll3_pfd1_540m = devm_clk_get(&pdev->dev, "pll3_pfd1_540m");
		if (IS_ERR(pll3_pfd1_540m)) {
			dev_err(busfreq_dev,
				"%s: failed to get pll3_pfd1_540m\n", __func__);
			return PTR_ERR(pll3_pfd1_540m);
		}
	}

	if (cpu_is_imx6sl() || cpu_is_imx6sx()) {
		ahb_clk = devm_clk_get(&pdev->dev, "ahb");
		if (IS_ERR(ahb_clk)) {
			dev_err(busfreq_dev, "%s: failed to get ahb_clk\n",
				__func__);
			return PTR_ERR(ahb_clk);
		}

		ocram_clk = devm_clk_get(&pdev->dev, "ocram");
		if (IS_ERR(ocram_clk)) {
			dev_err(busfreq_dev, "%s: failed to get ocram_clk\n",
				__func__);
			return PTR_ERR(ocram_clk);
		}

		periph2_clk = devm_clk_get(&pdev->dev, "periph2");
		if (IS_ERR(periph2_clk)) {
			dev_err(busfreq_dev, "%s: failed to get periph2\n",
				__func__);
			return PTR_ERR(periph2_clk);
		}

		periph2_pre_clk = devm_clk_get(&pdev->dev, "periph2_pre");
		if (IS_ERR(periph2_pre_clk)) {
			dev_err(busfreq_dev,
				"%s: failed to get periph2_pre_clk\n",
				__func__);
			return PTR_ERR(periph2_pre_clk);
		}

		periph2_clk2 = devm_clk_get(&pdev->dev, "periph2_clk2");
		if (IS_ERR(periph2_clk2)) {
			dev_err(busfreq_dev,
				"%s: failed to get periph2_clk2\n",
				__func__);
			return PTR_ERR(periph2_clk2);
		}

		periph2_clk2_sel = devm_clk_get(&pdev->dev, "periph2_clk2_sel");
		if (IS_ERR(periph2_clk2_sel)) {
			dev_err(busfreq_dev,
				"%s: failed to get periph2_clk2_sel\n",
				__func__);
			return PTR_ERR(periph2_clk2_sel);
		}

		step_clk = devm_clk_get(&pdev->dev, "step");
		if (IS_ERR(step_clk)) {
			dev_err(busfreq_dev,
				"%s: failed to get step_clk\n",
				__func__);
			return PTR_ERR(step_clk);
		}
	}
	if (cpu_is_imx6sl()) {
		pll1 = devm_clk_get(&pdev->dev, "pll1");
		if (IS_ERR(pll1)) {
			dev_err(busfreq_dev, "%s: failed to get pll1\n",
				__func__);
			return PTR_ERR(pll1);
		}

		pll1_bypass = devm_clk_get(&pdev->dev, "pll1_bypass");
		if (IS_ERR(pll1_bypass)) {
			dev_err(busfreq_dev, "%s: failed to get pll1_bypass\n",
				__func__);
			return PTR_ERR(pll1_bypass);
		}

		pll1_bypass_src = devm_clk_get(&pdev->dev, "pll1_bypass_src");
		if (IS_ERR(pll1_bypass_src)) {
			dev_err(busfreq_dev, "%s: failed to get pll1_bypass_src\n",
				__func__);
			return PTR_ERR(pll1_bypass_src);
		}

		pll1_sys = devm_clk_get(&pdev->dev, "pll1_sys");
		if (IS_ERR(pll1_sys)) {
			dev_err(busfreq_dev, "%s: failed to get pll1_sys\n",
				__func__);
			return PTR_ERR(pll1_sys);
		}

		pll1_sw_clk = devm_clk_get(&pdev->dev, "pll1_sw");
		if (IS_ERR(pll1_sw_clk)) {
			dev_err(busfreq_dev, "%s: failed to get pll1_sw_clk\n",
				__func__);
			return PTR_ERR(pll1_sw_clk);
		}

		pll2_bypass_src = devm_clk_get(&pdev->dev, "pll2_bypass_src");
		if (IS_ERR(pll2_bypass_src)) {
			dev_err(busfreq_dev, "%s: failed to get pll2_bypass_src\n",
				__func__);
			return PTR_ERR(pll2_bypass_src);
		}

		pll2 = devm_clk_get(&pdev->dev, "pll2");
		if (IS_ERR(pll2)) {
			dev_err(busfreq_dev, "%s: failed to get pll2\n",
				__func__);
			return PTR_ERR(pll2);
		}

		pll2_bypass = devm_clk_get(&pdev->dev, "pll2_bypass");
		if (IS_ERR(pll2_bypass)) {
			dev_err(busfreq_dev, "%s: failed to get pll2_bypass\n",
				__func__);
			return PTR_ERR(pll2_bypass);
		}
	}
	if (cpu_is_imx6sx()) {
		mmdc_clk = devm_clk_get(&pdev->dev, "mmdc");
		if (IS_ERR(mmdc_clk)) {
			dev_err(busfreq_dev,
				"%s: failed to get mmdc_clk\n",
				__func__);
			return PTR_ERR(mmdc_clk);
		}
		m4_clk = devm_clk_get(&pdev->dev, "m4");
		if (IS_ERR(m4_clk)) {
			dev_err(busfreq_dev,
				"%s: failed to get m4_clk\n",
				__func__);
			return PTR_ERR(m4_clk);
		}
	}

	err = sysfs_create_file(&busfreq_dev->kobj, &dev_attr_enable.attr);
	if (err) {
		dev_err(busfreq_dev,
		       "Unable to register sysdev entry for BUSFREQ");
		return err;
	}

	if (of_property_read_u32(pdev->dev.of_node, "fsl,max_ddr_freq",
			&ddr_normal_rate)) {
		dev_err(busfreq_dev, "max_ddr_freq entry missing\n");
		return -EINVAL;
	}

	high_bus_freq_mode = 1;
	med_bus_freq_mode = 0;
	low_bus_freq_mode = 0;
	audio_bus_freq_mode = 0;
	ultra_low_bus_freq_mode = 0;

	bus_freq_scaling_is_active = 1;
	bus_freq_scaling_initialized = 1;

	ddr_low_rate = LPAPM_CLK;
	if (cpu_is_imx6q()) {
		if (of_property_read_u32(pdev->dev.of_node, "fsl,med_ddr_freq",
				&ddr_med_rate)) {
			dev_info(busfreq_dev,
					"DDR medium rate not supported.\n");
			ddr_med_rate = ddr_normal_rate;
		}
	}

	INIT_DELAYED_WORK(&low_bus_freq_handler, reduce_bus_freq_handler);
	INIT_DELAYED_WORK(&bus_freq_daemon, bus_freq_daemon_handler);
	register_pm_notifier(&imx_bus_freq_pm_notifier);
	register_reboot_notifier(&imx_busfreq_reboot_notifier);

	/*
	 * Need to make sure to an entry for the ddr freq change code address in the IRAM page table.
	 * This is only required if the DDR freq code and suspend/idle code are in different OCRAM spaces.
	 */
	if ((iram_tlb_phys_addr & 0xFFF00000) != (ddr_freq_change_iram_phys & 0xFFF00000)) {
		unsigned long i;

		/*
		 * Make sure the ddr_iram virtual address has a mapping
		 * in the IRAM page table.
		 */
		i = ((IMX_IO_P2V(ddr_freq_change_iram_phys) >> 20) << 2) / 4;
		*((unsigned long *)iram_tlb_base_addr + i) =
			(ddr_freq_change_iram_phys  & 0xFFF00000) | TT_ATTRIB_NON_CACHEABLE_1M;
	}

	if (cpu_is_imx6sl()) {
		err = init_mmdc_lpddr2_settings(pdev);
	} else if (cpu_is_imx6sx()) {
		ddr_type = imx_mmdc_get_ddr_type();
		/* check whether it is a DDR3 or LPDDR2 board */
		if (ddr_type == MMDC_MDMISC_DDR_TYPE_DDR3)
			err = init_mmdc_ddr3_settings_imx6sx(pdev);
		else if (ddr_type == MMDC_MDMISC_DDR_TYPE_LPDDR2)
			err = init_mmdc_lpddr2_settings(pdev);
		/* if M4 is enabled and rate > 24MHz, add high bus count */
		if (imx_src_is_m4_enabled() &&
			(clk_get_rate(m4_clk) > LPAPM_CLK))
			high_bus_count++;
	} else {
		err = init_mmdc_ddr3_settings_imx6q(pdev);
	}

	if (err) {
		dev_err(busfreq_dev, "Busfreq init of MMDC failed\n");
		return err;
	}
	return 0;
}

static const struct of_device_id imx6_busfreq_ids[] = {
	{ .compatible = "fsl,imx6_busfreq", },
	{ /* sentinel */ }
};

static struct platform_driver busfreq_driver = {
	.driver = {
		.name = "imx6_busfreq",
		.owner  = THIS_MODULE,
		.of_match_table = imx6_busfreq_ids,
		},
	.probe = busfreq_probe,
};

/*!
 * Initialise the busfreq_driver.
 *
 * @return  The function always returns 0.
 */

static int __init busfreq_init(void)
{
#ifndef CONFIG_MX6_VPU_352M
	if (platform_driver_register(&busfreq_driver) != 0)
		return -ENODEV;

	printk(KERN_INFO "Bus freq driver module loaded\n");
#endif
	return 0;
}

static void __exit busfreq_cleanup(void)
{
	sysfs_remove_file(&busfreq_dev->kobj, &dev_attr_enable.attr);

	/* Unregister the device structure */
	platform_driver_unregister(&busfreq_driver);
	bus_freq_scaling_initialized = 0;
}

module_init(busfreq_init);
module_exit(busfreq_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("BusFreq driver");
MODULE_LICENSE("GPL");
