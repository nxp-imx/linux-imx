/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file bus_freq.c
 *
 * @brief A common API for the Freescale Semiconductor i.MXC CPUfreq module
 * and DVFS CORE module.
 *
 * The APIs are for setting bus frequency to low or high.
 *
 * @ingroup PM
 */
#include <asm/io.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <mach/iram.h>
#include <mach/hardware.h>
#include <mach/clock.h>
#include <mach/mxc_dvfs.h>
#include <mach/sdram_autogating.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>
#include <asm/cacheflush.h>
#include <asm/tlb.h>
#include "crm_regs.h"
#include <linux/suspend.h>

#define LPAPM_CLK		24000000
#define DDR_AUDIO_CLK	100000000
#define DDR_MED_CLK		400000000
#define DDR3_NORMAL_CLK		528000000
#define GPC_PGC_GPU_PGCR_OFFSET	0x260
#define GPC_CNTR_OFFSET		0x0

static DEFINE_SPINLOCK(freq_lock);

int low_bus_freq_mode;
int audio_bus_freq_mode;
int high_bus_freq_mode;
int med_bus_freq_mode;

int bus_freq_scaling_initialized;
static struct device *busfreq_dev;
static int busfreq_suspended;

/* True if bus_frequency is scaled not using DVFS-PER */
int bus_freq_scaling_is_active;

int lp_high_freq;
int lp_med_freq;
int lp_audio_freq;
int high_cpu_freq;
unsigned int ddr_low_rate;
unsigned int ddr_med_rate;
unsigned int ddr_normal_rate;

int low_freq_bus_used(void);
void set_ddr_freq(int ddr_freq);
void *mx6sl_wfi_iram_base;
void (*mx6sl_wfi_iram)(int arm_podf, unsigned long wfi_iram_addr,\
		int audio_mode) = NULL;
extern void mx6sl_wait (int arm_podf, unsigned long wfi_iram_addr);

void *mx6sl_ddr_freq_base;
void (*mx6sl_ddr_freq_change_iram)(int ddr_freq, int low_bus_freq_mode) = NULL;
extern void mx6sl_ddr_iram(int ddr_freq);

extern int init_mmdc_settings(void);
extern struct cpu_op *(*get_cpu_op)(int *op);
extern int update_ddr_freq(int ddr_rate);
extern int chip_rev;

DEFINE_MUTEX(bus_freq_mutex);

struct timeval start_time;
struct timeval end_time;

static int cpu_op_nr;
static u32 org_arm_podf;
static struct cpu_op *cpu_op_tbl;
static struct clk *pll2_400;
static struct clk *axi_clk;
static struct clk *ahb_clk;
static struct clk *periph_clk;
static struct clk *osc_clk;
static struct clk *cpu_clk;
static struct clk *pll3;
static struct clk *pll2;
static struct clk *pll1;
static struct clk *pll1_sw_clk;
static struct clk *pll3_sw_clk;
static struct clk *pll2_200;
struct clk *mmdc_ch0_axi;
static struct clk *pll3_540;

static struct delayed_work low_bus_freq_handler;

void reduce_bus_freq(void)
{
	if (!cpu_is_mx6sl()) {
		if (cpu_is_mx6dl() &&
			(clk_get_parent(axi_clk) != periph_clk))
			/* Set the axi_clk to be sourced from the periph_clk.
			  * So that its frequency can be lowered down to 50MHz
			  * or 24MHz as the case may be.
			  */
			clk_set_parent(axi_clk, periph_clk);

		clk_enable(pll3);
		if (lp_audio_freq) {
			/* Need to ensure that PLL2_PFD_400M is kept ON. */
			clk_enable(pll2_400);
			update_ddr_freq(DDR_AUDIO_CLK);
			/* Make sure periph clk's parent also got updated */
			clk_set_parent(periph_clk, pll2_200);
			audio_bus_freq_mode = 1;
			low_bus_freq_mode = 0;
		} else {
			update_ddr_freq(LPAPM_CLK);
			/* Make sure periph clk's parent also got updated */
			clk_set_parent(periph_clk, osc_clk);
			if (audio_bus_freq_mode)
				clk_disable(pll2_400);
			low_bus_freq_mode = 1;
			audio_bus_freq_mode = 0;
		}

		if (med_bus_freq_mode)
			clk_disable(pll2_400);

		clk_disable(pll3);
		med_bus_freq_mode = 0;
	} else {
		u32 reg;
		u32  div;
		unsigned long flags;

		if (high_bus_freq_mode) {
			/* Set periph_clk to be sourced from OSC_CLK */
			/* Set AXI to 24MHz. */
			clk_set_parent(periph_clk, osc_clk);
			clk_set_rate(axi_clk,
				clk_round_rate(axi_clk, LPAPM_CLK));
			/* Set AHB to 24MHz. */
			clk_set_rate(ahb_clk,
				clk_round_rate(ahb_clk, LPAPM_CLK));
		}
		if (lp_audio_freq) {
			/* PLL2 is on in this mode, as DDR is at 50MHz. */
			/* Now change DDR freq while running from IRAM. */

			/* Set AHB to 24MHz. */
			clk_set_rate(ahb_clk,
				clk_round_rate(ahb_clk, LPAPM_CLK / 3));

			spin_lock_irqsave(&freq_lock, flags);
			mx6sl_ddr_freq_change_iram(DDR_AUDIO_CLK,
							low_bus_freq_mode);
			spin_unlock_irqrestore(&freq_lock, flags);

			if (low_bus_freq_mode) {
				/* Swtich ARM to run off PLL2_PFD2_400MHz
				 * since DDR is anway at 50MHz.
				 */
				clk_set_parent(pll1_sw_clk, pll2_400);

				/* Ensure that the clock will be
				  * at original speed.
				  */
				reg = __raw_writel(org_arm_podf, MXC_CCM_CACRR);
				while (__raw_readl(MXC_CCM_CDHIPR))
					;
				/* We have enabled PLL1 in the code below when
				  * ARM is from PLL1, so disable it here.
				  */
				clk_disable(pll1);
			}
			low_bus_freq_mode = 0;
			audio_bus_freq_mode = 1;
		} else {
			/* Set MMDC clk to 24MHz. */
			/* Since we are going to set PLL2 in bypass mode,
			  * move the CPU clock off PLL2.
			  */
			/* Ensure that the clock will be at
			  * lowest possible freq.
			  */
			org_arm_podf = __raw_readl(MXC_CCM_CACRR);
			/* Need to enable PLL1 before setting its rate. */
			clk_enable(pll1);
			clk_set_rate(pll1,
				cpu_op_tbl[cpu_op_nr - 1].pll_lpm_rate);
			div = clk_get_rate(pll1) /
					cpu_op_tbl[cpu_op_nr - 1].cpu_rate;

			reg = __raw_writel(div - 1, MXC_CCM_CACRR);
			while (__raw_readl(MXC_CCM_CDHIPR))
				;
			clk_set_parent(pll1_sw_clk, pll1);

			spin_lock_irqsave(&freq_lock, flags);
			/* Now change DDR freq while running from IRAM. */
			mx6sl_ddr_freq_change_iram(LPAPM_CLK,
					low_bus_freq_mode);
			spin_unlock_irqrestore(&freq_lock, flags);

			low_bus_freq_mode = 1;
			audio_bus_freq_mode = 0;
		}
	}
	high_bus_freq_mode = 0;

}

static void reduce_bus_freq_handler(struct work_struct *work)
{
	mutex_lock(&bus_freq_mutex);

	if (!low_freq_bus_used()) {
		mutex_unlock(&bus_freq_mutex);
		return;
	}
	/* If we are already in audio bus freq mode,
	  * just return if lp_audio_freq is true.
	  */
	if (audio_bus_freq_mode && lp_audio_freq) {
		mutex_unlock(&bus_freq_mutex);
		return;
	}

	/* If we dont want to transition from low bus to
	  * audio bus mode and are already in
	  *low bus mode, then return.
	  */
	if (!lp_audio_freq && low_bus_freq_mode) {
		mutex_unlock(&bus_freq_mutex);
		return;
	}
	reduce_bus_freq();

	mutex_unlock(&bus_freq_mutex);
}

/* Set the DDR, AHB to 24MHz.
  * This mode will be activated only when none of the modules that
  * need a higher DDR or AHB frequency are active.
  */
int set_low_bus_freq(void)
{
	if (busfreq_suspended)
		return 0;

	if (!bus_freq_scaling_initialized || !bus_freq_scaling_is_active)
		return 0;

	/* Check to see if we need to get from
	  * low bus freq mode to audio bus freq mode.
	  * If so, the change needs to be done immediately.
	  */
	if (lp_audio_freq && low_bus_freq_mode)
		reduce_bus_freq();
	else
		/* Don't lower the frequency immediately. Instead
		  * scheduled a delayed work and drop the freq if
		  * the conditions still remain the same.
		  */
		schedule_delayed_work(&low_bus_freq_handler,
					usecs_to_jiffies(3000000));
	return 0;
}

/* Set the DDR to either 528MHz or 400MHz for MX6q
 * or 400MHz for MX6DL.
 */
int set_high_bus_freq(int high_bus_freq)
{
	if (bus_freq_scaling_initialized && bus_freq_scaling_is_active)
		cancel_delayed_work_sync(&low_bus_freq_handler);

	if (busfreq_suspended)
		return 0;


	if (!bus_freq_scaling_initialized || !bus_freq_scaling_is_active)
		return 0;

	if (cpu_is_mx6sl())
		high_bus_freq = 1;

	if (high_bus_freq_mode && high_bus_freq)
		return 0;


	/* medium bus freq is only supported for MX6DQ */
	if (cpu_is_mx6q() && med_bus_freq_mode && !high_bus_freq)
		return 0;

	if (cpu_is_mx6dl() && high_bus_freq)
		high_bus_freq = 0;

	if (cpu_is_mx6dl() && med_bus_freq_mode)
		return 0;

	if ((high_bus_freq_mode && (high_bus_freq || lp_high_freq)) ||
	    (med_bus_freq_mode && !high_bus_freq && lp_med_freq &&
	     !lp_high_freq))
		return 0;

	if (cpu_is_mx6sl()) {
		u32 reg;
		unsigned long flags;

		spin_lock_irqsave(&freq_lock, flags);
		/* Change DDR freq in IRAM. */
		mx6sl_ddr_freq_change_iram(ddr_normal_rate, low_bus_freq_mode);
		spin_unlock_irqrestore(&freq_lock, flags);

		/* Set periph_clk to be sourced from pll2_pfd2_400M */
		/* First need to set the divider before changing the */
		/* parent if parent clock is larger than previous one */
		clk_set_rate(ahb_clk, clk_round_rate(ahb_clk,
						     LPAPM_CLK / 3));
		clk_set_rate(axi_clk,
			     clk_round_rate(axi_clk, LPAPM_CLK / 2));
		clk_set_parent(periph_clk, pll2_400);

		if (low_bus_freq_mode) {
			/* Now move ARM to be sourced from PLL2_400 too. */
			clk_set_parent(pll1_sw_clk, pll2_400);

			/* Ensure that the clock will be at original speed. */
			reg = __raw_writel(org_arm_podf, MXC_CCM_CACRR);
			while (__raw_readl(MXC_CCM_CDHIPR))
				;
			clk_disable(pll1);
		}
		high_bus_freq_mode = 1;
		low_bus_freq_mode = 0;
		audio_bus_freq_mode = 0;
	} else {
		clk_enable(pll3);
		if (high_bus_freq) {
			update_ddr_freq(ddr_normal_rate);
			/* Make sure periph clk's parent also got updated */
			clk_set_parent(periph_clk, pll2);
			if (med_bus_freq_mode)
				clk_disable(pll2_400);
			high_bus_freq_mode = 1;
			med_bus_freq_mode = 0;
		} else {
			clk_enable(pll2_400);
			update_ddr_freq(ddr_med_rate);
			/* Make sure periph clk's parent also got updated */
			clk_set_parent(periph_clk, pll2_400);
			high_bus_freq_mode = 0;
			med_bus_freq_mode = 1;
		}
		if (audio_bus_freq_mode)
			clk_disable(pll2_400);

		/* AXI_CLK is sourced from PLL3_PFD_540 on MX6DL */
		if (cpu_is_mx6dl() &&
			clk_get_parent(axi_clk) != pll3_540)
			clk_set_parent(axi_clk, pll3_540);

		low_bus_freq_mode = 0;
		audio_bus_freq_mode = 0;

		clk_disable(pll3);
	}
	return 0;
}

int low_freq_bus_used(void)
{
	if (!bus_freq_scaling_initialized)
		return 0;

	/* We only go the lowest setpoint if ARM is also
	 * at the lowest setpoint.
	 */
	if (high_cpu_freq)
		return 0;

	if ((lp_high_freq == 0)
	    && (lp_med_freq == 0))
		return 1;
	else
		return 0;
}

void bus_freq_update(struct clk *clk, bool flag)
{
	mutex_lock(&bus_freq_mutex);

	if (flag) {
		if (clk == cpu_clk) {
			/* The CPU freq is being increased.
			  * check if we need to increase the bus freq
			  */
			high_cpu_freq = 1;
			if (low_bus_freq_mode || audio_bus_freq_mode)
				set_high_bus_freq(1);
		} else {
			/* Update count */
			if (clk->flags & AHB_HIGH_SET_POINT)
				lp_high_freq++;
			else if (clk->flags & AHB_MED_SET_POINT)
				lp_med_freq++;
			else if (clk->flags & AHB_AUDIO_SET_POINT)
				lp_audio_freq++;
			/* Update bus freq */
			if ((clk->flags & CPU_FREQ_TRIG_UPDATE)
				&& (clk_get_usecount(clk) == 0)) {
				if (!(clk->flags &
					(AHB_HIGH_SET_POINT | AHB_MED_SET_POINT))) {
					if (low_freq_bus_used())
						set_low_bus_freq();
				} else {
					if ((clk->flags & AHB_MED_SET_POINT)
						&& !med_bus_freq_mode) {
						/* Set to Medium setpoint */
						set_high_bus_freq(0);
					} else if ((clk->flags & AHB_HIGH_SET_POINT)
						&& !high_bus_freq_mode) {
						/* Currently at low or medium
						  * set point, need to set to
						  * high setpoint
						  */
						set_high_bus_freq(1);
					}
				}
			}
		}
	} else {
		if (clk == cpu_clk) {
			/* CPU freq is dropped, check if we can
			  * lower the bus freq.
			  */
			high_cpu_freq = 0;

			if (low_freq_bus_used() &&
				!(low_bus_freq_mode || audio_bus_freq_mode))
				set_low_bus_freq();
		} else {
			/* Update count */
			if (clk->flags & AHB_HIGH_SET_POINT)
				lp_high_freq--;
			else if (clk->flags & AHB_MED_SET_POINT)
				lp_med_freq--;
			else if (clk->flags & AHB_AUDIO_SET_POINT)
				lp_audio_freq--;
			/* Update bus freq */
			if ((clk->flags & CPU_FREQ_TRIG_UPDATE)
				&& (clk_get_usecount(clk) == 0)) {
				if (low_freq_bus_used())
					set_low_bus_freq();
				else {
					/* Set to either high or
					  * medium setpoint.
					  */
					set_high_bus_freq(1);
				}
			}
		}
	}
	mutex_unlock(&bus_freq_mutex);
	return;
}
void setup_pll(void)
{
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
#ifdef CONFIG_MX6_VPU_352M
		if (cpu_is_mx6q())
			/*do not enable bus freq*/
			bus_freq_scaling_is_active = 0;
		printk(KERN_WARNING "Bus frequency can't be enabled if using VPU 352M!\n");
		return size;
#else
		bus_freq_scaling_is_active = 1;
#endif
		set_high_bus_freq(1);
		/* Make sure system can enter low bus mode if it should be in
		low bus mode */
		if (low_freq_bus_used() && !low_bus_freq_mode)
			set_low_bus_freq();
	} else if (strncmp(buf, "0", 1) == 0) {
		if (bus_freq_scaling_is_active)
			set_high_bus_freq(1);
		bus_freq_scaling_is_active = 0;
	}
	return size;
}

static int busfreq_suspend(struct platform_device *pdev, pm_message_t message)
{
	return 0;
}

static int bus_freq_pm_notify(struct notifier_block *nb, unsigned long event,
	void *dummy)
{
	mutex_lock(&bus_freq_mutex);

	if (event == PM_SUSPEND_PREPARE) {
		set_high_bus_freq(1);
		busfreq_suspended = 1;
	} else if (event == PM_POST_SUSPEND) {
		busfreq_suspended = 0;
	}

	mutex_unlock(&bus_freq_mutex);

	return NOTIFY_OK;
}
static int busfreq_resume(struct platform_device *pdev)
{
	return  0;
}
static struct notifier_block imx_bus_freq_pm_notifier = {
	.notifier_call = bus_freq_pm_notify,
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

static int __devinit busfreq_probe(struct platform_device *pdev)
{
	u32 err;

	busfreq_dev = &pdev->dev;

	pll2_400 = clk_get(NULL, "pll2_pfd_400M");
	if (IS_ERR(pll2_400)) {
		printk(KERN_DEBUG "%s: failed to get pll2_pfd_400M\n",
		       __func__);
		return PTR_ERR(pll2_400);
	}

	pll2_200 = clk_get(NULL, "pll2_200M");
	if (IS_ERR(pll2_200)) {
		printk(KERN_DEBUG "%s: failed to get pll2_200M\n",
		       __func__);
		return PTR_ERR(pll2_200);
	}

	pll2 = clk_get(NULL, "pll2");
	if (IS_ERR(pll2)) {
		printk(KERN_DEBUG "%s: failed to get pll2\n",
		       __func__);
		return PTR_ERR(pll2);
	}

	pll1 = clk_get(NULL, "pll1_main_clk");
	if (IS_ERR(pll1)) {
		printk(KERN_DEBUG "%s: failed to get pll1\n",
		       __func__);
		return PTR_ERR(pll1);
	}

	pll1_sw_clk = clk_get(NULL, "pll1_sw_clk");
	if (IS_ERR(pll1_sw_clk)) {
		printk(KERN_DEBUG "%s: failed to get pll1_sw_clk\n",
		       __func__);
		return PTR_ERR(pll1_sw_clk);
	}


	if (IS_ERR(pll2)) {
		printk(KERN_DEBUG "%s: failed to get pll2\n",
		       __func__);
		return PTR_ERR(pll2);
	}


	cpu_clk = clk_get(NULL, "cpu_clk");
	if (IS_ERR(cpu_clk)) {
		printk(KERN_DEBUG "%s: failed to get cpu_clk\n",
		       __func__);
		return PTR_ERR(cpu_clk);
	}

	pll3 = clk_get(NULL, "pll3_main_clk");
	if (IS_ERR(pll3)) {
		printk(KERN_DEBUG "%s: failed to get pll3\n",
		       __func__);
		return PTR_ERR(pll3);
	}

	pll3_540 = clk_get(NULL, "pll3_pfd_540M");
	if (IS_ERR(pll3_540)) {
		printk(KERN_DEBUG "%s: failed to get periph_clk\n",
		       __func__);
		return PTR_ERR(pll3_540);
	}

	pll3_sw_clk = clk_get(NULL, "pll3_sw_clk");
	if (IS_ERR(pll3_sw_clk)) {
		printk(KERN_DEBUG "%s: failed to get pll3_sw_clk\n",
		       __func__);
		return PTR_ERR(pll3_sw_clk);
	}

	axi_clk = clk_get(NULL, "axi_clk");
	if (IS_ERR(axi_clk)) {
		printk(KERN_DEBUG "%s: failed to get axi_clk\n",
		       __func__);
		return PTR_ERR(axi_clk);
	}

	ahb_clk = clk_get(NULL, "ahb");
	if (IS_ERR(ahb_clk)) {
		printk(KERN_DEBUG "%s: failed to get ahb_clk\n",
		       __func__);
		return PTR_ERR(ahb_clk);
	}

	periph_clk = clk_get(NULL, "periph_clk");
	if (IS_ERR(periph_clk)) {
		printk(KERN_DEBUG "%s: failed to get periph_clk\n",
		       __func__);
		return PTR_ERR(periph_clk);
	}

	osc_clk = clk_get(NULL, "osc");
	if (IS_ERR(osc_clk)) {
		printk(KERN_DEBUG "%s: failed to get osc_clk\n",
		       __func__);
		return PTR_ERR(osc_clk);
	}

	mmdc_ch0_axi = clk_get(NULL, "mmdc_ch0_axi");
	if (IS_ERR(mmdc_ch0_axi)) {
		printk(KERN_DEBUG "%s: failed to get mmdc_ch0_axi\n",
		       __func__);
		return PTR_ERR(mmdc_ch0_axi);
	}

	err = sysfs_create_file(&busfreq_dev->kobj, &dev_attr_enable.attr);
	if (err) {
		printk(KERN_ERR
		       "Unable to register sysdev entry for BUSFREQ");
		return err;
	}

	cpu_op_tbl = get_cpu_op(&cpu_op_nr);
	low_bus_freq_mode = 0;
	if (cpu_is_mx6dl()) {
		high_bus_freq_mode = 0;
		med_bus_freq_mode = 1;
		/* To make pll2_400 use count right, as when
		system enter 24M, it will disable pll2_400 */
		clk_enable(pll2_400);
	} else if (cpu_is_mx6sl()) {
		/* Set med_bus_freq_mode to 1 since med_bus_freq_mode
		is not supported as yet for MX6SL */
		high_bus_freq_mode = 1;
		med_bus_freq_mode = 1;
	} else {
		high_bus_freq_mode = 1;
		med_bus_freq_mode = 0;
	}
	bus_freq_scaling_is_active = 0;
	bus_freq_scaling_initialized = 1;

	if (cpu_is_mx6q()) {
		ddr_low_rate = LPAPM_CLK;
		ddr_med_rate = DDR_MED_CLK;
		ddr_normal_rate = DDR3_NORMAL_CLK;
	}
	if (cpu_is_mx6dl() || cpu_is_mx6sl()) {
		ddr_low_rate = LPAPM_CLK;
		ddr_normal_rate = ddr_med_rate = DDR_MED_CLK;
	}

	INIT_DELAYED_WORK(&low_bus_freq_handler, reduce_bus_freq_handler);
	register_pm_notifier(&imx_bus_freq_pm_notifier);

	if (!cpu_is_mx6sl())
		init_mmdc_settings();
	else {
		unsigned long iram_paddr;

		/* Allocate IRAM for WFI code when system is
		  * in low freq mode.
		  */
		iram_alloc(SZ_4K, &iram_paddr);
		/* Need to remap the area here since we want
		   * the memory region to be executable.
		   */
		mx6sl_wfi_iram_base = __arm_ioremap(iram_paddr,
						SZ_4K, MT_MEMORY_NONCACHED);
		memcpy(mx6sl_wfi_iram_base, mx6sl_wait, SZ_4K);
		mx6sl_wfi_iram = (void *)mx6sl_wfi_iram_base;

		/* Allocate IRAM for WFI code when system is
		  *in low freq mode.
		  */
		iram_alloc(SZ_4K, &iram_paddr);
		/* Need to remap the area here since we want the memory region
			 to be executable. */
		mx6sl_ddr_freq_base = __arm_ioremap(iram_paddr,
					SZ_4K, MT_MEMORY_NONCACHED);
		memcpy(mx6sl_ddr_freq_base, mx6sl_ddr_iram, SZ_4K);
		mx6sl_ddr_freq_change_iram = (void *)mx6sl_ddr_freq_base;

	}

	return 0;
}

static struct platform_driver busfreq_driver = {
	.driver = {
		   .name = "imx_busfreq",
		},
	.probe = busfreq_probe,
	.suspend = busfreq_suspend,
	.resume = busfreq_resume,
};

/*!
 * Initialise the busfreq_driver.
 *
 * @return  The function always returns 0.
 */

static int __init busfreq_init(void)
{
	if (platform_driver_register(&busfreq_driver) != 0) {
		printk(KERN_ERR "busfreq_driver register failed\n");
		return -ENODEV;
	}

	printk(KERN_INFO "Bus freq driver module loaded\n");

#ifdef CONFIG_MX6_VPU_352M
	if (cpu_is_mx6q())
		bus_freq_scaling_is_active = 0;/*disable bus_freq*/

#else
	/* Enable busfreq by default. */
	bus_freq_scaling_is_active = 1;
#endif
	if (cpu_is_mx6q())
		set_high_bus_freq(1);
	else if (cpu_is_mx6dl())
		set_high_bus_freq(0);

	printk(KERN_INFO "Bus freq driver Enabled\n");
	return 0;
}

static void __exit busfreq_cleanup(void)
{
	sysfs_remove_file(&busfreq_dev->kobj, &dev_attr_enable.attr);

	/* Unregister the device structure */
	platform_driver_unregister(&busfreq_driver);
	bus_freq_scaling_initialized = 0;
}

late_initcall(busfreq_init);
module_exit(busfreq_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("BusFreq driver");
MODULE_LICENSE("GPL");
