/*
 *  Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/suspend.h>
#include <linux/regulator/machine.h>
#include <linux/proc_fs.h>
#include <linux/iram_alloc.h>
#include <linux/fsl_devices.h>
#include <asm/mach-types.h>
#include <asm/cacheflush.h>
#include <asm/tlb.h>
#include <asm/delay.h>
#include <asm/mach/map.h>
#include <mach/hardware.h>
#include <mach/imx-pm.h>
#ifdef CONFIG_ARCH_MX61
#include <mach/iomux-mx6q.h>
#endif
#include "crm_regs.h"

#define SCU_CTRL_OFFSET		0x00
#define GPC_IMR1_OFFSET		0x08
#define GPC_IMR2_OFFSET		0x0c
#define GPC_IMR3_OFFSET		0x10
#define GPC_IMR4_OFFSET		0x14
#define GPC_PGC_CPU_PUPSCR_OFFSET	0x2a4
#define GPC_PGC_CPU_PDNSCR_OFFSET	0x2a8
#define UART_UCR3_OFFSET		0x88
#define UART_USR1_OFFSET		0x94
#define UART_UCR3_AWAKEN		(1 << 4)
#define UART_USR1_AWAKE			(1 << 4)
static struct clk *cpu_clk;
static struct pm_platform_data *pm_data;

#if defined(CONFIG_CPU_FREQ)
static int org_freq;
extern int set_cpu_freq(int wp);
#endif

extern void mx6q_suspend(u32 m4if_addr);
static struct device *pm_dev;
struct clk *gpc_dvfs_clk;
void *suspend_iram_base;
void (*suspend_in_iram)(void *sdclk_iomux_addr) = NULL;
void __iomem *suspend_param1;

static void __iomem *scu_base;
static void __iomem *gpc_base;
static u32 ccm_clpcr, scu_ctrl;
static u32 gpc_imr[4], gpc_cpu_pup, gpc_cpu_pdn;

static void mx6_suspend_store()
{
	/* save some settings before suspend */
	ccm_clpcr = __raw_readl(MXC_CCM_CLPCR);
	scu_ctrl = __raw_readl(scu_base + SCU_CTRL_OFFSET);
	gpc_imr[0] = __raw_readl(gpc_base + GPC_IMR1_OFFSET);
	gpc_imr[1] = __raw_readl(gpc_base + GPC_IMR2_OFFSET);
	gpc_imr[2] = __raw_readl(gpc_base + GPC_IMR3_OFFSET);
	gpc_imr[3] = __raw_readl(gpc_base + GPC_IMR4_OFFSET);
	gpc_cpu_pup = __raw_readl(gpc_base + GPC_PGC_CPU_PUPSCR_OFFSET);
	gpc_cpu_pdn = __raw_readl(gpc_base + GPC_PGC_CPU_PDNSCR_OFFSET);
}

static void mx6_suspend_restore()
{
	/* restore settings after suspend */
	__raw_writel(ccm_clpcr, MXC_CCM_CLPCR);
	__raw_writel(scu_ctrl, scu_base + SCU_CTRL_OFFSET);
	__raw_writel(gpc_imr[0], gpc_base + GPC_IMR1_OFFSET);
	__raw_writel(gpc_imr[1], gpc_base + GPC_IMR2_OFFSET);
	__raw_writel(gpc_imr[2], gpc_base + GPC_IMR3_OFFSET);
	__raw_writel(gpc_imr[3], gpc_base + GPC_IMR4_OFFSET);
	__raw_writel(gpc_cpu_pup, gpc_base + GPC_PGC_CPU_PUPSCR_OFFSET);
	__raw_writel(gpc_cpu_pdn, gpc_base + GPC_PGC_CPU_PDNSCR_OFFSET);
}
static int mx6_suspend_enter(suspend_state_t state)
{
	mx6_suspend_store();

	switch (state) {
	case PM_SUSPEND_MEM:
		mxc_cpu_lp_set(STOP_POWER_OFF);
		break;
	case PM_SUSPEND_STANDBY:
		mxc_cpu_lp_set(WAIT_UNCLOCKED_POWER_OFF);
		break;
	default:
		return -EINVAL;
	}

	if (state == PM_SUSPEND_MEM) {
		if (pm_data && pm_data->suspend_enter)
			pm_data->suspend_enter();

		local_flush_tlb_all();
		flush_cache_all();
		outer_cache.flush_all();

		suspend_in_iram(suspend_param1);

		mx6_suspend_restore();
		if (pm_data && pm_data->suspend_exit)
			pm_data->suspend_exit();
	} else {
			cpu_do_idle();
	}
	return 0;
}


/*
 * Called after processes are frozen, but before we shut down devices.
 */
static int mx6_suspend_prepare(void)
{
	return 0;
}

/*
 * Called before devices are re-setup.
 */
static void mx6_suspend_finish(void)
{
#if defined(CONFIG_CPU_FREQ)
	struct cpufreq_freqs freqs;

	freqs.old = clk_get_rate(cpu_clk) / 1000;
	freqs.new = org_freq / 1000;
	freqs.cpu = 0;
	freqs.flags = 0;

	if (org_freq != clk_get_rate(cpu_clk)) {
		set_cpu_freq(org_freq);
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	}
#endif
}

/*
 * Called after devices are re-setup, but before processes are thawed.
 */
static void mx6_suspend_end(void)
{
}

static int mx6_pm_valid(suspend_state_t state)
{
	return (state > PM_SUSPEND_ON && state <= PM_SUSPEND_MAX);
}

struct platform_suspend_ops mx6_suspend_ops = {
	.valid = mx6_pm_valid,
	.prepare = mx6_suspend_prepare,
	.enter = mx6_suspend_enter,
	.finish = mx6_suspend_finish,
	.end = mx6_suspend_end,
};

static int __devinit mx6_pm_probe(struct platform_device *pdev)
{
	pm_dev = &pdev->dev;
	pm_data = pdev->dev.platform_data;

	return 0;
}

static struct platform_driver mx6_pm_driver = {
	.driver = {
		   .name = "imx_pm",
		   },
	.probe = mx6_pm_probe,
};

static int __init pm_init(void)
{
	unsigned long iram_paddr, cpaddr;

	scu_base = IO_ADDRESS(SCU_BASE_ADDR);
	gpc_base = IO_ADDRESS(GPC_BASE_ADDR);

	pr_info("Static Power Management for Freescale i.MX6\n");

	if (platform_driver_register(&mx6_pm_driver) != 0) {
		printk(KERN_ERR "mx6_pm_driver register failed\n");
		return -ENODEV;
	}

	suspend_set_ops(&mx6_suspend_ops);
	/* Move suspend routine into iRAM */
	cpaddr = (unsigned long)iram_alloc(SZ_4K, &iram_paddr);
	/* Need to remap the area here since we want the memory region
		 to be executable. */
	suspend_iram_base = __arm_ioremap(iram_paddr, SZ_4K,
					  MT_HIGH_VECTORS);
	pr_info("cpaddr = %x suspend_iram_base=%x\n",
		(unsigned int)cpaddr, (unsigned int)suspend_iram_base);

	/*
	 * Need to run the suspend code from IRAM as the DDR needs
	 * to be put into self refresh mode manually.
	 */
	memcpy((void *)cpaddr, mx6q_suspend, SZ_4K);

	suspend_in_iram = (void *)suspend_iram_base;

	cpu_clk = clk_get(NULL, "cpu_clk");
	if (IS_ERR(cpu_clk)) {
		printk(KERN_DEBUG "%s: failed to get cpu_clk\n", __func__);
		return PTR_ERR(cpu_clk);
	}
	printk(KERN_INFO "PM driver module loaded\n");

	return 0;
}


static void __exit pm_cleanup(void)
{
	/* Unregister the device structure */
	platform_driver_unregister(&mx6_pm_driver);
}

module_init(pm_init);
module_exit(pm_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("PM driver");
MODULE_LICENSE("GPL");

