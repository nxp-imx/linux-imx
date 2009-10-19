/*
 *  Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <asm/cacheflush.h>
#include <asm/tlb.h>
#include <asm/mach/map.h>
#include <mach/hardware.h>
#include "crm_regs.h"

static struct device *pm_dev;
struct clk *gpc_dvfs_clk;
extern void cpu_do_suspend_workaround(u32 sdclk_iomux_addr);
extern void cpu_cortexa8_do_idle(void *);

extern int iram_ready;
void *suspend_iram_base;
void (*suspend_in_iram)(void *sdclk_iomux_addr) = NULL;

static int mx51_suspend_enter(suspend_state_t state)
{
	void __iomem *sdclk_iomux_addr = IO_ADDRESS(IOMUXC_BASE_ADDR + 0x4b8);

	if (gpc_dvfs_clk == NULL)
		gpc_dvfs_clk = clk_get(NULL, "gpc_dvfs_clk");
	/* gpc clock is needed for SRPG */
	clk_enable(gpc_dvfs_clk);
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

	if (tzic_enable_wake(0) != 0)
		return -EAGAIN;

	if (state == PM_SUSPEND_MEM) {
		local_flush_tlb_all();
		flush_cache_all();

		/* Run the suspend code from iRAM. */
		suspend_in_iram(sdclk_iomux_addr);

		/*clear the EMPGC0/1 bits */
		__raw_writel(0, MXC_SRPG_EMPGC0_SRPGCR);
		__raw_writel(0, MXC_SRPG_EMPGC1_SRPGCR);
	} else {
		if ((mxc_cpu_is_rev(CHIP_REV_2_0)) < 0) {
			/* do cpu_idle_workaround */
			u32 l2_iram_addr = IDLE_IRAM_BASE_ADDR;
			if (!iram_ready)
				return 0;
			if (l2_iram_addr > 0x1FFE8000)
				cpu_cortexa8_do_idle(IO_ADDRESS(l2_iram_addr));
		} else {
			cpu_do_idle();
		}
	}
	clk_disable(gpc_dvfs_clk);

	return 0;
}

/*
 * Called after processes are frozen, but before we shut down devices.
 */
static int mx51_suspend_prepare(void)
{
	return 0;
}

/*
 * Called before devices are re-setup.
 */
static void mx51_suspend_finish(void)
{
}

/*
 * Called after devices are re-setup, but before processes are thawed.
 */
static void mx51_suspend_end(void)
{
}

static int mx51_pm_valid(suspend_state_t state)
{
	return (state > PM_SUSPEND_ON && state <= PM_SUSPEND_MAX);
}

struct platform_suspend_ops mx51_suspend_ops = {
	.valid = mx51_pm_valid,
	.prepare = mx51_suspend_prepare,
	.enter = mx51_suspend_enter,
	.finish = mx51_suspend_finish,
	.end = mx51_suspend_end,
};


static int __devinit mx51_pm_probe(struct platform_device *pdev)
{
	pm_dev = &pdev->dev;
	return 0;
}

static struct platform_driver mx51_pm_driver = {
	.driver = {
		   .name = "mx51_pm",
		   },
	.probe = mx51_pm_probe,
};

static int __init pm_init(void)
{
	pr_info("Static Power Management for Freescale i.MX51\n");
	if (platform_driver_register(&mx51_pm_driver) != 0) {
		printk(KERN_ERR "mx51_pm_driver register failed\n");
		return -ENODEV;
	}
	suspend_set_ops(&mx51_suspend_ops);
	/* Move suspend routine into iRAM */
	suspend_iram_base = IO_ADDRESS(SUSPEND_IRAM_BASE_ADDR);
	memcpy(suspend_iram_base, cpu_do_suspend_workaround, SZ_4K);
	/* Need to remap the area here since we want the memory region
		 to be executable. */
	suspend_iram_base = __arm_ioremap(SUSPEND_IRAM_BASE_ADDR, SZ_4K,
										MT_HIGH_VECTORS);
	suspend_in_iram = (void *)suspend_iram_base;

	printk(KERN_INFO "PM driver module loaded\n");

	return 0;
}


static void __exit pm_cleanup(void)
{
	/* Unregister the device structure */
	platform_driver_unregister(&mx51_pm_driver);
}

module_init(pm_init);
module_exit(pm_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("PM driver");
MODULE_LICENSE("GPL");
