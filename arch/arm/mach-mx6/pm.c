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
#include <linux/delay.h>
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
#include <asm/hardware/cache-l2x0.h>
#include <asm/hardware/gic.h>
#ifdef CONFIG_ARCH_MX6Q
#include <mach/iomux-mx6q.h>
#endif
#include "crm_regs.h"
#include "src-reg.h"

#define SCU_CTRL_OFFSET				0x00
#define GPC_IMR1_OFFSET				0x08
#define GPC_IMR2_OFFSET				0x0c
#define GPC_IMR3_OFFSET				0x10
#define GPC_IMR4_OFFSET				0x14
#define GPC_ISR1_OFFSET				0x18
#define GPC_ISR2_OFFSET				0x1c
#define GPC_ISR3_OFFSET				0x20
#define GPC_ISR4_OFFSET				0x24
#define GPC_CNTR_OFFSET				0x0
#define GPC_PGC_GPU_PGCR_OFFSET		0x260
#define GPC_PGC_CPU_PDN_OFFSET		0x2a0
#define GPC_PGC_CPU_PUPSCR_OFFSET	0x2a4
#define GPC_PGC_CPU_PDNSCR_OFFSET	0x2a8
#define UART_UCR3_OFFSET			0x88
#define UART_USR1_OFFSET			0x94
#define UART_UCR3_AWAKEN			(1 << 4)
#define UART_USR1_AWAKE				(1 << 4)
#define LOCAL_TWD_LOAD_OFFSET		0x0
#define LOCAL_TWD_COUNT_OFFSET		0x4
#define LOCAL_TWD_CONTROL_OFFSET	0x8
#define LOCAL_TWD_INT_OFFSET		0xc
#define ANATOP_REG_2P5_OFFSET		0x130
#define ANATOP_REG_CORE_OFFSET		0x140

static struct clk *cpu_clk;
static struct pm_platform_data *pm_data;

#if defined(CONFIG_CPU_FREQ)
extern int set_cpu_freq(int wp);
#endif
extern void mx6q_suspend(suspend_state_t state);
extern void mx6_init_irq(void);
extern unsigned int gpc_wake_irq[4];

static struct device *pm_dev;
struct clk *gpc_dvfs_clk;
static void __iomem *scu_base;
static void __iomem *gpc_base;
static void __iomem *src_base;
static void __iomem *local_twd_base;
static void __iomem *gic_dist_base;
static void __iomem *gic_cpu_base;
static void __iomem *anatop_base;

static void *suspend_iram_base;
static void (*suspend_in_iram)(suspend_state_t state,
	unsigned long iram_paddr, unsigned long suspend_iram_base) = NULL;
static unsigned long iram_paddr, cpaddr;

static u32 ccm_ccr, ccm_clpcr, scu_ctrl;
static u32 gpc_imr[4], gpc_cpu_pup, gpc_cpu_pdn, gpc_cpu, gpc_ctr;
static u32 anatop[2], ccgr1, ccgr6;

static void gpu_power_down(void)
{
	int reg;

	/* enable power down request */
	reg = __raw_readl(gpc_base + GPC_PGC_GPU_PGCR_OFFSET);
	__raw_writel(reg | 0x1, gpc_base + GPC_PGC_GPU_PGCR_OFFSET);
	/* power down request */
	reg = __raw_readl(gpc_base + GPC_CNTR_OFFSET);
	__raw_writel(reg | 0x1, gpc_base + GPC_CNTR_OFFSET);
	/* disable clocks */
	__raw_writel(ccgr1 & 0xf0ffffff, MXC_CCM_CCGR1);
	__raw_writel(ccgr6 & 0x00003fff, MXC_CCM_CCGR6);
	/* power off pu */
	reg = __raw_readl(anatop_base + ANATOP_REG_CORE_OFFSET);
	reg &= ~0x0003fe00;
	__raw_writel(reg, anatop_base + ANATOP_REG_CORE_OFFSET);
}

static void gpu_power_up(void)
{
	int reg;
	/* power on pu */
	reg = __raw_readl(anatop_base + ANATOP_REG_CORE_OFFSET);
	reg &= ~0x0003fe00;
	reg |= 0x10 << 9; /* 1.1v */
	__raw_writel(reg, anatop_base + ANATOP_REG_CORE_OFFSET);
	mdelay(10);

	/* enable clocks */
	__raw_writel(ccgr1 | 0x0f000000, MXC_CCM_CCGR1);
	__raw_writel(ccgr6 | 0x0000c000, MXC_CCM_CCGR6);

	/* enable power up request */
	reg = __raw_readl(gpc_base + GPC_PGC_GPU_PGCR_OFFSET);
	__raw_writel(reg | 0x1, gpc_base + GPC_PGC_GPU_PGCR_OFFSET);
	/* power up request */
	reg = __raw_readl(gpc_base + GPC_CNTR_OFFSET);
	__raw_writel(reg | 0x2, gpc_base + GPC_CNTR_OFFSET);
	udelay(10);
}



static void mx6_suspend_store(void)
{
	/* save some settings before suspend */
	ccm_ccr = __raw_readl(MXC_CCM_CCR);
	ccm_clpcr = __raw_readl(MXC_CCM_CLPCR);
	ccgr1 = __raw_readl(MXC_CCM_CCGR1);
	ccgr6 = __raw_readl(MXC_CCM_CCGR6);
	scu_ctrl = __raw_readl(scu_base + SCU_CTRL_OFFSET);
	gpc_imr[0] = __raw_readl(gpc_base + GPC_IMR1_OFFSET);
	gpc_imr[1] = __raw_readl(gpc_base + GPC_IMR2_OFFSET);
	gpc_imr[2] = __raw_readl(gpc_base + GPC_IMR3_OFFSET);
	gpc_imr[3] = __raw_readl(gpc_base + GPC_IMR4_OFFSET);
	gpc_cpu_pup = __raw_readl(gpc_base + GPC_PGC_CPU_PUPSCR_OFFSET);
	gpc_cpu_pdn = __raw_readl(gpc_base + GPC_PGC_CPU_PDNSCR_OFFSET);
	gpc_cpu = __raw_readl(gpc_base + GPC_PGC_CPU_PDN_OFFSET);
	gpc_ctr = __raw_readl(gpc_base + GPC_CNTR_OFFSET);
	anatop[0] = __raw_readl(anatop_base + ANATOP_REG_2P5_OFFSET);
	anatop[1] = __raw_readl(anatop_base + ANATOP_REG_CORE_OFFSET);
}

static void mx6_suspend_restore(void)
{
	/* restore settings after suspend */
	__raw_writel(anatop[0], anatop_base + ANATOP_REG_2P5_OFFSET);
	__raw_writel(anatop[1], anatop_base + ANATOP_REG_CORE_OFFSET);
	udelay(50);
	__raw_writel(ccm_ccr, MXC_CCM_CCR);
	__raw_writel(ccm_clpcr, MXC_CCM_CLPCR);
	__raw_writel(scu_ctrl, scu_base + SCU_CTRL_OFFSET);
	__raw_writel(gpc_imr[0], gpc_base + GPC_IMR1_OFFSET);
	__raw_writel(gpc_imr[1], gpc_base + GPC_IMR2_OFFSET);
	__raw_writel(gpc_imr[2], gpc_base + GPC_IMR3_OFFSET);
	__raw_writel(gpc_imr[3], gpc_base + GPC_IMR4_OFFSET);
	__raw_writel(gpc_cpu_pup, gpc_base + GPC_PGC_CPU_PUPSCR_OFFSET);
	__raw_writel(gpc_cpu_pdn, gpc_base + GPC_PGC_CPU_PDNSCR_OFFSET);
	__raw_writel(gpc_cpu, gpc_base + GPC_PGC_CPU_PDN_OFFSET);

	__raw_writel(ccgr1, MXC_CCM_CCGR1);
	__raw_writel(ccgr6, MXC_CCM_CCGR6);
}

static int mx6_suspend_enter(suspend_state_t state)
{
	unsigned int wake_irq_isr[4];
	struct gic_dist_state gds;
	struct gic_cpu_state gcs;

	wake_irq_isr[0] = __raw_readl(gpc_base +
			GPC_ISR1_OFFSET) & gpc_wake_irq[0];
	wake_irq_isr[1] = __raw_readl(gpc_base +
			GPC_ISR2_OFFSET) & gpc_wake_irq[1];
	wake_irq_isr[2] = __raw_readl(gpc_base +
			GPC_ISR3_OFFSET) & gpc_wake_irq[2];
	wake_irq_isr[3] = __raw_readl(gpc_base +
			GPC_ISR4_OFFSET) & gpc_wake_irq[3];
	if (wake_irq_isr[0] | wake_irq_isr[1] |
			wake_irq_isr[2] | wake_irq_isr[3]) {
		printk(KERN_INFO "There are wakeup irq pending,system resume!\n");
		printk(KERN_INFO "wake_irq_isr[0-3]: 0x%x, 0x%x, 0x%x, 0x%x\n",
				wake_irq_isr[0], wake_irq_isr[1],
				wake_irq_isr[2], wake_irq_isr[3]);
		return 0;
	}
	mx6_suspend_store();

	switch (state) {
	case PM_SUSPEND_MEM:
		gpu_power_down();
		mxc_cpu_lp_set(ARM_POWER_OFF);
		break;
	case PM_SUSPEND_STANDBY:
		mxc_cpu_lp_set(STOP_POWER_OFF);
		break;
	default:
		return -EINVAL;
	}

	if (state == PM_SUSPEND_MEM || state == PM_SUSPEND_STANDBY) {
		if (pm_data && pm_data->suspend_enter)
			pm_data->suspend_enter();

		local_flush_tlb_all();
		flush_cache_all();

		if (state == PM_SUSPEND_MEM) {
			/* preserve gic state */
			save_gic_dist_state(0, &gds);
			save_gic_cpu_state(0, &gcs);
		}

		suspend_in_iram(state, (unsigned long)iram_paddr,
			(unsigned long)suspend_iram_base);

		if (state == PM_SUSPEND_MEM) {
			/* restore gic registers */
			restore_gic_dist_state(0, &gds);
			restore_gic_cpu_state(0, &gcs);
			gpu_power_up();
		}
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
	scu_base = IO_ADDRESS(SCU_BASE_ADDR);
	gpc_base = IO_ADDRESS(GPC_BASE_ADDR);
	src_base = IO_ADDRESS(SRC_BASE_ADDR);
	gic_dist_base = IO_ADDRESS(IC_DISTRIBUTOR_BASE_ADDR);
	gic_cpu_base = IO_ADDRESS(IC_INTERFACES_BASE_ADDR);
	local_twd_base = IO_ADDRESS(LOCAL_TWD_ADDR);
	anatop_base = IO_ADDRESS(ANATOP_BASE_ADDR);

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
					  MT_MEMORY_NONCACHED);
	pr_info("cpaddr = %x suspend_iram_base=%x\n",
		(unsigned int)cpaddr, (unsigned int)suspend_iram_base);

	/*
	 * Need to run the suspend code from IRAM as the DDR needs
	 * to be put into low power mode manually.
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

