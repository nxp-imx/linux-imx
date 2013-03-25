/*
 *  Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <mach/arc_otg.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/hardware/gic.h>
#ifdef CONFIG_ARCH_MX6Q
#include <mach/iomux-mx6q.h>
#endif
#include "crm_regs.h"
#include "src-reg.h"
#include "regs-anadig.h"

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
#define GPC_PGC_DISP_PGCR_OFFSET	0x240
#define GPC_PGC_DISP_PUPSCR_OFFSET	0x244
#define GPC_PGC_DISP_PDNSCR_OFFSET	0x248
#define GPC_PGC_DISP_SR_OFFSET		0x24c
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
#define VDD3P0_VOLTAGE                   3200000

static struct clk *cpu_clk;
static struct clk *axi_clk;
static struct clk *periph_clk;
static struct clk *pll3_usb_otg_main_clk;
static struct regulator *vdd3p0_regulator;

static struct pm_platform_data *pm_data;


#if defined(CONFIG_CPU_FREQ)
extern int set_cpu_freq(int wp);
#endif
extern void mx6_suspend(suspend_state_t state);
extern void mx6_init_irq(void);
extern unsigned int gpc_wake_irq[4];

extern bool enable_wait_mode;
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
	unsigned long iram_paddr, unsigned long suspend_iram_base, unsigned int cpu_type) = NULL;
static unsigned long iram_paddr, cpaddr;

static u32 ccm_ccr, ccm_clpcr, scu_ctrl;
static u32 gpc_imr[4], gpc_cpu_pup, gpc_cpu_pdn, gpc_cpu, gpc_ctr, gpc_disp;
static u32 anatop[2], ccgr1, ccgr2, ccgr3, ccgr6;
static u32 ccm_analog_pfd528;
static u32 ccm_analog_pll3_480;
static u32 ccm_anadig_ana_misc2;
static bool usb_vbus_wakeup_enabled;

/*
 * The USB VBUS wakeup should be disabled to avoid vbus wake system
 * up due to vbus comparator is closed at weak 2p5 mode.
 */
static void usb_power_down_handler(void)
{
	u32 temp;
	bool usb_oh3_clk_already_on;
	if ((__raw_readl(anatop_base + HW_ANADIG_ANA_MISC0)
		& BM_ANADIG_ANA_MISC0_STOP_MODE_CONFIG) != 0) {
			usb_vbus_wakeup_enabled = false;
			return;
	}
	/* enable usb oh3 clock if needed*/
	temp = __raw_readl(MXC_CCM_CCGR6);
	usb_oh3_clk_already_on =	\
		((temp & (MXC_CCM_CCGRx_CG_MASK << MXC_CCM_CCGRx_CG0_OFFSET))  \
		== (MXC_CCM_CCGRx_CG_MASK << MXC_CCM_CCGRx_CG0_OFFSET));
	if (!usb_oh3_clk_already_on) {
		temp |= MXC_CCM_CCGRx_CG_MASK << MXC_CCM_CCGRx_CG0_OFFSET;
		__raw_writel(temp, MXC_CCM_CCGR6);
	}
	/* disable vbus wakeup */
	usb_vbus_wakeup_enabled = !!(USB_OTG_CTRL & UCTRL_WKUP_VBUS_EN);
	if (usb_vbus_wakeup_enabled) {
		USB_OTG_CTRL &= ~UCTRL_WKUP_VBUS_EN;
	}
	/* disable usb oh3 clock */
	if (!usb_oh3_clk_already_on) {
		temp = __raw_readl(MXC_CCM_CCGR6);
		temp &= ~(MXC_CCM_CCGRx_CG_MASK << MXC_CCM_CCGRx_CG0_OFFSET);
		__raw_writel(temp, MXC_CCM_CCGR6);
	}
}

static void usb_power_up_handler(void)
{
	/* enable vbus wakeup at runtime if needed */
	if (usb_vbus_wakeup_enabled) {
		u32 temp;
		bool usb_oh3_clk_already_on;
		/* enable usb oh3 clock if needed*/
		temp = __raw_readl(MXC_CCM_CCGR6);
		usb_oh3_clk_already_on =	\
			((temp & (MXC_CCM_CCGRx_CG_MASK << MXC_CCM_CCGRx_CG0_OFFSET))  \
			== (MXC_CCM_CCGRx_CG_MASK << MXC_CCM_CCGRx_CG0_OFFSET));
		if (!usb_oh3_clk_already_on) {
			temp |= MXC_CCM_CCGRx_CG_MASK << MXC_CCM_CCGRx_CG0_OFFSET;
			__raw_writel(temp, MXC_CCM_CCGR6);
		}

		/* restore usb wakeup enable setting */
		USB_OTG_CTRL |= UCTRL_WKUP_VBUS_EN;

		/* disable usb oh3 clock */
		if (!usb_oh3_clk_already_on) {
			temp = __raw_readl(MXC_CCM_CCGR6);
			temp &= ~(MXC_CCM_CCGRx_CG_MASK << MXC_CCM_CCGRx_CG0_OFFSET);
			__raw_writel(temp, MXC_CCM_CCGR6);
		}
	}
}


static void disp_power_down(void)
{
	if (cpu_is_mx6sl() && (mx6sl_revision() >= IMX_CHIP_REVISION_1_2)) {

		__raw_writel(0xFFFFFFFF, gpc_base + GPC_PGC_DISP_PUPSCR_OFFSET);
		__raw_writel(0xFFFFFFFF, gpc_base + GPC_PGC_DISP_PDNSCR_OFFSET);

		__raw_writel(0x1, gpc_base + GPC_PGC_DISP_PGCR_OFFSET);
		__raw_writel(0x10, gpc_base + GPC_CNTR_OFFSET);

		/* Disable EPDC/LCDIF pix clock, and EPDC/LCDIF/PXP axi clock */
		__raw_writel(ccgr3 &
			~MXC_CCM_CCGRx_CG5_MASK &
			~MXC_CCM_CCGRx_CG4_MASK &
			~MXC_CCM_CCGRx_CG3_MASK &
			~MXC_CCM_CCGRx_CG2_MASK &
			~MXC_CCM_CCGRx_CG1_MASK, MXC_CCM_CCGR3);

	}
}

static void disp_power_up(void)
{
	if (cpu_is_mx6sl() && (mx6sl_revision() >= IMX_CHIP_REVISION_1_2)) {
		/*
		 * Need to enable EPDC/LCDIF pix clock, and
		 * EPDC/LCDIF/PXP axi clock before power up.
		 */
		__raw_writel(ccgr3 |
			MXC_CCM_CCGRx_CG5_MASK |
			MXC_CCM_CCGRx_CG4_MASK |
			MXC_CCM_CCGRx_CG3_MASK |
			MXC_CCM_CCGRx_CG2_MASK |
			MXC_CCM_CCGRx_CG1_MASK, MXC_CCM_CCGR3);

		__raw_writel(0x0, gpc_base + GPC_PGC_DISP_PGCR_OFFSET);
		__raw_writel(0x20, gpc_base + GPC_CNTR_OFFSET);
		__raw_writel(0x1, gpc_base + GPC_PGC_DISP_SR_OFFSET);
	}
}

static void mx6_suspend_store(void)
{
	/* save some settings before suspend */
	ccm_ccr = __raw_readl(MXC_CCM_CCR);
	ccm_clpcr = __raw_readl(MXC_CCM_CLPCR);
	ccm_analog_pfd528 = __raw_readl(PFD_528_BASE_ADDR);
	ccm_analog_pll3_480 = __raw_readl(PLL3_480_USB1_BASE_ADDR);
	ccm_anadig_ana_misc2 = __raw_readl(MXC_PLL_BASE + HW_ANADIG_ANA_MISC2);
	ccgr1 = __raw_readl(MXC_CCM_CCGR1);
	ccgr2 = __raw_readl(MXC_CCM_CCGR2);
	ccgr3 = __raw_readl(MXC_CCM_CCGR3);
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
	if (cpu_is_mx6sl())
		gpc_disp = __raw_readl(gpc_base + GPC_PGC_DISP_PGCR_OFFSET);
	anatop[0] = __raw_readl(anatop_base + ANATOP_REG_2P5_OFFSET);
	anatop[1] = __raw_readl(anatop_base + ANATOP_REG_CORE_OFFSET);
}

static void mx6_suspend_restore(void)
{
	/* restore settings after suspend */
	__raw_writel(anatop[0], anatop_base + ANATOP_REG_2P5_OFFSET);
	__raw_writel(anatop[1], anatop_base + ANATOP_REG_CORE_OFFSET);
	/* Per spec, the count needs to be zeroed and reconfigured on exit from
	 * low power mode
	 */
	__raw_writel(ccm_ccr & ~MXC_CCM_CCR_REG_BYPASS_CNT_MASK &
		~MXC_CCM_CCR_WB_COUNT_MASK, MXC_CCM_CCR);
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
	if (cpu_is_mx6sl())
		__raw_writel(gpc_disp, gpc_base + GPC_PGC_DISP_PGCR_OFFSET);
	__raw_writel(ccgr1, MXC_CCM_CCGR1);
	__raw_writel(ccgr2, MXC_CCM_CCGR2);
	__raw_writel(ccgr3, MXC_CCM_CCGR3);
	__raw_writel(ccgr6, MXC_CCM_CCGR6);
	__raw_writel(ccm_analog_pfd528, PFD_528_BASE_ADDR);
	__raw_writel(ccm_analog_pll3_480, PLL3_480_USB1_BASE_ADDR);
	__raw_writel(ccm_anadig_ana_misc2, MXC_PLL_BASE + HW_ANADIG_ANA_MISC2);
}

static int mx6_suspend_enter(suspend_state_t state)
{
	unsigned int wake_irq_isr[4];
	unsigned int cpu_type;
	struct gic_dist_state gds;
	struct gic_cpu_state gcs;
	bool arm_pg = false;

	if (cpu_is_mx6q())
		cpu_type = MXC_CPU_MX6Q;
	else if (cpu_is_mx6dl())
		cpu_type = MXC_CPU_MX6DL;
	else
		cpu_type = MXC_CPU_MX6SL;

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

	/*
	 * i.MX6dl TO1.0/i.MX6dq TO1.1/1.0 TKT094231: can't support
	 * ARM_POWER_OFF mode.
	 */
	if (state == PM_SUSPEND_MEM &&
		((mx6dl_revision() == IMX_CHIP_REVISION_1_0) ||
		(cpu_is_mx6q() && mx6q_revision() <= IMX_CHIP_REVISION_1_1))) {
		state = PM_SUSPEND_STANDBY;
	}

	switch (state) {
	case PM_SUSPEND_MEM:
		disp_power_down();
		usb_power_down_handler();
		mxc_cpu_lp_set(ARM_POWER_OFF);
		arm_pg = true;
		break;
	case PM_SUSPEND_STANDBY:
		if (cpu_is_mx6sl()) {
			disp_power_down();
			usb_power_down_handler();
			mxc_cpu_lp_set(STOP_XTAL_ON);
			arm_pg = true;
		} else
			mxc_cpu_lp_set(STOP_POWER_OFF);
		break;
	default:
		return -EINVAL;
	}

	/*
	 * L2 can exit by 'reset' or Inband beacon (from remote EP)
	 * toggling phy_powerdown has same effect as 'inband beacon'
	 * So, toggle bit18 of GPR1, to fix errata
	 * "PCIe PCIe does not support L2 Power Down"
	 */
	__raw_writel(__raw_readl(IOMUXC_GPR1) | (1 << 18), IOMUXC_GPR1);

	if (state == PM_SUSPEND_MEM || state == PM_SUSPEND_STANDBY) {

		local_flush_tlb_all();
		flush_cache_all();

		if (arm_pg) {
			/* preserve gic state */
			save_gic_dist_state(0, &gds);
			save_gic_cpu_state(0, &gcs);
		}

		if (pm_data && pm_data->suspend_enter)
			pm_data->suspend_enter();

		suspend_in_iram(state, (unsigned long)iram_paddr,
			(unsigned long)suspend_iram_base, cpu_type);

		if (pm_data && pm_data->suspend_exit)
			pm_data->suspend_exit();

		/* Reset the RBC counter. */
		/* All interrupts should be masked before the
		  * RBC counter is reset.
		 */
		/* Mask all interrupts. These will be unmasked by
		  * the mx6_suspend_restore routine below.
		  */
		__raw_writel(0xffffffff, gpc_base + 0x08);
		__raw_writel(0xffffffff, gpc_base + 0x0c);
		__raw_writel(0xffffffff, gpc_base + 0x10);
		__raw_writel(0xffffffff, gpc_base + 0x14);

		/* Clear the RBC counter and RBC_EN bit. */
		/* Disable the REG_BYPASS_COUNTER. */
		__raw_writel(__raw_readl(MXC_CCM_CCR) &
			~MXC_CCM_CCR_RBC_EN, MXC_CCM_CCR);
		/* Make sure we clear REG_BYPASS_COUNT*/
		__raw_writel(__raw_readl(MXC_CCM_CCR) &
		(~MXC_CCM_CCR_REG_BYPASS_CNT_MASK), MXC_CCM_CCR);
		/* Need to wait for a minimum of 2 CLKILS (32KHz) for the
		  * counter to clear and reset.
		  */
		udelay(80);

		if (arm_pg) {
			/* restore gic registers */
			restore_gic_dist_state(0, &gds);
			restore_gic_cpu_state(0, &gcs);
		}
		if (state == PM_SUSPEND_MEM || (cpu_is_mx6sl())) {
			usb_power_up_handler();
			disp_power_up();
		}

		mx6_suspend_restore();

		__raw_writel(BM_ANADIG_ANA_MISC0_STOP_MODE_CONFIG,
			anatop_base + HW_ANADIG_ANA_MISC0_CLR);

	} else {
			cpu_do_idle();
	}

	/*
	 * L2 can exit by 'reset' or Inband beacon (from remote EP)
	 * toggling phy_powerdown has same effect as 'inband beacon'
	 * So, toggle bit18 of GPR1, to fix errata
	 * "PCIe PCIe does not support L2 Power Down"
	 */
	__raw_writel(__raw_readl(IOMUXC_GPR1) & (~(1 << 18)), IOMUXC_GPR1);

	return 0;
}


/*
 * Called after processes are frozen, but before we shut down devices.
 */
static int mx6_suspend_prepare(void)
{
	int ret;
	ret = regulator_disable(vdd3p0_regulator);
	if (ret) {
		printk(KERN_ERR "%s: failed to disable 3p0 regulator Err: %d\n",
							__func__, ret);
	}
	return 0;
}

/*
 * Called before devices are re-setup.
 */
static void mx6_suspend_finish(void)
{
	int ret;
	ret = regulator_enable(vdd3p0_regulator);
	if (ret) {
		printk(KERN_ERR "%s: failed to enable 3p0 regulator Err: %d\n",
						__func__, ret);
	}
}

static int mx6_suspend_begin(suspend_state_t state)
{
	return 0;
}

/*
 * Called after devices are re-setup, but before processes are thawed.
 */

static int mx6_pm_valid(suspend_state_t state)
{
	return (state > PM_SUSPEND_ON && state <= PM_SUSPEND_MAX);
}

struct platform_suspend_ops mx6_suspend_ops = {
	.valid = mx6_pm_valid,
	.begin = mx6_suspend_begin,
	.prepare = mx6_suspend_prepare,
	.enter = mx6_suspend_enter,
	.finish = mx6_suspend_finish,
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
	int ret = 0;
	scu_base = IO_ADDRESS(SCU_BASE_ADDR);
	gpc_base = IO_ADDRESS(GPC_BASE_ADDR);
	src_base = IO_ADDRESS(SRC_BASE_ADDR);
	gic_dist_base = IO_ADDRESS(IC_DISTRIBUTOR_BASE_ADDR);
	gic_cpu_base = IO_ADDRESS(IC_INTERFACES_BASE_ADDR);
	local_twd_base = IO_ADDRESS(LOCAL_TWD_ADDR);
	anatop_base = IO_ADDRESS(ANATOP_BASE_ADDR);

	pr_info("Static Power Management for Freescale i.MX6\n");

	pr_info("wait mode is %s for i.MX6\n", enable_wait_mode ?
			"enabled" : "disabled");

	if (platform_driver_register(&mx6_pm_driver) != 0) {
		printk(KERN_ERR "mx6_pm_driver register failed\n");
		return -ENODEV;
	}

	suspend_set_ops(&mx6_suspend_ops);
	/* Move suspend routine into iRAM */
	cpaddr = (unsigned long)iram_alloc(SZ_8K, &iram_paddr);
	/* Need to remap the area here since we want the memory region
		 to be executable. */
	suspend_iram_base = __arm_ioremap(iram_paddr, SZ_8K,
					  MT_MEMORY_NONCACHED);
	pr_info("cpaddr = %x suspend_iram_base=%x\n",
		(unsigned int)cpaddr, (unsigned int)suspend_iram_base);

	/*
	 * Need to run the suspend code from IRAM as the DDR needs
	 * to be put into low power mode manually.
	 */
	memcpy((void *)cpaddr, mx6_suspend, SZ_8K);

	suspend_in_iram = (void *)suspend_iram_base;

	cpu_clk = clk_get(NULL, "cpu_clk");
	if (IS_ERR(cpu_clk)) {
		printk(KERN_DEBUG "%s: failed to get cpu_clk\n", __func__);
		return PTR_ERR(cpu_clk);
	}
	axi_clk = clk_get(NULL, "axi_clk");
	if (IS_ERR(axi_clk)) {
		printk(KERN_DEBUG "%s: failed to get axi_clk\n", __func__);
		return PTR_ERR(axi_clk);
	}
	periph_clk = clk_get(NULL, "periph_clk");
	if (IS_ERR(periph_clk)) {
		printk(KERN_DEBUG "%s: failed to get periph_clk\n", __func__);
		return PTR_ERR(periph_clk);
	}
	pll3_usb_otg_main_clk = clk_get(NULL, "pll3_main_clk");
	if (IS_ERR(pll3_usb_otg_main_clk)) {
		printk(KERN_DEBUG "%s: failed to get pll3_main_clk\n", __func__);
		return PTR_ERR(pll3_usb_otg_main_clk);
	}

	vdd3p0_regulator = regulator_get(NULL, "cpu_vdd3p0");
	if (IS_ERR(vdd3p0_regulator)) {
		printk(KERN_ERR "%s: failed to get 3p0 regulator Err: %d\n",
						__func__, ret);
		return PTR_ERR(vdd3p0_regulator);
	}
	ret = regulator_set_voltage(vdd3p0_regulator, VDD3P0_VOLTAGE,
							VDD3P0_VOLTAGE);
	if (ret) {
		printk(KERN_ERR "%s: failed to set 3p0 regulator voltage Err: %d\n",
						__func__, ret);
	}
	ret = regulator_enable(vdd3p0_regulator);
	if (ret) {
		printk(KERN_ERR "%s: failed to enable 3p0 regulator Err: %d\n",
						__func__, ret);
	}

	printk(KERN_INFO "PM driver module loaded\n");

	return 0;
}

static void __exit pm_cleanup(void)
{
	/* Unregister the device structure */
	platform_driver_unregister(&mx6_pm_driver);
	regulator_put(vdd3p0_regulator);
}

module_init(pm_init);
module_exit(pm_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("PM driver");
MODULE_LICENSE("GPL");

