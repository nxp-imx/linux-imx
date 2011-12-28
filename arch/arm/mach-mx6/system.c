/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/clock.h>
#include <asm/proc-fns.h>
#include <asm/system.h>
#include "crm_regs.h"

#define SCU_CTRL					0x00
#define SCU_CONFIG					0x04
#define SCU_CPU_STATUS				0x08
#define SCU_INVALIDATE				0x0c
#define SCU_FPGA_REVISION			0x10
#define GPC_CNTR_OFFSET				0x0
#define GPC_PGC_GPU_PGCR_OFFSET		0x260
#define GPC_PGC_CPU_PDN_OFFSET		0x2a0
#define GPC_PGC_CPU_PUPSCR_OFFSET	0x2a4
#define GPC_PGC_CPU_PDNSCR_OFFSET	0x2a8
#define ANATOP_REG_2P5_OFFSET		0x130
#define ANATOP_REG_CORE_OFFSET		0x140

#define MODULE_CLKGATE		(1 << 30)
#define MODULE_SFTRST		(1 << 31)
/* static DEFINE_SPINLOCK(wfi_lock); */

extern unsigned int gpc_wake_irq[4];
extern int mx6q_revision(void);

/* static unsigned int cpu_idle_mask; */

static void __iomem *gpc_base = IO_ADDRESS(GPC_BASE_ADDR);

extern void (*mx6_wait_in_iram)(void);
extern void mx6_wait(void);
extern void *mx6_wait_in_iram_base;
extern bool enable_wait_mode;

void gpc_set_wakeup(unsigned int irq[4])
{
	/* Mask all wake up source */
	__raw_writel(~irq[0], gpc_base + 0x8);
	__raw_writel(~irq[1], gpc_base + 0xc);
	__raw_writel(~irq[2], gpc_base + 0x10);
	__raw_writel(~irq[3], gpc_base + 0x14);

	return;
}
/* set cpu low power mode before WFI instruction */
void mxc_cpu_lp_set(enum mxc_cpu_pwr_mode mode)
{

	int stop_mode = 0;
	void __iomem *anatop_base = IO_ADDRESS(ANATOP_BASE_ADDR);
	u32 ccm_clpcr, anatop_val;

	ccm_clpcr = __raw_readl(MXC_CCM_CLPCR) & ~(MXC_CCM_CLPCR_LPM_MASK);

	switch (mode) {
	case WAIT_CLOCKED:
		break;
	case WAIT_UNCLOCKED:
		ccm_clpcr |= 0x1 << MXC_CCM_CLPCR_LPM_OFFSET;
		break;
	case WAIT_UNCLOCKED_POWER_OFF:
	case STOP_POWER_OFF:
	case ARM_POWER_OFF:
		if (mode == WAIT_UNCLOCKED_POWER_OFF) {
			ccm_clpcr &= ~MXC_CCM_CLPCR_VSTBY;
			ccm_clpcr &= ~MXC_CCM_CLPCR_SBYOS;
			ccm_clpcr |= 0x1 << MXC_CCM_CLPCR_LPM_OFFSET;
			ccm_clpcr |= MXC_CCM_CLPCR_BYP_MMDC_CH1_LPM_HS;
			stop_mode = 0;
		} else if (mode == STOP_POWER_OFF) {
			ccm_clpcr |= 0x2 << MXC_CCM_CLPCR_LPM_OFFSET;
			ccm_clpcr |= 0x3 << MXC_CCM_CLPCR_STBY_COUNT_OFFSET;
			ccm_clpcr |= MXC_CCM_CLPCR_VSTBY;
			ccm_clpcr |= MXC_CCM_CLPCR_SBYOS;
			ccm_clpcr |= MXC_CCM_CLPCR_BYP_MMDC_CH1_LPM_HS;
			stop_mode = 1;
		} else {
			ccm_clpcr |= 0x2 << MXC_CCM_CLPCR_LPM_OFFSET;
			ccm_clpcr |= 0x3 << MXC_CCM_CLPCR_STBY_COUNT_OFFSET;
			ccm_clpcr |= MXC_CCM_CLPCR_VSTBY;
			ccm_clpcr |= MXC_CCM_CLPCR_SBYOS;
			ccm_clpcr |= MXC_CCM_CLPCR_BYP_MMDC_CH1_LPM_HS;
			stop_mode = 2;
		}
		break;
	case STOP_POWER_ON:
		ccm_clpcr |= 0x2 << MXC_CCM_CLPCR_LPM_OFFSET;

		break;
	default:
		printk(KERN_WARNING "UNKNOWN cpu power mode: %d\n", mode);
		return;
	}

	if (stop_mode > 0) {
		gpc_set_wakeup(gpc_wake_irq);
		/* Power down and power up sequence */
		__raw_writel(0xFFFFFFFF, gpc_base + GPC_PGC_CPU_PUPSCR_OFFSET);
		__raw_writel(0xFFFFFFFF, gpc_base + GPC_PGC_CPU_PDNSCR_OFFSET);

		/* dormant mode, need to power off the arm core */
		if (stop_mode == 2) {
			__raw_writel(0x1, gpc_base + GPC_PGC_CPU_PDN_OFFSET);
			__raw_writel(0x1, gpc_base + GPC_PGC_GPU_PGCR_OFFSET);
			__raw_writel(0x1, gpc_base + GPC_CNTR_OFFSET);
			/* Enable weak 2P5 linear regulator */
			anatop_val = __raw_readl(anatop_base + ANATOP_REG_2P5_OFFSET);
			anatop_val |= 1 << 18;
			__raw_writel(anatop_val, anatop_base + ANATOP_REG_2P5_OFFSET);
			/* Make sure ARM and SOC domain has same voltage */
			anatop_val = __raw_readl(anatop_base + ANATOP_REG_CORE_OFFSET);
			anatop_val &= ~(0x1f << 18);
			anatop_val |= (anatop_val & 0x1f) << 18;
			__raw_writel(anatop_val, anatop_base + ANATOP_REG_CORE_OFFSET);
			__raw_writel(__raw_readl(MXC_CCM_CCR) | MXC_CCM_CCR_RBC_EN, MXC_CCM_CCR);
			ccm_clpcr |= MXC_CCM_CLPCR_WB_PER_AT_LPM;
		}
	}
	__raw_writel(ccm_clpcr, MXC_CCM_CLPCR);
}

 void arch_idle(void)
{
	if (enable_wait_mode) {
		if ((num_online_cpus() == num_present_cpus())
			&& mx6_wait_in_iram != NULL) {
			mxc_cpu_lp_set(WAIT_UNCLOCKED_POWER_OFF);
			if (smp_processor_id() == 0 &&
				(mx6q_revision() <= IMX_CHIP_REVISION_1_0))
				mx6_wait_in_iram();
			else
				cpu_do_idle();
		}
	} else
		cpu_do_idle();
}

static int __mxs_reset_block(void __iomem *hwreg, int just_enable)
{
	u32 c;
	int timeout;

	/* the process of software reset of IP block is done
	   in several steps:

	   - clear SFTRST and wait for block is enabled;
	   - clear clock gating (CLKGATE bit);
	   - set the SFTRST again and wait for block is in reset;
	   - clear SFTRST and wait for reset completion.
	 */
	c = __raw_readl(hwreg);
	c &= ~MODULE_SFTRST;	/* clear SFTRST */
	__raw_writel(c, hwreg);
	for (timeout = 1000000; timeout > 0; timeout--)
		/* still in SFTRST state ? */
		if ((__raw_readl(hwreg) & MODULE_SFTRST) == 0)
			break;
	if (timeout <= 0) {
		printk(KERN_ERR "%s(%p): timeout when enabling\n",
		       __func__, hwreg);
		return -ETIME;
	}

	c = __raw_readl(hwreg);
	c &= ~MODULE_CLKGATE;	/* clear CLKGATE */
	__raw_writel(c, hwreg);

	if (!just_enable) {
		c = __raw_readl(hwreg);
		c |= MODULE_SFTRST;	/* now again set SFTRST */
		__raw_writel(c, hwreg);
		for (timeout = 1000000; timeout > 0; timeout--)
			/* poll until CLKGATE set */
			if (__raw_readl(hwreg) & MODULE_CLKGATE)
				break;
		if (timeout <= 0) {
			printk(KERN_ERR "%s(%p): timeout when resetting\n",
			       __func__, hwreg);
			return -ETIME;
		}

		c = __raw_readl(hwreg);
		c &= ~MODULE_SFTRST;	/* clear SFTRST */
		__raw_writel(c, hwreg);
		for (timeout = 1000000; timeout > 0; timeout--)
			/* still in SFTRST state ? */
			if ((__raw_readl(hwreg) & MODULE_SFTRST) == 0)
				break;
		if (timeout <= 0) {
			printk(KERN_ERR "%s(%p): timeout when enabling "
			       "after reset\n", __func__, hwreg);
			return -ETIME;
		}

		c = __raw_readl(hwreg);
		c &= ~MODULE_CLKGATE;	/* clear CLKGATE */
		__raw_writel(c, hwreg);
	}
	for (timeout = 1000000; timeout > 0; timeout--)
		/* still in SFTRST state ? */
		if ((__raw_readl(hwreg) & MODULE_CLKGATE) == 0)
			break;

	if (timeout <= 0) {
		printk(KERN_ERR "%s(%p): timeout when unclockgating\n",
		       __func__, hwreg);
		return -ETIME;
	}

	return 0;
}

int mxs_reset_block(void __iomem *hwreg, int just_enable)
{
	int try = 10;
	int r;

	while (try--) {
		r = __mxs_reset_block(hwreg, just_enable);
		if (!r)
			break;
		pr_debug("%s: try %d failed\n", __func__, 10 - try);
	}
	return r;
}
EXPORT_SYMBOL(mxs_reset_block);
