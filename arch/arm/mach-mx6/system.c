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

#define SCU_CTRL		0x00
#define SCU_CONFIG		0x04
#define SCU_CPU_STATUS		0x08
#define SCU_INVALIDATE		0x0c
#define SCU_FPGA_REVISION	0x10

void gpc_enable_wakeup(unsigned int irq)
{
	void __iomem *gpc_base = IO_ADDRESS(GPC_BASE_ADDR);

	if ((irq < 32) || (irq > 158))
		printk(KERN_ERR "Invalid irq number!\n");

	/* Enable wake up source */
	__raw_writel(~(1 << (irq % 32)),
		gpc_base + 0x8 + (irq / 32 - 1) * 4);

}
/* set cpu low power mode before WFI instruction */
void mxc_cpu_lp_set(enum mxc_cpu_pwr_mode mode)
{
	void __iomem *scu_base = IO_ADDRESS(SCU_BASE_ADDR);
	void __iomem *gpc_base = IO_ADDRESS(GPC_BASE_ADDR);
	u32 scu_cr, ccm_clpcr;
	int stop_mode = 0;

	scu_cr = __raw_readl(scu_base + SCU_CTRL);
	ccm_clpcr = __raw_readl(MXC_CCM_CLPCR) & ~(MXC_CCM_CLPCR_LPM_MASK);

	switch (mode) {
	case WAIT_CLOCKED:
		break;
	case WAIT_UNCLOCKED:
		ccm_clpcr |= 0x1 << MXC_CCM_CLPCR_LPM_OFFSET;
		break;
	case WAIT_UNCLOCKED_POWER_OFF:
	case STOP_POWER_OFF:
		if (mode == WAIT_UNCLOCKED_POWER_OFF) {
			ccm_clpcr |= 0x1 << MXC_CCM_CLPCR_LPM_OFFSET;
			ccm_clpcr &= ~MXC_CCM_CLPCR_VSTBY;
			ccm_clpcr &= ~MXC_CCM_CLPCR_SBYOS;
			stop_mode = 0;
		} else {
			ccm_clpcr |= 0x2 << MXC_CCM_CLPCR_LPM_OFFSET;
			ccm_clpcr |= 0x3 << MXC_CCM_CLPCR_STBY_COUNT_OFFSET;
			ccm_clpcr |= MXC_CCM_CLPCR_VSTBY;
			ccm_clpcr |= MXC_CCM_CLPCR_SBYOS;
			ccm_clpcr |= MXC_CCM_CLPCR_BYP_MMDC_CH1_LPM_HS;
			stop_mode = 1;
		}
		/* scu standby enable, scu clk will be
		 * off after all cpu enter WFI */
		scu_cr |= 0x20;
		break;
	case STOP_POWER_ON:
		ccm_clpcr |= 0x2 << MXC_CCM_CLPCR_LPM_OFFSET;
		break;
	default:
		printk(KERN_WARNING "UNKNOWN cpu power mode: %d\n", mode);
		return;
	}

	if (stop_mode == 1) {
		/* Mask all wake up source */
		__raw_writel(0xFFFFFFFF, gpc_base + 0x8);
		__raw_writel(0xFFFFFFFF, gpc_base + 0xC);
		__raw_writel(0xFFFFFFFF, gpc_base + 0x10);
		__raw_writel(0xFFFFFFFF, gpc_base + 0x14);
		/* Power down and power up sequence */
		__raw_writel(0xFFFFFFFF, gpc_base + 0x2a4);
		__raw_writel(0xFFFFFFFF, gpc_base + 0x2a8);

		gpc_enable_wakeup(MXC_INT_UART4_ANDED);
	}

	__raw_writel(scu_cr, scu_base + SCU_CTRL);
	__raw_writel(ccm_clpcr, MXC_CCM_CLPCR);
}

void arch_idle(void)
{
	cpu_do_idle();
}
