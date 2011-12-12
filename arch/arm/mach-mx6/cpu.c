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

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/iram_alloc.h>
#include <linux/delay.h>

#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/mach/map.h>

#include "crm_regs.h"
#include "cpu_op-mx6.h"


void *mx6_wait_in_iram_base;
void (*mx6_wait_in_iram)();
extern void mx6_wait(void);


struct cpu_op *(*get_cpu_op)(int *op);
bool enable_wait_mode;
u32 arm_max_freq = CPU_AT_1GHz;

void __iomem *gpc_base;
void __iomem *ccm_base;

static int cpu_silicon_rev = -1;
#define SI_REV_OFFSET 	0x48

static int get_mx6q_srev(void)
{
	void __iomem *romcp = ioremap(BOOT_ROM_BASE_ADDR, SZ_8K);
	u32 rev;

	if (!romcp) {
		cpu_silicon_rev = -EINVAL;
		return 0;
	}

	rev = __raw_readl(romcp + SI_REV_OFFSET);
	rev &= 0xff;

	iounmap(romcp);
	if (rev == 0x10)
		return IMX_CHIP_REVISION_1_0;
	else if (rev == 0x11)
		return IMX_CHIP_REVISION_1_1;
	else if (rev == 0x20)
		return IMX_CHIP_REVISION_2_0;
	return 0;
}

/*
 * Returns:
 *	the silicon revision of the cpu
 *	-EINVAL - not a mx50
 */
int mx6q_revision(void)
{
	if (!cpu_is_mx6q())
		return -EINVAL;

	if (cpu_silicon_rev == -1)
		cpu_silicon_rev = get_mx6q_srev();

	return cpu_silicon_rev;
}
EXPORT_SYMBOL(mx6q_revision);

static int __init post_cpu_init(void)
{
	unsigned int reg;
	void __iomem *base;
	unsigned long iram_paddr, cpaddr;


	iram_init(MX6Q_IRAM_BASE_ADDR, MX6Q_IRAM_SIZE);

	base = ioremap(AIPS1_ON_BASE_ADDR, PAGE_SIZE);
	__raw_writel(0x0, base + 0x40);
	__raw_writel(0x0, base + 0x44);
	__raw_writel(0x0, base + 0x48);
	__raw_writel(0x0, base + 0x4C);
	reg = __raw_readl(base + 0x50) & 0x00FFFFFF;
	__raw_writel(reg, base + 0x50);
	iounmap(base);

	base = ioremap(AIPS2_ON_BASE_ADDR, PAGE_SIZE);
	__raw_writel(0x0, base + 0x40);
	__raw_writel(0x0, base + 0x44);
	__raw_writel(0x0, base + 0x48);
	__raw_writel(0x0, base + 0x4C);
	reg = __raw_readl(base + 0x50) & 0x00FFFFFF;
	__raw_writel(reg, base + 0x50);
	iounmap(base);

	if (enable_wait_mode) {
		/* Allow SCU_CLK to be disabled when all cores are in WFI*/
		base = IO_ADDRESS(SCU_BASE_ADDR);
		reg = __raw_readl(base);
		reg |= 0x20;
		__raw_writel(reg, base);
	}

	/* Disable SRC warm reset to work aound system reboot issue */
	base = IO_ADDRESS(SRC_BASE_ADDR);
	reg = __raw_readl(base);
	reg &= ~0x1;
	__raw_writel(reg, base);

	/* Allocate IRAM for WAIT code. */
	/* Move wait routine into iRAM */
	cpaddr = (unsigned long)iram_alloc(SZ_4K, &iram_paddr);
	/* Need to remap the area here since we want the memory region
		 to be executable. */
	mx6_wait_in_iram_base = __arm_ioremap(iram_paddr, SZ_4K,
					  MT_MEMORY_NONCACHED);
	pr_info("cpaddr = %x wait_iram_base=%x\n",
		(unsigned int)cpaddr, (unsigned int)mx6_wait_in_iram_base);

	/*
	 * Need to run the suspend code from IRAM as the DDR needs
	 * to be put into low power mode manually.
	 */
	memcpy((void *)cpaddr, mx6_wait, SZ_4K);
	mx6_wait_in_iram = (void *)mx6_wait_in_iram_base;

	gpc_base = MX6_IO_ADDRESS(GPC_BASE_ADDR);
	ccm_base = MX6_IO_ADDRESS(CCM_BASE_ADDR);

	return 0;
}
postcore_initcall(post_cpu_init);

static int __init enable_wait(char *p)
{
	enable_wait_mode = true;
	return 0;
}
early_param("enable_wait_mode", enable_wait);

static int __init arm_core_max(char *p)
{
	if (memcmp(p, "1000", 4) == 0) {
		arm_max_freq = CPU_AT_1GHz;
		p += 4;
	} else if (memcmp(p, "800", 3) == 0) {
		arm_max_freq = CPU_AT_800MHz;
		p += 3;
	}
	return 0;
}

early_param("arm_freq", arm_core_max);


