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
#include <mach/hardware.h>
#include <asm/io.h>

#include "crm_regs.h"

struct cpu_op *(*get_cpu_op)(int *op);

int mx6_set_cpu_voltage(u32 cpu_volt)
{
	u32 reg, val;

	val = (cpu_volt - 725000) / 25000;

	reg = __raw_readl(ANADIG_REG_CORE);
	reg &= ~(ANADIG_REG_TARGET_MASK << ANADIG_REG0_CORE_TARGET_OFFSET);
	reg |= ((val + 1) << ANADIG_REG0_CORE_TARGET_OFFSET);

	__raw_writel(reg, ANADIG_REG_CORE);

	return 0;
}

static int __init post_cpu_init(void)
{
	unsigned int reg;
	void __iomem *base;

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

	return 0;
}

postcore_initcall(post_cpu_init);
