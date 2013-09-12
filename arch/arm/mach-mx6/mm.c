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

/*
 * Create static mapping between physical to virtual memory.
 */

#include <linux/mm.h>
#include <linux/init.h>

#include <asm/mach/map.h>
#include <mach/iomux-v3.h>

#include <mach/hardware.h>
#include <mach/common.h>
#include <mach/iomux-v3.h>
#include <asm/hardware/cache-l2x0.h>
#include "crm_regs.h"

/*!
 * This structure defines the MX6 memory map.
 */
static struct map_desc mx6_io_desc[] __initdata = {
	{
	.virtual = BOOT_ROM_BASE_ADDR_VIRT,
	.pfn = __phys_to_pfn(BOOT_ROM_BASE_ADDR),
	.length = ROMCP_SIZE,
	.type = MT_DEVICE},
	{
	.virtual = AIPS1_BASE_ADDR_VIRT,
	.pfn = __phys_to_pfn(AIPS1_ARB_BASE_ADDR),
	.length = AIPS1_SIZE,
	.type = MT_DEVICE},
	{
	.virtual = AIPS2_BASE_ADDR_VIRT,
	.pfn = __phys_to_pfn(AIPS2_ARB_BASE_ADDR),
	.length = AIPS2_SIZE,
	.type = MT_DEVICE},
	{
	.virtual = ARM_PERIPHBASE_VIRT,
	.pfn = __phys_to_pfn(ARM_PERIPHBASE),
	.length = ARM_PERIPHBASE_SIZE,
	.type = MT_DEVICE},
};

static void mx6_set_cpu_type(void)
{
	u32 cpu_type = readl(IO_ADDRESS(ANATOP_BASE_ADDR + 0x280));

	cpu_type >>= 16;
	if (cpu_type == 0x60) {
		mxc_set_cpu_type(MXC_CPU_MX6SL);
		imx_print_silicon_rev("i.MX6SoloLite", mx6sl_revision());
		return;
	}

	cpu_type = readl(IO_ADDRESS(ANATOP_BASE_ADDR + 0x260));
	cpu_type >>= 16;
	if (cpu_type == 0x63) {
		mxc_set_cpu_type(MXC_CPU_MX6Q);
		imx_print_silicon_rev("i.MX6Q", mx6q_revision());
	} else if (cpu_type == 0x61) {
		mxc_set_cpu_type(MXC_CPU_MX6DL);
		imx_print_silicon_rev("i.MX6DL/SOLO", mx6dl_revision());
	} else
		pr_err("Unknown CPU type: %x\n", cpu_type);
}

/*!
 * This function initializes the memory map. It is called during the
 * system startup to create static physical to virtual memory map for
 * the IO modules.
 */
void __init mx6_map_io(void)
{
	iotable_init(mx6_io_desc, ARRAY_SIZE(mx6_io_desc));
	mxc_iomux_v3_init(IO_ADDRESS(MX6Q_IOMUXC_BASE_ADDR));
	mxc_arch_reset_init(IO_ADDRESS(MX6Q_WDOG1_BASE_ADDR));
	mx6_set_cpu_type();
	mxc_cpu_lp_set(WAIT_CLOCKED);
}
#ifdef CONFIG_CACHE_L2X0
int mxc_init_l2x0(void)
{
	unsigned int val;

	#define IOMUXC_GPR11_L2CACHE_AS_OCRAM 0x00000002

	val = readl(IOMUXC_GPR11);
	if (cpu_is_mx6sl() && (val & IOMUXC_GPR11_L2CACHE_AS_OCRAM)) {
		/* L2 cache configured as OCRAM, reset it */
		val &= ~IOMUXC_GPR11_L2CACHE_AS_OCRAM;
		writel(val, IOMUXC_GPR11);
	}

	writel(0x132, IO_ADDRESS(L2_BASE_ADDR + L2X0_TAG_LATENCY_CTRL));
	writel(0x132, IO_ADDRESS(L2_BASE_ADDR + L2X0_DATA_LATENCY_CTRL));

	val = readl(IO_ADDRESS(L2_BASE_ADDR + L2X0_PREFETCH_CTRL));

	/* Turn on the L2 I/D prefetch */
	val |= 0x30000000;

	/*
	 * The L2 cache controller(PL310) version on the i.MX6D/Q is r3p1-50rel0
	 * The L2 cache controller(PL310) version on the i.MX6DL/SOLO/SL is r3p2
	 * But according to ARM PL310 errata: 752271
	 * ID: 752271: Double linefill feature can cause data corruption
	 * Fault Status: Present in: r3p0, r3p1, r3p1-50rel0. Fixed in r3p2
	 * Workaround: The only workaround to this erratum is to disable the
	 * double linefill feature. This is the default behavior.
	 */
	if (!cpu_is_mx6q())
		val |= 0x40800000;
	writel(val, IO_ADDRESS(L2_BASE_ADDR + L2X0_PREFETCH_CTRL));

	val = readl(IO_ADDRESS(L2_BASE_ADDR + L2X0_POWER_CTRL));
	val |= L2X0_DYNAMIC_CLK_GATING_EN;
	val |= L2X0_STNDBY_MODE_EN;
	writel(val, IO_ADDRESS(L2_BASE_ADDR + L2X0_POWER_CTRL));

	l2x0_init(IO_ADDRESS(L2_BASE_ADDR), 0x0, ~0x00000000);
	return 0;
}


arch_initcall(mxc_init_l2x0);
#endif
