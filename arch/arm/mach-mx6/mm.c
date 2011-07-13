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

/*!
 * This structure defines the MX6 memory map.
 */
static struct map_desc mx6_io_desc[] __initdata = {
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

/*!
 * This function initializes the memory map. It is called during the
 * system startup to create static physical to virtual memory map for
 * the IO modules.
 */
void __init mx6_map_io(void)
{
	iotable_init(mx6_io_desc, ARRAY_SIZE(mx6_io_desc));
	mxc_iomux_v3_init(IO_ADDRESS(MX6Q_IOMUXC_BASE_ADDR));
	mxc_arch_reset_init(IO_ADDRESS(WDOG1_BASE_ADDR));
}
#ifdef CONFIG_CACHE_L2X0
static int mxc_init_l2x0(void)
{

	l2x0_init(IO_ADDRESS(L2_BASE_ADDR), 0x0, ~0x00000000);
	return 0;
}


arch_initcall(mxc_init_l2x0);
#endif
