/*
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License.  You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 *
 * Create static mapping between physical to virtual memory.
 */

#include <linux/mm.h>
#include <linux/init.h>

#include <asm/mach/map.h>
#include <mach/iomux-v3.h>

#include <mach/hardware.h>
#include <mach/common.h>
#include <mach/iomux-v3.h>

/*!
 * This structure defines the MX5x memory map.
 */
static struct map_desc mx5_io_desc[] __initdata = {
	{
	 .virtual = AIPS1_BASE_ADDR_VIRT,
	 .pfn = __phys_to_pfn(AIPS1_BASE_ADDR),
	 .length = AIPS1_SIZE,
	 .type = MT_DEVICE},
	{
	 .virtual = SPBA0_BASE_ADDR_VIRT,
	 .pfn = __phys_to_pfn(SPBA0_BASE_ADDR),
	 .length = SPBA0_SIZE,
	 .type = MT_DEVICE},
	{
	 .virtual = AIPS2_BASE_ADDR_VIRT,
	 .pfn = __phys_to_pfn(AIPS2_BASE_ADDR),
	 .length = AIPS2_SIZE,
	 .type = MT_DEVICE},
};

/*!
 * This function initializes the memory map. It is called during the
 * system startup to create static physical to virtual memory map for
 * the IO modules.
 */
void __init mx5_map_io(void)
{
	int i;

	mxc_iomux_v3_init(IO_ADDRESS(IOMUXC_BASE_ADDR));
	/* Fixup the mappings for MX53 */
	if (cpu_is_mx53() || cpu_is_mx50()) {
		for (i = 0; i < ARRAY_SIZE(mx5_io_desc); i++)
			mx5_io_desc[i].pfn -= __phys_to_pfn(0x20000000);
	}

	iotable_init(mx5_io_desc, ARRAY_SIZE(mx5_io_desc));
	mxc_arch_reset_init(IO_ADDRESS(WDOG1_BASE_ADDR));
}

