/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License.  You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/mm.h>
#include <linux/init.h>
#include <mach/hardware.h>
#include <asm/pgtable.h>
#include <asm/mach/map.h>

/*!
 * @file mach-mx51/mm.c
 *
 * @brief This file creates static mapping between physical to virtual memory.
 *
 * @ingroup Memory_MX51
 */

/*!
 * This structure defines the MX51 memory map.
 */
static struct map_desc mx51_io_desc[] __initdata = {
	{
	 .virtual = IRAM_BASE_ADDR_VIRT,
	 .pfn = __phys_to_pfn(IRAM_BASE_ADDR),
	 .length = IRAM_SIZE,
	 .type = MT_DEVICE},
	{
	 .virtual = DEBUG_BASE_ADDR_VIRT,
	 .pfn = __phys_to_pfn(DEBUG_BASE_ADDR),
	 .length = DEBUG_SIZE,
	 .type = MT_DEVICE},
	{
	 .virtual = TZIC_BASE_ADDR_VIRT,
	 .pfn = __phys_to_pfn(TZIC_BASE_ADDR),
	 .length = TZIC_SIZE,
	 .type = MT_DEVICE},
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
	{
	 .virtual = NFC_BASE_ADDR_AXI_VIRT,
	 .pfn = __phys_to_pfn(NFC_BASE_ADDR_AXI),
	 .length = NFC_AXI_SIZE,
	 .type = MT_DEVICE},
};

/*!
 * This function initializes the memory map. It is called during the
 * system startup to create static physical to virtual memory map for
 * the IO modules.
 */
void __init mx51_map_io(void)
{
	u32 tzic_addr;
	if (cpu_is_mx51_rev(CHIP_REV_2_0) < 0)
		tzic_addr = 0x8FFFC000;
	else
		tzic_addr = 0xE0003000;

	mx51_io_desc[2].pfn =  __phys_to_pfn(tzic_addr);
	iotable_init(mx51_io_desc, ARRAY_SIZE(mx51_io_desc));
}
