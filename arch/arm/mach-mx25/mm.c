/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
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
 * @file mach-mx25/mm.c
 *
 * @brief This file creates static mapping between physical to virtual memory.
 *
 * @ingroup Memory_MX25
 */

/*!
 * This structure defines the MX25 memory map.
 */
static struct map_desc mx25_io_desc[] __initdata = {
	{
	 .virtual = IRAM_BASE_ADDR_VIRT,
	 .pfn = __phys_to_pfn(IRAM_BASE_ADDR),
	 .length = IRAM_SIZE,
	 .type = MT_DEVICE},
	{
	 .virtual = X_MEMC_BASE_ADDR_VIRT,
	 .pfn = __phys_to_pfn(X_MEMC_BASE_ADDR),
	 .length = X_MEMC_SIZE,
	 .type = MT_DEVICE},
	{
	 .virtual = NFC_BASE_ADDR_VIRT,
	 .pfn = __phys_to_pfn(NFC_BASE_ADDR),
	 .length = NFC_SIZE,
	 .type = MT_DEVICE},
	{
	 .virtual = ROMP_BASE_ADDR_VIRT,
	 .pfn = __phys_to_pfn(ROMP_BASE_ADDR),
	 .length = ROMP_SIZE,
	 .type = MT_DEVICE},
	{
	 .virtual = ASIC_BASE_ADDR_VIRT,
	 .pfn = __phys_to_pfn(ASIC_BASE_ADDR),
	 .length = ASIC_SIZE,
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
};

/*!
 * This function initializes the memory map. It is called during the
 * system startup to create static physical to virtual memory map for
 * the IO modules.
 */
void __init mx25_map_io(void)
{
	iotable_init(mx25_io_desc, ARRAY_SIZE(mx25_io_desc));
}
