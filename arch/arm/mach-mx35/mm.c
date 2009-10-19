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
 * @file mach-mx35/mm.c
 *
 * @brief This file creates static mapping between physical to virtual memory.
 *
 * @ingroup Memory_MX35
 */

/*!
 * This structure defines the MX35 memory map.
 */
static struct map_desc mx35_io_desc[] __initdata = {
	{
	 .virtual = IRAM_BASE_ADDR_VIRT,
	 .pfn = __phys_to_pfn(MX35_IRAM_BASE_ADDR),
	 .length = MX35_IRAM_SIZE,
	 .type = MT_DEVICE_NONSHARED},
	{
	 .virtual = X_MEMC_BASE_ADDR_VIRT,
	 .pfn = __phys_to_pfn(X_MEMC_BASE_ADDR),
	 .length = X_MEMC_SIZE,
	 .type = MT_DEVICE},
	{
	 .virtual = NFC_BASE_ADDR_VIRT,
	 .pfn = __phys_to_pfn(NFC_BASE_ADDR),
	 .length = NFC_SIZE,
	 .type = MT_DEVICE_NONSHARED},
	{
	 .virtual = AVIC_BASE_ADDR_VIRT,
	 .pfn = __phys_to_pfn(AVIC_BASE_ADDR),
	 .length = AVIC_SIZE,
	 .type = MT_DEVICE_NONSHARED},
	{
	 .virtual = AIPS1_BASE_ADDR_VIRT,
	 .pfn = __phys_to_pfn(AIPS1_BASE_ADDR),
	 .length = AIPS1_SIZE,
	 .type = MT_DEVICE_NONSHARED},
	{
	 .virtual = SPBA0_BASE_ADDR_VIRT,
	 .pfn = __phys_to_pfn(SPBA0_BASE_ADDR),
	 .length = SPBA0_SIZE,
	 .type = MT_DEVICE_NONSHARED},
	{
	 .virtual = AIPS2_BASE_ADDR_VIRT,
	 .pfn = __phys_to_pfn(AIPS2_BASE_ADDR),
	 .length = AIPS2_SIZE,
	 .type = MT_DEVICE_NONSHARED},
};

/*!
 * This function initializes the memory map. It is called during the
 * system startup to create static physical to virtual memory map for
 * the IO modules.
 */
void __init mx35_map_io(void)
{
	iotable_init(mx35_io_desc, ARRAY_SIZE(mx35_io_desc));
}
