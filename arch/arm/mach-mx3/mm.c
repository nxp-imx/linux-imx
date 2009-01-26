/*
 *  Copyright (C) 1999,2000 Arm Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd
 *  Copyright (C) 2002 Shane Nay (shane@minirl.com)
 *  Copyright 2005-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 *    - add MX31 specific definitions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/mm.h>
#include <linux/init.h>
#include <asm/pgtable.h>
#include <asm/mach/map.h>
#include <mach/common.h>
#include <mach/hardware.h>

/*!
 * This table defines static virtual address mappings for I/O regions.
 * These are the mappings common across all MX3 boards.
 */
static struct map_desc mxc_io_desc[] __initdata = {
	{
	 .virtual = IRAM_BASE_ADDR_VIRT & 0xFFF00000,
	 .pfn = __phys_to_pfn(IRAM_BASE_ADDR & 0xFFF00000),
	 .length = SZ_1M,
	 .type = MT_DEVICE_NONSHARED},
	{
	 .virtual = X_MEMC_BASE_ADDR_VIRT,
	 .pfn = __phys_to_pfn(X_MEMC_BASE_ADDR),
	 .length = X_MEMC_SIZE,
	 .type = MT_DEVICE},
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
	{
	 .virtual = CS4_BASE_ADDR_VIRT,
	 .pfn = __phys_to_pfn(CS4_BASE_ADDR),
	 .length = CS4_SIZE,
	 .type = MT_DEVICE},
};

/*!
 * This function initializes the memory map. It is called during the
 * system startup to create static physical to virtual memory mappings
 * for the IO modules.
 */
void __init mxc_map_io(void)
{
	iotable_init(mxc_io_desc, ARRAY_SIZE(mxc_io_desc));
}
