/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

#include <asm/page.h>
#include <asm/sizes.h>

/*
 * Physical DRAM offset.
 */
#define PHYS_OFFSET	UL(0x40000000)

#ifndef __ASSEMBLY__

#ifdef CONFIG_DMA_ZONE_SIZE
#define MXS_DMA_ZONE_SIZE	((CONFIG_DMA_ZONE_SIZE * SZ_1M) >> PAGE_SHIFT)
#else
#define MXS_DMA_ZONE_SIZE	((12 * SZ_1M) >> PAGE_SHIFT)
#endif

static inline void __arch_adjust_zones(int node, unsigned long *zone_size,
				       unsigned long *zhole_size)
{
	if (node != 0)
		return;
	/* Create separate zone to reserve memory for DMA */
	zone_size[1] = zone_size[0] - MXS_DMA_ZONE_SIZE;
	zone_size[0] = MXS_DMA_ZONE_SIZE;
	zhole_size[1] = zhole_size[0];
	zhole_size[0] = 0;
}

#define arch_adjust_zones(node, size, holes) \
	__arch_adjust_zones(node, size, holes)

#endif

#define ISA_DMA_THRESHOLD	(0x0003ffffULL)

#define CONSISTENT_DMA_SIZE	SZ_32M

#endif
