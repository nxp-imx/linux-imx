/*
 * Copyright (C) 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_MXC_MEMORY_H__
#define __ASM_ARCH_MXC_MEMORY_H__

#include <asm/sizes.h>
#include <asm/page.h>

#define MX1_PHYS_OFFSET		UL(0x08000000)
#define MX21_PHYS_OFFSET	UL(0xc0000000)
#define MX25_PHYS_OFFSET	UL(0x80000000)
#define MX27_PHYS_OFFSET	UL(0xa0000000)
#define MX3x_PHYS_OFFSET	UL(0x80000000)
#define MX37_PHYS_OFFSET	UL(0x40000000)
#define MX50_PHYS_OFFSET	UL(0x70000000)
#define MX51_PHYS_OFFSET	UL(0x90000000)
#define MX53_PHYS_OFFSET	UL(0x70000000)
#define MXC91231_PHYS_OFFSET	UL(0x90000000)

#if !defined(CONFIG_RUNTIME_PHYS_OFFSET)
# if defined CONFIG_ARCH_MX1
#  define PHYS_OFFSET		MX1_PHYS_OFFSET
# elif defined CONFIG_MACH_MX21
#  define PHYS_OFFSET		MX21_PHYS_OFFSET
# elif defined CONFIG_ARCH_MX25
#  define PHYS_OFFSET		MX25_PHYS_OFFSET
# elif defined CONFIG_MACH_MX27
#  define PHYS_OFFSET		MX27_PHYS_OFFSET
# elif defined CONFIG_ARCH_MX3
#  define PHYS_OFFSET		MX3x_PHYS_OFFSET
# elif defined CONFIG_ARCH_MXC91231
#  define PHYS_OFFSET		MXC91231_PHYS_OFFSET
# elif defined CONFIG_ARCH_MX51
#  define PHYS_OFFSET		MX51_PHYS_OFFSET
# elif defined CONFIG_ARCH_MX53
#  define PHYS_OFFSET		MX53_PHYS_OFFSET
# elif defined CONFIG_ARCH_MX50
#  define PHYS_OFFSET		MX50_PHYS_OFFSET
# endif
#endif

#if defined(CONFIG_MX3_VIDEO)
/*
 * Increase size of DMA-consistent memory region.
 * This is required for mx3 camera driver to capture at least two QXGA frames.
 */
#define CONSISTENT_DMA_SIZE SZ_8M

#elif defined(CONFIG_MX1_VIDEO)
/*
 * Increase size of DMA-consistent memory region.
 * This is required for i.MX camera driver to capture at least four VGA frames.
 */
#define CONSISTENT_DMA_SIZE SZ_4M
#else

#ifdef CONFIG_ARCH_MX5
#define CONSISTENT_DMA_SIZE	(96 * SZ_1M)
#else
#define CONSISTENT_DMA_SIZE	(32 * SZ_1M)
#endif

#endif /* CONFIG_MX1_VIDEO */

#ifndef __ASSEMBLY__

#ifdef CONFIG_DMA_ZONE_SIZE
#define MXC_DMA_ZONE_SIZE	((CONFIG_DMA_ZONE_SIZE * SZ_1M) >> PAGE_SHIFT)
#else
#define MXC_DMA_ZONE_SIZE	((12 * SZ_1M) >> PAGE_SHIFT)
#endif

static inline void __arch_adjust_zones(int node, unsigned long *zone_size,
				       unsigned long *zhole_size)
{
	if (node != 0)
		return;
	/* Create separate zone to reserve memory for DMA */
	zone_size[1] = zone_size[0] - MXC_DMA_ZONE_SIZE;
	zone_size[0] = MXC_DMA_ZONE_SIZE;
	zhole_size[1] = zhole_size[0];
	zhole_size[0] = 0;
}

#define arch_adjust_zones(node, size, holes) \
	__arch_adjust_zones(node, size, holes)

#endif

#endif /* __ASM_ARCH_MXC_MEMORY_H__ */
