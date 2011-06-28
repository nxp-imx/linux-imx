/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef FSL_ARCH_SMP_H
#define FSL_ARCH_SMP_H

#include <asm/hardware/gic.h>

/* Needed for secondary core boot */
extern void mx6_secondary_startup(void);
/*extern u32 mx6_modify_auxcoreboot0(u32 set_mask, u32 clear_mask);
extern void mx6_auxcoreboot_addr(u32 cpu_addr);
extern u32 mx6_read_auxcoreboot0(void);*/

/*
 * We use Soft IRQ1 as the IPI
 */
static inline void smp_cross_call(const struct cpumask *mask, int ipi)
{
	gic_raise_softirq(mask, ipi);
}
#endif
