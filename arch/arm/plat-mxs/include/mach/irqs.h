/*
 * Copyright (C) 1999 ARM Limited
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc.
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

#ifndef __ASM_ARCH_IRQS_H__
#define __ASM_ARCH_IRQS_H__

#include <mach/hardware.h>

#define NR_IRQS	(ARCH_NR_IRQS + ARCH_NR_GPIOS + MXS_EXTEND_IRQS)

#ifndef __ASSEMBLY__
struct irq_ic_info {
	unsigned int id_val;
	unsigned int id_mask;
	const char *name;
	unsigned int base;
};

#define __irq_ic_info_attr __attribute__((__section__(".irq_ic_info.array")))

extern struct irq_ic_info *current_irq_ic_info;

void mxs_set_irq_fiq(unsigned int irq, unsigned int type);
void mxs_enable_fiq_functionality(int enable);

#endif

#endif /* __ASM_ARCH_SYSTEM_H__ */
