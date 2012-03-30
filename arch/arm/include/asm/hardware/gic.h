/*
 *  arch/arm/include/asm/hardware/gic.h
 *
 *  Copyright (C) 2002 ARM Limited, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __ASM_ARM_HARDWARE_GIC_H
#define __ASM_ARM_HARDWARE_GIC_H

#include <linux/compiler.h>

#define GIC_CPU_CTRL			0x00
#define GIC_CPU_PRIMASK			0x04
#define GIC_CPU_BINPOINT		0x08
#define GIC_CPU_INTACK			0x0c
#define GIC_CPU_EOI			0x10
#define GIC_CPU_RUNNINGPRI		0x14
#define GIC_CPU_HIGHPRI			0x18

#define GIC_DIST_CTRL			0x000
#define GIC_DIST_CTR			0x004
#define GIC_DIST_ENABLE_SET		0x100
#define GIC_DIST_ENABLE_CLEAR		0x180
#define GIC_DIST_PENDING_SET		0x200
#define GIC_DIST_PENDING_CLEAR		0x280
#define GIC_DIST_ACTIVE_BIT		0x300
#define GIC_DIST_PRI			0x400
#define GIC_DIST_TARGET			0x800
#define GIC_DIST_CONFIG			0xc00
#define GIC_DIST_SOFTINT		0xf00

#ifndef __ASSEMBLY__
extern void __iomem *gic_cpu_base_addr;
extern struct irq_chip gic_arch_extn;

struct gic_dist_state {
	u32	icddcr;			/* 0x000 */
					/* 0x004 RO */
					/* 0x008 RO */
					/* 0x00c ~ 0x07c Reserved */
	u32	icdisrn[8];		/* 0x080 ~ 0x09c */
	u32	icdisern[8];		/* 0x100 ~ 0x11c Reserved */
					/* 0x120 ~ 0x17c */
	u32	icdicern[8];		/* 0x180 ~ 0x19c */
	u32	icdisprn[32];		/* 0x200 ~ 0x27c */
	u32	icdicprn[8];		/* 0x280 ~ 0x29c */
	u32	icdabrn[8];		/* 0x300 ~ 0x31c */
					/* 0x320 ~ 0x3fc Reserved */
	u32	icdiprn[64];		/* 0x400 ~ 0x4fc */
					/* 0x500 ~ 0x7fc Reserved */
	u32	icdiptrn[64];		/* 0x800 ~ 0x8fc */
					/* 0x900 ~ 0xbfc Reserved */
	u32	icdicfrn[16];		/* 0xc00 ~ 0xc3c */
};

struct gic_cpu_state {
	u32	iccicr;			/* 0x000 */
	u32	iccpmr;			/* 0x004 */
	u32	iccbpr;			/* 0x008 */
};

void gic_init(unsigned int, unsigned int, void __iomem *, void __iomem *);
void gic_secondary_init(unsigned int);
void gic_cascade_irq(unsigned int gic_nr, unsigned int irq);
void gic_raise_softirq(const struct cpumask *mask, unsigned int irq);
void gic_enable_ppi(unsigned int);
void gic_disable_ppi(unsigned int);
void save_gic_cpu_state(unsigned int gic_nr, struct gic_cpu_state *gcs);
void restore_gic_cpu_state(unsigned int gic_nr, struct gic_cpu_state *gcs);
void save_gic_dist_state(unsigned int gic_nr, struct gic_dist_state *gds);
void restore_gic_dist_state(unsigned int gic_nr, struct gic_dist_state *gds);
#endif

#endif
