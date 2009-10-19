/*
 *  Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_MXC_IRQS_H__
#define __ASM_ARCH_MXC_IRQS_H__

/*
 * So far all i.MX SoCs have 128 internal interrupts
 */
#define MXC_INTERNAL_IRQS	128

#define MXC_GPIO_IRQ_START	MXC_INTERNAL_IRQS

#if defined CONFIG_ARCH_MX1
#define MXC_GPIO_IRQS		(32 * 4)
#elif defined CONFIG_ARCH_MX2
#define MXC_GPIO_IRQS		(32 * 6)
#elif defined CONFIG_ARCH_MX25
#define MXC_GPIO_IRQS		(32 * 4)
#elif defined CONFIG_ARCH_MX3 || defined CONFIG_ARCH_MX35
#define MXC_GPIO_IRQS		(32 * 3)
#elif defined CONFIG_ARCH_MX37
#define MXC_GPIO_IRQS		(32 * 3)
#elif defined CONFIG_ARCH_MX51
#define MXC_GPIO_IRQS		(32 * 4)
#endif

/*
 * The next 16 interrupts are for board specific purposes.  Since
 * the kernel can only run on one machine at a time, we can re-use
 * these.  If you need more, increase MXC_BOARD_IRQS, but keep it
 * within sensible limits.
 */
#define MXC_BOARD_IRQ_START	(MXC_INTERNAL_IRQS + MXC_GPIO_IRQS)
#ifdef CONFIG_MXC_PSEUDO_IRQS
#define MXC_PSEUDO_IO_BASE	(MXC_BOARD_IRQ_START + 16)
#define MXC_MAX_PSEUDO_IO_LINES 16
#define MXC_BOARD_IRQS	32
#else
#define MXC_BOARD_IRQS	16
#define MXC_MAX_PSEUDO_IO_LINES 0
#endif

#define MXC_IPU_IRQ_START	(MXC_BOARD_IRQ_START + MXC_BOARD_IRQS)

#ifdef CONFIG_MX3_IPU_IRQS
#define MX3_IPU_IRQS CONFIG_MX3_IPU_IRQS
#else
#define MX3_IPU_IRQS 0
#endif

#define NR_IRQS			(MXC_IPU_IRQ_START + MX3_IPU_IRQS)

extern int imx_irq_set_priority(unsigned char irq, unsigned char prio);

/* all normal IRQs can be FIQs */
#define FIQ_START	0
/* switch betwean IRQ and FIQ */
extern int mxc_set_irq_fiq(unsigned int irq, unsigned int type);

#define MXC_IRQ_TO_EXPIO(irq)   ((irq) - MXC_BOARD_IRQ_START)

/*
 * This function is used to get the AVIC Lo and Hi interrupts
 * that are enabled as wake up sources to wake up the core from suspend
 */
void mxc_get_wake_irq(u32 * wake_src[]);

/* Define interrupt number for OProfile */
#if defined CONFIG_ARCH_MX51
#define MXC_INT_PMU		77
#endif

#endif /* __ASM_ARCH_MXC_IRQS_H__ */
