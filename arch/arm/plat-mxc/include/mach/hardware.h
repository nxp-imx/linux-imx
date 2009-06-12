/*
 *  Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_MXC_HARDWARE_H__
#define __ASM_ARCH_MXC_HARDWARE_H__

#include <asm/sizes.h>

/*
 * ---------------------------------------------------------------------------
 * Processor specific defines
 * ---------------------------------------------------------------------------
 */
#define CHIP_REV_1_0		0x10
#define CHIP_REV_1_1		0x11
#define CHIP_REV_1_2		0x12
#define CHIP_REV_1_3		0x13
#define CHIP_REV_2_0		0x20
#define CHIP_REV_2_1		0x21
#define CHIP_REV_2_2		0x22
#define CHIP_REV_2_3		0x23
#define CHIP_REV_3_0		0x30
#define CHIP_REV_3_1		0x31
#define CHIP_REV_3_2		0x32

#define BOARD_REV_2		0x100

#ifndef __ASSEMBLY__
extern unsigned int system_rev;
#endif
#define mxc_set_system_rev(part, rev) {			\
	system_rev = (part << 12) | rev;		\
}

#define mxc_cpu()		(system_rev >> 12)
#define mxc_is_cpu(part)	((mxc_cpu() == part) ? 1 : 0)
#define mxc_cpu_rev()		(system_rev & 0xFF)
#define mxc_cpu_rev_major()	((system_rev >> 4) & 0xF)
#define mxc_cpu_rev_minor()	(system_rev & 0xF)
#define mxc_cpu_is_rev(rev)	\
	((mxc_cpu_rev() == rev) ? 1 : ((mxc_cpu_rev() < rev) ? -1 : 2))
#define MXC_REV(type)				\
static inline int type## _rev (int rev)		\
{						\
	return (type() ? mxc_cpu_is_rev(rev) : 0);	\
}

#ifdef CONFIG_ARCH_MX3
# include <mach/mx31.h>
#define cpu_is_mx31()		(mxc_is_cpu(0x31))	/*system_rev got from Redboot */
#define cpu_is_mx32()		(mxc_is_cpu(0x32))	/*system_rev got from Redboot */
#else
#define cpu_is_mx31()		(0)
#define cpu_is_mx32()		(0)
#endif

#ifdef CONFIG_ARCH_MX35
#include <mach/mx35.h>
#define cpu_is_mx35()   (1)
#define board_is_mx35(rev)   ((system_rev & rev) ? 1 : 0)
#else
#define cpu_is_mx35()   (0)
#define board_is_mx35(rev) (0)
#endif

#ifdef CONFIG_ARCH_MX37
#include <mach/mx37.h>
#define cpu_is_mx37()   (1)
#define board_is_mx37(rev)   ((system_rev & rev) ? 1 : 0)
#else
#define cpu_is_mx37()   (0)
#define board_is_mx37(rev)   (0)
#endif

#ifdef CONFIG_ARCH_MX51
#include <mach/mx51.h>
#define cpu_is_mx51()   (1)
#define board_is_mx51(rev)   ((system_rev & rev) ? 1 : 0)
/* BB25:Bit8 is set to 1, BB20: Bit8 is set to 0 */
#define board_is_babbage_2_5()   ((system_rev & 0x1FF) >> 8)
#else
#define cpu_is_mx51()   (0)
#define board_is_mx51(rev)   (0)
#define board_is_babbage_2_5()   (0)
#endif

#ifdef CONFIG_ARCH_MX21
#include <mach/mx21.h>
#define cpu_is_mx21()		(1)
#else
#define cpu_is_mx21()		(0)
#endif

#ifdef CONFIG_ARCH_MX25
#include <mach/mx25.h>
#define cpu_is_mx25()		(1)
#else
#define cpu_is_mx25()		(0)
#endif

#ifdef CONFIG_ARCH_MX27
#include <mach/mx27.h>
#define cpu_is_mx27()		(1)
#else
#define cpu_is_mx27()		(0)
#endif

#ifndef __ASSEMBLY__
/*
 * Create inline functions to test for cpu revision
 * Function name is cpu_is_<cpu name>_rev(rev)
 *
 * Returns:
 *	 0 - not the cpu queried
 *	 1 - cpu and revision match
 *	 2 - cpu matches, but cpu revision is greater than queried rev
 *	-1 - cpu matches, but cpu revision is less than queried rev
 */
MXC_REV(cpu_is_mx21);
MXC_REV(cpu_is_mx25);
MXC_REV(cpu_is_mx27);
MXC_REV(cpu_is_mx31);
MXC_REV(cpu_is_mx32);
MXC_REV(cpu_is_mx35);
MXC_REV(cpu_is_mx37);
MXC_REV(cpu_is_mx51);
#endif
#include <mach/mxc.h>

#define MXC_MAX_GPIO_LINES      (GPIO_NUM_PIN * GPIO_PORT_NUM)

#define MXC_EXP_IO_BASE		(MXC_MAX_INT_LINES + MXC_MAX_GPIO_LINES)
#define MXC_MAX_EXP_IO_LINES	16

#ifdef CONFIG_MXC_PSEUDO_IRQS
#define MXC_PSEUDO_IO_BASE	(MXC_EXP_IO_BASE + MXC_MAX_EXP_IO_LINES)
#define MXC_MAX_PSEUDO_IO_LINES 16
#else
#define MXC_MAX_PSEUDO_IO_LINES 0
#endif

#ifndef MXC_INT_FORCE
#define MXC_INT_FORCE	-1
#endif
#define MXC_MAX_INTS            (MXC_MAX_INT_LINES + \
				MXC_MAX_GPIO_LINES + \
				MXC_MAX_EXP_IO_LINES + \
				MXC_MAX_PSEUDO_IO_LINES)
#endif /* __ASM_ARCH_MXC_HARDWARE_H__ */
