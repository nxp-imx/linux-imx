/*
 * Copyright (C) 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Juergen Beisert, kernel@pengutronix.de
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
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

#define BOARD_REV_1		0x000
#define BOARD_REV_2		0x100
#define BOARD_REV_3		0x200

#define IMX_IO_ADDRESS(addr, module)					\
	((void __force __iomem *)					\
	 (((unsigned long)((addr) - (module ## _BASE_ADDR)) < module ## _SIZE) ?\
	 (addr) - (module ## _BASE_ADDR) + (module ## _BASE_ADDR_VIRT) : 0))

#ifdef CONFIG_ARCH_MX5
#include <mach/mx5x.h>
#endif

#ifdef CONFIG_ARCH_MX3
#include <mach/mx3x.h>
#include <mach/mx31.h>
#include <mach/mx35.h>
#endif

#ifdef CONFIG_ARCH_MX37
#include <mach/mx37.h>
#endif

#ifdef CONFIG_ARCH_MX2
# include <mach/mx2x.h>
# ifdef CONFIG_MACH_MX21
#  include <mach/mx21.h>
# endif
# ifdef CONFIG_MACH_MX27
#  include <mach/mx27.h>
# endif
#endif

#ifdef CONFIG_ARCH_MX1
# include <mach/mx1.h>
#endif

#ifdef CONFIG_ARCH_MX25
# include <mach/mx25.h>
#endif

#ifdef CONFIG_ARCH_MXC91231
# include <mach/mxc91231.h>
#endif

#ifndef __ASSEMBLY__
extern unsigned int system_rev;
#define board_is_rev(rev)	(((system_rev & 0x0F00) == rev) ? 1 : 0)
#endif

#ifdef CONFIG_ARCH_MX5
#define board_is_mx53_arm2() (cpu_is_mx53() && board_is_rev(BOARD_REV_2))
#define board_is_mx53_evk_a()    (cpu_is_mx53() && board_is_rev(BOARD_REV_1))
#define board_is_mx53_evk_b()    (cpu_is_mx53() && board_is_rev(BOARD_REV_3))
#endif

#include <mach/mxc.h>

/*!
 * Register an interrupt handler for the SMN as well as the SCC.  In some
 * implementations, the SMN is not connected at all, and in others, it is
 * on the same interrupt line as the SCM. Comment this line out accordingly
 */
#define USE_SMN_INTERRUPT

/*!
 * This option is used to set or clear the RXDMUXSEL bit in control reg 3.
 * Certain platforms need this bit to be set in order to receive Irda data.
 */
#define MXC_UART_IR_RXDMUX      0x0004
/*!
 * This option is used to set or clear the RXDMUXSEL bit in control reg 3.
 * Certain platforms need this bit to be set in order to receive UART data.
 */
#define MXC_UART_RXDMUX         0x0004

#ifndef MXC_INT_FORCE
#define MXC_INT_FORCE	-1
#endif
#endif /* __ASM_ARCH_MXC_HARDWARE_H__ */
