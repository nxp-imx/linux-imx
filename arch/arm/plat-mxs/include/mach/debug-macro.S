/*
 * Debugging macro include header
 *
 * Embedded Alley Solutions, Inc <source@embeddedalley.com>
 *
 * Copyright 2008-2010 Freescale Semiconductor, Inc.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <mach/hardware.h>

		.macro	addruart,rx,tmp
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1			@ MMU enabled?
	 	ldreq   \rx, =MXS_LL_UART_PADDR @ physical
		ldrne   \rx, =MXS_LL_UART_VADDR @ virtual
		.endm

		.macro	senduart,rd,rx
		strb	\rd, [\rx, #0]		@ data register at 0
		.endm

		.macro	waituart,rd,rx
1001:		ldr	\rd, [\rx, #0x18]	@ UARTFLG
		tst	\rd, #1 << 5		@ UARTFLGUTXFF - 1 when full
		bne	1001b
		.endm

		.macro	busyuart,rd,rx
1001:		ldr	\rd, [\rx, #0x18]	@ UARTFLG
		tst	\rd, #1 << 3		@ UARTFLGUBUSY - 1 when busy
		bne	1001b
		.endm
