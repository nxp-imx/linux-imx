/*
 * Low-level IRQ helper macros for Freescale MXS-based
 *
 * Copyright (C) 2009-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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

	.macro	disable_fiq
	.endm

	.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp
	ldr \base, =g_icoll_base
	ldr \base, [\base]
	ldr \irqnr, [\base, #0x70]
	cmp \irqnr, #0x7F
	bne 100f
	/* If the irq is 0x7F, check the raw bit to see
	whether it is a real GPIO0 irq */
	ldr \tmp, [\base, #0xD0]
	mov \tmp, \tmp, lsr #31
	and \tmp, \tmp, #0x1
	cmp \tmp, #0x1
	bne 200f
	/* Do a dummy compare to make sure Z = 0 */
	cmp \tmp, #0xFF
	b 100f
200:	movs \irqnr, #0
100:
	.endm

	.macro  get_irqnr_preamble, base, tmp
	.endm

	.macro  arch_ret_to_user, tmp1, tmp2
.endm
