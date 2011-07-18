/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/linkage.h>

#define ARM_CTRL_DCACHE     1 << 2
#define ARM_CTRL_ICACHE     1 << 12
#define ARM_AUXCR_L2EN      1 << 1

/*
 *  mx6q_suspend
 *
 *  Suspend the processor (eg, wait for interrupt).
 *  Set the DDR into Self Refresh
 *  IRQs are already disabled.
 */
ENTRY(mx6q_suspend)
	stmfd   sp!, {r0-r4}     @ Save registers
	dsb
	wfi
	ldmfd   sp!, {r0-r4}
	.type   mx6q_do_suspend, #object
ENTRY(mx6q_do_suspend)
	.word   mx6q_suspend
	.size   mx6q_suspend, . - mx6q_suspend
