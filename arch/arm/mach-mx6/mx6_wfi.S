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

/*
 *  mx6_wait
 *
 *  Idle the processor (eg, wait for interrupt).
 *  Make sure DDR is in self-refresh.
 *  IRQs are already disabled.
 */
ENTRY(mx6_wait)

	stmfd   sp!, {r3,r4,r5,r6,r7}     @ Save registers

	wfi

	/*Wait for 170ns due to L2 cache errata (TKT065875) */
	/*System is more stable only if the wait is closer to ~380ns */
	/* Each IO read takes about 76ns. */

	ldr   r6, [r0]
	ldr   r6, [r0, #4]
	ldr   r6, [r0, #8]
	ldr   r6, [r0, #0xc]
	ldr   r6, [r0, #0x10]
	ldr   r6, [r0, #0x14]
	ldr   r6, [r0, #0x18]
	ldr   r6, [r0, #0x1c]
	ldr   r6, [r0, #0x20]
	ldr   r6, [r0, #0x24]

    /* Restore registers */
    ldmfd sp!, {r3,r4,r5,r6,r7}
    mov     pc, lr

    .type   mx6_do_wait, #object
ENTRY(mx6_do_wait)
    .word   mx6_wait
    .size    mx6_wait, . - mx6_wait
