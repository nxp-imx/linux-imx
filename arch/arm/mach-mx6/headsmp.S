/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/init.h>

ENTRY(v7_invalidate_l1)
        mov     r0, #0
        mcr     p15, 2, r0, c0, c0, 0
        mrc     p15, 1, r0, c0, c0, 0

        ldr     r1, =0x7fff
        and     r2, r1, r0, lsr #13

        ldr     r1, =0x3ff

        and     r3, r1, r0, lsr #3  @ NumWays - 1
        add     r2, r2, #1          @ NumSets

        and     r0, r0, #0x7
        add     r0, r0, #4          @ SetShift

        clz     r1, r3              @ WayShift
        add     r4, r3, #1          @ NumWays
1:      sub     r2, r2, #1          @ NumSets--
        mov     r3, r4              @ Temp = NumWays
2:      subs    r3, r3, #1          @ Temp--
        mov     r5, r3, lsl r1
        mov     r6, r2, lsl r0
        orr     r5, r5, r6          @ Reg = (Temp<<WayShift)|(NumSets<<SetShift)
        mcr     p15, 0, r5, c7, c6, 2
        bgt     2b
        cmp     r2, #0
        bgt     1b
        dsb
        isb
        mov     pc, lr
ENDPROC(v7_invalidate_l1)

	__CPUINIT
ENTRY(mx6_secondary_startup)
	/*************************************************************
	The following code include workaround for smp wdog reset issue,
	when system reset, core1~core3 will check its GPR register, if
	it contain a valid pointer, they will continue to run, but
	since when a reset happen, DRAM can't be access before its
	controller initialized, so we must make sure the pointer value
	is in iRAM space, also, ARM recommend that there should not be
	any AXI access pending in the system if we want to reset
	individual CPU, it is better to put CPU in WFI state before
	reset, so now we implement the following flow to make sure
	this scenario:

         _______________________             _______________________
        |           CPU0        |     ----> |        CPU<n>         |
        |_______________________|    |      |_______________________|
                    |                |                 |
                    |                |                 |
                   \|/               |                \|/
         _______________________     |      _______________________
        |  GPR<n*2+2>=parameter |    |     | Rom jump to boot_entry|
        |_______________________|    |     |_______________________|
                    |                |                 |
                    |                |                 |
                   \|/               |                \|/
         _______________________     |      _______________________
        | GPR<n*2+1>=boot_entry |    |  -- |     GPR<n*2+1>=0      |
        |_______________________|    | |   |_______________________|
                    |                | |                |
                    |                | |                |    <----------------
                   \|/               | |               \|/                    |
         _______________________     | |          _____________     N     _________
        |        Reset CPU<n>   |____| |    ---> /CPU<n*2+2>=0?\ ------->|  WFI    |
        |_______________________|      |    |    \_____________/         |_________|
                    |                  |    |            |                    ^
-------------->     |                  |    |            | Y                  |
|                  \|/                 |    |           \|/                   |
|      N      _____________            |    |                                 |
-------------/GPR<n*2+1>=0?\ <---------     |                                 |
             \_____________/                |                                 |
                    |                       |                                 |
                 Y  |                       |                                 |
                   \|/                      |                                 |
         _______________________            |                                 |
        |       GPR<n*2+2>=0    | ----------                                  |
        |_______________________|                                             |
                    |                                                         |
                    |                                                         |
                   \|/                                                        |
         _______________________                                              |
        |    IPI software irq   |---------------------------------------------
        |_______________________|


	This function code is copied to iRAM 0x93f000, since
	there is function call below, such as v7_invalidate_l1 and
	secondary_startup, we have to use absolute address jump,
	to get the physical address of these functions, we need
	the offset of physical and virtual address, the
	offset is passed from GPR parameter, currently we store
	it at r8, future code change should avoid using r8.
*****************************************************************************/
	/* Invalidate L1 I-cache first */
	mov	r1,	#0x0
	mcr p15, 0, r1, c7, c5, 0 @ Invalidate I-Cache
	/* count the offset value and store it in r8 */
	ldr r3, =mx6_secondary_startup
	mrc	p15, 0, r0, c0, c0, 5
	and r0, r0, #15
	ldr r1, =0x020d8024
	add r1, r0, LSL#3
	ldr r0, [r1]
	sub r8, r3, r0

	msr     cpsr_fsxc, #0xd3

	/* must enable gic cpu, then cpu<n> can wakeup when cpu0
	send a software irq*/
	ldr r1, =0xa00100
	mov r0, #0x1
	str r0, [r1]
	mov r0, #0xf0
	str r0, [r1, #0x4]
	mov r0, #0x2
	str r0, [r1, #0x8]

	/* read cpu number in order to clear related GPRx */
	mrc	p15, 0, r0, c0, c0, 5
	and r0, r0, #15
	ldr r1, =0x020d8020
	add r1, r0, LSL#3
	/* clear GPR boot_entry register */
	mov r0, #0
	str r0, [r1]

	/* check whether GPR paremeter register is cleared */
	ldr r0, [r1, #0x4]
	cmp r0, #0x0
	beq 4f
3:
	wfi
	ldr r0, [r1, #0x4]
	cmp r0, #0x0
	bne 3b
4:
	/* invalidate l1-cache first */
	ldr r0, =v7_invalidate_l1
	sub r0, r0, r8
	mov lr, pc
	add lr, lr, #0x4
	mov pc, r0
	ldr r0, =secondary_startup
	sub r0, r0, r8

	/* jump to secondary_startup */
	mov pc, r0

ENDPROC(mx6_secondary_startup)
