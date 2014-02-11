/*
 * Copyright (C) 2010-2014 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <asm/hardware/cache-l2x0.h>
#include "hardware.h"

#define MX6Q_SRC_GPR1	0x20
#define MX6Q_SRC_GPR2	0x24
#define MX6Q_MMDC_MAPSR	0x404
#define MX6Q_MMDC_MPDGCTRL0	0x83c
#define MX6Q_GPC_IMR1	0x08
#define MX6Q_GPC_IMR2	0x0c
#define MX6Q_GPC_IMR3	0x10
#define MX6Q_GPC_IMR4	0x14
#define MX6Q_CCM_CCR	0x0
#define MX6Q_ANATOP_CORE	0x140

	.align 3

	.macro	imx6sx_ddr_io_save

	.endm

	.macro	imx6sx_ddr_io_restore

	.endm

	.macro	imx6sx_ddr_io_set_lpm

	.endm

	.macro	imx6sl_ddr_io_save

	ldr	r4, [r8, #0x30c] /* DRAM_DQM0 */
	ldr	r5, [r8, #0x310] /* DRAM_DQM1 */
	ldr	r6, [r8, #0x314] /* DRAM_DQM2 */
	ldr	r7, [r8, #0x318] /* DRAM_DQM3 */
	stmfd	r10!, {r4-r7}

	ldr	r4, [r8, #0x5c4] /* GPR_B0DS */
	ldr	r5, [r8, #0x5cc] /* GPR_B1DS */
	ldr	r6, [r8, #0x5d4] /* GPR_B2DS */
	ldr	r7, [r8, #0x5d8] /* GPR_B3DS */
	stmfd	r10!, {r4-r7}

	ldr	r4, [r8, #0x300] /* DRAM_CAS */
	ldr	r5, [r8, #0x31c] /* DRAM_RAS */
	ldr	r6, [r8, #0x338] /* DRAM_SDCLK_0 */
	ldr	r7, [r8, #0x5ac] /* GPR_ADDS*/
	stmfd	r10!, {r4-r7}

	ldr	r4, [r8, #0x5b0] /* DDRMODE_CTL */
	ldr	r5, [r8, #0x5c0] /* DDRMODE */
	ldr	r6, [r8, #0x33c] /* DRAM_SODT0*/
	ldr	r7, [r8, #0x340] /* DRAM_SODT1*/
	stmfd	r10!, {r4-r7}

	ldr	r4, [r8, #0x330] /* DRAM_SDCKE0 */
	ldr	r5, [r8, #0x334] /* DRAM_SDCKE1 */
	ldr	r6, [r8, #0x320] /* DRAM_RESET */
	stmfd	r10!, {r4-r6}

	.endm

	.macro	imx6sl_ddr_io_restore

	ldmea	r10!, {r4-r7}
	str	r4, [r8, #0x30c] /* DRAM_DQM0 */
	str	r5, [r8, #0x310] /* DRAM_DQM1 */
	str	r6, [r8, #0x314] /* DRAM_DQM2 */
	str	r7, [r8, #0x318] /* DRAM_DQM3 */

	ldmea	r10!, {r4-r7}
	str	r4, [r8, #0x5c4] /* GPR_B0DS */
	str	r5, [r8, #0x5cc] /* GPR_B1DS */
	str	r6, [r8, #0x5d4] /* GPR_B2DS */
	str	r7, [r8, #0x5d8] /* GPR_B3DS */

	ldmea	r10!, {r4-r7}
	str	r4, [r8, #0x300] /* DRAM_CAS */
	str	r5, [r8, #0x31c] /* DRAM_RAS */
	str	r6, [r8, #0x338] /* DRAM_SDCLK_0 */
	str	r7, [r8, #0x5ac] /* GPR_ADDS*/

	ldmea	r10!, {r4-r7}
	str	r4, [r8, #0x5b0] /* DDRMODE_CTL */
	str	r5, [r8, #0x5c0] /* DDRMODE */
	str	r6, [r8, #0x33c] /* DRAM_SODT0*/
	str	r7, [r8, #0x340] /* DRAM_SODT1*/

	ldmea	r10!, {r4-r6}
	str	r4, [r8, #0x330] /* DRAM_SDCKE0 */
	str	r5, [r8, #0x334] /* DRAM_SDCKE1 */
	str	r6, [r8, #0x320] /* DRAM_RESET */

	.endm

	.macro	imx6sl_ddr_io_set_lpm

	mov	r10, #0
	str	r10, [r8, #0x30c] /* DRAM_DQM0 */
	str	r10, [r8, #0x310] /* DRAM_DQM1 */
	str	r10, [r8, #0x314] /* DRAM_DQM2 */
	str	r10, [r8, #0x318] /* DRAM_DQM3 */

	str	r10, [r8, #0x5c4] /* GPR_B0DS */
	str	r10, [r8, #0x5cc] /* GPR_B1DS */
	str	r10, [r8, #0x5d4] /* GPR_B2DS */
	str	r10, [r8, #0x5d8] /* GPR_B3DS */

	str	r10, [r8, #0x300] /* DRAM_CAS */
	str	r10, [r8, #0x31c] /* DRAM_RAS */
	str	r10, [r8, #0x338] /* DRAM_SDCLK_0 */
	str	r10, [r8, #0x5ac] /* GPR_ADDS*/

	str	r10, [r8, #0x5b0] /* DDRMODE_CTL */
	str	r10, [r8, #0x5c0] /* DDRMODE */
	str	r10, [r8, #0x33c] /* DRAM_SODT0*/
	str	r10, [r8, #0x340] /* DRAM_SODT1*/

	mov	r10, #0x80000
	str	r10, [r8, #0x320] /* DRAM_RESET */
	mov	r10, #0x1000
	str	r10, [r8, #0x330] /* DRAM_SDCKE0 */
	str	r10, [r8, #0x334] /* DRAM_SDCKE1 */

	.endm

	.macro	imx6dl_ddr_io_save

	ldr	r4, [r8, #0x470] /* DRAM_DQM0 */
	ldr	r5, [r8, #0x474] /* DRAM_DQM1 */
	ldr	r6, [r8, #0x478] /* DRAM_DQM2 */
	ldr	r7, [r8, #0x47c] /* DRAM_DQM3 */
	stmfd	r10!, {r4-r7}

	ldr	r4, [r8, #0x480] /* DRAM_DQM4 */
	ldr	r5, [r8, #0x484] /* DRAM_DQM5 */
	ldr	r6, [r8, #0x488] /* DRAM_DQM6 */
	ldr	r7, [r8, #0x48c] /* DRAM_DQM7 */
	stmfd	r10!, {r4-r7}

	ldr	r4, [r8, #0x464] /* DRAM_CAS */
	ldr	r5, [r8, #0x490] /* DRAM_RAS */
	ldr	r6, [r8, #0x4ac] /* DRAM_SDCLK_0 */
	ldr	r7, [r8, #0x4b0] /* DRAM_SDCLK_1 */
	stmfd	r10!, {r4-r7}

	ldr	r5, [r8, #0x750] /* DDRMODE_CTL */
	ldr	r6, [r8, #0x760] /* DDRMODE */
	stmfd	r10!, {r5-r6}

	ldr	r4, [r8, #0x4bc] /* DRAM_SDQS0 */
	ldr	r5, [r8, #0x4c0] /* DRAM_SDQS1 */
	ldr	r6, [r8, #0x4c4] /* DRAM_SDQS2 */
	ldr	r7, [r8, #0x4c8] /* DRAM_SDQS3 */
	stmfd	r10!, {r4-r7}

	ldr	r4, [r8, #0x4cc] /* DRAM_SDQS4 */
	ldr	r5, [r8, #0x4d0] /* DRAM_SDQS5 */
	ldr	r6, [r8, #0x4d4] /* DRAM_SDQS6 */
	ldr	r7, [r8, #0x4d8] /* DRAM_SDQS7 */
	stmfd	r10!, {r4-r7}

	ldr	r4, [r8, #0x764] /* GPR_B0DS */
	ldr	r5, [r8, #0x770] /* GPR_B1DS */
	ldr	r6, [r8, #0x778] /* GPR_B2DS */
	ldr	r7, [r8, #0x77c] /* GPR_B3DS */
	stmfd	r10!, {r4-r7}

	ldr	r4, [r8, #0x780] /* GPR_B4DS */
	ldr	r5, [r8, #0x784] /* GPR_B5DS */
	ldr	r6, [r8, #0x78c] /* GPR_B6DS */
	ldr	r7, [r8, #0x748] /* GPR_B7DS */
	stmfd	r10!, {r4-r7}

	ldr	r5, [r8, #0x74c] /* GPR_ADDS*/
	ldr	r6, [r8, #0x4b4] /* DRAM_SODT0*/
	ldr	r7, [r8, #0x4b8] /* DRAM_SODT1*/
	stmfd	r10!, {r5-r7}

	.endm

	.macro	imx6dl_ddr_io_restore

	ldmea	r10!, {r4-r7}
	str	r4, [r8, #0x470] /* DRAM_DQM0 */
	str	r5, [r8, #0x474] /* DRAM_DQM1 */
	str	r6, [r8, #0x478] /* DRAM_DQM2 */
	str	r7, [r8, #0x47c] /* DRAM_DQM3 */

	ldmea	r10!, {r4-r7}
	str	r4, [r8, #0x480] /* DRAM_DQM4 */
	str	r5, [r8, #0x484] /* DRAM_DQM5 */
	str	r6, [r8, #0x488] /* DRAM_DQM6 */
	str	r7, [r8, #0x48c] /* DRAM_DQM7 */

	ldmea	r10!, {r4-r7}
	str	r4, [r8, #0x464] /* DRAM_CAS */
	str	r5, [r8, #0x490] /* DRAM_RAS */
	str	r6, [r8, #0x4ac] /* DRAM_SDCLK_0 */
	str	r7, [r8, #0x4b0] /* DRAM_SDCLK_1 */

	ldmea	r10!, {r5-r6}
	str	r5, [r8, #0x750] /* DDRMODE_CTL */
	str	r6, [r8, #0x760] /* DDRMODE */

	ldmea	r10!, {r4-r7}
	str	r4, [r8, #0x4bc] /* DRAM_SDQS0 */
	str	r5, [r8, #0x4c0] /* DRAM_SDQS1 */
	str	r6, [r8, #0x4c4] /* DRAM_SDQS2 */
	str	r7, [r8, #0x4c8] /* DRAM_SDQS3 */

	ldmea	r10!, {r4-r7}
	str	r4, [r8, #0x4cc] /* DRAM_SDQS4 */
	str	r5, [r8, #0x4d0] /* DRAM_SDQS5 */
	str	r6, [r8, #0x4d4] /* DRAM_SDQS6 */
	str	r7, [r8, #0x4d8] /* DRAM_SDQS7 */

	ldmea	r10!, {r4-r7}
	str	r4, [r8, #0x764] /* GPR_B0DS */
	str	r5, [r8, #0x770] /* GPR_B1DS */
	str	r6, [r8, #0x778] /* GPR_B2DS */
	str	r7, [r8, #0x77c] /* GPR_B3DS */

	ldmea	r10!, {r4-r7}
	str	r4, [r8, #0x780] /* GPR_B4DS */
	str	r5, [r8, #0x784] /* GPR_B5DS */
	str	r6, [r8, #0x78c] /* GPR_B6DS */
	str	r7, [r8, #0x748] /* GPR_B7DS */

	ldmea	r10!, {r5-r7}
	str	r5, [r8, #0x74c] /* GPR_ADDS*/
	str	r6, [r8, #0x4b4] /* DRAM_SODT0*/
	str	r7, [r8, #0x4b8] /* DRAM_SODT1*/

	.endm

	.macro	imx6dl_ddr_io_set_lpm

	mov	r10, #0
	str	r10, [r8, #0x470] /* DRAM_DQM0 */
	str	r10, [r8, #0x474] /* DRAM_DQM1 */
	str	r10, [r8, #0x478] /* DRAM_DQM2 */
	str	r10, [r8, #0x47c] /* DRAM_DQM3 */

	str	r10, [r8, #0x480] /* DRAM_DQM4 */
	str	r10, [r8, #0x484] /* DRAM_DQM5 */
	str	r10, [r8, #0x488] /* DRAM_DQM6 */
	str	r10, [r8, #0x48c] /* DRAM_DQM7 */

	str	r10, [r8, #0x464] /* DRAM_CAS */
	str	r10, [r8, #0x490] /* DRAM_RAS */
	str	r10, [r8, #0x4ac] /* DRAM_SDCLK_0 */
	str	r10, [r8, #0x4b0] /* DRAM_SDCLK_1 */

	str	r10, [r8, #0x750] /* DDRMODE_CTL */
	str	r10, [r8, #0x760] /* DDRMODE */

	str	r10, [r8, #0x4bc] /* DRAM_SDQS0 */
	str	r10, [r8, #0x4c0] /* DRAM_SDQS1 */
	str	r10, [r8, #0x4c4] /* DRAM_SDQS2 */
	str	r10, [r8, #0x4c8] /* DRAM_SDQS3 */

	str	r10, [r8, #0x4cc] /* DRAM_SDQS4 */
	str	r10, [r8, #0x4d0] /* DRAM_SDQS5 */
	str	r10, [r8, #0x4d4] /* DRAM_SDQS6 */
	str	r10, [r8, #0x4d8] /* DRAM_SDQS7 */

	str	r10, [r8, #0x764] /* GPR_B0DS */
	str	r10, [r8, #0x770] /* GPR_B1DS */
	str	r10, [r8, #0x778] /* GPR_B2DS */
	str	r10, [r8, #0x77c] /* GPR_B3DS */

	str	r10, [r8, #0x780] /* GPR_B4DS */
	str	r10, [r8, #0x784] /* GPR_B5DS */
	str	r10, [r8, #0x78c] /* GPR_B6DS */
	str	r10, [r8, #0x748] /* GPR_B7DS */

	str	r10, [r8, #0x74c] /* GPR_ADDS*/
	str	r10, [r8, #0x4b4] /* DRAM_SODT0*/
	str	r10, [r8, #0x4b8] /* DRAM_SODT1*/

	.endm

	.macro	imx6dq_ddr_io_save

	ldr	r4, [r8, #0x5ac] /* DRAM_DQM0 */
	ldr	r5, [r8, #0x5b4] /* DRAM_DQM1 */
	ldr	r6, [r8, #0x528] /* DRAM_DQM2 */
	ldr	r7, [r8, #0x520] /* DRAM_DQM3 */
	stmfd	r10!, {r4-r7}

	ldr	r4, [r8, #0x514] /* DRAM_DQM4 */
	ldr	r5, [r8, #0x510] /* DRAM_DQM5 */
	ldr	r6, [r8, #0x5bc] /* DRAM_DQM6 */
	ldr	r7, [r8, #0x5c4] /* DRAM_DQM7 */
	stmfd	r10!, {r4-r7}

	ldr	r4, [r8, #0x56c] /* DRAM_CAS */
	ldr	r5, [r8, #0x578] /* DRAM_RAS */
	ldr	r6, [r8, #0x588] /* DRAM_SDCLK_0 */
	ldr	r7, [r8, #0x594] /* DRAM_SDCLK_1 */
	stmfd	r10!, {r4-r7}

	ldr	r5, [r8, #0x750] /* DDRMODE_CTL */
	ldr	r6, [r8, #0x774] /* DDRMODE */
	stmfd	r10!, {r5-r6}

	ldr	r4, [r8, #0x5a8] /* DRAM_SDQS0 */
	ldr	r5, [r8, #0x5b0] /* DRAM_SDQS1 */
	ldr	r6, [r8, #0x524] /* DRAM_SDQS2 */
	ldr	r7, [r8, #0x51c] /* DRAM_SDQS3 */
	stmfd	r10!, {r4-r7}

	ldr	r4, [r8, #0x518] /* DRAM_SDQS4 */
	ldr	r5, [r8, #0x50c] /* DRAM_SDQS5 */
	ldr	r6, [r8, #0x5b8] /* DRAM_SDQS6 */
	ldr	r7, [r8, #0x5c0] /* DRAM_SDQS7 */
	stmfd	r10!, {r4-r7}

	ldr	r4, [r8, #0x784] /* GPR_B0DS */
	ldr	r5, [r8, #0x788] /* GPR_B1DS */
	ldr	r6, [r8, #0x794] /* GPR_B2DS */
	ldr	r7, [r8, #0x79c] /* GPR_B3DS */
	stmfd	r10!, {r4-r7}

	ldr	r4, [r8, #0x7a0] /* GPR_B4DS */
	ldr	r5, [r8, #0x7a4] /* GPR_B5DS */
	ldr	r6, [r8, #0x7a8] /* GPR_B6DS */
	ldr	r7, [r8, #0x748] /* GPR_B7DS */
	stmfd	r10!, {r4-r7}

	ldr	r5, [r8, #0x74c] /* GPR_ADDS*/
	ldr	r6, [r8, #0x59c] /* DRAM_SODT0*/
	ldr	r7, [r8, #0x5a0] /* DRAM_SODT1*/
	stmfd	r10!, {r5-r7}

	.endm

	.macro	imx6dq_ddr_io_restore

	ldmea	r10!, {r4-r7}
	str	r4, [r8, #0x5ac] /* DRAM_DQM0 */
	str	r5, [r8, #0x5b4] /* DRAM_DQM1 */
	str	r6, [r8, #0x528] /* DRAM_DQM2 */
	str	r7, [r8, #0x520] /* DRAM_DQM3 */

	ldmea	r10!, {r4-r7}
	str	r4, [r8, #0x514] /* DRAM_DQM4 */
	str	r5, [r8, #0x510] /* DRAM_DQM5 */
	str	r6, [r8, #0x5bc] /* DRAM_DQM6 */
	str	r7, [r8, #0x5c4] /* DRAM_DQM7 */

	ldmea	r10!, {r4-r7}
	str	r4, [r8, #0x56c] /* DRAM_CAS */
	str	r5, [r8, #0x578] /* DRAM_RAS */
	str	r6, [r8, #0x588] /* DRAM_SDCLK_0 */
	str	r7, [r8, #0x594] /* DRAM_SDCLK_1 */

	ldmea	r10!, {r5-r6}
	str	r5, [r8, #0x750] /* DDRMODE_CTL */
	str	r6, [r8, #0x774] /* DDRMODE */

	ldmea	r10!, {r4-r7}
	str	r4, [r8, #0x5a8] /* DRAM_SDQS0 */
	str	r5, [r8, #0x5b0] /* DRAM_SDQS1 */
	str	r6, [r8, #0x524] /* DRAM_SDQS2 */
	str	r7, [r8, #0x51c] /* DRAM_SDQS3 */

	ldmea	r10!, {r4-r7}
	str	r4, [r8, #0x518] /* DRAM_SDQS4 */
	str	r5, [r8, #0x50c] /* DRAM_SDQS5 */
	str	r6, [r8, #0x5b8] /* DRAM_SDQS6 */
	str	r7, [r8, #0x5c0] /* DRAM_SDQS7 */

	ldmea	r10!, {r4-r7}
	str	r4, [r8, #0x784] /* GPR_B0DS */
	str	r5, [r8, #0x788] /* GPR_B1DS */
	str	r6, [r8, #0x794] /* GPR_B2DS */
	str	r7, [r8, #0x79c] /* GPR_B3DS */

	ldmea	r10!, {r4-r7}
	str	r4, [r8, #0x7a0] /* GPR_B4DS */
	str	r5, [r8, #0x7a4] /* GPR_B5DS */
	str	r6, [r8, #0x7a8] /* GPR_B6DS */
	str	r7, [r8, #0x748] /* GPR_B7DS */

	ldmea	r10!, {r5-r7}
	str	r5, [r8, #0x74c] /* GPR_ADDS*/
	str	r6, [r8, #0x59c] /* DRAM_SODT0*/
	str	r7, [r8, #0x5a0] /* DRAM_SODT1*/

	.endm

	.macro	imx6dq_ddr_io_set_lpm

	mov	r10, #0
	str	r10, [r8, #0x5ac] /* DRAM_DQM0 */
	str	r10, [r8, #0x5b4] /* DRAM_DQM1 */
	str	r10, [r8, #0x528] /* DRAM_DQM2 */
	str	r10, [r8, #0x520] /* DRAM_DQM3 */

	str	r10, [r8, #0x514] /* DRAM_DQM4 */
	str	r10, [r8, #0x510] /* DRAM_DQM5 */
	str	r10, [r8, #0x5bc] /* DRAM_DQM6 */
	str	r10, [r8, #0x5c4] /* DRAM_DQM7 */

	str	r10, [r8, #0x56c] /* DRAM_CAS */
	str	r10, [r8, #0x578] /* DRAM_RAS */
	str	r10, [r8, #0x588] /* DRAM_SDCLK_0 */
	str	r10, [r8, #0x594] /* DRAM_SDCLK_1 */

	str	r10, [r8, #0x750] /* DDRMODE_CTL */
	str	r10, [r8, #0x774] /* DDRMODE */

	str	r10, [r8, #0x5a8] /* DRAM_SDQS0 */
	str	r10, [r8, #0x5b0] /* DRAM_SDQS1 */
	str	r10, [r8, #0x524] /* DRAM_SDQS2 */
	str	r10, [r8, #0x51c] /* DRAM_SDQS3 */

	str	r10, [r8, #0x518] /* DRAM_SDQS4 */
	str	r10, [r8, #0x50c] /* DRAM_SDQS5 */
	str	r10, [r8, #0x5b8] /* DRAM_SDQS6 */
	str	r10, [r8, #0x5c0] /* DRAM_SDQS7 */

	str	r10, [r8, #0x784] /* GPR_B0DS */
	str	r10, [r8, #0x788] /* GPR_B1DS */
	str	r10, [r8, #0x794] /* GPR_B2DS */
	str	r10, [r8, #0x79c] /* GPR_B3DS */

	str	r10, [r8, #0x7a0] /* GPR_B4DS */
	str	r10, [r8, #0x7a4] /* GPR_B5DS */
	str	r10, [r8, #0x7a8] /* GPR_B6DS */
	str	r10, [r8, #0x748] /* GPR_B7DS */

	str	r10, [r8, #0x74c] /* GPR_ADDS*/
	str	r10, [r8, #0x59c] /* DRAM_SODT0*/
	str	r10, [r8, #0x5a0] /* DRAM_SODT1*/

	.endm

	.macro  sync_l2_cache

	/* sync L2 cache to drain L2's buffers to DRAM. */
#ifdef CONFIG_CACHE_L2X0
	ldr	r8, =IMX_IO_P2V(MX6Q_L2_BASE_ADDR)
	mov	r5, #0x0
	str	r5, [r8, #L2X0_CACHE_SYNC]
1:
	ldr	r5, [r8, #L2X0_CACHE_SYNC]
	ands	r5, r5, #0x1
	bne	1b
#endif
	.endm

ENTRY(imx6_suspend)

	/*
	 * counting the resume address in iram
	 * to set it in SRC register.
	 */
	ldr	r4, =imx6_suspend
	ldr	r5, =resume
	sub	r5, r5, r4
	add	r9, r1, r5

	/*
	 * make sure TLB contain the addr we want,
	 * as we will access after DDR IO floated.
	 */

	ldr	r8, =IMX_IO_P2V(MX6Q_ANATOP_BASE_ADDR)
	ldr	r7, [r8, #MX6Q_ANATOP_CORE]
	ldr	r8, =IMX_IO_P2V(MX6Q_CCM_BASE_ADDR)
	ldr	r7, [r8, #0x0]
	ldr	r8, =IMX_IO_P2V(MX6Q_GPC_BASE_ADDR)
	ldr	r7, [r8, #0x0]

	/* use r8 to store the IO address */
	ldr	r8, =IMX_IO_P2V(MX6Q_SRC_BASE_ADDR)

	/*
	 * read previous resume address from SRC
	 * register, which is v7_cpu_resume, this
	 * is for the jump when we finish DDR IO
	 * restore.
	 */
	ldr	r5, [r8, #MX6Q_SRC_GPR1]
	add	r10, r0, #MX6_SUSPEND_IRAM_SIZE
	stmfd	r10!, {r5}

	/* save cpu type */
	stmfd	r10!, {r2}

	str	r9, [r8, #MX6Q_SRC_GPR1]
	add	r3, r1, #MX6_SUSPEND_IRAM_SIZE
	str	r3, [r8, #MX6Q_SRC_GPR2]

	ldr	r8, =IMX_IO_P2V(MX6Q_IOMUXC_BASE_ADDR)

	cmp	r2, #MXC_CPU_IMX6Q
	bne	dl_io_dsm_save
	imx6dq_ddr_io_save
	b	ddr_io_save_dsm_done
dl_io_dsm_save:
	cmp     r2, #MXC_CPU_IMX6DL
	bne	sl_io_save
	imx6dl_ddr_io_save
	b	ddr_io_save_dsm_done
sl_io_save:
	cmp	r2, #MXC_CPU_IMX6SL
	bne	sx_io_save
	imx6sl_ddr_io_save
	b	ddr_io_save_dsm_done
sx_io_save:
	imx6sx_ddr_io_save
ddr_io_save_dsm_done:

	/* need to sync L2 cache before DSM. */
	sync_l2_cache

	ldr	r8, =IMX_IO_P2V(MX6Q_MMDC_P0_BASE_ADDR)
	/*
	 * put DDR explicitly into self-refresh and
	 * disable Automatic power savings.
	 */
	ldr	r7, [r8, #MX6Q_MMDC_MAPSR]
	orr	r7, r7, #0x01
	str	r7, [r8, #MX6Q_MMDC_MAPSR]

	/* make the DDR explicitly enter self-refresh. */
	ldr	r7, [r8, #MX6Q_MMDC_MAPSR]
	orr	r7, r7, #(1 << 21)
	str	r7, [r8, #MX6Q_MMDC_MAPSR]

poll_dvfs_set_1:
	ldr	r7, [r8, #0x404]
	ands	r7, r7, #(1 << 25)
	beq	poll_dvfs_set_1

	ldr	r8, =IMX_IO_P2V(MX6Q_IOMUXC_BASE_ADDR)

	cmp	r2, #MXC_CPU_IMX6Q
	bne	dl_io_dsm_set_lpm
	imx6dq_ddr_io_set_lpm
	b	ddr_io_set_lpm_dsm_done
dl_io_dsm_set_lpm:
	cmp     r2, #MXC_CPU_IMX6DL
	bne	sl_io_dsm_set_lpm
	imx6dl_ddr_io_set_lpm
	b	ddr_io_set_lpm_dsm_done
sl_io_dsm_set_lpm:
	cmp	r2, #MXC_CPU_IMX6SL
	bne	sx_io_dsm_set_lpm
	imx6sl_ddr_io_set_lpm
	b	ddr_io_set_lpm_dsm_done
sx_io_dsm_set_lpm:
	imx6sx_ddr_io_set_lpm
ddr_io_set_lpm_dsm_done:

	/*
	 * mask all GPC interrupts before
	 * enabling the RBC counters to
	 * avoid the counter starting too
	 * early if an interupt is already
	 * pending.
	 */
	ldr	r8, =IMX_IO_P2V(MX6Q_CCM_BASE_ADDR)
	/* save CCM base in r9 */
	mov	r9, r8
	ldr	r8, =IMX_IO_P2V(MX6Q_GPC_BASE_ADDR)
	ldr	r4, [r8, #MX6Q_GPC_IMR1]
	ldr	r5, [r8, #MX6Q_GPC_IMR2]
	ldr	r6, [r8, #MX6Q_GPC_IMR3]
	ldr	r7, [r8, #MX6Q_GPC_IMR4]

	ldr	r3, =0xffffffff
	str	r3, [r8, #MX6Q_GPC_IMR1]
	str	r3, [r8, #MX6Q_GPC_IMR2]
	str	r3, [r8, #MX6Q_GPC_IMR3]
	str	r3, [r8, #MX6Q_GPC_IMR4]

	/*
	 * enable the RBC bypass counter here
	 * to hold off the interrupts. RBC counter
	 * = 32 (1ms), Minimum RBC delay should be
	 * 400us for the analog LDOs to power down.
	 */
	ldr	r3, [r9, #MX6Q_CCM_CCR]
	bic	r3, r3, #(0x3f << 21)
	orr	r3, r3, #(0x20 << 21)
	str	r3, [r9, #MX6Q_CCM_CCR]

	/* enable the counter. */
	ldr	r3, [r9, #MX6Q_CCM_CCR]
	orr	r3, r3, #(0x1 << 27)
	str	r3, [r9, #MX6Q_CCM_CCR]

	/* unmask all the GPC interrupts. */
	str	r4, [r8, #MX6Q_GPC_IMR1]
	str	r5, [r8, #MX6Q_GPC_IMR2]
	str	r6, [r8, #MX6Q_GPC_IMR3]
	str	r7, [r8, #MX6Q_GPC_IMR4]

	/*
	 * now delay for a short while (3usec)
	 * ARM is at 1GHz at this point
	 * so a short loop should be enough.
	 * this delay is required to ensure that
	 * the RBC counter can start counting in
	 * case an interrupt is already pending
	 * or in case an interrupt arrives just
	 * as ARM is about to assert DSM_request.
	 */
	ldr     r4, =2000
rbc_loop:
	sub     r4, r4, #0x1
	cmp     r4, #0x0
	bne     rbc_loop

	/*
	 * if internal ldo(VDDARM) bypassed,analog bypass
	 * it for DSM(0x1e) and restore it when resume(0x1f).
	 */
	ldr	r8, =IMX_IO_P2V(MX6Q_ANATOP_BASE_ADDR)
	ldr	r7, [r8, #MX6Q_ANATOP_CORE]
	and	r7, r7, #0x1f
	cmp	r7, #0x1f
	bne	ldo_check_done1
ldo_analog_bypass:
	ldr	r7, [r8, #MX6Q_ANATOP_CORE]
	bic	r7, r7, #0x1f
	orr	r7, r7, #0x1e
	str	r7, [r8, #MX6Q_ANATOP_CORE]
ldo_check_done1:

	/* Zzz, enter stop mode */
	wfi
	nop
	nop
	nop
	nop

	/*
	 * run to here means there is pending
	 * wakeup source, system should auto
	 * resume, we need to restore DDR IO first
	 */

	/* restore it with 0x1f if use ldo bypass mode.*/
	ldr	r8, =IMX_IO_P2V(MX6Q_ANATOP_BASE_ADDR)
	ldr	r7, [r8, #MX6Q_ANATOP_CORE]
	and	r7, r7, #0x1f
	cmp	r7, #0x1e
	bne	ldo_check_done2
ldo_bypass_restore:
	ldr	r7, [r8, #MX6Q_ANATOP_CORE]
	orr	r7, r7, #0x1f
	str	r7, [r8, #MX6Q_ANATOP_CORE]
ldo_check_done2:

	add	r10, r0, #MX6_SUSPEND_IRAM_SIZE
	/* skip the lr saved in iram */
	sub	r10, r10, #0x4
	/* skip the cpu type saved in iram */
	sub	r10, r10, #0x4

	ldr	r8, =IMX_IO_P2V(MX6Q_IOMUXC_BASE_ADDR)

	cmp	r2, #MXC_CPU_IMX6Q
	bne	dl_io_restore
	imx6dq_ddr_io_restore
	b	ddr_io_restore_done
dl_io_restore:
	cmp     r2, #MXC_CPU_IMX6DL
	bne	sl_io_restore
	imx6dl_ddr_io_restore
	b	ddr_io_restore_done
sl_io_restore:
	cmp	r2, #MXC_CPU_IMX6SL
	bne	sx_io_restore
	imx6sl_ddr_io_restore
	ldr	r8, =IMX_IO_P2V(MX6Q_MMDC_P0_BASE_ADDR)
	/* reset read FIFO, RST_RD_FIFO */
	ldr	r7, =MX6Q_MMDC_MPDGCTRL0
	ldr	r6, [r8, r7]
	orr     r6, r6, #(1 << 31)
	str	r6, [r8, r7]
fifo_reset1_wait:
	ldr	r6, [r8, r7]
	and	r6, r6, #(1 << 31)
	cmp	r6, #0
	bne	fifo_reset1_wait

	/* reset FIFO a second time */
	ldr	r6, [r8, r7]
	orr     r6, r6, #(1 << 31)
	str	r6, [r8, r7]
fifo_reset2_wait:
	ldr	r6, [r8, r7]
	and	r6, r6, #(1 << 31)
	cmp	r6, #0
	bne	fifo_reset2_wait
	b	ddr_io_restore_done
sx_io_restore:
	imx6sx_ddr_io_restore
ddr_io_restore_done:

	ldr	r8, =IMX_IO_P2V(MX6Q_MMDC_P0_BASE_ADDR)
	/* let DDR out of self-refresh. */
	ldr	r7, [r8, #MX6Q_MMDC_MAPSR]
	bic	r7, r7, #(1 << 21)
	str	r7, [r8, #MX6Q_MMDC_MAPSR]

poll_dvfs_clear_2:
	ldr	r7, [r8, #MX6Q_MMDC_MAPSR]
	ands	r7, r7, #(1 << 25)
	bne     poll_dvfs_clear_2
	/* enable DDR auto power saving */
	ldr	r7, [r8, #MX6Q_MMDC_MAPSR]
	bic	r7, r7, #0x1
	str	r7, [r8, #MX6Q_MMDC_MAPSR]
	/* return to suspend finish */
	mov	pc, lr

resume:
	/* invalidate L1 I-cache first */
	mov     r1,     #0x0
	mcr     p15, 0, r1, c7, c5, 0
	mcr     p15, 0, r1, c7, c5, 0
	mcr     p15, 0, r1, c7, c5, 6
	/* enable the Icache and branch prediction */
	mov     r1, #0x1800
	mcr     p15, 0, r1, c1, c0, 0
	isb

	/* restore it with 0x1f if use ldo bypass mode.*/
	ldr	r8, =MX6Q_ANATOP_BASE_ADDR
	ldr	r7, [r8, #MX6Q_ANATOP_CORE]
	and	r7, r7, #0x1f
	cmp	r7, #0x1e
	bne	ldo_check_done3
	ldr	r7, [r8, #MX6Q_ANATOP_CORE]
	orr	r7, r7, #0x1f
	str	r7, [r8, #MX6Q_ANATOP_CORE]
ldo_check_done3:

	ldr	r5, =MX6Q_SRC_BASE_ADDR
	ldr	r10, [r5, #MX6Q_SRC_GPR2]
	ldmea	r10!, {lr}

	/* get cpu tpye */
	ldmea	r10!, {r2}

	/* clear core0's entry and parameter */
	ldr	r8, =MX6Q_SRC_BASE_ADDR
	mov	r7, #0
	str	r7, [r8, #MX6Q_SRC_GPR1]
	str	r7, [r8, #MX6Q_SRC_GPR2]

	ldr	r8, =MX6Q_IOMUXC_BASE_ADDR

	cmp	r2, #MXC_CPU_IMX6Q
	bne	dl_io_dsm_restore
	imx6dq_ddr_io_restore
	b	ddr_io_restore_dsm_done
dl_io_dsm_restore:
	cmp     r2, #MXC_CPU_IMX6DL
	bne	sl_io_dsm_restore
	imx6dl_ddr_io_restore
	b	ddr_io_restore_dsm_done
sl_io_dsm_restore:
	cmp	r2, #MXC_CPU_IMX6SL
	bne	sx_io_dsm_restore
	imx6sl_ddr_io_restore
	ldr	r8, =MX6Q_MMDC_P0_BASE_ADDR
	/* reset read FIFO, RST_RD_FIFO */
	ldr	r7, =MX6Q_MMDC_MPDGCTRL0
	ldr	r6, [r8, r7]
	orr     r6, r6, #(1 << 31)
	str	r6, [r8, r7]
dsm_fifo_reset1_wait:
	ldr	r6, [r8, r7]
	and	r6, r6, #(1 << 31)
	cmp	r6, #0
	bne	dsm_fifo_reset1_wait

	/* reset FIFO a second time */
	ldr	r6, [r8, r7]
	orr     r6, r6, #(1 << 31)
	str	r6, [r8, r7]
dsm_fifo_reset2_wait:
	ldr	r6, [r8, r7]
	and	r6, r6, #(1 << 31)
	cmp	r6, #0
	bne	dsm_fifo_reset2_wait
	b	ddr_io_restore_dsm_done
sx_io_dsm_restore:
	imx6sx_ddr_io_restore
ddr_io_restore_dsm_done:

	ldr	r8, =MX6Q_MMDC_P0_BASE_ADDR
	/* let DDR out of self-refresh */
	ldr	r7, [r8, #MX6Q_MMDC_MAPSR]
	bic	r7, r7, #(1 << 21)
	str	r7, [r8, #MX6Q_MMDC_MAPSR]

poll_dvfs_clear_1:
	ldr	r7, [r8, #MX6Q_MMDC_MAPSR]
	ands	r7, r7, #(1 << 25)
	bne	poll_dvfs_clear_1
	/* enable DDR auto power saving */
	ldr	r7, [r8, #MX6Q_MMDC_MAPSR]
	bic	r7, r7, #0x1
	str	r7, [r8, #MX6Q_MMDC_MAPSR]
	mov	pc, lr
