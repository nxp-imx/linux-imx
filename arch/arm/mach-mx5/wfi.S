/*
 * Copyright (C) 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */
/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/linkage.h>

#define ARM_CTRL_DCACHE  1 << 2
#define ARM_AUXCR_L2EN   1 << 1
/*
 *	cpu_cortexa8_do_idle()
 *
 *	Idle the processor (eg, wait for interrupt).
 *
 *	IRQs are already disabled.
 */
ENTRY(cpu_cortexa8_do_idle)

    mrc     p15, 0, r1, c1, c0, 1   @ R1 = auxiliary control reg
    ands    r2, r1, #ARM_AUXCR_L2EN @ Check if L2 is disabled
    beq     SkipL2Access

    mrc     p15, 0, r2, c1, c0, 0    @ R2 = system control reg
    bic     r2, r2, #ARM_CTRL_DCACHE @ Disable DCache
    mcr     p15, 0, r2, c1, c0, 0    @ Update system control reg

    bic     r1, r1, #ARM_AUXCR_L2EN  @ Disable L2 cache
    mcr     p15, 0, r1, c1, c0, 1    @ Update aux control reg

    ldr     r1, =(0x0 << 6)          @ A[6] = 0
    mcr     p15, 0, r1, c15, c9, 2   @ Read L2 tag RAM into L2 data 0 register
    mrc     p15, 0, r2, c15, c8, 0   @ Move L2 data 0 register to R2
    str     r2, [r0, #0x00]          @ Save tag info

    ldr     r1, =(0x1 << 6)          @ A[6] = 1
    mcr     p15, 0, r1, c15, c9, 2  @ Read L2 tag RAM into L2 data 0 register
    mrc     p15, 0, r2, c15, c8, 0  @ Move L2 data 0 register to R2
    str     r2, [r0, #0x04]         @ Save tag info

    ldr     r1, =(0x0 << 3)         @ A[6:3] = b0000
    mcr     p15, 0, r1, c15, c9, 3  @ Read L2 Data RAM into L2 data 0-2 register
    mrc     p15, 0, r2, c15, c8, 0  @ Move L2 data 0 register to R2
    str     r2, [r0, #0x08]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 1  @ Move L2 data 1 register to R2
    str     r2, [r0, #0x0C]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 5  @ Move L2 data 2 register to R2
    str     r2, [r0, #0x10]         @ Store data info

    ldr     r1, =(0x1 << 3)         @ A[6:3] = b0001
    mcr     p15, 0, r1, c15, c9, 3  @ Read L2 Data RAM into L2 data 0-2 register
    mrc     p15, 0, r2, c15, c8, 0  @ Move L2 data 0 register to R2
    str     r2, [r0, #0x14]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 1  @ Move L2 data 1 register to R2
    str     r2, [r0, #0x18]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 5  @ Move L2 data 2 register to R2
    str     r2, [r0, #0x1C]         @ Store data info

    ldr     r1, =(0x2 << 3)         @ A[6:3] = b0010
    mcr     p15, 0, r1, c15, c9, 3  @ Read L2 Data RAM into L2 data 0-2 register
    mrc     p15, 0, r2, c15, c8, 0  @ Move L2 data 0 register to R2
    str     r2, [r0, #0x20]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 1  @ Move L2 data 1 register to R2
    str     r2, [r0, #0x24]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 5  @ Move L2 data 2 register to R2
    str     r2, [r0, #0x28]         @ Store data info

    ldr     r1, =(0x3 << 3)         @ A[6:3] = b0011
    mcr     p15, 0, r1, c15, c9, 3  @ Read L2 Data RAM into L2 data 0-2 register
    mrc     p15, 0, r2, c15, c8, 0  @ Move L2 data 0 register to R2
    str     r2, [r0, #0x2C]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 1  @ Move L2 data 1 register to R2
    str     r2, [r0, #0x30]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 5  @ Move L2 data 2 register to R2
    str     r2, [r0, #0x34]         @ Store data info

    ldr     r1, =(0x4 << 3)         @ A[6:3] = b0100
    mcr     p15, 0, r1, c15, c9, 3  @ Read L2 Data RAM into L2 data 0-2 register
    mrc     p15, 0, r2, c15, c8, 0  @ Move L2 data 0 register to R2
    str     r2, [r0, #0x38]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 1  @ Move L2 data 1 register to R2
    str     r2, [r0, #0x3C]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 5  @ Move L2 data 2 register to R2
    str     r2, [r0, #0x40]         @ Store data info

    ldr     r1, =(0x5 << 3)         @ A[6:3] = b0101
    mcr     p15, 0, r1, c15, c9, 3  @ Read L2 Data RAM into L2 data 0-2 register
    mrc     p15, 0, r2, c15, c8, 0  @ Move L2 data 0 register to R2
    str     r2, [r0, #0x44]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 1  @ Move L2 data 1 register to R2
    str     r2, [r0, #0x48]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 5  @ Move L2 data 2 register to R2
    str     r2, [r0, #0x4C]         @ Store data info

    ldr     r1, =(0x6 << 3)         @ A[6:3] = b0110
    mcr     p15, 0, r1, c15, c9, 3  @ Read L2 Data RAM into L2 data 0-2 register
    mrc     p15, 0, r2, c15, c8, 0  @ Move L2 data 0 register to R2
    str     r2, [r0, #0x50]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 1  @ Move L2 data 1 register to R2
    str     r2, [r0, #0x54]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 5  @ Move L2 data 2 register to R2
    str     r2, [r0, #0x58]         @ Store data info

    ldr     r1, =(0x7 << 3)         @ A[6:3] = b0111
    mcr     p15, 0, r1, c15, c9, 3  @ Read L2 Data RAM into L2 data 0-2 register
    mrc     p15, 0, r2, c15, c8, 0  @ Move L2 data 0 register to R2
    str     r2, [r0, #0x5C]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 1  @ Move L2 data 1 register to R2
    str     r2, [r0, #0x60]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 5  @ Move L2 data 2 register to R2
    str     r2, [r0, #0x64]         @ Store data info

    ldr     r1, =(0x8 << 3)         @ A[6:3] = b1000
    mcr     p15, 0, r1, c15, c9, 3  @ Read L2 Data RAM into L2 data 0-2 register
    mrc     p15, 0, r2, c15, c8, 0  @ Move L2 data 0 register to R2
    str     r2, [r0, #0x68]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 1  @ Move L2 data 1 register to R2
    str     r2, [r0, #0x6C]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 5  @ Move L2 data 2 register to R2
    str     r2, [r0, #0x70]         @ Store data info

    ldr     r1, =(0x9 << 3)         @ A[6:3] = b1001
    mcr     p15, 0, r1, c15, c9, 3  @ Read L2 Data RAM into L2 data 0-2 register
    mrc     p15, 0, r2, c15, c8, 0  @ Move L2 data 0 register to R2
    str     r2, [r0, #0x74]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 1  @ Move L2 data 1 register to R2
    str     r2, [r0, #0x78]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 5  @ Move L2 data 2 register to R2
    str     r2, [r0, #0x7C]         @ Store data info

    ldr     r1, =(0xA << 3)         @ A[6:3] = b1010
    mcr     p15, 0, r1, c15, c9, 3  @ Read L2 Data RAM into L2 data 0-2 register
    mrc     p15, 0, r2, c15, c8, 0  @ Move L2 data 0 register to R2
    str     r2, [r0, #0x80]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 1  @ Move L2 data 1 register to R2
    str     r2, [r0, #0x84]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 5  @ Move L2 data 2 register to R2
    str     r2, [r0, #0x88]         @ Store data info

    ldr     r1, =(0xB << 3)         @ A[6:3] = b1011
    mcr     p15, 0, r1, c15, c9, 3  @ Read L2 Data RAM into L2 data 0-2 register
    mrc     p15, 0, r2, c15, c8, 0  @ Move L2 data 0 register to R2
    str     r2, [r0, #0x8C]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 1  @ Move L2 data 1 register to R2
    str     r2, [r0, #0x90]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 5  @ Move L2 data 2 register to R2
    str     r2, [r0, #0x94]         @ Store data info

    ldr     r1, =(0xC << 3)         @ A[6:3] = b1100
    mcr     p15, 0, r1, c15, c9, 3  @ Read L2 Data RAM into L2 data 0-2 register
    mrc     p15, 0, r2, c15, c8, 0  @ Move L2 data 0 register to R2
    str     r2, [r0, #0x98]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 1  @ Move L2 data 1 register to R2
    str     r2, [r0, #0x9C]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 5  @ Move L2 data 2 register to R2
    str     r2, [r0, #0xA0]         @ Store data info

    ldr     r1, =(0xD << 3)         @ A[6:3] = b1101
    mcr     p15, 0, r1, c15, c9, 3  @ Read L2 Data RAM into L2 data 0-2 register
    mrc     p15, 0, r2, c15, c8, 0  @ Move L2 data 0 register to R2
    str     r2, [r0, #0xA4]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 1  @ Move L2 data 1 register to R2
    str     r2, [r0, #0xA8]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 5  @ Move L2 data 2 register to R2
    str     r2, [r0, #0xAC]         @ Store data info

    ldr     r1, =(0xE << 3)         @ A[6:3] = b1110
    mcr     p15, 0, r1, c15, c9, 3  @ Read L2 Data RAM into L2 data 0-2 register
    mrc     p15, 0, r2, c15, c8, 0  @ Move L2 data 0 register to R2
    str     r2, [r0, #0xB0]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 1  @ Move L2 data 1 register to R2
    str     r2, [r0, #0xB4]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 5  @ Move L2 data 2 register to R2
    str     r2, [r0, #0xB8]         @ Store data info

    ldr     r1, =(0xF << 3)         @ A[6:3] = b1111
    mcr     p15, 0, r1, c15, c9, 3  @ Read L2 Data RAM into L2 data 0-2 register
    mrc     p15, 0, r2, c15, c8, 0  @ Move L2 data 0 register to R2
    str     r2, [r0, #0xBC]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 1  @ Move L2 data 1 register to R2
    str     r2, [r0, #0xC0]         @ Store data info
    mrc     p15, 0, r2, c15, c8, 5  @ Move L2 data 2 register to R2
    str     r2, [r0, #0xC4]         @ Store data info

    ldr     r1, =(0x2 << 29) | (0x0 << 6)   @ WAY = A[31:29] = 2, A[6] = 0
    mcr     p15, 0, r1, c15, c9, 2  @ Read L2 tag RAM into L2 data 0 register
    mrc     p15, 0, r2, c15, c8, 0  @ Move L2 data 0 register to R2
    str     r2, [r0, #0xC8]         @ Save tag info

    ldr     r1, =(0x2 << 29) | (0x1 << 6)   @ WAY = A[31:29] = 2, A[6] = 1
    mcr     p15, 0, r1, c15, c9, 2  @ Read L2 tag RAM into L2 data 0 register
    mrc     p15, 0, r2, c15, c8, 0  @ Move L2 data 0 register to R2
    str     r2, [r0, #0xCC]         @ Save tag info

    ldr     r1, =(0x4 << 29) | (0x0 << 6)   @ WAY = A[31:29] = 4, A[6] = 0
    mcr     p15, 0, r1, c15, c9, 2  @ Read L2 tag RAM into L2 data 0 register
    mrc     p15, 0, r2, c15, c8, 0  @ Move L2 data 0 register to R2
    str     r2, [r0, #0xD0]         @ Save tag info

    ldr     r1, =(0x4 << 29) | (0x1 << 6)   @ WAY = A[31:29] = 4, A[6] = 1
    mcr     p15, 0, r1, c15, c9, 2  @ Read L2 tag RAM into L2 data 0 register
    mrc     p15, 0, r2, c15, c8, 0  @ Move L2 data 0 register to R2
    str     r2, [r0, #0xD4]         @ Save tag info

    ldr     r1, =(0x6 << 29) | (0x0 << 6)   @ WAY = A[31:29] = 6, A[6] = 0
    mcr     p15, 0, r1, c15, c9, 2  @ Read L2 tag RAM into L2 data 0 register
    mrc     p15, 0, r2, c15, c8, 0  @ Move L2 data 0 register to R2
    str     r2, [r0, #0xD8]         @ Save tag info

    ldr     r1, =(0x6 << 29) | (0x1 << 6)   @ WAY = A[31:29] = 6, A[6] = 1
    mcr     p15, 0, r1, c15, c9, 2  @ Read L2 tag RAM into L2 data 0 register
    mrc     p15, 0, r2, c15, c8, 0  @ Move L2 data 0 register to R2
    str     r2, [r0, #0xDC]         @ Save tag info

    .long     0xe320f003              @ Opcode for WFI

    ldr     r1, =(0x0 << 6)         @ A[6] = 0
    ldr     r2, [r0, #0x00]         @ Load tag info
    mcr     p15, 0, r2, c15, c8, 0  @ Move R2 to L2 data 0 register
    mcr     p15, 0, r1, c15, c8, 2  @ Write L2 data 0 register to L2 tag RAM

    ldr     r1, =(0x1 << 6)         @ A[6] = 1
    ldr     r2, [r0, #0x04]         @ Load tag info
    mcr     p15, 0, r2, c15, c8, 0  @ Move R2 to L2 data 0 register
    mcr     p15, 0, r1, c15, c8, 2  @ Write L2 data 0 register to L2 tag RAM

    ldr     r1, =(0x0 << 3)         @ A[6:3] = b0000
    ldr     r2, [r0, #0x08]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 0  @ Move R2 to L2 data 0 register
    ldr     r2, [r0, #0x0C]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 1  @ Move R2 to L2 data 1 register
    ldr     r2, [r0, #0x10]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 5  @ Move R2 to L2 data 2 register
    mcr     p15, 0, r1, c15, c8, 3  @ Write L2 data 0-2 registers to L2 data RAM

    ldr     r1, =(0x1 << 3)         @ A[6:3] = b0001
    ldr     r2, [r0, #0x14]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 0  @ Move R2 to L2 data 0 register
    ldr     r2, [r0, #0x18]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 1  @ Move R2 to L2 data 1 register
    ldr     r2, [r0, #0x1C]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 5  @ Move R2 to L2 data 2 register
    mcr     p15, 0, r1, c15, c8, 3  @ Write L2 data 0-2 registers to L2 data RAM

    ldr     r1, =(0x2 << 3)         @ A[6:3] = b0010
    ldr     r2, [r0, #0x20]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 0  @ Move R2 to L2 data 0 register
    ldr     r2, [r0, #0x24]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 1  @ Move R2 to L2 data 1 register
    ldr     r2, [r0, #0x28]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 5  @ Move R2 to L2 data 2 register
    mcr     p15, 0, r1, c15, c8, 3  @ Write L2 data 0-2 registers to L2 data RAM

    ldr     r1, =(0x3 << 3)         @ A[6:3] = b0011
    ldr     r2, [r0, #0x2C]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 0  @ Move R2 to L2 data 0 register
    ldr     r2, [r0, #0x30]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 1  @ Move R2 to L2 data 1 register
    ldr     r2, [r0, #0x34]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 5  @ Move R2 to L2 data 2 register
    mcr     p15, 0, r1, c15, c8, 3  @ Write L2 data 0-2 registers to L2 data RAM

    ldr     r1, =(0x4 << 3)         @ A[6:3] = b0100
    ldr     r2, [r0, #0x38]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 0  @ Move R2 to L2 data 0 register
    ldr     r2, [r0, #0x3C]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 1  @ Move R2 to L2 data 1 register
    ldr     r2, [r0, #0x40]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 5  @ Move R2 to L2 data 2 register
    mcr     p15, 0, r1, c15, c8, 3  @ Write L2 data 0-2 registers to L2 data RAM

    ldr     r1, =(0x5 << 3)         @ A[6:3] = b0101
    ldr     r2, [r0, #0x44]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 0  @ Move R2 to L2 data 0 register
    ldr     r2, [r0, #0x48]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 1  @ Move R2 to L2 data 1 register
    ldr     r2, [r0, #0x4C]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 5  @ Move R2 to L2 data 2 register
    mcr     p15, 0, r1, c15, c8, 3  @ Write L2 data 0-2 registers to L2 data RAM

    ldr     r1, =(0x6 << 3)         @ A[6:3] = b0110
    ldr     r2, [r0, #0x50]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 0  @ Move R2 to L2 data 0 register
    ldr     r2, [r0, #0x54]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 1  @ Move R2 to L2 data 1 register
    ldr     r2, [r0, #0x58]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 5  @ Move R2 to L2 data 2 register
    mcr     p15, 0, r1, c15, c8, 3  @ Write L2 data 0-2 registers to L2 data RAM

    ldr     r1, =(0x7 << 3)         @ A[6:3] = b0111
    ldr     r2, [r0, #0x5C]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 0  @ Move R2 to L2 data 0 register
    ldr     r2, [r0, #0x60]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 1  @ Move R2 to L2 data 1 register
    ldr     r2, [r0, #0x64]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 5  @ Move R2 to L2 data 2 register
    mcr     p15, 0, r1, c15, c8, 3  @ Write L2 data 0-2 registers to L2 data RAM

    ldr     r1, =(0x8 << 3)         @ A[6:3] = b1000
    ldr     r2, [r0, #0x68]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 0  @ Move R2 to L2 data 0 register
    ldr     r2, [r0, #0x6C]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 1  @ Move R2 to L2 data 1 register
    ldr     r2, [r0, #0x70]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 5  @ Move R2 to L2 data 2 register
    mcr     p15, 0, r1, c15, c8, 3  @ Write L2 data 0-2 registers to L2 data RAM

    ldr     r1, =(0x9 << 3)         @ A[6:3] = b1001
    ldr     r2, [r0, #0x74]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 0  @ Move R2 to L2 data 0 register
    ldr     r2, [r0, #0x78]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 1  @ Move R2 to L2 data 1 register
    ldr     r2, [r0, #0x7C]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 5  @ Move R2 to L2 data 2 register
    mcr     p15, 0, r1, c15, c8, 3  @ Write L2 data 0-2 registers to L2 data RAM

    ldr     r1, =(0xA << 3)         @ A[6:3] = b1010
    ldr     r2, [r0, #0x80]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 0  @ Move R2 to L2 data 0 register
    ldr     r2, [r0, #0x84]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 1  @ Move R2 to L2 data 1 register
    ldr     r2, [r0, #0x88]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 5  @ Move R2 to L2 data 2 register
    mcr     p15, 0, r1, c15, c8, 3  @ Write L2 data 0-2 registers to L2 data RAM

    ldr     r1, =(0xB << 3)         @ A[6:3] = b1011
    ldr     r2, [r0, #0x8C]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 0  @ Move R2 to L2 data 0 register
    ldr     r2, [r0, #0x90]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 1  @ Move R2 to L2 data 1 register
    ldr     r2, [r0, #0x94]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 5  @ Move R2 to L2 data 2 register
    mcr     p15, 0, r1, c15, c8, 3  @ Write L2 data 0-2 registers to L2 data RAM

    ldr     r1, =(0xC << 3)         @ A[6:3] = b1100
    ldr     r2, [r0, #0x98]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 0  @ Move R2 to L2 data 0 register
    ldr     r2, [r0, #0x9C]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 1  @ Move R2 to L2 data 1 register
    ldr     r2, [r0, #0xA0]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 5  @ Move R2 to L2 data 2 register
    mcr     p15, 0, r1, c15, c8, 3  @ Write L2 data 0-2 registers to L2 data RAM

    ldr     r1, =(0xD << 3)         @ A[6:3] = b1101
    ldr     r2, [r0, #0xA4]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 0  @ Move R2 to L2 data 0 register
    ldr     r2, [r0, #0xA8]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 1  @ Move R2 to L2 data 1 register
    ldr     r2, [r0, #0xAC]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 5  @ Move R2 to L2 data 2 register
    mcr     p15, 0, r1, c15, c8, 3  @ Write L2 data 0-2 registers to L2 data RAM

    ldr     r1, =(0xE << 3)         @ A[6:3] = b1110
    ldr     r2, [r0, #0xB0]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 0  @ Move R2 to L2 data 0 register
    ldr     r2, [r0, #0xB4]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 1  @ Move R2 to L2 data 1 register
    ldr     r2, [r0, #0xB8]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 5  @ Move R2 to L2 data 2 register
    mcr     p15, 0, r1, c15, c8, 3  @ Write L2 data 0-2 registers to L2 data RAM

    ldr     r1, =(0xF << 3)         @ A[6:3] = b1111
    ldr     r2, [r0, #0xBC]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 0  @ Move R2 to L2 data 0 register
    ldr     r2, [r0, #0xC0]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 1  @ Move R2 to L2 data 1 register
    ldr     r2, [r0, #0xC4]         @ Load data info
    mcr     p15, 0, r2, c15, c8, 5  @ Move R2 to L2 data 2 register
    mcr     p15, 0, r1, c15, c8, 3  @ Write L2 data 0-2 registers to L2 data RAM

    ldr     r1, =(0x2 << 29) | (0x0 << 6)   @ WAY = A[31:29] = 2, A[6] = 0
    ldr     r2, [r0, #0xC8]         @ Load tag info
    mcr     p15, 0, r2, c15, c8, 0  @ Move R2 to L2 data 0 register
    mcr     p15, 0, r1, c15, c8, 2  @ Write L2 data 0 register to L2 tag RAM

    ldr     r1, =(0x2 << 29) | (0x1 << 6)   @ WAY = A[31:29] = 2, A[6] = 1
    ldr     r2, [r0, #0xCC]         @ Load tag info
    mcr     p15, 0, r2, c15, c8, 0  @ Move R2 to L2 data 0 register
    mcr     p15, 0, r1, c15, c8, 2  @ Write L2 data 0 register to L2 tag RAM

    ldr     r1, =(0x4 << 29) | (0x0 << 6)   @ WAY = A[31:29] = 4, A[6] = 0
    ldr     r2, [r0, #0xD0]         @ Load tag info
    mcr     p15, 0, r2, c15, c8, 0  @ Move R2 to L2 data 0 register
    mcr     p15, 0, r1, c15, c8, 2  @ Write L2 data 0 register to L2 tag RAM

    ldr     r1, =(0x4 << 29) | (0x1 << 6)   @ WAY = A[31:29] = 4, A[6] = 1
    ldr     r2, [r0, #0xD4]         @ Load tag info
    mcr     p15, 0, r2, c15, c8, 0  @ Move R2 to L2 data 0 register
    mcr     p15, 0, r1, c15, c8, 2  @ Write L2 data 0 register to L2 tag RAM

    ldr     r1, =(0x6 << 29) | (0x0 << 6)   @ WAY = A[31:29] = 6, A[6] = 0
    ldr     r2, [r0, #0xD8]         @ Load tag info
    mcr     p15, 0, r2, c15, c8, 0  @ Move R2 to L2 data 0 register
    mcr     p15, 0, r1, c15, c8, 2  @ Write L2 data 0 register to L2 tag RAM

    ldr     r1, =(0x6 << 29) | (0x1 << 6)   @ WAY = A[31:29] = 6, A[6] = 1
    ldr     r2, [r0, #0xDC]         @ Load tag info
    mcr     p15, 0, r2, c15, c8, 0  @ Move R2 to L2 data 0 register
    mcr     p15, 0, r1, c15, c8, 2  @ Write L2 data 0 register to L2 tag RAM

    mrc     p15, 0, r1, c1, c0, 1   @ R1 = auxiliary control reg
    orr     r1, r1, #ARM_AUXCR_L2EN @ Enable L2 cache
    mcr     p15, 0, r1, c1, c0, 1   @ Update aux control reg

    mrc     p15, 0, r2, c1, c0, 0    @ R2 = system control reg
    orr     r2, r2, #ARM_CTRL_DCACHE @ Enable DCache
    mcr     p15, 0, r2, c1, c0, 0    @ Update system control reg

    b       Done

SkipL2Access:
    .long     0xe320f003              @ Opcode for WFI

Done:
    mov		pc, lr

	.type	cortexa8_idle_workaround, #object
ENTRY(cortexa8_idle_workaround)
	.word	cpu_cortexa8_do_idle
	.size	cortexa8_idle_workaround, . - cortexa8_idle_workaround
