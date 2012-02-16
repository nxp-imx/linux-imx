/*
 * Copyright (C) 2008-2012 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <mach/hardware.h>
#include <mach/mx51.h>
#include <mach/mx53.h>

#define ARM_CTRL_DCACHE  1 << 2
#define ARM_CTRL_ICACHE  1 << 12
#define ARM_AUXCR_L2EN   1 << 1

.macro PM_SET_BACKUP_REG, addr, num
	ldr r2, =\addr
	ldr r2, [r1, r2]
	str r2, [r3, #(\num * 4)]
.endm

.macro PM_SET_HIGHZ_PAD, addr
	ldr r2, =\addr
	str r4, [r1, r2]
.endm

.macro PM_SET_RESTORE_REG, addr, num
    ldr     r4, [r3, #(\num * 4)]
    ldr     r2, =\addr
    str     r4, [r1, r2]
.endm

.macro PM_SET_ADDR_REG, addr, reg
	mov	\reg, #(\addr & 0x000000FF)
	orr	\reg, \reg, #(\addr & 0x0000FF00)
	orr	\reg, \reg, #(\addr & 0x00FF0000)
	orr	\reg, \reg, #(\addr & 0xFF000000)
.endm

#define SUSPEND_ID_MX51 1
#define SUSPEND_ID_MX53 3
#define SUSPEND_ID_NONE 4

#define MX51_DRAM_SDCLK_PAD_CTRL_ADDR MX51_IO_ADDRESS(0x73FA84B8)
#define MX51_CCM_BASE MX51_IO_ADDRESS(0x73fd4000)
#define MX51_PLL1_BASE MX51_IO_ADDRESS(0x83f80000)

#define M4IF_MCR0_OFFSET	(0x008C)
#define M4IF_MCR0_FDVFS		(0x1 << 11)
#define M4IF_MCR0_FDVACK	(0x1 << 27)
#define IOMUXC_BASE_ADDR_VIRT	MX53_IO_ADDRESS(MX53_IOMUXC_BASE_ADDR)
#define M4IF_BASE_ADDR_VIRT	MX53_IO_ADDRESS(MX53_M4IF_BASE_ADDR)

#define IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM3		0x554
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS3	0x558
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM2		0x560
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDODT1	0x564
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS2	0x568
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDCLK_1	0x570
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_CAS		0x574
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDCLK_0	0x578
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS0	0x57c
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDODT0	0x580
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM0		0x584
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_RAS		0x588
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS1	0x590
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM1		0x594
#define IOMUXC_SW_PAD_CTL_GRP_ADDDS			0x6f0
#define IOMUXC_SW_PAD_CTL_GRP_B0DS			0x718
#define IOMUXC_SW_PAD_CTL_GRP_B1DS			0x71c
#define IOMUXC_SW_PAD_CTL_GRP_CTLDS			0x720
#define IOMUXC_SW_PAD_CTL_GRP_B2DS			0x728
#define IOMUXC_SW_PAD_CTL_GRP_B3DS			0x72c

/*
 *	cpu_do_suspend_workaround()
 *
 *	Suspend the processor (eg, wait for interrupt).
 *
 *	IRQs are already disabled.
 */
ENTRY(cpu_do_suspend_workaround)
    stmfd   sp!, {r4,r5,r6,r7,r9,r10,r11}     @ Save registers

    mov    r6, r0                       @save iomux address
    cmp     r6, #SUSPEND_ID_MX51
    bne     mx53_start               @ don't disable cache on imx53
    /* Disable L1 caches */
    mrc     p15, 0, r0, c1, c0, 0    @ R0 = system control reg
    bic     r0, r0, #ARM_CTRL_ICACHE @ Disable ICache
    bic     r0, r0, #ARM_CTRL_DCACHE @ Disable DCache
    mcr     p15, 0, r0, c1, c0, 0    @ Update system control reg

    mrc     p15, 1, r0, c0, c0, 1   @ Read CLIDR
    ands    r3, r0, #0x7000000       @ Isolate level of coherency
    mov     r3, r3, lsr #23         @ Cache level value (naturally aligned)
    beq     FinishedClean
    mov     r10, #0
Loop1Clean:
    add     r2, r10, r10, lsr #1    @ Work out cache level
    mov     r1, r0, lsr r2          @ R0 bottom 3 bits = Cache Type for this level
    and     r1, r1, #7              @ Get those 3 bits alone
    cmp     r1, #2
    blt     SkipClean                @ No cache or only instruction cache at this level
    mcr     p15, 2, r10, c0, c0, 0  @ Write the Cache Size selection register
    mov     r1, #0
    .long   0xF57FF06F              @ ISB
    mrc     p15, 1, r1, c0, c0, 0   @ Reads current Cache Size ID register
    and     r2, r1, #7             @ Extract the line length field
    add     r2, r2, #4              @ Add 4 for the line length offset (log2 16 bytes)
    ldr     r4, =0x3FF
    ands    r4, r4, r1, lsr #3      @ R4 is the max number on the way size (right aligned)
    clz     r5, r4                  @ R5 is the bit position of the way size increment
    ldr     r7, =0x00007FFF
    ands    r7, r7, r1, lsr #13     @ R7 is the max number of the index size (right aligned)
Loop2Clean:
    mov     r9, r4                  @ R9 working copy of the max way size (right aligned)
Loop3Clean:
    orr     r11, r10, r9, lsl r5    @ Factor in the way number and cache number into R11
    orr     r11, r11, r7, lsl r2    @ Factor in the index number
    mcr     p15, 0, r11, c7, c14, 2 @ Clean and invalidate by set/way
    subs    r9, r9, #1              @ Decrement the way number
    bge     Loop3Clean
    subs    r7, r7, #1              @ Decrement the index
    bge     Loop2Clean
SkipClean:
    add     r10, r10, #2            @ Increment the cache number
    cmp     r3, r10
    bgt     Loop1Clean

FinishedClean:

    /* Disable L2 cache */
    mrc     p15, 0, r0, c1, c0, 1   @ R0 = auxiliary control reg
    bic     r0, r0, #ARM_AUXCR_L2EN @ Disable L2 cache
    mcr     p15, 0, r0, c1, c0, 1   @ Update aux control reg

mx53_start:
    /* Do nothing for DDR */
    cmp     r6, #SUSPEND_ID_NONE
    beq     mx5x_wfi
	/*Set the DDR drive strength to low */
    cmp     r6, #SUSPEND_ID_MX51
    bne     mx53_reduce_ddr_drive_strength
	ldr r0, =MX51_DRAM_SDCLK_PAD_CTRL_ADDR
	ldr	r1, [r0]
	str r1, __mx5x_temp_stack
	bic r1, r1, #0x6
	str r1, [r0]
mx53_reduce_ddr_drive_strength:
    cmp     r6, #SUSPEND_ID_MX53
    bne     mx5x_wfi

mx53_force_ddr_selfrefresh:
    /* Point R0 at M4IF register set */
    ldr     r0, =M4IF_BASE_ADDR_VIRT

    /* Point R1 at IOMUX register set */
    ldr     r1, =IOMUXC_BASE_ADDR_VIRT

    /* Point R3 at temporary IRAM storage for DDR pad config */
    adr     r3, __mx5x_temp_stack

	PM_SET_BACKUP_REG IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM3,		0
	PM_SET_BACKUP_REG IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS3, 	1
	PM_SET_BACKUP_REG IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM2,		2
	PM_SET_BACKUP_REG IOMUXC_SW_PAD_CTL_PAD_DRAM_SDODT1,	3
	PM_SET_BACKUP_REG IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS2, 	4
	PM_SET_BACKUP_REG IOMUXC_SW_PAD_CTL_PAD_DRAM_SDCLK_1,	5
	PM_SET_BACKUP_REG IOMUXC_SW_PAD_CTL_PAD_DRAM_CAS,		6
	PM_SET_BACKUP_REG IOMUXC_SW_PAD_CTL_PAD_DRAM_SDCLK_0,	7
	PM_SET_BACKUP_REG IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS0,		8
	PM_SET_BACKUP_REG IOMUXC_SW_PAD_CTL_PAD_DRAM_SDODT0,	9
	PM_SET_BACKUP_REG IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM0,		10
	PM_SET_BACKUP_REG IOMUXC_SW_PAD_CTL_PAD_DRAM_RAS,		11
	PM_SET_BACKUP_REG IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS1, 	12
	PM_SET_BACKUP_REG IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM1,		13
	PM_SET_BACKUP_REG IOMUXC_SW_PAD_CTL_GRP_ADDDS,			14
	PM_SET_BACKUP_REG IOMUXC_SW_PAD_CTL_GRP_B0DS,			15
	PM_SET_BACKUP_REG IOMUXC_SW_PAD_CTL_GRP_B1DS, 			16
	PM_SET_BACKUP_REG IOMUXC_SW_PAD_CTL_GRP_CTLDS,			17
	PM_SET_BACKUP_REG IOMUXC_SW_PAD_CTL_GRP_B2DS, 			18
	PM_SET_BACKUP_REG IOMUXC_SW_PAD_CTL_GRP_B3DS, 			19

    /* Set FDVFS bit of M4IF_MCR0 to request DDR to enter self-refresh */
    ldr     r2,[r0, #M4IF_MCR0_OFFSET]
    orr     r2, r2, #M4IF_MCR0_FDVFS
    str     r2,[r0, #M4IF_MCR0_OFFSET]

    /* Poll FDVACK bit of M4IF_MCR to wait for DDR to enter self-refresh */
WAIT_SR_ACK:
    ldr     r2,[r0, #M4IF_MCR0_OFFSET]
    ands    r2, r2, #M4IF_MCR0_FDVACK
    beq     WAIT_SR_ACK

    /*
     * Set DSE of all DDR I/O pads to 0 => HighZ
     * except CKE which must drive during self-refresh
     * according to JEDEC
	 */

    ldr     r4, =0
	PM_SET_HIGHZ_PAD IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM3
    PM_SET_HIGHZ_PAD IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS3
    PM_SET_HIGHZ_PAD IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM2
    PM_SET_HIGHZ_PAD IOMUXC_SW_PAD_CTL_PAD_DRAM_SDODT1
    PM_SET_HIGHZ_PAD IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS2
    PM_SET_HIGHZ_PAD IOMUXC_SW_PAD_CTL_PAD_DRAM_SDCLK_1
    PM_SET_HIGHZ_PAD IOMUXC_SW_PAD_CTL_PAD_DRAM_CAS
    PM_SET_HIGHZ_PAD IOMUXC_SW_PAD_CTL_PAD_DRAM_SDCLK_0
    PM_SET_HIGHZ_PAD IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS0
    PM_SET_HIGHZ_PAD IOMUXC_SW_PAD_CTL_PAD_DRAM_SDODT0
    PM_SET_HIGHZ_PAD IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM0
    PM_SET_HIGHZ_PAD IOMUXC_SW_PAD_CTL_PAD_DRAM_RAS
    PM_SET_HIGHZ_PAD IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS1
    PM_SET_HIGHZ_PAD IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM1
    PM_SET_HIGHZ_PAD IOMUXC_SW_PAD_CTL_GRP_ADDDS
    PM_SET_HIGHZ_PAD IOMUXC_SW_PAD_CTL_GRP_B0DS
    PM_SET_HIGHZ_PAD IOMUXC_SW_PAD_CTL_GRP_B1DS
    PM_SET_HIGHZ_PAD IOMUXC_SW_PAD_CTL_GRP_B2DS
    PM_SET_HIGHZ_PAD IOMUXC_SW_PAD_CTL_GRP_B3DS
	/* use DSE=1 for CKE pin,when DDR is in self-refresh */
    ldr     r4, =1
    PM_SET_HIGHZ_PAD IOMUXC_SW_PAD_CTL_GRP_CTLDS
mx5x_wfi:
    /*
     * PLL1 workaround as the following: For mx51 only.
     * Before enter WFI
     *      (1) switch DDR and ARM to PLL2
     *      (2) Disable AREN bit to avoid PLL1 restart during MFN change)
     *      (3) set PLL1 to ~864Mhz with MFI = 8, MFN = 180, MFD = 179
     *          thus the equation |MFN/(MFD+1)|  < 1
     *      (4) Manual restart PLL1
     *      (5) Wait PLL1 lock
     * After CPU out of WFI
     *      (6) Set PLL1 to 800Mhz with only change MFN to 60, others keep
     *      (7) Wait MFN change complete by delay 4.6us,
     *      (8) Switch DDR and ARM back to PLL1
     */
	cmp r6, #SUSPEND_ID_MX51

	bne WFI
	PM_SET_ADDR_REG MX51_PLL1_BASE, r3
	PM_SET_ADDR_REG MX51_CCM_BASE, r4

	/* step 1 */
	ldr     r0, [r4, #0x14]
	bic     r0, r0, #(0x1 << 30)
	str     r0, [r4, #0x14]

1:
	ldr     r0, [r4, #0x48]
	ands    r0, r0, #(1 << 8)
	bne     1b

	ldr     r0, [r4, #0x0c]
	bic     r0, r0, #(0xf << 5)
	orr     r0, r0, #(0x1 << 8)
	str     r0, [r4, #0x0c]

	orr     r0, r0, #(1 << 2)
	str     r0, [r4, #0x0c]

	/* step 2 */
	ldr r0, [r3, #0x4]
	bic r0, r0, #0x2
	str r0, [r3, #0x4] /* disable auto-restart AREN bit */

	/* step 3 */
	mov r0, #0x80
	mov r1, #179
	mov r2, #180
	str r0, [r3, #0x08]
	str r0, [r3, #0x1c]

	str r1, [r3, #0x0c]
	str r1, [r3, #0x20]

	str r2, [r3, #0x10]
	str r2, [r3, #0x24]

	/* step 4 */
	ldr r0, =0x00001236 /* Set PLM =1, manual restart and enable PLL*/
	str r0, [r3, #0x0]
1:      ldr r0, [r3, #0x0]
	ands r0, r0, #0x1
	beq 1b
WFI:
	mov r0,#0x0
	.long     0xe320f003              @ Opcode for WFI

	cmp     r6, #SUSPEND_ID_MX51
	bne      wfi_done

	/* step 5 */
	ldr     r0, =60
	str     r0, [r3, #0x10]

	/* step 6 */
	/* Load MFN by setting LDREQ */
	ldr     r0, [r3, #0x04]
	orr     r0, r0, #0x1
	str     r0, [r3, #0x04]

      /* Wait for LDREQ bit to clear. */
2:    ldr     r0, [r3, #0x4]
       tst    r0, #1
       bne   2b

	mov r0, #100 /* delay more than 4.6 us */
3:      subs r0, r0, #1
	bge 3b

	/* step 8 */
	ldr     r0, [r4, #0x0c]
	bic     r0, r0, #(1 << 2)
	str     r0, [r4, #0x0c]

	/* Source step_clk from LPAPM. */
	ldr     r0, [r4, #0x0c]
	bic     r0, r0, #(3 << 7)
	str     r0, [r4, #0x0c]

	ldr     r0, [r4, #0x14]
	orr     r0, r0, #(0x1 << 30)
	str     r0, [r4, #0x14]

3:
	ldr     r0, [r4, #0x48]
	ands    r0, r0, #(1 << 8)
	bne     3b

wfi_done:
    cmp     r6, #SUSPEND_ID_NONE
    beq     mx5x_post_wfi

    /*Set the DDR drive strength to max */
    cmp     r6, #SUSPEND_ID_MX51
    bne     mx53_restore_ddr_drive_strength
	ldr r0, =MX51_DRAM_SDCLK_PAD_CTRL_ADDR
	ldr r1, __mx5x_temp_stack
	str r1, [r0]
mx53_restore_ddr_drive_strength:
    cmp     r6, #SUSPEND_ID_MX53
    bne     mx5x_post_wfi

    ldr     r0, =M4IF_BASE_ADDR_VIRT
    ldr     r1, =IOMUXC_BASE_ADDR_VIRT
    adr     r3, __mx5x_temp_stack
	PM_SET_RESTORE_REG IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM3,		0
	PM_SET_RESTORE_REG IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS3, 	1
	PM_SET_RESTORE_REG IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM2,		2
	PM_SET_RESTORE_REG IOMUXC_SW_PAD_CTL_PAD_DRAM_SDODT1,	3
	PM_SET_RESTORE_REG IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS2, 	4
	PM_SET_RESTORE_REG IOMUXC_SW_PAD_CTL_PAD_DRAM_SDCLK_1,	5
	PM_SET_RESTORE_REG IOMUXC_SW_PAD_CTL_PAD_DRAM_CAS,		6
	PM_SET_RESTORE_REG IOMUXC_SW_PAD_CTL_PAD_DRAM_SDCLK_0,	7
	PM_SET_RESTORE_REG IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS0,	8
	PM_SET_RESTORE_REG IOMUXC_SW_PAD_CTL_PAD_DRAM_SDODT0,	9
	PM_SET_RESTORE_REG IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM0,		10
	PM_SET_RESTORE_REG IOMUXC_SW_PAD_CTL_PAD_DRAM_RAS,		11
	PM_SET_RESTORE_REG IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS1, 	12
	PM_SET_RESTORE_REG IOMUXC_SW_PAD_CTL_PAD_DRAM_DQM1,		13
	PM_SET_RESTORE_REG IOMUXC_SW_PAD_CTL_GRP_ADDDS,			14
	PM_SET_RESTORE_REG IOMUXC_SW_PAD_CTL_GRP_B0DS,			15
	PM_SET_RESTORE_REG IOMUXC_SW_PAD_CTL_GRP_B1DS, 			16
	PM_SET_RESTORE_REG IOMUXC_SW_PAD_CTL_GRP_CTLDS,			17
	PM_SET_RESTORE_REG IOMUXC_SW_PAD_CTL_GRP_B2DS, 			18
	PM_SET_RESTORE_REG IOMUXC_SW_PAD_CTL_GRP_B3DS, 			19

    /* Clear FDVFS bit of M4IF_MCR0 to request DDR to exit self-refresh */
    ldr     r2,[r0, #M4IF_MCR0_OFFSET]
    bic     r2, r2, #M4IF_MCR0_FDVFS
    str     r2,[r0, #M4IF_MCR0_OFFSET]

    /* Poll FDVACK bit of M4IF_MCR to wait for DDR to exit self-refresh */
WAIT_AR_ACK:
    ldr     r2,[r0, #M4IF_MCR0_OFFSET]
    ands    r2, r2, #M4IF_MCR0_FDVACK
    bne     WAIT_AR_ACK

    cmp     r6, #SUSPEND_ID_MX51
    bne     mx53_end
mx5x_post_wfi:
    mov     r0, #0
    mcr     p15, 0, r0, c7, c5, 0   @ Invalidate inst cache

    /* Invalidate data caches */
    mrc     p15, 1, r0, c0, c0, 1   @ Read CLIDR
    ands    r3, r0, #0x7000000      @ Isolate level of coherency
    mov     r3, r3, lsr #23         @ Cache level value (naturally aligned)
    beq     FinishedInvalidate
    mov     r10, #0
Loop1Invalidate:
    add     r2, r10, r10, lsr #1    @ Work out cache level
    mov     r1, r0, lsr r2          @ R0 bottom 3 bits = Cache Type for this level
    and     r1, r1, #7              @ Get those 3 bits alone
    cmp     r1, #2
    blt     SkipInvalidate          @ No cache or only instruction cache at this level
    mcr     p15, 2, r10, c0, c0, 0  @ Write the Cache Size selection register
    mov     r1, #0
    .long   0xF57FF06F              @ ISB
    mrc     p15, 1, r1, c0, c0, 0   @ Reads current Cache Size ID register
    and     r2, r1, #7              @ Extract the line length field
    add     r2, r2, #4              @ Add 4 for the line length offset (log2 16 bytes)
    ldr     r4, =0x3FF
    ands    r4, r4, r1, lsr #3      @ R4 is the max number on the way size (right aligned)
    clz     r5, r4                  @ R5 is the bit position of the way size increment
    ldr     r7, =0x00007FFF
    ands    r7, r7, r1, lsr #13     @ R7 is the max number of the index size (right aligned)
Loop2Invalidate:
    mov     r9, r4                  @ R9 working copy of the max way size (right aligned)
Loop3Invalidate:
    orr     r11, r10, r9, lsl r5    @ Factor in the way number and cache number into R11
    orr     r11, r11, r7, lsl r2    @ Factor in the index number
    mcr     p15, 0, r11, c7, c6, 2  @ Invalidate by set/way
    subs    r9, r9, #1              @ Decrement the way number
    bge     Loop3Invalidate
    subs    r7, r7, #1              @ Decrement the index
    bge     Loop2Invalidate
SkipInvalidate:
    add     r10, r10, #2            @ Increment the cache number
    cmp     r3, r10
    bgt     Loop1Invalidate

FinishedInvalidate:

    /* Enable L2 cache */
    mrc     p15, 0, r0, c1, c0, 1   @ R0 = auxiliary control reg
    orr     r0, r0, #ARM_AUXCR_L2EN @ Enable L2 cache
    mcr     p15, 0, r0, c1, c0, 1   @ Update aux control reg

    /* Enable L1 caches */
    mrc     p15, 0, r0, c1, c0, 0    @ R0 = system control reg
    orr     r0, r0, #ARM_CTRL_ICACHE @ Enable ICache
    orr     r0, r0, #ARM_CTRL_DCACHE @ Enable DCache
    mcr     p15, 0, r0, c1, c0, 0    @ Update system control reg

mx53_end:
    /* Restore registers */
    ldmfd sp!, {r4,r5,r6,r7,r9,r10,r11}
    mov		pc, lr
__mx5x_temp_stack:
	.space 128

	.type	cpu_do_suspend, #object
ENTRY(cpu_do_suspend)
	.word	cpu_do_suspend_workaround
	.size	cpu_do_suspend_workaround, . - cpu_do_suspend_workaround
