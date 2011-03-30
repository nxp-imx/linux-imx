/*
 * Copyright (C) 2008-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <mach/mx5x.h>

#define ARM_CTRL_DCACHE  1 << 2
#define ARM_CTRL_ICACHE  1 << 12
#define ARM_AUXCR_L2EN   1 << 1

.macro PM_SET_BACKUP_REG, addr, bitmask, val, num
	mov	r0, #(\addr & 0x000000FF)
	orr	r0, r0, #(\addr & 0x0000FF00)
	orr	r0, r0, #(\addr & 0x00FF0000)
	orr	r0, r0, #(\addr & 0xFF000000)
	ldr	r1, [r0]
	str r1, __mx5x_temp_stack + \num * 4
	bic r1, r1, #(\bitmask)
	orr r1, r1, #(\val)
	str r1, [r0]
.endm

.macro PM_SET_RESTORE_REG, addr, num
	mov	r0, #(\addr & 0x000000FF)
	orr	r0, r0, #(\addr & 0x0000FF00)
	orr	r0, r0, #(\addr & 0x00FF0000)
	orr	r0, r0, #(\addr & 0xFF000000)
	ldr r1, __mx5x_temp_stack + \num * 4
	str r1, [r0]
.endm

#define MX51_DRAM_SDCLK_PAD_CTRL_ADDR AIPS1_IO_ADDRESS(0x73FA84B8)
#define MX53_DRAM_SDCLK0_PAD_CTRL_ADDR AIPS1_IO_ADDRESS(0x73FA8578)
#define MX53_DRAM_SDCLK1_PAD_CTRL_ADDR AIPS1_IO_ADDRESS(0x73FA8570)
#define MX53_DRAM_DQM3_PAD_CTRL_ADDR AIPS1_IO_ADDRESS(0x73FA8554)
#define MX53_DRAM_SDQS3_PAD_CTRL_ADDR AIPS1_IO_ADDRESS(0x73FA8558)
#define MX53_DRAM_SDCKE1_PAD_CTRL_ADDR AIPS1_IO_ADDRESS(0x73FA855C)
#define MX53_DRAM_DQM2_PAD_CTRL_ADDR AIPS1_IO_ADDRESS(0x73FA8560)
#define MX53_DRAM_DODT1_PAD_CTRL_ADDR AIPS1_IO_ADDRESS(0x73FA8564)
#define MX53_DRAM_SDQS2_PAD_CTRL_ADDR AIPS1_IO_ADDRESS(0x73FA8568)
#define MX53_DRAM_RESET_PAD_CTRL_ADDR AIPS1_IO_ADDRESS(0x73FA856c)
#define MX53_DRAM_CAS_PAD_CTRL_ADDR AIPS1_IO_ADDRESS(0x73FA8574)
#define MX53_DRAM_SDQS0_PAD_CTRL_ADDR AIPS1_IO_ADDRESS(0x73FA857C)
#define MX53_DRAM_DODT0_PAD_CTRL_ADDR AIPS1_IO_ADDRESS(0x73FA8580)
#define MX53_DRAM_DQM0_PAD_CTRL_ADDR AIPS1_IO_ADDRESS(0x73FA8584)
#define MX53_DRAM_RAS_PAD_CTRL_ADDR AIPS1_IO_ADDRESS(0x73FA8588)
#define MX53_DRAM_SDQS1_PAD_CTRL_ADDR AIPS1_IO_ADDRESS(0x73FA8590)
#define MX53_DRAM_DQM1_PAD_CTRL_ADDR AIPS1_IO_ADDRESS(0x73FA8594)

#define MX53_DRAM_GRP_ADDDS_PAD_CTRL_ADDR AIPS1_IO_ADDRESS(0x73FA86f0)
#define MX53_DRAM_GRP_B0DS_PAD_CTRL_ADDR AIPS1_IO_ADDRESS(0x73FA8718)
#define MX53_DRAM_GRP_B1DS_PAD_CTRL_ADDR AIPS1_IO_ADDRESS(0x73FA871C)
#define MX53_DRAM_GRP_CTLDS_PAD_CTRL_ADDR AIPS1_IO_ADDRESS(0x73FA8720)
#define MX53_DRAM_GRP_B2DS_PAD_CTRL_ADDR AIPS1_IO_ADDRESS(0x73FA8728)
#define MX53_DRAM_GRP_B3DS_PAD_CTRL_ADDR AIPS1_IO_ADDRESS(0x73FA872C)




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

	/*Set the DDR drive strength to low */
    cmp     r6, #1
    bne     mx53_reduce_ddr_drive_strength
    PM_SET_BACKUP_REG MX51_DRAM_SDCLK_PAD_CTRL_ADDR, 0x6, 0, 0
mx53_reduce_ddr_drive_strength:
    cmp     r6, #3
    bne     mx5x_wfi
    PM_SET_BACKUP_REG MX53_DRAM_SDCLK0_PAD_CTRL_ADDR, 0x380000, 0x180000, 0
    PM_SET_BACKUP_REG MX53_DRAM_SDCLK1_PAD_CTRL_ADDR, 0x380000, 0x180000, 1
    PM_SET_BACKUP_REG MX53_DRAM_DQM3_PAD_CTRL_ADDR , 0x380000, 0x180000, 2
    PM_SET_BACKUP_REG MX53_DRAM_SDQS3_PAD_CTRL_ADDR, 0x380000, 0x180000, 3
    PM_SET_BACKUP_REG MX53_DRAM_SDCKE1_PAD_CTRL_ADDR, 0x380000, 0x180000, 4
    PM_SET_BACKUP_REG MX53_DRAM_DQM2_PAD_CTRL_ADDR , 0x380000, 0x180000, 5
    PM_SET_BACKUP_REG MX53_DRAM_DODT1_PAD_CTRL_ADDR, 0x380000, 0x180000, 6
    PM_SET_BACKUP_REG MX53_DRAM_SDQS2_PAD_CTRL_ADDR, 0x380000, 0x180000, 7
    PM_SET_BACKUP_REG MX53_DRAM_RESET_PAD_CTRL_ADDR, 0x380000, 0x180000, 8
    PM_SET_BACKUP_REG MX53_DRAM_CAS_PAD_CTRL_ADDR  , 0x380000, 0x180000, 9
    PM_SET_BACKUP_REG MX53_DRAM_SDQS0_PAD_CTRL_ADDR, 0x380000, 0x180000, 10
    PM_SET_BACKUP_REG MX53_DRAM_DODT0_PAD_CTRL_ADDR, 0x380000, 0x180000, 11
    PM_SET_BACKUP_REG MX53_DRAM_DQM0_PAD_CTRL_ADDR , 0x380000, 0x180000, 12
    PM_SET_BACKUP_REG MX53_DRAM_RAS_PAD_CTRL_ADDR  , 0x380000, 0x180000, 13
    PM_SET_BACKUP_REG MX53_DRAM_SDQS1_PAD_CTRL_ADDR, 0x380000, 0x180000, 14
    PM_SET_BACKUP_REG MX53_DRAM_DQM1_PAD_CTRL_ADDR , 0x380000, 0x180000, 15

    PM_SET_BACKUP_REG MX53_DRAM_GRP_ADDDS_PAD_CTRL_ADDR, 0x380000, 0x180000, 16
    PM_SET_BACKUP_REG MX53_DRAM_GRP_B0DS_PAD_CTRL_ADDR, 0x380000, 0x180000, 17
    PM_SET_BACKUP_REG MX53_DRAM_GRP_B1DS_PAD_CTRL_ADDR, 0x380000, 0x180000, 18
    PM_SET_BACKUP_REG MX53_DRAM_GRP_CTLDS_PAD_CTRL_ADDR, 0x380000, 0x180000, 19
    PM_SET_BACKUP_REG MX53_DRAM_GRP_B2DS_PAD_CTRL_ADDR, 0x380000, 0x180000, 20
    PM_SET_BACKUP_REG MX53_DRAM_GRP_B3DS_PAD_CTRL_ADDR, 0x380000, 0x180000, 21

mx5x_wfi:
    .long     0xe320f003              @ Opcode for WFI

	/*Set the DDR drive strength to max */
    cmp     r6, #1
    bne     mx53_restore_ddr_drive_strength
    PM_SET_RESTORE_REG MX51_DRAM_SDCLK_PAD_CTRL_ADDR, 0
mx53_restore_ddr_drive_strength:
    cmp     r6, #3
    bne     mx5x_post_wfi
    PM_SET_RESTORE_REG MX53_DRAM_SDCLK0_PAD_CTRL_ADDR, 0
    PM_SET_RESTORE_REG MX53_DRAM_SDCLK1_PAD_CTRL_ADDR, 1
    PM_SET_RESTORE_REG MX53_DRAM_DQM3_PAD_CTRL_ADDR , 2
    PM_SET_RESTORE_REG MX53_DRAM_SDQS3_PAD_CTRL_ADDR, 3
    PM_SET_RESTORE_REG MX53_DRAM_SDCKE1_PAD_CTRL_ADDR, 4
    PM_SET_RESTORE_REG MX53_DRAM_DQM2_PAD_CTRL_ADDR , 5
    PM_SET_RESTORE_REG MX53_DRAM_DODT1_PAD_CTRL_ADDR, 6
    PM_SET_RESTORE_REG MX53_DRAM_SDQS2_PAD_CTRL_ADDR, 7
    PM_SET_RESTORE_REG MX53_DRAM_RESET_PAD_CTRL_ADDR, 8
    PM_SET_RESTORE_REG MX53_DRAM_CAS_PAD_CTRL_ADDR  , 9
    PM_SET_RESTORE_REG MX53_DRAM_SDQS0_PAD_CTRL_ADDR, 10
    PM_SET_RESTORE_REG MX53_DRAM_DODT0_PAD_CTRL_ADDR, 11
    PM_SET_RESTORE_REG MX53_DRAM_DQM0_PAD_CTRL_ADDR , 12
    PM_SET_RESTORE_REG MX53_DRAM_RAS_PAD_CTRL_ADDR  , 13
    PM_SET_RESTORE_REG MX53_DRAM_SDQS1_PAD_CTRL_ADDR, 14
    PM_SET_RESTORE_REG MX53_DRAM_DQM1_PAD_CTRL_ADDR , 15

    PM_SET_RESTORE_REG MX53_DRAM_GRP_ADDDS_PAD_CTRL_ADDR ,16
    PM_SET_RESTORE_REG MX53_DRAM_GRP_B0DS_PAD_CTRL_ADDR ,17
    PM_SET_RESTORE_REG MX53_DRAM_GRP_B1DS_PAD_CTRL_ADDR ,18
    PM_SET_RESTORE_REG MX53_DRAM_GRP_CTLDS_PAD_CTRL_ADDR ,19
    PM_SET_RESTORE_REG MX53_DRAM_GRP_B2DS_PAD_CTRL_ADDR ,20
    PM_SET_RESTORE_REG MX53_DRAM_GRP_B3DS_PAD_CTRL_ADDR ,21

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

    /* Restore registers */
    ldmfd sp!, {r4,r5,r6,r7,r9,r10,r11}
    mov		pc, lr
__mx5x_temp_stack:
	.space 128

	.type	cpu_do_suspend, #object
ENTRY(cpu_do_suspend)
	.word	cpu_do_suspend_workaround
	.size	cpu_do_suspend_workaround, . - cpu_do_suspend_workaround
