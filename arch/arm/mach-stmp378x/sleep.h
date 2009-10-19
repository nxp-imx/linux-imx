/*
 *  Deep Sleep related defines
 *
 *  Author: Vitaly Wool <vital@embeddedalley.com>
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
#ifndef __PM_H__
#define __PM_H__

#include <mach/regs-clkctrl.h>

#define MMUTTB1_MASK		0x00003FE0
#define MMUTTBC_MASK		0xFFFFFFFC

#define LINK_OFFS	0x08
#define MMUCTL_OFFS	0x0C
#define MMUAUXCTL_OFFS	0x10
#define MMUCPACCESS_OFS	0x14
#define MMUTTB_OFFS	0x18
#define MMUPID_OFFS	0x1C
#define MMUDOMAIN_OFFS	0x20
#define SVC_R8_OFFS	0x2C
#define SVC_SP_OFFS	0x40
#define SVC_SPSR_OFFS	0x44
#define FIQ_SPSR_OFFS	0x48
#define FIQ_R8_OFFS	0x4C
#define FIQ_SP_OFFS	0x60
#define ABT_R8_OFFS	0x68
#define ABT_SPSR_OFFS	0x7C
#define ABT_SP_OFFS	0x80
#define IRQ_R8_OFFS	0x88
#define IRQ_SPSR_OFFS	0x9C
#define IRQ_SP_OFFS	0xA0
#define UND_SPSR_OFFS	0xA8
#define UND_SP_OFFS	0xAC
#define SYS_SPSR_OFFS	0xB4
#define SYS_SP_OFFS	0xB8

#ifndef __ASSEMBLER__
#define SLEEP_DATA_FINGERPRINT 0xdeadbeef
struct sleep_data {
	u32		fingerprint;
	u32		wake_addr;
	u32		link_addr;
	u32		mmuctl;
	u32		mmuauxctl;
	u32		mmucpaccess;
	u32		mmuttb;
	u32		mmupid;
	u32		mmudomain;
	u32		svc_r6;
	u32		svc_r7;
	u32		svc_r8;
	u32		svc_r9;
	u32		svc_r10;
	u32		svc_r11;
	u32		svc_r12;
	u32		svc_sp;
	u32		svc_spsr;
	u32		fiq_spsr;
	u32		fiq_r8;
	u32		fiq_r9;
	u32		fiq_r10;
	u32		fiq_r11;
	u32		fiq_r12;
	u32		fiq_sp;
	u32		fiq_lr;
	u32		abt_r8;
	u32		abt_r9;
	u32		abt_r10;
	u32		abt_r11;
	u32		abt_r12;
	u32		abt_spsr;
	u32		abt_sp;
	u32		abt_lr;
	u32		irq_r8;
	u32		irq_r9;
	u32		irq_r10;
	u32		irq_r11;
	u32		irq_r12;
	u32		irq_spsr;
	u32		irq_sp;
	u32		irq_lr;
	u32		und_spsr;
	u32		und_sp;
	u32		und_lr;
	u32		sys_spsr;
	u32		sys_sp;
	u32		sys_lr;
	u32		pinmux[0x100];
	u32		icoll_ctrl;
	union {
		u32		prio[0x10];
		u32		intr[0x80];
	} icoll;
	u32		clks[16];
	u32		old_c00;
	u32		old_c04;
};

extern int stmp_s2ram_alloc_sz;
void stmp37xx_cpu_suspend(void);
extern int stmp_standby_alloc_sz;
void stmp37xx_cpu_standby(void);
void stmp3xxx_suspend_timer(void);
void stmp3xxx_resume_timer(void);

#endif /* __ASSEMBLER__ */
#endif /* __PM_H__ */
