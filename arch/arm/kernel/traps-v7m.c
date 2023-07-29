/*
 * linux/arch/arm/kernel/traps-v7m.c
 *
 * Copyright (C) 2011 Dmitry Cherukhin, Emcraft Systems
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/io.h>

#include <asm/cacheflush.h>
#include <asm/sections.h>
#include <asm/page.h>
#include <asm/setup.h>
#include <asm/mpu.h>
#include <asm/mach/arch.h>
#include <asm/system_misc.h>
#include <asm/exception.h>

struct nvic_regs {
	unsigned long	some_regs1[837];	/* +000 */
	unsigned long	config_control;		/* +d14 */
	unsigned long	some_regs2[3];		/* +d18 */
	unsigned long	system_handler_csr;	/* +d24 */
	unsigned long	local_fault_status;	/* +d28 */
	unsigned long	hard_fault_status;	/* +d2c */
	unsigned long	some_regs3[1];		/* +d30 */
	unsigned long	mfar;			/* +d34 */
	unsigned long	bfar;			/* +d38 */
	unsigned long	some_regs4[21];		/* +d3c */
	unsigned long	mpu_type;               /* +d90 */
	unsigned long	mpu_control;		/* +d94 */
	unsigned long	reg_number;		/* +d98 */
	unsigned long	reg_base;		/* +d9c */
	unsigned long	reg_attr;		/* +da0 */
};

#define NVIC_REGS_BASE  0xE000E000
#define NVIC		((struct nvic_regs *)(NVIC_REGS_BASE))

enum config_control_bits {
	UNALIGN_TRP	= 0x00000008,
	DIV_0_TRP	= 0x00000010,
};

enum system_handler_csr_bits {
	BUSFAULTENA	= 0x00020000,
	USGFAULTENA	= 0x00040000,
};

enum local_fault_status_bits {
	IBUSERR		= 0x00000100,
	PRECISERR	= 0x00000200,
	IMPRECISERR	= 0x00000400,
	UNSTKERR	= 0x00000800,
	STKERR		= 0x00001000,
	BFARVALID	= 0x00008000,
	UNDEFINSTR	= 0x00010000,
	INVSTATE	= 0x00020000,
	INVPC		= 0x00040000,
	NOCP		= 0x00080000,
	UNALIGNED	= 0x01000000,
	DIVBYZERO	= 0x02000000,
};

enum hard_fault_status_bits {
	VECTTBL		= 0x00000002,
	FORCED		= 0x40000000,
};

enum interrupts {
	HARDFAULT	= 3,
	BUSFAULT	= 5,
	USAGEFAULT	= 6,
};

struct traps {
	char	*name;
	int	test_bit;
	int	handler;
};

static struct traps traps[] = {
	{"Vector Read error",		VECTTBL,	HARDFAULT},
	{"uCode stack push error",	STKERR,		BUSFAULT},
	{"uCode stack pop error",	UNSTKERR,	BUSFAULT},
	{"Escalated to Hard Fault",	FORCED,		HARDFAULT},
	{"Pre-fetch error",		IBUSERR,	BUSFAULT},
	{"Precise data bus error",	PRECISERR,	BUSFAULT},
	{"Imprecise data bus error",	IMPRECISERR,	BUSFAULT},
	{"No Coprocessor",		NOCP,		USAGEFAULT},
	{"Undefined Instruction",	UNDEFINSTR,	USAGEFAULT},
	{"Invalid ISA state",		INVSTATE,	USAGEFAULT},
	{"Return to invalid PC",	INVPC,		USAGEFAULT},
	{"Illegal unaligned access",	UNALIGNED,	USAGEFAULT},
	{"Divide By 0",			DIVBYZERO,	USAGEFAULT},
	{NULL}
};

extern void show_regs(struct pt_regs * regs);

/*
 * The function initializes Bus fault and Usage fault exceptions,
 * forbids unaligned data access and division by 0.
 */
void traps_v7m_init(void){
	writel(readl(&NVIC->system_handler_csr) | USGFAULTENA | BUSFAULTENA,
			&NVIC->system_handler_csr);
	writel(
#ifndef CONFIG_ARM_V7M_NO_UNALIGN_TRP
		UNALIGN_TRP |
#endif
		DIV_0_TRP | readl(&NVIC->config_control),
			&NVIC->config_control);
}

/*
 * The function prints information about the reason of the exception
 * @param name		name of current executable (process or kernel)
 * @param regs		state of registers when the exception occurs
 * @param in		IPSR, the number of the exception
 * @param addr		address caused the interrupt, or current pc
 * @param hstatus	status register for hard fault
 * @param lstatus	status register for local fault
 */
static void traps_v7m_print_message(char *name, struct pt_regs *regs,
		enum interrupts in, unsigned long addr,
		unsigned long hstatus, unsigned long lstatus)
{
	int i;
	printk("\n\n%s: fault at 0x%08lx [pc=0x%08lx, sp=0x%08lx]\n",
			name, addr, instruction_pointer(regs), regs->ARM_sp);
	for (i = 0; traps[i].name != NULL; ++i) {
		if ((traps[i].handler == HARDFAULT ? hstatus : lstatus)
				& traps[i].test_bit) {
			printk("%s\n", traps[i].name);
		}
	}
	printk("\n");
}

/*
 * Common routine for high-level exception handlers.
 * @param regs		state of registers when the exception occurs
 * @param in		IPSR, the number of the exception
 */
void traps_v7m_common(struct pt_regs *regs, enum interrupts in)
{
	unsigned long hstatus;
	unsigned long lstatus;
	unsigned long addr;

	hstatus = readl(&NVIC->hard_fault_status);
	lstatus = readl(&NVIC->local_fault_status);

	if (lstatus & BFARVALID && (in == BUSFAULT ||
			(in == HARDFAULT && hstatus & FORCED))) {
		addr = readl(&NVIC->bfar);
	} else {
		addr = instruction_pointer(regs);
	}

	writel(hstatus, &NVIC->hard_fault_status);
	writel(lstatus, &NVIC->local_fault_status);

#if defined(CONFIG_VFPM)
	if (lstatus & NOCP) {
		extern int vfpm_handle_exception(void);
		if (vfpm_handle_exception()) {
			return;
		}
	}
#endif

	if (user_mode(regs)) {
#if defined(CONFIG_DEBUG_USER)
		extern unsigned int user_debug;

		if (user_debug & UDBG_SEGV) {
			traps_v7m_print_message(current->comm, regs, in, addr,
					hstatus, lstatus);
		}
#endif
		if (lstatus & UNDEFINSTR) {
			send_sig(SIGTRAP, current, 0);
		}
		else {
			send_sig(SIGSEGV, current, 0);
		}
	} else {
		traps_v7m_print_message("KERNEL", regs, in, addr,
				hstatus, lstatus);
		show_regs(regs);
		panic(0);
	}
}

/*
 * High-level exception handler for exception 3 (Hard fault).
 * @param regs		state of registers when the exception occurs
 */
asmlinkage void __exception_irq_entry do_hardfault(struct pt_regs *regs)
{
	traps_v7m_common(regs, HARDFAULT);
}

/*
 * High-level exception handler for exception 5 (Bus fault).
 * @param regs		state of registers when the exception occurs
 */
asmlinkage void __exception_irq_entry do_busfault(struct pt_regs *regs)
{
	traps_v7m_common(regs, BUSFAULT);
}

/*
 * High-level exception handler for exception 6 (Usage fault).
 * @param regs		state of registers when the exception occurs
 */
asmlinkage void __exception_irq_entry do_usagefault(struct pt_regs *regs)
{
	traps_v7m_common(regs, USAGEFAULT);
}

