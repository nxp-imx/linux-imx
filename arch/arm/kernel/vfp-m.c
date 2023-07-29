/*
 * arch/arm/kernel/vfp-m.c
 *
 * Copyright (C) 2018 Christer Weinigel <christer@weinigel.se>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>

#include <asm/thread_notify.h>
#include <asm/v7m.h>
#include <asm/vfp.h>
#include <asm/exception.h>

#define FPCCR		((u32 *)0xe000ef34)	/* Floating-point Context Control Register */
#define FPCCR_ASPEN	(1 << 31)	/* Enable FPU stacking */
#define FPCCR_LSPEN	(1 << 30)	/* Enable lazy FPU stacking */
#define FPCCR_LSPACT	(1 << 0)	/* 1 => Deferred (lazy) FPU stacking is waiting to be saved */

#define FPCAR		((u32 *)0xe000ef38)	/* Floating-point Context Address Register */

#define CPACR		((u32 *)0xe000ed88)	/* Coprocessor Access Control Register */
#define CPACR_CP0_FULL	(0x3 << 20)	/* Full access to coprocessor 0 */
#define CPACR_CP1_FULL	(0x3 << 22)	/* Full access to coprocessor 1 */

#undef MVFR0
#define MVFR0		((u32 *)0xe000ef40)	/* Media and VFP Feature Register 0 */

static struct thread_info *vfpm_last_thread;

static void vfpm_disable_access(void)
{
	writel(readl(CPACR) & ~(CPACR_CP0_FULL | CPACR_CP1_FULL), CPACR);
}

static void vfpm_enable_access(void)
{
	writel(readl(CPACR) | CPACR_CP0_FULL | CPACR_CP1_FULL, CPACR);
}

static void vfpm_save_context(union vfp_state *vfp)
{
	u32 dummy;
	asm volatile("  stc     p11, cr0, [%0], #32*4"
		     : "=r" (dummy) : "0" (vfp->hard.fpregs) : "cc");
	asm("	fmrx %0, fpscr"
	    : "=r" (vfp->hard.fpscr) : : "cc");
}

static void vfpm_load_context(union vfp_state *vfp)
{
	u32 dummy;
	asm volatile("  ldc     p11, cr0, [%0], #32*4"
		     : "=r" (dummy) : "0" (vfp->hard.fpregs) : "cc");
	asm("	fmxr fpscr, %0"
	    : : "r" (vfp->hard.fpscr) : "cc");
}

static void vfpm_switch_to(struct thread_info *thread)
{
	vfpm_enable_access();

	if (vfpm_last_thread != thread) {
		if (vfpm_last_thread)
			vfpm_save_context(&vfpm_last_thread->vfpstate);

		vfpm_load_context(&thread->vfpstate);
		vfpm_last_thread = thread;
	}
}

int vfpm_handle_exception(void)
{
	struct thread_info *thread = current_thread_info();

	if ((readl(MVFR0) & 0xf0) != 0x20)
		return 0;

	WARN_ON(thread->vfpstate.hard.used);

	thread->vfpstate.hard.used = 1;
	vfpm_switch_to(thread);

	return 1;
}

static int vfpm_notifier(struct notifier_block *self, unsigned long cmd,
			 void *t)
{
	struct thread_info *thread = t;
	struct thread_info *current_thread = current_thread_info();

	switch (cmd) {
	case THREAD_NOTIFY_FLUSH:
		memset(&thread->vfpstate, 0, sizeof(thread->vfpstate));
		thread->vfpstate.hard.used = 0;
		/* fallthrough */

	case THREAD_NOTIFY_EXIT:
		vfpm_disable_access();
		vfpm_last_thread = NULL;
		break;

	case THREAD_NOTIFY_SWITCH:
		if (thread->vfpstate.hard.used)
			vfpm_switch_to(thread);
		else
			vfpm_disable_access();

		break;

	case THREAD_NOTIFY_COPY:
		if (current_thread->vfpstate.hard.used) {
			vfpm_save_context(&current_thread->vfpstate);

			thread->vfpstate.hard.used = 1;
			vfpm_last_thread = thread;
		} else
			memset(&thread->vfpstate, 0, sizeof(thread->vfpstate));
		break;
	}

	return NOTIFY_DONE;
}

static struct notifier_block vfpm_notifier_block = {
	.notifier_call	= vfpm_notifier,
};

static int __init vfpm_init(void)
{
	/* check for single-precision VFP operations */
	if (((readl(MVFR0) & MVFR0_SP_MASK) >> MVFR0_SP_BIT) != 0x2)
		return 0;

	printk(KERN_INFO "ARMv7-M VFP coprocessor found\n");
	if (((readl(MVFR0) & MVFR0_DP_MASK) >> MVFR0_DP_BIT) == 0x2)
               pr_info("VFP: Double precision floating points are supported\n");

	writel(0, FPCCR);	/* disable state preservation */
	vfpm_disable_access();

	elf_hwcap |= HWCAP_VFP;
	thread_register_notifier(&vfpm_notifier_block);

	return 0;
}

late_initcall(vfpm_init);
