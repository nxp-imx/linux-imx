/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/smp.h>
#include <linux/delay.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <linux/io.h>
#include "src-reg.h"
#include <linux/sched.h>
#include <asm/cacheflush.h>

extern unsigned int num_cpu_idle_lock;
void __iomem *src_base = IO_ADDRESS(SRC_BASE_ADDR);

int platform_cpu_kill(unsigned int cpu)
{
	unsigned int val;

	val = jiffies;
	/* wait secondary cpu to die, timeout is 50ms */
	while (__raw_readl(src_base + SRC_GPR1_OFFSET + (8 * cpu) + 4) == 0) {
		if (time_after(jiffies, (unsigned long)(val + HZ / 20))) {
			printk(KERN_WARNING "cpu %d: cpu could not die\n", cpu);
			break;
		}
	}

	/*
	 * we're ready for shutdown now, so do it
	 */
	val = __raw_readl(src_base + SRC_SCR_OFFSET);
	val &= ~(1 << (BP_SRC_SCR_CORES_DBG_RST + cpu));
	val |= (1 << (BP_SRC_SCR_CORE0_RST + cpu));
	__raw_writel(val, src_base + SRC_SCR_OFFSET);

	val = jiffies;
	/* wait secondary cpu reset done, timeout is 10ms */
	while ((__raw_readl(src_base + SRC_SCR_OFFSET) &
		(1 << (BP_SRC_SCR_CORE0_RST + cpu))) != 0) {
		if (time_after(jiffies, (unsigned long)(val + HZ / 100))) {
			printk(KERN_WARNING "cpu %d: cpu reset fail\n", cpu);
			break;
		}
	}

	__raw_writel(0x0, src_base + SRC_GPR1_OFFSET + (8 * cpu) + 4);
	return 1;
}

/*
 * platform-specific code to shutdown a CPU
 * Called with IRQs disabled
 */
void platform_cpu_die(unsigned int cpu)
{
	unsigned int v;
	if (cpu == 0) {
		printk(KERN_ERR "CPU0 can't be disabled!\n");
		return;
	}
	flush_cache_all();
	asm volatile(
	"	mcr p15, 0, %1, c7, c5, 0\n" /* Invalidate I cache */
	"	mcr p15, 0, %1, c7, c10, 4\n" /* DSB */
	/*
	* Turn off coherency
	*/
	"	mrc p15, 0, %0, c1, c0, 1\n" /* Disable SMP in ACTLR */
	"	bic %0, %0, %3\n"
	"	mcr p15, 0, %0, c1, c0, 1\n"
	"	mrc p15, 0, %0, c1, c0, 0\n" /* Disable D cache in SCTLR */
	"	bic %0, %0, %2\n"
	"	mcr p15, 0, %0, c1, c0, 0\n"
	:	"=&r" (v)
	:	"r" (0), "Ir" (CR_C), "Ir" (0x40)
	:	"cc");
	/* Tell cpu0 to kill this core, as this core's cache is
	already disabled, and we want to set a flag to tell cpu0
	to kill this core, so I write the flag to this core's SRC
	parameter register, after cpu0 kill this core, it will
	clear this register. */

	__raw_writel(0x1, src_base + SRC_GPR1_OFFSET + (8 * cpu) + 4);

	for (;;) {
		/*
		 * Execute WFI
		 */
		asm(".word	0xe320f003\n"
		    :
		    :
		    : "memory", "cc");
		printk(KERN_ERR "cpu %d wake up from wfi !!!\n", cpu);
	}
	asm volatile(
	"	mrc	p15, 0, %0, c1, c0, 0\n" /* Enable D cache in SCTLR */
	"	orr	%0, %0, %1\n"
	"	mcr	p15, 0, %0, c1, c0, 0\n"
	"	mrc	p15, 0, %0, c1, c0, 1\n" /* Enable SMP in ACTLR */
	"	orr	%0, %0, %2\n"
	"	mcr	p15, 0, %0, c1, c0, 1\n"
	:	"=&r" (v)
	:	"Ir" (CR_C), "Ir" (0x40)
	:	"cc");
	__raw_writel(0x0, src_base + SRC_GPR1_OFFSET + (8 * cpu) + 4);
}

int platform_cpu_disable(unsigned int cpu)
{
	/*
	 * we don't allow CPU 0 to be shutdown (it is still too special
	 * e.g. clock tick interrupts)
	 */
	*((char *)(&num_cpu_idle_lock) + (u8)cpu) = 0xff;
	return cpu == 0 ? -EPERM : 0;
}
