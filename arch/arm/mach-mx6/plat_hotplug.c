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

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/smp.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <linux/io.h>
#include "src-reg.h"
#include <linux/sched.h>
#include <asm/cacheflush.h>

static atomic_t cpu_die_done = ATOMIC_INIT(0);
int platform_cpu_kill(unsigned int cpu)
{
	void __iomem *src_base = IO_ADDRESS(SRC_BASE_ADDR);
	unsigned int val;

	val = jiffies;
	/* wait secondary cpu to die, timeout is 500ms */
	while (atomic_read(&cpu_die_done) == 0) {
		if (time_after(jiffies, (unsigned long)(val + HZ / 2))) {
			printk(KERN_WARNING "cpu %d: cpu could not die\n", cpu);
			break;
		}
	}
	/*
	 * we're ready for shutdown now, so do it
	 */
	val = readl(src_base + SRC_SCR_OFFSET);
	val &= ~(1 << (BP_SRC_SCR_CORES_DBG_RST + cpu));
	writel(val, src_base + SRC_SCR_OFFSET);

	atomic_set(&cpu_die_done, 0);
	return 1;
}

/*
 * platform-specific code to shutdown a CPU
 * Called with IRQs disabled
 */
void platform_cpu_die(unsigned int cpu)
{
	if (cpu == 0) {
		printk(KERN_ERR "CPU0 can't be disabled!\n");
		return;
	}

	flush_cache_all();
	dsb();

	/* tell cpu0 to kill me */
	atomic_set(&cpu_die_done, 1);
	for (;;) {
		/*
		 * Execute WFI
		 */
		cpu_do_idle();
		printk(KERN_INFO "CPU%u: spurious wakeup call\n", cpu);
	}
}

int platform_cpu_disable(unsigned int cpu)
{
	/*
	 * we don't allow CPU 0 to be shutdown (it is still too special
	 * e.g. clock tick interrupts)
	 */
	return cpu == 0 ? -EPERM : 0;
}
