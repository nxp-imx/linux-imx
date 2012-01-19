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

#include <linux/init.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <asm/cacheflush.h>
#include <asm/mach-types.h>
#include <asm/localtimer.h>
#include <asm/smp_scu.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/mx6.h>
#include <mach/smp.h>
#include "src-reg.h"

static DEFINE_SPINLOCK(boot_lock);

static void __iomem *scu_base_addr(void)
{
	return IO_ADDRESS(SCU_BASE_ADDR);
}

void __cpuinit platform_secondary_init(unsigned int cpu)
{
	trace_hardirqs_off();

	spin_lock(&boot_lock);
	/*
	* if any interrupts are already enabled for the primary
	* core (e.g. timer irq), then they will not have been enabled
	* for us: do so
	*/
	gic_secondary_init(0);

	/*
	* Synchronise with the boot thread.
	*/

	spin_unlock(&boot_lock);

}

int __cpuinit boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long boot_entry;
	void __iomem *src_base = IO_ADDRESS(SRC_BASE_ADDR);
	unsigned int val;

	 /*
	  * set synchronisation state between this boot processor
	  * and the secondary one
	  */
	spin_lock(&boot_lock);

	/* set entry point for cpu1-cpu3*/
	boot_entry = virt_to_phys(mx6_secondary_startup);

	writel(boot_entry, src_base + SRC_GPR1_OFFSET + 4 * 2 * cpu);
	writel(0, src_base + SRC_GPR1_OFFSET + 4 * 2 * cpu + 4);

	smp_wmb();
	dsb();
	flush_cache_all();

	/* reset cpu<n> */
	val = readl(src_base + SRC_SCR_OFFSET);
	val |= 1 << (BP_SRC_SCR_CORE0_RST + cpu);
	val |= 1 << (BP_SRC_SCR_CORES_DBG_RST + cpu);
	writel(val, src_base + SRC_SCR_OFFSET);

	/*
	* now the secondary core is starting up let it run its
	* calibrations, then wait for it to finish
	*/
	spin_unlock(&boot_lock);

	return 0;

}

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
void __init smp_init_cpus(void)
{
	void __iomem *scu_base = scu_base_addr();
	unsigned int i, ncores;

	ncores = scu_base ? scu_get_core_count(scu_base) : 1;

	/* sanity check */
	if (ncores == 0) {
		pr_err("mx6: strange CM count of 0? Default to 1\n");
		ncores = 1;
	}
	if (ncores > NR_CPUS) {
		pr_warning("mx6: no. of cores (%d) greater than configured "
			"maximum of %d - clipping\n",
			ncores, NR_CPUS);
		ncores = NR_CPUS;
	}

	for (i = 0; i < ncores; i++)
		set_cpu_possible(i, true);

	set_smp_cross_call(gic_raise_softirq);
}
static void __init wakeup_secondary(void)
{

}

void __init platform_smp_prepare_cpus(unsigned int max_cpus)
{
	int i;
	void __iomem *scu_base = scu_base_addr();

	/*
	 * Initialise the present map, which describes the set of CPUs
	 * actually populated at the present time.
	 */
	for (i = 0; i < max_cpus; i++)
		set_cpu_present(i, true);

	/*
	 * Initialise the SCU and wake up the secondary core using
	 * wakeup_secondary().
	 */
	scu_enable(scu_base);
	wakeup_secondary();
}
