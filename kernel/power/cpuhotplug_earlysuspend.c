/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/earlysuspend.h>
#include <linux/cpu.h>

enum {
	DEBUG_USER_STATE = 1U << 0,
	DEBUG_SUSPEND = 1U << 2,
	DEBUG_VERBOSE = 1U << 3,
};
static int debug_mask = DEBUG_USER_STATE;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

static DEFINE_PER_CPU(int, tag);
static struct work_struct cpu_up_work;
static struct workqueue_struct *cpu_op_workqueue;


static void earlysuspend_cpu_op(int cpu, bool status)
{
	/* tag the cpu is onling/offline */
	if (status) {
		per_cpu(tag, cpu) = 0;
		cpu_up(cpu);
	} else {
		per_cpu(tag, cpu) = 1;
		cpu_down(cpu);
	}
}

static void cpuhotplug_early_suspend(struct early_suspend *p)
{
	int first_cpu, cpu;
	/* skip the first cpu, cpu0 always online */
	first_cpu = cpumask_first(cpu_online_mask);
	for_each_possible_cpu(cpu) {
		if (cpu == first_cpu)
			continue;
		if (cpu_online(cpu))
			earlysuspend_cpu_op(cpu, false);
	}
}

static void cpu_up_work_func(struct work_struct *work)
{
	int first_cpu, c;
	/* skip the first cpu, cpu0 always online */
	first_cpu = cpumask_first(cpu_online_mask);
	for_each_possible_cpu(c) {
		if (c == first_cpu)
			continue;
		if (debug_mask & DEBUG_SUSPEND)
			pr_info(" %s: CPU%d tag %d\n", __func__, c,
				       per_cpu(tag, c));
		if (!cpu_online(c) && per_cpu(tag, c))
			earlysuspend_cpu_op(c, true);
	}
}


static void cpuhotplug_late_resume(struct early_suspend *p)
{
	if (debug_mask & DEBUG_SUSPEND)
		pr_info(" %s: bootup secondary cpus\n", __func__);
	queue_work(cpu_op_workqueue, &cpu_up_work);

}

struct early_suspend cpuhotplug_earlysuspend = {
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB,
	.suspend = cpuhotplug_early_suspend,
	.resume = cpuhotplug_late_resume,
};


static int __init cpuhotplug_earlysuspend_init(void)
{
	cpu_op_workqueue = create_workqueue("cpu hotplug earlysuspend wq");
	INIT_WORK(&cpu_up_work, cpu_up_work_func);

	register_early_suspend(&cpuhotplug_earlysuspend);
	return 0;
}

static void __exit cpuhotplug_earlysuspend_exit(void)
{
	if (NULL != cpu_op_workqueue)
		destroy_workqueue(cpu_op_workqueue);
	unregister_early_suspend(&cpuhotplug_earlysuspend);
}

module_init(cpuhotplug_earlysuspend_init);
module_exit(cpuhotplug_earlysuspend_exit);
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
