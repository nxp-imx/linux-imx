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

#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/thermal.h>
#include <linux/syscalls.h>
#include "anatop_driver.h"

#define	CPUx_SHUTDOWN		"/sys/devices/system/cpu/cpu"

extern unsigned long temperature_hotplug;
static unsigned char cpu_shutdown[40];
static unsigned int cpu_mask;
static unsigned int anatop_thermal_cpufreq_is_init;
static DEFINE_PER_CPU(unsigned int, cpufreq_thermal_reduction_pctg);

int anatop_thermal_cpu_hotplug(bool cpu_on)
{
	unsigned int ret, i;
	char online;
	char cpu_number[9];
	int fd;

	ret = -EINVAL;
	if (cpu_on) {
		for (i = 1; i < 4; i++) {
			strcpy(cpu_shutdown, CPUx_SHUTDOWN);
			sprintf(cpu_number, "%d%s", i, "/online");
			strcat(cpu_shutdown, cpu_number);
			fd = sys_open((const char __user __force *)cpu_shutdown,
			  O_RDWR, 0700);
			if (fd >= 0) {
				sys_read(fd, &online, 1);
				if (online == '0') {
					sys_write(fd, (char *)"1", 1);
					cpu_mask &= ~(0x1 << i);
					ret = 0;
					break;
				}
				sys_close(fd);
			}
		}
	} else {
		for (i = 3; i > 0; i--) {
			strcpy(cpu_shutdown, CPUx_SHUTDOWN);
			sprintf(cpu_number, "%d%s", i, "/online");
			strcat(cpu_shutdown, cpu_number);
			fd = sys_open((const char __user __force *)cpu_shutdown,
					O_RDWR, 0700);
			if (fd >= 0) {
				sys_read(fd, &online, 1);
				if (online == '1') {
					sys_write(fd, (char *)"0", 1);
					cpu_mask |= 0x1 << i;
					ret = 0;
					break;
				}
				sys_close(fd);
			}
		}
	}
	return ret;
}

static int
imx_processor_get_max_state(struct thermal_cooling_device *cdev,
			unsigned long *state)
{
	*state = cpu_mask | (0x1 << 1) | (0x1 << 2) | (0x1 << 3);
	return 0;
}

static int
imx_processor_get_cur_state(struct thermal_cooling_device *cdev,
			unsigned long *cur_state)
{
	*cur_state = cpu_mask;
	return 0;
}

static int
imx_processor_set_cur_state(struct thermal_cooling_device *cdev,
			unsigned long state)
{
	int result = 0;
	int i;

	/* state =0 means we are at a low temp, we should try to attach the
	secondary CPUs that detached by thermal driver */
	if (!state) {
		for (i = 1; i < 4; i++) {
			if (cpu_mask && (0x1 << i)) {
				anatop_thermal_cpu_hotplug(true);
				temperature_hotplug = 0;
				break;
			}
		}
	} else
		printk(KERN_DEBUG "Temperature is about to reach hot point!\n");
	return result;
}

static int anatop_thermal_cpufreq_notifier(struct notifier_block *nb,
					 unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;
	unsigned long max_freq = 0;

	if (event != CPUFREQ_ADJUST)
		goto out;

	max_freq = (
	    policy->cpuinfo.max_freq *
	    (100 - per_cpu(cpufreq_thermal_reduction_pctg, policy->cpu) * 20)
	) / 100;

	cpufreq_verify_within_limits(policy, 0, max_freq);

out:
	return 0;
}
static struct notifier_block anatop_thermal_cpufreq_notifier_block = {
	.notifier_call = anatop_thermal_cpufreq_notifier,
};

void anatop_thermal_cpufreq_init(void)
{
	int i;

	for (i = 0; i < nr_cpu_ids; i++)
		if (cpu_present(i))
			per_cpu(cpufreq_thermal_reduction_pctg, i) = 0;

	i = cpufreq_register_notifier(&anatop_thermal_cpufreq_notifier_block,
				      CPUFREQ_POLICY_NOTIFIER);
	if (!i)
		anatop_thermal_cpufreq_is_init = 1;
}

void anatop_thermal_cpufreq_exit(void)
{
	if (anatop_thermal_cpufreq_is_init)
		cpufreq_unregister_notifier
		    (&anatop_thermal_cpufreq_notifier_block,
		     CPUFREQ_POLICY_NOTIFIER);

	anatop_thermal_cpufreq_is_init = 0;
}

struct thermal_cooling_device_ops imx_processor_cooling_ops = {
	.get_max_state = imx_processor_get_max_state,
	.get_cur_state = imx_processor_get_cur_state,
	.set_cur_state = imx_processor_set_cur_state,
};

