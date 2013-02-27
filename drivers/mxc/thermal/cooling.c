/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <mach/hardware.h>

#define	CPUx		"/sys/devices/system/cpu/cpu"
#define MAX_GOVERNOR_NAME_LEN 12
#define MAX_CPU_FREQ_LEN 7
#define MAX_CPU_ONLINE_LEN 1

/* Save the last hot point trigger temperature */
extern unsigned long temperature_cooling;
/* cooling device selection */
extern bool cooling_cpuhotplug;
extern struct cpu_op *(*get_cpu_op)(int *op);
/* Save the string we will read/write to the sys file */
static unsigned char cpu_sys_file[80];
static unsigned int cpu_mask;
static unsigned int anatop_thermal_cpufreq_is_init;
/* Save cpu operation table and current work point */
static struct cpu_op *cpu_op_tbl;
static int cpu_op_nr, cpu_op_cur;

/* Save the cpu freq necessray setting before thermal driver change it */
static unsigned int saved_cpu_freq[5];
static char saved_max_cpufreq_str[4][10];

/* New cpufreq to be set */
static unsigned int cpufreq_new;
static char cpufreq_new_str[10];

/* This variable record the cpufreq change, when we lower the
cpufreq, it minor 1, and when we promote cpufreq, it add 1, so
if it is 0, mean we didn't change the cpufreq */
static int cpufreq_change_count;

extern atomic_t thermal_on;
extern int thermal_notifier_call_chain(unsigned long val);
int anatop_thermal_get_cpufreq_cur(void)
{
	int ret = -EINVAL;
#ifdef CONFIG_CPU_FREQ
	unsigned int i, freq;

	freq = cpufreq_quick_get(0);
	saved_cpu_freq[abs(cpufreq_change_count)] = freq;
	printk(KERN_WARNING "cooling: cpu cur freq is %d\n", freq * 1000);
	freq *= 1000;

	for (i = 0; i < cpu_op_nr; i++) {
		if (freq == cpu_op_tbl[i].cpu_rate)
			break;
	}

	if (i >= cpu_op_nr) {
		printk(KERN_WARNING "cooling: can't get cpufreq\
			current operating point!\n");
		return ret;
	}
	cpu_op_cur = i;
#endif
	return 0;

}

int anatop_thermal_cpufreq_up(void)
{
	int ret = -EINVAL;
#ifdef CONFIG_CPU_FREQ
	int fd;

	anatop_thermal_get_cpufreq_cur();
	if (cpu_op_cur == 0 || cpu_op_cur >= cpu_op_nr) {
		printk(KERN_ERR "cooling: Bad cpu_op_cur!\n");
		return ret;
	}

	cpufreq_new = cpu_op_tbl[cpu_op_cur - 1].cpu_rate;
	printk(KERN_INFO "cooling: cpu max freq set to %s\n",
		saved_max_cpufreq_str[abs(cpufreq_change_count) - 1]);

	strcpy(cpu_sys_file, CPUx);
	strcat(cpu_sys_file, "0/cpufreq/scaling_max_freq");

	fd = sys_open((const char __user __force *)cpu_sys_file,
	  O_RDWR, 0700);
	if (fd >= 0) {
		sys_write(fd, saved_max_cpufreq_str[abs(cpufreq_change_count) - 1],
			strlen(saved_max_cpufreq_str[abs(cpufreq_change_count) - 1]));
		cpufreq_update_policy(0);
		sys_close(fd);
		ret = 0;
	}

	cpufreq_change_count++;
#endif
	return ret;
}
int anatop_thermal_cpufreq_down(void)
{
	int ret = -EINVAL;
#ifdef CONFIG_CPU_FREQ
	int fd;

	anatop_thermal_get_cpufreq_cur();
	if (cpu_op_cur == (cpu_op_nr - 1) || cpu_op_cur >= cpu_op_nr) {
		printk(KERN_ERR "cooling: Bad cpu_op_cur!\n");
		return ret;
	}

	cpufreq_new = cpu_op_tbl[cpu_op_cur + 1].cpu_rate;

	printk(KERN_INFO "cooling: cpu max freq set to %d\n", cpufreq_new);
	cpufreq_new /= 1000;

	strcpy(cpu_sys_file, CPUx);
	sprintf(cpufreq_new_str, "%d", cpufreq_new);
	strcat(cpu_sys_file, "0/cpufreq/scaling_max_freq");

	fd = sys_open((const char __user __force *)cpu_sys_file,
	  O_RDWR, 0700);
	if (fd >= 0) {
		sys_read(fd, saved_max_cpufreq_str[abs(cpufreq_change_count)],
			MAX_CPU_FREQ_LEN);
		sys_write(fd, cpufreq_new_str, strlen(cpufreq_new_str));
		sys_close(fd);
		ret = 0;
	}

	cpufreq_change_count--;
	cpufreq_update_policy(0);
#endif
	return ret;
}


int anatop_thermal_cpu_hotplug(bool cpu_on)
{
	int ret = -EINVAL;
#ifdef CONFIG_HOTPLUG
	unsigned int cpu;
	char online;
	char cpu_number[9];
	int fd;

	if (cpu_on) {
		for (cpu = 1; cpu < num_possible_cpus(); cpu++) {
			strcpy(cpu_sys_file, CPUx);
			sprintf(cpu_number, "%d%s", cpu, "/online");
			strcat(cpu_sys_file, cpu_number);
			fd = sys_open((const char __user __force *)cpu_sys_file,
			  O_RDWR, 0700);
			if (fd >= 0) {
				sys_read(fd, &online, MAX_CPU_ONLINE_LEN);
				if (online == '0') {
					sys_write(fd, (char *)"1", MAX_CPU_ONLINE_LEN);
					cpu_mask &= ~(0x1 << cpu);
					ret = 0;
					sys_close(fd);
					break;
				}
				sys_close(fd);
			}
		}
	} else {
		if (num_online_cpus() < 2)
			return ret;

		for (cpu = num_possible_cpus() - 1; cpu > 0; cpu--) {
			strcpy(cpu_sys_file, CPUx);
			sprintf(cpu_number, "%d%s", cpu, "/online");
			strcat(cpu_sys_file, cpu_number);
			fd = sys_open((const char __user __force *)cpu_sys_file,
					O_RDWR, 0700);
			if (fd >= 0) {
				sys_read(fd, &online, 1);
				if (online == '1') {
					sys_write(fd, (char *)"0", MAX_CPU_ONLINE_LEN);
					cpu_mask |= 0x1 << cpu;
					ret = 0;
					sys_close(fd);
					break;
				}
				sys_close(fd);
			}
		}
	}
#endif
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
	if (cooling_cpuhotplug) {
		if (!state) {
			if (atomic_read(&thermal_on))
				thermal_notifier_call_chain(0);
			for (i = 1; i < 4; i++) {
				if (cpu_mask && (0x1 << i)) {
					anatop_thermal_cpu_hotplug(true);
					temperature_cooling = 0;
					break;
				}
			}
		}
	} else {
		if (!state) {
			if (atomic_read(&thermal_on))
				thermal_notifier_call_chain(0);
			if (cpufreq_change_count < 0)
				anatop_thermal_cpufreq_up();
			else if (cpufreq_change_count > 0)
				anatop_thermal_cpufreq_down();
			temperature_cooling = 0;
		}
	}

	return result;
}

void anatop_thermal_cpufreq_init(void)
{
	cpu_op_tbl = get_cpu_op(&cpu_op_nr);
	anatop_thermal_cpufreq_is_init = 1;
}

void anatop_thermal_cpufreq_exit(void)
{
	anatop_thermal_cpufreq_is_init = 0;
}

struct thermal_cooling_device_ops imx_processor_cooling_ops = {
	.get_max_state = imx_processor_get_max_state,
	.get_cur_state = imx_processor_get_cur_state,
	.set_cur_state = imx_processor_set_cur_state,
};

