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


struct cpu_op {
	u32 pll_reg;
	u32 pll_rate;
	u32 cpu_rate;
	u32 pdr0_reg;
	u32 pdf;
	u32 mfi;
	u32 mfd;
	u32 mfn;
	u32 cpu_voltage;
	u32 cpu_podf;
};

#define	CPUx		"/sys/devices/system/cpu/cpu"

extern unsigned long temperature_cooling;
extern bool cooling_cpuhotplug;
extern struct cpu_op *(*get_cpu_op)(int *op);
static unsigned char cpu_sys_file[80];
static unsigned int cpu_mask;
static unsigned int anatop_thermal_cpufreq_is_init;
static struct cpu_op *cpu_op_tbl;
static int cpu_op_nr, cpu_op_cur;
static unsigned int cpufreq;
static unsigned int cpufreq_new;
static char cpufreq_new_str[10];
static int cpufreq_change_count;

int anatop_thermal_cpufreq_get_cur(void)
{
	int i;

	cpufreq = cpufreq_quick_get(0);
	cpufreq *= 1000;

	for (i = 0; i < cpu_op_nr; i++) {
		if (cpufreq == cpu_op_tbl[i].cpu_rate)
			break;
	}

	if (i >= cpu_op_nr) {
		printk(KERN_WARNING "%s: can't get cpufreq\
			current operating point!\n", __func__);
		return -EINVAL;
	}
	cpu_op_cur = i;

	return 1;

}

int anatop_thermal_cpufreq_up(void)
{
	int fd;
	char cpu_number[30];

	anatop_thermal_cpufreq_get_cur();
	if (cpu_op_cur == 0 || cpu_op_cur >= cpu_op_nr)
		printk(KERN_ERR "%s: Bad cpu_op_cur!\n", __func__);

	cpufreq_new = cpu_op_tbl[cpu_op_cur - 1].cpu_rate;
	printk(KERN_INFO "%s: cpufreq up from %d to %d\n",
		__func__, cpufreq, cpufreq_new);
	cpufreq_new /= 1000;

	strcpy(cpu_sys_file, CPUx);
	sprintf(cpu_number, "%d%s", 0, "/cpufreq/scaling_setspeed");
	sprintf(cpufreq_new_str, "%d", cpufreq_new);
	strcat(cpu_sys_file, cpu_number);

	fd = sys_open((const char __user __force *)cpu_sys_file,
	  O_RDWR, 0700);
	if (fd >= 0) {
		sys_write(fd, cpufreq_new_str, strlen(cpufreq_new_str));
		sys_close(fd);
	}
	cpufreq_change_count++;

	return 1;
}
int anatop_thermal_cpufreq_down(void)
{
	int fd;
	char cpu_number[30];

	anatop_thermal_cpufreq_get_cur();
	if (cpu_op_cur == (cpu_op_nr - 1) || cpu_op_cur >= cpu_op_nr)
		printk(KERN_ERR "%s: Bad cpu_op_cur!\n", __func__);

	cpufreq_new = cpu_op_tbl[cpu_op_cur + 1].cpu_rate;

	printk(KERN_INFO "%s: cpufreq down from %d to %d\n",
		__func__, cpufreq, cpufreq_new);
	cpufreq_new /= 1000;

	strcpy(cpu_sys_file, CPUx);
	sprintf(cpu_number, "%d%s", 0, "/cpufreq/scaling_setspeed");
	sprintf(cpufreq_new_str, "%d", cpufreq_new);
	strcat(cpu_sys_file, cpu_number);

	fd = sys_open((const char __user __force *)cpu_sys_file,
	  O_RDWR, 0700);
	if (fd >= 0) {
		sys_write(fd, cpufreq_new_str, strlen(cpufreq_new_str));
		sys_close(fd);
	}
	cpufreq_change_count--;

	return 1;
}


int anatop_thermal_cpu_hotplug(bool cpu_on)
{
	unsigned int ret, i;
	char online;
	char cpu_number[9];
	int fd;

	ret = -EINVAL;
	if (cpu_on) {
		for (i = 1; i < 4; i++) {
			strcpy(cpu_sys_file, CPUx);
			sprintf(cpu_number, "%d%s", i, "/online");
			strcat(cpu_sys_file, cpu_number);
			fd = sys_open((const char __user __force *)cpu_sys_file,
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
			strcpy(cpu_sys_file, CPUx);
			sprintf(cpu_number, "%d%s", i, "/online");
			strcat(cpu_sys_file, cpu_number);
			fd = sys_open((const char __user __force *)cpu_sys_file,
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
	if (cooling_cpuhotplug) {
		if (!state) {
			for (i = 1; i < 4; i++) {
				if (cpu_mask && (0x1 << i)) {
					anatop_thermal_cpu_hotplug(true);
					temperature_cooling = 0;
					break;
				}
			}
		} else
			printk(KERN_DEBUG "Temperature is about to reach hot point!\n");
	} else {
		if (!state) {
			if (cpufreq_change_count < 0)
				anatop_thermal_cpufreq_up();
			else if (cpufreq_change_count > 0)
				anatop_thermal_cpufreq_down();
			temperature_cooling = 0;
		} else
			printk(KERN_DEBUG "Temperature is about to reach hot point!\n");
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

