/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>

extern struct cpu_op *(*get_cpu_op)(int *op);
extern void (*set_num_cpu_op)(int num);
extern struct dvfs_op *(*get_dvfs_core_op)(int *wp);

static int num_cpu_op;

/* working point(wp): 0 - 800MHz; 1 - 166.25MHz; */
static struct cpu_op mx51_cpu_op[] = {
	{
	 .pll_rate = 1000000000,
	 .cpu_rate = 1000000000,
	 .pdf = 0,
	 .mfi = 10,
	 .mfd = 11,
	 .mfn = 5,
	 .cpu_podf = 0,
	 .cpu_voltage = 1175000,},
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 800000000,
	 .pdf = 0,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 0,
	 .cpu_voltage = 1100000,
	},
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 400000000,
	 .pdf = 0,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 1,
	 .cpu_voltage = 950000,
	 },
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 166250000,
	 .cpu_podf = 4,
	 .cpu_voltage = 850000,
	},
};

static struct dvfs_op dvfs_core_setpoint[] = {
	{33, 13, 33, 10, 10, 0x08}, /* 800MHz*/
	{28, 8, 33, 10, 10, 0x08},   /* 400MHz */
	{20, 0, 33, 20, 10, 0x08},   /* 160MHz*/
	{28, 8, 33, 20, 30, 0x08},   /*160MHz, AHB 133MHz, LPAPM mode*/
	{29, 0, 33, 20, 10, 0x08},}; /* 160MHz, AHB 24MHz */

struct cpu_op *mx51_get_cpu_op(int *op)
{
	*op = num_cpu_op;
	return mx51_cpu_op;
}

void mx51_set_num_cpu_op(int num)
{
	num_cpu_op = num;
	return;
}


static struct dvfs_op *mx51_get_dvfs_core_table(int *wp)
{
	*wp = ARRAY_SIZE(dvfs_core_setpoint);
	return dvfs_core_setpoint;
}

void mx51_cpu_op_init(void)
{
	get_cpu_op = mx51_get_cpu_op;
	set_num_cpu_op = mx51_set_num_cpu_op;
	num_cpu_op = ARRAY_SIZE(mx51_cpu_op);
	get_dvfs_core_op = mx51_get_dvfs_core_table;
}
