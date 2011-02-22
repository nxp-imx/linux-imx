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
#include <mach/hardware.h>
#include <linux/kernel.h>

extern struct cpu_op *(*get_cpu_op)(int *op);
extern void (*set_num_cpu_op)(int num);
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
	 .cpu_rate = 166250000,
	 .cpu_podf = 4,
	 .cpu_voltage = 850000,
	},
};

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

void mx51_cpu_op_init()
{
	get_cpu_op = mx51_get_cpu_op;
	set_num_cpu_op = mx51_set_num_cpu_op;

	num_cpu_op = ARRAY_SIZE(mx51_cpu_op);
}
