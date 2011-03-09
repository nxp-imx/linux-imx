/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <mach/mxc_dvfs.h>

/* working point(wp): 0 - 800MHz; 1 - 400MHz, 2 - 160MHz; */
static struct cpu_op mx50_cpu_op[] = {
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 800000000,
	 .pdf = 0,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 0,
	 .cpu_voltage = 1050000,},
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 400000000,
	 .cpu_podf = 1,
	 .cpu_voltage = 1050000,},
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 160000000,
	 .cpu_podf = 4,
	 .cpu_voltage = 850000,},
};

struct cpu_op *mx50_get_cpu_op(int *op)
{
	*op = ARRAY_SIZE(mx50_cpu_op);
	return mx50_cpu_op;
}
