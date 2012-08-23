/*
 * Copyright (C) 2010-2012 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include "cpu_op-mx6.h"

extern struct cpu_op *(*get_cpu_op)(int *op);
extern struct dvfs_op *(*get_dvfs_core_op)(int *wp);
extern void (*set_num_cpu_op)(int num);
extern u32 arm_max_freq;
static int num_cpu_op;

/* working point(wp): 0 - 1.2GHz; 1 - 792MHz, 2 - 498MHz 3 - 396MHz */
static struct cpu_op mx6_cpu_op_1_2G[] = {
	{
	 .pll_rate = 1200000000,
	 .cpu_rate = 1200000000,
	 .cpu_podf = 0,
	 .pu_voltage = 1250000,
	 .soc_voltage = 1250000,
	 .cpu_voltage = 1275000,},
	{
	 .pll_rate = 792000000,
	 .cpu_rate = 792000000,
	 .cpu_podf = 0,
#ifdef CONFIG_MX6_VPU_352M
	/*VPU 352Mhz need voltage 1.25V*/
	 .pu_voltage = 1250000,
	 .soc_voltage = 1250000,
#else
	 .pu_voltage = 1150000,
	 .soc_voltage = 1150000,
#endif
	 .cpu_voltage = 1100000,},
#ifdef CONFIG_MX6_VPU_352M
	/*pll2_pfd_400M will be fix on 352M,to avoid modify other code
	which assume ARM clock sourcing from pll2_pfd_400M, change cpu
	freq from 396M to 352M.*/
	 {
	  .pll_rate = 352000000,
	  .cpu_rate = 352000000,
	  .cpu_podf = 0,
	  .pu_voltage = 1250000,
	  .soc_voltage = 1250000,
	  .cpu_voltage = 925000,},
#else
	 {
	  .pll_rate = 396000000,
	  .cpu_rate = 396000000,
	  .cpu_podf = 0,
	  .pu_voltage = 1150000,
	  .soc_voltage = 1150000,
	  .cpu_voltage = 925000,},
#endif
};

/* working point(wp): 0 - 1GHz; 1 - 792MHz, 2 - 498MHz 3 - 396MHz */
static struct cpu_op mx6_cpu_op_1G[] = {
	{
	 .pll_rate = 996000000,
	 .cpu_rate = 996000000,
	 .cpu_podf = 0,
	 .pu_voltage = 1200000,
	 .soc_voltage = 1200000,
	 .cpu_voltage = 1225000,},
	{
	 .pll_rate = 792000000,
	 .cpu_rate = 792000000,
	 .cpu_podf = 0,
#ifdef CONFIG_MX6_VPU_352M
	/*VPU 352Mhz need voltage 1.25V*/
	 .pu_voltage = 1250000,
	 .soc_voltage = 1250000,
#else
	 .pu_voltage = 1150000,
	 .soc_voltage = 1150000,
#endif
	 .cpu_voltage = 1100000,},
#ifdef CONFIG_MX6_VPU_352M
	/*pll2_pfd_400M will be fix on 352M,to avoid modify other code
	which assume ARM clock sourcing from pll2_pfd_400M, change cpu
	freq from 396M to 352M.*/
	 {
	  .pll_rate = 352000000,
	  .cpu_rate = 352000000,
	  .cpu_podf = 0,
	  .pu_voltage = 1250000,
	  .soc_voltage = 1250000,
	  .cpu_voltage = 925000,},
#else
	 {
	  .pll_rate = 396000000,
	  .cpu_rate = 396000000,
	  .cpu_podf = 0,
	  .pu_voltage = 1150000,
	  .soc_voltage = 1150000,
	  .cpu_voltage = 925000,},
#endif
};

static struct cpu_op mx6_cpu_op[] = {
	{
	 .pll_rate = 792000000,
	 .cpu_rate = 792000000,
	 .cpu_podf = 0,
#ifdef CONFIG_MX6_VPU_352M
	/*VPU 352Mhz need voltage 1.25V*/
	 .pu_voltage = 1250000,
	 .soc_voltage = 1250000,
#else
	 .pu_voltage = 1150000,
	 .soc_voltage = 1150000,
#endif
	 .cpu_voltage = 1100000,},
#ifdef CONFIG_MX6_VPU_352M
	/*pll2_pfd_400M will be fix on 352M,to avoid modify other code
	which assume ARM clock sourcing from pll2_pfd_400M, change cpu
	freq from 396M to 352M.*/
	 {
	  .pll_rate = 352000000,
	  .cpu_rate = 352000000,
	  .cpu_podf = 0,
	  .pu_voltage = 1250000,
	  .soc_voltage = 1250000,
	  .cpu_voltage = 925000,},
#else
	 {
	  .pll_rate = 396000000,
	  .cpu_rate = 396000000,
	  .cpu_podf = 0,
	  .pu_voltage = 1150000,
	  .soc_voltage = 1150000,
	  .cpu_voltage = 925000,},
#endif
};

/* working point(wp): 0 - 1.2GHz; 1 - 800MHz, 2 - 400MHz, 3  - 200MHz */
static struct cpu_op mx6dl_cpu_op_1_2G[] = {
	{
	 .pll_rate = 1200000000,
	 .cpu_rate = 1200000000,
	 .cpu_podf = 0,
	 .pu_voltage = 1250000,
	 .soc_voltage = 1250000,
	 .cpu_voltage = 1275000,},
	{
	 .pll_rate = 792000000,
	 .cpu_rate = 792000000,
	 .cpu_podf = 0,
	 .pu_voltage = 1150000,
	 .soc_voltage = 1150000,
	 .cpu_voltage = 1100000,},
	 {
	  .pll_rate = 396000000,
	  .cpu_rate = 396000000,
	  .cpu_podf = 0,
	  .pu_voltage = 1150000,
	  .soc_voltage = 1150000,
	  .cpu_voltage = 1025000,},
	 {
	  .pll_rate = 396000000,
	  .cpu_rate = 198000000,
	  .cpu_podf = 1,
	  .pu_voltage = 1150000,
	  .soc_voltage = 1150000,
	  .cpu_voltage = 1025000,},
};
/* working point(wp): 0 - 1GHz; 1 - 800MHz, 2 - 400MHz, 3  - 200MHz */
static struct cpu_op mx6dl_cpu_op_1G[] = {
	{
	 .pll_rate = 996000000,
	 .cpu_rate = 996000000,
	 .cpu_podf = 0,
	 .pu_voltage = 1200000,
	 .soc_voltage = 1200000,
	 .cpu_voltage = 1225000,},
	{
	 .pll_rate = 792000000,
	 .cpu_rate = 792000000,
	 .cpu_podf = 0,
	 .pu_voltage = 1150000,
	 .soc_voltage = 1150000,
	 .cpu_voltage = 1100000,},
	{
	 .pll_rate = 396000000,
	 .cpu_rate = 396000000,
	 .cpu_podf = 0,
	 .pu_voltage = 1150000,
	 .soc_voltage = 1150000,
	 .cpu_voltage = 1025000,},
	 {
	  .pll_rate = 396000000,
	  .cpu_rate = 198000000,
	  .cpu_podf = 1,
	  .pu_voltage = 1150000,
	  .soc_voltage = 1150000,
	  .cpu_voltage = 1025000,},
};
static struct cpu_op mx6dl_cpu_op[] = {
	{
	 .pll_rate = 792000000,
	 .cpu_rate = 792000000,
	 .cpu_podf = 0,
	 .pu_voltage = 1150000,
	 .soc_voltage = 1150000,
	 .cpu_voltage = 1100000,},
	 {
	  .pll_rate = 396000000,
	  .cpu_rate = 396000000,
	  .cpu_podf = 0,
	  .pu_voltage = 1150000,
	  .soc_voltage = 1150000,
	  .cpu_voltage = 1025000,},
	{
	 .pll_rate = 396000000,
	 .cpu_rate = 198000000,
	 .cpu_podf = 1,
	 .pu_voltage = 1150000,
	 .soc_voltage = 1150000,
	 .cpu_voltage = 1025000,},
};

static struct dvfs_op dvfs_core_setpoint_1_2G[] = {
	{33, 14, 33, 10, 128, 0x10},     /* 1.2GHz*/
	{30, 12, 33, 100, 200, 0x10},   /* 800MHz */
	{28, 12, 33, 100, 200, 0x10},   /* 672MHz */
	{26, 8, 33, 100, 200, 0x10},   /* 400MHz */
	{20, 0, 33, 20, 10, 0x10} };   /* 200MHz*/

static struct dvfs_op dvfs_core_setpoint_1G[] = {
	{33, 14, 33, 10, 128, 0x10}, /* 1GHz*/
	{30, 12, 33, 100, 200, 0x10},   /* 800MHz */
	{28, 12, 33, 100, 200, 0x10},   /* 672MHz */
	{26, 8, 33, 100, 200, 0x10},   /* 400MHz */
	{20, 0, 33, 20, 10, 0x10} };   /* 200MHz*/

static struct dvfs_op dvfs_core_setpoint[] = {
	{33, 14, 33, 10, 128, 0x08},   /* 800MHz */
	{26, 8, 33, 100, 200, 0x08},   /* 400MHz */
	{20, 0, 33, 100, 10, 0x08} };   /* 200MHz*/

static struct dvfs_op *mx6_get_dvfs_core_table(int *wp)
{
	if (arm_max_freq == CPU_AT_1_2GHz) {
		*wp = ARRAY_SIZE(dvfs_core_setpoint_1_2G);
		return dvfs_core_setpoint_1_2G;
	} else if (arm_max_freq == CPU_AT_1GHz) {
		*wp = ARRAY_SIZE(dvfs_core_setpoint_1G);
		return dvfs_core_setpoint_1G;
	} else {
		*wp = ARRAY_SIZE(dvfs_core_setpoint);
		return dvfs_core_setpoint;
	}
}

struct cpu_op *mx6_get_cpu_op(int *op)
{
	if (cpu_is_mx6dl()) {
		if (arm_max_freq == CPU_AT_1_2GHz) {
			*op =  num_cpu_op = ARRAY_SIZE(mx6dl_cpu_op_1_2G);
			return mx6dl_cpu_op_1_2G;
		} else if (arm_max_freq == CPU_AT_1GHz) {
			*op =  num_cpu_op = ARRAY_SIZE(mx6dl_cpu_op_1G);
			return mx6dl_cpu_op_1G;
		} else {
			*op =  num_cpu_op = ARRAY_SIZE(mx6dl_cpu_op);
			return mx6dl_cpu_op;
		}
	} else {
		if (arm_max_freq == CPU_AT_1_2GHz) {
			*op =  num_cpu_op = ARRAY_SIZE(mx6_cpu_op_1_2G);
			return mx6_cpu_op_1_2G;
		} else if (arm_max_freq == CPU_AT_1GHz) {
			*op =  num_cpu_op = ARRAY_SIZE(mx6_cpu_op_1G);
			return mx6_cpu_op_1G;
		} else {
			*op =  num_cpu_op = ARRAY_SIZE(mx6_cpu_op);
			return mx6_cpu_op;
		}
	}
}

void mx6_set_num_cpu_op(int num)
{
	num_cpu_op = num;
	return;
}

void mx6_cpu_op_init(void)
{
	get_cpu_op = mx6_get_cpu_op;
	set_num_cpu_op = mx6_set_num_cpu_op;

	get_dvfs_core_op = mx6_get_dvfs_core_table;
}

