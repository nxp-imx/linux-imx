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
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>

#include <asm/cpu.h>

#include <mach/clock.h>
#include <mach/hardware.h>

struct regulator *cpu_regulator;
char *gp_reg_id;
static struct clk *cpu_clk;
static int cpu_op_nr;
static struct cpu_op *cpu_op_tbl;
extern struct cpu_op *(*get_cpu_op)(int *op);


void mx6_cpu_regulator_init(void)
{
	cpu_regulator = regulator_get(NULL, gp_reg_id);
	if (IS_ERR(cpu_regulator))
		printk(KERN_ERR "%s: failed to get cpu regulator\n", __func__);
	else {
		cpu_clk = clk_get(NULL, "cpu_clk");
		if (IS_ERR(cpu_clk)) {
			printk(KERN_ERR "%s: failed to get cpu clock\n",
			       __func__);
		} else {
			cpu_op_tbl = get_cpu_op(&cpu_op_nr);
			/* Set the core to max frequency requested. */
			regulator_set_voltage(cpu_regulator,
					      cpu_op_tbl[0].cpu_voltage,
					      cpu_op_tbl[0].cpu_voltage);
			clk_set_rate(cpu_clk, cpu_op_tbl[0].pll_rate);
		}
	}
}

