/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/jiffies.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#if defined(CONFIG_CPU_FREQ)
#include <linux/cpufreq.h>
#endif
#include <linux/io.h>
#include <asm/cpu.h>

#include <mach/clock.h>
#include <mach/hardware.h>
#include <mach/system.h>
#include "regs-anadig.h"
#include "crm_regs.h"
struct regulator *cpu_regulator;
struct regulator *soc_regulator;
struct regulator *pu_regulator;
char *gp_reg_id;
char *soc_reg_id;
char *pu_reg_id;
static struct clk *cpu_clk;
static int cpu_op_nr;
static struct cpu_op *cpu_op_tbl;
extern struct cpu_op *(*get_cpu_op)(int *op);

extern unsigned long loops_per_jiffy;
extern u32 enable_ldo_mode;
int external_pureg;

static inline unsigned long mx6_cpu_jiffies(unsigned long old, u_int div,
					      u_int mult)
{
#if BITS_PER_LONG == 32

	u64 result = ((u64) old) * ((u64) mult);
	do_div(result, div);
	return (unsigned long) result;

#elif BITS_PER_LONG == 64

	unsigned long result = old * ((u64) mult);
	result /= div;
	return result;

#endif
}

void mx6_cpu_regulator_init(void)
{
	int cpu;
	u32 curr_cpu = 0;
	unsigned int reg;
#ifndef CONFIG_SMP
	unsigned long old_loops_per_jiffy;
#endif
	void __iomem *gpc_base = IO_ADDRESS(GPC_BASE_ADDR);
	external_pureg = 0;
	/*If internal ldo actived, use internal cpu_* regulator to replace the
	*regulator ids from board file. If internal ldo bypassed, use the
	*regulator ids which defined in board file and source from extern pmic
	*power rails.
	*If you want to use ldo bypass,you should do:
	*1.set enable_ldo_mode=LDO_MODE_BYPASSED in your board file by default
	*   or set in commandline from u-boot
	*2.set your extern pmic regulator name in your board file.
	*/
	if (enable_ldo_mode != LDO_MODE_BYPASSED) {
		gp_reg_id = "cpu_vddgp";
		soc_reg_id = "cpu_vddsoc";
		pu_reg_id = "cpu_vddgpu";
	}
	printk(KERN_INFO "cpu regulator mode:%s\n", (enable_ldo_mode ==
		LDO_MODE_BYPASSED) ? "ldo_bypass" : "ldo_enable");
	cpu_regulator = regulator_get(NULL, gp_reg_id);
	if (IS_ERR(cpu_regulator))
		printk(KERN_ERR "%s: failed to get cpu regulator\n", __func__);
	else {
		cpu_clk = clk_get(NULL, "cpu_clk");
		if (IS_ERR(cpu_clk)) {
			printk(KERN_ERR "%s: failed to get cpu clock\n",
			       __func__);
		} else {
			curr_cpu = clk_get_rate(cpu_clk);
			cpu_op_tbl = get_cpu_op(&cpu_op_nr);

			soc_regulator = regulator_get(NULL, soc_reg_id);
			if (IS_ERR(soc_regulator))
				printk(KERN_ERR "%s: failed to get soc regulator\n",
					__func__);
			else
				/* set soc to highest setpoint voltage. */
				regulator_set_voltage(soc_regulator,
					      cpu_op_tbl[0].soc_voltage,
					      cpu_op_tbl[0].soc_voltage);

			pu_regulator = regulator_get(NULL, pu_reg_id);
			if (IS_ERR(pu_regulator))
				printk(KERN_ERR "%s: failed to get pu regulator\n",
					__func__);
			else
				/* set pu to higheset setpoint voltage. */
				regulator_set_voltage(pu_regulator,
					      cpu_op_tbl[0].pu_voltage,
					      cpu_op_tbl[0].pu_voltage);
			/* set the core to higheset setpoint voltage. */
			regulator_set_voltage(cpu_regulator,
					      cpu_op_tbl[0].cpu_voltage,
					      cpu_op_tbl[0].cpu_voltage);
			if (enable_ldo_mode == LDO_MODE_BYPASSED) {
				/* digital bypass VDDPU/VDDSOC/VDDARM */
				reg = __raw_readl(ANADIG_REG_CORE);
				reg &= ~BM_ANADIG_REG_CORE_REG0_TRG;
				reg |= BF_ANADIG_REG_CORE_REG0_TRG(0x1f);
				reg &= ~BM_ANADIG_REG_CORE_REG1_TRG;
				reg |= BF_ANADIG_REG_CORE_REG1_TRG(0x1f);
				reg &= ~BM_ANADIG_REG_CORE_REG2_TRG;
				reg |= BF_ANADIG_REG_CORE_REG2_TRG(0x1f);
				__raw_writel(reg, ANADIG_REG_CORE);
				/* mask the ANATOP brown out irq in the GPC. */
				reg = __raw_readl(gpc_base + 0x14);
				reg |= 0x80000000;
				__raw_writel(reg, gpc_base + 0x14);
			}

			clk_set_rate(cpu_clk, cpu_op_tbl[0].cpu_rate);

			/* fix loops-per-jiffy */
#ifdef CONFIG_SMP
			for_each_online_cpu(cpu)
				per_cpu(cpu_data, cpu).loops_per_jiffy =
				mx6_cpu_jiffies(
					per_cpu(cpu_data, cpu).loops_per_jiffy,
					curr_cpu / 1000,
					clk_get_rate(cpu_clk) / 1000);
#else
			old_loops_per_jiffy = loops_per_jiffy;

			loops_per_jiffy =
				mx6_cpu_jiffies(old_loops_per_jiffy,
						curr_cpu/1000,
						clk_get_rate(cpu_clk) / 1000);
#endif
#if defined(CONFIG_CPU_FREQ)
			/* Fix CPU frequency for CPUFREQ. */
			for (cpu = 0; cpu < num_online_cpus(); cpu++)
				cpufreq_get(cpu);
#endif
		}
	}
	/*
	 * if use ldo bypass and VDDPU_IN is single supplied
	 * by external pmic, it means VDDPU_IN can be turned off
	 * if GPU/VPU driver not running.In this case we should set
	 * external_pureg which can be used in pu_enable/pu_disable of
	 * arch/arm/mach-mx6/mx6_anatop_regulator.c to
	 * enable or disable external VDDPU regulator from pmic. But for FSL
	 * reference boards, VDDSOC_IN connect with VDDPU_IN, so we didn't set
	 * pu_reg_id to the external pmic regulator supply name in the board
	 * file. In this case external_pureg should be 0 and can't turn off
	 * extern pmic regulator, but can turn off VDDPU by internal anatop
	 * power gate.
	 *
	 * if enable internal ldo , external_pureg will be 0, and
	 * VDDPU can be turned off by internal anatop anatop power gate.
	 *
	 */
	if (!IS_ERR(pu_regulator) && strcmp(pu_reg_id, "cpu_vddgpu"))
		external_pureg = 1;
}

