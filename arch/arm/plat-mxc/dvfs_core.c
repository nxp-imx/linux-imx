/*
 * Copyright 2008-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file dvfs_core.c
 *
 * @brief A simplied driver for the Freescale Semiconductor MXC DVFS module.
 *
 * Upon initialization, the DVFS driver initializes the DVFS hardware
 * sets up driver nodes attaches to the DVFS interrupt and initializes internal
 * data structures. When the DVFS interrupt occurs the driver checks the cause
 * of the interrupt (lower frequency, increase frequency or emergency) and
 * changes the CPU voltage according to translation table that is loaded into
 * the driver.
 *
 * @ingroup PM
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#if defined(CONFIG_CPU_FREQ)
#include <linux/cpufreq.h>
#endif
#include <asm/cpu.h>

#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>

#define MXC_DVFSTHRS_UPTHR_MASK               0x0FC00000
#define MXC_DVFSTHRS_UPTHR_OFFSET             22
#define MXC_DVFSTHRS_DNTHR_MASK               0x003F0000
#define MXC_DVFSTHRS_DNTHR_OFFSET             16
#define MXC_DVFSTHRS_PNCTHR_MASK              0x0000003F
#define MXC_DVFSTHRS_PNCTHR_OFFSET            0

#define MXC_DVFSCOUN_DNCNT_MASK               0x00FF0000
#define MXC_DVFSCOUN_DNCNT_OFFSET             16
#define MXC_DVFSCOUN_UPCNT_MASK              0x000000FF
#define MXC_DVFSCOUN_UPCNT_OFFSET            0

#define MXC_DVFSEMAC_EMAC_MASK               0x000001FF
#define MXC_DVFSEMAC_EMAC_OFFSET             0

#define MXC_DVFSCNTR_DVFEV                   0x10000000
#define MXC_DVFSCNTR_LBMI                    0x08000000
#define MXC_DVFSCNTR_LBFL                    0x06000000
#define MXC_DVFSCNTR_DVFIS                   0x01000000
#define MXC_DVFSCNTR_FSVAIM                  0x00400000
#define MXC_DVFSCNTR_FSVAI_MASK              0x00300000
#define MXC_DVFSCNTR_FSVAI_OFFSET            20
#define MXC_DVFSCNTR_WFIM                    0x00080000
#define MXC_DVFSCNTR_WFIM_OFFSET             19
#define MXC_DVFSCNTR_MAXF_MASK               0x00040000
#define MXC_DVFSCNTR_MAXF_OFFSET             18
#define MXC_DVFSCNTR_MINF_MASK               0x00020000
#define MXC_DVFSCNTR_MINF_OFFSET             17
#define MXC_DVFSCNTR_LTBRSR_MASK             0x00000018
#define MXC_DVFSCNTR_LTBRSR_OFFSET           3
#define MXC_DVFSCNTR_DVFEN                   0x00000001

#define CCM_CDCR_SW_DVFS_EN			0x20
#define CCM_CDCR_ARM_FREQ_SHIFT_DIVIDER		0x4
#define CCM_CDHIPR_ARM_PODF_BUSY		0x10000

int cpufreq_trig_needed;
int dvfs_core_is_active;
static struct mxc_dvfs_platform_data *dvfs_data;
static struct device *dvfs_dev;
static struct cpu_op *cpu_op_tbl;
static int dvfs_core_resume;
static int curr_op;
static int old_op;
static int dvfs_core_op;
static int dvfs_config_setpoint;

static int maxf;
static int minf;

extern void setup_pll(void);
extern struct regulator *cpu_regulator;

struct timeval core_prev_intr;

void dump_dvfs_core_regs(void);
void stop_dvfs(void);
struct dvfs_op *(*get_dvfs_core_op)(int *op);

static struct delayed_work dvfs_core_handler;

/*
 * Clock structures
 */
static struct clk *pll1_sw_clk;
static struct clk *cpu_clk;
static struct clk *dvfs_clk;

static int cpu_op_nr;
extern struct cpu_op *(*get_cpu_op)(int *op);

static inline unsigned long dvfs_cpu_jiffies(unsigned long old, u_int div, u_int mult)
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

enum {
	FSVAI_FREQ_NOCHANGE = 0x0,
	FSVAI_FREQ_INCREASE,
	FSVAI_FREQ_DECREASE,
	FSVAI_FREQ_EMERG,
};

/*
 * Load tracking buffer source: 1 for ld_add; 0 for pre_ld_add; 2 for after EMA
 */
#define DVFS_LTBRSR		(2 << MXC_DVFSCNTR_LTBRSR_OFFSET)

static struct dvfs_op *dvfs_core_setpoint;
extern int low_bus_freq_mode;
extern int high_bus_freq_mode;
extern int set_low_bus_freq(void);
extern int set_high_bus_freq(int high_bus_speed);
extern int low_freq_bus_used(void);
extern struct mutex bus_freq_mutex;

DEFINE_SPINLOCK(mxc_dvfs_core_lock);

static void dvfs_load_config(int set_point)
{
	u32 reg;
	reg = 0;

	reg |= dvfs_core_setpoint[set_point].upthr << MXC_DVFSTHRS_UPTHR_OFFSET;
	reg |= dvfs_core_setpoint[set_point].downthr <<
	    MXC_DVFSTHRS_DNTHR_OFFSET;
	reg |= dvfs_core_setpoint[set_point].panicthr;
	__raw_writel(reg, dvfs_data->membase + MXC_DVFSCORE_THRS);

	reg = 0;
	reg |= dvfs_core_setpoint[set_point].downcnt <<
	    MXC_DVFSCOUN_DNCNT_OFFSET;
	reg |= dvfs_core_setpoint[set_point].upcnt << MXC_DVFSCOUN_UPCNT_OFFSET;
	__raw_writel(reg, dvfs_data->membase + MXC_DVFSCORE_COUN);

	/* Set EMAC value */
	reg = __raw_readl(dvfs_data->membase + MXC_DVFSCORE_EMAC);
	reg &= ~MXC_DVFSEMAC_EMAC_MASK;
	reg |= dvfs_core_setpoint[set_point].emac << MXC_DVFSEMAC_EMAC_OFFSET;
	__raw_writel(reg, dvfs_data->membase + MXC_DVFSCORE_EMAC);

	dvfs_config_setpoint = set_point;
}

static int mx5_set_cpu_freq(int op)
{
	int arm_podf;
	int podf;
	int vinc = 0;
	int ret = 0;
	int org_cpu_rate;
	unsigned long rate = 0;
	int gp_volt = 0;
	u32 reg;
	u32 reg1;
	u32 en_sw_dvfs = 0;
	unsigned long flags;

	if (cpu_op_tbl[op].pll_rate != cpu_op_tbl[old_op].pll_rate) {
		org_cpu_rate = clk_get_rate(cpu_clk);
		rate = cpu_op_tbl[op].cpu_rate;

		if (org_cpu_rate == rate)
			return ret;

		gp_volt = cpu_op_tbl[op].cpu_voltage;
		if (gp_volt == 0)
			return ret;

		/*Set the voltage for the GP domain. */
		if (rate > org_cpu_rate) {
			ret = regulator_set_voltage(cpu_regulator, gp_volt,
						    gp_volt);
			if (ret < 0) {
				printk(KERN_DEBUG "COULD NOT SET GP VOLTAGE\n");
				return ret;
			}
			udelay(dvfs_data->delay_time);
		}
		spin_lock_irqsave(&mxc_dvfs_core_lock, flags);
		/* PLL_RELOCK, set ARM_FREQ_SHIFT_DIVIDER */
		reg = __raw_readl(ccm_base + dvfs_data->ccm_cdcr_offset);
		/* Check if software_dvfs_en bit set */
		if ((reg & CCM_CDCR_SW_DVFS_EN) != 0)
			en_sw_dvfs = CCM_CDCR_SW_DVFS_EN;
		else
			en_sw_dvfs = 0x0;
		reg &= ~(CCM_CDCR_SW_DVFS_EN);
		reg &= 0xFFFFFFFB;
		__raw_writel(reg, ccm_base + dvfs_data->ccm_cdcr_offset);

		setup_pll();
		/* START the GPC main control FSM */
		/* set VINC */
		reg = __raw_readl(gpc_base + dvfs_data->gpc_vcr_offset);
		reg &= ~(MXC_GPCVCR_VINC_MASK | MXC_GPCVCR_VCNTU_MASK |
			 MXC_GPCVCR_VCNT_MASK);

		if (rate > org_cpu_rate)
			reg |= 1 << MXC_GPCVCR_VINC_OFFSET;

		reg |= (1 << MXC_GPCVCR_VCNTU_OFFSET) |
		       (1 << MXC_GPCVCR_VCNT_OFFSET);
		__raw_writel(reg, gpc_base + dvfs_data->gpc_vcr_offset);

		reg = __raw_readl(gpc_base + dvfs_data->gpc_cntr_offset);
		reg &= ~(MXC_GPCCNTR_ADU_MASK | MXC_GPCCNTR_FUPD_MASK);
		reg |= MXC_GPCCNTR_FUPD;
		reg |= MXC_GPCCNTR_ADU;
		__raw_writel(reg, gpc_base + dvfs_data->gpc_cntr_offset);

		reg |= MXC_GPCCNTR_STRT;
		__raw_writel(reg, gpc_base + dvfs_data->gpc_cntr_offset);
		while (__raw_readl(gpc_base + dvfs_data->gpc_cntr_offset)
				& 0x4000)
			udelay(10);
		spin_unlock_irqrestore(&mxc_dvfs_core_lock, flags);

		if (rate < org_cpu_rate) {
			ret = regulator_set_voltage(cpu_regulator, gp_volt,
						    gp_volt);
			if (ret < 0) {
				printk(KERN_DEBUG
				       "COULD NOT SET GP VOLTAGE!!!!\n");
				return ret;
			}
			udelay(dvfs_data->delay_time);
		}
		/* set software_dvfs_en bit back to original setting*/
		reg = __raw_readl(ccm_base + dvfs_data->ccm_cdcr_offset);
		reg &= ~(CCM_CDCR_SW_DVFS_EN);
		reg |= en_sw_dvfs;
		clk_set_rate(cpu_clk, rate);
	} else {
		podf = cpu_op_tbl[op].cpu_podf;
		gp_volt = cpu_op_tbl[op].cpu_voltage;

		/* Change arm_podf only */
		/* set ARM_FREQ_SHIFT_DIVIDER */
		reg = __raw_readl(ccm_base + dvfs_data->ccm_cdcr_offset);

		/* Check if software_dvfs_en bit set */
		if ((reg & CCM_CDCR_SW_DVFS_EN) != 0)
			en_sw_dvfs = CCM_CDCR_SW_DVFS_EN;
		else
			en_sw_dvfs = 0x0;

		reg &= ~(CCM_CDCR_SW_DVFS_EN | CCM_CDCR_ARM_FREQ_SHIFT_DIVIDER);
		reg |= CCM_CDCR_ARM_FREQ_SHIFT_DIVIDER;
		__raw_writel(reg, ccm_base + dvfs_data->ccm_cdcr_offset);

		/* Get ARM_PODF */
		reg = __raw_readl(ccm_base + dvfs_data->ccm_cacrr_offset);
		arm_podf = reg & 0x07;
		if (podf == arm_podf) {
			printk(KERN_DEBUG
			       "No need to change freq and voltage!!!!\n");
			return 0;
		}
		/* Check if FSVAI indicate freq up */
		if (podf < arm_podf) {
			ret = regulator_set_voltage(cpu_regulator, gp_volt,
						    gp_volt);
			if (ret < 0) {
				printk(KERN_DEBUG
				       "COULD NOT SET GP VOLTAGE!!!!\n");
				return 0;
			}
			udelay(dvfs_data->delay_time);
			vinc = 1;
		} else {
			vinc = 0;
		}

		arm_podf = podf;
		/* Set ARM_PODF */
		reg &= 0xFFFFFFF8;
		reg |= arm_podf;
		spin_lock_irqsave(&mxc_dvfs_core_lock, flags);

		reg1 = __raw_readl(ccm_base + dvfs_data->ccm_cdhipr_offset);
		while (1) {
			if ((reg1 & CCM_CDHIPR_ARM_PODF_BUSY) == 0) {
				__raw_writel(reg,
					ccm_base + dvfs_data->ccm_cacrr_offset);
				break;
			} else {
				reg1 = __raw_readl(
				ccm_base + dvfs_data->ccm_cdhipr_offset);
				printk(KERN_DEBUG "ARM_PODF still in busy!!!!\n");
			}
		}
		/* set VINC */
		reg = __raw_readl(gpc_base + dvfs_data->gpc_vcr_offset);
		reg &=
		    ~(MXC_GPCVCR_VINC_MASK | MXC_GPCVCR_VCNTU_MASK |
		      MXC_GPCVCR_VCNT_MASK);
		reg |= (1 << MXC_GPCVCR_VCNTU_OFFSET) |
		    (100 << MXC_GPCVCR_VCNT_OFFSET) |
		    (vinc << MXC_GPCVCR_VINC_OFFSET);
		__raw_writel(reg, gpc_base + dvfs_data->gpc_vcr_offset);

		reg = __raw_readl(gpc_base + dvfs_data->gpc_cntr_offset);
		reg &= (~(MXC_GPCCNTR_ADU | MXC_GPCCNTR_FUPD
				| MXC_GPCCNTR_STRT));
		__raw_writel(reg, gpc_base + dvfs_data->gpc_cntr_offset);
		reg = __raw_readl(gpc_base + dvfs_data->gpc_cntr_offset);
		reg |= MXC_GPCCNTR_ADU | MXC_GPCCNTR_FUPD;
		__raw_writel(reg, gpc_base + dvfs_data->gpc_cntr_offset);
		reg = __raw_readl(gpc_base + dvfs_data->gpc_cntr_offset);
		reg |= MXC_GPCCNTR_STRT;
		__raw_writel(reg, gpc_base + dvfs_data->gpc_cntr_offset);

		/* Wait for arm podf Enable */
		while ((__raw_readl(gpc_base + dvfs_data->gpc_cntr_offset) &
			MXC_GPCCNTR_STRT) == MXC_GPCCNTR_STRT) {
			printk(KERN_DEBUG "Waiting arm_podf enabled!\n");
			udelay(10);
		}
		spin_unlock_irqrestore(&mxc_dvfs_core_lock, flags);

		if (vinc == 0) {
			ret = regulator_set_voltage(cpu_regulator, gp_volt,
						    gp_volt);
			if (ret < 0) {
				printk(KERN_DEBUG
				       "COULD NOT SET GP VOLTAGE\n!!!");
				return ret;
			}
			udelay(dvfs_data->delay_time);
		}

		/* Clear the ARM_FREQ_SHIFT_DIVIDER and */
		/* set software_dvfs_en bit back to original setting*/
		reg = __raw_readl(ccm_base + dvfs_data->ccm_cdcr_offset);
		reg &= ~(CCM_CDCR_SW_DVFS_EN | CCM_CDCR_ARM_FREQ_SHIFT_DIVIDER);
		reg |= en_sw_dvfs;
		__raw_writel(reg, ccm_base + dvfs_data->ccm_cdcr_offset);
	}
	return ret;
}

static int mx6_set_cpu_freq(int op)
{
	int ret = 0;
	int org_cpu_rate;
	unsigned long rate = 0;
	int gp_volt = cpu_op_tbl[op].cpu_voltage;

	org_cpu_rate = clk_get_rate(cpu_clk);
	rate = cpu_op_tbl[op].cpu_rate;

	if (rate == org_cpu_rate)
		return ret;

	if (rate > org_cpu_rate) {
		/* Increase voltage first. */
		ret = regulator_set_voltage(cpu_regulator, gp_volt,
					    gp_volt);
		if (ret < 0) {
			printk(KERN_DEBUG "COULD NOT INCREASE GP VOLTAGE!!!!\n");
			return ret;
		}
		udelay(dvfs_data->delay_time);
	}
	ret = clk_set_rate(cpu_clk, rate);
	if (ret != 0) {
		printk(KERN_DEBUG "cannot set CPU clock rate\n");
		return ret;
	}

	if (rate < org_cpu_rate) {
		/* Increase voltage first. */
		ret = regulator_set_voltage(cpu_regulator, gp_volt,
					    gp_volt);
		if (ret < 0) {
			printk(KERN_DEBUG "COULD NOT INCREASE GP VOLTAGE!!!!\n");
			return ret;
		}
	}
	return ret;
}


static int set_cpu_freq(int op)
{
	int ret = 0;

	if (cpu_is_mx6())
		ret = mx6_set_cpu_freq(op);
	else
		ret = mx5_set_cpu_freq(op);

	cpufreq_trig_needed = 1;
	old_op = op;
	return ret;
}

static int start_dvfs(void)
{
	u32 reg, cpu_rate;
	unsigned long flags;

	if (dvfs_core_is_active)
		return 0;

	spin_lock_irqsave(&mxc_dvfs_core_lock, flags);

	clk_enable(dvfs_clk);

	/* get current working point */
	cpu_rate = clk_get_rate(cpu_clk);
	curr_op = cpu_op_nr - 1;
	do {
		if (cpu_rate <= cpu_op_tbl[curr_op].cpu_rate)
			break;
	} while (--curr_op >= 0);
	old_op = curr_op;

	dvfs_load_config(curr_op);

	if (curr_op == 0)
		maxf = 1;
	else
		maxf = 0;
	if (curr_op == (cpu_op_nr - 1))
		minf = 1;
	else
		minf = 0;

	/* config reg GPC_CNTR */
	reg = __raw_readl(gpc_base + dvfs_data->gpc_cntr_offset);

	reg &= ~MXC_GPCCNTR_GPCIRQM;
	/* GPCIRQ=1, select ARM IRQ */
	reg |= MXC_GPCCNTR_GPCIRQ_ARM;
	/* ADU=1, select ARM domain */
	if (!cpu_is_mx6())
		reg |= MXC_GPCCNTR_ADU;
	__raw_writel(reg, gpc_base + dvfs_data->gpc_cntr_offset);

	/* Set PREDIV bits */
	reg = __raw_readl(dvfs_data->membase + MXC_DVFSCORE_CNTR);
	reg = (reg & ~(dvfs_data->prediv_mask));
	reg |= (dvfs_data->prediv_val) << (dvfs_data->prediv_offset);
	__raw_writel(reg, dvfs_data->membase + MXC_DVFSCORE_CNTR);

	/* Enable DVFS interrupt */
	reg = __raw_readl(dvfs_data->membase + MXC_DVFSCORE_CNTR);
	/* FSVAIM=0 */
	reg = (reg & ~MXC_DVFSCNTR_FSVAIM);

	/* Set MAXF, MINF */
	reg = (reg & ~(MXC_DVFSCNTR_MAXF_MASK
				| MXC_DVFSCNTR_MINF_MASK));
	reg |= 1 << MXC_DVFSCNTR_MAXF_OFFSET;

	/* Select ARM domain */
	reg |= MXC_DVFSCNTR_DVFIS;
	/* Enable DVFS frequency adjustment interrupt */
	reg = (reg & ~MXC_DVFSCNTR_FSVAIM);
	/* Set load tracking buffer register source */
	reg = (reg & ~MXC_DVFSCNTR_LTBRSR_MASK);
	reg |= DVFS_LTBRSR;
	/* Set DIV3CK */
	reg = (reg & ~(dvfs_data->div3ck_mask));
	reg |= (dvfs_data->div3ck_val) << (dvfs_data->div3ck_offset);
	__raw_writel(reg, dvfs_data->membase + MXC_DVFSCORE_CNTR);

	/* Enable DVFS */
	if (cpu_is_mx6()) {
		unsigned long cpu_wfi = 0;
		int num_cpus = num_possible_cpus();
		reg = __raw_readl(dvfs_data->membase + MXC_DVFSCORE_EMAC);
		/* Need to enable DVFS tracking for each core that is active */
		do {
			if (cpu_active(num_cpus))
				set_bit(num_cpus, &cpu_wfi);
		} while (num_cpus--);
		reg |= cpu_wfi << 9;
		__raw_writel(reg, dvfs_data->membase + MXC_DVFSCORE_EMAC);
	} else {
		reg = __raw_readl(dvfs_data->membase + MXC_DVFSCORE_CNTR);
		reg |= MXC_DVFSCNTR_DVFEN;
		__raw_writel(reg, dvfs_data->membase + MXC_DVFSCORE_CNTR);
	}

	dvfs_core_is_active = 1;

	spin_unlock_irqrestore(&mxc_dvfs_core_lock, flags);

	printk(KERN_DEBUG "DVFS is started\n");
	return 0;
}

/*!
 * This function is called for module initialization.
 * It sets up the DVFS hardware.
 * It sets default values for DVFS thresholds and counters. The default
 * values was chosen from a set of different reasonable values. They was tested
 * and the default values in the driver gave the best results.
 * More work should be done to find optimal values.
 *
 * @return   0 if successful; non-zero otherwise.
 *
 */
static int init_dvfs_controller(void)
{
	/* DVFS loading config */
	dvfs_load_config(0);

	return 0;
}

static irqreturn_t dvfs_irq(int irq, void *dev_id)
{
	u32 reg;

	/* Check if DVFS0 (ARM) id requesting for freqency/voltage update */
	if ((__raw_readl(gpc_base + dvfs_data->gpc_cntr_offset)
			& MXC_GPCCNTR_DVFS0CR) == 0)
		return IRQ_NONE;

	/* Mask DVFS irq */
	reg = __raw_readl(dvfs_data->membase + MXC_DVFSCORE_CNTR);
	/* FSVAIM=1 */
	reg |= MXC_DVFSCNTR_FSVAIM;
	__raw_writel(reg, dvfs_data->membase + MXC_DVFSCORE_CNTR);

	/* Mask GPC1 irq */
	reg = __raw_readl(gpc_base + dvfs_data->gpc_cntr_offset);
	reg |= MXC_GPCCNTR_GPCIRQM | 0x1000000;
	__raw_writel(reg, gpc_base + dvfs_data->gpc_cntr_offset);

	schedule_delayed_work(&dvfs_core_handler, 0);
	return IRQ_HANDLED;
}

static void dvfs_core_work_handler(struct work_struct *work)
{
	u32 fsvai;
	u32 reg;
	u32 curr_cpu = 0;
	int ret = 0;
	int low_freq_bus_ready = 0;
	int bus_incr = 0, cpu_dcr = 0;
	int cpu;

	low_freq_bus_ready = low_freq_bus_used();

	/* Check DVFS frequency adjustment interrupt status */
	reg = __raw_readl(dvfs_data->membase + MXC_DVFSCORE_CNTR);
	fsvai = (reg & MXC_DVFSCNTR_FSVAI_MASK) >> MXC_DVFSCNTR_FSVAI_OFFSET;
	/* Check FSVAI, FSVAI=0 is error */
	if (fsvai == FSVAI_FREQ_NOCHANGE) {
		/* Do nothing. Freq change is not required */
		goto END;
	}
	curr_cpu = clk_get_rate(cpu_clk);
	/* If FSVAI indicate freq down,
	   check arm-clk is not in lowest frequency*/
	if (fsvai == FSVAI_FREQ_DECREASE) {
		if (curr_cpu <= cpu_op_tbl[cpu_op_nr - 1].cpu_rate) {
			minf = 1;
			mutex_lock(&bus_freq_mutex);
			if (low_bus_freq_mode) {
				mutex_unlock(&bus_freq_mutex);
				goto END;
			} else
				mutex_unlock(&bus_freq_mutex);
		} else {
			/* freq down */
			curr_op++;
			maxf = 0;
			if (curr_op >= cpu_op_nr) {
				curr_op = cpu_op_nr - 1;
				goto END;
			}
			cpu_dcr = 1;
			dvfs_load_config(curr_op);
		}
	} else {
		if (curr_cpu == cpu_op_tbl[0].cpu_rate) {
			maxf = 1;
			goto END;
		} else {
			mutex_lock(&bus_freq_mutex);
			if (!high_bus_freq_mode &&
				dvfs_config_setpoint == (cpu_op_nr + 1)) {
				/* bump up LP freq first. */
				bus_incr = 1;
				dvfs_load_config(cpu_op_nr);
			} else {
				/* freq up */
				curr_op = 0;
				maxf = 1;
				minf = 0;
				dvfs_load_config(0);
			}
			mutex_unlock(&bus_freq_mutex);
		}
	}

	low_freq_bus_ready = low_freq_bus_used();
	mutex_lock(&bus_freq_mutex);
	if ((curr_op == cpu_op_nr - 1) && (!low_bus_freq_mode)
	    && (low_freq_bus_ready) && !bus_incr) {
		if (!minf)
			set_cpu_freq(curr_op);
		/* If dvfs_core_op is greater than cpu_op_nr, it implies
		 * we support LPAPM mode for this platform.
		 */
		if (dvfs_core_op > cpu_op_nr) {
			set_low_bus_freq();
			dvfs_load_config(cpu_op_nr + 1);
		}
		mutex_unlock(&bus_freq_mutex);
	} else {
		if (!high_bus_freq_mode) {
			mutex_unlock(&bus_freq_mutex);
			set_high_bus_freq(1);
		} else
			mutex_unlock(&bus_freq_mutex);
		if (!bus_incr)
			ret = set_cpu_freq(curr_op);
		bus_incr = 0;
	}

END:
	if (cpufreq_trig_needed == 1) {
		/*Fix loops-per-jiffy */
#ifdef CONFIG_SMP
		for_each_online_cpu(cpu)
			per_cpu(cpu_data, cpu).loops_per_jiffy =
			dvfs_cpu_jiffies(per_cpu(cpu_data, cpu).loops_per_jiffy,
				curr_cpu / 1000, clk_get_rate(cpu_clk) / 1000);
#else
		u32 old_loops_per_jiffy = loops_per_jiffy;

		loops_per_jiffy =
			dvfs_cpu_jiffies(old_loops_per_jiffy,
				curr_cpu/1000, clk_get_rate(cpu_clk) / 1000);
#endif
#if defined (CONFIG_CPU_FREQ)
		/* Fix CPU frequency for CPUFREQ. */
		for (cpu = 0; cpu < num_online_cpus(); cpu++)
			cpufreq_get(cpu);
#endif
		cpufreq_trig_needed = 0;
	}

	/* Set MAXF, MINF */
	reg = __raw_readl(dvfs_data->membase + MXC_DVFSCORE_CNTR);
	reg = (reg & ~(MXC_DVFSCNTR_MAXF_MASK | MXC_DVFSCNTR_MINF_MASK));
	reg |= maxf << MXC_DVFSCNTR_MAXF_OFFSET;
	reg |= minf << MXC_DVFSCNTR_MINF_OFFSET;

	/* Enable DVFS interrupt */
	/* FSVAIM=0 */
	reg = (reg & ~MXC_DVFSCNTR_FSVAIM);
	reg |= FSVAI_FREQ_NOCHANGE;
	/* LBFL=1 */
	reg = (reg & ~MXC_DVFSCNTR_LBFL);
	reg |= MXC_DVFSCNTR_LBFL;
	__raw_writel(reg, dvfs_data->membase + MXC_DVFSCORE_CNTR);
	/*Unmask GPC1 IRQ */
	reg = __raw_readl(gpc_base + dvfs_data->gpc_cntr_offset);
	reg &= ~MXC_GPCCNTR_GPCIRQM;
	__raw_writel(reg, gpc_base + dvfs_data->gpc_cntr_offset);

}


/*!
 * This function disables the DVFS module.
 */
void stop_dvfs(void)
{
	u32 reg = 0;
	unsigned long flags;
	u32 curr_cpu;
	int cpu;
#ifndef CONFIG_SMP
	unsigned long old_loops_per_jiffy;
#endif

	if (dvfs_core_is_active) {

		/* Mask dvfs irq, disable DVFS */
		reg = __raw_readl(dvfs_data->membase
				  + MXC_DVFSCORE_CNTR);
		/* FSVAIM=1 */
		reg |= MXC_DVFSCNTR_FSVAIM;
		__raw_writel(reg, dvfs_data->membase
				  + MXC_DVFSCORE_CNTR);

		curr_op = 0;
		mutex_lock(&bus_freq_mutex);
		if (!high_bus_freq_mode) {
			mutex_unlock(&bus_freq_mutex);
			set_high_bus_freq(1);
		} else
			mutex_unlock(&bus_freq_mutex);

		curr_cpu = clk_get_rate(cpu_clk);
		if (curr_cpu != cpu_op_tbl[curr_op].cpu_rate) {
			set_cpu_freq(curr_op);

			/*Fix loops-per-jiffy */
#ifdef CONFIG_SMP
			for_each_online_cpu(cpu)
				per_cpu(cpu_data, cpu).loops_per_jiffy =
				dvfs_cpu_jiffies(per_cpu(cpu_data, cpu).loops_per_jiffy,
					curr_cpu/1000, clk_get_rate(cpu_clk) / 1000);
#else
		old_loops_per_jiffy = loops_per_jiffy;

		loops_per_jiffy =
			dvfs_cpu_jiffies(old_loops_per_jiffy,
				curr_cpu/1000, clk_get_rate(cpu_clk) / 1000);
#endif

#if defined (CONFIG_CPU_FREQ)
			/* Fix CPU frequency for CPUFREQ. */
			for (cpu = 0; cpu < num_online_cpus(); cpu++)
				cpufreq_get(cpu);
#endif
		}
		spin_lock_irqsave(&mxc_dvfs_core_lock, flags);

		reg = __raw_readl(dvfs_data->membase
				  + MXC_DVFSCORE_CNTR);
		reg = (reg & ~MXC_DVFSCNTR_DVFEN);
		__raw_writel(reg, dvfs_data->membase
				  + MXC_DVFSCORE_CNTR);

		spin_unlock_irqrestore(&mxc_dvfs_core_lock, flags);

		dvfs_core_is_active = 0;

		clk_disable(dvfs_clk);
	}

	printk(KERN_DEBUG "DVFS is stopped\n");
}

void dump_dvfs_core_regs()
{
	struct timeval cur;
	u32 diff = 0;
	if (core_prev_intr.tv_sec == 0)
		do_gettimeofday(&core_prev_intr);
	else {
		do_gettimeofday(&cur);
		diff = (cur.tv_sec - core_prev_intr.tv_sec)*1000000
			 + (cur.tv_usec - core_prev_intr.tv_usec);
		core_prev_intr = cur;
	}
	if (diff < 90000)
		printk(KERN_DEBUG "diff = %d\n", diff);

	printk(KERN_INFO "THRS = 0x%08x\n",
			__raw_readl(dvfs_data->membase
				    + MXC_DVFSCORE_THRS));
	printk(KERN_INFO "COUNT = 0x%08x\n",
			__raw_readl(dvfs_data->membase
				    + MXC_DVFSCORE_THRS + 0x04));
	printk(KERN_INFO "SIG1 = 0x%08x\n",
			__raw_readl(dvfs_data->membase
				    + MXC_DVFSCORE_THRS + 0x08));
	printk(KERN_INFO "SIG0 = 0x%08x\n",
			__raw_readl(dvfs_data->membase
				    + MXC_DVFSCORE_THRS + 0x0c));
	printk(KERN_INFO "GPC0 = 0x%08x\n",
			__raw_readl(dvfs_data->membase
				    + MXC_DVFSCORE_THRS + 0x10));
	printk(KERN_INFO "GPC1 = 0x%08x\n",
			__raw_readl(dvfs_data->membase
				    + MXC_DVFSCORE_THRS + 0x14));
	printk(KERN_INFO "GPBT = 0x%08x\n",
			__raw_readl(dvfs_data->membase
				    + MXC_DVFSCORE_THRS + 0x18));
	printk(KERN_INFO "EMAC = 0x%08x\n",
			__raw_readl(dvfs_data->membase
				    + MXC_DVFSCORE_THRS + 0x1c));
	printk(KERN_INFO "CNTR = 0x%08x\n",
			__raw_readl(dvfs_data->membase
				    + MXC_DVFSCORE_THRS + 0x20));
	printk(KERN_INFO "LTR0_0 = 0x%08x\n",
			__raw_readl(dvfs_data->membase
				    + MXC_DVFSCORE_THRS + 0x24));
	printk(KERN_INFO "LTR0_1 = 0x%08x\n",
			__raw_readl(dvfs_data->membase
				    + MXC_DVFSCORE_THRS + 0x28));
	printk(KERN_INFO "LTR1_0 = 0x%08x\n",
			__raw_readl(dvfs_data->membase
				    + MXC_DVFSCORE_THRS + 0x2c));
	printk(KERN_DEBUG "LTR1_1 = 0x%08x\n",
			__raw_readl(dvfs_data->membase
				    + MXC_DVFSCORE_THRS + 0x30));
	printk(KERN_INFO "PT0 = 0x%08x\n",
			__raw_readl(dvfs_data->membase
				    + MXC_DVFSCORE_THRS + 0x34));
	printk(KERN_INFO "PT1 = 0x%08x\n",
			__raw_readl(dvfs_data->membase
				    + MXC_DVFSCORE_THRS + 0x38));
	printk(KERN_INFO "PT2 = 0x%08x\n",
			__raw_readl(dvfs_data->membase
				    + MXC_DVFSCORE_THRS + 0x3c));
	printk(KERN_INFO "PT3 = 0x%08x\n",
			__raw_readl(dvfs_data->membase
				    + MXC_DVFSCORE_THRS + 0x40));
}
#if 0
static ssize_t downthreshold_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", dvfs_core_setpoint[0].downthr);
}

static ssize_t downthreshold_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	int ret = 0;
	int val;
	ret = sscanf(buf, "%u", &val);
	dvfs_core_setpoint[0].downthr = val;

	return size;
}

static ssize_t downcount_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", dvfs_core_setpoint[0].downcnt);
}

static ssize_t downcount_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	int ret = 0;
	int val;
	ret = sscanf(buf, "%u", &val);
	dvfs_core_setpoint[0].downcnt = val;

	return size;
}
#endif

static ssize_t dvfs_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (dvfs_core_is_active)
		return sprintf(buf, "DVFS is enabled\n");
	else
		return sprintf(buf, "DVFS is disabled\n");
}

static ssize_t dvfs_enable_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	if (strstr(buf, "1") != NULL) {
		if (start_dvfs() != 0)
			printk(KERN_ERR "Failed to start DVFS\n");
	} else if (strstr(buf, "0") != NULL)
		stop_dvfs();

	return size;
}

static ssize_t dvfs_regs_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (dvfs_core_is_active)
		dump_dvfs_core_regs();
	return 0;
}

static ssize_t dvfs_regs_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	if (dvfs_core_is_active)
		dump_dvfs_core_regs();
	return 0;

	return size;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR,
				dvfs_enable_show, dvfs_enable_store);
static DEVICE_ATTR(show_regs, S_IRUGO, dvfs_regs_show,
				dvfs_regs_store);

/*!
 * This is the probe routine for the DVFS driver.
 *
 * @param   pdev   The platform device structure
 *
 * @return         The function returns 0 on success
 */
static int __devinit mxc_dvfs_core_probe(struct platform_device *pdev)
{
	int err = 0;
	struct resource *res;

	printk(KERN_INFO "mxc_dvfs_core_probe\n");
	dvfs_dev = &pdev->dev;
	dvfs_data = pdev->dev.platform_data;

	INIT_DELAYED_WORK(&dvfs_core_handler, dvfs_core_work_handler);

	pll1_sw_clk = clk_get(NULL, "pll1_sw_clk");
	if (IS_ERR(pll1_sw_clk)) {
		printk(KERN_INFO "%s: failed to get pll1_sw_clk\n", __func__);
		return PTR_ERR(pll1_sw_clk);
	}

	cpu_clk = clk_get(NULL, dvfs_data->clk1_id);
	if (IS_ERR(cpu_clk)) {
		printk(KERN_ERR "%s: failed to get cpu clock\n", __func__);
		return PTR_ERR(cpu_clk);
	}
	if (!cpu_is_mx6()) {
		dvfs_clk = clk_get(NULL, dvfs_data->clk2_id);
		if (IS_ERR(dvfs_clk)) {
			printk(KERN_ERR "%s: failed to get dvfs clock\n", __func__);
			return PTR_ERR(dvfs_clk);
		}
	}
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		err = -ENODEV;
		goto err1;
	}
	dvfs_data->membase = ioremap(res->start, res->end - res->start + 1);
	/*
	 * Request the DVFS interrupt
	 */
	dvfs_data->irq = platform_get_irq(pdev, 0);
	if (dvfs_data->irq < 0) {
		err = dvfs_data->irq;
		goto err2;
	}

	/* request the DVFS interrupt */
	err = request_irq(dvfs_data->irq, dvfs_irq, IRQF_SHARED, "dvfs",
			  dvfs_dev);
	if (err) {
		printk(KERN_ERR
		       "DVFS: Unable to attach to DVFS interrupt,err = %d",
		       err);
		goto err2;
	}

	dvfs_core_setpoint = get_dvfs_core_op(&dvfs_core_op);
	if (dvfs_core_setpoint == NULL) {
		printk(KERN_ERR "No dvfs_core working point table defined\n");
		goto err3;
	}

	clk_enable(dvfs_clk);
	err = init_dvfs_controller();
	if (err) {
		printk(KERN_ERR "DVFS: Unable to initialize DVFS");
		return err;
	}
	clk_disable(dvfs_clk);

	err = sysfs_create_file(&pdev->dev.kobj, &dev_attr_enable.attr);
	if (err) {
		printk(KERN_ERR
		       "DVFS: Unable to register sysdev entry for DVFS");
		goto err3;
	}

	err = sysfs_create_file(&dvfs_dev->kobj, &dev_attr_show_regs.attr);
	if (err) {
		printk(KERN_ERR
		       "DVFS: Unable to register sysdev entry for DVFS");
		goto err3;
	}

	/* Set the current working point. */
	cpu_op_tbl = get_cpu_op(&cpu_op_nr);
	old_op = 0;
	curr_op = 0;
	dvfs_core_resume = 0;
	cpufreq_trig_needed = 0;

	return err;
err3:
	free_irq(dvfs_data->irq, dvfs_dev);
err2:
	iounmap(dvfs_data->membase);
err1:
	dev_err(&pdev->dev, "Failed to probe DVFS CORE\n");
	return err;
}

/*!
 * This function is called to put DVFS in a low power state.
 *
 * @param   pdev  the device structure
 * @param   state the power state the device is entering
 *
 * @return  The function always returns 0.
 */
static int mxc_dvfs_core_suspend(struct platform_device *pdev,
				 pm_message_t state)
{
	if (dvfs_core_is_active) {
		dvfs_core_resume = 1;
		stop_dvfs();
	}

	return 0;
}

/*!
 * This function is called to resume the MU from a low power state.
 *
 * @param   dev   the device structure
 * @param   level the stage in device suspension process that we want the
 *                device to be put in
 *
 * @return  The function always returns 0.
 */
static int mxc_dvfs_core_resume(struct platform_device *pdev)
{
	if (dvfs_core_resume) {
		dvfs_core_resume = 0;
		start_dvfs();
	}

	return 0;
}

static struct platform_driver mxc_dvfs_core_driver = {
	.driver = {
		   .name = "imx_dvfscore",
		   },
	.probe = mxc_dvfs_core_probe,
	.suspend = mxc_dvfs_core_suspend,
	.resume = mxc_dvfs_core_resume,
};

static int __init dvfs_init(void)
{
	if (platform_driver_register(&mxc_dvfs_core_driver) != 0) {
		printk(KERN_ERR "mxc_dvfs_core_driver register failed\n");
		return -ENODEV;
	}

	dvfs_core_is_active = 0;
	printk(KERN_INFO "DVFS driver module loaded\n");

	return 0;
}

static void __exit dvfs_cleanup(void)
{
	stop_dvfs();

	/* release the DVFS interrupt */
	free_irq(dvfs_data->irq, dvfs_dev);

	sysfs_remove_file(&dvfs_dev->kobj, &dev_attr_enable.attr);
	/* Unregister the device structure */
	platform_driver_unregister(&mxc_dvfs_core_driver);

	iounmap(ccm_base);
	iounmap(dvfs_data->membase);
	clk_put(cpu_clk);
	clk_put(dvfs_clk);

	dvfs_core_is_active = 0;
	printk(KERN_INFO "DVFS driver module unloaded\n");

}

late_initcall(dvfs_init);
module_exit(dvfs_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("DVFS driver");
MODULE_LICENSE("GPL");
