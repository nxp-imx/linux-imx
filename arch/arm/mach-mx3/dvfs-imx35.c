/*
 *  Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
  * @file dvfs.c
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
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <mach/hardware.h>
#include "crm_regs.h"

/*
 * The frequency of div_3_clk will affect the dvfs sample rate..
 */
#define DVFS_DIV3CK		(3 << MXC_CCM_LTR0_DIV3CK_OFFSET)

/*
 * Panic threshold. Panic frequency change request
 * will be sent if DVFS counter value will be more than this value.
 */
#define DVFS_PNCTHR		(63 << MXC_CCM_LTR1_PNCTHR_OFFSET)

/*
 * Load tracking buffer source: 1 for ld_add; 0 for pre_ld_add
 */
#define DVFS_LTBRSR		(1 << MXC_CCM_LTR1_LTBRSR_OFFSET)

/* EMAC defines how many samples are included in EMA calculation */
#define DVFS_EMAC		(0x20 << MXC_CCM_LTR2_EMAC_OFFSET)

/*
 * Frequency increase threshold. Increase frequency change request
 * will be sent if DVFS counter value will be more than this value.
 */
#define DVFS_UPTHR(val)		(val << MXC_CCM_LTR0_UPTHR_OFFSET)

/*
 * Frequency decrease threshold. Decrease frequency change request
 * will be sent if DVFS counter value will be less than this value.
 */
#define DVFS_DNTHR(val)		(val << MXC_CCM_LTR0_DNTHR_OFFSET)

/*
 * DNCNT defines the amount of times the down threshold should be exceeded
 * before DVFS will trigger frequency decrease request.
 */
#define DVFS_DNCNT(val)		(val << MXC_CCM_LTR1_DNCNT_OFFSET)

/*
 * UPCNT defines the amount of times the up threshold should be exceeded
 * before DVFS will trigger frequency increase request.
 */
#define DVFS_UPCNT(val)		(val << MXC_CCM_LTR1_UPCNT_OFFSET)

#define DVFS_DVSUP(val)		(val << MXC_CCM_PMCR0_DVSUP_OFFSET)

#define MXC_DVFS_MAX_WP_NUM 2

enum {
	FSVAI_FREQ_NOCHANGE = 0x0,
	FSVAI_FREQ_INCREASE,
	FSVAI_FREQ_DECREASE,
	FSVAI_FREQ_EMERG,
};

struct dvfs_wp {
	unsigned long cpu_rate;
	u32 core_voltage;
	u32 dvsup;
	u32 dnthr;
	u32 upthr;
	u32 dncnt;
	u32 upcnt;
};

/* the default working points for MX35 TO2 DVFS. */
static struct dvfs_wp dvfs_wp_tbl[MXC_DVFS_MAX_WP_NUM] = {
	{399000000, 1200000, DVFS_DVSUP(DVSUP_LOW), DVFS_DNTHR(18),
	 DVFS_UPTHR(31), DVFS_DNCNT(0x33),
	 DVFS_UPCNT(0x33)},
/* TBD: Need to set default voltage according to published data sheet */
	{532000000, 1350000, DVFS_DVSUP(DVSUP_TURBO), DVFS_DNTHR(18),
	 DVFS_UPTHR(30), DVFS_DNCNT(0x33),
	 DVFS_UPCNT(0x33)}
};

static u8 dvfs_wp_num = MXC_DVFS_MAX_WP_NUM;

 /* Used for tracking the number of interrupts */
static u32 dvfs_nr_up[MXC_DVFS_MAX_WP_NUM];
static u32 dvfs_nr_dn[MXC_DVFS_MAX_WP_NUM];
static unsigned long stored_cpu_rate;	/* cpu rate before DVFS starts */
static u32 stored_pmcr0;
static int dvfs_is_active;	/* indicate DVFS is active or not */

static struct delayed_work dvfs_work;

/*
 * Clock structures
 */
static struct clk *cpu_clk;
static struct regulator *core_reg;

const static u8 ltr_gp_weight[] = {
	0,			/* 0 */
	0,
	0,
	0,
	0,
	0,			/* 5 */
	0,
	0,
	0,
	0,
	0,			/* 10 */
	0,
	0,
	0,
	0,
	0,			/* 15 */
};

DEFINE_SPINLOCK(mxc_dvfs_lock);

/*!
 * This function sets the weight of general purpose signals
 * @param   gp_id   number of general purpose bit
 * @param   weight  the weight of the general purpose bit
 */
static void set_gp_weight(int gp_id, u8 weight)
{
	u32 reg;

	if (gp_id < 9) {
		reg = __raw_readl(MXC_CCM_LTR3);
		reg = (reg & ~(MXC_CCM_LTR3_WSW_MASK(gp_id))) |
		    (weight << MXC_CCM_LTR3_WSW_OFFSET(gp_id));
		__raw_writel(reg, MXC_CCM_LTR3);
	} else if (gp_id < 16) {
		reg = __raw_readl(MXC_CCM_LTR2);
		reg = (reg & ~(MXC_CCM_LTR2_WSW_MASK(gp_id))) |
		    (weight << MXC_CCM_LTR2_WSW_OFFSET(gp_id));
		__raw_writel(reg, MXC_CCM_LTR2);
	}
}

/*!
 * This function sets upper threshold, lower threshold,
 * up-counter, down-counter for load tracking.
 * @param   upthr  upper threshold
 * @param   dnthr  lower threshold
 * @param   upcnt  up counter
 * @param   dncnt  down counter
 */
static void set_ltr_thres_counter(u32 upthr, u32 dnthr, u32 upcnt, u32 dncnt)
{
	u32 reg;
	reg = __raw_readl(MXC_CCM_LTR0);
	reg =
	    (reg &
	     ~(MXC_CCM_LTR0_UPTHR_MASK |
	       MXC_CCM_LTR0_DNTHR_MASK)) | upthr | dnthr;
	__raw_writel(reg, MXC_CCM_LTR0);

	reg = __raw_readl(MXC_CCM_LTR1);
	reg =
	    (reg &
	     ~(MXC_CCM_LTR1_UPCNT_MASK |
	       MXC_CCM_LTR1_DNCNT_MASK)) | upcnt | dncnt;
	__raw_writel(reg, MXC_CCM_LTR1);
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
	u32 i, reg;

	/* setup LTR0 */
	reg = __raw_readl(MXC_CCM_LTR0);
	reg = (reg & ~(MXC_CCM_LTR0_DIV3CK_MASK)) | DVFS_DIV3CK;
	__raw_writel(reg, MXC_CCM_LTR0);

	/* set up LTR1 */
	reg = __raw_readl(MXC_CCM_LTR1);
	reg = (reg & ~(MXC_CCM_LTR1_PNCTHR_MASK | MXC_CCM_LTR1_LTBRSR_MASK));
	reg = reg | DVFS_PNCTHR | DVFS_LTBRSR;
	__raw_writel(reg, MXC_CCM_LTR1);

	/* setup LTR2 */
	reg = __raw_readl(MXC_CCM_LTR2);
	reg = (reg & ~(MXC_CCM_LTR2_EMAC_MASK)) | DVFS_EMAC;
	__raw_writel(reg, MXC_CCM_LTR2);

	/* Set general purpose weights to 0 */
	for (i = 0; i < 16; i++)
		set_gp_weight(i, ltr_gp_weight[i]);

	/* ARM interrupt, mask load buf full interrupt */
	reg = __raw_readl(MXC_CCM_PMCR0);
	reg |= MXC_CCM_PMCR0_DVFIS | MXC_CCM_PMCR0_LBMI;
	__raw_writel(reg, MXC_CCM_PMCR0);

	return 0;
}

static void dvfs_workqueue_handler(struct work_struct *work)
{
	u32 pmcr0 = stored_pmcr0;
	u32 fsvai = (pmcr0 & MXC_CCM_PMCR0_FSVAI_MASK) >>
	    MXC_CCM_PMCR0_FSVAI_OFFSET;
	u32 dvsup = (pmcr0 & MXC_CCM_PMCR0_DVSUP_MASK) >>
	    MXC_CCM_PMCR0_DVSUP_OFFSET;
	u32 curr_cpu;
	u8 curr_dvfs;

	if (!dvfs_is_active)
		return;

	if (fsvai == FSVAI_FREQ_NOCHANGE) {
		/* Do nothing. Freq change is not required */
		printk(KERN_WARNING "fsvai should not be 0\n");
		goto exit;
	}

	if (((dvsup == DVSUP_LOW) && (fsvai == FSVAI_FREQ_DECREASE)) ||
	    ((dvsup == DVSUP_TURBO) && ((fsvai == FSVAI_FREQ_INCREASE) ||
					(fsvai == FSVAI_FREQ_EMERG)))) {
		/* Interrupt should be disabled in these cases according to
		 * the spec since DVFS is already at lowest (highest) state */
		printk(KERN_WARNING "Something is wrong?\n");
		goto exit;
	}

	/*Disable DPTC voltage update */
	pmcr0 = pmcr0 & ~MXC_CCM_PMCR0_DPVCR;
	__raw_writel(pmcr0, MXC_CCM_PMCR0);

	curr_cpu = clk_get_rate(cpu_clk);
	for (curr_dvfs = 0; curr_dvfs < dvfs_wp_num; curr_dvfs++) {
		if (dvfs_wp_tbl[curr_dvfs].cpu_rate == curr_cpu) {
			if (fsvai == FSVAI_FREQ_DECREASE) {
				curr_dvfs--;
				dvfs_nr_dn[dvsup]++;
				/*reduce frequency and then voltage */
				clk_set_rate(cpu_clk,
					     dvfs_wp_tbl[curr_dvfs].cpu_rate);
				regulator_set_voltage(core_reg,
						      dvfs_wp_tbl[curr_dvfs].
						      core_voltage,
						      dvfs_wp_tbl[curr_dvfs].
						      core_voltage);
				pr_info("Decrease frequency to: %ld \n",
					dvfs_wp_tbl[curr_dvfs].cpu_rate);
			} else {
				/*increase freq to the highest one */
				curr_dvfs = dvfs_wp_num - 1;
				dvfs_nr_up[dvsup]++;
				/*Increase voltage and then frequency */
				regulator_set_voltage(core_reg,
						      dvfs_wp_tbl[curr_dvfs].
						      core_voltage,
						      dvfs_wp_tbl[curr_dvfs].
						      core_voltage);
				clk_set_rate(cpu_clk,
					     dvfs_wp_tbl[curr_dvfs].cpu_rate);
				pr_info("Increase frequency to: %ld \n",
					dvfs_wp_tbl[curr_dvfs].cpu_rate);
			}
			pmcr0 = (pmcr0 & ~MXC_CCM_PMCR0_DVSUP_MASK)
			    | (dvfs_wp_tbl[curr_dvfs].dvsup);
			__raw_writel(pmcr0, MXC_CCM_PMCR0);

			set_ltr_thres_counter(dvfs_wp_tbl[curr_dvfs].upthr,
					      dvfs_wp_tbl[curr_dvfs].dnthr,
					      dvfs_wp_tbl[curr_dvfs].upcnt,
					      dvfs_wp_tbl[curr_dvfs].dncnt);
			break;
		}
	}

      exit:
	/* unmask interrupt */
	pmcr0 = pmcr0 & ~MXC_CCM_PMCR0_FSVAIM;
	__raw_writel(pmcr0, MXC_CCM_PMCR0);
	/*DVFS update finish */
	pmcr0 = (pmcr0 | MXC_CCM_PMCR0_DVFS_UPDATE_FINISH);
	__raw_writel(pmcr0, MXC_CCM_PMCR0);
}

static irqreturn_t dvfs_irq(int irq, void *dev_id)
{

	u32 pmcr0 = __raw_readl(MXC_CCM_PMCR0);

	/* Config dvfs_start bit */
	pmcr0 = pmcr0 | MXC_CCM_PMCR0_DVFS_START;
	/*Mask interrupt */
	pmcr0 = pmcr0 | MXC_CCM_PMCR0_FSVAIM;
	__raw_writel(pmcr0, MXC_CCM_PMCR0);

	stored_pmcr0 = pmcr0;
	schedule_delayed_work(&dvfs_work, 0);

	return IRQ_RETVAL(1);
}

/*!
 * This function enables the DVFS module.
 */
static int start_dvfs(void)
{
	u32 reg = 0;
	unsigned long flags;
	u8 i;

	if (dvfs_is_active) {
		pr_info("DVFS is already started\n");
		return 0;
	}

	spin_lock_irqsave(&mxc_dvfs_lock, flags);

	stored_cpu_rate = clk_get_rate(cpu_clk);
	for (i = 0; i < dvfs_wp_num; i++) {
		if (dvfs_wp_tbl[i].cpu_rate == stored_cpu_rate) {
			/*Set LTR0 and LTR1 */
			set_ltr_thres_counter(dvfs_wp_tbl[i].upthr,
					      dvfs_wp_tbl[i].dnthr,
					      dvfs_wp_tbl[i].upcnt,
					      dvfs_wp_tbl[i].dncnt);

			reg = __raw_readl(MXC_CCM_PMCR0);
			reg =
			    (reg & ~MXC_CCM_PMCR0_DVSUP_MASK) | (dvfs_wp_tbl[i].
								 dvsup);
			/* enable dvfs and interrupt */
			reg =
			    (reg & ~MXC_CCM_PMCR0_FSVAIM) | MXC_CCM_PMCR0_DVFEN;

			__raw_writel(reg, MXC_CCM_PMCR0);

			dvfs_is_active = 1;
			pr_info("DVFS Starts\n");
			break;
		}
	}

	spin_unlock_irqrestore(&mxc_dvfs_lock, flags);
	if (dvfs_is_active)
		return 0;
	else
		return 1;
}

/*!
 * This function disables the DVFS module.
 */
static void stop_dvfs(void)
{
	u32 pmcr0;
	unsigned long curr_cpu = clk_get_rate(cpu_clk);
	u8 index;

	if (dvfs_is_active) {

		pmcr0 = __raw_readl(MXC_CCM_PMCR0);
		/* disable dvfs and its interrupt */
		pmcr0 = (pmcr0 & ~MXC_CCM_PMCR0_DVFEN) | MXC_CCM_PMCR0_FSVAIM;
		__raw_writel(pmcr0, MXC_CCM_PMCR0);

		if (stored_cpu_rate < curr_cpu) {
			for (index = 0; index < dvfs_wp_num; index++) {
				if (dvfs_wp_tbl[index].cpu_rate ==
				    stored_cpu_rate)
					break;
			}
			clk_set_rate(cpu_clk, stored_cpu_rate);
			regulator_set_voltage(core_reg,
					      dvfs_wp_tbl[index].core_voltage,
					      dvfs_wp_tbl[index].core_voltage);
		} else if (stored_cpu_rate > curr_cpu) {
			for (index = 0; index < dvfs_wp_num; index++) {
				if (dvfs_wp_tbl[index].cpu_rate ==
				    stored_cpu_rate)
					break;
			}
			regulator_set_voltage(core_reg,
					      dvfs_wp_tbl[index].core_voltage,
					      dvfs_wp_tbl[index].core_voltage);
			clk_set_rate(cpu_clk, stored_cpu_rate);
		}

		dvfs_is_active = 0;
	}

	pr_info("DVFS is stopped\n");
}

static ssize_t dvfs_enable_store(struct sys_device *dev,
					struct sysdev_attribute *attr,
					const char *buf, size_t size)
{
	if (strstr(buf, "1") != NULL) {
		if (start_dvfs() != 0)
			printk(KERN_ERR "Failed to start DVFS\n");
	} else if (strstr(buf, "0") != NULL) {
		stop_dvfs();
	}

	return size;
}

static ssize_t dvfs_status_show(struct sys_device *dev,
					struct sysdev_attribute *attr,
					char *buf)
{
	int size = 0, i;

	if (dvfs_is_active)
		size = sprintf(buf, "DVFS is enabled\n");
	else
		size = sprintf(buf, "DVFS is disabled\n");

	size += sprintf((buf + size), "UP:\t");
	for (i = 0; i < MXC_DVFS_MAX_WP_NUM; i++)
		size += sprintf((buf + size), "%d\t", dvfs_nr_up[i]);
	size += sprintf((buf + size), "\n");

	size += sprintf((buf + size), "DOWN:\t");
	for (i = 0; i < MXC_DVFS_MAX_WP_NUM; i++)
		size += sprintf((buf + size), "%d\t", dvfs_nr_dn[i]);
	size += sprintf((buf + size), "\n");

	return size;
}

static ssize_t dvfs_status_store(struct sys_device *dev,
					struct sysdev_attribute *attr,
					const char *buf, size_t size)
{
	if (strstr(buf, "reset") != NULL) {
		int i;
		for (i = 0; i < MXC_DVFS_MAX_WP_NUM; i++) {
			dvfs_nr_up[i] = 0;
			dvfs_nr_dn[i] = 0;
		}
	}

	return size;
}

static SYSDEV_ATTR(enable, 0200, NULL, dvfs_enable_store);
static SYSDEV_ATTR(status, 0644, dvfs_status_show, dvfs_status_store);

static struct sysdev_class dvfs_sysclass = {
	.name = "dvfs",
};

static struct sys_device dvfs_device = {
	.id = 0,
	.cls = &dvfs_sysclass,
};

static int dvfs_sysdev_ctrl_init(void)
{
	int err;

	err = sysdev_class_register(&dvfs_sysclass);
	if (!err)
		err = sysdev_register(&dvfs_device);
	if (!err) {
		err = sysdev_create_file(&dvfs_device, &attr_enable);
		err = sysdev_create_file(&dvfs_device, &attr_status);
	}

	return err;
}

static void dvfs_sysdev_ctrl_exit(void)
{
	sysdev_remove_file(&dvfs_device, &attr_enable);
	sysdev_remove_file(&dvfs_device, &attr_status);
	sysdev_unregister(&dvfs_device);
	sysdev_class_unregister(&dvfs_sysclass);
}

static int __init dvfs_init(void)
{
	int err = 0;
	u8 index;
	unsigned long curr_cpu;

	if (cpu_is_mx35_rev(CHIP_REV_1_0) == 1) {
		/*
		 * Don't support DVFS for auto path in TO1 because
		 * the voltages under 399M are all 1.2v
		 */
		if (!(__raw_readl(MXC_CCM_PDR0) & MXC_CCM_PDR0_AUTO_CON)) {
			pr_info("MX35 TO1 auto path, no need to use DVFS \n");
			return -1;
		}
	}

	cpu_clk = clk_get(NULL, "cpu_clk");
	curr_cpu = clk_get_rate(cpu_clk);

	if (board_is_rev(BOARD_REV_2))
		core_reg = regulator_get(NULL, "SW2");
	else
		core_reg = regulator_get(NULL, "SW3");

	dvfs_is_active = 0;

	/*Set voltage */
	for (index = 0; index < dvfs_wp_num; index++) {
		if (dvfs_wp_tbl[index].cpu_rate == curr_cpu
			&& !IS_ERR(core_reg)) {
			regulator_set_voltage(core_reg,
					      dvfs_wp_tbl[index].core_voltage,
					      dvfs_wp_tbl[index].core_voltage);
			break;
		}
	}

	err = init_dvfs_controller();
	if (err) {
		printk(KERN_ERR "DVFS: Unable to initialize DVFS");
		return err;
	}

	INIT_DELAYED_WORK(&dvfs_work, dvfs_workqueue_handler);

	/* request the DVFS interrupt */
	err = request_irq(MXC_INT_DVFS, dvfs_irq, IRQF_DISABLED, "dvfs", NULL);
	if (err) {
		printk(KERN_ERR "DVFS: Unable to attach to DVFS interrupt");
		return err;
	}

	err = dvfs_sysdev_ctrl_init();
	if (err) {
		printk(KERN_ERR
		       "DVFS: Unable to register sysdev entry for dvfs");
		return err;
	}

	return err;
}

static void __exit dvfs_cleanup(void)
{
	stop_dvfs();

	/* release the DVFS interrupt */
	free_irq(MXC_INT_DVFS, NULL);

	dvfs_sysdev_ctrl_exit();

	clk_put(cpu_clk);
	regulator_put(core_reg);
}

module_init(dvfs_init);
module_exit(dvfs_cleanup);

MODULE_AUTHOR("Freescale Seminconductor, Inc.");
MODULE_DESCRIPTION("DVFS driver");
MODULE_LICENSE("GPL");
