/*
 * Copyright 2005-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @defgroup MXC_Oprofile ARM11 EVTMON Driver for Oprofile
 */

/*!
 * @file op_model_arm11_evtmon.c
 *
 *Based on the op_model_xscale.c driver by author Zwane Mwaikambo
 *
 * @ingroup MXC_Oprofile
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/oprofile.h>
#include <linux/interrupt.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/system.h>

#include "op_counter.h"
#include "op_arm_model.h"
#include "evtmon_regs.h"

struct pmu_counter {
	volatile unsigned long ovf;
	unsigned long reset_counter;
};

enum { EMC0 = MAX_PMUCOUNTERS, EMC1, EMC2, EMC3, MAX_L2COUNTERS };

static struct pmu_counter results[MAX_L2COUNTERS];

struct pmu_type {
	int id;
	char *name;
	int num_counters;
	unsigned int int_enable;
	unsigned int cnt_ovf[MAX_L2COUNTERS];
	unsigned int int_mask[MAX_L2COUNTERS];
};

static struct pmu_type pmu_parms[] = {
	{
	 .id = 0,
	 .int_mask = {[EMC0] = 0x800,[EMC1] = 0x400,[EMC2] =
		      0x200,[EMC3] = 0x100},
	 .cnt_ovf = {[EMC0] = 0x1,[EMC1] = 0x2,[EMC2] = 0x4,[EMC3] = 0x8},
	 },
};

static struct pmu_type *pmu;

extern void l2x0_evtbus_enable(void);
extern void l2x0_evtbus_disable(void);

/*!
 * function is used to write the EVTMON counter configuration register.
 */
int l2em_configure_counter(int nr, int type)
{
	/* Configure the counter event source */
	__raw_writel(((type << 2) & 0x7c), L2EM_CC(nr));
	if (type)
		__raw_writel((__raw_readl(L2EM_CC(nr)) | EM_SET_INT),
			     L2EM_CC(nr));

	return 0;
}

/*!
 * function is used to write the EVTMON counters
 */
void write_l2counter(int nr, u32 val)
{
	__raw_writel(val, L2EM_CNT(nr));
}

/*!
 * function is used to check the status of the ARM11 evtmon counters
 */
int arm11_evtmon_setup_ctrs(void)
{
	int i;

	for (i = EMC0; i < MAX_L2COUNTERS; i++) {
		if (counter_config[i].enabled)
			continue;
		counter_config[i].event = EVT_UNUSED;
	}

	for (i = EMC0; i < MAX_L2COUNTERS; i++) {
		if (counter_config[i].event == EVT_UNUSED) {
			counter_config[i].event = 0;
			pmu->int_enable &= ~pmu->int_mask[i];
			pr_debug
			    ("arm11_setup_ctrs: The counter event is %lu for counter%d\n",
			     counter_config[i].event, i);
			continue;
		}

		results[i].reset_counter = counter_config[i].count;
		write_l2counter(i - EMC0, -(u32) counter_config[i].count);
		l2em_configure_counter(i - EMC0, counter_config[i].event);

		pmu->int_enable |= pmu->int_mask[i];
		pr_debug
		    ("arm11_setup_ctrs: The values of int mask and enables are %x, %x\n",
		     pmu->int_mask[i], pmu->int_enable);

	}

	return 0;
}

/*!
 * function used to start the ARM11 evtmon counters
 */
void arm11_evtmon_stop(void)
{

	/* Disable the EVTMON */
	__raw_writel((__raw_readl(L2EM_CTRL) & ~EVTMON_ENABLE), L2EM_CTRL);

	/* Disable the EVTBUS */
	l2x0_evtbus_disable();

}

/*!
 * function used to start the ARM11 evtmon counters
 */
int arm11_evtmon_start(void)
{

	/* Enable the EVTBUS */
	l2x0_evtbus_enable();

#ifdef ECT_WORKAROUND
	__raw_writel(ENABLE_CTI_CLOCK, CLKCTL_SET_CTRL);
	/* Unlock the AHB Interface */
	__raw_writel(UNLOCK_ECT_CODE, ECT_CTI_LOCK);
	/* Trigger to Channel Mapping */
	__raw_writel(ECT_CTI_CHAN_2, ECT_CTI_INEN(ECT_CTI_TRIGIN_1));
	/* Channel to triggers mapping */
	__raw_writel(ECT_CTI_CHAN_2, ECT_CTI_OUTEN(ECT_CTI_TRIGOUT_2));
	/* Trigger to Channel Mapping */
	__raw_writel(ECT_CTI_CHAN_3, ECT_CTI_INEN(ECT_CTI_TRIGIN_7));
	/* Channel to triggers mapping */
	__raw_writel(ECT_CTI_CHAN_3, ECT_CTI_OUTEN(ECT_CTI_TRIGOUT_6));
	/* Enable CTI Logic */
	__raw_writel(ENABLE_ECT, ECT_CTI_CONTROL);
#endif

	/* Enable EVTMON with Edge triggered interrupt of one Clock Cycle */
	__raw_writel((__raw_readl(L2EM_CTRL) |
		      (L2EM_INT_EDGE | L2EM_INT_CLK_CYCLES)), L2EM_CTRL);
	__raw_writel((__raw_readl(L2EM_CTRL) | L2EM_ENABLE_MASK), L2EM_CTRL);

	return 0;
}

/*!
 * function detect the ARM11 evtmon counters
 */
int arm11_evtmon_detect(void)
{
	int ret = 0;

	pmu = &pmu_parms[0];
	op_armv6_spec.num_counters = MAX_L2COUNTERS;

	return ret;
}
