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
 * @defgroup MXC_Oprofile ARM11 Driver for Oprofile
 */

/*!
 * @file op_model_arm11.c
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

/*!
 * defines used in ARM11 performance unit
 */
#define PMU_ENABLE       0x001	/* Enable counters */
#define EVTMON_ENABLE    0x001	/* Enable EVTMON */
#define PMN_RESET        0x002	/* Reset event counters */
#define CCNT_RESET       0x004	/* Reset clock counter */
#define PMU_RESET        (CCNT_RESET | PMN_RESET)
#define PMU_CNT64        0x008	/* Make CCNT count every 64th cycle */

#define PMU_FLAG_CR0     0x080
#define PMU_FLAG_CR1     0x100
#define PMU_FLAG_CC      0x200

/*!
 * Different types of events that can be counted by the ARM11 PMU
 * as used by Oprofile userspace.
 */
#define EVT_ICACHE_MISS                  0x00
#define EVT_STALL_INSTR                  0x01
#define EVT_DATA_STALL                   0x02
#define EVT_ITLB_MISS                    0x03
#define EVT_DTLB_MISS                    0x04
#define EVT_BRANCH                       0x05
#define EVT_BRANCH_MISS                  0x06
#define EVT_INSTRUCTION                  0x07
#define EVT_DCACHE_FULL_STALL_CONTIG     0x09
#define EVT_DCACHE_ACCESS                0x0A
#define EVT_DCACHE_MISS                  0x0B
#define EVT_DCACE_WRITE_BACK             0x0C
#define EVT_PC_CHANGED                   0x0D
#define EVT_TLB_MISS                     0x0F
#define EVT_BCU_REQUEST                  0x10
#define EVT_BCU_FULL                     0x11
#define EVT_BCU_DRAIN                    0x12
#define EVT_ETMEXTOT0                    0x20
#define EVT_ETMEXTOT1                    0x21
/* EVT_CCNT is not hardware defined */
#define EVT_CCNT                         0xFE
#define EVT_INCREMENT                    0xFF
#define EVT_UNUSED                       0x100

#define ECT_WORKAROUND

#define COUNTER_MSB		0x80000000
#define ENABLE_L2CACHE		0x1
#define ENABLE_EVTBUS		0x100000
#define PMU_OVERFLOWBIT_MASK	0x700
#define VAR_NUM			0x0
#define REV_NUM			0x2

#ifdef ECT_WORKAROUND
#define ENABLE_CTI_CLOCK	0x00000020
#define UNLOCK_ECT_CODE 	0x0ACCE550
#define ECT_CTI_CHAN_2		0x4
#define ECT_CTI_CHAN_3		0x8
#define ECT_CTI_TRIGIN_1	1
#define ECT_CTI_TRIGIN_7	7
#define ECT_CTI_TRIGOUT_2	2
#define ECT_CTI_TRIGOUT_6	6
#define ENABLE_ECT		0x1
#define ACK_TRIG_OUT_2		0x4
#define EM_SET_INT		L2EM_ENABLE_CNTINCRINT
#define EVENT_OVERFLOW_INT	INT_ECT
#else
#define EVENT_OVERFLOW_INT	ARM11_PMU_IRQ
#define EM_SET_INT		L2EM_ENABLE_OVERFLOWINT
#endif

struct pmu_counter {
	volatile unsigned long ovf;
	unsigned long reset_counter;
};

static unsigned int r0p2_or_older_core;
enum { CCNT, PMN0, PMN1, MAX_PMUCOUNTERS };
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
	 .name = "arm/arm11",
	 .num_counters = MAX_L2COUNTERS,
	 .int_mask = {[PMN0] = 0x10,[PMN1] = 0x20,
		      [CCNT] = 0x40,[EMC0] = 0x800,[EMC1] = 0x400,[EMC2] =
		      0x200,[EMC3] = 0x100},
	 .cnt_ovf = {[CCNT] = 0x400,[PMN0] = 0x100,
		     [PMN1] = 0x200,[EMC0] = 0x1,[EMC1] = 0x2,[EMC2] =
		     0x4,[EMC3] = 0x8},
	 },
};

static struct pmu_type *pmu;

extern void l2_evtbus_enable(void);
extern void l2_evtbus_disable(void);

/*!
 * function is used to write the EVTMON counter configuration register.
 */
static int l2em_configure_counter(int nr, int type)
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
static void write_l2counter(int nr, u32 val)
{
	__raw_writel(val, L2EM_CNT(nr));
}

/*!
 * function is used to write the control register for the ARM11 performance counters
 */
static void write_pmnc(u32 val)
{
	pr_debug("PMC value written is %#08x\n", val);
	__asm__ __volatile__("mcr p15, 0, %0, c15, c12, 0"::"r"(val));
}

/*!
 * function is used to read the control register for the ARM11 performance counters
 */
static u32 read_pmnc(void)
{
	u32 val;
	pr_debug("In function %s\n", __FUNCTION__);
	__asm__ __volatile__("mrc p15, 0, %0, c15, c12, 0":"=r"(val));
	pr_debug("PMC value read is %#08x\n", val);
	return val;
}

/*!
 * function is used to read the ARM11 performance counters
 */
static u32 read_counter(int counter)
{
	u32 val = 0;
	pr_debug("In function %s\n", __FUNCTION__);

	switch (counter) {
	case CCNT:
	      __asm__ __volatile__("mrc p15, 0, %0, c15, c12, 1":"=r"(val));
		break;
	case PMN0:
	      __asm__ __volatile__("mrc p15, 0, %0, c15, c12, 2":"=r"(val));
		break;
	case PMN1:
	      __asm__ __volatile__("mrc p15, 0, %0, c15, c12, 3":"=r"(val));
		break;
	}

	pr_debug("counter %d value read is %#08x\n", counter, val);
	return val;
}

/*!
 * function is used to write to the ARM11 performance counters
 */
static void write_counter(int counter, u32 val)
{
	pr_debug("counter %d value written is %#08x\n", counter, val);

	switch (counter) {
	case CCNT:
	      __asm__ __volatile__("mcr p15, 0, %0, c15, c12, 1": :"r"(val));
		break;
	case PMN0:
	      __asm__ __volatile__("mcr p15, 0, %0, c15, c12, 2": :"r"(val));
		break;
	case PMN1:
	      __asm__ __volatile__("mcr p15, 0, %0, c15, c12, 3": :"r"(val));
		break;
	}
}

/*!
 * function is used to check the status of the ARM11 performance counters
 */
static int arm11_setup_ctrs(void)
{
	u32 pmnc = 0;
	int i;

	for (i = CCNT; i < MAX_L2COUNTERS; i++) {
		if (counter_config[i].enabled)
			continue;
		counter_config[i].event = EVT_UNUSED;
	}

	if (counter_config[PMN0].enabled)
		pmnc |= (counter_config[PMN0].event << 20);

	if (counter_config[PMN1].enabled)
		pmnc |= (counter_config[PMN1].event << 12);

	pr_debug("arm11_setup_ctrs: pmnc: %#08x\n", pmnc);
	write_pmnc(pmnc);

	for (i = CCNT; i < MAX_L2COUNTERS; i++) {
		if (counter_config[i].event == EVT_UNUSED) {
			counter_config[i].event = 0;
			pmu->int_enable &= ~pmu->int_mask[i];
			pr_debug
			    ("arm11_setup_ctrs: The counter event is %d for counter%d\n",
			     counter_config[i].event, i);
			continue;
		}

		results[i].reset_counter = counter_config[i].count;
		if (i < MAX_PMUCOUNTERS)
			write_counter(i, -(u32) counter_config[i].count);
		else {
			write_l2counter(i - EMC0,
					-(u32) counter_config[i].count);
			l2em_configure_counter(i - EMC0,
					       counter_config[i].event);
		}
		pmu->int_enable |= pmu->int_mask[i];
		pr_debug
		    ("arm11_setup_ctrs: The values of int mask and enables are %x, %x\n",
		     pmu->int_mask[i], pmu->int_enable);

		pr_debug("arm11_setup_ctrs: counter%d %#08x from %#08lx\n", i,
			 read_counter(i), counter_config[i].count);
	}

	return 0;
}

/*!
 * function is the interrupt service handler for the ARM11 performance counters
 */
static irqreturn_t arm11_pmu_interrupt(int irq, void *arg, struct pt_regs *regs)
{
	int i;
	u32 pmnc, emcs;

	/* Disable L2_EVTMON */
	emcs = __raw_readl(L2EM_STAT);
	__raw_writel((__raw_readl(L2EM_CTRL) & ~EVTMON_ENABLE), L2EM_CTRL);

	/* Disable ARM11 PMU while retaining interrupts and overflow bits */
	pmnc = read_pmnc();
	pmnc &= ~(PMU_ENABLE | PMU_OVERFLOWBIT_MASK);
	write_pmnc(pmnc);

	/* Read the overflow flag bits */
	pmnc = read_pmnc();

#ifdef ECT_WORKAROUND
	for (i = CCNT; i < MAX_PMUCOUNTERS; i++) {
#else
	for (i = CCNT; i < MAX_L2COUNTERS; i++) {
#endif
		/* Process the counters only if respective overflow interrupt is enabled */
		if (!(pmu->int_mask[i] & pmu->int_enable)) {
			continue;
		}

		/* As per ARM11 errata ARM11 cores with revision less than or equal to R0P2
		 * have known bug i.e., missing overflow interrupt for two events(event
		 * no 0x7 and 0x22) due to double increament for cycle. In this case we will
		 * discard the sample as the pc value belongs to different interrupt.
		 */
		if (r0p2_or_older_core && !(pmnc & pmu->cnt_ovf[i])
		    && !(read_counter(i) & COUNTER_MSB)) {
			write_counter(i, -(u32) (results[i].reset_counter));
		}

		/* Check for the overflowed counter by checking set overflow flag bits */
		if (!(pmnc & pmu->cnt_ovf[i]) && !(emcs & pmu->cnt_ovf[i])) {
			continue;
		}

		/* Reload the overflowed counter with preset value and
		 * add the sample for respective event.
		 */
		pr_debug("arm11_pmu_interrupt: writing to file\n");
		if (i < MAX_PMUCOUNTERS)
			write_counter(i, -(u32) results[i].reset_counter);
		else
			write_l2counter(i - EMC0,
					-(u32) counter_config[i].count);

		oprofile_add_sample(regs, i);
	}

	/* Clear overflow flags */
	write_pmnc(pmnc);

#ifdef ECT_WORKAROUND
	/*
	 * If ECTTRIGOUT signal is interrupt it should be acknowledged
	 * until trigger is off.
	 */
	while (__raw_readl(ECT_CTI_TRIGOUTSTATUS) & ECT_CTI_CHAN_2)
		__raw_writel(ACK_TRIG_OUT_2, ECT_CTI_INTACK);
#endif

	/* Re-enable ARM11 PMU */
	pmnc |= PMU_ENABLE;
	write_pmnc(pmnc);

	/* Re-enable L2_EVTMON */
	__raw_writel((__raw_readl(L2EM_CTRL) | L2EM_ENABLE_MASK), L2EM_CTRL);

	return IRQ_HANDLED;
}

/*!
 * function used to start the ARM11 performance counters
 */
static void arm11_pmu_stop(void)
{
	u32 pmnc = read_pmnc();

	pmnc &= ~PMU_ENABLE;
	write_pmnc(pmnc);
	/* Disable the EVTMON */
	__raw_writel((__raw_readl(L2EM_CTRL) & ~EVTMON_ENABLE), L2EM_CTRL);

	/* Disable the EVTBUS */
	l2_evtbus_disable();

	free_irq(EVENT_OVERFLOW_INT, results);
}

/*!
 * function used to start the ARM11 performance counters
 */
static int arm11_pmu_start(void)
{
	int ret;
	u32 pmnc = read_pmnc();

	ret = request_irq(EVENT_OVERFLOW_INT, arm11_pmu_interrupt, SA_INTERRUPT,
			  "ARM11 PMU", (void *)results);
	pr_debug("requested IRQ\n");

	if (ret < 0) {
		printk(KERN_ERR
		       "oprofile: unable to request IRQ%d for ARM11 PMU\n",
		       ARM11_PMU_IRQ);
		return ret;
	}

	/* Enable the EVTBUS */
	l2_evtbus_enable();

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
	pmnc |= pmu->int_enable;
	pmnc |= PMU_ENABLE;

	write_pmnc(pmnc);
	pr_debug("arm11_pmu_start: pmnc: %#08x mask: %08x\n", pmnc,
		 pmu->int_enable);

	/* Enable EVTMON with Edge triggered interrupt of one Clock Cycle */
	__raw_writel((__raw_readl(L2EM_CTRL) |
		      (L2EM_INT_EDGE | L2EM_INT_CLK_CYCLES)), L2EM_CTRL);
	__raw_writel((__raw_readl(L2EM_CTRL) | L2EM_ENABLE_MASK), L2EM_CTRL);

	return 0;
}

/*!
 * function detect the ARM11 performance counters
 */
static int arm11_detect_pmu(void)
{
	int ret = 0;
	u32 id, rev;

	id = (read_cpuid(CPUID_ID) >> 0x10) & 0xF;

	switch (id) {
	case 7:
		pmu = &pmu_parms[0];
		rev = read_cpuid(CPUID_ID);
		/* Check if the ARM11 core is less than or equal to R0P2 */
		if ((((rev >> 0x14) & 0xF) == VAR_NUM)
		    && (((rev & 0xF) <= REV_NUM))) {
			r0p2_or_older_core = 1;
		}
		break;
	default:
		ret = -ENODEV;
		break;
	}

	if (!ret) {
		op_arm_spec.name = pmu->name;
		op_arm_spec.num_counters = pmu->num_counters;
		pr_debug("arm11_detect_pmu: detected %s PMU\n", pmu->name);
	}

	return ret;
}

struct op_arm_model_spec op_arm_spec = {
	.init = arm11_detect_pmu,
	.setup_ctrs = arm11_setup_ctrs,
	.start = arm11_pmu_start,
	.stop = arm11_pmu_stop,
};
