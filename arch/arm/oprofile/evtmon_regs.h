/*
 * Copyright 2005-2007 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file evtmon_regs.h
 *
 * @brief EVTMON Register definitions
 *
 * @ingroup MXC_Oprofile
 */
#ifndef _EVTMON_REGS_INCLUDED_
#define _EVTMON_REGS_INCLUDED_

#define MXC_CRM_AP_BASE		(IO_ADDRESS(CRM_AP_BASE_ADDR))
#define ECT_CTI_BASE		(IO_ADDRESS(ECT_CTIO_BASE_ADDR))
#define CLKCTR_BASE_ADDR	(IO_ADDRESS(CLKCTL_BASE_ADDR))
#define L2CC_BASE_ADDRESS       (IO_ADDRESS(L2CC_BASE_ADDR))
#define L2EM_BASE_ADDRESS       (IO_ADDRESS(EVTMON_BASE_ADDR))

/* L2 INT settings */
#define L2EM_ENABLE_OVERFLOWINT	0x1
#define L2EM_ENABLE_CNTINCRINT	0x3
#define L2EM_ENABLE_MASK	0x1
#define L2EM_INT_EDGE		0x2
#define L2EM_INT_HIGH		0x4
#define L2EM_INT_CLK_CYCLES	(0x0 << 3)

/* Reg definitions for EVTMON */
#define L2EM_CTRL           (L2EM_BASE_ADDRESS + 0x0)
#define L2EM_STAT           (L2EM_BASE_ADDRESS + 0x4)
#define L2EM_CC(nr)         (L2EM_BASE_ADDRESS + 0x8 +(4*nr))
#define L2EM_CNT(nr)        (L2EM_BASE_ADDRESS + 0x20 +(4*nr))

/* Reg definitions for CLK_CTL */
#define CLKCTL_SET_CTRL    (CLKCTR_BASE_ADDR + 0x04)

/* Reg definitions for ECT */
#define ECT_CTI_CONTROL   (ECT_CTI_BASE + 0x0)
#define ECT_CTI_LOCK      (ECT_CTI_BASE + 0x8)
#define ECT_CTI_INEN(nr)  (ECT_CTI_BASE + 0x20 + (4*nr))
#define ECT_CTI_OUTEN(nr) (ECT_CTI_BASE + 0xA0 + (4*nr))
#define ECT_CTI_INTACK    (ECT_CTI_BASE + 0x10)
#define ECT_CTI_TRIGOUTSTATUS    (ECT_CTI_BASE + 0x134)

#define ENABLE_L2CACHE		0x1
#define EVTMON_ENABLE		0x001	/* Enable EVTMON */
#define EVT_UNUSED		0x100
#define MAX_PMUCOUNTERS         3

#ifdef CONFIG_OPROFILE_ARM11_EVTMON
#define ECT_WORKAROUND
#endif

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
#define EVENT_OVERFLOW_INT	MXC_INT_ECT
#else
#define EVENT_OVERFLOW_INT	ARM11_PMU_IRQ
#define EM_SET_INT		L2EM_ENABLE_OVERFLOWINT
#endif

int l2em_configure_counter(int nr, int type);
void write_l2counter(int nr, u32 val);
#endif
