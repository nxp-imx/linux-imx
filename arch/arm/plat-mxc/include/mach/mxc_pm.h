
/*
 * Copyright 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @defgroup LPMD Low-Level Power Management Driver
 */

/*!
 * @file arch-mxc/mxc_pm.h
 *
 * @brief This file contains the  chip level configuration details and
 * public API declarations for CRM_AP module
 *
 * @ingroup LPMD
 */

#ifndef __ASM_ARCH_MXC_PM_H__
#define __ASM_ARCH_MXC_PM_H__

#define WAIT_MODE               111
#define DOZE_MODE               112
#define STOP_MODE               113
#define DSM_MODE                114
/*
 * MXC91231 Break-Point Frequency below which is low frequency and
 * above which is high frequency
 */
#define BREAKPT_FREQ            ((long)(400000000))

#define GATE_STOP_WAIT          9
#define GATE_STOP               10

/*
 * Used for MHz conversion
 */
#define MEGA_HERTZ              1000000

/*
 * If invalid frequency value other than the following
 * CORE_133 - ARM desired to run @133MHz, LoV (1.2V)
 * CORE_266 - ARM desired to run @266MHz, LoV (1.2V)
 * CORE_399 - ARM desired to run @399MHz, LoV (1.2V)
 * CORE_532 - ARM desired to run @133MHz, HiV (1.6V)
 * are passed then this error is returned,
 */
#define ERR_FREQ_INVALID          1

/*
 * For MXC91231 Pass1, Integer DVFS greater than 133MHz is not allowed
 * due to the hardware issue
 */
#define INTEGER_DVFS_NOT_ALLOW  1

/*
 * If PLL freq is less than desired ARM frequency during Integer
 * DVFS, then return this error
 */
#define PLL_LESS_ARM_ERR        2

/*
 * Frequency change within the same-lo voltage is not approved.
 * Inorder to do Integer DFS, move to the high voltage range and
 * then set LFDF and move to the low voltage range
 */
#define INT_DFS_LOW_NOT_ALLOW   3

/*
 * If the desired AHB or IPG exceeds 133MHz or 66.5MHz respectively,
 * then return this error
 */
#define AHB_IPG_EXCEED_LIMIT    4

/*
 * If the desired ARM frequency is too low to get by PLL scaling
 * and the mxc_pm_pllscale API is called, return this error:
 */
#define PLL_DVFS_FREQ_TOO_LOW   5

/*
 * Invalid frequencies requested
 */
#define MXC_PM_INVALID_PARAM    6

/*
 * If AHB and/or IPG frequencies are greater than maximum allowed
 */
#define FREQ_OUT_OF_RANGE       2

/*
 * If AHB and/or IPG frequencies are other than 100 or 50Mhz
 */
#define BUS_FREQ_INVALID        2

/*
 * If MAX_PDF is greater than max value (8) then return this error
 */
#define AHB_MAX_DIV_ERR         3

/*
 * If IPG_PDF is greater than max value (2) then return this error
 */
#define IPG_MAX_DIV_ERR         4

/*
 * If ARM freq is out of range i.e., less than 133 or greater than
 * 399 then return this error
 */
#define INVALID_ARM_FREQ        5

/*
 * This file includes all platform APIs. Some of the APIs are not
 * appicable to some platforms. So, this error is used to indicate
 * that a particular API is not available
 */
#define MXC_PM_API_NOT_SUPPORTED	6

/*
 * Error when frequency scaling is attempted while switch between MPLL and
 * TPLL is in progress on MXC91321
 */
#define ERR_DFSP_SWITCH            2

/*!
 * Additional define for stop mode
 */
#define PM_SUSPEND_STOP         ((__force suspend_state_t) 2)

/*!
 * CKOH pins configuration
 */
#define CKOH_AP_SEL             1
#define CKOH_AHB_SEL            2
#define CKOH_IP_SEL             3

/*!
 * Defines for Stop and DSM mode acknowledgements
 */
#define MXC_PM_LOWPWR_ACK_SDMA  0x01
#define MXC_PM_LOWPWR_ACK_IPU   0x02
#define MXC_PM_LOWPWR_ACK_MAX   0x04
#define MXC_PM_LOWPWR_ACK_MQSPI 0x08
#define MXC_PM_LOWPWR_ACK_USB   0x10
#define MXC_PM_LOWPWR_ACK_RTIC  0x20

/*
 * PMIC configuration
 */
#define MXC_PMIC_1_2_VOLT                      0xC
#define MXC_PMIC_1_6_VOLT                      0x1C
#define MXC_PMIC_1_0_VOLT                      0x4
#if defined(CONFIG_ARCH_MXC91321) || defined(CONFIG_ARCH_MXC91231)
#define MXC_PMIC_DVS_SPEED                     0x1
#else
#define MXC_PMIC_DVS_SPEED                     0x3
#endif

/*!
 * Implementing Level 1 CRM Gate Control. Level 2 gate control
 * is provided at module level using LPMD registers
 *
 * @param   group   The desired clock gate control register bits.
 *                  Possible values are 0 through 6
 * @param   opt     The desired option requesting clock to run during stop
 *                  and wait modes or just during the stop mode. Possible
 *                  values are GATE_STOP_WAIT and GATE_STOP.
 *
 */
void mxc_pm_clockgate(int group, int opt);

/*!
 * Implementing steps required to transition to low-power modes
 *
 * @param   mode    The desired low-power mode. Possible values are,
 *                  WAIT_MODE, STOP_MODE or DSM_MODE
 *
 */
void mxc_pm_lowpower(int mode);

/*!
 * Enables acknowledgement from module when entering stop or DSM mode.
 *
 * @param   ack     The desired module acknowledgement to enable.
 *
 */
void mxc_pm_lp_ack_enable(int ack);

/*!
 * Disables acknowledgement from module when entering stop or DSM mode.
 *
 * @param   ack     The desired module acknowledgement to disable.
 *
 */
void mxc_pm_lp_ack_disable(int ack);

/*!
 * Implementing steps required to set Integer Scaling
 *
 * @param   armfreq    The desired ARM frequency. AHB and IP
 *                     frequency are changed depending on ARM
 *                     frequency and the divider values.
 * @param   ahbfreq    The desired AHB frequency
 * @param   ipfreq     The desired IP frequency
 *
 * @return             Returns 0 on success or
 *                     Returns -PLL_LESS_ARM_ERR if pllfreq is less than
 *                     desired core freq
 */
int mxc_pm_intscale(long armfreq, long ahbfreq, long ipfreq);

/*!
 * To calculate MFI, MFN, MFD values. Using this the output frequency
 * whose value is calculated using,
 * 2 * REF_FREQ * (MF / PDF), where
 * REF_FREQ is 26 Mhz
 * MF = MFI + (MFN + MFD)
 * PDF is assumed to be 1
 *
 * @param   armfreq    The desired ARM frequency
 * @param   ahbfreq    The desired AHB frequency
 * @param   ipfreq     The desired IP frequency
 *
 * @return             Returns 0 on success or
 *                     Returns -1 on error
 */
int mxc_pm_pllscale(long armfreq, long ahbfreq, long ipfreq);

/*!
 * To change AP core frequency and/or voltage suitably
 *
 * @param   armfreq    The desired ARM frequency
 * @param   ahbfreq    The desired AHB frequency
 * @param   ipfreq     The desired IP frequency
 *
 * @return             Returns -ERR_FREQ_INVALID on failure
 *                     Returns 0 on success
 */
int mxc_pm_dvfs(unsigned long armfreq, long ahbfreq, long ipfreq);

extern void mxc_pm_arch_entry(void *entry, u32 size);

#endif
