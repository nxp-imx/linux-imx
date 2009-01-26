
/*
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file arch-mxc/mxc_gpc.h
 *
 * @brief This file contains the  chip level configuration details and
 * public API declarations for GPC module
 *
 * @ingroup LPMD
 */

#ifndef __ASM_ARCH_MXC_GPC_H__
#define __ASM_ARCH_MXC_GPC_H__

/* AP Power Gating modules */
typedef enum {
	POWER_GATING_MODULE_AP_EMBEDDED_MEM_DEEPSLEEP,
	POWER_GATING_MODULE_DISPLAY_BUFFER,
	POWER_GATING_MODULE_EMI_DEEPSLEEP,
	POWER_GATING_MODULE_IPU_STOP,
	POWER_GATING_MODULE_L2_MEM_STOP,
	POWER_GATING_MODULE_ARM_PLATFORM_STOP,
} mxc_pm_ap_power_gating_modules_t;

/* AP Power Gating pull-down config of modules */
typedef enum {
	POWER_GATING_PULL_DOWN_DISPLAY_BUFFER,
	POWER_GATING_PULL_DOWN_EMI,
	POWER_GATING_PULL_DOWN_IPU,
	POWER_GATING_PULL_DOWN_L2_MEM,
	POWER_GATING_PULL_DOWN_ARMPLATFORM,
} mxc_pm_ap_power_gating_pulldown_t;

/*!
 * This function enables/disables the AP power gating by writing the APPCR
 * register of the GPC module.
 *
 * @param   enable  Enable/Disable module power down
 *                  0 - disable; 1 - enable
 * @param   modules The desired module to be power gated
 *
 */
void mxc_gpc_powergate_module(int enable,
			      mxc_pm_ap_power_gating_modules_t module);

/*!
 * This function enables/disables the AP power gating pull down selection of a
 * module by writing the APPCR register of the GPC module.
 *
 * @param   enable  Enable/Disable module pull down
 *                  0 - disable; 1 - enable
 * @param   modules The desired module to be pulled down
 *
 */
void mxc_gpc_powergate_pulldown(int enable,
				mxc_pm_ap_power_gating_pulldown_t pulldown);

#endif
