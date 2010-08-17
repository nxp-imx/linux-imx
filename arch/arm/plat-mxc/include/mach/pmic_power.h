/*
 * Copyright 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU Lesser General
 * Public License.  You may obtain a copy of the GNU Lesser General
 * Public License Version 2.1 or later at the following locations:
 *
 * http://www.opensource.org/licenses/lgpl-license.html
 * http://www.gnu.org/copyleft/lgpl.html
 */
#ifndef __ASM_ARCH_MXC_PMIC_POWER_H__
#define __ASM_ARCH_MXC_PMIC_POWER_H__

/*!
 * @defgroup PMIC_POWER  PMIC Power Driver
 * @ingroup PMIC_DRVRS
 */

/*!
 * @file arch-mxc/pmic_power.h
 * @brief This is the header of PMIC power driver.
 *
 * @ingroup PMIC_POWER
 */

#include <linux/ioctl.h>
#include <linux/pmic_status.h>
#include <linux/pmic_external.h>

/*!
 * @name IOCTL user space interface
 */
/*! @{ */

/*!
 * Turn on a regulator.
 */
#define PMIC_REGULATOR_ON			_IOWR('p', 0xf0, int)

/*!
 * Turn off a regulator.
 */
#define PMIC_REGULATOR_OFF        	 	_IOWR('p', 0xf1, int)

/*!
 * Set regulator configuration.
 */
#define PMIC_REGULATOR_SET_CONFIG         	_IOWR('p', 0xf2, int)

/*!
 * Get regulator configuration.
 */
#define PMIC_REGULATOR_GET_CONFIG         	_IOWR('p', 0xf3, int)

/*!
 * Miscellaneous Power Test.
 */
#define PMIC_POWER_CHECK_MISC			_IOWR('p', 0xf4, int)

/*! @} */

/*!
 * This enumeration define all power interrupts
 */
typedef enum {
	/*!
	 * BP turn on threshold detection
	 */
	PWR_IT_BPONI = 0,
	/*!
	 * End of life / low battery detect
	 */
	PWR_IT_LOBATLI,
	/*!
	 * Low battery warning
	 */
	PWR_IT_LOBATHI,
	/*!
	 * ON1B event
	 */
	PWR_IT_ONOFD1I,
	/*!
	 * ON2B event
	 */
	PWR_IT_ONOFD2I,
	/*!
	 * ON3B event
	 */
	PWR_IT_ONOFD3I,
	/*!
	 * System reset
	 */
	PWR_IT_SYSRSTI,
	/*!
	 * Power ready
	 */
	PWR_IT_PWRRDYI,
	/*!
	 * Power cut event
	 */
	PWR_IT_PCI,
	/*!
	 * Warm start event
	 */
	PWR_IT_WARMI,
	/*!
	 * Memory hold event
	 */
} t_pwr_int;

/*!
 * VHOLD regulator output voltage setting.
 */
typedef enum {
	VH_1_875V,		/*!< 1.875V */
	VH_2_5V,		/*!< 2.5V */
	VH_1_55V,		/*!< 1.55V */
	VH_PASSTHROUGH,		/*!< Pass-through mode */
} t_vhold_voltage;

/*!
 * PMIC power control configuration.
 */

typedef struct {
	bool pc_enable;		/*!< Power cut enable */
	unsigned char pc_timer;	/*!< Power cut timer value */
	bool pc_count_enable;	/*!< Power cut counter enable,
				   If TURE, Power cuts are disabled
				   when pc_count > pc_max_count;
				   If FALSE, Power cuts are not
				   disabled when
				   pc_count > pc_max_count */
	unsigned char pc_count;	/*!< Power cut count */
	unsigned char pc_max_count;	/*!< Power cut maximum count */
	bool warm_enable;	/*!< User Off state enable */
	bool user_off_pc;	/*!< Automatic transition to user off
				   during power cut */
	bool clk_32k_enable;	/*!< 32 kHz output buffer enable
				   during memory hold */
	bool clk_32k_user_off;	/*!< Keeps the CLK32KMCU active during
				   user off power cut modes */
	bool en_vbkup1;		/*!< enable VBKUP1 regulator */
	bool auto_en_vbkup1;	/*!< automatically enable VBKUP1
				   regulator in the memory hold
				   and user of modes */
	t_vhold_voltage vhold_voltage;	/*!< output voltage for VBKUP1 */
	bool en_vbkup2;		/*!< enable VBKUP2 regulator */
	bool auto_en_vbkup2;	/*!< automatically enable VBKUP2
				   regulator in the memory hold
				   and user of modes */
	t_vhold_voltage vhold_voltage2;	/*!< output voltage for VBKUP2 */
	unsigned char mem_timer;	/*!< duration of the memory hold
					   timer */
	bool mem_allon;		/*!< memory hold timer infinity mode,
				   If TRUE, the memory hold timer
				   will be set to infinity and
				   the mem_timer filed will be
				   ignored */
} t_pc_config;

/*!
 * brief PMIC regulators.
 */

typedef enum {
	SW_SW1A = 0,		/*!< SW1A or SW1 */
	SW_SW1B,		/*!< SW1B */
	SW_SW2A,		/*!< SW2A or SW2 */
	SW_SW2B,		/*!< SW2B */
	SW_SW3,			/*!< SW3 */
	SW_PLL,			/*!< PLL */
	REGU_VAUDIO,		/*!< VAUDIO */
	REGU_VIOHI,		/*!< VIOHI */
	REGU_VIOLO,		/*!< VIOLO */
	REGU_VDIG,		/*!< VDIG */
	REGU_VGEN,		/*!< VGEN */
	REGU_VRFDIG,		/*!< VRFDIG */
	REGU_VRFREF,		/*!< VRFREF */
	REGU_VRFCP,		/*!< VRFCP */
	REGU_VSIM,		/*!< VSIM */
	REGU_VESIM,		/*!< VESIM */
	REGU_VCAM,		/*!< VCAM */
	REGU_VRFBG,		/*!< VRFBG */
	REGU_VVIB,		/*!< VVIB */
	REGU_VRF1,		/*!< VRF1 */
	REGU_VRF2,		/*!< VRF2 */
	REGU_VMMC1,		/*!< VMMC1 or VMMC */
	REGU_VMMC2,		/*!< VMMC2 */
	REGU_GPO1,		/*!< GPIO1 */
	REGU_GPO2,		/*!< GPO2 */
	REGU_GPO3,		/*!< GPO3 */
	REGU_GPO4,		/*!< GPO4 */
	REGU_V1,		/*!< V1 */
	REGU_V2,		/*!< V2 */
	REGU_V3,		/*!< V3 */
	REGU_V4,		/*!< V4 */
} t_pmic_regulator;

/*!
 * @enum t_pmic_regulator_voltage_sw1
 * @brief PMIC Switch mode regulator SW1 output voltages.
 */

typedef enum {
	SW1_1V = 0,		/*!< 1.0 V */
	SW1_1_1V,		/*!< 1.1 V */
	SW1_1_2V,		/*!< 1.2 V */
	SW1_1_3V,		/*!< 1.3 V */
	SW1_1_4V,		/*!< 1.4 V */
	SW1_1_55V,		/*!< 1.55 V */
	SW1_1_625V,		/*!< 1.625 V */
	SW1_1_875V,		/*!< 1.875 V */
} t_pmic_regulator_voltage_sw1;

/*!
 * @enum t_pmic_regulator_voltage_sw1a
 * @brief PMIC regulator SW1A output voltage.
 */
typedef enum {
	SW1A_0_9V = 0,		/*!< 0.900 V */
	SW1A_0_925V,		/*!< 0.925 V */
	SW1A_0_95V,		/*!< 0.950 V */
	SW1A_0_975V,		/*!< 0.975 V */
	SW1A_1V,		/*!< 1.000 V */
	SW1A_1_025V,		/*!< 1.025 V */
	SW1A_1_05V,		/*!< 1.050 V */
	SW1A_1_075V,		/*!< 1.075 V */
	SW1A_1_1V,		/*!< 1.100 V */
	SW1A_1_125V,		/*!< 1.125 V */
	SW1A_1_15V,		/*!< 1.150 V */
	SW1A_1_175V,		/*!< 1.175 V */
	SW1A_1_2V,		/*!< 1.200 V */
	SW1A_1_225V,		/*!< 1.225 V */
	SW1A_1_25V,		/*!< 1.250 V */
	SW1A_1_275V,		/*!< 1.275 V */
	SW1A_1_3V,		/*!< 1.300 V */
	SW1A_1_325V,		/*!< 1.325 V */
	SW1A_1_35V,		/*!< 1.350 V */
	SW1A_1_375V,		/*!< 1.375 V */
	SW1A_1_4V,		/*!< 1.400 V */
	SW1A_1_425V,		/*!< 1.425 V */
	SW1A_1_45V,		/*!< 1.450 V */
	SW1A_1_475V,		/*!< 1.475 V */
	SW1A_1_5V,		/*!< 1.500 V */
	SW1A_1_525V,		/*!< 1.525 V */
	SW1A_1_55V,		/*!< 1.550 V */
	SW1A_1_575V,		/*!< 1.575 V */
	SW1A_1_6V,		/*!< 1.600 V */
	SW1A_1_625V,		/*!< 1.625 V */
	SW1A_1_65V,		/*!< 1.650 V */
	SW1A_1_675V,		/*!< 1.675 V */
	SW1A_1_7V,		/*!< 1.700 V */
	SW1A_1_8V = 36,		/*!< 1.800 V */
	SW1A_1_85V = 40,	/*!< 1.850 V */
	SW1A_2V = 44,		/*!< 2_000 V */
	SW1A_2_1V = 48,		/*!< 2_100 V */
	SW1A_2_2V = 52,		/*!< 2_200 V */
} t_pmic_regulator_voltage_sw1a;

/*!
 * @enum t_pmic_regulator_voltage_sw1b
 * @brief PMIC regulator SW1B output voltage.
 */
typedef enum {
	SW1B_0_9V = 0,		/*!< 0.900 V */
	SW1B_0_925V,		/*!< 0.925 V */
	SW1B_0_95V,		/*!< 0.950 V */
	SW1B_0_975V,		/*!< 0.975 V */
	SW1B_1V,		/*!< 1.000 V */
	SW1B_1_025V,		/*!< 1.025 V */
	SW1B_1_05V,		/*!< 1.050 V */
	SW1B_1_075V,		/*!< 1.075 V */
	SW1B_1_1V,		/*!< 1.100 V */
	SW1B_1_125V,		/*!< 1.125 V */
	SW1B_1_15V,		/*!< 1.150 V */
	SW1B_1_175V,		/*!< 1.175 V */
	SW1B_1_2V,		/*!< 1.200 V */
	SW1B_1_225V,		/*!< 1.225 V */
	SW1B_1_25V,		/*!< 1.250 V */
	SW1B_1_275V,		/*!< 1.275 V */
	SW1B_1_3V,		/*!< 1.300 V */
	SW1B_1_325V,		/*!< 1.325 V */
	SW1B_1_35V,		/*!< 1.350 V */
	SW1B_1_375V,		/*!< 1.375 V */
	SW1B_1_4V,		/*!< 1.400 V */
	SW1B_1_425V,		/*!< 1.425 V */
	SW1B_1_45V,		/*!< 1.450 V */
	SW1B_1_475V,		/*!< 1.475 V */
	SW1B_1_5V,		/*!< 1.500 V */
	SW1B_1_525V,		/*!< 1.525 V */
	SW1B_1_55V,		/*!< 1.550 V */
	SW1B_1_575V,		/*!< 1.575 V */
	SW1B_1_6V,		/*!< 1.600 V */
	SW1B_1_625V,		/*!< 1.625 V */
	SW1B_1_65V,		/*!< 1.650 V */
	SW1B_1_675V,		/*!< 1.675 V */
	SW1B_1_7V,		/*!< 1.700 V */
	SW1B_1_8V = 36,		/*!< 1.800 V */
	SW1B_1_85V = 40,	/*!< 1.850 V */
	SW1B_2V = 44,		/*!< 2_000 V */
	SW1B_2_1V = 48,		/*!< 2_100 V */
	SW1B_2_2V = 52,		/*!< 2_200 V */
} t_pmic_regulator_voltage_sw1b;

/*!
 * @enum t_pmic_regulator_voltage_sw2
 * @brief PMIC Switch mode regulator SW2 output voltages.
  */
typedef enum {
	SW2_1V = 0,		/*!< 1.0 V */
	SW2_1_1V,		/*!< 1.1 V */
	SW2_1_2V,		/*!< 1.2 V */
	SW2_1_3V,		/*!< 1.3 V */
	SW2_1_4V,		/*!< 1.4 V */
	SW2_1_55V,		/*!< 1.55 V */
	SW2_1_625V,		/*!< 1.625 V */
	SW2_1_875V,		/*!< 1.875 V */
} t_pmic_regulator_voltage_sw2;

/*!
 * @enum t_pmic_regulator_voltage_sw2a
 * @brief PMIC regulator SW2A output voltage.
 */
typedef enum {
	SW2A_0_9V = 0,		/*!< 0.900 V */
	SW2A_0_925V,		/*!< 0.925 V */
	SW2A_0_95V,		/*!< 0.950 V */
	SW2A_0_975V,		/*!< 0.975 V */
	SW2A_1V,		/*!< 1.000 V */
	SW2A_1_025V,		/*!< 1.025 V */
	SW2A_1_05V,		/*!< 1.050 V */
	SW2A_1_075V,		/*!< 1.075 V */
	SW2A_1_1V,		/*!< 1.100 V */
	SW2A_1_125V,		/*!< 1.125 V */
	SW2A_1_15V,		/*!< 1.150 V */
	SW2A_1_175V,		/*!< 1.175 V */
	SW2A_1_2V,		/*!< 1.200 V */
	SW2A_1_225V,		/*!< 1.225 V */
	SW2A_1_25V,		/*!< 1.250 V */
	SW2A_1_275V,		/*!< 1.275 V */
	SW2A_1_3V,		/*!< 1.300 V */
	SW2A_1_325V,		/*!< 1.325 V */
	SW2A_1_35V,		/*!< 1.350 V */
	SW2A_1_375V,		/*!< 1.375 V */
	SW2A_1_4V,		/*!< 1.400 V */
	SW2A_1_425V,		/*!< 1.425 V */
	SW2A_1_45V,		/*!< 1.450 V */
	SW2A_1_475V,		/*!< 1.475 V */
	SW2A_1_5V,		/*!< 1.500 V */
	SW2A_1_525V,		/*!< 1.525 V */
	SW2A_1_55V,		/*!< 1.550 V */
	SW2A_1_575V,		/*!< 1.575 V */
	SW2A_1_6V,		/*!< 1.600 V */
	SW2A_1_625V,		/*!< 1.625 V */
	SW2A_1_65V,		/*!< 1.650 V */
	SW2A_1_675V,		/*!< 1.675 V */
	SW2A_1_7V,		/*!< 1.700 V */
	SW2A_1_8V = 36,		/*!< 1.800 V */
	SW2A_1_9V = 40,		/*!< 1.900 V */
	SW2A_2V = 44,		/*!< 2_000 V */
	SW2A_2_1V = 48,		/*!< 2_100 V */
	SW2A_2_2V = 52,		/*!< 2_200 V */
} t_pmic_regulator_voltage_sw2a;

/*!
 * @enum t_pmic_regulator_voltage_sw2b
 * @brief PMIC regulator SW2B output voltage.
 */
typedef enum {
	SW2B_0_9V = 0,		/*!< 0.900 V */
	SW2B_0_925V,		/*!< 0.925 V */
	SW2B_0_95V,		/*!< 0.950 V */
	SW2B_0_975V,		/*!< 0.975 V */
	SW2B_1V,		/*!< 1.000 V */
	SW2B_1_025V,		/*!< 1.025 V */
	SW2B_1_05V,		/*!< 1.050 V */
	SW2B_1_075V,		/*!< 1.075 V */
	SW2B_1_1V,		/*!< 1.100 V */
	SW2B_1_125V,		/*!< 1.125 V */
	SW2B_1_15V,		/*!< 1.150 V */
	SW2B_1_175V,		/*!< 1.175 V */
	SW2B_1_2V,		/*!< 1.200 V */
	SW2B_1_225V,		/*!< 1.225 V */
	SW2B_1_25V,		/*!< 1.250 V */
	SW2B_1_275V,		/*!< 1.275 V */
	SW2B_1_3V,		/*!< 1.300 V */
	SW2B_1_325V,		/*!< 1.325 V */
	SW2B_1_35V,		/*!< 1.350 V */
	SW2B_1_375V,		/*!< 1.375 V */
	SW2B_1_4V,		/*!< 1.400 V */
	SW2B_1_425V,		/*!< 1.425 V */
	SW2B_1_45V,		/*!< 1.450 V */
	SW2B_1_475V,		/*!< 1.475 V */
	SW2B_1_5V,		/*!< 1.500 V */
	SW2B_1_525V,		/*!< 1.525 V */
	SW2B_1_55V,		/*!< 1.550 V */
	SW2B_1_575V,		/*!< 1.575 V */
	SW2B_1_6V,		/*!< 1.600 V */
	SW2B_1_625V,		/*!< 1.625 V */
	SW2B_1_65V,		/*!< 1.650 V */
	SW2B_1_675V,		/*!< 1.675 V */
	SW2B_1_7V,		/*!< 1.700 V */
	SW2B_1_8V = 36,		/*!< 1.800 V */
	SW2B_1_9V = 40,		/*!< 1.900 V */
	SW2B_2V = 44,		/*!< 2_000 V */
	SW2B_2_1V = 48,		/*!< 2_100 V */
	SW2B_2_2V = 52,		/*!< 2_200 V */
} t_pmic_regulator_voltage_sw2b;

/*!
 * @enum t_pmic_regulator_voltage_sw3
 * @brief PMIC Switch mode regulator SW3 output voltages.
 */
typedef enum {
	SW3_5V = 0,		/*!< 5.0 V */
	SW3_5_1V = 0,		/*!< 5.1 V */
	SW3_5_6V,		/*!< 5.6 V */
} t_pmic_regulator_voltage_sw3;

/*!
 * @enum t_switcher_factor
 * @brief PLL multiplication factor
 */
typedef enum {
	FACTOR_28 = 0,		/*!< 917 504 kHz */
	FACTOR_29,		/*!< 950 272 kHz */
	FACTOR_30,		/*!< 983 040 kHz */
	FACTOR_31,		/*!< 1 015 808 kHz */
	FACTOR_32,		/*!< 1 048 576 kHz */
	FACTOR_33,		/*!< 1 081 344 kHz */
	FACTOR_34,		/*!< 1 114 112 kHz */
	FACTOR_35,		/*!< 1 146 880 kHz */
} t_switcher_factor;

/*!
 * @enum t_pmic_regulator_voltage_violo
 * @brief PMIC regulator VIOLO output voltage.
 */
typedef enum {
	VIOLO_1_2V = 0,		/*!< 1.2 V */
	VIOLO_1_3V,		/*!< 1.3 V */
	VIOLO_1_5V,		/*!< 1.5 V */
	VIOLO_1_8V,		/*!< 1.8 V */
} t_pmic_regulator_voltage_violo;

/*!
 * @enum t_pmic_regulator_voltage_vdig
 * @brief PMIC regulator VDIG output voltage.
 */
typedef enum {
	VDIG_1_2V = 0,		/*!< 1.2 V */
	VDIG_1_3V,		/*!< 1.3 V */
	VDIG_1_5V,		/*!< 1.5 V */
	VDIG_1_8V,		/*!< 1.8 V */
} t_pmic_regulator_voltage_vdig;

/*!
 * @enum t_pmic_regulator_voltage_vgen
 * @brief PMIC regulator VGEN output voltage.
 */
typedef enum {
	VGEN_1_2V = 0,		/*!< 1.2 V */
	VENG_1_3V,		/*!< 1.3 V */
	VGEN_1_5V,		/*!< 1.5 V */
	VGEN_1_8V,		/*!< 1.8 V */
	VGEN_1_1V,		/*!< 1.1 V */
	VGEN_2V,		/*!< 2 V */
	VGEN_2_775V,		/*!< 2.775 V */
	VGEN_2_4V,		/*!< 2.4 V */
} t_pmic_regulator_voltage_vgen;

/*!
 * @enum t_pmic_regulator_voltage_vrfdig
 * @brief PMIC regulator VRFDIG output voltage.
 */
typedef enum {
	VRFDIG_1_2V = 0,	/*!< 1.2 V */
	VRFDIG_1_5V,		/*!< 1.5 V */
	VRFDIG_1_8V,		/*!< 1.8 V */
	VRFDIG_1_875V,		/*!< 1.875 V */
} t_pmic_regulator_voltage_vrfdig;

/*!
 * @enum t_pmic_regulator_voltage_vrfref
 * @brief PMIC regulator VRFREF output voltage.
 */
typedef enum {
	VRFREF_2_475V = 0,	/*!< 2.475 V */
	VRFREF_2_6V,		/*!< 2.600 V */
	VRFREF_2_7V,		/*!< 2.700 V */
	VRFREF_2_775V,		/*!< 2.775 V */
} t_pmic_regulator_voltage_vrfref;

/*!
 * @enum t_pmic_regulator_voltage_vrfcp
 * @brief PMIC regulator VRFCP output voltage.
 */
typedef enum {
	VRFCP_2_7V = 0,		/*!< 2.700 V */
	VRFCP_2_775V,		/*!< 2.775 V */
} t_pmic_regulator_voltage_vrfcp;

/*!
 * @enum t_pmic_regulator_voltage_vsim
 * @brief PMIC linear regulator VSIM output voltage.
 */
typedef enum {
	VSIM_1_8V = 0,		/*!< 1.8 V */
	VSIM_2_9V,		/*!< 2.90 V */
	VSIM_3V = 1,		/*!< 3 V */
} t_pmic_regulator_voltage_vsim;

/*!
 * @enum t_pmic_regulator_voltage_vesim
 * @brief PMIC regulator VESIM output voltage.
 */
typedef enum {
	VESIM_1_8V = 0,		/*!< 1.80 V */
	VESIM_2_9V,		/*!< 2.90 V */
} t_pmic_regulator_voltage_vesim;

/*!
 * @enum t_pmic_regulator_voltage_vcam
 * @brief PMIC regulator VCAM output voltage.
 */
typedef enum {
	VCAM_1_5V = 0,		/*!< 1.50 V */
	VCAM_1_8V,		/*!< 1.80 V */
	VCAM_2_5V,		/*!< 2.50 V */
	VCAM_2_55V,		/*!< 2.55 V */
	VCAM_2_6V,		/*!< 2.60 V */
	VCAM_2_75V,		/*!< 2.75 V */
	VCAM_2_8V,		/*!< 2.80 V */
	VCAM_3V,		/*!< 3.00 V */
} t_pmic_regulator_voltage_vcam;

/*!
 * @enum t_pmic_regulator_voltage_vvib
 * @brief PMIC linear regulator V_VIB output voltage.
 */
typedef enum {
	VVIB_1_3V = 0,		/*!< 1.30 V */
	VVIB_1_8V,		/*!< 1.80 V */
	VVIB_2V,		/*!< 2 V */
	VVIB_3V,		/*!< 3 V */
} t_pmic_regulator_voltage_vvib;

/*!
 * @enum t_pmic_regulator_voltage_vrf1
 * @brief PMIC regulator VRF1 output voltage.
 */
typedef enum {
	VRF1_1_5V = 0,		/*!< 1.500 V */
	VRF1_1_875V,		/*!< 1.875 V */
	VRF1_2_7V,		/*!< 2.700 V */
	VRF1_2_775V,		/*!< 2.775 V */
} t_pmic_regulator_voltage_vrf1;

/*!
 * @enum t_pmic_regulator_voltage_vrf2
 * @brief PMIC regulator VRF2 output voltage.
 */
typedef enum {
	VRF2_1_5V = 0,		/*!< 1.500 V */
	VRF2_1_875V,		/*!< 1.875 V */
	VRF2_2_7V,		/*!< 2.700 V */
	VRF2_2_775V,		/*!< 2.775 V */
} t_pmic_regulator_voltage_vrf2;

/*!
 * @enum t_pmic_regulator_voltage_vmmc
 * @brief PMIC linear regulator VMMC output voltage.
 */
typedef enum {
	VMMC_OFF = 0,		/*!< Output off */
	VMMC_1_6V,		/*!< 1.6 V */
	VMMC_1_8V,		/*!< 1.8 V */
	VMMC_2V,		/*!< 2 V */
	VMMC_2_2V,		/*!< 2.2 V */
	VMMC_2_4V,		/*!< 2.4 V */
	VMMC_2_6V,		/*!< 2.6 V */
	VMMC_2_8V,		/*!< 2.8 V */
	VMMC_3V,		/*!< 3 V */
	VMMC_3_2V,		/*!< 3.2 V */
	VMMC_3_3V,		/*!< 3.3 V */
	VMMC_3_4V,		/*!< 3.4 V */
} t_pmic_regulator_voltage_vmmc;

/*!
 * @enum t_pmic_regulator_voltage_vmmc1
 * @brief PMIC regulator VMMC1 output voltage.
 */
typedef enum {
	VMMC1_1_6V = 0,		/*!< 1.60 V */
	VMMC1_1_8V,		/*!< 1.80 V */
	VMMC1_2V,		/*!< 2.00 V */
	VMMC1_2_6V,		/*!< 2.60 V */
	VMMC1_2_7V,		/*!< 2.70 V */
	VMMC1_2_8V,		/*!< 2.80 V */
	VMMC1_2_9V,		/*!< 2.90 V */
	VMMC1_3V,		/*!< 3.00 V */
} t_pmic_regulator_voltage_vmmc1;

/*!
 * @enum t_pmic_regulator_voltage_vmmc2
 * @brief PMIC regulator VMMC2 output voltage.
 */
typedef enum {
	VMMC2_1_6V = 0,		/*!< 1.60 V */
	VMMC2_1_8V,		/*!< 1.80 V */
	VMMC2_2V,		/*!< 2.00 V */
	VMMC2_2_6V,		/*!< 2.60 V */
	VMMC2_2_7V,		/*!< 2.70 V */
	VMMC2_2_8V,		/*!< 2.80 V */
	VMMC2_2_9V,		/*!< 2.90 V */
	VMMC2_3V,		/*!< 3.00 V */
} t_pmic_regulator_voltage_vmmc2;

/*!
 * @enum t_pmic_regulator_voltage_v1
 * @brief PMIC linear regulator V1 output voltages.
 */
typedef enum {
	V1_2_775V = 0,		/*!< 2.775 V */
	V1_1_2V,		/*!< 1.2 V */
	V1_1_3V,		/*!< 1.3 V */
	V1_1_4V,		/*!< 1.4 V */
	V1_1_55V,		/*!< 1.55 V */
	V1_1_75V,		/*!< 1.75 V */
	V1_1_875V,		/*!< 1.875 V */
	V1_2_475V,		/*!< 2.475 V */
} t_pmic_regulator_voltage_v1;

/*!
 * @enum t_pmic_regulator_voltage_v2
 * @brief PMIC linear regulator V2 output voltage, V2 has fixed
 * output voltage 2.775 volts.
 */
typedef enum {
	V2_2_775V = 0,		/*!< 2.775 V */
} t_pmic_regulator_voltage_v2;

/*!
 * @enum t_pmic_regulator_voltage_v3
 * @brief PMIC linear regulator V3 output voltage.
 */
typedef enum {
	V3_1_875V = 0,		/*!< 1.875 V */
	V3_2_775V,		/*!< 2.775 V */
} t_pmic_regulator_voltage_v3;

/*!
 * @enum t_pmic_regulator_voltage_v4
 * @brief PMIC linear regulator V4 output voltage, V4 has fixed
 * output voltage 2.775 volts.
 */
typedef enum {
	V4_2_775V = 0,		/*!< 2.775 V */
} t_pmic_regulator_voltage_v4;

/*!
 * @union t_regulator_voltage
 * @brief PMIC regulator output voltages.
 */
typedef union {
	t_pmic_regulator_voltage_sw1 sw1;	/*!< SW1 voltage */
	t_pmic_regulator_voltage_sw1a sw1a;	/*!< SW1A voltage */
	t_pmic_regulator_voltage_sw1b sw1b;	/*!< SW1B voltage */
	t_pmic_regulator_voltage_sw2 sw2;	/*!< SW2 voltage */
	t_pmic_regulator_voltage_sw2a sw2a;	/*!< SW2A voltage */
	t_pmic_regulator_voltage_sw2b sw2b;	/*!< SW2B voltage */
	t_pmic_regulator_voltage_sw3 sw3;	/*!< SW3 voltage */
	t_pmic_regulator_voltage_violo violo;	/*!< VIOLO voltage */
	t_pmic_regulator_voltage_vdig vdig;	/*!< VDIG voltage */
	t_pmic_regulator_voltage_vgen vgen;	/*!< VGEN voltage */
	t_pmic_regulator_voltage_vrfdig vrfdig;	/*!< VRFDIG voltage */
	t_pmic_regulator_voltage_vrfref vrfref;	/*!< VRFREF voltage */
	t_pmic_regulator_voltage_vrfcp vrfcp;	/*!< VRFCP voltage */
	t_pmic_regulator_voltage_vsim vsim;	/*!< VSIM voltage */
	t_pmic_regulator_voltage_vesim vesim;	/*!< VESIM voltage */
	t_pmic_regulator_voltage_vcam vcam;	/*!< VCAM voltage */
	t_pmic_regulator_voltage_vvib vvib;	/*!< VVIB voltage */
	t_pmic_regulator_voltage_vrf1 vrf1;	/*!< VRF1 voltage */
	t_pmic_regulator_voltage_vrf2 vrf2;	/*!< VRF2 voltage */
	t_pmic_regulator_voltage_vmmc vmmc;	/*!< VMMC voltage */
	t_pmic_regulator_voltage_vmmc1 vmmc1;	/*!< VMMC1 voltage */
	t_pmic_regulator_voltage_vmmc2 vmmc2;	/*!< VMMC2 voltage */
	t_pmic_regulator_voltage_v1 v1;	/*!< V1 voltage */
	t_pmic_regulator_voltage_v2 v2;	/*!< V2 voltage */
	t_pmic_regulator_voltage_v3 v3;	/*!< V3 voltage */
	t_pmic_regulator_voltage_v4 v4;	/*!< V4 voltage */
} t_regulator_voltage;

/*!
 * @enum t_pmic_regulator_sw_mode
 * @brief define switch mode regulator mode.
 *
 * The synchronous rectifier can be disabled (and pulse-skipping enabled)
 * to improve low current efficiency. Software should disable synchronous
 * rectifier / enable the pulse skipping for average loads less than
 * approximately 30 mA, depending on the quiescent current penalty due to
 * synchronous mode.
 */
typedef enum {
	SYNC_RECT = 0,
	NO_PULSE_SKIP,
	PULSE_SKIP,
	LOW_POWER,
} t_pmic_regulator_sw_mode;

/*!
 * Generic PMIC switch mode regulator mode.
 */
typedef t_pmic_regulator_sw_mode t_regulator_sw_mode;
typedef t_pmic_regulator_sw_mode t_regulator_stby_mode;

/*!
 * @enum t_regulator_lp_mode
 * @brief Low power mode control modes.
 */

typedef enum {
	/*!
	 * Low Power Mode is disabled
	 */
	LOW_POWER_DISABLED = 0,
	/*!
	 * Low Power Mode is controlled by STANDBY pin and/or LVS pin
	 */
	LOW_POWER_CTRL_BY_PIN,
	/*!
	 * Set Low Power mode no matter of hardware pins
	 */
	LOW_POWER_EN,
	/*!
	 * Set Low Power mode and control by STANDBY
	 */
	LOW_POWER_AND_LOW_POWER_CTRL_BY_PIN,
} t_regulator_lp_mode;

/*!
 * @enum t_switcher_dvs_speed
 * @brief DVS speed setting
 */
typedef enum {
	/*!
	 * Transition speed is dictated by the current
	 * limit and input -output conditions
	 */
	DICTATED = 0,
	/*!
	 * 25mV step each 4us
	 */
	DVS_4US,
	/*!
	 * 25mV step each 8us
	 */
	DVS_8US,
	/*!
	 * 25mV step each 16us
	 */
	DVS_16US,
} t_switcher_dvs_speed;

/*!
 * @struct t_regulator_config
 * @brief regulator configuration.
 *
 */

typedef struct {
	/*!
	 * Switch mode regulator operation mode. This field only applies to
	 * switch mode regulators.
	 */
	t_regulator_sw_mode mode;
	/*!
	 * Switch mode stby regulator operation mode. This field only applies
	 * to switch mode regulators.
	 */
	t_regulator_stby_mode stby_mode;
	/*!
	 * Regulator output voltage.
	 */
	t_regulator_voltage voltage;
	/*!
	 * Regulator output voltage in LVS mode.
	 */
	t_regulator_voltage voltage_lvs;
	/*!
	 * Regulator output voltage in standby mode.
	 */
	t_regulator_voltage voltage_stby;
	/*!
	 * Regulator low power mode.
	 */
	t_regulator_lp_mode lp_mode;
	/*!
	 * Switcher dvs speed
	 */
	t_switcher_dvs_speed dvs_speed;
	/*!
	 * Switcher panic mode
	 */
	bool panic_mode;
	/*!
	 * Switcher softstart
	 */
	bool softstart;
	/*!
	 * PLL Multiplication factor
	 */
	t_switcher_factor factor;
} t_regulator_config;

/*!
 * @struct t_regulator_cfg_param
 * @brief regulator configuration structure for IOCTL.
 *
 */
typedef struct {
	/*!
	 * Regulator.
	 */
	t_pmic_regulator regulator;
	/*!
	 * Regulator configuration.
	 */
	t_regulator_config cfg;
} t_regulator_cfg_param;

/*!
 * This struct list all state reads in Power Up Sense
 */
struct t_p_up_sense {
	/*!
	 * power up sense ictest
	 */
	bool state_ictest;
	/*!
	 * power up sense clksel
	 */
	bool state_clksel;
	/*!
	 * power up mode supply 1
	 */
	bool state_pums1;
	/*!
	 * power up mode supply 2
	 */
	bool state_pums2;
	/*!
	 * power up mode supply 3
	 */
	bool state_pums3;
	/*!
	 * power up sense charge mode 0
	 */
	bool state_chrgmode0;
	/*!
	 * power up sense charge mode 1
	 */
	bool state_chrgmode1;
	/*!
	 * power up sense USB mode
	 */
	bool state_umod;
	/*!
	 * power up sense boot mode enable for USB/RS232
	 */
	bool state_usben;
	/*!
	 * power up sense switcher 1a1b joined
	 */
	bool state_sw_1a1b_joined;
	/*!
	 * power up sense switcher 1a1b joined
	 */
	bool state_sw_2a2b_joined;
};

/*!
 * This enumeration define all On_OFF button
 */
typedef enum {
	/*!
	 * ON1B
	 */
	BT_ON1B = 0,
	/*!
	 * ON2B
	 */
	BT_ON2B,
	/*!
	 * ON3B
	 */
	BT_ON3B,
} t_button;

#ifdef __KERNEL__
/* EXPORTED FUNCTIONS */

/*!
 * This function sets user power off in power control register and thus powers
 * off the phone.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
void pmic_power_off(void);

/*!
 * This function sets the power control configuration.
 *
 * @param        pc_config   power control configuration.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_set_pc_config(t_pc_config *pc_config);

/*!
 * This function retrives the power control configuration.
 *
 * @param        pc_config   pointer to power control configuration.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_get_pc_config(t_pc_config *pc_config);

/*!
 * This function turns on a regulator.
 *
 * @param        regulator    The regulator to be turned on.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_regulator_on(t_pmic_regulator regulator);

/*!
 * This function turns off a regulator.
 *
 * @param        regulator    The regulator to be turned off.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_regulator_off(t_pmic_regulator regulator);

/*!
 * This function sets the regulator output voltage.
 *
 * @param        regulator    The regulator to be turned off.
 * @param        voltage      The regulator output voltage.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_regulator_set_voltage(t_pmic_regulator regulator,
					     t_regulator_voltage voltage);

/*!
 * This function retrieves the regulator output voltage.
 *
 * @param        regulator    The regulator to be turned off.
 * @param        voltage      Pointer to regulator output voltage.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_regulator_get_voltage(t_pmic_regulator regulator,
					     t_regulator_voltage *voltage);

/*!
 * This function sets the DVS voltage
 *
 * @param        regulator    The regulator to be configured.
 * @param        dvs          The switch Dynamic Voltage Scaling
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_switcher_set_dvs(t_pmic_regulator regulator,
					t_regulator_voltage dvs);

/*!
 * This function gets the DVS voltage
 *
 * @param        regulator    The regulator to be handled.
 * @param        dvs          The switch Dynamic Voltage Scaling
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_switcher_get_dvs(t_pmic_regulator regulator,
					t_regulator_voltage *dvs);

/*!
 * This function sets the standby voltage
 *
 * @param        regulator    The regulator to be configured.
 * @param        stby         The switch standby voltage
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_switcher_set_stby(t_pmic_regulator regulator,
					 t_regulator_voltage stby);

/*!
 * This function gets the standby voltage
 *
 * @param        regulator    The regulator to be handled.
 * @param        stby         The switch standby voltage
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_switcher_get_stby(t_pmic_regulator regulator,
					 t_regulator_voltage *stby);

/*!
 * This function sets the switchers mode.
 *
 * @param        regulator    The regulator to be configured.
 * @param        mode         The switcher mode
 * @param        stby         Switch between main and standby.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_switcher_set_mode(t_pmic_regulator regulator,
					 t_regulator_sw_mode mode, bool stby);

/*!
 * This function gets the switchers mode.
 *
 * @param        regulator    The regulator to be handled.
 * @param        mode         The switcher mode.
 * @param        stby         Switch between main and standby.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_switcher_get_mode(t_pmic_regulator regulator,
					 t_regulator_sw_mode *mode, bool stby);

/*!
 * This function sets the switch dvs speed
 *
 * @param        regulator    The regulator to be configured.
 * @param        speed        The dvs speed.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_switcher_set_dvs_speed(t_pmic_regulator regulator,
					      t_switcher_dvs_speed speed);

/*!
 * This function gets the switch dvs speed
 *
 * @param        regulator    The regulator to be handled.
 * @param        speed        The dvs speed.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_switcher_get_dvs_speed(t_pmic_regulator regulator,
					      t_switcher_dvs_speed *speed);

/*!
 * This function sets the switch panic mode
 *
 * @param        regulator    The regulator to be configured.
 * @param        panic_mode   Enable or disable panic mode
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_switcher_set_panic_mode(t_pmic_regulator regulator,
					       bool panic_mode);

/*!
 * This function gets the switch panic mode
 *
 * @param        regulator    The regulator to be handled
 * @param        panic_mode   Enable or disable panic mode
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_switcher_get_panic_mode(t_pmic_regulator regulator,
					       bool *panic_mode);

/*!
 * This function sets the switch softstart mode
 *
 * @param        regulator    The regulator to be configured.
 * @param        softstart    Enable or disable softstart.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_switcher_set_softstart(t_pmic_regulator regulator,
					      bool softstart);

/*!
 * This function gets the switch softstart mode
 *
 * @param        regulator    The regulator to be handled
 * @param        softstart    Enable or disable softstart.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_switcher_get_softstart(t_pmic_regulator regulator,
					      bool *softstart);

/*!
 * This function sets the PLL multiplication factor
 *
 * @param        regulator    The regulator to be configured.
 * @param        factor              The multiplication factor.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_switcher_set_factor(t_pmic_regulator regulator,
					   t_switcher_factor factor);

/*!
 * This function gets the PLL multiplication factor
 *
 * @param        regulator    The regulator to be handled
 * @param        factor       The multiplication factor.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_switcher_get_factor(t_pmic_regulator regulator,
					   t_switcher_factor *factor);

/*!
 * This function enables or disables low power mode.
 *
 * @param        regulator    The regulator to be configured.
 * @param        mode         Select nominal or low power mode.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_regulator_set_lp_mode(t_pmic_regulator regulator,
					     t_regulator_lp_mode lp_mode);

/*!
 * This function gets low power mode.
 *
 * @param        regulator    The regulator to be handled
 * @param        mode         Select nominal or low power mode.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_regulator_get_lp_mode(t_pmic_regulator regulator,
					     t_regulator_lp_mode *lp_mode);

/*!
 * This function sets the regulator configuration.
 *
 * @param        regulator    The regulator to be turned off.
 * @param        config       The regulator output configuration.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_regulator_set_config(t_pmic_regulator regulator,
					    t_regulator_config *config);

/*!
 * This function retrieves the regulator output configuration.
 *
 * @param        regulator    The regulator to be turned off.
 * @param        config       Pointer to regulator configuration.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_regulator_get_config(t_pmic_regulator regulator,
					    t_regulator_config *config);

/*!
 * This function enables automatically VBKUP2 in the memory hold modes.
 *
 * @param        en           if true, enable VBKUP2AUTOMH
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_vbkup2_auto_en(bool en);

/*!
 * This function gets state of automatically VBKUP2.
 *
 * @param        en           if true, VBKUP2AUTOMH is enabled
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_get_vbkup2_auto_state(bool *en);

/*!
 * This function enables battery detect function.
 *
 * @param        en           if true, enable BATTDETEN
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_bat_det_en(bool en);

/*!
 * This function gets state of battery detect function.
 *
 * @param        en           if true, BATTDETEN is enabled
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_get_bat_det_state(bool *en);

/*!
 * This function enables control of VVIB by VIBEN pin.
 *
 * @param        en           if true, enable VIBPINCTRL
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_vib_pin_en(bool en);

/*!
 * This function gets state of control of VVIB by VIBEN pin.
 * @param        en           if true, VIBPINCTRL is enabled
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_gets_vib_pin_state(bool *en);

/*!
 * This function returns power up sense value
 *
 * @param        p_up_sense     value of power up sense
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_get_power_mode_sense(struct t_p_up_sense *p_up_sense);

/*!
 * This function configures the Regen assignment for all regulator
 *
 * @param        regulator      type of regulator
 * @param        en_dis         if true, the regulator is enabled by regen.
 *
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS pmic_power_set_regen_assig(t_pmic_regulator regulator, bool en_dis);

/*!
 * This function gets the Regen assignment for all regulator
 *
 * @param        regulator      type of regulator
 * @param        en_dis         return value, if true :
 *       		         the regulator is enabled by regen.
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS pmic_power_get_regen_assig(t_pmic_regulator regu, bool *en_dis);

/*!
 * This function sets the Regen polarity.
 *
 * @param        en_dis         If true regen is inverted.
 *
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS pmic_power_set_regen_inv(bool en_dis);

/*!
 * This function gets the Regen polarity.
 *
 * @param        en_dis         If true regen is inverted.
 *
 * @return       This function returns 0 if successful.
 */

PMIC_STATUS pmic_power_get_regen_inv(bool *en_dis);

/*!
 * This function enables esim control voltage.
 *
 * @param        vesim          if true, enable VESIMESIMEN
 * @param        vmmc1          if true, enable VMMC1ESIMEN
 * @param        vmmc2          if true, enable VMMC2ESIMEN
 *
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS pmic_power_esim_v_en(bool vesim, bool vmmc1, bool vmmc2);

/*!
 * This function gets esim control voltage values.
 *
 * @param        vesim          if true, enable VESIMESIMEN
 * @param        vmmc1          if true, enable VMMC1ESIMEN
 * @param        vmmc2          if true, enable VMMC2ESIMEN
 *
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS pmic_power_gets_esim_v_state(bool *vesim,
					 bool *vmmc1, bool *vmmc2);

/*!
 * This function enables auto reset after a system reset.
 *
 * @param        en         if true, the auto reset is enabled
 *
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS pmic_power_set_auto_reset_en(bool en);

/*!
 * This function gets auto reset configuration.
 *
 * @param        en         if true, the auto reset is enabled
 *
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS pmic_power_get_auto_reset_en(bool *en);

/*!
 * This function configures a system reset on a button.
 *
 * @param       bt         type of button.
 * @param       sys_rst    if true, enable the system reset on this button
 * @param       deb_time   sets the debounce time on this button pin
 *
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS pmic_power_set_conf_button(t_button bt, bool sys_rst, int deb_time);

/*!
 * This function gets configuration of a button.
 *
 * @param       bt         type of button.
 * @param       sys_rst    if true, the system reset is enabled on this button
 * @param       deb_time   gets the debounce time on this button pin
 *
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS pmic_power_get_conf_button(t_button bt,
				       bool *sys_rst, int *deb_time);

/*!
 * This function is used to subscribe on power event IT.
 *
 * @param        event          type of event.
 * @param        callback       event callback function.
 *
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS pmic_power_event_sub(t_pwr_int event, void *callback);

/*!
 * This function is used to un subscribe on power event IT.
 *
 * @param        event          type of event.
 * @param        callback       event callback function.
 *
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS pmic_power_event_unsub(t_pwr_int event, void *callback);

#endif				/* __KERNEL__ */

#endif				/* __ASM_ARCH_MXC_PMIC_POWER_H__ */
