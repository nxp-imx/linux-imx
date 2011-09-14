/*
 * Copyright 2004-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mc13783/pmic_power.c
 * @brief This is the main file of PMIC(mc13783) Power driver.
 *
 * @ingroup PMIC_POWER
 */

/*
 * Includes
 */

#include <linux/platform_device.h>
#include <linux/ioctl.h>
#include <linux/pmic_status.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <mach/pmic_power.h>

#include "pmic_power_defs.h"

#ifdef CONFIG_MXC_HWEVENT
#include <mach/hw_events.h>
#endif

#include <asm/mach-types.h>

#define MC13783_REGCTRL_GPOx_MASK 0x18000

static bool VBKUP1_EN;
static bool VBKUP2_EN;

/*
 * Power Pmic API
 */

/* EXPORTED FUNCTIONS */
EXPORT_SYMBOL(pmic_power_off);
EXPORT_SYMBOL(pmic_power_set_pc_config);
EXPORT_SYMBOL(pmic_power_get_pc_config);
EXPORT_SYMBOL(pmic_power_regulator_on);
EXPORT_SYMBOL(pmic_power_regulator_off);
EXPORT_SYMBOL(pmic_power_regulator_set_voltage);
EXPORT_SYMBOL(pmic_power_regulator_get_voltage);
EXPORT_SYMBOL(pmic_power_regulator_set_config);
EXPORT_SYMBOL(pmic_power_regulator_get_config);
EXPORT_SYMBOL(pmic_power_vbkup2_auto_en);
EXPORT_SYMBOL(pmic_power_get_vbkup2_auto_state);
EXPORT_SYMBOL(pmic_power_bat_det_en);
EXPORT_SYMBOL(pmic_power_get_bat_det_state);
EXPORT_SYMBOL(pmic_power_vib_pin_en);
EXPORT_SYMBOL(pmic_power_gets_vib_pin_state);
EXPORT_SYMBOL(pmic_power_get_power_mode_sense);
EXPORT_SYMBOL(pmic_power_set_regen_assig);
EXPORT_SYMBOL(pmic_power_get_regen_assig);
EXPORT_SYMBOL(pmic_power_set_regen_inv);
EXPORT_SYMBOL(pmic_power_get_regen_inv);
EXPORT_SYMBOL(pmic_power_esim_v_en);
EXPORT_SYMBOL(pmic_power_gets_esim_v_state);
EXPORT_SYMBOL(pmic_power_set_auto_reset_en);
EXPORT_SYMBOL(pmic_power_get_auto_reset_en);
EXPORT_SYMBOL(pmic_power_set_conf_button);
EXPORT_SYMBOL(pmic_power_get_conf_button);
EXPORT_SYMBOL(pmic_power_event_sub);
EXPORT_SYMBOL(pmic_power_event_unsub);

/*!
 * This function is called to put the power in a low power state.
 * Switching off the platform cannot be decided by
 * the power module. It has to be handled by the
 * client application.
 *
 * @param   pdev  the device structure used to give information on which power
 *                device (0 through 3 channels) to suspend
 * @param   state the power state the device is entering
 *
 * @return  The function always returns 0.
 */
static int pmic_power_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
};

/*!
 * This function is called to resume the power from a low power state.
 *
 * @param   pdev  the device structure used to give information on which power
 *                device (0 through 3 channels) to suspend
 *
 * @return  The function always returns 0.
 */
static int pmic_power_resume(struct platform_device *pdev)
{
	return 0;
};

/*!
 * This function sets user power off in power control register and thus powers
 * off the phone.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
void pmic_power_off(void)
{
	unsigned int mask, value;

	mask = BITFMASK(MC13783_PWRCTRL_USER_OFF_SPI);
	value = BITFVAL(MC13783_PWRCTRL_USER_OFF_SPI,
			MC13783_PWRCTRL_USER_OFF_SPI_ENABLE);

	pmic_write_reg(REG_POWER_CONTROL_0, value, mask);
}

/*!
 * This function sets the power control configuration.
 *
 * @param        pc_config   power control configuration.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_set_pc_config(t_pc_config *pc_config)
{
	unsigned int pwrctrl_val_reg0 = 0;
	unsigned int pwrctrl_val_reg1 = 0;
	unsigned int pwrctrl_mask_reg0 = 0;
	unsigned int pwrctrl_mask_reg1 = 0;

	if (pc_config == NULL) {
		return PMIC_PARAMETER_ERROR;
	}

	if (pc_config->pc_enable != false) {
		pwrctrl_val_reg0 |= BITFVAL(MC13783_PWRCTRL_PCEN,
					    MC13783_PWRCTRL_PCEN_ENABLE);
		pwrctrl_val_reg1 |= BITFVAL(MC13783_PWRCTRL_PCT,
					    pc_config->pc_timer);
	} else {
		pwrctrl_val_reg0 |= BITFVAL(MC13783_PWRCTRL_PCEN,
					    MC13783_PWRCTRL_PCEN_DISABLE);
	}
	pwrctrl_mask_reg0 |= BITFMASK(MC13783_PWRCTRL_PCEN);
	pwrctrl_mask_reg1 |= BITFMASK(MC13783_PWRCTRL_PCT);

	if (pc_config->pc_count_enable != false) {

		pwrctrl_val_reg0 |= BITFVAL(MC13783_PWRCTRL_PC_COUNT_EN,
					    MC13783_PWRCTRL_PC_COUNT_EN_ENABLE);
		pwrctrl_val_reg1 |= BITFVAL(MC13783_PWRCTRL_PC_COUNT,
					    pc_config->pc_count);
		pwrctrl_val_reg1 |= BITFVAL(MC13783_PWRCTRL_PC_MAX_CNT,
					    pc_config->pc_max_count);
	} else {
		pwrctrl_val_reg0 |= BITFVAL(MC13783_PWRCTRL_PC_COUNT_EN,
					    MC13783_PWRCTRL_PC_COUNT_EN_DISABLE);
	}
	pwrctrl_mask_reg0 |= BITFMASK(MC13783_PWRCTRL_PC_COUNT_EN);
	pwrctrl_mask_reg1 |= BITFMASK(MC13783_PWRCTRL_PC_MAX_CNT) |
	    BITFMASK(MC13783_PWRCTRL_PC_COUNT);

	if (pc_config->warm_enable != false) {
		pwrctrl_val_reg0 |= BITFVAL(MC13783_PWRCTRL_WARM_EN,
					    MC13783_PWRCTRL_WARM_EN_ENABLE);
	} else {
		pwrctrl_val_reg0 |= BITFVAL(MC13783_PWRCTRL_WARM_EN,
					    MC13783_PWRCTRL_WARM_EN_DISABLE);
	}
	pwrctrl_mask_reg0 |= BITFMASK(MC13783_PWRCTRL_WARM_EN);

	if (pc_config->user_off_pc != false) {
		pwrctrl_val_reg0 |= BITFVAL(MC13783_PWRCTRL_USER_OFF_PC,
					    MC13783_PWRCTRL_USER_OFF_PC_ENABLE);
	} else {
		pwrctrl_val_reg0 |= BITFVAL(MC13783_PWRCTRL_WARM_EN,
					    MC13783_PWRCTRL_USER_OFF_PC_DISABLE);
	}
	pwrctrl_mask_reg0 |= BITFMASK(MC13783_PWRCTRL_USER_OFF_PC);

	if (pc_config->clk_32k_user_off != false) {
		pwrctrl_val_reg0 |= BITFVAL(MC13783_PWRCTRL_32OUT_USER_OFF,
					    MC13783_PWRCTRL_32OUT_USER_OFF_ENABLE);
	} else {
		pwrctrl_val_reg0 |= BITFVAL(MC13783_PWRCTRL_32OUT_USER_OFF,
					    MC13783_PWRCTRL_32OUT_USER_OFF_DISABLE);
	}
	pwrctrl_mask_reg0 |= BITFMASK(MC13783_PWRCTRL_32OUT_USER_OFF);

	if (pc_config->clk_32k_enable != false) {
		pwrctrl_val_reg0 |= BITFVAL(MC13783_PWRCTRL_32OUT_EN,
					    MC13783_PWRCTRL_32OUT_EN_ENABLE);
	} else {
		pwrctrl_val_reg0 |= BITFVAL(MC13783_PWRCTRL_32OUT_EN,
					    MC13783_PWRCTRL_32OUT_EN_DISABLE);
	}
	pwrctrl_mask_reg0 |= BITFMASK(MC13783_PWRCTRL_32OUT_EN);

	if (pc_config->en_vbkup1 != false) {
		pwrctrl_val_reg0 |= BITFVAL(MC13783_PWRCTRL_VBKUP1_EN,
					    MC13783_PWRCTRL_VBKUP_ENABLE);
		VBKUP1_EN = true;
	} else {
		pwrctrl_val_reg0 |= BITFVAL(MC13783_PWRCTRL_VBKUP1_EN,
					    MC13783_PWRCTRL_VBKUP_DISABLE);
		VBKUP1_EN = false;
	}
	pwrctrl_mask_reg0 |= BITFMASK(MC13783_PWRCTRL_VBKUP1_EN);

	if (pc_config->en_vbkup2 != false) {
		pwrctrl_val_reg0 |= BITFVAL(MC13783_PWRCTRL_VBKUP2_EN,
					    MC13783_PWRCTRL_VBKUP_ENABLE);
		VBKUP2_EN = true;
	} else {
		pwrctrl_val_reg0 |= BITFVAL(MC13783_PWRCTRL_VBKUP2_EN,
					    MC13783_PWRCTRL_VBKUP_DISABLE);
		VBKUP2_EN = false;
	}
	pwrctrl_mask_reg0 |= BITFMASK(MC13783_PWRCTRL_VBKUP2_EN);

	if (pc_config->auto_en_vbkup1 != false) {
		pwrctrl_val_reg0 |= BITFVAL(MC13783_PWRCTRL_VBKUP1_AUTO_EN,
					    MC13783_PWRCTRL_VBKUP_ENABLE);
	} else {
		pwrctrl_val_reg0 |= BITFVAL(MC13783_PWRCTRL_VBKUP1_AUTO_EN,
					    MC13783_PWRCTRL_VBKUP_DISABLE);
	}
	pwrctrl_mask_reg0 |= BITFMASK(MC13783_PWRCTRL_VBKUP1_AUTO_EN);

	if (pc_config->auto_en_vbkup2 != false) {
		pwrctrl_val_reg0 |= BITFVAL(MC13783_PWRCTRL_VBKUP2_AUTO_EN,
					    MC13783_PWRCTRL_VBKUP_ENABLE);
	} else {
		pwrctrl_val_reg0 |= BITFVAL(MC13783_PWRCTRL_VBKUP2_AUTO_EN,
					    MC13783_PWRCTRL_VBKUP_DISABLE);
	}
	pwrctrl_mask_reg0 |= BITFMASK(MC13783_PWRCTRL_VBKUP2_AUTO_EN);

	if (VBKUP1_EN != false) {
		if (pc_config->vhold_voltage > 3
		    || pc_config->vhold_voltage < 0) {
			return PMIC_PARAMETER_ERROR;
		} else {

			pwrctrl_val_reg0 |= BITFVAL(MC13783_PWRCTRL_VBKUP1,
						    pc_config->vhold_voltage);
		}
	}
	if (VBKUP2_EN != false) {
		if (pc_config->vhold_voltage > 3
		    || pc_config->vhold_voltage < 0) {
			return PMIC_PARAMETER_ERROR;
		} else {
			pwrctrl_val_reg0 |= BITFVAL(MC13783_PWRCTRL_VBKUP2,
						    pc_config->vhold_voltage2);
		}
	}
	pwrctrl_mask_reg0 |= BITFMASK(MC13783_PWRCTRL_VBKUP1) |
	    BITFMASK(MC13783_PWRCTRL_VBKUP2);

	if (pc_config->mem_allon != false) {
		pwrctrl_val_reg1 |= BITFVAL(MC13783_PWRCTRL_MEM_ALLON,
					    MC13783_PWRCTRL_MEM_ALLON_ENABLE);
		pwrctrl_val_reg1 |= BITFVAL(MC13783_PWRCTRL_MEM_TMR,
					    pc_config->mem_timer);
	} else {
		pwrctrl_val_reg1 |= BITFVAL(MC13783_PWRCTRL_MEM_ALLON,
					    MC13783_PWRCTRL_MEM_ALLON_DISABLE);
	}
	pwrctrl_mask_reg1 |= BITFMASK(MC13783_PWRCTRL_MEM_ALLON) |
	    BITFMASK(MC13783_PWRCTRL_MEM_TMR);

	CHECK_ERROR(pmic_write_reg(REG_POWER_CONTROL_0,
				   pwrctrl_val_reg0, pwrctrl_mask_reg0));
	CHECK_ERROR(pmic_write_reg(REG_POWER_CONTROL_1,
				   pwrctrl_val_reg1, pwrctrl_mask_reg1));

	return PMIC_SUCCESS;
}

/*!
 * This function retrives the power control configuration.
 *
 * @param        pc_config   pointer to power control configuration.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_get_pc_config(t_pc_config *pc_config)
{
	unsigned int pwrctrl_val_reg0 = 0;
	unsigned int pwrctrl_val_reg1 = 0;

	if (pc_config == NULL) {
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_read_reg(REG_POWER_CONTROL_0,
				  &pwrctrl_val_reg0, PMIC_ALL_BITS));
	CHECK_ERROR(pmic_read_reg(REG_POWER_CONTROL_1,
				  &pwrctrl_val_reg1, PMIC_ALL_BITS));

	if (BITFEXT(pwrctrl_val_reg0, MC13783_PWRCTRL_PCEN)
	    == MC13783_PWRCTRL_PCEN_ENABLE) {
		pc_config->pc_enable = true;
		pc_config->pc_timer = BITFEXT(pwrctrl_val_reg1,
					      MC13783_PWRCTRL_PCT);

	} else {
		pc_config->pc_enable = false;
		pc_config->pc_timer = 0;
	}

	if (BITFEXT(pwrctrl_val_reg0, MC13783_PWRCTRL_PC_COUNT_EN)
	    == MC13783_PWRCTRL_PCEN_ENABLE) {
		pc_config->pc_count_enable = true;
		pc_config->pc_count = BITFEXT(pwrctrl_val_reg1,
					      MC13783_PWRCTRL_PC_COUNT);
		pc_config->pc_max_count = BITFEXT(pwrctrl_val_reg1,
						  MC13783_PWRCTRL_PC_MAX_CNT);
	} else {
		pc_config->pc_count_enable = false;
		pc_config->pc_count = 0;
		pc_config->pc_max_count = 0;
	}

	if (BITFEXT(pwrctrl_val_reg0, MC13783_PWRCTRL_WARM_EN)
	    == MC13783_PWRCTRL_WARM_EN_ENABLE) {
		pc_config->warm_enable = true;
	} else {
		pc_config->warm_enable = false;
	}

	if (BITFEXT(pwrctrl_val_reg0, MC13783_PWRCTRL_USER_OFF_PC)
	    == MC13783_PWRCTRL_USER_OFF_PC_ENABLE) {
		pc_config->user_off_pc = true;
	} else {
		pc_config->user_off_pc = false;
	}

	if (BITFEXT(pwrctrl_val_reg0, MC13783_PWRCTRL_32OUT_USER_OFF)
	    == MC13783_PWRCTRL_32OUT_USER_OFF_ENABLE) {
		pc_config->clk_32k_user_off = true;
	} else {
		pc_config->clk_32k_user_off = false;
	}

	if (BITFEXT(pwrctrl_val_reg0, MC13783_PWRCTRL_32OUT_EN)
	    == MC13783_PWRCTRL_32OUT_EN_ENABLE) {
		pc_config->clk_32k_enable = true;
	} else {
		pc_config->clk_32k_enable = false;
	}

	if (BITFEXT(pwrctrl_val_reg0, MC13783_PWRCTRL_VBKUP1_AUTO_EN)
	    == MC13783_PWRCTRL_VBKUP_ENABLE) {
		pc_config->auto_en_vbkup1 = true;
	} else {
		pc_config->auto_en_vbkup1 = false;
	}

	if (BITFEXT(pwrctrl_val_reg0, MC13783_PWRCTRL_VBKUP2_AUTO_EN)
	    == MC13783_PWRCTRL_VBKUP_ENABLE) {
		pc_config->auto_en_vbkup2 = true;
	} else {
		pc_config->auto_en_vbkup2 = false;
	}

	if (BITFEXT(pwrctrl_val_reg0, MC13783_PWRCTRL_VBKUP1_EN)
	    == MC13783_PWRCTRL_VBKUP_ENABLE) {
		pc_config->en_vbkup1 = true;
		pc_config->vhold_voltage = BITFEXT(pwrctrl_val_reg0,
						   MC13783_PWRCTRL_VBKUP1);
	} else {
		pc_config->en_vbkup1 = false;
		pc_config->vhold_voltage = 0;
	}

	if (BITFEXT(pwrctrl_val_reg0, MC13783_PWRCTRL_VBKUP2_EN)
	    == MC13783_PWRCTRL_VBKUP_ENABLE) {
		pc_config->en_vbkup2 = true;
		pc_config->vhold_voltage2 = BITFEXT(pwrctrl_val_reg0,
						    MC13783_PWRCTRL_VBKUP2);
	} else {
		pc_config->en_vbkup2 = false;
		pc_config->vhold_voltage2 = 0;
	}

	if (BITFEXT(pwrctrl_val_reg1, MC13783_PWRCTRL_MEM_ALLON) ==
	    MC13783_PWRCTRL_MEM_ALLON_ENABLE) {
		pc_config->mem_allon = true;
		pc_config->mem_timer = BITFEXT(pwrctrl_val_reg1,
					       MC13783_PWRCTRL_MEM_TMR);
	} else {
		pc_config->mem_allon = false;
		pc_config->mem_timer = 0;
	}

	return PMIC_SUCCESS;
}

/*!
 * This function turns on a regulator.
 *
 * @param        regulator    The regulator to be truned on.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_regulator_on(t_pmic_regulator regulator)
{
	unsigned int reg_val = 0, reg_mask = 0;
	unsigned int reg;

	switch (regulator) {
	case SW_PLL:
		reg_val = BITFVAL(MC13783_SWCTRL_PLL_EN,
				  MC13783_SWCTRL_PLL_EN_ENABLE);
		reg_mask = BITFMASK(MC13783_SWCTRL_PLL_EN);
		reg = REG_SWITCHERS_4;
		break;
	case SW_SW3:
		reg_val = BITFVAL(MC13783_SWCTRL_SW3_EN,
				  MC13783_SWCTRL_SW3_EN_ENABLE);
		reg_mask = BITFMASK(MC13783_SWCTRL_SW3_EN);
		reg = REG_SWITCHERS_5;
		break;
	case REGU_VAUDIO:
		reg_val = BITFVAL(MC13783_REGCTRL_VAUDIO_EN,
				  MC13783_REGCTRL_VAUDIO_EN_ENABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VAUDIO_EN);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VIOHI:
		reg_val = BITFVAL(MC13783_REGCTRL_VIOHI_EN,
				  MC13783_REGCTRL_VIOHI_EN_ENABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VIOHI_EN);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VIOLO:
		reg_val = BITFVAL(MC13783_REGCTRL_VIOLO_EN,
				  MC13783_REGCTRL_VIOLO_EN_ENABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VIOLO_EN);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VDIG:
		reg_val = BITFVAL(MC13783_REGCTRL_VDIG_EN,
				  MC13783_REGCTRL_VDIG_EN_ENABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VDIG_EN);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VGEN:
		reg_val = BITFVAL(MC13783_REGCTRL_VGEN_EN,
				  MC13783_REGCTRL_VGEN_EN_ENABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VGEN_EN);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VRFDIG:
		reg_val = BITFVAL(MC13783_REGCTRL_VRFDIG_EN,
				  MC13783_REGCTRL_VRFDIG_EN_ENABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VRFDIG_EN);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VRFREF:
		reg_val = BITFVAL(MC13783_REGCTRL_VRFREF_EN,
				  MC13783_REGCTRL_VRFREF_EN_ENABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VRFREF_EN);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VRFCP:
		reg_val = BITFVAL(MC13783_REGCTRL_VRFCP_EN,
				  MC13783_REGCTRL_VRFCP_EN_ENABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VRFCP_EN);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VSIM:
		reg_val = BITFVAL(MC13783_REGCTRL_VSIM_EN,
				  MC13783_REGCTRL_VSIM_EN_ENABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VSIM_EN);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VESIM:
		reg_val = BITFVAL(MC13783_REGCTRL_VESIM_EN,
				  MC13783_REGCTRL_VESIM_EN_ENABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VESIM_EN);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VCAM:
		reg_val = BITFVAL(MC13783_REGCTRL_VCAM_EN,
				  MC13783_REGCTRL_VCAM_EN_ENABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VCAM_EN);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VRFBG:
		reg_val = BITFVAL(MC13783_REGCTRL_VRFBG_EN,
				  MC13783_REGCTRL_VRFBG_EN_ENABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VRFBG_EN);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VVIB:
		reg_val = BITFVAL(MC13783_REGCTRL_VVIB_EN,
				  MC13783_REGCTRL_VVIB_EN_ENABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VVIB_EN);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VRF1:
		reg_val = BITFVAL(MC13783_REGCTRL_VRF1_EN,
				  MC13783_REGCTRL_VRF1_EN_ENABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VRF1_EN);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VRF2:
		reg_val = BITFVAL(MC13783_REGCTRL_VRF2_EN,
				  MC13783_REGCTRL_VRF2_EN_ENABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VRF2_EN);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VMMC1:
		reg_val = BITFVAL(MC13783_REGCTRL_VMMC1_EN,
				  MC13783_REGCTRL_VMMC1_EN_ENABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VMMC1_EN);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VMMC2:
		reg_val = BITFVAL(MC13783_REGCTRL_VMMC2_EN,
				  MC13783_REGCTRL_VMMC2_EN_ENABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VMMC2_EN);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_GPO1:
		reg_val = BITFVAL(MC13783_REGCTRL_GPO1_EN,
				  MC13783_REGCTRL_GPO1_EN_ENABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_GPO1_EN);
		reg = REG_POWER_MISCELLANEOUS;
		break;
	case REGU_GPO2:
		reg_val = BITFVAL(MC13783_REGCTRL_GPO2_EN,
				  MC13783_REGCTRL_GPO2_EN_ENABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_GPO2_EN);
		reg = REG_POWER_MISCELLANEOUS;
		break;
	case REGU_GPO3:
		reg_val = BITFVAL(MC13783_REGCTRL_GPO3_EN,
				  MC13783_REGCTRL_GPO3_EN_ENABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_GPO3_EN);
		reg = REG_POWER_MISCELLANEOUS;
		break;
	case REGU_GPO4:
		reg_val = BITFVAL(MC13783_REGCTRL_GPO4_EN,
				  MC13783_REGCTRL_GPO4_EN_ENABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_GPO4_EN);
		reg = REG_POWER_MISCELLANEOUS;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(reg, reg_val, reg_mask));

	return PMIC_SUCCESS;
}

/*!
 * This function turns off a regulator.
 *
 * @param        regulator    The regulator to be truned off.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_regulator_off(t_pmic_regulator regulator)
{
	unsigned int reg_val = 0, reg_mask = 0;
	unsigned int reg;

	switch (regulator) {
	case SW_PLL:
		reg_val = BITFVAL(MC13783_SWCTRL_PLL_EN,
				  MC13783_SWCTRL_PLL_EN_DISABLE);
		reg_mask = BITFMASK(MC13783_SWCTRL_PLL_EN);
		reg = REG_SWITCHERS_4;
		break;
	case SW_SW3:
		reg_val = BITFVAL(MC13783_SWCTRL_SW3_EN,
				  MC13783_SWCTRL_SW3_EN_DISABLE);
		reg_mask = BITFMASK(MC13783_SWCTRL_SW3_EN);
		reg = REG_SWITCHERS_5;
		break;
	case REGU_VAUDIO:
		reg_val = BITFVAL(MC13783_REGCTRL_VAUDIO_EN,
				  MC13783_REGCTRL_VAUDIO_EN_DISABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VAUDIO_EN);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VIOHI:
		reg_val = BITFVAL(MC13783_REGCTRL_VIOHI_EN,
				  MC13783_REGCTRL_VIOHI_EN_DISABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VIOHI_EN);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VIOLO:
		reg_val = BITFVAL(MC13783_REGCTRL_VIOLO_EN,
				  MC13783_REGCTRL_VIOLO_EN_DISABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VIOLO_EN);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VDIG:
		reg_val = BITFVAL(MC13783_REGCTRL_VDIG_EN,
				  MC13783_REGCTRL_VDIG_EN_DISABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VDIG_EN);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VGEN:
		reg_val = BITFVAL(MC13783_REGCTRL_VGEN_EN,
				  MC13783_REGCTRL_VGEN_EN_DISABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VGEN_EN);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VRFDIG:
		reg_val = BITFVAL(MC13783_REGCTRL_VRFDIG_EN,
				  MC13783_REGCTRL_VRFDIG_EN_DISABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VRFDIG_EN);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VRFREF:
		reg_val = BITFVAL(MC13783_REGCTRL_VRFREF_EN,
				  MC13783_REGCTRL_VRFREF_EN_DISABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VRFREF_EN);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VRFCP:
		reg_val = BITFVAL(MC13783_REGCTRL_VRFCP_EN,
				  MC13783_REGCTRL_VRFCP_EN_DISABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VRFCP_EN);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VSIM:
		reg_val = BITFVAL(MC13783_REGCTRL_VSIM_EN,
				  MC13783_REGCTRL_VSIM_EN_DISABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VSIM_EN);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VESIM:
		reg_val = BITFVAL(MC13783_REGCTRL_VESIM_EN,
				  MC13783_REGCTRL_VESIM_EN_DISABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VESIM_EN);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VCAM:
		reg_val = BITFVAL(MC13783_REGCTRL_VCAM_EN,
				  MC13783_REGCTRL_VCAM_EN_DISABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VCAM_EN);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VRFBG:
		reg_val = BITFVAL(MC13783_REGCTRL_VRFBG_EN,
				  MC13783_REGCTRL_VRFBG_EN_DISABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VRFBG_EN);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VVIB:
		reg_val = BITFVAL(MC13783_REGCTRL_VVIB_EN,
				  MC13783_REGCTRL_VVIB_EN_DISABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VVIB_EN);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VRF1:
		reg_val = BITFVAL(MC13783_REGCTRL_VRF1_EN,
				  MC13783_REGCTRL_VRF1_EN_DISABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VRF1_EN);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VRF2:
		reg_val = BITFVAL(MC13783_REGCTRL_VRF2_EN,
				  MC13783_REGCTRL_VRF2_EN_DISABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VRF2_EN);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VMMC1:
		reg_val = BITFVAL(MC13783_REGCTRL_VMMC1_EN,
				  MC13783_REGCTRL_VMMC1_EN_DISABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VMMC1_EN);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VMMC2:
		reg_val = BITFVAL(MC13783_REGCTRL_VMMC2_EN,
				  MC13783_REGCTRL_VMMC2_EN_DISABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_VMMC2_EN);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_GPO1:
		reg_val = BITFVAL(MC13783_REGCTRL_GPO1_EN,
				  MC13783_REGCTRL_GPO1_EN_DISABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_GPO1_EN);
		reg = REG_POWER_MISCELLANEOUS;
		break;
	case REGU_GPO2:
		reg_val = BITFVAL(MC13783_REGCTRL_GPO2_EN,
				  MC13783_REGCTRL_GPO2_EN_DISABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_GPO2_EN);
		reg = REG_POWER_MISCELLANEOUS;
		break;
	case REGU_GPO3:
		reg_val = BITFVAL(MC13783_REGCTRL_GPO3_EN,
				  MC13783_REGCTRL_GPO3_EN_DISABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_GPO3_EN);
		reg = REG_POWER_MISCELLANEOUS;
		break;
	case REGU_GPO4:
		reg_val = BITFVAL(MC13783_REGCTRL_GPO4_EN,
				  MC13783_REGCTRL_GPO4_EN_DISABLE);
		reg_mask = BITFMASK(MC13783_REGCTRL_GPO4_EN);
		reg = REG_POWER_MISCELLANEOUS;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(reg, reg_val, reg_mask));

	return PMIC_SUCCESS;
}

/*!
 * This function sets the regulator output voltage.
 *
 * @param        regulator    The regulator to be configured.
 * @param        voltage      The regulator output voltage.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_regulator_set_voltage(t_pmic_regulator regulator,
					     t_regulator_voltage voltage)
{
	unsigned int reg_val = 0, reg_mask = 0;
	unsigned int reg;

	switch (regulator) {
	case SW_SW1A:
		if ((voltage.sw1a < SW1A_0_9V) || (voltage.sw1a > SW1A_2_2V)) {
			return PMIC_PARAMETER_ERROR;
		}
		reg_val = BITFVAL(MC13783_SWSET_SW1A, voltage.sw1a);
		reg_mask = BITFMASK(MC13783_SWSET_SW1A);
		reg = REG_SWITCHERS_0;
		break;
	case SW_SW1B:
		if ((voltage.sw1b < SW1B_0_9V) || (voltage.sw1b > SW1B_2_2V)) {
			return PMIC_PARAMETER_ERROR;
		}
		reg_val = BITFVAL(MC13783_SWSET_SW1B, voltage.sw1b);
		reg_mask = BITFMASK(MC13783_SWSET_SW1B);
		reg = REG_SWITCHERS_1;
		break;
	case SW_SW2A:
		if ((voltage.sw2a < SW2A_0_9V) || (voltage.sw2a > SW2A_2_2V)) {
			return PMIC_PARAMETER_ERROR;
		}
		reg_val = BITFVAL(MC13783_SWSET_SW2A, voltage.sw1a);
		reg_mask = BITFMASK(MC13783_SWSET_SW2A);
		reg = REG_SWITCHERS_2;
		break;
	case SW_SW2B:
		if ((voltage.sw2b < SW2B_0_9V) || (voltage.sw2b > SW2B_2_2V)) {
			return PMIC_PARAMETER_ERROR;
		}
		reg_val = BITFVAL(MC13783_SWSET_SW2B, voltage.sw2b);
		reg_mask = BITFMASK(MC13783_SWSET_SW1A);
		reg = REG_SWITCHERS_3;
		break;
	case SW_SW3:
		if (voltage.sw3 != SW3_5V) {
			return PMIC_PARAMETER_ERROR;
		}
		reg_val = BITFVAL(MC13783_SWSET_SW3, voltage.sw3);
		reg_mask = BITFMASK(MC13783_SWSET_SW3);
		reg = REG_SWITCHERS_5;
		break;
	case REGU_VIOLO:
		if ((voltage.violo < VIOLO_1_2V) ||
		    (voltage.violo > VIOLO_1_8V)) {
			return PMIC_PARAMETER_ERROR;
		}
		reg_val = BITFVAL(MC13783_REGSET_VIOLO, voltage.violo);
		reg_mask = BITFMASK(MC13783_REGSET_VIOLO);
		reg = REG_REGULATOR_SETTING_0;
		break;
	case REGU_VDIG:
		if ((voltage.vdig < VDIG_1_2V) || (voltage.vdig > VDIG_1_8V)) {
			return PMIC_PARAMETER_ERROR;
		}
		reg_val = BITFVAL(MC13783_REGSET_VDIG, voltage.vdig);
		reg_mask = BITFMASK(MC13783_REGSET_VDIG);
		reg = REG_REGULATOR_SETTING_0;
		break;
	case REGU_VGEN:
		if ((voltage.vgen < VGEN_1_2V) || (voltage.vgen > VGEN_2_4V)) {
			return PMIC_PARAMETER_ERROR;
		}
		reg_val = BITFVAL(MC13783_REGSET_VGEN, voltage.vgen);
		reg_mask = BITFMASK(MC13783_REGSET_VGEN);
		reg = REG_REGULATOR_SETTING_0;
		break;
	case REGU_VRFDIG:
		if ((voltage.vrfdig < VRFDIG_1_2V) ||
		    (voltage.vrfdig > VRFDIG_1_875V)) {
			return PMIC_PARAMETER_ERROR;
		}
		reg_val = BITFVAL(MC13783_REGSET_VRFDIG, voltage.vrfdig);
		reg_mask = BITFMASK(MC13783_REGSET_VRFDIG);
		reg = REG_REGULATOR_SETTING_0;
		break;
	case REGU_VRFREF:
		if ((voltage.vrfref < VRFREF_2_475V) ||
		    (voltage.vrfref > VRFREF_2_775V)) {
			return PMIC_PARAMETER_ERROR;
		}
		reg_val = BITFVAL(MC13783_REGSET_VRFREF, voltage.vrfref);
		reg_mask = BITFMASK(MC13783_REGSET_VRFREF);
		reg = REG_REGULATOR_SETTING_0;
		break;
	case REGU_VRFCP:
		if ((voltage.vrfcp < VRFCP_2_7V) ||
		    (voltage.vrfcp > VRFCP_2_775V)) {
			return PMIC_PARAMETER_ERROR;
		}
		reg_val = BITFVAL(MC13783_REGSET_VRFCP, voltage.vrfcp);
		reg_mask = BITFMASK(MC13783_REGSET_VRFCP);
		reg = REG_REGULATOR_SETTING_0;
		break;
	case REGU_VSIM:
		if ((voltage.vsim < VSIM_1_8V) || (voltage.vsim > VSIM_2_9V)) {
			return PMIC_PARAMETER_ERROR;
		}
		reg_val = BITFVAL(MC13783_REGSET_VSIM, voltage.vsim);
		reg_mask = BITFMASK(MC13783_REGSET_VSIM);
		reg = REG_REGULATOR_SETTING_0;
		break;
	case REGU_VESIM:
		if ((voltage.vesim < VESIM_1_8V) ||
		    (voltage.vesim > VESIM_2_9V)) {
			return PMIC_PARAMETER_ERROR;
		}
		reg_val = BITFVAL(MC13783_REGSET_VESIM, voltage.vesim);
		reg_mask = BITFMASK(MC13783_REGSET_VESIM);
		reg = REG_REGULATOR_SETTING_0;
		break;
	case REGU_VCAM:
		if ((voltage.vcam < VCAM_1_5V) || (voltage.vcam > VCAM_3V)) {
			return PMIC_PARAMETER_ERROR;
		}
		reg_val = BITFVAL(MC13783_REGSET_VCAM, voltage.vcam);
		reg_mask = BITFMASK(MC13783_REGSET_VCAM);
		reg = REG_REGULATOR_SETTING_0;
		break;
	case REGU_VVIB:
		if ((voltage.vvib < VVIB_1_3V) || (voltage.vvib > VVIB_3V)) {
			return PMIC_PARAMETER_ERROR;
		}
		reg_val = BITFVAL(MC13783_REGSET_VVIB, voltage.vvib);
		reg_mask = BITFMASK(MC13783_REGSET_VVIB);
		reg = REG_REGULATOR_SETTING_1;
		break;
	case REGU_VRF1:
		if ((voltage.vrf1 < VRF1_1_5V) || (voltage.vrf1 > VRF1_2_775V)) {
			return PMIC_PARAMETER_ERROR;
		}
		reg_val = BITFVAL(MC13783_REGSET_VRF1, voltage.vrf1);
		reg_mask = BITFMASK(MC13783_REGSET_VRF1);
		reg = REG_REGULATOR_SETTING_1;
		break;
	case REGU_VRF2:
		if ((voltage.vrf2 < VRF2_1_5V) || (voltage.vrf2 > VRF2_2_775V)) {
			return PMIC_PARAMETER_ERROR;
		}
		reg_val = BITFVAL(MC13783_REGSET_VRF2, voltage.vrf2);
		reg_mask = BITFMASK(MC13783_REGSET_VRF2);
		reg = REG_REGULATOR_SETTING_1;
		break;
	case REGU_VMMC1:
		if ((voltage.vmmc1 < VMMC1_1_6V) || (voltage.vmmc1 > VMMC1_3V)) {
			return PMIC_PARAMETER_ERROR;
		}
		reg_val = BITFVAL(MC13783_REGSET_VMMC1, voltage.vmmc1);
		reg_mask = BITFMASK(MC13783_REGSET_VMMC1);
		reg = REG_REGULATOR_SETTING_1;
		break;
	case REGU_VMMC2:
		if ((voltage.vmmc2 < VMMC2_1_6V) || (voltage.vmmc2 > VMMC2_3V)) {
			return PMIC_PARAMETER_ERROR;
		}
		reg_val = BITFVAL(MC13783_REGSET_VMMC2, voltage.vmmc2);
		reg_mask = BITFMASK(MC13783_REGSET_VMMC2);
		reg = REG_REGULATOR_SETTING_1;
		break;
	case REGU_VAUDIO:
	case REGU_VIOHI:
	case REGU_VRFBG:
	case REGU_GPO1:
	case REGU_GPO2:
	case REGU_GPO3:
	case REGU_GPO4:
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(reg, reg_val, reg_mask));

	return PMIC_SUCCESS;
}

/*!
 * This function retrives the regulator output voltage.
 *
 * @param        regulator    The regulator to be truned off.
 * @param        voltage      Pointer to regulator output voltage.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_regulator_get_voltage(t_pmic_regulator regulator,
					     t_regulator_voltage *voltage)
{
	unsigned int reg_val = 0;

	if (regulator == SW_SW1A) {
		CHECK_ERROR(pmic_read_reg(REG_SWITCHERS_0,
					  &reg_val, PMIC_ALL_BITS));
	} else if (regulator == SW_SW1B) {
		CHECK_ERROR(pmic_read_reg(REG_SWITCHERS_1,
					  &reg_val, PMIC_ALL_BITS));
	} else if (regulator == SW_SW2A) {
		CHECK_ERROR(pmic_read_reg(REG_SWITCHERS_2,
					  &reg_val, PMIC_ALL_BITS));
	} else if (regulator == SW_SW2B) {
		CHECK_ERROR(pmic_read_reg(REG_SWITCHERS_3,
					  &reg_val, PMIC_ALL_BITS));
	} else if (regulator == SW_SW3) {
		CHECK_ERROR(pmic_read_reg(REG_SWITCHERS_5,
					  &reg_val, PMIC_ALL_BITS));
	} else if ((regulator == REGU_VIOLO) || (regulator == REGU_VDIG) ||
		   (regulator == REGU_VGEN) ||
		   (regulator == REGU_VRFDIG) ||
		   (regulator == REGU_VRFREF) ||
		   (regulator == REGU_VRFCP) ||
		   (regulator == REGU_VSIM) ||
		   (regulator == REGU_VESIM) || (regulator == REGU_VCAM)) {
		CHECK_ERROR(pmic_read_reg(REG_REGULATOR_SETTING_0,
					  &reg_val, PMIC_ALL_BITS));
	} else if ((regulator == REGU_VVIB) || (regulator == REGU_VRF1) ||
		   (regulator == REGU_VRF2) ||
		   (regulator == REGU_VMMC1) || (regulator == REGU_VMMC2)) {
		CHECK_ERROR(pmic_read_reg(REG_REGULATOR_SETTING_1,
					  &reg_val, PMIC_ALL_BITS));
	}

	switch (regulator) {
	case SW_SW1A:
		voltage->sw1a = BITFEXT(reg_val, MC13783_SWSET_SW1A);
		break;
	case SW_SW1B:
		voltage->sw1b = BITFEXT(reg_val, MC13783_SWSET_SW1B);
		break;
	case SW_SW2A:
		voltage->sw2a = BITFEXT(reg_val, MC13783_SWSET_SW2A);
		break;
	case SW_SW2B:
		voltage->sw2b = BITFEXT(reg_val, MC13783_SWSET_SW2B);
		break;
	case SW_SW3:
		voltage->sw3 = BITFEXT(reg_val, MC13783_SWSET_SW3);
		break;
	case REGU_VIOLO:
		voltage->violo = BITFEXT(reg_val, MC13783_REGSET_VIOLO);
		break;
	case REGU_VDIG:
		voltage->vdig = BITFEXT(reg_val, MC13783_REGSET_VDIG);
		break;
	case REGU_VGEN:
		voltage->vgen = BITFEXT(reg_val, MC13783_REGSET_VGEN);
		break;
	case REGU_VRFDIG:
		voltage->vrfdig = BITFEXT(reg_val, MC13783_REGSET_VRFDIG);
		break;
	case REGU_VRFREF:
		voltage->vrfref = BITFEXT(reg_val, MC13783_REGSET_VRFREF);
		break;
	case REGU_VRFCP:
		voltage->vrfcp = BITFEXT(reg_val, MC13783_REGSET_VRFCP);
		break;
	case REGU_VSIM:
		voltage->vsim = BITFEXT(reg_val, MC13783_REGSET_VSIM);
		break;
	case REGU_VESIM:
		voltage->vesim = BITFEXT(reg_val, MC13783_REGSET_VESIM);
		break;
	case REGU_VCAM:
		voltage->vcam = BITFEXT(reg_val, MC13783_REGSET_VCAM);
		break;
	case REGU_VVIB:
		voltage->vvib = BITFEXT(reg_val, MC13783_REGSET_VVIB);
		break;
	case REGU_VRF1:
		voltage->vrf1 = BITFEXT(reg_val, MC13783_REGSET_VRF1);
		break;
	case REGU_VRF2:
		voltage->vrf2 = BITFEXT(reg_val, MC13783_REGSET_VRF2);
		break;
	case REGU_VMMC1:
		voltage->vmmc1 = BITFEXT(reg_val, MC13783_REGSET_VMMC1);
		break;
	case REGU_VMMC2:
		voltage->vmmc2 = BITFEXT(reg_val, MC13783_REGSET_VMMC2);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	return PMIC_SUCCESS;
}

/*!
 * This function sets the DVS voltage
 *
 * @param        regulator    The regulator to be configured.
 * @param        dvs          The switch Dynamic Voltage Scaling
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_switcher_set_dvs(t_pmic_regulator regulator,
					t_regulator_voltage dvs)
{
	unsigned int reg_val = 0, reg_mask = 0;
	unsigned int reg;

	switch (regulator) {
	case SW_SW1A:
		if ((dvs.sw1a < SW1A_0_9V) || (dvs.sw1a > SW1A_2_2V)) {
			return PMIC_PARAMETER_ERROR;
		}
		reg_val = BITFVAL(MC13783_SWSET_SW1A_DVS, dvs.sw1a);
		reg_mask = BITFMASK(MC13783_SWSET_SW1A_DVS);
		reg = REG_SWITCHERS_0;
		break;
	case SW_SW1B:
		if ((dvs.sw1b < SW1B_0_9V) || (dvs.sw1b > SW1B_2_2V)) {
			return PMIC_PARAMETER_ERROR;
		}
		reg_val = BITFVAL(MC13783_SWSET_SW1B_DVS, dvs.sw1b);
		reg_mask = BITFMASK(MC13783_SWSET_SW1B_DVS);
		reg = REG_SWITCHERS_1;
		break;
	case SW_SW2A:
		if ((dvs.sw2a < SW2A_0_9V) || (dvs.sw2a > SW2A_2_2V)) {
			return PMIC_PARAMETER_ERROR;
		}
		reg_val = BITFVAL(MC13783_SWSET_SW2A_DVS, dvs.sw2a);
		reg_mask = BITFMASK(MC13783_SWSET_SW2A_DVS);
		reg = REG_SWITCHERS_2;
		break;
	case SW_SW2B:
		if ((dvs.sw2b < SW2B_0_9V) || (dvs.sw2b > SW2B_2_2V)) {
			return PMIC_PARAMETER_ERROR;
		}
		reg_val = BITFVAL(MC13783_SWSET_SW2B_DVS, dvs.sw2b);
		reg_mask = BITFMASK(MC13783_SWSET_SW2B_DVS);
		reg = REG_SWITCHERS_3;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(reg, reg_val, reg_mask));

	return PMIC_SUCCESS;
}

/*!
 * This function gets the DVS voltage
 *
 * @param        regulator    The regulator to be handled.
 * @param        dvs          The switch Dynamic Voltage Scaling
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_switcher_get_dvs(t_pmic_regulator regulator,
					t_regulator_voltage *dvs)
{
	unsigned int reg_val = 0, reg_mask = 0;
	unsigned int reg;

	switch (regulator) {
	case SW_SW1A:
		reg_mask = BITFMASK(MC13783_SWSET_SW1A_DVS);
		reg = REG_SWITCHERS_0;
		break;
	case SW_SW1B:
		reg_mask = BITFMASK(MC13783_SWSET_SW1B_DVS);
		reg = REG_SWITCHERS_1;
		break;
	case SW_SW2A:
		reg_mask = BITFMASK(MC13783_SWSET_SW2A_DVS);
		reg = REG_SWITCHERS_2;
		break;
	case SW_SW2B:
		reg_mask = BITFMASK(MC13783_SWSET_SW2B_DVS);
		reg = REG_SWITCHERS_3;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_read_reg(reg, &reg_val, reg_mask));

	switch (regulator) {
	case SW_SW1A:
		*dvs = (t_regulator_voltage) BITFEXT(reg_val,
						     MC13783_SWSET_SW1A_DVS);
		break;
	case SW_SW1B:
		*dvs = (t_regulator_voltage) BITFEXT(reg_val,
						     MC13783_SWSET_SW1B_DVS);
		break;
	case SW_SW2A:
		*dvs = (t_regulator_voltage) BITFEXT(reg_val,
						     MC13783_SWSET_SW2A_DVS);
		break;
	case SW_SW2B:
		*dvs = (t_regulator_voltage) BITFEXT(reg_val,
						     MC13783_SWSET_SW2B_DVS);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	return PMIC_SUCCESS;
}

/*!
 * This function sets the standiby voltage
 *
 * @param        regulator    The regulator to be configured.
 * @param        stby         The switch standby voltage
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_switcher_set_stby(t_pmic_regulator regulator,
					 t_regulator_voltage stby)
{
	unsigned int reg_val = 0, reg_mask = 0;
	unsigned int reg;

	switch (regulator) {
	case SW_SW1A:
		if ((stby.sw1a < SW1A_0_9V) || (stby.sw1a > SW1A_2_2V)) {
			return PMIC_PARAMETER_ERROR;
		}
		reg_val = BITFVAL(MC13783_SWSET_SW1A_STDBY, stby.sw1a);
		reg_mask = BITFMASK(MC13783_SWSET_SW1A_STDBY);
		reg = REG_SWITCHERS_0;
		break;
	case SW_SW1B:
		if ((stby.sw1b < SW1B_0_9V) || (stby.sw1b > SW1B_2_2V)) {
			return PMIC_PARAMETER_ERROR;
		}
		reg_val = BITFVAL(MC13783_SWSET_SW1B_STDBY, stby.sw1b);
		reg_mask = BITFMASK(MC13783_SWSET_SW1B_STDBY);
		reg = REG_SWITCHERS_1;
		break;
	case SW_SW2A:
		if ((stby.sw2a < SW2A_0_9V) || (stby.sw2a > SW2A_2_2V)) {
			return PMIC_PARAMETER_ERROR;
		}
		reg_val = BITFVAL(MC13783_SWSET_SW2A_STDBY, stby.sw2a);
		reg_mask = BITFMASK(MC13783_SWSET_SW2A_STDBY);
		reg = REG_SWITCHERS_2;
		break;
	case SW_SW2B:
		if ((stby.sw2b < SW2B_0_9V) || (stby.sw2b > SW2B_2_2V)) {
			return PMIC_PARAMETER_ERROR;
		}
		reg_val = BITFVAL(MC13783_SWSET_SW2B_STDBY, stby.sw2b);
		reg_mask = BITFMASK(MC13783_SWSET_SW2B_STDBY);
		reg = REG_SWITCHERS_3;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(reg, reg_val, reg_mask));

	return PMIC_SUCCESS;
}

/*!
 * This function gets the standiby voltage
 *
 * @param        regulator    The regulator to be handled.
 * @param        stby         The switch standby voltage
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_switcher_get_stby(t_pmic_regulator regulator,
					 t_regulator_voltage *stby)
{
	unsigned int reg_val = 0, reg_mask = 0;
	unsigned int reg;

	switch (regulator) {
	case SW_SW1A:
		reg_mask = BITFMASK(MC13783_SWSET_SW1A_STDBY);
		reg = REG_SWITCHERS_0;
		break;
	case SW_SW1B:
		reg_mask = BITFMASK(MC13783_SWSET_SW1B_STDBY);
		reg = REG_SWITCHERS_1;
		break;
	case SW_SW2A:
		reg_mask = BITFMASK(MC13783_SWSET_SW2A_STDBY);
		reg = REG_SWITCHERS_2;
		break;
	case SW_SW2B:
		reg_mask = BITFMASK(MC13783_SWSET_SW2B_STDBY);
		reg = REG_SWITCHERS_3;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_read_reg(reg, &reg_val, reg_mask));

	switch (regulator) {
	case SW_SW1A:
		*stby = (t_regulator_voltage) BITFEXT(reg_val,
						      MC13783_SWSET_SW1A_STDBY);
		break;
	case SW_SW1B:
		*stby = (t_regulator_voltage) BITFEXT(reg_val,
						      MC13783_SWSET_SW1B_STDBY);
		break;
	case SW_SW2A:
		*stby = (t_regulator_voltage) BITFEXT(reg_val,
						      MC13783_SWSET_SW2A_STDBY);
		break;
	case SW_SW2B:
		*stby = (t_regulator_voltage) BITFEXT(reg_val,
						      MC13783_SWSET_SW2B_STDBY);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	return PMIC_SUCCESS;
}

/*!
 * This function sets the switchers mode.
 *
 * @param        regulator    The regulator to be configured.
 * @param        mode	      The switcher mode
 * @param        stby	      Switch between main and standby.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_switcher_set_mode(t_pmic_regulator regulator,
					 t_regulator_sw_mode mode, bool stby)
{
	unsigned int reg_val = 0, reg_mask = 0;
	unsigned int reg;
	unsigned int l_mode;

	if (mode == SYNC_RECT) {
		l_mode = MC13783_SWCTRL_SW_MODE_SYNC_RECT_EN;
	} else if (mode == NO_PULSE_SKIP) {
		l_mode = MC13783_SWCTRL_SW_MODE_PULSE_NO_SKIP_EN;
	} else if (mode == PULSE_SKIP) {
		l_mode = MC13783_SWCTRL_SW_MODE_PULSE_SKIP_EN;
	} else if (mode == LOW_POWER) {
		l_mode = MC13783_SWCTRL_SW_MODE_LOW_POWER_EN;
	} else {
		return PMIC_PARAMETER_ERROR;
	}

	switch (regulator) {
	case SW_SW1A:
		if (stby) {
			reg_val =
			    BITFVAL(MC13783_SWCTRL_SW1A_STBY_MODE, l_mode);
			reg_mask = BITFMASK(MC13783_SWCTRL_SW1A_STBY_MODE);
		} else {
			reg_val = BITFVAL(MC13783_SWCTRL_SW1A_MODE, l_mode);
			reg_mask = BITFMASK(MC13783_SWCTRL_SW1A_MODE);
		}
		reg = REG_SWITCHERS_4;
		break;
	case SW_SW1B:
		if (stby) {
			reg_val =
			    BITFVAL(MC13783_SWCTRL_SW1B_STBY_MODE, l_mode);
			reg_mask = BITFMASK(MC13783_SWCTRL_SW1B_STBY_MODE);
		} else {
			reg_val = BITFVAL(MC13783_SWCTRL_SW1B_MODE, l_mode);
			reg_mask = BITFMASK(MC13783_SWCTRL_SW1B_MODE);
		}
		reg = REG_SWITCHERS_4;
		break;
	case SW_SW2A:
		if (stby) {
			reg_val =
			    BITFVAL(MC13783_SWCTRL_SW2A_STBY_MODE, l_mode);
			reg_mask = BITFMASK(MC13783_SWCTRL_SW2A_STBY_MODE);
		} else {
			reg_val = BITFVAL(MC13783_SWCTRL_SW2A_MODE, l_mode);
			reg_mask = BITFMASK(MC13783_SWCTRL_SW2A_MODE);
		}
		reg = REG_SWITCHERS_5;
		break;
	case SW_SW2B:
		if (stby) {
			reg_val =
			    BITFVAL(MC13783_SWCTRL_SW2B_STBY_MODE, l_mode);
			reg_mask = BITFMASK(MC13783_SWCTRL_SW2B_STBY_MODE);
		} else {
			reg_val = BITFVAL(MC13783_SWCTRL_SW2B_MODE, l_mode);
			reg_mask = BITFMASK(MC13783_SWCTRL_SW2B_MODE);
		}
		reg = REG_SWITCHERS_5;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(reg, reg_val, reg_mask));

	return PMIC_SUCCESS;
}

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
					 t_regulator_sw_mode *mode, bool stby)
{
	unsigned int reg_val = 0, reg_mask = 0;
	unsigned int reg = 0;
	unsigned int l_mode = 0;

	switch (regulator) {
	case SW_SW1A:
		if (stby) {
			reg_mask = BITFMASK(MC13783_SWCTRL_SW1A_STBY_MODE);
		} else {
			reg_mask = BITFMASK(MC13783_SWCTRL_SW1A_MODE);
		}
		reg = REG_SWITCHERS_4;
		break;
	case SW_SW1B:
		if (stby) {
			reg_mask = BITFMASK(MC13783_SWCTRL_SW1B_STBY_MODE);
		} else {
			reg_mask = BITFMASK(MC13783_SWCTRL_SW1B_MODE);
		}
		reg = REG_SWITCHERS_4;
		break;
	case SW_SW2A:
		if (stby) {
			reg_mask = BITFMASK(MC13783_SWCTRL_SW2A_STBY_MODE);
		} else {
			reg_mask = BITFMASK(MC13783_SWCTRL_SW2A_MODE);
		}
		reg = REG_SWITCHERS_5;
		break;
	case SW_SW2B:
		if (stby) {
			reg_mask = BITFMASK(MC13783_SWCTRL_SW2B_STBY_MODE);
		} else {
			reg_mask = BITFMASK(MC13783_SWCTRL_SW2B_MODE);
		}
		reg = REG_SWITCHERS_5;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_read_reg(reg, &reg_val, reg_mask));

	switch (regulator) {
	case SW_SW1A:
		if (stby) {
			l_mode =
			    BITFEXT(reg_val, MC13783_SWCTRL_SW1A_STBY_MODE);
		} else {
			l_mode = BITFEXT(reg_val, MC13783_SWCTRL_SW1A_MODE);
		}
		break;
	case SW_SW1B:
		if (stby) {
			l_mode =
			    BITFEXT(reg_val, MC13783_SWCTRL_SW1B_STBY_MODE);
		} else {
			l_mode = BITFEXT(reg_val, MC13783_SWCTRL_SW1B_MODE);
		}
		break;
	case SW_SW2A:
		if (stby) {
			l_mode =
			    BITFEXT(reg_val, MC13783_SWCTRL_SW2A_STBY_MODE);
		} else {
			l_mode = BITFEXT(reg_val, MC13783_SWCTRL_SW2A_MODE);
		}
		break;
	case SW_SW2B:
		if (stby) {
			l_mode =
			    BITFEXT(reg_val, MC13783_SWCTRL_SW2B_STBY_MODE);
		} else {
			l_mode = BITFEXT(reg_val, MC13783_SWCTRL_SW2B_MODE);
		}
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	if (l_mode == MC13783_SWCTRL_SW_MODE_SYNC_RECT_EN) {
		*mode = SYNC_RECT;
	} else if (l_mode == MC13783_SWCTRL_SW_MODE_PULSE_NO_SKIP_EN) {
		*mode = NO_PULSE_SKIP;
	} else if (l_mode == MC13783_SWCTRL_SW_MODE_PULSE_SKIP_EN) {
		*mode = PULSE_SKIP;
	} else if (l_mode == MC13783_SWCTRL_SW_MODE_LOW_POWER_EN) {
		*mode = LOW_POWER;
	} else {
		return PMIC_PARAMETER_ERROR;
	}

	return PMIC_SUCCESS;
}

/*!
 * This function sets the switch dvs speed
 *
 * @param        regulator    The regulator to be configured.
 * @param        speed	      The dvs speed.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_switcher_set_dvs_speed(t_pmic_regulator regulator,
					      t_switcher_dvs_speed speed)
{
	unsigned int reg_val = 0, reg_mask = 0;
	unsigned int reg;
	if (speed > 3 || speed < 0) {
		return PMIC_PARAMETER_ERROR;
	}

	switch (regulator) {
	case SW_SW1A:
		reg_val = BITFVAL(MC13783_SWCTRL_SW1A_DVS_SPEED, speed);
		reg_mask = BITFMASK(MC13783_SWCTRL_SW1A_DVS_SPEED);
		reg = REG_SWITCHERS_4;
		break;
	case SW_SW1B:
		reg_val = BITFVAL(MC13783_SWCTRL_SW2B_DVS_SPEED, speed);
		reg_mask = BITFMASK(MC13783_SWCTRL_SW1B_DVS_SPEED);
		reg = REG_SWITCHERS_4;
		break;
	case SW_SW2A:
		reg_val = BITFVAL(MC13783_SWCTRL_SW2A_DVS_SPEED, speed);
		reg_mask = BITFMASK(MC13783_SWCTRL_SW2A_DVS_SPEED);
		reg = REG_SWITCHERS_5;
		break;
	case SW_SW2B:
		reg_val = BITFVAL(MC13783_SWCTRL_SW2B_DVS_SPEED, speed);
		reg_mask = BITFMASK(MC13783_SWCTRL_SW2B_DVS_SPEED);
		reg = REG_SWITCHERS_5;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(reg, reg_val, reg_mask));

	return PMIC_SUCCESS;
}

/*!
 * This function gets the switch dvs speed
 *
 * @param        regulator    The regulator to be handled.
 * @param        speed        The dvs speed.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_switcher_get_dvs_speed(t_pmic_regulator regulator,
					      t_switcher_dvs_speed *speed)
{
	unsigned int reg_val = 0, reg_mask = 0;
	unsigned int reg;

	switch (regulator) {
	case SW_SW1A:
		reg_mask = BITFMASK(MC13783_SWCTRL_SW1A_DVS_SPEED);
		reg = REG_SWITCHERS_4;
		break;
	case SW_SW1B:
		reg_mask = BITFMASK(MC13783_SWCTRL_SW1B_DVS_SPEED);
		reg = REG_SWITCHERS_4;
		break;
	case SW_SW2A:
		reg_mask = BITFMASK(MC13783_SWCTRL_SW2A_DVS_SPEED);
		reg = REG_SWITCHERS_5;
		break;
	case SW_SW2B:
		reg_mask = BITFMASK(MC13783_SWCTRL_SW2B_DVS_SPEED);
		reg = REG_SWITCHERS_5;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_read_reg(reg, &reg_val, reg_mask));

	switch (regulator) {
	case SW_SW1A:
		*speed = BITFEXT(reg_val, MC13783_SWCTRL_SW1A_DVS_SPEED);
		break;
	case SW_SW1B:
		*speed = BITFEXT(reg_val, MC13783_SWCTRL_SW1B_DVS_SPEED);
		break;
	case SW_SW2A:
		*speed = BITFEXT(reg_val, MC13783_SWCTRL_SW2A_DVS_SPEED);
		break;
	case SW_SW2B:
		*speed = BITFEXT(reg_val, MC13783_SWCTRL_SW2B_DVS_SPEED);
		break;
	default:
		break;
	}

	return PMIC_SUCCESS;
}

/*!
 * This function sets the switch panic mode
 *
 * @param        regulator    The regulator to be configured.
 * @param        panic_mode   Enable or disable panic mode
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_switcher_set_panic_mode(t_pmic_regulator regulator,
					       bool panic_mode)
{
	unsigned int reg_val = 0, reg_mask = 0;
	unsigned int reg;

	switch (regulator) {
	case SW_SW1A:
		reg_val = BITFVAL(MC13783_SWCTRL_SW1A_PANIC_MODE, panic_mode);
		reg_mask = BITFMASK(MC13783_SWCTRL_SW1A_PANIC_MODE);
		reg = REG_SWITCHERS_4;
		break;
	case SW_SW1B:
		reg_val = BITFVAL(MC13783_SWCTRL_SW2B_PANIC_MODE, panic_mode);
		reg_mask = BITFMASK(MC13783_SWCTRL_SW1B_PANIC_MODE);
		reg = REG_SWITCHERS_4;
		break;
	case SW_SW2A:
		reg_val = BITFVAL(MC13783_SWCTRL_SW2A_PANIC_MODE, panic_mode);
		reg_mask = BITFMASK(MC13783_SWCTRL_SW2A_PANIC_MODE);
		reg = REG_SWITCHERS_5;
		break;
	case SW_SW2B:
		reg_val = BITFVAL(MC13783_SWCTRL_SW2B_PANIC_MODE, panic_mode);
		reg_mask = BITFMASK(MC13783_SWCTRL_SW2B_PANIC_MODE);
		reg = REG_SWITCHERS_5;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(reg, reg_val, reg_mask));

	return PMIC_SUCCESS;
}

/*!
 * This function gets the switch panic mode
 *
 * @param        regulator    The regulator to be handled
 * @param        panic_mode   Enable or disable panic mode
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_switcher_get_panic_mode(t_pmic_regulator regulator,
					       bool *panic_mode)
{
	unsigned int reg_val = 0, reg_mask = 0;
	unsigned int reg;

	switch (regulator) {
	case SW_SW1A:
		reg_mask = BITFMASK(MC13783_SWCTRL_SW1A_PANIC_MODE);
		reg = REG_SWITCHERS_4;
		break;
	case SW_SW1B:
		reg_mask = BITFMASK(MC13783_SWCTRL_SW1B_PANIC_MODE);
		reg = REG_SWITCHERS_4;
		break;
	case SW_SW2A:
		reg_mask = BITFMASK(MC13783_SWCTRL_SW2A_PANIC_MODE);
		reg = REG_SWITCHERS_5;
		break;
	case SW_SW2B:
		reg_mask = BITFMASK(MC13783_SWCTRL_SW2B_PANIC_MODE);
		reg = REG_SWITCHERS_5;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_read_reg(reg, &reg_val, reg_mask));

	switch (regulator) {
	case SW_SW1A:
		*panic_mode = BITFEXT(reg_val, MC13783_SWCTRL_SW1A_PANIC_MODE);
		break;
	case SW_SW1B:
		*panic_mode = BITFEXT(reg_val, MC13783_SWCTRL_SW1B_PANIC_MODE);
		break;
	case SW_SW2A:
		*panic_mode = BITFEXT(reg_val, MC13783_SWCTRL_SW2A_PANIC_MODE);
		break;
	case SW_SW2B:
		*panic_mode = BITFEXT(reg_val, MC13783_SWCTRL_SW2B_PANIC_MODE);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	return PMIC_SUCCESS;
}

/*!
 * This function sets the switch softstart mode
 *
 * @param        regulator    The regulator to be configured.
 * @param        softstart    Enable or disable softstart.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_switcher_set_softstart(t_pmic_regulator regulator,
					      bool softstart)
{
	unsigned int reg_val = 0, reg_mask = 0;
	unsigned int reg;

	switch (regulator) {
	case SW_SW1A:
		reg_val = BITFVAL(MC13783_SWCTRL_SW1A_SOFTSTART, softstart);
		reg_mask = BITFMASK(MC13783_SWCTRL_SW1A_SOFTSTART);
		reg = REG_SWITCHERS_4;
		break;
	case SW_SW1B:
		reg_val = BITFVAL(MC13783_SWCTRL_SW2B_SOFTSTART, softstart);
		reg_mask = BITFMASK(MC13783_SWCTRL_SW1B_SOFTSTART);
		reg = REG_SWITCHERS_4;
		break;
	case SW_SW2A:
		reg_val = BITFVAL(MC13783_SWCTRL_SW2A_SOFTSTART, softstart);
		reg_mask = BITFMASK(MC13783_SWCTRL_SW2A_SOFTSTART);
		reg = REG_SWITCHERS_5;
		break;
	case SW_SW2B:
		reg_val = BITFVAL(MC13783_SWCTRL_SW2B_SOFTSTART, softstart);
		reg_mask = BITFMASK(MC13783_SWCTRL_SW2B_SOFTSTART);
		reg = REG_SWITCHERS_5;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(reg, reg_val, reg_mask));

	return PMIC_SUCCESS;
}

/*!
 * This function gets the switch softstart mode
 *
 * @param        regulator    The regulator to be handled
 * @param        softstart    Enable or disable softstart.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_switcher_get_softstart(t_pmic_regulator regulator,
					      bool *softstart)
{
	unsigned int reg_val = 0, reg_mask = 0;
	unsigned int reg;

	switch (regulator) {
	case SW_SW1A:
		reg_mask = BITFMASK(MC13783_SWCTRL_SW1A_SOFTSTART);
		reg = REG_SWITCHERS_4;
		break;
	case SW_SW1B:
		reg_mask = BITFMASK(MC13783_SWCTRL_SW1B_SOFTSTART);
		reg = REG_SWITCHERS_4;
		break;
	case SW_SW2A:
		reg_mask = BITFMASK(MC13783_SWCTRL_SW2A_SOFTSTART);
		reg = REG_SWITCHERS_5;
		break;
	case SW_SW2B:
		reg_mask = BITFMASK(MC13783_SWCTRL_SW2B_SOFTSTART);
		reg = REG_SWITCHERS_5;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_read_reg(reg, &reg_val, reg_mask));

	switch (regulator) {
	case SW_SW1A:
		*softstart = BITFEXT(reg_val, MC13783_SWCTRL_SW1A_SOFTSTART);
		break;
	case SW_SW1B:
		*softstart = BITFEXT(reg_val, MC13783_SWCTRL_SW2B_SOFTSTART);
		break;
	case SW_SW2A:
		*softstart = BITFEXT(reg_val, MC13783_SWCTRL_SW2A_SOFTSTART);
		break;
	case SW_SW2B:
		*softstart = BITFEXT(reg_val, MC13783_SWCTRL_SW2B_SOFTSTART);
		break;
	default:
		break;
	}

	return PMIC_SUCCESS;
}

/*!
 * This function sets the PLL multiplication factor
 *
 * @param        regulator    The regulator to be configured.
 * @param        factor       The multiplication factor.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_switcher_set_factor(t_pmic_regulator regulator,
					   t_switcher_factor factor)
{
	unsigned int reg_val = 0, reg_mask = 0;

	if (regulator != SW_PLL) {
		return PMIC_PARAMETER_ERROR;
	}
	if (factor < FACTOR_28 || factor > FACTOR_35) {
		return PMIC_PARAMETER_ERROR;
	}
	reg_val = BITFVAL(MC13783_SWCTRL_PLL_FACTOR, factor);
	reg_mask = BITFMASK(MC13783_SWCTRL_PLL_FACTOR);

	CHECK_ERROR(pmic_write_reg(REG_SWITCHERS_4, reg_val, reg_mask));

	return PMIC_SUCCESS;
}

/*!
 * This function gets the PLL multiplication factor
 *
 * @param        regulator    The regulator to be handled
 * @param        factor       The multiplication factor.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_switcher_get_factor(t_pmic_regulator regulator,
					   t_switcher_factor *factor)
{
	unsigned int reg_val = 0, reg_mask = 0;

	if (regulator != SW_PLL) {
		return PMIC_PARAMETER_ERROR;
	}
	reg_mask = BITFMASK(MC13783_SWCTRL_PLL_FACTOR);

	CHECK_ERROR(pmic_read_reg(REG_SWITCHERS_4, &reg_val, reg_mask));

	*factor = BITFEXT(reg_val, MC13783_SWCTRL_PLL_FACTOR);

	return PMIC_SUCCESS;
}

/*!
 * This function enables or disables low power mode.
 *
 * @param        regulator    The regulator to be configured.
 * @param        lp_mode      Select nominal or low power mode.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_regulator_set_lp_mode(t_pmic_regulator regulator,
					     t_regulator_lp_mode lp_mode)
{
	unsigned int reg_val = 0, reg_mask = 0;
	unsigned int reg;
	unsigned int l_mode, l_stby;

	if (lp_mode == LOW_POWER_DISABLED) {
		l_mode = MC13783_REGTRL_LP_MODE_DISABLE;
		l_stby = MC13783_REGTRL_STBY_MODE_DISABLE;
	} else if (lp_mode == LOW_POWER_CTRL_BY_PIN) {
		l_mode = MC13783_REGTRL_LP_MODE_DISABLE;
		l_stby = MC13783_REGTRL_STBY_MODE_ENABLE;
	} else if (lp_mode == LOW_POWER_EN) {
		l_mode = MC13783_REGTRL_LP_MODE_ENABLE;
		l_stby = MC13783_REGTRL_STBY_MODE_DISABLE;
	} else if (lp_mode == LOW_POWER_AND_LOW_POWER_CTRL_BY_PIN) {
		l_mode = MC13783_REGTRL_LP_MODE_ENABLE;
		l_stby = MC13783_REGTRL_STBY_MODE_ENABLE;
	} else {
		return PMIC_PARAMETER_ERROR;
	}

	switch (regulator) {
	case SW_SW3:
		reg_val = BITFVAL(MC13783_SWCTRL_SW3_MODE, l_mode) |
		    BITFVAL(MC13783_SWCTRL_SW3_STBY, l_stby);
		reg_mask = BITFMASK(MC13783_SWCTRL_SW3_MODE) |
		    BITFMASK(MC13783_SWCTRL_SW3_STBY);
		reg = REG_SWITCHERS_5;
		break;
	case REGU_VAUDIO:
		reg_val = BITFVAL(MC13783_REGCTRL_VAUDIO_MODE, l_mode) |
		    BITFVAL(MC13783_REGCTRL_VAUDIO_STBY, l_stby);
		reg_mask = BITFMASK(MC13783_REGCTRL_VAUDIO_MODE) |
		    BITFMASK(MC13783_REGCTRL_VAUDIO_STBY);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VIOHI:
		reg_val = BITFVAL(MC13783_REGCTRL_VIOHI_MODE, l_mode) |
		    BITFVAL(MC13783_REGCTRL_VIOHI_STBY, l_stby);
		reg_mask = BITFMASK(MC13783_REGCTRL_VIOHI_MODE) |
		    BITFMASK(MC13783_REGCTRL_VIOHI_STBY);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VIOLO:
		reg_val = BITFVAL(MC13783_REGCTRL_VIOLO_MODE, l_mode) |
		    BITFVAL(MC13783_REGCTRL_VIOLO_STBY, l_stby);
		reg_mask = BITFMASK(MC13783_REGCTRL_VIOLO_MODE) |
		    BITFMASK(MC13783_REGCTRL_VIOLO_STBY);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VDIG:
		reg_val = BITFVAL(MC13783_REGCTRL_VDIG_MODE, l_mode) |
		    BITFVAL(MC13783_REGCTRL_VDIG_STBY, l_stby);
		reg_mask = BITFMASK(MC13783_REGCTRL_VDIG_MODE) |
		    BITFMASK(MC13783_REGCTRL_VDIG_STBY);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VGEN:
		reg_val = BITFVAL(MC13783_REGCTRL_VGEN_MODE, l_mode) |
		    BITFVAL(MC13783_REGCTRL_VGEN_STBY, l_stby);
		reg_mask = BITFMASK(MC13783_REGCTRL_VGEN_MODE) |
		    BITFMASK(MC13783_REGCTRL_VGEN_STBY);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VRFDIG:
		reg_val = BITFVAL(MC13783_REGCTRL_VRFDIG_MODE, l_mode) |
		    BITFVAL(MC13783_REGCTRL_VRFDIG_STBY, l_stby);
		reg_mask = BITFMASK(MC13783_REGCTRL_VRFDIG_MODE) |
		    BITFMASK(MC13783_REGCTRL_VRFDIG_STBY);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VRFREF:
		reg_val = BITFVAL(MC13783_REGCTRL_VRFREF_MODE, l_mode) |
		    BITFVAL(MC13783_REGCTRL_VRFREF_STBY, l_stby);
		reg_mask = BITFMASK(MC13783_REGCTRL_VRFREF_MODE) |
		    BITFMASK(MC13783_REGCTRL_VRFREF_STBY);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VRFCP:
		reg_val = BITFVAL(MC13783_REGCTRL_VRFCP_MODE, l_mode) |
		    BITFVAL(MC13783_REGCTRL_VRFCP_STBY, l_stby);
		reg_mask = BITFMASK(MC13783_REGCTRL_VRFCP_MODE) |
		    BITFMASK(MC13783_REGCTRL_VRFCP_STBY);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VSIM:
		reg_val = BITFVAL(MC13783_REGCTRL_VSIM_MODE, l_mode) |
		    BITFVAL(MC13783_REGCTRL_VSIM_STBY, l_stby);
		reg_mask = BITFMASK(MC13783_REGCTRL_VSIM_MODE) |
		    BITFMASK(MC13783_REGCTRL_VSIM_STBY);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VESIM:
		reg_val = BITFVAL(MC13783_REGCTRL_VESIM_MODE, l_mode) |
		    BITFVAL(MC13783_REGCTRL_VESIM_STBY, l_stby);
		reg_mask = BITFMASK(MC13783_REGCTRL_VESIM_MODE) |
		    BITFMASK(MC13783_REGCTRL_VESIM_STBY);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VCAM:
		reg_val = BITFVAL(MC13783_REGCTRL_VCAM_MODE, l_mode) |
		    BITFVAL(MC13783_REGCTRL_VCAM_STBY, l_stby);
		reg_mask = BITFMASK(MC13783_REGCTRL_VCAM_MODE) |
		    BITFMASK(MC13783_REGCTRL_VCAM_STBY);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VRFBG:
		if ((lp_mode == LOW_POWER) ||
		    (lp_mode == LOW_POWER_AND_LOW_POWER_CTRL_BY_PIN)) {
			return PMIC_PARAMETER_ERROR;
		}
		reg_val = BITFVAL(MC13783_REGCTRL_VRFBG_STBY, l_mode);
		reg_mask = BITFMASK(MC13783_REGCTRL_VRFBG_STBY);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VRF1:
		reg_val = BITFVAL(MC13783_REGCTRL_VRF1_MODE, l_mode) |
		    BITFVAL(MC13783_REGCTRL_VRF1_STBY, l_stby);
		reg_mask = BITFMASK(MC13783_REGCTRL_VRF1_MODE) |
		    BITFMASK(MC13783_REGCTRL_VRF1_STBY);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VRF2:
		reg_val = BITFVAL(MC13783_REGCTRL_VRF2_MODE, l_mode) |
		    BITFVAL(MC13783_REGCTRL_VRF2_STBY, l_stby);
		reg_mask = BITFMASK(MC13783_REGCTRL_VRF2_MODE) |
		    BITFMASK(MC13783_REGCTRL_VRF2_STBY);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VMMC1:
		reg_val = BITFVAL(MC13783_REGCTRL_VMMC1_MODE, l_mode) |
		    BITFVAL(MC13783_REGCTRL_VMMC1_STBY, l_stby);
		reg_mask = BITFMASK(MC13783_REGCTRL_VMMC1_MODE) |
		    BITFMASK(MC13783_REGCTRL_VMMC1_STBY);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VMMC2:
		reg_val = BITFVAL(MC13783_REGCTRL_VMMC2_MODE, l_mode) |
		    BITFVAL(MC13783_REGCTRL_VMMC2_STBY, l_stby);
		reg_mask = BITFMASK(MC13783_REGCTRL_VMMC2_MODE) |
		    BITFMASK(MC13783_REGCTRL_VMMC2_STBY);
		reg = REG_REGULATOR_MODE_1;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(reg, reg_val, reg_mask));

	return PMIC_SUCCESS;
}

/*!
 * This function gets low power mode.
 *
 * @param        regulator    The regulator to be handled
 * @param        lp_mode      Select nominal or low power mode.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_regulator_get_lp_mode(t_pmic_regulator regulator,
					     t_regulator_lp_mode *lp_mode)
{
	unsigned int reg_val = 0, reg_mask = 0;
	unsigned int reg;
	unsigned int l_mode, l_stby;

	switch (regulator) {
	case SW_SW3:
		reg_mask = BITFMASK(MC13783_SWCTRL_SW3_MODE) |
		    BITFMASK(MC13783_SWCTRL_SW3_STBY);
		reg = REG_SWITCHERS_5;
		break;
	case REGU_VAUDIO:
		reg_mask = BITFMASK(MC13783_REGCTRL_VAUDIO_MODE) |
		    BITFMASK(MC13783_REGCTRL_VAUDIO_STBY);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VIOHI:
		reg_mask = BITFMASK(MC13783_REGCTRL_VIOHI_MODE) |
		    BITFMASK(MC13783_REGCTRL_VIOHI_STBY);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VIOLO:
		reg_mask = BITFMASK(MC13783_REGCTRL_VIOLO_MODE) |
		    BITFMASK(MC13783_REGCTRL_VIOLO_STBY);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VDIG:
		reg_mask = BITFMASK(MC13783_REGCTRL_VDIG_MODE) |
		    BITFMASK(MC13783_REGCTRL_VDIG_STBY);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VGEN:
		reg_mask = BITFMASK(MC13783_REGCTRL_VGEN_MODE) |
		    BITFMASK(MC13783_REGCTRL_VGEN_STBY);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VRFDIG:
		reg_mask = BITFMASK(MC13783_REGCTRL_VRFDIG_MODE) |
		    BITFMASK(MC13783_REGCTRL_VRFDIG_STBY);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VRFREF:
		reg_mask = BITFMASK(MC13783_REGCTRL_VRFREF_MODE) |
		    BITFMASK(MC13783_REGCTRL_VRFREF_STBY);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VRFCP:
		reg_mask = BITFMASK(MC13783_REGCTRL_VRFCP_MODE) |
		    BITFMASK(MC13783_REGCTRL_VRFCP_STBY);
		reg = REG_REGULATOR_MODE_0;
		break;
	case REGU_VSIM:
		reg_mask = BITFMASK(MC13783_REGCTRL_VSIM_MODE) |
		    BITFMASK(MC13783_REGCTRL_VSIM_STBY);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VESIM:
		reg_mask = BITFMASK(MC13783_REGCTRL_VESIM_MODE) |
		    BITFMASK(MC13783_REGCTRL_VESIM_STBY);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VCAM:
		reg_mask = BITFMASK(MC13783_REGCTRL_VCAM_MODE) |
		    BITFMASK(MC13783_REGCTRL_VCAM_STBY);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VRFBG:
		reg_mask = BITFMASK(MC13783_REGCTRL_VRFBG_STBY);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VRF1:
		reg_mask = BITFMASK(MC13783_REGCTRL_VRF1_MODE) |
		    BITFMASK(MC13783_REGCTRL_VRF1_STBY);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VRF2:
		reg_mask = BITFMASK(MC13783_REGCTRL_VRF2_MODE) |
		    BITFMASK(MC13783_REGCTRL_VRF2_STBY);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VMMC1:
		reg_mask = BITFMASK(MC13783_REGCTRL_VMMC1_MODE) |
		    BITFMASK(MC13783_REGCTRL_VMMC1_STBY);
		reg = REG_REGULATOR_MODE_1;
		break;
	case REGU_VMMC2:
		reg_mask = BITFMASK(MC13783_REGCTRL_VMMC2_MODE) |
		    BITFMASK(MC13783_REGCTRL_VMMC2_STBY);
		reg = REG_REGULATOR_MODE_1;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_read_reg(reg, &reg_val, reg_mask));

	switch (regulator) {
	case SW_SW3:
		l_mode = BITFEXT(reg_val, MC13783_SWCTRL_SW3_MODE);
		l_stby = BITFEXT(reg_val, MC13783_SWCTRL_SW3_STBY);
		break;
	case REGU_VAUDIO:
		l_mode = BITFEXT(reg_val, MC13783_REGCTRL_VAUDIO_MODE);
		l_stby = BITFEXT(reg_val, MC13783_REGCTRL_VAUDIO_STBY);
		break;
	case REGU_VIOHI:
		l_mode = BITFEXT(reg_val, MC13783_REGCTRL_VIOHI_MODE);
		l_stby = BITFEXT(reg_val, MC13783_REGCTRL_VIOHI_STBY);
		break;
	case REGU_VIOLO:
		l_mode = BITFEXT(reg_val, MC13783_REGCTRL_VIOLO_MODE);
		l_stby = BITFEXT(reg_val, MC13783_REGCTRL_VIOLO_STBY);
		break;
	case REGU_VDIG:
		l_mode = BITFEXT(reg_val, MC13783_REGCTRL_VDIG_MODE);
		l_stby = BITFEXT(reg_val, MC13783_REGCTRL_VDIG_STBY);
		break;
	case REGU_VGEN:
		l_mode = BITFEXT(reg_val, MC13783_REGCTRL_VGEN_MODE);
		l_stby = BITFEXT(reg_val, MC13783_REGCTRL_VGEN_STBY);
		break;
	case REGU_VRFDIG:
		l_mode = BITFEXT(reg_val, MC13783_REGCTRL_VRFDIG_MODE);
		l_stby = BITFEXT(reg_val, MC13783_REGCTRL_VRFDIG_STBY);
		break;
	case REGU_VRFREF:
		l_mode = BITFEXT(reg_val, MC13783_REGCTRL_VRFREF_MODE);
		l_stby = BITFEXT(reg_val, MC13783_REGCTRL_VRFREF_STBY);
		break;
	case REGU_VRFCP:
		l_mode = BITFEXT(reg_val, MC13783_REGCTRL_VRFCP_MODE);
		l_stby = BITFEXT(reg_val, MC13783_REGCTRL_VRFCP_STBY);
		break;
	case REGU_VSIM:
		l_mode = BITFEXT(reg_val, MC13783_REGCTRL_VSIM_MODE);
		l_stby = BITFEXT(reg_val, MC13783_REGCTRL_VSIM_STBY);
		break;
	case REGU_VESIM:
		l_mode = BITFEXT(reg_val, MC13783_REGCTRL_VESIM_MODE);
		l_stby = BITFEXT(reg_val, MC13783_REGCTRL_VESIM_STBY);
		break;
	case REGU_VCAM:
		l_mode = BITFEXT(reg_val, MC13783_REGCTRL_VCAM_MODE);
		l_stby = BITFEXT(reg_val, MC13783_REGCTRL_VCAM_STBY);
		break;
	case REGU_VRFBG:
		l_mode = MC13783_REGTRL_LP_MODE_DISABLE;
		l_stby = BITFEXT(reg_val, MC13783_REGCTRL_VRFBG_STBY);
		break;
	case REGU_VRF1:
		l_mode = BITFEXT(reg_val, MC13783_REGCTRL_VRF1_MODE);
		l_stby = BITFEXT(reg_val, MC13783_REGCTRL_VRF1_STBY);
		break;
	case REGU_VRF2:
		l_mode = BITFEXT(reg_val, MC13783_REGCTRL_VRF2_MODE);
		l_stby = BITFEXT(reg_val, MC13783_REGCTRL_VRF2_STBY);
		break;
	case REGU_VMMC1:
		l_mode = BITFEXT(reg_val, MC13783_REGCTRL_VMMC1_MODE);
		l_stby = BITFEXT(reg_val, MC13783_REGCTRL_VMMC1_STBY);
		break;
	case REGU_VMMC2:
		l_mode = BITFEXT(reg_val, MC13783_REGCTRL_VMMC2_MODE);
		l_stby = BITFEXT(reg_val, MC13783_REGCTRL_VMMC2_STBY);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	if ((l_mode == MC13783_REGTRL_LP_MODE_DISABLE) &&
	    (l_stby == MC13783_REGTRL_STBY_MODE_DISABLE)) {
		*lp_mode = LOW_POWER_DISABLED;
	} else if ((l_mode == MC13783_REGTRL_LP_MODE_DISABLE) &&
		   (l_stby == MC13783_REGTRL_STBY_MODE_ENABLE)) {
		*lp_mode = LOW_POWER_CTRL_BY_PIN;
	} else if ((l_mode == MC13783_REGTRL_LP_MODE_ENABLE) &&
		   (l_stby == MC13783_REGTRL_STBY_MODE_DISABLE)) {
		*lp_mode = LOW_POWER_EN;
	} else {
		*lp_mode = LOW_POWER_AND_LOW_POWER_CTRL_BY_PIN;
	}

	return PMIC_SUCCESS;
}

/*!
 * This function sets the regulator configuration.
 *
 * @param        regulator    The regulator to be configured.
 * @param        config       The regulator output configuration.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_regulator_set_config(t_pmic_regulator regulator,
					    t_regulator_config *config)
{
	if (config == NULL) {
		return PMIC_ERROR;
	}

	switch (regulator) {
	case SW_SW1A:
	case SW_SW1B:
	case SW_SW2A:
	case SW_SW2B:
		CHECK_ERROR(pmic_power_regulator_set_voltage
			    (regulator, config->voltage));
		CHECK_ERROR(pmic_power_switcher_set_dvs
			    (regulator, config->voltage_lvs));
		CHECK_ERROR(pmic_power_switcher_set_stby
			    (regulator, config->voltage_stby));
		CHECK_ERROR(pmic_power_switcher_set_mode
			    (regulator, config->mode, false));
		CHECK_ERROR(pmic_power_switcher_set_mode
			    (regulator, config->stby_mode, true));
		CHECK_ERROR(pmic_power_switcher_set_dvs_speed
			    (regulator, config->dvs_speed));
		CHECK_ERROR(pmic_power_switcher_set_panic_mode
			    (regulator, config->panic_mode));
		CHECK_ERROR(pmic_power_switcher_set_softstart
			    (regulator, config->softstart));
		break;
	case SW_PLL:
		CHECK_ERROR(pmic_power_switcher_set_factor
			    (regulator, config->factor));
		break;
	case SW_SW3:
	case REGU_VIOLO:
	case REGU_VDIG:
	case REGU_VGEN:
	case REGU_VRFDIG:
	case REGU_VRFREF:
	case REGU_VRFCP:
	case REGU_VSIM:
	case REGU_VESIM:
	case REGU_VCAM:
	case REGU_VRF1:
	case REGU_VRF2:
	case REGU_VMMC1:
	case REGU_VMMC2:
		CHECK_ERROR(pmic_power_regulator_set_voltage
			    (regulator, config->voltage));
		CHECK_ERROR(pmic_power_regulator_set_lp_mode
			    (regulator, config->lp_mode));
		break;
	case REGU_VVIB:
		CHECK_ERROR(pmic_power_regulator_set_voltage
			    (regulator, config->voltage));
		break;
	case REGU_VAUDIO:
	case REGU_VIOHI:
	case REGU_VRFBG:
		CHECK_ERROR(pmic_power_regulator_set_lp_mode
			    (regulator, config->lp_mode));
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	return PMIC_SUCCESS;
}

/*!
 * This function retrives the regulator output configuration.
 *
 * @param        regulator    The regulator to be truned off.
 * @param        config       Pointer to regulator configuration.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_regulator_get_config(t_pmic_regulator regulator,
					    t_regulator_config *config)
{
	if (config == NULL) {
		return PMIC_ERROR;
	}

	switch (regulator) {
	case SW_SW1A:
	case SW_SW1B:
	case SW_SW2A:
	case SW_SW2B:
		CHECK_ERROR(pmic_power_regulator_get_voltage
			    (regulator, &config->voltage));
		CHECK_ERROR(pmic_power_switcher_get_dvs
			    (regulator, &config->voltage_lvs));
		CHECK_ERROR(pmic_power_switcher_get_stby
			    (regulator, &config->voltage_stby));
		CHECK_ERROR(pmic_power_switcher_get_mode
			    (regulator, &config->mode, false));
		CHECK_ERROR(pmic_power_switcher_get_mode
			    (regulator, &config->stby_mode, true));
		CHECK_ERROR(pmic_power_switcher_get_dvs_speed
			    (regulator, &config->dvs_speed));
		CHECK_ERROR(pmic_power_switcher_get_panic_mode
			    (regulator, &config->panic_mode));
		CHECK_ERROR(pmic_power_switcher_get_softstart
			    (regulator, &config->softstart));
		break;
	case SW_PLL:
		CHECK_ERROR(pmic_power_switcher_get_factor
			    (regulator, &config->factor));
		break;
	case SW_SW3:
	case REGU_VIOLO:
	case REGU_VDIG:
	case REGU_VGEN:
	case REGU_VRFDIG:
	case REGU_VRFREF:
	case REGU_VRFCP:
	case REGU_VSIM:
	case REGU_VESIM:
	case REGU_VCAM:
	case REGU_VRF1:
	case REGU_VRF2:
	case REGU_VMMC1:
	case REGU_VMMC2:
		CHECK_ERROR(pmic_power_regulator_get_voltage
			    (regulator, &config->voltage));
		CHECK_ERROR(pmic_power_regulator_get_lp_mode
			    (regulator, &config->lp_mode));
		break;
	case REGU_VVIB:
		CHECK_ERROR(pmic_power_regulator_get_voltage
			    (regulator, &config->voltage));
		break;
	case REGU_VAUDIO:
	case REGU_VIOHI:
	case REGU_VRFBG:
		CHECK_ERROR(pmic_power_regulator_get_lp_mode
			    (regulator, &config->lp_mode));
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	return PMIC_SUCCESS;
}

/*!
 * This function enables automatically VBKUP2 in the memory hold modes.
 * Only on mc13783 2.0 or higher
 *
 * @param        en           if true, enable VBKUP2AUTOMH
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_vbkup2_auto_en(bool en)
{
	unsigned int reg_val = 0, reg_mask = 0;
	pmic_version_t mc13783_ver;
	mc13783_ver = pmic_get_version();
	if (mc13783_ver.revision >= 20) {
		reg_val = BITFVAL(MC13783_REGCTRL_VBKUP2AUTOMH, en);
		reg_mask = BITFMASK(MC13783_REGCTRL_VBKUP2AUTOMH);

		CHECK_ERROR(pmic_write_reg(REG_POWER_CONTROL_0,
					   reg_val, reg_mask));
		return PMIC_SUCCESS;
	} else {
		return PMIC_NOT_SUPPORTED;
	}
}

/*!
 * This function gets state of automatically VBKUP2.
 * Only on mc13783 2.0 or higher
 *
 * @param        en           if true, VBKUP2AUTOMH is enabled
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_get_vbkup2_auto_state(bool *en)
{
	unsigned int reg_val = 0, reg_mask = 0;
	pmic_version_t mc13783_ver;
	mc13783_ver = pmic_get_version();
	if (mc13783_ver.revision >= 20) {
		reg_mask = BITFMASK(MC13783_REGCTRL_VBKUP2AUTOMH);
		CHECK_ERROR(pmic_read_reg(REG_POWER_CONTROL_0,
					  &reg_val, reg_mask));
		*en = BITFEXT(reg_val, MC13783_REGCTRL_VBKUP2AUTOMH);

		return PMIC_SUCCESS;
	} else {
		return PMIC_NOT_SUPPORTED;
	}
}

/*!
 * This function enables battery detect function.
 * Only on mc13783 2.0 or higher
 *
 * @param        en           if true, enable BATTDETEN
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_bat_det_en(bool en)
{
	unsigned int reg_val = 0, reg_mask = 0;
	pmic_version_t mc13783_ver;
	mc13783_ver = pmic_get_version();
	if (mc13783_ver.revision >= 20) {
		reg_val = BITFVAL(MC13783_REGCTRL_BATTDETEN, en);
		reg_mask = BITFMASK(MC13783_REGCTRL_BATTDETEN);

		CHECK_ERROR(pmic_write_reg(REG_POWER_CONTROL_0,
					   reg_val, reg_mask));
		return PMIC_SUCCESS;
	} else {
		return PMIC_NOT_SUPPORTED;
	}
}

/*!
 * This function gets state of battery detect function.
 * Only on mc13783 2.0 or higher
 *
 * @param        en           if true, BATTDETEN is enabled
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_get_bat_det_state(bool *en)
{
	unsigned int reg_val = 0, reg_mask = 0;
	pmic_version_t mc13783_ver;
	mc13783_ver = pmic_get_version();
	if (mc13783_ver.revision >= 20) {
		reg_mask = BITFMASK(MC13783_REGCTRL_BATTDETEN);

		CHECK_ERROR(pmic_read_reg(REG_POWER_CONTROL_0,
					  &reg_val, reg_mask));
		*en = BITFEXT(reg_val, MC13783_REGCTRL_BATTDETEN);
		return PMIC_SUCCESS;
	} else {
		return PMIC_NOT_SUPPORTED;
	}
}

/*!
 * This function enables control of VVIB by VIBEN pin.
 * Only on mc13783 2.0 or higher
 *
 * @param        en           if true, enable VIBPINCTRL
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_vib_pin_en(bool en)
{
	unsigned int reg_val = 0, reg_mask = 0;
	pmic_version_t mc13783_ver;
	mc13783_ver = pmic_get_version();
	if (mc13783_ver.revision >= 20) {
		reg_val = BITFVAL(MC13783_REGCTRL_VIBPINCTRL, en);
		reg_mask = BITFMASK(MC13783_REGCTRL_VIBPINCTRL);

		CHECK_ERROR(pmic_write_reg(REG_POWER_MISCELLANEOUS,
					   reg_val, reg_mask));
		return PMIC_SUCCESS;
	} else {
		return PMIC_NOT_SUPPORTED;
	}
}

/*!
 * This function gets state of control of VVIB by VIBEN pin.
 * Only on mc13783 2.0 or higher
 * @param        en           if true, VIBPINCTRL is enabled
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_gets_vib_pin_state(bool *en)
{
	unsigned int reg_val = 0, reg_mask = 0;
	pmic_version_t mc13783_ver;
	mc13783_ver = pmic_get_version();
	if (mc13783_ver.revision >= 20) {
		reg_mask = BITFMASK(MC13783_REGCTRL_VIBPINCTRL);
		CHECK_ERROR(pmic_read_reg(REG_POWER_MISCELLANEOUS,
					  &reg_val, reg_mask));
		*en = BITFEXT(reg_val, MC13783_REGCTRL_VIBPINCTRL);
		return PMIC_SUCCESS;
	} else {
		return PMIC_NOT_SUPPORTED;
	}
}

/*!
 * This function returns power up sense value
 *
 * @param        p_up_sense     value of power up sense
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_power_get_power_mode_sense(struct t_p_up_sense *p_up_sense)
{
	unsigned int reg_value = 0;
	CHECK_ERROR(pmic_read_reg(REG_POWER_UP_MODE_SENSE,
				  &reg_value, PMIC_ALL_BITS));
	p_up_sense->state_ictest = (STATE_ICTEST_MASK & reg_value);
	p_up_sense->state_clksel = ((STATE_CLKSEL_MASK & reg_value)
				    >> STATE_CLKSEL_BIT);
	p_up_sense->state_pums1 = ((STATE_PUMS1_MASK & reg_value)
				   >> STATE_PUMS1_BITS);
	p_up_sense->state_pums2 = ((STATE_PUMS2_MASK & reg_value)
				   >> STATE_PUMS2_BITS);
	p_up_sense->state_pums3 = ((STATE_PUMS3_MASK & reg_value)
				   >> STATE_PUMS3_BITS);
	p_up_sense->state_chrgmode0 = ((STATE_CHRGM1_MASK & reg_value)
				       >> STATE_CHRGM1_BITS);
	p_up_sense->state_chrgmode1 = ((STATE_CHRGM2_MASK & reg_value)
				       >> STATE_CHRGM2_BITS);
	p_up_sense->state_umod = ((STATE_UMOD_MASK & reg_value)
				  >> STATE_UMOD_BITS);
	p_up_sense->state_usben = ((STATE_USBEN_MASK & reg_value)
				   >> STATE_USBEN_BIT);
	p_up_sense->state_sw_1a1b_joined = ((STATE_SW1A_J_B_MASK & reg_value)
					    >> STATE_SW1A_J_B_BIT);
	p_up_sense->state_sw_2a2b_joined = ((STATE_SW2A_J_B_MASK & reg_value)
					    >> STATE_SW2A_J_B_BIT);
	return PMIC_SUCCESS;
}

/*!
 * This function configures the Regen assignment for all regulator
 *
 * @param        regulator      type of regulator
 * @param        en_dis         if true, the regulator is enabled by regen.
 *
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS pmic_power_set_regen_assig(t_pmic_regulator regulator, bool en_dis)
{
	unsigned int reg_val = 0, reg_mask = 0;

	switch (regulator) {
	case REGU_VAUDIO:
		reg_val = BITFVAL(MC13783_REGGEN_VAUDIO, en_dis);
		reg_mask = BITFMASK(MC13783_REGGEN_VAUDIO);
		break;
	case REGU_VIOHI:
		reg_val = BITFVAL(MC13783_REGGEN_VIOHI, en_dis);
		reg_mask = BITFMASK(MC13783_REGGEN_VIOHI);
		break;
	case REGU_VIOLO:
		reg_val = BITFVAL(MC13783_REGGEN_VIOLO, en_dis);
		reg_mask = BITFMASK(MC13783_REGGEN_VIOLO);
		break;
	case REGU_VDIG:
		reg_val = BITFVAL(MC13783_REGGEN_VDIG, en_dis);
		reg_mask = BITFMASK(MC13783_REGGEN_VDIG);
		break;
	case REGU_VGEN:
		reg_val = BITFVAL(MC13783_REGGEN_VGEN, en_dis);
		reg_mask = BITFMASK(MC13783_REGGEN_VGEN);
		break;
	case REGU_VRFDIG:
		reg_val = BITFVAL(MC13783_REGGEN_VRFDIG, en_dis);
		reg_mask = BITFMASK(MC13783_REGGEN_VRFDIG);
		break;
	case REGU_VRFREF:
		reg_val = BITFVAL(MC13783_REGGEN_VRFREF, en_dis);
		reg_mask = BITFMASK(MC13783_REGGEN_VRFREF);
		break;
	case REGU_VRFCP:
		reg_val = BITFVAL(MC13783_REGGEN_VRFCP, en_dis);
		reg_mask = BITFMASK(MC13783_REGGEN_VRFCP);
		break;
	case REGU_VCAM:
		reg_val = BITFVAL(MC13783_REGGEN_VCAM, en_dis);
		reg_mask = BITFMASK(MC13783_REGGEN_VCAM);
		break;
	case REGU_VRFBG:
		reg_val = BITFVAL(MC13783_REGGEN_VRFBG, en_dis);
		reg_mask = BITFMASK(MC13783_REGGEN_VRFBG);
		break;
	case REGU_VRF1:
		reg_val = BITFVAL(MC13783_REGGEN_VRF1, en_dis);
		reg_mask = BITFMASK(MC13783_REGGEN_VRF1);
		break;
	case REGU_VRF2:
		reg_val = BITFVAL(MC13783_REGGEN_VRF2, en_dis);
		reg_mask = BITFMASK(MC13783_REGGEN_VRF2);
		break;
	case REGU_VMMC1:
		reg_val = BITFVAL(MC13783_REGGEN_VMMC1, en_dis);
		reg_mask = BITFMASK(MC13783_REGGEN_VMMC1);
		break;
	case REGU_VMMC2:
		reg_val = BITFVAL(MC13783_REGGEN_VMMC2, en_dis);
		reg_mask = BITFMASK(MC13783_REGGEN_VMMC2);
		break;
	case REGU_GPO1:
		reg_val = BITFVAL(MC13783_REGGEN_GPO1, en_dis);
		reg_mask = BITFMASK(MC13783_REGGEN_GPO1);
		break;
	case REGU_GPO2:
		reg_val = BITFVAL(MC13783_REGGEN_GPO2, en_dis);
		reg_mask = BITFMASK(MC13783_REGGEN_GPO2);
		break;
	case REGU_GPO3:
		reg_val = BITFVAL(MC13783_REGGEN_GPO3, en_dis);
		reg_mask = BITFMASK(MC13783_REGGEN_GPO3);
		break;
	case REGU_GPO4:
		reg_val = BITFVAL(MC13783_REGGEN_GPO4, en_dis);
		reg_mask = BITFMASK(MC13783_REGGEN_GPO4);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(REG_REGEN_ASSIGNMENT, reg_val, reg_mask));

	return PMIC_SUCCESS;
}

/*!
 * This function gets the Regen assignment for all regulator
 *
 * @param        regulator      type of regulator
 * @param        en_dis         return value, if true :
 *                              the regulator is enabled by regen.
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS pmic_power_get_regen_assig(t_pmic_regulator regulator,
				       bool *en_dis)
{
	unsigned int reg_val = 0, reg_mask = 0;

	switch (regulator) {
	case REGU_VAUDIO:
		reg_mask = BITFMASK(MC13783_REGGEN_VAUDIO);
		break;
	case REGU_VIOHI:
		reg_mask = BITFMASK(MC13783_REGGEN_VIOHI);
		break;
	case REGU_VIOLO:
		reg_mask = BITFMASK(MC13783_REGGEN_VIOLO);
		break;
	case REGU_VDIG:
		reg_mask = BITFMASK(MC13783_REGGEN_VDIG);
		break;
	case REGU_VGEN:
		reg_mask = BITFMASK(MC13783_REGGEN_VGEN);
		break;
	case REGU_VRFDIG:
		reg_mask = BITFMASK(MC13783_REGGEN_VRFDIG);
		break;
	case REGU_VRFREF:
		reg_mask = BITFMASK(MC13783_REGGEN_VRFREF);
		break;
	case REGU_VRFCP:
		reg_mask = BITFMASK(MC13783_REGGEN_VRFCP);
		break;
	case REGU_VCAM:
		reg_mask = BITFMASK(MC13783_REGGEN_VCAM);
		break;
	case REGU_VRFBG:
		reg_mask = BITFMASK(MC13783_REGGEN_VRFBG);
		break;
	case REGU_VRF1:
		reg_mask = BITFMASK(MC13783_REGGEN_VRF1);
		break;
	case REGU_VRF2:
		reg_mask = BITFMASK(MC13783_REGGEN_VRF2);
		break;
	case REGU_VMMC1:
		reg_mask = BITFMASK(MC13783_REGGEN_VMMC1);
		break;
	case REGU_VMMC2:
		reg_mask = BITFMASK(MC13783_REGGEN_VMMC2);
		break;
	case REGU_GPO1:
		reg_mask = BITFMASK(MC13783_REGGEN_GPO1);
		break;
	case REGU_GPO2:
		reg_mask = BITFMASK(MC13783_REGGEN_GPO2);
		break;
	case REGU_GPO3:
		reg_mask = BITFMASK(MC13783_REGGEN_GPO3);
		break;
	case REGU_GPO4:
		reg_mask = BITFMASK(MC13783_REGGEN_GPO4);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_read_reg(REG_REGEN_ASSIGNMENT, &reg_val, reg_mask));

	switch (regulator) {
	case REGU_VAUDIO:
		*en_dis = BITFEXT(reg_val, MC13783_REGGEN_VAUDIO);
		break;
	case REGU_VIOHI:
		*en_dis = BITFEXT(reg_val, MC13783_REGGEN_VIOHI);
		break;
	case REGU_VIOLO:
		*en_dis = BITFEXT(reg_val, MC13783_REGGEN_VIOLO);
		break;
	case REGU_VDIG:
		*en_dis = BITFEXT(reg_val, MC13783_REGGEN_VDIG);
		break;
	case REGU_VGEN:
		*en_dis = BITFEXT(reg_val, MC13783_REGGEN_VGEN);
		break;
	case REGU_VRFDIG:
		*en_dis = BITFEXT(reg_val, MC13783_REGGEN_VRFDIG);
		break;
	case REGU_VRFREF:
		*en_dis = BITFEXT(reg_val, MC13783_REGGEN_VRFREF);
		break;
	case REGU_VRFCP:
		*en_dis = BITFEXT(reg_val, MC13783_REGGEN_VRFCP);
		break;
	case REGU_VCAM:
		*en_dis = BITFEXT(reg_val, MC13783_REGGEN_VCAM);
		break;
	case REGU_VRFBG:
		*en_dis = BITFEXT(reg_val, MC13783_REGGEN_VRFBG);
		break;
	case REGU_VRF1:
		*en_dis = BITFEXT(reg_val, MC13783_REGGEN_VRF1);
		break;
	case REGU_VRF2:
		*en_dis = BITFEXT(reg_val, MC13783_REGGEN_VRF2);
		break;
	case REGU_VMMC1:
		*en_dis = BITFEXT(reg_val, MC13783_REGGEN_VMMC1);
		break;
	case REGU_VMMC2:
		*en_dis = BITFEXT(reg_val, MC13783_REGGEN_VMMC2);
		break;
	case REGU_GPO1:
		*en_dis = BITFEXT(reg_val, MC13783_REGGEN_GPO1);
		break;
	case REGU_GPO2:
		*en_dis = BITFEXT(reg_val, MC13783_REGGEN_GPO2);
		break;
	case REGU_GPO3:
		*en_dis = BITFEXT(reg_val, MC13783_REGGEN_GPO3);
		break;
	case REGU_GPO4:
		*en_dis = BITFEXT(reg_val, MC13783_REGGEN_GPO4);
		break;
	default:
		break;
	}

	return PMIC_SUCCESS;
}

/*!
 * This function sets the Regen polarity.
 *
 * @param        en_dis         If true regen is inverted.
 *
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS pmic_power_set_regen_inv(bool en_dis)
{
	unsigned int reg_val = 0, reg_mask = 0;

	reg_val = BITFVAL(MC13783_REGGEN_INV, en_dis);
	reg_mask = BITFMASK(MC13783_REGGEN_INV);

	CHECK_ERROR(pmic_write_reg(REG_REGEN_ASSIGNMENT, reg_val, reg_mask));
	return PMIC_SUCCESS;
}

/*!
 * This function gets the Regen polarity.
 *
 * @param        en_dis         If true regen is inverted.
 *
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS pmic_power_get_regen_inv(bool *en_dis)
{
	unsigned int reg_val = 0, reg_mask = 0;

	reg_mask = BITFMASK(MC13783_REGGEN_INV);
	CHECK_ERROR(pmic_read_reg(REG_REGEN_ASSIGNMENT, &reg_val, reg_mask));
	*en_dis = BITFEXT(reg_val, MC13783_REGGEN_INV);
	return PMIC_SUCCESS;
}

/*!
 * This function enables esim control voltage.
 * Only on mc13783 2.0 or higher
 *
 * @param        vesim          if true, enable VESIMESIMEN
 * @param        vmmc1          if true, enable VMMC1ESIMEN
 * @param        vmmc2          if true, enable VMMC2ESIMEN
 *
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS pmic_power_esim_v_en(bool vesim, bool vmmc1, bool vmmc2)
{
	unsigned int reg_val = 0, reg_mask = 0;
	pmic_version_t mc13783_ver;
	mc13783_ver = pmic_get_version();
	if (mc13783_ver.revision >= 20) {
		reg_val = BITFVAL(MC13783_REGGEN_VESIMESIM, vesim) |
		    BITFVAL(MC13783_REGGEN_VMMC1ESIM, vesim) |
		    BITFVAL(MC13783_REGGEN_VMMC2ESIM, vesim);
		reg_mask = BITFMASK(MC13783_REGGEN_VESIMESIM) |
		    BITFMASK(MC13783_REGGEN_VMMC1ESIM) |
		    BITFMASK(MC13783_REGGEN_VMMC2ESIM);
		CHECK_ERROR(pmic_write_reg(REG_REGEN_ASSIGNMENT,
					   reg_val, reg_mask));
		return PMIC_SUCCESS;
	} else {
		return PMIC_NOT_SUPPORTED;
	}
}

/*!
 * This function gets esim control voltage values.
 * Only on mc13783 2.0 or higher
 *
 * @param        vesim          if true, enable VESIMESIMEN
 * @param        vmmc1          if true, enable VMMC1ESIMEN
 * @param        vmmc2          if true, enable VMMC2ESIMEN
 *
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS pmic_power_gets_esim_v_state(bool *vesim, bool *vmmc1,
					 bool *vmmc2)
{
	unsigned int reg_val = 0, reg_mask = 0;
	pmic_version_t mc13783_ver;
	mc13783_ver = pmic_get_version();
	if (mc13783_ver.revision >= 20) {
		reg_mask = BITFMASK(MC13783_REGGEN_VESIMESIM) |
		    BITFMASK(MC13783_REGGEN_VMMC1ESIM) |
		    BITFMASK(MC13783_REGGEN_VMMC2ESIM);
		CHECK_ERROR(pmic_read_reg(REG_REGEN_ASSIGNMENT,
					  &reg_val, reg_mask));
		*vesim = BITFEXT(reg_val, MC13783_REGGEN_VESIMESIM);
		*vmmc1 = BITFEXT(reg_val, MC13783_REGGEN_VMMC1ESIM);
		*vmmc2 = BITFEXT(reg_val, MC13783_REGGEN_VMMC2ESIM);
		return PMIC_SUCCESS;
	} else {
		return PMIC_NOT_SUPPORTED;
	}
}

/*!
 * This function enables auto reset after a system reset.
 *
 * @param        en         if true, the auto reset is enabled
 *
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS pmic_power_set_auto_reset_en(bool en)
{
	unsigned int reg_val = 0, reg_mask = 0;

	reg_val = BITFVAL(MC13783_AUTO_RESTART, en);
	reg_mask = BITFMASK(MC13783_AUTO_RESTART);

	CHECK_ERROR(pmic_write_reg(REG_POWER_CONTROL_2, reg_val, reg_mask));
	return PMIC_SUCCESS;
}

/*!
 * This function gets auto reset configuration.
 *
 * @param        en         if true, the auto reset is enabled
 *
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS pmic_power_get_auto_reset_en(bool *en)
{
	unsigned int reg_val = 0, reg_mask = 0;

	reg_mask = BITFMASK(MC13783_AUTO_RESTART);
	CHECK_ERROR(pmic_read_reg(REG_POWER_CONTROL_2, &reg_val, reg_mask));
	*en = BITFEXT(reg_val, MC13783_AUTO_RESTART);
	return PMIC_SUCCESS;
}

/*!
 * This function configures a system reset on a button.
 *
 * @param       bt         type of button.
 * @param       sys_rst    if true, enable the system reset on this button
 * @param       deb_time   sets the debounce time on this button pin
 *
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS pmic_power_set_conf_button(t_button bt, bool sys_rst, int deb_time)
{
	int max_val = 0;
	unsigned int reg_val = 0, reg_mask = 0;

	max_val = (1 << MC13783_DEB_BT_ON1B_WID) - 1;
	if (deb_time > max_val) {
		return PMIC_PARAMETER_ERROR;
	}

	switch (bt) {
	case BT_ON1B:
		reg_val = BITFVAL(MC13783_EN_BT_ON1B, sys_rst) |
		    BITFVAL(MC13783_DEB_BT_ON1B, deb_time);
		reg_mask = BITFMASK(MC13783_EN_BT_ON1B) |
		    BITFMASK(MC13783_DEB_BT_ON1B);
		break;
	case BT_ON2B:
		reg_val = BITFVAL(MC13783_EN_BT_ON2B, sys_rst) |
		    BITFVAL(MC13783_DEB_BT_ON2B, deb_time);
		reg_mask = BITFMASK(MC13783_EN_BT_ON2B) |
		    BITFMASK(MC13783_DEB_BT_ON2B);
		break;
	case BT_ON3B:
		reg_val = BITFVAL(MC13783_EN_BT_ON3B, sys_rst) |
		    BITFVAL(MC13783_DEB_BT_ON3B, deb_time);
		reg_mask = BITFMASK(MC13783_EN_BT_ON3B) |
		    BITFMASK(MC13783_DEB_BT_ON3B);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(REG_POWER_CONTROL_2, reg_val, reg_mask));

	return PMIC_SUCCESS;
}

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
				       bool *sys_rst, int *deb_time)
{
	unsigned int reg_val = 0, reg_mask = 0;

	switch (bt) {
	case BT_ON1B:
		reg_mask = BITFMASK(MC13783_EN_BT_ON1B) |
		    BITFMASK(MC13783_DEB_BT_ON1B);
		break;
	case BT_ON2B:
		reg_mask = BITFMASK(MC13783_EN_BT_ON2B) |
		    BITFMASK(MC13783_DEB_BT_ON2B);
		break;
	case BT_ON3B:
		reg_mask = BITFMASK(MC13783_EN_BT_ON3B) |
		    BITFMASK(MC13783_DEB_BT_ON3B);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_read_reg(REG_POWER_CONTROL_2, &reg_val, reg_mask));

	switch (bt) {
	case BT_ON1B:
		*sys_rst = BITFEXT(reg_val, MC13783_EN_BT_ON1B);
		*deb_time = BITFEXT(reg_val, MC13783_DEB_BT_ON1B);
		break;
	case BT_ON2B:
		*sys_rst = BITFEXT(reg_val, MC13783_EN_BT_ON2B);
		*deb_time = BITFEXT(reg_val, MC13783_DEB_BT_ON2B);
		break;
	case BT_ON3B:
		*sys_rst = BITFEXT(reg_val, MC13783_EN_BT_ON3B);
		*deb_time = BITFEXT(reg_val, MC13783_DEB_BT_ON3B);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}
	return PMIC_SUCCESS;
}

/*!
 * This function is used to un/subscribe on power event IT.
 *
 * @param        event          type of event.
 * @param        callback       event callback function.
 * @param        sub            define if Un/subscribe event.
 *
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS pmic_power_event(t_pwr_int event, void *callback, bool sub)
{
	pmic_event_callback_t power_callback;
	type_event power_event;

	power_callback.func = callback;
	power_callback.param = NULL;
	switch (event) {
	case PWR_IT_BPONI:
		power_event = EVENT_BPONI;
		break;
	case PWR_IT_LOBATLI:
		power_event = EVENT_LOBATLI;
		break;
	case PWR_IT_LOBATHI:
		power_event = EVENT_LOBATHI;
		break;
	case PWR_IT_ONOFD1I:
		power_event = EVENT_ONOFD1I;
		break;
	case PWR_IT_ONOFD2I:
		power_event = EVENT_ONOFD2I;
		break;
	case PWR_IT_ONOFD3I:
		power_event = EVENT_ONOFD3I;
		break;
	case PWR_IT_SYSRSTI:
		power_event = EVENT_SYSRSTI;
		break;
	case PWR_IT_PWRRDYI:
		power_event = EVENT_PWRRDYI;
		break;
	case PWR_IT_PCI:
		power_event = EVENT_PCI;
		break;
	case PWR_IT_WARMI:
		power_event = EVENT_WARMI;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}
	if (sub == true) {
		CHECK_ERROR(pmic_event_subscribe(power_event, power_callback));
	} else {
		CHECK_ERROR(pmic_event_unsubscribe
			    (power_event, power_callback));
	}
	return PMIC_SUCCESS;
}

/*!
 * This function is used to subscribe on power event IT.
 *
 * @param        event          type of event.
 * @param        callback       event callback function.
 *
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS pmic_power_event_sub(t_pwr_int event, void *callback)
{
	return pmic_power_event(event, callback, true);
}

/*!
 * This function is used to un subscribe on power event IT.
 *
 * @param        event          type of event.
 * @param        callback       event callback function.
 *
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS pmic_power_event_unsub(t_pwr_int event, void *callback)
{
	return pmic_power_event(event, callback, false);
}

void pmic_power_key_callback(void)
{
#ifdef CONFIG_MXC_HWEVENT
	/*read the power key is pressed or up */
	t_sensor_bits sense;
	struct mxc_hw_event event = { HWE_POWER_KEY, 0 };

	pmic_get_sensors(&sense);
	if (sense.sense_onofd1s) {
		pr_debug("PMIC Power key up\n");
		event.args = PWRK_UNPRESS;
	} else {
		pr_debug("PMIC Power key pressed\n");
		event.args = PWRK_PRESS;
	}
	/* send hw event */
	hw_event_send(HWE_DEF_PRIORITY, &event);
#endif
}

static irqreturn_t power_key_int(int irq, void *dev_id)
{
	pr_info(KERN_INFO "on-off key pressed\n");

	return 0;
}

extern void gpio_power_key_active(void);

/*
 * Init and Exit
 */

static int pmic_power_probe(struct platform_device *pdev)
{
	int irq, ret;
	struct pmic_platform_data *ppd;

	/* configure on/off button */
	gpio_power_key_active();

	ppd = pdev->dev.platform_data;
	if (ppd)
		irq = ppd->power_key_irq;
	else
		goto done;

	if (irq == 0) {
		pr_info(KERN_INFO "PMIC Power has no platform data\n");
		goto done;
	}
	set_irq_type(irq, IRQF_TRIGGER_RISING);

	ret = request_irq(irq, power_key_int, 0, "power_key", 0);
	if (ret)
		pr_info(KERN_ERR "register on-off key interrupt failed\n");

	set_irq_wake(irq, 1);

      done:
	pr_info(KERN_INFO "PMIC Power successfully probed\n");
	return 0;
}

static struct platform_driver pmic_power_driver_ldm = {
	.driver = {
		   .name = "pmic_power",
		   },
	.suspend = pmic_power_suspend,
	.resume = pmic_power_resume,
	.probe = pmic_power_probe,
	.remove = NULL,
};

static int __init pmic_power_init(void)
{
	pr_debug("PMIC Power driver loading..\n");
	pmic_power_event_sub(PWR_IT_ONOFD1I, pmic_power_key_callback);
	/* set power off hook to mc13783 power off */
	pm_power_off = pmic_power_off;
	return platform_driver_register(&pmic_power_driver_ldm);
}
static void __exit pmic_power_exit(void)
{
	pmic_power_event_unsub(PWR_IT_ONOFD1I, pmic_power_key_callback);
	platform_driver_unregister(&pmic_power_driver_ldm);
	pr_debug("PMIC Power driver successfully unloaded\n");
}

/*
 * Module entry points
 */

subsys_initcall_sync(pmic_power_init);
module_exit(pmic_power_exit);

MODULE_DESCRIPTION("pmic_power driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
