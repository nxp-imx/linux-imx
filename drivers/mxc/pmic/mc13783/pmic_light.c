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
 * @file mc13783/pmic_light.c
 * @brief This is the main file of PMIC(mc13783) Light and Backlight driver.
 *
 * @ingroup PMIC_LIGHT
 */

/*
 * Includes
 */
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/pmic_light.h>
#include <linux/pmic_status.h>
#include "pmic_light_defs.h"

#define NB_LIGHT_REG      6

static int pmic_light_major;

/*!
 * Number of users waiting in suspendq
 */
static int swait;

/*!
 * To indicate whether any of the light devices are suspending
 */
static int suspend_flag;

/*!
 * The suspendq is used to block application calls
 */
static wait_queue_head_t suspendq;

static struct class *pmic_light_class;

/* EXPORTED FUNCTIONS */
EXPORT_SYMBOL(pmic_bklit_tcled_master_enable);
EXPORT_SYMBOL(pmic_bklit_tcled_master_disable);
EXPORT_SYMBOL(pmic_bklit_master_enable);
EXPORT_SYMBOL(pmic_bklit_master_disable);
EXPORT_SYMBOL(pmic_bklit_set_current);
EXPORT_SYMBOL(pmic_bklit_get_current);
EXPORT_SYMBOL(pmic_bklit_set_dutycycle);
EXPORT_SYMBOL(pmic_bklit_get_dutycycle);
EXPORT_SYMBOL(pmic_bklit_set_cycle_time);
EXPORT_SYMBOL(pmic_bklit_get_cycle_time);
EXPORT_SYMBOL(pmic_bklit_set_mode);
EXPORT_SYMBOL(pmic_bklit_get_mode);
EXPORT_SYMBOL(pmic_bklit_rampup);
EXPORT_SYMBOL(pmic_bklit_off_rampup);
EXPORT_SYMBOL(pmic_bklit_rampdown);
EXPORT_SYMBOL(pmic_bklit_off_rampdown);
EXPORT_SYMBOL(pmic_bklit_enable_edge_slow);
EXPORT_SYMBOL(pmic_bklit_disable_edge_slow);
EXPORT_SYMBOL(pmic_bklit_get_edge_slow);
EXPORT_SYMBOL(pmic_bklit_set_strobemode);
EXPORT_SYMBOL(pmic_tcled_enable);
EXPORT_SYMBOL(pmic_tcled_disable);
EXPORT_SYMBOL(pmic_tcled_get_mode);
EXPORT_SYMBOL(pmic_tcled_ind_set_current);
EXPORT_SYMBOL(pmic_tcled_ind_get_current);
EXPORT_SYMBOL(pmic_tcled_ind_set_blink_pattern);
EXPORT_SYMBOL(pmic_tcled_ind_get_blink_pattern);
EXPORT_SYMBOL(pmic_tcled_fun_set_current);
EXPORT_SYMBOL(pmic_tcled_fun_get_current);
EXPORT_SYMBOL(pmic_tcled_fun_set_cycletime);
EXPORT_SYMBOL(pmic_tcled_fun_get_cycletime);
EXPORT_SYMBOL(pmic_tcled_fun_set_dutycycle);
EXPORT_SYMBOL(pmic_tcled_fun_get_dutycycle);
EXPORT_SYMBOL(pmic_tcled_fun_blendedramps);
EXPORT_SYMBOL(pmic_tcled_fun_sawramps);
EXPORT_SYMBOL(pmic_tcled_fun_blendedbowtie);
EXPORT_SYMBOL(pmic_tcled_fun_chasinglightspattern);
EXPORT_SYMBOL(pmic_tcled_fun_strobe);
EXPORT_SYMBOL(pmic_tcled_fun_rampup);
EXPORT_SYMBOL(pmic_tcled_get_fun_rampup);
EXPORT_SYMBOL(pmic_tcled_fun_rampdown);
EXPORT_SYMBOL(pmic_tcled_get_fun_rampdown);
EXPORT_SYMBOL(pmic_tcled_fun_triode_on);
EXPORT_SYMBOL(pmic_tcled_fun_triode_off);
EXPORT_SYMBOL(pmic_tcled_enable_edge_slow);
EXPORT_SYMBOL(pmic_tcled_disable_edge_slow);
EXPORT_SYMBOL(pmic_tcled_enable_half_current);
EXPORT_SYMBOL(pmic_tcled_disable_half_current);
EXPORT_SYMBOL(pmic_tcled_enable_audio_modulation);
EXPORT_SYMBOL(pmic_tcled_disable_audio_modulation);
EXPORT_SYMBOL(pmic_bklit_set_boost_mode);
EXPORT_SYMBOL(pmic_bklit_get_boost_mode);
EXPORT_SYMBOL(pmic_bklit_config_boost_mode);
EXPORT_SYMBOL(pmic_bklit_gets_boost_mode);

/*!
 * This is the suspend of power management for the pmic light API.
 * It suports SAVE and POWER_DOWN state.
 *
 * @param        pdev           the device
 * @param        state          the state
 *
 * @return       This function returns 0 if successful.
 */
static int pmic_light_suspend(struct platform_device *dev, pm_message_t state)
{
	suspend_flag = 1;
	/* switch off all leds and backlights */
	CHECK_ERROR(pmic_light_init_reg());

	return 0;
};

/*!
 * This is the resume of power management for the pmic light API.
 * It suports RESTORE state.
 *
 * @param        dev            the device
 *
 * @return       This function returns 0 if successful.
 */
static int pmic_light_resume(struct platform_device *pdev)
{
	suspend_flag = 0;
	while (swait > 0) {
		swait--;
		wake_up_interruptible(&suspendq);
	}

	return 0;
};

/*!
 * This function enables backlight & tcled.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_bklit_tcled_master_enable(void)
{
	unsigned int reg_value = 0;
	unsigned int mask = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	reg_value = BITFVAL(BIT_LEDEN, 1);
	mask = BITFMASK(BIT_LEDEN);
	CHECK_ERROR(pmic_write_reg(LREG_0, reg_value, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function disables backlight & tcled.
 *
 * @return       This function returns PMIC_SUCCESS if successful
 */
PMIC_STATUS pmic_bklit_tcled_master_disable(void)
{
	unsigned int reg_value = 0;
	unsigned int mask = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	reg_value = BITFVAL(BIT_LEDEN, 0);
	mask = BITFMASK(BIT_LEDEN);
	CHECK_ERROR(pmic_write_reg(LREG_0, reg_value, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function enables backlight. Not supported on mc13783
 * Use pmic_bklit_tcled_master_enable.
 *
 * @return       This function returns PMIC_NOT_SUPPORTED
 */
PMIC_STATUS pmic_bklit_master_enable(void)
{
	return PMIC_NOT_SUPPORTED;
}

/*!
 * This function disables backlight. Not supported on mc13783
 * Use pmic_bklit_tcled_master_enable.
 *
 * @return       This function returns PMIC_NOT_SUPPORTED
 */
PMIC_STATUS pmic_bklit_master_disable(void)
{
	return PMIC_NOT_SUPPORTED;
}

/*!
 * This function sets backlight current level.
 *
 * @param        channel   Backlight channel
 * @param        level     Backlight current level, as the following table.
 *                         @verbatim
 * 		level     main & aux   keyboard
 *		 ------    -----------  --------
 *		 0         0 mA         0 mA
 *		 1         3 mA         12 mA
 *		 2         6 mA         24 mA
 *		 3         9 mA         36 mA
 *		 4         12 mA        48 mA
 *		 5         15 mA        60 mA
 *		 6         18 mA        72 mA
 *		 7         21 mA        84 mA
 *		 @endverbatim
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_bklit_set_current(t_bklit_channel channel, unsigned char level)
{
	unsigned int mask;
	unsigned int value;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	switch (channel) {
	case BACKLIGHT_LED1:
		value = BITFVAL(BIT_CL_MAIN, level);
		mask = BITFMASK(BIT_CL_MAIN);
		break;
	case BACKLIGHT_LED2:
		value = BITFVAL(BIT_CL_AUX, level);
		mask = BITFMASK(BIT_CL_AUX);
		break;
	case BACKLIGHT_LED3:
		value = BITFVAL(BIT_CL_KEY, level);
		mask = BITFMASK(BIT_CL_KEY);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}
	CHECK_ERROR(pmic_write_reg(LREG_2, value, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function retrives backlight current level.
 * The channels are not individually adjustable, hence
 * the channel parameter is ignored.
 *
 * @param        channel   Backlight channel (Ignored because the
 *                         channels are not individually adjustable)
 * @param        level     Pointer to store backlight current level result.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_bklit_get_current(t_bklit_channel channel,
				   unsigned char *level)
{
	unsigned int reg_value = 0;
	unsigned int mask = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	switch (channel) {
	case BACKLIGHT_LED1:
		mask = BITFMASK(BIT_CL_MAIN);
		break;
	case BACKLIGHT_LED2:
		mask = BITFMASK(BIT_CL_AUX);
		break;
	case BACKLIGHT_LED3:
		mask = BITFMASK(BIT_CL_KEY);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_read_reg(LREG_2, &reg_value, mask));

	switch (channel) {
	case BACKLIGHT_LED1:
		*level = BITFEXT(reg_value, BIT_CL_MAIN);
		break;
	case BACKLIGHT_LED2:
		*level = BITFEXT(reg_value, BIT_CL_AUX);
		break;
	case BACKLIGHT_LED3:
		*level = BITFEXT(reg_value, BIT_CL_KEY);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	return PMIC_SUCCESS;
}

/*!
 * This function sets a backlight channel duty cycle.
 * LED perceived brightness for each zone may be individually set by setting
 * duty cycle. The default setting is for 0% duty cycle; this keeps all zone
 * drivers turned off even after the master enable command. Each LED current
 * sink can be turned on and adjusted for brightness with an independent 4 bit
 * word for a duty cycle ranging from 0% to 100% in approximately 6.7% steps.
 *
 * @param        channel   Backlight channel.
 * @param        dc        Backlight duty cycle, as the following table.
 *                         @verbatim
 *		 dc        Duty Cycle (% On-time over Cycle Time)
 *		 ------    ---------------------------------------
 *		 0        0%
 *		 1        6.7%
 *		 2        13.3%
 *		 3        20%
 *		 4        26.7%
 *		 5        33.3%
 *		 6        40%
 *		 7        46.7%
 *		 8        53.3%
 *		 9        60%
 *		 10        66.7%
 *		 11        73.3%
 *		 12        80%
 *		 13        86.7%
 *		 14        93.3%
 *		 15        100%
 *		 @endverbatim
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_bklit_set_dutycycle(t_bklit_channel channel, unsigned char dc)
{
	unsigned int reg_value = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	if (dc > 15) {
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_read_reg(LREG_2, &reg_value, PMIC_ALL_BITS));

	switch (channel) {
	case BACKLIGHT_LED1:
		reg_value = reg_value & (~MASK_DUTY_CYCLE);
		reg_value = reg_value | (dc << BIT_DUTY_CYCLE);
		break;
	case BACKLIGHT_LED2:
		reg_value = reg_value & (~(MASK_DUTY_CYCLE << INDEX_AUX));
		reg_value = reg_value | (dc << (BIT_DUTY_CYCLE + INDEX_AUX));
		break;
	case BACKLIGHT_LED3:
		reg_value = reg_value & (~(MASK_DUTY_CYCLE << INDEX_KYD));
		reg_value = reg_value | (dc << (BIT_DUTY_CYCLE + INDEX_KYD));
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}
	CHECK_ERROR(pmic_write_reg(LREG_2, reg_value, PMIC_ALL_BITS));
	return PMIC_SUCCESS;

}

/*!
 * This function retrives a backlight channel duty cycle.
 *
 * @param        channel   Backlight channel.
 * @param        dc        Pointer to backlight duty cycle.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_bklit_get_dutycycle(t_bklit_channel channel, unsigned char *dc)
{
	unsigned int reg_value = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}
	CHECK_ERROR(pmic_read_reg(LREG_2, &reg_value, PMIC_ALL_BITS));

	switch (channel) {
	case BACKLIGHT_LED1:
		*dc = (int)((reg_value & (MASK_DUTY_CYCLE))
			    >> BIT_DUTY_CYCLE);

		break;
	case BACKLIGHT_LED2:
		*dc = (int)((reg_value & (MASK_DUTY_CYCLE << INDEX_AUX))
			    >> (BIT_DUTY_CYCLE + INDEX_AUX));
		break;
	case BACKLIGHT_LED3:
		*dc = (int)((reg_value & (MASK_DUTY_CYCLE <<
					  INDEX_KYD)) >> (BIT_DUTY_CYCLE +
							  INDEX_KYD));
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	return PMIC_SUCCESS;
}

/*!
 * This function sets a backlight channel cycle time.
 * Cycle Time is defined as the period of a complete cycle of
 * Time_on + Time_off. The default Cycle Time is set to 0.01 seconds such that
 * the 100 Hz on-off cycling is averaged out by the eye to eliminate
 * flickering. Additionally, the Cycle Time can be programmed to intentionally
 * extend the period of on-off cycles for a visual pulsating or blinking effect.
 *
 * @param        period    Backlight cycle time, as the following table.
 *                         @verbatim
 *		 period      Cycle Time
 *		 --------    ------------
 *		 0          0.01 seconds
 *		 1          0.1 seconds
 *		 2          0.5 seconds
 *		 3          2 seconds
 *		 @endverbatim
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_bklit_set_cycle_time(unsigned char period)
{
	unsigned int mask;
	unsigned int value;

	if (suspend_flag == 1) {
		return -EBUSY;
	}
	if (period > 3) {
		return PMIC_PARAMETER_ERROR;
	}
	mask = BITFMASK(BIT_PERIOD);
	value = BITFVAL(BIT_PERIOD, period);
	CHECK_ERROR(pmic_write_reg(LREG_2, value, mask));
	return PMIC_SUCCESS;
}

/*!
 * This function retrives a backlight channel cycle time setting.
 *
 * @param        period    Pointer to save backlight cycle time setting result.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_bklit_get_cycle_time(unsigned char *period)
{
	unsigned int mask;
	unsigned int value;

	if (suspend_flag == 1) {
		return -EBUSY;
	}
	mask = BITFMASK(BIT_PERIOD);
	CHECK_ERROR(pmic_read_reg(LREG_2, &value, mask));
	*period = BITFEXT(value, BIT_PERIOD);
	return PMIC_SUCCESS;
}

/*!
 * This function sets backlight operation mode. There are two modes of
 * operations: current control and triode mode.
 * The Duty Cycle/Cycle Time control is retained in Triode Mode. Audio
 * coupling is not available in Triode Mode.
 *
 * @param        channel   Backlight channel.
 * @param        mode      Backlight operation mode.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_bklit_set_mode(t_bklit_channel channel, t_bklit_mode mode)
{
	unsigned int reg_value = 0;
	unsigned int clear_val = 0;
	unsigned int triode_val = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	CHECK_ERROR(pmic_read_reg(LREG_0, &reg_value, PMIC_ALL_BITS));

	switch (channel) {
	case BACKLIGHT_LED1:
		clear_val = ~(MASK_TRIODE_MAIN_BL);
		triode_val = MASK_TRIODE_MAIN_BL;
		break;
	case BACKLIGHT_LED2:
		clear_val = ~(MASK_TRIODE_MAIN_BL << INDEX_AUXILIARY);
		triode_val = (MASK_TRIODE_MAIN_BL << INDEX_AUXILIARY);
		break;
	case BACKLIGHT_LED3:
		clear_val = ~(MASK_TRIODE_MAIN_BL << INDEX_KEYPAD);
		triode_val = (MASK_TRIODE_MAIN_BL << INDEX_KEYPAD);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	reg_value = (reg_value & clear_val);

	if (mode == BACKLIGHT_TRIODE_MODE) {
		reg_value = (reg_value | triode_val);
	}

	CHECK_ERROR(pmic_write_reg(LREG_0, reg_value, PMIC_ALL_BITS));
	return PMIC_SUCCESS;
}

/*!
 * This function gets backlight operation mode. There are two modes of
 * operations: current control and triode mode.
 * The Duty Cycle/Cycle Time control is retained in Triode Mode. Audio
 * coupling is not available in Triode Mode.
 *
 * @param        channel   Backlight channel.
 * @param        mode      Backlight operation mode.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_bklit_get_mode(t_bklit_channel channel, t_bklit_mode *mode)
{
	unsigned int reg_value = 0;
	unsigned int mask = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	switch (channel) {
	case BACKLIGHT_LED1:
		mask = BITFMASK(BIT_TRIODE_MAIN_BL);
		break;
	case BACKLIGHT_LED2:
		mask = BITFMASK(BIT_TRIODE_AUX_BL);
		break;
	case BACKLIGHT_LED3:
		mask = BITFMASK(BIT_TRIODE_KEY_BL);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_read_reg(LREG_0, &reg_value, mask));

	switch (channel) {
	case BACKLIGHT_LED1:
		*mode = BITFEXT(reg_value, BIT_TRIODE_MAIN_BL);
		break;
	case BACKLIGHT_LED2:
		*mode = BITFEXT(reg_value, BIT_TRIODE_AUX_BL);
		break;
	case BACKLIGHT_LED3:
		*mode = BITFEXT(reg_value, BIT_TRIODE_KEY_BL);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	return PMIC_SUCCESS;
}

/*!
 * This function starts backlight brightness ramp up function; ramp time is
 * fixed at 0.5 seconds.
 *
 * @param        channel   Backlight channel.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_bklit_rampup(t_bklit_channel channel)
{
	unsigned int reg_value = 0;
	unsigned int mask = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	switch (channel) {
	case BACKLIGHT_LED1:
		mask = BITFMASK(BIT_UP_MAIN_BL);
		reg_value = BITFVAL(BIT_UP_MAIN_BL, 1);
		break;
	case BACKLIGHT_LED2:
		mask = BITFMASK(BIT_UP_AUX_BL);
		reg_value = BITFVAL(BIT_UP_AUX_BL, 1);
		break;
	case BACKLIGHT_LED3:
		mask = BITFMASK(BIT_UP_KEY_BL);
		reg_value = BITFVAL(BIT_UP_KEY_BL, 1);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(LREG_0, reg_value, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function stops backlight brightness ramp up function;
 *
 * @param        channel   Backlight channel.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_bklit_off_rampup(t_bklit_channel channel)
{
	unsigned int reg_value = 0;
	unsigned int mask = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	switch (channel) {
	case BACKLIGHT_LED1:
		mask = BITFMASK(BIT_UP_MAIN_BL);
		break;
	case BACKLIGHT_LED2:
		mask = BITFMASK(BIT_UP_AUX_BL);
		break;
	case BACKLIGHT_LED3:
		mask = BITFMASK(BIT_UP_KEY_BL);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(LREG_0, reg_value, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function starts backlight brightness ramp down function; ramp time is
 * fixed at 0.5 seconds.
 *
 * @param        channel   Backlight channel.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_bklit_rampdown(t_bklit_channel channel)
{
	unsigned int reg_value = 0;
	unsigned int mask = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	switch (channel) {
	case BACKLIGHT_LED1:
		mask = BITFMASK(BIT_DOWN_MAIN_BL);
		reg_value = BITFVAL(BIT_DOWN_MAIN_BL, 1);
		break;
	case BACKLIGHT_LED2:
		mask = BITFMASK(BIT_DOWN_AUX_BL);
		reg_value = BITFVAL(BIT_DOWN_AUX_BL, 1);
		break;
	case BACKLIGHT_LED3:
		mask = BITFMASK(BIT_DOWN_KEY_BL);
		reg_value = BITFVAL(BIT_DOWN_KEY_BL, 1);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(LREG_0, reg_value, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function stops backlight brightness ramp down function.
 *
 * @param        channel   Backlight channel.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_bklit_off_rampdown(t_bklit_channel channel)
{
	unsigned int reg_value = 0;
	unsigned int mask = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	switch (channel) {
	case BACKLIGHT_LED1:
		mask = BITFMASK(BIT_DOWN_MAIN_BL);
		break;
	case BACKLIGHT_LED2:
		mask = BITFMASK(BIT_DOWN_AUX_BL);
		break;
	case BACKLIGHT_LED3:
		mask = BITFMASK(BIT_DOWN_KEY_BL);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(LREG_0, reg_value, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function enables backlight analog edge slowing mode. Analog Edge
 * Slowing slows down the transient edges to reduce the chance of coupling LED
 * modulation activity into other circuits. Rise and fall times will be targeted
 * for approximately 50usec.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_bklit_enable_edge_slow(void)
{
	unsigned int mask;
	unsigned int value;

	if (suspend_flag == 1) {
		return -EBUSY;
	}
	mask = BITFMASK(BIT_SLEWLIMBL);
	value = BITFVAL(BIT_SLEWLIMBL, 1);
	CHECK_ERROR(pmic_write_reg(LREG_2, value, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function disables backlight analog edge slowing mode. The backlight
 * drivers will default to an <93>Instant On<94> mode.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_bklit_disable_edge_slow(void)
{
	unsigned int mask;

	if (suspend_flag == 1) {
		return -EBUSY;
	}
	mask = BITFMASK(BIT_SLEWLIMBL);
	CHECK_ERROR(pmic_write_reg(LREG_2, 0, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function gets backlight analog edge slowing mode. DThe backlight
 *
 * @param        edge      Edge slowing mode.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_bklit_get_edge_slow(bool *edge)
{
	unsigned int mask;
	unsigned int value;

	if (suspend_flag == 1) {
		return -EBUSY;
	}
	mask = BITFMASK(BIT_SLEWLIMBL);
	CHECK_ERROR(pmic_read_reg(LREG_2, &value, mask));
	*edge = (bool) BITFEXT(value, BIT_SLEWLIMBL);

	return PMIC_SUCCESS;
}

/*!
 * This function sets backlight Strobe Light Pulsing mode.
 *
 * @param        channel   Backlight channel.
 * @param        mode      Strobe Light Pulsing mode.
 *
 * @return       This function returns PMIC_NOT_SUPPORTED.
 */
PMIC_STATUS pmic_bklit_set_strobemode(t_bklit_channel channel,
				      t_bklit_strobe_mode mode)
{
	return PMIC_NOT_SUPPORTED;
}

/*!
 * This function enables tri-color LED.
 *
 * @param        mode      Tri-color LED operation mode.
 * @param	 bank      Selected tri-color bank
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_tcled_enable(t_tcled_mode mode, t_funlight_bank bank)
{
	unsigned int mask = 0;
	unsigned int value = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	switch (mode) {
	case TCLED_FUN_MODE:
		switch (bank) {
		case TCLED_FUN_BANK1:
			mask = MASK_BK1_FL;
			value = MASK_BK1_FL;
			break;
		case TCLED_FUN_BANK2:
			mask = MASK_BK2_FL;
			value = MASK_BK2_FL;
			break;
		case TCLED_FUN_BANK3:
			mask = MASK_BK3_FL;
			value = MASK_BK3_FL;
			break;
		default:
			return PMIC_PARAMETER_ERROR;
		}
		break;
	case TCLED_IND_MODE:
		mask = MASK_BK1_FL | MASK_BK2_FL | MASK_BK3_FL;
		break;
	}

	CHECK_ERROR(pmic_write_reg(LREG_0, value, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function disables tri-color LED.
 *
 * @param        bank      Selected tri-color bank
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 *
 */
PMIC_STATUS pmic_tcled_disable(t_funlight_bank bank)
{
	unsigned int mask = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	switch (bank) {
	case TCLED_FUN_BANK1:
		mask = MASK_BK1_FL;
		break;
	case TCLED_FUN_BANK2:
		mask = MASK_BK2_FL;
		break;
	case TCLED_FUN_BANK3:
		mask = MASK_BK3_FL;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(LREG_0, 0, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function retrives tri-color LED operation mode.
 *
 * @param        mode      Pointer to Tri-color LED operation mode.
 * @param        bank      Selected tri-color bank
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_tcled_get_mode(t_tcled_mode *mode, t_funlight_bank bank)
{
	unsigned int val;
	unsigned int mask;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	switch (bank) {
	case TCLED_FUN_BANK1:
		mask = MASK_BK1_FL;
		break;
	case TCLED_FUN_BANK2:
		mask = MASK_BK2_FL;
		break;
	case TCLED_FUN_BANK3:
		mask = MASK_BK3_FL;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_read_reg(LREG_0, &val, mask));

	if (val) {
		*mode = TCLED_FUN_MODE;
	} else {
		*mode = TCLED_IND_MODE;
	}

	return PMIC_SUCCESS;
}

/*!
 * This function sets a tri-color LED channel current level in indicator mode.
 *
 * @param        channel      Tri-color LED channel.
 * @param        level        Current level.
 * @param        bank         Selected tri-color bank
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_tcled_ind_set_current(t_ind_channel channel,
				       t_tcled_cur_level level,
				       t_funlight_bank bank)
{
	unsigned int reg_conf = 0;
	unsigned int mask;
	unsigned int value;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	if (level > TCLED_CUR_LEVEL_4) {
		return PMIC_PARAMETER_ERROR;
	}

	switch (bank) {
	case TCLED_FUN_BANK1:
		reg_conf = LREG_3;
		break;
	case TCLED_FUN_BANK2:
		reg_conf = LREG_4;
		break;
	case TCLED_FUN_BANK3:
		reg_conf = LREG_5;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	switch (channel) {
	case TCLED_IND_RED:
		value = BITFVAL(BITS_CL_RED, level);
		mask = BITFMASK(BITS_CL_RED);
		break;
	case TCLED_IND_GREEN:
		value = BITFVAL(BITS_CL_GREEN, level);
		mask = BITFMASK(BITS_CL_GREEN);
		break;
	case TCLED_IND_BLUE:
		value = BITFVAL(BITS_CL_BLUE, level);
		mask = BITFMASK(BITS_CL_BLUE);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(reg_conf, value, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function retrives a tri-color LED channel current level
 * in indicator mode.
 *
 * @param        channel      Tri-color LED channel.
 * @param        level        Pointer to current level.
 * @param        bank         Selected tri-color bank
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_tcled_ind_get_current(t_ind_channel channel,
				       t_tcled_cur_level *level,
				       t_funlight_bank bank)
{
	unsigned int reg_conf = 0;
	unsigned int mask = 0;
	unsigned int value = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	switch (bank) {
	case TCLED_FUN_BANK1:
		reg_conf = LREG_3;
		break;
	case TCLED_FUN_BANK2:
		reg_conf = LREG_4;
		break;
	case TCLED_FUN_BANK3:
		reg_conf = LREG_5;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	switch (channel) {
	case TCLED_IND_RED:
		mask = BITFMASK(BITS_CL_RED);
		break;
	case TCLED_IND_GREEN:
		mask = BITFMASK(BITS_CL_GREEN);
		break;
	case TCLED_IND_BLUE:
		mask = BITFMASK(BITS_CL_BLUE);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_read_reg(reg_conf, &value, mask));

	switch (channel) {
	case TCLED_IND_RED:
		*level = BITFEXT(value, BITS_CL_RED);
		break;
	case TCLED_IND_GREEN:
		*level = BITFEXT(value, BITS_CL_GREEN);
		break;
	case TCLED_IND_BLUE:
		*level = BITFEXT(value, BITS_CL_BLUE);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	return PMIC_SUCCESS;
}

/*!
 * This function sets a tri-color LED channel blinking pattern in indication
 * mode.
 *
 * @param        channel      Tri-color LED channel.
 * @param        pattern      Blinking pattern.
 * @param        skip         If true, skip a cycle after each cycle.
 * @param        bank         Selected tri-color bank
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_tcled_ind_set_blink_pattern(t_ind_channel channel,
					     t_tcled_ind_blink_pattern pattern,
					     bool skip, t_funlight_bank bank)
{
	unsigned int reg_conf = 0;
	unsigned int mask = 0;
	unsigned int value = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	if (skip == true) {
		return PMIC_NOT_SUPPORTED;
	}

	switch (bank) {
	case TCLED_FUN_BANK1:
		reg_conf = LREG_3;
		break;
	case TCLED_FUN_BANK2:
		reg_conf = LREG_4;
		break;
	case TCLED_FUN_BANK3:
		reg_conf = LREG_5;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	switch (channel) {
	case TCLED_IND_RED:
		value = BITFVAL(BITS_DC_RED, pattern);
		mask = BITFMASK(BITS_DC_RED);
		break;
	case TCLED_IND_GREEN:
		value = BITFVAL(BITS_DC_GREEN, pattern);
		mask = BITFMASK(BITS_DC_GREEN);
		break;
	case TCLED_IND_BLUE:
		value = BITFVAL(BITS_DC_BLUE, pattern);
		mask = BITFMASK(BITS_DC_BLUE);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(reg_conf, value, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function retrives a tri-color LED channel blinking pattern in
 * indication mode.
 *
 * @param        channel      Tri-color LED channel.
 * @param        pattern      Pointer to Blinking pattern.
 * @param        skip         Pointer to a boolean varible indicating if skip
 * @param        bank         Selected tri-color bank
 *                            a cycle after each cycle.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_tcled_ind_get_blink_pattern(t_ind_channel channel,
					     t_tcled_ind_blink_pattern *
					     pattern, bool *skip,
					     t_funlight_bank bank)
{
	unsigned int reg_conf = 0;
	unsigned int mask = 0;
	unsigned int value = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	switch (bank) {
	case TCLED_FUN_BANK1:
		reg_conf = LREG_3;
		break;
	case TCLED_FUN_BANK2:
		reg_conf = LREG_4;
		break;
	case TCLED_FUN_BANK3:
		reg_conf = LREG_5;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	switch (channel) {
	case TCLED_IND_RED:
		mask = BITFMASK(BITS_DC_RED);
		break;
	case TCLED_IND_GREEN:
		mask = BITFMASK(BITS_DC_GREEN);
		break;
	case TCLED_IND_BLUE:
		mask = BITFMASK(BITS_DC_BLUE);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_read_reg(reg_conf, &value, mask));

	switch (channel) {
	case TCLED_IND_RED:
		*pattern = BITFEXT(value, BITS_DC_RED);
		break;
	case TCLED_IND_GREEN:
		*pattern = BITFEXT(value, BITS_DC_GREEN);
		break;
	case TCLED_IND_BLUE:
		*pattern = BITFEXT(value, BITS_DC_BLUE);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	return PMIC_SUCCESS;
}

/*!
 * This function sets a tri-color LED channel current level in Fun Light mode.
 *
 * @param        bank         Tri-color LED bank
 * @param        channel      Tri-color LED channel.
 * @param        level        Current level.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_tcled_fun_set_current(t_funlight_bank bank,
				       t_funlight_channel channel,
				       t_tcled_cur_level level)
{
	unsigned int reg_conf = 0;
	unsigned int mask;
	unsigned int value;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	if (level > TCLED_CUR_LEVEL_4) {
		return PMIC_PARAMETER_ERROR;
	}

	switch (bank) {
	case TCLED_FUN_BANK1:
		reg_conf = LREG_3;
		break;
	case TCLED_FUN_BANK2:
		reg_conf = LREG_4;
		break;
	case TCLED_FUN_BANK3:
		reg_conf = LREG_5;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	switch (channel) {
	case TCLED_FUN_CHANNEL1:
		value = BITFVAL(BITS_CL_RED, level);
		mask = BITFMASK(BITS_CL_RED);
		break;
	case TCLED_FUN_CHANNEL2:
		value = BITFVAL(BITS_CL_GREEN, level);
		mask = BITFMASK(BITS_CL_GREEN);
		break;
	case TCLED_FUN_CHANNEL3:
		value = BITFVAL(BITS_CL_BLUE, level);
		mask = BITFMASK(BITS_CL_BLUE);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(reg_conf, value, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function retrives a tri-color LED channel current level
 * in Fun Light mode.
 *
 * @param        bank         Tri-color LED bank
 * @param        channel      Tri-color LED channel.
 * @param        level        Pointer to current level.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_tcled_fun_get_current(t_funlight_bank bank,
				       t_funlight_channel channel,
				       t_tcled_cur_level *level)
{
	unsigned int reg_conf = 0;
	unsigned int mask = 0;
	unsigned int value = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	switch (bank) {
	case TCLED_FUN_BANK1:
		reg_conf = LREG_3;
		break;
	case TCLED_FUN_BANK2:
		reg_conf = LREG_4;
		break;
	case TCLED_FUN_BANK3:
		reg_conf = LREG_5;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	switch (channel) {
	case TCLED_FUN_CHANNEL1:
		mask = BITFMASK(BITS_CL_RED);
		break;
	case TCLED_FUN_CHANNEL2:
		mask = BITFMASK(BITS_CL_GREEN);
		break;
	case TCLED_FUN_CHANNEL3:
		mask = BITFMASK(BITS_CL_BLUE);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_read_reg(reg_conf, &value, mask));

	switch (channel) {
	case TCLED_FUN_CHANNEL1:
		*level = BITFEXT(value, BITS_CL_RED);
		break;
	case TCLED_FUN_CHANNEL2:
		*level = BITFEXT(value, BITS_CL_GREEN);
		break;
	case TCLED_FUN_CHANNEL3:
		*level = BITFEXT(value, BITS_CL_BLUE);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	return PMIC_SUCCESS;
}

/*!
 * This function sets tri-color LED cycle time.
 *
 * @param        bank         Tri-color LED bank
 * @param        ct           Cycle time.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_tcled_fun_set_cycletime(t_funlight_bank bank,
					 t_tcled_fun_cycle_time ct)
{
	unsigned int reg_conf = 0;
	unsigned int mask;
	unsigned int value;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	if (ct > TC_CYCLE_TIME_4) {
		return PMIC_PARAMETER_ERROR;
	}

	switch (bank) {
	case TCLED_FUN_BANK1:
		reg_conf = LREG_3;
		break;
	case TCLED_FUN_BANK2:
		reg_conf = LREG_4;
		break;
	case TCLED_FUN_BANK3:
		reg_conf = LREG_5;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	value = BITFVAL(BIT_PERIOD, ct);
	mask = BITFMASK(BIT_PERIOD);

	CHECK_ERROR(pmic_write_reg(reg_conf, value, mask));
	return PMIC_SUCCESS;
}

/*!
 * This function retrives tri-color LED cycle time in Fun Light mode.
 *
 * @param        bank         Tri-color LED bank
 * @param        ct           Pointer to cycle time.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_tcled_fun_get_cycletime(t_funlight_bank bank,
					 t_tcled_fun_cycle_time *ct)
{
	unsigned int reg_conf = 0;
	unsigned int mask;
	unsigned int value;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	if (*ct > TC_CYCLE_TIME_4) {
		return PMIC_PARAMETER_ERROR;
	}

	switch (bank) {
	case TCLED_FUN_BANK1:
		reg_conf = LREG_3;
		break;
	case TCLED_FUN_BANK2:
		reg_conf = LREG_4;
		break;
	case TCLED_FUN_BANK3:
		reg_conf = LREG_5;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	mask = BITFMASK(BIT_PERIOD);
	CHECK_ERROR(pmic_read_reg(reg_conf, &value, mask));

	*ct = BITFVAL(BIT_PERIOD, value);

	return PMIC_SUCCESS;
}

/*!
 * This function sets a tri-color LED channel duty cycle in Fun Light mode.
 *
 * @param        bank         Tri-color LED bank
 * @param        channel      Tri-color LED channel.
 * @param        dc           Duty cycle.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_tcled_fun_set_dutycycle(t_funlight_bank bank,
					 t_funlight_channel channel,
					 unsigned char dc)
{
	unsigned int reg_conf = 0;
	unsigned int mask = 0;
	unsigned int value = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	switch (bank) {
	case TCLED_FUN_BANK1:
		reg_conf = LREG_3;
		break;
	case TCLED_FUN_BANK2:
		reg_conf = LREG_4;
		break;
	case TCLED_FUN_BANK3:
		reg_conf = LREG_5;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	switch (channel) {
	case TCLED_FUN_CHANNEL1:
		value = BITFVAL(BITS_DC_RED, dc);
		mask = BITFMASK(BITS_DC_RED);
		break;
	case TCLED_FUN_CHANNEL2:
		value = BITFVAL(BITS_DC_GREEN, dc);
		mask = BITFMASK(BITS_DC_GREEN);
		break;
	case TCLED_FUN_CHANNEL3:
		value = BITFVAL(BITS_DC_BLUE, dc);
		mask = BITFMASK(BITS_DC_BLUE);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(reg_conf, value, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function retrives a tri-color LED channel duty cycle in Fun Light mode.
 *
 * @param        bank         Tri-color LED bank
 * @param        channel      Tri-color LED channel.
 * @param        dc           Pointer to duty cycle.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_tcled_fun_get_dutycycle(t_funlight_bank bank,
					 t_funlight_channel channel,
					 unsigned char *dc)
{
	unsigned int reg_conf = 0;
	unsigned int mask = 0;
	unsigned int value = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	switch (bank) {
	case TCLED_FUN_BANK1:
		reg_conf = LREG_3;
		break;
	case TCLED_FUN_BANK2:
		reg_conf = LREG_4;
		break;
	case TCLED_FUN_BANK3:
		reg_conf = LREG_5;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	switch (channel) {
	case TCLED_FUN_CHANNEL1:
		mask = BITFMASK(BITS_DC_RED);
		break;
	case TCLED_FUN_CHANNEL2:
		mask = BITFMASK(BITS_DC_GREEN);
		break;
	case TCLED_FUN_CHANNEL3:
		mask = BITFMASK(BITS_DC_BLUE);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_read_reg(reg_conf, &value, mask));

	switch (channel) {
	case TCLED_FUN_CHANNEL1:
		*dc = BITFEXT(value, BITS_DC_RED);
		break;
	case TCLED_FUN_CHANNEL2:
		*dc = BITFEXT(value, BITS_DC_GREEN);
		break;
	case TCLED_FUN_CHANNEL3:
		*dc = BITFEXT(value, BITS_DC_BLUE);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	return PMIC_SUCCESS;
}

/*!
 * This function initiates Blended Ramp fun light pattern.
 *
 * @param        bank         Tri-color LED bank
 * @param        speed        Speed of pattern.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_tcled_fun_blendedramps(t_funlight_bank bank,
					t_tcled_fun_speed speed)
{
	unsigned int mask = 0;
	unsigned int value = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	switch (speed) {
	case TC_OFF:
		value = BITFVAL(BITS_FUN_LIGHT, FUN_LIGHTS_OFF);
		break;
	case TC_SLOW:
		value = BITFVAL(BITS_FUN_LIGHT, BLENDED_RAMPS_SLOW);
		break;
	case TC_FAST:
		value = BITFVAL(BITS_FUN_LIGHT, BLENDED_RAMPS_FAST);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	mask = BITFMASK(BITS_FUN_LIGHT);
	CHECK_ERROR(pmic_write_reg(LREG_0, value, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function initiates Saw Ramp fun light pattern.
 *
 * @param        bank         Tri-color LED bank
 * @param        speed        Speed of pattern.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_tcled_fun_sawramps(t_funlight_bank bank,
				    t_tcled_fun_speed speed)
{
	unsigned int mask = 0;
	unsigned int value = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	switch (speed) {
	case TC_OFF:
		value = BITFVAL(BITS_FUN_LIGHT, FUN_LIGHTS_OFF);
		break;
	case TC_SLOW:
		value = BITFVAL(BITS_FUN_LIGHT, SAW_RAMPS_SLOW);
		break;
	case TC_FAST:
		value = BITFVAL(BITS_FUN_LIGHT, SAW_RAMPS_FAST);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	mask = BITFMASK(BITS_FUN_LIGHT);
	CHECK_ERROR(pmic_write_reg(LREG_0, value, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function initiates Blended Bowtie fun light pattern.
 *
 * @param        bank         Tri-color LED bank
 * @param        speed        Speed of pattern.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_tcled_fun_blendedbowtie(t_funlight_bank bank,
					 t_tcled_fun_speed speed)
{
	unsigned int mask = 0;
	unsigned int value = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	switch (speed) {
	case TC_OFF:
		value = BITFVAL(BITS_FUN_LIGHT, FUN_LIGHTS_OFF);
		break;
	case TC_SLOW:
		value = BITFVAL(BITS_FUN_LIGHT, BLENDED_INVERSE_RAMPS_SLOW);
		break;
	case TC_FAST:
		value = BITFVAL(BITS_FUN_LIGHT, BLENDED_INVERSE_RAMPS_FAST);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	mask = BITFMASK(BITS_FUN_LIGHT);
	CHECK_ERROR(pmic_write_reg(LREG_0, value, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function initiates Chasing Lights fun light pattern.
 *
 * @param        bank         Tri-color LED bank
 * @param        pattern      Chasing light pattern mode.
 * @param        speed        Speed of pattern.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_tcled_fun_chasinglightspattern(t_funlight_bank bank,
						t_chaselight_pattern pattern,
						t_tcled_fun_speed speed)
{
	unsigned int mask = 0;
	unsigned int value = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	if (pattern > BGR) {
		return PMIC_PARAMETER_ERROR;
	}

	switch (speed) {
	case TC_OFF:
		value = BITFVAL(BITS_FUN_LIGHT, FUN_LIGHTS_OFF);
		break;
	case TC_SLOW:
		if (pattern == PMIC_RGB) {
			value =
			    BITFVAL(BITS_FUN_LIGHT, CHASING_LIGHTS_RGB_SLOW);
		} else {
			value =
			    BITFVAL(BITS_FUN_LIGHT, CHASING_LIGHTS_BGR_SLOW);
		}
		break;
	case TC_FAST:
		if (pattern == PMIC_RGB) {
			value =
			    BITFVAL(BITS_FUN_LIGHT, CHASING_LIGHTS_RGB_FAST);
		} else {
			value =
			    BITFVAL(BITS_FUN_LIGHT, CHASING_LIGHTS_BGR_FAST);
		}
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	mask = BITFMASK(BITS_FUN_LIGHT);
	CHECK_ERROR(pmic_write_reg(LREG_0, value, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function initiates Strobe Mode fun light pattern.
 *
 * @param        bank         Tri-color LED bank
 * @param        channel      Tri-color LED channel.
 * @param        speed        Speed of pattern.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_tcled_fun_strobe(t_funlight_bank bank,
				  t_funlight_channel channel,
				  t_tcled_fun_strobe_speed speed)
{
	/* not supported on mc13783 */

	return PMIC_NOT_SUPPORTED;
}

/*!
 * This function initiates Tri-color LED brightness Ramp Up function; Ramp time
 * is fixed at 1 second.
 *
 * @param        bank         Tri-color LED bank
 * @param        channel      Tri-color LED channel.
 * @param        rampup       Ramp-up configuration.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_tcled_fun_rampup(t_funlight_bank bank,
				  t_funlight_channel channel, bool rampup)
{
	unsigned int mask = 0;
	unsigned int value = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	switch (bank) {
	case TCLED_FUN_BANK1:
		mask = LEDR1RAMPUP;
		value = LEDR1RAMPUP;
		break;
	case TCLED_FUN_BANK2:
		mask = LEDR2RAMPUP;
		value = LEDR2RAMPUP;
		break;
	case TCLED_FUN_BANK3:
		mask = LEDR3RAMPUP;
		value = LEDR3RAMPUP;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	switch (channel) {
	case TCLED_FUN_CHANNEL1:
		mask = mask;
		value = value;
		break;
	case TCLED_FUN_CHANNEL2:
		mask = mask * 2;
		value = value * 2;
		break;
	case TCLED_FUN_CHANNEL3:
		mask = mask * 4;
		value = value * 4;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	if (!rampup) {
		value = 0;
	}

	CHECK_ERROR(pmic_write_reg(LREG_1, value, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function gets Tri-color LED brightness Ramp Up function; Ramp time
 * is fixed at 1 second.
 *
 * @param        bank         Tri-color LED bank
 * @param        channel      Tri-color LED channel.
 * @param        rampup       Ramp-up configuration.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_tcled_get_fun_rampup(t_funlight_bank bank,
				      t_funlight_channel channel, bool *rampup)
{
	unsigned int mask = 0;
	unsigned int value = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	switch (bank) {
	case TCLED_FUN_BANK1:
		mask = LEDR1RAMPUP;
		break;
	case TCLED_FUN_BANK2:
		mask = LEDR2RAMPUP;
		break;
	case TCLED_FUN_BANK3:
		mask = LEDR3RAMPUP;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	switch (channel) {
	case TCLED_FUN_CHANNEL1:
		mask = mask;
		break;
	case TCLED_FUN_CHANNEL2:
		mask = mask * 2;
		break;
	case TCLED_FUN_CHANNEL3:
		mask = mask * 4;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_read_reg(LREG_1, &value, mask));
	if (value) {
		*rampup = true;
	} else {
		*rampup = false;
	}

	return PMIC_SUCCESS;
}

/*!
 * This function initiates Tri-color LED brightness Ramp Down function; Ramp
 * time is fixed at 1 second.
 *
 * @param        bank         Tri-color LED bank
 * @param        channel      Tri-color LED channel.
 * @param        rampdown     Ramp-down configuration.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_tcled_fun_rampdown(t_funlight_bank bank,
				    t_funlight_channel channel, bool rampdown)
{
	unsigned int mask = 0;
	unsigned int value = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	switch (bank) {
	case TCLED_FUN_BANK1:
		mask = LEDR1RAMPDOWN;
		value = LEDR1RAMPDOWN;
		break;
	case TCLED_FUN_BANK2:
		mask = LEDR2RAMPDOWN;
		value = LEDR2RAMPDOWN;
		break;
	case TCLED_FUN_BANK3:
		mask = LEDR3RAMPDOWN;
		value = LEDR3RAMPDOWN;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	switch (channel) {
	case TCLED_FUN_CHANNEL1:
		mask = mask;
		value = value;
		break;
	case TCLED_FUN_CHANNEL2:
		mask = mask * 2;
		value = value * 2;
		break;
	case TCLED_FUN_CHANNEL3:
		mask = mask * 4;
		value = value * 4;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	if (!rampdown) {
		value = 0;
	}

	CHECK_ERROR(pmic_write_reg(LREG_1, value, mask));
	return PMIC_SUCCESS;
}

/*!
 * This function initiates Tri-color LED brightness Ramp Down function; Ramp
 * time is fixed at 1 second.
 *
 * @param        bank         Tri-color LED bank
 * @param        channel      Tri-color LED channel.
 * @param        rampdown     Ramp-down configuration.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_tcled_get_fun_rampdown(t_funlight_bank bank,
					t_funlight_channel channel,
					bool *rampdown)
{
	unsigned int mask = 0;
	unsigned int value = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	switch (bank) {
	case TCLED_FUN_BANK1:
		mask = LEDR1RAMPDOWN;
		break;
	case TCLED_FUN_BANK2:
		mask = LEDR2RAMPDOWN;
		break;
	case TCLED_FUN_BANK3:
		mask = LEDR3RAMPDOWN;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	switch (channel) {
	case TCLED_FUN_CHANNEL1:
		mask = mask;
		break;
	case TCLED_FUN_CHANNEL2:
		mask = mask * 2;
		break;
	case TCLED_FUN_CHANNEL3:
		mask = mask * 4;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_read_reg(LREG_1, &value, mask));
	if (value) {
		*rampdown = true;
	} else {
		*rampdown = false;
	}
	return PMIC_SUCCESS;
}

/*!
 * This function enables a Tri-color channel triode mode.
 *
 * @param        bank         Tri-color LED bank
 * @param        channel      Tri-color LED channel.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_tcled_fun_triode_on(t_funlight_bank bank,
				     t_funlight_channel channel)
{
	unsigned int mask = 0;
	unsigned int value = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	switch (bank) {
	case TCLED_FUN_BANK1:
		mask = MASK_BK1_FL;
		value = ENABLE_BK1_FL;
		break;
	case TCLED_FUN_BANK2:
		mask = MASK_BK2_FL;
		value = ENABLE_BK2_FL;
		break;
	case TCLED_FUN_BANK3:
		mask = MASK_BK3_FL;
		value = ENABLE_BK2_FL;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(LREG_0, value, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function disables a Tri-color LED channel triode mode.
 *
 * @param        bank         Tri-color LED bank
 * @param        channel      Tri-color LED channel.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_tcled_fun_triode_off(t_funlight_bank bank,
				      t_funlight_channel channel)
{
	unsigned int mask = 0;
	unsigned int value = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	switch (bank) {
	case TCLED_FUN_BANK1:
		mask = MASK_BK1_FL;
		break;
	case TCLED_FUN_BANK2:
		mask = MASK_BK2_FL;
		break;
	case TCLED_FUN_BANK3:
		mask = MASK_BK3_FL;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(LREG_0, value, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function enables Tri-color LED edge slowing.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_tcled_enable_edge_slow(void)
{
	unsigned int mask = 0;
	unsigned int value = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	value = BITFVAL(BIT_SLEWLIMTC, 1);
	mask = BITFMASK(BIT_SLEWLIMTC);

	CHECK_ERROR(pmic_write_reg(LREG_1, value, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function disables Tri-color LED edge slowing.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_tcled_disable_edge_slow(void)
{
	unsigned int mask = 0;
	unsigned int value = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	value = BITFVAL(BIT_SLEWLIMTC, 0);
	mask = BITFMASK(BIT_SLEWLIMTC);

	CHECK_ERROR(pmic_write_reg(LREG_1, value, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function enables Tri-color LED half current mode.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_tcled_enable_half_current(void)
{
	unsigned int mask = 0;
	unsigned int value = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	value = BITFVAL(BIT_TC1HALF, 1);
	mask = BITFMASK(BIT_TC1HALF);

	CHECK_ERROR(pmic_write_reg(LREG_1, value, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function disables Tri-color LED half current mode.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_tcled_disable_half_current(void)
{
	unsigned int mask = 0;
	unsigned int value = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	value = BITFVAL(BIT_TC1HALF, 0);
	mask = BITFMASK(BIT_TC1HALF);

	CHECK_ERROR(pmic_write_reg(LREG_1, value, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function enables backlight or Tri-color LED audio modulation.
 *
 * @return       This function returns PMIC_NOT_SUPPORTED.
 */
PMIC_STATUS pmic_tcled_enable_audio_modulation(t_led_channel channel,
					       t_aud_path path,
					       t_aud_gain gain, bool lpf_bypass)
{
	return PMIC_NOT_SUPPORTED;
}

/*!
 * This function disables backlight or Tri-color LED audio modulation.
 *
 * @return       This function returns PMIC_NOT_SUPPORTED.
 */
PMIC_STATUS pmic_tcled_disable_audio_modulation(void)
{
	return PMIC_NOT_SUPPORTED;
}

/*!
 * This function enables the boost mode.
 * Only on mc13783 2.0 or higher
 *
 * @param       en_dis   Enable or disable the boost mode
 *
 * @return      This function returns 0 if successful.
 */
PMIC_STATUS pmic_bklit_set_boost_mode(bool en_dis)
{

	pmic_version_t mc13783_ver;
	unsigned int mask;
	unsigned int value;
	mc13783_ver = pmic_get_version();
	if (mc13783_ver.revision >= 20) {

		if (suspend_flag == 1) {
			return -EBUSY;
		}

		value = BITFVAL(BIT_BOOSTEN, en_dis);
		mask = BITFMASK(BIT_BOOSTEN);
		CHECK_ERROR(pmic_write_reg(LREG_0, value, mask));
		return PMIC_SUCCESS;
	} else {
		return PMIC_NOT_SUPPORTED;
	}
}

/*!
 * This function gets the boost mode.
 * Only on mc13783 2.0 or higher
 *
 * @param       en_dis   Enable or disable the boost mode
 *
 * @return      This function returns 0 if successful.
 */
PMIC_STATUS pmic_bklit_get_boost_mode(bool *en_dis)
{
	pmic_version_t mc13783_ver;
	unsigned int mask;
	unsigned int value;
	mc13783_ver = pmic_get_version();
	if (mc13783_ver.revision >= 20) {

		if (suspend_flag == 1) {
			return -EBUSY;
		}
		mask = BITFMASK(BIT_BOOSTEN);
		CHECK_ERROR(pmic_read_reg(LREG_0, &value, mask));
		*en_dis = BITFEXT(value, BIT_BOOSTEN);
		return PMIC_SUCCESS;
	} else {
		return PMIC_NOT_SUPPORTED;
	}
}

/*!
 * This function sets boost mode configuration
 * Only on mc13783 2.0 or higher
 *
 * @param    abms      Define adaptive boost mode selection
 * @param    abr       Define adaptive boost reference
 *
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS pmic_bklit_config_boost_mode(unsigned int abms, unsigned int abr)
{
	unsigned int conf_boost = 0;
	unsigned int mask;
	unsigned int value;
	pmic_version_t mc13783_ver;

	mc13783_ver = pmic_get_version();
	if (mc13783_ver.revision >= 20) {
		if (suspend_flag == 1) {
			return -EBUSY;
		}

		if (abms > MAX_BOOST_ABMS) {
			return PMIC_PARAMETER_ERROR;
		}

		if (abr > MAX_BOOST_ABR) {
			return PMIC_PARAMETER_ERROR;
		}

		conf_boost = abms | (abr << 3);

		value = BITFVAL(BITS_BOOST, conf_boost);
		mask = BITFMASK(BITS_BOOST);
		CHECK_ERROR(pmic_write_reg(LREG_0, value, mask));

		return PMIC_SUCCESS;
	} else {
		return PMIC_NOT_SUPPORTED;
	}
}

/*!
 * This function gets boost mode configuration
 * Only on mc13783 2.0 or higher
 *
 * @param    abms      Define adaptive boost mode selection
 * @param    abr       Define adaptive boost reference
 *
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS pmic_bklit_gets_boost_mode(unsigned int *abms, unsigned int *abr)
{
	unsigned int mask;
	unsigned int value;
	pmic_version_t mc13783_ver;
	mc13783_ver = pmic_get_version();
	if (mc13783_ver.revision >= 20) {
		if (suspend_flag == 1) {
			return -EBUSY;
		}

		mask = BITFMASK(BITS_BOOST_ABMS);
		CHECK_ERROR(pmic_read_reg(LREG_0, &value, mask));
		*abms = BITFEXT(value, BITS_BOOST_ABMS);

		mask = BITFMASK(BITS_BOOST_ABR);
		CHECK_ERROR(pmic_read_reg(LREG_0, &value, mask));
		*abr = BITFEXT(value, BITS_BOOST_ABR);
		return PMIC_SUCCESS;
	} else {
		return PMIC_NOT_SUPPORTED;
	}
}

/*!
 * This function implements IOCTL controls on a PMIC Light device.
 *

 * @param        inode       pointer on the node
 * @param        file        pointer on the file
 * @param        cmd         the command
 * @param        arg         the parameter
 * @return       This function returns 0 if successful.
 */
static int pmic_light_ioctl(struct inode *inode, struct file *file,
			    unsigned int cmd, unsigned long arg)
{
	t_bklit_setting_param *bklit_setting = NULL;
	t_tcled_enable_param *tcled_setting;
	t_fun_param *fun_param;
	t_tcled_ind_param *tcled_ind;

	if (_IOC_TYPE(cmd) != 'p')
		return -ENOTTY;

	bklit_setting = kmalloc(sizeof(t_bklit_setting_param), GFP_KERNEL);
	tcled_setting = kmalloc(sizeof(t_tcled_enable_param), GFP_KERNEL);
	fun_param = kmalloc(sizeof(t_fun_param), GFP_KERNEL);
	tcled_ind = kmalloc(sizeof(t_tcled_ind_param), GFP_KERNEL);
	switch (cmd) {
	case PMIC_BKLIT_TCLED_ENABLE:
		pmic_bklit_tcled_master_enable();
		break;

	case PMIC_BKLIT_TCLED_DISABLE:
		pmic_bklit_tcled_master_disable();
		break;

	case PMIC_BKLIT_ENABLE:
		pmic_bklit_master_enable();
		break;

	case PMIC_BKLIT_DISABLE:
		pmic_bklit_master_disable();
		break;

	case PMIC_SET_BKLIT:
		if (bklit_setting == NULL)
			return -ENOMEM;

		if (copy_from_user(bklit_setting, (t_bklit_setting_param *) arg,
				   sizeof(t_bklit_setting_param))) {
			kfree(bklit_setting);
			return -EFAULT;
		}

		CHECK_ERROR_KFREE(pmic_bklit_set_mode(bklit_setting->channel,
						      bklit_setting->mode),
				  (kfree(bklit_setting)));

		CHECK_ERROR_KFREE(pmic_bklit_set_current(bklit_setting->channel,
							 bklit_setting->
							 current_level),
				  (kfree(bklit_setting)));
		CHECK_ERROR_KFREE(pmic_bklit_set_dutycycle
				  (bklit_setting->channel,
				   bklit_setting->duty_cycle),
				  (kfree(bklit_setting)));
		CHECK_ERROR_KFREE(pmic_bklit_set_cycle_time
				  (bklit_setting->cycle_time),
				  (kfree(bklit_setting)));
		CHECK_ERROR_KFREE(pmic_bklit_set_boost_mode
				  (bklit_setting->en_dis),
				  (kfree(bklit_setting)));
		CHECK_ERROR_KFREE(pmic_bklit_config_boost_mode
				  (bklit_setting->abms, bklit_setting->abr),
				  (kfree(bklit_setting)));
		if (bklit_setting->edge_slow != false) {
			CHECK_ERROR_KFREE(pmic_bklit_enable_edge_slow(),
					  (kfree(bklit_setting)));
		} else {
			CHECK_ERROR_KFREE(pmic_bklit_disable_edge_slow(),
					  (kfree(bklit_setting)));
		}

		kfree(bklit_setting);
		break;

	case PMIC_GET_BKLIT:
		if (bklit_setting == NULL)
			return -ENOMEM;

		if (copy_from_user(bklit_setting, (t_bklit_setting_param *) arg,
				   sizeof(t_bklit_setting_param))) {
			kfree(bklit_setting);
			return -EFAULT;
		}

		CHECK_ERROR_KFREE(pmic_bklit_get_current(bklit_setting->channel,
							 &bklit_setting->
							 current_level),
				  (kfree(bklit_setting)));
		CHECK_ERROR_KFREE(pmic_bklit_get_cycle_time
				  (&bklit_setting->cycle_time),
				  (kfree(bklit_setting)));
		CHECK_ERROR_KFREE(pmic_bklit_get_dutycycle
				  (bklit_setting->channel,
				   &bklit_setting->duty_cycle),
				  (kfree(bklit_setting)));
		bklit_setting->strobe = BACKLIGHT_STROBE_NONE;
		CHECK_ERROR_KFREE(pmic_bklit_get_mode(bklit_setting->channel,
						      &bklit_setting->mode),
				  (kfree(bklit_setting)));
		CHECK_ERROR_KFREE(pmic_bklit_get_edge_slow
				  (&bklit_setting->edge_slow),
				  (kfree(bklit_setting)));
		CHECK_ERROR_KFREE(pmic_bklit_get_boost_mode
				  (&bklit_setting->en_dis),
				  (kfree(bklit_setting)));
		CHECK_ERROR_KFREE(pmic_bklit_gets_boost_mode
				  (&bklit_setting->abms, &bklit_setting->abr),
				  (kfree(bklit_setting)));

		if (copy_to_user((t_bklit_setting_param *) arg, bklit_setting,
				 sizeof(t_bklit_setting_param))) {
			kfree(bklit_setting);
			return -EFAULT;
		}
		kfree(bklit_setting);
		break;

	case PMIC_RAMPUP_BKLIT:
		CHECK_ERROR(pmic_bklit_rampup((t_bklit_channel) arg));
		break;

	case PMIC_RAMPDOWN_BKLIT:
		CHECK_ERROR(pmic_bklit_rampdown((t_bklit_channel) arg));
		break;

	case PMIC_OFF_RAMPUP_BKLIT:
		CHECK_ERROR(pmic_bklit_off_rampup((t_bklit_channel) arg));
		break;

	case PMIC_OFF_RAMPDOWN_BKLIT:
		CHECK_ERROR(pmic_bklit_off_rampdown((t_bklit_channel) arg));
		break;

	case PMIC_TCLED_ENABLE:
		if (tcled_setting == NULL)
			return -ENOMEM;

		if (copy_from_user(tcled_setting, (t_tcled_enable_param *) arg,
				   sizeof(t_tcled_enable_param))) {
			kfree(tcled_setting);
			return -EFAULT;
		}
		CHECK_ERROR_KFREE(pmic_tcled_enable(tcled_setting->mode,
						    tcled_setting->bank),
				  (kfree(bklit_setting)));
		break;

	case PMIC_TCLED_DISABLE:
		CHECK_ERROR(pmic_tcled_disable((t_funlight_bank) arg));
		break;

	case PMIC_TCLED_PATTERN:
		if (fun_param == NULL)
			return -ENOMEM;

		if (copy_from_user(fun_param,
				   (t_fun_param *) arg, sizeof(t_fun_param))) {
			kfree(fun_param);
			return -EFAULT;
		}

		switch (fun_param->pattern) {
		case BLENDED_RAMPS_SLOW:
			CHECK_ERROR_KFREE(pmic_tcled_fun_blendedramps
					  (fun_param->bank, TC_SLOW),
					  (kfree(fun_param)));
			break;

		case BLENDED_RAMPS_FAST:
			CHECK_ERROR_KFREE(pmic_tcled_fun_blendedramps
					  (fun_param->bank, TC_FAST),
					  (kfree(fun_param)));
			break;

		case SAW_RAMPS_SLOW:
			CHECK_ERROR_KFREE(pmic_tcled_fun_sawramps
					  (fun_param->bank, TC_SLOW),
					  (kfree(fun_param)));
			break;

		case SAW_RAMPS_FAST:
			CHECK_ERROR_KFREE(pmic_tcled_fun_sawramps
					  (fun_param->bank, TC_FAST),
					  (kfree(fun_param)));
			break;

		case BLENDED_BOWTIE_SLOW:
			CHECK_ERROR_KFREE(pmic_tcled_fun_blendedbowtie
					  (fun_param->bank, TC_SLOW),
					  (kfree(fun_param)));
			break;

		case BLENDED_BOWTIE_FAST:
			CHECK_ERROR_KFREE(pmic_tcled_fun_blendedbowtie
					  (fun_param->bank, TC_FAST),
					  (kfree(fun_param)));
			break;

		case STROBE_SLOW:
			CHECK_ERROR_KFREE(pmic_tcled_fun_strobe
					  (fun_param->bank, fun_param->channel,
					   TC_STROBE_SLOW), (kfree(fun_param)));
			break;

		case STROBE_FAST:
			CHECK_ERROR_KFREE(pmic_tcled_fun_strobe
					  (fun_param->bank,
					   fun_param->channel, TC_STROBE_SLOW),
					  (kfree(fun_param)));
			break;

		case CHASING_LIGHT_RGB_SLOW:
			CHECK_ERROR_KFREE(pmic_tcled_fun_chasinglightspattern
					  (fun_param->bank, PMIC_RGB, TC_SLOW),
					  (kfree(fun_param)));
			break;

		case CHASING_LIGHT_RGB_FAST:
			CHECK_ERROR_KFREE(pmic_tcled_fun_chasinglightspattern
					  (fun_param->bank, PMIC_RGB, TC_FAST),
					  (kfree(fun_param)));
			break;

		case CHASING_LIGHT_BGR_SLOW:
			CHECK_ERROR_KFREE(pmic_tcled_fun_chasinglightspattern
					  (fun_param->bank, BGR, TC_SLOW),
					  (kfree(fun_param)));
			break;

		case CHASING_LIGHT_BGR_FAST:
			CHECK_ERROR_KFREE(pmic_tcled_fun_chasinglightspattern
					  (fun_param->bank, BGR, TC_FAST),
					  (kfree(fun_param)));
			break;
		}

		kfree(fun_param);
		break;

	case PMIC_SET_TCLED:
		if (tcled_ind == NULL)
			return -ENOMEM;

		if (copy_from_user(tcled_ind, (t_tcled_ind_param *) arg,
				   sizeof(t_tcled_ind_param))) {
			kfree(tcled_ind);
			return -EFAULT;
		}
		CHECK_ERROR_KFREE(pmic_tcled_ind_set_current(tcled_ind->channel,
							     tcled_ind->level,
							     tcled_ind->bank),
				  (kfree(tcled_ind)));
		CHECK_ERROR_KFREE(pmic_tcled_ind_set_blink_pattern
				  (tcled_ind->channel, tcled_ind->pattern,
				   tcled_ind->skip, tcled_ind->bank),
				  (kfree(tcled_ind)));
		CHECK_ERROR_KFREE(pmic_tcled_fun_rampup
				  (tcled_ind->bank, tcled_ind->channel,
				   tcled_ind->rampup), (kfree(tcled_ind)));
		CHECK_ERROR_KFREE(pmic_tcled_fun_rampdown
				  (tcled_ind->bank, tcled_ind->channel,
				   tcled_ind->rampdown), (kfree(tcled_ind)));
		if (tcled_ind->half_current) {
			CHECK_ERROR_KFREE(pmic_tcled_enable_half_current(),
					  (kfree(tcled_ind)));
		} else {
			CHECK_ERROR_KFREE(pmic_tcled_disable_half_current(),
					  (kfree(tcled_ind)));
		}

		kfree(tcled_ind);
		break;

	case PMIC_GET_TCLED:
		if (tcled_ind == NULL)
			return -ENOMEM;

		if (copy_from_user(tcled_ind, (t_tcled_ind_param *) arg,
				   sizeof(t_tcled_ind_param))) {
			kfree(tcled_ind);
			return -EFAULT;
		}
		CHECK_ERROR_KFREE(pmic_tcled_ind_get_current(tcled_ind->channel,
							     &tcled_ind->level,
							     tcled_ind->bank),
				  (kfree(tcled_ind)));
		CHECK_ERROR_KFREE(pmic_tcled_ind_get_blink_pattern
				  (tcled_ind->channel, &tcled_ind->pattern,
				   &tcled_ind->skip, tcled_ind->bank),
				  (kfree(tcled_ind)));
		CHECK_ERROR_KFREE(pmic_tcled_get_fun_rampup
				  (tcled_ind->bank, tcled_ind->channel,
				   &tcled_ind->rampup), (kfree(tcled_ind)));
		CHECK_ERROR_KFREE(pmic_tcled_get_fun_rampdown
				  (tcled_ind->bank, tcled_ind->channel,
				   &tcled_ind->rampdown), (kfree(tcled_ind)));
		if (copy_to_user
		    ((t_tcled_ind_param *) arg, tcled_ind,
		     sizeof(t_tcled_ind_param))) {
			return -EFAULT;
		}
		kfree(tcled_ind);

		break;

	default:
		return -EINVAL;
	}
	return 0;
}

/*!
 * This function initialize Light registers of mc13783 with 0.
 *
 * @return       This function returns 0 if successful.
 */
int pmic_light_init_reg(void)
{
	CHECK_ERROR(pmic_write_reg(LREG_0, 0, PMIC_ALL_BITS));
	CHECK_ERROR(pmic_write_reg(LREG_1, 0, PMIC_ALL_BITS));
	CHECK_ERROR(pmic_write_reg(LREG_2, 0, PMIC_ALL_BITS));
	CHECK_ERROR(pmic_write_reg(LREG_3, 0, PMIC_ALL_BITS));
	CHECK_ERROR(pmic_write_reg(LREG_4, 0, PMIC_ALL_BITS));
	CHECK_ERROR(pmic_write_reg(LREG_5, 0, PMIC_ALL_BITS));
	return 0;
}

/*!
 * This function implements the open method on a mc13783 light device.
 *
 * @param        inode       pointer on the node
 * @param        file        pointer on the file
 * @return       This function returns 0.
 */
static int pmic_light_open(struct inode *inode, struct file *file)
{
	while (suspend_flag == 1) {
		swait++;
		/* Block if the device is suspended */
		if (wait_event_interruptible(suspendq, (suspend_flag == 0))) {
			return -ERESTARTSYS;
		}
	}
	return 0;
}

/*!
 * This function implements the release method on a mc13783 light device.
 *
 * @param        inode       pointer on the node
 * @param        file        pointer on the file
 * @return       This function returns 0.
 */
static int pmic_light_release(struct inode *inode, struct file *file)
{
	while (suspend_flag == 1) {
		swait++;
		/* Block if the device is suspended */
		if (wait_event_interruptible(suspendq, (suspend_flag == 0))) {
			return -ERESTARTSYS;
		}
	}
	return 0;
}

static struct file_operations pmic_light_fops = {
	.owner = THIS_MODULE,
	.ioctl = pmic_light_ioctl,
	.open = pmic_light_open,
	.release = pmic_light_release,
};

static int pmic_light_remove(struct platform_device *pdev)
{
	device_destroy(pmic_light_class, MKDEV(pmic_light_major, 0));
	class_destroy(pmic_light_class);
	unregister_chrdev(pmic_light_major, "pmic_light");
	return 0;
}

static int pmic_light_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *temp_class;

	while (suspend_flag == 1) {
		swait++;
		/* Block if the device is suspended */
		if (wait_event_interruptible(suspendq, (suspend_flag == 0))) {
			return -ERESTARTSYS;
		}
	}
	pmic_light_major = register_chrdev(0, "pmic_light", &pmic_light_fops);

	if (pmic_light_major < 0) {
		printk(KERN_ERR "Unable to get a major for pmic_light\n");
		return pmic_light_major;
	}
	init_waitqueue_head(&suspendq);

	pmic_light_class = class_create(THIS_MODULE, "pmic_light");
	if (IS_ERR(pmic_light_class)) {
		printk(KERN_ERR "Error creating pmic_light class.\n");
		ret = PTR_ERR(pmic_light_class);
		goto err_out1;
	}

	temp_class = device_create(pmic_light_class, NULL,
				   MKDEV(pmic_light_major, 0), NULL,
				   "pmic_light");
	if (IS_ERR(temp_class)) {
		printk(KERN_ERR "Error creating pmic_light class device.\n");
		ret = PTR_ERR(temp_class);
		goto err_out2;
	}

	ret = pmic_light_init_reg();
	if (ret != PMIC_SUCCESS) {
		goto err_out3;
	}

	printk(KERN_INFO "PMIC Light successfully loaded\n");
	return ret;

      err_out3:
	device_destroy(pmic_light_class, MKDEV(pmic_light_major, 0));
      err_out2:
	class_destroy(pmic_light_class);
      err_out1:
	unregister_chrdev(pmic_light_major, "pmic_light");
	return ret;
}

static struct platform_driver pmic_light_driver_ldm = {
	.driver = {
		   .name = "pmic_light",
		   },
	.suspend = pmic_light_suspend,
	.resume = pmic_light_resume,
	.probe = pmic_light_probe,
	.remove = pmic_light_remove,
};

/*
 * Initialization and Exit
 */

static int __init pmic_light_init(void)
{
	pr_debug("PMIC Light driver loading...\n");
	return platform_driver_register(&pmic_light_driver_ldm);
}
static void __exit pmic_light_exit(void)
{
	platform_driver_unregister(&pmic_light_driver_ldm);
	pr_debug("PMIC Light driver successfully unloaded\n");
}

/*
 * Module entry points
 */

subsys_initcall(pmic_light_init);
module_exit(pmic_light_exit);

MODULE_DESCRIPTION("PMIC_LIGHT");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
