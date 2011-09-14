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
 * @file mc13783/pmic_battery.c
 * @brief This is the main file of PMIC(mc13783) Battery driver.
 *
 * @ingroup PMIC_BATTERY
 */

/*
 * Includes
 */
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/wait.h>

#include <linux/pmic_battery.h>
#include <linux/pmic_adc.h>
#include <linux/pmic_status.h>

#include "pmic_battery_defs.h"

#include <mach/pmic_power.h>
#ifdef CONFIG_MXC_HWEVENT
#include <mach/hw_events.h>
#endif

static int pmic_battery_major;

/*!
 * Number of users waiting in suspendq
 */
static int swait;

/*!
 * To indicate whether any of the battery devices are suspending
 */
static int suspend_flag;

/*!
 * The suspendq is used to block application calls
 */
static wait_queue_head_t suspendq;

static struct class *pmic_battery_class;

/* EXPORTED FUNCTIONS */
EXPORT_SYMBOL(pmic_batt_enable_charger);
EXPORT_SYMBOL(pmic_batt_disable_charger);
EXPORT_SYMBOL(pmic_batt_set_charger);
EXPORT_SYMBOL(pmic_batt_get_charger_setting);
EXPORT_SYMBOL(pmic_batt_get_charge_current);
EXPORT_SYMBOL(pmic_batt_enable_eol);
EXPORT_SYMBOL(pmic_batt_bp_enable_eol);
EXPORT_SYMBOL(pmic_batt_disable_eol);
EXPORT_SYMBOL(pmic_batt_set_out_control);
EXPORT_SYMBOL(pmic_batt_set_threshold);
EXPORT_SYMBOL(pmic_batt_led_control);
EXPORT_SYMBOL(pmic_batt_set_reverse_supply);
EXPORT_SYMBOL(pmic_batt_set_unregulated);
EXPORT_SYMBOL(pmic_batt_set_5k_pull);
EXPORT_SYMBOL(pmic_batt_event_subscribe);
EXPORT_SYMBOL(pmic_batt_event_unsubscribe);

static DECLARE_MUTEX(count_mutex);	/* open count mutex */
static int open_count;		/* open count for device file */

/*!
 * Callback function for events, we want on MGN board
 */
static void callback_chg_detect(void)
{
#ifdef CONFIG_MXC_HWEVENT
	t_sensor_bits sensor;
	struct mxc_hw_event event = { HWE_BAT_CHARGER_PLUG, 0 };

	pr_debug("In callback_chg_detect\n");

	/* get sensor values */
	pmic_get_sensors(&sensor);

	pr_debug("Callback, charger detect:%d\n", sensor.sense_chgdets);

	if (sensor.sense_chgdets)
		event.args = 1;
	else
		event.args = 0;
	/* send hardware event */
	hw_event_send(HWE_DEF_PRIORITY, &event);
#endif
}

static void callback_low_battery(void)
{
#ifdef CONFIG_MXC_HWEVENT
	struct mxc_hw_event event = { HWE_BAT_BATTERY_LOW, 0 };

	pr_debug("In callback_low_battery\n");
	/* send hardware event */
	hw_event_send(HWE_DEF_PRIORITY, &event);
#endif
}

static void callback_power_fail(void)
{
#ifdef CONFIG_MXC_HWEVENT
	struct mxc_hw_event event = { HWE_BAT_POWER_FAILED, 0 };

	pr_debug("In callback_power_fail\n");
	/* send hardware event */
	hw_event_send(HWE_DEF_PRIORITY, &event);
#endif
}

static void callback_chg_overvoltage(void)
{
#ifdef CONFIG_MXC_HWEVENT
	struct mxc_hw_event event = { HWE_BAT_CHARGER_OVERVOLTAGE, 0 };

	pr_debug("In callback_chg_overvoltage\n");
	/* send hardware event */
	hw_event_send(HWE_DEF_PRIORITY, &event);
#endif
}

static void callback_chg_full(void)
{
#ifdef CONFIG_MXC_HWEVENT
	t_sensor_bits sensor;
	struct mxc_hw_event event = { HWE_BAT_CHARGER_FULL, 0 };

	pr_debug("In callback_chg_full\n");

	/* disable charge function */
	pmic_batt_disable_charger(BATT_MAIN_CHGR);

	/* get charger sensor */
	pmic_get_sensors(&sensor);

	/* if did not detect the charger */
	if (sensor.sense_chgdets)
		return;
	/* send hardware event */
	hw_event_send(HWE_DEF_PRIORITY, &event);
#endif
}

/*!
 * This is the suspend of power management for the pmic battery API.
 * It suports SAVE and POWER_DOWN state.
 *
 * @param        pdev           the device
 * @param        state          the state
 *
 * @return       This function returns 0 if successful.
 */
static int pmic_battery_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	unsigned int reg_value = 0;

	suspend_flag = 1;
	CHECK_ERROR(pmic_write_reg(REG_CHARGER, reg_value, PMIC_ALL_BITS));

	return 0;
};

/*!
 * This is the resume of power management for the pmic battery API.
 * It suports RESTORE state.
 *
 * @param        pdev           the device
 *
 * @return       This function returns 0 if successful.
 */
static int pmic_battery_resume(struct platform_device *pdev)
{
	suspend_flag = 0;
	while (swait > 0) {
		swait--;
		wake_up_interruptible(&suspendq);
	}

	return 0;
};

/*!
 * This function is used to start charging a battery. For different charger,
 * different voltage and current range are supported. \n
 *
 *
 * @param      chgr        Charger as defined in \b t_batt_charger.
 * @param      c_voltage   Charging voltage.
 * @param      c_current   Charging current.
 *
 * @return     This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_batt_enable_charger(t_batt_charger chgr,
				     unsigned char c_voltage,
				     unsigned char c_current)
{
	unsigned int val, mask, reg;

	val = 0;
	mask = 0;
	reg = 0;

	if (suspend_flag == 1)
		return PMIC_ERROR;

	switch (chgr) {
	case BATT_MAIN_CHGR:
		val = BITFVAL(MC13783_BATT_DAC_DAC, c_current) |
		    BITFVAL(MC13783_BATT_DAC_V_DAC, c_voltage);
		mask = BITFMASK(MC13783_BATT_DAC_DAC) |
		    BITFMASK(MC13783_BATT_DAC_V_DAC);
		reg = REG_CHARGER;
		break;

	case BATT_CELL_CHGR:
		val = BITFVAL(MC13783_BATT_DAC_V_COIN, c_voltage) |
		    BITFVAL(MC13783_BATT_DAC_COIN_CH_EN,
			    MC13783_BATT_DAC_COIN_CH_EN_ENABLED);
		mask = BITFMASK(MC13783_BATT_DAC_V_COIN) |
		    BITFMASK(MC13783_BATT_DAC_COIN_CH_EN);
		reg = REG_POWER_CONTROL_0;
		break;

	case BATT_TRCKLE_CHGR:
		val = BITFVAL(MC13783_BATT_DAC_TRCKLE, c_current);
		mask = BITFMASK(MC13783_BATT_DAC_TRCKLE);
		reg = REG_CHARGER;
		break;

	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(reg, val, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function turns off a charger.
 *
 * @param      chgr        Charger as defined in \b t_batt_charger.
 *
 * @return     This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_batt_disable_charger(t_batt_charger chgr)
{
	unsigned int val, mask, reg;

	val = 0;
	mask = 0;
	reg = 0;

	if (suspend_flag == 1)
		return PMIC_ERROR;
	switch (chgr) {
	case BATT_MAIN_CHGR:
		val = BITFVAL(MC13783_BATT_DAC_DAC, 0) |
		    BITFVAL(MC13783_BATT_DAC_V_DAC, 0);
		mask = BITFMASK(MC13783_BATT_DAC_DAC) |
		    BITFMASK(MC13783_BATT_DAC_V_DAC);
		reg = REG_CHARGER;
		break;

	case BATT_CELL_CHGR:
		val = BITFVAL(MC13783_BATT_DAC_COIN_CH_EN,
			      MC13783_BATT_DAC_COIN_CH_EN_DISABLED);
		mask = BITFMASK(MC13783_BATT_DAC_COIN_CH_EN);
		reg = REG_POWER_CONTROL_0;
		break;

	case BATT_TRCKLE_CHGR:
		val = BITFVAL(MC13783_BATT_DAC_TRCKLE, 0);
		mask = BITFMASK(MC13783_BATT_DAC_TRCKLE);
		reg = REG_CHARGER;
		break;

	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(reg, val, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function is used to change the charger setting.
 *
 * @param      chgr        Charger as defined in \b t_batt_charger.
 * @param      c_voltage   Charging voltage.
 * @param      c_current   Charging current.
 *
 * @return     This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_batt_set_charger(t_batt_charger chgr,
				  unsigned char c_voltage,
				  unsigned char c_current)
{
	unsigned int val, mask, reg;

	val = 0;
	mask = 0;
	reg = 0;

	if (suspend_flag == 1)
		return PMIC_ERROR;

	switch (chgr) {
	case BATT_MAIN_CHGR:
		val = BITFVAL(MC13783_BATT_DAC_DAC, c_current) |
		    BITFVAL(MC13783_BATT_DAC_V_DAC, c_voltage);
		mask = BITFMASK(MC13783_BATT_DAC_DAC) |
		    BITFMASK(MC13783_BATT_DAC_V_DAC);
		reg = REG_CHARGER;
		break;

	case BATT_CELL_CHGR:
		val = BITFVAL(MC13783_BATT_DAC_V_COIN, c_voltage);
		mask = BITFMASK(MC13783_BATT_DAC_V_COIN);
		reg = REG_POWER_CONTROL_0;
		break;

	case BATT_TRCKLE_CHGR:
		val = BITFVAL(MC13783_BATT_DAC_TRCKLE, c_current);
		mask = BITFMASK(MC13783_BATT_DAC_TRCKLE);
		reg = REG_CHARGER;
		break;

	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(reg, val, mask));
	return PMIC_SUCCESS;
}

/*!
 * This function is used to retrive the charger setting.
 *
 * @param      chgr        Charger as defined in \b t_batt_charger.
 * @param      c_voltage   Output parameter for charging voltage setting.
 * @param      c_current   Output parameter for charging current setting.
 *
 * @return     This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_batt_get_charger_setting(t_batt_charger chgr,
					  unsigned char *c_voltage,
					  unsigned char *c_current)
{
	unsigned int val, reg;

	reg = 0;

	if (suspend_flag == 1)
		return PMIC_ERROR;

	switch (chgr) {
	case BATT_MAIN_CHGR:
	case BATT_TRCKLE_CHGR:
		reg = REG_CHARGER;
		break;
	case BATT_CELL_CHGR:
		reg = REG_POWER_CONTROL_0;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_read_reg(reg, &val, PMIC_ALL_BITS));

	switch (chgr) {
	case BATT_MAIN_CHGR:
		*c_voltage = BITFEXT(val, MC13783_BATT_DAC_V_DAC);;
		*c_current = BITFEXT(val, MC13783_BATT_DAC_DAC);
		break;

	case BATT_CELL_CHGR:
		*c_voltage = BITFEXT(val, MC13783_BATT_DAC_V_COIN);
		*c_current = 0;
		break;

	case BATT_TRCKLE_CHGR:
		*c_voltage = 0;
		*c_current = BITFEXT(val, MC13783_BATT_DAC_TRCKLE);
		break;

	default:
		return PMIC_PARAMETER_ERROR;
	}

	return PMIC_SUCCESS;
}

/*!
 * This function is retrives the main battery voltage.
 *
 * @param      b_voltage   Output parameter for voltage setting.
 *
 * @return     This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_batt_get_batt_voltage(unsigned short *b_voltage)
{
	t_channel channel;
	unsigned short result[8];

	if (suspend_flag == 1)
		return PMIC_ERROR;
	channel = BATTERY_VOLTAGE;
	CHECK_ERROR(pmic_adc_convert(channel, result));
	*b_voltage = result[0];

	return PMIC_SUCCESS;
}

/*!
 * This function is retrives the main battery current.
 *
 * @param      b_current   Output parameter for current setting.
 *
 * @return     This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_batt_get_batt_current(unsigned short *b_current)
{
	t_channel channel;
	unsigned short result[8];

	if (suspend_flag == 1)
		return PMIC_ERROR;

	channel = BATTERY_CURRENT;
	CHECK_ERROR(pmic_adc_convert(channel, result));
	*b_current = result[0];

	return PMIC_SUCCESS;
}

/*!
 * This function is retrives the main battery temperature.
 *
 * @param      b_temper   Output parameter for temperature setting.
 *
 * @return     This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_batt_get_batt_temperature(unsigned short *b_temper)
{
	t_channel channel;
	unsigned short result[8];

	if (suspend_flag == 1)
		return PMIC_ERROR;

	channel = GEN_PURPOSE_AD5;
	CHECK_ERROR(pmic_adc_convert(channel, result));
	*b_temper = result[0];

	return PMIC_SUCCESS;
}

/*!
 * This function is retrives the main battery charging voltage.
 *
 * @param      c_voltage   Output parameter for charging voltage setting.
 *
 * @return     This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_batt_get_charge_voltage(unsigned short *c_voltage)
{
	t_channel channel;
	unsigned short result[8];

	if (suspend_flag == 1)
		return PMIC_ERROR;

	channel = CHARGE_VOLTAGE;
	CHECK_ERROR(pmic_adc_convert(channel, result));
	*c_voltage = result[0];

	return PMIC_SUCCESS;
}

/*!
 * This function is retrives the main battery charging current.
 *
 * @param      c_current   Output parameter for charging current setting.
 *
 * @return     This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_batt_get_charge_current(unsigned short *c_current)
{
	t_channel channel;
	unsigned short result[8];

	if (suspend_flag == 1)
		return PMIC_ERROR;

	channel = CHARGE_CURRENT;
	CHECK_ERROR(pmic_adc_convert(channel, result));
	*c_current = result[0];

	return PMIC_SUCCESS;
}

/*!
 * This function enables End-of-Life comparator. Not supported on
 * mc13783. Use pmic_batt_bp_enable_eol function.
 *
 * @param      threshold  End-of-Life threshold.
 *
 * @return     This function returns PMIC_UNSUPPORTED
 */
PMIC_STATUS pmic_batt_enable_eol(unsigned char threshold)
{
	return PMIC_NOT_SUPPORTED;
}

/*!
 * This function enables End-of-Life comparator.
 *
 * @param      typical  Falling Edge Threshold threshold.
 * 			@verbatim
			BPDET	UVDET	LOBATL
			____	_____   ___________
			0	2.6	UVDET + 0.2
			1	2.6	UVDET + 0.3
			2	2.6	UVDET + 0.4
			3	2.6	UVDET + 0.5
 *
 * @return     This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_batt_bp_enable_eol(t_bp_threshold typical)
{
	unsigned int val, mask;

	if (suspend_flag == 1)
		return PMIC_ERROR;

	val = BITFVAL(MC13783_BATT_DAC_EOL_CMP_EN,
		      MC13783_BATT_DAC_EOL_CMP_EN_ENABLE) |
	    BITFVAL(MC13783_BATT_DAC_EOL_SEL, typical);
	mask = BITFMASK(MC13783_BATT_DAC_EOL_CMP_EN) |
	    BITFMASK(MC13783_BATT_DAC_EOL_SEL);

	CHECK_ERROR(pmic_write_reg(REG_POWER_CONTROL_0, val, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function disables End-of-Life comparator.
 *
 * @return     This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_batt_disable_eol(void)
{
	unsigned int val, mask;

	if (suspend_flag == 1)
		return PMIC_ERROR;

	val = BITFVAL(MC13783_BATT_DAC_EOL_CMP_EN,
		      MC13783_BATT_DAC_EOL_CMP_EN_DISABLE);
	mask = BITFMASK(MC13783_BATT_DAC_EOL_CMP_EN);

	CHECK_ERROR(pmic_write_reg(REG_POWER_CONTROL_0, val, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function sets the output controls.
 * It sets the FETOVRD and FETCTRL bits of mc13783
 *
 * @param        control        type of control.
 *
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS pmic_batt_set_out_control(t_control control)
{
	unsigned int val, mask;
	if (suspend_flag == 1)
		return PMIC_ERROR;

	switch (control) {
	case CONTROL_HARDWARE:
		val = BITFVAL(MC13783_BATT_DAC_FETOVRD_EN, 0) |
		    BITFVAL(MC13783_BATT_DAC_FETCTRL_EN, 0);
		mask = BITFMASK(MC13783_BATT_DAC_FETOVRD_EN) |
		    BITFMASK(MC13783_BATT_DAC_FETCTRL_EN);
		break;
	case CONTROL_BPFET_LOW:
		val = BITFVAL(MC13783_BATT_DAC_FETOVRD_EN, 1) |
		    BITFVAL(MC13783_BATT_DAC_FETCTRL_EN, 0);
		mask = BITFMASK(MC13783_BATT_DAC_FETOVRD_EN) |
		    BITFMASK(MC13783_BATT_DAC_FETCTRL_EN);
		break;
	case CONTROL_BPFET_HIGH:
		val = BITFVAL(MC13783_BATT_DAC_FETOVRD_EN, 1) |
		    BITFVAL(MC13783_BATT_DAC_FETCTRL_EN, 1);
		mask = BITFMASK(MC13783_BATT_DAC_FETOVRD_EN) |
		    BITFMASK(MC13783_BATT_DAC_FETCTRL_EN);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(REG_CHARGER, val, mask));
	return PMIC_SUCCESS;
}

/*!
 * This function sets over voltage threshold.
 *
 * @param        threshold      value of over voltage threshold.
 *
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS pmic_batt_set_threshold(int threshold)
{
	unsigned int val, mask;

	if (suspend_flag == 1)
		return PMIC_ERROR;

	if (threshold > BAT_THRESHOLD_MAX)
		return PMIC_PARAMETER_ERROR;

	val = BITFVAL(MC13783_BATT_DAC_OVCTRL, threshold);
	mask = BITFMASK(MC13783_BATT_DAC_OVCTRL);
	CHECK_ERROR(pmic_write_reg(REG_CHARGER, val, mask));
	return PMIC_SUCCESS;
}

/*!
 * This function controls charge LED.
 *
 * @param      on   If on is ture, LED will be turned on,
 *                  or otherwise, LED will be turned off.
 *
 * @return     This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_batt_led_control(bool on)
{
	unsigned val, mask;

	if (suspend_flag == 1)
		return PMIC_ERROR;

	val = BITFVAL(MC13783_BATT_DAC_LED_EN, on);
	mask = BITFMASK(MC13783_BATT_DAC_LED_EN);

	CHECK_ERROR(pmic_write_reg(REG_CHARGER, val, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function sets reverse supply mode.
 *
 * @param      enable     If enable is ture, reverse supply mode is enable,
 *                        or otherwise, reverse supply mode is disabled.
 *
 * @return     This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_batt_set_reverse_supply(bool enable)
{
	unsigned val, mask;

	if (suspend_flag == 1)
		return PMIC_ERROR;

	val = BITFVAL(MC13783_BATT_DAC_REVERSE_SUPPLY, enable);
	mask = BITFMASK(MC13783_BATT_DAC_REVERSE_SUPPLY);

	CHECK_ERROR(pmic_write_reg(REG_CHARGER, val, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function sets unregulatored charging mode on main battery.
 *
 * @param      enable     If enable is ture, unregulated charging mode is
 *                        enable, or otherwise, disabled.
 *
 * @return     This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_batt_set_unregulated(bool enable)
{
	unsigned val, mask;

	if (suspend_flag == 1)
		return PMIC_ERROR;

	val = BITFVAL(MC13783_BATT_DAC_UNREGULATED, enable);
	mask = BITFMASK(MC13783_BATT_DAC_UNREGULATED);

	CHECK_ERROR(pmic_write_reg(REG_CHARGER, val, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function sets a 5K pull down at CHRGRAW.
 * To be used in the dual path charging configuration.
 *
 * @param      enable     If enable is true, 5k pull down is
 *                        enable, or otherwise, disabled.
 *
 * @return     This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_batt_set_5k_pull(bool enable)
{
	unsigned val, mask;

	if (suspend_flag == 1)
		return PMIC_ERROR;

	val = BITFVAL(MC13783_BATT_DAC_5K, enable);
	mask = BITFMASK(MC13783_BATT_DAC_5K);

	CHECK_ERROR(pmic_write_reg(REG_CHARGER, val, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function is used to un/subscribe on battery event IT.
 *
 * @param        event          type of event.
 * @param        callback       event callback function.
 * @param        sub            define if Un/subscribe event.
 *
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS mc13783_battery_event(t_batt_event event, void *callback, bool sub)
{
	pmic_event_callback_t bat_callback;
	type_event bat_event;

	bat_callback.func = callback;
	bat_callback.param = NULL;
	switch (event) {
	case BAT_IT_CHG_DET:
		bat_event = EVENT_CHGDETI;
		break;
	case BAT_IT_CHG_OVERVOLT:
		bat_event = EVENT_CHGOVI;
		break;
	case BAT_IT_CHG_REVERSE:
		bat_event = EVENT_CHGREVI;
		break;
	case BAT_IT_CHG_SHORT_CIRCUIT:
		bat_event = EVENT_CHGSHORTI;
		break;
	case BAT_IT_CCCV:
		bat_event = EVENT_CCCVI;
		break;
	case BAT_IT_BELOW_THRESHOLD:
		bat_event = EVENT_CHRGCURRI;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}
	if (sub == true) {
		CHECK_ERROR(pmic_event_subscribe(bat_event, bat_callback));
	} else {
		CHECK_ERROR(pmic_event_unsubscribe(bat_event, bat_callback));
	}
	return 0;
}

/*!
 * This function is used to subscribe on battery event IT.
 *
 * @param        event          type of event.
 * @param        callback       event callback function.
 *
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS pmic_batt_event_subscribe(t_batt_event event, void *callback)
{
	if (suspend_flag == 1)
		return PMIC_ERROR;

	return mc13783_battery_event(event, callback, true);
}

/*!
 * This function is used to un subscribe on battery event IT.
 *
 * @param        event          type of event.
 * @param        callback       event callback function.
 *
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS pmic_batt_event_unsubscribe(t_batt_event event, void *callback)
{
	if (suspend_flag == 1)
		return PMIC_ERROR;

	return mc13783_battery_event(event, callback, false);
}

/*!
 * This function implements IOCTL controls on a PMIC Battery device.
 *
 * @param        inode       pointer on the node
 * @param        file        pointer on the file
 * @param        cmd         the command
 * @param        arg         the parameter
 * @return       This function returns 0 if successful.
 */
static int pmic_battery_ioctl(struct inode *inode, struct file *file,
			      unsigned int cmd, unsigned long arg)
{
	t_charger_setting *chgr_setting = NULL;
	unsigned short c_current;
	unsigned int bc_info;
	t_eol_setting *eol_setting;

	if (_IOC_TYPE(cmd) != 'p')
		return -ENOTTY;

	chgr_setting = kmalloc(sizeof(t_charger_setting), GFP_KERNEL);
	eol_setting = kmalloc(sizeof(t_eol_setting), GFP_KERNEL);
	switch (cmd) {
	case PMIC_BATT_CHARGER_CONTROL:
		if (chgr_setting == NULL)
			return -ENOMEM;

		if (copy_from_user(chgr_setting, (t_charger_setting *) arg,
				   sizeof(t_charger_setting))) {
			kfree(chgr_setting);
			return -EFAULT;
		}

		if (chgr_setting->on != false) {
			CHECK_ERROR_KFREE(pmic_batt_enable_charger
					  (chgr_setting->chgr,
					   chgr_setting->c_voltage,
					   chgr_setting->c_current),
					  (kfree(chgr_setting)));
		} else {
			CHECK_ERROR(pmic_batt_disable_charger
				    (chgr_setting->chgr));
		}

		kfree(chgr_setting);
		break;

	case PMIC_BATT_SET_CHARGER:
		if (chgr_setting == NULL)
			return -ENOMEM;

		if (copy_from_user(chgr_setting, (t_charger_setting *) arg,
				   sizeof(t_charger_setting))) {
			kfree(chgr_setting);
			return -EFAULT;
		}

		CHECK_ERROR_KFREE(pmic_batt_set_charger(chgr_setting->chgr,
							chgr_setting->c_voltage,
							chgr_setting->
							c_current),
				  (kfree(chgr_setting)));

		kfree(chgr_setting);
		break;

	case PMIC_BATT_GET_CHARGER:
		if (chgr_setting == NULL)
			return -ENOMEM;

		if (copy_from_user(chgr_setting, (t_charger_setting *) arg,
				   sizeof(t_charger_setting))) {
			kfree(chgr_setting);
			return -EFAULT;
		}

		CHECK_ERROR_KFREE(pmic_batt_get_charger_setting
				  (chgr_setting->chgr, &chgr_setting->c_voltage,
				   &chgr_setting->c_current),
				  (kfree(chgr_setting)));
		if (copy_to_user
		    ((t_charger_setting *) arg, chgr_setting,
		     sizeof(t_charger_setting))) {
			return -EFAULT;
		}

		kfree(chgr_setting);
		break;

	case PMIC_BATT_GET_CHARGER_SENSOR:
		{
			t_sensor_bits sensor;
			pmic_get_sensors(&sensor);
			if (copy_to_user
			    ((unsigned int *)arg, &sensor.sense_chgdets,
			     sizeof(unsigned int)))
				return -EFAULT;

			break;
		}
	case PMIC_BATT_GET_BATTERY_VOLTAGE:
		CHECK_ERROR(pmic_batt_get_batt_voltage(&c_current));
		bc_info = (unsigned int)c_current * 2300 / 1023 + 2400;
		if (copy_to_user((unsigned int *)arg, &bc_info,
				 sizeof(unsigned int)))
			return -EFAULT;

		break;

	case PMIC_BATT_GET_BATTERY_CURRENT:
		CHECK_ERROR(pmic_batt_get_batt_current(&c_current));
		bc_info = (unsigned int)c_current * 5750 / 1023;
		if (copy_to_user((unsigned int *)arg, &bc_info,
				 sizeof(unsigned int)))
			return -EFAULT;
		break;

	case PMIC_BATT_GET_BATTERY_TEMPERATURE:
		CHECK_ERROR(pmic_batt_get_batt_temperature(&c_current));
		bc_info = (unsigned int)c_current;
		if (copy_to_user((unsigned int *)arg, &bc_info,
				 sizeof(unsigned int)))
			return -EFAULT;

		break;

	case PMIC_BATT_GET_CHARGER_VOLTAGE:
		CHECK_ERROR(pmic_batt_get_charge_voltage(&c_current));
		bc_info = (unsigned int)c_current * 23000 / 1023;
		if (copy_to_user((unsigned int *)arg, &bc_info,
				 sizeof(unsigned int)))
			return -EFAULT;

		break;

	case PMIC_BATT_GET_CHARGER_CURRENT:
		CHECK_ERROR(pmic_batt_get_charge_current(&c_current));
		bc_info = (unsigned int)c_current * 5750 / 1023;
		if (copy_to_user((unsigned int *)arg, &bc_info,
				 sizeof(unsigned int)))
			return -EFAULT;

		break;

	case PMIC_BATT_EOL_CONTROL:
		if (eol_setting == NULL)
			return -ENOMEM;

		if (copy_from_user(eol_setting, (t_eol_setting *) arg,
				   sizeof(t_eol_setting))) {
			kfree(eol_setting);
			return -EFAULT;
		}

		if (eol_setting->enable != false) {
			CHECK_ERROR_KFREE(pmic_batt_bp_enable_eol
					  (eol_setting->typical),
					  (kfree(chgr_setting)));
		} else {
			CHECK_ERROR_KFREE(pmic_batt_disable_eol(),
					  (kfree(chgr_setting)));
		}

		kfree(eol_setting);
		break;

	case PMIC_BATT_SET_OUT_CONTROL:
		CHECK_ERROR(pmic_batt_set_out_control((t_control) arg));
		break;

	case PMIC_BATT_SET_THRESHOLD:
		CHECK_ERROR(pmic_batt_set_threshold((int)arg));
		break;

	case PMIC_BATT_LED_CONTROL:
		CHECK_ERROR(pmic_batt_led_control((bool) arg));
		break;

	case PMIC_BATT_REV_SUPP_CONTROL:
		CHECK_ERROR(pmic_batt_set_reverse_supply((bool) arg));
		break;

	case PMIC_BATT_UNREG_CONTROL:
		CHECK_ERROR(pmic_batt_set_unregulated((bool) arg));
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

/*!
 * This function implements the open method on a Pmic battery device.
 *
 * @param        inode       pointer on the node
 * @param        file        pointer on the file
 * @return       This function returns 0.
 */
static int pmic_battery_open(struct inode *inode, struct file *file)
{
	while (suspend_flag == 1) {
		swait++;
		/* Block if the device is suspended */
		if (wait_event_interruptible(suspendq, (suspend_flag == 0))) {
			return -ERESTARTSYS;
		}
	}

	/* check open count, if open firstly, register callbacks */
	down(&count_mutex);
	if (open_count++ > 0) {
		up(&count_mutex);
		return 0;
	}

	pr_debug("Subscribe the callbacks\n");
	/* register battery event callback */
	if (pmic_batt_event_subscribe(BAT_IT_CHG_DET, callback_chg_detect)) {
		pr_debug("Failed to subscribe the charger detect callback\n");
		goto event_err1;
	}
	if (pmic_power_event_sub(PWR_IT_LOBATLI, callback_power_fail)) {
		pr_debug("Failed to subscribe the power failed callback\n");
		goto event_err2;
	}
	if (pmic_power_event_sub(PWR_IT_LOBATHI, callback_low_battery)) {
		pr_debug("Failed to subscribe the low battery callback\n");
		goto event_err3;
	}
	if (pmic_batt_event_subscribe
	    (BAT_IT_CHG_OVERVOLT, callback_chg_overvoltage)) {
		pr_debug("Failed to subscribe the low battery callback\n");
		goto event_err4;
	}
	if (pmic_batt_event_subscribe
	    (BAT_IT_BELOW_THRESHOLD, callback_chg_full)) {
		pr_debug("Failed to subscribe the charge full callback\n");
		goto event_err5;
	}

	up(&count_mutex);

	return 0;

	/* un-subscribe the event callbacks */
event_err5:
	pmic_batt_event_unsubscribe(BAT_IT_CHG_OVERVOLT,
				    callback_chg_overvoltage);
event_err4:
	pmic_power_event_unsub(PWR_IT_LOBATHI, callback_low_battery);
event_err3:
	pmic_power_event_unsub(PWR_IT_LOBATLI, callback_power_fail);
event_err2:
	pmic_batt_event_unsubscribe(BAT_IT_CHG_DET, callback_chg_detect);
event_err1:

	open_count--;
	up(&count_mutex);

	return -EFAULT;

}

/*!
 * This function implements the release method on a Pmic battery device.
 *
 * @param        inode       pointer on the node
 * @param        file        pointer on the file
 * @return       This function returns 0.
 */
static int pmic_battery_release(struct inode *inode, struct file *file)
{
	while (suspend_flag == 1) {
		swait++;
		/* Block if the device is suspended */
		if (wait_event_interruptible(suspendq, (suspend_flag == 0))) {
			return -ERESTARTSYS;
		}
	}

	/* check open count, if open firstly, register callbacks */
	down(&count_mutex);
	if (--open_count == 0) {
		/* unregister these event callback */
		pr_debug("Unsubscribe the callbacks\n");
		pmic_batt_event_unsubscribe(BAT_IT_BELOW_THRESHOLD,
					    callback_chg_full);
		pmic_batt_event_unsubscribe(BAT_IT_CHG_OVERVOLT,
					    callback_chg_overvoltage);
		pmic_power_event_unsub(PWR_IT_LOBATHI, callback_low_battery);
		pmic_power_event_unsub(PWR_IT_LOBATLI, callback_power_fail);
		pmic_batt_event_unsubscribe(BAT_IT_CHG_DET,
					    callback_chg_detect);
	}
	up(&count_mutex);

	return 0;
}

static struct file_operations pmic_battery_fops = {
	.owner = THIS_MODULE,
	.ioctl = pmic_battery_ioctl,
	.open = pmic_battery_open,
	.release = pmic_battery_release,
};

static int pmic_battery_remove(struct platform_device *pdev)
{
	device_destroy(pmic_battery_class, MKDEV(pmic_battery_major, 0));
	class_destroy(pmic_battery_class);
	unregister_chrdev(pmic_battery_major, PMIC_BATTERY_STRING);
	return 0;
}

static int pmic_battery_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *temp_class;

	pmic_battery_major = register_chrdev(0, PMIC_BATTERY_STRING,
					     &pmic_battery_fops);

	if (pmic_battery_major < 0) {
		printk(KERN_ERR "Unable to get a major for pmic_battery\n");
		return pmic_battery_major;
	}
	init_waitqueue_head(&suspendq);

	pmic_battery_class = class_create(THIS_MODULE, PMIC_BATTERY_STRING);
	if (IS_ERR(pmic_battery_class)) {
		printk(KERN_ERR "Error creating PMIC battery class.\n");
		ret = PTR_ERR(pmic_battery_class);
		goto err_out1;
	}

	temp_class = device_create(pmic_battery_class, NULL,
				   MKDEV(pmic_battery_major, 0), NULL,
				   PMIC_BATTERY_STRING);
	if (IS_ERR(temp_class)) {
		printk(KERN_ERR "Error creating PMIC battery class device.\n");
		ret = PTR_ERR(temp_class);
		goto err_out2;
	}

	pmic_batt_led_control(true);
	pmic_batt_set_5k_pull(true);

	printk(KERN_INFO "PMIC Battery successfully probed\n");

	return ret;

      err_out2:
	class_destroy(pmic_battery_class);
      err_out1:
	unregister_chrdev(pmic_battery_major, PMIC_BATTERY_STRING);
	return ret;
}

static struct platform_driver pmic_battery_driver_ldm = {
	.driver = {
		   .name = "pmic_battery",
		   .bus = &platform_bus_type,
		   },
	.suspend = pmic_battery_suspend,
	.resume = pmic_battery_resume,
	.probe = pmic_battery_probe,
	.remove = pmic_battery_remove,
};

/*
 * Init and Exit
 */

static int __init pmic_battery_init(void)
{
	pr_debug("PMIC Battery driver loading...\n");
	return platform_driver_register(&pmic_battery_driver_ldm);
}

static void __exit pmic_battery_exit(void)
{
	platform_driver_unregister(&pmic_battery_driver_ldm);
	pr_debug("PMIC Battery driver successfully unloaded\n");
}

/*
 * Module entry points
 */

module_init(pmic_battery_init);
module_exit(pmic_battery_exit);

MODULE_DESCRIPTION("pmic_battery driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
