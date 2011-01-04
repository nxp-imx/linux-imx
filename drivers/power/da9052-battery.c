/*
 * da9052-battery.c  --  Batttery Driver for Dialog DA9052
 *
 * Copyright(c) 2009 Dialog Semiconductor Ltd.
 *
 * Author: Dialog Semiconductor Ltd <dchen@diasemi.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/uaccess.h>
#include <linux/jiffies.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/freezer.h>

#include <linux/mfd/da9052/da9052.h>
#include <linux/mfd/da9052/reg.h>
#include <linux/mfd/da9052/bat.h>
#include <linux/mfd/da9052/adc.h>

#define DA9052_BAT_DEVICE_NAME			"da9052-bat"

static const char  __initdata banner[] = KERN_INFO "DA9052 BAT, (c) \
2009 Dialog semiconductor Ltd.\n";

static struct da9052_bat_device bat_info;
static struct da9052_bat_status bat_status;
static struct da9052_bat_hysteresis bat_hysteresis;
static struct da9052_bat_event_registration event_status;
static struct monitoring_state monitoring_status;
struct power_supply_info battery_info;

struct da9052_charger_device charger;

static u16 bat_target_voltage;
u8 tbat_event_occur;
static int da9052_bat_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val);

s32 monitoring_thread_pid;
u8 monitoring_thread_state = ACTIVE;
struct completion monitoring_thread_notifier;

static u16 array_hys_batvoltage[2];
static u16 bat_volt_arr[3];
static u8 hys_flag = FALSE;

static enum power_supply_property da902_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
#if 0
	/* Not supported in 2.6.28 kernel */
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
#endif
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static int da9052_read(struct da9052 *da9052, u8 reg_address, u8 *reg_data)
{
	struct da9052_ssc_msg msg;
	int ret;

	msg.addr = reg_address;
	msg.data = 0;

	da9052_lock(da9052);
	ret = da9052->read(da9052, &msg);
	if (ret)
		goto ssc_comm_err;
	da9052_unlock(da9052);

	*reg_data = msg.data;
	return 0;
ssc_comm_err:
	da9052_unlock(da9052);
	return ret;
}

static int da9052_write(struct da9052 *da9052, u8 reg_address, u8 reg_data)
{
	struct da9052_ssc_msg msg;
	int ret;

	msg.addr = reg_address;
	msg.data = reg_data;

	da9052_lock(da9052);
	ret = da9052->write(da9052, &msg);
	if (ret)
		goto ssc_comm_err;
	da9052_unlock(da9052);

	return 0;
ssc_comm_err:
	da9052_unlock(da9052);
	return ret;
}


static s32 da9052_adc_read_ich(struct da9052 *da9052, u16 *data)
{
	struct da9052_ssc_msg msg;
	da9052_lock(da9052);
	/* Read charging conversion register */
	msg.addr = DA9052_ICHGAV_REG;
	msg.data = 0;
	if (da9052->read(da9052, &msg)) {
		da9052_unlock(da9052);
		return DA9052_SSC_FAIL;
	}
	da9052_unlock(da9052);

	*data = (u16)msg.data;
	DA9052_DEBUG(
       "In function: %s, ICHGAV_REG value read (1)= 0x%X \n",
		__func__, msg.data);
	return SUCCESS;
}

static s32 da9052_adc_read_vddout(struct da9052 *da9052, u16 *data)
{
	u8 reg_data;
	s32 ret;

	ret = da9052_read(da9052, DA9052_ADCCONT_REG, &reg_data);
	if (ret)
		return ret;

	if (!(reg_data & DA9052_ADCCONT_AUTOVDDEN)) {
		reg_data = (reg_data | DA9052_ADCCONT_AUTOVDDEN);

		ret = da9052_write(da9052, DA9052_INPUTCONT_REG, reg_data);
		if (ret)
			return ret;
		reg_data = 0x0;

		ret = da9052_read(da9052, DA9052_ADCCONT_REG, &reg_data);
		if (ret)
			return ret;

		if (reg_data & DA9052_ADCCONT_ADCMODE)
			msleep(1);
		else
			msleep(10);

		ret = da9052_read(da9052, DA9052_VDDRES_REG, &reg_data);
		if (ret)
			return ret;

		*data = (u16)reg_data;

		ret = da9052_read(da9052, DA9052_ADCCONT_REG, &reg_data);
		if (ret)
			return ret;

		reg_data = reg_data & ~(DA9052_ADCCONT_AUTOVDDEN);
		ret = da9052_write(da9052, DA9052_ADCCONT_REG, reg_data);
		if (ret)
			return ret;
	} else {
		ret = da9052_read(da9052, DA9052_VDDRES_REG, &reg_data);
		if (ret)
			return ret;

		*data = (u16)reg_data;
	}
	return 0;
}


static s32 da9052_adc_read_tbat(struct da9052 *da9052, u16 *data)
{
	s32 ret;
	u8 reg_data;

	ret = da9052_read(da9052, DA9052_TBATRES_REG, &reg_data);
	if (ret)
		return ret;
	*data = (u16)reg_data;

	/*printk(KERN_INFO "BAT_LOG:\t Bat Temperature in adc_read_tbat=
	%d\n", reg_data);*/

	DA9052_DEBUG("In function: %s, TBATRES_REG value read (1)= 0x%X \n",
		__func__, msg.data);
	return SUCCESS;
}

s32 da9052_adc_read_tjunc(struct da9052 *da9052, u16 *data)
{
	struct da9052_ssc_msg msg;
	u16 temp;

	da9052_lock(da9052);

	/* Read TJunction conversion result */
	msg.addr = DA9052_TJUNCRES_REG;
	if (da9052->read(da9052, &msg)) {
		da9052_unlock(da9052);
		return DA9052_SSC_FAIL;
	}
	temp = msg.data;
	/* Read calibration for junction temperature */
	msg.addr = DA9052_TOFFSET_REG;
	if (da9052->read(da9052, &msg)) {
		da9052_unlock(da9052);
		return DA9052_SSC_FAIL;
	}

	da9052_unlock(da9052);
	/* Calculate Junction temperature */
	*data = temp - (u16)msg.data;

	DA9052_DEBUG("In function: %s, Calc JUNC TEMP value (1)= 0x%X \n",
		__func__, *data);
	return 0;
}

s32 da9052_adc_read_vbat(struct da9052 *da9052, u16 *data)
{
	s32 ret;

	ret = da9052_manual_read(da9052, DA9052_ADC_VBAT);
	DA9052_DEBUG("In function: %s, VBAT value read (1)= 0x%X \n",
		__func__, temp);
	if (ret == -EIO) {
		*data = 0;
		return ret;
	} else {
		*data = ret;
		return 0;
	}
	return 0;
}


static u16 filter_sample(u16 *buffer)
{
	u8 count;
	u16 tempvalue = 0;
	u16 ret;

	if (buffer == NULL)
		return -EINVAL;

	for (count = 0; count < FILTER_SIZE; count++)
		tempvalue = tempvalue + *(buffer + count);

	ret = tempvalue/FILTER_SIZE;
	return ret;
}


static s32 da9052_bat_get_charger_vddout(struct da9052 *da9052, u16 *buffer)
{

	u8 count;
	u16 filterqueue[FILTER_SIZE];

	if (bat_status.status != DA9052_CHARGING)
		return -EIO;

	/* Measure the charger voltage using ADC function. Number
					of read equal to average filter size*/
	for (count = 0; count < FILTER_SIZE; count++)
		if (da9052_adc_read_vddout(da9052, &filterqueue[count]))
			return -EIO;

	/*Apply average filter */
	filterqueue[0] = filter_sample(filterqueue);
	/* Convert the charger voltage in terms of mV */
	bat_info.vddout = vddout_reg_to_mV(filterqueue[0]);
	*buffer = bat_info.vddout;

	return SUCCESS;
}

static s32  da9052_bat_get_battery_temperature(struct da9052 *da9052,
	u16 *buffer)
{

	u8 count;
	u16 filterqueue[FILTER_SIZE];

	/* Measure the battery temperature using ADC function.
		Number of read equal to average filter size*/

	for (count = 0; count < FILTER_SIZE; count++)
		if (da9052_adc_read_tbat(da9052, &filterqueue[count]))
			return -EIO;

	/* Apply Average filter */
	filterqueue[0] = filter_sample(filterqueue);

	bat_info.bat_temp = filterqueue[0];
	*buffer = bat_info.bat_temp;

	return SUCCESS;
}
static s32  da9052_bat_get_chg_current(struct da9052 *da9052, u16 *buffer)
{

	if (bat_status.status == DA9052_DISCHARGING_WITHOUT_CHARGER)
		return -EIO;

	/* Measure the Charger current using ADC function */
	if (da9052_adc_read_ich(da9052, buffer))
		return -EIO;

	/* Convert the raw value in terms of mA */
	bat_info.chg_current = ichg_reg_to_mA(*buffer);
	*buffer = bat_info.chg_current;

	return 0;
}

static s32 da9052_bat_get_chg_junc_temperature(struct da9052 *da9052,
	u16 *buffer)
{
	u8 count;
	u16 filterqueue[FILTER_SIZE];

	if (bat_status.status != DA9052_CHARGING)
		return -EIO;

	/* Measure the junciton temperature using ADC function. Number
	  of read equal to average filter size*/
	for (count = 0; count < FILTER_SIZE; count++)
		if (da9052_adc_read_tjunc(da9052, &filterqueue[count]))
			return -EIO;


	/* Apply average filter */
	filterqueue[0] = filter_sample(filterqueue);

	/* Convert the junction temperature raw value in terms of C */
	bat_info.chg_junc_temp = (((1708 *
						filterqueue[0])/1000) - 106);
	*buffer = bat_info.chg_junc_temp;

	return 0;
}

s32  da9052_bat_get_battery_voltage(struct da9052 *da9052, u16 *buffer)
{

	u8 count;
	u16 filterqueue[FILTER_SIZE];

	/* Measure the battery voltage using ADC function.
		Number of read equal to average filter size*/
	for (count = 0; count < FILTER_SIZE; count++)
		if (da9052_adc_read_vbat(da9052, &filterqueue[count]))
			return -EIO;

	/* Apply average filter */
	filterqueue[0] = filter_sample(filterqueue);

	/* Convert battery voltage raw value in terms of mV */
	bat_info.bat_voltage = volt_reg_to_mV(filterqueue[0]);
	*buffer = bat_info.bat_voltage;
	return 0;
}

static void da9052_charger_status_update(struct da9052_charger_device
	*chg_device)
{
	struct da9052_ssc_msg msg;
	u16 current_value = 0;
	u8 regvalue = 0;

	DA9052_DEBUG("FUNCTION = %s \n", __func__);

	msg.addr = DA9052_STATUSA_REG;
	msg.data = 0;
	da9052_lock(chg_device->da9052);

	if (chg_device->da9052->read(chg_device->da9052, &msg)) {
		DA9052_DEBUG("%s : failed\n", __func__);
		da9052_unlock(chg_device->da9052);
		return;
	}
	regvalue = msg.data;

	msg.addr = DA9052_STATUSB_REG;
	msg.data = 0;
	if (chg_device->da9052->read(chg_device->da9052, &msg)) {
		DA9052_DEBUG("%s : failed\n", __func__);
		da9052_unlock(chg_device->da9052);
		return;
	}
	da9052_unlock(chg_device->da9052);


	if ((regvalue & DA9052_STATUSA_DCINSEL)
				&& (regvalue & DA9052_STATUSA_DCINDET)) {

		/*printk(KERN_INFO "BAT_LOG:\t BAT DCIN SEL/DET:\n");*/
		if ((msg.data & DA9052_STATUSB_CHGEND) != 0)  {
			if (da9052_bat_get_chg_current(chg_device->da9052,
				&current_value)) {
					return;
			}

			if (current_value >= chg_device->chg_end_current) {
				bat_status.status = DA9052_CHARGING;
				bat_status.charger_type = DA9052_WALL_CHARGER;
			} else {
				bat_status.charger_type = DA9052_WALL_CHARGER;
				bat_status.status =
					DA9052_DISCHARGING_WITH_CHARGER;
			}
		}
		/* if Charging end flag is clered then battery is charging */
		else {
			bat_status.status = DA9052_CHARGING;
			bat_status.charger_type = DA9052_WALL_CHARGER;
		}
	} else if ((regvalue & DA9052_STATUSA_VBUSSEL)
		&& (regvalue & DA9052_STATUSA_VBUSDET)) {

			/*printk(KERN_INFO "BAT_LOG:\t BAT VBUS SEL/DET:\n");*/
			if (regvalue & DA9052_STATUSA_VDATDET) {
				bat_status.charger_type = DA9052_USB_CHARGER;
			} else {
			/* Else it has to be USB Host charger */
				bat_status.charger_type = DA9052_USB_HUB;
			}


		if ((msg.data & DA9052_STATUSB_CHGEND) != 0) {
			if (da9052_bat_get_chg_current(chg_device->da9052,
				&current_value)) {
				return;
			}
			if (current_value >= chg_device->chg_end_current) {
				bat_status.status = DA9052_CHARGING;
			} else {
				bat_status.status =
					DA9052_DISCHARGING_WITH_CHARGER;
			}
		}
		/* if Charging end flag is clered then battery is charging */
		else
			bat_status.status = DA9052_CHARGING;

	} else if (regvalue & DA9052_STATUSA_DCINDET) {

		/*printk(KERN_INFO "BAT_LOG:\t BAT DCIN DET:	\n");*/
		bat_status.charger_type = DA9052_WALL_CHARGER;
		bat_status.status = DA9052_DISCHARGING_WITH_CHARGER;

	} else if (regvalue & DA9052_STATUSA_VBUSDET) {

		/*printk(KERN_INFO "BAT_LOG:\t BAT VBUS DET:	\n");*/
		if (regvalue & DA9052_STATUSA_VDATDET) {
			bat_status.charger_type = DA9052_USB_CHARGER;
			bat_status.status = DA9052_DISCHARGING_WITH_CHARGER;
		} else {
			bat_status.charger_type = DA9052_USB_HUB;
			bat_status.status = DA9052_DISCHARGING_WITH_CHARGER;
		}
	} else {
		bat_status.charger_type = DA9052_NOCHARGER;
		bat_status.status = DA9052_DISCHARGING_WITHOUT_CHARGER;
	}
	return;
}

s32 da9052_get_bat_status(struct da9052_charger_device *chg_device,
				struct da9052_bat_status *status_buffer)
{
	struct da9052_ssc_msg msg;
	u16 buffer;

	/* If battery is in discharging mode then set charging
							mode and return */
	if (bat_status.status == DA9052_DISCHARGING_WITHOUT_CHARGER)
		bat_status.charging_mode = DA9052_NONE;

	if (bat_status.status == DA9052_DISCHARGING_WITH_CHARGER) {
		bat_status.charging_mode = DA9052_NONE;
	} else {
		da9052_lock(chg_device->da9052);
		msg.addr = DA9052_STATUSB_REG;
		/* Read STATUS_B register */
		if (chg_device->da9052->read(chg_device->da9052, &msg)) {
			da9052_unlock(chg_device->da9052);
			return DA9052_SSC_FAIL;
		}
		da9052_unlock(chg_device->da9052);

		/* Check if the battery is in Pre-charging mode */
		if (msg.data & DA9052_STATUSB_CHGPRE) {
			bat_status.charging_mode = DA9052_PRECHARGING;
			bat_status.status = DA9052_CHARGING;
		} else {
			if (da9052_bat_get_battery_voltage
			(chg_device->da9052, &buffer)) {
				DA9052_DEBUG("%s : failed\n", __func__);
				return -EIO;
			}
			if ((buffer < (chg_device->bat_target_voltage -
				chg_device->charger_voltage_drop)) &&
				(buffer > chg_device->bat_pdata->
				bat_volt_cutoff)) {
				/* Set linear charging status and return */
				bat_status.charging_mode =
					DA9052_LINEARCHARGING;
			}
			/* Else battey is in charge termintation mode.
			Set Charge end status and return */
			else if (buffer > (chg_device->bat_target_voltage -
					chg_device->charger_voltage_drop)) {
				bat_status.charging_mode = DA9052_CHARGEEND;
			}
		}
	}

	status_buffer->cal_capacity = bat_status.cal_capacity;
	status_buffer->charging_mode = bat_status.charging_mode;
	status_buffer->charger_type = bat_status.charger_type;
	status_buffer->status = bat_status.status;
	status_buffer->illegalbattery = bat_status.illegalbattery;

	return 0;
}
static s32 da9052_bat_suspend_charging(struct da9052 *da9052)
{
	struct da9052_ssc_msg msg;

	if ((bat_status.status == DA9052_DISCHARGING_WITHOUT_CHARGER) ||
		(bat_status.status == DA9052_DISCHARGING_WITH_CHARGER))
		return 0;

	msg.addr = DA9052_INPUTCONT_REG;
	msg.data = 0;
	da9052_lock(da9052);
	/* Read Input condition register */
	if (da9052->read(da9052, &msg)) {
		da9052_unlock(da9052);
		return DA9052_SSC_FAIL;
	}

	/* set both Wall charger and USB charger suspend bit */
	msg.data = set_bits(msg.data, DA9052_INPUTCONT_DCINSUSP);
	msg.data = set_bits(msg.data, DA9052_INPUTCONT_VBUSSUSP);

	/* Write to Input control register */
	if (da9052->write(da9052, &msg)) {
		da9052_unlock(da9052);
		DA9052_DEBUG("%s : failed\n", __func__);
		return DA9052_SSC_FAIL;
	}
	da9052_unlock(da9052);

	DA9052_DEBUG("%s : Sucess\n", __func__);
	return 0;
}

static s32 monitor_current(struct da9052_charger_device *chg_device)
{
	static u8 flag1 = FALSE;
	u8 count = 0;
	u16 current_value = 0;
	u16 tempvalue = 0;
	u16 avg_value = 0;

	/* Read the charger current value from the
		current measurement function */
	if (da9052_bat_get_chg_current(chg_device->da9052, &current_value)) {
		DA9052_DEBUG("%s : failed\n", __func__);
		return -EIO;
	}

	/* If montoring function is called first time then set the window */
	if (flag1 == FALSE) {
		for (count = 0; count < DA9052_NUMBER_OF_STORE_CURENT_READING;
		count++)
			bat_info.chg_current_raw[count] = 0;

		tempvalue = (CURRENT_MONITORING_WINDOW * current_value)/100;
		chg_device->threshold.ichg_av_thr_min = current_value-tempvalue;
		chg_device->threshold.ichg_av_thr_max = current_value+tempvalue;
		flag1 = TRUE;
	}

	for (count = (DA9052_NUMBER_OF_STORE_CURENT_READING-1); count > 0 ;
		count--)
			bat_info.chg_current_raw[count] =
				bat_info.chg_current_raw[count-1];

	bat_info.chg_current_raw[0] = current_value;

	/* Form last stored value of the charger current get
		the average value */
	for (count = 0; count < DA9052_NUMBER_OF_STORE_CURENT_READING;
		count++) {
			if (bat_info.chg_current_raw[count] == 0)
				break;
			avg_value = avg_value + bat_info.chg_current_raw[count];
			}
	if (count != 0)
		avg_value = avg_value/count;
	else
		avg_value = current_value;

	/* DA9052_DEBUG("Average_Current = %d\n",avg_value);*/
	/* Window is reallign with 10% of the average measur1e value*/
	tempvalue = (CURRENT_MONITORING_WINDOW * avg_value)/100;

	/* Check measured value with surge window */
	if (((current_value < chg_device->threshold.ichg_av_thr_min)
		|| (current_value > chg_device->threshold.ichg_av_thr_max))) {

		monitoring_status.current_status = TRUE;
		monitoring_status.current_value = current_value;

#if DA9052_BAT_FILTER_HYS
		printk(KERN_CRIT "\nBAT_LOG: Current Monitoring Failed = %d mA\n",
				current_value);
#endif
		chg_device->threshold.ichg_av_thr_min = avg_value - tempvalue;
		chg_device->threshold.ichg_av_thr_max = avg_value + tempvalue;
		return -EIO;
	} else {
		monitoring_status.current_status = FALSE;
		monitoring_status.current_value = current_value;
	}

	chg_device->threshold.ichg_av_thr_min = avg_value - tempvalue;
	chg_device->threshold.ichg_av_thr_max = avg_value + tempvalue;
	return 0;

}

static s32 monitor_junc_temperature(struct da9052_charger_device *chg_device)
{
	u16 buffer = 0;
	u8 ret = 0;

	/* Measure the Junction temperature using BAT internal function */
	ret = da9052_bat_get_chg_junc_temperature(chg_device->da9052, &buffer);
	if (ret)
		return ret;

	if (buffer > chg_device->threshold.tjunc_thr_limit) {
		/* If software monitoring is enabled then suspend charging */
		if (chg_device->sw_temp_cntr == 1)
			da9052_bat_suspend_charging(chg_device->da9052);

		monitoring_status.junc_temp_status = TRUE;
		monitoring_status.junc_temp_value = buffer;
		return -EIO;
	} else {
		monitoring_status.junc_temp_status = FALSE;
		monitoring_status.junc_temp_value = buffer;
	}
	return 0;
}


static s32 monitor_bat_temperature(struct da9052_charger_device *chg_device)
{
	u16 buffer;
	u8 ret = 0;

	/* Measure the BAT temperature using BAT internal function */
	ret = da9052_bat_get_battery_temperature(chg_device->da9052, &buffer);
	if (ret)
		return ret;

	if (buffer > chg_device->threshold.tbat_thr_limit) {
		/* If software monitoring is enabled then suspend charging */
		if (chg_device->sw_temp_cntr == 1)
			da9052_bat_suspend_charging(chg_device->da9052);

		monitoring_status.bat_temp_status = TRUE;
		monitoring_status.bat_temp_value = buffer;
		return -EIO;
	} else {
		monitoring_status.bat_temp_status = FALSE;
		monitoring_status.bat_temp_value = buffer;
	}
	return 0;
}

u32 interpolated(u32 vbat_lower, u32  vbat_upper, u32  level_lower,
	u32  level_upper, u32 bat_voltage)
{
	s32 temp;
	/*apply formula y= yk + (x - xk) * (yk+1 -yk)/(xk+1 -xk) */
	temp = ((level_upper - level_lower) * 1000)/(vbat_upper - vbat_lower);
	temp = level_lower + (((bat_voltage - vbat_lower) * temp)/1000);

	return temp;
}


s32 capture_first_correct_vbat_sample(struct da9052_charger_device *chg_device,
u16 *battery_voltage)
{
	static u8 count;
	s32 ret = 0;
	u32 temp_data = 0;

	ret = da9052_bat_get_battery_voltage(chg_device->da9052,
		&bat_volt_arr[count]);
	if (ret)
		return ret;
	count++;

	if (count < chg_device->bat_pdata->vbat_first_valid_detect_iteration)
		return FAILURE;
	for (count = 0; count <
		(chg_device->bat_pdata->vbat_first_valid_detect_iteration - 1);
		count++) {
			temp_data = (bat_volt_arr[count] *
			(chg_device->bat_pdata->hysteresis_window_size))/100;
		bat_hysteresis.upper_limit = bat_volt_arr[count] + temp_data;
		bat_hysteresis.lower_limit = bat_volt_arr[count] - temp_data;

		if ((bat_volt_arr[count + 1] < bat_hysteresis.upper_limit) &&
			(bat_volt_arr[count + 1] >
			bat_hysteresis.lower_limit)) {
				*battery_voltage = (bat_volt_arr[count] +
				bat_volt_arr[count+1]) / 2;
				hys_flag = TRUE;
			return 0;
		}
	}

	for (count = 0; count <
		(chg_device->bat_pdata->vbat_first_valid_detect_iteration - 1);
			count++)
		bat_volt_arr[count] = bat_volt_arr[count + 1];

	return FAILURE;
}

s32 check_hystersis(struct da9052_charger_device *chg_device, u16 *bat_voltage)
{
	u8 ret = 0;
	u32 offset = 0;

	/* Measure battery voltage using BAT internal function*/
	if (hys_flag == FALSE) {
		ret = capture_first_correct_vbat_sample
			(chg_device, &array_hys_batvoltage[0]);
		if (ret)
			return ret;
	}

	ret = da9052_bat_get_battery_voltage
		(chg_device->da9052, &array_hys_batvoltage[1]);
	if (ret)
		return ret;
	*bat_voltage = array_hys_batvoltage[1];

#if DA9052_BAT_FILTER_HYS
	printk(KERN_CRIT "\nBAT_LOG: Previous Battery Voltage = %d mV\n",
				array_hys_batvoltage[0]);
	printk(KERN_CRIT "\nBAT_LOG:Battery Voltage Before Filter = %d mV\n",
				array_hys_batvoltage[1]);
#endif
	/* Check if measured battery voltage value is within the hysteresis
		window limit using measured battey votlage value */
	if ((bat_hysteresis.upper_limit < *bat_voltage) ||
			(bat_hysteresis.lower_limit > *bat_voltage)) {

		bat_hysteresis.index++;

		if (bat_hysteresis.index ==
			chg_device->bat_pdata->hysteresis_no_of_reading) {
			/* Hysteresis Window is set to +- of
			HYSTERESIS_WINDOW_SIZE percentage of current VBAT */
			bat_hysteresis.index = 0;
			offset = ((*bat_voltage) *
				chg_device->bat_pdata->hysteresis_window_size)/
				100;
			bat_hysteresis.upper_limit = (*bat_voltage) + offset;
			bat_hysteresis.lower_limit = (*bat_voltage) - offset;

		} else {
#if DA9052_BAT_FILTER_HYS
			printk(KERN_CRIT "CheckHystersis: Failed\n");
#endif
			return -EIO;
		}
	} else {
		bat_hysteresis.index = 0;
		offset = ((*bat_voltage) *
			chg_device->bat_pdata->hysteresis_window_size)/100;
		bat_hysteresis.upper_limit = (*bat_voltage) + offset;
		bat_hysteresis.lower_limit = (*bat_voltage) - offset;
	}

	/* Digital C Filter, formula Yn = k Yn-1 + (1-k) Xn */
	*bat_voltage = ((chg_device->bat_pdata->chg_hysteresis_const *
		array_hys_batvoltage[0])/100) +
		(((100 - chg_device->bat_pdata->chg_hysteresis_const) *
		array_hys_batvoltage[1])/100);

	if ((bat_status.status == DA9052_DISCHARGING_WITHOUT_CHARGER) &&
		(*bat_voltage > array_hys_batvoltage[0])) {
			*bat_voltage = array_hys_batvoltage[0];
	}

	/*DA9052_DEBUG("Voltage Final =%d \n",*bat_voltage);*/
	array_hys_batvoltage[0] = *bat_voltage;

#if DA9052_BAT_FILTER_HYS
	printk(KERN_CRIT "\nBAT_LOG:Battery Voltage After Filter = %d mV\n",\
		*bat_voltage);
#endif
	return 0;
}

u8 select_temperature(u8 temp_index, u16 bat_temperature)
{
	u16 temp_temperature = 0;
	temp_temperature = (temperature_lookup_ref[temp_index] +
				temperature_lookup_ref[temp_index+1]) / 2;

	if (bat_temperature >= temp_temperature) {
		temp_index += 1;
		return temp_index;
	} else
		return temp_index;
}

s32 da9052_get_bat_level(struct da9052_charger_device *chg_device)
{
	u16 bat_temperature;
	u16 bat_voltage;
	u32 vbat_lower, vbat_upper, level_upper, level_lower, level;
	u8 access_index = 0;
	u8 index = 0, ret;
	u8 flag = FALSE;

	ret = 0;
	vbat_lower = 0;
	vbat_upper = 0;
	level_upper = 0;
	level_lower = 0;

	ret = check_hystersis(chg_device, &bat_voltage);
	if (ret)
		return ret;

	ret = da9052_bat_get_battery_temperature(chg_device->da9052,
		&bat_temperature);
	if (ret)
		return ret;

	for (index = 0; index < (DA9052_NO_OF_LOOKUP_TABLE-1); index++) {
		if (bat_temperature <= temperature_lookup_ref[0]) {
			access_index = 0;
			break;
		} else if (bat_temperature >
			temperature_lookup_ref[DA9052_NO_OF_LOOKUP_TABLE]){
				access_index = DA9052_NO_OF_LOOKUP_TABLE - 1;
			break;
		} else if ((bat_temperature >= temperature_lookup_ref[index]) &&
			(bat_temperature >= temperature_lookup_ref[index+1])) {
			access_index = select_temperature(index,
				bat_temperature);
			break;
		}
	}
	if (bat_voltage >= vbat_vs_capacity_look_up[access_index][0][0]) {
		bat_status.cal_capacity = 100;
		return 0;
	}
	if (bat_voltage <= vbat_vs_capacity_look_up[access_index]
		[DA9052_LOOK_UP_TABLE_SIZE-1][0]){
			bat_status.cal_capacity = 0;
			return 0;
	}
	flag = FALSE;

	for (index = 0; index < (DA9052_LOOK_UP_TABLE_SIZE-1); index++) {
		if ((bat_voltage <=
		vbat_vs_capacity_look_up[access_index][index][0]) &&
		(bat_voltage >=
		vbat_vs_capacity_look_up[access_index][index+1][0])) {
			vbat_upper =
			vbat_vs_capacity_look_up[access_index][index][0];
			vbat_lower =
			vbat_vs_capacity_look_up[access_index][index+1][0];
			level_upper =
			vbat_vs_capacity_look_up[access_index][index][1];
			level_lower =
			vbat_vs_capacity_look_up[access_index][index+1][1];
			flag = TRUE;
			break;
		}
	}
	if (!flag)
		return -EIO;

	level = interpolated(vbat_lower, vbat_upper, level_lower,
		level_upper, bat_voltage);
	bat_status.cal_capacity = level;
	DA9052_DEBUG(" TOTAl_BAT_CAPACITY : %d\n", bat_status.cal_capacity);
	return 0;
}

void da9052_bat_vddlow_handler(struct da9052_eh_nb *eh_data, unsigned int event)
{	u16 buffer = 0;
	struct da9052_charger_device *charger =
		container_of(eh_data, struct da9052_charger_device,
			vddlow_eh_data/*da9052_eh_nb*/);

	if (!monitoring_status.vddout_status) {
		/* Measure VDDOUT to update the Monitoring status*/
		monitoring_status.vddout_status = TRUE;
		if (da9052_bat_get_charger_vddout(charger->da9052, &buffer)) {
			DA9052_DEBUG("%s : VDDOUT Measurement Fails.\n",
			__func__);
		}
		monitoring_status.vddout_value = buffer;
	}
}

void da9052_bat_tbat_handler(struct da9052_eh_nb *eh_data, unsigned int event)
{
	if (!tbat_event_occur) {
		bat_status.health = POWER_SUPPLY_HEALTH_OVERHEAT;
		tbat_event_occur = TRUE;
		/* update the TBAt value */
		monitoring_status.bat_temp_status = TRUE;
		monitoring_status.bat_temp_value = bat_info.bat_temp;
	}
}

static s32 da9052_bat_register_event(struct da9052_charger_device *chg_device,
	u8 event_type)
{
	s32 ret;
	switch (event_type) {
	case VDD_LOW_EVE:
		if (event_status.da9052_event_vddlow == FALSE) {
			chg_device->vddlow_eh_data.eve_type = event_type;
			chg_device->vddlow_eh_data.call_back =
				da9052_bat_vddlow_handler;
			DA9052_DEBUG("events = %d\n", event_type);
			ret = chg_device->da9052->register_event_notifier
				(chg_device->da9052,
					&chg_device->vddlow_eh_data);
			if (ret)
				return -EIO;
			event_status.da9052_event_vddlow = TRUE;
		}
		break;
	case TBAT_EVE:
		if (event_status.da9052_event_tbat == FALSE) {
			chg_device->tbat_eh_data.eve_type = event_type;
			chg_device->tbat_eh_data.call_back =
				da9052_bat_tbat_handler;
			DA9052_DEBUG("events = %d\n", event_type);
			ret = chg_device->da9052->register_event_notifier
				(chg_device->da9052, &chg_device->tbat_eh_data);
		if (ret)
			return -EIO;
		event_status.da9052_event_tbat = TRUE;
	}
	break;
	default:
		return -EIO;
	}
	return 0;
}

static s32 da9052_bat_unregister_event(struct da9052_charger_device *chg_device,
	u8 event_type)
{
	s32 ret;
	switch (event_type) {
	case VDD_LOW_EVE:
		if (event_status.da9052_event_vddlow) {
			ret =
				chg_device->da9052->unregister_event_notifier
				(chg_device->da9052,
					&chg_device->vddlow_eh_data);
		if (ret)
				return -EIO;
			event_status.da9052_event_vddlow = FALSE;
		}
	break;
	case TBAT_EVE:
		if (event_status.da9052_event_tbat) {
			ret =
				chg_device->da9052->unregister_event_notifier
				(chg_device->da9052, &chg_device->tbat_eh_data);
			if (ret)
				return -EIO;
			event_status.da9052_event_tbat = FALSE;
		}
	break;
	default:
		return -EIO;
	}
	return 0;
}

static s32 monitoring_thread(void *data)
{
	u8 mon_count = 0;
	s32 ret = 0;
#if DA9052_BAT_PROFILE
	u8 mon_times = 0;
	u32 jiffies_count = 0;
	u32 msec_time_bat_Level = 0;
#endif

	struct da9052_charger_device *chg_device =
		(struct da9052_charger_device *)data;

	set_freezable();

	while (monitoring_thread_state == ACTIVE) {
		/* Make this thread friendly to system suspend and resume */
		try_to_freeze();

		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(chg_device->monitoring_interval);

#if DA9052_BAT_PROFILE
		jiffies_count = jiffies;
#endif
		da9052_charger_status_update(chg_device);
		/* check if battery is in charging mode */
		if (bat_status.status == DA9052_CHARGING) {
			/*  If battery is in charging mode then only call
			charger current and bat temperature monitoring  */
			if (mon_count == 0) {
				if (-EIO == monitor_current(chg_device)) {
					DA9052_DEBUG("charging Current\
					Monitoring failed, %d\n", mon_count);
				}
			}
			/* call junction temperature monitoring */
			else if (mon_count == 1) {
				if (-EIO ==
					monitor_junc_temperature(chg_device))
					DA9052_DEBUG("Charger Junction\
					Temperature Monitoring failed\n");
			}
		}

		if (mon_count == 2) {
			ret = da9052_get_bat_level(chg_device);
			if (!ret) {
				/* BAT Capacity is low then
				update the monitoring status*/
				if (bat_status.cal_capacity <
				chg_device->bat_pdata->bat_capacity_limit_low) {
					monitoring_status.bat_level_status =
						TRUE;
					monitoring_status.bat_level =
						bat_status.cal_capacity;
				} else {
					monitoring_status.bat_level_status = 0;
					monitoring_status.bat_level =
						bat_status.cal_capacity;
				}
			} else
				DA9052_DEBUG("Battery Measurement Fails =\
				%d\n", ret);
		}

		if (mon_count == 3)
			if (-EIO == monitor_bat_temperature(chg_device))
				DA9052_DEBUG("BAT Temperature\
					Monitoring failed\n");
#if DA9052_BAT_PROFILE
		jiffies_count =  jiffies - jiffies_count;

		if (mon_count == 2) {
			mon_times++;
			msec_time_bat_Level = msec_time_bat_Level +
						jiffies_to_msecs(jiffies_count);

			if (mon_times == 5) {
				mon_times = 0;
				msec_time_bat_Level = 0;
			}
		}
#endif
		mon_count++;
		if (mon_count == 4)
			mon_count = 0;
	}

	complete_and_exit(&monitoring_thread_notifier, 0);

	return 0;
}

#if (DA9052_ILLEGAL_BATTERY_DETECT)
static s32 detect_illegal_battery(struct da9052_charger_device *chg_device)
{
	u16 buffer = 0;
	s32  ret = 0;

	/* Measure battery temeperature */
	ret = da9052_bat_get_battery_temperature(chg_device->da9052, &buffer);
	if (ret) {
		DA9052_DEBUG("%s: Battery temperature measurement failed \n",
		__func__);
		return ret;
	}

	if (buffer > chg_device->bat_pdata->bat_with_no_resistor)
		bat_status.illegalbattery = TRUE;
	else
		bat_status.illegalbattery = FALSE;


	/* suspend charging of battery if illegal battey is detected */
	if (bat_status.illegalbattery)
		da9052_bat_suspend_charging(chg_device->da9052);

	return SUCCESS;
}
#endif

static int da9052_bat_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	s32 ret = 0;

	struct da9052_charger_device *chg_device =
	container_of(psy, struct da9052_charger_device, psy);


	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (bat_status.status == DA9052_CHARGING)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;

		else if (bat_status.status ==
			DA9052_DISCHARGING_WITH_CHARGER)
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;

		else if (bat_status.status ==
				DA9052_DISCHARGING_WITHOUT_CHARGER)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;

		else if (bat_status.status == DA9052_CHARGEEND)
			val->intval = POWER_SUPPLY_STATUS_FULL;

	break;
	case POWER_SUPPLY_PROP_ONLINE:
		if (bat_status.charger_type == DA9052_NOCHARGER)
			val->intval = 0;
		else
			val->intval = 1;
	break;
	case POWER_SUPPLY_PROP_PRESENT:
		if (bat_status.illegalbattery)
			val->intval = 0;
	else
		val->intval = 1;
	break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (bat_status.health != POWER_SUPPLY_HEALTH_OVERHEAT) {
			if (bat_status.illegalbattery)
				bat_status.health = POWER_SUPPLY_HEALTH_UNKNOWN;

			else if (bat_status.cal_capacity <
				chg_device->bat_pdata->bat_capacity_limit_low)
				bat_status.health = POWER_SUPPLY_HEALTH_DEAD;

			else
				bat_status.health = POWER_SUPPLY_HEALTH_GOOD;
		}
		val->intval = bat_status.health;
	break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = (bat_target_voltage * 1000);
	break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = (chg_device->bat_pdata->bat_volt_cutoff * 1000);
	break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		val->intval = (bat_info.bat_voltage * 1000);
	break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = (bat_info.chg_current * 1000);
	break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bat_status.cal_capacity;
	break;
#if 0
	/* Following properties are not supported in 2.6.28 kernel */
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		if (bat_status.illegalbattery)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;

		else if (bat_status.cal_capacity <
			chg_device->bat_pdata->bat_capacity_limit_low)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;

		else if (bat_status.cal_capacity <
			chg_device->bat_pdata->bat_capacity_limit_high)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;

		else if (bat_status.cal_capacity ==
			chg_device->bat_pdata->bat_capacity_full)
				val->intval = POWER_SUPPLY_CAPACITY_LEVEL_FULL;

		else if (bat_status.cal_capacity >
			chg_device->bat_pdata->bat_capacity_limit_high)
				val->intval = POWER_SUPPLY_CAPACITY_LEVEL_HIGH;

		else
			val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
#endif
	break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = bat_temp_reg_to_C(bat_info.bat_temp);
	break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = BAT_MANUFACTURER;
	break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = BAT_TYPE;
	break;
	default:
		ret = -EINVAL;
	break;
	}
	return 0;
}

static void da9052_battery_setup_psy(struct da9052_charger_device *chg_device)
{
	battery_info.name = DA9052_BAT_DEVICE_NAME;
	battery_info.technology = BAT_TYPE;
	battery_info.voltage_max_design =
				(chg_device->bat_target_voltage*1000);
	battery_info.voltage_min_design =
		(chg_device->bat_pdata->bat_volt_cutoff*1000);
	battery_info.energy_full_design =
		chg_device->bat_pdata->bat_capacity_full;
	battery_info.energy_empty_design =
		chg_device->bat_pdata->bat_capacity_limit_low;
	battery_info.use_for_apm = 1;

	chg_device->psy.name = DA9052_BAT_DEVICE_NAME;
	chg_device->psy.use_for_apm = 1;
	chg_device->psy.type = POWER_SUPPLY_TYPE_BATTERY;
	chg_device->psy.get_property = da9052_bat_get_property;

	chg_device->psy.properties = da902_bat_props;
	chg_device->psy.num_properties = ARRAY_SIZE(da902_bat_props);

};

void get_bat_mode(u8 mode_num, char *temp_name)
{
	if (mode_num == DA9052_NONE)
		sprintf(temp_name, "NONE");
	else if (mode_num == DA9052_CHARGING)
		sprintf(temp_name, "CHARGING");
	else if (mode_num == DA9052_DISCHARGING_WITH_CHARGER)
		sprintf(temp_name, "DISCHARGING_WITH_CHARGER");
	else if (mode_num == DA9052_DISCHARGING_WITHOUT_CHARGER)
		sprintf(temp_name, "DISCHARGING_WITHOUT_CHARGER");
}

void get_charging_mode(u8 mode, char *temp_name)
{
	if (mode == DA9052_NONE)
		sprintf(temp_name, "NONE");
	else if (mode == DA9052_PRECHARGING)
		sprintf(temp_name, "PRECHARGING");
	else if (mode == DA9052_LINEARCHARGING)
		sprintf(temp_name, "LINEARCHARGING");
	else if (mode == DA9052_CHARGEEND)
		sprintf(temp_name, "CHARGEEND");
}

void get_charger_type(u8 type, char *temp_name)
{
	if (type == DA9052_USB_HUB)
		sprintf(temp_name, "USB_HUB");
	else if (type == DA9052_NOCHARGER)
		sprintf(temp_name, "NOCHARGER");
	else if (type == DA9052_USB_CHARGER)
		sprintf(temp_name, "USB_CHARGER");
	else if (type == DA9052_WALL_CHARGER)
		sprintf(temp_name, "WALL_CHARGER");
}

#if (DA9052_BAT_STATUS == 1)
static ssize_t da9052_bat_print_status(void *ptr)
{
	s32 result;
	struct da9052_bat_status bat_status;
	struct da9052_charger_device *chg_device =
		(struct da9052_charger_device *)ptr;
	struct da9052_ssc_msg msg;
	u8 regvalue = 0;


	set_freezable();

	while (chg_device->print_bat_status.state == ACTIVE) {

		try_to_freeze();

		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies((40*1000)));

		#if 1

		msg.addr = DA9052_STATUSA_REG;
		msg.data = 0;
		da9052_lock(chg_device->da9052);
		if (0 != chg_device->da9052->read(chg_device->da9052, &msg)) {
			DA9052_DEBUG("%s : failed\n", __func__);
			da9052_unlock(chg_device->da9052);
		} else {
			da9052_unlock(chg_device->da9052);

			regvalue = msg.data;

			if ((regvalue & DA9052_STATUSA_DCINSEL)
				&& (regvalue & DA9052_STATUSA_DCINDET))
				printk(KERN_INFO "BAT_LOG:\t BAT DCIN SEL/DET:	\n");
			else if ((regvalue & DA9052_STATUSA_VBUSSEL)
				&& (regvalue & DA9052_STATUSA_VBUSDET))
				printk(KERN_INFO "BAT_LOG:\t BAT VBUS SEL/DET:	\n");
			else if (regvalue & DA9052_STATUSA_DCINDET)
				printk(KERN_INFO "BAT_LOG:\t BAT DCIN DET:	\n");
			else if (regvalue & DA9052_STATUSA_VBUSDET)
				printk(KERN_INFO "BAT_LOG:\t BAT VBUS DET:	\n");
		}

		#endif

		result = da9052_get_bat_status(chg_device, &bat_status);
		if (result)
			goto end_monitoring_thread;

		switch (bat_status.illegalbattery) {
		case 1:
			printk(KERN_INFO "BAT_LOG:\t Illegal Battery Detected \n");
		break;
		default:
			printk(KERN_INFO "BAT_LOG:\t Valid Battery Detected\n");
		break;
		}

		switch (bat_status.charger_type) {
		case 1:
			printk(KERN_INFO "BAT_LOG:\t Charger_Type= DA9052_NOCHARGER\n");
		break;
		case 2:
			printk(KERN_INFO "BAT_LOG:\t Charging_Type= DA9052_USB_HUB\n");
		break;
		case 3:
			printk(KERN_INFO "BAT_LOG:\t Charging_Type= DA9052_USB_CHARGER\n");
		break;
		case 4:
			printk(KERN_INFO "BAT_LOG:\t Charging_Type= DA9052_WALL_CHARGER \n");
		default:
			printk(KERN_INFO "BAT_LOG:\t Charging_Type= INVALID_CHARGER \n");
		break;
		}
		switch (bat_status.charging_mode) {
		case 1:
			printk(KERN_INFO "BAT_LOG:\t Charging_Mode = DA9052_NONE \n");
		break;
		case 2:
			printk(KERN_INFO "BAT_LOG:\t Charging_Mode = DA9052_CHARGING \n");
		break;
		case 3:
			printk(KERN_INFO "BAT_LOG:\t Charging_Mode =DA9052_DISCHARGING_WITH_CHARGER\n");
		break;
		case 4:
			printk(KERN_INFO "BAT_LOG:\t Charging_Mode =DA9052_DISCHARGING_WITHOUT_CHARGER \n");
		break;
		case 5:
			printk(KERN_INFO "BAT_LOG:\t Charging_Mode = DA9052_PRECHARGING\n");
		break;
		case 6:
			printk(KERN_INFO "BAT_LOG:\t Charging_Mode = DA9052_LINEAR_CHARGING\n");
		break;
		case 7:
			printk(KERN_INFO "BAT_LOG:\t Charging_Mode = DA9052_CHARGE_END\n");
		break;
		default:
			printk(KERN_INFO "BAT_LOG:\t Charging_Mode =\
			DA9052_INVALID_MODE \n");
		break;
		}
		printk(KERN_INFO "BAT_LOG:\t BAT Level Value: %d percent\n",
		bat_status.cal_capacity);
		printk(KERN_INFO "BAT_LOG:\t BAT Voltage: %d \n",
		bat_info.bat_voltage);
		printk(KERN_INFO "BAT_LOG:\t BAT Temperture: %d \n",
		bat_info.bat_temp);
		printk(KERN_INFO "BAT_LOG:\t Junction Temperture: %d \n",
		bat_info.chg_junc_temp);
		printk(KERN_INFO "BAT_LOG:\t Charging Current: %d \n\n",
		bat_info.chg_current);
	}
end_monitoring_thread:
	complete_and_exit(&chg_device->print_bat_status.notifier, 0);
	return SUCCESS;
}
#endif

static s32 __devinit da9052_bat_probe(struct platform_device *pdev)
{
	struct da9052_bat_platform_data *pdata = pdev->dev.platform_data;
	struct da9052_charger_device *chg_device;
	u8 reg_data;
	int ret;

	chg_device = kzalloc(sizeof(*chg_device), GFP_KERNEL);
	if (!chg_device)
		return -ENOMEM;

	chg_device->da9052 = dev_get_drvdata(pdev->dev.parent);

	chg_device->bat_pdata = pdata;

	platform_set_drvdata(pdev, chg_device);

	chg_device->monitoring_interval =
		msecs_to_jiffies(chg_device->bat_pdata->monitoring_interval);
	chg_device->sw_temp_cntr = chg_device->bat_pdata->sw_temp_control_en;
	chg_device->usb_charger_current = 0;

	ret = da9052_read(chg_device->da9052, DA9052_CHGCONT_REG, &reg_data);
	if (ret)
		goto err_charger_init;
	chg_device->charger_voltage_drop = bat_drop_reg_to_mV(reg_data &&
							DA9052_CHGCONT_TCTR);
	chg_device->bat_target_voltage =
			bat_reg_to_mV(reg_data && DA9052_CHGCONT_VCHGBAT);
	bat_target_voltage = chg_device->bat_target_voltage;

	reg_data = 0;
	ret = da9052_read(chg_device->da9052, DA9052_ICHGEND_REG, &reg_data);
	if (ret)
		goto err_charger_init;

	chg_device->chg_end_current = ichg_reg_to_mA(reg_data);

	chg_device->threshold.tbat_thr_limit =
		chg_device->bat_pdata->sw_bat_temp_threshold;
	chg_device->threshold.tjunc_thr_limit =
		chg_device->bat_pdata->sw_junc_temp_threshold;

	sprintf(bat_info.manufacture, BAT_MANUFACTURER);
	bat_status.illegalbattery = 0;

	bat_hysteresis.upper_limit = 0;
	bat_hysteresis.lower_limit = 0;
	bat_hysteresis.hys_flag = 0;

	bat_status.charger_type = DA9052_NOCHARGER;
	bat_status.status = DA9052_CHARGING;
	bat_status.charging_mode = DA9052_NONE;
	tbat_event_occur = 0;
#if (DA9052_ILLEGAL_BATTERY_DETECT)
	detect_illegal_battery(chg_device);
#endif
	da9052_charger_status_update(chg_device);

	da9052_battery_setup_psy(chg_device);

	ret = da9052_bat_register_event(chg_device, VDD_LOW_EVE);
	if (ret)
		goto err_charger_init;
	da9052_bat_register_event(chg_device, TBAT_EVE);
	if (ret)
		goto err_charger_init;

		ret = power_supply_register(&pdev->dev, &chg_device->psy);
	 if (ret)
		goto err_charger_init;

	monitoring_thread_state = ACTIVE;
	init_completion(&monitoring_thread_notifier);
	monitoring_thread_pid = kernel_thread(monitoring_thread, chg_device,
						CLONE_KERNEL | SIGCHLD);
	if (monitoring_thread_pid > 0) {
		printk(KERN_ERR "Monitoring thread is successfully started,\
		pid = %d\n", monitoring_thread_pid);
	}
#if (DA9052_BAT_STATUS == 1)
	init_completion(&chg_device->print_bat_status.notifier);
	chg_device->print_bat_status.state = ACTIVE;
	chg_device->print_bat_status.pid =
		kernel_thread(da9052_bat_print_status, chg_device,
			CLONE_KERNEL | SIGCHLD);
#endif

	printk(KERN_INFO "Exiting DA9052 battery probe \n");
	return 0;

err_charger_init:
	platform_set_drvdata(pdev, NULL);
	kfree(chg_device);
	return ret;
}
static int __devexit da9052_bat_remove(struct platform_device *dev)
{
	struct da9052_charger_device *chg_device = platform_get_drvdata(dev);
	s32 ret;

	monitoring_thread_state = INACTIVE;
	wait_for_completion(&monitoring_thread_notifier);

#if (DA9052_BAT_STATUS == 1)
	/* stop and delete monitoring timer */
	chg_device->print_bat_status.state = INACTIVE;
	wait_for_completion(&chg_device->print_bat_status.notifier);
#endif

	/* unregister the events.*/
	ret = da9052_bat_unregister_event(chg_device, VDD_LOW_EVE);
	ret = da9052_bat_unregister_event(chg_device, TBAT_EVE);
	return 0;
}

static struct platform_driver da9052_bat_driver = {
	.probe		= da9052_bat_probe,
	.remove		= __devexit_p(da9052_bat_remove),
	.driver		= {
		.name	= DA9052_BAT_DEVICE_NAME,
		.owner	= THIS_MODULE,
	},
};

static s32 __init da9052_bat_init(void)
{
	printk(banner);
	return platform_driver_register(&da9052_bat_driver);
}
module_init(da9052_bat_init);

static void __exit da9052_bat_exit(void)
{
	printk("DA9052: Unregistering BAT device.\n");
	platform_driver_unregister(&da9052_bat_driver);
}
module_exit(da9052_bat_exit);

MODULE_AUTHOR("Dialog Semiconductor Ltd");
MODULE_DESCRIPTION("DA9052 BAT Device Driver");
MODULE_LICENSE("GPL");
