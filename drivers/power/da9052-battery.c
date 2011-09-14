/*
 * da9052-battery.c  --  Batttery Driver for Dialog DA9052
 *
 * Copyright(c) 2009 Dialog Semiconductor Ltd.
 
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

static struct da9052_bat_hysteresis bat_hysteresis;
static struct da9052_bat_event_registration event_status;


static u16 array_hys_batvoltage[2];
static u16 bat_volt_arr[3];
static u8 hys_flag = FALSE;

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


static s32 da9052_adc_read_tbat(struct da9052 *da9052, u16 *data)
{
	s32 ret;
	u8 reg_data;

	ret = da9052_read(da9052, DA9052_TBATRES_REG, &reg_data);
	if (ret)
		return ret;
	*data = (u16)reg_data;

	DA9052_DEBUG("In function: %s, TBATRES_REG value read (1)= 0x%X \n",
		__func__, msg.data);
	return SUCCESS;
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

	for (count = 0; count < DA9052_FILTER_SIZE; count++)
		tempvalue = tempvalue + *(buffer + count);

	ret = tempvalue/DA9052_FILTER_SIZE;
	return ret;
}

static s32  da9052_bat_get_battery_temperature(struct da9052_charger_device
	*chg_device, u16 *buffer)
{

	u8 count;
	u16 filterqueue[DA9052_FILTER_SIZE];

	/* Measure the battery temperature using ADC function.
		Number of read equal to average filter size*/

	for (count = 0; count < DA9052_FILTER_SIZE; count++)
		if (da9052_adc_read_tbat(chg_device->da9052, &filterqueue[count]))
			return -EIO;

	/* Apply Average filter */
	filterqueue[0] = filter_sample(filterqueue);

	chg_device->bat_temp = filterqueue[0];
	*buffer = chg_device->bat_temp;

	return SUCCESS;
}

static s32  da9052_bat_get_chg_current(struct da9052_charger_device
	*chg_device, u16 *buffer)
{
	if (chg_device->status == POWER_SUPPLY_STATUS_DISCHARGING)
		return -EIO;

		/* Measure the Charger current using ADC function */
	if (da9052_adc_read_ich(chg_device->da9052, buffer))
		return -EIO;

	/* Convert the raw value in terms of mA */
	chg_device->chg_current = ichg_reg_to_mA(*buffer);
	*buffer = chg_device->chg_current;

	return 0;
}


s32  da9052_bat_get_battery_voltage(struct da9052_charger_device *chg_device,
 u16 *buffer)
{
	u8 count;
	u16 filterqueue[DA9052_FILTER_SIZE];

	/* Measure the battery voltage using ADC function.
		Number of read equal to average filter size*/
	for (count = 0; count < DA9052_FILTER_SIZE; count++)
		if (da9052_adc_read_vbat(chg_device->da9052, &filterqueue[count]))
			return -EIO;

	/* Apply average filter */
	filterqueue[0] = filter_sample(filterqueue);

	/* Convert battery voltage raw value in terms of mV */
	chg_device->bat_voltage = volt_reg_to_mV(filterqueue[0]);
	*buffer = chg_device->bat_voltage;
	return 0;
}

static void da9052_bat_status_update(struct da9052_charger_device
	*chg_device)
{
	struct da9052_ssc_msg msg;
	u16 current_value = 0;
	u16 buffer =0;
	u8 regvalue = 0;
	u8 old_status = chg_device->status;
	
	DA9052_DEBUG("FUNCTION = %s \n", __func__);

	/* Read Status A register */
	msg.addr = DA9052_STATUSA_REG;
	msg.data = 0;
	da9052_lock(chg_device->da9052);

	if (chg_device->da9052->read(chg_device->da9052, &msg)) {
		DA9052_DEBUG("%s : failed\n", __func__);
		da9052_unlock(chg_device->da9052);
		return;
	}
	regvalue = msg.data;

	/* Read Status B register */
	msg.addr = DA9052_STATUSB_REG;
	msg.data = 0;
	if (chg_device->da9052->read(chg_device->da9052, &msg)) {
		DA9052_DEBUG("%s : failed\n", __func__);
		da9052_unlock(chg_device->da9052);
		return;
	}
	da9052_unlock(chg_device->da9052);

	/* If DCINDET and DCINSEL are set then connected charger is
						WALL Charger unit */
	if( (regvalue & DA9052_STATUSA_DCINSEL) 
				&& (regvalue & DA9052_STATUSA_DCINDET) ) {

		chg_device->charger_type = DA9052_WALL_CHARGER;
	}
	/* If VBUS_DET and VBUSEL are set then connected charger is
		USB Type */
	else if((regvalue & DA9052_STATUSA_VBUSSEL) 
				&& (regvalue & DA9052_STATUSA_VBUSDET)) {
		if (regvalue & DA9052_STATUSA_VDATDET) {
			chg_device->charger_type = DA9052_USB_CHARGER;
		}
		else {
			/* Else it has to be USB Host charger */
			chg_device->charger_type = DA9052_USB_HUB;
		}
	}
	/* Battery is discharging since charging device is not present */
	else
	{
		chg_device->charger_type = DA9052_NOCHARGER;
		/* Eqv to DISCHARGING_WITHOUT_CHARGER state */
		chg_device->status = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	
	if( chg_device->charger_type != DA9052_NOCHARGER ) {
		/* if Charging end flag is set and Charging current is greater
			than charging end limit then battery is charging */
		if ((msg.data & DA9052_STATUSB_CHGEND) != 0)  {
				
			if(da9052_bat_get_chg_current(chg_device,&current_value)) {
					return;
			}
				
			if( current_value >= chg_device->chg_end_current ) {
				chg_device->status = POWER_SUPPLY_STATUS_CHARGING;
			}
			else {
				/* Eqv to DISCHARGING_WITH_CHARGER state*/
				chg_device->status = POWER_SUPPLY_STATUS_NOT_CHARGING;
			}
		}
		/* if Charging end flag is cleared then battery is charging */
		else {
			chg_device->status = POWER_SUPPLY_STATUS_CHARGING;
		}
		
		if( POWER_SUPPLY_STATUS_CHARGING == chg_device->status){
			if(msg.data != DA9052_STATUSB_CHGPRE) {
				/* Measure battery voltage. if battery voltage is greater than
				(VCHG_BAT - VCHG_DROP) then battery is in the termintation mode.
				*/
				if(da9052_bat_get_battery_voltage(chg_device,&buffer)) {
					DA9052_DEBUG("%s : failed\n",__FUNCTION__);
					return ;
				}
				if(buffer > (chg_device->bat_target_voltage -
					chg_device->charger_voltage_drop) &&
					( chg_device->cal_capacity >= 99 ) ){
					chg_device->status = POWER_SUPPLY_STATUS_FULL;
				}
				
			}
		}
	}
	
	if(chg_device->illegal)
		chg_device->health = POWER_SUPPLY_HEALTH_UNKNOWN;
	else if (chg_device->cal_capacity < chg_device->bat_capacity_limit_low)
		chg_device->health = POWER_SUPPLY_HEALTH_DEAD;
	else
		chg_device->health = POWER_SUPPLY_HEALTH_GOOD;
	
	if ( chg_device->status != old_status)
		power_supply_changed(&chg_device->psy);
		
	return;
}

static s32 da9052_bat_suspend_charging(struct da9052_charger_device *chg_device)
{
	struct da9052_ssc_msg msg;

	if ((chg_device->status == POWER_SUPPLY_STATUS_DISCHARGING) ||
		(chg_device->status == POWER_SUPPLY_STATUS_NOT_CHARGING))
		return 0;

	msg.addr = DA9052_INPUTCONT_REG;
	msg.data = 0;
	da9052_lock(chg_device->da9052);
	/* Read Input condition register */
	if (chg_device->da9052->read(chg_device->da9052, &msg)) {
		da9052_unlock(chg_device->da9052);
		return DA9052_SSC_FAIL;
	}

	/* set both Wall charger and USB charger suspend bit */
	msg.data = set_bits(msg.data, DA9052_INPUTCONT_DCINSUSP);
	msg.data = set_bits(msg.data, DA9052_INPUTCONT_VBUSSUSP);

	/* Write to Input control register */
	if (chg_device->da9052->write(chg_device->da9052, &msg)) {
		da9052_unlock(chg_device->da9052);
		DA9052_DEBUG("%s : failed\n", __func__);
		return DA9052_SSC_FAIL;
	}
	da9052_unlock(chg_device->da9052);

	DA9052_DEBUG("%s : Sucess\n", __func__);
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

	ret = da9052_bat_get_battery_voltage(chg_device,
		&bat_volt_arr[count]);
	if (ret)
		return ret;
	count++;

	if (count < chg_device->vbat_first_valid_detect_iteration)
		return FAILURE;
	for (count = 0; count <
		(chg_device->vbat_first_valid_detect_iteration - 1);
		count++) {
			temp_data = (bat_volt_arr[count] *
			(chg_device->hysteresis_window_size))/100;
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
		(chg_device->vbat_first_valid_detect_iteration - 1);
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
		(chg_device, &array_hys_batvoltage[1]);
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
			chg_device->hysteresis_no_of_reading) {
			/* Hysteresis Window is set to +- of
			HYSTERESIS_WINDOW_SIZE percentage of current VBAT */
			bat_hysteresis.index = 0;
			offset = ((*bat_voltage) *
				chg_device->hysteresis_window_size)/
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
			chg_device->hysteresis_window_size)/100;
		bat_hysteresis.upper_limit = (*bat_voltage) + offset;
		bat_hysteresis.lower_limit = (*bat_voltage) - offset;
	}

	/* Digital C Filter, formula Yn = k Yn-1 + (1-k) Xn */
	*bat_voltage = ((chg_device->chg_hysteresis_const *
		array_hys_batvoltage[0])/100) +
		(((100 - chg_device->chg_hysteresis_const) *
		array_hys_batvoltage[1])/100);

	if ((chg_device->status == POWER_SUPPLY_STATUS_DISCHARGING) &&
		(*bat_voltage > array_hys_batvoltage[0])) {
			*bat_voltage = array_hys_batvoltage[0];
	}

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

s32 da9052_bat_level_update(struct da9052_charger_device *chg_device)
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

	ret = da9052_bat_get_battery_temperature(chg_device,
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
		chg_device->cal_capacity = 100;
		return 0;
	}
	if (bat_voltage <= vbat_vs_capacity_look_up[access_index]
		[DA9052_LOOK_UP_TABLE_SIZE-1][0]){
			chg_device->cal_capacity = 0;
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
	chg_device->cal_capacity = level;
	DA9052_DEBUG(" TOTAl_BAT_CAPACITY : %d\n", chg_device->cal_capacity);
	return 0;
}

void da9052_bat_tbat_handler(struct da9052_eh_nb *eh_data, unsigned int event)
{
	struct da9052_charger_device *chg_device =
	container_of(eh_data, struct da9052_charger_device, tbat_eh_data);
	
	chg_device->health = POWER_SUPPLY_HEALTH_OVERHEAT;
	
}

static s32 da9052_bat_register_event(struct da9052_charger_device *chg_device)
{
	s32 ret;
	
	if (event_status.da9052_event_tbat == FALSE) {
		chg_device->tbat_eh_data.eve_type = TBAT_EVE;
		chg_device->tbat_eh_data.call_back =da9052_bat_tbat_handler;
		DA9052_DEBUG("events = %d\n",TBAT_EVE);
		ret = chg_device->da9052->register_event_notifier
			(chg_device->da9052, &chg_device->tbat_eh_data);
		if (ret)
			return -EIO;
		event_status.da9052_event_tbat = TRUE;
	}
	
	return 0;
}

static s32 da9052_bat_unregister_event(struct da9052_charger_device *chg_device)
{
	s32 ret;
	
	if (event_status.da9052_event_tbat) {
		ret =
			chg_device->da9052->unregister_event_notifier
				(chg_device->da9052, &chg_device->tbat_eh_data);
		if (ret)
			return -EIO;
		event_status.da9052_event_tbat = FALSE;
	}
	
	return 0;
}

static int da9052_bat_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct da9052_charger_device *chg_device =
	container_of(psy, struct da9052_charger_device, psy);

	/* Validate battery presence */ 
	if( chg_device->illegal &&  psp != POWER_SUPPLY_PROP_PRESENT  ) {
		return -ENODEV;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = chg_device->status;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = (chg_device->charger_type == DA9052_NOCHARGER) ? 0: 1;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = chg_device->illegal;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chg_device->health;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = chg_device->bat_target_voltage * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = chg_device->bat_volt_cutoff * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		val->intval = chg_device->bat_voltage * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = chg_device->chg_current * 1000;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = chg_device->cal_capacity;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = bat_temp_reg_to_C(chg_device->bat_temp);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = chg_device->technology;
		break;
	default:
		return -EINVAL;
	break;
	}
	return 0;
}


static u8 detect_illegal_battery(struct da9052_charger_device *chg_device)
{
	u16 buffer = 0;
	s32  ret = 0;

	/* Measure battery temeperature */
	ret = da9052_bat_get_battery_temperature(chg_device, &buffer);
	if (ret) {
		DA9052_DEBUG("%s: Battery temperature measurement failed \n",
		__func__);
		return ret;
	}

	if (buffer > chg_device->bat_with_no_resistor)
		chg_device->illegal = TRUE;
	else
		chg_device->illegal = FALSE;


	/* suspend charging of battery if illegal battey is detected */
	if (chg_device->illegal)
		da9052_bat_suspend_charging(chg_device);

	return chg_device->illegal;
}

void da9052_update_bat_properties(struct da9052_charger_device *chg_device)
{
	/* Get Bat status and type */
	da9052_bat_status_update(chg_device);
	da9052_bat_level_update(chg_device);
}

static void da9052_bat_external_power_changed(struct power_supply *psy)
{
	struct da9052_charger_device *chg_device =
	container_of(psy, struct da9052_charger_device, psy);

	cancel_delayed_work(&chg_device->monitor_work);
	queue_delayed_work(chg_device->monitor_wqueue, &chg_device->monitor_work, HZ/10);
}


static void da9052_bat_work(struct work_struct *work) 
{
	struct da9052_charger_device *chg_device = container_of(work,
		struct da9052_charger_device,monitor_work.work);
		
	da9052_update_bat_properties(chg_device);
	queue_delayed_work(chg_device->monitor_wqueue, &chg_device->monitor_work, HZ * 8);
}

static enum power_supply_property da9052_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	
};

static s32 __devinit da9052_bat_probe(struct platform_device *pdev)
{
	struct da9052_charger_device *chg_device;
	u8 reg_data;
	int ret;

	chg_device = kzalloc(sizeof(*chg_device), GFP_KERNEL);
	if (!chg_device)
		return -ENOMEM;

	chg_device->da9052 = dev_get_drvdata(pdev->dev.parent);
	platform_set_drvdata(pdev, chg_device);
	
	chg_device->psy.name			= DA9052_BAT_DEVICE_NAME;
	chg_device->psy.type			= POWER_SUPPLY_TYPE_BATTERY;
	chg_device->psy.properties 		= da9052_bat_props;
	chg_device->psy.num_properties 		= ARRAY_SIZE(da9052_bat_props);
	chg_device->psy.get_property 		= da9052_bat_get_property;
	chg_device->psy.external_power_changed 	= da9052_bat_external_power_changed;
	chg_device->psy.use_for_apm 		= 1;
	chg_device->charger_type 		= DA9052_NOCHARGER;
	chg_device->status 			= POWER_SUPPLY_STATUS_UNKNOWN;
	chg_device->health 			= POWER_SUPPLY_HEALTH_UNKNOWN;
	chg_device->technology 			= POWER_SUPPLY_TECHNOLOGY_LION;
	chg_device->bat_with_no_resistor 	= 62;
	chg_device->bat_capacity_limit_low 	= 4;
	chg_device->bat_capacity_limit_high 	= 70;
	chg_device->bat_capacity_full 		= 100;
	chg_device->bat_volt_cutoff 		= 2800;
	chg_device->vbat_first_valid_detect_iteration = 3;
	chg_device->hysteresis_window_size	=1;
	chg_device->chg_hysteresis_const	=89;
	chg_device->hysteresis_reading_interval	=1000;
	chg_device->hysteresis_no_of_reading	=10;

	ret = da9052_read(chg_device->da9052, DA9052_CHGCONT_REG, &reg_data);
	if (ret)
		goto err_charger_init;
	chg_device->charger_voltage_drop = bat_drop_reg_to_mV(reg_data &&
							DA9052_CHGCONT_TCTR);
	chg_device->bat_target_voltage =
			bat_reg_to_mV(reg_data && DA9052_CHGCONT_VCHGBAT);
	
	ret = da9052_read(chg_device->da9052, DA9052_ICHGEND_REG, &reg_data);
	if (ret)
		goto err_charger_init;
	chg_device->chg_end_current = ichg_reg_to_mA(reg_data);
	
	bat_hysteresis.upper_limit 	= 0;
	bat_hysteresis.lower_limit 	= 0;
	bat_hysteresis.hys_flag 	= 0;

	chg_device->illegal 		= FALSE;
	detect_illegal_battery(chg_device);

	da9052_bat_register_event(chg_device);
	if (ret)
		goto err_charger_init;
	
	ret = power_supply_register(&pdev->dev, &chg_device->psy);
	 if (ret)
		goto err_charger_init;
	
	INIT_DELAYED_WORK(&chg_device->monitor_work, da9052_bat_work);
	chg_device->monitor_wqueue = create_singlethread_workqueue(pdev->dev.bus_id);
	if (!chg_device->monitor_wqueue) {
		goto err_charger_init;
	}
	queue_delayed_work(chg_device->monitor_wqueue, &chg_device->monitor_work, HZ * 1);
	
	return 0;

err_charger_init:
	platform_set_drvdata(pdev, NULL);
	kfree(chg_device);
	return ret;
}
static int __devexit da9052_bat_remove(struct platform_device *dev)
{
	struct da9052_charger_device *chg_device = platform_get_drvdata(dev);
	
	/* unregister the events.*/
	da9052_bat_unregister_event(chg_device);
	
	cancel_rearming_delayed_workqueue(chg_device->monitor_wqueue,
					  &chg_device->monitor_work);
	destroy_workqueue(chg_device->monitor_wqueue);
	
	power_supply_unregister(&chg_device->psy);
	
	return 0;
}

static struct platform_driver da9052_bat_driver = {
	.probe		= da9052_bat_probe,
	.remove		= __devexit_p(da9052_bat_remove),
	.driver.name	= DA9052_BAT_DEVICE_NAME,
	.driver.owner	= THIS_MODULE,
};

static int __init da9052_bat_init(void)
{
	printk(banner);
	return platform_driver_register(&da9052_bat_driver);
}

static void __exit da9052_bat_exit(void)
{
	// To remove printk("DA9052: Unregistering BAT device.\n");
	platform_driver_unregister(&da9052_bat_driver);
}

module_init(da9052_bat_init);
module_exit(da9052_bat_exit);

MODULE_AUTHOR("Dialog Semiconductor Ltd");
MODULE_DESCRIPTION("DA9052 BAT Device Driver");
MODULE_LICENSE("GPL");
