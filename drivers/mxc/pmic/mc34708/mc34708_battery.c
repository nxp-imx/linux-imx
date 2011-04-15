/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

/*
 * Includes
 */
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <asm/mach-types.h>
#include <linux/mfd/mc34708/mc34708_battery.h>
#include <linux/mfd/mc34708/mc34708_adc.h>
#include <linux/pmic_status.h>
#include <linux/pmic_external.h>
#include <linux/mfd/mc34708/mc34708.h>

#define CHRCV_LSH 6
#define CHRCV_WID 6

#define CHRCC_LSH 12
#define CHRCC_WID 4

#define CHRITERM_LSH 16
#define CHRITERM_WID 3

#define USBDETS_LSH 3
#define USBDETS_WID 1

#define AUXDETS_LSH 4
#define AUXDETS_WID 1

#define VBAT_TRKL_LSH 0
#define VBAT_TRKL_WID 2

#define LOWBATT_LSH 4
#define LOWBATT_WID 2

#define BATTEMPH_LSH 21
#define BATTEMPH_WID 2

#define EOCBUCKEN_LSH 23
#define EOCBUCKEN_WID 1

#define BATTEMPL_LSH 19
#define BATTEMPL_WID 2

#define CHRITERMEN_LSH 2
#define CHRITERMEN_WID 1

#define CHREN_LSH 3
#define CHREN_WID 1

#define MUSBCHRG_LSH 13
#define MUSBCHRG_WID 2

#define MSW_LSH 1
#define MSW_WID 1

#define AUXWEAKEN_LSH 22
#define AUXWEAKEN_WID 1

#define VBUSWEAKEN_LSH 23
#define VBUSWEAKEN_WID 1

#define AUXILIM_LSH 11
#define AUXILIM_WID 3

#define ILIM1P5_LSH 20
#define ILIM1P5_WID 1

#define ACC_STARTCC_LSH		0
#define ACC_STARTCC_WID		1
#define ACC_RSTCC_LSH		1
#define ACC_RSTCC_WID		1
#define ACC_CCFAULT_LSH		7
#define ACC_CCFAULT_WID		7
#define ACC_CCOUT_LSH		8
#define ACC_CCOUT_WID		16
#define ACC1_ONEC_LSH		0
#define ACC1_ONEC_WID		15

#define ACC_CALIBRATION 0x17
#define ACC_START_COUNTER 0x07
#define ACC_STOP_COUNTER 0x2
#define ACC_CONTROL_BIT_MASK 0x1f
#define ACC_ONEC_VALUE 2621
#define ACC_COULOMB_PER_LSB 1
#define ACC_CALIBRATION_DURATION_MSECS 20

#define BAT_VOLTAGE_UNIT_UV 4692
#define BAT_CURRENT_UNIT_UA 7813
#define CHG_VOLTAGE_UINT_UV 23474
#define CHG_MIN_CURRENT_UA 3500

#define COULOMB_TO_UAH(c) (10000 * c / 36)

#define BATTOVP_LSH 9
#define BATTOVP_WID 1

#define USBDETM_LSH 3
#define USBDETM_WID 1
#define AUXDETM_LSH 4
#define AUXDETM_WID 1
#define ATTACHM_LSH 15
#define ATTACHM_WID 1
#define DETACHM_LSH 16
#define DETACHM_WID 1
#define ADCCHANGEM_LSH 21
#define ADCCHANGEM_WID 1

#define BATTISOEN_LSH 23
#define BATTISOEN_WID 1

#define USBHOST 0x4
#define USBCHARGER 0x20
#define DEDICATEDCHARGER 0x40

static int suspend_flag;
static int charging_flag;
static int power_change_flag;

/*
usb_type = 0x4; USB host;
usb_type = 0x20; USB charger;
usb_type = 0x40; Dedicated charger;
*/
static int usb_type;
static struct mc34708_charger_setting_point ripley_charger_setting_point[] = {
	{
	 .microVolt = 4200000,
	 .microAmp = 1150000,
	 },
};

static struct mc34708_charger_config ripley_charge_config = {
	.batteryTempLow = 0,
	.batteryTempHigh = 0,
	.hasTempSensor = 0,
	.trickleThreshold = 3000000,
	.vbusThresholdLow = 4600000,
	.vbusThresholdWeak = 4800000,
	.vbusThresholdHigh = 5000000,
	.vauxThresholdLow = 4600000,
	.vauxThresholdWeak = 4800000,
	.vauxThresholdHigh = 5000000,
	.lowBattThreshold = 3100000,
	.toppingOffMicroAmp = 50000,	/* 50mA */
	.chargingPoints = ripley_charger_setting_point,
	.pointsNumber = 1,
};

static int dump_ripley_register(int reg);

static int enable_charger(int enable)
{
	charging_flag = enable ? 1 : 0;
	CHECK_ERROR(pmic_write_reg(MC34708_REG_BATTERY_PROFILE,
				   BITFVAL(CHREN, enable ? 1 : 0),
				   BITFMASK(CHREN)));
	return 0;
}

static int ripley_get_batt_voltage(unsigned short *voltage)
{
	t_channel channel;
	unsigned short result[8];

	channel = BATTERY_VOLTAGE;
	CHECK_ERROR(mc34708_pmic_adc_convert(channel, result));
	*voltage = result[0];

	return 0;
}

static int ripley_get_batt_current(unsigned short *curr)
{
	t_channel channel;
	unsigned short result[8];

	channel = BATTERY_CURRENT;
	CHECK_ERROR(mc34708_pmic_adc_convert(channel, result));
	*curr = result[0];

	return 0;
}

static int coulomb_counter_calibration;
static unsigned int coulomb_counter_start_time_msecs;

static int ripley_start_coulomb_counter(void)
{
	/* set scaler */
	CHECK_ERROR(pmic_write_reg(REG_ACC1,
				   ACC_COULOMB_PER_LSB * ACC_ONEC_VALUE,
				   BITFMASK(ACC1_ONEC)));

	CHECK_ERROR(pmic_write_reg
		    (REG_ACC0, ACC_START_COUNTER, ACC_CONTROL_BIT_MASK));
	coulomb_counter_start_time_msecs = jiffies_to_msecs(jiffies);
	pr_debug("coulomb counter start time %u\n",
		 coulomb_counter_start_time_msecs);
	return 0;
}

static int ripley_stop_coulomb_counter(void)
{
	CHECK_ERROR(pmic_write_reg
		    (REG_ACC0, ACC_STOP_COUNTER, ACC_CONTROL_BIT_MASK));
	return 0;
}

static int ripley_calibrate_coulomb_counter(void)
{
	int ret;
	unsigned int value;

	/* set scaler */
	CHECK_ERROR(pmic_write_reg(REG_ACC1, 0x1, BITFMASK(ACC1_ONEC)));

	CHECK_ERROR(pmic_write_reg
		    (REG_ACC0, ACC_CALIBRATION, ACC_CONTROL_BIT_MASK));
	msleep(ACC_CALIBRATION_DURATION_MSECS);

	ret = pmic_read_reg(REG_ACC0, &value, BITFMASK(ACC_CCOUT));
	if (ret != 0)
		return -1;
	value = BITFEXT(value, ACC_CCOUT);
	pr_debug("calibrate value = %x\n", value);
	coulomb_counter_calibration = (int)((s16) ((u16) value));
	pr_debug("coulomb_counter_calibration = %d\n",
		 coulomb_counter_calibration);

	return 0;

}

static int ripley_get_charger_coulomb(int *coulomb)
{
	int ret;
	unsigned int value;
	int calibration;
	unsigned int time_diff_msec;

	ret = pmic_read_reg(REG_ACC0, &value, BITFMASK(ACC_CCOUT));
	if (ret != 0)
		return -1;
	value = BITFEXT(value, ACC_CCOUT);
	pr_debug("counter value = %x\n", value);
	*coulomb = ((s16) ((u16) value)) * ACC_COULOMB_PER_LSB;

	if (abs(*coulomb) >= ACC_COULOMB_PER_LSB) {
		/* calibrate */
		time_diff_msec = jiffies_to_msecs(jiffies);
		time_diff_msec =
		    (time_diff_msec > coulomb_counter_start_time_msecs) ?
		    (time_diff_msec - coulomb_counter_start_time_msecs) :
		    (0xffffffff - coulomb_counter_start_time_msecs
		     + time_diff_msec);
		calibration = coulomb_counter_calibration * (int)time_diff_msec
		    / (ACC_ONEC_VALUE * ACC_CALIBRATION_DURATION_MSECS);
		*coulomb -= calibration;
	}

	return 0;
}

struct ripley_dev_info {
	struct device *dev;

	unsigned short voltage_raw;
	int voltage_uV;
	unsigned short current_raw;
	int current_uA;
	int battery_status;
	int full_counter;
	int chargeTimeSeconds;
	int usb_charger_online;
	int aux_charger_online;
	int charger_voltage_uV;
	int accum_current_uAh;

	int currChargePoint;

	struct power_supply bat;
	struct power_supply usb_charger;
	struct power_supply aux_charger;

	struct workqueue_struct *monitor_wqueue;
	struct delayed_work monitor_work;
	struct mc34708_charger_config *chargeConfig;
};

#define ripley_SENSER	25
#define to_ripley_dev_info(x) container_of((x), struct ripley_dev_info, \
					      bat);

static enum power_supply_property ripley_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_STATUS,
};

static enum power_supply_property ripley_aux_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property ripley_usb_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

#define BATTTEMPH_TO_BITS(temp)	((temp - 45) / 5)
#define BATTTEMPL_TO_BITS(temp)	(temp / 5)
#define VBAT_TRKL_UV_TO_BITS(uv)	((uv-2800000) / 100000)
#define LOWBATT_UV_TO_BITS(uv)	((uv - 3100000) / 100000)
#define CHRITEM_UV_TO_BITS(uv)	(((uv / 1000) - 50) / 50)

static int dump_ripley_register(int reg)
{
	unsigned int value;
	pmic_read_reg(reg, &value, PMIC_ALL_BITS);
	pr_info("ripley reg %d = 0x%x\n", reg, value);
	return 0;
}

static int init_charger(struct mc34708_charger_config *config)
{
	/* set charger current termination threshold */
	CHECK_ERROR(pmic_write_reg(MC34708_REG_BATTERY_PROFILE,
				   BITFVAL(CHRITERM,
					   CHRITEM_UV_TO_BITS
					   (config->toppingOffMicroAmp)),
				   BITFMASK(CHRITERM)));
	/* enable charger current termination */
	CHECK_ERROR(pmic_write_reg(MC34708_REG_BATTERY_PROFILE,
				   BITFVAL(CHRITERMEN, 1),
				   BITFMASK(CHRITERMEN)));

	/* enable EOC buck */
	CHECK_ERROR(pmic_write_reg(MC34708_REG_BATTERY_PROFILE,
				   BITFVAL(EOCBUCKEN, 1), BITFMASK(EOCBUCKEN)));
	/* disable manual switch */
	CHECK_ERROR(pmic_write_reg(MC34708_REG_USB_CTL,
				   BITFVAL(MSW, 0), BITFMASK(MSW)));
	/* enable 1P5 large current */
	CHECK_ERROR(pmic_write_reg(MC34708_REG_CHARGER_DEBOUNCE,
				   BITFVAL(ILIM1P5, 1), BITFMASK(ILIM1P5)));

	/* enable ISO */
	CHECK_ERROR(pmic_write_reg(MC34708_REG_CHARGER_DEBOUNCE,
				   BITFVAL(BATTISOEN, 1), BITFMASK(BATTISOEN)));

	CHECK_ERROR(pmic_write_reg(MC34708_REG_CHARGER_SOURCE,
				   BITFVAL(AUXWEAKEN, 1), BITFMASK(AUXWEAKEN)));

	CHECK_ERROR(pmic_write_reg(MC34708_REG_CHARGER_SOURCE,
				   BITFVAL(VBUSWEAKEN, 1),
				   BITFMASK(VBUSWEAKEN)));

	CHECK_ERROR(pmic_write_reg(MC34708_REG_BATTERY_PROFILE,
				   BITFVAL(VBAT_TRKL,
					   VBAT_TRKL_UV_TO_BITS
					   (config->trickleThreshold)),
				   BITFMASK(VBAT_TRKL)));
	CHECK_ERROR(pmic_write_reg
		    (MC34708_REG_BATTERY_PROFILE,
		     BITFVAL(LOWBATT,
			     LOWBATT_UV_TO_BITS(config->lowBattThreshold)),
		     BITFMASK(LOWBATT)));

	if (config->hasTempSensor) {
		CHECK_ERROR(pmic_write_reg(MC34708_REG_BATTERY_PROFILE,
					   BITFVAL(BATTEMPH,
						   BATTTEMPH_TO_BITS
						   (config->batteryTempHigh)),
					   BITFMASK(BATTEMPH)));
		CHECK_ERROR(pmic_write_reg
			    (MC34708_REG_BATTERY_PROFILE,
			     BITFVAL(BATTEMPL,
				     BATTTEMPL_TO_BITS(config->batteryTempLow)),
			     BITFMASK(BATTEMPL)));
	}

	return 0;
}

#define CHRCV_UV_TO_BITS(uv)	((uv - 3500000) / 20000)
#define CHRCC_UA_TO_BITS(ua)	((ua - 250000) / 100000)

static int set_charging_point(struct ripley_dev_info *di, int point)
{
	unsigned int val, mask;
	if (point >= 0 && point < di->chargeConfig->pointsNumber) {

		val =
		    BITFVAL(CHRCV,
			    CHRCV_UV_TO_BITS(di->
					     chargeConfig->chargingPoints
					     [point].microVolt));
		switch (usb_type) {
		case USBHOST:
			/* set current limit to 500mA */
			CHECK_ERROR(pmic_write_reg(MC34708_REG_USB_CTL,
						   BITFVAL(MUSBCHRG, 1),
						   BITFMASK(MUSBCHRG)));
			val |= BITFVAL(CHRCC, CHRCC_UA_TO_BITS(250000));
			break;
		case USBCHARGER:
			/* set current limit to 950mA */
			CHECK_ERROR(pmic_write_reg(MC34708_REG_USB_CTL,
						   BITFVAL(MUSBCHRG, 3),
						   BITFMASK(MUSBCHRG)));
			val |= BITFVAL(CHRCC, CHRCC_UA_TO_BITS(350000));
			break;
		case DEDICATEDCHARGER:
			/* set current limit to 950mA */
			CHECK_ERROR(pmic_write_reg(MC34708_REG_USB_CTL,
						   BITFVAL(MUSBCHRG, 3),
						   BITFMASK(MUSBCHRG)));
			val |= BITFVAL(CHRCC, CHRCC_UA_TO_BITS(350000));
			break;
		default:
			val |=
			    BITFVAL(CHRCC,
				    CHRCC_UA_TO_BITS(di->
						     chargeConfig->chargingPoints
						     [point].microAmp));
			break;
		}

		mask = BITFMASK(CHRCV) | BITFMASK(CHRCC);
		CHECK_ERROR(pmic_write_reg(MC34708_REG_BATTERY_PROFILE,
					   val, mask));

		di->currChargePoint = point;
		return 0;
	}
	return -EINVAL;
}

#define  FULL_DEBOUNCE_TIME 3
static int adjust_charging_parameter(struct mc34708_charger_config **config)
{
	struct ripley_dev_info *di =
	    container_of((config), struct ripley_dev_info, chargeConfig);

	if (di->voltage_uV >=
	    (*config)->chargingPoints[di->currChargePoint].microVolt) {

		/* for NOT the final charge point */
		if (di->currChargePoint < (*config)->pointsNumber - 1) {
			di->full_counter++;
			if (di->full_counter > FULL_DEBOUNCE_TIME) {
				set_charging_point(di, di->currChargePoint + 1);
				pr_info
				    ("shift to charge point %d, volt=%d uV, curr=%d uA\n",
				     di->currChargePoint,
				     (*config)->
				     chargingPoints
				     [di->currChargePoint].microVolt,
				     (*config)->
				     chargingPoints
				     [di->currChargePoint].microAmp);
			}
		} else {
			if (di->current_uA <= (*config)->toppingOffMicroAmp)
				di->full_counter++;
			if (di->full_counter > FULL_DEBOUNCE_TIME)
				di->battery_status =
				    POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
	}

	return 0;
}

static int ripley_charger_update_status(struct ripley_dev_info *di)
{
	int ret;
	unsigned int value;
	unsigned int reg_usb_type;
	int usbOnline, auxOnline;
	int restartCharging = 0;
	int stopCharging = 0;

	ret = pmic_read_reg(MC34708_REG_INT_SENSE0, &value, PMIC_ALL_BITS);

	if (ret == 0) {
		usbOnline = BITFEXT(value, USBDETS);
		auxOnline = BITFEXT(value, AUXDETS);
		if (!(di->aux_charger_online || di->usb_charger_online) &&
		    (usbOnline || auxOnline))
			restartCharging = 1;
		if ((di->aux_charger_online || di->usb_charger_online) &&
		    !(usbOnline || auxOnline))
			stopCharging = 1;

		if (auxOnline != di->aux_charger_online) {
			msleep(500);
			di->aux_charger_online = auxOnline;
			dev_info(di->aux_charger.dev,
				 "aux charger status: %s\n",
				 auxOnline ? "online" : "offline");
			power_supply_changed(&di->aux_charger);
		}
		if (usbOnline != di->usb_charger_online) {
			/* need some delay to know the usb type */
			msleep(800);
			ret = pmic_read_reg(MC34708_REG_USB_DEVICE_TYPE,
					    &reg_usb_type, PMIC_ALL_BITS);
			usb_type = 0;
			if ((reg_usb_type & USBHOST) != 0) {
				usb_type = USBHOST;
				pr_info("USB host attached!!!\n");
			}
			if ((reg_usb_type & USBCHARGER) != 0) {
				usb_type = USBCHARGER;
				pr_info("USB charger attached!!!\n");
			}
			if ((reg_usb_type & DEDICATEDCHARGER) != 0) {
				usb_type = DEDICATEDCHARGER;
				pr_info("Dedicated charger attached!!!\n");
			}
			di->usb_charger_online = usbOnline;
			dev_info(di->usb_charger.dev, "usb cable status: %s\n",
				 usbOnline ? "online" : "offline");
			power_supply_changed(&di->usb_charger);
		}

		if (restartCharging) {
			ripley_start_coulomb_counter();
			enable_charger(1);
			cancel_delayed_work(&di->monitor_work);
			queue_delayed_work(di->monitor_wqueue,
					   &di->monitor_work, HZ / 10);
		} else if (stopCharging) {
			ripley_stop_coulomb_counter();
			cancel_delayed_work(&di->monitor_work);
			queue_delayed_work(di->monitor_wqueue,
					   &di->monitor_work, HZ / 10);
		}
	}
	power_change_flag = 1;

	return ret;
}

static int ripley_aux_charger_get_property(struct power_supply *psy,
					   enum power_supply_property psp,
					   union power_supply_propval *val)
{
	struct ripley_dev_info *di =
	    container_of((psy), struct ripley_dev_info, aux_charger);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = di->aux_charger_online;
		return 0;
	default:
		break;
	}

	return -EINVAL;
}

static int ripley_usb_charger_get_property(struct power_supply *psy,
					   enum power_supply_property psp,
					   union power_supply_propval *val)
{
	struct ripley_dev_info *di =
	    container_of((psy), struct ripley_dev_info, usb_charger);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = di->usb_charger_online;
		return 0;
	default:
		break;
	}

	return -EINVAL;
}

static int ripley_battery_read_status(struct ripley_dev_info *di)
{
	int retval;
	int coulomb;
	retval = ripley_get_batt_voltage(&(di->voltage_raw));
	if (retval == 0)
		di->voltage_uV = di->voltage_raw * BAT_VOLTAGE_UNIT_UV;

	retval = ripley_get_batt_current(&(di->current_raw));
	if (retval == 0) {
		if (di->current_raw & 0x200)
			di->current_uA =
			    (0x1FF - (di->current_raw & 0x1FF)) *
			    BAT_CURRENT_UNIT_UA * (-1);
		else
			di->current_uA =
			    (di->current_raw & 0x1FF) * BAT_CURRENT_UNIT_UA;
	}
	retval = ripley_get_charger_coulomb(&coulomb);
	if (retval == 0)
		di->accum_current_uAh = COULOMB_TO_UAH(coulomb);
	return retval;
}

static void ripley_battery_update_status(struct ripley_dev_info *di)
{
	unsigned int point = 0;
	struct mc34708_charger_config *config = di->chargeConfig;
	int old_battery_status = di->battery_status;

	if (di->battery_status == POWER_SUPPLY_STATUS_UNKNOWN)
		di->full_counter = 0;

	ripley_battery_read_status(di);

	if (di->usb_charger_online || di->aux_charger_online) {
		if (di->battery_status == POWER_SUPPLY_STATUS_UNKNOWN ||
		    di->battery_status == POWER_SUPPLY_STATUS_DISCHARGING) {
			point = 0;
			init_charger(config);
			set_charging_point(di, point);
			enable_charger(1);
		} else if (di->battery_status == POWER_SUPPLY_STATUS_CHARGING)
			adjust_charging_parameter(&(di->chargeConfig));
		if (di->full_counter > FULL_DEBOUNCE_TIME &&
		    di->currChargePoint >= di->chargeConfig->pointsNumber) {
			di->battery_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		} else
			di->battery_status = POWER_SUPPLY_STATUS_CHARGING;
	} else {
		di->battery_status = POWER_SUPPLY_STATUS_DISCHARGING;
		di->full_counter = 0;
	}

	if (power_change_flag) {
		pr_info("battery status: %d, old status: %d, point: %d\n",
			di->battery_status, old_battery_status,
			di->currChargePoint);
		power_change_flag = 0;
	}
	dev_dbg(di->bat.dev, "bat status: %d\n", di->battery_status);
	if (old_battery_status != POWER_SUPPLY_STATUS_UNKNOWN &&
	    di->battery_status != old_battery_status)
		power_supply_changed(&di->bat);
}

static void ripley_battery_work(struct work_struct *work)
{
	struct ripley_dev_info *di = container_of(work,
						  struct ripley_dev_info,
						  monitor_work.work);
	const int interval = HZ * 15;

	dev_dbg(di->dev, "%s\n", __func__);

	ripley_battery_update_status(di);
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, interval);
}

static void usb_charger_online_event_callback(void *para)
{
	struct ripley_dev_info *di = (struct ripley_dev_info *)para;
	ripley_charger_update_status(di);
}

static void aux_charger_online_event_callback(void *para)
{
	struct ripley_dev_info *di = (struct ripley_dev_info *)para;
	ripley_charger_update_status(di);
}

static void usb_over_voltage_event_callback(void *para)
{
	struct ripley_dev_info *di = (struct ripley_dev_info *)para;
	ripley_charger_update_status(di);
}

static void aux_over_voltage_event_callback(void *para)
{
	struct ripley_dev_info *di = (struct ripley_dev_info *)para;
	ripley_charger_update_status(di);
}

static void battery_over_voltage_event_callback(void *para)
{
	struct ripley_dev_info *di = (struct ripley_dev_info *)para;
	ripley_charger_update_status(di);
}

static void battery_over_temp_event_callback(void *para)
{
	struct ripley_dev_info *di = (struct ripley_dev_info *)para;
	ripley_charger_update_status(di);
}

static void battery_charge_complete_event_callback(void *para)
{
	struct ripley_dev_info *di = (struct ripley_dev_info *)para;
	pr_info("\n\n battery charge complete event, disable charging\n");
}

static int ripley_battery_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct ripley_dev_info *di = to_ripley_dev_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (di->battery_status == POWER_SUPPLY_STATUS_UNKNOWN) {
			ripley_charger_update_status(di);
			ripley_battery_update_status(di);
		}
		val->intval = di->battery_status;
		return 0;
	default:
		break;
	}

	ripley_battery_read_status(di);

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = di->voltage_uV;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = di->current_uA;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = di->accum_current_uAh;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = 3800000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = 3300000;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ripley_battery_remove(struct platform_device *pdev)
{
	pmic_event_callback_t bat_event_callback;
	struct ripley_dev_info *di = platform_get_drvdata(pdev);

	bat_event_callback.func = usb_charger_online_event_callback;
	bat_event_callback.param = (void *)di;
	pmic_event_unsubscribe(MC34708_EVENT_USBDET, bat_event_callback);
	bat_event_callback.func = aux_charger_online_event_callback;
	bat_event_callback.param = (void *)di;
	pmic_event_unsubscribe(MC34708_EVENT_AUXDET, bat_event_callback);

	cancel_rearming_delayed_workqueue(di->monitor_wqueue,
					  &di->monitor_work);
	destroy_workqueue(di->monitor_wqueue);
	power_supply_unregister(&di->bat);
	power_supply_unregister(&di->usb_charger);
	power_supply_unregister(&di->aux_charger);

	kfree(di);

	return 0;
}

static int ripley_battery_probe(struct platform_device *pdev)
{
	int retval = 0;
	struct ripley_dev_info *di;
	pmic_event_callback_t bat_event_callback;
	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		retval = -ENOMEM;
		goto di_alloc_failed;
	}

	platform_set_drvdata(pdev, di);

	di->aux_charger.name = "ripley_aux_charger";
	di->aux_charger.type = POWER_SUPPLY_TYPE_MAINS;
	di->aux_charger.properties = ripley_aux_charger_props;
	di->aux_charger.num_properties = ARRAY_SIZE(ripley_aux_charger_props);
	di->aux_charger.get_property = ripley_aux_charger_get_property;
	retval = power_supply_register(&pdev->dev, &di->aux_charger);
	if (retval) {
		dev_err(di->dev, "failed to register charger\n");
		goto charger_failed;
	}

	di->usb_charger.name = "ripley_usb_charger";
	di->usb_charger.type = POWER_SUPPLY_TYPE_USB;
	di->usb_charger.properties = ripley_usb_charger_props;
	di->usb_charger.num_properties = ARRAY_SIZE(ripley_usb_charger_props);
	di->usb_charger.get_property = ripley_usb_charger_get_property;
	retval = power_supply_register(&pdev->dev, &di->usb_charger);
	if (retval) {
		dev_err(di->dev, "failed to register charger\n");
		goto charger_failed;
	}

	INIT_DELAYED_WORK(&di->monitor_work, ripley_battery_work);
	di->monitor_wqueue =
	    create_singlethread_workqueue(dev_name(&pdev->dev));
	if (!di->monitor_wqueue) {
		retval = -ESRCH;
		goto workqueue_failed;
	}

	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, HZ * 10);

	di->dev = &pdev->dev;
	di->chargeConfig = &ripley_charge_config;
	di->bat.name = "ripley_bat";
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = ripley_battery_props;
	di->bat.num_properties = ARRAY_SIZE(ripley_battery_props);
	di->bat.get_property = ripley_battery_get_property;
	di->bat.use_for_apm = 1;

	di->battery_status = POWER_SUPPLY_STATUS_UNKNOWN;

	retval = power_supply_register(&pdev->dev, &di->bat);
	if (retval) {
		dev_err(di->dev, "failed to register battery\n");
		goto batt_failed;
	}

	bat_event_callback.func = usb_over_voltage_event_callback;
	bat_event_callback.param = (void *)di;
	pmic_event_subscribe(MC34708_EVENT_USBOVP, bat_event_callback);

	bat_event_callback.func = aux_over_voltage_event_callback;
	bat_event_callback.param = (void *)di;
	pmic_event_subscribe(MC34708_EVENT_AUXOVP, bat_event_callback);

	bat_event_callback.func = usb_charger_online_event_callback;
	bat_event_callback.param = (void *)di;
	pmic_event_subscribe(MC34708_EVENT_USBDET, bat_event_callback);

	bat_event_callback.func = aux_charger_online_event_callback;
	bat_event_callback.param = (void *)di;
	pmic_event_subscribe(MC34708_EVENT_AUXDET, bat_event_callback);

	bat_event_callback.func = battery_over_voltage_event_callback;
	bat_event_callback.param = (void *)di;
	pmic_event_subscribe(MC34708_EVENT_BATTOVP, bat_event_callback);

	bat_event_callback.func = battery_over_temp_event_callback;
	bat_event_callback.param = (void *)di;
	pmic_event_subscribe(MC34708_EVENT_BATTOTP, bat_event_callback);

	bat_event_callback.func = battery_charge_complete_event_callback;
	bat_event_callback.param = (void *)di;
	pmic_event_subscribe(MC34708_EVENT_CHRCMPL, bat_event_callback);

	ripley_stop_coulomb_counter();
	ripley_calibrate_coulomb_counter();

	goto success;

workqueue_failed:
	power_supply_unregister(&di->aux_charger);
	power_supply_unregister(&di->usb_charger);
charger_failed:
	power_supply_unregister(&di->bat);
batt_failed:
	kfree(di);
di_alloc_failed:
success:
	dev_dbg(di->dev, "%s battery probed!\n", __func__);
	return retval;

	return 0;
}

static int ripley_battery_suspend(struct platform_device *pdev,
				  pm_message_t state)
{
	suspend_flag = 1;
	CHECK_ERROR(pmic_write_reg
		    (MC34708_REG_INT_STATUS0, BITFVAL(BATTOVP, 0),
		     BITFMASK(BATTOVP)));
	CHECK_ERROR(pmic_write_reg
		    (MC34708_REG_INT_MASK0, BITFVAL(BATTOVP, 1),
		     BITFMASK(BATTOVP)));

	return 0;
};

static int ripley_battery_resume(struct platform_device *pdev)
{
	suspend_flag = 0;
	CHECK_ERROR(pmic_write_reg
		    (MC34708_REG_INT_MASK0, BITFVAL(BATTOVP, 0),
		     BITFMASK(BATTOVP)));

	return 0;
};

static struct platform_driver ripley_battery_driver_ldm = {
	.driver = {
		   .name = "mc34708_battery",
		   .bus = &platform_bus_type,
		   },
	.probe = ripley_battery_probe,
	.remove = ripley_battery_remove,
	.suspend = ripley_battery_suspend,
	.resume = ripley_battery_resume,
};

static int __init ripley_battery_init(void)
{
	pr_debug("Ripley Battery driver loading...\n");
	return platform_driver_register(&ripley_battery_driver_ldm);
}

static void __exit ripley_battery_exit(void)
{
	platform_driver_unregister(&ripley_battery_driver_ldm);
	pr_debug("Ripley Battery driver successfully unloaded\n");
}

module_init(ripley_battery_init);
module_exit(ripley_battery_exit);

MODULE_DESCRIPTION("ripley_battery driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
