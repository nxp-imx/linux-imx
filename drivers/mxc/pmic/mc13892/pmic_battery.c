/*
 * Copyright 2009-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * Includes
 */
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <asm/mach-types.h>
#include <linux/pmic_battery.h>
#include <linux/pmic_adc.h>
#include <linux/pmic_status.h>

#define BIT_CHG_VOL_LSH		0
#define BIT_CHG_VOL_WID		3

#define BIT_CHG_CURR_LSH		3
#define BIT_CHG_CURR_WID		4

#define BIT_CHG_PLIM_LSH		15
#define BIT_CHG_PLIM_WID		2

#define BIT_CHG_DETS_LSH 6
#define BIT_CHG_DETS_WID 1
#define BIT_CHG_CURRS_LSH 11
#define BIT_CHG_CURRS_WID 1

#define TRICKLE_CHG_EN_LSH	7
#define	LOW_POWER_BOOT_ACK_LSH	8
#define BAT_TH_CHECK_DIS_LSH	9
#define	BATTFET_CTL_EN_LSH	10
#define BATTFET_CTL_LSH		11
#define	REV_MOD_EN_LSH		13
#define PLIM_DIS_LSH		17
#define	CHG_LED_EN_LSH		18
#define RESTART_CHG_STAT_LSH	20
#define	AUTO_CHG_DIS_LSH	21
#define CYCLING_DIS_LSH		22
#define	VI_PROGRAM_EN_LSH	23

#define TRICKLE_CHG_EN_WID	1
#define	LOW_POWER_BOOT_ACK_WID	1
#define BAT_TH_CHECK_DIS_WID	1
#define	BATTFET_CTL_EN_WID	1
#define BATTFET_CTL_WID		1
#define	REV_MOD_EN_WID		1
#define PLIM_DIS_WID		1
#define	CHG_LED_EN_WID		1
#define RESTART_CHG_STAT_WID	1
#define	AUTO_CHG_DIS_WID	1
#define CYCLING_DIS_WID		1
#define	VI_PROGRAM_EN_WID	1

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
#define BAT_CURRENT_UNIT_UA 5870
#define CHG_VOLTAGE_UINT_UV 23474
#define CHG_MIN_CURRENT_UA 3500

#define COULOMB_TO_UAH(c) (10000 * c / 36)

enum chg_setting {
       TRICKLE_CHG_EN,
       LOW_POWER_BOOT_ACK,
       BAT_TH_CHECK_DIS,
       BATTFET_CTL_EN,
       BATTFET_CTL,
       REV_MOD_EN,
       PLIM_DIS,
       CHG_LED_EN,
       RESTART_CHG_STAT,
       AUTO_CHG_DIS,
       CYCLING_DIS,
       VI_PROGRAM_EN
};

/* Flag used to indicate if Charger workaround is active. */
int chg_wa_is_active;
/* Flag used to indicate if Charger workaround timer is on. */
int chg_wa_timer;
int disable_chg_timer;
struct workqueue_struct *chg_wq;
struct delayed_work chg_work;

static int pmic_set_chg_current(unsigned short curr)
{
	unsigned int mask;
	unsigned int value;

	value = BITFVAL(BIT_CHG_CURR, curr);
	mask = BITFMASK(BIT_CHG_CURR);
	CHECK_ERROR(pmic_write_reg(REG_CHARGE, value, mask));

	return 0;
}

static int pmic_set_chg_misc(enum chg_setting type, unsigned short flag)
{

	unsigned int reg_value = 0;
	unsigned int mask = 0;

	switch (type) {
	case TRICKLE_CHG_EN:
		reg_value = BITFVAL(TRICKLE_CHG_EN, flag);
		mask = BITFMASK(TRICKLE_CHG_EN);
		break;
	case LOW_POWER_BOOT_ACK:
		reg_value = BITFVAL(LOW_POWER_BOOT_ACK, flag);
		mask = BITFMASK(LOW_POWER_BOOT_ACK);
		break;
	case BAT_TH_CHECK_DIS:
		reg_value = BITFVAL(BAT_TH_CHECK_DIS, flag);
		mask = BITFMASK(BAT_TH_CHECK_DIS);
		break;
	case BATTFET_CTL_EN:
		reg_value = BITFVAL(BATTFET_CTL_EN, flag);
		mask = BITFMASK(BATTFET_CTL_EN);
		break;
	case BATTFET_CTL:
		reg_value = BITFVAL(BATTFET_CTL, flag);
		mask = BITFMASK(BATTFET_CTL);
		break;
	case REV_MOD_EN:
		reg_value = BITFVAL(REV_MOD_EN, flag);
		mask = BITFMASK(REV_MOD_EN);
		break;
	case PLIM_DIS:
		reg_value = BITFVAL(PLIM_DIS, flag);
		mask = BITFMASK(PLIM_DIS);
		break;
	case CHG_LED_EN:
		reg_value = BITFVAL(CHG_LED_EN, flag);
		mask = BITFMASK(CHG_LED_EN);
		break;
	case RESTART_CHG_STAT:
		reg_value = BITFVAL(RESTART_CHG_STAT, flag);
		mask = BITFMASK(RESTART_CHG_STAT);
		break;
	case AUTO_CHG_DIS:
		reg_value = BITFVAL(AUTO_CHG_DIS, flag);
		mask = BITFMASK(AUTO_CHG_DIS);
		break;
	case CYCLING_DIS:
		reg_value = BITFVAL(CYCLING_DIS, flag);
		mask = BITFMASK(CYCLING_DIS);
		break;
	case VI_PROGRAM_EN:
		reg_value = BITFVAL(VI_PROGRAM_EN, flag);
		mask = BITFMASK(VI_PROGRAM_EN);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(REG_CHARGE, reg_value, mask));

	return 0;
}

static int pmic_get_batt_voltage(unsigned short *voltage)
{
	t_channel channel;
	unsigned short result[8];

	channel = BATTERY_VOLTAGE;
	CHECK_ERROR(pmic_adc_convert(channel, result));
	*voltage = result[0];

	return 0;
}

static int pmic_get_batt_current(unsigned short *curr)
{
	t_channel channel;
	unsigned short result[8];

	channel = BATTERY_CURRENT;
	CHECK_ERROR(pmic_adc_convert(channel, result));
	*curr = result[0];

	return 0;
}

static int coulomb_counter_calibration;
static unsigned int coulomb_counter_start_time_msecs;

static int pmic_start_coulomb_counter(void)
{
	/* set scaler */
	CHECK_ERROR(pmic_write_reg(REG_ACC1,
		ACC_COULOMB_PER_LSB * ACC_ONEC_VALUE, BITFMASK(ACC1_ONEC)));

	CHECK_ERROR(pmic_write_reg(
		REG_ACC0, ACC_START_COUNTER, ACC_CONTROL_BIT_MASK));
	coulomb_counter_start_time_msecs = jiffies_to_msecs(jiffies);
	pr_debug("coulomb counter start time %u\n",
		coulomb_counter_start_time_msecs);
	return 0;
}

static int pmic_stop_coulomb_counter(void)
{
	CHECK_ERROR(pmic_write_reg(
		REG_ACC0, ACC_STOP_COUNTER, ACC_CONTROL_BIT_MASK));
	return 0;
}

static int pmic_calibrate_coulomb_counter(void)
{
	int ret;
	unsigned int value;

	/* set scaler */
	CHECK_ERROR(pmic_write_reg(REG_ACC1,
		0x1, BITFMASK(ACC1_ONEC)));

	CHECK_ERROR(pmic_write_reg(
		REG_ACC0, ACC_CALIBRATION, ACC_CONTROL_BIT_MASK));
	msleep(ACC_CALIBRATION_DURATION_MSECS);

	ret = pmic_read_reg(REG_ACC0, &value, BITFMASK(ACC_CCOUT));
	if (ret != 0)
		return -1;
	value = BITFEXT(value, ACC_CCOUT);
	pr_debug("calibrate value = %x\n", value);
	coulomb_counter_calibration = (int)((s16)((u16) value));
	pr_debug("coulomb_counter_calibration = %d\n",
		coulomb_counter_calibration);

	return 0;

}

static int pmic_get_charger_coulomb(int *coulomb)
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
	*coulomb = ((s16)((u16)value)) * ACC_COULOMB_PER_LSB;

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

static int pmic_restart_charging(void)
{
	pmic_set_chg_misc(BAT_TH_CHECK_DIS, 1);
	pmic_set_chg_misc(AUTO_CHG_DIS, 0);
	pmic_set_chg_misc(VI_PROGRAM_EN, 1);
	pmic_set_chg_current(0x8);
	pmic_set_chg_misc(RESTART_CHG_STAT, 1);
	pmic_set_chg_misc(PLIM_DIS, 3);
	return 0;
}

struct mc13892_dev_info {
	struct device *dev;

	unsigned short voltage_raw;
	int voltage_uV;
	unsigned short current_raw;
	int current_uA;
	int battery_status;
	int full_counter;
	int charger_online;
	int charger_voltage_uV;
	int accum_current_uAh;

	struct power_supply bat;
	struct power_supply charger;

	struct workqueue_struct *monitor_wqueue;
	struct delayed_work monitor_work;
};

#define mc13892_SENSER	25
#define to_mc13892_dev_info(x) container_of((x), struct mc13892_dev_info, \
					      bat);

static enum power_supply_property mc13892_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_STATUS,
};

static enum power_supply_property mc13892_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int pmic_get_chg_value(unsigned int *value)
{
	t_channel channel;
	unsigned short result[8], max1 = 0, min1 = 0, max2 = 0, min2 = 0, i;
	unsigned int average = 0, average1 = 0, average2 = 0;

	channel = CHARGE_CURRENT;
	CHECK_ERROR(pmic_adc_convert(channel, result));


	for (i = 0; i < 8; i++) {
		if ((result[i] & 0x200) != 0) {
			result[i] = 0x400 - result[i];
			average2 += result[i];
			if ((max2 == 0) || (max2 < result[i]))
				max2 = result[i];
			if ((min2 == 0) || (min2 > result[i]))
				min2 = result[i];
		} else {
			average1 += result[i];
			if ((max1 == 0) || (max1 < result[i]))
				max1 = result[i];
			if ((min1 == 0) || (min1 > result[i]))
				min1 = result[i];
		}
	}

	if (max1 != 0) {
		average1 -= max1;
		if (max2 != 0)
			average2 -= max2;
		else
			average1 -= min1;
	} else
		average2 -= max2 + min2;

	if (average1 >= average2) {
		average = (average1 - average2) / 6;
		*value = average;
	} else {
		average = (average2 - average1) / 6;
		*value = ((~average) + 1) & 0x3FF;
	}

	return 0;
}

static void chg_thread(struct work_struct *work)
{
	int ret;
	unsigned int value = 0;
	int dets;

	if (disable_chg_timer) {
		disable_chg_timer = 0;
		pmic_set_chg_current(0x8);
		queue_delayed_work(chg_wq, &chg_work, 100);
		chg_wa_timer = 1;
		return;
	}

	ret = pmic_read_reg(REG_INT_SENSE0, &value, BITFMASK(BIT_CHG_DETS));

	if (ret == 0) {
		dets = BITFEXT(value, BIT_CHG_DETS);
		pr_debug("dets=%d\n", dets);

		if (dets == 1) {
			pmic_get_chg_value(&value);
			pr_debug("average value=%d\n", value);
			if ((value <= 3) | ((value & 0x200) != 0)) {
				pr_debug("%s: Disable the charger\n", __func__);
				pmic_set_chg_current(0);
				disable_chg_timer = 1;
			}

			queue_delayed_work(chg_wq, &chg_work, 100);
			chg_wa_timer = 1;
		}
	}
}

static int mc13892_charger_update_status(struct mc13892_dev_info *di)
{
	int ret;
	unsigned int value;
	int online;

	ret = pmic_read_reg(REG_INT_SENSE0, &value, BITFMASK(BIT_CHG_DETS));

	if (ret == 0) {
		online = BITFEXT(value, BIT_CHG_DETS);
		if (online != di->charger_online) {
			di->charger_online = online;
			dev_info(di->charger.dev, "charger status: %s\n",
				online ? "online" : "offline");
			power_supply_changed(&di->charger);

			cancel_delayed_work(&di->monitor_work);
			queue_delayed_work(di->monitor_wqueue,
				&di->monitor_work, HZ / 10);
			if (online) {
				pmic_start_coulomb_counter();
				pmic_restart_charging();
				queue_delayed_work(chg_wq, &chg_work, 100);
				chg_wa_timer = 1;
			} else {
				cancel_delayed_work(&chg_work);
				chg_wa_timer = 0;
				pmic_stop_coulomb_counter();
		}
	}
	}

	return ret;
}

static int mc13892_charger_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct mc13892_dev_info *di =
		container_of((psy), struct mc13892_dev_info, charger);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = di->charger_online;
		return 0;
	default:
		break;
	}
	return -EINVAL;
}

static int mc13892_battery_read_status(struct mc13892_dev_info *di)
{
	int retval;
	int coulomb;
	retval = pmic_get_batt_voltage(&(di->voltage_raw));
	if (retval == 0)
		di->voltage_uV = di->voltage_raw * BAT_VOLTAGE_UNIT_UV;

	retval = pmic_get_batt_current(&(di->current_raw));
	if (retval == 0) {
		if (di->current_raw & 0x200)
			di->current_uA =
				(0x1FF - (di->current_raw & 0x1FF)) *
				BAT_CURRENT_UNIT_UA * (-1);
		else
			di->current_uA =
				(di->current_raw & 0x1FF) * BAT_CURRENT_UNIT_UA;
	}
	retval = pmic_get_charger_coulomb(&coulomb);
	if (retval == 0)
		di->accum_current_uAh = COULOMB_TO_UAH(coulomb);

	return retval;
}

static void mc13892_battery_update_status(struct mc13892_dev_info *di)
{
	unsigned int value;
	int retval;
	int old_battery_status = di->battery_status;

	if (di->battery_status == POWER_SUPPLY_STATUS_UNKNOWN)
		di->full_counter = 0;

	if (di->charger_online) {
		retval = pmic_read_reg(REG_INT_SENSE0,
					&value, BITFMASK(BIT_CHG_CURRS));

		if (retval == 0) {
			value = BITFEXT(value, BIT_CHG_CURRS);
			if (value)
				di->battery_status =
					POWER_SUPPLY_STATUS_CHARGING;
			else
				di->battery_status =
					POWER_SUPPLY_STATUS_NOT_CHARGING;
		}

		if (di->battery_status == POWER_SUPPLY_STATUS_NOT_CHARGING)
			di->full_counter++;
		else
			di->full_counter = 0;
	} else {
		di->battery_status = POWER_SUPPLY_STATUS_DISCHARGING;
		di->full_counter = 0;
	}

	dev_dbg(di->bat.dev, "bat status: %d\n",
		di->battery_status);

	if (old_battery_status != POWER_SUPPLY_STATUS_UNKNOWN &&
		di->battery_status != old_battery_status)
		power_supply_changed(&di->bat);
}

static void mc13892_battery_work(struct work_struct *work)
{
	struct mc13892_dev_info *di = container_of(work,
						     struct mc13892_dev_info,
						     monitor_work.work);
	const int interval = HZ * 60;

	dev_dbg(di->dev, "%s\n", __func__);

	mc13892_battery_update_status(di);
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, interval);
}

static void charger_online_event_callback(void *para)
{
	struct mc13892_dev_info *di = (struct mc13892_dev_info *) para;
	pr_info("\n\n DETECTED charger plug/unplug event\n");
	mc13892_charger_update_status(di);
}


static int mc13892_battery_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct mc13892_dev_info *di = to_mc13892_dev_info(psy);
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (di->battery_status == POWER_SUPPLY_STATUS_UNKNOWN) {
			mc13892_charger_update_status(di);
			mc13892_battery_update_status(di);
		}
		val->intval = di->battery_status;
		return 0;
	default:
		break;
	}

	mc13892_battery_read_status(di);

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

static ssize_t chg_wa_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (chg_wa_is_active & chg_wa_timer)
		return sprintf(buf, "Charger LED workaround timer is on\n");
	else
		return sprintf(buf, "Charger LED workaround timer is off\n");
}

static ssize_t chg_wa_enable_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	if (strstr(buf, "1") != NULL) {
		if (chg_wa_is_active) {
			if (chg_wa_timer)
				printk(KERN_INFO "Charger timer is already on\n");
			else {
				queue_delayed_work(chg_wq, &chg_work, 100);
				chg_wa_timer = 1;
				printk(KERN_INFO "Turned on the timer\n");
			}
		}
	} else if (strstr(buf, "0") != NULL) {
		if (chg_wa_is_active) {
			if (chg_wa_timer) {
				cancel_delayed_work(&chg_work);
				chg_wa_timer = 0;
				printk(KERN_INFO "Turned off charger timer\n");
			 } else {
				printk(KERN_INFO "The Charger workaround timer is off\n");
			}
		}
	}

	return size;
}

static DEVICE_ATTR(enable, 0644, chg_wa_enable_show, chg_wa_enable_store);

static int pmic_battery_remove(struct platform_device *pdev)
{
	pmic_event_callback_t bat_event_callback;
	struct mc13892_dev_info *di = platform_get_drvdata(pdev);

	bat_event_callback.func = charger_online_event_callback;
	bat_event_callback.param = (void *) di;
	pmic_event_unsubscribe(EVENT_CHGDETI, bat_event_callback);

	cancel_rearming_delayed_workqueue(di->monitor_wqueue,
					  &di->monitor_work);
	cancel_rearming_delayed_workqueue(chg_wq,
					  &chg_work);
	destroy_workqueue(di->monitor_wqueue);
	destroy_workqueue(chg_wq);
	chg_wa_timer = 0;
	chg_wa_is_active = 0;
	disable_chg_timer = 0;
	power_supply_unregister(&di->bat);
	power_supply_unregister(&di->charger);

	kfree(di);

	return 0;
}

static int pmic_battery_probe(struct platform_device *pdev)
{
	int retval = 0;
	struct mc13892_dev_info *di;
	pmic_event_callback_t bat_event_callback;
	pmic_version_t pmic_version;

	/* Only apply battery driver for MC13892 V2.0 due to ENGR108085 */
	pmic_version = pmic_get_version();
	if (pmic_version.revision < 20) {
		pr_debug("Battery driver is only applied for MC13892 V2.0\n");
		return -1;
	}
	if (machine_is_mx50_arm2()) {
		pr_debug("mc13892 charger is not used for this platform\n");
		return -1;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		retval = -ENOMEM;
		goto di_alloc_failed;
	}

	platform_set_drvdata(pdev, di);

	di->charger.name	= "mc13892_charger";
	di->charger.type = POWER_SUPPLY_TYPE_MAINS;
	di->charger.properties = mc13892_charger_props;
	di->charger.num_properties = ARRAY_SIZE(mc13892_charger_props);
	di->charger.get_property = mc13892_charger_get_property;
	retval = power_supply_register(&pdev->dev, &di->charger);
	if (retval) {
		dev_err(di->dev, "failed to register charger\n");
		goto charger_failed;
	}

	INIT_DELAYED_WORK(&chg_work, chg_thread);
	chg_wq = create_singlethread_workqueue("mxc_chg");
	if (!chg_wq) {
		retval = -ESRCH;
		goto workqueue_failed;
	}

	INIT_DELAYED_WORK(&di->monitor_work, mc13892_battery_work);
	di->monitor_wqueue = create_singlethread_workqueue(dev_name(&pdev->dev));
	if (!di->monitor_wqueue) {
		retval = -ESRCH;
		goto workqueue_failed;
	}
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, HZ * 10);

	di->dev	= &pdev->dev;
	di->bat.name	= "mc13892_bat";
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = mc13892_battery_props;
	di->bat.num_properties = ARRAY_SIZE(mc13892_battery_props);
	di->bat.get_property = mc13892_battery_get_property;
	di->bat.use_for_apm = 1;

	di->battery_status = POWER_SUPPLY_STATUS_UNKNOWN;

	retval = power_supply_register(&pdev->dev, &di->bat);
	if (retval) {
		dev_err(di->dev, "failed to register battery\n");
		goto batt_failed;
	}

	bat_event_callback.func = charger_online_event_callback;
	bat_event_callback.param = (void *) di;
	pmic_event_subscribe(EVENT_CHGDETI, bat_event_callback);
	retval = sysfs_create_file(&pdev->dev.kobj, &dev_attr_enable.attr);

	if (retval) {
		printk(KERN_ERR
		       "Battery: Unable to register sysdev entry for Battery");
		goto workqueue_failed;
	}
	chg_wa_is_active = 1;
	chg_wa_timer = 0;
	disable_chg_timer = 0;

	pmic_stop_coulomb_counter();
	pmic_calibrate_coulomb_counter();
	goto success;

workqueue_failed:
	power_supply_unregister(&di->charger);
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

static struct platform_driver pmic_battery_driver_ldm = {
	.driver = {
		   .name = "pmic_battery",
		   .bus = &platform_bus_type,
		   },
	.probe = pmic_battery_probe,
	.remove = pmic_battery_remove,
};

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

module_init(pmic_battery_init);
module_exit(pmic_battery_exit);

MODULE_DESCRIPTION("pmic_battery driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
