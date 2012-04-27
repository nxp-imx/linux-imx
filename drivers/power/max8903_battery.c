/*
 * max8903_battery.c - Maxim 8903 USB/Adapter Charger Driver
 *
 * Copyright (C) 2011 Samsung Electronics
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc.
 * Based on max8903_charger.c
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/power/max8903_battery.h>
#include <linux/sort.h>


#define	BATTERY_UPDATE_INTERVAL	5 /*seconds*/
#define LOW_VOLT_THRESHOLD	2800000
#define HIGH_VOLT_THRESHOLD	4200000
#define ADC_SAMPLE_COUNT	4

struct max8903_data {
	struct max8903_pdata *pdata;
	struct device *dev;
	struct power_supply psy;
	bool fault;
	bool usb_in;
	bool ta_in;
	bool chg_state;
	struct delayed_work work;
	unsigned int interval;
	unsigned short thermal_raw;
	int voltage_uV;
	int current_uA;
	int battery_status;
	int charger_online;
	int charger_voltage_uV;
	int real_capacity;
	int percent;
	int old_percent;
	struct power_supply bat;
	struct mutex work_lock;
};

typedef struct {
	u32 voltage;
	u32 percent;
} battery_capacity , *pbattery_capacity;

static bool capacity_changed_flag;

static battery_capacity chargingTable[] = {
    {4148, 100},
	{4126,	99},
	{4123,	98},
	{4120,	97},
	{4105,	96},
	{4090,	96},
	{4075,	95},
	{4060,	94},
	{4045,	93},
	{4030,	92},
	{4015,  91},
    {4000,  90},
    {3900,  85},
    {3790,  80},
    {3760,  75},
    {3730,  70},
    {3700,  65},
    {3680,  60},
    {3660,  55},
    {3640,  50},
    {3600,  45},
    {3550,  40},
    {3510,  35},
    {3450,  30},
    {3310,  25},
    {3240,  20},
    {3180,  15},
    {3030,  10},
    {2820,  5},
    {2800,  0},
    {0,  0}
};
static battery_capacity dischargingTable[] = {
    {4100, 100},
	{4090,	99},
	{4080,	98},
	{4060,	97},
	{4040,	96},
	{3920,	96},
	{3900,	95},
	{3970,	94},
	{3940,	93},
	{3910,	92},
	{3890,  91},
    {3860,  90},
    {3790,  85},
    {3690,  80},
    {3660,  75},
    {3630,  70},
    {3600,  65},
    {3580,  60},
    {3560,  55},
    {3540,  50},
    {3500,  45},
    {3450,  40},
    {3410,  35},
    {3350,  30},
    {3310,  25},
    {3240,  20},
    {3180,  15},
    {3030,  10},
    {2820,  5},
    {2800,  0},
    {0,  0}
};

u32 calibrate_battery_capability_percent(struct max8903_data *data)
{
    u8 i;
    pbattery_capacity pTable;
    u32 tableSize;
    if (data->battery_status  == POWER_SUPPLY_STATUS_DISCHARGING) {
		pTable = dischargingTable;
		tableSize = sizeof(dischargingTable)/sizeof(dischargingTable[0]);
    } else {
		pTable = chargingTable;
		tableSize = sizeof(chargingTable)/sizeof(chargingTable[0]);
    }
    for (i = 0; i < tableSize; i++) {
		if (data->voltage_uV >= pTable[i].voltage)
			return	pTable[i].percent;
    }

    return 0;
}

static enum power_supply_property max8903_charger_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
};

static enum power_supply_property max8903_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
};

extern u32 max11801_read_adc(void);

static void max8903_charger_update_status(struct max8903_data *data)
{
	if (data->usb_in || data->ta_in) {
			data->charger_online = 1;
		} else {
			data->charger_online = 0;
		}
	if (data->charger_online == 0) {
			data->battery_status = POWER_SUPPLY_STATUS_DISCHARGING;
		} else {
			if (data->pdata->chg) {
					if (gpio_get_value(data->pdata->chg) == 0) {
						data->battery_status = POWER_SUPPLY_STATUS_CHARGING;
					} else if ((data->usb_in || data->ta_in) &&
						gpio_get_value(data->pdata->chg) == 1) {
						data->battery_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
					}
			}
		}
	pr_debug("chg: %d \n", gpio_get_value(data->pdata->chg));
}
static int cmp_func(const void *_a, const void *_b)
{
	const int *a = _a, *b = _b;

	if (*a > *b)
		return 1;
	if (*a < *b)
		return -1;
	return 0;
}
u32 calibration_voltage(struct max8903_data *data)
{
	int volt[ADC_SAMPLE_COUNT];
	u32 voltage_data;
	int i;
	for (i = 0; i < ADC_SAMPLE_COUNT; i++) {
		if (data->charger_online == 0) {
			/* ADC offset when battery is discharger*/
			volt[i] = max11801_read_adc()-1880;
			} else {
			volt[i] = max11801_read_adc()-1960;
			}
	}
	sort(volt, i, 4, cmp_func, NULL);
	for (i = 0; i < ADC_SAMPLE_COUNT; i++)
		pr_debug("volt_sorted[%2d]: %d\n", i, volt[i]);
	/* get the average of second max/min of remained. */
	voltage_data = (volt[1] + volt[i - 2]) / 2;
	return voltage_data;
}

static void max8903_battery_update_status(struct max8903_data *data)
{
	static int counter;
#if 0
	data->voltage_uV = max11801_read_adc();
#endif
	mutex_lock(&data->work_lock);
	data->voltage_uV = calibration_voltage(data);
	data->percent = calibrate_battery_capability_percent(data);
	if (data->percent != data->old_percent) {
		data->old_percent = data->percent;
		capacity_changed_flag = true;
	}
	if ((capacity_changed_flag == true) && (data->charger_online)) {
		counter++;
		if (counter > 2) {
			counter = 0;
			capacity_changed_flag = false;
			power_supply_changed(&data->bat);
		}
	}
	mutex_unlock(&data->work_lock);
}

static int max8903_battery_get_property(struct power_supply *bat,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct max8903_data *di = container_of(bat,
			struct max8903_data, bat);
	static unsigned long last;
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		if (di->pdata->chg) {
			if (gpio_get_value(di->pdata->chg) == 0) {
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			} else if ((di->usb_in || di->ta_in) && gpio_get_value(di->pdata->chg) == 1) {
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			} else {
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			}
		}
		di->battery_status = val->intval;
		return 0;
	default:
		break;
	}
	if (!last || time_after(jiffies, last + HZ / 2)) {
		last = jiffies;
		max8903_charger_update_status(di);
		max8903_battery_update_status(di);
	}
	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = di->voltage_uV;
		break;
#if 0
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = di->current_uA;
		break;
#endif
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = HIGH_VOLT_THRESHOLD;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = LOW_VOLT_THRESHOLD;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = di->percent < 0 ? 0 :
				(di->percent > 100 ? 100 : di->percent);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 0;
		if (di->usb_in || di->ta_in)
			val->intval = 1;
		di->charger_online = val->intval;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
static int max8903_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct max8903_data *data = container_of(psy,
			struct max8903_data, psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		if (data->pdata->chg) {
			if (gpio_get_value(data->pdata->chg) == 0) {
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
				}
			else if ((data->usb_in || data->ta_in) &&
				gpio_get_value(data->pdata->chg) == 1) {
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			} else {
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			}
		}
		data->battery_status = val->intval;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 0;
		if (data->usb_in || data->ta_in)
			val->intval = 1;
		data->charger_online = val->intval;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		if (data->fault)
			val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static irqreturn_t max8903_dcin(int irq, void *_data)
{
	struct max8903_data *data = _data;
	struct max8903_pdata *pdata = data->pdata;
	bool ta_in;
	enum power_supply_type old_type;

	ta_in = gpio_get_value(pdata->dok) ? false : true;

	if (ta_in == data->ta_in)
		return IRQ_HANDLED;

	data->ta_in = ta_in;
#if 0
	/* Set Current-Limit-Mode 1:DC 0:USB */
	if (pdata->dcm)
		gpio_set_value(pdata->dcm, ta_in ? 1 : 0);
	/* Charger Enable / Disable (cen is negated) */
	if (pdata->cen)
		gpio_set_value(pdata->cen, ta_in ? 0 :
				(data->usb_in ? 0 : 1));
#endif
	pr_info("TA(DC-IN) Charger %s.\n", ta_in ?
			"Connected" : "Disconnected");

	old_type = data->psy.type;

	if (data->ta_in)
		data->psy.type = POWER_SUPPLY_TYPE_MAINS;
	else if (data->usb_in)
		data->psy.type = POWER_SUPPLY_TYPE_USB;
	else
		data->psy.type = POWER_SUPPLY_TYPE_BATTERY;

	if (old_type != data->psy.type) {
		power_supply_changed(&data->psy);
		power_supply_changed(&data->bat);
		}
	return IRQ_HANDLED;
}

static irqreturn_t max8903_usbin(int irq, void *_data)
{
	struct max8903_data *data = _data;
	struct max8903_pdata *pdata = data->pdata;
	bool usb_in;
	enum power_supply_type old_type;

	usb_in = gpio_get_value(pdata->uok) ? false : true;

	if (usb_in == data->usb_in)
		return IRQ_HANDLED;

	data->usb_in = usb_in;

#if 0
	/* Do not touch Current-Limit-Mode */

	/* Charger Enable / Disable (cen is negated) */
	if (pdata->cen)
		gpio_set_value(pdata->cen, usb_in ? 0 :
				(data->ta_in ? 0 : 1));
#endif
	pr_info("USB Charger %s.\n", usb_in ?
			"Connected" : "Disconnected");

	old_type = data->psy.type;

	if (data->ta_in)
		data->psy.type = POWER_SUPPLY_TYPE_MAINS;
	else if (data->usb_in)
		data->psy.type = POWER_SUPPLY_TYPE_USB;
	else
		data->psy.type = POWER_SUPPLY_TYPE_BATTERY;

	if (old_type != data->psy.type) {
		power_supply_changed(&data->psy);
		power_supply_changed(&data->bat);
		}
	return IRQ_HANDLED;
}

static irqreturn_t max8903_fault(int irq, void *_data)
{
	struct max8903_data *data = _data;
	struct max8903_pdata *pdata = data->pdata;
	bool fault;

	fault = gpio_get_value(pdata->flt) ? false : true;

	if (fault == data->fault)
		return IRQ_HANDLED;

	data->fault = fault;

	if (fault)
		dev_err(data->dev, "Charger suffers a fault and stops.\n");
	else
		dev_err(data->dev, "Charger recovered from a fault.\n");
	return IRQ_HANDLED;
}

static irqreturn_t max8903_chg(int irq, void *_data)
{
	struct max8903_data *data = _data;
	struct max8903_pdata *pdata = data->pdata;
	int chg_state;

	chg_state = gpio_get_value(pdata->chg) ? false : true;

	if (chg_state == data->chg_state)
		return IRQ_HANDLED;

	data->chg_state = chg_state;
#if 0
	if (chg_state)
		pr_info("Charger stop.\n ");
	else
		pr_info("Charger start.\n ");
#endif
	return IRQ_HANDLED;
}

static void max8903_battery_work(struct work_struct *work)
{
	struct max8903_data *data;
	data = container_of(work, struct max8903_data, work.work);
	data->interval = HZ * BATTERY_UPDATE_INTERVAL;

	max8903_charger_update_status(data);
	max8903_battery_update_status(data);

	pr_debug("battery voltage: %4d mV\n" , data->voltage_uV);
	pr_debug("charger online status: %d\n" , data->charger_online);
	pr_debug("battery status : %d\n" , data->battery_status);
	pr_debug("battery capacity percent: %3d\n" , data->percent);
	pr_debug("data->usb_in: %x , data->ta_in: %x \n" , data->usb_in, data->ta_in);
	/* reschedule for the next time */
	schedule_delayed_work(&data->work, data->interval);
}
static __devinit int max8903_probe(struct platform_device *pdev)
{
	struct max8903_data *data;
	struct device *dev = &pdev->dev;
	struct max8903_pdata *pdata = pdev->dev.platform_data;
	int ret = 0;
	int gpio = 0;
	int ta_in = 0;
	int usb_in = 0;
	int retval;

	data = kzalloc(sizeof(struct max8903_data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(dev, "Cannot allocate memory.\n");
		return -ENOMEM;
	}
	data->pdata = pdata;
	data->dev = dev;

	platform_set_drvdata(pdev, data);
	capacity_changed_flag = false;
	data->usb_in = 0;
	data->ta_in = 0;

	if (pdata->dc_valid == false && pdata->usb_valid == false) {
		dev_err(dev, "No valid power sources.\n");
		printk(KERN_INFO "No valid power sources.\n");
		ret = -EINVAL;
		goto err;
	}
	if (pdata->dc_valid) {
#if 0
		if (pdata->dok && gpio_is_valid(pdata->dok) &&
				pdata->dcm && gpio_is_valid(pdata->dcm)) {
#endif
		if (pdata->dok && gpio_is_valid(pdata->dok)) {
			gpio = pdata->dok; /* PULL_UPed Interrupt */
			ta_in = gpio_get_value(gpio) ? 0 : 1;
#if 0
			gpio = pdata->dcm; /* Output */
			gpio_set_value(gpio, ta_in);
#endif
		} else if (pdata->dok && gpio_is_valid(pdata->dok) && pdata->dcm_always_high) {
			ta_in = pdata->dok; /* PULL_UPed Interrupt */
			ta_in = gpio_get_value(gpio) ? 0 : 1;
		} else {
			dev_err(dev, "When DC is wired, DOK and DCM should"
					" be wired as well."
					" or set dcm always high\n");
			ret = -EINVAL;
			goto err;
		}
	}
	if (pdata->usb_valid) {
		if (pdata->uok && gpio_is_valid(pdata->uok)) {
			gpio = pdata->uok;
			usb_in = gpio_get_value(gpio) ? 0 : 1;
		} else {
			dev_err(dev, "When USB is wired, UOK should be wired."
					"as well.\n");
			ret = -EINVAL;
			goto err;
		}
	}
	if (pdata->chg) {
		if (!gpio_is_valid(pdata->chg)) {
			dev_err(dev, "Invalid pin: chg.\n");
			ret = -EINVAL;
			goto err;
		}
	}

	if (pdata->flt) {
		if (!gpio_is_valid(pdata->flt)) {
			dev_err(dev, "Invalid pin: flt.\n");
			ret = -EINVAL;
			goto err;
		}
	}

	if (pdata->usus) {
		if (!gpio_is_valid(pdata->usus)) {
			dev_err(dev, "Invalid pin: usus.\n");
			ret = -EINVAL;
			goto err;
		}
	}
	mutex_init(&data->work_lock);
	data->fault = false;
	data->ta_in = ta_in;
	data->usb_in = usb_in;
	data->psy.name = "max8903-ac";
	data->psy.type = (ta_in) ? POWER_SUPPLY_TYPE_MAINS :
			((usb_in) ? POWER_SUPPLY_TYPE_USB :
			 POWER_SUPPLY_TYPE_BATTERY);
	data->psy.get_property = max8903_get_property;
	data->psy.properties = max8903_charger_props;
	data->psy.num_properties = ARRAY_SIZE(max8903_charger_props);
	ret = power_supply_register(dev, &data->psy);
	if (ret) {
		dev_err(dev, "failed: power supply register.\n");
		goto err_psy;
	}
	data->bat.name = "max8903-charger";
	data->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	data->bat.properties = max8903_battery_props;
	data->bat.num_properties = ARRAY_SIZE(max8903_battery_props);
	data->bat.get_property = max8903_battery_get_property;
	data->bat.use_for_apm = 1;
	retval = power_supply_register(&pdev->dev, &data->bat);
	if (retval) {
		dev_err(data->dev, "failed to register battery\n");
		goto battery_failed;
	}
	INIT_DELAYED_WORK(&data->work, max8903_battery_work);
	schedule_delayed_work(&data->work, data->interval);
	if (pdata->dc_valid) {
		ret = request_threaded_irq(gpio_to_irq(pdata->dok),
				NULL, max8903_dcin,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"MAX8903 DC IN", data);
		if (ret) {
			dev_err(dev, "Cannot request irq %d for DC (%d)\n",
					gpio_to_irq(pdata->dok), ret);
			goto err_usb_irq;
		}
	}

	if (pdata->usb_valid) {
		ret = request_threaded_irq(gpio_to_irq(pdata->uok),
				NULL, max8903_usbin,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"MAX8903 USB IN", data);
		if (ret) {
			dev_err(dev, "Cannot request irq %d for USB (%d)\n",
					gpio_to_irq(pdata->uok), ret);
			goto err_dc_irq;
		}
	}

	if (pdata->flt) {
		ret = request_threaded_irq(gpio_to_irq(pdata->flt),
				NULL, max8903_fault,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"MAX8903 Fault", data);
		if (ret) {
			dev_err(dev, "Cannot request irq %d for Fault (%d)\n",
					gpio_to_irq(pdata->flt), ret);
			goto err_flt_irq;
		}
	}

	if (pdata->chg) {
		ret = request_threaded_irq(gpio_to_irq(pdata->chg),
				NULL, max8903_chg,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"MAX8903 Fault", data);
		if (ret) {
			dev_err(dev, "Cannot request irq %d for Fault (%d)\n",
					gpio_to_irq(pdata->flt), ret);
			goto err_chg_irq;
		}
	}
	return 0;
err_psy:
	power_supply_unregister(&data->psy);
battery_failed:
	power_supply_unregister(&data->bat);
err_usb_irq:
	if (pdata->usb_valid)
		free_irq(gpio_to_irq(pdata->uok), data);
	cancel_delayed_work(&data->work);
err_dc_irq:
	if (pdata->dc_valid)
		free_irq(gpio_to_irq(pdata->dok), data);
	cancel_delayed_work(&data->work);
err_flt_irq:
	if (pdata->usb_valid)
		free_irq(gpio_to_irq(pdata->uok), data);
	cancel_delayed_work(&data->work);
err_chg_irq:
	if (pdata->dc_valid)
		free_irq(gpio_to_irq(pdata->dok), data);
	cancel_delayed_work(&data->work);
err:
	kfree(data);
	return ret;
}

static __devexit int max8903_remove(struct platform_device *pdev)
{
	struct max8903_data *data = platform_get_drvdata(pdev);
	if (data) {
		struct max8903_pdata *pdata = data->pdata;
		if (pdata->flt)
			free_irq(gpio_to_irq(pdata->flt), data);
		if (pdata->usb_valid)
			free_irq(gpio_to_irq(pdata->uok), data);
		if (pdata->dc_valid)
			free_irq(gpio_to_irq(pdata->dok), data);
		if (pdata->dc_valid)
			free_irq(gpio_to_irq(pdata->chg), data);
		cancel_delayed_work_sync(&data->work);
		power_supply_unregister(&data->psy);
		power_supply_unregister(&data->bat);
		kfree(data);
	}
	return 0;
}

static int max8903_suspend(struct platform_device *pdev,
				  pm_message_t state)
{
	struct max8903_data *data = platform_get_drvdata(pdev);

	cancel_delayed_work(&data->work);
	return 0;
}

static int max8903_resume(struct platform_device *pdev)
{
	struct max8903_data *data = platform_get_drvdata(pdev);

	schedule_delayed_work(&data->work, BATTERY_UPDATE_INTERVAL);
	return 0;

}

static struct platform_driver max8903_driver = {
	.probe	= max8903_probe,
	.remove	= __devexit_p(max8903_remove),
	.suspend = max8903_suspend,
	.resume = max8903_resume,
	.driver = {
		.name	= "max8903-charger",
		.owner	= THIS_MODULE,
	},
};

static int __init max8903_init(void)
{
	return platform_driver_register(&max8903_driver);
}
module_init(max8903_init);

static void __exit max8903_exit(void)
{
	platform_driver_unregister(&max8903_driver);
}
module_exit(max8903_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MAX8903 Battery Driver");
MODULE_ALIAS("max8903_battery");
