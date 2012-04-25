/*
 * max8903_charger.c - Maxim 8903 USB/Adapter Charger Driver
 *
 * Copyright (C) 2011 Samsung Electronics
 * MyungJoo Ham <myungjoo.ham@samsung.com>
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/power/max8903_charger.h>

#define MAX8903_DELAY		(50 * HZ)
struct max8903_data {
	struct max8903_pdata *pdata;
	struct device *dev;
	struct power_supply psy;
	struct power_supply acpsy;
	bool fault;
	bool usb_in;
	bool ta_in;
	int cap;
	struct delayed_work work;
};

static enum power_supply_property max8903_charger_props[] = {
	POWER_SUPPLY_PROP_STATUS, /* Charger status output */
	POWER_SUPPLY_PROP_HEALTH, /* Fault or OK */
	POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property max8903_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE, /* External power source */
};

/* fake capacity */
static inline void get_cap(struct max8903_data *data)
{
	data->cap = 90;
	power_supply_changed(&data->psy);
}

static int max8903_get_ac_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct max8903_data *data = container_of(psy,
			struct max8903_data, acpsy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 0;
		if (data->usb_in || data->ta_in)
			val->intval = 1;
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
			if (gpio_get_value(data->pdata->chg) == 0)
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			else if (data->usb_in || data->ta_in) {
				if (data->cap == 100)
					val->intval = POWER_SUPPLY_STATUS_FULL;
				else
					val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			} else
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		}
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		if (data->fault)
			val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = data->cap; /* fake capacity */
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 0;
		if (data->usb_in || data->ta_in)
			val->intval = 1;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void max8903_work(struct work_struct *work)
{
	struct max8903_data *data = container_of(work,
			struct max8903_data, work.work);
	get_cap(data);
	schedule_delayed_work(&data->work, MAX8903_DELAY);
}

static irqreturn_t max8903_dcin(int irq, void *_data)
{
	struct max8903_data *data = _data;
	struct max8903_pdata *pdata = data->pdata;
	bool ta_in;

	ta_in = gpio_get_value(pdata->dok) ? false : true;

	if (ta_in == data->ta_in)
		return IRQ_HANDLED;

	data->ta_in = ta_in;

	/* Set Current-Limit-Mode 1:DC 0:USB */
	if (pdata->dcm)
		gpio_set_value(pdata->dcm, ta_in ? 1 : 0);

	/* Charger Enable / Disable (cen is negated) */
	if (pdata->cen)
		gpio_set_value(pdata->cen, ta_in ? 0 :
				(data->usb_in ? 0 : 1));

	dev_dbg(data->dev, "TA(DC-IN) Charger %s.\n", ta_in ?
			"Connected" : "Disconnected");

	power_supply_changed(&data->psy);
	power_supply_changed(&data->acpsy);

	return IRQ_HANDLED;
}

static irqreturn_t max8903_usbin(int irq, void *_data)
{
	struct max8903_data *data = _data;
	struct max8903_pdata *pdata = data->pdata;
	bool usb_in;

	usb_in = gpio_get_value(pdata->uok) ? false : true;

	if (usb_in == data->usb_in)
		return IRQ_HANDLED;

	data->usb_in = usb_in;

	/* Do not touch Current-Limit-Mode */

	/* Charger Enable / Disable (cen is negated) */
	if (pdata->cen)
		gpio_set_value(pdata->cen, usb_in ? 0 :
				(data->ta_in ? 0 : 1));

	dev_dbg(data->dev, "USB Charger %s.\n", usb_in ?
			"Connected" : "Disconnected");

	power_supply_changed(&data->psy);
	power_supply_changed(&data->acpsy);

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

static __devinit int max8903_probe(struct platform_device *pdev)
{
	struct max8903_data *data;
	struct max8903_data *ac_data;
	struct device *dev = &pdev->dev;
	struct max8903_pdata *pdata = pdev->dev.platform_data;
	int ret = 0;
	int gpio = 0;
	int ta_in = 0;
	int usb_in = 0;
	int error;

	data = kzalloc(sizeof(struct max8903_data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(dev, "Cannot allocate memory.\n");
		return -ENOMEM;
	}
	data->pdata = pdata;
	data->dev = dev;
	platform_set_drvdata(pdev, data);

	if (pdata->dc_valid == false && pdata->usb_valid == false) {
		dev_err(dev, "No valid power sources.\n");
		ret = -EINVAL;
		goto err;
	}

	if (pdata->dc_valid) {
		if (pdata->dok && gpio_is_valid(pdata->dok) &&
				pdata->dcm && gpio_is_valid(pdata->dcm)) {
			gpio = pdata->dok; /* PULL_UPed Interrupt */
			ta_in = gpio_get_value(gpio) ? 0 : 1;

			gpio = pdata->dcm; /* Output */
			gpio_set_value(gpio, ta_in);
		} else if (pdata->dok && gpio_is_valid(pdata->dok) &&
			   pdata->dcm_always_high) {
			gpio = pdata->dok; /* PULL_UPed Interrupt */

			error = gpio_request(gpio, "chg_dc");
			if (error < 0) {
				dev_err(dev, "failed to configure"
					" request/direction for GPIO %d, error %d\n",
					gpio, error);
				goto err;
			}
			gpio_direction_input(gpio);

			ta_in = gpio_get_value(gpio) ? 0 : 1;

			if (ta_in)
				data->ta_in = true;
			else
				data->ta_in = false;
		} else {
			dev_err(dev, "When DC is wired, DOK and DCM should"
					" be wired as well."
					" or set dcm always high\n");
			ret = -EINVAL;
			goto err;
		}
	} else {
		if (pdata->dcm) {
			if (gpio_is_valid(pdata->dcm))
				gpio_set_value(pdata->dcm, 0);
			else {
				dev_err(dev, "Invalid pin: dcm.\n");
				ret = -EINVAL;
				goto err;
			}
		}
	}

	if (pdata->usb_valid) {
		if (pdata->uok && gpio_is_valid(pdata->uok)) {
			gpio = pdata->uok;
			error = gpio_request(gpio, "chg_usb");
			if (error < 0) {
				dev_err(dev, "failed to configure"
					" request/direction for GPIO %d, error %d\n",
					gpio, error);
				goto err;
			}

			gpio_direction_input(gpio);
			usb_in = gpio_get_value(gpio) ? 0 : 1;
			if (usb_in)
				data->usb_in = true;
			else
				data->usb_in = false;
		} else {
			dev_err(dev, "When USB is wired, UOK should be wired."
					"as well.\n");
			ret = -EINVAL;
			goto err;
		}
	}

	if (pdata->cen) {
		if (gpio_is_valid(pdata->cen)) {
			gpio_set_value(pdata->cen, (ta_in || usb_in) ? 0 : 1);
		} else {
			dev_err(dev, "Invalid pin: cen.\n");
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
		error = gpio_request(pdata->chg, "chg_status");
		if (error < 0) {
			dev_err(dev, "failed to configure"
				" request/direction for GPIO %d, error %d\n",
				pdata->chg, error);
			goto err;
		}
		error = gpio_direction_input(pdata->chg);
	}

	if (pdata->flt) {
		if (!gpio_is_valid(pdata->flt)) {
			dev_err(dev, "Invalid pin: flt.\n");
			ret = -EINVAL;
			goto err;
		}
		error = gpio_request(pdata->flt, "chg_fault");
		if (error < 0) {
			dev_err(dev, "failed to configure"
				" request/direction for GPIO %d, error %d\n",
				pdata->flt, error);
			goto err;
		}
		error = gpio_direction_input(pdata->flt);
	}

	if (pdata->usus) {
		if (!gpio_is_valid(pdata->usus)) {
			dev_err(dev, "Invalid pin: usus.\n");
			ret = -EINVAL;
			goto err;
		}
	}

	data->fault = false;
	data->ta_in = ta_in;
	data->usb_in = usb_in;

	data->acpsy.name = "max8903-ac";
	data->acpsy.type = POWER_SUPPLY_TYPE_MAINS;
	data->acpsy.get_property = max8903_get_ac_property;
	data->acpsy.properties = max8903_ac_props;
	data->acpsy.num_properties = ARRAY_SIZE(max8903_ac_props);

	ret = power_supply_register(dev, &data->acpsy);
	if (ret) {
		dev_err(dev, "failed: power supply register.\n");
		goto err;
	}

	data->psy.name = "max8903-charger";
	data->psy.type = POWER_SUPPLY_TYPE_BATTERY;
	data->psy.get_property = max8903_get_property;
	data->psy.properties = max8903_charger_props;
	data->psy.num_properties = ARRAY_SIZE(max8903_charger_props);

	ret = power_supply_register(dev, &data->psy);
	if (ret) {
		dev_err(dev, "failed: power supply register.\n");
		goto err;
	}

	if (pdata->dc_valid) {
		ret = request_threaded_irq(gpio_to_irq(pdata->dok),
				NULL, max8903_dcin,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"MAX8903 DC IN", data);
		if (ret) {
			dev_err(dev, "Cannot request irq %d for DC (%d)\n",
					gpio_to_irq(pdata->dok), ret);
			goto err_psy;
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
			goto err_usb_irq;
		}
	}
	/* should remove this if capacity is supported */
	data->cap = 90;

	INIT_DELAYED_WORK_DEFERRABLE(&data->work, max8903_work);

	power_supply_changed(&data->psy);
	power_supply_changed(&data->acpsy);
	schedule_delayed_work(&data->work, MAX8903_DELAY);

	return 0;

err_usb_irq:
	if (pdata->usb_valid)
		free_irq(gpio_to_irq(pdata->uok), data);
err_dc_irq:
	if (pdata->dc_valid)
		free_irq(gpio_to_irq(pdata->dok), data);
err_psy:
	power_supply_unregister(&data->psy);
	power_supply_unregister(&data->acpsy);
err:
	if (pdata->uok)
		gpio_free(pdata->uok);
	if (pdata->dok)
		gpio_free(pdata->dok);
	if (pdata->flt)
		gpio_free(pdata->flt);
	if (pdata->chg)
		gpio_free(pdata->chg);
	kfree(data);
	return ret;
}

static __devexit int max8903_remove(struct platform_device *pdev)
{
	struct max8903_data *data = platform_get_drvdata(pdev);

	if (data) {
		struct max8903_pdata *pdata = data->pdata;

		if (pdata->flt) {
			free_irq(gpio_to_irq(pdata->flt), data);
			gpio_free(pdata->flt);
		}
		if (pdata->usb_valid) {
			free_irq(gpio_to_irq(pdata->uok), data);
			gpio_free(pdata->uok);
		}
		if (pdata->dc_valid) {
			free_irq(gpio_to_irq(pdata->dok), data);
			gpio_free(pdata->dok);
		}
		power_supply_unregister(&data->psy);
		power_supply_unregister(&data->acpsy);
		if (pdata->chg)
			gpio_free(pdata->chg);
		kfree(data);
	}

	return 0;
}

static struct platform_driver max8903_driver = {
	.probe	= max8903_probe,
	.remove	= __devexit_p(max8903_remove),
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

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MAX8903 Charger Driver");
MODULE_AUTHOR("MyungJoo Ham <myungjoo.ham@samsung.com>");
MODULE_ALIAS("max8903-charger");
