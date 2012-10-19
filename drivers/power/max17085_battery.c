/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * MAX17085 Battery driver
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <mach/gpio.h>

struct max17085_chip {
	struct device *dev;

	/*gpio*/
	int pwr_good;
	int ac_in;
	int charge_now;
	int charge_done;

	struct power_supply	ac;
	struct power_supply	bat;
	struct delayed_work	work;

	int online;
	int health;
	int status;
	int voltage_uV;
	int cap;
};

#define MAX17085_DELAY		3000
#define MAX17085_DEF_TEMP	30

static int max17085_bat_get_mfr(struct power_supply *psy,
		union power_supply_propval *val)
{
	val->strval = "unknown";
	return 0;
}

static int max17085_bat_get_tech(struct power_supply *psy,
		union power_supply_propval *val)
{
	val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
	return 0;
}

static int max17085_bat_get_cap(struct power_supply *psy,
		union power_supply_propval *val)
{
	struct max17085_chip *chip = container_of(psy,
			struct max17085_chip, bat);

	val->intval = chip->cap;
	return 0;
}

static int max17085_bat_get_volt(struct power_supply *psy,
		union power_supply_propval *val)
{
	struct max17085_chip *chip = container_of(psy,
			struct max17085_chip, bat);

	val->intval = chip->voltage_uV;
	return 0;
}

static int max17085_bat_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	int ret = 0;
	struct max17085_chip *chip = container_of(psy,
			struct max17085_chip, bat);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = chip->status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chip->health;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = max17085_bat_get_cap(psy, val);
		if (ret)
			return ret;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = max17085_bat_get_volt(psy, val);
		if (ret)
			return ret;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		ret = max17085_bat_get_mfr(psy, val);
		if (ret)
			return ret;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		ret = max17085_bat_get_tech(psy, val);
		if (ret)
			return ret;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = MAX17085_DEF_TEMP;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int max17085_ac_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct max17085_chip *chip = container_of(psy,
			struct max17085_chip, ac);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->online;
		break;
	default:
		break;
	}

	return 0;
}

static void max17085_get_online(struct max17085_chip *chip)
{
	int level = gpio_get_value(chip->ac_in);
	int cur_online = !level;

	if (chip->online != cur_online) {
		chip->online = cur_online;
		power_supply_changed(&chip->ac);
	}
}

static void max17085_get_health(struct max17085_chip *chip)
{
	int level = gpio_get_value(chip->pwr_good);

	if (level && (chip->voltage_uV >= 0))
		chip->health = POWER_SUPPLY_HEALTH_GOOD;
	else
		chip->health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
}

extern int da9052_adc_read(unsigned char channel);
#define VOLT_REG_TO_MV(val)	((val * 2500) / 1024)
#define BATT_TO_ADC_SCALE	11
#define DA9052_ADCMAN_ADCIN6	6
static void max17085_get_volt(struct max17085_chip *chip)
{
	int val;
	val = da9052_adc_read(DA9052_ADCMAN_ADCIN6);

	/* Ignore lower 2 bits out of 10 bits due to noise */
	val = val & 0x3fc;
	if (val > 0)
		chip->voltage_uV = VOLT_REG_TO_MV(val)*1000*BATT_TO_ADC_SCALE;
	else
		chip->voltage_uV = -1;
}

#define BATT_EMPTY_MV		9955
#define BATT_FULL_MV		12000
static void max17085_get_cap(struct max17085_chip *chip)
{
	int old_cap = chip->cap;

	if (chip->voltage_uV > BATT_EMPTY_MV*1000) {
		chip->cap = (chip->voltage_uV/1000 - BATT_EMPTY_MV) * 100/
				(BATT_FULL_MV - BATT_EMPTY_MV);
		if (chip->cap > 100)
			chip->cap = 100;
	} else
		chip->cap = 0;

	if (chip->cap != old_cap)
		power_supply_changed(&chip->bat);
}

static void max17085_update_status(struct max17085_chip *chip)
{
	int old_status = chip->status;

	if (chip->online) {
		if (chip->voltage_uV/1000 < BATT_FULL_MV)
			chip->status =
				POWER_SUPPLY_STATUS_CHARGING;
		else
			chip->status =
				POWER_SUPPLY_STATUS_NOT_CHARGING;
	} else
		chip->status = POWER_SUPPLY_STATUS_DISCHARGING;

	if (chip->voltage_uV/1000 >= BATT_FULL_MV)
		chip->status = POWER_SUPPLY_STATUS_FULL;

	if (old_status != POWER_SUPPLY_STATUS_UNKNOWN &&
		old_status != chip->status)
		power_supply_changed(&chip->bat);

	if (chip->cap < 20) {
		gpio_set_value(chip->charge_now, 1);
		gpio_set_value(chip->charge_done, 0);
	} else if (chip->cap < 80) {
		gpio_set_value(chip->charge_now, 1);
		gpio_set_value(chip->charge_done, 1);
	} else {
		gpio_set_value(chip->charge_now, 0);
		gpio_set_value(chip->charge_done, 1);
	}
}

static void max17085_work(struct work_struct *work)
{
	struct max17085_chip *chip = container_of(work,
			struct max17085_chip, work.work);

	max17085_get_online(chip);
	max17085_get_volt(chip);
	max17085_get_health(chip);
	max17085_get_cap(chip);
	max17085_update_status(chip);

	schedule_delayed_work(&chip->work, msecs_to_jiffies(MAX17085_DELAY));
}

static enum power_supply_property max17085_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static enum power_supply_property max17085_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int max17085_bat_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct max17085_chip *chip;
	struct resource *res;

	chip = kzalloc(sizeof(struct max17085_chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto chip_alloc_failed;
	}

	res = platform_get_resource_byname(pdev,
			IORESOURCE_IO, "pwr-good");
	if (res == NULL) {
		ret = -EINVAL;
		goto resource_failed;
	}
	chip->pwr_good = res->start;
	res = platform_get_resource_byname(pdev,
			IORESOURCE_IO, "ac-in");
	if (res == NULL) {
		ret = -EINVAL;
		goto resource_failed;
	}
	chip->ac_in = res->start;
	res = platform_get_resource_byname(pdev,
			IORESOURCE_IO, "charge-now");
	if (res == NULL) {
		ret = -EINVAL;
		goto resource_failed;
	}
	chip->charge_now = res->start;
	res = platform_get_resource_byname(pdev,
			IORESOURCE_IO, "charge-done");
	if (res == NULL) {
		ret = -EINVAL;
		goto resource_failed;
	}
	chip->charge_done = res->start;

	chip->dev = &pdev->dev;
	chip->bat.name = "battery";
	chip->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	chip->bat.properties = max17085_bat_props;
	chip->bat.num_properties = ARRAY_SIZE(max17085_bat_props);
	chip->bat.get_property = max17085_bat_get_property;
	chip->bat.use_for_apm = 1;

	chip->ac.name = "ac";
	chip->ac.type = POWER_SUPPLY_TYPE_MAINS;
	chip->ac.properties = max17085_ac_props;
	chip->ac.num_properties = ARRAY_SIZE(max17085_ac_props);
	chip->ac.get_property = max17085_ac_get_property;

	platform_set_drvdata(pdev, chip);

	ret = power_supply_register(&pdev->dev, &chip->ac);
	if (ret) {
		dev_err(chip->dev, "failed to register ac\n");
		goto register_ac_failed;
	}

	ret = power_supply_register(&pdev->dev, &chip->bat);
	if (ret) {
		dev_err(chip->dev, "failed to register battery\n");
		goto register_batt_failed;
	}

	max17085_get_online(chip);

	INIT_DELAYED_WORK_DEFERRABLE(&chip->work, max17085_work);
	schedule_delayed_work(&chip->work, msecs_to_jiffies(MAX17085_DELAY));

	return ret;

register_batt_failed:
	power_supply_unregister(&chip->ac);
register_ac_failed:
resource_failed:
	kfree(chip);
chip_alloc_failed:
	return ret;
}

static int max17085_bat_remove(struct platform_device *pdev)
{
	struct max17085_chip *chip = platform_get_drvdata(pdev);

	cancel_delayed_work(&chip->work);
	power_supply_unregister(&chip->bat);
	power_supply_unregister(&chip->ac);
	kfree(chip);

	return 0;
}

static void max17085_bat_shutdown(struct platform_device *pdev)
{
	struct max17085_chip *chip = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&chip->work);
	gpio_set_value(chip->charge_now, 0);
	gpio_set_value(chip->charge_done, 0);
}

static struct platform_driver max17085_bat_driver = {
	.probe = max17085_bat_probe,
	.remove = max17085_bat_remove,
	.shutdown = max17085_bat_shutdown,
	.driver = {
		.name = "max17085_bat",
	},

};

static int __devinit max17085_bat_init(void)
{
	return platform_driver_register(&max17085_bat_driver);
}

static void __devexit max17085_bat_exit(void)
{
	platform_driver_unregister(&max17085_bat_driver);
}

module_init(max17085_bat_init);
module_exit(max17085_bat_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MAX17085 battery driver");
MODULE_LICENSE("GPL");
