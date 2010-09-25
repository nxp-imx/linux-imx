/*
* Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
*/

/*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.

* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.

* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/idr.h>
#include <linux/power_supply.h>

#include "../w1/w1.h"
#include "../w1/slaves/w1_ds2438.h"

struct ds2438_device_info {
	struct device *dev;
	/* DS2438 data, valid after calling ds2438_battery_read_status() */
	unsigned long update_time;	/* jiffies when data read */
	char raw[DS2438_PAGE_SIZE];	/* raw DS2438 data */
	int voltage_uV;
	int current_uA;
	int accum_current_uAh;
	int temp_C;
	int charge_status;
	u8 init:1;
	u8 setup:1;
	u8 calibrate:1;
	u8 input_src:1;
	u8 ee_flg:1;
	u8 resv_bit:3;
	u8 threshold:8;
	u16 resv_bytes;
	u32 senser;

	struct power_supply bat;
	struct device *w1_dev;
	struct ds2438_ops ops;
	struct workqueue_struct *monitor_wqueue;
	struct delayed_work monitor_work;
};

#define DS2438_SENSER	25
#define to_ds2438_device_info(x) container_of((x), struct ds2438_device_info, \
					      bat);


static enum power_supply_property ds2438_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_NOW,
};

static char ds2438_sensers_title[] = "DS2438 senserin thousands of resister:";
static unsigned int cache_time = 1000;
module_param(cache_time, uint, 0644);
MODULE_PARM_DESC(cache_time, "cache time in milliseconds");

static ssize_t ds2438_show_input(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct ds2438_device_info *di = to_ds2438_device_info(psy);

	return sprintf(buf, "%s\n", di->input_src ? "1:VDD" : "0:VAD");
}

static ssize_t ds2438_show_senser(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int len;
	struct power_supply *psy = dev_get_drvdata(dev);
	struct ds2438_device_info *di = to_ds2438_device_info(psy);

	len = sprintf(buf, "%s\n", ds2438_sensers_title);
	len += sprintf(buf + len, "%d\n", di->senser);
	return len;
}

static ssize_t ds2438_show_ee(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct ds2438_device_info *di = to_ds2438_device_info(psy);

	return sprintf(buf, "%d\n", di->ee_flg);
}

static ssize_t ds2438_show_threshold(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct ds2438_device_info *di = to_ds2438_device_info(psy);

	return sprintf(buf, "%d\n", di->threshold);
}

static ssize_t ds2438_set_input(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct ds2438_device_info *di = to_ds2438_device_info(psy);
	di->input_src = !!simple_strtoul(buf, NULL, 0);
	return count;
}

static ssize_t ds2438_set_senser(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	u32 resister;
	struct power_supply *psy = dev_get_drvdata(dev);
	struct ds2438_device_info *di = to_ds2438_device_info(psy);
	resister = simple_strtoul(buf, NULL, 0);
	if (resister)
		di->senser = resister;
	return count;
}

static ssize_t ds2438_set_ee(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct ds2438_device_info *di = to_ds2438_device_info(psy);

	di->ee_flg = !!simple_strtoul(buf, NULL, 0);
	di->setup = 1;
	return count;
}

static ssize_t ds2438_set_threshold(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int threshold;
	struct power_supply *psy = dev_get_drvdata(dev);
	struct ds2438_device_info *di = to_ds2438_device_info(psy);

	threshold = simple_strtoul(buf, NULL, 0);
	if (threshold < 256) {
		di->threshold = threshold;
		di->setup = 1;
		return count;
	}
	return -EINVAL;
}

static ssize_t ds2438_set_calibrate(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct ds2438_device_info *di = to_ds2438_device_info(psy);

	di->calibrate = !!simple_strtoul(buf, NULL, 0);
	return count;
}

static struct device_attribute ds2438_dev_attr[] = {
	__ATTR(input_src, 0664, ds2438_show_input, ds2438_set_input),
	__ATTR(senser, 0664, ds2438_show_senser, ds2438_set_senser),
	__ATTR(ee_flg, 0664, ds2438_show_ee, ds2438_set_ee),
	__ATTR(threshold, 0664, ds2438_show_threshold, ds2438_set_threshold),
	__ATTR(calibrate, 0220, NULL, ds2438_set_calibrate),
};

static void ds2438_setup(struct ds2438_device_info *di)
{
	di->ops.load_sram(di->w1_dev, PAGE0_CONTROL);
	di->ops.read_page(di->w1_dev, PAGE0_CONTROL, di->raw);
	if (di->init && di->setup) {
		if (di->ee_flg)
			di->raw[PAGE0_STAT_CTRL] |= DS2438_CTRL_EE;
		else
			di->raw[PAGE0_STAT_CTRL] &= ~DS2438_CTRL_EE;
		if (di->input_src)
			di->raw[PAGE0_STAT_CTRL] |= DS2438_CTRL_AD;
		else
			di->raw[PAGE0_STAT_CTRL] &= ~DS2438_CTRL_AD;
		di->raw[PAGE0_THRESHOLD] = di->threshold;
	} else {
		di->ee_flg = !!(di->raw[PAGE0_STAT_CTRL] & DS2438_CTRL_EE);
		di->input_src = !!(di->raw[PAGE0_STAT_CTRL] & DS2438_CTRL_AD);
		di->threshold = di->raw[PAGE0_THRESHOLD];
		di->raw[PAGE0_STAT_CTRL] |= DS2438_CTRL_IAD | DS2438_CTRL_CA;
	}
	di->ops.write_page(di->w1_dev, PAGE0_CONTROL, di->raw);
	di->ops.drain_sram(di->w1_dev, PAGE0_CONTROL);
	if (!di->init) {
		di->calibrate = 1;
		di->init = 1;
	}
	di->setup = 0;
}

static void ds2438_calibrate(struct ds2438_device_info *di)
{
	int current_raw;
	/* disable ICA */
	di->ops.load_sram(di->w1_dev, PAGE0_CONTROL);
	di->ops.read_page(di->w1_dev, PAGE0_CONTROL, di->raw);
	di->raw[PAGE0_STAT_CTRL] &= ~DS2438_CTRL_IAD;
	di->ops.write_page(di->w1_dev, PAGE0_CONTROL, di->raw);
	di->ops.drain_sram(di->w1_dev, PAGE0_CONTROL);

	/* Zero offset */
	di->ops.load_sram(di->w1_dev, PAGE1_ETM);
	di->ops.read_page(di->w1_dev, PAGE1_ETM, di->raw);
	ds2438_writew(di->raw + PAGE1_OFFSET_LSB, 0);
	di->ops.drain_sram(di->w1_dev, PAGE1_ETM_BYTE0);

	/* enable ICA & read current */
	di->ops.load_sram(di->w1_dev, PAGE0_CONTROL);
	di->ops.read_page(di->w1_dev, PAGE0_CONTROL, di->raw);
	di->raw[PAGE0_STAT_CTRL] |= DS2438_CTRL_IAD;
	di->ops.write_page(di->w1_dev, PAGE0_CONTROL, di->raw);
	di->ops.drain_sram(di->w1_dev, PAGE0_CONTROL);
	/*wait current convert about 36HZ */
	mdelay(30);
	/* disable ICA */
	di->ops.load_sram(di->w1_dev, PAGE0_CONTROL);
	di->ops.read_page(di->w1_dev, PAGE0_CONTROL, di->raw);
	di->raw[PAGE0_STAT_CTRL] &= ~DS2438_CTRL_IAD;
	di->ops.write_page(di->w1_dev, PAGE0_CONTROL, di->raw);
	di->ops.drain_sram(di->w1_dev, PAGE0_CONTROL);
	/* read current value */
	current_raw = ds2438_readw(di->raw + PAGE0_CURRENT_LSB);
	/* write offset by current value */
	di->ops.load_sram(di->w1_dev, PAGE1_ETM);
	di->ops.read_page(di->w1_dev, PAGE1_ETM, di->raw);
	ds2438_writew(di->raw + PAGE1_OFFSET_LSB, current_raw << 8);
	di->ops.write_page(di->w1_dev, PAGE1_ETM, di->raw);
	di->ops.drain_sram(di->w1_dev, PAGE1_ETM);

	/*enable ICA again */
	di->ops.load_sram(di->w1_dev, PAGE0_CONTROL);
	di->ops.read_page(di->w1_dev, PAGE0_CONTROL, di->raw);
	di->raw[PAGE0_STAT_CTRL] |= DS2438_CTRL_IAD;
	di->ops.write_page(di->w1_dev, PAGE0_CONTROL, di->raw);
	di->ops.drain_sram(di->w1_dev, PAGE0_CONTROL);
	di->calibrate = 0;
}

/*
 * power supply temperture is in tenths of degree.
 */
static inline int ds2438_get_temp(u16 raw)
{
	int degree, s;
	s = !!(raw & 0x8000);

	if (s)
		raw = ((~raw & 0x7FFF) + 1);
	degree = ((raw >> 8) * 10) + (((raw & 0xFF) * 5) + 63) / 128;
	return s ? -degree : degree;
}

/*
 * power supply current is in uA.
 */
static inline int ds2438_get_current(u32 senser, u16 raw)
{
	int s, current_uA;
	s = !!(raw & 0xFC00);
	/* (x * 1000 * 1000)uA / (4096 * (Rsens / 1000)) */
	raw &= 0x3FF;
	current_uA = raw * 125 * 125 * 125;
	current_uA /= (senser << 3);
	return s ? -current_uA : current_uA;
}

/*
 * power supply current is in uAh.
 */
static inline int ds2438_get_ica(u32 senser, u8 raw)
{
	int charge_uAh;
	/* (x * 1000 * 1000)uA / (2048 * (Rsens / 1000)) */
	charge_uAh = (raw * 125 * 125 * 125) >> 4;
	charge_uAh /= (senser << 4);
	return charge_uAh;
}

static int ds2438_battery_update_page1(struct ds2438_device_info *di)
{
	int ica_raw;
	di->ops.load_sram(di->w1_dev, PAGE1_ETM);
	di->ops.read_page(di->w1_dev, PAGE1_ETM, di->raw);
	ica_raw = di->raw[PAGE1_ICA];
	di->accum_current_uAh = ds2438_get_ica(di->senser, ica_raw);
	return 0;
}

static int ds2438_battery_read_status(struct ds2438_device_info *di)
{
	u8 status;
	int temp_raw, voltage_raw, current_raw;

	if (!(di->init) || di->setup)
		ds2438_setup(di);

	if (di->calibrate)
		ds2438_calibrate(di);

	if (di->update_time && time_before(jiffies, di->update_time +
					   msecs_to_jiffies(cache_time)))
		return 0;

	di->ops.load_sram(di->w1_dev, PAGE0_CONTROL);
	di->ops.read_page(di->w1_dev, PAGE0_CONTROL, di->raw);
	status = di->raw[PAGE0_STAT_CTRL];
	temp_raw = ds2438_readw(di->raw + PAGE0_TEMP_LSB);
	voltage_raw = ds2438_readw(di->raw + PAGE0_VOLTAGE_LSB);
	current_raw = ds2438_readw(di->raw + PAGE0_CURRENT_LSB);
	di->temp_C = ds2438_get_temp(temp_raw);
	di->voltage_uV = voltage_raw * 10000;
	di->current_uA = ds2438_get_current(di->senser, current_raw);

	ds2438_battery_update_page1(di);

	if (!(status & DS2438_STAT_TB))
		di->ops.command(di->w1_dev, DS2438_CONVERT_TEMP, 0);
	if (!(status & DS2438_STAT_ADB))
		di->ops.command(di->w1_dev, DS2438_CONVERT_VOLT, 0);
	di->update_time = jiffies;
	return 0;
}

static void ds2438_battery_update_status(struct ds2438_device_info *di)
{
	int old_charge_status = di->charge_status;

	ds2438_battery_read_status(di);

	if (di->charge_status != old_charge_status)
		power_supply_changed(&di->bat);
}

static void ds2438_battery_work(struct work_struct *work)
{
	struct ds2438_device_info *di = container_of(work,
						     struct ds2438_device_info,
						     monitor_work.work);
	const int interval = HZ * 60;

	dev_dbg(di->w1_dev, "%s\n", __func__);

	ds2438_battery_update_status(di);
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, interval);
}

static void ds2438_battery_external_power_changed(struct power_supply *psy)
{
	struct ds2438_device_info *di = to_ds2438_device_info(psy);

	dev_dbg(di->w1_dev, "%s\n", __func__);

	cancel_delayed_work(&di->monitor_work);
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, HZ / 10);
}

static int ds2438_battery_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct ds2438_device_info *di = to_ds2438_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = di->charge_status;
		return 0;
	default:
		break;
	}

	ds2438_battery_read_status(di);

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = di->voltage_uV;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = di->current_uA;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = di->temp_C;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = di->accum_current_uAh;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static inline void ds2438_defaut_ops(struct ds2438_ops *ops)
{
	ops->read_page = w1_ds2438_read_page;
	ops->write_page = w1_ds2438_write_page;
	ops->drain_sram = w1_ds2438_drain_sram;
	ops->load_sram = w1_ds2438_load_sram;
	ops->command = w1_ds2438_command;
}


static int ds2438_battery_probe(struct platform_device *pdev)
{
	int i, retval = 0;
	struct ds2438_device_info *di;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		retval = -ENOMEM;
		goto di_alloc_failed;
	}

	di->dev	= &pdev->dev;
	di->w1_dev = pdev->dev.parent;
	di->bat.name = dev_name(&pdev->dev);
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = ds2438_battery_props;
	di->bat.num_properties = ARRAY_SIZE(ds2438_battery_props);
	di->bat.get_property = ds2438_battery_get_property;
	di->bat.external_power_changed = ds2438_battery_external_power_changed;
	ds2438_defaut_ops(&di->ops);
	di->senser = DS2438_SENSER;
	di->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;

	retval = power_supply_register(&pdev->dev, &di->bat);
	if (retval) {
		dev_err(&pdev->dev, "failed to register battery\n");
		goto batt_failed;
	}

	for (i = 0; i < ARRAY_SIZE(ds2438_dev_attr); i++) {
		if (device_create_file(di->bat.dev, ds2438_dev_attr + i)) {
			printk(KERN_ERR "Customize attribute file fail!\n");
			break;
		}
	}

	if (i != ARRAY_SIZE(ds2438_dev_attr)) {
		for (; i >= 0; i++)
			device_remove_file(di->bat.dev, ds2438_dev_attr + i);
		goto workqueue_failed;
	}
	INIT_DELAYED_WORK(&di->monitor_work, ds2438_battery_work);
	di->monitor_wqueue = create_singlethread_workqueue(
				dev_name(&pdev->dev));
	if (!di->monitor_wqueue) {
		retval = -ESRCH;
		goto workqueue_failed;
	}

	platform_set_drvdata(pdev, di);
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, HZ / 2);

	goto success;

workqueue_failed:
	power_supply_unregister(&di->bat);
batt_failed:
	kfree(di);
di_alloc_failed:
success:
	return retval;
}

static int ds2438_battery_remove(struct platform_device *pdev)
{
	struct ds2438_device_info *di = platform_get_drvdata(pdev);

	cancel_rearming_delayed_workqueue(di->monitor_wqueue,
					  &di->monitor_work);
	destroy_workqueue(di->monitor_wqueue);
	power_supply_unregister(&di->bat);
	return 0;
}

#ifdef CONFIG_PM
static int ds2438_battery_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	struct ds2438_device_info *di = platform_get_drvdata(pdev);

	di->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;

	return 0;
}

static int ds2438_battery_resume(struct platform_device *pdev)
{
	struct ds2438_device_info *di = platform_get_drvdata(pdev);

	di->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;
	power_supply_changed(&di->bat);

	cancel_delayed_work(&di->monitor_work);
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, HZ);

	return 0;
}

#else

#define ds2438_battery_suspend NULL
#define ds2438_battery_resume NULL

#endif /* CONFIG_PM */
static struct platform_driver ds2438_battery_driver = {
	.driver = {
		.name = DS2438_DEV_NAME,
	},
	.probe		= ds2438_battery_probe,
	.remove		= ds2438_battery_remove,
	.suspend	= ds2438_battery_suspend,
	.resume		= ds2438_battery_resume,
};

static int __init ds2438_battery_init(void)
{
	pr_info("ds2438 battery driver\n");
	return platform_driver_register(&ds2438_battery_driver);
}

static void __exit ds2438_battery_exit(void)
{
	platform_driver_unregister(&ds2438_battery_driver);
}

module_init(ds2438_battery_init);
module_exit(ds2438_battery_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Freescale Semiconductors Inc");
MODULE_DESCRIPTION("DS2438 battery driver.");
