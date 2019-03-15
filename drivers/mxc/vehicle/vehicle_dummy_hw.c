/*
 * Copyright 2018 NXP
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
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/extcon.h>
#include <linux/of_platform.h>
#include "vehicle_core.h"

#define FAN_SPEED_0 2
#define FAN_SPEED_1 4
#define FAN_SPEED_2 6
#define FAN_SPEED_3 8
#define FAN_SPEED_4 10
#define FAN_SPEED_5 12
#define FAN_DIRECTION_0 2
#define FAN_DIRECTION_1 4
#define FAN_DIRECTION_2 6
#define FAN_DIRECTION_3 12
#define RECIRC_ON 2
#define RECIRC_OFF 0
#define HVAC_ON 2
#define HVAC_OFF 0
#define AUTO_ON 2
#define AUTO_OFF 0
#define AC_ON 2
#define AC_OFF 0
#define DEFROST_ON 2
#define DEFROST_OFF 0
//VEHICLE is in state parking
#define GEAR_0 1
//VEHICLE is in state reverse
#define GEAR_1 2
//VEHICLE is in state driver
#define GEAR_2 4
// no turn signal
#define TURN_0 0
// left turn signal
#define TURN_1 1
// right turn signal
#define TURN_2 2

// temperature set from hardware on Android OREO and PIE uses below indexs
#define AC_TEMP_LEFT_INDEX 1
#define AC_TEMP_RIGHT_INDEX 4

// temperature set from APP on Android PIE uses below indexs
#define PIE_AC_TEMP_LEFT_INDEX 49
#define PIE_AC_TEMP_RIGHT_INDEX 68

struct vehicle_dummy_drvdata {
	struct device *dev;
	u32 gear;
	u32 turn;
	u32 temp_right;
	u32 temp_left;
	u32 fan_direction;
	u32 fan_speed;
	u32 defrost_left;
	u32 defrost_right;
	u32 ac_on;
	u32 auto_on;
	u32 hvac_on;
	u32 recirc_on;
};

#ifdef CONFIG_EXTCON
static const unsigned int imx_vehicle_dummy_extcon_register_cables[] = {
	EXTCON_VEHICLE_RPMSG_REGISTER,
	EXTCON_NONE,
};
static const unsigned int imx_vehicle_dummy_extcon_event_cables[] = {
	EXTCON_VEHICLE_RPMSG_EVENT,
	EXTCON_NONE,
};
struct extcon_dev *rg_edev;
struct extcon_dev *ev_edev;
#endif

extern void vehicle_hw_prop_ops_register(const struct hw_prop_ops* prop_ops);
extern void vehicle_hal_set_property(u16 prop, u8 index, u32 value);
static struct vehicle_dummy_drvdata *vehicle_dummy;
static struct class* vehicle_dummy_class;

void mcu_set_control_commands(u32 prop, u32 area, u32 value)
{
	switch (prop) {
	case HVAC_FAN_SPEED:
		pr_info("set fan speed with value %d\n", value);
		vehicle_dummy->fan_speed = value;
		break;
	case HVAC_FAN_DIRECTION:
		pr_info("set fan direction with value %d\n", value);
		vehicle_dummy->fan_direction = value;
		break;
	case HVAC_AUTO_ON:
		pr_info("set fan auto on with value %d\n", value);
		vehicle_dummy->auto_on = value;
		break;
	case HVAC_AC_ON:
		pr_info("set fan ac on with value %d\n", value);
		vehicle_dummy->ac_on = value;
		break;
	case HVAC_RECIRC_ON:
		pr_info("set fan recirc on with value %d\n", value);
		vehicle_dummy->recirc_on = value;
		break;
	case HVAC_DEFROSTER:
		pr_info("set defroster index %d with value %d\n", area, value);
		if (area == 1)
			vehicle_dummy->defrost_left = value;
		else
			vehicle_dummy->defrost_right = value;
		break;
	case HVAC_TEMPERATURE_SET:
		pr_info("set temp index %d with value %d\n", area, value);
#ifdef CONFIG_VEHICLE_DRIVER_OREO
		if (area == AC_TEMP_LEFT_INDEX)
			vehicle_dummy->temp_left = value;
		else if (area == AC_TEMP_RIGHT_INDEX)
			vehicle_dummy->temp_right = value;
#else
		if (area == PIE_AC_TEMP_LEFT_INDEX)
			vehicle_dummy->temp_left = value;
		else if (area == PIE_AC_TEMP_RIGHT_INDEX)
			vehicle_dummy->temp_right = value;
#endif
		break;
	case HVAC_POWER_ON:
		pr_info("set hvac power on with value %d\n", value);
		vehicle_dummy->hvac_on = value;
		break;
	default:
		pr_err("this type is not correct!\n");
	}
}

static ssize_t turn_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%u\n", vehicle_dummy->turn);
}

/* echo 0/1/2(none/left/right) > /sys/devices/platform/vehicle_dummy_hw/turn*/
static ssize_t turn_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t size)
{
	u32 turn;

	if (!size)
		return -EINVAL;
	turn = simple_strtoul(buf, NULL, 10);
	if (turn != TURN_0 && turn != TURN_1 && turn != TURN_2) {
		pr_err("input value is not correct, please type correct one \n");
		return -EINVAL;
	}
	if (turn != vehicle_dummy->turn) {
		vehicle_dummy->turn = turn;
		vehicle_hal_set_property(VEHICLE_TURN_SIGNAL, 0, turn);
	}
	return size;
}
static DEVICE_ATTR(turn, 0664, turn_show, turn_store);

static ssize_t gear_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%u\n", vehicle_dummy->gear);
}

/*echo 1/2/4(parking/reverse/drive) > /sys/devices/platform/vehicle_dummy_hw/gear*/
static ssize_t gear_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t size)
{
	u32 gear;

	if (!size)
		return -EINVAL;
	gear = simple_strtoul(buf, NULL, 10);
	if (gear != vehicle_dummy->gear) {
		vehicle_dummy->gear = gear;
		vehicle_hal_set_property(VEHICLE_GEAR, 0, gear);
		if (gear != GEAR_0 && gear != GEAR_1 && gear != GEAR_2) {
			pr_err("input value is not correct, please type correct one \n");
			return -EINVAL;
		}
#ifdef CONFIG_EXTCON
		if (VEHICLE_GEAR_DRIVE == gear)
			extcon_set_state_sync(ev_edev, EXTCON_VEHICLE_RPMSG_EVENT, 0);
		else if (VEHICLE_GEAR_REVERSE == gear)
			extcon_set_state_sync(ev_edev, EXTCON_VEHICLE_RPMSG_EVENT, 1);
#endif
	}
	return size;
}

static DEVICE_ATTR(gear, 0664, gear_show, gear_store);

static ssize_t temp_left_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d\n", vehicle_dummy->temp_left);
}

/*echo 1100713529 > /sys/devices/platform/vehicle_dummy_hw/temp_left*/
static ssize_t temp_left_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t size)
{
	u32 temp;

	if (!size)
		return -EINVAL;
	temp = simple_strtoul(buf, NULL, 10);
	if (temp != vehicle_dummy->temp_left) {
		vehicle_dummy->temp_left = temp;
		vehicle_hal_set_property(VEHICLE_AC_TEMP, AC_TEMP_LEFT_INDEX, temp);
	}
	return size;
}

static DEVICE_ATTR(temp_left, 0664, temp_left_show, temp_left_store);

static ssize_t temp_right_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d\n", vehicle_dummy->temp_right);
}

/*echo 1100713529 > /sys/devices/platform/vehicle_dummy_hw/temp_right*/
static ssize_t temp_right_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t size)
{
	u32 temp;

	if (!size)
		return -EINVAL;
	temp = simple_strtoul(buf, NULL, 10);
	if (temp != vehicle_dummy->temp_right) {
		vehicle_dummy->temp_right = temp;
		vehicle_hal_set_property(VEHICLE_AC_TEMP, AC_TEMP_RIGHT_INDEX, temp);
	}
	return size;
}

static DEVICE_ATTR(temp_right, 0664, temp_right_show, temp_right_store);

static ssize_t fan_direction_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%u\n", vehicle_dummy->fan_direction);
}

/*echo 2/4/6/12 > /sys/devices/platform/vehicle_dummy_hw/fan_direction*/
static ssize_t fan_direction_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t size)
{
	u32 fan_direction;

	if (!size)
		return -EINVAL;
	fan_direction = simple_strtoul(buf, NULL, 10);
	if (fan_direction != FAN_DIRECTION_0 && fan_direction != FAN_DIRECTION_1 &&
		fan_direction != FAN_DIRECTION_2 && fan_direction != FAN_DIRECTION_3) {
		pr_err("input value is not correct, please type correct one \n");
		return -EINVAL;
	}
	if (fan_direction != vehicle_dummy->fan_direction) {
		vehicle_dummy->fan_direction = fan_direction;
		vehicle_hal_set_property(VEHICLE_FAN_DIRECTION, 0, fan_direction);
	}
	return size;
}

static DEVICE_ATTR(fan_direction, 0664, fan_direction_show, fan_direction_store);

static ssize_t fan_speed_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%u\n", vehicle_dummy->fan_speed);
}

/*echo 2/4/6/8/10/12 > /sys/devices/platform/vehicle_dummy_hw/fan_speed*/
static ssize_t fan_speed_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t size)
{
	u32 fan_speed;

	if (!size)
		return -EINVAL;
	fan_speed = simple_strtoul(buf, NULL, 10);
	if (fan_speed != FAN_SPEED_0 && fan_speed != FAN_SPEED_1 &&
		fan_speed != FAN_SPEED_2 && fan_speed != FAN_SPEED_3 &&
		fan_speed != FAN_SPEED_4 && fan_speed != FAN_SPEED_5) {
		pr_err("input value is not correct, please type correct one \n");
		return -EINVAL;
	}

	if (fan_speed != vehicle_dummy->fan_speed) {
		vehicle_dummy->fan_speed = fan_speed;
		vehicle_hal_set_property(VEHICLE_FAN_SPEED, 0, fan_speed);
	}
	return size;
}

static DEVICE_ATTR(fan_speed, 0664, fan_speed_show, fan_speed_store);

static ssize_t defrost_left_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%u\n", vehicle_dummy->defrost_left);
}

/*echo 0/2 > /sys/devices/platform/vehicle_dummy_hw/defrost_left*/
static ssize_t defrost_left_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t size)
{
	u32 defrost;

	if (!size)
		return -EINVAL;

	defrost = simple_strtoul(buf, NULL, 10);
	if (defrost != DEFROST_ON && defrost != DEFROST_OFF) {
		pr_err("input value is not correct, please type correct one \n");
		return -EINVAL;
	}
	if (defrost != vehicle_dummy->defrost_left) {
		vehicle_dummy->defrost_left = defrost;
		vehicle_hal_set_property(VEHICLE_DEFROST, 1, defrost);
	}
	return size;
}

static DEVICE_ATTR(defrost_left, 0664, defrost_left_show, defrost_left_store);

static ssize_t defrost_right_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%u\n", vehicle_dummy->defrost_right);
}

/*echo 0/2 > /sys/devices/platform/vehicle_dummy_hw/defrost_right*/
static ssize_t defrost_right_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t size)
{
	u32 defrost;

	if (!size)
		return -EINVAL;

	defrost = simple_strtoul(buf, NULL, 10);
	if (defrost != DEFROST_ON && defrost != DEFROST_OFF) {
		pr_err("input value is not correct, please type correct one \n");
		return -EINVAL;
	}
	if (defrost != vehicle_dummy->defrost_right) {
		vehicle_dummy->defrost_right = defrost;
		vehicle_hal_set_property(VEHICLE_DEFROST, 2, defrost);
	}
	return size;
}

static DEVICE_ATTR(defrost_right, 0664, defrost_right_show, defrost_right_store);

static ssize_t ac_on_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%u\n", vehicle_dummy->ac_on);
}

/*echo 0/2 > /sys/devices/platform/vehicle_dummy_hw/ac_on*/
static ssize_t ac_on_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t size)
{
	u32 ac_on;

	if (!size)
		return -EINVAL;

	ac_on = simple_strtoul(buf, NULL, 10);
	if (ac_on != AC_ON && ac_on != AC_OFF) {
		pr_err("input value is not correct, please type correct one \n");
		return -EINVAL;
	}
	if (ac_on != vehicle_dummy->ac_on) {
		vehicle_dummy->ac_on = ac_on;
		vehicle_hal_set_property(VEHICLE_AC, 0, ac_on);
	}
	return size;
}

static DEVICE_ATTR(ac_on, 0664, ac_on_show, ac_on_store);

static ssize_t auto_on_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%u\n", vehicle_dummy->auto_on);
}

/*echo 0/2 > /sys/devices/platform/vehicle_dummy_hw/auto_on*/
static ssize_t auto_on_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t size)
{
	u32 auto_on;

	if (!size)
		return -EINVAL;

	auto_on = simple_strtoul(buf, NULL, 10);
	if (auto_on != AUTO_ON && auto_on != AUTO_OFF) {
		pr_err("input value is not correct, please type correct one \n");
		return -EINVAL;
	}
	if (auto_on != vehicle_dummy->auto_on) {
		vehicle_dummy->auto_on = auto_on;
		vehicle_hal_set_property(VEHICLE_AUTO_ON, 0, auto_on);
	}
	return size;
}

static DEVICE_ATTR(auto_on, 0664, auto_on_show, auto_on_store);

static ssize_t hvac_on_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%u\n", vehicle_dummy->ac_on);
}

/*echo 0/2 > /sys/devices/platform/vehicle_dummy_hw/hvac_on*/
static ssize_t hvac_on_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t size)
{
	u32 hvac_on;

	if (!size)
		return -EINVAL;

	hvac_on = simple_strtoul(buf, NULL, 10);
	if (hvac_on != HVAC_ON && hvac_on != HVAC_OFF) {
		pr_err("input value is not correct, please type correct one \n");
		return -EINVAL;
	}
	if (hvac_on != vehicle_dummy->hvac_on) {
		vehicle_dummy->hvac_on = hvac_on;
		vehicle_hal_set_property(VEHICLE_HVAC_POWER_ON, 0, hvac_on);
	}
	return size;
}

static DEVICE_ATTR(hvac_on, 0664, hvac_on_show, hvac_on_store);

static ssize_t recirc_on_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%u\n", vehicle_dummy->recirc_on);
}

/* echo 0/2 > /sys/devices/platform/vehicle_dummy_hw/recirc_on*/
static ssize_t recirc_on_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t size)
{
	u32 recirc_on;

	if (!size)
		return -EINVAL;

	recirc_on = simple_strtoul(buf, NULL, 10);
	if (recirc_on != RECIRC_ON && recirc_on != RECIRC_OFF) {
		pr_err("input value is not correct, please type correct one \n");
		return -EINVAL;
	}
	if (recirc_on != vehicle_dummy->recirc_on) {
		vehicle_dummy->recirc_on = recirc_on;
		vehicle_hal_set_property(VEHICLE_RECIRC_ON, 0, recirc_on);
	}
	return size;
}

static DEVICE_ATTR(recirc_on, 0664, recirc_on_show, recirc_on_store);

static struct vehicle_dummy_drvdata *
vehicle_get_devtree_pdata(struct device *dev)
{
	struct vehicle_dummy_drvdata *ddata;

	ddata = devm_kzalloc(dev,
			     sizeof(*ddata),
			     GFP_KERNEL);
	if (!ddata)
		return ERR_PTR(-ENOMEM);

	return ddata;
}

static const struct hw_prop_ops hw_prop_mcu_ops = {
	.set_control_commands = mcu_set_control_commands,
};

static int vehicle_dummy_hw_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vehicle_dummy_drvdata *ddata;
	int err;

	ddata = vehicle_get_devtree_pdata(dev);
	if (IS_ERR(ddata))
		return PTR_ERR(ddata);

	vehicle_dummy = ddata;
	platform_set_drvdata(pdev, ddata);

	vehicle_dummy_class = class_create(THIS_MODULE, "vehicle_dummy_hw");
	if (IS_ERR(vehicle_dummy_class)) {
		dev_err(dev, "failed to create class.\n");
		return PTR_ERR(vehicle_dummy_class);
	}

	err = device_create_file(dev, &dev_attr_recirc_on) ||
		device_create_file(dev, &dev_attr_hvac_on) ||
		device_create_file(dev, &dev_attr_auto_on) ||
		device_create_file(dev, &dev_attr_ac_on) ||
		device_create_file(dev, &dev_attr_defrost_right) ||
		device_create_file(dev, &dev_attr_defrost_left) ||
		device_create_file(dev, &dev_attr_fan_speed) ||
		device_create_file(dev, &dev_attr_fan_direction) ||
		device_create_file(dev, &dev_attr_temp_left) ||
		device_create_file(dev, &dev_attr_temp_right) ||
		device_create_file(dev, &dev_attr_gear) ||
		device_create_file(dev, &dev_attr_turn);
	if (err)
		return err;

	vehicle_hw_prop_ops_register(&hw_prop_mcu_ops);

#ifdef CONFIG_EXTCON
	rg_edev = devm_extcon_dev_allocate(dev, imx_vehicle_dummy_extcon_register_cables);
	if (IS_ERR(rg_edev)) {
		dev_err(dev, "failed to allocate extcon device\n");
	}
	err = devm_extcon_dev_register(dev,rg_edev);
	if (err < 0) {
		dev_err(dev, "failed to register extcon device\n");
	}
	ev_edev = devm_extcon_dev_allocate(dev, imx_vehicle_dummy_extcon_event_cables);
	if (IS_ERR(ev_edev)) {
		dev_err(dev, "failed to allocate extcon device\n");
	}
	err = devm_extcon_dev_register(dev,ev_edev);
	if (err < 0) {
		dev_err(dev, "failed to register extcon device\n");
	}
#endif

	return 0;
}

static void vehicle_dummy_hw_remove(struct platform_device *pdev)
{
	class_destroy(vehicle_dummy_class);
}

static struct platform_driver vehicle_dummy_hw_driver = {
	.probe          = vehicle_dummy_hw_probe,
	.remove         = vehicle_dummy_hw_remove,
	.driver         =  {
		.name   = "vehicle-dummy",
	}
};

static struct platform_device *dummy_pdev;

static int vehicle_dummy_init(void)
{
	int err;

	dummy_pdev = platform_device_alloc("vehicle-dummy", -1);
	if (!dummy_pdev) {
		pr_err("Failed to allocate dummy vehicle device\n");
		return;
	}

	err = platform_device_add(dummy_pdev);
	if (err != 0) {
		pr_err("Failed to register dummy regulator device: %d\n", err);
		platform_device_put(dummy_pdev);
		return;
	}

	err = platform_driver_register(&vehicle_dummy_hw_driver);
	if (err) {
		pr_err("Failed to register dummy vehicle driver\n");
		platform_device_unregister(dummy_pdev);
		return err;
	}
	return 0;
}

static void __exit vehicle_dummy_exit(void)
{
	platform_driver_unregister(&vehicle_dummy_hw_driver);
	platform_device_unregister(dummy_pdev);
}

late_initcall(vehicle_dummy_init);
module_exit(vehicle_dummy_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("VEHICLE DUMMY HW");
