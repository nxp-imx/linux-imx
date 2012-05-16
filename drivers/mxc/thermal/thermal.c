/*
 *  anatop_thermal.c - anatop Thermal Zone Driver ($Revision: 41 $)
 *
 *  Copyright (C) 2001, 2002 Andy Grover <andrew.grover@intel.com>
 *  Copyright (C) 2001, 2002 Paul Diefenbaugh <paul.s.diefenbaugh@intel.com>
 *  Copyright (C) 2011-2012 Freescale Semiconductor, Inc.
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or (at
 *  your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dmi.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/jiffies.h>
#include <linux/kmod.h>
#include <linux/reboot.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/thermal.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/syscalls.h>
#include <linux/cpufreq.h>
#include "anatop_driver.h"

/* register define of anatop */
#define HW_ANADIG_ANA_MISC0	(0x00000150)
#define HW_ANADIG_ANA_MISC0_SET	(0x00000154)
#define HW_ANADIG_ANA_MISC0_CLR	(0x00000158)
#define HW_ANADIG_ANA_MISC0_TOG	(0x0000015c)

#define HW_ANADIG_TEMPSENSE0	(0x00000180)
#define HW_ANADIG_TEMPSENSE0_SET	(0x00000184)
#define HW_ANADIG_TEMPSENSE0_CLR	(0x00000188)
#define HW_ANADIG_TEMPSENSE0_TOG	(0x0000018c)

#define HW_ANADIG_TEMPSENSE1	(0x00000190)
#define HW_ANADIG_TEMPSENSE1_SET	(0x00000194)
#define HW_ANADIG_TEMPSENSE1_CLR	(0x00000198)

#define BM_ANADIG_ANA_MISC0_REFTOP_SELBIASOFF 0x00000008

#define BP_ANADIG_TEMPSENSE0_TEMP_VALUE      8
#define BM_ANADIG_TEMPSENSE0_TEMP_VALUE 0x000FFF00
#define BF_ANADIG_TEMPSENSE0_TEMP_VALUE(v)  \
	(((v) << 8) & BM_ANADIG_TEMPSENSE0_TEMP_VALUE)
#define BM_ANADIG_TEMPSENSE0_RSVD0 0x00000080
#define BM_ANADIG_TEMPSENSE0_TEST 0x00000040
#define BP_ANADIG_TEMPSENSE0_VBGADJ      3
#define BM_ANADIG_TEMPSENSE0_VBGADJ 0x00000038
#define BF_ANADIG_TEMPSENSE0_VBGADJ(v)  \
	(((v) << 3) & BM_ANADIG_TEMPSENSE0_VBGADJ)
#define BM_ANADIG_TEMPSENSE0_FINISHED 0x00000004
#define BM_ANADIG_TEMPSENSE0_MEASURE_TEMP 0x00000002
#define BM_ANADIG_TEMPSENSE0_POWER_DOWN 0x00000001

#define BP_ANADIG_TEMPSENSE1_RSVD0      16
#define BM_ANADIG_TEMPSENSE1_RSVD0 0xFFFF0000
#define BF_ANADIG_TEMPSENSE1_RSVD0(v) \
	(((v) << 16) & BM_ANADIG_TEMPSENSE1_RSVD0)
#define BP_ANADIG_TEMPSENSE1_MEASURE_FREQ      0
#define BM_ANADIG_TEMPSENSE1_MEASURE_FREQ 0x0000FFFF
#define BF_ANADIG_TEMPSENSE1_MEASURE_FREQ(v)  \
	(((v) << 0) & BM_ANADIG_TEMPSENSE1_MEASURE_FREQ)

/* define */
#define PREFIX "ANATOP: "
#define ANATOP_THERMAL_DRIVER_NAME	"Anatop Thermal"
#define ANATOP_THERMAL_FILE_STATE		"state"
#define ANATOP_THERMAL_FILE_TEMPERATURE	"temperature"
#define ANATOP_THERMAL_FILE_TRIP_POINTS	"trip_points"
#define ANATOP_THERMAL_FILE_COOLING_MODE	"cooling_mode"
#define ANATOP_THERMAL_FILE_POLLING_FREQ	"polling_frequency"
#define ANATOP_THERMAL_NOTIFY_TEMPERATURE	0x80
#define ANATOP_THERMAL_NOTIFY_THRESHOLDS	0x81
#define ANATOP_THERMAL_NOTIFY_DEVICES	0x82
#define ANATOP_THERMAL_NOTIFY_CRITICAL	0xF0
#define ANATOP_THERMAL_NOTIFY_HOT		0xF1
#define ANATOP_THERMAL_MODE_ACTIVE	0x00
#define ANATOP_THERMAL_MAX_ACTIVE	10
#define ANATOP_THERMAL_MAX_LIMIT_STR_LEN 65
#define ANATOP_TRIPS_CRITICAL	0x01
#define ANATOP_TRIPS_HOT		0x02
#define ANATOP_TRIPS_PASSIVE	0x04
#define ANATOP_TRIPS_ACTIVE	    0x08
#define ANATOP_TRIPS_DEVICES	0x10

#define ANATOP_TRIPS_POINT_CRITICAL	0x0
#define ANATOP_TRIPS_POINT_HOT	    0x1
#define ANATOP_TRIPS_POINT_ACTIVE	0x2

#define ANATOP_TRIPS_INIT      (ANATOP_TRIPS_CRITICAL | \
		ANATOP_TRIPS_HOT | ANATOP_TRIPS_ACTIVE)

#define _COMPONENT		ANATOP_THERMAL_COMPONENT
#define KELVIN_OFFSET			273
#define POLLING_FREQ			2000 /* 2s */
#define TEMP_CRITICAL			373 /* 100 C*/
#define TEMP_HOT				363 /* 90 C*/
#define TEMP_ACTIVE				353 /* 80 C*/
#define MEASURE_FREQ			327  /* 327 RTC clocks delay, 10ms */
#define KELVIN_TO_CEL(t, off) (((t) - (off)))
#define CEL_TO_KELVIN(t, off) (((t) + (off)))
#define DEFAULT_RATIO			145
#define DEFAULT_N25C			1541
#define REG_VALUE_TO_CEL(ratio, raw) ((raw_n25c - raw) * 100 / ratio - 25)
#define ANATOP_DEBUG			false
#define THERMAL_FUSE_NAME		"/sys/fsl_otp/HW_OCOTP_ANA1"

/* variables */
unsigned long anatop_base;
unsigned int ratio;
unsigned int raw_25c, raw_hot, hot_temp, raw_n25c;
static bool full_run = true;
bool cooling_cpuhotplug;
bool cooling_device_disable;
unsigned long temperature_cooling;
static const struct anatop_device_id thermal_device_ids[] = {
	{ANATOP_THERMAL_HID},
	{""},
};

enum {
	DEBUG_USER_STATE = 1U << 0,
	DEBUG_VERBOSE = 1U << 1,
};
static int debug_mask = DEBUG_USER_STATE;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

/* functions */
static int anatop_thermal_add(struct anatop_device *device);
static int anatop_thermal_remove(struct platform_device *pdev);
static int anatop_thermal_suspend(struct platform_device *pdev,
		pm_message_t state);
static int anatop_thermal_resume(struct platform_device *pdev);
static int anatop_thermal_get_calibration_data(unsigned int *fuse);
static int anatop_thermal_counting_ratio(unsigned int fuse_data);

/* struct */
struct anatop_thermal_state {
	u8 critical:1;
	u8 hot:1;
	u8 passive:1;
	u8 active:1;
	u8 reserved:4;
	int active_index;
};

struct anatop_thermal_state_flags {
	u8 valid:1;
	u8 enabled:1;
	u8 reserved:6;
};

struct anatop_thermal_critical {
	struct anatop_thermal_state_flags flags;
	unsigned long temperature;
};

struct anatop_thermal_hot {
	struct anatop_thermal_state_flags flags;
	unsigned long temperature;
};

struct anatop_thermal_passive {
	struct anatop_thermal_state_flags flags;
	unsigned long temperature;
	unsigned long tc1;
	unsigned long tc2;
	unsigned long tsp;
};

struct anatop_thermal_active {
	struct anatop_thermal_state_flags flags;
	unsigned long temperature;
};

struct anatop_thermal_trips {
	struct anatop_thermal_critical critical;
	struct anatop_thermal_hot hot;
	struct anatop_thermal_passive passive;
	struct anatop_thermal_active active;
};

struct anatop_thermal_flags {
	u8 cooling_mode:1;
	u8 devices:1;
	u8 reserved:6;
};

struct anatop_thermal {
	struct anatop_device *device;
	long temperature;
	unsigned long last_temperature;
	unsigned long polling_frequency;
	volatile u8 zombie;
	struct anatop_thermal_flags flags;
	struct anatop_thermal_state state;
	struct anatop_thermal_trips trips;
	struct anatop_handle_list devices;
	struct thermal_zone_device *thermal_zone;
	int tz_enabled;
	int kelvin_offset;
	struct mutex lock;
};

/***************************************************************
						Function
****************************************************************/
static int anatop_dump_temperature_register(void)
{
	if (!anatop_base) {
		pr_info("anatop_base is not initialized!!!\n");
		return -EINVAL;
	}
	pr_info("HW_ANADIG_TEMPSENSE0 = 0x%x\n",
			__raw_readl(anatop_base + HW_ANADIG_TEMPSENSE0));
	pr_info("HW_ANADIG_TEMPSENSE1 = 0x%x\n",
			__raw_readl(anatop_base + HW_ANADIG_TEMPSENSE1));
	return 0;
}
static int anatop_thermal_get_temp(struct thermal_zone_device *thermal,
			    long *temp)
{
	struct anatop_thermal *tz = thermal->devdata;
	unsigned int tmp;
	unsigned int reg;
	unsigned int i;

	if (!tz)
		return -EINVAL;
#ifdef CONFIG_FSL_OTP
	if (!ratio) {
		anatop_thermal_get_calibration_data(&tmp);
		*temp = KELVIN_TO_CEL(TEMP_ACTIVE, KELVIN_OFFSET);
		return 0;
	}
#else
	if (!cooling_device_disable)
		pr_info("%s: can't get calibration data, disable cooling!!!\n", __func__);
	cooling_device_disable = true;
	ratio = DEFAULT_RATIO;
#endif

	tz->last_temperature = tz->temperature;

	/* now we only using single measure, every time we measure
	the temperature, we will power on/down the anadig module*/
	__raw_writel(BM_ANADIG_TEMPSENSE0_POWER_DOWN,
			anatop_base + HW_ANADIG_TEMPSENSE0_CLR);
	__raw_writel(BM_ANADIG_ANA_MISC0_REFTOP_SELBIASOFF,
			anatop_base + HW_ANADIG_ANA_MISC0_SET);

	/* write measure freq */
	reg = __raw_readl(anatop_base + HW_ANADIG_TEMPSENSE1);
	reg &= ~BM_ANADIG_TEMPSENSE1_MEASURE_FREQ;
	reg |= MEASURE_FREQ;
	__raw_writel(reg, anatop_base + HW_ANADIG_TEMPSENSE1);

	__raw_writel(BM_ANADIG_TEMPSENSE0_MEASURE_TEMP,
		anatop_base + HW_ANADIG_TEMPSENSE0_CLR);
	__raw_writel(BM_ANADIG_TEMPSENSE0_FINISHED,
		anatop_base + HW_ANADIG_TEMPSENSE0_CLR);
	__raw_writel(BM_ANADIG_TEMPSENSE0_MEASURE_TEMP,
		anatop_base + HW_ANADIG_TEMPSENSE0_SET);

	tmp = 0;
	/* read five times of temperature values to get average*/
	for (i = 0; i < 5; i++) {
		while ((__raw_readl(anatop_base + HW_ANADIG_TEMPSENSE0)
			& BM_ANADIG_TEMPSENSE0_FINISHED) == 0)
			msleep(10);
		reg = __raw_readl(anatop_base + HW_ANADIG_TEMPSENSE0);
		tmp += (reg & BM_ANADIG_TEMPSENSE0_TEMP_VALUE)
				>> BP_ANADIG_TEMPSENSE0_TEMP_VALUE;
		__raw_writel(BM_ANADIG_TEMPSENSE0_FINISHED,
			anatop_base + HW_ANADIG_TEMPSENSE0_CLR);
		if (ANATOP_DEBUG)
			anatop_dump_temperature_register();
	}

	tmp = tmp / 5;
	if (tmp <= raw_n25c)
		tz->temperature = REG_VALUE_TO_CEL(ratio, tmp);
	else
		tz->temperature = -25;
	if (debug_mask & DEBUG_VERBOSE)
		pr_info("Cooling device Temperature is %lu C\n", tz->temperature);
	/* power down anatop thermal sensor */
	__raw_writel(BM_ANADIG_TEMPSENSE0_POWER_DOWN,
			anatop_base + HW_ANADIG_TEMPSENSE0_SET);
	__raw_writel(BM_ANADIG_ANA_MISC0_REFTOP_SELBIASOFF,
			anatop_base + HW_ANADIG_ANA_MISC0_CLR);

	*temp = (cooling_device_disable && tz->temperature >= KELVIN_TO_CEL(TEMP_CRITICAL, KELVIN_OFFSET)) ?
			KELVIN_TO_CEL(TEMP_CRITICAL - 1, KELVIN_OFFSET) : tz->temperature;

	return 0;
}

static int anatop_thermal_set_cooling_mode(struct anatop_thermal *tz, int mode)
{
	return 0;
}

static int anatop_thermal_trips_update(struct anatop_thermal *tz, int flag)
{
	/* Critical Shutdown */
	if (flag & ANATOP_TRIPS_CRITICAL) {
		tz->trips.critical.temperature = TEMP_CRITICAL;
		tz->trips.critical.flags.valid = 1;
	}

	/* Hot */
	if (flag & ANATOP_TRIPS_HOT) {
			tz->trips.hot.temperature = TEMP_HOT;
			tz->trips.hot.flags.valid = 1;
	}

	/* Active */
	if (flag & ANATOP_TRIPS_ACTIVE) {
			tz->trips.active.temperature = TEMP_ACTIVE;
			tz->trips.active.flags.valid = 1;
	}

	return 0;
}

static int anatop_thermal_get_trip_points(struct anatop_thermal *tz)
{
	int valid, ret;

	ret = anatop_thermal_trips_update(tz, ANATOP_TRIPS_INIT);

	valid = tz->trips.critical.flags.valid |
		tz->trips.hot.flags.valid |
		tz->trips.active.flags.valid;

	if (!valid) {
		printk(KERN_WARNING "No valid trip found\n");
		ret = -ENODEV;
	}
	return ret;
}

static const char enabled[] = "kernel";
static const char disabled[] = "user";
static int anatop_thermal_get_mode(struct thermal_zone_device *thermal,
				enum thermal_device_mode *mode)
{
	struct anatop_thermal *tz = thermal->devdata;

	if (!tz)
		return -EINVAL;

	*mode = tz->tz_enabled ? THERMAL_DEVICE_ENABLED :
		THERMAL_DEVICE_DISABLED;

	return 0;
}

static int anatop_thermal_set_mode(struct thermal_zone_device *thermal,
				enum thermal_device_mode mode)
{
	struct anatop_thermal *tz = thermal->devdata;
	int enable;

	if (!tz)
		return -EINVAL;

	/* enable/disable thermal management from thermal driver */
	if (mode == THERMAL_DEVICE_ENABLED)
		enable = 1;
	else if (mode == THERMAL_DEVICE_DISABLED)
		enable = 0;
	else
		return -EINVAL;

	if (enable != tz->tz_enabled) {
		tz->tz_enabled = enable;
		printk(KERN_INFO "%s anatop thermal control\n",
			tz->tz_enabled ? enabled : disabled);
		thermal_zone_device_update(tz->thermal_zone);
	}
	return 0;
}

static int anatop_thermal_get_trip_type(struct thermal_zone_device *thermal,
				 int trip, enum thermal_trip_type *type)
{
	struct anatop_thermal *tz = thermal->devdata;

	if (!tz || trip < 0)
		return -EINVAL;

	if (tz->trips.critical.flags.valid) {
		if (!trip) {
			*type = THERMAL_TRIP_CRITICAL;
			return 0;
		}
		trip--;
	}

	if (tz->trips.hot.flags.valid) {
		if (!trip) {
			*type = THERMAL_TRIP_HOT;
			return 0;
		}
		trip--;
	}

	if (tz->trips.passive.flags.valid) {
		if (!trip) {
			*type = THERMAL_TRIP_PASSIVE;
			return 0;
		}
		trip--;
	}

	if (tz->trips.active.flags.valid) {
		if (!trip) {
			*type = THERMAL_TRIP_ACTIVE;
			return 0;
		}
		trip--;
	}

	return -EINVAL;
}

static int anatop_thermal_set_trip_temp(struct thermal_zone_device *thermal,
				 int trip, unsigned long *temp)
{
	struct anatop_thermal *tz = thermal->devdata;

	if (!tz || trip < 0)
		return -EINVAL;

	switch (trip) {
	case ANATOP_TRIPS_POINT_CRITICAL:
		/* Critical Shutdown */
		if (tz->trips.critical.flags.valid)
			tz->trips.critical.temperature = CEL_TO_KELVIN(
				*temp, tz->kelvin_offset);
		break;
	case ANATOP_TRIPS_POINT_HOT:
		/* Hot */
		if (tz->trips.hot.flags.valid)
				tz->trips.hot.temperature = CEL_TO_KELVIN(
				*temp, tz->kelvin_offset);
		break;
	case ANATOP_TRIPS_POINT_ACTIVE:
		/* Active */
		if (tz->trips.active.flags.valid)
				tz->trips.active.temperature = CEL_TO_KELVIN(
				*temp, tz->kelvin_offset);
		break;
	default:
		break;
	}

	return 0;
}

static int anatop_thermal_get_trip_temp(struct thermal_zone_device *thermal,
				 int trip, unsigned long *temp)
{
	struct anatop_thermal *tz = thermal->devdata;

	if (!tz || trip < 0)
		return -EINVAL;
	if (tz->trips.critical.flags.valid) {
		if (!trip) {
			*temp = KELVIN_TO_CEL(
				tz->trips.critical.temperature,
				tz->kelvin_offset);
			return 0;
		}
		trip--;
	}

	if (tz->trips.hot.flags.valid) {
		if (!trip) {
			*temp = KELVIN_TO_CEL(
				tz->trips.hot.temperature,
				tz->kelvin_offset);
			return 0;
		}
		trip--;
	}

	if (tz->trips.passive.flags.valid) {
		if (!trip) {
			*temp = KELVIN_TO_CEL(
				tz->trips.passive.temperature,
				tz->kelvin_offset);
			return 0;
		}
		trip--;
	}

	if (tz->trips.active.flags.valid) {
		if (!trip) {
			*temp = KELVIN_TO_CEL(
				tz->trips.active.temperature,
				tz->kelvin_offset);
			return 0;
		}
		trip--;
	}

	return -EINVAL;
}

static int anatop_thermal_get_crit_temp(struct thermal_zone_device *thermal,
				unsigned long *temperature) {
	struct anatop_thermal *tz = thermal->devdata;

	if (tz->trips.critical.flags.valid) {
		*temperature = KELVIN_TO_CEL(
				tz->trips.critical.temperature,
				tz->kelvin_offset);
		return 0;
	} else
		return -EINVAL;
}

static int anatop_thermal_notify(struct thermal_zone_device *thermal, int trip,
			   enum thermal_trip_type trip_type)
{
	int ret = 0;
	u8 type = 0;
	char mode = 'r';
	const char *cmd = "reboot";
	struct anatop_thermal *tz = thermal->devdata;

	if (cooling_device_disable)
		return ret;

	if (trip_type == THERMAL_TRIP_CRITICAL) {
		type = ANATOP_THERMAL_NOTIFY_CRITICAL;
		printk(KERN_WARNING "thermal_notify: trip_critical reached!\n");
		arch_reset(mode, cmd);
	} else if (trip_type == THERMAL_TRIP_HOT) {
		printk(KERN_DEBUG "thermal_notify: trip_hot reached!\n");
		type = ANATOP_THERMAL_NOTIFY_HOT;
		/* if temperature increase, continue to detach secondary CPUs*/
		if (tz->temperature > (temperature_cooling + 1)) {
			if (cooling_cpuhotplug)
				anatop_thermal_cpu_hotplug(false);
			else
				anatop_thermal_cpufreq_down();
			temperature_cooling = tz->temperature;
		}
		if (ret)
			printk(KERN_INFO "No secondary CPUs detached!\n");
		full_run = false;
	} else {
		if (!full_run) {
			temperature_cooling = 0;
			if (cooling_cpuhotplug)
				anatop_thermal_cpu_hotplug(true);
			else
				anatop_thermal_cpufreq_up();
			if (ret)
				printk(KERN_INFO "No secondary CPUs attached!\n");
		}
	}

	return 0;
}

typedef int (*cb)(struct thermal_zone_device *, int,
		  struct thermal_cooling_device *);
static int anatop_thermal_cooling_device_cb(struct thermal_zone_device *thermal,
					struct thermal_cooling_device *cdev,
					cb action)
{
	struct anatop_thermal *tz = thermal->devdata;
	anatop_handle handle;
	int i;
	int trip = -1;
	int result = 0;

	if (tz->trips.critical.flags.valid)
		trip++;

	if (tz->trips.hot.flags.valid)
		trip++;

	if (tz->trips.passive.flags.valid) {
		trip++;
		result = action(thermal, trip, cdev);
		if (result)
			goto failed;
	}

	if (tz->trips.active.flags.valid) {
		trip++;
		result = action(thermal, trip, cdev);
		if (result)
			goto failed;
	}

	for (i = 0; i < tz->devices.count; i++) {
		handle = tz->devices.handles[i];
		result = action(thermal, -1, cdev);
		if (result)
			goto failed;
	}

failed:
	return result;

}

static int
anatop_thermal_bind_cooling_device(struct thermal_zone_device *thermal,
					struct thermal_cooling_device *cdev)
{
	return anatop_thermal_cooling_device_cb(thermal, cdev,
				thermal_zone_bind_cooling_device);
}

static int
anatop_thermal_unbind_cooling_device(struct thermal_zone_device *thermal,
					struct thermal_cooling_device *cdev)
{
	return anatop_thermal_cooling_device_cb(thermal, cdev,
				thermal_zone_unbind_cooling_device);
}

static struct thermal_zone_device_ops anatop_thermal_zone_ops = {
	.bind = anatop_thermal_bind_cooling_device,
	.unbind	= anatop_thermal_unbind_cooling_device,
	.get_temp = anatop_thermal_get_temp,
	.get_mode = anatop_thermal_get_mode,
	.set_mode = anatop_thermal_set_mode,
	.get_trip_type = anatop_thermal_get_trip_type,
	.get_trip_temp = anatop_thermal_get_trip_temp,
	.set_trip_temp = anatop_thermal_set_trip_temp,
	.get_crit_temp = anatop_thermal_get_crit_temp,
	.notify = anatop_thermal_notify,
};

static int anatop_thermal_register_thermal_zone(struct anatop_thermal *tz)
{
	int trips = 0;

	if (tz->trips.critical.flags.valid)
		trips++;

	if (tz->trips.hot.flags.valid)
		trips++;

	if (tz->trips.passive.flags.valid)
		trips++;

	if (tz->trips.active.flags.valid)
		trips++;

	if (tz->trips.passive.flags.valid)
		tz->thermal_zone =
			thermal_zone_device_register("anatoptz", trips, tz,
						     &anatop_thermal_zone_ops,
						     tz->trips.passive.tc1,
						     tz->trips.passive.tc2,
						     tz->trips.passive.tsp*100,
						     tz->polling_frequency*100);
	else
		tz->thermal_zone =
			thermal_zone_device_register("anatoptz", trips, tz,
						     &anatop_thermal_zone_ops,
						     0, 0, 0,
						     tz->polling_frequency);
	tz->tz_enabled = 1;

	printk(KERN_INFO "%s registered as thermal_zone%d\n",
		 tz->device->name, tz->thermal_zone->id);
	return 0;
}

static void anatop_thermal_unregister_thermal_zone(struct anatop_thermal *tz)
{
	sysfs_remove_link(&tz->device->dev.kobj, "thermal_zone");
	sysfs_remove_link(&tz->thermal_zone->device.kobj, "device");
	thermal_zone_device_unregister(tz->thermal_zone);
	tz->thermal_zone = NULL;
}

/* --------------------------------------------------------------
						Driver Interface
   ------------------------------------------------------------- */
static int anatop_thermal_get_info(struct anatop_thermal *tz)
{
	int result = 0;

	if (!tz)
		return -EINVAL;

	/* Get trip points [_CRT, _PSV, etc.] (required) */
	result = anatop_thermal_get_trip_points(tz);
	if (result)
		return result;

	/* Set the cooling mode [_SCP] to active cooling (default) */
	result = anatop_thermal_set_cooling_mode(tz,
			ANATOP_THERMAL_MODE_ACTIVE);
	if (!result)
		tz->flags.cooling_mode = 1;

	/* Get default polling frequency [_TZP] (optional) */
	tz->polling_frequency = POLLING_FREQ;

	return 0;
}
static int anatop_thermal_add(struct anatop_device *device)
{
	int result = 0;
	struct anatop_thermal *tz = NULL;
	struct thermal_cooling_device *cd = NULL;

	if (!device)
		return -EINVAL;

	tz = kzalloc(sizeof(struct anatop_thermal), GFP_KERNEL);
	if (!tz)
		return -ENOMEM;

	cd = kzalloc(sizeof(struct thermal_cooling_device), GFP_KERNEL);
	if (!cd)
		return -ENOMEM;

	tz->device = device;
	strcpy(device->name, ANATOP_THERMAL_DRIVER_NAME);
	device->driver_data = tz;
	mutex_init(&tz->lock);

	result = anatop_thermal_get_info(tz);
	if (result)
		goto free_memory;

	tz->kelvin_offset = KELVIN_OFFSET;

	result = anatop_thermal_register_thermal_zone(tz);
	if (result)
		goto free_memory;

	cd = thermal_cooling_device_register("i.mx_processor", device,
						&imx_processor_cooling_ops);
	if (IS_ERR(cd)) {
		result = PTR_ERR(cd);
		goto free_memory;
	}
	goto end;

free_memory:
	kfree(tz);
	kfree(cd);
end:
	return result;
}

static int __init anatop_thermal_cooling_device_setup(char *str)
{
	if (!strcmp(str, "cpuhotplug")) {
		cooling_cpuhotplug = 1;
		pr_info("%s: cooling device set to hotplug!\n", __func__);
	}
	return 1;
}
__setup("cooling_device=", anatop_thermal_cooling_device_setup);

static int __init anatop_thermal_cooling_device_disable(char *str)
{
	cooling_device_disable = 1;
	pr_info("%s: cooling device is disabled!\n", __func__);
	return 1;
}
__setup("no_cooling_device", anatop_thermal_cooling_device_disable);

static int anatop_thermal_get_calibration_data(unsigned int *fuse)
{
	int ret = -EINVAL;
	int fd;
	char fuse_data[11];

	if (fuse == NULL) {
		printk(KERN_ERR "%s: NULL pointer!\n", __func__);
		return ret;
	}

	fd = sys_open((const char __user __force *)THERMAL_FUSE_NAME,
		O_RDWR, 0700);
	if (fd < 0)
		return ret;

	sys_read(fd, fuse_data, sizeof(fuse_data));
	sys_close(fd);
	ret = 0;

	*fuse = simple_strtol(fuse_data, NULL, 0);
	pr_info("Thermal: fuse data 0x%x\n", *fuse);
	anatop_thermal_counting_ratio(*fuse);

	return ret;
}
static int anatop_thermal_counting_ratio(unsigned int fuse_data)
{
	int ret = -EINVAL;

	if (fuse_data == 0 || fuse_data == 0xffffffff) {
		pr_info("%s: invalid calibration data, disable cooling!!!\n", __func__);
		cooling_device_disable = true;
		ratio = DEFAULT_RATIO;
		return ret;
	}

	ret = 0;
	/* Fuse data layout:
	 * [31:20] sensor value @ 25C
	 * [19:8] sensor value of hot
	 * [7:0] hot temperature value */
	raw_25c = fuse_data >> 20;
	raw_hot = (fuse_data & 0xfff00) >> 8;
	hot_temp = fuse_data & 0xff;

	ratio = ((raw_25c - raw_hot) * 100) / (hot_temp - 25);
	raw_n25c = raw_25c + ratio / 2;

	return ret;
}

static int anatop_thermal_probe(struct platform_device *pdev)
{
	int retval = 0;
	struct resource *res;
	void __iomem *base;
	struct anatop_device *device;

	device = kzalloc(sizeof(*device), GFP_KERNEL);
	if (!device) {
		retval = -ENOMEM;
		goto device_alloc_failed;
	}

	platform_set_drvdata(pdev, device);

	/* ioremap the base address */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "No anatop base address provided!\n");
		goto anatop_failed;
	}

	base = ioremap(res->start, res->end - res->start);
	if (base == NULL) {
		dev_err(&pdev->dev, "failed to remap anatop base address!\n");
		goto anatop_failed;
	}
	anatop_base = (unsigned long)base;
	raw_n25c = DEFAULT_N25C;

	anatop_thermal_add(device);
	anatop_thermal_cpufreq_init();
	pr_info("%s: default cooling device is cpufreq!\n", __func__);

	goto success;
anatop_failed:
	kfree(device);
device_alloc_failed:
success:

	return retval;
}

static int anatop_thermal_remove(struct platform_device *pdev)
{
	struct anatop_thermal *tz;

	if (!pdev)
		return -EINVAL;

	tz = platform_get_drvdata(pdev);

	anatop_thermal_unregister_thermal_zone(tz);

	mutex_destroy(&tz->lock);
	kfree(tz);
	return 0;
}
static int anatop_thermal_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	return 0;
}
static int anatop_thermal_resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver anatop_thermal_driver_ldm = {
	.driver = {
		   .name = "anatop_thermal",
		   .bus = &platform_bus_type,
		   },
	.probe = anatop_thermal_probe,
	.remove = anatop_thermal_remove,
	.suspend = anatop_thermal_suspend,
	.resume = anatop_thermal_resume,
};

static int __init anatop_thermal_init(void)
{
	pr_debug("Anatop Thermal driver loading\n");
	return platform_driver_register(&anatop_thermal_driver_ldm);
}

static void __exit anatop_thermal_exit(void)
{
	platform_driver_unregister(&anatop_thermal_driver_ldm);
	pr_debug("Anatop Thermal driver successfully unloaded\n");
}

module_init(anatop_thermal_init);
module_exit(anatop_thermal_exit);
