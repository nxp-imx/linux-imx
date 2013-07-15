/*
 *  anatop_thermal.c - anatop Thermal Zone Driver ($Revision: 41 $)
 *
 *  Copyright (C) 2001, 2002 Andy Grover <andrew.grover@intel.com>
 *  Copyright (C) 2001, 2002 Paul Diefenbaugh <paul.s.diefenbaugh@intel.com>
 *  Copyright (C) 2011-2013 Freescale Semiconductor, Inc.
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
#include <linux/interrupt.h>
#include <linux/syscalls.h>
#include <linux/cpufreq.h>
#include <linux/clk.h>
#include "anatop_driver.h"
#include <mach/hardware.h>

/* register define of anatop */
#define HW_ANADIG_ANA_MISC0	(0x00000150)
#define HW_ANADIG_ANA_MISC0_SET	(0x00000154)
#define HW_ANADIG_ANA_MISC0_CLR	(0x00000158)
#define HW_ANADIG_ANA_MISC0_TOG	(0x0000015c)
#define HW_ANADIG_ANA_MISC1	(0x00000160)
#define HW_ANADIG_ANA_MISC1_SET	(0x00000164)
#define HW_ANADIG_ANA_MISC1_CLR	(0x00000168)
#define HW_ANADIG_ANA_MISC1_TOG	(0x0000016c)

#define HW_ANADIG_TEMPSENSE0	(0x00000180)
#define HW_ANADIG_TEMPSENSE0_SET	(0x00000184)
#define HW_ANADIG_TEMPSENSE0_CLR	(0x00000188)
#define HW_ANADIG_TEMPSENSE0_TOG	(0x0000018c)

#define HW_ANADIG_TEMPSENSE1	(0x00000190)
#define HW_ANADIG_TEMPSENSE1_SET	(0x00000194)
#define HW_ANADIG_TEMPSENSE1_CLR	(0x00000198)

#define BM_ANADIG_ANA_MISC0_REFTOP_SELBIASOFF 0x00000008
#define BM_ANADIG_ANA_MISC1_IRQ_TEMPSENSE 0x20000000

#define BP_ANADIG_TEMPSENSE0_ALARM_VALUE      20
#define BM_ANADIG_TEMPSENSE0_ALARM_VALUE 0xFFF00000
#define BF_ANADIG_TEMPSENSE0_ALARM_VALUE(v)  \
	(((v) << 20) & BM_ANADIG_TEMPSENSE0_ALARM_VALUE)
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
#define MEASURE_FREQ			3276  /* 3276 RTC clocks delay, 100ms */
#define KELVIN_TO_CEL(t, off) (((t) - (off)))
#define CEL_TO_KELVIN(t, off) (((t) + (off)))
#define DEFAULT_RATIO			145
#define DEFAULT_N40C			1563
#define REG_VALUE_TO_CEL(ratio, raw) ((raw_n40c - raw) * 100 / ratio - 40)
#define ANATOP_DEBUG			false
#define THERMAL_FUSE_NAME		"/sys/fsl_otp/HW_OCOTP_ANA1"

#define	FACTOR1		15976
#define	FACTOR2		4297157

/* variables */
unsigned long anatop_base;
unsigned int ratio;
unsigned int raw_25c, raw_hot, hot_temp, raw_n40c, raw_125c, raw_critical;
static struct clk *pll3_clk;
static bool full_run = true;
static bool suspend_flag;
static unsigned int thermal_irq;
bool cooling_cpuhotplug;
bool cooling_device_disable;
static bool calibration_valid;
unsigned long temperature_cooling;
static const struct anatop_device_id thermal_device_ids[] = {
	{ANATOP_THERMAL_HID},
	{""},
};
atomic_t thermal_on = ATOMIC_INIT(1);

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
	pr_info("HW_ANADIG_MISC1 = 0x%x\n",
			__raw_readl(anatop_base + HW_ANADIG_ANA_MISC1));
	return 0;
}
static void anatop_update_alarm(unsigned int alarm_value)
{
	if (cooling_device_disable || suspend_flag)
		return;
	/* set alarm value */
	__raw_writel(BM_ANADIG_TEMPSENSE0_ALARM_VALUE,
		anatop_base + HW_ANADIG_TEMPSENSE0_CLR);
	__raw_writel(BF_ANADIG_TEMPSENSE0_ALARM_VALUE(alarm_value),
		anatop_base + HW_ANADIG_TEMPSENSE0_SET);

	return;
}
static int anatop_thermal_get_temp(struct thermal_zone_device *thermal,
			    long *temp)
{
	struct anatop_thermal *tz = thermal->devdata;
	unsigned int tmp;
	unsigned int reg;
	unsigned int val;

	if (!tz)
		return -EINVAL;

	if (!ratio || suspend_flag) {
		*temp = KELVIN_TO_CEL(TEMP_ACTIVE, KELVIN_OFFSET);
		return 0;
	}

	tz->last_temperature = tz->temperature;

	if ((__raw_readl(anatop_base + HW_ANADIG_TEMPSENSE0) &
		BM_ANADIG_TEMPSENSE0_POWER_DOWN) != 0) {
		/* need to keep sensor power up as we enable alarm
		function */
		__raw_writel(BM_ANADIG_TEMPSENSE0_POWER_DOWN,
			anatop_base + HW_ANADIG_TEMPSENSE0_CLR);
		__raw_writel(BM_ANADIG_ANA_MISC0_REFTOP_SELBIASOFF,
			anatop_base + HW_ANADIG_ANA_MISC0_SET);

		/* write measure freq */
		reg = __raw_readl(anatop_base + HW_ANADIG_TEMPSENSE1);
		reg &= ~BM_ANADIG_TEMPSENSE1_MEASURE_FREQ;
		reg |= MEASURE_FREQ;
		__raw_writel(reg, anatop_base + HW_ANADIG_TEMPSENSE1);
	}
	__raw_writel(BM_ANADIG_TEMPSENSE0_MEASURE_TEMP,
		anatop_base + HW_ANADIG_TEMPSENSE0_CLR);
	__raw_writel(BM_ANADIG_TEMPSENSE0_FINISHED,
		anatop_base + HW_ANADIG_TEMPSENSE0_CLR);
	__raw_writel(BM_ANADIG_TEMPSENSE0_MEASURE_TEMP,
		anatop_base + HW_ANADIG_TEMPSENSE0_SET);

	tmp = 0;
	val = jiffies;
	/* read temperature values */
	while ((__raw_readl(anatop_base + HW_ANADIG_TEMPSENSE0)
		& BM_ANADIG_TEMPSENSE0_FINISHED) == 0) {
		if (time_after(jiffies, (unsigned long)(val + HZ / 2))) {
			pr_info("Thermal sensor timeout, retry!\n");
			return 0;
		}
		msleep(10);
	}
	reg = __raw_readl(anatop_base + HW_ANADIG_TEMPSENSE0);
	tmp = (reg & BM_ANADIG_TEMPSENSE0_TEMP_VALUE)
		>> BP_ANADIG_TEMPSENSE0_TEMP_VALUE;
	__raw_writel(BM_ANADIG_TEMPSENSE0_FINISHED,
		anatop_base + HW_ANADIG_TEMPSENSE0_CLR);

	if (ANATOP_DEBUG)
		anatop_dump_temperature_register();
	/* only the temp between -40C and 125C is valid, this
	is for save */
	if (tmp <= raw_n40c && tmp >= raw_125c)
		tz->temperature = REG_VALUE_TO_CEL(ratio, tmp);
	else {
		printk(KERN_WARNING "Invalid temperature, force it to 25C\n");
		tz->temperature = 25;
	}

	if (debug_mask & DEBUG_VERBOSE)
		pr_info("Cooling device Temperature is %lu C\n", tz->temperature);

	*temp = (cooling_device_disable && tz->temperature >= KELVIN_TO_CEL(TEMP_CRITICAL, KELVIN_OFFSET)) ?
			KELVIN_TO_CEL(TEMP_CRITICAL - 1, KELVIN_OFFSET) : tz->temperature;

	/* Set alarm threshold if necessary */
	if ((__raw_readl(anatop_base + HW_ANADIG_TEMPSENSE0) &
		BM_ANADIG_TEMPSENSE0_ALARM_VALUE) == 0)
		anatop_update_alarm(raw_critical);

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
		if (tz->trips.critical.flags.valid) {
			tz->trips.critical.temperature = CEL_TO_KELVIN(
				*temp, tz->kelvin_offset);
			raw_critical = raw_25c - ratio * (*temp - 25) / 100;
			anatop_update_alarm(raw_critical);
		}
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

static BLOCKING_NOTIFIER_HEAD(thermal_chain_head);

int register_thermal_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&thermal_chain_head, nb);
}
EXPORT_SYMBOL_GPL(register_thermal_notifier);

int unregister_thermal_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&thermal_chain_head, nb);
}
EXPORT_SYMBOL_GPL(unregister_thermal_notifier);

int thermal_notifier_call_chain(unsigned long val)
{
	return (blocking_notifier_call_chain(&thermal_chain_head, val, NULL)
		== NOTIFY_BAD) ? -EINVAL : 0;
}
EXPORT_SYMBOL_GPL(thermal_notifier_call_chain);

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
		if (atomic_read(&thermal_on))
			thermal_notifier_call_chain(1);
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
		if (atomic_read(&thermal_on))
			thermal_notifier_call_chain(0);
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

static int __init anatop_thermal_use_calibration(char *str)
{
	calibration_valid = true;
	pr_info("%s: use calibration data for thermal sensor!\n", __func__);

	return 1;
}
__setup("use_calibration", anatop_thermal_use_calibration);

static int anatop_thermal_counting_ratio(unsigned int fuse_data)
{
	int ret = -EINVAL;

	pr_info("Thermal calibration data is 0x%x\n", fuse_data);
	if (fuse_data == 0 || fuse_data == 0xffffffff ||
		(fuse_data & 0xfff00000) == 0) {
		pr_info("%s: invalid calibration data, disable cooling!!!\n", __func__);
		cooling_device_disable = true;
		ratio = DEFAULT_RATIO;
		disable_irq(thermal_irq);
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

	if (!calibration_valid)
		/*
		 * The universal equation for thermal sensor
		 * is slope = 0.4297157 - (0.0015976 * 25C fuse),
		 * here we convert them to integer to make them
		 * easy for counting, FACTOR1 is 15976,
		 * FACTOR2 is 4297157. Our ratio = -100 * slope.
		 */
		ratio = ((FACTOR1 * raw_25c - FACTOR2) + 50000) / 100000;
	else
		ratio = ((raw_25c - raw_hot) * 100) / (hot_temp - 25);

	pr_info("Thermal sensor with ratio = %d\n", ratio);
	raw_n40c = raw_25c + (13 * ratio) / 20;
	raw_125c = raw_25c - ratio;
	/* Init default critical temp to set alarm */
	raw_critical = raw_25c - ratio * (KELVIN_TO_CEL(TEMP_CRITICAL, KELVIN_OFFSET) - 25) / 100;
	clk_enable(pll3_clk);

	return ret;
}

static irqreturn_t anatop_thermal_alarm_handler(int irq, void *dev_id)
{
	char mode = 'r';
	const char *cmd = "reboot";

	if (cooling_device_disable)
		return IRQ_HANDLED;
	printk(KERN_WARNING "\nChip is too hot, reboot!!!\n");
	/* reboot */
	arch_reset(mode, cmd);

	return IRQ_HANDLED;
}

static ssize_t anatop_thermal_flag_show(struct device *dev,
		struct device_attribute *attr, char *buf) {
	return sprintf(buf, "read thermal_hot_flag:%d\n",
			atomic_read(&thermal_on));
}

static ssize_t anatop_thermal_flag_store(struct device *dev,
		struct device_attribute *attr, const char *buf,
		size_t count)
{
	int ret;
	unsigned long data;
	ret = strict_strtoul(buf, 10, &data);
	if (data == 0)
		atomic_set(&thermal_on, 0);
	else
		atomic_set(&thermal_on, 1);
	return count;
}

static struct device_attribute anatop_thermal_flag_dev_attr = {
	.attr = {
		.name = "thermal_hot_flag",
		.mode = S_IRUSR | S_IWUSR,
	},
	.show = anatop_thermal_flag_show,
	.store = anatop_thermal_flag_store,
};

static int anatop_thermal_probe(struct platform_device *pdev)
{
	int retval = 0;
	struct resource *res_io, *res_irq, *res_calibration;
	void __iomem *base, *calibration_addr;
	struct anatop_device *device;

	device = kzalloc(sizeof(*device), GFP_KERNEL);
	if (!device) {
		retval = -ENOMEM;
		goto device_alloc_failed;
	}

	platform_set_drvdata(pdev, device);

	/* ioremap the base address */
	res_io = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res_io == NULL) {
		dev_err(&pdev->dev, "No anatop base address provided!\n");
		goto anatop_failed;
	}

	base = ioremap(res_io->start, res_io->end - res_io->start);
	if (base == NULL) {
		dev_err(&pdev->dev, "failed to remap anatop base address!\n");
		goto anatop_failed;
	}
	anatop_base = (unsigned long)base;
	/* ioremap the calibration data address */
	res_calibration = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res_calibration == NULL) {
		dev_err(&pdev->dev, "No anatop calibration data address provided!\n");
		goto anatop_failed;
	}

	calibration_addr = ioremap(res_calibration->start,
		res_calibration->end - res_calibration->start);
	if (calibration_addr == NULL) {
		dev_err(&pdev->dev, "failed to remap anatop calibration data address!\n");
		goto anatop_failed;
	}

	pll3_clk = clk_get(NULL, "pll3_main_clk");
	if (IS_ERR(pll3_clk)) {
		retval = -ENOENT;
		goto anatop_failed;
	}

	raw_n40c = DEFAULT_N40C;
	/* use calibration data to get ratio */
	anatop_thermal_counting_ratio(__raw_readl(calibration_addr));

	res_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res_irq == NULL) {
		dev_err(&pdev->dev, "No anatop thermal irq provided!\n");
		goto anatop_failed;
	}
	retval = request_irq(res_irq->start, anatop_thermal_alarm_handler, 0, "THERMAL_ALARM_IRQ",
			  NULL);
	thermal_irq = res_irq->start;


	anatop_thermal_add(device);
	anatop_thermal_cpufreq_init();
	retval = device_create_file(&pdev->dev, &anatop_thermal_flag_dev_attr);
	if (retval)
		dev_err(&pdev->dev, "create device file failed!\n");
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
	/* turn off alarm */
	anatop_update_alarm(0);
	suspend_flag = true;
	/* Power down anatop thermal sensor */
	__raw_writel(BM_ANADIG_TEMPSENSE0_MEASURE_TEMP,
		anatop_base + HW_ANADIG_TEMPSENSE0_CLR);
	__raw_writel(BM_ANADIG_TEMPSENSE0_FINISHED,
		anatop_base + HW_ANADIG_TEMPSENSE0_CLR);
	__raw_writel(BM_ANADIG_TEMPSENSE0_POWER_DOWN,
		anatop_base + HW_ANADIG_TEMPSENSE0_SET);
	__raw_writel(BM_ANADIG_ANA_MISC0_REFTOP_SELBIASOFF,
		anatop_base + HW_ANADIG_ANA_MISC0_CLR);
	return 0;
}
static int anatop_thermal_resume(struct platform_device *pdev)
{
	/* Power up anatop thermal sensor */
	__raw_writel(BM_ANADIG_TEMPSENSE0_POWER_DOWN,
		anatop_base + HW_ANADIG_TEMPSENSE0_CLR);
	__raw_writel(BM_ANADIG_ANA_MISC0_REFTOP_SELBIASOFF,
		anatop_base + HW_ANADIG_ANA_MISC0_SET);
	suspend_flag = false;
	/* turn on alarm */
	anatop_update_alarm(raw_critical);
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
