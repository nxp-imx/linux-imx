/*
 *  anatop_thermal.c - anatop Thermal Zone Driver ($Revision: 41 $)
 *
 *  Copyright (C) 2001, 2002 Andy Grover <andrew.grover@intel.com>
 *  Copyright (C) 2001, 2002 Paul Diefenbaugh <paul.s.diefenbaugh@intel.com>
 *  Copyright (C) 2011 Freescale Semiconductor, Inc.
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
 *
 *  This driver fully implements the anatop thermal policy as described in the
 *  anatop 2.0 Specification.
 *
 *  TBD: 1. Implement passive cooling hysteresis.
 *       2. Enhance passive cooling (CPU) states/limit interface to support
 *          concepts of 'multiple limiters', upper/lower limits, etc.
 *
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


#define PREFIX "ANATOP: "

#define ANATOP_THERMAL_DRIVER_NAME	"Anatop Thermal"
#define ANATOP_THERMAL_DEVICE_NAME	"Thermal Zone"
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

#define _COMPONENT		ANATOP_THERMAL_COMPONENT
#define POLLING_FREQ			20
#define TEMP_CRITICAL_SHUTDOWN	380
#define TEMP_CRITICAL_SLEEP		360
#define TEMP_PASSIVE			320
#define MEASURE_FREQ			327  /* 327 RTC clocks delay, 10ms */
#define CONVER_CONST			1413  /* rough value, need calibration */
#define CONVER_DIV_1024			550  /* div value, need calibration */

#define ANATOP_DEBUG	false

MODULE_AUTHOR("Anson Huang");
MODULE_DESCRIPTION("ANATOP Thermal Zone Driver");
MODULE_LICENSE("GPL");

static int psv;
unsigned long anatop_base;

module_param(psv, int, 0644);
MODULE_PARM_DESC(psv, "Disable or override all passive trip points.");

static int anatop_thermal_add(struct anatop_device *device);
static int anatop_thermal_remove(struct platform_device *pdev);
static int anatop_thermal_suspend(struct platform_device *pdev,
	pm_message_t state);
static int anatop_thermal_resume(struct platform_device *pdev);
/* static void anatop_thermal_notify(struct anatop_device *device,
 * u32 event); */
static int anatop_thermal_power_on(void);
static int anatop_thermal_power_down(void);

static const struct anatop_device_id thermal_device_ids[] = {
	{ANATOP_THERMAL_HID, 0},
	{"", 0},
};
MODULE_DEVICE_TABLE(anatop, thermal_device_ids);

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
	struct anatop_thermal_active active[ANATOP_THERMAL_MAX_ACTIVE];
};

struct anatop_thermal_flags {
	u8 cooling_mode:1;	/* _SCP */
	u8 devices:1;		/* _TZD */
	u8 reserved:6;
};

struct anatop_thermal {
	struct anatop_device *device;
	unsigned long temperature;
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

/* -----------------------------------------------------------
			Thermal Zone Management
   ----------------------------------------------------------- */
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
static int anatop_tempsense_value_to_dc(int val)
{
	int val_dc;
	val_dc = (CONVER_CONST - val) * CONVER_DIV_1024 / 1024;

	return val_dc;
}
static int anatop_thermal_get_temperature(struct anatop_thermal *tz)
{
	unsigned int tmp = 300;
	unsigned int reg;
	unsigned int i;
	if (!tz)
		return -EINVAL;

	tz->last_temperature = tz->temperature;

	/* now we only using single measure, every time we measure
	the temperature, we will power on/down the anadig module*/
	anatop_thermal_power_on();

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
	tz->temperature = anatop_tempsense_value_to_dc(tmp);

	pr_debug("Temperature is %lu C\n", tz->temperature);
	anatop_thermal_power_down();

	return 0;
}

static int anatop_thermal_get_polling_frequency(struct anatop_thermal *tz)
{
	/* treat the polling interval as freq */
	tz->polling_frequency = POLLING_FREQ;

	return 0;
}

static int anatop_thermal_set_cooling_mode(struct anatop_thermal *tz, int mode)
{

	return 0;
}

#define ANATOP_TRIPS_CRITICAL	0x01
#define ANATOP_TRIPS_HOT		0x02
#define ANATOP_TRIPS_PASSIVE	0x04
#define ANATOP_TRIPS_ACTIVE	    0x08
#define ANATOP_TRIPS_DEVICES	0x10

#define ANATOP_TRIPS_INIT      (ANATOP_TRIPS_CRITICAL | \
		ANATOP_TRIPS_HOT | ANATOP_TRIPS_PASSIVE)

static int anatop_thermal_trips_update(struct anatop_thermal *tz, int flag)
{
	/* Critical Shutdown */
	if (flag & ANATOP_TRIPS_CRITICAL) {
		tz->trips.critical.temperature = TEMP_CRITICAL_SHUTDOWN;
		tz->trips.critical.flags.valid = 1;

	}

	/* Critical Sleep(HOT) (optional) */
	if (flag & ANATOP_TRIPS_HOT) {
			tz->trips.hot.temperature = TEMP_CRITICAL_SLEEP;
			tz->trips.hot.flags.valid = 1;

	}

	/* Passive (optional) */
	if (flag & ANATOP_TRIPS_PASSIVE) {
			tz->trips.passive.temperature = TEMP_PASSIVE;
			tz->trips.passive.flags.valid = 1;
	}

	return 0;
}

static int anatop_thermal_get_trip_points(struct anatop_thermal *tz)
{
	int i, valid, ret = anatop_thermal_trips_update(tz, ANATOP_TRIPS_INIT);

	if (ret)
		return ret;

	valid = tz->trips.critical.flags.valid |
		tz->trips.hot.flags.valid |
		tz->trips.passive.flags.valid;

	for (i = 0; i < ANATOP_THERMAL_MAX_ACTIVE; i++)
		valid |= tz->trips.active[i].flags.valid;

	if (!valid) {
		printk(KERN_WARNING FW_BUG "No valid trip found\n");
		return -ENODEV;
	}
	return 0;
}

static void anatop_thermal_check(void *data)
{
	struct anatop_thermal *tz = data;

	thermal_zone_device_update(tz->thermal_zone);
}

/* sys I/F for generic thermal sysfs support */
#define KELVIN_TO_CEL(t, off) (((t) - (off)))

static int thermal_get_temp(struct thermal_zone_device *thermal,
			    unsigned long *temp)
{
	struct anatop_thermal *tz = thermal->devdata;
	int result;

	if (!tz)
		return -EINVAL;
	result = 0;
	result = anatop_thermal_get_temperature(tz);
	*temp = tz->temperature;
	if (result)
		return result;
	else
		return 0;
}
static const char enabled[] = "kernel";
static const char disabled[] = "user";
static int thermal_get_mode(struct thermal_zone_device *thermal,
				enum thermal_device_mode *mode)
{
	struct anatop_thermal *tz = thermal->devdata;

	if (!tz)
		return -EINVAL;

	*mode = tz->tz_enabled ? THERMAL_DEVICE_ENABLED :
		THERMAL_DEVICE_DISABLED;

	return 0;
}

static int thermal_set_mode(struct thermal_zone_device *thermal,
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
		/*
		printk((KERN_INFO,
			"%s anatop thermal control\n",
			tz->tz_enabled ? enabled : disabled));
		*/
		anatop_thermal_check(tz);
	}
	return 0;
}

static int thermal_get_trip_type(struct thermal_zone_device *thermal,
				 int trip, enum thermal_trip_type *type)
{
	struct anatop_thermal *tz = thermal->devdata;
	int i;

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

	for (i = 0; i < ANATOP_THERMAL_MAX_ACTIVE &&
		tz->trips.active[i].flags.valid; i++) {
		if (!trip) {
			*type = THERMAL_TRIP_ACTIVE;
			return 0;
		}
		trip--;
	}

	return -EINVAL;
}

static int thermal_get_trip_temp(struct thermal_zone_device *thermal,
				 int trip, unsigned long *temp)
{
	struct anatop_thermal *tz = thermal->devdata;
	int i;

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

	for (i = 0; i < ANATOP_THERMAL_MAX_ACTIVE &&
		tz->trips.active[i].flags.valid; i++) {
		if (!trip) {
			*temp = KELVIN_TO_CEL(
				tz->trips.active[i].temperature,
				tz->kelvin_offset);
			return 0;
		}
		trip--;
	}

	return -EINVAL;
}

static int thermal_get_crit_temp(struct thermal_zone_device *thermal,
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

static int thermal_notify(struct thermal_zone_device *thermal, int trip,
			   enum thermal_trip_type trip_type)
{
	u8 type = 0;
	/* struct anatop_thermal *tz = thermal->devdata; */

	if (trip_type == THERMAL_TRIP_CRITICAL)
		type = ANATOP_THERMAL_NOTIFY_CRITICAL;
	else if (trip_type == THERMAL_TRIP_HOT)
		type = ANATOP_THERMAL_NOTIFY_HOT;
	else
		return 0;
/*
	if (trip_type == THERMAL_TRIP_CRITICAL && nocrt)
		return 1;
*/
	return 0;
}

typedef int (*cb)(struct thermal_zone_device *, int,
		  struct thermal_cooling_device *);
static int anatop_thermal_cooling_device_cb(struct thermal_zone_device *thermal,
					struct thermal_cooling_device *cdev,
					cb action)
{
	/* struct anatop_thermal *tz = thermal->devdata; */

	int result = 0;

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
	.get_temp = thermal_get_temp,
	.get_mode = thermal_get_mode,
	.set_mode = thermal_set_mode,
	.get_trip_type = thermal_get_trip_type,
	.get_trip_temp = thermal_get_trip_temp,
	.get_crit_temp = thermal_get_crit_temp,
	.notify = thermal_notify,
};

static int anatop_thermal_register_thermal_zone(struct anatop_thermal *tz)
{
	int trips = 0;
	int i;

	if (tz->trips.critical.flags.valid)
		trips++;

	if (tz->trips.hot.flags.valid)
		trips++;

	if (tz->trips.passive.flags.valid)
		trips++;

	for (i = 0; i < ANATOP_THERMAL_MAX_ACTIVE
			&& tz->trips.active[i].flags.valid; i++, trips++)
		;

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
						     tz->polling_frequency*100);
#if 0
	result = sysfs_create_link(&tz->device->dev.kobj,
				&tz->thermal_zone->device.kobj, "thermal_zone");
	if (result)
		return result;

	result = sysfs_create_link(&tz->thermal_zone->device.kobj,
				   &tz->device->dev.kobj, "device");
	if (result)
		return result;
#endif

	tz->tz_enabled = 1;

	dev_info(&tz->device->dev, "registered as thermal_zone%d\n",
		 tz->thermal_zone->id);
	return 0;
}
/*
static void anatop_thermal_unregister_thermal_zone(struct anatop_thermal *tz)
{
	sysfs_remove_link(&tz->device->dev.kobj, "thermal_zone");
	sysfs_remove_link(&tz->thermal_zone->device.kobj, "device");
	thermal_zone_device_unregister(tz->thermal_zone);
	tz->thermal_zone = NULL;
}
*/

/* --------------------------------------------------------------
						Driver Interface
   ------------------------------------------------------------- */
static int anatop_thermal_get_info(struct anatop_thermal *tz)
{
	int result = 0;

	if (!tz)
		return -EINVAL;

	/* Get temperature [_TMP] (required) */
	result = anatop_thermal_get_temperature(tz);
	if (result)
		return result;

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
	anatop_thermal_get_polling_frequency(tz);

	return 0;
}

static void anatop_thermal_guess_offset(struct anatop_thermal *tz)
{
	tz->kelvin_offset = 273;
}

static int anatop_thermal_add(struct anatop_device *device)
{
	int result = 0;
	struct anatop_thermal *tz = NULL;
	if (!device)
		return -EINVAL;

	tz = kzalloc(sizeof(struct anatop_thermal), GFP_KERNEL);
	if (!tz)
		return -ENOMEM;

	tz->device = device;
	strcpy(device->name, ANATOP_THERMAL_DEVICE_NAME);
	device->driver_data = tz;
	mutex_init(&tz->lock);

	result = anatop_thermal_get_info(tz);
	if (result)
		goto free_memory;

	anatop_thermal_guess_offset(tz);

	result = anatop_thermal_register_thermal_zone(tz);
	if (result)
		goto free_memory;

	goto end;

free_memory:
	kfree(tz);
end:
	return result;
}
static int anatop_thermal_remove(struct platform_device *pdev)
{
	/* struct anatop_device *device = platform_get_drvdata(pdev); */
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
static int anatop_thermal_power_on(void)
{
	__raw_writel(BM_ANADIG_TEMPSENSE0_POWER_DOWN,
			anatop_base + HW_ANADIG_TEMPSENSE0_CLR);

	__raw_writel(BM_ANADIG_ANA_MISC0_REFTOP_SELBIASOFF,
			anatop_base + HW_ANADIG_ANA_MISC0_SET);
	return 0;
}

static int anatop_thermal_power_down(void)
{
	__raw_writel(BM_ANADIG_TEMPSENSE0_POWER_DOWN,
			anatop_base + HW_ANADIG_TEMPSENSE0_SET);

	__raw_writel(BM_ANADIG_ANA_MISC0_REFTOP_SELBIASOFF,
			anatop_base + HW_ANADIG_ANA_MISC0_CLR);
	return 0;
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
		dev_err(&pdev->dev, "No anatop base address provided\n");
		goto anatop_failed;
	}

	base = ioremap(res->start, res->end - res->start);
	if (base == NULL) {
		dev_err(&pdev->dev, "failed to rebase anatop base address\n");
		goto anatop_failed;
	}
	anatop_base = (unsigned long)base;

	anatop_thermal_add(device);

	goto success;
anatop_failed:
	kfree(device);
device_alloc_failed:
success:
	pr_info("%s\n", __func__);
	return retval;
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
