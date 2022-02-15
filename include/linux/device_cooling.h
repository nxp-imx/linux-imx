/*
 * Copyright (C) 2013-2015 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __DEVICE_THERMAL_H__
#define __DEVICE_THERMAL_H__

#include <linux/thermal.h>

#if IS_ENABLED(CONFIG_DEVICE_THERMAL)
int register_devfreq_cooling_notifier(struct notifier_block *nb);
int unregister_devfreq_cooling_notifier(struct notifier_block *nb);
struct thermal_cooling_device *device_cooling_register(struct device_node *np,
							unsigned long states);
void device_cooling_unregister(struct thermal_cooling_device *cdev);
#if defined(CONFIG_ANDROID)
struct thermal_cooling_device *device_gpu_cooling_register(struct device_node *np,
							unsigned long states);
void device_gpu_cooling_unregister(struct thermal_cooling_device *cdev);
#endif
#else
static inline
int register_devfreq_cooling_notifier(struct notifier_block *nb)
{
	return 0;
}

static inline
int unregister_devfreq_cooling_notifier(struct notifier_block *nb)
{
	return 0;
}

static inline
struct thermal_cooling_device *device_cooling_register(struct device_node *np,
                                                        unsigned long states)
{
	return NULL;
}

static inline
void device_cooling_unregister(struct thermal_cooling_device *cdev)
{
	return;
}

#if defined(CONFIG_ANDROID)
static inline
struct thermal_cooling_device *device_gpu_cooling_register(struct device_node *np,
							unsigned long states)
{
	return NULL;
}

static inline
void device_gpu_cooling_unregister(struct thermal_cooling_device *cdev)
{
	return;
}
#endif
#endif
#endif /* __DEVICE_THERMAL_H__ */
