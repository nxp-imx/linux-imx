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

#include <gsl.h>

#include <linux/timer.h>
#include <linux/spinlock.h>
#include <linux/slab.h>

typedef struct _gsl_autogate_t {
    struct timer_list timer;	
    spinlock_t lock;
    int active;
    int timeout;
    gsl_device_t *dev;
} gsl_autogate_t;


#define KGSL_DEVICE_IDLE_TIMEOUT 5000	/* unit ms */

int kgsl_device_active(gsl_device_t *dev)
{
	unsigned long flags;
	gsl_autogate_t *autogate = dev->autogate;
	if (!autogate) {
		printk(KERN_ERR "%s: autogate has exited!\n", __func__);
		return 0;
	}
//	printk(KERN_ERR "%s:%d id %d active %d\n", __func__, __LINE__, dev->id, autogate->active);

	spin_lock_irqsave(&autogate->lock, flags);
	if (!autogate->active)
		kgsl_clock(autogate->dev->id, 1);
	autogate->active = 1;
	mod_timer(&autogate->timer, jiffies + msecs_to_jiffies(autogate->timeout));
	spin_unlock_irqrestore(&autogate->lock, flags);
	return 0;
}

static void kgsl_device_inactive(unsigned long data)
{
	gsl_autogate_t *autogate = (gsl_autogate_t *)data;
	unsigned long flags;

//	printk(KERN_ERR "%s:%d id %d active %d\n", __func__, __LINE__, autogate->dev->id, autogate->active);
	del_timer(&autogate->timer);
	spin_lock_irqsave(&autogate->lock, flags);
	WARN(!autogate->active, "GPU Device %d is already inactive\n", autogate->dev->id);
	autogate->active = 0;
	/* idle check may sleep, so don't use it */
//	if (autogate->dev->ftbl.device_idle) 
//		autogate->dev->ftbl.device_idle(autogate->dev, GSL_TIMEOUT_DEFAULT);
	kgsl_clock(autogate->dev->id, 0);
	spin_unlock_irqrestore(&autogate->lock, flags);
}

int kgsl_device_clock(gsl_deviceid_t id, int enable)
{
	int ret = GSL_SUCCESS;
	gsl_device_t *device;

	device = &gsl_driver.device[id-1];       // device_id is 1 based
	if (device->flags & GSL_FLAGS_INITIALIZED) {
		if (enable)
			kgsl_device_active(device);
		else
			kgsl_device_inactive((unsigned long)device);
	} else {
		printk(KERN_ERR "%s: Dev %d clock is already off!\n", __func__, id);
		ret = GSL_FAILURE;
	}
	
	return ret;
}

int kgsl_device_autogate_init(gsl_device_t *dev)
{
	gsl_autogate_t *autogate;

//	printk(KERN_ERR "%s:%d id %d\n", __func__, __LINE__, dev->id);
	autogate = kmalloc(sizeof(gsl_autogate_t), GFP_KERNEL);
	if (!autogate) {
		printk(KERN_ERR "%s: out of memory!\n", __func__);
		return -ENOMEM;
	}
	autogate->dev = dev;
	autogate->active = 1;
	spin_lock_init(&autogate->lock);
	autogate->timeout = KGSL_DEVICE_IDLE_TIMEOUT;
	init_timer(&autogate->timer);
	autogate->timer.expires = jiffies + msecs_to_jiffies(autogate->timeout);
	autogate->timer.function = kgsl_device_inactive;
	autogate->timer.data = (unsigned long)autogate;
	add_timer(&autogate->timer);
	dev->autogate = autogate;
	return 0;
}

void kgsl_device_autogate_exit(gsl_device_t *dev)
{
	gsl_autogate_t *autogate = dev->autogate;

//	printk(KERN_ERR "%s:%d id %d active %d\n", __func__, __LINE__, dev->id,  autogate->active);
	if (autogate->active)
		del_timer(&autogate->timer);
	else
		kgsl_clock(autogate->dev->id, 1);

	kfree(autogate);
	dev->autogate = NULL;

}
