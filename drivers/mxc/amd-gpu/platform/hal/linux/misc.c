/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/hardirq.h>
#include <linux/semaphore.h>

typedef struct _gsl_autogate_t {
    struct timer_list timer;	
    spinlock_t lock;
    int active;
    /* pending indicate the timer has been fired but clock not yet disabled. */
    int pending;
    int timeout;
    gsl_device_t *dev;
    struct work_struct dis_task;
} gsl_autogate_t;

static gsl_autogate_t *g_autogate[2];
static DECLARE_MUTEX(sem_dev);

#define KGSL_DEVICE_IDLE_TIMEOUT 5000	/* unit ms */

static void clk_disable_task(struct work_struct *work)
{
	gsl_autogate_t *autogate;
	autogate = container_of(work, gsl_autogate_t, dis_task);
	if (autogate->dev->ftbl.device_idle)
		autogate->dev->ftbl.device_idle(autogate->dev, GSL_TIMEOUT_DEFAULT);
	kgsl_clock(autogate->dev->id, 0);
	autogate->pending = 0;
}

static int _kgsl_device_active(gsl_device_t *dev, int all)
{
	unsigned long flags;
	int to_active = 0;
	gsl_autogate_t *autogate = dev->autogate;
	if (!autogate) {
		printk(KERN_ERR "%s: autogate has exited!\n", __func__);
		return 0;
	}
//	printk(KERN_ERR "%s:%d id %d active %d\n", __func__, __LINE__, dev->id, autogate->active);

	spin_lock_irqsave(&autogate->lock, flags);
	if (in_interrupt()) {
		if (!autogate->active && !autogate->pending)
			BUG();
	} else {
		to_active = !autogate->active;
		autogate->active = 1;
	}
	mod_timer(&autogate->timer, jiffies + msecs_to_jiffies(autogate->timeout));
	spin_unlock_irqrestore(&autogate->lock, flags);
	if (to_active)
		kgsl_clock(autogate->dev->id, 1);
	if (to_active && all) {
		int index;
		index = autogate->dev->id == GSL_DEVICE_G12 ? GSL_DEVICE_YAMATO - 1 :
			GSL_DEVICE_G12 - 1;
		down(&sem_dev);
		if (g_autogate[index])
			_kgsl_device_active(g_autogate[index]->dev, 0);
		up(&sem_dev);
	}
	return 0;
}
int kgsl_device_active(gsl_device_t *dev)
{
	return _kgsl_device_active(dev, 0);
}

static void kgsl_device_inactive(unsigned long data)
{
	gsl_autogate_t *autogate = (gsl_autogate_t *)data;
	unsigned long flags;

//	printk(KERN_ERR "%s:%d id %d active %d\n", __func__, __LINE__, autogate->dev->id, autogate->active);
	del_timer(&autogate->timer);
	spin_lock_irqsave(&autogate->lock, flags);
	WARN(!autogate->active, "GPU Device %d is already inactive\n", autogate->dev->id);
	if (autogate->active) {
		autogate->active = 0;
		autogate->pending = 1;
		schedule_work(&autogate->dis_task);
	}
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
	autogate = kzalloc(sizeof(gsl_autogate_t), GFP_KERNEL);
	if (!autogate) {
		printk(KERN_ERR "%s: out of memory!\n", __func__);
		return -ENOMEM;
	}
	down(&sem_dev);
	autogate->dev = dev;
	autogate->active = 1;
	spin_lock_init(&autogate->lock);
	autogate->timeout = KGSL_DEVICE_IDLE_TIMEOUT;
	init_timer(&autogate->timer);
	autogate->timer.expires = jiffies + msecs_to_jiffies(autogate->timeout);
	autogate->timer.function = kgsl_device_inactive;
	autogate->timer.data = (unsigned long)autogate;
	add_timer(&autogate->timer);
	INIT_WORK(&autogate->dis_task, clk_disable_task);
	dev->autogate = autogate;
	g_autogate[dev->id - 1] = autogate;
	up(&sem_dev);
	return 0;
}

void kgsl_device_autogate_exit(gsl_device_t *dev)
{
	gsl_autogate_t *autogate = dev->autogate;

//	printk(KERN_ERR "%s:%d id %d active %d\n", __func__, __LINE__, dev->id,  autogate->active);
	down(&sem_dev);
	del_timer_sync(&autogate->timer);
	if (!autogate->active)
		kgsl_clock(autogate->dev->id, 1);
	flush_work(&autogate->dis_task);
	g_autogate[dev->id - 1] = NULL;
	up(&sem_dev);
	kfree(autogate);
	dev->autogate = NULL;
}
