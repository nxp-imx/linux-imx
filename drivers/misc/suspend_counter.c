/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/delay.h>


static int suspend_counter;
static ktime_t suspend_time;
static ktime_t tmp_time;

static int suspend_counter_notifier(struct notifier_block *nb,
				   unsigned long event, void *dummy)
{
	struct timespec tv;
	switch (event) {
	case PM_SUSPEND_PREPARE:
		suspend_counter++;
		suspend_time = ktime_get_real();
		printk(KERN_INFO "suspend: start %d suspend...\n",
		       suspend_counter);
	case PM_POST_SUSPEND:
		tmp_time = ktime_sub(ktime_get_real(), suspend_time);
		tv = ktime_to_timespec(tmp_time);
		printk(KERN_INFO "suspend: finish %d suspend "
		       "after:%lu.%03lu seconds...\n",
		       suspend_counter,
		       (unsigned long) tv.tv_sec,
		       (unsigned long) tv.tv_nsec);
	default:
		break;
	}
	return 0;
}

static struct notifier_block suspend_counter_notifier_block = {
	.notifier_call = suspend_counter_notifier,
};

static int __init suspend_counter_init(void)
{
	return register_pm_notifier(&suspend_counter_notifier_block);
}

device_initcall(suspend_counter_init);
