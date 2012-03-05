/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
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

#include <linux/init.h>
#include <linux/smp.h>
#include <linux/clockchips.h>
#include <asm/smp_twd.h>
#include <asm/localtimer.h>
#include <mach/irqs.h>
#include <mach/hardware.h>


extern bool enable_wait_mode;
/*
 * Setup the local clock events for a CPU.
 */
int __cpuinit local_timer_setup(struct clock_event_device *evt)
{
#ifdef CONFIG_LOCAL_TIMERS
	if (!enable_wait_mode) {
		evt->irq = IRQ_LOCALTIMER;
		twd_timer_setup(evt);
		return 0;
	}
#endif
	return -1;
}
