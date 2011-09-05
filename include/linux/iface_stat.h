/* include/linux/iface_stat.h
 *
 * Copyright (C) 2011 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __IFACE_STAT_H
#define __IFACE_STAT_H

/*
 * Contains definitions for persistent data usage tracking per
 * network interface.
 */

#ifdef CONFIG_IFACE_STAT
/*
 * Create a new entry for tracking the specified interface.
 * Do nothing if the entry already exists.
 * Called when an interface is configured with a valid IP address.
 */
void create_iface_stat(const struct in_device *in_dev);

/*
 * Update stats for the specified interface.
 * Do nothing if the entry does not exist (when a device was never
 * configured with an IP address). Called when an device is being
 * unregistered.
 */
void iface_stat_update(struct net_device *dev);

#else

static inline void create_iface_stat(in_dev)
{ }

static inline void iface_stat_update(dev)
{ }

#endif

#endif /* __IFACE_STAT_H */
