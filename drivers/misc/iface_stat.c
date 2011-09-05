/* drivers/misc/iface_stat.c
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

#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/in.h>
#include <linux/list.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/netdevice.h>
#include <linux/inetdevice.h>
#include <linux/rtnetlink.h>
#include <linux/iface_stat.h>

static LIST_HEAD(iface_list);
static struct proc_dir_entry *iface_stat_procdir;

struct iface_stat {
	struct list_head if_link;
	char *iface_name;
	unsigned long tx_bytes;
	unsigned long rx_bytes;
	unsigned long tx_packets;
	unsigned long rx_packets;
	bool active;
};

static int read_proc_entry(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len;
	unsigned long value;
	char *p = page;
	unsigned long *iface_entry = (unsigned long *) data;
	if (!data)
		return 0;

	value = (unsigned long) (*iface_entry);
	p += sprintf(p, "%lu\n", value);
	len = (p - page) - off;
	*eof = (len <= count) ? 1 : 0;
	*start = page + off;
	return len;
}

static int read_proc_bool_entry(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len;
	bool value;
	char *p = page;
	unsigned long *iface_entry = (unsigned long *) data;
	if (!data)
		return 0;

	value = (bool) (*iface_entry);
	p += sprintf(p, "%u\n", value ? 1 : 0);
	len = (p - page) - off;
	*eof = (len <= count) ? 1 : 0;
	*start = page + off;
	return len;
}

/* Find the entry for tracking the specified interface. */
static struct iface_stat *get_iface_stat(const char *ifname)
{
	struct iface_stat *iface_entry;
	if (!ifname)
		return NULL;

	list_for_each_entry(iface_entry, &iface_list, if_link) {
		if (!strcmp(iface_entry->iface_name, ifname))
			return iface_entry;
	}
	return NULL;
}

/*
 * Create a new entry for tracking the specified interface.
 * Do nothing if the entry already exists.
 * Called when an interface is configured with a valid IP address.
 */
void create_iface_stat(const struct in_device *in_dev)
{
	struct iface_stat *new_iface;
	struct proc_dir_entry *proc_entry;
	const struct net_device *dev;
	const char *ifname;
	struct iface_stat *entry;
	__be32 ipaddr = 0;
	struct in_ifaddr *ifa = NULL;

	ASSERT_RTNL(); /* No need for separate locking */

	dev = in_dev->dev;
	if (!dev) {
		pr_err("iface_stat: This should never happen.\n");
		return;
	}

	ifname = dev->name;
	for (ifa = in_dev->ifa_list; ifa; ifa = ifa->ifa_next)
		if (!strcmp(dev->name, ifa->ifa_label))
			break;

	if (ifa)
		ipaddr = ifa->ifa_local;
	else {
		pr_err("iface_stat: Interface not found.\n");
		return;
	}

	entry = get_iface_stat(dev->name);
	if (entry != NULL) {
		pr_debug("iface_stat: Already monitoring device %s\n", ifname);
		if (ipv4_is_loopback(ipaddr)) {
			entry->active = false;
			pr_debug("iface_stat: Disabling monitor for "
					"loopback device %s\n", ifname);
		} else {
			entry->active = true;
			pr_debug("iface_stat: Re-enabling monitor for "
					"device %s with ip %pI4\n",
					ifname, &ipaddr);
		}
		return;
	} else if (ipv4_is_loopback(ipaddr)) {
		pr_debug("iface_stat: Ignoring monitor for "
				"loopback device %s with ip %pI4\n",
				ifname, &ipaddr);
		return;
	}

	/* Create a new entry for tracking the specified interface. */
	new_iface = kmalloc(sizeof(struct iface_stat), GFP_KERNEL);
	if (new_iface == NULL)
		return;

	new_iface->iface_name = kmalloc((strlen(ifname)+1)*sizeof(char),
						GFP_KERNEL);
	if (new_iface->iface_name == NULL) {
		kfree(new_iface);
		return;
	}

	strcpy(new_iface->iface_name, ifname);
	/* Counters start at 0, so we can track 4GB of network traffic. */
	new_iface->tx_bytes = 0;
	new_iface->rx_bytes = 0;
	new_iface->rx_packets = 0;
	new_iface->tx_packets = 0;
	new_iface->active = true;

	/* Append the newly created iface stat struct to the list. */
	list_add_tail(&new_iface->if_link, &iface_list);
	proc_entry = proc_mkdir(ifname, iface_stat_procdir);

	/* Keep reference to iface_stat so we know where to read stats from. */
	create_proc_read_entry("tx_bytes", S_IRUGO, proc_entry,
			read_proc_entry, &new_iface->tx_bytes);

	create_proc_read_entry("rx_bytes", S_IRUGO, proc_entry,
			read_proc_entry, &new_iface->rx_bytes);

	create_proc_read_entry("tx_packets", S_IRUGO, proc_entry,
			read_proc_entry, &new_iface->tx_packets);

	create_proc_read_entry("rx_packets", S_IRUGO, proc_entry,
			read_proc_entry, &new_iface->rx_packets);

	create_proc_read_entry("active", S_IRUGO, proc_entry,
			read_proc_bool_entry, &new_iface->active);

	pr_debug("iface_stat: Now monitoring device %s with ip %pI4\n",
			ifname, &ipaddr);
}

/*
 * Update stats for the specified interface. Do nothing if the entry
 * does not exist (when a device was never configured with an IP address).
 * Called when an device is being unregistered.
 */
void iface_stat_update(struct net_device *dev)
{
	const struct net_device_stats *stats = dev_get_stats(dev);
	struct iface_stat *entry;

	ASSERT_RTNL();

	entry = get_iface_stat(dev->name);
	if (entry == NULL) {
		pr_debug("iface_stat: dev %s monitor not found\n", dev->name);
		return;
	}

	if (entry->active) { /* FIXME: Support for more than 4GB */
		entry->tx_bytes += stats->tx_bytes;
		entry->tx_packets += stats->tx_packets;
		entry->rx_bytes += stats->rx_bytes;
		entry->rx_packets += stats->rx_packets;
		entry->active = false;
		pr_debug("iface_stat: Updating stats for "
			       "dev %s which went down\n", dev->name);
	} else
		pr_debug("iface_stat: Didn't update stats for "
				"dev %s which went down\n", dev->name);
}

static int __init iface_stat_init(void)
{
	iface_stat_procdir = proc_mkdir("iface_stat", NULL);
	if (!iface_stat_procdir) {
		pr_err("iface_stat: failed to create proc entry\n");
		return -1;
	}

	return 0;
}

device_initcall(iface_stat_init);
