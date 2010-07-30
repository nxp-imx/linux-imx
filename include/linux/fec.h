/* include/linux/fec.h
 *
 * Copyright (c) 2009 Orex Computed Radiography
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc.
 *   Baruch Siach <baruch@tkos.co.il>
 *
 * Header file for the FEC platform data
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __LINUX_FEC_H__
#define __LINUX_FEC_H__

#include <linux/etherdevice.h>
#include <linux/phy.h>

struct fec_platform_data {
	phy_interface_t phy;
	unsigned char mac[ETH_ALEN];
	int (*init)(void);
	int (*uninit)(void);
	struct regulator *vddio_reg;
};

struct switch_platform_data {
	int	id;
	int     hash_table;
	unsigned int *switch_hw;
	struct fec_platform_data *fec_enet;
	void    (*request_intrs)(struct net_device *dev,
		irqreturn_t (*)(int, void *),
		void *irq_privatedata);
	void    (*set_mii)(struct net_device *dev);
	void    (*get_mac)(struct net_device *dev);
	void    (*enable_phy_intr)(void);
	void    (*disable_phy_intr)(void);
	void    (*phy_ack_intr)(void);
	void    (*localhw_setup)(void);
	void    (*uncache)(unsigned long addr);
	void    (*platform_flush_cache)(void);
};

#endif
