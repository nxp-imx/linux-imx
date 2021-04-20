/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright 2021 NXP
 */

#ifndef _LINUX_ION_IMX_H
#define _LINUX_ION_IMX_H
#include <linux/ion.h>

struct ion_imx_phys_data {
	int dmafd;
	unsigned long phys;
};

#define ION_GET_PHYS   _IOWR(ION_IOC_MAGIC, 15, struct ion_imx_phys_data)

#endif
