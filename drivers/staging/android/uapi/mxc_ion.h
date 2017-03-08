/*
 * drivers/gpu/mxc/mxc_ion.h
 *
 * Copyright (C) 2011 Google, Inc.
 * Copyright (C) 2012-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP.
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

#ifndef _LINUX_MXC_ION_H
#define _LINUX_MXC_ION_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define ION_CMA_HEAP_ID     0

struct ion_phys_data {
	ion_user_handle_t handle;
	unsigned long phys;
};

struct ion_phys_dma_data {
	unsigned long phys;
	size_t size;
	int dmafd;
};

struct ion_phys_virt_data {
	unsigned long virt;
	unsigned long phys;
	size_t size;
};

#define ION_IOC_PHYS   _IOWR(ION_IOC_MAGIC, 8, struct ion_phys_data)

#define ION_IOC_PHYS_DMA   _IOWR(ION_IOC_MAGIC, 9, struct ion_phys_dma_data)

#define ION_IOC_PHYS_VIRT   _IOWR(ION_IOC_MAGIC, 10, struct ion_phys_virt_data)

#endif
