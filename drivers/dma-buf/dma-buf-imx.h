/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright 2021 NXP
 */

#ifndef _LINUX_DMABUF_IMX_H
#define _LINUX_DMABUF_IMX_H

struct dmabuf_imx_phys_data {
	__u32 dmafd;
	uint64_t phys;
};

#define DMABUF_GET_PHYS   _IOWR('M', 32, struct dmabuf_imx_phys_data)

#endif
