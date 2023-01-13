/*
 * Copyright 2021 NXP
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
 *
 */
#ifndef _LINUX_DMABUF_IMX_H
#define _LINUX_DMABUF_IMX_H

#include <linux/types.h>

struct dmabuf_imx_phys_data {
	__u32 dmafd;
	__u64 phys;
};
struct dmabuf_imx_heap_name {
	__u32 dmafd;
	__u8 name[32];
};

#define DMABUF_GET_PHYS   _IOWR('M', 32, struct dmabuf_imx_phys_data)
#define DMABUF_GET_HEAP_NAME   _IOWR('M', 33, struct dmabuf_imx_heap_name)

#endif
