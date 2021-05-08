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
#ifndef _LINUX_ION_IMX_H
#define _LINUX_ION_IMX_H
#include <linux/ion.h>

struct ion_imx_phys_data {
	int dmafd;
	unsigned long phys;
};

#define ION_GET_PHYS   _IOWR(ION_IOC_MAGIC, 15, struct ion_imx_phys_data)

#endif
