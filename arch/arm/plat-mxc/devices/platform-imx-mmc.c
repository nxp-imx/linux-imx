/*
 * Copyright (C) 2009-2010 Pengutronix
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
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
#include <asm/sizes.h>
#include <mach/devices-common.h>

struct platform_device *__init imx_add_imx_mmc(int id,
		resource_size_t iobase, resource_size_t iosize, int irq1,
		int irq2, const struct mxc_mmc_platform_data *pdata)
{
	struct resource res[] = {
		{
			.start = iobase,
			.end = iobase + iosize - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.start = irq1,
			.end = irq1,
			.flags = IORESOURCE_IRQ,
		}, {
			.start = irq2,
			.end = irq2,
			.flags = IORESOURCE_IRQ,
		}
	};

	return imx_add_platform_device("mxsdhci", id, res, ARRAY_SIZE(res),
			pdata, sizeof(*pdata));
}
