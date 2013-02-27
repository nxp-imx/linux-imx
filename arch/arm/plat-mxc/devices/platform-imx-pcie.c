/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
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

#include <mach/hardware.h>
#include <mach/devices-common.h>

#define imx_pcie_data_entry_single(soc, _id, _hwid, size)		\
	{								\
		.id = _id,						\
		.iobase = soc ## _PCIE ## _hwid ## _BASE_ADDR,	\
		.iosize = size,						\
		.irq	= soc ## _INT_PCIE ## _hwid,			\
	}

#define imx_pcie_data_entry(soc, _id, _hwid, _size)			\
	[_id] = imx_pcie_data_entry_single(soc, _id, _hwid, _size)

#ifdef CONFIG_SOC_IMX6Q
#define MX6Q_PCIE_BASE_ADDR (PCIE_ARB_END_ADDR - SZ_16K + 1)
#define MX6Q_INT_PCIE MXC_INT_PCIE_3
const struct imx_pcie_data imx6q_pcie_data __initconst =
			imx_pcie_data_entry_single(MX6Q, 0, , SZ_16K);
#endif

struct platform_device *__init imx_add_pcie(
		const struct imx_pcie_data *data,
		const struct imx_pcie_platform_data *pdata)
{
	struct resource res[] = {
		{
			.start = data->iobase,
			.end = data->iobase + data->iosize - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.start = data->irq,
			.end = data->irq,
			.flags = IORESOURCE_IRQ,
		},
	};

	if (!fuse_dev_is_available(MXC_DEV_PCIE))
		return ERR_PTR(-ENODEV);

	return imx_add_platform_device("imx-pcie", -1,
			res, ARRAY_SIZE(res),
			pdata, sizeof(*pdata));
}
