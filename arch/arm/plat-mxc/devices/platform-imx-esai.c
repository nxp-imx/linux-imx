/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */

#include <mach/hardware.h>
#include <mach/devices-common.h>

#define imx53_esai_data_entry(soc, _id, _size)			\
	[_id] = {							\
		.id = _id,						\
		.iobase = MX53_ESAI_BASE_ADDR,		\
		.iosize = _size,					\
		.irq = MX53_INT_ESAI,			\
		.dmatx = soc ## _DMA_REQ_ESAI ## _TX,		\
		.dmarx = soc ## _DMA_REQ_ESAI ## _RX,		\
	}

#define imx6q_esai_data_entry(soc, _id, _size)			\
	[_id] = {							\
		.id = _id,						\
		.iobase = ESAI1_BASE_ADDR,		\
		.iosize = _size,					\
		.irq = MXC_INT_ESAI,			\
		.dmatx = soc ## _DMA_REQ_ESAI ## _TX,		\
		.dmarx = soc ## _DMA_REQ_ESAI ## _RX,		\
	}

#ifdef CONFIG_SOC_IMX53
const struct imx_imx_esai_data imx53_imx_esai_data[] __initconst = {
#define imx53_imx_esai_data_entry(_id)				\
		imx53_esai_data_entry(MX53, _id, SZ_4K)
		imx53_imx_esai_data_entry(0),
};
#endif /* ifdef CONFIG_SOC_IMX53 */

#ifdef CONFIG_SOC_IMX6Q
const struct imx_imx_esai_data imx6q_imx_esai_data[] __initconst = {
#define imx6q_imx_esai_data_entry(_id)				\
	imx6q_esai_data_entry(MX6Q, _id, SZ_4K)
	imx6q_imx_esai_data_entry(0),
};
#endif /* ifdef CONFIG_SOC_IMX6Q */

struct platform_device *__init imx_add_imx_esai(
		const struct imx_imx_esai_data *data,
		const struct imx_esai_platform_data *pdata)
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
#define DMARES(_name) {							\
	.name = #_name,							\
	.start = data->dma ## _name,					\
	.end = data->dma ## _name,					\
	.flags = IORESOURCE_DMA,					\
}
		DMARES(tx),
		DMARES(rx),
	};

	return imx_add_platform_device("imx-esai", data->id,
			res, ARRAY_SIZE(res),
			pdata, sizeof(*pdata));
}
