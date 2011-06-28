/*
 * Copyright (C) 2010 Pengutronix
 * Uwe Kleine-Koenig <u.kleine-koenig@pengutronix.de>
 *
 * Copyright (C) 2011 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */
#include <mach/hardware.h>
#include <mach/devices-common.h>

#define MXC_SPDIF_TX_REG          0x2C
#define MXC_SPDIF_RX_REG          0x14

#define imx_spdif_dai_data_entry(soc)					\
	{								\
		.iobase = soc ## _SPDIF_BASE_ADDR,			\
		.dmatx = soc ## _DMA_REQ_SPDIF_TX,			\
		.dmarx = soc ## _DMA_REQ_SPDIF_RX,			\
	}

#ifdef CONFIG_SOC_IMX51
const struct imx_spdif_dai_data imx51_spdif_dai_data __initconst = {
		.iobase = MX51_SPDIF_BASE_ADDR,
		.dmatx = MX51_DMA_REQ_SPDIF,
	};
#endif /* ifdef CONFIG_SOC_IMX51 */

#ifdef CONFIG_SOC_IMX53
const struct imx_spdif_dai_data imx53_spdif_dai_data __initconst =
			imx_spdif_dai_data_entry(MX53);
#endif /* ifdef CONFIG_SOC_IMX53 */

#ifdef CONFIG_SOC_IMX6Q
const struct imx_spdif_dai_data imx6q_spdif_dai_data __initconst =
			imx_spdif_dai_data_entry(MX6Q);
#endif /* #ifdef CONFIG_SOC_IMX6Q */

struct platform_device *__init imx_add_spdif_dai(
			const struct imx_spdif_dai_data *data)
{
	struct resource res[] = {
		{
			.name = "tx_reg",
			.start = data->iobase + MXC_SPDIF_TX_REG,
			.end = data->iobase + MXC_SPDIF_TX_REG,
			.flags = IORESOURCE_DMA,
		}, {
			.name = "rx_reg",
			.start = data->iobase + MXC_SPDIF_RX_REG,
			.end = data->iobase + MXC_SPDIF_RX_REG,
			.flags = IORESOURCE_DMA,
		}, {
			.name = "tx",
			.start = data->dmatx,
			.end = data->dmatx,
			.flags = IORESOURCE_DMA,
		}, {
			.name = "rx",
			.start = data->dmarx,
			.end = data->dmarx,
			.flags = IORESOURCE_DMA,
		},
	};

	return imx_add_platform_device("imx-spdif-dai", 0,
				res, ARRAY_SIZE(res), NULL, 0);
}
