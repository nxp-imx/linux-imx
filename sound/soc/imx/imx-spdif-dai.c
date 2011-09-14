/*
 * ALSA SoC SPDIF Audio Layer for MXS
 *
 * Copyright (C) 2008-2011 Freescale Semiconductor, Inc.
 *
 * Based on stmp3xxx_spdif_dai.c
 * Vladimir Barinov <vbarinov@embeddedalley.com>
 * Copyright 2008 SigmaTel, Inc
 * Copyright 2008 Embedded Alley Solutions, Inc
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program  is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <mach/dma.h>
#include <mach/hardware.h>

#include "../codecs/mxc_spdif.h"

#define MXC_SPDIF_TXFIFO_WML      0x8
#define MXC_SPDIF_RXFIFO_WML      0x8

struct imx_pcm_dma_params dma_params_tx;
struct imx_pcm_dma_params dma_params_rx;

static int imx_spdif_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params,
			       struct snd_soc_dai *cpu_dai)
{
	/* Tx/Rx config */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		snd_soc_dai_set_dma_data(cpu_dai, substream, &dma_params_tx);
	else
		snd_soc_dai_set_dma_data(cpu_dai, substream, &dma_params_rx);

	return 0;
}

struct snd_soc_dai_ops imx_spdif_dai_ops = {
	.hw_params      = imx_spdif_hw_params,
};

struct snd_soc_dai_driver imx_spdif_dai = {
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = MXC_SPDIF_RATES_PLAYBACK,
		.formats = MXC_SPDIF_FORMATS_PLAYBACK,
	},
	.capture = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = MXC_SPDIF_RATES_CAPTURE,
		.formats = MXC_SPDIF_FORMATS_CAPTURE,
	},
	.ops = &imx_spdif_dai_ops,
};

static int imx_spdif_dai_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret = 0;

	dma_params_tx.burstsize = MXC_SPDIF_TXFIFO_WML;
	dma_params_rx.burstsize = MXC_SPDIF_RXFIFO_WML;

	dma_params_tx.peripheral_type = IMX_DMATYPE_SPDIF;
	dma_params_rx.peripheral_type = IMX_DMATYPE_SPDIF;

	res = platform_get_resource_byname(pdev, IORESOURCE_DMA, "tx");
	if (res)
		dma_params_tx.dma = res->start;

	res = platform_get_resource_byname(pdev, IORESOURCE_DMA, "rx");
	if (res)
		dma_params_rx.dma = res->start;

	res = platform_get_resource_byname(pdev, IORESOURCE_DMA, "tx_reg");
	if (res)
		dma_params_tx.dma_addr = res->start;

	res = platform_get_resource_byname(pdev, IORESOURCE_DMA, "rx_reg");
	if (res)
		dma_params_rx.dma_addr = res->start;

	ret = snd_soc_register_dai(&pdev->dev, &imx_spdif_dai);
	if (ret) {
		dev_err(&pdev->dev, "register DAI failed\n");
		goto failed_register;
	}

	return 0;

failed_register:
	return ret;
}

static int __devexit imx_spdif_dai_remove(struct platform_device *pdev)
{
	snd_soc_unregister_dai(&pdev->dev);
	return 0;
}

static struct platform_driver imx_ssi_driver = {
	.probe = imx_spdif_dai_probe,
	.remove = __devexit_p(imx_spdif_dai_remove),

	.driver = {
		.name = "imx-spdif-dai",
		.owner = THIS_MODULE,
	},
};

static int __init imx_spdif_dai_init(void)
{
	return platform_driver_register(&imx_ssi_driver);
}

static void __exit imx_spdif_dai_exit(void)
{
	platform_driver_unregister(&imx_ssi_driver);
}
module_init(imx_spdif_dai_init);
module_exit(imx_spdif_dai_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("IMX SPDIF DAI");
MODULE_LICENSE("GPL");
