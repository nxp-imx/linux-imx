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
#include "imx-pcm.h"
#include "imx-spdif.h"

struct snd_soc_dai_ops imx_spdif_dai_ops = {
};

struct snd_soc_dai imx_spdif_dai = {
	.name = "imx-spdif-dai",
	.id = IMX_DAI_SPDIF,
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
EXPORT_SYMBOL_GPL(imx_spdif_dai);

static int __init imx_spdif_dai_init(void)
{
	return snd_soc_register_dai(&imx_spdif_dai);
}

static void __exit imx_spdif_dai_exit(void)
{
	snd_soc_unregister_dai(&imx_spdif_dai);
}
module_init(imx_spdif_dai_init);
module_exit(imx_spdif_dai_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("IMX SPDIF DAI");
MODULE_LICENSE("GPL");
