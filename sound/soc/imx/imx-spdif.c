/*
 * ASoC S/PDIF driver for IMX development boards
 *
 * Copyright (C) 2008-2011 Freescale Semiconductor, Inc.
 *
 * based on stmp3780_devb_spdif.c
 *
 * Vladimir Barinov <vbarinov@embeddedalley.com>
 *
 * Copyright 2008 SigmaTel, Inc
 * Copyright 2008 Embedded Alley Solutions, Inc
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program  is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <asm/dma.h>
#include <mach/hardware.h>

#include "imx-spdif-dai.h"
#include "../codecs/mxc_spdif.h"
#include "imx-pcm.h"

/* imx digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link imx_3stack_dai = {
	.name = "IMX SPDIF",
	.stream_name = "IMX SPDIF",
	.cpu_dai = &imx_spdif_dai,
	.codec_dai = &mxc_spdif_codec_dai,
};

static struct snd_soc_card snd_soc_card_imx_spdif = {
	.name = "imx-3stack-spdif",
	.platform = &imx_soc_platform,
	.dai_link = &imx_3stack_dai,
	.num_links = 1,
};

static struct snd_soc_device imx_spdif_snd_devdata = {
	.card = &snd_soc_card_imx_spdif,
	.codec_dev = &soc_codec_dev_spdif,
};

static int __devinit imx_spdif_audio_probe(struct platform_device *pdev)
{
	/* get mxc_audio_platform_data for spdif */
	imx_3stack_dai.cpu_dai->dev = &pdev->dev;
	return 0;
}

static int __devexit imx_spdif_audio_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver imx_spdif_audio_driver = {
	.probe = imx_spdif_audio_probe,
	.remove = __devexit_p(imx_spdif_audio_remove),
	.driver = {
		   .name = "imx-spdif-audio-device",
		   .owner = THIS_MODULE,
		   },
};

static struct platform_device *imx_spdif_snd_device;

static int __init imx_spdif_init(void)
{
	int ret = 0;

	platform_driver_register(&imx_spdif_audio_driver);

	imx_spdif_snd_device = platform_device_alloc("soc-audio", -1);
	if (!imx_spdif_snd_device)
		return -ENOMEM;

	platform_set_drvdata(imx_spdif_snd_device, &imx_spdif_snd_devdata);
	imx_spdif_snd_devdata.dev = &imx_spdif_snd_device->dev;
	imx_spdif_snd_device->dev.platform_data = &imx_spdif_snd_devdata;

	ret = platform_device_add(imx_spdif_snd_device);
	if (ret)
		platform_device_put(imx_spdif_snd_device);

	return ret;
}

static void __exit imx_spdif_exit(void)
{
	platform_driver_unregister(&imx_spdif_audio_driver);
	platform_device_unregister(imx_spdif_snd_device);
}

module_init(imx_spdif_init);
module_exit(imx_spdif_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("IMX EVK development board ASoC driver");
MODULE_LICENSE("GPL");
