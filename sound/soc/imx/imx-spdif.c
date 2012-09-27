/*
 * ASoC S/PDIF driver for IMX development boards
 *
 * Copyright (C) 2008-2012 Freescale Semiconductor, Inc.
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

#include "imx-ssi.h"
#include "../codecs/mxc_spdif.h"

/* imx digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link imx_spdif_dai_link = {
	.name = "IMX SPDIF",
	.stream_name = "IMX SPDIF",
	.codec_dai_name = "mxc-spdif",
	.codec_name = "mxc_spdif.0",
	.cpu_dai_name = "imx-spdif-dai.0",
	.platform_name = "imx-pcm-audio.2", /* imx-pcm-dma-mx2 */
};

static struct snd_soc_card snd_soc_card_imx_spdif = {
	.name = "imx-spdif",
	.dai_link = &imx_spdif_dai_link,
	.num_links = 1,
};

static int __devinit imx_spdif_audio_probe(struct platform_device *pdev)
{
	struct imx_ssi *ssi;
	int ret;

	ssi = kzalloc(sizeof(*ssi), GFP_KERNEL);
	if (!ssi)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, ssi);

	ssi->soc_platform_pdev = platform_device_alloc("imx-pcm-audio", 2);
	if (!ssi->soc_platform_pdev) {
		pr_err("%s failed platform_device_alloc\n", __func__);
		return -ENOMEM;
	}

	/* It may seem weird to be using struct imx_ssi here because S/PDIF
	   doesn't use ssi.  We only use soc_platform_pdev in struct imx_ssi.
	   But we need it because imx-pcm-dma-mx2 initializes other fields. */
	platform_set_drvdata(ssi->soc_platform_pdev, ssi);

	ret = platform_device_add(ssi->soc_platform_pdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to add platform device\n");
		platform_device_put(ssi->soc_platform_pdev);
		return ret;
	}

	return 0;
}

static int __devexit imx_spdif_audio_remove(struct platform_device *pdev)
{
	struct imx_ssi *ssi = platform_get_drvdata(pdev);

	platform_device_unregister(ssi->soc_platform_pdev);
	snd_soc_unregister_dai(&pdev->dev);
	kfree(ssi);

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

	ret = platform_driver_register(&imx_spdif_audio_driver);
	if (ret) {
		pr_err("%s - failed platform_driver_register\n", __func__);
		return -ENOMEM;
	}

	imx_spdif_snd_device = platform_device_alloc("soc-audio", 3);
	if (!imx_spdif_snd_device) {
		pr_err("%s - failed platform_device_alloc\n", __func__);
		return -ENOMEM;
	}

	if (machine_is_mx6sl_evk()) {
		imx_spdif_dai_link.name = "HDMI-Audio";
		imx_spdif_dai_link.stream_name = "HDMI-Audio";
	}

	platform_set_drvdata(imx_spdif_snd_device, &snd_soc_card_imx_spdif);

	ret = platform_device_add(imx_spdif_snd_device);
	if (ret) {
		pr_err("ASoC S/PDIF: Platform device allocation failed\n");
		platform_device_put(imx_spdif_snd_device);
	}

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
