/*
 * ASoC driver for MXS Evk development board
 *
 * Copyright (C) 2008-2010 Freescale Semiconductor, Inc.
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
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <asm/dma.h>
#include <mach/hardware.h>

#include "mxs-spdif-dai.h"
#include "../codecs/mxs_spdif.h"
#include "mxs-pcm.h"

/* mxs devb digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link mxs_devb_dai = {
	.name = "MXS SPDIF",
	.stream_name = "MXS SPDIF",
	.cpu_dai = &mxs_spdif_dai,
	.codec_dai = &mxs_spdif_codec_dai,
};

/* mxs devb audio machine driver */
static struct snd_soc_card snd_soc_machine_mxs_devb = {
	.name = "mxs-evk",
	.platform = &mxs_soc_platform,
	.dai_link = &mxs_devb_dai,
	.num_links = 1,
};

/* mxs devb audio subsystem */
static struct snd_soc_device mxs_devb_snd_devdata = {
	.card = &snd_soc_machine_mxs_devb,
	.codec_dev = &soc_spdif_codec_dev_mxs,
};

static struct platform_device *mxs_devb_snd_device;

static int __init mxs_devb_init(void)
{
	int ret = 0;

	mxs_devb_snd_device = platform_device_alloc("soc-audio", 2);
	if (!mxs_devb_snd_device)
		return -ENOMEM;

	platform_set_drvdata(mxs_devb_snd_device,
			     &mxs_devb_snd_devdata);
	mxs_devb_snd_devdata.dev = &mxs_devb_snd_device->dev;
	mxs_devb_snd_device->dev.platform_data =
		&mxs_devb_snd_devdata;

	ret = platform_device_add(mxs_devb_snd_device);
	if (ret)
		platform_device_put(mxs_devb_snd_device);

	return ret;
}

static void __exit mxs_devb_exit(void)
{
	platform_device_unregister(mxs_devb_snd_device);
}

module_init(mxs_devb_init);
module_exit(mxs_devb_exit);

MODULE_AUTHOR("Vladimir Barinov");
MODULE_DESCRIPTION("MXS EVK development board ASoC driver");
MODULE_LICENSE("GPL");
