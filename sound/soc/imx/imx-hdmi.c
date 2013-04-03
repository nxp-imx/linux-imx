/*
 * ASoC HDMI Transmitter driver for IMX development boards
 *
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc.
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
#include <linux/clk.h>
#include <linux/io.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <asm/dma.h>
#include <mach/hardware.h>

#include <mach/mxc_hdmi.h>
#include <linux/mfd/mxc-hdmi-core.h>
#include "imx-hdmi.h"

/* imx digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link imx_hdmi_dai_link = {
	.name = "IMX HDMI TX",
	.stream_name = "IMX HDMI TX",
	.codec_dai_name = "mxc-hdmi-soc",
	.codec_name = "mxc_hdmi_soc.0",
	.cpu_dai_name = "imx-hdmi-soc-dai.0",
	.platform_name = "imx-hdmi-soc-audio.0",
};

static struct snd_soc_card snd_soc_card_imx_hdmi = {
	.name = "imx-hdmi-soc",
	.dai_link = &imx_hdmi_dai_link,
	.num_links = 1,
};

static struct platform_device *imx_hdmi_snd_device;

static int __init imx_hdmi_init(void)
{
	int ret = 0;

	if (!hdmi_get_registered()) {
		pr_err("Initialize HDMI-audio failed. Load HDMI-video first!\n");
		return -ENODEV;
	}

	imx_hdmi_snd_device = platform_device_alloc("soc-audio", 4);
	if (!imx_hdmi_snd_device) {
		pr_err("%s - failed platform_device_alloc\n", __func__);
		return -ENOMEM;
	}

	platform_set_drvdata(imx_hdmi_snd_device, &snd_soc_card_imx_hdmi);

	ret = platform_device_add(imx_hdmi_snd_device);
	if (ret) {
		pr_err("ASoC HDMI TX: Platform device allocation failed\n");
		platform_device_put(imx_hdmi_snd_device);
	}

	return ret;
}

static void __exit imx_hdmi_exit(void)
{
	platform_device_unregister(imx_hdmi_snd_device);
}

module_init(imx_hdmi_init);
module_exit(imx_hdmi_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("IMX HDMI TX ASoC driver");
MODULE_LICENSE("GPL");
