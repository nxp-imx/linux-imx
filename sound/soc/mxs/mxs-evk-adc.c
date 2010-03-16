/*
 * ASoC driver for Freescale MXS EVK board
 *
 * Author: Vladislav Buzov <vbuzov@embeddedalley.com>
 *
 * Copyright 2008-2010 Freescale Semiconductor, Inc.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
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

#include "../codecs/mxs-adc-codec.h"
#include "mxs-adc.h"
#include "mxs-pcm.h"

/* mxs evk dac/adc audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link mxs_evk_codec_dai = {
	.name = "MXS ADC/DAC",
	.stream_name = "MXS ADC/DAC",
	.cpu_dai = &mxs_adc_dai,
	.codec_dai = &mxs_codec_dai,
};

/* mxs evk audio machine driver */
static struct snd_soc_card snd_soc_card_mxs_evk = {
	.name = "MXS EVK",
	.platform = &mxs_soc_platform,
	.dai_link = &mxs_evk_codec_dai,
	.num_links = 1,
};

/* mxs evk audio subsystem */
static struct snd_soc_device mxs_evk_snd_devdata = {
	.card = &snd_soc_card_mxs_evk,
	.codec_dev = &soc_codec_dev_mxs,
};

static struct platform_device *mxs_evk_snd_device;

static int __init mxs_evk_adc_init(void)
{
	int ret = 0;

	mxs_evk_snd_device = platform_device_alloc("soc-audio", 0);
	if (!mxs_evk_snd_device)
		return -ENOMEM;

	platform_set_drvdata(mxs_evk_snd_device,
			&mxs_evk_snd_devdata);
	mxs_evk_snd_devdata.dev = &mxs_evk_snd_device->dev;
	mxs_evk_snd_device->dev.platform_data =
		&mxs_evk_snd_devdata;

	ret = platform_device_add(mxs_evk_snd_device);
	if (ret)
		platform_device_put(mxs_evk_snd_device);

	return ret;
}

static void __exit mxs_evk_adc_exit(void)
{
	platform_device_unregister(mxs_evk_snd_device);
}

module_init(mxs_evk_adc_init);
module_exit(mxs_evk_adc_exit);

MODULE_AUTHOR("Vladislav Buzov");
MODULE_DESCRIPTION("MXS EVK board ADC/DAC driver");
MODULE_LICENSE("GPL");
