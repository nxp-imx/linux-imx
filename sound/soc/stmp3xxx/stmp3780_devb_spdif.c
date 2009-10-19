/*
 * ASoC driver for STMP3780 development board
 *
 * Vladimir Barinov <vbarinov@embeddedalley.com>
 *
 * Copyright 2008 SigmaTel, Inc
 * Copyright 2008 Embedded Alley Solutions, Inc
 * Copyright 2008-2009 Freescale Semiconductor, Inc.
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
#include <mach/regs-apbx.h>

#include <mach/stmp3xxx.h>

#include "../codecs/stmp3xxx_spdif.h"
#include "stmp3xxx_spdif_dai.h"
#include "stmp3xxx_pcm.h"

extern int spdif_pinmux(int);

/* stmp3780 devb digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link stmp3780_devb_dai = {
	.name = "STMP3XXX SPDIF",
	.stream_name = "STMP3XXX SPDIF",
	.cpu_dai = &stmp3xxx_spdif_dai,
	.codec_dai = &stmp3xxx_spdif_codec_dai,
};

/* stmp3780 devb audio machine driver */
static struct snd_soc_card snd_soc_machine_stmp3780_devb = {
	.name = "STMP3780 Devb",
	.platform = &stmp3xxx_soc_platform,
	.dai_link = &stmp3780_devb_dai,
	.num_links = 1,
};

/* stmp3780 devb audio subsystem */
static struct snd_soc_device stmp3780_devb_snd_devdata = {
	.card = &snd_soc_machine_stmp3780_devb,
	.codec_dev = &soc_spdif_codec_dev_stmp3xxx,
};

static struct platform_device *stmp3780_devb_snd_device;

static int __init stmp3780_devb_init(void)
{
	int ret = 0;

	stmp3780_devb_snd_device = platform_device_alloc("soc-audio", 1);
	if (!stmp3780_devb_snd_device)
		return -ENOMEM;

	platform_set_drvdata(stmp3780_devb_snd_device,
			     &stmp3780_devb_snd_devdata);
	stmp3780_devb_snd_devdata.dev = &stmp3780_devb_snd_device->dev;
	stmp3780_devb_snd_device->dev.platform_data =
		&stmp3780_devb_snd_devdata;

	ret = platform_device_add(stmp3780_devb_snd_device);
	if (ret)
		platform_device_put(stmp3780_devb_snd_device);

	spdif_pinmux(1);

	return ret;
}

static void __exit stmp3780_devb_exit(void)
{
	spdif_pinmux(0);
	platform_device_unregister(stmp3780_devb_snd_device);
}

module_init(stmp3780_devb_init);
module_exit(stmp3780_devb_exit);

MODULE_AUTHOR("Vladimir Barinov");
MODULE_DESCRIPTION("STMP3780 development board ASoC driver");
MODULE_LICENSE("GPL");
