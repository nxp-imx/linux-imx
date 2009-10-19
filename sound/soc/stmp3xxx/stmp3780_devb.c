/*
 * ASoC driver for Freescale STMP3780 development board
 *
 * Author: Vladislav Buzov <vbuzov@embeddedalley.com>
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <mach/regs-apbx.h>

#include "../codecs/stmp378x_codec.h"
#include "stmp3xxx_dai.h"
#include "stmp3xxx_pcm.h"

/* stmp3780 devb digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link stmp3780_devb_dai = {
	.name = "STMP378X ADC/DAC",
	.stream_name = "STMP378X ADC/DAC",
	.cpu_dai = &stmp3xxx_adc_dai,
	.codec_dai = &stmp378x_codec_dai,
};

/* stmp3780 devb audio machine driver */
static struct snd_soc_card snd_soc_card_stmp3780_devb = {
	.name = "STMP3780 Devb",
	.platform = &stmp3xxx_soc_platform,
	.dai_link = &stmp3780_devb_dai,
	.num_links = 1,
};

/* stmp3780 devb audio subsystem */
static struct snd_soc_device stmp3780_devb_snd_devdata = {
	.card = &snd_soc_card_stmp3780_devb,
	.codec_dev = &soc_codec_dev_stmp378x,
};

static struct platform_device *stmp3780_devb_snd_device;

static int __init stmp3780_devb_init(void)
{
	int ret = 0;

	stmp3780_devb_snd_device = platform_device_alloc("soc-audio", 0);
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

	return ret;
}

static void __exit stmp3780_devb_exit(void)
{
	platform_device_unregister(stmp3780_devb_snd_device);
}

module_init(stmp3780_devb_init);
module_exit(stmp3780_devb_exit);

MODULE_AUTHOR("Vladislav Buzov");
MODULE_DESCRIPTION("STMP3780 development board ASoC driver");
MODULE_LICENSE("GPL");
