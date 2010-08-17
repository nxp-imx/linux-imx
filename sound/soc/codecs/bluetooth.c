/*
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file bluetooth.c
 * @brief Driver for bluetooth PCM interface
 *
 * @ingroup Sound
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#define BLUETOOTH_RATES SNDRV_PCM_RATE_8000

#define BLUETOOTH_FORMATS SNDRV_PCM_FMTBIT_S16_LE

struct snd_soc_dai bt_dai = {
	.name = "bluetooth",
	.playback = {
		     .stream_name = "Playback",
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = BLUETOOTH_RATES,
		     .formats = BLUETOOTH_FORMATS,
		     },
	.capture = {
		    .stream_name = "Capture",
		    .channels_min = 1,
		    .channels_max = 2,
		    .rates = BLUETOOTH_RATES,
		    .formats = BLUETOOTH_FORMATS,
		    },
};
EXPORT_SYMBOL_GPL(bt_dai);

static int bt_init(struct snd_soc_device *socdev)
{
	struct snd_soc_codec *codec = socdev->card->codec;
	int ret = 0;

	codec->name = "bluetooth";
	codec->owner = THIS_MODULE;
	codec->dai = &bt_dai;
	codec->num_dai = 1;

	snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		pr_err("failed to create bluetooth pcms\n");
		return ret;
	}

	ret = snd_soc_init_card(socdev);
	strcpy(codec->card->id, "bluetooth");

	if (ret < 0) {
		pr_err("bluetooth: failed to register card\n");
		snd_soc_free_pcms(socdev);
		snd_soc_dapm_free(socdev);
		return ret;
	}
	return 0;
}

static struct snd_soc_device *bt_socdev;

static int bt_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
	int ret = 0;

	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	socdev->card->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	bt_socdev = socdev;

	ret = bt_init(socdev);
	if (ret < 0) {
		pr_err("Bluetooth codec initialisation failed\n");
		kfree(codec);
	}

	return ret;
}

/* power down chip */
static int bt_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
	kfree(codec);

	return 0;
}

static int bt_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int bt_resume(struct platform_device *pdev)
{
	return 0;
}

struct snd_soc_codec_device soc_codec_dev_bt = {
	.probe = bt_probe,
	.remove = bt_remove,
	.suspend = bt_suspend,
	.resume = bt_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_bt);

static int __init bluetooth_modinit(void)
{
	return snd_soc_register_dai(&bt_dai);
}

module_init(bluetooth_modinit);

static void __exit bluetooth_exit(void)
{
	snd_soc_unregister_dai(&bt_dai);
}

module_exit(bluetooth_exit);

MODULE_DESCRIPTION("ASoC bluetooth codec driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
