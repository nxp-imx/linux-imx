// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
//
// This file is provided under a dual BSD/GPLv2 license.  When using or
// redistributing this file, you may do so under either license.
//
// Copyright(c) 2018 Intel Corporation
//
// Author: Liam Girdwood <liam.r.girdwood@linux.intel.com>
//

#include <linux/module.h>
#include <sound/sof.h>
#include "sof-audio.h"
#include "sof-priv.h"

static struct snd_soc_card sof_nocodec_card = {
	.name = "nocodec", /* the sof- prefix is added by the core */
	.topology_shortname = "sof-nocodec",
	.owner = THIS_MODULE
};

static int sof_nocodec_bes_setup(struct device *dev,
				 struct snd_soc_dai_driver *drv,
				 struct snd_soc_dai_link *links,
				 int link_num, struct snd_soc_card *card)
{
	struct snd_soc_dai_link_component *dlc;
	int i;

	if (!drv || !links || !card)
		return -EINVAL;

	/* set up BE dai_links */
	for (i = 0; i < link_num; i++) {
		dlc = devm_kcalloc(dev, 2, sizeof(*dlc), GFP_KERNEL);
		if (!dlc)
			return -ENOMEM;

		links[i].name = devm_kasprintf(dev, GFP_KERNEL,
					       "NoCodec-%d", i);
		if (!links[i].name)
			return -ENOMEM;

		links[i].stream_name = links[i].name;

		links[i].cpus = &dlc[0];
		links[i].codecs = &snd_soc_dummy_dlc;
		links[i].platforms = &dlc[1];

		links[i].num_cpus = 1;
		links[i].num_codecs = 1;
		links[i].num_platforms = 1;

		links[i].id = i;
		links[i].no_pcm = 1;
		links[i].cpus->dai_name = drv[i].name;

		if (!dev->of_node) {
			links[i].platforms->name = dev_name(dev->parent);
			links[i].platforms->of_node = NULL;
		} else {
			links[i].platforms->of_node = dev->of_node;
			links[i].platforms->name = NULL;
		}

		if (drv[i].playback.channels_min)
			links[i].dpcm_playback = 1;
		if (drv[i].capture.channels_min)
			links[i].dpcm_capture = 1;

		links[i].be_hw_params_fixup = sof_pcm_dai_link_fixup;
	}

	card->dai_link = links;
	card->num_links = link_num;

	return 0;
}

static int sof_nocodec_setup(struct device *dev,
			     u32 num_dai_drivers,
			     struct snd_soc_dai_driver *dai_drivers)
{
	struct snd_soc_dai_link *links;

	if (!dai_drivers) {
		dev_err(dev, "ERROR: dai_drivers is NULL\n");
		return -EINVAL;
	}

	/* create dummy BE dai_links */
	links = devm_kcalloc(dev, num_dai_drivers, sizeof(struct snd_soc_dai_link), GFP_KERNEL);
	if (!links)
		return -ENOMEM;

	return sof_nocodec_bes_setup(dev, dai_drivers, links, num_dai_drivers, &sof_nocodec_card);
}

static int sof_nocodec_parse_dt_dai_info(struct device *dev,
					 u32 *num_dai_drivers,
					 struct snd_soc_dai_driver **dai_drivers)
{
	struct device_node *np = dev->of_node;
	const char *dai_name;
	int playback_channels, capture_channels;
	int ret, i;

	ret = of_property_read_u32(np, "sof,num-dai-drivers", num_dai_drivers);
	if (ret)
		return ret;

	*dai_drivers = devm_kcalloc(dev, *num_dai_drivers,
				   sizeof(struct snd_soc_dai_driver), GFP_KERNEL);
	if (!*dai_drivers)
		return -ENOMEM;

	for (i = 0; i < *num_dai_drivers; i++) {

		ret = of_property_read_string_index(np, "sof,dai-driver-names", i, &dai_name);
		if (ret)
			return ret;

		ret = of_property_read_u32_index(np, "sof,dai-playback-channels", i, &playback_channels);
		if (ret)
			playback_channels = 0;

		ret = of_property_read_u32_index(np, "sof,dai-capture-channels", i, &capture_channels);
		if (ret)
			capture_channels = 0;

		(*dai_drivers)[i].name = devm_kstrdup(dev, dai_name, GFP_KERNEL);
		if (!(*dai_drivers)[i].name)
			return -ENOMEM;

		(*dai_drivers)[i].id = i;

		if (playback_channels > 0) {
			(*dai_drivers)[i].playback.channels_min = 1;
			(*dai_drivers)[i].playback.channels_max = 32;
		}

		if (capture_channels > 0) {
			(*dai_drivers)[i].capture.channels_min = 1;
			(*dai_drivers)[i].capture.channels_max = 32;
		}

	}

	return 0;
}

static int sof_nocodec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct snd_soc_card *card = &sof_nocodec_card;
	struct snd_soc_acpi_mach *mach;
	int ret;

	card->dev = dev;
	card->topology_shortname_created = true;

	if (dev->of_node) {
		u32 num_dai_drivers = 0;
		struct snd_soc_dai_driver *dai_drivers = NULL;

		ret = sof_nocodec_parse_dt_dai_info(dev, &num_dai_drivers, &dai_drivers);

		if (ret) {
			dev_err(dev, "Failed to parse DT info: %d\n", ret);
			return ret;
		}

		ret = sof_nocodec_setup(dev, num_dai_drivers, dai_drivers);
	} else {
		mach = dev->platform_data;

		ret = sof_nocodec_setup(dev,
					mach->mach_params.num_dai_drivers,
					mach->mach_params.dai_drivers);
	}

	if (ret < 0)
		return ret;

	return devm_snd_soc_register_card(dev, card);
}

static const struct of_device_id sof_nocodec_of_match[] = {
	{ .compatible = "sof-audio-nocodec", },
	{ },
};
MODULE_DEVICE_TABLE(of, sof_nocodec_of_match);

static struct platform_driver sof_nocodec_audio = {
	.probe = sof_nocodec_probe,
	.driver = {
		.name = "sof-nocodec",
		.pm = &snd_soc_pm_ops,
		.of_match_table = sof_nocodec_of_match,
	},
};
module_platform_driver(sof_nocodec_audio)

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("ASoC sof nocodec");
MODULE_AUTHOR("Liam Girdwood");
MODULE_ALIAS("platform:sof-nocodec");
