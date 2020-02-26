// SPDX-License-Identifier: GPL-2.0
// Copyright 2019 NXP

#include <linux/module.h>
#include <linux/of_platform.h>
#include <sound/soc.h>
#include "fsl_xcvr.h"

struct imx_xcvr_data {
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
};

static int imx_xcvr_audio_probe(struct platform_device *pdev)
{
	struct device_node *xcvr_np, *np = pdev->dev.of_node;
	struct snd_soc_dai_link_component *dlc;
	struct imx_xcvr_data *data;
	int ret = 0;

	xcvr_np = of_parse_phandle(np, "cpu-dai", 0);
	if (!xcvr_np) {
		dev_err(&pdev->dev, "failed to find cpu-dai\n");
		ret = -EINVAL;
		goto end;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto end;
	}

	dlc = devm_kzalloc(&pdev->dev, 3 * sizeof(*dlc), GFP_KERNEL);
	if (!dlc) {
		ret = -ENOMEM;
		goto end;
	}

	data->dai.cpus = &dlc[0];
	data->dai.num_cpus = 1;
	data->dai.platforms = &dlc[1];
	data->dai.num_platforms = 1;
	data->dai.codecs = &dlc[2];
	data->dai.num_codecs = 1;

	data->dai.name = "XCVR PCM";
	data->dai.stream_name = "XCVR PCM";
	data->dai.codecs->dai_name = "snd-soc-dummy-dai";
	data->dai.codecs->name = "snd-soc-dummy";
	data->dai.cpus->of_node = xcvr_np;
	data->dai.platforms->of_node = xcvr_np;

	data->card.dev = &pdev->dev;
	data->card.dai_link = &data->dai;
	data->card.num_links = 1;
	data->card.owner = THIS_MODULE;

	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret)
		goto end;

	ret = devm_snd_soc_register_card(&pdev->dev, &data->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed: %d\n", ret);
		goto end;
	}
end:
	of_node_put(xcvr_np);
	return ret;
}

static const struct of_device_id imx_xcvr_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-xcvr", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_xcvr_dt_ids);

static struct platform_driver imx_xcvr_driver = {
	.driver = {
		.name = "imx-xcvr",
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_xcvr_dt_ids,
	},
	.probe = imx_xcvr_audio_probe,
};

module_platform_driver(imx_xcvr_driver);

MODULE_AUTHOR("Viorel Suman <viorel.suman@nxp.com>");
MODULE_DESCRIPTION("NXP Audio Transceiver (XCVR) machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-xcvr");
