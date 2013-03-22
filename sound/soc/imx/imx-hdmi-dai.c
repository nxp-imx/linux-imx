/*
 * ALSA SoC HDMI Audio Layer for MXS
 *
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc.
 *
 * Based on stmp3xxx_spdif_dai.c
 * Vladimir Barinov <vbarinov@embeddedalley.com>
 * Copyright 2008 SigmaTel, Inc
 * Copyright 2008 Embedded Alley Solutions, Inc
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program  is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <mach/dma.h>
#include <mach/hardware.h>

#include <linux/mfd/mxc-hdmi-core.h>
#include "imx-hdmi.h"

static struct snd_soc_dai_driver imx_hdmi_dai = {
	.playback = {
		.channels_min = 2,
		.channels_max = 8,
		.rates = MXC_HDMI_RATES_PLAYBACK,
		.formats = MXC_HDMI_FORMATS_PLAYBACK,
	},
};

static int imx_hdmi_dai_probe(struct platform_device *pdev)
{
	struct imx_hdmi *hdmi_data;
	int ret = 0;

	if (!hdmi_get_registered()) {
		dev_err(&pdev->dev, "Failed: Load HDMI-video first.\n");
		return -ENODEV;
	}

	hdmi_data = kzalloc(sizeof(*hdmi_data), GFP_KERNEL);
	if (!hdmi_data)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, hdmi_data);
	hdmi_data->pdev = pdev;

	hdmi_data->irq = platform_get_irq(pdev, 0);
	if (hdmi_data->irq <= 0) {
		dev_err(&pdev->dev, "MXC hdmi: invalid irq number (%d)\n",
			hdmi_data->irq);
		goto e_reg_dai;
	}

	ret = snd_soc_register_dai(&pdev->dev, &imx_hdmi_dai);
	if (ret) {
		dev_err(&pdev->dev, "register DAI failed\n");
		goto e_reg_dai;
	}

	hdmi_data->soc_platform_pdev =
		platform_device_alloc("imx-hdmi-soc-audio", 0);

	if (!hdmi_data->soc_platform_pdev) {
		dev_err(&pdev->dev, "failed platform_device_alloc\n");
		goto e_alloc_dai;
	}

	platform_set_drvdata(hdmi_data->soc_platform_pdev, hdmi_data);

	ret = platform_device_add(hdmi_data->soc_platform_pdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to add platform device\n");
		platform_device_put(hdmi_data->soc_platform_pdev);
		goto e_pdev_add;
	}

	return 0;
e_pdev_add:
	platform_device_put(hdmi_data->soc_platform_pdev);
e_alloc_dai:
	snd_soc_unregister_dai(&pdev->dev);
e_reg_dai:
	kfree(hdmi_data);
	return ret;
}

static int __devexit imx_hdmi_dai_remove(struct platform_device *pdev)
{
	struct imx_hdmi *hdmi_data = platform_get_drvdata(pdev);

	snd_soc_unregister_dai(&pdev->dev);
	kfree(hdmi_data);

	return 0;
}

static struct platform_driver imx_hdmi_driver = {
	.probe = imx_hdmi_dai_probe,
	.remove = __devexit_p(imx_hdmi_dai_remove),
	.driver = {
		.name = "imx-hdmi-soc-dai",
		.owner = THIS_MODULE,
	},
};

static int __init imx_hdmi_dai_init(void)
{
	return platform_driver_register(&imx_hdmi_driver);
}

static void __exit imx_hdmi_dai_exit(void)
{
	platform_driver_unregister(&imx_hdmi_driver);
}

module_init(imx_hdmi_dai_init);
module_exit(imx_hdmi_dai_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("IMX HDMI TX DAI");
MODULE_LICENSE("GPL");
