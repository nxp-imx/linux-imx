/*
 * Copyright (C) 2008-2013 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/fsl_devices.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <mach/audmux.h>
#include <mach/ssi.h>

#include "imx-ssi.h"

static struct imx_si4763_priv {
	int sysclk;
	int hw;
	int active;
	struct platform_device *pdev;
} card_priv;

static int imx_audmux_config(int slave, int master)
{
	unsigned int ptcr, pdcr;
	slave = slave - 1;
	master = master - 1;

	ptcr = MXC_AUDMUX_V2_PTCR_SYN |
		MXC_AUDMUX_V2_PTCR_TFSDIR |
		MXC_AUDMUX_V2_PTCR_TFSEL(slave) |
		MXC_AUDMUX_V2_PTCR_TCLKDIR |
		MXC_AUDMUX_V2_PTCR_TCSEL(slave);
	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(slave);
	mxc_audmux_v2_configure_port(master, ptcr, pdcr);

	ptcr = MXC_AUDMUX_V2_PTCR_SYN;
	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(master);
	mxc_audmux_v2_configure_port(slave, ptcr, pdcr);

	return 0;
}


static int imx_3stack_si4763_startup(struct snd_pcm_substream *substream)
{
	struct imx_si4763_priv *priv = &card_priv;

	priv->active++;
	return 0;
}

static int imx_3stack_si4763_hw_params(struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int channels = params_channels(params);
	struct imx_ssi *ssi_mode = snd_soc_dai_get_drvdata(cpu_dai);
	int ret = 0;
	u32 dai_format;
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	    SND_SOC_DAIFMT_CBS_CFS;

	ssi_mode->flags |= IMX_SSI_SYN;

	/* set i.MX active slot mask */
	snd_soc_dai_set_tdm_slot(cpu_dai,
			channels == 1 ? 0xfffffffe : 0xfffffffc,
			channels == 1 ? 0xfffffffe : 0xfffffffc,
			2, 32);

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret < 0)
		return ret;

	/* set the SSI system clock as input (unused) */
	snd_soc_dai_set_sysclk(cpu_dai, IMX_SSP_SYS_CLK, 0, SND_SOC_CLOCK_IN);

	snd_soc_dai_set_clkdiv(cpu_dai, IMX_SSI_TX_DIV_PM, 4);
	snd_soc_dai_set_clkdiv(cpu_dai, IMX_SSI_TX_DIV_2, 1);
	snd_soc_dai_set_clkdiv(cpu_dai, IMX_SSI_TX_DIV_PSR, 0);
	return 0;
}

static void imx_3stack_si4763_shutdown(struct snd_pcm_substream *substream)
{
	struct imx_si4763_priv *priv = &card_priv;
	priv->active--;
}

/*
 * imx_3stack bt DAI opserations.
 */
static struct snd_soc_ops imx_3stack_si4763_ops = {
	.startup = imx_3stack_si4763_startup,
	.hw_params = imx_3stack_si4763_hw_params,
	.shutdown = imx_3stack_si4763_shutdown,
};

static struct snd_soc_dai_link imx_3stack_dai = {
	.name = "imx-si4763",
	.stream_name = "imx-si4763",
	.codec_dai_name = "si4763",
	.codec_name     = "si4763.0",
	.cpu_dai_name   = "imx-ssi.1",
	.platform_name  = "imx-pcm-audio.1",
	.ops = &imx_3stack_si4763_ops,
};

static struct snd_soc_card snd_soc_card_imx_3stack = {
	.name = "imx-audio-si4763",
	.dai_link = &imx_3stack_dai,
	.num_links = 1,
};

static int __devinit imx_3stack_si4763_probe(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

	card_priv.pdev = pdev;
	card_priv.sysclk = plat->sysclk;
	imx_audmux_config(plat->src_port, plat->ext_port);

	return 0;

}

static int __devexit imx_3stack_si4763_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver imx_3stack_si4763_driver = {
	.probe = imx_3stack_si4763_probe,
	.remove = __devexit_p(imx_3stack_si4763_remove),
	.driver = {
		   .name = "imx-tuner-si4763",
		   .owner = THIS_MODULE,
		   },
};

static struct platform_device *imx_3stack_snd_device;

static int __init imx_3stack_asoc_init(void)
{
	int ret;
	ret = platform_driver_register(&imx_3stack_si4763_driver);
	if (ret < 0)
		goto exit;

	imx_3stack_snd_device = platform_device_alloc("soc-audio", 6);
	if (!imx_3stack_snd_device)
		goto err_device_alloc;

	platform_set_drvdata(imx_3stack_snd_device, &snd_soc_card_imx_3stack);
	ret = platform_device_add(imx_3stack_snd_device);
	if (0 == ret)
		goto exit;


	platform_device_put(imx_3stack_snd_device);
err_device_alloc:
	platform_driver_unregister(&imx_3stack_si4763_driver);
exit:
	return ret;
}

static void __exit imx_3stack_asoc_exit(void)
{
	platform_driver_unregister(&imx_3stack_si4763_driver);
	platform_device_unregister(imx_3stack_snd_device);
}

module_init(imx_3stack_asoc_init);
module_exit(imx_3stack_asoc_exit);

/* Module information */
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("ALSA SoC si4763 imx");
MODULE_LICENSE("GPL");
