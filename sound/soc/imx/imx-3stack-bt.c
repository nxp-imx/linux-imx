/*
 * imx-3stack-bt.c  --  SoC bluetooth audio for imx_3stack
 *
 * Copyright (C) 2008-2010 Freescale  Semiconductor, Inc. All Rights Reserved.
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
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include "imx-pcm.h"
#include "imx-ssi.h"
#include "imx-3stack-bt.h"

#define BT_SSI_MASTER	1

struct imx_3stack_priv {
	struct platform_device *pdev;
	int active;
};

static struct imx_3stack_priv card_priv;

static void imx_3stack_init_dam(int ssi_port, int dai_port)
{
	/* bt uses SSI1 or SSI2 via AUDMUX port dai_port for audio */
	unsigned int ssi_ptcr = 0;
	unsigned int dai_ptcr = 0;
	unsigned int ssi_pdcr = 0;
	unsigned int dai_pdcr = 0;

	/* reset port ssi_port & dai_port */
	__raw_writel(0, DAM_PTCR(ssi_port));
	__raw_writel(0, DAM_PTCR(dai_port));
	__raw_writel(0, DAM_PDCR(ssi_port));
	__raw_writel(0, DAM_PDCR(dai_port));

	/* set to synchronous */
	ssi_ptcr |= AUDMUX_PTCR_SYN;
	dai_ptcr |= AUDMUX_PTCR_SYN;

#if BT_SSI_MASTER
	/* set Rx sources ssi_port <--> dai_port */
	ssi_pdcr |= AUDMUX_PDCR_RXDSEL(dai_port);
	dai_pdcr |= AUDMUX_PDCR_RXDSEL(ssi_port);

	/* set Tx frame direction and source  dai_port--> ssi_port output */
	ssi_ptcr |= AUDMUX_PTCR_TFSDIR;
	ssi_ptcr |= AUDMUX_PTCR_TFSSEL(AUDMUX_FROM_TXFS, dai_port);

	/* set Tx Clock direction and source dai_port--> ssi_port output */
	ssi_ptcr |= AUDMUX_PTCR_TCLKDIR;
	ssi_ptcr |= AUDMUX_PTCR_TCSEL(AUDMUX_FROM_TXFS, dai_port);
#else
	/* set Rx sources ssi_port <--> dai_port */
	ssi_pdcr |= AUDMUX_PDCR_RXDSEL(dai_port);
	dai_pdcr |= AUDMUX_PDCR_RXDSEL(ssi_port);

	/* set Tx frame direction and source  ssi_port --> dai_port output */
	dai_ptcr |= AUDMUX_PTCR_TFSDIR;
	dai_ptcr |= AUDMUX_PTCR_TFSSEL(AUDMUX_FROM_TXFS, ssi_port);

	/* set Tx Clock direction and source ssi_port--> dai_port output */
	dai_ptcr |= AUDMUX_PTCR_TCLKDIR;
	dai_ptcr |= AUDMUX_PTCR_TCSEL(AUDMUX_FROM_TXFS, ssi_port);
#endif

	__raw_writel(ssi_ptcr, DAM_PTCR(ssi_port));
	__raw_writel(dai_ptcr, DAM_PTCR(dai_port));
	__raw_writel(ssi_pdcr, DAM_PDCR(ssi_port));
	__raw_writel(dai_pdcr, DAM_PDCR(dai_port));
}

static int imx_3stack_bt_startup(struct snd_pcm_substream *substream)
{
	struct imx_3stack_priv *priv = &card_priv;

	if (!priv->active)
		gpio_activate_bt_audio_port();
	priv->active++;
	return 0;
}

static int imx_3stack_bt_hw_params(struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *pcm_link = rtd->dai;
	struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;
	unsigned int channels = params_channels(params);
	struct imx_ssi *ssi_mode = (struct imx_ssi *)cpu_dai->private_data;
	int ret = 0;
	u32 dai_format;

#if BT_SSI_MASTER
	dai_format = SND_SOC_DAIFMT_LEFT_J | SND_SOC_DAIFMT_IB_IF |
	    SND_SOC_DAIFMT_CBM_CFM;
#else
	dai_format = SND_SOC_DAIFMT_LEFT_J | SND_SOC_DAIFMT_IB_IF |
	    SND_SOC_DAIFMT_CBS_CFS;
#endif

	ssi_mode->sync_mode = 1;
	if (channels == 1)
		ssi_mode->network_mode = 0;
	else
		ssi_mode->network_mode = 1;

	/* set i.MX active slot mask */
	snd_soc_dai_set_tdm_slot(cpu_dai,
				 channels == 1 ? 0xfffffffe : 0xfffffffc, 2);

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret < 0)
		return ret;

	/* set the SSI system clock as input (unused) */
	snd_soc_dai_set_sysclk(cpu_dai, IMX_SSP_SYS_CLK, 0, SND_SOC_CLOCK_IN);

	return 0;
}

static void imx_3stack_bt_shutdown(struct snd_pcm_substream *substream)
{
	struct imx_3stack_priv *priv = &card_priv;

	priv->active--;
	if (!priv->active)
		gpio_inactivate_bt_audio_port();
}

/*
 * imx_3stack bt DAI opserations.
 */
static struct snd_soc_ops imx_3stack_bt_ops = {
	.startup = imx_3stack_bt_startup,
	.hw_params = imx_3stack_bt_hw_params,
	.shutdown = imx_3stack_bt_shutdown,
};

static struct snd_soc_dai_link imx_3stack_dai = {
	.name = "bluetooth",
	.stream_name = "bluetooth",
	.codec_dai = &bt_dai,
	.ops = &imx_3stack_bt_ops,
};

static struct snd_soc_card snd_soc_card_imx_3stack = {
	.name = "imx-3stack",
	.platform = &imx_soc_platform,
	.dai_link = &imx_3stack_dai,
	.num_links = 1,
};

static struct snd_soc_device imx_3stack_snd_devdata = {
	.card = &snd_soc_card_imx_3stack,
	.codec_dev = &soc_codec_dev_bt,
};

/*
 * This function will register the snd_soc_pcm_link drivers.
 * It also registers devices for platform DMA, I2S, SSP and registers an
 * I2C driver to probe the codec.
 */
static int __init imx_3stack_bt_probe(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *dev_data = pdev->dev.platform_data;
	struct imx_3stack_priv *priv = &card_priv;
	struct snd_soc_dai *bt_cpu_dai;

	if (dev_data->src_port == 1)
		bt_cpu_dai = imx_ssi_dai[0];
	else
		bt_cpu_dai = imx_ssi_dai[2];

	bt_cpu_dai->dev = &pdev->dev;
	imx_3stack_dai.cpu_dai = bt_cpu_dai;

	/* Configure audio port */
	imx_3stack_init_dam(dev_data->src_port, dev_data->ext_port);

	priv->pdev = pdev;
	priv->active = 0;
	return 0;

}

static int __devexit imx_3stack_bt_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver imx_3stack_bt_driver = {
	.probe = imx_3stack_bt_probe,
	.remove = __devexit_p(imx_3stack_bt_remove),
	.driver = {
		   .name = "imx-3stack-bt",
		   .owner = THIS_MODULE,
		   },
};

static struct platform_device *imx_3stack_snd_device;

static int __init imx_3stack_asoc_init(void)
{
	int ret;

	ret = platform_driver_register(&imx_3stack_bt_driver);
	if (ret < 0)
		goto exit;
	imx_3stack_snd_device = platform_device_alloc("soc-audio", 4);
	if (!imx_3stack_snd_device)
		goto err_device_alloc;
	platform_set_drvdata(imx_3stack_snd_device, &imx_3stack_snd_devdata);
	imx_3stack_snd_devdata.dev = &imx_3stack_snd_device->dev;
	ret = platform_device_add(imx_3stack_snd_device);
	if (0 == ret)
		goto exit;

	platform_device_put(imx_3stack_snd_device);
err_device_alloc:
	platform_driver_unregister(&imx_3stack_bt_driver);
exit:
	return ret;
}

static void __exit imx_3stack_asoc_exit(void)
{
	platform_driver_unregister(&imx_3stack_bt_driver);
	platform_device_unregister(imx_3stack_snd_device);
}

module_init(imx_3stack_asoc_init);
module_exit(imx_3stack_asoc_exit);

/* Module information */
MODULE_DESCRIPTION("ALSA SoC bluetooth imx_3stack");
MODULE_LICENSE("GPL");
