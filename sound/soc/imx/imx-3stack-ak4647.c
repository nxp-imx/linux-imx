/*
 * imx-3stack-ak4647.c  --  SoC audio for imx_3stack
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
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <mach/clock.h>
#include <linux/regulator/consumer.h>

#include "imx-pcm.h"
#include "imx-ssi.h"

#define AK4647_SSI_MASTER	1

extern struct snd_soc_dai ak4647_hifi_dai;
extern struct snd_soc_codec_device soc_codec_dev_ak4647;

static void headphone_detect_handler(struct work_struct *work);
static DECLARE_WORK(hp_event, headphone_detect_handler);
static int ak4647_jack_func;
static int ak4647_spk_func;

struct imx_3stack_priv {
	struct platform_device *pdev;
};

static struct imx_3stack_priv card_priv;

static void imx_3stack_init_dam(int ssi_port, int dai_port)
{
	/* AK4647 uses SSI1 or SSI2 via AUDMUX port dai_port for audio */
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

#if AK4647_SSI_MASTER
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

static int imx_3stack_hifi_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *pcm_link = rtd->dai;
	struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;
	struct snd_soc_dai *codec_dai = pcm_link->codec_dai;
	unsigned int channels = params_channels(params);
	unsigned int rate = params_rate(params);
	struct imx_ssi *ssi_mode = (struct imx_ssi *)cpu_dai->private_data;
	int ret = 0;
	u32 dai_format;

#if AK4647_SSI_MASTER
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	    SND_SOC_DAIFMT_CBM_CFM;
#else
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	    SND_SOC_DAIFMT_CBS_CFS;
#endif

	ssi_mode->sync_mode = 1;
	if (channels == 1)
		ssi_mode->network_mode = 0;
	else
		ssi_mode->network_mode = 1;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret < 0)
		return ret;

	/* set i.MX active slot mask */
	snd_soc_dai_set_tdm_slot(cpu_dai,
				 channels == 1 ? 0xfffffffe : 0xfffffffc, 2);

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret < 0)
		return ret;

	/* set the SSI system clock as input (unused) */
	snd_soc_dai_set_sysclk(cpu_dai, IMX_SSP_SYS_CLK, 0, SND_SOC_CLOCK_IN);

	snd_soc_dai_set_sysclk(codec_dai, 0, rate, 0);

	/* set codec BCLK division for sample rate */
	snd_soc_dai_set_clkdiv(codec_dai, 0, 0);

	return 0;
}

/*
 * imx_3stack ak4647 HiFi DAI operations.
 */
static struct snd_soc_ops imx_3stack_hifi_ops = {
	.hw_params = imx_3stack_hifi_hw_params,
};

static int ak4647_get_jack(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = ak4647_jack_func;
	return 0;
}

static int ak4647_set_jack(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	if (ak4647_jack_func == ucontrol->value.integer.value[0])
		return 0;

	ak4647_jack_func = ucontrol->value.integer.value[0];

	if (ak4647_jack_func)
		snd_soc_dapm_enable_pin(codec, "Headphone Jack");
	else
		snd_soc_dapm_disable_pin(codec, "Headphone Jack");

	snd_soc_dapm_sync(codec);
	return 1;
}

static int ak4647_get_spk(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = ak4647_spk_func;
	return 0;
}

static int ak4647_set_spk(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	if (ak4647_spk_func == ucontrol->value.integer.value[0])
		return 0;

	ak4647_spk_func = ucontrol->value.integer.value[0];
	if (ak4647_spk_func)
		snd_soc_dapm_enable_pin(codec, "Line Out Jack");
	else
		snd_soc_dapm_disable_pin(codec, "Line Out Jack");

	snd_soc_dapm_sync(codec);
	return 1;
}

static int spk_amp_event(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event)
{
	struct imx_3stack_priv *priv = &card_priv;
	struct platform_device *pdev = priv->pdev;
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

	if (plat->amp_enable == NULL)
		return 0;

	if (SND_SOC_DAPM_EVENT_ON(event))
		plat->amp_enable(1);
	else
		plat->amp_enable(0);
	return 0;
}

/* imx_3stack card dapm widgets */
static const struct snd_soc_dapm_widget imx_3stack_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Mic1 Jack", NULL),
	SND_SOC_DAPM_LINE("Line In Jack", NULL),
	SND_SOC_DAPM_LINE("Line Out Jack", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", spk_amp_event),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* mic is connected to mic1 - with bias */
	{"Left Input", NULL, "Mic1 Jack"},

	/* Line in jack */
	{"Left Input", NULL, "Line In Jack"},
	{"Right Input", NULL, "Line In Jack"},

	/* Headphone jack */
	{"Headphone Jack", NULL, "HPL"},
	{"Headphone Jack", NULL, "HPR"},

	/* Line out jack */
	{"Line Out Jack", NULL, "LOUT"},

	/* Ext Spk */
	{"Ext Spk", NULL, "LOUT"},

};

static const char *jack_function[] = { "off", "on" };

static const char *spk_function[] = { "off", "on" };

static const struct soc_enum ak4647_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, jack_function),
	SOC_ENUM_SINGLE_EXT(2, spk_function),
};

static const struct snd_kcontrol_new ak4647_card_controls[] = {
	SOC_ENUM_EXT("Jack Function", ak4647_enum[0], ak4647_get_jack,
		     ak4647_set_jack),
	SOC_ENUM_EXT("Speaker Function", ak4647_enum[1], ak4647_get_spk,
		     ak4647_set_spk),
};

static void headphone_detect_handler(struct work_struct *work)
{
	struct imx_3stack_priv *priv = &card_priv;
	struct platform_device *pdev = priv->pdev;

	sysfs_notify(&pdev->dev.kobj, NULL, "headphone");
}

static irqreturn_t imx_headphone_detect_handler(int irq, void *dev_id)
{
	schedule_work(&hp_event);
	return IRQ_HANDLED;

}

static ssize_t show_headphone(struct device_driver *dev, char *buf)
{
	struct imx_3stack_priv *priv = &card_priv;
	struct platform_device *pdev = priv->pdev;
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	unsigned int value;

	value = plat->hp_status();

	if (value == 0)
		strcpy(buf, "speaker\n");
	else
		strcpy(buf, "headphone\n");

	return strlen(buf);
}

DRIVER_ATTR(headphone, S_IRUGO | S_IWUSR, show_headphone, NULL);

static int imx_3stack_ak4647_init(struct snd_soc_codec *codec)
{
	int i, ret;
	for (i = 0; i < ARRAY_SIZE(ak4647_card_controls); i++) {
		ret = snd_ctl_add(codec->card,
				  snd_soc_cnew(&ak4647_card_controls[i],
					       codec, NULL));
		if (ret < 0)
			return ret;
	}

	snd_soc_dapm_new_controls(codec, imx_3stack_dapm_widgets,
				  ARRAY_SIZE(imx_3stack_dapm_widgets));

	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_sync(codec);

	return 0;
}

static struct snd_soc_dai_link imx_3stack_dai = {
	.name = "ak4647",
	.stream_name = "ak4647",
	.codec_dai = &ak4647_hifi_dai,
	.init = imx_3stack_ak4647_init,
	.ops = &imx_3stack_hifi_ops,
};

static struct snd_soc_card snd_soc_card_imx_3stack = {
	.name = "imx-3stack",
	.platform = &imx_soc_platform,
	.dai_link = &imx_3stack_dai,
	.num_links = 1,
};

static struct snd_soc_device imx_3stack_snd_devdata = {
	.card = &snd_soc_card_imx_3stack,
	.codec_dev = &soc_codec_dev_ak4647,
};

/*
 * This function will register the snd_soc_pcm_link drivers.
 * It also registers devices for platform DMA, I2S, SSP and registers an
 * I2C driver to probe the codec.
 */
static int __init imx_3stack_ak4647_probe(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *dev_data = pdev->dev.platform_data;
	struct imx_3stack_priv *priv = &card_priv;
	struct snd_soc_dai *ak4647_cpu_dai;
	int ret = 0;

	dev_data->init();

	if (dev_data->src_port == 1)
		ak4647_cpu_dai = imx_ssi_dai[0];
	else
		ak4647_cpu_dai = imx_ssi_dai[2];

	imx_3stack_dai.cpu_dai = ak4647_cpu_dai;

	imx_3stack_init_dam(dev_data->src_port, dev_data->ext_port);

	ret = request_irq(dev_data->intr_id_hp, imx_headphone_detect_handler, 0,
			"headphone", NULL);
	if (ret < 0)
		goto err;

	ret = driver_create_file(pdev->dev.driver, &driver_attr_headphone);
	if (ret < 0)
		goto sysfs_err;

	priv->pdev = pdev;
	return ret;

sysfs_err:
	free_irq(dev_data->intr_id_hp, NULL);
err:
	return ret;
}

static int __devexit imx_3stack_ak4647_remove(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *dev_data = pdev->dev.platform_data;
	free_irq(dev_data->intr_id_hp, NULL);
	driver_remove_file(pdev->dev.driver, &driver_attr_headphone);
	return 0;
}

static struct platform_driver imx_3stack_ak4647_driver = {
	.probe = imx_3stack_ak4647_probe,
	.remove = __devexit_p(imx_3stack_ak4647_remove),
	.driver = {
		   .name = "imx-3stack-ak4647",
		   .owner = THIS_MODULE,
		   },
};

static struct platform_device *imx_3stack_snd_device;

static int __init imx_3stack_asoc_init(void)
{
	int ret;

	ret = platform_driver_register(&imx_3stack_ak4647_driver);
	if (ret < 0)
		goto exit;

	if (snd_soc_card_imx_3stack.codec == NULL) {
		ret = -ENOMEM;
		goto err_device_alloc;
	}

	imx_3stack_snd_device = platform_device_alloc("soc-audio", 3);
	if (!imx_3stack_snd_device)
		goto err_device_alloc;
	platform_set_drvdata(imx_3stack_snd_device, &imx_3stack_snd_devdata);
	imx_3stack_snd_devdata.dev = &imx_3stack_snd_device->dev;
	ret = platform_device_add(imx_3stack_snd_device);
	if (0 == ret)
		goto exit;

	platform_device_put(imx_3stack_snd_device);
err_device_alloc:
	platform_driver_unregister(&imx_3stack_ak4647_driver);
exit:
	return ret;
}

static void __exit imx_3stack_asoc_exit(void)
{
	platform_driver_unregister(&imx_3stack_ak4647_driver);
	platform_device_unregister(imx_3stack_snd_device);
}

module_init(imx_3stack_asoc_init);
module_exit(imx_3stack_asoc_exit);

/* Module information */
MODULE_DESCRIPTION("ALSA SoC ak4647 imx_3stack");
MODULE_LICENSE("GPL");
