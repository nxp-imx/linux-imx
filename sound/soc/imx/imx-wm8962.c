/*
 * imx-wm8962.c
 *
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/fsl_devices.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/jack.h>
#include <mach/dma.h>
#include <mach/clock.h>
#include <mach/audmux.h>
#include <mach/gpio.h>
#include <asm/mach-types.h>

#include "imx-ssi.h"
#include "../codecs/wm8962.h"

struct imx_priv {
	int sysclk;         /*mclk from the outside*/
	int codec_sysclk;
	int dai_hifi;
	int hp_irq;
	int hp_status;
	int amic_irq;
	int amic_status;
	struct platform_device *pdev;
};
unsigned int sample_format = SNDRV_PCM_FMTBIT_S16_LE;
static struct imx_priv card_priv;
static struct snd_soc_card snd_soc_card_imx;
static struct snd_soc_codec *gcodec;

static int imx_hifi_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct imx_priv *priv = &card_priv;
	struct mxc_audio_platform_data *plat = priv->pdev->dev.platform_data;

	if (!codec_dai->active)
		plat->clock_enable(1);

	return 0;
}

static void imx_hifi_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct imx_priv *priv = &card_priv;
	struct mxc_audio_platform_data *plat = priv->pdev->dev.platform_data;

	if (!codec_dai->active)
		plat->clock_enable(0);

	return;
}

static int imx_hifi_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct imx_priv *priv = &card_priv;
	unsigned int channels = params_channels(params);
	unsigned int sample_rate = 44100;
	int ret = 0;
	u32 dai_format;
	unsigned int pll_out;

	if (codec->dapm.bias_level != SND_SOC_BIAS_OFF)
		return 0;

	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBM_CFM;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret < 0)
		return ret;

	/* set i.MX active slot mask */
	snd_soc_dai_set_tdm_slot(cpu_dai,
				 channels == 1 ? 0xfffffffe : 0xfffffffc,
				 channels == 1 ? 0xfffffffe : 0xfffffffc,
				 2, 32);

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret < 0)
		return ret;

	sample_rate = params_rate(params);
	sample_format = params_format(params);

	if (sample_format == SNDRV_PCM_FORMAT_S24_LE)
		pll_out = sample_rate * 192;
	else
		pll_out = sample_rate * 256;

	ret = snd_soc_dai_set_pll(codec_dai, WM8962_FLL_MCLK,
				  WM8962_FLL_MCLK, priv->sysclk,
				  pll_out);
	if (ret < 0)
		pr_err("Failed to start FLL: %d\n", ret);

	ret = snd_soc_dai_set_sysclk(codec_dai,
					 WM8962_SYSCLK_FLL,
					 pll_out,
					 SND_SOC_CLOCK_IN);
	if (ret < 0) {
		pr_err("Failed to set SYSCLK: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct snd_kcontrol_new controls[] = {
	SOC_DAPM_PIN_SWITCH("Ext Spk"),
};

/* imx card dapm widgets */
static const struct snd_soc_dapm_widget imx_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_MIC("AMIC", NULL),
	SND_SOC_DAPM_MIC("DMIC", NULL),
};

/* imx machine connections to the codec pins */
static const struct snd_soc_dapm_route audio_map[] = {
	{ "Headphone Jack", NULL, "HPOUTL" },
	{ "Headphone Jack", NULL, "HPOUTR" },

	{ "Ext Spk", NULL, "SPKOUTL" },
	{ "Ext Spk", NULL, "SPKOUTR" },

	{ "MICBIAS", NULL, "AMIC" },
	{ "IN3R", NULL, "MICBIAS" },

	{ "DMIC", NULL, "MICBIAS" },
	{ "DMICDAT", NULL, "DMIC" },

};

static void headphone_detect_handler(struct work_struct *wor)
{
	struct imx_priv *priv = &card_priv;
	struct platform_device *pdev = priv->pdev;
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	char *envp[3];
	char *buf;

	/*sysfs_notify(&pdev->dev.kobj, NULL, "headphone");*/
	priv->hp_status = gpio_get_value(plat->hp_gpio);

	/* setup a message for userspace headphone in */
	buf = kmalloc(32, GFP_ATOMIC);
	if (!buf) {
		pr_err("%s kmalloc failed\n", __func__);
		return;
	}

	if (priv->hp_status != plat->hp_active_low)
		snprintf(buf, 32, "STATE=%d", 2);
	else
		snprintf(buf, 32, "STATE=%d", 0);

	envp[0] = "NAME=headphone";
	envp[1] = buf;
	envp[2] = NULL;
	kobject_uevent_env(&pdev->dev.kobj, KOBJ_CHANGE, envp);
	kfree(buf);

	enable_irq(priv->hp_irq);

	return;
}

static DECLARE_DELAYED_WORK(hp_event, headphone_detect_handler);

static irqreturn_t imx_headphone_detect_handler(int irq, void *data)
{
	disable_irq_nosync(irq);
	schedule_delayed_work(&hp_event, msecs_to_jiffies(200));
	return IRQ_HANDLED;
}

static ssize_t show_headphone(struct device_driver *dev, char *buf)
{
	struct imx_priv *priv = &card_priv;
	struct platform_device *pdev = priv->pdev;
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

	/* determine whether hp is plugged in */
	priv->hp_status = gpio_get_value(plat->hp_gpio);

	if (priv->hp_status != plat->hp_active_low)
		strcpy(buf, "headphone\n");
	else
		strcpy(buf, "speaker\n");

	return strlen(buf);
}

static DRIVER_ATTR(headphone, S_IRUGO | S_IWUSR, show_headphone, NULL);

static void amic_detect_handler(struct work_struct *work)
{
	struct imx_priv *priv = &card_priv;
	struct platform_device *pdev = priv->pdev;
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	char *envp[3];
	char *buf;

	/* sysfs_notify(&pdev->dev.kobj, NULL, "amic"); */
	priv->amic_status = gpio_get_value(plat->mic_gpio);

	/* if amic is inserted, disable dmic */
	if (priv->amic_status != plat->mic_active_low)
		snd_soc_dapm_nc_pin(&gcodec->dapm, "DMIC");
	else
		snd_soc_dapm_enable_pin(&gcodec->dapm, "DMIC");

	/* setup a message for userspace headphone in */
	buf = kmalloc(32, GFP_ATOMIC);
	if (!buf) {
		pr_err("%s kmalloc failed\n", __func__);
		return;
	}

	if (priv->amic_status == 0)
		snprintf(buf, 32, "STATE=%d", 2);
	else
		snprintf(buf, 32, "STATE=%d", 0);

	envp[0] = "NAME=amic";
	envp[1] = buf;
	envp[2] = NULL;
	kobject_uevent_env(&pdev->dev.kobj, KOBJ_CHANGE, envp);
	kfree(buf);

	enable_irq(priv->amic_irq);
}

static DECLARE_DELAYED_WORK(amic_event, amic_detect_handler);

static irqreturn_t imx_amic_detect_handler(int irq, void *data)
{
	disable_irq_nosync(irq);
	schedule_delayed_work(&amic_event, msecs_to_jiffies(200));
	return IRQ_HANDLED;
}

static ssize_t show_amic(struct device_driver *dev, char *buf)
{
	struct imx_priv *priv = &card_priv;
	struct platform_device *pdev = priv->pdev;
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

	/* determine whether amic is plugged in */
	priv->amic_status = gpio_get_value(plat->hp_gpio);

	if (priv->amic_status != plat->mic_active_low)
		strcpy(buf, "amic\n");
	else
		strcpy(buf, "dmic\n");

	return strlen(buf);
}

static DRIVER_ATTR(amic, S_IRUGO | S_IWUSR, show_amic, NULL);

static int imx_wm8962_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct imx_priv *priv = &card_priv;
	struct platform_device *pdev = priv->pdev;
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	int ret = 0;

	gcodec = rtd->codec;

	/* Add imx specific widgets */
	snd_soc_dapm_new_controls(&codec->dapm, imx_dapm_widgets,
				  ARRAY_SIZE(imx_dapm_widgets));

	/* Set up imx specific audio path audio_map */
	snd_soc_dapm_add_routes(&codec->dapm, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_enable_pin(&codec->dapm, "Headphone Jack");
	snd_soc_dapm_enable_pin(&codec->dapm, "AMIC");

	snd_soc_dapm_sync(&codec->dapm);

	if (plat->hp_gpio != -1) {
			priv->hp_irq = gpio_to_irq(plat->hp_gpio);

			ret = request_irq(priv->hp_irq,
						imx_headphone_detect_handler,
						IRQ_TYPE_EDGE_BOTH, pdev->name, priv);

			if (ret < 0) {
				ret = -EINVAL;
				return ret;
			}

			ret = driver_create_file(pdev->dev.driver,
							&driver_attr_headphone);
			if (ret < 0) {
				ret = -EINVAL;
				return ret;
			}
		}

	if (plat->mic_gpio != -1) {
		priv->amic_irq = gpio_to_irq(plat->mic_gpio);

		ret = request_irq(priv->amic_irq,
					imx_amic_detect_handler,
					IRQ_TYPE_EDGE_BOTH, pdev->name, priv);

		if (ret < 0) {
			ret = -EINVAL;
			return ret;
		}

		ret = driver_create_file(pdev->dev.driver, &driver_attr_amic);
		if (ret < 0) {
			ret = -EINVAL;
			return ret;
		}

		priv->amic_status = gpio_get_value(plat->mic_gpio);

		/* if amic is inserted, disable DMIC */
		if (priv->amic_status != plat->mic_active_low)
			snd_soc_dapm_nc_pin(&codec->dapm, "DMIC");
		else
			snd_soc_dapm_enable_pin(&codec->dapm, "DMIC");
	} else if (!snd_soc_dapm_get_pin_status(&codec->dapm, "DMICDAT"))
		snd_soc_dapm_nc_pin(&codec->dapm, "DMIC");

	return 0;
}

static struct snd_soc_ops imx_hifi_ops = {
	.startup = imx_hifi_startup,
	.shutdown = imx_hifi_shutdown,
	.hw_params = imx_hifi_hw_params,
};

static struct snd_soc_dai_link imx_dai[] = {
	{
		.name = "HiFi",
		.stream_name = "HiFi",
		.codec_dai_name	= "wm8962",
		.codec_name	= "wm8962.0-001a",
		.cpu_dai_name	= "imx-ssi.1",
		.platform_name	= "imx-pcm-audio.1",
		.init		= imx_wm8962_init,
		.ops		= &imx_hifi_ops,
	},
};

static struct snd_soc_card snd_soc_card_imx = {
	.name		= "wm8962-audio",
	.dai_link	= imx_dai,
	.num_links	= ARRAY_SIZE(imx_dai),
};

static int imx_audmux_config(int slave, int master)
{
	unsigned int ptcr, pdcr;
	slave = slave - 1;
	master = master - 1;

	ptcr = MXC_AUDMUX_V2_PTCR_SYN |
		MXC_AUDMUX_V2_PTCR_TFSDIR |
		MXC_AUDMUX_V2_PTCR_TFSEL(master) |
		MXC_AUDMUX_V2_PTCR_TCLKDIR |
		MXC_AUDMUX_V2_PTCR_TCSEL(master);
	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(master);
	mxc_audmux_v2_configure_port(slave, ptcr, pdcr);

	ptcr = MXC_AUDMUX_V2_PTCR_SYN;
	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(slave);
	mxc_audmux_v2_configure_port(master, ptcr, pdcr);

	return 0;
}

/*
 * This function will register the snd_soc_pcm_link drivers.
 */
static int __devinit imx_wm8962_probe(struct platform_device *pdev)
{

	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	struct imx_priv *priv = &card_priv;
	int ret = 0;

	priv->pdev = pdev;

	imx_audmux_config(plat->src_port, plat->ext_port);

	if (plat->init && plat->init()) {
		ret = -EINVAL;
		return ret;
	}

	priv->sysclk = plat->sysclk;

	return ret;
}

static int __devexit imx_wm8962_remove(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

	plat->clock_enable(0);

	if (plat->finit)
		plat->finit();

	return 0;
}

static struct platform_driver imx_wm8962_driver = {
	.probe = imx_wm8962_probe,
	.remove = imx_wm8962_remove,
	.driver = {
		   .name = "imx-wm8962",
		   .owner = THIS_MODULE,
		   },
};

static struct platform_device *imx_snd_device;

static int __init imx_asoc_init(void)
{
	int ret;

	ret = platform_driver_register(&imx_wm8962_driver);
	if (ret < 0)
		goto exit;

	if (machine_is_mx6q_sabresd())
		imx_dai[0].codec_name = "wm8962.0-001a";
	else if (machine_is_mx6sl_arm2())
		imx_dai[0].codec_name = "wm8962.1-001a";

	imx_snd_device = platform_device_alloc("soc-audio", 5);
	if (!imx_snd_device)
		goto err_device_alloc;

	platform_set_drvdata(imx_snd_device, &snd_soc_card_imx);

	ret = platform_device_add(imx_snd_device);

	if (0 == ret)
		goto exit;

	platform_device_put(imx_snd_device);

err_device_alloc:
	platform_driver_unregister(&imx_wm8962_driver);
exit:
	return ret;
}

static void __exit imx_asoc_exit(void)
{
	platform_driver_unregister(&imx_wm8962_driver);
	platform_device_unregister(imx_snd_device);
}

module_init(imx_asoc_init);
module_exit(imx_asoc_exit);

/* Module information */
MODULE_DESCRIPTION("ALSA SoC imx wm8962");
MODULE_LICENSE("GPL");
