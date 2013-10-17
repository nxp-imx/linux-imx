/*
 * Copyright 2008-2013 Freescale Semiconductor, Inc. All Rights Reserved.
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
  * @file       fsl-esai.c
  * @brief      this file implements the esai interface
  *             in according to ASoC architeture
  */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/clk.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "fsl_esai.h"
#include "imx-pcm.h"

#define IMX_ESAI_NET            (1 << 0)
#define IMX_ESAI_SYN            (1 << 1)

static int fsl_esai_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
				   int clk_id, unsigned int freq, int dir)
{
	struct fsl_esai *esai = snd_soc_dai_get_drvdata(cpu_dai);
	u32 ecr, tccr, rccr;

	ecr = readl(esai->base + ESAI_ECR);
	tccr = readl(esai->base + ESAI_TCCR);
	rccr = readl(esai->base + ESAI_RCCR);

	if (dir == SND_SOC_CLOCK_IN) {
		tccr &= ~(ESAI_TCCR_THCKD | ESAI_TCCR_TCKD | ESAI_TCCR_TFSD);
		rccr &= ~(ESAI_RCCR_RHCKD | ESAI_RCCR_RCKD | ESAI_RCCR_RFSD);
	} else {
		tccr |= ESAI_TCCR_THCKD | ESAI_TCCR_TCKD | ESAI_TCCR_TFSD;
		rccr |= ESAI_RCCR_RHCKD | ESAI_RCCR_RCKD | ESAI_RCCR_RFSD;

		if (clk_id == ESAI_CLK_FSYS) {
			ecr &= ~(ESAI_ECR_ETI | ESAI_ECR_ETO);
			ecr &= ~(ESAI_ECR_ERI | ESAI_ECR_ERO);
		} else if (clk_id == ESAI_CLK_EXTAL) {
			ecr |= ESAI_ECR_ETI;
			ecr |= ESAI_ECR_ETO;
			ecr |= ESAI_ECR_ERI;
			ecr |= ESAI_ECR_ERO;
		} else if (clk_id == ESAI_CLK_EXTAL_DIV) {
			ecr |= ESAI_ECR_ETI;
			ecr &= ~ESAI_ECR_ETO;
			ecr |= ESAI_ECR_ERI;
			ecr &= ~ESAI_ECR_ERO;
		}
	}

	writel(ecr, esai->base + ESAI_ECR);
	writel(tccr, esai->base + ESAI_TCCR);
	writel(rccr, esai->base + ESAI_RCCR);

	ESAI_DUMP();
	return 0;
}

static int fsl_esai_set_dai_clkdiv(struct snd_soc_dai *cpu_dai,
				   int div_id, int div)
{
	struct fsl_esai *esai = snd_soc_dai_get_drvdata(cpu_dai);
	u32 tccr, rccr;

	tccr = readl(esai->base + ESAI_TCCR);
	rccr = readl(esai->base + ESAI_RCCR);

	switch (div_id) {
	case ESAI_TX_DIV_PSR:
		if ((div << ESAI_TCCR_TPSR_SHIFT) & ESAI_TCCR_TPSR_MASK)
			return -EINVAL;
		tccr &= ESAI_TCCR_TPSR_MASK;
		if (div)
			tccr |= ESAI_TCCR_TPSR_BYPASS;
		else
			tccr &= ~ESAI_TCCR_TPSR_DIV8;
		break;
	case ESAI_TX_DIV_PM:
		if ((div << ESAI_TCCR_TPM_SHIFT) & ESAI_TCCR_TPM_MASK)
			return -EINVAL;
		tccr &= ESAI_TCCR_TPM_MASK;
		tccr |= ESAI_TCCR_TPM(div);
		break;
	case ESAI_TX_DIV_FP:
		if ((div << ESAI_TCCR_TFP_SHIFT) & ESAI_TCCR_TFP_MASK)
			return -EINVAL;
		tccr &= ESAI_TCCR_TFP_MASK;
		tccr |= ESAI_TCCR_TFP(div);
		break;
	case ESAI_RX_DIV_PSR:
		if ((div << ESAI_RCCR_RPSR_SHIFT) & ESAI_RCCR_RPSR_MASK)
			return -EINVAL;
		rccr &= ESAI_RCCR_RPSR_MASK;
		if (div)
			rccr |= ESAI_RCCR_RPSR_BYPASS;
		else
			rccr &= ~ESAI_RCCR_RPSR_DIV8;
		break;
	case ESAI_RX_DIV_PM:
		if ((div << ESAI_RCCR_RPM_SHIFT) & ESAI_RCCR_RPM_MASK)
			return -EINVAL;
		rccr &= ESAI_RCCR_RPM_MASK;
		rccr |= ESAI_RCCR_RPM(div);
		break;
	case ESAI_RX_DIV_FP:
		if ((div << ESAI_RCCR_RFP_SHIFT) & ESAI_RCCR_RFP_MASK)
			return -EINVAL;
		rccr &= ESAI_RCCR_RFP_MASK;
		rccr |= ESAI_RCCR_RFP(div);
		break;
	default:
		return -EINVAL;
	}
	writel(tccr, esai->base + ESAI_TCCR);
	writel(rccr, esai->base + ESAI_RCCR);
	return 0;
}

/*
 * ESAI Network Mode or TDM slots configuration.
 */
static int fsl_esai_set_dai_tdm_slot(struct snd_soc_dai *cpu_dai,
	unsigned int tx_mask, unsigned int rx_mask, int slots, int slot_width)
{
	struct fsl_esai *esai = snd_soc_dai_get_drvdata(cpu_dai);
	u32 tccr, rccr;

	tccr = readl(esai->base + ESAI_TCCR);

	tccr &= ESAI_TCCR_TDC_MASK;
	tccr |= ESAI_TCCR_TDC(slots - 1);

	writel(tccr, esai->base + ESAI_TCCR);
	writel((tx_mask & 0xffff), esai->base + ESAI_TSMA);
	writel(((tx_mask >> 16) & 0xffff), esai->base + ESAI_TSMB);

	rccr = readl(esai->base + ESAI_RCCR);

	rccr &= ESAI_RCCR_RDC_MASK;
	rccr |= ESAI_RCCR_RDC(slots - 1);

	writel(rccr, esai->base + ESAI_RCCR);
	writel((rx_mask & 0xffff), esai->base + ESAI_RSMA);
	writel(((rx_mask >> 16) & 0xffff), esai->base + ESAI_RSMB);

	ESAI_DUMP();
	return 0;
}

/*
 * ESAI DAI format configuration.
 */
static int fsl_esai_set_dai_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	struct fsl_esai *esai = snd_soc_dai_get_drvdata(cpu_dai);
	u32 tcr, tccr, rcr, rccr, saicr;

	tcr = readl(esai->base + ESAI_TCR);
	tccr = readl(esai->base + ESAI_TCCR);
	rcr = readl(esai->base + ESAI_RCR);
	rccr = readl(esai->base + ESAI_RCCR);
	saicr = readl(esai->base + ESAI_SAICR);

	/* DAI mode */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		/* data on rising edge of bclk, frame low 1clk before data */
		tcr &= ~ESAI_TCR_TFSL;
		tcr |= ESAI_TCR_TFSR;
		rcr &= ~ESAI_RCR_RFSL;
		rcr |= ESAI_RCR_RFSR;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		/* data on rising edge of bclk, frame high with data */
		tcr &= ~(ESAI_TCR_TFSL | ESAI_TCR_TFSR);
		rcr &= ~(ESAI_RCR_RFSL | ESAI_RCR_RFSR);
		break;
	case SND_SOC_DAIFMT_DSP_B:
		/* data on rising edge of bclk, frame high with data */
		tcr |= ESAI_TCR_TFSL;
		rcr |= ESAI_RCR_RFSL;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		/* data on rising edge of bclk, frame high 1clk before data */
		tcr |= ESAI_TCR_TFSL;
		rcr |= ESAI_RCR_RFSL;
		break;
	default:
		return -EINVAL;
	}

	/* DAI clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_IB_IF:
		tccr |= ESAI_TCCR_TFSP;
		tccr &= ~(ESAI_TCCR_TCKP | ESAI_TCCR_THCKP);
		rccr &= ~(ESAI_RCCR_RCKP | ESAI_RCCR_RHCKP);
		rccr |= ESAI_RCCR_RFSP;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		tccr &= ~(ESAI_TCCR_TCKP | ESAI_TCCR_THCKP | ESAI_TCCR_TFSP);
		rccr &= ~(ESAI_RCCR_RCKP | ESAI_RCCR_RHCKP | ESAI_RCCR_RFSP);
		break;
	case SND_SOC_DAIFMT_NB_IF:
		tccr |= ESAI_TCCR_TCKP | ESAI_TCCR_THCKP | ESAI_TCCR_TFSP;
		rccr |= ESAI_RCCR_RCKP | ESAI_RCCR_RHCKP | ESAI_RCCR_RFSP;
		break;
	case SND_SOC_DAIFMT_NB_NF:
		tccr &= ~ESAI_TCCR_TFSP;
		tccr |= ESAI_TCCR_TCKP | ESAI_TCCR_THCKP;
		rccr &= ~ESAI_RCCR_RFSP;
		rccr |= ESAI_RCCR_RCKP | ESAI_RCCR_RHCKP;
		break;
	default:
		return -EINVAL;
	}

	/* DAI clock master masks */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		tccr &= ~(ESAI_TCCR_TFSD | ESAI_TCCR_TCKD);
		rccr &= ~(ESAI_RCCR_RFSD | ESAI_RCCR_RCKD);
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		tccr &= ~ESAI_TCCR_TFSD;
		tccr |= ESAI_TCCR_TCKD;
		rccr &= ~ESAI_RCCR_RFSD;
		rccr |= ESAI_RCCR_RCKD;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		tccr &= ~ESAI_TCCR_TCKD;
		tccr |= ESAI_TCCR_TFSD;
		rccr &= ~ESAI_RCCR_RCKD;
		rccr |= ESAI_RCCR_RFSD;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		tccr |= (ESAI_TCCR_TFSD | ESAI_TCCR_TCKD);
		rccr |= (ESAI_RCCR_RFSD | ESAI_RCCR_RCKD);
		break;
	default:
		return -EINVAL;
	}

	/* sync */
	if (esai->flags & IMX_ESAI_SYN)
		saicr |= ESAI_SAICR_SYNC;
	else
		saicr &= ~ESAI_SAICR_SYNC;

	tcr &= ESAI_TCR_TMOD_MASK;
	rcr &= ESAI_RCR_RMOD_MASK;
	if (esai->flags & IMX_ESAI_NET) {
		tcr |= ESAI_TCR_TMOD_NETWORK;
		rcr |= ESAI_RCR_RMOD_NETWORK;
	} else {
		tcr |= ESAI_TCR_TMOD_NORMAL;
		rcr |= ESAI_RCR_RMOD_NORMAL;
	}

	writel(tcr, esai->base + ESAI_TCR);
	writel(tccr, esai->base + ESAI_TCCR);
	writel(rcr, esai->base + ESAI_RCR);
	writel(rccr, esai->base + ESAI_RCCR);

	writel(saicr, esai->base + ESAI_SAICR);

	ESAI_DUMP();
	return 0;
}

static int fsl_esai_startup(struct snd_pcm_substream *substream,
			    struct snd_soc_dai *cpu_dai)
{
	struct fsl_esai *esai = snd_soc_dai_get_drvdata(cpu_dai);

	clk_enable(esai->clk);
	if (!cpu_dai->active) {
		writel(ESAI_GPIO_ESAI, esai->base + ESAI_PRRC);
		writel(ESAI_GPIO_ESAI, esai->base + ESAI_PCRC);
	}
	ESAI_DUMP();
	return 0;
}

/*
 * This function is called to initialize the TX port before enable
 * the tx port.
 */
static int fsl_esai_hw_tx_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *cpu_dai)
{
	struct fsl_esai *esai = snd_soc_dai_get_drvdata(cpu_dai);
	u32 tcr, tfcr;
	unsigned int channels;

	tcr = readl(esai->base + ESAI_TCR);
	tfcr = readl(esai->base + ESAI_TFCR);

	tfcr |= ESAI_TFCR_TFR;
	writel(tfcr, esai->base + ESAI_TFCR);
	tfcr &= ~ESAI_TFCR_TFR;
	/* DAI data (word) size */
	tfcr &= ESAI_TFCR_TWA_MASK;
	tcr &= ESAI_TCR_TSWS_MASK;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		tfcr |= ESAI_WORD_LEN_16;
		tcr |= ESAI_TCR_TSHFD_MSB | ESAI_TCR_TSWS_STL32_WDL16;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		tfcr |= ESAI_WORD_LEN_20;
		tcr |= ESAI_TCR_TSHFD_MSB | ESAI_TCR_TSWS_STL32_WDL20;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		tfcr |= ESAI_WORD_LEN_24;
		tcr |= ESAI_TCR_TSHFD_MSB | ESAI_TCR_TSWS_STL32_WDL24;
		break;
	default:
		return -EINVAL;
	}

	channels = params_channels(params);
	tfcr &= ESAI_TFCR_TE_MASK;
	tfcr |= ESAI_TFCR_TE(channels);

	tfcr |= ESAI_TFCR_TFWM(esai->fifo_depth);

	/* Left aligned, Zero padding */
	tcr |= ESAI_TCR_PADC;
	/* TDR initialized from the FIFO */
	tfcr |= ESAI_TFCR_TIEN;

	writel(tcr, esai->base + ESAI_TCR);
	writel(tfcr, esai->base + ESAI_TFCR);

	ESAI_DUMP();
	return 0;
}

/*
 * This function is called to initialize the RX port before enable
 * the rx port.
 */
static int fsl_esai_hw_rx_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *cpu_dai)
{
	struct fsl_esai *esai = snd_soc_dai_get_drvdata(cpu_dai);
	u32 rcr, rfcr;
	unsigned int channels;

	rcr = readl(esai->base + ESAI_RCR);
	rfcr = readl(esai->base + ESAI_RFCR);

	rfcr |= ESAI_RFCR_RFR;
	writel(rfcr, esai->base + ESAI_RFCR);
	rfcr &= ~ESAI_RFCR_RFR;

	rfcr &= ESAI_RFCR_RWA_MASK;
	rcr &= ESAI_RCR_RSWS_MASK;
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		rfcr |= ESAI_WORD_LEN_16;
		rcr |= ESAI_RCR_RSHFD_MSB | ESAI_RCR_RSWS_STL32_WDL16;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		rfcr |= ESAI_WORD_LEN_20;
		rcr |= ESAI_RCR_RSHFD_MSB | ESAI_RCR_RSWS_STL32_WDL20;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		rfcr |= ESAI_WORD_LEN_24;
		rcr |= ESAI_RCR_RSHFD_MSB | ESAI_RCR_RSWS_STL32_WDL24;
		break;
	default:
		return -EINVAL;
	}

	channels = params_channels(params);
	rfcr &= ESAI_RFCR_RE_MASK;
	rfcr |= ESAI_RFCR_RE(channels);

	rfcr |= ESAI_RFCR_RFWM(esai->fifo_depth);

	writel(rcr, esai->base + ESAI_RCR);
	writel(rfcr, esai->base + ESAI_RFCR);

	ESAI_DUMP();
	return 0;
}

/*
 * This function is called to initialize the TX or RX port,
 */
static int fsl_esai_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params,
			      struct snd_soc_dai *cpu_dai)
{
	struct fsl_esai *esai = snd_soc_dai_get_drvdata(cpu_dai);

	/* Tx/Rx config */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (readl(esai->base + ESAI_TCR) & ESAI_TCR_TE0)
			return 0;

		return fsl_esai_hw_tx_params(substream, params, cpu_dai);
	} else {
		if (readl(esai->base + ESAI_RCR) & ESAI_RCR_RE1)
			return 0;

		return fsl_esai_hw_rx_params(substream, params, cpu_dai);
	}
}

static void fsl_esai_shutdown(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *cpu_dai)
{
	struct fsl_esai *esai = snd_soc_dai_get_drvdata(cpu_dai);

	clk_disable(esai->clk);
}

static int fsl_esai_trigger(struct snd_pcm_substream *substream, int cmd,
			    struct snd_soc_dai *cpu_dai)
{
	struct fsl_esai *esai = snd_soc_dai_get_drvdata(cpu_dai);
	u32 reg, tfcr = 0, rfcr = 0;
	int i;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		tfcr = readl(esai->base + ESAI_TFCR);
		reg = readl(esai->base + ESAI_TCR);
	} else {
		rfcr = readl(esai->base + ESAI_RFCR);
		reg = readl(esai->base + ESAI_RCR);
	}
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			tfcr |= ESAI_TFCR_TFEN;
			writel(tfcr, esai->base + ESAI_TFCR);
			/* write initial words to ETDR register */
			for (i = 0; i < substream->runtime->channels; i++)
				writel(0x0, esai->base + ESAI_ETDR);
			reg |= ESAI_TCR_TE(substream->runtime->channels);
			writel(reg, esai->base + ESAI_TCR);
		} else {
			rfcr |= ESAI_RFCR_RFEN;
			writel(rfcr, esai->base + ESAI_RFCR);
			reg |= ESAI_RCR_RE(substream->runtime->channels);
			writel(reg, esai->base + ESAI_RCR);
		}
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			reg &= ~ESAI_TCR_TE(substream->runtime->channels);
			writel(reg, esai->base + ESAI_TCR);
			tfcr |= ESAI_TFCR_TFR;
			tfcr &= ~ESAI_TFCR_TFEN;
			writel(tfcr, esai->base + ESAI_TFCR);
			tfcr &= ~ESAI_TFCR_TFR;
			writel(tfcr, esai->base + ESAI_TFCR);
		} else {
			reg &= ~ESAI_RCR_RE(substream->runtime->channels);
			writel(reg, esai->base + ESAI_RCR);
			rfcr |= ESAI_RFCR_RFR;
			rfcr &= ~ESAI_RFCR_RFEN;
			writel(rfcr, esai->base + ESAI_RFCR);
			rfcr &= ~ESAI_RFCR_RFR;
			writel(rfcr, esai->base + ESAI_RFCR);
		}
		break;
	default:
		return -EINVAL;
	}

	ESAI_DUMP();
	return 0;
}

#define IMX_ESAI_RATES  SNDRV_PCM_RATE_8000_192000

#define IMX_ESAI_FORMATS \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
	SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_ops fsl_esai_dai_ops = {
	.startup = fsl_esai_startup,
	.shutdown = fsl_esai_shutdown,
	.trigger = fsl_esai_trigger,
	.hw_params = fsl_esai_hw_params,
	.set_sysclk = fsl_esai_set_dai_sysclk,
	.set_clkdiv = fsl_esai_set_dai_clkdiv,
	.set_fmt = fsl_esai_set_dai_fmt,
	.set_tdm_slot = fsl_esai_set_dai_tdm_slot,
};

static int fsl_esai_dai_probe(struct snd_soc_dai *dai)
{
	struct fsl_esai *esai = snd_soc_dai_get_drvdata(dai);

	dai->playback_dma_data = &esai->dma_params_tx;
	dai->capture_dma_data = &esai->dma_params_rx;
	return 0;
}

static struct snd_soc_dai_driver fsl_esai_dai = {
	.probe = fsl_esai_dai_probe,
	.playback = {
		.stream_name = "esai-Playback",
		.channels_min = 1,
		.channels_max = 12,
		.rates = IMX_ESAI_RATES,
		.formats = IMX_ESAI_FORMATS,
	},
	.capture = {
		.stream_name = "esai-Capture",
		.channels_min = 1,
		.channels_max = 8,
		.rates = IMX_ESAI_RATES,
		.formats = IMX_ESAI_FORMATS,
	},
	.ops = &fsl_esai_dai_ops,
};

static const struct snd_soc_component_driver fsl_esai_component = {
	.name		= "fsl-esai",
};

static int fsl_esai_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct resource res;
	struct fsl_esai *esai;
	const uint32_t *iprop;
	uint32_t flag;
	const char *p;
	u32 dma_events[2];
	int ret = 0;

	esai = devm_kzalloc(&pdev->dev, sizeof(*esai), GFP_KERNEL);
	if (!esai) {
		dev_err(&pdev->dev, "mem allocation failed\n");
		return -ENOMEM;
	}

	ret = of_property_read_u32(np, "fsl,flags", &flag);
	if (ret < 0) {
		dev_err(&pdev->dev, "There is no flag for esai\n");
		return -EINVAL;
	}
	esai->flags = flag;

	esai->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(esai->clk)) {
		ret = PTR_ERR(esai->clk);
		dev_err(&pdev->dev, "Cannot get the clock: %d\n", ret);
		return ret;
	}
	clk_prepare(esai->clk);

	ret = of_address_to_resource(np, 0, &res);
	if (ret) {
		dev_err(&pdev->dev, "could not determine device resources\n");
		goto failed_get_resource;
	}

	esai->base = of_iomap(np, 0);
	if (!esai->base) {
		dev_err(&pdev->dev, "could not map device resources\n");
		ret = -ENOMEM;
		goto failed_iomap;
	}

	esai->irq = irq_of_parse_and_map(np, 0);

	/* Determine the FIFO depth. */
	iprop = of_get_property(np, "fsl,fifo-depth", NULL);
	if (iprop)
		esai->fifo_depth = be32_to_cpup(iprop);
	else
		esai->fifo_depth = 64;

	esai->dma_params_tx.maxburst = 16;
	esai->dma_params_rx.maxburst = 16;

	esai->dma_params_tx.addr = res.start + ESAI_ETDR;
	esai->dma_params_rx.addr = res.start + ESAI_ERDR;

	esai->dma_params_tx.filter_data = &esai->filter_data_tx;
	esai->dma_params_rx.filter_data = &esai->filter_data_rx;

	ret = of_property_read_u32_array(pdev->dev.of_node,
				"fsl,esai-dma-events", dma_events, 2);
	if (ret) {
		dev_err(&pdev->dev, "could not get dma events\n");
		goto failed_get_dma;
	}

	esai->filter_data_tx.dma_request0 = dma_events[0];
	esai->filter_data_rx.dma_request0 = dma_events[1];
	esai->filter_data_tx.peripheral_type = IMX_DMATYPE_ESAI;
	esai->filter_data_rx.peripheral_type = IMX_DMATYPE_ESAI;

	platform_set_drvdata(pdev, esai);

	p = strrchr(np->full_name, '/') + 1;
	strcpy(esai->name, p);
	fsl_esai_dai.name = esai->name;

	ret = snd_soc_register_component(&pdev->dev, &fsl_esai_component,
					 &fsl_esai_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "register DAI failed\n");
		goto failed_register;
	}

	ret = imx_pcm_dma_init(pdev, SND_DMAENGINE_PCM_FLAG_NO_RESIDUE |
				     SND_DMAENGINE_PCM_FLAG_NO_DT |
				     SND_DMAENGINE_PCM_FLAG_COMPAT,
				     IMX_ESAI_DMABUF_SIZE);
	if (ret) {
		dev_err(&pdev->dev, "init pcm dma failed\n");
		goto failed_pcm_init;
	}

	writel(ESAI_ECR_ERST, esai->base + ESAI_ECR);
	writel(ESAI_ECR_ESAIEN, esai->base + ESAI_ECR);
	return 0;

failed_pcm_init:
	snd_soc_unregister_component(&pdev->dev);
failed_register:
failed_get_dma:
	irq_dispose_mapping(esai->irq);
	iounmap(esai->base);
failed_iomap:
failed_get_resource:
	clk_unprepare(esai->clk);
	return ret;
}

static int fsl_esai_remove(struct platform_device *pdev)
{
	struct fsl_esai *esai = platform_get_drvdata(pdev);

	imx_pcm_dma_exit(pdev);
	snd_soc_unregister_component(&pdev->dev);

	irq_dispose_mapping(esai->irq);
	iounmap(esai->base);
	clk_unprepare(esai->clk);
	return 0;
}

static const struct of_device_id fsl_esai_ids[] = {
	{ .compatible = "fsl,imx6q-esai", },
	{}
};

static struct platform_driver fsl_esai_driver = {
	.probe = fsl_esai_probe,
	.remove = fsl_esai_remove,
	.driver = {
		.name = "fsl-esai-dai",
		.owner = THIS_MODULE,
		.of_match_table = fsl_esai_ids,
	},
};

module_platform_driver(fsl_esai_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("i.MX ASoC ESAI driver");
MODULE_ALIAS("platform:fsl-esai-dai");
MODULE_LICENSE("GPL");
