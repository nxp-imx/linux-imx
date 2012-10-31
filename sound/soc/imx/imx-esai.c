/*
 * Copyright 2008-2012 Freescale Semiconductor, Inc. All Rights Reserved.
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
  * @file       imx-esai.c
  * @brief      this file implements the esai interface
  *             in according to ASoC architeture
  */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <mach/dma.h>
#include <mach/clock.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/esai.h>

#include "imx-esai.h"
#include "imx-pcm.h"

static struct imx_esai *local_esai;

static int imx_esai_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
				   int clk_id, unsigned int freq, int dir)
{
	struct imx_esai *esai = snd_soc_dai_get_drvdata(cpu_dai);
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

static int imx_esai_set_dai_clkdiv(struct snd_soc_dai *cpu_dai,
				   int div_id, int div)
{
	struct imx_esai *esai = snd_soc_dai_get_drvdata(cpu_dai);
	u32 tccr, rccr;

	tccr = readl(esai->base + ESAI_TCCR);
	rccr = readl(esai->base + ESAI_RCCR);

	switch (div_id) {
	case ESAI_TX_DIV_PSR:
		tccr &= ESAI_TCCR_TPSR_MASK;
		if (div)
			tccr |= ESAI_TCCR_TPSR_BYPASS;
		else
			tccr &= ~ESAI_TCCR_TPSR_DIV8;
		break;
	case ESAI_TX_DIV_PM:
		tccr &= ESAI_TCCR_TPM_MASK;
		tccr |= ESAI_TCCR_TPM(div);
		break;
	case ESAI_TX_DIV_FP:
		tccr &= ESAI_TCCR_TFP_MASK;
		tccr |= ESAI_TCCR_TFP(div);
		break;
	case ESAI_RX_DIV_PSR:
		rccr &= ESAI_RCCR_RPSR_MASK;
		if (div)
			rccr |= ESAI_RCCR_RPSR_BYPASS;
		else
			rccr &= ~ESAI_RCCR_RPSR_DIV8;
		break;
	case ESAI_RX_DIV_PM:
		rccr &= ESAI_RCCR_RPM_MASK;
		rccr |= ESAI_RCCR_RPM(div);
		break;
	case ESAI_RX_DIV_FP:
		rccr &= ESAI_RCCR_RFP_MASK;
		rccr |= ESAI_RCCR_RFP(div);
		break;
		return -EINVAL;
	}
	writel(tccr, esai->base + ESAI_TCCR);
	writel(rccr, esai->base + ESAI_RCCR);
	return 0;
}

/*
 * ESAI Network Mode or TDM slots configuration.
 */
static int imx_esai_set_dai_tdm_slot(struct snd_soc_dai *cpu_dai,
	unsigned int tx_mask, unsigned int rx_mask, int slots, int slot_width)
{
	struct imx_esai *esai = snd_soc_dai_get_drvdata(cpu_dai);
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
static int imx_esai_set_dai_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	struct imx_esai *esai = snd_soc_dai_get_drvdata(cpu_dai);
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

static int imx_esai_startup(struct snd_pcm_substream *substream,
			    struct snd_soc_dai *cpu_dai)
{
	struct imx_esai *esai = snd_soc_dai_get_drvdata(cpu_dai);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK &&
			(local_esai->imx_esai_txrx_state & IMX_DAI_ESAI_TX)) {
		pr_err("error: too much esai playback!\n");
		return -EINVAL;
	}

	if (!(local_esai->imx_esai_txrx_state & IMX_DAI_ESAI_TXRX)) {
		clk_enable(esai->clk);

		writel(ESAI_GPIO_ESAI, esai->base + ESAI_PRRC);
		writel(ESAI_GPIO_ESAI, esai->base + ESAI_PCRC);
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		local_esai->imx_esai_txrx_state |= IMX_DAI_ESAI_TX;
	else
		local_esai->imx_esai_txrx_state |= IMX_DAI_ESAI_RX;

	ESAI_DUMP();
	return 0;
}

/*
 * This function is called to initialize the TX port before enable
 * the tx port.
 */
static int imx_esai_hw_tx_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *cpu_dai)
{
	struct imx_pcm_runtime_data *iprtd = substream->runtime->private_data;

	struct imx_esai *esai = snd_soc_dai_get_drvdata(cpu_dai);
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

	if (iprtd->asrc_enable) {
		switch (iprtd->p2p->p2p_width) {
		case ASRC_WIDTH_16_BIT:
			tfcr |= ESAI_WORD_LEN_16;
			tcr |= ESAI_TCR_TSHFD_MSB | ESAI_TCR_TSWS_STL32_WDL16;
			break;
		case ASRC_WIDTH_24_BIT:
			tfcr |= ESAI_WORD_LEN_24;
			tcr |= ESAI_TCR_TSHFD_MSB | ESAI_TCR_TSWS_STL32_WDL24;
			break;
		default:
			return -EINVAL;

		}
		tfcr |= ESAI_WORD_LEN_24;
		tcr |= ESAI_TCR_TSHFD_MSB | ESAI_TCR_TSWS_STL32_WDL24;
	} else {
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
	}

	channels = params_channels(params);
	tfcr &= ESAI_TFCR_TE_MASK;
	tfcr |= ESAI_TFCR_TE(channels);

	tfcr |= ESAI_TFCR_TFWM(64);

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
static int imx_esai_hw_rx_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *cpu_dai)
{
	struct imx_esai *esai = snd_soc_dai_get_drvdata(cpu_dai);
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
	}

	channels = params_channels(params);
	rfcr &= ESAI_RFCR_RE_MASK;
	rfcr |= ESAI_RFCR_RE(channels);

	rfcr |= ESAI_RFCR_RFWM(64);

	writel(rcr, esai->base + ESAI_RCR);
	writel(rfcr, esai->base + ESAI_RFCR);

	ESAI_DUMP();
	return 0;
}

/*
 * This function is called to initialize the TX or RX port,
 */
static int imx_esai_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params,
			      struct snd_soc_dai *cpu_dai)
{
	struct imx_esai *esai = snd_soc_dai_get_drvdata(cpu_dai);
	struct imx_pcm_dma_params *dma_data;

	/* Tx/Rx config */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (readl(esai->base + ESAI_TCR) & ESAI_TCR_TE0)
			return 0;

		dma_data = &esai->dma_params_tx;
		snd_soc_dai_set_dma_data(cpu_dai, substream, dma_data);
		return imx_esai_hw_tx_params(substream, params, cpu_dai);
	} else {
		if (readl(esai->base + ESAI_RCR) & ESAI_RCR_RE1)
			return 0;

		dma_data = &esai->dma_params_rx;
		snd_soc_dai_set_dma_data(cpu_dai, substream, dma_data);
		return imx_esai_hw_rx_params(substream, params, cpu_dai);
	}
}

static void imx_esai_shutdown(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *cpu_dai)
{
	struct imx_esai *esai = snd_soc_dai_get_drvdata(cpu_dai);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		local_esai->imx_esai_txrx_state &= ~IMX_DAI_ESAI_TX;
	else
		local_esai->imx_esai_txrx_state &= ~IMX_DAI_ESAI_RX;

	if (!(local_esai->imx_esai_txrx_state & IMX_DAI_ESAI_TXRX))
		/* close easi clock */
		clk_disable(esai->clk);
}

static int imx_esai_trigger(struct snd_pcm_substream *substream, int cmd,
			    struct snd_soc_dai *cpu_dai)
{
	struct imx_esai *esai = snd_soc_dai_get_drvdata(cpu_dai);
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

#ifdef CONFIG_PM
static int imx_esai_suspend(struct snd_soc_dai *cpu_dai)
{
	return 0;
}

static int imx_esai_resume(struct snd_soc_dai *cpu_dai)
{
	return 0;
}

#else
#define imx_esai_suspend	NULL
#define imx_esai_resume	NULL
#endif


#define IMX_ESAI_RATES  SNDRV_PCM_RATE_8000_192000

#define IMX_ESAI_FORMATS \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
	SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_ops imx_esai_dai_ops = {
	.startup = imx_esai_startup,
	.shutdown = imx_esai_shutdown,
	.trigger = imx_esai_trigger,
	.hw_params = imx_esai_hw_params,
	.set_sysclk = imx_esai_set_dai_sysclk,
	.set_clkdiv = imx_esai_set_dai_clkdiv,
	.set_fmt = imx_esai_set_dai_fmt,
	.set_tdm_slot = imx_esai_set_dai_tdm_slot,
};

static int imx_esai_dai_probe(struct snd_soc_dai *dai)
{
	struct imx_esai *esai = dev_get_drvdata(dai->dev);
	local_esai->imx_esai_txrx_state = 0;
	snd_soc_dai_set_drvdata(dai, esai);
	return 0;
}

static struct snd_soc_dai_driver imx_esai_dai = {
	 .name = "imx-esai.0",
	 .probe = imx_esai_dai_probe,
	 .suspend = imx_esai_suspend,
	 .resume = imx_esai_resume,
	 .playback = {
		      .channels_min = 1,
		      .channels_max = 12,
		      .rates = IMX_ESAI_RATES,
		      .formats = IMX_ESAI_FORMATS,
		      },
	 .capture = {
		     .channels_min = 1,
		     .channels_max = 8,
		     .rates = IMX_ESAI_RATES,
		     .formats = IMX_ESAI_FORMATS,
		     },
	 .ops = &imx_esai_dai_ops,
};

static int imx_esai_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct imx_esai *esai;
	struct imx_esai_platform_data *pdata = pdev->dev.platform_data;
	int ret = 0;

	esai = kzalloc(sizeof(*esai), GFP_KERNEL);
	if (!esai)
		return -ENOMEM;
	local_esai = esai;
	dev_set_drvdata(&pdev->dev, esai);

	if (pdata)
		esai->flags = pdata->flags;

	esai->irq = platform_get_irq(pdev, 0);

	esai->clk = clk_get(&pdev->dev, "esai_clk");
	if (IS_ERR(esai->clk)) {
		ret = PTR_ERR(esai->clk);
		dev_err(&pdev->dev, "Cannot get the clock: %d\n",
			ret);
		goto failed_clk;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -ENODEV;
		goto failed_get_resource;
	}

	if (!request_mem_region(res->start, resource_size(res), DRV_NAME)) {
		dev_err(&pdev->dev, "request_mem_region failed\n");
		ret = -EBUSY;
		goto failed_get_resource;
	}

	esai->base = ioremap(res->start, resource_size(res));
	if (!esai->base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENODEV;
		goto failed_ioremap;
	}

	esai->dma_params_rx.dma_addr = res->start + ESAI_ERDR;
	esai->dma_params_tx.dma_addr = res->start + ESAI_ETDR;

	esai->dma_params_tx.peripheral_type = IMX_DMATYPE_ESAI;
	esai->dma_params_rx.peripheral_type = IMX_DMATYPE_ESAI;

	res = platform_get_resource_byname(pdev, IORESOURCE_DMA, "tx");
	if (res)
		esai->dma_params_tx.dma = res->start;

	res = platform_get_resource_byname(pdev, IORESOURCE_DMA, "rx");
	if (res)
		esai->dma_params_rx.dma = res->start;

	platform_set_drvdata(pdev, esai);
	ret = snd_soc_register_dai(&pdev->dev, &imx_esai_dai);
	if (ret) {
		dev_err(&pdev->dev, "register DAI failed\n");
		goto failed_register;
	}

	esai->soc_platform_pdev_fiq = \
		platform_device_alloc("imx-fiq-pcm-audio", pdev->id);
	if (!esai->soc_platform_pdev_fiq) {
		ret = -ENOMEM;
		goto failed_pdev_fiq_alloc;
	}

	platform_set_drvdata(esai->soc_platform_pdev_fiq, esai);
	ret = platform_device_add(esai->soc_platform_pdev_fiq);
	if (ret) {
		dev_err(&pdev->dev, "failed to add platform device\n");
		goto failed_pdev_fiq_add;
	}

	esai->soc_platform_pdev = \
		platform_device_alloc("imx-pcm-audio", pdev->id);
	if (!esai->soc_platform_pdev) {
		ret = -ENOMEM;
		goto failed_pdev_alloc;
	}

	platform_set_drvdata(esai->soc_platform_pdev, esai);
	ret = platform_device_add(esai->soc_platform_pdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to add platform device\n");
		goto failed_pdev_add;
	}

	writel(ESAI_ECR_ERST, esai->base + ESAI_ECR);
	writel(ESAI_ECR_ESAIEN, esai->base + ESAI_ECR);

	return 0;

failed_pdev_add:
	platform_device_put(esai->soc_platform_pdev);
failed_pdev_alloc:
	platform_device_del(esai->soc_platform_pdev_fiq);
failed_pdev_fiq_add:
	platform_device_put(esai->soc_platform_pdev_fiq);
failed_pdev_fiq_alloc:
	snd_soc_unregister_dai(&pdev->dev);
failed_register:
	iounmap(esai->base);
failed_ioremap:
	release_mem_region(res->start, resource_size(res));
failed_get_resource:
	clk_disable(esai->clk);
	clk_put(esai->clk);
failed_clk:
	kfree(esai);

	return ret;
}
static int __devexit imx_esai_remove(struct platform_device *pdev)
{
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	struct imx_esai *esai = platform_get_drvdata(pdev);

	platform_device_unregister(esai->soc_platform_pdev);
	platform_device_unregister(esai->soc_platform_pdev_fiq);

	snd_soc_unregister_dai(&pdev->dev);

	iounmap(esai->base);
	release_mem_region(res->start, resource_size(res));
	clk_put(esai->clk);
	kfree(esai);

	return 0;
}


static struct platform_driver imx_esai_driver = {
	.probe = imx_esai_probe,
	.remove = __devexit_p(imx_esai_remove),
	.driver = {
		   .name = "imx-esai",
		   },
};

static int __init imx_esai_init(void)
{
	return platform_driver_register(&imx_esai_driver);
}

static void __exit imx_esai_exit(void)
{
	platform_driver_unregister(&imx_esai_driver);
}

module_init(imx_esai_init);
module_exit(imx_esai_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("i.MX ASoC ESAI driver");
MODULE_LICENSE("GPL");
