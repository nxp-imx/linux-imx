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
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <mach/dma.h>
#include <mach/clock.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>

#include "imx-esai.h"
#include "imx-pcm.h"

static int imx_esai_txrx_state;
static struct imx_esai imx_esai_priv[3];
static void __iomem *esai_ioaddr;

static int imx_esai_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
				   int clk_id, unsigned int freq, int dir)
{
	u32 ecr, tccr, rccr;

	ecr = __raw_readl(ESAI_ECR);
	tccr = __raw_readl(ESAI_TCCR);
	rccr = __raw_readl(ESAI_RCCR);

	if (dir == SND_SOC_CLOCK_IN) {
		if (cpu_dai->id & IMX_DAI_ESAI_TX)
			tccr &=
			    ~(ESAI_TCCR_THCKD | ESAI_TCCR_TCKD |
			      ESAI_TCCR_TFSD);
		if (cpu_dai->id & IMX_DAI_ESAI_RX)
			rccr &=
			    ~(ESAI_RCCR_RHCKD | ESAI_RCCR_RCKD |
			      ESAI_RCCR_RFSD);
	} else {
			tccr |=
			    ESAI_TCCR_THCKD | ESAI_TCCR_TCKD | ESAI_TCCR_TFSD;
			rccr |=
			    ESAI_RCCR_RHCKD | ESAI_RCCR_RCKD | ESAI_RCCR_RFSD;
		if (clk_id == ESAI_CLK_FSYS) {
			if (cpu_dai->id & IMX_DAI_ESAI_TX)
				ecr &= ~(ESAI_ECR_ETI | ESAI_ECR_ETO);
			if (cpu_dai->id & IMX_DAI_ESAI_RX)
				ecr &= ~(ESAI_ECR_ERI | ESAI_ECR_ERO);
		} else if (clk_id == ESAI_CLK_EXTAL) {
				ecr |= ESAI_ECR_ETI;
				ecr |= ESAI_ECR_ETO;
				ecr |= ESAI_ECR_ERI;
				ecr |= ESAI_ECR_ERO;
		}
	}

	__raw_writel(ecr, ESAI_ECR);
	if (cpu_dai->id & IMX_DAI_ESAI_TX)
		__raw_writel(tccr, ESAI_TCCR);
	if (cpu_dai->id & IMX_DAI_ESAI_RX)
		__raw_writel(rccr, ESAI_RCCR);

	ESAI_DUMP();

	return 0;
}

static int imx_esai_set_dai_clkdiv(struct snd_soc_dai *cpu_dai,
				   int div_id, int div)
{
	u32 tccr, rccr;

	tccr = __raw_readl(ESAI_TCCR);
	rccr = __raw_readl(ESAI_RCCR);

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
	if (cpu_dai->id & IMX_DAI_ESAI_TX)
		__raw_writel(tccr, ESAI_TCCR);
	if (cpu_dai->id & IMX_DAI_ESAI_RX)
		__raw_writel(rccr, ESAI_RCCR);
	return 0;
}

/*
 * ESAI Network Mode or TDM slots configuration.
 */
static int imx_esai_set_dai_tdm_slot(struct snd_soc_dai *dai,
	unsigned int tx_mask, unsigned int rx_mask, int slots, int slot_width)
{
	u32 tccr, rccr;

	if (dai->id & IMX_DAI_ESAI_TX) {
		tccr = __raw_readl(ESAI_TCCR);

		tccr &= ESAI_TCCR_TDC_MASK;
		tccr |= ESAI_TCCR_TDC(slots - 1);

		__raw_writel(tccr, ESAI_TCCR);
		__raw_writel((tx_mask & 0xffff), ESAI_TSMA);
		__raw_writel(((tx_mask >> 16) & 0xffff), ESAI_TSMB);
	}

	if (dai->id & IMX_DAI_ESAI_RX) {
		rccr = __raw_readl(ESAI_RCCR);

		rccr &= ESAI_RCCR_RDC_MASK;
		rccr |= ESAI_RCCR_RDC(slots - 1);

		__raw_writel(rccr, ESAI_RCCR);
		__raw_writel((rx_mask & 0xffff), ESAI_RSMA);
		__raw_writel(((rx_mask >> 16) & 0xffff), ESAI_RSMB);
	}

	ESAI_DUMP();

	return 0;
}

/*
 * ESAI DAI format configuration.
 */
static int imx_esai_set_dai_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	struct imx_esai *esai_mode = (struct imx_esai *)cpu_dai->private_data;
	u32 tcr, tccr, rcr, rccr, saicr;

	tcr = __raw_readl(ESAI_TCR);
	tccr = __raw_readl(ESAI_TCCR);
	rcr = __raw_readl(ESAI_RCR);
	rccr = __raw_readl(ESAI_RCCR);
	saicr = __raw_readl(ESAI_SAICR);

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
	if (esai_mode->sync_mode)
		saicr |= ESAI_SAICR_SYNC;
	else
		saicr &= ~ESAI_SAICR_SYNC;

	tcr &= ESAI_TCR_TMOD_MASK;
	rcr &= ESAI_RCR_RMOD_MASK;
	if (esai_mode->network_mode) {
		tcr |= ESAI_TCR_TMOD_NETWORK;
		rcr |= ESAI_RCR_RMOD_NETWORK;
	} else {
		tcr |= ESAI_TCR_TMOD_NORMAL;
		rcr |= ESAI_RCR_RMOD_NORMAL;
	}

	if (cpu_dai->id & IMX_DAI_ESAI_TX) {
		__raw_writel(tcr, ESAI_TCR);
		__raw_writel(tccr, ESAI_TCCR);
	}
	if (cpu_dai->id & IMX_DAI_ESAI_RX) {
		__raw_writel(rcr, ESAI_RCR);
		__raw_writel(rccr, ESAI_RCCR);
	}

	__raw_writel(saicr, ESAI_SAICR);

	ESAI_DUMP();
	return 0;
}

static struct clk *esai_clk;

static int fifo_err_counter;

static irqreturn_t esai_irq(int irq, void *dev_id)
{
	if (fifo_err_counter++ % 1000 == 0)
		printk(KERN_ERR
		       "esai_irq SAISR %x fifo_errs=%d\n",
		       __raw_readl(ESAI_SAISR), fifo_err_counter);
	return IRQ_HANDLED;
}

static int imx_esai_startup(struct snd_pcm_substream *substream,
			    struct snd_soc_dai *cpu_dai)
{
	if (cpu_dai->playback.active && (cpu_dai->id & IMX_DAI_ESAI_TX))
		return 0;
	if (cpu_dai->capture.active && (cpu_dai->id & IMX_DAI_ESAI_RX))
		return 0;

	if (!(imx_esai_txrx_state & IMX_DAI_ESAI_TXRX)) {
		if (request_irq(MXC_INT_ESAI, esai_irq, 0, "esai", NULL)) {
			pr_err("%s: failure requesting esai irq\n", __func__);
			return -EBUSY;
		}
		clk_enable(esai_clk);
		__raw_writel(ESAI_ECR_ERST, ESAI_ECR);
		__raw_writel(ESAI_ECR_ESAIEN, ESAI_ECR);

		__raw_writel(ESAI_GPIO_ESAI, ESAI_PRRC);
		__raw_writel(ESAI_GPIO_ESAI, ESAI_PCRC);
	}

	if (cpu_dai->id & IMX_DAI_ESAI_TX) {
		imx_esai_txrx_state |= IMX_DAI_ESAI_TX;
		__raw_writel(ESAI_TCR_TPR, ESAI_TCR);
	}
	if (cpu_dai->id & IMX_DAI_ESAI_RX) {
		imx_esai_txrx_state |= IMX_DAI_ESAI_RX;
		__raw_writel(ESAI_RCR_RPR, ESAI_RCR);
	}

	ESAI_DUMP();
	return 0;
}

/*
 * This function is called to initialize the TX port before enable
 * the tx port.
 */
static int imx_esai_hw_tx_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	u32 tcr, tfcr;
	unsigned int channels;

	tcr = __raw_readl(ESAI_TCR);
	tfcr = __raw_readl(ESAI_TFCR);

	tfcr |= ESAI_TFCR_TFR;
	__raw_writel(tfcr, ESAI_TFCR);
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
	}

	channels = params_channels(params);
	tfcr &= ESAI_TFCR_TE_MASK;
	tfcr |= ESAI_TFCR_TE(channels);

	tfcr |= ESAI_TFCR_TFWM(64);

	/* Left aligned, Zero padding */
	tcr |= ESAI_TCR_PADC;

	__raw_writel(tcr, ESAI_TCR);
	__raw_writel(tfcr, ESAI_TFCR);

	ESAI_DUMP();
	return 0;
}

/*
 * This function is called to initialize the RX port before enable
 * the rx port.
 */
static int imx_esai_hw_rx_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	u32 rcr, rfcr;
	unsigned int channels;

	rcr = __raw_readl(ESAI_RCR);
	rfcr = __raw_readl(ESAI_RFCR);

	rfcr |= ESAI_RFCR_RFR;
	__raw_writel(rfcr, ESAI_RFCR);
	rfcr &= ~ESAI_RFCR_RFR;

	rfcr &= ESAI_RFCR_RWA_MASK;
	rcr &= ESAI_RCR_RSWS_MASK;
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		rfcr |= ESAI_WORD_LEN_16;
		rcr |= ESAI_RCR_RSHFD_MSB | ESAI_RCR_RSWS_STL32_WDL16;
		break;
	}

	channels = params_channels(params);
	rfcr &= ESAI_RFCR_RE_MASK;
	rfcr |= ESAI_RFCR_RE(channels);

	rfcr |= ESAI_RFCR_RFWM(64);

	__raw_writel(rcr, ESAI_RCR);
	__raw_writel(rfcr, ESAI_RFCR);
	return 0;
}

/*
 * This function is called to initialize the TX or RX port,
 */
static int imx_esai_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params,
			      struct snd_soc_dai *dai)
{
	/* Tx/Rx config */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (__raw_readl(ESAI_TCR) & ESAI_TCR_TE0)
			return 0;
		return imx_esai_hw_tx_params(substream, params, dai);
	} else {
		if (__raw_readl(ESAI_RCR) & ESAI_RCR_RE1)
			return 0;
		return imx_esai_hw_rx_params(substream, params, dai);
	}
}

static int imx_esai_trigger(struct snd_pcm_substream *substream, int cmd,
			    struct snd_soc_dai *dai)
{
	u32 reg, tfcr = 0, rfcr = 0;
	u32 temp;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		tfcr = __raw_readl(ESAI_TFCR);
		reg = __raw_readl(ESAI_TCR);
	} else {
		rfcr = __raw_readl(ESAI_RFCR);
		reg = __raw_readl(ESAI_RCR);
	}
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			tfcr |= ESAI_TFCR_TFEN;
			__raw_writel(tfcr, ESAI_TFCR);
			reg &= ~ESAI_TCR_TPR;
			reg |= ESAI_TCR_TE(substream->runtime->channels);
			__raw_writel(reg, ESAI_TCR);
		} else {
			temp = __raw_readl(ESAI_TCR);
			temp &= ~ESAI_TCR_TPR;
			__raw_writel(temp, ESAI_TCR);
			rfcr |= ESAI_RFCR_RFEN;
			__raw_writel(rfcr, ESAI_RFCR);
			reg &= ~ESAI_RCR_RPR;
			reg |= ESAI_RCR_RE(substream->runtime->channels);
			__raw_writel(reg, ESAI_RCR);
		}
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			reg &= ~ESAI_TCR_TE(substream->runtime->channels);
			__raw_writel(reg, ESAI_TCR);
			reg |= ESAI_TCR_TPR;
			__raw_writel(reg, ESAI_TCR);
			tfcr |= ESAI_TFCR_TFR;
			tfcr &= ~ESAI_TFCR_TFEN;
			__raw_writel(tfcr, ESAI_TFCR);
			tfcr &= ~ESAI_TFCR_TFR;
			__raw_writel(tfcr, ESAI_TFCR);
		} else {
			reg &= ~ESAI_RCR_RE(substream->runtime->channels);
			__raw_writel(reg, ESAI_RCR);
			reg |= ESAI_RCR_RPR;
			__raw_writel(reg, ESAI_RCR);
			rfcr |= ESAI_RFCR_RFR;
			rfcr &= ~ESAI_RFCR_RFEN;
			__raw_writel(rfcr, ESAI_RFCR);
			rfcr &= ~ESAI_RFCR_RFR;
			__raw_writel(rfcr, ESAI_RFCR);
		}
		break;
	default:
		return -EINVAL;
	}
	ESAI_DUMP();
	return 0;
}

static void imx_esai_shutdown(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	if (dai->id & IMX_DAI_ESAI_TX)
		imx_esai_txrx_state &= ~IMX_DAI_ESAI_TX;
	if (dai->id & IMX_DAI_ESAI_RX)
		imx_esai_txrx_state &= ~IMX_DAI_ESAI_RX;

	/* shutdown ESAI if neither Tx or Rx is active */
	if (!(imx_esai_txrx_state & IMX_DAI_ESAI_TXRX)) {
		free_irq(MXC_INT_ESAI, NULL);
		clk_disable(esai_clk);
	}
}

#ifdef CONFIG_PM
static int imx_esai_suspend(struct snd_soc_dai *dai)
{
	if (!dai->active)
		return 0;

	/*do we need to disable any clocks */
	return 0;
}

static int imx_esai_resume(struct snd_soc_dai *dai)
{
	if (!dai->active)
		return 0;

	/* do we need to enable any clocks */
	return 0;
}

#else
#define imx_esai_suspend	NULL
#define imx_esai_resume	NULL
#endif

static int imx_esai_probe(struct platform_device *pdev, struct snd_soc_dai *dai)
{
	imx_esai_txrx_state = 0;

	esai_clk = clk_get(NULL, "esai_clk");
	if (!esai_clk)
		printk(KERN_WARNING "Can't get the clock esai_clk \n");
	return 0;
}

static void imx_esai_remove(struct platform_device *pdev,
			    struct snd_soc_dai *dai)
{

	clk_put(esai_clk);
}

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

struct snd_soc_dai imx_esai_dai[] = {
	{
	 .name = "imx-esai-tx",
	 .id = IMX_DAI_ESAI_TX,
	 .probe = imx_esai_probe,
	 .remove = imx_esai_remove,
	 .suspend = imx_esai_suspend,
	 .resume = imx_esai_resume,
	 .playback = {
		      .channels_min = 1,
		      .channels_max = 6,
		      .rates = IMX_ESAI_RATES,
		      .formats = IMX_ESAI_FORMATS,
		      },
	 .capture = {
		     .channels_min = 1,
		     .channels_max = 4,
		     .rates = IMX_ESAI_RATES,
		     .formats = IMX_ESAI_FORMATS,
		     },
	 .ops = &imx_esai_dai_ops,
	 .private_data = &imx_esai_priv[0],
	 },
	{
	 .name = "imx-esai-rx",
	 .id = IMX_DAI_ESAI_RX,
	 .probe = imx_esai_probe,
	 .remove = imx_esai_remove,
	 .suspend = imx_esai_suspend,
	 .resume = imx_esai_resume,
	 .playback = {
		      .channels_min = 1,
		      .channels_max = 6,
		      .rates = IMX_ESAI_RATES,
		      .formats = IMX_ESAI_FORMATS,
		      },
	 .capture = {
		     .channels_min = 1,
		     .channels_max = 4,
		     .rates = IMX_ESAI_RATES,
		     .formats = IMX_ESAI_FORMATS,
		     },
	 .ops = &imx_esai_dai_ops,
	 .private_data = &imx_esai_priv[1],
	 },
	{
	 .name = "imx-esai-txrx",
	 .id = IMX_DAI_ESAI_TXRX,
	 .probe = imx_esai_probe,
	 .remove = imx_esai_remove,
	 .suspend = imx_esai_suspend,
	 .resume = imx_esai_resume,
	 .playback = {
		      .channels_min = 1,
		      .channels_max = 6,
		      .rates = IMX_ESAI_RATES,
		      .formats = IMX_ESAI_FORMATS,
		      },
	 .capture = {
		     .channels_min = 1,
		     .channels_max = 4,
		     .rates = IMX_ESAI_RATES,
		     .formats = IMX_ESAI_FORMATS,
		     },
	 .ops = &imx_esai_dai_ops,
	 .private_data = &imx_esai_priv[2],
	 },

};

EXPORT_SYMBOL_GPL(imx_esai_dai);

static int imx_esai_dev_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct mxc_esai_platform_data *plat_data = pdev->dev.platform_data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	esai_ioaddr = ioremap(res->start, res->end - res->start + 1);

	if (plat_data->activate_esai_ports)
		plat_data->activate_esai_ports();

	snd_soc_register_dais(imx_esai_dai, ARRAY_SIZE(imx_esai_dai));
	return 0;
}

static int __devexit imx_esai_dev_remove(struct platform_device *pdev)
{

	struct mxc_esai_platform_data *plat_data = pdev->dev.platform_data;
	iounmap(esai_ioaddr);
	if (plat_data->deactivate_esai_ports)
		plat_data->deactivate_esai_ports();

	snd_soc_unregister_dais(imx_esai_dai, ARRAY_SIZE(imx_esai_dai));
	return 0;
}


static struct platform_driver imx_esai_driver = {
	.probe = imx_esai_dev_probe,
	.remove = __devexit_p(imx_esai_dev_remove),
	.driver = {
		   .name = "mxc_esai",
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
