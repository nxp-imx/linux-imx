/*
 * imx-ssi.c  --  SSI driver for Freescale IMX
 *
 * Copyright 2006 Wolfson Microelectronics PLC.
 * Author: Liam Girdwood
 *         liam.girdwood@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 * Copyright (C) 2006-2010 Freescale Semiconductor, Inc.
 * Based on mxc-alsa-mc13783
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    29th Aug 2006   Initial version.
 *
 * TODO:
 *   Need to rework SSI register defs when new defs go into mainline.
 *   Add support for TDM and FIFO 1.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
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

#include "imx-ssi.h"
#include "imx-pcm.h"

#define SSI_STX0   (0x00)
#define SSI_STX1   (0x04)
#define SSI_SRX0   (0x08)
#define SSI_SRX1   (0x0c)
#define SSI_SCR    (0x10)
#define SSI_SISR   (0x14)
#define SSI_SIER   (0x18)
#define SSI_STCR   (0x1c)
#define SSI_SRCR   (0x20)
#define SSI_STCCR  (0x24)
#define SSI_SRCCR  (0x28)
#define SSI_SFCSR  (0x2c)
#define SSI_STR    (0x30)
#define SSI_SOR    (0x34)
#define SSI_SACNT  (0x38)
#define SSI_SACADD (0x3c)
#define SSI_SACDAT (0x40)
#define SSI_SATAG  (0x44)
#define SSI_STMSK  (0x48)
#define SSI_SRMSK  (0x4c)
#define SSI_SACCST (0x50)
#define SSI_SACCEN (0x54)
#define SSI_SACCDIS (0x58)

/* debug */
#define IMX_SSI_DEBUG 0
#if IMX_SSI_DEBUG
#define dbg(format, arg...) printk(format, ## arg)
#else
#define dbg(format, arg...)
#endif

#define IMX_SSI_DUMP 0
#if IMX_SSI_DUMP
#define SSI_DUMP() \
	do { \
		printk(KERN_INFO "dump @ %s\n", __func__); \
		printk(KERN_INFO "scr %x\t, %x\n", \
		       __raw_readl(ioaddr + SSI_SCR), \
		       __raw_readl(ioaddr + SSI_SCR));	\
		printk(KERN_INFO "sisr %x\t, %x\n", \
		       __raw_readl(ioaddr + SSI_SISR), \
		       __raw_readl(ioaddr + SSI_SISR));	\
		printk(KERN_INFO "stcr %x\t, %x\n", \
		       __raw_readl(ioaddr + SSI_STCR), \
		       __raw_readl(ioaddr + SSI_STCR)); \
		printk(KERN_INFO "srcr %x\t, %x\n", \
		       __raw_readl(ioaddr + SSI_SRCR), \
		       __raw_readl(ioaddr + SSI_SRCR)); \
		printk(KERN_INFO "stccr %x\t, %x\n", \
		       __raw_readl(ioaddr + SSI_STCCR), \
		       __raw_readl(ioaddr + SSI_STCCR)); \
		printk(KERN_INFO "srccr %x\t, %x\n", \
		       __raw_readl(ioaddr + SSI_SRCCR), \
		       __raw_readl(ioaddr + SSI_SRCCR)); \
		printk(KERN_INFO "sfcsr %x\t, %x\n", \
		       __raw_readl(ioaddr + SSI_SFCSR), \
		       __raw_readl(ioaddr + SSI_SFCSR)); \
		printk(KERN_INFO "stmsk %x\t, %x\n", \
		       __raw_readl(ioaddr + SSI_STMSK), \
		       __raw_readl(ioaddr + SSI_STMSK)); \
		printk(KERN_INFO "srmsk %x\t, %x\n", \
		       __raw_readl(ioaddr + SSI_SRMSK), \
		       __raw_readl(ioaddr + SSI_SRMSK)); \
		printk(KERN_INFO "sier %x\t, %x\n", \
		       __raw_readl(ioaddr + SSI_SIER), \
		       __raw_readl(ioaddr + SSI_SIER)); \
	} while (0);
#else
#define SSI_DUMP()
#endif

/*
 * SSI system clock configuration.
 * Should only be called when port is inactive (i.e. SSIEN = 0).
 */
static int imx_ssi_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct imx_ssi *priv = (struct imx_ssi *)cpu_dai->private_data;
	void __iomem *ioaddr = priv->ioaddr;
	u32 scr;

	scr = __raw_readl(ioaddr + SSI_SCR);

	if (scr & SSI_SCR_SSIEN)
		return 0;

	switch (clk_id) {
	case IMX_SSP_SYS_CLK:
		if (dir == SND_SOC_CLOCK_OUT)
			scr |= SSI_SCR_SYS_CLK_EN;
		else
			scr &= ~SSI_SCR_SYS_CLK_EN;
		break;
	default:
		return -EINVAL;
	}

	__raw_writel(scr, ioaddr + SSI_SCR);

	return 0;
}

/*
 * SSI Clock dividers
 * Should only be called when port is inactive (i.e. SSIEN = 0).
 */
static int imx_ssi_set_dai_clkdiv(struct snd_soc_dai *cpu_dai,
				  int div_id, int div)
{
	struct imx_ssi *priv = (struct imx_ssi *)cpu_dai->private_data;
	void __iomem *ioaddr = priv->ioaddr;
	u32 stccr, srccr;

	if (__raw_readl(ioaddr + SSI_SCR) & SSI_SCR_SSIEN)
		return 0;

	srccr = __raw_readl(ioaddr + SSI_SRCCR);
	stccr = __raw_readl(ioaddr + SSI_STCCR);

	switch (div_id) {
	case IMX_SSI_TX_DIV_2:
		stccr &= ~SSI_STCCR_DIV2;
		stccr |= div;
		break;
	case IMX_SSI_TX_DIV_PSR:
		stccr &= ~SSI_STCCR_PSR;
		stccr |= div;
		break;
	case IMX_SSI_TX_DIV_PM:
		stccr &= ~0xff;
		stccr |= SSI_STCCR_PM(div);
		break;
	case IMX_SSI_RX_DIV_2:
		stccr &= ~SSI_STCCR_DIV2;
		stccr |= div;
		break;
	case IMX_SSI_RX_DIV_PSR:
		stccr &= ~SSI_STCCR_PSR;
		stccr |= div;
		break;
	case IMX_SSI_RX_DIV_PM:
		stccr &= ~0xff;
		stccr |= SSI_STCCR_PM(div);
		break;
	default:
		return -EINVAL;
	}

	__raw_writel(stccr, ioaddr + SSI_STCCR);
	__raw_writel(srccr, ioaddr + SSI_SRCCR);

	return 0;
}

/*
 * SSI Network Mode or TDM slots configuration.
 * Should only be called when port is inactive (i.e. SSIEN = 0).
 */
static int imx_ssi_set_dai_tdm_slot(struct snd_soc_dai *cpu_dai,
				unsigned int tx_mask,
				unsigned int rx_mask,
				int slots, int slot_width)
{
	struct imx_ssi *priv = (struct imx_ssi *)cpu_dai->private_data;
	void __iomem *ioaddr = priv->ioaddr;
	u32 stmsk, srmsk, stccr;

	if (__raw_readl(ioaddr + SSI_SCR) & SSI_SCR_SSIEN)
		return 0;
	stccr = __raw_readl(ioaddr + SSI_STCCR);

	stmsk = tx_mask;
	srmsk = rx_mask;
	stccr &= ~SSI_STCCR_DC_MASK;
	stccr |= SSI_STCCR_DC(slots - 1);

	__raw_writel(stmsk, ioaddr + SSI_STMSK);
	__raw_writel(srmsk, ioaddr + SSI_SRMSK);
	__raw_writel(stccr, ioaddr + SSI_STCCR);
	__raw_writel(stccr, ioaddr + SSI_SRCCR);

	return 0;
}

/*
 * SSI DAI format configuration.
 * Should only be called when port is inactive (i.e. SSIEN = 0).
 * Note: We don't use the I2S modes but instead manually configure the
 * SSI for I2S.
 */
static int imx_ssi_set_dai_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	struct imx_ssi *priv = (struct imx_ssi *)cpu_dai->private_data;
	void __iomem *ioaddr = priv->ioaddr;
	u32 stcr = 0, srcr = 0, scr;

	scr = __raw_readl(ioaddr + SSI_SCR) & ~(SSI_SCR_SYN | SSI_SCR_NET);

	if (scr & SSI_SCR_SSIEN)
		return 0;

	/* DAI mode */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		/* data on rising edge of bclk, frame low 1clk before data */
		stcr |= SSI_STCR_TFSI | SSI_STCR_TEFS | SSI_STCR_TXBIT0;
		srcr |= SSI_SRCR_RFSI | SSI_SRCR_REFS | SSI_SRCR_RXBIT0;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		/* data on rising edge of bclk, frame high with data */
		stcr |= SSI_STCR_TXBIT0;
		srcr |= SSI_SRCR_RXBIT0;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		/* data on rising edge of bclk, frame high with data */
		stcr |= SSI_STCR_TFSL;
		srcr |= SSI_SRCR_RFSL;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		/* data on rising edge of bclk, frame high 1clk before data */
		stcr |= SSI_STCR_TFSL | SSI_STCR_TEFS;
		srcr |= SSI_SRCR_RFSL | SSI_SRCR_REFS;
		break;
	}

	/* DAI clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_IB_IF:
		stcr &= ~(SSI_STCR_TSCKP | SSI_STCR_TFSI);
		srcr &= ~(SSI_SRCR_RSCKP | SSI_SRCR_RFSI);
		break;
	case SND_SOC_DAIFMT_IB_NF:
		stcr |= SSI_STCR_TFSI;
		stcr &= ~SSI_STCR_TSCKP;
		srcr |= SSI_SRCR_RFSI;
		srcr &= ~SSI_SRCR_RSCKP;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		stcr &= ~SSI_STCR_TFSI;
		stcr |= SSI_STCR_TSCKP;
		srcr &= ~SSI_SRCR_RFSI;
		srcr |= SSI_SRCR_RSCKP;
		break;
	case SND_SOC_DAIFMT_NB_NF:
		stcr |= SSI_STCR_TFSI | SSI_STCR_TSCKP;
		srcr |= SSI_SRCR_RFSI | SSI_SRCR_RSCKP;
		break;
	}

	/* DAI clock master masks */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		stcr |= SSI_STCR_TFDIR | SSI_STCR_TXDIR;
		if (((fmt & SND_SOC_DAIFMT_FORMAT_MASK) == SND_SOC_DAIFMT_I2S)
		    && priv->network_mode) {
			scr &= ~SSI_SCR_I2S_MODE_MASK;
			scr |= SSI_SCR_I2S_MODE_MSTR;
		}
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		stcr |= SSI_STCR_TFDIR;
		srcr |= SSI_SRCR_RFDIR;
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		stcr |= SSI_STCR_TXDIR;
		srcr |= SSI_SRCR_RXDIR;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		if (((fmt & SND_SOC_DAIFMT_FORMAT_MASK) == SND_SOC_DAIFMT_I2S)
		    && priv->network_mode) {
			scr &= ~SSI_SCR_I2S_MODE_MASK;
			scr |= SSI_SCR_I2S_MODE_SLAVE;
		}
		break;
	}

	/* sync */
	if (priv->sync_mode)
		scr |= SSI_SCR_SYN;

	/* tdm - only for stereo atm */
	if (priv->network_mode)
		scr |= SSI_SCR_NET;
#ifdef CONFIG_MXC_SSI_DUAL_FIFO
	if (cpu_is_mx51() || cpu_is_mx53()) {
		stcr |= SSI_STCR_TFEN1;
		srcr |= SSI_SRCR_RFEN1;
		scr |= SSI_SCR_TCH_EN;
	}
#endif
	__raw_writel(stcr, ioaddr + SSI_STCR);
	__raw_writel(srcr, ioaddr + SSI_SRCR);
	__raw_writel(scr, ioaddr + SSI_SCR);

	SSI_DUMP();
	return 0;
}

static int imx_ssi_startup(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *cpu_dai)
{
	struct imx_ssi *priv = (struct imx_ssi *)cpu_dai->private_data;
	void __iomem *ioaddr = priv->ioaddr;

	/* we cant really change any SSI values after SSI is enabled
	 * need to fix in software for max flexibility - lrg */
	if (cpu_dai->playback.active || cpu_dai->capture.active)
		return 0;

	/* reset the SSI port - Sect 45.4.4 */
	if (clk_get_usecount(priv->ssi_clk) != 0) {
		clk_enable(priv->ssi_clk);
		return 0;
	}

	__raw_writel(0, ioaddr + SSI_SCR);
	clk_enable(priv->ssi_clk);

	/* BIG FAT WARNING
	 * SDMA FIFO watermark must == SSI FIFO watermark for best results.
	 */
	__raw_writel((SSI_SFCSR_RFWM1(SSI_RXFIFO_WATERMARK) |
		      SSI_SFCSR_RFWM0(SSI_RXFIFO_WATERMARK) |
		      SSI_SFCSR_TFWM1(SSI_TXFIFO_WATERMARK) |
		      SSI_SFCSR_TFWM0(SSI_TXFIFO_WATERMARK)),
		     ioaddr + SSI_SFCSR);
	__raw_writel(0, ioaddr + SSI_SIER);

	SSI_DUMP();
	return 0;
}

static int imx_ssi_hw_tx_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *cpu_dai)
{
	struct imx_ssi *priv = (struct imx_ssi *)cpu_dai->private_data;
	void __iomem *ioaddr = priv->ioaddr;
	u32 stccr, stcr, sier;

	stccr = __raw_readl(ioaddr + SSI_STCCR) & ~SSI_STCCR_WL_MASK;
	stcr = __raw_readl(ioaddr + SSI_STCR);
	sier = __raw_readl(ioaddr + SSI_SIER);

	/* DAI data (word) size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		stccr |= SSI_STCCR_WL(16);
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		stccr |= SSI_STCCR_WL(20);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		stccr |= SSI_STCCR_WL(24);
		break;
	}

	/* enable interrupts */
	if ((cpu_dai->id & 0x1) == 0)
		stcr |= SSI_STCR_TFEN0;
	else
		stcr |= SSI_STCR_TFEN1;
	sier |= SSI_SIER_TDMAE | SSI_SIER_TIE | SSI_SIER_TUE0_EN;

	__raw_writel(stcr, ioaddr + SSI_STCR);
	__raw_writel(stccr, ioaddr + SSI_STCCR);
	__raw_writel(sier, ioaddr + SSI_SIER);

	return 0;
}

static int imx_ssi_hw_rx_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *cpu_dai)
{
	u32 srccr, srcr, sier;
	struct imx_ssi *priv = (struct imx_ssi *)cpu_dai->private_data;
	void __iomem *ioaddr = priv->ioaddr;
	bool sync_mode = priv->sync_mode;

	srccr =
	    sync_mode ? __raw_readl(ioaddr + SSI_STCCR) :
	    __raw_readl(ioaddr + SSI_SRCCR);
	srcr = __raw_readl(ioaddr + SSI_SRCR);
	sier = __raw_readl(ioaddr + SSI_SIER);
	srccr &= ~SSI_SRCCR_WL_MASK;

	/* DAI data (word) size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		srccr |= SSI_SRCCR_WL(16);
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		srccr |= SSI_SRCCR_WL(20);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		srccr |= SSI_SRCCR_WL(24);
		break;
	}

	/* enable interrupts */
	if ((cpu_dai->id & 0x1) == 0)
		srcr |= SSI_SRCR_RFEN0;
	else
		srcr |= SSI_SRCR_RFEN1;
	sier |= SSI_SIER_RDMAE | SSI_SIER_RIE | SSI_SIER_ROE0_EN;

	__raw_writel(srcr, ioaddr + SSI_SRCR);
	if (sync_mode)
		__raw_writel(srccr, ioaddr + SSI_STCCR);
	else
		__raw_writel(srccr, ioaddr + SSI_SRCCR);
	__raw_writel(sier, ioaddr + SSI_SIER);

	return 0;
}

/*
 * Should only be called when port is inactive (i.e. SSIEN = 0),
 * although can be called multiple times by upper layers.
 */
static int imx_ssi_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *cpu_dai)
{
	struct imx_ssi *priv = (struct imx_ssi *)cpu_dai->private_data;
	void __iomem *ioaddr = priv->ioaddr;
	int id;

	id = cpu_dai->id;

	/* Tx/Rx config */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* cant change any parameters when SSI is running */
		if ((__raw_readl(ioaddr + SSI_SCR) & SSI_SCR_SSIEN) &&
		    (__raw_readl(ioaddr + SSI_SCR) & SSI_SCR_TE))
			return 0;
		return imx_ssi_hw_tx_params(substream, params, cpu_dai);
	} else {
		/* cant change any parameters when SSI is running */
		if ((__raw_readl(ioaddr + SSI_SCR) & SSI_SCR_SSIEN) &&
		    (__raw_readl(ioaddr + SSI_SCR) & SSI_SCR_RE))
			return 0;
		return imx_ssi_hw_rx_params(substream, params, cpu_dai);
	}
}

static int imx_ssi_prepare(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *cpu_dai)
{
	struct imx_ssi *priv = (struct imx_ssi *)cpu_dai->private_data;
	void __iomem *ioaddr = priv->ioaddr;
	u32 scr;

	/* enable the SSI port, note that no other port config
	 * should happen after SSIEN is set */
	scr = __raw_readl(ioaddr + SSI_SCR);
	__raw_writel((scr | SSI_SCR_SSIEN), ioaddr + SSI_SCR);

	SSI_DUMP();
	return 0;
}

static int imx_ssi_trigger(struct snd_pcm_substream *substream, int cmd,
			   struct snd_soc_dai *cpu_dai)
{
	struct imx_ssi *priv = (struct imx_ssi *)cpu_dai->private_data;
	void __iomem *ioaddr = priv->ioaddr;
	u32 scr;

	scr = __raw_readl(ioaddr + SSI_SCR);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			if (scr & SSI_SCR_RE) {
				__raw_writel(0, ioaddr + SSI_SCR);
			}
			scr |= SSI_SCR_TE;
		} else
			scr |= SSI_SCR_RE;
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			scr &= ~SSI_SCR_TE;
		else
			scr &= ~SSI_SCR_RE;
		break;
	default:
		return -EINVAL;
	}
	__raw_writel(scr, ioaddr + SSI_SCR);

	SSI_DUMP();
	return 0;
}

static void imx_ssi_shutdown(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *cpu_dai)
{
	struct imx_ssi *priv = (struct imx_ssi *)cpu_dai->private_data;
	void __iomem *ioaddr = priv->ioaddr;
	int id;

	id = cpu_dai->id;

	/* shutdown SSI if neither Tx or Rx is active */
	if (cpu_dai->playback.active || cpu_dai->capture.active)
		return;

	if (clk_get_usecount(priv->ssi_clk) > 1) {
		clk_disable(priv->ssi_clk);
		return;
	}

	__raw_writel(0, ioaddr + SSI_SCR);

	clk_disable(priv->ssi_clk);
}

#ifdef CONFIG_PM
static int imx_ssi_suspend(struct snd_soc_dai *dai)
{
	if (!dai->active)
		return 0;

	/* do we need to disable any clocks? */

	return 0;
}

static int imx_ssi_resume(struct snd_soc_dai *dai)
{
	if (!dai->active)
		return 0;

	/* do we need to enable any clocks? */

	return 0;
}
#else
#define imx_ssi_suspend	NULL
#define imx_ssi_resume	NULL
#endif

static int fifo_err_counter;

static irqreturn_t imx_ssi_irq(int irq, void *dev_id)
{
	struct imx_ssi *priv = (struct imx_ssi *)dev_id;
	void __iomem *ioaddr = priv->ioaddr;
	if (fifo_err_counter++ % 1000 == 0)
		printk(KERN_ERR "%s %s SISR %x SIER %x fifo_errs=%d\n",
		       __func__, priv->pdev->name,
		       __raw_readl(ioaddr + SSI_SISR),
		       __raw_readl(ioaddr + SSI_SIER), fifo_err_counter);
	__raw_writel((SSI_SIER_TUE0_EN | SSI_SIER_ROE0_EN), ioaddr + SSI_SISR);
	return IRQ_HANDLED;
}

static int imx_ssi_probe(struct platform_device *pdev, struct snd_soc_dai *dai)
{
	struct imx_ssi *priv = (struct imx_ssi *)dai->private_data;

	if (priv->irq >= 0) {
		if (request_irq(priv->irq, imx_ssi_irq, IRQF_SHARED,
				pdev->name, priv)) {
			printk(KERN_ERR "%s: failure requesting irq for %s\n",
			       __func__, pdev->name);
			return -EBUSY;
		}
	}

	return 0;
}

static void imx_ssi_remove(struct platform_device *pdev,
			   struct snd_soc_dai *dai)
{
	struct imx_ssi *priv = (struct imx_ssi *)dai->private_data;

	if (priv->irq >= 0)
		free_irq(priv->irq, dai);
}

#define IMX_SSI_RATES \
	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | \
	SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | \
	SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
	SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | \
	SNDRV_PCM_RATE_96000)

#define IMX_SSI_FORMATS \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
	SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_ops imx_ssi_dai_ops = {
	.startup = imx_ssi_startup,
	.shutdown = imx_ssi_shutdown,
	.trigger = imx_ssi_trigger,
	.prepare = imx_ssi_prepare,
	.hw_params = imx_ssi_hw_params,
	.set_sysclk = imx_ssi_set_dai_sysclk,
	.set_clkdiv = imx_ssi_set_dai_clkdiv,
	.set_fmt = imx_ssi_set_dai_fmt,
	.set_tdm_slot = imx_ssi_set_dai_tdm_slot,
};

struct snd_soc_dai *imx_ssi_dai[MAX_SSI_CHANNELS];
EXPORT_SYMBOL_GPL(imx_ssi_dai);

static int imx_ssi_dev_probe(struct platform_device *pdev)
{
	int fifo0_channel = pdev->id * 2;
	struct snd_soc_dai *dai;
	struct imx_ssi *priv;
	int fifo, channel;
	struct resource *res;
	int ret;

	BUG_ON(fifo0_channel >= MAX_SSI_CHANNELS);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	priv = kzalloc(sizeof(struct imx_ssi), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	/* Each SSI block has 2 fifos which share the same
	   private data (struct imx_ssi) */
	priv->baseaddr = res->start;
	priv->ioaddr = ioremap(res->start, 0x5C);
	priv->irq = platform_get_irq(pdev, 0);
	priv->ssi_clk = clk_get(&pdev->dev, "ssi_clk");
	priv->pdev = pdev;

	for (fifo = 0; fifo < 2; fifo++) {
		channel = (pdev->id * 2) + fifo;

		dai = kzalloc(sizeof(struct snd_soc_dai), GFP_KERNEL);
		if (IS_ERR(dai)) {
			ret = -ENOMEM;
			goto DAI_ERR;
		}

		dai->name = kasprintf(GFP_KERNEL, "imx-ssi-%d-%d",
				      pdev->id + 1, fifo);
		if (IS_ERR(dai->name)) {
			kfree(dai);
			ret = -ENOMEM;
			goto DAI_ERR;
		}

		dai->probe = imx_ssi_probe;
		dai->suspend = imx_ssi_suspend;
		dai->remove = imx_ssi_remove;
		dai->resume = imx_ssi_resume;

		dai->playback.channels_min = 1;
		dai->playback.channels_max = 2;
		dai->playback.rates = IMX_SSI_RATES;
		dai->playback.formats = IMX_SSI_FORMATS;

		dai->capture.channels_min = 1;
		dai->capture.channels_max = 2;
		dai->capture.rates = IMX_SSI_RATES;
		dai->capture.formats = IMX_SSI_FORMATS;

		dai->ops = &imx_ssi_dai_ops;

		dai->private_data = priv;

		dai->id = channel;
		imx_ssi_dai[channel] = dai;

		ret = snd_soc_register_dai(dai);
		if (ret < 0) {
			kfree(dai->name);
			kfree(dai);
			goto DAI_ERR;
		}
	}
	return 0;

DAI_ERR:
	if (fifo == 1) {
		dai = imx_ssi_dai[fifo0_channel];
		snd_soc_unregister_dai(dai);
		kfree(dai->name);
		kfree(dai);
	}

	clk_put(priv->ssi_clk);
	iounmap(priv->ioaddr);
	kfree(priv);
	return ret;
}

static int __devexit imx_ssi_dev_remove(struct platform_device *pdev)
{
	int fifo0_channel = pdev->id * 2;
	int fifo1_channel = (pdev->id * 2) + 1;
	struct imx_ssi *priv = imx_ssi_dai[fifo0_channel]->private_data;

	snd_soc_unregister_dai(imx_ssi_dai[fifo0_channel]);
	snd_soc_unregister_dai(imx_ssi_dai[fifo1_channel]);

	kfree(imx_ssi_dai[fifo0_channel]->name);
	kfree(imx_ssi_dai[fifo0_channel]);

	kfree(imx_ssi_dai[fifo1_channel]->name);
	kfree(imx_ssi_dai[fifo1_channel]);

	clk_put(priv->ssi_clk);
	iounmap(priv->ioaddr);
	kfree(priv);
	return 0;
}

static struct platform_driver imx_ssi_driver = {
	.probe = imx_ssi_dev_probe,
	.remove = __devexit_p(imx_ssi_dev_remove),
	.driver = {
		   .name = "mxc_ssi",
		   },
};

static int __init imx_ssi_init(void)
{
	return platform_driver_register(&imx_ssi_driver);
}

static void __exit imx_ssi_exit(void)
{
	platform_driver_unregister(&imx_ssi_driver);
}

module_init(imx_ssi_init);
module_exit(imx_ssi_exit);
MODULE_AUTHOR
    ("Liam Girdwood, liam.girdwood@wolfsonmicro.com, www.wolfsonmicro.com");
MODULE_DESCRIPTION("i.MX ASoC I2S driver");
MODULE_LICENSE("GPL");
