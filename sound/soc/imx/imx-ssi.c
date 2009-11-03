/*
 * imx-ssi.c  --  SSI driver for Freescale IMX
 *
 * Copyright 2006 Wolfson Microelectronics PLC.
 * Author: Liam Girdwood
 *         liam.girdwood@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 * Based on mxc-alsa-mc13783 (C) 2006-2009 Freescale Semiconductor, Inc.
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

static struct imx_ssi imx_ssi_data[4];

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
		       __raw_readl(SSI1_SCR), __raw_readl(SSI2_SCR));	\
		printk(KERN_INFO "sisr %x\t, %x\n", \
		       __raw_readl(SSI1_SISR), __raw_readl(SSI2_SISR));	\
		printk(KERN_INFO "stcr %x\t, %x\n", \
		       __raw_readl(SSI1_STCR), __raw_readl(SSI2_STCR)); \
		printk(KERN_INFO "srcr %x\t, %x\n", \
		       __raw_readl(SSI1_SRCR), __raw_readl(SSI2_SRCR)); \
		printk(KERN_INFO "stccr %x\t, %x\n", \
		       __raw_readl(SSI1_STCCR), __raw_readl(SSI2_STCCR)); \
		printk(KERN_INFO "srccr %x\t, %x\n", \
		       __raw_readl(SSI1_SRCCR), __raw_readl(SSI2_SRCCR)); \
		printk(KERN_INFO "sfcsr %x\t, %x\n", \
		       __raw_readl(SSI1_SFCSR), __raw_readl(SSI2_SFCSR)); \
		printk(KERN_INFO "stmsk %x\t, %x\n", \
		       __raw_readl(SSI1_STMSK), __raw_readl(SSI2_STMSK)); \
		printk(KERN_INFO "srmsk %x\t, %x\n", \
		       __raw_readl(SSI1_SRMSK), __raw_readl(SSI2_SRMSK)); \
		printk(KERN_INFO "sier %x\t, %x\n", \
		       __raw_readl(SSI1_SIER), __raw_readl(SSI2_SIER)); \
	} while (0);
#else
#define SSI_DUMP()
#endif

#define SSI1_PORT	0
#define SSI2_PORT	1

static int ssi_active[2] = { 0, 0 };

/*
 * SSI system clock configuration.
 * Should only be called when port is inactive (i.e. SSIEN = 0).
 */
static int imx_ssi_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
				  int clk_id, unsigned int freq, int dir)
{
	u32 scr;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1)
		scr = __raw_readl(SSI1_SCR);
	else
		scr = __raw_readl(SSI2_SCR);

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

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1)
		__raw_writel(scr, SSI1_SCR);
	else
		__raw_writel(scr, SSI2_SCR);

	return 0;
}

/*
 * SSI Clock dividers
 * Should only be called when port is inactive (i.e. SSIEN = 0).
 */
static int imx_ssi_set_dai_clkdiv(struct snd_soc_dai *cpu_dai,
				  int div_id, int div)
{
	u32 stccr, srccr;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		if (__raw_readl(SSI1_SCR) & SSI_SCR_SSIEN)
			return 0;

		srccr = __raw_readl(SSI1_SRCCR);
		stccr = __raw_readl(SSI1_STCCR);
	} else {
		if (__raw_readl(SSI2_SCR) & SSI_SCR_SSIEN)
			return 0;

		srccr = __raw_readl(SSI2_SRCCR);
		stccr = __raw_readl(SSI2_STCCR);
	}

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

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		__raw_writel(stccr, SSI1_STCCR);
		__raw_writel(srccr, SSI1_SRCCR);
	} else {
		__raw_writel(stccr, SSI2_STCCR);
		__raw_writel(srccr, SSI2_SRCCR);
	}
	return 0;
}

/*
 * SSI Network Mode or TDM slots configuration.
 * Should only be called when port is inactive (i.e. SSIEN = 0).
 */
static int imx_ssi_set_dai_tdm_slot(struct snd_soc_dai *cpu_dai,
				    unsigned int mask, int slots)
{
	u32 stmsk, srmsk, stccr;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		if (__raw_readl(SSI1_SCR) & SSI_SCR_SSIEN)
			return 0;
		stccr = __raw_readl(SSI1_STCCR);
	} else {
		if (__raw_readl(SSI2_SCR) & SSI_SCR_SSIEN)
			return 0;
		stccr = __raw_readl(SSI2_STCCR);
	}

	stmsk = srmsk = mask;
	stccr &= ~SSI_STCCR_DC_MASK;
	stccr |= SSI_STCCR_DC(slots - 1);

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		__raw_writel(stmsk, SSI1_STMSK);
		__raw_writel(srmsk, SSI1_SRMSK);
		__raw_writel(stccr, SSI1_STCCR);
		__raw_writel(stccr, SSI1_SRCCR);
	} else {
		__raw_writel(stmsk, SSI2_STMSK);
		__raw_writel(srmsk, SSI2_SRMSK);
		__raw_writel(stccr, SSI2_STCCR);
		__raw_writel(stccr, SSI2_SRCCR);
	}

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
	struct imx_ssi *ssi_mode = (struct imx_ssi *)cpu_dai->private_data;
	u32 stcr = 0, srcr = 0, scr;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1)
		scr = __raw_readl(SSI1_SCR) & ~(SSI_SCR_SYN | SSI_SCR_NET);
	else
		scr = __raw_readl(SSI2_SCR) & ~(SSI_SCR_SYN | SSI_SCR_NET);

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
		    && ssi_mode->network_mode) {
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
		    && ssi_mode->network_mode) {
			scr &= ~SSI_SCR_I2S_MODE_MASK;
			scr |= SSI_SCR_I2S_MODE_SLAVE;
		}
		break;
	}

	/* sync */
	if (ssi_mode->sync_mode)
		scr |= SSI_SCR_SYN;

	/* tdm - only for stereo atm */
	if (ssi_mode->network_mode)
		scr |= SSI_SCR_NET;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		__raw_writel(stcr, SSI1_STCR);
		__raw_writel(srcr, SSI1_SRCR);
		__raw_writel(scr, SSI1_SCR);
	} else {
		__raw_writel(stcr, SSI2_STCR);
		__raw_writel(srcr, SSI2_SRCR);
		__raw_writel(scr, SSI2_SCR);
	}
	SSI_DUMP();
	return 0;
}

static struct clk *ssi1_clk;
static struct clk *ssi2_clk;

static int imx_ssi_startup(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *cpu_dai)
{
	/* we cant really change any SSI values after SSI is enabled
	 * need to fix in software for max flexibility - lrg */
	if (cpu_dai->playback.active || cpu_dai->capture.active)
		return 0;

	/* reset the SSI port - Sect 45.4.4 */
	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {

		if (ssi_active[SSI1_PORT]++)
			return 0;

		__raw_writel(0, SSI1_SCR);
		ssi1_clk = clk_get(NULL, "ssi_clk.0");
		clk_enable(ssi1_clk);

		/* BIG FAT WARNING
		 * SDMA FIFO watermark must == SSI FIFO watermark for
		 * best results.
		 */
		__raw_writel((SSI_SFCSR_RFWM1(SSI_RXFIFO_WATERMARK) |
			      SSI_SFCSR_RFWM0(SSI_RXFIFO_WATERMARK) |
			      SSI_SFCSR_TFWM1(SSI_TXFIFO_WATERMARK) |
			      SSI_SFCSR_TFWM0(SSI_TXFIFO_WATERMARK)),
			     SSI1_SFCSR);
		__raw_writel(0, SSI1_SIER);
	} else {

		if (ssi_active[SSI2_PORT]++)
			return 0;

		__raw_writel(0, SSI2_SCR);
		ssi2_clk = clk_get(NULL, "ssi_clk.1");
		clk_enable(ssi2_clk);
		/* above warning applies here too */
		__raw_writel((SSI_SFCSR_RFWM1(SSI_RXFIFO_WATERMARK) |
			      SSI_SFCSR_RFWM0(SSI_RXFIFO_WATERMARK) |
			      SSI_SFCSR_TFWM1(SSI_TXFIFO_WATERMARK) |
			      SSI_SFCSR_TFWM0(SSI_TXFIFO_WATERMARK)),
			     SSI2_SFCSR);
		__raw_writel(0, SSI2_SIER);
	}

	SSI_DUMP();
	return 0;
}

static int imx_ssi_hw_tx_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *cpu_dai)
{
	u32 stccr, stcr, sier;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		stccr = __raw_readl(SSI1_STCCR) & ~SSI_STCCR_WL_MASK;
		stcr = __raw_readl(SSI1_STCR);
		sier = __raw_readl(SSI1_SIER);
	} else {
		stccr = __raw_readl(SSI2_STCCR) & ~SSI_STCCR_WL_MASK;
		stcr = __raw_readl(SSI2_STCR);
		sier = __raw_readl(SSI2_SIER);
	}

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
	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI2)
		stcr |= SSI_STCR_TFEN0;
	else
		stcr |= SSI_STCR_TFEN1;
	sier |= SSI_SIER_TDMAE | SSI_SIER_TIE | SSI_SIER_TUE0_EN;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		__raw_writel(stcr, SSI1_STCR);
		__raw_writel(stccr, SSI1_STCCR);
		__raw_writel(sier, SSI1_SIER);
	} else {
		__raw_writel(stcr, SSI2_STCR);
		__raw_writel(stccr, SSI2_STCCR);
		__raw_writel(sier, SSI2_SIER);
	}

	return 0;
}

static int imx_ssi_hw_rx_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *cpu_dai)
{
	u32 srccr, srcr, sier;
	struct imx_ssi *ssi_mode = (struct imx_ssi *)cpu_dai->private_data;
	bool sync_mode = ssi_mode->sync_mode;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		srccr =
		    sync_mode ? __raw_readl(SSI1_STCCR) :
		    __raw_readl(SSI1_SRCCR);
		srcr = __raw_readl(SSI1_SRCR);
		sier = __raw_readl(SSI1_SIER);
	} else {
		srccr =
		    sync_mode ? __raw_readl(SSI2_STCCR) :
		    __raw_readl(SSI2_SRCCR);
		srcr = __raw_readl(SSI2_SRCR);
		sier = __raw_readl(SSI2_SIER);
	}
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
	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI2)
		srcr |= SSI_SRCR_RFEN0;
	else
		srcr |= SSI_SRCR_RFEN1;
	sier |= SSI_SIER_RDMAE | SSI_SIER_RIE | SSI_SIER_ROE0_EN;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		__raw_writel(srcr, SSI1_SRCR);
		if (sync_mode)
			__raw_writel(srccr, SSI1_STCCR);
		else
			__raw_writel(srccr, SSI1_SRCCR);
		__raw_writel(sier, SSI1_SIER);
	} else {
		__raw_writel(srcr, SSI2_SRCR);
		if (sync_mode)
			__raw_writel(srccr, SSI2_STCCR);
		else
			__raw_writel(srccr, SSI2_SRCCR);
		__raw_writel(sier, SSI2_SIER);
	}
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
	int id;

	id = cpu_dai->id;

	/* Tx/Rx config */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* cant change any parameters when SSI is running */
		if (id == IMX_DAI_SSI0 || id == IMX_DAI_SSI1) {
			if ((__raw_readl(SSI1_SCR) & SSI_SCR_SSIEN) &&
			    (__raw_readl(SSI1_SCR) & SSI_SCR_TE))
				return 0;
		} else {
			if ((__raw_readl(SSI2_SCR) & SSI_SCR_SSIEN) &&
			    (__raw_readl(SSI2_SCR) & SSI_SCR_TE))
				return 0;
		}
		return imx_ssi_hw_tx_params(substream, params, cpu_dai);
	} else {
		/* cant change any parameters when SSI is running */
		if (id == IMX_DAI_SSI0 || id == IMX_DAI_SSI1) {
			if ((__raw_readl(SSI1_SCR) & SSI_SCR_SSIEN) &&
			    (__raw_readl(SSI1_SCR) & SSI_SCR_RE))
				return 0;
		} else {
			if ((__raw_readl(SSI2_SCR) & SSI_SCR_SSIEN) &&
			    (__raw_readl(SSI2_SCR) & SSI_SCR_RE))
				return 0;
		}
		return imx_ssi_hw_rx_params(substream, params, cpu_dai);
	}
}

static int imx_ssi_prepare(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *cpu_dai)
{
	u32 scr;

	/* enable the SSI port, note that no other port config
	 * should happen after SSIEN is set */
	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		scr = __raw_readl(SSI1_SCR);
		__raw_writel((scr | SSI_SCR_SSIEN), SSI1_SCR);
	} else {
		scr = __raw_readl(SSI2_SCR);
		__raw_writel((scr | SSI_SCR_SSIEN), SSI2_SCR);
	}
	SSI_DUMP();
	return 0;
}

static int imx_ssi_trigger(struct snd_pcm_substream *substream, int cmd,
			   struct snd_soc_dai *cpu_dai)
{
	u32 scr;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1)
		scr = __raw_readl(SSI1_SCR);
	else
		scr = __raw_readl(SSI2_SCR);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			if (scr & SSI_SCR_RE) {
				if (cpu_dai->id == IMX_DAI_SSI0
				    || cpu_dai->id == IMX_DAI_SSI1)
					__raw_writel(0, SSI1_SCR);
				else
					__raw_writel(0, SSI2_SCR);
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
	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1)
		__raw_writel(scr, SSI1_SCR);
	else
		__raw_writel(scr, SSI2_SCR);

	SSI_DUMP();
	return 0;
}

static void imx_ssi_shutdown(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *cpu_dai)
{
	int id;

	id = cpu_dai->id;

	/* shutdown SSI if neither Tx or Rx is active */
	if (cpu_dai->playback.active || cpu_dai->capture.active)
		return;

	if (id == IMX_DAI_SSI0 || id == IMX_DAI_SSI1) {

		if (--ssi_active[SSI1_PORT] > 1)
			return;

		__raw_writel(0, SSI1_SCR);

		clk_disable(ssi1_clk);
		clk_put(ssi1_clk);

	} else {
		if (--ssi_active[SSI2_PORT])
			return;
		__raw_writel(0, SSI2_SCR);
		clk_disable(ssi2_clk);
		clk_put(ssi2_clk);
	}
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

static irqreturn_t ssi1_irq(int irq, void *dev_id)
{
	if (fifo_err_counter++ % 1000 == 0)
		printk(KERN_ERR "ssi1_irq SISR %x SIER %x fifo_errs=%d\n",
		       __raw_readl(SSI1_SISR), __raw_readl(SSI1_SIER),
		       fifo_err_counter);
	__raw_writel((SSI_SIER_TUE0_EN | SSI_SIER_ROE0_EN), SSI1_SISR);
	return IRQ_HANDLED;
}

static irqreturn_t ssi2_irq(int irq, void *dev_id)
{
	if (fifo_err_counter++ % 1000 == 0)
		printk(KERN_ERR "ssi2_irq SISR %x SIER %x fifo_errs=%d\n",
		       __raw_readl(SSI2_SISR), __raw_readl(SSI2_SIER),
		       fifo_err_counter);
	__raw_writel((SSI_SIER_TUE0_EN | SSI_SIER_ROE0_EN), SSI2_SISR);
	return IRQ_HANDLED;
}

static int imx_ssi_probe(struct platform_device *pdev, struct snd_soc_dai *dai)
{
	if ((!strcmp(dai->name, "imx-ssi-1-0")) ||
	    (!strcmp(dai->name, "imx-ssi-1-1")))
		if (request_irq(MXC_INT_SSI1, ssi1_irq, 0, "ssi1", dai)) {
			printk(KERN_ERR "%s: failure requesting irq %s\n",
			       __func__, "ssi1");
			return -EBUSY;
		}

	if ((!strcmp(dai->name, "imx-ssi-2-0")) ||
	    (!strcmp(dai->name, "imx-ssi-2-1")))
		if (request_irq(MXC_INT_SSI2, ssi2_irq, 0, "ssi2", dai)) {
			printk(KERN_ERR "%s: failure requesting irq %s\n",
			       __func__, "ssi2");
			return -EBUSY;
		}

	return 0;
}

static void imx_ssi_remove(struct platform_device *pdev,
			   struct snd_soc_dai *dai)
{
	if ((!strcmp(dai->name, "imx-ssi-1-0")) ||
	    (!strcmp(dai->name, "imx-ssi-1-1")))
		free_irq(MXC_INT_SSI1, dai);

	if ((!strcmp(dai->name, "imx-ssi-2-0")) ||
	    (!strcmp(dai->name, "imx-ssi-2-1")))
		free_irq(MXC_INT_SSI2, dai);
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

struct snd_soc_dai imx_ssi_dai[] = {
	{
	 .name = "imx-ssi-1-0",
	 .id = IMX_DAI_SSI0,
	 .probe = imx_ssi_probe,
	 .suspend = imx_ssi_suspend,
	 .remove = imx_ssi_remove,
	 .resume = imx_ssi_resume,
	 .playback = {
		      .channels_min = 1,
		      .channels_max = 2,
		      .rates = IMX_SSI_RATES,
		      .formats = IMX_SSI_FORMATS,
		      },
	 .capture = {
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = IMX_SSI_RATES,
		     .formats = IMX_SSI_FORMATS,
		     },
	 .ops = &imx_ssi_dai_ops,
	 .private_data = &imx_ssi_data[IMX_DAI_SSI0],
	 },
	{
	 .name = "imx-ssi-1-1",
	 .id = IMX_DAI_SSI1,
	 .probe = imx_ssi_probe,
	 .suspend = imx_ssi_suspend,
	 .remove = imx_ssi_remove,
	 .resume = imx_ssi_resume,
	 .playback = {
		      .channels_min = 1,
		      .channels_max = 2,
		      .rates = IMX_SSI_RATES,
		      .formats = IMX_SSI_FORMATS,
		      },
	 .capture = {
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = IMX_SSI_RATES,
		     .formats = IMX_SSI_FORMATS,
		     },
	 .ops = &imx_ssi_dai_ops,
	 .private_data = &imx_ssi_data[IMX_DAI_SSI1],
	 },
	{
	 .name = "imx-ssi-2-0",
	 .id = IMX_DAI_SSI2,
	 .probe = imx_ssi_probe,
	 .suspend = imx_ssi_suspend,
	 .remove = imx_ssi_remove,
	 .resume = imx_ssi_resume,
	 .playback = {
		      .channels_min = 1,
		      .channels_max = 2,
		      .rates = IMX_SSI_RATES,
		      .formats = IMX_SSI_FORMATS,
		      },
	 .capture = {
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = IMX_SSI_RATES,
		     .formats = IMX_SSI_FORMATS,
		     },
	 .ops = &imx_ssi_dai_ops,
	 .private_data = &imx_ssi_data[IMX_DAI_SSI2],
	 },
	{
	 .name = "imx-ssi-2-1",
	 .id = IMX_DAI_SSI3,
	 .probe = imx_ssi_probe,
	 .suspend = imx_ssi_suspend,
	 .remove = imx_ssi_remove,
	 .resume = imx_ssi_resume,
	 .playback = {
		      .channels_min = 1,
		      .channels_max = 2,
		      .rates = IMX_SSI_RATES,
		      .formats = IMX_SSI_FORMATS,
		      },
	 .capture = {
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = IMX_SSI_RATES,
		     .formats = IMX_SSI_FORMATS,
		     },
	 .ops = &imx_ssi_dai_ops,
	 .private_data = &imx_ssi_data[IMX_DAI_SSI3],
	 },
};

EXPORT_SYMBOL_GPL(imx_ssi_dai);

static int __init imx_ssi_init(void)
{
	return snd_soc_register_dais(imx_ssi_dai, ARRAY_SIZE(imx_ssi_dai));
}

static void __exit imx_ssi_exit(void)
{
	snd_soc_unregister_dais(imx_ssi_dai, ARRAY_SIZE(imx_ssi_dai));
}

module_init(imx_ssi_init);
module_exit(imx_ssi_exit);
MODULE_AUTHOR
    ("Liam Girdwood, liam.girdwood@wolfsonmicro.com, www.wolfsonmicro.com");
MODULE_DESCRIPTION("i.MX ASoC I2S driver");
MODULE_LICENSE("GPL");
