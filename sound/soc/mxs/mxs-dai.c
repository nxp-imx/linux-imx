/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <mach/dma.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>

#include <mach/mx28.h>

#include "mxs-pcm.h"
#include "mxs-dai.h"

#define SAIF0_CTRL (IO_ADDRESS(SAIF0_PHYS_ADDR) + HW_SAIF_CTRL)
#define SAIF1_CTRL (IO_ADDRESS(SAIF1_PHYS_ADDR) + HW_SAIF_CTRL)
#define SAIF0_CTRL_SET (IO_ADDRESS(SAIF0_PHYS_ADDR) + HW_SAIF_CTRL_SET)
#define SAIF1_CTRL_SET (IO_ADDRESS(SAIF1_PHYS_ADDR) + HW_SAIF_CTRL_SET)
#define SAIF0_CTRL_CLR (IO_ADDRESS(SAIF0_PHYS_ADDR) + HW_SAIF_CTRL_CLR)
#define SAIF1_CTRL_CLR (IO_ADDRESS(SAIF1_PHYS_ADDR) + HW_SAIF_CTRL_CLR)

#define SAIF0_STAT (IO_ADDRESS(SAIF0_PHYS_ADDR) + HW_SAIF_STAT)
#define SAIF1_STAT (IO_ADDRESS(SAIF1_PHYS_ADDR) + HW_SAIF_STAT)
#define SAIF0_STAT_SET (IO_ADDRESS(SAIF0_PHYS_ADDR) + HW_SAIF_STAT_SET)
#define SAIF1_STAT_SET (IO_ADDRESS(SAIF1_PHYS_ADDR) + HW_SAIF_STAT_SET)
#define SAIF0_STAT_CLR (IO_ADDRESS(SAIF0_PHYS_ADDR) + HW_SAIF_STAT_CLR)
#define SAIF1_STAT_CLR (IO_ADDRESS(SAIF1_PHYS_ADDR) + HW_SAIF_STAT_CLR)

#define SAIF0_DATA (IO_ADDRESS(SAIF0_PHYS_ADDR) + HW_SAIF_DATA)
#define SAIF1_DATA (IO_ADDRESS(SAIF1_PHYS_ADDR) + HW_SAIF_DATA)
#define SAIF0_DATA_SET (IO_ADDRESS(SAIF0_PHYS_ADDR) + HW_SAIF_DATA_SET)
#define SAIF1_DATA_SET (IO_ADDRESS(SAIF1_PHYS_ADDR) + HW_SAIF_DATA_SET)
#define SAIF0_DATA_CLR (IO_ADDRESS(SAIF0_PHYS_ADDR) + HW_SAIF_DATA_CLR)
#define SAIF1_DATA_CLR (IO_ADDRESS(SAIF1_PHYS_ADDR) + HW_SAIF_DATA_CLR)

#define SAIF0_VERSION (IO_ADDRESS(SAIF0_PHYS_ADDR) + HW_SAIF_VERSION)
#define SAIF1_VERSION (IO_ADDRESS(SAIF1_PHYS_ADDR) + HW_SAIF_VERSION)

#define HW_SAIF_CTRL	(0x00000000)
#define HW_SAIF_CTRL_SET	(0x00000004)
#define HW_SAIF_CTRL_CLR	(0x00000008)
#define HW_SAIF_CTRL_TOG	(0x0000000c)

#define BM_SAIF_CTRL_SFTRST	0x80000000
#define BM_SAIF_CTRL_CLKGATE	0x40000000
#define BP_SAIF_CTRL_BITCLK_MULT_RATE	27
#define BM_SAIF_CTRL_BITCLK_MULT_RATE	0x38000000
#define BF_SAIF_CTRL_BITCLK_MULT_RATE(v)  \
		(((v) << 27) & BM_SAIF_CTRL_BITCLK_MULT_RATE)
#define BM_SAIF_CTRL_BITCLK_BASE_RATE	0x04000000
#define BM_SAIF_CTRL_FIFO_ERROR_IRQ_EN	0x02000000
#define BM_SAIF_CTRL_FIFO_SERVICE_IRQ_EN	0x01000000
#define BP_SAIF_CTRL_RSRVD2	21
#define BM_SAIF_CTRL_RSRVD2	0x00E00000
#define BF_SAIF_CTRL_RSRVD2(v)  \
		(((v) << 21) & BM_SAIF_CTRL_RSRVD2)
#define BP_SAIF_CTRL_DMAWAIT_COUNT	16
#define BM_SAIF_CTRL_DMAWAIT_COUNT	0x001F0000
#define BF_SAIF_CTRL_DMAWAIT_COUNT(v)  \
		(((v) << 16) & BM_SAIF_CTRL_DMAWAIT_COUNT)
#define BP_SAIF_CTRL_CHANNEL_NUM_SELECT	14
#define BM_SAIF_CTRL_CHANNEL_NUM_SELECT	0x0000C000
#define BF_SAIF_CTRL_CHANNEL_NUM_SELECT(v)  \
		(((v) << 14) & BM_SAIF_CTRL_CHANNEL_NUM_SELECT)
#define BM_SAIF_CTRL_LRCLK_PULSE	0x00002000
#define BM_SAIF_CTRL_BIT_ORDER	0x00001000
#define BM_SAIF_CTRL_DELAY	0x00000800
#define BM_SAIF_CTRL_JUSTIFY	0x00000400
#define BM_SAIF_CTRL_LRCLK_POLARITY	0x00000200
#define BM_SAIF_CTRL_BITCLK_EDGE	0x00000100
#define BP_SAIF_CTRL_WORD_LENGTH	4
#define BM_SAIF_CTRL_WORD_LENGTH	0x000000F0
#define BF_SAIF_CTRL_WORD_LENGTH(v)  \
		(((v) << 4) & BM_SAIF_CTRL_WORD_LENGTH)
#define BM_SAIF_CTRL_BITCLK_48XFS_ENABLE	0x00000008
#define BM_SAIF_CTRL_SLAVE_MODE	0x00000004
#define BM_SAIF_CTRL_READ_MODE	0x00000002
#define BM_SAIF_CTRL_RUN	0x00000001

#define HW_SAIF_STAT	(0x00000010)
#define HW_SAIF_STAT_SET	(0x00000014)
#define HW_SAIF_STAT_CLR	(0x00000018)
#define HW_SAIF_STAT_TOG	(0x0000001c)

#define BM_SAIF_STAT_PRESENT	0x80000000
#define BP_SAIF_STAT_RSRVD2	17
#define BM_SAIF_STAT_RSRVD2	0x7FFE0000
#define BF_SAIF_STAT_RSRVD2(v)  \
		(((v) << 17) & BM_SAIF_STAT_RSRVD2)
#define BM_SAIF_STAT_DMA_PREQ	0x00010000
#define BP_SAIF_STAT_RSRVD1	7
#define BM_SAIF_STAT_RSRVD1	0x0000FF80
#define BF_SAIF_STAT_RSRVD1(v)  \
		(((v) << 7) & BM_SAIF_STAT_RSRVD1)
#define BM_SAIF_STAT_FIFO_UNDERFLOW_IRQ	0x00000040
#define BM_SAIF_STAT_FIFO_OVERFLOW_IRQ	0x00000020
#define BM_SAIF_STAT_FIFO_SERVICE_IRQ	0x00000010
#define BP_SAIF_STAT_RSRVD0	1
#define BM_SAIF_STAT_RSRVD0	0x0000000E
#define BF_SAIF_STAT_RSRVD0(v)  \
		(((v) << 1) & BM_SAIF_STAT_RSRVD0)
#define BM_SAIF_STAT_BUSY	0x00000001

#define HW_SAIF_DATA	(0x00000020)
#define HW_SAIF_DATA_SET	(0x00000024)
#define HW_SAIF_DATA_CLR	(0x00000028)
#define HW_SAIF_DATA_TOG	(0x0000002c)

#define BP_SAIF_DATA_PCM_RIGHT	16
#define BM_SAIF_DATA_PCM_RIGHT	0xFFFF0000
#define BF_SAIF_DATA_PCM_RIGHT(v) \
		(((v) << 16) & BM_SAIF_DATA_PCM_RIGHT)
#define BP_SAIF_DATA_PCM_LEFT	0
#define BM_SAIF_DATA_PCM_LEFT	0x0000FFFF
#define BF_SAIF_DATA_PCM_LEFT(v)  \
		(((v) << 0) & BM_SAIF_DATA_PCM_LEFT)

#define HW_SAIF_VERSION	(0x00000030)

#define BP_SAIF_VERSION_MAJOR	24
#define BM_SAIF_VERSION_MAJOR	0xFF000000
#define BF_SAIF_VERSION_MAJOR(v) \
		(((v) << 24) & BM_SAIF_VERSION_MAJOR)
#define BP_SAIF_VERSION_MINOR	16
#define BM_SAIF_VERSION_MINOR	0x00FF0000
#define BF_SAIF_VERSION_MINOR(v)  \
		(((v) << 16) & BM_SAIF_VERSION_MINOR)
#define BP_SAIF_VERSION_STEP	0
#define BM_SAIF_VERSION_STEP	0x0000FFFF
#define BF_SAIF_VERSION_STEP(v)  \
		(((v) << 0) & BM_SAIF_VERSION_STEP)
/* debug */
#define MXS_SAIF_DEBUG 0
#if MXS_SAIF_DEBUG
#define dbg(format, arg...) printk(format, ## arg)
#else
#define dbg(format, arg...)
#endif

#define MXS_SAIF_DUMP 0
#if MXS_SAIF_DUMP
#define SAIF_DUMP() \
	do { \
		printk(KERN_INFO "dump @ %s\n", __func__);\
		printk(KERN_INFO "scr %x\t, %x\n", \
			__raw_readl(SAIF0_CTRL), __raw_readl(SAIF1_CTRL));\
		printk(KERN_INFO "stat %x\t, %x\n", \
			__raw_readl(SAIF0_STAT), __raw_readl(SAIF1_STAT));\
		printk(KERN_INFO "data %x\t, %x\n", \
			__raw_readl(SAIF0_DATA), __raw_readl(SAIF1_DATA));\
		printk(KERN_INFO "version %x\t, %x\n", \
			__raw_readl(SAIF0_VERSION), __raw_readl(SAIF1_VERSION));
	} while (0);
#else
#define SAIF_DUMP()
#endif

#define SAIF0_PORT	0
#define SAIF1_PORT	1

#define MXS_DAI_SAIF0 0
#define MXS_DAI_SAIF1 1

static struct mxs_saif mxs_saif_en;

static int saif_active[2] = { 0, 0 };

struct mxs_pcm_dma_params mxs_saif_0 = {
	.name = "mxs-saif-0",
	.dma_ch	= MXS_DMA_CHANNEL_AHB_APBX_SAIF0,
	.irq = IRQ_SAIF0_DMA,
};

struct mxs_pcm_dma_params mxs_saif_1 = {
	.name = "mxs-saif-1",
	.dma_ch	= MXS_DMA_CHANNEL_AHB_APBX_SAIF1,
	.irq = IRQ_SAIF1_DMA,
};

/*
* SAIF system clock configuration.
* Should only be called when port is inactive.
*/
static int mxs_saif_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct clk *saif_clk;
	u32 scr;
	struct mxs_saif *saif_select = (struct mxs_saif *)cpu_dai->private_data;

	switch (clk_id) {
	case IMX_SSP_SYS_CLK:
		saif_clk = saif_select->saif_mclk;
		if (IS_ERR(saif_clk)) {
			pr_err("%s:failed to get sys_clk\n", __func__);
			return -EINVAL;
		}
		clk_set_rate(saif_clk, freq);
		clk_enable(saif_clk);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/*
 * SAIF Clock dividers
 * Should only be called when port is inactive.
 */
static int mxs_saif_set_dai_clkdiv(struct snd_soc_dai *cpu_dai,
				  int div_id, int div)
{
	u32 scr;
	struct mxs_saif *saif_select = (struct mxs_saif *)cpu_dai->private_data;

	if (saif_select->saif_clk == SAIF0)
		scr = __raw_readl(SAIF0_CTRL);
	else
		scr = __raw_readl(SAIF1_CTRL);

	scr &= ~BM_SAIF_CTRL_BITCLK_MULT_RATE;
	scr &= ~BM_SAIF_CTRL_BITCLK_BASE_RATE;

	switch (div_id) {
	case IMX_SSP_SYS_MCLK:
		switch (div) {
		case 32:
			scr |= BF_SAIF_CTRL_BITCLK_MULT_RATE(4);
			scr &= ~BM_SAIF_CTRL_BITCLK_BASE_RATE;
			break;
		case 64:
			scr |= BF_SAIF_CTRL_BITCLK_MULT_RATE(3);
			scr &= ~BM_SAIF_CTRL_BITCLK_BASE_RATE;
			break;
		case 128:
			scr |= BF_SAIF_CTRL_BITCLK_MULT_RATE(2);
			scr &= ~BM_SAIF_CTRL_BITCLK_BASE_RATE;
			break;
		case 256:
			scr |= BF_SAIF_CTRL_BITCLK_MULT_RATE(1);
			scr &= ~BM_SAIF_CTRL_BITCLK_BASE_RATE;
			break;
		case 512:
			scr |= BF_SAIF_CTRL_BITCLK_MULT_RATE(0);
			scr &= ~BM_SAIF_CTRL_BITCLK_BASE_RATE;
			break;
		case 48:
			scr |= BF_SAIF_CTRL_BITCLK_MULT_RATE(3);
			scr |= BM_SAIF_CTRL_BITCLK_BASE_RATE;
			break;
		case 96:
			scr |= BF_SAIF_CTRL_BITCLK_MULT_RATE(2);
			scr |= BM_SAIF_CTRL_BITCLK_BASE_RATE;
			break;
		case 192:
			scr |= BF_SAIF_CTRL_BITCLK_MULT_RATE(1);
			scr |= BM_SAIF_CTRL_BITCLK_BASE_RATE;
			break;
		case 384:
			scr |= BF_SAIF_CTRL_BITCLK_MULT_RATE(0);
			scr |= BM_SAIF_CTRL_BITCLK_BASE_RATE;
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	if (saif_select->saif_clk == SAIF0)
		__raw_writel(scr, SAIF0_CTRL);
	else
		__raw_writel(scr, SAIF1_CTRL);

	return 0;
}

/*
 * SAIF DAI format configuration.
 * Should only be called when port is inactive.
 * Note: We don't use the I2S modes but instead manually configure the
 * SAIF for I2S.
 */
static int mxs_saif_set_dai_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	u32 scr, stat;
	u32 scr0, scr1;
	struct mxs_saif *saif_select = (struct mxs_saif *)cpu_dai->private_data;

	stat = (__raw_readl(SAIF0_STAT)) | (__raw_readl(SAIF1_STAT));
	if (stat & BM_SAIF_STAT_BUSY)
		return 0;

	scr0 = __raw_readl(SAIF0_CTRL);
	scr1 = __raw_readl(SAIF1_CTRL);
	scr0 = scr0 & ~BM_SAIF_CTRL_BITCLK_EDGE & ~BM_SAIF_CTRL_LRCLK_POLARITY \
		& ~BM_SAIF_CTRL_JUSTIFY & ~BM_SAIF_CTRL_DELAY;
	scr1 = scr1 & ~BM_SAIF_CTRL_BITCLK_EDGE & ~BM_SAIF_CTRL_LRCLK_POLARITY \
		& ~BM_SAIF_CTRL_JUSTIFY & ~BM_SAIF_CTRL_DELAY;
	scr = 0;

	/* DAI mode */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		/* data frame low 1clk before data */
		scr |= BM_SAIF_CTRL_DELAY;
		scr &= ~BM_SAIF_CTRL_LRCLK_POLARITY;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		/* data frame high with data */
		scr &= ~BM_SAIF_CTRL_DELAY;
		scr &= ~BM_SAIF_CTRL_LRCLK_POLARITY;
		scr &= ~BM_SAIF_CTRL_JUSTIFY;
		break;
	}

	/* DAI clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_IB_IF:
		scr |= BM_SAIF_CTRL_BITCLK_EDGE;
		scr |= BM_SAIF_CTRL_LRCLK_POLARITY;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		scr |= BM_SAIF_CTRL_BITCLK_EDGE;
		scr &= ~BM_SAIF_CTRL_LRCLK_POLARITY;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		scr &= ~BM_SAIF_CTRL_BITCLK_EDGE;
		scr |= BM_SAIF_CTRL_LRCLK_POLARITY;
		break;
	case SND_SOC_DAIFMT_NB_NF:
		scr &= ~BM_SAIF_CTRL_BITCLK_EDGE;
		scr &= ~BM_SAIF_CTRL_LRCLK_POLARITY;
		break;
	}

	/* DAI clock master masks */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		if (saif_select->saif_clk == SAIF0) {
			scr &= ~BM_SAIF_CTRL_SLAVE_MODE;
			__raw_writel(scr | scr0, SAIF0_CTRL);
			scr |= BM_SAIF_CTRL_SLAVE_MODE;
			__raw_writel(scr | scr1, SAIF1_CTRL);
		} else {
			scr &= ~BM_SAIF_CTRL_SLAVE_MODE;
			__raw_writel(scr | scr1, SAIF1_CTRL);
			scr |= BM_SAIF_CTRL_SLAVE_MODE;
			__raw_writel(scr | scr0, SAIF0_CTRL);
		}
		break;
	}

	SAIF_DUMP();
	return 0;
}

static int mxs_saif_startup(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *cpu_dai)
{
	/* we cant really change any saif values after saif is enabled*/
	struct mxs_saif *saif_select = (struct mxs_saif *)cpu_dai->private_data;

	if (((saif_select->stream_mapping == PLAYBACK_SAIF0_CAPTURE_SAIF1) && \
		(substream->stream == SNDRV_PCM_STREAM_PLAYBACK)) || \
		((saif_select->stream_mapping == PLAYBACK_SAIF1_CAPTURE_SAIF0) \
		&& (substream->stream == SNDRV_PCM_STREAM_CAPTURE)))
		snd_soc_dai_set_dma_data(cpu_dai, substream, &mxs_saif_0);
	else
		snd_soc_dai_set_dma_data(cpu_dai, substream, &mxs_saif_1);

	if (cpu_dai->playback.active && cpu_dai->capture.active)
		return 0;

	if (((saif_select->stream_mapping == PLAYBACK_SAIF0_CAPTURE_SAIF1) && \
		(substream->stream == SNDRV_PCM_STREAM_PLAYBACK)) || \
		((saif_select->stream_mapping == PLAYBACK_SAIF1_CAPTURE_SAIF0) \
		&& (substream->stream == SNDRV_PCM_STREAM_CAPTURE)))
		if (saif_active[SAIF0_PORT]++)
			return 0;
	if (((saif_select->stream_mapping == PLAYBACK_SAIF0_CAPTURE_SAIF1) && \
		(substream->stream == SNDRV_PCM_STREAM_CAPTURE)) || \
		((saif_select->stream_mapping == PLAYBACK_SAIF1_CAPTURE_SAIF0) \
		&& (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)))
		if (saif_active[SAIF1_PORT]++)
			return 0;
	SAIF_DUMP();
	return 0;
}

/*
 * Should only be called when port is inactive.
 * although can be called multiple times by upper layers.
 */
static int mxs_saif_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *cpu_dai)
{
	u32 scr, stat;
	struct mxs_saif *saif_select = (struct mxs_saif *)cpu_dai->private_data;
	if (((saif_select->stream_mapping == PLAYBACK_SAIF0_CAPTURE_SAIF1) && \
		(substream->stream == SNDRV_PCM_STREAM_PLAYBACK)) || \
		((saif_select->stream_mapping == PLAYBACK_SAIF1_CAPTURE_SAIF0) \
		&& (substream->stream == SNDRV_PCM_STREAM_CAPTURE))) {
		scr = __raw_readl(SAIF0_CTRL);
		stat = __raw_readl(SAIF0_STAT);
	} else {
		scr = __raw_readl(SAIF1_CTRL);
		stat = __raw_readl(SAIF1_STAT);
	}
	/* cant change any parameters when SAIF is running */
	/* DAI data (word) size */
	scr &= ~BM_SAIF_CTRL_WORD_LENGTH;
	scr &= ~BM_SAIF_CTRL_BITCLK_48XFS_ENABLE;
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		scr |= BF_SAIF_CTRL_WORD_LENGTH(0);
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		scr |= BF_SAIF_CTRL_WORD_LENGTH(4);
		scr |= BM_SAIF_CTRL_BITCLK_48XFS_ENABLE;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		scr |= BF_SAIF_CTRL_WORD_LENGTH(8);
		scr |= BM_SAIF_CTRL_BITCLK_48XFS_ENABLE;
		break;
	}
	/* Tx/Rx config */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* enable TX mode */
		scr &= ~BM_SAIF_CTRL_READ_MODE;
	} else {
		/* enable RX mode */
		scr |= BM_SAIF_CTRL_READ_MODE;
	}

	if (((saif_select->stream_mapping == PLAYBACK_SAIF0_CAPTURE_SAIF1) && \
		(substream->stream == SNDRV_PCM_STREAM_PLAYBACK)) || \
		((saif_select->stream_mapping == PLAYBACK_SAIF1_CAPTURE_SAIF0) \
		&& (substream->stream == SNDRV_PCM_STREAM_CAPTURE)))
		__raw_writel(scr, SAIF0_CTRL);
	else
		__raw_writel(scr, SAIF1_CTRL);
	return 0;
}

static int mxs_saif_prepare(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *cpu_dai)
{
	struct mxs_saif *saif_select = (struct mxs_saif *)cpu_dai->private_data;
	if (((saif_select->stream_mapping == PLAYBACK_SAIF0_CAPTURE_SAIF1) && \
		(substream->stream == SNDRV_PCM_STREAM_PLAYBACK)) || \
		((saif_select->stream_mapping == PLAYBACK_SAIF1_CAPTURE_SAIF0) \
		&& (substream->stream == SNDRV_PCM_STREAM_CAPTURE)))
		__raw_writel(BM_SAIF_CTRL_CLKGATE, SAIF0_CTRL_CLR);
	else
		__raw_writel(BM_SAIF_CTRL_CLKGATE, SAIF1_CTRL_CLR);
	SAIF_DUMP();
	return 0;
}

static int mxs_saif_trigger(struct snd_pcm_substream *substream, int cmd,
				struct snd_soc_dai *cpu_dai)
{
	void __iomem *reg;
	struct mxs_saif *saif_select = (struct mxs_saif *)cpu_dai->private_data;
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:

	 if (((saif_select->stream_mapping == PLAYBACK_SAIF0_CAPTURE_SAIF1) && \
		(substream->stream == SNDRV_PCM_STREAM_PLAYBACK)) || \
		((saif_select->stream_mapping == PLAYBACK_SAIF1_CAPTURE_SAIF0) \
		&& (substream->stream == SNDRV_PCM_STREAM_CAPTURE)))
			reg = (void __iomem *)SAIF0_DATA;
		else
			reg = (void __iomem *)SAIF1_DATA;

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			/*write a data to saif data register to trigger
				the transfer*/
			__raw_writel(0, reg);
		else
			/*read a data from saif data register to trigger
				the receive*/
			__raw_readl(reg);
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		break;
	default:
		return -EINVAL;
	}
	SAIF_DUMP();
	return 0;
}

static void mxs_saif_shutdown(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *cpu_dai)
{
	struct mxs_saif *saif_select = (struct mxs_saif *)cpu_dai->private_data;
	/* shutdown SAIF if neither Tx or Rx is active */
	if (cpu_dai->playback.active || cpu_dai->capture.active)
		return;

	if (((saif_select->stream_mapping == PLAYBACK_SAIF0_CAPTURE_SAIF1) && \
		(substream->stream == SNDRV_PCM_STREAM_PLAYBACK)) || \
		((saif_select->stream_mapping == PLAYBACK_SAIF1_CAPTURE_SAIF0) \
		&& (substream->stream == SNDRV_PCM_STREAM_CAPTURE)))
		if (--saif_active[SAIF0_PORT] > 1)
			return;

	if (((saif_select->stream_mapping == PLAYBACK_SAIF0_CAPTURE_SAIF1) && \
		(substream->stream == SNDRV_PCM_STREAM_CAPTURE)) || \
		((saif_select->stream_mapping == PLAYBACK_SAIF1_CAPTURE_SAIF0) \
		&& (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)))
		if (--saif_active[SAIF1_PORT])
			return;
}

#ifdef CONFIG_PM
static int mxs_saif_suspend(struct snd_soc_dai *dai)
{
	if (!dai->active)
		return 0;
	/* do we need to disable any clocks? */
	return 0;
}

static int mxs_saif_resume(struct snd_soc_dai *dai)
{
	if (!dai->active)
		return 0;
	/* do we need to enable any clocks? */
	return 0;
}
#else
#define mxs_saif_suspend	NULL
#define mxs_saif_resume	NULL
#endif

static int fifo_err_counter;

static irqreturn_t saif0_irq(int irq, void *dev_id)
{
	if (fifo_err_counter++ % 100 == 0)
		printk(KERN_ERR "saif0_irq SAIF_STAT %x SAIF_CTRL %x fifo_errs=\
			%d\n",
		       __raw_readl(SAIF0_STAT),
		       __raw_readl(SAIF0_CTRL),
		       fifo_err_counter);
	__raw_writel(BM_SAIF_STAT_FIFO_UNDERFLOW_IRQ | \
			BM_SAIF_STAT_FIFO_OVERFLOW_IRQ, SAIF0_STAT_CLR);
	return IRQ_HANDLED;
}

static irqreturn_t saif1_irq(int irq, void *dev_id)
{
	if (fifo_err_counter++ % 100 == 0)
		printk(KERN_ERR "saif1_irq SAIF_STAT %x SAIF_CTRL %x \
			fifo_errs=%d\n",
		       __raw_readl(SAIF1_STAT),
		       __raw_readl(SAIF1_CTRL),
		       fifo_err_counter);
	__raw_writel(BM_SAIF_STAT_FIFO_UNDERFLOW_IRQ | \
			BM_SAIF_STAT_FIFO_OVERFLOW_IRQ, SAIF1_STAT_CLR);
	return IRQ_HANDLED;
}

static int mxs_saif_probe(struct platform_device *pdev, struct snd_soc_dai *dai)
{
	if (request_irq(IRQ_SAIF0, saif0_irq, 0, "saif0", dai)) {
		printk(KERN_ERR "%s: failure requesting irq %s\n",
		       __func__, "saif0");
		return -EBUSY;
	}

	if (request_irq(IRQ_SAIF1, saif1_irq, 0, "saif1", dai)) {
		printk(KERN_ERR "%s: failure requesting irq %s\n",
		       __func__, "saif1");
		return -EBUSY;
	}
	return 0;
}

static void mxs_saif_remove(struct platform_device *pdev,
			   struct snd_soc_dai *dai)
{
	free_irq(IRQ_SAIF0, dai);
	free_irq(IRQ_SAIF1, dai);
}

#define MXS_SAIF_RATES \
	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | \
	SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | \
	SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
	SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | \
	SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 | \
	SNDRV_PCM_RATE_192000)

#define MXS_SAIF_FORMATS \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
	SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_ops mxs_saif_dai_ops = {
	.startup = mxs_saif_startup,
	.shutdown = mxs_saif_shutdown,
	.trigger = mxs_saif_trigger,
	.prepare = mxs_saif_prepare,
	.hw_params = mxs_saif_hw_params,
	.set_sysclk = mxs_saif_set_dai_sysclk,
	.set_clkdiv = mxs_saif_set_dai_clkdiv,
	.set_fmt = mxs_saif_set_dai_fmt,
};

struct snd_soc_dai mxs_saif_dai[] = {
	{
	.name = "mxs-saif",
	.probe = mxs_saif_probe,
	.remove = mxs_saif_remove,
	.suspend = mxs_saif_suspend,
	.resume = mxs_saif_resume,
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = MXS_SAIF_RATES,
		.formats = MXS_SAIF_FORMATS,
	},
	.capture = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = MXS_SAIF_RATES,
		.formats = MXS_SAIF_FORMATS,
	},
	.ops = &mxs_saif_dai_ops,
	.private_data = &mxs_saif_en,
	}
};
EXPORT_SYMBOL_GPL(mxs_saif_dai);

static int __init mxs_saif_init(void)
{
	return snd_soc_register_dais(mxs_saif_dai, ARRAY_SIZE(mxs_saif_dai));
}

static void __exit mxs_saif_exit(void)
{
	snd_soc_unregister_dais(mxs_saif_dai, ARRAY_SIZE(mxs_saif_dai));
}

module_init(mxs_saif_init);
module_exit(mxs_saif_exit);
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("i.MX28 ASoC I2S driver");
MODULE_LICENSE("GPL");
