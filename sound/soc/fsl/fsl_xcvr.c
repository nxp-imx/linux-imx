// SPDX-License-Identifier: GPL-2.0
// Copyright 2019 NXP

#include <linux/bitrev.h>
#include <linux/clk.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <sound/dmaengine_pcm.h>

#include "fsl_xcvr.h"
#include "imx-pcm.h"

struct fsl_xcvr {
	struct platform_device *pdev;
	struct regmap *regmap;
	struct clk *ipg_clk;
	struct clk *phy_clk;
	struct clk *spba_clk;
	struct reset_control *reset;
	const char *fw_name;
	u8 streams;
	u32 mode;
	void __iomem *ram_addr;
	struct snd_dmaengine_dai_dma_data dma_prms_rx;
	struct snd_dmaengine_dai_dma_data dma_prms_tx;
	struct snd_aes_iec958 rx_iec958;
	struct snd_aes_iec958 tx_iec958;
};

static const u32 fsl_xcvr_earc_channels[] = { 1, 2, 8, 16, 32, }; /* one bit 6, 12 ? */
static const struct snd_pcm_hw_constraint_list fsl_xcvr_earc_channels_constr = {
	.count = ARRAY_SIZE(fsl_xcvr_earc_channels),
	.list = fsl_xcvr_earc_channels,
};

static const u32 fsl_xcvr_earc_bits[] = { 24, };
static const struct snd_pcm_hw_constraint_list fsl_xcvr_earc_bits_constr = {
	.count = ARRAY_SIZE(fsl_xcvr_earc_bits),
	.list = fsl_xcvr_earc_bits,
};

static const u32 fsl_xcvr_earc_rates[] = {
	32000, 44100, 48000, 64000, 88200, 96000,
	128000, 176400, 192000, 256000, 352800, 384000,
	512000, 705600, 768000, 1024000, 1411200, 1536000,
};
static const struct snd_pcm_hw_constraint_list fsl_xcvr_earc_rates_constr = {
	.count = ARRAY_SIZE(fsl_xcvr_earc_rates),
	.list = fsl_xcvr_earc_rates,
};

static const u32 fsl_xcvr_spdif_channels[] = { 2, };
static const u32 fsl_xcvr_spdif_bits[] = { 16, 20, 24, };

/*
 * pll_phy:  pll 0; phy 1;
*/
static int fsl_xcvr_phy_write(struct fsl_xcvr *xcvr, int reg, int data, int pll_phy)
{
	int val;
	int idx0, idx1;

	idx0 = pll_phy > 0 ? 26 : 24;
	idx1 = pll_phy > 0 ? 24 : 26;

	regmap_write(xcvr->regmap, FSL_XCVR_PHY_AI_WDATA, data);
	regmap_read(xcvr->regmap, FSL_XCVR_PHY_AI_CTRL, &val);

	val = ((~(val & BIT(idx0))) & BIT(idx0)) | (val & BIT(idx1)) | reg | BIT(15);

	regmap_write(xcvr->regmap, FSL_XCVR_PHY_AI_CTRL, val);

	do {
		regmap_read(xcvr->regmap, FSL_XCVR_PHY_AI_CTRL, &val);
	} while ((val & BIT(idx0)) != ((val & BIT(idx0 + 1)) >> 1));

	return 0;
}

static int fsl_xcvr_prepare(struct snd_pcm_substream *substream,
			    struct snd_soc_dai *dai)
{
	struct fsl_xcvr *xcvr = snd_soc_dai_get_drvdata(dai);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	u32 m_ctl = 0, v_ctl = 0, m_isr = 0, v_isr = 0;
	int ret = 0;

	ret = regmap_update_bits(xcvr->regmap, FSL_XCVR_EXT_IER0,
				 FSL_XCVR_IRQ_EARC_ALL, FSL_XCVR_IRQ_EARC_ALL);
	if (ret < 0) {
		dev_err(dai->dev, "Error while setting IER0: %d\n", ret);
		return ret;
	}

	/* configure EXT_CTRL */
	switch (xcvr->mode & FSL_XCVR_AMODE_MASK) {
	case FSL_XCVR_AMODE_SPDIF:
		/* set SPDIF MODE */
		m_ctl |= FSL_XCVR_EXT_CTRL_SPDIF_MODE;
		v_ctl |= FSL_XCVR_EXT_CTRL_SPDIF_MODE;
		m_isr |= FSL_XCVR_ISR_SET_SPDIF_MODE(tx);
		v_isr |= FSL_XCVR_ISR_SET_SPDIF_MODE(tx);
		if (xcvr->streams == 3) { // both Tx and Rx are in use
			m_isr |= FSL_XCVR_ISR_DMAC_SPARE_INT;
			v_isr |= FSL_XCVR_ISR_DMAC_SPARE_INT;
		}
		break;
	case FSL_XCVR_AMODE_ARC:
		/* Enable ISR */
		break;
	case FSL_XCVR_AMODE_EARC:
		/* clear CMDC RESET */
		m_ctl |= FSL_XCVR_EXT_CTRL_CMDC_RESET(tx);
		/* set TX_RX_MODE */
		m_ctl |= FSL_XCVR_EXT_CTRL_TX_RX_MODE;
		v_ctl |= (tx ? FSL_XCVR_EXT_CTRL_TX_RX_MODE : 0);
		break;
	}

	/* clear DPATH RESET */
	m_ctl |= FSL_XCVR_EXT_CTRL_DPTH_RESET(tx);
	ret = regmap_update_bits(xcvr->regmap, FSL_XCVR_EXT_CTRL, m_ctl, v_ctl);
	if (ret < 0) {
		dev_err(dai->dev, "Error while setting EXT_CTRL: %d\n", ret);
		return ret;
	}

	ret = regmap_update_bits(xcvr->regmap, FSL_XCVR_ISR_SET, m_isr, v_isr);
	if (ret < 0) {
		dev_err(dai->dev, "Error while setting MO ISR: %d\n", ret);
		return ret;
	}

	return 0;
}

static int fsl_xcvr_constr(const struct snd_pcm_substream *substream,
			   const struct snd_pcm_hw_constraint_list *bits,
			   const struct snd_pcm_hw_constraint_list *channels,
			   const struct snd_pcm_hw_constraint_list *rates)
{
	struct snd_pcm_runtime *rt = substream->runtime;
	int ret;

	ret = snd_pcm_hw_constraint_list(rt, 0, SNDRV_PCM_HW_PARAM_SAMPLE_BITS,
					 bits);
	if (ret < 0)
		return ret;

	ret = snd_pcm_hw_constraint_list(rt, 0, SNDRV_PCM_HW_PARAM_CHANNELS,
					 channels);
	if (ret < 0)
		return ret;

	ret = snd_pcm_hw_constraint_list(rt, 0, SNDRV_PCM_HW_PARAM_RATE,
					 rates);
	if (ret < 0)
		return ret;

	return 0;
}

static int fsl_xcvr_startup(struct snd_pcm_substream *substream,
			    struct snd_soc_dai *dai)
{
	struct fsl_xcvr *xcvr = snd_soc_dai_get_drvdata(dai);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	int ret = 0;

	/* check stream direction is enabled in DTS */
	if (!(xcvr->mode & BIT(substream->stream))) {
		dev_err(dai->dev, "%sX not supported\n", tx ? "T" : "R");
		return -EINVAL;
	}

	if (xcvr->streams & BIT(substream->stream)) {
		dev_err(dai->dev, "%sX busy\n", tx ? "T" : "R");
		return -EBUSY;
	}

	switch (xcvr->mode & FSL_XCVR_AMODE_MASK) {
	case FSL_XCVR_AMODE_SPDIF:
	case FSL_XCVR_AMODE_ARC:
		ret = 0; /* @todo */
		break;
	case FSL_XCVR_AMODE_EARC:
		ret = fsl_xcvr_constr(substream, &fsl_xcvr_earc_bits_constr,
				      &fsl_xcvr_earc_channels_constr,
				      &fsl_xcvr_earc_rates_constr);
		break;
	}
	if (ret < 0)
		return ret;

	xcvr->streams |= BIT(substream->stream);

	switch (xcvr->mode & FSL_XCVR_AMODE_MASK) {
	case FSL_XCVR_AMODE_SPDIF:
		if (tx) {
			fsl_xcvr_phy_write(xcvr, 0x54, 0x1, 0);
			fsl_xcvr_phy_write(xcvr, 0x4, 0x1, 1);
			fsl_xcvr_phy_write(xcvr, 0x0, 0x28, 0);
			fsl_xcvr_phy_write(xcvr, 0x20, 0x60, 0);
			fsl_xcvr_phy_write(xcvr, 0x30, 0x64, 0);
			fsl_xcvr_phy_write(xcvr, 0x4, 0x1006000, 0);
			udelay(25);
			fsl_xcvr_phy_write(xcvr, 0x8, 0x2000, 0);
			udelay(100);
			fsl_xcvr_phy_write(xcvr, 0x40, 0x5, 0);

			fsl_xcvr_phy_write(xcvr, 0x4,  0x20, 1);
			fsl_xcvr_phy_write(xcvr, 0x74, 0x4000, 1);
		}
		break;
	case FSL_XCVR_AMODE_EARC:
		if (tx) {
			fsl_xcvr_phy_write(xcvr, 0x54, 0x1, 0);
			fsl_xcvr_phy_write(xcvr, 0x4, 0x1, 1);
			fsl_xcvr_phy_write(xcvr, 0x0, 0x28, 0);
			fsl_xcvr_phy_write(xcvr, 0x20, 0x60, 0);
			fsl_xcvr_phy_write(xcvr, 0x30, 0x64, 0);
			fsl_xcvr_phy_write(xcvr, 0x4, 0x1006000, 0);
			udelay(25);
			fsl_xcvr_phy_write(xcvr, 0x8, 0x2000, 0);
			udelay(100);
			fsl_xcvr_phy_write(xcvr, 0x40, 0x1, 0);

			fsl_xcvr_phy_write(xcvr, 0x4,  0x20, 1);
			fsl_xcvr_phy_write(xcvr, 0x74, 0x4000, 1);
		}
		break;
	default:
		break;
	}

	return 0;
}

static void fsl_xcvr_shutdown(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct fsl_xcvr *xcvr = snd_soc_dai_get_drvdata(dai);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	u32 mask = 0, val = 0;
	int ret;

	xcvr->streams &= ~BIT(substream->stream);
	ret = regmap_update_bits(xcvr->regmap, FSL_XCVR_EXT_IER0,
				 FSL_XCVR_IRQ_EARC_ALL, 0);
	if (ret < 0) {
		dev_err(dai->dev, "Failed to set IER0: %d\n", ret);
		return;
	}

	switch (xcvr->mode & FSL_XCVR_AMODE_MASK) {
	case FSL_XCVR_AMODE_SPDIF:
		/* clear SPDIF MODE */
		mask |= FSL_XCVR_EXT_CTRL_SPDIF_MODE;
		break;
	case FSL_XCVR_AMODE_EARC:
		/* set CMDC RESET */
		mask |= FSL_XCVR_EXT_CTRL_CMDC_RESET(tx);
		val  |= FSL_XCVR_EXT_CTRL_CMDC_RESET(tx);
		break;
	}

	/* set DPATH RESET */
	mask |= FSL_XCVR_EXT_CTRL_DPTH_RESET(tx);
	val  |= FSL_XCVR_EXT_CTRL_DPTH_RESET(tx);

	ret = regmap_update_bits(xcvr->regmap, FSL_XCVR_EXT_CTRL, mask, val);
	if (ret < 0) {
		dev_err(dai->dev, "Err setting DPATH RESET: %d\n", ret);
		return;
	}
}

static int fsl_xcvr_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params,
			      struct snd_soc_dai *dai)
{
	struct fsl_xcvr *xcvr = snd_soc_dai_get_drvdata(dai);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	u32 mask, val, r = params_rate(params), ch = params_channels(params);
	int ret;

	if (tx) {
		mask  = FSL_XCVR_CS_DATA_0_FS_MASK;
		mask |= FSL_XCVR_CS_DATA_0_CH_MASK;

		switch (r) {
		case 32000:  val = FSL_XCVR_CS_DATA_0_FS_32000;  break;
		case 44100:  val = FSL_XCVR_CS_DATA_0_FS_44100;  break;
		case 48000:  val = FSL_XCVR_CS_DATA_0_FS_48000;  break;
		case 64000:  val = FSL_XCVR_CS_DATA_0_FS_64000;  break;
		case 88200:  val = FSL_XCVR_CS_DATA_0_FS_88200;  break;
		case 96000:  val = FSL_XCVR_CS_DATA_0_FS_96000;  break;
		case 176400: val = FSL_XCVR_CS_DATA_0_FS_176400; break;
		case 192000: val = FSL_XCVR_CS_DATA_0_FS_192000; break;
		default:
			dev_err(dai->dev, "Unsupported rate: %u\n", r);
			return -EINVAL;
		}
		val |= FSL_XCVR_CS_DATA_0_CH_UMLPCM;

		/* Update TX CS data bits 0 */
		ret = regmap_update_bits(xcvr->regmap, FSL_XCVR_TX_CS_DATA_0,
					 mask, val);
		if (ret < 0) {
			dev_err(dai->dev, "Failed to set TX CS 0: %d\n", ret);
			return ret;
		}

		switch (ch) {
		case 2:  val = FSL_XCVR_CS_DATA_1_CH_2;  break;
		case 8:  val = FSL_XCVR_CS_DATA_1_CH_8;  break;
		case 16: val = FSL_XCVR_CS_DATA_1_CH_16; break;
		case 32: val = FSL_XCVR_CS_DATA_1_CH_32; break;
		default:
			dev_err(dai->dev, "Unsupported channels: %u\n", ch);
			return -EINVAL;
		}

		/* Update TX CS data bits 1 */
		ret = regmap_update_bits(xcvr->regmap, FSL_XCVR_TX_CS_DATA_1,
					 FSL_XCVR_CS_DATA_1_CH_MASK, val);
		if (ret < 0) {
			dev_err(dai->dev, "Failed to set TX CS 1: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int fsl_xcvr_trigger(struct snd_pcm_substream *substream, int cmd,
			    struct snd_soc_dai *dai)
{
	struct fsl_xcvr *xcvr = snd_soc_dai_get_drvdata(dai);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	int ret;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (tx) {
			switch (xcvr->mode & FSL_XCVR_AMODE_MASK) {
			case FSL_XCVR_AMODE_EARC:
				/* set isr_cmdc_tx_en, w1c */
				ret = regmap_update_bits(xcvr->regmap,
							 FSL_XCVR_ISR_SET,
							 FSL_XCVR_ISR_CMDC_TX_EN,
							 FSL_XCVR_ISR_CMDC_TX_EN);
				if (ret < 0) {
					dev_err(dai->dev,
						"err updating isr %d\n", ret);
					return ret;
				}
				/* fall through */

			case FSL_XCVR_AMODE_SPDIF:
				ret = regmap_update_bits(xcvr->regmap,
					 FSL_XCVR_TX_DPTH_CTRL_SET,
					 FSL_XCVR_TX_DPTH_CTRL_STRT_DATA_TX,
					 FSL_XCVR_TX_DPTH_CTRL_STRT_DATA_TX);
				if (ret < 0) {
					dev_err(dai->dev, "Failed to set TX_DPTH_CTRL_STRT_DATA_TX: %d\n", ret);
					return ret;
				}
				break;
			}
		} else {
			/* Clear RX FIFO */
			ret = regmap_update_bits(xcvr->regmap,
					 FSL_XCVR_RX_DPTH_CTRL_SET,
					 FSL_XCVR_RX_DPTH_CTRL_CLR_RX_FIFO,
					 FSL_XCVR_RX_DPTH_CTRL_CLR_RX_FIFO);
			if (ret < 0) {
				dev_err(dai->dev, "Failed to clear RX FIFO: %d\n", ret);
				return ret;
			}

			/* Flip RX FIFO bits */
			ret = regmap_update_bits(xcvr->regmap,
					 FSL_XCVR_RX_DPTH_CTRL_SET,
					 FSL_XCVR_RX_DPTH_CTRL_STORE_FMT,
					 FSL_XCVR_RX_DPTH_CTRL_STORE_FMT);
			if (ret < 0) {
				dev_err(dai->dev, "Failed to set reverse bit in RX FIFO: %d\n", ret);
				return ret;
			}
		}

		/* enable DMA RD/WR */
		ret = regmap_update_bits(xcvr->regmap, FSL_XCVR_EXT_CTRL,
					 FSL_XCVR_EXT_CTRL_DMA_DIS(tx), 0);
		if (ret < 0) {
			dev_err(dai->dev, "Failed to enable DMA: %d\n", ret);
			return ret;
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		/* disable DMA RD/WR */
		ret = regmap_update_bits(xcvr->regmap, FSL_XCVR_EXT_CTRL,
					 FSL_XCVR_EXT_CTRL_DMA_DIS(tx),
					 FSL_XCVR_EXT_CTRL_DMA_DIS(tx));
		if (ret < 0) {
			dev_err(dai->dev, "Failed to disable DMA: %d\n", ret);
			return ret;
		}

		if (tx) {
			switch (xcvr->mode & FSL_XCVR_AMODE_MASK) {
			case FSL_XCVR_AMODE_SPDIF:
				ret = regmap_update_bits(xcvr->regmap,
					 FSL_XCVR_TX_DPTH_CTRL_CLR,
					 FSL_XCVR_TX_DPTH_CTRL_STRT_DATA_TX,
					 FSL_XCVR_TX_DPTH_CTRL_STRT_DATA_TX);
				if (ret < 0) {
					dev_err(dai->dev, "Failed to clr TX_DPTH_CTRL_STRT_DATA_TX: %d\n", ret);
					return ret;
				}
				/* fall through ...*/
			case FSL_XCVR_AMODE_EARC:
				/* clear ISR_CMDC_TX_EN, W1C */
				ret = regmap_update_bits(xcvr->regmap,
							 FSL_XCVR_ISR_CLR,
							 FSL_XCVR_ISR_CMDC_TX_EN,
							 FSL_XCVR_ISR_CMDC_TX_EN);
				if (ret < 0) {
					dev_err(dai->dev,
						"Err updating ISR %d\n", ret);
					return ret;
				}
				break;
			}
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int fsl_xcvr_load_firmware(struct fsl_xcvr *xcvr)
{
	struct device *dev = &xcvr->pdev->dev;
	const struct firmware *fw;
	int ret = 0, rem, off, out, page = 0, size = FSL_XCVR_REG_OFFSET;
	u32 mask, val;

	ret = request_firmware(&fw, xcvr->fw_name, dev);
	if (ret) {
		dev_err(dev, "failed to request firmware.\n");
		return ret;
	}

	rem = fw->size;

	/* RAM is 20KiB => max 10 pages 2KiB each */
	for (page = 0; page < 10; page++)
	{
		ret = regmap_update_bits(xcvr->regmap, FSL_XCVR_EXT_CTRL,
					 FSL_XCVR_EXT_CTRL_PAGE_MASK,
					 FSL_XCVR_EXT_CTRL_PAGE(page));
		if (ret < 0) {
			dev_err(dev, "FW: failed to set page %d, err=%d \n",
				page, ret);
			goto err_firmware;
		}

		off = page * size;
		out = min(rem, size);
		/* IPG clock is assumed to be running, otherwise it will hang */
		if (out > 0) {
			/* write firmware into code memory */
			memcpy_toio(xcvr->ram_addr, fw->data + off, out);
			rem -= out;
			if (rem == 0) {
				/* last part of firmware written */
				/* clean remaining part of code memory page */
				memset_io(xcvr->ram_addr + out, 0, size - out);
			}
		} else {
			/* clean current page, including data memory */
			memset_io(xcvr->ram_addr, 0, size);
		}
	};

err_firmware:
	release_firmware(fw);
	if (ret < 0)
		return ret;

	/* configure watermarks */
	mask = FSL_XCVR_EXT_CTRL_RX_FWM_MASK | FSL_XCVR_EXT_CTRL_TX_FWM_MASK;
	val  = FSL_XCVR_EXT_CTRL_RX_FWM(FSL_XCVR_FIFO_WMK_RX);
	val |= FSL_XCVR_EXT_CTRL_TX_FWM(FSL_XCVR_FIFO_WMK_TX);
	/* disable DMA RD/WR */
	mask |= FSL_XCVR_EXT_CTRL_DMA_RD_DIS | FSL_XCVR_EXT_CTRL_DMA_WR_DIS;
	val |= FSL_XCVR_EXT_CTRL_DMA_RD_DIS | FSL_XCVR_EXT_CTRL_DMA_WR_DIS;
	ret = regmap_update_bits(xcvr->regmap, FSL_XCVR_EXT_CTRL, mask, val);
	if (ret < 0) {
		dev_err(dev, "Failed to set watermarks: %d\n", ret);
		return ret;
	}

	return 0;
}

static int fsl_xcvr_type_iec958_info(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_IEC958;
	uinfo->count = 1;

	return 0;
}

static int fsl_xcvr_type_bytes_info(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	uinfo->count = FIELD_SIZEOF(struct snd_aes_iec958, status);

	return 0;
}

static int fsl_xcvr_rx_cs_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *dai = snd_kcontrol_chip(kcontrol);
	struct fsl_xcvr *xcvr = snd_soc_dai_get_drvdata(dai);

	memcpy(ucontrol->value.iec958.status, xcvr->rx_iec958.status, 24);

	return 0;
}

static int fsl_xcvr_tx_cs_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *dai = snd_kcontrol_chip(kcontrol);
	struct fsl_xcvr *xcvr = snd_soc_dai_get_drvdata(dai);

	memcpy(ucontrol->value.iec958.status, xcvr->tx_iec958.status, 24);

	return 0;
}

static int fsl_xcvr_tx_cs_put(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *dai = snd_kcontrol_chip(kcontrol);
	struct fsl_xcvr *xcvr = snd_soc_dai_get_drvdata(dai);

	memcpy(xcvr->tx_iec958.status, ucontrol->value.iec958.status, 24);

	return 0;
}

static struct snd_kcontrol_new fsl_xcvr_rx_ctls[] = {
	/* Channel status controller */
	{
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.name = SNDRV_CTL_NAME_IEC958("", CAPTURE, DEFAULT),
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.info = fsl_xcvr_type_iec958_info,
		.get = fsl_xcvr_rx_cs_get,
	},
	/* Capture channel status, bytes */
	{
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.name = "Capture Channel Status",
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.info = fsl_xcvr_type_bytes_info,
		.get = fsl_xcvr_rx_cs_get,
	},
};

static struct snd_kcontrol_new fsl_xcvr_tx_ctls[] = {
	/* Channel status controller */
	{
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.name = SNDRV_CTL_NAME_IEC958("", PLAYBACK, DEFAULT),
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = fsl_xcvr_type_iec958_info,
		.get = fsl_xcvr_tx_cs_get,
		.put = fsl_xcvr_tx_cs_put,
	},
};

static struct snd_soc_dai_ops fsl_xcvr_dai_ops = {
	.prepare = fsl_xcvr_prepare,
	.startup = fsl_xcvr_startup,
	.shutdown = fsl_xcvr_shutdown,
	.trigger = fsl_xcvr_trigger,
	.hw_params = fsl_xcvr_hw_params,
};

static int fsl_xcvr_dai_probe(struct snd_soc_dai *dai)
{
	struct fsl_xcvr *xcvr = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai, &xcvr->dma_prms_tx, &xcvr->dma_prms_rx);
	snd_soc_dai_set_drvdata(dai, xcvr);

	if (xcvr->mode & FSL_XCVR_DMODE_TX)
		snd_soc_add_dai_controls(dai, fsl_xcvr_tx_ctls,
					 ARRAY_SIZE(fsl_xcvr_tx_ctls));
	if (xcvr->mode & FSL_XCVR_DMODE_RX)
		snd_soc_add_dai_controls(dai, fsl_xcvr_rx_ctls,
					 ARRAY_SIZE(fsl_xcvr_rx_ctls));
	return 0;
}

static const struct snd_soc_pcm_stream playback = {
	.stream_name = "CPU-Playback",
	.channels_min = 1,
	.channels_max = 32,
	.rate_min = 32000,
	.rate_max = 1536000,
	.rates = SNDRV_PCM_RATE_KNOT,
	.formats = FSL_XCVR_FORMATS,
};

static const struct snd_soc_pcm_stream capture = {
	.stream_name = "CPU-Capture",
	.channels_min = 1,
	.channels_max = 32,
	.rate_min = 32000,
	.rate_max = 1536000,
	.rates = SNDRV_PCM_RATE_KNOT,
	.formats = FSL_XCVR_FORMATS,
};

static struct snd_soc_dai_driver fsl_xcvr_dai = {
	.probe  = fsl_xcvr_dai_probe,
	.ops = &fsl_xcvr_dai_ops,
};

static const struct snd_soc_component_driver fsl_xcvr_comp = {
	.name = "fsl-xcvr-dai",
};

static const struct reg_default fsl_xcvr_reg_defaults[] = {
	{ FSL_XCVR_VERSION,	0x00000000 },
	{ FSL_XCVR_EXT_CTRL,	0xF8204040 },
	{ FSL_XCVR_EXT_STATUS,	0x00000000 },
	{ FSL_XCVR_EXT_IER0,	0x00000000 },
	{ FSL_XCVR_EXT_IER1,	0x00000000 },
	{ FSL_XCVR_EXT_ISR,	0x00000000 },
	{ FSL_XCVR_EXT_ISR_SET,	0x00000000 },
	{ FSL_XCVR_EXT_ISR_CLR,	0x00000000 },
	{ FSL_XCVR_EXT_ISR_TOG,	0x00000000 },
	{ FSL_XCVR_IER,		0x00000000 },
	{ FSL_XCVR_ISR,		0x00000000 },
	{ FSL_XCVR_ISR_SET,	0x00000000 },
	{ FSL_XCVR_ISR_CLR,	0x00000000 },
	{ FSL_XCVR_ISR_TOG,	0x00000000 },
	{ FSL_XCVR_RX_DPTH_CTRL,	0x00002C89 },
	{ FSL_XCVR_RX_DPTH_CTRL_SET,	0x00002C89 },
	{ FSL_XCVR_RX_DPTH_CTRL_CLR,	0x00002C89 },
	{ FSL_XCVR_RX_DPTH_CTRL_TOG,	0x00002C89 },
	{ FSL_XCVR_TX_DPTH_CTRL,	0x00000000 },
	{ FSL_XCVR_TX_DPTH_CTRL_SET,	0x00000000 },
	{ FSL_XCVR_TX_DPTH_CTRL_CLR,	0x00000000 },
	{ FSL_XCVR_TX_DPTH_CTRL_TOG,	0x00000000 },
	{ FSL_XCVR_TX_CS_DATA_0,	0x00000000 },
	{ FSL_XCVR_TX_CS_DATA_1,	0x00000000 },
	{ FSL_XCVR_TX_CS_DATA_2,	0x00000000 },
	{ FSL_XCVR_TX_CS_DATA_3,	0x00000000 },
	{ FSL_XCVR_TX_CS_DATA_4,	0x00000000 },
	{ FSL_XCVR_TX_CS_DATA_5,	0x00000000 },
	{ FSL_XCVR_DEBUG_REG_0,		0x00000000 },
	{ FSL_XCVR_DEBUG_REG_1,		0x00000000 },
};

static bool fsl_xcvr_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case FSL_XCVR_VERSION:
	case FSL_XCVR_EXT_CTRL:
	case FSL_XCVR_EXT_STATUS:
	case FSL_XCVR_EXT_IER0:
	case FSL_XCVR_EXT_IER1:
	case FSL_XCVR_EXT_ISR:
	case FSL_XCVR_EXT_ISR_SET:
	case FSL_XCVR_EXT_ISR_CLR:
	case FSL_XCVR_EXT_ISR_TOG:
	case FSL_XCVR_IER:
	case FSL_XCVR_ISR:
	case FSL_XCVR_ISR_SET:
	case FSL_XCVR_ISR_CLR:
	case FSL_XCVR_ISR_TOG:
	case FSL_XCVR_PHY_AI_CTRL:
	case FSL_XCVR_PHY_AI_CTRL_SET:
	case FSL_XCVR_PHY_AI_CTRL_CLR:
	case FSL_XCVR_PHY_AI_CTRL_TOG:
	case FSL_XCVR_PHY_AI_RDATA:
	case FSL_XCVR_CLK_CTRL:
	case FSL_XCVR_RX_DPTH_CTRL:
	case FSL_XCVR_RX_DPTH_CTRL_SET:
	case FSL_XCVR_RX_DPTH_CTRL_CLR:
	case FSL_XCVR_RX_DPTH_CTRL_TOG:
	case FSL_XCVR_TX_DPTH_CTRL:
	case FSL_XCVR_TX_DPTH_CTRL_SET:
	case FSL_XCVR_TX_DPTH_CTRL_CLR:
	case FSL_XCVR_TX_DPTH_CTRL_TOG:
	case FSL_XCVR_TX_CS_DATA_0:
	case FSL_XCVR_TX_CS_DATA_1:
	case FSL_XCVR_TX_CS_DATA_2:
	case FSL_XCVR_TX_CS_DATA_3:
	case FSL_XCVR_TX_CS_DATA_4:
	case FSL_XCVR_TX_CS_DATA_5:
	case FSL_XCVR_DEBUG_REG_0:
	case FSL_XCVR_DEBUG_REG_1:
		return true;
	default:
		return false;
	}
}

static bool fsl_xcvr_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case FSL_XCVR_EXT_CTRL:
	case FSL_XCVR_EXT_IER0:
	case FSL_XCVR_EXT_IER1:
	case FSL_XCVR_EXT_ISR:
	case FSL_XCVR_EXT_ISR_SET:
	case FSL_XCVR_EXT_ISR_CLR:
	case FSL_XCVR_EXT_ISR_TOG:
	case FSL_XCVR_IER:
	case FSL_XCVR_ISR_SET:
	case FSL_XCVR_ISR_CLR:
	case FSL_XCVR_ISR_TOG:
	case FSL_XCVR_PHY_AI_CTRL:
	case FSL_XCVR_PHY_AI_CTRL_SET:
	case FSL_XCVR_PHY_AI_CTRL_CLR:
	case FSL_XCVR_PHY_AI_CTRL_TOG:
	case FSL_XCVR_PHY_AI_WDATA:
	case FSL_XCVR_CLK_CTRL:
	case FSL_XCVR_RX_DPTH_CTRL:
	case FSL_XCVR_RX_DPTH_CTRL_SET:
	case FSL_XCVR_RX_DPTH_CTRL_CLR:
	case FSL_XCVR_RX_DPTH_CTRL_TOG:
	case FSL_XCVR_TX_DPTH_CTRL_SET:
	case FSL_XCVR_TX_DPTH_CTRL_CLR:
	case FSL_XCVR_TX_DPTH_CTRL_TOG:
	case FSL_XCVR_TX_CS_DATA_0:
	case FSL_XCVR_TX_CS_DATA_1:
	case FSL_XCVR_TX_CS_DATA_2:
	case FSL_XCVR_TX_CS_DATA_3:
	case FSL_XCVR_TX_CS_DATA_4:
	case FSL_XCVR_TX_CS_DATA_5:
		return true;
	default:
		return false;
	}
}

static bool fsl_xcvr_volatile_reg(struct device *dev, unsigned int reg)
{
	return fsl_xcvr_readable_reg(dev, reg);
}

static const struct regmap_config fsl_xcvr_regmap_cfg = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = FSL_XCVR_MAX_REG,
	.reg_defaults = fsl_xcvr_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(fsl_xcvr_reg_defaults),
	.readable_reg = fsl_xcvr_readable_reg,
	.volatile_reg = fsl_xcvr_volatile_reg,
	.writeable_reg = fsl_xcvr_writeable_reg,
	.cache_type = REGCACHE_FLAT,
};

static irqreturn_t irq0_isr(int irq, void *devid)
{
	struct fsl_xcvr *xcvr = (struct fsl_xcvr *)devid;
	struct device *dev = &xcvr->pdev->dev;
	struct regmap *regmap = xcvr->regmap;
	void __iomem *reg_ctrl, *reg_buff;
	u32 isr, val;

	regmap_read(regmap, FSL_XCVR_EXT_ISR, &isr);
	regmap_write(regmap, FSL_XCVR_EXT_ISR_CLR, isr);

	if (isr & FSL_XCVR_IRQ_NEW_CS) {
		dev_dbg(dev, "Received new CS block\n");
		/* Data RAM is 4KiB, last two pages: 8 and 9. Select page 8. */
		regmap_update_bits(xcvr->regmap, FSL_XCVR_EXT_CTRL,
				   FSL_XCVR_EXT_CTRL_PAGE_MASK,
				   FSL_XCVR_EXT_CTRL_PAGE(8));

		/* Find updated CS buffer */
		reg_ctrl = xcvr->ram_addr + FSL_XCVR_RX_CS_CTRL_0;
		reg_buff = xcvr->ram_addr + FSL_XCVR_RX_CS_BUFF_0;
		memcpy_fromio(&val, reg_ctrl, sizeof(val));
		if (!val) {
			reg_ctrl = xcvr->ram_addr + FSL_XCVR_RX_CS_CTRL_1;
			reg_buff = xcvr->ram_addr + FSL_XCVR_RX_CS_BUFF_1;
			memcpy_fromio(&val, reg_ctrl, sizeof(val));
		}

		if (val) {
			/* copy CS buffer */
			memcpy_fromio(&xcvr->rx_iec958.status, reg_buff,
				      sizeof(xcvr->rx_iec958.status));
			/* clear CS control register */
			memset_io(reg_ctrl, 0, sizeof(val));
		}
	}
	if (isr & FSL_XCVR_IRQ_NEW_UD)
		dev_dbg(dev, "Received new UD block\n");
	if (isr & FSL_XCVR_IRQ_MUTE)
		dev_dbg(dev, "HW mute bit detected\n");
	if (isr & FSL_XCVR_IRQ_FIFO_UOFL_ERR)
		dev_dbg(dev, "RX/TX FIFO full/empty\n");
	if (isr & FSL_XCVR_IRQ_ARC_MODE)
		dev_dbg(dev, "CMDC SM falls out of eARC mode\n");
	if (isr & FSL_XCVR_IRQ_DMA_RD_REQ)
		dev_dbg(dev, "DMA read request\n");
	if (isr & FSL_XCVR_IRQ_DMA_WR_REQ)
		dev_dbg(dev, "DMA write request\n");

	return IRQ_HANDLED;
}

static irqreturn_t irq1_isr(int irq, void *devid)
{
	struct fsl_xcvr *xcvr = (struct fsl_xcvr *)devid;
	struct device *dev = &xcvr->pdev->dev;

	dev_dbg(dev, "irq[1]: %d\n", irq);

	return IRQ_HANDLED;
}

static irqreturn_t irq2_isr(int irq, void *devid)
{
	struct fsl_xcvr *xcvr = (struct fsl_xcvr *)devid;
	struct device *dev = &xcvr->pdev->dev;

	dev_dbg(dev, "irq[2]: %d\n", irq);

	return IRQ_HANDLED;
}

static const struct of_device_id fsl_xcvr_dt_ids[] = {
	{ .compatible = "fsl,imx8mp-xcvr", },
	{ }
};
MODULE_DEVICE_TABLE(of, fsl_xcvr_dt_ids);

static int fsl_xcvr_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	const struct of_device_id *of_id;
	struct fsl_xcvr *xcvr;
	struct resource *res,
		ram_res = { .flags = IORESOURCE_MEM, },
		regs_res = { .flags = IORESOURCE_MEM, };
	void __iomem *regs;
	int ret, irq;

	of_id = of_match_device(fsl_xcvr_dt_ids, dev);
	if (!of_id)
		return -EINVAL;

	xcvr = devm_kzalloc(dev, sizeof(*xcvr), GFP_KERNEL);
	if (!xcvr)
		return -ENOMEM;

	xcvr->pdev = pdev;
	xcvr->ipg_clk = devm_clk_get(dev, "ipg");
	if (IS_ERR(xcvr->ipg_clk)) {
		dev_err(dev, "failed to get ipg clock\n");
		return PTR_ERR(xcvr->ipg_clk);
	}

	xcvr->phy_clk = devm_clk_get(dev, "phy");
	if (IS_ERR(xcvr->phy_clk)) {
		dev_err(dev, "failed to get phy clock\n");
		return PTR_ERR(xcvr->phy_clk);
	}

	xcvr->spba_clk = devm_clk_get(dev, "spba");
	if (IS_ERR(xcvr->spba_clk)) {
		dev_err(dev, "failed to get spba clock\n");
		return PTR_ERR(xcvr->spba_clk);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	ram_res.start = res->start;
	ram_res.end = res->start + FSL_XCVR_REG_OFFSET - 1;
	xcvr->ram_addr = devm_ioremap_resource(dev, &ram_res);
	if (IS_ERR(xcvr->ram_addr))
		return PTR_ERR(xcvr->ram_addr);

	regs_res.start = res->start + FSL_XCVR_REG_OFFSET;
	regs_res.end = res->end;
	regs = devm_ioremap_resource(dev, &regs_res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	xcvr->regmap = devm_regmap_init_mmio_clk(dev, NULL, regs,
						 &fsl_xcvr_regmap_cfg);
	if (IS_ERR(xcvr->regmap)) {
		dev_err(dev, "failed to init XCVR regmap: %ld\n",
			PTR_ERR(xcvr->regmap));
		return PTR_ERR(xcvr->regmap);
	}

	xcvr->reset = of_reset_control_get(np, NULL);

	ret = of_property_read_string(np, "fsl,xcvr-fw", &xcvr->fw_name);
	if (ret) {
		dev_err(dev, "failed to get fsl,xcvr-fw: %d\n", ret);
		return ret;
	}

	ret = of_property_read_u32(np, "fsl,xcvr-mode", &xcvr->mode);
	if (ret) {
		dev_err(dev, "failed to get fsl,xcvr-mode: %d\n", ret);
		return ret;
	}

	dev_info(dev, "fsl,xcvr-mode: %x\n", xcvr->mode);
	switch (xcvr->mode & FSL_XCVR_AMODE_MASK) {
	case FSL_XCVR_AMODE_SPDIF:
		dev_info(dev, "Setting SPDIF mode ...\n");
		break;
	case FSL_XCVR_AMODE_ARC:
		dev_info(dev, "Setting ARC mode ...\n");
		break;
	case FSL_XCVR_AMODE_EARC:
		dev_info(dev, "Setting EARC mode ...\n");
		break;
	default:
		dev_err(dev, "Unknown mode ...\n");
		break;
	}

	if (!(xcvr->mode & FSL_XCVR_DMODE_MASK)) {
		dev_err(dev, "RX, TX or both must be set, mode=%x\n",
			xcvr->mode);
		return -EINVAL;
	}

	if ((xcvr->mode & FSL_XCVR_AMODE_MASK) == FSL_XCVR_AMODE_RESERVED) {
		dev_err(dev, "wrong AMODE set, mode=%x\n", xcvr->mode);
		return -EINVAL;
	}

	/* get IRQs */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "no irq[0]: %d\n", irq);
		return irq;
	}

	ret = devm_request_irq(dev, irq, irq0_isr, 0, pdev->name, xcvr);
	if (ret) {
		dev_err(dev, "failed to claim IRQ0: %i\n", ret);
		return ret;
	}

	irq = platform_get_irq(pdev, 1);
	if (irq < 0) {
		dev_err(dev, "no irq[1]: %d\n", irq);
		return irq;
	}

	ret = devm_request_irq(dev, irq, irq1_isr, 0, pdev->name, xcvr);
	if (ret) {
		dev_err(dev, "failed to claim IRQ1: %i\n", ret);
		return ret;
	}

	irq = platform_get_irq(pdev, 2);
	if (irq < 0) {
		dev_err(dev, "no irq[2]: %d\n", irq);
		return irq;
	}

	ret = devm_request_irq(dev, irq, irq2_isr, 0, pdev->name, xcvr);
	if (ret) {
		dev_err(dev, "failed to claim IRQ2: %i\n", ret);
		return ret;
	}

	xcvr->dma_prms_rx.chan_name = "rx";
	xcvr->dma_prms_tx.chan_name = "tx";
	xcvr->dma_prms_rx.addr = res->start + FSL_XCVR_RX_FIFO_ADDR;
	xcvr->dma_prms_tx.addr = res->start + FSL_XCVR_TX_FIFO_ADDR;
	xcvr->dma_prms_rx.maxburst = FSL_XCVR_MAXBURST_RX;
	xcvr->dma_prms_tx.maxburst = FSL_XCVR_MAXBURST_TX;

	platform_set_drvdata(pdev, xcvr);
	pm_runtime_enable(dev);

	if (xcvr->mode & FSL_XCVR_DMODE_TX)
		fsl_xcvr_dai.playback = playback;

	if (xcvr->mode & FSL_XCVR_DMODE_RX)
		fsl_xcvr_dai.capture = capture;

	ret = devm_snd_soc_register_component(dev, &fsl_xcvr_comp,
					      &fsl_xcvr_dai, 1);
	if (ret) {
		dev_err(dev, "failed to register component %s\n",
			fsl_xcvr_comp.name);
		return ret;
	}

	ret = imx_pcm_platform_register(dev);
	if (ret)
		dev_err(dev, "failed to pcm register\n");

	return ret;
}

#ifdef CONFIG_PM
static int fsl_xcvr_runtime_suspend(struct device *dev)
{
	struct fsl_xcvr *xcvr = dev_get_drvdata(dev);
	int ret;

	/* Assert M0+ reset */
	ret = regmap_update_bits(xcvr->regmap, FSL_XCVR_EXT_CTRL,
				 FSL_XCVR_EXT_CTRL_CORE_RESET,
				 FSL_XCVR_EXT_CTRL_CORE_RESET);
	if (ret < 0)
		dev_err(dev, "Failed to assert M0+ core: %d\n", ret);

	regcache_cache_only(xcvr->regmap, true);

	clk_disable_unprepare(xcvr->spba_clk);
	clk_disable_unprepare(xcvr->phy_clk);
	clk_disable_unprepare(xcvr->ipg_clk);

	return 0;
}

static int fsl_xcvr_runtime_resume(struct device *dev)
{
	struct fsl_xcvr *xcvr = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(xcvr->ipg_clk);
	if (ret) {
		dev_err(dev, "failed to start IPG clock.\n");
		return ret;
	}

	ret = clk_prepare_enable(xcvr->phy_clk);
	if (ret) {
		dev_err(dev, "failed to start PHY clock.\n");
		clk_disable_unprepare(xcvr->ipg_clk);
		return ret;
	}

	ret = clk_prepare_enable(xcvr->spba_clk);
	if (ret) {
		dev_err(dev, "failed to start SPBA clock.\n");
		clk_disable_unprepare(xcvr->phy_clk);
		clk_disable_unprepare(xcvr->ipg_clk);
		return ret;
	}

	regcache_cache_only(xcvr->regmap, false);
	regcache_mark_dirty(xcvr->regmap);
	ret = regcache_sync(xcvr->regmap);

	if (ret) {
		dev_err(dev, "failed to sync regcache.\n");
		return ret;
	}

	reset_control_assert(xcvr->reset);
	reset_control_deassert(xcvr->reset);

	ret = fsl_xcvr_load_firmware(xcvr);
	if (ret) {
		dev_err(dev, "failed to load firmware.\n");
		return ret;
	}

	/* Release M0+ reset */
	ret = regmap_update_bits(xcvr->regmap, FSL_XCVR_EXT_CTRL,
				 FSL_XCVR_EXT_CTRL_CORE_RESET, 0);
	if (ret < 0) {
		dev_err(dev, "Failed to release M0+ core: %d\n", ret);
		return ret;
	}

	return 0;
}
#endif /* CONFIG_PM*/

static const struct dev_pm_ops fsl_xcvr_pm_ops = {
	SET_RUNTIME_PM_OPS(fsl_xcvr_runtime_suspend,
			   fsl_xcvr_runtime_resume,
			   NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

static struct platform_driver fsl_xcvr_driver = {
	.probe = fsl_xcvr_probe,
	.driver = {
		.name = "fsl,imx8mp-audio-xcvr",
		.pm = &fsl_xcvr_pm_ops,
		.of_match_table = fsl_xcvr_dt_ids,
	},
};
module_platform_driver(fsl_xcvr_driver);

MODULE_AUTHOR("Viorel Suman <viorel.suman@nxp.com>");
MODULE_DESCRIPTION("NXP Audio Transceiver (XCVR) driver");
MODULE_LICENSE("GPL v2");
