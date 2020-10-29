/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2019 NXP
 */

#ifndef _FSL_AUD2HTX_H
#define _FSL_AUD2HTX_H

#define FSL_AUD2HTX_FORMATS (SNDRV_PCM_FMTBIT_S24_LE | \
			     SNDRV_PCM_FMTBIT_S32_LE)

#define AUD2HTX_CTRL          0x0
#define AUD2HTX_CTRL_EXT      0x4
#define AUD2HTX_WR            0x8
#define AUD2HTX_STATUS        0xC
#define AUD2HTX_IRQ_NOMASK    0x10
#define AUD2HTX_IRQ_MASKED    0x14
#define AUD2HTX_IRQ_MASK      0x18

#define AUD2HTX_CTRL_EN          BIT(0)
#define AUD2HTX_CTRE_DE          BIT(0)
#define AUD2HTX_CTRE_DT_SHIFT    0x1
#define AUD2HTX_CTRE_DT_WIDTH    0x2
#define AUD2HTX_CTRE_DT_MASK     ((BIT(AUD2HTX_CTRE_DT_WIDTH) - 1) \
				 << AUD2HTX_CTRE_DT_SHIFT)

#define AUD2HTX_CTRE_WL_SHIFT    16
#define AUD2HTX_CTRE_WL_WIDTH    5
#define AUD2HTX_CTRE_WL_MASK     ((BIT(AUD2HTX_CTRE_WL_WIDTH) - 1) \
				 << AUD2HTX_CTRE_WL_SHIFT)

#define AUD2HTX_CTRE_WH_SHIFT    24
#define AUD2HTX_CTRE_WH_WIDTH    5
#define AUD2HTX_CTRE_WH_MASK     ((BIT(AUD2HTX_CTRE_WH_WIDTH) - 1) \
				 << AUD2HTX_CTRE_WH_SHIFT)

#define AUD2HTX_WM_HIGH_IRQ_MASK BIT(2)
#define AUD2HTX_WM_LOW_IRQ_MASK  BIT(1)
#define AUD2HTX_OVF_MASK         BIT(0)

struct fsl_aud2htx {
	struct platform_device *pdev;
	struct regmap *regmap;
	struct clk *bus_clk;

	struct snd_dmaengine_dai_dma_data dma_params_rx;
	struct snd_dmaengine_dai_dma_data dma_params_tx;
};

#endif /* _FSL_AUD2HTX_H */
