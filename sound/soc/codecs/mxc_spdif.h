/*
 * ALSA SoC MXC SPDIF codec driver
 *
 * Copyright (C) 2008-2012 Freescale Semiconductor, Inc.
 *
 * Based on stmp3xxx_spdif.h by:
 * Vladimir Barinov <vbarinov@embeddedalley.com>
 *
 * Copyright 2008 SigmaTel, Inc
 * Copyright 2008 Embedded Alley Solutions, Inc
 * Copyright 2008-2009 Freescale Semiconductor, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program  is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */
#ifndef __MXC_SPDIF_CODEC_H
#define __MXC_SPDIF_CODEC_H

#define SPDIF_REG_SCR		0x00
#define SPDIF_REG_SRCD		0x04
#define SPDIF_REG_SRPC		0x08
#define SPDIF_REG_SIE		0x0C
#define SPDIF_REG_SIS		0x10
#define SPDIF_REG_SIC		0x10
#define SPDIF_REG_SRL		0x14
#define SPDIF_REG_SRR		0x18
#define SPDIF_REG_SRCSLH	0x1C
#define SPDIF_REG_SRCSLL	0x20
#define SPDIF_REG_SQU		0x24
#define SPDIF_REG_SRQ		0x28
#define SPDIF_REG_STL		0x2C
#define SPDIF_REG_STR		0x30
#define SPDIF_REG_STCSCH	0x34
#define SPDIF_REG_STCSCL	0x38
#define SPDIF_REG_SRFM		0x44
#define SPDIF_REG_STC		0x50

/* SPDIF Configuration register */
#define SCR_RXFIFO_CTL_ZERO	(1 << 23)
#define SCR_RXFIFO_OFF		(1 << 22)
#define SCR_RXFIFO_RST		(1 << 21)
#define SCR_RXFIFO_FSEL_BIT	(19)
#define SCR_RXFIFO_FSEL_MASK	(0x3 << SCR_RXFIFO_FSEL_BIT)
#define SCR_RXFIFO_AUTOSYNC	(1 << 18)
#define SCR_TXFIFO_AUTOSYNC	(1 << 17)
#define SCR_TXFIFO_ESEL_BIT	(15)
#define SCR_TXFIFO_ESEL_MASK	(0x3 << SCR_TXFIFO_FSEL_BIT)
#define SCR_LOW_POWER		(1 << 13)
#define SCR_SOFT_RESET		(1 << 12)
#define SCR_TXFIFO_ZERO		(0 << 10)
#define SCR_TXFIFO_NORMAL	(1 << 10)
#define SCR_TXFIFO_ONESAMPLE	(1 << 11)
#define SCR_DMA_RX_EN		(1 << 9)
#define SCR_DMA_TX_EN		(1 << 8)
#define SCR_VAL_CLEAR		(1 << 5)
#define SCR_TXSEL_OFF		(0 << 2)
#define SCR_TXSEL_RX		(1 << 2)
#define SCR_TXSEL_NORMAL	(0x5 << 2)
#define SCR_USRC_SEL_NONE	(0x0)
#define SCR_USRC_SEL_RECV	(0x1)
#define SCR_USRC_SEL_CHIP	(0x3)

/* SPDIF CDText control */
#define SRCD_CD_USER_OFFSET	1
#define SRCD_CD_USER		(1 << SRCD_CD_USER_OFFSET)

/* SPDIF Phase Configuration register */
#define SRPC_DPLL_LOCKED	(1 << 6)
#define SRPC_CLKSRC_SEL_OFFSET	7
#define SRPC_CLKSRC_SEL_LOCKED	5
/* gain sel */
#define SRPC_GAINSEL_OFFSET	3

enum spdif_gainsel {
	GAINSEL_MULTI_24 = 0,
	GAINSEL_MULTI_16,
	GAINSEL_MULTI_12,
	GAINSEL_MULTI_8,
	GAINSEL_MULTI_6,
	GAINSEL_MULTI_4,
	GAINSEL_MULTI_3,
	GAINSEL_MULTI_MAX,
};

#define SPDIF_DEFAULT_GAINSEL	GAINSEL_MULTI_8

/* SPDIF interrupt mask define */
#define INT_DPLL_LOCKED		(1 << 20)
#define INT_TXFIFO_UNOV		(1 << 19)
#define INT_TXFIFO_RESYNC	(1 << 18)
#define INT_CNEW		(1 << 17)
#define INT_VAL_NOGOOD		(1 << 16)
#define INT_SYM_ERR		(1 << 15)
#define INT_BIT_ERR		(1 << 14)
#define INT_URX_FUL		(1 << 10)
#define INT_URX_OV		(1 << 9)
#define INT_QRX_FUL		(1 << 8)
#define INT_QRX_OV		(1 << 7)
#define INT_UQ_SYNC		(1 << 6)
#define INT_UQ_ERR		(1 << 5)
#define INT_RX_UNOV		(1 << 4)
#define INT_RX_RESYNC		(1 << 3)
#define INT_LOSS_LOCK		(1 << 2)
#define INT_TX_EMPTY		(1 << 1)
#define INT_RXFIFO_FUL		(1 << 0)

/* SPDIF Clock register */
#define STC_SYSCLK_DIV_OFFSET	11
#define STC_TXCLK_SRC_OFFSET	8
#define STC_TXCLK_SRC_EN_OFFSET 7
#define STC_TXCLK_SRC_EN        (1 << 7)
#define STC_TXCLK_DIV_OFFSET	0

#define SPDIF_CSTATUS_BYTE	6
#define SPDIF_UBITS_SIZE	96
#define SPDIF_QSUB_SIZE		(SPDIF_UBITS_SIZE/8)

/* SPDIF clock source */
enum spdif_clk_src {
	SPDIF_CLK_SRC1 = 0,
	SPDIF_CLK_SRC2,
	SPDIF_CLK_SRC3,
	SPDIF_CLK_SRC4,
	SPDIF_CLK_SRC5,
};

enum spdif_max_wdl {
	SPDIF_MAX_WDL_20,
	SPDIF_MAX_WDL_24
};

enum spdif_wdl {
	SPDIF_WDL_DEFAULT = 0,
	SPDIF_WDL_FIFTH = 4,
	SPDIF_WDL_FOURTH = 3,
	SPDIF_WDL_THIRD = 2,
	SPDIF_WDL_SECOND = 1,
	SPDIF_WDL_MAX = 5
};

#define MXC_SPDIF_RATES_PLAYBACK	(SNDRV_PCM_RATE_32000 |	\
					 SNDRV_PCM_RATE_44100 |	\
					 SNDRV_PCM_RATE_48000)

#define MXC_SPDIF_RATES_CAPTURE		(SNDRV_PCM_RATE_16000 | \
					 SNDRV_PCM_RATE_32000 |	\
					 SNDRV_PCM_RATE_44100 | \
					 SNDRV_PCM_RATE_48000 |	\
					 SNDRV_PCM_RATE_64000 | \
					 SNDRV_PCM_RATE_96000)

#define MXC_SPDIF_FORMATS_PLAYBACK	(SNDRV_PCM_FMTBIT_S16_LE | \
					 SNDRV_PCM_FMTBIT_S20_3LE | \
					 SNDRV_PCM_FMTBIT_S24_LE)

#define MXC_SPDIF_FORMATS_CAPTURE	(SNDRV_PCM_FMTBIT_S24_LE)

#endif /* __MXC_SPDIF_CODEC_H */
