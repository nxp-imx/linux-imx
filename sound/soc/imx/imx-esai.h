/*
 * imx-esai.h  --  ESAI driver header file for Freescale IMX
 *
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

#ifndef _MXC_ESAI_H
#define _MXC_ESAI_H

#ifdef IMX_ESAI_DUMP
#define ESAI_DUMP() \
	do {pr_info("dump @ %s\n", __func__); \
	pr_info("ESAI_ECR   0x%08x\n", readl(esai->base + ESAI_ECR)); \
	pr_info("ESAI_ESR   0x%08x\n", readl(esai->base + ESAI_ESR)); \
	pr_info("ESAI_TFCR  0x%08x\n", readl(esai->base + ESAI_TFCR)); \
	pr_info("ESAI_TFSR  0x%08x\n", readl(esai->base + ESAI_TFSR)); \
	pr_info("ESAI_RFCR  0x%08x\n", readl(esai->base + ESAI_RFCR)); \
	pr_info("ESAI_RFSR  0x%08x\n", readl(esai->base + ESAI_RFSR)); \
	pr_info("ESAI_TSR   0x%08x\n", readl(esai->base + ESAI_TSR)); \
	pr_info("ESAI_SAISR 0x%08x\n", readl(esai->base + ESAI_SAISR)); \
	pr_info("ESAI_SAICR 0x%08x\n", readl(esai->base + ESAI_SAICR)); \
	pr_info("ESAI_TCR   0x%08x\n", readl(esai->base + ESAI_TCR)); \
	pr_info("ESAI_TCCR  0x%08x\n", readl(esai->base + ESAI_TCCR)); \
	pr_info("ESAI_RCR   0x%08x\n", readl(esai->base + ESAI_RCR)); \
	pr_info("ESAI_RCCR  0x%08x\n", readl(esai->base + ESAI_RCCR)); \
	pr_info("ESAI_TSMA  0x%08x\n", readl(esai->base + ESAI_TSMA)); \
	pr_info("ESAI_TSMB  0x%08x\n", readl(esai->base + ESAI_TSMB)); \
	pr_info("ESAI_RSMA  0x%08x\n", readl(esai->base + ESAI_RSMA)); \
	pr_info("ESAI_RSMB  0x%08x\n", readl(esai->base + ESAI_RSMB)); \
	pr_info("ESAI_PRRC  0x%08x\n", readl(esai->base + ESAI_PRRC)); \
	pr_info("ESAI_PCRC  0x%08x\n", readl(esai->base + ESAI_PCRC)); \
	} while (0);
#else
#define ESAI_DUMP()
#endif

#define ESAI_ETDR	0x00
#define ESAI_ERDR	0x04
#define ESAI_ECR	0x08
#define ESAI_ESR	0x0C
#define ESAI_TFCR	0x10
#define ESAI_TFSR	0x14
#define ESAI_RFCR	0x18
#define ESAI_RFSR	0x1C
#define ESAI_TX0	0x80
#define ESAI_TX1	0x84
#define ESAI_TX2	0x88
#define ESAI_TX3	0x8C
#define ESAI_TX4	0x90
#define ESAI_TX5	0x94
#define ESAI_TSR	0x98
#define ESAI_RX0	0xA0
#define ESAI_RX1	0xA4
#define ESAI_RX2	0xA8
#define ESAI_RX3	0xAC
#define ESAI_SAISR	0xCC
#define ESAI_SAICR	0xD0
#define ESAI_TCR	0xD4
#define ESAI_TCCR	0xD8
#define ESAI_RCR	0xDC
#define ESAI_RCCR	0xE0
#define ESAI_TSMA	0xE4
#define ESAI_TSMB	0xE8
#define ESAI_RSMA	0xEC
#define ESAI_RSMB	0xF0
#define ESAI_PRRC	0xF8
#define ESAI_PCRC	0xFC

#define ESAI_ECR_ETI	(1 << 19)
#define ESAI_ECR_ETO	(1 << 18)
#define ESAI_ECR_ERI	(1 << 17)
#define ESAI_ECR_ERO	(1 << 16)
#define ESAI_ECR_ERST	(1 << 1)
#define ESAI_ECR_ESAIEN	(1 << 0)

#define ESAI_ESR_TINIT	(1 << 10)
#define ESAI_ESR_RFF	(1 << 9)
#define ESAI_ESR_TFE	(1 << 8)
#define ESAI_ESR_TLS	(1 << 7)
#define ESAI_ESR_TDE	(1 << 6)
#define ESAI_ESR_TED	(1 << 5)
#define ESAI_ESR_TD	(1 << 4)
#define ESAI_ESR_RLS	(1 << 3)
#define ESAI_ESR_RDE	(1 << 2)
#define ESAI_ESR_RED	(1 << 1)
#define ESAI_ESR_RD	(1 << 0)

#define ESAI_TFCR_TIEN	(1 << 19)
#define ESAI_TFCR_TE5	(1 << 7)
#define ESAI_TFCR_TE4	(1 << 6)
#define ESAI_TFCR_TE3	(1 << 5)
#define ESAI_TFCR_TE2	(1 << 4)
#define ESAI_TFCR_TE1	(1 << 3)
#define ESAI_TFCR_TE0	(1 << 2)
#define ESAI_TFCR_TFR	(1 << 1)
#define ESAI_TFCR_TFEN	(1 << 0)
#define ESAI_TFCR_TE(x) ((0x3f >> (6 - ((x + 1) >> 1))) << 2)
#define ESAI_TFCR_TE_MASK	0xfff03
#define ESAI_TFCR_TFWM(x)	((x - 1) << 8)
#define ESAI_TFCR_TWA_MASK	0xf8ffff

#define ESAI_RFCR_REXT	(1 << 19)
#define ESAI_RFCR_RE3	(1 << 5)
#define ESAI_RFCR_RE2	(1 << 4)
#define ESAI_RFCR_RE1	(1 << 3)
#define ESAI_RFCR_RE0	(1 << 2)
#define ESAI_RFCR_RFR	(1 << 1)
#define ESAI_RFCR_RFEN	(1 << 0)
#define ESAI_RFCR_RE(x) ((0xf >> (4 - ((x + 1) >> 1))) << 2)
#define ESAI_RFCR_RE_MASK	0xfffc3
#define ESAI_RFCR_RFWM(x)       ((x-1) << 8)
#define ESAI_RFCR_RWA_MASK	0xf8ffff

#define ESAI_WORD_LEN_32	(0x00 << 16)
#define ESAI_WORD_LEN_28	(0x01 << 16)
#define ESAI_WORD_LEN_24	(0x02 << 16)
#define ESAI_WORD_LEN_20	(0x03 << 16)
#define ESAI_WORD_LEN_16	(0x04 << 16)
#define ESAI_WORD_LEN_12	(0x05 << 16)
#define ESAI_WORD_LEN_8	(0x06 << 16)
#define ESAI_WORD_LEN_4	(0x07 << 16)

#define ESAI_SAISR_TODFE	(1 << 17)
#define ESAI_SAISR_TEDE	(1 << 16)
#define ESAI_SAISR_TDE	(1 << 15)
#define ESAI_SAISR_TUE	(1 << 14)
#define ESAI_SAISR_TFS	(1 << 13)
#define ESAI_SAISR_RODF	(1 << 10)
#define ESAI_SAISR_REDF	(1 << 9)
#define ESAI_SAISR_RDF	(1 << 8)
#define ESAI_SAISR_ROE	(1 << 7)
#define ESAI_SAISR_RFS	(1 << 6)
#define ESAI_SAISR_IF2	(1 << 2)
#define ESAI_SAISR_IF1	(1 << 1)
#define ESAI_SAISR_IF0	(1 << 0)

#define ESAI_SAICR_ALC	(1 << 8)
#define ESAI_SAICR_TEBE	(1 << 7)
#define ESAI_SAICR_SYNC	(1 << 6)
#define ESAI_SAICR_OF2	(1 << 2)
#define ESAI_SAICR_OF1	(1 << 1)
#define ESAI_SAICR_OF0	(1 << 0)

#define ESAI_TCR_TLIE	(1 << 23)
#define ESAI_TCR_TIE	(1 << 22)
#define ESAI_TCR_TEDIE	(1 << 21)
#define ESAI_TCR_TEIE	(1 << 20)
#define ESAI_TCR_TPR	(1 << 19)
#define ESAI_TCR_PADC	(1 << 17)
#define ESAI_TCR_TFSR	(1 << 16)
#define ESAI_TCR_TFSL	(1 << 15)
#define ESAI_TCR_TWA	(1 << 7)
#define ESAI_TCR_TSHFD_MSB	(0 << 6)
#define ESAI_TCR_TSHFD_LSB	(1 << 6)
#define ESAI_TCR_TE5	(1 << 5)
#define ESAI_TCR_TE4	(1 << 4)
#define ESAI_TCR_TE3	(1 << 3)
#define ESAI_TCR_TE2	(1 << 2)
#define ESAI_TCR_TE1	(1 << 1)
#define ESAI_TCR_TE0	(1 << 0)
#define ESAI_TCR_TE(x) (0x3f >> (6 - ((x + 1) >> 1)))

#define ESAI_TCR_TSWS_MASK	0xff83ff
#define ESAI_TCR_TSWS_STL8_WDL8	(0x00 << 10)
#define ESAI_TCR_TSWS_STL12_WDL8	(0x04 << 10)
#define ESAI_TCR_TSWS_STL12_WDL12	(0x01 << 10)
#define ESAI_TCR_TSWS_STL16_WDL8	(0x08 << 10)
#define ESAI_TCR_TSWS_STL16_WDL12	(0x05 << 10)
#define ESAI_TCR_TSWS_STL16_WDL16	(0x02 << 10)
#define ESAI_TCR_TSWS_STL20_WDL8	(0x0c << 10)
#define ESAI_TCR_TSWS_STL20_WDL12	(0x09 << 10)
#define ESAI_TCR_TSWS_STL20_WDL16	(0x06 << 10)
#define ESAI_TCR_TSWS_STL20_WDL20	(0x03 << 10)
#define ESAI_TCR_TSWS_STL24_WDL8	(0x10 << 10)
#define ESAI_TCR_TSWS_STL24_WDL12	(0x0d << 10)
#define ESAI_TCR_TSWS_STL24_WDL16	(0x0a << 10)
#define ESAI_TCR_TSWS_STL24_WDL20	(0x07 << 10)
#define ESAI_TCR_TSWS_STL24_WDL24	(0x1e << 10)
#define ESAI_TCR_TSWS_STL32_WDL8	(0x18 << 10)
#define ESAI_TCR_TSWS_STL32_WDL12	(0x15 << 10)
#define ESAI_TCR_TSWS_STL32_WDL16	(0x12 << 10)
#define ESAI_TCR_TSWS_STL32_WDL20	(0x0f << 10)
#define ESAI_TCR_TSWS_STL32_WDL24	(0x1f << 10)

#define ESAI_TCR_TMOD_MASK	0xfffcff
#define ESAI_TCR_TMOD_NORMAL	(0x00 << 8)
#define ESAI_TCR_TMOD_ONDEMAND	(0x01 << 8)
#define ESAI_TCR_TMOD_NETWORK	(0x01 << 8)
#define ESAI_TCR_TMOD_RESERVED (0x02 << 8)
#define ESAI_TCR_TMOD_AC97	(0x03 << 8)

#define ESAI_TCCR_THCKD	(1 << 23)
#define ESAI_TCCR_TFSD	(1 << 22)
#define ESAI_TCCR_TCKD	(1 << 21)
#define ESAI_TCCR_THCKP	(1 << 20)
#define ESAI_TCCR_TFSP	(1 << 19)
#define ESAI_TCCR_TCKP	(1 << 18)

#define ESAI_TCCR_TPSR_MASK 0xfffeff
#define ESAI_TCCR_TPSR_BYPASS (1 << 8)
#define ESAI_TCCR_TPSR_DIV8 (0 << 8)

#define ESAI_TCCR_TFP_MASK	0xfc3fff
#define ESAI_TCCR_TFP(x)	((x & 0xf) << 14)

#define ESAI_TCCR_TDC_MASK	0xffc1ff
#define ESAI_TCCR_TDC(x)	(((x) & 0x1f) << 9)

#define ESAI_TCCR_TPM_MASK	0xffff00
#define ESAI_TCCR_TPM(x)	(x & 0xff)

#define ESAI_RCR_RLIE	(1 << 23)
#define ESAI_RCR_RIE	(1 << 22)
#define ESAI_RCR_REDIE	(1 << 21)
#define ESAI_RCR_REIE	(1 << 20)
#define ESAI_RCR_RPR	(1 << 19)
#define ESAI_RCR_RFSR	(1 << 16)
#define ESAI_RCR_RFSL	(1 << 15)
#define ESAI_RCR_RWA	(1 << 7)
#define ESAI_RCR_RSHFD_MSB (0 << 6)
#define ESAI_RCR_RSHFD_LSB (1 << 6)
#define ESAI_RCR_RE3	(1 << 3)
#define ESAI_RCR_RE2	(1 << 2)
#define ESAI_RCR_RE1	(1 << 1)
#define ESAI_RCR_RE0	(1 << 0)
#define ESAI_RCR_RE(x) (0xf >> (4 - ((x + 1) >> 1)))

#define ESAI_RCR_RSWS_MASK	0xff83ff
#define ESAI_RCR_RSWS_STL8_WDL8	(0x00 << 10)
#define ESAI_RCR_RSWS_STL12_WDL8	(0x04 << 10)
#define ESAI_RCR_RSWS_STL12_WDL12	(0x01 << 10)
#define ESAI_RCR_RSWS_STL16_WDL8	(0x08 << 10)
#define ESAI_RCR_RSWS_STL16_WDL12	(0x05 << 10)
#define ESAI_RCR_RSWS_STL16_WDL16	(0x02 << 10)
#define ESAI_RCR_RSWS_STL20_WDL8	(0x0c << 10)
#define ESAI_RCR_RSWS_STL20_WDL12	(0x09 << 10)
#define ESAI_RCR_RSWS_STL20_WDL16	(0x06 << 10)
#define ESAI_RCR_RSWS_STL20_WDL20	(0x03 << 10)
#define ESAI_RCR_RSWS_STL24_WDL8	(0x10 << 10)
#define ESAI_RCR_RSWS_STL24_WDL12	(0x0d << 10)
#define ESAI_RCR_RSWS_STL24_WDL16	(0x0a << 10)
#define ESAI_RCR_RSWS_STL24_WDL20	(0x07 << 10)
#define ESAI_RCR_RSWS_STL24_WDL24	(0x1e << 10)
#define ESAI_RCR_RSWS_STL32_WDL8	(0x18 << 10)
#define ESAI_RCR_RSWS_STL32_WDL12	(0x15 << 10)
#define ESAI_RCR_RSWS_STL32_WDL16	(0x12 << 10)
#define ESAI_RCR_RSWS_STL32_WDL20	(0x0f << 10)
#define ESAI_RCR_RSWS_STL32_WDL24	(0x1f << 10)

#define ESAI_RCR_RMOD_MASK	0xfffcff
#define ESAI_RCR_RMOD_NORMAL	(0x00 << 8)
#define ESAI_RCR_RMOD_ONDEMAND	(0x01 << 8)
#define ESAI_RCR_RMOD_NETWORK	(0x01 << 8)
#define ESAI_RCR_RMOD_RESERVED (0x02 << 8)
#define ESAI_RCR_RMOD_AC97	(0x03 << 8)

#define ESAI_RCCR_RHCKD	(1 << 23)
#define ESAI_RCCR_RFSD	(1 << 22)
#define ESAI_RCCR_RCKD	(1 << 21)
#define ESAI_RCCR_RHCKP	(1 << 20)
#define ESAI_RCCR_RFSP	(1 << 19)
#define ESAI_RCCR_RCKP	(1 << 18)

#define ESAI_RCCR_RPSR_MASK 0xfffeff
#define ESAI_RCCR_RPSR_BYPASS (1 << 8)
#define ESAI_RCCR_RPSR_DIV8 (0 << 8)

#define ESAI_RCCR_RFP_MASK	0xfc3fff
#define ESAI_RCCR_RFP(x)	((x & 0xf) << 14)

#define ESAI_RCCR_RDC_MASK	0xffc1ff
#define ESAI_RCCR_RDC(x)	(((x) & 0x1f) << 9)

#define ESAI_RCCR_RPM_MASK	0xffff00
#define ESAI_RCCR_RPM(x)	(x & 0xff)

#define ESAI_GPIO_ESAI	0xfff

/* ESAI clock source */
#define ESAI_CLK_FSYS	0
#define ESAI_CLK_EXTAL 1
#define ESAI_CLK_EXTAL_DIV 2

/* ESAI clock divider */
#define ESAI_TX_DIV_PSR	0
#define ESAI_TX_DIV_PM 1
#define ESAI_TX_DIV_FP	2
#define ESAI_RX_DIV_PSR	3
#define ESAI_RX_DIV_PM	4
#define ESAI_RX_DIV_FP	5

#define DRV_NAME "imx-esai"

#include <linux/dmaengine.h>
#include <mach/dma.h>

#define IMX_DAI_ESAI_TX 0x04
#define IMX_DAI_ESAI_RX 0x08
#define IMX_DAI_ESAI_TXRX (IMX_DAI_ESAI_TX | IMX_DAI_ESAI_RX)

struct imx_esai {
	struct platform_device *ac97_dev;
	struct snd_soc_dai *imx_ac97;
	struct clk *clk;
	void __iomem *base;
	int irq;
	int fiq_enable;
	unsigned int offset;

	unsigned int flags;

	void (*ac97_reset) (struct snd_ac97 *ac97);
	void (*ac97_warm_reset)(struct snd_ac97 *ac97);

	struct imx_pcm_dma_params dma_params_rx;
	struct imx_pcm_dma_params dma_params_tx;

	int enabled;
	int imx_esai_txrx_state;

	struct platform_device *soc_platform_pdev;
	struct platform_device *soc_platform_pdev_fiq;
};

#endif
