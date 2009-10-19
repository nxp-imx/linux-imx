/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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

/*#define IMX_ESAI_DUMP 1*/

#ifdef IMX_ESAI_DUMP
#define ESAI_DUMP() \
	do {pr_info("dump @ %s\n", __func__); \
	pr_info("ecr %x\n", __raw_readl(ESAI_ECR)); \
	pr_info("esr %x\n", __raw_readl(ESAI_ESR)); \
	pr_info("tfcr %x\n", __raw_readl(ESAI_TFCR)); \
	pr_info("tfsr %x\n", __raw_readl(ESAI_TFSR)); \
	pr_info("rfcr %x\n", __raw_readl(ESAI_RFCR)); \
	pr_info("rfsr %x\n", __raw_readl(ESAI_RFSR)); \
	pr_info("tsr %x\n", __raw_readl(ESAI_TSR)); \
	pr_info("saisr %x\n", __raw_readl(ESAI_SAISR)); \
	pr_info("saicr %x\n", __raw_readl(ESAI_SAICR)); \
	pr_info("tcr %x\n", __raw_readl(ESAI_TCR)); \
	pr_info("tccr %x\n", __raw_readl(ESAI_TCCR)); \
	pr_info("rcr %x\n", __raw_readl(ESAI_RCR)); \
	pr_info("rccr %x\n", __raw_readl(ESAI_RCCR)); \
	pr_info("tsma %x\n", __raw_readl(ESAI_TSMA)); \
	pr_info("tsmb %x\n", __raw_readl(ESAI_TSMB)); \
	pr_info("rsma %x\n", __raw_readl(ESAI_RSMA)); \
	pr_info("rsmb %x\n", __raw_readl(ESAI_RSMB)); \
	pr_info("prrc %x\n", __raw_readl(ESAI_PRRC)); \
	pr_info("pcrc %x\n", __raw_readl(ESAI_PCRC)); } while (0);
#else
#define ESAI_DUMP()
#endif

#define ESAI_IO_BASE_ADDR	IO_ADDRESS(ESAI_BASE_ADDR)

#define ESAI_ETDR	(ESAI_IO_BASE_ADDR + 0x00)
#define ESAI_ERDR	(ESAI_IO_BASE_ADDR + 0x04)
#define ESAI_ECR	(ESAI_IO_BASE_ADDR + 0x08)
#define ESAI_ESR	(ESAI_IO_BASE_ADDR + 0x0C)
#define ESAI_TFCR	(ESAI_IO_BASE_ADDR + 0x10)
#define ESAI_TFSR	(ESAI_IO_BASE_ADDR + 0x14)
#define ESAI_RFCR	(ESAI_IO_BASE_ADDR + 0x18)
#define ESAI_RFSR	(ESAI_IO_BASE_ADDR + 0x1C)
#define ESAI_TX0	(ESAI_IO_BASE_ADDR + 0x80)
#define ESAI_TX1	(ESAI_IO_BASE_ADDR + 0x84)
#define ESAI_TX2	(ESAI_IO_BASE_ADDR + 0x88)
#define ESAI_TX3	(ESAI_IO_BASE_ADDR + 0x8C)
#define ESAI_TX4	(ESAI_IO_BASE_ADDR + 0x90)
#define ESAI_TX5	(ESAI_IO_BASE_ADDR + 0x94)
#define ESAI_TSR	(ESAI_IO_BASE_ADDR + 0x98)
#define ESAI_RX0	(ESAI_IO_BASE_ADDR + 0xA0)
#define ESAI_RX1	(ESAI_IO_BASE_ADDR + 0xA4)
#define ESAI_RX2	(ESAI_IO_BASE_ADDR + 0xA8)
#define ESAI_RX3	(ESAI_IO_BASE_ADDR + 0xAC)
#define ESAI_SAISR	(ESAI_IO_BASE_ADDR + 0xCC)
#define ESAI_SAICR	(ESAI_IO_BASE_ADDR + 0xD0)
#define ESAI_TCR	(ESAI_IO_BASE_ADDR + 0xD4)
#define ESAI_TCCR	(ESAI_IO_BASE_ADDR + 0xD8)
#define ESAI_RCR	(ESAI_IO_BASE_ADDR + 0xDC)
#define ESAI_RCCR	(ESAI_IO_BASE_ADDR + 0xE0)
#define ESAI_TSMA	(ESAI_IO_BASE_ADDR + 0xE4)
#define ESAI_TSMB	(ESAI_IO_BASE_ADDR + 0xE8)
#define ESAI_RSMA	(ESAI_IO_BASE_ADDR + 0xEC)
#define ESAI_RSMB	(ESAI_IO_BASE_ADDR + 0xF0)
#define ESAI_PRRC	(ESAI_IO_BASE_ADDR + 0xF8)
#define ESAI_PCRC	(ESAI_IO_BASE_ADDR + 0xFC)

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
#define ESAI_RFCR_RE(x) ((0xf >> (4 - ((x + 1) >> 1))) << 3)
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
#define ESAI_RCR_RE(x) ((0xf >> (4 - ((x + 1) >> 1))) << 1)

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

/* ESAI clock divider */
#define ESAI_TX_DIV_PSR	0
#define ESAI_TX_DIV_PM 1
#define ESAI_TX_DIV_FP	2
#define ESAI_RX_DIV_PSR	3
#define ESAI_RX_DIV_PM	4
#define ESAI_RX_DIV_FP	5

static int imx_esai_txrx_state;

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
		if (cpu_dai->id & IMX_DAI_ESAI_TX)
			tccr |=
			    ESAI_TCCR_THCKD | ESAI_TCCR_TCKD | ESAI_TCCR_TFSD;
		if (cpu_dai->id & IMX_DAI_ESAI_RX)
			rccr |=
			    ESAI_RCCR_RHCKD | ESAI_RCCR_RCKD | ESAI_RCCR_RFSD;
		if (clk_id == ESAI_CLK_FSYS) {
			if (cpu_dai->id & IMX_DAI_ESAI_TX)
				ecr &= ~(ESAI_ECR_ETI | ESAI_ECR_ETO);
			if (cpu_dai->id & IMX_DAI_ESAI_RX)
				ecr &= ~(ESAI_ECR_ERI | ESAI_ECR_ERO);
		} else if (clk_id == ESAI_CLK_EXTAL) {
			if (cpu_dai->id & IMX_DAI_ESAI_TX) {
				ecr |= ESAI_ECR_ETI;
				ecr &= ~ESAI_ECR_ETO;
			}
			if (cpu_dai->id & IMX_DAI_ESAI_RX) {
				ecr |= ESAI_ECR_ERI;
				ecr &= ~ESAI_ECR_ERO;
			}
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
		tccr |= div;
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
		rccr |= div;
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
static int imx_esai_set_dai_tdm_slot(struct snd_soc_dai *cpu_dai,
				     unsigned int mask, int slots)
{
	u32 tcr, rcr, tccr, rccr;

	if (cpu_dai->id & IMX_DAI_ESAI_TX) {
		tcr = __raw_readl(ESAI_TCR);
		tccr = __raw_readl(ESAI_TCCR);

		tcr &= ESAI_TCR_TMOD_MASK;
		tcr |= ESAI_TCR_TMOD_NETWORK;

		tccr &= ESAI_TCCR_TDC_MASK;
		tccr |= ESAI_TCCR_TDC(slots - 1);

		__raw_writel(tcr, ESAI_TCR);
		__raw_writel(tccr, ESAI_TCCR);
		__raw_writel((mask & 0xffff), ESAI_TSMA);
		__raw_writel(((mask >> 16) & 0xffff), ESAI_TSMB);
	}

	if (cpu_dai->id & IMX_DAI_ESAI_RX) {
		rcr = __raw_readl(ESAI_RCR);
		rccr = __raw_readl(ESAI_RCCR);

		rcr &= ESAI_RCR_RMOD_MASK;
		rcr |= ESAI_RCR_RMOD_NETWORK;

		rccr &= ESAI_RCCR_RDC_MASK;
		rccr |= ESAI_RCCR_RDC(slots - 1);

		__raw_writel(rcr, ESAI_RCR);
		__raw_writel(rccr, ESAI_RCCR);
		__raw_writel((mask & 0xffff), ESAI_RSMA);
		__raw_writel(((mask >> 16) & 0xffff), ESAI_RSMB);
	}

	ESAI_DUMP();

	return 0;
}

/*
 * ESAI DAI format configuration.
 */
static int imx_esai_set_dai_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	bool sync_mode = cpu_dai->symmetric_rates;
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
	if (sync_mode)
		saicr |= ESAI_SAICR_SYNC;
	else
		saicr &= ~ESAI_SAICR_SYNC;

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
		rcr |= ESAI_RCR_RSHFD_MSB | ESAI_RCR_RSWS_STL16_WDL16;
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
	if (!strcmp("imx-esai-tx", dai->name))
		dai->id = IMX_DAI_ESAI_TX;
	else if (!strcmp("imx-esai-rx", dai->name))
		dai->id = IMX_DAI_ESAI_RX;
	else if (!strcmp("imx-esai-txrx", dai->name))
		dai->id = IMX_DAI_ESAI_TXRX;
	else {
		pr_err("%s: invalid device %s\n", __func__, dai->name);
		return -ENODEV;
	}

	imx_esai_txrx_state = 0;

	esai_clk = clk_get(NULL, "esai_clk");

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

struct snd_soc_dai imx_esai_dai = {
	.name = "imx-esai",
	.id = 0,
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
};

EXPORT_SYMBOL_GPL(imx_esai_dai);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("i.MX ASoC ESAI driver");
MODULE_LICENSE("GPL");
