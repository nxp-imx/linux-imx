/*
 * imx-esai.h  --  ESAI driver header file for Freescale IMX
 *
 * Copyright 2008-2010 Freescale  Semiconductor, Inc. All Rights Reserved.
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

#define ESAI_IO_BASE_ADDR	(esai_ioaddr)

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

/* ESAI clock divider */
#define ESAI_TX_DIV_PSR	0
#define ESAI_TX_DIV_PM 1
#define ESAI_TX_DIV_FP	2
#define ESAI_RX_DIV_PSR	3
#define ESAI_RX_DIV_PM	4
#define ESAI_RX_DIV_FP	5

#define IMX_DAI_ESAI_TX 0x04
#define IMX_DAI_ESAI_RX 0x08
#define IMX_DAI_ESAI_TXRX (IMX_DAI_ESAI_TX | IMX_DAI_ESAI_RX)

struct imx_esai {
	bool network_mode;
	bool sync_mode;
};

extern struct snd_soc_dai imx_esai_dai[];

#endif
