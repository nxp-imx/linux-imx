/*
 * NXP i.MX HyperBus Memory Controller driver.
 *
 * Copyright 2019 NXP
 *
 * Author: Han Xu <han.xu@nxp.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/pm_runtime.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/hyperbus.h>
#include <linux/slab.h>

/* runtime pm timeout */
#define HYPERBUS_RPM_TIMEOUT 50 /* 50ms */

/* The registers */
#define HYPERBUS_MCR0			0x00
#define HYPERBUS_MCR0_AHB_TIMEOUT_SHIFT	24
#define HYPERBUS_MCR0_AHB_TIMEOUT_MASK	(0xFF << HYPERBUS_MCR0_AHB_TIMEOUT_SHIFT)
#define HYPERBUS_MCR0_IP_TIMEOUT_SHIFT	16
#define HYPERBUS_MCR0_IP_TIMEOUT_MASK	(0xFF << HYPERBUS_MCR0_IP_TIMEOUT_SHIFT)
#define HYPERBUS_MCR0_LEARN_EN_SHIFT	15
#define HYPERBUS_MCR0_LEARN_EN_MASK	(1 << HYPERBUS_MCR0_LEARN_EN_SHIFT)
#define HYPERBUS_MCR0_SCRFRUN_EN_SHIFT	14
#define HYPERBUS_MCR0_SCRFRUN_EN_MASK	(1 << HYPERBUS_MCR0_SCRFRUN_EN_SHIFT)
#define HYPERBUS_MCR0_OCTCOMB_EN_SHIFT	13
#define HYPERBUS_MCR0_OCTCOMB_EN_MASK	(1 << HYPERBUS_MCR0_OCTCOMB_EN_SHIFT)
#define HYPERBUS_MCR0_DOZE_EN_SHIFT	12
#define HYPERBUS_MCR0_DOZE_EN_MASK	(1 << HYPERBUS_MCR0_DOZE_EN_SHIFT)
#define HYPERBUS_MCR0_HSEN_SHIFT		11
#define HYPERBUS_MCR0_HSEN_MASK		(1 << HYPERBUS_MCR0_HSEN_SHIFT)
#define HYPERBUS_MCR0_SERCLKDIV_SHIFT	8
#define HYPERBUS_MCR0_SERCLKDIV_MASK	(7 << HYPERBUS_MCR0_SERCLKDIV_SHIFT)
#define HYPERBUS_MCR0_ATDF_EN_SHIFT	7
#define HYPERBUS_MCR0_ATDF_EN_MASK	(1 << HYPERBUS_MCR0_ATDF_EN_SHIFT)
#define HYPERBUS_MCR0_ARDF_EN_SHIFT	6
#define HYPERBUS_MCR0_ARDF_EN_MASK	(1 << HYPERBUS_MCR0_ARDF_EN_SHIFT)
#define HYPERBUS_MCR0_RXCLKSRC_SHIFT	4
#define HYPERBUS_MCR0_RXCLKSRC_MASK	(3 << HYPERBUS_MCR0_RXCLKSRC_SHIFT)
#define HYPERBUS_MCR0_END_CFG_SHIFT	2
#define HYPERBUS_MCR0_END_CFG_MASK	(3 << HYPERBUS_MCR0_END_CFG_SHIFT)
#define HYPERBUS_MCR0_MDIS_SHIFT		1
#define HYPERBUS_MCR0_MDIS_MASK		(1 << HYPERBUS_MCR0_MDIS_SHIFT)
#define HYPERBUS_MCR0_SWRST_SHIFT	0
#define HYPERBUS_MCR0_SWRST_MASK		(1 << HYPERBUS_MCR0_SWRST_SHIFT)

#define HYPERBUS_MCR1			0x04
#define HYPERBUS_MCR1_SEQ_TIMEOUT_SHIFT	16
#define HYPERBUS_MCR1_SEQ_TIMEOUT_MASK	\
	(0xFFFF << HYPERBUS_MCR1_SEQ_TIMEOUT_SHIFT)
#define HYPERBUS_MCR1_AHB_TIMEOUT_SHIFT	0
#define HYPERBUS_MCR1_AHB_TIMEOUT_MASK	\
	(0xFFFF << HYPERBUS_MCR1_AHB_TIMEOUT_SHIFT)

#define HYPERBUS_MCR2			0x08
#define HYPERBUS_MCR2_IDLE_WAIT_SHIFT	24
#define HYPERBUS_MCR2_IDLE_WAIT_MASK	(0xFF << HYPERBUS_MCR2_IDLE_WAIT_SHIFT)
#define HYPERBUS_MCR2_SCKBDIFFOPT_SHIFT	19
#define HYPERBUS_MCR2_SCKBDIFFOPT_MASK	(1 << HYPERBUS_MCR2_SCKBDIFFOPT_SHIFT)
#define HYPERBUS_MCR2_SAMEFLASH_SHIFT	15
#define HYPERBUS_MCR2_SAMEFLASH_MASK	(1 << HYPERBUS_MCR2_SAMEFLASH_SHIFT)
#define HYPERBUS_MCR2_CLRLRPHS_SHIFT	14
#define HYPERBUS_MCR2_CLRLRPHS_MASK	(1 << HYPERBUS_MCR2_CLRLRPHS_SHIFT)
#define HYPERBUS_MCR2_ABRDATSZ_SHIFT	8
#define HYPERBUS_MCR2_ABRDATSZ_MASK	(1 << HYPERBUS_MCR2_ABRDATSZ_SHIFT)
#define HYPERBUS_MCR2_ABRLEARN_SHIFT	7
#define HYPERBUS_MCR2_ABRLEARN_MASK	(1 << HYPERBUS_MCR2_ABRLEARN_SHIFT)
#define HYPERBUS_MCR2_ABR_READ_SHIFT	6
#define HYPERBUS_MCR2_ABR_READ_MASK	(1 << HYPERBUS_MCR2_ABR_READ_SHIFT)
#define HYPERBUS_MCR2_ABRWRITE_SHIFT	5
#define HYPERBUS_MCR2_ABRWRITE_MASK	(1 << HYPERBUS_MCR2_ABRWRITE_SHIFT)
#define HYPERBUS_MCR2_ABRDUMMY_SHIFT	4
#define HYPERBUS_MCR2_ABRDUMMY_MASK	(1 << HYPERBUS_MCR2_ABRDUMMY_SHIFT)
#define HYPERBUS_MCR2_ABR_MODE_SHIFT	3
#define HYPERBUS_MCR2_ABR_MODE_MASK	(1 << HYPERBUS_MCR2_ABR_MODE_SHIFT)
#define HYPERBUS_MCR2_ABRCADDR_SHIFT	2
#define HYPERBUS_MCR2_ABRCADDR_MASK	(1 << HYPERBUS_MCR2_ABRCADDR_SHIFT)
#define HYPERBUS_MCR2_ABRRADDR_SHIFT	1
#define HYPERBUS_MCR2_ABRRADDR_MASK	(1 << HYPERBUS_MCR2_ABRRADDR_SHIFT)
#define HYPERBUS_MCR2_ABR_CMD_SHIFT	0
#define HYPERBUS_MCR2_ABR_CMD_MASK	(1 << HYPERBUS_MCR2_ABR_CMD_SHIFT)

#define HYPERBUS_AHBCR			0x0c
#define HYPERBUS_AHBCR_RDADDROPT_SHIFT	6
#define HYPERBUS_AHBCR_RDADDROPT_MASK	(1 << HYPERBUS_AHBCR_RDADDROPT_SHIFT)
#define HYPERBUS_AHBCR_PREF_EN_SHIFT	5
#define HYPERBUS_AHBCR_PREF_EN_MASK	(1 << HYPERBUS_AHBCR_PREF_EN_SHIFT)
#define HYPERBUS_AHBCR_BUFF_EN_SHIFT	4
#define HYPERBUS_AHBCR_BUFF_EN_MASK	(1 << HYPERBUS_AHBCR_BUFF_EN_SHIFT)
#define HYPERBUS_AHBCR_CACH_EN_SHIFT	3
#define HYPERBUS_AHBCR_CACH_EN_MASK	(1 << HYPERBUS_AHBCR_CACH_EN_SHIFT)
#define HYPERBUS_AHBCR_CLRTXBUF_SHIFT	2
#define HYPERBUS_AHBCR_CLRTXBUF_MASK	(1 << HYPERBUS_AHBCR_CLRTXBUF_SHIFT)
#define HYPERBUS_AHBCR_CLRRXBUF_SHIFT	1
#define HYPERBUS_AHBCR_CLRRXBUF_MASK	(1 << HYPERBUS_AHBCR_CLRRXBUF_SHIFT)
#define HYPERBUS_AHBCR_PAR_EN_SHIFT	0
#define HYPERBUS_AHBCR_PAR_EN_MASK	(1 << HYPERBUS_AHBCR_PAR_EN_SHIFT)

#define HYPERBUS_INTEN			0x10
#define HYPERBUS_INTEN_SCLKSBWR_SHIFT	9
#define HYPERBUS_INTEN_SCLKSBWR_MASK	(1 << HYPERBUS_INTEN_SCLKSBWR_SHIFT)
#define HYPERBUS_INTEN_SCLKSBRD_SHIFT	8
#define HYPERBUS_INTEN_SCLKSBRD_MASK	(1 << HYPERBUS_INTEN_SCLKSBRD_SHIFT)
#define HYPERBUS_INTEN_DATALRNFL_SHIFT	7
#define HYPERBUS_INTEN_DATALRNFL_MASK	(1 << HYPERBUS_INTEN_DATALRNFL_SHIFT)
#define HYPERBUS_INTEN_IPTXWE_SHIFT	6
#define HYPERBUS_INTEN_IPTXWE_MASK	(1 << HYPERBUS_INTEN_IPTXWE_SHIFT)
#define HYPERBUS_INTEN_IPRXWA_SHIFT	5
#define HYPERBUS_INTEN_IPRXWA_MASK	(1 << HYPERBUS_INTEN_IPRXWA_SHIFT)
#define HYPERBUS_INTEN_AHBCMDERR_SHIFT	4
#define HYPERBUS_INTEN_AHBCMDERR_MASK	(1 << HYPERBUS_INTEN_AHBCMDERR_SHIFT)
#define HYPERBUS_INTEN_IPCMDERR_SHIFT	3
#define HYPERBUS_INTEN_IPCMDERR_MASK	(1 << HYPERBUS_INTEN_IPCMDERR_SHIFT)
#define HYPERBUS_INTEN_AHBCMDGE_SHIFT	2
#define HYPERBUS_INTEN_AHBCMDGE_MASK	(1 << HYPERBUS_INTEN_AHBCMDGE_SHIFT)
#define HYPERBUS_INTEN_IPCMDGE_SHIFT	1
#define HYPERBUS_INTEN_IPCMDGE_MASK	(1 << HYPERBUS_INTEN_IPCMDGE_SHIFT)
#define HYPERBUS_INTEN_IPCMDDONE_SHIFT	0
#define HYPERBUS_INTEN_IPCMDDONE_MASK	(1 << HYPERBUS_INTEN_IPCMDDONE_SHIFT)

#define HYPERBUS_INTR			0x14
#define HYPERBUS_INTR_SCLKSBWR_SHIFT	9
#define HYPERBUS_INTR_SCLKSBWR_MASK	(1 << HYPERBUS_INTR_SCLKSBWR_SHIFT)
#define HYPERBUS_INTR_SCLKSBRD_SHIFT	8
#define HYPERBUS_INTR_SCLKSBRD_MASK	(1 << HYPERBUS_INTR_SCLKSBRD_SHIFT)
#define HYPERBUS_INTR_DATALRNFL_SHIFT	7
#define HYPERBUS_INTR_DATALRNFL_MASK	(1 << HYPERBUS_INTR_DATALRNFL_SHIFT)
#define HYPERBUS_INTR_IPTXWE_SHIFT	6
#define HYPERBUS_INTR_IPTXWE_MASK	(1 << HYPERBUS_INTR_IPTXWE_SHIFT)
#define HYPERBUS_INTR_IPRXWA_SHIFT	5
#define HYPERBUS_INTR_IPRXWA_MASK	(1 << HYPERBUS_INTR_IPRXWA_SHIFT)
#define HYPERBUS_INTR_AHBCMDERR_SHIFT	4
#define HYPERBUS_INTR_AHBCMDERR_MASK	(1 << HYPERBUS_INTR_AHBCMDERR_SHIFT)
#define HYPERBUS_INTR_IPCMDERR_SHIFT	3
#define HYPERBUS_INTR_IPCMDERR_MASK	(1 << HYPERBUS_INTR_IPCMDERR_SHIFT)
#define HYPERBUS_INTR_AHBCMDGE_SHIFT	2
#define HYPERBUS_INTR_AHBCMDGE_MASK	(1 << HYPERBUS_INTR_AHBCMDGE_SHIFT)
#define HYPERBUS_INTR_IPCMDGE_SHIFT	1
#define HYPERBUS_INTR_IPCMDGE_MASK	(1 << HYPERBUS_INTR_IPCMDGE_SHIFT)
#define HYPERBUS_INTR_IPCMDDONE_SHIFT	0
#define HYPERBUS_INTR_IPCMDDONE_MASK	(1 << HYPERBUS_INTR_IPCMDDONE_SHIFT)

#define HYPERBUS_LUTKEY			0x18
#define HYPERBUS_LUTKEY_VALUE		0x5AF05AF0

#define HYPERBUS_LCKCR			0x1C
#define HYPERBUS_LCKER_LOCK		0x1
#define HYPERBUS_LCKER_UNLOCK		0x2

#define HYPERBUS_BUFXCR_INVALID_MSTRID	0xe
#define HYPERBUS_AHBRX_BUF0CR0		0x20
#define HYPERBUS_AHBRX_BUF1CR0		0x24
#define HYPERBUS_AHBRX_BUF2CR0		0x28
#define HYPERBUS_AHBRX_BUF3CR0		0x2C
#define HYPERBUS_AHBRX_BUF4CR0		0x30
#define HYPERBUS_AHBRX_BUF5CR0		0x34
#define HYPERBUS_AHBRX_BUF6CR0		0x38
#define HYPERBUS_AHBRX_BUF7CR0		0x3C
#define HYPERBUS_AHBRXBUF0CR7_PREF_SHIFT	31
#define HYPERBUS_AHBRXBUF0CR7_PREF_MASK	(1 << HYPERBUS_AHBRXBUF0CR7_PREF_SHIFT)

#define HYPERBUS_AHBRX_BUF0CR1		0x40
#define HYPERBUS_AHBRX_BUF1CR1		0x44
#define HYPERBUS_AHBRX_BUF2CR1		0x48
#define HYPERBUS_AHBRX_BUF3CR1		0x4C
#define HYPERBUS_AHBRX_BUF4CR1		0x50
#define HYPERBUS_AHBRX_BUF5CR1		0x54
#define HYPERBUS_AHBRX_BUF6CR1		0x58
#define HYPERBUS_AHBRX_BUF7CR1		0x5C
#define HYPERBUS_BUFXCR1_MSID_SHIFT	0
#define HYPERBUS_BUFXCR1_MSID_MASK	(0xF << HYPERBUS_BUFXCR1_MSID_SHIFT)
#define HYPERBUS_BUFXCR1_PRIO_SHIFT	8
#define HYPERBUS_BUFXCR1_PRIO_MASK	(0x7 << HYPERBUS_BUFXCR1_PRIO_SHIFT)

#define HYPERBUS_FLSHA1CR0		0x60
#define HYPERBUS_FLSHA2CR0		0x64
#define HYPERBUS_FLSHB1CR0		0x68
#define HYPERBUS_FLSHB2CR0		0x6C
#define HYPERBUS_FLSHXCR0_SZ_SHIFT	10
#define HYPERBUS_FLSHXCR0_SZ_MASK	(0x3FFFFF << HYPERBUS_FLSHXCR0_SZ_SHIFT)

#define HYPERBUS_FLSHA1CR1		0x70
#define HYPERBUS_FLSHA2CR1		0x74
#define HYPERBUS_FLSHB1CR1		0x78
#define HYPERBUS_FLSHB2CR1		0x7C
#define HYPERBUS_FLSHXCR1_CSINTR_SHIFT	16
#define HYPERBUS_FLSHXCR1_CSINTR_MASK	\
	(0xFFFF << HYPERBUS_FLSHXCR1_CSINTR_SHIFT)
#define HYPERBUS_FLSHXCR1_CAS_SHIFT	11
#define HYPERBUS_FLSHXCR1_CAS_MASK	(0xF << HYPERBUS_FLSHXCR1_CAS_SHIFT)
#define HYPERBUS_FLSHXCR1_WA_SHIFT	10
#define HYPERBUS_FLSHXCR1_WA_MASK	(1 << HYPERBUS_FLSHXCR1_WA_SHIFT)
#define HYPERBUS_FLSHXCR1_TCSH_SHIFT	5
#define HYPERBUS_FLSHXCR1_TCSH_MASK	(0x1F << HYPERBUS_FLSHXCR1_TCSH_SHIFT)
#define HYPERBUS_FLSHXCR1_TCSS_SHIFT	0
#define HYPERBUS_FLSHXCR1_TCSS_MASK	(0x1F << HYPERBUS_FLSHXCR1_TCSS_SHIFT)

#define HYPERBUS_FLSHA1CR2		0x80
#define HYPERBUS_FLSHA2CR2		0x84
#define HYPERBUS_FLSHB1CR2		0x88
#define HYPERBUS_FLSHB2CR2		0x8C
#define HYPERBUS_FLSHXCR2_CLRINSP_SHIFT	24
#define HYPERBUS_FLSHXCR2_CLRINSP_MASK	(1 << HYPERBUS_FLSHXCR2_CLRINSP_SHIFT)
#define HYPERBUS_FLSHXCR2_AWRWAIT_SHIFT	16
#define HYPERBUS_FLSHXCR2_AWRWAIT_MASK	(0xFF << HYPERBUS_FLSHXCR2_AWRWAIT_SHIFT)
#define HYPERBUS_FLSHXCR2_AWRSEQN_SHIFT	13
#define HYPERBUS_FLSHXCR2_AWRSEQN_MASK	(0x7 << HYPERBUS_FLSHXCR2_AWRSEQN_SHIFT)
#define HYPERBUS_FLSHXCR2_AWRSEQI_SHIFT	8
#define HYPERBUS_FLSHXCR2_AWRSEQI_MASK	(0xF << HYPERBUS_FLSHXCR2_AWRSEQI_SHIFT)
#define HYPERBUS_FLSHXCR2_ARDSEQN_SHIFT	5
#define HYPERBUS_FLSHXCR2_ARDSEQN_MASK	(0x7 << HYPERBUS_FLSHXCR2_ARDSEQN_SHIFT)
#define HYPERBUS_FLSHXCR2_ARDSEQI_SHIFT	0
#define HYPERBUS_FLSHXCR2_ARDSEQI_MASK	(0xF << HYPERBUS_FLSHXCR2_ARDSEQI_SHIFT)

#define HYPERBUS_IPCR0			0xA0

#define HYPERBUS_IPCR1			0xA4
#define HYPERBUS_IPCR1_IPAREN_SHIFT	31
#define HYPERBUS_IPCR1_IPAREN_MASK	(1 << HYPERBUS_IPCR1_IPAREN_SHIFT)
#define HYPERBUS_IPCR1_SEQNUM_SHIFT	24
#define HYPERBUS_IPCR1_SEQNUM_MASK	(0xF << HYPERBUS_IPCR1_SEQNUM_SHIFT)
#define HYPERBUS_IPCR1_SEQID_SHIFT	16
#define HYPERBUS_IPCR1_SEQID_MASK	(0xF << HYPERBUS_IPCR1_SEQID_SHIFT)
#define HYPERBUS_IPCR1_IDATSZ_SHIFT	0
#define HYPERBUS_IPCR1_IDATSZ_MASK	(0xFFFF << HYPERBUS_IPCR1_IDATSZ_SHIFT)

#define HYPERBUS_IPCMD			0xB0
#define HYPERBUS_IPCMD_TRG_SHIFT		0
#define HYPERBUS_IPCMD_TRG_MASK		(1 << HYPERBUS_IPCMD_TRG_SHIFT)

#define HYPERBUS_DLPR			0xB4

#define HYPERBUS_IPRXFCR			0xB8
#define HYPERBUS_IPRXFCR_CLR_SHIFT	0
#define HYPERBUS_IPRXFCR_CLR_MASK	(1 << HYPERBUS_IPRXFCR_CLR_SHIFT)
#define HYPERBUS_IPRXFCR_DMA_EN_SHIFT	1
#define HYPERBUS_IPRXFCR_DMA_EN_MASK	(1 << HYPERBUS_IPRXFCR_DMA_EN_SHIFT)
#define HYPERBUS_IPRXFCR_WMRK_SHIFT	2
#define HYPERBUS_IPRXFCR_WMRK_MASK	(0x1F << HYPERBUS_IPRXFCR_WMRK_SHIFT)

#define HYPERBUS_IPTXFCR			0xBC
#define HYPERBUS_IPTXFCR_CLR_SHIFT	0
#define HYPERBUS_IPTXFCR_CLR_MASK	(1 << HYPERBUS_IPTXFCR_CLR_SHIFT)
#define HYPERBUS_IPTXFCR_DMA_EN_SHIFT	1
#define HYPERBUS_IPTXFCR_DMA_EN_MASK	(1 << HYPERBUS_IPTXFCR_DMA_EN_SHIFT)
#define HYPERBUS_IPTXFCR_WMRK_SHIFT	2
#define HYPERBUS_IPTXFCR_WMRK_MASK	(0x1F << HYPERBUS_IPTXFCR_WMRK_SHIFT)

#define HYPERBUS_DLLACR			0xC0
#define HYPERBUS_DLLBCR			0xC4

#define HYPERBUS_STS0			0xE0
#define HYPERBUS_STS0_DLPHA_SHIFT	9
#define HYPERBUS_STS0_DLPHA_MASK		(0x1F << HYPERBUS_STS0_DLPHA_SHIFT)
#define HYPERBUS_STS0_DLPHB_SHIFT	4
#define HYPERBUS_STS0_DLPHB_MASK		(0x1F << HYPERBUS_STS0_DLPHB_SHIFT)
#define HYPERBUS_STS0_CMD_SRC_SHIFT	2
#define HYPERBUS_STS0_CMD_SRC_MASK	(3 << HYPERBUS_STS0_CMD_SRC_SHIFT)
#define HYPERBUS_STS0_ARB_IDLE_SHIFT	1
#define HYPERBUS_STS0_ARB_IDLE_MASK	(1 << HYPERBUS_STS0_ARB_IDLE_SHIFT)
#define HYPERBUS_STS0_SEQ_IDLE_SHIFT	0
#define HYPERBUS_STS0_SEQ_IDLE_MASK	(1 << HYPERBUS_STS0_SEQ_IDLE_SHIFT)

#define HYPERBUS_STS1			0xE4
#define HYPERBUS_STS1_IP_ERRCD_SHIFT	24
#define HYPERBUS_STS1_IP_ERRCD_MASK	(0xF << HYPERBUS_STS1_IP_ERRCD_SHIFT)
#define HYPERBUS_STS1_IP_ERRID_SHIFT	16
#define HYPERBUS_STS1_IP_ERRID_MASK	(0xF << HYPERBUS_STS1_IP_ERRID_SHIFT)
#define HYPERBUS_STS1_AHB_ERRCD_SHIFT	8
#define HYPERBUS_STS1_AHB_ERRCD_MASK	(0xF << HYPERBUS_STS1_AHB_ERRCD_SHIFT)
#define HYPERBUS_STS1_AHB_ERRID_SHIFT	0
#define HYPERBUS_STS1_AHB_ERRID_MASK	(0xF << HYPERBUS_STS1_AHB_ERRID_SHIFT)

#define HYPERBUS_AHBSPNST		0xEC
#define HYPERBUS_AHBSPNST_DATLFT_SHIFT	16
#define HYPERBUS_AHBSPNST_DATLFT_MASK	\
	(0xFFFF << HYPERBUS_AHBSPNST_DATLFT_SHIFT)
#define HYPERBUS_AHBSPNST_BUFID_SHIFT	1
#define HYPERBUS_AHBSPNST_BUFID_MASK	(7 << HYPERBUS_AHBSPNST_BUFID_SHIFT)
#define HYPERBUS_AHBSPNST_ACTIVE_SHIFT	0
#define HYPERBUS_AHBSPNST_ACTIVE_MASK	(1 << HYPERBUS_AHBSPNST_ACTIVE_SHIFT)

#define HYPERBUS_IPRXFSTS		0xF0
#define HYPERBUS_IPRXFSTS_RDCNTR_SHIFT	16
#define HYPERBUS_IPRXFSTS_RDCNTR_MASK	\
	(0xFFFF << HYPERBUS_IPRXFSTS_RDCNTR_SHIFT)
#define HYPERBUS_IPRXFSTS_FILL_SHIFT	0
#define HYPERBUS_IPRXFSTS_FILL_MASK	(0xFF << HYPERBUS_IPRXFSTS_FILL_SHIFT)

#define HYPERBUS_IPTXFSTS		0xF4
#define HYPERBUS_IPTXFSTS_WRCNTR_SHIFT	16
#define HYPERBUS_IPTXFSTS_WRCNTR_MASK	\
	(0xFFFF << HYPERBUS_IPTXFSTS_WRCNTR_SHIFT)
#define HYPERBUS_IPTXFSTS_FILL_SHIFT	0
#define HYPERBUS_IPTXFSTS_FILL_MASK	(0xFF << HYPERBUS_IPTXFSTS_FILL_SHIFT)

#define HYPERBUS_RFDR			0x100
#define HYPERBUS_TFDR			0x180

#define HYPERBUS_LUT_BASE		0x200

/*
 * The definition of the LUT register shows below:
 *
 *  ---------------------------------------------------
 *  | INSTR1 | PAD1 | OPRND1 | INSTR0 | PAD0 | OPRND0 |
 *  ---------------------------------------------------
 */
#define OPRND0_SHIFT		0
#define PAD0_SHIFT		8
#define INSTR0_SHIFT		10
#define OPRND1_SHIFT		16

/* Instruction set for the LUT register. */

#define LUT_STOP		0x00
#define LUT_CMD			0x01
#define LUT_ADDR		0x02
#define LUT_CADDR_SDR		0x03
#define LUT_MODE		0x04
#define LUT_MODE2		0x05
#define LUT_MODE4		0x06
#define LUT_MODE8		0x07
#define LUT_FSL_WRITE		0x08
#define LUT_FSL_READ		0x09
#define LUT_LEARN_SDR		0x0A
#define LUT_DATSZ_SDR		0x0B
#define LUT_DUMMY		0x0C
#define LUT_DUMMY_RWDS_SDR	0x0D
#define LUT_JMP_ON_CS		0x1F
#define LUT_CMD_DDR		0x21
#define LUT_RADDR_DDR		0x22
#define LUT_CADDR_DDR		0x23
#define LUT_MODE_DDR		0x24
#define LUT_MODE2_DDR		0x25
#define LUT_MODE4_DDR		0x26
#define LUT_MODE8_DDR		0x27
#define LUT_WRITE_DDR		0x28
#define LUT_READ_DDR		0x29
#define LUT_LEARN_DDR		0x2A
#define LUT_DATSZ_DDR		0x2B
#define LUT_DUMMY_DDR		0x2C
#define LUT_DUMMY_RWDS_DDR	0x2D

/*
 * The PAD definitions for LUT register.
 *
 * The pad stands for the lines number of IO[0:3].
 * For example, the Quad read need four IO lines, so you should
 * set LUT_PAD4 which means we use four IO lines.
 */
#define LUT_PAD1		0
#define LUT_PAD2		1
#define LUT_PAD4		2
#define LUT_PAD8		3

/* Oprands for the LUT register. */
#define ADDR24BIT		0x18
#define ADDR32BIT		0x20

/* Macros for constructing the LUT register. */
#define LUT0(ins, pad, opr)						\
		(((opr) << OPRND0_SHIFT) | ((LUT_##pad) << PAD0_SHIFT) | \
		((LUT_##ins) << INSTR0_SHIFT))

#define LUT1(ins, pad, opr)	(LUT0(ins, pad, opr) << OPRND1_SHIFT)

/* other macros for LUT register. */
#define HYPERBUS_LUT(x)          (HYPERBUS_LUT_BASE + (x) * 4)
#define HYPERBUS_LUT_NUM		64

enum imx_hbmc_devtype {
	IMX_HYPERBUS_IMX8QXP,
};

struct imx_hbmc_devtype_data {
	enum imx_hbmc_devtype devtype;
	int ahb_buf_size; /* in 64 bits */
	int driver_data;
};

static const struct imx_hbmc_devtype_data imx8qxp_data = {
	.devtype = IMX_HYPERBUS_IMX8QXP,
	.ahb_buf_size = 2048 / 8, /* in 64 bits */
	.driver_data = 0,
};

static const struct of_device_id imx_hbmc_id_table[] = {
	/* i.MX8QXP */
	{ .compatible = "fsl,imx8qxp-hyperbus", .data = &imx8qxp_data, },
	{ }
};
MODULE_DEVICE_TABLE(of, imx_hbmc_id_table);

struct imx_hbmc {
	struct hyperbus_device hbdev;
	struct hyperbus_ctlr ctlr;
	void __iomem *iobase;
	struct device *dev;
	struct clk *clk;
	struct imx_hbmc_devtype_data *devtype_data;
#define HYPERBUS_INITIALIZED	(1 << 0)
	int flags;
};

static void imx_hbmc_writel(u32 val, void __iomem *addr)
{
	iowrite32(val, addr);
}

static u32 imx_hbmc_readl(void __iomem *addr)
{
	return ioread32(addr);
}

static void imx_hbmc_unlock_lut(struct imx_hbmc *h)
{
	imx_hbmc_writel(HYPERBUS_LUTKEY_VALUE, h->iobase + HYPERBUS_LUTKEY);
	imx_hbmc_writel(HYPERBUS_LCKER_UNLOCK, h->iobase + HYPERBUS_LCKCR);
}

static void imx_hbmc_lock_lut(struct imx_hbmc *h)
{
	imx_hbmc_writel(HYPERBUS_LUTKEY_VALUE, h->iobase + HYPERBUS_LUTKEY);
	imx_hbmc_writel(HYPERBUS_LCKER_LOCK, h->iobase + HYPERBUS_LCKCR);
}

static void imx_hbmc_init_lut(struct imx_hbmc *h)
{
	void __iomem *base = h->iobase;
	int lut_base;
	int addrlen;
	int i;

	imx_hbmc_unlock_lut(h);

	/* Clear all the LUT table */
	for (i = 0; i < HYPERBUS_LUT_NUM; i++)
		imx_hbmc_writel(0, base + HYPERBUS_LUT_BASE + i * 4);

	addrlen = ADDR24BIT;

	/* read lut settings */
	lut_base = 0;

	imx_hbmc_writel(LUT0(CMD_DDR, PAD8, 0xa0) |
	       LUT1(RADDR_DDR, PAD8, addrlen),
	       base + HYPERBUS_LUT(lut_base));

	imx_hbmc_writel(LUT0(CADDR_DDR, PAD8, 0x10) |
	       LUT1(DUMMY_RWDS_DDR, PAD8, 0x10),
	       base + HYPERBUS_LUT(lut_base + 1));

	imx_hbmc_writel(LUT0(READ_DDR, PAD8, 0x0),
	       base + HYPERBUS_LUT(lut_base + 2));

	imx_hbmc_writel(LUT0(STOP, PAD1, 0),
	       base + HYPERBUS_LUT(lut_base + 3));

	/* write lut settings */
	lut_base = 4;
	imx_hbmc_writel(LUT0(CMD_DDR, PAD8, 0x20) |
	       LUT1(RADDR_DDR, PAD8, addrlen),
	       base + HYPERBUS_LUT(lut_base));

	imx_hbmc_writel(LUT0(CADDR_DDR, PAD8, 0x10) |
	       LUT1(WRITE_DDR, PAD8, 0x0),
	       base + HYPERBUS_LUT(lut_base + 1));

	imx_hbmc_writel(LUT0(STOP, PAD1, 0),
	       base + HYPERBUS_LUT(lut_base + 2));

	imx_hbmc_lock_lut(h);
}

static void imx_hbmc_init_ahb(struct imx_hbmc *h)
{
	void __iomem *base = h->iobase;
	int i;

	imx_hbmc_writel(0, base + HYPERBUS_AHBCR);
	/* AHB configuration for access buffer 0/1/2 .*/
	for (i = 0; i < 7; i++)
		imx_hbmc_writel(0, base + HYPERBUS_AHBRX_BUF0CR0 + 4 * i);
	/*
	 * Set ADATSZ with the maximum AHB buffer size to improve the
	 * read performance.
	 */
	imx_hbmc_writel((h->devtype_data->ahb_buf_size
			| HYPERBUS_AHBRXBUF0CR7_PREF_MASK),
			base + HYPERBUS_AHBRX_BUF7CR0);

	/* prefetch and no start address alignment limitation */
	imx_hbmc_writel(HYPERBUS_AHBCR_RDADDROPT_MASK, base + HYPERBUS_AHBCR);

	/* Set the default lut sequence for AHB */
	/* WRSEQNUM|WRSEQID|RDSEQNUM|RDSEQID */
	/* 0	   |1	   |0	    |0	 */
	imx_hbmc_writel(0x42000100, base + HYPERBUS_FLSHA1CR2);
}

static void  imx_hbmc_setup(struct imx_hbmc *h)
{
	void __iomem *base = h->iobase;
	u32 reg;
	u32 reg2;

	/* SW rst the controller */
	/* hyperbus_swrst(h); */

	/* default value for MCR0 */
	reg = 0xFFFFA032;

	/* Disable the module */
	imx_hbmc_writel(reg | HYPERBUS_MCR0_MDIS_MASK, base + HYPERBUS_MCR0);

	/* set MCR0 */
	imx_hbmc_writel(reg, base + HYPERBUS_MCR0);

	/* set MCR2, enable diff clk for 1.8v hyperflash */
	reg2 = imx_hbmc_readl(base + HYPERBUS_MCR2);
	imx_hbmc_writel(reg2 | HYPERBUS_MCR2_SCKBDIFFOPT_MASK, base + HYPERBUS_MCR2);

	/* set CR1 */
	imx_hbmc_writel(3 << 11 | 1 << 10 | 3 << 5 | 3, base + HYPERBUS_FLSHA1CR1);

	/* set the dll */
	imx_hbmc_writel(0x7f00, base + HYPERBUS_DLLACR);

	/* Reset the FLASHxCR2 */
	imx_hbmc_writel(0, base + HYPERBUS_FLSHA1CR2);
	imx_hbmc_writel(0, base + HYPERBUS_FLSHA2CR2);
	imx_hbmc_writel(0, base + HYPERBUS_FLSHB1CR2);
	imx_hbmc_writel(0, base + HYPERBUS_FLSHB2CR2);

	/* enable module and SW reset*/
	imx_hbmc_writel((reg & ~HYPERBUS_MCR0_MDIS_MASK) | HYPERBUS_MCR0_SWRST_MASK, base + HYPERBUS_MCR0);
	do {
		udelay(1);
	} while (0x1 & readl(base + HYPERBUS_MCR0));

	/* Init the LUT table. */
	imx_hbmc_init_lut(h);
	imx_hbmc_init_ahb(h);
}

/* This function was used to prepare and enable clock */
static int imx_hbmc_clk_prep_enable(struct imx_hbmc *h)
{
	int ret;

	ret = clk_prepare_enable(h->clk);
	if (ret) {
		dev_err(h->dev, "failed to enable the clock\n");
		return ret;
	}

	return 0;
}

/* This function was used to disable and unprepare QSPI clock */
static void imx_hbmc_clk_disable_unprep(struct imx_hbmc *h)
{
	clk_disable_unprepare(h->clk);
}

static int imx_hbmc_init_rpm(struct imx_hbmc *h)
{
	struct device *dev = h->dev;

	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, HYPERBUS_RPM_TIMEOUT);
	pm_runtime_use_autosuspend(dev);

	return 0;
}

static u16 imx_hbmc_read16(struct hyperbus_device *hbdev, unsigned long addr)
{
	struct imx_hbmc *h;
	void __iomem *base;
	u16 ret;

	h = (struct imx_hbmc *) container_of(hbdev, struct imx_hbmc, hbdev);
	base = h->iobase;

	/* turn off the prefetch */
	imx_hbmc_writel(h->devtype_data->ahb_buf_size, base + HYPERBUS_AHBRX_BUF7CR0);

	ret = __raw_readw(hbdev->map.virt + addr);

	/* turn on the prefetch */
	imx_hbmc_writel((h->devtype_data->ahb_buf_size
			| HYPERBUS_AHBRXBUF0CR7_PREF_MASK),
			base + HYPERBUS_AHBRX_BUF7CR0);

	return ret;
}

static void imx_hbmc_write16(struct hyperbus_device *hbdev, unsigned long addr, u16 val)
{
	return __raw_writew(val, hbdev->map.virt + addr);
}

static int imx_hbmc_prep(struct hyperbus_device *hbdev)
{
	struct imx_hbmc *h;
	int ret;

	h = (struct imx_hbmc *) container_of(hbdev, struct imx_hbmc, hbdev);

	ret = pm_runtime_get_sync(h->dev);
	if (ret < 0) {
		dev_err(h->dev, "Failed to enable clock %d\n", __LINE__);
		return ret;
	}
	return 0;
}

static void imx_hbmc_unprep(struct hyperbus_device *hbdev)
{
	struct imx_hbmc *h;

	h = (struct imx_hbmc *)
		container_of(hbdev, struct imx_hbmc, hbdev);
	pm_runtime_mark_last_busy(h->dev);
	pm_runtime_put_autosuspend(h->dev);
}

static void imx_hbmc_aligned_memcpy(void *to, const void *from, ssize_t len, int alignment)
{
	unsigned long begin, length, off;
	char *tmp;

	unsigned long src = (unsigned long)from;
	unsigned long dst = (unsigned long)to;

	if (!(src % alignment) && !(dst % alignment) && !(len % alignment)) {
		memcpy(to, from, len);
		return;
	}

	tmp = kzalloc(alignment, GFP_KERNEL);

	/* copy the head */
	begin = src / alignment * alignment;
	off = src - begin;
	length = alignment - off;

	memcpy(tmp, (char *)begin, alignment);

	if (length >= len) {
		memcpy((char *)dst, tmp + off, len);
		kfree(tmp);
		return;
	} else {
		memcpy((char *)dst, tmp + off, length);
		dst += length;
		begin += alignment;
		len -= length;
	}

	/* copy the middle parts if necessary */
	if (len / alignment) {
		length = len / alignment * alignment;
		memcpy((void *)dst, (void *)begin, length);
		dst += length;
		begin += length;
		len -= length;
	}

	/* copy the tail if necessary */
	if (len) {
		memcpy(tmp, (void *)begin, alignment);
		memcpy((void *)dst, tmp, len);
	}

	kfree(tmp);
}

static void imx_hbmc_copy_from(struct hyperbus_device *hbdev, void *to,
			       unsigned long from, ssize_t len)
{
	imx_hbmc_aligned_memcpy(to, (const void *)hbdev->map.virt + from,
				len, 16);
}

static void imx_hbmc_copy_to(struct hyperbus_device *hbdev, unsigned long to,
			       const void *from, ssize_t len)
{
	imx_hbmc_aligned_memcpy(hbdev->map.virt + to, from, len, 16);
}

static const struct hyperbus_ops imx_hbmc_ops = {
	.read16 = imx_hbmc_read16,
	.write16 = imx_hbmc_write16,
	.copy_from = imx_hbmc_copy_from,
	.copy_to = imx_hbmc_copy_to,
	.calibrate = NULL,
	.prepare = imx_hbmc_prep,
	.unprepare = imx_hbmc_unprep,
};

static int imx_hbmc_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct device *dev = &pdev->dev;
	struct imx_hbmc *h;
	int ret;

	const struct of_device_id *of_id =
			of_match_device(imx_hbmc_id_table, &pdev->dev);

	h = devm_kzalloc(dev, sizeof(*h), GFP_KERNEL);
	h->dev = dev;
	h->devtype_data = (struct imx_hbmc_devtype_data *)of_id->data;
	dev_set_drvdata(dev, h);
	platform_set_drvdata(pdev, h);

	/* get the resource */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "HyperBus");

	h->iobase = devm_ioremap_resource(dev, res);
	if (IS_ERR(h->iobase)) {
		ret = PTR_ERR(h->iobase);
		goto ioremap_err;
	}

	/* get the clock */
	h->clk = devm_clk_get(&pdev->dev, "hyperbus");
	if (IS_ERR(h->clk)) {
		ret = PTR_ERR(h->clk);
		dev_err(dev, "cannot get the clock\n");
		goto ioremap_err;
	}

	/* enable the rpm*/
	ret = imx_hbmc_init_rpm(h);
	if (ret) {
		dev_err(dev, "cannot enable the rpm\n");
		goto ioremap_err;
	}

	/* enable the clock*/
	ret = pm_runtime_get_sync(h->dev);
	if (ret < 0) {
		dev_err(h->dev, "Failed to enable clock %d\n", __LINE__);
		goto rpm_err;
	}

	/* setup the controller */
	imx_hbmc_setup(h);

	h->ctlr.dev = dev;
	h->ctlr.ops = &imx_hbmc_ops;
	h->hbdev.np = of_get_next_child(dev->of_node, NULL);
	h->hbdev.ctlr = &h->ctlr;

	ret = hyperbus_register_device(&h->hbdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register controller, %d\n", ret);
		goto rpm_err;
	}

	h->flags |= HYPERBUS_INITIALIZED;

	dev_info(&pdev->dev, "Driver registered.\n");

	pm_runtime_mark_last_busy(h->dev);
	pm_runtime_put_autosuspend(h->dev);

	return ret;

rpm_err:
	pm_runtime_dont_use_autosuspend(h->dev);
	pm_runtime_disable(h->dev);
ioremap_err:
	dev_err(dev, "i.MX HyperBus driver probe failed\n");
	return ret;
}

static int imx_hbmc_remove(struct platform_device *pdev)
{
	struct imx_hbmc *h = platform_get_drvdata(pdev);

	/* disable the hardware */
	imx_hbmc_writel(HYPERBUS_MCR0_MDIS_MASK, h->iobase + HYPERBUS_MCR0);
	pm_runtime_disable(h->dev);

	return 0;
}

static int imx_hbmc_initialized(struct imx_hbmc *h)
{
	return h->flags & HYPERBUS_INITIALIZED;
}

static int imx_hbmc_need_reinit(struct imx_hbmc *h)
{
	/* check the HYPERBUS_AHBCR_RDADDROPT_MASK to determine */
	/* if the controller once lost power and need to reinit */
	return !(readl(h->iobase + HYPERBUS_AHBCR) & HYPERBUS_AHBCR_RDADDROPT_MASK);
}

static int imx_hbmc_runtime_suspend(struct device *dev)
{
	struct imx_hbmc *h = dev_get_drvdata(dev);

	imx_hbmc_clk_disable_unprep(h);

	return 0;
}

static int imx_hbmc_runtime_resume(struct device *dev)
{
	struct imx_hbmc *h = dev_get_drvdata(dev);

	imx_hbmc_clk_prep_enable(h);

	if (imx_hbmc_initialized(h) &&
			imx_hbmc_need_reinit(h)) {
		imx_hbmc_setup(h);
	}

	return 0;
}

static const struct dev_pm_ops imx_hbmc_pm_ops = {
	SET_RUNTIME_PM_OPS(imx_hbmc_runtime_suspend, imx_hbmc_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
};

static struct platform_driver imx_hbmc_driver = {
	.driver = {
		.name		= "imx-hyperbus",
		.of_match_table	= imx_hbmc_id_table,
		.pm		= &imx_hbmc_pm_ops,
	},
	.probe	= imx_hbmc_probe,
	.remove	= imx_hbmc_remove,
};
module_platform_driver(imx_hbmc_driver);

MODULE_AUTHOR("Han Xu <han.xu@nxp.com>");
MODULE_DESCRIPTION("i.MX HyperBus Memory Controller Driver");
MODULE_LICENSE("GPL");
