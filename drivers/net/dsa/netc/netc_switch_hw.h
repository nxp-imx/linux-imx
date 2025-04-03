/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/*
 * Copyright 2024 NXP
 */

#ifndef _NETC_SWITCH_HW_H
#define _NETC_SWITCH_HW_H

#include <linux/bitops.h>

#define NETC_SWITCH_VENDOR_ID		0x1131
#define NETC_SWITCH_DEVICE_ID		0xeef2

/* Definition of Switch base registers */
#define NETC_SCAPR0			0x0000
#define NETC_SCAPR1			0x0004
#define NETC_BPCAPR			0x0008
#define  BPCAPR_NUM_BP			GENMASK(7, 0)
#define   BPCAPR_GET_NUM_BP(x)		((x) & BPCAPR_NUM_BP)
#define  BPCAPR_NUM_SBP			GENMASK(20, 16)
#define   BPCAPR_GET_NUM_SBP(x)		FIELD_GET(BPCAPR_NUM_SBP, x)

#define NETC_CCAR			0x0080

#define NETC_CBDRMR(a)			(0x0800 + (a) * 0x30)
#define  CBDRMR_EN			BIT(31)

#define NETC_CBDRSR(a)			(0x0804 + (a) * 0x30)
#define  CBDRSR_BUSY			BIT(0)

#define NETC_CBDRBAR0(a)		(0x0810 + (a) * 0x30)
#define NETC_CBDRBAR1(a)		(0x0814 + (a) * 0x30)
#define NETC_CBDRPIR(a)			(0x0818 + (a) * 0x30)
#define NETC_CBDRCIR(a)			(0x081c + (a) * 0x30)
#define NETC_CBDRLENR(a)		(0x0820 + (a) * 0x30)

#define NETC_DOSL2CR			0x1220
#define  DOSL2CR_SAMEADDR		BIT(0)
#define  DOSL2CR_MSAMCC			BIT(1)

#define NETC_DOSL3CR			0x1224
#define  DOSL3CR_SAMEADDR		BIT(0)
#define  DOSL3CR_IPSAMCC		BIT(1)

#define NETC_RPITCAPR			0x1814
#define NETC_ISCITCAPR			0x1824
#define NETC_ISITCAPR			0x1834
#define NETC_ISQGITCAPR			0x1844
#define NETC_SGIITCAPR			0x1864
#define NETC_FMITCAPR			0x1888
#define NETC_FMDITCAPR			0x1894
#define NETC_ETTCAPR			0x18c4
#define NETC_ESQRTCAPR			0x18e4
#define NETC_ECTCAPR			0x18ec
/* Index table NUM_ENTRIES mask */
#define NETC_NUM_ENTRIES		GENMASK(15, 0)
#define NETC_GET_NUM_ENTRIES(v)		((v) & NETC_NUM_ENTRIES)

#define NETC_SGCLITCAPR			0x1874
#define NETC_TGSTCAPR			0x18d4
#define NETC_TGSTMOR			0x18dc
/* Hash table or hardware managed ternary match table NUM_WORDS mask */
#define NETC_NUM_WORDS			GENMASK(15, 0)
#define NETC_GET_NUM_WORDS(v)		((v) & NETC_NUM_WORDS)

#define NETC_ISIDKCCR0(a)		(0x1924 + (a) * 0x20)
#define  ISIDKCCR0_VALID		BIT(0)
#define  ISIDKCCR0_PORTP		BIT(1)
#define  ISIDKCCR0_DMACP		BIT(3)
#define  ISIDKCCR0_SMACP		BIT(4)
#define  ISIDKCCR0_OVIDP		BIT(5)
#define  ISIDKCCR0_SQTP			BIT(9)
#define  ISIDKCCR0_ETP			BIT(10)

#define NETC_VFHTDECR0			0x2010
#define  VFHTDECR0_PORT(a)		BIT((a))
#define  VFHTDECR0_STG_ID		GENMASK(27, 24)
#define  VFHTDECR0_IPMFE		BIT(29)
#define  VFHTDECR0_IPMFLE		BIT(30)

#define NETC_VFHTDECR1			0x2014

#define NETC_VFHTDECR2			0x2018
#define  VFHTDECR2_ET_PORT(a)		BIT((a))
#define  VFHTDECR2_MLO			GENMASK(26, 24)
#define  VFHTDECR2_MFO			GENMASK(28, 27)

/* Definition of Switch port registers */
#define NETC_PCAPR			0x0000
#define  PCAPR_LINK_TYPE		BIT(4)
#define  PCAPR_NUM_TC			GENMASK(15, 12)
#define  PCAPR_NUM_Q			GENMASK(19, 16)
#define  PCAPR_NUM_CG			GENMASK(27, 24)
#define  PCAPR_TGS			BIT(28)
#define  PCAPR_CBS			BIT(29)

#define NETC_PMCAPR			0x0004
#define  PMCAPR_HD			BIT(8)
#define  PMCAPR_FP			GENMASK(10, 9)
#define   FP_SUPPORT			2

#define NETC_PCR			0x0010
#define  PCR_HDR_FMT			BIT(0)
#define  PCR_NS_TAG_PORT		BIT(3)
#define  PCR_L2DOSE			BIT(4)
#define  PCR_L3DOSE			BIT(5)
#define  PCR_TIMER_CS			BIT(8)
#define  PCR_PSPEED			GENMASK(29, 16)
#define   PSPEED_SET_VAL(s)		FIELD_PREP(PCR_PSPEED, ((s) / 10 - 1))

#define NETC_PMAR0			0x0020
#define NETC_PMAR1			0x0024
#define NETC_PQOSMR			0x0054
#define  PQOSMR_VS			BIT(0)
#define  PQOSMR_VE			BIT(1)
#define  PQOSMR_DDR			GENMASK(3, 2)
#define  PQOSMR_DIPV			GENMASK(6, 4)
#define  PQOSMR_VQMP			GENMASK(19, 16)
#define  PQOSMR_QVMP			GENMASK(23, 20)

#define NETC_PIPFCR			0x0084
#define  PIPFCR_EN			BIT(0)

#define NETC_POR			0x100
#define  PCR_TXDIS			BIT(0)
#define  PCR_RXDIS			BIT(1)

#define NETC_PTGSCR			0x110
#define  PTGSCR_TGE			BIT(31)

#define NETC_PFPCR			0x134
#define  PFPCR_FPE_TC(a)		BIT(a)

/* Port Rx/TX discard registers */
#define NETC_PRXDCR			0x1c0
#define NETC_PRXDCRRR			0x1c4
#define NETC_PRXDCRR0			0x1c8
#define NETC_PRXDCRR1			0x1cc
#define NETC_PTXDCR			0x1e0
#define NETC_PTXDCRRR			0x1e4
#define NETC_PTXDCRR0			0x1e8
#define NETC_PTXDCRR1			0x1ec

#define NETC_PTCTMSDUR(a)		(0x208 + (a) * 0x20)
#define  PTCTMSDUR_MAXSDU		GENMASK(15, 0)
#define  PTCTMSDUR_SDU_TYPE		GENMASK(17, 16)
#define   SDU_TYPE_PPDU			0
#define   SDU_TYPE_MPDU			1
#define   SDU_TYPE_MSDU			2

#define NETC_PTCCBSR1(a)		(0x214 + (a) * 0x20)
#define NETC_PTCCBSR2(a)		(0x218 + (a) * 0x20)
#define  PTCCBSR2_IDLESLOPE		GENMASK(26, 0)
#define  PTCCBSR2_CBSE			BIT(31)

#define NETC_PBPMCR0			0x0400
#define NETC_PBPMCR1			0x0404

#define NETC_PISIDCR			0x460
#define  PISIDCR_KCPAIR			BIT(0)
#define  PISIDCR_KC0EN			BIT(1)
#define  PISIDCR_KC1EN			BIT(2)

#define NETC_BPCR			0x500
#define  BPCR_DYN_LIMIT			GENMASK(15, 0)
#define  BPCR_MLO			GENMASK(22, 20)
#define  BPCR_UUCASTE			BIT(24)
#define  BPCR_UMCASTE			BIT(25)
#define  BPCR_MCASTE			BIT(26)
#define  BPCR_BCASTE			BIT(27)
#define  BPCR_STAMVD			BIT(28)
#define  BPCR_SRCPRND			BIT(29)

/* MAC learning options, see BPCR[MLO], VFHTDECR2[MLO] and
 * VLAN Filter Table CFGE_DATA[MLO]
 */
#define MLO_NOT_OVERRIDE		0
#define MLO_DISABLE			1
#define MLO_HW				2
#define MLO_SW_SEC			3
#define MLO_SW_UNSEC			4
#define MLO_DISABLE_SMAC		5

/* MAC forwarding options, see VFHTDECR2[MFO] and VLAN
 * Filter Table CFGE_DATA[MFO]
 */
#define MFO_NO_FDB_LOOKUP		1
#define MFO_NO_MATCH_FLOOD		2
#define MFO_NO_MATCH_DISCARD		3

#define NETC_BPDVR			0x510
#define  BPDVR_VID			GENMASK(11, 0)
#define  BPDVR_DEI			BIT(12)
#define  BPDVR_PCP			GENMASK(15, 13)
#define  BPDVR_TPID			BIT(16)
#define  BPDVR_RXTAGA			GENMASK(23, 20)
#define  BPDVR_RXVAM			BIT(24)
#define  BPDVR_TXTAGA			GENMASK(26, 25)

#define NETC_BPSTGSR			0x0520

#define NETC_BPDCR			0x580
#define NETC_BPDCRRR			0x584
#define NETC_BPDCRR0			0x588
#define NETC_BPDCRR1			0x58c

/* Definition of Switch ethernet MAC port registers */
#define NETC_PM_SCRATCH(a)		(0x1004 + (a) * 0x400)

#define NETC_PMAC_OFFSET		0x400
#define NETC_PM_CMD_CFG(a)		(0x1008 + (a) * 0x400)
#define  PM_CMD_CFG_TX_EN		BIT(0)
#define  PM_CMD_CFG_RX_EN		BIT(1)
#define  PM_CMD_CFG_PAUSE_FWD		BIT(7)
#define  PM_CMD_CFG_PAUSE_IGN		BIT(8)
#define  PM_CMD_CFG_LOOP_EN		BIT(10)
#define  PM_CMD_CFG_LPBK_MODE		GENMASK(12, 11)
#define  PM_CMD_CFG_CNT_FRM_EN		BIT(13)
#define  PM_CMD_CFG_TS_PNT		BIT(14)
#define  PM_CMD_CFG_TXP			BIT(15)
#define  PM_CMD_CFG_SEND_IDLE		BIT(16)
#define  PM_CMD_CFG_HD_FCEN		BIT(18)
#define  PM_CMD_CFG_SFD			BIT(21)
#define  PM_CMD_CFG_TX_FLUSH		BIT(22)
#define  PM_CMD_CFG_LOWP_EN		BIT(23)
#define  PM_CMD_CFG_RX_LOWP_ETY		BIT(24)
#define  PM_CMD_CFG_SWR			BIT(26)
#define  PM_CMD_CFG_RX_FLUSH		BIT(28)
#define  PM_CMD_CFG_RXSTP		BIT(29)
#define  PM_CMD_CFG_TS_MODE		BIT(30)
#define  PM_CMD_CFG_MG			BIT(31)

#define NETC_PM_MAXFRM(a)		(0x1014 + (a) * 0x400)
#define  PM_MAXFRAM			GENMASK(15, 0)

#define NETC_PM_MINFRM(a)		(0x1018 + (a) * 0x400)

#define NETC_PM0_MDIO_CFG		0x1030
#define NETC_IMDIO_BASE			NETC_PM0_MDIO_CFG

#define NETC_PM_PAUSE_QUANTA(a)		(0x1054 + (a) * 0x400)
#define  PAUSE_QUANTA_PQNT		GENMASK(15, 0)

#define NETC_PM_PAUSE_TRHESH(a)		(0x1064 + (a) * 0x400)
#define  PAUSE_TRHESH_QTH		GENMASK(15, 0)

#define NETC_PM_SINGLE_STEP(a)		(0x10c0 + (a) * 0x400)
#define  PM_SINGLE_STEP_CH		BIT(6)
#define  PM_SINGLE_STEP_OFFSET		GENMASK(15, 7)
#define  PM_SINGLE_STEP_EN		BIT(31)

/* Port MAC 0/1 Receive Ethernet Octets Counter */
#define NETC_PM_REOCT(a)		(0x1100 + (a) * 0x400)

/* Port MAC 0/1 Receive Octets Counter */
#define NETC_PM_ROCT(a)			(0x1108 + (a) * 0x400)

/* Port MAC 0/1 Receive Alignment Error Counter Register */
#define NETC_PM_RALN(a)			(0x1110 + (a) * 0x400)

/* Port MAC 0/1 Receive Valid Pause Frame Counter */
#define NETC_PM_RXPF(a)			(0x1118 + (a) * 0x400)

/* Port MAC 0/1 Receive Frame Counter */
#define NETC_PM_RFRM(a)			(0x1120 + (a) * 0x400)

/* Port MAC 0/1 Receive Frame Check Sequence Error Counter */
#define NETC_PM_RFCS(a)			(0x1128 + (a) * 0x400)

/* Port MAC 0/1 Receive VLAN Frame Counter */
#define NETC_PM_RVLAN(a)		(0x1130 + (a) * 0x400)

/* Port MAC 0/1 Receive Frame Error Counter */
#define NETC_PM_RERR(a)			(0x1138 + (a) * 0x400)

/* Port MAC 0/1 Receive Unicast Frame Counter */
#define NETC_PM_RUCA(a)			(0x1140 + (a) * 0x400)

/* Port MAC 0/1 Receive Multicast Frame Counter */
#define NETC_PM_RMCA(a)			(0x1148 + (a) * 0x400)

/* Port MAC 0/1 Receive Broadcast Frame Counter */
#define NETC_PM_RBCA(a)			(0x1150 + (a) * 0x400)

/* Port MAC 0/1 Receive Dropped Packets Counter */
#define NETC_PM_RDRP(a)			(0x1158 + (a) * 0x400)

/* Port MAC 0/1 Receive Packets Counter */
#define NETC_PM_RPKT(a)			(0x1160 + (a) * 0x400)

/* Port MAC 0/1 Receive Undersized Packet Counter */
#define NETC_PM_RUND(a)			(0x1168 + (a) * 0x400)

/* Port MAC 0/1 Receive 64-Octet Packet Counter */
#define NETC_PM_R64(a)			(0x1170 + (a) * 0x400)

/* Port MAC 0/1 Receive 65 to 127-Octet Packet Counter */
#define NETC_PM_R127(a)			(0x1178 + (a) * 0x400)

/* Port MAC 0/1 Receive 128 to 255-Octet Packet Counter */
#define NETC_PM_R255(a)			(0x1180 + (a) * 0x400)

/* Port MAC 0/1 Receive 256 to 511-Octet Packet Counter */
#define NETC_PM_R511(a)			(0x1188 + (a) * 0x400)

/* Port MAC 0/1 Receive 512 to 1023-Octet Packet Counter */
#define NETC_PM_R1023(a)		(0x1190 + (a) * 0x400)

/* Port MAC 0/1 Receive 1024 to 1522-Octet Packet Counter */
#define NETC_PM_R1522(a)		(0x1198 + (a) * 0x400)

/* Port MAC 0/1 Receive 1523 to Max-Octet Packet Counter */
#define NETC_PM_R1523X(a)		(0x11a0 + (a) * 0x400)

/* Port MAC 0/1 Receive Oversized Packet Counter */
#define NETC_PM_ROVR(a)			(0x11a8 + (a) * 0x400)

/* Port MAC 0/1 Receive Jabber Packet Counter */
#define NETC_PM_RJBR(a)			(0x11b0 + (a) * 0x400)

/* Port MAC 0/1 Receive Fragment Packet Counter */
#define NETC_PM_RFRG(a)			(0x11b8 + (a) * 0x400)

/* Port MAC 0/1 Receive Control Packet Counter */
#define NETC_PM_RCNP(a)			(0x11c0 + (a) * 0x400)

/* Port MAC 0/1 Receive Dropped Not Truncated Packets Counter */
#define NETC_PM_RDRNTP(a)		(0x11c8 + (a) * 0x400)

/* Port MAC 0/1 Transmit Ethernet Octets Counter */
#define NETC_PM_TEOCT(a)		(0x1200 + (a) * 0x400)

/* Port MAC 0/1 Transmit Octets Counter */
#define NETC_PM_TOCT(a)			(0x1208 + (a) * 0x400)

/* Port MAC 0/1 Transmit Valid Pause Frame Counter */
#define NETC_PM_TXPF(a)			(0x1218 + (a) * 0x400)

/* Port MAC 0/1 Transmit Frame Counter */
#define NETC_PM_TFRM(a)			(0x1220 + (a) * 0x400)

/* Port MAC 0/1 Transmit Frame Check Sequence Error Counter */
#define NETC_PM_TFCS(a)			(0x1228 + (a) * 0x400)

/* Port MAC 0/1 Transmit VLAN Frame Counter */
#define NETC_PM_TVLAN(a)		(0x1230 + (a) * 0x400)

/* Port MAC 0/1 Transmit Frame Error Counter */
#define NETC_PM_TERR(a)			(0x1238 + (a) * 0x400)

/* Port MAC 0/1 Transmit Unicast Frame Counter */
#define NETC_PM_TUCA(a)			(0x1240 + (a) * 0x400)

/* Port MAC 0/1 Transmit Multicast Frame Counter */
#define NETC_PM_TMCA(a)			(0x1248 + (a) * 0x400)

/* Port MAC 0/1 Transmit Broadcast Frame Counter */
#define NETC_PM_TBCA(a)			(0x1250 + (a) * 0x400)

/* Port MAC 0/1 Transmit Packets Counter */
#define NETC_PM_TPKT(a)			(0x1260 + (a) * 0x400)

/* Port MAC 0/1 Transmit Undersized Packet Counter */
#define NETC_PM_TUND(a)			(0x1268 + (a) * 0x400)

/* Port MAC 0/1 Transmit 64-Octet Packet Counter */
#define NETC_PM_T64(a)			(0x1270 + (a) * 0x400)

/* Port MAC 0/1 Transmit 65 to 127-Octet Packet Counter */
#define NETC_PM_T127(a)			(0x1278 + (a) * 0x400)

/* Port MAC 0/1 Transmit 128 to 255-Octet Packet Counter */
#define NETC_PM_T255(a)			(0x1280 + (a) * 0x400)

/* Port MAC 0/1 Transmit 256 to 511-Octet Packet Counter */
#define NETC_PM_T511(a)			(0x1288 + (a) * 0x400)

/* Port MAC 0/1 Transmit 512 to 1023-Octet Packet Counter */
#define NETC_PM_T1023(a)		(0x1290 + (a) * 0x400)

/* Port MAC 0/1 Transmit 1024 to 1522-Octet Packet Counter */
#define NETC_PM_T1522(a)		(0x1298 + (a) * 0x400)

/* Port MAC 0/1 Transmit 1523 to TX_MTU-Octet Packet Counter */
#define NETC_PM_T1523X(a)		(0x12a0 + (a) * 0x400)

/* Port MAC 0/1 Transmit Control Packet Counter */
#define NETC_PM_TCNP(a)			(0x12c0 + (a) * 0x400)

/* Port MAC 0/1 Transmit Deferred Packet Counter */
#define NETC_PM_TDFR(a)			(0x12d0 + (a) * 0x400)

/* Port MAC 0/1 Transmit Multiple Collisions Counter */
#define NETC_PM_TMCOL(a)		(0x12d8 + (a) * 0x400)

/* Port MAC 0/1 Transmit Single Collision */
#define NETC_PM_TSCOL(a)		(0x12e0 + (a) * 0x400)

/* Port MAC 0/1 Transmit Late Collision Counter */
#define NETC_PM_TLCOL(a)		(0x12e8 + (a) * 0x400)

/* Port MAC 0/1 Transmit Excessive Collisions Counter */
#define NETC_PM_TECOL(a)		(0x12f0 + (a) * 0x400)

#define NETC_PM_IF_MODE(a)		(0x1300 + (a) * 0x400)
#define  PM_IF_MODE_IFMODE		GENMASK(2, 0)
#define   IFMODE_MII			1
#define   IFMODE_RMII			3
#define   IFMODE_RGMII			4
#define   IFMODE_SGMII			5
#define  PM_IF_MODE_REVMII		BIT(3)
#define  PM_IF_MODE_M10			BIT(4)
#define  PM_IF_MODE_HD			BIT(6)
#define  PM_IF_MODE_RGMII_RX_SKW	BIT(10)
#define  PM_IF_MODE_RGMII_TX_SKW	BIT(11)
#define  PM_IF_MODE_CLK_STOP		BIT(12)
#define  PM_IF_MODE_SSP			GENMASK(14, 13)
#define   SSP_100M			0
#define   SSP_10M			1
#define   SSP_1G			2
#define  PM_IF_MODE_ENA			BIT(15)

#define NETC_MAC_MERGE_MMCSR		0x1800
#define  MAC_MERGE_MMCSR_LPE		BIT(1)
#define  MAC_MERGE_MMCSR_LAFS		GENMASK(4, 3)
#define   MMCSR_GET_LAFS(x)		FIELD_GET(MAC_MERGE_MMCSR_LAFS, x)
#define  MAC_MERGE_MMCSR_RAFS		GENMASK(9, 8)
#define   MMCSR_GET_RAFS(x)		FIELD_GET(MAC_MERGE_MMCSR_RAFS, x)
#define  MAC_MERGE_MMCSR_ME		GENMASK(16, 15)
#define   MMCSR_ME_FP_DISABLE		0
#define   MMCSR_ME_FP_1B_BOUNDARY	1
#define   MMCSR_ME_FP_4B_BOUNDARY	2
#define   MMCSR_GET_ME(x)		FIELD_GET(MAC_MERGE_MMCSR_ME, x)
#define  MAC_MERGE_MMCSR_VDIS		BIT(17)
#define  MAC_MERGE_MMCSR_VSTS		GENMASK(20, 18)
#define   MMCSR_GET_VSTS(x)		FIELD_GET(MAC_MERGE_MMCSR_VSTS, x)
#define  MAC_MERGE_MMCSR_TXSTS		GENMASK(22, 21)
#define  MAC_MERGE_MMCSR_VT		GENMASK(29, 23)
#define   MMCSR_GET_VT(x)		FIELD_GET(MAC_MERGE_MMCSR_VT, x)
#define  MAC_MERGE_MMCSR_LINK_FAIL	BIT(31)

#define NETC_MAC_MERGE_MMFAECR		0x1808
#define NETC_MAC_MERGE_MMFSECR		0x180c
#define NETC_MAC_MERGE_MMFAOCR		0x1810
#define NETC_MAC_MERGE_MMFCRXR		0x1814
#define NETC_MAC_MERGE_MMFCTXR		0x1818
#define NETC_MAC_MERGE_MMHCR		0x181c

/* Definition of Switch pseudo MAC port registers */
#define NETC_PPMSR			0x1000
#define NETC_PPMCR			0x1010
#define NETC_PPMROCR			0x1080	/* 64 bits register*/
#define NETC_PPMRUFCR			0x1088
#define NETC_PPMRMFCR			0x1090
#define NETC_PPMRBFCR			0x1098
#define NETC_PPMTOCR			0x10c0
#define NETC_PPMTUFCR			0x10c8
#define NETC_PPMTMFCR			0x10d0
#define NETC_PPMTBFCR			0x10d8

/* Definition of global registers (read only) */
#define NETC_SMCAPR			0x0000
#define NETC_SMDTR			0x0004
#define NETC_IPBRR0			0x0bf8
#define  IPBRR0_IP_REV			GENMASK(15, 0)
#define  IPBRR0_IP_MN			GENMASK(7, 0)
#define  IPBRR0_IP_MJ			GENMASK(15, 8)
#define NETC_IPBRR1			0x0bfc

#endif
