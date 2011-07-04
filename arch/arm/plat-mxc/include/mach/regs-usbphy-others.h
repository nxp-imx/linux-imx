/*
 * Copyright (C) 2005-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __ASM_ARCH_MXC_USB_PHY_OTHERS_H__
#define __ASM_ARCH_MXC_USB_PHY_OTHERS_H__

/*
 * This file describes the registers of chipidea's phy
 * This file can be used for i.MX1x, i.MX21. i.MX25. i.MX3x. i.MX5x.
 */

#define USBCTRL			USBOTHER_REG(0x00)	/* USB Control register */
#define USB_OTG_MIRROR		USBOTHER_REG(0x04)	/* USB OTG mirror register */
#define USB_PHY_CTR_FUNC	USBOTHER_REG(0x08)      /* OTG UTMI PHY Function Control register */
#define USB_PHY_CTR_FUNC2	USBOTHER_REG(0x0c)      /* OTG UTMI PHY Function Control register */
#define USB_CTRL_1		USBOTHER_REG(0x10)	/* USB Cotrol Register 1*/
#define USBCTRL_HOST2		USBOTHER_REG(0x14)	/* USB Cotrol Register 1*/
#define USBCTRL_HOST3		USBOTHER_REG(0x18)	/* USB Cotrol Register 1*/
#define USBH1_PHY_CTRL0		USBOTHER_REG(0x1c)	/* USB Cotrol Register 1*/
#define USBH1_PHY_CTRL1		USBOTHER_REG(0x20)	/* USB Cotrol Register 1*/
#define USB_CLKONOFF_CTRL       USBOTHER_REG(0x24)      /* USB Clock on/off Control Register */

/*
 * register bits
 */

/* USBCTRL */
#define UCTRL_OWIR		(1 << 31)	/* OTG wakeup intr request received */
#define UCTRL_OSIC_MASK		(3 << 29)	/* OTG  Serial Interface Config: */
#define UCTRL_OSIC_DU6		(0 << 29)	/* Differential/unidirectional 6 wire */
#define UCTRL_OSIC_DB4		(1 << 29)	/* Differential/bidirectional  4 wire */
#define UCTRL_OSIC_SU6		(2 << 29)	/* single-ended/unidirectional 6 wire */
#define UCTRL_OSIC_SB3		(3 << 29)	/* single-ended/bidirectional  3 wire */
#define UCTRL_OUIE		(1 << 28)	/* OTG ULPI intr enable */
#define UCTRL_OWIE		(1 << 27)	/* OTG wakeup intr enable */
#define UCTRL_OBPVAL_RXDP	(1 << 26)	/* OTG RxDp status in bypass mode */
#define UCTRL_OBPVAL_RXDM	(1 << 25)	/* OTG RxDm status in bypass mode */
#define UCTRL_OPM		(1 << 24)	/* OTG power mask */
#define UCTRL_O_PWR_POL	(1 << 24)	/* OTG power pin polarity */
#ifdef CONFIG_ARCH_MX5
#define UCTRL_H2WIR		(1 << 17)	/* HOST2 wakeup intr request received */
#else
#define UCTRL_H2WIR		(1 << 23)	/* HOST2 wakeup intr request received */
#endif
#define UCTRL_H2SIC_MASK	(3 << 21)	/* HOST2 Serial Interface Config: */
#define UCTRL_H2SIC_DU6		(0 << 21)	/* Differential/unidirectional 6 wire */
#define UCTRL_H2SIC_DB4		(1 << 21)	/* Differential/bidirectional  4 wire */
#define UCTRL_H2SIC_SU6		(2 << 21)	/* single-ended/unidirectional 6 wire */
#define UCTRL_H2SIC_SB3		(3 << 21)	/* single-ended/bidirectional  3 wire */

#ifdef CONFIG_ARCH_MX5
#define UCTRL_H2UIE		(1 << 8)	/* HOST2 ULPI intr enable */
#define UCTRL_H2WIE		(1 << 7)	/* HOST2 wakeup intr enable */
#define UCTRL_H2PP		0	/* Power Polarity for uh2 */
#define UCTRL_H2PM		(1 << 4)	/* HOST2 power mask */
#else
#define UCTRL_H2UIE		(1 << 20)	/* HOST2 ULPI intr enable */
#define UCTRL_H2WIE		(1 << 19)	/* HOST2 wakeup intr enable */
#define UCTRL_H2PP		(1 << 18)	/* Power Polarity for uh2 */
#define UCTRL_H2PM		(1 << 16)	/* HOST2 power mask */
#endif
#define UCTRL_H2OVBWK_EN	(1 << 6) /* OTG VBUS Wakeup Enable */
#define UCTRL_H2OIDWK_EN	(1 << 5) /* OTG ID Wakeup Enable */

#define UCTRL_H1WIR		(1 << 15)	/* HOST1 wakeup intr request received */
#define UCTRL_H1SIC_MASK	(3 << 13)	/* HOST1 Serial Interface Config: */
#define UCTRL_H1SIC_DU6		(0 << 13)	/* Differential/unidirectional 6 wire */
#define UCTRL_H1SIC_DB4		(1 << 13)	/* Differential/bidirectional  4 wire */
#define UCTRL_H1SIC_SU6		(2 << 13)	/* single-ended/unidirectional 6 wire */
#define UCTRL_H1SIC_SB3		(3 << 13)	/* single-ended/bidirectional  3 wire */
#define UCTRL_OLOCKD		(1 << 13)	/* otg lock disable */
#define UCTRL_H2LOCKD		(1 << 12)	/* HOST2 lock disable */
#define UCTRL_H1UIE		(1 << 12)	/* Host1 ULPI interrupt enable */

#if defined(CONFIG_ARCH_MX37)
/* VBUS wakeup enable, UTMI only */
#define UCTRL_VBUS_WKUP_EN	(1 << 12)
#elif defined(CONFIG_ARCH_MX25) || defined(CONFIG_ARCH_MX35)
#define UCTRL_VBUS_WKUP_EN      (1 << 15)
#endif

#define UCTRL_PP                (1 << 11)       /* power polarity bit */
#define UCTRL_H1WIE		(1 << 11)	/* HOST1 wakeup intr enable */
#define UCTRL_H1BPVAL_RXDP	(1 << 10)	/* HOST1 RxDp status in bypass mode */
#define UCTRL_XCSO              (1 << 10)       /* Xcvr Clock Select for OTG port */
#define UCTRL_H1BPVAL_RXDM	(1 <<  9)	/* HOST1 RxDm status in bypass mode */
#define UCTRL_XCSH2             (1 <<  9)       /* Xcvr Clock Select for Host port */
#define UCTRL_H1PM		(1 <<  8)	/* HOST1 power mask */
#define UCTRL_IP_PULIDP         (1 <<  8)       /* Ipp_Puimpel_Pullup_Dp */

#define UCTRL_IP_PUE_UP         (1 <<  7)       /* ipp_pue_pullup_dp */
#define UCTRL_IP_PUE_DOWN       (1 <<  6)       /* ipp_pue_pulldwn_dpdm */
#define UCTRL_H2DT		(1 <<  5)	/* HOST2 TLL disabled */
#define UCTRL_H1DT		(1 <<  4)	/* HOST1 TLL disabled */
#define UCTRL_USBTE             (1 <<  4)       /* USBT Transceiver enable */
#define UCTRL_OCPOL             (1 <<  3)       /* OverCurrent Polarity */
#define UCTRL_OCE               (1 <<  2)       /* OverCurrent Enable */
#define UCTRL_H2OCPOL		(1 <<  2)       /* OverCurrent Polarity of Host2 */
#define UCTRL_H2OCS             (1 <<  1)       /* Host OverCurrent State */
#define UCTRL_BPE		(1 <<  0)	/* bypass mode enable */
#define UCTRL_OTD		(1 <<  0)	/* OTG TLL Disable */
#define UCTRL_OOCS              (1 <<  0)       /* OTG OverCurrent State */

/* OTG_MIRROR */
#define OTGM_SESEND		(1 << 4)	/* B device session end */
#define OTGM_VBUSVAL		(1 << 3)	/* Vbus valid */
#define OTGM_BSESVLD		(1 << 2)	/* B session Valid */
#define OTGM_ASESVLD		(1 << 1)	/* A session Valid */
#define OTGM_IDIDG		(1 << 0)	/* OTG ID pin status */
				/* 1=high: Operate as B-device */
				/* 0=low : Operate as A-device */

/* USB_PHY_CTRL_FUNC */
/* PHY control0 Register Bit Masks */
#define USB_UTMI_PHYCTRL_CONF2	(1 << 26)

#define USB_UTMI_PHYCTRL_UTMI_ENABLE (1 << 24)
#define USB_UTMI_PHYCTRL_CHGRDETEN (1 << 24)    /* Enable Charger Detector */
#define USB_UTMI_PHYCTRL_CHGRDETON (1 << 23)    /* Charger Detector Power On Control */
#define USB_UTMI_PHYCTRL_OC_POL	(1 << 9)	/* OTG Polarity of Overcurrent */
#define USB_UTMI_PHYCTRL_OC_DIS	(1 << 8)	/* OTG Disable Overcurrent Event */
#define USB_UH1_OC_DIS	(1 << 5)		/* UH1 Disable Overcurrent Event */
#define USB_UH1_OC_POL	(1 << 6)		/* UH1 Polarity of OC,Low active */
/* USB_PHY_CTRL_FUNC2*/
#define USB_UTMI_PHYCTRL2_PLLDIV_MASK		0x3
#define USB_UTMI_PHYCTRL2_PLLDIV_SHIFT		0
#define USB_UTMI_PHYCTRL2_HSDEVSEL_MASK		0x3
#define USB_UTMI_PHYCTRL2_HSDEVSEL_SHIFT	19

/* USB_CTRL_1 */
#define USB_CTRL_UH1_EXT_CLK_EN			(1 << 25)
#define USB_CTRL_UH2_EXT_CLK_EN			(1 << 26)
#define USB_CTRL_UH2_CLK_FROM_ULPI_PHY  	(1 << 2)
/* ULPIVIEW register bits */
#define ULPIVW_OFF		(0x170)
#define ULPIVW_WU		(1 << 31)	/* Wakeup */
#define ULPIVW_RUN		(1 << 30)	/* read/write run */
#define ULPIVW_WRITE		(1 << 29)	/* 0=read  1=write */
#define ULPIVW_SS		(1 << 27)	/* SyncState */
#define ULPIVW_PORT_MASK	0x07	/* Port field */
#define ULPIVW_PORT_SHIFT	24
#define ULPIVW_ADDR_MASK	0xFF	/* data address field */
#define ULPIVW_ADDR_SHIFT	16
#define ULPIVW_RDATA_MASK	0xFF	/* read data field */
#define ULPIVW_RDATA_SHIFT	8
#define ULPIVW_WDATA_MASK	0xFF	/* write data field */
#define ULPIVW_WDATA_SHIFT	0

/* USB Clock on/off Control Register */
#define OTG_AHBCLK_OFF          (0x1<<17)      /* 1: OFF */
#define H1_AHBCLK_OFF           (0x1<<18)      /* 1: OFF */
#endif /* __ASM_ARCH_MXC_USB_PHY_OTHERS_H__ */
