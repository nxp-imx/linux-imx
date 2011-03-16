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
#ifndef __ASM_ARCH_MXC_ARC_OTG_H__
#define __ASM_ARCH_MXC_ARC_OTG_H__

#ifdef CONFIG_ARCH_MX3
extern volatile u32 *mx3_usb_otg_addr;
#define OTG_BASE_ADDR		mx3_usb_otg_addr
#endif
#define USB_OTGREGS_BASE	(OTG_BASE_ADDR + 0x000)
#define USB_H1REGS_BASE		(OTG_BASE_ADDR + 0x200)
#define USB_H2REGS_BASE		(OTG_BASE_ADDR + 0x400)
#ifdef CONFIG_ARCH_MX5
#define USB_H3REGS_BASE		(OTG_BASE_ADDR + 0x600)
#define USB_OTHERREGS_BASE	(OTG_BASE_ADDR + 0x800)
#else
#define USB_OTHERREGS_BASE	(OTG_BASE_ADDR + 0x600)
#endif


#define USBOTG_REG32(offset)	(*((volatile u32 *)(IO_ADDRESS(USB_OTGREGS_BASE + (offset)))))
#define USBOTG_REG16(offset)	(*((volatile u16 *)(IO_ADDRESS(USB_OTGREGS_BASE + (offset)))))

#define USBH1_REG32(offset)	(*((volatile u32 *)(IO_ADDRESS(USB_H1REGS_BASE + (offset)))))
#define USBH1_REG16(offset)	(*((volatile u16 *)(IO_ADDRESS(USB_H1REGS_BASE + (offset)))))

#define USBH2_REG32(offset)	(*((volatile u32 *)(IO_ADDRESS(USB_H2REGS_BASE + (offset)))))
#define USBH2_REG16(offset)	(*((volatile u16 *)(IO_ADDRESS(USB_H2REGS_BASE + (offset)))))

#define USBOTHER_REG(offset)	(*((volatile u32 *)(IO_ADDRESS(USB_OTHERREGS_BASE + (offset)))))

/*
 * OTG registers
 */
#define UOG_ID			USBOTG_REG32(0x00)	/* Host ID */
#define UOG_HWGENERAL		USBOTG_REG32(0x04)	/* Host General */
#define UOG_HWHOST		USBOTG_REG32(0x08)	/* Host h/w params */
#define UOG_HWTXBUF		USBOTG_REG32(0x10)	/* TX buffer h/w params */
#define UOG_HWRXBUF		USBOTG_REG32(0x14)	/* RX buffer h/w params */
#define UOG_CAPLENGTH		USBOTG_REG16(0x100)	/* Capability register length */
#define UOG_HCIVERSION		USBOTG_REG16(0x102)	/* Host Interface version */
#define UOG_HCSPARAMS		USBOTG_REG32(0x104)	/* Host control structural params */
#define UOG_HCCPARAMS		USBOTG_REG32(0x108)	/* control capability params */
#define UOG_DCIVERSION		USBOTG_REG32(0x120)	/* device interface version */
/* start EHCI registers: */
#define UOG_USBCMD		USBOTG_REG32(0x140)	/* USB command register */
#define UOG_USBSTS		USBOTG_REG32(0x144)	/* USB status register */
#define UOG_USBINTR		USBOTG_REG32(0x148)	/* interrupt enable register */
#define UOG_FRINDEX		USBOTG_REG32(0x14c)	/* USB frame index */
/*      segment                             (0x150)	   addr bits 63:32 if needed */
#define UOG_PERIODICLISTBASE	USBOTG_REG32(0x154)	/* host crtlr frame list base addr */
#define UOG_DEVICEADDR		USBOTG_REG32(0x154)	/* device crtlr device address */
#define UOG_ASYNCLISTADDR	USBOTG_REG32(0x158)	/* host ctrlr next async addr */
#define UOG_EPLISTADDR		USBOTG_REG32(0x158)	/* device ctrlr endpoint list addr */
#define UOG_BURSTSIZE		USBOTG_REG32(0x160)	/* host ctrlr embedded TT async buf status */
#define UOG_TXFILLTUNING	USBOTG_REG32(0x164)	/* TX FIFO fill tuning */
#define UOG_ULPIVIEW		USBOTG_REG32(0x170)	/* ULPI viewport */
#define UOG_CFGFLAG		USBOTG_REG32(0x180)	/* configflag (supports HS) */
#define UOG_PORTSC1		USBOTG_REG32(0x184)	/* port status and control */
/* end EHCI registers: */
#define UOG_OTGSC		USBOTG_REG32(0x1a4)	/* OTG status and control */
#define UOG_USBMODE		USBOTG_REG32(0x1a8)	/* USB device mode */
#define UOG_ENDPTSETUPSTAT	USBOTG_REG32(0x1ac)	/* endpoint setup status */
#define UOG_ENDPTPRIME		USBOTG_REG32(0x1b0)	/* endpoint initialization */
#define UOG_ENDPTFLUSH		USBOTG_REG32(0x1b4)	/* endpoint de-initialize */
#define UOG_ENDPTSTAT		USBOTG_REG32(0x1b8)	/* endpoint status */
#define UOG_ENDPTCOMPLETE	USBOTG_REG32(0x1bc)	/* endpoint complete */
#define UOG_EPCTRL0		USBOTG_REG32(0x1c0)	/* endpoint control0 */
#define UOG_EPCTRL1		USBOTG_REG32(0x1c4)	/* endpoint control1 */
#define UOG_EPCTRL2		USBOTG_REG32(0x1c8)	/* endpoint control2 */
#define UOG_EPCTRL3		USBOTG_REG32(0x1cc)	/* endpoint control3 */
#define UOG_EPCTRL4		USBOTG_REG32(0x1d0)	/* endpoint control4 */
#define UOG_EPCTRL5		USBOTG_REG32(0x1d4)	/* endpoint control5 */
#define UOG_EPCTRL6		USBOTG_REG32(0x1d8)	/* endpoint control6 */
#define UOG_EPCTRL7		USBOTG_REG32(0x1dc)	/* endpoint control7 */

/*
 * Host 1 registers
 */
#define UH1_ID			USBH1_REG32(0x00)	/* Host ID */
#define UH1_HWGENERAL		USBH1_REG32(0x04)	/* Host General */
#define UH1_HWHOST		USBH1_REG32(0x08)	/* Host h/w params */
#define UH1_HWTXBUF		USBH1_REG32(0x10)	/* TX buffer h/w params */
#define UH1_HWRXBUF		USBH1_REG32(0x14)	/* RX buffer h/w params */
#define UH1_CAPLENGTH		USBH1_REG16(0x100)	/* Capability register length */
#define UH1_HCIVERSION		USBH1_REG16(0x102)	/* Host Interface version */
#define UH1_HCSPARAMS		USBH1_REG32(0x104)	/* Host control structural params */
#define UH1_HCCPARAMS		USBH1_REG32(0x108)	/* control capability params */
/* start EHCI registers: */
#define UH1_USBCMD		USBH1_REG32(0x140)	/* USB command register */
#define UH1_USBSTS		USBH1_REG32(0x144)	/* USB status register */
#define UH1_USBINTR		USBH1_REG32(0x148)	/* interrupt enable register */
#define UH1_FRINDEX		USBH1_REG32(0x14c)	/* USB frame index */
/*      segment                            (0x150)	   addr bits 63:32 if needed */
#define UH1_PERIODICLISTBASE	USBH1_REG32(0x154)	/* host crtlr frame list base addr */
#define UH1_ASYNCLISTADDR	USBH1_REG32(0x158)	/* host ctrlr nest async addr */
#define UH1_BURSTSIZE		USBH1_REG32(0x160)	/* host ctrlr embedded TT async buf status */
#define UH1_TXFILLTUNING	USBH1_REG32(0x164)	/* TX FIFO fill tuning */
/*      configured_flag                    (0x180)	   configflag (supports HS) */
#define UH1_PORTSC1		USBH1_REG32(0x184)	/* port status and control */
/* end EHCI registers: */
#define UH1_USBMODE		USBH1_REG32(0x1a8)	/* USB device mode */

/*
 * Host 2 registers
 */
#define UH2_ID			USBH2_REG32(0x00)	/* Host ID */
#define UH2_HWGENERAL		USBH2_REG32(0x04)	/* Host General */
#define UH2_HWHOST		USBH2_REG32(0x08)	/* Host h/w params */
#define UH2_HWTXBUF		USBH2_REG32(0x10)	/* TX buffer h/w params */
#define UH2_HWRXBUF		USBH2_REG32(0x14)	/* RX buffer h/w params */
#define UH2_CAPLENGTH		USBH2_REG16(0x100)	/* Capability register length */
#define UH2_HCIVERSION		USBH2_REG16(0x102)	/* Host Interface version */
#define UH2_HCSPARAMS		USBH2_REG32(0x104)	/* Host control structural params */
#define UH2_HCCPARAMS		USBH2_REG32(0x108)	/* control capability params */
/* start EHCI registers: */
#define UH2_USBCMD		USBH2_REG32(0x140)	/* USB command register */
#define UH2_USBSTS		USBH2_REG32(0x144)	/* USB status register */
#define UH2_USBINTR		USBH2_REG32(0x148)	/* interrupt enable register */
#define UH2_FRINDEX		USBH2_REG32(0x14c)	/* USB frame index */
/*      segment                            (0x150)	   addr bits 63:32 if needed */
#define UH2_PERIODICLISTBASE	USBH2_REG32(0x154)	/* host crtlr frame list base addr */
#define UH2_ASYNCLISTADDR	USBH2_REG32(0x158)	/* host ctrlr nest async addr */
#define UH2_BURSTSIZE		USBH2_REG32(0x160)	/* host ctrlr embedded TT async buf status */
#define UH2_TXFILLTUNING	USBH2_REG32(0x164)	/* TX FIFO fill tuning */
#define UH2_ULPIVIEW		USBH2_REG32(0x170)	/* ULPI viewport */
/*      configured_flag                    (0x180)	   configflag (supports HS) */
#define UH2_PORTSC1		USBH2_REG32(0x184)	/* port status and control */
/* end EHCI registers */
#define UH2_USBMODE		USBH2_REG32(0x1a8)	/* USB device mode */

/*
 * other regs (not part of ARC core)
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

/* x_PORTSCx */
#define PORTSC_PTS_MASK			(3 << 30)	/* parallel xcvr select mask */
#define PORTSC_PTS_UTMI			(0 << 30)	/* UTMI/UTMI+ */
#define PORTSC_PTS_PHILIPS		(1 << 30)	/* Philips classic */
#define PORTSC_PTS_ULPI			(2 << 30)	/* ULPI */
#define PORTSC_PTS_SERIAL		(3 << 30)	/* serial */
#define PORTSC_STS			(1 << 29)	/* serial xcvr select */
#define PORTSC_PTW                      (1 << 28)       /* UTMI width */
#define PORTSC_PHCD                     (1 << 23)       /* Low Power Suspend */
#define PORTSC_PORT_POWER		(1 << 12)	/* port power */
#define PORTSC_LS_MASK			(3 << 10)	/* Line State mask */
#define PORTSC_LS_SE0			(0 << 10)	/* SE0     */
#define PORTSC_LS_K_STATE		(1 << 10)	/* K-state */
#define PORTSC_LS_J_STATE		(2 << 10)	/* J-state */
#define PORTSC_PORT_RESET		(1 <<  8)	/* Port reset */
#define PORTSC_PORT_SUSPEND		(1 <<  7)	/* Suspend */
#define PORTSC_PORT_FORCE_RESUME	(1 <<  6)	/* Force port resume */
#define PORTSC_OVER_CURRENT_CHG		(1 <<  5)	/* over current change */
#define PORTSC_OVER_CURRENT_ACT		(1 <<  4)	/* over currrent active */
#define PORTSC_PORT_EN_DIS_CHANGE	(1 <<  3)	/* port {en,dis}able change */
#define PORTSC_PORT_ENABLE		(1 <<  2)	/* port enabled */
#define PORTSC_CONNECT_STATUS_CHANGE	(1 <<  1)	/* connect status change */
#define PORTSC_CURRENT_CONNECT_STATUS	(1 <<  0)	/* current connect status */

#define PORTSC_W1C_BITS                     \
	(PORTSC_CONNECT_STATUS_CHANGE | \
	PORTSC_PORT_EN_DIS_CHANGE | \
	PORTSC_OVER_CURRENT_CHG)

/* UOG_OTGSC Register Bits */
/* control bits: */
#define  OTGSC_CTRL_VBUS_DISCHARGE	(1 <<  0)
#define  OTGSC_CTRL_VBUS_CHARGE		(1 <<  1)
#define  OTGSC_CTRL_OTG_TERM		(1 <<  3)	/* controls DM pulldown */
#define  OTGSC_CTRL_DATA_PULSING	(1 <<  4)
#define  OTGSC_CTRL_USB_ID_PU		(1 <<  5)	/* enable ID pullup */
/* current status: (R/O) */
#define  OTGSC_STS_USB_ID		(1 <<  8)	/* 0=A-device  1=B-device */
#define  OTGSC_STS_A_VBUS_VALID		(1 <<  9)
#define  OTGSC_STS_A_SESSION_VALID	(1 << 10)
#define  OTGSC_STS_B_SESSION_VALID	(1 << 11)
#define  OTGSC_STS_B_SESSION_END	(1 << 12)
#define  OTGSC_STS_1ms_TIMER		(1 << 13)
#define  OTGSC_STS_DATA_PULSE		(1 << 14)
/* interrupt status: (write to clear) */
#define  OTGSC_IS_MASK			(0x7f << 16)
#define  OTGSC_IS_USB_ID		(1 << 16)
#define  OTGSC_IS_A_VBUS_VALID		(1 << 17)
#define  OTGSC_IS_A_SESSION_VALID	(1 << 18)
#define  OTGSC_IS_B_SESSION_VALID	(1 << 19)
#define  OTGSC_IS_B_SESSION_END		(1 << 20)
#define  OTGSC_IS_1ms_TIMER		(1 << 21)
#define  OTGSC_IS_DATA_PULSE		(1 << 22)
/* interrupt enables: */
#define  OTGSC_IE_MASK			(0x7f << 24)
#define  OTGSC_IE_USB_ID		(1 << 24)
#define  OTGSC_IE_A_VBUS_VALID		(1 << 25)
#define  OTGSC_IE_A_SESSION_VALID	(1 << 26)
#define  OTGSC_IE_B_SESSION_VALID	(1 << 27)
#define  OTGSC_IE_B_SESSION_END		(1 << 28)
#define  OTGSC_IE_1ms_TIMER		(1 << 29)
#define  OTGSC_IE_DATA_PULSE		(1 << 30)
/* UOG_USBSTS bits */
#define USBSTS_PCI			(1 << 2) /* Port Change Detect */
#define USBSTS_URI			(1 << 6) /* USB Reset Received */

#if 1				/* FIXME these here for compatibility between my names and Leo's */
/* OTG interrupt enable bit masks */
#define  OTGSC_INTERRUPT_ENABLE_BITS_MASK	OTGSC_IE_MASK
#define  OTGSC_INTSTS_MASK			OTGSC_IS_MASK

/* OTG interrupt status bit masks */
#define  OTGSC_INTERRUPT_STATUS_BITS_MASK	OTGSC_IS_MASK
#endif

/* x_USBMODE */
#define USBMODE_SLOM		(1 << 3)	/* setup lockout mode */
#define USBMODE_ES		(1 << 2)	/* (big) endian select */
#define USBMODE_CM_MASK		(3 << 0)	/* controller mode mask */
#define USBMODE_CM_HOST		(3 << 0)	/* host */
#define USBMODE_CM_DEVICE	(2 << 0)	/* device */
#define USBMODE_CM_reserved	(1 << 0)	/* reserved */

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

/* USBCMD */
#define UCMD_RUN_STOP           (1 << 0)        /* controller run/stop */
#define UCMD_RESET		(1 << 1)	/* controller reset */
#define UCMD_ITC_NO_THRESHOLD	 (~(0xff << 16))	/* Interrupt Threshold Control */

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

#define HCSPARAMS_PPC           (0x1<<4)        /* Port Power Control */
/* USB Clock on/off Control Register */
#define OTG_AHBCLK_OFF          (0x1<<17)      /* 1: OFF */
#define H1_AHBCLK_OFF           (0x1<<18)      /* 1: OFF */
extern enum fsl_usb2_modes get_usb_mode(struct fsl_usb2_platform_data *pdata);
#endif
