/*
 * Copyright (C) 1999 ARM Limited
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef __ASM_ARCH_MXC_ARC_OTG_H__
#define __ASM_ARCH_MXC_ARC_OTG_H__

#include "../../regs-usbphy.h"
#if defined(CONFIG_USB_STATIC_IRAM) \
    || defined(CONFIG_USB_STATIC_IRAM_PPH)
#define USB_IRAM_SIZE   SZ_8K
#else
#define USB_IRAM_SIZE 0
#endif

#define OTG_BASE_ADDR		USBCTRL0_PHYS_ADDR
#define USB_OTGREGS_BASE	(OTG_BASE_ADDR + 0x000)
/*
 * OTG registers
 */
#define UOG_ID			(0x00)	/* Host ID */
#define UOG_HWGENERAL		(0x04)	/* Host General */
#define UOG_HWHOST		(0x08)	/* Host h/w params */
#define UOG_HWTXBUF		(0x10)	/* TX buffer h/w params */
#define UOG_HWRXBUF		(0x14)	/* RX buffer h/w params */
#define UOG_CAPLENGTH		(0x100)	/* Capability register length */
#define UOG_HCIVERSION		(0x102)	/* Host Interface version */
#define UOG_HCSPARAMS		(0x104)	/* Host control structural params */
#define UOG_HCCPARAMS		(0x108)	/* control capability params */
#define UOG_DCIVERSION		(0x120)	/* device interface version */
/* start EHCI registers: */
#define UOG_USBCMD		(0x140)	/* USB command register */
#define UOG_USBSTS		(0x144)	/* USB status register */
#define UOG_USBINTR		(0x148)	/* interrupt enable register */
#define UOG_FRINDEX		(0x14c)	/* USB frame index */
/*      segment         (0x150)	   addr bits 63:32 if needed */
#define UOG_PERIODICLISTBASE	(0x154)	/* host crtlr frame list base addr */
#define UOG_DEVICEADDR		(0x154)	/* device crtlr device address */
#define UOG_ASYNCLISTADDR	(0x158)	/* host ctrlr next async addr */
#define UOG_EPLISTADDR		(0x158)	/* device ctrlr endpoint list addr */
#define UOG_BURSTSIZE		(0x160)	/* host ctrlr embedded TT
					   async buf status */
#define UOG_TXFILLTUNING	(0x164)	/* TX FIFO fill tuning */
#define UOG_ULPIVIEW		(0x170)	/* ULPI viewport */
#define UOG_CFGFLAG		(0x180)	/* configflag (supports HS) */
#define UOG_PORTSC1		(0x184)	/* port status and control */
/* end EHCI registers: */
#define UOG_OTGSC		(0x1a4)	/* OTG status and control */
#define UOG_USBMODE		(0x1a8)	/* USB device mode */
#define UOG_ENDPTSETUPSTAT	(0x1ac)	/* endpoint setup status */
#define UOG_ENDPTPRIME		(0x1b0)	/* endpoint initialization */
#define UOG_ENDPTFLUSH		(0x1b4)	/* endpoint de-initialize */
#define UOG_ENDPTSTAT		(0x1b8)	/* endpoint status */
#define UOG_ENDPTCOMPLETE	(0x1bc)	/* endpoint complete */
#define UOG_EPCTRL0		(0x1c0)	/* endpoint control0 */
/*
 * register bits
 */

/* x_PORTSCx */
#define PORTSC_PTS_MASK			(3 << 30) /* parallel xcvr mask */
#define PORTSC_PTS_UTMI			(0 << 30) /* UTMI/UTMI+ */
#define PORTSC_PTS_PHILIPS		(1 << 30) /* Philips classic */
#define PORTSC_PTS_ULPI			(2 << 30) /* ULPI */
#define PORTSC_PTS_SERIAL		(3 << 30) /* serial */
#define PORTSC_STS			(1 << 29) /* serial xcvr select */
#define PORTSC_PTW                      (1 << 28) /* UTMI width */
#define PORTSC_PHCD                     (1 << 23) /* Low Power Suspend */
#define PORTSC_PORT_POWER		(1 << 12) /* port power */
#define PORTSC_LS_MASK			(3 << 10) /* Line State mask */
#define PORTSC_LS_SE0			(0 << 10) /* SE0     */
#define PORTSC_LS_K_STATE		(1 << 10) /* K-state */
#define PORTSC_LS_J_STATE		(2 << 10) /* J-state */
#define PORTSC_PORT_RESET		(1 <<  8) /* Port reset */
#define PORTSC_PORT_SUSPEND		(1 <<  7) /* Suspend */
#define PORTSC_PORT_FORCE_RESUME	(1 <<  6) /* Force port resume */
#define PORTSC_OVER_CURRENT_CHG		(1 <<  5) /* over current change */
#define PORTSC_OVER_CURRENT_ACT		(1 <<  4) /* over currrent active */
#define PORTSC_PORT_EN_DIS_CHANGE	(1 <<  3) /* port change */
#define PORTSC_PORT_ENABLE		(1 <<  2) /* port enabled */
#define PORTSC_CONNECT_STATUS_CHANGE	(1 <<  1) /* connect status change */
#define PORTSC_CURRENT_CONNECT_STATUS	(1 <<  0) /* current connect status */

#define PORTSC_W1C_BITS	\
	(PORTSC_CONNECT_STATUS_CHANGE |	\
	PORTSC_PORT_EN_DIS_CHANGE	|	\
	PORTSC_OVER_CURRENT_CHG)

/* UOG_OTGSC Register Bits */
/* control bits: */
#define  OTGSC_CTRL_VBUS_DISCHARGE	(1 << 0)
#define  OTGSC_CTRL_VBUS_CHARGE		(1 << 1)
/* controls DM pulldown */
#define  OTGSC_CTRL_OTG_TERM		(1 << 3)
#define  OTGSC_CTRL_DATA_PULSING	(1 << 4)
#define  OTGSC_CTRL_USB_ID_PU		(1 << 5)
/* current status: (R/O) */
/* 0=A-device  1=B-device */
#define  OTGSC_STS_USB_ID		(1 << 8)
#define  OTGSC_STS_A_VBUS_VALID		(1 << 9)
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

#if 1	/* FIXME these here for compatibility between my names and Leo's */
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

/* USBCMD */
#define UCMD_RUN_STOP           (1 << 0)        /* controller run/stop */
#define UCMD_RESET		(1 << 1)	/* controller reset */
#define UCMD_ITC_NO_THRESHOLD	(~(0xff << 16))	/* Interrupt Threshold */

/* OTG_MIRROR */
#define OTGM_SESEND		(1 << 4)	/* B device session end */
#define OTGM_VBUSVAL		(1 << 3)	/* Vbus valid */
#define OTGM_BSESVLD		(1 << 2)	/* B session Valid */
#define OTGM_ASESVLD		(1 << 1)	/* A session Valid */
#define OTGM_IDIDG		(1 << 0)	/* OTG ID pin status */
				/* 1=high: Operate as B-device */
				/* 0=low : Operate as A-device */

#define HCSPARAMS_PPC           (0x1<<4)        /* Port Power Control */

extern enum fsl_usb2_modes get_usb_mode(struct fsl_usb2_platform_data *pdata);
#endif
