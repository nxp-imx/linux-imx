/*
 * Copyright 2005-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <mach/hardware.h>

/*
 * register bits
 */

/* x_PORTSCx */
#define PORTSC_PTS_MASK			(3 << 30)/* parallel xcvr select mask */
#define PORTSC_PTS_UTMI			(0 << 30)/* UTMI/UTMI+ */
#define PORTSC_PTS_PHILIPS		(1 << 30)/* Philips classic */
#define PORTSC_PTS_ULPI			(2 << 30)/* ULPI */
#define PORTSC_PTS_SERIAL		(3 << 30)/* serial */
#define PORTSC_STS			(1 << 29)/* serial xcvr select */
#define PORTSC_PTW                      (1 << 28)/* UTMI width */
#define PORTSC_PORT_POWER		(1 << 12)/* port power */
#define PORTSC_LS_MASK			(3 << 10)/* Line State mask */
#define PORTSC_LS_SE0			(0 << 10)/* SE0     */
#define PORTSC_LS_K_STATE		(1 << 10)/* K-state */
#define PORTSC_LS_J_STATE		(2 << 10)/* J-state */
#define PORTSC_PORT_RESET		(1 <<  8)/* Port reset */
#define PORTSC_PORT_SUSPEND		(1 <<  7)/* Suspend */
#define PORTSC_PORT_FORCE_RESUME	(1 <<  6)/* Force port resume */
#define PORTSC_OVER_CURRENT_CHG		(1 <<  5)/* over current change */
#define PORTSC_OVER_CURRENT_ACT		(1 <<  4)/* over currrent active */
#define PORTSC_PORT_EN_DIS_CHANGE	(1 <<  3)/* port {en,dis}able change */
#define PORTSC_PORT_ENABLE		(1 <<  2)/* port enabled */
#define PORTSC_CONNECT_STATUS_CHANGE	(1 <<  1)/* connect status change */
#define PORTSC_CURRENT_CONNECT_STATUS	(1 <<  0)/* current connect status */

#define PORTSC_W1C_BITS (PORTSC_CONNECT_STATUS_CHANGE |	\
	PORTSC_PORT_EN_DIS_CHANGE |	\
	PORTSC_OVER_CURRENT_CHG)

/* UOG_OTGSC Register Bits */
/* control bits: */
#define  OTGSC_CTRL_VBUS_DISCHARGE	(1 <<  0)
#define  OTGSC_CTRL_VBUS_CHARGE		(1 <<  1)
#define  OTGSC_CTRL_OTG_TERM		(1 <<  3)/* controls DM pulldown */
#define  OTGSC_CTRL_DATA_PULSING	(1 <<  4)
#define  OTGSC_CTRL_USB_ID_PU		(1 <<  5)/* enable ID pullup */
/* current status: (R/O) */
#define  OTGSC_STS_USB_ID		(1 <<  8)/* 0=A-device  1=B-device */
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
#define UCMD_ITC_NO_THRESHOLD	(~(0xff << 16))/* Interrupt Threshold Control */

#define HCSPARAMS_PPC           (0x1<<4)        /* Port Power Control */
#endif
