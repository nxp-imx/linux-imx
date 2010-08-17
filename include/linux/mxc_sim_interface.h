/*
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mxc_sim_interface.h
 *
 * @brief Driver for Freescale IMX SIM interface
 *
 */

#ifndef MXC_SIM_INTERFACE_H
#define MXC_SIM_INTERFACE_H

#define SIM_ATR_LENGTH_MAX 32

/* Raw ATR SIM_IOCTL_GET_ATR */
typedef struct {
	uint32_t size;		/* length of ATR received */
	uint8_t t[SIM_ATR_LENGTH_MAX];	/* raw ATR string received */
} sim_atr_t;

/* Communication parameters for SIM_IOCTL_[GET|SET]_PARAM */
typedef struct {
	uint8_t convention;	/* direct = 0, indirect = 1 */
	uint8_t FI, DI;		/* frequency multiplier and devider indices */
	uint8_t PI1, II;	/* programming voltage and current indices */
	uint8_t N;		/* extra guard time */
	uint8_t T;		/* protocol type: T0 = 0, T1 = 1 */
	uint8_t PI2;		/* programming voltage 2 value */
	uint8_t WWT;		/* working wait time */
} sim_param_t;

/* ISO7816-3 protocols */
#define SIM_PROTOCOL_T0  0
#define SIM_PROTOCOL_T1  1

/* Transfer data for SIM_IOCTL_XFER */
typedef struct {
	uint8_t *xmt_buffer;	/* transmit buffer pointer */
	int32_t xmt_length;	/* transmit buffer length */
	uint8_t *rcv_buffer;	/* receive buffer pointer */
	int32_t rcv_length;	/* receive buffer length */
	int type;		/* transfer type: TPDU = 0, PTS = 1 */
	int timeout;		/* transfer timeout in milliseconds */
	uint8_t sw1;		/* status word 1 */
	uint8_t sw2;		/* status word 2 */
} sim_xfer_t;

/* Transfer types for SIM_IOCTL_XFER */
#define SIM_XFER_TYPE_TPDU 0
#define SIM_XFER_TYPE_PTS  1

/* Interface power states */
#define SIM_POWER_OFF 0
#define SIM_POWER_ON  1

/* Return values for SIM_IOCTL_GET_PRESENSE */
#define SIM_PRESENT_REMOVED     0
#define SIM_PRESENT_DETECTED    1
#define SIM_PRESENT_OPERATIONAL 2

/* Return values for SIM_IOCTL_GET_ERROR */
#define SIM_OK                         0
#define SIM_E_ACCESS                   1
#define SIM_E_TPDUSHORT                2
#define SIM_E_PTSEMPTY                 3
#define SIM_E_INVALIDXFERTYPE          4
#define SIM_E_INVALIDXMTLENGTH         5
#define SIM_E_INVALIDRCVLENGTH         6
#define SIM_E_NACK                     7
#define SIM_E_TIMEOUT                  8
#define SIM_E_NOCARD                   9
#define SIM_E_PARAM_FI_INVALID         10
#define SIM_E_PARAM_DI_INVALID         11
#define SIM_E_PARAM_FBYD_WITHFRACTION  12
#define SIM_E_PARAM_FBYD_NOTDIVBY8OR12 13
#define SIM_E_PARAM_DIVISOR_RANGE      14
#define SIM_E_MALLOC                   15
#define SIM_E_IRQ                      16
#define SIM_E_POWERED_ON               17
#define SIM_E_POWERED_OFF              18

/* ioctl encodings */
#define SIM_IOCTL_BASE 0xc0
#define SIM_IOCTL_GET_PRESENSE   _IOR(SIM_IOCTL_BASE, 1, int)
#define SIM_IOCTL_GET_ATR        _IOR(SIM_IOCTL_BASE, 2, sim_atr_t)
#define SIM_IOCTL_GET_PARAM_ATR  _IOR(SIM_IOCTL_BASE, 3, sim_param_t)
#define SIM_IOCTL_GET_PARAM      _IOR(SIM_IOCTL_BASE, 4, sim_param_t)
#define SIM_IOCTL_SET_PARAM      _IOW(SIM_IOCTL_BASE, 5, sim_param_t)
#define SIM_IOCTL_XFER           _IOR(SIM_IOCTL_BASE, 6, sim_xfer_t)
#define SIM_IOCTL_POWER_ON       _IO(SIM_IOCTL_BASE, 7)
#define SIM_IOCTL_POWER_OFF      _IO(SIM_IOCTL_BASE, 8)
#define SIM_IOCTL_WARM_RESET     _IO(SIM_IOCTL_BASE, 9)
#define SIM_IOCTL_COLD_RESET     _IO(SIM_IOCTL_BASE, 10)
#define SIM_IOCTL_CARD_LOCK      _IO(SIM_IOCTL_BASE, 11)
#define SIM_IOCTL_CARD_EJECT     _IO(SIM_IOCTL_BASE, 12)

#endif
