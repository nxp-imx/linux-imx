/*
 * Copyright 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __MXC_UART_REG_H__
#define __MXC_UART_REG_H__

/* Address offsets of the UART registers */
#define MXC_UARTURXD            0x000	/* Receive reg */
#define MXC_UARTUTXD            0x040	/* Transmitter reg */
#define	MXC_UARTUCR1            0x080	/* Control reg 1 */
#define MXC_UARTUCR2            0x084	/* Control reg 2 */
#define MXC_UARTUCR3            0x088	/* Control reg 3 */
#define MXC_UARTUCR4            0x08C	/* Control reg 4 */
#define MXC_UARTUFCR            0x090	/* FIFO control reg */
#define MXC_UARTUSR1            0x094	/* Status reg 1 */
#define MXC_UARTUSR2            0x098	/* Status reg 2 */
#define MXC_UARTUESC            0x09C	/* Escape character reg */
#define MXC_UARTUTIM            0x0A0	/* Escape timer reg */
#define MXC_UARTUBIR            0x0A4	/* BRM incremental reg */
#define MXC_UARTUBMR            0x0A8	/* BRM modulator reg */
#define MXC_UARTUBRC            0x0AC	/* Baud rate count reg */
#define MXC_UARTONEMS           0x0B0	/* One millisecond reg */
#define MXC_UARTUTS             0x0B4	/* Test reg */

/* Bit definations of UCR1 */
#define MXC_UARTUCR1_ADEN       0x8000
#define MXC_UARTUCR1_ADBR       0x4000
#define MXC_UARTUCR1_TRDYEN     0x2000
#define MXC_UARTUCR1_IDEN       0x1000
#define MXC_UARTUCR1_RRDYEN     0x0200
#define MXC_UARTUCR1_RXDMAEN    0x0100
#define MXC_UARTUCR1_IREN       0x0080
#define MXC_UARTUCR1_TXMPTYEN   0x0040
#define MXC_UARTUCR1_RTSDEN     0x0020
#define MXC_UARTUCR1_SNDBRK     0x0010
#define MXC_UARTUCR1_TXDMAEN    0x0008
#define MXC_UARTUCR1_ATDMAEN    0x0004
#define MXC_UARTUCR1_DOZE       0x0002
#define MXC_UARTUCR1_UARTEN     0x0001

/* Bit definations of UCR2 */
#define MXC_UARTUCR2_ESCI       0x8000
#define MXC_UARTUCR2_IRTS       0x4000
#define MXC_UARTUCR2_CTSC       0x2000
#define MXC_UARTUCR2_CTS        0x1000
#define MXC_UARTUCR2_PREN       0x0100
#define MXC_UARTUCR2_PROE       0x0080
#define MXC_UARTUCR2_STPB       0x0040
#define MXC_UARTUCR2_WS         0x0020
#define MXC_UARTUCR2_RTSEN      0x0010
#define MXC_UARTUCR2_ATEN       0x0008
#define MXC_UARTUCR2_TXEN       0x0004
#define MXC_UARTUCR2_RXEN       0x0002
#define MXC_UARTUCR2_SRST       0x0001

/* Bit definations of UCR3 */
#define MXC_UARTUCR3_DTREN      0x2000
#define MXC_UARTUCR3_PARERREN   0x1000
#define MXC_UARTUCR3_FRAERREN   0x0800
#define MXC_UARTUCR3_DSR        0x0400
#define MXC_UARTUCR3_DCD        0x0200
#define MXC_UARTUCR3_RI         0x0100
#define MXC_UARTUCR3_RXDSEN     0x0040
#define MXC_UARTUCR3_AWAKEN     0x0010
#define MXC_UARTUCR3_DTRDEN     0x0008
#define MXC_UARTUCR3_RXDMUXSEL  0x0004
#define MXC_UARTUCR3_INVT       0x0002

/* Bit definations of UCR4 */
#define MXC_UARTUCR4_CTSTL_OFFSET       10
#define MXC_UARTUCR4_CTSTL_MASK         (0x3F << 10)
#define MXC_UARTUCR4_INVR               0x0200
#define MXC_UARTUCR4_ENIRI              0x0100
#define MXC_UARTUCR4_REF16              0x0040
#define MXC_UARTUCR4_IRSC               0x0020
#define MXC_UARTUCR4_TCEN               0x0008
#define MXC_UARTUCR4_OREN               0x0002
#define MXC_UARTUCR4_DREN               0x0001

/* Bit definations of UFCR */
#define MXC_UARTUFCR_RFDIV              0x0200	/* Ref freq div is set to 2 */
#define MXC_UARTUFCR_RFDIV_OFFSET       7
#define MXC_UARTUFCR_RFDIV_MASK         (0x7 << 7)
#define MXC_UARTUFCR_TXTL_OFFSET        10
#define MXC_UARTUFCR_DCEDTE             0x0040

/* Bit definations of URXD */
#define MXC_UARTURXD_ERR        0x4000
#define MXC_UARTURXD_OVRRUN     0x2000
#define MXC_UARTURXD_FRMERR     0x1000
#define MXC_UARTURXD_BRK        0x0800
#define MXC_UARTURXD_PRERR      0x0400

/* Bit definations of USR1 */
#define MXC_UARTUSR1_PARITYERR  0x8000
#define MXC_UARTUSR1_RTSS       0x4000
#define MXC_UARTUSR1_TRDY       0x2000
#define MXC_UARTUSR1_RTSD       0x1000
#define MXC_UARTUSR1_FRAMERR    0x0400
#define MXC_UARTUSR1_RRDY       0x0200
#define MXC_UARTUSR1_AGTIM      0x0100
#define MXC_UARTUSR1_DTRD       0x0080
#define MXC_UARTUSR1_AWAKE      0x0010

/* Bit definations of USR2 */
#define MXC_UARTUSR2_TXFE       0x4000
#define MXC_UARTUSR2_IDLE       0x1000
#define MXC_UARTUSR2_RIDELT     0x0400
#define MXC_UARTUSR2_RIIN       0x0200
#define MXC_UARTUSR2_DCDDELT    0x0040
#define MXC_UARTUSR2_DCDIN      0x0020
#define MXC_UARTUSR2_TXDC       0x0008
#define MXC_UARTUSR2_ORE        0x0002
#define MXC_UARTUSR2_RDR        0x0001

/* Bit definations of UTS */
#define MXC_UARTUTS_LOOP        0x1000

#endif				/* __MXC_UART_REG_H__ */
