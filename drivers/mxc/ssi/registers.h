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

 /*!
  * @file ../ssi/registers.h
  * @brief This header file contains SSI driver low level definition to access module registers.
  *
  * @ingroup SSI
  */

#ifndef __MXC_SSI_REGISTERS_H__
#define __MXC_SSI_REGISTERS_H__

/*!
 * This include to define bool type, false and true definitions.
 */
#include <mach/hardware.h>

#define SPBA_CPU_SSI            0x07

#define    MXC_SSISTX0		0x00
#define    MXC_SSISTX1		0x04
#define    MXC_SSISRX0		0x08
#define    MXC_SSISRX1   	0x0C
#define    MXC_SSISCR           0x10
#define    MXC_SSISISR          0x14
#define    MXC_SSISIER   	0x18
#define    MXC_SSISTCR   	0x1C
#define    MXC_SSISRCR   	0x20
#define    MXC_SSISTCCR  	0x24
#define    MXC_SSISRCCR  	0x28
#define    MXC_SSISFCSR  	0x2C
#define    MXC_SSISTR           0x30
#define    MXC_SSISOR           0x34
#define    MXC_SSISACNT  	0x38
#define    MXC_SSISACADD 	0x3C
#define    MXC_SSISACDAT 	0x40
#define    MXC_SSISATAG  	0x44
#define    MXC_SSISTMSK  	0x48
#define    MXC_SSISRMSK  	0x4C

/* MXC 91221 only */
#define    MXC_SSISACCST  	0x50
#define    MXC_SSISACCEN  	0x54
#define    MXC_SSISACCDIS  	0x58

/*! SSI1 registers offset*/
#define    MXC_SSI1STX0		0x00
#define    MXC_SSI1STX1		0x04
#define    MXC_SSI1SRX0		0x08
#define    MXC_SSI1SRX1   	0x0C
#define    MXC_SSI1SCR          0x10
#define    MXC_SSI1SISR         0x14
#define    MXC_SSI1SIER   	0x18
#define    MXC_SSI1STCR   	0x1C
#define    MXC_SSI1SRCR   	0x20
#define    MXC_SSI1STCCR  	0x24
#define    MXC_SSI1SRCCR  	0x28
#define    MXC_SSI1SFCSR  	0x2C
#define    MXC_SSI1STR          0x30
#define    MXC_SSI1SOR          0x34
#define    MXC_SSI1SACNT  	0x38
#define    MXC_SSI1SACADD 	0x3C
#define    MXC_SSI1SACDAT 	0x40
#define    MXC_SSI1SATAG  	0x44
#define    MXC_SSI1STMSK  	0x48
#define    MXC_SSI1SRMSK  	0x4C

/* MXC91221 only */

#define    MXC_SSISACCST  	0x50
#define    MXC_SSISACCEN  	0x54
#define    MXC_SSISACCDIS  	0x58

/* Not on MXC91221 */
/*! SSI2 registers offset*/
#define    MXC_SSI2STX0   	0x00
#define    MXC_SSI2STX1         0x04
#define    MXC_SSI2SRX0    	0x08
#define    MXC_SSI2SRX1         0x0C
#define    MXC_SSI2SCR          0x10
#define    MXC_SSI2SISR         0x14
#define    MXC_SSI2SIER         0x18
#define    MXC_SSI2STCR         0x1C
#define    MXC_SSI2SRCR         0x20
#define    MXC_SSI2STCCR  	0x24
#define    MXC_SSI2SRCCR   	0x28
#define    MXC_SSI2SFCSR        0x2C
#define    MXC_SSI2STR    	0x30
#define    MXC_SSI2SOR    	0x34
#define    MXC_SSI2SACNT  	0x38
#define    MXC_SSI2SACADD 	0x3C
#define    MXC_SSI2SACDAT 	0x40
#define    MXC_SSI2SATAG  	0x44
#define    MXC_SSI2STMSK  	0x48
#define    MXC_SSI2SRMSK  	0x4C

/*!
 * SCR Register bit shift definitions
 */
#define SSI_ENABLE_SHIFT            0
#define SSI_TRANSMIT_ENABLE_SHIFT   1
#define SSI_RECEIVE_ENABLE_SHIFT    2
#define SSI_NETWORK_MODE_SHIFT      3
#define SSI_SYNCHRONOUS_MODE_SHIFT  4
#define SSI_I2S_MODE_SHIFT          5
#define SSI_SYSTEM_CLOCK_SHIFT      7
#define SSI_TWO_CHANNEL_SHIFT       8
#define SSI_CLOCK_IDLE_SHIFT        9

/* MXC91221 only*/
#define SSI_TX_FRAME_CLOCK_DISABLE_SHIFT        10
#define SSI_RX_FRAME_CLOCK_DISABLE_SHIFT        11

/*!
 * STCR & SRCR Registers bit shift definitions
 */
#define SSI_EARLY_FRAME_SYNC_SHIFT    0
#define SSI_FRAME_SYNC_LENGTH_SHIFT   1
#define SSI_FRAME_SYNC_INVERT_SHIFT   2
#define SSI_CLOCK_POLARITY_SHIFT      3
#define SSI_SHIFT_DIRECTION_SHIFT     4
#define SSI_CLOCK_DIRECTION_SHIFT     5
#define SSI_FRAME_DIRECTION_SHIFT     6
#define SSI_FIFO_ENABLE_0_SHIFT       7
#define SSI_FIFO_ENABLE_1_SHIFT       8
#define SSI_BIT_0_SHIFT               9

/* MXC91221 only*/
#define SSI_TX_FRAME_CLOCK_DISABLE_SHIFT        10
#define SSI_RX_DATA_EXTENSION_SHIFT   10	/*SRCR only */
/*!
 * STCCR & SRCCR Registers bit shift definitions
 */
#define SSI_PRESCALER_MODULUS_SHIFT          0
#define SSI_FRAME_RATE_DIVIDER_SHIFT         8
#define SSI_WORD_LENGTH_SHIFT               13
#define SSI_PRESCALER_RANGE_SHIFT           17
#define SSI_DIVIDE_BY_TWO_SHIFT             18
#define SSI_FRAME_DIVIDER_MASK              31
#define SSI_MIN_FRAME_DIVIDER_RATIO          1
#define SSI_MAX_FRAME_DIVIDER_RATIO         32
#define SSI_PRESCALER_MODULUS_MASK         255
#define SSI_MIN_PRESCALER_MODULUS_RATIO      1
#define SSI_MAX_PRESCALER_MODULUS_RATIO    256
#define SSI_WORD_LENGTH_MASK                15

#define SSI_IRQ_STATUS_NUMBER        25

/*!
 * SFCSR Register bit shift definitions
 */
#define SSI_RX_FIFO_1_COUNT_SHIFT       28
#define SSI_TX_FIFO_1_COUNT_SHIFT       24
#define SSI_RX_FIFO_1_WATERMARK_SHIFT   20
#define SSI_TX_FIFO_1_WATERMARK_SHIFT   16
#define SSI_RX_FIFO_0_COUNT_SHIFT       12
#define SSI_TX_FIFO_0_COUNT_SHIFT        8
#define SSI_RX_FIFO_0_WATERMARK_SHIFT    4
#define SSI_TX_FIFO_0_WATERMARK_SHIFT    0
#define SSI_MIN_FIFO_WATERMARK           0
#define SSI_MAX_FIFO_WATERMARK           8

/*!
 * SSI Option Register (SOR) bit shift definitions
 */
#define SSI_FRAME_SYN_RESET_SHIFT        0
#define SSI_WAIT_SHIFT                   1
#define SSI_INIT_SHIFT                   3
#define SSI_TRANSMITTER_CLEAR_SHIFT      4
#define SSI_RECEIVER_CLEAR_SHIFT         5
#define SSI_CLOCK_OFF_SHIFT              6
#define SSI_WAIT_STATE_MASK            0x3

/*!
 * SSI AC97 Control Register (SACNT) bit shift definitions
 */
#define AC97_MODE_ENABLE_SHIFT           0
#define AC97_VARIABLE_OPERATION_SHIFT    1
#define AC97_TAG_IN_FIFO_SHIFT           2
#define AC97_READ_COMMAND_SHIFT          3
#define AC97_WRITE_COMMAND_SHIFT         4
#define AC97_FRAME_RATE_DIVIDER_SHIFT    5
#define AC97_FRAME_RATE_MASK          0x3F

/*!
 * SSI Test Register (STR) bit shift definitions
 */
#define SSI_TEST_MODE_SHIFT               15
#define SSI_RCK2TCK_SHIFT                 14
#define SSI_RFS2TFS_SHIFT                 13
#define SSI_RXSTATE_SHIFT                 8
#define SSI_TXD2RXD_SHIFT                 7
#define SSI_TCK2RCK_SHIFT                 6
#define SSI_TFS2RFS_SHIFT                 5
#define SSI_TXSTATE_SHIFT                 0

#endif				/* __MXC_SSI_REGISTERS_H__ */
