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
 * @file ak4647.h
 * @brief Driver for AK4647
 *
 * @ingroup Sound
 */
#ifndef _AK4647_H_
#define _AK4647_H_

#ifdef __KERNEL__

/*!
 * AK4647 registers
 */

#define AK4647_PM1			0x00
#define AK4647_PM2			0x01
#define AK4647_SIG1		0x02
#define AK4647_SIG2		0x03
#define AK4647_MODE1		0x04
#define AK4647_MODE2		0x05
#define AK4647_TIMER		0x06
#define AK4647_ALC1		0x07
#define AK4647_ALC2		0x08

#define AK4647_LEFT_INPUT_VOLUME	0x09
#define AK4647_LEFT_DGT_VOLUME		0x0A
#define AK4647_ALC3						0x0B
#define AK4647_RIGHT_INPUT_VOLUME	0x0C
#define AK4647_RIGHT_DGT_VOLUME		0x0D
#define AK4647_MODE3						0x0E
#define AK4647_MODE4						0x0F
#define AK4647_PM3							0x10
#define AK4647_DGT_FIL_SEL				0x11

/* filter 3 coeffecient*/

#define AK4647_FIL3_COEF0	0x12
#define AK4647_FIL3_COEF1	0x13
#define AK4647_FIL3_COEF2	0x14
#define AK4647_FIL3_COEF3	0x15

/* eq coeffecient*/

#define AK4647_EQ_COEF0		0x16
#define AK4647_EQ_COEF1		0x17
#define AK4647_EQ_COEF2		0x18
#define AK4647_EQ_COEF3		0x19
#define AK4647_EQ_COEF4		0x1A
#define AK4647_EQ_COEF5		0x1B

/* filter 3 coeffecient*/

#define AK4647_FIL1_COEF0	0x1C
#define AK4647_FIL1_COEF1	0x1D
#define AK4647_FIL1_COEF2	0x1E
#define AK4647_FIL1_COEF3	0x1F

#define AK4647_REG_START	0x00
#define AK4647_REG_END		0x1F
#define AK4647_REG_NUMBER	0x20

/* clock divider id's */
#define AK4647_BCLK_CLKDIV		0
#define AK4647_MCLK_CLKDIV		1

/* bit clock div values (AK4647_BCLK_CLKDIV)*/
#define AK4647_BCLK_DIV_32		0
#define AK4647_BCLK_DIV_64		1

/* m clock div values (AK4647_MCLK_CLKDIV)*/
#define AK4647_MCLK_DIV_32		0
#define AK4647_MCLK_DIV_64		1
#define AK4647_MCLK_DIV_128	2
#define AK4647_MCLK_DIV_256	3

#endif				/* __KERNEL__ */

#endif				/* _AK4647_H_ */
