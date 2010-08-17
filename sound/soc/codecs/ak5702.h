/*
 * ak5702.h  --  AK5702 Soc Audio driver
 *
 * Copyright 2009-2010 Freescale Semiconductor, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _AK5702_H
#define _AK5702_H

/* AK5702 register space */

#define AK5702_PM1		0x00
#define AK5702_PLL1		0x01
#define AK5702_SIG1		0x02
#define AK5702_MICG1		0x03
#define AK5702_FMT1		0x04
#define AK5702_FS1		0x05
#define AK5702_CLK1		0x06
#define AK5702_VOL1		0x07
#define AK5702_LVOL1		0x08
#define AK5702_RVOL1		0x09
#define AK5702_TIMER1		0x0a
#define AK5702_ALC11		0x0b
#define AK5702_ALC12		0x0c
#define AK5702_MODE11		0x0d
#define AK5702_MODE12		0x0e
#define AK5702_MODE13		0x0f

#define AK5702_PM2		0x10
#define AK5702_PLL2		0x11
#define AK5702_SIG2		0x12
#define AK5702_MICG2		0x13
#define AK5702_FMT2		0x14
#define AK5702_FS2		0x15
#define AK5702_CLK2		0x16
#define AK5702_VOL2		0x17
#define AK5702_LVOL2		0x18
#define AK5702_RVOL2		0x19
#define AK5702_TIMER2		0x1a
#define AK5702_ALC21		0x1b
#define AK5702_ALC22		0x1c
#define AK5702_MODE21		0x1d
#define AK5702_MODE22		0x1e

#define AK5702_CACHEREGNUM	0x1F

#define AK5702_PM1_PMADAL	0x01
#define AK5702_PM1_PMADAR	0x02
#define AK5702_PM1_PMVCM	0x04
#define AK5702_PM2_PMADBL	0x01
#define AK5702_PM2_PMADBR	0x02

#define AK5702_PLL1_POWERDOWN	0x0
#define AK5702_PLL1_POWERUP	0x01
#define AK5702_PLL1_MASTER	0x02
#define AK5702_PLL1_SLAVE	0x0
#define AK5702_PLL1_11289600	0x10
#define AK5702_PLL1_12000000	0x24
#define AK5702_PLL1_12288000	0x14
#define AK5702_PLL1_19200000	0x20

#define AK5702_SIG1_L_LIN1	0x0
#define AK5702_SIG1_L_LIN2	0x01
#define AK5702_SIG1_R_RIN1	0x0
#define AK5702_SIG1_R_RIN2	0x02
#define AK5702_SIG1_PMMPA	0x10
#define AK5702_SIG2_L_LIN3	0x0
#define AK5702_SIG2_L_LIN4	0x01
#define AK5702_SIG2_R_RIN3	0x0
#define AK5702_SIG2_R_RIN4	0x02
#define AK5702_SIG2_PMMPB	0x10

#define AK5702_MICG1_INIT	0x0
#define AK5702_MICG2_INIT	0x0

#define AK5702_FMT1_I2S	0x23
#define AK5702_FMT1_MSB	0x22
#define AK5702_FMT2_STEREO	0x20
#define AK5702_FS1_BCKO_32FS	0x10
#define AK5702_FS1_BCKO_64FS	0x20
#define AK5702_CLK1_PS_256FS	0x0
#define AK5702_CLK1_PS_128FS	0x01
#define AK5702_CLK1_PS_64FS	0x02
#define AK5702_CLK1_PS_32FS	0x03
#define AK5702_VOL1_IVOLAC	0x01
#define AK5702_VOL2_IVOLBC	0x01
#define AK5702_LVOL1_INIT	0x91
#define AK5702_RVOL1_INIT	0x91
#define AK5702_LVOL2_INIT	0x91
#define AK5702_RVOL2_INIT	0x91

#define AK5702_PLL1_PM_MASK	0x01
#define AK5702_PLL1_MODE_MASK	0x02
#define AK5702_PLL1_PLL_MASK	0x3c
#define AK5702_FS1_BCKO_MASK	0x30
#define AK5702_FS1_FS_MASK	0x0f
#define AK5702_CLK1_PS_MASK	0x03

/* clock divider id */
#define AK5702_BCLK_CLKDIV	0
#define AK5702_MCLK_CLKDIV	1

/* bit clock div values */
#define AK5702_BCLK_DIV_32	0
#define AK5702_BCLK_DIV_64	1

/* m clock div values */
#define AK5702_MCLK_DIV_32	0
#define AK5702_MCLK_DIV_64	1
#define AK5702_MCLK_DIV_128	2
#define AK5702_MCLK_DIV_256	3

/* PLL master and slave modes */
#define AK5702_PLL_POWERDOWN	0
#define AK5702_PLL_MASTER	1
#define AK5702_PLL_SLAVE	2

struct ak5702_setup_data {
	int i2c_bus;
	unsigned short i2c_address;
};

extern struct snd_soc_dai ak5702_dai;
extern struct snd_soc_codec_device soc_codec_dev_ak5702;

#endif
