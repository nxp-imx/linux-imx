/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/* mpr.h - Header file for Freescale MPR121 Capacitive Touch Sensor Controllor */

#ifndef MPR_H
#define MPR_H

/* Register definitions */
#define ELE_TOUCH_STATUS_0_ADDR	0x0
#define ELE_TOUCH_STATUS_1_ADDR	0X1
#define MHD_RISING_ADDR		0x2b
#define NHD_RISING_ADDR		0x2c
#define NCL_RISING_ADDR		0x2d
#define FDL_RISING_ADDR		0x2e
#define MHD_FALLING_ADDR	0x2f
#define NHD_FALLING_ADDR	0x30
#define NCL_FALLING_ADDR	0x31
#define FDL_FALLING_ADDR	0x32
#define ELE0_TOUCH_THRESHOLD_ADDR	0x41
#define ELE0_RELEASE_THRESHOLD_ADDR	0x42
/* ELE0...ELE11's threshold will set in a loop */
#define AFE_CONF_ADDR			0x5c
#define FILTER_CONF_ADDR		0x5d

/* ELECTRODE_CONF: this register is most important register, it
 * control how many of electrode is enabled. If you set this register
 * to 0x0, it make the sensor going to suspend mode. Other value(low
 * bit is non-zero) will make the sensor into Run mode.  This register
 * should be write at last.
 */
#define ELECTRODE_CONF_ADDR		0x5e
#define AUTO_CONFIG_CTRL_ADDR		0x7b
/* AUTO_CONFIG_USL: Upper Limit for auto baseline search, this
 * register is related to VDD supplied on your board, the value of
 * this register is calc by EQ: `((VDD-0.7)/VDD) * 256`.
 * AUTO_CONFIG_LSL: Low Limit of auto baseline search. This is 65% of
 * USL AUTO_CONFIG_TL: The Traget Level of auto baseline search, This
 * is 90% of USL  */
#define AUTO_CONFIG_USL_ADDR		0x7d
#define AUTO_CONFIG_LSL_ADDR		0x7e
#define AUTO_CONFIG_TL_ADDR		0x7f

/* Threshold of touch/release trigger */
#define TOUCH_THRESHOLD			0x0f
#define RELEASE_THRESHOLD		0x0a
/* Mask Button bits of STATUS_0 & STATUS_1 register */
#define TOUCH_STATUS_MASK		0xfff
/* MPR121 have 12 electrodes */
#define MPR121_MAX_KEY_COUNT		12


/**
 * @keycount: how many key maped
 * @vdd_uv: voltage of vdd supply the chip in uV
 * @matrix: maxtrix of keys
 * @wakeup: can key wake up system.
 */
struct mpr121_platform_data {
	u16 keycount;
	u16 *matrix;
	int wakeup;
	int vdd_uv;
};

#endif
