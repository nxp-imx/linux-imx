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
