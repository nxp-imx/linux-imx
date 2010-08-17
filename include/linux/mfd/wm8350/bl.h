/*
 * bl.h  --  Backlight Driver for Wolfson WM8350 PMIC
 *
 * Copyright 2009 Wolfson Microelectronics PLC
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __LINUX_MFD_WM8350_BL_H_
#define __LINUX_MFD_WM8350_BL_H_

#include <linux/fb.h>

/*
 * WM8350 Backlight platform data
 */
struct wm8350_bl_platform_data {
	int isink;
	int dcdc;
	int voltage_ramp;
	int retries;
	int max_brightness;
	int power;
	int brightness;
	int (*check_fb)(struct fb_info *);
};

#endif
