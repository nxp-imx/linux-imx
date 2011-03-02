/*
 * da9052 SM (watchdog) module declarations.
 *
 * Copyright(c) 2009 Dialog Semiconductor Ltd.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __LINUX_MFD_DA9052_WDT_H
#define __LINUX_MFD_DA9052_WDT_H

#include <linux/platform_device.h>

/* To enable debug output for your module, set this to 1 */
#define	DA9052_SM_DEBUG				0

/* Error codes */
#define BUS_ERR					2
#define INIT_FAIL				3
#define SM_OPEN_FAIL				4
#define NO_IOCTL_CMD				5
#define INVALID_SCALING_VALUE			6
#define STROBING_FILTER_ERROR			7
#define TIMER_DELETE_ERR			8
#define STROBING_MODE_ERROR			9

/* IOCTL Switch */
/* For strobe watchdog function */
#define DA9052_SM_IOCTL_STROBE_WDT		1

/* For setting watchdog timer time */
#define DA9052_SM_IOCTL_SET_WDT			2

/* For enabling/disabling strobing filter */
#define DA9052_SM_IOCTL_SET_STROBING_FILTER	3

/* For enabling/disabling strobing filter */
#define DA9052_SM_IOCTL_SET_STROBING_MODE	4

/* Watchdog time scaling TWDMAX scaling macros */
#define DA9052_WDT_DISABLE			0
#define DA9052_SCALE_1X				1
#define DA9052_SCALE_2X				2
#define DA9052_SCALE_4X				3
#define DA9052_SCALE_8X				4
#define DA9052_SCALE_16X			5
#define DA9052_SCALE_32X			6
#define DA9052_SCALE_64X			7

#define DA9052_STROBE_WIN_FILTER_PER		80
#define DA9052_X1_WINDOW	((1 * 2048 * DA9052_STROBE_WIN_FILTER_PER)/100)
#define DA9052_X2_WINDOW	((2 * 2048 * DA9052_STROBE_WIN_FILTER_PER)/100)
#define DA9052_X4_WINDOW	((4 * 2048 * DA9052_STROBE_WIN_FILTER_PER)/100)
#define DA9052_X8_WINDOW	((8 * 2048 * DA9052_STROBE_WIN_FILTER_PER)/100)
#define DA9052_X16_WINDOW	((16 * 2048 * DA9052_STROBE_WIN_FILTER_PER)/100)
#define DA9052_X32_WINDOW	((32 * 2048 * DA9052_STROBE_WIN_FILTER_PER)/100)
#define DA9052_X64_WINDOW	((64 * 2048 * DA9052_STROBE_WIN_FILTER_PER)/100)

#define DA9052_STROBE_AUTO			1
#define DA9052_STROBE_MANUAL			0

#define DA9052_SM_STROBE_CONF			DISABLE

#define DA9052_ADC_TWDMIN_TIME			500

void start_strobing(struct work_struct *work);
/* Create a handler for the scheduling start_strobing function */
DECLARE_WORK(strobing_action, start_strobing);

#endif /* __LINUX_MFD_DA9052_WDT_H */
