/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mxc_srtc.h
 *
 * @brief SRTC IOCTL definitions
 *
 * @ingroup RTC
 */


#define RTC_READ_TIME_47BIT	_IOR('p', 0x20, unsigned long long)
/* blocks until LPSCMR is set, returns difference */
#define RTC_WAIT_TIME_SET	_IOR('p', 0x21, int64_t)
