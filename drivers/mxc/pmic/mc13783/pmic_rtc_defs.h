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
#ifndef __MC13783_RTC_DEFS_H__
#define __MC13783_RTC_DEFS_H__

/*!
 * @file mc13783/pmic_rtc_defs.h
 * @brief This is the internal header of PMIC(mc13783) RTC driver.
 *
 * @ingroup PMIC_RTC
 */

/*
 * RTC Time
 */
#define MC13783_RTCTIME_TIME_LSH	0
#define MC13783_RTCTIME_TIME_WID	17

/*
 * RTC Alarm
 */
#define MC13783_RTCALARM_TIME_LSH	0
#define MC13783_RTCALARM_TIME_WID	17

/*
 * RTC Day
 */
#define MC13783_RTCDAY_DAY_LSH		0
#define MC13783_RTCDAY_DAY_WID		15

/*
 * RTC Day alarm
 */
#define MC13783_RTCALARM_DAY_LSH	0
#define MC13783_RTCALARM_DAY_WID	15

#endif				/* __MC13783_RTC_DEFS_H__ */
