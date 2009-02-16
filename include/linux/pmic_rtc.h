/*
 * Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU Lesser General
 * Public License.  You may obtain a copy of the GNU Lesser General
 * Public License Version 2.1 or later at the following locations:
 *
 * http://www.opensource.org/licenses/lgpl-license.html
 * http://www.gnu.org/copyleft/lgpl.html
 */

#ifndef __ASM_ARCH_MXC_PMIC_RTC_H__
#define __ASM_ARCH_MXC_PMIC_RTC_H__

/*!
 * @defgroup PMIC_RTC PMIC RTC Driver
 * @ingroup PMIC_DRVRS
 */

/*!
 * @file arch-mxc/pmic_rtc.h
 * @brief This is the header of PMIC RTC driver.
 *
 * @ingroup PMIC_RTC
 */

/*
 * Includes
 */
#include <linux/ioctl.h>
#include <linux/pmic_status.h>
#include <linux/pmic_external.h>

#define         PMIC_RTC_SET_TIME                    _IOWR('p',0xd1, int)
#define         PMIC_RTC_GET_TIME                    _IOWR('p',0xd2, int)
#define         PMIC_RTC_SET_ALARM		     _IOWR('p',0xd3, int)
#define         PMIC_RTC_GET_ALARM		     _IOWR('p',0xd4, int)
#define         PMIC_RTC_WAIT_ALARM		     _IOWR('p',0xd5, int)
#define         PMIC_RTC_ALARM_REGISTER              _IOWR('p',0xd6, int)
#define         PMIC_RTC_ALARM_UNREGISTER            _IOWR('p',0xd7, int)

/*!
 * This enumeration define all RTC interrupt
 */
typedef enum {
	/*!
	 * Time of day alarm
	 */
	RTC_IT_ALARM,
	/*!
	 * 1 Hz timetick
	 */
	RTC_IT_1HZ,
	/*!
	 * RTC reset occurred
	 */
	RTC_IT_RST,
} t_rtc_int;

/*
 * RTC PMIC API
 */

/* EXPORTED FUNCTIONS */
#ifdef __KERNEL__

/*!
 * This function set the real time clock of PMIC
 *
 * @param        pmic_time  	value of date and time
 *
 * @return       This function returns PMIC_STATUS if successful.
 */
PMIC_STATUS pmic_rtc_set_time(struct timeval *pmic_time);

/*!
 * This function get the real time clock of PMIC
 *
 * @param        pmic_time  	return value of date and time
 *
 * @return       This function returns PMIC_STATUS if successful.
 */
PMIC_STATUS pmic_rtc_get_time(struct timeval *pmic_time);

/*!
 * This function set the real time clock alarm of PMIC
 *
 * @param        pmic_time  	value of date and time
 *
 * @return       This function returns PMIC_STATUS if successful.
 */
PMIC_STATUS pmic_rtc_set_time_alarm(struct timeval *pmic_time);

/*!
 * This function get the real time clock alarm of PMIC
 *
 * @param        pmic_time  	return value of date and time
 *
 * @return       This function returns PMIC_STATUS if successful.
 */
PMIC_STATUS pmic_rtc_get_time_alarm(struct timeval *pmic_time);

/*!
 * This function wait the Alarm event
 *
 * @return       This function returns PMIC_STATUS if successful.
 */
PMIC_STATUS pmic_rtc_wait_alarm(void);

/*!
 * This function is used to un/subscribe on RTC event IT.
 *
 * @param        event  	type of event.
 * @param        callback  	event callback function.
 * @param        sub      	define if Un/subscribe event.
 *
 * @return       This function returns PMIC_STATUS if successful.
 */
PMIC_STATUS pmic_rtc_event(t_rtc_int event, void *callback, bool sub);

/*!
 * This function is used to subscribe on RTC event IT.
 *
 * @param        event  	type of event.
 * @param        callback  	event callback function.
 *
 * @return       This function returns PMIC_STATUS if successful.
 */
PMIC_STATUS pmic_rtc_event_sub(t_rtc_int event, void *callback);

/*!
 * This function is used to un-subscribe on RTC event IT.
 *
 * @param        event  	type of event.
 * @param        callback  	event callback function.
 *
 * @return       This function returns PMIC_STATUS if successful.
 */
PMIC_STATUS pmic_rtc_event_unsub(t_rtc_int event, void *callback);

/*!
 * This function is used to tell if PMIC RTC has been correctly loaded.
 *
 * @return       This function returns 1 if RTC was successfully loaded
 * 		 0 otherwise.
 */
int pmic_rtc_loaded(void);

#endif				/* __KERNEL__ */

#endif				/* __ASM_ARCH_MXC_PMIC_RTC_H__ */
