/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

/*!
 * Header file containing the public API for the System Controller (SC)
 * Timer function.
 *
 * @addtogroup TIMER_SVC (SVC) Timer Service
 *
 * Module for the Timer service. This includes support for the watchdog, RTC,
 * and system counter. Note every resource partition has a watchdog it can
 * use.
 *
 * @{
 */

#ifndef _SC_TIMER_API_H
#define _SC_TIMER_API_H

/* Includes */

#include <soc/imx8/sc/types.h>
#include <soc/imx8/sc/svc/rm/api.h>

/* Defines */

/* Types */

/*!
 * This type is used to declare a watchdog time value in milliseconds.
 */
typedef uint32_t sc_timer_wdog_time_t;

/* Functions */

/*!
 * @name Wathdog Functions
 * @{
 */

/*!
 * This function sets the watchdog timeout in milliseconds. If not
 * set then the timeout defaults to the max. Once locked this value
 * cannot be changed.
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     timeout     timeout period for the watchdog
 *
 * @return Returns an error code (SC_ERR_NONE = success, SC_ERR_LOCKED
 *         = locked).
 */
sc_err_t sc_timer_set_wdog_timeout(sc_ipc_t ipc, sc_timer_wdog_time_t timeout);

/*!
 * This function starts the watchdog.
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     lock        boolean indicating the lock status
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 *
 * If \a lock is set then the watchdog cannot be stopped or the timeout
 * period changed.
 */
sc_err_t sc_timer_start_wdog(sc_ipc_t ipc, bool lock);

/*!
 * This function stops the watchdog if it is not locked.
 *
 * @param[in]     ipc         IPC handle
 *
 * @return Returns an error code (SC_ERR_NONE = success, SC_ERR_LOCKED
 *         = locked).
 */
sc_err_t sc_timer_stop_wdog(sc_ipc_t ipc);

/*!
 * This function pings (services, kicks) the watchdog resetting the time
 * before expiration back to the timeout.
 *
 * @param[in]     ipc         IPC handle
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 */
sc_err_t sc_timer_ping_wdog(sc_ipc_t ipc);

/*!
 * This function gets the status of the watchdog. All arguments are
 * in milliseconds.
 *
 * @param[in]     ipc         IPC handle
 * @param[out]    timeout         pointer to return the timeout
 * @param[out]    max_timeout     pointer to return the max timeout
 * @param[out]    remaining_time  pointer to return the time remaining
 *                                until trigger
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 */
sc_err_t sc_timer_get_wdog_status(sc_ipc_t ipc,
				  sc_timer_wdog_time_t *timeout,
				  sc_timer_wdog_time_t *max_timeout,
				  sc_timer_wdog_time_t *remaining_time);

/* @} */

/*!
 * @name Real-Time Clock (RTC) Functions
 * @{
 */

/*!
 * This function sets the RTC time. Only the owner of the SC_R_SYSTEM
 * resource can set the time.
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     year        year (min 1970)
 * @param[in]     mon         month (1-12)
 * @param[in]     day         day of the month (1-31)
 * @param[in]     hour        hour (0-23)
 * @param[in]     min         minute (0-59)
 * @param[in]     sec         second (0-59)
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 *
 * Return errors:
 * - SC_ERR_PARM if invalid time/date parameters,
 * - SC_ERR_NOACCESS if caller's partition is not the RTC owner
 */
sc_err_t sc_timer_set_rtc_time(sc_ipc_t ipc, uint16_t year, uint8_t mon,
			       uint8_t day, uint8_t hour, uint8_t min,
			       uint8_t sec);

/*!
 * This function gets the RTC time.
 *
 * @param[in]     ipc         IPC handle
 * @param[out]    year        pointer to return year (min 1970)
 * @param[out]    mon         pointer to return month (1-12)
 * @param[out]    day         pointer to return day of the month (1-31)
 * @param[out]    hour        pointer to return hour (0-23)
 * @param[out]    min         pointer to return minute (0-59)
 * @param[out]    sec         pointer to return second (0-59)
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 */
sc_err_t sc_timer_get_rtc_time(sc_ipc_t ipc, uint16_t *year, uint8_t *mon,
			       uint8_t *day, uint8_t *hour, uint8_t *min,
			       uint8_t *sec);

/*!
 * This function sets the RTC alarm. Only the owner of the SC_R_SYSTEM
 * resource can set the alarm.
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     year        year (min 1970)
 * @param[in]     mon         month (1-12)
 * @param[in]     day         day of the month (1-31)
 * @param[in]     hour        hour (0-23)
 * @param[in]     min         minute (0-59)
 * @param[in]     sec         second (0-59)
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 *
 * Return errors:
 * - SC_ERR_PARM if invalid time/date parameters,
 * - SC_ERR_NOACCESS if caller's partition is not the RTC owner
 */
sc_err_t sc_timer_set_rtc_alarm(sc_ipc_t ipc, uint16_t year, uint8_t mon,
				uint8_t day, uint8_t hour, uint8_t min,
				uint8_t sec);

/*!
 * This function gets the RTC time in seconds since 1/1/1970.
 *
 * @param[in]     ipc         IPC handle
 * @param[out]    sec         pointer to return second
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 */
sc_err_t sc_timer_get_rtc_sec1970(sc_ipc_t ipc, uint32_t *sec);

/* @} */

#endif				/* _SC_TIMER_API_H */

/**@}*/
