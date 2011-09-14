/*
 * da9052 TSI configuration module declarations.
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

#ifndef __LINUX_MFD_DA9052_TSI_CFG_H
#define __LINUX_MFD_DA9052_TSI_CFG_H

#define		DA9052_TSI_DEBUG	0

#define AUTO_MODE			0
#define IDLE				1
#define DEFAULT_TSI_STATE		AUTO_MODE

#define TSI_SLOT_SKIP_VALUE		0

#define TSI_DELAY_VALUE			3

#define TSI_MODE_VALUE			0

#define ENABLE_AVERAGE_FILTER		1

#define DEFAULT_AVERAGE_FILTER_SIZE	3

#define ENABLE_WINDOW_FILTER		1

#define TSI_X_WINDOW_SIZE		50
#define TSI_Y_WINDOW_SIZE		50

#define SAMPLE_CNT_FOR_WIN_REF		3

#define TSI_ECONOMY_MODE		0
#define TSI_FAST_MODE			1
#define DEFAULT_TSI_SAMPLING_MODE	TSI_FAST_MODE

#define TSI_USE_CALIBRATION   		1

#define DA9052_TSI_CALIB_AN		1
#define DA9052_TSI_CALIB_BN		0
#define DA9052_TSI_CALIB_CN		0
#define DA9052_TSI_CALIB_DN		0
#define DA9052_TSI_CALIB_EN		1
#define DA9052_TSI_CALIB_FN		0
#define DA9052_TSI_CALIB_DIVIDER	1

#define TS_X_MIN	(0)
#define TS_X_MAX	(1023)
#define TS_Y_MIN	(0)
#define TS_Y_MAX	(1023)

#define DISPLAY_X_MIN	(0)
#define DISPLAY_X_MAX	(1023)
#define DISPLAY_Y_MIN	(0)
#define DISPLAY_Y_MAX	(1023)

#define ENABLE_TSI_DEBOUNCE		0

#define TSI_DEBOUNCE_DATA_CNT		3


#define RELEASE
#define DA9052_TSI_RAW_DATA_PROFILING		0
#define DA9052_TSI_WIN_FLT_DATA_PROFILING  	0
#define DA9052_TSI_AVG_FLT_DATA_PROFILING 	0
#define DA9052_TSI_CALIB_DATA_PROFILING 	0
#define DA9052_TSI_OS_DATA_PROFILING 		1
#define DA9052_TSI_PRINT_DEBOUNCED_DATA		0
#define DA9052_TSI_PRINT_PREVIOUS_DATA		0


#if ENABLE_AVERAGE_FILTER
#define TSI_AVERAGE_FILTER_SIZE		DEFAULT_AVERAGE_FILTER_SIZE
#else
#define TSI_AVERAGE_FILTER_SIZE			1
#endif

#define TSI_FAST_MODE_SAMPLE_CNT		1000
#define TSI_ECO_MODE_SAMPLE_CNT			100

#define TSI_POLL_SAMPLE_CNT			10

#define TSI_FAST_MODE_REG_DATA_PROCESSING_INTERVAL	\
	((1000 / TSI_FAST_MODE_SAMPLE_CNT)* TSI_POLL_SAMPLE_CNT)
#define TSI_ECO_MODE_REG_DATA_PROCESSING_INTERVAL	\
	((1000 / TSI_ECO_MODE_SAMPLE_CNT)* TSI_POLL_SAMPLE_CNT)

#if DEFAULT_TSI_SAMPLING_MODE
#define DEFAULT_REG_DATA_PROCESSING_INTERVAL \
	TSI_FAST_MODE_REG_DATA_PROCESSING_INTERVAL
#else
#define DEFAULT_REG_DATA_PROCESSING_INTERVAL \
	TSI_ECO_MODE_REG_DATA_PROCESSING_INTERVAL
#endif

#define TSI_REG_DATA_BUF_SIZE 	(2 * TSI_POLL_SAMPLE_CNT)

#define TSI_FAST_MODE_RAW_DATA_PROCESSING_INTERVAL	\
	((1000 / TSI_FAST_MODE_SAMPLE_CNT) * (TSI_AVERAGE_FILTER_SIZE))
#define TSI_ECO_MODE_RAW_DATA_PROCESSING_INTERVAL	\
	((1000 / TSI_ECO_MODE_SAMPLE_CNT) * (TSI_AVERAGE_FILTER_SIZE))


#if DEFAULT_TSI_SAMPLING_MODE
#define DEFAULT_RAW_DATA_PROCESSING_INTERVAL \
	TSI_FAST_MODE_RAW_DATA_PROCESSING_INTERVAL
#else
#define DEFAULT_RAW_DATA_PROCESSING_INTERVAL \
	TSI_ECO_MODE_RAW_DATA_PROCESSING_INTERVAL
#endif


#define TSI_RAW_DATA_BUF_SIZE  \
	(TSI_REG_DATA_BUF_SIZE * \
	( (TSI_AVERAGE_FILTER_SIZE / TSI_POLL_SAMPLE_CNT) + 1))

#endif /* __LINUX_MFD_DA9052_TSI_CFG_H */
