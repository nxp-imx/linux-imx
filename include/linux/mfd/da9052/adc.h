/*
 * da9052 ADC module declarations.
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

#ifndef __LINUX_MFD_DA9052_ADC_H
#define __LINUX_MFD_DA9052_ADC_H

#include "gpio.h"

#define DA9052_ADC_DEVICE_NAME				"da9052_adc"

/* Channel Definations */
#define DA9052_ADC_VDDOUT				0
#define DA9052_ADC_ICH					1
#define DA9052_ADC_TBAT					2
#define DA9052_ADC_VBAT					3
#define DA9052_ADC_ADCIN4				4
#define DA9052_ADC_ADCIN5				5
#define DA9052_ADC_ADCIN6				6
#define DA9052_ADC_TSI					7
#define DA9052_ADC_TJUNC				8
#define DA9052_ADC_VBBAT				9

#if (DA9052_GPIO_PIN_0 == DA9052_GPIO_CONFIG_ADC)
#define DA9052_ADC_CONF_ADC4				1
#else
#define DA9052_ADC_CONF_ADC4				0
#endif
#if (DA9052_GPIO_PIN_1 == DA9052_GPIO_CONFIG_ADC)
#define DA9052_ADC_CONF_ADC5				1
#else
#define DA9052_ADC_CONF_ADC5				0
#endif
#if (DA9052_GPIO_PIN_2 == DA9052_GPIO_CONFIG_ADC)
#define DA9052_ADC_CONF_ADC6				1
#else
#define DA9052_ADC_CONF_ADC6				0
#endif

/* Maximum retry count to check manual conversion over */
#define DA9052_ADC_MAX_MANCONV_RETRY_COUNT		8

struct da9052_adc_priv {
	struct da9052 *da9052;
	struct device *hwmon_dev;
	struct mutex manconv_lock;
};

#endif /* __LINUX_MFD_DA9052_ADC_H */
