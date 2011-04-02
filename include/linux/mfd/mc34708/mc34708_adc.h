/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#ifndef __ASM_ARCH_MXC_MC34708_PMIC_ADC_H__
#define __ASM_ARCH_MXC_MC34708_PMIC_ADC_H__

/*!
 * @defgroup PMIC_ADC PMIC Digitizer Driver
 * @ingroup PMIC_DRVRS
 */

#include <linux/pmic_status.h>
#include <linux/pmic_external.h>

/*!
 * This enumeration defines input channels for PMIC ADC
 */

typedef enum {
	BATTERY_VOLTAGE,
	BATTERY_CURRENT,
	APPLICATION_SUPPLY_VOLTAGE,
	DIE_TEMP,
	AUX_CHARGER_VOLTAGE,
	USB_VOLTAGE,
	RESERVER0,
	BATTERY_THEMISTOR,
	COINCELL_VOLTAGE,
	GEN_PURPOSE_AD9,
	GEN_PURPOSE_AD10,
	GEN_PURPOSE_AD11,
	GEN_PURPOSE_AD12,
	GEN_PURPOSE_AD13,
	GEN_PURPOSE_AD14,
	GEN_PURPOSE_AD15,
} t_channel;


/* EXPORTED FUNCTIONS */

#ifdef __KERNEL__
/*!
 * This function initializes all ADC registers with default values. This
 * function also registers the interrupt events.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS mc34708_pmic_adc_init(void);

/*!
 * This function disables the ADC, de-registers the interrupt events.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS mc34708_pmic_adc_deinit(void);

/*!
 * This function triggers a conversion and returns one sampling result of one
 * channel.
 *
 * @param        channel   The channel to be sampled
 * @param        result    The pointer to the conversion result. The memory
 *                         should be allocated by the caller of this function.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */

PMIC_STATUS mc34708_pmic_adc_convert(t_channel channel, unsigned short *result);



#endif				/* _KERNEL */
#endif				/* __ASM_ARCH_MXC_PMIC_ADC_H__ */
