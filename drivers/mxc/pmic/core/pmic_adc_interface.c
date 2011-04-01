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

#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/pmic_adc.h>

#include "pmic.h"

static struct pmic_adc_api *pmic_adc_apis;

PMIC_STATUS pmic_adc_convert(t_channel channel, unsigned short *result)
{
	if (pmic_adc_apis && pmic_adc_apis->pmic_adc_convert)
		return pmic_adc_apis->pmic_adc_convert(channel, result);

	return PMIC_ERROR;
}
EXPORT_SYMBOL(pmic_adc_convert);

PMIC_STATUS pmic_adc_get_touch_sample(t_touch_screen *ts_value, int wait)
{
	if (pmic_adc_apis && pmic_adc_apis->pmic_adc_get_touch_sample)
		return pmic_adc_apis->pmic_adc_get_touch_sample(ts_value, wait);

	return PMIC_ERROR;
}
EXPORT_SYMBOL(pmic_adc_get_touch_sample);

int is_pmic_adc_ready(void)
{
	if (pmic_adc_apis && pmic_adc_apis->is_pmic_adc_ready)
		return pmic_adc_apis->is_pmic_adc_ready();

	return 0;
}
EXPORT_SYMBOL(is_pmic_adc_ready);

void register_adc_apis(struct pmic_adc_api *papi)
{
	pmic_adc_apis = papi;
}
