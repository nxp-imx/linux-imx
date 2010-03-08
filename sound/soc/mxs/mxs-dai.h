/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _MXS_SAIF_H
#define _MXS_SAIF_H

#include <mach/hardware.h>

/* SSI clock sources */
#define IMX_SSP_SYS_CLK			0


/* SSI Div 2 */
#define IMX_SSI_DIV_2_OFF		(~SSI_STCCR_DIV2)
#define IMX_SSI_DIV_2_ON		SSI_STCCR_DIV2

#define IMX_DAI_AC97_1 0
#define IMX_DAI_AC97_2 1

extern struct snd_soc_dai mxs_saif_dai[];

#endif
