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

#ifndef _EMI_SETTINGS_H_
#define _EMI_SETTINGS_H_

#define MX28_DRAMCTRLREGNUM 190

#define SCALING_DATA_EMI_DIV_OFFSET     0
#define SCALING_DATA_FRAC_DIV_OFFSET    4
#define SCALING_DATA_CUR_FREQ_OFFSET    8
#define SCALING_DATA_NEW_FREQ_OFFSET    12

#ifndef __ASSEMBLER__
void mxs_ram_freq_scale_end();
void DDR2EmiController_EDE1116_133MHz(void);
void DDR2EmiController_EDE1116_166MHz(void);
void DDR2EmiController_EDE1116_200MHz(void);
void mDDREmiController_24MHz(void);
void mDDREmiController_133MHz(void);
void mDDREmiController_200MHz(void);
unsigned int *get_current_emidata();
#endif

#endif
