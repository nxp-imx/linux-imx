/*
 * da9052 TSI calibration module declarations.
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

#ifndef __LINUX_MFD_DA9052_TSI_CALIBRATE_H
#define __LINUX_MFD_DA9052_TSI_CALIBRATE_H

#include <linux/mfd/da9052/tsi_filter.h>

struct Calib_xform_matrix_t {
	s32	An;
	s32	Bn;
	s32	Cn;
	s32	Dn;
	s32	En;
	s32	Fn;
	s32	Divider;
} ;


struct calib_cfg_t {
	u8 calibrate_flag;
} ;

ssize_t da9052_tsi_set_calib_matrix(struct da9052_tsi_data *displayPtr,
				   struct da9052_tsi_data *screenPtr);
u8 configure_tsi_calib(struct calib_cfg_t *tsi_calib);
struct calib_cfg_t *get_calib_config(void);
#endif /* __LINUX_MFD_DA9052_TSI_CALIBRATE_H */

