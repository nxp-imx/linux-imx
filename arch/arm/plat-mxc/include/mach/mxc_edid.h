/*
 * Copyright 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @defgroup Framebuffer Framebuffer Driver for SDC and ADC.
 */

/*!
 * @file mxc_edid.h
 *
 * @brief MXC EDID tools
 *
 * @ingroup Framebuffer
 */

#ifndef MXC_EDID_H
#define MXC_EDID_H

int read_edid(struct i2c_adapter *adp,
	       struct fb_var_screeninfo *einfo,
	       int *dvi);

#endif
