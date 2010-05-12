/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*!
 * @file include/linux/ldb.h
 *
 * @brief This file contains the LDB driver API declarations.
 *
 * @ingroup LDB
 */

#ifndef __MXC_LDB_H__
#define __MXC_LDB_H__

#include <linux/types.h>

typedef enum {
	LDB_INT_REF = 0,
	LDB_EXT_REF = 1,
} ldb_bgref_t;

typedef enum {
	LDB_VS_ACT_H = 0,
	LDB_VS_ACT_L = 1,
} ldb_vsync_t;

typedef enum {
	LDB_BIT_MAP_SPWG = 0,
	LDB_BIT_MAP_JEIDA = 1,
} ldb_bitmap_t;

typedef enum {
	LDB_CHAN_MODE_SIN = 0,
	LDB_CHAN_MODE_SEP = 1,
	LDB_CHAN_MODE_DUL = 2,
	LDB_CHAN_MODE_SPL = 3,
} ldb_channel_mode_t;

typedef struct _ldb_bgref_parm {
	ldb_bgref_t		bgref_mode;
} ldb_bgref_parm;

typedef struct _ldb_vsync_parm {
	int			di;
	ldb_vsync_t		vsync_mode;
} ldb_vsync_parm;

typedef struct _ldb_bitmap_parm {
	int			channel;
	ldb_bitmap_t		bitmap_mode;
} ldb_bitmap_parm;

typedef struct _ldb_data_width_parm {
	int			channel;
	int			data_width;
} ldb_data_width_parm;

typedef struct _ldb_chan_mode_parm {
	int				di;
	ldb_channel_mode_t		channel_mode;
} ldb_chan_mode_parm;

/* IOCTL commands */
#define LDB_BGREF_RMODE			_IOW('L', 0x1, ldb_bgref_parm)
#define LDB_VSYNC_POL			_IOW('L', 0x2, ldb_vsync_parm)
#define LDB_BIT_MAP			_IOW('L', 0x3, ldb_bitmap_parm)
#define LDB_DATA_WIDTH			_IOW('L', 0x4, ldb_data_width_parm)
#define LDB_CHAN_MODE			_IOW('L', 0x5, ldb_chan_mode_parm)
#define LDB_ENABLE			_IOW('L', 0x6, int)
#define LDB_DISABLE			_IOW('L', 0x7, int)
#endif
