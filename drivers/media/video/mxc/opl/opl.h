/*
 * Copyright 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @defgroup OPLIP OPL Image Processing
 */
/*!
 * @file opl.h
 *
 * @brief The OPL (Open Primitives Library) Image Processing library defines
 * efficient functions for rotation and mirroring.
 *
 * It includes ARM9-optimized rotation and mirroring functions. It is derived
 * from the original OPL project which is found at sourceforge.freescale.net.
 *
 * @ingroup OPLIP
 */
#ifndef __OPL_H__
#define __OPL_H__

#include <linux/types.h>

#define BYTES_PER_PIXEL			2
#define CACHE_LINE_WORDS		8
#define BYTES_PER_WORD			4

#define BYTES_PER_2PIXEL		(BYTES_PER_PIXEL * 2)
#define BYTES_PER_4PIXEL		(BYTES_PER_PIXEL * 4)
#define BYTES_PER_8PIXEL		(BYTES_PER_PIXEL * 8)

#define QCIF_Y_WIDTH			176
#define QCIF_Y_HEIGHT			144

/*! Enumerations of opl error code */
enum opl_error {
	OPLERR_SUCCESS = 0,
	OPLERR_NULL_PTR,
	OPLERR_BAD_ARG,
	OPLERR_DIV_BY_ZERO,
	OPLERR_OVER_FLOW,
	OPLERR_UNDER_FLOW,
	OPLERR_MISALIGNED,
};

/*!
 * @brief Rotate a 16bbp buffer 90 degrees clockwise.
 *
 * @param src             Pointer to the input buffer
 * @param src_line_stride Length in bytes of a raster line of the input buffer
 * @param width           Width in pixels of the region in the input buffer
 * @param height          Height in pixels of the region in the input buffer
 * @param dst             Pointer to the output buffer
 * @param dst_line_stride Length in bytes of a raster line of the output buffer
 *
 * @return Standard OPL error code. See enumeration for possible result codes.
 */
int opl_rotate90_u16(const u8 *src, int src_line_stride, int width, int height,
		     u8 *dst, int dst_line_stride);

/*!
 * @brief Rotate a 16bbp buffer 180 degrees clockwise.
 *
 * @param src             Pointer to the input buffer
 * @param src_line_stride Length in bytes of a raster line of the input buffer
 * @param width           Width in pixels of the region in the input buffer
 * @param height          Height in pixels of the region in the input buffer
 * @param dst             Pointer to the output buffer
 * @param dst_line_stride Length in bytes of a raster line of the output buffer
 *
 * @return Standard OPL error code. See enumeration for possible result codes.
 */
int opl_rotate180_u16(const u8 *src, int src_line_stride, int width,
		      int height, u8 *dst, int dst_line_stride);

/*!
 * @brief Rotate a 16bbp buffer 270 degrees clockwise
 *
 * @param src             Pointer to the input buffer
 * @param src_line_stride Length in bytes of a raster line of the input buffer
 * @param width           Width in pixels of the region in the input buffer
 * @param height          Height in pixels of the region in the input buffer
 * @param dst             Pointer to the output buffer
 * @param dst_line_stride Length in bytes of a raster line of the output buffer
 *
 * @return Standard OPL error code. See enumeration for possible result codes.
 */
int opl_rotate270_u16(const u8 *src, int src_line_stride, int width,
		      int height, u8 *dst, int dst_line_stride);

/*!
 * @brief Mirror a 16bpp buffer horizontally
 *
 * @param src             Pointer to the input buffer
 * @param src_line_stride Length in bytes of a raster line of the input buffer
 * @param width           Width in pixels of the region in the input buffer
 * @param height          Height in pixels of the region in the input buffer
 * @param dst             Pointer to the output buffer
 * @param dst_line_stride Length in bytes of a raster line of the output buffer
 *
 * @return Standard OPL error code. See enumeration for possible result codes.
 */
int opl_hmirror_u16(const u8 *src, int src_line_stride, int width, int height,
		    u8 *dst, int dst_line_stride);

/*!
 * @brief Mirror a 16bpp buffer vertically
 *
 * @param src             Pointer to the input buffer
 * @param src_line_stride Length in bytes of a raster line of the input buffer
 * @param width           Width in pixels of the region in the input buffer
 * @param height          Height in pixels of the region in the input buffer
 * @param dst             Pointer to the output buffer
 * @param dst_line_stride Length in bytes of a raster line of the output buffer
 *
 * @return Standard OPL error code. See enumeration for possible result codes.
 */
int opl_vmirror_u16(const u8 *src, int src_line_stride, int width, int height,
		    u8 *dst, int dst_line_stride);

/*!
 * @brief Rotate a 16bbp buffer 90 degrees clockwise and mirror vertically
 *	  It is equivalent to rotate 270 degree and mirror horizontally
 *
 * @param src             Pointer to the input buffer
 * @param src_line_stride Length in bytes of a raster line of the input buffer
 * @param width           Width in pixels of the region in the input buffer
 * @param height          Height in pixels of the region in the input buffer
 * @param dst             Pointer to the output buffer
 * @param dst_line_stride Length in bytes of a raster line of the output buffer
 *
 * @return Standard OPL error code. See enumeration for possible result codes.
 */
int opl_rotate90_vmirror_u16(const u8 *src, int src_line_stride, int width,
			     int height, u8 *dst, int dst_line_stride);

/*!
 * @brief Rotate a 16bbp buffer 270 degrees clockwise and mirror vertically
 *	  It is equivalent to rotate 90 degree and mirror horizontally
 *
 * @param src             Pointer to the input buffer
 * @param src_line_stride Length in bytes of a raster line of the input buffer
 * @param width           Width in pixels of the region in the input buffer
 * @param height          Height in pixels of the region in the input buffer
 * @param dst             Pointer to the output buffer
 * @param dst_line_stride Length in bytes of a raster line of the output buffer
 *
 * @return Standard OPL error code. See enumeration for possible result codes.
 */
int opl_rotate270_vmirror_u16(const u8 *src, int src_line_stride, int width,
			      int height, u8 *dst, int dst_line_stride);

#endif				/* __OPL_H__ */
