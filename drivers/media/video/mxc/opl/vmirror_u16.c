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

#include <linux/module.h>
#include <linux/string.h>
#include "opl.h"

int opl_vmirror_u16(const u8 *src, int src_line_stride, int width, int height,
		    u8 *dst, int dst_line_stride)
{
	const u8 *src_row_addr;
	u8 *dst_row_addr;
	int i;

	if (!src || !dst)
		return OPLERR_NULL_PTR;

	if (width == 0 || height == 0 || src_line_stride == 0
	    || dst_line_stride == 0)
		return OPLERR_BAD_ARG;

	src_row_addr = src;
	dst_row_addr = dst + (height - 1) * dst_line_stride;

	/* Loop over all rows */
	for (i = 0; i < height; i++) {
		/* memcpy each row */
		memcpy(dst_row_addr, src_row_addr, BYTES_PER_PIXEL * width);
		src_row_addr += src_line_stride;
		dst_row_addr -= dst_line_stride;
	}

	return OPLERR_SUCCESS;
}

EXPORT_SYMBOL(opl_vmirror_u16);
