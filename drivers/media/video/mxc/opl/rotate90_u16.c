/*
 * Copyright 2004-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include "opl.h"

static int opl_rotate90_u16_by16(const u8 *src, int src_line_stride, int width,
				 int height, u8 *dst, int dst_line_stride,
				 int vmirror);
static int opl_rotate90_u16_by4(const u8 *src, int src_line_stride, int width,
				int height, u8 *dst, int dst_line_stride,
				int vmirror);
static int opl_rotate90_vmirror_u16_both(const u8 *src, int src_line_stride,
					 int width, int height, u8 *dst,
					 int dst_line_stride, int vmirror);
int opl_rotate90_u16_qcif(const u8 *src, u8 *dst);

int opl_rotate90_u16(const u8 *src, int src_line_stride, int width, int height,
		     u8 *dst, int dst_line_stride)
{
	return opl_rotate90_vmirror_u16_both(src, src_line_stride, width,
					    height, dst, dst_line_stride, 0);
}

int opl_rotate90_vmirror_u16(const u8 *src, int src_line_stride, int width,
			    int height, u8 *dst, int dst_line_stride)
{
	return opl_rotate90_vmirror_u16_both(src, src_line_stride, width,
					    height, dst, dst_line_stride, 1);
}

static int opl_rotate90_vmirror_u16_both(const u8 *src, int src_line_stride,
					 int width, int height, u8 *dst,
					 int dst_line_stride, int vmirror)
{
	const int BLOCK_SIZE_PIXELS = CACHE_LINE_WORDS * BYTES_PER_WORD
	    / BYTES_PER_PIXEL;
	const int BLOCK_SIZE_PIXELS_BY4 = CACHE_LINE_WORDS * BYTES_PER_WORD
	    / BYTES_PER_PIXEL / 4;

	if (!src || !dst)
		return OPLERR_NULL_PTR;

	if (width == 0 || height == 0 || src_line_stride == 0
	    || dst_line_stride == 0)
		return OPLERR_BAD_ARG;

	/* The QCIF algorithm doesn't support vertical mirroring */
	if (vmirror == 0 && width == QCIF_Y_WIDTH && height == QCIF_Y_HEIGHT
	    && src_line_stride == QCIF_Y_WIDTH * 2
	    && src_line_stride == QCIF_Y_HEIGHT * 2)
		return opl_rotate90_u16_qcif(src, dst);
	else if (width % BLOCK_SIZE_PIXELS == 0
		 && height % BLOCK_SIZE_PIXELS == 0)
		return opl_rotate90_u16_by16(src, src_line_stride, width,
					     height, dst, dst_line_stride,
					     vmirror);
	else if (width % BLOCK_SIZE_PIXELS_BY4 == 0
		 && height % BLOCK_SIZE_PIXELS_BY4 == 0)
		return opl_rotate90_u16_by4(src, src_line_stride, width, height,
					    dst, dst_line_stride, vmirror);
	else
		return OPLERR_BAD_ARG;
}

/*
 * Performs clockwise rotation (and possibly vertical mirroring depending
 * on the vmirror flag) using block sizes of 16x16
 * The algorithm is similar to 270 degree clockwise rotation algorithm
 */
static int opl_rotate90_u16_by16(const u8 *src, int src_line_stride, int width,
				 int height, u8 *dst, int dst_line_stride,
				 int vmirror)
{
	const int BLOCK_SIZE_PIXELS = CACHE_LINE_WORDS * BYTES_PER_WORD
	    / BYTES_PER_PIXEL;
	const int BLOCK_SIZE_BYTES = BYTES_PER_PIXEL * BLOCK_SIZE_PIXELS;
	const int IN_INDEX = src_line_stride * BLOCK_SIZE_PIXELS
	    + BYTES_PER_PIXEL;
	const int OUT_INDEX = vmirror ?
	    -dst_line_stride - BLOCK_SIZE_BYTES
	    : dst_line_stride - BLOCK_SIZE_BYTES;
	const u8 *in_block_ptr;
	u8 *out_block_ptr;
	int i, k;

	for (k = height / BLOCK_SIZE_PIXELS; k > 0; k--) {
		in_block_ptr = src + src_line_stride * (height - 1)
		    - (src_line_stride * BLOCK_SIZE_PIXELS *
		       (height / BLOCK_SIZE_PIXELS - k));
		out_block_ptr = dst + BYTES_PER_PIXEL * BLOCK_SIZE_PIXELS *
		    ((height / BLOCK_SIZE_PIXELS) - k);

		/*
		 * For vertical mirroring the writing starts from the
		 * bottom line
		 */
		if (vmirror)
			out_block_ptr += dst_line_stride * (width - 1);

		for (i = width; i > 0; i--) {
			__asm__ volatile (
				"ldrh	r2, [%0], -%4\n\t"
				"ldrh	r3, [%0], -%4\n\t"
				"ldrh	r4, [%0], -%4\n\t"
				"ldrh	r5, [%0], -%4\n\t"
				"orr	r2, r2, r3, lsl #16\n\t"
				"orr	r4, r4, r5, lsl #16\n\t"
				"str	r2, [%1], #4\n\t"
				"str	r4, [%1], #4\n\t"

				"ldrh	r2, [%0], -%4\n\t"
				"ldrh	r3, [%0], -%4\n\t"
				"ldrh	r4, [%0], -%4\n\t"
				"ldrh	r5, [%0], -%4\n\t"
				"orr	r2, r2, r3, lsl #16\n\t"
				"orr	r4, r4, r5, lsl #16\n\t"
				"str	r2, [%1], #4\n\t"
				"str	r4, [%1], #4\n\t"

				"ldrh	r2, [%0], -%4\n\t"
				"ldrh	r3, [%0], -%4\n\t"
				"ldrh	r4, [%0], -%4\n\t"
				"ldrh	r5, [%0], -%4\n\t"
				"orr	r2, r2, r3, lsl #16\n\t"
				"orr	r4, r4, r5, lsl #16\n\t"
				"str	r2, [%1], #4\n\t"
				"str	r4, [%1], #4\n\t"

				"ldrh	r2, [%0], -%4\n\t"
				"ldrh	r3, [%0], -%4\n\t"
				"ldrh	r4, [%0], -%4\n\t"
				"ldrh	r5, [%0], -%4\n\t"
				"orr	r2, r2, r3, lsl #16\n\t"
				"orr	r4, r4, r5, lsl #16\n\t"
				"str	r2, [%1], #4\n\t"
				"str	r4, [%1], #4\n\t"

				: "+r" (in_block_ptr), "+r"(out_block_ptr)	/* output */
				: "0"(in_block_ptr), "1"(out_block_ptr), "r"(src_line_stride)	/* input  */
				: "r2", "r3", "r4", "r5", "memory"	/* modify */
			);
			in_block_ptr += IN_INDEX;
			out_block_ptr += OUT_INDEX;
		}
	}

	return OPLERR_SUCCESS;
}

/*
 * Performs clockwise rotation (and possibly vertical mirroring depending
 * on the vmirror flag) using block sizes of 4x4
 * The algorithm is similar to 270 degree clockwise rotation algorithm
 */
static int opl_rotate90_u16_by4(const u8 *src, int src_line_stride, int width,
				int height, u8 *dst, int dst_line_stride,
				int vmirror)
{
	const int BLOCK_SIZE_PIXELS = CACHE_LINE_WORDS * BYTES_PER_WORD
	    / BYTES_PER_PIXEL / 4;
	const int BLOCK_SIZE_BYTES = BYTES_PER_PIXEL * BLOCK_SIZE_PIXELS;
	const int IN_INDEX = src_line_stride * BLOCK_SIZE_PIXELS
	    + BYTES_PER_PIXEL;
	const int OUT_INDEX = vmirror ?
	    -dst_line_stride - BLOCK_SIZE_BYTES
	    : dst_line_stride - BLOCK_SIZE_BYTES;
	const u8 *in_block_ptr;
	u8 *out_block_ptr;
	int i, k;

	for (k = height / BLOCK_SIZE_PIXELS; k > 0; k--) {
		in_block_ptr = src + src_line_stride * (height - 1)
		    - (src_line_stride * BLOCK_SIZE_PIXELS *
		       (height / BLOCK_SIZE_PIXELS - k));
		out_block_ptr = dst + BYTES_PER_PIXEL * BLOCK_SIZE_PIXELS
		    * ((height / BLOCK_SIZE_PIXELS) - k);

		/*
		 * For horizontal mirroring the writing starts from the
		 * bottom line
		 */
		if (vmirror)
			out_block_ptr += dst_line_stride * (width - 1);

		for (i = width; i > 0; i--) {
			__asm__ volatile (
				"ldrh	r2, [%0], -%4\n\t"
				"ldrh	r3, [%0], -%4\n\t"
				"ldrh	r4, [%0], -%4\n\t"
				"ldrh	r5, [%0], -%4\n\t"
				"orr	r2, r2, r3, lsl #16\n\t"
				"orr	r4, r4, r5, lsl #16\n\t"
				"str	r2, [%1], #4\n\t"
				"str	r4, [%1], #4\n\t"

				: "+r" (in_block_ptr), "+r"(out_block_ptr)	/* output */
				: "0"(in_block_ptr), "1"(out_block_ptr), "r"(src_line_stride)	/* input  */
				: "r2", "r3", "r4", "r5", "memory"	/* modify */
			);
			in_block_ptr += IN_INDEX;
			out_block_ptr += OUT_INDEX;
		}
	}

	return OPLERR_SUCCESS;
}

EXPORT_SYMBOL(opl_rotate90_u16);
EXPORT_SYMBOL(opl_rotate90_vmirror_u16);
