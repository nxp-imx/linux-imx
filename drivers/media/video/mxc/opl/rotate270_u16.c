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

static int opl_rotate270_u16_by16(const u8 *src, int src_line_stride,
				  int width, int height, u8 *dst,
				  int dst_line_stride, int vmirror);
static int opl_rotate270_u16_by4(const u8 *src, int src_line_stride, int width,
				 int height, u8 *dst, int dst_line_stride,
				 int vmirror);
static int opl_rotate270_vmirror_u16_both(const u8 *src, int src_line_stride,
					  int width, int height, u8 *dst,
					  int dst_line_stride, int vmirror);
int opl_rotate270_u16_qcif(const u8 *src, u8 *dst);

int opl_rotate270_u16(const u8 *src, int src_line_stride, int width,
		      int height, u8 *dst, int dst_line_stride)
{
	return opl_rotate270_vmirror_u16_both(src, src_line_stride, width,
					      height, dst, dst_line_stride, 0);
}

int opl_rotate270_vmirror_u16(const u8 *src, int src_line_stride, int width,
			      int height, u8 *dst, int dst_line_stride)
{
	return opl_rotate270_vmirror_u16_both(src, src_line_stride, width,
					      height, dst, dst_line_stride, 1);
}

static int opl_rotate270_vmirror_u16_both(const u8 *src, int src_line_stride,
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
		return opl_rotate270_u16_qcif(src, dst);
	else if (width % BLOCK_SIZE_PIXELS == 0
		 && height % BLOCK_SIZE_PIXELS == 0)
		return opl_rotate270_u16_by16(src, src_line_stride, width,
					      height, dst, dst_line_stride,
					      vmirror);
	else if (width % BLOCK_SIZE_PIXELS_BY4 == 0
		 && height % BLOCK_SIZE_PIXELS_BY4 == 0)
		return opl_rotate270_u16_by4(src, src_line_stride, width,
					     height, dst, dst_line_stride,
					     vmirror);
	else
		return OPLERR_BAD_ARG;
}

/*
 * Rotate Counter Clockwise, divide RGB component into 16 row strips, read
 * non sequentially and write sequentially. This is done in 16 line strips
 * so that the cache is used better. Cachelines are 8 words = 32 bytes. Pixels
 * are 2 bytes. The 16 reads will be cache misses, but the next 240 should
 * be from cache. The writes to the output buffer will be sequential for 16
 * writes.
 *
 * Example:
 * Input data matrix:	    output matrix
 *
 * 0 | 1 | 2 | 3 | 4 |      4 | 0 | 0 | 3 |
 * 4 | 3 | 2 | 1 | 0 |      3 | 1 | 9 | 6 |
 * 6 | 7 | 8 | 9 | 0 |      2 | 2 | 8 | 2 |
 * 5 | 3 | 2 | 6 | 3 |      1 | 3 | 7 | 3 |
 * ^			    0 | 4 | 6 | 5 | < Write the input data sequentially
 * Read first column
 * Start at the bottom
 * Move to next column and repeat
 *
 * Loop over k decreasing (blocks)
 * in_block_ptr = src + (((RGB_HEIGHT_PIXELS / BLOCK_SIZE_PIXELS) - k)
 *                * BLOCK_SIZE_PIXELS) * (RGB_WIDTH_BYTES)
 * out_block_ptr = dst + (((RGB_HEIGHT_PIXELS / BLOCK_SIZE_PIXELS) - k)
 *                * BLOCK_SIZE_BYTES) + (RGB_WIDTH_PIXELS - 1)
 *                * RGB_HEIGHT_PIXELS * BYTES_PER_PIXEL
 *
 * Loop over i decreasing (width)
 * Each pix:
 * in_block_ptr += RGB_WIDTH_BYTES
 * out_block_ptr += 4
 *
 * Each row of block:
 * in_block_ptr -= RGB_WIDTH_BYTES * BLOCK_SIZE_PIXELS - 2
 * out_block_ptr -= RGB_HEIGHT_PIXELS * BYTES_PER_PIXEL + 2 * BLOCK_SIZE_PIXELS;
 *
 * It may perform vertical mirroring too depending on the vmirror flag.
 */
static int opl_rotate270_u16_by16(const u8 *src, int src_line_stride,
				  int width, int height, u8 *dst,
				  int dst_line_stride, int vmirror)
{
	const int BLOCK_SIZE_PIXELS = CACHE_LINE_WORDS * BYTES_PER_WORD
	    / BYTES_PER_PIXEL;
	const int IN_INDEX = src_line_stride * BLOCK_SIZE_PIXELS
	    - BYTES_PER_PIXEL;
	const int OUT_INDEX = vmirror ?
	    -dst_line_stride + BYTES_PER_PIXEL * BLOCK_SIZE_PIXELS
	    : dst_line_stride + BYTES_PER_PIXEL * BLOCK_SIZE_PIXELS;
	const u8 *in_block_ptr;
	u8 *out_block_ptr;
	int i, k;

	for (k = height / BLOCK_SIZE_PIXELS; k > 0; k--) {
		in_block_ptr = src + (((height / BLOCK_SIZE_PIXELS) - k)
				      * BLOCK_SIZE_PIXELS) * src_line_stride;
		out_block_ptr = dst + (((height / BLOCK_SIZE_PIXELS) - k)
				       * BLOCK_SIZE_PIXELS * BYTES_PER_PIXEL) +
		    (width - 1) * dst_line_stride;

		/*
		* For vertical mirroring the writing starts from the
		* first line
		*/
		if (vmirror)
			out_block_ptr -= dst_line_stride * (width - 1);

		for (i = width; i > 0; i--) {
			__asm__ volatile (
				"ldrh	r2, [%0], %4\n\t"
				"ldrh	r3, [%0], %4\n\t"
				"ldrh	r4, [%0], %4\n\t"
				"ldrh	r5, [%0], %4\n\t"
				"orr	r2, r2, r3, lsl #16\n\t"
				"orr	r4, r4, r5, lsl #16\n\t"
				"str	r2, [%1], #4\n\t"
				"str	r4, [%1], #4\n\t"

				"ldrh	r2, [%0], %4\n\t"
				"ldrh	r3, [%0], %4\n\t"
				"ldrh	r4, [%0], %4\n\t"
				"ldrh	r5, [%0], %4\n\t"
				"orr	r2, r2, r3, lsl #16\n\t"
				"orr	r4, r4, r5, lsl #16\n\t"
				"str	r2, [%1], #4\n\t"
				"str	r4, [%1], #4\n\t"

				"ldrh	r2, [%0], %4\n\t"
				"ldrh	r3, [%0], %4\n\t"
				"ldrh	r4, [%0], %4\n\t"
				"ldrh	r5, [%0], %4\n\t"
				"orr	r2, r2, r3, lsl #16\n\t"
				"orr	r4, r4, r5, lsl #16\n\t"
				"str	r2, [%1], #4\n\t"
				"str	r4, [%1], #4\n\t"

				"ldrh	r2, [%0], %4\n\t"
				"ldrh	r3, [%0], %4\n\t"
				"ldrh	r4, [%0], %4\n\t"
				"ldrh	r5, [%0], %4\n\t"
				"orr	r2, r2, r3, lsl #16\n\t"
				"orr	r4, r4, r5, lsl #16\n\t"
				"str	r2, [%1], #4\n\t"
				"str	r4, [%1], #4\n\t"

				: "+r" (in_block_ptr), "+r"(out_block_ptr)	/* output */
				: "0"(in_block_ptr), "1"(out_block_ptr), "r"(src_line_stride)	/* input  */
				: "r2", "r3", "r4", "r5", "memory"	/* modify */
			);
			in_block_ptr -= IN_INDEX;
			out_block_ptr -= OUT_INDEX;
		}
	}

	return OPLERR_SUCCESS;
}

/*
 * Rotate Counter Clockwise, divide RGB component into 4 row strips, read
 * non sequentially and write sequentially. This is done in 4 line strips
 * so that the cache is used better. Cachelines are 8 words = 32 bytes. Pixels
 * are 2 bytes. The 4 reads will be cache misses, but the next 60 should
 * be from cache. The writes to the output buffer will be sequential for 4
 * writes.
 *
 * Example:
 * Input data matrix:	    output matrix
 *
 * 0 | 1 | 2 | 3 | 4 |      4 | 0 | 0 | 3 |
 * 4 | 3 | 2 | 1 | 0 |      3 | 1 | 9 | 6 |
 * 6 | 7 | 8 | 9 | 0 |      2 | 2 | 8 | 2 |
 * 5 | 3 | 2 | 6 | 3 |      1 | 3 | 7 | 3 |
 * ^			    0 | 4 | 6 | 5 | < Write the input data sequentially
 * Read first column
 * Start at the bottom
 * Move to next column and repeat
 *
 * Loop over k decreasing (blocks)
 * in_block_ptr = src + (((RGB_HEIGHT_PIXELS / BLOCK_SIZE_PIXELS) - k)
 *                * BLOCK_SIZE_PIXELS) * (RGB_WIDTH_BYTES)
 * out_block_ptr = dst + (((RGB_HEIGHT_PIXELS / BLOCK_SIZE_PIXELS) - k)
 *                * BLOCK_SIZE_BYTES) + (RGB_WIDTH_PIXELS - 1)
 *                * RGB_HEIGHT_PIXELS * BYTES_PER_PIXEL
 *
 * Loop over i decreasing (width)
 * Each pix:
 * in_block_ptr += RGB_WIDTH_BYTES
 * out_block_ptr += 4
 *
 * Each row of block:
 * in_block_ptr -= RGB_WIDTH_BYTES * BLOCK_SIZE_PIXELS - 2
 * out_block_ptr -= RGB_HEIGHT_PIXELS * BYTES_PER_PIXEL + 2 * BLOCK_SIZE_PIXELS;
 *
 * It may perform vertical mirroring too depending on the vmirror flag.
 */
static int opl_rotate270_u16_by4(const u8 *src, int src_line_stride, int width,
				 int height, u8 *dst, int dst_line_stride,
				 int vmirror)
{
	const int BLOCK_SIZE_PIXELS = CACHE_LINE_WORDS * BYTES_PER_WORD
	    / BYTES_PER_PIXEL / 4;
	const int IN_INDEX = src_line_stride * BLOCK_SIZE_PIXELS
	    - BYTES_PER_PIXEL;
	const int OUT_INDEX = vmirror ?
	    -dst_line_stride + BYTES_PER_PIXEL * BLOCK_SIZE_PIXELS
	    : dst_line_stride + BYTES_PER_PIXEL * BLOCK_SIZE_PIXELS;
	const u8 *in_block_ptr;
	u8 *out_block_ptr;
	int i, k;

	for (k = height / BLOCK_SIZE_PIXELS; k > 0; k--) {
		in_block_ptr = src + (((height / BLOCK_SIZE_PIXELS) - k)
				      * BLOCK_SIZE_PIXELS) * src_line_stride;
		out_block_ptr = dst + (((height / BLOCK_SIZE_PIXELS) - k)
				       * BLOCK_SIZE_PIXELS * BYTES_PER_PIXEL)
		    + (width - 1) * dst_line_stride;

		/*
		* For vertical mirroring the writing starts from the
		* first line
		*/
		if (vmirror)
			out_block_ptr -= dst_line_stride * (width - 1);

		for (i = width; i > 0; i--) {
			__asm__ volatile (
				"ldrh	r2, [%0], %4\n\t"
				"ldrh	r3, [%0], %4\n\t"
				"ldrh	r4, [%0], %4\n\t"
				"ldrh	r5, [%0], %4\n\t"
				"orr	r2, r2, r3, lsl #16\n\t"
				"orr	r4, r4, r5, lsl #16\n\t"
				"str	r2, [%1], #4\n\t"
				"str	r4, [%1], #4\n\t"

				: "+r" (in_block_ptr), "+r"(out_block_ptr)	/* output */
				: "0"(in_block_ptr), "1"(out_block_ptr), "r"(src_line_stride)	/* input  */
				: "r2", "r3", "r4", "r5", "memory"	/* modify */
			);
			in_block_ptr -= IN_INDEX;
			out_block_ptr -= OUT_INDEX;
		}
	}

	return OPLERR_SUCCESS;
}

EXPORT_SYMBOL(opl_rotate270_u16);
EXPORT_SYMBOL(opl_rotate270_vmirror_u16);
