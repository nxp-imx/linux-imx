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

static inline u32 rot_left_u16(u16 x, unsigned int n)
{
	return (x << n) | (x >> (16 - n));
}

static inline u32 rot_left_u32(u32 x, unsigned int n)
{
	return (x << n) | (x >> (32 - n));
}

static inline u32 byte_swap_u32(u32 x)
{
	u32 t1, t2, t3;

	t1 = x ^ ((x << 16) | x >> 16);
	t2 = t1 & 0xff00ffff;
	t3 = (x >> 8) | (x << 24);
	return t3 ^ (t2 >> 8);
}

static int opl_hmirror_u16_by1(const u8 *src, int src_line_stride, int width,
			       int height, u8 *dst, int dst_line_stride,
			       int vmirror);
static int opl_hmirror_u16_by2(const u8 *src, int src_line_stride, int width,
			       int height, u8 *dst, int dst_line_stride,
			       int vmirror);
static int opl_hmirror_u16_by4(const u8 *src, int src_line_stride, int width,
			       int height, u8 *dst, int dst_line_stride,
			       int vmirror);
static int opl_hmirror_u16_by8(const u8 *src, int src_line_stride, int width,
			       int height, u8 *dst, int dst_line_stride,
			       int vmirror);

int opl_hmirror_u16(const u8 *src, int src_line_stride, int width, int height,
		    u8 *dst, int dst_line_stride)
{
	if (!src || !dst)
		return OPLERR_NULL_PTR;

	if (width == 0 || height == 0 || src_line_stride == 0
	    || dst_line_stride == 0)
		return OPLERR_BAD_ARG;

	if (width % 8 == 0)
		return opl_hmirror_u16_by8(src, src_line_stride, width, height,
					   dst, dst_line_stride, 0);
	else if (width % 4 == 0)
		return opl_hmirror_u16_by4(src, src_line_stride, width, height,
					   dst, dst_line_stride, 0);
	else if (width % 2 == 0)
		return opl_hmirror_u16_by2(src, src_line_stride, width, height,
					   dst, dst_line_stride, 0);
	else			/* (width % 1) */
		return opl_hmirror_u16_by1(src, src_line_stride, width, height,
					   dst, dst_line_stride, 0);
}

int opl_rotate180_u16(const u8 *src, int src_line_stride, int width,
		      int height, u8 *dst, int dst_line_stride)
{
	if (!src || !dst)
		return OPLERR_NULL_PTR;

	if (width == 0 || height == 0 || src_line_stride == 0
	    || dst_line_stride == 0)
		return OPLERR_BAD_ARG;

	if (width % 8 == 0)
		return opl_hmirror_u16_by8(src, src_line_stride, width, height,
					   dst, dst_line_stride, 1);
	else if (width % 4 == 0)
		return opl_hmirror_u16_by4(src, src_line_stride, width, height,
					   dst, dst_line_stride, 1);
	else if (width % 2 == 0)
		return opl_hmirror_u16_by2(src, src_line_stride, width, height,
					   dst, dst_line_stride, 1);
	else			/* (width % 1) */
		return opl_hmirror_u16_by1(src, src_line_stride, width, height,
					   dst, dst_line_stride, 1);
}

static int opl_hmirror_u16_by1(const u8 *src, int src_line_stride, int width,
			       int height, u8 *dst, int dst_line_stride,
			       int vmirror)
{
	const u8 *src_row_addr;
	const u8 *psrc;
	u8 *dst_row_addr, *pdst;
	int i, j;
	u16 pixel;

	src_row_addr = src;
	if (vmirror) {
		dst_row_addr = dst + dst_line_stride * (height - 1);
		dst_line_stride = -dst_line_stride;
	} else
		dst_row_addr = dst;

	/* Loop over all rows */
	for (i = 0; i < height; i++) {
		/* Loop over each pixel */
		psrc = src_row_addr;
		pdst = dst_row_addr + (width - 1) * BYTES_PER_PIXEL
		    - (BYTES_PER_PIXEL - BYTES_PER_PIXEL);
		for (j = 0; j < width; j++) {
			pixel = *(u16 *) psrc;
			*(u16 *) pdst = pixel;
			psrc += BYTES_PER_PIXEL;
			pdst -= BYTES_PER_PIXEL;
		}
		src_row_addr += src_line_stride;
		dst_row_addr += dst_line_stride;
	}

	return OPLERR_SUCCESS;
}

static int opl_hmirror_u16_by2(const u8 *src, int src_line_stride, int width,
			       int height, u8 *dst, int dst_line_stride,
			       int vmirror)
{
	const u8 *src_row_addr;
	const u8 *psrc;
	u8 *dst_row_addr, *pdst;
	int i, j;
	u32 pixelsin, pixelsout;

	src_row_addr = src;
	if (vmirror) {
		dst_row_addr = dst + dst_line_stride * (height - 1);
		dst_line_stride = -dst_line_stride;
	} else
		dst_row_addr = dst;

	/* Loop over all rows */
	for (i = 0; i < height; i++) {
		/* Loop over each pixel */
		psrc = src_row_addr;
		pdst = dst_row_addr + (width - 2) * BYTES_PER_PIXEL;
		for (j = 0; j < (width >> 1); j++) {
			pixelsin = *(u32 *) psrc;
			pixelsout = rot_left_u32(pixelsin, 16);
			*(u32 *) pdst = pixelsout;
			psrc += BYTES_PER_2PIXEL;
			pdst -= BYTES_PER_2PIXEL;
		}
		src_row_addr += src_line_stride;
		dst_row_addr += dst_line_stride;
	}

	return OPLERR_SUCCESS;
}

static int opl_hmirror_u16_by4(const u8 *src, int src_line_stride, int width,
			       int height, u8 *dst, int dst_line_stride,
			       int vmirror)
{
	const u8 *src_row_addr;
	const u8 *psrc;
	u8 *dst_row_addr, *pdst;
	int i, j;

	union doubleword {
		u64 dw;
		u32 w[2];
	};

	union doubleword inbuf;
	union doubleword outbuf;

	src_row_addr = src;
	if (vmirror) {
		dst_row_addr = dst + dst_line_stride * (height - 1);
		dst_line_stride = -dst_line_stride;
	} else
		dst_row_addr = dst;

	/* Loop over all rows */
	for (i = 0; i < height; i++) {
		/* Loop over each pixel */
		psrc = src_row_addr;
		pdst = dst_row_addr + (width - 4) * BYTES_PER_PIXEL;
		for (j = 0; j < (width >> 2); j++) {
			inbuf.dw = *(u64 *) psrc;
			outbuf.w[0] = rot_left_u32(inbuf.w[1], 16);
			outbuf.w[1] = rot_left_u32(inbuf.w[0], 16);
			*(u64 *) pdst = outbuf.dw;
			psrc += BYTES_PER_4PIXEL;
			pdst -= BYTES_PER_4PIXEL;
		}
		src_row_addr += src_line_stride;
		dst_row_addr += dst_line_stride;
	}
	return OPLERR_SUCCESS;
}

static int opl_hmirror_u16_by8(const u8 *src, int src_line_stride, int width,
			       int height, u8 *dst, int dst_line_stride,
			       int vmirror)
{
	const u8 *src_row_addr;
	const u8 *psrc;
	u8 *dst_row_addr, *pdst;
	int i, j;

	src_row_addr = src;
	if (vmirror) {
		dst_row_addr = dst + dst_line_stride * (height - 1);
		dst_line_stride = -dst_line_stride;
	} else
		dst_row_addr = dst;

	/* Loop over all rows */
	for (i = 0; i < height; i++) {
		/* Loop over each pixel */
		psrc = src_row_addr;
		pdst = dst_row_addr + (width - 1) * BYTES_PER_PIXEL - 2;
		for (j = (width >> 3); j > 0; j--) {
			__asm__ volatile (
				"ldmia	%0!,{r2-r5}\n\t"
				"mov	r6, r2\n\t"
				"mov	r7, r3\n\t"
				"mov	r2, r5, ROR #16\n\t"
				"mov	r3, r4, ROR #16\n\t"
				"mov	r4, r7, ROR #16\n\t"
				"mov	r5, r6, ROR #16\n\t"
				"stmda	%1!,{r2-r5}\n\t"

				: "+r"(psrc), "+r"(pdst)
				: "0"(psrc), "1"(pdst)
				: "r2", "r3", "r4", "r5", "r6", "r7",
				"memory"
			);
		}
		src_row_addr += src_line_stride;
		dst_row_addr += dst_line_stride;
	}

	return OPLERR_SUCCESS;
}

EXPORT_SYMBOL(opl_hmirror_u16);
EXPORT_SYMBOL(opl_rotate180_u16);
