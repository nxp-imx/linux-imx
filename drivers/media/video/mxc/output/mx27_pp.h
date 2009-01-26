/*
 * Copyright 2005-2007 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mx27_pp.h
 *
 * @brief Header file for MX27 V4L2 Video Output Driver
 *
 * @ingroup MXC_V4L2_OUTPUT
 */
#ifndef __MX27_PP_H__
#define __MX27_PP_H__

#include "mxc_v4l2_output.h"

/* PP register definitions */
#define PP_REG(ofs)    (IO_ADDRESS(EMMA_BASE_ADDR) - 0x400 + ofs)

/* Register offsets */
#define PP_CNTL			PP_REG(0x00)
#define PP_INTRCNTL	 	PP_REG(0x04)
#define PP_INTRSTATUS		PP_REG(0x08)
#define PP_SOURCE_Y_PTR		PP_REG(0x0C)
#define PP_SOURCE_CB_PTR	PP_REG(0x10)
#define PP_SOURCE_CR_PTR	PP_REG(0x14)
#define PP_DEST_RGB_PTR 	PP_REG(0x18)
#define PP_QUANTIZER_PTR	PP_REG(0x1C)
#define PP_PROCESS_FRAME_PARA	PP_REG(0x20)
#define PP_SOURCE_FRAME_WIDTH	PP_REG(0x24)
#define PP_DEST_DISPLAY_WIDTH	PP_REG(0x28)
#define PP_DEST_IMAGE_SIZE	PP_REG(0x2C)
#define PP_DEST_FRAME_FMT_CNTL	PP_REG(0x30)
#define PP_RESIZE_INDEX		PP_REG(0x34)
#define	PP_CSC_COEF_0123	PP_REG(0x38)
#define	PP_CSC_COEF_4		PP_REG(0x3C)
#define PP_RESIZE_COEF_TBL	PP_REG(0x100)

/* resize table dimensions
    dest pixel index    left/32    right/32    #src pixels to read
    0                   [BC_COEF]  [BC_COEF]   [BC_NXT]
    :
    pp_tbl_max-1
*/
#define BC_NXT		2
#define BC_COEF		5
#define SZ_COEF		(1 << BC_COEF)
#define SZ_NXT		(1 << BC_NXT)

/* PP operations */
#define EN_DEBLOCK 	0x02
#define EN_DERING	0x04
#define EN_CSC		0x10
#define EN_MACROBLOCK	0x20
#define EN_DEF		0x16
#define EN_MASK		0x36
#define EN_BIGDATA	0x1000
#define EN_BIGQP	0x2000

/* PP CSC tables */
#define CSC_TBL_NONE	0x80
#define CSC_TBL_REUSE	0x81
#define CSC_TBL_A1	0x00
#define CSC_TBL_A0	0x20
#define CSC_TBL_B1	0x40
#define CSC_TBL_B0	0x60
/* converts from 4 decimal fixed point to hw setting & vice versa */
#define PP_CSC_FP4_2_HW(coeff)	((((coeff) << 7) + 5000) / 10000)
#define PP_CSC_HW_2_FP4(coeff)	((((coeff) * 10000) + 64) >> 7)

#define PP_PIX_YUYV	0
#define PP_PIX_YVYU	8
#define PP_PIX_UYVY	16
#define PP_PIX_VYUY	24

/* PP size & width calculation macros */
#define PP_CALC_QP_WIDTH(cfg)	\
	(!((cfg)->operation & (EN_DEBLOCK | EN_DERING)) ? 0 : \
		(((((cfg)->dim.in.width + 15) >> 4) + 3) & ~3))
#define PP_CALC_Y_SIZE(cfg)		\
	((cfg)->in_y_stride * (cfg)->dim.in.height)
#define PP_CALC_CH_SIZE(cfg)		(PP_CALC_Y_SIZE(cfg) >> 2)
#define PP_CALC_BPP(cfg) 		\
	((cfg)->rgb_resolution > 16 ?  4 : ((cfg)->rgb_resolution >> 3))
#define PP_CALC_YUV_SIZE(cfg)		\
	((PP_CALC_Y_SIZE(cfg) * 3) >> 1)
#define PP_CALC_QP_SIZE(cfg)		\
	(PP_CALC_QP_WIDTH(cfg) * (((cfg)->dim.in.height + 15) >> 4))
#define PP_CALC_DEST_WIDTH(cfg)	\
	(((cfg)->out_stride & ~1) * PP_CALC_BPP(cfg))
#define PP_CALC_DEST_SIZE(cfg)	\
	((cfg)->dim.out.height * PP_CALC_DEST_WIDTH(cfg))

/*
 * physical addresses for bus mastering
 * v=0 -> yuv packed
 * v=0 & qp=0 -> yuv packed with qp appended
 */
typedef struct _emma_pp_ptr {
	unsigned int y;		/* Y data (line align8) */
	unsigned int u;		/* U data (line align4) */
	unsigned int v;		/* V data (line align4) */
	unsigned int qp;	/* Quantization (line align4) */
} emma_pp_ptr;

typedef struct _emma_pp_size {
	int width;
	int height;
} emma_pp_size;

/*
 * if num.width != 0
 * 	resize ratio = num.width : den.width
 * else
 * 	resize ratio = in.width : out.width
 * same for height
 */
typedef struct _emma_pp_scale {
	emma_pp_size num;
	emma_pp_size den;
	emma_pp_size in;	/* clip */
	emma_pp_size out;	/* 0 -> same as in */
} emma_pp_scale;

typedef struct _emma_pp_cfg {
	unsigned char operation;	/* OR of EN_xx defines */

	/*
	 * input color coeff
	 * fixed pt 8 bits, steps of 1/128
	 * csc[5] is 1 or 0 to indicate Y + 16
	 * csc[0] is matrix id 0-3 while csc[1-5]=0
	 */
	unsigned short csc_table[6];

	/*
	 * Output color (shade width, shade offset, pixel resolution)
	 * Eg. 16bpp RGB565 resolution, the values could be:
	 * red_width = 5, green_width = 6, blue_width = 6
	 * red_offset = 11, green_offset = 5, blue_offset = 0 (defaults)
	 * rgb_resolution = 16 (default)
	 * For YUV422: xxx_width=0, blue_offset=PP_PIX_xxx
	 */
	unsigned short red_width;
	unsigned short green_width;
	unsigned short blue_width;
	/* if offsets are 0, the offsets are by width LSb to MSb B:G:R */
	unsigned short red_offset;
	unsigned short blue_offset;
	unsigned short green_offset;
	/* if resolution is 0, the minimum for the sum of widths is chosen */
	short rgb_resolution;	/* 8,16,24 bpp only */

	emma_pp_ptr ptr;	/* dma buffer pointers */
	unsigned int outptr;	/* RGB/YUV output */
	emma_pp_scale dim;	/* in/out dimensions */

	/* pixels between two adjacent input Y rows */
	unsigned short in_y_stride;	/* 0 = in_width */
	/* PIXELS between two adjacent output rows */
	unsigned short out_stride;	/* 0 = out_width */
} emma_pp_cfg;

int pp_ptr(unsigned long ptr);
int pp_enable(int flag);
int pp_cfg(vout_data * vout);
int pp_init(vout_data * vout);
int pp_num_last(void);
void pp_exit(vout_data * vout);

#endif				/* __MX27_PP_H__ */
