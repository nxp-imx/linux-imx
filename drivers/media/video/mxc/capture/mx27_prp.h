/*
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mx27_prp.h
 *
 * @brief Header file for MX27 V4L2 capture driver
 *
 * @ingroup MXC_V4L2_CAPTURE
 */
#ifndef __MX27_PRP_H__
#define __MX27_PRP_H__

#define PRP_REG(ofs)	(IO_ADDRESS(EMMA_BASE_ADDR) + ofs)

/* Register definitions of PrP */
#define PRP_CNTL			PRP_REG(0x00)
#define PRP_INTRCNTL	 		PRP_REG(0x04)
#define PRP_INTRSTATUS			PRP_REG(0x08)
#define PRP_SOURCE_Y_PTR		PRP_REG(0x0C)
#define PRP_SOURCE_CB_PTR		PRP_REG(0x10)
#define PRP_SOURCE_CR_PTR		PRP_REG(0x14)
#define PRP_DEST_RGB1_PTR		PRP_REG(0x18)
#define PRP_DEST_RGB2_PTR		PRP_REG(0x1C)
#define PRP_DEST_Y_PTR			PRP_REG(0x20)
#define PRP_DEST_CB_PTR			PRP_REG(0x24)
#define PRP_DEST_CR_PTR			PRP_REG(0x28)
#define PRP_SOURCE_FRAME_SIZE  		PRP_REG(0x2C)
#define PRP_CH1_LINE_STRIDE		PRP_REG(0x30)
#define PRP_SRC_PIXEL_FORMAT_CNTL	PRP_REG(0x34)
#define PRP_CH1_PIXEL_FORMAT_CNTL	PRP_REG(0x38)
#define PRP_CH1_OUT_IMAGE_SIZE		PRP_REG(0x3C)
#define PRP_CH2_OUT_IMAGE_SIZE		PRP_REG(0x40)
#define PRP_SOURCE_LINE_STRIDE		PRP_REG(0x44)
#define PRP_CSC_COEF_012		PRP_REG(0x48)
#define PRP_CSC_COEF_345		PRP_REG(0x4C)
#define PRP_CSC_COEF_678		PRP_REG(0x50)
#define PRP_CH1_RZ_HORI_COEF1		PRP_REG(0x54)
#define PRP_CH1_RZ_HORI_COEF2		PRP_REG(0x58)
#define PRP_CH1_RZ_HORI_VALID		PRP_REG(0x5C)
#define PRP_CH1_RZ_VERT_COEF1		PRP_REG(0x60)
#define PRP_CH1_RZ_VERT_COEF2		PRP_REG(0x64)
#define PRP_CH1_RZ_VERT_VALID		PRP_REG(0x68)
#define PRP_CH2_RZ_HORI_COEF1		PRP_REG(0x6C)
#define PRP_CH2_RZ_HORI_COEF2		PRP_REG(0x70)
#define PRP_CH2_RZ_HORI_VALID		PRP_REG(0x74)
#define PRP_CH2_RZ_VERT_COEF1		PRP_REG(0x78)
#define PRP_CH2_RZ_VERT_COEF2		PRP_REG(0x7C)
#define PRP_CH2_RZ_VERT_VALID		PRP_REG(0x80)

#define B_SET(b)			(1 << (b))

/* Bit definitions for PrP control register */
#define PRP_CNTL_RSTVAL			0x28
#define PRP_CNTL_CH1EN			B_SET(0)
#define PRP_CNTL_CH2EN			B_SET(1)
#define PRP_CNTL_CSI			B_SET(2)
#define PRP_CNTL_IN_32			B_SET(3)
#define PRP_CNTL_IN_RGB			B_SET(4)
#define PRP_CNTL_IN_YUV420		0
#define PRP_CNTL_IN_YUV422		PRP_CNTL_IN_32
#define PRP_CNTL_IN_RGB16		PRP_CNTL_IN_RGB
#define PRP_CNTL_IN_RGB32		(PRP_CNTL_IN_RGB | PRP_CNTL_IN_32)
#define PRP_CNTL_CH1_RGB8		0
#define PRP_CNTL_CH1_RGB16		B_SET(5)
#define PRP_CNTL_CH1_RGB32		B_SET(6)
#define PRP_CNTL_CH1_YUV422		(B_SET(5) | B_SET(6))
#define PRP_CNTL_CH2_YUV420		0
#define PRP_CNTL_CH2_YUV422		B_SET(7)
#define PRP_CNTL_CH2_YUV444		B_SET(8)
#define PRP_CNTL_CH1_LOOP		B_SET(9)
#define PRP_CNTL_CH2_LOOP		B_SET(10)
#define PRP_CNTL_AUTODROP		B_SET(11)
#define PRP_CNTL_RST			B_SET(12)
#define PRP_CNTL_CNTREN			B_SET(13)
#define PRP_CNTL_WINEN			B_SET(14)
#define PRP_CNTL_UNCHAIN		B_SET(15)
#define PRP_CNTL_IN_SKIP_NONE		0
#define PRP_CNTL_IN_SKIP_1_2		B_SET(16)
#define PRP_CNTL_IN_SKIP_1_3		B_SET(17)
#define PRP_CNTL_IN_SKIP_2_3		(B_SET(16) | B_SET(17))
#define PRP_CNTL_IN_SKIP_1_4		B_SET(18)
#define PRP_CNTL_IN_SKIP_3_4		(B_SET(16) | B_SET(18))
#define PRP_CNTL_IN_SKIP_2_5		(B_SET(17) | B_SET(18))
#define PRP_CNTL_IN_SKIP_3_5		(B_SET(16) | B_SET(17) | B_SET(18))
#define PRP_CNTL_CH1_SKIP_NONE		0
#define PRP_CNTL_CH1_SKIP_1_2		B_SET(19)
#define PRP_CNTL_CH1_SKIP_1_3		B_SET(20)
#define PRP_CNTL_CH1_SKIP_2_3		(B_SET(19) | B_SET(20))
#define PRP_CNTL_CH1_SKIP_1_4		B_SET(21)
#define PRP_CNTL_CH1_SKIP_3_4		(B_SET(19) | B_SET(21))
#define PRP_CNTL_CH1_SKIP_2_5		(B_SET(20) | B_SET(21))
#define PRP_CNTL_CH1_SKIP_3_5		(B_SET(19) | B_SET(20) | B_SET(21))
#define PRP_CNTL_CH2_SKIP_NONE		0
#define PRP_CNTL_CH2_SKIP_1_2		B_SET(22)
#define PRP_CNTL_CH2_SKIP_1_3		B_SET(23)
#define PRP_CNTL_CH2_SKIP_2_3		(B_SET(22) | B_SET(23))
#define PRP_CNTL_CH2_SKIP_1_4		B_SET(24)
#define PRP_CNTL_CH2_SKIP_3_4		(B_SET(22) | B_SET(24))
#define PRP_CNTL_CH2_SKIP_2_5		(B_SET(23) | B_SET(24))
#define PRP_CNTL_CH2_SKIP_3_5		(B_SET(22) | B_SET(23) | B_SET(24))
#define PRP_CNTL_FIFO_I128		0
#define PRP_CNTL_FIFO_I96		B_SET(25)
#define PRP_CNTL_FIFO_I64		B_SET(26)
#define PRP_CNTL_FIFO_I32		(B_SET(25) | B_SET(26))
#define PRP_CNTL_FIFO_O64		0
#define PRP_CNTL_FIFO_O48		B_SET(27)
#define PRP_CNTL_FIFO_O32		B_SET(28)
#define PRP_CNTL_FIFO_O16		(B_SET(27) | B_SET(28))
#define PRP_CNTL_CH2B1			B_SET(29)
#define PRP_CNTL_CH2B2			B_SET(30)
#define PRP_CNTL_CH2_FLOWEN		B_SET(31)

/* Bit definitions for PrP interrupt control register */
#define PRP_INTRCNTL_RDERR		B_SET(0)
#define PRP_INTRCNTL_CH1WERR		B_SET(1)
#define PRP_INTRCNTL_CH2WERR		B_SET(2)
#define PRP_INTRCNTL_CH1FC		B_SET(3)
#define PRP_INTRCNTL_CH2FC		B_SET(5)
#define PRP_INTRCNTL_LBOVF		B_SET(7)
#define PRP_INTRCNTL_CH2OVF		B_SET(8)

/* Bit definitions for PrP interrupt status register */
#define PRP_INTRSTAT_RDERR		B_SET(0)
#define PRP_INTRSTAT_CH1WERR		B_SET(1)
#define PRP_INTRSTAT_CH2WERR		B_SET(2)
#define PRP_INTRSTAT_CH2BUF2		B_SET(3)
#define PRP_INTRSTAT_CH2BUF1		B_SET(4)
#define PRP_INTRSTAT_CH1BUF2		B_SET(5)
#define PRP_INTRSTAT_CH1BUF1		B_SET(6)
#define PRP_INTRSTAT_LBOVF		B_SET(7)
#define PRP_INTRSTAT_CH2OVF		B_SET(8)

#define PRP_CHANNEL_1		0x1
#define PRP_CHANNEL_2		0x2

/* PRP-CSI config */
#define PRP_CSI_EN		0x80
#define PRP_CSI_LOOP		(0x40 | PRP_CSI_EN)
#define PRP_CSI_IRQ_FRM		(0x08 | PRP_CSI_LOOP)
#define PRP_CSI_IRQ_CH1ERR	(0x10 | PRP_CSI_LOOP)
#define PRP_CSI_IRQ_CH2ERR	(0x20 | PRP_CSI_LOOP)
#define PRP_CSI_IRQ_ALL		(0x38 | PRP_CSI_LOOP)
#define PRP_CSI_SKIP_NONE	0
#define PRP_CSI_SKIP_1OF2	1
#define PRP_CSI_SKIP_1OF3	2
#define PRP_CSI_SKIP_2OF3	3
#define PRP_CSI_SKIP_1OF4	4
#define PRP_CSI_SKIP_3OF4	5
#define PRP_CSI_SKIP_2OF5	6
#define PRP_CSI_SKIP_4OF5	7

#define PRP_PIXIN_RGB565	0x2CA00565
#define PRP_PIXIN_RGB888	0x41000888
#define PRP_PIXIN_YUV420	0
#define PRP_PIXIN_YUYV		0x22000888
#define PRP_PIXIN_YVYU		0x20100888
#define PRP_PIXIN_UYVY		0x03080888
#define PRP_PIXIN_VYUY		0x01180888
#define PRP_PIXIN_YUV422	0x62080888

#define PRP_PIX1_RGB332		0x14400322
#define PRP_PIX1_RGB565		0x2CA00565
#define PRP_PIX1_RGB888		0x41000888
#define PRP_PIX1_YUYV		0x62000888
#define PRP_PIX1_YVYU		0x60100888
#define PRP_PIX1_UYVY		0x43080888
#define PRP_PIX1_VYUY		0x41180888
#define PRP_PIX1_UNUSED		0

#define PRP_PIX2_YUV420		0
#define PRP_PIX2_YUV422		1
#define PRP_PIX2_YUV444		4
#define PRP_PIX2_UNUSED		8

#define PRP_ALGO_WIDTH_ANY	0
#define PRP_ALGO_HEIGHT_ANY	0
#define PRP_ALGO_WIDTH_BIL	1
#define PRP_ALGO_WIDTH_AVG	2
#define PRP_ALGO_HEIGHT_BIL	4
#define PRP_ALGO_HEIGHT_AVG	8
#define PRP_ALGO_BYPASS		0x10

typedef struct _emma_prp_ratio {
	unsigned short num;
	unsigned short den;
} emma_prp_ratio;

/*
 * The following definitions are for resizing. Definition values must not
 * be changed otherwise decision logic will be wrong.
 */
#define SCALE_RETRY	16	/* retry times if ratio is not supported */

#define BC_COEF		3
#define MAX_TBL		20
#define SZ_COEF		(1 << BC_COEF)

#define ALGO_AUTO	0
#define ALGO_BIL	1
#define ALGO_AVG	2

typedef struct {
	char tbl[20];		/* table entries */
	char len;		/* table length used */
	char algo;		/* ALGO_xxx */
	char ratio[20];		/* ratios used */
} scale_t;

/*
 * structure for prp scaling.
 * algorithm - bilinear or averaging for each axis
 * PRP_ALGO_WIDTH_x | PRP_ALGO_HEIGHT_x | PRP_ALGO_BYPASS
 * PRP_ALGO_BYPASS - Ch1 will not use Ch2 scaling with this flag
 */
typedef struct _emma_prp_scale {
	unsigned char algo;
	emma_prp_ratio width;
	emma_prp_ratio height;
} emma_prp_scale;

typedef struct emma_prp_cfg {
	unsigned int in_pix;	/* PRP_PIXIN_xxx */
	unsigned short in_width;	/* image width, 32 - 2044 */
	unsigned short in_height;	/* image height, 32 - 2044 */
	unsigned char in_csi;	/* PRP_CSI_SKIP_x | PRP_CSI_LOOP */
	unsigned short in_line_stride;	/* in_line_stride and in_line_skip */
	unsigned short in_line_skip;	/* allow cropping from CSI */
	unsigned int in_ptr;	/* bus address */
	/*
	 * in_csc[9] = 1 -> Y-16
	 * if in_csc[1..9] == 0
	 *      in_csc[0] represents YUV range 0-3 = A0,A1,B0,B1;
	 * else
	 *      in_csc[0..9] represents either format
	 */
	unsigned short in_csc[10];

	unsigned char ch2_pix;	/* PRP_PIX2_xxx */
	emma_prp_scale ch2_scale;	/* resizing paramters */
	unsigned short ch2_width;	/* 4-2044, 0 = scaled */
	unsigned short ch2_height;	/* 4-2044, 0 = scaled */
	unsigned int ch2_ptr;	/* bus addr */
	unsigned int ch2_ptr2;	/* bus addr for 2nd buf (loop mode) */
	unsigned char ch2_csi;	/* PRP_CSI_SKIP_x | PRP_CSI_LOOP */

	unsigned int ch1_pix;	/* PRP_PIX1_xxx */
	emma_prp_scale ch1_scale;	/* resizing parameters */
	unsigned short ch1_width;	/* 4-2044, 0 = scaled */
	unsigned short ch1_height;	/* 4-2044, 0 = scaled */
	unsigned short ch1_stride;	/* 4-4088, 0 = ch1_width */
	unsigned int ch1_ptr;	/* bus addr */
	unsigned int ch1_ptr2;	/* bus addr for 2nd buf (loop mode) */
	unsigned char ch1_csi;	/* PRP_CSI_SKIP_x | PRP_CSI_LOOP */

	/*
	 * channel resizing coefficients
	 * scale[0] for channel 1 width
	 * scale[1] for channel 1 height
	 * scale[2] for channel 2 width
	 * scale[3] for channel 2 height
	 */
	scale_t scale[4];
} emma_prp_cfg;

int prphw_reset(void);
int prphw_enable(int channel);
int prphw_disable(int channel);
int prphw_inptr(emma_prp_cfg *);
int prphw_ch1ptr(emma_prp_cfg *);
int prphw_ch1ptr2(emma_prp_cfg *);
int prphw_ch2ptr(emma_prp_cfg *);
int prphw_ch2ptr2(emma_prp_cfg *);
int prphw_cfg(emma_prp_cfg *);
int prphw_isr(void);
void prphw_init(void);
void prphw_exit(void);

/*
 * scale	out	coefficient table
 * din		in	scale numerator
 * dout		in	scale denominator
 * inv		in	pre-scale dimension
 * vout		in/out	post-scale output dimension
 * pout		out	post-scale internal dimension [opt]
 * retry	in	retry times (round the output length) when need
 */
int prp_scale(scale_t * pscale, int din, int dout, int inv,
	      unsigned short *vout, unsigned short *pout, int retry);

int prp_init(void *dev_id);
void prp_exit(void *dev_id);
int prp_enc_select(void *data);
int prp_enc_deselect(void *data);
int prp_vf_select(void *data);
int prp_vf_deselect(void *data);
int prp_still_select(void *data);
int prp_still_deselect(void *data);

#endif				/* __MX27_PRP_H__ */
