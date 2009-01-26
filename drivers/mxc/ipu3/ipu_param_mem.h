/*
 * Copyright 2005-2008 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __INCLUDE_IPU_PARAM_MEM_H__
#define __INCLUDE_IPU_PARAM_MEM_H__

#include <linux/types.h>
#include <linux/bitrev.h>

extern u32 *ipu_cpmem_base;

struct ipu_ch_param_word {
	uint32_t data[5];
	uint32_t res[3];
};

struct ipu_ch_param {
	struct ipu_ch_param_word word[2];
};

#define ipu_ch_param_addr(ch) (((struct ipu_ch_param *)ipu_cpmem_base) + (ch))

#define _param_word(base, w) \
	(((struct ipu_ch_param *)(base))->word[(w)].data)

#define ipu_ch_param_set_field(base, w, bit, size, v) { \
	int i = (bit) / 32; \
	int off = (bit) % 32; \
	_param_word(base, w)[i] |= (v) << off; \
	if (((bit)+(size)-1)/32 > i) { \
		_param_word(base, w)[i + 1] |= (v) >> (off ? (32 - off) : 0); \
	} \
}

#define ipu_ch_param_mod_field(base, w, bit, size, v) { \
	int i = (bit) / 32; \
	int off = (bit) % 32; \
	u32 mask = (1UL << size) - 1; \
	u32 temp = _param_word(base, w)[i]; \
	temp &= ~(mask << off); \
	_param_word(base, w)[i] = temp | (v) << off; \
	if (((bit)+(size)-1)/32 > i) { \
		temp = _param_word(base, w)[i + 1]; \
		temp &= ~(mask >> (32 - off)); \
		_param_word(base, w)[i + 1] = \
			temp | ((v) >> (off ? (32 - off) : 0)); \
	} \
}

#define ipu_ch_param_read_field(base, w, bit, size) ({ \
	u32 temp2; \
	int i = (bit) / 32; \
	int off = (bit) % 32; \
	u32 mask = (1UL << size) - 1; \
	u32 temp1 = _param_word(base, w)[i]; \
	temp1 = mask & (temp1 >> off); \
	if (((bit)+(size)-1)/32 > i) { \
		temp2 = _param_word(base, w)[i + 1]; \
		temp2 &= mask >> (off ? (32 - off) : 0); \
		temp1 |= temp2 << (off ? (32 - off) : 0); \
	} \
	temp1; \
})

static inline void _ipu_ch_params_set_packing(struct ipu_ch_param *p,
					      int red_width, int red_offset,
					      int green_width, int green_offset,
					      int blue_width, int blue_offset,
					      int alpha_width, int alpha_offset)
{
	/* Setup red width and offset */
	ipu_ch_param_set_field(p, 1, 116, 3, red_width - 1);
	ipu_ch_param_set_field(p, 1, 128, 5, red_offset);
	/* Setup green width and offset */
	ipu_ch_param_set_field(p, 1, 119, 3, green_width - 1);
	ipu_ch_param_set_field(p, 1, 133, 5, green_offset);
	/* Setup blue width and offset */
	ipu_ch_param_set_field(p, 1, 122, 3, blue_width - 1);
	ipu_ch_param_set_field(p, 1, 138, 5, blue_offset);
	/* Setup alpha width and offset */
	ipu_ch_param_set_field(p, 1, 125, 3, alpha_width - 1);
	ipu_ch_param_set_field(p, 1, 143, 5, alpha_offset);
}

static inline void _ipu_ch_param_dump(int ch)
{
	struct ipu_ch_param *p = ipu_ch_param_addr(ch);
	pr_debug("ch %d word 0 - %08X %08X %08X %08X %08X\n", ch,
		 p->word[0].data[0], p->word[0].data[1], p->word[0].data[2],
		 p->word[0].data[3], p->word[0].data[4]);
	pr_debug("ch %d word 1 - %08X %08X %08X %08X %08X\n", ch,
		 p->word[1].data[0], p->word[1].data[1], p->word[1].data[2],
		 p->word[1].data[3], p->word[1].data[4]);
	pr_debug("PFS 0x%x, ",
		 ipu_ch_param_read_field(ipu_ch_param_addr(ch), 1, 85, 4));
	pr_debug("BPP 0x%x, ",
		 ipu_ch_param_read_field(ipu_ch_param_addr(ch), 0, 107, 3));
	pr_debug("NPB 0x%x\n",
		 ipu_ch_param_read_field(ipu_ch_param_addr(ch), 1, 78, 7));

	pr_debug("FW %d, ",
		 ipu_ch_param_read_field(ipu_ch_param_addr(ch), 0, 125, 13));
	pr_debug("FH %d, ",
		 ipu_ch_param_read_field(ipu_ch_param_addr(ch), 0, 138, 12));
	pr_debug("Stride %d\n",
		 ipu_ch_param_read_field(ipu_ch_param_addr(ch), 1, 102, 14));

	pr_debug("Width0 %d+1, ",
		 ipu_ch_param_read_field(ipu_ch_param_addr(ch), 1, 116, 3));
	pr_debug("Width1 %d+1, ",
		 ipu_ch_param_read_field(ipu_ch_param_addr(ch), 1, 119, 3));
	pr_debug("Width2 %d+1, ",
		 ipu_ch_param_read_field(ipu_ch_param_addr(ch), 1, 122, 3));
	pr_debug("Width3 %d+1, ",
		 ipu_ch_param_read_field(ipu_ch_param_addr(ch), 1, 125, 3));
	pr_debug("Offset0 %d, ",
		 ipu_ch_param_read_field(ipu_ch_param_addr(ch), 1, 128, 5));
	pr_debug("Offset1 %d, ",
		 ipu_ch_param_read_field(ipu_ch_param_addr(ch), 1, 133, 5));
	pr_debug("Offset2 %d, ",
		 ipu_ch_param_read_field(ipu_ch_param_addr(ch), 1, 138, 5));
	pr_debug("Offset3 %d\n",
		 ipu_ch_param_read_field(ipu_ch_param_addr(ch), 1, 143, 5));
}

static inline void _ipu_ch_param_init(int ch,
				      uint32_t pixel_fmt, uint32_t width,
				      uint32_t height, uint32_t stride,
				      uint32_t u, uint32_t v,
				      uint32_t uv_stride, dma_addr_t addr0,
				      dma_addr_t addr1)
{
	uint32_t u_offset = 0;
	uint32_t v_offset = 0;
	struct ipu_ch_param params;

	memset(&params, 0, sizeof(params));

	ipu_ch_param_set_field(&params, 0, 125, 13, width - 1);
	ipu_ch_param_set_field(&params, 0, 138, 12, height - 1);
	ipu_ch_param_set_field(&params, 1, 102, 14, stride - 1);

	ipu_ch_param_set_field(&params, 1, 0, 29, addr0 >> 3);
	ipu_ch_param_set_field(&params, 1, 29, 29, addr1 >> 3);

	switch (pixel_fmt) {
	case IPU_PIX_FMT_GENERIC:
		/*Represents 8-bit Generic data */
		ipu_ch_param_set_field(&params, 0, 107, 3, 5);	/* bits/pixel */
		ipu_ch_param_set_field(&params, 1, 85, 4, 6);	/* pix format */
		ipu_ch_param_set_field(&params, 1, 78, 7, 63);	/* burst size */
		break;
	case IPU_PIX_FMT_GENERIC_32:
		/*Represents 32-bit Generic data */
		break;
	case IPU_PIX_FMT_RGB565:
		ipu_ch_param_set_field(&params, 0, 107, 3, 3);	/* bits/pixel */
		ipu_ch_param_set_field(&params, 1, 85, 4, 7);	/* pix format */
		ipu_ch_param_set_field(&params, 1, 78, 7, 31);	/* burst size */

		_ipu_ch_params_set_packing(&params, 5, 0, 6, 5, 5, 11, 1, 16);
		break;
	case IPU_PIX_FMT_BGR24:
		ipu_ch_param_set_field(&params, 0, 107, 3, 1);	/* bits/pixel */
		ipu_ch_param_set_field(&params, 1, 85, 4, 7);	/* pix format */
		ipu_ch_param_set_field(&params, 1, 78, 7, 19);	/* burst size */

		_ipu_ch_params_set_packing(&params, 8, 0, 8, 8, 8, 16, 1, 24);
		break;
	case IPU_PIX_FMT_RGB24:
	case IPU_PIX_FMT_YUV444:
		ipu_ch_param_set_field(&params, 0, 107, 3, 1);	/* bits/pixel */
		ipu_ch_param_set_field(&params, 1, 85, 4, 7);	/* pix format */
		ipu_ch_param_set_field(&params, 1, 78, 7, 19);	/* burst size */

		_ipu_ch_params_set_packing(&params, 8, 16, 8, 8, 8, 0, 1, 24);
		break;
	case IPU_PIX_FMT_BGRA32:
	case IPU_PIX_FMT_BGR32:
		ipu_ch_param_set_field(&params, 0, 107, 3, 0);	/* bits/pixel */
		ipu_ch_param_set_field(&params, 1, 85, 4, 7);	/* pix format */
		ipu_ch_param_set_field(&params, 1, 78, 7, 15);	/* burst size */

		_ipu_ch_params_set_packing(&params, 8, 8, 8, 16, 8, 24, 8, 0);
		break;
	case IPU_PIX_FMT_RGBA32:
	case IPU_PIX_FMT_RGB32:
		ipu_ch_param_set_field(&params, 0, 107, 3, 0);	/* bits/pixel */
		ipu_ch_param_set_field(&params, 1, 85, 4, 7);	/* pix format */
		ipu_ch_param_set_field(&params, 1, 78, 7, 15);	/* burst size */

		_ipu_ch_params_set_packing(&params, 8, 24, 8, 16, 8, 8, 8, 0);
		break;
	case IPU_PIX_FMT_ABGR32:
		ipu_ch_param_set_field(&params, 0, 107, 3, 0);	/* bits/pixel */
		ipu_ch_param_set_field(&params, 1, 85, 4, 7);	/* pix format */
		ipu_ch_param_set_field(&params, 1, 78, 7, 15);	/* burst size */

		_ipu_ch_params_set_packing(&params, 8, 0, 8, 8, 8, 16, 8, 24);
		break;
	case IPU_PIX_FMT_UYVY:
		ipu_ch_param_set_field(&params, 0, 107, 3, 3);	/* bits/pixel */
		ipu_ch_param_set_field(&params, 1, 85, 4, 0xA);	/* pix format */
		ipu_ch_param_set_field(&params, 1, 78, 7, 31);	/* burst size */
		break;
	case IPU_PIX_FMT_YUYV:
		ipu_ch_param_set_field(&params, 0, 107, 3, 3);	/* bits/pixel */
		ipu_ch_param_set_field(&params, 1, 85, 4, 0x8);	/* pix format */
		ipu_ch_param_set_field(&params, 1, 78, 7, 31);	/* burst size */
		break;
	case IPU_PIX_FMT_YUV420P2:
	case IPU_PIX_FMT_YUV420P:
		ipu_ch_param_set_field(&params, 1, 85, 4, 2);	/* pix format */
		ipu_ch_param_set_field(&params, 1, 78, 7, 31);	/* burst size */

		if (uv_stride < stride / 2)
			uv_stride = stride / 2;

		u_offset = stride * height;
		v_offset = u_offset + (uv_stride * height / 2);
		break;
	case IPU_PIX_FMT_YVU422P:
		/* BPP & pixel format */
		ipu_ch_param_set_field(&params, 1, 85, 4, 1);	/* pix format */
		ipu_ch_param_set_field(&params, 1, 78, 7, 31);	/* burst size */

		if (uv_stride < stride / 2)
			uv_stride = stride / 2;

		v_offset = (v == 0) ? stride * height : v;
		u_offset = (u == 0) ? v_offset + v_offset / 2 : u;
		break;
	case IPU_PIX_FMT_YUV422P:
		/* BPP & pixel format */
		ipu_ch_param_set_field(&params, 1, 85, 4, 1);	/* pix format */
		ipu_ch_param_set_field(&params, 1, 78, 7, 31);	/* burst size */

		if (uv_stride < stride / 2)
			uv_stride = stride / 2;

		u_offset = (u == 0) ? stride * height : u;
		v_offset = (v == 0) ? u_offset + u_offset / 2 : v;
		break;
	case IPU_PIX_FMT_NV12:
		/* BPP & pixel format */
		ipu_ch_param_set_field(&params, 1, 85, 4, 4);	/* pix format */
		ipu_ch_param_set_field(&params, 1, 78, 7, 31);	/* burst size */
		uv_stride = stride;
		u_offset = (u == 0) ? stride * height : u;
		break;
	default:
		dev_err(g_ipu_dev, "mxc ipu: unimplemented pixel format\n");
		break;
	}

	if (uv_stride)
		ipu_ch_param_set_field(&params, 1, 128, 14, uv_stride - 1);

	if (u > u_offset)
		u_offset = u;

	if (v > v_offset)
		v_offset = v;

	ipu_ch_param_set_field(&params, 0, 46, 22, u_offset / 8);
	ipu_ch_param_set_field(&params, 0, 68, 22, v_offset / 8);

	pr_debug("initializing idma ch %d @ %p\n", ch, ipu_ch_param_addr(ch));
	memcpy(ipu_ch_param_addr(ch), &params, sizeof(params));
};

static inline void _ipu_ch_param_set_burst_size(uint32_t ch,
						uint16_t burst_pixels)
{
	ipu_ch_param_mod_field(ipu_ch_param_addr(ch), 1, 78, 7,
			       burst_pixels - 1);
};

static inline int _ipu_ch_param_get_burst_size(uint32_t ch)
{
	return ipu_ch_param_read_field(ipu_ch_param_addr(ch), 1, 78, 7) + 1;
};

static inline int _ipu_ch_param_get_bpp(uint32_t ch)
{
	return ipu_ch_param_read_field(ipu_ch_param_addr(ch), 0, 107, 3);
};

static inline void _ipu_ch_param_set_buffer(uint32_t ch, int bufNum,
					    dma_addr_t phyaddr)
{
	ipu_ch_param_mod_field(ipu_ch_param_addr(ch), 1, 29 * bufNum, 29,
			       phyaddr / 8);
};

static inline void _ipu_ch_param_set_rotation(uint32_t ch,
					      ipu_rotate_mode_t rot)
{
	u32 temp_rot = bitrev8(rot) >> 5;
	ipu_ch_param_mod_field(ipu_ch_param_addr(ch), 0, 119, 3, temp_rot);
};

static inline void _ipu_ch_param_set_block_mode(uint32_t ch)
{
	ipu_ch_param_mod_field(ipu_ch_param_addr(ch), 0, 117, 2, 1);
};

static inline void _ipu_ch_param_set_interlaced_scan(uint32_t ch)
{
	u32 stride;
	ipu_ch_param_set_field(ipu_ch_param_addr(ch), 0, 113, 1, 1);
	stride = ipu_ch_param_read_field(ipu_ch_param_addr(ch), 1, 102, 14) + 1;
	ipu_ch_param_mod_field(ipu_ch_param_addr(ch), 1, 58, 20, stride / 8);
	stride *= 2;
	ipu_ch_param_mod_field(ipu_ch_param_addr(ch), 1, 102, 14, stride - 1);
};

static inline void _ipu_ch_param_set_high_priority(uint32_t ch)
{
	ipu_ch_param_mod_field(ipu_ch_param_addr(ch), 1, 93, 2, 1);
};

#endif
