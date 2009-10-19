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
#ifndef __INCLUDE_IPU_PARAM_MEM_H__
#define __INCLUDE_IPU_PARAM_MEM_H__

#include <linux/types.h>

static __inline void _ipu_ch_param_set_size(uint32_t * params,
					    uint32_t pixel_fmt, uint16_t width,
					    uint16_t height, uint16_t stride,
					    uint32_t u, uint32_t v)
{
	uint32_t u_offset = 0;
	uint32_t v_offset = 0;
	memset(params, 0, 40);

	params[3] =
	    (uint32_t) ((width - 1) << 12) | ((uint32_t) (height - 1) << 24);
	params[4] = (uint32_t) (height - 1) >> 8;
	params[7] = (uint32_t) (stride - 1) << 3;

	switch (pixel_fmt) {
	case IPU_PIX_FMT_GENERIC:
		/*Represents 8-bit Generic data */
		params[7] |= 3 | (7UL << (81 - 64)) | (31L << (89 - 64));	/* BPP & PFS */
		params[8] = 2;	/* SAT = use 32-bit access */
		break;
	case IPU_PIX_FMT_GENERIC_32:
		/*Represents 32-bit Generic data */
		params[7] |= (7UL << (81 - 64)) | (7L << (89 - 64));	/* BPP & PFS */
		params[8] = 2;	/* SAT = use 32-bit access */
		break;
	case IPU_PIX_FMT_RGB565:
		params[7] |= 2L | (4UL << (81 - 64)) | (7L << (89 - 64));	/* BPP & PFS */
		params[8] = 2 |	/* SAT = 32-bit access */
		    (0UL << (99 - 96)) |	/* Red bit offset */
		    (5UL << (104 - 96)) |	/* Green bit offset     */
		    (11UL << (109 - 96)) |	/* Blue bit offset */
		    (16UL << (114 - 96)) |	/* Alpha bit offset     */
		    (4UL << (119 - 96)) |	/* Red bit width - 1 */
		    (5UL << (122 - 96)) |	/* Green bit width - 1 */
		    (4UL << (125 - 96));	/* Blue bit width - 1 */
		break;
	case IPU_PIX_FMT_BGR24:	/* 24 BPP & RGB PFS */
		params[7] |= 1 | (4UL << (81 - 64)) | (7L << (89 - 64));
		params[8] = 2 |	/* SAT = 32-bit access */
		    (8UL << (104 - 96)) |	/* Green bit offset     */
		    (16UL << (109 - 96)) |	/* Blue bit offset */
		    (24UL << (114 - 96)) |	/* Alpha bit offset     */
		    (7UL << (119 - 96)) |	/* Red bit width - 1 */
		    (7UL << (122 - 96)) |	/* Green bit width - 1 */
		    (uint32_t) (7UL << (125 - 96));	/* Blue bit width - 1 */
		break;
	case IPU_PIX_FMT_RGB24:	/* 24 BPP & RGB PFS     */
		params[7] |= 1 | (4UL << (81 - 64)) | (7L << (89 - 64));
		params[8] = 2 |	/* SAT = 32-bit access */
		    (16UL << (99 - 96)) |	/* Red bit offset */
		    (8UL << (104 - 96)) |	/* Green bit offset */
		    (0UL << (109 - 96)) |	/* Blue bit offset */
		    (24UL << (114 - 96)) |	/* Alpha bit offset */
		    (7UL << (119 - 96)) |	/* Red bit width - 1 */
		    (7UL << (122 - 96)) |	/* Green bit width - 1 */
		    (uint32_t) (7UL << (125 - 96));	/* Blue bit width - 1 */
		break;
	case IPU_PIX_FMT_BGRA32:
	case IPU_PIX_FMT_BGR32:
		/* BPP & pixel fmt */
		params[7] |= 0 | (4UL << (81 - 64)) | (7 << (89 - 64));
		params[8] = 2 |	/* SAT = 32-bit access */
		    (8UL << (99 - 96)) |	/* Red bit offset */
		    (16UL << (104 - 96)) |	/* Green bit offset     */
		    (24UL << (109 - 96)) |	/* Blue bit offset */
		    (0UL << (114 - 96)) |	/* Alpha bit offset     */
		    (7UL << (119 - 96)) |	/* Red bit width - 1 */
		    (7UL << (122 - 96)) |	/* Green bit width - 1 */
		    (uint32_t) (7UL << (125 - 96));	/* Blue bit width - 1 */
		params[9] = 7;	/* Alpha bit width - 1 */
		break;
	case IPU_PIX_FMT_RGBA32:
	case IPU_PIX_FMT_RGB32:
		/* BPP & pixel fmt */
		params[7] |= 0 | (4UL << (81 - 64)) | (7 << (89 - 64));
		params[8] = 2 |	/* SAT = 32-bit access */
		    (24UL << (99 - 96)) |	/* Red bit offset */
		    (16UL << (104 - 96)) |	/* Green bit offset     */
		    (8UL << (109 - 96)) |	/* Blue bit offset */
		    (0UL << (114 - 96)) |	/* Alpha bit offset     */
		    (7UL << (119 - 96)) |	/* Red bit width - 1 */
		    (7UL << (122 - 96)) |	/* Green bit width - 1 */
		    (uint32_t) (7UL << (125 - 96));	/* Blue bit width - 1 */
		params[9] = 7;	/* Alpha bit width - 1 */
		break;
	case IPU_PIX_FMT_ABGR32:
		/* BPP & pixel fmt */
		params[7] |= 0 | (4UL << (81 - 64)) | (7 << (89 - 64));
		params[8] = 2 |	/* SAT = 32-bit access */
		    (0UL << (99 - 96)) |	/* Alpha bit offset     */
		    (8UL << (104 - 96)) |	/* Blue bit offset */
		    (16UL << (109 - 96)) |	/* Green bit offset     */
		    (24UL << (114 - 96)) |	/* Red bit offset */
		    (7UL << (119 - 96)) |	/* Alpha bit width - 1 */
		    (7UL << (122 - 96)) |	/* Blue bit width - 1 */
		    (uint32_t) (7UL << (125 - 96));	/* Green bit width - 1 */
		params[9] = 7;	/* Red bit width - 1 */
		break;
	case IPU_PIX_FMT_UYVY:
		/* BPP & pixel format */
		params[7] |= 2 | (6UL << 17) | (7 << (89 - 64));
		params[8] = 2;	/* SAT = 32-bit access */
		break;
	case IPU_PIX_FMT_YUV420P2:
	case IPU_PIX_FMT_YUV420P:
		/* BPP & pixel format */
		params[7] |= 3 | (3UL << 17) | (7 << (89 - 64));
		params[8] = 2;	/* SAT = 32-bit access */
		u_offset = (u == 0) ? stride * height : u;
		v_offset = (v == 0) ? u_offset + u_offset / 4 : v;
		break;
	case IPU_PIX_FMT_YVU422P:
		/* BPP & pixel format */
		params[7] |= 3 | (2UL << 17) | (7 << (89 - 64));
		params[8] = 2;	/* SAT = 32-bit access */
		v_offset = (v == 0) ? stride * height : v;
		u_offset = (u == 0) ? v_offset + v_offset / 2 : u;
		break;
	case IPU_PIX_FMT_YUV422P:
		/* BPP & pixel format */
		params[7] |= 3 | (2UL << 17) | (7 << (89 - 64));
		params[8] = 2;	/* SAT = 32-bit access */
		u_offset = (u == 0) ? stride * height : u;
		v_offset = (v == 0) ? u_offset + u_offset / 2 : v;
		break;
	default:
		dev_err(g_ipu_dev, "mxc ipu: unimplemented pixel format\n");
		break;
	}

	params[1] = (1UL << (46 - 32)) | (u_offset << (53 - 32));
	params[2] = u_offset >> (64 - 53);
	params[2] |= v_offset << (79 - 64);
	params[3] |= v_offset >> (96 - 79);
}

static __inline void _ipu_ch_param_set_burst_size(uint32_t * params,
						  uint16_t burst_pixels)
{
	params[7] &= ~(0x3FL << (89 - 64));
	params[7] |= (uint32_t) (burst_pixels - 1) << (89 - 64);
};

static __inline void _ipu_ch_param_set_buffer(uint32_t * params,
					      dma_addr_t buf0, dma_addr_t buf1)
{
	params[5] = buf0;
	params[6] = buf1;
};

static __inline void _ipu_ch_param_set_rotation(uint32_t * params,
						ipu_rotate_mode_t rot)
{
	params[7] |= (uint32_t) rot << (84 - 64);
};

void _ipu_write_param_mem(uint32_t addr, uint32_t * data, uint32_t numWords);

#endif
