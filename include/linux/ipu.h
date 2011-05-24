/*
 * Copyright 2005-2011 Freescale Semiconductor, Inc.
 */

/*
 * The code contained herein is licensed under the GNU Lesser General
 * Public License.  You may obtain a copy of the GNU Lesser General
 * Public License Version 2.1 or later at the following locations:
 *
 * http://www.opensource.org/licenses/lgpl-license.html
 * http://www.gnu.org/copyleft/lgpl.html
 */

/*!
 * @defgroup IPU MXC Image Processing Unit (IPU) Driver
 */
/*!
 * @file arch-mxc/ipu.h
 *
 * @brief This file contains the IPU driver API declarations.
 *
 * @ingroup IPU
 */

#ifndef __ASM_ARCH_IPU_H__
#define __ASM_ARCH_IPU_H__

#include <linux/types.h>
#include <linux/videodev2.h>
#ifdef __KERNEL__
#include <linux/interrupt.h>
#else
#ifndef __cplusplus
typedef unsigned char bool;
#endif
#define irqreturn_t int
#define dma_addr_t int
#define u32 unsigned int
#define __u32 u32
#endif

/*!
 * Enumeration of IPU rotation modes
 */
typedef enum {
	/* Note the enum values correspond to BAM value */
	IPU_ROTATE_NONE = 0,
	IPU_ROTATE_VERT_FLIP = 1,
	IPU_ROTATE_HORIZ_FLIP = 2,
	IPU_ROTATE_180 = 3,
	IPU_ROTATE_90_RIGHT = 4,
	IPU_ROTATE_90_RIGHT_VFLIP = 5,
	IPU_ROTATE_90_RIGHT_HFLIP = 6,
	IPU_ROTATE_90_LEFT = 7,
} ipu_rotate_mode_t;

/*!
 * Enumeration of Post Filter modes
 */
typedef enum {
	PF_DISABLE_ALL = 0,
	PF_MPEG4_DEBLOCK = 1,
	PF_MPEG4_DERING = 2,
	PF_MPEG4_DEBLOCK_DERING = 3,
	PF_H264_DEBLOCK = 4,
} pf_operation_t;

/*!
 * Enumeration of Synchronous (Memory-less) panel types
 */
typedef enum {
	IPU_PANEL_SHARP_TFT,
	IPU_PANEL_TFT,
} ipu_panel_t;

/*!
 * Enumeration of VDI MOTION select
 */
typedef enum {
	MED_MOTION = 0,
	LOW_MOTION = 1,
	HIGH_MOTION = 2,
} ipu_motion_sel;

/*  IPU Pixel format definitions */
/*  Four-character-code (FOURCC) */
#define fourcc(a, b, c, d)\
	 (((__u32)(a)<<0)|((__u32)(b)<<8)|((__u32)(c)<<16)|((__u32)(d)<<24))

/*!
 * @name IPU Pixel Formats
 *
 * Pixel formats are defined with ASCII FOURCC code. The pixel format codes are
 * the same used by V4L2 API.
 */

/*! @{ */
/*! @name Generic or Raw Data Formats */
/*! @{ */
#define IPU_PIX_FMT_GENERIC fourcc('I', 'P', 'U', '0')	/*!< IPU Generic Data */
#define IPU_PIX_FMT_GENERIC_32 fourcc('I', 'P', 'U', '1')	/*!< IPU Generic Data */
#define IPU_PIX_FMT_LVDS666 fourcc('L', 'V', 'D', '6')	/*!< IPU Generic Data */
#define IPU_PIX_FMT_LVDS888 fourcc('L', 'V', 'D', '8')	/*!< IPU Generic Data */
/*! @} */
/*! @name RGB Formats */
/*! @{ */
#define IPU_PIX_FMT_RGB332  fourcc('R', 'G', 'B', '1')	/*!<  8  RGB-3-3-2    */
#define IPU_PIX_FMT_RGB555  fourcc('R', 'G', 'B', 'O')	/*!< 16  RGB-5-5-5    */
#define IPU_PIX_FMT_RGB565  fourcc('R', 'G', 'B', 'P')	/*!< 1 6  RGB-5-6-5   */
#define IPU_PIX_FMT_RGB666  fourcc('R', 'G', 'B', '6')	/*!< 18  RGB-6-6-6    */
#define IPU_PIX_FMT_BGR666  fourcc('B', 'G', 'R', '6')	/*!< 18  BGR-6-6-6    */
#define IPU_PIX_FMT_BGR24   fourcc('B', 'G', 'R', '3')	/*!< 24  BGR-8-8-8    */
#define IPU_PIX_FMT_RGB24   fourcc('R', 'G', 'B', '3')	/*!< 24  RGB-8-8-8    */
#define IPU_PIX_FMT_GBR24   fourcc('G', 'B', 'R', '3')	/*!< 24  GBR-8-8-8    */
#define IPU_PIX_FMT_BGR32   fourcc('B', 'G', 'R', '4')	/*!< 32  BGR-8-8-8-8  */
#define IPU_PIX_FMT_BGRA32  fourcc('B', 'G', 'R', 'A')	/*!< 32  BGR-8-8-8-8  */
#define IPU_PIX_FMT_RGB32   fourcc('R', 'G', 'B', '4')	/*!< 32  RGB-8-8-8-8  */
#define IPU_PIX_FMT_RGBA32  fourcc('R', 'G', 'B', 'A')	/*!< 32  RGB-8-8-8-8  */
#define IPU_PIX_FMT_ABGR32  fourcc('A', 'B', 'G', 'R')	/*!< 32  ABGR-8-8-8-8 */
/*! @} */
/*! @name YUV Interleaved Formats */
/*! @{ */
#define IPU_PIX_FMT_YUYV    fourcc('Y', 'U', 'Y', 'V')	/*!< 16 YUV 4:2:2 */
#define IPU_PIX_FMT_UYVY    fourcc('U', 'Y', 'V', 'Y')	/*!< 16 YUV 4:2:2 */
#define IPU_PIX_FMT_YVYU    fourcc('Y', 'V', 'Y', 'U')  /*!< 16 YVYU 4:2:2 */
#define IPU_PIX_FMT_VYUY    fourcc('V', 'Y', 'U', 'Y')  /*!< 16 VYYU 4:2:2 */
#define IPU_PIX_FMT_Y41P    fourcc('Y', '4', '1', 'P')	/*!< 12 YUV 4:1:1 */
#define IPU_PIX_FMT_YUV444  fourcc('Y', '4', '4', '4')	/*!< 24 YUV 4:4:4 */
/* two planes -- one Y, one Cb + Cr interleaved  */
#define IPU_PIX_FMT_NV12    fourcc('N', 'V', '1', '2') /* 12  Y/CbCr 4:2:0  */
/*! @} */
/*! @name YUV Planar Formats */
/*! @{ */
#define IPU_PIX_FMT_GREY    fourcc('G', 'R', 'E', 'Y')	/*!< 8  Greyscale */
#define IPU_PIX_FMT_YVU410P fourcc('Y', 'V', 'U', '9')	/*!< 9  YVU 4:1:0 */
#define IPU_PIX_FMT_YUV410P fourcc('Y', 'U', 'V', '9')	/*!< 9  YUV 4:1:0 */
#define IPU_PIX_FMT_YVU420P fourcc('Y', 'V', '1', '2')	/*!< 12 YVU 4:2:0 */
#define IPU_PIX_FMT_YUV420P fourcc('I', '4', '2', '0')	/*!< 12 YUV 4:2:0 */
#define IPU_PIX_FMT_YUV420P2 fourcc('Y', 'U', '1', '2')	/*!< 12 YUV 4:2:0 */
#define IPU_PIX_FMT_YVU422P fourcc('Y', 'V', '1', '6')	/*!< 16 YVU 4:2:2 */
#define IPU_PIX_FMT_YUV422P fourcc('4', '2', '2', 'P')	/*!< 16 YUV 4:2:2 */
/*! @} */

/* IPU Driver channels definitions.	*/
/* Note these are different from IDMA channels */
#ifdef CONFIG_MXC_IPU_V1
#define _MAKE_CHAN(num, in, out, sec)    ((num << 24) | (sec << 16) | (out << 8) | in)
#define IPU_CHAN_ID(ch)         (ch >> 24)
#define IPU_CHAN_SEC_DMA(ch)    ((uint32_t) (ch >> 16) & 0xFF)
#define IPU_CHAN_OUT_DMA(ch)    ((uint32_t) (ch >> 8) & 0xFF)
#define IPU_CHAN_IN_DMA(ch)     ((uint32_t) (ch & 0xFF))

#else
#define IPU_MAX_CH	32
#define _MAKE_CHAN(num, v_in, g_in, a_in, out) \
	((num << 24) | (v_in << 18) | (g_in << 12) | (a_in << 6) | out)
#define _MAKE_ALT_CHAN(ch)		(ch | (IPU_MAX_CH << 24))
#define IPU_CHAN_ID(ch)			(ch >> 24)
#define IPU_CHAN_ALT(ch)		(ch & 0x02000000)
#define IPU_CHAN_ALPHA_IN_DMA(ch)	((uint32_t) (ch >> 6) & 0x3F)
#define IPU_CHAN_GRAPH_IN_DMA(ch)	((uint32_t) (ch >> 12) & 0x3F)
#define IPU_CHAN_VIDEO_IN_DMA(ch)	((uint32_t) (ch >> 18) & 0x3F)
#define IPU_CHAN_OUT_DMA(ch)		((uint32_t) (ch & 0x3F))
#define NO_DMA 0x3F
#define ALT	1
#endif
/*!
 * Enumeration of IPU logical channels. An IPU logical channel is defined as a
 * combination of an input (memory to IPU), output (IPU to memory), and/or
 * secondary input IDMA channels and in some cases an Image Converter task.
 * Some channels consist of only an input or output.
 */
typedef enum {
	CHAN_NONE = -1,
#ifdef CONFIG_MXC_IPU_V1
	CSI_MEM = _MAKE_CHAN(1, 0xFF, 7, 0xFF),	/*!< CSI raw sensor data to memory */

	CSI_PRP_ENC_MEM = _MAKE_CHAN(2, 0xFF, 0, 0xFF),	/*!< CSI to IC Encoder PreProcessing to Memory */
	MEM_PRP_ENC_MEM = _MAKE_CHAN(3, 6, 0, 0xFF),	/*!< Memory to IC Encoder PreProcessing to Memory */
	MEM_ROT_ENC_MEM = _MAKE_CHAN(4, 10, 8, 0xFF),	/*!< Memory to IC Encoder Rotation to Memory */

	CSI_PRP_VF_MEM = _MAKE_CHAN(5, 0xFF, 1, 0xFF),	/*!< CSI to IC Viewfinder PreProcessing to Memory */
	CSI_PRP_VF_ADC = _MAKE_CHAN(6, 0xFF, 1, 0xFF),	/*!< CSI to IC Viewfinder PreProcessing to ADC */
	MEM_PRP_VF_MEM = _MAKE_CHAN(7, 6, 1, 3),	/*!< Memory to IC Viewfinder PreProcessing to Memory */
	MEM_PRP_VF_ADC = _MAKE_CHAN(8, 6, 1, 3),	/*!< Memory to IC Viewfinder PreProcessing to ADC */
	MEM_ROT_VF_MEM = _MAKE_CHAN(9, 11, 9, 0xFF),	/*!< Memory to IC Viewfinder Rotation to Memory */

	MEM_PP_MEM = _MAKE_CHAN(10, 5, 2, 4),	/*!< Memory to IC PostProcessing to Memory */
	MEM_ROT_PP_MEM = _MAKE_CHAN(11, 13, 12, 0xFF),	/*!< Memory to IC PostProcessing Rotation to Memory */
	MEM_PP_ADC = _MAKE_CHAN(12, 5, 2, 4),	/*!< Memory to IC PostProcessing to ADC */

	MEM_SDC_BG = _MAKE_CHAN(14, 14, 0xFF, 0xFF),	/*!< Memory to SDC Background plane */
	MEM_SDC_FG = _MAKE_CHAN(15, 15, 0xFF, 0xFF),	/*!< Memory to SDC Foreground plane */
	MEM_SDC_MASK = _MAKE_CHAN(16, 16, 0xFF, 0xFF),	/*!< Memory to SDC Mask */

	MEM_BG_SYNC = MEM_SDC_BG,
	MEM_FG_SYNC = MEM_SDC_FG,

	ADC_SYS1 = _MAKE_CHAN(17, 18, 22, 20),	/*!< Memory to ADC System Channel 1 */
	ADC_SYS2 = _MAKE_CHAN(18, 19, 23, 21),	/*!< Memory to ADC System Channel 2 */

	MEM_PF_Y_MEM = _MAKE_CHAN(19, 26, 29, 24),	/*!< Y and PF Memory to Post-filter to Y Memory */
	MEM_PF_U_MEM = _MAKE_CHAN(20, 27, 30, 25),	/*!< U and PF Memory to Post-filter to U Memory */
	MEM_PF_V_MEM = _MAKE_CHAN(21, 28, 31, 0xFF),	/*!< V Memory to Post-filter to V Memory */

	MEM_DC_SYNC = CHAN_NONE,
	DIRECT_ASYNC0 = CHAN_NONE,
	DIRECT_ASYNC1 = CHAN_NONE,
	MEM_VDI_PRP_VF_MEM_P = CHAN_NONE,
	MEM_VDI_PRP_VF_MEM = CHAN_NONE,
	MEM_VDI_PRP_VF_MEM_N = CHAN_NONE,
#else
	MEM_ROT_ENC_MEM = _MAKE_CHAN(1, 45, NO_DMA, NO_DMA, 48),
	MEM_ROT_VF_MEM = _MAKE_CHAN(2, 46, NO_DMA, NO_DMA, 49),
	MEM_ROT_PP_MEM = _MAKE_CHAN(3, 47, NO_DMA, NO_DMA, 50),

	MEM_PRP_ENC_MEM = _MAKE_CHAN(4, 12, 14, 17, 20),
	MEM_PRP_VF_MEM = _MAKE_CHAN(5, 12, 14, 17, 21),
	MEM_PP_MEM = _MAKE_CHAN(6, 11, 15, 18, 22),

	MEM_DC_SYNC = _MAKE_CHAN(7, 28, NO_DMA, NO_DMA, NO_DMA),
	MEM_DC_ASYNC = _MAKE_CHAN(8, 41, NO_DMA, NO_DMA, NO_DMA),
	MEM_BG_SYNC = _MAKE_CHAN(9, 23, NO_DMA, 51, NO_DMA),
	MEM_FG_SYNC = _MAKE_CHAN(10, 27, NO_DMA, 31, NO_DMA),

	MEM_BG_ASYNC0 = _MAKE_CHAN(11, 24, NO_DMA, 52, NO_DMA),
	MEM_FG_ASYNC0 = _MAKE_CHAN(12, 29, NO_DMA, 33, NO_DMA),
	MEM_BG_ASYNC1 = _MAKE_ALT_CHAN(MEM_BG_ASYNC0),
	MEM_FG_ASYNC1 = _MAKE_ALT_CHAN(MEM_FG_ASYNC0),

	DIRECT_ASYNC0 = _MAKE_CHAN(13, NO_DMA, NO_DMA, NO_DMA, NO_DMA),
	DIRECT_ASYNC1 = _MAKE_CHAN(14, NO_DMA, NO_DMA, NO_DMA, NO_DMA),

	CSI_MEM0 = _MAKE_CHAN(15, NO_DMA, NO_DMA, NO_DMA, 0),
	CSI_MEM1 = _MAKE_CHAN(16, NO_DMA, NO_DMA, NO_DMA, 1),
	CSI_MEM2 = _MAKE_CHAN(17, NO_DMA, NO_DMA, NO_DMA, 2),
	CSI_MEM3 = _MAKE_CHAN(18, NO_DMA, NO_DMA, NO_DMA, 3),

	CSI_MEM = CSI_MEM0,

	CSI_PRP_ENC_MEM = _MAKE_CHAN(19, NO_DMA, NO_DMA, NO_DMA, 20),
	CSI_PRP_VF_MEM = _MAKE_CHAN(20, NO_DMA, NO_DMA, NO_DMA, 21),

	MEM_VDI_PRP_VF_MEM_P = _MAKE_CHAN(21, 8, 14, 17, 21),
	MEM_VDI_PRP_VF_MEM = _MAKE_CHAN(22, 9, 14, 17, 21),
	MEM_VDI_PRP_VF_MEM_N = _MAKE_CHAN(23, 10, 14, 17, 21),

	MEM_PP_ADC = CHAN_NONE,
	ADC_SYS2 = CHAN_NONE,
#endif

} ipu_channel_t;

/*!
 * Enumeration of types of buffers for a logical channel.
 */
typedef enum {
	IPU_OUTPUT_BUFFER = 0,	/*!< Buffer for output from IPU */
	IPU_ALPHA_IN_BUFFER = 1,	/*!< Buffer for input to IPU */
	IPU_GRAPH_IN_BUFFER = 2,	/*!< Buffer for input to IPU */
	IPU_VIDEO_IN_BUFFER = 3,	/*!< Buffer for input to IPU */
	IPU_INPUT_BUFFER = IPU_VIDEO_IN_BUFFER,
	IPU_SEC_INPUT_BUFFER = IPU_GRAPH_IN_BUFFER,
} ipu_buffer_t;

#define IPU_PANEL_SERIAL		1
#define IPU_PANEL_PARALLEL		2

/*!
 * Enumeration of DI ports for ADC.
 */
typedef enum {
	DISP0,
	DISP1,
	DISP2,
	DISP3
} display_port_t;

/*!
 * Enumeration of ADC channel operation mode.
 */
typedef enum {
	Disable,
	WriteTemplateNonSeq,
	ReadTemplateNonSeq,
	WriteTemplateUnCon,
	ReadTemplateUnCon,
	WriteDataWithRS,
	WriteDataWoRS,
	WriteCmd
} mcu_mode_t;

/*!
 * Enumeration of ADC channel addressing mode.
 */
typedef enum {
	FullWoBE,
	FullWithBE,
	XY
} display_addressing_t;

/*!
 * Union of initialization parameters for a logical channel.
 */
typedef union {
	struct {
		uint32_t csi;
		uint32_t mipi_id;
		bool mipi_en;
		bool interlaced;
	} csi_mem;
	struct {
		uint32_t in_width;
		uint32_t in_height;
		uint32_t in_pixel_fmt;
		uint32_t out_width;
		uint32_t out_height;
		uint32_t out_pixel_fmt;
		uint32_t csi;
	} csi_prp_enc_mem;
	struct {
		uint32_t in_width;
		uint32_t in_height;
		uint32_t in_pixel_fmt;
		uint32_t out_width;
		uint32_t out_height;
		uint32_t out_pixel_fmt;
		uint32_t outh_resize_ratio;
		uint32_t outv_resize_ratio;
	} mem_prp_enc_mem;
	struct {
		uint32_t in_width;
		uint32_t in_height;
		uint32_t in_pixel_fmt;
		uint32_t out_width;
		uint32_t out_height;
		uint32_t out_pixel_fmt;
	} mem_rot_enc_mem;
	struct {
		uint32_t in_width;
		uint32_t in_height;
		uint32_t in_pixel_fmt;
		uint32_t out_width;
		uint32_t out_height;
		uint32_t out_pixel_fmt;
		bool graphics_combine_en;
		bool global_alpha_en;
		bool key_color_en;
		uint32_t csi;
	} csi_prp_vf_mem;
	struct {
		uint32_t in_width;
		uint32_t in_height;
		uint32_t in_pixel_fmt;
		uint32_t out_width;
		uint32_t out_height;
		uint32_t out_pixel_fmt;
		bool graphics_combine_en;
		bool global_alpha_en;
		bool key_color_en;
		display_port_t disp;
		uint32_t out_left;
		uint32_t out_top;
	} csi_prp_vf_adc;
	struct {
		uint32_t in_width;
		uint32_t in_height;
		uint32_t in_pixel_fmt;
		uint32_t out_width;
		uint32_t out_height;
		uint32_t out_pixel_fmt;
		uint32_t outh_resize_ratio;
		uint32_t outv_resize_ratio;
		bool graphics_combine_en;
		bool global_alpha_en;
		bool key_color_en;
		uint32_t in_g_pixel_fmt;
		uint8_t alpha;
		uint32_t key_color;
		bool alpha_chan_en;
		ipu_motion_sel motion_sel;
		enum v4l2_field field_fmt;
	} mem_prp_vf_mem;
	struct {
		uint32_t temp;
	} mem_prp_vf_adc;
	struct {
		uint32_t temp;
	} mem_rot_vf_mem;
	struct {
		uint32_t in_width;
		uint32_t in_height;
		uint32_t in_pixel_fmt;
		uint32_t out_width;
		uint32_t out_height;
		uint32_t out_pixel_fmt;
		uint32_t outh_resize_ratio;
		uint32_t outv_resize_ratio;
		bool graphics_combine_en;
		bool global_alpha_en;
		bool key_color_en;
		uint32_t in_g_pixel_fmt;
		uint8_t alpha;
		uint32_t key_color;
		bool alpha_chan_en;
	} mem_pp_mem;
	struct {
		uint32_t temp;
	} mem_rot_mem;
	struct {
		uint32_t in_width;
		uint32_t in_height;
		uint32_t in_pixel_fmt;
		uint32_t out_width;
		uint32_t out_height;
		uint32_t out_pixel_fmt;
		bool graphics_combine_en;
		bool global_alpha_en;
		bool key_color_en;
		display_port_t disp;
		uint32_t out_left;
		uint32_t out_top;
	} mem_pp_adc;
	struct {
		pf_operation_t operation;
	} mem_pf_mem;
	struct {
		uint32_t di;
		bool interlaced;
		uint32_t in_pixel_fmt;
		uint32_t out_pixel_fmt;
	} mem_dc_sync;
	struct {
		uint32_t temp;
	} mem_sdc_fg;
	struct {
		uint32_t di;
		bool interlaced;
		uint32_t in_pixel_fmt;
		uint32_t out_pixel_fmt;
		bool alpha_chan_en;
	} mem_dp_bg_sync;
	struct {
		uint32_t temp;
	} mem_sdc_bg;
	struct {
		uint32_t di;
		bool interlaced;
		uint32_t in_pixel_fmt;
		uint32_t out_pixel_fmt;
		bool alpha_chan_en;
	} mem_dp_fg_sync;
	struct {
		uint32_t di;
	} direct_async;
	struct {
		display_port_t disp;
		mcu_mode_t ch_mode;
		uint32_t out_left;
		uint32_t out_top;
	} adc_sys1;
	struct {
		display_port_t disp;
		mcu_mode_t ch_mode;
		uint32_t out_left;
		uint32_t out_top;
	} adc_sys2;
} ipu_channel_params_t;

/*!
 * Enumeration of IPU interrupt sources.
 */
enum ipu_irq_line {
#ifdef CONFIG_MXC_IPU_V1
	IPU_IRQ_DC_FC_1 = -1,

	IPU_IRQ_PRP_ENC_OUT_EOF = 0,
	IPU_IRQ_PRP_VF_OUT_EOF = 1,
	IPU_IRQ_PP_OUT_EOF = 2,
	IPU_IRQ_PRP_GRAPH_IN_EOF = 3,
	IPU_IRQ_PP_GRAPH_IN_EOF = 4,
	IPU_IRQ_PP_IN_EOF = 5,
	IPU_IRQ_PRP_IN_EOF = 6,
	IPU_IRQ_SENSOR_OUT_EOF = 7,
	IPU_IRQ_CSI0_OUT_EOF = IPU_IRQ_SENSOR_OUT_EOF,
	IPU_IRQ_PRP_ENC_ROT_OUT_EOF = 8,
	IPU_IRQ_PRP_VF_ROT_OUT_EOF = 9,
	IPU_IRQ_PRP_ENC_ROT_IN_EOF = 10,
	IPU_IRQ_PRP_VF_ROT_IN_EOF = 11,
	IPU_IRQ_PP_ROT_OUT_EOF = 12,
	IPU_IRQ_PP_ROT_IN_EOF = 13,
	IPU_IRQ_BG_SYNC_EOF = 14,
	IPU_IRQ_SDC_BG_EOF = IPU_IRQ_BG_SYNC_EOF,
	IPU_IRQ_FG_SYNC_EOF = 15,
	IPU_IRQ_SDC_FG_EOF = IPU_IRQ_FG_SYNC_EOF,
	IPU_IRQ_SDC_MASK_EOF = 16,
	IPU_IRQ_SDC_BG_PART_EOF = 17,
	IPU_IRQ_ADC_SYS1_WR_EOF = 18,
	IPU_IRQ_ADC_SYS2_WR_EOF = 19,
	IPU_IRQ_ADC_SYS1_CMD_EOF = 20,
	IPU_IRQ_ADC_SYS2_CMD_EOF = 21,
	IPU_IRQ_ADC_SYS1_RD_EOF = 22,
	IPU_IRQ_ADC_SYS2_RD_EOF = 23,
	IPU_IRQ_PF_QP_IN_EOF = 24,
	IPU_IRQ_PF_BSP_IN_EOF = 25,
	IPU_IRQ_PF_Y_IN_EOF = 26,
	IPU_IRQ_PF_U_IN_EOF = 27,
	IPU_IRQ_PF_V_IN_EOF = 28,
	IPU_IRQ_PF_Y_OUT_EOF = 29,
	IPU_IRQ_PF_U_OUT_EOF = 30,
	IPU_IRQ_PF_V_OUT_EOF = 31,

	IPU_IRQ_PRP_ENC_OUT_NF = 32,
	IPU_IRQ_PRP_VF_OUT_NF = 33,
	IPU_IRQ_PP_OUT_NF = 34,
	IPU_IRQ_PRP_GRAPH_IN_NF = 35,
	IPU_IRQ_PP_GRAPH_IN_NF = 36,
	IPU_IRQ_PP_IN_NF = 37,
	IPU_IRQ_PRP_IN_NF = 38,
	IPU_IRQ_SENSOR_OUT_NF = 39,
	IPU_IRQ_PRP_ENC_ROT_OUT_NF = 40,
	IPU_IRQ_PRP_VF_ROT_OUT_NF = 41,
	IPU_IRQ_PRP_ENC_ROT_IN_NF = 42,
	IPU_IRQ_PRP_VF_ROT_IN_NF = 43,
	IPU_IRQ_PP_ROT_OUT_NF = 44,
	IPU_IRQ_PP_ROT_IN_NF = 45,
	IPU_IRQ_SDC_FG_NF = 46,
	IPU_IRQ_SDC_BG_NF = 47,
	IPU_IRQ_SDC_MASK_NF = 48,
	IPU_IRQ_SDC_BG_PART_NF = 49,
	IPU_IRQ_ADC_SYS1_WR_NF = 50,
	IPU_IRQ_ADC_SYS2_WR_NF = 51,
	IPU_IRQ_ADC_SYS1_CMD_NF = 52,
	IPU_IRQ_ADC_SYS2_CMD_NF = 53,
	IPU_IRQ_ADC_SYS1_RD_NF = 54,
	IPU_IRQ_ADC_SYS2_RD_NF = 55,
	IPU_IRQ_PF_QP_IN_NF = 56,
	IPU_IRQ_PF_BSP_IN_NF = 57,
	IPU_IRQ_PF_Y_IN_NF = 58,
	IPU_IRQ_PF_U_IN_NF = 59,
	IPU_IRQ_PF_V_IN_NF = 60,
	IPU_IRQ_PF_Y_OUT_NF = 61,
	IPU_IRQ_PF_U_OUT_NF = 62,
	IPU_IRQ_PF_V_OUT_NF = 63,

	IPU_IRQ_BREAKRQ = 64,
	IPU_IRQ_SDC_BG_OUT_EOF = 65,
	IPU_IRQ_BG_SF_END = IPU_IRQ_SDC_BG_OUT_EOF,
	IPU_IRQ_SDC_FG_OUT_EOF = 66,
	IPU_IRQ_SDC_MASK_OUT_EOF = 67,
	IPU_IRQ_ADC_SERIAL_DATA_OUT = 68,
	IPU_IRQ_SENSOR_NF = 69,
	IPU_IRQ_SENSOR_EOF = 70,
	IPU_IRQ_SDC_DISP3_VSYNC = 80,
	IPU_IRQ_ADC_DISP0_VSYNC = 81,
	IPU_IRQ_ADC_DISP12_VSYNC = 82,
	IPU_IRQ_ADC_PRP_EOF = 83,
	IPU_IRQ_ADC_PP_EOF = 84,
	IPU_IRQ_ADC_SYS1_EOF = 85,
	IPU_IRQ_ADC_SYS2_EOF = 86,

	IPU_IRQ_PRP_ENC_OUT_NFB4EOF_ERR = 96,
	IPU_IRQ_PRP_VF_OUT_NFB4EOF_ERR = 97,
	IPU_IRQ_PP_OUT_NFB4EOF_ERR = 98,
	IPU_IRQ_PRP_GRAPH_IN_NFB4EOF_ERR = 99,
	IPU_IRQ_PP_GRAPH_IN_NFB4EOF_ERR = 100,
	IPU_IRQ_PP_IN_NFB4EOF_ERR = 101,
	IPU_IRQ_PRP_IN_NFB4EOF_ERR = 102,
	IPU_IRQ_SENSOR_OUT_NFB4EOF_ERR = 103,
	IPU_IRQ_PRP_ENC_ROT_OUT_NFB4EOF_ERR = 104,
	IPU_IRQ_PRP_VF_ROT_OUT_NFB4EOF_ERR = 105,
	IPU_IRQ_PRP_ENC_ROT_IN_NFB4EOF_ERR = 106,
	IPU_IRQ_PRP_VF_ROT_IN_NFB4EOF_ERR = 107,
	IPU_IRQ_PP_ROT_OUT_NFB4EOF_ERR = 108,
	IPU_IRQ_PP_ROT_IN_NFB4EOF_ERR = 109,
	IPU_IRQ_SDC_FG_NFB4EOF_ERR = 110,
	IPU_IRQ_SDC_BG_NFB4EOF_ERR = 111,
	IPU_IRQ_SDC_MASK_NFB4EOF_ERR = 112,
	IPU_IRQ_SDC_BG_PART_NFB4EOF_ERR = 113,
	IPU_IRQ_ADC_SYS1_WR_NFB4EOF_ERR = 114,
	IPU_IRQ_ADC_SYS2_WR_NFB4EOF_ERR = 115,
	IPU_IRQ_ADC_SYS1_CMD_NFB4EOF_ERR = 116,
	IPU_IRQ_ADC_SYS2_CMD_NFB4EOF_ERR = 117,
	IPU_IRQ_ADC_SYS1_RD_NFB4EOF_ERR = 118,
	IPU_IRQ_ADC_SYS2_RD_NFB4EOF_ERR = 119,
	IPU_IRQ_PF_QP_IN_NFB4EOF_ERR = 120,
	IPU_IRQ_PF_BSP_IN_NFB4EOF_ERR = 121,
	IPU_IRQ_PF_Y_IN_NFB4EOF_ERR = 122,
	IPU_IRQ_PF_U_IN_NFB4EOF_ERR = 123,
	IPU_IRQ_PF_V_IN_NFB4EOF_ERR = 124,
	IPU_IRQ_PF_Y_OUT_NFB4EOF_ERR = 125,
	IPU_IRQ_PF_U_OUT_NFB4EOF_ERR = 126,
	IPU_IRQ_PF_V_OUT_NFB4EOF_ERR = 127,

	IPU_IRQ_BAYER_BUFOVF_ERR = 128,
	IPU_IRQ_ENC_BUFOVF_ERR = 129,
	IPU_IRQ_VF_BUFOVF_ERR = 130,
	IPU_IRQ_ADC_PP_TEAR_ERR = 131,
	IPU_IRQ_ADC_SYS1_TEAR_ERR = 132,
	IPU_IRQ_ADC_SYS2_TEAR_ERR = 133,
	IPU_IRQ_SDC_BGD_ERR = 134,
	IPU_IRQ_SDC_FGD_ERR = 135,
	IPU_IRQ_SDC_MASKD_ERR = 136,
	IPU_IRQ_BAYER_FRM_LOST_ERR = 137,
	IPU_IRQ_ENC_FRM_LOST_ERR = 138,
	IPU_IRQ_VF_FRM_LOST_ERR = 139,
	IPU_IRQ_ADC_LOCK_ERR = 140,
	IPU_IRQ_DI_LLA_LOCK_ERR = 141,
	IPU_IRQ_AHB_M1_ERR = 142,
	IPU_IRQ_AHB_M12_ERR = 143,
#else
	IPU_IRQ_CSI0_OUT_EOF = 0,
	IPU_IRQ_CSI1_OUT_EOF = 1,
	IPU_IRQ_CSI2_OUT_EOF = 2,
	IPU_IRQ_CSI3_OUT_EOF = 3,
	IPU_IRQ_VDI_P_IN_EOF = 8,
	IPU_IRQ_VDI_C_IN_EOF = 9,
	IPU_IRQ_VDI_N_IN_EOF = 10,
	IPU_IRQ_PP_IN_EOF = 11,
	IPU_IRQ_PRP_IN_EOF = 12,
	IPU_IRQ_PRP_GRAPH_IN_EOF = 14,
	IPU_IRQ_PP_GRAPH_IN_EOF = 15,
	IPU_IRQ_PRP_ALPHA_IN_EOF = 17,
	IPU_IRQ_PP_ALPHA_IN_EOF = 18,
	IPU_IRQ_PRP_ENC_OUT_EOF = 20,
	IPU_IRQ_PRP_VF_OUT_EOF = 21,
	IPU_IRQ_PP_OUT_EOF = 22,
	IPU_IRQ_BG_SYNC_EOF = 23,
	IPU_IRQ_BG_ASYNC_EOF = 24,
	IPU_IRQ_FG_SYNC_EOF = 27,
	IPU_IRQ_DC_SYNC_EOF = 28,
	IPU_IRQ_FG_ASYNC_EOF = 29,
	IPU_IRQ_FG_ALPHA_SYNC_EOF = 31,

	IPU_IRQ_FG_ALPHA_ASYNC_EOF = 33,
	IPU_IRQ_DC_READ_EOF = 40,
	IPU_IRQ_DC_ASYNC_EOF = 41,
	IPU_IRQ_DC_CMD1_EOF = 42,
	IPU_IRQ_DC_CMD2_EOF = 43,
	IPU_IRQ_DC_MASK_EOF = 44,
	IPU_IRQ_PRP_ENC_ROT_IN_EOF = 45,
	IPU_IRQ_PRP_VF_ROT_IN_EOF = 46,
	IPU_IRQ_PP_ROT_IN_EOF = 47,
	IPU_IRQ_PRP_ENC_ROT_OUT_EOF = 48,
	IPU_IRQ_PRP_VF_ROT_OUT_EOF = 49,
	IPU_IRQ_PP_ROT_OUT_EOF = 50,
	IPU_IRQ_BG_ALPHA_SYNC_EOF = 51,
	IPU_IRQ_BG_ALPHA_ASYNC_EOF = 52,

	IPU_IRQ_DP_SF_START = 448 + 2,
	IPU_IRQ_DP_SF_END = 448 + 3,
	IPU_IRQ_BG_SF_END = IPU_IRQ_DP_SF_END,
	IPU_IRQ_DC_FC_0 = 448 + 8,
	IPU_IRQ_DC_FC_1 = 448 + 9,
	IPU_IRQ_DC_FC_2 = 448 + 10,
	IPU_IRQ_DC_FC_3 = 448 + 11,
	IPU_IRQ_DC_FC_4 = 448 + 12,
	IPU_IRQ_DC_FC_6 = 448 + 13,
	IPU_IRQ_VSYNC_PRE_0 = 448 + 14,
	IPU_IRQ_VSYNC_PRE_1 = 448 + 15,
#endif

	IPU_IRQ_COUNT
};

/*!
 * Bitfield of Display Interface signal polarities.
 */
typedef struct {
	unsigned datamask_en:1;
	unsigned int_clk:1;
	unsigned interlaced:1;
	unsigned odd_field_first:1;
	unsigned clksel_en:1;
	unsigned clkidle_en:1;
	unsigned data_pol:1;	/* true = inverted */
	unsigned clk_pol:1;	/* true = rising edge */
	unsigned enable_pol:1;
	unsigned Hsync_pol:1;	/* true = active high */
	unsigned Vsync_pol:1;
} ipu_di_signal_cfg_t;

/*!
 * Bitfield of CSI signal polarities and modes.
 */

typedef struct {
	unsigned data_width:4;
	unsigned clk_mode:3;
	unsigned ext_vsync:1;
	unsigned Vsync_pol:1;
	unsigned Hsync_pol:1;
	unsigned pixclk_pol:1;
	unsigned data_pol:1;
	unsigned sens_clksrc:1;
	unsigned pack_tight:1;
	unsigned force_eof:1;
	unsigned data_en_pol:1;
	unsigned data_fmt;
	unsigned csi;
	unsigned mclk;
} ipu_csi_signal_cfg_t;

/*!
 * Enumeration of CSI data bus widths.
 */
enum {
	IPU_CSI_DATA_WIDTH_4,
	IPU_CSI_DATA_WIDTH_8,
	IPU_CSI_DATA_WIDTH_10,
	IPU_CSI_DATA_WIDTH_16,
};

/*!
 * Enumeration of CSI clock modes.
 */
enum {
	IPU_CSI_CLK_MODE_GATED_CLK,
	IPU_CSI_CLK_MODE_NONGATED_CLK,
	IPU_CSI_CLK_MODE_CCIR656_PROGRESSIVE,
	IPU_CSI_CLK_MODE_CCIR656_INTERLACED,
	IPU_CSI_CLK_MODE_CCIR1120_PROGRESSIVE_DDR,
	IPU_CSI_CLK_MODE_CCIR1120_PROGRESSIVE_SDR,
	IPU_CSI_CLK_MODE_CCIR1120_INTERLACED_DDR,
	IPU_CSI_CLK_MODE_CCIR1120_INTERLACED_SDR,
};

enum {
	IPU_CSI_MIPI_DI0,
	IPU_CSI_MIPI_DI1,
	IPU_CSI_MIPI_DI2,
	IPU_CSI_MIPI_DI3,
};

typedef enum {
	RGB,
	YCbCr,
	YUV
} ipu_color_space_t;

/*!
 * Enumeration of ADC vertical sync mode.
 */
typedef enum {
	VsyncNone,
	VsyncInternal,
	VsyncCSI,
	VsyncExternal
} vsync_t;

typedef enum {
	DAT,
	CMD
} cmddata_t;

/*!
 * Enumeration of ADC display update mode.
 */
typedef enum {
	IPU_ADC_REFRESH_NONE,
	IPU_ADC_AUTO_REFRESH,
	IPU_ADC_AUTO_REFRESH_SNOOP,
	IPU_ADC_SNOOPING,
} ipu_adc_update_mode_t;

/*!
 * Enumeration of ADC display interface types (serial or parallel).
 */
enum {
	IPU_ADC_IFC_MODE_SYS80_TYPE1,
	IPU_ADC_IFC_MODE_SYS80_TYPE2,
	IPU_ADC_IFC_MODE_SYS68K_TYPE1,
	IPU_ADC_IFC_MODE_SYS68K_TYPE2,
	IPU_ADC_IFC_MODE_3WIRE_SERIAL,
	IPU_ADC_IFC_MODE_4WIRE_SERIAL,
	IPU_ADC_IFC_MODE_5WIRE_SERIAL_CLK,
	IPU_ADC_IFC_MODE_5WIRE_SERIAL_CS,
};

enum {
	IPU_ADC_IFC_WIDTH_8,
	IPU_ADC_IFC_WIDTH_16,
};

/*!
 * Enumeration of ADC display interface burst mode.
 */
enum {
	IPU_ADC_BURST_WCS,
	IPU_ADC_BURST_WBLCK,
	IPU_ADC_BURST_NONE,
	IPU_ADC_BURST_SERIAL,
};

/*!
 * Enumeration of ADC display interface RW signal timing modes.
 */
enum {
	IPU_ADC_SER_NO_RW,
	IPU_ADC_SER_RW_BEFORE_RS,
	IPU_ADC_SER_RW_AFTER_RS,
};

/*!
 * Bitfield of ADC signal polarities and modes.
 */
typedef struct {
	unsigned data_pol:1;
	unsigned clk_pol:1;
	unsigned cs_pol:1;
	unsigned rs_pol:1;
	unsigned addr_pol:1;
	unsigned read_pol:1;
	unsigned write_pol:1;
	unsigned Vsync_pol:1;
	unsigned burst_pol:1;
	unsigned burst_mode:2;
	unsigned ifc_mode:3;
	unsigned ifc_width:5;
	unsigned ser_preamble_len:4;
	unsigned ser_preamble:8;
	unsigned ser_rw_mode:2;
} ipu_adc_sig_cfg_t;

/*!
 * Enumeration of ADC template commands.
 */
enum {
	RD_DATA,
	RD_ACK,
	RD_WAIT,
	WR_XADDR,
	WR_YADDR,
	WR_ADDR,
	WR_CMND,
	WR_DATA,
};

/*!
 * Enumeration of ADC template command flow control.
 */
enum {
	SINGLE_STEP,
	PAUSE,
	STOP,
};


/*Define template constants*/
#define     ATM_ADDR_RANGE      0x20	/*offset address of DISP */
#define     TEMPLATE_BUF_SIZE   0x20	/*size of template */

/*!
 * Define to create ADC template command entry.
 */
#define ipu_adc_template_gen(oc, rs, fc, dat) (((rs) << 29) | ((fc) << 27) | \
			((oc) << 24) | (dat))

typedef struct {
	u32 reg;
	u32 value;
} ipu_lpmc_reg_t;

#define IPU_LPMC_REG_READ       0x80000000L

#define CSI_MCLK_VF  1
#define CSI_MCLK_ENC 2
#define CSI_MCLK_RAW 4
#define CSI_MCLK_I2C 8

/* Common IPU API */
int32_t ipu_init_channel(ipu_channel_t channel, ipu_channel_params_t *params);
void ipu_uninit_channel(ipu_channel_t channel);

static inline bool ipu_can_rotate_in_place(ipu_rotate_mode_t rot)
{
#ifdef CONFIG_MXC_IPU_V3D
	return (rot < IPU_ROTATE_HORIZ_FLIP);
#else
	return (rot < IPU_ROTATE_90_RIGHT);
#endif
}

int32_t ipu_init_channel_buffer(ipu_channel_t channel, ipu_buffer_t type,
				uint32_t pixel_fmt,
				uint16_t width, uint16_t height,
				uint32_t stride,
				ipu_rotate_mode_t rot_mode,
				dma_addr_t phyaddr_0, dma_addr_t phyaddr_1,
				dma_addr_t phyaddr_2,
				uint32_t u_offset, uint32_t v_offset);

int32_t ipu_update_channel_buffer(ipu_channel_t channel, ipu_buffer_t type,
				  uint32_t bufNum, dma_addr_t phyaddr);

int32_t ipu_update_channel_offset(ipu_channel_t channel, ipu_buffer_t type,
				uint32_t pixel_fmt,
				uint16_t width, uint16_t height,
				uint32_t stride,
				uint32_t u, uint32_t v,
				uint32_t vertical_offset, uint32_t horizontal_offset);

int32_t ipu_select_buffer(ipu_channel_t channel,
			  ipu_buffer_t type, uint32_t bufNum);
int32_t ipu_select_multi_vdi_buffer(uint32_t bufNum);

int32_t ipu_link_channels(ipu_channel_t src_ch, ipu_channel_t dest_ch);
int32_t ipu_unlink_channels(ipu_channel_t src_ch, ipu_channel_t dest_ch);

int32_t ipu_is_channel_busy(ipu_channel_t channel);
int32_t ipu_check_buffer_ready(ipu_channel_t channel, ipu_buffer_t type,
		uint32_t bufNum);
void ipu_clear_buffer_ready(ipu_channel_t channel, ipu_buffer_t type,
		uint32_t bufNum);
uint32_t ipu_get_cur_buffer_idx(ipu_channel_t channel, ipu_buffer_t type);
int32_t ipu_enable_channel(ipu_channel_t channel);
int32_t ipu_disable_channel(ipu_channel_t channel, bool wait_for_stop);
int32_t ipu_swap_channel(ipu_channel_t from_ch, ipu_channel_t to_ch);

int32_t ipu_enable_csi(uint32_t csi);
int32_t ipu_disable_csi(uint32_t csi);

int ipu_lowpwr_display_enable(void);
int ipu_lowpwr_display_disable(void);

void ipu_enable_irq(uint32_t irq);
void ipu_disable_irq(uint32_t irq);
void ipu_clear_irq(uint32_t irq);
int ipu_request_irq(uint32_t irq,
		    irqreturn_t(*handler) (int, void *),
		    uint32_t irq_flags, const char *devname, void *dev_id);
void ipu_free_irq(uint32_t irq, void *dev_id);
bool ipu_get_irq_status(uint32_t irq);
void ipu_set_csc_coefficients(ipu_channel_t channel, int32_t param[][3]);

/* SDC API */
int32_t ipu_sdc_init_panel(ipu_panel_t panel,
			   uint32_t pixel_clk,
			   uint16_t width, uint16_t height,
			   uint32_t pixel_fmt,
			   uint16_t hStartWidth, uint16_t hSyncWidth,
			   uint16_t hEndWidth, uint16_t vStartWidth,
			   uint16_t vSyncWidth, uint16_t vEndWidth,
			   ipu_di_signal_cfg_t sig);

int32_t ipu_sdc_set_global_alpha(bool enable, uint8_t alpha);
int32_t ipu_sdc_set_color_key(ipu_channel_t channel, bool enable,
			      uint32_t colorKey);
int32_t ipu_sdc_set_brightness(uint8_t value);

int32_t ipu_init_sync_panel(int disp,
			    uint32_t pixel_clk,
			    uint16_t width, uint16_t height,
			    uint32_t pixel_fmt,
			    uint16_t h_start_width, uint16_t h_sync_width,
			    uint16_t h_end_width, uint16_t v_start_width,
			    uint16_t v_sync_width, uint16_t v_end_width,
			    uint32_t v_to_h_sync, ipu_di_signal_cfg_t sig);

void ipu_uninit_sync_panel(int disp);

int32_t ipu_disp_set_window_pos(ipu_channel_t channel, int16_t x_pos,
				int16_t y_pos);
int32_t ipu_disp_get_window_pos(ipu_channel_t channel, int16_t *x_pos,
				int16_t *y_pos);
int32_t ipu_disp_set_global_alpha(ipu_channel_t channel, bool enable,
				  uint8_t alpha);
int32_t ipu_disp_set_color_key(ipu_channel_t channel, bool enable,
			       uint32_t colorKey);
int32_t ipu_disp_set_gamma_correction(ipu_channel_t channel, bool enable,
				int constk[], int slopek[]);

int ipu_init_async_panel(int disp, int type, uint32_t cycle_time,
			 uint32_t pixel_fmt, ipu_adc_sig_cfg_t sig);
void ipu_disp_direct_write(ipu_channel_t channel, u32 value, u32 offset);
void ipu_reset_disp_panel(void);

/* ADC API */
int32_t ipu_adc_write_template(display_port_t disp, uint32_t *pCmd,
			       bool write);

int32_t ipu_adc_set_update_mode(ipu_channel_t channel,
				ipu_adc_update_mode_t mode,
				uint32_t refresh_rate, unsigned long addr,
				uint32_t *size);

int32_t ipu_adc_get_snooping_status(uint32_t *statl, uint32_t *stath);

int32_t ipu_adc_write_cmd(display_port_t disp, cmddata_t type,
			  uint32_t cmd, const uint32_t *params,
			  uint16_t numParams);

int32_t ipu_adc_init_panel(display_port_t disp,
			   uint16_t width, uint16_t height,
			   uint32_t pixel_fmt,
			   uint32_t stride,
			   ipu_adc_sig_cfg_t sig,
			   display_addressing_t addr,
			   uint32_t vsync_width, vsync_t mode);

int32_t ipu_adc_init_ifc_timing(display_port_t disp, bool read,
				uint32_t cycle_time,
				uint32_t up_time,
				uint32_t down_time,
				uint32_t read_latch_time, uint32_t pixel_clk);

/* CMOS Sensor Interface API */
int32_t ipu_csi_init_interface(uint16_t width, uint16_t height,
			       uint32_t pixel_fmt, ipu_csi_signal_cfg_t sig);

int32_t ipu_csi_get_sensor_protocol(uint32_t csi);

int32_t ipu_csi_enable_mclk(int src, bool flag, bool wait);

static inline int32_t ipu_csi_enable_mclk_if(int src, uint32_t csi,
		bool flag, bool wait)
{
#ifdef CONFIG_MXC_IPU_V1
	return ipu_csi_enable_mclk(src, flag, wait);
#else
	return ipu_csi_enable_mclk(csi, flag, wait);
#endif
}

int ipu_csi_read_mclk_flag(void);

void ipu_csi_flash_strobe(bool flag);

void ipu_csi_get_window_size(uint32_t *width, uint32_t *height, uint32_t csi);

void ipu_csi_set_window_size(uint32_t width, uint32_t height, uint32_t csi);

void ipu_csi_set_window_pos(uint32_t left, uint32_t top, uint32_t csi);

/* Post Filter functions */
int32_t ipu_pf_set_pause_row(uint32_t pause_row);

uint32_t bytes_per_pixel(uint32_t fmt);

/* New added for IPU-lib functionality*/
int ipu_open(void);
int ipu_register_generic_isr(int irq, void *dev);
void ipu_close(void);

/* two stripe calculations */
struct stripe_param{
	unsigned int input_width; /* width of the input stripe */
	unsigned int output_width; /* width of the output stripe */
	unsigned int input_column; /* the first column on the input stripe */
	unsigned int output_column; /* the first column on the output stripe */
	unsigned int idr;
	/* inverse downisizing ratio parameter; expressed as a power of 2 */
	unsigned int irr;
	/* inverse resizing ratio parameter; expressed as a multiple of 2^-13 */
};

typedef struct _ipu_stripe_parm {
	unsigned int input_width;
	unsigned int output_width;
	unsigned int maximal_stripe_width;
	unsigned long long cirr;
	unsigned int equal_stripes;
	u32 input_pixelformat;
	u32 output_pixelformat;
	struct stripe_param left;
	struct stripe_param right;
} ipu_stripe_parm;

typedef struct _ipu_channel_parm {
	ipu_channel_t channel;
	ipu_channel_params_t params;
	bool flag;
} ipu_channel_parm;

typedef struct _ipu_channel_buf_parm {
	ipu_channel_t channel;
	ipu_buffer_t type;
	uint32_t pixel_fmt;
	uint16_t width;
	uint16_t height;
	uint16_t stride;
	ipu_rotate_mode_t rot_mode;
	dma_addr_t phyaddr_0;
	dma_addr_t phyaddr_1;
	dma_addr_t phyaddr_2;
	uint32_t u_offset;
	uint32_t v_offset;
	uint32_t bufNum;
} ipu_channel_buf_parm;

typedef struct _ipu_buf_offset_parm {
	ipu_channel_t channel;
	ipu_buffer_t type;
	uint32_t pixel_fmt;
	uint16_t width;
	uint16_t height;
	uint16_t stride;
	uint32_t u_offset;
	uint32_t v_offset;
	uint32_t vertical_offset;
	uint32_t horizontal_offset;
} ipu_buf_offset_parm;

typedef struct _ipu_channel_link {
	ipu_channel_t src_ch;
	ipu_channel_t dest_ch;
} ipu_channel_link;

typedef struct _ipu_channel_info {
	ipu_channel_t channel;
	bool stop;
} ipu_channel_info;

typedef struct ipu_irq_info {
	uint32_t irq;
	 irqreturn_t(*handler) (int, void *);
	uint32_t irq_flags;
	char *devname;
	void *dev_id;
} ipu_irq_info;

typedef struct _ipu_sdc_panel_info {
	ipu_panel_t panel;
	uint32_t pixel_clk;
	uint16_t width;
	uint16_t height;
	uint32_t pixel_fmt;
	uint16_t hStartWidth;
	uint16_t hSyncWidth;
	uint16_t hEndWidth;
	uint16_t vStartWidth;
	uint16_t vSyncWidth;
	uint16_t vEndWidth;
	ipu_di_signal_cfg_t signal;
} ipu_sdc_panel_info;

typedef struct _ipu_sdc_window_pos {
	ipu_channel_t channel;
	int16_t x_pos;
	int16_t y_pos;
} ipu_sdc_window_pos;

typedef struct _ipu_sdc_global_alpha {
	bool enable;
	uint8_t alpha;
} ipu_sdc_global_alpha;

typedef struct _ipu_sdc_color_key {
	ipu_channel_t channel;
	bool enable;
	uint32_t colorKey;
} ipu_sdc_color_key;

typedef struct _ipu_adc_template {
	display_port_t disp;
	uint32_t *pCmd;
	bool write;
} ipu_adc_template;

typedef struct _ipu_adc_update {
	ipu_channel_t channel;
	ipu_adc_update_mode_t mode;
	uint32_t refresh_rate;
	unsigned long addr;
	uint32_t *size;
} ipu_adc_update;

typedef struct _ipu_adc_snoop {
	uint32_t *statl;
	uint32_t *stath;
} ipu_adc_snoop;

typedef struct _ipu_adc_cmd {
	display_port_t disp;
	cmddata_t type;
	uint32_t cmd;
	uint32_t *params;
	uint16_t numParams;
} ipu_adc_cmd;

typedef struct _ipu_adc_panel {
	display_port_t disp;
	uint16_t width;
	uint16_t height;
	uint32_t pixel_fmt;
	uint32_t stride;
	ipu_adc_sig_cfg_t signal;
	display_addressing_t addr;
	uint32_t vsync_width;
	vsync_t mode;
} ipu_adc_panel;

typedef struct _ipu_adc_ifc_timing {
	display_port_t disp;
	bool read;
	uint32_t cycle_time;
	uint32_t up_time;
	uint32_t down_time;
	uint32_t read_latch_time;
	uint32_t pixel_clk;
} ipu_adc_ifc_timing;

typedef struct _ipu_csi_interface {
	uint16_t width;
	uint16_t height;
	uint16_t pixel_fmt;
	ipu_csi_signal_cfg_t signal;
} ipu_csi_interface;

typedef struct _ipu_csi_mclk {
	int src;
	bool flag;
	bool wait;
} ipu_csi_mclk;

typedef struct _ipu_csi_window {
	uint32_t left;
	uint32_t top;
} ipu_csi_window;

typedef struct _ipu_csi_window_size {
	uint32_t width;
	uint32_t height;
} ipu_csi_window_size;

typedef struct _ipu_event_info {
	int irq;
	void *dev;
} ipu_event_info;

typedef struct _ipu_mem_info {
	dma_addr_t paddr;
	void *vaddr;
	int size;
} ipu_mem_info;

typedef struct _ipu_csc_update {
	ipu_channel_t channel;
	int **param;
} ipu_csc_update;

/* IOCTL commands */

#define IPU_INIT_CHANNEL              _IOW('I', 0x1, ipu_channel_parm)
#define IPU_UNINIT_CHANNEL            _IOW('I', 0x2, ipu_channel_t)
#define IPU_INIT_CHANNEL_BUFFER       _IOW('I', 0x3, ipu_channel_buf_parm)
#define IPU_UPDATE_CHANNEL_BUFFER     _IOW('I', 0x4, ipu_channel_buf_parm)
#define IPU_SELECT_CHANNEL_BUFFER     _IOW('I', 0x5, ipu_channel_buf_parm)
#define IPU_LINK_CHANNELS             _IOW('I', 0x6, ipu_channel_link)
#define IPU_UNLINK_CHANNELS           _IOW('I', 0x7, ipu_channel_link)
#define IPU_ENABLE_CHANNEL            _IOW('I', 0x8, ipu_channel_t)
#define IPU_DISABLE_CHANNEL           _IOW('I', 0x9, ipu_channel_info)
#define IPU_ENABLE_IRQ                _IOW('I', 0xA, int)
#define IPU_DISABLE_IRQ               _IOW('I', 0xB, int)
#define IPU_CLEAR_IRQ                 _IOW('I', 0xC, int)
#define IPU_FREE_IRQ                  _IOW('I', 0xD, ipu_irq_info)
#define IPU_REQUEST_IRQ_STATUS        _IOW('I', 0xE, int)
#define IPU_SDC_INIT_PANEL            _IOW('I', 0xF, ipu_sdc_panel_info)
#define IPU_SDC_SET_WIN_POS           _IOW('I', 0x10, ipu_sdc_window_pos)
#define IPU_SDC_SET_GLOBAL_ALPHA      _IOW('I', 0x11, ipu_sdc_global_alpha)
#define IPU_SDC_SET_COLOR_KEY         _IOW('I', 0x12, ipu_sdc_color_key)
#define IPU_SDC_SET_BRIGHTNESS        _IOW('I', 0x13, int)
#define IPU_ADC_WRITE_TEMPLATE        _IOW('I', 0x14, ipu_adc_template)
#define IPU_ADC_UPDATE                _IOW('I', 0x15, ipu_adc_update)
#define IPU_ADC_SNOOP                 _IOW('I', 0x16, ipu_adc_snoop)
#define IPU_ADC_CMD                   _IOW('I', 0x17, ipu_adc_cmd)
#define IPU_ADC_INIT_PANEL            _IOW('I', 0x18, ipu_adc_panel)
#define IPU_ADC_IFC_TIMING            _IOW('I', 0x19, ipu_adc_ifc_timing)
#define IPU_CSI_INIT_INTERFACE        _IOW('I', 0x1A, ipu_csi_interface)
#define IPU_CSI_ENABLE_MCLK           _IOW('I', 0x1B, ipu_csi_mclk)
#define IPU_CSI_READ_MCLK_FLAG        _IOR('I', 0x1C, ipu_csi_mclk)
#define IPU_CSI_FLASH_STROBE          _IOW('I', 0x1D, ipu_csi_mclk)
#define IPU_CSI_GET_WIN_SIZE          _IOR('I', 0x1E, ipu_csi_window_size)
#define IPU_CSI_SET_WIN_SIZE          _IOW('I', 0x1F, ipu_csi_window_size)
#define IPU_CSI_SET_WINDOW            _IOW('I', 0x20, ipu_csi_window)
#define IPU_PF_SET_PAUSE_ROW          _IOW('I', 0x21, uint32_t)
#define IPU_REGISTER_GENERIC_ISR      _IOW('I', 0x22, ipu_event_info)
#define IPU_GET_EVENT                 _IOWR('I', 0x23, ipu_event_info)
#define IPU_ALOC_MEM		      _IOWR('I', 0x24, ipu_mem_info)
#define IPU_FREE_MEM		      _IOW('I', 0x25, ipu_mem_info)
#define IPU_IS_CHAN_BUSY	      _IOW('I', 0x26, ipu_channel_t)
#define IPU_CALC_STRIPES_SIZE	      _IOWR('I', 0x27, ipu_stripe_parm)
#define IPU_UPDATE_BUF_OFFSET         _IOW('I', 0x28, ipu_buf_offset_parm)
#define IPU_CSC_UPDATE                _IOW('I', 0x29, ipu_csc_update)
#define IPU_SELECT_MULTI_VDI_BUFFER   _IOW('I', 0x2A, uint32_t)

int ipu_calc_stripes_sizes(const unsigned int input_frame_width,
				unsigned int output_frame_width,
				const unsigned int maximal_stripe_width,
				const unsigned long long cirr,
				const unsigned int equal_stripes,
				u32 input_pixelformat,
				u32 output_pixelformat,
				struct stripe_param *left,
				struct stripe_param *right);
#endif
