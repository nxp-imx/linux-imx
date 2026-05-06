/* SPDX-License-Identifier: ((GPL-2.0+ WITH Linux-syscall-note) OR MIT) */
/*
 * NXP NEOISP userspace API
 *
 * Copyright 2023-2026 NXP
 */

#ifndef __UAPI_NXP_NEOISP_H
#define __UAPI_NXP_NEOISP_H

#include <linux/media/v4l2-isp.h>
#include <linux/types.h>
#include <linux/v4l2-controls.h>

/*
 * Check Documentation/admin-guide/media/nxp-neoisp.rst for control details.
 */
#define V4L2_CID_NEOISP_SUPPORTED_PARAMS_BLOCKS (V4L2_CID_USER_NEOISP_BASE + 0)
#define V4L2_CID_NEOISP_QUERYCAP (V4L2_CID_USER_NEOISP_BASE + 1)

/* Values for Neoisp 'capabilities' in custom QUERYCAP */
#define NEO_CAP_ALIGNMENT_MSB (1u << 0)

/* Local memories sizes (words size) */

/* CTemp statistics - 256 bytes - 64 x 32bits words */
#define NEO_CTEMP_R_SUM_CNT 64
#define NEO_CTEMP_G_SUM_CNT 64
#define NEO_CTEMP_B_SUM_CNT 64
/* CTemp statistics pixel count - 128 bytes - 64 x 16bits words */
#define NEO_CTEMP_PIX_CNT_CNT 64
/* RGBIR histogram - 1024 bytes - 256 x 32bits words */
#define NEO_RGBIR_HIST_CNT 256
/* Histograms/Statistics - 2048 bytes - 512 x 32bits words */
#define NEO_HIST_STAT_CNT 512
/* DRC Global histograms - 1664 bytes - 416 x 32bits words */
#define NEO_DRC_GLOBAL_HIST_ROI_CNT 416
/* DRC local sum - 4096 - 1024 x 32bits words */
#define NEO_DRC_LOCAL_SUM_CNT 1024
/* Vignetting look up table - 6144 bytes - 3072 x 16bits words */
#define NEO_VIGNETTING_TABLE_SIZE 3072
/* DRC Global Tonemap - 832 bytes - 416 x 16bits words */
#define NEO_DRC_GLOBAL_TONEMAP_SIZE 416
/* DRC Local Tonemap - 1024 bytes - 1024 x 8bits words */
#define NEO_DRC_LOCAL_TONEMAP_SIZE 1024

/**
 * struct neoisp_pipe_conf_cfg_s - Pipeline Configuration
 * @img_conf_inalign0:	Input image 0 pixel alignment (0: LSB; 1: MSB)
 * @img_conf_lpalign0:	Linepath 0 pixel alignment (0: LSB; 1: MSB)
 * @img_conf_inalign1:	Input image 1 pixel alignment (0: LSB; 1: MSB)
 * @img_conf_lpalign1:	Linepath 1 pixel alignment (0: LSB; 1: MSB)
 *
 * These fields configure how the images are fetched into the NEO pipeline.
 *
 * INALIGN0/1 configures if the image is fetched MSB or LSB-aligned from the
 * 16-bit aligned words in the DDR buffer.
 *
 * LPALIGN0/1 configures how the N-bit pixel data is fetched from the DDR
 * buffer will be stored into the ISP internal pipeline.
 */
struct neoisp_pipe_conf_cfg_s {
	__u8 img_conf_inalign0;
	__u8 img_conf_lpalign0;
	__u8 img_conf_inalign1;
	__u8 img_conf_lpalign1;
};

/**
 * struct neoisp_head_color_cfg_s - Head color configuration
 * @ctrl_hoffset:	Horizontal Head Pixel offset
 * @ctrl_voffset:	Vertical Head Pixel offset
 */
struct neoisp_head_color_cfg_s {
	__u8 ctrl_hoffset;
	__u8 ctrl_voffset;
};

/**
 * struct neoisp_hdr_decompress0_cfg_s - HDR Decompression for line path 0 configuration
 * @knee_point1:	Knee point 1 value for interpolation step of the decompression
 * @knee_point2:	Knee point 2 value for interpolation step of the decompression
 * @knee_point3:	Knee point 3 value for interpolation step of the decompression
 * @knee_point4:	Knee point 4 value for interpolation step of the decompression
 * @knee_offset0:	Knee point offset 0 value for interpolation step of the decompression
 * @knee_offset1:	Knee point offset 1 value for interpolation step of the decompression
 * @knee_offset2:	Knee point offset 2 value for interpolation step of the decompression
 * @knee_offset3:	Knee point offset 3 value for interpolation step of the decompression
 * @knee_offset4:	Knee point offset 4 value for interpolation step of the decompression
 * @knee_ratio0:	Knee point ratio 0 value for interpolation step of the decompression
 * @knee_ratio1:	Knee point ratio 1 value for interpolation step of the decompression
 * @knee_ratio2:	Knee point ratio 2 value for interpolation step of the decompression
 * @knee_ratio3:	Knee point ratio 3 value for interpolation step of the decompression
 * @knee_ratio4:	Knee point ratio 4 value for interpolation step of the decompression
 * @knee_npoint0:	New knee point 0 value for the output
 * @knee_npoint1:	New knee point 1 value for the output
 * @knee_npoint2:	New knee point 2 value for the output
 * @knee_npoint3:	New knee point 3 value for the output
 * @knee_npoint4:	New knee point 4 value for the output
 */
struct neoisp_hdr_decompress0_cfg_s {
	__u16 knee_point1;
	__u16 knee_point2;
	__u16 knee_point3;
	__u16 knee_point4;
	__u16 knee_offset0;
	__u16 knee_offset1;
	__u16 knee_offset2;
	__u16 knee_offset3;
	__u16 knee_offset4;
	__u16 knee_ratio0;
	__u16 knee_ratio1;
	__u16 knee_ratio2;
	__u16 knee_ratio3;
	__u16 knee_ratio4;
	__u32 knee_npoint0;
	__u32 knee_npoint1;
	__u32 knee_npoint2;
	__u32 knee_npoint3;
	__u32 knee_npoint4;
};

/**
 * struct neoisp_hdr_decompress1_cfg_s - HDR Decompression for line path 1 configuration
 * @knee_point1:	Knee point 1 value for interpolation step of the decompression
 * @knee_point2:	Knee point 2 value for interpolation step of the decompression
 * @knee_point3:	Knee point 3 value for interpolation step of the decompression
 * @knee_point4:	Knee point 4 value for interpolation step of the decompression
 * @knee_offset0:	Knee point offset 0 value for interpolation step of the decompression
 * @knee_offset1:	Knee point offset 1 value for interpolation step of the decompression
 * @knee_offset2:	Knee point offset 2 value for interpolation step of the decompression
 * @knee_offset3:	Knee point offset 3 value for interpolation step of the decompression
 * @knee_offset4:	Knee point offset 4 value for interpolation step of the decompression
 * @knee_ratio0:	Knee point ratio 0 value for interpolation step of the decompression
 * @knee_ratio1:	Knee point ratio 1 value for interpolation step of the decompression
 * @knee_ratio2:	Knee point ratio 2 value for interpolation step of the decompression
 * @knee_ratio3:	Knee point ratio 3 value for interpolation step of the decompression
 * @knee_ratio4:	Knee point ratio 4 value for interpolation step of the decompression
 * @knee_npoint0:	New knee point 0 value for the output
 * @knee_npoint1:	New knee point 1 value for the output
 * @knee_npoint2:	New knee point 2 value for the output
 * @knee_npoint3:	New knee point 3 value for the output
 * @knee_npoint4:	New knee point 4 value for the output
 */
struct neoisp_hdr_decompress1_cfg_s {
	__u16 knee_point1;
	__u16 knee_point2;
	__u16 knee_point3;
	__u16 knee_point4;
	__u16 knee_offset0;
	__u16 knee_offset1;
	__u16 knee_offset2;
	__u16 knee_offset3;
	__u16 knee_offset4;
	__u16 knee_ratio0;
	__u16 knee_ratio1;
	__u16 knee_ratio2;
	__u16 knee_ratio3;
	__u16 knee_ratio4;
	__u16 knee_npoint0;
	__u16 knee_npoint1;
	__u16 knee_npoint2;
	__u16 knee_npoint3;
	__u16 knee_npoint4;
};

#define NEO_OBWB_CNT (3)

/**
 * struct neoisp_obwb_cfg_s - Optical Black correction and White Balance configuration
 * @ctrl_obpp:		Indicates the size of pixel components output
 *			(0: 12bpp; 1: 14bpp; 2: 16bpp; 3: 20bpp)
 * @r_ctrl_gain:	Provides gain for red channel
 * @r_ctrl_offset:	Provides offset for red channel
 * @gr_ctrl_gain:	Provides gain for green red channel
 * @gr_ctrl_offset:	Provides offset for green red channel
 * @gb_ctrl_gain:	Provides gain for green blue channel
 * @gb_ctrl_offset:	Provides offset for green blue channel
 * @b_ctrl_gain:	Provides gain for blue channel
 * @b_ctrl_offset:	Provides offset for blue channel
 */
struct neoisp_obwb_cfg_s {
	__u8 ctrl_obpp;
	__u16 r_ctrl_gain;
	__u16 r_ctrl_offset;
	__u16 gr_ctrl_gain;
	__u16 gr_ctrl_offset;
	__u16 gb_ctrl_gain;
	__u16 gb_ctrl_offset;
	__u16 b_ctrl_gain;
	__u16 b_ctrl_offset;
};

/**
 * struct neoisp_hdr_merge_cfg_s - HDR merge of 2 incoming images in a line-by-line manner
 * @ctrl_motion_fix_en:		Set 1 to enable fixing of HDR artifacts due to motion
 * @ctrl_blend_3x3:		Selects the HDR blending mode (0: 1x1; 1:3x3)
 * @ctrl_gain1bpp:		Size of pixel components after gain on line path 1
 * @ctrl_gain0bpp:		Size of pixel components after gain on line path 0
 * @ctrl_obpp:			Size of pixel components for the HDR Merge output
 * @gain_offset_offset1:	Offset parameter for input image 1
 * @gain_offset_offset0:	Offset parameter for input image 0
 * @gain_scale_scale1:		Scale factor of input pixel components of image 1
 * @gain_scale_scale0:		Scale factor of input pixel components of image 0
 * @gain_shift_shift1:		Shift factor (right shift) for gained image 1
 * @gain_shift_shift0:		Shift factor (right shift) for gained image 0
 * @luma_th_th0:		Provides luminance threshold 0
 * @luma_scale_scale:		Scaling value which multiplies the threshold-conditioned luminance
 * @luma_scale_shift:		Right shift value the scaling factor
 * @luma_scale_thshift:		Right shift value for binomial output before threshold function
 * @downscale_imgscale1:	Down scaling (right shift) value corresponding to image 1
 * @downscale_imgscale0:	Down scaling (right shift) value corresponding to image 0
 * @upscale_imgscale1:		Up scaling (left shift) value corresponding to image 1
 * @upscale_imgscale0:		Up scaling (left shift) value corresponding to image 0
 * @post_scale_scale:		Down scaling (right shift) of the final blended output
 */
struct neoisp_hdr_merge_cfg_s {
	__u8 ctrl_motion_fix_en;
	__u8 ctrl_blend_3x3;
	__u8 ctrl_gain1bpp;
	__u8 ctrl_gain0bpp;
	__u8 ctrl_obpp;
	__u16 gain_offset_offset1;
	__u16 gain_offset_offset0;
	__u16 gain_scale_scale1;
	__u16 gain_scale_scale0;
	__u8 gain_shift_shift1;
	__u8 gain_shift_shift0;
	__u16 luma_th_th0;
	__u16 luma_scale_scale;
	__u8 luma_scale_shift;
	__u8 luma_scale_thshift;
	__u8 downscale_imgscale1;
	__u8 downscale_imgscale0;
	__u8 upscale_imgscale1;
	__u8 upscale_imgscale0;
	__u8 post_scale_scale;
};

/**
 * struct neoisp_roi_cfg_s - common ROI structure
 * @xpos:	Provides the horizontal start position (pixel number) of the ROI
 * @ypos:	Provides the vertical start position (line number) of the ROI
 * @width:	Provides the horizontal width of the ROI
 * @height:	Provides the vertical height of the ROI
 */
struct neoisp_roi_cfg_s {
	__u16 xpos;
	__u16 ypos;
	__u16 width;
	__u16 height;
};

/**
 * struct neoisp_stat_hist_cfg_s - common stat histograms structure
 * @hist_ctrl_offset:		Black level correction offset for each pixel
 * @hist_ctrl_channel:		Binary value of channel for binning on respective histogram
 * @hist_ctrl_pattern:		Defines neighbouring pixel 1x1 (0) vs 2x2 (1)
 * @hist_ctrl_dir_input1_dif:	Defines Direct (0) vs Difference (1)
 * @hist_ctrl_lin_input1_log:	Defines Linear (0) vs Logarithmic (1)
 * @hist_scale_scale:		Scaling factor on the input pixel for bin determination
 */
struct neoisp_stat_hist_cfg_s {
	__u16 hist_ctrl_offset;
	__u8 hist_ctrl_channel;
	__u8 hist_ctrl_pattern;
	__u8 hist_ctrl_dir_input1_dif;
	__u8 hist_ctrl_lin_input1_log;
	__u32 hist_scale_scale;
};

#define NEO_RGBIR_ROI_CNT       (2)
#define NEO_RGBIR_STAT_HIST_CNT (2)

/**
 * struct neoisp_rgbir_cfg_s - RGBIR to RGGB and IR unit configuration
 * @ccm0_ccm:		Color correction parameter for component 0 (crosstalk 0) red if RGGB
 * @ccm1_ccm:		Color correction parameter for component 1 (crosstalk 1) both green if RGGB
 * @ccm2_ccm:		Color correction parameter for component 2 (crosstalk 2) blue if RGGB
 * @ccm0_th_threshold:	Crosstalk removal threshold from channel 3 (IR) to channel 0 (red)
 * @ccm1_th_threshold:	Crosstalk removal threshold from channel 3 (IR) to channel 1 (green)
 * @ccm2_th_threshold:	Crosstalk removal threshold from channel 3 (IR) to channel 2 (blue)
 * @roi:		Array of region of interests
 * @hists:		Array of histograms parameters
 */
struct neoisp_rgbir_cfg_s {
	__u16 ccm0_ccm;
	__u16 ccm1_ccm;
	__u16 ccm2_ccm;
	__u32 ccm0_th_threshold;
	__u32 ccm1_th_threshold;
	__u32 ccm2_th_threshold;
	struct neoisp_roi_cfg_s roi[NEO_RGBIR_ROI_CNT];
	struct neoisp_stat_hist_cfg_s hists[NEO_RGBIR_STAT_HIST_CNT];
};

#define NEO_STAT_HIST_CNT (4)

/**
 * struct neoisp_stat_cfg_s - Statistics and Histogram unit configuration
 * @roi0:	Region of interest 0
 * @roi1:	Region of interest 1
 * @hists:	Control parameters for building the histogram
 */
struct neoisp_stat_cfg_s {
	struct neoisp_roi_cfg_s roi0;
	struct neoisp_roi_cfg_s roi1;
	struct neoisp_stat_hist_cfg_s hists[NEO_STAT_HIST_CNT];
};

/**
 * struct neoisp_ir_compress_cfg_s - Infra-red Compression unit configuration
 * @ctrl_obpp:			bpp of compressed output IR (0: 8bpp; 1: 16bpp)
 * @knee_point1_kneepoint:	Knee point 1 value for interpolation step of ir compression
 * @knee_point2_kneepoint:	Knee point 2 value for interpolation step of ir compression
 * @knee_point3_kneepoint:	Knee point 3 value for interpolation step of ir compression
 * @knee_point4_kneepoint:	Knee point 4 value for interpolation step of ir compression
 * @knee_offset0_offset:	Offset 0 value for interpolation step of ir compression
 * @knee_offset1_offset:	Offset 1 value for interpolation step of ir compression
 * @knee_offset2_offset:	Offset 2 value for interpolation step of ir compression
 * @knee_offset3_offset:	Offset 3 value for interpolation step of ir compression
 * @knee_offset4_offset:	Offset 4 value for interpolation step of ir compression
 * @knee_ratio01_ratio0:	Ratio 0 value for interpolation step of ir compression (u1.15)
 * @knee_ratio01_ratio1:	Ratio 1 value for interpolation step of ir compression (u1.15)
 * @knee_ratio23_ratio2:	Ratio 2 value for interpolation step of ir compression (u1.15)
 * @knee_ratio23_ratio3:	Ratio 3 value for interpolation step of ir compression (u1.15)
 * @knee_ratio4_ratio4:		Ratio 4 value for interpolation step of ir compression (u1.15)
 * @knee_npoint0_kneepoint:	New 0 knee point value for the output
 * @knee_npoint1_kneepoint:	New 1 knee point value for the output
 * @knee_npoint2_kneepoint:	New 2 knee point value for the output
 * @knee_npoint3_kneepoint:	New 3 knee point value for the output
 * @knee_npoint4_kneepoint:	New 4 knee point value for the output
 */
struct neoisp_ir_compress_cfg_s {
	__u8 ctrl_obpp;
	__u32 knee_point1_kneepoint;
	__u32 knee_point2_kneepoint;
	__u32 knee_point3_kneepoint;
	__u32 knee_point4_kneepoint;
	__u32 knee_offset0_offset;
	__u32 knee_offset1_offset;
	__u32 knee_offset2_offset;
	__u32 knee_offset3_offset;
	__u32 knee_offset4_offset;
	__u16 knee_ratio01_ratio0;
	__u16 knee_ratio01_ratio1;
	__u16 knee_ratio23_ratio2;
	__u16 knee_ratio23_ratio3;
	__u16 knee_ratio4_ratio4;
	__u16 knee_npoint0_kneepoint;
	__u16 knee_npoint1_kneepoint;
	__u16 knee_npoint2_kneepoint;
	__u16 knee_npoint3_kneepoint;
	__u16 knee_npoint4_kneepoint;
};

/**
 * struct neoisp_bnr_cfg_s - Bayer Noise Reduction unit configuration
 * @ctrl_debug:		Debug view for on-target tuning (0:off)
 * @ctrl_obpp:		Output bpp (0: 12bpp; 1: 14bpp; 2: 16bpp; 3: 20bpp)
 * @ctrl_nhood:		Neighbourhood Pattern (0: 2x2; 1: 1x1)
 * @ypeak_peak_outsel:	Output scaling (0: no scaling; 1: enable scaling)
 * @ypeak_peak_sel:	Selecting the boundary pixel among the sorted list (0..3: 1..4 positions)
 * @ypeak_peak_low:	Lower scale value of the clipping function (u4.8)
 * @ypeak_peak_high:	Higher scale value of the clipping function (u4.8)
 * @yedge_th0_edge_th0:	Lower edge threshold for long alpha blending coefficient calculation
 * @yedge_scale_scale:	Scaling factor for long alpha blending factor determination
 * @yedge_scale_shift:	Right shift factor for blending factor determination
 *			For example, a shift value of 10, implements u6.10 scaling factor
 * @yedges_th0_edge_th0: Lower threshold for short alpha blending function in the BNR
 * @yedges_scale_scale:	Scale factor for determining the short alpha blending threshold value
 * @yedges_scale_shift:	Right shift factor for blending factor determination
 *			For example, a shift value of 10, implements u6.10 scaling factor
 * @yedgea_th0_edge_th0: Lower threshold for final alpha blending function in the BNR
 * @yedgea_scale_scale:	Scale factor for determining the final alpha blending threshold value
 * @yedgea_scale_shift:	Right shift factor for blending factor determination
 *			For example, a shift value of 10, implements u6.10 scaling factor
 * @yluma_x_th0_th:	X threshold 0 for blending coefficient calculation
 * @yluma_y_th_luma_y_th0: 10-bit value for Y threshold 0
 * @yluma_y_th_luma_y_th1: 10-bit value for Y threshold 1
 * @yluma_scale_scale:	Scale for the threshold-conditioned luma factor determination
 * @yluma_scale_shift:	Right shift factor for blending factor determination
 *			For example, a shift value of 10, implements u6.10 scaling factor
 * @yalpha_gain_gain:	Gain value (multiplication factor) for the alpha coefficient
 * @yalpha_gain_offset:	Offset value (addition factor) for the gain'd alpha coefficient
 * @cpeak_peak_outsel:	Set 1 to enable scaling of the output, 0 no scaling
 * @cpeak_peak_sel:	Provides selection for selecting the boundary pixel among the sorted list
 * @cpeak_peak_low:	ower scale value of the clipping function (u4.8)
 * @cpeak_peak_high:	Higher scale value of the clipping function (u4.8)
 * @cedge_th0_edge_th0:	Lower threshold for blending function in the BNR unit
 * @cedge_scale_scale:	Scale for the threshold-conditioned blending factor determination
 * @cedge_scale_shift:	Right shift factor for blending factor determination
 *			For example, a shift value of 10, implements u6.10 scaling factor
 * @cedges_th0_edge_th0: Lower threshold for short alpha blending function in the BNR unit
 * @cedges_scale_scale:	Scale for the threshold-conditioned short alpha blending factor
 * @cedges_scale_shift:	Right shift factor for blending factor determination
 *			For example, a shift value of 10, implements u6.10 scaling factor
 * @cedgea_th0_edge_th0: Lower threshold for final alpha blending function
 * @cedgea_scale_scale:	Scale for the threshold-conditioned final alpha blending factor
 * @cedgea_scale_shift:	Right shift factor for blending factor determination
 *			For example, a shift value of 10, implements u6.10 scaling factor
 * @cluma_x_th0_th:	Provides the X threshold 0 for blending coefficient calculation
 * @cluma_y_th_luma_y_th0: 10-bit value for Y threshold 0
 * @cluma_y_th_luma_y_th1: 10-bit value for Y threshold 1
 * @cluma_scale_scale:	Scale for the threshold-conditioned luma factor determination
 * @cluma_scale_shift:	Right shift factor for blending factor determination
 *			For example, a shift value of 10, implements u6.10 scaling factor
 * @calpha_gain_gain:	Provides the gain value (multiplication factor) for the alpha coefficient
 * @calpha_gain_offset:	Provides the offset value (addition factor) for the gain'd alpha coefficient
 * @stretch_gain:	Provides the gain factor for all the pixels at the output of BNR (u8.8)
 *			This gain is applied even when BNR is disabled
 */
struct neoisp_bnr_cfg_s {
	__u8 ctrl_debug;
	__u8 ctrl_obpp;
	__u8 ctrl_nhood;
	__u8 ypeak_peak_outsel;
	__u8 ypeak_peak_sel;
	__u16 ypeak_peak_low;
	__u16 ypeak_peak_high;
	__u32 yedge_th0_edge_th0;
	__u16 yedge_scale_scale;
	__u8 yedge_scale_shift;
	__u32 yedges_th0_edge_th0;
	__u16 yedges_scale_scale;
	__u8 yedges_scale_shift;
	__u32 yedgea_th0_edge_th0;
	__u16 yedgea_scale_scale;
	__u8 yedgea_scale_shift;
	__u32 yluma_x_th0_th;
	__u16 yluma_y_th_luma_y_th0;
	__u16 yluma_y_th_luma_y_th1;
	__u16 yluma_scale_scale;
	__u8 yluma_scale_shift;
	__u16 yalpha_gain_gain;
	__u16 yalpha_gain_offset;
	__u8 cpeak_peak_outsel;
	__u8 cpeak_peak_sel;
	__u16 cpeak_peak_low;
	__u16 cpeak_peak_high;
	__u32 cedge_th0_edge_th0;
	__u16 cedge_scale_scale;
	__u8 cedge_scale_shift;
	__u32 cedges_th0_edge_th0;
	__u16 cedges_scale_scale;
	__u8 cedges_scale_shift;
	__u32 cedgea_th0_edge_th0;
	__u16 cedgea_scale_scale;
	__u8 cedgea_scale_shift;
	__u32 cluma_x_th0_th;
	__u16 cluma_y_th_luma_y_th0;
	__u16 cluma_y_th_luma_y_th1;
	__u16 cluma_scale_scale;
	__u8 cluma_scale_shift;
	__u16 calpha_gain_gain;
	__u16 calpha_gain_offset;
	__u16 stretch_gain;
};

/**
 * struct neoisp_vignetting_ctrl_cfg_s - Vignetting controlling configuration
 * @blk_conf_rows:	Provides number of rows into which the input image is partitioned
 * @blk_conf_cols:	Provides number of columns into which the input image is partitioned
 * @blk_size_ysize:	Number of rows per block
 * @blk_size_xsize:	Number of pixels per block
 * @blk_stepy_step:	Vertical scaling factor (u0.16)
 * @blk_stepx_step:	Horizontal scaling factor (u0.16)
 */
struct neoisp_vignetting_ctrl_cfg_s {
	__u8 blk_conf_rows;
	__u8 blk_conf_cols;
	__u16 blk_size_ysize;
	__u16 blk_size_xsize;
	__u16 blk_stepy_step;
	__u16 blk_stepx_step;
};

#define NEO_CTEMP_COLOR_ROIS_CNT (10)
#define NEO_CTEMP_CSC_MATRIX_SIZE (3)
#define NEO_CTEMP_CSC_OFFSET_VECTOR_SIZE (4)

/**
 * struct neoisp_ctemp_roi_desc_s - common color ROI Position Register
 * @pos_roverg_low:	Low value of red over green (u1.7)
 * @pos_roverg_high:	High value of red over green (u1.7)
 * @pos_boverg_low:	Low value of blue over green (u1.7)
 * @pos_boverg_high:	High value of blue over green (u1.7)
 */
struct neoisp_ctemp_roi_desc_s {
	__u8 pos_roverg_low;
	__u8 pos_roverg_high;
	__u8 pos_boverg_low;
	__u8 pos_boverg_high;
};

/**
 * struct neoisp_ctemp_cfg_s - Color temperature unit configuration
 * @ctrl_cscon:		Color Space Correction ON (1), (0) disabled
 * @ctrl_ibpp:		Size of pixel components on input (0: 12bpp; 1: 14bpp; 2: 16bpp; 3: 20bpp)
 * @luma_th_thl:	Provides the low threshold for luminance range check
 * @luma_th_thh:	Provides the high threshold for luminance range check
 * @roi:		Array of regions of interest
 * @redgain_min:	Minimum gain for the red channel (u1.7)
 * @redgain_max:	Maximum gain for the red channel (u1.7)
 * @bluegain_min:	Minimum gain for the blue channel (u1.7)
 * @bluegain_max:	Maximum gain for the blue channel (u1.7)
 * @point1_blue:	Point 1 value for blue over green curve (u1.7)
 * @point1_red:		Point 1 value for red over green curve (u1.7)
 * @point2_blue:	Point 2 value for blue over green curve (u1.7)
 * @point2_red:		Point 2 value for red over green curve (u1.7)
 * @hoffset_right:	Offset in increasing horizontal indices (u1.7)
 * @hoffset_left:	Offset in decreasing horizontal indices (u1.7)
 * @voffset_up:		Offset in increasing vertical indices (u1.7)
 * @voffset_down:	Offset in decreasing vertical indices (u1.7)
 * @point1_slope_slope_l: Left slope for point 1 (s8.8)
 * @point1_slope_slope_r: Right slope for point 1 (s8.8)
 * @point2_slope_slope_l: Left slope for point 2 (s8.8)
 * @point2_slope_slope_r: Right slope for point 2 (s8.8)
 * @csc_matrix:		A 3x3 color space correction matrix for respective camera context (s8.8)
 * @offsets:		Correction offsets values of input filter array pixel
 * @stat_blk_size0_xsize: Number of pixels per block. Should always be multiple of 2
 * @stat_blk_size0_ysize: Number of image lines per block. Should always be multiple of 2
 * @color_rois:		Array of color regions of interest
 * @gr_avg_in_gr_agv:	Subtracted from the GR values before accumulation into the GR vs GB sums
 * @gb_avg_in_gb_agv:	Subtracted from the GB values before accumulation into the GR vs GB sums
 */
struct neoisp_ctemp_cfg_s {
	__u8 ctrl_cscon;
	__u8 ctrl_ibpp;
	__u16 luma_th_thl;
	__u16 luma_th_thh;
	struct neoisp_roi_cfg_s roi;
	__u8 redgain_min;
	__u8 redgain_max;
	__u8 bluegain_min;
	__u8 bluegain_max;
	__u8 point1_blue;
	__u8 point1_red;
	__u8 point2_blue;
	__u8 point2_red;
	__u8 hoffset_right;
	__u8 hoffset_left;
	__u8 voffset_up;
	__u8 voffset_down;
	__s16 point1_slope_slope_l;
	__s16 point1_slope_slope_r;
	__s16 point2_slope_slope_l;
	__s16 point2_slope_slope_r;
	__s16 csc_matrix[NEO_CTEMP_CSC_MATRIX_SIZE][NEO_CTEMP_CSC_MATRIX_SIZE];
	__s16 offsets[NEO_CTEMP_CSC_OFFSET_VECTOR_SIZE];
	__u16 stat_blk_size0_xsize;
	__u16 stat_blk_size0_ysize;
	struct neoisp_ctemp_roi_desc_s color_rois[NEO_CTEMP_COLOR_ROIS_CNT];
	__u32 gr_avg_in_gr_agv;
	__u32 gb_avg_in_gb_agv;
};

/**
 * struct neoisp_demosaic_cfg_s - Demosaic function on the input bayer image configuration
 * @ctrl_fmt:			Format of the input image (0: rggb; 1: rccc; 2: monochrome)
 * @activity_ctl_alpha:		Alpha Blending Factor (u1.8)
 * @activity_ctl_act_ratio:	Activity Ratio (u8.8)
 * @dynamics_ctl0_strengthg:	Feedback strength for green pixel interpolation (u8.8)
 * @dynamics_ctl0_strengthc:	Feedback strength for color (red or blue) pixel interpolation (u8.8)
 * @dynamics_ctl2_max_impact:	Maximum impact of the dynamics on the interpolated values (u8.8)
 */
struct neoisp_demosaic_cfg_s {
	__u8 ctrl_fmt;
	__u16 activity_ctl_alpha;
	__u16 activity_ctl_act_ratio;
	__u16 dynamics_ctl0_strengthg;
	__u16 dynamics_ctl0_strengthc;
	__u16 dynamics_ctl2_max_impact;
};

#define NEO_RGB2YUV_MATRIX_SIZE (3)

/**
 * struct neoisp_rgb2yuv_cfg_s - Color space conversion RGB to YUV data configuration
 * @gain_ctrl_rgain:	Provides the gain factor corresponding to red component
 * @gain_ctrl_bgain:	Provides the gain factor corresponding to blue component
 * @mat_rxcy:		Provides the values of elements of the 3x3 color space conversion matrix
 * @csc_offsets:	Provides the offsets of the color space conversion matrix (s21)
 */
struct neoisp_rgb2yuv_cfg_s {
	__u16 gain_ctrl_rgain;
	__u16 gain_ctrl_bgain;
	__s16 mat_rxcy[NEO_RGB2YUV_MATRIX_SIZE][NEO_RGB2YUV_MATRIX_SIZE];
	__s32 csc_offsets[NEO_RGB2YUV_MATRIX_SIZE];
};

/**
 * struct neoisp_dr_comp_cfg_s - Dynamic Range Compression unit configuration
 * @roi0:			Region of interest 0
 * @roi1:			Region of interest 1
 * @groi_sum_shift_shift0:	Global ROI 0 sum shift value (u5)
 * @groi_sum_shift_shift1:	Global ROI 1 sum shift value (u5)
 * @gbl_gain_gain:		Provides a gain for the global DRC (u8.8)
 * @lcl_blk_size_xsize:		Provides number of pixels per block
 * @lcl_blk_size_ysize:		Provides number of rows per block
 * @lcl_stretch_offset:		Black level value before applying gamma
 * @lcl_stretch_stretch:	Provides local DRC stretch value of the input (u8.8)
 * @lcl_blk_stepx_step:		Horizontal scaling factor (u0.16)
 * @lcl_blk_stepy_step:		Vertical scaling factor (u0.16)
 * @lcl_sum_shift_shift:	Provides shift value for building the local DRC (u5)
 * @alpha_alpha:		Alpha value for blending step between global and local DRC (u9)
 */
struct neoisp_dr_comp_cfg_s {
	struct neoisp_roi_cfg_s roi0;
	struct neoisp_roi_cfg_s roi1;
	__u8 groi_sum_shift_shift0;
	__u8 groi_sum_shift_shift1;
	__u16 gbl_gain_gain;
	__u16 lcl_blk_size_xsize;
	__u16 lcl_blk_size_ysize;
	__u16 lcl_stretch_offset;
	__u16 lcl_stretch_stretch;
	__u16 lcl_blk_stepx_step;
	__u16 lcl_blk_stepy_step;
	__u8 lcl_sum_shift_shift;
	__u16 alpha_alpha;
};

/**
 * struct neoisp_nr_cfg_s - Noise Reduction unit configuration
 * @ctrl_debug:		This field controls if tuning/debug information
 * @blend_scale_gain:	Gain value for the blending factor determination (u4.4)
 * @blend_scale_shift:	Shift value for the blending factor determination
 * @blend_scale_scale:	Scale factor for the blending factor determination
 * @blend_th0_th:	Provides threshold 0 value for determining the blending factor (u20)
 */
struct neoisp_nr_cfg_s {
	__u8 ctrl_debug;
	__u8 blend_scale_gain;
	__u8 blend_scale_shift;
	__u16 blend_scale_scale;
	__u32 blend_th0_th;
};

#define NEO_AF_ROIS_CNT (9)
#define NEO_AF_FILTERS_CNT (9)

/**
 * struct neoisp_af_cfg_s - AutoFocus unit configuration
 * @af_roi:		Array of regions of interest
 * @fil0_coeffs:	Array of Autofocus Filter 0 Coefficients
 * @fil0_shift_shift:	Provides the shift (scale down) factor at the output of filter 0 (u5)
 * @fil1_coeffs:	Array of Autofocus Filter 1 Coefficients
 * @fil1_shift_shift:	Provides the shift (scale down) factor at the output of filter 1 (u5)
 */
struct neoisp_af_cfg_s {
	struct neoisp_roi_cfg_s af_roi[NEO_AF_ROIS_CNT];
	__s8 fil0_coeffs[NEO_AF_FILTERS_CNT];
	__u8 fil0_shift_shift;
	__s8 fil1_coeffs[NEO_AF_FILTERS_CNT];
	__u8 fil1_shift_shift;
};

/**
 * struct neoisp_ee_cfg_s - Edge Enhancement unit configuration
 * @ctrl_debug:		This field controls if tuning/debug information is shown in the
 *			output image (0: Off; 1: edge pixels shown as white; 2: edge
 *			pixels shown as white and all others)
 * @maskgain_gain:	Gain value for the HPF factor determination (u4.4)
 * @coring_coring:	Coring value for the mask factor determination (u20)
 * @clip_clip:		Clip value for the mask factor determination (u20)
 */
struct neoisp_ee_cfg_s {
	__u8 ctrl_debug;
	__u8 maskgain_gain;
	__u32 coring_coring;
	__u32 clip_clip;
};

/**
 * struct neoisp_df_cfg_s - Direction Filter unit configuration
 * @ctrl_debug:		This field controls if tuning/debug information
 * @blend_shift_shift:	Shift factor for the blending factor determination (u6)
 * @th_scale_scale:	Scale factor for the blending factor determination (u20)
 * @blend_th0_th:	Provides threshold 0 value for determining the blending factor (u20)
 */
struct neoisp_df_cfg_s {
	__u8 ctrl_debug;
	__u8 blend_shift_shift;
	__u32 th_scale_scale;
	__u32 blend_th0_th;
};

/**
 * struct neoisp_convmed_cfg_s - Color Convolution and Median Filter unit configuration
 * @ctrl_flt:	This field controls the type of filtering to be executed:
 *		(0: bypassed; 1: convolution (5x5 binomial); 2: median (5x5))
 */
struct neoisp_convmed_cfg_s {
	__u8 ctrl_flt;
};

/**
 * struct neoisp_cas_cfg_s - Color Adaptive Saturation unit configuration
 * @gain_shift:		Shift value for the suppression factor determination
 * @gain_scale:		Scale factor for the suppression factor
 * @corr_corr:		Minimum correction factor for dark pixels (u8.8)
 * @offset_offset:	Offset value for the suppression factor determination
 */
struct neoisp_cas_cfg_s {
	__u8 gain_shift;
	__u16 gain_scale;
	__u16 corr_corr;
	__u16 offset_offset;
};

#define NEO_GAMMA_MATRIX_SIZE (3)
#define NEO_GAMMA_OFFSETS_SIZE (3)

/**
 * struct neoisp_gcm_cfg_s - Gamma Correction Matrix unit configuration
 * @imat_rxcy:			3x3 input gamma correction matrix (s8.8)
 * @ioffsets:			Offset values for input channels
 * @omat_rxcy:			3x3 output gamma correction matrix
 * @ooffsets:			Offset values of 3x3 output matrix (s12)
 * @gamma0_gamma0:		Provides the gamma value of channel 0 (u1.8)
 * @gamma0_offset0:		Provides the offset value of channel 0 (u12)
 * @gamma1_gamma1:		Provides the gamma value of channel 1 (u1.8)
 * @gamma1_offset1:		Provides the offset value of channel 1 (u12)
 * @gamma2_gamma2:		Provides the gamma value of channel 2 (u1.8)
 * @gamma2_offset2:		Provides the offset value of channel 2 (u12)
 * @blklvl0_ctrl_gain0:		Gain value for the linear part of the channel 0 gamma curve (u8.8)
 * @blklvl0_ctrl_offset0:	Blacklevel value to be subtracted on channel 0
 * @blklvl1_ctrl_gain1:		Gain value for the linear part of the channel 1 gamma curve (u8.8)
 * @blklvl1_ctrl_offset1:	Blacklevel value to be subtracted on channel 1
 * @blklvl2_ctrl_gain2:		Gain value for the linear part of the channel 2 gamma curve (u8.8)
 * @blklvl2_ctrl_offset2:	Blacklevel value to be subtracted on channel 2
 * @lowth_ctrl01_threshold0:	Threshold for low area of the dynamic range of channel 0 (u12.4)
 * @lowth_ctrl01_threshold1:	Threshold for low area of the dynamic range of channel 1 (u12.4)
 * @lowth_ctrl2_threshold2:	Threshold for low area of the dynamic range of channel 2 (u12.4)
 * @mat_confg_sign_confg:	Set 0 for signe gcm, 1 Unsigned
 */
struct neoisp_gcm_cfg_s {
	__s16 imat_rxcy[NEO_GAMMA_MATRIX_SIZE][NEO_GAMMA_MATRIX_SIZE];
	__s16 ioffsets[NEO_GAMMA_OFFSETS_SIZE];
	__s16 omat_rxcy[NEO_GAMMA_MATRIX_SIZE][NEO_GAMMA_MATRIX_SIZE];
	__s16 ooffsets[NEO_GAMMA_OFFSETS_SIZE];
	__u16 gamma0_gamma0;
	__u16 gamma0_offset0;
	__u16 gamma1_gamma1;
	__u16 gamma1_offset1;
	__u16 gamma2_gamma2;
	__u16 gamma2_offset2;
	__u16 blklvl0_ctrl_gain0;
	__s16 blklvl0_ctrl_offset0;
	__u16 blklvl1_ctrl_gain1;
	__s16 blklvl1_ctrl_offset1;
	__u16 blklvl2_ctrl_gain2;
	__s16 blklvl2_ctrl_offset2;
	__u16 lowth_ctrl01_threshold0;
	__u16 lowth_ctrl01_threshold1;
	__u16 lowth_ctrl2_threshold2;
	__u8 mat_confg_sign_confg;
};

/**
 * struct neoisp_vignetting_table_mem_params_s - Vignetting table values
 * @vignetting_table:	Array of vignetting lookup table
 */
struct neoisp_vignetting_table_mem_params_s {
	__u16 vignetting_table[NEO_VIGNETTING_TABLE_SIZE];
};

/**
 * struct neoisp_drc_global_tonemap_mem_params_s - DRC Global Tonemap
 * @drc_global_tonemap:	Global DRC tonemap lookup table
 */
struct neoisp_drc_global_tonemap_mem_params_s {
	__u16 drc_global_tonemap[NEO_DRC_GLOBAL_TONEMAP_SIZE];
};

/**
 * struct neoisp_drc_local_tonemap_mem_params_s - DRC Local Tonemap
 * @drc_local_tonemap:	Local DRC tonemap lookup table
 */
struct neoisp_drc_local_tonemap_mem_params_s {
	__u8 drc_local_tonemap[NEO_DRC_LOCAL_TONEMAP_SIZE];
};

/**
 * enum neoisp_param_block_type_e - Enumeration of Neoisp parameter blocks
 *
 * This enumeration defines the types of Neoisp parameters block. Each entry
 * configures a specific processing block of the Neoisp. The block type
 * allows the driver to correctly interpret the parameters block data.
 *
 * It is the responsibility of userspace to correctly set the type of each
 * parameters block.
 *
 * @NEOISP_PARAM_BLK_PIPE_CONF: Pipe configuration block
 * @NEOISP_PARAM_BLK_HEAD_COLOR: Head Color block
 * @NEOISP_PARAM_BLK_HDR_DECOMPRESS0: HDR decompression of line path 0
 * @NEOISP_PARAM_BLK_HDR_DECOMPRESS1: HDR decompression of line path 1
 * @NEOISP_PARAM_BLK_OBWB0: Optical Black Correction and White Balance of line path 0
 * @NEOISP_PARAM_BLK_OBWB1: Optical Black Correction and White Balance of line path 1
 * @NEOISP_PARAM_BLK_OBWB2: Optical Black Correction and White Balance of merged path
 * @NEOISP_PARAM_BLK_HDR_MERGE: HDR merge block
 * @NEOISP_PARAM_BLK_RGBIR: RGB-IR block
 * @NEOISP_PARAM_BLK_STAT: Statistics block
 * @NEOISP_PARAM_BLK_IR_COMPRESS: Infrared compression block
 * @NEOISP_PARAM_BLK_BNR: Bayer noise reduction block
 * @NEOISP_PARAM_BLK_VIGNETTING_CTRL: Vignetting control block
 * @NEOISP_PARAM_BLK_CTEMP: Color temperature block
 * @NEOISP_PARAM_BLK_DEMOSAIC: Demosaicing block
 * @NEOISP_PARAM_BLK_RGB2YUV: RGB to YUV block
 * @NEOISP_PARAM_BLK_DR_COMP: Dynamic range compression
 * @NEOISP_PARAM_BLK_NR: Noise reduction block
 * @NEOISP_PARAM_BLK_AF: Auto focus block
 * @NEOISP_PARAM_BLK_EE: Edge enhancement block
 * @NEOISP_PARAM_BLK_DF: Direction filter block
 * @NEOISP_PARAM_BLK_CONVMED: Convolution and median filter block
 * @NEOISP_PARAM_BLK_CAS: Color adaptive saturation block
 * @NEOISP_PARAM_BLK_GCM: Gamma correction matrix block
 * @NEOISP_PARAM_BLK_VIGNETTING_TABLE: Vignetting lookup table
 * @NEOISP_PARAM_BLK_DRC_GLOBAL_TONEMAP: Global tonemap table
 * @NEOISP_PARAM_BLK_DRC_LOCAL_TONEMAP: Local tonemap table
 */
enum neoisp_param_block_type_e {
	NEOISP_PARAM_BLK_PIPE_CONF,
	NEOISP_PARAM_BLK_HEAD_COLOR,
	NEOISP_PARAM_BLK_HDR_DECOMPRESS0,
	NEOISP_PARAM_BLK_HDR_DECOMPRESS1,
	NEOISP_PARAM_BLK_OBWB0,
	NEOISP_PARAM_BLK_OBWB1,
	NEOISP_PARAM_BLK_OBWB2,
	NEOISP_PARAM_BLK_HDR_MERGE,
	NEOISP_PARAM_BLK_RGBIR,
	NEOISP_PARAM_BLK_STAT,
	NEOISP_PARAM_BLK_CTEMP,
	NEOISP_PARAM_BLK_IR_COMPRESS,
	NEOISP_PARAM_BLK_BNR,
	NEOISP_PARAM_BLK_VIGNETTING_CTRL,
	NEOISP_PARAM_BLK_DEMOSAIC,
	NEOISP_PARAM_BLK_RGB2YUV,
	NEOISP_PARAM_BLK_DR_COMP,
	NEOISP_PARAM_BLK_NR,
	NEOISP_PARAM_BLK_AF,
	NEOISP_PARAM_BLK_EE,
	NEOISP_PARAM_BLK_DF,
	NEOISP_PARAM_BLK_CONVMED,
	NEOISP_PARAM_BLK_CAS,
	NEOISP_PARAM_BLK_GCM,
	NEOISP_PARAM_BLK_VIGNETTING_TABLE,
	NEOISP_PARAM_BLK_DRC_GLOBAL_TONEMAP,
	NEOISP_PARAM_BLK_DRC_LOCAL_TONEMAP,
};

/**
 * struct neoisp_pipe_conf_cfg_es - Neoisp extensible params pipeline configuration
 *
 * Neoisp extensible params block for pipelines alignment configuration.
 * Identified by :c:type:`NEOISP_PARAM_BLK_PIPE_CONF`.
 *
 * @header:	The Neoisp extensible parameters header, see
 *		:c:type:`v4l2_isp_block_header`
 * @cfg:	Pipeline configuration, see
 *		:c:type:`neoisp_pipe_conf_cfg_s`
 */
struct neoisp_pipe_conf_cfg_es {
	struct v4l2_isp_block_header header;
	struct neoisp_pipe_conf_cfg_s cfg;
} __attribute__((aligned(8)));

/**
 * struct neoisp_head_color_cfg_es - Neoisp extensible Head color configuration
 *
 * Neoisp extensible params block for head color configuration.
 * Identified by :c:type:`NEOISP_PARAM_BLK_HEAD_COLOR`.
 *
 * @header:	The Neoisp extensible parameters header, see
 *		:c:type:`v4l2_isp_block_header`
 * @cfg:	Head color configuration, see
 *		:c:type:`neoisp_head_color_cfg_s`
 */
struct neoisp_head_color_cfg_es {
	struct v4l2_isp_block_header header;
	struct neoisp_head_color_cfg_s cfg;
} __attribute__((aligned(8)));

/**
 * struct neoisp_hdr_decompress0_cfg_es - Neoisp extensible HDR Decompress0 configuration
 *
 * Neoisp extensible params block for HDR Decompression configuration of line path 0.
 * Identified by :c:type:`NEOISP_PARAM_BLK_HDR_DECOMPRESS0`.
 *
 * @header:	The Neoisp extensible parameters header, see
 *		:c:type:`v4l2_isp_block_header`
 * @cfg:	HDR Decompression configuration for line path 0, see
 *		:c:type:`neoisp_hdr_decompress0_cfg_s`
 */
struct neoisp_hdr_decompress0_cfg_es {
	struct v4l2_isp_block_header header;
	struct neoisp_hdr_decompress0_cfg_s cfg;
} __attribute__((aligned(8)));

/**
 * struct neoisp_hdr_decompress1_cfg_es - Neoisp extensible HDR Decompress1 configuration
 *
 * Neoisp extensible params block for HDR Decompression configuration of line path 1.
 * Identified by :c:type:`NEOISP_PARAM_BLK_HDR_DECOMPRESS1`.
 *
 * @header:	The Neoisp extensible parameters header, see
 *		:c:type:`v4l2_isp_block_header`
 * @cfg:	HDR Decompression configuration for line path 1, see
 *		:c:type:`neoisp_hdr_decompress1_cfg_s`
 */
struct neoisp_hdr_decompress1_cfg_es {
	struct v4l2_isp_block_header header;
	struct neoisp_hdr_decompress1_cfg_s cfg;
} __attribute__((aligned(8)));

/**
 * struct neoisp_obwb_cfg_es - Neoisp extensible OBWB configuration
 *
 * Neoisp extensible params block for Optical Black correction and White Balance
 * configuration of the different OBWB instances.
 * Identified by :c:type:`NEOISP_PARAM_BLK_OBWB0`, :c:type:`NEOISP_PARAM_BLK_OBWB1`
 * or :c:type:`NEOISP_PARAM_BLK_OBWB2`
 *
 * @header:	The Neoisp extensible parameters header, see
 *		:c:type:`v4l2_isp_block_header`
 * @cfg:	Optical Black correction and White Balance configuration, see
 *		:c:type:`neoisp_obwb_cfg_s`
 */
struct neoisp_obwb_cfg_es {
	struct v4l2_isp_block_header header;
	struct neoisp_obwb_cfg_s cfg;
} __attribute__((aligned(8)));

/**
 * struct neoisp_hdr_merge_cfg_es - Neoisp extensible HDR merge configuration
 *
 * Neoisp extensible params block for the HDR merge unit configuration.
 * Identified by :c:type:`NEOISP_PARAM_HDR_MERGE`.
 *
 * @header:	The Neoisp extensible parameters header, see
 *		:c:type:`v4l2_isp_block_header`
 * @cfg:	HDR merge configuration, see
 *		:c:type:`neoisp_hdr_merge_cfg_s`
 */
struct neoisp_hdr_merge_cfg_es {
	struct v4l2_isp_block_header header;
	struct neoisp_hdr_merge_cfg_s cfg;
} __attribute__((aligned(8)));

/**
 * struct neoisp_rgbir_cfg_es - Neoisp extensible RGBIR to RGGB and IR configuration
 *
 * Neoisp extensible params block for the RGBIR to RGGB and IR conversion unit configuration.
 * Identified by :c:type:`NEOISP_PARAM_RGBIR`.
 *
 * @header:	The Neoisp extensible parameters header, see
 *		:c:type:`v4l2_isp_block_header`
 * @cfg:	RGBIR to RGGB and IR unit configuration, see
 *		:c:type:`neoisp_rgbir_cfg_s`
 */
struct neoisp_rgbir_cfg_es {
	struct v4l2_isp_block_header header;
	struct neoisp_rgbir_cfg_s cfg;
} __attribute__((aligned(8)));

/**
 * struct neoisp_stat_cfg_es - Neoisp extensible Statistics and Histogram configuration
 *
 * Neoisp extensible params block for the Statistics and Histogram unit configuration.
 * Identified by :c:type:`NEOISP_PARAM_BLK_STAT`.
 *
 * @header:	The Neoisp extensible parameters header, see
 *		:c:type:`v4l2_isp_block_header`
 * @cfg:	Statistics and Histogram unit configuration, see
 *		:c:type:`neoisp_stat_cfg_s`
 */
struct neoisp_stat_cfg_es {
	struct v4l2_isp_block_header header;
	struct neoisp_stat_cfg_s cfg;
} __attribute__((aligned(8)));

/**
 * struct neoisp_ir_compress_cfg_es - Neoisp extensible IR Compression configuration
 *
 * Neoisp extensible params block for the Infra-red Compression unit configuration.
 * Identified by :c:type:`NEOISP_PARAM_BLK_IR_COMP`.
 *
 * @header:	The Neoisp extensible parameters header, see
 *		:c:type:`v4l2_isp_block_header`
 * @cfg:	Infra-red Compression configuration, see
 *		:c:type:`neoisp_ir_compress_cfg_s`
 */
struct neoisp_ir_compress_cfg_es {
	struct v4l2_isp_block_header header;
	struct neoisp_ir_compress_cfg_s cfg;
} __attribute__((aligned(8)));

/**
 * struct neoisp_bnr_cfg_es - Neoisp extensible BNR configuration
 *
 * Neoisp extensible params block for the Bayer Noise Reduction unit configuration.
 * Identified by :c:type:`NEOISP_PARAM_BLK_BNR`.
 *
 * @header:	The Neoisp extensible parameters header, see
 *		:c:type:`v4l2_isp_block_header`
 * @cfg:	Bayer Noise Reduction configuration, see
 *		:c:type:`neoisp_bnr_cfg_s`
 */
struct neoisp_bnr_cfg_es {
	struct v4l2_isp_block_header header;
	struct neoisp_bnr_cfg_s cfg;
} __attribute__((aligned(8)));

/**
 * struct neoisp_vignetting_ctrl_cfg_es - Neoisp extensible Vignetting configuration
 *
 * Neoisp extensible params block for the Vignetting unit configuration.
 * Identified by :c:type:`NEOISP_PARAM_BLK_VIGNETTING_CTRL`.
 *
 * @header:	The Neoisp extensible parameters header, see
 *		:c:type:`v4l2_isp_block_header`
 * @cfg:	Vignetting unit configuration, see
 *		:c:type:`neoisp_vignetting_ctrl_cfg_s`
 */
struct neoisp_vignetting_ctrl_cfg_es {
	struct v4l2_isp_block_header header;
	struct neoisp_vignetting_ctrl_cfg_s cfg;
} __attribute__((aligned(8)));

/**
 * struct neoisp_ctemp_cfg_es - Neoisp extensible Color Temperature configuration
 *
 * Neoisp extensible params block for the Color Temperature unit configuration.
 * Identified by :c:type:`NEOISP_PARAM_BLK_CTEMP`.
 *
 * @header:	The Neoisp extensible parameters header, see
 *		:c:type:`v4l2_isp_block_header`
 * @cfg:	Color Temperature unit configuration, see
 *		:c:type:`neoisp_ctemp_cfg_s`
 */
struct neoisp_ctemp_cfg_es {
	struct v4l2_isp_block_header header;
	struct neoisp_ctemp_cfg_s cfg;
} __attribute__((aligned(8)));

/**
 * struct neoisp_demosaic_cfg_es - Neoisp extensible Demosaic configuration
 *
 * Neoisp extensible params block for the Demosaic unit configuration.
 * Identified by :c:type:`NEOISP_PARAM_BLK_DEMOSAIC`.
 *
 * @header:	The Neoisp extensible parameters header, see
 *		:c:type:`v4l2_isp_block_header`
 * @cfg:	Demosaic unit configuration, see
 *		:c:type:`neoisp_demosaic_cfg_s`
 */
struct neoisp_demosaic_cfg_es {
	struct v4l2_isp_block_header header;
	struct neoisp_demosaic_cfg_s cfg;
} __attribute__((aligned(8)));

/**
 * struct neoisp_rgb2yuv_cfg_es - Neoisp extensible RGB to YUV configuration
 *
 * Neoisp extensible params block for the RGB to YUV color space conversion
 * unit configuration.
 * Identified by :c:type:`NEOISP_PARAM_BLK_RGB2YUV`.
 *
 * @header:	The Neoisp extensible parameters header, see
 *		:c:type:`v4l2_isp_block_header`
 * @cfg:	Color space conversion unit configuration, see
 *		:c:type:`neoisp_rgb2yuv_cfg_s`
 */
struct neoisp_rgb2yuv_cfg_es {
	struct v4l2_isp_block_header header;
	struct neoisp_rgb2yuv_cfg_s cfg;
} __attribute__((aligned(8)));

/**
 * struct neoisp_dr_comp_cfg_es - Neoisp extensible DRC unit configuration
 *
 * Neoisp extensible params block for the Dynamic Range Compression unit configuration.
 * Identified by :c:type:`NEOISP_PARAM_BLK_DR_COMP`.
 *
 * @header:	The Neoisp extensible parameters header, see
 *		:c:type:`v4l2_isp_block_header`
 * @cfg:	Dynamic Range Compression unit configuration, see
 *		:c:type:`neoisp_dr_comp_cfg_s`
 */
struct neoisp_dr_comp_cfg_es {
	struct v4l2_isp_block_header header;
	struct neoisp_dr_comp_cfg_s cfg;
} __attribute__((aligned(8)));

/**
 * struct neoisp_nr_cfg_es - Neoisp extensible NR unit configuration
 *
 * Neoisp extensible params block for the Noise Reduction unit configuration.
 * Identified by :c:type:`NEOISP_PARAM_BLK_NR`.
 *
 * @header:	The Neoisp extensible parameters header, see
 *		:c:type:`v4l2_isp_block_header`
 * @cfg:	Noise Reduction unit configuration, see
 *		:c:type:`neoisp_nr_cfg_s`
 */
struct neoisp_nr_cfg_es {
	struct v4l2_isp_block_header header;
	struct neoisp_nr_cfg_s cfg;
} __attribute__((aligned(8)));

/**
 * struct neoisp_af_cfg_es - Neoisp extensible AutoFocus unit configuration
 *
 * Neoisp extensible params block for the AutoFocus unit configuration.
 * Identified by :c:type:`NEOISP_PARAM_BLK_AF`.
 *
 * @header:	The Neoisp extensible parameters header, see
 *		:c:type:`v4l2_isp_block_header`
 * @cfg:	AutoFocus unit configuration, see
 *		:c:type:`neoisp_af_cfg_s`
 */
struct neoisp_af_cfg_es {
	struct v4l2_isp_block_header header;
	struct neoisp_af_cfg_s cfg;
} __attribute__((aligned(8)));

/**
 * struct neoisp_ee_cfg_es - Neoisp extensible Edge Enhancement unit configuration
 *
 * Neoisp extensible params block for the Edge Enhancement unit configuration.
 * Identified by :c:type:`NEOISP_PARAM_BLK_EE`.
 *
 * @header:	The Neoisp extensible parameters header, see
 *		:c:type:`v4l2_isp_block_header`
 * @cfg:	Edge Enhancement unit configuration, see
 *		:c:type:`neoisp_ee_cfg_s`
 */
struct neoisp_ee_cfg_es {
	struct v4l2_isp_block_header header;
	struct neoisp_ee_cfg_s cfg;
} __attribute__((aligned(8)));

/**
 * struct neoisp_df_cfg_es - Neoisp extensible Direction Filter configuration
 *
 * Neoisp extensible params block for the Direction Filter unit configuration.
 * Identified by :c:type:`NEOISP_PARAM_BLK_DF`.
 *
 * @header:	The Neoisp extensible parameters header, see
 *		:c:type:`v4l2_isp_block_header`
 * @cfg:	Direction Filter configuration, see
 *		:c:type:`neoisp_df_cfg_s`
 */
struct neoisp_df_cfg_es {
	struct v4l2_isp_block_header header;
	struct neoisp_df_cfg_s cfg;
} __attribute__((aligned(8)));

/**
 * struct neoisp_convmed_cfg_es - Neoisp extensible Convmed configuration
 *
 * Neoisp extensible params block for the Color Convolution and Median Filter
 * unit configuration.
 * Identified by :c:type:`NEOISP_PARAM_BLK_CONVMED`.
 *
 * @header:	The Neoisp extensible parameters header, see
 *		:c:type:`v4l2_isp_block_header`
 * @cfg:	Color Convolution and Median Filter unit configuration, see
 *		:c:type:`neoisp_convmed_cfg_s`
 */
struct neoisp_convmed_cfg_es {
	struct v4l2_isp_block_header header;
	struct neoisp_convmed_cfg_s cfg;
} __attribute__((aligned(8)));

/**
 * struct neoisp_cas_cfg_es - Neoisp extensible CAS configuration
 *
 * Neoisp extensible params block for the Color Adaptive Saturation unit configuration.
 * Identified by :c:type:`NEOISP_PARAM_BLK_CAS`.
 *
 * @header:	The Neoisp extensible parameters header, see
 *		:c:type:`v4l2_isp_block_header`
 * @cfg:	Color Adaptive Saturation unit configuration, see
 *		:c:type:`neoisp_cas_cfg_s`
 */
struct neoisp_cas_cfg_es {
	struct v4l2_isp_block_header header;
	struct neoisp_cas_cfg_s cfg;
} __attribute__((aligned(8)));

/**
 * struct neoisp_gcm_cfg_es - Neoisp extensible GCM configuration
 *
 * Neoisp extensible params block for the Gamma Correction matrix unit configuration.
 * Identified by :c:type:`NEOISP_PARAM_BLK_GCM`.
 *
 * @header:	The Neoisp extensible parameters header, see
 *		:c:type:`v4l2_isp_block_header`
 * @cfg:	Gamma Correction Matrix configuration, see
 *		:c:type:`neoisp_gcm_cfg_s`
 */
struct neoisp_gcm_cfg_es {
	struct v4l2_isp_block_header header;
	struct neoisp_gcm_cfg_s cfg;
} __attribute__((aligned(8)));

/**
 * struct neoisp_vignetting_table_mem_params_es - Neoisp extensible Vignetting LUT configuration
 *
 * Neoisp extensible params block for the Vignetting look up table configuration.
 * Identified by :c:type:`NEOISP_PARAM_BLK_VIGNETTING_TABLE`.
 *
 * @header:	The Neoisp extensible parameters header, see
 *		:c:type:`v4l2_isp_block_header`
 * @cfg:	Vignetting LUT configuration, see
 *		:c:type:`neoisp_vignetting_table_mem_params_s`
 */
struct neoisp_vignetting_table_mem_params_es {
	struct v4l2_isp_block_header header;
	struct neoisp_vignetting_table_mem_params_s cfg;
} __attribute__((aligned(8)));

/**
 * struct neoisp_drc_global_tonemap_mem_params_es - Neoisp extensible DRC Global Tonemap LUT
 * configuration
 *
 * Neoisp extensible params block for the DRC Global Tonemap look up table configuration.
 * Identified by :c:type:`NEOISP_PARAM_BLK_DRC_GLOBAL_TONEMAP`.
 *
 * @header:	The Neoisp extensible parameters header, see
 *		:c:type:`v4l2_isp_block_header`
 * @cfg:	DRC Global Tonemap LUT configuration, see
 *		:c:type:`neoisp_drc_global_tonemap_mem_params_s`
 */
struct neoisp_drc_global_tonemap_mem_params_es {
	struct v4l2_isp_block_header header;
	struct neoisp_drc_global_tonemap_mem_params_s cfg;
} __attribute__((aligned(8)));

/**
 * struct neoisp_drc_local_tonemap_mem_params_es - Neoisp extensible DRC Local Tonemap LUT
 * configuration
 *
 * Neoisp extensible params block for the DRC Local Tonemap look up table configuration.
 * Identified by :c:type:`NEOISP_PARAM_BLK_DRC_LOCAL_TONEMAP`.
 *
 * @header:	The Neoisp extensible parameters header, see
 *		:c:type:`v4l2_isp_block_header`
 * @cfg:	DRC Local tonemap LUT configuration, see
 *		:c:type:`neoisp_drc_local_tonemap_mem_params_s`
 */
struct neoisp_drc_local_tonemap_mem_params_es {
	struct v4l2_isp_block_header header;
	struct neoisp_drc_local_tonemap_mem_params_s cfg;
} __attribute__((aligned(8)));

/**
 * define NEOISP_EXT_PARAMS_MAX_SIZE - Maximum size of all Neoisp Parameters
 *
 * Though the parameters for the Neoisp are passed as optional blocks, the
 * driver still needs to know the absolute maximum size so that it can allocate
 * a buffer sized appropriately to accommodate userspace attempting to set all
 * possible parameters in a single frame.
 *
 * Some structs are in this list multiple times. Where that's the case, it just
 * reflects the fact that the same struct can be used with multiple different
 * header types from :c:type:`neoisp_param_block_type_e`.
 */
#define NEOISP_EXT_PARAMS_MAX_SIZE                                \
	(sizeof(struct neoisp_pipe_conf_cfg_es) +                 \
	 sizeof(struct neoisp_head_color_cfg_es) +                \
	 sizeof(struct neoisp_hdr_decompress0_cfg_es) +           \
	 sizeof(struct neoisp_hdr_decompress1_cfg_es) +           \
	 (sizeof(struct neoisp_obwb_cfg_es) * NEO_OBWB_CNT) +     \
	 sizeof(struct neoisp_hdr_merge_cfg_es) +                 \
	 sizeof(struct neoisp_rgbir_cfg_es) +                     \
	 sizeof(struct neoisp_ctemp_cfg_es) +                     \
	 sizeof(struct neoisp_stat_cfg_es) +                      \
	 sizeof(struct neoisp_ir_compress_cfg_es) +               \
	 sizeof(struct neoisp_bnr_cfg_es) +                       \
	 sizeof(struct neoisp_vignetting_ctrl_cfg_es) +           \
	 sizeof(struct neoisp_demosaic_cfg_es) +                  \
	 sizeof(struct neoisp_rgb2yuv_cfg_es) +                   \
	 sizeof(struct neoisp_dr_comp_cfg_es) +                   \
	 sizeof(struct neoisp_nr_cfg_es) +                        \
	 sizeof(struct neoisp_af_cfg_es) +                        \
	 sizeof(struct neoisp_ee_cfg_es) +                        \
	 sizeof(struct neoisp_df_cfg_es) +                        \
	 sizeof(struct neoisp_convmed_cfg_es) +                   \
	 sizeof(struct neoisp_cas_cfg_es) +                       \
	 sizeof(struct neoisp_gcm_cfg_es) +                       \
	 sizeof(struct neoisp_vignetting_table_mem_params_es) +   \
	 sizeof(struct neoisp_drc_global_tonemap_mem_params_es) + \
	 sizeof(struct neoisp_drc_local_tonemap_mem_params_es))

/*
 * Statistics
 */
#define NEO_CTEMP_REG_STATS_CROIS_CNT (10)
#define NEO_AF_REG_STATS_ROIS_CNT (9)

/**
 * struct neoisp_reg_stats_crois_s - Color region of interest
 * @pixcnt_pixcnt:	Pixel count saturates once it reaches all 1s
 * @sumred_sum:		Accumulated red value of total number
 * @sumgreen_sum:	Accumulated green value of total number
 * @sumblue_sum:	Accumulated blue value of total number
 */
struct neoisp_reg_stats_crois_s {
	__u32 pixcnt_pixcnt;
	__u32 sumred_sum;
	__u32 sumgreen_sum;
	__u32 sumblue_sum;
};

/**
 * struct neoisp_ctemp_reg_stats_s - Color Temperature statistics located in registers
 * @cnt_white_white:	Number of white pixels
 * @sumr_sum_l:		Lower 32-bits of accumulated value of the red channel
 * @sumr_sum_h:		Higher 32-bits of accumulated value of the red channel
 * @sumg_sum_l:		Lower 32-bits of accumulated value of the green channel
 * @sumg_sum_h:		Higher 32-bits of accumulated value of the green channel
 * @sumb_sum_l:		Lower 32-bits of accumulated value of the blue channel
 * @sumb_sum_h:		Higher 32-bits of accumulated value of the blue channel
 * @sumrg_sum_l:	Lower 32-bits of accumulated red over green gain
 * @sumrg_sum_h:	Higher 32-bits of accumulated red over green gain
 * @sumbg_sum_l:	Lower 32-bits of accumulated blue over green gain
 * @sumbg_sum_h:	Higher 32-bits of accumulated blue over green gain
 * @crois:		Color regions of interest
 * @gr_gb_cnt_cnt:	Number of counted pixels in the gr vs gb sums
 * @gr_sum_sum:		Sum of counted GR values (msb: 27 bits mantissa, lsb: 5 bits exponent)
 * @gb_sum_sum:		Sum of counted GB values (msb: 27 bits mantissa, lsb: 5 bits exponent)
 * @gr2_sum_sum:	Sum of squared GR values (msb: 27 bits mantissa, lsb: 5 bits exponent)
 * @gb2_sum_sum:	Sum of squared GB values (msb: 27 bits mantissa, lsb: 5 bits exponent)
 * @pad:		Pad two word for alignment
 * @grgb_sum_sum:	Sum of GR*GB values (msb: 27 bits mantissa, lsb: 5 bits exponent)
 */
struct neoisp_ctemp_reg_stats_s {
	__u32 cnt_white_white;
	__u32 sumr_sum_l; /* split low and high to avoid padding and keep aligned with hw */
	__u32 sumr_sum_h;
	__u32 sumg_sum_l;
	__u32 sumg_sum_h;
	__u32 sumb_sum_l;
	__u32 sumb_sum_h;
	__u32 sumrg_sum_l;
	__u32 sumrg_sum_h;
	__u32 sumbg_sum_l;
	__u32 sumbg_sum_h;
	struct neoisp_reg_stats_crois_s crois[NEO_CTEMP_REG_STATS_CROIS_CNT];
	__u32 gr_gb_cnt_cnt;
	__u32 gr_sum_sum;
	__u32 gb_sum_sum;
	__u32 gr2_sum_sum;
	__u32 gb2_sum_sum;
	__u32 pad[2];
	__u32 grgb_sum_sum;
};

/**
 * struct neoisp_drc_reg_stats_s - Dynamic Range Compression statistics
 * @groi0_sum_val:	Sum of pixels within the global region of interest 0
 * @groi1_sum_val:	Sum of pixels within the global region of interest 1
 */
struct neoisp_drc_reg_stats_s {
	__u32  groi0_sum_val;
	__u32  groi1_sum_val;
};

/**
 * struct neoisp_af_reg_stats_sums_s - common Auto Focus sum registers pair
 * @sum0:	Provides the 32-bit accumulated value for filter 0 for a ROI
 * @sum1:	Provides the 32-bit accumulated value for filter 1 for a ROI
 */
struct neoisp_af_reg_stats_sums_s {
	__u32 sum0;
	__u32 sum1;
};

/**
 * struct neoisp_af_reg_stats_s - Auto Focus statistics
 * @rois:	Array of filters 0 and 1 sums for each ROI
 */
struct neoisp_af_reg_stats_s {
	struct neoisp_af_reg_stats_sums_s rois[NEO_AF_REG_STATS_ROIS_CNT];
};

/**
 * struct neoisp_bnr_reg_stats_s - Bayer Noise Reduction statistics
 * @edge_stat_edge_pixels:	Number of edge pixels that are above the L threshold (u24)
 * @edges_stat_edge_pixels:	Number of edge pixels that are above the S threshold (u24)
 */
struct neoisp_bnr_reg_stats_s {
	__u32 edge_stat_edge_pixels;
	__u32 edges_stat_edge_pixels;
};

/**
 * struct neoisp_nr_reg_stats_s - Noise Reduction statistics
 * @edgecnt_val:	Number of filtered pixels for respective camera context (u24)
 */
struct neoisp_nr_reg_stats_s {
	__u32 edgecnt_val;
};

/**
 * struct neoisp_ee_reg_stats_s - Edge enhancement statistics
 * @edgecnt_val:	Number of filtered pixels for respective camera context (u24)
 */
struct neoisp_ee_reg_stats_s {
	__u32 edgecnt_val;
};

/**
 * struct neoisp_df_reg_stats_s - Direction Filter statistics
 * @edgecnt_val:	Number of filtered pixels for respective camera context (u24)
 */
struct neoisp_df_reg_stats_s {
	__u32 edgecnt_val;
};

/**
 * struct neoisp_ctemp_mem_stats_s - Color Temperature statistics located in memory
 * @ctemp_r_sum:	Array of red sums
 * @ctemp_g_sum:	Array of green sums
 * @ctemp_b_sum:	Array of blue sums
 * @ctemp_pix_cnt:	Array of pixel counts
 */
struct neoisp_ctemp_mem_stats_s {
	__u32 ctemp_r_sum[NEO_CTEMP_R_SUM_CNT];
	__u32 ctemp_g_sum[NEO_CTEMP_G_SUM_CNT];
	__u32 ctemp_b_sum[NEO_CTEMP_B_SUM_CNT];
	__u16 ctemp_pix_cnt[NEO_CTEMP_PIX_CNT_CNT];
};

/**
 * struct neoisp_rgbir_mem_stats_s - RGBIR statistics located in memory
 * @rgbir_hist:		Rgbir histograms
 */
struct neoisp_rgbir_mem_stats_s {
	__u32 rgbir_hist[NEO_RGBIR_HIST_CNT];
};

/**
 * struct neoisp_hist_mem_stats_s - Histograms located in memory
 * @hist_stat:		Array of histograms and statistics
 */
struct neoisp_hist_mem_stats_s {
	__u32 hist_stat[NEO_HIST_STAT_CNT];
};

/**
 * struct neoisp_drc_mem_stats_s - DRC statistics located in memory
 * @drc_local_sum:		DRC local sums array
 * @drc_global_hist_roi0:	DRC global histogram fir region of interest 0
 * @drc_global_hist_roi1:	DRC global histogram fir region of interest 1
 */
struct neoisp_drc_mem_stats_s {
	__u32 drc_local_sum[NEO_DRC_LOCAL_SUM_CNT];
	__u32 drc_global_hist_roi0[NEO_DRC_GLOBAL_HIST_ROI_CNT];
	__u32 drc_global_hist_roi1[NEO_DRC_GLOBAL_HIST_ROI_CNT];
};

/**
 * enum neoisp_stats_block_type_e - Enumeration of Neoisp statistics blocks
 *
 * This enumeration defines the types of Neoisp statistics block. Each entry
 * contains statistics specific to a processing block of the Neoisp. The block
 * type allows the driver to correctly interpret the statistics block data.
 *
 * It is driver responsability to correctly set the type of each statistics block.
 *
 * @NEOISP_STATS_BLK_RCTEMP: Color Temperature statistics registers
 * @NEOISP_STATS_BLK_RDRC: Dynamic Range Compression statistics registers
 * @NEOISP_STATS_BLK_RAF: Auto Focus statistics registers
 * @NEOISP_STATS_BLK_RBNR: Bayer Noise Reduction statistics registers
 * @NEOISP_STATS_BLK_RNR: Noise Reduction statistics registers
 * @NEOISP_STATS_BLK_REE: Edge enhancement statistics
 * @NEOISP_STATS_BLK_RDF: Direction Filter statistics registers
 * @NEOISP_STATS_BLK_MCTEMP: Color Temperature statistics memories
 * @NEOISP_STATS_BLK_MRGBIR: RGBIR statistics memories
 * @NEOISP_STATS_BLK_MHIST: Histograms statistics memories
 * @NEOISP_STATS_BLK_MDRC: DRC statistics memories
 */
enum neoisp_stats_block_type_e {
	NEOISP_STATS_BLK_RCTEMP,
	NEOISP_STATS_BLK_RDRC,
	NEOISP_STATS_BLK_RAF,
	NEOISP_STATS_BLK_RBNR,
	NEOISP_STATS_BLK_RNR,
	NEOISP_STATS_BLK_REE,
	NEOISP_STATS_BLK_RDF,
	NEOISP_STATS_BLK_MCTEMP,
	NEOISP_STATS_BLK_MRGBIR,
	NEOISP_STATS_BLK_MHIST,
	NEOISP_STATS_BLK_MDRC,
};

/**
 * struct neoisp_ctemp_reg_stats_es - Neoisp extensible pipeline configuration
 *
 * Neoisp extensible pipelines alignment configuration block.
 * Identified by :c:type:`NEOISP_STATS_BLK_RCTEMP`.
 *
 * @header:	The Neoisp extensible statistics header, see
 *		:c:type:`v4l2_isp_block_header`
 * @stat:	Pipeline configuration, see
 *		:c:type:`neoisp_ctemp_reg_stats_s`
 */
struct neoisp_ctemp_reg_stats_es {
	struct v4l2_isp_block_header header;
	struct neoisp_ctemp_reg_stats_s stat;
} __attribute__((aligned(8)));

/**
 * struct neoisp_drc_reg_stats_es - Neoisp extensible pipeline configuration
 *
 * Neoisp extensible pipelines alignment configuration block.
 * Identified by :c:type:`NEOISP_STATS_BLK_RDRC`.
 *
 * @header:	The Neoisp extensible statistics header, see
 *		:c:type:`v4l2_isp_block_header`
 * @stat:	Pipeline configuration, see
 *		:c:type:`neoisp_drc_reg_stats_s`
 */
struct neoisp_drc_reg_stats_es {
	struct v4l2_isp_block_header header;
	struct neoisp_drc_reg_stats_s stat;
} __attribute__((aligned(8)));

/**
 * struct neoisp_af_reg_stats_es - Neoisp extensible pipeline configuration
 *
 * Neoisp extensible pipelines alignment configuration block.
 * Identified by :c:type:`NEOISP_STATS_BLK_RAF`.
 *
 * @header:	The Neoisp extensible statistics header, see
 *		:c:type:`v4l2_isp_block_header`
 * @stat:	Pipeline configuration, see
 *		:c:type:`neoisp_af_reg_stats_s`
 */
struct neoisp_af_reg_stats_es {
	struct v4l2_isp_block_header header;
	struct neoisp_af_reg_stats_s stat;
} __attribute__((aligned(8)));

/**
 * struct neoisp_bnr_reg_stats_es - Neoisp extensible pipeline configuration
 *
 * Neoisp extensible pipelines alignment configuration block.
 * Identified by :c:type:`NEOISP_STATS_BLK_RBNR`.
 *
 * @header:	The Neoisp extensible statistics header, see
 *		:c:type:`v4l2_isp_block_header`
 * @stat:	Pipeline configuration, see
 *		:c:type:`neoisp_bnr_reg_stats_s`
 */
struct neoisp_bnr_reg_stats_es {
	struct v4l2_isp_block_header header;
	struct neoisp_bnr_reg_stats_s stat;
} __attribute__((aligned(8)));

/**
 * struct neoisp_nr_reg_stats_es - Neoisp extensible pipeline configuration
 *
 * Neoisp extensible pipelines alignment configuration block.
 * Identified by :c:type:`NEOISP_STATS_BLK_RNR`.
 *
 * @header:	The Neoisp extensible statistics header, see
 *		:c:type:`v4l2_isp_block_header`
 * @stat:	Pipeline configuration, see
 *		:c:type:`neoisp_nr_reg_stats_s`
 */
struct neoisp_nr_reg_stats_es {
	struct v4l2_isp_block_header header;
	struct neoisp_nr_reg_stats_s stat;
} __attribute__((aligned(8)));

/**
 * struct neoisp_ee_reg_stats_es - Neoisp extensible pipeline configuration
 *
 * Neoisp extensible pipelines alignment configuration block.
 * Identified by :c:type:`NEOISP_STATS_BLK_REE`.
 *
 * @header:	The Neoisp extensible statistics header, see
 *		:c:type:`v4l2_isp_block_header`
 * @stat:	Pipeline configuration, see
 *		:c:type:`neoisp_ee_reg_stats_s`
 */
struct neoisp_ee_reg_stats_es {
	struct v4l2_isp_block_header header;
	struct neoisp_ee_reg_stats_s stat;
} __attribute__((aligned(8)));

/**
 * struct neoisp_df_reg_stats_es - Neoisp extensible pipeline configuration
 *
 * Neoisp extensible pipelines alignment configuration block.
 * Identified by :c:type:`NEOISP_STATS_BLK_RDF`.
 *
 * @header:	The Neoisp extensible statistics header, see
 *		:c:type:`v4l2_isp_block_header`
 * @stat:	Pipeline configuration, see
 *		:c:type:`neoisp_df_reg_stats_s`
 */
struct neoisp_df_reg_stats_es {
	struct v4l2_isp_block_header header;
	struct neoisp_df_reg_stats_s stat;
} __attribute__((aligned(8)));

/**
 * struct neoisp_ctemp_mem_stats_es - Neoisp extensible pipeline configuration
 *
 * Neoisp extensible pipelines alignment configuration block.
 * Identified by :c:type:`NEOISP_STATS_BLK_MCTEMP`.
 *
 * @header:	The Neoisp extensible statistics header, see
 *		:c:type:`v4l2_isp_block_header`
 * @stat:	Pipeline configuration, see
 *		:c:type:`neoisp_ctemp_mem_stats_s`
 */
struct neoisp_ctemp_mem_stats_es {
	struct v4l2_isp_block_header header;
	struct neoisp_ctemp_mem_stats_s stat;
} __attribute__((aligned(8)));

/**
 * struct neoisp_rgbir_mem_stats_es - Neoisp extensible pipeline configuration
 *
 * Neoisp extensible pipelines alignment configuration block.
 * Identified by :c:type:`NEOISP_STATS_BLK_MRGBIR`.
 *
 * @header:	The Neoisp extensible statistics header, see
 *		:c:type:`v4l2_isp_block_header`
 * @stat:	Pipeline configuration, see
 *		:c:type:`neoisp_rgbir_mem_stats_s`
 */
struct neoisp_rgbir_mem_stats_es {
	struct v4l2_isp_block_header header;
	struct neoisp_rgbir_mem_stats_s stat;
} __attribute__((aligned(8)));

/**
 * struct neoisp_hist_mem_stats_es - Neoisp extensible pipeline configuration
 *
 * Neoisp extensible pipelines alignment configuration block.
 * Identified by :c:type:`NEOISP_STATS_BLK_MHIST`.
 *
 * @header:	The Neoisp extensible statistics header, see
 *		:c:type:`v4l2_isp_block_header`
 * @stat:	Pipeline configuration, see
 *		:c:type:`neoisp_hist_mem_stats_s`
 */
struct neoisp_hist_mem_stats_es {
	struct v4l2_isp_block_header header;
	struct neoisp_hist_mem_stats_s stat;
} __attribute__((aligned(8)));

/**
 * struct neoisp_drc_mem_stats_es - Neoisp extensible pipeline configuration
 *
 * Neoisp extensible pipelines alignment configuration block.
 * Identified by :c:type:`NEOISP_STATS_BLK_MDRC`.
 *
 * @header:	The Neoisp extensible statistics header, see
 *		:c:type:`v4l2_isp_block_header`
 * @stat:	Pipeline configuration, see
 *		:c:type:`neoisp_drc_mem_stats_s`
 */
struct neoisp_drc_mem_stats_es {
	struct v4l2_isp_block_header header;
	struct neoisp_drc_mem_stats_s stat;
} __attribute__((aligned(8)));

/**
 * define NEOISP_EXT_STATS_MAX_SIZE - Maximum size of all Neoisp Statistics
 *
 * Though the statistics of the Neoisp are passed as optional blocks, the
 * userspace still needs to know the absolute maximum size so that it can
 * allocate a buffer sized appropriately to accommodate driver attempting to
 * set all possible statistics in a single frame.
 */
#define NEOISP_EXT_STATS_MAX_SIZE                   \
	(sizeof(struct neoisp_ctemp_reg_stats_es) + \
	 sizeof(struct neoisp_drc_reg_stats_es) +   \
	 sizeof(struct neoisp_af_reg_stats_es) +    \
	 sizeof(struct neoisp_bnr_reg_stats_es) +   \
	 sizeof(struct neoisp_nr_reg_stats_es) +    \
	 sizeof(struct neoisp_ee_reg_stats_es) +    \
	 sizeof(struct neoisp_df_reg_stats_es) +    \
	 sizeof(struct neoisp_ctemp_mem_stats_es) + \
	 sizeof(struct neoisp_rgbir_mem_stats_es) + \
	 sizeof(struct neoisp_hist_mem_stats_es) +  \
	 sizeof(struct neoisp_drc_mem_stats_es))

#endif /* __UAPI_NXP_NEOISP_H */
