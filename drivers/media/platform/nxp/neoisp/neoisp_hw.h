/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * NEOISP hardware structures definition
 *
 * Copyright 2023-2026 NXP
 */

#ifndef __NXP_NEOISP_HW_H
#define __NXP_NEOISP_HW_H

#include "neoisp_regs.h"

struct neoisp_pipe_conf_s {
	u32 reset;
	u32 bus_param;
	u32 xfer_dis;
	u32 unusedw0[1];
	u32 csi_ctrl;
	u32 frame_num;
	u32 shd_ctrl;
	u32 reg_shd_cmd;
	u32 trig_cam0;
	u32 int_en;
	u32 int_stat;
	u32 csi_stat;
	u32 img_conf;
	u32 img_size;
	u32 unusedw1[1];
	u32 img0_in_addr;
	u32 img1_in_addr;
	u32 outch0_addr;
	u32 outch1_addr;
	u32 outir_addr;
	u32 img0_in_ls;
	u32 img1_in_ls;
	u32 outch0_ls;
	u32 outch1_ls;
	u32 outir_ls;
	u32 skip_ctrl;
}; /* 26 words */

struct neoisp_hc_s {
	u32 ctrl;
};

struct neoisp_hdr_decompress0_s {
	u32 ctrl;
	u32 knee_point1;
	u32 knee_point2;
	u32 knee_point3;
	u32 knee_point4;
	u32 knee_offset0;
	u32 knee_offset1;
	u32 knee_offset2;
	u32 knee_offset3;
	u32 knee_offset4;
	u32 knee_ratio01;
	u32 knee_ratio23;
	u32 knee_ratio4;
	u32 knee_npoint0;
	u32 knee_npoint1;
	u32 knee_npoint2;
	u32 knee_npoint3;
	u32 knee_npoint4;
};

struct neoisp_hdr_decompress1_s {
	u32 ctrl;
	u32 knee_point1;
	u32 knee_point2;
	u32 knee_point3;
	u32 knee_point4;
	u32 knee_offset0;
	u32 knee_offset1;
	u32 knee_offset2;
	u32 knee_offset3;
	u32 knee_offset4;
	u32 knee_ratio01;
	u32 knee_ratio23;
	u32 knee_ratio4;
	u32 knee_npoint0;
	u32 knee_npoint1;
	u32 knee_npoint2;
	u32 knee_npoint3;
	u32 knee_npoint4;
};

struct neoisp_obwb_s {
	u32 ctrl;
	u32 r_ctrl;
	u32 gr_ctrl;
	u32 gb_ctrl;
	u32 b_ctrl;
};

struct neoisp_hdr_merge_s {
	u32 ctrl;
	u32 gain_offset;
	u32 gain_scale;
	u32 gain_shift;
	u32 luma_th;
	u32 luma_scale;
	u32 downscale;
	u32 upscale;
	u32 post_scale;
	u32 s_gain_offset;
	u32 s_gain_scale;
	u32 s_gain_shift;
	u32 s_luma_th;
	u32 s_luma_scale;
	u32 s_downscale;
	u32 s_upscale;
	u32 s_post_scale;
	u32 s_line_num;
};

struct neoisp_ctemp_s {
	u32 ctrl;
	u32 roi_pos;
	u32 roi_size;
	u32 redgain;
	u32 bluegain;
	u32 point1;
	u32 point2;
	u32 hoffset;
	u32 voffset;
	u32 point1_slope;
	u32 point2_slope;
	u32 luma_th;
	u32 csc_mat0;
	u32 csc_mat1;
	u32 csc_mat2;
	u32 csc_mat3;
	u32 csc_mat4;
	u32 r_gr_offset;
	u32 gb_b_offset;
	u32 cnt_white;
	u32 sumrl;
	u32 sumrh;
	u32 sumgl;
	u32 sumgh;
	u32 sumbl;
	u32 sumbh;
	u32 sumrgl;
	u32 sumrgh;
	u32 sumbgl;
	u32 sumbgh;
	u32 unused0[2];
	u32 stat_blk_size0;
	u32 unused1[1];
	u32 stat_curr_blk_y0;
	u32 unused2[1];
	u32 croi0_pos;
	u32 unused3[1];
	u32 croi0_pixcnt;
	u32 croi0_sumred;
	u32 croi0_sumgreen;
	u32 croi0_sumblue;
	u32 croi1_pos;
	u32 unused4[1];
	u32 croi1_pixcnt;
	u32 croi1_sumred;
	u32 croi1_sumgreen;
	u32 croi1_sumblue;
	u32 croi2_pos;
	u32 unused5[1];
	u32 croi2_pixcnt;
	u32 croi2_sumred;
	u32 croi2_sumgreen;
	u32 croi2_sumblue;
	u32 croi3_pos;
	u32 unused6[1];
	u32 croi3_pixcnt;
	u32 croi3_sumred;
	u32 croi3_sumgreen;
	u32 croi3_sumblue;
	u32 croi4_pos;
	u32 unused7[1];
	u32 croi4_pixcnt;
	u32 croi4_sumred;
	u32 croi4_sumgreen;
	u32 croi4_sumblue;
	u32 croi5_pos;
	u32 unused8[1];
	u32 croi5_pixcnt;
	u32 croi5_sumred;
	u32 croi5_sumgreen;
	u32 croi5_sumblue;
	u32 croi6_pos;
	u32 unused9[1];
	u32 croi6_pixcnt;
	u32 croi6_sumred;
	u32 croi6_sumgreen;
	u32 croi6_sumblue;
	u32 croi7_pos;
	u32 unused10[1];
	u32 croi7_pixcnt;
	u32 croi7_sumred;
	u32 croi7_sumgreen;
	u32 croi7_sumblue;
	u32 croi8_pos;
	u32 unused11[1];
	u32 croi8_pixcnt;
	u32 croi8_sumred;
	u32 croi8_sumgreen;
	u32 croi8_sumblue;
	u32 croi9_pos;
	u32 unused12[1];
	u32 croi9_pixcnt;
	u32 croi9_sumred;
	u32 croi9_sumgreen;
	u32 croi9_sumblue;
	u32 unused13[1];
	u32 gr_avg_in;
	u32 gb_avg_in;
	u32 gr_gb_cnt;
	s32 gr_sum;
	s32 gb_sum;
	u32 gr2_sum;
	u32 gb2_sum;
	s32 grgb_sum;
};

struct neoisp_rgbir_s {
	u32 ctrl;
	u32 ccm0;
	u32 ccm1;
	u32 ccm2;
	u32 ccm0_th;
	u32 ccm1_th;
	u32 ccm2_th;
	u32 unused0[1];
	u32 roi0_pos;
	u32 roi0_size;
	u32 roi1_pos;
	u32 roi1_size;
	u32 hist0_ctrl;
	u32 hist0_scale;
	u32 hist1_ctrl;
	u32 hist1_scale;
};

struct neoisp_stat_s {
	u32 roi0_pos;
	u32 roi0_size;
	u32 roi1_pos;
	u32 roi1_size;
	u32 unused0[4];
	u32 hist0_ctrl;
	u32 hist0_scale;
	u32 hist1_ctrl;
	u32 hist1_scale;
	u32 hist2_ctrl;
	u32 hist2_scale;
	u32 hist3_ctrl;
	u32 hist3_scale;
};

struct neoisp_ir_compress_s {
	u32 ctrl;
	u32 knee_point1;
	u32 knee_point2;
	u32 knee_point3;
	u32 knee_point4;
	u32 knee_offset0;
	u32 knee_offset1;
	u32 knee_offset2;
	u32 knee_offset3;
	u32 knee_offset4;
	u32 knee_ratio01;
	u32 knee_ratio23;
	u32 knee_ratio4;
	u32 knee_npoint0;
	u32 knee_npoint1;
	u32 knee_npoint2;
	u32 knee_npoint3;
	u32 knee_npoint4;
};

struct neoisp_bnr_s {
	u32 ctrl;
	u32 ypeak;
	u32 yedge_th0;
	u32 yedge_scale;
	u32 yedges_th0;
	u32 yedges_scale;
	u32 yedgea_th0;
	u32 yedgea_scale;
	u32 yluma_x_th0;
	u32 yluma_y_th;
	u32 yluma_scale;
	u32 yalpha_gain;
	u32 cpeak;
	u32 cedge_th0;
	u32 cedge_scale;
	u32 cedges_th0;
	u32 cedges_scale;
	u32 cedgea_th0;
	u32 cedgea_scale;
	u32 cluma_x_th0;
	u32 cluma_y_th;
	u32 cluma_scale;
	u32 calpha_gain;
	u32 edge_stat;
	u32 edges_stat;
	u32 stretch;
};

struct neoisp_vignetting_ctrl_s {
	u32 ctrl;
	u32 blk_conf;
	u32 blk_size;
	u32 blk_stepy;
	u32 blk_stepx;
	u32 punused0[3];
	u32 blk_c_line;
	u32 blk_c_row;
	u32 blk_c_fracy;
};

struct neoisp_idbg1_s {
	u32 line_num_t;
	u32 curr_line_num_t;
	u32 ima_t;
	u32 imd;
	u32 done_stat_t;
};

struct neoisp_demosaic_s {
	u32 ctrl;
	u32 activity_ctl;
	u32 dynamics_ctl0;
	u32 dynamics_ctl2;
};

struct neoisp_rgb2yuv_s {
	u32 gain_ctrl;
	u32 mat0;
	u32 mat1;
	u32 mat2;
	u32 mat3;
	u32 mat4;
	u32 mat5;
	u32 unused0[1];
	u32 offset0;
	u32 offset1;
	u32 offset2;
};

struct neoisp_dr_comp_s {
	u32 roi0_pos;
	u32 roi0_size;
	u32 roi1_pos;
	u32 roi1_size;
	u32 groi_sum_shift;
	u32 gbl_gain;
	u32 unused0[2];
	u32 lcl_blk_size;
	u32 lcl_stretch;
	u32 lcl_blk_stepy;
	u32 lcl_blk_stepx;
	u32 lcl_sum_shift;
	u32 alpha;
	u32 unused1[2];
	u32 groi0_sum;
	u32 groi1_sum;
	u32 unused2[2];
	u32 stat_blk_y;
	u32 curr_yfract;
};

struct neoisp_nr_s {
	u32 ctrl;
	u32 blend_scale;
	u32 blend_th0;
	u32 punused0[1];
	u32 edgecnt;
};

struct neoisp_df_s {
	u32 ctrl;
	u32 th_scale;
	u32 blend_shift;
	u32 blend_th0;
	u32 edgecnt;
};

struct neoisp_ee_s {
	u32 ctrl;
	u32 coring;
	u32 clip;
	u32 maskgain;
	u32 edgecnt;
};

struct neoisp_convmed_s {
	u32 ctrl;
};

struct neoisp_cas_s {
	u32 unused0[1];
	u32 gain;
	u32 corr;
	u32 offset;
};

struct neoisp_packetizer_s {
	u32 ch0_ctrl;
	u32 ch12_ctrl;
	u32 pack_ctrl;
};

struct neoisp_gcm_s {
	u32 imat0;
	u32 imat1;
	u32 punused0[1];
	u32 imat2;
	u32 imat3;
	u32 punused1[1];
	u32 imat4;
	u32 imat5;
	u32 ioffset0;
	u32 ioffset1;
	u32 ioffset2;
	u32 punused2[1];
	u32 omat0;
	u32 omat1;
	u32 omat2;
	u32 omat3;
	u32 omat4;
	u32 omat5;
	u32 ooffset0;
	u32 ooffset1;
	u32 ooffset2;
	u32 punused3[3];
	u32 gamma0;
	u32 gamma1;
	u32 gamma2;
	u32 blklvl0_ctrl;
	u32 blklvl1_ctrl;
	u32 blklvl2_ctrl;
	u32 lowth_ctrl01;
	u32 lowth_ctrl2;
	u32 mat_confg;
};

struct neoisp_autofocus_s {
	u32 roi0_pos;
	u32 roi0_size;
	u32 roi1_pos;
	u32 roi1_size;
	u32 roi2_pos;
	u32 roi2_size;
	u32 roi3_pos;
	u32 roi3_size;
	u32 roi4_pos;
	u32 roi4_size;
	u32 roi5_pos;
	u32 roi5_size;
	u32 roi6_pos;
	u32 roi6_size;
	u32 roi7_pos;
	u32 roi7_size;
	u32 roi8_pos;
	u32 roi8_size;
	u32 unused0[2];
	u32 fil0_coeffs0;
	u32 fil0_coeffs1;
	u32 fil0_coeffs2;
	u32 fil0_shift;
	u32 fil1_coeffs0;
	u32 fil1_coeffs1;
	u32 fil1_coeffs2;
	u32 fil1_shift;
	u32 roi0_sum0_cam0;
	u32 roi0_sum1_cam0;
	u32 roi1_sum0_cam0;
	u32 roi1_sum1_cam0;
	u32 roi2_sum0_cam0;
	u32 roi2_sum1_cam0;
	u32 roi3_sum0_cam0;
	u32 roi3_sum1_cam0;
	u32 roi4_sum0_cam0;
	u32 roi4_sum1_cam0;
	u32 roi5_sum0_cam0;
	u32 roi5_sum1_cam0;
	u32 roi6_sum0_cam0;
	u32 roi6_sum1_cam0;
	u32 roi7_sum0_cam0;
	u32 roi7_sum1_cam0;
	u32 roi8_sum0_cam0;
	u32 roi8_sum1_cam0;
};

struct neoisp_idbg2_s {
	u32 line_num;
	u32 curr_line_num;
	u32 ima;
	u32 imd;
	u32 done_stat;
};

struct neoisp_hw_s {
	struct neoisp_pipe_conf_s pipe_conf;
	u32 unused0[22];
	struct neoisp_hc_s hc;
	u32 unused1[15];
	struct neoisp_hdr_decompress0_s hdr_decompress0;
	u32 unused2[14];
	struct neoisp_hdr_decompress1_s hdr_decompress1;
	u32 unused3[14];
	struct neoisp_obwb_s obwb0;
	u32 unused4[11];
	struct neoisp_obwb_s obwb1;
	u32 unused5[11];
	struct neoisp_obwb_s obwb2;
	u32 unused6[27];
	struct neoisp_hdr_merge_s hdr_merge;
	u32 unused7[46];
	struct neoisp_ctemp_s ctemp;
	u32 unused8[23];
	struct neoisp_rgbir_s rgbir;
	u32 unused9[48];
	struct neoisp_stat_s stat;
	u32 unused10[16];
	struct neoisp_ir_compress_s ir_compress;
	u32 unused11[14];
	struct neoisp_bnr_s bnr;
	u32 unused12[38];
	struct neoisp_vignetting_ctrl_s vignetting_ctrl;
	u32 unused13[421];
	struct neoisp_idbg1_s idbg1;
	u32 unused14[107];
	struct neoisp_demosaic_s demosaic;
	u32 unused15[12];
	struct neoisp_rgb2yuv_s rgb2yuv;
	u32 unused16[69];
	struct neoisp_dr_comp_s drc;
	u32 unused17[42];
	struct neoisp_nr_s nr;
	u32 unused18[11];
	struct neoisp_df_s df;
	u32 unused19[11];
	struct neoisp_ee_s ee;
	u32 unused20[11];
	struct neoisp_convmed_s convmed;
	u32 unused21[15];
	struct neoisp_cas_s cas;
	u32 unused22[28];
	struct neoisp_packetizer_s packetizer;
	u32 unused23[29];
	struct neoisp_gcm_s gcm;
	u32 unused24[31];
	struct neoisp_autofocus_s autofocus;
};

#endif /* __NXP_NEOISP_HW_H */
