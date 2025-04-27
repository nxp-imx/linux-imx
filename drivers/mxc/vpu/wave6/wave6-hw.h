/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Wave6 series multi-standard codec IP - wave6 backend interface
 *
 * Copyright (C) 2025 CHIPS&MEDIA INC
 */

#ifndef __WAVE6_HW_H__
#define __WAVE6_HW_H__

#define STD_AVC		0
#define STD_HEVC	12

enum product_id {
	PRODUCT_ID_617,
	PRODUCT_ID_627,
	PRODUCT_ID_637,
	PRODUCT_ID_NONE,
};

#define BSOPTION_ENABLE_EXPLICIT_END	BIT(0)
#define NON_VCL_PARAM_ENCODE_VUI	BIT(1)

#define DECODE_ALL_TEMPORAL_LAYERS	0
#define DECODE_ALL_SPATIAL_LAYERS	0

#define REGISTER_DISPLAY_BUFFER	1
#define DEFAULT_PIXEL_ORDER	1

#define WTL_RIGHT_JUSTIFIED	0
#define WTL_LEFT_JUSTIFIED	1
#define WTL_PIXEL_8BIT		0
#define WTL_PIXEL_16BIT		1
#define WTL_PIXEL_32BIT		2

#define MAX_CSC_COEFF_NUM	4

bool wave6_vpu_is_init(struct vpu_device *vpu_dev);
void wave6_vpu_check_state(struct vpu_device *vpu_dev);
int wave6_vpu_get_version(struct vpu_device *vpu_dev, u32 *version_info, uint32_t *revision);
void wave6_vpu_enable_interrupt(struct vpu_device *vpu_dev);
int wave6_vpu_build_up_dec_param(struct vpu_instance *inst, struct dec_open_param *param);

void wave6_vpu_dec_set_bitstream_end(struct vpu_instance *inst, bool eos);
int wave6_vpu_dec_register_frame_buffer(struct vpu_instance *inst,
					struct frame_buffer *fb_arr, enum tiled_map_type map_type,
					uint32_t count);
int wave6_vpu_dec_register_display_buffer(struct vpu_instance *inst, struct frame_buffer fb);
int wave6_vpu_dec_init_seq(struct vpu_instance *inst);
int wave6_vpu_dec_get_seq_info(struct vpu_instance *inst, struct dec_initial_info *info);
int wave6_vpu_decode(struct vpu_instance *inst, struct dec_param *option, u32 *fail_res);
int wave6_vpu_dec_get_result(struct vpu_instance *inst, struct dec_output_info *result);
int wave6_vpu_dec_fini_seq(struct vpu_instance *inst, u32 *fail_res);
dma_addr_t wave6_vpu_dec_get_rd_ptr(struct vpu_instance *inst);
int wave6_vpu_dec_flush(struct vpu_instance *inst);

int wave6_vpu_build_up_enc_param(struct device *dev, struct vpu_instance *inst,
				 struct enc_open_param *param);
int wave6_vpu_enc_init_seq(struct vpu_instance *inst);
int wave6_vpu_enc_change_seq(struct vpu_instance *inst, bool *changed);
int wave6_vpu_enc_get_seq_info(struct vpu_instance *inst, struct enc_initial_info *info);
int wave6_vpu_enc_register_frame_buffer(struct vpu_instance *inst,
					struct frame_buffer *fb_arr);
int wave6_vpu_encode(struct vpu_instance *inst, struct enc_param *option, u32 *fail_res);
int wave6_vpu_enc_get_result(struct vpu_instance *inst, struct enc_output_info *result);
int wave6_vpu_enc_fini_seq(struct vpu_instance *inst, u32 *fail_res);
int wave6_vpu_enc_check_open_param(struct vpu_instance *inst, struct enc_open_param *pop);

#endif /* __WAVE6_HW_H__ */
