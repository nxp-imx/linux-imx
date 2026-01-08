/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Coda series multi-standard codec IP - coda backend definitions
 *
 * Copyright (C) 2024-2026 CHIPS&MEDIA INC
 */

#ifndef __CODA_HW_H__
#define __CODA_HW_H__

bool coda_hw_is_init(struct vpu_device *vpu);
int coda_hw_init(struct vpu_device *vpu, u8 *firmware, size_t size);
int coda_hw_sleep_wake(struct vpu_device *vpu, bool sleep);
int coda_hw_reset(struct vpu_device *vpu, enum coda_sw_reset_mode reset_mode);
int coda_hw_get_version(struct vpu_device *vpu, u32 *version, u32 *revision);
int coda_hw_build_up_enc_param(struct vpu_instance *inst,
			       struct coda_enc_open_param *open_param);
int coda_hw_enc_init_seq(struct vpu_instance *inst);
int coda_hw_enc_get_seq_info(struct vpu_instance *inst);
int coda_hw_enc_register_frame_buffer(struct vpu_instance *inst, struct coda_frame_buffer *fb_arr);
int coda_hw_enc_encode(struct vpu_instance *inst, struct coda_enc_param *option, u32 *fail_res);
int coda_hw_enc_get_result(struct vpu_instance *inst, struct coda_enc_output_info *info);
int coda_hw_enc_finish_seq(struct vpu_instance *inst, u32 *fail_res);

#endif
