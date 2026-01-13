// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Coda series multi-standard codec IP - helper functions
 *
 * Copyright (C) 2024-2026 CHIPS&MEDIA INC
 */

#include "coda-vpuapi.h"
#include "coda-hw.h"
#include "coda-helper.h"

static int coda_vpuapi_initialize_vpu(struct device *dev, u8 *code, size_t size)
{
	struct vpu_device *vpu = dev_get_drvdata(dev);
	int ret;

	ret = mutex_lock_interruptible(&vpu->hw_lock);
	if (ret)
		return ret;

	if (coda_hw_is_init(vpu))
		goto mutex_unlock;

	ret = coda_hw_reset(vpu, SW_RESET_ON_BOOT);
	if (ret)
		goto mutex_unlock;

	ret = coda_hw_init(vpu, code, size);

mutex_unlock:
	mutex_unlock(&vpu->hw_lock);
	return ret;
}

int coda_vpuapi_init_with_bitcode(struct device *dev, u8 *code, size_t size)
{
	if (!code || size == 0)
		return -EINVAL;

	return coda_vpuapi_initialize_vpu(dev, code, size);
}

int coda_vpuapi_sleep_wake(struct device *dev, bool sleep)
{
	struct vpu_device *vpu = dev_get_drvdata(dev);
	int ret;

	ret = mutex_lock_interruptible(&vpu->hw_lock);
	if (ret)
		return ret;

	ret = coda_hw_sleep_wake(vpu, sleep);

	mutex_unlock(&vpu->hw_lock);
	return ret;
}

int coda_vpuapi_get_version_info(struct device *dev, u32 *version, u32 *revision)
{
	struct vpu_device *vpu = dev_get_drvdata(dev);
	int ret;

	ret = mutex_lock_interruptible(&vpu->hw_lock);
	if (ret)
		return ret;

	if (!coda_hw_is_init(vpu)) {
		ret = -EINVAL;
		goto err_out;
	}

	ret = coda_hw_get_version(vpu, version, revision);

err_out:
	mutex_unlock(&vpu->hw_lock);
	return ret;
}

static int coda_vpuapi_enc_check_open_param(struct vpu_instance *inst,
					    struct coda_enc_open_param *open_param)
{
	if (!open_param)
		return -EINVAL;

	if (open_param->pic_width % CODA_ENC_CODEC_PIC_STEP ||
	    open_param->pic_height % CODA_ENC_CODEC_PIC_STEP)
		return -EINVAL;
	if (open_param->pic_width < CODA_ENC_MIN_PIC_WIDTH ||
	    open_param->pic_width > CODA_ENC_MAX_PIC_WIDTH)
		return -EINVAL;
	if (open_param->pic_height < CODA_ENC_MIN_PIC_HEIGHT ||
	    open_param->pic_height > CODA_ENC_MAX_PIC_HEIGHT)
		return -EINVAL;

	return 0;
}

int coda_vpuapi_enc_open(struct vpu_instance *inst, struct coda_enc_open_param *open_param)
{
	struct vpu_device *vpu = inst->vpu_dev;
	int ret;

	ret = coda_vpuapi_enc_check_open_param(inst, open_param);
	if (ret)
		return ret;

	ret = mutex_lock_interruptible(&vpu->hw_lock);
	if (ret)
		return ret;

	if (!coda_hw_is_init(vpu)) {
		mutex_unlock(&vpu->hw_lock);
		return -ENODEV;
	}

	inst->codec_info = kzalloc(sizeof(*inst->codec_info), GFP_KERNEL);
	if (!inst->codec_info) {
		mutex_unlock(&vpu->hw_lock);
		return -ENOMEM;
	}

	ret = coda_hw_build_up_enc_param(inst, open_param);
	if (ret)
		goto free_codec_info;

	mutex_unlock(&vpu->hw_lock);
	return 0;

free_codec_info:
	kfree(inst->codec_info);
	mutex_unlock(&vpu->hw_lock);
	return ret;
}

int coda_vpuapi_enc_close(struct vpu_instance *inst, u32 *fail_res)
{
	struct vpu_device *vpu = inst->vpu_dev;
	int ret;

	*fail_res = 0;

	if (!inst->codec_info)
		return -EINVAL;

	ret = mutex_lock_interruptible(&vpu->hw_lock);
	if (ret)
		return ret;

	ret = coda_hw_enc_finish_seq(inst, fail_res);
	if (ret) {
		mutex_unlock(&vpu->hw_lock);
		return ret;
	}

	kfree(inst->codec_info);
	mutex_unlock(&vpu->hw_lock);
	return 0;
}

int coda_vpuapi_enc_issue_seq_init(struct vpu_instance *inst)
{
	struct vpu_device *vpu = inst->vpu_dev;
	int ret;

	guard(mutex)(&vpu->hw_lock);
	ret = coda_hw_enc_init_seq(inst);
	if (ret)
		dev_err(inst->vpu_dev->dev, "Failed to init seq, ret = %d\n", ret);

	if (coda_vpu_wait_interrupt(inst, CODA_VPU_TIMEOUT) < 0) {
		dev_err(inst->vpu_dev->dev, "seq init timeout\n");
		ret = -ETIMEDOUT;
		return ret;
	}

	ret = coda_hw_enc_get_seq_info(inst);
	return ret;
}

int coda_vpuapi_enc_register_frame_buffer(struct vpu_instance *inst, unsigned int num,
					  unsigned int stride, int height,
					  enum coda_tiled_map_type map_type)
{
	struct vpu_device *vpu = inst->vpu_dev;
	struct coda_enc_info *p_enc_info;
	int ret;

	if (!inst->codec_info)
		return -EINVAL;

	p_enc_info = &inst->codec_info->enc_info;
	if (p_enc_info->stride)
		return -EINVAL;

	if (stride == 0 || stride % CODA_ENC_RAW_PIC_STEP != 0)
		return -EINVAL;

	if (height <= 0)
		return -EINVAL;

	ret = mutex_lock_interruptible(&vpu->hw_lock);
	if (ret)
		return ret;

	p_enc_info->frame_buf_num = num;
	p_enc_info->stride = stride;
	p_enc_info->frame_buf_height = height;

	ret = coda_hw_enc_register_frame_buffer(inst, &inst->frame_buf[0]);

	mutex_unlock(&vpu->hw_lock);
	return ret;
}

static int coda_vpuapi_enc_check_param(struct vpu_instance *inst, struct coda_enc_param *param)
{
	if (!param)
		return -EINVAL;

	return 0;
}

int coda_vpuapi_enc_start_one_frame(struct vpu_instance *inst, struct coda_enc_param *param,
				    u32 *fail_res)
{
	struct vpu_device *vpu = inst->vpu_dev;
	struct coda_enc_info *p_enc_info;
	int ret;

	*fail_res = 0;

	if (!inst->codec_info)
		return -EINVAL;

	p_enc_info = &inst->codec_info->enc_info;
	if (p_enc_info->stride == 0)
		return -EINVAL;

	ret = coda_vpuapi_enc_check_param(inst, param);
	if (ret)
		return ret;

	ret = mutex_lock_interruptible(&vpu->hw_lock);
	if (ret)
		return ret;

	p_enc_info->pts_map[param->src_idx] = param->pts;
	p_enc_info->bitstream_buf = param->bitstream_buf;
	p_enc_info->bitstream_buf_size = param->bitstream_buf_size;

	ret = coda_hw_enc_encode(inst, param, fail_res);

	mutex_unlock(&vpu->hw_lock);
	return ret;
}

int coda_vpuapi_enc_get_output_info(struct vpu_instance *inst, struct coda_enc_output_info *info)
{
	struct vpu_device *vpu = inst->vpu_dev;
	struct coda_enc_info *p_enc_info;
	int ret;

	if (!info)
		return -EINVAL;
	if (!inst->codec_info)
		return -EINVAL;

	p_enc_info = &inst->codec_info->enc_info;

	ret = mutex_lock_interruptible(&vpu->hw_lock);
	if (ret)
		return ret;

	memset(info, 0, sizeof(*info));

	ret = coda_hw_enc_get_result(inst, info);
	if (ret) {
		info->pts = 0;
		goto unlock;
	}

	if (info->recon_frame_index >= 0)
		info->pts = p_enc_info->pts_map[info->enc_src_idx];

unlock:
	mutex_unlock(&vpu->hw_lock);
	return ret;
}
