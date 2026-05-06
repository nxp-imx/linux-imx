/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Wave5 series multi-standard codec IP - vpu control interface
 *
 * Copyright (C) 2026 CHIPS&MEDIA INC
 */

#ifndef __WAVE5_VPU_CTRL_H__
#define __WAVE5_VPU_CTRL_H__

#include <linux/device.h>

enum {
	WAVE5_VPU_STATE_OFF,
	WAVE5_VPU_STATE_PREPARE,
	WAVE5_VPU_STATE_ON,
	WAVE5_VPU_STATE_SLEEP,
};

struct wave5_vpu_entity {
	struct list_head list;
	struct device *dev;
	u32 (*read_reg)(struct device *dev, u32 addr);
	void (*write_reg)(struct device *dev, u32 addr, u32 data);
	void (*on_boot)(struct device *dev);
	void (*scan_instances)(struct device *dev);
	bool booted;
};

int wave5_convert_endian(unsigned int endian);
void wave5_swap_endian(u8 *data, int len, int endian);
int wave5_vpu_ctrl_resume_and_get(struct device *dev, struct wave5_vpu_entity *entity);
void wave5_vpu_ctrl_put_sync(struct device *dev, struct wave5_vpu_entity *entity);
int wave5_vpu_ctrl_get_state(struct device *dev);
int wave5_vpu_ctrl_wait_done(struct device *dev);
int wave5_vpu_ctrl_require_buffer(struct device *dev, struct wave5_vpu_entity *entity);
bool wave5_vpu_ctrl_support_follower(struct device *dev);
struct imx_mur_node *wave5_vpu_ctrl_get_recorder(struct device *dev);
#endif
