// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2024 NXP
 */

#ifndef _OX03C10_PRIV_H_
#define _OX03C10_PRIV_H_

#include <uapi/linux/ox03c10.h>

enum ox03c10_custom_ctrls {
	OX03C10_EXPOSURE,
	OX03C10_AGAIN,
	OX03C10_DGAIN,
	OX03C10_WBGAIN,
	OX03C10_PWL_EN,
	OX03C10_PWL_CTRL,
	OX03C10_PWL_KNEE_POINTS_LUT,
	OX03C10_OTP_CORRECTION,
};

struct ox03c10 {
	struct i2c_client *client;
	struct device *dev;
	struct regmap *rmap;

	struct v4l2_ctrl_handler ctrl_handler;

	const struct ox03c10_mode *cur_mode;

	bool streaming;

	bool gh_open[4];

	s32 exposure_input;
	struct ox03c10_exposure exposure;
	s32 again_input;
	struct ox03c10_analog_gain again;
	s32 dgain_input;
	struct ox03c10_digital_gain dgain;

	struct v4l2_ctrl *vflip;

	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;

	/* this needs to be the last element in the structure */
	struct v4l2_ctrl *ctrls[];
};

#define OX03C10_NATIVE_WIDTH		1936
#define OX03C10_NATIVE_HEIGHT		1296

#define OX03C10_PIXEL_ARRAY_TOP		8
#define OX03C10_PIXEL_ARRAY_LEFT	8
#define OX03C10_PIXEL_ARRAY_WIDTH	1920
#define OX03C10_PIXEL_ARRAY_HEIGHT	1282 /* first 2 lines is embedded data */

struct ox03c10_mode {
	u32 width;
	u32 height;
	u32 hts;
	u32 vts;
	u16 fps;
	struct v4l2_rect crop;
};

int ox03c10_v4l2_controls_init(struct ox03c10 *ox03c10);
struct ox03c10 *ox03c10_init_with_dummy_client(struct i2c_client *client,
					       bool use_dummy);
int ox03c10_streaming_start(struct ox03c10 *sensor, bool start);
struct v4l2_ctrl_handler *ox03c10_ctrl_handler_get(struct ox03c10 *sensor);
void ox03c10_ctrl_handler_free(struct ox03c10 *sensor);
struct ox03c10_mode *ox03c10_get_mode(int index);
const struct ox03c10_mode *ox03c10_find_closest_mode(struct ox03c10 *sensor, u16 width, u16 height);
int ox03c10_set_mode(struct ox03c10 *sensor, const struct ox03c10_mode *mode);

#endif