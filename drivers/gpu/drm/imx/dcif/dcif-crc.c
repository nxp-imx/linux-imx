// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright 2025 NXP
 */

#include <linux/regmap.h>

#include <drm/drm_atomic.h>
#include <drm/drm_rect.h>

#include "dcif-crc.h"
#include "dcif-drv.h"
#include "dcif-reg.h"

#define MAX_DCIF_CRC_NUM       4

static int dcif_crc_config(struct dcif_dev *dcif, struct drm_rect *roi, int ncrc)
{
	int pos, size;

	if (ncrc >= MAX_DCIF_CRC_NUM)
		return -EINVAL;

	pos = DCIF_CRC_POS_CRC_HOR_POS(roi->x1) |
	      DCIF_CRC_POS_CRC_VER_POS(roi->y1);
	size = DCIF_CRC_SIZE_CRC_HOR_SIZE(roi->x2 - roi->x1) |
	       DCIF_CRC_SIZE_CRC_VER_SIZE(roi->y2 - roi->y1);

	regmap_write(dcif->regmap, DCIF_CRC_POS_R(ncrc), pos);
	regmap_write(dcif->regmap, DCIF_CRC_SIZE_R(ncrc), size);

	regmap_set_bits(dcif->regmap, DCIF_CRC_CTRL,
			DCIF_CRC_CTRL_CRC_EN(ncrc) | DCIF_CRC_CTRL_CRC_ERR_CNT_RST);

	return 0;
}

void dcif_crtc_enable_crc_source(struct dcif_dev *dcif,
				 enum dcif_crc_source source,
				 struct drm_rect *roi,
				 int ncrc)
{
	if (ncrc >= MAX_DCIF_CRC_NUM)
		return;

	if (source == DCIF_CRC_SRC_NONE)
		return;

	if (dcif->crc_is_enabled)
		return;

	dcif_crc_config(dcif, roi, ncrc);

	regmap_set_bits(dcif->regmap, DCIF_CRC_CTRL,
			DCIF_CRC_CTRL_CRC_MODE | DCIF_CRC_CTRL_CRC_SHADOW_LOAD_EN |
			DCIF_CRC_CTRL_CRC_TRIG);

	dcif->crc_is_enabled = true;
}

void dcif_crtc_disable_crc_source(struct dcif_dev *dcif, int ncrc)
{
	if (!dcif->crc_is_enabled)
		return;

	if (ncrc >= MAX_DCIF_CRC_NUM)
		return;

	regmap_clear_bits(dcif->regmap, DCIF_CRC_CTRL, DCIF_CRC_CTRL_CRC_EN(ncrc));

	dcif->crc_is_enabled = false;
}

/*
 * Supported modes and source names:
 * 1) auto mode:
 *    "auto" should be selected as the source name.
 *    The evaluation window is the same to the display region as
 *    indicated by drm_crtc_state->adjusted_mode.
 *
 * 2) region of interest(ROI) mode:
 *    "roi:x1,y1,x2,y2" should be selected as the source name.
 *    The region of interest is defined by the inclusive upper left
 *    position at (x1, y1) and the exclusive lower right position
 *    at (x2, y2), see struct drm_rect for the same idea.
 *    The evaluation window is the region of interest.
 */
static int
dcif_crc_parse_source(const char *source_name, enum dcif_crc_source *s,
		      struct drm_rect *roi)
{
	static const char roi_prefix[] = "roi:";

	if (!source_name) {
		*s = DCIF_CRC_SRC_NONE;
	} else if (!strcmp(source_name, "auto")) {
		*s = DCIF_CRC_SRC_FRAME;
	} else if (strstarts(source_name, roi_prefix)) {
		char *options, *opt;
		int len = strlen(roi_prefix);
		int params[4];
		int i = 0, ret;

		options = kstrdup(source_name + len, GFP_KERNEL);

		while ((opt = strsep(&options, ",")) != NULL) {
			if (i > 3)
				return -EINVAL;

			ret = kstrtouint(opt, 10, &params[i]);
			if (ret < 0)
				return ret;

			if (params[i] < 0)
				return -EINVAL;

			i++;
		}

		if (i != 4)
			return -EINVAL;

		roi->x1 = params[0];
		roi->y1 = params[1];
		roi->x2 = params[2];
		roi->y2 = params[3];

		if (!drm_rect_visible(roi))
			return -EINVAL;

		*s = DCIF_CRC_SRC_FRAME_ROI;
	} else {
		return -EINVAL;
	}

	return 0;
}

int dcif_crtc_verify_crc_source(struct drm_crtc *crtc, const char *source_name,
				size_t *values_cnt)
{
	struct dcif_dev *dcif = crtc_to_dcif_dev(crtc);
	enum dcif_crc_source source;
	struct drm_rect roi;

	if (dcif_crc_parse_source(source_name, &source, &roi) < 0) {
		dev_dbg(dcif->drm.dev, "unknown source %s\n", source_name);
		return -EINVAL;
	}

	*values_cnt = 1;

	return 0;
}

int dcif_crtc_set_crc_source(struct drm_crtc *crtc, const char *source_name)
{
	struct dcif_dev *dcif = crtc_to_dcif_dev(crtc);
	struct drm_modeset_acquire_ctx ctx;
	struct drm_crtc_state *crtc_state;
	struct drm_atomic_state *state;
	struct drm_rect roi = {0, 0, 0, 0};
	enum dcif_crc_source source;
	int ret;

	if (dcif_crc_parse_source(source_name, &source, &roi) < 0) {
		dev_dbg(dcif->drm.dev, "unknown source %s\n", source_name);
		return -EINVAL;
	}

	/* Perform an atomic commit to set the CRC source. */
	drm_modeset_acquire_init(&ctx, 0);

	state = drm_atomic_state_alloc(crtc->dev);
	if (!state) {
		ret = -ENOMEM;
		goto unlock;
	}

	state->acquire_ctx = &ctx;

retry:
	crtc_state = drm_atomic_get_crtc_state(state, crtc);
	if (!IS_ERR(crtc_state)) {
		struct dcif_crtc_state *dcif_crtc_state;

		dcif_crtc_state = to_dcif_crtc_state(crtc_state);

		dcif_crtc_state->crc.source = source;
		dcif_copy_roi(&roi, &dcif_crtc_state->crc.roi);

		ret = drm_atomic_commit(state);
	} else {
		ret = PTR_ERR(crtc_state);
	}

	if (ret == -EDEADLK) {
		drm_atomic_state_clear(state);
		drm_modeset_backoff(&ctx);
		goto retry;
	}

	drm_atomic_state_put(state);

unlock:
	drm_modeset_drop_locks(&ctx);
	drm_modeset_acquire_fini(&ctx);

	return ret;
}

