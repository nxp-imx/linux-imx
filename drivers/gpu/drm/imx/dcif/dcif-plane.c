// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright 2025 NXP
 */

#include <linux/regmap.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_print.h>
#include <drm/drm_rect.h>

#include "dcif-drv.h"
#include "dcif-reg.h"

static const u32 dcif_primary_plane_formats[] = {
	/* RGB */
	DRM_FORMAT_RGB565,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_XRGB1555,
	DRM_FORMAT_XRGB4444,
	DRM_FORMAT_XRGB8888,

	/* Packed YCbCr */
	DRM_FORMAT_YUYV,
	DRM_FORMAT_YVYU,
	DRM_FORMAT_UYVY,
	DRM_FORMAT_VYUY,
};

static const u32 dcif_overlay_plane_formats[] = {
	/* RGB */
	DRM_FORMAT_RGB565,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_XRGB1555,
	DRM_FORMAT_XRGB4444,
	DRM_FORMAT_XRGB8888,
};

static inline struct dcif_dev *plane_to_dcif_dev(struct drm_plane *plane)
{
	return to_dcif_dev(plane->dev);
}

static inline dma_addr_t drm_plane_state_to_baseaddr(struct drm_plane_state *state)
{
	struct drm_framebuffer *fb = state->fb;
	struct drm_gem_dma_object *dma_obj;
	unsigned int x = state->src.x1 >> 16;
	unsigned int y = state->src.y1 >> 16;

	dma_obj = drm_fb_dma_get_gem_obj(fb, 0);

	return dma_obj->dma_addr + fb->offsets[0] + fb->pitches[0] * y +
	       fb->format->cpp[0] * x;
}

static int dcif_plane_get_layer_id(struct drm_plane *plane)
{
	return (plane->type == DRM_PLANE_TYPE_PRIMARY) ? 0 : 1;
}

static int dcif_plane_atomic_check(struct drm_plane *plane, struct drm_atomic_state *state)
{
	struct drm_plane_state *new_plane_state =
				drm_atomic_get_new_plane_state(state, plane);
	struct drm_plane_state *old_plane_state =
				drm_atomic_get_old_plane_state(state, plane);
	struct dcif_dev *dcif = plane_to_dcif_dev(plane);
	struct drm_crtc_state *crtc_state;
	struct drm_framebuffer *fb = new_plane_state->fb;
	struct drm_framebuffer *old_fb = old_plane_state->fb;
	int ret;

	if (!fb)
		return 0;

	crtc_state = drm_atomic_get_new_crtc_state(state, &dcif->crtc);
	if (WARN_ON(!crtc_state))
		return -EINVAL;
	/*
	 * Force CRTC mode change if framebuffer stride or pixel format have changed.
	 */
	if (plane->type == DRM_PLANE_TYPE_PRIMARY && old_fb &&
	    (fb->pitches[0] != old_fb->pitches[0] ||
	     fb->format->format != old_fb->format->format))
		crtc_state->mode_changed = true;

	ret = drm_atomic_helper_check_plane_state(new_plane_state, crtc_state,
						  DRM_PLANE_NO_SCALING,
						  DRM_PLANE_NO_SCALING,
						  true, true);
	if (ret)
		return ret;

	return 0;
}

static void dcif_plane_atomic_update(struct drm_plane *plane,
				     struct drm_atomic_state *state)
{
	struct drm_plane_state *new_state = drm_atomic_get_new_plane_state(state, plane);
	struct dcif_dev *dcif = plane_to_dcif_dev(plane);
	int layer_id = dcif_plane_get_layer_id(plane);
	struct drm_framebuffer *fb = new_state->fb;
	u32 crtc_x, crtc_y, crtc_h, crtc_w;
	u32 layer_fmt = 0, yuv_fmt = 0;
	dma_addr_t baseaddr;
	u32 reg;

	if (!fb)
		return;

	crtc_x = new_state->crtc_x;
	crtc_y = new_state->crtc_y;
	crtc_h = new_state->crtc_h;
	crtc_w = new_state->crtc_w;

	/* visible portion of plane on crtc */
	regmap_write(dcif->regmap, DCIF_CTRLDESC1(layer_id),
		     DCIF_CTRLDESC1_POSX(crtc_x) | DCIF_CTRLDESC1_POSY(crtc_y));
	regmap_write(dcif->regmap, DCIF_CTRLDESC2(layer_id),
		     DCIF_CTRLDESC2_WIDTH(crtc_w) | DCIF_CTRLDESC2_HEIGHT(crtc_h));

	/* pitch size */
	reg = DCIF_CTRLDESC3_P_SIZE(2) | DCIF_CTRLDESC3_T_SIZE(2) |
	      DCIF_CTRLDESC3_PITCH(fb->pitches[0]);
	regmap_write(dcif->regmap, DCIF_CTRLDESC3(layer_id), reg);

	/*  address */
	baseaddr = drm_fb_dma_get_gem_addr(new_state->fb, new_state, 0);

	drm_dbg_kms(plane->dev, "[PLANE:%d:%s] fb address %pad, pitch 0x%08x\n",
		    plane->base.id, plane->name, &baseaddr, fb->pitches[0]);

	regmap_write(dcif->regmap, DCIF_CTRLDESC4(layer_id), baseaddr);

	/* Format */
	switch (fb->format->format) {
	/* RGB Formats */
	case DRM_FORMAT_RGB565:
		layer_fmt = CTRLDESCL0_FORMAT_RGB565;
		break;
	case DRM_FORMAT_RGB888:
		layer_fmt = CTRLDESCL0_FORMAT_RGB888;
		break;
	case DRM_FORMAT_XRGB1555:
		layer_fmt = CTRLDESCL0_FORMAT_ARGB1555;
		break;
	case DRM_FORMAT_XRGB4444:
		layer_fmt = CTRLDESCL0_FORMAT_ARGB4444;
		break;
	case DRM_FORMAT_XBGR8888:
		layer_fmt = CTRLDESCL0_FORMAT_ABGR8888;
		break;
	case DRM_FORMAT_XRGB8888:
		layer_fmt = CTRLDESCL0_FORMAT_ARGB8888;
		break;

	/* YUV Formats */
	case DRM_FORMAT_YUYV:
		layer_fmt = CTRLDESCL0_FORMAT_YCBCR422;
		yuv_fmt = CTRLDESCL0_YUV_FORMAT_VY2UY1;
		break;
	case DRM_FORMAT_YVYU:
		layer_fmt = CTRLDESCL0_FORMAT_YCBCR422;
		yuv_fmt = CTRLDESCL0_YUV_FORMAT_UY2VY1;
		break;
	case DRM_FORMAT_UYVY:
		layer_fmt = CTRLDESCL0_FORMAT_YCBCR422;
		yuv_fmt = CTRLDESCL0_YUV_FORMAT_Y2VY1U;
		break;
	case DRM_FORMAT_VYUY:
		layer_fmt = CTRLDESCL0_FORMAT_YCBCR422;
		yuv_fmt = CTRLDESCL0_YUV_FORMAT_Y2UY1V;
		break;

	default:
		dev_err(dcif->drm.dev, "Unknown pixel format 0x%x\n", fb->format->format);
		break;
	}

	if (plane->type == DRM_PLANE_TYPE_OVERLAY && yuv_fmt == CTRLDESCL0_YUV_FORMAT_Y2UY1V) {
		dev_err(dcif->drm.dev, "Overlay plane could not support YUV format\n");
		return;
	}

	reg = DCIF_CTRLDESC0_EN | DCIF_CTRLDESC0_SHADOW_LOAD_EN |
	      DCIF_CTRLDESC0_FORMAT(layer_fmt) | DCIF_CTRLDESC0_YUV_FORMAT(yuv_fmt);

	/* Alpha */
	if (new_state->pixel_blend_mode == DRM_MODE_BLEND_COVERAGE)
		reg |= DCIF_CTRLDESC0_GLOBAL_ALPHA(new_state->alpha >> 8) | ALPHA_GLOBAL;
	else
		reg |= DCIF_CTRLDESC0_GLOBAL_ALPHA(255) | ALPHA_GLOBAL;

	regmap_write(dcif->regmap, DCIF_CTRLDESC0(layer_id), reg);
}

static void dcif_overlay_plane_atomic_disable(struct drm_plane *plane,
					      struct drm_atomic_state *state)
{
	struct dcif_dev *dcif = plane_to_dcif_dev(plane);

	regmap_update_bits(dcif->regmap, DCIF_CTRLDESC0(1),
			   DCIF_CTRLDESC0_EN | DCIF_CTRLDESC0_SHADOW_LOAD_EN,
			   DCIF_CTRLDESC0_SHADOW_LOAD_EN);
}

static const struct drm_plane_helper_funcs dcif_primary_plane_helper_funcs = {
	.prepare_fb	= drm_gem_plane_helper_prepare_fb,
	.atomic_check	= dcif_plane_atomic_check,
	.atomic_update	= dcif_plane_atomic_update,
};

static const struct drm_plane_helper_funcs dcif_overlay_plane_helper_funcs = {
	.atomic_check	= dcif_plane_atomic_check,
	.atomic_update	= dcif_plane_atomic_update,
	.atomic_disable = dcif_overlay_plane_atomic_disable,
};

static const struct drm_plane_funcs dcif_plane_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.destroy		= drm_plane_cleanup,
	.reset			= drm_atomic_helper_plane_reset,
	.atomic_duplicate_state	= drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
};

int dcif_plane_init(struct dcif_dev *dcif)
{
	const u32 supported_encodings = BIT(DRM_COLOR_YCBCR_BT601) |
					BIT(DRM_COLOR_YCBCR_BT709) |
					BIT(DRM_COLOR_YCBCR_BT2020);
	const u32 supported_ranges = BIT(DRM_COLOR_YCBCR_LIMITED_RANGE) |
				     BIT(DRM_COLOR_YCBCR_FULL_RANGE);
	int ret;

	/* primary plane */
	drm_plane_helper_add(&dcif->planes.primary,
			     &dcif_primary_plane_helper_funcs);
	ret = drm_universal_plane_init(&dcif->drm, &dcif->planes.primary, 1,
				       &dcif_plane_funcs,
				       dcif_primary_plane_formats,
				       ARRAY_SIZE(dcif_primary_plane_formats),
				       NULL,
				       DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret) {
		drm_err(&dcif->drm, "failed to initialize primary plane: %d\n", ret);
		return ret;
	}

	ret = drm_plane_create_color_properties(&dcif->planes.primary,
						supported_encodings,
						supported_ranges,
						DRM_COLOR_YCBCR_BT601,
						DRM_COLOR_YCBCR_LIMITED_RANGE);
	if (ret)
		return ret;

	ret = drm_plane_create_alpha_property(&dcif->planes.primary);
	if (ret)
		return ret;

	ret = drm_plane_create_blend_mode_property(&dcif->planes.primary,
						   BIT(DRM_MODE_BLEND_PIXEL_NONE) |
						   BIT(DRM_MODE_BLEND_PREMULTI)   |
						   BIT(DRM_MODE_BLEND_COVERAGE));
	if (ret)
		return ret;

	/* overlay plane */
	drm_plane_helper_add(&dcif->planes.overlay,
			     &dcif_overlay_plane_helper_funcs);
	ret = drm_universal_plane_init(&dcif->drm, &dcif->planes.overlay, 1,
				       &dcif_plane_funcs,
				       dcif_overlay_plane_formats,
				       ARRAY_SIZE(dcif_overlay_plane_formats),
				       NULL,
				       DRM_PLANE_TYPE_OVERLAY, NULL);
	if (ret) {
		drm_err(&dcif->drm, "failed to initialize overlay plane: %d\n", ret);
		return ret;
	}

	ret = drm_plane_create_alpha_property(&dcif->planes.overlay);
	if (ret)
		return ret;

	return drm_plane_create_blend_mode_property(&dcif->planes.overlay,
						    BIT(DRM_MODE_BLEND_PIXEL_NONE) |
						    BIT(DRM_MODE_BLEND_PREMULTI)   |
						    BIT(DRM_MODE_BLEND_COVERAGE));
}
