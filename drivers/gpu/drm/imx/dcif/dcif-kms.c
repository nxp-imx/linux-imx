// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright 2025 NXP
 */

#include <linux/of.h>
#include <linux/of_graph.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_vblank.h>

#include "dcif-drv.h"
#include "dcif-reg.h"

static int dcif_kms_init(struct dcif_dev *dcif)
{
	struct drm_device *drm = &dcif->drm;
	struct drm_bridge *bridge;
	struct drm_connector *connector;
	struct device_node *np = drm->dev->of_node;
	int ret;

	ret = dcif_crtc_init(dcif);
	if (ret)
		return ret;

	bridge = devm_drm_of_get_bridge(drm->dev, np, 0, 0);
	if (IS_ERR(bridge))
		return dev_err_probe(drm->dev, PTR_ERR(bridge), "Failed to find bridge\n");

	dcif->encoder.possible_crtcs = drm_crtc_mask(&dcif->crtc);
	ret = drm_simple_encoder_init(drm, &dcif->encoder, DRM_MODE_ENCODER_NONE);
	if (ret) {
		drm_err(drm, "failed to initialize encoder: %d\n", ret);
		return ret;
	}

	ret = drm_bridge_attach(&dcif->encoder, bridge, NULL, DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret) {
		drm_err(drm, "failed to attach bridge to encoder: %d\n", ret);
		return ret;
	}

	connector = drm_bridge_connector_init(drm, &dcif->encoder);
	if (IS_ERR(connector)) {
		ret = PTR_ERR(connector);
		drm_err(drm, "failed to initialize bridge connector: %d\n", ret);
		return ret;
	}

	ret = drm_connector_attach_encoder(connector, &dcif->encoder);
	if (ret)
		drm_err(drm, "failed to attach encoder to connector: %d\n", ret);

	return ret;
}

static const struct drm_mode_config_funcs dcif_mode_config_funcs = {
	.fb_create     = drm_gem_fb_create,
	.atomic_check  = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static const struct drm_mode_config_helper_funcs dcif_mode_config_helpers = {
	.atomic_commit_tail = drm_atomic_helper_commit_tail_rpm,
};

int dcif_kms_prepare(struct dcif_dev *dcif)
{
	struct drm_device *drm = &dcif->drm;
	int ret;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;

	ret = dcif_kms_init(dcif);
	if (ret)
		return ret;

	drm->mode_config.min_width	= 1;
	drm->mode_config.min_height	= 1;
	drm->mode_config.max_width	= 1920;
	drm->mode_config.max_height	= 1920;
	drm->mode_config.funcs		= &dcif_mode_config_funcs;
	drm->mode_config.helper_private	= &dcif_mode_config_helpers;

	ret = drm_vblank_init(drm, 1);
	if (ret < 0) {
		drm_err(drm, "failed to initialize vblank: %d\n", ret);
		return ret;
	}

	drm_mode_config_reset(drm);

	drm_kms_helper_poll_init(drm);

	return 0;
}
