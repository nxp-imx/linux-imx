// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2019 NXP
 */

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>

static void lcdifv3_drm_atomic_commit_tail(struct drm_atomic_state *state)
{
	struct drm_device *dev = state->dev;

	drm_atomic_helper_commit_modeset_disables(dev, state);

	drm_atomic_helper_commit_modeset_enables(dev, state);

	drm_atomic_helper_commit_planes(dev, state, DRM_PLANE_COMMIT_ACTIVE_ONLY);

	drm_atomic_helper_commit_hw_done(state);

	drm_atomic_helper_wait_for_vblanks(dev, state);

	drm_atomic_helper_cleanup_planes(dev, state);
}

static int lcdifv3_drm_atomic_check(struct drm_device *dev, struct drm_atomic_state *state)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *new_crtc_state;
	struct drm_display_mode *mode;
	int i;

	if (state->allow_modeset) {
		for_each_new_crtc_in_state(state, crtc, new_crtc_state, i) {
			WARN_ON(!drm_modeset_is_locked(&crtc->mutex));

			mode = &new_crtc_state->mode;
			if (!strcmp(mode->name, "1280x720") && drm_mode_vrefresh(mode) == 60) {
				new_crtc_state->mode_changed = true;
			}
		}
	}

	return drm_atomic_helper_check(dev, state);
}
const struct drm_mode_config_funcs lcdifv3_drm_mode_config_funcs = {
	.fb_create     = drm_gem_fb_create,
	.atomic_check  = lcdifv3_drm_atomic_check,
	.atomic_commit = drm_atomic_helper_commit,
};

struct drm_mode_config_helper_funcs lcdifv3_drm_mode_config_helpers = {
	.atomic_commit_tail = lcdifv3_drm_atomic_commit_tail,
};
