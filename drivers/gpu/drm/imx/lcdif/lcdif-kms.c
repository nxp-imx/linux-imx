/*
 * Copyright 2018 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <drm/drm_vblank.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>

static void lcdif_drm_atomic_commit_tail(struct drm_atomic_state *state)
{
	struct drm_device *dev = state->dev;

	drm_atomic_helper_commit_modeset_disables(dev, state);

	drm_atomic_helper_commit_modeset_enables(dev, state);

	drm_atomic_helper_commit_planes(dev, state, DRM_PLANE_COMMIT_ACTIVE_ONLY);

	drm_atomic_helper_commit_hw_done(state);

	drm_atomic_helper_wait_for_vblanks(dev, state);

	drm_atomic_helper_cleanup_planes(dev, state);
}

static int lcdif_drm_atomic_check(struct drm_device *dev, struct drm_atomic_state *state)
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

const struct drm_mode_config_funcs lcdif_drm_mode_config_funcs = {
	.fb_create     = drm_gem_fb_create,
	.atomic_check  = lcdif_drm_atomic_check,
	.atomic_commit = drm_atomic_helper_commit,
};

struct drm_mode_config_helper_funcs lcdif_drm_mode_config_helpers = {
	.atomic_commit_tail = lcdif_drm_atomic_commit_tail,
};
