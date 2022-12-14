/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2019 Intel Corporation
 */

#ifndef __INTEL_PANEL_H__
#define __INTEL_PANEL_H__

#include <linux/types.h>

enum drm_connector_status;
struct drm_connector;
struct drm_connector_state;
struct drm_display_mode;
struct drm_i915_private;
struct intel_connector;
struct intel_crtc_state;
struct intel_panel;

int intel_panel_init(struct intel_panel *panel,
		     struct drm_display_mode *fixed_mode,
		     struct drm_display_mode *downclock_mode);
void intel_panel_fini(struct intel_panel *panel);
enum drm_connector_status
intel_panel_detect(struct drm_connector *connector, bool force);
void intel_fixed_panel_mode(const struct drm_display_mode *fixed_mode,
			    struct drm_display_mode *adjusted_mode);
int intel_pch_panel_fitting(struct intel_crtc_state *crtc_state,
			    const struct drm_connector_state *conn_state);
int intel_gmch_panel_fitting(struct intel_crtc_state *crtc_state,
			     const struct drm_connector_state *conn_state);
struct drm_display_mode *
intel_panel_edid_downclock_mode(struct intel_connector *connector,
				const struct drm_display_mode *fixed_mode);
struct drm_display_mode *
intel_panel_edid_fixed_mode(struct intel_connector *connector);
struct drm_display_mode *
intel_panel_vbt_fixed_mode(struct intel_connector *connector);

#endif /* __INTEL_PANEL_H__ */
