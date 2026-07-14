/* SPDX-License-Identifier: GPL-2.0+ */

/*
 * Copyright 2023,2026 NXP
 */

#ifndef __DPU95_DRV_H__
#define __DPU95_DRV_H__

#include <drm/drm_atomic.h>
#include <drm/drm_device.h>
#include <drm/drm_encoder.h>

#include "dpu95.h"
#include "dpu95-crtc.h"
#include "dpu95-plane.h"
#include "dpu95-blit.h"
#include "dpu95-ld.h"

#define DPU95_CRTCS	2
#define DPU95_ENCODERS	DPU95_CRTCS
#define DPU95_PRIMARYS	DPU95_CRTCS

struct dpu95_private_state {
	struct drm_private_state	base;
	struct drm_plane		*plane;
	struct drm_crtc			*crtc;
	struct drm_crtc_commit		*pending_commit;
};

#define to_dpu95_private_state(_state)	\
	container_of_const(_state, struct dpu95_private_state, base)

struct dpu95_private_state *
dpu95_private_get_new_state(const struct drm_atomic_state *state,
			    struct drm_private_obj *manager);
struct dpu95_private_state *
dpu95_private_get_old_state(const struct drm_atomic_state *state,
			    struct drm_private_obj *manager);
struct dpu95_private_state *
dpu95_private_get_state(struct drm_atomic_state *state,
			struct drm_private_obj *manager);

enum dpu95_private_obj_type {
	DPU95_PRIVATE_OBJ_FU,
	DPU95_PRIVATE_OBJ_LB,
	DPU95_PRIVATE_OBJ_HS,
	DPU95_PRIVATE_OBJ_VS,
};

union dpu95_private_obj_res {
	struct dpu95_fetchunit	*fu;
	struct dpu95_layerblend	*lb;
	struct dpu95_hscaler	*hs;
	struct dpu95_vscaler	*vs;
};

struct dpu95_private_obj {
	struct drm_private_obj		base;
	enum dpu95_private_obj_type	type;
	struct list_head		node;
	union dpu95_private_obj_res	res;
};

#define to_dpu95_private_obj(_obj)	\
	container_of_const(_obj, struct dpu95_private_obj, base)

#define node_to_dpu95_private_obj(_node)	\
	container_of_const(_node, struct dpu95_private_obj, node)

struct dpu95_drm_device {
	struct drm_device		base;
	struct dpu95_soc		dpu_soc;
	struct dpu_bliteng		dpu_be;
	struct dpu95_crtc		dpu_crtc[DPU95_CRTCS];
	struct dpu95_plane		dpu_primary[DPU95_PRIMARYS];
	struct dpu95_plane		*dpu_overlay;
	struct dpu95_plane_grp		dpu_plane_grp;
	struct drm_encoder		encoder[DPU95_ENCODERS];
	struct dpu95_private_obj	*fu_manager;
	struct dpu95_private_obj	*lb_manager;
	struct dpu95_private_obj	hs_manager;
	struct dpu95_private_obj	vs_manager;
	struct list_head		obj_list;
	unsigned int			dpu_overlay_cnt;
	unsigned int			dpu_hw_plane_cnt;
	u32				crtc_mask;
};

static inline struct dpu95_drm_device *
to_dpu95_drm_device(struct drm_device *drm)
{
	return container_of(drm, struct dpu95_drm_device, base);
}

static inline bool is_dpu95_private_obj(struct dpu95_drm_device *dpu_drm,
					struct drm_private_obj *obj)
{
	struct dpu95_private_obj *priv_obj;
	struct list_head *node;

	list_for_each(node, &dpu_drm->obj_list) {
		priv_obj = node_to_dpu95_private_obj(node);
		if (obj == &priv_obj->base)
			return true;
	}

	return false;
}

int dpu95_core_init(struct dpu95_drm_device *dpu_drm);

int dpu95_kms_prepare(struct dpu95_drm_device *dpu_drm);
void dpu95_kms_unprepare(struct dpu95_drm_device *dpu_drm);

#endif /* __DPU95_DRV_H__ */
