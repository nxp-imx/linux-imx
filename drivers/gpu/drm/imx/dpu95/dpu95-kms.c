// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright 2017-2020,2022,2023,2025,2026 NXP
 */

#include <linux/list.h>
#include <linux/slab.h>
#include <linux/sort.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>

#include "dpu95.h"
#include "dpu95-crtc.h"
#include "dpu95-drv.h"
#include "dpu95-fetchunit.h"
#include "dpu95-plane.h"

#define DPU95_PRIVATE_OBJ_NAME_SIZE	12

struct dpu95_private_state *
dpu95_private_get_old_state(const struct drm_atomic_state *state,
			    struct drm_private_obj *manager)
{
	struct drm_private_state *priv_state;

	priv_state = drm_atomic_get_old_private_obj_state(state, manager);
	if (!priv_state)
		return ERR_PTR(-EINVAL);

	return to_dpu95_private_state(priv_state);
}

struct dpu95_private_state *
dpu95_private_get_state(struct drm_atomic_state *state,
			struct drm_private_obj *manager)
{
	struct drm_private_state *priv_state;

	priv_state = drm_atomic_get_private_obj_state(state, manager);
	if (IS_ERR(priv_state))
		return ERR_CAST(priv_state);

	return to_dpu95_private_state(priv_state);
}

static int zpos_cmp(const void *a, const void *b)
{
	const struct drm_plane_state *sa = *(struct drm_plane_state **)a;
	const struct drm_plane_state *sb = *(struct drm_plane_state **)b;

	return sa->normalized_zpos - sb->normalized_zpos;
}

static int
dpu95_atomic_sort_planes_per_crtc(struct drm_crtc_state *crtc_state,
				  struct drm_plane_state **plane_states)
{
	struct drm_atomic_state *state = crtc_state->state;
	struct drm_plane *plane;
	int n = 0;

	drm_atomic_crtc_state_for_each_plane(plane, crtc_state) {
		struct drm_plane_state *plane_state =
			drm_atomic_get_plane_state(state, plane);
		if (IS_ERR(plane_state))
			return PTR_ERR(plane_state);
		plane_states[n++] = plane_state;
	}

	sort(plane_states, n, sizeof(*plane_states), zpos_cmp, NULL);

	return n;
}

static void
dpu95_atomic_set_top_plane_per_crtc(struct drm_plane_state **plane_states, int n)
{
	struct dpu95_plane_state *dpstate;
	int i;

	for (i = 0; i < n; i++) {
		dpstate = to_dpu95_plane_state(plane_states[i]);
		dpstate->is_top = (i == (n - 1)) ? true : false;
	}
}

static bool
dpu95_private_obj_is_available(struct drm_plane *plane, struct drm_crtc *crtc,
			       struct dpu95_private_state *new_private_state,
			       struct dpu95_private_state *old_private_state)
{
	/* Unavailable if the object is already used by another plane. */
	if (new_private_state->plane && new_private_state->plane != plane)
		return false;

	/* Unavailable if the object is being disabled on another CRTC. */
	if (old_private_state->plane && !new_private_state->plane &&
	    old_private_state->crtc != crtc)
		return false;

	return true;
}

static int dpu95_private_obj_assign(struct drm_plane *plane,
				    struct drm_crtc *crtc,
				    struct drm_atomic_state *state,
				    struct drm_private_obj *manager)
{
	struct dpu95_private_state *new_private_state;
	struct dpu95_private_state *old_private_state;
	bool is_available;

	new_private_state = dpu95_private_get_state(state, manager);
	if (IS_ERR(new_private_state))
		return PTR_ERR(new_private_state);

	old_private_state = dpu95_private_get_old_state(state, manager);
	if (IS_ERR(old_private_state))
		return PTR_ERR(old_private_state);

	is_available = dpu95_private_obj_is_available(plane, crtc,
						      new_private_state,
						      old_private_state);
	if (!is_available)
		return -EINVAL;

	new_private_state->plane = plane;
	new_private_state->crtc = crtc;

	return 0;
}

static int dpu95_private_obj_disable(struct drm_atomic_state *state,
				     struct drm_private_obj *manager)
{
	struct dpu95_private_state *private_state;

	private_state = dpu95_private_get_state(state, manager);
	if (IS_ERR(private_state))
		return PTR_ERR(private_state);

	private_state->plane = NULL;
	private_state->crtc = NULL;

	return 0;
}

static struct dpu95_hscaler *
dpu95_plane_assign_hscaler(struct drm_crtc *crtc,
			   struct drm_atomic_state *state,
			   struct drm_plane *plane,
			   struct dpu95_fetchunit *fu)
{
	const struct dpu95_fetchunit_ops *fu_ops;
	const struct dpu95_hscaler_ops *hs_ops;
	struct dpu95_hscaler *hs;
	struct drm_private_obj *manager;
	int ret;

	fu_ops = dpu95_fu_get_ops(fu);
	hs = fu_ops->get_hscaler(fu);
	hs_ops = dpu95_hs_get_ops(hs);
	manager = hs_ops->get_manager(hs);

	ret = dpu95_private_obj_assign(plane, crtc, state, manager);
	if (ret)
		return (struct dpu95_hscaler *)ERR_PTR(ret);

	return hs;
}

static struct dpu95_vscaler *
dpu95_plane_assign_vscaler(struct drm_crtc *crtc,
			   struct drm_atomic_state *state,
			   struct drm_plane *plane,
			   struct dpu95_fetchunit *fu)
{
	const struct dpu95_fetchunit_ops *fu_ops;
	const struct dpu95_vscaler_ops *vs_ops;
	struct drm_private_obj *manager;
	struct dpu95_vscaler *vs;
	int ret;

	fu_ops = dpu95_fu_get_ops(fu);
	vs = fu_ops->get_vscaler(fu);
	vs_ops = dpu95_vs_get_ops(vs);
	manager = vs_ops->get_manager(vs);

	ret = dpu95_private_obj_assign(plane, crtc, state, manager);
	if (ret)
		return (struct dpu95_vscaler *)ERR_PTR(ret);

	return vs;
}

static int
dpu95_atomic_assign_plane_source_per_crtc(struct drm_crtc *crtc,
					  struct drm_atomic_state *state,
					  struct drm_plane_state **plane_states,
					  int n)
{
	struct dpu95_drm_device *dpu_drm = to_dpu95_drm_device(crtc->dev);
	struct dpu95_crtc *dpu_crtc = to_dpu95_crtc(crtc);
	const struct dpu95_layerblend_ops *lb_ops;
	const struct dpu95_fetchunit_ops *fu_ops;
	unsigned int sid = dpu_crtc->stream_id;
	struct drm_plane_state *plane_state;
	struct dpu95_plane_state *dpstate;
	struct drm_private_obj *manager;
	struct dpu95_layerblend *blend;
	u32 src_w, src_h, dst_w, dst_h;
	union dpu95_plane_stage stage;
	struct dpu95_plane_grp *grp;
	struct dpu95_plane_res *res;
	struct dpu95_plane *dplane;
	struct drm_framebuffer *fb;
	struct dpu95_fetchunit *fu;
	struct dpu95_hscaler *hs;
	struct dpu95_vscaler *vs;
	struct drm_plane *plane;
	bool fb_is_packed_yuv422;
	struct list_head *node;
	bool found_fu;
	bool need_fe;
	bool need_hs;
	bool need_vs;
	u32 cap_mask;
	int i, j;
	int ret;

	/* for active planes only */
	for (i = 0; i < n; i++) {
		plane_state = plane_states[i];
		dpstate = to_dpu95_plane_state(plane_state);
		plane = plane_state->plane;
		dplane = to_dpu95_plane(plane);
		fb = plane_state->fb;
		grp = dplane->grp;
		res = &grp->res;

		src_w = plane_state->src_w >> 16;
		src_h = plane_state->src_h >> 16;
		dst_w = plane_state->crtc_w;
		dst_h = plane_state->crtc_h;

		fb_is_packed_yuv422 =
				drm_format_info_is_yuv_packed(fb->format) &&
				drm_format_info_is_yuv_sampling_422(fb->format);
		need_fe = fb->format->num_planes > 1;
		need_hs = src_w != dst_w;
		need_vs = (src_h != dst_h);

		cap_mask = 0;
		if (need_fe)
			cap_mask |= DPU95_FETCHUNIT_CAP_USE_FETCHECO;
		if (need_hs)
			cap_mask |= DPU95_FETCHUNIT_CAP_USE_HSCALER;
		if (need_vs)
			cap_mask |= DPU95_FETCHUNIT_CAP_USE_VSCALER4;
		if (fb_is_packed_yuv422)
			cap_mask |= DPU95_FETCHUNIT_CAP_PACKED_YUV422;

		/* assign source */
		found_fu = false;
		list_for_each(node, &grp->fu_list) {
			fu = dpu95_fu_get_from_list(node);

			fu_ops = dpu95_fu_get_ops(fu);

			/* enough capability? */
			if ((cap_mask & fu_ops->get_cap_mask(fu)) != cap_mask)
				continue;

			if (need_hs) {
				hs = dpu95_plane_assign_hscaler(crtc, state, plane, fu);
				if (IS_ERR(hs))
					return PTR_ERR(hs);

				dpstate->hs = hs;
			} else {
				dpstate->hs = NULL;
			}

			if (need_vs) {
				vs = dpu95_plane_assign_vscaler(crtc, state, plane, fu);
				if (IS_ERR(vs))
					return PTR_ERR(vs);

				dpstate->vs = vs;
			} else {
				dpstate->vs = NULL;
			}

			manager = fu_ops->get_manager(fu);

			ret = dpu95_private_obj_assign(plane, crtc, state, manager);
			if (ret)
				continue;

			found_fu = true;
			break;
		}

		if (!found_fu) {
			dpu95_plane_dbg(plane,
					"failed to find fetchunit on stream%u\n",
					sid);
			return -EINVAL;
		}

		dpstate->source = fu;

		/* assign stage and blend */
		if (sid) {
			j = dpu_drm->dpu_hw_plane_cnt - (n - i);
			blend = res->lb[j];
			if (i == 0)
				stage.cf = grp->cf[sid];
			else
				stage.lb = res->lb[j - 1];
		} else {
			blend = res->lb[i];
			if (i == 0)
				stage.cf = grp->cf[sid];
			else
				stage.lb = res->lb[i - 1];
		}

		lb_ops = dpu95_lb_get_ops(blend);
		manager = lb_ops->get_manager(blend);

		ret = dpu95_private_obj_assign(plane, crtc, state, manager);
		if (ret) {
			dpu95_plane_dbg(plane,
					"failed to assign LayerBlend%u on stream%u\n",
					dpu95_lb_get_id(blend), sid);
			return ret;
		}

		dpstate->stage = stage;
		dpstate->blend = blend;
	}

	return 0;
}

static int dpu95_atomic_assign_plane_source(struct drm_atomic_state *state)
{
	struct dpu95_drm_device *dpu_drm = to_dpu95_drm_device(state->dev);
	struct drm_plane_state *old_plane_state;
	struct drm_plane_state **plane_states;
	struct drm_crtc_state *crtc_state;
	struct drm_plane *plane;
	struct drm_crtc *crtc;
	int ret, i, n;

	/*
	 * For any plane in previous active status, disable all private
	 * states for that plane.
	 */
	for_each_old_plane_in_state(state, plane, old_plane_state, i) {
		const struct dpu95_fetchunit_ops *fu_ops;
		const struct dpu95_layerblend_ops *lb_ops;
		struct dpu95_plane_state *old_dpstate;
		struct drm_private_obj *manager;

		if (!old_plane_state->fb || !old_plane_state->crtc)
			continue;

		old_dpstate = to_dpu95_plane_state(old_plane_state);
		if (WARN_ON(!old_dpstate->source || !old_dpstate->blend))
			return -EINVAL;

		fu_ops = dpu95_fu_get_ops(old_dpstate->source);
		manager = fu_ops->get_manager(old_dpstate->source);

		ret = dpu95_private_obj_disable(state, manager);
		if (ret) {
			dpu95_plane_dbg(plane,
					"failed to disable %s private state: %d\n",
					fu_ops->get_name(old_dpstate->source),
					ret);
			return ret;
		}

		lb_ops = dpu95_lb_get_ops(old_dpstate->blend);
		manager = lb_ops->get_manager(old_dpstate->blend);

		ret = dpu95_private_obj_disable(state, manager);
		if (ret) {
			dpu95_plane_dbg(plane,
					"failed to disable LayerBlend%u private state: %d\n",
					dpu95_lb_get_id(old_dpstate->blend),
					ret);
			return ret;
		}

		if (old_dpstate->hs) {
			const struct dpu95_hscaler_ops *hs_ops;

			hs_ops = dpu95_hs_get_ops(old_dpstate->hs);
			manager = hs_ops->get_manager(old_dpstate->hs);

			ret = dpu95_private_obj_disable(state, manager);
			if (ret) {
				dpu95_plane_dbg(plane,
						"failed to disable HScaler%u private state: %d\n",
						dpu95_hs_get_id(old_dpstate->hs), ret);
				return ret;
			}
		}

		if (old_dpstate->vs) {
			const struct dpu95_vscaler_ops *vs_ops;

			vs_ops = dpu95_vs_get_ops(old_dpstate->vs);
			manager = vs_ops->get_manager(old_dpstate->vs);

			ret = dpu95_private_obj_disable(state, manager);
			if (ret) {
				dpu95_plane_dbg(plane,
						"failed to disable VScaler%u private state: %d\n",
						dpu95_vs_get_id(old_dpstate->vs), ret);
				return ret;
			}
		}
	}

	for_each_new_crtc_in_state(state, crtc, crtc_state, i) {
		/* Skip if no active plane. */
		if (crtc_state->plane_mask == 0)
			continue;

		plane_states = kmalloc_array(dpu_drm->dpu_hw_plane_cnt,
					     sizeof(*plane_states), GFP_KERNEL);
		if (!plane_states) {
			ret = -ENOMEM;
			dpu95_crtc_dbg(crtc,
				       "failed to alloc plane state ptrs: %d\n",
				       ret);
			return ret;
		}

		n = dpu95_atomic_sort_planes_per_crtc(crtc_state, plane_states);
		if (n < 0) {
			dpu95_crtc_dbg(crtc, "failed to sort planes: %d\n", n);
			kfree(plane_states);
			return n;
		}

		dpu95_atomic_set_top_plane_per_crtc(plane_states, n);

		ret = dpu95_atomic_assign_plane_source_per_crtc(crtc, state,
								plane_states, n);
		if (ret) {
			dpu95_crtc_dbg(crtc,
				       "failed to assign resource to plane: %d\n",
				       ret);
			kfree(plane_states);
			return ret;
		}

		kfree(plane_states);
	}

	return 0;
}

static int dpu95_drm_atomic_check(struct drm_device *dev,
				  struct drm_atomic_state *state)
{
	struct drm_crtc_state *crtc_state;
	struct drm_crtc *crtc;
	int i, ret;

	ret = drm_atomic_helper_check_modeset(dev, state);
	if (ret)
		return ret;

	for_each_new_crtc_in_state(state, crtc, crtc_state, i) {
		ret = drm_atomic_add_affected_planes(state, crtc);
		if (ret)
			return ret;
	}

	ret = drm_atomic_normalize_zpos(dev, state);
	if (ret) {
		drm_dbg_kms(dev, "failed to normalize zpos: %d\n", ret);
		return ret;
	}

	/*
	 * Assign HW resources to planes in question.
	 * It is likely to fail due to some reasons, e.g., no enough
	 * fetchunits, users ask for more features than the HW resources
	 * can provide, HW resource hot-migration bewteen CRTCs is needed.
	 */
	ret = dpu95_atomic_assign_plane_source(state);
	if (ret) {
		drm_dbg_kms(dev, "failed to assign source to plane: %d\n", ret);
		return ret;
	}

	return drm_atomic_helper_check_planes(dev, state);
}

static const struct drm_mode_config_funcs dpu95_drm_mode_config_funcs = {
	.fb_create	= drm_gem_fb_create,
	.atomic_check	= dpu95_drm_atomic_check,
	.atomic_commit	= drm_atomic_helper_commit,
};

static void dpu95_get_private_obj_name(const struct dpu95_private_obj *priv_obj,
				       char *name)
{
	memset(name, 0, DPU95_PRIVATE_OBJ_NAME_SIZE);

	switch (priv_obj->type) {
	case DPU95_PRIVATE_OBJ_FU: {
		const struct dpu95_fetchunit_ops *fu_ops;

		fu_ops = dpu95_fu_get_ops(priv_obj->res.fu);
		strscpy(name, fu_ops->get_name(priv_obj->res.fu),
			sizeof(priv_obj->res.fu->name));
		break;
	}
	case DPU95_PRIVATE_OBJ_LB:
		snprintf(name, DPU95_PRIVATE_OBJ_NAME_SIZE, "LayerBlend%u",
			 dpu95_lb_get_id(priv_obj->res.lb));
		break;
	case DPU95_PRIVATE_OBJ_HS:
		snprintf(name, DPU95_PRIVATE_OBJ_NAME_SIZE, "HScaler%u",
			 dpu95_hs_get_id(priv_obj->res.hs));
		break;
	case DPU95_PRIVATE_OBJ_VS:
		snprintf(name, DPU95_PRIVATE_OBJ_NAME_SIZE, "VScaler%u",
			 dpu95_vs_get_id(priv_obj->res.vs));
		break;
	default:
		WARN(true, "unknown private object type%d\n", priv_obj->type);
	}
}

static int dpu95_drm_atomic_commit_setup(struct drm_atomic_state *state)
{
	struct dpu95_drm_device *dpu_drm = to_dpu95_drm_device(state->dev);
	struct drm_private_state *old_obj_state, *new_obj_state;
	struct dpu95_private_state *new_private_state;
	struct dpu95_private_state *old_private_state;
	struct drm_crtc_state *crtc_state;
	struct drm_private_obj *obj;
	u32 crtc_mask_in_state = 0;
	struct drm_crtc *crtc;
	int i;

	for_each_new_crtc_in_state(state, crtc, crtc_state, i)
		crtc_mask_in_state |= drm_crtc_mask(crtc);

	for_each_oldnew_private_obj_in_state(state, obj, old_obj_state, new_obj_state, i) {
		struct dpu95_private_obj *priv_obj;
		char obj_name[DPU95_PRIVATE_OBJ_NAME_SIZE];

		if (!is_dpu95_private_obj(dpu_drm, obj))
			continue;

		new_private_state = to_dpu95_private_state(new_obj_state);
		old_private_state = to_dpu95_private_state(old_obj_state);

		if (!old_private_state->crtc && !new_private_state->crtc)
			continue;

		priv_obj = to_dpu95_private_obj(obj);
		dpu95_get_private_obj_name(priv_obj, obj_name);

		/* Add pending commit if an object is being disabled. */
		if (old_private_state->crtc && !new_private_state->crtc &&
		    (crtc_mask_in_state & drm_crtc_mask(old_private_state->crtc))) {
			crtc_state = drm_atomic_get_new_crtc_state(state,
								   old_private_state->crtc);
			if (WARN_ON(!crtc_state))
				return -EINVAL;

			if (crtc_state->commit) {
				new_private_state->pending_commit =
					drm_crtc_commit_get(crtc_state->commit);

				dpu95_crtc_dbg(crtc_state->crtc,
					       "add pending commit for disabling %s\n",
					       obj_name);
			}
		}

		/* Add pending commit if an object is being used. */
		if (new_private_state->crtc &&
		    (crtc_mask_in_state & drm_crtc_mask(new_private_state->crtc))) {
			crtc_state = drm_atomic_get_new_crtc_state(state,
								   new_private_state->crtc);
			if (WARN_ON(!crtc_state))
				return -EINVAL;

			if (crtc_state->commit) {
				new_private_state->pending_commit =
					drm_crtc_commit_get(crtc_state->commit);

				dpu95_crtc_dbg(crtc_state->crtc,
					       "add pending commit for using %s\n",
					       obj_name);
			}
		}
	}

	return 0;
}

static void dpu95_drm_atomic_commit_tail(struct drm_atomic_state *state)
{
	struct dpu95_drm_device *dpu_drm = to_dpu95_drm_device(state->dev);
	struct dpu95_private_state *old_private_state;
	struct drm_private_state *old_obj_state;
	struct drm_crtc_commit *commit;
	struct drm_private_obj *obj;
	int i, ret;

	for_each_old_private_obj_in_state(state, obj, old_obj_state, i) {
		struct dpu95_private_obj *priv_obj;
		char obj_name[DPU95_PRIVATE_OBJ_NAME_SIZE];

		if (!is_dpu95_private_obj(dpu_drm, obj))
			continue;

		old_private_state = to_dpu95_private_state(old_obj_state);

		commit = old_private_state->pending_commit;
		if (!commit)
			continue;

		priv_obj = to_dpu95_private_obj(obj);
		dpu95_get_private_obj_name(priv_obj, obj_name);

		dpu95_crtc_dbg(commit->crtc, "wait for pending commit for %s\n",
			       obj_name);

		ret = drm_crtc_commit_wait(commit);
		if (ret)
			dpu95_crtc_err(commit->crtc,
				       "timed out waiting for commit for %s\n",
				       obj_name);

		drm_crtc_commit_put(commit);
		old_private_state->pending_commit = NULL;
	}

	drm_atomic_helper_commit_tail(state);
}

static struct drm_mode_config_helper_funcs dpu95_drm_mode_config_helpers = {
	.atomic_commit_setup	= dpu95_drm_atomic_commit_setup,
	.atomic_commit_tail	= dpu95_drm_atomic_commit_tail,
};

static int dpu95_kms_init_encoder_per_crtc(struct dpu95_drm_device *dpu_drm,
					   struct dpu95_crtc *dpu_crtc)
{
	struct drm_device *drm = &dpu_drm->base;
	struct drm_crtc *crtc = &dpu_crtc->base;
	struct drm_connector *connector;
	struct drm_encoder *encoder;
	struct drm_bridge *bridge;
	int ret;

	bridge = devm_drm_of_get_bridge(drm->dev, drm->dev->of_node,
					dpu_crtc->stream_id, 0);
	if (!bridge) {
		ret = -EPROBE_DEFER;
		drm_dbg_kms(drm, "failed to find bridge for stream%u: %d\n",
			    dpu_crtc->stream_id, ret);
		goto out;
	} else if (IS_ERR(bridge)) {
		ret = PTR_ERR(bridge);
		if (ret == -ENODEV)
			return 0;
		else
			return ret;
	}

	encoder = &dpu_drm->encoder[dpu_crtc->stream_id];
	ret = drm_simple_encoder_init(drm, encoder, DRM_MODE_ENCODER_NONE);
	if (ret) {
		drm_err(drm, "failed to initialize encoder for stream%u: %d\n",
			dpu_crtc->stream_id, ret);
		goto out;
	}

	encoder->possible_crtcs = drm_crtc_mask(crtc);

	ret = drm_bridge_attach(encoder, bridge, NULL,
				DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret) {
		drm_err(drm,
			"failed to attach bridge to encoder for stream%u: %d\n",
			dpu_crtc->stream_id, ret);
		goto out;
	}

	connector = drm_bridge_connector_init(drm, encoder);
	if (IS_ERR(connector)) {
		ret = PTR_ERR(connector);
		drm_err(drm, "failed to initialize bridge connector for stream%u: %d\n",
			dpu_crtc->stream_id, ret);
		goto out;
	}

	ret = drm_connector_attach_encoder(connector, encoder);
	if (ret)
		drm_err(drm, "failed to attach encoder to connector for stream%u: %d\n",
			dpu_crtc->stream_id, ret);

out:
	return ret;
}

static struct drm_private_state *
dpu95_private_duplicate_state(struct drm_private_obj *obj)
{
	struct dpu95_private_state *old_state = to_dpu95_private_state(obj->state);
	struct dpu95_private_state *state;

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return NULL;

	__drm_atomic_helper_private_obj_duplicate_state(obj, &state->base);

	state->plane = old_state->plane;
	state->crtc = old_state->crtc;

	return &state->base;
}

static void dpu95_private_destroy_state(struct drm_private_obj *obj,
					struct drm_private_state *state)
{
	struct dpu95_private_state *private_state = to_dpu95_private_state(state);

	if (private_state->pending_commit)
		drm_crtc_commit_put(private_state->pending_commit);

	kfree(private_state);
}

static void dpu95_private_print_state(struct drm_printer *p,
				      const struct drm_private_state *state)
{
	const struct dpu95_private_state *private_state = to_dpu95_private_state(state);
	const struct dpu95_private_obj *priv_obj = to_dpu95_private_obj(state->obj);
	char obj_name[DPU95_PRIVATE_OBJ_NAME_SIZE];

	dpu95_get_private_obj_name(priv_obj, obj_name);

	drm_printf(p, "%s State\n", obj_name);
	drm_printf(p, "\tplane=%s\n",
		   private_state->plane ? private_state->plane->name : "null");
	drm_printf(p, "\tcrtc=%s\n",
		   private_state->crtc ? private_state->crtc->name : "null");
}

static const struct drm_private_state_funcs dpu95_private_state_funcs = {
	.atomic_duplicate_state = dpu95_private_duplicate_state,
	.atomic_destroy_state = dpu95_private_destroy_state,
	.atomic_print_state = dpu95_private_print_state,
};

static void dpu95_private_obj_fini(struct drm_device *dev, void *data)
{
	struct drm_private_obj *manager = data;

	drm_atomic_private_obj_fini(manager);
}

static int __dpu95_private_obj_initialize(struct dpu95_drm_device *dpu_drm,
					  struct dpu95_private_obj *priv_obj,
					  enum dpu95_private_obj_type type)
{
	struct drm_device *drm = &dpu_drm->base;
	struct dpu95_private_state *private_state;

	private_state = kzalloc(sizeof(*private_state), GFP_KERNEL);
	if (!private_state)
		return -ENOMEM;

	list_add(&priv_obj->node, &dpu_drm->obj_list);

	priv_obj->type = type;

	drm_atomic_private_obj_init(drm, &priv_obj->base, &private_state->base,
				    &dpu95_private_state_funcs);

	return drmm_add_action_or_reset(drm, dpu95_private_obj_fini,
					&priv_obj->base);
}

static int dpu95_private_obj_initialize(struct dpu95_drm_device *dpu_drm)
{
	struct dpu95_plane_grp *plane_grp = &dpu_drm->dpu_plane_grp;
	const struct dpu95_layerblend_ops *lb_ops;
	const struct dpu95_fetchunit_ops *fu_ops;
	const struct dpu95_hscaler_ops *hs_ops;
	const struct dpu95_vscaler_ops *vs_ops;
	struct dpu95_layerblend *lb;
	struct dpu95_fetchunit *fu;
	struct list_head *node;
	int i, ret;

	INIT_LIST_HEAD(&dpu_drm->obj_list);

	/*
	 * Initialize H/W resources in a particular order so that private
	 * states could be printed out from debugfs in a fashion close to
	 * pixel engine block diagram.
	 */
	for (i = 0; i < dpu_drm->dpu_hw_plane_cnt; i++) {
		ret = __dpu95_private_obj_initialize(dpu_drm,
						     &dpu_drm->fu_manager[i],
						     DPU95_PRIVATE_OBJ_FU);
		if (ret)
			return ret;
	}

	ret = __dpu95_private_obj_initialize(dpu_drm, &dpu_drm->hs_manager,
					     DPU95_PRIVATE_OBJ_HS);
	if (ret)
		return ret;

	ret = __dpu95_private_obj_initialize(dpu_drm, &dpu_drm->vs_manager,
					     DPU95_PRIVATE_OBJ_VS);
	if (ret)
		return ret;

	for (i = 0; i < dpu_drm->dpu_hw_plane_cnt; i++) {
		ret = __dpu95_private_obj_initialize(dpu_drm,
						     &dpu_drm->lb_manager[i],
						     DPU95_PRIVATE_OBJ_LB);
		if (ret)
			return ret;
	}

	i = 0;
	list_for_each(node, &plane_grp->fu_list) {
		fu = dpu95_fu_get_from_list(node);

		fu_ops = dpu95_fu_get_ops(fu);
		fu_ops->set_manager(fu, &dpu_drm->fu_manager[i].base);
		dpu_drm->fu_manager[i].res.fu = fu;

		i++;
	}

	hs_ops = dpu95_hs_get_ops(plane_grp->hs);
	hs_ops->set_manager(plane_grp->hs, &dpu_drm->hs_manager.base);
	dpu_drm->hs_manager.res.hs = plane_grp->hs;

	vs_ops = dpu95_vs_get_ops(plane_grp->vs);
	vs_ops->set_manager(plane_grp->vs, &dpu_drm->vs_manager.base);
	dpu_drm->vs_manager.res.vs = plane_grp->vs;

	i = 0;
	list_for_each(node, &plane_grp->lb_list) {
		lb = dpu95_lb_get_from_list(node);

		lb_ops = dpu95_lb_get_ops(lb);
		lb_ops->set_manager(lb, &dpu_drm->lb_manager[i].base);
		dpu_drm->lb_manager[i].res.lb = lb;

		i++;
	}

	return 0;
}

int dpu95_kms_prepare(struct dpu95_drm_device *dpu_drm)
{
	struct dpu95_soc *dpu = &dpu_drm->dpu_soc;
	struct drm_device *drm = &dpu_drm->base;
	struct dpu95_plane *dpu_overlay;
	struct dpu95_crtc *dpu_crtc;
	int ret, i;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;

	drm->mode_config.min_width = 60;
	drm->mode_config.min_height = 60;
	drm->mode_config.max_width = 8192;
	drm->mode_config.max_height = 8192;
	drm->mode_config.funcs = &dpu95_drm_mode_config_funcs;
	drm->mode_config.helper_private = &dpu95_drm_mode_config_helpers;
	drm->max_vblank_count = DPU95_FRAMEGEN_MAX_FRAME_INDEX;

	dpu_drm->dpu_hw_plane_cnt = dpu->fl_cnt + dpu->fy_cnt;

	for (i = 0; i < DPU95_CRTCS; i++) {
		dpu_crtc = &dpu_drm->dpu_crtc[i];

		ret = dpu95_crtc_init(dpu_drm, dpu_crtc, i);
		if (ret)
			return ret;

		ret = dpu95_kms_init_encoder_per_crtc(dpu_drm, dpu_crtc);
		if (ret)
			return ret;
	}

	dpu_drm->dpu_overlay_cnt = dpu_drm->dpu_hw_plane_cnt - 1;
	dpu_drm->dpu_overlay = drmm_kcalloc(drm, dpu_drm->dpu_overlay_cnt,
					    sizeof(struct dpu95_plane),
					    GFP_KERNEL);
	if (!dpu_drm->dpu_overlay)
		return -ENOMEM;

	for (i = 0; i < dpu_drm->dpu_overlay_cnt; i++) {
		dpu_overlay = &dpu_drm->dpu_overlay[i];

		ret = dpu95_plane_initialize(dpu_drm, dpu_overlay,
					     dpu_drm->crtc_mask,
					     DRM_PLANE_TYPE_OVERLAY);
		if (ret) {
			drm_err(drm, "failed to init overlay plane%d: %d\n",
				i, ret);
			return ret;
		}
	}

	dpu_drm->fu_manager = drmm_kcalloc(drm, dpu_drm->dpu_hw_plane_cnt,
					   sizeof(struct dpu95_private_obj),
					   GFP_KERNEL);
	if (!dpu_drm->fu_manager)
		return -ENOMEM;

	dpu_drm->lb_manager = drmm_kcalloc(drm, dpu_drm->dpu_hw_plane_cnt,
					   sizeof(struct dpu95_private_obj),
					   GFP_KERNEL);
	if (!dpu_drm->lb_manager)
		return -ENOMEM;

	ret = dpu95_private_obj_initialize(dpu_drm);
	if (ret) {
		drm_err(drm, "failed to init drm private objs: %d\n", ret);
		return ret;
	}

	ret = drm_vblank_init(drm, DPU95_CRTCS);
	if (ret) {
		drm_err(drm, "failed to initialize vblank support: %d\n", ret);
		return ret;
	}

	drm_mode_config_reset(drm);

	drm_kms_helper_poll_init(drm);

	return 0;
}

void dpu95_kms_unprepare(struct dpu95_drm_device *dpu_drm)
{
	drm_kms_helper_poll_fini(&dpu_drm->base);
}
