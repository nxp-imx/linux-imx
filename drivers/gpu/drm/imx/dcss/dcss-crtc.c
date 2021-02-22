// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP.
 */

#include <drm/drm_atomic_helper.h>
#include <drm/drm_vblank.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include "dcss-dev.h"
#include "dcss-kms.h"

static int dcss_enable_vblank(struct drm_crtc *crtc)
{
	struct dcss_crtc *dcss_crtc = container_of(crtc, struct dcss_crtc,
						   base);
	struct dcss_dev *dcss = crtc->dev->dev_private;

	if (dcss_crtc->irq_enabled)
		return 0;

	dcss_crtc->irq_enabled = true;

	dcss_dtg_vblank_irq_enable(dcss->dtg, true);

	dcss_dtg_ctxld_kick_irq_enable(dcss->dtg, true);

	enable_irq(dcss_crtc->irq);

	return 0;
}

static void dcss_disable_vblank(struct drm_crtc *crtc)
{
	struct dcss_crtc *dcss_crtc = container_of(crtc, struct dcss_crtc,
						   base);
	struct dcss_dev *dcss = dcss_crtc->base.dev->dev_private;

	disable_irq_nosync(dcss_crtc->irq);

	dcss_dtg_vblank_irq_enable(dcss->dtg, false);

	if (!dcss_dtrc_is_running(dcss->dtrc))
		dcss_dtg_ctxld_kick_irq_enable(dcss->dtg, false);

	dcss_crtc->irq_enabled = false;
}

static const struct drm_crtc_funcs dcss_crtc_funcs = {
	.set_config = drm_atomic_helper_set_config,
	.destroy = drm_crtc_cleanup,
	.page_flip = drm_atomic_helper_page_flip,
	.reset = drm_atomic_helper_crtc_reset,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = dcss_enable_vblank,
	.disable_vblank = dcss_disable_vblank,
};

static void dcss_crtc_atomic_begin(struct drm_crtc *crtc,
				   struct drm_crtc_state *old_crtc_state)
{
	drm_crtc_vblank_on(crtc);

	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->event) {
		WARN_ON(drm_crtc_vblank_get(crtc));
		drm_crtc_arm_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);
}

static void dcss_crtc_atomic_flush(struct drm_crtc *crtc,
				   struct drm_crtc_state *old_crtc_state)
{
	struct dcss_crtc *dcss_crtc = container_of(crtc, struct dcss_crtc,
						   base);
	struct dcss_dev *dcss = dcss_crtc->base.dev->dev_private;

	if (dcss_dtg_is_enabled(dcss->dtg))
		dcss_ctxld_enable(dcss->ctxld);
}

static void dcss_crtc_atomic_enable(struct drm_crtc *crtc,
				    struct drm_crtc_state *old_crtc_state)
{
	struct dcss_crtc *dcss_crtc = container_of(crtc, struct dcss_crtc,
						   base);
	struct dcss_dev *dcss = dcss_crtc->base.dev->dev_private;
	struct drm_display_mode *mode = &crtc->state->adjusted_mode;
	struct videomode vm;

	drm_display_mode_to_videomode(mode, &vm);

	pm_runtime_get_sync(dcss->dev);

	vm.pixelclock = mode->crtc_clock * 1000;

	dcss_dtg_sync_set(dcss->dtg, &vm);

	dcss_ss_subsam_set(dcss->ss, dcss_crtc->output_is_yuv);
	dcss_ss_sync_set(dcss->ss, &vm, mode->flags & DRM_MODE_FLAG_PHSYNC,
			 mode->flags & DRM_MODE_FLAG_PVSYNC);

	dcss_dtg_css_set(dcss->dtg, dcss_crtc->output_is_yuv);

	dcss_ss_enable(dcss->ss);
	dcss_dtg_enable(dcss->dtg, true, NULL);
	dcss_ctxld_enable(dcss->ctxld);

	dcss_enable_vblank(crtc);

	reinit_completion(&dcss_crtc->en_completion);
	wait_for_completion_timeout(&dcss_crtc->en_completion,
				    msecs_to_jiffies(500));
}

static void dcss_crtc_atomic_disable(struct drm_crtc *crtc,
				     struct drm_crtc_state *old_crtc_state)
{
	struct dcss_crtc *dcss_crtc = container_of(crtc, struct dcss_crtc,
						   base);
	struct dcss_dev *dcss = dcss_crtc->base.dev->dev_private;

	drm_atomic_helper_disable_planes_on_crtc(old_crtc_state, false);

	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->event) {
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);

	dcss_dtg_ctxld_kick_irq_enable(dcss->dtg, true);

	dcss_ss_disable(dcss->ss);
	dcss_dtg_enable(dcss->dtg, false, &dcss_crtc->dis_completion);
	dcss_ctxld_enable(dcss->ctxld);

	reinit_completion(&dcss_crtc->dis_completion);
	wait_for_completion_timeout(&dcss_crtc->dis_completion,
				    msecs_to_jiffies(100));

	drm_crtc_vblank_off(crtc);

	dcss_dtg_ctxld_kick_irq_enable(dcss->dtg, false);

	pm_runtime_put_sync(dcss->dev);
}

static enum drm_mode_status dcss_crtc_mode_valid(struct drm_crtc *crtc,
                        const struct drm_display_mode *mode)
{
   /*
    * From DCSS perspective, dissallow any mode higher than
    * 3840x2160 or 2160x3840.
    */
   if (mode->hdisplay * mode->vdisplay > 3840 * 2160)
       return MODE_BAD;

   return MODE_OK;
}

static const struct drm_crtc_helper_funcs dcss_helper_funcs = {
	.atomic_begin = dcss_crtc_atomic_begin,
	.atomic_flush = dcss_crtc_atomic_flush,
	.atomic_enable = dcss_crtc_atomic_enable,
	.atomic_disable = dcss_crtc_atomic_disable,
	.mode_valid = dcss_crtc_mode_valid,
};

static irqreturn_t dcss_crtc_irq_handler(int irq, void *dev_id)
{
	struct dcss_crtc *dcss_crtc = dev_id;
	struct dcss_dev *dcss = dcss_crtc->base.dev->dev_private;

	if (!dcss_dtg_vblank_irq_valid(dcss->dtg))
		return IRQ_HANDLED;

	complete(&dcss_crtc->en_completion);

	if (dcss_ctxld_is_flushed(dcss->ctxld))
		drm_crtc_handle_vblank(&dcss_crtc->base);

	dcss_dtg_vblank_irq_clear(dcss->dtg);

	return IRQ_HANDLED;
}

int dcss_crtc_init(struct dcss_crtc *crtc, struct drm_device *drm)
{
	struct dcss_dev *dcss = drm->dev_private;
	struct platform_device *pdev = to_platform_device(dcss->dev);
	int ret;

	crtc->plane[0] = dcss_plane_init(drm, drm_crtc_mask(&crtc->base),
					 DRM_PLANE_TYPE_PRIMARY, 2);
	if (IS_ERR(crtc->plane[0]))
		return PTR_ERR(crtc->plane[0]);

	crtc->base.port = dcss->of_port;

	drm_crtc_helper_add(&crtc->base, &dcss_helper_funcs);
	ret = drm_crtc_init_with_planes(drm, &crtc->base, &crtc->plane[0]->base,
					NULL, &dcss_crtc_funcs, NULL);
	if (ret) {
		dev_err(dcss->dev, "failed to init crtc\n");
		return ret;
	}

	crtc->plane[1] = dcss_plane_init(drm, drm_crtc_mask(&crtc->base),
					 DRM_PLANE_TYPE_OVERLAY, 1);
	if (IS_ERR(crtc->plane[1]))
		crtc->plane[1] = NULL;

	crtc->plane[2] = dcss_plane_init(drm, drm_crtc_mask(&crtc->base),
					 DRM_PLANE_TYPE_OVERLAY, 0);
	if (IS_ERR(crtc->plane[2]))
		crtc->plane[2] = NULL;

	drm_plane_create_alpha_property(&crtc->plane[0]->base);

	crtc->irq = platform_get_irq_byname(pdev, "vblank");
	if (crtc->irq < 0) {
		dev_err(dcss->dev, "unable to get vblank interrupt\n");
		return crtc->irq;
	}

	init_completion(&crtc->en_completion);
	init_completion(&crtc->dis_completion);

	ret = devm_request_irq(dcss->dev, crtc->irq, dcss_crtc_irq_handler,
			       IRQF_TRIGGER_RISING, "dcss_drm", crtc);
	if (ret) {
		dev_err(dcss->dev, "irq request failed with %d.\n", ret);
		return ret;
	}

	disable_irq(crtc->irq);

	return 0;
}

void dcss_crtc_attach_color_mgmt_properties(struct dcss_crtc *crtc)
{
	int i;

	/* create color management properties only for video planes */
	for (i = 1; i < 3; i++) {
		if (crtc->plane[i]->type == DRM_PLANE_TYPE_PRIMARY)
			return;

		drm_plane_create_color_properties(&crtc->plane[i]->base,
					BIT(DRM_COLOR_YCBCR_BT601) |
					BIT(DRM_COLOR_YCBCR_BT709) |
					BIT(DRM_COLOR_YCBCR_BT2020),
					BIT(DRM_COLOR_YCBCR_FULL_RANGE) |
					BIT(DRM_COLOR_YCBCR_LIMITED_RANGE),
					DRM_COLOR_YCBCR_BT709,
					DRM_COLOR_YCBCR_FULL_RANGE);
	}
}

void dcss_crtc_deinit(struct dcss_crtc *crtc, struct drm_device *drm)
{
	struct dcss_dev *dcss = drm->dev_private;

	devm_free_irq(dcss->dev, crtc->irq, crtc);
}
