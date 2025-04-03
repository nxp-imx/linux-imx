// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright 2025 NXP
 */

#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/media-bus-format.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_plane.h>
#include <drm/drm_print.h>
#include <drm/drm_vblank.h>

#include "dcif-crc.h"
#include "dcif-drv.h"
#include "dcif-reg.h"

#define DCIF_MAX_PIXEL_CLOCK		148500000

/* -----------------------------------------------------------------------------
 * CRTC
 */

/*
 * For conversion from YCbCr to RGB, the CSC operates as follows:
 *
 * |R|   |A1 A2 A3|   |Y  + D1|
 * |G| = |B1 B2 B3| * |Cb + D2|
 * |B|   |C1 C2 C3|   |Cr + D3|
 *
 * The A, B and C coefficients are expressed as Q2.8 fixed point values, and
 * the D coefficients as Q0.8. Despite the reference manual stating the
 * opposite, the D1, D2 and D3 offset values are added to Y, Cb and Cr, not
 * subtracted. They must thus be programmed with negative values.
 */
static const u32 dcif_yuv2rgb_coeffs[3][2][6] = {
	[DRM_COLOR_YCBCR_BT601] = {
		[DRM_COLOR_YCBCR_LIMITED_RANGE] = {
			/*
			 * BT.601 limited range:
			 *
			 * |R|   |1.1644  0.0000  1.5960|   |Y  - 16 |
			 * |G| = |1.1644 -0.3917 -0.8129| * |Cb - 128|
			 * |B|   |1.1644  2.0172  0.0000|   |Cr - 128|
			 */
			DCIF_CSC_COEF0_L0_A1(0x12a) | DCIF_CSC_COEF0_L0_A2(0x000),
			DCIF_CSC_COEF1_L0_A3(0x199) | DCIF_CSC_COEF1_L0_B1(0x12a),
			DCIF_CSC_COEF2_L0_B2(0x79c) | DCIF_CSC_COEF2_L0_B3(0x730),
			DCIF_CSC_COEF3_L0_C1(0x12a) | DCIF_CSC_COEF3_L0_C2(0x204),
			DCIF_CSC_COEF4_L0_C3(0x000) | DCIF_CSC_COEF4_L0_D1(0x1f0),
			DCIF_CSC_COEF5_L0_D2(0x180) | DCIF_CSC_COEF5_L0_D3(0x180),
		},
		[DRM_COLOR_YCBCR_FULL_RANGE] = {
			/*
			 * BT.601 full range:
			 *
			 * |R|   |1.0000  0.0000  1.4020|   |Y  - 0  |
			 * |G| = |1.0000 -0.3441 -0.7141| * |Cb - 128|
			 * |B|   |1.0000  1.7720  0.0000|   |Cr - 128|
			 */
			DCIF_CSC_COEF0_L0_A1(0x100) | DCIF_CSC_COEF0_L0_A2(0x000),
			DCIF_CSC_COEF1_L0_A3(0x167) | DCIF_CSC_COEF1_L0_B1(0x100),
			DCIF_CSC_COEF2_L0_B2(0x7a8) | DCIF_CSC_COEF2_L0_B3(0x749),
			DCIF_CSC_COEF3_L0_C1(0x100) | DCIF_CSC_COEF3_L0_C2(0x1c6),
			DCIF_CSC_COEF4_L0_C3(0x000) | DCIF_CSC_COEF4_L0_D1(0x000),
			DCIF_CSC_COEF5_L0_D2(0x180) | DCIF_CSC_COEF5_L0_D3(0x180),
		},
	},
	[DRM_COLOR_YCBCR_BT709] = {
		[DRM_COLOR_YCBCR_LIMITED_RANGE] = {
			/*
			 * Rec.709 limited range:
			 *
			 * |R|   |1.1644  0.0000  1.7927|   |Y  - 16 |
			 * |G| = |1.1644 -0.2132 -0.5329| * |Cb - 128|
			 * |B|   |1.1644  2.1124  0.0000|   |Cr - 128|
			 */
			DCIF_CSC_COEF0_L0_A1(0x12a) | DCIF_CSC_COEF0_L0_A2(0x000),
			DCIF_CSC_COEF1_L0_A3(0x1cb) | DCIF_CSC_COEF1_L0_B1(0x12a),
			DCIF_CSC_COEF2_L0_B2(0x7c9) | DCIF_CSC_COEF2_L0_B3(0x778),
			DCIF_CSC_COEF3_L0_C1(0x12a) | DCIF_CSC_COEF3_L0_C2(0x21d),
			DCIF_CSC_COEF4_L0_C3(0x000) | DCIF_CSC_COEF4_L0_D1(0x1f0),
			DCIF_CSC_COEF5_L0_D2(0x180) | DCIF_CSC_COEF5_L0_D3(0x180),
		},
		[DRM_COLOR_YCBCR_FULL_RANGE] = {
			/*
			 * Rec.709 full range:
			 *
			 * |R|   |1.0000  0.0000  1.5748|   |Y  - 0  |
			 * |G| = |1.0000 -0.1873 -0.4681| * |Cb - 128|
			 * |B|   |1.0000  1.8556  0.0000|   |Cr - 128|
			 */
			DCIF_CSC_COEF0_L0_A1(0x100) | DCIF_CSC_COEF0_L0_A2(0x000),
			DCIF_CSC_COEF1_L0_A3(0x193) | DCIF_CSC_COEF1_L0_B1(0x100),
			DCIF_CSC_COEF2_L0_B2(0x7d0) | DCIF_CSC_COEF2_L0_B3(0x788),
			DCIF_CSC_COEF3_L0_C1(0x100) | DCIF_CSC_COEF3_L0_C2(0x1db),
			DCIF_CSC_COEF4_L0_C3(0x000) | DCIF_CSC_COEF4_L0_D1(0x000),
			DCIF_CSC_COEF5_L0_D2(0x180) | DCIF_CSC_COEF5_L0_D3(0x180),
		},
	},
	[DRM_COLOR_YCBCR_BT2020] = {
		[DRM_COLOR_YCBCR_LIMITED_RANGE] = {
			/*
			 * BT.2020 limited range:
			 *
			 * |R|   |1.1644  0.0000  1.6787|   |Y  - 16 |
			 * |G| = |1.1644 -0.1874 -0.6505| * |Cb - 128|
			 * |B|   |1.1644  2.1418  0.0000|   |Cr - 128|
			 */
			DCIF_CSC_COEF0_L0_A1(0x12a) | DCIF_CSC_COEF0_L0_A2(0x000),
			DCIF_CSC_COEF1_L0_A3(0x1ae) | DCIF_CSC_COEF1_L0_B1(0x12a),
			DCIF_CSC_COEF2_L0_B2(0x7d0) | DCIF_CSC_COEF2_L0_B3(0x759),
			DCIF_CSC_COEF3_L0_C1(0x12a) | DCIF_CSC_COEF3_L0_C2(0x224),
			DCIF_CSC_COEF4_L0_C3(0x000) | DCIF_CSC_COEF4_L0_D1(0x1f0),
			DCIF_CSC_COEF5_L0_D2(0x180) | DCIF_CSC_COEF5_L0_D3(0x180),
		},
		[DRM_COLOR_YCBCR_FULL_RANGE] = {
			/*
			 * BT.2020 full range:
			 *
			 * |R|   |1.0000  0.0000  1.4746|   |Y  - 0  |
			 * |G| = |1.0000 -0.1646 -0.5714| * |Cb - 128|
			 * |B|   |1.0000  1.8814  0.0000|   |Cr - 128|
			 */
			DCIF_CSC_COEF0_L0_A1(0x100) | DCIF_CSC_COEF0_L0_A2(0x000),
			DCIF_CSC_COEF1_L0_A3(0x179) | DCIF_CSC_COEF1_L0_B1(0x100),
			DCIF_CSC_COEF2_L0_B2(0x7d6) | DCIF_CSC_COEF2_L0_B3(0x76e),
			DCIF_CSC_COEF3_L0_C1(0x100) | DCIF_CSC_COEF3_L0_C2(0x1e2),
			DCIF_CSC_COEF4_L0_C3(0x000) | DCIF_CSC_COEF4_L0_D1(0x000),
			DCIF_CSC_COEF5_L0_D2(0x180) | DCIF_CSC_COEF5_L0_D3(0x180),
		},
	},
};

static enum drm_mode_status
dcif_crtc_mode_valid(struct drm_crtc *crtc,
		     const struct drm_display_mode *mode)
{
	if (mode->crtc_clock > DCIF_MAX_PIXEL_CLOCK)
		return MODE_CLOCK_HIGH;

	return MODE_OK;
}

static void dcif_set_formats(struct dcif_dev *dcif,
			     struct drm_plane_state *plane_state,
			     const u32 bus_format)
{
	struct drm_device *drm = &dcif->drm;
	const u32 format = plane_state->fb->format->format;
	bool in_yuv = false;
	u32 reg = 0;

	switch (bus_format) {
	case MEDIA_BUS_FMT_RGB565_1X16:
		reg |= DCIF_DPI_CTRL_DATA_PATTERN(PATTERN_RGB565);
		break;
	case MEDIA_BUS_FMT_RGB888_1X24:
		reg |= DCIF_DPI_CTRL_DATA_PATTERN(PATTERN_RGB888);
		break;
	default:
		dev_err(drm->dev, "Unknown media bus format 0x%x\n", bus_format);
		break;
	}

	regmap_update_bits(dcif->regmap, DCIF_DPI_CTRL, DCIF_DPI_CTRL_DATA_PATTERN_MASK, reg);

	reg = 0;
	switch (format) {
	/* RGB Formats */
	case DRM_FORMAT_RGB565:
		reg |= DCIF_CTRLDESC0_FORMAT(CTRLDESCL0_FORMAT_RGB565);
		break;
	case DRM_FORMAT_RGB888:
		reg |= DCIF_CTRLDESC0_FORMAT(CTRLDESCL0_FORMAT_RGB888);
		break;
	case DRM_FORMAT_XRGB1555:
		reg |= DCIF_CTRLDESC0_FORMAT(CTRLDESCL0_FORMAT_ARGB1555);
		break;
	case DRM_FORMAT_XRGB4444:
		reg |= DCIF_CTRLDESC0_FORMAT(CTRLDESCL0_FORMAT_ARGB4444);
		break;
	case DRM_FORMAT_XBGR8888:
		reg |= DCIF_CTRLDESC0_FORMAT(CTRLDESCL0_FORMAT_ABGR8888);
		break;
	case DRM_FORMAT_XRGB8888:
		reg |= DCIF_CTRLDESC0_FORMAT(CTRLDESCL0_FORMAT_ARGB8888);
		break;

	/* YUV Formats */
	case DRM_FORMAT_YUYV:
		reg |= DCIF_CTRLDESC0_FORMAT(CTRLDESCL0_FORMAT_YCBCR422) |
		       DCIF_CTRLDESC0_YUV_FORMAT(CTRLDESCL0_YUV_FORMAT_VY2UY1);
		in_yuv = true;
		break;
	case DRM_FORMAT_YVYU:
		reg |= DCIF_CTRLDESC0_FORMAT(CTRLDESCL0_FORMAT_YCBCR422) |
		       DCIF_CTRLDESC0_YUV_FORMAT(CTRLDESCL0_YUV_FORMAT_UY2VY1);
		in_yuv = true;
		break;
	case DRM_FORMAT_UYVY:
		reg |= DCIF_CTRLDESC0_FORMAT(CTRLDESCL0_FORMAT_YCBCR422) |
		       DCIF_CTRLDESC0_YUV_FORMAT(CTRLDESCL0_YUV_FORMAT_Y2VY1U);
		in_yuv = true;
		break;
	case DRM_FORMAT_VYUY:
		reg |= DCIF_CTRLDESC0_FORMAT(CTRLDESCL0_FORMAT_YCBCR422) |
		       DCIF_CTRLDESC0_YUV_FORMAT(CTRLDESCL0_YUV_FORMAT_Y2UY1V);
		in_yuv = true;
		break;

	default:
		dev_err(drm->dev, "Unknown pixel format 0x%x\n", format);
		break;
	}

	regmap_update_bits(dcif->regmap, DCIF_CTRLDESC0(0),
			   DCIF_CTRLDESC0_FORMAT_MASK | DCIF_CTRLDESC0_YUV_FORMAT_MASK,
			   reg);

	if (in_yuv) {
		/* Enable CSC YCbCr -> RGB */
		const u32 *coeffs =
			dcif_yuv2rgb_coeffs[plane_state->color_encoding]
					    [plane_state->color_range];

		regmap_write(dcif->regmap, DCIF_CSC_COEF0_L0, coeffs[0]);
		regmap_write(dcif->regmap, DCIF_CSC_COEF1_L0, coeffs[1]);
		regmap_write(dcif->regmap, DCIF_CSC_COEF2_L0, coeffs[2]);
		regmap_write(dcif->regmap, DCIF_CSC_COEF3_L0, coeffs[3]);
		regmap_write(dcif->regmap, DCIF_CSC_COEF4_L0, coeffs[4]);
		regmap_write(dcif->regmap, DCIF_CSC_COEF5_L0, coeffs[5]);

		regmap_write(dcif->regmap, DCIF_CSC_CTRL_L0,
			     DCIF_CSC_CTRL_L0_CSC_EN |
			     DCIF_CSC_CTRL_L0_CSC_MODE_YCBCR2RGB);
	} else {
		regmap_write(dcif->regmap, DCIF_CSC_CTRL_L0, 0);
	}
}

static void dcif_set_mode(struct dcif_dev *dcif, u32 bus_flags)
{
	struct drm_display_mode *m = &dcif->crtc.state->adjusted_mode;
	u32 reg = 0;

	if (m->flags & DRM_MODE_FLAG_NHSYNC)
		reg |= DCIF_DPI_CTRL_HSYNC_POL_LOW;
	if (m->flags & DRM_MODE_FLAG_NVSYNC)
		reg |= DCIF_DPI_CTRL_VSYNC_POL_LOW;
	if (bus_flags & DRM_BUS_FLAG_DE_LOW)
		reg |= DCIF_DPI_CTRL_DE_POL_LOW;
	if (bus_flags & DRM_BUS_FLAG_PIXDATA_DRIVE_NEGEDGE)
		reg |= DCIF_DPI_CTRL_PCLK_EDGE_FALLING;

	regmap_update_bits(dcif->regmap, DCIF_DPI_CTRL, DCIF_DPI_CTRL_POL_MASK, reg);

	/* config display timings */
	reg = DCIF_DISP_SIZE_DISP_WIDTH(m->hdisplay) |
	      DCIF_DISP_SIZE_DISP_HEIGHT(m->vdisplay);
	regmap_write(dcif->regmap, DCIF_DISP_SIZE, reg);

	reg = DCIF_DPI_HSYN_PAR_BP_H(m->htotal - m->hsync_end) |
	      DCIF_DPI_HSYN_PAR_FP_H(m->hsync_start - m->hdisplay);
	regmap_write(dcif->regmap, DCIF_DPI_HSYN_PAR, reg);

	reg = DCIF_DPI_VSYN_PAR_BP_V(m->vtotal - m->vsync_end) |
	      DCIF_DPI_VSYN_PAR_FP_V(m->vsync_start - m->vdisplay);
	regmap_write(dcif->regmap, DCIF_DPI_VSYN_PAR, reg);

	reg = DCIF_DPI_VSYN_HSYN_WIDTH_PW_V(m->vsync_end - m->vsync_start) |
	      DCIF_DPI_VSYN_HSYN_WIDTH_PW_H(m->hsync_end - m->hsync_start);
	regmap_write(dcif->regmap, DCIF_DPI_VSYN_HSYN_WIDTH, reg);

	/* Layer 0 frame size */
	reg = DCIF_CTRLDESC2_HEIGHT(m->vdisplay) |
	      DCIF_CTRLDESC2_WIDTH(m->hdisplay);
	regmap_write(dcif->regmap, DCIF_CTRLDESC2(0), reg);

	/*
	 * Configure P_SIZE, T_SIZE and pitch
	 * 1. P_SIZE and T_SIZE should never be less than AXI bus width.
	 * 2. P_SIZE should never be less than T_SIZE.
	 */
	reg = DCIF_CTRLDESC3_P_SIZE(2) | DCIF_CTRLDESC3_T_SIZE(2) |
	      DCIF_CTRLDESC3_PITCH(dcif->crtc.primary->state->fb->pitches[0]);
	regmap_write(dcif->regmap, DCIF_CTRLDESC3(0), reg);
}

static void dcif_enable_plane_panic(struct dcif_dev *dcif)
{
	u32 reg;

	/* Set FIFO Panic watermarks, low 1/3, high 2/3. */
	reg = DCIF_PANIC_THRES_LOW(1 * PANIC0_THRES_MAX / 3) |
	      DCIF_PANIC_THRES_HIGH(2 * PANIC0_THRES_MAX / 3) |
	      DCIF_PANIC_THRES_REQ_EN;
	regmap_write(dcif->regmap, DCIF_PANIC_THRES(0), reg);

	/*
	 * Enable FIFO Panic, this does not generate interrupt, but
	 * boosts NoC priority based on FIFO Panic watermarks.
	 */
	regmap_read(dcif->regmap, DCIF_IE1(0), &reg);
	reg |= DCIF_INT1_FIFO_PANIC0;
	regmap_write(dcif->regmap, DCIF_IE1(0), reg);
	regmap_write(dcif->regmap, DCIF_IE1(1), reg);
	regmap_write(dcif->regmap, DCIF_IE1(2), reg);
}

static void dcif_disable_plane_panic(struct dcif_dev *dcif)
{
	u32 reg;

	/* Disable FIFO Panic NoC priority booster. */
	regmap_read(dcif->regmap, DCIF_IE1(0), &reg);
	reg &= ~DCIF_INT1_FIFO_PANIC0;
	regmap_write(dcif->regmap, DCIF_IE1(0), reg);
	regmap_write(dcif->regmap, DCIF_IE1(1), reg);
	regmap_write(dcif->regmap, DCIF_IE1(2), reg);

	regmap_clear_bits(dcif->regmap, DCIF_PANIC_THRES(0), DCIF_PANIC_THRES_REQ_EN);
}

static void dcif_enable_controller(struct dcif_dev *dcif)
{
	/* Enable Display */
	regmap_set_bits(dcif->regmap, DCIF_DISP_CTRL, DCIF_DISP_CTRL_DISP_ON);

	/* Enable layer 0 */
	regmap_set_bits(dcif->regmap, DCIF_CTRLDESC0(0), DCIF_CTRLDESC0_EN);
}

static void dcif_disable_controller(struct dcif_dev *dcif)
{
	u32 reg;
	int ret;

	/* Disable layer 0 */
	regmap_clear_bits(dcif->regmap, DCIF_CTRLDESC0(0), DCIF_CTRLDESC0_EN);

	ret = regmap_read_poll_timeout(dcif->regmap, DCIF_CTRLDESC0(0),
				       reg, !(reg & DCIF_CTRLDESC0_EN),
				       0, 36000);	/* Wait ~2 frame times max */
	if (ret)
		drm_err(&dcif->drm, "Failed to disable controller!\n");

	/* Disable Display */
	regmap_clear_bits(dcif->regmap, DCIF_DISP_CTRL, DCIF_DISP_CTRL_DISP_ON);
}

static void dcif_shadow_load_enable(struct dcif_dev *dcif)
{
	regmap_write_bits(dcif->regmap, DCIF_CTRLDESC0(0),
			  DCIF_CTRLDESC0_SHADOW_LOAD_EN, DCIF_CTRLDESC0_SHADOW_LOAD_EN);
}

static void dcif_reset_block(struct dcif_dev *dcif)
{
	regmap_set_bits(dcif->regmap, DCIF_DISP_CTRL, DCIF_DISP_CTRL_SW_RST);

	regmap_clear_bits(dcif->regmap, DCIF_DISP_CTRL, DCIF_DISP_CTRL_SW_RST);
}

static void dcif_crtc_atomic_destroy_state(struct drm_crtc *crtc,
					   struct drm_crtc_state *state)
{
	__drm_atomic_helper_crtc_destroy_state(state);
	kfree(to_dcif_crtc_state(state));
}

static void dcif_crtc_reset(struct drm_crtc *crtc)
{
	struct dcif_crtc_state *state;

	if (crtc->state)
		dcif_crtc_atomic_destroy_state(crtc, crtc->state);

	crtc->state = NULL;

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (state)
		__drm_atomic_helper_crtc_reset(crtc, &state->base);
}

static struct drm_crtc_state *
dcif_crtc_atomic_duplicate_state(struct drm_crtc *crtc)
{
	struct dcif_crtc_state *old = to_dcif_crtc_state(crtc->state);
	struct dcif_crtc_state *new;

	if (WARN_ON(!crtc->state))
		return NULL;

	new = kzalloc(sizeof(*new), GFP_KERNEL);
	if (!new)
		return NULL;

	__drm_atomic_helper_crtc_duplicate_state(crtc, &new->base);

	new->bus_format = old->bus_format;
	new->bus_flags = old->bus_flags;
	new->crc.source = old->crc.source;
	dcif_copy_roi(&old->crc.roi, &new->crc.roi);

	return &new->base;
}

static void dcif_crtc_mode_set_nofb(struct drm_crtc_state *crtc_state,
				    struct drm_plane_state *plane_state)
{
	struct dcif_crtc_state *dcif_crtc_state = to_dcif_crtc_state(crtc_state);
	struct drm_device *drm = crtc_state->crtc->dev;
	struct dcif_dev *dcif = crtc_to_dcif_dev(crtc_state->crtc);
	struct drm_display_mode *m = &crtc_state->adjusted_mode;

	dev_dbg(drm->dev, "Pixel clock: %dkHz\n", m->crtc_clock);
	dev_dbg(drm->dev, "Bridge bud_flags: 0x%08X\n", dcif_crtc_state->bus_flags);
	dev_dbg(drm->dev, "Mode flags: 0x%08X\n", m->flags);

	/* TODO: Mandatory DCIF reset as per the Reference Manual */
	dcif_reset_block(dcif);

	dcif_set_formats(dcif, plane_state, dcif_crtc_state->bus_format);

	dcif_set_mode(dcif, dcif_crtc_state->bus_flags);
}

static void dcif_crtc_queue_state_event(struct drm_crtc *crtc)
{
	struct dcif_dev *dcif = crtc_to_dcif_dev(crtc);

	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->event) {
		WARN_ON(drm_crtc_vblank_get(crtc));
		WARN_ON(dcif->event);
		dcif->event = crtc->state->event;
		crtc->state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);
}

static int dcif_crtc_atomic_check(struct drm_crtc *crtc,
				  struct drm_atomic_state *state)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state,
									  crtc);
	bool enable_primary = crtc_state->plane_mask & drm_plane_mask(crtc->primary);
	int ret;

	if (crtc_state->active && !enable_primary)
		return -EINVAL;

	if (crtc_state->active_changed && crtc_state->active) {
		/*
		 * If mode_changed is set by us, call
		 * drm_atomic_helper_check_modeset() as it's Kerneldoc requires.
		 */
		if (!crtc_state->mode_changed) {
			crtc_state->mode_changed = true;
			ret = drm_atomic_helper_check_modeset(crtc->dev, state);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static void dcif_crtc_atomic_flush(struct drm_crtc *crtc,
				   struct drm_atomic_state *state)
{
	struct drm_crtc_state *old_crtc_state = drm_atomic_get_old_crtc_state(state, crtc);
	struct dcif_crtc_state *old_dcif_crtc_state = to_dcif_crtc_state(old_crtc_state);
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state, crtc);
	struct dcif_crtc_state *dcif_crtc_state = to_dcif_crtc_state(crtc_state);
	bool need_modeset = drm_atomic_crtc_needs_modeset(crtc->state);
	struct dcif_dev *dcif = crtc_to_dcif_dev(crtc);

	dcif_shadow_load_enable(dcif);

	if (!crtc->state->active && !old_crtc_state->active)
		return;

	if (!need_modeset && to_disable_dcif_crc(dcif_crtc_state, old_dcif_crtc_state))
		dcif_crtc_disable_crc_source(dcif, 0);

	if (!need_modeset)
		dcif_crtc_queue_state_event(crtc);

	if (!need_modeset && to_enable_dcif_crc(dcif_crtc_state, old_dcif_crtc_state))
		dcif_crtc_enable_crc_source(dcif, dcif_crtc_state->crc.source,
					    &dcif_crtc_state->crc.roi, 0);
}

static void dcif_crtc_atomic_enable(struct drm_crtc *crtc,
				    struct drm_atomic_state *state)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state, crtc);
	struct drm_plane_state *plane_state = drm_atomic_get_new_plane_state(state, crtc->primary);
	struct dcif_crtc_state *dcif_crtc_state = to_dcif_crtc_state(crtc_state);
	struct drm_display_mode *adj = &crtc_state->adjusted_mode;
	struct dcif_dev *dcif = crtc_to_dcif_dev(crtc);
	struct drm_device *drm = crtc->dev;
	dma_addr_t baseaddr;

	dev_dbg(drm->dev, "mode " DRM_MODE_FMT "\n", DRM_MODE_ARG(adj));

	/* enable power when we start to set mode for CRTC */
	pm_runtime_get_sync(drm->dev);

	drm_crtc_vblank_on(crtc);

	dcif_crtc_mode_set_nofb(crtc_state, plane_state);

	baseaddr = drm_fb_dma_get_gem_addr(plane_state->fb, plane_state, 0);
	if (baseaddr)
		regmap_write(dcif->regmap, DCIF_CTRLDESC4(0), baseaddr);

	dcif_enable_plane_panic(dcif);
	dcif_enable_controller(dcif);

	dcif_crtc_queue_state_event(crtc);

	if (dcif->has_crc && dcif_crtc_state->crc.source != DCIF_CRC_SRC_NONE)
		dcif_crtc_enable_crc_source(dcif,
					    dcif_crtc_state->crc.source,
					    &dcif_crtc_state->crc.roi,
					    0);
}

static void dcif_crtc_atomic_disable(struct drm_crtc *crtc,
				     struct drm_atomic_state *state)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state,
									  crtc);
	struct dcif_crtc_state *dcif_crtc_state = to_dcif_crtc_state(crtc_state);
	struct drm_device *drm = crtc->dev;
	struct dcif_dev *dcif = crtc_to_dcif_dev(crtc);

	if (dcif->has_crc && dcif_crtc_state->crc.source != DCIF_CRC_SRC_NONE)
		dcif_crtc_disable_crc_source(dcif, 0);

	dcif_disable_controller(dcif);
	dcif_disable_plane_panic(dcif);

	drm_crtc_vblank_off(crtc);

	/* disable power when CRTC is disabled */
	pm_runtime_put_sync(drm->dev);

	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->event && !crtc->state->active) {
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);
}

static const struct drm_crtc_helper_funcs dcif_crtc_helper_funcs = {
	.mode_valid	= dcif_crtc_mode_valid,
	.atomic_check	= dcif_crtc_atomic_check,
	.atomic_flush	= dcif_crtc_atomic_flush,
	.atomic_enable	= dcif_crtc_atomic_enable,
	.atomic_disable	= dcif_crtc_atomic_disable,
};

static int dcif_crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct dcif_dev *dcif = crtc_to_dcif_dev(crtc);
	int domain = dcif->cpu_domain;

	/* Clear and enable VS_BLANK IRQ */
	regmap_set_bits(dcif->regmap, DCIF_IS0(domain), DCIF_INT0_VS_BLANK);
	regmap_set_bits(dcif->regmap, DCIF_IE0(domain), DCIF_INT0_VS_BLANK);

	return 0;
}

static void dcif_crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct dcif_dev *dcif = crtc_to_dcif_dev(crtc);
	int domain = dcif->cpu_domain;

	/* Disable and clear VS_BLANK IRQ */
	regmap_clear_bits(dcif->regmap, DCIF_IE0(domain), DCIF_INT0_VS_BLANK);
	regmap_clear_bits(dcif->regmap, DCIF_IS0(domain), DCIF_INT0_VS_BLANK);
}

static const struct drm_crtc_funcs dcif_crtc_funcs = {
	.reset			= dcif_crtc_reset,
	.destroy		= drm_crtc_cleanup,
	.set_config		= drm_atomic_helper_set_config,
	.page_flip		= drm_atomic_helper_page_flip,
	.atomic_duplicate_state	= dcif_crtc_atomic_duplicate_state,
	.atomic_destroy_state	= dcif_crtc_atomic_destroy_state,
	.enable_vblank		= dcif_crtc_enable_vblank,
	.disable_vblank		= dcif_crtc_disable_vblank,
	.set_crc_source         = dcif_crtc_set_crc_source,
	.verify_crc_source      = dcif_crtc_verify_crc_source,
};

irqreturn_t dcif_irq_handler(int irq, void *data)
{
	struct drm_device *drm = data;
	struct dcif_dev *dcif = to_dcif_dev(drm);
	int domain = dcif->cpu_domain;
	unsigned long flags;
	u32 stat, crc;

	regmap_read(dcif->regmap, DCIF_IS0(domain), &stat);
	if (stat & DCIF_INT0_VS_BLANK) {
		drm_crtc_handle_vblank(&dcif->crtc);

		spin_lock_irqsave(&drm->event_lock, flags);
		if (dcif->event) {
			drm_crtc_send_vblank_event(&dcif->crtc, dcif->event);
			dcif->event = NULL;
			drm_crtc_vblank_put(&dcif->crtc);
		}
		if (dcif->crc_is_enabled) {
			regmap_read(dcif->regmap, DCIF_CRC_VAL_R(0), &crc);
			drm_crtc_add_crc_entry(&dcif->crtc, false, 0, &crc);
			dev_dbg(drm->dev, "crc=0x%x\n",  crc);
		}
		spin_unlock_irqrestore(&drm->event_lock, flags);
	} else {
		dev_info(drm->dev, "irq stat=0x%x\n", stat);
	}

	/* W1C */
	regmap_write(dcif->regmap, DCIF_IS0(domain), stat);

	return IRQ_HANDLED;
}

int dcif_crtc_init(struct dcif_dev *dcif)
{
	int ret;

	ret = dcif_plane_init(dcif);
	if (ret)
		return ret;

	drm_crtc_helper_add(&dcif->crtc, &dcif_crtc_helper_funcs);
	ret = drm_crtc_init_with_planes(&dcif->drm, &dcif->crtc,
					&dcif->planes.primary, NULL,
					&dcif_crtc_funcs, NULL);
	if (ret) {
		drm_err(&dcif->drm, "failed to initialize CRTC: %d\n", ret);
		return ret;
	}

	return 0;
}
