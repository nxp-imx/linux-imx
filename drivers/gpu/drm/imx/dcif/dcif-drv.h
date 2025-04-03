/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright 2025 NXP
 */

#ifndef __DCIF_DRV_H__
#define __DCIF_DRV_H__

#include <linux/clk.h>
#include <linux/irqreturn.h>
#include <linux/kernel.h>

#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_encoder.h>
#include <drm/drm_plane.h>
#include <drm/drm_vblank.h>

struct dcif_crc;

struct dcif_dev {
	struct drm_device	drm;
	void __iomem		*reg_base;

	struct regmap		*regmap;
	struct regmap		*blkctrl_regmap;
	int			irq[3];

	unsigned int num_clks;
	struct clk_bulk_data *clks;

	struct drm_crtc crtc;
	struct {
		struct drm_plane primary;
		struct drm_plane overlay;
	} planes;
	struct drm_encoder encoder;

	struct drm_pending_vblank_event *event;
	/* Implement crc */
	bool			has_crc;
	bool			crc_is_enabled;

	/* CPU domain for interrupt control */
	int			cpu_domain;
};

enum dcif_crc_source {
	DCIF_CRC_SRC_NONE,
	DCIF_CRC_SRC_FRAME,
	DCIF_CRC_SRC_FRAME_ROI,
};

struct dcif_crc {
	enum dcif_crc_source	source;
	struct drm_rect		roi;
};

struct dcif_crtc_state {
	struct drm_crtc_state	base;	/* always be the first member */
	struct dcif_crc		crc;
	u32			bus_format;
	u32			bus_flags;
};

static inline struct dcif_dev *to_dcif_dev(struct drm_device *drm_dev)
{
	return container_of(drm_dev, struct dcif_dev, drm);
}

static inline struct dcif_dev *crtc_to_dcif_dev(struct drm_crtc *crtc)
{
	return to_dcif_dev(crtc->dev);
}

static inline struct dcif_crtc_state *
to_dcif_crtc_state(struct drm_crtc_state *s)
{
	return container_of(s, struct dcif_crtc_state, base);
}

irqreturn_t dcif_irq_handler(int irq, void *data);

int dcif_crtc_init(struct dcif_dev *dcif);

int dcif_plane_init(struct dcif_dev *dcif);

int dcif_kms_prepare(struct dcif_dev *dcif);

#endif
