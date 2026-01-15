/* SPDX-License-Identifier: GPL-2.0+ */

/*
 * Copyright 2026 NXP
 */

#ifndef __DPU95_LD_H__
#define __DPU95_LD_H__

#include <drm/drm_modes.h>
#include <linux/firmware/imx/disp-mu.h>
#include <linux/remoteproc.h>
#include <linux/workqueue.h>

struct dpu95_localdimming;

struct dpu95_ld_cfg {
	int led_size;
	int h_led;
	int v_led;
};

struct dpu95_ld_reg_def {
	u32 offset;
	u32 value;
};

/* Structure for holding information about the LD-FW core (usually the CM0+) */
struct dpu95_ld_fw {
	struct rproc *rproc;
	struct disp_mu_client mu;
	struct workqueue_struct *workq;
	struct work_struct start_work;
	bool ready;
	bool enabled;
};

struct dpu95_localdimming {
	void __iomem *base;
	int id;
	unsigned int index;
	struct dpu95_soc *dpu;
	bool enabled;
	struct dpu95_ld_fw fw;
	struct dpu95_ld_cfg config;
	struct drm_display_mode mode;
};

struct dpu95_ld_intr {
	int ldvintdly;
	int smslvon;
	int smvlvon;
	int smblvon;
	int ledinton;
	int blinton;
	int vinton;
};

struct dpu95_drm_device;

int dpu95_ld_load(struct dpu95_drm_device *dpu_drm);
void dpu95_ld_unload(struct dpu95_drm_device *dpu_drm);

#endif
