// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright 2026 NXP
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/sizes.h>
#include <linux/slab.h>

#include <drm/drm_color_mgmt.h>
#include <drm/drm_mode.h>
#include <drm/drm_property.h>

#include "dpu95.h"

#define STATICCONTROL			0x8
#define CONTROL				0xc
#define  MODE_MASK			0x3
#define  ALPHAMASK			BIT(4)
#define  ALPHAINVERT			BIT(5)
#define RED(n)				(0x10 + 0x4 * (n))
#define GREEN(n)			(0x18 + 0x4 * (n))
#define BLUE(n)				(0x20 + 0x4 * (n))
#define ALPHA(n)			(0x28 + 0x4 * (n))
#define OFFSETVECTOR(n)			(0x30 + 0x4 * (n))
#define  COEFF1(n)			((n) & 0x1fff)
#define  COEFF2(n)			(((n) & 0x1fff) << 16)
#define  COEFF3(n)			((n) & 0x1fff)

/*
 * There are two derivatives of the Matrix processing unit:
 * - Matrix: Full feature set.
 * - MatrixL: 3x3 matrix instead of 4x4 (alpha is bypassed and does not affect
 *            RGB components in MATRIX mode).  This unit can be used in the
 *            display engine, where no alpha is available.
 */
struct dpu95_matrix {
	void __iomem *pec_base;
	void __iomem *base;
	unsigned int id;
	unsigned int index;
	struct dpu95_soc *dpu;
};

static inline u32 dpu95_cm_read(struct dpu95_matrix *cm, unsigned int offset)
{
	return readl(cm->base + offset);
}

static inline void dpu95_cm_write(struct dpu95_matrix *cm,
				  unsigned int offset, u32 value)
{
	writel(value, cm->base + offset);
}

static inline void dpu95_cm_write_mask(struct dpu95_matrix *cm,
				       unsigned int offset, u32 mask, u32 value)
{
	u32 tmp;

	tmp = dpu95_cm_read(cm, offset);
	tmp &= ~mask;
	dpu95_cm_write(cm, offset, tmp | value);
}

static void dpu95_cm_enable_shden(struct dpu95_matrix *cm)
{
	dpu95_cm_write_mask(cm, STATICCONTROL, SHDEN, SHDEN);
}

static void dpu95_cm_control_init(struct dpu95_matrix *cm)
{
	dpu95_cm_write(cm, CONTROL, 0);
}

void dpu95_cm_mode(struct dpu95_matrix *cm, enum dpu95_cm_mode m)
{
	dpu95_cm_write_mask(cm, CONTROL, MODE_MASK, m);
}

void dpu95_cm_set_matrix(struct dpu95_matrix *cm,
			 struct drm_property_blob *ctm_blob)
{
	struct drm_color_ctm *ctm;
	u32 coeffs[9];
	int i;

	if (!ctm_blob)
		return;

	ctm = ctm_blob->data;

	for (i = 0; i < ARRAY_SIZE(coeffs); i++)
		coeffs[i] = drm_color_ctm_s31_32_to_qm_n(ctm->matrix[i], 3, 10);

	dpu95_cm_write(cm, RED(0),   COEFF1(coeffs[0]) | COEFF2(coeffs[1]));
	dpu95_cm_write(cm, RED(1),   COEFF3(coeffs[2]));
	dpu95_cm_write(cm, GREEN(0), COEFF1(coeffs[3]) | COEFF2(coeffs[4]));
	dpu95_cm_write(cm, GREEN(1), COEFF3(coeffs[5]));
	dpu95_cm_write(cm, BLUE(0),  COEFF1(coeffs[6]) | COEFF2(coeffs[7]));
	dpu95_cm_write(cm, BLUE(1),  COEFF3(coeffs[8]));
	dpu95_cm_write(cm, OFFSETVECTOR(0), 0);
	dpu95_cm_write(cm, OFFSETVECTOR(1), 0);
}

struct dpu95_matrix *dpu95_cm_get(struct dpu95_soc *dpu, unsigned int id)
{
	struct dpu95_matrix *cm;
	int i;

	for (i = 0; i < ARRAY_SIZE(dpu->cm); i++) {
		cm = dpu->cm[i];
		if (cm->id == id)
			break;
	}

	if (i == ARRAY_SIZE(dpu->cm))
		return ERR_PTR(-EINVAL);

	return cm;
}

void dpu95_cm_hw_init(struct dpu95_soc *dpu, unsigned int index)
{
	struct dpu95_matrix *cm = dpu->cm[index];

	dpu95_cm_enable_shden(cm);
	dpu95_cm_control_init(cm);
}

int dpu95_cm_init(struct dpu95_soc *dpu, unsigned int index,
		  unsigned int id, enum dpu95_unit_type type,
		  unsigned long pec_base, unsigned long base)
{
	struct dpu95_matrix *cm;

	cm = devm_kzalloc(dpu->dev, sizeof(*cm), GFP_KERNEL);
	if (!cm)
		return -ENOMEM;

	dpu->cm[index] = cm;

	if (pec_base) {
		cm->pec_base = devm_ioremap(dpu->dev, pec_base, SZ_16);
		if (!cm->pec_base)
			return -ENOMEM;
	}

	cm->base = devm_ioremap(dpu->dev, base, SZ_64);
	if (!cm->base)
		return -ENOMEM;

	cm->dpu = dpu;
	cm->id = id;
	cm->index = index;

	return 0;
}
