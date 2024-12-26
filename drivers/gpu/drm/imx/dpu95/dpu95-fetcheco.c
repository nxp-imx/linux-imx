// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright 2017-2020,2022,2023,2025,2026 NXP
 */

#include <linux/kernel.h>
#include <linux/sizes.h>

#include "dpu95.h"
#include "dpu95-fetchunit.h"

static void
dpu95_fe_set_src_buf_dimensions(struct dpu95_fetchunit *fu,
				unsigned int w, unsigned int h,
				const struct drm_format_info *format)
{
	struct dpu95_soc *dpu = fu->dpu;
	unsigned int width, height;

	width = w / format->hsub;
	height = h / format->vsub;

	switch (format->format) {
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
	case DRM_FORMAT_NV16:
	case DRM_FORMAT_NV61:
	case DRM_FORMAT_NV24:
	case DRM_FORMAT_NV42:
		break;
	default:
		dev_warn(dpu->dev, "%s - unsupported pixel format 0x%08x\n",
			 fu->name, format->format);
		return;
	}

	dpu95_fu_write(fu, SOURCEBUFFERDIMENSION(fu),
		       LINEWIDTH(width) | LINECOUNT(height));
}

static void dpu95_fe_set_fmt(struct dpu95_fetchunit *fu,
			     const struct drm_format_info *format,
			     enum drm_color_encoding unused1,
			     enum drm_color_range unused2)
{
	struct dpu95_soc *dpu = fu->dpu;
	u32 bits = 0, shifts = 0;
	unsigned int x, y;

	switch (format->format) {
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
		break;
	default:
		dev_warn(dpu->dev, "%s - unsupported pixel format 0x%08x\n",
			 fu->name, format->format);
		return;
	}

	switch (format->hsub) {
	case 1:
		x = 0x4;
		break;
	case 2:
		x = 0x2;
		break;
	default:
		dev_warn(dpu->dev, "%s - unsupported horizontal subsampling %u\n",
			 fu->name, format->hsub);
		return;
	}

	switch (format->vsub) {
	case 1:
		y = 0x4;
		break;
	case 2:
		y = 0x2;
		break;
	default:
		dev_warn(dpu->dev, "%s - unsupported vertical subsampling %u\n",
			 fu->name, format->vsub);
		return;
	}

	dpu95_fu_set_src_bpp(fu, 16);

	dpu95_fu_write_mask(fu, FRAMERESAMPLING(fu), DELTAX_MASK | DELTAY_MASK,
			    DELTAX(x) | DELTAY(y));

	dpu95_fu_write_mask(fu, CONTROL(fu), RASTERMODE_MASK,
			    RASTERMODE(RASTERMODE_NORMAL));

	dpu95_fu_get_pixel_format_bits(fu, format->format, &bits);
	dpu95_fu_get_pixel_format_shifts(fu, format->format, &shifts);

	dpu95_fu_write(fu, COLORCOMPONENTBITS(fu), bits & ~Y_BITS_MASK);
	dpu95_fu_write(fu, COLORCOMPONENTSHIFT(fu), shifts & ~Y_SHIFT_MASK);
}

static void dpu95_fe_set_framedimensions(struct dpu95_fetchunit *fu,
					 unsigned int w, unsigned int h)
{
	dpu95_fu_write(fu, FRAMEDIMENSIONS(fu), FRAMEWIDTH(w) | FRAMEHEIGHT(h));
}

static void dpu95_fe_set_ops(struct dpu95_fetchunit *fu)
{
	memcpy(&fu->ops, &dpu95_fu_common_ops, sizeof(dpu95_fu_common_ops));
	fu->ops.set_src_buf_dimensions	= dpu95_fe_set_src_buf_dimensions;
	fu->ops.set_fmt			= dpu95_fe_set_fmt;
	fu->ops.set_framedimensions	= dpu95_fe_set_framedimensions;
}

struct dpu95_fetchunit *dpu95_fe_get(struct dpu95_soc *dpu, unsigned int id)
{
	struct dpu95_fetchunit *fu;
	int i;

	for (i = 0; i < dpu->fe_cnt; i++) {
		fu = dpu->fe[i];
		if (fu->id == id)
			break;
	}

	if (i == dpu->fe_cnt)
		return ERR_PTR(-EINVAL);

	return fu;
}

void dpu95_fe_hw_init(struct dpu95_soc *dpu, unsigned int index)
{
	dpu95_fu_common_hw_init(dpu->fe[index]);
}

int dpu95_fe_init(struct dpu95_soc *dpu, unsigned int index,
		  unsigned int id, enum dpu95_unit_type type,
		  unsigned long pec_base, unsigned long base)
{
	const struct dpu95_data *data = dpu->data;
	struct dpu95_fetchunit *fu;

	fu = devm_kzalloc(dpu->dev, sizeof(*fu), GFP_KERNEL);
	if (!fu)
		return -ENOMEM;

	dpu->fe[index] = fu;

	fu->pec_base = devm_ioremap(dpu->dev, pec_base, SZ_16);
	if (!fu->pec_base)
		return -ENOMEM;

	fu->base = devm_ioremap(dpu->dev, base, SZ_128);
	if (!fu->base)
		return -ENOMEM;

	fu->dpu = dpu;
	fu->id = id;
	fu->index = index;
	fu->type = type;
	fu->link_id = data->link_id_fe[index];
	fu->reg_offset1 = 0x10;
	fu->reg_offset2 = 0x48;
	fu->reg_burstbuffermanagement = 0x0c;
	fu->reg_burstbufferproperties = 0x60;
	snprintf(fu->name, sizeof(fu->name), "FetchECO%u", id);

	dpu95_fe_set_ops(fu);

	return 0;
}
