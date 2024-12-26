// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright 2023,2026 NXP
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/sizes.h>

#include "dpu95.h"

/* register POLARITYCTRL */
#define POLEN	BIT(2)
#define POLVS	BIT(1)
#define POLHS	BIT(0)

struct dpu95_dither {
	void __iomem *aux_base;
	void __iomem *base;
	int id;
	unsigned int index;
	struct dpu95_soc *dpu;
	u8 reg_polarityctrl;
};

static inline u32 dpu95_aux_dt_read(struct dpu95_dither *dt,
				    unsigned int offset)
{
	return readl(dt->aux_base + offset);
}

static inline void dpu95_aux_dt_write(struct dpu95_dither *dt,
				      unsigned int offset, u32 value)
{
	writel(value, dt->aux_base + offset);
}

static inline void dpu95_aux_dt_write_mask(struct dpu95_dither *dt,
					   unsigned int offset,
					   u32 mask, u32 value)
{
	u32 tmp;

	tmp = dpu95_aux_dt_read(dt, offset);
	tmp &= ~mask;
	dpu95_aux_dt_write(dt, offset, tmp | value);
}

void dpu95_dt_polhs_active_high(struct dpu95_dither *dt)
{
	dpu95_aux_dt_write_mask(dt, dt->reg_polarityctrl, POLHS, POLHS);
}

void dpu95_dt_polhs_active_low(struct dpu95_dither *dt)
{
	dpu95_aux_dt_write_mask(dt, dt->reg_polarityctrl, POLHS, 0);
}

void dpu95_dt_polvs_active_high(struct dpu95_dither *dt)
{
	dpu95_aux_dt_write_mask(dt, dt->reg_polarityctrl, POLVS, POLVS);
}

void dpu95_dt_polvs_active_low(struct dpu95_dither *dt)
{
	dpu95_aux_dt_write_mask(dt, dt->reg_polarityctrl, POLVS, 0);
}

void dpu95_dt_polen_active_high(struct dpu95_dither *dt)
{
	dpu95_aux_dt_write_mask(dt, dt->reg_polarityctrl, POLEN, POLEN);
}

void dpu95_dt_polen_active_low(struct dpu95_dither *dt)
{
	dpu95_aux_dt_write_mask(dt, dt->reg_polarityctrl, POLEN, 0);
}

struct dpu95_dither *dpu95_dt_get(struct dpu95_soc *dpu, int id)
{
	struct dpu95_dither *dt;
	int i;

	for (i = 0; i < ARRAY_SIZE(dpu->dt); i++) {
		dt = dpu->dt[i];
		if (dt->id == id)
			break;
	}

	if (i == ARRAY_SIZE(dpu->dt))
		return ERR_PTR(-EINVAL);

	return dt;
}

void dpu95_dt_hw_init(struct dpu95_soc *dpu, unsigned int index)
{
}

int dpu95_dt_init(struct dpu95_soc *dpu, unsigned int index,
		  unsigned int id, enum dpu95_unit_type type,
		  unsigned long aux_base, unsigned long base)
{
	const struct dpu95_data *data = dpu->data;
	struct dpu95_dither *dt;

	dt = devm_kzalloc(dpu->dev, sizeof(*dt), GFP_KERNEL);
	if (!dt)
		return -ENOMEM;

	dpu->dt[index] = dt;

	dt->aux_base = devm_ioremap(dpu->dev, aux_base, SZ_16);
	if (!dt->aux_base)
		return -ENOMEM;

	dt->base = devm_ioremap(dpu->dev, base, SZ_32);
	if (!dt->base)
		return -ENOMEM;

	dt->dpu = dpu;
	dt->id = id;
	dt->index = index;
	dt->reg_polarityctrl = data->reg_polarityctrl;

	return 0;
}
