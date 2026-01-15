// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2020,2022,2023,2025,2026 NXP
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include "dpu95.h"
#include "dpu95-drv.h"

static const char * const dpu95_unit_names[] = {
	"ConstFrame",
	"DomainBlend",
	"Dither",
	"ExtDst",
	"FetchEco",
	"FrameGen",
	"FetchLayer",
	"FetchYUV",
	"HScaler",
	"LayerBlend",
	"LocalDimming",
	"VScaler",
};

static inline u32 dpu95_comctrl_irq_read(struct dpu95_soc *dpu, unsigned int offset)
{
	return readl(dpu->comctrl_irq_reg + offset);
}

static inline void dpu95_comctrl_irq_write(struct dpu95_soc *dpu,
					 unsigned int offset, u32 value)
{
	writel(value, dpu->comctrl_irq_reg + offset);
}

static inline u32 dpu95_disp_irq0_read(struct dpu95_soc *dpu, unsigned int offset)
{
	return readl(dpu->disp_irq0_reg + offset);
}

static inline void dpu95_disp_irq0_write(struct dpu95_soc *dpu,
					 unsigned int offset, u32 value)
{
	writel(value, dpu->disp_irq0_reg + offset);
}

static inline u32 dpu95_disp_irq2_read(struct dpu95_soc *dpu, unsigned int offset)
{
	return readl(dpu->disp_irq2_reg + offset);
}

static inline void dpu95_disp_irq2_write(struct dpu95_soc *dpu,
					 unsigned int offset, u32 value)
{
	writel(value, dpu->disp_irq2_reg + offset);
}

static inline void dpu95_dm_mask_write(struct dpu95_soc *dpu,
				       unsigned int offset, u32 value)
{
	writel(value, dpu->dm_mask_reg + offset);
}

static void dpu95_dm_extdst0_dm_allow_all(struct dpu95_soc *dpu)
{
	dpu95_dm_mask_write(dpu, EXTDST0_DM, ALLOW_ALL);
}

static void dpu95_dm_extdst1_dm_allow_all(struct dpu95_soc *dpu)
{
	dpu95_dm_mask_write(dpu, EXTDST1_DM, ALLOW_ALL);
}

static void dpu95_dm_extdst4_dm_allow_all(struct dpu95_soc *dpu)
{
	dpu95_dm_mask_write(dpu, EXTDST4_DM, ALLOW_ALL);
}

static void dpu95_dm_extdst5_dm_allow_all(struct dpu95_soc *dpu)
{
	dpu95_dm_mask_write(dpu, EXTDST5_DM, ALLOW_ALL);
}

void dpu95_enable_display_pipeline_sync(struct dpu95_soc *dpu)
{
	dpu95_dm_mask_write(dpu, DISPLAY_STATIC, PIPELINE_SYNC);
}

void dpu95_disable_display_pipeline_sync(struct dpu95_soc *dpu)
{
	dpu95_dm_mask_write(dpu, DISPLAY_STATIC, 0);
}

static void dpu95_dm_extdst0_master(struct dpu95_soc *dpu)
{
	dpu95_dm_mask_write(dpu, EXTDST0_STATIC, MASTER);
}

static void dpu95_dm_extdst1_master(struct dpu95_soc *dpu)
{
	dpu95_dm_mask_write(dpu, EXTDST1_STATIC, MASTER);
}

static void dpu95_dm_extdst4_master(struct dpu95_soc *dpu)
{
	dpu95_dm_mask_write(dpu, EXTDST4_STATIC, MASTER);
}

static void dpu95_dm_extdst5_master(struct dpu95_soc *dpu)
{
	dpu95_dm_mask_write(dpu, EXTDST5_STATIC, MASTER);
}

void dpu95_comctrl_irq_handle(struct irq_desc *desc, int irq)
{
	struct dpu95_soc *dpu = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int virq;
	u32 status;

	chained_irq_enter(chip, desc);

	status = dpu95_comctrl_irq_read(dpu, INTERRUPTSTATUS(irq / 32));
	status &= dpu95_comctrl_irq_read(dpu, INTERRUPTENABLE(irq / 32));

	if (status & BIT(irq % 32)) {
		virq = irq_find_mapping(dpu->comctrl_irq_domain, irq);
		if (virq)
			generic_handle_irq(virq);
	}

	chained_irq_exit(chip, desc);
}

void dpu95_disp_irq0_handle(struct irq_desc *desc, int irq)
{
	struct dpu95_soc *dpu = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int virq;
	u32 status;

	chained_irq_enter(chip, desc);

	status = dpu95_disp_irq0_read(dpu, INTERRUPTSTATUS(irq / 32));
	status &= dpu95_disp_irq0_read(dpu, INTERRUPTENABLE(irq / 32));

	if (status & BIT(irq % 32)) {
		virq = irq_find_mapping(dpu->disp_irq0_domain, irq);
		if (virq)
			generic_handle_irq(virq);
	}

	chained_irq_exit(chip, desc);
}

void dpu95_disp_irq2_handle(struct irq_desc *desc, int irq)
{
	struct dpu95_soc *dpu = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int virq;
	u32 status;

	chained_irq_enter(chip, desc);

	status = dpu95_disp_irq2_read(dpu, INTERRUPTSTATUS(irq / 32));
	status &= dpu95_disp_irq2_read(dpu, INTERRUPTENABLE(irq / 32));

	if (status & BIT(irq % 32)) {
		virq = irq_find_mapping(dpu->disp_irq2_domain, irq);
		if (virq)
			generic_handle_irq(virq);
	}

	chained_irq_exit(chip, desc);
}

int dpu95_map_comctrl_irq(struct dpu95_soc *dpu, int irq)
{
	int virq = irq_find_mapping(dpu->comctrl_irq_domain, irq);

	if (!virq)
		virq = irq_create_mapping(dpu->comctrl_irq_domain, irq);

	return virq;
}

int dpu95_map_disp_irq0(struct dpu95_soc *dpu, int irq)
{
	int virq = irq_find_mapping(dpu->disp_irq0_domain, irq);

	if (!virq)
		virq = irq_create_mapping(dpu->disp_irq0_domain, irq);

	return virq;
}

int dpu95_map_disp_irq2(struct dpu95_soc *dpu, int irq)
{
	int virq = irq_find_mapping(dpu->disp_irq2_domain, irq);

	if (!virq)
		virq = irq_create_mapping(dpu->disp_irq2_domain, irq);

	return virq;
}

void dpu95_irq_hw_init(struct dpu95_soc *dpu)
{
	const struct dpu95_data *data = dpu->data;
	int i;

	for (i = 0; i < data->irq_cnt; i += 32) {
		/* mask and clear all interrupts */
		dpu95_comctrl_irq_write(dpu, INTERRUPTENABLE(i / 32), 0);
		dpu95_comctrl_irq_write(dpu, INTERRUPTCLEAR(i / 32),
					~data->unused_irq[i / 32]);
		dpu95_disp_irq0_write(dpu, INTERRUPTENABLE(i / 32), 0);
		dpu95_disp_irq0_write(dpu, INTERRUPTCLEAR(i / 32),
				      ~data->unused_irq[i / 32]);
		dpu95_disp_irq2_write(dpu, INTERRUPTENABLE(i / 32), 0);
		dpu95_disp_irq2_write(dpu, INTERRUPTCLEAR(i / 32),
				      ~data->unused_irq[i / 32]);
	}
}

static struct irq_domain *dpu95_find_parent_irq_domain(struct device *dev)
{
	struct device_node *parent;
	struct irq_domain *domain;

	parent = of_irq_find_parent(dev->of_node);
	if (!parent) {
		dev_err(dev, "failed to find parent irq node\n");
		return NULL;
	}

	domain = irq_find_host(parent);
	of_node_put(parent);
	if (!domain) {
		dev_err(dev, "failed to find parent irq domain\n");
		return NULL;
	}

	return domain;
}

static int dpu95_irq_init(struct platform_device *pdev, struct dpu95_soc *dpu)
{
	const struct dpu95_data *data = dpu->data;
	struct irq_domain *parent_domain;
	struct device *dev = &pdev->dev;
	struct irq_chip_generic *gc;
	struct irq_chip_type *ct;
	int ret, i, j;

	parent_domain = dpu95_find_parent_irq_domain(dev);
	if (!parent_domain)
		return -ENODEV;

	dpu->comctrl_irq = devm_kcalloc(dev, data->comctrl_irq_cnt, sizeof(int),
					GFP_KERNEL);
	dpu->disp_irq0 = devm_kcalloc(dev, data->disp_irq0_cnt, sizeof(int),
				      GFP_KERNEL);
	dpu->disp_irq2 = devm_kcalloc(dev, data->disp_irq2_cnt, sizeof(int),
				      GFP_KERNEL);
	if (!dpu->comctrl_irq || !dpu->disp_irq0 || !dpu->disp_irq2)
		return -ENOMEM;

	for (i = 0; i < data->comctrl_irq_cnt; i++) {
		dpu->comctrl_irq[i] = platform_get_irq(pdev,
						       data->comctrl_irq[i]);
		if (dpu->comctrl_irq[i] < 0)
			return dev_err_probe(dev, dpu->comctrl_irq[i],
					     "failed to get comctrl irq[%d]\n",
					     data->comctrl_irq[i]);
	}

	for (i = 0; i < data->disp_irq0_cnt; i++) {
		dpu->disp_irq0[i] = platform_get_irq(pdev,
						     data->disp_irq0[i]);
		if (dpu->disp_irq0[i] < 0)
			return dev_err_probe(dev, dpu->disp_irq0[i],
					     "failed to get display irq0[%d]\n",
					     data->disp_irq0[i]);
	}

	for (i = 0; i < data->disp_irq2_cnt; i++) {
		dpu->disp_irq2[i] = platform_get_irq(pdev,
						     data->disp_irq2[i]);
		if (dpu->disp_irq2[i] < 0)
			return dev_err_probe(dev, dpu->disp_irq2[i],
					     "failed to get display irq2[%d]\n",
					     data->disp_irq2[i]);
	}

	dpu->comctrl_irq_domain = irq_domain_add_linear(dev->of_node,
						      data->irq_cnt,
						      &irq_generic_chip_ops,
						      dpu);
	if (!dpu->comctrl_irq_domain) {
		dev_err(dev, "failed to add comctrl irq domain\n");
		return -ENODEV;
	}

	dpu->disp_irq0_domain = irq_domain_add_linear(dev->of_node,
						      data->irq_cnt,
						      &irq_generic_chip_ops,
						      dpu);
	if (!dpu->disp_irq0_domain) {
		dev_err(dev, "failed to add display irq0 domain\n");
		ret = -ENODEV;
		goto err0;
	}

	dpu->disp_irq2_domain = irq_domain_add_linear(dev->of_node,
						      data->irq_cnt,
						      &irq_generic_chip_ops,
						      dpu);
	if (!dpu->disp_irq2_domain) {
		dev_err(dev, "failed to add display irq2 domain\n");
		ret = -ENODEV;
		goto err1;
	}

	ret = irq_alloc_domain_generic_chips(dpu->comctrl_irq_domain, 32, 1,
					     "DPU COMCTRL IRQ",
					     handle_level_irq, 0, 0, 0);
	if (ret < 0) {
		dev_err(dev, "failed to alloc generic irq chips for comctrl irq: %d\n",
			ret);
		goto err2;
	}

	ret = irq_alloc_domain_generic_chips(dpu->disp_irq0_domain, 32, 1,
					     "DPU DISP IRQ0",
					     handle_level_irq, 0, 0, 0);
	if (ret < 0) {
		dev_err(dev, "failed to alloc generic irq chips for display irq0: %d\n",
			ret);
		goto err2;
	}

	ret = irq_alloc_domain_generic_chips(dpu->disp_irq2_domain, 32, 1,
					     "DPU DISP IRQ2",
					     handle_level_irq, 0, 0, 0);
	if (ret < 0) {
		dev_err(dev, "failed to alloc generic irq chips for display irq2: %d\n",
			ret);
		goto err2;
	}

	for (i = 0; i < data->irq_cnt; i += 32) {
		gc = irq_get_domain_generic_chip(dpu->comctrl_irq_domain, i);
		gc->reg_base = dpu->comctrl_irq_reg;
		gc->unused = data->unused_irq[i / 32];
		ct = gc->chip_types;
		ct->chip.irq_ack = irq_gc_ack_set_bit;
		ct->chip.irq_mask = irq_gc_mask_clr_bit;
		ct->chip.irq_unmask = irq_gc_mask_set_bit;
		ct->regs.ack = INTERRUPTCLEAR(i / 32);
		ct->regs.mask = INTERRUPTENABLE(i / 32);
	}

	for (i = 0; i < data->irq_cnt; i += 32) {
		gc = irq_get_domain_generic_chip(dpu->disp_irq0_domain, i);
		gc->reg_base = dpu->disp_irq0_reg;
		gc->unused = data->unused_irq[i / 32];
		ct = gc->chip_types;
		ct->chip.irq_ack = irq_gc_ack_set_bit;
		ct->chip.irq_mask = irq_gc_mask_clr_bit;
		ct->chip.irq_unmask = irq_gc_mask_set_bit;
		ct->regs.ack = INTERRUPTCLEAR(i / 32);
		ct->regs.mask = INTERRUPTENABLE(i / 32);
	}

	for (i = 0; i < data->irq_cnt; i += 32) {
		gc = irq_get_domain_generic_chip(dpu->disp_irq2_domain, i);
		gc->reg_base = dpu->disp_irq2_reg;
		gc->unused = data->unused_irq[i / 32];
		ct = gc->chip_types;
		ct->chip.irq_ack = irq_gc_ack_set_bit;
		ct->chip.irq_mask = irq_gc_mask_clr_bit;
		ct->chip.irq_unmask = irq_gc_mask_set_bit;
		ct->regs.ack = INTERRUPTCLEAR(i / 32);
		ct->regs.mask = INTERRUPTENABLE(i / 32);
	}

	ret = pm_runtime_resume_and_get(parent_domain->pm_dev);
	if (ret < 0) {
		dev_err(dev, "failed to get parent irq domain RPM: %d\n", ret);
		goto err2;
	}

	for (i = 0; i < data->irq_cnt; i++) {
		if (!data->comctrl_irq_handler[i])
			continue;

		for (j = 0; j < data->comctrl_irq_cnt; j++) {
			if (data->comctrl_irq[j] != i)
				continue;

			irq_set_chained_handler_and_data(dpu->comctrl_irq[j],
							 data->comctrl_irq_handler[i],
							 dpu);
			break;
		}
	}

	for (i = 0; i < data->irq_cnt; i++) {
		if (!data->disp_irq0_handler[i])
			continue;

		for (j = 0; j < data->disp_irq0_cnt; j++) {
			if (data->disp_irq0[j] != i)
				continue;

			irq_set_chained_handler_and_data(dpu->disp_irq0[j],
							 data->disp_irq0_handler[i],
							 dpu);
			break;
		}
	}

	for (i = 0; i < data->irq_cnt; i++) {
		if (!data->disp_irq2_handler[i])
			continue;

		for (j = 0; j < data->disp_irq2_cnt; j++) {
			if (data->disp_irq2[j] != i)
				continue;

			irq_set_chained_handler_and_data(dpu->disp_irq2[j],
							 data->disp_irq2_handler[i],
							 dpu);
			break;
		}
	}

	pm_runtime_put(parent_domain->pm_dev);

	return 0;

err2:
	irq_domain_remove(dpu->disp_irq2_domain);
err1:
	irq_domain_remove(dpu->disp_irq0_domain);
err0:
	irq_domain_remove(dpu->comctrl_irq_domain);
	return ret;
}

static void devm_dpu95_irq_exit(void *data)
{
	struct irq_domain *parent_domain;
	struct dpu95_soc *dpu = data;
	unsigned int irq;
	int ret, i, j;

	parent_domain = dpu95_find_parent_irq_domain(dpu->dev);
	if (!parent_domain)
		return;

	ret = pm_runtime_resume_and_get(parent_domain->pm_dev);
	if (ret < 0) {
		dev_err(dpu->dev, "failed to get parent irq domain RPM: %d\n",
			ret);
		return;
	}

	for (i = 0; i < dpu->data->irq_cnt; i++) {
		if (!dpu->data->comctrl_irq_handler[i])
			continue;

		for (j = 0; j < dpu->data->comctrl_irq_cnt; j++) {
			if (dpu->data->comctrl_irq[j] != i)
				continue;

			irq_set_chained_handler_and_data(dpu->comctrl_irq[j],
							 NULL, NULL);
			break;
		}
	}

	for (i = 0; i < dpu->data->irq_cnt; i++) {
		if (!dpu->data->disp_irq0_handler[i])
			continue;

		for (j = 0; j < dpu->data->disp_irq0_cnt; j++) {
			if (dpu->data->disp_irq0[j] != i)
				continue;

			irq_set_chained_handler_and_data(dpu->disp_irq0[j],
							 NULL, NULL);
			break;
		}
	}

	for (i = 0; i < dpu->data->irq_cnt; i++) {
		if (!dpu->data->disp_irq2_handler[i])
			continue;

		for (j = 0; j < dpu->data->disp_irq2_cnt; j++) {
			if (dpu->data->disp_irq2[j] != i)
				continue;

			irq_set_chained_handler_and_data(dpu->disp_irq2[j],
							 NULL, NULL);
			break;
		}
	}

	pm_runtime_put(parent_domain->pm_dev);

	for (i = 0; i < dpu->data->comctrl_irq_cnt; i++) {
		irq = irq_find_mapping(dpu->comctrl_irq_domain,
				       dpu->data->comctrl_irq[i]);
		if (irq)
			irq_dispose_mapping(irq);
	}

	for (i = 0; i < dpu->data->disp_irq0_cnt; i++) {
		irq = irq_find_mapping(dpu->disp_irq0_domain,
				       dpu->data->disp_irq0[i]);
		if (irq)
			irq_dispose_mapping(irq);
	}

	for (i = 0; i < dpu->data->disp_irq2_cnt; i++) {
		irq = irq_find_mapping(dpu->disp_irq2_domain,
				       dpu->data->disp_irq2[i]);
		if (irq)
			irq_dispose_mapping(irq);
	}

	irq_domain_remove(dpu->comctrl_irq_domain);
	irq_domain_remove(dpu->disp_irq0_domain);
	irq_domain_remove(dpu->disp_irq2_domain);
}

void dpu95_submodules_hw_init(struct dpu95_soc *dpu)
{
	const struct dpu95_units *us;
	int i, j;

	dpu95_dm_extdst0_dm_allow_all(dpu);
	dpu95_dm_extdst1_dm_allow_all(dpu);
	dpu95_dm_extdst4_dm_allow_all(dpu);
	dpu95_dm_extdst5_dm_allow_all(dpu);

	dpu95_dm_extdst0_master(dpu);
	dpu95_dm_extdst1_master(dpu);
	dpu95_dm_extdst4_master(dpu);
	dpu95_dm_extdst5_master(dpu);

	for (i = 0; i < dpu->data->units_cnt; i++) {
		us = dpu->data->units[i];

		for (j = 0; j < us->cnt; j++)
			us->hw_init(dpu, j);
	}
}

static int dpu95_submodules_init(struct dpu95_soc *dpu, unsigned long dpu_base)
{
	const struct dpu95_units *us;
	unsigned long aux_ofs;
	int i, j, ret;

	for (i = 0; i < dpu->data->units_cnt; i++) {
		us = dpu->data->units[i];

		switch (us->name) {
		case DPU95_FETCHECO:
			dpu->fe = devm_kcalloc(dpu->dev, us->cnt,
					       sizeof(struct dpu95_fetchunit *),
					       GFP_KERNEL);
			if (!dpu->fe)
				return -ENOMEM;
			dpu->fe_cnt = us->cnt;
			break;
		case DPU95_FETCHLAYER:
			dpu->fl = devm_kcalloc(dpu->dev, us->cnt,
					       sizeof(struct dpu95_fetchunit *),
					       GFP_KERNEL);
			if (!dpu->fl)
				return -ENOMEM;
			dpu->fl_cnt = us->cnt;
			break;
		case DPU95_FETCHYUV:
			dpu->fy = devm_kcalloc(dpu->dev, us->cnt,
					       sizeof(struct dpu95_fetchunit *),
					       GFP_KERNEL);
			if (!dpu->fy)
				return -ENOMEM;
			dpu->fy_cnt = us->cnt;
			break;
		case DPU95_LAYERBLEND:
			dpu->lb = devm_kcalloc(dpu->dev, us->cnt,
					       sizeof(struct dpu95_layerblend *),
					       GFP_KERNEL);
			if (!dpu->lb)
				return -ENOMEM;
			dpu->lb_cnt = us->cnt;
			break;
		default:
			break;
		}

		for (j = 0; j < us->cnt; j++) {
			aux_ofs = us->aux_ofss ? dpu_base + us->aux_ofss[j] : 0;

			ret = us->init(dpu, j, us->ids[j], us->types[j],
				       aux_ofs, dpu_base + us->ofss[j]);
			if (ret) {
				dev_err(dpu->dev,
					"failed to initialize %s%d: %d\n",
					dpu95_unit_names[us->name], us->ids[j],
					ret);
				return ret;
			}
		}
	}

	return 0;
}

static int dpu95_get_layerblends_for_plane_grp(struct dpu95_soc *dpu,
					       struct dpu95_plane_res *res)
{
	const struct dpu95_units *us;
	int i, ret;

	for (i = 0; i < dpu->data->units_cnt; i++) {
		us = dpu->data->units[i];
		if (us->name == DPU95_LAYERBLEND)
			break;
	}

	res->lb_cnt = dpu->lb_cnt;

	res->lb = devm_kcalloc(dpu->dev, res->lb_cnt,
			       sizeof(struct dpu95_layerblend *), GFP_KERNEL);
	if (!res->lb)
		return -ENOMEM;

	for (i = 0; i < us->cnt; i++) {
		res->lb[i] = dpu95_lb_get(dpu, us->ids[i]);
		if (IS_ERR(res->lb[i])) {
			ret = PTR_ERR(res->lb[i]);
			dev_err(dpu->dev, "failed to get %s%d: %d\n",
				dpu95_unit_names[us->name], us->ids[i], ret);
			return ret;
		}
	}

	return 0;
}

static int dpu95_get_plane_grp_res(struct dpu95_soc *dpu,
				   struct dpu95_plane_grp *grp)
{
	struct dpu95_plane_res *res = &grp->res;
	const struct dpu95_units *us;
	int i, ret;

	ret = dpu95_get_layerblends_for_plane_grp(dpu, res);
	if (ret)
		return ret;

	INIT_LIST_HEAD(&grp->fu_list);

	for (i = 0; i < dpu->data->units_cnt; i++) {
		us = dpu->data->units[i];
		if (us->name == DPU95_FETCHYUV)
			break;
	}

	res->fy_cnt = dpu->fy_cnt;

	res->fy = devm_kcalloc(dpu->dev, res->fy_cnt,
			       sizeof(struct dpu95_fetchunit *), GFP_KERNEL);
	if (!res->fy)
		return -ENOMEM;

	for (i = 0; i < us->cnt; i++) {
		res->fy[i] = dpu95_fy_get(dpu, us->ids[i]);
		if (IS_ERR(res->fy[i])) {
			ret = PTR_ERR(res->fy[i]);
			dev_err(dpu->dev, "failed to get %s%d: %d\n",
				dpu95_unit_names[us->name], us->ids[i], ret);
			return ret;
		}
	}

	for (i = us->cnt - 1; i >= 0; i--)
		dpu95_fu_add_to_list(res->fy[i], &grp->fu_list);

	for (i = 0; i < dpu->data->units_cnt; i++) {
		us = dpu->data->units[i];
		if (us->name == DPU95_FETCHLAYER)
			break;
	}

	res->fl_cnt = dpu->fl_cnt;

	res->fl = devm_kcalloc(dpu->dev, res->fl_cnt,
			       sizeof(struct dpu95_fetchunit *), GFP_KERNEL);
	if (!res->fl)
		return -ENOMEM;

	for (i = 0; i < us->cnt; i++) {
		res->fl[i] = dpu95_fl_get(dpu, us->ids[i]);
		if (IS_ERR(res->fl[i])) {
			ret = PTR_ERR(res->fl[i]);
			dev_err(dpu->dev, "failed to get %s%d: %d\n",
				dpu95_unit_names[us->name], us->ids[i], ret);
			return ret;
		}
	}

	for (i = us->cnt - 1; i >= 0; i--)
		dpu95_fu_add_to_list(res->fl[i], &grp->fu_list);

	return 0;
}

int dpu95_core_init(struct dpu95_drm_device *dpu_drm)
{
	struct drm_device *drm = &dpu_drm->base;
	struct device *dev = drm->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct dpu95_soc *dpu = &dpu_drm->dpu_soc;
	struct device_node *np = dev->of_node;
	unsigned long dpu_base;
	struct resource *res;
	int ret;

	dpu->data = of_device_get_match_data(dev);
	if (!dpu->data)
		return -EINVAL;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	dpu_base = res->start;

	dpu->dev = dev;

	dpu->comctrl_irq_reg = devm_ioremap(dev, dpu_base + 0x1000, SZ_64);
	if (!dpu->comctrl_irq_reg)
		return -ENOMEM;

	dpu->dm_mask_reg = devm_ioremap(dev, dpu_base + 0x2000, SZ_64);
	if (!dpu->dm_mask_reg)
		return -ENOMEM;

	dpu->disp_irq0_reg = devm_ioremap(dev, dpu_base + dpu->data->irq0_addr,
					  SZ_64);
	if (!dpu->disp_irq0_reg)
		return -ENOMEM;

	dpu->disp_irq2_reg = devm_ioremap(dev, dpu_base + dpu->data->irq2_addr,
					  SZ_64);
	if (!dpu->disp_irq2_reg)
		return -ENOMEM;

	dpu->regmap = syscon_regmap_lookup_by_phandle(np, "nxp,blk-ctrl");
	if (IS_ERR(dpu->regmap)) {
		ret = PTR_ERR(dpu->regmap);
		dev_err_probe(dev, ret, "failed to get blk-ctrl regmap\n");
		return ret;
	}

	dpu->clk_axi = devm_clk_get(dev, "axi");
	if (IS_ERR(dpu->clk_axi))
		return dev_err_probe(dev, PTR_ERR(dpu->clk_axi),
				     "failed to get AXI clock\n");

	dpu->clk_apb = devm_clk_get(dev, "apb");
	if (IS_ERR(dpu->clk_apb))
		return dev_err_probe(dev, PTR_ERR(dpu->clk_apb),
				     "failed to get APB clock\n");

	dpu->clk_pix = devm_clk_get(dev, "pix");
	if (IS_ERR(dpu->clk_pix))
		return dev_err_probe(dev, PTR_ERR(dpu->clk_pix),
				     "failed to get pixel clock\n");

	dpu->clk_ocram = devm_clk_get(dev, "ocram");
	if (IS_ERR(dpu->clk_ocram))
		return dev_err_probe(dev, PTR_ERR(dpu->clk_ocram),
				     "failed to get ocram clock\n");

	dpu->clk_ldb = devm_clk_get(dev, "ldb");
	if (IS_ERR(dpu->clk_ldb))
		return dev_err_probe(dev, PTR_ERR(dpu->clk_ldb),
				     "failed to get ldb clock\n");

	dpu->clk_ldb_vco = devm_clk_get(dev, "ldb_vco");
	if (IS_ERR(dpu->clk_ldb_vco))
		return dev_err_probe(dev, PTR_ERR(dpu->clk_ldb_vco),
				     "failed to get ldb_vco clock\n");

	ret = dpu95_submodules_init(dpu, dpu_base);
	if (ret)
		return ret;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(40));
	if (ret)
		return ret;

	ret = dpu95_get_plane_grp_res(dpu, &dpu_drm->dpu_plane_grp);
	if (ret)
		return ret;

	ret = dpu95_irq_init(pdev, dpu);
	if (ret)
		return ret;

	return devm_add_action_or_reset(dev, devm_dpu95_irq_exit, dpu);
}
