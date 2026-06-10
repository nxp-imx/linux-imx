// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2026 NXP
 * Author: Robert Chiras <robert.chiras@nxp.com>
 *
 * Remoteproc driver for generic FW running on an M0+ core.
 *
 */

#include <linux/clk.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>

#include "imx_rproc.h"

#define IMX95_BLKCTL_REG_CM0P_CLK_GATE		0x0
#define IMX95_BLKCTL_REG_CM0P_ADDR_OFFSET1	0x4
#define IMX95_BLKCTL_REG_CM0P_ADDR_OFFSET2	0x8
#define IMX95_BLKCTL_REG_CM0P_CPUWAIT		0xc
#define IMX95_BLKCTL_REG_CM0P_CTL		0x10
#define IMX95_BLKCTL_REG_CM0P_STAT		0x14
#define IMX95_BLKCTL_CM0P_CPUWAIT_CPW		BIT(0)
#define IMX95_BLKCTL_CM0P_CPUWAIT_RST		BIT(1)

#define IMX95_CSR_REG_SRAMCTL_RAMCR	0x0
#define IMX95_CSR_REG_SRAMCTL_RAMIAS	0x4
#define IMX95_CSR_REG_SRAMCTL_RAMIAE	0x8
#define IMX95_CSR_REG_SRAMCTL_RAMSR	0xc
#define IMX95_CSR_RAMSR_IDONE		BIT(0)

static const struct reg_field ocram_done =
			REG_FIELD(IMX95_CSR_REG_SRAMCTL_RAMSR, 0, 0);

#define MAX_WAIT 20000

struct imx_cm0p_rproc {
	struct rproc *rproc;
	struct device *dev;

	struct clk *cm0p_clk;
	struct clk *ocram_clk;
	struct clk *spi_clk;
	struct regmap *cm0p_ctl;
	struct regmap *ocram_ctl;
	struct regmap *ocram_cfg;
	const struct imx_cm0p_rproc_dcfg *cfg;

	phys_addr_t mba_phys;
	size_t mba_size;
	bool mba_init;
};

/* Custom registers to initialize */
struct imx_cm0p_dcfg {
	u32 src_reg;
	u32 src_mask;
	u32 src_val;
};

struct imx_cm0p_rproc_dcfg {
	const struct imx_rproc_dcfg *dcfg;
	const struct imx_cm0p_dcfg cm0p_dcfg[3];
};

static const struct imx_rproc_dcfg cm0p_rproc_cfg_imx952 = {
	.src_reg	= IMX95_BLKCTL_REG_CM0P_CPUWAIT,
	.src_mask	= IMX95_BLKCTL_CM0P_CPUWAIT_CPW |
			  IMX95_BLKCTL_CM0P_CPUWAIT_RST,
	.src_start	= IMX95_BLKCTL_CM0P_CPUWAIT_RST,
	.src_stop	= IMX95_BLKCTL_CM0P_CPUWAIT_CPW,
	.method		= IMX_RPROC_MMIO,
};

static const struct imx_cm0p_rproc_dcfg imx_cm0p_rproc_cfg_imx952 = {
	.dcfg		= &cm0p_rproc_cfg_imx952,
	.cm0p_dcfg	= {
		{
			.src_reg = IMX95_BLKCTL_REG_CM0P_CLK_GATE,
			.src_mask = 0x1,
			.src_val = 0X0
		},
		{
			.src_reg = IMX95_BLKCTL_REG_CM0P_ADDR_OFFSET1,
			.src_mask = 0xFFFFFF00,
			.src_val = 0X80000000
		},
		{
			.src_reg = IMX95_BLKCTL_REG_CM0P_ADDR_OFFSET2,
			.src_mask = 0xFFFFFF00,
			.src_val = 0X20000000
		},
	}
};

static int imx_cm0p_start(struct rproc *rproc)
{
	struct imx_cm0p_rproc *cm0p = (struct imx_cm0p_rproc *)rproc->priv;
	const struct imx_cm0p_rproc_dcfg *cfg = cm0p->cfg;
	const struct imx_rproc_dcfg *dcfg = cfg->dcfg;
	int ret = 0;
	int i;

	ret = clk_prepare_enable(cm0p->cm0p_clk);
	if (ret) {
		dev_err(cm0p->dev, "failed to enable M0 clock: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(cm0p->spi_clk);
	if (ret)
		dev_warn(cm0p->dev, "failed to enable SPI clock: %d\n", ret);

	regmap_read(cm0p->cm0p_ctl, IMX95_BLKCTL_REG_CM0P_CLK_GATE, &i);

	for (i = 0; i < ARRAY_SIZE(cfg->cm0p_dcfg); i++) {
		const struct imx_cm0p_dcfg *cm0p_cfg = &cfg->cm0p_dcfg[i];

		if (!cm0p_cfg->src_reg)
			continue;

		regmap_update_bits(cm0p->cm0p_ctl,
				   cm0p_cfg->src_reg,
				   cm0p_cfg->src_mask,
				   cm0p_cfg->src_val);
	}

	regmap_read(cm0p->cm0p_ctl, IMX95_BLKCTL_REG_CM0P_CLK_GATE, &i);

	/* Start the processor */
	ret |= regmap_update_bits(cm0p->cm0p_ctl,
				 dcfg->src_reg,
				 dcfg->src_mask,
				 IMX95_BLKCTL_CM0P_CPUWAIT_CPW |
				 IMX95_BLKCTL_CM0P_CPUWAIT_RST);
	ret |= regmap_update_bits(cm0p->cm0p_ctl,
				 dcfg->src_reg,
				 dcfg->src_mask,
				 dcfg->src_start);
	if (ret) {
		dev_err(cm0p->dev, "Failed to enable remote core!\n");
		clk_disable_unprepare(cm0p->cm0p_clk);
		clk_disable_unprepare(cm0p->spi_clk);
	}

	return ret;
}

static int imx_cm0p_stop(struct rproc *rproc)
{
	struct imx_cm0p_rproc *cm0p = (struct imx_cm0p_rproc *)rproc->priv;
	const struct imx_cm0p_rproc_dcfg *cfg = cm0p->cfg;
	const struct imx_rproc_dcfg *dcfg = cfg->dcfg;
	int ret;

	/* Stop the processor */
	cm0p->mba_init = false;
	ret = regmap_update_bits(cm0p->cm0p_ctl,
				 dcfg->src_reg,
				 dcfg->src_mask,
				 dcfg->src_stop);

	if (ret) {
		dev_err(cm0p->dev, "Failed to disable remote core!\n");
	} else {
		clk_disable_unprepare(cm0p->cm0p_clk);
		clk_disable_unprepare(cm0p->spi_clk);
	}

	return ret;
}

static int imx_cm0p_alloc_memory_region(struct imx_cm0p_rproc *cm0p)
{
	struct regmap_field *field;
	struct device_node *child;
	struct device_node *node;
	unsigned int map[2] = {0};
	struct resource r;
	u32 init_done = 0;
	int ret = 0;

	child = of_get_child_by_name(cm0p->dev->of_node, "mba");
	if (!child) {
		node = of_parse_phandle(cm0p->dev->of_node,
					"memory-region", 0);
	} else {
		node = of_parse_phandle(child, "memory-region", 0);
		of_property_read_variable_u32_array(child, "ocram-map",
						    map, 0, ARRAY_SIZE(map));
		of_node_put(child);
	}

	ret = of_address_to_resource(node, 0, &r);
	of_node_put(node);
	if (ret) {
		dev_err(cm0p->dev, "unable to resolve memory-region\n");
		return ret;
	}

	cm0p->mba_phys = r.start;
	cm0p->mba_size = resource_size(&r);

	if (!map[0])
		map[0] = 0;
	if (!map[1])
		map[1] = cm0p->mba_size;

	/* Initialize OCRAM */
	dev_info(cm0p->dev, "OCRAM init: %pa+%zx", &cm0p->mba_phys, cm0p->mba_size);
	field = devm_regmap_field_alloc(cm0p->dev, cm0p->ocram_cfg, ocram_done);
	if (IS_ERR(field)) {
		dev_err(cm0p->dev, "ocram_done field alloc failed\n");
		return PTR_ERR(field);
	}

	ret = regmap_update_bits(cm0p->ocram_cfg,
				 IMX95_CSR_REG_SRAMCTL_RAMIAS,
				 ~0,
				 map[0]);
	ret |= regmap_update_bits(cm0p->ocram_cfg,
				  IMX95_CSR_REG_SRAMCTL_RAMIAE,
				  ~0,
				  map[1]);
	/* Request initialization */
	ret |= regmap_update_bits(cm0p->ocram_cfg,
				  IMX95_CSR_REG_SRAMCTL_RAMCR,
				  BIT(0),
				  BIT(0));

	if (ret) {
		dev_err(cm0p->dev, "OCRAM init map failed: %d\n", ret);
		return ret;
	}

	regmap_field_read_poll_timeout(field, init_done, init_done, 200, MAX_WAIT);

	if (!init_done) {
		dev_err(cm0p->dev, "OCRAM initialization failed\n");
		return -EFAULT;
	}

	ret = regmap_update_bits(cm0p->ocram_cfg,
				 IMX95_CSR_REG_SRAMCTL_RAMSR,
				 BIT(0),
				 BIT(0));

	dev_info(cm0p->dev, "OCRAM mapped at: %u+%x", map[0], map[1]);

	cm0p->mba_init = true;

	return ret;
}

static int imx_cm0p_load(struct rproc *rproc, const struct firmware *fw)
{
	struct imx_cm0p_rproc *cm0p = rproc->priv;
	void *mba_region;
	int ret = 0;

	ret = pm_runtime_resume_and_get(cm0p->dev);
	if (ret < 0) {
		dev_err(cm0p->dev, "pm_runtime_resume_and_get failed: %d\n",
			ret);
		return ret;
	}

	if (!cm0p->mba_init) {
		ret = imx_cm0p_alloc_memory_region(cm0p);
		if (ret)
			goto err_put_rpm;
	}

	if (fw->size > cm0p->mba_size) {
		dev_err(cm0p->dev, "CM0+ firmware file too big: %lu > %lu",
			fw->size, cm0p->mba_size);
		ret = -EINVAL;
		goto err_put_rpm;
	}

	mba_region = memremap(cm0p->mba_phys, cm0p->mba_size, MEMREMAP_WC);
	if (!mba_region) {
		dev_err(cm0p->dev, "unable to map memory region: %pa+%zx\n",
			&cm0p->mba_phys, cm0p->mba_size);
		ret = -EBUSY;
		goto err_put_rpm;
	}

	memcpy(mba_region, fw->data, fw->size);
	memunmap(mba_region);
	dev_info(cm0p->dev, "CM0+ firmware (%lu bytes) loaded to: %pa+%zx",
		 fw->size, &cm0p->mba_phys, cm0p->mba_size);

err_put_rpm:
	pm_runtime_put(cm0p->dev);
	return ret;
}

/* pm runtime functions */
static int imx_cm0p_runtime_resume(struct device *dev)
{
	struct rproc *rproc = dev_get_drvdata(dev);
	struct imx_cm0p_rproc *cm0p = rproc->priv;
	int ret;

	ret = clk_prepare_enable(cm0p->cm0p_clk);
	if (ret) {
		dev_err(cm0p->dev, "failed to enable M0 clock: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(cm0p->spi_clk);
	if (ret)
		dev_warn(cm0p->dev, "failed to enable SPI clock: %d\n", ret);

	ret = clk_prepare_enable(cm0p->ocram_clk);
	if (ret) {
		dev_err(cm0p->dev, "failed to enable OCRAM clock: %d\n", ret);
		return ret;
	}

	return 0;
}

static int imx_cm0p_runtime_suspend(struct device *dev)
{
	struct rproc *rproc = dev_get_drvdata(dev);
	struct imx_cm0p_rproc *cm0p = rproc->priv;

	clk_disable_unprepare(cm0p->cm0p_clk);
	clk_disable_unprepare(cm0p->spi_clk);
	clk_disable_unprepare(cm0p->ocram_clk);

	return 0;
}

static void imx_cm0p_load_firmware(const struct firmware *fw, void *context)
{
	struct rproc *rproc = context;
	int ret;

	ret = imx_cm0p_load(rproc, fw);
	if (ret)
		return;

	/* Start the remote processor */
	if (rproc->state == RPROC_RUNNING)
		rproc->ops->start(rproc);
}


static int imx_cm0p_suspend(struct device *dev)
{
	struct rproc *rproc = dev_get_drvdata(dev);
	int ret;

	/* Stop the remote processor */
	ret = rproc->ops->stop(rproc);
	if (ret)
		return ret;

	return pm_runtime_force_suspend(dev);
}

static int imx_cm0p_resume(struct device *dev)
{
	struct rproc *rproc = dev_get_drvdata(dev);
	int ret = 0;

	ret = pm_runtime_force_resume(dev);
	if (ret)
		return ret;

	if (rproc->state != RPROC_RUNNING)
		return 0;

	ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_UEVENT,
				      rproc->firmware, dev, GFP_KERNEL,
				      rproc, imx_cm0p_load_firmware);
	if (ret < 0) {
		dev_err(dev, "load firmware failed: %d\n", ret);
		goto err;
	}

	return 0;

err:
	pm_runtime_force_suspend(dev);

	return ret;
}

static const struct rproc_ops imx_cm0p_rproc_ops = {
	.start = imx_cm0p_start,
	.stop = imx_cm0p_stop,
	.load = imx_cm0p_load,
};

static int imx_cm0p_rproc_probe(struct platform_device *pdev)
{
	const struct imx_cm0p_rproc_dcfg *cm0p_cfg;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct imx_cm0p_rproc *cm0p_rproc;
	const char *fw_name;
	struct rproc *rproc;
	int ret;

	cm0p_cfg = of_device_get_match_data(dev);
	if (!cm0p_cfg)
		return -ENODEV;

	ret = of_property_read_string(np, "firmware-name",
				      &fw_name);
	if (ret) {
		dev_err(dev, "No firmware filename given\n");
		return -ENODEV;
	}

	rproc = rproc_alloc(dev, "imx-cm0p-rproc", &imx_cm0p_rproc_ops,
			    fw_name, sizeof(*cm0p_rproc));
	if (!rproc)
		return -ENOMEM;

	rproc->auto_boot = false;

	cm0p_rproc = (struct imx_cm0p_rproc *)rproc->priv;
	cm0p_rproc->rproc = rproc;
	cm0p_rproc->cfg = cm0p_cfg;
	cm0p_rproc->dev = dev;

	cm0p_rproc->cm0p_clk = devm_clk_get(dev, "cm0p");
	if (IS_ERR(cm0p_rproc->cm0p_clk)) {
		ret = dev_err_probe(dev, PTR_ERR(cm0p_rproc->cm0p_clk),
				    "failed to get M0 clock\n");
		goto err_rproc;
	}

	/* OCRAM clock is optional */
	cm0p_rproc->ocram_clk = devm_clk_get_optional(dev, "ocram");

	/* LPSPI clock is optional */
	cm0p_rproc->spi_clk = devm_clk_get_optional(dev, "lpspi");

	cm0p_rproc->cm0p_ctl = syscon_regmap_lookup_by_phandle(np, "nxp,cm0p-ctrl");
	if (IS_ERR(cm0p_rproc->cm0p_ctl)) {
		ret = PTR_ERR(cm0p_rproc->cm0p_ctl);
		dev_err_probe(dev, ret, "failed to get cm0p-ctrl: %d\n", ret);
		goto err_rproc;
	}

	/* ocram-ctrl is optional */
	cm0p_rproc->ocram_ctl = syscon_regmap_lookup_by_phandle(np, "nxp,ocram-ctrl");

	cm0p_rproc->ocram_cfg = syscon_regmap_lookup_by_phandle(np, "nxp,ocram-cfg");
	if (IS_ERR(cm0p_rproc->ocram_cfg)) {
		ret = PTR_ERR(cm0p_rproc->ocram_cfg);
		dev_err_probe(dev, ret, "failed to get ocram-cfg: %d\n", ret);
		goto err_rproc;
	}

	dev_set_drvdata(dev, rproc);

	ret = rproc_add(rproc);
	if (ret) {
		dev_err(dev, "rproc_add failed\n");
		rproc_free(rproc);
		goto err_clk;
	}

	return devm_pm_runtime_enable(dev);

err_clk:
	clk_disable_unprepare(cm0p_rproc->ocram_clk);
err_rproc:
	rproc_free(rproc);

	return ret;
};

static void imx_cm0p_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct imx_cm0p_rproc *cm0p = rproc->priv;

	rproc_del(rproc);
	rproc_free(rproc);

	clk_disable_unprepare(cm0p->ocram_clk);

	pm_runtime_disable(&pdev->dev);
}

static const struct dev_pm_ops imx_cm0p_rproc_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(imx_cm0p_suspend, imx_cm0p_resume)
	RUNTIME_PM_OPS(imx_cm0p_runtime_suspend, imx_cm0p_runtime_resume, NULL)
};

static const struct of_device_id imx_cm0p_rproc_of_match[] = {
	{ .compatible = "nxp,imx952-cm0p-rproc",
	  .data = &imx_cm0p_rproc_cfg_imx952,
	},
	{},
};
MODULE_DEVICE_TABLE(of, imx_cm0p_rproc_of_match);

static struct platform_driver imx_cm0p_rproc_driver = {
	.probe = imx_cm0p_rproc_probe,
	.remove = imx_cm0p_rproc_remove,
	.driver = {
		.name = "imx-cm0p-rproc",
		.of_match_table = imx_cm0p_rproc_of_match,
		.pm = pm_ptr(&imx_cm0p_rproc_pm_ops),
	},
};
module_platform_driver(imx_cm0p_rproc_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("i.MX CM0+ Core Remote Processor Control Driver");
MODULE_AUTHOR("Robert Chiras <robert.chiras@nxp.com>");
