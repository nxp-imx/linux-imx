// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright 2025 NXP
 */

#include <linux/dma-mapping.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#include "dcif-drv.h"
#include "dcif-reg.h"

#define QOS_SETTING			0x1c
#define  DISPLAY_PANIC_QOS_MASK		0x70
#define  DISPLAY_PANIC_QOS(n)		(((n) & 0x7) << 4)
#define  DISPLAY_ARQOS_MASK		0x7
#define  DISPLAY_ARQOS(n)		((n) & 0x7)

#define DCIF_CPU_DOMAIN			2

#define DRIVER_NAME			"imx-dcif-drm"

static int legacyfb_depth = 32;
module_param(legacyfb_depth, uint, 0444);

DEFINE_DRM_GEM_DMA_FOPS(dcif_driver_fops);

static struct drm_driver dcif_driver = {
	.driver_features	= DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,
	DRM_GEM_DMA_DRIVER_OPS,
	.fops			= &dcif_driver_fops,
	.name			= "imx-dcif",
	.desc			= "i.MX DCIF DRM graphics",
	.date			= "20240305",
	.major			= 1,
	.minor			= 0,
	.patchlevel		= 0,
};

static int dcif_set_qos(struct dcif_dev *dcif)
{
	struct drm_device *drm = &dcif->drm;
	int ret;

	ret = regmap_update_bits(dcif->blkctrl_regmap, QOS_SETTING,
				 DISPLAY_PANIC_QOS_MASK | DISPLAY_ARQOS_MASK,
				 DISPLAY_PANIC_QOS(0x3) | DISPLAY_ARQOS(0x3));
	if (ret < 0)
		dev_err(drm->dev, "failed to set QoS: %d\n", ret);

	return ret;
}

static void dcif_read_chip_info(struct dcif_dev *dcif)
{
	struct drm_device *drm = &dcif->drm;
	u32 val, vmin, vmaj;

	pm_runtime_get_sync(drm->dev);

	regmap_read(dcif->regmap, DCIF_VER, &val);

	dcif->has_crc = val & 0x2;

	vmin = DCIF_VER_GET_MINOR(val);
	vmaj = DCIF_VER_GET_MAJOR(val);
	DRM_DEV_DEBUG(drm->dev, "DCIF version is %d.%d\n", vmaj, vmin);

	pm_runtime_put_sync(drm->dev);
}

static const struct regmap_config dcif_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.fast_io = true,
	.max_register = 0x20250,
	.cache_type = REGCACHE_NONE,
	.disable_locking = true,
};

static const char * const dcif_clks[] = {
	"apb",
	"axi",
	"pix",
};

static int dcif_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct dcif_dev *dcif;
	struct drm_device *drm;
	int ret;
	int i;

	if (!pdev->dev.of_node)
		return -ENODEV;

	dcif = devm_drm_dev_alloc(&pdev->dev, &dcif_driver, struct dcif_dev, drm);
	if (IS_ERR(dcif))
		return PTR_ERR(dcif);

	/* CPU 0 domain for interrupt control */
	dcif->cpu_domain = DCIF_CPU_DOMAIN;

	drm = &dcif->drm;
	dev_set_drvdata(&pdev->dev, dcif);

	dcif->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dcif->reg_base))
		return dev_err_probe(drm->dev, PTR_ERR(dcif->reg_base),
				     "failed to get reg base\n");

	dcif->irq[0] = platform_get_irq(pdev, 0);
	if (dcif->irq[0] < 0)
		return dev_err_probe(drm->dev, dcif->irq[0],
				     "failed to get domain0 irq\n");

	dcif->irq[1] = platform_get_irq(pdev, 1);
	if (dcif->irq[1] < 0)
		return dev_err_probe(drm->dev, dcif->irq[1],
				     "failed to get domain1 irq\n");

	dcif->irq[2] = platform_get_irq(pdev, 2);
	if (dcif->irq[2] < 0)
		return dev_err_probe(drm->dev, dcif->irq[2],
				     "failed to get domain2 irq\n");

	dcif->blkctrl_regmap = syscon_regmap_lookup_by_phandle(np, "nxp,blk-ctrl");
	if (IS_ERR(dcif->blkctrl_regmap))
		return dev_err_probe(drm->dev, PTR_ERR(dcif->blkctrl_regmap),
				     "failed to get blk-ctrl regmap\n");

	dcif->regmap = devm_regmap_init_mmio(drm->dev, dcif->reg_base, &dcif_regmap_config);
	if (IS_ERR(dcif->regmap))
		return dev_err_probe(drm->dev, PTR_ERR(dcif->regmap),
				     "failed to init DCIF regmap\n");

	dcif->num_clks = ARRAY_SIZE(dcif_clks);
	dcif->clks = devm_kcalloc(drm->dev, dcif->num_clks, sizeof(*dcif->clks), GFP_KERNEL);
	if (!dcif->clks)
		return -ENOMEM;

	for (i = 0; i < dcif->num_clks; i++)
		dcif->clks[i].id = dcif_clks[i];

	ret = devm_clk_bulk_get(drm->dev, dcif->num_clks, dcif->clks);
	if (ret)
		return dev_err_probe(drm->dev, ret, "cannot get required clocks\n");

	ret = dma_set_mask_and_coherent(drm->dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(drm->dev, ret, "failed to set dma mask and coherent\n");

	pm_runtime_enable(drm->dev);

	ret = devm_request_irq(drm->dev, dcif->irq[dcif->cpu_domain],
			       dcif_irq_handler, 0, drm->driver->name, drm);
	if (ret < 0) {
		dev_err(drm->dev, "failed to install IRQ handler: %d\n", ret);
		goto err_irq_install;
	}

	dcif_read_chip_info(dcif);

	ret = dcif_kms_prepare(dcif);
	if (ret)
		goto err_irq_install;

	ret = drm_dev_register(drm, 0);
	if (ret) {
		dev_err(drm->dev, "failed to register drm device: %d\n", ret);
		goto err_register;
	}

	if (legacyfb_depth != 16 && legacyfb_depth != 32) {
		dev_info(drm->dev, "Invalid legacyfb_depth.  Defaulting to 32bpp\n");
		legacyfb_depth = 32;
	}

	drm_fbdev_dma_setup(drm, legacyfb_depth);

	return 0;

err_register:
	drm_kms_helper_poll_fini(drm);
err_irq_install:
	pm_runtime_disable(drm->dev);
	return ret;
}

static void dcif_remove(struct platform_device *pdev)
{
	struct dcif_dev *dcif = dev_get_drvdata(&pdev->dev);
	struct drm_device *drm = &dcif->drm;

	drm_dev_unregister(drm);

	drm_kms_helper_poll_fini(drm);

	drm_atomic_helper_shutdown(drm);

	pm_runtime_disable(drm->dev);
}

static int dcif_runtime_suspend(struct device *dev)
{
	struct dcif_dev *dcif = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(dcif->num_clks, dcif->clks);

	return 0;
}

static int dcif_runtime_resume(struct device *dev)
{
	struct dcif_dev *dcif = dev_get_drvdata(dev);
	int ret;

	ret = clk_bulk_prepare_enable(dcif->num_clks, dcif->clks);
	if (ret) {
		dev_err(dev, "failed to enable clocks: %d\n", ret);
		return ret;
	}

	ret = dcif_set_qos(dcif);
	if (ret) {
		clk_bulk_disable_unprepare(dcif->num_clks, dcif->clks);
		return ret;
	}

	return 0;
}

static int dcif_suspend(struct device *dev)
{
	struct dcif_dev *dcif = dev_get_drvdata(dev);
	int ret;

	ret = drm_mode_config_helper_suspend(&dcif->drm);
	if (ret < 0)
		return ret;

	if (pm_runtime_suspended(dev))
		return 0;
	
	return dcif_runtime_suspend(dev);
}

static int dcif_resume(struct device *dev)
{
	struct dcif_dev *dcif = dev_get_drvdata(dev);
	int ret;

	if (!pm_runtime_suspended(dev)) {
		ret = dcif_runtime_resume(dev);
		if (ret < 0)
			return ret;
	}

	return drm_mode_config_helper_resume(&dcif->drm);
}

static const struct dev_pm_ops dcif_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dcif_suspend, dcif_resume)
	SET_RUNTIME_PM_OPS(dcif_runtime_suspend, dcif_runtime_resume, NULL)
};

static const struct of_device_id dcif_dt_ids[] = {
	{ .compatible = "nxp,imx94-dcif", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, dcif_dt_ids);

static struct platform_driver dcif_platform_driver = {
	.probe	= dcif_probe,
	.remove	= dcif_remove,
	.driver	= {
		.name		= DRIVER_NAME,
		.of_match_table	= dcif_dt_ids,
		.pm		= pm_ptr(&dcif_pm_ops),
	},
};
module_platform_driver(dcif_platform_driver);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("i.MX94 DCIF DRM driver");
MODULE_LICENSE("GPL");
