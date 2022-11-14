// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP.
 */

#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_device.h>
#include <linux/busfreq-imx.h>
#include <drm/drm_modeset_helper.h>
#include <linux/of_reserved_mem.h>

#include "dcss-dev.h"
#include "dcss-kms.h"

static void dcss_clocks_enable(struct dcss_dev *dcss)
{
	if (dcss->hdmi_output) {
		clk_prepare_enable(dcss->pll_phy_ref_clk);
		clk_prepare_enable(dcss->pll_src_clk);
	}

	clk_prepare_enable(dcss->axi_clk);
	clk_prepare_enable(dcss->apb_clk);
	clk_prepare_enable(dcss->rtrm_clk);
	clk_prepare_enable(dcss->dtrc_clk);
	clk_prepare_enable(dcss->pix_clk);
}

static void dcss_clocks_disable(struct dcss_dev *dcss)
{
	clk_disable_unprepare(dcss->pix_clk);
	clk_disable_unprepare(dcss->dtrc_clk);
	clk_disable_unprepare(dcss->rtrm_clk);
	clk_disable_unprepare(dcss->apb_clk);
	clk_disable_unprepare(dcss->axi_clk);

	if (dcss->hdmi_output) {
		clk_disable_unprepare(dcss->pll_src_clk);
		clk_disable_unprepare(dcss->pll_phy_ref_clk);
	}
}

static void dcss_disable_dtg_and_ss_cb(void *data)
{
	struct dcss_dev *dcss = data;

	dcss->disable_callback = NULL;

	dcss_ss_shutoff(dcss->ss);
	dcss_dtg_shutoff(dcss->dtg);

	complete(&dcss->disable_completion);
}

void dcss_disable_dtg_and_ss(struct dcss_dev *dcss)
{
	dcss->disable_callback = dcss_disable_dtg_and_ss_cb;
}

void dcss_enable_dtg_and_ss(struct dcss_dev *dcss)
{
	if (dcss->disable_callback)
		dcss->disable_callback = NULL;

	dcss_dtg_enable(dcss->dtg);
	dcss_ss_enable(dcss->ss);
}

static int dcss_submodules_init(struct dcss_dev *dcss)
{
	int ret = 0;
	u32 base_addr = dcss->start_addr;
	const struct dcss_type_data *devtype = dcss->devtype;

	dcss_clocks_enable(dcss);

	ret = dcss_blkctl_init(dcss, base_addr + devtype->blkctl_ofs);
	if (ret)
		return ret;

	ret = dcss_ctxld_init(dcss, base_addr + devtype->ctxld_ofs);
	if (ret)
		goto ctxld_err;

	ret = dcss_dtg_init(dcss, base_addr + devtype->dtg_ofs);
	if (ret)
		goto dtg_err;

	ret = dcss_ss_init(dcss, base_addr + devtype->ss_ofs);
	if (ret)
		goto ss_err;

	ret = dcss_dtrc_init(dcss, base_addr + devtype->dtrc_ofs);
	if (ret)
		goto dtrc_err;

	ret = dcss_dpr_init(dcss, base_addr + devtype->dpr_ofs);
	if (ret)
		goto dpr_err;

	ret = dcss_wrscl_init(dcss, base_addr + devtype->wrscl_ofs);
	if (ret)
		goto wrscl_err;

	ret = dcss_rdsrc_init(dcss, base_addr + devtype->rdsrc_ofs);
	if (ret)
		goto rdsrc_err;

	ret = dcss_scaler_init(dcss, base_addr + devtype->scaler_ofs);
	if (ret)
		goto scaler_err;

	ret = dcss_dec400d_init(dcss, base_addr + devtype->dec400d_ofs);
	if (ret)
		goto dec400d_err;

	ret = dcss_hdr10_init(dcss, base_addr + devtype->hdr10_ofs);
	if (ret)
		goto hdr10_err;

	dcss_clocks_disable(dcss);

	return 0;

hdr10_err:
	dcss_dec400d_exit(dcss->dec400d);

dec400d_err:
	dcss_scaler_exit(dcss->scaler);

scaler_err:
	dcss_rdsrc_exit(dcss->rdsrc);

rdsrc_err:
	dcss_wrscl_exit(dcss->wrscl);

wrscl_err:
	dcss_dpr_exit(dcss->dpr);

dpr_err:
	dcss_dtrc_exit(dcss->dtrc);

dtrc_err:
	dcss_ss_exit(dcss->ss);

ss_err:
	dcss_dtg_exit(dcss->dtg);

dtg_err:
	dcss_ctxld_exit(dcss->ctxld);

ctxld_err:
	dcss_blkctl_exit(dcss->blkctl);

	dcss_clocks_disable(dcss);

	return ret;
}

static void dcss_submodules_stop(struct dcss_dev *dcss)
{
	dcss_clocks_enable(dcss);
	dcss_hdr10_exit(dcss->hdr10);
	dcss_dec400d_exit(dcss->dec400d);
	dcss_scaler_exit(dcss->scaler);
	dcss_rdsrc_exit(dcss->rdsrc);
	dcss_wrscl_exit(dcss->wrscl);
	dcss_dpr_exit(dcss->dpr);
	dcss_dtrc_exit(dcss->dtrc);
	dcss_ss_exit(dcss->ss);
	dcss_dtg_exit(dcss->dtg);
	dcss_ctxld_exit(dcss->ctxld);
	dcss_blkctl_exit(dcss->blkctl);
	dcss_clocks_disable(dcss);
}

static int dcss_clks_init(struct dcss_dev *dcss)
{
	int i;
	struct {
		const char *id;
		struct clk **clk;
		bool required;
	} clks[] = {
		{"apb",   &dcss->apb_clk, true},
		{"axi",   &dcss->axi_clk, true},
		{"pix",   &dcss->pix_clk, true},
		{"rtrm",  &dcss->rtrm_clk, true},
		{"dtrc",  &dcss->dtrc_clk, true},
		{"pll_src",  &dcss->pll_src_clk, dcss->hdmi_output},
		{"pll_phy_ref",  &dcss->pll_phy_ref_clk, dcss->hdmi_output},
	};

	for (i = 0; i < ARRAY_SIZE(clks); i++) {
		*clks[i].clk = devm_clk_get(dcss->dev, clks[i].id);
		if (IS_ERR(*clks[i].clk) && clks[i].required) {
			dev_err(dcss->dev, "failed to get %s clock\n",
				clks[i].id);
			return PTR_ERR(*clks[i].clk);
		}
	}

	return 0;
}

static void dcss_clks_release(struct dcss_dev *dcss)
{
	devm_clk_put(dcss->dev, dcss->dtrc_clk);
	devm_clk_put(dcss->dev, dcss->rtrm_clk);
	devm_clk_put(dcss->dev, dcss->pix_clk);
	devm_clk_put(dcss->dev, dcss->axi_clk);
	devm_clk_put(dcss->dev, dcss->apb_clk);
}

struct dcss_dev *dcss_dev_create(struct device *dev, bool hdmi_output)
{
	struct platform_device *pdev = to_platform_device(dev);
	int ret;
	struct resource *res;
	struct dcss_dev *dcss;
	const struct dcss_type_data *devtype;
	struct device_node *np = dev->of_node;
	struct device_node *sp;
	struct platform_device * pd;

	devtype = of_device_get_match_data(dev);
	if (!devtype) {
		dev_err(dev, "no device match found\n");
		return ERR_PTR(-ENODEV);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "cannot get memory resource\n");
		return ERR_PTR(-EINVAL);
	}

	dcss = kzalloc(sizeof(*dcss), GFP_KERNEL);
	if (!dcss)
		return ERR_PTR(-ENOMEM);

	dcss->dev = dev;
	dcss->devtype = devtype;
	dcss->hdmi_output = hdmi_output;

	ret = dcss_clks_init(dcss);
	if (ret) {
		dev_err(dev, "clocks initialization failed\n");
		goto err;
	}

	dcss->of_port = of_graph_get_port_by_id(dev->of_node, 0);
	if (!dcss->of_port) {
		dev_err(dev, "no port@0 node in %s\n", dev->of_node->full_name);
		ret = -ENODEV;
		goto clks_err;
	}

	dcss->start_addr = res->start;

	ret = of_reserved_mem_device_init_by_idx(&pdev->dev, np, 0);
	if (ret) {
		dev_err(&pdev->dev,"dcss: dma memory initial failed\n");
	}

  	dcss->trusty_dev = NULL;
	if (of_find_property(np, "trusty", NULL)) {
		sp = of_find_node_by_name(NULL, "trusty");
		if (sp != NULL) {
			pd = of_find_device_by_node(sp);
			if (pd != NULL) {
				if (!trusty_fast_call32(&(pd->dev), SMC_IMX_DCSS_ECHO, 0, 0, 0)) {
					dcss->trusty_dev = &(pd->dev);
					dev_err(&pdev->dev, "dcss: get trusty_dev node, use Trusty mode.\n");
				} else
					dev_err(&pdev->dev, "dcss: failed to get response of echo. Use normal mode.\n");
			} else
				dev_err(&pdev->dev, "dcss: failed to get trusty_dev node.\n");
		} else
			dev_err(&pdev->dev, "dcss: failed to find trusty node. Use normal mode.\n");
	}

	ret = dcss_submodules_init(dcss);
	if (ret) {
		of_node_put(dcss->of_port);
		dev_err(dev, "submodules initialization failed\n");
		goto clks_err;
	}

	init_completion(&dcss->disable_completion);

	pm_runtime_set_autosuspend_delay(dev, 100);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_suspended(dev);
	pm_runtime_allow(dev);
	pm_runtime_enable(dev);

	return dcss;

clks_err:
	dcss_clks_release(dcss);

err:
	kfree(dcss);

	return ERR_PTR(ret);
}

void dcss_dev_destroy(struct dcss_dev *dcss)
{
	if (!pm_runtime_suspended(dcss->dev)) {
		dcss_ctxld_suspend(dcss->ctxld);
		dcss_clocks_disable(dcss);
	}

	of_node_put(dcss->of_port);

	pm_runtime_disable(dcss->dev);

	dcss_submodules_stop(dcss);

	dcss_clks_release(dcss);

	kfree(dcss);
}

#ifdef CONFIG_PM_SLEEP
int dcss_dev_suspend(struct device *dev)
{
	struct dcss_dev *dcss = dcss_drv_dev_to_dcss(dev);
	struct drm_device *ddev = dcss_drv_dev_to_drm(dev);
	struct dcss_kms_dev *kms = container_of(ddev, struct dcss_kms_dev, base);
	int ret;

	if (!dcss)
		return 0;

	if (!dcss_drv_is_componentized(dev))
		drm_bridge_connector_disable_hpd(kms->connector);

	drm_mode_config_helper_suspend(ddev);

	if (pm_runtime_suspended(dev))
		return 0;

	ret = dcss_ctxld_suspend(dcss->ctxld);
	if (ret)
		return ret;

	dcss_clocks_disable(dcss);

	release_bus_freq(BUS_FREQ_HIGH);

	return 0;
}

int dcss_dev_resume(struct device *dev)
{
	struct dcss_dev *dcss = dcss_drv_dev_to_dcss(dev);
	struct drm_device *ddev = dcss_drv_dev_to_drm(dev);
	struct dcss_kms_dev *kms = container_of(ddev, struct dcss_kms_dev, base);

	if (!dcss)
		return 0;

	if (pm_runtime_suspended(dev)) {
		drm_mode_config_helper_resume(ddev);
		return 0;
	}

	request_bus_freq(BUS_FREQ_HIGH);

	dcss_clocks_enable(dcss);

	dcss_blkctl_cfg(dcss->blkctl);

	dcss_ctxld_resume(dcss->ctxld);

	drm_mode_config_helper_resume(ddev);

	if (!dcss_drv_is_componentized(dev))
		drm_bridge_connector_enable_hpd(kms->connector);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_PM
int dcss_dev_runtime_suspend(struct device *dev)
{
	struct dcss_dev *dcss = dcss_drv_dev_to_dcss(dev);
	int ret;

	if (!dcss)
		return 0;

	ret = dcss_ctxld_suspend(dcss->ctxld);
	if (ret)
		return ret;

	dcss_clocks_disable(dcss);

	release_bus_freq(BUS_FREQ_HIGH);

	return 0;
}

int dcss_dev_runtime_resume(struct device *dev)
{
	struct dcss_dev *dcss = dcss_drv_dev_to_dcss(dev);

	if (!dcss)
		return 0;

	request_bus_freq(BUS_FREQ_HIGH);

	dcss_clocks_enable(dcss);

	dcss_blkctl_cfg(dcss->blkctl);

	dcss_ctxld_resume(dcss->ctxld);

	return 0;
}
#endif /* CONFIG_PM */
