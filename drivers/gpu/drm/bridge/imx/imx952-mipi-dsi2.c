// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright 2026 NXP
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/math.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mux/consumer.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-mipi-dphy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <drm/bridge/dw_mipi_dsi2.h>
#include <drm/drm_bridge.h>
#include <drm/drm_edid.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>

#define DSI_HOST_CONFIGURATION		0x14
#define  PIXEL_LINK_FORMAT_MASK		GENMASK(2, 0)
#define  SHUTDOWN			BIT(4)
#define  COLORMODE			BIT(5)

#define IMX952_DSI_ENDPOINT_PL0		0
#define IMX952_DSI_ENDPOINT_PL1		1

#define PIXEL_LINK_STREAMS		2

#define ESC_CLK_RATE_HZ			18562500

enum dsi_pixel_link_format {
	RGB_24BIT,
	RGB_30BIT,
	RGB_18BIT,
	RGB_16BIT,
	YCBCR_20BIT_422,
	YCBCR_16BIT_422,
};

struct imx952_dsi2 {
	struct device *dev;
	struct regmap *dsi_csr;
	struct clk *clk_pixel;
	struct clk *clk_phy_pll;
	struct phy *phy;
	struct mux_control *mux;
	bool use_pl0;
	enum mipi_dsi_pixel_format format;
	struct dw_mipi_dsi2 *dmd;
	struct dw_mipi_dsi2_plat_data pdata;
	union phy_configure_opts phy_cfg;
	unsigned long target_pclk_rate;
};

static int imx952_dsi2_get_clk(struct imx952_dsi2 *dsi)
{
	struct device *dev = dsi->dev;

	dsi->clk_pixel = devm_clk_get(dev, "pix");
	if (IS_ERR(dsi->clk_pixel))
		return dev_err_probe(dev, PTR_ERR(dsi->clk_pixel),
				     "failed to get pixel clk\n");

	dsi->clk_phy_pll = devm_clk_get(dev, "phy_pll");
	if (IS_ERR(dsi->clk_phy_pll))
		return dev_err_probe(dev, PTR_ERR(dsi->clk_phy_pll),
				     "failed to get PHY PLL clk\n");

	return 0;
}

static int imx952_dsi2_get_regmap(struct imx952_dsi2 *dsi)
{
	struct device *dev = dsi->dev;
	struct device_node *np = dev->of_node;

	dsi->dsi_csr = syscon_regmap_lookup_by_phandle(np,
						       "nxp,display-dsi-csr");
	if (IS_ERR(dsi->dsi_csr))
		return dev_err_probe(dev, PTR_ERR(dsi->dsi_csr),
				     "failed to get DSI CSR\n");

	return 0;
}

static int imx952_dsi2_get_phy(struct imx952_dsi2 *dsi)
{
	struct device *dev = dsi->dev;

	dsi->phy = devm_phy_get(dev, "dphy");
	if (IS_ERR(dsi->phy))
		return dev_err_probe(dev, PTR_ERR(dsi->phy),
				     "failed to get DPHY\n");

	return 0;
}

static int imx952_dsi2_get_mux(struct imx952_dsi2 *dsi)
{
	dsi->mux = devm_mux_control_get(dsi->dev, NULL);
	if (IS_ERR(dsi->mux))
		return dev_err_probe(dsi->dev, PTR_ERR(dsi->mux),
				     "failed to get mux\n");

	return 0;
}

static int imx952_dsi2_select_input(struct imx952_dsi2 *dsi)
{
	struct device_node *remote_ldb_ch0, *remote_ldb_ch1 = NULL;
	struct device_node *remote_pi0, *remote_pi1 = NULL;
	struct device_node *remote0, *remote1 = NULL;
	struct device *dev = dsi->dev;
	int ret = 0;
	u32 port;

	/* pixel link0 */
	remote0 = of_graph_get_remote_node(dev->of_node, 0,
					   IMX952_DSI_ENDPOINT_PL0);
	/* pixel interleaver channel0 */
	remote_pi0 = of_graph_get_remote_node(remote0,
					      IMX952_DSI_ENDPOINT_PL0, 0);
	/* ldb channel0 */
	port = IMX952_DSI_ENDPOINT_PL0 + PIXEL_LINK_STREAMS;
	remote_ldb_ch0 = of_graph_get_remote_node(remote0, port, 1);
	if (remote_pi0 && !remote_ldb_ch0) {
		dsi->use_pl0 = true;
	} else {
		/* pixel link1 */
		remote1 = of_graph_get_remote_node(dev->of_node, 0,
						   IMX952_DSI_ENDPOINT_PL1);
		/* pixel interleaver channel1 */
		remote_pi1 = of_graph_get_remote_node(remote1,
						      IMX952_DSI_ENDPOINT_PL1, 0);
		/* ldb channel1 */
		port = IMX952_DSI_ENDPOINT_PL1 + PIXEL_LINK_STREAMS;
		remote_ldb_ch1 = of_graph_get_remote_node(remote1, port, 1);
		if (!remote_pi1 || remote_ldb_ch1) {
			dev_err(dev, "No valid input endpoint found\n");
			ret = -EINVAL;
			goto out;
		}

		dsi->use_pl0 = false;
	}

	dev_info(dev, "Using Pixel Link%d as input source\n",
		 dsi->use_pl0 ? 0 : 1);

out:
	of_node_put(remote_ldb_ch1);
	of_node_put(remote_pi1);
	of_node_put(remote1);
	of_node_put(remote_ldb_ch0);
	of_node_put(remote_pi0);
	of_node_put(remote0);

	return ret;
}

static inline unsigned long data_rate_to_fout(unsigned long data_rate)
{
	/* Fout is half of data rate */
	return data_rate / 2;
}

static int imx952_dsi2_phy_init(void *priv_data)
{
	struct imx952_dsi2 *dsi = priv_data;
	int bpp, ret;

	ret = mux_control_try_select(dsi->mux, !dsi->use_pl0);
	if (ret < 0) {
		dev_err(dsi->dev, "failed to select input: %d\n", ret);
		return ret;
	}

	bpp = mipi_dsi_pixel_format_to_bpp(dsi->format);
	if (bpp < 0) {
		dev_err(dsi->dev, "failed to get dsi format bpp\n");
		return bpp;
	}

	switch (bpp) {
	case 24:
		regmap_write(dsi->dsi_csr, DSI_HOST_CONFIGURATION, RGB_24BIT);
		break;
	case 18:
		regmap_write(dsi->dsi_csr, DSI_HOST_CONFIGURATION, RGB_18BIT);
		break;
	case 16:
		regmap_write(dsi->dsi_csr, DSI_HOST_CONFIGURATION, RGB_16BIT);
		break;
	default:
		dev_err(dsi->dev, "invalid dsi format bpp %d\n", bpp);
		return -EINVAL;
	}

	ret = phy_set_mode(dsi->phy, PHY_MODE_MIPI_DPHY);
	if (ret) {
		dev_err(dsi->dev, "failed to set phy mode: %d\n", ret);
		return ret;
	}

	ret = phy_init(dsi->phy);
	if (ret < 0) {
		dev_err(dsi->dev, "failed to init phy: %d\n", ret);
		return ret;
	}

	ret = phy_configure(dsi->phy, &dsi->phy_cfg);
	if (ret < 0) {
		dev_err(dsi->dev, "failed to configure phy: %d\n", ret);
		goto uninit_phy;
	}

	ret = phy_power_on(dsi->phy);
	if (ret < 0) {
		dev_err(dsi->dev, "failed to power on phy: %d\n", ret);
		goto uninit_phy;
	}

	return 0;

uninit_phy:
	phy_exit(dsi->phy);
	return ret;
}

static void imx952_dsi2_phy_power_off(void *priv_data)
{
	struct imx952_dsi2 *dsi = priv_data;
	int ret;

	ret = phy_power_off(dsi->phy);
	if (ret < 0)
		dev_err(dsi->dev, "failed to power off phy: %d\n", ret);

	ret = phy_exit(dsi->phy);
	if (ret < 0)
		dev_err(dsi->dev, "failed to exit phy: %d\n", ret);

	ret = mux_control_deselect(dsi->mux);
	if (ret < 0)
		dev_err(dsi->dev, "failed to deselect input: %d\n", ret);
}

static void imx952_dsi2_phy_get_iface(void *priv_data,
				      struct dw_mipi_dsi2_phy_iface *iface)
{
	/* PPI width is fixed to 8 bits in DCPHY */
	iface->ppi_width = 8;
	iface->phy_type = DW_MIPI_DSI2_DPHY;
}

static int
imx952_dsi2_get_phy_configure_opts(struct imx952_dsi2 *dsi,
				   unsigned long pclk_rate,
				   union phy_configure_opts *phy_cfg,
				   unsigned long mode_flags,
				   u32 lanes, u32 format)
{
	struct phy_configure_opts_mipi_dphy *dphy_opts = &phy_cfg->mipi_dphy;
	struct device *dev = dsi->dev;
	unsigned long target_pixel_clock;
	unsigned long fout;
	int bpp;
	int ret;

	bpp = mipi_dsi_pixel_format_to_bpp(format);
	if (bpp < 0) {
		dev_dbg(dev, "failed to get bpp for pixel format %d\n", format);
		return -EINVAL;
	}

	target_pixel_clock = pclk_rate;
	if (mode_flags & MIPI_DSI_MODE_VIDEO_BURST)
		target_pixel_clock = target_pixel_clock * 10 / 9;

	ret = phy_mipi_dphy_get_default_config(target_pixel_clock, bpp,
					       lanes, dphy_opts);
	if (ret < 0) {
		dev_dbg(dev, "failed to get default phy cfg %d\n", ret);
		return ret;
	}

	ret = phy_validate(dsi->phy, PHY_MODE_MIPI_DPHY, 0, phy_cfg);
	if (ret < 0) {
		dev_dbg(dev, "failed to validate phy cfg %d\n", ret);
		return -EINVAL;
	}

	fout = data_rate_to_fout(dphy_opts->hs_clk_rate);
	if (fout != clk_round_rate(dsi->clk_phy_pll, fout)) {
		dev_dbg(dev, "failed to round phy PLL clk rate %luHz\n", fout);
		return -EINVAL;
	}

	return 0;
}

static enum drm_mode_status
imx952_dsi2_validate_phy(struct imx952_dsi2 *dsi, unsigned long pclk_rate,
			 unsigned long mode_flags, u32 lanes, u32 format)
{
	union phy_configure_opts phy_cfg;
	struct device *dev = dsi->dev;
	int ret;

	ret = imx952_dsi2_get_phy_configure_opts(dsi, pclk_rate, &phy_cfg,
						 mode_flags, lanes, format);
	if (ret < 0) {
		dev_dbg(dev, "failed to get phy cfg opts %d\n", ret);
		return MODE_ERROR;
	}

	return MODE_OK;
}

static enum drm_mode_status
imx952_dsi2_mode_valid(void *priv_data, const struct drm_display_mode *mode,
		       unsigned long mode_flags, u32 lanes, u32 format)
{
	unsigned long pclk_rate = mode->clock * 1000;
	unsigned long target_pclk_rate = pclk_rate;
	struct imx952_dsi2 *dsi = priv_data;
	struct drm_bridge *bridge, *iter;
	struct device *dev = dsi->dev;
	struct drm_encoder *encoder;
	enum drm_mode_status ret;
	u8 vic;

	bridge = dw_mipi_dsi2_get_bridge(dsi->dmd);
	encoder = bridge->encoder;

	list_for_each_entry_reverse(iter, &encoder->bridge_chain, chain_node) {
		if (!(iter->ops & DRM_BRIDGE_OP_DETECT) ||
		    !(iter->ops & DRM_BRIDGE_OP_EDID))
			continue;

		/*
		 * Since clk_round_rate() returns unreasonable rate for
		 * dsi->clk_pixel, we have to validate mode against magic mode
		 * clock rates.
		 */
		if (mode->clock != 297000 && mode->clock != 148500 && mode->clock != 74250)
			return MODE_NOCLOCK;

		/* Allow VIC 94 & 95(3840x2160@25/30) for 4K */
		vic = drm_match_cea_mode(mode);
		if (mode->clock == 297000 && vic != 94 && vic != 95)
			return MODE_BAD;

		/* Allow +/-0.5% pixel clock rate deviation */
		target_pclk_rate = clk_round_rate(dsi->clk_pixel, pclk_rate);
		if (target_pclk_rate < pclk_rate * 995 / 1000 ||
		    target_pclk_rate > pclk_rate * 1005 / 1000) {
			dev_dbg(dev, "failed to round clock for mode " DRM_MODE_FMT "\n",
				DRM_MODE_ARG(mode));
			return MODE_NOCLOCK;
		}

		break;
	}

	ret = imx952_dsi2_validate_phy(dsi, target_pclk_rate, mode_flags, lanes,
				       format);
	if (ret != MODE_OK) {
		dev_dbg(dev, "failed to validate phy for mode " DRM_MODE_FMT "\n",
			DRM_MODE_ARG(mode));
		return ret;
	}

	dsi->target_pclk_rate = target_pclk_rate;

	return MODE_OK;
}

static bool imx952_dsi2_mode_fixup(void *priv_data,
				   const struct drm_display_mode *mode,
				   struct drm_display_mode *adjusted_mode)
{
	struct imx952_dsi2 *dsi = priv_data;
	unsigned long pixel_clock_rate;
	unsigned long rounded_rate;

	pixel_clock_rate = mode->clock * 1000;
	rounded_rate = clk_round_rate(dsi->clk_pixel, pixel_clock_rate);

	memcpy(adjusted_mode, mode, sizeof(*mode));
	adjusted_mode->clock = rounded_rate / 1000;

	dev_dbg(dsi->dev, "adj clock %d for mode " DRM_MODE_FMT "\n",
		adjusted_mode->clock, DRM_MODE_ARG(mode));

	/* pixel link always generates active low HSYNC and VSYNC */
	adjusted_mode->flags &= ~(DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC);
	adjusted_mode->flags |= DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC;

	return true;
}

static int
imx952_dsi2_phy_get_lane_mbps(void *priv_data,
			      const struct drm_display_mode *mode,
			      unsigned long mode_flags, u32 lanes, u32 format,
			      unsigned int *lane_mbps)
{
	struct imx952_dsi2 *dsi = priv_data;
	union phy_configure_opts phy_cfg;
	struct device *dev = dsi->dev;
	int ret;

	ret = imx952_dsi2_get_phy_configure_opts(dsi, dsi->target_pclk_rate,
						 &phy_cfg, mode_flags, lanes,
						 format);
	if (ret < 0) {
		dev_dbg(dev, "failed to get phy cfg opts %d\n", ret);
		return ret;
	}

	*lane_mbps = DIV_ROUND_UP(phy_cfg.mipi_dphy.hs_clk_rate, USEC_PER_SEC);

	memcpy(&dsi->phy_cfg, &phy_cfg, sizeof(phy_cfg));

	dev_dbg(dev, "get lane_mbps %u for mode " DRM_MODE_FMT "\n",
		*lane_mbps, DRM_MODE_ARG(mode));

	return 0;
}

static int imx952_dsi2_phy_get_timing(void *priv_data, unsigned int lane_mbps,
				      struct dw_mipi_dsi2_phy_timing *timing)
{
	struct imx952_dsi2 *dsi = priv_data;
	struct phy_configure_opts_mipi_dphy *cfg = &dsi->phy_cfg.mipi_dphy;
	unsigned long long tmp;
	unsigned long long hstx_clk;

	hstx_clk = DIV_ROUND_CLOSEST_ULL(lane_mbps, 8);

	/* PHY_LP2HS_TIME = (TLPX + THS-PREPARE + THS-ZERO) / Tphy_hstx_clk */
	tmp = cfg->lpx + cfg->hs_prepare + cfg->hs_zero;
	tmp = DIV_ROUND_CLOSEST_ULL((tmp * hstx_clk) << 16, USEC_PER_SEC);
	timing->data_lp2hs = tmp;

	/* PHY_HS2LP_TIME = (THS-TRAIL + THS-EXIT) / Tphy_hstx_clk */
	tmp = cfg->hs_trail + cfg->hs_exit;

	/* empirical fixup for 4Kp30/25 with 4 data lanes */
	if (lane_mbps == 1782 && cfg->lanes == 4)
		tmp *= 20;

	tmp = DIV_ROUND_CLOSEST_ULL((tmp * hstx_clk) << 16, USEC_PER_SEC);
	timing->data_hs2lp = tmp;

	return 0;
}

static int
imx952_dsi2_phy_get_esc_clk_rate(void *priv_data, unsigned long *esc_clk_rate)
{
	*esc_clk_rate = ESC_CLK_RATE_HZ;

	return 0;
}

static const struct dw_mipi_dsi2_phy_ops imx952_dsi2_phy_ops = {
	.init = imx952_dsi2_phy_init,
	.power_off = imx952_dsi2_phy_power_off,
	.get_interface = imx952_dsi2_phy_get_iface,
	.get_lane_mbps = imx952_dsi2_phy_get_lane_mbps,
	.get_timing = imx952_dsi2_phy_get_timing,
	.get_esc_clk_rate = imx952_dsi2_phy_get_esc_clk_rate,
};

static int imx952_dsi2_imx_host_attach(void *priv_data,
				       struct mipi_dsi_device *device)
{
	struct imx952_dsi2 *dsi = priv_data;

	dsi->format = device->format;

	return 0;
}

static const struct dw_mipi_dsi2_host_ops imx952_dsi2_host_ops = {
	.attach = imx952_dsi2_imx_host_attach,
};

static int imx952_dsi2_parse_dt(struct imx952_dsi2 *dsi)
{
	int ret;

	ret = imx952_dsi2_get_clk(dsi);
	if (ret)
		return ret;

	ret = imx952_dsi2_get_regmap(dsi);
	if (ret)
		return ret;

	ret = imx952_dsi2_get_phy(dsi);
	if (ret)
		return ret;

	ret = imx952_dsi2_get_mux(dsi);
	if (ret)
		return ret;

	ret = imx952_dsi2_select_input(dsi);
	if (ret)
		return ret;

	return 0;
}

static int imx952_dsi2_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imx952_dsi2 *dsi;
	int ret;

	dsi = devm_kzalloc(dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return -ENOMEM;

	dsi->dev = dev;
	platform_set_drvdata(pdev, dsi);

	ret = imx952_dsi2_parse_dt(dsi);
	if (ret)
		return ret;

	dsi->pdata.max_data_lanes = 4;
	dsi->pdata.ipi_lanes = 1;
	dsi->pdata.ipi_fifo_depth = 960;
	dsi->pdata.ipi_mapping = DW_MIPI_DSI2_IPI_MAPPING_DPI_CONFIG1;
	dsi->pdata.cri_cmd_wr_pld_fifo_depth = 32;
	dsi->pdata.cri_cmd_rd_pld_fifo_depth = 128;
	dsi->pdata.mode_valid = imx952_dsi2_mode_valid;
	dsi->pdata.mode_fixup = imx952_dsi2_mode_fixup;
	dsi->pdata.phy_ops = &imx952_dsi2_phy_ops;
	dsi->pdata.host_ops = &imx952_dsi2_host_ops;
	dsi->pdata.priv_data = dsi;

	dsi->dmd = dw_mipi_dsi2_probe(pdev, &dsi->pdata);
	if (IS_ERR(dsi->dmd))
		return dev_err_probe(dev, PTR_ERR(dsi->dmd),
				     "failed to probe dw_mipi_dsi\n");

	return 0;
}

static void imx952_dsi2_remove(struct platform_device *pdev)
{
	struct imx952_dsi2 *dsi = platform_get_drvdata(pdev);

	dw_mipi_dsi2_remove(dsi->dmd);
}

static const struct of_device_id imx952_dsi2_dt_ids[] = {
	{ .compatible = "nxp,imx952-mipi-dsi2", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx952_dsi2_dt_ids);

static struct platform_driver imx952_dsi2_driver = {
	.probe	= imx952_dsi2_probe,
	.remove	= imx952_dsi2_remove,
	.driver	= {
		.of_match_table = imx952_dsi2_dt_ids,
		.name = "imx952-mipi-dsi2",
	},
};

module_platform_driver(imx952_dsi2_driver);

MODULE_DESCRIPTION("i.MX952 MIPI DSI2 driver");
MODULE_AUTHOR("NXP Semiconductor");
MODULE_LICENSE("GPL");
