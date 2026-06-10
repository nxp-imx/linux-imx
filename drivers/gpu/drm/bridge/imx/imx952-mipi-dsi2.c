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

#define MAX_ESC_CLK_RATE_HZ		20000000

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
	unsigned long esc_clk_rate;
	unsigned long mode_flags;
	int phy_submode;
	bool hs2lp_lp2hs_quirk;
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

	ret = phy_set_mode_ext(dsi->phy, PHY_MODE_MIPI_DPHY, dsi->phy_submode);
	if (ret) {
		dev_err(dsi->dev, "failed to set phy mode: %d\n", ret);
		return ret;
	}

	ret = phy_init(dsi->phy);
	if (ret < 0) {
		dev_err(dsi->dev, "failed to init phy: %d\n", ret);
		return ret;
	}

	return 0;
}

static void imx952_dsi2_phy_power_on(void *priv_data)
{
	struct imx952_dsi2 *dsi = priv_data;
	union phy_configure_opts *phy_cfg = &dsi->phy_cfg;
	struct phy_configure_opts_mipi_dphy *dphy_opts = &phy_cfg->mipi_dphy;
	int ret;

	if (WARN_ON(dsi->esc_clk_rate == 0))
		return;

	dphy_opts->lpx = PSEC_PER_SEC / dsi->esc_clk_rate;

	dev_dbg(dsi->dev, "PHY lpx = %ups\n", dphy_opts->lpx);

	ret = phy_configure(dsi->phy, phy_cfg);
	if (ret < 0) {
		dev_err(dsi->dev, "failed to configure phy: %d\n", ret);
		return;
	}

	ret = phy_power_on(dsi->phy);
	if (ret < 0)
		dev_err(dsi->dev, "failed to power on phy: %d\n", ret);
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

static const u8 adv7535_vics[] = {
4, 16, 19, 31, 32, 33, 34, 68, 69, 72, 73, 74, 75, 76, };
static const bool adv7535_vic_quirks[] = {
	true, /* 4 */	true, /* 16 */	false,/* 19 */	true, /* 31 */
	false,/* 32 */	false,/* 33 */	true, /* 34 */	false,/* 68 */
	true, /* 69 */	false,/* 72 */	false,/* 73 */	true, /* 74 */
	true, /* 75 */	true, /* 76 */
};

struct dmt_mini_mode {
	int hdisplay;
	int vdisplay;
	int vrefresh;
	bool rb;
};

static const struct dmt_mini_mode adv7535_valid_dmt_mini_modes[] = {
	/* 0x09 - 800x600@60Hz */
	{ 800, 600, 60, false },
	/* 0x10 - 1024x768@60Hz */
	{ 1024, 768, 60, false },
	/* 0x1b - 1280x800@60Hz RB */
	{ 1280, 800, 60, true },
	/* 0x1c - 1280x800@60Hz */
	{ 1280, 800, 60, false },
	/* 0x52 - 1920x1080@60Hz */
	{ 1920, 1080, 60, false },
};

static const bool adv7535_dmt_quirks[] = { true, false, true, true, true, };
/* PHY multiplication factor x10 */
static const int adv7535_dmt_phy_submodes[] = { 55, 55, 60, 55, 55, };

static bool is_adv7535_valid_cea_mode(const struct drm_display_mode *mode, int *i)
{
	u8 vic;
	int j;

	vic = drm_match_cea_mode(mode);
	if (vic > 0) {
		for (j = 0; j < ARRAY_SIZE(adv7535_vics); j++) {
			if (vic == adv7535_vics[j]) {
				*i = j;
				return true;
			}
		}
	}

	return false;
}

static bool check_vendor_dmt_mode(struct drm_device *drm,
				  const struct drm_display_mode *mode,
				  const struct dmt_mini_mode *dmt_mini_modes,
				  int num_dmt_mini_modes, int *i)
{
	const struct dmt_mini_mode *mini_mode;
	struct drm_display_mode *dmt_mode;
	int j;

	for (j = 0; j < num_dmt_mini_modes; j++) {
		mini_mode = &dmt_mini_modes[j];

		dmt_mode = drm_mode_find_dmt(drm,
					     mini_mode->hdisplay,
					     mini_mode->vdisplay,
					     mini_mode->vrefresh,
					     mini_mode->rb);
		if (WARN_ON(!dmt_mode))
			continue;

		if (drm_mode_equal(dmt_mode, mode)) {
			drm_mode_destroy(drm, dmt_mode);
			if (i)
				*i = j;
			return true;
		}

		drm_mode_destroy(drm, dmt_mode);
	}

	return false;
}

static bool is_adv7535_valid_dmt_mode(struct drm_device *drm,
				      const struct drm_display_mode *mode,
				      int *i)
{
	return check_vendor_dmt_mode(drm, mode, adv7535_valid_dmt_mini_modes,
				     ARRAY_SIZE(adv7535_valid_dmt_mini_modes),
				     i);
}

static const u8 dsi_serdes_vics[] = { 1, 2, 3, 16, 17, 18, 76, };

static const struct dmt_mini_mode dsi_serdes_valid_dmt_mini_modes[] = {
	/* 0x04 - 640x480@60Hz */
	{ 640, 480, 60, false },
	/* 0x09 - 800x600@60Hz */
	{ 800, 600, 60, false },
	/* 0x0b - 800x600@75Hz */
	{ 800, 600, 75, false },
	/* 0x10 - 1024x768@60Hz */
	{ 1024, 768, 60, false },
	/* 0x1b - 1280x800@60Hz RB */
	{ 1280, 800, 60, true },
};

static bool is_dsi_serdes_valid_cea_mode(const struct drm_display_mode *mode)
{
	u8 vic;
	int i;

	vic = drm_match_cea_mode(mode);
	if (vic > 0) {
		for (i = 0; i < ARRAY_SIZE(dsi_serdes_vics); i++) {
			if (vic == dsi_serdes_vics[i])
				return true;
		}
	}

	return false;
}

static bool is_dsi_serdes_valid_dmt_mode(struct drm_device *drm,
					 const struct drm_display_mode *mode)
{
	return check_vendor_dmt_mode(drm, mode, dsi_serdes_valid_dmt_mini_modes,
				     ARRAY_SIZE(dsi_serdes_valid_dmt_mini_modes),
				     NULL);
}

static const struct dmt_mini_mode lt9611uxc_invalid_dmt_mini_modes[] = {
	/* 0x1b - 1280x800@60Hz RB */
	{ 1280, 800, 60, true },
	/* 0x1c - 1280x800@60Hz */
	{ 1280, 800, 60, false },
	/* 0x51 - 1366x768@60Hz */
	{ 1366, 768, 60, false },
};

static bool is_lt9611uxc_invalid_dmt_mode(struct drm_device *drm,
					  const struct drm_display_mode *mode)
{
	return check_vendor_dmt_mode(drm, mode, lt9611uxc_invalid_dmt_mini_modes,
				     ARRAY_SIZE(lt9611uxc_invalid_dmt_mini_modes),
				     NULL);
}

static int imx952_dsi2_get_target_pclk_rate(struct imx952_dsi2 *dsi,
					    const struct drm_display_mode *mode,
					    unsigned long *target_pclk_rate)
{
	unsigned long pclk_rate = mode->clock * 1000;
	struct drm_bridge *bridge, *iter;
	struct device *dev = dsi->dev;
	struct drm_encoder *encoder;

	*target_pclk_rate = pclk_rate;

	bridge = dw_mipi_dsi2_get_bridge(dsi->dmd);
	encoder = bridge->encoder;

	/*
	 * Get target_pclk_rate for cases where downstream bridges have
	 * DRM_BRIDGE_OP_DETECT and DRM_BRIDGE_OP_EDID flags.
	 */
	list_for_each_entry_reverse(iter, &encoder->bridge_chain, chain_node) {
		if (!(iter->ops & DRM_BRIDGE_OP_DETECT) ||
		    !(iter->ops & DRM_BRIDGE_OP_EDID))
			continue;

		/* Allow +/-0.5% pixel clock rate deviation */
		*target_pclk_rate = clk_round_rate(dsi->clk_pixel, pclk_rate);
		if (*target_pclk_rate < pclk_rate * 995 / 1000 ||
		    *target_pclk_rate > pclk_rate * 1005 / 1000) {
			dev_dbg(dev, "failed to round clock for mode " DRM_MODE_FMT "\n",
				DRM_MODE_ARG(mode));
			return -EINVAL;
		}
	}

	return 0;
}

static enum drm_mode_status
imx952_dsi2_mode_valid_downstream_bridge(struct imx952_dsi2 *dsi,
					 const struct drm_display_mode *mode,
					 int *phy_submode,
					 bool *hs2lp_lp2hs_quirk)
{
	struct drm_bridge *bridge, *iter;
	struct drm_encoder *encoder;

	/* PHY multiplication factor 1.1 */
	*phy_submode = 11;
	*hs2lp_lp2hs_quirk = false;

	bridge = dw_mipi_dsi2_get_bridge(dsi->dmd);
	encoder = bridge->encoder;

	list_for_each_entry_reverse(iter, &encoder->bridge_chain, chain_node) {
		if (!(iter->ops & DRM_BRIDGE_OP_DETECT) ||
		    !(iter->ops & DRM_BRIDGE_OP_EDID))
			continue;

		if (!iter->product)
			break;

		if (strcmp(iter->product, "ADV7535") == 0) {
			bool vic_match, dmt_match = false;
			int i;

			vic_match = is_adv7535_valid_cea_mode(mode, &i);

			if (vic_match) {
				*hs2lp_lp2hs_quirk = adv7535_vic_quirks[i];

				/* PHY multiplication factor 5.5 */
				*phy_submode = 55;
			} else {
				dmt_match = is_adv7535_valid_dmt_mode(encoder->dev, mode, &i);
				if (dmt_match) {
					*hs2lp_lp2hs_quirk = adv7535_dmt_quirks[i];
					*phy_submode = adv7535_dmt_phy_submodes[i];
				}
			}

			if (!vic_match && !dmt_match)
				return MODE_BAD;
		} else if (strcmp(iter->product, "IT6263") == 0) {
			bool vic_match, dmt_match = false;

			vic_match = is_dsi_serdes_valid_cea_mode(mode);

			if (!vic_match)
				dmt_match = is_dsi_serdes_valid_dmt_mode(encoder->dev, mode);

			if (!vic_match && !dmt_match)
				return MODE_BAD;
		} else if (strcmp(iter->product, "LT9611UXC") == 0) {
			u8 vic;

			/* Allow VIC 94 & 95(3840x2160@25/30) for 4K */
			vic = drm_match_cea_mode(mode);
			if (mode->clock == 297000 && vic != 94 && vic != 95)
				return MODE_BAD;

			if (is_lt9611uxc_invalid_dmt_mode(encoder->dev, mode))
				return MODE_BAD;
		}

		break;
	}

	return MODE_OK;
}

static enum drm_mode_status
imx952_dsi2_mode_valid(void *priv_data, const struct drm_display_mode *mode,
		       unsigned long mode_flags, u32 lanes, u32 format)
{
	struct imx952_dsi2 *dsi = priv_data;
	unsigned long target_pclk_rate;
	struct device *dev = dsi->dev;
	enum drm_mode_status ret;
	bool hs2lp_lp2hs_quirk;
	int phy_submode;
	int err;

	err = imx952_dsi2_get_target_pclk_rate(dsi, mode, &target_pclk_rate);
	if (err)
		return MODE_NOCLOCK;

	ret = imx952_dsi2_mode_valid_downstream_bridge(dsi, mode, &phy_submode,
						       &hs2lp_lp2hs_quirk);
	if (ret != MODE_OK) {
		dev_dbg(dev, "failed to validate downstream bridge for mode " DRM_MODE_FMT "\n",
			DRM_MODE_ARG(mode));
		return ret;
	}

	ret = imx952_dsi2_validate_phy(dsi, target_pclk_rate, mode_flags, lanes,
				       format);
	if (ret != MODE_OK) {
		dev_dbg(dev, "failed to validate phy for mode " DRM_MODE_FMT "\n",
			DRM_MODE_ARG(mode));
		return ret;
	}

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
	enum drm_mode_status mode_status;
	unsigned long target_pclk_rate;
	struct device *dev = dsi->dev;
	int ret;

	ret = imx952_dsi2_get_target_pclk_rate(dsi, mode, &target_pclk_rate);
	if (ret)
		return ret;

	mode_status = imx952_dsi2_mode_valid_downstream_bridge(dsi, mode,
							       &dsi->phy_submode,
							       &dsi->hs2lp_lp2hs_quirk);
	if (mode_status != MODE_OK) {
		dev_dbg(dev, "failed to validate downstream bridge for mode " DRM_MODE_FMT "\n",
			DRM_MODE_ARG(mode));
		return -EINVAL;
	}

	ret = imx952_dsi2_get_phy_configure_opts(dsi, target_pclk_rate,
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
	unsigned int lp2hs_m, lp2hs_b;
	unsigned int hs2lp_m, hs2lp_b;

	if (dsi->hs2lp_lp2hs_quirk) {
		timing->data_lp2hs = 0x10000;
		timing->data_hs2lp = 0x10000;
		dev_dbg(dsi->dev, "hs2lp_lp2hs_quirk\n");

		return 0;
	}

	/* PHY_LP2HS/HS2LP_TIME = DIV_ROUND_UP((lane_mbps * m), 100) + b */
	if (dsi->mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS) {
		lp2hs_m = 13;
		lp2hs_b = 20;
		hs2lp_m = 7;
		hs2lp_b = 25;
	} else {
		lp2hs_m = 7;
		lp2hs_b = 20;
		hs2lp_m = 5;
		hs2lp_b = 10;
	}

	timing->data_lp2hs = (DIV_ROUND_UP(lane_mbps * lp2hs_m, 100) + lp2hs_b) << 16;
	timing->data_hs2lp = (DIV_ROUND_UP(lane_mbps * hs2lp_m, 100) + hs2lp_b) << 16;

	return 0;
}

static int
imx952_dsi2_phy_get_esc_clk_rate(void *priv_data, unsigned long *esc_clk_rate)
{
	struct imx952_dsi2 *dsi = priv_data;
	unsigned long pclk_rate;
	unsigned int div = 2;

	pclk_rate = clk_get_rate(dsi->clk_pixel);
	if (pclk_rate == 0)
		return -EINVAL;

	*esc_clk_rate = pclk_rate;
	while (*esc_clk_rate > MAX_ESC_CLK_RATE_HZ) {
		*esc_clk_rate = pclk_rate / div;
		div = div + 2;

		if (div > 126)
			return -EINVAL;
	}

	dsi->esc_clk_rate = *esc_clk_rate;

	dev_dbg(dsi->dev, "get esc_clk_rate = %lu\n", *esc_clk_rate);

	return 0;
}

static const struct dw_mipi_dsi2_phy_ops imx952_dsi2_phy_ops = {
	.init = imx952_dsi2_phy_init,
	.power_on = imx952_dsi2_phy_power_on,
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
	dsi->mode_flags = device->mode_flags;

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
