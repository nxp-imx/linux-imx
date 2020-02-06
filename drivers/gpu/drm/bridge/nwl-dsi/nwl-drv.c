// SPDX-License-Identifier: GPL-2.0+
/*
 * i.MX8 NWL MIPI DSI host driver
 *
 * Copyright (C) 2017 NXP
 * Copyright (C) 2019 Purism SPC
 */

#include <dt-bindings/firmware/imx/rsrc.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/irq.h>
#include <linux/firmware/imx/sci.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mux/consumer.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/reset.h>
#include <linux/regmap.h>
#include <linux/sys_soc.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>


#include "nwl-drv.h"
#include "nwl-dsi.h"

#define DRV_NAME "nwl-dsi"

/* Possible platform specific clocks */
#define NWL_DSI_CLK_CORE	"core"
#define NWL_DSI_CLK_BYPASS	"bypass"
#define NWL_DSI_CLK_PIXEL	"pixel"

/* Maximum Video PLL frequency */
#define MAX_PLL_FREQ 1200000000

#define MBPS(x) ((x) * 1000000)
#define MIN_PHY_RATE MBPS(24)
#define MAX_PHY_RATE MBPS(30)

#define DC_ID(x)	IMX_SC_R_DC_ ## x
#define MIPI_ID(x)	IMX_SC_R_MIPI_ ## x
#define SYNC_CTRL(x)	IMX_SC_C_SYNC_CTRL ## x
#define PXL_VLD(x)	IMX_SC_C_PXL_LINK_MST ## x ## _VLD
#define PXL_ADDR(x)	IMX_SC_C_PXL_LINK_MST ## x ## _ADDR

/* Possible valid PHY reference clock rates*/
static u32 phyref_rates[] = {
	27000000,
	25000000,
	24000000,
};

/*
 * TODO: find a better way to access imx_crtc_state
 */
struct imx_crtc_state {
	struct drm_crtc_state			base;
	u32					bus_format;
	u32					bus_flags;
	int					di_hsync_pin;
	int					di_vsync_pin;
};

static inline struct imx_crtc_state *to_imx_crtc_state(struct drm_crtc_state *s)
{
	return container_of(s, struct imx_crtc_state, base);
}

enum nwl_dsi_ext_regs {
       NWL_DSI_IMX_REG_GPR = BIT(1),
};

static const struct regmap_config nwl_dsi_regmap_config = {
	.reg_bits = 16,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = NWL_DSI_IRQ_MASK2,
	.name = DRV_NAME,
};

struct nwl_dsi_platform_data {
	int (*pclk_reset)(struct nwl_dsi *dsi, bool reset);
	int (*mipi_reset)(struct nwl_dsi *dsi, bool reset);
	int (*dpi_reset)(struct nwl_dsi *dsi, bool reset);
	int (*select_input)(struct nwl_dsi *dsi);
	int (*deselect_input)(struct nwl_dsi *dsi);
	struct nwl_dsi_plat_clk_config clk_config[NWL_DSI_MAX_PLATFORM_CLOCKS];
	u32 reg_tx_ulps;
	u32 reg_pxl2dpi;
	u32 max_instances;
	u32 tx_clk_rate;
	u32 rx_clk_rate;
	bool mux_present;
	bool shared_phy;
};

static inline struct nwl_dsi *bridge_to_dsi(struct drm_bridge *bridge)
{
	return container_of(bridge, struct nwl_dsi, bridge);
}

static unsigned long nwl_dsi_get_bit_clock(struct nwl_dsi *dsi,
		unsigned long pixclock, u32 lanes)
{
	int bpp;

	if (lanes < 1 || lanes > 4)
		return 0;

	bpp = mipi_dsi_pixel_format_to_bpp(dsi->format);

	return (pixclock * bpp) / lanes;
}

/*
 * Utility function to calculate least commom multiple, using an improved
 * version of the Euclidean algorithm for greatest common factor.
 */
static unsigned long nwl_dsi_get_lcm(unsigned long a, unsigned long b)
{
	u32 gcf = 0; /* greatest common factor */
	unsigned long tmp_a = a;
	unsigned long tmp_b = b;

	if (!a || !b)
		return 0;

	while (tmp_a % tmp_b) {
		gcf = tmp_a % tmp_b;
		tmp_a = tmp_b;
		tmp_b = gcf;
	}

	if (!gcf)
		return a;

	return ((unsigned long long)a * b) / gcf;
}

/*
 * This function tries to adjust the crtc_clock for a DSI device in such a way
 * that the video pll will be able to satisfy both Display Controller pixel
 * clock (feeding out DPI interface) and our input phy_ref clock.
 * Also, the DC pixel clock must be lower than the actual clock in order to
 * have enough blanking space to send DSI commands, if the device is a panel.
 */
static void nwl_dsi_setup_pll_config(struct mode_config *config, u32 lvl)
{
	unsigned long pll_rate;
	int div;
	size_t i, num_rates = ARRAY_SIZE(config->phy_rates);

	config->crtc_clock = 0;

	for (i = 0; i < num_rates; i++) {
		int crtc_clock;

		if (!config->phy_rates[i])
			break;
		/*
		 * First, we need to check if phy_ref can actually be obtained
		 * from pixel clock. To do this, we check their lowest common
		 * multiple, which has to be in PLL range.
		 */
		pll_rate = nwl_dsi_get_lcm(config->clock, config->phy_rates[i]);
		if (pll_rate > MAX_PLL_FREQ) {
			/* Drop pll_rate to a realistic value */
			while (pll_rate > MAX_PLL_FREQ)
				pll_rate >>= 1;
			/* Make sure pll_rate can provide phy_ref rate */
			div = DIV_ROUND_UP(pll_rate, config->phy_rates[i]);
			pll_rate = config->phy_rates[i] * div;
		} else {
			/*
			 * Increase the pll rate to highest possible rate for
			 * better accuracy.
			 */
			while (pll_rate <= MAX_PLL_FREQ)
				pll_rate <<= 1;
			pll_rate >>= 1;
		}

		/*
		 * Next, we need to tweak the pll_rate to a value that can also
		 * satisfy the crtc_clock.
		 */
		div = DIV_ROUND_CLOSEST(pll_rate, config->clock);
		if (lvl)
			pll_rate -= config->phy_rates[i] * lvl;
		crtc_clock = pll_rate / div;
		config->pll_rates[i] = pll_rate;

		/*
		 * Pick a crtc_clock which is closest to pixel clock.
		 * Also, make sure that the pixel clock is a multiply of
		 * 50Hz.
		 */
		if (!(crtc_clock % 50) &&
		    abs(config->clock - crtc_clock) <
		    abs(config->clock - config->crtc_clock)) {
			config->crtc_clock = crtc_clock;
			config->phy_rate_idx = i;
		}
	}
}

/*
 * This function will try the required phy speed for current mode
 * If the phy speed can be achieved, the phy will save the speed
 * configuration
 */
static struct mode_config *nwl_dsi_mode_probe(struct nwl_dsi *dsi,
			    const struct drm_display_mode *mode)
{
	struct device *dev = dsi->dev;
	struct mode_config *config;
	union phy_configure_opts phy_opts;
	unsigned long clock = mode->clock * 1000;
	unsigned long bit_clk = 0;
	unsigned long phy_rates[3] = {0};
	int match_rates = 0;
	u32 lanes = dsi->lanes;
	size_t i = 0, num_rates = ARRAY_SIZE(phyref_rates);

	list_for_each_entry(config, &dsi->valid_modes, list)
		if (config->clock == clock)
			return config;

	phy_mipi_dphy_get_default_config(clock,
			mipi_dsi_pixel_format_to_bpp(dsi->format),
			lanes, &phy_opts.mipi_dphy);
	phy_opts.mipi_dphy.lp_clk_rate = clk_get_rate(dsi->tx_esc_clk);

	while (i < num_rates) {
		int ret;

		bit_clk = nwl_dsi_get_bit_clock(dsi, clock, lanes);

		clk_set_rate(dsi->pll_clk, phyref_rates[i] * 32);
		clk_set_rate(dsi->phy_ref_clk, phyref_rates[i]);
		ret = phy_validate(dsi->phy, PHY_MODE_MIPI_DPHY, 0, &phy_opts);

		/* Pick the non-failing rate, and search for more */
		if (!ret) {
			phy_rates[match_rates++] = phyref_rates[i++];
			continue;
		}

		if (match_rates)
			break;

		/* Reached the end of phyref_rates, try another lane config */
		if ((i++ == num_rates - 1) && (--lanes > 2)) {
			i = 0;
			continue;
		}
	}

	/*
	 * Try swinging between min and max pll rates and see what rate (in terms
	 * of kHz) we can custom use to get the required bit-clock.
	 */
	if (!match_rates) {
		int min_div, max_div;
		int bit_clk_khz;

		lanes = dsi->lanes;
		bit_clk = nwl_dsi_get_bit_clock(dsi, clock, lanes);

		min_div = DIV_ROUND_UP(bit_clk, MAX_PHY_RATE);
		max_div = DIV_ROUND_DOWN_ULL(bit_clk, MIN_PHY_RATE);
		bit_clk_khz = bit_clk / 1000;

		for (i = max_div; i > min_div; i--) {
			if (!(bit_clk_khz % i)) {
				phy_rates[0] = bit_clk / i;
				match_rates = 1;
				break;
			}
		}
	}

	if (!match_rates) {
		DRM_DEV_DEBUG_DRIVER(dev,
			"Cannot setup PHY for mode: %ux%u @%d kHz\n",
			mode->hdisplay,
			mode->vdisplay,
			mode->clock);

		return NULL;
	}

	config = devm_kzalloc(dsi->dev, sizeof(struct mode_config), GFP_KERNEL);
	config->clock = clock;
	config->lanes = lanes;
	config->bitclock = bit_clk;
	memcpy(&config->phy_rates, &phy_rates, sizeof(phy_rates));
	list_add(&config->list, &dsi->valid_modes);

	return config;
}

static int nwl_dsi_set_platform_clocks(struct nwl_dsi *dsi, bool enable)
{
	struct device *dev = dsi->dev;
	const char *id;
	struct clk *clk;
	size_t i;
	unsigned long rate;
	int ret, result = 0;

	DRM_DEV_DEBUG_DRIVER(dev, "%s platform clocks\n",
			     enable ? "enabling" : "disabling");
	for (i = 0; i < ARRAY_SIZE(dsi->pdata->clk_config); i++) {
		if (!dsi->clk_config[i].present)
			continue;
		id = dsi->clk_config[i].id;
		clk = dsi->clk_config[i].clk;

		if (enable) {
			ret = clk_prepare_enable(clk);
			if (ret < 0) {
				DRM_DEV_ERROR(dev,
					      "Failed to enable %s clk: %d\n",
					      id, ret);
				result = result ?: ret;
			}
			rate = clk_get_rate(clk);
			DRM_DEV_DEBUG_DRIVER(dev, "Enabled %s clk @%lu Hz\n",
					     id, rate);
		} else {
			clk_disable_unprepare(clk);
			DRM_DEV_DEBUG_DRIVER(dev, "Disabled %s clk\n", id);
		}
	}

	return result;
}

static void nwl_dsi_bridge_pre_enable(struct drm_bridge *bridge)
{
	struct nwl_dsi *dsi = bridge_to_dsi(bridge);
	int ret;

	pm_runtime_get_sync(dsi->dev);

	if (dsi->lcdif_clk)
		clk_prepare_enable(dsi->lcdif_clk);

	if (nwl_dsi_set_platform_clocks(dsi, true) < 0)
		return;

	/* Perform Step 1 from DSI reset-out instructions */
	ret = dsi->pdata->pclk_reset(dsi, false);
	if (ret < 0) {
		DRM_DEV_ERROR(dsi->dev, "Failed to deassert PCLK: %d\n", ret);
		return;
	}

	/* Perform Step 2 from DSI reset-out instructions */
	nwl_dsi_enable(dsi);

	/* Perform Step 3 from DSI reset-out instructions */
	ret = dsi->pdata->mipi_reset(dsi, false);
	if (ret < 0) {
		DRM_DEV_ERROR(dsi->dev, "Failed to deassert MIPI: %d\n", ret);
		return;
	}

	/*
	 * We need to force call enable for the panel here, in order to
	 * make the panel initialization execute before our call to
	 * bridge_enable, where we will enable the DPI and start streaming
	 * pixels on the data lanes.
	 */
	drm_bridge_enable(dsi->panel_bridge);
}

static void nwl_dsi_bridge_enable(struct drm_bridge *bridge)
{
	struct nwl_dsi *dsi = bridge_to_dsi(bridge);
	int ret;

	/* Perform Step 5 from DSI reset-out instructions */
	ret = dsi->pdata->dpi_reset(dsi, false);
	if (ret < 0)
		DRM_DEV_ERROR(dsi->dev, "Failed to deassert DPI: %d\n", ret);

}

static void nwl_dsi_bridge_disable(struct drm_bridge *bridge)
{
	struct nwl_dsi *dsi = bridge_to_dsi(bridge);

	nwl_dsi_disable(dsi);

	dsi->pdata->dpi_reset(dsi, true);
	dsi->pdata->mipi_reset(dsi, true);
	dsi->pdata->pclk_reset(dsi, true);

	nwl_dsi_set_platform_clocks(dsi, false);

	if (dsi->lcdif_clk)
		clk_disable_unprepare(dsi->lcdif_clk);

	pm_runtime_put(dsi->dev);
}

static int nwl_dsi_get_dphy_params(struct nwl_dsi *dsi,
				   const struct drm_display_mode *mode,
				   union phy_configure_opts *phy_opts)
{
	unsigned long rate;
	int ret;

	if (dsi->lanes < 1 || dsi->lanes > 4)
		return -EINVAL;

	/*
	 * So far the DPHY spec minimal timings work for both mixel
	 * dphy and nwl dsi host
	 */
	ret = phy_mipi_dphy_get_default_config(
		mode->clock * 1000,
		mipi_dsi_pixel_format_to_bpp(dsi->format), dsi->lanes,
		&phy_opts->mipi_dphy);
	if (ret < 0)
		return ret;

	rate = clk_get_rate(dsi->tx_esc_clk);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "LP clk is @%lu Hz\n", rate);
	phy_opts->mipi_dphy.lp_clk_rate = rate;

	return 0;
}

static bool nwl_dsi_bridge_mode_fixup(struct drm_bridge *bridge,
				      const struct drm_display_mode *mode,
				      struct drm_display_mode *adjusted)
{
	struct nwl_dsi *dsi = bridge_to_dsi(bridge);
	struct mode_config *config;
	unsigned long pll_rate;

	DRM_DEV_DEBUG_DRIVER(dsi->dev, "Fixup mode:\n");
	drm_mode_debug_printmodeline(adjusted);

	config = nwl_dsi_mode_probe(dsi, adjusted);
	if (!config)
		return false;

	DRM_DEV_DEBUG_DRIVER(dsi->dev, "lanes=%u, data_rate=%lu\n",
			     config->lanes, config->bitclock);
	if (config->lanes < 2 || config->lanes > 4)
		return false;

	/* Max data rate for this controller is 1.5Gbps */
	if (config->bitclock > 1500000000)
		return false;

	pll_rate = config->pll_rates[config->phy_rate_idx];
	if (dsi->pll_clk && pll_rate) {
		clk_set_rate(dsi->pll_clk, pll_rate);
		DRM_DEV_DEBUG_DRIVER(dsi->dev,
				     "Video pll rate: %lu (actual: %lu)",
				     pll_rate, clk_get_rate(dsi->pll_clk));
	}
	/* Update the crtc_clock to be used by display controller */
	if (config->crtc_clock)
		adjusted->crtc_clock = config->crtc_clock / 1000;
	else if (dsi->clk_drop_lvl) {
		int div;
		unsigned long phy_ref_rate;

		phy_ref_rate = config->phy_rates[config->phy_rate_idx];
		pll_rate = config->bitclock;
		div = DIV_ROUND_CLOSEST(pll_rate, config->clock);
		pll_rate -= phy_ref_rate * dsi->clk_drop_lvl;
		adjusted->crtc_clock = (pll_rate / div) / 1000;
	}

	if (!dsi->use_dcss) {
		/* LCDIF + NWL needs active high sync */
		adjusted->flags |= (DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC);
		adjusted->flags &= ~(DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC);
	} else {
		adjusted->flags &= ~(DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC);
		adjusted->flags |= (DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC);
	}

	return true;
}

static enum drm_mode_status
nwl_dsi_bridge_mode_valid(struct drm_bridge *bridge,
			  const struct drm_display_mode *mode)
{
	struct nwl_dsi *dsi = bridge_to_dsi(bridge);
	struct mode_config *config;
	unsigned long pll_rate;
	int bit_rate;

	bit_rate = nwl_dsi_get_bit_clock(dsi, mode->clock * 1000, dsi->lanes);

	DRM_DEV_DEBUG_DRIVER(dsi->dev, "Validating mode:");
	drm_mode_debug_printmodeline(mode);

	if (bit_rate > MBPS(1500))
		return MODE_CLOCK_HIGH;

	if (bit_rate < MBPS(80))
		return MODE_CLOCK_LOW;

	config = nwl_dsi_mode_probe(dsi, mode);
	if (!config)
		return MODE_NOCLOCK;

	pll_rate = config->pll_rates[config->phy_rate_idx];
	if (dsi->pll_clk && !pll_rate)
		nwl_dsi_setup_pll_config(config, dsi->clk_drop_lvl);

	return MODE_OK;
}

static void
nwl_dsi_bridge_mode_set(struct drm_bridge *bridge,
			const struct drm_display_mode *mode,
			const struct drm_display_mode *adjusted)
{
	struct nwl_dsi *dsi = bridge_to_dsi(bridge);
	struct device *dev = dsi->dev;
	union phy_configure_opts new_cfg;
	unsigned long phy_ref_rate;
	struct mode_config *config;
	size_t i;
	const char *id;
	struct clk *clk;
	int ret;

	DRM_DEV_DEBUG_DRIVER(dsi->dev, "Setting mode:\n");
	drm_mode_debug_printmodeline(adjusted);

	config = nwl_dsi_mode_probe(dsi, adjusted);
	/* New mode? This should NOT happen */
	if (!config) {
		DRM_DEV_ERROR(dsi->dev, "Unsupported mode provided:\n");
		drm_mode_debug_printmodeline(adjusted);
		return;
	}

	for (i = 0; i < ARRAY_SIZE(dsi->pdata->clk_config); i++) {
		if (!dsi->clk_config[i].present)
			continue;
		id = dsi->clk_config[i].id;
		clk = dsi->clk_config[i].clk;

		/* Set bypass and pixel clocks to mode clock rate */
		if (!strcmp(id, NWL_DSI_CLK_BYPASS) ||
		    !strcmp(id, NWL_DSI_CLK_PIXEL))
			clk_set_rate(clk, adjusted->crtc_clock * 1000);
	}

	memcpy(&dsi->mode, adjusted, sizeof(dsi->mode));

	phy_ref_rate = config->phy_rates[config->phy_rate_idx];
	clk_set_rate(dsi->phy_ref_clk, phy_ref_rate);
	ret = nwl_dsi_get_dphy_params(dsi, adjusted, &new_cfg);
	if (ret < 0)
		return;

	/*
	 * If hs clock is unchanged, we're all good - all parameters are
	 * derived from it atm.
	 */
	if (new_cfg.mipi_dphy.hs_clk_rate == dsi->phy_cfg.mipi_dphy.hs_clk_rate)
		return;

	DRM_DEV_DEBUG_DRIVER(dev,
			     "PHY at ref rate: %lu (actual: %lu)\n",
			     phy_ref_rate, clk_get_rate(dsi->phy_ref_clk));
	/* Save the new desired phy config */
	memcpy(&dsi->phy_cfg, &new_cfg, sizeof(new_cfg));

}

static int nwl_dsi_bridge_attach(struct drm_bridge *bridge)
{
	struct nwl_dsi *dsi = bridge->driver_private;
	struct drm_bridge *panel_bridge;
	struct drm_panel *panel;
	struct clk *phy_parent;
	int ret;

	ret = drm_of_find_panel_or_bridge(dsi->dev->of_node, 1, 0, &panel,
					  &panel_bridge);
	if (ret)
		return ret;

	if (panel) {
		panel_bridge = drm_panel_bridge_add(panel,
						    DRM_MODE_CONNECTOR_DSI);
		if (IS_ERR(panel_bridge))
			return PTR_ERR(panel_bridge);
	}

	dsi->panel_bridge = panel_bridge;

	if (!dsi->panel_bridge)
		return -EPROBE_DEFER;

	phy_parent = devm_clk_get(dsi->dev, "phy_parent");
	if (!IS_ERR_OR_NULL(phy_parent)) {
		ret = clk_set_parent(dsi->phy_ref_clk, phy_parent);
		ret |= clk_set_parent(dsi->tx_esc_clk, phy_parent);
		ret |= clk_set_parent(dsi->rx_esc_clk, phy_parent);

		if (ret) {
			dev_err(dsi->dev,
				 "Error re-parenting phy/tx/rx clocks: %d",
				 ret);

			return ret;
		}

		if (dsi->pdata->tx_clk_rate)
			clk_set_rate(dsi->tx_esc_clk, dsi->pdata->tx_clk_rate);

		if (dsi->pdata->rx_clk_rate)
			clk_set_rate(dsi->rx_esc_clk, dsi->pdata->rx_clk_rate);
	}

	return drm_bridge_attach(bridge->encoder, dsi->panel_bridge, bridge);
}

static void nwl_dsi_bridge_detach(struct drm_bridge *bridge)
{
	struct nwl_dsi *dsi = bridge->driver_private;

	drm_of_panel_bridge_remove(dsi->dev->of_node, 1, 0);
}

static const struct drm_bridge_funcs nwl_dsi_bridge_funcs = {
	.pre_enable = nwl_dsi_bridge_pre_enable,
	.enable     = nwl_dsi_bridge_enable,
	.disable    = nwl_dsi_bridge_disable,
	.mode_fixup = nwl_dsi_bridge_mode_fixup,
	.mode_set   = nwl_dsi_bridge_mode_set,
	.mode_valid = nwl_dsi_bridge_mode_valid,
	.attach	    = nwl_dsi_bridge_attach,
	.detach	    = nwl_dsi_bridge_detach,
};

static void nwl_dsi_encoder_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

static const struct drm_encoder_funcs nwl_dsi_encoder_funcs = {
	.destroy = nwl_dsi_encoder_destroy,
};

static int nwl_dsi_parse_dt(struct nwl_dsi *dsi)
{
	struct device_node *np = dsi->dev->of_node;
	struct platform_device *pdev = to_platform_device(dsi->dev);
	struct clk *clk;
	const char *clk_id;
	void __iomem *base;
	int i, ret, id;

	dsi->phy = devm_phy_get(dsi->dev, "dphy");
	if (IS_ERR(dsi->phy)) {
		ret = PTR_ERR(dsi->phy);
		if (ret != -EPROBE_DEFER)
			DRM_DEV_ERROR(dsi->dev, "Could not get PHY: %d\n", ret);
		return ret;
	}

	id = of_alias_get_id(np, "mipi_dsi");
	if (id > 0) {
		if (id > dsi->pdata->max_instances - 1) {
			dev_err(dsi->dev,
				"Too many instances! (cur: %d, max: %d)\n",
				id, dsi->pdata->max_instances);
			return -ENODEV;
		}
		dsi->instance = id;
	}

	/* Platform dependent clocks */
	memcpy(dsi->clk_config, dsi->pdata->clk_config,
	       sizeof(dsi->pdata->clk_config));

	for (i = 0; i < ARRAY_SIZE(dsi->pdata->clk_config); i++) {
		if (!dsi->clk_config[i].present)
			continue;

		clk_id = dsi->clk_config[i].id;
		clk = devm_clk_get(dsi->dev, clk_id);
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			DRM_DEV_ERROR(dsi->dev, "Failed to get %s clock: %d\n",
				      clk_id, ret);
			return ret;
		}
		DRM_DEV_DEBUG_DRIVER(dsi->dev, "Setup clk %s (rate: %lu)\n",
				     clk_id, clk_get_rate(clk));
		dsi->clk_config[i].clk = clk;
	}

	/* DSI clocks */
	clk = devm_clk_get(dsi->dev, "phy_ref");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		DRM_DEV_ERROR(dsi->dev, "Failed to get phy_ref clock: %d\n",
			      ret);
		return ret;
	}
	dsi->phy_ref_clk = clk;

	clk = devm_clk_get(dsi->dev, "rx_esc");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		DRM_DEV_ERROR(dsi->dev, "Failed to get rx_esc clock: %d\n",
			      ret);
		return ret;
	}
	dsi->rx_esc_clk = clk;

	clk = devm_clk_get(dsi->dev, "tx_esc");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		DRM_DEV_ERROR(dsi->dev, "Failed to get tx_esc clock: %d\n",
			      ret);
		return ret;
	}
	dsi->tx_esc_clk = clk;

	/* The video_pll clock is optional */
	clk = devm_clk_get(dsi->dev, "video_pll");
	if (!IS_ERR(clk))
		dsi->pll_clk = clk;

	/*
	 * Usually, we don't need this clock here, but due to a design issue,
	 * the MIPI Reset Synchronizer block depends on this clock. So, we will
	 * use this clock when we need to reset (or take out of reset) MIPI PHY
	 */
	dsi->lcdif_clk = devm_clk_get(dsi->dev, "lcdif");
	if (IS_ERR(dsi->lcdif_clk))
		dsi->lcdif_clk = NULL;

	if (dsi->pdata->mux_present) {
		dsi->mux = devm_mux_control_get(dsi->dev, NULL);
		if (IS_ERR(dsi->mux)) {
			ret = PTR_ERR(dsi->mux);
			if (ret != -EPROBE_DEFER)
				DRM_DEV_ERROR(dsi->dev,
					      "Failed to get mux: %d\n", ret);
			return ret;
		}
	}

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	dsi->regmap =
		devm_regmap_init_mmio(dsi->dev, base, &nwl_dsi_regmap_config);
	if (IS_ERR(dsi->regmap)) {
		ret = PTR_ERR(dsi->regmap);
		DRM_DEV_ERROR(dsi->dev, "Failed to create NWL DSI regmap: %d\n",
			      ret);
		return ret;
	}

	/* For these two regs we need a mapping to MIPI-DSI CSR */
	if (dsi->pdata->reg_tx_ulps || dsi->pdata->reg_pxl2dpi) {
		dsi->csr = syscon_regmap_lookup_by_phandle(np, "csr");
		if (IS_ERR(dsi->csr)) {
			ret = PTR_ERR(dsi->csr);
			dev_err(dsi->dev,
				"Failed to get CSR regmap: %d\n", ret);
			return ret;
		}
	}

	dsi->irq = platform_get_irq(pdev, 0);
	if (dsi->irq < 0) {
		if (dsi->irq != -EPROBE_DEFER)
			DRM_DEV_ERROR(dsi->dev,
				      "Failed to get device IRQ: %d\n",
				      dsi->irq);
		return dsi->irq;
	}

	/*
	 * Wee need to manage individual resets, since the order of their usage
	 * really matters to DSI Host Controller. The order of operation,
	 * should be like this:
	 * 1. Deassert pclk reset (this is needed to have access to DSI regs)
	 * 2. Configure DSI Host and DPHY and enable DPHY
	 * 3. Deassert ESC and BYTE resets (needed for Host TX operations)
	 * 4. Send DSI cmds (if the DSI peripheral needs configuration)
	 * 5. Deassert DPI reset (deasserting DPI reset, enables DPI to receive
	 *    DPI pixels and start streamming DSI data)
	 */
	dsi->rst_pclk = __devm_reset_control_get(dsi->dev,
						 "pclk", 0, false, true, true);
	if (IS_ERR(dsi->rst_pclk)) {
		DRM_DEV_ERROR(dsi->dev, "Failed to get pclk reset: %ld\n",
			      PTR_ERR(dsi->rst_pclk));
		return PTR_ERR(dsi->rst_pclk);
	}
	dsi->rst_byte = __devm_reset_control_get(dsi->dev,
						 "byte", 0, false, true, true);
	if (IS_ERR(dsi->rst_byte)) {
		DRM_DEV_ERROR(dsi->dev, "Failed to get byte reset: %ld\n",
			      PTR_ERR(dsi->rst_byte));
		return PTR_ERR(dsi->rst_byte);
	}
	dsi->rst_esc = __devm_reset_control_get(dsi->dev,
						"esc", 0, false, true, true);
	if (IS_ERR(dsi->rst_esc)) {
		DRM_DEV_ERROR(dsi->dev, "Failed to get esc reset: %ld\n",
			      PTR_ERR(dsi->rst_esc));
		return PTR_ERR(dsi->rst_esc);
	}
	dsi->rst_dpi = __devm_reset_control_get(dsi->dev,
						"dpi", 0, false, true, true);
	if (IS_ERR(dsi->rst_dpi)) {
		DRM_DEV_ERROR(dsi->dev, "Failed to get dpi reset: %ld\n",
			      PTR_ERR(dsi->rst_dpi));
		return PTR_ERR(dsi->rst_dpi);
	}

	of_property_read_u32(np, "fsl,clock-drop-level", &dsi->clk_drop_lvl);

	return 0;
}

static int imx8mq_dsi_pclk_reset(struct nwl_dsi *dsi, bool reset)
{
	int ret = 0;

	if (dsi->rst_pclk) {
		if (reset)
			ret = reset_control_assert(dsi->rst_pclk);
		else
			ret = reset_control_deassert(dsi->rst_pclk);
	}

	return ret;

}

static int imx8mq_dsi_mipi_reset(struct nwl_dsi *dsi, bool reset)
{
	int ret = 0;

	if (dsi->rst_esc) {
		if (reset)
			ret = reset_control_assert(dsi->rst_esc);
		else
			ret = reset_control_deassert(dsi->rst_esc);
	}

	if (dsi->rst_byte) {
		if (reset)
			ret = reset_control_assert(dsi->rst_byte);
		else
			ret = reset_control_deassert(dsi->rst_byte);
	}

	return ret;

}

static int imx8mq_dsi_dpi_reset(struct nwl_dsi *dsi, bool reset)
{
	int ret = 0;

	if (dsi->rst_dpi) {
		if (reset)
			ret = reset_control_assert(dsi->rst_dpi);
		else
			ret = reset_control_deassert(dsi->rst_dpi);
	}

	return ret;

}

static int imx8mq_dsi_select_input(struct nwl_dsi *dsi)
{
	struct device_node *remote;
	u32 use_dcss = 1;
	int ret;

	remote = of_graph_get_remote_node(dsi->dev->of_node, 0, 0);
	if (strcmp(remote->name, "lcdif") == 0)
		use_dcss = 0;

	DRM_DEV_INFO(dsi->dev, "Using %s as input source\n",
		     (use_dcss) ? "DCSS" : "LCDIF");

	ret = mux_control_try_select(dsi->mux, use_dcss);
	if (ret < 0)
		DRM_DEV_ERROR(dsi->dev, "Failed to select input: %d\n", ret);

	of_node_put(remote);

	dsi->use_dcss = use_dcss;

	return ret;
}

static int imx8mq_dsi_deselect_input(struct nwl_dsi *dsi)
{
	int ret;

	ret = mux_control_deselect(dsi->mux);
	if (ret < 0)
		DRM_DEV_ERROR(dsi->dev, "Failed to deselect input: %d\n", ret);

	return ret;
}

static int imx8q_dsi_pclk_reset(struct nwl_dsi *dsi, bool reset)
{
	struct imx_sc_ipc *handle;
	u32 mipi_id, dc_id;
	u8 ctrl;
	bool shared_phy = dsi->pdata->shared_phy;
	int ret = 0;

	ret = imx_scu_get_handle(&handle);
	if (ret) {
		DRM_DEV_ERROR(dsi->dev,
			      "Failed to get scu ipc handle (%d)\n", ret);
		return ret;
	}

	mipi_id = (dsi->instance)?MIPI_ID(1):MIPI_ID(0);
	dc_id = (!shared_phy && dsi->instance)?DC_ID(1):DC_ID(0);
	DRM_DEV_DEBUG_DRIVER(dsi->dev,
			     "Power %s PCLK MIPI:%u DC:%u\n",
			     (reset)?"OFF":"ON", mipi_id, dc_id);

	if (shared_phy) {
		ret |= imx_sc_misc_set_control(handle,
				mipi_id, IMX_SC_C_MODE, reset);
		ret |= imx_sc_misc_set_control(handle,
				mipi_id, IMX_SC_C_DUAL_MODE, reset);
		ret |= imx_sc_misc_set_control(handle,
				mipi_id, IMX_SC_C_PXL_LINK_SEL, reset);
	}

	/* Initialize Pixel Link */
	ctrl = (shared_phy && dsi->instance)?PXL_ADDR(2):PXL_ADDR(1);
	ret |= imx_sc_misc_set_control(handle, dc_id, ctrl, reset);

	ctrl = (shared_phy && dsi->instance)?PXL_VLD(2):PXL_VLD(1);
	ret |= imx_sc_misc_set_control(handle, dc_id, ctrl, !reset);

	ctrl = (shared_phy && dsi->instance)?SYNC_CTRL(1):SYNC_CTRL(0);
	ret |= imx_sc_misc_set_control(handle, dc_id, ctrl, !reset);

	return ret;
}

static int imx8q_dsi_mipi_reset(struct nwl_dsi *dsi, bool reset)
{
	struct imx_sc_ipc *handle;
	u32 mipi_id;
	int ret = 0;

	ret = imx_scu_get_handle(&handle);
	if (ret) {
		DRM_DEV_ERROR(dsi->dev,
			      "Failed to get scu ipc handle (%d)\n", ret);
		return ret;
	}

	mipi_id = (dsi->instance)?MIPI_ID(1):MIPI_ID(0);
	DRM_DEV_DEBUG_DRIVER(dsi->dev,
			     "Power %s HOST MIPI:%u\n",
			     (reset)?"OFF":"ON", mipi_id);

	ret |= imx_sc_misc_set_control(handle, mipi_id,
				       IMX_SC_C_PHY_RESET, !reset);
	ret |= imx_sc_misc_set_control(handle, mipi_id,
				       IMX_SC_C_MIPI_RESET, !reset);

	return ret;
}

static int imx8q_dsi_dpi_reset(struct nwl_dsi *dsi, bool reset)
{
	struct imx_sc_ipc *handle;
	u32 mipi_id;
	int ret = 0;

	ret = imx_scu_get_handle(&handle);
	if (ret) {
		DRM_DEV_ERROR(dsi->dev,
			      "Failed to get scu ipc handle (%d)\n", ret);
		return ret;
	}

	mipi_id = (dsi->instance)?MIPI_ID(1):MIPI_ID(0);
	DRM_DEV_DEBUG_DRIVER(dsi->dev,
			     "Power %s DPI MIPI:%u\n",
			     (reset)?"OFF":"ON", mipi_id);

	regmap_write(dsi->csr, dsi->pdata->reg_tx_ulps, 0);
	regmap_write(dsi->csr, dsi->pdata->reg_pxl2dpi, NWL_DSI_DPI_24_BIT);

	ret |= imx_sc_misc_set_control(handle, mipi_id,
				       IMX_SC_C_DPI_RESET, !reset);

	return ret;
}

static const struct drm_bridge_timings nwl_dsi_timings = {
	.input_bus_flags = DRM_BUS_FLAG_DE_LOW,
};

static const struct nwl_dsi_platform_data imx8mq_dev = {
	.pclk_reset = &imx8mq_dsi_pclk_reset,
	.mipi_reset = &imx8mq_dsi_mipi_reset,
	.dpi_reset = &imx8mq_dsi_dpi_reset,
	.select_input = &imx8mq_dsi_select_input,
	.deselect_input = &imx8mq_dsi_deselect_input,
	.clk_config = {
		{ .id = NWL_DSI_CLK_CORE, .present = true },
	},
	.mux_present = true,
};

static const struct nwl_dsi_platform_data imx8qm_dev = {
	.pclk_reset = &imx8q_dsi_pclk_reset,
	.mipi_reset = &imx8q_dsi_mipi_reset,
	.dpi_reset = &imx8q_dsi_dpi_reset,
	.clk_config = {
		{ .id = NWL_DSI_CLK_BYPASS, .present = true },
		{ .id = NWL_DSI_CLK_PIXEL, .present = true },
	},
	.reg_tx_ulps = 0x00,
	.reg_pxl2dpi = 0x04,
	.max_instances = 2,
	.tx_clk_rate = 18000000,
	.rx_clk_rate = 72000000,
	.shared_phy = false,
};

static const struct nwl_dsi_platform_data imx8qx_dev = {
	.pclk_reset = &imx8q_dsi_pclk_reset,
	.mipi_reset = &imx8q_dsi_mipi_reset,
	.dpi_reset = &imx8q_dsi_dpi_reset,
	.clk_config = {
		{ .id = NWL_DSI_CLK_BYPASS, .present = true },
		{ .id = NWL_DSI_CLK_PIXEL, .present = true },
	},
	.reg_tx_ulps = 0x30,
	.reg_pxl2dpi = 0x40,
	.max_instances = 2,
	.tx_clk_rate = 18000000,
	.rx_clk_rate = 72000000,
	.shared_phy = true,
};

static const struct of_device_id nwl_dsi_dt_ids[] = {
	{ .compatible = "fsl,imx8mq-nwl-dsi", .data = &imx8mq_dev, },
	{ .compatible = "fsl,imx8qm-nwl-dsi", .data = &imx8qm_dev, },
	{ .compatible = "fsl,imx8qx-nwl-dsi", .data = &imx8qx_dev, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nwl_dsi_dt_ids);

static const struct soc_device_attribute nwl_dsi_quirks_match[] = {
	{ .soc_id = "i.MX8MQ", .revision = "2.0",
	  .data = (void *)(E11418_HS_MODE_QUIRK | SRC_RESET_QUIRK) },
	{ /* sentinel. */ },
};

static int nwl_dsi_encoder_atomic_check(struct drm_encoder *encoder,
					struct drm_crtc_state *crtc_state,
					struct drm_connector_state *conn_state)
{
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(crtc_state);

	imx_crtc_state->bus_format = MEDIA_BUS_FMT_RGB101010_1X30;

	return 0;
}

static const struct drm_encoder_helper_funcs nwl_dsi_encoder_helper_funcs = {
	.atomic_check = nwl_dsi_encoder_atomic_check,
};

static int nwl_dsi_bind(struct device *dev,
			struct device *master,
			void *data)
{
	struct drm_device *drm = data;
	uint32_t crtc_mask;
	struct nwl_dsi *dsi = dev_get_drvdata(dev);
	int ret = 0;

	crtc_mask = drm_of_find_possible_crtcs(drm, dev->of_node);
	/*
	 * If we failed to find the CRTC(s) which this encoder is
	 * supposed to be connected to, it's because the CRTC has
	 * not been registered yet.  Defer probing, and hope that
	 * the required CRTC is added later.
	 */
	if (crtc_mask == 0)
		return -EPROBE_DEFER;

	DRM_DEV_DEBUG_DRIVER(dev, "id = %s\n", (dsi->instance)?"DSI1":"DSI0");

	dsi->encoder.possible_crtcs = crtc_mask;
	dsi->encoder.possible_clones = ~0;

	drm_encoder_helper_add(&dsi->encoder,
			       &nwl_dsi_encoder_helper_funcs);
	ret = drm_encoder_init(drm,
			       &dsi->encoder,
			       &nwl_dsi_encoder_funcs,
			       DRM_MODE_ENCODER_DSI,
			       NULL);
	if (ret) {
		DRM_DEV_ERROR(dev, "failed to init DSI encoder (%d)\n", ret);
		return ret;
	}

	ret = drm_bridge_attach(&dsi->encoder, &dsi->bridge, NULL);
	if (ret)
		drm_encoder_cleanup(&dsi->encoder);

	/*
	 *  -ENODEV is returned when there is no node connected to us. Since
	 *  it might be disabled because the device is not actually connected,
	 *  just cleanup and return 0.
	 */
	if (ret == -ENODEV)
		return 0;

	return ret;
}

static void nwl_dsi_unbind(struct device *dev,
			   struct device *master,
			   void *data)
{
	struct nwl_dsi *dsi = dev_get_drvdata(dev);

	DRM_DEV_DEBUG_DRIVER(dev, "id = %s\n", (dsi->instance)?"DSI1":"DSI0");

	if (dsi->encoder.dev)
		drm_encoder_cleanup(&dsi->encoder);
}

static const struct component_ops nwl_dsi_component_ops = {
	.bind	= nwl_dsi_bind,
	.unbind	= nwl_dsi_unbind,
};

static int nwl_dsi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *of_id = of_match_device(nwl_dsi_dt_ids, dev);
	const struct nwl_dsi_platform_data *pdata = of_id->data;
	const struct soc_device_attribute *attr;
	struct nwl_dsi *dsi;
	int ret;

	dsi = devm_kzalloc(dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return -ENOMEM;

	dsi->dev = dev;
	dsi->pdata = pdata;

	attr = soc_device_match(nwl_dsi_quirks_match);
	if (attr)
		dsi->quirks = (uintptr_t)attr->data;

	ret = nwl_dsi_parse_dt(dsi);
	if (ret)
		return ret;

	ret = devm_request_irq(dev, dsi->irq, nwl_dsi_irq_handler, 0,
			       dev_name(dev), dsi);
	if (ret < 0) {
		if (ret != -EPROBE_DEFER)
			DRM_DEV_ERROR(dev, "Failed to request IRQ %d: %d\n",
				      dsi->irq, ret);
		return ret;
	}

	dsi->dsi_host.ops = &nwl_dsi_host_ops;
	dsi->dsi_host.dev = dev;
	ret = mipi_dsi_host_register(&dsi->dsi_host);
	if (ret) {
		DRM_DEV_ERROR(dev, "Failed to register MIPI host: %d\n", ret);
		return ret;
	}

	dsi->bridge.driver_private = dsi;
	dsi->bridge.funcs = &nwl_dsi_bridge_funcs;
	dsi->bridge.of_node = dev->of_node;
	dsi->bridge.timings = &nwl_dsi_timings;

	if (dsi->pdata->select_input)
		dsi->pdata->select_input(dsi);

	drm_bridge_add(&dsi->bridge);

	INIT_LIST_HEAD(&dsi->valid_modes);

	dev_set_drvdata(dev, dsi);
	pm_runtime_enable(dev);

	if (of_property_read_bool(dev->of_node, "use-disp-ss"))
		ret = component_add(&pdev->dev, &nwl_dsi_component_ops);

	if (ret) {
		pm_runtime_disable(dev);
		drm_bridge_remove(&dsi->bridge);
		mipi_dsi_host_unregister(&dsi->dsi_host);
		return ret;
	}

	return ret;
}

static int nwl_dsi_remove(struct platform_device *pdev)
{
	struct nwl_dsi *dsi = platform_get_drvdata(pdev);
	struct mode_config *config;
	struct list_head *pos, *tmp;

	list_for_each_safe(pos, tmp, &dsi->valid_modes) {
		config = list_entry(pos, struct mode_config, list);
		list_del(pos);
		devm_kfree(dsi->dev, config);
	}

	pm_runtime_disable(&pdev->dev);
	drm_bridge_remove(&dsi->bridge);
	mipi_dsi_host_unregister(&dsi->dsi_host);

	return 0;
}

static struct platform_driver nwl_dsi_driver = {
	.probe		= nwl_dsi_probe,
	.remove		= nwl_dsi_remove,
	.driver		= {
		.of_match_table = nwl_dsi_dt_ids,
		.name	= DRV_NAME,
	},
};

module_platform_driver(nwl_dsi_driver);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_AUTHOR("Purism SPC");
MODULE_DESCRIPTION("Northwest Logic MIPI-DSI driver");
MODULE_LICENSE("GPL"); /* GPLv2 or later */
