/*
 * NWL DSI drm driver - Northwest Logic MIPI DSI bridge
 *
 * Copyright (C) 2017 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <asm/unaligned.h>
#include <drm/bridge/nwl_dsi.h>
#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-mixel-mipi-dsi.h>
#include <linux/spinlock.h>
#include <video/mipi_display.h>
#include <video/videomode.h>

#define MIPI_FIFO_TIMEOUT msecs_to_jiffies(500)

/* DSI HOST registers */
#define CFG_NUM_LANES			0x0
#define CFG_NONCONTINUOUS_CLK		0x4
#define CFG_T_PRE			0x8
#define CFG_T_POST			0xc
#define CFG_TX_GAP			0x10
#define CFG_AUTOINSERT_EOTP		0x14
#define CFG_EXTRA_CMDS_AFTER_EOTP	0x18
#define CFG_HTX_TO_COUNT		0x1c
#define CFG_LRX_H_TO_COUNT		0x20
#define CFG_BTA_H_TO_COUNT		0x24
#define CFG_TWAKEUP			0x28
#define CFG_STATUS_OUT			0x2c
#define RX_ERROR_STATUS			0x30

/* DSI DPI registers */
#define PIXEL_PAYLOAD_SIZE		0x200
#define PIXEL_FIFO_SEND_LEVEL		0x204
#define INTERFACE_COLOR_CODING		0x208
#define PIXEL_FORMAT			0x20c
#define VSYNC_POLARITY			0x210
#define HSYNC_POLARITY			0x214
#define VIDEO_MODE			0x218
#define HFP				0x21c
#define HBP				0x220
#define HSA				0x224
#define ENABLE_MULT_PKTS		0x228
#define VBP				0x22c
#define VFP				0x230
#define BLLP_MODE			0x234
#define USE_NULL_PKT_BLLP		0x238
#define VACTIVE				0x23c
#define VC				0x240

/* DSI APB PKT control */
#define TX_PAYLOAD			0x280
#define PKT_CONTROL			0x284
#define SEND_PACKET			0x288
#define PKT_STATUS			0x28c
#define PKT_FIFO_WR_LEVEL		0x290
#define PKT_FIFO_RD_LEVEL		0x294
#define RX_PAYLOAD			0x298
#define RX_PKT_HEADER			0x29c

/* PKT reg bit manipulation */
#define REG_MASK(e, s) (((1 << ((e) - (s) + 1)) - 1) << (s))
#define REG_PUT(x, e, s) (((x) << (s)) & REG_MASK(e, s))
#define REG_GET(x, e, s) (((x) & REG_MASK(e, s)) >> (s))

/*
 * PKT_CONTROL format:
 * [15: 0] - word count
 * [17:16] - virtual channel
 * [23:18] - data type
 * [24]    - LP or HS select (0 - LP, 1 - HS)
 * [25]    - perform BTA after packet is sent
 * [26]    - perform BTA only, no packet tx
 */
#define WC(x)		REG_PUT((x), 15,  0)
#define TX_VC(x)	REG_PUT((x), 17, 16)
#define TX_DT(x)	REG_PUT((x), 23, 18)
#define HS_SEL(x)	REG_PUT((x), 24, 24)
#define BTA_TX(x)	REG_PUT((x), 25, 25)
#define BTA_NO_TX(x)	REG_PUT((x), 26, 26)

/*
 * RX_PKT_HEADER format:
 * [15: 0] - word count
 * [21:16] - data type
 * [23:22] - virtual channel
 */
#define RX_DT(x)	REG_GET((x), 21, 16)
#define RX_VC(x)	REG_GET((x), 23, 22)

/* DSI IRQ handling */
#define IRQ_STATUS			0x2a0
#define SM_NOT_IDLE			BIT(0)
#define TX_PKT_DONE			BIT(1)
#define DPHY_DIRECTION			BIT(2)
#define TX_FIFO_OVFLW			BIT(3)
#define TX_FIFO_UDFLW			BIT(4)
#define RX_FIFO_OVFLW			BIT(5)
#define RX_FIFO_UDFLW			BIT(6)
#define RX_PKT_HDR_RCVD			BIT(7)
#define RX_PKT_PAYLOAD_DATA_RCVD	BIT(8)
#define BTA_TIMEOUT			BIT(29)
#define LP_RX_TIMEOUT			BIT(30)
#define HS_TX_TIMEOUT			BIT(31)

#define IRQ_STATUS2			0x2a4
#define SINGLE_BIT_ECC_ERR		BIT(0)
#define MULTI_BIT_ECC_ERR		BIT(1)
#define CRC_ERR				BIT(2)

#define IRQ_MASK			0x2a8
#define SM_NOT_IDLE_MASK		BIT(0)
#define TX_PKT_DONE_MASK		BIT(1)
#define DPHY_DIRECTION_MASK		BIT(2)
#define TX_FIFO_OVFLW_MASK		BIT(3)
#define TX_FIFO_UDFLW_MASK		BIT(4)
#define RX_FIFO_OVFLW_MASK		BIT(5)
#define RX_FIFO_UDFLW_MASK		BIT(6)
#define RX_PKT_HDR_RCVD_MASK		BIT(7)
#define RX_PKT_PAYLOAD_DATA_RCVD_MASK	BIT(8)
#define BTA_TIMEOUT_MASK		BIT(29)
#define LP_RX_TIMEOUT_MASK		BIT(30)
#define HS_TX_TIMEOUT_MASK		BIT(31)

#define IRQ_MASK2			0x2ac
#define SINGLE_BIT_ECC_ERR_MASK		BIT(0)
#define MULTI_BIT_ECC_ERR_MASK		BIT(1)
#define CRC_ERR_MASK			BIT(2)

static const char IRQ_NAME[] = "nwl-dsi";

/*
 * TODO: Currently, filter-out unsupported modes by their clocks.
 * Need to find a better way to do this.
 * These are the pixel clocks that the controller can handle successfully.
 */
static int valid_clocks[] = {
	162000,
	148500,
	135000,
	132000,
	108000,
	74250,
	49500,
	31500,
};

/* Possible valid PHY reference clock rates*/
static u32 phyref_rates[] = {
	27000000,
	25000000,
	24000000,
};

enum {
	CLK_PHY_REF	= BIT(1),
	CLK_RX_ESC	= BIT(2),
	CLK_TX_ESC	= BIT(3)
};

enum transfer_direction {
	DSI_PACKET_SEND,
	DSI_PACKET_RECEIVE
};

struct mipi_dsi_transfer {
	const struct mipi_dsi_msg *msg;
	struct mipi_dsi_packet packet;
	struct completion completed;

	int status;    /* status of transmission */
	enum transfer_direction direction;
	bool need_bta;
	u8 cmd;
	u16 rx_word_count;
	size_t tx_len; /* bytes sent */
	size_t rx_len; /* bytes received */
};

struct clk_config {
	struct clk *clk;
	unsigned long rate;
	bool enabled;
};

struct mode_config {
	int				pixclock;
	unsigned int			lanes;
	unsigned long			bitclock;
	u32				phyref_rate;
	struct list_head		list;
};

struct nwl_mipi_dsi {
	struct device			*dev;
	struct drm_panel		*panel;
	struct drm_bridge		*next_bridge;
	struct drm_bridge		bridge;
	struct drm_connector		connector;
	struct mipi_dsi_host		host;
	struct mipi_dsi_device		*dsi_device;

	struct phy			*phy;

	/* Mandatory clocks */
	struct clk_config		phy_ref;
	struct clk_config		rx_esc;
	struct clk_config		tx_esc;

	/* Optional clocks */
	struct clk_config		phy_ref_sel;
	struct clk_config		rx_esc_sel;
	struct clk_config		tx_esc_sel;
	struct clk_config		phy_parent;

	void __iomem			*base;
	int				irq;

	struct mipi_dsi_transfer	*xfer;

	struct drm_display_mode		*curr_mode;
	struct list_head		valid_modes;
	u32				lanes;
	bool				no_clk_reset;
	bool				enabled;
};

static inline void nwl_dsi_write(struct nwl_mipi_dsi *dsi, u32 reg, u32 val)
{
	writel(val, dsi->base + reg);
}

static inline u32 nwl_dsi_read(struct nwl_mipi_dsi *dsi, u32 reg)
{
	return readl(dsi->base + reg);
}

static enum mipi_dsi_pixel_format mipi_dsi_format_from_bus_format(
		u32 bus_format)
{
	switch (bus_format) {
	case MEDIA_BUS_FMT_RGB565_1X16:
		return MIPI_DSI_FMT_RGB565;
	case MEDIA_BUS_FMT_RGB666_1X18:
		return MIPI_DSI_FMT_RGB666;
	case MEDIA_BUS_FMT_RGB888_1X24:
		return MIPI_DSI_FMT_RGB888;
	default:
		return MIPI_DSI_FMT_RGB888;
	}
}

static enum dpi_interface_color_coding nwl_dsi_get_dpi_interface_color_coding(
		enum mipi_dsi_pixel_format format)
{
	switch (format) {
	case MIPI_DSI_FMT_RGB565:
		return DPI_16_BIT_565_PACKED;
	case MIPI_DSI_FMT_RGB666:
		return DPI_18_BIT_ALIGNED;
	case MIPI_DSI_FMT_RGB666_PACKED:
		return DPI_18_BIT_PACKED;
	case MIPI_DSI_FMT_RGB888:
		return DPI_24_BIT;
	default:
		return DPI_24_BIT;
	}
}

static enum dpi_pixel_format nwl_dsi_get_dpi_pixel_format(
		enum mipi_dsi_pixel_format format)
{
	switch (format) {
	case MIPI_DSI_FMT_RGB565:
		return DPI_FMT_16_BIT;
	case MIPI_DSI_FMT_RGB666:
		return DPI_FMT_18_BIT;
	case MIPI_DSI_FMT_RGB666_PACKED:
		return DPI_FMT_18_BIT_LOOSELY_PACKED;
	case MIPI_DSI_FMT_RGB888:
		return DPI_FMT_24_BIT;
	default:
		return DPI_FMT_24_BIT;
	}
}

static unsigned long nwl_dsi_get_bit_clock(struct nwl_mipi_dsi *dsi,
		unsigned long pixclock)
{
	struct mipi_dsi_device *dsi_device = dsi->dsi_device;
	int bpp;
	u32 bus_fmt;
	struct drm_crtc *crtc = 0;

	if (dsi_device->lanes < 1 || dsi_device->lanes > 4)
		return 0;

	/* if CTRC updated the bus format, update dsi->format */
	if (dsi->bridge.encoder)
		crtc = dsi->bridge.encoder->crtc;
	if (crtc && crtc->mode.private_flags & 0x1) {
		bus_fmt = (crtc->mode.private_flags & 0x1FFFE) >> 1;
		/* propagate the format to the attached panel/bridge */
		dsi_device->format = mipi_dsi_format_from_bus_format(bus_fmt);
		/* clear bus format change indication*/
		crtc->mode.private_flags &= ~0x1;
	}

	bpp = mipi_dsi_pixel_format_to_bpp(dsi_device->format);

	return (pixclock * bpp) / dsi_device->lanes;
}

static void nwl_dsi_config_host(struct nwl_mipi_dsi *dsi)
{
	struct mipi_dsi_device *dsi_device = dsi->dsi_device;

	if (dsi_device->lanes < 1 || dsi_device->lanes > 4)
		return;

	nwl_dsi_write(dsi, CFG_NUM_LANES, dsi_device->lanes - 1);

	if (dsi_device->mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS)
		nwl_dsi_write(dsi, CFG_NONCONTINUOUS_CLK, 0x01);
	else
		nwl_dsi_write(dsi, CFG_NONCONTINUOUS_CLK, 0x00);

	if (dsi_device->mode_flags & MIPI_DSI_MODE_EOT_PACKET)
		nwl_dsi_write(dsi, CFG_AUTOINSERT_EOTP, 0x00);
	else
		nwl_dsi_write(dsi, CFG_AUTOINSERT_EOTP, 0x01);

	nwl_dsi_write(dsi, CFG_T_PRE, 0x01);
	nwl_dsi_write(dsi, CFG_T_POST, 0x34);
	nwl_dsi_write(dsi, CFG_TX_GAP, 0x0D);
	nwl_dsi_write(dsi, CFG_EXTRA_CMDS_AFTER_EOTP, 0x00);
	nwl_dsi_write(dsi, CFG_HTX_TO_COUNT, 0x00);
	nwl_dsi_write(dsi, CFG_LRX_H_TO_COUNT, 0x00);
	nwl_dsi_write(dsi, CFG_BTA_H_TO_COUNT, 0x00);
	nwl_dsi_write(dsi, CFG_TWAKEUP, 0x3a98);
}

static void nwl_dsi_config_dpi(struct nwl_mipi_dsi *dsi)
{
	struct device *dev = dsi->dev;
	struct mipi_dsi_device *dsi_device = dsi->dsi_device;
	struct videomode vm;
	enum dpi_pixel_format pixel_format =
		nwl_dsi_get_dpi_pixel_format(dsi_device->format);
	enum dpi_interface_color_coding color_coding =
		nwl_dsi_get_dpi_interface_color_coding(dsi_device->format);
	bool burst_mode;

	drm_display_mode_to_videomode(dsi->curr_mode, &vm);

	nwl_dsi_write(dsi, INTERFACE_COLOR_CODING, color_coding);
	nwl_dsi_write(dsi, PIXEL_FORMAT, pixel_format);
	DRM_DEV_DEBUG_DRIVER(dev, "DSI format is: %d (CC=%d, PF=%d)\n",
			dsi_device->format, color_coding, pixel_format);

	/*TODO: need to make polarity configurable */
	nwl_dsi_write(dsi, VSYNC_POLARITY, 0x00);
	nwl_dsi_write(dsi, HSYNC_POLARITY, 0x00);

	burst_mode = (dsi_device->mode_flags & MIPI_DSI_MODE_VIDEO_BURST) &&
		!(dsi_device->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE);

	if (burst_mode) {
		nwl_dsi_write(dsi, VIDEO_MODE, 0x2);
		nwl_dsi_write(dsi, PIXEL_FIFO_SEND_LEVEL, 256);
	} else {
		nwl_dsi_write(dsi, VIDEO_MODE, 0x0);
		nwl_dsi_write(dsi, PIXEL_FIFO_SEND_LEVEL, vm.hactive);
	}

	nwl_dsi_write(dsi, HFP, vm.hfront_porch);
	nwl_dsi_write(dsi, HBP, vm.hback_porch);
	nwl_dsi_write(dsi, HSA, vm.hsync_len);

	nwl_dsi_write(dsi, ENABLE_MULT_PKTS, 0x0);
	nwl_dsi_write(dsi, BLLP_MODE, 0x1);
	nwl_dsi_write(dsi, ENABLE_MULT_PKTS, 0x0);
	nwl_dsi_write(dsi, USE_NULL_PKT_BLLP, 0x0);
	nwl_dsi_write(dsi, VC, 0x0);

	nwl_dsi_write(dsi, PIXEL_PAYLOAD_SIZE, vm.hactive);
	nwl_dsi_write(dsi, VACTIVE, vm.vactive);
	nwl_dsi_write(dsi, VBP, vm.vback_porch);
	nwl_dsi_write(dsi, VFP, vm.vfront_porch);
}

static void nwl_dsi_enable_clocks(struct nwl_mipi_dsi *dsi, u32 clks)
{
	struct device *dev = dsi->dev;
	unsigned long rate;

	if (clks & CLK_PHY_REF && !dsi->phy_ref.enabled) {
		if (!IS_ERR_OR_NULL(dsi->phy_parent.clk))
			clk_set_parent(dsi->phy_ref_sel.clk,
				       dsi->phy_parent.clk);
		clk_prepare_enable(dsi->phy_ref.clk);
		dsi->phy_ref.enabled = true;
		rate = clk_get_rate(dsi->phy_ref.clk);
		DRM_DEV_DEBUG_DRIVER(dev, "Enabled phy_ref clk (rate=%lu)\n",
				     rate);
	}

	if (clks & CLK_RX_ESC && !dsi->rx_esc.enabled) {
		if (!IS_ERR_OR_NULL(dsi->phy_parent.clk))
			clk_set_parent(dsi->rx_esc_sel.clk,
				       dsi->phy_parent.clk);
		clk_set_rate(dsi->rx_esc.clk, dsi->rx_esc.rate);
		clk_prepare_enable(dsi->rx_esc.clk);
		dsi->rx_esc.enabled = true;
		rate = clk_get_rate(dsi->rx_esc.clk);
		DRM_DEV_DEBUG_DRIVER(dev, "Enabled rx_esc clk (rate=%lu)\n",
				     rate);
	}

	if (clks & CLK_TX_ESC && !dsi->tx_esc.enabled) {
		if (!IS_ERR_OR_NULL(dsi->phy_parent.clk))
			clk_set_parent(dsi->tx_esc_sel.clk,
				       dsi->phy_parent.clk);
		clk_set_rate(dsi->tx_esc.clk, dsi->tx_esc.rate);
		clk_prepare_enable(dsi->tx_esc.clk);
		dsi->tx_esc.enabled = true;
		rate = clk_get_rate(dsi->tx_esc.clk);
		DRM_DEV_DEBUG_DRIVER(dev, "Enabled tx_esc clk (rate=%lu)\n",
				     rate);
	}
}

static void nwl_dsi_disable_clocks(struct nwl_mipi_dsi *dsi, u32 clks)
{
	struct device *dev = dsi->dev;

	if (clks & CLK_PHY_REF && dsi->phy_ref.enabled) {
		clk_disable_unprepare(dsi->phy_ref.clk);
		dsi->phy_ref.enabled = false;
		DRM_DEV_DEBUG_DRIVER(dev, "Disabled phy_ref clk\n");
	}

	if (clks & CLK_RX_ESC && dsi->rx_esc.enabled) {
		clk_disable_unprepare(dsi->rx_esc.clk);
		dsi->rx_esc.enabled = false;
		DRM_DEV_DEBUG_DRIVER(dev, "Disabled rx_esc clk\n");
	}

	if (clks & CLK_TX_ESC && dsi->tx_esc.enabled) {
		clk_disable_unprepare(dsi->tx_esc.clk);
		dsi->tx_esc.enabled = false;
		DRM_DEV_DEBUG_DRIVER(dev, "Disabled tx_esc clk\n");
	}
}

static void nwl_dsi_init_interrupts(struct nwl_mipi_dsi *dsi)
{
	u32 irq_enable;

	nwl_dsi_write(dsi, IRQ_MASK, 0xffffffff);
	nwl_dsi_write(dsi, IRQ_MASK2, 0x7);

	irq_enable = ~(u32)(TX_PKT_DONE_MASK |
			RX_PKT_HDR_RCVD_MASK);

	nwl_dsi_write(dsi, IRQ_MASK, irq_enable);
}

/*
 * This function will try the required phy speed for current mode
 * If the phy speed can be achieved, the phy will save the speed
 * configuration
 */
static struct mode_config *nwl_dsi_mode_probe(struct nwl_mipi_dsi *dsi,
			    const struct drm_display_mode *mode)
{
	struct device *dev = dsi->dev;
	struct mode_config *config;
	unsigned long pixclock = mode->clock * 1000;
	unsigned long bit_clk = 0;
	u32 phyref_rate = 0, lanes = dsi->lanes;
	size_t i = 0, num_rates = ARRAY_SIZE(phyref_rates);
	int ret = 0;

	list_for_each_entry(config, &dsi->valid_modes, list)
		if (config->pixclock == pixclock)
			return config;

	while (i < num_rates) {
		bit_clk = nwl_dsi_get_bit_clock(dsi, pixclock);
		phyref_rate = phyref_rates[i];
		ret = mixel_phy_mipi_set_phy_speed(dsi->phy,
			bit_clk,
			phyref_rate,
			false);

		/* Pick the first non-failing rate */
		if (!ret)
			break;

		/* Reached the end of phyref_rates, try another lane config */
		if ((i++ == num_rates - 1) && (--lanes > 1)) {
			i = 0;
			continue;
		}
	}

	if (ret < 0) {
		DRM_DEV_DEBUG_DRIVER(dev,
			"Cannot setup PHY for mode: %ux%u @%d kHz\n",
			mode->hdisplay,
			mode->vdisplay,
			mode->clock);
		DRM_DEV_DEBUG_DRIVER(dev, "phy_ref clk: %u, bit clk: %lu\n",
			phyref_rate, bit_clk);

		return NULL;
	}

	config = devm_kzalloc(dsi->dev, sizeof(struct mode_config), GFP_KERNEL);
	config->pixclock = pixclock;
	config->lanes = lanes;
	config->bitclock = bit_clk;
	config->phyref_rate = phyref_rate;
	list_add(&config->list, &dsi->valid_modes);

	return config;
}

static enum drm_mode_status nwl_dsi_bridge_mode_valid(struct drm_bridge *bridge,
			   const struct drm_display_mode *mode)
{
	struct nwl_mipi_dsi *dsi = bridge->driver_private;
	size_t i, num_modes = ARRAY_SIZE(valid_clocks);
	bool clock_ok = false;

	DRM_DEV_DEBUG_DRIVER(dsi->dev, "Validating mode:");
	drm_mode_debug_printmodeline(mode);

	for (i = 0; i < num_modes; i++)
		if (mode->clock == valid_clocks[i]) {
			clock_ok = true;
			break;
		}

	if (!clock_ok)
		return MODE_NOCLOCK;

	if (!nwl_dsi_mode_probe(dsi, mode))
		return MODE_NOCLOCK;

	return MODE_OK;
}

static bool nwl_dsi_bridge_mode_fixup(struct drm_bridge *bridge,
			   const struct drm_display_mode *mode,
			   struct drm_display_mode *adjusted)
{
	struct nwl_mipi_dsi *dsi = bridge->driver_private;
	struct mode_config *config;

	DRM_DEV_DEBUG_DRIVER(dsi->dev, "Fixup mode:\n");
	drm_mode_debug_printmodeline(adjusted);

	config = nwl_dsi_mode_probe(dsi, adjusted);
	if (!config)
		return false;

	DRM_DEV_DEBUG_DRIVER(dsi->dev, "lanes=%u, data_rate=%lu\n",
			     config->lanes, config->bitclock);
	if (config->lanes < 1 || config->lanes > 4)
		return false;

	/* Max data rate for this controller is 1.5Gbps */
	if (config->bitclock > 1500000000)
		return false;

	return true;
}

static void nwl_dsi_bridge_mode_set(struct drm_bridge *bridge,
				     struct drm_display_mode *mode,
				     struct drm_display_mode *adjusted)
{
	struct nwl_mipi_dsi *dsi = bridge->driver_private;
	struct mode_config *config;
	u32 actual_phy_rate;

	DRM_DEV_DEBUG_DRIVER(dsi->dev, "Setting mode:\n");
	drm_mode_debug_printmodeline(adjusted);

	config = nwl_dsi_mode_probe(dsi, adjusted);
	/* New mode? This should NOT happen */
	if (!config) {
		DRM_DEV_ERROR(dsi->dev, "Unsupported mode provided:\n");
		drm_mode_debug_printmodeline(adjusted);
		return;
	}

	mixel_phy_mipi_set_phy_speed(dsi->phy,
			config->bitclock,
			config->phyref_rate,
			false);
	clk_set_rate(dsi->phy_ref.clk, config->phyref_rate);
	actual_phy_rate = clk_get_rate(dsi->phy_ref.clk);
	dsi->dsi_device->lanes = config->lanes;
	DRM_DEV_DEBUG_DRIVER(dsi->dev,
			     "Using phy_ref rate: %u (actual: %u), bitclock: %lu, lanes: %u\n",
			     config->phyref_rate, actual_phy_rate,
			     config->bitclock, config->lanes);

	dsi->curr_mode = drm_mode_duplicate(bridge->dev, adjusted);
}

static int nwl_dsi_host_attach(struct mipi_dsi_host *host,
			       struct mipi_dsi_device *device)
{
	struct nwl_mipi_dsi *dsi = container_of(host,
						struct nwl_mipi_dsi,
						host);
	struct device *dev = dsi->dev;

	DRM_DEV_INFO(dev, "lanes=%u, format=0x%x flags=0x%lx\n",
		     device->lanes, device->format, device->mode_flags);

	if (device->lanes < 1 || device->lanes > 4)
		return -EINVAL;

	/*
	 * Someone has attached to us; it could be a panel or another bridge.
	 * Check to is if this is a panel or not.
	 */
	if (!dsi->next_bridge ||
	    device->dev.of_node != dsi->next_bridge->of_node)
		dsi->panel = of_drm_find_panel(device->dev.of_node);

	/*
	 * Bridge has priority in front of panel.
	 * Since the panel driver cannot tell if there is a physical
	 * panel connected, we'll asume that there is no physical panel if there
	 * is a bridge registered.
	 */
	if (dsi->next_bridge &&
	    device->dev.of_node != NULL &&
	    device->dev.of_node != dsi->next_bridge->of_node) {
		dsi->panel = NULL;
		return -EPERM;
	}

	if (dsi->panel)
		DRM_DEV_DEBUG_DRIVER(dsi->dev, "Panel attached\n");
	else
		DRM_DEV_DEBUG_DRIVER(dsi->dev, "Bridge attached\n");

	dsi->dsi_device = device;
	dsi->lanes = device->lanes;

	/*
	 * If this happened right before a mode_set, it means that our
	 * bridge/panel doesn't like the current mode parameters and changed
	 * something on the dsi_device (lanes or format). In this case, we have
	 * to reconfigure the phy.
	 */
	if (dsi->curr_mode) {
		unsigned long pixclock = dsi->curr_mode->clock * 1000;
		struct mode_config *config;

		DRM_DEV_DEBUG_DRIVER(dsi->dev, "Re-setting mode:\n");
		drm_mode_debug_printmodeline(dsi->curr_mode);
		drm_mode_destroy(dsi->bridge.dev, dsi->curr_mode);
		list_for_each_entry(config, &dsi->valid_modes, list)
			if (config->pixclock == pixclock)
				break;

		if (device->lanes != config->lanes)
			return 0;

		clk_set_rate(dsi->phy_ref.clk, config->phyref_rate);
		device->lanes = config->lanes;
		DRM_DEV_DEBUG_DRIVER(dsi->dev,
			"Using phy_ref rate: %d (actual: %ld), "
			"bitclock: %lu, lanes: %d\n",
			config->phyref_rate, clk_get_rate(dsi->phy_ref.clk),
			config->bitclock, config->lanes);
	}

	if (dsi->connector.dev)
		drm_helper_hpd_irq_event(dsi->connector.dev);

	return 0;
}

static int nwl_dsi_host_detach(struct mipi_dsi_host *host,
			       struct mipi_dsi_device *device)
{
	struct nwl_mipi_dsi *dsi = container_of(host,
						struct nwl_mipi_dsi,
						host);
	if (dsi->panel)
		dsi->panel = NULL;

	if (dsi->connector.dev)
		drm_helper_hpd_irq_event(dsi->connector.dev);

	dsi->dsi_device = NULL;

	return 0;
}

static void nwl_dsi_print_error(struct device *dev, u16 error)
{
	DRM_DEV_DEBUG_DRIVER(dev, "DSI Error Register (detailed report):\n");
	if (error & BIT(0))
		DRM_DEV_DEBUG_DRIVER(dev,
			"SoT Error\n");
	if (error & BIT(1))
		DRM_DEV_DEBUG_DRIVER(dev,
			"SoT Sync Error\n");
	if (error & BIT(2))
		DRM_DEV_DEBUG_DRIVER(dev,
			"EoT Sync Error\n");
	if (error & BIT(3))
		DRM_DEV_DEBUG_DRIVER(dev,
			"Escape Mode Entry Command Error\n");
	if (error & BIT(4))
		DRM_DEV_DEBUG_DRIVER(dev,
			"Low-Power Transmit Sync Error\n");
	if (error & BIT(5))
		DRM_DEV_DEBUG_DRIVER(dev,
			"Peripheral Timeout Error\n");
	if (error & BIT(6))
		DRM_DEV_DEBUG_DRIVER(dev,
			"False Control Error\n");
	if (error & BIT(7))
		DRM_DEV_DEBUG_DRIVER(dev,
			"Contention Detected\n");
	if (error & BIT(8))
		DRM_DEV_DEBUG_DRIVER(dev,
			"ECC Error, single-bit (detected and corrected)\n");
	if (error & BIT(9))
		DRM_DEV_DEBUG_DRIVER(dev,
			"ECC Error, multi-bit (detected, not corrected)\n");
	if (error & BIT(10))
		DRM_DEV_DEBUG_DRIVER(dev,
			"Checksum Error (long packet only)\n");
	if (error & BIT(11))
		DRM_DEV_DEBUG_DRIVER(dev,
			"DSI Data Type Not Recognized\n");
	if (error & BIT(12))
		DRM_DEV_DEBUG_DRIVER(dev,
			"DSI VC ID Invalid\n");
	if (error & BIT(13))
		DRM_DEV_DEBUG_DRIVER(dev,
			"Invalid Transmission Length\n");
	/* BIT(14) is reserved */
	if (error & BIT(15))
		DRM_DEV_DEBUG_DRIVER(dev,
			"DSI Protocol Violation\n");
}

static bool nwl_dsi_read_packet(struct nwl_mipi_dsi *dsi, u32 status)
{
	struct device *dev = dsi->dev;
	struct mipi_dsi_transfer *xfer = dsi->xfer;
	u8 *payload = xfer->msg->rx_buf;
	u32 val;
	u16 word_count;
	u8 channel;
	u8 data_type;

	xfer->status = 0;

	if (xfer->rx_word_count == 0) {
		if (!(status & RX_PKT_HDR_RCVD))
			return false;
		/* Get the RX header and parse it */
		val = nwl_dsi_read(dsi, RX_PKT_HEADER);
		word_count = WC(val);
		channel	= RX_VC(val);
		data_type = RX_DT(val);

		if (channel != xfer->msg->channel) {
			DRM_DEV_ERROR(dev,
				"[%02X] Channel missmatch (%u != %u)\n",
				 xfer->cmd, channel, xfer->msg->channel);
			return true;
		}

		switch (data_type) {
		case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
			/* Fall through */
		case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
			if (xfer->msg->rx_len > 1) {
				/* read second byte */
				payload[1] = word_count >> 8;
				++xfer->rx_len;
			}
			/* Fall through */
		case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
			/* Fall through */
		case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
			if (xfer->msg->rx_len > 0) {
				/* read first byte */
				payload[0] = word_count & 0xff;
				++xfer->rx_len;
			}
			xfer->status = xfer->rx_len;
			return true;
		case MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT:
			word_count &= 0xff;
			DRM_DEV_ERROR(dev,
				"[%02X] DSI error report: 0x%02x\n",
				xfer->cmd, word_count);
			nwl_dsi_print_error(dev, word_count);
			xfer->status = -EPROTO;
			return true;

		}

		if (word_count > xfer->msg->rx_len) {
			DRM_DEV_ERROR(dev,
				"[%02X] Receive buffer too small: %lu (< %u)\n",
				 xfer->cmd,
				 xfer->msg->rx_len,
				 word_count);
			return true;
		}

		xfer->rx_word_count = word_count;
	} else {
		/* Set word_count from previous header read */
		word_count = xfer->rx_word_count;
	}

	/* If RX payload is not yet received, wait for it */
	if (!(status & RX_PKT_PAYLOAD_DATA_RCVD))
		return false;

	/* Read the RX payload */
	while (word_count >= 4) {
		val = nwl_dsi_read(dsi, RX_PAYLOAD);
		payload[0] = (val >>  0) & 0xff;
		payload[1] = (val >>  8) & 0xff;
		payload[2] = (val >> 16) & 0xff;
		payload[3] = (val >> 24) & 0xff;
		payload += 4;
		xfer->rx_len += 4;
		word_count -= 4;
	}

	if (word_count > 0) {
		val = nwl_dsi_read(dsi, RX_PAYLOAD);
		switch (word_count) {
		case 3:
			payload[2] = (val >> 16) & 0xff;
			++xfer->rx_len;
			/* Fall through */
		case 2:
			payload[1] = (val >>  8) & 0xff;
			++xfer->rx_len;
			/* Fall through */
		case 1:
			payload[0] = (val >>  0) & 0xff;
			++xfer->rx_len;
			break;
		}
	}

	xfer->status = xfer->rx_len;

	return true;
}

static void nwl_dsi_finish_transmission(struct nwl_mipi_dsi *dsi, u32 status)
{
	struct mipi_dsi_transfer *xfer = dsi->xfer;
	bool end_packet = false;

	if (!xfer)
		return;

	if (xfer->direction == DSI_PACKET_SEND && status & TX_PKT_DONE) {
		xfer->status = xfer->tx_len;
		end_packet = true;
	} else if (status & DPHY_DIRECTION && status & RX_PKT_HDR_RCVD)
		end_packet = nwl_dsi_read_packet(dsi, status);

	if (end_packet)
		complete(&xfer->completed);
}

static void nwl_dsi_begin_transmission(struct nwl_mipi_dsi *dsi)
{
	struct mipi_dsi_transfer *xfer = dsi->xfer;
	struct mipi_dsi_packet *pkt = &xfer->packet;
	const u8 *payload;
	size_t length;
	u16 word_count;
	u8 lp_mode;
	u32 val;

	/* Send the payload, if any */
	/* TODO: Need to check the TX FIFO overflow */
	length = pkt->payload_length;
	payload = pkt->payload;

	while (length >= 4) {
		val = get_unaligned_le32(payload);
		nwl_dsi_write(dsi, TX_PAYLOAD, val);
		payload += 4;
		length -= 4;
	}
	/* Send the rest of the payload */
	val = 0;
	switch (length) {
	case 3:
		val |= payload[2] << 16;
		/* Fall through */
	case 2:
		val |= payload[1] << 8;
		/* Fall through */
	case 1:
		val |= payload[0];
		nwl_dsi_write(dsi, TX_PAYLOAD, val);
		break;
	}
	xfer->tx_len = length;

	/*
	 * Now, send the header
	 * header structure is:
	 * header[0] = Virtual Channel + Data Type
	 * header[1] = Word Count LSB
	 * header[2] = Word Count MSB
	 */
	word_count = pkt->header[1] | (pkt->header[2] << 8);
	lp_mode = (xfer->msg->flags & MIPI_DSI_MSG_USE_LPM)?0:1;
	val = WC(word_count) |
		TX_VC(xfer->msg->channel) |
		TX_DT(xfer->msg->type) |
		HS_SEL(lp_mode) |
		BTA_TX(xfer->need_bta);
	nwl_dsi_write(dsi, PKT_CONTROL, val);

	/* Send packet command */
	nwl_dsi_write(dsi, SEND_PACKET, 0x1);
}

static ssize_t nwl_dsi_host_transfer(struct mipi_dsi_host *host,
				     const struct mipi_dsi_msg *msg)
{
	struct nwl_mipi_dsi *dsi = container_of(host,
						struct nwl_mipi_dsi,
						host);
	struct mipi_dsi_transfer xfer;
	ssize_t ret = 0;

	/* Create packet to be sent */
	dsi->xfer = &xfer;
	ret = mipi_dsi_create_packet(&xfer.packet, msg);
	if (ret < 0) {
		dsi->xfer = NULL;
		return ret;
	}

	if ((msg->type & MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM ||
	    msg->type & MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM ||
	    msg->type & MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM ||
	    msg->type & MIPI_DSI_DCS_READ) &&
	    msg->rx_len > 0 &&
	    msg->rx_buf != NULL)
		xfer.direction = DSI_PACKET_RECEIVE;
	else
		xfer.direction = DSI_PACKET_SEND;

	xfer.need_bta = (xfer.direction == DSI_PACKET_RECEIVE);
	xfer.need_bta |= (msg->flags & MIPI_DSI_MSG_REQ_ACK)?1:0;
	xfer.msg = msg;
	xfer.status = -ETIMEDOUT;
	xfer.rx_word_count = 0;
	xfer.rx_len = 0;
	xfer.cmd = 0x00;
	if (msg->tx_len > 0)
		xfer.cmd = ((u8 *)(msg->tx_buf))[0];
	init_completion(&xfer.completed);

	nwl_dsi_enable_clocks(dsi, CLK_RX_ESC);

	/* Initiate the DSI packet transmision */
	nwl_dsi_begin_transmission(dsi);

	wait_for_completion_timeout(&xfer.completed, MIPI_FIFO_TIMEOUT);

	ret = xfer.status;
	if (xfer.status == -ETIMEDOUT)
		DRM_DEV_ERROR(host->dev, "[%02X] DSI transfer timed out\n",
			xfer.cmd);

	nwl_dsi_disable_clocks(dsi, CLK_RX_ESC);

	return ret;
}

static const struct mipi_dsi_host_ops nwl_dsi_host_ops = {
	.attach = nwl_dsi_host_attach,
	.detach = nwl_dsi_host_detach,
	.transfer = nwl_dsi_host_transfer,
};

static irqreturn_t nwl_dsi_irq_handler(int irq, void *data)
{
	u32 irq_status;
	struct nwl_mipi_dsi *dsi = data;

	irq_status = nwl_dsi_read(dsi, IRQ_STATUS);

	if (irq_status & TX_PKT_DONE ||
	    irq_status & RX_PKT_HDR_RCVD ||
	    irq_status & RX_PKT_PAYLOAD_DATA_RCVD)
		nwl_dsi_finish_transmission(dsi, irq_status);

	return IRQ_HANDLED;
}

static enum drm_connector_status nwl_dsi_connector_detect(
	struct drm_connector *connector, bool force)
{
	struct nwl_mipi_dsi *dsi = container_of(connector,
						struct nwl_mipi_dsi,
						connector);

	if (dsi->panel)
		return connector_status_connected;

	return connector_status_unknown;
}

static int nwl_dsi_connector_get_modes(struct drm_connector *connector)
{
	struct nwl_mipi_dsi *dsi = container_of(connector,
						struct nwl_mipi_dsi,
						connector);
	int num_modes = 0;
	struct drm_display_mode *mode;

	DRM_DEV_DEBUG_DRIVER(dsi->dev, "\n");
	if (dsi->panel)
		num_modes = drm_panel_get_modes(dsi->panel);

	/*
	 * We need to inform the CRTC about the actual bit clock that we need
	 * for each mode
	 */
	list_for_each_entry(mode, &connector->probed_modes, head) {
		struct mode_config *config;
		u32 phy_rate;

		config = nwl_dsi_mode_probe(dsi, mode);
		/* Unsupported mode */
		if (!config)
			continue;

		/* Actual pixel clock that should be used by CRTC */
		phy_rate = config->phyref_rate / 1000;
		mode->crtc_clock = phy_rate * (mode->clock / phy_rate);
	}

	return num_modes;
}

static const struct drm_connector_funcs nwl_dsi_connector_funcs = {
	.detect = nwl_dsi_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_connector_helper_funcs
	nwl_dsi_connector_helper_funcs = {
	.get_modes = nwl_dsi_connector_get_modes,
};

static int nwl_dsi_create_connector(struct drm_device *drm,
				    struct nwl_mipi_dsi *dsi)
{
	struct device *dev = dsi->dev;
	int ret;

	ret = drm_connector_init(drm, &dsi->connector,
				 &nwl_dsi_connector_funcs,
				 DRM_MODE_CONNECTOR_DSI);
	if (ret) {
		DRM_DEV_ERROR(dev, "Failed to init drm connector: %d\n", ret);
		return ret;
	}

	drm_connector_helper_add(&dsi->connector,
				 &nwl_dsi_connector_helper_funcs);

	dsi->connector.dpms = DRM_MODE_DPMS_OFF;
	drm_mode_connector_attach_encoder(&dsi->connector, dsi->bridge.encoder);

	ret = drm_panel_attach(dsi->panel, &dsi->connector);
	if (ret) {
		DRM_DEV_ERROR(dev, "Failed to attach panel: %d\n", ret);
		drm_connector_cleanup(&dsi->connector);
		return ret;
	}

	return 0;
}

static int nwl_dsi_bridge_attach(struct drm_bridge *bridge)
{
	struct nwl_mipi_dsi *dsi = bridge->driver_private;
	struct device *dev = dsi->dev;
	struct drm_encoder *encoder = bridge->encoder;
	struct device_node *np = dev->of_node;
	struct device_node *remote_node, *endpoint;

	int ret = 0;

	DRM_DEV_DEBUG_DRIVER(dsi->dev, "\n");
	if (!encoder) {
		DRM_DEV_ERROR(dev, "Parent encoder object not found\n");
		return -ENODEV;
	}

	dsi->host.ops = &nwl_dsi_host_ops;
	dsi->host.dev = dev;
	ret = mipi_dsi_host_register(&dsi->host);
	if (ret < 0) {
		dev_err(dev, "failed to register DSI host (%d)\n", ret);
		return ret;
	}

	endpoint = of_graph_get_next_endpoint(np, NULL);
	while (endpoint && !dsi->next_bridge) {
		remote_node = of_graph_get_remote_port_parent(endpoint);
		if (!remote_node) {
			DRM_DEV_ERROR(dev, "No endpoint found!\n");
			return -ENODEV;
		}

		dsi->next_bridge = of_drm_find_bridge(remote_node);
		ret = drm_bridge_attach(encoder, dsi->next_bridge, bridge);
		if (ret)
			dsi->next_bridge = NULL;
		of_node_put(remote_node);
		endpoint = of_graph_get_next_endpoint(np, endpoint);
	};

	/*
	 * Create the connector. If we have a bridge, attach it and let the
	 * bridge create the connector.
	 */
	if (dsi->panel)
		ret = nwl_dsi_create_connector(encoder->dev, dsi);
	else if (!dsi->next_bridge)
		ret = -ENODEV;

	return ret;
}

static void nwl_dsi_bridge_detach(struct drm_bridge *bridge)
{
	struct nwl_mipi_dsi *dsi = bridge->driver_private;

	DRM_DEV_DEBUG_DRIVER(dsi->dev, "\n");
	if (dsi->panel) {
		drm_panel_detach(dsi->panel);
		drm_connector_cleanup(&dsi->connector);
		dsi->panel = NULL;
	} else if (dsi->next_bridge) {
		dsi->next_bridge = NULL;
	}
	if (dsi->host.dev)
		mipi_dsi_host_unregister(&dsi->host);
}

static void nwl_dsi_bridge_enable(struct drm_bridge *bridge)
{
	struct nwl_mipi_dsi *dsi = bridge->driver_private;
	struct mipi_dsi_device *dsi_device = dsi->dsi_device;
	struct device *dev = dsi->dev;
	int ret;

	if (dsi->enabled || (!dsi->panel && !dsi->next_bridge))
		return;

	if (!dsi_device) {
		DRM_DEV_ERROR(dev, "Bridge not set up properly!\n");
		return;
	}

	pm_runtime_get_sync(dev);

	ret = devm_request_irq(dev, dsi->irq,
			       nwl_dsi_irq_handler, 0, IRQ_NAME, dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to request IRQ: %d (%d)\n",
			      dsi->irq, ret);
		return;
	}

	nwl_dsi_enable_clocks(dsi, CLK_PHY_REF | CLK_TX_ESC);

	phy_init(dsi->phy);

	ret = phy_power_on(dsi->phy);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to power on DPHY (%d)\n", ret);
		goto phy_err;
	}

	nwl_dsi_init_interrupts(dsi);
	nwl_dsi_config_dpi(dsi);

	if (dsi->panel && drm_panel_prepare(dsi->panel)) {
		DRM_DEV_ERROR(dev, "Failed to setup panel\n");
		goto prepare_err;
	}

	nwl_dsi_config_host(dsi);

	if (dsi->panel && drm_panel_enable(dsi->panel)) {
		DRM_DEV_ERROR(dev, "Failed to enable panel\n");
		drm_panel_unprepare(dsi->panel);
		goto enable_err;
	}

	if (dsi_device->mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS)
		nwl_dsi_write(dsi, CFG_NONCONTINUOUS_CLK, 0x00);

	dsi->enabled = true;

	return;

enable_err:
	drm_panel_unprepare(dsi->panel);

prepare_err:
	phy_power_off(dsi->phy);

phy_err:
	phy_exit(dsi->phy);
	nwl_dsi_disable_clocks(dsi, CLK_PHY_REF | CLK_TX_ESC);
	devm_free_irq(dev, dsi->irq, dsi);
}

static void nwl_dsi_bridge_disable(struct drm_bridge *bridge)
{
	struct nwl_mipi_dsi *dsi = bridge->driver_private;
	struct device *dev = dsi->dev;

	if (!dsi->enabled)
		return;

	if (dsi->panel) {
		if (drm_panel_disable(dsi->panel)) {
			DRM_DEV_ERROR(dev, "failed to disable panel\n");
			return;
		}
		drm_panel_unprepare(dsi->panel);
	}

	phy_power_off(dsi->phy);
	phy_exit(dsi->phy);

	if (!dsi->no_clk_reset)
		nwl_dsi_disable_clocks(dsi, CLK_PHY_REF | CLK_TX_ESC);

	devm_free_irq(dev, dsi->irq, dsi);

	pm_runtime_put_sync(dev);

	dsi->enabled = false;
}

static const struct drm_bridge_funcs nwl_dsi_bridge_funcs = {
	.enable = nwl_dsi_bridge_enable,
	.disable = nwl_dsi_bridge_disable,
	.mode_valid = nwl_dsi_bridge_mode_valid,
	.mode_fixup = nwl_dsi_bridge_mode_fixup,
	.mode_set = nwl_dsi_bridge_mode_set,
	.attach = nwl_dsi_bridge_attach,
	.detach = nwl_dsi_bridge_detach,
};

static int nwl_dsi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nwl_mipi_dsi *dsi;
	struct clk *clk;
	struct resource *res;
	int ret;
	u32 rx_clk = 0, tx_clk = 0;

	dsi = devm_kzalloc(dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return -ENOMEM;

	dsi->phy = devm_phy_get(dev, "dphy");
	if (IS_ERR(dsi->phy)) {
		ret = PTR_ERR(dsi->phy);
		dev_err(dev, "Could not get PHY (%d)\n", ret);
		return ret;
	}

	clk = devm_clk_get(dev, "phy_ref");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		dev_err(dev, "Failed to get phy_ref clock: %d\n", ret);
		return ret;
	}
	dsi->phy_ref.clk = clk;
	dsi->phy_ref.rate = clk_get_rate(clk);
	dsi->phy_ref.enabled = false;

	clk = devm_clk_get(dev, "rx_esc");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		dev_err(dev, "Failed to get rx_esc clock: %d\n", ret);
		return ret;
	}
	dsi->rx_esc.clk = clk;
	ret = of_property_read_u32(dev->of_node, "nwl,rx-esc-clock-rate",
				   &rx_clk);
	if (!ret) {
		dsi->rx_esc.rate = rx_clk;
		dev_info(dev, "Found nwl,rx-esc-clock-rate %ld\n",
			 dsi->rx_esc.rate);
	} else
		dsi->rx_esc.rate = clk_get_rate(clk);
	dsi->rx_esc.enabled = false;

	clk = devm_clk_get(dev, "tx_esc");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		dev_err(dev, "Failed to get tx_esc clock: %d\n", ret);
		return ret;
	}
	dsi->tx_esc.clk = clk;
	ret = of_property_read_u32(dev->of_node, "nwl,tx-esc-clock-rate",
				   &tx_clk);
	if (!ret) {
		dsi->tx_esc.rate = tx_clk;
		dev_info(dev, "Found nwl,tx-esc-clock-rate %ld\n",
			 dsi->tx_esc.rate);
	} else
		dsi->tx_esc.rate = clk_get_rate(clk);
	dsi->tx_esc.enabled = false;

	clk = devm_clk_get(dev, "phy_parent");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		dev_info(dev, "Failed to get optional phy_parent clock: %d\n",
			 ret);
	}

	dsi->phy_parent.clk = clk;
	dsi->phy_parent.enabled = false;

	clk = devm_clk_get(dev, "phy_ref_sel");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		dev_info(dev, "Failed to get optional phy_ref_sel clock: %d\n",
			 ret);
	}
	dsi->phy_ref_sel.clk = clk;
	dsi->phy_ref_sel.enabled = false;

	clk = devm_clk_get(dev, "rx_esc_sel");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		dev_info(dev, "Failed to get rx_esc_sel clock: %d\n", ret);
	}
	dsi->rx_esc_sel.clk = clk;
	dsi->rx_esc_sel.enabled = false;

	clk = devm_clk_get(dev, "tx_esc_sel");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		dev_info(dev, "Failed to get tx_esc_sel clock: %d\n", ret);
	}
	dsi->tx_esc_sel.clk = clk;
	dsi->tx_esc_sel.enabled = false;

	/* TX clk rate must be RX clk rate divided by 4 */
	if (dsi->tx_esc.rate != (dsi->rx_esc.rate / 4))
		dsi->tx_esc.rate = dsi->rx_esc.rate / 4;

	dsi->enabled = false;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EBUSY;

	dsi->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(dsi->base))
		return PTR_ERR(dsi->base);

	dsi->irq = platform_get_irq(pdev, 0);
	if (dsi->irq < 0) {
		DRM_DEV_ERROR(dev, "Failed to get device IRQ!\n");
		return -EINVAL;
	}

	pm_runtime_enable(dev);

	dsi->no_clk_reset = of_property_read_bool(dev->of_node, "no_clk_reset");

	dsi->dev = dev;
	platform_set_drvdata(pdev, dsi);

	dsi->bridge.driver_private = dsi;
	dsi->bridge.funcs = &nwl_dsi_bridge_funcs;
	dsi->bridge.of_node = dev->of_node;

	ret = drm_bridge_add(&dsi->bridge);
	if (ret < 0)
		dev_err(dev, "Failed to add nwl-dsi bridge (%d)\n", ret);

	INIT_LIST_HEAD(&dsi->valid_modes);

	return ret;
}

static int nwl_dsi_remove(struct platform_device *pdev)
{
	struct nwl_mipi_dsi *dsi = platform_get_drvdata(pdev);
	struct mode_config *config;
	struct list_head *pos, *tmp;


	drm_bridge_remove(&dsi->bridge);

	list_for_each_safe(pos, tmp, &dsi->valid_modes) {
		config = list_entry(pos, struct mode_config, list);
		list_del(pos);
		devm_kfree(dsi->dev, config);
	}

	pm_runtime_disable(&pdev->dev);

	return 0;
}

static const struct of_device_id nwl_dsi_dt_ids[] = {
	{ .compatible = "nwl,mipi-dsi" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nwl_dsi_dt_ids);

static struct platform_driver imx_nwl_dsi_driver = {
	.probe		= nwl_dsi_probe,
	.remove		= nwl_dsi_remove,
	.driver		= {
		.of_match_table = nwl_dsi_dt_ids,
		.name	= "nwl-mipi-dsi",
	},
};

module_platform_driver(imx_nwl_dsi_driver);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("NWL MIPI-DSI transmitter driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nwl-dsi");
