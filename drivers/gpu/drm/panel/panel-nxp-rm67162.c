// SPDX-License-Identifier: GPL-2.0
/*
 * Circular MIPI-DSI panel driver with RM67162 chip
 *
 * Copyright 2023 NXP
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

/* Panel specific color-format bits */
#define COL_FMT_16BPP 0x75
#define COL_FMT_18BPP 0x76
#define COL_FMT_24BPP 0x77

/* Write Manufacture Command Set Control */
#define WRMAUCCTR 0xFE

/* Manufacturer Command Set pages (CMD2) */
struct cmd_set_entry {
	u8 cmd;
	u8 param;
};

/*
 * There is no description in the Reference Manual about these commands.
 * We received them from vendor, so just use them as is.
 */
static const struct cmd_set_entry mcs_rm67162_400x392[] = { // 400x392
	{0xFE, 0x01}, {0x06, 0x62}, {0x0E, 0x80}, {0x0F, 0x80}, {0x10, 0x71},
	{0x13, 0x81}, {0x14, 0x81}, {0x15, 0x82}, {0x16, 0x82}, {0x18, 0x88},
	{0x19, 0x55}, {0x1A, 0x10}, {0x1C, 0x99}, {0x1D, 0x03}, {0x1E, 0x03},
	{0x1F, 0x03}, {0x20, 0x03}, {0x25, 0x03}, {0x26, 0x8D}, {0x2A, 0x03},
	{0x2B, 0x8D}, {0x36, 0x00}, {0x37, 0x10}, {0x3A, 0x00}, {0x3B, 0x00},
	{0x3D, 0x20}, {0x3F, 0x3A}, {0x40, 0x30}, {0x41, 0x30}, {0x42, 0x33},
	{0x43, 0x22}, {0x44, 0x11}, {0x45, 0x66}, {0x46, 0x55}, {0x47, 0x44},
	{0x4C, 0x33}, {0x4D, 0x22}, {0x4E, 0x11}, {0x4F, 0x66}, {0x50, 0x55},
	{0x51, 0x44}, {0x57, 0xB3}, {0x6B, 0x19}, {0x70, 0x55}, {0x74, 0x0C},
	/* VGMP/VGSP Voltage Control */
	{0xFE, 0x02}, {0x9B, 0x40}, {0x9C, 0x67}, {0x9D, 0x20},
	/* VGMP/VGSP Voltage Control */
	{0xFE, 0x03}, {0x9B, 0x40}, {0x9C, 0x67}, {0x9D, 0x20},
	/* VSR Command */
	{0xFE, 0x04}, {0x5D, 0x10},
	/* VSR1 Timing Set */
	{0xFE, 0x04}, {0x00, 0x8D}, {0x01, 0x00}, {0x02, 0x01}, {0x03, 0x01},
	{0x04, 0x10}, {0x05, 0x01}, {0x06, 0xA7}, {0x07, 0x20}, {0x08, 0x00},
	/* VSR2 Timing Set */
	{0xFE, 0x04}, {0x09, 0xC2}, {0x0A, 0x00}, {0x0B, 0x02}, {0x0C, 0x01},
	{0x0D, 0x40}, {0x0E, 0x06}, {0x0F, 0x01}, {0x10, 0xA7}, {0x11, 0x00},
	/* VSR3 Timing Set */
	{0xFE, 0x04}, {0x12, 0xC2}, {0x13, 0x00}, {0x14, 0x02}, {0x15, 0x01},
	{0x16, 0x40}, {0x17, 0x07}, {0x18, 0x01}, {0x19, 0xA7}, {0x1A, 0x00},
	/* VSR4 Timing Set */
	{0xFE, 0x04}, {0x1B, 0x82}, {0x1C, 0x00}, {0x1D, 0xFF}, {0x1E, 0x05},
	{0x1F, 0x60}, {0x20, 0x02}, {0x21, 0x01}, {0x22, 0x7C}, {0x23, 0x00},
	/* VSR5 Timing Set */
	{0xFE, 0x04}, {0x24, 0xC2}, {0x25, 0x00}, {0x26, 0x04}, {0x27, 0x02},
	{0x28, 0x70}, {0x29, 0x05}, {0x2A, 0x74}, {0x2B, 0x8D}, {0x2D, 0x00},
	/* VSR6 Timing Set */
	{0xFE, 0x04}, {0x2F, 0xC2}, {0x30, 0x00}, {0x31, 0x04}, {0x32, 0x02},
	{0x33, 0x70}, {0x34, 0x07}, {0x35, 0x74}, {0x36, 0x8D}, {0x37, 0x00},
	/* VSR Marping command  */
	{0xFE, 0x04}, {0x5E, 0x20}, {0x5F, 0x31}, {0x60, 0x54}, {0x61, 0x76},
	{0x62, 0x98},
	/* ELVSS -2.4V(RT4723). 0x15: RT4723. 0x01: RT4723B. 0x17: STAM1332. */
	{0xFE, 0x05}, {0x05, 0x15}, {0x2A, 0x04}, {0x91, 0x00},
	{0xFE, 0x00}, {0x35, 0x00}, /* TE enable. */
};

static const struct cmd_set_entry mcs_rm67162_400x400[] = {
	/* Page 3:GOA */
	{0xFE, 0x04},
	/* GOA SETTING */
	{0x00, 0xDC}, {0x01, 0x00}, {0x02, 0x02}, {0x03, 0x00}, {0x04, 0x00},
	{0x05, 0x03}, {0x06, 0x16}, {0x07, 0x13}, {0x08, 0x08}, {0x09, 0xDC},
	{0x0A, 0x00}, {0x0B, 0x02}, {0x0C, 0x00}, {0x0D, 0x00}, {0x0E, 0x02},
	{0x0F, 0x16}, {0x10, 0x18}, {0x11, 0x08}, {0x12, 0x92}, {0x13, 0x00},
	{0x14, 0x02}, {0x15, 0x05}, {0x16, 0x40}, {0x17, 0x03}, {0x18, 0x16},
	{0x19, 0xD7}, {0x1A, 0x01}, {0x1B, 0xDC}, {0x1C, 0x00}, {0x1D, 0x04},
	{0x1E, 0x00}, {0x1F, 0x00}, {0x20, 0x03}, {0x21, 0x16}, {0x22, 0x18},
	{0x23, 0x08}, {0x24, 0xDC}, {0x25, 0x00}, {0x26, 0x04}, {0x27, 0x00},
	{0x28, 0x00}, {0x29, 0x01}, {0x2A, 0x16}, {0x2B, 0x18}, {0x2D, 0x08},
	{0x4C, 0x99}, {0x4D, 0x00}, {0x4E, 0x00}, {0x4F, 0x00}, {0x50, 0x01},
	{0x51, 0x0A}, {0x52, 0x00}, {0x5A, 0xE4}, {0x5E, 0x77}, {0x5F, 0x77},
	{0x60, 0x34}, {0x61, 0x02}, {0x62, 0x81},
	/* Page 6 */
	{0xFE, 0x07}, {0x07, 0x4F},
	/* Page 0 */
	{0xFE, 0x01},
	/* Display Resolution Panel Option */
	{0x05, 0x15},
	/* DDVDH Charge Pump Control Normal Mode */
	{0x0E, 0x8B},
	/* DDVDH Charge Pump Control ldle Mode */
	{0x0F, 0x8B},
	/* DDVDH/VCL Regulator Enable */
	{0x10, 0x11},
	/* VCL Charge Pump Control Normal Mode */
	{0x11, 0xA2},
	/* VCL Charge Pump Control Idle Mode */
	{0x12, 0xA0},
	/* VGH Charge Pump Control ldle Mode */
	{0x14, 0xA1},
	/* VGL Charge Pump Control Normal Mode */
	{0x15, 0x82},
	/* VGHR Control */
	{0x18, 0x47},
	/* VGLR Control */
	{0x19, 0x36},
	/* VREFPN5 REGULATOR ENABLE */
	{0x1A, 0x10},
	/* VREFPN5 */
	{0x1C, 0x57},
	/* SWITCH EQ Control */
	{0x1D, 0x02},
	/* VGMP Control */
	{0x21, 0xF8},
	/* VGSP Control */
	{0x22, 0x90},
	/* VGMP / VGSP control */
	{0x23, 0x00},
	/* Low Frame Rate Control Normal Mode */
	{0x25, 0x03}, {0x26, 0x4a},
	/* Low Frame Rate Control Idle Mode */
	{0x2A, 0x03}, {0x2B, 0x4A}, {0x2D, 0x12}, {0x2F, 0x12}, {0x30, 0x45},
	/* Source Control */
	{0x37, 0x0C},
	/* Switch Timing Control */
	{0x3A, 0x00}, {0x3B, 0x20}, {0x3D, 0x0B}, {0x3F, 0x38}, {0x40, 0x0B},
	{0x41, 0x0B},
	/* Switch Output Selection */
	{0x42, 0x33}, {0x43, 0x66}, {0x44, 0x11}, {0x45, 0x44}, {0x46, 0x22},
	{0x47, 0x55}, {0x4C, 0x33}, {0x4D, 0x66}, {0x4E, 0x11}, {0x4f, 0x44},
	{0x50, 0x22}, {0x51, 0x55},
	/* Source Data Output Selection */
	{0x56, 0x11}, {0x58, 0x44}, {0x59, 0x22}, {0x5A, 0x55}, {0x5B, 0x33},
	{0x5C, 0x66}, {0x61, 0x11}, {0x62, 0x44}, {0x63, 0x22}, {0x64, 0x55},
	{0x65, 0x33}, {0x66, 0x66}, {0x6D, 0x90}, {0x6E, 0x40},
	/* Source Sequence 2 */
	{0x70, 0xA5},
	/* OVDD control */
	{0x72, 0x04},
	/* OVSS control */
	{0x73, 0x15},
	/* Page 9 */
	{0xFE, 0x0A}, {0x29, 0x10},
	/* Page 4 */
	{0xFE, 0x05},
	/* ELVSS -2.4V(RT4723). 0x15: RT4723. 0x01: RT4723B. 0x17: STAM1332. */
	{0x05, 0x15},
	/* enable TE. */
	{0xFE, 0x00}, {0x35, 0x00},
};

static const u32 nxp_bus_formats[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_RGB666_1X18,
	MEDIA_BUS_FMT_RGB565_1X16,
};

static const u32 nxp_bus_flags = DRM_BUS_FLAG_DE_LOW |
				  DRM_BUS_FLAG_PIXDATA_SAMPLE_POSEDGE;

struct rm67162_platform_data {
	const struct drm_display_mode *mode;
	const struct cmd_set_entry *cmds;
	int cmds_cnt;
};

struct nxp_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;

	struct gpio_desc *enable;
	struct gpio_desc *reset;
	struct backlight_device *backlight;

	struct regulator_bulk_data *supplies;
	unsigned int num_supplies;

	bool prepared;
	bool enabled;

	const struct rm67162_platform_data *pdata;
};

static const struct drm_display_mode default_mode_400x392 = {
	.clock = 12000,
	.hdisplay = 400,
	.hsync_start = 400 + 20,
	.hsync_end = 400 + 20 + 40,
	.htotal = 400 + 20 + 40 + 20,
	.vdisplay = 392,
	.vsync_start = 392 + 20,
	.vsync_end = 392 + 20 + 12,
	.vtotal = 392 + 20 + 12 + 4,
	.width_mm = 35,
	.height_mm = 35,
	.flags = DRM_MODE_FLAG_NHSYNC |
		 DRM_MODE_FLAG_NVSYNC,
};

static const struct drm_display_mode default_mode_400x400 = {
	.clock = 12000,
	.hdisplay = 400,
	.hsync_start = 400 + 20,
	.hsync_end = 400 + 20 + 40,
	.htotal = 400 + 20 + 40 + 20,
	.vdisplay = 400,
	.vsync_start = 400 + 20,
	.vsync_end = 400 + 20 + 12,
	.vtotal = 400 + 20 + 12 + 4,
	.width_mm = 35,
	.height_mm = 35,
	.flags = DRM_MODE_FLAG_NHSYNC |
		 DRM_MODE_FLAG_NVSYNC,
};

static inline struct nxp_panel *to_nxp_panel(struct drm_panel *panel)
{
	return container_of(panel, struct nxp_panel, panel);
}

static int nxp_panel_push_cmd_list(struct mipi_dsi_device *dsi,
				   struct cmd_set_entry const *cmd_set,
				   size_t count)
{
	size_t i;
	int ret = 0;

	for (i = 0; i < count; i++) {
		const struct cmd_set_entry *entry = cmd_set++;
		u8 buffer[2] = { entry->cmd, entry->param };

		ret = mipi_dsi_generic_write(dsi, &buffer, sizeof(buffer));
		if (ret < 0)
			return ret;
	}

	return ret;
};

static int color_format_from_dsi_format(enum mipi_dsi_pixel_format format)
{
	switch (format) {
	case MIPI_DSI_FMT_RGB565:
		return COL_FMT_16BPP;
	case MIPI_DSI_FMT_RGB666:
	case MIPI_DSI_FMT_RGB666_PACKED:
		return COL_FMT_18BPP;
	case MIPI_DSI_FMT_RGB888:
		return COL_FMT_24BPP;
	default:
		return COL_FMT_24BPP; /* for backward compatibility */
	}
};

static int nxp_panel_prepare(struct drm_panel *panel)
{
	struct nxp_panel *nxp = to_nxp_panel(panel);
	int ret;

	if (nxp->prepared)
		return 0;

	ret = regulator_bulk_enable(nxp->num_supplies, nxp->supplies);
	if (ret)
		return ret;

	if (nxp->enable)
		gpiod_set_value_cansleep(nxp->enable, 1);

	/* At lest 10ms needed between power-on and reset-out as RM specifies */
	usleep_range(10000, 12000);

	if (nxp->reset) {
		gpiod_set_value_cansleep(nxp->reset, 0);
		/*
		 * 12ms delay after reset-out, as per manufacturer initalization
		 * sequence.
		 */
		msleep(12);
	}
	msleep(150);

	nxp->prepared = true;

	return 0;
}

static int nxp_panel_unprepare(struct drm_panel *panel)
{
	struct nxp_panel *nxp = to_nxp_panel(panel);
	int ret;

	if (!nxp->prepared)
		return 0;

	if (nxp->reset) {
		gpiod_set_value_cansleep(nxp->reset, 1);
		usleep_range(1000, 2000);
	}
	if (nxp->enable)
		gpiod_set_value_cansleep(nxp->enable, 0);

	ret = regulator_bulk_disable(nxp->num_supplies, nxp->supplies);
	if (ret)
		return ret;

	nxp->prepared = false;

	return 0;
}

static int nxp_panel_enable(struct drm_panel *panel)
{
	struct nxp_panel *nxp = to_nxp_panel(panel);
	struct mipi_dsi_device *dsi = nxp->dsi;
	struct device *dev = &dsi->dev;
	u8 dsi_mode;
	int color_format = color_format_from_dsi_format(dsi->format);
	int ret;

	if (nxp->enabled)
		return 0;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = nxp_panel_push_cmd_list(dsi, nxp->pdata->cmds, nxp->pdata->cmds_cnt);
	if (ret < 0) {
		dev_err(dev, "Failed to send MCS (%d)\n", ret);
		goto fail;
	}

	/* Select User Command Set table (CMD1) */
	ret = mipi_dsi_generic_write(dsi, (u8[]){ WRMAUCCTR, 0x00 }, 2);
	if (ret < 0)
		goto fail;

	/* Software reset */
	ret = mipi_dsi_dcs_soft_reset(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to do Software Reset (%d)\n", ret);
		goto fail;
	}

	usleep_range(15000, 17000);

	/* Set DSI mode */
	dsi_mode = (dsi->mode_flags & MIPI_DSI_MODE_VIDEO) ? 0x0B : 0x00;
	ret = mipi_dsi_generic_write(dsi, (u8[]){ 0xC2, dsi_mode }, 2);
	if (ret < 0) {
		dev_err(dev, "Failed to set DSI mode (%d)\n", ret);
		goto fail;
	}
	/* Set tear ON */
	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0) {
		dev_err(dev, "Failed to set tear ON (%d)\n", ret);
		goto fail;
	}

	/* Set tear scanline */
	ret = mipi_dsi_dcs_set_tear_scanline(dsi, 0x380);
	if (ret < 0) {
		dev_err(dev, "Failed to set tear scanline (%d)\n", ret);
		goto fail;
	}

	/* Set pixel format */
	ret = mipi_dsi_dcs_set_pixel_format(dsi, color_format);
	dev_dbg(dev, "Interface color format set to 0x%x\n", color_format);
	if (ret < 0) {
		dev_err(dev, "Failed to set pixel format (%d)\n", ret);
		goto fail;
	}
	msleep(50);
	/* Exit sleep mode */
	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to exit sleep mode (%d)\n", ret);
		goto fail;
	}

	msleep(150);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display ON (%d)\n", ret);
		goto fail;
	}

	backlight_enable(nxp->backlight);

	nxp->enabled = true;

	return 0;

fail:
	gpiod_set_value_cansleep(nxp->reset, 1);

	return ret;
}

static int nxp_panel_disable(struct drm_panel *panel)
{
	struct nxp_panel *nxp = to_nxp_panel(panel);
	struct mipi_dsi_device *dsi = nxp->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	if (!nxp->enabled)
		return 0;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	backlight_disable(nxp->backlight);

	usleep_range(10000, 12000);

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display OFF (%d)\n", ret);
		return ret;
	}

	usleep_range(5000, 10000);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to enter sleep mode (%d)\n", ret);
		return ret;
	}

	nxp->enabled = false;

	return 0;
}

static int nxp_panel_get_modes(struct drm_panel *panel,
			       struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	struct nxp_panel *nxp = to_nxp_panel(panel);

	mode = drm_mode_duplicate(connector->dev, nxp->pdata->mode);
	if (!mode) {
		dev_err(panel->dev, "failed to add mode %ux%u@%u\n",
			nxp->pdata->mode->hdisplay, nxp->pdata->mode->vdisplay,
			drm_mode_vrefresh(nxp->pdata->mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	connector->display_info.bus_flags = nxp_bus_flags;

	drm_display_info_set_bus_formats(&connector->display_info,
					 nxp_bus_formats,
					 ARRAY_SIZE(nxp_bus_formats));
	return 1;
}

static int nxp_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct nxp_panel *nxp = mipi_dsi_get_drvdata(dsi);
	u16 brightness;
	int ret;

	if (!nxp->prepared)
		return 0;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
	ret = mipi_dsi_dcs_get_display_brightness(dsi, &brightness);
	if (ret < 0)
		return ret;

	bl->props.brightness = brightness;

	return brightness & 0xff;
}

static int nxp_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct nxp_panel *nxp = mipi_dsi_get_drvdata(dsi);
	int ret = 0;

	if (!nxp->prepared)
		return 0;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
	ret = mipi_dsi_dcs_set_display_brightness(dsi, bl->props.brightness);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct backlight_ops nxp_bl_ops = {
	.update_status = nxp_bl_update_status,
	.get_brightness = nxp_bl_get_brightness,
};

static const struct drm_panel_funcs nxp_panel_funcs = {
	.prepare = nxp_panel_prepare,
	.unprepare = nxp_panel_unprepare,
	.enable = nxp_panel_enable,
	.disable = nxp_panel_disable,
	.get_modes = nxp_panel_get_modes,
};

static struct rm67162_platform_data panel_400x392 = {
	.mode = &default_mode_400x392,
	.cmds = mcs_rm67162_400x392,
	.cmds_cnt = ARRAY_SIZE(mcs_rm67162_400x392),
};
static struct rm67162_platform_data panel_400x400 = {
	.mode = &default_mode_400x400,
	.cmds = mcs_rm67162_400x400,
	.cmds_cnt = ARRAY_SIZE(mcs_rm67162_400x400),
};

static const struct of_device_id nxp_of_match[] = {
	{ .compatible = "nxp,rm67162", .data = &panel_400x392},
	{ .compatible = "usmp,rm67162", .data = &panel_400x400},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nxp_of_match);

static int nxp_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct of_device_id *of_id = of_match_device(nxp_of_match, dev);
	struct device_node *np = dev->of_node;
	struct nxp_panel *panel;
	struct backlight_properties bl_props;
	int ret;
	u32 video_mode;

	panel = devm_kzalloc(&dsi->dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, panel);

	panel->dsi = dsi;
	panel->pdata = of_id->data;

	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO_HSE;

	ret = of_property_read_u32(np, "video-mode", &video_mode);
	if (!ret) {
		switch (video_mode) {
		case 0:
			/* burst mode */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_BURST |
					   MIPI_DSI_MODE_VIDEO;
			break;
		case 1:
			/* non-burst mode with sync event */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO;
			break;
		case 2:
			/* non-burst mode with sync pulse */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
					   MIPI_DSI_MODE_VIDEO;
			break;
		case 3:
			/* command mode */
			dsi->mode_flags |= MIPI_DSI_CLOCK_NON_CONTINUOUS |
					   MIPI_DSI_MODE_VSYNC_FLUSH;
			break;
		default:
			dev_warn(dev, "invalid video mode %d\n", video_mode);
			break;
		}
	}

	ret = of_property_read_u32(np, "dsi-lanes", &dsi->lanes);
	if (ret) {
		dev_err(dev, "Failed to get dsi-lanes property (%d)\n", ret);
		return ret;
	}

	panel->enable = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(panel->enable)) {
		ret = PTR_ERR(panel->enable);
		dev_err(dev, "Failed to get enable gpio (%d)\n", ret);
		return ret;
	}
	panel->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(panel->reset)) {
		ret = PTR_ERR(panel->reset);
		dev_err(dev, "Failed to get reset gpio (%d)\n", ret);
		return ret;
	}
	gpiod_set_value_cansleep(panel->reset, 1);

	memset(&bl_props, 0, sizeof(bl_props));
	bl_props.type = BACKLIGHT_RAW;
	bl_props.brightness = 255;
	bl_props.max_brightness = 255;

	panel->backlight = devm_backlight_device_register(dev, dev_name(dev),
							  dev, dsi, &nxp_bl_ops,
							  &bl_props);
	if (IS_ERR(panel->backlight)) {
		ret = PTR_ERR(panel->backlight);
		dev_err(dev, "Failed to register backlight (%d)\n", ret);
		return ret;
	}

	drm_panel_init(&panel->panel, dev, &nxp_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	dev_set_drvdata(dev, panel);

	drm_panel_add(&panel->panel);
	ret = mipi_dsi_attach(dsi);
	if (ret)
		drm_panel_remove(&panel->panel);

	return ret;
}

static void nxp_panel_remove(struct mipi_dsi_device *dsi)
{
	struct nxp_panel *nxp = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret)
		dev_err(dev, "Failed to detach from host (%d)\n", ret);

	drm_panel_remove(&nxp->panel);
}

static void nxp_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct nxp_panel *nxp = mipi_dsi_get_drvdata(dsi);

	nxp_panel_disable(&nxp->panel);
	nxp_panel_unprepare(&nxp->panel);
}

static struct mipi_dsi_driver nxp_panel_driver = {
	.driver = {
		.name = "panel-rm67162",
		.of_match_table = nxp_of_match,
	},
	.probe = nxp_panel_probe,
	.remove = nxp_panel_remove,
	.shutdown = nxp_panel_shutdown,
};
module_mipi_dsi_driver(nxp_panel_driver);

MODULE_AUTHOR("Zhang Bo <bo.zhang@nxp.com>");
MODULE_DESCRIPTION("DRM Driver for RM67162 MIPI DSI panel");
MODULE_LICENSE("GPL v2");
