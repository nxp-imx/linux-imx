// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2022 NXP
 *
 * Based on drivers/gpu/drm/panel/panel-seiko-43wvf1g.c
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <video/display_timing.h>
#include <video/videomode.h>

#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_panel.h>

struct ontat_kd50g21_40nt_a1_panel_desc {
	const struct drm_display_mode *modes;
	unsigned int num_modes;
	const struct display_timing *timings;
	unsigned int num_timings;

	unsigned int bpc;

	/**
	 * @width: width (in millimeters) of the panel's active display area
	 * @height: height (in millimeters) of the panel's active display area
	 */
	struct {
		unsigned int width;
		unsigned int height;
	} size;

	u32 bus_format;
	u32 bus_flags;
};

struct ontat_kd50g21_40nt_a1_panel {
	struct drm_panel base;
	bool enabled;
	bool prepared;
	ktime_t prepared_time;
	ktime_t unprepared_time;
	const struct ontat_kd50g21_40nt_a1_panel_desc *desc;
	struct regulator *supply;
	struct gpio_desc *enable_gpio;
};

static inline struct ontat_kd50g21_40nt_a1_panel *
to_ontat_kd50g21_40nt_a1_panel(struct drm_panel *panel)
{
	return container_of(panel, struct ontat_kd50g21_40nt_a1_panel, base);
}

static int
ontat_kd50g21_40nt_a1_panel_get_fixed_modes(struct ontat_kd50g21_40nt_a1_panel *panel,
					    struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	unsigned int i, num = 0;

	if (!panel->desc)
		return 0;

	for (i = 0; i < panel->desc->num_timings; i++) {
		const struct display_timing *dt = &panel->desc->timings[i];
		struct videomode vm;

		videomode_from_timing(dt, &vm);
		mode = drm_mode_create(connector->dev);
		if (!mode) {
			dev_err(panel->base.dev, "failed to add mode %ux%u\n",
				dt->hactive.typ, dt->vactive.typ);
			continue;
		}

		drm_display_mode_from_videomode(&vm, mode);

		mode->type |= DRM_MODE_TYPE_DRIVER;

		if (panel->desc->num_timings == 1)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_probed_add(connector, mode);
		num++;
	}

	for (i = 0; i < panel->desc->num_modes; i++) {
		const struct drm_display_mode *m = &panel->desc->modes[i];

		mode = drm_mode_duplicate(connector->dev, m);
		if (!mode) {
			dev_err(panel->base.dev, "failed to add mode %ux%u@%u\n",
				m->hdisplay, m->vdisplay,
				drm_mode_vrefresh(m));
			continue;
		}

		mode->type |= DRM_MODE_TYPE_DRIVER;

		if (panel->desc->num_modes == 1)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_set_name(mode);

		drm_mode_probed_add(connector, mode);
		num++;
	}

	connector->display_info.bpc = panel->desc->bpc;
	connector->display_info.width_mm = panel->desc->size.width;
	connector->display_info.height_mm = panel->desc->size.height;
	if (panel->desc->bus_format)
		drm_display_info_set_bus_formats(&connector->display_info,
						 &panel->desc->bus_format, 1);
	connector->display_info.bus_flags = panel->desc->bus_flags;

	return num;
}

static void ontat_kd50g21_40nt_a1_panel_wait(ktime_t start_ktime,
					     unsigned int min_ms)
{
	ktime_t now_ktime, min_ktime;

	if (!min_ms)
		return;

	min_ktime = ktime_add(start_ktime, ms_to_ktime(min_ms));
	now_ktime = ktime_get();

	if (ktime_before(now_ktime, min_ktime))
		msleep(ktime_to_ms(ktime_sub(min_ktime, now_ktime)) + 1);
}

static int ontat_kd50g21_40nt_a1_panel_disable(struct drm_panel *panel)
{
	struct ontat_kd50g21_40nt_a1_panel *p = to_ontat_kd50g21_40nt_a1_panel(panel);

	if (!p->enabled)
		return 0;

	msleep(114);

	p->enabled = false;

	return 0;
}

static int ontat_kd50g21_40nt_a1_panel_unprepare(struct drm_panel *panel)
{
	struct ontat_kd50g21_40nt_a1_panel *p = to_ontat_kd50g21_40nt_a1_panel(panel);

	if (!p->prepared)
		return 0;

	gpiod_set_value_cansleep(p->enable_gpio, 0);
	regulator_disable(p->supply);
	p->unprepared_time = ktime_get();

	p->prepared = false;

	return 0;
}

static int ontat_kd50g21_40nt_a1_panel_prepare(struct drm_panel *panel)
{
	struct ontat_kd50g21_40nt_a1_panel *p = to_ontat_kd50g21_40nt_a1_panel(panel);
	int err;

	if (p->prepared)
		return 0;

	ontat_kd50g21_40nt_a1_panel_wait(p->unprepared_time, 17);

	err = regulator_enable(p->supply);
	if (err < 0) {
		dev_err(panel->dev, "failed to enable power supply: %d\n", err);
		return err;
	}

	gpiod_set_value_cansleep(p->enable_gpio, 1);

	msleep(36);

	p->prepared_time = ktime_get();

	p->prepared = true;

	return 0;
}

static int ontat_kd50g21_40nt_a1_panel_enable(struct drm_panel *panel)
{
	struct ontat_kd50g21_40nt_a1_panel *p = to_ontat_kd50g21_40nt_a1_panel(panel);

	if (p->enabled)
		return 0;

	msleep(163);

	ontat_kd50g21_40nt_a1_panel_wait(p->prepared_time, 0);

	p->enabled = true;

	return 0;
}

static int
ontat_kd50g21_40nt_a1_panel_get_modes(struct drm_panel *panel,
				      struct drm_connector *connector)
{
	struct ontat_kd50g21_40nt_a1_panel *p = to_ontat_kd50g21_40nt_a1_panel(panel);

	/* add hard-coded panel modes */
	return ontat_kd50g21_40nt_a1_panel_get_fixed_modes(p, connector);
}

static int
ontat_kd50g21_40nt_a1_panel_get_timings(struct drm_panel *panel,
					unsigned int num_timings,
					struct display_timing *timings)
{
	struct ontat_kd50g21_40nt_a1_panel *p = to_ontat_kd50g21_40nt_a1_panel(panel);
	unsigned int i;

	if (p->desc->num_timings < num_timings)
		num_timings = p->desc->num_timings;

	if (timings)
		for (i = 0; i < num_timings; i++)
			timings[i] = p->desc->timings[i];

	return p->desc->num_timings;
}

static const struct drm_panel_funcs ontat_kd50g21_40nt_a1_panel_funcs = {
	.disable = ontat_kd50g21_40nt_a1_panel_disable,
	.unprepare = ontat_kd50g21_40nt_a1_panel_unprepare,
	.prepare = ontat_kd50g21_40nt_a1_panel_prepare,
	.enable = ontat_kd50g21_40nt_a1_panel_enable,
	.get_modes = ontat_kd50g21_40nt_a1_panel_get_modes,
	.get_timings = ontat_kd50g21_40nt_a1_panel_get_timings,
};

static int
ontat_kd50g21_40nt_a1_panel_probe(struct device *dev,
				  const struct ontat_kd50g21_40nt_a1_panel_desc *desc)
{
	struct ontat_kd50g21_40nt_a1_panel *panel;
	int err;

	panel = devm_kzalloc(dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	panel->enabled = false;
	panel->prepared = false;
	panel->desc = desc;

	panel->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(panel->supply))
		return PTR_ERR(panel->supply);

	panel->enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(panel->enable_gpio)) {
		err = PTR_ERR(panel->enable_gpio);
		if (err != -EPROBE_DEFER)
			dev_err(dev, "failed to request GPIO: %d\n", err);
		return err;
	}

	dev_set_drvdata(dev, panel);

	drm_panel_init(&panel->base, dev, &ontat_kd50g21_40nt_a1_panel_funcs,
		       DRM_MODE_CONNECTOR_DPI);

	err = drm_panel_of_backlight(&panel->base);
	if (err)
		return err;

	drm_panel_add(&panel->base);

	return 0;
}

static int ontat_kd50g21_40nt_a1_panel_remove(struct platform_device *pdev)
{
	struct ontat_kd50g21_40nt_a1_panel *panel = platform_get_drvdata(pdev);

	drm_panel_remove(&panel->base);
	drm_panel_disable(&panel->base);
	drm_panel_unprepare(&panel->base);

	return 0;
}

static void ontat_kd50g21_40nt_a1_panel_shutdown(struct platform_device *pdev)
{
	struct ontat_kd50g21_40nt_a1_panel *panel = platform_get_drvdata(pdev);

	drm_panel_disable(&panel->base);
	drm_panel_unprepare(&panel->base);
}

static const struct display_timing ontat_kd50g21_40nt_a1_timing = {
	.pixelclock = { 30000000, 30000000, 30000000 },
	.hactive = { 800, 800, 800 },
	.hfront_porch = {  40, 40, 40 },
	.hback_porch = { 40, 40, 40 },
	.hsync_len = { 48, 48, 48 },
	.vactive = { 480, 480, 480 },
	.vfront_porch = { 13, 13, 13 },
	.vback_porch = { 29, 29, 29 },
	.vsync_len = { 3, 3, 3 },
	.flags = DISPLAY_FLAGS_VSYNC_LOW | DISPLAY_FLAGS_HSYNC_LOW,
};

static const struct ontat_kd50g21_40nt_a1_panel_desc ontat_kd50g21_40nt_a1 = {
	.timings = &ontat_kd50g21_40nt_a1_timing,
	.num_timings = 1,
	.bpc = 8,
	.size = {
		.width = 108,
		.height = 65,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
	.bus_flags = DRM_BUS_FLAG_DE_HIGH | DRM_BUS_FLAG_PIXDATA_SAMPLE_NEGEDGE,
};

static const struct of_device_id platform_of_match[] = {
	{
		.compatible = "ontat,kd50g21-40nt-a1",
		.data = &ontat_kd50g21_40nt_a1,
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, platform_of_match);

static int ontat_kd50g21_40nt_a1_panel_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;

	id = of_match_node(platform_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	return ontat_kd50g21_40nt_a1_panel_probe(&pdev->dev, id->data);
}

static struct platform_driver ontat_kd50g21_40nt_a1_panel_platform_driver = {
	.driver = {
		.name = "ontat_kd50g21_40nt_a1_panel",
		.of_match_table = platform_of_match,
	},
	.probe = ontat_kd50g21_40nt_a1_panel_platform_probe,
	.remove = ontat_kd50g21_40nt_a1_panel_remove,
	.shutdown = ontat_kd50g21_40nt_a1_panel_shutdown,
};
module_platform_driver(ontat_kd50g21_40nt_a1_panel_platform_driver);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("ON Tat Industrial Company KD50G21-40NT-A1 panel driver");
MODULE_LICENSE("GPL v2");
