// SPDX-License-Identifier: GPL-2.0-only
/*
 * A V4L2 driver for Omnivision OX05B1S RGB-IR camera.
 *
 * Copyright 2024-2025 NXP
 *
 * Inspired from Sony imx219, imx290, imx214 and imx334 camera drivers
 *
 */

#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-cci.h>
#include <media/mipi-csi2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

enum ox05b1s_pad_ids {
	OX05B1S_PAD_SRC,
	OX05B1S_PAD_IMGL, /* long exposure image internal pad */
	OX05B1S_PAD_IMGS, /* short exposure image internal pad */
	OX05B1S_PAD_NUM
};

enum ox05b1s_stream_ids {
	OX05B1S_STREAM_IMGL, /* long exposure image stream */
	OX05B1S_STREAM_IMGS, /* short exposure image stream */
	OX05B1S_STREAM_NUM
};

#define OX05B1S_REG_SW_STB		CCI_REG8(0x0100)
#define OX05B1S_REG_SW_RST		CCI_REG8(0x0103)
#define OX05B1S_REG_CHIP_ID		CCI_REG24(0x300a)
#define OX05B1S_REG_TIMING_HTS		CCI_REG16(0x380c)
#define OX05B1S_REG_TIMING_VTS		CCI_REG16(0x380e)
#define OX05B1S_REG_EXPOSURE		CCI_REG16(0x3501)
#define OX05B1S_REG_GAIN		CCI_REG16(0x3508)
#define OX05B1S_REG_X_OUTPUT_SIZE	CCI_REG16(0x3808)
#define OX05B1S_REG_Y_OUTPUT_SIZE	CCI_REG16(0x380a)

#define client_to_ox05b1s(client)\
	container_of(i2c_get_clientdata(client), struct ox05b1s, subdev)

struct ox05b1s_sizes {
	u32	code;
	const struct v4l2_area *sizes;
};

struct ox05b1s;
struct ox05b1s_plat_data {
	char				name[20];
	u32				chip_id;
	u32				native_width;
	u32				native_height;
	u32				active_top;
	u32				active_left;
	u32				active_width;
	u32				active_height;
	const struct ox05b1s_mode	*supported_modes;
	u32				default_mode_index;
	const struct ox05b1s_sizes	*supported_codes;
	const char * const		*hdr_modes;
	u32				hdr_modes_count;
	int (*set_hdr_mode)(struct ox05b1s *sensor, u32 hdr_mode);
};

struct ox05b1s_ctrls {
	struct v4l2_ctrl_handler handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *hdr_mode;
};

#include "os08a20_regs_1080p.h"
#include "os08a20_regs_4k.h"
#include "os08a20_regs_4k_hdr.h"
#include "ox05b1s_regs_5mp.h"

struct ox05b1s_mode {
	u32 index;
	u32 width;
	u32 height;
	u32 code;
	u32 bpp;
	u32 vts; /* default VTS */
	u32 hts; /* default HTS */
	u32 exp; /* max exposure */
	bool h_bin; /* horizontal binning */
	s64 pixel_rate;
	const struct cci_reg_sequence *reg_data;
	u32 reg_data_count;
};

/* regulator supplies */
static const char * const ox05b1s_supply_name[] = {
	"avdd",  /* Analog voltage supply, 2.8 volts */
	"dvdd",  /* Digital I/O voltage supply, 1.8 volts */
	"dovdd", /* Digital voltage supply, 1.2 volts */
};

#define OX05B1S_NUM_SUPPLIES ARRAY_SIZE(ox05b1s_supply_name)

struct ox05b1s {
	struct i2c_client *i2c_client;
	struct regmap *regmap;
	struct gpio_desc *rst_gpio;
	struct regulator_bulk_data supplies[OX05B1S_NUM_SUPPLIES];
	struct clk *sensor_clk;
	const struct ox05b1s_plat_data *model;
	struct v4l2_subdev subdev;
	struct media_pad pads[OX05B1S_PAD_NUM];
	const struct ox05b1s_mode *mode;
	struct mutex lock; /* sensor lock */
	u32 stream_status;
	struct ox05b1s_ctrls ctrls;
	u64 enabled_source_streams;
};

#define OS08A20_PIXEL_RATE_144M	144000000
#define OS08A20_PIXEL_RATE_288M	288000000
static const struct ox05b1s_mode os08a20_supported_modes[] = {
	{
		/* 1080p BGGR10, no hdr, 60fps */
		.index		= 0,
		.width		= 1920,
		.height		= 1080,
		.code		= MEDIA_BUS_FMT_SBGGR10_1X10,
		.bpp		= 10,
		.vts		= 0x4a4,
		.hts		= 0x790,
		.exp		= 0x4a4 - 8,
		.h_bin		= true,
		.pixel_rate	= OS08A20_PIXEL_RATE_144M,
		.reg_data	= os08a20_init_setting_1080p,
		.reg_data_count	= ARRAY_SIZE(os08a20_init_setting_1080p),
	},
	{
		/* 4k BGGR10, no hdr, 30fps */
		.index		= 1,
		.width		= 3840,
		.height		= 2160,
		.code		= MEDIA_BUS_FMT_SBGGR10_1X10,
		.bpp		= 10,
		.vts		= 0x90a,
		.hts		= 0x818,
		.exp		= 0x90a - 8,
		.h_bin		= false,
		.pixel_rate	= OS08A20_PIXEL_RATE_288M,
		.reg_data	= os08a20_init_setting_4k_hdr,
		.reg_data_count	= ARRAY_SIZE(os08a20_init_setting_4k_hdr),
	},
	{
		/* 4k BGGR12, no hdr, 30fps */
		.index		= 2,
		.width		= 3840,
		.height		= 2160,
		.code		= MEDIA_BUS_FMT_SBGGR12_1X12,
		.bpp		= 12,
		.vts		= 0x90a,
		.hts		= 0x818,
		.exp		= 0x90a - 8,
		.h_bin		= false,
		.pixel_rate	= OS08A20_PIXEL_RATE_288M,
		.reg_data	= os08a20_init_setting_4k,
		.reg_data_count	= ARRAY_SIZE(os08a20_init_setting_4k),
	}, {
		/* sentinel */
	}
};

/* keep in sync with os08a20_supported_modes */
static const struct v4l2_area os08a20_sbggr10_sizes[] = {
	{
		.width = 1920,
		.height = 1080,
	}, {
		.width = 3840,
		.height = 2160,
	}, {
		/* sentinel */
	}
};

static const struct v4l2_area os08a20_sbggr12_sizes[] = {
	{
		.width = 3840,
		.height = 2160,
	}, {
		/* sentinel */
	}
};

static const struct ox05b1s_sizes os08a20_supported_codes[] = {
	{
		.code = MEDIA_BUS_FMT_SBGGR10_1X10,
		.sizes = os08a20_sbggr10_sizes
	},
	{
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.sizes = os08a20_sbggr12_sizes,
	}, {
		/* sentinel */
	}
};

#define OX05B1S_PIXEL_RATE_48M	48000000
static const struct ox05b1s_mode ox05b1s_supported_modes[] = {
	{
		/* 5Mp GRBG10, 30fps */
		.index		= 0,
		.width		= 2592,
		.height		= 1944,
		.code		= MEDIA_BUS_FMT_SGRBG10_1X10,
		.bpp		= 10,
		.vts		= 0x850, /* 2128 */
		.hts		= 0x2f0, /* 752 */
		.exp		= 0x850 - 8,
		.h_bin		= false,
		.pixel_rate	= OX05B1S_PIXEL_RATE_48M,
		.reg_data	= ovx5b_init_setting_2592x1944,
		.reg_data_count	= ARRAY_SIZE(ovx5b_init_setting_2592x1944),
	}, {
		/* sentinel */
	}
};

/* keep in sync with ox05b1s_supported_modes */
static const struct v4l2_area ox05b1s_sgrbg10_sizes[] = {
	{
		.width = 2592,
		.height = 1944,
	}, {
		/* sentinel */
	}
};

static const struct ox05b1s_sizes ox05b1s_supported_codes[] = {
	{
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.sizes = ox05b1s_sgrbg10_sizes,
	}, {
		/* sentinel */
	}
};

static int ox05b1s_power_on(struct ox05b1s *sensor)
{
	struct device *dev = &sensor->i2c_client->dev;
	int ret;

	ret = regulator_bulk_enable(OX05B1S_NUM_SUPPLIES, sensor->supplies);
	if (ret) {
		dev_err(dev, "Failed to enable regulators\n");
		return ret;
	}

	/* get out of powerdown and reset */
	gpiod_set_value_cansleep(sensor->rst_gpio, 0);

	ret = clk_prepare_enable(sensor->sensor_clk);
	if (ret < 0) {
		dev_err(dev, "Enable sensor clk fail ret=%d\n", ret);
		goto reg_off;
	}

	/* with XVCLK@24MHz, t2 = 6ms before first ox05b1s SCCB transaction */
	fsleep(6000);

	return 0;

reg_off:
	regulator_bulk_disable(OX05B1S_NUM_SUPPLIES, sensor->supplies);

	return ret;
}

static int ox05b1s_power_off(struct ox05b1s *sensor)
{
	gpiod_set_value_cansleep(sensor->rst_gpio, 1);

	/* XVCLK must be active for 512 cycles after last SCCB transaction */
	fsleep(350); /* 512 cycles = 0.34 ms at 24MHz */
	clk_disable_unprepare(sensor->sensor_clk);

	regulator_bulk_disable(OX05B1S_NUM_SUPPLIES, sensor->supplies);

	return 0;
}

static int ox05b1s_runtime_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ox05b1s *sensor = client_to_ox05b1s(client);

	return ox05b1s_power_off(sensor);
}

static int ox05b1s_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ox05b1s *sensor = client_to_ox05b1s(client);

	return ox05b1s_power_on(sensor);
}

static const char * const os08a20_hdr_modes[] = {
	"NO HDR", /* No HDR, single exposure */
	"HDR Staggered", /* Staggered HDR mode, 2 exposures on separate VC */
};

static const struct cci_reg_sequence os08a20_init_setting_hdr_en[] = {
	{CCI_REG8(0x3661), BIT(0)}, /* CORE1[0] STG_HDR_ALIGN_EN */
	{CCI_REG8(0x3821), BIT(5)}, /* FORMAT2[5] STG_HDR_EN */
	{CCI_REG8(0x4813), BIT(3)}, /* MIPI_CTRL_13[3] */
	{CCI_REG8(0x486e), BIT(2)}, /* MIPI_CTRL_6E[2] MIPI_VC_ENABLE */
};

static int os08a20_enable_staggered_hdr(struct ox05b1s *sensor)
{
	int ret;

	for (int i = 0; i < ARRAY_SIZE(os08a20_init_setting_hdr_en); i++) {
		ret = cci_update_bits(sensor->regmap,
				      os08a20_init_setting_hdr_en[i].reg,
				      os08a20_init_setting_hdr_en[i].val,
				      os08a20_init_setting_hdr_en[i].val, &ret);
	}

	return ret;
}

static int os08a20_disable_staggered_hdr(struct ox05b1s *sensor)
{
	int ret;

	for (int i = 0; i < ARRAY_SIZE(os08a20_init_setting_hdr_en); i++) {
		ret = cci_update_bits(sensor->regmap,
				      os08a20_init_setting_hdr_en[i].reg,
				      os08a20_init_setting_hdr_en[i].val,
				      0, &ret);
	}

	return ret;
}

static int os08a20_set_hdr_mode(struct ox05b1s *sensor, u32 hdr_mode)
{
	switch (hdr_mode) {
	case 0:
		return os08a20_disable_staggered_hdr(sensor);
	case 1:
		return os08a20_enable_staggered_hdr(sensor);
	default:
		return -EINVAL;
	}
}

static const char * const ox05b1s_hdr_modes[] = {
	"NO context switch",	/* single exposure */
	"Context switch, 2 exposures/VCs", /* context switch, RGB and IR */
};

/* ctx0 for long exposure (IR) on VC0, ctx1 for short exposure (RGB) on VC1 */
static const struct cci_reg_sequence ovx5b_init_setting_ctx_switch_en[] = {
	{CCI_REG8(0x320a), 0x01}, /* frames stay in group0 */
	{CCI_REG8(0x320b), 0x01}, /* frames stay in group1 */

	{CCI_REG8(0x3208), 0x00}, /* group0 start */
	{CCI_REG8(0x3501), 0x01}, /* exposure */
	{CCI_REG8(0x3502), 0x00}, /* exposure */
	{CCI_REG8(0x4813), 0x00}, /* mipi vc0 */
	{CCI_REG8(0x3208), 0x10}, /* group0 end */

	{CCI_REG8(0x3208), 0x01}, /* group1 start */
	{CCI_REG8(0x3501), 0x00}, /* exposure */
	{CCI_REG8(0x3502), 0x80}, /* exposure */
	{CCI_REG8(0x4813), 0x01}, /* mipi vc1 */
	{CCI_REG8(0x3208), 0x11}, /* group1 end */

	{CCI_REG8(0x3211), 0x30}, /* context switch en */
	{CCI_REG8(0x3208), 0xa0}, /* repeat launch */
};

static const struct cci_reg_sequence ovx5b_init_setting_ctx_switch_dis[] = {
	{CCI_REG8(0x3211), 0x61},
	{CCI_REG8(0x320a), 0x0},
	{CCI_REG8(0x320b), 0x0},
};

static int ox05b1s_enable_context_switching(struct ox05b1s *sensor)
{
	return cci_multi_reg_write(sensor->regmap,
				   ovx5b_init_setting_ctx_switch_en,
				   ARRAY_SIZE(ovx5b_init_setting_ctx_switch_en),
				   NULL);
}

static int ox05b1s_disable_context_switching(struct ox05b1s *sensor)
{
	return cci_multi_reg_write(sensor->regmap,
				   ovx5b_init_setting_ctx_switch_dis,
				   ARRAY_SIZE(ovx5b_init_setting_ctx_switch_dis),
				   NULL);
}

static int ox05b1s_set_hdr_mode(struct ox05b1s *sensor, u32 hdr_mode)
{
	switch (hdr_mode) {
	case 0:
		return ox05b1s_disable_context_switching(sensor);
	case 1:
		return ox05b1s_enable_context_switching(sensor);
	default:
		return -EINVAL;
	}
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct ox05b1s,
			     ctrls.handler)->subdev;
}

static int ox05b1s_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ox05b1s *sensor = client_to_ox05b1s(client);
	u32 w = sensor->mode->width;
	u32 h = sensor->mode->height;
	int ret = 0;
	u32 hts;

	/* apply V4L2 controls values only if power is already up */
	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	/* s_ctrl holds sensor lock */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		ret = cci_write(sensor->regmap, OX05B1S_REG_TIMING_VTS,
				h + ctrl->val, NULL);
		break;
	case V4L2_CID_HBLANK:
		hts = (sensor->mode->h_bin) ?
			  w + ctrl->val : (w + ctrl->val) / 2;
		ret = cci_write(sensor->regmap, OX05B1S_REG_TIMING_HTS,
				hts, NULL);
		break;
	case V4L2_CID_PIXEL_RATE:
		/* Read-only, but we adjust it based on mode. */
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(sensor->regmap, OX05B1S_REG_GAIN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_EXPOSURE:
		ret = cci_write(sensor->regmap, OX05B1S_REG_EXPOSURE,
				ctrl->val, NULL);
		break;
	case V4L2_CID_HDR_SENSOR_MODE:
		if (sensor->model->set_hdr_mode)
			ret = sensor->model->set_hdr_mode(sensor, ctrl->val);
		else
			ret = -EINVAL;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ox05b1s_ctrl_ops = {
	.s_ctrl = ox05b1s_s_ctrl,
};

/*
 * MIPI CSI-2 link frequencies.
 * link_freq = (pixel_rate * bpp) / (2 * data_lanes)
 */
static const s64 ox05b1s_csi2_link_freqs[] = {
	200000000,
};

/* Link freq for default mode: 1080p RAW10, 4 data lanes 800 Mbps/lane. */
#define OX05B1S_DEFAULT_LINK_FREQ	0

static int ox05b1s_init_controls(struct ox05b1s *sensor)
{
	const struct v4l2_ctrl_ops *ops = &ox05b1s_ctrl_ops;
	struct ox05b1s_ctrls *ctrls = &sensor->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	struct device *dev = &sensor->i2c_client->dev;
	struct v4l2_fwnode_device_properties props;
	int ret;

	v4l2_ctrl_handler_init(hdl, 7);

	/* we can use our own mutex for the ctrl lock */
	hdl->lock = &sensor->lock;

	/* Clock related controls */
	ctrls->link_freq = v4l2_ctrl_new_int_menu(hdl, ops,
						  V4L2_CID_LINK_FREQ,
						  ARRAY_SIZE(ox05b1s_csi2_link_freqs) - 1,
						  OX05B1S_DEFAULT_LINK_FREQ,
						  ox05b1s_csi2_link_freqs);

	/* mode dependent, actual range set in ox05b1s_update_controls */
	ctrls->pixel_rate = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_PIXEL_RATE,
					      0, 0, 1, 0);

	ctrls->hblank = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HBLANK,
					  0, 0, 1, 0);

	ctrls->vblank = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VBLANK,
					  0, 0, 1, 0);

	ctrls->exposure = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE,
					    0, 0, 1, 0);

	ctrls->gain = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_ANALOGUE_GAIN,
					0, 0xFFFF, 1, 0x80);

	if (sensor->model->hdr_modes)
		ctrls->hdr_mode = v4l2_ctrl_new_std_menu_items(hdl, ops, V4L2_CID_HDR_SENSOR_MODE,
							       sensor->model->hdr_modes_count - 1,
								0, 0, sensor->model->hdr_modes);
	else
		ctrls->hdr_mode = NULL;

	if (hdl->error) {
		ret = hdl->error;
		goto free_ctrls;
	}

	ctrls->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	ctrls->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ret = v4l2_fwnode_device_parse(dev, &props);
	if (ret)
		goto free_ctrls;

	ret = v4l2_ctrl_new_fwnode_properties(hdl, ops, &props);
	if (ret)
		goto free_ctrls;

	sensor->subdev.ctrl_handler = hdl;
	return 0;

free_ctrls:
	dev_err(dev, "Failed to init controls\n");
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static int ox05b1s_apply_current_mode(struct ox05b1s *sensor);

static int ox05b1s_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ox05b1s *sensor = client_to_ox05b1s(client);
	int ret;

	if (enable) {
		ret = pm_runtime_resume_and_get(&client->dev);
		if (ret < 0)
			return ret;
		ret = ox05b1s_apply_current_mode(sensor);
		if (!ret)
			ret = cci_write(sensor->regmap, OX05B1S_REG_SW_STB,
					0x01, NULL);
	} else {
		ret = cci_write(sensor->regmap, OX05B1S_REG_SW_STB, 0x00, NULL);
	}

	sensor->stream_status = enable;

	if (!enable || ret) {
		pm_runtime_mark_last_busy(&sensor->i2c_client->dev);
		pm_runtime_put_autosuspend(&client->dev);
	}

	return 0;
}

static void ox05b1s_update_pad_format(const struct ox05b1s_mode *mode,
				      struct v4l2_mbus_framefmt *fmt)
{
	if (!fmt)
		return;

	fmt->code = mode->code;
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int ox05b1s_propagate_fmt(struct v4l2_subdev_state *state,
				 const struct ox05b1s_mode *mode)
{
	struct v4l2_mbus_framefmt *format;

	/* Initialize all the formats according to indicated mode */
	format = v4l2_subdev_state_get_format(state, OX05B1S_PAD_SRC,
					      OX05B1S_STREAM_IMGL);
	ox05b1s_update_pad_format(mode, format);

	format = v4l2_subdev_state_get_format(state, OX05B1S_PAD_IMGL, 0);
	ox05b1s_update_pad_format(mode, format);

	format = v4l2_subdev_state_get_format(state, OX05B1S_PAD_SRC,
					      OX05B1S_STREAM_IMGS);
	ox05b1s_update_pad_format(mode, format);

	format = v4l2_subdev_state_get_format(state, OX05B1S_PAD_IMGS, 0);
	ox05b1s_update_pad_format(mode, format);

	return 0;
}

static int ox05b1s_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ox05b1s *sensor = client_to_ox05b1s(client);
	struct v4l2_subdev_route routes[OX05B1S_STREAM_NUM];
	struct v4l2_subdev_krouting routing = { };
	int i;
	int ret;

	/* initialize routes from all internal sink pads to the source pad */
	for (i = 0; i < OX05B1S_STREAM_NUM; i++) {
		routes[i].source_pad = 0;
		routes[i].source_stream = i;
		routes[i].sink_pad = OX05B1S_PAD_IMGL + i;
		routes[i].sink_stream = 0;
		routes[i].flags = 0;
	}

	/* keep all routes inactive by default, except IMGL */
	routes[OX05B1S_STREAM_IMGL].flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE;

	routing.num_routes = OX05B1S_STREAM_NUM;
	routing.routes = routes;

	ret = v4l2_subdev_set_routing(sd, state, &routing);
	if (ret)
		return ret;

	/* Initialize all the formats according to current mode */
	return ox05b1s_propagate_fmt(state, sensor->mode);
}

static int ox05b1s_enum_mbus_code_def(struct v4l2_subdev *sd,
				      struct v4l2_subdev_mbus_code_enum *code)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ox05b1s *sensor = client_to_ox05b1s(client);
	u32 default_mode_index = sensor->model->default_mode_index;

	if (code->index > 0)
		return -EINVAL;

	code->code = sensor->model->supported_modes[default_mode_index].code;

	return 0;
}

static int ox05b1s_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ox05b1s *sensor = client_to_ox05b1s(client);
	const struct ox05b1s_sizes *codes = sensor->model->supported_codes;
	int i = 0;

	/* for internal pads, return the default code */
	if (code->pad != OX05B1S_PAD_SRC)
		return ox05b1s_enum_mbus_code_def(sd, code);

	while (i++ < code->index && codes->code)
		codes++;

	if (!codes->code) /* code->index outside supported_codes[] */
		return -EINVAL;

	code->code = codes->code;

	return 0;
}

static int ox05b1s_enum_frame_size_def(struct v4l2_subdev *sd,
				       struct v4l2_subdev_frame_size_enum *fse)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ox05b1s *sensor = client_to_ox05b1s(client);
	u32 default_mode_index = sensor->model->default_mode_index;
	const struct ox05b1s_mode *default_mode;

	if (fse->index > 0)
		return -EINVAL;

	default_mode = &sensor->model->supported_modes[default_mode_index];
	fse->min_width = default_mode->width;
	fse->max_width = fse->min_width;
	fse->min_height = default_mode->height;
	fse->max_height = fse->min_height;

	return 0;
}

static int ox05b1s_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ox05b1s *sensor = client_to_ox05b1s(client);
	const struct ox05b1s_sizes *codes = sensor->model->supported_codes;
	const struct v4l2_area *sizes;
	int i = 0;

	/* for internal pads, return the default size */
	if (fse->pad != OX05B1S_PAD_SRC)
		return ox05b1s_enum_frame_size_def(sd, fse);

	/* image streams */
	while (codes->code) {
		if (codes->code == fse->code)
			break;
		codes++;
	}

	if (!codes->code) /* fse->code not in supported_codes[] */
		return -EINVAL;

	sizes = codes->sizes;
	while (i++ < fse->index && sizes->width)
		sizes++;

	if (!sizes->width) /* fse->index outside sizes[] */
		return -EINVAL;

	fse->min_width = sizes->width;
	fse->max_width = fse->min_width;
	fse->min_height = sizes->height;
	fse->max_height = fse->min_height;

	return 0;
}

/* Update control ranges based on current streaming mode, needs sensor lock */
static int ox05b1s_update_controls(struct ox05b1s *sensor)
{
	int ret;
	struct device *dev = &sensor->i2c_client->dev;
	u32 hts = sensor->mode->hts;
	u32 hblank;
	u32 vts = sensor->mode->vts;
	u32 vblank = vts - sensor->mode->height;
	u64 pixel_rate = sensor->mode->pixel_rate;
	u32 min_exp = 8;
	u32 max_exp = vts - 8;

	ret = __v4l2_ctrl_modify_range(sensor->ctrls.pixel_rate, pixel_rate,
				       pixel_rate, 1, pixel_rate);
	if (ret) {
		dev_err(dev, "Modify range for pixel_rate %llu-%llu failed\n",
			pixel_rate, pixel_rate);
		goto out;
	}

	if (sensor->mode->h_bin)
		hblank = hts - sensor->mode->width;
	else
		hblank = 2 * hts - sensor->mode->width;

	ret = __v4l2_ctrl_modify_range(sensor->ctrls.hblank, hblank, hblank,
				       1, hblank);
	if (ret) {
		dev_err(dev, "Modify range for hblank %u-%u failed\n",
			hblank, hblank);
		goto out;
	}
	__v4l2_ctrl_s_ctrl(sensor->ctrls.hblank,
			   sensor->ctrls.hblank->default_value);

	ret = __v4l2_ctrl_modify_range(sensor->ctrls.vblank, 0, vblank * 4,
				       1, vblank);
	if (ret) {
		dev_err(dev, "Modify range for vblank %u-%u failed\n",
			vblank, vblank);
		goto out;
	}
	__v4l2_ctrl_s_ctrl(sensor->ctrls.vblank,
			   sensor->ctrls.vblank->default_value);

	ret = __v4l2_ctrl_modify_range(sensor->ctrls.exposure, min_exp, max_exp,
				       1, max_exp / 2);
	if (ret) {
		dev_err(dev, "Modify range for exposure %u-%u failed\n",
			min_exp, max_exp);
		goto out;
	}
	__v4l2_ctrl_s_ctrl(sensor->ctrls.exposure,
			   sensor->ctrls.exposure->default_value);

out:
	return ret;
}

/* needs sensor lock and power on */
static int ox05b1s_apply_current_mode(struct ox05b1s *sensor)
{
	struct device *dev = &sensor->i2c_client->dev;
	const struct cci_reg_sequence *reg_data = NULL;
	u32 w = sensor->mode->width;
	u32 h = sensor->mode->height;
	int ret;

	cci_write(sensor->regmap, OX05B1S_REG_SW_RST, 0x01, &ret);

	reg_data = sensor->mode->reg_data;
	cci_multi_reg_write(sensor->regmap, reg_data,
				  sensor->mode->reg_data_count, &ret);

	cci_write(sensor->regmap, OX05B1S_REG_X_OUTPUT_SIZE, w, &ret);
	cci_write(sensor->regmap, OX05B1S_REG_Y_OUTPUT_SIZE, h, &ret);

	if (ret)
		goto out;

	/* setup handler will write actual controls into sensor registers */
	ret = __v4l2_ctrl_handler_setup(&sensor->ctrls.handler);

out:
	if (ret < 0)
		dev_err(dev, "Failed to apply mode %dx%d,bpp=%d\n", w, h,
			sensor->mode->bpp);

	return ret;
}

/* similar with v4l2_find_nearest_size but filter for mbus code, needs sensor lock */
static const struct ox05b1s_mode *ox05b1s_nearest_size(const struct ox05b1s_mode *supported_modes,
						       struct v4l2_subdev_format *fmt)
{
	u32 err, min_error = U32_MAX;
	const struct ox05b1s_mode *best = NULL;

	if (!supported_modes)
		return NULL;

	for (; supported_modes->width; supported_modes++) {
		const u32 w = supported_modes->width;
		const u32 h = supported_modes->height;

		if (supported_modes->code != fmt->format.code)
			continue;

		err = abs(w - fmt->format.width) + abs(h - fmt->format.height);
		if (err > min_error)
			continue;

		min_error = err;
		best = supported_modes;
		if (!err)
			break;
	}

	return best;
}

/* get a valid mbus code, either the requested one or the default one */
static u32 ox05b1s_find_code(const struct ox05b1s_plat_data *model, u32 code)
{
	const struct ox05b1s_sizes *supported_codes = model->supported_codes;
	u32 found_code = 0;

	while (supported_codes->code) {
		if (supported_codes->code == code) {
			found_code = code;
			break;
		}

		supported_codes++;
	}

	if (!supported_codes->code) /* code not in supported_codes[] */
		found_code = supported_codes[model->default_mode_index].code;

	return found_code;
}

static int ox05b1s_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state,
			   struct v4l2_subdev_format *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ox05b1s *sensor = client_to_ox05b1s(client);
	struct device *dev = &sensor->i2c_client->dev;

	/*
	 * The driver is mode-based, the format can be set on the source pad
	 * only, and only for the long exposure stream.
	 */
	if (fmt->pad != OX05B1S_PAD_SRC || fmt->stream != OX05B1S_STREAM_IMGL)
		return v4l2_subdev_get_fmt(sd, state, fmt);

	/* if no matching mbus codes found, use the one from the default mode */
	fmt->format.code = ox05b1s_find_code(sensor->model, fmt->format.code);
	sensor->mode = ox05b1s_nearest_size(sensor->model->supported_modes, fmt);
	/* update controls that depend on current mode */
	ox05b1s_update_controls(sensor);

	fmt->format.width = sensor->mode->width;
	fmt->format.height = sensor->mode->height;
	fmt->format.field = V4L2_FIELD_NONE;

	dev_dbg(dev, "Set mode index=%d, %d x %d, code=0x%x, on pad %d stream %d\n",
		sensor->mode->index, fmt->format.width, fmt->format.height,
		fmt->format.code, fmt->pad, fmt->stream);

	/* propagate the format on the sensor */
	return ox05b1s_propagate_fmt(state, sensor->mode);
}

static u8 ox05b1s_code2dt(const u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
		return MIPI_CSI2_DT_RAW10;
	case MEDIA_BUS_FMT_SBGGR12_1X12:
		return MIPI_CSI2_DT_RAW12;
	default:
		return MIPI_CSI2_DT_RAW10;
	}
}

static int ox05b1s_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_frame_desc *fd)
{
	struct v4l2_subdev_route *route;
	struct v4l2_subdev_state *state;
	const struct v4l2_mbus_framefmt *fmt;

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	for_each_active_route(&state->routing, route) {
		fd->entry[fd->num_entries].stream = route->source_stream;
		fmt = v4l2_subdev_state_get_format(state, OX05B1S_PAD_SRC,
						   route->source_stream);
		fd->entry[fd->num_entries].pixelcode = fmt->code;
		fd->entry[fd->num_entries].bus.csi2.dt = ox05b1s_code2dt(fmt->code);
		switch (route->source_stream) {
		case OX05B1S_STREAM_IMGL:
			fd->entry[fd->num_entries].bus.csi2.vc = 0; break;
		case OX05B1S_STREAM_IMGS:
			fd->entry[fd->num_entries].bus.csi2.vc = 1; break;
		}
		dev_dbg(sd->dev, "%s using VC=%d and DT=%x for stream %d\n",
			__func__, fd->entry[fd->num_entries].bus.csi2.vc,
			fd->entry[fd->num_entries].bus.csi2.dt,
			route->source_stream);
		fd->num_entries++;
	}
	v4l2_subdev_unlock_state(state);

	return 0;
}

static int ox05b1s_get_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_selection *sel)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ox05b1s *sensor = client_to_ox05b1s(client);

	switch (sel->target) {
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = sensor->model->native_width;
		sel->r.height = sensor->model->native_height;
		return 0;
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.top = sensor->model->active_top;
		sel->r.left = sensor->model->active_left;
		sel->r.width = sensor->model->active_width;
		sel->r.height = sensor->model->active_height;
		return 0;
	}

	return -EINVAL;
}

static int ox05b1s_set_routing(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state,
			       enum v4l2_subdev_format_whence which,
			       struct v4l2_subdev_krouting *routing)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ox05b1s *sensor = client_to_ox05b1s(client);
	struct v4l2_subdev_route *route;
	struct v4l2_ctrl *hdr_ctrl = sensor->ctrls.hdr_mode;

	if (which == V4L2_SUBDEV_FORMAT_ACTIVE &&
	    media_entity_is_streaming(&sd->entity))
		return -EBUSY;

	ret = v4l2_subdev_routing_validate(sd, routing,
					   V4L2_SUBDEV_ROUTING_ONLY_1_TO_1);
	if (ret)
		return ret;

	ret = v4l2_subdev_set_routing(sd, state, routing);
	if (ret)
		return ret;

	/* Initialize all the formats according to current mode */
	ret = ox05b1s_propagate_fmt(state, sensor->mode);
	if (ret)
		return ret;

	/* if the short exposure stream route is active, activate hdr mode */
	hdr_ctrl->cur.val = 0; /* reset hdr mode from previous routing */
	for_each_active_route(&state->routing, route) {
		if (route->source_stream == OX05B1S_STREAM_IMGS) {
			hdr_ctrl->cur.val = 1;
			break;
		}
	}

	return 0;
}

static int ox05b1s_enable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  u32 src_pad, u64 streams_mask)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ox05b1s *sensor = client_to_ox05b1s(client);
	struct device *dev = &sensor->i2c_client->dev;
	int ret;

	dev_dbg(dev, "Enable streams with mask %llx\n", streams_mask);
	if (!sensor->enabled_source_streams) {
		/* if the sensor is not streaming already... */
		ret = ox05b1s_s_stream(sd, 1);
		if (ret)
			return ret;
	}
	sensor->enabled_source_streams |= streams_mask;

	return 0;
}

static int ox05b1s_disable_streams(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   u32 src_pad, u64 streams_mask)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ox05b1s *sensor = client_to_ox05b1s(client);
	struct device *dev = &sensor->i2c_client->dev;
	int ret = 0;

	dev_dbg(dev, "Disable streams with mask %llx\n", streams_mask);
	sensor->enabled_source_streams &= ~streams_mask;
	/* stop the sensor when there is no more stream enabled */
	if (!sensor->enabled_source_streams)
		ret = ox05b1s_s_stream(sd, 0);

	return ret;
}

static const struct v4l2_subdev_video_ops ox05b1s_subdev_video_ops = {
	.s_stream = ox05b1s_s_stream,
};

static const struct v4l2_subdev_pad_ops ox05b1s_subdev_pad_ops = {
	.set_fmt		= ox05b1s_set_fmt,
	.get_fmt		= v4l2_subdev_get_fmt,
	.get_frame_desc		= ox05b1s_get_frame_desc,
	.enum_mbus_code		= ox05b1s_enum_mbus_code,
	.enum_frame_size	= ox05b1s_enum_frame_size,
	.get_selection		= ox05b1s_get_selection,
	.set_routing		= ox05b1s_set_routing,
	.enable_streams		= ox05b1s_enable_streams,
	.disable_streams	= ox05b1s_disable_streams,
};

static const struct v4l2_subdev_ops ox05b1s_subdev_ops = {
	.video = &ox05b1s_subdev_video_ops,
	.pad   = &ox05b1s_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops ox05b1s_internal_ops = {
	.init_state = ox05b1s_init_state,
};

static void ox05b1s_get_gpios(struct ox05b1s *sensor)
{
	struct device *dev = &sensor->i2c_client->dev;

	sensor->rst_gpio = devm_gpiod_get_optional(dev, "reset",
						   GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->rst_gpio))
		dev_warn(dev, "No sensor reset pin available\n");
}

static int ox05b1s_get_regulators(struct ox05b1s *sensor)
{
	struct device *dev = &sensor->i2c_client->dev;
	unsigned int i;

	for (i = 0; i < OX05B1S_NUM_SUPPLIES; i++)
		sensor->supplies[i].supply = ox05b1s_supply_name[i];

	return devm_regulator_bulk_get(dev, OX05B1S_NUM_SUPPLIES,
				       sensor->supplies);
}

static int ox05b1s_read_chip_id(struct ox05b1s *sensor)
{
	struct device *dev = &sensor->i2c_client->dev;
	u64 chip_id;
	char *camera_name;
	int ret;

	ret = cci_read(sensor->regmap, OX05B1S_REG_CHIP_ID, &chip_id, NULL);
	if (ret) {
		dev_err(dev, "Camera chip_id read error\n");
		return -ENODEV;
	}

	switch (chip_id) {
	case 0x530841:
		camera_name = "os08a20";
		break;
	case 0x580542:
		camera_name = "ox05b1s";
		break;
	default:
		camera_name = "unknown";
		break;
	}

	if (chip_id == sensor->model->chip_id) {
		dev_info(dev, "Camera %s detected, chip_id=%llx\n",
			 camera_name, chip_id);
	} else {
		dev_err(dev, "Detected %s camera (chip_id=%llx), but expected %s (chip_id=%x)\n",
			camera_name, chip_id,
			sensor->model->name, sensor->model->chip_id);
		return -ENODEV;
	}

	return 0;
}

static int ox05b1s_probe(struct i2c_client *client)
{
	int ret;
	struct device *dev = &client->dev;
	struct v4l2_subdev *sd;
	struct ox05b1s *sensor;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sensor->regmap))
		return dev_err_probe(dev, PTR_ERR(sensor->regmap),
				     "Failed to allocate sensor register map\n");

	sensor->i2c_client = client;

	sensor->model = of_device_get_match_data(dev);

	ox05b1s_get_gpios(sensor);

	/* Get system clock, xvclk */
	sensor->sensor_clk = devm_clk_get(dev, NULL);
	if (IS_ERR(sensor->sensor_clk))
		return dev_err_probe(dev, PTR_ERR(sensor->sensor_clk),
				     "Failed to get xvclk\n");

	ret = ox05b1s_get_regulators(sensor);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	sd = &sensor->subdev;
	v4l2_i2c_subdev_init(sd, client, &ox05b1s_subdev_ops);
	sd->internal_ops = &ox05b1s_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_STREAMS;
	sd->dev = &client->dev;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->pads[OX05B1S_PAD_SRC].flags = MEDIA_PAD_FL_SOURCE;
	sensor->pads[OX05B1S_PAD_IMGL].flags = MEDIA_PAD_FL_SINK |
					       MEDIA_PAD_FL_INTERNAL;
	sensor->pads[OX05B1S_PAD_IMGS].flags = MEDIA_PAD_FL_SINK |
					       MEDIA_PAD_FL_INTERNAL;
	ret = media_entity_pads_init(&sd->entity, OX05B1S_PAD_NUM,
				     sensor->pads);
	if (ret)
		goto probe_out;

	ret = devm_mutex_init(dev, &sensor->lock);
	if (ret)
		goto probe_out;

	ret = ox05b1s_init_controls(sensor);
	if (ret)
		goto probe_err_entity_cleanup;

	/* power on manually */
	ret = ox05b1s_power_on(sensor);
	if (ret) {
		dev_err_probe(dev, ret, "Failed to power on\n");
		goto probe_err_free_ctrls;
	}

	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);

	ret = ox05b1s_read_chip_id(sensor);
	if (ret)
		goto probe_err_pm_runtime;

	v4l2_i2c_subdev_set_name(sd, client, sensor->model->name, NULL);

	sensor->mode = &sensor->model->supported_modes[0];
	ox05b1s_update_controls(sensor);

	/* Centrally managed subdev active state */
	sd->state_lock = &sensor->lock;
	ret = v4l2_subdev_init_finalize(sd);
	if (ret < 0) {
		dev_err_probe(dev, ret, "Subdev init error: %d\n", ret);
		goto probe_err_pm_runtime;
	}

	ret = v4l2_async_register_subdev_sensor(sd);
	if (ret < 0) {
		dev_err_probe(&client->dev, ret,
			      "Async register failed, ret=%d\n", ret);
		goto probe_err_subdev_cleanup;
	}

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_put_autosuspend(dev);

	return 0;

probe_err_subdev_cleanup:
	v4l2_subdev_cleanup(sd);
probe_err_pm_runtime:
	pm_runtime_put_noidle(dev);
	pm_runtime_disable(dev);
	ox05b1s_runtime_suspend(dev);
probe_err_free_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
probe_err_entity_cleanup:
	media_entity_cleanup(&sd->entity);
probe_out:
	return ret;
}

static void ox05b1s_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ox05b1s *sensor = client_to_ox05b1s(client);
	struct device *dev = &client->dev;

	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev))
		ox05b1s_runtime_suspend(dev);
	pm_runtime_set_suspended(dev);
	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
}

static DEFINE_RUNTIME_DEV_PM_OPS(ox05b1s_pm_ops, ox05b1s_runtime_suspend,
				 ox05b1s_runtime_resume, NULL);

static const struct ox05b1s_plat_data os08a20_data = {
	.name			= "os08a20",
	.chip_id		= 0x530841,
	.native_width		= 3872, /* 16 dummy + 3840 active + 16 dummy */
	.native_height		= 2192, /* 16 dummy + 2160 active + 16 dummy */
	.active_top		= 16,
	.active_left		= 16,
	.active_width		= 3840,
	.active_height		= 2160,
	.supported_modes	= os08a20_supported_modes,
	.default_mode_index	= 0,
	.supported_codes	= os08a20_supported_codes,
	.hdr_modes		= os08a20_hdr_modes,
	.hdr_modes_count	= ARRAY_SIZE(os08a20_hdr_modes),
	.set_hdr_mode		= os08a20_set_hdr_mode,
};

static const struct ox05b1s_plat_data ox05b1s_data = {
	.name			= "ox05b1s",
	.chip_id		= 0x580542,
	.native_width		= 2608, /* 8 dummy + 2592 active + 8 dummy */
	.native_height		= 1960, /* 8 dummy + 1944 active + 8 dummy */
	.active_top		= 8,
	.active_left		= 8,
	.active_width		= 2592,
	.active_height		= 1944,
	.supported_modes	= ox05b1s_supported_modes,
	.default_mode_index	= 0,
	.supported_codes	= ox05b1s_supported_codes,
	.hdr_modes		= ox05b1s_hdr_modes,
	.hdr_modes_count	= ARRAY_SIZE(ox05b1s_hdr_modes),
	.set_hdr_mode		= ox05b1s_set_hdr_mode,
};

static const struct i2c_device_id ox05b1s_id[] = {
	{"ox05b1s", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ox05b1s_id);

static const struct of_device_id ox05b1s_of_match[] = {
	{
		.compatible = "ovti,os08a20",
		.data = &os08a20_data,
	},
	{
		.compatible = "ovti,ox05b1s",
		.data = &ox05b1s_data,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ox05b1s_of_match);

static struct i2c_driver ox05b1s_i2c_driver = {
	.driver = {
		.name = "ox05b1s",
		.pm = pm_ptr(&ox05b1s_pm_ops),
		.of_match_table	= ox05b1s_of_match,
	},
	.probe	= ox05b1s_probe,
	.remove = ox05b1s_remove,
	.id_table = ox05b1s_id,
};

module_i2c_driver(ox05b1s_i2c_driver);
MODULE_DESCRIPTION("Omnivision OX05B1S MIPI Camera Subdev Driver");
MODULE_AUTHOR("Mirela Rabulea <mirela.rabulea@nxp.com>");
MODULE_LICENSE("GPL");
