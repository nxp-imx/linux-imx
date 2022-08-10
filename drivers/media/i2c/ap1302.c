/*
 * Copyright (c) 2013 Intel Corporation. All Rights Reserved.
 * Copyright 2022 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/regmap.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

#define AP1302_NAME		"ap1302_mipi"
#define AP1302_CHIP_ID		0x265
#define AP1302_I2C_MAX_LEN	65534
#define AP1302_FW_WINDOW_OFFSET	0x8000
#define AP1302_FW_WINDOW_SIZE	0x2000
#define AP1302_NUM_CONSUMERS	3

/* AP1302 registers */
#define REG_CHIP_VERSION	0x0000
#define REG_CHIP_REV		0x0050
#define REG_MF_ID		0x0004
#define REG_ERROR		0x0006
#define REG_CTRL		0x1000
#define REG_DZ_TGT_FCT		0x1010
#define REG_SFX_MODE		0x1016
#define REG_SS_HEAD_PT0		0x1174
#define REG_ATOMIC		0x1184
#define REG_PREVIEW_WIDTH	0x2000
#define REG_PREVIEW_HEIGHT	0x2002
#define REG_AE_BV_OFF		0x5014
#define REG_AE_BV_BIAS		0x5016
#define REG_AWB_CTRL		0x5100
#define REG_FLICK_CTRL		0x5440
#define REG_SCENE_CTRL		0x5454
#define REG_BOOTDATA_STAGE	0x6002
#define REG_SENSOR_SELECT	0x600C
#define REG_SYS_START		0x601A
#define REG_BOOTDATA_CHECKSUM	0x6134
#define REG_SIP_CRC		0xF052

#define AP1302_SENS_PAD_SOURCE	0
#define AP1302_SENS_PADS_NUM	1

struct ap1302_res_info {
	u32 width;
	u32 height;
	u32 hact;
	u32 vact;
};

struct ap1302_firmware {
	u32 crc;
	u32 checksum;
	u32 pll_init_size;
	u32 total_size;
};

enum AP1302_AF_MODE {
	AF_MODE_AUTO,
	AF_MODE_MANUAL,
};

struct ap1302_device {
	struct v4l2_subdev subdev;
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_fract frame_interval;
	struct media_pad pads[AP1302_SENS_PADS_NUM];

	struct i2c_client *i2c_client;

	struct mutex lock;
	struct regmap *regmap16;

	struct regulator_bulk_data supplies[AP1302_NUM_CONSUMERS];

	/* GPIO descriptor */
	struct gpio_desc *reset;
	struct gpio_desc *isp_en;

	const struct firmware *fw;
	const struct ap1302_res_info *cur_mode;

	bool mode_change;
};
/* Static definitions */
static struct regmap_config ap1302_reg16_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
};

/* regulator supplies */
static const char * const ap1302_supply_name[] = {
	"AVDD",
	"DVDD",
	"VDDIO",
};

static const struct ap1302_res_info ap1302_preview_res[] = {
	{
		.width  = 1920,
		.height = 1080,
		.hact   = 1920,
		.vact   = 1080,
	}, {
		.width  = 1280,
		.height = 800,
		.hact   = 1280,
		.vact   = 800,
	},
	{
		.width  = 1280,
		.height = 720,
		.hact   = 1280,
		.vact   = 720,
	},
	{
		.width  = 640,
		.height = 480,
		.hact   = 640,
		.vact   = 480,
	},
};

static int ap1302_s_stream(struct v4l2_subdev *sd, int enable);
#define to_ap1302_device(sub_dev) \
		container_of(sub_dev, struct ap1302_device, subdev)

static int ap1302_write_reg(struct ap1302_device *ap1302_dev, u16 reg,  u16 val)
{
	struct i2c_client *client = ap1302_dev->i2c_client;
	u8 au8Buf[4] = {0};

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val >> 8;
	au8Buf[3] = val & 0xff;

	if (i2c_master_send(client, au8Buf, 4) < 0) {
		dev_err(&client->dev, "Write reg error: reg=0x%x, val=0x%x\n", reg, val);
		return -1;
	}

	return 0;
}

static int ap1302_read_reg(struct ap1302_device *ap1302_dev, u16 reg, u16 *val)
{
	struct i2c_client *client = ap1302_dev->i2c_client;

	u8 RegBuf[2] = {0};
	u8 ValBuf[2] = {0};

	RegBuf[0] = reg >> 8;
	RegBuf[1] = reg & 0xff;

	if (i2c_master_send(client, RegBuf, 2) != 2) {
		dev_err(&client->dev, "Read reg error: reg=0x%x\n", reg);
		return -1;
	}

	if (i2c_master_recv(client, ValBuf, 2) != 2) {
		dev_err(&client->dev, "Read reg error: reg=0x%x\n", reg);
		return -1;
	}

	*val = ((u16)ValBuf[0] << 8) | (u16)ValBuf[1];

	return 0;
}

/* When loading firmware, host writes firmware data from address 0x8000.
 * When the address reaches 0x9FFF, the next address should return to 0x8000.
 * This function handles this address window and load firmware data to AP1302.
 * win_pos indicates the offset within this window. Firmware loading procedure
 * may call this function several times. win_pos records the current position
 * that has been written to.
 */

static int ap1302_write_fw_window(struct ap1302_device *dev,
				  u16 *win_pos, const u8 *buf, u32 len)
{
	int ret;
	u32 pos;
	u32 sub_len;

	for (pos = 0; pos < len; pos += sub_len) {
		if (len - pos < AP1302_FW_WINDOW_SIZE - *win_pos)
			sub_len = len - pos;
		else
			sub_len = AP1302_FW_WINDOW_SIZE - *win_pos;

		ret = regmap_raw_write(dev->regmap16,
					*win_pos + AP1302_FW_WINDOW_OFFSET,
					buf + pos, sub_len);

		if (ret)
			return ret;
		*win_pos += sub_len;
		if (*win_pos >= AP1302_FW_WINDOW_SIZE)
			*win_pos = 0;
	}
	return 0;
}

static void ap1302_fw_handler(const struct firmware *fw, void *context)
{
	struct ap1302_device *ap1302_dev = context;
	struct device *dev = &ap1302_dev->i2c_client->dev;
	struct v4l2_subdev *sd = &ap1302_dev->subdev;
	struct ap1302_firmware *ap1302_fw;
	const u8 *fw_data;
	u16 regVal, win_pos = 0;
	int ret;

	if (fw == NULL) {
		dev_err(dev, "Failed to request_firmware\n");
		return;
	}

	ap1302_fw = (struct ap1302_firmware *)fw->data;

	/* The fw binary contains a header of struct ap1302_firmware.
	 * Following the header is the bootdata of AP1302.
	 * The bootdata pointer can be referenced as &fw[1].
	 */
	fw_data = (u8 *)&ap1302_fw[1];

	/* Clear crc register. */
	ret = ap1302_write_reg(ap1302_dev, REG_SIP_CRC, 0xffff);
	if (ret) {
		dev_err(dev, "Fail to write AP1302[0x%x], ret=%d\n",
			REG_SIP_CRC, ret);
		return;
	}

	/* Load FW data for PLL init stage. */
	ret = ap1302_write_fw_window(ap1302_dev, &win_pos, fw_data,
				     ap1302_fw->pll_init_size);
	if (ret) {
		dev_err(dev, "Fail to write AP1302 firware window ret=%d\n", ret);
		return;
	}

	/* Write 2 to bootdata_stage register to apply basic_init_hp
	 * settings and enable PLL.
	 */
	ret = ap1302_write_reg(ap1302_dev, REG_BOOTDATA_STAGE, 0x0002);
	if (ret) {
		dev_err(dev, "Fail to write AP1302[0x%x], ret=%d\n",
			REG_BOOTDATA_STAGE, ret);
		return;
	}

	/* Wait 1ms for PLL to lock. */
	msleep(20);

	/* Load the rest of bootdata content. */
	ret = ap1302_write_fw_window(ap1302_dev, &win_pos,
				     fw_data + ap1302_fw->pll_init_size,
				     ap1302_fw->total_size - ap1302_fw->pll_init_size);
	if (ret) {
		dev_err(dev, "Fail to write AP1302 firware window, ret=%d\n", ret);
		return;
	}

	/* Write 0xFFFF to bootdata_stage register to indicate AP1302 that
	 * the whole bootdata content has been loaded.
	 */
	ret = ap1302_write_reg(ap1302_dev, REG_BOOTDATA_STAGE, 0xFFFF);
	if (ret) {
		dev_err(dev, "Fail to write AP1302[0x%x], ret=%d\n",
			REG_BOOTDATA_STAGE, ret);
		return;
	}

	/* Delay 50ms */
	msleep(50);

	ret = ap1302_read_reg(ap1302_dev, REG_BOOTDATA_CHECKSUM, &regVal);
	if (ret) {
		dev_err(dev, "Fail to read AP1302[0x%x], ret=%d\n",
			REG_BOOTDATA_CHECKSUM, ret);
		return;
	}

	if (regVal != ap1302_fw->checksum) {
		dev_err(dev, "checksum does not match. T:0x%04X F:0x%04X\n",
			ap1302_fw->checksum, regVal);
		return;
	}
	ap1302_write_reg(ap1302_dev, 0x6124, 0x0001);

	release_firmware(fw);
	ap1302_s_stream(sd, 0);

	dev_info(dev, "Load firmware successfully.\n");
}

static void ap1302_reset(struct ap1302_device *ap1302_dev)
{
	gpiod_set_value_cansleep(ap1302_dev->reset, 1);
	udelay(5000);

	gpiod_set_value_cansleep(ap1302_dev->reset, 0);
	msleep(20);
}

static int ap1302_get_regulators(struct ap1302_device *sensor)
{
	int i;

	for (i = 0; i < AP1302_NUM_CONSUMERS; i++)
		sensor->supplies[i].supply = ap1302_supply_name[i];

	return devm_regulator_bulk_get(&sensor->i2c_client->dev,
				       AP1302_NUM_CONSUMERS, sensor->supplies);
}

static void ap1302_set_af_mode(struct ap1302_device *ap1302_dev,
			       enum AP1302_AF_MODE mode)
{
	if (mode == AF_MODE_AUTO) {
		ap1302_write_reg(ap1302_dev, 0x5058, 0x0186);
	} else {
		ap1302_write_reg(ap1302_dev, 0x5058, 0x0183);
		ap1302_write_reg(ap1302_dev, 0x505C, 100);
	}
}

static const struct ap1302_res_info *
ap1302_find_mode(struct ap1302_device *ap1302_dev, u32 width, u32 height)
{
	struct device *dev = &ap1302_dev->i2c_client->dev;
	const struct ap1302_res_info *mode;

	mode = v4l2_find_nearest_size(ap1302_preview_res,
				      ARRAY_SIZE(ap1302_preview_res),
				      hact, vact,
				      width, height);
	if (!mode) {
		dev_err(dev, "Invalid resolution: w/h=(%d, %d)\n", width, height);
		return NULL;
	}

	return mode;
}

/*!
 * ap1302_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int ap1302_s_power(struct v4l2_subdev *sd, int on)
{
	return 0;
}

static int ap1302_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad != 0)
		return -EINVAL;

	if (code->index)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_UYVY8_2X8;
	return 0;
}

static int ap1302_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *format)
{
	struct ap1302_device *ap1302_dev = to_ap1302_device(sd);
	struct v4l2_mbus_framefmt *fmt = &ap1302_dev->fmt;
	struct v4l2_mbus_framefmt *mbus_fmt = &format->format;
	const struct ap1302_res_info *mode;

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&ap1302_dev->lock);
	mode = ap1302_find_mode(ap1302_dev, mbus_fmt->width, mbus_fmt->height);
	if (!mode)
	      return -EINVAL;

	ap1302_dev->mode_change = false;
	if (mode != ap1302_dev->cur_mode) {
		ap1302_dev->cur_mode = mode;
		ap1302_dev->mode_change = true;
	}

	memcpy(fmt, mbus_fmt, sizeof(*fmt));
	fmt->width  = mode->hact;
	fmt->height = mode->vact;
	mutex_unlock(&ap1302_dev->lock);

	return 0;
}

static int ap1302_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *format)
{
	struct ap1302_device *ap1302_dev = to_ap1302_device(sd);
	struct v4l2_mbus_framefmt *fmt = &ap1302_dev->fmt;

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&ap1302_dev->lock);
	format->format = *fmt;
	mutex_unlock(&ap1302_dev->lock);

	return 0;
}

/*!
 * ap1302_enum_frame_size - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ap1302_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct ap1302_device *ap1302_dev = to_ap1302_device(sd);

	if (fse->pad != 0)
		return -EINVAL;

	if (fse->index >= ARRAY_SIZE(ap1302_preview_res))
		return -EINVAL;

	mutex_lock(&ap1302_dev->lock);
	fse->min_width  = ap1302_preview_res[fse->index].width;
	fse->min_height = ap1302_preview_res[fse->index].height;
	fse->max_width  = ap1302_preview_res[fse->index].width;
	fse->max_height = ap1302_preview_res[fse->index].height;
	mutex_unlock(&ap1302_dev->lock);

	return 0;
}

static int ap1302_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct ap1302_device *ap1302_dev = to_ap1302_device(sd);

	mutex_lock(&ap1302_dev->lock);
	fi->interval = ap1302_dev->frame_interval;
	mutex_unlock(&ap1302_dev->lock);

	return 0;
}

static int ap1302_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	return 0;
}

static int ap1302_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ap1302_device *ap1302_dev = to_ap1302_device(sd);
	struct v4l2_mbus_framefmt *fmt = &ap1302_dev->fmt;

	mutex_lock(&ap1302_dev->lock);

	ap1302_write_reg(ap1302_dev, 0x601A, 0x0080);

	if (enable) {
		ap1302_write_reg(ap1302_dev, 0x601A, 0x0380);
		mdelay(50);
		if (ap1302_dev->mode_change) {
			ap1302_write_reg(ap1302_dev, REG_ATOMIC, 0x1);
			ap1302_write_reg(ap1302_dev, REG_PREVIEW_WIDTH, fmt->width);
			ap1302_write_reg(ap1302_dev, REG_PREVIEW_HEIGHT, fmt->height);
			ap1302_write_reg(ap1302_dev, REG_ATOMIC, 0xB);
			mdelay(50);
		}
	}
	else {
		ap1302_write_reg(ap1302_dev, 0x601A, 0x0180);
		mdelay(50);
	}
	mutex_unlock(&ap1302_dev->lock);
	return 0;
}

static int ap1302_link_setup(struct media_entity *entity,
				const struct media_pad *local,
				const struct media_pad *remote, u32 flags)
{
	return 0;
}

static struct v4l2_subdev_video_ops ap1302_subdev_video_ops = {
	.g_frame_interval = ap1302_g_frame_interval,
	.s_frame_interval = ap1302_s_frame_interval,
	.s_stream = ap1302_s_stream,
};

static struct v4l2_subdev_core_ops ap1302_subdev_core_ops = {
	.s_power	= ap1302_s_power,
};

static const struct v4l2_subdev_pad_ops ap1302_subdev_pad_ops = {
	.enum_mbus_code        = ap1302_enum_mbus_code,
	.enum_frame_size       = ap1302_enum_frame_size,
	.get_fmt               = ap1302_get_fmt,
	.set_fmt               = ap1302_set_fmt,
};

static struct v4l2_subdev_ops ap1302_subdev_ops = {
	.core	= &ap1302_subdev_core_ops,
	.video	= &ap1302_subdev_video_ops,
	.pad	= &ap1302_subdev_pad_ops,
};

static const struct media_entity_operations ap1302_sd_media_ops = {
	.link_setup = ap1302_link_setup,
};

/*!
 * ap1302 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int ap1302_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct ap1302_device *ap1302_dev;
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_subdev *sd;
	u16 regVal;
	int ret;

	/* allocate device & init sub device */
	ap1302_dev = devm_kmalloc(dev, sizeof(*ap1302_dev), GFP_KERNEL);
	if (!ap1302_dev) {
		dev_err(dev, "Fail to alloc memory for ap1302\n");
		return -ENOMEM;
	}

	/* Set initial values for the sensor struct. */
	memset(ap1302_dev, 0, sizeof(*ap1302_dev));

	ap1302_dev->i2c_client = client;
	mutex_init(&ap1302_dev->lock);

	/* request reset pin */
	ap1302_dev->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ap1302_dev->reset)) {
		ret = PTR_ERR(ap1302_dev->reset);
		dev_err(dev, "fail to get reset pin for AP1302 ret=%d\n", ret);
		return ret;
	}

	ap1302_dev->isp_en = devm_gpiod_get_optional(dev, "isp_en", GPIOD_OUT_HIGH);
	if (IS_ERR(ap1302_dev->isp_en)) {
		ret = PTR_ERR(ap1302_dev->isp_en);
		dev_err(dev, "fail to get enable isp pin for AP1302 ret=%d\n", ret);
		return ret;
	}
	/*Enable ISP by default*/
	gpiod_set_value_cansleep(ap1302_dev->isp_en, 1);

	ap1302_dev->regmap16 = devm_regmap_init_i2c(client, &ap1302_reg16_config);
	if (IS_ERR(ap1302_dev->regmap16)) {
		ret = PTR_ERR(ap1302_dev->regmap16);
		dev_err(dev, "Failed to allocate 16bit register map: %d\n", ret);
		return ret;
	}

	ret = ap1302_get_regulators(ap1302_dev);
	if (ret) {
		dev_err(dev, "Fail to get regulators for AP1302\n");
		return ret;
	}

	ret = regulator_bulk_enable(AP1302_NUM_CONSUMERS, ap1302_dev->supplies);
	if (ret) {
		dev_err(dev, "Fail to enable regulators for AP1302\n");
		return ret;
	}

	/* reset AP1302 */
	ap1302_reset(ap1302_dev);

	/* chip id */
	ret = ap1302_read_reg(ap1302_dev, REG_CHIP_VERSION, &regVal);
	if (ret || regVal != AP1302_CHIP_ID) {
		dev_err(dev, "Fail to get AP1302 chip id(ret=%d)\n", ret);
		goto fail;
	}
	dev_info(dev, "AP1302 Chip ID is 0x%X\n", regVal);

	/* request firmware for AP1302 */
	ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_UEVENT,
				"imx/camera/ap1302.fw", dev, GFP_KERNEL,
				ap1302_dev, ap1302_fw_handler);
	if (ret) {
		dev_err(dev, "Failed request_firmware_nowait err %d\n", ret);
		goto fail;
	}

	/* config auto AF mode by default */
	ap1302_set_af_mode(ap1302_dev, AF_MODE_AUTO);

	/* format initialization */
	fmt = &ap1302_dev->fmt;
	fmt->code         = MEDIA_BUS_FMT_UYVY8_2X8;
	fmt->colorspace   = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc    = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func    = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->width        = ap1302_preview_res[1].width;
	fmt->height       = ap1302_preview_res[1].height;
	fmt->field        = V4L2_FIELD_NONE;
	ap1302_dev->cur_mode = &ap1302_preview_res[1];

	/* default 60fps */
	ap1302_dev->frame_interval.numerator = 1;
	ap1302_dev->frame_interval.denominator = 60;

	/* Initilize v4l2_subdev */
	sd = &ap1302_dev->subdev;
	sd->flags          |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sd->entity.ops      = &ap1302_sd_media_ops;
	v4l2_i2c_subdev_init(sd, client, &ap1302_subdev_ops);

	ap1302_dev->pads[AP1302_SENS_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, AP1302_SENS_PADS_NUM,
				     ap1302_dev->pads);
	if (ret)
		goto fail;

	ret = v4l2_async_register_subdev_sensor(sd);
	if (ret < 0) {
		dev_err(dev, "Async register failed, ret=%d\n", ret);
		goto fail;
	}

	dev_info(dev, "AP1302 is found\n");
	return 0;

fail:
	regulator_bulk_disable(AP1302_NUM_CONSUMERS, ap1302_dev->supplies);
	return ret;
}

static int ap1302_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ap1302_device *ap1302_dev = to_ap1302_device(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	mutex_destroy(&ap1302_dev->lock);
	return 0;
}

static const struct i2c_device_id ap1302_id[] = {
	{AP1302_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ap1302_id);

static const struct of_device_id ap1302_dt_ids[] = {
	{ .compatible = "onsemi,ap1302" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ap1302_dt_ids);

static struct i2c_driver ap1302_i2c_driver = {
	.probe    = ap1302_probe,
	.remove   = ap1302_remove,
	.id_table = ap1302_id,
	.driver = {
		.owner = THIS_MODULE,
		.name  = AP1302_NAME,
		.of_match_table	= ap1302_dt_ids,
	},
};
module_i2c_driver(ap1302_i2c_driver);

MODULE_AUTHOR("NXP Semiconductor, Inc.");
MODULE_DESCRIPTION("i.MX AP1302 Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("MIPI CSI");
