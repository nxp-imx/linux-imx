/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <mach/mipi_csi2.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-int-device.h>
#include "mxc_v4l2_capture.h"

#define OV8820_VOLTAGE_ANALOG               2800000
#define OV8820_VOLTAGE_DIGITAL_CORE         1500000
#define OV8820_VOLTAGE_DIGITAL_IO           1800000

#define MIN_FPS 15
#define MAX_FPS 30
#define DEFAULT_FPS 30

#define OV8820_XCLK_MIN 6000000
#define OV8820_XCLK_MAX 24000000

enum ov8820_mode {
	ov8820_mode_MIN = 0,
	ov8820_mode_480_480 = 0,
	ov8820_mode_MAX = 0
};

enum ov8820_frame_rate {
	ov8820_15_fps,
	ov8820_30_fps
};

struct reg_value {
	u16 u16RegAddr;
	u8 u8Val;
	u8 u8Mask;
	u32 u32Delay_ms;
};

struct ov8820_mode_info {
	enum ov8820_mode mode;
	u32 width;
	u32 height;
	struct reg_value *init_data_ptr;
	u32 init_data_size;
};

/*!
 * Maintains the information on the current state of the sesor.
 */
static struct sensor {
	const struct ov8820_platform_data *platform_data;
	struct v4l2_int_device *v4l2_int_device;
	struct i2c_client *i2c_client;
	struct v4l2_pix_format pix;
	struct v4l2_captureparm streamcap;
	bool on;

	/* control settings */
	int brightness;
	int hue;
	int contrast;
	int saturation;
	int red;
	int green;
	int blue;
	int ae_mode;

	u32 mclk;
	u8 mclk_source;
	int csi;
} ov8820_data;

static struct reg_value ov8820_setting_30fps_480_480[] = {
	{0x0103, 0x01, 0, 5}, {0x3000, 0x02, 0, 0}, {0x3001, 0x00, 0, 0},
	{0x3002, 0x6c, 0, 0}, {0x300d, 0x00, 0, 0}, {0x301f, 0x09, 0, 0},
	{0x3010, 0x00, 0, 0}, {0x3018, 0x00, 0, 0}, {0x3300, 0x00, 0, 0},
	{0x3500, 0x00, 0, 0}, {0x3503, 0x07, 0, 0}, {0x3509, 0x00, 0, 0},
	{0x3600, 0x08, 0, 0}, {0x3601, 0x44, 0, 0}, {0x3602, 0x75, 0, 0},
	{0x3603, 0x5c, 0, 0}, {0x3604, 0x98, 0, 0}, {0x3605, 0xe9, 0, 0},
	{0x3609, 0xb8, 0, 0}, {0x360a, 0xbc, 0, 0}, {0x360b, 0xb4, 0, 0},
	{0x360c, 0x0d, 0, 0}, {0x3613, 0x02, 0, 0}, {0x3614, 0x0f, 0, 0},
	{0x3615, 0x00, 0, 0}, {0x3616, 0x03, 0, 0}, {0x3617, 0x01, 0, 0},
	{0x3618, 0x00, 0, 0}, {0x3619, 0x00, 0, 0}, {0x361a, 0x00, 0, 0},
	{0x361b, 0x00, 0, 0}, {0x3700, 0x20, 0, 0}, {0x3701, 0x44, 0, 0},
	{0x3702, 0x50, 0, 0}, {0x3703, 0xcc, 0, 0}, {0x3704, 0x19, 0, 0},
	{0x3706, 0x4b, 0, 0}, {0x3707, 0x63, 0, 0}, {0x3708, 0x84, 0, 0},
	{0x3709, 0x40, 0, 0}, {0x370b, 0x01, 0, 0}, {0x370c, 0x50, 0, 0},
	{0x370d, 0x0c, 0, 0}, {0x370e, 0x00, 0, 0}, {0x3711, 0x01, 0, 0},
	{0x3712, 0x9c, 0, 0}, {0x3800, 0x00, 0, 0}, {0x3804, 0x0c, 0, 0},
	{0x3810, 0x00, 0, 0}, {0x3812, 0x00, 0, 0}, {0x3816, 0x02, 0, 0},
	{0x3817, 0x40, 0, 0}, {0x3818, 0x00, 0, 0}, {0x3819, 0x40, 0, 0},
	{0x3d00, 0x00, 0, 0}, {0x3d01, 0x00, 0, 0}, {0x3d02, 0x00, 0, 0},
	{0x3d03, 0x00, 0, 0}, {0x3d04, 0x00, 0, 0}, {0x3d05, 0x00, 0, 0},
	{0x3d06, 0x00, 0, 0}, {0x3d07, 0x00, 0, 0}, {0x3d08, 0x00, 0, 0},
	{0x3d09, 0x00, 0, 0}, {0x3d0a, 0x00, 0, 0}, {0x3d0b, 0x00, 0, 0},
	{0x3d0c, 0x00, 0, 0}, {0x3d0d, 0x00, 0, 0}, {0x3d0e, 0x00, 0, 0},
	{0x3d0f, 0x00, 0, 0}, {0x3d10, 0x00, 0, 0}, {0x3d11, 0x00, 0, 0},
	{0x3d12, 0x00, 0, 0}, {0x3d13, 0x00, 0, 0}, {0x3d14, 0x00, 0, 0},
	{0x3d15, 0x00, 0, 0}, {0x3d16, 0x00, 0, 0}, {0x3d17, 0x00, 0, 0},
	{0x3d18, 0x00, 0, 0}, {0x3d19, 0x00, 0, 0}, {0x3d1a, 0x00, 0, 0},
	{0x3d1b, 0x00, 0, 0}, {0x3d1c, 0x00, 0, 0}, {0x3d1d, 0x00, 0, 0},
	{0x3d1e, 0x00, 0, 0}, {0x3d1f, 0x00, 0, 0}, {0x3d80, 0x00, 0, 0},
	{0x3d81, 0x00, 0, 0}, {0x3d84, 0x00, 0, 0}, {0x3f01, 0xfc, 0, 0},
	{0x3f05, 0x10, 0, 0}, {0x3f06, 0x00, 0, 0}, {0x3f07, 0x00, 0, 0},
	{0x4000, 0x29, 0, 0}, {0x4001, 0x02, 0, 0}, {0x4002, 0x45, 0, 0},
	{0x4003, 0x08, 0, 0}, {0x4004, 0x04, 0, 0}, {0x4005, 0x18, 0, 0},
	{0x4300, 0xff, 0, 0}, {0x4303, 0x00, 0, 0}, {0x4304, 0x08, 0, 0},
	{0x4307, 0x00, 0, 0}, {0x4800, 0x04, 0, 0}, {0x4801, 0x0f, 0, 0},
	{0x4843, 0x02, 0, 0}, {0x5000, 0x00, 0, 0}, {0x5001, 0x00, 0, 0},
	{0x5002, 0x00, 0, 0}, {0x501f, 0x00, 0, 0}, {0x5c00, 0x80, 0, 0},
	{0x5c01, 0x00, 0, 0}, {0x5c02, 0x00, 0, 0}, {0x5c03, 0x00, 0, 0},
	{0x5c04, 0x00, 0, 0}, {0x5c05, 0x00, 0, 0}, {0x5c06, 0x00, 0, 0},
	{0x5c07, 0x80, 0, 0}, {0x5c08, 0x10, 0, 0}, {0x6700, 0x05, 0, 0},
	{0x6701, 0x19, 0, 0}, {0x6702, 0xfd, 0, 0}, {0x6703, 0xd1, 0, 0},
	{0x6704, 0xff, 0, 0}, {0x6705, 0xff, 0, 0}, {0x6800, 0x10, 0, 0},
	{0x6801, 0x02, 0, 0}, {0x6802, 0x90, 0, 0}, {0x6803, 0x10, 0, 0},
	{0x6804, 0x59, 0, 0}, {0x6900, 0x61, 0, 0}, {0x6901, 0x04, 0, 0},
	{0x3612, 0x00, 0, 0}, {0x3617, 0xa1, 0, 0}, {0x3b1f, 0x00, 0, 0},
	{0x3000, 0x12, 0, 0}, {0x3000, 0x16, 0, 0}, {0x3b1f, 0x00, 0, 0},
	{0x3003, 0xce, 0, 0}, {0x3004, 0xd8, 0, 0}, {0x3005, 0x00, 0, 0},
	{0x3006, 0x10, 0, 0}, {0x3007, 0x3b, 0, 0}, {0x3012, 0x80, 0, 0},
	{0x3013, 0x39, 0, 0}, {0x3104, 0x20, 0, 0}, {0x3503, 0x07, 0, 0},
	{0x3500, 0x00, 0, 0}, {0x3501, 0x14, 0, 0}, {0x3502, 0x80, 0, 0},
	{0x350b, 0xff, 0, 0}, {0x3400, 0x04, 0, 0}, {0x3401, 0x00, 0, 0},
	{0x3402, 0x04, 0, 0}, {0x3403, 0x00, 0, 0}, {0x3404, 0x04, 0, 0},
	{0x3405, 0x00, 0, 0}, {0x3406, 0x01, 0, 0}, {0x5001, 0x01, 0, 0},
	{0x5000, 0x06, 0, 0}, {0x0100, 0x00, 0, 0}, {0x3004, 0xbf, 0, 0},
	{0x3005, 0x10, 0, 0}, {0x3006, 0x00, 0, 0}, {0x3011, 0x02, 0, 0},
	{0x370a, 0x74, 0, 0}, {0x3801, 0x08, 0, 0}, {0x3802, 0x00, 0, 0},
	{0x3803, 0x00, 0, 0}, {0x3805, 0xd7, 0, 0}, {0x3806, 0x09, 0, 0},
	{0x3807, 0x97, 0, 0}, {0x3808, 0x01, 0, 0}, {0x3809, 0xe0, 0, 0},
	{0x380a, 0x01, 0, 0}, {0x380b, 0xe0, 0, 0}, {0x380c, 0x0d, 0, 0},
	{0x380d, 0xb0, 0, 0}, {0x380e, 0x02, 0, 0}, {0x380f, 0x7a, 0, 0},
	{0x3811, 0x04, 0, 0}, {0x3813, 0x02, 0, 0}, {0x3814, 0x71, 0, 0},
	{0x3815, 0x71, 0, 0}, {0x3820, 0x00, 0, 0}, {0x3821, 0x16, 0, 0},
	{0x3f00, 0x00, 0, 0}, {0x4600, 0x14, 0, 0}, {0x4601, 0x14, 0, 0},
	{0x4602, 0x00, 0, 0}, {0x4837, 0x1e, 0, 0}, {0x5068, 0x59, 0, 0},
	{0x506a, 0x5a, 0, 0}, {0x0100, 0x01, 0, 0},
};

static struct ov8820_mode_info ov8820_mode_info_data[2][ov8820_mode_MAX + 1] = {
	{
		{ov8820_mode_480_480, 0, 0, NULL, 0},
	},
	{
		{ov8820_mode_480_480,   480,  480,
		ov8820_setting_30fps_480_480,
		ARRAY_SIZE(ov8820_setting_30fps_480_480)},
	},
};

static struct regulator *io_regulator;
static struct regulator *core_regulator;
static struct regulator *analog_regulator;
static struct regulator *gpo_regulator;
static struct fsl_mxc_camera_platform_data *camera_plat;

static int ov8820_probe(struct i2c_client *adapter,
				const struct i2c_device_id *device_id);
static int ov8820_remove(struct i2c_client *client);

static s32 ov8820_read_reg(u16 reg, u8 *val);
static s32 ov8820_write_reg(u16 reg, u8 val);

static const struct i2c_device_id ov8820_id[] = {
	{"ov8820_mipi", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ov8820_id);

static struct i2c_driver ov8820_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "ov8820_mipi",
		  },
	.probe  = ov8820_probe,
	.remove = ov8820_remove,
	.id_table = ov8820_id,
};


static s32 ov8820_write_reg(u16 reg, u8 val)
{
	u8 au8Buf[3] = {0};

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val;

	if (i2c_master_send(ov8820_data.i2c_client, au8Buf, 3) < 0) {
		pr_err("%s:write reg error:reg=%x,val=%x\n",
			__func__, reg, val);
		return -1;
	}

	return 0;
}

static s32 ov8820_read_reg(u16 reg, u8 *val)
{
	u8 au8RegBuf[2] = {0};
	u8 u8RdVal = 0;

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;

	if (2 != i2c_master_send(ov8820_data.i2c_client, au8RegBuf, 2)) {
		pr_err("%s:write reg error:reg=%x\n",
				__func__, reg);
		return -1;
	}

	if (1 != i2c_master_recv(ov8820_data.i2c_client, &u8RdVal, 1)) {
		pr_err("%s:read reg error:reg=%x,val=%x\n",
				__func__, reg, u8RdVal);
		return -1;
	}

	*val = u8RdVal;

	return u8RdVal;
}

static int ov8820_init_mode(enum ov8820_frame_rate frame_rate,
			    enum ov8820_mode mode)
{
	struct reg_value *pModeSetting = NULL;
	s32 i = 0;
	s32 iModeSettingArySize = 0;
	register u32 Delay_ms = 0;
	register u16 RegAddr = 0;
	register u8 Mask = 0;
	register u8 Val = 0;
	u8 RegVal = 0;
	int retval = 0;
	void *mipi_csi2_info;
	u32 mipi_reg;

	if (mode > ov8820_mode_MAX || mode < ov8820_mode_MIN) {
		pr_err("Wrong ov8820 mode detected!\n");
		return -1;
	}

	mipi_csi2_info = mipi_csi2_get_info();

	/* initial mipi dphy */
	if (mipi_csi2_info) {
		if (!mipi_csi2_get_status(mipi_csi2_info))
			mipi_csi2_enable(mipi_csi2_info);

		if (mipi_csi2_get_status(mipi_csi2_info)) {
			mipi_csi2_set_lanes(mipi_csi2_info);
			mipi_csi2_reset(mipi_csi2_info);

			if (ov8820_data.pix.pixelformat == V4L2_PIX_FMT_SBGGR10)
				mipi_csi2_set_datatype(mipi_csi2_info, MIPI_DT_RAW10);
			else
				pr_err("currently this sensor format can not be supported!\n");
		} else {
			pr_err("Can not enable mipi csi2 driver!\n");
			return -1;
		}
	} else {
		printk(KERN_ERR "Fail to get mipi_csi2_info!\n");
		return -1;
	}

	pModeSetting = ov8820_mode_info_data[frame_rate][mode].init_data_ptr;
	iModeSettingArySize =
		ov8820_mode_info_data[frame_rate][mode].init_data_size;

	ov8820_data.pix.width = ov8820_mode_info_data[frame_rate][mode].width;
	ov8820_data.pix.height = ov8820_mode_info_data[frame_rate][mode].height;

	if (ov8820_data.pix.width == 0 || ov8820_data.pix.height == 0 ||
	    pModeSetting == NULL || iModeSettingArySize == 0)
		return -EINVAL;

	for (i = 0; i < iModeSettingArySize; ++i, ++pModeSetting) {
		Delay_ms = pModeSetting->u32Delay_ms;
		RegAddr = pModeSetting->u16RegAddr;
		Val = pModeSetting->u8Val;
		Mask = pModeSetting->u8Mask;

		if (Mask) {
			retval = ov8820_read_reg(RegAddr, &RegVal);
			if (retval < 0)
				goto err;

			RegVal &= ~(u8)Mask;
			Val &= Mask;
			Val |= RegVal;
		}

		retval = ov8820_write_reg(RegAddr, Val);
		if (retval < 0)
			goto err;

		if (Delay_ms)
			msleep(Delay_ms);
	}

	if (mipi_csi2_info) {
		unsigned int i;

		i = 0;

		/* wait for mipi sensor ready */
		mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
		while ((mipi_reg == 0x200) && (i < 10)) {
			mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
			i++;
			msleep(10);
		}

		if (i >= 10) {
			pr_err("mipi csi2 can not receive sensor clk!\n");
			return -1;
		}

		i = 0;

		/* wait for mipi stable */
		mipi_reg = mipi_csi2_get_error1(mipi_csi2_info);
		while ((mipi_reg != 0x0) && (i < 10)) {
			mipi_reg = mipi_csi2_get_error1(mipi_csi2_info);
			i++;
			msleep(10);
		}

		if (i >= 10) {
			pr_err("mipi csi2 can not reveive data correctly!\n");
			return -1;
		}
	}
err:
	return retval;
}

/* --------------- IOCTL functions from v4l2_int_ioctl_desc --------------- */

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	memset(p, 0, sizeof(*p));
	p->u.bt656.clock_curr = ov8820_data.mclk;
	pr_debug("   clock_curr=mclk=%d\n", ov8820_data.mclk);
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.clock_min = OV8820_XCLK_MIN;
	p->u.bt656.clock_max = OV8820_XCLK_MAX;
	p->u.bt656.bt_sync_correct = 1;  /* Indicate external vsync */

	return 0;
}

/*!
 * ioctl_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct sensor *sensor = s->priv;

	if (on && !sensor->on) {
		if (io_regulator)
			if (regulator_enable(io_regulator) != 0)
				return -EIO;
		if (core_regulator)
			if (regulator_enable(core_regulator) != 0)
				return -EIO;
		if (gpo_regulator)
			if (regulator_enable(gpo_regulator) != 0)
				return -EIO;
		if (analog_regulator)
			if (regulator_enable(analog_regulator) != 0)
				return -EIO;
		/* Make sure power on */
		if (camera_plat->pwdn)
			camera_plat->pwdn(0);

	} else if (!on && sensor->on) {
		if (analog_regulator)
			regulator_disable(analog_regulator);
		if (core_regulator)
			regulator_disable(core_regulator);
		if (io_regulator)
			regulator_disable(io_regulator);
		if (gpo_regulator)
			regulator_disable(gpo_regulator);
	}

	sensor->on = on;

	return 0;
}

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor *sensor = s->priv;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	u32 tgt_fps;	/* target frames per secound */
	enum ov8820_frame_rate frame_rate;
	int ret = 0;

	/* Make sure power on */
	if (camera_plat->pwdn)
		camera_plat->pwdn(0);

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		/* Check that the new frame rate is allowed. */
		if ((timeperframe->numerator == 0) ||
		    (timeperframe->denominator == 0)) {
			timeperframe->denominator = DEFAULT_FPS;
			timeperframe->numerator = 1;
		}

		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps > MAX_FPS) {
			timeperframe->denominator = MAX_FPS;
			timeperframe->numerator = 1;
		} else if (tgt_fps < MIN_FPS) {
			timeperframe->denominator = MIN_FPS;
			timeperframe->numerator = 1;
		}

		/* Actual frame rate we use */
		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps == 15)
			frame_rate = ov8820_15_fps;
		else if (tgt_fps == 30)
			frame_rate = ov8820_30_fps;
		else {
			pr_err(" The camera frame rate is not supported!\n");
			return -EINVAL;
		}

		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode =
				(u32)a->parm.capture.capturemode;

		ret = ov8820_init_mode(frame_rate,
				       sensor->streamcap.capturemode);
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_debug("   type is not " \
			"V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			a->type);
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor *sensor = s->priv;

	f->fmt.pix = sensor->pix;

	return 0;
}

/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int ret = 0;

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		vc->value = ov8820_data.brightness;
		break;
	case V4L2_CID_HUE:
		vc->value = ov8820_data.hue;
		break;
	case V4L2_CID_CONTRAST:
		vc->value = ov8820_data.contrast;
		break;
	case V4L2_CID_SATURATION:
		vc->value = ov8820_data.saturation;
		break;
	case V4L2_CID_RED_BALANCE:
		vc->value = ov8820_data.red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		vc->value = ov8820_data.blue;
		break;
	case V4L2_CID_EXPOSURE:
		vc->value = ov8820_data.ae_mode;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int retval = 0;

	pr_debug("In ov8820:ioctl_s_ctrl %d\n",
		 vc->id);

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		break;
	case V4L2_CID_CONTRAST:
		break;
	case V4L2_CID_SATURATION:
		break;
	case V4L2_CID_HUE:
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		break;
	case V4L2_CID_RED_BALANCE:
		break;
	case V4L2_CID_BLUE_BALANCE:
		break;
	case V4L2_CID_GAMMA:
		break;
	case V4L2_CID_EXPOSURE:
		break;
	case V4L2_CID_AUTOGAIN:
		break;
	case V4L2_CID_GAIN:
		break;
	case V4L2_CID_HFLIP:
		break;
	case V4L2_CID_VFLIP:
		break;
	default:
		retval = -EPERM;
		break;
	}

	return retval;
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index > ov8820_mode_MAX)
		return -EINVAL;

	fsize->pixel_format = ov8820_data.pix.pixelformat;
	fsize->discrete.width =
			max(ov8820_mode_info_data[0][fsize->index].width,
			    ov8820_mode_info_data[1][fsize->index].width);
	fsize->discrete.height =
			max(ov8820_mode_info_data[0][fsize->index].height,
			    ov8820_mode_info_data[1][fsize->index].height);
	return 0;
}

/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name, "ov8820_camera");

	return 0;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{

	return 0;
}

/*!
 * ioctl_enum_fmt_cap - V4L2 sensor interface handler for VIDIOC_ENUM_FMT
 * @s: pointer to standard V4L2 device structure
 * @fmt: pointer to standard V4L2 fmt description structure
 *
 * Return 0.
 */
static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
			      struct v4l2_fmtdesc *fmt)
{
	if (fmt->index > ov8820_mode_MAX)
		return -EINVAL;

	fmt->pixelformat = ov8820_data.pix.pixelformat;

	return 0;
}

/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct sensor *sensor = s->priv;
	u32 tgt_xclk;	/* target xclk */
	u32 tgt_fps;	/* target frames per secound */
	int ret;
	enum ov8820_frame_rate frame_rate;
	void *mipi_csi2_info;

	ov8820_data.on = true;

	/* mclk */
	tgt_xclk = ov8820_data.mclk;
	tgt_xclk = min(tgt_xclk, (u32)OV8820_XCLK_MAX);
	tgt_xclk = max(tgt_xclk, (u32)OV8820_XCLK_MIN);
	ov8820_data.mclk = tgt_xclk;

	pr_debug("   Setting mclk to %d MHz\n", tgt_xclk / 1000000);
	set_mclk_rate(&ov8820_data.mclk, ov8820_data.mclk_source);

	/* Default camera frame rate is set in probe */
	tgt_fps = sensor->streamcap.timeperframe.denominator /
		  sensor->streamcap.timeperframe.numerator;

	if (tgt_fps == 15)
		frame_rate = ov8820_15_fps;
	else if (tgt_fps == 30)
		frame_rate = ov8820_30_fps;
	else
		return -EINVAL; /* Only support 15fps or 30fps now. */

	mipi_csi2_info = mipi_csi2_get_info();

	/* enable mipi csi2 */
	if (mipi_csi2_info)
		mipi_csi2_enable(mipi_csi2_info);
	else {
		printk(KERN_ERR "Fail to get mipi_csi2_info!\n");
		return -EPERM;
	}

	ret = ov8820_init_mode(frame_rate,
				sensor->streamcap.capturemode);

	return ret;
}

/*!
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the device when slave detaches to the master.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	void *mipi_csi2_info;

	mipi_csi2_info = mipi_csi2_get_info();

	/* disable mipi csi2 */
	if (mipi_csi2_info)
		if (mipi_csi2_get_status(mipi_csi2_info))
			mipi_csi2_disable(mipi_csi2_info);

	return 0;
}

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc ov8820_ioctl_desc[] = {
	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func*) ioctl_dev_init},
	{vidioc_int_dev_exit_num, ioctl_dev_exit},
	{vidioc_int_s_power_num, (v4l2_int_ioctl_func*) ioctl_s_power},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func*) ioctl_g_ifparm},
/*	{vidioc_int_g_needs_reset_num,
				(v4l2_int_ioctl_func *)ioctl_g_needs_reset}, */
/*	{vidioc_int_reset_num, (v4l2_int_ioctl_func *)ioctl_reset}, */
	{vidioc_int_init_num, (v4l2_int_ioctl_func*) ioctl_init},
	{vidioc_int_enum_fmt_cap_num,
				(v4l2_int_ioctl_func *) ioctl_enum_fmt_cap},
/*	{vidioc_int_try_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_try_fmt_cap}, */
	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func *) ioctl_g_fmt_cap},
/*	{vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func *) ioctl_s_fmt_cap}, */
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *) ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *) ioctl_s_parm},
/*	{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func *)ioctl_queryctrl}, */
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *) ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *) ioctl_s_ctrl},
	{vidioc_int_enum_framesizes_num,
				(v4l2_int_ioctl_func *) ioctl_enum_framesizes},
	{vidioc_int_g_chip_ident_num,
				(v4l2_int_ioctl_func *) ioctl_g_chip_ident},
};

static struct v4l2_int_slave ov8820_slave = {
	.ioctls = ov8820_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(ov8820_ioctl_desc),
};

static struct v4l2_int_device ov8820_int_device = {
	.module = THIS_MODULE,
	.name = "ov8820",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &ov8820_slave,
	},
};

/*!
 * ov8820 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int ov8820_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int retval;
	struct fsl_mxc_camera_platform_data *plat_data = client->dev.platform_data;

	/* Set initial values for the sensor struct. */
	memset(&ov8820_data, 0, sizeof(ov8820_data));
	ov8820_data.mclk = 24000000; /* 6 - 54 MHz, typical 24MHz */
	ov8820_data.mclk = plat_data->mclk;
	ov8820_data.mclk_source = plat_data->mclk_source;
	ov8820_data.csi = plat_data->csi;

	ov8820_data.i2c_client = client;
	ov8820_data.pix.pixelformat = V4L2_PIX_FMT_SBGGR10;
	ov8820_data.pix.width = 480;
	ov8820_data.pix.height = 480;
	ov8820_data.streamcap.capability = V4L2_MODE_HIGHQUALITY |
					   V4L2_CAP_TIMEPERFRAME;
	ov8820_data.streamcap.capturemode = 0;
	ov8820_data.streamcap.timeperframe.denominator = DEFAULT_FPS;
	ov8820_data.streamcap.timeperframe.numerator = 1;

	if (plat_data->io_regulator) {
		io_regulator = regulator_get(&client->dev,
					     plat_data->io_regulator);
		if (!IS_ERR(io_regulator)) {
			regulator_set_voltage(io_regulator,
					      OV8820_VOLTAGE_DIGITAL_IO,
					      OV8820_VOLTAGE_DIGITAL_IO);
			if (regulator_enable(io_regulator) != 0) {
				pr_err("%s:io set voltage error\n", __func__);
				goto err1;
			} else {
				dev_dbg(&client->dev,
					"%s:io set voltage ok\n", __func__);
			}
		} else
			io_regulator = NULL;
	}

	if (plat_data->core_regulator) {
		core_regulator = regulator_get(&client->dev,
					       plat_data->core_regulator);
		if (!IS_ERR(core_regulator)) {
			regulator_set_voltage(core_regulator,
					      OV8820_VOLTAGE_DIGITAL_CORE,
					      OV8820_VOLTAGE_DIGITAL_CORE);
			if (regulator_enable(core_regulator) != 0) {
				pr_err("%s:core set voltage error\n", __func__);
				goto err2;
			} else {
				dev_dbg(&client->dev,
					"%s:core set voltage ok\n", __func__);
			}
		} else
			core_regulator = NULL;
	}

	if (plat_data->analog_regulator) {
		analog_regulator = regulator_get(&client->dev,
						 plat_data->analog_regulator);
		if (!IS_ERR(analog_regulator)) {
			regulator_set_voltage(analog_regulator,
					      OV8820_VOLTAGE_ANALOG,
					      OV8820_VOLTAGE_ANALOG);
			if (regulator_enable(analog_regulator) != 0) {
				pr_err("%s:analog set voltage error\n",
					__func__);
				goto err3;
			} else {
				dev_dbg(&client->dev,
					"%s:analog set voltage ok\n", __func__);
			}
		} else
			analog_regulator = NULL;
	}

	if (plat_data->io_init)
		plat_data->io_init();

	if (plat_data->pwdn)
		plat_data->pwdn(0);

	camera_plat = plat_data;

	ov8820_int_device.priv = &ov8820_data;
	retval = v4l2_int_device_register(&ov8820_int_device);

	return retval;

err3:
	if (core_regulator) {
		regulator_disable(core_regulator);
		regulator_put(core_regulator);
	}
err2:
	if (io_regulator) {
		regulator_disable(io_regulator);
		regulator_put(io_regulator);
	}
err1:
	return -1;
}

/*!
 * ov8820 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int ov8820_remove(struct i2c_client *client)
{
	v4l2_int_device_unregister(&ov8820_int_device);

	if (gpo_regulator) {
		regulator_disable(gpo_regulator);
		regulator_put(gpo_regulator);
	}

	if (analog_regulator) {
		regulator_disable(analog_regulator);
		regulator_put(analog_regulator);
	}

	if (core_regulator) {
		regulator_disable(core_regulator);
		regulator_put(core_regulator);
	}

	if (io_regulator) {
		regulator_disable(io_regulator);
		regulator_put(io_regulator);
	}

	return 0;
}

/*!
 * ov8820 init function
 * Called by insmod ov8820_camera.ko.
 *
 * @return  Error code indicating success or failure
 */
static __init int ov8820_init(void)
{
	u8 err;

	err = i2c_add_driver(&ov8820_i2c_driver);
	if (err != 0)
		pr_err("%s:driver registration failed, error=%d\n",
			__func__, err);

	return err;
}

/*!
 * OV8820 cleanup function
 * Called on rmmod ov8820_camera.ko
 *
 * @return  Error code indicating success or failure
 */
static void __exit ov8820_clean(void)
{
	i2c_del_driver(&ov8820_i2c_driver);
}

module_init(ov8820_init);
module_exit(ov8820_clean);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("OV8820 MIPI Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
