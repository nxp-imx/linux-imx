/*
 * Copyright 2005-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/regulator/regulator.h>
#include <media/v4l2-int-device.h>
#include "mxc_v4l2_capture.h"

#define CAMERA_DBG

#ifdef CAMERA_DBG
	#define CAMERA_TRACE(x) (printk)x
#else
	#define CAMERA_TRACE(x)
#endif

#define OV3640_VOLTAGE_ANALOG               2800000
#define OV3640_VOLTAGE_DIGITAL_CORE         1500000
#define OV3640_VOLTAGE_DIGITAL_IO           1800000


/* Check these values! */
#define MIN_FPS 15
#define MAX_FPS 30
#define DEFAULT_FPS 30

#define OV3640_XCLK_MIN 6000000
#define OV3640_XCLK_MAX 24000000

enum ov3640_mode {
	ov3640_mode_MIN = 0,
	ov3640_mode_VGA_640_480 = 0,
	ov3640_mode_QVGA_320_240 = 1,
	ov3640_mode_QXGA_2048_1536 = 2,
	ov3640_mode_XGA_1024_768 = 3,
	ov3640_mode_MAX = 3
};

enum ov3640_frame_rate {
	ov3640_15_fps,
	ov3640_30_fps
};

struct reg_value {
	u16 u16RegAddr;
	u8 u8Val;
	u8 u8Mask;
	u32 u32Delay_ms;
};

struct ov3640_mode_info {
	enum ov3640_mode mode;
	u32 width;
	u32 height;
	struct reg_value *init_data_ptr;
	u32 init_data_size;
};

/*!
 * Maintains the information on the current state of the sesor.
 */
struct sensor {
	const struct ov3640_platform_data *platform_data;
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
	int csi;
} ov3640_data;

static struct reg_value ov3640_setting_15fps_QXGA_2048_1536[] = {
	{0x3012, 0x80, 0, 0}, {0x304d, 0x45, 0, 0}, {0x30a7, 0x5e, 0, 0},
	{0x3087, 0x16, 0, 0}, {0x309c, 0x1a, 0, 0}, {0x30a2, 0xe4, 0, 0},
	{0x30aa, 0x42, 0, 0}, {0x30b0, 0xff, 0, 0}, {0x30b1, 0xff, 0, 0},
	{0x30b2, 0x10, 0, 0}, {0x300e, 0x32, 0, 0}, {0x300f, 0x21, 0, 0},
	{0x3010, 0x20, 0, 0}, {0x3011, 0x00, 0, 0}, {0x304c, 0x81, 0, 0},
	{0x30d7, 0x10, 0, 0}, {0x30d9, 0x0d, 0, 0}, {0x30db, 0x08, 0, 0},
	{0x3016, 0x82, 0, 0}, {0x3018, 0x38, 0, 0}, {0x3019, 0x30, 0, 0},
	{0x301a, 0x61, 0, 0}, {0x307d, 0x00, 0, 0}, {0x3087, 0x02, 0, 0},
	{0x3082, 0x20, 0, 0}, {0x3015, 0x12, 0, 0}, {0x3014, 0x04, 0, 0},
	{0x3013, 0xf7, 0, 0}, {0x303c, 0x08, 0, 0}, {0x303d, 0x18, 0, 0},
	{0x303e, 0x06, 0, 0}, {0x303f, 0x0c, 0, 0}, {0x3030, 0x62, 0, 0},
	{0x3031, 0x26, 0, 0}, {0x3032, 0xe6, 0, 0}, {0x3033, 0x6e, 0, 0},
	{0x3034, 0xea, 0, 0}, {0x3035, 0xae, 0, 0}, {0x3036, 0xa6, 0, 0},
	{0x3037, 0x6a, 0, 0}, {0x3104, 0x02, 0, 0}, {0x3105, 0xfd, 0, 0},
	{0x3106, 0x00, 0, 0}, {0x3107, 0xff, 0, 0}, {0x3300, 0x12, 0, 0},
	{0x3301, 0xde, 0, 0}, {0x3302, 0xcf, 0, 0}, {0x3312, 0x26, 0, 0},
	{0x3314, 0x42, 0, 0}, {0x3313, 0x2b, 0, 0}, {0x3315, 0x42, 0, 0},
	{0x3310, 0xd0, 0, 0}, {0x3311, 0xbd, 0, 0}, {0x330c, 0x18, 0, 0},
	{0x330d, 0x18, 0, 0}, {0x330e, 0x56, 0, 0}, {0x330f, 0x5c, 0, 0},
	{0x330b, 0x1c, 0, 0}, {0x3306, 0x5c, 0, 0}, {0x3307, 0x11, 0, 0},
	{0x336a, 0x52, 0, 0}, {0x3370, 0x46, 0, 0}, {0x3376, 0x38, 0, 0},
	{0x30b8, 0x20, 0, 0}, {0x30b9, 0x17, 0, 0}, {0x30ba, 0x04, 0, 0},
	{0x30bb, 0x08, 0, 0}, {0x3507, 0x06, 0, 0}, {0x350a, 0x4f, 0, 0},
	{0x3100, 0x02, 0, 0}, {0x3301, 0xde, 0, 0}, {0x3304, 0x00, 0, 0},
	{0x3400, 0x00, 0, 0}, {0x3404, 0x02, 0, 0}, {0x3600, 0xc4, 0, 0},
	{0x3302, 0xef, 0, 0}, {0x3020, 0x01, 0, 0}, {0x3021, 0x1d, 0, 0},
	{0x3022, 0x00, 0, 0}, {0x3023, 0x0a, 0, 0}, {0x3024, 0x08, 0, 0},
	{0x3025, 0x00, 0, 0}, {0x3026, 0x06, 0, 0}, {0x3027, 0x00, 0, 0},
	{0x335f, 0x68, 0, 0}, {0x3360, 0x00, 0, 0}, {0x3361, 0x00, 0, 0},
	{0x3362, 0x68, 0, 0}, {0x3363, 0x00, 0, 0}, {0x3364, 0x00, 0, 0},
	{0x3403, 0x00, 0, 0}, {0x3088, 0x08, 0, 0}, {0x3089, 0x00, 0, 0},
	{0x308a, 0x06, 0, 0}, {0x308b, 0x00, 0, 0}, {0x307c, 0x10, 0, 0},
	{0x3090, 0xc0, 0, 0}, {0x304c, 0x84, 0, 0}, {0x308d, 0x04, 0, 0},
	{0x3086, 0x03, 0, 0}, {0x3086, 0x00, 0, 0}, {0x3012, 0x00, 0, 0},
	{0x3020, 0x01, 0, 0}, {0x3021, 0x1d, 0, 0}, {0x3022, 0x00, 0, 0},
	{0x3023, 0x0a, 0, 0}, {0x3024, 0x08, 0, 0}, {0x3025, 0x18, 0, 0},
	{0x3026, 0x06, 0, 0}, {0x3027, 0x0c, 0, 0}, {0x302a, 0x06, 0, 0},
	{0x302b, 0x20, 0, 0}, {0x3075, 0x44, 0, 0}, {0x300d, 0x00, 0, 0},
	{0x30d7, 0x00, 0, 0}, {0x3069, 0x40, 0, 0}, {0x303e, 0x01, 0, 0},
	{0x303f, 0x80, 0, 0}, {0x3302, 0x20, 0, 0}, {0x335f, 0x68, 0, 0},
	{0x3360, 0x18, 0, 0}, {0x3361, 0x0c, 0, 0}, {0x3362, 0x68, 0, 0},
	{0x3363, 0x08, 0, 0}, {0x3364, 0x04, 0, 0}, {0x3403, 0x42, 0, 0},
	{0x3088, 0x08, 0, 0}, {0x3089, 0x00, 0, 0}, {0x308a, 0x06, 0, 0},
	{0x308b, 0x00, 0, 0},
};

static struct reg_value ov3640_setting_15fps_XGA_1024_768[] = {
	{0x3012, 0x80, 0, 0}, {0x304d, 0x45, 0, 0}, {0x30a7, 0x5e, 0, 0},
	{0x3087, 0x16, 0, 0}, {0x309c, 0x1a, 0, 0}, {0x30a2, 0xe4, 0, 0},
	{0x30aa, 0x42, 0, 0}, {0x30b0, 0xff, 0, 0}, {0x30b1, 0xff, 0, 0},
	{0x30b2, 0x10, 0, 0}, {0x300e, 0x32, 0, 0}, {0x300f, 0x21, 0, 0},
	{0x3010, 0x20, 0, 0}, {0x3011, 0x00, 0, 0}, {0x304c, 0x81, 0, 0},
	{0x3016, 0x82, 0, 0}, {0x3018, 0x38, 0, 0}, {0x3019, 0x30, 0, 0},
	{0x301a, 0x61, 0, 0}, {0x307d, 0x00, 0, 0}, {0x3087, 0x02, 0, 0},
	{0x3082, 0x20, 0, 0}, {0x3015, 0x12, 0, 0}, {0x3014, 0x04, 0, 0},
	{0x3013, 0xf7, 0, 0}, {0x303c, 0x08, 0, 0}, {0x303d, 0x18, 0, 0},
	{0x303e, 0x06, 0, 0}, {0x303f, 0x0c, 0, 0}, {0x3030, 0x62, 0, 0},
	{0x3031, 0x26, 0, 0}, {0x3032, 0xe6, 0, 0}, {0x3033, 0x6e, 0, 0},
	{0x3034, 0xea, 0, 0}, {0x3035, 0xae, 0, 0}, {0x3036, 0xa6, 0, 0},
	{0x3037, 0x6a, 0, 0}, {0x3104, 0x02, 0, 0}, {0x3105, 0xfd, 0, 0},
	{0x3106, 0x00, 0, 0}, {0x3107, 0xff, 0, 0}, {0x3300, 0x12, 0, 0},
	{0x3301, 0xde, 0, 0}, {0x3302, 0xcf, 0, 0}, {0x3312, 0x26, 0, 0},
	{0x3314, 0x42, 0, 0}, {0x3313, 0x2b, 0, 0}, {0x3315, 0x42, 0, 0},
	{0x3310, 0xd0, 0, 0}, {0x3311, 0xbd, 0, 0}, {0x330c, 0x18, 0, 0},
	{0x330d, 0x18, 0, 0}, {0x330e, 0x56, 0, 0}, {0x330f, 0x5c, 0, 0},
	{0x330b, 0x1c, 0, 0}, {0x3306, 0x5c, 0, 0}, {0x3307, 0x11, 0, 0},
	{0x336a, 0x52, 0, 0}, {0x3370, 0x46, 0, 0}, {0x3376, 0x38, 0, 0},
	{0x30b8, 0x20, 0, 0}, {0x30b9, 0x17, 0, 0}, {0x30ba, 0x04, 0, 0},
	{0x30bb, 0x08, 0, 0}, {0x3507, 0x06, 0, 0}, {0x350a, 0x4f, 0, 0},
	{0x3100, 0x02, 0, 0}, {0x3301, 0xde, 0, 0}, {0x3304, 0x00, 0, 0},
	{0x3400, 0x01, 0, 0}, {0x3404, 0x1d, 0, 0}, {0x3600, 0xc4, 0, 0},
	{0x3302, 0xef, 0, 0}, {0x3020, 0x01, 0, 0}, {0x3021, 0x1d, 0, 0},
	{0x3022, 0x00, 0, 0}, {0x3023, 0x0a, 0, 0}, {0x3024, 0x08, 0, 0},
	{0x3025, 0x00, 0, 0}, {0x3026, 0x06, 0, 0}, {0x3027, 0x00, 0, 0},
	{0x335f, 0x68, 0, 0}, {0x3360, 0x00, 0, 0}, {0x3361, 0x00, 0, 0},
	{0x3362, 0x34, 0, 0}, {0x3363, 0x00, 0, 0}, {0x3364, 0x00, 0, 0},
	{0x3403, 0x00, 0, 0}, {0x3088, 0x04, 0, 0}, {0x3089, 0x00, 0, 0},
	{0x308a, 0x03, 0, 0}, {0x308b, 0x00, 0, 0}, {0x307c, 0x10, 0, 0},
	{0x3090, 0xc0, 0, 0}, {0x304c, 0x84, 0, 0}, {0x308d, 0x04, 0, 0},
	{0x3086, 0x03, 0, 0}, {0x3086, 0x00, 0, 0}, {0x3011, 0x01, 0, 0},
};

static struct reg_value ov3640_setting_30fps_XGA_1024_768[] = {
	{0x0, 0x0, 0}
};

static struct reg_value ov3640_setting_15fps_VGA_640_480[] = {
	{0x3012, 0x80, 0, 0}, {0x304d, 0x45, 0, 0}, {0x30a7, 0x5e, 0, 0},
	{0x3087, 0x16, 0, 0}, {0x309c, 0x1a, 0, 0}, {0x30a2, 0xe4, 0, 0},
	{0x30aa, 0x42, 0, 0}, {0x30b0, 0xff, 0, 0}, {0x30b1, 0xff, 0, 0},
	{0x30b2, 0x10, 0, 0}, {0x300e, 0x32, 0, 0}, {0x300f, 0x21, 0, 0},
	{0x3010, 0x20, 0, 0}, {0x3011, 0x00, 0, 0}, {0x304c, 0x81, 0, 0},
	{0x30d7, 0x10, 0, 0}, {0x30d9, 0x0d, 0, 0}, {0x30db, 0x08, 0, 0},
	{0x3016, 0x82, 0, 0}, {0x3018, 0x38, 0, 0}, {0x3019, 0x30, 0, 0},
	{0x301a, 0x61, 0, 0}, {0x307d, 0x00, 0, 0}, {0x3087, 0x02, 0, 0},
	{0x3082, 0x20, 0, 0}, {0x3015, 0x12, 0, 0}, {0x3014, 0x04, 0, 0},
	{0x3013, 0xf7, 0, 0}, {0x303c, 0x08, 0, 0}, {0x303d, 0x18, 0, 0},
	{0x303e, 0x06, 0, 0}, {0x303f, 0x0c, 0, 0}, {0x3030, 0x62, 0, 0},
	{0x3031, 0x26, 0, 0}, {0x3032, 0xe6, 0, 0}, {0x3033, 0x6e, 0, 0},
	{0x3034, 0xea, 0, 0}, {0x3035, 0xae, 0, 0}, {0x3036, 0xa6, 0, 0},
	{0x3037, 0x6a, 0, 0}, {0x3104, 0x02, 0, 0}, {0x3105, 0xfd, 0, 0},
	{0x3106, 0x00, 0, 0}, {0x3107, 0xff, 0, 0}, {0x3300, 0x12, 0, 0},
	{0x3301, 0xde, 0, 0}, {0x3302, 0xcf, 0, 0}, {0x3312, 0x26, 0, 0},
	{0x3314, 0x42, 0, 0}, {0x3313, 0x2b, 0, 0}, {0x3315, 0x42, 0, 0},
	{0x3310, 0xd0, 0, 0}, {0x3311, 0xbd, 0, 0}, {0x330c, 0x18, 0, 0},
	{0x330d, 0x18, 0, 0}, {0x330e, 0x56, 0, 0}, {0x330f, 0x5c, 0, 0},
	{0x330b, 0x1c, 0, 0}, {0x3306, 0x5c, 0, 0}, {0x3307, 0x11, 0, 0},
	{0x336a, 0x52, 0, 0}, {0x3370, 0x46, 0, 0}, {0x3376, 0x38, 0, 0},
	{0x30b8, 0x20, 0, 0}, {0x30b9, 0x17, 0, 0}, {0x30ba, 0x04, 0, 0},
	{0x30bb, 0x08, 0, 0}, {0x3507, 0x06, 0, 0}, {0x350a, 0x4f, 0, 0},
	{0x3100, 0x02, 0, 0}, {0x3301, 0xde, 0, 0}, {0x3304, 0x00, 0, 0},
	{0x3400, 0x00, 0, 0}, {0x3404, 0x42, 0, 0}, {0x3600, 0xc4, 0, 0},
	{0x3302, 0xef, 0, 0}, {0x3020, 0x01, 0, 0}, {0x3021, 0x1d, 0, 0},
	{0x3022, 0x00, 0, 0}, {0x3023, 0x0a, 0, 0}, {0x3024, 0x08, 0, 0},
	{0x3025, 0x00, 0, 0}, {0x3026, 0x06, 0, 0}, {0x3027, 0x00, 0, 0},
	{0x335f, 0x68, 0, 0}, {0x3360, 0x00, 0, 0}, {0x3361, 0x00, 0, 0},
	{0x3362, 0x12, 0, 0}, {0x3363, 0x80, 0, 0}, {0x3364, 0xe0, 0, 0},
	{0x3403, 0x00, 0, 0}, {0x3088, 0x02, 0, 0}, {0x3089, 0x80, 0, 0},
	{0x308a, 0x01, 0, 0}, {0x308b, 0xe0, 0, 0}, {0x307c, 0x10, 0, 0},
	{0x3090, 0xc0, 0, 0}, {0x304c, 0x84, 0, 0}, {0x308d, 0x04, 0, 0},
	{0x3086, 0x03, 0, 0}, {0x3086, 0x00, 0, 0}, {0x3011, 0x00, 0, 0},
};

static struct reg_value ov3640_setting_30fps_VGA_640_480[] = {
	{0x3012, 0x80, 0, 0}, {0x304d, 0x45, 0, 0}, {0x30a7, 0x5e, 0, 0},
	{0x3087, 0x16, 0, 0}, {0x309c, 0x1a, 0, 0}, {0x30a2, 0xe4, 0, 0},
	{0x30aa, 0x42, 0, 0}, {0x30b0, 0xff, 0, 0}, {0x30b1, 0xff, 0, 0},
	{0x30b2, 0x10, 0, 0}, {0x300e, 0x32, 0, 0}, {0x300f, 0x21, 0, 0},
	{0x3010, 0x20, 0, 0}, {0x3011, 0x01, 0, 0}, {0x304c, 0x82, 0, 0},
	{0x30d7, 0x10, 0, 0}, {0x30d9, 0x0d, 0, 0}, {0x30db, 0x08, 0, 0},
	{0x3016, 0x82, 0, 0}, {0x3018, 0x38, 0, 0}, {0x3019, 0x30, 0, 0},
	{0x301a, 0x61, 0, 0}, {0x307d, 0x00, 0, 0}, {0x3087, 0x02, 0, 0},
	{0x3082, 0x20, 0, 0}, {0x3015, 0x12, 0, 0}, {0x3014, 0x0c, 0, 0},
	{0x3013, 0xf7, 0, 0}, {0x303c, 0x08, 0, 0}, {0x303d, 0x18, 0, 0},
	{0x303e, 0x06, 0, 0}, {0x303f, 0x0c, 0, 0}, {0x3030, 0x62, 0, 0},
	{0x3031, 0x26, 0, 0}, {0x3032, 0xe6, 0, 0}, {0x3033, 0x6e, 0, 0},
	{0x3034, 0xea, 0, 0}, {0x3035, 0xae, 0, 0}, {0x3036, 0xa6, 0, 0},
	{0x3037, 0x6a, 0, 0}, {0x3104, 0x02, 0, 0}, {0x3105, 0xfd, 0, 0},
	{0x3106, 0x00, 0, 0}, {0x3107, 0xff, 0, 0}, {0x3300, 0x12, 0, 0},
	{0x3301, 0xde, 0, 0}, {0x3302, 0xcf, 0, 0}, {0x3312, 0x26, 0, 0},
	{0x3314, 0x42, 0, 0}, {0x3313, 0x2b, 0, 0}, {0x3315, 0x42, 0, 0},
	{0x3310, 0xd0, 0, 0}, {0x3311, 0xbd, 0, 0}, {0x330c, 0x18, 0, 0},
	{0x330d, 0x18, 0, 0}, {0x330e, 0x56, 0, 0}, {0x330f, 0x5c, 0, 0},
	{0x330b, 0x1c, 0, 0}, {0x3306, 0x5c, 0, 0}, {0x3307, 0x11, 0, 0},
	{0x336a, 0x52, 0, 0}, {0x3370, 0x46, 0, 0}, {0x3376, 0x38, 0, 0},
	{0x3300, 0x13, 0, 0}, {0x30b8, 0x20, 0, 0}, {0x30b9, 0x17, 0, 0},
	{0x30ba, 0x04, 0, 0}, {0x30bb, 0x08, 0, 0}, {0x3100, 0x02, 0, 0},
	{0x3301, 0x10, 0x30, 0}, {0x3304, 0x00, 0x03, 0}, {0x3400, 0x00, 0, 0},
	{0x3404, 0x02, 0, 0}, {0x3600, 0xc0, 0, 0}, {0x308d, 0x04, 0, 0},
	{0x3086, 0x03, 0, 0}, {0x3086, 0x00, 0, 0}, {0x3012, 0x10, 0, 0},
	{0x3023, 0x06, 0, 0}, {0x3026, 0x03, 0, 0}, {0x3027, 0x04, 0, 0},
	{0x302a, 0x03, 0, 0}, {0x302b, 0x10, 0, 0}, {0x3075, 0x24, 0, 0},
	{0x300d, 0x01, 0, 0}, {0x30d7, 0x80, 0x80, 0}, {0x3069, 0x00, 0x40, 0},
	{0x303e, 0x00, 0, 0}, {0x303f, 0xc0, 0, 0}, {0x3302, 0x20, 0x20, 0},
	{0x335f, 0x34, 0, 0}, {0x3360, 0x0c, 0, 0}, {0x3361, 0x04, 0, 0},
	{0x3362, 0x12, 0, 0}, {0x3363, 0x88, 0, 0}, {0x3364, 0xe4, 0, 0},
	{0x3403, 0x42, 0, 0}, {0x3088, 0x02, 0, 0}, {0x3089, 0x80, 0, 0},
	{0x308a, 0x01, 0, 0}, {0x308b, 0xe0, 0, 0}, {0x3362, 0x12, 0, 0},
	{0x3363, 0x88, 0, 0}, {0x3364, 0xe4, 0, 0}, {0x3403, 0x42, 0, 0},
	{0x3088, 0x02, 0, 0}, {0x3089, 0x80, 0, 0}, {0x308a, 0x01, 0, 0},
	{0x308b, 0xe0, 0, 0}, {0x300e, 0x37, 0, 0}, {0x300f, 0xe1, 0, 0},
	{0x3010, 0x22, 0, 0}, {0x3011, 0x01, 0, 0}, {0x304c, 0x84, 0, 0},
	{0x3014, 0x04, 0, 0}, {0x3015, 0x02, 0, 0}, {0x302e, 0x00, 0, 0},
	{0x302d, 0x00, 0, 0},
};

static struct reg_value ov3640_setting_15fps_QVGA_320_240[] = {
	{0x3012, 0x80, 0, 0}, {0x304d, 0x45, 0, 0}, {0x30a7, 0x5e, 0, 0},
	{0x3087, 0x16, 0, 0}, {0x309c, 0x1a, 0, 0}, {0x30a2, 0xe4, 0, 0},
	{0x30aa, 0x42, 0, 0}, {0x30b0, 0xff, 0, 0}, {0x30b1, 0xff, 0, 0},
	{0x30b2, 0x10, 0, 0}, {0x300e, 0x32, 0, 0}, {0x300f, 0x21, 0, 0},
	{0x3010, 0x20, 0, 0}, {0x3011, 0x00, 0, 0}, {0x304c, 0x81, 0, 0},
	{0x30d7, 0x10, 0, 0}, {0x30d9, 0x0d, 0, 0}, {0x30db, 0x08, 0, 0},
	{0x3016, 0x82, 0, 0}, {0x3018, 0x38, 0, 0}, {0x3019, 0x30, 0, 0},
	{0x301a, 0x61, 0, 0}, {0x307d, 0x00, 0, 0}, {0x3087, 0x02, 0, 0},
	{0x3082, 0x20, 0, 0}, {0x3015, 0x12, 0, 0}, {0x3014, 0x04, 0, 0},
	{0x3013, 0xf7, 0, 0}, {0x303c, 0x08, 0, 0}, {0x303d, 0x18, 0, 0},
	{0x303e, 0x06, 0, 0}, {0x303f, 0x0c, 0, 0}, {0x3030, 0x62, 0, 0},
	{0x3031, 0x26, 0, 0}, {0x3032, 0xe6, 0, 0}, {0x3033, 0x6e, 0, 0},
	{0x3034, 0xea, 0, 0}, {0x3035, 0xae, 0, 0}, {0x3036, 0xa6, 0, 0},
	{0x3037, 0x6a, 0, 0}, {0x3104, 0x02, 0, 0}, {0x3105, 0xfd, 0, 0},
	{0x3106, 0x00, 0, 0}, {0x3107, 0xff, 0, 0}, {0x3300, 0x12, 0, 0},
	{0x3301, 0xde, 0, 0}, {0x3302, 0xcf, 0, 0}, {0x3312, 0x26, 0, 0},
	{0x3314, 0x42, 0, 0}, {0x3313, 0x2b, 0, 0}, {0x3315, 0x42, 0, 0},
	{0x3310, 0xd0, 0, 0}, {0x3311, 0xbd, 0, 0}, {0x330c, 0x18, 0, 0},
	{0x330d, 0x18, 0, 0}, {0x330e, 0x56, 0, 0}, {0x330f, 0x5c, 0, 0},
	{0x330b, 0x1c, 0, 0}, {0x3306, 0x5c, 0, 0}, {0x3307, 0x11, 0, 0},
	{0x336a, 0x52, 0, 0}, {0x3370, 0x46, 0, 0}, {0x3376, 0x38, 0, 0},
	{0x30b8, 0x20, 0, 0}, {0x30b9, 0x17, 0, 0}, {0x30ba, 0x04, 0, 0},
	{0x30bb, 0x08, 0, 0}, {0x3507, 0x06, 0, 0}, {0x350a, 0x4f, 0, 0},
	{0x3100, 0x02, 0, 0}, {0x3301, 0xde, 0, 0}, {0x3304, 0x00, 0, 0},
	{0x3400, 0x00, 0, 0}, {0x3404, 0x42, 0, 0}, {0x3600, 0xc4, 0, 0},
	{0x3302, 0xef, 0, 0}, {0x3020, 0x01, 0, 0}, {0x3021, 0x1d, 0, 0},
	{0x3022, 0x00, 0, 0}, {0x3023, 0x0a, 0, 0}, {0x3024, 0x08, 0, 0},
	{0x3025, 0x00, 0, 0}, {0x3026, 0x06, 0, 0}, {0x3027, 0x00, 0, 0},
	{0x335f, 0x68, 0, 0}, {0x3360, 0x00, 0, 0}, {0x3361, 0x00, 0, 0},
	{0x3362, 0x01, 0, 0}, {0x3363, 0x40, 0, 0}, {0x3364, 0xf0, 0, 0},
	{0x3403, 0x00, 0, 0}, {0x3088, 0x01, 0, 0}, {0x3089, 0x40, 0, 0},
	{0x308a, 0x00, 0, 0}, {0x308b, 0xf0, 0, 0}, {0x307c, 0x10, 0, 0},
	{0x3090, 0xc0, 0, 0}, {0x304c, 0x84, 0, 0}, {0x308d, 0x04, 0, 0},
	{0x3086, 0x03, 0, 0}, {0x3086, 0x00, 0, 0}, {0x3011, 0x01, 0, 0},
};

static struct reg_value ov3640_setting_30fps_QVGA_320_240[] = {
	{0x3012, 0x80, 0, 0}, {0x304d, 0x45, 0, 0}, {0x30a7, 0x5e, 0, 0},
	{0x3087, 0x16, 0, 0}, {0x309c, 0x1a, 0, 0}, {0x30a2, 0xe4, 0, 0},
	{0x30aa, 0x42, 0, 0}, {0x30b0, 0xff, 0, 0}, {0x30b1, 0xff, 0, 0},
	{0x30b2, 0x10, 0, 0}, {0x300e, 0x32, 0, 0}, {0x300f, 0x21, 0, 0},
	{0x3010, 0x20, 0, 0}, {0x3011, 0x01, 0, 0}, {0x304c, 0x82, 0, 0},
	{0x30d7, 0x10, 0, 0}, {0x30d9, 0x0d, 0, 0}, {0x30db, 0x08, 0, 0},
	{0x3016, 0x82, 0, 0}, {0x3018, 0x38, 0, 0}, {0x3019, 0x30, 0, 0},
	{0x301a, 0x61, 0, 0}, {0x307d, 0x00, 0, 0}, {0x3087, 0x02, 0, 0},
	{0x3082, 0x20, 0, 0}, {0x3015, 0x12, 0, 0}, {0x3014, 0x0c, 0, 0},
	{0x3013, 0xf7, 0, 0}, {0x303c, 0x08, 0, 0}, {0x303d, 0x18, 0, 0},
	{0x303e, 0x06, 0, 0}, {0x303f, 0x0c, 0, 0}, {0x3030, 0x62, 0, 0},
	{0x3031, 0x26, 0, 0}, {0x3032, 0xe6, 0, 0}, {0x3033, 0x6e, 0, 0},
	{0x3034, 0xea, 0, 0}, {0x3035, 0xae, 0, 0}, {0x3036, 0xa6, 0, 0},
	{0x3037, 0x6a, 0, 0}, {0x3104, 0x02, 0, 0}, {0x3105, 0xfd, 0, 0},
	{0x3106, 0x00, 0, 0}, {0x3107, 0xff, 0, 0}, {0x3300, 0x12, 0, 0},
	{0x3301, 0xde, 0, 0}, {0x3302, 0xcf, 0, 0}, {0x3312, 0x26, 0, 0},
	{0x3314, 0x42, 0, 0}, {0x3313, 0x2b, 0, 0}, {0x3315, 0x42, 0, 0},
	{0x3310, 0xd0, 0, 0}, {0x3311, 0xbd, 0, 0}, {0x330c, 0x18, 0, 0},
	{0x330d, 0x18, 0, 0}, {0x330e, 0x56, 0, 0}, {0x330f, 0x5c, 0, 0},
	{0x330b, 0x1c, 0, 0}, {0x3306, 0x5c, 0, 0}, {0x3307, 0x11, 0, 0},
	{0x336a, 0x52, 0, 0}, {0x3370, 0x46, 0, 0}, {0x3376, 0x38, 0, 0},
	{0x3300, 0x13, 0, 0}, {0x30b8, 0x20, 0, 0}, {0x30b9, 0x17, 0, 0},
	{0x30ba, 0x04, 0, 0}, {0x30bb, 0x08, 0, 0}, {0x3100, 0x02, 0, 0},
	{0x3301, 0x10, 0x30, 0}, {0x3304, 0x00, 0x03, 0}, {0x3400, 0x00, 0, 0},
	{0x3404, 0x02, 0, 0}, {0x3600, 0xc0, 0, 0}, {0x308d, 0x04, 0, 0},
	{0x3086, 0x03, 0, 0}, {0x3086, 0x00, 0, 0}, {0x3012, 0x10, 0, 0},
	{0x3023, 0x06, 0, 0}, {0x3026, 0x03, 0, 0}, {0x3027, 0x04, 0, 0},
	{0x302a, 0x03, 0, 0}, {0x302b, 0x10, 0, 0}, {0x3075, 0x24, 0, 0},
	{0x300d, 0x01, 0, 0}, {0x30d7, 0x80, 0x80, 0}, {0x3069, 0x00, 0x40, 0},
	{0x303e, 0x00, 0, 0}, {0x303f, 0xc0, 0, 0}, {0x3302, 0x20, 0x20, 0},
	{0x335f, 0x34, 0, 0}, {0x3360, 0x0c, 0, 0}, {0x3361, 0x04, 0, 0},
	{0x3362, 0x34, 0, 0}, {0x3363, 0x08, 0, 0}, {0x3364, 0x04, 0, 0},
	{0x3403, 0x42, 0, 0}, {0x3088, 0x04, 0, 0}, {0x3089, 0x00, 0, 0},
	{0x308a, 0x03, 0, 0}, {0x308b, 0x00, 0, 0}, {0x3362, 0x12, 0, 0},
	{0x3363, 0x88, 0, 0}, {0x3364, 0xe4, 0, 0}, {0x3403, 0x42, 0, 0},
	{0x3088, 0x02, 0, 0}, {0x3089, 0x80, 0, 0}, {0x308a, 0x01, 0, 0},
	{0x308b, 0xe0, 0, 0}, {0x300e, 0x37, 0, 0}, {0x300f, 0xe1, 0, 0},
	{0x3010, 0x22, 0, 0}, {0x3011, 0x01, 0, 0}, {0x304c, 0x84, 0, 0},
};

static struct ov3640_mode_info ov3640_mode_info_data[2][ov3640_mode_MAX + 1] = {
	{
		{ov3640_mode_VGA_640_480,    640,  480,
		ov3640_setting_15fps_VGA_640_480,
		ARRAY_SIZE(ov3640_setting_15fps_VGA_640_480)},
		{ov3640_mode_QVGA_320_240,   320,  240,
		ov3640_setting_15fps_QVGA_320_240,
		ARRAY_SIZE(ov3640_setting_15fps_QVGA_320_240)},
		{ov3640_mode_XGA_1024_768,   1024, 768,
		ov3640_setting_15fps_XGA_1024_768,
		ARRAY_SIZE(ov3640_setting_15fps_XGA_1024_768)},
		{ov3640_mode_QXGA_2048_1536, 2048, 1536,
		ov3640_setting_15fps_QXGA_2048_1536,
		ARRAY_SIZE(ov3640_setting_15fps_QXGA_2048_1536)},
	},
	{
		{ov3640_mode_VGA_640_480,    640,  480,
		ov3640_setting_30fps_VGA_640_480,
		ARRAY_SIZE(ov3640_setting_30fps_VGA_640_480)},
		{ov3640_mode_QVGA_320_240,   320,  240,
		ov3640_setting_30fps_QVGA_320_240,
		ARRAY_SIZE(ov3640_setting_30fps_QVGA_320_240)},
		{ov3640_mode_XGA_1024_768,   1024, 768,
		ov3640_setting_30fps_XGA_1024_768,
		ARRAY_SIZE(ov3640_setting_30fps_XGA_1024_768)},
		{ov3640_mode_QXGA_2048_1536, 0, 0, NULL, 0},
	},
};

static struct regulator *io_regulator;
static struct regulator *core_regulator;
static struct regulator *analog_regulator;
static struct regulator *gpo_regulator;

static int ov3640_probe(struct i2c_client *adapter,
				const struct i2c_device_id *device_id);
static int ov3640_remove(struct i2c_client *client);

static s32 ov3640_read_reg(u16 reg, u8 *val);
static s32 ov3640_write_reg(u16 reg, u8 val);

static const struct i2c_device_id ov3640_id[] = {
	{"ov3640", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ov3640_id);

static struct i2c_driver ov3640_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "ov3640",
		  },
	.probe  = ov3640_probe,
	.remove = ov3640_remove,
	.id_table = ov3640_id,
};

extern void gpio_sensor_active(unsigned int csi_index);
extern void gpio_sensor_inactive(unsigned int csi);

static s32 ov3640_write_reg(u16 reg, u8 val)
{
	u8 au8Buf[3] = {0};

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val;

	if (i2c_master_send(ov3640_data.i2c_client, au8Buf, 3) < 0) {
		pr_err("%s:write reg error:reg=%x,val=%x\n",
			__func__, reg, val);
		return -1;
	}

	return 0;
}

static s32 ov3640_read_reg(u16 reg, u8 *val)
{
	u8 au8RegBuf[2] = {0};
	u8 u8RdVal = 0;

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;

	if (2 != i2c_master_send(ov3640_data.i2c_client, au8RegBuf, 2)) {
		pr_err("%s:write reg error:reg=%x\n",
				__func__, reg);
		return -1;
	}

	if (1 != i2c_master_recv(ov3640_data.i2c_client, &u8RdVal, 1)) {
		pr_err("%s:read reg error:reg=%x,val=%x\n",
				__func__, reg, u8RdVal);
		return -1;
	}

	*val = u8RdVal;

	return u8RdVal;
}

static int ov3640_init_mode(enum ov3640_frame_rate frame_rate,
			    enum ov3640_mode mode)
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

	CAMERA_TRACE(("CAMERA_DBG Entry: ov3640_init_mode\n"));

	if (mode > ov3640_mode_MAX || mode < ov3640_mode_MIN) {
		pr_err("Wrong ov3640 mode detected!\n");
		return -1;
	}

	pModeSetting = ov3640_mode_info_data[frame_rate][mode].init_data_ptr;
	iModeSettingArySize =
		ov3640_mode_info_data[frame_rate][mode].init_data_size;

	ov3640_data.pix.width = ov3640_mode_info_data[frame_rate][mode].width;
	ov3640_data.pix.height = ov3640_mode_info_data[frame_rate][mode].height;

	for (i = 0; i < iModeSettingArySize; ++i, ++pModeSetting) {
		Delay_ms = pModeSetting->u32Delay_ms;
		RegAddr = pModeSetting->u16RegAddr;
		Val = pModeSetting->u8Val;
		Mask = pModeSetting->u8Mask;

		if (Mask) {
			retval = ov3640_read_reg(RegAddr, &RegVal);
			if (retval < 0)
				goto err;

			RegVal &= ~(u8)Mask;
			Val &= Mask;
			Val |= RegVal;
		}

		retval = ov3640_write_reg(RegAddr, Val);
		if (retval < 0)
			goto err;

		if (Delay_ms)
			msleep(Delay_ms);
	}
err:
	CAMERA_TRACE(("CAMERA_DBG Exit: ov3640_init_mode\n"));

	return retval;
}

/* --------------- IOCTL functions from v4l2_int_ioctl_desc --------------- */

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	CAMERA_TRACE(("In ov3640:ioctl_g_ifparm\n"));
	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	memset(p, 0, sizeof(*p));
	p->u.bt656.clock_curr = ov3640_data.mclk;
	pr_debug("   clock_curr=mclk=%d\n", ov3640_data.mclk);
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.clock_min = OV3640_XCLK_MIN;
	p->u.bt656.clock_max = OV3640_XCLK_MAX;
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

	CAMERA_TRACE(("In ov3640:ioctl_s_power\n"));

	if (on && !sensor->on) {
		gpio_sensor_active(ov3640_data.csi);
		if (!IS_ERR_VALUE((unsigned long)io_regulator))
			if (regulator_enable(io_regulator) != 0)
				return -EIO;
		if (!IS_ERR_VALUE((unsigned long)core_regulator))
			if (regulator_enable(core_regulator) != 0)
				return -EIO;
		if (!IS_ERR_VALUE((unsigned long)gpo_regulator))
			if (regulator_enable(gpo_regulator) != 0)
				return -EIO;
		if (!IS_ERR_VALUE((unsigned long)analog_regulator))
			if (regulator_enable(analog_regulator) != 0)
				return -EIO;
	} else if (!on && sensor->on) {
		if (!IS_ERR_VALUE((unsigned long)analog_regulator))
			regulator_disable(analog_regulator);
		if (!IS_ERR_VALUE((unsigned long)core_regulator))
			regulator_disable(core_regulator);
		if (!IS_ERR_VALUE((unsigned long)io_regulator))
			regulator_disable(io_regulator);
		if (!IS_ERR_VALUE((unsigned long)gpo_regulator))
			regulator_disable(gpo_regulator);
		gpio_sensor_inactive(ov3640_data.csi);
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

	CAMERA_TRACE(("In ov3640:ioctl_g_parm\n"));
	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		CAMERA_TRACE(("   type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n"));
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
		CAMERA_TRACE(("   type is not " \
			"V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			a->type));
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
	enum ov3640_frame_rate frame_rate;
	int ret = 0;

	CAMERA_TRACE(("In ov3640:ioctl_s_parm\n"));
	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		CAMERA_TRACE(("   type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n"));

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
			frame_rate = ov3640_15_fps;
		else if (tgt_fps == 30)
			frame_rate = ov3640_30_fps;
		else {
			pr_err(" The camera frame rate is not supported!\n");
			return -EINVAL;
		}

		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode =
				(u32)a->parm.capture.capturemode;

		ret = ov3640_init_mode(frame_rate,
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

	CAMERA_TRACE(("In ov3640:ioctl_g_fmt_cap.\n"));

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

	CAMERA_TRACE(("In ov3640:ioctl_g_ctrl\n"));
	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		vc->value = ov3640_data.brightness;
		break;
	case V4L2_CID_HUE:
		vc->value = ov3640_data.hue;
		break;
	case V4L2_CID_CONTRAST:
		vc->value = ov3640_data.contrast;
		break;
	case V4L2_CID_SATURATION:
		vc->value = ov3640_data.saturation;
		break;
	case V4L2_CID_RED_BALANCE:
		vc->value = ov3640_data.red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		vc->value = ov3640_data.blue;
		break;
	case V4L2_CID_EXPOSURE:
		vc->value = ov3640_data.ae_mode;
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

	pr_debug("In ov3640:ioctl_s_ctrl %d\n",
		 vc->id);

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		CAMERA_TRACE(("   V4L2_CID_BRIGHTNESS\n"));
		break;
	case V4L2_CID_CONTRAST:
		CAMERA_TRACE(("   V4L2_CID_CONTRAST\n"));
		break;
	case V4L2_CID_SATURATION:
		CAMERA_TRACE(("   V4L2_CID_SATURATION\n"));
		break;
	case V4L2_CID_HUE:
		CAMERA_TRACE(("   V4L2_CID_HUE\n"));
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		CAMERA_TRACE(("   V4L2_CID_AUTO_WHITE_BALANCE\n"));
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		CAMERA_TRACE(("   V4L2_CID_DO_WHITE_BALANCE\n"));
		break;
	case V4L2_CID_RED_BALANCE:
		CAMERA_TRACE(("   V4L2_CID_RED_BALANCE\n"));
		break;
	case V4L2_CID_BLUE_BALANCE:
		CAMERA_TRACE(("   V4L2_CID_BLUE_BALANCE\n"));
		break;
	case V4L2_CID_GAMMA:
		CAMERA_TRACE(("   V4L2_CID_GAMMA\n"));
		break;
	case V4L2_CID_EXPOSURE:
		CAMERA_TRACE(("   V4L2_CID_EXPOSURE\n"));
		break;
	case V4L2_CID_AUTOGAIN:
		CAMERA_TRACE(("   V4L2_CID_AUTOGAIN\n"));
		break;
	case V4L2_CID_GAIN:
		CAMERA_TRACE(("   V4L2_CID_GAIN\n"));
		break;
	case V4L2_CID_HFLIP:
		CAMERA_TRACE(("   V4L2_CID_HFLIP\n"));
		break;
	case V4L2_CID_VFLIP:
		CAMERA_TRACE(("   V4L2_CID_VFLIP\n"));
		break;
	default:
		CAMERA_TRACE(("   Default case\n"));
		retval = -EPERM;
		break;
	}

	return retval;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	CAMERA_TRACE(("In ov3640:ioctl_init\n"));

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
	enum ov3640_frame_rate frame_rate;

	CAMERA_TRACE(("In ov3640:ioctl_dev_init\n"));

	gpio_sensor_active(ov3640_data.csi);
	ov3640_data.on = true;

	/* mclk */
	tgt_xclk = ov3640_data.mclk;
	tgt_xclk = min(tgt_xclk, (u32)OV3640_XCLK_MAX);
	tgt_xclk = max(tgt_xclk, (u32)OV3640_XCLK_MIN);
	ov3640_data.mclk = tgt_xclk;

	pr_debug("   Setting mclk to %d MHz\n", tgt_xclk / 1000000);
	set_mclk_rate(&ov3640_data.mclk, ov3640_data.csi);

	/* Default camera frame rate is set in probe */
	tgt_fps = sensor->streamcap.timeperframe.denominator /
		  sensor->streamcap.timeperframe.numerator;

	if (tgt_fps == 15)
		frame_rate = ov3640_15_fps;
	else if (tgt_fps == 30)
		frame_rate = ov3640_30_fps;
	else
		return -EINVAL; /* Only support 15fps or 30fps now. */

	return ov3640_init_mode(frame_rate,
				sensor->streamcap.capturemode);
}

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc ov3640_ioctl_desc[] = {
	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func *)ioctl_dev_init},
/*	{vidioc_int_dev_exit_num, (v4l2_int_ioctl_func *)ioctl_dev_exit}, */
	{vidioc_int_s_power_num, (v4l2_int_ioctl_func *)ioctl_s_power},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func *)ioctl_g_ifparm},
/*	{vidioc_int_g_needs_reset_num,
				(v4l2_int_ioctl_func *)ioctl_g_needs_reset}, */
/*	{vidioc_int_reset_num, (v4l2_int_ioctl_func *)ioctl_reset}, */
	{vidioc_int_init_num, (v4l2_int_ioctl_func *)ioctl_init},
/*	{vidioc_int_enum_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_enum_fmt_cap}, */
/*	{vidioc_int_try_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_try_fmt_cap}, */
	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_g_fmt_cap},
/*	{vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_s_fmt_cap}, */
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *)ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *)ioctl_s_parm},
/*	{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func *)ioctl_queryctrl}, */
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *)ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *)ioctl_s_ctrl},
};

static struct v4l2_int_slave ov3640_slave = {
	.ioctls = ov3640_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(ov3640_ioctl_desc),
};

static struct v4l2_int_device ov3640_int_device = {
	.module = THIS_MODULE,
	.name = "ov3640",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &ov3640_slave,
	},
};

/*!
 * ov3640 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int ov3640_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int retval;
	struct mxc_camera_platform_data *plat_data = client->dev.platform_data;

	CAMERA_TRACE(("CAMERA_DBG Entry: ov3640_probe\n"));

	/* Set initial values for the sensor struct. */
	memset(&ov3640_data, 0, sizeof(ov3640_data));
	ov3640_data.mclk = 24000000; /* 6 - 54 MHz, typical 24MHz */
	ov3640_data.mclk = plat_data->mclk;
	ov3640_data.csi = plat_data->csi;

	ov3640_data.i2c_client = client;
	ov3640_data.pix.pixelformat = V4L2_PIX_FMT_UYVY;
	ov3640_data.pix.width = 640;
	ov3640_data.pix.height = 480;
	ov3640_data.streamcap.capability = V4L2_MODE_HIGHQUALITY |
					   V4L2_CAP_TIMEPERFRAME;
	ov3640_data.streamcap.capturemode = 0;
	ov3640_data.streamcap.timeperframe.denominator = DEFAULT_FPS;
	ov3640_data.streamcap.timeperframe.numerator = 1;

	io_regulator = regulator_get(&client->dev, plat_data->io_regulator);
	if (!IS_ERR_VALUE((u32)io_regulator)) {
		regulator_set_voltage(io_regulator, OV3640_VOLTAGE_DIGITAL_IO);
		if (regulator_enable(io_regulator) != 0) {
			pr_err("%s:io set voltage error\n", __func__);
			goto err1;
		} else {
			dev_dbg(&client->dev,
				"%s:io set voltage ok\n", __func__);
		}
	}

	core_regulator = regulator_get(&client->dev, plat_data->core_regulator);
	if (!IS_ERR_VALUE((u32)core_regulator)) {
		regulator_set_voltage(core_regulator,
				      OV3640_VOLTAGE_DIGITAL_CORE);
		if (regulator_enable(core_regulator) != 0) {
			pr_err("%s:core set voltage error\n", __func__);
			goto err2;
		} else {
			dev_dbg(&client->dev,
				"%s:core set voltage ok\n", __func__);
		}
	}

	analog_regulator =
		regulator_get(&client->dev, plat_data->analog_regulator);
	if (!IS_ERR_VALUE((u32)analog_regulator)) {
		regulator_set_voltage(analog_regulator, OV3640_VOLTAGE_ANALOG);
		if (regulator_enable(analog_regulator) != 0) {
			pr_err("%s:analog set voltage error\n", __func__);
			goto err3;
		} else {
			dev_dbg(&client->dev,
				"%s:analog set voltage ok\n", __func__);
		}
	}

	gpo_regulator = regulator_get(&client->dev, plat_data->gpo_regulator);
	if (!IS_ERR_VALUE((u32)gpo_regulator)) {
		if (regulator_enable(gpo_regulator) != 0) {
			pr_err("%s:gpo3 enable error\n", __func__);
			goto err4;
		} else {
			dev_dbg(&client->dev,
				"%s:gpo3 enable ok\n", __func__);
		}
	}

	ov3640_int_device.priv = &ov3640_data;
	retval = v4l2_int_device_register(&ov3640_int_device);

	return retval;

err4:
	if (!IS_ERR_VALUE((u32)analog_regulator)) {
		regulator_disable(analog_regulator);
		regulator_put(analog_regulator, &client->dev);
	}
err3:
	if (!IS_ERR_VALUE((u32)core_regulator)) {
		regulator_disable(core_regulator);
		regulator_put(core_regulator, &client->dev);
	}
err2:
	if (!IS_ERR_VALUE((u32)io_regulator)) {
		regulator_disable(io_regulator);
		regulator_put(io_regulator, &client->dev);
	}
err1:
	return -1;
}

/*!
 * ov3640 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int ov3640_remove(struct i2c_client *client)
{
	CAMERA_TRACE(("In ov3640_remove\n"));

	v4l2_int_device_unregister(&ov3640_int_device);

	if (!IS_ERR_VALUE((unsigned long)gpo_regulator)) {
		regulator_disable(gpo_regulator);
		regulator_put(gpo_regulator, &client->dev);
	}

	if (!IS_ERR_VALUE((unsigned long)analog_regulator)) {
		regulator_disable(analog_regulator);
		regulator_put(analog_regulator, &client->dev);
	}

	if (!IS_ERR_VALUE((unsigned long)core_regulator)) {
		regulator_disable(core_regulator);
		regulator_put(core_regulator, &client->dev);
	}

	if (!IS_ERR_VALUE((unsigned long)io_regulator)) {
		regulator_disable(io_regulator);
		regulator_put(io_regulator, &client->dev);
	}

	return 0;
}

/*!
 * ov3640 init function
 * Called by insmod ov3640_camera.ko.
 *
 * @return  Error code indicating success or failure
 */
static __init int ov3640_init(void)
{
	u8 err;

	CAMERA_TRACE(("CAMERA_DBG Entry: ov3640_init\n"));

	err = i2c_add_driver(&ov3640_i2c_driver);
	if (err != 0)
		pr_err("%s:driver registration failed, error=%d \n",
			__func__, err);

	CAMERA_TRACE(("CAMERA_DBG Exit: ov3640_init\n"));

	return err;
}

/*!
 * OV3640 cleanup function
 * Called on rmmod ov3640_camera.ko
 *
 * @return  Error code indicating success or failure
 */
static void __exit ov3640_clean(void)
{
	CAMERA_TRACE(("CAMERA_DBG Entry: ov3640_clean\n"));

	i2c_del_driver(&ov3640_i2c_driver);
	gpio_sensor_inactive(ov3640_data.csi);

	CAMERA_TRACE(("CAMERA_DBG Exit: ov3640_clean\n"));
}

module_init(ov3640_init);
module_exit(ov3640_clean);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("OV3640 Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
