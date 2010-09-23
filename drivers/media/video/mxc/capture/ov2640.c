/*
 * Copyright 2005-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file ov2640.c
 *
 * @brief ov2640 camera driver functions
 *
 * @ingroup Camera
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

#include <media/v4l2-int-device.h>
#include "mxc_v4l2_capture.h"

#define MIN_FPS 5
#define MAX_FPS 30
#define DEFAULT_FPS 30

#define OV2640_XCLK_MIN 6000000
#define OV2640_XCLK_MAX 27000000

/*
enum ov2640_mode {
	ov2640_mode_1600_1120,
	ov2640_mode_800_600
};
*/

struct reg_value {
	u8 reg;
	u8 value;
	int delay_ms;
};

static struct reg_value ov2640_setting_1600_1120[] = {
#ifdef CONFIG_MACH_MX25_3DS
	{0xff, 0x01, 0}, {0x12, 0x80, 5}, {0xff, 0x00, 0}, {0x2c, 0xff, 0},
	{0x2e, 0xdf, 0}, {0xff, 0x01, 0}, {0x3c, 0x32, 0}, {0x11, 0x00, 0},
	{0x09, 0x02, 0}, {0x04, 0x28, 0}, {0x13, 0xe5, 0}, {0x14, 0x48, 0},
	{0x2c, 0x0c, 0}, {0x33, 0x78, 0}, {0x3a, 0x33, 0}, {0x3b, 0xfb, 0},
	{0x3e, 0x00, 0}, {0x43, 0x11, 0}, {0x16, 0x10, 0}, {0x39, 0x02, 0},
	{0x35, 0x58, 0}, {0x22, 0x0a, 0}, {0x37, 0x40, 0}, {0x23, 0x00, 0},
	{0x34, 0xa0, 0}, {0x36, 0x1a, 0}, {0x06, 0x02, 0}, {0x07, 0xc0, 0},
	{0x0d, 0xb7, 0}, {0x0e, 0x01, 0}, {0x4c, 0x00, 0}, {0x4a, 0x81, 0},
	{0x21, 0x99, 0}, {0x24, 0x40, 0}, {0x25, 0x38, 0}, {0x26, 0x82, 0},
	{0x5c, 0x00, 0}, {0x63, 0x00, 0}, {0x46, 0x3f, 0}, {0x61, 0x70, 0},
	{0x62, 0x80, 0}, {0x7c, 0x05, 0}, {0x20, 0x80, 0}, {0x28, 0x30, 0},
	{0x6c, 0x00, 0}, {0x6d, 0x80, 0}, {0x6e, 0x00, 0}, {0x70, 0x02, 0},
	{0x71, 0x94, 0}, {0x73, 0xc1, 0}, {0x3d, 0x34, 0}, {0x5a, 0x57, 0},
	{0x4f, 0xbb, 0}, {0x50, 0x9c, 0}, {0xff, 0x00, 0}, {0xe5, 0x7f, 0},
	{0xf9, 0xc0, 0}, {0x41, 0x24, 0}, {0xe0, 0x14, 0}, {0x76, 0xff, 0},
	{0x33, 0xa0, 0}, {0x42, 0x20, 0}, {0x43, 0x18, 0}, {0x4c, 0x00, 0},
	{0x87, 0xd0, 0}, {0x88, 0x3f, 0}, {0xd7, 0x01, 0}, {0xd9, 0x10, 0},
	{0xd3, 0x82, 0}, {0xc8, 0x08, 0}, {0xc9, 0x80, 0}, {0x7c, 0x00, 0},
	{0x7d, 0x00, 0}, {0x7c, 0x03, 0}, {0x7d, 0x48, 0}, {0x7d, 0x48, 0},
	{0x7c, 0x08, 0}, {0x7d, 0x20, 0}, {0x7d, 0x10, 0}, {0x7d, 0x0e, 0},
	{0x90, 0x00, 0}, {0x91, 0x0e, 0}, {0x91, 0x1a, 0}, {0x91, 0x31, 0},
	{0x91, 0x5a, 0}, {0x91, 0x69, 0}, {0x91, 0x75, 0}, {0x91, 0x7e, 0},
	{0x91, 0x88, 0}, {0x91, 0x8f, 0}, {0x91, 0x96, 0}, {0x91, 0xa3, 0},
	{0x91, 0xaf, 0}, {0x91, 0xc4, 0}, {0x91, 0xd7, 0}, {0x91, 0xe8, 0},
	{0x91, 0x20, 0}, {0x92, 0x00, 0}, {0x93, 0x06, 0}, {0x93, 0xe3, 0},
	{0x93, 0x05, 0}, {0x93, 0x05, 0}, {0x93, 0x00, 0}, {0x93, 0x04, 0},
	{0x93, 0x00, 0}, {0x93, 0x00, 0}, {0x93, 0x00, 0}, {0x93, 0x00, 0},
	{0x93, 0x00, 0}, {0x93, 0x00, 0}, {0x93, 0x00, 0}, {0x96, 0x00, 0},
	{0x97, 0x08, 0}, {0x97, 0x19, 0}, {0x97, 0x02, 0}, {0x97, 0x0c, 0},
	{0x97, 0x24, 0}, {0x97, 0x30, 0}, {0x97, 0x28, 0}, {0x97, 0x26, 0},
	{0x97, 0x02, 0}, {0x97, 0x98, 0}, {0x97, 0x80, 0}, {0x97, 0x00, 0},
	{0x97, 0x00, 0}, {0xc3, 0xed, 0}, {0xa4, 0x00, 0}, {0xa8, 0x00, 0},
	{0xc5, 0x11, 0}, {0xc6, 0x51, 0}, {0xbf, 0x80, 0}, {0xc7, 0x10, 0},
	{0xb6, 0x66, 0}, {0xb8, 0xa5, 0}, {0xb7, 0x64, 0}, {0xb9, 0x7c, 0},
	{0xb3, 0xaf, 0}, {0xb4, 0x97, 0}, {0xb5, 0xff, 0}, {0xb0, 0xc5, 0},
	{0xb1, 0x94, 0}, {0xb2, 0x0f, 0}, {0xc4, 0x5c, 0}, {0xc0, 0xc8, 0},
	{0xc1, 0x96, 0}, {0x86, 0x1d, 0}, {0x50, 0x00, 0}, {0x51, 0x90, 0},
	{0x52, 0x2c, 0}, {0x53, 0x00, 0}, {0x54, 0x00, 0}, {0x55, 0x88, 0},
	{0x57, 0x00, 0}, {0x5a, 0x90, 0}, {0x5b, 0x2c, 0}, {0x5c, 0x05, 0},
	{0xc3, 0xed, 0}, {0x7f, 0x00, 0}, {0xda, 0x00, 0}, {0xe5, 0x1f, 0},
	{0xe1, 0x77, 0}, {0xe0, 0x00, 0}, {0xdd, 0x7f, 0}, {0x05, 0x00, 0},
	{0xff, 0x00, 0}, {0xe0, 0x04, 0}, {0xc0, 0xc8, 0}, {0xc1, 0x96, 0},
	{0x86, 0x3d, 0}, {0x50, 0x00, 0}, {0x51, 0x90, 0}, {0x52, 0x2c, 0},
	{0x53, 0x00, 0}, {0x54, 0x00, 0}, {0x55, 0x88, 0}, {0x57, 0x00, 0},
	{0x5a, 0x40, 0}, {0x5b, 0xf0, 0}, {0x5c, 0x01, 0}, {0xd3, 0x82, 0},
	{0xe0, 0x00, 1000}
#else
	{0xff, 0x1, 0}, {0x12, 0x80, 1}, {0xff, 0, 0}, {0x2c, 0xff, 0},
	{0x2e, 0xdf, 0}, {0xff, 0x1, 0}, {0x3c, 0x32, 0}, {0x11, 0x01, 0},
	{0x09, 0x00, 0}, {0x04, 0x28, 0}, {0x13, 0xe5, 0}, {0x14, 0x48, 0},
	{0x2c, 0x0c, 0}, {0x33, 0x78, 0}, {0x3a, 0x33, 0}, {0x3b, 0xfb, 0},
	{0x3e, 0x00, 0}, {0x43, 0x11, 0}, {0x16, 0x10, 0}, {0x39, 0x82, 0},
	{0x35, 0x88, 0}, {0x22, 0x0a, 0}, {0x37, 0x40, 0}, {0x23, 0x00, 0},
	{0x34, 0xa0, 0}, {0x36, 0x1a, 0}, {0x06, 0x02, 0}, {0x07, 0xc0, 0},
	{0x0d, 0xb7, 0}, {0x0e, 0x01, 0}, {0x4c, 0x00, 0}, {0x4a, 0x81, 0},
	{0x21, 0x99, 0}, {0x24, 0x40, 0}, {0x25, 0x38, 0}, {0x26, 0x82, 0},
	{0x5c, 0x00, 0}, {0x63, 0x00, 0}, {0x46, 0x3f, 0}, {0x0c, 0x3c, 0},
	{0x5d, 0x55, 0}, {0x5e, 0x7d, 0}, {0x5f, 0x7d, 0}, {0x60, 0x55, 0},
	{0x61, 0x70, 0}, {0x62, 0x80, 0}, {0x7c, 0x05, 0}, {0x20, 0x80, 0},
	{0x28, 0x30, 0}, {0x6c, 0x00, 0}, {0x6d, 0x80, 0}, {0x6e, 00, 0},
	{0x70, 0x02, 0}, {0x71, 0x94, 0}, {0x73, 0xc1, 0}, {0x3d, 0x34, 0},
	{0x5a, 0x57, 0}, {0x4f, 0xbb, 0}, {0x50, 0x9c, 0}, {0xff, 0x00, 0},
	{0xe5, 0x7f, 0}, {0xf9, 0xc0, 0}, {0x41, 0x24, 0}, {0x44, 0x06, 0},
	{0xe0, 0x14, 0}, {0x76, 0xff, 0}, {0x33, 0xa0, 0}, {0x42, 0x20, 0},
	{0x43, 0x18, 0}, {0x4c, 0x00, 0}, {0x87, 0xd0, 0}, {0xd7, 0x03, 0},
	{0xd9, 0x10, 0}, {0xd3, 0x82, 0}, {0xc8, 0x08, 0}, {0xc9, 0x80, 0},
	{0x7c, 0x00, 0}, {0x7d, 0x00, 0}, {0x7c, 0x03, 0}, {0x7d, 0x48, 0},
	{0x7d, 0x48, 0}, {0x7c, 0x08, 0}, {0x7d, 0x20, 0}, {0x7d, 0x10, 0},
	{0x7d, 0x0e, 0}, {0x90, 0x00, 0}, {0x91, 0x0e, 0}, {0x91, 0x1a, 0},
	{0x91, 0x31, 0}, {0x91, 0x5a, 0}, {0x91, 0x69, 0}, {0x91, 0x75, 0},
	{0x91, 0x7e, 0}, {0x91, 0x88, 0}, {0x91, 0x8f, 0}, {0x91, 0x96, 0},
	{0x91, 0xa3, 0}, {0x91, 0xaf, 0}, {0x91, 0xc4, 0}, {0x91, 0xd7, 0},
	{0x91, 0xe8, 0}, {0x91, 0x20, 0}, {0x92, 0x00, 0}, {0x93, 0x06, 0},
	{0x93, 0xe3, 0}, {0x93, 0x03, 0}, {0x93, 0x03, 0}, {0x93, 0x00, 0},
	{0x93, 0x02, 0}, {0x93, 0x00, 0}, {0x93, 0x00, 0}, {0x93, 0x00, 0},
	{0x93, 0x00, 0}, {0x93, 0x00, 0}, {0x93, 0x00, 0}, {0x93, 0x00, 0},
	{0x96, 0x00, 0}, {0x97, 0x08, 0}, {0x97, 0x19, 0}, {0x97, 0x02, 0},
	{0x97, 0x0c, 0}, {0x97, 0x24, 0}, {0x97, 0x30, 0}, {0x97, 0x28, 0},
	{0x97, 0x26, 0}, {0x97, 0x02, 0}, {0x97, 0x98, 0}, {0x97, 0x80, 0},
	{0x97, 0x00, 0}, {0x97, 0x00, 0}, {0xa4, 0x00, 0}, {0xa8, 0x00, 0},
	{0xc5, 0x11, 0}, {0xc6, 0x51, 0}, {0xbf, 0x80, 0}, {0xc7, 0x10, 0},
	{0xb6, 0x66, 0}, {0xb8, 0xa5, 0}, {0xb7, 0x64, 0}, {0xb9, 0x7c, 0},
	{0xb3, 0xaf, 0}, {0xb4, 0x97, 0}, {0xb5, 0xff, 0}, {0xb0, 0xc5, 0},
	{0xb1, 0x94, 0}, {0xb2, 0x0f, 0}, {0xc4, 0x5c, 0}, {0xa6, 0x00, 0},
	{0xa7, 0x20, 0}, {0xa7, 0xd8, 0}, {0xa7, 0x1b, 0}, {0xa7, 0x31, 0},
	{0xa7, 0x00, 0}, {0xa7, 0x18, 0}, {0xa7, 0x20, 0}, {0xa7, 0xd8, 0},
	{0xa7, 0x19, 0}, {0xa7, 0x31, 0}, {0xa7, 0x00, 0}, {0xa7, 0x18, 0},
	{0xa7, 0x20, 0}, {0xa7, 0xd8, 0}, {0xa7, 0x19, 0}, {0xa7, 0x31, 0},
	{0xa7, 0x00, 0}, {0xa7, 0x18, 0}, {0xc0, 0xc8, 0}, {0xc1, 0x96, 0},
	{0x86, 0x3d, 0}, {0x50, 0x00, 0}, {0x51, 0x90, 0}, {0x52, 0x18, 0},
	{0x53, 0x00, 0}, {0x54, 0x00, 0}, {0x55, 0x88, 0}, {0x57, 0x00, 0},
	{0x5a, 0x90, 0}, {0x5b, 0x18, 0}, {0x5c, 0x05, 0}, {0xc3, 0xef, 0},
	{0x7f, 0x00, 0}, {0xda, 0x01, 0}, {0xe5, 0x1f, 0}, {0xe1, 0x67, 0},
	{0xe0, 0x00, 0}, {0xdd, 0x7f, 0}, {0x05, 0x00, 0}
#endif
};

static struct reg_value ov2640_setting_800_600[] = {
#ifdef CONFIG_MACH_MX25_3DS
	{0xff, 0x01, 0}, {0x12, 0x80, 5}, {0xff, 0x00, 0}, {0x2c, 0xff, 0},
	{0x2e, 0xdf, 0}, {0xff, 0x01, 0}, {0x3c, 0x32, 0}, {0x11, 0x00, 0},
	{0x09, 0x02, 0}, {0x04, 0x28, 0}, {0x13, 0xe5, 0}, {0x14, 0x48, 0},
	{0x2c, 0x0c, 0}, {0x33, 0x78, 0}, {0x3a, 0x33, 0}, {0x3b, 0xfb, 0},
	{0x3e, 0x00, 0}, {0x43, 0x11, 0}, {0x16, 0x10, 0}, {0x39, 0x92, 0},
	{0x35, 0xda, 0}, {0x22, 0x1a, 0}, {0x37, 0xc3, 0}, {0x23, 0x00, 0},
	{0x34, 0xc0, 0}, {0x36, 0x1a, 0}, {0x06, 0x88, 0}, {0x07, 0xc0, 0},
	{0x0d, 0x87, 0}, {0x0e, 0x41, 0}, {0x4c, 0x00, 0},
	{0x48, 0x00, 0}, {0x5b, 0x00, 0}, {0x42, 0x03, 0}, {0x4a, 0x81, 0},
	{0x21, 0x99, 0}, {0x24, 0x40, 0}, {0x25, 0x38, 0}, {0x26, 0x82, 0},
	{0x5c, 0x00, 0}, {0x63, 0x00, 0}, {0x46, 0x22, 0}, {0x0c, 0x3c, 0},
	{0x61, 0x70, 0}, {0x62, 0x80, 0}, {0x7c, 0x05, 0}, {0x20, 0x80, 0},
	{0x28, 0x30, 0}, {0x6c, 0x00, 0}, {0x6d, 0x80, 0}, {0x6e, 0x00, 0},
	{0x70, 0x02, 0}, {0x71, 0x94, 0}, {0x73, 0xc1, 0}, {0x12, 0x40, 0},
	{0x17, 0x11, 0}, {0x18, 0x43, 0}, {0x19, 0x00, 0}, {0x1a, 0x4b, 0},
	{0x32, 0x09, 0}, {0x37, 0xc0, 0}, {0x4f, 0xca, 0}, {0x50, 0xa8, 0},
	{0x5a, 0x23, 0}, {0x6d, 0x00, 0}, {0x3d, 0x38, 0}, {0xff, 0x00, 0},
	{0xe5, 0x7f, 0}, {0xf9, 0xc0, 0}, {0x41, 0x24, 0}, {0xe0, 0x14, 0},
	{0x76, 0xff, 0}, {0x33, 0xa0, 0}, {0x42, 0x20, 0}, {0x43, 0x18, 0},
	{0x4c, 0x00, 0}, {0x87, 0xd5, 0}, {0x88, 0x3f, 0}, {0xd7, 0x01, 0},
	{0xd9, 0x10, 0}, {0xd3, 0x82, 0}, {0xc8, 0x08, 0}, {0xc9, 0x80, 0},
	{0x7c, 0x00, 0}, {0x7d, 0x00, 0}, {0x7c, 0x03, 0}, {0x7d, 0x48, 0},
	{0x7d, 0x48, 0}, {0x7c, 0x08, 0}, {0x7d, 0x20, 0}, {0x7d, 0x10, 0},
	{0x7d, 0x0e, 0}, {0x90, 0x00, 0}, {0x91, 0x0e, 0}, {0x91, 0x1a, 0},
	{0x91, 0x31, 0}, {0x91, 0x5a, 0}, {0x91, 0x69, 0}, {0x91, 0x75, 0},
	{0x91, 0x7e, 0}, {0x91, 0x88, 0}, {0x91, 0x8f, 0}, {0x91, 0x96, 0},
	{0x91, 0xa3, 0}, {0x91, 0xaf, 0}, {0x91, 0xc4, 0}, {0x91, 0xd7, 0},
	{0x91, 0xe8, 0}, {0x91, 0x20, 0}, {0x92, 0x00, 0}, {0x93, 0x06, 0},
	{0x93, 0xe3, 0}, {0x93, 0x05, 0}, {0x93, 0x05, 0}, {0x93, 0x00, 0},
	{0x93, 0x04, 0}, {0x93, 0x00, 0}, {0x93, 0x00, 0}, {0x93, 0x00, 0},
	{0x93, 0x00, 0}, {0x93, 0x00, 0}, {0x93, 0x00, 0}, {0x93, 0x00, 0},
	{0x96, 0x00, 0}, {0x97, 0x08, 0}, {0x97, 0x19, 0}, {0x97, 0x02, 0},
	{0x97, 0x0c, 0}, {0x97, 0x24, 0}, {0x97, 0x30, 0}, {0x97, 0x28, 0},
	{0x97, 0x26, 0}, {0x97, 0x02, 0}, {0x97, 0x98, 0}, {0x97, 0x80, 0},
	{0x97, 0x00, 0}, {0x97, 0x00, 0}, {0xc3, 0xed, 0}, {0xa4, 0x00, 0},
	{0xa8, 0x00, 0}, {0xc5, 0x11, 0}, {0xc6, 0x51, 0}, {0xbf, 0x80, 0},
	{0xc7, 0x10, 0}, {0xb6, 0x66, 0}, {0xb8, 0xa5, 0}, {0xb7, 0x64, 0},
	{0xb9, 0x7c, 0}, {0xb3, 0xaf, 0}, {0xb4, 0x97, 0}, {0xb5, 0xff, 0},
	{0xb0, 0xc5, 0}, {0xb1, 0x94, 0}, {0xb2, 0x0f, 0}, {0xc4, 0x5c, 0},
	{0xc0, 0x64, 0}, {0xc1, 0x4b, 0}, {0x8c, 0x00, 0}, {0x86, 0x3d, 0},
	{0x50, 0x00, 0}, {0x51, 0xc8, 0}, {0x52, 0x96, 0}, {0x53, 0x00, 0},
	{0x54, 0x00, 0}, {0x55, 0x00, 0}, {0x5a, 0xc8, 0}, {0x5b, 0x96, 0},
	{0x5c, 0x00, 0}, {0xd3, 0x82, 0}, {0xc3, 0xed, 0}, {0x7f, 0x00, 0},
	{0xda, 0x00, 0}, {0xe5, 0x1f, 0}, {0xe1, 0x67, 0}, {0xe0, 0x00, 0},
	{0xdd, 0x7f, 0}, {0x05, 0x00, 0}, {0xff, 0x00, 0}, {0xe0, 0x04, 0},
	{0xc0, 0x64, 0}, {0xc1, 0x4b, 0}, {0x8c, 0x00, 0}, {0x86, 0x3d, 0},
	{0x50, 0x00, 0}, {0x51, 0xc8, 0}, {0x52, 0x96, 0}, {0x53, 0x00, 0},
	{0x54, 0x00, 0}, {0x55, 0x00, 0}, {0x5a, 0xa0, 0}, {0x5b, 0x78, 0},
	{0x5c, 0x00, 0}, {0xd3, 0x82, 0}, {0xe0, 0x00, 1000}
#else
	{0xff, 0, 0}, {0xff, 1, 0}, {0x12, 0x80, 1}, {0xff, 00, 0},
	{0x2c, 0xff, 0}, {0x2e, 0xdf, 0}, {0xff, 0x1, 0}, {0x3c, 0x32, 0},
	{0x11, 0x01, 0}, {0x09, 0x00, 0}, {0x04, 0x28, 0}, {0x13, 0xe5, 0},
	{0x14, 0x48, 0}, {0x2c, 0x0c, 0}, {0x33, 0x78, 0}, {0x3a, 0x33, 0},
	{0x3b, 0xfb, 0}, {0x3e, 0x00, 0}, {0x43, 0x11, 0}, {0x16, 0x10, 0},
	{0x39, 0x92, 0}, {0x35, 0xda, 0}, {0x22, 0x1a, 0}, {0x37, 0xc3, 0},
	{0x23, 0x00, 0}, {0x34, 0xc0, 0}, {0x36, 0x1a, 0}, {0x06, 0x88, 0},
	{0x07, 0xc0, 0}, {0x0d, 0x87, 0}, {0x0e, 0x41, 0}, {0x4c, 0x00, 0},
	{0x4a, 0x81, 0}, {0x21, 0x99, 0}, {0x24, 0x40, 0}, {0x25, 0x38, 0},
	{0x26, 0x82, 0}, {0x5c, 0x00, 0}, {0x63, 0x00, 0}, {0x46, 0x22, 0},
	{0x0c, 0x3c, 0}, {0x5d, 0x55, 0}, {0x5e, 0x7d, 0}, {0x5f, 0x7d, 0},
	{0x60, 0x55, 0}, {0x61, 0x70, 0}, {0x62, 0x80, 0}, {0x7c, 0x05, 0},
	{0x20, 0x80, 0}, {0x28, 0x30, 0}, {0x6c, 0x00, 0}, {0x6d, 0x80, 0},
	{0x6e, 00, 0}, {0x70, 0x02, 0}, {0x71, 0x94, 0}, {0x73, 0xc1, 0},
	{0x12, 0x40, 0}, {0x17, 0x11, 0}, {0x18, 0x43, 0}, {0x19, 0x00, 0},
	{0x1a, 0x4b, 0}, {0x32, 0x09, 0}, {0x37, 0xc0, 0}, {0x4f, 0xca, 0},
	{0x50, 0xa8, 0}, {0x6d, 0x00, 0}, {0x3d, 0x38, 0}, {0xff, 0x00, 0},
	{0xe5, 0x7f, 0}, {0xf9, 0xc0, 0}, {0x41, 0x24, 0}, {0x44, 0x06, 0},
	{0xe0, 0x14, 0}, {0x76, 0xff, 0}, {0x33, 0xa0, 0}, {0x42, 0x20, 0},
	{0x43, 0x18, 0}, {0x4c, 0x00, 0}, {0x87, 0xd0, 0}, {0x88, 0x3f, 0},
	{0xd7, 0x03, 0}, {0xd9, 0x10, 0}, {0xd3, 0x82, 0}, {0xc8, 0x08, 0},
	{0xc9, 0x80, 0}, {0x7c, 0x00, 0}, {0x7d, 0x00, 0}, {0x7c, 0x03, 0},
	{0x7d, 0x48, 0}, {0x7d, 0x48, 0}, {0x7c, 0x08, 0}, {0x7d, 0x20, 0},
	{0x7d, 0x10, 0}, {0x7d, 0x0e, 0}, {0x90, 0x00, 0}, {0x91, 0x0e, 0},
	{0x91, 0x1a, 0}, {0x91, 0x31, 0}, {0x91, 0x5a, 0}, {0x91, 0x69, 0},
	{0x91, 0x75, 0}, {0x91, 0x7e, 0}, {0x91, 0x88, 0}, {0x91, 0x8f, 0},
	{0x91, 0x96, 0}, {0x91, 0xa3, 0}, {0x91, 0xaf, 0}, {0x91, 0xc4, 0},
	{0x91, 0xd7, 0}, {0x91, 0xe8, 0}, {0x91, 0x20, 0}, {0x92, 0x00, 0},
	{0x93, 0x06, 0}, {0x93, 0xe3, 0}, {0x93, 0x03, 0}, {0x93, 0x03, 0},
	{0x93, 0x00, 0}, {0x93, 0x02, 0}, {0x93, 0x00, 0}, {0x93, 0x00, 0},
	{0x93, 0x00, 0}, {0x93, 0x00, 0}, {0x93, 0x00, 0}, {0x93, 0x00, 0},
	{0x93, 0x00, 0}, {0x96, 0x00, 0}, {0x97, 0x08, 0}, {0x97, 0x19, 0},
	{0x97, 0x02, 0}, {0x97, 0x0c, 0}, {0x97, 0x24, 0}, {0x97, 0x30, 0},
	{0x97, 0x28, 0}, {0x97, 0x26, 0}, {0x97, 0x02, 0}, {0x97, 0x98, 0},
	{0x97, 0x80, 0}, {0x97, 0x00, 0}, {0x97, 0x00, 0}, {0xa4, 0x00, 0},
	{0xa8, 0x00, 0}, {0xc5, 0x11, 0}, {0xc6, 0x51, 0}, {0xbf, 0x80, 0},
	{0xc7, 0x10, 0}, {0xb6, 0x66, 0}, {0xb8, 0xa5, 0}, {0xb7, 0x64, 0},
	{0xb9, 0x7c, 0}, {0xb3, 0xaf, 0}, {0xb4, 0x97, 0}, {0xb5, 0xff, 0},
	{0xb0, 0xc5, 0}, {0xb1, 0x94, 0}, {0xb2, 0x0f, 0}, {0xc4, 0x5c, 0},
	{0xa6, 0x00, 0}, {0xa7, 0x20, 0}, {0xa7, 0xd8, 0}, {0xa7, 0x1b, 0},
	{0xa7, 0x31, 0}, {0xa7, 0x00, 0}, {0xa7, 0x18, 0}, {0xa7, 0x20, 0},
	{0xa7, 0xd8, 0}, {0xa7, 0x19, 0}, {0xa7, 0x31, 0}, {0xa7, 0x00, 0},
	{0xa7, 0x18, 0}, {0xa7, 0x20, 0}, {0xa7, 0xd8, 0}, {0xa7, 0x19, 0},
	{0xa7, 0x31, 0}, {0xa7, 0x00, 0}, {0xa7, 0x18, 0}, {0xc0, 0x64, 0},
	{0xc1, 0x4b, 0}, {0x86, 0x1d, 0}, {0x50, 0x00, 0}, {0x51, 0xc8, 0},
	{0x52, 0x96, 0}, {0x53, 0x00, 0}, {0x54, 0x00, 0}, {0x55, 0x00, 0},
	{0x57, 0x00, 0}, {0x5a, 0xc8, 0}, {0x5b, 0x96, 0}, {0x5c, 0x00, 0},
	{0xc3, 0xef, 0}, {0x7f, 0x00, 0}, {0xda, 0x01, 0}, {0xe5, 0x1f, 0},
	{0xe1, 0x67, 0}, {0xe0, 0x00, 0}, {0xdd, 0x7f, 0}, {0x05, 0x00, 0}
#endif
};

/*!
 * Maintains the information on the current state of the sesor.
 */
struct sensor {
	const struct ov2640_platform_data *platform_data;
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

	u32 csi;
	u32 mclk;

} ov2640_data;

static struct regulator *io_regulator;
static struct regulator *core_regulator;
static struct regulator *analog_regulator;
static struct regulator *gpo_regulator;

extern void gpio_sensor_active(void);
extern void gpio_sensor_inactive(void);

/* list of image formats supported by this sensor */
/*
const static struct v4l2_fmtdesc ov2640_formats[] = {
	{
		.description = "YUYV (YUV 4:2:2), packed",
		.pixelformat = V4L2_PIX_FMT_UYVY,
	},
};
 */

static int ov2640_init_mode(struct sensor *s)
{
	int ret = -1;
	struct reg_value *setting;
	int i, num;

	pr_debug("In ov2640:ov2640_init_mode capturemode is %d\n",
		s->streamcap.capturemode);

	if (s->streamcap.capturemode & V4L2_MODE_HIGHQUALITY) {
		s->pix.width = 1600;
		s->pix.height = 1120;
		setting = ov2640_setting_1600_1120;
		num = ARRAY_SIZE(ov2640_setting_1600_1120);
	} else {
		s->pix.width = 800;
		s->pix.height = 600;
		setting = ov2640_setting_800_600;
		num = ARRAY_SIZE(ov2640_setting_800_600);
	}

	for (i = 0; i < num; i++) {
		ret = i2c_smbus_write_byte_data(s->i2c_client,
						setting[i].reg,
						setting[i].value);
		if (ret < 0) {
			pr_err("write reg error: reg=%x, val=%x\n",
			       setting[i].reg, setting[i].value);
			return ret;
		}
		if (setting[i].delay_ms > 0)
			msleep(setting[i].delay_ms);
	}

	return ret;
}

/* At present only support change to 15fps(only for SVGA mode) */
static int ov2640_set_fps(struct sensor *s, int fps)
{
	int ret = 0;

	if (i2c_smbus_write_byte_data(s->i2c_client, 0xff, 0x01) < 0) {
		pr_err("in %s,change to sensor addr failed\n", __func__);
		ret = -EPERM;
	}

	/* change the camera framerate to 15fps(only for SVGA mode) */
	if (i2c_smbus_write_byte_data(s->i2c_client, 0x11, 0x01) < 0) {
		pr_err("change camera to 15fps failed\n");
		ret = -EPERM;
	}

	return ret;
}

static int ov2640_set_format(struct sensor *s, int format)
{
	int ret = 0;

	if (i2c_smbus_write_byte_data(s->i2c_client, 0xff, 0x00) < 0)
		ret = -EPERM;

	if (format == V4L2_PIX_FMT_RGB565) {
		/* set RGB565 format */
		if (i2c_smbus_write_byte_data(s->i2c_client, 0xda, 0x08) < 0)
			ret = -EPERM;

		if (i2c_smbus_write_byte_data(s->i2c_client, 0xd7, 0x03) < 0)
			ret = -EPERM;
	} else if (format == V4L2_PIX_FMT_YUV420) {
		/* set YUV420 format */
		if (i2c_smbus_write_byte_data(s->i2c_client, 0xda, 0x00) < 0)
			ret = -EPERM;

		if (i2c_smbus_write_byte_data(s->i2c_client, 0xd7, 0x1b) < 0)
			ret = -EPERM;
	} else {
		pr_debug("format not supported\n");
	}

	return ret;
}

/* --------------- IOCTL functions from v4l2_int_ioctl_desc --------------- */

/*!
 * ioctl_g_ifparm - V4L2 sensor interface handler for vidioc_int_g_ifparm_num
 * s: pointer to standard V4L2 device structure
 * p: pointer to standard V4L2 vidioc_int_g_ifparm_num ioctl structure
 *
 * Gets slave interface parameters.
 * Calculates the required xclk value to support the requested
 * clock parameters in p.  This value is returned in the p
 * parameter.
 *
 * vidioc_int_g_ifparm returns platform-specific information about the
 * interface settings used by the sensor.
 *
 * Given the image capture format in pix, the nominal frame period in
 * timeperframe, calculate the required xclk frequency.
 *
 * Called on open.
 */
static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	pr_debug("In ov2640:ioctl_g_ifparm\n");

	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	memset(p, 0, sizeof(*p));
	p->u.bt656.clock_curr = ov2640_data.mclk;
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.clock_min = OV2640_XCLK_MIN;
	p->u.bt656.clock_max = OV2640_XCLK_MAX;

	return 0;
}

/*!
 * Sets the camera power.
 *
 * s  pointer to the camera device
 * on if 1, power is to be turned on.  0 means power is to be turned off
 *
 * ioctl_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 * This is called on open, close, suspend and resume.
 */
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct sensor *sensor = s->priv;

	pr_debug("In ov2640:ioctl_s_power\n");

	if (on && !sensor->on) {
		gpio_sensor_active();
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
	} else if (!on && sensor->on) {
		if (analog_regulator)
			regulator_disable(analog_regulator);
		if (core_regulator)
			regulator_disable(core_regulator);
		if (io_regulator)
			regulator_disable(io_regulator);
		if (gpo_regulator)
			regulator_disable(gpo_regulator);
		gpio_sensor_inactive();
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

	pr_debug("In ov2640:ioctl_g_parm\n");

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n");
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
		pr_err("   type is not V4L2_BUF_TYPE_VIDEO_CAPTURE " \
			"but %d\n", a->type);
		ret = -EINVAL;
		break;

	default:
		pr_err("   type is unknown - %d\n", a->type);
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
	int ret = 0;

	pr_debug("In ov2640:ioctl_s_parm\n");

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n");

		/* Check that the new frame rate is allowed. */
		if ((timeperframe->numerator == 0)
		    || (timeperframe->denominator == 0)) {
			timeperframe->denominator = DEFAULT_FPS;
			timeperframe->numerator = 1;
		}
		tgt_fps = timeperframe->denominator
			  / timeperframe->numerator;

		if (tgt_fps > MAX_FPS) {
			timeperframe->denominator = MAX_FPS;
			timeperframe->numerator = 1;
		} else if (tgt_fps < MIN_FPS) {
			timeperframe->denominator = MIN_FPS;
			timeperframe->numerator = 1;
		}
		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode =
				(u32)a->parm.capture.capturemode;

		ret = ov2640_init_mode(sensor);
		if (tgt_fps == 15)
			ov2640_set_fps(sensor, tgt_fps);
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_err("   type is not V4L2_BUF_TYPE_VIDEO_CAPTURE " \
			"but %d\n", a->type);
		ret = -EINVAL;
		break;

	default:
		pr_err("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_s_fmt_cap - V4L2 sensor interface handler for ioctl_s_fmt_cap
 * 		     set camera output format and resolution format
 *
 * @s: pointer to standard V4L2 device structure
 * @arg: pointer to parameter, according this to set camera
 *
 * Returns 0 if set succeed, else return -1
 */
static int ioctl_s_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor *sensor = s->priv;
	u32 format = f->fmt.pix.pixelformat;
	int size = 0, ret = 0;

	size = f->fmt.pix.width * f->fmt.pix.height;
	switch (format) {
	case V4L2_PIX_FMT_RGB565:
		if (size > 640 * 480)
			sensor->streamcap.capturemode = V4L2_MODE_HIGHQUALITY;
		else
			sensor->streamcap.capturemode = 0;
		ret = ov2640_init_mode(sensor);

		ret = ov2640_set_format(sensor, V4L2_PIX_FMT_RGB565);
		break;
	case V4L2_PIX_FMT_UYVY:
		if (size > 640 * 480)
			sensor->streamcap.capturemode = V4L2_MODE_HIGHQUALITY;
		else
			sensor->streamcap.capturemode = 0;
		ret = ov2640_init_mode(sensor);
		break;
	case V4L2_PIX_FMT_YUV420:
		if (size > 640 * 480)
			sensor->streamcap.capturemode = V4L2_MODE_HIGHQUALITY;
		else
			sensor->streamcap.capturemode = 0;
		ret = ov2640_init_mode(sensor);

		/* YUYV: width * 2, YY: width */
		ret = ov2640_set_format(sensor, V4L2_PIX_FMT_YUV420);
		break;
	default:
		pr_debug("case not supported\n");
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

	pr_debug("In ov2640:ioctl_g_fmt_cap.\n");

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

	pr_debug("In ov2640:ioctl_g_ctrl\n");

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		vc->value = ov2640_data.brightness;
		break;
	case V4L2_CID_HUE:
		vc->value = ov2640_data.hue;
		break;
	case V4L2_CID_CONTRAST:
		vc->value = ov2640_data.contrast;
		break;
	case V4L2_CID_SATURATION:
		vc->value = ov2640_data.saturation;
		break;
	case V4L2_CID_RED_BALANCE:
		vc->value = ov2640_data.red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		vc->value = ov2640_data.blue;
		break;
	case V4L2_CID_EXPOSURE:
		vc->value = ov2640_data.ae_mode;
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

	pr_debug("In ov2640:ioctl_s_ctrl %d\n", vc->id);

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		pr_debug("   V4L2_CID_BRIGHTNESS\n");
		break;
	case V4L2_CID_CONTRAST:
		pr_debug("   V4L2_CID_CONTRAST\n");
		break;
	case V4L2_CID_SATURATION:
		pr_debug("   V4L2_CID_SATURATION\n");
		break;
	case V4L2_CID_HUE:
		pr_debug("   V4L2_CID_HUE\n");
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		pr_debug(
			"   V4L2_CID_AUTO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		pr_debug(
			"   V4L2_CID_DO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_RED_BALANCE:
		pr_debug("   V4L2_CID_RED_BALANCE\n");
		break;
	case V4L2_CID_BLUE_BALANCE:
		pr_debug("   V4L2_CID_BLUE_BALANCE\n");
		break;
	case V4L2_CID_GAMMA:
		pr_debug("   V4L2_CID_GAMMA\n");
		break;
	case V4L2_CID_EXPOSURE:
		pr_debug("   V4L2_CID_EXPOSURE\n");
		break;
	case V4L2_CID_AUTOGAIN:
		pr_debug("   V4L2_CID_AUTOGAIN\n");
		break;
	case V4L2_CID_GAIN:
		pr_debug("   V4L2_CID_GAIN\n");
		break;
	case V4L2_CID_HFLIP:
		pr_debug("   V4L2_CID_HFLIP\n");
		break;
	case V4L2_CID_VFLIP:
		pr_debug("   V4L2_CID_VFLIP\n");
		break;
	default:
		pr_debug("   Default case\n");
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
	pr_debug("In ov2640:ioctl_init\n");

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

	pr_debug("In ov2640:ioctl_dev_init\n");

	gpio_sensor_active();
	ov2640_data.on = true;

	tgt_xclk = ov2640_data.mclk;
	tgt_xclk = min(tgt_xclk, (u32)OV2640_XCLK_MAX);
	tgt_xclk = max(tgt_xclk, (u32)OV2640_XCLK_MIN);
	ov2640_data.mclk = tgt_xclk;

	pr_debug("   Setting mclk to %d MHz\n",
		tgt_xclk / 1000000);
	set_mclk_rate(&ov2640_data.mclk);

	return ov2640_init_mode(sensor);
}

/*!
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the device when slave detaches to the master.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	pr_debug("In ov2640:ioctl_dev_exit\n");

	gpio_sensor_inactive();

	return 0;
}

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc ov2640_ioctl_desc[] = {
	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func*)ioctl_dev_init},
	{vidioc_int_dev_exit_num, (v4l2_int_ioctl_func*)ioctl_dev_exit},
	{vidioc_int_s_power_num, (v4l2_int_ioctl_func*)ioctl_s_power},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func*)ioctl_g_ifparm},
/*	{vidioc_int_g_needs_reset_num,
				(v4l2_int_ioctl_func *)ioctl_g_needs_reset}, */
/*	{vidioc_int_reset_num, (v4l2_int_ioctl_func *)ioctl_reset}, */
	{vidioc_int_init_num, (v4l2_int_ioctl_func*)ioctl_init},
/*	{vidioc_int_enum_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_enum_fmt_cap}, */
/*	{vidioc_int_try_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_try_fmt_cap}, */
	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func*)ioctl_g_fmt_cap},
	{vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func*)ioctl_s_fmt_cap},
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func*)ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func*)ioctl_s_parm},
/*	{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func *)ioctl_queryctrl}, */
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func*)ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func*)ioctl_s_ctrl},
};

static struct v4l2_int_slave ov2640_slave = {
	.ioctls = ov2640_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(ov2640_ioctl_desc),
};

static struct v4l2_int_device ov2640_int_device = {
	.module = THIS_MODULE,
	.name = "ov2640",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &ov2640_slave,
		},
};

/*!
 * ov2640 I2C attach function
 * Function set in i2c_driver struct.
 * Called by insmod ov2640_camera.ko.
 *
 * @param client            struct i2c_client*
 * @return  Error code indicating success or failure
 */
static int ov2640_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int retval;
	struct mxc_camera_platform_data *plat_data = client->dev.platform_data;

	pr_debug("In ov2640_probe (RH_BT565)\n");

	/* Set initial values for the sensor struct. */
	memset(&ov2640_data, 0, sizeof(ov2640_data));
	ov2640_data.i2c_client = client;
	ov2640_data.mclk = 24000000;
	ov2640_data.mclk = plat_data->mclk;
	ov2640_data.pix.pixelformat = V4L2_PIX_FMT_UYVY;
	ov2640_data.pix.width = 800;
	ov2640_data.pix.height = 600;
	ov2640_data.streamcap.capability = V4L2_MODE_HIGHQUALITY
					   | V4L2_CAP_TIMEPERFRAME;
	ov2640_data.streamcap.capturemode = 0;
	ov2640_data.streamcap.timeperframe.denominator = DEFAULT_FPS;
	ov2640_data.streamcap.timeperframe.numerator = 1;

	if (plat_data->io_regulator) {
		io_regulator =
		    regulator_get(&client->dev, plat_data->io_regulator);
		if (!IS_ERR(io_regulator)) {
			regulator_set_voltage(io_regulator, 2800000, 2800000);
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
		core_regulator =
		    regulator_get(&client->dev, plat_data->core_regulator);
		if (!IS_ERR(core_regulator)) {
			regulator_set_voltage(core_regulator,
					 1300000, 1300000);
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
		analog_regulator =
		    regulator_get(&client->dev, plat_data->analog_regulator);
		if (!IS_ERR(analog_regulator)) {
			regulator_set_voltage(analog_regulator, 2000000, 2000000);
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

	if (plat_data->gpo_regulator) {
		gpo_regulator =
		    regulator_get(&client->dev, plat_data->gpo_regulator);
		if (!IS_ERR(gpo_regulator)) {
			if (regulator_enable(gpo_regulator) != 0) {
				pr_err("%s:gpo3 set voltage error\n", __func__);
				goto err4;
			} else {
				dev_dbg(&client->dev,
					"%s:gpo3 set voltage ok\n", __func__);
			}
		} else
			gpo_regulator = NULL;
	}

	/* This function attaches this structure to the /dev/video0 device.
	 * The pointer in priv points to the ov2640_data structure here.*/
	ov2640_int_device.priv = &ov2640_data;
	retval = v4l2_int_device_register(&ov2640_int_device);

	return retval;

err4:
	if (analog_regulator) {
		regulator_disable(analog_regulator);
		regulator_put(analog_regulator);
	}
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
 * ov2640 I2C detach function
 * Called on rmmod ov2640_camera.ko
 *
 * @param client            struct i2c_client*
 * @return  Error code indicating success or failure
 */
static int ov2640_remove(struct i2c_client *client)
{
	pr_debug("In ov2640_remove\n");

	v4l2_int_device_unregister(&ov2640_int_device);

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

static const struct i2c_device_id ov2640_id[] = {
	{"ov2640", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ov2640_id);

static struct i2c_driver ov2640_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "ov2640",
		  },
	.probe = ov2640_probe,
	.remove = ov2640_remove,
	.id_table = ov2640_id,
/* To add power management add .suspend and .resume functions */
};

/*!
 * ov2640 init function
 * Called by insmod ov2640_camera.ko.
 *
 * @return  Error code indicating success or failure
 */
static __init int ov2640_init(void)
{
	u8 err;

	pr_debug("In ov2640_init\n");

	err = i2c_add_driver(&ov2640_i2c_driver);
	if (err != 0)
		pr_err("%s:driver registration failed, error=%d \n",
			__func__, err);

	return err;
}

/*!
 * OV2640 cleanup function
 * Called on rmmod ov2640_camera.ko
 *
 * @return  Error code indicating success or failure
 */
static void __exit ov2640_clean(void)
{
	pr_debug("In ov2640_clean\n");
	i2c_del_driver(&ov2640_i2c_driver);
}

module_init(ov2640_init);
module_exit(ov2640_clean);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("OV2640 Camera Driver");
MODULE_LICENSE("GPL");
