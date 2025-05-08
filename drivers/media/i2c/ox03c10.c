// SPDX-License-Identifier: GPL-2.0+
/*
 * OX03C10 camera sensor driver library. The sensor is usually paired with a serializer device.
 *
 * Copyright 2024-2025 NXP
 *
 */

#include <linux/i2c.h>
#include <linux/regmap.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-common.h>
#include <media/v4l2-fwnode.h>

#include <uapi/linux/ox03c10.h>

#include "ox03c10.h"
#include "ox03c10_regs.h"

#define OX03C10_I2C_ADDR		0x36
#define OX03C10_PIXEL_RATE		90000000L

#define OX03C10_EXPOSURE_MIN		2U

#define OX03C10_AGAIN_MIN		0x10000L /* Q16.16 for 1.0 */
#define OX03C10_AGAIN_MAX		0xF0000 /* Q16.16 for 15.0 */

#define OX03C10_DGAIN_MIN		0x10000L /* Q16.16 for 1.0 */
#define OX03C10_DGAIN_MAX		0xF0000 /* Q16.16 for 15.0 */

#define OX03C10_L2S_RATIO		0x100000L /* Q16.16 for 16 */
#define OX03C10_L2VS_RATIO		0x800000L /* Q16.16 for 128 */
#define OX03C10_L2SPD_RATIO		0x4000000L /* Q16.16 for 1024 */

#define OX03C10_GAIN_CONV_RATIO		0x751EBL /* Q16.16 for 7.32 */
#define OX03C10_LPD_SPD_SENS_RATIO	0x6C8000L /* Q16.16 for 108.5 */

#define OX03C10_GAIN_VS_MIN		(OX03C10_AGAIN_MIN * OX03C10_DGAIN_MIN / 0x10000)
#define OX03C10_GAIN_S_MIN		(OX03C10_AGAIN_MIN * OX03C10_DGAIN_MIN / 0x10000)
#define OX03C10_GAIN_L_MIN		(OX03C10_GAIN_S_MIN * OX03C10_L2S_RATIO / 0x10000)

#define OX03C10_EXPOSURE_LINES_MIN	2
#define OX03C10_EXPOSURE_LINES_VS_MAX	31
#define OX03C10_EXPOSURE_LINES_VS_MIN	0

#define OX03C10_AGAIN_RANGE_1_MASK	0xFF000U
#define OX03C10_AGAIN_RANGE_2_MASK	0xFE000U
#define OX03C10_AGAIN_RANGE_3_MASK	0xFC000U
#define OX03C10_AGAIN_RANGE_4_MASK	0xF8000U
#define OX03C10_DGAIN_MASK		0xFFFC0U

#define OX03C10_AGAIN_RANGE_1_MIN	0x10000U
#define OX03C10_AGAIN_RANGE_1_MAX	0x20000U
#define OX03C10_AGAIN_RANGE_2_MIN	OX03C10_AGAIN_RANGE_1_MAX
#define OX03C10_AGAIN_RANGE_2_MAX	0x40000U
#define OX03C10_AGAIN_RANGE_3_MIN	OX03C10_AGAIN_RANGE_2_MAX
#define OX03C10_AGAIN_RANGE_3_MAX	0x80000U

#define OX03C10_CTRL_AGAIN_MIN		((OX03C10_AGAIN_MIN * OX03C10_GAIN_CONV_RATIO) >> 16)
#define OX03C10_CTRL_AGAIN_MAX		((OX03C10_AGAIN_MAX * OX03C10_GAIN_CONV_RATIO) >> 16)

#ifdef USE_OFFSET_M
#define OFFSET_M 0xEE /* U10.10 for 0.232621227534758f */
#else
#define OFFSET_M 0x00
#endif
#define OFFSET_VS 0x2EB /* U10.10 for 0.729738894540522f */

#define OX03C10_PWL_LUT_SIZE		132

static const struct ox03c10_reg {
	u16	addr;
	u8	val;
} ox03c10_init_data[] = {
	{ 0x4d5a, 0x1c }, { 0x0309, 0x02 }, { 0x0320, 0x02 }, { 0x0323, 0x05 }, { 0x0362, 0x8a },
	{ 0x0363, 0x00 }, { 0x2803, 0xfe }, { 0x280c, 0x79 }, { 0x3005, 0x80 }, { 0x3007, 0x01 },
	{ 0x3008, 0x80 }, { 0x3020, 0x05 }, { 0x3700, 0x28 }, { 0x3701, 0x15 }, { 0x3702, 0x19 },
	{ 0x3703, 0x23 }, { 0x3704, 0x0a }, { 0x3706, 0x3e }, { 0x3707, 0x0d }, { 0x3708, 0x50 },
	{ 0x3709, 0x5a }, { 0x370b, 0x96 }, { 0x3711, 0x11 }, { 0x372c, 0x40 }, { 0x3738, 0x36 },
	{ 0x3739, 0x36 }, { 0x373a, 0x25 }, { 0x373b, 0x25 }, { 0x3747, 0x28 }, { 0x3748, 0x28 },
	{ 0x3749, 0x19 }, { 0x3755, 0x1a }, { 0x3756, 0x0a }, { 0x3757, 0x1c }, { 0x3765, 0x19 },
	{ 0x3766, 0x05 }, { 0x3767, 0x05 }, { 0x3768, 0x13 }, { 0x3778, 0x20 }, { 0x377c, 0xc8 },
	{ 0x3781, 0x02 }, { 0x3783, 0x02 }, { 0x37c0, 0x01 }, { 0x37c4, 0x3e }, { 0x37c5, 0x3e },
	{ 0x37c6, 0x2a }, { 0x37c7, 0x28 }, { 0x37c9, 0x12 }, { 0x37cb, 0x29 }, { 0x37cd, 0x29 },
	{ 0x37d3, 0x73 }, { 0x37d7, 0x6b }, { 0x37df, 0x54 }, { 0x37f9, 0x01 }, { 0x37fb, 0x19 },
	{ 0x3c03, 0x01 }, { 0x3c04, 0x01 }, { 0x3c06, 0x21 }, { 0x3c08, 0x01 }, { 0x3c09, 0x01 },
	{ 0x3c0a, 0x01 }, { 0x3c0b, 0x21 }, { 0x3c13, 0x21 }, { 0x3c14, 0x82 }, { 0x3c16, 0x13 },
	{ 0x3c22, 0xf3 }, { 0x3c37, 0x12 }, { 0x3c38, 0x31 }, { 0x3c3c, 0x00 }, { 0x3c3d, 0x03 },
	{ 0x3c44, 0x16 }, { 0x3c5c, 0x8a }, { 0x3c5f, 0x03 }, { 0x3c6f, 0x2b }, { 0x3c70, 0x5f },
	{ 0x3c71, 0x2c }, { 0x3c72, 0x2c }, { 0x3c73, 0x2c }, { 0x3c76, 0x12 }, { 0x3182, 0x12 },
	{ 0x3219, 0x08 }, { 0x3506, 0x30 }, { 0x3586, 0x60 }, { 0x3549, 0x40 }, { 0x35c6, 0xa0 },
	{ 0x3605, 0x16 }, { 0x3609, 0xf0 }, { 0x360a, 0x01 }, { 0x360f, 0x10 }, { 0x3610, 0x70 },
	{ 0x3611, 0x3a }, { 0x3612, 0x28 }, { 0x361a, 0x29 }, { 0x361b, 0x6c }, { 0x361c, 0x0b },
	{ 0x361d, 0x00 }, { 0x364d, 0x0f }, { 0x364e, 0x18 }, { 0x364f, 0x12 }, { 0x3653, 0x1c },
	{ 0x3655, 0x1f }, { 0x3656, 0x1f }, { 0x3657, 0x0c }, { 0x3658, 0x0a }, { 0x3659, 0x14 },
	{ 0x365a, 0x18 }, { 0x365b, 0x14 }, { 0x365c, 0x10 }, { 0x365e, 0x12 }, { 0x3674, 0x08 },
	{ 0x3677, 0x3a }, { 0x3678, 0x3a }, { 0x3679, 0x19 }, { 0x3820, 0x20 }, { 0x3832, 0x00 },
	{ 0x3834, 0x00 }, { 0x3b40, 0x05 }, { 0x3b41, 0x40 }, { 0x3b43, 0x90 }, { 0x3b44, 0x02 },
	{ 0x3b45, 0x00 }, { 0x3b46, 0x02 }, { 0x3b47, 0x00 }, { 0x3b48, 0x19 }, { 0x3b49, 0x12 },
	{ 0x3b4a, 0x16 }, { 0x3b4b, 0x2e }, { 0x3b87, 0x34 }, { 0x3b89, 0x08 }, { 0x3b8a, 0x05 },
	{ 0x3b8b, 0x00 }, { 0x3b8d, 0x80 }, { 0x3b92, 0x05 }, { 0x3b93, 0x00 }, { 0x3b95, 0x80 },
	{ 0x3b9e, 0x09 }, { 0x3d82, 0x73 }, { 0x3d85, 0x05 }, { 0x3d9a, 0x9f }, { 0x3d9c, 0xa0 },
	{ 0x3da4, 0x00 }, { 0x3da7, 0x50 }, { 0x421f, 0x45 }, { 0x4301, 0xff }, { 0x430a, 0x13 },
	{ 0x430d, 0x93 },
	{ 0x430e, 0x14 }, /* bottom emb DT not as image DT */
	{ 0x430f, 0x17 }, /* disable output statistics */
	{ 0x4317, 0x28 }, /* enable top and disable bottom emb lines */
	{ 0x4319, 0x03 }, { 0x431f, 0x30 }, { 0x4583, 0x07 }, { 0x4584, 0x6a }, { 0x4585, 0x08 },
	{ 0x4586, 0x05 }, { 0x4587, 0x04 }, { 0x4588, 0x73 }, { 0x4589, 0x05 }, { 0x458a, 0x1f },
	{ 0x458b, 0x02 }, { 0x458c, 0xdc }, { 0x458d, 0x03 }, { 0x458e, 0x02 }, { 0x4597, 0x07 },
	{ 0x4598, 0x40 }, { 0x4599, 0x0e }, { 0x459a, 0x0e }, { 0x459b, 0xfb }, { 0x459c, 0xf3 },
	{ 0x480a, 0x22 }, { 0x4d15, 0x7d }, { 0x4d30, 0x0a }, { 0x4d31, 0x00 }, { 0x4d34, 0x7d },
	{ 0x4d3c, 0x7d },
	{ 0x5002, 0x60 }, /* PWL and retiming enable, all statistics blocks disabled */
	{ 0x6007, 0x04 }, { 0x6008, 0x05 }, { 0x6009, 0x02 }, { 0x600b, 0x08 }, { 0x600c, 0x07 },
	{ 0x600d, 0x88 }, { 0x6027, 0x04 }, { 0x6028, 0x05 }, { 0x6029, 0x02 }, { 0x602b, 0x08 },
	{ 0x602c, 0x07 }, { 0x602d, 0x88 }, { 0x6047, 0x04 }, { 0x6048, 0x05 }, { 0x6049, 0x02 },
	{ 0x604b, 0x08 }, { 0x604c, 0x07 }, { 0x604d, 0x88 }, { 0x6067, 0x04 }, { 0x6068, 0x05 },
	{ 0x6069, 0x02 }, { 0x606b, 0x08 }, { 0x606c, 0x07 }, { 0x606d, 0x88 }, { 0x6087, 0x04 },
	{ 0x6088, 0x05 }, { 0x6089, 0x02 }, { 0x608b, 0x08 }, { 0x608c, 0x07 }, { 0x608d, 0x88 },
	{ 0x5e01, 0x0f }, { 0x5e02, 0x0f }, { 0x5e03, 0x10 }, { 0x5e04, 0x11 }, { 0x5e05, 0x12 },
	{ 0x5e06, 0x13 }, { 0x5e07, 0x00 }, { 0x5e08, 0x00 }, { 0x5e09, 0x00 }, { 0x5e0a, 0x00 },
	{ 0x5e0b, 0x00 }, { 0x5e0c, 0x00 }, { 0x5e0d, 0x00 }, { 0x5e0e, 0x00 }, { 0x5e0f, 0x00 },
	{ 0x5e10, 0x00 }, { 0x5e11, 0x00 }, { 0x5e12, 0x00 }, { 0x5e13, 0x00 }, { 0x5e14, 0x00 },
	{ 0x5e15, 0x00 }, { 0x5e16, 0x00 }, { 0x5e17, 0x00 }, { 0x5e18, 0x00 }, { 0x5e19, 0x00 },
	{ 0x5e1a, 0x00 }, { 0x5e1b, 0x00 }, { 0x5e1c, 0x00 }, { 0x5e1d, 0x00 }, { 0x5e1e, 0x00 },
	{ 0x5e1f, 0x00 }, { 0x5e20, 0x00 }, { 0x5e21, 0x00 }, { 0x5e23, 0x7f }, { 0x5e24, 0xff },
	{ 0x5e26, 0x40 }, { 0x5e29, 0x20 }, { 0x5e2c, 0x04 }, { 0x5e2d, 0x92 }, { 0x5e2f, 0x09 },
	{ 0x5e30, 0x25 }, { 0x5e32, 0x12 }, { 0x5e33, 0x49 }, { 0x5e35, 0x00 }, { 0x5e38, 0x00 },
	{ 0x5e3b, 0x00 }, { 0x5e3e, 0x00 }, { 0x5e41, 0x00 }, { 0x5e44, 0x00 }, { 0x5e47, 0x00 },
	{ 0x5e4a, 0x00 }, { 0x5e4d, 0x00 }, { 0x5e4f, 0x7f }, { 0x5e50, 0x00 }, { 0x5e53, 0x00 },
	{ 0x5e56, 0x00 }, { 0x5e59, 0x00 }, { 0x5e5c, 0x00 }, { 0x5e5e, 0x7f }, { 0x5e5f, 0x00 },
	{ 0x5e62, 0x00 }, { 0x5e65, 0x00 }, { 0x5e68, 0x00 }, { 0x5e6d, 0x7f }, { 0x5e6e, 0x00 },
	{ 0x5e71, 0x00 }, { 0x5e74, 0x00 }, { 0x5e77, 0x00 }, { 0x5e7a, 0x00 }, { 0x5e7d, 0x00 },
	{ 0x5e80, 0x00 }, { 0x5e83, 0x20 }, { 0x5e84, 0x00 }, { 0x4008, 0x02 }, { 0x4009, 0x03 },
	{ 0x4022, 0x40 }, { 0x4023, 0x20 }, { 0x4082, 0x01 }, { 0x4083, 0x53 }, { 0x4084, 0x01 },
	{ 0x4085, 0x2b }, { 0x4086, 0x00 }, { 0x4087, 0xb3 }, { 0x4641, 0x11 }, { 0x4642, 0x0e },
	{ 0x4643, 0xee }, { 0x4646, 0x0f }, { 0x5003, 0x7a }, { 0x5b80, 0x08 }, { 0x5c00, 0x08 },
	{ 0x5c80, 0x00 }, { 0x5b8e, 0x60 }, { 0x5b92, 0x80 }, { 0x5b97, 0x20 }, { 0x5b9a, 0x40 },
	{ 0x5b9b, 0x20 }, { 0x5b9c, 0x00 }, { 0x5b9f, 0x00 }, { 0x5ba0, 0x00 }, { 0x5ba1, 0x00 },
	{ 0x5ba3, 0x00 }, { 0x5ba4, 0x00 }, { 0x5ba5, 0x00 }, { 0x5bae, 0x00 }, { 0x5baf, 0x80 },
	{ 0x5bb0, 0x00 }, { 0x5bb1, 0xc0 }, { 0x5bb2, 0x01 }, { 0x5bb3, 0x00 }, { 0x5c30, 0x00 },
	{ 0x5c31, 0xc0 }, { 0x5c32, 0x01 }, { 0x5c9d, 0x00 }, { 0x5ca5, 0x00 }, { 0x5be7, 0x80 },
	{ 0x5bd2, 0x20 }, { 0x5bd4, 0x40 }, { 0x5bd5, 0x20 }, { 0x5bd6, 0x00 }, { 0x5bd7, 0x00 },
	{ 0x5bd8, 0x00 }, { 0x5bd9, 0x00 }, { 0x5bda, 0x00 }, { 0x5bdb, 0x00 }, { 0x5bdc, 0x00 },
	{ 0x5bdd, 0x00 }, { 0x5bde, 0x00 }, { 0x5bdf, 0x00 }, { 0x5be0, 0x00 }, { 0x5c4d, 0x40 },
	{ 0x5c51, 0x60 }, { 0x5c52, 0x20 }, { 0x5c55, 0x80 }, { 0x5c56, 0x20 }, { 0x5c57, 0x00 },
	{ 0x5c59, 0x40 }, { 0x5c5a, 0x20 }, { 0x5c5b, 0x00 }, { 0x5c5c, 0x00 }, { 0x5c5d, 0x80 },
	{ 0x5c5e, 0x00 }, { 0x5c5f, 0x00 }, { 0x5c60, 0x00 }, { 0x5cd5, 0x80 }, { 0x5cd6, 0x60 },
	{ 0x5cd9, 0x80 }, { 0x5cda, 0x80 }, { 0x5cdb, 0x40 }, { 0x5cdd, 0x80 }, { 0x5cde, 0x80 },
	{ 0x5cdf, 0x80 }, { 0x5ce2, 0x80 }, { 0x5ce3, 0x80 }, { 0x5ce4, 0x80 }, { 0x52c9, 0x02 },
	{ 0x52ca, 0x01 }, { 0x52cb, 0x01 }, { 0x52cd, 0x02 }, { 0x52ce, 0x01 }, { 0x52cf, 0x01 },
	{ 0x54c9, 0x02 }, { 0x54ca, 0x01 }, { 0x54cb, 0x01 }, { 0x54cd, 0x02 }, { 0x54ce, 0x01 },
	{ 0x54cf, 0x01 }, { 0x56c9, 0x02 }, { 0x56ca, 0x01 }, { 0x56cb, 0x01 }, { 0x56cd, 0x02 },
	{ 0x56ce, 0x01 }, { 0x56cf, 0x01 }, { 0x58c9, 0x02 }, { 0x58ca, 0x01 }, { 0x58cb, 0x01 },
	{ 0x58cd, 0x02 }, { 0x58ce, 0x01 }, { 0x58cf, 0x01 }, { 0x5d15, 0x05 }, { 0x5d16, 0x05 },
	{ 0x5d17, 0x05 }, { 0x5d09, 0xb6 }, { 0x5d0b, 0xb6 }, { 0x5d19, 0xb6 }, { 0x5d62, 0x01 },
	{ 0x5d40, 0x02 }, { 0x5d63, 0x20 }, { 0x5d65, 0xff }, { 0x5d59, 0x20 }, { 0x5d5b, 0x20 },
	{ 0x5d5e, 0x03 }, { 0x5d5f, 0xb6 }, { 0x5d60, 0x03 }, { 0x5d61, 0xb6 }, { 0x5d4a, 0x02 },
	{ 0x5d4b, 0x40 }, { 0x5d4c, 0x10 }, { 0x5d4d, 0x40 }, { 0x5d4e, 0x10 }, { 0x5d4f, 0x40 },
	{ 0x5d50, 0x18 }, { 0x5d51, 0x80 }, { 0x5d52, 0x20 }, { 0x5d53, 0x80 }, { 0x5d54, 0x20 },
	{ 0x5d55, 0x80 }, { 0x5d47, 0x20 }, { 0x5d49, 0x60 }, { 0x5d66, 0x01 }, { 0x5004, 0x1e },
	{ 0x4221, 0x03 }, { 0x3501, 0x01 }, { 0x3502, 0xc8 }, { 0x3541, 0x01 }, { 0x3542, 0xc8 },
	{ 0x35c2, 0x01 }, { 0x420e, 0x54 }, { 0x420f, 0xa0 }, { 0x4210, 0xca }, { 0x4211, 0xf2 },
	{ 0x507a, 0x5f }, { 0x507b, 0x46 }, { 0x4f00, 0x00 }, { 0x4f01, 0x00 }, { 0x4f02, 0x80 },
	{ 0x4f03, 0x2c }, { 0x4f04, 0xf8 }, { 0x0307, 0x03 }, { 0x4837, 0x1a }, { 0x040d, 0xed },
	{ 0x0408, 0x70 }, { 0x0409, 0x62 }, { 0x040a, 0x2d }, { 0x040b, 0x09 }, { 0x0324, 0x01 },
	{ 0x0325, 0x36 }, { 0x0329, 0x02 }, { 0x032a, 0x05 }, { 0x032b, 0x08 }, { 0x032c, 0x02 },
	{ 0x0327, 0x09 }, { 0x0326, 0x0e }, { 0x380c, 0x05 }, { 0x380d, 0xe2 }, { 0x384d, 0xf1 },
	{ 0x0404, 0x09 }, { 0x0405, 0x2b }, { 0x0406, 0x8d }, { 0x388d, 0xf1 }, { 0x0400, 0x70 },
	{ 0x0401, 0x7f }, { 0x0403, 0x2d },

	/* Fsync */
	{ 0x3015, 0x0A }, { 0x3009, 0x02 }, { 0x3822, 0x24 }, { 0x3823, 0x50 }, { 0x383e, 0x81 },
	{ 0x3881, 0x34 }, { 0x3882, 0x02 }, { 0x3883, 0x8a }, { 0x3892, 0x44 },

	/* Declaring the registers to be included in the embedded data */
	{ 0x3208, 0x04 }, { 0x350e, 0x02 }, { 0x3514, 0x02 }, { 0x3518, 0x03 }, { 0x354e, 0x02 },
	{ 0x3554, 0x02 }, { 0x3558, 0x03 }, { 0x3594, 0x02 }, { 0x3598, 0x03 }, { 0x35ce, 0x02 },
	{ 0x35d4, 0x02 }, { 0x35d8, 0x03 }, { 0x483E, 0x02 }, { 0x4D2A, 0x02 }, { 0x5280, 0x08 },
	{ 0x5480, 0x08 }, { 0x5680, 0x08 }, { 0x5880, 0x0A }, { 0x3208, 0x14 },

	{ 0x431c, 0x6e }, { 0x0100, 0x00 },
};

static const struct regmap_range ox03c10_volatile_ranges[] = {
	{ 0x7057, 0x7059 }, { 0x705b, 0x705d }, { 0x705f, 0x7061 }, /* OTP correction registers */

};

static const struct regmap_access_table ox03c10_volatile_access_table = {
	.yes_ranges = ox03c10_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(ox03c10_volatile_ranges),
};

static const struct regmap_config ox03c10_sensor_regmap_cfg = {
	.name = "ox03c10",
	.reg_bits = 16,
	.val_bits = 8,

	.max_register = 0x7061,
	.volatile_table = &ox03c10_volatile_access_table,
	.cache_type = REGCACHE_RBTREE,
};

static struct ox03c10_mode ox03c10_modes[] = {
	{
		.width = OX03C10_PIXEL_ARRAY_WIDTH,
		.height = OX03C10_PIXEL_ARRAY_HEIGHT,
		.hts = 2186,
		.vts = 1372,
		.fps = 30,
		.crop = {
			.left = 8,
			.top = 4,
			.width = 1920,
			.height = 1280,
		},
	},
	{
		.width = 1920,
		.height = 1082,
		.hts = 2186,
		.vts = 1372,
		.fps = 30,
		.crop = {
			.left = 8,
			.top = 104,
			.width = 1920,
			.height = 1080,
		},
	},
};

const struct ox03c10_mode *ox03c10_find_closest_mode(struct ox03c10 *sensor, u16 width, u16 height)
{
	return v4l2_find_nearest_size(ox03c10_modes, ARRAY_SIZE(ox03c10_modes),
				      width, height, width, height);
}
EXPORT_SYMBOL(ox03c10_find_closest_mode);

static inline u32 ox03c10_get_dbl_row_time_ns(u32 hts_pixels)
{
	/*
	 * According to the specifications, dbl_row_time = HTS / SCLK, where HTS is the horizontal
	 * time size, measured in SCLK cycles and SCLK is the system clock. However, we can easily
	 * derive the row_time from the mode.
	 * For example:
	 *  * using HTS/SCLK: HTS=3012 cycles and SCLK = 62MHz => dbl_row_time = 48.58us
	 *  * using the mode where hts is 2186 pixels and the pixel clock is 90MHz:
	 *      dbl_row_time = 2 * hts / 90000000 = 48.58us
	 */

	return 2 * (u64)hts_pixels * NSEC_PER_SEC / OX03C10_PIXEL_RATE;
}

static u32 ox03c10_us_to_dbl_rows(const struct ox03c10_mode *mode, u32 exposure_us)
{
	u32 dbl_row_time_ns = ox03c10_get_dbl_row_time_ns(mode->hts);

	return (exposure_us * 1000 + dbl_row_time_ns / 2) / dbl_row_time_ns;
}

static u32 ox03c10_dbl_rows_to_us(const struct ox03c10_mode *mode, u32 exposure_in_dbl_rows)
{
	u32 dbl_row_time_ns = ox03c10_get_dbl_row_time_ns(mode->hts);

	return (exposure_in_dbl_rows * dbl_row_time_ns) / 1000;
}

static u32 ox03c10_calc_additional_gain(const struct ox03c10_mode *mode, u32 exposure_us,
					u32 exposure_in_dbl_rows)
{
	u32 dbl_row_time_ns = ox03c10_get_dbl_row_time_ns(mode->hts);
	const u64 exposure_ns = exposure_us * 1000;
	const u64 exposure_dbl_rows_ns = exposure_in_dbl_rows * dbl_row_time_ns;

	return (exposure_ns * 0x100U + exposure_dbl_rows_ns / 2) / exposure_dbl_rows_ns;
}

static u32 ox03c10_distribute_again(u32 gain, u32 min_gain, u32 max_gain, u32 *dgain)
{
	u64 tmp_gain;
	u64 current_dgain = *dgain;
	u32 res_gain = gain;

	if (max_gain < res_gain) {
		tmp_gain = res_gain;
		res_gain = max_gain;
		/* Carry overflow gain into digitalGain */
		current_dgain = (tmp_gain * current_dgain) / res_gain;
	} else {
		/* Should not enter here, fractional gain is bad, but just for completeness sake */
		if (min_gain > res_gain)
			res_gain = min_gain;
	}

	tmp_gain = res_gain;

	/* Select appropriate mask for Analog gain. See page 42 of data sheet */
	if (res_gain <= OX03C10_AGAIN_RANGE_1_MAX)
		res_gain = res_gain & OX03C10_AGAIN_RANGE_1_MASK;
	else if (res_gain <= OX03C10_AGAIN_RANGE_2_MAX)
		res_gain = res_gain & OX03C10_AGAIN_RANGE_2_MASK;
	else if (res_gain <= OX03C10_AGAIN_RANGE_3_MAX)
		res_gain = res_gain & OX03C10_AGAIN_RANGE_3_MASK;
	else
		res_gain = res_gain & OX03C10_AGAIN_RANGE_4_MASK;

	/* Attempt to carry masked gain into digital */
	*dgain = (u32)((current_dgain * tmp_gain + res_gain / 2U) / res_gain);

	return res_gain;
}

static u32 ox03c10_distribute_dgain(u32 gain, u32 min_gain, u32 max_gain)
{
	gain = clamp_t(u32, gain, min_gain, max_gain);

	/* Mask gain to valid settings */
	return gain & OX03C10_DGAIN_MASK;
}

static int ox03c10_gh_set(struct ox03c10 *sensor, int gh_no)
{
	if (!sensor->streaming || sensor->gh_open[gh_no])
		return 0;

	sensor->gh_open[gh_no] = true;

	return regmap_write(sensor->rmap, OX03C10_GRP_HOLD_8, gh_no & 0xf);
}

static int ox03c10_gh_close_and_launch(struct ox03c10 *sensor, int gh_no)
{
	int ret;

	if (!sensor->streaming || !sensor->gh_open[gh_no])
		return 0;

	sensor->gh_open[gh_no] = false;

	ret = regmap_write(sensor->rmap, OX03C10_GRP_HOLD_8, 0x10 | (gh_no & 0xf));
	if (ret)
		return ret;

	return regmap_write(sensor->rmap, OX03C10_GRP_HOLD_8, 0xE0 | (gh_no & 0xf));
}

static int ox03c10_exposure_set(struct ox03c10 *sensor, struct ox03c10_exposure *exp)
{
	int ret = 0;
	u8 buf[2];

	if (exp->dcg != sensor->exposure.dcg) {
		buf[0] = (exp->dcg >> 8) & 0xff;
		buf[1] = exp->dcg & 0xff;
		ret |= regmap_bulk_write(sensor->rmap, OX03C10_AEC_HCG_CTRL_01, buf, 2);
	}

	if (exp->spd != sensor->exposure.spd) {
		buf[0] = (exp->spd >> 8) & 0xff;
		buf[1] = exp->spd & 0xff;
		ret |= regmap_bulk_write(sensor->rmap, OX03C10_AEC_SPD_CTRL_01, buf, 2);
	}

	if (exp->vs != sensor->exposure.vs) {
		if (exp->vs > 4 && !sensor->streaming)
			return -EINVAL;

		buf[0] = (exp->vs >> 8) & 0xff;
		buf[1] = exp->vs & 0xff;
		ret |= regmap_bulk_write(sensor->rmap, OX03C10_AEC_VS_CTRL_01, buf, 2);
	}

	sensor->exposure = *exp;

	return ret ? -EIO : 0;
}

static int ox03c10_exposure_set_gh(struct ox03c10 *sensor, struct ox03c10_exposure *exp)
{
	int ret = 0;

	ret = ox03c10_gh_set(sensor, 0);
	if (ret)
		return ret;

	ret = ox03c10_exposure_set(sensor, exp);
	if (ret)
		return ret;

	return ox03c10_gh_close_and_launch(sensor, 0);
}

static int ox03c10_analogue_gain_set(struct ox03c10 *sensor, struct ox03c10_analog_gain *gain)
{
	int ret = 0;
	u8 buf[2];

	if (gain->hcg != sensor->again.hcg) {
		buf[0] = (gain->hcg >> 4) & 0xf;
		buf[1] = (gain->hcg & 0xf) << 4;
		ret |= regmap_bulk_write(sensor->rmap, OX03C10_AEC_HCG_CTRL_08, buf, 2);
	}

	if (gain->spd != sensor->again.spd) {
		buf[0] = (gain->spd >> 4) & 0xf;
		buf[1] = (gain->spd & 0xf) << 4;
		ret |= regmap_bulk_write(sensor->rmap, OX03C10_AEC_SPD_CTRL_08, buf, 2);
	}

	if (gain->lcg != sensor->again.lcg) {
		buf[0] = (gain->lcg >> 4) & 0xf;
		buf[1] = (gain->lcg & 0xf) << 4;
		ret |= regmap_bulk_write(sensor->rmap, OX03C10_AEC_LCG_CTRL_08, buf, 2);
	}

	if (gain->vs != sensor->again.vs) {
		buf[0] = (gain->vs >> 4) & 0xf;
		buf[1] = (gain->vs & 0xf) << 4;
		ret |= regmap_bulk_write(sensor->rmap, OX03C10_AEC_VS_CTRL_08, buf, 2);
	}

	sensor->again = *gain;

	return ret ? -EIO : 0;
}

static int ox03c10_analogue_gain_set_gh(struct ox03c10 *sensor, struct ox03c10_analog_gain *gain)
{
	int ret = 0;

	ret = ox03c10_gh_set(sensor, 0);
	if (ret)
		return ret;

	ret = ox03c10_analogue_gain_set(sensor, gain);
	if (ret)
		return ret;

	return ox03c10_gh_close_and_launch(sensor, 0);
}

static int ox03c10_digital_gain_set(struct ox03c10 *sensor, struct ox03c10_digital_gain *gain)
{
	int ret = 0;
	u8 buf[3];

	if (gain->hcg != sensor->dgain.hcg) {
		buf[0] = (gain->hcg >> 10) & 0xf;
		buf[1] = (gain->hcg >> 2) & 0xff;
		buf[2] = (gain->hcg & 0x3) << 6;
		ret |= regmap_bulk_write(sensor->rmap, OX03C10_AEC_HCG_CTRL_0A, buf, 3);
	}

	if (gain->spd != sensor->dgain.spd) {
		buf[0] = (gain->spd >> 10) & 0xf;
		buf[1] = (gain->spd >> 2) & 0xff;
		buf[2] = (gain->spd & 0x3) << 6;
		ret |= regmap_bulk_write(sensor->rmap, OX03C10_AEC_SPD_CTRL_0A, buf, 3);
	}

	if (gain->lcg != sensor->dgain.lcg) {
		buf[0] = (gain->lcg >> 10) & 0xf;
		buf[1] = (gain->lcg >> 2) & 0xff;
		buf[2] = (gain->lcg & 0x3) << 6;
		ret |= regmap_bulk_write(sensor->rmap, OX03C10_AEC_LCG_CTRL_0A, buf, 3);
	}

	if (gain->vs != sensor->dgain.vs) {
		buf[0] = (gain->vs >> 10) & 0xf;
		buf[1] = (gain->vs >> 2) & 0xff;
		buf[2] = (gain->vs & 0x3) << 6;
		ret |= regmap_bulk_write(sensor->rmap, OX03C10_AEC_VS_CTRL_0A, buf, 3);
	}

	sensor->dgain = *gain;

	return ret ? -EIO : 0;
}

static int ox03c10_digital_gain_set_gh(struct ox03c10 *sensor, struct ox03c10_digital_gain *gain)
{
	int ret = 0;

	ret = ox03c10_gh_set(sensor, 0);
	if (ret)
		return ret;

	ret = ox03c10_digital_gain_set(sensor, gain);
	if (ret)
		return ret;

	return ox03c10_gh_close_and_launch(sensor, 0);
}

static int ox03c10_exposure_and_gains_update(struct ox03c10 *sensor, s32 exposure,
					     s32 again, s32 dgain)
{
	u64 total_exposure_l, total_exposure_l_rows;
	u32 exposure_us, exposure_dcg, exposure_spd, exposure_vs;
	u64 again_hcg, again_lcg, again_spd, again_vs;
	u32 dgain_hcg, dgain_lcg, dgain_spd, dgain_vs;
	struct ox03c10_exposure computed_exposure;
	struct ox03c10_analog_gain computed_again;
	struct ox03c10_digital_gain computed_dgain;
	u64 add_gain;
	int ret = 0;
	/* in double-rows */
	u32 max_exposure_lines = (sensor->cur_mode->vts / 2) - OX03C10_EXPOSURE_LINES_VS_MAX - 13;

	ret = ox03c10_gh_set(sensor, 0);
	if (ret)
		return ret;

	/* save the current exposure and gains values */
	sensor->exposure_input = exposure;
	sensor->again_input = again;
	sensor->dgain_input = dgain;

	/*
	 * According to specifications, the exposure and gains registers' values are in double-rows.
	 * From here on, the algorithm uses exposure in double-rows to perform the adjustments.
	 */
	exposure_us = ox03c10_dbl_rows_to_us(sensor->cur_mode, exposure / 2);

	exposure_dcg = exposure / 2;
	again_hcg = (u64)again * dgain / 0x10000;

	if (again_hcg < OX03C10_GAIN_L_MIN) {
		dev_dbg(sensor->dev, "Gain below minimum (0x%llx < 0x%lx). Value adjusted.\n",
			again_hcg, OX03C10_GAIN_L_MIN);

		total_exposure_l = exposure_us * 1000 * again_hcg;
		again_hcg = OX03C10_GAIN_L_MIN;
		exposure_us = total_exposure_l / (1000 * again_hcg);
		exposure_dcg = ox03c10_us_to_dbl_rows(sensor->cur_mode, exposure_us);

		if (exposure_dcg < OX03C10_EXPOSURE_LINES_MIN)
			exposure_dcg = OX03C10_EXPOSURE_LINES_MIN;
	}

	add_gain = ox03c10_calc_additional_gain(sensor->cur_mode, exposure_us, exposure_dcg);
	again_hcg = (again_hcg * add_gain + 256 / 2) / 256;

	if (again_hcg < OX03C10_GAIN_L_MIN)
		again_hcg = OX03C10_GAIN_L_MIN;

	again_lcg = again_hcg * 0x10000 / OX03C10_L2S_RATIO;

	/* SPD distribution */
	exposure_spd = max_exposure_lines; /* default to max */
	total_exposure_l_rows = again_hcg * ((u64)exposure_dcg *
					     OX03C10_LPD_SPD_SENS_RATIO / 0x10000);
	again_spd = total_exposure_l_rows * 0x10000 /
				(OX03C10_L2SPD_RATIO * (u64)exposure_spd);

	if (again_spd < OX03C10_AGAIN_MIN) {
		again_spd = OX03C10_AGAIN_MIN;
		exposure_spd = total_exposure_l_rows * 0x10000 /
			(OX03C10_L2SPD_RATIO * again_spd);

		if (exposure_spd <= OX03C10_EXPOSURE_LINES_MIN)
			exposure_spd = OX03C10_EXPOSURE_LINES_MIN;

		again_spd = total_exposure_l_rows * 0x10000 /
				(OX03C10_L2SPD_RATIO * (u64)exposure_spd);
	}

	/* VS distribution */
	exposure_vs = OX03C10_EXPOSURE_LINES_VS_MAX; /* default to max */

	total_exposure_l_rows = again_hcg * ((u64)exposure_dcg * 0x400 + OFFSET_M);
	again_vs = total_exposure_l_rows * 0x10000 /
				(OX03C10_L2VS_RATIO * ((u64)exposure_vs * 0x400 + OFFSET_M));

	if (again_vs < OX03C10_GAIN_VS_MIN) {
		again_vs = OX03C10_GAIN_VS_MIN;
		exposure_vs = total_exposure_l_rows * 0x10000 /
							(OX03C10_L2VS_RATIO * again_vs * 0x400);

		if (exposure_vs <= OX03C10_EXPOSURE_LINES_VS_MIN)
			exposure_vs = OX03C10_EXPOSURE_LINES_VS_MIN;

		again_vs = total_exposure_l_rows * 0x10000 /
				(OX03C10_L2VS_RATIO * ((u64)exposure_vs * 0x400 + OFFSET_VS));
		if (again_vs < OX03C10_GAIN_VS_MIN) {
			exposure_vs = exposure_vs * again_vs / 0x10000;

			if (exposure_vs <= OX03C10_EXPOSURE_LINES_VS_MIN)
				exposure_vs = OX03C10_EXPOSURE_LINES_VS_MIN;

			again_vs = total_exposure_l_rows * 0x10000 /
				(OX03C10_L2VS_RATIO * ((u64)exposure_vs * 0x400 + OFFSET_VS));
		}
	}

	computed_exposure.dcg = exposure_dcg;
	computed_exposure.spd = exposure_spd;
	computed_exposure.vs = exposure_vs;

	ret = __v4l2_ctrl_s_ctrl_compound(sensor->ctrls[OX03C10_EXPOSURE], V4L2_CTRL_TYPE_U8,
					  &computed_exposure);
	if (ret)
		return ret;

	dgain_hcg = OX03C10_DGAIN_MIN;
	dgain_lcg = OX03C10_DGAIN_MIN;
	dgain_spd = OX03C10_DGAIN_MIN;
	dgain_vs = OX03C10_DGAIN_MIN;

	again_hcg = (again_hcg * 0x10000) / (((u64)dgain_hcg * OX03C10_GAIN_CONV_RATIO) / 0x10000);
	again_lcg = (again_lcg * 0x10000) / dgain_lcg;
	again_spd = (again_spd * 0x10000) / dgain_spd;
	again_vs = (again_vs * 0x10000) / dgain_vs;

	again_hcg = ox03c10_distribute_again(again_hcg, OX03C10_AGAIN_MIN,
					     OX03C10_AGAIN_MAX, &dgain_hcg);
	again_lcg = ox03c10_distribute_again(again_lcg, OX03C10_AGAIN_MIN,
					     OX03C10_AGAIN_MAX, &dgain_lcg);
	again_spd = ox03c10_distribute_again(again_spd, OX03C10_AGAIN_MIN,
					     OX03C10_AGAIN_MAX, &dgain_spd);
	again_vs = ox03c10_distribute_again(again_vs, OX03C10_AGAIN_MIN,
					    OX03C10_AGAIN_MAX, &dgain_vs);

	computed_again.hcg = again_hcg >> 12;
	computed_again.lcg = again_lcg >> 12;
	computed_again.spd = again_spd >> 12;
	computed_again.vs = again_vs >> 12;

	ret = __v4l2_ctrl_s_ctrl_compound(sensor->ctrls[OX03C10_AGAIN], V4L2_CTRL_TYPE_U8,
					  &computed_again);
	if (ret)
		return ret;

	dgain_hcg = ox03c10_distribute_dgain(dgain_hcg, OX03C10_DGAIN_MIN, OX03C10_DGAIN_MAX);
	dgain_lcg = ox03c10_distribute_dgain(dgain_lcg, OX03C10_DGAIN_MIN, OX03C10_DGAIN_MAX);
	dgain_spd = ox03c10_distribute_dgain(dgain_spd, OX03C10_DGAIN_MIN, OX03C10_DGAIN_MAX);
	dgain_vs = ox03c10_distribute_dgain(dgain_vs, OX03C10_DGAIN_MIN, OX03C10_DGAIN_MAX);

	computed_dgain.hcg = dgain_hcg >> 6;
	computed_dgain.lcg = dgain_lcg >> 6;
	computed_dgain.spd = dgain_spd >> 6;
	computed_dgain.vs = dgain_vs >> 6;

	ret = __v4l2_ctrl_s_ctrl_compound(sensor->ctrls[OX03C10_DGAIN], V4L2_CTRL_TYPE_U8,
					  &computed_dgain);
	if (ret)
		return ret;

	return ox03c10_gh_close_and_launch(sensor, 0);
}

static int ox03c10_wb_gain_set(struct ox03c10 *sensor, struct ox03c10_wb_capture_gain *wb_gain)
{
	int i, ret = 0;
	u8 buf[8];
	u16 base_addr[4] = {
		OX03C10_AWB_GAIN_HCG_0,
		OX03C10_AWB_GAIN_LCG_0,
		OX03C10_AWB_GAIN_SPD_0,
		OX03C10_AWB_GAIN_VS_0
	};

	for (i = 0; i < 4; i++, wb_gain++) {
		buf[0] = (wb_gain->b >> 8) & 0xff;
		buf[1] = wb_gain->b & 0xff;
		buf[2] = (wb_gain->gb >> 8) & 0xff;
		buf[3] = wb_gain->gb & 0xff;
		buf[4] = (wb_gain->gr >> 8) & 0xff;
		buf[5] = wb_gain->gr & 0xff;
		buf[6] = (wb_gain->r >> 8) & 0xff;
		buf[7] = wb_gain->r & 0xff;

		ret |= regmap_bulk_write(sensor->rmap, base_addr[i], buf, 8);
	}

	return ret ? -EIO : 0;
}

static int ox03c10_wb_gain_set_gh(struct ox03c10 *sensor, struct ox03c10_wb_capture_gain *wb_gain)
{
	int ret = 0;

	ret = ox03c10_gh_set(sensor, 0);
	if (ret)
		return ret;

	ret = ox03c10_wb_gain_set(sensor, wb_gain);
	if (ret)
		return ret;

	return ox03c10_gh_close_and_launch(sensor, 0);
}

static int ox03c10_pwl_enable(struct ox03c10 *sensor, bool en)
{
	int ret;

	if (sensor->streaming)
		return -EBUSY;

	ret = regmap_update_bits(sensor->rmap, OX03C10_FORMAT_REG_1F, BIT(5), en ? BIT(5) : 0);
	ret |= regmap_update_bits(sensor->rmap, OX03C10_ISP_CTRL_02, BIT(6), en ? BIT(6) : 0);

	return ret ? -EIO : 0;
}

static int ox03c10_pwl_params_set(struct ox03c10 *sensor, struct ox03c10_pwl_ctrl *pwl_ctrl)
{
	if (sensor->streaming)
		return -EBUSY;

	return regmap_update_bits(sensor->rmap, OX03C10_FORMAT_REG_1F, 0xD8,
				  (pwl_ctrl->pack24bit_sel << 6) | (pwl_ctrl->pwl_mode << 3));
}

static int ox03c10_pwl_lut_set(struct ox03c10 *sensor, u8 *lut)
{
	if (sensor->streaming)
		return -EBUSY;

	return regmap_bulk_write(sensor->rmap, OX03C10_PWL0_0_1, lut, OX03C10_PWL_LUT_SIZE);
}

static int ox03c10_hflip_enable(struct ox03c10 *sensor, bool en)
{
	int ret;

	if (sensor->streaming)
		return -EBUSY;

	ret = regmap_update_bits(sensor->rmap, OX03C10_REG_WIN_09, BIT(0), BIT(0));
	ret |= regmap_update_bits(sensor->rmap, OX03C10_TIMING_CTRL_REG_20, BIT(5),
				  en ? 0 : BIT(5));

	return ret ? -EIO : 0;
}

static int ox03c10_vflip_enable(struct ox03c10 *sensor, bool en)
{
	int ret;

	if (sensor->streaming)
		return -EBUSY;

	/*
	 * Vertical flipping will not keep the CFA pattern. Setting OX03C10_REG_WIN_09[1] has no
	 * effect. In fact, it can freeze the sensor.
	 */
	ret = regmap_update_bits(sensor->rmap, OX03C10_TIMING_CTRL_REG_20, BIT(2),
				 en ? BIT(2) : 0);

	return ret ? -EIO : 0;
}

static int ox03c10_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ox03c10 *sensor = container_of(ctrl->handler, struct ox03c10, ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_OX03C10_EXPOSURE:
		return ox03c10_exposure_set_gh(sensor, ctrl->p_new.p);

	case V4L2_CID_EXPOSURE:
		return ox03c10_exposure_and_gains_update(sensor, ctrl->val,
							 sensor->again_input,
							 sensor->dgain_input);

	case V4L2_CID_OX03C10_ANALOGUE_GAIN:
		return ox03c10_analogue_gain_set_gh(sensor, ctrl->p_new.p);

	case V4L2_CID_ANALOGUE_GAIN:
		return ox03c10_exposure_and_gains_update(sensor, sensor->exposure_input,
							 ctrl->val, sensor->dgain_input);

	case V4L2_CID_OX03C10_DIGITAL_GAIN:
		return ox03c10_digital_gain_set_gh(sensor, ctrl->p_new.p);

	case V4L2_CID_DIGITAL_GAIN:
		return ox03c10_exposure_and_gains_update(sensor, sensor->exposure_input,
							 sensor->again_input, ctrl->val);

	case V4L2_CID_OX03C10_WB_GAIN:
		return ox03c10_wb_gain_set_gh(sensor, ctrl->p_new.p);

	case V4L2_CID_OX03C10_PWL_EN:
		return ox03c10_pwl_enable(sensor, ctrl->val);

	case V4L2_CID_OX03C10_PWL_CTRL:
		return ox03c10_pwl_params_set(sensor, ctrl->p_new.p);

	case V4L2_CID_OX03C10_PWL_KNEE_POINTS_LUT:
		return ox03c10_pwl_lut_set(sensor, ctrl->p_new.p);

	case V4L2_CID_HFLIP:
		return ox03c10_hflip_enable(sensor, ctrl->val);

	case V4L2_CID_VFLIP:
		return ox03c10_vflip_enable(sensor, ctrl->val);

	default:
		return -EINVAL;
	}
}

static const struct v4l2_ctrl_ops ox03c10_ctrl_ops = {
	.s_ctrl		 = ox03c10_s_ctrl,
};

static void ox03c10_ctrl_type_op_log(const struct v4l2_ctrl *ctrl)
{
	/* no logging yet */
}

static struct ox03c10_exposure ox03c10_initial_exposure;
static struct ox03c10_analog_gain ox03c10_initial_analog_gain;
static struct ox03c10_digital_gain ox03c10_initial_digital_gain;
static struct ox03c10_wb_capture_gain ox03c10_initial_wb_capture_gain[4];
static struct ox03c10_pwl_ctrl ox03c10_initial_pwl_ctrl;
static struct ox03c10_otp_correction ox03c10_initial_otp;
static u8 ox03c10_initial_pwl_knee_points_lut[OX03C10_PWL_LUT_SIZE];

static int ox03c10_get_initial_params(struct ox03c10 *sensor)
{
	u16 wb_base_addr[4] = {
		OX03C10_AWB_GAIN_HCG_0,
		OX03C10_AWB_GAIN_LCG_0,
		OX03C10_AWB_GAIN_SPD_0,
		OX03C10_AWB_GAIN_VS_0,
	};
	int ret = 0;
	u8 buf[8];
	int i;

	/* get initial exposure */
	ret |= regmap_bulk_read(sensor->rmap, OX03C10_AEC_HCG_CTRL_01, buf, 2);
	ox03c10_initial_exposure.dcg = buf[0] << 8 | buf[1];
	ret |= regmap_bulk_read(sensor->rmap, OX03C10_AEC_SPD_CTRL_01, buf, 2);
	ox03c10_initial_exposure.spd = buf[0] << 8 | buf[1];
	ret |= regmap_bulk_read(sensor->rmap, OX03C10_AEC_VS_CTRL_01, buf, 2);
	ox03c10_initial_exposure.vs = buf[0] << 8 | buf[1];

	/* get initial analog gains */
	ret |= regmap_bulk_read(sensor->rmap, OX03C10_AEC_HCG_CTRL_08, buf, 2);
	ox03c10_initial_analog_gain.hcg = ((buf[0] & 0xf) << 4) | ((buf[1] & 0xf0) >> 4);
	ret |= regmap_bulk_read(sensor->rmap, OX03C10_AEC_SPD_CTRL_08, buf, 2);
	ox03c10_initial_analog_gain.spd = ((buf[0] & 0xf) << 4) | ((buf[1] & 0xf0) >> 4);
	ret |= regmap_bulk_read(sensor->rmap, OX03C10_AEC_LCG_CTRL_08, buf, 2);
	ox03c10_initial_analog_gain.lcg = ((buf[0] & 0xf) << 4) | ((buf[1] & 0xf0) >> 4);
	ret |= regmap_bulk_read(sensor->rmap, OX03C10_AEC_VS_CTRL_08, buf, 2);
	ox03c10_initial_analog_gain.vs = ((buf[0] & 0xf) << 4) | ((buf[1] & 0xf0) >> 4);

	/* get initial digital gains */
	ret |= regmap_bulk_read(sensor->rmap, OX03C10_AEC_HCG_CTRL_0A, buf, 3);
	ox03c10_initial_digital_gain.hcg = ((buf[0] & 0xf) << 10) | (buf[1] << 2) |
					   ((buf[2] & 0x3) >> 6);
	ret |= regmap_bulk_read(sensor->rmap, OX03C10_AEC_SPD_CTRL_0A, buf, 3);
	ox03c10_initial_digital_gain.spd = ((buf[0] & 0xf) << 10) | (buf[1] << 2) |
					   ((buf[2] & 0x3) >> 6);
	ret |= regmap_bulk_read(sensor->rmap, OX03C10_AEC_LCG_CTRL_0A, buf, 3);
	ox03c10_initial_digital_gain.lcg = ((buf[0] & 0xf) << 10) | (buf[1] << 2) |
					   ((buf[2] & 0x3) >> 6);
	ret |= regmap_bulk_read(sensor->rmap, OX03C10_AEC_VS_CTRL_0A, buf, 3);
	ox03c10_initial_digital_gain.vs = ((buf[0] & 0xf) << 10) | (buf[1] << 2) |
					  ((buf[2] & 0x3) >> 6);

	/* get initial white balance settings */
	for (i = 0; i < 4; i++) {
		ret |= regmap_bulk_read(sensor->rmap, wb_base_addr[i], buf, 8);
		ox03c10_initial_wb_capture_gain[i].b  = (buf[0] << 8) | buf[1];
		ox03c10_initial_wb_capture_gain[i].gb = (buf[2] << 8) | buf[3];
		ox03c10_initial_wb_capture_gain[i].gr = (buf[4] << 8) | buf[5];
		ox03c10_initial_wb_capture_gain[i].r  = (buf[6] << 8) | buf[7];
	}

	/* get initial PWL control params */
	ret |= regmap_bulk_read(sensor->rmap, OX03C10_FORMAT_REG_1F, buf, 1);
	ox03c10_initial_pwl_ctrl.pack24bit_sel = (buf[0] & 0xc0) >> 6;
	ox03c10_initial_pwl_ctrl.pwl_mode = (buf[0] & 0x18) >> 3;

	/* get initial PWL knee points LUT */
	ret |= regmap_bulk_read(sensor->rmap, OX03C10_PWL0_0_1,
				ox03c10_initial_pwl_knee_points_lut, OX03C10_PWL_LUT_SIZE);

	/* start streaming in order to retrieve OTP values */
	regmap_write(sensor->rmap, OX03C10_SMIA_R0100, 1);

	/*
	 * OTP values are updated after streaming is started but some sensors take longer to
	 * update their values. Wait a maximum of 100ms and keep retrying until the values are
	 * populated.
	 */
	for (i = 0; i < 10; i++) {
		ret = regmap_bulk_read(sensor->rmap, 0x7057, buf, 3);
		ox03c10_initial_otp.val1 = (buf[0] << 16) | (buf[1] << 8) | buf[2];
		ret |= regmap_bulk_read(sensor->rmap, 0x705b, buf, 3);
		ox03c10_initial_otp.val2 = (buf[0] << 16) | (buf[1] << 8) | buf[2];
		ret |= regmap_bulk_read(sensor->rmap, 0x705f, buf, 3);
		ox03c10_initial_otp.val3 = (buf[0] << 16) | (buf[1] << 8) | buf[2];

		if (ret || (ox03c10_initial_otp.val1 && ox03c10_initial_otp.val2 &&
			    ox03c10_initial_otp.val3))
			break;

		fsleep(10000);
	}

	if (i == 10 && !ox03c10_initial_otp.val1 && !ox03c10_initial_otp.val2 &&
	    !ox03c10_initial_otp.val3)
		dev_warn(sensor->dev, "OTP values not populated after 100ms...\n");

	/* stop the streaming */
	regmap_write(sensor->rmap, OX03C10_SMIA_R0100, 0);

	return ret ? -EIO : 0;
}

static void ox03c10_v4l2_ctrl_type_op_init(const struct v4l2_ctrl *ctrl, u32 from_idx,
					   union v4l2_ctrl_ptr ptr)
{
	u32 tot_elems = ctrl->elems;
	u32 elems = tot_elems - from_idx;

	if (from_idx >= elems)
		return;

	switch (ctrl->id) {
	case V4L2_CID_OX03C10_EXPOSURE:
		memcpy(ptr.p, &ox03c10_initial_exposure, sizeof(ox03c10_initial_exposure));
		break;

	case V4L2_CID_OX03C10_ANALOGUE_GAIN:
		memcpy(ptr.p, &ox03c10_initial_analog_gain, sizeof(ox03c10_initial_analog_gain));
		break;

	case V4L2_CID_OX03C10_DIGITAL_GAIN:
		memcpy(ptr.p, &ox03c10_initial_digital_gain, sizeof(ox03c10_initial_digital_gain));
		break;

	case V4L2_CID_OX03C10_WB_GAIN:
		memcpy(ptr.p, &ox03c10_initial_wb_capture_gain,
		       sizeof(ox03c10_initial_wb_capture_gain));
		break;

	case V4L2_CID_OX03C10_PWL_CTRL:
		memcpy(ptr.p, &ox03c10_initial_pwl_ctrl, sizeof(ox03c10_initial_pwl_ctrl));
		break;

	case V4L2_CID_OX03C10_PWL_KNEE_POINTS_LUT:
		memcpy(ptr.p, &ox03c10_initial_pwl_knee_points_lut,
		       sizeof(ox03c10_initial_pwl_knee_points_lut));
		break;

	case V4L2_CID_OX03C10_OTP_CORRECTION:
		memcpy(ptr.p, &ox03c10_initial_otp, sizeof(ox03c10_initial_otp));
		break;

	default:
		v4l2_ctrl_type_op_init(ctrl, from_idx, ptr);
		break;
	}
}

static const struct v4l2_ctrl_type_ops ox03c10_ctrl_type_ops = {
	.init		= ox03c10_v4l2_ctrl_type_op_init,
	.validate	= v4l2_ctrl_type_op_validate,
	.equal		= v4l2_ctrl_type_op_equal,
	.log		= ox03c10_ctrl_type_op_log,
};

static const struct v4l2_ctrl_config ox03c10_ctrl_cfgs[] = {
	[OX03C10_EXPOSURE] = {
		.ops		= &ox03c10_ctrl_ops,
		.type_ops	= &ox03c10_ctrl_type_ops,
		.id		= V4L2_CID_OX03C10_EXPOSURE,
		.name		= "Exposure for: DCG, SPD, VS",
		.type		= V4L2_CTRL_TYPE_U8,
		.min		= 0x00,
		.max		= 0xff,
		.step		= 1,
		.def		= 0,
		.dims		= { sizeof(struct ox03c10_exposure) },
	},
	[OX03C10_AGAIN] = {
		.ops		= &ox03c10_ctrl_ops,
		.type_ops	= &ox03c10_ctrl_type_ops,
		.id		= V4L2_CID_OX03C10_ANALOGUE_GAIN,
		.name		= "Analog gains for: HCG, LCG, SPD, VS",
		.type		= V4L2_CTRL_TYPE_U8,
		.min		= 0x00,
		.max		= 0xff,
		.step		= 1,
		.def		= 0x0,
		.dims		= { sizeof(struct ox03c10_analog_gain) },
	},
	[OX03C10_DGAIN] = {
		.ops		= &ox03c10_ctrl_ops,
		.type_ops	= &ox03c10_ctrl_type_ops,
		.id		= V4L2_CID_OX03C10_DIGITAL_GAIN,
		.name		= "Digital gains for: HCG, LCG, SPD, VS",
		.type		= V4L2_CTRL_TYPE_U8,
		.min		= 0x00,
		.max		= 0xff,
		.step		= 1,
		.def		= 0x00,
		.dims		= { sizeof(struct ox03c10_digital_gain) },
	},
	[OX03C10_WBGAIN] = {
		.ops		= &ox03c10_ctrl_ops,
		.type_ops	= &ox03c10_ctrl_type_ops,
		.id		= V4L2_CID_OX03C10_WB_GAIN,
		.name		= "White balance gain for: HCG, LCG, SPD, VS",
		.type		= V4L2_CTRL_TYPE_U8,
		.min		= 0x00,
		.max		= 0xff,
		.step		= 1,
		.def		= 0x00,
		.dims		= { 4 * sizeof(struct ox03c10_wb_capture_gain) },
	},
	[OX03C10_PWL_EN] = {
		.ops		= &ox03c10_ctrl_ops,
		.id		= V4L2_CID_OX03C10_PWL_EN,
		.name		= "Enable PWL compression",
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.min		= false,
		.max		= true,
		.step		= 1,
		.def		= true,
	},
	[OX03C10_PWL_CTRL] = {
		.ops		= &ox03c10_ctrl_ops,
		.type_ops	= &ox03c10_ctrl_type_ops,
		.id		= V4L2_CID_OX03C10_PWL_CTRL,
		.name		= "PWL compression control params",
		.type		= V4L2_CTRL_TYPE_U8,
		.min		= 0x00,
		.max		= 0xff,
		.step		= 1,
		.def		= 0,
		.dims		= { sizeof(struct ox03c10_pwl_ctrl) }
	},
	[OX03C10_PWL_KNEE_POINTS_LUT] = {
		.ops		= &ox03c10_ctrl_ops,
		.type_ops	= &ox03c10_ctrl_type_ops,
		.id		= V4L2_CID_OX03C10_PWL_KNEE_POINTS_LUT,
		.name		= "PWL knee points LUT",
		.type		= V4L2_CTRL_TYPE_U8,
		.min		= 0x00,
		.max		= 0xff,
		.step		= 1,
		.def		= 0,
		.dims		= { 132 }
	},
	[OX03C10_OTP_CORRECTION] = {
		.ops		= &ox03c10_ctrl_ops,
		.type_ops	= &ox03c10_ctrl_type_ops,
		.id		= V4L2_CID_OX03C10_OTP_CORRECTION,
		.name		= "OTP correction values",
		.type		= V4L2_CTRL_TYPE_U8,
		.min		= 0x00,
		.max		= 0xff,
		.step		= 1,
		.def		= 0,
		.dims		= { sizeof(struct ox03c10_otp_correction) },
		.flags		= V4L2_CTRL_FLAG_READ_ONLY,
	},
};

int ox03c10_v4l2_controls_init(struct ox03c10 *sensor)
{
	struct device *dev = &sensor->client->dev;
	struct v4l2_ctrl_handler *ctrl_handler = &sensor->ctrl_handler;
	struct v4l2_fwnode_device_properties props;
	u32 exposure_max;
	int i;
	u16 hblank, vblank;
	int ret;

	ret = v4l2_ctrl_handler_init(&sensor->ctrl_handler, ARRAY_SIZE(ox03c10_ctrl_cfgs) + 9);
	if (ret < 0) {
		dev_err(dev, "Cannot initialize V4L2 ctrl handler.\n");
		return ret;
	}

	exposure_max = sensor->cur_mode->vts - 24;
	v4l2_ctrl_new_std(ctrl_handler, &ox03c10_ctrl_ops, V4L2_CID_EXPOSURE, OX03C10_EXPOSURE_MIN,
			  exposure_max, 1, OX03C10_EXPOSURE_MIN);
	sensor->exposure_input = OX03C10_EXPOSURE_MIN;

	v4l2_ctrl_new_std(ctrl_handler, &ox03c10_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  OX03C10_CTRL_AGAIN_MIN, OX03C10_CTRL_AGAIN_MAX, 1,
			  OX03C10_CTRL_AGAIN_MIN);
	sensor->again_input = OX03C10_AGAIN_MIN;

	v4l2_ctrl_new_std(ctrl_handler, &ox03c10_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  OX03C10_DGAIN_MIN, OX03C10_DGAIN_MAX, 1,
			  OX03C10_DGAIN_MIN);
	sensor->dgain_input = OX03C10_DGAIN_MIN;

	v4l2_ctrl_new_std(ctrl_handler, &ox03c10_ctrl_ops, V4L2_CID_PIXEL_RATE,
			  OX03C10_PIXEL_RATE, OX03C10_PIXEL_RATE, 1, OX03C10_PIXEL_RATE);

	hblank = sensor->cur_mode->hts - sensor->cur_mode->width;

	sensor->hblank = v4l2_ctrl_new_std(ctrl_handler, &ox03c10_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank, 1,
					   hblank);

	vblank = sensor->cur_mode->vts - sensor->cur_mode->height;

	sensor->vblank = v4l2_ctrl_new_std(ctrl_handler, &ox03c10_ctrl_ops,
					   V4L2_CID_VBLANK, vblank, vblank, 1,
					   vblank);

	v4l2_ctrl_new_std(ctrl_handler, &ox03c10_ctrl_ops, V4L2_CID_AUTO_WHITE_BALANCE, 0, 1, 1, 0);

	v4l2_ctrl_new_std(ctrl_handler, &ox03c10_ctrl_ops, V4L2_CID_HFLIP, 0, 1, 1, 0);

	sensor->vflip = v4l2_ctrl_new_std(ctrl_handler, &ox03c10_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (sensor->vflip)
		sensor->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	ret = v4l2_fwnode_device_parse(sensor->dev, &props);
	if (ret)
		goto free_ctrls;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_handler, &ox03c10_ctrl_ops, &props);
	if (ret)
		goto free_ctrls;

	for (i = 0; i < ARRAY_SIZE(ox03c10_ctrl_cfgs); i++) {
		sensor->ctrls[i] = v4l2_ctrl_new_custom(ctrl_handler, &ox03c10_ctrl_cfgs[i], NULL);
		if (ctrl_handler->error) {
			dev_err(sensor->dev, "Adding control (%d) failed: %d\n",
				i, ctrl_handler->error);
			ret = ctrl_handler->error;
			goto free_ctrls;
		}
	}

	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(ctrl_handler);
	return ret;
}
EXPORT_SYMBOL(ox03c10_v4l2_controls_init);

int ox03c10_streaming_start(struct ox03c10 *sensor, bool start)
{
	int ret;

	if (!start) {
		/*
		 * For stopping, we need to use group hold registers in order to be able to
		 * stop during vertical blanking to avoid MIPI issues.
		 */
		ret = regmap_write(sensor->rmap, OX03C10_GRP_HOLD_8, 0x00);
		ret |= regmap_write(sensor->rmap, OX03C10_SMIA_R0100, 0);
		ret |= regmap_write(sensor->rmap, OX03C10_GRP_HOLD_8, 0x10);

		ret |= regmap_write(sensor->rmap, OX03C10_GRP_HOLD_8, 0xA0);

		/* Wait a maximum of 1 time frame. Worst case is 33.33ms. */
		msleep(34);

		/*
		 * OX03C10 messes up the frames if the VS exposure is higher than 4 before streaming
		 * is started. The following is working around this issue by lowering the VS to 4
		 * if the value was set higher during streaming.
		 */
		if (sensor->exposure.vs > 4) {
			struct ox03c10_exposure new_exposure = sensor->exposure;

			new_exposure.vs = 4;
			__v4l2_ctrl_s_ctrl_compound(sensor->ctrls[OX03C10_EXPOSURE],
						    V4L2_CTRL_TYPE_U8,
						    &new_exposure);
		}
	} else {
		ret = regmap_write(sensor->rmap, OX03C10_SMIA_R0100, 1);
	}

	sensor->streaming = start;

	return ret ? -EIO : 0;
}
EXPORT_SYMBOL(ox03c10_streaming_start);

int ox03c10_set_mode(struct ox03c10 *sensor, const struct ox03c10_mode *mode)
{
	int ret;
	u8 buf[4];

	buf[0] = (mode->crop.left >> 8) & 0xff;
	buf[1] = mode->crop.left & 0xff;
	buf[2] = (mode->crop.top >> 8) & 0xff;
	buf[3] = mode->crop.top & 0xff;

	ret = regmap_bulk_write(sensor->rmap, OX03C10_ISP_X_WIN_INT_H, buf, 4);
	if (ret)
		return ret;

	buf[0] = (mode->crop.width >> 8) & 0xff;
	buf[1] = mode->crop.width & 0xff;
	buf[2] = (mode->crop.height >> 8) & 0xff;
	buf[3] = mode->crop.height & 0xff;

	ret = regmap_bulk_write(sensor->rmap, OX03C10_X_OUTPUT_SIZE_INT_H, buf, 4);
	if (ret)
		return ret;

	sensor->cur_mode = mode;

	return 0;
}
EXPORT_SYMBOL(ox03c10_set_mode);

static int ox03c10_sensor_init(struct ox03c10 *sensor)
{
	const struct ox03c10_reg *reg;
	int ret = 0;
	int i;

	/* software reset */
	regmap_write(sensor->rmap, OX03C10_SMIA_R0103, 1);
	regmap_write(sensor->rmap, OX03C10_SMIA_R0107, 1);

	usleep_range(100, 200);

	for (i = 0; i < ARRAY_SIZE(ox03c10_init_data); i++) {
		reg = &ox03c10_init_data[i];

		/*
		 * Re-enable the cache after the embedded data registers ranges have been set.
		 */
		if (reg->addr == OX03C10_GRP_HOLD_8 && (reg->val == 0x14 || reg->val == 0x15))
			regcache_cache_bypass(sensor->rmap, false);

		ret = regmap_write(sensor->rmap, reg->addr, reg->val);
		if (ret < 0) {
			dev_err(&sensor->client->dev, "Failed to write addr 0x%04x with 0x%02x\n",
				reg->addr, reg->val);
			return ret;
		}

		/*
		 * Make sure we bypass the cache when setting address ranges for embedded data.
		 * Otherwise, our cache will hold a range instead of the actual value...
		 */
		if (reg->addr == OX03C10_GRP_HOLD_8 && (reg->val == 0x04 || reg->val == 0x05))
			regcache_cache_bypass(sensor->rmap, true);
	}

	sensor->cur_mode = &ox03c10_modes[0];

	return ox03c10_get_initial_params(sensor);
}

struct ox03c10 *ox03c10_init_with_dummy_client(struct i2c_client *client,
					       bool use_dummy)
{
	struct device *dev = &client->dev;
	struct ox03c10 *sensor;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor) +
				ARRAY_SIZE(ox03c10_ctrl_cfgs) * sizeof(struct v4l2_ctrl *),
				GFP_KERNEL);
	if (!sensor)
		return ERR_PTR(-ENOMEM);

	sensor->dev = dev;

	if (use_dummy) {
		sensor->client =
			devm_i2c_new_dummy_device(dev, client->adapter,
						  OX03C10_I2C_ADDR);
		if (IS_ERR(sensor->client))
			return ERR_PTR(-ENODEV);
	} else {
		sensor->client = client;
	}

	sensor->rmap = devm_regmap_init_i2c(sensor->client, &ox03c10_sensor_regmap_cfg);
	if (IS_ERR(sensor->rmap)) {
		ret = PTR_ERR(sensor->rmap);
		dev_err(dev, "Failed to allocate sensor register map: %d\n", ret);
		goto error;
	}

	ret = ox03c10_sensor_init(sensor);
	if (ret)
		goto error;

	return sensor;

error:
	return ERR_PTR(ret);
}
EXPORT_SYMBOL(ox03c10_init_with_dummy_client);

struct v4l2_ctrl_handler *ox03c10_ctrl_handler_get(struct ox03c10 *sensor)
{
	return &sensor->ctrl_handler;
}
EXPORT_SYMBOL(ox03c10_ctrl_handler_get);

void ox03c10_ctrl_handler_free(struct ox03c10 *sensor)
{
	v4l2_ctrl_handler_free(&sensor->ctrl_handler);
}
EXPORT_SYMBOL(ox03c10_ctrl_handler_free);

struct ox03c10_mode *ox03c10_get_mode(int index)
{
	if (index >= ARRAY_SIZE(ox03c10_modes))
		return ERR_PTR(-EINVAL);

	return &ox03c10_modes[index];
}
EXPORT_SYMBOL(ox03c10_get_mode);

MODULE_DESCRIPTION("Omnivision OX03C10 sensor library");
MODULE_AUTHOR("Laurentiu Palcu");
MODULE_LICENSE("GPL");
