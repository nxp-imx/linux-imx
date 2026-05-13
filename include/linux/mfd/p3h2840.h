/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2025-2026 NXP
 * This header file contain private Reg address and its bit mapping etc.
 */

#ifndef _LINUX_MFD_P3H2840_H
#define _LINUX_MFD_P3H2840_H

#include <linux/types.h>

/* Device Configuration Registers */
#define P3H2X4X_DEV_REG_PROTECTION_CODE				0x10
#define P3H2X4X_REGISTERS_LOCK_CODE				0x00
#define P3H2X4X_REGISTERS_UNLOCK_CODE				0x69
#define P3H2X4X_CP1_REGISTERS_UNLOCK_CODE			0x6a

/* Reg config for Regmap */
#define P3H2X4X_REG_BITS					8
#define P3H2X4X_VAL_BITS					8

struct p3h2x4x_dev {
	struct i3c_device *i3cdev;
	struct regmap *regmap;
	bool is_p3h2x4x_in_i3c;
};
#endif /* _LINUX_MFD_P3H2840_H */
