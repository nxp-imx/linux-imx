/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2024-2025, NXP
 */

#ifndef OX05B1S_H
#define OX05B1S_H

#include <linux/regmap.h>
#include <linux/types.h>

struct ox05b1s_reglist {
	const struct cci_reg_sequence *regs;
	unsigned int count;
};

extern const struct ox05b1s_reglist os08a20_reglist_4k_10b[];
extern const struct ox05b1s_reglist os08a20_reglist_4k_12b[];
extern const struct ox05b1s_reglist os08a20_reglist_1080p_10b[];

extern const struct ox05b1s_reglist ox05b1s_reglist_2592x1944[];

#endif /* OX05B1S_H */
