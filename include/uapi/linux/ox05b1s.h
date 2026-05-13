/* SPDX-License-Identifier: ((GPL-2.0+ WITH Linux-syscall-note) OR BSD-3-Clause) */
/*
 * Copyright 2025 NXP
 */

#ifndef _OX05B1S_H_
#define _OX05B1S_H_

#include <linux/v4l2-controls.h>

/* id for long exposure for os08a20, context 0 for ox05b1s */
#define OX05B1S_EXP0 0

/* id for short exposure for os08a20, context 1 for ox05b1s */
#define OX05B1S_EXP1 1

#define OX05B1S_NUM_EXP 2

#define OX05B1S_EXP_MAX 0xFFFFFF

/* ox05b1s real gain = gaincode / 16, max 15.5x */
#define OX05B1S_AGAIN_MAX 0xF8
#define OX05B1S_AGAIN_1X 0x10

/* os08a20 real gain = gaincode / 8 / 16, max 15.5x */
#define OS08A20_AGAIN_MAX 0x7C0
#define OS08A20_AGAIN_1X 0x80

#define OX05B1S_DGAIN_MAX 0x3FFF

#endif
