/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2026 NXP
 * Copyright (c) 2015-2020, VeriSilicon Inc.
 * Copyright (c) 2011-2014, Google Inc.
 */

#ifndef __HANTROENC_H
#define __HANTROENC_H

#include <linux/types.h>

#define HX280ENC_IOC_MAGIC	'k'
#define HX280ENC_IOC_MAXNR	30

struct hantro_enc_regs_buffer {
	u32 core_id;
	u32 *regs;
	u32 offset;
	u32 size;
	u32 *reserved;
};

#define HANTROENC_INT_STATUS_ENA			BIT(0)
#define HANTROENC_INT_STATUS_DIS			BIT(1)
#define HANTROENC_INT_STATUS_FRAME_DONE			BIT(2)
#define HANTROENC_INT_STATUS_BUS_ERROR			BIT(3)
#define HANTROENC_INT_STATUS_SW_RESET			BIT(4)
#define HANTROENC_INT_STATUS_BUF_FULL			BIT(5)
#define HANTROENC_INT_STATUS_TIMEOUT			BIT(6)
#define HANTROENC_INT_STATUS_LINE			BIT(7)
#define HANTROENC_INT_STATUS_SLICE_DONE			BIT(8)
#define HANTROENC_INT_STATUS_FUSE_ERROR			BIT(9)

#define HANTROENC_INT_STATUS_TIMEOUT_ENA		BIT(11)

#endif
