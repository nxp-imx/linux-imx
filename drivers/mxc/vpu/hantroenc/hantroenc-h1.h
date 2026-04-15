/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2026 NXP
 * Copyright (c) 2015-2020, VeriSilicon Inc.
 * Copyright (c) 2011-2014, Google Inc.
 */

#ifndef __HANTROENC_H1_H
#define __HANTROENC_H1_H

#include "hantro-enc.h"

#define H1ENC_IOCGHWOFFSET	_IOR(HX280ENC_IOC_MAGIC,  3, __u32 *)
#define H1ENC_IOCGHWIOSIZE	_IOR(HX280ENC_IOC_MAGIC,  4, __u32 *)
#define H1ENC_IOCH_ENC_RESERVE	_IOR(HX280ENC_IOC_MAGIC, 11, __u32 *)
#define H1ENC_IOCH_ENC_RELEASE	_IOR(HX280ENC_IOC_MAGIC, 12, __u32 *)
#define H1ENC_IOCG_CORE_WAIT	_IOR(HX280ENC_IOC_MAGIC, 13, __u32 *)
#define H1ENC_IOC_WRITE_REGS	_IOW(HX280ENC_IOC_MAGIC, 14, struct hantro_enc_regs_buffer *)
#define H1ENC_IOC_READ_REGS	_IOR(HX280ENC_IOC_MAGIC, 15, struct hantro_enc_regs_buffer *)
#define H1ENC_IOCG_EN_CORE	_IO(HX280ENC_IOC_MAGIC, 16)

#define H1_WRITE1_CLEAR_MASK			BIT(23)

#endif
