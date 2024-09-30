/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2021-2024 NXP
 *
 * Header file for the ELE Base API(s).
 */

#ifndef ELE_BASE_MSG_H
#define ELE_BASE_MSG_H

#include <linux/types.h>

#define WORD_SZ				4
#define ELE_NONE_VAL			0x0

#define ELE_SUCCESS_IND			0xD6

#define ELE_GET_INFO_REQ		0xDA
#define ELE_GET_INFO_REQ_MSG_SZ		0x10
#define ELE_GET_INFO_RSP_MSG_SZ		0x08

#define ELE_GET_INFO_BUFF_SZ		0x100
#define ELE_GET_INFO_READ_SZ		0xA0
#define DEVICE_GET_INFO_SZ		0x100

#define GET_INFO_SOC_INFO_WORD_OFFSET	1
#define GET_INFO_UUID_WORD_OFFSET	3
#define GET_INFO_SL_NUM_MSB_WORD_OFF \
	(GET_INFO_UUID_WORD_OFFSET + 3)
#define GET_INFO_SL_NUM_LSB_WORD_OFF \
	(GET_INFO_UUID_WORD_OFFSET + 0)

#define ELE_PING_REQ			0x01
#define ELE_PING_REQ_SZ			0x04
#define ELE_PING_RSP_SZ			0x08

#define ELE_START_RNG_REQ		0xA3
#define ELE_START_RNG_REQ_MSG_SZ	0x04
#define ELE_START_RNG_RSP_MSG_SZ	0x08

#define ELE_GET_TRNG_STATE_REQ		0xA4
#define ELE_GET_TRNG_STATE_REQ_MSG_SZ	0x04
#define ELE_GET_TRNG_STATE_RSP_MSG_SZ	0x0C
#define ELE_TRNG_STATE_OK		0x203
#define ELE_GET_TRNG_STATE_RETRY_COUNT	0x5
#define CSAL_TRNG_STATE_MASK		0x0000ffff

#define ELE_SERVICE_SWAP_REQ		0xDF
#define ELE_SERVICE_SWAP_REQ_MSG_SZ	0x18
#define ELE_SERVICE_SWAP_RSP_MSG_SZ	0x0C
#define ELE_IMEM_SIZE			0x10000
#define ELE_IMEM_STATE_OK		0xCA
#define ELE_IMEM_STATE_BAD		0xFE
#define ELE_IMEM_STATE_WORD		0x27
#define ELE_IMEM_STATE_MASK		0x00ff0000
#define ELE_IMEM_EXPORT			0x1
#define ELE_IMEM_IMPORT			0x2

#define ELE_FW_AUTH_REQ			0x02
#define ELE_FW_AUTH_REQ_SZ		0x10
#define ELE_FW_AUTH_RSP_MSG_SZ		0x08

#define ELE_READ_FUSE_REQ		0x97
#define ELE_READ_FUSE_REQ_MSG_SZ	0x08
#define ELE_READ_FUSE_RSP_MSG_SZ	0x0C
#define ELE_READ_FUSE_OTP_UNQ_ID_RSP_MSG_SZ \
					0x1C
#define OTP_UNIQ_ID			0x01
#define OTFAD_CONFIG			0x2

#define ELE_GET_STATE			0xB2
#define ELE_GET_STATE_REQ_SZ		0x04
#define ELE_GET_STATE_RSP_SZ		0x10

#define ELE_WRITE_FUSE			0xD6
#define ELE_WRITE_FUSE_REQ_MSG_SZ	12
#define ELE_WRITE_FUSE_RSP_MSG_SZ	12

#define V2X_FW_STATE_UNKNOWN		0x00
#define V2X_FW_STATE_RUNNING		0x15

#define ELE_VOLT_CHANGE_START_REQ			0x12
#define ELE_VOLT_CHANGE_FINISH_REQ			0x13
#define ELE_VOLT_CHANGE_REQ_MSG_SZ			0x4
#define ELE_VOLT_CHANGE_RSP_MSG_SZ			0x8

int ele_get_info(struct device *dev, phys_addr_t addr, u32 data_size);
int ele_ping(struct device *dev);
int ele_start_rng(struct device *dev);
int ele_get_trng_state(struct device *dev);
int ele_service_swap(struct device *dev,
		     phys_addr_t addr,
		     u32 addr_size, u16 flag);
int ele_get_v2x_fw_state(struct device *dev, uint32_t *state);
int ele_write_fuse(struct device *dev, uint16_t fuse_index, u32 value, bool block);
int ele_voltage_change_req(struct device *dev, bool start);

int read_common_fuse(struct device *dev,
		     uint16_t fuse_id, u32 *value);

int ele_fw_authenticate(struct device *dev, phys_addr_t addr);
#endif
