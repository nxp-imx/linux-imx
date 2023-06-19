/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2021-2022 NXP
 */

#ifndef ELE_BASE_MSG_H
#define ELE_BASE_MSG_H

#include <linux/types.h>


#define MESSAGING_VERSION_6		0x6

#define ELE_PING_REQ			0x1
#define ELE_OEM_CNTN_AUTH_REQ		0x87
#define ELE_VERIFY_IMAGE_REQ		0x88
#define ELE_RELEASE_CONTAINER_REQ	0x89
#define ELE_READ_FUSE_REQ		0x97
#define OTP_UNIQ_ID			0x01
#define OTFAD_CONFIG			0x2
#define ELE_GET_INFO_REQ                0xDA
#define ELE_SERVICE_SWAP_REQ		0xDF
#define GET_INFO_DATA                   0x17
#define ELE_START_RNG_REQ		0xA3
#define ELE_GET_TRNG_STATE_REQ		0xA4
#define CSAL_TRNG_STATE_MASK		0x0000ffff

#define ELE_VERSION			0x6

#define ELE_OEM_CNTN_AUTH_REQ_SIZE	3
#define ELE_VERIFY_IMAGE_REQ_SIZE	2
#define ELE_REL_CONTAINER_REQ_SIZE	1
#define ELE_IMEM_EXPORT			0x1
#define ELE_IMEM_IMPORT			0x2

int read_common_fuse(uint16_t fuse_index, u32 *value);
int ele_ping(void);
int ele_get_info(phys_addr_t addr, u32 data_size);
int ele_service_swap(phys_addr_t addr, u32 addr_size, u16 flag);
int ele_start_rng(void);
int ele_get_trng_state(void);

#endif
