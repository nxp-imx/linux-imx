/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Wave6 series multi-standard codec IP - low level access interface
 *
 * Copyright (C) 2025 CHIPS&MEDIA INC
 */

#ifndef __WAVE6_VDI_H__
#define __WAVE6_VDI_H__

#include <linux/string.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <uapi/linux/dma-heap.h>
#include "wave6-vpuconfig.h"

#define vpu_write_reg(VPU_DEV, ADDR, DATA) wave6_vdi_writel(VPU_DEV, ADDR, DATA)
#define vpu_read_reg(VPU_DEV, ADDR) wave6_vdi_readl(VPU_DEV, ADDR)

struct vpu_buf {
	size_t size;
	dma_addr_t daddr;
	void *vaddr;
	struct device *dev;
	struct dma_buf* secure_dma_buf;
	struct imx_mur_node *recorder;
	const char *label;
};

struct vpu_dma_buf {
	size_t size;
	dma_addr_t dma_addr;
	void *vaddr;
	phys_addr_t phys_addr;
};

enum endian_mode {
	VDI_LITTLE_ENDIAN = 0,
	VDI_BIG_ENDIAN,
	VDI_32BIT_LITTLE_ENDIAN,
	VDI_32BIT_BIG_ENDIAN,
	VDI_128BIT_LITTLE_ENDIAN = 16,
	VDI_128BIT_LE_BYTE_SWAP,
	VDI_128BIT_LE_WORD_SWAP,
	VDI_128BIT_LE_WORD_BYTE_SWAP,
	VDI_128BIT_LE_DWORD_SWAP,
	VDI_128BIT_LE_DWORD_BYTE_SWAP,
	VDI_128BIT_LE_DWORD_WORD_SWAP,
	VDI_128BIT_LE_DWORD_WORD_BYTE_SWAP,
	VDI_128BIT_BE_DWORD_WORD_BYTE_SWAP,
	VDI_128BIT_BE_DWORD_WORD_SWAP,
	VDI_128BIT_BE_DWORD_BYTE_SWAP,
	VDI_128BIT_BE_DWORD_SWAP,
	VDI_128BIT_BE_WORD_BYTE_SWAP,
	VDI_128BIT_BE_WORD_SWAP,
	VDI_128BIT_BE_BYTE_SWAP,
	VDI_128BIT_BIG_ENDIAN = 31,
	VDI_ENDIAN_MAX
};

#define VDI_128BIT_ENDIAN_MASK 0xf

#endif /* __WAVE6_VDI_H__ */
