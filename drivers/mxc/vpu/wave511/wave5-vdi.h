/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Wave5 series multi-standard codec IP - low level access functions
 *
 * Copyright (C) 2021-2023 CHIPS&MEDIA INC
 */

#ifndef _VDI_H_
#define _VDI_H_

#include "wave5-vpuconfig.h"
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/device.h>

/* system register write */
#define vpu_write_reg(VPU_INST, ADDR, DATA) wave5_vdi_write_register(VPU_INST, ADDR, DATA)
/* system register read */
#define vpu_read_reg(CORE, ADDR) wave5_vdi_read_register(CORE, ADDR)

struct vpu_buf {
	size_t size;
	dma_addr_t daddr;
	void *vaddr;
	struct device *dev;
};

enum endian_mode {
	VDI_LITTLE_ENDIAN = 0, /* 64bit LE */
	VDI_BIG_ENDIAN, /* 64bit BE */
	VDI_32BIT_LITTLE_ENDIAN,
	VDI_32BIT_BIG_ENDIAN,
	/* WAVE PRODUCTS */
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

int wave5_vdi_allocate_dma_memory(struct device *dev, struct vpu_buf *vb);
int wave5_vdi_allocate_array(struct device *dev, struct vpu_buf *array, unsigned int count,
			     size_t size);
void wave5_vdi_free_dma_memory(struct vpu_buf *vb);
int wave5_vdi_clear_memory(struct vpu_buf *vb);
int wave5_vdi_write_memory(struct vpu_buf *vb, size_t offset, u8 *data, size_t len);

#endif //#ifndef _VDI_H_
