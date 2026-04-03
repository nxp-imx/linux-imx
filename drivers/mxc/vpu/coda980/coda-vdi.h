/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Coda series multi-standard codec IP - low level access functions
 *
 * Copyright (C) 2024-2026 CHIPS&MEDIA INC
 */

#ifndef __CODA_VDI_H__
#define __CODA_VDI_H__

#include <linux/string.h>
#include <linux/slab.h>
#include <linux/device.h>

/* system register write */
#define vpu_write_reg(DEV, ADDR, DATA) coda_vdi_writel(DEV, ADDR, DATA)
/* system register read */
#define vpu_read_reg(DEV, ADDR) coda_vdi_readl(DEV, ADDR)

struct vpu_buf {
	size_t size;
	dma_addr_t daddr;
	void *vaddr;
	phys_addr_t paddr;
	struct device *dev;
};

enum vpu_endian_mode {
	VDI_LITTLE_ENDIAN = 0, /* 64bit LE */
	VDI_BIG_ENDIAN, /* 64bit BE */
	VDI_32BIT_LITTLE_ENDIAN,
	VDI_32BIT_BIG_ENDIAN,
	VDI_ENDIAN_MAX
};

void coda_vdi_writel(struct device *dev, unsigned int addr, unsigned int data);
unsigned int coda_vdi_readl(struct device *dev, u32 addr);
void coda_vdi_free_dma_memory(struct vpu_buf *vb);
int coda_vdi_allocate_dma_memory(struct device *dev, struct vpu_buf *vb);
int coda_vdi_write_memory(struct vpu_buf *vb, size_t offset,
			  u8 *data, int len, int endian);
int coda_vdi_read_memory(struct vpu_buf *vb, size_t offset,
			 u8 *data, int len, int endian);

#endif
