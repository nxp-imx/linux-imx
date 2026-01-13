// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Coda series multi-standard codec IP - low level access functions
 *
 * Copyright (C) 2024-2026 CHIPS&MEDIA INC
 */

#include <linux/bug.h>
#include <linux/delay.h>
#include "coda-vpu.h"
#include "coda-vdi.h"
#include "coda-regdefine.h"

static int coda_vdi_convert_endian(unsigned int endian)
{
	u32 converted_endian = endian;

	switch (endian) {
	case VDI_LITTLE_ENDIAN:
		converted_endian = 0;
		break;
	case VDI_BIG_ENDIAN:
		converted_endian = 7;
		break;
	case VDI_32BIT_LITTLE_ENDIAN:
		converted_endian = 4;
		break;
	case VDI_32BIT_BIG_ENDIAN:
		converted_endian = 3;
		break;
	}

	return converted_endian;
}

static void coda_vdi_byte_swap(unsigned char *data, int len)
{
	u8 temp;
	int i;

	for (i = 0; i < len; i += 2) {
		temp = data[i];
		data[i] = data[i + 1];
		data[i + 1] = temp;
	}
}

static void coda_vdi_word_swap(unsigned char *data, int len)
{
	u16 temp;
	u16 *ptr = (u16 *)data;
	int i;
	s32 size = len / sizeof(u16);

	for (i = 0; i < size; i += 2) {
		temp = ptr[i];
		ptr[i] = ptr[i + 1];
		ptr[i + 1] = temp;
	}
}

static void coda_vdi_dword_swap(unsigned char *data, int len)
{
	u32 temp;
	u32 *ptr = (u32 *)data;
	s32 size = len / sizeof(u32);
	int i;

	for (i = 0; i < size; i += 2) {
		temp = ptr[i];
		ptr[i] = ptr[i + 1];
		ptr[i + 1] = temp;
	}
}

static void coda_vdi_lword_swap(unsigned char *data, int len)
{
	u64 temp;
	u64 *ptr = (u64 *)data;
	s32 size = len / sizeof(uint64_t);
	int i;

	for (i = 0; i < size; i += 2) {
		temp = ptr[i];
		ptr[i] = ptr[i + 1];
		ptr[i + 1] = temp;
	}
}

static int coda_vdi_swap_endian(unsigned char *data, int len, int endian)
{
	int changes;
	int sys_endian;
	bool byte_change, word_change, dword_change, lword_change;

	sys_endian = VDI_LITTLE_ENDIAN;

	endian = coda_vdi_convert_endian(endian);
	sys_endian = coda_vdi_convert_endian(sys_endian);
	if (endian == sys_endian)
		return 0;

	changes = endian ^ sys_endian;
	byte_change = changes & 0x01;
	word_change = ((changes & 0x02) == 0x02);
	dword_change = ((changes & 0x04) == 0x04);
	lword_change = ((changes & 0x08) == 0x08);

	if (byte_change)
		coda_vdi_byte_swap(data, len);
	if (word_change)
		coda_vdi_word_swap(data, len);
	if (dword_change)
		coda_vdi_dword_swap(data, len);
	if (lword_change)
		coda_vdi_lword_swap(data, len);

	return 1;
}

int coda_vdi_read_memory(struct vpu_buf *vb, size_t offset,
			 u8 *data, int len, int endian)
{
	if (!vb)
		return -EINVAL;

	if (!vb->vaddr) {
		dev_err(vb->dev, "%s(): unable to write to unmapped buffer\n", __func__);
		return -EINVAL;
	}

	if (offset > vb->size || len > vb->size || offset + len > vb->size) {
		dev_err(vb->dev, "%s(): buffer too small\n", __func__);
		return -ENOSPC;
	}

	memcpy(data, vb->vaddr + offset, len);
	coda_vdi_swap_endian(data, len, endian);
	return len;
}

int coda_vdi_write_memory(struct vpu_buf *vb, size_t offset,
			  u8 *data, int len, int endian)
{
	if (!vb)
		return -EINVAL;

	if (!vb->vaddr) {
		dev_err(vb->dev, "%s(): unable to write to unmapped buffer\n", __func__);
		return -EINVAL;
	}

	if (offset > vb->size || len > vb->size || offset + len > vb->size) {
		dev_err(vb->dev, "%s(): buffer too small\n", __func__);
		return -ENOSPC;
	}

	coda_vdi_swap_endian(data, len, endian);
	memcpy(vb->vaddr + offset, data, len);
	return len;
}

int coda_vdi_allocate_dma_memory(struct device *dev, struct vpu_buf *vb)
{
	void *vaddr;
	dma_addr_t daddr;

	if (!vb || !vb->size) {
		dev_err(dev, "%s(): requested size==0\n", __func__);
		return -EINVAL;
	}
	vaddr = dma_alloc_coherent(dev, vb->size, &daddr, GFP_KERNEL);
	if (!vaddr)
		return -ENOMEM;
	vb->vaddr = vaddr;
	vb->daddr = daddr;
	vb->dev = dev;

	return 0;
}

void coda_vdi_free_dma_memory(struct vpu_buf *vb)
{
	if (!vb || !vb->size || !vb->vaddr)
		return;

	dma_free_coherent(vb->dev, vb->size, vb->vaddr, vb->daddr);
	memset(vb, 0, sizeof(*vb));
}

void coda_vdi_writel(struct device *dev, unsigned int addr, unsigned int data)
{
	struct vpu_device *vpu = dev_get_drvdata(dev);

	writel(data, vpu->reg_base + addr);
}

unsigned int coda_vdi_readl(struct device *dev, u32 addr)
{
	struct vpu_device *vpu = dev_get_drvdata(dev);

	return readl(vpu->reg_base + addr);
}
