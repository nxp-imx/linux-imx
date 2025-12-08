// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Wave5 series multi-standard codec IP - low level access functions
 *
 * Copyright (C) 2021-2023 CHIPS&MEDIA INC
 */

#include <linux/bug.h>
#include "wave5-vdi.h"
#include "wave5-vpu.h"
#include "wave5-regdefine.h"
#include <linux/delay.h>

void wave5_vdi_write_register(struct vpu_device *vpu_dev, u32 addr, u32 data)
{
	writel(data, vpu_dev->vdb_register + addr);
}
EXPORT_SYMBOL_GPL(wave5_vdi_write_register);

unsigned int wave5_vdi_read_register(struct vpu_device *vpu_dev, u32 addr)
{
	return readl(vpu_dev->vdb_register + addr);
}
EXPORT_SYMBOL_GPL(wave5_vdi_read_register);

int wave5_vdi_clear_memory(struct vpu_buf *vb)
{
	if (!vb)
		return -EINVAL;

	if (!vb->vaddr) {
		dev_err(vb->dev, "%s: unable to clear unmapped buffer\n", __func__);
		return -EINVAL;
	}

	memset(vb->vaddr, 0, vb->size);
	return vb->size;
}
EXPORT_SYMBOL_GPL(wave5_vdi_clear_memory);

int wave5_vdi_write_memory(struct vpu_buf *vb, size_t offset, u8 *data, size_t len)
{
	if (!vb)
		return -EINVAL;

	if (!vb->vaddr) {
		dev_err(vb->dev, "%s: unable to write to unmapped buffer\n", __func__);
		return -EINVAL;
	}

	if (offset > vb->size || len > vb->size || offset + len > vb->size) {
		dev_err(vb->dev, "%s: buffer too small\n", __func__);
		return -ENOSPC;
	}

	memcpy(vb->vaddr + offset, data, len);
	return len;
}
EXPORT_SYMBOL_GPL(wave5_vdi_write_memory);

int wave5_vdi_allocate_dma_memory(struct device *dev, struct vpu_buf *vb)
{
	void *vaddr;
	dma_addr_t daddr;

	if (!dev || !vb)
		return -EINVAL;

	if (!vb->size) {
		dev_err(dev, "%s: requested size==0\n", __func__);
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
EXPORT_SYMBOL_GPL(wave5_vdi_allocate_dma_memory);

void wave5_vdi_free_dma_memory(struct vpu_buf *vb)
{
	if (!vb || !vb->size)
		return;

	if (!vb->vaddr) {
		dev_err(vb->dev, "%s: requested free of unmapped buffer\n", __func__);
		return;
	}

	dma_free_coherent(vb->dev, vb->size, vb->vaddr, vb->daddr);
	memset(vb, 0, sizeof(*vb));
}
EXPORT_SYMBOL_GPL(wave5_vdi_free_dma_memory);

int wave5_vdi_allocate_array(struct device *dev, struct vpu_buf *array, unsigned int count,
			     size_t size)
{
	struct vpu_buf vb_buf;
	int i, ret = 0;

	vb_buf.size = size;

	for (i = 0; i < count; i++) {
		if (array[i].size == size)
			continue;

		if (array[i].size != 0)
			wave5_vdi_free_dma_memory(&array[i]);

		ret = wave5_vdi_allocate_dma_memory(dev, &vb_buf);
		if (ret)
			return -ENOMEM;
		array[i] = vb_buf;
	}

	for (i = count; i < MAX_REG_FRAME; i++)
		wave5_vdi_free_dma_memory(&array[i]);

	return 0;
}
EXPORT_SYMBOL_GPL(wave5_vdi_allocate_array);
