// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Wave5 series multi-standard codec IP - low level access functions
 *
 * Copyright (C) 2026 CHIPS&MEDIA INC
 */

#include <linux/bug.h>
#include "wave5-vdi.h"
#include "wave5-vpu.h"
#include "wave5-regdefine.h"
#include <linux/delay.h>
#include <linux/trusty/smcall.h>
#include <linux/trusty/trusty.h>

#define SMC_ENTITY_IMX_WAVE_LINUX_OPT 55
#define SMC_IMX_VPU_REG SMC_FASTCALL_NR(SMC_ENTITY_IMX_WAVE_LINUX_OPT, 1)
#define OPT_WRITE 0x1
#define OPT_READ  0x2

#ifdef writel
#undef writel
#define writel(val, addr) \
	do { \
		if (vpu_dev->trusty_dev) { \
			trusty_vpu_set_reg(vpu_dev->trusty_dev, (addr - vpu_dev->vdb_register), val, vpu_dev->vpu_id); \
		} else { \
			{ __iowmb(); writel_relaxed((val),(addr)); } \
		} \
	} while (0)
#endif

static void trusty_vpu_set_reg(struct device *dev, u32 target, u32 val, u8 vpu_id) {
	trusty_fast_call32(dev, SMC_IMX_VPU_REG, target, OPT_WRITE | (vpu_id << 16), val);
}

static int trusty_vpu_get_reg(struct device *dev, u32 target, u8 vpu_id) {
	return trusty_fast_call32(dev, SMC_IMX_VPU_REG, target, OPT_READ | (vpu_id << 16), 0);
}

void wave5_vdi_write_register(struct vpu_device *vpu_dev, u32 addr, u32 data)
{
	writel(data, vpu_dev->vdb_register + addr);
}
EXPORT_SYMBOL_GPL(wave5_vdi_write_register);

unsigned int wave5_vdi_read_register(struct vpu_device *vpu_dev, u32 addr)
{
	if (vpu_dev->trusty_dev)
		return trusty_vpu_get_reg(vpu_dev->trusty_dev, addr, vpu_dev->vpu_id);
	else
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

	if (vb->recorder) {
		if (vb->label)
			imx_mur_long_new_and_add(vb->recorder, vb->size, vb->label);
		else
			imx_mur_long_add(vb->recorder, vb->size);
	}

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

	if (vb->recorder) {
		if (vb->label)
			imx_mur_long_sub_and_del_by_name(vb->recorder, vb->size, vb->label);
		else
			imx_mur_long_sub(vb->recorder, vb->size);
	}

	dma_free_coherent(vb->dev, vb->size, vb->vaddr, vb->daddr);
	memset(vb, 0, sizeof(*vb));
}
EXPORT_SYMBOL_GPL(wave5_vdi_free_dma_memory);

int wave5_vdi_allocate_array(struct device *dev, struct vpu_buf *array, unsigned int count,
			     size_t size)
{
	struct vpu_buf vb_buf = { 0 };
	int i, ret = 0;
	struct vpu_device *vpu_dev = NULL;

	vpu_dev = dev_get_drvdata(dev);
	vb_buf.size = size;

	for (i = 0; i < count; i++) {
		if (array[i].size == size)
			continue;

		if (array[i].size != 0) {
			if (vpu_dev->secure_mode)
				wave5_free_secure_dma_memory(&array[i]);
			else
				wave5_vdi_free_dma_memory(&array[i]);
		}

		if (vpu_dev->secure_mode) {
			ret = wave5_allocate_secure_dma_memory(dev, &vb_buf);
		} else {
			ret = wave5_vdi_allocate_dma_memory(dev, &vb_buf);
		}
		if (ret)
			return -ENOMEM;
		array[i] = vb_buf;
	}

	for (i = count; i < WAVE5_MAX_FBS; i++) {
		if (vpu_dev->secure_mode)
			wave5_free_secure_dma_memory(&array[i]);
		else
			wave5_vdi_free_dma_memory(&array[i]);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(wave5_vdi_allocate_array);

int wave5_allocate_secure_dma_memory(struct device *dev, struct vpu_buf *vb)
{
	const char* secure_heap_name = "secure";
	struct dma_heap* secure_heap;
	struct dma_buf* buf;
	struct dma_buf_attachment *attachment = NULL;
	struct sg_table *sgt = NULL;
	unsigned long phys = 0;

	if (!vb->size) {
		dev_err(dev, "%s(): requested size==0\n", __func__);
		return -EINVAL;
	}

	secure_heap = dma_heap_find(secure_heap_name);
	// allocate secure dma_buf
	buf = dma_heap_buffer_alloc(secure_heap, vb->size, O_RDWR | O_CLOEXEC, 0);

	// Get phys addr
	attachment = dma_buf_attach(buf, dev);

	if (!attachment || IS_ERR(attachment)) {
		dma_buf_put(buf);
		return -EFAULT;
	}

	sgt = dma_buf_map_attachment(attachment, DMA_BIDIRECTIONAL);
	if (sgt && !IS_ERR(sgt)) {
		phys = sg_dma_address(sgt->sgl);

		dma_buf_unmap_attachment(attachment, sgt,
			DMA_BIDIRECTIONAL);
	}

	dma_buf_detach(buf, attachment);
	vb->daddr = (dma_addr_t)phys;
	vb->secure_dma_buf = buf;
	vb->dev = dev;
	return 0;
}
EXPORT_SYMBOL_GPL(wave5_allocate_secure_dma_memory);

void wave5_free_secure_dma_memory(struct vpu_buf *vb)
{
	if (vb->size == 0)
		return;

	dma_heap_buffer_free(vb->secure_dma_buf);
	memset(vb, 0, sizeof(*vb));
}
EXPORT_SYMBOL_GPL(wave5_free_secure_dma_memory);
MODULE_IMPORT_NS("DMA_BUF");
