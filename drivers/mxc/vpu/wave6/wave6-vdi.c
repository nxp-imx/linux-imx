// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Wave6 series multi-standard codec IP - low level access functions
 *
 * Copyright (C) 2021 CHIPS&MEDIA INC
 */

#include <linux/bug.h>
#include "wave6-vdi.h"
#include "wave6-vpu.h"
#include "wave6-regdefine.h"
#include <linux/delay.h>
#include <linux/trusty/smcall.h>
#include <linux/trusty/trusty.h>

#define VDI_SYSTEM_ENDIAN VDI_LITTLE_ENDIAN
#define VDI_128BIT_BUS_SYSTEM_ENDIAN VDI_128BIT_LITTLE_ENDIAN

#define SMC_ENTITY_IMX_WAVE_LINUX_OPT 55
#define SMC_IMX_VPU_REG SMC_FASTCALL_NR(SMC_ENTITY_IMX_WAVE_LINUX_OPT, 1)
#define OPT_WRITE 0x1
#define OPT_READ  0x2

#ifdef writel
#undef writel
#define writel(val, addr) \
	do { \
		if (vpu_dev->trusty_dev) { \
			trusty_vpu_set_reg(vpu_dev->trusty_dev, (addr - vpu_dev->reg_base), val); \
		} else { \
			{ __iowmb(); writel_relaxed((val),(addr)); } \
		}\
	} while (0)
#endif

static void trusty_vpu_set_reg(struct device *dev, u32 target, u32 val) {
	trusty_fast_call32(dev, SMC_IMX_VPU_REG, target, OPT_WRITE, val);
}

static int trusty_vpu_get_reg(struct device *dev, u32 target) {
	return trusty_fast_call32(dev, SMC_IMX_VPU_REG, target, OPT_READ, 0);
}

void wave6_vdi_writel(struct vpu_device *vpu_dev, unsigned int addr, unsigned int data)
{
	writel(data, vpu_dev->reg_base + addr);
}

unsigned int wave6_vdi_readl(struct vpu_device *vpu_dev, u32 addr)
{
	if (vpu_dev->trusty_dev)
		return trusty_vpu_get_reg(vpu_dev->trusty_dev, addr);
	else
		return readl(vpu_dev->reg_base + addr);
}

int wave6_allocate_secure_dma_memory(struct vpu_device *vpu_dev, struct vpu_buf *vb)
{
	const char* secure_heap_name = "secure";
	struct dma_heap* secure_heap;
	struct dma_buf* buf;
	struct dma_buf_attachment *attachment = NULL;
	struct sg_table *sgt = NULL;
	struct device* dev;
	unsigned long phys = 0;
	dev = vpu_dev->dev;

	if (!vb->size) {
		dev_err(vpu_dev->dev, "%s(): requested size==0\n", __func__);
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
	return 0;
}

void wave6_free_secure_dma_memory(struct vpu_device *vpu_dev, struct vpu_buf *vb)
{
	if (vb->size == 0)
		return;

	dma_heap_buffer_free(vb->secure_dma_buf);

        memset(vb, 0, sizeof(*vb));
}

MODULE_IMPORT_NS(DMA_BUF);
