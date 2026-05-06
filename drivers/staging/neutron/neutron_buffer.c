// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2023-2026 NXP
 */

/****************************************************************************/

#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/of_address.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/scatterlist.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>

#include "neutron_buffer.h"
#include "neutron_device.h"
#include "uapi/neutron.h"

/****************************************************************************/

static void neutron_buffer_destroy(struct kref *kref)
{
	struct neutron_buffer *buf =
		container_of(kref, struct neutron_buffer, kref);

	dev_dbg(buf->ndev->dev, "Buffer destroy. buf=0x%pS\n", buf);

	if (buf->firmware_p)
		release_firmware(buf->firmware_p);

	dma_free_attrs(buf->ndev->dev, buf->size, buf->cpu_addr,
		       buf->dma_addr, DMA_ATTR_FORCE_CONTIGUOUS);
	devm_kfree(buf->ndev->dev, buf);
}

/****************************************************************************/
/* DMA-BUF operations                                                       */
/****************************************************************************/

static struct sg_table *neutron_dmabuf_map(struct dma_buf_attachment *attach,
					   enum dma_data_direction dir)
{
	struct neutron_buffer *buf = attach->dmabuf->priv;
	struct sg_table *sgt;
	int ret;

	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return ERR_PTR(-ENOMEM);

	/* Build sg_table from the CMA allocation — dma_get_sgtable_attrs
	 * correctly describes the physical pages regardless of how the
	 * kernel virtual address was mapped.
	 */
	ret = dma_get_sgtable_attrs(buf->ndev->dev, sgt, buf->cpu_addr,
				    buf->dma_addr, buf->size,
				    DMA_ATTR_FORCE_CONTIGUOUS);
	if (ret) {
		kfree(sgt);
		return ERR_PTR(ret);
	}

	/* Create IOMMU mappings for the IMPORTING device (e.g. Mali GPU).
	 * This is critical — without it, the importer's IOMMU has no
	 * translation for these pages and writes go nowhere.
	 */
	ret = dma_map_sgtable(attach->dev, sgt, dir, DMA_ATTR_SKIP_CPU_SYNC);
	if (ret) {
		sg_free_table(sgt);
		kfree(sgt);
		return ERR_PTR(ret);
	}

	return sgt;
}

static void neutron_dmabuf_unmap(struct dma_buf_attachment *attach,
				 struct sg_table *sgt,
				 enum dma_data_direction dir)
{
	dma_unmap_sgtable(attach->dev, sgt, dir, DMA_ATTR_SKIP_CPU_SYNC);
	sg_free_table(sgt);
	kfree(sgt);
}

static int neutron_dmabuf_mmap(struct dma_buf *dmabuf,
			       struct vm_area_struct *vma)
{
	struct neutron_buffer *buf = dmabuf->priv;

	/* dma_buf_mmap() sets vma->vm_pgoff to the offset within the buffer.
	 * dma_mmap_attrs() adds vm_pgoff internally, so reset to 0 here.
	 */
	vma->vm_pgoff = 0;

	return dma_mmap_attrs(buf->ndev->dev, vma, buf->cpu_addr,
			      buf->dma_addr, buf->size,
			      DMA_ATTR_FORCE_CONTIGUOUS);
}

static void neutron_dmabuf_release(struct dma_buf *dmabuf)
{
	struct neutron_buffer *buf = dmabuf->priv;

	neutron_buffer_put(buf);
}

static int neutron_dmabuf_begin_cpu_access(struct dma_buf *dmabuf,
					   enum dma_data_direction dir)
{
	struct neutron_buffer *buf = dmabuf->priv;

	dma_sync_single_for_cpu(buf->ndev->dev, buf->dma_addr,
				buf->size, dir);
	return 0;
}

static int neutron_dmabuf_end_cpu_access(struct dma_buf *dmabuf,
					 enum dma_data_direction dir)
{
	struct neutron_buffer *buf = dmabuf->priv;

	dma_sync_single_for_device(buf->ndev->dev, buf->dma_addr,
				   buf->size, dir);
	return 0;
}

static const struct dma_buf_ops neutron_dmabuf_ops = {
	.map_dma_buf      = neutron_dmabuf_map,
	.unmap_dma_buf    = neutron_dmabuf_unmap,
	.mmap             = neutron_dmabuf_mmap,
	.release          = neutron_dmabuf_release,
	.begin_cpu_access = neutron_dmabuf_begin_cpu_access,
	.end_cpu_access   = neutron_dmabuf_end_cpu_access,
};

/****************************************************************************/

int neutron_buffer_create(struct neutron_device *ndev,
			  size_t size, __u64 *addr_out)
{
	struct neutron_buffer *buf;
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	int ret = -ENOMEM;

	if (!size)
		return -EINVAL;

	buf = devm_kzalloc(ndev->dev, sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf->ndev = ndev;
	buf->size = size;
	kref_init(&buf->kref);

	buf->cpu_addr = dma_alloc_attrs(buf->ndev->dev, size,
					&buf->dma_addr, GFP_KERNEL, DMA_ATTR_FORCE_CONTIGUOUS);
	if (!buf->cpu_addr)
		goto free_buf;

	exp_info.ops = &neutron_dmabuf_ops;
	exp_info.size = PAGE_ALIGN(buf->size);
	exp_info.flags = O_RDWR | O_CLOEXEC;
	exp_info.priv = buf;
	exp_info.exp_name = "neutron";

	buf->dmabuf = dma_buf_export(&exp_info);
	if (IS_ERR(buf->dmabuf)) {
		ret = PTR_ERR(buf->dmabuf);
		goto free_dma;
	}

	ret = dma_buf_fd(buf->dmabuf, O_RDWR | O_CLOEXEC);
	if (ret < 0)
		goto free_dmabuf;

	buf->file = buf->dmabuf->file;
	*addr_out = buf->dma_addr;

	dev_dbg(buf->ndev->dev,
		"Buffer create. fd=%d, size=%zu, cpu_addr=0x%pK, dma_addr=0x%llx\n",
		ret, size, buf->cpu_addr, buf->dma_addr);

	return ret;

free_dmabuf:
	dma_buf_put(buf->dmabuf);

free_dma:
	dma_free_attrs(buf->ndev->dev, buf->size, buf->cpu_addr,
		       buf->dma_addr, DMA_ATTR_FORCE_CONTIGUOUS);

free_buf:
	devm_kfree(buf->ndev->dev, buf);

	return ret;
}

struct neutron_buffer *neutron_buffer_get_from_fd(int fd)
{
	struct neutron_buffer *buf;
	struct dma_buf *dmabuf;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf))
		return ERR_PTR(-EINVAL);

	if (dmabuf->ops != &neutron_dmabuf_ops) {
		dma_buf_put(dmabuf);
		return ERR_PTR(-EINVAL);
	}

	buf = dmabuf->priv;
	dma_buf_put(dmabuf);

	return buf;
}

void neutron_buffer_get(struct neutron_buffer *buf)
{
	kref_get(&buf->kref);
}

void neutron_buffer_put(struct neutron_buffer *buf)
{
	kref_put(&buf->kref, neutron_buffer_destroy);
}
