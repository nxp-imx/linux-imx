// SPDX-License-Identifier: GPL-2.0
/*
 * DMABUF Protected heap exporter
 *
 * Copyright 2025 NXP
 */

#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <linux/dma-mapping.h>
#include <linux/highmem.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/protected_memory_allocator.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>

static struct dma_heap *protected_heap;
static struct protected_memory_allocator_device *pma_dev_global;

struct protected_heap_buffer {
	struct protected_memory_allocator_device *pma_dev;
	struct protected_memory_allocation *pma;
	unsigned long len;
	unsigned int order;
	struct sg_table sg_table;
};

struct protected_heap_attachment {
	struct device *dev;
	struct sg_table *table;
};

static struct sg_table *dup_sg_table(struct sg_table *table)
{
	struct sg_table *new_table;
	int ret, i;
	struct scatterlist *sg, *new_sg;

	new_table = kzalloc(sizeof(*new_table), GFP_KERNEL);
	if (!new_table)
		return ERR_PTR(-ENOMEM);

	ret = sg_alloc_table(new_table, table->orig_nents, GFP_KERNEL);
	if (ret) {
		kfree(new_table);
		return ERR_PTR(-ENOMEM);
	}

	new_sg = new_table->sgl;
	for_each_sgtable_sg(table, sg, i) {
		sg_set_page(new_sg, sg_page(sg), sg->length, sg->offset);
		new_sg = sg_next(new_sg);
	}

	return new_table;
}

static int protected_heap_attach(struct dma_buf *dmabuf,
				 struct dma_buf_attachment *attachment)
{
	struct protected_heap_buffer *buffer = dmabuf->priv;
	struct protected_heap_attachment *a;
	struct sg_table *table;

	a = kzalloc(sizeof(*a), GFP_KERNEL);
	if (!a)
		return -ENOMEM;

	table = dup_sg_table(&buffer->sg_table);
	if (IS_ERR(a->table)) {
		kfree(a);
		return PTR_ERR(a->table);
	}

	a->table = table;
	a->dev = attachment->dev;
	attachment->priv = a;

	pr_debug("protected_heap: attach dev=%s\n", dev_name(a->dev));
	return 0;
}

static void protected_heap_detach(struct dma_buf *dmabuf,
				  struct dma_buf_attachment *attachment)
{
	struct protected_heap_attachment *a = attachment->priv;

	if (!a)
		return;

	if (a->table) {
		sg_free_table(a->table);
		kfree(a->table);
	}

	kfree(a);
	attachment->priv = NULL;
}

static struct sg_table *protected_heap_map_dma_buf(struct dma_buf_attachment *attachment,
                                                   enum dma_data_direction direction)
{
	struct protected_heap_attachment *a = attachment->priv;
	struct sg_table *table = a->table;
	int attr = attachment->dma_map_attrs;
	int ret;

	if (!a || !a->table) {
		pr_err("protected_heap: invalid attachment or table\n");
		return ERR_PTR(-EINVAL);
	}

	attr |= DMA_ATTR_SKIP_CPU_SYNC;
	ret = dma_map_sgtable(attachment->dev, table, direction, attr);

	if (ret) {
		pr_err("protected_heap: dma_map_sgtable failed\n");
		return ERR_PTR(ret);
	}
	return table;
}

static void protected_heap_unmap_dma_buf(struct dma_buf_attachment *attachment,
					 struct sg_table *table,
					 enum dma_data_direction direction)
{
	struct protected_heap_attachment *a = attachment->priv;
	int attr = attachment->dma_map_attrs;
	attr |= DMA_ATTR_SKIP_CPU_SYNC;

	if (!a || !a->table)
		return;

	dma_unmap_sgtable(attachment->dev, table, direction, attr);
}

static int protected_heap_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	pr_warn("protected_heap: mmap not allowed\n");
	return -EPERM;
}

static void protected_heap_release(struct dma_buf *dmabuf)
{
	struct protected_heap_buffer *buffer = dmabuf->priv;

	if (!buffer)
		return;

	sg_free_table(&buffer->sg_table);

	if (buffer->pma && buffer->pma_dev)
		buffer->pma_dev->ops.pma_free_page(buffer->pma_dev, buffer->pma);

	kfree(buffer);
}

static const struct dma_buf_ops protected_dma_buf_ops = {
	.attach = protected_heap_attach,
	.detach = protected_heap_detach,
	.map_dma_buf = protected_heap_map_dma_buf,
	.unmap_dma_buf = protected_heap_unmap_dma_buf,
	.mmap = protected_heap_mmap,
	.release = protected_heap_release,
};

static struct dma_buf *protected_heap_allocate(struct dma_heap *heap,
					       unsigned long len,
					       u32 fd_flags,
					       u64 heap_flags)
{
	struct protected_heap_buffer *buffer;
	struct protected_memory_allocation *pma;
	struct dma_buf *dmabuf;
	struct sg_table *table;
	unsigned int order = get_order(len);
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	phys_addr_t pa;
	int ret = -ENOMEM;

	pma = pma_dev_global->ops.pma_alloc_page(pma_dev_global, order);
	if (!pma)
		return ERR_PTR(ret);

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer) {
		pma_dev_global->ops.pma_free_page(buffer->pma_dev, buffer->pma);
		return ERR_PTR(ret);
	}

	buffer->pma_dev = pma_dev_global;
	buffer->pma = pma;
	buffer->order = order;
	buffer->len = PAGE_SIZE << order;

	table = &buffer->sg_table;
	ret = sg_alloc_table(table, 1, GFP_KERNEL);
	if (ret)
		goto free_buffer;

	pa = buffer->pma_dev->ops.pma_get_phys_addr(buffer->pma_dev, buffer->pma);
	sg_set_page(table->sgl, phys_to_page(pa), buffer->len, 0);
	sg_dma_address(table->sgl) = pa;
	sg_dma_len(table->sgl) = buffer->len;

	exp_info.exp_name = dma_heap_get_name(heap);
	exp_info.ops = &protected_dma_buf_ops;
	exp_info.size = buffer->len;
	exp_info.flags = fd_flags;
	exp_info.priv = buffer;
	dmabuf = dma_buf_export(&exp_info);
	if (IS_ERR(dmabuf)) {
		ret = PTR_ERR(dmabuf);
		goto free_pages;
	}

	return dmabuf;

free_pages:
	sg_free_table(table);
free_buffer:
	buffer->pma_dev->ops.pma_free_page(buffer->pma_dev, buffer->pma);
	kfree(buffer);
	return ERR_PTR(ret);
}

static const struct dma_heap_ops protected_heap_ops = {
	.allocate = protected_heap_allocate,
};

static int protected_heap_create(void)
{
	struct dma_heap_export_info exp_info;
	struct device_node *np;
	struct platform_device *pdev;

	np = of_find_compatible_node(NULL, NULL, "arm,protected-memory-allocator");
	if (!np) {
		pr_err("protected_heap: DT node not found\n");
		return -ENODEV;
	}

	pdev = of_find_device_by_node(np);
	if (!pdev) {
		pr_err("protected_heap: fail to find pdev for PMA\n");
		return -ENODEV;
	}

	of_node_put(np);
	pma_dev_global = platform_get_drvdata(pdev);
	if (!pma_dev_global) {
		pr_err("protected_heap: failed to get PMA drvdata\n");
		return -ENODEV;
	}

	exp_info.name = "protected";
	exp_info.ops = &protected_heap_ops;
	exp_info.priv = NULL;

	protected_heap = dma_heap_add(&exp_info);
	if (IS_ERR(protected_heap)) {
		pr_err("protected_heap: failed to add dma-heap\n");
		return PTR_ERR(protected_heap);
	}

	pr_info("DMABUF Protected heap initialized successfully\n");
	return 0;
}
module_init(protected_heap_create);
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS("DMA_BUF");
MODULE_IMPORT_NS("DMA_BUF_HEAP");
