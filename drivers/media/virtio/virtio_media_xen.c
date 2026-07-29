// SPDX-License-Identifier: GPL-2.0+
/*
 * Xen grant-table memory backend for the virtio-media driver.
 *
 * Copyright 2026 NXP
 *
 * This file implements the struct virtio_media_mem_ops that shares host
 * buffers with the Dom0 backend using Xen grant references:
 *
 *  - MMAP buffers: the host returns one grant reference per page; this backend
 *    allocates balloon pages, maps the foreign frames into them with
 *    gnttab_map_refs() and inserts them into userspace VMAs / exports them as
 *    dma-bufs.
 *  - imported dma-bufs: the backing pages are granted to the backend's domain
 *    (either decoded from the Xen grant DMA ops' encoding, or granted by hand).
 *
 * All Xen/grant-table specifics live here so the core driver
 * (virtio_media_driver.c) stays hypervisor-agnostic and can drive an
 * alternative backend (e.g. a virtio shared-memory region) through the same
 * ops.
 */

#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/dma-map-ops.h>
#include <linux/mm.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/virtio_dma_buf.h>

#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>

#include <xen/grant_table.h>
#include <xen/page.h>

#include <uapi/linux/virtio_media.h>
#include "session.h"
#include "virtio_media.h"

/**
 * struct virtio_media_xen_map - Xen grant-map backend private state.
 * @pages: balloon-backed pages holding the foreign frames granted by the host.
 * @map_ops: grant map operations, one per page.
 * @unmap_ops: grant unmap operations, one per page.
 *
 * Hung off virtio_media_host_mapping::priv for the lifetime of the mapping.
 */
struct virtio_media_xen_map {
	struct page **pages;
	struct gnttab_map_grant_ref *map_ops;
	struct gnttab_unmap_grant_ref *unmap_ops;
};

/**
 * struct virtio_media_xen_import - Xen grant-import backend private state.
 * @manual_grant: true if foreign access was granted by hand (grant DMA ops not
 *	active), so it must be ended at teardown.
 * @grant_ref_head: first grant reference of the manually-allocated sequence.
 * @grant_ref_count: number of references in that sequence.
 *
 * Hung off virtio_media_dmabuf_import::priv, allocated only when foreign access
 * is granted by hand.
 */
struct virtio_media_xen_import {
	bool manual_grant;
	grant_ref_t grant_ref_head;
	u32 grant_ref_count;
};

/*
 * DMA-address encoding used by the Xen grant DMA ops (see
 * drivers/xen/grant-dma-ops.c grant_to_dma()/dma_to_grant()): the top bit
 * flags a grant-backed address and the grant reference sits in the page-number
 * field. These constants are private to grant-dma-ops.c and not exported, so
 * they are duplicated here; they are part of the Xen para-virtual ABI and only
 * change in lockstep with that file. Decoding must use XEN_PAGE_SHIFT (the
 * grant page size), which happens to equal PAGE_SHIFT on this 4K-page arm64
 * build but is not guaranteed to in general.
 */
#define VIRTIO_MEDIA_XEN_GRANT_DMA_ADDR_OFF (1ULL << 63)

static grant_ref_t virtio_media_dma_to_grant(dma_addr_t dma)
{
	return (grant_ref_t)((dma & ~VIRTIO_MEDIA_XEN_GRANT_DMA_ADDR_OFF) >>
			     XEN_PAGE_SHIFT);
}

/*
 * dma-buf ops of buffers this driver exports via VIDIOC_EXPBUF; used to detect
 * (and reject) an attempt to re-import one of our own exported buffers. Defined
 * below with the EXPBUF implementation. The buffers are exported as
 * virtio-dma-bufs (host-object model: host pages + UUID), so their ops are
 * embedded in a struct virtio_dma_buf_ops.
 */
static const struct virtio_dma_buf_ops virtio_media_dmabuf_ops;

/*
 * MMAP grant-map lifecycle
 */

/**
 * virtio_media_xen_map_create() - Map the host's grant references into local
 *	balloon pages.
 * @vv: virtio-media device in use.
 * @map: grant map with its generic fields (grant_ref_count, len, uuid, ...)
 *	already filled in by the core.
 * @refs: array of @num_refs grant references returned by the host.
 * @num_refs: number of references (equals map->num_pages).
 * @rw: whether the host should be granted write access to the pages.
 *
 * Allocates the backend-private state (balloon pages + grant map/unmap op
 * arrays), maps the foreign frames and stores the state in map->priv.
 *
 * Returns 0 on success or a negative error code (the core frees @map).
 */
static int virtio_media_xen_map_create(struct virtio_media *vv,
				       struct virtio_media_host_mapping *map,
				       const u32 *refs, u32 num_refs, bool rw)
{
	struct virtio_media_xen_map *xm;
	u32 i;
	int ret;

	xm = kzalloc(sizeof(*xm), GFP_KERNEL);
	if (!xm)
		return -ENOMEM;

	xm->pages = kcalloc(num_refs, sizeof(struct page *), GFP_KERNEL);
	if (!xm->pages) {
		ret = -ENOMEM;
		goto err_free_xm;
	}

	/*
	 * Allocate empty, balloon-backed pages to be filled in with the
	 * foreign frames granted by the host via gnttab_map_refs(). This is
	 * the correct pairing for mapping foreign grant refs (do NOT use
	 * gnttab_dma_alloc_pages(), which allocates local DMA-able backing
	 * storage for the grantor side and returns non-cacheable coherent
	 * memory that faults on DC ZVA when zeroed).
	 */
	ret = gnttab_alloc_pages(num_refs, xm->pages);
	if (ret) {
		v4l2_err(&vv->v4l2_dev, "gnttab_alloc_pages failed: %d\n", ret);
		goto err_free_pages_array;
	}

	xm->map_ops = kcalloc(num_refs, sizeof(struct gnttab_map_grant_ref),
			      GFP_KERNEL);
	if (!xm->map_ops) {
		ret = -ENOMEM;
		goto err_free_grant_pages;
	}

	xm->unmap_ops = kcalloc(num_refs, sizeof(struct gnttab_unmap_grant_ref),
				GFP_KERNEL);
	if (!xm->unmap_ops) {
		ret = -ENOMEM;
		goto err_free_map_ops;
	}

	for (i = 0; i < num_refs; i++) {
		unsigned long pfn = page_to_pfn(xm->pages[i]);

		gnttab_set_map_op(&xm->map_ops[i],
				  (unsigned long)pfn_to_kaddr(pfn),
				  GNTMAP_host_map | (rw ? 0 : GNTMAP_readonly),
				  refs[i], 0 /* Dom0 */);
		gnttab_set_unmap_op(&xm->unmap_ops[i],
				    (unsigned long)pfn_to_kaddr(pfn),
				    GNTMAP_host_map | (rw ? 0 : GNTMAP_readonly),
				    0 /* handle - filled after map */);
	}

	ret = gnttab_map_refs(xm->map_ops, NULL, xm->pages, num_refs);
	if (ret) {
		v4l2_err(&vv->v4l2_dev, "gnttab_map_refs failed: %d\n", ret);
		goto err_free_unmap_ops;
	}

	for (i = 0; i < num_refs; i++) {
		if (xm->map_ops[i].status != GNTST_okay) {
			v4l2_err(&vv->v4l2_dev,
				 "grant map op %d failed: status %d\n", i,
				 xm->map_ops[i].status);
			ret = -ENOMEM;
			goto err_unmap_refs;
		}
		xm->unmap_ops[i].handle = xm->map_ops[i].handle;
	}

	map->priv = xm;
	return 0;

err_unmap_refs:
	gnttab_unmap_refs(xm->unmap_ops, NULL, xm->pages, num_refs);
err_free_unmap_ops:
	kfree(xm->unmap_ops);
err_free_map_ops:
	kfree(xm->map_ops);
err_free_grant_pages:
	gnttab_free_pages(num_refs, xm->pages);
err_free_pages_array:
	kfree(xm->pages);
err_free_xm:
	kfree(xm);
	return ret;
}

/**
 * virtio_media_xen_map_stop() - Unmap the grant references before MUNMAP.
 * @vv: virtio-media device in use.
 * @map: grant map being torn down.
 *
 * Runs before the core sends MUNMAP, matching the original teardown order
 * (unmap grant refs, then MUNMAP, then free pages).
 */
static void virtio_media_xen_map_stop(struct virtio_media *vv,
				      struct virtio_media_host_mapping *map)
{
	struct virtio_media_xen_map *xm = map->priv;
	int ret;

	if (!xm)
		return;

	ret = gnttab_unmap_refs(xm->unmap_ops, NULL, xm->pages,
				map->num_pages);
	if (ret)
		v4l2_err(&vv->v4l2_dev, "gnttab_unmap_refs failed: %d\n", ret);
}

/**
 * virtio_media_xen_map_free() - Free the grant pages and backend state.
 * @vv: virtio-media device in use.
 * @map: grant map being torn down.
 *
 * Runs after the core has sent MUNMAP.
 */
static void virtio_media_xen_map_free(struct virtio_media *vv,
				      struct virtio_media_host_mapping *map)
{
	struct virtio_media_xen_map *xm = map->priv;

	if (!xm)
		return;

	/* Free the balloon-backed grant pages. */
	gnttab_free_pages(map->num_pages, xm->pages);

	kfree(xm->map_ops);
	kfree(xm->unmap_ops);
	kfree(xm->pages);
	kfree(xm);
	map->priv = NULL;
}

/**
 * virtio_media_xen_sync() - CPU/device cache maintenance for one grant map.
 * @map: grant map to sync.
 * @dir: DMA direction (DMA_TO_DEVICE cleans, otherwise invalidates).
 *
 * The grant pages back a buffer physically owned by Dom0; the local VPU is
 * non-coherent, so the guest must maintain caches by physical address around
 * QBUF/DQBUF.
 */
static void virtio_media_xen_sync(struct virtio_media_host_mapping *map,
				  enum dma_data_direction dir)
{
	struct virtio_media_xen_map *xm = map->priv;
	u32 i;

	if (!xm)
		return;

	for (i = 0; i < map->num_pages; i++) {
		phys_addr_t phys = page_to_phys(xm->pages[i]);

		if (dir == DMA_TO_DEVICE)
			arch_sync_dma_for_device(phys, PAGE_SIZE, dir);
		else
			arch_sync_dma_for_cpu(phys, PAGE_SIZE, dir);
	}
}

/*
 * Userspace VMA mapping of MMAP buffers.
 */

/**
 * struct virtio_media_vma_handler - refcount handler for a mmapped grant map.
 * @vv: virtio-media device the mapping belongs to.
 * @map: grant map backing this VMA.
 * @refs: number of VMAs currently sharing this handler. Governs only the
 *	lifetime of the handler allocation itself; each of those VMAs also
 *	holds one reference on @map (so @refs equals this handler's
 *	contribution to map->refs, but the two counts are tracked separately
 *	to keep their responsibilities distinct: @refs frees the handler,
 *	map->refs frees the grant map).
 *
 * Stored in vma->vm_private_data so that fork()/partial-unmap of a userspace
 * mapping of a MMAP buffer keeps the grant map alive for exactly as long as
 * any VMA references it, mirroring vb2_common_vm_ops.
 */
struct virtio_media_vma_handler {
	struct virtio_media *vv;
	struct virtio_media_host_mapping *map;
	refcount_t refs;
};

static void virtio_media_vm_open(struct vm_area_struct *vma)
{
	struct virtio_media_vma_handler *h = vma->vm_private_data;

	/*
	 * A new VMA (e.g. from fork() or a VMA split) now shares this handler
	 * and references the same grant map. Take a grant-map reference so the
	 * buffer's grant refs and pages outlive every userspace mapping, and a
	 * handler reference so the handler outlives every VMA using it.
	 */
	refcount_inc(&h->refs);
	virtio_media_mapping_get(h->map);
}

static void virtio_media_vm_close(struct vm_area_struct *vma)
{
	struct virtio_media_vma_handler *h = vma->vm_private_data;
	struct virtio_media *vv = h->vv;
	struct virtio_media_host_mapping *map = h->map;

	/*
	 * Drop this VMA's grant-map reference, then its handler reference.
	 * Read h->vv/h->map into locals first: once the handler reference is
	 * dropped below h may be freed, so it must not be dereferenced after.
	 *
	 * vm_close() runs under mmap_lock, but virtio_media_mapping_put() is
	 * lock-free and never tears the map down inline (the MUNMAP, which
	 * sleeps waiting for the host, is deferred to the release worker), so
	 * it is safe to call here without stalling the address space or
	 * risking an mmap_lock -> vlock deadlock.
	 */
	virtio_media_mapping_put(vv, map);
	if (refcount_dec_and_test(&h->refs))
		kfree(h);
}

static const struct vm_operations_struct virtio_media_vm_ops = {
	.open = virtio_media_vm_open,
	.close = virtio_media_vm_close,
};

/**
 * virtio_media_xen_mmap - Perform a mmap request from userspace.
 * @file: opened file of the session to map for.
 * @vma: VM area struct describing the desired mapping.
 *
 * This maps the grant-backed pages of an already-created MMAP buffer into the
 * userspace address space.
 */
static int virtio_media_xen_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct video_device *video_dev = video_devdata(file);
	struct virtio_media *vv = to_virtio_media(video_dev);
	struct virtio_media_session *session =
		fh_to_session(file->private_data);
	struct virtio_media_host_mapping *map;
	struct virtio_media_xen_map *xm;
	struct virtio_media_vma_handler *handler;
	unsigned long addr;
	u32 offset;
	int i, ret;

	if (!(vma->vm_flags & VM_SHARED))
		return -EINVAL;
	if (!(vma->vm_flags & (VM_READ | VM_WRITE)))
		return -EINVAL;

	offset = vma->vm_pgoff << PAGE_SHIFT;

	/*
	 * The grant map for this buffer must already exist: it is created at
	 * QUERYBUF time. We only map the already grant-mapped pages into the
	 * userspace VMA here.
	 *
	 * Hold vv->vlock across the whole mapping: it is the lock that
	 * serialises grant map creation/destruction (QUERYBUF, REQBUFS via
	 * virtio_media_free_queue_host_mappings(), session close). Without it a
	 * concurrent REQBUFS could free @map and its pages while we walk the
	 * list and dereference map->pages here, causing a use-after-free. The
	 * inner queues_lock keeps the ordering consistent with expbuf
	 * (vv->vlock -> queues_lock) and guards the host_mappings list walk.
	 */
	mutex_lock(&vv->vlock);
	mutex_lock(&session->queues_lock);
	map = virtio_media_find_host_mapping_by_offset(session, offset);
	if (!map) {
		dev_dbg(&video_dev->dev,
			"mmap for offset 0x%x with no grant map (QUERYBUF first?)\n",
			offset);
		ret = -EINVAL;
		goto unlock;
	}

	/* We cannot let the mapping be larger than the buffer. */
	if (vma->vm_end - vma->vm_start > PAGE_ALIGN(map->len)) {
		dev_dbg(&video_dev->dev,
			"invalid MMAP, as it would overflow buffer length\n");
		ret = -EINVAL;
		goto unlock;
	}

	xm = map->priv;

	/*
	 * The grant-mapped pages are real struct pages obtained from
	 * gnttab_alloc_pages(), so insert them individually with
	 * vm_insert_page(). This keeps the pages properly refcounted and
	 * turns the VMA into a VM_MIXEDMAP mapping, avoiding the COW
	 * (do_wp_page) path that a VM_PFNMAP mapping via remap_pfn_range()
	 * would otherwise trigger on the first write.
	 */
	vm_flags_set(vma, VM_DONTEXPAND | VM_DONTDUMP);

	addr = vma->vm_start;
	for (i = 0; i < map->num_pages && addr < vma->vm_end; i++) {
		ret = vm_insert_page(vma, addr, xm->pages[i]);
		if (ret) {
			dev_err(&video_dev->dev,
				"vm_insert_page failed at page %d: %d\n", i,
				ret);
			goto unlock;
		}
		addr += PAGE_SIZE;
	}

	/*
	 * Track this userspace mapping with a refcount on the grant map, the
	 * same way vb2_common_vm_ops tracks vb2 buffer mmaps. Without this a
	 * REQBUFS (source change) that releases the queue's own reference would
	 * free the grant map -- unmapping the grant refs and telling the host
	 * to release the buffer -- while the mapping is still live in
	 * userspace, so Dom0 would log "g.e. 0x... still in use!". Holding a
	 * reference here keeps the grant refs and pages mapped until the last
	 * VMA is torn down (virtio_media_vm_close()).
	 *
	 * Set vma->vm_ops only after every page is inserted: if we bailed out
	 * above, the mapping is torn down by the caller without a close()
	 * callback and no reference has been taken, so there is nothing to
	 * unwind.
	 */
	handler = kzalloc(sizeof(*handler), GFP_KERNEL);
	if (!handler) {
		ret = -ENOMEM;
		goto unlock;
	}
	handler->vv = vv;
	handler->map = map;
	refcount_set(&handler->refs, 1);
	virtio_media_mapping_get(map);
	vma->vm_ops = &virtio_media_vm_ops;
	vma->vm_private_data = handler;

	ret = 0;
unlock:
	mutex_unlock(&session->queues_lock);
	mutex_unlock(&vv->vlock);
	return ret;
}

/*
 * dma-buf exporter for VIDIOC_EXPBUF support
 */

struct virtio_media_dmabuf_priv {
	struct file *file;
	struct virtio_media *vv;
	struct virtio_media_host_mapping *map;
	struct page **pages;
	unsigned int num_pages;
};

static struct sg_table *virtio_media_dmabuf_map(struct dma_buf_attachment *attach,
						enum dma_data_direction dir)
{
	struct virtio_media_dmabuf_priv *priv = attach->dmabuf->priv;
	struct sg_table *sgt;
	int ret;

	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return ERR_PTR(-ENOMEM);

	ret = sg_alloc_table_from_pages(sgt, priv->pages, priv->num_pages,
					0, (unsigned long)priv->num_pages * PAGE_SIZE,
					GFP_KERNEL);
	if (ret) {
		kfree(sgt);
		return ERR_PTR(ret);
	}

	/*
	 * priv->pages come from gnttab_alloc_pages(): they are Xen grant pages
	 * backing a buffer physically owned by the Dom0 backend. On arm64 Xen
	 * DomU these pages are not in the kernel linear map, so page_to_virt()
	 * on them is invalid. A regular dma_map_sgtable() would have the
	 * DMA-direct layer call arch_sync_dma_for_device() ->
	 * dcache_clean_poc() on that bogus virtual address and take a ttbr
	 * address size fault. Skip the CPU cache maintenance: the buffer is
	 * shared coherently through the grant/virtio path and cache management
	 * for the real device happens on the Dom0 side.
	 */
	if (dma_map_sgtable(attach->dev, sgt, dir, DMA_ATTR_SKIP_CPU_SYNC)) {
		sg_free_table(sgt);
		kfree(sgt);
		return ERR_PTR(-ENOMEM);
	}

	return sgt;
}

static void virtio_media_dmabuf_unmap(struct dma_buf_attachment *attach,
				      struct sg_table *sgt,
				      enum dma_data_direction dir)
{
	dma_unmap_sgtable(attach->dev, sgt, dir, DMA_ATTR_SKIP_CPU_SYNC);
	sg_free_table(sgt);
	kfree(sgt);
}

static void virtio_media_dmabuf_release(struct dma_buf *buf)
{
	struct virtio_media_dmabuf_priv *priv = buf->priv;

	/*
	 * Drop this dma-buf's reference on the grant map. The grant map is torn
	 * down (grant refs unmapped, host told to release the buffer via MUNMAP
	 * and pages freed) only once its refcount reaches zero, i.e. after the
	 * owning queue released its reference (REQBUFS / session close) AND
	 * every userspace mapping of this dma-buf has been torn down (each holds
	 * its own reference via virtio_media_vm_ops). Holding a reference until
	 * here guarantees the grant pages outlive every mapping of this dma-buf.
	 */
	virtio_media_mapping_put(priv->vv, priv->map);

	/*
	 * Drop the reference on the session's open file taken in
	 * virtio_media_xen_expbuf().
	 */
	fput(priv->file);
	kfree(priv);
}

static int virtio_media_dmabuf_mmap(struct dma_buf *buf,
				    struct vm_area_struct *vma)
{
	struct virtio_media_dmabuf_priv *priv = buf->priv;
	struct virtio_media_vma_handler *handler;
	unsigned long addr = vma->vm_start;
	int i, ret;

	/*
	 * Same rationale as virtio_media_xen_mmap(): the pages are real
	 * struct pages from gnttab_alloc_pages(), so insert them with
	 * vm_insert_page() to keep them refcounted and avoid the COW
	 * (do_wp_page) path that a VM_PFNMAP mapping would trigger on write.
	 */
	vm_flags_set(vma, VM_DONTEXPAND | VM_DONTDUMP);

	for (i = 0; i < priv->num_pages && addr < vma->vm_end; i++) {
		ret = vm_insert_page(vma, addr, priv->pages[i]);
		if (ret)
			return ret;
		addr += PAGE_SIZE;
	}

	/*
	 * Track this userspace mapping with a grant-map reference, exactly like
	 * virtio_media_xen_mmap(). userspace may close() the dma-buf fd
	 * while keeping the mapping, which drops the dma-buf's own grant-map
	 * reference; without a reference for the mapping itself the grant map
	 * could be torn down (grant refs unmapped + MUNMAP) while the VMA still
	 * points at those grant pages, making Dom0 log "g.e. 0x... still in
	 * use!". Set vma->vm_ops only after every page is inserted (see the
	 * matching comment in virtio_media_xen_mmap()).
	 */
	handler = kzalloc(sizeof(*handler), GFP_KERNEL);
	if (!handler)
		return -ENOMEM;
	handler->vv = priv->vv;
	handler->map = priv->map;
	refcount_set(&handler->refs, 1);
	virtio_media_mapping_get(priv->map);
	vma->vm_ops = &virtio_media_vm_ops;
	vma->vm_private_data = handler;

	return 0;
}

static int virtio_media_dmabuf_get_uuid(struct dma_buf *buf, uuid_t *uuid)
{
	struct virtio_media_dmabuf_priv *priv = buf->priv;

	/*
	 * Return the shared-object UUID the backend registered this host buffer
	 * under (host-object model: host pages + UUID). A consumer device on
	 * the same hypervisor resolves it back to the host dma-buf
	 * cross-process. The UUID is all-zero if the backend registered none,
	 * in which case the cross-process handoff is unavailable but the buffer
	 * is otherwise usable through the normal grant/MMAP data path.
	 */
	import_uuid(uuid, priv->map->uuid);
	return 0;
}

static const struct virtio_dma_buf_ops virtio_media_dmabuf_ops = {
	.ops = {
		.map_dma_buf = virtio_media_dmabuf_map,
		.unmap_dma_buf = virtio_media_dmabuf_unmap,
		.release = virtio_media_dmabuf_release,
		.mmap = virtio_media_dmabuf_mmap,
		/*
		 * virtio_dma_buf_export() requires ops.attach to be
		 * virtio_dma_buf_attach so is_virtio_dma_buf() recognises the
		 * buffer and get_uuid() can be queried by an importer.
		 */
		.attach = virtio_dma_buf_attach,
	},
	.get_uuid = virtio_media_dmabuf_get_uuid,
};

static int virtio_media_xen_expbuf(struct file *file, void *fh,
				   struct v4l2_exportbuffer *eb)
{
	struct virtio_media_session *session =
		fh_to_session(file->private_data);
	struct virtio_media_host_mapping *map;
	struct virtio_media_xen_map *xm;
	struct virtio_media_dmabuf_priv *priv;
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	struct dma_buf *dmabuf;
	int fd;

	if (eb->type > VIRTIO_MEDIA_LAST_QUEUE)
		return -EINVAL;

	/*
	 * The grant map for this (type, index, plane) must already exist: it is
	 * created at QUERYBUF time. Looking it up by exact type/index/plane
	 * ensures a CAPTURE buffer never picks up an OUTPUT buffer's mapping.
	 */
	scoped_guard(mutex, &session->queues_lock) {
		map = virtio_media_find_host_mapping(session, eb->type, eb->index,
						   eb->plane);
		if (!map)
			return -EINVAL;

		xm = map->priv;

		priv = kzalloc(sizeof(*priv), GFP_KERNEL);
		if (!priv)
			return -ENOMEM;

		priv->vv = to_virtio_media(session->fh.vdev);
		priv->map = map;
		priv->pages = xm->pages;
		priv->num_pages = map->num_pages;
		/*
		 * Keep the grant map (and its grant pages) alive for as long as
		 * this dma-buf exists, even if the buffers are freed with
		 * REQBUFS(0) or the session is closed first. The reference is
		 * dropped in virtio_media_dmabuf_release(). Taken under
		 * queues_lock while the map is still guaranteed to be live.
		 */
		virtio_media_mapping_get(map);
		/*
		 * Also reference the session's open file so the session (and
		 * thus the device used to send MUNMAP at release time) stays
		 * valid until the dma-buf is released.
		 */
		priv->file = get_file(session->file);

		exp_info.ops = &virtio_media_dmabuf_ops.ops;
		/*
		 * The dma-buf is mapped page by page (grant_ref_count pages), so
		 * its size must be the page-aligned page count rather than the
		 * raw byte length. A non page-aligned map->len would make the
		 * kernel dma-buf mmap path reject user mappings whose
		 * (page-rounded) length exceeds dmabuf->size.
		 */
		exp_info.size = (u64)map->num_pages << PAGE_SHIFT;
		exp_info.flags = eb->flags;
		exp_info.priv = priv;

		/*
		 * Export as a virtio-dma-buf (host-object model: host pages +
		 * UUID) so an importer can query the buffer's shared-object
		 * UUID via virtio_dma_buf_get_uuid() (our get_uuid callback).
		 * Falls back to nothing else: virtio_dma_buf_export() only
		 * validates the ops and forwards to dma_buf_export().
		 */
		dmabuf = virtio_dma_buf_export(&exp_info);
	}
	if (IS_ERR(dmabuf)) {
		/*
		 * The release callback is not called when export fails, so
		 * undo the references taken above by hand.
		 */
		virtio_media_mapping_put(priv->vv, priv->map);
		fput(priv->file);
		kfree(priv);
		return PTR_ERR(dmabuf);
	}

	fd = dma_buf_fd(dmabuf, eb->flags);
	if (fd < 0) {
		dma_buf_put(dmabuf);
		return fd;
	}

	eb->fd = fd;
	return 0;
}

/*
 * dma-buf import: granting foreign access to imported buffers' pages.
 */

/**
 * virtio_media_import_decode_refs() - Collect the grant references backing an
 *	imported dma-buf, from the grant DMA ops' encoding of @sgt.
 * @imp: import whose mapped sg_table to decode.
 * @refs_out: on success, receives a kmalloc'ed array of grant references the
 *	caller must kfree().
 * @num_refs_out: on success, receives the number of references in @refs_out.
 *
 * Used when the virtio device's Xen grant DMA ops are active: each DMA address
 * in @sgt already carries the grant reference the grant DMA ops assigned to it
 * (top bit set, see virtio_media_dma_to_grant()). Walks @sgt and decodes each
 * DMA address into its run of references.
 *
 * Returns 0 on success or a negative error code.
 */
static int
virtio_media_import_decode_refs(struct virtio_media_dmabuf_import *imp,
				u32 **refs_out, u32 *num_refs_out)
{
	struct scatterlist *sg;
	u32 *refs;
	u32 num_refs = 0;
	unsigned int i;
	u32 k = 0;

	/* First pass: count grant references across the mapped sg_table. */
	for_each_sgtable_dma_sg(imp->sgt, sg, i)
		num_refs += gnttab_count_grant(sg_dma_address(sg),
					       sg_dma_len(sg));

	if (!num_refs)
		return -EINVAL;

	refs = kmalloc_array(num_refs, sizeof(*refs), GFP_KERNEL);
	if (!refs)
		return -ENOMEM;

	/* Second pass: decode each DMA address into its run of grant refs. */
	for_each_sgtable_dma_sg(imp->sgt, sg, i) {
		dma_addr_t addr = sg_dma_address(sg);
		grant_ref_t base = virtio_media_dma_to_grant(addr);
		unsigned int n = gnttab_count_grant(addr, sg_dma_len(sg));
		unsigned int j;

		for (j = 0; j < n; j++)
			refs[k++] = base + j;
	}

	*refs_out = refs;
	*num_refs_out = num_refs;

	return 0;
}

/**
 * virtio_media_import_manual_grant() - Grant foreign access to an imported
 *	dma-buf's pages by hand.
 * @vv: virtio-media device in use.
 * @imp: import whose mapped sg_table to grant. On success, the backend-private
 *	state (imp->priv) records the granted sequence.
 * @refs_out: on success, receives a kmalloc'ed array of grant references the
 *	caller must kfree().
 * @num_refs_out: on success, receives the number of references in @refs_out.
 *
 * Used when the virtio device's Xen grant DMA ops are NOT active, so the DMA
 * addresses in @sgt are plain guest-physical addresses rather than encoded
 * grant references. Allocates one contiguous grant-reference sequence covering
 * all pages of @sgt and grants the backend's domain foreign access to each
 * page, mirroring what the grant DMA ops would have done. The references must
 * be ended (see virtio_media_import_end_grant()) at teardown.
 *
 * Returns 0 on success or a negative error code.
 */
static int
virtio_media_import_manual_grant(struct virtio_media *vv,
				 struct virtio_media_dmabuf_import *imp,
				 u32 **refs_out, u32 *num_refs_out)
{
	/*
	 * The backend always runs in Dom0 on this platform, so grant foreign
	 * access to domid 0. If virtio-media is ever served from a non-Dom0
	 * backend this must instead come from the device's backend domid.
	 */
	domid_t backend_domid = 0;
	struct virtio_media_xen_import *xi;
	struct scatterlist *sg;
	grant_ref_t head;
	u32 *refs;
	u32 num_refs = 0;
	unsigned int i;
	u32 k = 0;
	int ret;

	/* First pass: count the pages we must grant across the sg_table. */
	for_each_sgtable_dma_sg(imp->sgt, sg, i)
		num_refs += gnttab_count_grant(sg_dma_address(sg),
					       sg_dma_len(sg));

	if (!num_refs)
		return -EINVAL;

	refs = kmalloc_array(num_refs, sizeof(*refs), GFP_KERNEL);
	if (!refs)
		return -ENOMEM;

	xi = kzalloc(sizeof(*xi), GFP_KERNEL);
	if (!xi) {
		kfree(refs);
		return -ENOMEM;
	}

	ret = gnttab_alloc_grant_reference_seq(num_refs, &head);
	if (ret) {
		v4l2_err(&vv->v4l2_dev,
			 "failed to allocate %u grant references: %d\n",
			 num_refs, ret);
		kfree(xi);
		kfree(refs);
		return ret;
	}

	/* Second pass: grant foreign access to each page, in sequence. */
	for_each_sgtable_dma_sg(imp->sgt, sg, i) {
		unsigned long gfn = sg_dma_address(sg) >> XEN_PAGE_SHIFT;
		unsigned int n = gnttab_count_grant(sg_dma_address(sg),
						    sg_dma_len(sg));
		unsigned int j;

		for (j = 0; j < n; j++, k++) {
			gnttab_grant_foreign_access_ref(head + k, backend_domid,
							gfn + j, 0);
			refs[k] = head + k;
		}
	}

	xi->manual_grant = true;
	xi->grant_ref_head = head;
	xi->grant_ref_count = num_refs;
	imp->priv = xi;

	*refs_out = refs;
	*num_refs_out = num_refs;

	return 0;
}

/**
 * virtio_media_xen_import_get_entries() - Obtain the grant references backing an
 *	imported dma-buf, whichever grant path applies.
 * @vv: virtio-media device in use.
 * @imp: import whose mapped sg_table to grant/decode.
 * @refs_out: on success, receives a kmalloc'ed array the caller must kfree().
 * @num_refs_out: on success, receives the number of references.
 *
 * Detects whether the Xen grant DMA ops are active by inspecting the top bit
 * of the first DMA address (set only when the grant DMA ops encoded a grant
 * reference there, see virtio_media_dma_to_grant()). If set, the references
 * are decoded from @sgt; otherwise foreign access is granted by hand.
 *
 * Returns 0 on success or a negative error code.
 */
static int
virtio_media_xen_import_get_entries(struct virtio_media *vv,
				 struct virtio_media_dmabuf_import *imp,
				 u32 **refs_out, u32 *num_refs_out)
{
	dma_addr_t first;

	if (!imp->sgt || !imp->sgt->sgl)
		return -EINVAL;

	first = sg_dma_address(imp->sgt->sgl);

	if (first & VIRTIO_MEDIA_XEN_GRANT_DMA_ADDR_OFF)
		return virtio_media_import_decode_refs(imp, refs_out,
						       num_refs_out);

	return virtio_media_import_manual_grant(vv, imp, refs_out,
						num_refs_out);
}

/**
 * virtio_media_xen_import_put_entries() - End foreign access granted by hand.
 * @imp: import previously processed by virtio_media_xen_import_get_entries().
 *
 * Ends foreign access on each reference and frees the sequence. No-op unless
 * foreign access was granted by hand. Must be called after the backend has
 * dropped its reference to the backing (i.e. after DMABUF_DETACH), otherwise
 * Xen refuses to end a grant still in use.
 */
static void
virtio_media_xen_import_put_entries(struct virtio_media_dmabuf_import *imp)
{
	struct virtio_media_xen_import *xi = imp->priv;
	u32 i;

	if (!xi)
		return;

	if (xi->manual_grant) {
		for (i = 0; i < xi->grant_ref_count; i++)
			gnttab_end_foreign_access_ref(xi->grant_ref_head + i);

		gnttab_free_grant_reference_seq(xi->grant_ref_head,
						xi->grant_ref_count);
	}

	kfree(xi);
	imp->priv = NULL;
}

const struct virtio_media_mem_ops virtio_media_xen_mem_ops = {
	.name = "xen",
	.wants_entries_buffer = true,
	.map_create = virtio_media_xen_map_create,
	.map_stop = virtio_media_xen_map_stop,
	.map_free = virtio_media_xen_map_free,
	.mmap = virtio_media_xen_mmap,
	.sync = virtio_media_xen_sync,
	.expbuf = virtio_media_xen_expbuf,
	.import_get_entries = virtio_media_xen_import_get_entries,
	.import_put_entries = virtio_media_xen_import_put_entries,
};
