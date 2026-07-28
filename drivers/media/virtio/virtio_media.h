/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0+ */

/*
 * Virtio-media structures & functions declarations.
 *
* Copyright (c) 2024-2026 Google LLC.
 */

#ifndef __VIRTIO_MEDIA_H
#define __VIRTIO_MEDIA_H

#include <linux/virtio_config.h>
#include <linux/refcount.h>
#include <linux/llist.h>
#include <linux/idr.h>
#include <media/v4l2-device.h>
#include <xen/grant_table.h>
#include <linux/dma-map-ops.h>

#include <uapi/linux/virtio_media.h>

/**
 * struct virtio_media_grant_map - Tracks a grant reference mapping.
 * @grant_ref_header: first grant reference of the mapping.
 * @grant_ref_count: number of grant references (pages) in the mapping.
 * @len: length of the mapping in bytes.
 * @pages: array of struct page pointers for the mapped pages.
 * @map_ops: array of grant map operations.
 * @unmap_ops: array of grant unmap operations.
 * @v4l2_offset: v4l2 mem_offset uniquely identifying this (type, index, plane).
 * @type: v4l2 buffer type this mapping belongs to.
 * @index: buffer index within the queue.
 * @plane: plane index within the buffer.
 * @refs: reference count. Held by the owning queue while the mapping is
 *	linked into its grant_maps list, by every dma-buf exported from this
 *	mapping (VIDIOC_EXPBUF), and by every userspace VMA that mmaps it. The
 *	grant references are unmapped, the host is told to release the buffer
 *	(MUNMAP) and the pages are freed only when this count drops to zero, so
 *	grant pages outlive any dma-buf or mmap still referencing them.
 * @list: link into the queue's grant_maps list.
 * @release_node: link into virtio_media.gmap_release_list. Once the last
 *	reference is dropped the gmap is queued here and torn down by
 *	virtio_media.gmap_release_work (the sole teardown path; see
 *	virtio_media_gmap_put()).
 * @uuid: shared-object UUID the device registered this host buffer under
 *	(host-object model: host pages + UUID). Reported in the MMAP response
 *	and returned by the exported virtio-dma-buf's get_uuid() callback so a
 *	consumer device can resolve the buffer cross-process. All-zero if the
 *	device registered none (UUID handoff unavailable for this buffer).
 */
struct virtio_media_grant_map {
	u32 grant_ref_header;
	u32 grant_ref_count;
	u64 len;
	struct page **pages;
	struct gnttab_map_grant_ref *map_ops;
	struct gnttab_unmap_grant_ref *unmap_ops;
	u32 v4l2_offset;
	u32 type;
	u32 index;
	u32 plane;
	refcount_t refs;
	struct list_head list;
	struct llist_node release_node;
	u8 uuid[16];
};

/**
 * struct virtio_media_dmabuf_import - A guest dma-buf imported for
 *	V4L2_MEMORY_DMABUF, with its pages granted to the backend's domain.
 * @dmabuf: the imported dma-buf (holds a reference via dma_buf_get()).
 * @attach: our attachment to @dmabuf, against the virtio device.
 * @sgt: mapped scatter-gather table for @attach.
 * @resource_id: device-assigned id under which the backend knows this backing
 *	(sent with DMABUF_ATTACH and, thereafter, as the ``m.fd`` of every
 *	QBUF/PREPARE_BUF that uses it).
 *
 * Grant references backing @sgt are obtained one of two ways, transparently:
 *
 * - If the virtio device's Xen grant DMA ops are active
 *   (CONFIG_XEN_VIRTIO_FORCE_GRANT and a "xen,grant-dma" IOMMU node in the
 *   guest device tree), mapping @attach against the virtio device already
 *   grants the pages to the backend's domain and encodes the grant references
 *   in the sg_table's DMA addresses. The references are then decoded from @sgt
 *   and released automatically when @sgt is unmapped and @attach detached.
 *
 * - Otherwise (grant DMA ops not installed, e.g. no such IOMMU node), the
 *   DMA addresses in @sgt are plain guest-physical addresses. We then grant
 *   foreign access to the backing pages by hand (see @manual_grant), and must
 *   end that foreign access and free the reference sequence ourselves at
 *   teardown.
 *
 * @manual_grant: true if we granted foreign access by hand (second case
 *	above); the grants must be ended at teardown. False when the grant DMA
 *	ops did it for us.
 * @grant_ref_head: first grant reference of the contiguous sequence allocated
 *	for a manual grant (valid only when @manual_grant).
 * @grant_ref_count: number of references in that sequence (valid only when
 *	@manual_grant).
 *
 * @uuid_backed: true if this import is a host-owned buffer identified by a
 *	shared-object UUID rather than grant-imported guest pages. Such an
 *	import holds no attach/sgt/grants and no backend resource id: it only
 *	records the UUID and keeps a reference on the imported virtio-dma-buf
 *	(the guest-side alpha refcount chain). Its planes are sent on the
 *	QBUF/PREPARE_BUF wire as a per-plane UUID footer, not an @resource_id.
 * @uuid: the shared-object UUID (valid only when @uuid_backed), as returned by
 *	the imported virtio-dma-buf's get_uuid() callback.
 *
 * @list: link into the owning queue's ``dmabuf_imports`` list.
 *
 * An import is owned by the queue (linked into its ``dmabuf_imports`` list) and
 * deduplicated by ``struct dma_buf *``: the same dma-buf queued on several
 * buffers or planes is imported (attached + DMABUF_ATTACH) only once, and every
 * (buffer index, plane) slot that uses it (see struct virtio_media_buffer's
 * dmabuf_bindings) simply points at the one shared import without owning it.
 *
 * This deliberately departs from videobuf2's per-(buffer, plane) dma-buf
 * lifetime (vb2 detaches on rebind): detaching while the queue is still
 * streaming races the backend, which may still hold the backing (the buffer is
 * on the device and has not been DQBUF'd), causing the guest to end grants Xen
 * still considers in use ("g.e. still in use"), leaking references and failing
 * the next export. So instead the backing stays attached for the queue's whole
 * REQBUFS lifetime and is torn down (DMABUF_DETACH + end grants + unmap +
 * detach) only once the device has quiesced: when the queue's buffers are freed
 * (REQBUFS with any count, including 0) or the session closes.
 */
struct virtio_media_dmabuf_import {
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	u32 resource_id;
	bool manual_grant;
	grant_ref_t grant_ref_head;
	u32 grant_ref_count;
	bool uuid_backed;
	u8 uuid[16];
	struct list_head list;
};

#define DESC_CHAIN_MAX_LEN SG_MAX_SINGLE_ALLOC

/**
 * struct virtio_media_cmd_callback_param - Callback parameters to the virtio
 *                                          command queue.
 * @vv: virtio-media device in use.
 * @done: flag to be switched once the command is completed.
 * @resp_len: length of the received response from the command. Only valid
 * after @done has switched to ``true``.
 */
struct virtio_media_cmd_callback_param {
	struct virtio_media *vv;
	bool done;
	size_t resp_len;
};

/*
 * Maximum number of grant references (pages) supported for a single MMAP
 * buffer mapping. Sizes the response buffer the host uses to return the
 * (non-contiguous) grant ref array. 4096 refs covers a 16 MiB buffer.
 */
#define VIRTIO_MEDIA_MAX_GRANT_REFS 4096

#define VIRTIO_MEDIA_DEFAULT_DRIVER_NAME "virtio-media"

extern bool virtio_media_allow_userptr;
extern bool virtio_media_allow_dmabuf;

/**
 * struct virtio_media - Virtio-media device.
 * @v4l2_dev: v4l2_device for the media device.
 * @video_dev: video_device for the media device.
 * @virtio_dev: virtio device for the media device.
 * @commandq: virtio command queue.
 * @eventq: virtio event queue.
 * @eventq_work: work to run when events are received on @eventq.
 * @mmap_region: region into which MMAP buffers are mapped by the host.
 * @event_buffer: buffer for event descriptors.
 * @sessions: list of active sessions on the device.
 * @sessions_lock: protects @sessions and &struct virtio_media_session.list.
 * @events_lock: prevents concurrent processing of events.
 * @cmd: union of the device commands ``open`` and ``munmap``. The other
 *       commands are handled by &struct virtio_media_session
 * @resp: union of responses to device commands ``open`` and ``munmap``. The
 *        other responses are handled by &struct virtio_media_session
 * @vlock: serializes access to the command queue.
 * @wq: waitqueue for host responses on the command queue.
 */
struct virtio_media {
	struct v4l2_device v4l2_dev;
	struct video_device video_dev;

	struct virtio_device *virtio_dev;
	struct virtqueue *commandq;
	struct virtqueue *eventq;
	struct work_struct eventq_work;

	/*
	 * Deferred grant-map teardown. Tearing a grant map down is a heavy,
	 * sleeping operation (unmap grant refs + send MUNMAP and wait seconds
	 * for the host, under vlock). virtio_media_gmap_put() therefore never
	 * does it inline -- on the last reference it pushes the gmap onto
	 * @gmap_release_list and schedules @gmap_release_work, which is the
	 * sole teardown path. This keeps the put path lock-free and callable
	 * from any context (e.g. mmap close() under mmap_lock, dma-buf release,
	 * or REQBUFS/close while holding vlock).
	 */
	struct llist_head gmap_release_list;
	struct work_struct gmap_release_work;

	struct virtio_shm_region mmap_region;

	void *event_buffer;

	struct list_head sessions;
	struct mutex sessions_lock; /* protects sessions list */

	struct mutex events_lock; /* prevents concurrent event processing */

	__dma_from_device_group_begin();
	union {
		struct virtio_media_cmd_open open;
		struct virtio_media_cmd_munmap munmap;
		struct virtio_media_cmd_dmabuf_attach attach;
		struct virtio_media_cmd_dmabuf_detach detach;
	} cmd;

	union {
		struct virtio_media_resp_open open;
		struct virtio_media_resp_munmap munmap;
		struct virtio_media_resp_dmabuf_attach attach;
		struct virtio_media_resp_dmabuf_detach detach;
	} resp;
	__dma_from_device_group_end();

	struct mutex vlock; /* serializes command queue access */
	wait_queue_head_t wq;

	/*
	 * Callback parameter for the in-flight command on @commandq. Its
	 * lifetime is tied to the device (not the caller's stack) because a
	 * command that times out leaves its buffer in the virtqueue: a late
	 * response would otherwise make commandq_callback() dereference a
	 * freed stack object. Access is serialized by @vlock; the interrupt
	 * callback only ever touches the object pointed to here and the
	 * device-global @wq, both of which outlive any single command.
	 * @cmd_abandoned is set when the waiter gave up (timeout) so the
	 * callback knows the response is orphaned.
	 */
	struct virtio_media_cmd_callback_param cmd_cb;
	bool cmd_abandoned; /* in-flight command timed out; response is orphaned */

	/*
	 * When true, command payloads are placed in coherent DMA memory and
	 * submitted to the command virtqueue as premapped scatterlists, to
	 * avoid cache-coherency issues when the host accesses them through
	 * foreign/grant mappings. @cmd_dma_dev is the device to allocate that
	 * coherent memory against. Set from the ``use_coherent_shadow_buffer``
	 * module parameter, and cleared if the commandq does not use the DMA
	 * API.
	 */
	bool use_coherent;
	struct device *cmd_dma_dev;

	/*
	 * Coherent control buffers for the device-global OPEN/MUNMAP commands,
	 * used only when @use_coherent is set. Access is serialized by @vlock.
	 * Each is one page, which is enough for the fixed command/response
	 * structures. MMAP's variable-length grant-ref array is allocated
	 * separately (see virtio_media_send_mmap_cmd()).
	 */
	void *ctrl_cmd;
	dma_addr_t ctrl_cmd_dma;
	void *ctrl_resp;
	dma_addr_t ctrl_resp_dma;

	/*
	 * Allocator for V4L2_MEMORY_DMABUF resource ids. Each imported dma-buf
	 * backing is assigned one id (see virtio_media_resource_id_get()), sent
	 * to the backend with DMABUF_ATTACH and reused as the buffer's m.fd in
	 * every subsequent QBUF/PREPARE_BUF, and freed on DMABUF_DETACH (see
	 * virtio_media_resource_id_put()).
	 *
	 * An IDA (not a monotonic counter) is used so that ids are bounded by
	 * the number of *concurrently* attached backings and reclaimed on
	 * detach. This avoids two problems a free-running counter would have on
	 * a long-lived device: the counter eventually wrapping around the u32
	 * space, and -- worse -- a wrapped id colliding with one still held by
	 * a long-lived session (e.g. a 24/7 encoder) while short-lived decoder
	 * instances churn through ids (an ABA hazard). The IDA never hands out
	 * an id that is currently in use, so a live backing's id is unique for
	 * as long as it is attached. Accessed under @vlock (attach/detach run
	 * from ioctls that hold it), and the IDA has its own internal locking.
	 */
	struct ida resource_ida;

#if IS_ENABLED(CONFIG_DEBUG_FS)
	/* Parent debugfs dir holding the per-session "instance.<id>" files. */
	struct dentry *debugfs;
#endif
};

static inline struct virtio_media *
to_virtio_media(struct video_device *video_dev)
{
	return container_of(video_dev, struct virtio_media, video_dev);
}

/* virtio_media_driver.c */

int virtio_media_send_command(struct virtio_media *vv, struct scatterlist **sgs,
			      const size_t out_sgs, const size_t in_sgs,
			      bool premapped, size_t minimum_resp_len,
			      size_t *resp_len);
void virtio_media_process_events(struct virtio_media *vv);

/* virtio_media_driver.c */

int virtio_media_expbuf(struct file *file, void *fh,
		       struct v4l2_exportbuffer *eb);

struct virtio_media_session;
int virtio_media_map_buffer(struct virtio_media_session *session, u32 type,
			    u32 index);
void virtio_media_free_queue_grant_maps(struct virtio_media_session *session,
					u32 type);
void virtio_media_free_session_grant_maps(struct virtio_media_session *session);
void virtio_media_sync_buffer(struct virtio_media_session *session, u32 type,
			      u32 index, enum dma_data_direction dir);
int virtio_media_bind_buffer_dmabufs(struct virtio_media_session *session,
				     struct v4l2_buffer *b);
void virtio_media_free_queue_dmabuf_imports(struct virtio_media_session *session,
					    u32 type);
void virtio_media_free_session_dmabuf_imports(
	struct virtio_media_session *session);
int virtio_media_dmabuf_substitute_resource_ids(
	struct virtio_media_session *session, struct v4l2_buffer *b,
	u32 *saved_fds);
void virtio_media_dmabuf_restore_resource_ids(struct v4l2_buffer *b,
					      u32 *saved_fds);
int virtio_media_dmabuf_collect_uuids(struct virtio_media_session *session,
				      struct v4l2_buffer *b,
				      struct virtio_media_dmabuf_uuid *footers,
				      unsigned int *n_out);

/* virtio_media_ioctls.c */

long virtio_media_device_ioctl(struct file *file, unsigned int cmd,
			       unsigned long arg);
extern const struct v4l2_ioctl_ops virtio_media_ioctl_ops;

struct virtio_media_queue_state;
void virtio_media_free_queue_buffers(struct virtio_media_queue_state *queue);

#endif // __VIRTIO_MEDIA_H

