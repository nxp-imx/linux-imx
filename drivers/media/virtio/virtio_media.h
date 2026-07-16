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
	} cmd;

	union {
		struct virtio_media_resp_open open;
		struct virtio_media_resp_munmap munmap;
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

/* virtio_media_ioctls.c */

long virtio_media_device_ioctl(struct file *file, unsigned int cmd,
			       unsigned long arg);
extern const struct v4l2_ioctl_ops virtio_media_ioctl_ops;

#endif // __VIRTIO_MEDIA_H

