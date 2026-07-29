/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0+ */

/*
 * Definitions of virtio-media session related structures.
 *
 * Copyright (c) 2024-2026 Google LLC.
 */

#ifndef __VIRTIO_MEDIA_SESSION_H
#define __VIRTIO_MEDIA_SESSION_H

#include <linux/scatterlist.h>
#include <media/v4l2-fh.h>

#include <uapi/linux/virtio_media.h>
#include "virtio_media_debug.h"

#define VIRTIO_MEDIA_LAST_QUEUE (V4L2_BUF_TYPE_META_OUTPUT)

/*
 * Size of the per-session virtio shadow and event buffers. 16K should be
 * enough to contain everything we need.
 */
#define VIRTIO_SHADOW_BUF_SIZE 0x4000

/**
 * struct virtio_media_buffer - Current state of a buffer.
 * @buffer: &struct v4l2_buffer with current information about the buffer.
 * @planes: backing planes array for @buffer.
 * @dmabuf_bindings: for a ``V4L2_MEMORY_DMABUF`` buffer, the dma-buf import
 * bound to each plane (or NULL). This is a *non-owning* back-pointer into the
 * queue's ``dmabuf_imports`` list: imports are owned and deduplicated by the
 * queue (keyed by ``struct dma_buf *``), so several planes/buffers queued with
 * the same dma-buf share one import. Used only to resolve a plane to its
 * backend resource id on the QBUF/PREPARE_BUF wire path; the pointer is cleared
 * (not freed here) when the queue's imports are released. NULL for non-DMABUF
 * buffers.
 * @prepared: set once the buffer has been prepared (memory-type binding done
 * and, for DMABUF, the planes attached) by PREPARE_BUF or the first QBUF, and
 * cleared on DQBUF and STREAMOFF. Mirrors videobuf2's ``vb->prepared`` so the
 * preparation work runs at most once per queueing cycle regardless of whether
 * userspace calls PREPARE_BUF before QBUF.
 * @list: link into the list of buffers pending dequeue.
 */
struct virtio_media_buffer {
	struct v4l2_buffer buffer;
	struct v4l2_plane planes[VIDEO_MAX_PLANES];
	struct virtio_media_dmabuf_import *dmabuf_bindings[VIDEO_MAX_PLANES];
	bool prepared;
	struct list_head list;
};

/**
 * struct virtio_media_queue_state - Represents the state of a V4L2 queue.
 * @streaming: Whether the queue is currently streaming.
 * @allocated_bufs: How many buffers are currently allocated.
 * @is_capture_last: set to true when the last buffer has been received on a
 * capture queue, so we can return -EPIPE on subsequent DQBUF requests.
 * @buffers: Array of @allocated_bufs pointers to individually allocated buffer
 * states. The buffers are allocated separately (rather than as one contiguous
 * array) so that growing the pool with CREATE_BUFS only reallocates this
 * pointer array: the buffer states themselves never move, keeping every
 * reference to them (notably the pending_dqbufs list nodes embedded in each
 * buffer) valid across a resize.
 * @queued_bufs: How many buffers are currently queued on the device.
 * @pending_dqbufs: Buffers that are available for being dequeued.
 * @host_mappings: list of ``struct virtio_media_host_mapping`` for buffers of this
 * queue (one entry per (index, plane)). Populated at QUERYBUF time and freed
 * on REQBUFS(0) or session close.
 * @dmabuf_imports: list of ``struct virtio_media_dmabuf_import`` for a
 * ``V4L2_MEMORY_DMABUF`` queue. Imports are owned here and deduplicated by
 * ``struct dma_buf *`` (one entry per distinct dma-buf, shared by every plane
 * that queues it). Populated lazily at QBUF/PREPARE_BUF time and freed only
 * once the device has quiesced: on REQBUFS (any count) or session close.
 * @qbuf_count: running total of buffers successfully queued (QBUF) on this
 * queue, for debugfs statistics only.
 * @dqbuf_count: running total of buffers dequeued (DQBUF events) on this queue,
 * for debugfs statistics only.
 */
struct virtio_media_queue_state {
	bool streaming;
	size_t allocated_bufs;
	bool is_capture_last;

	struct virtio_media_buffer **buffers;
	size_t queued_bufs;
	struct list_head pending_dqbufs;
	struct list_head host_mappings;
	struct list_head dmabuf_imports;
	enum v4l2_memory memory;

	u64 qbuf_count;
	u64 dqbuf_count;
};

/**
 * struct virtio_media_session - A session on a virtio_media device.
 * @fh: file handler for the session.
 * @file: file pointer associated with the session's file handler.
 * @id: session ID used to communicate with the device.
 * @nonblocking_dequeue: whether dequeue should block or not (nonblocking if
 *                       file opened with O_NONBLOCK).
 * @uses_mplane: whether the queues for this session use the MPLANE API or not.
 * @cmd: union of session commands ``close``, ``ioctl``, and ``mmap``. A
 *       session can have one command currently running. The rest of the
 *       commands are handled by &struct virtio_media.
 * @resp: union of responses to session commands ``close``, ``ioctl``, and
 *        ``mmap``. A session can wait on one command only. The rest of the
 *        responses are handled by &struct virtio_media.
 * @shadow_buf: shadow buffer where data to be added to the descriptor chain can
 *              be staged before being sent to the device.
 * @command_sgs: SG table gathering descriptors for a given command and its
 *               response.
 * @queues: state of all the queues for this session.
 * @queues_lock: protects all members for the queues for this session.
 * @dqbuf_wait: waitqueue for dequeued buffers, if ``VIDIOC_DQBUF`` needs to
 *              block or when polling.
 * @list: link into the list of sessions for the device.
 */
struct virtio_media_session {
	struct v4l2_fh fh;
	struct file *file;
	u32 id;
	bool nonblocking_dequeue;
	bool uses_mplane;
	/* Set once a VIRTIO_MEDIA_EVT_ERROR was received for this session. */
	bool error;

	__dma_from_device_group_begin();
	union {
		struct virtio_media_cmd_close close;
		struct virtio_media_cmd_ioctl ioctl;
		struct virtio_media_cmd_mmap mmap;
		struct virtio_media_cmd_dmabuf_attach dmabuf_attach;
		struct virtio_media_cmd_dmabuf_detach dmabuf_detach;
	} cmd;

	union {
		struct virtio_media_resp_ioctl ioctl;
		struct virtio_media_resp_mmap mmap;
		struct virtio_media_resp_dmabuf_attach dmabuf_attach;
		struct virtio_media_resp_dmabuf_detach dmabuf_detach;
	} resp;
	__dma_from_device_group_end();

	void *shadow_buf;
	dma_addr_t shadow_buf_dma;

	struct sg_table command_sgs;

	struct virtio_media_queue_state queues[VIRTIO_MEDIA_LAST_QUEUE + 1];
	struct mutex queues_lock; /* protects queues array and states */
	wait_queue_head_t dqbuf_wait;

	struct list_head list;

#if IS_ENABLED(CONFIG_DEBUG_FS)
	/* Per-session debugfs "instance.<id>" file and its flow recorder. */
	struct dentry *debugfs;
	struct virtio_media_flow flow;
#endif
};

static inline struct virtio_media_session *fh_to_session(struct v4l2_fh *fh)
{
	return container_of(fh, struct virtio_media_session, fh);
}

static inline void
virtio_media_session_fh_add(struct virtio_media_session *session,
			    struct file *file)
{
	v4l2_fh_add(&session->fh, file);
	session->file = file;
}

static inline void
virtio_media_session_fh_del(struct virtio_media_session *session)
{
	v4l2_fh_del(&session->fh, session->file);
}

#endif // __VIRTIO_MEDIA_SESSION_H
