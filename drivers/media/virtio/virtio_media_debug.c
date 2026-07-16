// SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0+

/*
 * virtio-media debug interface.
 *
 * Exposes, per session (one open fd == one "instance"), a debugfs file
 *   <debugfs>/virtio-media/instance.<id>
 * showing the session's basic state and a ring buffer of its most recent flow
 * events. Modelled on the wave6 VPU driver's debugfs so the guest-side (DomU)
 * flow can be diffed against the host-side (Dom0) wave6 flow -- e.g. to check
 * whether a source-change event the host emitted was actually received and
 * acted upon by the guest.
 *
 * Copyright 2025-2026 NXP
 */

#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include "session.h"
#include "virtio_media.h"
#include "virtio_media_debug.h"

#define VIRTIO_MEDIA_DEBUGFS_DIR "virtio-media"

static const char * const virtio_media_flow_name[] = {
	[VIRTIO_MEDIA_FLOW_NONE] = "none",
	[VIRTIO_MEDIA_FLOW_OPEN] = "open",
	[VIRTIO_MEDIA_FLOW_CLOSE] = "close",
	[VIRTIO_MEDIA_FLOW_STREAMON] = "streamon",
	[VIRTIO_MEDIA_FLOW_STREAMOFF] = "streamoff",
	[VIRTIO_MEDIA_FLOW_REQBUFS] = "reqbufs",
	[VIRTIO_MEDIA_FLOW_SUBSCRIBE_EVENT] = "subscribe_event",
	[VIRTIO_MEDIA_FLOW_EVENT] = "event",
	[VIRTIO_MEDIA_FLOW_SOURCE_CHANGE] = "source_change",
	[VIRTIO_MEDIA_FLOW_EOS] = "eos",
	[VIRTIO_MEDIA_FLOW_DECODER_CMD] = "decoder_cmd",
	[VIRTIO_MEDIA_FLOW_ERROR] = "error",
	[VIRTIO_MEDIA_FLOW_MIN_BUFFERS] = "min_buffers_cap",
};

static const char *virtio_media_buf_type_name(u32 type)
{
	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		return "CAPTURE";
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		return "CAPTURE_MPLANE";
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		return "OUTPUT";
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		return "OUTPUT_MPLANE";
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
		return "OVERLAY";
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_OVERLAY:
		return "OUTPUT_OVERLAY";
	case V4L2_BUF_TYPE_VBI_CAPTURE:
		return "VBI_CAPTURE";
	case V4L2_BUF_TYPE_VBI_OUTPUT:
		return "VBI_OUTPUT";
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
		return "SLICED_VBI_CAPTURE";
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		return "SLICED_VBI_OUTPUT";
	case V4L2_BUF_TYPE_SDR_CAPTURE:
		return "SDR_CAPTURE";
	case V4L2_BUF_TYPE_SDR_OUTPUT:
		return "SDR_OUTPUT";
	case V4L2_BUF_TYPE_META_CAPTURE:
		return "META_CAPTURE";
	case V4L2_BUF_TYPE_META_OUTPUT:
		return "META_OUTPUT";
	default:
		return "UNKNOWN";
	}
}

static const char *virtio_media_memory_name(enum v4l2_memory memory)
{
	switch (memory) {
	case V4L2_MEMORY_MMAP:
		return "MMAP";
	case V4L2_MEMORY_USERPTR:
		return "USERPTR";
	case V4L2_MEMORY_OVERLAY:
		return "OVERLAY";
	case V4L2_MEMORY_DMABUF:
		return "DMABUF";
	default:
		return "NONE";
	}
}

void virtio_media_record_flow(struct virtio_media_session *session, u32 flow,
			      u32 arg1, u32 arg2)
{
	int index;

	if (!session)
		return;

	scoped_guard(spinlock, &session->flow.lock)
		index = session->flow.index++;

	index %= VIRTIO_MEDIA_FLOW_DEPTH;
	session->flow.flows[index].arg1 = arg1;
	session->flow.flows[index].arg2 = arg2;
	/* Publish the fully populated slot last so the reader sees it whole. */
	WRITE_ONCE(session->flow.flows[index].key, flow);
}

static void virtio_media_dbg_queue(struct seq_file *s, u32 type,
				   struct virtio_media_queue_state *q)
{
	unsigned int pending = 0;
	struct list_head *e;

	list_for_each(e, &q->pending_dqbufs)
		pending++;

	seq_printf(s,
		   "%s: streaming %d, memory %s, allocated %zu, queued %zu, pending_dqbuf %u, capture_last %d, qbuf %llu, dqbuf %llu\n",
		   virtio_media_buf_type_name(type), q->streaming,
		   virtio_media_memory_name(q->memory), q->allocated_bufs,
		   q->queued_bufs, pending, q->is_capture_last,
		   q->qbuf_count, q->dqbuf_count);
}

static int virtio_media_dbg_instance_show(struct seq_file *s, void *data)
{
	struct virtio_media_session *session = s->private;
	int start;
	int i;

	seq_printf(s, "[session %u] mplane %d, error %d, nonblocking %d\n",
		   session->id, session->uses_mplane, session->error,
		   session->nonblocking_dequeue);

	/*
	 * The queue array is indexed by V4L2 buffer type; only print the ones
	 * that were actually used (allocated or streaming) to keep the dump
	 * focused. For a decoder these are the OUTPUT and CAPTURE queues (in
	 * their single- or multi-planar variant).
	 */
	for (i = 0; i <= VIRTIO_MEDIA_LAST_QUEUE; i++) {
		struct virtio_media_queue_state *q = &session->queues[i];

		if (!q->allocated_bufs && !q->streaming)
			continue;

		virtio_media_dbg_queue(s, i, q);
	}

	seq_puts(s, "flow (oldest first):\n");

	/*
	 * Snapshot the write index under the lock, then walk the ring from the
	 * oldest slot. Slot contents are read without the lock; recorders only
	 * append, and the key is published last, so a torn read at worst shows
	 * a stale/empty slot -- acceptable for a diagnostic dump.
	 */
	scoped_guard(spinlock, &session->flow.lock)
		start = session->flow.index;

	for (i = 0; i < VIRTIO_MEDIA_FLOW_DEPTH; i++) {
		struct virtio_media_flow_item *item =
			&session->flow.flows[(start + i) % VIRTIO_MEDIA_FLOW_DEPTH];
		u32 key = READ_ONCE(item->key);

		if (key == VIRTIO_MEDIA_FLOW_NONE || key >= VIRTIO_MEDIA_FLOW_MAXIMUM)
			continue;

		seq_printf(s, "    %s, %u, %u\n", virtio_media_flow_name[key],
			   item->arg1, item->arg2);
	}

	return 0;
}

DEFINE_SHOW_ATTRIBUTE(virtio_media_dbg_instance);

void virtio_media_debug_session_init(struct virtio_media_session *session)
{
	spin_lock_init(&session->flow.lock);
}

/*
 * Create <debugfs>/virtio-media/<video-node>/ to hold this device's per-session
 * files. Keying the subdir on the video device node keeps multiple virtio-media
 * devices from colliding, and lets the whole subtree be removed on unbind.
 * Must be called after video_register_device() so the node name is assigned.
 */
void virtio_media_debug_init_device(struct virtio_media *vv)
{
	struct dentry *root;

	root = debugfs_lookup(VIRTIO_MEDIA_DEBUGFS_DIR, NULL);
	if (IS_ERR_OR_NULL(root))
		root = debugfs_create_dir(VIRTIO_MEDIA_DEBUGFS_DIR, NULL);
	if (IS_ERR_OR_NULL(root))
		return;

	vv->debugfs = debugfs_create_dir(video_device_node_name(&vv->video_dev),
					 root);

	/*
	 * debugfs_lookup() takes a reference on the shared root dentry; drop it
	 * now. The directory itself stays alive as long as it has children, and
	 * is intentionally left in place across unbind (harmless empty dir).
	 */
	dput(root);
}

void virtio_media_debug_release_device(struct virtio_media *vv)
{
	debugfs_remove_recursive(vv->debugfs);
	vv->debugfs = NULL;
}

void virtio_media_debug_create_session(struct virtio_media *vv,
				       struct virtio_media_session *session)
{
	char name[32];

	if (IS_ERR_OR_NULL(vv->debugfs))
		return;

	scnprintf(name, sizeof(name), "instance.%u", session->id);
	session->debugfs = debugfs_create_file(name, 0444, vv->debugfs,
					       session,
					       &virtio_media_dbg_instance_fops);
	/*
	 * The only expected failure is a name clash if the host reused a
	 * session id whose old debugfs file has not been torn down yet
	 * (debugfs_remove() blocks on in-flight readers). Warn so a missing
	 * instance.<id> file is explained rather than silently absent; the
	 * ERR_PTR is harmless as debugfs_remove() ignores IS_ERR_OR_NULL.
	 */
	if (IS_ERR(session->debugfs))
		v4l2_warn(&vv->v4l2_dev,
			  "failed to create debugfs file %s: %ld\n", name,
			  PTR_ERR(session->debugfs));
}

void virtio_media_debug_remove_session(struct virtio_media_session *session)
{
	debugfs_remove(session->debugfs);
	session->debugfs = NULL;
}
