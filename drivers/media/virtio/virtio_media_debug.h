/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0+ */

/*
 * virtio-media debug interface: per-session (instance) debugfs showing basic
 * state plus a ring buffer of the most recent flow events. Modelled on the
 * wave6 VPU driver's debugfs so the guest flow can be compared against
 * the host-side wave6 flow side by side -- in particular to tell whether a
 * source-change event that the host emitted was actually received and acted
 * upon by the guest.
 *
 * Copyright 2025-2026 NXP
 */

#ifndef __VIRTIO_MEDIA_DEBUG_H
#define __VIRTIO_MEDIA_DEBUG_H

#include <linux/spinlock.h>

struct virtio_media;
struct virtio_media_session;

/*
 * Flow events recorded per session. Only the significant milestones of a
 * stream are traced (state transitions, buffer (re)allocation, stream
 * start/stop, EOS, source change, errors); high-frequency per-buffer QBUF/DQBUF
 * are deliberately excluded and exposed as running counters instead. Keep this
 * list and virtio_media_flow_name[] (in virtio_media_debug.c) in sync. arg1/arg2
 * meaning is per-event and documented next to each virtio_media_record_flow()
 * call site.
 */
enum virtio_media_flow_event {
	VIRTIO_MEDIA_FLOW_NONE = 0,
	VIRTIO_MEDIA_FLOW_OPEN,
	VIRTIO_MEDIA_FLOW_CLOSE,
	VIRTIO_MEDIA_FLOW_STREAMON,
	VIRTIO_MEDIA_FLOW_STREAMOFF,
	VIRTIO_MEDIA_FLOW_REQBUFS,
	VIRTIO_MEDIA_FLOW_SUBSCRIBE_EVENT,
	VIRTIO_MEDIA_FLOW_EVENT,
	VIRTIO_MEDIA_FLOW_SOURCE_CHANGE,
	VIRTIO_MEDIA_FLOW_EOS,
	VIRTIO_MEDIA_FLOW_DECODER_CMD,
	VIRTIO_MEDIA_FLOW_ERROR,
	VIRTIO_MEDIA_FLOW_MIN_BUFFERS,
	VIRTIO_MEDIA_FLOW_MAXIMUM,
};

#define VIRTIO_MEDIA_FLOW_DEPTH 48

struct virtio_media_flow_item {
	u32 key;
	u32 arg1;
	u32 arg2;
};

/**
 * struct virtio_media_flow - Ring buffer of recent flow events for a session.
 * @flows: ring of recorded events.
 * @index: next slot to write (monotonic, taken modulo the depth).
 * @lock: protects @index against concurrent recorders (ioctl vs. event work).
 */
struct virtio_media_flow {
	struct virtio_media_flow_item flows[VIRTIO_MEDIA_FLOW_DEPTH];
	int index;
	spinlock_t lock; /* protects the flow recorder */
};

#if IS_ENABLED(CONFIG_DEBUG_FS)

void virtio_media_debug_init_device(struct virtio_media *vv);
void virtio_media_debug_release_device(struct virtio_media *vv);
void virtio_media_debug_session_init(struct virtio_media_session *session);
void virtio_media_debug_create_session(struct virtio_media *vv,
				       struct virtio_media_session *session);
void virtio_media_debug_remove_session(struct virtio_media_session *session);
void virtio_media_record_flow(struct virtio_media_session *session, u32 flow,
			      u32 arg1, u32 arg2);

#else

static inline void virtio_media_debug_init_device(struct virtio_media *vv)
{
}

static inline void virtio_media_debug_release_device(struct virtio_media *vv)
{
}

static inline void
virtio_media_debug_session_init(struct virtio_media_session *session)
{
}

static inline void
virtio_media_debug_create_session(struct virtio_media *vv,
				  struct virtio_media_session *session)
{
}

static inline void
virtio_media_debug_remove_session(struct virtio_media_session *session)
{
}

static inline void
virtio_media_record_flow(struct virtio_media_session *session, u32 flow,
			 u32 arg1, u32 arg2)
{
}

#endif /* CONFIG_DEBUG_FS */

#endif /* __VIRTIO_MEDIA_DEBUG_H */
