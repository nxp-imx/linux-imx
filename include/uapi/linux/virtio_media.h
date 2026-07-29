/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0+ */

/*
 * Definitions of virtio-media protocol structures.
 *
* Copyright (c) 2024-2026 Google LLC.
 */

#ifndef __VIRTIO_MEDIA_PROTOCOL_H
#define __VIRTIO_MEDIA_PROTOCOL_H

#include <linux/videodev2.h>

/*
 * Virtio protocol definition.
 */

/**
 * struct virtio_media_cmd_header - Header for all virtio-media commands.
 * @cmd: one of VIRTIO_MEDIA_CMD_*.
 * @__reserved: must be set to zero by the driver.
 *
 * This header starts all commands from the driver to the device on the
 * commandq.
 */
struct virtio_media_cmd_header {
	u32 cmd;
	u32 __reserved;
};

/**
 * struct virtio_media_resp_header - Header for all virtio-media responses.
 * @status: 0 if the command was successful, or one of the standard Linux error
 *          codes.
 * @__reserved: must be set to zero by the device.
 *
 * This header starts all responses from the device to the driver on the
 * commandq.
 */
struct virtio_media_resp_header {
	u32 status;
	u32 __reserved;
};

/**
 * VIRTIO_MEDIA_CMD_OPEN - Command for creating a new session.
 *
 * This is the equivalent of calling ``open`` on a V4L2 device node. Upon
 * success, a session id is returned which can be used to perform other
 * commands on the session, notably ioctls.
 */
#define VIRTIO_MEDIA_CMD_OPEN 1

/**
 * struct virtio_media_cmd_open - Driver command for VIRTIO_MEDIA_CMD_OPEN.
 * @hdr: header with cmd member set to VIRTIO_MEDIA_CMD_OPEN.
 */
struct virtio_media_cmd_open {
	struct virtio_media_cmd_header hdr;
};

/**
 * struct virtio_media_resp_open - Device response for VIRTIO_MEDIA_CMD_OPEN.
 * @hdr: header containing the status of the command.
 * @session_id: if &struct virtio_media_resp_header.status == 0, contains the
 *              id of the newly created session.
 * @__reserved: must be set to zero by the device.
 */
struct virtio_media_resp_open {
	struct virtio_media_resp_header hdr;
	u32 session_id;
	u32 __reserved;
};

/**
 * VIRTIO_MEDIA_CMD_CLOSE - Command for closing an active session.
 *
 * This is the equivalent of calling ``close`` on a previously opened V4L2
 * session. All resources associated with this session will be freed and the
 * session ID shall not be used again after queueing this command.
 *
 * This command does not require a response from the device.
 */
#define VIRTIO_MEDIA_CMD_CLOSE 2

/**
 * struct virtio_media_cmd_close - Driver command for VIRTIO_MEDIA_CMD_CLOSE.
 * @hdr: header with cmd member set to VIRTIO_MEDIA_CMD_CLOSE.
 * @session_id: id of the session to close.
 * @__reserved: must be set to zero by the driver.
 */
struct virtio_media_cmd_close {
	struct virtio_media_cmd_header hdr;
	u32 session_id;
	u32 __reserved;
};

/**
 * VIRTIO_MEDIA_CMD_IOCTL - Driver command for executing an ioctl.
 *
 * This command asks the device to run one of the ``VIDIOC_*`` ioctls on the
 * active session.
 *
 * The code of the ioctl is extracted from the VIDIOC_* definitions in
 * ``videodev2.h``, and consists of the second argument of the ``_IO*`` macro.
 *
 * Each ioctl has a payload, which is defined by the third argument of the
 * ``_IO*`` macro defining it. It can be writable by the driver (``_IOW``), the
 * device (``_IOR``), or both (``_IOWR``).
 *
 * If an ioctl is writable by the driver, it must be followed by a
 * driver-writable descriptor containing the payload.
 *
 * If an ioctl is writable by the device, it must be followed by a
 * device-writable descriptor of the size of the payload that the device will
 * write into.
 *
 */
#define VIRTIO_MEDIA_CMD_IOCTL 3

/**
 * struct virtio_media_cmd_ioctl - Driver command for VIRTIO_MEDIA_CMD_IOCTL.
 * @hdr: header with cmd member set to VIRTIO_MEDIA_CMD_IOCTL.
 * @session_id: id of the session to run the ioctl on.
 * @code: code of the ioctl to run.
 */
struct virtio_media_cmd_ioctl {
	struct virtio_media_cmd_header hdr;
	u32 session_id;
	u32 code;
};

/**
 * struct virtio_media_resp_ioctl - Device response for VIRTIO_MEDIA_CMD_IOCTL.
 * @hdr: header containing the status of the ioctl.
 */
struct virtio_media_resp_ioctl {
	struct virtio_media_resp_header hdr;
};

/**
 * struct virtio_media_sg_entry - Description of part of a scattered guest
 *                                memory.
 * @start: start guest address of the memory segment.
 * @len: length of this memory segment.
 * @__reserved: must be set to zero by the driver.
 */
struct virtio_media_sg_entry {
	u64 start;
	u32 len;
	u32 __reserved;
};

/**
 * VIRTIO_MEDIA_MMAP_FLAG_RW - Bit position of the VIRTIO_MEDIA_MMAP_FLAG_RW
 *                             flag.
 */
#define VIRTIO_MEDIA_MMAP_FLAG_RW 0

/**
 * VIRTIO_MEDIA_CMD_MMAP - Command for mapping a MMAP buffer into the driver's
 *                         address space.
 */
#define VIRTIO_MEDIA_CMD_MMAP 4

/**
 * struct virtio_media_cmd_mmap - Driver command for VIRTIO_MEDIA_CMD_MMAP.
 * @hdr: header with cmd member set to VIRTIO_MEDIA_CMD_MMAP.
 * @session_id: ID of the session we are mapping for.
 * @flags: combination of VIRTIO_MEDIA_MMAP_FLAG_*.
 * @offset: mem_offset field of the plane to map, as returned by
 *          VIDIOC_QUERYBUF.
 */
struct virtio_media_cmd_mmap {
	struct virtio_media_cmd_header hdr;
	u32 session_id;
	u32 flags;
	u32 offset;
};

/**
 * struct virtio_media_resp_mmap - Device response for VIRTIO_MEDIA_CMD_MMAP.
 * @hdr: header containing the status of the command.
 * @map_handle: opaque handle identifying the mapping, chosen by the device.
 * @num_pages: number of pages in the mapping.
 * @len: length of the mapping.
 * @uuid: shared-object UUID identifying this host dma-buf (host-object model:
 *	host pages + UUID). The device registers each exported host buffer with
 *	the hypervisor under this UUID; the driver wraps the buffer in a
 *	virtio-dma-buf whose get_uuid() returns it, so a consumer device can
 *	look the buffer up cross-process. All-zero if the device registered
 *	none.
 */
struct virtio_media_resp_mmap {
	struct virtio_media_resp_header hdr;
	u32 map_handle;
	u32 num_pages;
	u64 len;
	u8 uuid[16];
};

/**
 * VIRTIO_MEDIA_CMD_MUNMAP - Unmap a MMAP buffer previously mapped using
 *                           VIRTIO_MEDIA_CMD_MMAP.
 */
#define VIRTIO_MEDIA_CMD_MUNMAP 5

/**
 * struct virtio_media_cmd_munmap - Driver command for VIRTIO_MEDIA_CMD_MUNMAP.
 * @hdr: header with cmd member set to VIRTIO_MEDIA_CMD_MUNMAP.
 * @map_handle: opaque handle of the mapping to unmap (as returned in the
 *	MMAP response).
 * @num_pages: number of pages in the mapping.
 */
struct virtio_media_cmd_munmap {
	struct virtio_media_cmd_header hdr;
	u32 map_handle;
	u32 num_pages;
};

/**
 * struct virtio_media_resp_munmap - Device response for
 *                                   VIRTIO_MEDIA_CMD_MUNMAP.
 * @hdr: header containing the status of the command.
 */
struct virtio_media_resp_munmap {
	struct virtio_media_resp_header hdr;
};

/**
 * VIRTIO_MEDIA_CMD_DMABUF_ATTACH - Attach a guest dma-buf's backing to the
 * device for use as a ``V4L2_MEMORY_DMABUF`` buffer plane.
 *
 * For a ``V4L2_MEMORY_DMABUF`` queue the guest owns the buffers. Before a
 * dma-buf plane is first queued (or after it is rebound to a different
 * dma-buf), the driver shares the plane's page backing with the device and
 * sends a device-readable array of backing entries with this command,
 * together with a @resource_id the driver assigns to identify this backing.
 * The device reconstructs a local dma-buf from the backing entries and
 * remembers it under @resource_id.
 *
 * The interpretation of each backing entry is defined by the guest's memory
 * backend and the host's matching import path; the wire format is an opaque
 * ``__u32`` per entry.
 *
 * Subsequent QBUF/PREPARE_BUF of the same plane only carry @resource_id (in
 * the ``v4l2_buffer``'s ``m.fd`` / plane ``m.fd`` field); the backing entries
 * are not resent. This mirrors virtio-gpu's RESOURCE_ATTACH_BACKING + stable
 * resource id model.
 *
 * The command header is followed, in the device-readable part of the chain, by
 * a ``__u32 entries[num_entries]`` array describing the plane's page backing.
 *
 * Only dma-bufs backed by real guest pages are attached this way. dma-bufs
 * with no guest pages (virtio exported-objects, identified by
 * ``is_virtio_dma_buf()``) are rejected by the driver with ``-EINVAL`` and
 * never reach this path.
 */
#define VIRTIO_MEDIA_CMD_DMABUF_ATTACH 6

/**
 * struct virtio_media_cmd_dmabuf_attach - Driver command for
 * VIRTIO_MEDIA_CMD_DMABUF_ATTACH.
 * @hdr: header with cmd member set to VIRTIO_MEDIA_CMD_DMABUF_ATTACH.
 * @session_id: id of the session the buffer belongs to.
 * @resource_id: driver-assigned id identifying this dma-buf backing. Reused as
 *               the ``m.fd`` value in subsequent QBUF/PREPARE_BUF commands.
 * @num_entries: number of backing entries that follow this command.
 */
struct virtio_media_cmd_dmabuf_attach {
	struct virtio_media_cmd_header hdr;
	u32 session_id;
	u32 resource_id;
	u32 num_entries;
	u32 __reserved;
};

/**
 * struct virtio_media_resp_dmabuf_attach - Device response for
 * VIRTIO_MEDIA_CMD_DMABUF_ATTACH.
 * @hdr: header containing the status of the command.
 */
struct virtio_media_resp_dmabuf_attach {
	struct virtio_media_resp_header hdr;
};

/**
 * VIRTIO_MEDIA_CMD_DMABUF_DETACH - Detach a dma-buf backing previously
 * attached with VIRTIO_MEDIA_CMD_DMABUF_ATTACH.
 *
 * The device releases the local dma-buf it exported for @resource_id, letting
 * the guest release the shared page backing. Sent when the plane is
 * rebound to a different dma-buf, when a QBUF that would have used the backing
 * fails, or when the queue's buffers are freed (REQBUFS / session close).
 */
#define VIRTIO_MEDIA_CMD_DMABUF_DETACH 7

/**
 * struct virtio_media_cmd_dmabuf_detach - Driver command for
 * VIRTIO_MEDIA_CMD_DMABUF_DETACH.
 * @hdr: header with cmd member set to VIRTIO_MEDIA_CMD_DMABUF_DETACH.
 * @session_id: id of the session the backing belongs to.
 * @resource_id: id of the backing to detach (as assigned at ATTACH time).
 */
struct virtio_media_cmd_dmabuf_detach {
	struct virtio_media_cmd_header hdr;
	u32 session_id;
	u32 resource_id;
};

/**
 * struct virtio_media_resp_dmabuf_detach - Device response for
 * VIRTIO_MEDIA_CMD_DMABUF_DETACH.
 * @hdr: header containing the status of the command.
 */
struct virtio_media_resp_dmabuf_detach {
	struct virtio_media_resp_header hdr;
};

/**
 * VIRTIO_MEDIA_DMABUF_F_UUID - Flag in struct virtio_media_dmabuf_uuid marking
 * a QBUF/PREPARE_BUF plane as backed by a shared-object UUID (host-object
 * model) rather than a driver-assigned resource id (guest-pages model).
 */
#define VIRTIO_MEDIA_DMABUF_F_UUID BIT(0)

/**
 * struct virtio_media_dmabuf_uuid - Per-frame shared-object UUID footer.
 * @uuid: shared-object UUID of the host dma-buf this buffer is backed by, as
 *	obtained from the imported virtio-dma-buf's get_uuid() callback.
 * @flags: combination of VIRTIO_MEDIA_DMABUF_F_*.
 * @__reserved: must be set to zero by the driver.
 *
 * For a ``V4L2_MEMORY_DMABUF`` buffer whose planes are backed by a host dma-buf
 * shared under a UUID (host-object model), the driver appends one such footer
 * per plane to the device-readable part of a QBUF/PREPARE_BUF command's
 * descriptor chain, right after the ``v4l2_buffer`` (and its planes array, if
 * multiplanar). Each plane may be backed by a distinct host dma-buf, so each
 * carries its own UUID, mirroring the guest-pages model's per-plane ``m.fd``
 * resource ids. The device resolves each UUID to its local dma-buf fd (see the
 * consumer path) instead of treating the plane's ``m.fd`` as a resource id.
 * Absent for guest-pages dma-buf buffers, which carry resource
 * ids in ``m.fd`` instead.
 */
struct virtio_media_dmabuf_uuid {
	u8 uuid[16];
	u32 flags;
	u32 __reserved;
};

/* The values for these events are set by the virtio-media specification. */
#define VIRTIO_MEDIA_EVT_ERROR 0
#define VIRTIO_MEDIA_EVT_DQBUF 1
#define VIRTIO_MEDIA_EVT_EVENT 2

/**
 * struct virtio_media_event_header - Header for events on the eventq.
 * @event: one of VIRTIO_MEDIA_EVT_*
 * @session_id: ID of the session the event applies to.
 */
struct virtio_media_event_header {
	u32 event;
	u32 session_id;
};

/**
 * struct virtio_media_event_error - Unrecoverable device-side error.
 * @hdr: header for the event.
 * @errno: error code describing the kind of error that occurred.
 * @__reserved: must be set to zero by the device.
 *
 * Upon receiving this event, the session mentioned in the header is considered
 * corrupted and closed.
 */
struct virtio_media_event_error {
	struct virtio_media_event_header hdr;
	u32 errno;
	u32 __reserved;
};

/* This is set to VIDEO_MAX_PLANES defined in include/uapi/linux/videodev2.h.
 * It is renamed here to match the constant that is defined in the virtio-media
 * specification.
 */
#define VIRTIO_MEDIA_MAX_PLANES VIDEO_MAX_PLANES

/**
 * struct virtio_media_event_dqbuf - Dequeued buffer event.
 * @hdr: header for the event.
 * @buffer: &struct v4l2_buffer describing the buffer that has been dequeued.
 * @planes: plane information for the dequeued buffer.
 *
 * This event is used to signal that a buffer is not being used anymore by the
 * device and is returned to the driver.
 */
struct virtio_media_event_dqbuf {
	struct virtio_media_event_header hdr;
	struct v4l2_buffer buffer;
	struct v4l2_plane planes[VIRTIO_MEDIA_MAX_PLANES];
};

/**
 * struct virtio_media_event_event - V4L2 event.
 * @hdr: header for the event.
 * @event: description of the event that occurred.
 *
 * This event signals that a V4L2 event has been emitted for a session.
 */
struct virtio_media_event_event {
	struct virtio_media_event_header hdr;
	struct v4l2_event event;
};

/* Maximum size of an event. We will queue descriptors of this size on the
 * eventq.
 */
#define VIRTIO_MEDIA_EVENT_MAX_SIZE sizeof(struct virtio_media_event_dqbuf)

#endif // __VIRTIO_MEDIA_PROTOCOL_H
