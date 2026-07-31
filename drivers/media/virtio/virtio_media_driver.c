// SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0+

/*
 * Virtio-media driver.
 *
* Copyright (c) 2024-2026 Google LLC.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dev_printk.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/scatterlist.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/virtio.h>
#include <linux/virtio_config.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/virtio_dma_buf.h>
#include <linux/virtio_ids.h>
#include <uapi/linux/virtio_ring.h>

#include <media/frame_vector.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-memops.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>

#include <uapi/linux/virtio_media.h>
#include "session.h"
#include "virtio_media.h"

#define VIRTIO_MEDIA_NUM_EVENT_BUFS 16

/* ID of the SHM region into which MMAP buffer will be mapped. */
#define VIRTIO_MEDIA_SHM_MMAP 0

/*
 * Name of the driver to expose to user-space.
 *
 * This is configurable because v4l2-compliance has workarounds specific to
 * some drivers. When proxying these directly from the host, this allows it to
 * apply them as needed.
 */
char *virtio_media_driver_name;
module_param_named(driver_name, virtio_media_driver_name, charp, 0660);

/*
 * Whether USERPTR buffers are allowed.
 *
 * This is disabled by default as USERPTR buffers are dangerous, but the option
 * is left to enable them if desired.
 */
bool virtio_media_allow_userptr;
module_param_named(allow_userptr, virtio_media_allow_userptr, bool, 0660);

bool virtio_media_allow_dmabuf = true;
module_param_named(allow_dmabuf, virtio_media_allow_dmabuf, bool, 0660);

/*
 * If set, command descriptor payloads are bounced through a per-device
 * cache-coherent DMA buffer and submitted to the command virtqueue as
 * premapped scatterlists. This avoids cache-coherency issues when the host
 * accesses the payloads through host mappings that bypass the guest's
 * non-coherent DMA path.
 *
 * Defaults to false, which preserves the original behaviour of referencing the
 * driver-provided (non-coherent) memory directly.
 */
static bool use_coherent_shadow_buffer = true;
module_param(use_coherent_shadow_buffer, bool, 0660);

/**
 * virtio_media_session_alloc - Allocate a new session.
 * @vv: virtio-media device the session belongs to.
 * @id: ID of the session.
 * @nonblocking_dequeue: whether dequeuing of buffers should be blocking or
 * not.
 *
 * The ``id`` and ``list`` fields must still be set by the caller.
 */
static struct virtio_media_session *
virtio_media_session_alloc(struct virtio_media *vv, u32 id,
			   struct file *file)
{
	struct virtio_media_session *session;
	int i;
	int ret;

	session = kzalloc_obj(*session, GFP_KERNEL);
	if (!session)
		goto err_session;

	if (vv->use_coherent)
		session->shadow_buf = dma_alloc_coherent(vv->cmd_dma_dev,
							 VIRTIO_SHADOW_BUF_SIZE,
							 &session->shadow_buf_dma,
							 GFP_KERNEL);
	else
		session->shadow_buf = kzalloc(VIRTIO_SHADOW_BUF_SIZE,
					      GFP_KERNEL);
	if (!session->shadow_buf)
		goto err_shadow_buf;

	ret = sg_alloc_table(&session->command_sgs, DESC_CHAIN_MAX_LEN,
			     GFP_KERNEL);
	if (ret)
		goto err_payload_sgs;

	session->id = id;
	session->nonblocking_dequeue = file->f_flags & O_NONBLOCK;

	INIT_LIST_HEAD(&session->list);
	v4l2_fh_init(&session->fh, &vv->video_dev);
	virtio_media_session_fh_add(session, file);

	virtio_media_debug_session_init(session);

	for (i = 0; i <= VIRTIO_MEDIA_LAST_QUEUE; i++) {
		INIT_LIST_HEAD(&session->queues[i].pending_dqbufs);
		INIT_LIST_HEAD(&session->queues[i].host_mappings);
		INIT_LIST_HEAD(&session->queues[i].dmabuf_imports);
	}
	mutex_init(&session->queues_lock);

	init_waitqueue_head(&session->dqbuf_wait);

	mutex_lock(&vv->sessions_lock);
	list_add_tail(&session->list, &vv->sessions);
	mutex_unlock(&vv->sessions_lock);

	return session;

err_payload_sgs:
	if (vv->use_coherent)
		dma_free_coherent(vv->cmd_dma_dev, VIRTIO_SHADOW_BUF_SIZE,
				  session->shadow_buf, session->shadow_buf_dma);
	else
		kfree(session->shadow_buf);
err_shadow_buf:
	kfree(session);
err_session:
	return ERR_PTR(-ENOMEM);
}

/**
 * struct virtio_media_coherent_cmd - Helper to stage a control command through
 * the device's coherent DMA buffers so it can be submitted premapped.
 * @vv: virtio-media device in use.
 * @cmd_off: current write offset into @vv->ctrl_cmd (device-readable buffers).
 * @resp_off: current write offset into @vv->ctrl_resp (device-writable buffers).
 * @nr_in: number of device-writable buffers staged so far.
 *
 * Only valid while @vv->vlock is held (the coherent ctrl buffers are shared per
 * device).
 */
struct virtio_media_coherent_cmd {
	struct virtio_media *vv;
	size_t cmd_off;
	size_t resp_off;
	unsigned int nr_in;
	/* Track device-writable buffers so responses can be copied back. */
	struct {
		void *dst;
		size_t off;
		size_t len;
	} in[3];
};

static void virtio_media_coherent_cmd_init(struct virtio_media_coherent_cmd *c,
					   struct virtio_media *vv)
{
	c->vv = vv;
	c->cmd_off = 0;
	c->resp_off = 0;
	c->nr_in = 0;
}

/* Stage a device-readable buffer (copied into the coherent cmd buffer). */
static int virtio_media_coherent_add_out(struct virtio_media_coherent_cmd *c,
					 struct scatterlist *sg, void *buf,
					 size_t len)
{
	struct virtio_media *vv = c->vv;

	if (c->cmd_off + len > PAGE_SIZE)
		return -ENOSPC;

	memcpy(vv->ctrl_cmd + c->cmd_off, buf, len);
	sg_init_table(sg, 1);
	sg_set_buf(sg, vv->ctrl_cmd + c->cmd_off, len);
	sg_dma_address(sg) = vv->ctrl_cmd_dma + c->cmd_off;
	sg_dma_len(sg) = len;
	c->cmd_off += len;

	return 0;
}

/* Stage a device-writable buffer (space reserved in the coherent resp buffer). */
static int virtio_media_coherent_add_in(struct virtio_media_coherent_cmd *c,
					struct scatterlist *sg, void *dst,
					size_t len)
{
	struct virtio_media *vv = c->vv;

	if (c->resp_off + len > PAGE_SIZE)
		return -ENOSPC;
	if (c->nr_in >= ARRAY_SIZE(c->in))
		return -ENOSPC;

	sg_init_table(sg, 1);
	sg_set_buf(sg, vv->ctrl_resp + c->resp_off, len);
	sg_dma_address(sg) = vv->ctrl_resp_dma + c->resp_off;
	sg_dma_len(sg) = len;

	c->in[c->nr_in].dst = dst;
	c->in[c->nr_in].off = c->resp_off;
	c->in[c->nr_in].len = len;
	c->nr_in++;
	c->resp_off += len;

	return 0;
}

/* Copy device-written responses back from the coherent resp buffer. */
static void
virtio_media_coherent_retrieve(struct virtio_media_coherent_cmd *c)
{
	struct virtio_media *vv = c->vv;
	unsigned int i;

	for (i = 0; i < c->nr_in; i++)
		memcpy(c->in[i].dst, vv->ctrl_resp + c->in[i].off,
		       c->in[i].len);
}

/**
 * virtio_media_session_send_close - Notify the host that a session is closing.
 * @vv: virtio-media device the session belongs to.
 * @session: session being closed.
 *
 * Sends the ``VIRTIO_MEDIA_CMD_CLOSE`` command to the device.
 */
static void virtio_media_session_send_close(struct virtio_media *vv,
					    struct virtio_media_session *session)
{
	struct virtio_media_cmd_close *cmd_close = &session->cmd.close;
	struct scatterlist cmd_sg = {};
	struct scatterlist *sgs[1] = { &cmd_sg };
	struct virtio_media_coherent_cmd cc;
	int ret;

	mutex_lock(&vv->vlock);

	cmd_close->hdr.cmd = VIRTIO_MEDIA_CMD_CLOSE;
	cmd_close->session_id = session->id;

	if (vv->use_coherent) {
		virtio_media_coherent_cmd_init(&cc, vv);
		ret = virtio_media_coherent_add_out(&cc, &cmd_sg, cmd_close,
						    sizeof(*cmd_close));
		if (ret) {
			mutex_unlock(&vv->vlock);
			v4l2_err(&vv->v4l2_dev,
				 "failed to stage CLOSE command: %d\n", ret);
			return;
		}
	} else {
		sg_set_buf(&cmd_sg, cmd_close, sizeof(*cmd_close));
		sg_mark_end(&cmd_sg);
	}

	ret = virtio_media_send_command(vv, sgs, 1, 0, vv->use_coherent, 0,
					NULL);
	mutex_unlock(&vv->vlock);
	if (ret < 0)
		v4l2_err(&vv->v4l2_dev, "failed to send CLOSE command: %d\n",
			 ret);
}

/**
 * virtio_media_session_free - Free all resources of a session.
 * @vv: virtio-media device the session belongs to.
 * @session: session to destroy.
 * @notify_host: whether to send VIRTIO_MEDIA_CMD_CLOSE to the host.
 *
 * All the resources of @session, as well as the backing memory of @session
 * itself, are freed.
 *
 * The host mappings are released (their owning-queue reference dropped) and the
 * deferred release worker is flushed so every MUNMAP completes *before*
 * CMD_CLOSE, so that by the time the host acts on CLOSE (and releases the
 * underlying V4L2 buffers) the guest has already unmapped every backend
 * entry. When called from device removal the host is gone, so
 * @notify_host is false and neither the flush nor CLOSE is done.
 */
static void virtio_media_session_free(struct virtio_media *vv,
				      struct virtio_media_session *session,
				      bool notify_host)
{
	int i;

	mutex_lock(&vv->sessions_lock);
	list_del(&session->list);
	mutex_unlock(&vv->sessions_lock);

	virtio_media_debug_remove_session(session);

	/*
	 * Drop the owning-queue reference on every host mapping. The actual
	 * teardown (unmapping backend entries and sending MUNMAP) is deferred to
	 * vv->mapping_release_work. Free them while session->fh.vdev is still
	 * valid, since the helper derives the device from it. Must run before
	 * v4l2_fh_exit().
	 */
	virtio_media_free_session_host_mappings(session);

	/*
	 * CLOSE must be sent only after every backend entry has actually been
	 * unmapped (MUNMAP), otherwise the host could release the underlying
	 * V4L2 buffers while the guest still has backing mapped. Since teardown
	 * is deferred to the release worker, flush it here before sending
	 * CLOSE so all MUNMAPs have completed first.
	 *
	 * On device removal (!notify_host) CLOSE is not sent. The MUNMAPs still
	 * scheduled by the puts above are instead drained by the caller
	 * (virtio_media_remove() cancel_work_sync()s the release worker after
	 * freeing every session, before resetting the device).
	 */
	if (notify_host) {
		flush_work(&vv->mapping_release_work);
		virtio_media_session_send_close(vv, session);
	}

	/*
	 * Release any dma-buf imports. Unmapping releases the backing the
	 * backend was given, so it must happen only after the backend can no
	 * longer be using it: on the notify_host path CLOSE (sent
	 * above) has made the backend drop its exported dma-bufs; on device
	 * removal the backend is gone. Safe in both cases.
	 */
	virtio_media_free_session_dmabuf_imports(session);

	virtio_media_session_fh_del(session);
	v4l2_fh_exit(&session->fh);

	sg_free_table(&session->command_sgs);

	for (i = 0; i <= VIRTIO_MEDIA_LAST_QUEUE; i++)
		virtio_media_free_queue_buffers(&session->queues[i]);

	if (vv->use_coherent)
		dma_free_coherent(vv->cmd_dma_dev, VIRTIO_SHADOW_BUF_SIZE,
				  session->shadow_buf, session->shadow_buf_dma);
	else
		kfree(session->shadow_buf);
	kfree(session);
}

/**
 * virtio_media_find_session_locked - Lookup a session with a given ID.
 * @vv: virtio-media device to lookup the session from.
 * @id: ID of the session to lookup.
 *
 * The caller must hold @vv->sessions_lock and must keep holding it for as long
 * as it uses the returned session, otherwise the session may be freed
 * concurrently by virtio_media_session_free().
 */
static struct virtio_media_session *
virtio_media_find_session_locked(struct virtio_media *vv, u32 id)
{
	struct list_head *p;

	lockdep_assert_held(&vv->sessions_lock);

	list_for_each(p, &vv->sessions) {
		struct virtio_media_session *s =
			list_entry(p, struct virtio_media_session, list);
		if (s->id == id)
			return s;
	}

	return NULL;
}

/**
 * commandq_callback: Callback for the command queue.
 * @queue: command virtqueue.
 *
 * This just wakes up the thread that was waiting on the command to complete.
 */
static void commandq_callback(struct virtqueue *queue)
{
	unsigned int len;
	struct virtio_media_cmd_callback_param *param;

process_bufs:
	while ((param = virtqueue_get_buf(queue, &len))) {
		struct virtio_media *vv = param->vv;

		/*
		 * @param points at vv->cmd_cb, whose lifetime is tied to the
		 * device, so this dereference is always safe even for a
		 * response that arrives after its waiter timed out. If the
		 * waiter did give up (cmd_abandoned), this is an orphaned
		 * response: consume the buffer to reclaim the slot, but do not
		 * touch @done or wake anyone, as no one is waiting on it.
		 */
		if (READ_ONCE(vv->cmd_abandoned)) {
			WRITE_ONCE(vv->cmd_abandoned, false);
			continue;
		}

		param->resp_len = len;
		/* Pairs with the wait_event_timeout() load of @done. */
		smp_store_release(&param->done, true);
		wake_up(&vv->wq);
	}

	if (!virtqueue_enable_cb(queue)) {
		virtqueue_disable_cb(queue);
		goto process_bufs;
	}
}

/**
 * virtio_media_kick_command - send a command to the commandq.
 * @vv: virtio-media device in use.
 * @sgs: descriptor chain to send.
 * @out_sgs: number of device-readable descriptors in @sgs.
 * @in_sgs: number of device-writable descriptors in @sgs.
 * @premapped: if true, @sgs already carry valid DMA addresses (via
 * sg_dma_address()/sg_dma_len()) and must be submitted without virtio core
 * performing any DMA mapping.
 * @resp_len: output parameter. Upon success, contains the size of the response
 * in bytes.
 *
 */
static int virtio_media_kick_command(struct virtio_media *vv,
				     struct scatterlist **sgs,
				     const size_t out_sgs, const size_t in_sgs,
				     bool premapped, size_t *resp_len)
{
	struct virtio_media_cmd_callback_param *cb_param = &vv->cmd_cb;
	struct virtio_media_resp_header *resp_header;
	int ret;

	/*
	 * Commands are serialized by @vlock, so there is at most one in-flight
	 * command and it is safe to reuse the device-global callback param.
	 * Reset it (and clear any stale abandoned flag) before submitting.
	 */
	cb_param->vv = vv;
	cb_param->resp_len = 0;
	WRITE_ONCE(vv->cmd_abandoned, false);
	/* Publish the reset @done=false before the buffer is made visible. */
	smp_store_release(&cb_param->done, false);

	if (premapped)
		ret = virtqueue_add_sgs_premapped(vv->commandq, sgs, out_sgs,
						  in_sgs, cb_param,
						  GFP_ATOMIC);
	else
		ret = virtqueue_add_sgs(vv->commandq, sgs, out_sgs, in_sgs,
					cb_param, GFP_ATOMIC);
	if (ret) {
		v4l2_err(&vv->v4l2_dev,
			 "failed to add sgs to command virtqueue\n");
		return ret;
	}

	if (!virtqueue_kick(vv->commandq)) {
		v4l2_err(&vv->v4l2_dev, "failed to kick command virtqueue\n");
		return -EINVAL;
	}

	/*
	 * Wait for the response. The acquire load of @done pairs with the
	 * release store in commandq_callback() so that @resp_len (written
	 * before that store) is visible once @done reads true.
	 */
	ret = wait_event_timeout(vv->wq, smp_load_acquire(&cb_param->done),
				 5 * HZ);
	if (ret == 0) {
		/*
		 * The command timed out but its buffer is still owned by the
		 * virtqueue. Mark it abandoned so that a late response is
		 * discarded by commandq_callback() instead of waking a caller
		 * that has already moved on (which previously dereferenced a
		 * freed on-stack param and crashed).
		 */
		WRITE_ONCE(vv->cmd_abandoned, true);
		v4l2_err(&vv->v4l2_dev,
			 "timed out waiting for response to command\n");
		return -ETIMEDOUT;
	}

	if (resp_len)
		*resp_len = cb_param->resp_len;

	if (in_sgs > 0) {
		/*
		 * If we expect a response, make sure we have at least a
		 * response header - anything shorter is invalid.
		 */
		if (cb_param->resp_len < sizeof(*resp_header)) {
			v4l2_err(&vv->v4l2_dev,
				 "received response header is too short\n");
			return -EINVAL;
		}

		resp_header = sg_virt(sgs[out_sgs]);
		if (resp_header->status)
			/* Host returns a positive error code. */
			return -resp_header->status;
	}

	return 0;
}

/**
 * virtio_media_ioctl_error_is_benign() - Tell whether a failed ioctl is normal
 * control flow rather than a device error.
 * @err: negative errno returned by the host for a VIRTIO_MEDIA_CMD_IOCTL.
 *
 * Many V4L2 ioctls report routine, expected conditions by failing:
 * enumeration ioctls (VIDIOC_ENUM*, VIDIOC_QUERYMENU) return -EINVAL to mark
 * the end of a list, VIDIOC_QUERY(_EXT_)CTRL returns -EINVAL for control ids a
 * device does not support, the host returns -ENOTTY for ioctls it does not
 * implement, and a drained decoder returns -EPIPE. Userspace probes these in
 * loops, so logging each failure at error level floods the kernel log for no
 * reason. Such codes are logged at debug level instead.
 *
 * Returns true if @err is one of those expected codes.
 */
static bool virtio_media_ioctl_error_is_benign(int err)
{
	switch (err) {
	case -EINVAL:	/* enumeration end; unsupported QUERY(_EXT_)CTRL id */
	case -ENOTTY:	/* ioctl not implemented by the host */
	case -ENODATA:	/* no data for this ioctl (e.g. empty enumeration) */
	case -EPIPE:	/* end of stream (e.g. decoder drain) */
		return true;
	default:
		return false;
	}
}

/**
 * virtio_media_send_command - Send a command to the device and wait for its
 * response.
 * @vv: virtio-media device in use.
 * @sgs: descriptor chain to send.
 * @out_sgs: number of device-readable descriptors in @sgs.
 * @in_sgs: number of device-writable descriptors in @sgs.
 * @minimum_resp_len: minimum length of the response expected by the caller
 * when the command is successful. Anything shorter than that will result in
 * ``-EINVAL`` being returned.
 * @resp_len: output parameter. Upon success, contains the size of the response
 * in bytes.
 */
int virtio_media_send_command(struct virtio_media *vv, struct scatterlist **sgs,
			      const size_t out_sgs, const size_t in_sgs,
			      bool premapped, size_t minimum_resp_len,
			      size_t *resp_len)
{
	size_t local_resp_len = resp_len ? *resp_len : 0;
	int ret = virtio_media_kick_command(vv, sgs, out_sgs, in_sgs,
					    premapped, &local_resp_len);
	if (resp_len)
		*resp_len = local_resp_len;

	/*
	 * If the host could not process the command, there is no valid
	 * response.
	 */
	if (ret < 0) {
		const struct virtio_media_cmd_header *cmd_hdr =
			out_sgs ? sg_virt(sgs[0]) : NULL;
		u32 cmd = cmd_hdr ? cmd_hdr->cmd : 0;

		if (cmd == VIRTIO_MEDIA_CMD_IOCTL) {
			const struct virtio_media_cmd_ioctl *cmd_ioctl =
				(const struct virtio_media_cmd_ioctl *)cmd_hdr;

			/*
			 * Many ioctl failures are normal control flow (end of
			 * an enumeration, an unsupported control query, an
			 * unimplemented ioctl). Log those at debug level so the
			 * error log is not flooded by routine userspace probes;
			 * only genuinely unexpected failures stay at error
			 * level.
			 */
			if (virtio_media_ioctl_error_is_benign(ret))
				dev_dbg(vv->v4l2_dev.dev,
					"ioctl returned expected error (cmd = %u, ioctl code = %u), ret = %d\n",
					cmd, cmd_ioctl->code, ret);
			else
				v4l2_err(&vv->v4l2_dev,
					 "fail to send command (cmd = %u, ioctl code = %u), ret = %d\n",
					 cmd, cmd_ioctl->code, ret);
		} else {
			v4l2_err(&vv->v4l2_dev,
				 "fail to send command (cmd = %u), ret = %d\n",
				 cmd, ret);
		}
		return ret;
	}

	/* Make sure the host wrote a complete reply. */
	if (local_resp_len < minimum_resp_len) {
		v4l2_err(&vv->v4l2_dev,
			 "received response is too short: received %zu, expected at least %zu\n",
			 local_resp_len, minimum_resp_len);
		return -EINVAL;
	}

	return 0;
}

/**
 * virtio_media_send_event_buffer() - Sends an event buffer to the host so it
 * can return it with an event.
 * @vv: virtio-media device in use.
 * @event_buffer: pointer to the event buffer to send to the device.
 */
static int virtio_media_send_event_buffer(struct virtio_media *vv,
					  void *event_buffer)
{
	struct scatterlist *sgs[1], vresp;
	int ret;

	sg_init_one(&vresp, event_buffer, VIRTIO_MEDIA_EVENT_MAX_SIZE);
	sgs[0] = &vresp;

	ret = virtqueue_add_sgs(vv->eventq, sgs, 0, 1, event_buffer,
				GFP_ATOMIC);
	if (ret) {
		v4l2_err(&vv->v4l2_dev,
			 "failed to add sgs to event virtqueue\n");
		return ret;
	}

	if (!virtqueue_kick(vv->eventq)) {
		v4l2_err(&vv->v4l2_dev, "failed to kick event virtqueue\n");
		return -EINVAL;
	}

	return 0;
}

/**
 * eventq_callback() - Callback for the event queue.
 * @queue: event virtqueue.
 *
 * This just schedules for event work to be run.
 */
static void eventq_callback(struct virtqueue *queue)
{
	struct virtio_media *vv = queue->vdev->priv;

	schedule_work(&vv->eventq_work);
}

/**
 * virtio_media_process_dqbuf_event() - Process a dequeued event for a session.
 * @vv: virtio-media device in use.
 * @session: session the event is addressed to.
 * @dqbuf_evt: the dequeued event to process.
 *
 * Invalid events are ignored with an error log.
 */
static void
virtio_media_process_dqbuf_event(struct virtio_media *vv,
				 struct virtio_media_session *session,
				 struct virtio_media_event_dqbuf *dqbuf_evt)
{
	struct virtio_media_buffer *dqbuf;
	const enum v4l2_buf_type queue_type = dqbuf_evt->buffer.type;
	struct virtio_media_queue_state *queue;
	typeof(dqbuf->buffer.m) buffer_m;
	typeof(dqbuf->buffer.m.planes[0].m) plane_m;
	int i;

	if (queue_type >= ARRAY_SIZE(session->queues)) {
		v4l2_err(&vv->v4l2_dev,
			 "unmanaged queue %d passed to dqbuf event",
			 dqbuf_evt->buffer.type);
		return;
	}
	queue = &session->queues[queue_type];

	/*
	 * Hold queues_lock across the whole lookup and update. It serialises
	 * against REQBUFS/CREATE_BUFS, which swap queue->buffers and free the
	 * old buffers/pointer array under the same lock: without it this worker
	 * could read a stale queue->buffers pointer (or a stale allocated_bufs)
	 * and dereference a freed buffer or index out of bounds.
	 */
	scoped_guard(mutex, &session->queues_lock) {
		if (dqbuf_evt->buffer.index >= queue->allocated_bufs) {
			v4l2_err(&vv->v4l2_dev,
				 "invalid buffer ID %d for queue %d in dqbuf event",
				 dqbuf_evt->buffer.index, dqbuf_evt->buffer.type);
			return;
		}

		dqbuf = queue->buffers[dqbuf_evt->buffer.index];

		/*
		 * Preserve the 'm' union that was passed to us during QBUF so
		 * userspace gets back the information it submitted.
		 */
		buffer_m = dqbuf->buffer.m;
		memcpy(&dqbuf->buffer, &dqbuf_evt->buffer, sizeof(dqbuf->buffer));
		dqbuf->buffer.m = buffer_m;
		if (V4L2_TYPE_IS_MULTIPLANAR(dqbuf->buffer.type)) {
			if (dqbuf->buffer.length > VIDEO_MAX_PLANES) {
				v4l2_err(&vv->v4l2_dev,
					 "invalid number of planes received from host for a multiplanar buffer\n");
				return;
			}
			for (i = 0; i < dqbuf->buffer.length; i++) {
				plane_m = dqbuf->planes[i].m;
				memcpy(&dqbuf->planes[i], &dqbuf_evt->planes[i],
				       sizeof(struct v4l2_plane));
				dqbuf->planes[i].m = plane_m;
			}
		}

		/* Set the DONE flag as the buffer is waiting for being dequeued. */
		dqbuf->buffer.flags |= V4L2_BUF_FLAG_DONE;

		list_add_tail(&dqbuf->list, &queue->pending_dqbufs);
		queue->queued_bufs -= 1;
		queue->dqbuf_count += 1;
	}

	wake_up(&session->dqbuf_wait);
}

/**
 * virtio_media_process_events() - Process all pending events on a device.
 * @vv: device which pending events we want to process.
 *
 * Retrieves all pending events on @vv's event queue and dispatch them to their
 * corresponding session.
 *
 * Invalid events are ignored with an error log.
 */
void virtio_media_process_events(struct virtio_media *vv)
{
	struct virtio_media_event_error *error_evt;
	struct virtio_media_event_dqbuf *dqbuf_evt;
	struct virtio_media_event_event *event_evt;
	struct virtio_media_session *session;
	struct virtio_media_event_header *evt;
	unsigned int len;

	mutex_lock(&vv->events_lock);

process_bufs:
	while ((evt = virtqueue_get_buf(vv->eventq, &len))) {
		/* Make sure we received enough data */
		if (len < sizeof(*evt)) {
			v4l2_err(&vv->v4l2_dev,
				 "event is too short: got %u, expected at least %zu\n",
				 len, sizeof(*evt));
			goto end_of_event;
		}

		/*
		 * Look up the session and process the event while holding
		 * sessions_lock, so the session cannot be freed by a
		 * concurrent close (virtio_media_session_free() takes the
		 * same lock to unlink the session before freeing it). Using
		 * the session outside this lock would be a use-after-free.
		 */
		mutex_lock(&vv->sessions_lock);
		session = virtio_media_find_session_locked(vv, evt->session_id);
		if (!session) {
			mutex_unlock(&vv->sessions_lock);
			v4l2_err(&vv->v4l2_dev, "cannot find session %d\n",
				 evt->session_id);
			goto end_of_event;
		}

		switch (evt->event) {
		case VIRTIO_MEDIA_EVT_ERROR:
			if (len < sizeof(*error_evt)) {
				v4l2_err(&vv->v4l2_dev,
					 "error event is too short: got %u, expected %zu\n",
					 len, sizeof(*error_evt));
				break;
			}
			error_evt = (struct virtio_media_event_error *)evt;
			v4l2_err(&vv->v4l2_dev,
				 "received error %d for session %d",
				 error_evt->errno, error_evt->hdr.session_id);
			virtio_media_record_flow(session,
						 VIRTIO_MEDIA_FLOW_ERROR,
						 error_evt->errno, 0);
			/*
			 * Flag the session as errored and wake up any poller so
			 * it can report EPOLLERR. The session is torn down when
			 * user-space closes the file descriptor.
			 */
			session->error = true;
			wake_up(&session->dqbuf_wait);
			break;

		/*
		 * Dequeued buffer: put it into the right queue so user-space
		 * can dequeue it.
		 */
		case VIRTIO_MEDIA_EVT_DQBUF:
			if (len < sizeof(*dqbuf_evt)) {
				v4l2_err(&vv->v4l2_dev,
					 "dqbuf event is too short: got %u, expected %zu\n",
					 len, sizeof(*dqbuf_evt));
				break;
			}
			dqbuf_evt = (struct virtio_media_event_dqbuf *)evt;
			virtio_media_process_dqbuf_event(vv, session,
							 dqbuf_evt);
			break;

		case VIRTIO_MEDIA_EVT_EVENT:
			if (len < sizeof(*event_evt)) {
				v4l2_err(&vv->v4l2_dev,
					 "session event is too short: got %u expected %zu\n",
					 len, sizeof(*event_evt));
				break;
			}

			event_evt = (struct virtio_media_event_event *)evt;
			/*
			 * Record the forwarded V4L2 event, calling out source
			 * changes specifically: comparing this against the
			 * host-side wave6 flow shows whether a source-change the
			 * host emitted actually reached and was acted upon by
			 * the guest.
			 */
			if (event_evt->event.type == V4L2_EVENT_SOURCE_CHANGE)
				virtio_media_record_flow(session,
					VIRTIO_MEDIA_FLOW_SOURCE_CHANGE,
					event_evt->event.u.src_change.changes,
					0);
			else
				virtio_media_record_flow(session,
					VIRTIO_MEDIA_FLOW_EVENT,
					event_evt->event.type, 0);
			v4l2_event_queue_fh(&session->fh, &event_evt->event);
			break;

		default:
			v4l2_err(&vv->v4l2_dev, "unknown event type %d\n",
				 evt->event);
			break;
		}
		mutex_unlock(&vv->sessions_lock);

end_of_event:
		virtio_media_send_event_buffer(vv, evt);
	}

	if (!virtqueue_enable_cb(vv->eventq)) {
		virtqueue_disable_cb(vv->eventq);
		goto process_bufs;
	}

	mutex_unlock(&vv->events_lock);
}

static void virtio_media_event_work(struct work_struct *work)
{
	struct virtio_media *vv =
		container_of(work, struct virtio_media, eventq_work);

	virtio_media_process_events(vv);
}

/**
 * virtio_media_device_open() - Create a new session from an opened file.
 * @file: opened file for the session.
 */
static int virtio_media_device_open(struct file *file)
{
	struct video_device *video_dev = video_devdata(file);
	struct virtio_media *vv = to_virtio_media(video_dev);
	struct virtio_media_cmd_open *cmd_open = &vv->cmd.open;
	struct virtio_media_resp_open *resp_open = &vv->resp.open;
	struct scatterlist cmd_sg = {}, resp_sg = {};
	struct scatterlist *sgs[2] = { &cmd_sg, &resp_sg };
	struct virtio_media_coherent_cmd cc;
	struct virtio_media_session *session;
	u32 session_id;
	int ret;

	mutex_lock(&vv->vlock);

	cmd_open->hdr.cmd = VIRTIO_MEDIA_CMD_OPEN;

	if (vv->use_coherent) {
		virtio_media_coherent_cmd_init(&cc, vv);
		ret = virtio_media_coherent_add_out(&cc, &cmd_sg, cmd_open,
						    sizeof(*cmd_open));
		if (!ret)
			ret = virtio_media_coherent_add_in(&cc, &resp_sg,
							   resp_open,
							   sizeof(*resp_open));
		if (ret) {
			mutex_unlock(&vv->vlock);
			return ret;
		}
	} else {
		sg_set_buf(&cmd_sg, cmd_open, sizeof(*cmd_open));
		sg_mark_end(&cmd_sg);

		sg_set_buf(&resp_sg, resp_open, sizeof(*resp_open));
		sg_mark_end(&resp_sg);
	}

	ret = virtio_media_send_command(vv, sgs, 1, 1, vv->use_coherent,
					sizeof(*resp_open), NULL);
	if (!ret && vv->use_coherent)
		virtio_media_coherent_retrieve(&cc);
	session_id = resp_open->session_id;
	mutex_unlock(&vv->vlock);
	if (ret < 0)
		return ret;

	session = virtio_media_session_alloc(vv, session_id, file);
	if (IS_ERR(session))
		return PTR_ERR(session);

	virtio_media_debug_create_session(vv, session);
	virtio_media_record_flow(session, VIRTIO_MEDIA_FLOW_OPEN, session_id, 0);

	file->private_data = &session->fh;

	return 0;
}

/**
 * virtio_media_device_close() - Close a previously opened session.
 * @file: file of the session to close.
 *
 * This sends to ``VIRTIO_MEDIA_CMD_CLOSE`` command to the device, and close
 * the session on the driver side.
 */
static int virtio_media_device_close(struct file *file)
{
	struct video_device *video_dev = video_devdata(file);
	struct virtio_media *vv = to_virtio_media(video_dev);
	struct virtio_media_session *session =
		fh_to_session(file->private_data);

	/*
	 * The session (and its host mappings) is torn down here. Every dma-buf
	 * exported via VIDIOC_EXPBUF holds a reference to this file via
	 * get_file(), so device close - and therefore this teardown - only
	 * runs after the open fd is closed AND every exported dma-buf has been
	 * released. By then no user mapping of any backing page remains, so it
	 * is safe to unmap the backend entries and notify the host with CMD_CLOSE.
	 */
	virtio_media_session_free(vv, session, true);
	return 0;
}

/**
 * virtio_media_device_poll() - Poll logic for a virtio-media device.
 * @file: file of the session to poll.
 * @wait: poll table to wait on.
 */
static __poll_t virtio_media_device_poll(struct file *file, poll_table *wait)
{
	struct virtio_media_session *session =
		fh_to_session(file->private_data);
	enum v4l2_buf_type capture_type =
		session->uses_mplane ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE :
				       V4L2_BUF_TYPE_VIDEO_CAPTURE;
	enum v4l2_buf_type output_type =
		session->uses_mplane ? V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE :
				       V4L2_BUF_TYPE_VIDEO_OUTPUT;
	struct virtio_media_queue_state *capture_queue =
		&session->queues[capture_type];
	struct virtio_media_queue_state *output_queue =
		&session->queues[output_type];
	__poll_t req_events = poll_requested_events(wait);
	__poll_t rc = 0;

	poll_wait(file, &session->dqbuf_wait, wait);
	poll_wait(file, &session->fh.wait, wait);

	scoped_guard(mutex, &session->queues_lock) {
		if (session->error) {
			rc |= EPOLLERR;
		} else {
			if (req_events & (EPOLLOUT | EPOLLWRNORM | EPOLLIN | EPOLLRDNORM)) {
				/*
				 * Signal readable on the CAPTURE queue either when a
				 * buffer is pending, or after the LAST buffer of a
				 * drain has been dequeued (is_capture_last). In the
				 * latter case pending_dqbufs is empty but the client
				 * must still be woken so its DQBUF returns -EPIPE and
				 * the drain / dynamic-resolution-change sequence can
				 * complete. This mirrors vb2_core_poll(), which returns
				 * EPOLLIN|EPOLLRDNORM when q->last_buffer_dequeued is
				 * set even though the done_list is empty. Without it a
				 * source change (implicit drain) can stall the pipeline
				 * during seek/trick play: the client dequeues the
				 * (possibly empty) LAST buffer, goes back to poll(), and
				 * blocks forever because pending_dqbufs is empty and the
				 * SOURCE_CHANGE event may already have been consumed.
				 */
				if (!list_empty(&capture_queue->pending_dqbufs) ||
				    capture_queue->is_capture_last)
					rc |= EPOLLIN | EPOLLRDNORM;
				if (!list_empty(&output_queue->pending_dqbufs))
					rc |= EPOLLOUT | EPOLLWRNORM;
			}
		}
	}

	if (v4l2_event_pending(&session->fh))
		rc |= EPOLLPRI;

	return rc;
}

/*
 * Host-mapping subsystem for zero-copy MMAP buffer sharing with the host.
 *
 * A host mapping is created per (type, index, plane) at QUERYBUF time and lives
 * until the queue is torn down by REQBUFS or the session is closed. The
 * mapping is decoupled from any userspace VMA: closing a VMA only tears down
 * the userspace PTEs, while the underlying host-mappingped pages persist so that
 * seek (which re-mmaps buffers without re-allocating them) keeps working and
 * the host keeps ownership of the buffer.
 *
 * Locking: the host_mappings list is created/destroyed under vv->vlock (QUERYBUF,
 * REQBUFS, session close) and additionally walked under queues_lock by the
 * paths that do not run under vv->vlock. Callers of the find helpers below
 * must hold at least one of those locks; any path that then dereferences the
 * returned map while it might race a REQBUFS/close teardown must hold
 * vv->vlock (or an explicit map reference) so the map and its pages cannot
 * be freed under it.
 */

struct virtio_media_host_mapping *
virtio_media_find_host_mapping(struct virtio_media_session *session, u32 type,
			    u32 index, u32 plane)
{
	struct virtio_media_queue_state *queue;
	struct virtio_media_host_mapping *map;

	if (type > VIRTIO_MEDIA_LAST_QUEUE)
		return NULL;

	queue = &session->queues[type];
	list_for_each_entry(map, &queue->host_mappings, list) {
		if (map->index == index && map->plane == plane)
			return map;
	}
	return NULL;
}

/* Find a host mapping in any queue of the session by its v4l2 mem_offset. */
struct virtio_media_host_mapping *
virtio_media_find_host_mapping_by_offset(struct virtio_media_session *session,
				      u32 offset)
{
	struct virtio_media_host_mapping *map;
	int i;

	for (i = 0; i <= VIRTIO_MEDIA_LAST_QUEUE; i++) {
		list_for_each_entry(map, &session->queues[i].host_mappings, list) {
			if (map->v4l2_offset == offset)
				return map;
		}
	}
	return NULL;
}

/*
 * Cache maintenance for host-mapped buffers.
 *
 * The buffers are host pages shared with the guest and mapped into the guest
 * with a Normal (cacheable) memory type. The host-side hardware (e.g. the VPU)
 * is a non-coherent DMA master, so the guest must explicitly maintain the CPU
 * caches around ownership transfer:
 *  - before QBUF (guest -> device): clean the CPU caches so the data the guest
 *    wrote is visible to the device in DRAM (DMA_TO_DEVICE).
 *  - after DQBUF (device -> guest): invalidate the CPU caches so the guest
 *    reads the data the device wrote to DRAM (DMA_FROM_DEVICE).
 *
 * Must be called with @vv->vlock held (it walks the queue's host_mappings list).
 */
void virtio_media_sync_buffer(struct virtio_media_session *session, u32 type,
			      u32 index, enum dma_data_direction dir)
{
	struct virtio_media *vv = to_virtio_media(session->fh.vdev);
	u32 plane;

	if (type > VIRTIO_MEDIA_LAST_QUEUE)
		return;

	for (plane = 0; plane < VIDEO_MAX_PLANES; plane++) {
		struct virtio_media_host_mapping *map =
			virtio_media_find_host_mapping(session, type, index, plane);

		if (!map)
			break;

		if (vv->mem_ops->sync)
			vv->mem_ops->sync(map, dir);
	}
}

/*
 * Tear down a single host mapping: unmap the backend entries, tell the host to release
 * the buffer (MUNMAP), free the backend-allocated DMA pages and the tracking
 * structure. Called only from virtio_media_mapping_release_work() (the sole
 * teardown path), once the last reference to @map has been dropped and the
 * map unlinked from its queue's host_mappings list. Sends a command and sleeps
 * waiting for the host, so it must run in a sleepable context with @vv->vlock
 * held (the worker satisfies both).
 */
static void virtio_media_mapping_destroy_locked(struct virtio_media *vv,
					     struct virtio_media_host_mapping *map)
{
	struct virtio_media_cmd_munmap *cmd_munmap = &vv->cmd.munmap;
	struct virtio_media_resp_munmap *resp_munmap = &vv->resp.munmap;
	struct scatterlist cmd_sg = {}, resp_sg = {};
	struct scatterlist *sgs[2] = { &cmd_sg, &resp_sg };
	struct virtio_media_coherent_cmd cc;
	int ret;

	lockdep_assert_held(&vv->vlock);

	/* Backend-specific pre-MUNMAP teardown (e.g. unmap backend entries). */
	if (vv->mem_ops->map_stop)
		vv->mem_ops->map_stop(vv, map);

	cmd_munmap->hdr.cmd = VIRTIO_MEDIA_CMD_MUNMAP;
	cmd_munmap->map_handle = map->map_handle;
	cmd_munmap->num_pages = map->num_pages;

	/* Tell the host it can release the buffer. */
	if (vv->use_coherent) {
		virtio_media_coherent_cmd_init(&cc, vv);
		ret = virtio_media_coherent_add_out(&cc, &cmd_sg, cmd_munmap,
						    sizeof(*cmd_munmap));
		if (!ret)
			ret = virtio_media_coherent_add_in(&cc, &resp_sg,
							   resp_munmap,
							   sizeof(*resp_munmap));
		if (ret) {
			v4l2_err(&vv->v4l2_dev,
				 "failed to stage MUNMAP command: %d\n", ret);
			goto free_pages;
		}
	} else {
		sg_set_buf(&cmd_sg, cmd_munmap, sizeof(*cmd_munmap));
		sg_mark_end(&cmd_sg);
		sg_set_buf(&resp_sg, resp_munmap, sizeof(*resp_munmap));
		sg_mark_end(&resp_sg);
	}

	ret = virtio_media_send_command(vv, sgs, 1, 1, vv->use_coherent,
					sizeof(*resp_munmap), NULL);
	if (!ret && vv->use_coherent)
		virtio_media_coherent_retrieve(&cc);
	if (ret < 0)
		v4l2_err(&vv->v4l2_dev, "host failed to unmap buffer: %d\n",
			 ret);

free_pages:
	/* Backend-specific post-MUNMAP teardown (free pages + map->priv). */
	if (vv->mem_ops->map_free)
		vv->mem_ops->map_free(vv, map);

	kfree(map);
}

/* Take an extra reference on a host mapping. */
void virtio_media_mapping_get(struct virtio_media_host_mapping *map)
{
	refcount_inc(&map->refs);
}

/*
 * Worker that tears down host mappings queued for deferred release by
 * virtio_media_mapping_put(). Destroying a host mapping sends a MUNMAP command and
 * waits (seconds) for the host to acknowledge it, and needs @vv->vlock. This
 * is the ONLY place gmaps are destroyed: it runs in process context off any
 * external lock, so it can freely sleep and take @vv->vlock.
 */
static void virtio_media_mapping_release_work(struct work_struct *work)
{
	struct virtio_media *vv =
		container_of(work, struct virtio_media, mapping_release_work);
	struct llist_node *node;
	struct virtio_media_host_mapping *map, *tmp;

	node = llist_del_all(&vv->mapping_release_list);
	if (!node)
		return;

	mutex_lock(&vv->vlock);
	llist_for_each_entry_safe(map, tmp, node, release_node)
		virtio_media_mapping_destroy_locked(vv, map);
	mutex_unlock(&vv->vlock);
}

/*
 * Drop a reference on a host mapping. When the last reference goes away the
 * actual teardown (unmap backend entries, tell the host to release the buffer via
 * MUNMAP, free the pages) is always deferred to virtio_media_mapping_release_work().
 *
 * The teardown is a heavy, sleeping operation: it sends a MUNMAP command and
 * waits seconds for the host, under @vv->vlock. Callers drop references from a
 * variety of contexts -- some already holding @vv->vlock (REQBUFS, session
 * close), some that must not sleep at all (mmap close() runs under mmap_lock,
 * where sleeping on the command queue would stall the address space and the
 * mmap_lock -> vlock order would risk deadlock). Rather than have each caller
 * reason about its context, this single put never does the teardown inline: it
 * only decrements the refcount and, on the last reference, queues the map for
 * the release worker. This keeps the put path lock-free and callable from any
 * context.
 *
 * Callers that need the MUNMAP to have completed before proceeding (e.g. the
 * session-close path, which must unmap every mapping before sending CLOSE) must
 * flush_work(&vv->mapping_release_work) after their put.
 */
void virtio_media_mapping_put(struct virtio_media *vv,
			   struct virtio_media_host_mapping *map)
{
	if (refcount_dec_and_test(&map->refs)) {
		llist_add(&map->release_node, &vv->mapping_release_list);
		schedule_work(&vv->mapping_release_work);
	}
}

/*
 * Create a host mapping for a single (type, index, plane) identified by its v4l2
 * @offset. Idempotent: if a mapping for this (type, index, plane) already
 * exists it is returned as-is. @rw indicates whether the host should be given
 * write access to the pages.
 */
static int
virtio_media_create_host_mapping(struct virtio_media_session *session, u32 type,
			      u32 index, u32 plane, u32 offset, bool rw)
{
	struct virtio_media *vv = to_virtio_media(session->fh.vdev);
	struct virtio_media_cmd_mmap *cmd_mmap = &session->cmd.mmap;
	struct virtio_media_resp_mmap *resp_mmap = &session->resp.mmap;
	struct scatterlist cmd_sg = {}, resp_sg = {}, entries_sg = {};
	struct scatterlist *sgs[3] = { &cmd_sg, &resp_sg, &entries_sg };
	struct virtio_media_queue_state *queue = &session->queues[type];
	struct virtio_media_host_mapping *map;
	struct virtio_media_coherent_cmd cc;
	size_t entries_size = VIRTIO_MEDIA_MAX_MAPPING_PAGES * sizeof(u32);
	void *entries_coherent = NULL;
	dma_addr_t entries_coherent_dma = 0;
	u32 *entries = NULL;
	/*
	 * A memory backend with its own host-sharing mechanism (e.g. a virtio
	 * shared-memory region) sets wants_entries_buffer = false: it needs no
	 * per-page backing-entries buffer, so we neither allocate nor stage one
	 * and pass NULL/0 to map_create. The Xen backend sets it to true.
	 */
	bool wants_entries = vv->mem_ops->wants_entries_buffer;
	unsigned int in_sgs = wants_entries ? 2 : 1;
	int ret;

	/* Idempotent: skip if already mapped. */
	if (virtio_media_find_host_mapping(session, type, index, plane))
		return 0;

	/* Caller (QUERYBUF ioctl) already holds vv->vlock. */
	lockdep_assert_held(&vv->vlock);

	cmd_mmap->hdr.cmd = VIRTIO_MEDIA_CMD_MMAP;
	cmd_mmap->session_id = session->id;
	cmd_mmap->flags = rw ? BIT(VIRTIO_MEDIA_MMAP_FLAG_RW) : 0;
	cmd_mmap->offset = offset;

	/*
	 * The host returns one backend entry per page of the buffer. The
	 * entries are NOT guaranteed to be contiguous, so the host writes the
	 * full array into a second response buffer following the fixed
	 * response header. Provision a buffer large enough for the biggest
	 * mapping we support. Backends without an entries buffer skip this.
	 */
	if (wants_entries) {
		entries = kcalloc(VIRTIO_MEDIA_MAX_MAPPING_PAGES,
				  sizeof(*entries), GFP_KERNEL);
		if (!entries) {
			ret = -ENOMEM;
			goto end;
		}
	}

	if (vv->use_coherent) {
		/*
		 * The backing-entries array is too large for the shared ctrl_resp
		 * buffer, so give it its own coherent allocation that the host
		 * can write into through its host mapping.
		 */
		if (wants_entries) {
			entries_coherent = dma_alloc_coherent(vv->cmd_dma_dev,
							   entries_size,
							   &entries_coherent_dma,
							   GFP_KERNEL);
			if (!entries_coherent) {
				ret = -ENOMEM;
				goto err_free_refs;
			}
		}

		virtio_media_coherent_cmd_init(&cc, vv);
		ret = virtio_media_coherent_add_out(&cc, &cmd_sg, cmd_mmap,
						    sizeof(*cmd_mmap));
		if (!ret)
			ret = virtio_media_coherent_add_in(&cc, &resp_sg,
							   resp_mmap,
							   sizeof(*resp_mmap));
		if (ret) {
			v4l2_err(&vv->v4l2_dev,
				 "failed to stage MMAP command: %d\n", ret);
			goto err_free_coherent;
		}

		if (wants_entries) {
			sg_init_table(&entries_sg, 1);
			sg_set_buf(&entries_sg, entries_coherent, entries_size);
			sg_dma_address(&entries_sg) = entries_coherent_dma;
			sg_dma_len(&entries_sg) = entries_size;
		}
	} else {
		sg_set_buf(&cmd_sg, cmd_mmap, sizeof(*cmd_mmap));
		sg_mark_end(&cmd_sg);
		sg_set_buf(&resp_sg, resp_mmap, sizeof(*resp_mmap));
		sg_mark_end(&resp_sg);
		if (wants_entries) {
			sg_set_buf(&entries_sg, entries, entries_size);
			sg_mark_end(&entries_sg);
		}
	}

	ret = virtio_media_send_command(vv, sgs, 1, in_sgs, vv->use_coherent,
					sizeof(*resp_mmap), NULL);
	if (ret < 0)
		goto err_free_coherent;

	if (vv->use_coherent) {
		virtio_media_coherent_retrieve(&cc);
		if (wants_entries)
			memcpy(entries, entries_coherent, entries_size);
	}

	if (resp_mmap->num_pages > VIRTIO_MEDIA_MAX_MAPPING_PAGES) {
		v4l2_err(&vv->v4l2_dev,
			 "host returned too many backend entries: %u\n",
			 resp_mmap->num_pages);
		ret = -EINVAL;
		goto err_free_coherent;
	}

	map = kzalloc(sizeof(*map), GFP_KERNEL);
	if (!map) {
		ret = -ENOMEM;
		goto err_free_coherent;
	}

	map->map_handle = resp_mmap->map_handle;
	map->num_pages = resp_mmap->num_pages;
	map->len = resp_mmap->len;
	memcpy(map->uuid, resp_mmap->uuid, sizeof(map->uuid));
	map->v4l2_offset = offset;
	map->type = type;
	map->index = index;
	map->plane = plane;
	/* Held by the owning queue while linked into its host_mappings list. */
	refcount_set(&map->refs, 1);

	/*
	 * Hand the returned references to the memory backend, which allocates
	 * the backing pages and makes the host buffer accessible (e.g. maps
	 * the backend entries). Backend-private state is stored in map->priv.
	 */
	ret = vv->mem_ops->map_create(vv, map,
				      wants_entries ? entries : NULL,
				      wants_entries ? map->num_pages : 0,
				      rw);
	if (ret)
		goto err_free_map;

	list_add_tail(&map->list, &queue->host_mappings);
	if (entries_coherent)
		dma_free_coherent(vv->cmd_dma_dev, entries_size, entries_coherent,
				  entries_coherent_dma);
	kfree(entries);
	return 0;

err_free_map:
	kfree(map);
err_free_coherent:
	if (entries_coherent)
		dma_free_coherent(vv->cmd_dma_dev, entries_size, entries_coherent,
				  entries_coherent_dma);
err_free_refs:
	kfree(entries);
end:
	return ret;
}

/*
 * Create host mappings for every plane of a single MMAP buffer. Called from
 * QUERYBUF once the host has returned the per-plane mem_offsets. Idempotent.
 */
int virtio_media_map_buffer(struct virtio_media_session *session, u32 type,
			    u32 index)
{
	struct virtio_media_queue_state *queue;
	struct virtio_media_buffer *buffer;
	/* Give the host RW access so it can fill CAPTURE and read OUTPUT. */
	bool rw = true;
	int ret;

	if (type > VIRTIO_MEDIA_LAST_QUEUE)
		return -EINVAL;

	queue = &session->queues[type];
	if (index >= queue->allocated_bufs || !queue->buffers)
		return -EINVAL;

	buffer = queue->buffers[index];

	if (V4L2_TYPE_IS_MULTIPLANAR(type)) {
		u32 num_planes = buffer->buffer.length;
		u32 plane;

		if (num_planes > VIDEO_MAX_PLANES)
			return -EINVAL;
		for (plane = 0; plane < num_planes; plane++) {
			ret = virtio_media_create_host_mapping(
				session, type, index, plane,
				buffer->planes[plane].m.mem_offset, rw);
			if (ret)
				return ret;
		}
	} else {
		ret = virtio_media_create_host_mapping(session, type, index, 0,
						    buffer->buffer.m.offset, rw);
		if (ret)
			return ret;
	}

	return 0;
}

/* Free all host mappings of a single queue (e.g. on REQBUFS(0)). */
void virtio_media_free_queue_host_mappings(struct virtio_media_session *session,
					u32 type)
{
	struct virtio_media *vv = to_virtio_media(session->fh.vdev);
	struct virtio_media_host_mapping *map, *tmp;

	if (type > VIRTIO_MEDIA_LAST_QUEUE)
		return;

	/*
	 * Caller must hold vv->vlock: it protects the host_mappings list we walk
	 * and unlink from here. The gmap_put() below is itself lock-free (it
	 * only decrements the refcount and, on the last ref, queues the map
	 * for the release worker), so it is safe to call while holding vlock --
	 * the worker takes vlock later, from its own context.
	 */
	lockdep_assert_held(&vv->vlock);
	list_for_each_entry_safe(map, tmp, &session->queues[type].host_mappings,
				 list) {
		/*
		 * Unlink first, then drop the owning-queue reference. If a
		 * dma-buf or userspace mmap of this buffer is still alive it
		 * holds its own reference, so the backend entries and pages
		 * survive (unmapped only once the last reference is dropped)
		 * and the host never sees an in-use backend entry being torn down.
		 *
		 * Unlinking here is what invalidates this buffer's identity: a
		 * source change re-REQBUFS the queue and the host reuses the
		 * same (index, mem_offset) for the new buffers. Because the old
		 * map is off the host_mappings list, neither the QUERYBUF path
		 * (virtio_media_find_host_mapping by index) nor the mmap path
		 * (virtio_media_find_host_mapping_by_offset) can find it, so the
		 * new buffer always gets a fresh map and a fresh MMAP command
		 * (new backend entries) rather than aliasing the still-referenced
		 * old one. The host-side daemon must likewise not reuse its old
		 * mmap entry for the reused offset (it marks it stale on
		 * REQBUFS); the two sides together mirror vb2 removing the old
		 * buffer from the queue while its memory is refcounted away.
		 */
		list_del(&map->list);
		virtio_media_mapping_put(vv, map);
	}
}

/* Free all host mappings of all queues (session close safety net). */
void virtio_media_free_session_host_mappings(struct virtio_media_session *session)
{
	struct virtio_media *vv = to_virtio_media(session->fh.vdev);
	int i;

	/*
	 * free_queue_host_mappings() requires vv->vlock to walk and unlink the
	 * host_mappings lists; the close path does not already hold it, so take it
	 * here. The teardown itself (MUNMAP) is done later by the release
	 * worker; callers that need it completed before proceeding must
	 * flush_work(&vv->mapping_release_work) afterwards (see
	 * virtio_media_session_free()).
	 */
	mutex_lock(&vv->vlock);
	for (i = 0; i <= VIRTIO_MEDIA_LAST_QUEUE; i++)
		virtio_media_free_queue_host_mappings(session, i);
	mutex_unlock(&vv->vlock);
}

/**
 * virtio_media_send_dmabuf_attach() - Tell the backend to attach a dma-buf
 *	backing under a resource id.
 * @vv: virtio-media device in use.
 * @session_id: session the backing belongs to.
 * @resource_id: id the backend should remember this backing under.
 * @entries: backend entries of the imported dma-buf's pages.
 * @num_entries: number of references in @entries.
 *
 * Sends VIRTIO_MEDIA_CMD_DMABUF_ATTACH with the backend entries appended in
 * the device-readable part of the chain. Caller must hold vv->vlock.
 *
 * Returns 0 on success or a negative error code.
 */
static int virtio_media_send_dmabuf_attach(struct virtio_media *vv,
					   u32 session_id, u32 resource_id,
					   const u32 *entries,
					   u32 num_entries)
{
	struct virtio_media_cmd_dmabuf_attach *cmd = &vv->cmd.attach;
	struct virtio_media_resp_dmabuf_attach *resp = &vv->resp.attach;
	struct scatterlist cmd_sg = {}, entries_sg = {}, resp_sg = {};
	struct scatterlist *sgs[3] = { &cmd_sg, &entries_sg, &resp_sg };
	size_t entries_size = (size_t)num_entries * sizeof(*entries);
	struct virtio_media_coherent_cmd cc;
	void *entries_coherent = NULL;
	dma_addr_t entries_coherent_dma = 0;
	int ret;

	lockdep_assert_held(&vv->vlock);

	cmd->hdr.cmd = VIRTIO_MEDIA_CMD_DMABUF_ATTACH;
	cmd->hdr.__reserved = 0;
	cmd->session_id = session_id;
	cmd->resource_id = resource_id;
	cmd->num_entries = num_entries;
	cmd->__reserved = 0;

	if (vv->use_coherent) {
		/*
		 * The entries array can be larger than the shared ctrl_cmd
		 * buffer, so give it its own coherent allocation the device
		 * can read through its host mapping.
		 */
		entries_coherent = dma_alloc_coherent(vv->cmd_dma_dev, entries_size,
						   &entries_coherent_dma,
						   GFP_KERNEL);
		if (!entries_coherent)
			return -ENOMEM;
		memcpy(entries_coherent, entries, entries_size);

		virtio_media_coherent_cmd_init(&cc, vv);
		ret = virtio_media_coherent_add_out(&cc, &cmd_sg, cmd,
						    sizeof(*cmd));
		if (ret)
			goto err_free_coherent;

		sg_init_table(&entries_sg, 1);
		sg_set_buf(&entries_sg, entries_coherent, entries_size);
		sg_dma_address(&entries_sg) = entries_coherent_dma;
		sg_dma_len(&entries_sg) = entries_size;

		ret = virtio_media_coherent_add_in(&cc, &resp_sg, resp,
						   sizeof(*resp));
		if (ret)
			goto err_free_coherent;
	} else {
		sg_set_buf(&cmd_sg, cmd, sizeof(*cmd));
		sg_mark_end(&cmd_sg);
		sg_set_buf(&entries_sg, (void *)entries, entries_size);
		sg_mark_end(&entries_sg);
		sg_set_buf(&resp_sg, resp, sizeof(*resp));
		sg_mark_end(&resp_sg);
	}

	ret = virtio_media_send_command(vv, sgs, 2, 1, vv->use_coherent,
					sizeof(*resp), NULL);
	if (!ret && vv->use_coherent)
		virtio_media_coherent_retrieve(&cc);

err_free_coherent:
	if (entries_coherent)
		dma_free_coherent(vv->cmd_dma_dev, entries_size, entries_coherent,
				  entries_coherent_dma);
	return ret;
}

/**
 * virtio_media_send_dmabuf_detach() - Tell the backend to detach a dma-buf
 *	backing.
 * @vv: virtio-media device in use.
 * @session_id: session the backing belongs to.
 * @resource_id: id of the backing to detach.
 *
 * Sends VIRTIO_MEDIA_CMD_DMABUF_DETACH so the backend releases the local
 * dma-buf it exported, letting us then release the shared backing. Caller must
 * hold vv->vlock.
 */
static void virtio_media_send_dmabuf_detach(struct virtio_media *vv,
					    u32 session_id, u32 resource_id)
{
	struct virtio_media_cmd_dmabuf_detach *cmd = &vv->cmd.detach;
	struct virtio_media_resp_dmabuf_detach *resp = &vv->resp.detach;
	struct scatterlist cmd_sg = {}, resp_sg = {};
	struct scatterlist *sgs[2] = { &cmd_sg, &resp_sg };
	struct virtio_media_coherent_cmd cc;
	int ret;

	lockdep_assert_held(&vv->vlock);

	cmd->hdr.cmd = VIRTIO_MEDIA_CMD_DMABUF_DETACH;
	cmd->hdr.__reserved = 0;
	cmd->session_id = session_id;
	cmd->resource_id = resource_id;

	if (vv->use_coherent) {
		virtio_media_coherent_cmd_init(&cc, vv);
		ret = virtio_media_coherent_add_out(&cc, &cmd_sg, cmd,
						    sizeof(*cmd));
		if (!ret)
			ret = virtio_media_coherent_add_in(&cc, &resp_sg, resp,
							   sizeof(*resp));
		if (ret) {
			v4l2_err(&vv->v4l2_dev,
				 "failed to stage DMABUF_DETACH command: %d\n",
				 ret);
			return;
		}
	} else {
		sg_set_buf(&cmd_sg, cmd, sizeof(*cmd));
		sg_mark_end(&cmd_sg);
		sg_set_buf(&resp_sg, resp, sizeof(*resp));
		sg_mark_end(&resp_sg);
	}

	ret = virtio_media_send_command(vv, sgs, 1, 1, vv->use_coherent,
					sizeof(*resp), NULL);
	if (!ret && vv->use_coherent)
		virtio_media_coherent_retrieve(&cc);
	if (ret < 0)
		v4l2_err(&vv->v4l2_dev,
			 "backend failed to detach dma-buf (resource %u): %d\n",
			 resource_id, ret);
}

/**
 * virtio_media_resource_id_get() - Allocate a dma-buf resource id.
 * @vv: virtio-media device in use.
 * @resource_id: on success, receives the allocated id.
 *
 * Hands out the smallest currently-free id from @vv->resource_ida, so ids are
 * bounded by the number of concurrently-attached backings and a live backing's
 * id is never re-handed-out (no wrap, no ABA hazard - see the kerneldoc on
 * struct virtio_media::resource_ida). Ids start at 1 so 0 stays free to mean
 * "none". Paired with virtio_media_resource_id_put() on DMABUF_DETACH.
 *
 * Returns 0 on success or a negative error code.
 */
static int virtio_media_resource_id_get(struct virtio_media *vv,
					u32 *resource_id)
{
	int id = ida_alloc_min(&vv->resource_ida, 1, GFP_KERNEL);

	if (id < 0)
		return id;

	*resource_id = (u32)id;
	return 0;
}

/* Release a dma-buf resource id back to the allocator (see _get()). */
static void virtio_media_resource_id_put(struct virtio_media *vv,
					 u32 resource_id)
{
	ida_free(&vv->resource_ida, resource_id);
}

/**
 * virtio_media_dmabuf_import_alloc() - Import a guest dma-buf fd and attach it
 *	to the backend.
 * @session: session the buffer belongs to.
 * @fd: dma-buf file descriptor passed by userspace.
 *
 * Resolves @fd to a dma-buf, attaches it to the virtio device and maps it.
 * The memory backend produces the backing entries describing the mapped
 * sg_table (see import_get_entries). Those entries are sent to the backend
 * with DMABUF_ATTACH under a freshly allocated resource id, which the backend
 * uses to reconstruct the dma-buf on its side.
 *
 * Only dma-bufs backed by real guest pages take this path
 * (guest-pages model). Two other kinds are handled specially:
 *
 *   - virtio exported-objects carrying a shared-object UUID (identified by
 *     is_virtio_dma_buf()): these have no guest pages to share. This is the
 *     host-object consumer path - the backend already knows the buffer under
 *     that UUID, so we record the UUID and hold a reference rather than
 *     sharing backing. A dma-buf this virtio-media instance exported itself via
 *     VIDIOC_EXPBUF also lands here (its virtio_media_dmabuf_ops make
 *     is_virtio_dma_buf() true); it is legitimate as long as the backend
 *     registered a non-zero UUID for it. An all-zero UUID means no backend
 *     registration exists (see the protocol note in virtio_media.h): such a
 *     buffer cannot be resolved by the backend, so it is rejected with
 *     -EINVAL rather than queued with an unusable all-zero footer.
 *
 * Caller must hold vv->vlock (QBUF/PREPARE_BUF run under it).
 *
 * Returns the import on success, or an ERR_PTR on failure.
 */
static struct virtio_media_dmabuf_import *
virtio_media_dmabuf_import_alloc(struct virtio_media_session *session, int fd)
{
	struct virtio_media *vv = to_virtio_media(session->fh.vdev);
	struct virtio_media_dmabuf_import *imp;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	struct dma_buf *dmabuf;
	u32 *entries = NULL;
	u32 num_entries = 0;
	u32 resource_id;
	int ret;

	lockdep_assert_held(&vv->vlock);

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf))
		return ERR_CAST(dmabuf);

	/*
	 * dma-bufs exported by a virtio device carry a shared-object UUID and
	 * no guest pages. This is the host-object consumer path: the buffer is
	 * a host-owned dma-buf the backend already knows under that UUID, only
	 * wrapped into a virtio-dma-buf on the guest. Rather than share
	 * anything, we record the UUID and hold a reference on the dma-buf (the
	 * guest-side alpha refcount); every QBUF/PREPARE_BUF then carries the
	 * UUID so the backend resolves it to its own dma-buf fd for the real
	 * device. No attach, no sgt, no backing entries, no resource id.
	 */
	if (is_virtio_dma_buf(dmabuf)) {
		uuid_t uuid;

		ret = virtio_dma_buf_get_uuid(dmabuf, &uuid);
		if (ret) {
			dma_buf_put(dmabuf);
			return ERR_PTR(ret);
		}

		/*
		 * An all-zero UUID means the backend never registered this
		 * buffer as a shared object (virtio_media.h). It cannot be resolved
		 * on the host, so reject it rather than queue an unusable
		 * all-zero footer.
		 */
		if (uuid_is_null(&uuid)) {
			dma_buf_put(dmabuf);
			return ERR_PTR(-EINVAL);
		}

		imp = kzalloc(sizeof(*imp), GFP_KERNEL);
		if (!imp) {
			dma_buf_put(dmabuf);
			return ERR_PTR(-ENOMEM);
		}

		imp->dmabuf = dmabuf;
		imp->uuid_backed = true;
		export_uuid(imp->uuid, &uuid);

		return imp;
	}

	imp = kzalloc(sizeof(*imp), GFP_KERNEL);
	if (!imp) {
		ret = -ENOMEM;
		goto err_put;
	}

	attach = dma_buf_attach(dmabuf, vv->virtio_dev->dev.parent);
	if (IS_ERR(attach)) {
		ret = PTR_ERR(attach);
		goto err_free;
	}

	sgt = dma_buf_map_attachment_unlocked(attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		goto err_detach;
	}

	imp->dmabuf = dmabuf;
	imp->attach = attach;
	imp->sgt = sgt;

	ret = vv->mem_ops->import_get_entries(vv, imp, &entries, &num_entries);
	if (ret)
		goto err_unmap;

	/*
	 * Assign a resource id and tell the backend to attach the backing. The
	 * id is unique for as long as the backing stays attached (freed on
	 * DETACH), so the backend can key its export cache on it safely.
	 */
	ret = virtio_media_resource_id_get(vv, &resource_id);
	if (ret) {
		kfree(entries);
		goto err_put_entries;
	}
	imp->resource_id = resource_id;

	ret = virtio_media_send_dmabuf_attach(vv, session->id, resource_id,
					      entries, num_entries);
	kfree(entries);
	if (ret)
		goto err_put_id;

	return imp;

err_put_id:
	virtio_media_resource_id_put(vv, resource_id);
err_put_entries:
	/*
	 * The backend never attached (ATTACH not sent, or it failed), so no
	 * one holds a reference to the backing; releasing it now is safe.
	 */
	vv->mem_ops->import_put_entries(imp);
err_unmap:
	dma_buf_unmap_attachment_unlocked(attach, sgt, DMA_BIDIRECTIONAL);
err_detach:
	dma_buf_detach(dmabuf, attach);
err_free:
	kfree(imp);
err_put:
	dma_buf_put(dmabuf);
	return ERR_PTR(ret);
}

/*
 * Tear down a single dma-buf import: tell the backend to detach the backing
 * (so it drops the local dma-buf it exported), then release the backing by
 * unmapping, detach and drop our reference. Caller must hold vv->vlock.
 */
static void
virtio_media_dmabuf_import_free(struct virtio_media *vv, u32 session_id,
				struct virtio_media_dmabuf_import *imp)
{
	lockdep_assert_held(&vv->vlock);

	/*
	 * A host-object (UUID-backed) import holds no attach/sgt/backing and no
	 * backend resource id: the backing is a host-owned dma-buf the backend
	 * already knows under the UUID. Tearing it down is just dropping our
	 * reference on the imported virtio-dma-buf (the guest-side alpha
	 * refcount), which lets the exporter release it once no consumer holds
	 * it. Nothing to tell the backend here.
	 */
	if (imp->uuid_backed) {
		dma_buf_put(imp->dmabuf);
		kfree(imp);
		return;
	}

	/*
	 * DETACH first so the backend closes its exported dma-buf and drops
	 * its reference to the backing; only then is it safe to unmap and
	 * release the backing, otherwise the backend could still be using it.
	 */
	virtio_media_send_dmabuf_detach(vv, session_id, imp->resource_id);

	/*
	 * Return the id to the allocator only after DETACH: while the backend
	 * still knows the backing under this id, it must stay reserved so a
	 * concurrent attach cannot be handed the same id.
	 */
	virtio_media_resource_id_put(vv, imp->resource_id);

	/*
	 * Release the backing now that the backend has detached. Backend-
	 * private; must precede the unmap/detach below.
	 */
	vv->mem_ops->import_put_entries(imp);

	dma_buf_unmap_attachment_unlocked(imp->attach, imp->sgt,
					  DMA_BIDIRECTIONAL);
	dma_buf_detach(imp->dmabuf, imp->attach);
	dma_buf_put(imp->dmabuf);
	kfree(imp);
}

/**
 * virtio_media_find_dmabuf_import() - Find a queue's import for a dma-buf.
 * @queue: queue whose dmabuf_imports list to search.
 * @dmabuf: dma-buf to look for.
 *
 * Returns the existing import bound to @dmabuf on this queue, or NULL. Imports
 * are deduplicated by ``struct dma_buf *`` (a stable, non-reused key for as
 * long as the import holds a reference), so the same dma-buf queued on several
 * buffers/planes maps to one import. Caller must hold vv->vlock.
 */
static struct virtio_media_dmabuf_import *
virtio_media_find_dmabuf_import(struct virtio_media_queue_state *queue,
				struct dma_buf *dmabuf)
{
	struct virtio_media_dmabuf_import *imp;

	list_for_each_entry(imp, &queue->dmabuf_imports, list)
		if (imp->dmabuf == dmabuf)
			return imp;

	return NULL;
}

/**
 * virtio_media_bind_plane_dmabuf() - Bind one plane of a buffer to a dma-buf.
 * @session: session owning the buffer.
 * @queue: queue the buffer belongs to (owns the dmabuf_imports list).
 * @buffer: buffer state whose plane binding to update.
 * @plane: plane index within the buffer.
 * @fd: dma-buf file descriptor userspace passed for this plane.
 *
 * Binds the plane to the queue-level import for @fd's dma-buf, importing it
 * (attach + DMABUF_ATTACH) only if this queue has not imported that dma-buf
 * yet; otherwise the existing shared import is reused and no ATTACH is resent.
 * The plane binding is a non-owning pointer: the import stays attached for the
 * queue's whole REQBUFS lifetime and is torn down only when the queue's imports
 * are freed (see virtio_media_free_queue_dmabuf_imports()), never on rebind, so
 * we do not release backing the backend may still be using while streaming. Caller
 * must hold vv->vlock.
 *
 * Returns 0 on success or a negative error code (the slot is left bound to its
 * previous import, if any, on failure).
 */
static int
virtio_media_bind_plane_dmabuf(struct virtio_media_session *session,
			       struct virtio_media_queue_state *queue,
			       struct virtio_media_buffer *buffer, u32 plane,
			       int fd)
{
	struct virtio_media *vv = to_virtio_media(session->fh.vdev);
	struct virtio_media_dmabuf_import *imp;
	struct dma_buf *dmabuf;

	lockdep_assert_held(&vv->vlock);

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	/*
	 * Reuse the queue's existing import if this dma-buf was already
	 * imported (by this or any other plane/buffer on the queue): the
	 * backend already has the backing under its resource id. The import
	 * holds its own reference, so drop ours before returning.
	 */
	imp = virtio_media_find_dmabuf_import(queue, dmabuf);
	dma_buf_put(dmabuf);
	if (imp) {
		buffer->dmabuf_bindings[plane] = imp;
		return 0;
	}

	/* First time this dma-buf is queued: import it and cache on the queue. */
	imp = virtio_media_dmabuf_import_alloc(session, fd);
	if (IS_ERR(imp))
		return PTR_ERR(imp);

	list_add_tail(&imp->list, &queue->dmabuf_imports);
	buffer->dmabuf_bindings[plane] = imp;

	return 0;
}

/**
 * virtio_media_clear_buffer_dmabuf_bindings() - Drop a buffer's plane bindings.
 * @session: session owning the buffer.
 * @type: v4l2 buffer type (selects the queue).
 * @index: buffer index within the queue.
 *
 * Clears every plane's non-owning binding pointer for the buffer. Does NOT tear
 * down the imports themselves (those are owned by the queue and shared across
 * buffers; they are freed together in virtio_media_free_queue_dmabuf_imports()).
 * Used to roll a buffer back after a failed queueing without disturbing imports
 * that other buffers may still be using. Caller must hold vv->vlock.
 */
static void
virtio_media_clear_buffer_dmabuf_bindings(struct virtio_media_session *session,
					  u32 type, u32 index)
{
	struct virtio_media *vv = to_virtio_media(session->fh.vdev);
	struct virtio_media_queue_state *queue;
	struct virtio_media_buffer *buffer;
	u32 plane;

	if (type > VIRTIO_MEDIA_LAST_QUEUE)
		return;

	lockdep_assert_held(&vv->vlock);

	queue = &session->queues[type];
	if (index >= queue->allocated_bufs || !queue->buffers)
		return;

	buffer = queue->buffers[index];
	for (plane = 0; plane < VIDEO_MAX_PLANES; plane++)
		buffer->dmabuf_bindings[plane] = NULL;
}

/**
 * virtio_media_bind_buffer_dmabufs() - Bind (attach) every dma-buf plane of a
 *	buffer before it is queued.
 * @session: session the buffer belongs to.
 * @b: v4l2_buffer being queued (QBUF/PREPARE_BUF).
 *
 * For a V4L2_MEMORY_DMABUF buffer, binds each plane's dma-buf fd (see
 * virtio_media_bind_plane_dmabuf()). Does nothing for other memory types. On
 * failure any plane bindings recorded during this call are cleared so the
 * buffer is left with no partial bindings (the queue-level imports themselves
 * are left intact, since they may be shared with other buffers). Caller must
 * hold vv->vlock.
 *
 * Returns 0 on success or a negative error code.
 */
int virtio_media_bind_buffer_dmabufs(struct virtio_media_session *session,
				     struct v4l2_buffer *b)
{
	struct virtio_media_queue_state *queue;
	struct virtio_media_buffer *buffer;
	int i, ret;

	if (b->memory != V4L2_MEMORY_DMABUF)
		return 0;

	if (b->type > VIRTIO_MEDIA_LAST_QUEUE)
		return -EINVAL;
	queue = &session->queues[b->type];
	if (b->index >= queue->allocated_bufs)
		return -EINVAL;
	buffer = queue->buffers[b->index];

	if (V4L2_TYPE_IS_MULTIPLANAR(b->type)) {
		if (b->length > VIDEO_MAX_PLANES)
			return -EINVAL;
		for (i = 0; i < b->length; i++) {
			ret = virtio_media_bind_plane_dmabuf(
				session, queue, buffer, i,
				b->m.planes[i].m.fd);
			if (ret)
				goto err_unbind;
		}
	} else {
		ret = virtio_media_bind_plane_dmabuf(session, queue, buffer, 0,
						     b->m.fd);
		if (ret)
			return ret;
	}

	return 0;

err_unbind:
	/*
	 * A plane failed to bind. Clear this buffer's plane bindings (the
	 * queue-level imports stay attached for any other buffers using them);
	 * the next queueing re-resolves all planes.
	 */
	virtio_media_clear_buffer_dmabuf_bindings(session, b->type, b->index);
	return ret;
}

/**
 * virtio_media_dmabuf_substitute_resource_ids() - Swap plane guest fds for
 *	backend resource ids on the wire.
 * @session: session owning the buffer.
 * @b: v4l2_buffer about to be sent (QBUF/PREPARE_BUF, memory DMABUF).
 * @saved_fds: caller storage (VIDEO_MAX_PLANES entries) for the original fds.
 *
 * The device knows a dma-buf backing only by the resource id assigned at
 * DMABUF_ATTACH, never by the guest fd. For each plane, saves the guest fd and
 * writes the bound import's resource id into m.fd in its place. Must be paired
 * with virtio_media_dmabuf_restore_resource_ids() so userspace sees its own
 * fds again. Caller must hold vv->vlock.
 *
 * Returns 0 on success, or -EINVAL if a plane has no bound import (a queueing
 * bug: bind runs before this).
 */
int virtio_media_dmabuf_substitute_resource_ids(
	struct virtio_media_session *session, struct v4l2_buffer *b,
	u32 *saved_fds)
{
	struct virtio_media_queue_state *queue;
	struct virtio_media_buffer *buffer;
	unsigned int n, i;

	if (b->type > VIRTIO_MEDIA_LAST_QUEUE)
		return -EINVAL;
	queue = &session->queues[b->type];
	if (b->index >= queue->allocated_bufs)
		return -EINVAL;
	buffer = queue->buffers[b->index];

	n = V4L2_TYPE_IS_MULTIPLANAR(b->type) ? b->length : 1;
	if (n > VIDEO_MAX_PLANES)
		return -EINVAL;

	/*
	 * Validate the whole plane set first (all bound, all the same kind),
	 * mirroring virtio_media_dmabuf_collect_uuids(). Deciding on plane 0
	 * only after confirming consistency avoids substituting some planes
	 * and leaving others untouched (which would leak uninitialised
	 * saved_fds entries back to userspace via restore).
	 */
	for (i = 0; i < n; i++) {
		struct virtio_media_dmabuf_import *imp =
			buffer->dmabuf_bindings[i];

		if (!imp)
			return -EINVAL;

		if (imp->uuid_backed != buffer->dmabuf_bindings[0]->uuid_backed)
			return -EINVAL;
	}

	/*
	 * Host-object (UUID-backed) buffers carry no backend resource id; their
	 * planes are identified on the wire by an appended UUID footer (see
	 * virtio_media_dmabuf_collect_uuids()), so leave m.fd untouched.
	 */
	if (buffer->dmabuf_bindings[0]->uuid_backed)
		return 0;

	for (i = 0; i < n; i++) {
		struct virtio_media_dmabuf_import *imp =
			buffer->dmabuf_bindings[i];
		u32 *fd = V4L2_TYPE_IS_MULTIPLANAR(b->type) ?
				  &b->m.planes[i].m.fd :
				  &b->m.fd;

		saved_fds[i] = *fd;
		*fd = imp->resource_id;
	}

	return 0;
}

/**
 * virtio_media_dmabuf_restore_resource_ids() - Restore plane guest fds after
 *	sending.
 * @b: v4l2_buffer whose m.fd fields were substituted.
 * @saved_fds: the original fds saved by the substitute helper.
 *
 * Undoes virtio_media_dmabuf_substitute_resource_ids() so the V4L2 core copies
 * the caller's own fds back to userspace rather than backend resource ids.
 */
void virtio_media_dmabuf_restore_resource_ids(struct v4l2_buffer *b,
					      u32 *saved_fds)
{
	unsigned int n, i;

	n = V4L2_TYPE_IS_MULTIPLANAR(b->type) ? b->length : 1;
	if (n > VIDEO_MAX_PLANES)
		n = VIDEO_MAX_PLANES;

	for (i = 0; i < n; i++) {
		if (V4L2_TYPE_IS_MULTIPLANAR(b->type))
			b->m.planes[i].m.fd = saved_fds[i];
		else
			b->m.fd = saved_fds[i];
	}
}

/**
 * virtio_media_dmabuf_collect_uuids() - Gather a DMABUF buffer's per-plane
 *	shared-object UUID footers (host-object consumer path).
 * @session: session owning the buffer.
 * @b: v4l2_buffer about to be sent (QBUF/PREPARE_BUF, memory DMABUF).
 * @footers: caller storage (VIDEO_MAX_PLANES entries) filled on a host-object
 *	buffer, one footer per plane.
 * @n_out: receives the number of footers written: >0 for a host-object
 *	(UUID-backed) buffer, 0 for a guest-pages (guest-pages) or
 *	non-DMABUF buffer (which use resource ids in m.fd instead, see
 *	virtio_media_dmabuf_substitute_resource_ids()).
 *
 * A buffer's planes are either all UUID-backed (host-object) or all
 * guest-pages (guest-pages); mixing is a queueing bug. For a host-object
 * buffer, fills @footers with each plane's UUID (from its bound import) so the
 * caller can append them to the command's device-readable chain. Caller must
 * hold vv->vlock.
 *
 * Returns 0 on success (with *@n_out set), or -EINVAL if a plane has no bound
 * import or the buffer mixes UUID-backed and guest-pages planes.
 */
int virtio_media_dmabuf_collect_uuids(struct virtio_media_session *session,
				      struct v4l2_buffer *b,
				      struct virtio_media_dmabuf_uuid *footers,
				      unsigned int *n_out)
{
	struct virtio_media_queue_state *queue;
	struct virtio_media_buffer *buffer;
	unsigned int n, i;

	*n_out = 0;

	if (b->memory != V4L2_MEMORY_DMABUF)
		return 0;

	if (b->type > VIRTIO_MEDIA_LAST_QUEUE)
		return -EINVAL;
	queue = &session->queues[b->type];
	if (b->index >= queue->allocated_bufs)
		return -EINVAL;
	buffer = queue->buffers[b->index];

	n = V4L2_TYPE_IS_MULTIPLANAR(b->type) ? b->length : 1;
	if (n > VIDEO_MAX_PLANES)
		return -EINVAL;

	/*
	 * First pass: every plane must be bound, and all planes must agree on
	 * being UUID-backed (host-object) or not (guest-pages). Validate the
	 * whole set before deciding, so a buffer that mixes the two kinds is
	 * rejected regardless of plane order rather than silently trusting
	 * plane 0.
	 */
	for (i = 0; i < n; i++) {
		struct virtio_media_dmabuf_import *imp =
			buffer->dmabuf_bindings[i];

		if (!imp)
			return -EINVAL;

		if (imp->uuid_backed != buffer->dmabuf_bindings[0]->uuid_backed)
			return -EINVAL;
	}

	/* Guest-pages (guest-pages): no UUID footers, m.fd carries resource ids. */
	if (!buffer->dmabuf_bindings[0]->uuid_backed)
		return 0;

	/* Host-object (UUID-backed): one footer per plane. */
	for (i = 0; i < n; i++) {
		struct virtio_media_dmabuf_import *imp =
			buffer->dmabuf_bindings[i];

		memset(&footers[i], 0, sizeof(footers[i]));
		memcpy(footers[i].uuid, imp->uuid, sizeof(footers[i].uuid));
		footers[i].flags = VIRTIO_MEDIA_DMABUF_F_UUID;
	}

	*n_out = n;
	return 0;
}

/**
 * virtio_media_free_queue_dmabuf_imports() - Release all dma-buf imports of a
 *	queue.
 * @session: session owning the queue.
 * @type: v4l2 buffer type (selects the queue).
 *
 * Tears down (DMABUF_DETACH + release backing + unmap + detach) every import
 * owned by the queue and clears the non-owning plane bindings that pointed at
 * them. Must be called only once the backend can no longer be using the backing
 * (REQBUFS on this queue or session close, both of which the backend has
 * quiesced): unmapping backing still in use could corrupt the host's view.
 * Caller must hold vv->vlock.
 */
void virtio_media_free_queue_dmabuf_imports(struct virtio_media_session *session,
					    u32 type)
{
	struct virtio_media *vv = to_virtio_media(session->fh.vdev);
	struct virtio_media_dmabuf_import *imp, *tmp;
	struct virtio_media_queue_state *queue;
	size_t i;

	if (type > VIRTIO_MEDIA_LAST_QUEUE)
		return;

	lockdep_assert_held(&vv->vlock);

	queue = &session->queues[type];

	/*
	 * Clear every buffer's non-owning plane bindings first, so nothing
	 * points at an import once we start freeing them.
	 */
	if (queue->buffers) {
		for (i = 0; i < queue->allocated_bufs; i++) {
			struct virtio_media_buffer *buffer = queue->buffers[i];
			u32 plane;

			for (plane = 0; plane < VIDEO_MAX_PLANES; plane++)
				buffer->dmabuf_bindings[plane] = NULL;
		}
	}

	/* Then tear down the queue's imports themselves, once each. */
	list_for_each_entry_safe(imp, tmp, &queue->dmabuf_imports, list) {
		list_del(&imp->list);
		virtio_media_dmabuf_import_free(vv, session->id, imp);
	}
}

/* Free all dma-buf imports of all queues (session close safety net). */
void virtio_media_free_session_dmabuf_imports(
	struct virtio_media_session *session)
{
	struct virtio_media *vv = to_virtio_media(session->fh.vdev);
	int i;

	guard(mutex)(&vv->vlock);
	for (i = 0; i <= VIRTIO_MEDIA_LAST_QUEUE; i++)
		virtio_media_free_queue_dmabuf_imports(session, i);
}


/*
 * mmap and EXPBUF are memory-backend specific (they hand out the backend's
 * backing pages), so dispatch them to the active backend. A backend that has
 * no user-mappable pages (e.g. a shm-region backend) may leave these NULL.
 */
static int virtio_media_device_mmap(struct file *file,
				    struct vm_area_struct *vma)
{
	struct virtio_media *vv = to_virtio_media(video_devdata(file));

	return vv->mem_ops->mmap ? vv->mem_ops->mmap(file, vma) : -ENODEV;
}

int virtio_media_expbuf(struct file *file, void *fh,
			struct v4l2_exportbuffer *eb)
{
	struct virtio_media *vv = to_virtio_media(video_devdata(file));

	return vv->mem_ops->expbuf ? vv->mem_ops->expbuf(file, fh, eb) :
				     -ENOTTY;
}
static const struct v4l2_file_operations virtio_media_fops = {
	.owner = THIS_MODULE,
	.open = virtio_media_device_open,
	.release = virtio_media_device_close,
	.poll = virtio_media_device_poll,
	.unlocked_ioctl = virtio_media_device_ioctl,
	.mmap = virtio_media_device_mmap,
};

static int virtio_media_probe(struct virtio_device *virtio_dev)
{
	struct device *dev = &virtio_dev->dev;
	struct virtqueue *vqs[2];
	static struct virtqueue_info vq_info[2] = {
		{
			.name = "command",
			.callback = commandq_callback,
		},
		{
			.name = "event",
			.callback = eventq_callback,
		},
	};
	struct virtio_media *vv;
	struct video_device *vd;
	int i;
	int ret;

	vv = devm_kzalloc(dev, sizeof(*vv), GFP_KERNEL);
	if (!vv)
		return -ENOMEM;

	vv->event_buffer = devm_kzalloc(dev,
					VIRTIO_MEDIA_EVENT_MAX_SIZE *
					VIRTIO_MEDIA_NUM_EVENT_BUFS,
					GFP_KERNEL);
	if (!vv->event_buffer)
		return -ENOMEM;

	INIT_LIST_HEAD(&vv->sessions);
	mutex_init(&vv->sessions_lock);
	mutex_init(&vv->events_lock);
	mutex_init(&vv->vlock);

	vv->virtio_dev = virtio_dev;
	virtio_dev->priv = vv;

	/* Memory backend: use Xen grant-map when running as a Xen guest. */
#if IS_ENABLED(CONFIG_XEN)
	vv->mem_ops = &virtio_media_xen_mem_ops;
#endif

	/* Allocator for V4L2_MEMORY_DMABUF resource ids (ids start at 1). */
	ida_init(&vv->resource_ida);

	init_waitqueue_head(&vv->wq);

	ret = v4l2_device_register(dev, &vv->v4l2_dev);
	if (ret)
		return ret;

	ret = virtio_find_vqs(virtio_dev, 2, vqs, vq_info, NULL);
	if (ret)
		goto err_find_vqs;

	vv->commandq = vqs[0];
	vv->eventq = vqs[1];
	INIT_WORK(&vv->eventq_work, virtio_media_event_work);
	init_llist_head(&vv->mapping_release_list);
	INIT_WORK(&vv->mapping_release_work, virtio_media_mapping_release_work);

	if (use_coherent_shadow_buffer) {
		struct device *dma_dev = virtqueue_dma_dev(vv->commandq);

		if (!dma_dev) {
			/*
			 * The virtqueue does not use the DMA API, so it
			 * references buffers by physical address. A coherent
			 * DMA address would not match what the device expects,
			 * so fall back to the legacy (non-coherent) path.
			 */
			v4l2_warn(&vv->v4l2_dev,
				  "commandq does not use the DMA API; coherent command buffers disabled\n");
		} else {
			vv->cmd_dma_dev = dma_dev;
			vv->ctrl_cmd = dma_alloc_coherent(dma_dev, PAGE_SIZE,
							  &vv->ctrl_cmd_dma,
							  GFP_KERNEL);
			vv->ctrl_resp = dma_alloc_coherent(dma_dev, PAGE_SIZE,
							   &vv->ctrl_resp_dma,
							   GFP_KERNEL);
			if (!vv->ctrl_cmd || !vv->ctrl_resp) {
				ret = -ENOMEM;
				goto err_coherent;
			}
			vv->use_coherent = true;
			v4l2_info(&vv->v4l2_dev,
				  "using coherent DMA for command buffers\n");
		}
	}

	/* Get MMAP buffer mapping SHM region */
	virtio_get_shm_region(virtio_dev, &vv->mmap_region,
			      VIRTIO_MEDIA_SHM_MMAP);

	vd = &vv->video_dev;

	vd->v4l2_dev = &vv->v4l2_dev;
	vd->vfl_type = VFL_TYPE_VIDEO;
	vd->ioctl_ops = &virtio_media_ioctl_ops;
	vd->fops = &virtio_media_fops;
	vd->device_caps = virtio_cread32(virtio_dev, 0);
	if (vd->device_caps & (V4L2_CAP_VIDEO_M2M | V4L2_CAP_VIDEO_M2M_MPLANE))
		vd->vfl_dir = VFL_DIR_M2M;
	else if (vd->device_caps &
		 (V4L2_CAP_VIDEO_OUTPUT | V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE))
		vd->vfl_dir = VFL_DIR_TX;
	else
		vd->vfl_dir = VFL_DIR_RX;
	vd->release = video_device_release_empty;
	strscpy(vd->name, "virtio-media", sizeof(vd->name));

	video_set_drvdata(vd, vv);

	ret = video_register_device(vd, virtio_cread32(virtio_dev, 4), 0);
	if (ret)
		goto err_register_device;

	virtio_media_debug_init_device(vv);

	for (i = 0; i < VIRTIO_MEDIA_NUM_EVENT_BUFS; i++) {
		void *ebuf = vv->event_buffer + VIRTIO_MEDIA_EVENT_MAX_SIZE * i;

		ret = virtio_media_send_event_buffer(vv, ebuf);
		if (ret)
			goto err_send_event_buffer;
	}

	virtio_device_ready(virtio_dev);

	return 0;

err_send_event_buffer:
	virtio_media_debug_release_device(vv);
	video_unregister_device(&vv->video_dev);
err_register_device:
err_coherent:
	if (vv->ctrl_cmd)
		dma_free_coherent(vv->cmd_dma_dev, PAGE_SIZE, vv->ctrl_cmd,
				  vv->ctrl_cmd_dma);
	if (vv->ctrl_resp)
		dma_free_coherent(vv->cmd_dma_dev, PAGE_SIZE, vv->ctrl_resp,
				  vv->ctrl_resp_dma);
	virtio_dev->config->del_vqs(virtio_dev);
err_find_vqs:
	v4l2_device_unregister(&vv->v4l2_dev);

	return ret;
}

static void virtio_media_remove(struct virtio_device *virtio_dev)
{
	struct virtio_media *vv = virtio_dev->priv;
	struct list_head *p, *n;

	cancel_work_sync(&vv->eventq_work);

	virtio_media_debug_release_device(vv);

	/*
	 * Tear the sessions down while the command queue is still alive.
	 * virtio_media_session_free() drops each host mapping's owning-queue
	 * reference via virtio_media_mapping_put(), which -- on the last reference
	 * -- queues the map for the release worker (it never destroys inline).
	 * So this both frees the sessions and (re)schedules mapping_release_work.
	 */
	list_for_each_safe(p, n, &vv->sessions) {
		struct virtio_media_session *s =
			list_entry(p, struct virtio_media_session, list);

		virtio_media_session_free(vv, s, false);
	}

	/*
	 * Now drain the deferred host-mapping releases. The release worker sends
	 * MUNMAP commands on the command queue, so it must run *before* the
	 * device is reset and the vqs are deleted. cancel_work_sync() runs any
	 * pending worker to completion and prevents it from being requeued
	 * afterwards -- and since every session has already been freed above,
	 * nothing can schedule the worker again after this point. This must
	 * come after the session loop (which is what schedules the final
	 * releases) to avoid leaving a worker queued that would later run
	 * against a reset device and a freed @vv (use-after-free).
	 */
	cancel_work_sync(&vv->mapping_release_work);

	virtio_reset_device(virtio_dev);

	/*
	 * Every session has been freed above, so all dma-buf imports have been
	 * detached and their resource ids returned to the allocator; the IDA is
	 * empty and can be destroyed.
	 */
	ida_destroy(&vv->resource_ida);

	v4l2_device_unregister(&vv->v4l2_dev);
	if (vv->ctrl_cmd)
		dma_free_coherent(vv->cmd_dma_dev, PAGE_SIZE, vv->ctrl_cmd,
				  vv->ctrl_cmd_dma);
	if (vv->ctrl_resp)
		dma_free_coherent(vv->cmd_dma_dev, PAGE_SIZE, vv->ctrl_resp,
				  vv->ctrl_resp_dma);
	virtio_dev->config->del_vqs(virtio_dev);
	video_unregister_device(&vv->video_dev);
}

static struct virtio_device_id id_table[] = {
	{ VIRTIO_ID_MEDIA, VIRTIO_DEV_ANY_ID },
	{ 0 },
};

static unsigned int features[] = {};

/*
 * Under Xen the guest's one-shot indirect descriptor table lives in a
 * foreign-mapped guest page that the vhost-user-media daemon can observe with
 * stale contents, causing it to read a garbage descriptor and abort.  Since
 * virtio-media only ever uses tiny descriptor chains (command + response and
 * fixed-size event buffers), indirect descriptors provide no benefit.  Clear
 * the feature so the guest always uses direct descriptor chains.
 */
static int virtio_media_validate(struct virtio_device *vdev)
{
	__virtio_clear_bit(vdev, VIRTIO_RING_F_INDIRECT_DESC);
	return 0;
}

static struct virtio_driver virtio_media_driver = {
	.feature_table = features,
	.feature_table_size = ARRAY_SIZE(features),
	.driver.name = VIRTIO_MEDIA_DEFAULT_DRIVER_NAME,
	.driver.owner = THIS_MODULE,
	.id_table = id_table,
	.validate = virtio_media_validate,
	.probe = virtio_media_probe,
	.remove = virtio_media_remove,
};

module_virtio_driver(virtio_media_driver);

MODULE_DEVICE_TABLE(virtio, id_table);
MODULE_DESCRIPTION("virtio media driver");
MODULE_AUTHOR("Alexandre Courbot <gnurou@gmail.com>");
MODULE_LICENSE("Dual BSD/GPL");
