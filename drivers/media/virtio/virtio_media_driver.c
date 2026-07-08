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
#include <linux/virtio_ids.h>
#include <uapi/linux/virtio_ring.h>

#include <xen/grant_table.h>

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

/*
 * If set, command descriptor payloads are bounced through a per-device
 * cache-coherent DMA buffer and submitted to the command virtqueue as
 * premapped scatterlists. This avoids cache-coherency issues when the host
 * (e.g. vhost-user-media on Xen) accesses the payloads through foreign/grant
 * mappings that bypass the guest's non-coherent DMA path.
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

	for (i = 0; i <= VIRTIO_MEDIA_LAST_QUEUE; i++) {
		INIT_LIST_HEAD(&session->queues[i].pending_dqbufs);
		INIT_LIST_HEAD(&session->queues[i].grant_maps);
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
 * The grant maps are released (their owning-queue reference dropped) and the
 * deferred release worker is flushed so every MUNMAP completes *before*
 * CMD_CLOSE, so that by the time the host acts on CLOSE (and releases the
 * underlying V4L2 buffers) the guest has already unmapped every grant
 * reference. When called from device removal the host is gone, so
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

	/*
	 * Drop the owning-queue reference on every grant map. The actual
	 * teardown (unmapping grant refs and sending MUNMAP) is deferred to
	 * vv->gmap_release_work. Free them while session->fh.vdev is still
	 * valid, since the helper derives the device from it. Must run before
	 * v4l2_fh_exit().
	 */
	virtio_media_free_session_grant_maps(session);

	/*
	 * CLOSE must be sent only after every grant ref has actually been
	 * unmapped (MUNMAP), otherwise the host could release the underlying
	 * V4L2 buffers while the guest still has grants mapped. Since teardown
	 * is deferred to the release worker, flush it here before sending
	 * CLOSE so all MUNMAPs have completed first.
	 *
	 * On device removal (!notify_host) CLOSE is not sent. The MUNMAPs still
	 * scheduled by the puts above are instead drained by the caller
	 * (virtio_media_remove() cancel_work_sync()s the release worker after
	 * freeing every session, before resetting the device).
	 */
	if (notify_host) {
		flush_work(&vv->gmap_release_work);
		virtio_media_session_send_close(vv, session);
	}

	virtio_media_session_fh_del(session);
	v4l2_fh_exit(&session->fh);

	sg_free_table(&session->command_sgs);

	for (i = 0; i <= VIRTIO_MEDIA_LAST_QUEUE; i++)
		vfree(session->queues[i].buffers);

	if (vv->use_coherent)
		dma_free_coherent(vv->cmd_dma_dev, VIRTIO_SHADOW_BUF_SIZE,
				  session->shadow_buf, session->shadow_buf_dma);
	else
		kfree(session->shadow_buf);
	kfree(session);
}

/**
 * virtio_media_find_session - Lookup for the session with a given ID.
 * @vv: virtio-media device to lookup the session from.
 * @id: ID of the session to lookup.
 */
static struct virtio_media_session *
virtio_media_find_session(struct virtio_media *vv, u32 id)
{
	struct list_head *p;
	struct virtio_media_session *session = NULL;

	mutex_lock(&vv->sessions_lock);
	list_for_each(p, &vv->sessions) {
		struct virtio_media_session *s =
			list_entry(p, struct virtio_media_session, list);
		if (s->id == id) {
			session = s;
			break;
		}
	}
	mutex_unlock(&vv->sessions_lock);

	return session;
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
	if (ret < 0)
		return ret;

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

	if (dqbuf_evt->buffer.index >= queue->allocated_bufs) {
		v4l2_err(&vv->v4l2_dev,
			 "invalid buffer ID %d for queue %d in dqbuf event",
			 dqbuf_evt->buffer.index, dqbuf_evt->buffer.type);
		return;
	}

	dqbuf = &queue->buffers[dqbuf_evt->buffer.index];

	/*
	 * Preserve the 'm' union that was passed to us during QBUF so userspace
	 * gets back the information it submitted.
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

	mutex_lock(&session->queues_lock);
	list_add_tail(&dqbuf->list, &queue->pending_dqbufs);
	queue->queued_bufs -= 1;
	mutex_unlock(&session->queues_lock);

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

		session = virtio_media_find_session(vv, evt->session_id);
		if (!session) {
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
			v4l2_event_queue_fh(&session->fh, &event_evt->event);
			break;

		default:
			v4l2_err(&vv->v4l2_dev, "unknown event type %d\n",
				 evt->event);
			break;
		}

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
	 * The session (and its grant maps) is torn down here. Every dma-buf
	 * exported via VIDIOC_EXPBUF holds a reference to this file via
	 * get_file(), so device close - and therefore this teardown - only
	 * runs after the open fd is closed AND every exported dma-buf has been
	 * released. By then no user mapping of any grant page remains, so it
	 * is safe to unmap the grant refs and notify the host with CMD_CLOSE.
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

	mutex_lock(&session->queues_lock);
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
	mutex_unlock(&session->queues_lock);

	if (v4l2_event_pending(&session->fh))
		rc |= EPOLLPRI;

	return rc;
}

/*
 * Grant-map subsystem for zero-copy MMAP buffer sharing with the host.
 *
 * A grant map is created per (type, index, plane) at QUERYBUF time and lives
 * until the queue is torn down by REQBUFS or the session is closed. The
 * mapping is decoupled from any userspace VMA: closing a VMA only tears down
 * the userspace PTEs, while the underlying grant-mapped pages persist so that
 * seek (which re-mmaps buffers without re-allocating them) keeps working and
 * the host keeps ownership of the buffer.
 *
 * Locking: the grant_maps list is created/destroyed under vv->vlock (QUERYBUF,
 * REQBUFS, session close) and additionally walked under queues_lock by the
 * paths that do not run under vv->vlock. Callers of the find helpers below
 * must hold at least one of those locks; any path that then dereferences the
 * returned gmap while it might race a REQBUFS/close teardown must hold
 * vv->vlock (or an explicit gmap reference) so the gmap and its pages cannot
 * be freed under it.
 */

static struct virtio_media_grant_map *
virtio_media_find_grant_map(struct virtio_media_session *session, u32 type,
			    u32 index, u32 plane)
{
	struct virtio_media_queue_state *queue;
	struct virtio_media_grant_map *gmap;

	if (type > VIRTIO_MEDIA_LAST_QUEUE)
		return NULL;

	queue = &session->queues[type];
	list_for_each_entry(gmap, &queue->grant_maps, list) {
		if (gmap->index == index && gmap->plane == plane)
			return gmap;
	}
	return NULL;
}

/* Find a grant map in any queue of the session by its v4l2 mem_offset. */
static struct virtio_media_grant_map *
virtio_media_find_grant_map_by_offset(struct virtio_media_session *session,
				      u32 offset)
{
	struct virtio_media_grant_map *gmap;
	int i;

	for (i = 0; i <= VIRTIO_MEDIA_LAST_QUEUE; i++) {
		list_for_each_entry(gmap, &session->queues[i].grant_maps, list) {
			if (gmap->v4l2_offset == offset)
				return gmap;
		}
	}
	return NULL;
}

/*
 * Cache maintenance for grant-mapped buffers.
 *
 * The buffers are foreign pages granted by the host and mapped into the guest
 * with a Normal (cacheable) memory type. The host-side hardware (e.g. the VPU)
 * is a non-coherent DMA master, so the guest must explicitly maintain the CPU
 * caches around ownership transfer:
 *  - before QBUF (guest -> device): clean the CPU caches so the data the guest
 *    wrote is visible to the device in DRAM (DMA_TO_DEVICE).
 *  - after DQBUF (device -> guest): invalidate the CPU caches so the guest
 *    reads the data the device wrote to DRAM (DMA_FROM_DEVICE).
 *
 * Must be called with @vv->vlock held (it walks the queue's grant_maps list).
 */
void virtio_media_sync_buffer(struct virtio_media_session *session, u32 type,
			      u32 index, enum dma_data_direction dir)
{
	u32 plane;

	if (type > VIRTIO_MEDIA_LAST_QUEUE)
		return;

	for (plane = 0; plane < VIDEO_MAX_PLANES; plane++) {
		struct virtio_media_grant_map *gmap =
			virtio_media_find_grant_map(session, type, index, plane);
		int i;

		if (!gmap)
			break;

		for (i = 0; i < gmap->grant_ref_count; i++) {
			phys_addr_t phys = page_to_phys(gmap->pages[i]);

			if (dir == DMA_TO_DEVICE)
				arch_sync_dma_for_device(phys, PAGE_SIZE, dir);
			else
				arch_sync_dma_for_cpu(phys, PAGE_SIZE, dir);
		}
	}
}

/*
 * Tear down a single grant map: unmap the grant refs, tell the host to release
 * the buffer (MUNMAP), free the grant-allocated DMA pages and the tracking
 * structure. Called only from virtio_media_gmap_release_work() (the sole
 * teardown path), once the last reference to @gmap has been dropped and the
 * gmap unlinked from its queue's grant_maps list. Sends a command and sleeps
 * waiting for the host, so it must run in a sleepable context with @vv->vlock
 * held (the worker satisfies both).
 */
static void virtio_media_gmap_destroy_locked(struct virtio_media *vv,
					     struct virtio_media_grant_map *gmap)
{
	struct virtio_media_cmd_munmap *cmd_munmap = &vv->cmd.munmap;
	struct virtio_media_resp_munmap *resp_munmap = &vv->resp.munmap;
	struct scatterlist cmd_sg = {}, resp_sg = {};
	struct scatterlist *sgs[2] = { &cmd_sg, &resp_sg };
	struct virtio_media_coherent_cmd cc;
	int ret;

	lockdep_assert_held(&vv->vlock);

	/* Unmap the grant references. */
	ret = gnttab_unmap_refs(gmap->unmap_ops, NULL, gmap->pages,
				gmap->grant_ref_count);
	if (ret)
		v4l2_err(&vv->v4l2_dev, "gnttab_unmap_refs failed: %d\n", ret);

	cmd_munmap->hdr.cmd = VIRTIO_MEDIA_CMD_MUNMAP;
	cmd_munmap->grant_ref_header = gmap->grant_ref_header;
	cmd_munmap->grant_ref_count = gmap->grant_ref_count;

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
	/* Free the balloon-backed grant pages. */
	gnttab_free_pages(gmap->grant_ref_count, gmap->pages);

	kfree(gmap->map_ops);
	kfree(gmap->unmap_ops);
	kfree(gmap->pages);
	kfree(gmap);
}

/* Take an extra reference on a grant map. */
static void virtio_media_gmap_get(struct virtio_media_grant_map *gmap)
{
	refcount_inc(&gmap->refs);
}

/*
 * Worker that tears down grant maps queued for deferred release by
 * virtio_media_gmap_put(). Destroying a grant map sends a MUNMAP command and
 * waits (seconds) for the host to acknowledge it, and needs @vv->vlock. This
 * is the ONLY place gmaps are destroyed: it runs in process context off any
 * external lock, so it can freely sleep and take @vv->vlock.
 */
static void virtio_media_gmap_release_work(struct work_struct *work)
{
	struct virtio_media *vv =
		container_of(work, struct virtio_media, gmap_release_work);
	struct llist_node *node;
	struct virtio_media_grant_map *gmap, *tmp;

	node = llist_del_all(&vv->gmap_release_list);
	if (!node)
		return;

	mutex_lock(&vv->vlock);
	llist_for_each_entry_safe(gmap, tmp, node, release_node)
		virtio_media_gmap_destroy_locked(vv, gmap);
	mutex_unlock(&vv->vlock);
}

/*
 * Drop a reference on a grant map. When the last reference goes away the
 * actual teardown (unmap grant refs, tell the host to release the buffer via
 * MUNMAP, free the pages) is always deferred to virtio_media_gmap_release_work().
 *
 * The teardown is a heavy, sleeping operation: it sends a MUNMAP command and
 * waits seconds for the host, under @vv->vlock. Callers drop references from a
 * variety of contexts -- some already holding @vv->vlock (REQBUFS, session
 * close), some that must not sleep at all (mmap close() runs under mmap_lock,
 * where sleeping on the command queue would stall the address space and the
 * mmap_lock -> vlock order would risk deadlock). Rather than have each caller
 * reason about its context, this single put never does the teardown inline: it
 * only decrements the refcount and, on the last reference, queues the gmap for
 * the release worker. This keeps the put path lock-free and callable from any
 * context.
 *
 * Callers that need the MUNMAP to have completed before proceeding (e.g. the
 * session-close path, which must unmap every grant before sending CLOSE) must
 * flush_work(&vv->gmap_release_work) after their put.
 */
static void virtio_media_gmap_put(struct virtio_media *vv,
				  struct virtio_media_grant_map *gmap)
{
	if (refcount_dec_and_test(&gmap->refs)) {
		llist_add(&gmap->release_node, &vv->gmap_release_list);
		schedule_work(&vv->gmap_release_work);
	}
}

/*
 * Create a grant map for a single (type, index, plane) identified by its v4l2
 * @offset. Idempotent: if a mapping for this (type, index, plane) already
 * exists it is returned as-is. @rw indicates whether the host should be granted
 * write access to the pages.
 */
static int
virtio_media_create_grant_map(struct virtio_media_session *session, u32 type,
			      u32 index, u32 plane, u32 offset, bool rw)
{
	struct virtio_media *vv = to_virtio_media(session->fh.vdev);
	struct virtio_media_cmd_mmap *cmd_mmap = &session->cmd.mmap;
	struct virtio_media_resp_mmap *resp_mmap = &session->resp.mmap;
	struct scatterlist cmd_sg = {}, resp_sg = {}, refs_sg = {};
	struct scatterlist *sgs[3] = { &cmd_sg, &resp_sg, &refs_sg };
	struct virtio_media_queue_state *queue = &session->queues[type];
	struct virtio_media_grant_map *gmap;
	struct virtio_media_coherent_cmd cc;
	size_t refs_size = VIRTIO_MEDIA_MAX_GRANT_REFS * sizeof(u32);
	void *refs_coherent = NULL;
	dma_addr_t refs_coherent_dma = 0;
	u32 *refs;
	int i, ret;

	/* Idempotent: skip if already mapped. */
	if (virtio_media_find_grant_map(session, type, index, plane))
		return 0;

	/* Caller (QUERYBUF ioctl) already holds vv->vlock. */
	lockdep_assert_held(&vv->vlock);

	cmd_mmap->hdr.cmd = VIRTIO_MEDIA_CMD_MMAP;
	cmd_mmap->session_id = session->id;
	cmd_mmap->flags = rw ? BIT(VIRTIO_MEDIA_MMAP_FLAG_RW) : 0;
	cmd_mmap->offset = offset;

	/*
	 * The host returns one grant reference per page of the buffer. The
	 * refs are NOT guaranteed to be contiguous, so the host writes the
	 * full array into a second response buffer following the fixed
	 * response header. Provision a buffer large enough for the biggest
	 * mapping we support.
	 */
	refs = kcalloc(VIRTIO_MEDIA_MAX_GRANT_REFS, sizeof(*refs), GFP_KERNEL);
	if (!refs) {
		ret = -ENOMEM;
		goto end;
	}

	if (vv->use_coherent) {
		/*
		 * The grant-ref array is too large for the shared ctrl_resp
		 * buffer, so give it its own coherent allocation that the host
		 * can write into through its foreign mapping.
		 */
		refs_coherent = dma_alloc_coherent(vv->cmd_dma_dev, refs_size,
						   &refs_coherent_dma,
						   GFP_KERNEL);
		if (!refs_coherent) {
			ret = -ENOMEM;
			goto err_free_refs;
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

		sg_init_table(&refs_sg, 1);
		sg_set_buf(&refs_sg, refs_coherent, refs_size);
		sg_dma_address(&refs_sg) = refs_coherent_dma;
		sg_dma_len(&refs_sg) = refs_size;
	} else {
		sg_set_buf(&cmd_sg, cmd_mmap, sizeof(*cmd_mmap));
		sg_mark_end(&cmd_sg);
		sg_set_buf(&resp_sg, resp_mmap, sizeof(*resp_mmap));
		sg_mark_end(&resp_sg);
		sg_set_buf(&refs_sg, refs, refs_size);
		sg_mark_end(&refs_sg);
	}

	ret = virtio_media_send_command(vv, sgs, 1, 2, vv->use_coherent,
					sizeof(*resp_mmap), NULL);
	if (ret < 0)
		goto err_free_coherent;

	if (vv->use_coherent) {
		virtio_media_coherent_retrieve(&cc);
		memcpy(refs, refs_coherent, refs_size);
	}

	if (resp_mmap->grant_ref_count > VIRTIO_MEDIA_MAX_GRANT_REFS) {
		v4l2_err(&vv->v4l2_dev,
			 "host returned too many grant refs: %u\n",
			 resp_mmap->grant_ref_count);
		ret = -EINVAL;
		goto err_free_coherent;
	}

	gmap = kzalloc(sizeof(*gmap), GFP_KERNEL);
	if (!gmap) {
		ret = -ENOMEM;
		goto err_free_coherent;
	}

	gmap->grant_ref_header = resp_mmap->grant_ref_header;
	gmap->grant_ref_count = resp_mmap->grant_ref_count;
	gmap->len = resp_mmap->len;
	gmap->v4l2_offset = offset;
	gmap->type = type;
	gmap->index = index;
	gmap->plane = plane;
	/* Held by the owning queue while linked into its grant_maps list. */
	refcount_set(&gmap->refs, 1);

	gmap->pages = kcalloc(gmap->grant_ref_count, sizeof(struct page *),
			      GFP_KERNEL);
	if (!gmap->pages) {
		ret = -ENOMEM;
		goto err_free_gmap;
	}

	/*
	 * Allocate empty, balloon-backed pages to be filled in with the
	 * foreign frames granted by the host via gnttab_map_refs(). This is
	 * the correct pairing for mapping foreign grant refs (do NOT use
	 * gnttab_dma_alloc_pages(), which allocates local DMA-able backing
	 * storage for the grantor side and returns non-cacheable coherent
	 * memory that faults on DC ZVA when zeroed).
	 */
	ret = gnttab_alloc_pages(gmap->grant_ref_count, gmap->pages);
	if (ret) {
		v4l2_err(&vv->v4l2_dev, "gnttab_alloc_pages failed: %d\n",
			 ret);
		goto err_free_pages_array;
	}

	gmap->map_ops = kcalloc(gmap->grant_ref_count,
				sizeof(struct gnttab_map_grant_ref),
				GFP_KERNEL);
	if (!gmap->map_ops) {
		ret = -ENOMEM;
		goto err_free_grant_pages;
	}

	gmap->unmap_ops = kcalloc(gmap->grant_ref_count,
				  sizeof(struct gnttab_unmap_grant_ref),
				  GFP_KERNEL);
	if (!gmap->unmap_ops) {
		ret = -ENOMEM;
		goto err_free_map_ops;
	}

	for (i = 0; i < gmap->grant_ref_count; i++) {
		unsigned long pfn = page_to_pfn(gmap->pages[i]);

		gnttab_set_map_op(&gmap->map_ops[i],
				  (unsigned long)pfn_to_kaddr(pfn),
				  GNTMAP_host_map | (rw ? 0 : GNTMAP_readonly),
				  refs[i], 0 /* Dom0 */);
		gnttab_set_unmap_op(&gmap->unmap_ops[i],
				    (unsigned long)pfn_to_kaddr(pfn),
				    GNTMAP_host_map | (rw ? 0 : GNTMAP_readonly),
				    0 /* handle - filled after map */);
	}

	ret = gnttab_map_refs(gmap->map_ops, NULL, gmap->pages,
			      gmap->grant_ref_count);
	if (ret) {
		v4l2_err(&vv->v4l2_dev, "gnttab_map_refs failed: %d\n", ret);
		goto err_free_unmap_ops;
	}

	for (i = 0; i < gmap->grant_ref_count; i++) {
		if (gmap->map_ops[i].status != GNTST_okay) {
			v4l2_err(&vv->v4l2_dev,
				 "grant map op %d failed: status %d\n", i,
				 gmap->map_ops[i].status);
			ret = -ENOMEM;
			goto err_unmap_refs;
		}
		gmap->unmap_ops[i].handle = gmap->map_ops[i].handle;
	}

	list_add_tail(&gmap->list, &queue->grant_maps);
	if (refs_coherent)
		dma_free_coherent(vv->cmd_dma_dev, refs_size, refs_coherent,
				  refs_coherent_dma);
	kfree(refs);
	return 0;

err_unmap_refs:
	gnttab_unmap_refs(gmap->unmap_ops, NULL, gmap->pages,
			  gmap->grant_ref_count);
err_free_unmap_ops:
	kfree(gmap->unmap_ops);
err_free_map_ops:
	kfree(gmap->map_ops);
err_free_grant_pages:
	gnttab_free_pages(gmap->grant_ref_count, gmap->pages);
err_free_pages_array:
	kfree(gmap->pages);
err_free_gmap:
	kfree(gmap);
err_free_coherent:
	if (refs_coherent)
		dma_free_coherent(vv->cmd_dma_dev, refs_size, refs_coherent,
				  refs_coherent_dma);
err_free_refs:
	kfree(refs);
end:
	return ret;
}

/*
 * Create grant maps for every plane of a single MMAP buffer. Called from
 * QUERYBUF once the host has returned the per-plane mem_offsets. Idempotent.
 */
int virtio_media_map_buffer(struct virtio_media_session *session, u32 type,
			    u32 index)
{
	struct virtio_media_queue_state *queue;
	struct virtio_media_buffer *buffer;
	/* Grant the host RW access so it can fill CAPTURE and read OUTPUT. */
	bool rw = true;
	int ret;

	if (type > VIRTIO_MEDIA_LAST_QUEUE)
		return -EINVAL;

	queue = &session->queues[type];
	if (index >= queue->allocated_bufs || !queue->buffers)
		return -EINVAL;

	buffer = &queue->buffers[index];

	if (V4L2_TYPE_IS_MULTIPLANAR(type)) {
		u32 num_planes = buffer->buffer.length;
		u32 plane;

		if (num_planes > VIDEO_MAX_PLANES)
			return -EINVAL;
		for (plane = 0; plane < num_planes; plane++) {
			ret = virtio_media_create_grant_map(
				session, type, index, plane,
				buffer->planes[plane].m.mem_offset, rw);
			if (ret)
				return ret;
		}
	} else {
		ret = virtio_media_create_grant_map(session, type, index, 0,
						    buffer->buffer.m.offset, rw);
		if (ret)
			return ret;
	}

	return 0;
}

/* Free all grant maps of a single queue (e.g. on REQBUFS(0)). */
void virtio_media_free_queue_grant_maps(struct virtio_media_session *session,
					u32 type)
{
	struct virtio_media *vv = to_virtio_media(session->fh.vdev);
	struct virtio_media_grant_map *gmap, *tmp;

	if (type > VIRTIO_MEDIA_LAST_QUEUE)
		return;

	/*
	 * Caller must hold vv->vlock: it protects the grant_maps list we walk
	 * and unlink from here. The gmap_put() below is itself lock-free (it
	 * only decrements the refcount and, on the last ref, queues the gmap
	 * for the release worker), so it is safe to call while holding vlock --
	 * the worker takes vlock later, from its own context.
	 */
	lockdep_assert_held(&vv->vlock);
	list_for_each_entry_safe(gmap, tmp, &session->queues[type].grant_maps,
				 list) {
		/*
		 * Unlink first, then drop the owning-queue reference. If a
		 * dma-buf or userspace mmap of this buffer is still alive it
		 * holds its own reference, so the grant references and pages
		 * survive (unmapped only once the last reference is dropped)
		 * and Dom0 never sees an in-use grant entry being torn down.
		 *
		 * Unlinking here is what invalidates this buffer's identity: a
		 * source change re-REQBUFS the queue and the host reuses the
		 * same (index, mem_offset) for the new buffers. Because the old
		 * gmap is off the grant_maps list, neither the QUERYBUF path
		 * (virtio_media_find_grant_map by index) nor the mmap path
		 * (virtio_media_find_grant_map_by_offset) can find it, so the
		 * new buffer always gets a fresh gmap and a fresh MMAP command
		 * (new grant refs) rather than aliasing the still-referenced
		 * old one. The host-side daemon must likewise not reuse its old
		 * mmap entry for the reused offset (it marks it stale on
		 * REQBUFS); the two sides together mirror vb2 removing the old
		 * buffer from the queue while its memory is refcounted away.
		 */
		list_del(&gmap->list);
		virtio_media_gmap_put(vv, gmap);
	}
}

/* Free all grant maps of all queues (session close safety net). */
void virtio_media_free_session_grant_maps(struct virtio_media_session *session)
{
	struct virtio_media *vv = to_virtio_media(session->fh.vdev);
	int i;

	/*
	 * free_queue_grant_maps() requires vv->vlock to walk and unlink the
	 * grant_maps lists; the close path does not already hold it, so take it
	 * here. The teardown itself (MUNMAP) is done later by the release
	 * worker; callers that need it completed before proceeding must
	 * flush_work(&vv->gmap_release_work) afterwards (see
	 * virtio_media_session_free()).
	 */
	mutex_lock(&vv->vlock);
	for (i = 0; i <= VIRTIO_MEDIA_LAST_QUEUE; i++)
		virtio_media_free_queue_grant_maps(session, i);
	mutex_unlock(&vv->vlock);
}


/**
 * struct virtio_media_vma_handler - refcount handler for a mmapped grant map.
 * @vv: virtio-media device the mapping belongs to.
 * @gmap: grant map backing this VMA.
 * @refs: number of VMAs currently sharing this handler. Governs only the
 *	lifetime of the handler allocation itself; each of those VMAs also
 *	holds one reference on @gmap (so @refs equals this handler's
 *	contribution to gmap->refs, but the two counts are tracked separately
 *	to keep their responsibilities distinct: @refs frees the handler,
 *	gmap->refs frees the grant map).
 *
 * Stored in vma->vm_private_data so that fork()/partial-unmap of a userspace
 * mapping of a MMAP buffer keeps the grant map alive for exactly as long as
 * any VMA references it, mirroring vb2_common_vm_ops.
 */
struct virtio_media_vma_handler {
	struct virtio_media *vv;
	struct virtio_media_grant_map *gmap;
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
	virtio_media_gmap_get(h->gmap);
}

static void virtio_media_vm_close(struct vm_area_struct *vma)
{
	struct virtio_media_vma_handler *h = vma->vm_private_data;
	struct virtio_media *vv = h->vv;
	struct virtio_media_grant_map *gmap = h->gmap;

	/*
	 * Drop this VMA's grant-map reference, then its handler reference.
	 * Read h->vv/h->gmap into locals first: once the handler reference is
	 * dropped below h may be freed, so it must not be dereferenced after.
	 *
	 * vm_close() runs under mmap_lock, but virtio_media_gmap_put() is
	 * lock-free and never tears the gmap down inline (the MUNMAP, which
	 * sleeps waiting for the host, is deferred to the release worker), so
	 * it is safe to call here without stalling the address space or
	 * risking an mmap_lock -> vlock deadlock.
	 */
	virtio_media_gmap_put(vv, gmap);
	if (refcount_dec_and_test(&h->refs))
		kfree(h);
}

static const struct vm_operations_struct virtio_media_vm_ops = {
	.open = virtio_media_vm_open,
	.close = virtio_media_vm_close,
};

/**
 * virtio_media_device_mmap - Perform a mmap request from userspace.
 * @file: opened file of the session to map for.
 * @vma: VM area struct describing the desired mapping.
 *
 * This requests the host to map a MMAP buffer using Xen grant references,
 * then maps the grant-backed pages into user-space address space.
 */
static int virtio_media_device_mmap(struct file *file,
				    struct vm_area_struct *vma)
{
	struct video_device *video_dev = video_devdata(file);
	struct virtio_media *vv = to_virtio_media(video_dev);
	struct virtio_media_session *session =
		fh_to_session(file->private_data);
	struct virtio_media_grant_map *gmap;
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
	 * virtio_media_free_queue_grant_maps(), session close). Without it a
	 * concurrent REQBUFS could free @gmap and its pages while we walk the
	 * list and dereference gmap->pages here, causing a use-after-free. The
	 * inner queues_lock keeps the ordering consistent with expbuf
	 * (vv->vlock -> queues_lock) and guards the grant_maps list walk.
	 */
	mutex_lock(&vv->vlock);
	mutex_lock(&session->queues_lock);
	gmap = virtio_media_find_grant_map_by_offset(session, offset);
	if (!gmap) {
		dev_dbg(&video_dev->dev,
			"mmap for offset 0x%x with no grant map (QUERYBUF first?)\n",
			offset);
		ret = -EINVAL;
		goto unlock;
	}

	/* We cannot let the mapping be larger than the buffer. */
	if (vma->vm_end - vma->vm_start > PAGE_ALIGN(gmap->len)) {
		dev_dbg(&video_dev->dev,
			"invalid MMAP, as it would overflow buffer length\n");
		ret = -EINVAL;
		goto unlock;
	}

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
	for (i = 0; i < gmap->grant_ref_count && addr < vma->vm_end; i++) {
		ret = vm_insert_page(vma, addr, gmap->pages[i]);
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
	handler->gmap = gmap;
	refcount_set(&handler->refs, 1);
	virtio_media_gmap_get(gmap);
	vma->vm_ops = &virtio_media_vm_ops;
	vma->vm_private_data = handler;

	ret = 0;
unlock:
	mutex_unlock(&session->queues_lock);
	mutex_unlock(&vv->vlock);
	return ret;
}



/* dma-buf exporter for VIDIOC_EXPBUF support */

struct virtio_media_dmabuf_priv {
	struct file *file;
	struct virtio_media *vv;
	struct virtio_media_grant_map *gmap;
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
	virtio_media_gmap_put(priv->vv, priv->gmap);

	/*
	 * Drop the reference on the session's open file taken in
	 * virtio_media_expbuf().
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
	 * Same rationale as virtio_media_device_mmap(): the pages are real
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
	 * virtio_media_device_mmap(). userspace may close() the dma-buf fd
	 * while keeping the mapping, which drops the dma-buf's own grant-map
	 * reference; without a reference for the mapping itself the grant map
	 * could be torn down (grant refs unmapped + MUNMAP) while the VMA still
	 * points at those grant pages, making Dom0 log "g.e. 0x... still in
	 * use!". Set vma->vm_ops only after every page is inserted (see the
	 * matching comment in virtio_media_device_mmap()).
	 */
	handler = kzalloc(sizeof(*handler), GFP_KERNEL);
	if (!handler)
		return -ENOMEM;
	handler->vv = priv->vv;
	handler->gmap = priv->gmap;
	refcount_set(&handler->refs, 1);
	virtio_media_gmap_get(priv->gmap);
	vma->vm_ops = &virtio_media_vm_ops;
	vma->vm_private_data = handler;

	return 0;
}

static const struct dma_buf_ops virtio_media_dmabuf_ops = {
	.map_dma_buf = virtio_media_dmabuf_map,
	.unmap_dma_buf = virtio_media_dmabuf_unmap,
	.release = virtio_media_dmabuf_release,
	.mmap = virtio_media_dmabuf_mmap,
};

int virtio_media_expbuf(struct file *file, void *fh,
			struct v4l2_exportbuffer *eb)
{
	struct virtio_media_session *session =
		fh_to_session(file->private_data);
	struct virtio_media_grant_map *gmap;
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
	mutex_lock(&session->queues_lock);
	gmap = virtio_media_find_grant_map(session, eb->type, eb->index,
					   eb->plane);
	if (!gmap) {
		mutex_unlock(&session->queues_lock);
		return -EINVAL;
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		mutex_unlock(&session->queues_lock);
		return -ENOMEM;
	}

	priv->vv = to_virtio_media(session->fh.vdev);
	priv->gmap = gmap;
	priv->pages = gmap->pages;
	priv->num_pages = gmap->grant_ref_count;
	/*
	 * Keep the grant map (and its grant pages) alive for as long as this
	 * dma-buf exists, even if the buffers are freed with REQBUFS(0) or the
	 * session is closed first. The reference is dropped in
	 * virtio_media_dmabuf_release(). Taken under queues_lock while the gmap
	 * is still guaranteed to be live.
	 */
	virtio_media_gmap_get(gmap);
	/*
	 * Also reference the session's open file so the session (and thus the
	 * device used to send MUNMAP at release time) stays valid until the
	 * dma-buf is released.
	 */
	priv->file = get_file(session->file);

	exp_info.ops = &virtio_media_dmabuf_ops;
	/*
	 * The dma-buf is mapped page by page (grant_ref_count pages), so its
	 * size must be the page-aligned page count rather than the raw byte
	 * length. A non page-aligned gmap->len would make the kernel dma-buf
	 * mmap path reject user mappings whose (page-rounded) length exceeds
	 * dmabuf->size.
	 */
	exp_info.size = (u64)gmap->grant_ref_count << PAGE_SHIFT;
	exp_info.flags = eb->flags;
	exp_info.priv = priv;

	dmabuf = dma_buf_export(&exp_info);
	mutex_unlock(&session->queues_lock);
	if (IS_ERR(dmabuf)) {
		/*
		 * The release callback is not called when export fails, so
		 * undo the references taken above by hand.
		 */
		virtio_media_gmap_put(priv->vv, priv->gmap);
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
	init_llist_head(&vv->gmap_release_list);
	INIT_WORK(&vv->gmap_release_work, virtio_media_gmap_release_work);

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

	for (i = 0; i < VIRTIO_MEDIA_NUM_EVENT_BUFS; i++) {
		void *ebuf = vv->event_buffer + VIRTIO_MEDIA_EVENT_MAX_SIZE * i;

		ret = virtio_media_send_event_buffer(vv, ebuf);
		if (ret)
			goto err_send_event_buffer;
	}

	virtio_device_ready(virtio_dev);

	return 0;

err_send_event_buffer:
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

	/*
	 * Tear the sessions down while the command queue is still alive.
	 * virtio_media_session_free() drops each grant map's owning-queue
	 * reference via virtio_media_gmap_put(), which -- on the last reference
	 * -- queues the gmap for the release worker (it never destroys inline).
	 * So this both frees the sessions and (re)schedules gmap_release_work.
	 */
	list_for_each_safe(p, n, &vv->sessions) {
		struct virtio_media_session *s =
			list_entry(p, struct virtio_media_session, list);

		virtio_media_session_free(vv, s, false);
	}

	/*
	 * Now drain the deferred grant-map releases. The release worker sends
	 * MUNMAP commands on the command queue, so it must run *before* the
	 * device is reset and the vqs are deleted. cancel_work_sync() runs any
	 * pending worker to completion and prevents it from being requeued
	 * afterwards -- and since every session has already been freed above,
	 * nothing can schedule the worker again after this point. This must
	 * come after the session loop (which is what schedules the final
	 * releases) to avoid leaving a worker queued that would later run
	 * against a reset device and a freed @vv (use-after-free).
	 */
	cancel_work_sync(&vv->gmap_release_work);

	virtio_reset_device(virtio_dev);

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
