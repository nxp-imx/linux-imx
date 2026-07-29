// SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0+

/*
 * Ioctl implementations for the virtio-media driver.
 *
 * Copyright (c) 2024-2026 Google LLC.
 */

#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/videodev2.h>
#include <linux/virtio_config.h>
#include <linux/vmalloc.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>

#include "scatterlist_builder.h"
#include "virtio_media.h"

/**
 * virtio_media_send_r_ioctl() - Send a read-only ioctl to the device.
 * @fh: file handler of the session doing the ioctl.
 * @ioctl: ``VIDIOC_*`` ioctl code.
 * @ioctl_data: pointer to the ioctl payload.
 * @ioctl_data_len: length in bytes of the ioctl payload.
 *
 * Send an ioctl that has no driver payload, but expects a response from the
 * host (i.e. an ioctl specified with ``_IOR``).
 */
static int virtio_media_send_r_ioctl(struct v4l2_fh *fh, u32 ioctl,
				     void *ioctl_data, size_t ioctl_data_len)
{
	struct video_device *video_dev = fh->vdev;
	struct virtio_media *vv = to_virtio_media(video_dev);
	struct virtio_media_session *session = fh_to_session(fh);
	struct scatterlist *sgs[3];
	struct scatterlist_builder builder = {
		.descs = session->command_sgs.sgl,
		.num_descs = DESC_CHAIN_MAX_LEN,
		.cur_desc = 0,
		.shadow_buffer = session->shadow_buf,
		.shadow_buffer_size = VIRTIO_SHADOW_BUF_SIZE,
		.shadow_buffer_pos = 0,
		.shadow_buffer_dma = session->shadow_buf_dma,
		.premapped = vv->use_coherent,
		.sgs = sgs,
		.num_sgs = ARRAY_SIZE(sgs),
		.cur_sg = 0,
	};

	/* Command descriptor */
	int ret = scatterlist_builder_add_ioctl_cmd(&builder, session, ioctl);

	if (ret)
		return ret;

	/* Response descriptor */
	ret = scatterlist_builder_add_ioctl_resp(&builder, session);
	if (ret)
		return ret;

	/* Response payload */
	ret = scatterlist_builder_add_data(&builder, ioctl_data,
					   ioctl_data_len);
	if (ret) {
		v4l2_err(&vv->v4l2_dev,
			 "failed to prepare command descriptor chain\n");
		return ret;
	}

	ret = virtio_media_send_command(vv, sgs, 1, 2, vv->use_coherent,
					sizeof(struct virtio_media_resp_ioctl) +
					ioctl_data_len, NULL);
	if (ret < 0)
		return ret;

	ret = scatterlist_builder_retrieve_data(&builder, 2, ioctl_data);
	if (ret) {
		v4l2_err(&vv->v4l2_dev,
			 "failed to retrieve response descriptor chain\n");
		return ret;
	}

	return 0;
}

/**
 * virtio_media_send_w_ioctl() - Send a write-only ioctl to the device.
 * @fh: file handler of the session doing the ioctl.
 * @ioctl: ``VIDIOC_*`` ioctl code.
 * @ioctl_data: pointer to the ioctl payload.
 * @ioctl_data_len: length in bytes of the ioctl payload.
 *
 * Send an ioctl that does not expect a reply beyond an error status (i.e. an
 * ioctl specified with ``_IOW``) to the host.
 */
static int virtio_media_send_w_ioctl(struct v4l2_fh *fh, u32 ioctl,
				     const void *ioctl_data,
				     size_t ioctl_data_len)
{
	struct video_device *video_dev = fh->vdev;
	struct virtio_media *vv = to_virtio_media(video_dev);
	struct virtio_media_session *session = fh_to_session(fh);
	struct scatterlist *sgs[3];
	struct scatterlist_builder builder = {
		.descs = session->command_sgs.sgl,
		.num_descs = DESC_CHAIN_MAX_LEN,
		.cur_desc = 0,
		.shadow_buffer = session->shadow_buf,
		.shadow_buffer_size = VIRTIO_SHADOW_BUF_SIZE,
		.shadow_buffer_pos = 0,
		.shadow_buffer_dma = session->shadow_buf_dma,
		.premapped = vv->use_coherent,
		.sgs = sgs,
		.num_sgs = ARRAY_SIZE(sgs),
		.cur_sg = 0,
	};

	/* Command descriptor */
	int ret = scatterlist_builder_add_ioctl_cmd(&builder, session, ioctl);

	if (ret)
		return ret;

	/* Command payload */
	ret = scatterlist_builder_add_data(&builder, (void *)ioctl_data,
					   ioctl_data_len);
	if (ret) {
		v4l2_err(&vv->v4l2_dev,
			 "failed to prepare command descriptor chain\n");
		return ret;
	}

	/* Response descriptor */
	ret = scatterlist_builder_add_ioctl_resp(&builder, session);
	if (ret)
		return ret;

	ret = virtio_media_send_command(vv, sgs, 2, 1, vv->use_coherent,
					sizeof(struct virtio_media_resp_ioctl),
					NULL);
	if (ret < 0)
		return ret;

	return 0;
}

/**
 * virtio_media_send_wr_ioctl() - Send a read-write ioctl to the device.
 * @fh: file handler of the session doing the ioctl.
 * @ioctl: ``VIDIOC_*`` ioctl code.
 * @ioctl_data: pointer to the ioctl payload.
 * @ioctl_data_len: length in bytes of the ioctl payload.
 * @minimum_resp_payload: minimum expected length of the response's payload.
 *
 * Sends an ioctl that expects a response of exactly the same size as the
 * input (i.e. an ioctl specified with ``_IOWR``) to the host.
 *
 * This corresponds to what most V4L2 ioctls do. For instance
 * ``VIDIOC_ENUM_FMT`` takes a partially-initialized &struct v4l2_fmtdesc
 * and returns its filled version.
 */
static int virtio_media_send_wr_ioctl(struct v4l2_fh *fh, u32 ioctl,
				      void *ioctl_data, size_t ioctl_data_len,
				      size_t minimum_resp_payload)
{
	struct video_device *video_dev = fh->vdev;
	struct virtio_media *vv = to_virtio_media(video_dev);
	struct virtio_media_session *session = fh_to_session(fh);
	struct scatterlist *sgs[4];
	struct scatterlist_builder builder = {
		.descs = session->command_sgs.sgl,
		.num_descs = DESC_CHAIN_MAX_LEN,
		.cur_desc = 0,
		.shadow_buffer = session->shadow_buf,
		.shadow_buffer_size = VIRTIO_SHADOW_BUF_SIZE,
		.shadow_buffer_pos = 0,
		.shadow_buffer_dma = session->shadow_buf_dma,
		.premapped = vv->use_coherent,
		.sgs = sgs,
		.num_sgs = ARRAY_SIZE(sgs),
		.cur_sg = 0,
	};

	/* Command descriptor */
	int ret = scatterlist_builder_add_ioctl_cmd(&builder, session, ioctl);

	if (ret)
		return ret;

	/* Command payload */
	ret = scatterlist_builder_add_data(&builder, ioctl_data,
					   ioctl_data_len);
	if (ret) {
		v4l2_err(&vv->v4l2_dev,
			 "failed to prepare command descriptor chain\n");
		return ret;
	}

	/* Response descriptor */
	ret = scatterlist_builder_add_ioctl_resp(&builder, session);
	if (ret)
		return ret;

	/* Response payload, same as command */
	ret = scatterlist_builder_add_descriptor(&builder, 1);
	if (ret)
		return ret;

	ret = virtio_media_send_command(vv, sgs, 2, 2, vv->use_coherent,
					sizeof(struct virtio_media_resp_ioctl) +
						minimum_resp_payload,
					NULL);
	if (ret < 0)
		return ret;

	ret = scatterlist_builder_retrieve_data(&builder, 3, ioctl_data);
	if (ret) {
		v4l2_err(&vv->v4l2_dev,
			 "failed to retrieve response descriptor chain\n");
		return ret;
	}

	return 0;
}

/**
 * virtio_media_send_buffer_ioctl() - Send an ioctl taking a buffer as
 * parameter to the device.
 * @fh: file handler of the session doing the ioctl.
 * @ioctl: ``VIDIOC_*`` ioctl code.
 * @b: &struct v4l2_buffer to be sent as the ioctl payload.
 *
 * Buffers can require an additional descriptor to send their planes array, and
 * can have pointers to userspace memory hence this dedicated function.
 */
static int virtio_media_send_buffer_ioctl(struct v4l2_fh *fh, u32 ioctl,
					  struct v4l2_buffer *b)
{
	struct video_device *video_dev = fh->vdev;
	struct virtio_media *vv = to_virtio_media(video_dev);
	struct virtio_media_session *session = fh_to_session(fh);
	struct v4l2_plane *orig_planes = NULL;
	struct scatterlist *sgs[64];
	/*
	 * End of the device-readable buffer SGs, to reuse in device-writable
	 * section.
	 */
	size_t num_cmd_sgs;
	size_t end_buf_sg;
	struct scatterlist_builder builder = {
		.descs = session->command_sgs.sgl,
		.num_descs = DESC_CHAIN_MAX_LEN,
		.cur_desc = 0,
		.shadow_buffer = session->shadow_buf,
		.shadow_buffer_size = VIRTIO_SHADOW_BUF_SIZE,
		.shadow_buffer_pos = 0,
		.shadow_buffer_dma = session->shadow_buf_dma,
		.premapped = vv->use_coherent,
		.sgs = sgs,
		.num_sgs = ARRAY_SIZE(sgs),
		.cur_sg = 0,
	};
	u32 saved_fds[VIDEO_MAX_PLANES];
	bool fds_substituted = false;
	struct virtio_media_dmabuf_uuid uuid_footers[VIDEO_MAX_PLANES];
	unsigned int num_uuid_footers = 0;
	size_t resp_len;
	int ret;
	int i;

	if (b->type > VIRTIO_MEDIA_LAST_QUEUE)
		return -EINVAL;

	if (V4L2_TYPE_IS_MULTIPLANAR(b->type))
		orig_planes = b->m.planes;

	/*
	 * On the wire, a DMABUF plane is identified one of two ways for
	 * QBUF/PREPARE_BUF (the only buffer ioctls that bind planes;
	 * QUERYBUF/DQBUF carry no meaningful dma-buf fds):
	 *
	 *  - guest-pages dma-buf (guest-pages): m.fd carries the backend
	 *    resource id of its bound dma-buf, not the guest fd (meaningless to
	 *    the device). Save the guest fds and substitute the resource ids
	 *    for the duration of the command, then restore them so the V4L2
	 *    core copies the original fds back to userspace.
	 *  - host-object dma-buf (UUID-backed): the plane carries no resource
	 *    id; instead a per-plane UUID footer is appended to the
	 *    device-readable chain (below) and m.fd is left untouched.
	 *
	 * A buffer's planes are all one kind or all the other. collect_uuids()
	 * reports which: num_uuid_footers > 0 means host-object, in which case
	 * substitute_resource_ids() leaves m.fd alone.
	 */
	if (b->memory == V4L2_MEMORY_DMABUF &&
	    (ioctl == VIDIOC_QBUF || ioctl == VIDIOC_PREPARE_BUF)) {
		ret = virtio_media_dmabuf_collect_uuids(session, b, uuid_footers,
							&num_uuid_footers);
		if (ret < 0)
			return ret;

		ret = virtio_media_dmabuf_substitute_resource_ids(session, b,
								  saved_fds);
		if (ret < 0)
			return ret;
		/*
		 * Host-object buffers leave m.fd untouched (substitute is a
		 * no-op for them), so there is nothing to restore. Only mark
		 * for restore when we actually substituted resource ids
		 * (guest-pages buffers).
		 */
		if (num_uuid_footers == 0)
			fds_substituted = true;
	}

	/* Command descriptor */
	ret = scatterlist_builder_add_ioctl_cmd(&builder, session, ioctl);
	if (ret)
		goto restore_fds;

	/* Command payload (struct v4l2_buffer) */
	ret = scatterlist_builder_add_buffer(&builder, b);
	if (ret < 0)
		goto restore_fds;

	end_buf_sg = builder.cur_sg;

	/* Payload of SHARED_PAGES buffers, if relevant */
	ret = scatterlist_builder_add_buffer_userptr(&builder, b);
	if (ret < 0)
		goto restore_fds;

	/*
	 * Host-object DMABUF (UUID-backed): append one per-plane UUID footer so
	 * the device can resolve each plane's host dma-buf by its shared-object
	 * UUID. These are device-readable only (like the userptr sg-entries
	 * above), so they go before num_cmd_sgs is taken.
	 */
	for (i = 0; i < num_uuid_footers; i++) {
		ret = scatterlist_builder_add_data(&builder, &uuid_footers[i],
						   sizeof(uuid_footers[i]));
		if (ret < 0)
			goto restore_fds;
	}

	num_cmd_sgs = builder.cur_sg;

	/* Response descriptor */
	ret = scatterlist_builder_add_ioctl_resp(&builder, session);
	if (ret)
		goto restore_fds;

	/* Response payload (same as input, but no userptr mapping) */
	for (i = 1; i < end_buf_sg; i++) {
		ret = scatterlist_builder_add_descriptor(&builder, i);
		if (ret < 0)
			goto restore_fds;
	}

	ret = virtio_media_send_command(vv, builder.sgs, num_cmd_sgs,
					builder.cur_sg - num_cmd_sgs,
					vv->use_coherent,
					sizeof(struct virtio_media_resp_ioctl) +
					sizeof(*b), &resp_len);
	if (ret < 0)
		goto restore_fds;

	resp_len -= sizeof(struct virtio_media_resp_ioctl);

	/* Make sure that the reply length covers our v4l2_buffer */
	if (resp_len < sizeof(*b)) {
		ret = -EINVAL;
		goto restore_fds;
	}

	ret = scatterlist_builder_retrieve_buffer(&builder, num_cmd_sgs + 1, b,
						  orig_planes);
	if (ret) {
		v4l2_err(&vv->v4l2_dev,
			 "failed to retrieve response descriptor chain\n");
		goto restore_fds;
	}

	ret = 0;

restore_fds:
	if (fds_substituted)
		virtio_media_dmabuf_restore_resource_ids(b, saved_fds);
	return ret;
}

/**
 * virtio_media_send_ext_controls_ioctl() - Send an ioctl taking extended
 * controls as parameters to the device.
 * @fh: file handler of the session doing the ioctl.
 * @ioctl: ``VIDIOC_*`` ioctl code.
 * @ctrls: &struct v4l2_ext_controls to be sent as the ioctl payload.
 *
 * Queues an ioctl that sends a &struct v4l2_ext_controls to the host and
 * receives an updated version.
 *
 * &struct v4l2_ext_controls has a pointer to an array of
 * &struct v4l2_ext_control, and also potentially pointers to user-space memory
 * that we need to map properly, hence the dedicated function.
 */
static int virtio_media_send_ext_controls_ioctl(struct v4l2_fh *fh, u32 ioctl,
						struct v4l2_ext_controls *ctrls)
{
	struct video_device *video_dev = fh->vdev;
	struct virtio_media *vv = to_virtio_media(video_dev);
	struct virtio_media_session *session = fh_to_session(fh);
	size_t num_cmd_sgs;
	size_t end_ctrls_sg;
	struct v4l2_ext_control *controls_backup = ctrls->controls;
	const u32 num_ctrls = ctrls->count;
	/*
	 * Per-compound-control bookkeeping. Compound controls (size > 0) carry
	 * their payload through a userspace pointer in ctrl->ptr (the V4L2 core
	 * does not deep-copy it for pass-through drivers). We bounce each
	 * payload through a kernel buffer so it can be serialised inline into
	 * the descriptor chain, and remember the original userspace pointer so
	 * we can (a) copy the device's result back to it and (b) restore the
	 * ctrl->ptr field the host clobbers when it writes the array back.
	 */
	struct ext_ctrl_payload {
		void __user *user_ptr;
		void *bounce;
		u32 size;
		unsigned int ctrl_index;
		size_t desc_index;
		size_t resp_sg_index;
	} *payloads = NULL;
	unsigned int num_payloads = 0;
	struct scatterlist *sgs[64];
	struct scatterlist_builder builder = {
		.descs = session->command_sgs.sgl,
		.num_descs = DESC_CHAIN_MAX_LEN,
		.cur_desc = 0,
		.shadow_buffer = session->shadow_buf,
		.shadow_buffer_size = VIRTIO_SHADOW_BUF_SIZE,
		.shadow_buffer_pos = 0,
		.shadow_buffer_dma = session->shadow_buf_dma,
		.premapped = vv->use_coherent,
		.sgs = sgs,
		.num_sgs = ARRAY_SIZE(sgs),
		.cur_sg = 0,
	};
	size_t resp_len = 0;
	int ret;
	int i;

	/*
	 * Legacy VIDIOC_G_CTRL/S_CTRL are translated by the V4L2 core
	 * (v4l_g_ctrl()/v4l_s_ctrl()) into a single-element v4l2_ext_controls
	 * built on the stack with only ->id and ->value initialised, leaving
	 * ->size (and ->reserved2) holding stack garbage. These are always
	 * USER-class simple controls that carry their value inline and never
	 * have a payload. If we forwarded the garbage ->size, both this driver
	 * (when adding userptrs) and the host would misinterpret it as a
	 * compound control payload and fail the ioctl with -EINVAL (e.g.
	 * VIDIOC_G_CTRL on V4L2_CID_MIN_BUFFERS_FOR_CAPTURE). Compound controls
	 * are never in the USER class, so force ->size to 0 for USER-class
	 * controls before serialising them.
	 */
	for (i = 0; i < num_ctrls; i++) {
		if (V4L2_CTRL_ID2WHICH(ctrls->controls[i].id) ==
		    V4L2_CTRL_CLASS_USER)
			ctrls->controls[i].size = 0;
	}

	/*
	 * Compound controls (size > 0) carry their payload behind a userspace
	 * pointer in ctrl->ptr that the V4L2 core leaves untouched for
	 * pass-through drivers. The host serialises ext-control payloads inline
	 * (right after the v4l2_ext_control array), so bounce each payload
	 * through a kernel buffer: copy IN from userspace for S/TRY, allocate a
	 * zeroed buffer for G, and rewrite ctrl->ptr to the bounce buffer so
	 * scatterlist_builder_add_data() serialises it inline. The original
	 * userspace pointers are saved for copy-back and restoration below.
	 */
	for (i = 0; i < num_ctrls; i++) {
		if (ctrls->controls[i].size > 0)
			num_payloads++;
	}

	if (num_payloads > 0) {
		unsigned int p = 0;

		payloads = kcalloc(num_payloads, sizeof(*payloads), GFP_KERNEL);
		if (!payloads)
			return -ENOMEM;

		for (i = 0; i < num_ctrls; i++) {
			struct v4l2_ext_control *ctrl = &ctrls->controls[i];
			void __user *uptr;

			if (ctrl->size == 0)
				continue;

			uptr = ctrl->ptr;
			payloads[p].user_ptr = uptr;
			payloads[p].size = ctrl->size;
			payloads[p].ctrl_index = i;

			payloads[p].bounce = kzalloc(ctrl->size, GFP_KERNEL);
			if (!payloads[p].bounce) {
				ret = -ENOMEM;
				goto free_payloads;
			}

			/*
			 * S/TRY_EXT_CTRLS send data to the host; G_EXT_CTRLS
			 * only receives it, so its bounce stays zeroed.
			 */
			if (ioctl != VIDIOC_G_EXT_CTRLS) {
				if (copy_from_user(payloads[p].bounce, uptr,
						   ctrl->size)) {
					ret = -EFAULT;
					goto free_payloads;
				}
			}

			/* Point ctrl->ptr at the bounce buffer for serialisation. */
			ctrl->ptr = payloads[p].bounce;
			p++;
		}
	}

	/* Command descriptor */
	ret = scatterlist_builder_add_ioctl_cmd(&builder, session, ioctl);

	if (ret)
		goto free_payloads;

	/* v4l2_controls (struct + v4l2_ext_control array) */
	ret = scatterlist_builder_add_ext_ctrls(&builder, ctrls);
	if (ret)
		goto free_payloads;

	end_ctrls_sg = builder.cur_sg;

	/*
	 * Compound payloads, inline and in control order. Record each payload's
	 * descriptor index so it can be echoed into the device-writable part of
	 * the chain and retrieved afterwards.
	 */
	for (i = 0; i < num_payloads; i++) {
		payloads[i].desc_index = builder.cur_desc;
		ret = scatterlist_builder_add_data(&builder,
						   payloads[i].bounce,
						   payloads[i].size);
		if (ret)
			goto free_payloads;
	}

	num_cmd_sgs = builder.cur_sg;

	/* Response descriptor */
	ret = scatterlist_builder_add_ioctl_resp(&builder, session);
	if (ret)
		goto free_payloads;

	/* Response payload: struct + v4l2_ext_control array (device-writable). */
	for (i = 1; i < end_ctrls_sg; i++) {
		ret = scatterlist_builder_add_descriptor(&builder, i);
		if (ret < 0)
			goto free_payloads;
	}

	/*
	 * Echo the compound payload descriptors into the device-writable part
	 * so the host can write results (G/TRY) back into the same buffers.
	 */
	for (i = 0; i < num_payloads; i++) {
		payloads[i].resp_sg_index = builder.cur_sg;
		ret = scatterlist_builder_add_descriptor(&builder,
							 payloads[i].desc_index);
		if (ret < 0)
			goto free_payloads;
	}

	ret = virtio_media_send_command(vv, builder.sgs, num_cmd_sgs,
					builder.cur_sg - num_cmd_sgs,
					vv->use_coherent,
					sizeof(struct virtio_media_resp_ioctl) +
					sizeof(*ctrls),
					&resp_len);

	/* Just in case the host touched these. */
	ctrls->controls = controls_backup;
	if (ctrls->count != num_ctrls) {
		v4l2_err(&vv->v4l2_dev,
			 "device returned a number of controls different than the one submitted\n");
	}
	if (ctrls->count > num_ctrls) {
		ret = -ENOSPC;
		goto restore_ptrs;
	}

	/*
	 * Even if we have received an error, we may need to read our payload
	 * back.
	 */
	if (ret < 0 && resp_len >= sizeof(struct virtio_media_resp_ioctl) +
					   sizeof(*ctrls)) {
		/*
		 * Deliberately ignore the error here as we want to return the
		 * previous one.
		 */
		scatterlist_builder_retrieve_ext_ctrls(&builder,
						       num_cmd_sgs + 1, ctrls);
		goto retrieve_payloads;
	}

	if (ret < 0)
		goto restore_ptrs;

	resp_len -= sizeof(struct virtio_media_resp_ioctl);

	/* Make sure that the reply's length covers our v4l2_ext_controls */
	if (resp_len < sizeof(*ctrls)) {
		ret = -EINVAL;
		goto restore_ptrs;
	}

	ret = scatterlist_builder_retrieve_ext_ctrls(&builder, num_cmd_sgs + 1,
						     ctrls);
	if (ret)
		goto restore_ptrs;

retrieve_payloads:
	/*
	 * Copy the (possibly device-written) compound payloads back to their
	 * original userspace buffers. This is the read-back path for
	 * G/TRY_EXT_CTRLS compound controls (e.g. array/string controls).
	 */
	for (i = 0; i < num_payloads; i++) {
		int rret = scatterlist_builder_retrieve_data(
			&builder, payloads[i].resp_sg_index,
			payloads[i].bounce);

		if (rret == 0 && ioctl != VIDIOC_S_EXT_CTRLS) {
			if (copy_to_user(payloads[i].user_ptr,
					 payloads[i].bounce,
					 payloads[i].size) && ret == 0)
				ret = -EFAULT;
		}
	}

restore_ptrs:
	/*
	 * The host writes the whole v4l2_ext_control array back, clobbering the
	 * ctrl->ptr field of every compound control with a meaningless host
	 * address. Restore the original userspace pointers so the V4L2 core
	 * copies valid pointers back to userspace (otherwise v4l2-ctl would
	 * dereference a bogus pointer and segfault).
	 */
	for (i = 0; i < num_payloads; i++)
		ctrls->controls[payloads[i].ctrl_index].ptr =
			payloads[i].user_ptr;

	if (ret)
		goto free_payloads;

	/*
	 * Trace the value the guest reads back for MIN_BUFFERS_FOR_CAPTURE.
	 * GStreamer sizes its CAPTURE buffer pool from this control after a
	 * source change, so recording it in the flow lets us confirm that the
	 * value the host (wave6) reported (reorder_delay + 1) actually reached
	 * the guest, and correlate it with how many buffers end up queued.
	 */
	if (ioctl == VIDIOC_G_EXT_CTRLS) {
		for (i = 0; i < ctrls->count; i++) {
			if (ctrls->controls[i].id ==
			    V4L2_CID_MIN_BUFFERS_FOR_CAPTURE) {
				virtio_media_record_flow(
					session,
					VIRTIO_MEDIA_FLOW_MIN_BUFFERS,
					ctrls->controls[i].value, 0);
				break;
			}
		}
	}

free_payloads:
	/*
	 * Restore any userspace pointers still pointing at bounce buffers (the
	 * early error paths jump here before restore_ptrs) and free the bounce
	 * buffers. ctrls->controls is the kernel copy the V4L2 core writes back
	 * to userspace (INFO_FL_ALWAYS_COPY), so it must never carry a kernel
	 * pointer out of this function.
	 */
	for (i = 0; i < num_payloads; i++) {
		if (!payloads[i].bounce)
			continue;
		ctrls->controls[payloads[i].ctrl_index].ptr =
			payloads[i].user_ptr;
		kfree(payloads[i].bounce);
	}
	kfree(payloads);

	return ret;
}

/**
 * virtio_media_free_queue_buffers() - free a queue's buffer pointer array and
 * every buffer it points to.
 * @queue: queue whose buffers should be freed.
 *
 * The buffers are allocated individually (see the @buffers comment in struct
 * virtio_media_queue_state), so each one is freed before the pointer array
 * itself. Leaves @queue->buffers NULL and @allocated_bufs zero. The caller is
 * responsible for any locking and for having already emptied pending_dqbufs
 * (the buffers being freed here embed those list nodes).
 */
void virtio_media_free_queue_buffers(struct virtio_media_queue_state *queue)
{
	size_t i;

	if (!queue->buffers)
		return;

	for (i = 0; i < queue->allocated_bufs; i++)
		kfree(queue->buffers[i]);

	kvfree(queue->buffers);
	queue->buffers = NULL;
	queue->allocated_bufs = 0;
}

/**
 * virtio_media_realloc_buffers() - grow a buffer pointer array to @new_count.
 * @old_buffers: existing pointer array (may be NULL when @old_count is 0).
 * @old_count: number of buffers currently held in @old_buffers.
 * @new_count: desired number of buffers, must be >= @old_count.
 *
 * Allocates a fresh pointer array of @new_count entries, carries over the first
 * @old_count buffer pointers from @old_buffers (their bodies are not moved, so
 * addresses stay stable and any pending_dqbufs nodes embedded in them remain
 * valid), and allocates new buffer bodies for the [@old_count, @new_count)
 * range.
 *
 * This helper only ever grows the pool: @new_count must be >= @old_count.
 * Shrinking is rejected (returns NULL) rather than handled, because the
 * carry-over loop below would otherwise write past the freshly allocated
 * @new_count-entry array, and the dropped buffer bodies would leak. Keeping
 * the guard here (not just at the caller) means a future caller cannot turn a
 * shrink request into silent heap corruption.
 *
 * The @old_buffers array itself is left untouched: the caller stays its owner
 * and must free it (typically with kvfree() once the new array is swapped in).
 * On allocation failure any bodies newly allocated here are freed and NULL is
 * returned.
 */
static struct virtio_media_buffer **
virtio_media_realloc_buffers(struct virtio_media_buffer **old_buffers,
			     size_t old_count, size_t new_count)
{
	struct virtio_media_buffer **buffers;
	size_t i;

	if (!new_count || new_count < old_count)
		return NULL;

	buffers = kvcalloc(new_count, sizeof(*buffers), GFP_KERNEL);
	if (!buffers)
		return NULL;

	/* Carry over existing buffers by pointer (their bodies do not move). */
	for (i = 0; i < old_count; i++)
		buffers[i] = old_buffers[i];

	for (i = old_count; i < new_count; i++) {
		buffers[i] = kzalloc(sizeof(*buffers[i]), GFP_KERNEL);
		if (!buffers[i]) {
			while (i-- > old_count)
				kfree(buffers[i]);
			kvfree(buffers);
			return NULL;
		}
	}

	return buffers;
}

/**
 * virtio_media_alloc_buffers() - allocate a pointer array of @count buffer
 * states, each buffer allocated individually.
 * @count: number of buffers to allocate.
 *
 * Returns the pointer array on success (caller owns it and must free it with
 * virtio_media_free_queue_buffers() semantics), or NULL on allocation failure
 * (any partial allocation is cleaned up before returning).
 */
static struct virtio_media_buffer **virtio_media_alloc_buffers(size_t count)
{
	return virtio_media_realloc_buffers(NULL, 0, count);
}

/**
 * virtio_media_clear_queue_locked() - clear all pending buffers on a
 *                                     streamed-off queue.
 * @queue: state of the queue to clear.
 *
 * Must be called with @session->queues_lock held. See
 * virtio_media_clear_queue() for the locked wrapper used by STREAMOFF.
 */
static void virtio_media_clear_queue_locked(struct virtio_media_queue_state *queue)
{
	struct list_head *p, *n;
	int i;

	list_for_each_safe(p, n, &queue->pending_dqbufs) {
		struct virtio_media_buffer *dqbuf =
			list_entry(p, struct virtio_media_buffer, list);

		list_del(&dqbuf->list);
	}

	/* All buffers are now dequeued. */
	for (i = 0; i < queue->allocated_bufs; i++) {
		queue->buffers[i]->buffer.flags = 0;
		queue->buffers[i]->prepared = false;
	}

	queue->queued_bufs = 0;
	queue->streaming = false;
	queue->is_capture_last = false;
}

/**
 * virtio_media_clear_queue() - clear all pending buffers on a streamed-off
 *                              queue.
 * @session: session which the queue to clear belongs to.
 * @queue: state of the queue to clear.
 *
 * Helper function to clear the list of buffers waiting to be dequeued on a
 * queue that has just been streamed off.
 */
static void virtio_media_clear_queue(struct virtio_media_session *session,
				     struct virtio_media_queue_state *queue)
{
	scoped_guard(mutex, &session->queues_lock)
		virtio_media_clear_queue_locked(queue);
}

/*
 * Macros suitable for defining ioctls with a constant size payload.
 */

#define SIMPLE_WR_IOCTL(name, ioctl, payload_t)                       \
	static int virtio_media_##name(struct file *file, void *fh,   \
				       payload_t *payload)            \
	{                                                             \
		struct v4l2_fh *vfh = file_to_v4l2_fh(file);          \
		return virtio_media_send_wr_ioctl(vfh, ioctl, payload,\
						  sizeof(*payload),   \
						  sizeof(*payload));  \
	}
#define SIMPLE_R_IOCTL(name, ioctl, payload_t)                       \
	static int virtio_media_##name(struct file *file, void *fh,  \
				       payload_t *payload)           \
	{                                                            \
		struct v4l2_fh *vfh = file_to_v4l2_fh(file);         \
		return virtio_media_send_r_ioctl(vfh, ioctl, payload,\
						 sizeof(*payload));  \
	}
#define SIMPLE_W_IOCTL(name, ioctl, payload_t)                       \
	static int virtio_media_##name(struct file *file, void *fh,  \
				       payload_t *payload)           \
	{                                                            \
		struct v4l2_fh *vfh = file_to_v4l2_fh(file);         \
		return virtio_media_send_w_ioctl(vfh, ioctl, payload,\
						 sizeof(*payload));  \
	}

/*
 * V4L2 ioctl handlers.
 *
 * Most of these functions just forward the ioctl to the host, for these we can
 * use one of the SIMPLE_*_IOCTL macros. Exceptions that have their own
 * standalone function follow.
 */

SIMPLE_WR_IOCTL(enum_fmt, VIDIOC_ENUM_FMT, struct v4l2_fmtdesc)
SIMPLE_WR_IOCTL(g_fmt, VIDIOC_G_FMT, struct v4l2_format)
SIMPLE_WR_IOCTL(s_fmt, VIDIOC_S_FMT, struct v4l2_format)
SIMPLE_WR_IOCTL(try_fmt, VIDIOC_TRY_FMT, struct v4l2_format)
SIMPLE_WR_IOCTL(enum_framesizes, VIDIOC_ENUM_FRAMESIZES,
		struct v4l2_frmsizeenum)
SIMPLE_WR_IOCTL(enum_frameintervals, VIDIOC_ENUM_FRAMEINTERVALS,
		struct v4l2_frmivalenum)
SIMPLE_WR_IOCTL(query_ext_ctrl, VIDIOC_QUERY_EXT_CTRL,
		struct v4l2_query_ext_ctrl)
SIMPLE_WR_IOCTL(s_dv_timings, VIDIOC_S_DV_TIMINGS, struct v4l2_dv_timings)
SIMPLE_WR_IOCTL(g_dv_timings, VIDIOC_G_DV_TIMINGS, struct v4l2_dv_timings)
SIMPLE_R_IOCTL(query_dv_timings, VIDIOC_QUERY_DV_TIMINGS,
	       struct v4l2_dv_timings)
SIMPLE_WR_IOCTL(enum_dv_timings, VIDIOC_ENUM_DV_TIMINGS,
		struct v4l2_enum_dv_timings)
SIMPLE_WR_IOCTL(dv_timings_cap, VIDIOC_DV_TIMINGS_CAP,
		struct v4l2_dv_timings_cap)
SIMPLE_WR_IOCTL(enuminput, VIDIOC_ENUMINPUT, struct v4l2_input)
SIMPLE_WR_IOCTL(querymenu, VIDIOC_QUERYMENU, struct v4l2_querymenu)
SIMPLE_WR_IOCTL(enumoutput, VIDIOC_ENUMOUTPUT, struct v4l2_output)
SIMPLE_WR_IOCTL(enumaudio, VIDIOC_ENUMAUDIO, struct v4l2_audio)
SIMPLE_R_IOCTL(g_audio, VIDIOC_G_AUDIO, struct v4l2_audio)
SIMPLE_W_IOCTL(s_audio, VIDIOC_S_AUDIO, const struct v4l2_audio)
SIMPLE_WR_IOCTL(enumaudout, VIDIOC_ENUMAUDOUT, struct v4l2_audioout)
SIMPLE_R_IOCTL(g_audout, VIDIOC_G_AUDOUT, struct v4l2_audioout)
SIMPLE_W_IOCTL(s_audout, VIDIOC_S_AUDOUT, const struct v4l2_audioout)
SIMPLE_WR_IOCTL(g_modulator, VIDIOC_G_MODULATOR, struct v4l2_modulator)
SIMPLE_W_IOCTL(s_modulator, VIDIOC_S_MODULATOR, const struct v4l2_modulator)
SIMPLE_WR_IOCTL(g_selection, VIDIOC_G_SELECTION, struct v4l2_selection)
SIMPLE_WR_IOCTL(s_selection, VIDIOC_S_SELECTION, struct v4l2_selection)
SIMPLE_R_IOCTL(g_enc_index, VIDIOC_G_ENC_INDEX, struct v4l2_enc_idx)
SIMPLE_WR_IOCTL(encoder_cmd, VIDIOC_ENCODER_CMD, struct v4l2_encoder_cmd)
SIMPLE_WR_IOCTL(try_encoder_cmd, VIDIOC_TRY_ENCODER_CMD,
		struct v4l2_encoder_cmd)
SIMPLE_WR_IOCTL(try_decoder_cmd, VIDIOC_TRY_DECODER_CMD,
		struct v4l2_decoder_cmd)
SIMPLE_WR_IOCTL(g_parm, VIDIOC_G_PARM, struct v4l2_streamparm)
SIMPLE_WR_IOCTL(s_parm, VIDIOC_S_PARM, struct v4l2_streamparm)
SIMPLE_R_IOCTL(g_std, VIDIOC_G_STD, v4l2_std_id)
SIMPLE_R_IOCTL(querystd, VIDIOC_QUERYSTD, v4l2_std_id)
SIMPLE_WR_IOCTL(enumstd, VIDIOC_ENUMSTD, struct v4l2_standard)
SIMPLE_WR_IOCTL(g_tuner, VIDIOC_G_TUNER, struct v4l2_tuner)
SIMPLE_W_IOCTL(s_tuner, VIDIOC_S_TUNER, const struct v4l2_tuner)
SIMPLE_WR_IOCTL(g_frequency, VIDIOC_G_FREQUENCY, struct v4l2_frequency)
SIMPLE_W_IOCTL(s_frequency, VIDIOC_S_FREQUENCY, const struct v4l2_frequency)
SIMPLE_WR_IOCTL(enum_freq_bands, VIDIOC_ENUM_FREQ_BANDS,
		struct v4l2_frequency_band)
SIMPLE_WR_IOCTL(g_sliced_vbi_cap, VIDIOC_G_SLICED_VBI_CAP,
		struct v4l2_sliced_vbi_cap)
SIMPLE_W_IOCTL(s_hw_freq_seek, VIDIOC_S_HW_FREQ_SEEK,
	       const struct v4l2_hw_freq_seek)

/*
 * QUERYCAP is handled by reading the configuration area.
 */

static int virtio_media_querycap(struct file *file, void *fh,
				 struct v4l2_capability *cap)
{
	struct video_device *video_dev = video_devdata(file);
	struct virtio_media *vv = to_virtio_media(video_dev);

	strscpy(cap->bus_info, "platform:virtio-media");
	strscpy(cap->driver, VIRTIO_MEDIA_DEFAULT_DRIVER_NAME);

	virtio_cread_bytes(vv->virtio_dev, 8, cap->card, sizeof(cap->card));

	cap->capabilities = video_dev->device_caps | V4L2_CAP_DEVICE_CAPS;
	cap->device_caps = video_dev->device_caps;

	return 0;
}

/*
 * Extended control ioctls are handled mostly identically.
 */

static int virtio_media_g_ext_ctrls(struct file *file, void *fh,
				    struct v4l2_ext_controls *ctrls)
{
	struct v4l2_fh *vfh = file_to_v4l2_fh(file);

	return virtio_media_send_ext_controls_ioctl(vfh, VIDIOC_G_EXT_CTRLS,
						    ctrls);
}

static int virtio_media_s_ext_ctrls(struct file *file, void *fh,
				    struct v4l2_ext_controls *ctrls)
{
	struct v4l2_fh *vfh = file_to_v4l2_fh(file);

	return virtio_media_send_ext_controls_ioctl(vfh, VIDIOC_S_EXT_CTRLS,
						    ctrls);
}

static int virtio_media_try_ext_ctrls(struct file *file, void *fh,
				      struct v4l2_ext_controls *ctrls)
{
	struct v4l2_fh *vfh = file_to_v4l2_fh(file);

	return virtio_media_send_ext_controls_ioctl(vfh, VIDIOC_TRY_EXT_CTRLS,
						    ctrls);
}

/*
 * Subscribe/unsubscribe from an event.
 */

static int
virtio_media_subscribe_event(struct v4l2_fh *fh,
			     const struct v4l2_event_subscription *sub)
{
	struct video_device *video_dev = fh->vdev;
	struct virtio_media *vv = to_virtio_media(video_dev);
	int ret;

	/* First subscribe to the event in the guest. */
	switch (sub->type) {
	case V4L2_EVENT_SOURCE_CHANGE:
		ret = v4l2_src_change_event_subscribe(fh, sub);
		break;
	default:
		ret = v4l2_event_subscribe(fh, sub, 1, NULL);
		break;
	}
	if (ret)
		return ret;

	/* Then ask the host to signal us these events. */
	ret = virtio_media_send_w_ioctl(fh, VIDIOC_SUBSCRIBE_EVENT, sub,
					sizeof(*sub));
	if (ret < 0) {
		v4l2_event_unsubscribe(fh, sub);
		return ret;
	}

	virtio_media_record_flow(fh_to_session(fh),
				 VIRTIO_MEDIA_FLOW_SUBSCRIBE_EVENT, sub->type,
				 0);

	/*
	 * Subscribing to an event may result in that event being signaled
	 * immediately. Process all pending events to make sure we don't
	 * miss it.
	 */
	if (sub->flags & V4L2_EVENT_SUB_FL_SEND_INITIAL)
		virtio_media_process_events(vv);

	return 0;
}

static int
virtio_media_unsubscribe_event(struct v4l2_fh *fh,
			       const struct v4l2_event_subscription *sub)
{
	int ret = virtio_media_send_w_ioctl(fh, VIDIOC_UNSUBSCRIBE_EVENT, sub,
					sizeof(*sub));
	if (ret < 0)
		return ret;

	ret = v4l2_event_unsubscribe(fh, sub);
	if (ret)
		return ret;

	return 0;
}

/*
 * Streamon/off affect the local queue state.
 */

static int virtio_media_streamon(struct file *file, void *fh,
				 enum v4l2_buf_type i)
{
	struct v4l2_fh *vfh = file_to_v4l2_fh(file);
	struct virtio_media_session *session = fh_to_session(vfh);
	int ret;

	if (i > VIRTIO_MEDIA_LAST_QUEUE)
		return -EINVAL;

	ret = virtio_media_send_w_ioctl(vfh, VIDIOC_STREAMON, &i, sizeof(i));
	if (ret < 0)
		return ret;

	session->queues[i].streaming = true;
	virtio_media_record_flow(session, VIRTIO_MEDIA_FLOW_STREAMON, i, 0);

	return 0;
}

static int virtio_media_streamoff(struct file *file, void *fh,
				  enum v4l2_buf_type i)
{
	struct v4l2_fh *vfh = file_to_v4l2_fh(file);
	struct virtio_media_session *session = fh_to_session(vfh);
	int ret;

	if (i > VIRTIO_MEDIA_LAST_QUEUE)
		return -EINVAL;

	ret = virtio_media_send_w_ioctl(vfh, VIDIOC_STREAMOFF, &i, sizeof(i));
	if (ret < 0)
		return ret;

	virtio_media_clear_queue(session, &session->queues[i]);
	virtio_media_record_flow(session, VIRTIO_MEDIA_FLOW_STREAMOFF, i, 0);

	return 0;
}

/*
 * Buffer creation/queuing functions deal with the local driver state.
 */

static int virtio_media_reqbufs(struct file *file, void *fh,
				struct v4l2_requestbuffers *b)
{
	struct v4l2_fh *vfh = file_to_v4l2_fh(file);
	struct virtio_media_session *session = fh_to_session(vfh);
	struct virtio_media_queue_state *queue;
	struct virtio_media_buffer **new_buffers = NULL;
	int ret;

	if (b->type > VIRTIO_MEDIA_LAST_QUEUE)
		return -EINVAL;

	if (b->memory == V4L2_MEMORY_USERPTR && !virtio_media_allow_userptr)
		return -EINVAL;

	if (b->memory == V4L2_MEMORY_DMABUF && !virtio_media_allow_dmabuf)
		return -EINVAL;

	ret = virtio_media_send_wr_ioctl(vfh, VIDIOC_REQBUFS, b, sizeof(*b),
					 sizeof(*b));
	if (ret)
		return ret;

	virtio_media_record_flow(session, VIRTIO_MEDIA_FLOW_REQBUFS, b->type,
				 b->count);

	queue = &session->queues[b->type];

	/*
	 * Allocate the new buffer set up front (outside queues_lock) so a
	 * failure leaves the old queue state untouched. The teardown of the
	 * old buffers and install of the new set happens under queues_lock
	 * below.
	 */
	if (b->count > 0) {
		new_buffers = virtio_media_alloc_buffers(b->count);
		if (!new_buffers)
			return -ENOMEM;
	}

	/*
	 * Any host mappings created for the old buffers must be released now. This
	 * is essential on a source change, where the client re-REQBUFS the
	 * CAPTURE queue with a new count *without* an intervening REQBUFS(0):
	 * failing to unmap here would leak the old backend entries and the host
	 * could not reuse or reallocate the buffers, stalling the stream.
	 *
	 * Note we still deliberately do NOT free host mappings on STREAMOFF, so
	 * that seek (STREAMOFF/STREAMON without REQBUFS) keeps the buffers
	 * mapped. The new buffers are lazily re-mapped at QUERYBUF time
	 * (virtio_media_map_buffer() is idempotent). If a dma-buf exported from
	 * an old buffer is still alive it holds its own reference, so the pages
	 * survive until the last dma-buf is released.
	 */
	virtio_media_free_queue_host_mappings(session, b->type);

	/*
	 * Likewise release any dma-buf imports of this queue's old buffers.
	 * REQBUFS (any count) tears down the previous buffer set, and the
	 * backend has released the backing it held for them by the time this
	 * command returns, so it is safe to unmap and detach them now.
	 */
	virtio_media_free_queue_dmabuf_imports(session, b->type);

	/*
	 * REQBUFS (with *any* count) frees every buffer previously allocated on
	 * this queue before installing the new set. clear_queue drops the
	 * pending_dqbufs entries that point into the old buffers and resets
	 * per-buffer flags; it must run *before* freeing those buffers, and in
	 * the same queues_lock critical section as the free and swap so the
	 * DQBUF event worker (which walks queue->buffers under queues_lock)
	 * can never re-queue or dereference a buffer that is about to be freed.
	 */
	scoped_guard(mutex, &session->queues_lock) {
		virtio_media_clear_queue_locked(queue);
		virtio_media_free_queue_buffers(queue);

		queue->buffers = new_buffers;
		queue->allocated_bufs = b->count;
		queue->memory = b->memory;
	}

	/*
	 * If a multiplanar queue is successfully used here, this means
	 * we are using the multiplanar interface.
	 */
	if (V4L2_TYPE_IS_MULTIPLANAR(b->type))
		session->uses_mplane = true;

	if (!virtio_media_allow_userptr)
		b->capabilities &= ~V4L2_BUF_CAP_SUPPORTS_USERPTR;

	/*
	 * DMABUF import is only advertised when explicitly enabled. When
	 * enabled, the queue may be set up for V4L2_MEMORY_DMABUF and QBUF
	 * will import guest-page-backed dma-bufs (see virtio_media_qbuf()).
	 */
	if (!virtio_media_allow_dmabuf)
		b->capabilities &= ~V4L2_BUF_CAP_SUPPORTS_DMABUF;

	return 0;
}

/**
 * virtio_media_fill_buffer_prepared() - Report V4L2_BUF_FLAG_PREPARED to
 *	userspace.
 * @buffer: driver-side buffer state.
 * @b: v4l2_buffer being returned to userspace.
 *
 * Mirrors videobuf2's __fill_v4l2_buffer(): the PREPARED flag is derived from
 * the @prepared state rather than stored, and only reported while the buffer
 * sits outside the device (its authoritative driver state is neither queued
 * nor done). Called on the return path of the ioctls that hand a buffer back
 * to userspace (QUERYBUF, PREPARE_BUF, QBUF, DQBUF).
 */
static void
virtio_media_fill_buffer_prepared(struct virtio_media_buffer *buffer,
				  struct v4l2_buffer *b)
{
	if (buffer->prepared &&
	    !(buffer->buffer.flags & (V4L2_BUF_FLAG_QUEUED | V4L2_BUF_FLAG_DONE)))
		b->flags |= V4L2_BUF_FLAG_PREPARED;
	else
		b->flags &= ~V4L2_BUF_FLAG_PREPARED;
}

static int virtio_media_querybuf(struct file *file, void *fh,
				 struct v4l2_buffer *b)
{
	struct v4l2_fh *vfh = file_to_v4l2_fh(file);
	struct virtio_media_session *session = fh_to_session(vfh);
	struct virtio_media_queue_state *queue;
	struct virtio_media_buffer *buffer;

	int ret = virtio_media_send_buffer_ioctl(vfh, VIDIOC_QUERYBUF, b);

	if (ret)
		return ret;

	if (b->type > VIRTIO_MEDIA_LAST_QUEUE)
		return -EINVAL;

	queue = &session->queues[b->type];
	if (b->index >= queue->allocated_bufs)
		return -EINVAL;

	buffer = queue->buffers[b->index];

	/*
	 * Store the host-provided mem_offset(s) into our buffer state so that
	 * virtio_media_map_buffer() below (and later mmap/expbuf) can locate
	 * them. QUERYBUF is where the offsets first become known.
	 */
	buffer->buffer.m = b->m;
	buffer->buffer.length = b->length;
	if (V4L2_TYPE_IS_MULTIPLANAR(b->type)) {
		int i;

		if (b->length > VIDEO_MAX_PLANES)
			return -EINVAL;
		for (i = 0; i < b->length; i++)
			buffer->planes[i].m = b->m.planes[i].m;
	}

	/*
	 * Create the host mappingping(s) for this buffer now that the host has
	 * provided the per-plane offsets. This is idempotent, so repeated
	 * QUERYBUF calls are harmless.
	 */
	if (queue->memory == V4L2_MEMORY_MMAP) {
		ret = virtio_media_map_buffer(session, b->type, b->index);
		if (ret)
			return ret;
	}

	/*
	 * Set the DONE flag if the buffer is waiting in our own dequeue
	 * queue.
	 */
	b->flags |= (buffer->buffer.flags & V4L2_BUF_FLAG_DONE);

	/* Report whether the buffer is currently prepared. */
	virtio_media_fill_buffer_prepared(buffer, b);

	return 0;
}

static int virtio_media_create_bufs(struct file *file, void *fh,
				    struct v4l2_create_buffers *b)
{
	struct v4l2_fh *vfh = file_to_v4l2_fh(file);
	struct virtio_media_session *session = fh_to_session(vfh);
	struct virtio_media_queue_state *queue;
	struct virtio_media_buffer **new_buffers;
	struct virtio_media_buffer **old_buffers;
	u32 type = b->format.type;
	size_t new_count;
	int ret;

	if (type > VIRTIO_MEDIA_LAST_QUEUE)
		return -EINVAL;

	if (b->memory == V4L2_MEMORY_USERPTR && !virtio_media_allow_userptr)
		return -EINVAL;

	if (b->memory == V4L2_MEMORY_DMABUF && !virtio_media_allow_dmabuf)
		return -EINVAL;

	queue = &session->queues[type];

	/*
	 * CREATE_BUFS may only add buffers of the memory model the queue was
	 * already set up with (by REQBUFS or an earlier CREATE_BUFS). A queue
	 * with no buffers yet (allocated_bufs == 0) has no established memory
	 * model, so any allowed type is accepted and adopted below. Reject a
	 * mismatch here, before the host allocates buffers we would refuse.
	 */
	if (queue->allocated_bufs > 0 && b->memory != queue->memory)
		return -EINVAL;

	ret = virtio_media_send_wr_ioctl(vfh, VIDIOC_CREATE_BUFS, b, sizeof(*b),
					 sizeof(*b));
	if (ret)
		return ret;

	if (!virtio_media_allow_userptr)
		b->capabilities &= ~V4L2_BUF_CAP_SUPPORTS_USERPTR;

	if (!virtio_media_allow_dmabuf)
		b->capabilities &= ~V4L2_BUF_CAP_SUPPORTS_DMABUF;

	/* If count is zero, we were just checking for format. */
	if (b->count == 0)
		return 0;

	new_count = b->index + b->count;

	/*
	 * The pool only ever grows. If the host places the new buffers within
	 * the range already allocated (new_count <= allocated_bufs), the
	 * existing pool already covers them: skip the realloc, which also
	 * guards the carry-over loop below against writing past new_buffers.
	 */
	if (new_count <= queue->allocated_bufs) {
		queue->memory = b->memory;
		return 0;
	}

	/*
	 * Grow the pool. Because each buffer is allocated individually, only
	 * the pointer array is reallocated here: the existing buffer states
	 * keep their addresses, so the pending_dqbufs list nodes embedded in
	 * them stay valid and need no fixup. Build the new array (with the
	 * existing buffers carried over and the new ones allocated) before
	 * taking queues_lock so the critical section that swaps it in cannot
	 * fail.
	 */
	new_buffers = virtio_media_realloc_buffers(queue->buffers,
						   queue->allocated_bufs,
						   new_count);
	if (!new_buffers)
		return -ENOMEM;

	scoped_guard(mutex, &session->queues_lock) {
		old_buffers = queue->buffers;

		queue->buffers = new_buffers;
		queue->allocated_bufs = new_count;
		queue->memory = b->memory;

		/*
		 * Free the old pointer array inside the lock so the DQBUF event
		 * worker (which reads queue->buffers under queues_lock) can
		 * never observe the swapped-out array after it is freed. Only
		 * the array is freed here; the buffer bodies live on via
		 * new_buffers.
		 */
		kvfree(old_buffers);
	}

	return 0;
}

/**
 * __virtio_media_buf_prepare() - Prepare a buffer before it is queued.
 * @vfh: v4l2 file handle owning the buffer.
 * @b: v4l2_buffer being prepared (from PREPARE_BUF or QBUF).
 *
 * Mirrors videobuf2's __buf_prepare(): the preparation work (recording the
 * userspace-provided ``m`` union and, for V4L2_MEMORY_DMABUF, binding/attaching
 * each plane's dma-buf) is done at most once per queueing cycle. If the buffer
 * has already been prepared (by an earlier PREPARE_BUF) this is a no-op, so a
 * QBUF following a PREPARE_BUF reuses the existing bindings and does not resend
 * an ATTACH. The @prepared flag is cleared again on DQBUF and STREAMOFF.
 *
 * Caller must hold vv->vlock. Returns 0 on success or a negative error code;
 * on failure the buffer is left unprepared with no partial bindings.
 */
static int __virtio_media_buf_prepare(struct v4l2_fh *vfh, struct v4l2_buffer *b)
{
	struct virtio_media_session *session = fh_to_session(vfh);
	struct virtio_media_queue_state *queue = &session->queues[b->type];
	struct virtio_media_buffer *buffer = queue->buffers[b->index];
	int i, ret;

	if (buffer->prepared)
		return 0;

	/*
	 * Store the buffer and plane `m` information so we can retrieve it
	 * again when DQBUF occurs.
	 */
	buffer->buffer.m = b->m;
	if (V4L2_TYPE_IS_MULTIPLANAR(b->type)) {
		if (b->length > VIDEO_MAX_PLANES)
			return -EINVAL;
		for (i = 0; i < b->length; i++)
			buffer->planes[i].m = b->m.planes[i].m;
	}

	/* Bind (and attach) any dma-buf planes before the device sees them. */
	ret = virtio_media_bind_buffer_dmabufs(session, b);
	if (ret)
		return ret;

	buffer->prepared = true;

	return 0;
}

static int virtio_media_prepare_buf(struct file *file, void *fh,
				    struct v4l2_buffer *b)
{
	struct v4l2_fh *vfh = file_to_v4l2_fh(file);
	struct virtio_media_session *session = fh_to_session(vfh);
	struct virtio_media_queue_state *queue;
	struct virtio_media_buffer *buffer;
	int ret;

	if (b->type > VIRTIO_MEDIA_LAST_QUEUE)
		return -EINVAL;
	queue = &session->queues[b->type];
	if (b->index >= queue->allocated_bufs)
		return -EINVAL;

	/* The buffer memory type must match the queue's (see QBUF). */
	if (b->memory != queue->memory)
		return -EINVAL;

	buffer = queue->buffers[b->index];

	ret = __virtio_media_buf_prepare(vfh, b);
	if (ret)
		return ret;

	ret = virtio_media_send_buffer_ioctl(vfh, VIDIOC_PREPARE_BUF, b);
	if (ret)
		return ret;

	/* The buffer is now prepared; report it to userspace. */
	virtio_media_fill_buffer_prepared(buffer, b);

	return 0;
}

static int virtio_media_qbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct v4l2_fh *vfh = file_to_v4l2_fh(file);
	struct virtio_media_session *session = fh_to_session(vfh);
	struct virtio_media_queue_state *queue;
	struct virtio_media_buffer *buffer;
	u32 old_flags;
	int ret;

	if (b->type > VIRTIO_MEDIA_LAST_QUEUE)
		return -EINVAL;
	queue = &session->queues[b->type];
	if (b->index >= queue->allocated_bufs)
		return -EINVAL;

	/*
	 * The buffer memory type must match the one the queue was set up with
	 * at REQBUFS/CREATE_BUFS time. This also rejects DMABUF/USERPTR
	 * buffers whenever their memory model has not been enabled, since the
	 * queue could only have been set to such a memory type when the
	 * corresponding module parameter allowed it.
	 */
	if (b->memory != queue->memory)
		return -EINVAL;

	buffer = queue->buffers[b->index];

	/*
	 * Prepare the buffer if it was not already PREPARE_BUF'd. This records
	 * the `m` union and binds any dma-buf planes; if it was prepared the
	 * bindings are reused. On failure the buffer is not queued.
	 */
	ret = __virtio_media_buf_prepare(vfh, b);
	if (ret)
		return ret;

	old_flags = buffer->buffer.flags;
	buffer->buffer.flags = V4L2_BUF_FLAG_QUEUED;

	/*
	 * The buffer memory is a host buffer mapped into the guest
	 * via backend entries and is cacheable. Clean the CPU caches so the data
	 * the guest wrote is visible to the non-coherent host DMA master.
	 */
	virtio_media_sync_buffer(session, b->type, b->index, DMA_TO_DEVICE);

	scoped_guard(mutex, &session->queues_lock)
		queue->queued_bufs += 1;
	ret = virtio_media_send_buffer_ioctl(vfh, VIDIOC_QBUF, b);
	if (ret) {
		/* Rollback the previous flags as the buffer is not queued. */
		buffer->buffer.flags = old_flags;
		scoped_guard(mutex, &session->queues_lock)
			queue->queued_bufs -= 1;
		/*
		 * The QBUF was rejected, so the buffer never entered the
		 * device's queue: return it to the unprepared state (as
		 * videobuf2 does when __buf_prepare/qbuf fails). This matters
		 * for V4L2_MEMORY_DMABUF, where the index is only a slot and a
		 * retry may carry a different dma-buf fd; re-running prepare
		 * next time rebinds the planes instead of reusing a stale
		 * binding.
		 */
		buffer->prepared = false;
		return ret;
	}

	/*
	 * Diagnostic counter only. Written solely from this ioctl context
	 * (serialised by vv->vlock), so no queues_lock is needed; the debugfs
	 * reader accesses it locklessly and tolerates torn reads.
	 */
	queue->qbuf_count += 1;

	/* A queued buffer is no longer reported as prepared. */
	virtio_media_fill_buffer_prepared(buffer, b);

	return 0;
}

static int virtio_media_dqbuf(struct file *file, void *fh,
			      struct v4l2_buffer *b)
{
	struct video_device *video_dev = video_devdata(file);
	struct virtio_media *vv = to_virtio_media(video_dev);
	struct virtio_media_session *session =
		fh_to_session(file_to_v4l2_fh(file));
	struct virtio_media_buffer *dqbuf = NULL;
	struct virtio_media_queue_state *queue;
	struct list_head *buffer_queue;
	struct v4l2_plane *planes_backup = NULL;
	const bool is_multiplanar = V4L2_TYPE_IS_MULTIPLANAR(b->type);
	int ret;

	if (b->type > VIRTIO_MEDIA_LAST_QUEUE)
		return -EINVAL;

	queue = &session->queues[b->type];

	/*
	 * If a buffer with the LAST flag has been returned, subsequent
	 * calls to DQBUF must return -EPIPE until the queue is cleared.
	 */
	if (queue->is_capture_last)
		return -EPIPE;

	buffer_queue = &queue->pending_dqbufs;

	if (queue->allocated_bufs == 0)
		return -EINVAL;
	else if (!queue->streaming)
		return -EINVAL;

	if (!session->nonblocking_dequeue) {
		/*
		 * vv->lock has been acquired by virtio_media_device_ioctl. Release it
		 * while we want to other ioctls for this session can be processed and
		 * potentially trigger dqbuf_wait.
		 */
		mutex_unlock(&vv->vlock);
		ret = wait_event_interruptible(session->dqbuf_wait,
					       !list_empty(buffer_queue));
		mutex_lock(&vv->vlock);
		if (ret)
			return -EINTR;
	}

	scoped_guard(mutex, &session->queues_lock) {
		if (!list_empty(buffer_queue)) {
			dqbuf = list_first_entry(buffer_queue,
						 struct virtio_media_buffer,
						 list);
			list_del(&dqbuf->list);
		}
	}
	if (!dqbuf)
		return -EAGAIN;

	/*
	 * The host DMA master has written the buffer. Invalidate the CPU
	 * caches so the guest reads the fresh data from DRAM rather than
	 * stale cache lines.
	 */
	virtio_media_sync_buffer(session, b->type, dqbuf->buffer.index,
				 DMA_FROM_DEVICE);

	/* Clear the DONE flag as the buffer is now being dequeued. */
	dqbuf->buffer.flags &= ~V4L2_BUF_FLAG_DONE;

	/*
	 * The buffer leaves the driver's hands: it must be prepared again
	 * before it can be requeued (mirrors videobuf2 clearing vb->prepared
	 * on dequeue).
	 */
	dqbuf->prepared = false;

	if (is_multiplanar) {
		size_t nb_planes = min_t(u32, b->length, VIDEO_MAX_PLANES);

		memcpy(b->m.planes, dqbuf->planes,
		       nb_planes * sizeof(struct v4l2_plane));
		planes_backup = b->m.planes;
	}

	memcpy(b, &dqbuf->buffer, sizeof(*b));

	if (is_multiplanar)
		b->m.planes = planes_backup;

	/* A dequeued buffer is no longer prepared. */
	virtio_media_fill_buffer_prepared(dqbuf, b);

	if (V4L2_TYPE_IS_CAPTURE(b->type) && b->flags & V4L2_BUF_FLAG_LAST) {
		queue->is_capture_last = true;
		virtio_media_record_flow(session, VIRTIO_MEDIA_FLOW_EOS, b->type,
					 b->index);
	}

	return 0;
}

/*
 * s/g_input/output work with an unsigned int - recast this to a u32 so the
 * size is unambiguous.
 */

static int virtio_media_g_input(struct file *file, void *fh, unsigned int *i)
{
	u32 input;

	int ret = virtio_media_send_wr_ioctl(file_to_v4l2_fh(file),
					 VIDIOC_G_INPUT, &input,
					 sizeof(input), sizeof(input));
	if (ret)
		return ret;

	*i = input;

	return 0;
}

static int virtio_media_s_input(struct file *file, void *fh, unsigned int i)
{
	u32 input = i;

	return virtio_media_send_wr_ioctl(file_to_v4l2_fh(file),
					  VIDIOC_S_INPUT, &input,
					  sizeof(input), sizeof(input));
}

static int virtio_media_g_output(struct file *file, void *fh, unsigned int *o)
{
	u32 output;

	int ret = virtio_media_send_wr_ioctl(file_to_v4l2_fh(file),
					 VIDIOC_G_OUTPUT, &output,
					 sizeof(output), sizeof(output));
	if (ret)
		return ret;

	*o = output;

	return 0;
}

static int virtio_media_s_output(struct file *file, void *fh, unsigned int o)
{
	u32 output = o;

	return virtio_media_send_wr_ioctl(file_to_v4l2_fh(file),
					  VIDIOC_S_OUTPUT, &output,
					  sizeof(output), sizeof(output));
}

/*
 * decoder_cmd can affect the state of the CAPTURE queue.
 */

static int virtio_media_decoder_cmd(struct file *file, void *fh,
				    struct v4l2_decoder_cmd *cmd)
{
	struct v4l2_fh *vfh = file_to_v4l2_fh(file);
	struct virtio_media_session *session = fh_to_session(vfh);

	int ret = virtio_media_send_wr_ioctl(vfh, VIDIOC_DECODER_CMD, cmd,
					 sizeof(*cmd), sizeof(*cmd));
	if (ret)
		return ret;

	virtio_media_record_flow(session, VIRTIO_MEDIA_FLOW_DECODER_CMD,
				 cmd->cmd, 0);

	/* A START command makes the CAPTURE queue able to dequeue again. */
	if (cmd->cmd == V4L2_DEC_CMD_START) {
		session->queues[V4L2_BUF_TYPE_VIDEO_CAPTURE].is_capture_last =
			false;
		session->queues[V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE]
			.is_capture_last = false;
	}

	return 0;
}

/*
 * s_std doesn't work with a pointer, so we cannot use SIMPLE_W_IOCTL.
 */

static int virtio_media_s_std(struct file *file, void *fh, v4l2_std_id s)
{
	int ret = virtio_media_send_w_ioctl(file_to_v4l2_fh(file), VIDIOC_S_STD,
					&s, sizeof(s));
	if (ret)
		return ret;

	return 0;
}

const struct v4l2_ioctl_ops virtio_media_ioctl_ops = {
	/* VIDIOC_QUERYCAP handler */
	.vidioc_querycap = virtio_media_querycap,

	/* VIDIOC_ENUM_FMT handlers */
	.vidioc_enum_fmt_vid_cap = virtio_media_enum_fmt,
	.vidioc_enum_fmt_vid_overlay = virtio_media_enum_fmt,
	.vidioc_enum_fmt_vid_out = virtio_media_enum_fmt,
	.vidioc_enum_fmt_sdr_cap = virtio_media_enum_fmt,
	.vidioc_enum_fmt_sdr_out = virtio_media_enum_fmt,
	.vidioc_enum_fmt_meta_cap = virtio_media_enum_fmt,
	.vidioc_enum_fmt_meta_out = virtio_media_enum_fmt,

	/* VIDIOC_G_FMT handlers */
	.vidioc_g_fmt_vid_cap = virtio_media_g_fmt,
	.vidioc_g_fmt_vid_overlay = virtio_media_g_fmt,
	.vidioc_g_fmt_vid_out = virtio_media_g_fmt,
	.vidioc_g_fmt_vid_out_overlay = virtio_media_g_fmt,
	.vidioc_g_fmt_vbi_cap = virtio_media_g_fmt,
	.vidioc_g_fmt_vbi_out = virtio_media_g_fmt,
	.vidioc_g_fmt_sliced_vbi_cap = virtio_media_g_fmt,
	.vidioc_g_fmt_sliced_vbi_out = virtio_media_g_fmt,
	.vidioc_g_fmt_vid_cap_mplane = virtio_media_g_fmt,
	.vidioc_g_fmt_vid_out_mplane = virtio_media_g_fmt,
	.vidioc_g_fmt_sdr_cap = virtio_media_g_fmt,
	.vidioc_g_fmt_sdr_out = virtio_media_g_fmt,
	.vidioc_g_fmt_meta_cap = virtio_media_g_fmt,
	.vidioc_g_fmt_meta_out = virtio_media_g_fmt,

	/* VIDIOC_S_FMT handlers */
	.vidioc_s_fmt_vid_cap = virtio_media_s_fmt,
	.vidioc_s_fmt_vid_overlay = virtio_media_s_fmt,
	.vidioc_s_fmt_vid_out = virtio_media_s_fmt,
	.vidioc_s_fmt_vid_out_overlay = virtio_media_s_fmt,
	.vidioc_s_fmt_vbi_cap = virtio_media_s_fmt,
	.vidioc_s_fmt_vbi_out = virtio_media_s_fmt,
	.vidioc_s_fmt_sliced_vbi_cap = virtio_media_s_fmt,
	.vidioc_s_fmt_sliced_vbi_out = virtio_media_s_fmt,
	.vidioc_s_fmt_vid_cap_mplane = virtio_media_s_fmt,
	.vidioc_s_fmt_vid_out_mplane = virtio_media_s_fmt,
	.vidioc_s_fmt_sdr_cap = virtio_media_s_fmt,
	.vidioc_s_fmt_sdr_out = virtio_media_s_fmt,
	.vidioc_s_fmt_meta_cap = virtio_media_s_fmt,
	.vidioc_s_fmt_meta_out = virtio_media_s_fmt,

	/* VIDIOC_TRY_FMT handlers */
	.vidioc_try_fmt_vid_cap = virtio_media_try_fmt,
	.vidioc_try_fmt_vid_overlay = virtio_media_try_fmt,
	.vidioc_try_fmt_vid_out = virtio_media_try_fmt,
	.vidioc_try_fmt_vid_out_overlay = virtio_media_try_fmt,
	.vidioc_try_fmt_vbi_cap = virtio_media_try_fmt,
	.vidioc_try_fmt_vbi_out = virtio_media_try_fmt,
	.vidioc_try_fmt_sliced_vbi_cap = virtio_media_try_fmt,
	.vidioc_try_fmt_sliced_vbi_out = virtio_media_try_fmt,
	.vidioc_try_fmt_vid_cap_mplane = virtio_media_try_fmt,
	.vidioc_try_fmt_vid_out_mplane = virtio_media_try_fmt,
	.vidioc_try_fmt_sdr_cap = virtio_media_try_fmt,
	.vidioc_try_fmt_sdr_out = virtio_media_try_fmt,
	.vidioc_try_fmt_meta_cap = virtio_media_try_fmt,
	.vidioc_try_fmt_meta_out = virtio_media_try_fmt,

	/* Buffer handlers */
	.vidioc_reqbufs = virtio_media_reqbufs,
	.vidioc_querybuf = virtio_media_querybuf,
	.vidioc_qbuf = virtio_media_qbuf,
	.vidioc_expbuf = virtio_media_expbuf,
	.vidioc_dqbuf = virtio_media_dqbuf,
	.vidioc_create_bufs = virtio_media_create_bufs,
	.vidioc_prepare_buf = virtio_media_prepare_buf,
	/* Overlay interface not supported yet */
	.vidioc_overlay = NULL,
	/* Overlay interface not supported yet */
	.vidioc_g_fbuf = NULL,
	/* Overlay interface not supported yet */
	.vidioc_s_fbuf = NULL,

	/* Stream on/off */
	.vidioc_streamon = virtio_media_streamon,
	.vidioc_streamoff = virtio_media_streamoff,

	/* Standard handling */
	.vidioc_g_std = virtio_media_g_std,
	.vidioc_s_std = virtio_media_s_std,
	.vidioc_querystd = virtio_media_querystd,

	/* Input handling */
	.vidioc_enum_input = virtio_media_enuminput,
	.vidioc_g_input = virtio_media_g_input,
	.vidioc_s_input = virtio_media_s_input,

	/* Output handling */
	.vidioc_enum_output = virtio_media_enumoutput,
	.vidioc_g_output = virtio_media_g_output,
	.vidioc_s_output = virtio_media_s_output,

	/* Control handling */
	.vidioc_query_ext_ctrl = virtio_media_query_ext_ctrl,
	.vidioc_g_ext_ctrls = virtio_media_g_ext_ctrls,
	.vidioc_s_ext_ctrls = virtio_media_s_ext_ctrls,
	.vidioc_try_ext_ctrls = virtio_media_try_ext_ctrls,
	.vidioc_querymenu = virtio_media_querymenu,

	/* Audio ioctls */
	.vidioc_enumaudio = virtio_media_enumaudio,
	.vidioc_g_audio = virtio_media_g_audio,
	.vidioc_s_audio = virtio_media_s_audio,

	/* Audio out ioctls */
	.vidioc_enumaudout = virtio_media_enumaudout,
	.vidioc_g_audout = virtio_media_g_audout,
	.vidioc_s_audout = virtio_media_s_audout,
	.vidioc_g_modulator = virtio_media_g_modulator,
	.vidioc_s_modulator = virtio_media_s_modulator,

	/* Crop ioctls */
	/*
	 * Not directly an ioctl (part of VIDIOC_CROPCAP), so no need to
	 * implement.
	 */
	.vidioc_g_pixelaspect = NULL,
	.vidioc_g_selection = virtio_media_g_selection,
	.vidioc_s_selection = virtio_media_s_selection,

	/* Compression ioctls */
	/* Deprecated in V4L2. */
	.vidioc_g_jpegcomp = NULL,
	/* Deprecated in V4L2. */
	.vidioc_s_jpegcomp = NULL,
	.vidioc_g_enc_index = virtio_media_g_enc_index,
	.vidioc_encoder_cmd = virtio_media_encoder_cmd,
	.vidioc_try_encoder_cmd = virtio_media_try_encoder_cmd,
	.vidioc_decoder_cmd = virtio_media_decoder_cmd,
	.vidioc_try_decoder_cmd = virtio_media_try_decoder_cmd,

	/* Stream type-dependent parameter ioctls */
	.vidioc_g_parm = virtio_media_g_parm,
	.vidioc_s_parm = virtio_media_s_parm,

	/* Tuner ioctls */
	.vidioc_g_tuner = virtio_media_g_tuner,
	.vidioc_s_tuner = virtio_media_s_tuner,
	.vidioc_g_frequency = virtio_media_g_frequency,
	.vidioc_s_frequency = virtio_media_s_frequency,
	.vidioc_enum_freq_bands = virtio_media_enum_freq_bands,

	/* Sliced VBI cap */
	.vidioc_g_sliced_vbi_cap = virtio_media_g_sliced_vbi_cap,

	/* Log status ioctl */
	/* Guest-only operation */
	.vidioc_log_status = NULL,

	.vidioc_s_hw_freq_seek = virtio_media_s_hw_freq_seek,

	.vidioc_enum_framesizes = virtio_media_enum_framesizes,
	.vidioc_enum_frameintervals = virtio_media_enum_frameintervals,

	/* DV Timings IOCTLs */
	.vidioc_s_dv_timings = virtio_media_s_dv_timings,
	.vidioc_g_dv_timings = virtio_media_g_dv_timings,
	.vidioc_query_dv_timings = virtio_media_query_dv_timings,
	.vidioc_enum_dv_timings = virtio_media_enum_dv_timings,
	.vidioc_dv_timings_cap = virtio_media_dv_timings_cap,
	.vidioc_g_edid = NULL,
	.vidioc_s_edid = NULL,

	.vidioc_subscribe_event = virtio_media_subscribe_event,
	.vidioc_unsubscribe_event = virtio_media_unsubscribe_event,

	/* For other private ioctls */
	.vidioc_default = NULL,
};

long virtio_media_device_ioctl(struct file *file, unsigned int cmd,
			       unsigned long arg)
{
	struct video_device *video_dev = video_devdata(file);
	struct virtio_media *vv = to_virtio_media(video_dev);
	struct v4l2_fh *vfh = NULL;
	struct v4l2_standard standard;
	v4l2_std_id std_id = 0;
	int ret;

	if (test_bit(V4L2_FL_USES_V4L2_FH, &video_dev->flags))
		vfh = file_to_v4l2_fh(file);

	mutex_lock(&vv->vlock);

	/*
	 * We need to handle a few ioctls manually because their results
	 * rely on vfd->tvnorms, which is normally updated by the driver
	 * as S_INPUT is called. Since we want to just pass these ioctls
	 * through, we have to hijack them from here.
	 */
	switch (cmd) {
	case VIDIOC_S_STD:
		ret = copy_from_user(&std_id, (void __user *)arg,
				     sizeof(std_id));
		if (ret) {
			ret = -EINVAL;
			break;
		}
		ret = virtio_media_s_std(file, vfh, std_id);
		break;
	case VIDIOC_ENUMSTD:
		ret = copy_from_user(&standard, (void __user *)arg,
				     sizeof(standard));
		if (ret) {
			ret = -EINVAL;
			break;
		}
		ret = virtio_media_enumstd(file, vfh, &standard);
		if (ret)
			break;
		ret = copy_to_user((void __user *)arg, &standard,
				   sizeof(standard));
		if (ret)
			ret = -EINVAL;
		break;
	case VIDIOC_QUERYSTD:
		ret = virtio_media_querystd(file, vfh, &std_id);
		if (ret)
			break;
		ret = copy_to_user((void __user *)arg, &std_id, sizeof(std_id));
		if (ret)
			ret = -EINVAL;
		break;
	default:
		ret = video_ioctl2(file, cmd, arg);
		break;
	}

	mutex_unlock(&vv->vlock);

	return ret;
}
