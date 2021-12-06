/*
 * Copyright 2019 NXP
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mxcfb.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/file.h>
#include <linux/sync_file.h>

#if IS_ENABLED(CONFIG_FB_FENCE)

static const struct dma_fence_ops fb_fence_ops;

static struct fb_fence_context *fence_to_context(struct dma_fence *fence)
{
	BUG_ON(fence->ops != &fb_fence_ops);
	return container_of(fence->lock, struct fb_fence_context, fence_lock);
}

static const char *fb_fence_get_driver_name(struct dma_fence *fence)
{
	struct fb_fence_context *ctx = fence_to_context(fence);

	return ctx->driver_name;
}

static const char *fb_fence_get_timeline_name(struct dma_fence *fence)
{
	struct fb_fence_context *ctx = fence_to_context(fence);

	return ctx->timeline_name;
}

static bool fb_fence_enable_signaling(struct dma_fence *fence)
{
	return true;
}

static const struct dma_fence_ops fb_fence_ops = {
	.get_driver_name = fb_fence_get_driver_name,
	.get_timeline_name = fb_fence_get_timeline_name,
	.enable_signaling = fb_fence_enable_signaling,
	.wait = dma_fence_default_wait,
};

int fb_init_fence_context(struct fb_fence_context *context, const char *driver, struct fb_info *fbi,
		int(*init)(int64_t, struct fb_var_screeninfo *, struct fb_info *))
{
	context->fb_info = fbi;
	context->fence_context = dma_fence_context_alloc(1);
	spin_lock_init(&context->fence_lock);
	INIT_LIST_HEAD(&context->present_queue);
	INIT_LIST_HEAD(&context->release_queue);
	context->on_screen = NULL;
	snprintf(context->timeline_name, sizeof(context->timeline_name),
		"%s", fbi->fix.id);
	snprintf(context->driver_name, sizeof(context->driver_name),
		"FB:%s", driver);
	context->update_screen = init;
	mutex_init(&context->mutex);

	return 0;
}
EXPORT_SYMBOL(fb_init_fence_context);
/**
 * fb_arm_release_fence - arm fence event after atomic commit.
 * @ctx: the source CRTC of the fence event
 * @e: the fence event to send
 *
 * It is android out fence event sent to fence queue.
 */
static void fb_arm_release_fence(struct fb_fence_context *ctx,
			struct fb_fence_event *e)
{
	unsigned long irqflags;

	spin_lock_irqsave(&ctx->fence_lock, irqflags);
	list_add_tail(&e->link, &ctx->release_queue);
	spin_unlock_irqrestore(&ctx->fence_lock, irqflags);
}

static void fb_arm_present_fence(struct fb_fence_context *ctx,
			struct fb_fence_event *e)
{
	unsigned long irqflags;

	spin_lock_irqsave(&ctx->fence_lock, irqflags);
	list_add_tail(&e->link, &ctx->present_queue);
	spin_unlock_irqrestore(&ctx->fence_lock, irqflags);
}

bool fb_handle_fence(struct fb_fence_context *ctx)
{
	unsigned long irqflags;
	struct fb_fence_event *e, *t;
	struct fb_fence_event *cur = NULL, *last = NULL;

	spin_lock_irqsave(&ctx->fence_lock, irqflags);

	// handle all present fence.
	list_for_each_entry_safe(e, t, &ctx->present_queue, link) {
		list_del(&e->link);
		dma_fence_signal_locked(&e->fence);
		dma_fence_put(&e->fence);
	}

	// handle release fence.
	last = ctx->on_screen;
	list_for_each_entry_safe(e, t, &ctx->release_queue, link) {
		list_del(&e->link);
		cur = e;
		break;
	}

	// no new buffer come.
	if (cur == NULL) {
		spin_unlock_irqrestore(&ctx->fence_lock, irqflags);
		return false;
	}

	// current buffer is on screen, signal last one.
	if (last != NULL) {
		dma_fence_signal_locked(&last->fence);
		dma_fence_put(&last->fence);
	}
	ctx->on_screen = cur;

	spin_unlock_irqrestore(&ctx->fence_lock, irqflags);

	return true;
}
EXPORT_SYMBOL(fb_handle_fence);

static void fb_commit_work(struct work_struct *work)
{
	struct fb_fence_commit *commit = container_of(work, struct fb_fence_commit, work);
	struct fb_fence_event *fence = commit->in_fence;
	struct fb_fence_context *context = commit->context;
	struct fb_info *fb_info = context->fb_info;

	if (fence != NULL) {
		dma_fence_wait(&fence->fence, true);
		dma_fence_put(&fence->fence);
	}

	if (commit->present_fence != NULL)
		fb_arm_present_fence(context, commit->present_fence);
	if (commit->release_fence != NULL)
		fb_arm_release_fence(context, commit->release_fence);

	mutex_lock(&context->mutex);
	context->update_screen(commit->smem_start, &commit->screeninfo, fb_info);
	mutex_unlock(&context->mutex);

	kfree(commit);
}

static struct fb_fence_commit* fb_get_commit(void)
{
	struct fb_fence_commit *commit = NULL;

	commit = kzalloc(sizeof(*commit), GFP_KERNEL);
	if (!commit)
		return NULL;

	INIT_WORK(&commit->work, fb_commit_work);

	return commit;
}

static int fb_setup_fence(struct dma_fence *fence, uint64_t val)
{
	int fd = -1;
	struct sync_file *sync_file = NULL;
	s32 __user *fence_ptr = u64_to_user_ptr(val);

	fd = get_unused_fd_flags(O_CLOEXEC);
	if (fd < 0) {
		return -EINVAL;
	}

	sync_file = sync_file_create(fence);
	if (!sync_file) {
		put_unused_fd(fd);
		return -ENOMEM;
	}

	if (fence_ptr) {
		if (put_user(fd, fence_ptr)) {
			put_unused_fd(fd);
			return -EFAULT;
		}
	}

	fd_install(fd, sync_file->file);
	return 0;
}

static struct fb_fence_event* fb_get_fence(struct fb_fence_context *ctx)
{
	struct fb_fence_event *event = NULL;

	event = kzalloc(sizeof(*event), GFP_KERNEL);
        if (!event) {
		return NULL;
	}

	dma_fence_init(&event->fence, &fb_fence_ops, &ctx->fence_lock,
		ctx->fence_context, ++ctx->fence_seqno);
	INIT_LIST_HEAD(&event->link);

	return event;
}

int fb_update_overlay(struct fb_fence_context *context, struct mxcfb_datainfo *buffer)
{
	int ret = 0;
	struct fb_fence_event *event = NULL;
	struct fb_fence_event *in_event = NULL;
	struct fb_fence_commit *commit = NULL;

	commit = fb_get_commit();
	if (!commit)
		return -ENOMEM;

	event = fb_get_fence(context);
	if (!event) {
		kfree(commit);
		return -ENOMEM;
	}

	ret = fb_setup_fence(&event->fence, buffer->fence_ptr);
	if (ret) {
		kfree(commit);
		dma_fence_put(&event->fence);
		return ret;
	}

	in_event = (struct fb_fence_event *)sync_file_get_fence(buffer->fence_fd);
	commit->context = context;
	commit->smem_start = buffer->smem_start;
	commit->in_fence = in_event;
	commit->release_fence = event;
	memcpy(&commit->screeninfo, &buffer->screeninfo, sizeof(commit->screeninfo));

	queue_work(system_unbound_wq, &commit->work);

	return 0;
}
EXPORT_SYMBOL(fb_update_overlay);

int fb_present_screen(struct fb_fence_context *context, struct mxcfb_datainfo *buffer)
{
	int ret = 0;
	struct fb_fence_event *event = NULL;
	struct fb_fence_event *in_event = NULL;
	struct fb_fence_commit *commit = NULL;

	commit = fb_get_commit();
	if (!commit)
		return -ENOMEM;

	event = fb_get_fence(context);
	if (!event) {
		kfree(commit);
		return -ENOMEM;
	}

	ret = fb_setup_fence(&event->fence, buffer->fence_ptr);
	if (ret) {
		kfree(commit);
		dma_fence_put(&event->fence);
		return ret;
	}

	in_event = (struct fb_fence_event *)sync_file_get_fence(buffer->fence_fd);
	commit->context = context;
	commit->smem_start = buffer->smem_start;
	commit->in_fence = in_event;
	commit->present_fence = event;
	memcpy(&commit->screeninfo, &buffer->screeninfo, sizeof(commit->screeninfo));

	queue_work(system_unbound_wq, &commit->work);

	return 0;
}
EXPORT_SYMBOL(fb_present_screen);
MODULE_LICENSE("GPL v2");

#endif

