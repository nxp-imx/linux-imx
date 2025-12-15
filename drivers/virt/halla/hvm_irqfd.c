// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 Samsung Electronics Co., Ltd.
 *            http://www.samsung.com/
 */

#include <linux/eventfd.h>
#include <linux/syscalls.h>

#include "hvm_drv.h"

struct hvm_irq_ack_notifier {
	struct hlist_node link;
	u32 gsi;
	void (*irq_acked)(struct hvm_irq_ack_notifier *ian);
};

/**
 * struct hvm_kernel_irqfd: hvm kernel irqfd descriptor.
 * @hvm: Pointer to struct hvm.
 * @wait: Wait queue entry.
 * @gsi: Used for level IRQ fast-path.
 * @eventfd: Used for setup/shutdown.
 * @list: struct list_head.
 * @pt: struct poll_table_struct.
 * @shutdown: struct work_struct.
 */
struct hvm_kernel_irqfd {
	struct hvm *hvm;
	wait_queue_entry_t wait;

	int gsi;

	struct eventfd_ctx *eventfd;
	struct list_head list;
	poll_table pt;
	struct work_struct shutdown;
};

static struct workqueue_struct *irqfd_cleanup_wq;

/**
 * irqfd_set_irq(): irqfd to inject virtual interrupt.
 * @hvm: Pointer to hvm.
 * @irq: This is spi interrupt number (starts from 0 instead of 32).
 * @level: irq triggered level.
 */
static void irqfd_set_irq(struct hvm *hvm, u32 irq, int level)
{
	if (level)
		hvm_inject_irq(hvm, 0, irq, level);
}

/**
 * irqfd_shutdown() - Race-free decouple logic (ordering is critical).
 * @work: Pointer to work_struct.
 */
static void irqfd_shutdown(struct work_struct *work)
{
	struct hvm_kernel_irqfd *irqfd =
		container_of(work, struct hvm_kernel_irqfd, shutdown);
	struct hvm *hvm = irqfd->hvm;
	u64 cnt;

	/* Make sure irqfd has been initialized in assign path. */
	synchronize_srcu(&hvm->irq_srcu);

	/*
	 * Synchronize with the wait-queue and unhook ourselves to prevent
	 * further events.
	 */
	eventfd_ctx_remove_wait_queue(irqfd->eventfd, &irqfd->wait, &cnt);

	/*
	 * It is now safe to release the object's resources
	 */
	eventfd_ctx_put(irqfd->eventfd);
	kfree(irqfd);
}

/**
 * irqfd_is_active()
 * @irqfd: Pointer to hvm_kernel_irqfd.
 *
 * Return:
 * * true			- irqfd is active.
 */
static bool irqfd_is_active(struct hvm_kernel_irqfd *irqfd)
{
	struct hvm *hvm = irqfd->hvm;

	lockdep_assert_held(&hvm->irqfds.lock);

	return list_empty(&irqfd->list) ? false : true;
}

/**
 * irqfd_deactivate() - Mark the irqfd as inactive and schedule it for removal.
 *
 * @irqfd: Pointer to hvm_kernel_irqfd.
 */
static void irqfd_deactivate(struct hvm_kernel_irqfd *irqfd)
{
	struct hvm *hvm = irqfd->hvm;

	if (!irqfd_is_active(irqfd))
		return;

	lockdep_assert_held(&hvm->irqfds.lock);

	list_del_init(&irqfd->list);

	queue_work(irqfd_cleanup_wq, &irqfd->shutdown);
}

/**
 * irqfd_wakeup() - Callback of irqfd wait queue, would be woken by writing to
 *                  irqfd to do virtual interrupt injection.
 * @wait: Pointer to wait_queue_entry_t.
 * @mode: Unused.
 * @sync: Unused.
 * @key: Get flags about Epoll events.
 *
 * Return:
 * * 0			- Success
 */
static int irqfd_wakeup(wait_queue_entry_t *wait, u32 mode, int sync,
			void *key)
{
	struct hvm_kernel_irqfd *irqfd =
		container_of(wait, struct hvm_kernel_irqfd, wait);
	__poll_t flags = key_to_poll(key);
	struct hvm *hvm = irqfd->hvm;

	if (flags & EPOLLIN) {
		u64 cnt;

		eventfd_ctx_do_read(irqfd->eventfd, &cnt);
		/* hvm's irq injection is not blocked, don't need workq */
		irqfd_set_irq(hvm, irqfd->gsi, 1);
	}

	if (flags & EPOLLHUP) {
		/* The eventfd is closing, detach from HVM */
		unsigned long iflags;

		spin_lock_irqsave(&hvm->irqfds.lock, iflags);

		/*
		 * Do more check if someone deactivated the irqfd before
		 * we could acquire the irqfds.lock.
		 */
		if (irqfd_is_active(irqfd))
			irqfd_deactivate(irqfd);

		spin_unlock_irqrestore(&hvm->irqfds.lock, iflags);
	}

	return 0;
}

static void irqfd_ptable_queue_proc(struct file *file, wait_queue_head_t *wqh,
				    poll_table *pt)
{
	struct hvm_kernel_irqfd *irqfd =
		container_of(pt, struct hvm_kernel_irqfd, pt);
	add_wait_queue_priority(wqh, &irqfd->wait);
}

static int hvm_irqfd_assign(struct hvm *hvm, struct hvm_irqfd *args)
{
	struct hvm_kernel_irqfd *irqfd, *tmp;
	struct fd f;
	struct eventfd_ctx *eventfd = NULL;
	int ret;
	int idx;

	irqfd = kzalloc(sizeof(*irqfd), GFP_KERNEL_ACCOUNT);
	if (!irqfd)
		return -ENOMEM;

	irqfd->hvm = hvm;
	irqfd->gsi = args->gsi;

	INIT_LIST_HEAD(&irqfd->list);
	INIT_WORK(&irqfd->shutdown, irqfd_shutdown);

	f = fdget(args->fd);
	if (!fd_file(f)) {
		ret = -EBADF;
		goto out;
	}

	eventfd = eventfd_ctx_fileget(fd_file(f));
	if (IS_ERR(eventfd)) {
		ret = PTR_ERR(eventfd);
		goto fail;
	}

	irqfd->eventfd = eventfd;

	/*
	 * Install our own custom wake-up handling so we are notified via
	 * a callback whenever someone signals the underlying eventfd
	 */
	init_waitqueue_func_entry(&irqfd->wait, irqfd_wakeup);
	init_poll_funcptr(&irqfd->pt, irqfd_ptable_queue_proc);

	spin_lock_irq(&hvm->irqfds.lock);

	ret = 0;
	list_for_each_entry(tmp, &hvm->irqfds.items, list) {
		if (irqfd->eventfd != tmp->eventfd)
			continue;
		/* This fd is used for another irq already. */
		ret = -EBUSY;
		spin_unlock_irq(&hvm->irqfds.lock);
		goto fail;
	}

	idx = srcu_read_lock(&hvm->irq_srcu);

	list_add_tail(&irqfd->list, &hvm->irqfds.items);

	spin_unlock_irq(&hvm->irqfds.lock);

	vfs_poll(fd_file(f), &irqfd->pt);

	srcu_read_unlock(&hvm->irq_srcu, idx);

	/*
	 * do not drop the file until the irqfd is fully initialized, otherwise
	 * we might race against the EPOLLHUP
	 */
	fdput(f);
	return 0;

fail:
	if (eventfd && !IS_ERR(eventfd))
		eventfd_ctx_put(eventfd);

	fdput(f);

out:
	kfree(irqfd);
	return ret;
}

/**
 * hvm_irqfd_deassign() - Shutdown any irqfd's that match fd+gsi.
 * @hvm: Pointer to hvm.
 * @args: Pointer to hvm_irqfd.
 *
 * Return:
 * * 0			- Success.
 * * Negative value	- Failure.
 */
static int hvm_irqfd_deassign(struct hvm *hvm, struct hvm_irqfd *args)
{
	struct hvm_kernel_irqfd *irqfd, *tmp;
	struct eventfd_ctx *eventfd;

	eventfd = eventfd_ctx_fdget(args->fd);
	if (IS_ERR(eventfd))
		return PTR_ERR(eventfd);

	spin_lock_irq(&hvm->irqfds.lock);

	list_for_each_entry_safe(irqfd, tmp, &hvm->irqfds.items, list) {
		if (irqfd->eventfd == eventfd && irqfd->gsi == args->gsi)
			irqfd_deactivate(irqfd);
	}

	spin_unlock_irq(&hvm->irqfds.lock);
	eventfd_ctx_put(eventfd);

	/*
	 * Block until we know all outstanding shutdown jobs have completed
	 * so that we guarantee there will not be any more interrupts on this
	 * gsi once this deassign function returns.
	 */
	flush_workqueue(irqfd_cleanup_wq);

	return 0;
}

int hvm_irqfd(struct hvm *hvm, struct hvm_irqfd *args)
{
	for (int i = 0; i < ARRAY_SIZE(args->pad); i++) {
		if (args->pad[i])
			return -EINVAL;
	}

	if (args->flags &
	    ~(HVM_IRQFD_FLAG_DEASSIGN | HVM_IRQFD_FLAG_RESAMPLE))
		return -EINVAL;

	if (args->flags & HVM_IRQFD_FLAG_DEASSIGN)
		return hvm_irqfd_deassign(hvm, args);

	return hvm_irqfd_assign(hvm, args);
}

/**
 * hvm_vm_irqfd_init() - Initialize irqfd data structure per VM
 *
 * @hvm: Pointer to struct hvm.
 *
 * Return:
 * * 0			- Success.
 * * Negative		- Failure.
 */
int hvm_vm_irqfd_init(struct hvm *hvm)
{
	spin_lock_init(&hvm->irqfds.lock);
	INIT_LIST_HEAD(&hvm->irqfds.items);
	if (init_srcu_struct(&hvm->irq_srcu))
		return -EINVAL;

	return 0;
}

/**
 * hvm_vm_irqfd_release() - This function is called as the hvm VM fd is being
 *			  released. Shutdown all irqfds that still remain open.
 * @hvm: Pointer to hvm.
 */
void hvm_vm_irqfd_release(struct hvm *hvm)
{
	struct hvm_kernel_irqfd *irqfd, *tmp;

	spin_lock_irq(&hvm->irqfds.lock);

	list_for_each_entry_safe(irqfd, tmp, &hvm->irqfds.items, list)
		irqfd_deactivate(irqfd);

	spin_unlock_irq(&hvm->irqfds.lock);

	/*
	 * Block until we know all outstanding shutdown jobs have completed.
	 */
	flush_workqueue(irqfd_cleanup_wq);
}

/**
 * hvm_drv_irqfd_init() - Erase flushing work items when a VM exits.
 *
 * Return:
 * * 0			- Success.
 * * Negative		- Failure.
 *
 * Create a host-wide workqueue for issuing deferred shutdown requests
 * aggregated from all vm* instances. We need our own isolated
 * queue to ease flushing work items when a VM exits.
 */
int hvm_drv_irqfd_init(void)
{
	irqfd_cleanup_wq = alloc_workqueue("hvm-irqfd-cleanup", 0, 0);
	if (!irqfd_cleanup_wq)
		return -ENOMEM;

	return 0;
}

void hvm_drv_irqfd_exit(void)
{
	destroy_workqueue(irqfd_cleanup_wq);
}
