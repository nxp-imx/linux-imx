// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 Samsung Electronics Co., Ltd.
 *            http://www.samsung.com/
 */

#include <linux/eventfd.h>
#include <linux/file.h>
#include <linux/syscalls.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/module.h>
#include <linux/slab.h>

#include "hvm_drv.h"

/**
 * ioeventfd_check_collision() - Check collison assumes hvm->ioevent_lock held.
 * @: Pointer to hvm.
 * @p: Pointer to hvm_ioevent.
 *
 * Return:
 * * true			- collison found
 * * false			- no collison
 */
static bool ioeventfd_check_collision(struct hvm *hvm, struct hvm_ioevent *p)
{
	struct hvm_ioevent *_p;

	lockdep_assert_held(&hvm->ioevent_lock);

	list_for_each_entry(_p, &hvm->ioevents, list) {
		if (_p->addr == p->addr &&
		    (!_p->len || !p->len ||
		     (_p->len == p->len &&
		      (_p->wildcard || p->wildcard ||
		       _p->datamatch == p->datamatch))))
			return true;
		if (p->addr >= _p->addr && p->addr < _p->addr + _p->len)
			return true;
	}

	return false;
}

static void hvm_ioevent_release(struct hvm *hvm, struct hvm_ioevent *p)
{
	lockdep_assert_held(&hvm->ioevent_lock);

	eventfd_ctx_put(p->evt_ctx);
	list_del(&p->list);
	kfree(p);
}

static bool hvm_ioevent_in_range(struct hvm_ioevent *p, u64 addr,
				 int len, const void *val)
{
	u64 _val;

	if (addr != p->addr)
		/* address must be precise for a hit */
		return false;

	if (!p->len)
		/* length = 0 means only look at the address, so always a hit */
		return true;

	if (len != p->len)
		/* address-range must be precise for a hit */
		return false;

	if (p->wildcard)
		/* all else equal, wildcard is always a hit */
		return true;

	/* otherwise, we have to actually compare the data */

	switch (len) {
	case 1:
		_val = *(u8 *)val;
		break;
	case 2:
		_val = *(u16 *)val;
		break;
	case 4:
		_val = *(u32 *)val;
		break;
	case 8:
		_val = *(u64 *)val;
		break;
	default:
		return false;
	}

	return _val == p->datamatch;
}

static int hvm_deassign_ioeventfd(struct hvm *hvm,
			       struct hvm_ioeventfd *args)
{
	struct hvm_ioevent *p, *tmp;
	struct eventfd_ctx *evt_ctx;
	int ret = -ENOENT;
	bool wildcard;

	evt_ctx = eventfd_ctx_fdget(args->fd);
	if (IS_ERR(evt_ctx))
		return PTR_ERR(evt_ctx);

	wildcard = !(args->flags & HVM_IOEVENTFD_FLAG_DATAMATCH);

	mutex_lock(&hvm->ioevent_lock);
	list_for_each_entry_safe(p, tmp, &hvm->ioevents, list) {
		if (p->evt_ctx != evt_ctx  ||
		    p->addr != args->addr  ||
		    p->len != args->len ||
		    p->wildcard != wildcard)
			continue;

		if (!p->wildcard && p->datamatch != args->datamatch)
			continue;

		hvm_ioevent_release(hvm, p);
		ret = 0;
		break;
	}

	mutex_unlock(&hvm->ioevent_lock);

	/* got in the front of this function */
	eventfd_ctx_put(evt_ctx);

	return ret;
}

static int hvm_assign_ioeventfd(struct hvm *hvm,
			     struct hvm_ioeventfd *args)
{
	struct eventfd_ctx *evt_ctx;
	struct hvm_ioevent *evt;
	int ret;

	evt_ctx = eventfd_ctx_fdget(args->fd);
	if (IS_ERR(evt_ctx))
		return PTR_ERR(evt_ctx);

	evt = kmalloc(sizeof(*evt), GFP_KERNEL);
	if (!evt)
		return -ENOMEM;
	*evt = (struct hvm_ioevent) {
		.addr = args->addr,
		.len = args->len,
		.evt_ctx = evt_ctx,
	};
	if (args->flags & HVM_IOEVENTFD_FLAG_DATAMATCH) {
		evt->datamatch = args->datamatch;
		evt->wildcard = false;
	} else {
		evt->wildcard = true;
	}

	mutex_lock(&hvm->ioevent_lock);
	if (ioeventfd_check_collision(hvm, evt)) {
		ret = -EEXIST;
		mutex_unlock(&hvm->ioevent_lock);
		goto err_free;
	}

	list_add_tail(&evt->list, &hvm->ioevents);
	mutex_unlock(&hvm->ioevent_lock);

	return 0;

err_free:
	kfree(evt);
	eventfd_ctx_put(evt_ctx);
	return ret;
}

/**
 * hvm_ioeventfd_check_valid() - Check user arguments is valid.
 * @args: Pointer to hvm_ioeventfd.
 *
 * Return:
 * * true if user arguments are valid.
 * * false if user arguments are invalid.
 */
static bool hvm_ioeventfd_check_valid(struct hvm_ioeventfd *args)
{
	/* must be natural-word sized, or 0 to ignore length */
	switch (args->len) {
	case 0:
	case 1:
	case 2:
	case 4:
	case 8:
		break;
	default:
		return false;
	}

	/* check for range overflow */
	if (args->addr + args->len < args->addr)
		return false;

	/* check for extra flags that we don't understand */
	if (args->flags & ~HVM_IOEVENTFD_VALID_FLAG_MASK)
		return false;

	/* ioeventfd with no length can't be combined with DATAMATCH */
	if (!args->len && (args->flags & HVM_IOEVENTFD_FLAG_DATAMATCH))
		return false;

	/*  does not support pio bus ioeventfd */
	if (args->flags & HVM_IOEVENTFD_FLAG_PIO)
		return false;

	return true;
}

/**
 * hvm_ioeventfd() - Register ioevent to ioevent list.
 * @: Pointer to hvm.
 * @args: Pointer to hvm_ioeventfd.
 *
 * Return:
 * * 0			- Success.
 * * Negative		- Failure.
 */
int hvm_ioeventfd(struct hvm *hvm, struct hvm_ioeventfd *args)
{
	if (hvm_ioeventfd_check_valid(args) == false)
		return -EINVAL;

	if (args->flags & HVM_IOEVENTFD_FLAG_DEASSIGN)
		return hvm_deassign_ioeventfd(hvm, args);

	return hvm_assign_ioeventfd(hvm, args);
}

/**
 * hvm_ioevent_write() - Travers this vm's registered ioeventfd to see if
 *			  need notifying it.
 * @vcpu: Pointer to vcpu.
 * @addr: mmio address.
 * @len: mmio size.
 * @val: Pointer to void.
 *
 * Return:
 * * true if this io is already sent to ioeventfd's listener.
 * * false if we cannot find any ioeventfd registering this mmio write.
 */
bool hvm_ioevent_write(struct hvm_vcpu *vcpu, u64 addr, int len,
			const void *val)
{
	struct hvm_ioevent *e;

	mutex_lock(&vcpu->hvm->ioevent_lock);
	list_for_each_entry(e, &vcpu->hvm->ioevents, list) {
		if (hvm_ioevent_in_range(e, addr, len, val)) {
			eventfd_signal(e->evt_ctx);
			mutex_unlock(&vcpu->hvm->ioevent_lock);
			return true;
		}
	}

	mutex_unlock(&vcpu->hvm->ioevent_lock);
	return false;
}

int hvm_init_ioeventfd(struct hvm *hvm)
{
	INIT_LIST_HEAD(&hvm->ioevents);
	mutex_init(&hvm->ioevent_lock);

	return 0;
}
