// SPDX-License-Identifier: GPL-2.0-only
/* binder_pick.c
 *
 * This file contains the logic for choosing between the C and Rust
 * implementations of the Android Binder driver.
 *
 * Copyright (C) 2026 Google LLC.
 */

#include <linux/kconfig.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/trace_events.h>

#include "binder_pick.h"

#ifdef MODULE_PARAM_PREFIX
#undef MODULE_PARAM_PREFIX
#endif
#define MODULE_PARAM_PREFIX "binder."

int binder_use_rust;
static int binder_loaded;

static DEFINE_MUTEX(binder_use_rust_lock);

int binder_try_unload_builtin(void)
{
	int ret = 0;

	mutex_lock(&binder_use_rust_lock);
	if (binder_loaded)
		ret = -EBUSY;
	else if (!binder_use_rust)
		ret = -EPERM;
	else
		binder_loaded = true;
	mutex_unlock(&binder_use_rust_lock);

	if (!ret)
		binder_unload_builtin();

	return ret;
}
EXPORT_SYMBOL_GPL(binder_try_unload_builtin);

/*
 * The driver that is built-in should call this when binderfs is mounted.
 */
int on_binderfs_mount(void)
{
	int ret = 0;

	mutex_lock(&binder_use_rust_lock);

	if (binder_loaded && binder_use_rust) {
		ret = -EINVAL;
		goto out;
	}

	/*
	 * C binder was mounted before loading the Rust Binder module. In this
	 * case, we fall back to using C Binder even if Rust Binder was
	 * requested.
	 */
	if (binder_use_rust)
		pr_warn("Using C Binder even though binder.impl=rust is set.\n");

	binder_use_rust = false;
	binder_loaded = true;

out:
	mutex_unlock(&binder_use_rust_lock);
	return ret;
}


static int binder_impl_param_set(const char *buffer, const struct kernel_param *kp)
{
	int ret = 0;

	mutex_lock(&binder_use_rust_lock);

	if (binder_loaded) {
		ret = -EPERM;
		goto out;
	}

	if (!strcmp(buffer, "rust"))
		binder_use_rust = true;
	else if (!strcmp(buffer, "c"))
		binder_use_rust = false;
	else
		ret = -EINVAL;

out:
	mutex_unlock(&binder_use_rust_lock);
	return ret;
}

static int binder_impl_param_get(char *buffer, const struct kernel_param *kp)
{
	/* The buffer is 4k bytes, so this will not overflow. */
	return sprintf(buffer, "%s\n", binder_use_rust ? "rust" : "c");
}

static const struct kernel_param_ops binder_param_ops = {
	.set = binder_impl_param_set,
	.get = binder_impl_param_get,
};

module_param_cb(impl, &binder_param_ops, NULL, 0644);

#include "../../kernel/trace/trace_output.h"
#include "../../kernel/trace/trace.h"

void binder_remove_trace_events(struct module *module)
{
	struct trace_event_call *call, *tmp;
	const char *name;

	mutex_lock(&event_mutex);
	mutex_lock(&trace_types_lock);
	down_write(&trace_event_sem);
	list_for_each_entry_safe(call, tmp, &ftrace_events, list) {
		name = trace_event_name(call);
		if (!name || strncmp(name, "binder_", 7))
			continue;
		if (call->module != module)
			continue;
		remove_event_from_tracers(call);
		list_del_init(&call->list);
	}
	up_write(&trace_event_sem);
	mutex_unlock(&trace_types_lock);
	mutex_unlock(&event_mutex);
}
EXPORT_SYMBOL_GPL(binder_remove_trace_events);
