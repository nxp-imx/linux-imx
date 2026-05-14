// SPDX-License-Identifier: GPL-2.0-only
/* rust_binder_hooks.c
 *
 * Rust Binder vendorhooks.
 *
 * Copyright 2026 Google LLC
 */

#include "rust_binder.h"

#define CREATE_TRACE_POINTS
#define CREATE_RUST_TRACE_POINTS
#include <trace/hooks/vendor_hooks.h>
#include <linux/tracepoint.h>

#include <trace/hooks/rust_binder.h>

/*
 * Used by vendor hooks to access Rust Binder data structures.
 */
struct rust_binder_layout RUST_BINDER_LAYOUT = {};
EXPORT_SYMBOL_GPL(RUST_BINDER_LAYOUT);

/*
 * Export tracepoints that act as a bare tracehook (ie: have no trace event
 * associated with them) to allow external modules to probe them.
 */
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_rust_binder_set_priority);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_rust_binder_restore_priority);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_rust_binder_looper_entry);

/*
 * Rust Binder itself calls these methods to trigger the vendor hook.
 */
EXPORT_SYMBOL_GPL(rust_do_trace_android_vh_rust_binder_set_priority);
EXPORT_SYMBOL_GPL(rust_do_trace_android_vh_rust_binder_restore_priority);
EXPORT_SYMBOL_GPL(rust_do_trace_android_vh_rust_binder_looper_entry);
