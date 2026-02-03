/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _LINUX_BINDER_PICK_IMPL_H
#define _LINUX_BINDER_PICK_IMPL_H

/*
 * Defined by binder_pick.c.
 */
extern int binder_use_rust;
void binder_remove_trace_events(struct module *module);
int binder_try_unload_builtin(void);
int on_binderfs_mount(void);

/*
 * Defined by whichever driver is built-in.
 */
void binder_unload_builtin(void);

#endif /* _LINUX_BINDER_PICK_IMPL_H */
