/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM fs

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH trace/hooks

#if !defined(_TRACE_HOOK_FS_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_HOOK_FS_H

#include <trace/hooks/vendor_hooks.h>
struct f2fs_sb_info;
struct va_format;

DECLARE_HOOK(android_vh_ep_create_wakeup_source,
	TP_PROTO(char *name, int len),
	TP_ARGS(name, len));

DECLARE_HOOK(android_vh_timerfd_create,
	TP_PROTO(char *name, int len),
	TP_ARGS(name, len));

DECLARE_HOOK(android_vh_f2fs_printk,
	TP_PROTO(unsigned long s_flag, struct va_format *vaf, int level, bool limit_rate),
	TP_ARGS(s_flag, vaf, level, limit_rate));

DECLARE_HOOK(android_vh_f2fs_create,
	TP_PROTO(struct inode *inode, struct dentry *dentry),
	TP_ARGS(inode, dentry));

DECLARE_HOOK(android_vh_f2fs_set_bio_flag,
	TP_PROTO(struct folio *folio, struct bio *bio),
	TP_ARGS(folio, bio));

#endif /* _TRACE_HOOK_FS_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
