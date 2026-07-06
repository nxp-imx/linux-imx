/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM blk

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH trace/hooks

#if !defined(_TRACE_HOOK_BLK_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_HOOK_BLK_H

#include <trace/hooks/vendor_hooks.h>

struct block_device;
struct path;
struct bio;
struct gendisk;
struct request;

DECLARE_HOOK(android_vh_check_set_ioprio,
	TP_PROTO(struct bio *bio),
	TP_ARGS(bio));

struct path;
struct vfsmount;

DECLARE_HOOK(android_vh_do_new_mount_fc,
	TP_PROTO(const struct path *mountpoint, struct vfsmount *mnt),
	TP_ARGS(mountpoint, mnt));

DECLARE_HOOK(android_vh_bd_link_disk_holder,
	TP_PROTO(struct block_device *bdev, struct gendisk *disk),
	TP_ARGS(bdev, disk));

struct blk_mq_hw_ctx;
struct request_queue;

DECLARE_HOOK(android_vh_blk_mq_delay_run_hw_queue,
	TP_PROTO(int cpu, struct blk_mq_hw_ctx *hctx, unsigned long delay, bool *skip),
	TP_ARGS(cpu, hctx, delay, skip));

DECLARE_HOOK(android_vh_blk_mq_kick_requeue_list,
	TP_PROTO(struct request_queue *q, unsigned long delay, bool *skip),
	TP_ARGS(q, delay, skip));

struct readahead_control;

DECLARE_HOOK(android_vh_f2fs_ra_op_flags,
	TP_PROTO(blk_opf_t *op_flag, struct readahead_control *rac),
	TP_ARGS(op_flag, rac));

DECLARE_RESTRICTED_HOOK(android_rvh_submit_bio_pre,
	TP_PROTO(struct bio *bio, struct request *rq),
	TP_ARGS(bio, rq), 1);

DECLARE_RESTRICTED_HOOK(android_rvh_submit_bio_post,
	TP_PROTO(struct bio *bio, struct request *rq),
	TP_ARGS(bio, rq), 1);

DECLARE_HOOK(android_vh_request_issue_err,
	TP_PROTO(struct request *rq),
	TP_ARGS(rq));

DECLARE_RESTRICTED_HOOK(android_rvh_rq_qos_wait,
	TP_PROTO(void *ignored),
	TP_ARGS(NULL), 1);

DECLARE_HOOK(android_vh_wbt_wait,
	TP_PROTO(struct bio *bio, int flags),
	TP_ARGS(bio, flags));

DECLARE_HOOK(android_vh_wbt_track,
	TP_PROTO(struct request *rq, struct bio *bio),
	TP_ARGS(rq, bio));

DECLARE_RESTRICTED_HOOK(android_rvh_blk_execute_rq,
	TP_PROTO(struct request *rq),
	TP_ARGS(rq), 1);

DECLARE_RESTRICTED_HOOK(android_rvh_blk_mq_get_tag,
	TP_PROTO(void *ignored),
	TP_ARGS(NULL), 1);

DECLARE_RESTRICTED_HOOK(android_rvh_io_schedule_prepare,
	TP_PROTO(void *ignored),
	TP_ARGS(NULL), 1);
#endif /* _TRACE_HOOK_BLK_H */
/* This part must be outside protection */
#include <trace/define_trace.h>
