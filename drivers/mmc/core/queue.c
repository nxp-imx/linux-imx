/*
 *  Copyright (C) 2003 Russell King, All Rights Reserved.
 *  Copyright 2006-2007 Pierre Ossman
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/blkdev.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>

#include <linux/mmc/card.h>
#include <linux/mmc/host.h>

#include "queue.h"
#include "block.h"
#include "core.h"
#include "card.h"

#define MMC_QUEUE_BOUNCESZ	65536

/*
 * Prepare a MMC request. This just filters out odd stuff.
 */
static int mmc_prep_request(struct request_queue *q, struct request *req)
{
	struct mmc_queue *mq = q->queuedata;

	if (mq && (mmc_card_removed(mq->card) || mmc_access_rpmb(mq)))
		return BLKPREP_KILL;

	req->rq_flags |= RQF_DONTPREP;
	req_to_mmc_queue_req(req)->retries = 0;

	return BLKPREP_OK;
}

static void mmc_cqe_request_fn(struct request_queue *q)
{
	struct mmc_queue *mq = q->queuedata;
	struct request *req;

	if (!mq) {
		while ((req = blk_fetch_request(q)) != NULL) {
			req->rq_flags |= RQF_QUIET;
			__blk_end_request_all(req, -EIO);
		}
		return;
	}

	if (mq->asleep && !mq->cqe_busy)
		wake_up_process(mq->thread);
}

static inline bool mmc_cqe_dcmd_busy(struct mmc_queue *mq)
{
	/* Allow only 1 DCMD at a time */
	return mq->cqe_in_flight[MMC_ISSUE_DCMD];
}

void mmc_cqe_kick_queue(struct mmc_queue *mq)
{
	if ((mq->cqe_busy & MMC_CQE_DCMD_BUSY) && !mmc_cqe_dcmd_busy(mq))
		mq->cqe_busy &= ~MMC_CQE_DCMD_BUSY;

	mq->cqe_busy &= ~MMC_CQE_QUEUE_FULL;

	if (mq->asleep && !mq->cqe_busy)
		__blk_run_queue(mq->queue);
}

static inline bool mmc_cqe_can_dcmd(struct mmc_host *host)
{
	return host->caps2 & MMC_CAP2_CQE_DCMD;
}

enum mmc_issue_type mmc_cqe_issue_type(struct mmc_host *host,
				       struct request *req)
{
	if (req && ((req_op(req) == REQ_OP_DISCARD) || (req_op(req) == REQ_OP_SECURE_ERASE))) {
		return MMC_ISSUE_SYNC;
	} else if (req && req_op(req) == REQ_OP_FLUSH) {
		return mmc_cqe_can_dcmd(host) ? MMC_ISSUE_DCMD : MMC_ISSUE_SYNC;
	} else {
		return MMC_ISSUE_ASYNC;
	}
}

static void __mmc_cqe_recovery_notifier(struct mmc_queue *mq)
{
	if (!mq->cqe_recovery_needed) {
		mq->cqe_recovery_needed = true;
		wake_up_process(mq->thread);
	}
}

static void mmc_cqe_recovery_notifier(struct mmc_host *host,
				      struct mmc_request *mrq)
{
	struct mmc_queue_req *mqrq = container_of(mrq, struct mmc_queue_req,
						  brq.mrq);
	struct request *req = mqrq->req;
	struct request_queue *q = req->q;
	struct mmc_queue *mq = q->queuedata;
	unsigned long flags;

	spin_lock_irqsave(q->queue_lock, flags);
	__mmc_cqe_recovery_notifier(mq);
	spin_unlock_irqrestore(q->queue_lock, flags);
}

static int mmc_cqe_thread(void *d)
{
	struct mmc_queue *mq = d;
	struct request_queue *q = mq->queue;
	struct mmc_card *card = mq->card;
	struct mmc_host *host = card->host;
	unsigned long flags;
	int get_put = 0;

	current->flags |= PF_MEMALLOC;

	down(&mq->thread_sem);
	spin_lock_irqsave(q->queue_lock, flags);
	while (1) {
		struct request *req = NULL;
		enum mmc_issue_type issue_type;
		bool retune_ok = false;

		if (mq->cqe_recovery_needed) {
			spin_unlock_irqrestore(q->queue_lock, flags);
			mmc_blk_cqe_recovery(mq);
			spin_lock_irqsave(q->queue_lock, flags);
			mq->cqe_recovery_needed = false;
		}

		set_current_state(TASK_INTERRUPTIBLE);

		if (!kthread_should_stop())
			req = blk_peek_request(q);

		if (req) {
			issue_type = mmc_cqe_issue_type(host, req);
			switch (issue_type) {
			case MMC_ISSUE_DCMD:
				if (mmc_cqe_dcmd_busy(mq)) {
					mq->cqe_busy |= MMC_CQE_DCMD_BUSY;
					req = NULL;
					break;
				}
				/* Fall through */
			case MMC_ISSUE_ASYNC:
				if (blk_queue_start_tag(q, req)) {
					mq->cqe_busy |= MMC_CQE_QUEUE_FULL;
					req = NULL;
				}
				break;
			default:
				/*
				 * Timeouts are handled by mmc core, so set a
				 * large value to avoid races.
				 */
				req->timeout = 600 * HZ;
				blk_start_request(req);
				break;
			}
			if (req) {
				mq->cqe_in_flight[issue_type] += 1;
				if (mmc_cqe_tot_in_flight(mq) == 1)
					get_put += 1;
				if (mmc_cqe_qcnt(mq) == 1)
					retune_ok = true;
			}
		}

		mq->asleep = !req;

		spin_unlock_irqrestore(q->queue_lock, flags);

		if (req) {
			enum mmc_issued issued;

			set_current_state(TASK_RUNNING);

			if (get_put) {
				get_put = 0;
				mmc_get_card(card);
			}

			if (host->need_retune && retune_ok &&
			    !host->hold_retune)
				host->retune_now = true;
			else
				host->retune_now = false;

			issued = mmc_blk_cqe_issue_rq(mq, req);

			cond_resched();

			spin_lock_irqsave(q->queue_lock, flags);

			switch (issued) {
			case MMC_REQ_STARTED:
				break;
			case MMC_REQ_BUSY:
				blk_requeue_request(q, req);
				goto finished;
			case MMC_REQ_FAILED_TO_START:
				__blk_end_request_all(req, -EIO);
				/* Fall through */
			case MMC_REQ_FINISHED:
finished:
				mq->cqe_in_flight[issue_type] -= 1;
				if (mmc_cqe_tot_in_flight(mq) == 0)
					get_put = -1;
			}
		} else {
			if (get_put < 0) {
				get_put = 0;
				mmc_put_card(card);
			}
			/*
			 * Do not stop with requests in flight in case recovery
			 * is needed.
			 */
			if (kthread_should_stop() &&
			    !mmc_cqe_tot_in_flight(mq)) {
				set_current_state(TASK_RUNNING);
				break;
			}
			up(&mq->thread_sem);
			schedule();
			down(&mq->thread_sem);
			spin_lock_irqsave(q->queue_lock, flags);
		}
	} /* loop */
	up(&mq->thread_sem);

	return 0;
}

static enum blk_eh_timer_return __mmc_cqe_timed_out(struct request *req)
{
	struct mmc_queue_req *mqrq = req_to_mmc_queue_req(req);
	struct mmc_request *mrq = &mqrq->brq.mrq;
	struct mmc_queue *mq = req->q->queuedata;
	struct mmc_host *host = mq->card->host;
	enum mmc_issue_type issue_type = mmc_cqe_issue_type(host, req);
	bool recovery_needed = false;

	switch (issue_type) {
	case MMC_ISSUE_ASYNC:
	case MMC_ISSUE_DCMD:
		if (host->cqe_ops->cqe_timeout(host, mrq, &recovery_needed)) {
			if (recovery_needed)
				__mmc_cqe_recovery_notifier(mq);
			return BLK_EH_RESET_TIMER;
		}
		/* No timeout */
		return BLK_EH_HANDLED;
	default:
		/* Timeout is handled by mmc core */
		return BLK_EH_RESET_TIMER;
	}
}

static enum blk_eh_timer_return mmc_cqe_timed_out(struct request *req)
{
	struct mmc_queue *mq = req->q->queuedata;

	if (mq->cqe_recovery_needed)
		return BLK_EH_RESET_TIMER;

	return __mmc_cqe_timed_out(req);
}

static int mmc_queue_thread(void *d)
{
	struct mmc_queue *mq = d;
	struct request_queue *q = mq->queue;
	struct mmc_context_info *cntx = &mq->card->host->context_info;

	current->flags |= PF_MEMALLOC;

	down(&mq->thread_sem);
	do {
		struct request *req;

		spin_lock_irq(q->queue_lock);
		set_current_state(TASK_INTERRUPTIBLE);
		req = blk_fetch_request(q);
		mq->asleep = false;
		cntx->is_waiting_last_req = false;
		cntx->is_new_req = false;
		if (!req) {
			/*
			 * Dispatch queue is empty so set flags for
			 * mmc_request_fn() to wake us up.
			 */
			if (mq->qcnt)
				cntx->is_waiting_last_req = true;
			else
				mq->asleep = true;
		}
		spin_unlock_irq(q->queue_lock);

		if (req || mq->qcnt) {
			set_current_state(TASK_RUNNING);
			mmc_blk_issue_rq(mq, req);
			cond_resched();
		} else {
			if (kthread_should_stop()) {
				set_current_state(TASK_RUNNING);
				break;
			}
			up(&mq->thread_sem);
			schedule();
			down(&mq->thread_sem);
		}
	} while (1);
	up(&mq->thread_sem);

	return 0;
}

/*
 * Generic MMC request handler.  This is called for any queue on a
 * particular host.  When the host is not busy, we look for a request
 * on any queue on this host, and attempt to issue it.  This may
 * not be the queue we were asked to process.
 */
static void mmc_request_fn(struct request_queue *q)
{
	struct mmc_queue *mq = q->queuedata;
	struct request *req;
	struct mmc_context_info *cntx;

	if (!mq) {
		while ((req = blk_fetch_request(q)) != NULL) {
			req->rq_flags |= RQF_QUIET;
			__blk_end_request_all(req, -EIO);
		}
		return;
	}

	cntx = &mq->card->host->context_info;

	if (cntx->is_waiting_last_req) {
		cntx->is_new_req = true;
		wake_up_interruptible(&cntx->wait);
	}

	if (mq->asleep)
		wake_up_process(mq->thread);
}

static struct scatterlist *mmc_alloc_sg(int sg_len, gfp_t gfp)
{
	struct scatterlist *sg;

	sg = kmalloc_array(sg_len, sizeof(*sg), gfp);
	if (sg)
		sg_init_table(sg, sg_len);

	return sg;
}

static void mmc_queue_setup_discard(struct request_queue *q,
				    struct mmc_card *card)
{
	unsigned max_discard;

	max_discard = mmc_calc_max_discard(card);
	if (!max_discard)
		return;

	queue_flag_set_unlocked(QUEUE_FLAG_DISCARD, q);
	blk_queue_max_discard_sectors(q, max_discard);
	if (card->erased_byte == 0 && !mmc_can_discard(card))
		q->limits.discard_zeroes_data = 1;
	q->limits.discard_granularity = card->pref_erase << 9;
	/* granularity must not be greater than max. discard */
	if (card->pref_erase > max_discard)
		q->limits.discard_granularity = 0;
	if (mmc_can_secure_erase_trim(card))
		queue_flag_set_unlocked(QUEUE_FLAG_SECERASE, q);
}

static unsigned int mmc_queue_calc_bouncesz(struct mmc_host *host)
{
	unsigned int bouncesz = MMC_QUEUE_BOUNCESZ;

	if (host->max_segs != 1 || (host->caps & MMC_CAP_NO_BOUNCE_BUFF))
		return 0;

	if (bouncesz > host->max_req_size)
		bouncesz = host->max_req_size;
	if (bouncesz > host->max_seg_size)
		bouncesz = host->max_seg_size;
	if (bouncesz > host->max_blk_count * 512)
		bouncesz = host->max_blk_count * 512;

	if (bouncesz <= 512)
		return 0;

	return bouncesz;
}

/**
 * mmc_init_request() - initialize the MMC-specific per-request data
 * @q: the request queue
 * @req: the request
 * @gfp: memory allocation policy
 */
static int mmc_init_request(struct request_queue *q, struct request *req,
			    gfp_t gfp)
{
	struct mmc_queue_req *mq_rq = req_to_mmc_queue_req(req);
	struct mmc_queue *mq = q->queuedata;
	struct mmc_card *card = mq->card;
	struct mmc_host *host = card->host;

	mq_rq->req = req;

	if (card->bouncesz) {
		mq_rq->bounce_buf = kmalloc(card->bouncesz, gfp);
		if (!mq_rq->bounce_buf)
			return -ENOMEM;
		if (card->bouncesz > 512) {
			mq_rq->sg = mmc_alloc_sg(1, gfp);
			if (!mq_rq->sg)
				return -ENOMEM;
			mq_rq->bounce_sg = mmc_alloc_sg(card->bouncesz / 512,
							gfp);
			if (!mq_rq->bounce_sg)
				return -ENOMEM;
		}
	} else {
		mq_rq->bounce_buf = NULL;
		mq_rq->bounce_sg = NULL;
		mq_rq->sg = mmc_alloc_sg(host->max_segs, gfp);
		if (!mq_rq->sg)
			return -ENOMEM;
	}

	return 0;
}

static void mmc_exit_request(struct request_queue *q, struct request *req)
{
	struct mmc_queue_req *mq_rq = req_to_mmc_queue_req(req);

	/* It is OK to kfree(NULL) so this will be smooth */
	kfree(mq_rq->bounce_sg);
	mq_rq->bounce_sg = NULL;

	kfree(mq_rq->bounce_buf);
	mq_rq->bounce_buf = NULL;

	kfree(mq_rq->sg);
	mq_rq->sg = NULL;

	mq_rq->req = NULL;
}

/**
 * mmc_init_queue - initialise a queue structure.
 * @mq: mmc queue
 * @card: mmc card to attach this queue
 * @lock: queue lock
 * @subname: partition subname
 *
 * Initialise a MMC card request queue.
 */
int mmc_init_queue(struct mmc_queue *mq, struct mmc_card *card,
		   spinlock_t *lock, const char *subname, int area_type)
{
	struct mmc_host *host = card->host;
	u64 limit = BLK_BOUNCE_HIGH;
	int ret = -ENOMEM;
	bool use_cqe = host->cqe_enabled && area_type != MMC_BLK_DATA_AREA_RPMB;

	if (mmc_dev(host)->dma_mask && *mmc_dev(host)->dma_mask)
		limit = (u64)dma_max_pfn(mmc_dev(host)) << PAGE_SHIFT;

	mq->card = card;
	mq->queue = blk_alloc_queue(GFP_KERNEL);
	if (!mq->queue)
		return -ENOMEM;
	mq->queue->queue_lock = lock;
	mq->queue->request_fn = use_cqe ? mmc_cqe_request_fn : mmc_request_fn;
	mq->queue->init_rq_fn = mmc_init_request;
	mq->queue->exit_rq_fn = mmc_exit_request;
	mq->queue->cmd_size = sizeof(struct mmc_queue_req);
	mq->queue->queuedata = mq;
	mq->qcnt = 0;
	ret = blk_init_allocated_queue(mq->queue);
	if (ret) {
		blk_cleanup_queue(mq->queue);
		return ret;
	}

	if (use_cqe) {
		int q_depth = card->ext_csd.cmdq_depth;

		if (q_depth > host->cqe_qdepth)
			q_depth = host->cqe_qdepth;

		ret = blk_queue_init_tags(mq->queue, q_depth, NULL,
					  BLK_TAG_ALLOC_FIFO);
		if (ret)
			goto cleanup_queue;

		blk_queue_softirq_done(mq->queue, mmc_blk_cqe_complete_rq);
		blk_queue_rq_timed_out(mq->queue, mmc_cqe_timed_out);
		blk_queue_rq_timeout(mq->queue, 60 * HZ);

		host->cqe_recovery_notifier = mmc_cqe_recovery_notifier;
	}

	blk_queue_prep_rq(mq->queue, mmc_prep_request);
	queue_flag_set_unlocked(QUEUE_FLAG_NONROT, mq->queue);
	queue_flag_clear_unlocked(QUEUE_FLAG_ADD_RANDOM, mq->queue);
	if (mmc_can_erase(card))
		mmc_queue_setup_discard(mq->queue, card);

	card->bouncesz = mmc_queue_calc_bouncesz(host);
	if (card->bouncesz) {
		blk_queue_bounce_limit(mq->queue, BLK_BOUNCE_ANY);
		blk_queue_max_hw_sectors(mq->queue, card->bouncesz / 512);
		blk_queue_max_segments(mq->queue, card->bouncesz / 512);
		blk_queue_max_segment_size(mq->queue, card->bouncesz);
	} else {
		blk_queue_bounce_limit(mq->queue, limit);
		blk_queue_max_hw_sectors(mq->queue,
			min(host->max_blk_count, host->max_req_size / 512));
		blk_queue_max_segments(mq->queue, host->max_segs);
		blk_queue_max_segment_size(mq->queue, host->max_seg_size);
	}

	sema_init(&mq->thread_sem, 1);

	mq->thread = kthread_run(use_cqe ? mmc_cqe_thread : mmc_queue_thread,
				 mq, "mmcqd/%d%s", host->index,
				 subname ? subname : "");
	if (IS_ERR(mq->thread)) {
		ret = PTR_ERR(mq->thread);
		goto cleanup_queue;
	}

	return 0;

cleanup_queue:
	blk_cleanup_queue(mq->queue);
	return ret;
}

void mmc_cleanup_queue(struct mmc_queue *mq)
{
	struct request_queue *q = mq->queue;
	unsigned long flags;

	/* Make sure the queue isn't suspended, as that will deadlock */
	mmc_queue_resume(mq);

	/* Then terminate our worker thread */
	kthread_stop(mq->thread);

	/* Empty the queue */
	spin_lock_irqsave(q->queue_lock, flags);
	q->queuedata = NULL;
	blk_start_queue(q);
	spin_unlock_irqrestore(q->queue_lock, flags);

	mq->card = NULL;
}
EXPORT_SYMBOL(mmc_cleanup_queue);

/**
 * mmc_queue_suspend - suspend a MMC request queue
 * @mq: MMC queue to suspend
 *
 * Stop the block request queue, and wait for our thread to
 * complete any outstanding requests.  This ensures that we
 * won't suspend while a request is being processed.
 */
void mmc_queue_suspend(struct mmc_queue *mq)
{
	struct request_queue *q = mq->queue;
	unsigned long flags;

	if (!mq->suspended) {
		mq->suspended |= true;

		spin_lock_irqsave(q->queue_lock, flags);
		blk_stop_queue(q);
		spin_unlock_irqrestore(q->queue_lock, flags);

		down(&mq->thread_sem);
	}
}

/**
 * mmc_queue_resume - resume a previously suspended MMC request queue
 * @mq: MMC queue to resume
 */
void mmc_queue_resume(struct mmc_queue *mq)
{
	struct request_queue *q = mq->queue;
	unsigned long flags;

	if (mq->suspended) {
		mq->suspended = false;

		up(&mq->thread_sem);

		spin_lock_irqsave(q->queue_lock, flags);
		blk_start_queue(q);
		spin_unlock_irqrestore(q->queue_lock, flags);
	}
}

/*
 * Prepare the sg list(s) to be handed of to the host driver
 */
unsigned int mmc_queue_map_sg(struct mmc_queue *mq, struct mmc_queue_req *mqrq)
{
	unsigned int sg_len;
	size_t buflen;
	struct scatterlist *sg;
	int i;

	if (!mqrq->bounce_buf)
		return blk_rq_map_sg(mq->queue, mqrq->req, mqrq->sg);

	sg_len = blk_rq_map_sg(mq->queue, mqrq->req, mqrq->bounce_sg);

	mqrq->bounce_sg_len = sg_len;

	buflen = 0;
	for_each_sg(mqrq->bounce_sg, sg, sg_len, i)
		buflen += sg->length;

	sg_init_one(mqrq->sg, mqrq->bounce_buf, buflen);

	return 1;
}

/*
 * If writing, bounce the data to the buffer before the request
 * is sent to the host driver
 */
void mmc_queue_bounce_pre(struct mmc_queue_req *mqrq)
{
	if (!mqrq->bounce_buf)
		return;

	if (rq_data_dir(mqrq->req) != WRITE)
		return;

	sg_copy_to_buffer(mqrq->bounce_sg, mqrq->bounce_sg_len,
		mqrq->bounce_buf, mqrq->sg[0].length);
}

/*
 * If reading, bounce the data from the buffer after the request
 * has been handled by the host driver
 */
void mmc_queue_bounce_post(struct mmc_queue_req *mqrq)
{
	if (!mqrq->bounce_buf)
		return;

	if (rq_data_dir(mqrq->req) != READ)
		return;

	sg_copy_from_buffer(mqrq->bounce_sg, mqrq->bounce_sg_len,
		mqrq->bounce_buf, mqrq->sg[0].length);
}
