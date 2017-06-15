#ifndef MMC_QUEUE_H
#define MMC_QUEUE_H

#include <linux/types.h>
#include <linux/blkdev.h>
#include <linux/blk-mq.h>
#include <linux/mmc/core.h>
#include <linux/mmc/host.h>

enum mmc_issued {
	MMC_REQ_STARTED,
	MMC_REQ_BUSY,
	MMC_REQ_FAILED_TO_START,
	MMC_REQ_FINISHED,
};

enum mmc_issue_type {
	MMC_ISSUE_SYNC,
	MMC_ISSUE_DCMD,
	MMC_ISSUE_ASYNC,
	MMC_ISSUE_MAX,
};

static inline struct mmc_queue_req *req_to_mmc_queue_req(struct request *rq)
{
	return blk_mq_rq_to_pdu(rq);
}

static inline bool mmc_req_is_special(struct request *req)
{
	return req &&
		(req_op(req) == REQ_OP_FLUSH ||
		 req_op(req) == REQ_OP_DISCARD ||
		 req_op(req) == REQ_OP_SECURE_ERASE);
}

struct task_struct;
struct mmc_blk_data;

struct mmc_blk_request {
	struct mmc_request	mrq;
	struct mmc_command	sbc;
	struct mmc_command	cmd;
	struct mmc_command	stop;
	struct mmc_data		data;
	int			retune_retry_done;
};

struct mmc_queue_req {
	struct request		*req;
	struct mmc_blk_request	brq;
	struct scatterlist	*sg;
	char			*bounce_buf;
	struct scatterlist	*bounce_sg;
	unsigned int		bounce_sg_len;
	struct mmc_async_req	areq;
	int			retries;
};

struct mmc_queue {
	struct mmc_card		*card;
	struct task_struct	*thread;
	struct semaphore	thread_sem;
	bool			suspended;
	bool			asleep;
	struct mmc_blk_data	*blkdata;
	struct request_queue	*queue;
	/*
	 * FIXME: this counter is not a very reliable way of keeping
	 * track of how many requests that are ongoing. Switch to just
	 * letting the block core keep track of requests and per-request
	 * associated mmc_queue_req data.
	 */
	int			qcnt;
	/* Following are defined for a Command Queue Engine */
	int			cqe_in_flight[MMC_ISSUE_MAX];
	unsigned int		cqe_busy;
	bool			cqe_recovery_needed;
	bool			cqe_in_recovery;
#define MMC_CQE_DCMD_BUSY	BIT(0)
#define MMC_CQE_QUEUE_FULL	BIT(1)
};

extern int mmc_init_queue(struct mmc_queue *, struct mmc_card *, spinlock_t *,
			  const char *, int);
extern void mmc_cleanup_queue(struct mmc_queue *);
extern void mmc_queue_suspend(struct mmc_queue *);
extern void mmc_queue_resume(struct mmc_queue *);

extern unsigned int mmc_queue_map_sg(struct mmc_queue *,
				     struct mmc_queue_req *);
extern void mmc_queue_bounce_pre(struct mmc_queue_req *);
extern void mmc_queue_bounce_post(struct mmc_queue_req *);

extern int mmc_access_rpmb(struct mmc_queue *);

void mmc_cqe_kick_queue(struct mmc_queue *mq);

enum mmc_issue_type mmc_cqe_issue_type(struct mmc_host *host,
				       struct request *req);

static inline int mmc_cqe_tot_in_flight(struct mmc_queue *mq)
{
	return mq->cqe_in_flight[MMC_ISSUE_SYNC] +
	       mq->cqe_in_flight[MMC_ISSUE_DCMD] +
	       mq->cqe_in_flight[MMC_ISSUE_ASYNC];
}

static inline int mmc_cqe_qcnt(struct mmc_queue *mq)
{
	return mq->cqe_in_flight[MMC_ISSUE_DCMD] +
	       mq->cqe_in_flight[MMC_ISSUE_ASYNC];
}

#endif
