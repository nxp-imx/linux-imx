/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2022 ARM Ltd.
 */

#ifndef _TRUSTY_TRANSPORT_H
#define _TRUSTY_TRANSPORT_H

#include <linux/device.h>
#include <linux/types.h>

/* 'TrsTrnsp' in hex */
#define TRUSTY_TRANSPORT_MAGIC 0x54727354726E7370ULL

struct ns_mem_page_info;
struct trusty_sched_share_state;
struct trusty_transport;

struct trusty_transport_ops {
	unsigned long (*call)(struct trusty_transport *tr, unsigned long smcnr,
			      unsigned long a0, unsigned long a1,
			      unsigned long a2);
	int (*share_or_lend_memory)(struct trusty_transport *tr, u64 *id,
				    struct scatterlist *sglist,
				    unsigned int nents, pgprot_t pgprot, u64 tag,
				    bool lend, struct ns_mem_page_info *pg_inf);
	int (*reclaim_memory)(struct trusty_transport *tr, u64 id,
			      struct scatterlist *sglist,
			      unsigned int nents);
	void (*set_sched_share_state)(struct trusty_transport *tr,
				      struct trusty_sched_share_state *tsss);
};

struct trusty_transport {
	u64 magic;
	const struct trusty_transport_ops *ops;

	/*
	 * Whether the NOP calls with arguments are invoked
	 * separately from the no-arguments calls that give
	 * Trusty cycles (like on the FF-A) transport.
	 * Set to 0 for transports where every SMC_SC_NOP
	 * always give Trusty scheduler cycles.
	 */
	unsigned split_nopcalls:1;
};

#endif /* _TRUSTY_TRANSPORT_H */
