/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __KVM_ARM_SMMU_V3_LIB_HYP_H
#define __KVM_ARM_SMMU_V3_LIB_HYP_H

#include <nvhe/mem_protect.h>
#include <nvhe/pkvm.h>

#include "arm_smmu_v3.h"

/*
 * Wait until @cond is true.
 * Return 0 on success, or -ETIMEDOUT
 */
#define smmu_wait(use_wfe, _cond)					\
({								\
	int __ret = 0;						\
	u64 delay = pkvm_time_get() + ARM_SMMU_POLL_TIMEOUT_US;	\
								\
	while (!(_cond)) {					\
		if (use_wfe)					\
			wfe();					\
		if (pkvm_time_get() >= delay) {			\
			__ret = -ETIMEDOUT;			\
			break;					\
		}						\
	}							\
	__ret;							\
})


static inline bool smmu_cmdq_full(struct arm_smmu_queue *cmdq)
{
	struct arm_smmu_ll_queue *llq = &cmdq->llq;

	WRITE_ONCE(llq->cons, readl_relaxed(cmdq->cons_reg));
	return queue_full(llq);
}

static inline bool smmu_cmdq_empty(struct arm_smmu_queue *cmdq)
{
	struct arm_smmu_ll_queue *llq = &cmdq->llq;

	WRITE_ONCE(llq->cons, readl_relaxed(cmdq->cons_reg));
	return queue_empty(llq);
}

static inline void smmu_add_cmd_raw(struct hyp_arm_smmu_v3_device *smmu,
				    u64 *cmd)
{
	struct arm_smmu_queue *q = &smmu->cmdq;
	struct arm_smmu_ll_queue *llq = &q->llq;

	queue_write(Q_ENT(q, llq->prod), cmd,  CMDQ_ENT_DWORDS);
	llq->prod = queue_inc_prod_n(llq, 1);
}

int smmu_take_pages(u64 phys, size_t size);
void smmu_reclaim_pages(u64 phys, size_t size);
int smmu_add_cmd(struct hyp_arm_smmu_v3_device *smmu,
		 struct arm_smmu_cmdq_ent *ent);
int smmu_sync_cmd(struct hyp_arm_smmu_v3_device *smmu);
bool smmu_cmdq_has_space(struct arm_smmu_queue *cmdq, u32 n);

#endif
