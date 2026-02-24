// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025 Google LLC
 * Author: Mostafa Saleh <smostafa@google.com>
 */
#include "arm-smmu-v3-lib-hyp.h"
#include "arm-smmu-v3-module.h"

/* Transfer ownership of memory */
int smmu_take_pages(u64 phys, size_t size)
{
	WARN_ON(!PAGE_ALIGNED(phys) || !PAGE_ALIGNED(size));
	return __pkvm_host_donate_hyp(phys >> PAGE_SHIFT, size >> PAGE_SHIFT);
}

void smmu_reclaim_pages(u64 phys, size_t size)
{
	WARN_ON(!PAGE_ALIGNED(phys) || !PAGE_ALIGNED(size));
	WARN_ON(__pkvm_hyp_donate_host(phys >> PAGE_SHIFT, size >> PAGE_SHIFT));
}

int smmu_add_cmd(struct hyp_arm_smmu_v3_device *smmu,
		 struct arm_smmu_cmdq_ent *ent)
{
	int ret;
	u64 cmd[CMDQ_ENT_DWORDS];

	ret = smmu_wait(smmu->features & ARM_SMMU_FEAT_SEV,
					!smmu_cmdq_full(&smmu->cmdq));
	if (ret)
		return ret;

	ret = arm_smmu_cmdq_build_cmd(cmd, ent);
	if (ret)
		return ret;

	smmu_add_cmd_raw(smmu, cmd);
	writel_relaxed(smmu->cmdq.llq.prod, smmu->cmdq.prod_reg);
	return 0;
}

int smmu_sync_cmd(struct hyp_arm_smmu_v3_device *smmu)
{
	int ret;
	struct arm_smmu_cmdq_ent cmd = {
		.opcode = CMDQ_OP_CMD_SYNC,
	};

	ret = smmu_add_cmd(smmu, &cmd);
	if (ret)
		return ret;

	return smmu_wait(smmu->features & ARM_SMMU_FEAT_SEV,
			 smmu_cmdq_empty(&smmu->cmdq));
}

bool smmu_cmdq_has_space(struct arm_smmu_queue *cmdq, u32 n)
{
	struct arm_smmu_ll_queue *llq = &cmdq->llq;

	WRITE_ONCE(llq->cons, readl_relaxed(cmdq->cons_reg));
	return queue_has_space(llq, n);
}
