// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2024 NXP
 */

#include "se_msg_sqfl_ctrl.h"

void se_start_enforce_msg_seq_flow(struct se_msg_seq_ctrl *se_msg_sq_ctl)
{
	mutex_lock(&se_msg_sq_ctl->se_msg_sq_lk);
	se_msg_sq_ctl->exp_tx_msg = NULL;
}

void se_continue_to_enforce_msg_seq_flow(struct se_msg_seq_ctrl *se_msg_sq_ctl,
					 void *tx_msg)
{
	se_msg_sq_ctl->exp_tx_msg = tx_msg;
}

void se_halt_to_enforce_msg_seq_flow(struct se_msg_seq_ctrl *se_msg_sq_ctl)
{
	se_msg_sq_ctl->exp_tx_msg = NULL;
	mutex_unlock(&se_msg_sq_ctl->se_msg_sq_lk);
}

bool se_qualify_msg_seq_flow(struct se_msg_seq_ctrl *se_msg_sq_ctl, void *tx_msg)
{
	/* Fast-path: if this caller is the sequence owner,
	 * proceed without blocking.
	 */
	if (se_msg_sq_ctl->exp_tx_msg == tx_msg) {
		se_msg_sq_ctl->exp_tx_msg = NULL;
		return false;
	}

	/* Sequence in progress and we are not the owner: wait for it to finish. */
	mutex_lock(&se_msg_sq_ctl->se_msg_sq_lk);
	return true;
}
