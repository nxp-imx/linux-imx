/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2025 NXP
 */

#ifndef __ELE_BBSM_H__
#define __ELE_BBSM_H__

#include "se_ctrl.h"

#define IMX_SIP_BBSM			0xC200000D
#define IMX_SIP_BBSM_CLEAR_INTERRUPT	0x01
#define IMX_SIP_BBSM_READ_TAMPER_STATUS 0x02

int ele_bbsm_irq_register(struct se_if_priv *priv);
int ele_bbsm_get_tamper_status(struct se_if_priv *priv);
#endif
