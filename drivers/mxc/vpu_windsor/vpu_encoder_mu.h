/*
 * Copyright 2019 NXP
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 *
 * @file vpu_encoder_mu.h
 *
 */

#ifndef _VPU_ENCODER_MU_H_
#define _VPU_ENCODER_MU_H_

#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/mx8_mu.h>
#include <linux/workqueue.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/mailbox_client.h>
#include <linux/pm_domain.h>
#include <linux/kfifo.h>

#include "vpu_encoder_b0.h"

int vpu_enc_mu_request(struct core_device *core_dev);
void vpu_enc_mu_free(struct core_device *core_dev);
void vpu_enc_mu_send_msg(struct core_device *core_dev, MSG_Type type, u_int32 value);
u_int32 vpu_enc_mu_receive_msg(struct core_device *core_dev, void *msg);
int vpu_enc_sc_check_fuse(void);
void vpu_enc_mu_enable_rx(struct core_device *core_dev);

#endif
