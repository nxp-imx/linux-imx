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
 * @file vpu_mu.h
 *
 */

#ifndef _VPU_MU_H_
#define _VPU_MU_H_

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

#include "vpu_b0.h"
#include "mediasys_types.h"
#include "vpu_debug_log.h"
#include "vpu_rpc.h"

int vpu_mu_request(struct vpu_dev *dev);
void vpu_mu_free(struct vpu_dev *dev);
void vpu_mu_send_msg(struct vpu_dev *dev, MSG_Type type, u_int32 value);
u_int32 vpu_mu_receive_msg(struct vpu_dev *dev, void *msg);
int vpu_sc_check_fuse(struct vpu_dev *dev, struct vpu_v4l2_fmt *pformat_table,
		     u_int32 table_size);
void vpu_mu_enable_rx(struct vpu_dev *dev);

#endif
