/*
 * Copyright 2018-2019 NXP
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef _VPU_EVENT_MSG_H
#define _VPU_EVENT_MSG_H

#include "vpu_encoder_config.h"

struct vpu_event_msg {
	struct list_head list;
	u32 idx;
	u32 msgid;
	u32 number;
	u32 data[MSG_DATA_DEFAULT_SIZE];
	u32 *ext_data;
};

int init_ctx_msg_queue(struct vpu_ctx *ctx);
void cleanup_ctx_msg_queue(struct vpu_ctx *ctx);
struct vpu_event_msg *get_idle_msg(struct vpu_ctx *ctx);
void put_idle_msg(struct vpu_ctx *ctx, struct vpu_event_msg *msg);
struct vpu_event_msg *pop_event_msg(struct vpu_ctx *ctx);
void push_back_event_msg(struct vpu_ctx *ctx, struct vpu_event_msg *msg);
int alloc_msg_ext_buffer(struct vpu_event_msg *msg, u32 number);
void free_msg_ext_buffer(struct vpu_event_msg *msg);
long long get_total_ext_data_number(void);

#endif
