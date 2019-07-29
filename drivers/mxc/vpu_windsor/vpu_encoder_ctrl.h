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

#ifndef _VPU_ENCODER_CTRL_H
#define _VPU_ENCODER_CTRL_H

#include "mediasys_types.h"

int vpu_enc_setup_ctrls(struct vpu_ctx *ctx);
int vpu_enc_free_ctrls(struct vpu_ctx *ctx);

#endif
