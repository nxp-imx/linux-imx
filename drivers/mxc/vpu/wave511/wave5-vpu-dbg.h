/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Wave5 series multi-standard codec IP - debug interface
 *
 * Copyright (C) 2026 CHIPS&MEDIA INC
 */

#ifndef __WAVE5_VPU_DBG_H__
#define __WAVE5_VPU_DBG_H__

#include <linux/debugfs.h>

#define WAVE5_VPU_DEBUGFS_DIR "wave5"

int wave5_vpu_create_dbgfs_file(struct vpu_instance *inst);
void wave5_vpu_remove_dbgfs_file(struct vpu_instance *inst);

#endif
