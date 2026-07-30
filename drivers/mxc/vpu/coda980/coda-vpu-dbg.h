/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Coda series multi-standard codec IP - debug interface
 *
 * Copyright (C) 2026 CHIPS&MEDIA INC
 */

#ifndef __CODA_VPU_DBG_H__
#define __CODA_VPU_DBG_H__

#include <linux/debugfs.h>

#define CODA_VPU_DEBUGFS_DIR "coda980"

int coda_vpu_create_dbgfs_file(struct vpu_instance *inst);
void coda_vpu_remove_dbgfs_file(struct vpu_instance *inst);

#endif
