/* SPDX-License-Identifier: GPL-2.0-only
 * Copyright (C) 2024 Google, Inc.
 */

#include <linux/linkage.h>

#include <asm/fpsimdmacros.h>

SYM_FUNC_START(trusty_fpsimd_save_state)
	fpsimd_save	x0, 1
	ret
SYM_FUNC_END(trusty_fpsimd_save_state)

SYM_FUNC_START(trusty_fpsimd_load_state)
	fpsimd_restore	x0, 1
	ret
SYM_FUNC_END(trusty_fpsimd_load_state)
