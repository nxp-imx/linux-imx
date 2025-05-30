/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2024-2025 NXP
 */


#ifndef __ELE_TRNG_H__
#define __ELE_TRNG_H__

#include "se_ctrl.h"

#if IS_ENABLED(CONFIG_IMX_ELE_TRNG)

int ele_trng_init(struct se_if_priv *priv);
int ele_trng_exit(struct se_if_priv *priv);

#else

#define ele_trng_init NULL
inline int ele_trng_exit(struct se_if_priv *priv) {return 0;};

#endif

#endif /*__ELE_TRNG_H__ */
