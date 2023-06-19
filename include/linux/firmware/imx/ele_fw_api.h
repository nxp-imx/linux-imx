/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2023 NXP
 */

#ifndef ELE_FW_API_H
#define ELE_FW_API_H

#include <linux/hw_random.h>

#define MESSAGING_VERSION_7		0x7

#define ELE_INIT_FW_REQ                 0x17
#define ELE_GET_RANDOM_REQ		0xCD

int ele_get_random(struct hwrng *rng, void *data, size_t len, bool wait);
int ele_init_fw(void);

#endif /* ELE_FW_API_H */
