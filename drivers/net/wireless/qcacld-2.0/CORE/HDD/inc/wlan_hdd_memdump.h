/*
 * Copyright (c) 2015-2017 The Linux Foundation. All rights reserved.
 *
 * Previously licensed under the ISC license by Qualcomm Atheros, Inc.
 *
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * This file was originally distributed by Qualcomm Atheros, Inc.
 * under proprietary terms before Copyright ownership was assigned
 * to the Linux Foundation.
 */

/**
 * DOC : wlan_hdd_memdump.h
 *
 * WLAN Host Device Driver file for dumping firmware memory
 *
 */

#if !defined(WLAN_HDD_MEMDUMP_H)
#define WLAN_HDD_MEMDUMP_H

#include "wlan_hdd_main.h"

/* Assigned size of driver memory dump is 4096 bytes */
#define DRIVER_MEM_DUMP_SIZE    4096

int hdd_driver_memdump_init(void);
void hdd_driver_memdump_deinit(void);

#endif /* if !defined(WLAN_HDD_MEMDUMP_H)*/
