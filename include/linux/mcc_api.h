/*
 * This file contains prototypes for MCC library API functions
 *
 * Copyright (C) 2014 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *
 * SPDX-License-Identifier: GPL-2.0+ and/or BSD-3-Clause
 * The GPL-2.0+ license for this file can be found in the COPYING.GPL file
 * included with this distribution or at
 * http://www.gnu.org/licenses/gpl-2.0.html
 * The BSD-3-Clause License for this file can be found in the COPYING.BSD file
 * included with this distribution or at
 * http://opensource.org/licenses/BSD-3-Clause
 */

#ifndef __MCC_API__
#define __MCC_API__

int mcc_initialize(MCC_NODE);
int mcc_destroy(MCC_NODE);
int mcc_create_endpoint(MCC_ENDPOINT*, MCC_PORT);
int mcc_destroy_endpoint(MCC_ENDPOINT*);
int mcc_send(MCC_ENDPOINT*, MCC_ENDPOINT*, void*, MCC_MEM_SIZE, unsigned int);
int mcc_recv(MCC_ENDPOINT*, MCC_ENDPOINT*, void*, MCC_MEM_SIZE, MCC_MEM_SIZE*, unsigned int);
int mcc_msgs_available(MCC_ENDPOINT*, unsigned int*);
int mcc_get_info(MCC_NODE, MCC_INFO_STRUCT*);

#if MCC_SEND_RECV_NOCOPY_API_ENABLED
int mcc_get_buffer(void**, MCC_MEM_SIZE*, unsigned int);
int mcc_send_nocopy(MCC_ENDPOINT*, MCC_ENDPOINT*, void*, MCC_MEM_SIZE);
int mcc_recv_nocopy(MCC_ENDPOINT*, MCC_ENDPOINT*, void**, MCC_MEM_SIZE*, unsigned int);
int mcc_free_buffer(void*);
#endif /* MCC_SEND_RECV_NOCOPY_API_ENABLED */

#endif /* __MCC_API__ */

