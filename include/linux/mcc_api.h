/*
 * Copyright (C) 2014 Freescale Semiconductor, Inc.
 * Freescale IMX Linux-specific MCC implementation.
 * MCC library API functions implementation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
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

