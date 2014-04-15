/*
 * Copyright (C) 2014 Freescale Semiconductor, Inc.
 * Freescale IMX Linux-specific MCC implementation.
 * Prototypes for MCC library API functions.
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

/*
 * MCC return values
 */
#define MCC_SUCCESS         (0) /* function returned successfully */
#define MCC_ERR_TIMEOUT     (1) /* blocking function timed out before completing */
#define MCC_ERR_INVAL       (2) /* invalid input parameter */
#define MCC_ERR_NOMEM       (3) /* out of shared memory for message transmission */
#define MCC_ERR_ENDPOINT    (4) /* invalid endpoint / endpoint doesn't exist */
#define MCC_ERR_SEMAPHORE   (5) /* semaphore handling error */
#define MCC_ERR_DEV         (6) /* Device Open Error */
#define MCC_ERR_INT         (7) /* Interrupt Error */
#define MCC_ERR_SQ_FULL     (8) /* Signal queue is full */
#define MCC_ERR_SQ_EMPTY    (9) /* Signal queue is empty */

int mcc_initialize(MCC_NODE);
int mcc_destroy(MCC_NODE);
int mcc_create_endpoint(MCC_ENDPOINT*, MCC_PORT);
int mcc_destroy_endpoint(MCC_ENDPOINT*);
int mcc_send(MCC_ENDPOINT*, void*, MCC_MEM_SIZE, unsigned int);
int mcc_recv_copy(MCC_ENDPOINT*, void*, MCC_MEM_SIZE, MCC_MEM_SIZE*, unsigned int);
int mcc_recv_nocopy(MCC_ENDPOINT*, void**, MCC_MEM_SIZE*, unsigned int);
int mcc_msgs_available(MCC_ENDPOINT*, unsigned int*);
int mcc_free_buffer(void*);
int mcc_get_info(MCC_NODE, MCC_INFO_STRUCT*);

#endif /* __MCC_API__ */
