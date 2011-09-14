/*
 * Copyright (C) 2004-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/**
* @file sah_driver_common.h
*
* @brief Provides types and defined values for use in the Driver Interface.
*
*/

#ifndef SAH_DRIVER_COMMON_H
#define SAH_DRIVER_COMMON_H

#include "fsl_platform.h"
#include <linux/mxc_sahara.h>
#include <adaptor.h>

/** This specifies the permissions for the device file. It is equivalent to
 * chmod 666.
 */
#define SAHARA_DEVICE_MODE S_IFCHR | S_IRUGO | S_IWUGO

/**
* The status of entries in the Queue.
*
******************************************************************************/
typedef enum
{
    /** This state indicates that the entry is in the queue and awaits
     *  execution on SAHARA. */
    SAH_STATE_PENDING,
    /** This state indicates that the entry has been written to the SAHARA
     *  DAR. */
    SAH_STATE_ON_SAHARA,
    /** This state indicates that the entry is off of SAHARA, and is awaiting
        post-processing. */
    SAH_STATE_OFF_SAHARA,
    /** This state indicates that the entry is successfully executed on SAHARA,
        and it is finished with post-processing. */
    SAH_STATE_COMPLETE,
    /** This state indicates that the entry caused an error or fault on SAHARA,
     *  and it is finished with post-processing. */
    SAH_STATE_FAILED,
    /** This state indicates that the entry was reset via the Reset IO
     *  Control, and it is finished with post-processing. */
    SAH_STATE_RESET,
    /** This state indicates that the entry was signalled from user-space and
     *  either in the DAR, IDAR or has finished executing pending Bottom Half
     *  processing. */
   SAH_STATE_IGNORE,
    /** This state indicates that the entry was signalled from user-space and
     *  has been processed by the bottom half. */
   SAH_STATE_IGNORED
} sah_Queue_Status;

/* any of these conditions being true indicates the descriptor's processing
 * is complete */
#define SAH_DESC_PROCESSED(status)                             \
                        (((status) == SAH_STATE_COMPLETE) ||   \
                         ((status) == SAH_STATE_FAILED  ) ||   \
                         ((status) == SAH_STATE_RESET   ))

extern os_lock_t desc_queue_lock;

extern uint32_t dar_count;
extern uint32_t interrupt_count;
extern uint32_t done1done2_count;
extern uint32_t done1busy2_count;
extern uint32_t done1_count;

#ifdef FSL_HAVE_SCC2
extern void *lookup_user_partition(fsl_shw_uco_t * user_ctx,
				   uint32_t user_base);
#endif

int sah_get_results_pointers(fsl_shw_uco_t* user_ctx, uint32_t arg);
fsl_shw_return_t sah_get_results_from_pool(volatile fsl_shw_uco_t* user_ctx,
                                           sah_results *arg);
fsl_shw_return_t sah_handle_registration(fsl_shw_uco_t *user_cts);
fsl_shw_return_t sah_handle_deregistration(fsl_shw_uco_t *user_cts);

int sah_Queue_Manager_Count_Entries(int ignore_state, sah_Queue_Status state);
unsigned long sah_Handle_Poll(sah_Head_Desc *entry);

#ifdef DIAG_DRV_IF
/******************************************************************************
* Descriptor and Link dumping functions.
******************************************************************************/
void sah_Dump_Chain(const sah_Desc *chain, dma_addr_t addr);
#endif /* DIAG_DRV_IF */

#endif  /* SAH_DRIVER_COMMON_H */
