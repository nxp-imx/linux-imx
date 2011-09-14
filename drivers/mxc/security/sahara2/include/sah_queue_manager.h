/*
 * Copyright (C) 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
* @file sah_queue_manager.h
*
* @brief This file definitions for the Queue Manager.

* The Queue Manager manages additions and removal from the queue and updates
* the status of queue entries. It also calls sah_HW_* functions to interact
* with the hardware.
*
*/

#ifndef SAH_QUEUE_MANAGER_H
#define SAH_QUEUE_MANAGER_H

#include <sah_driver_common.h>
#include <sah_status_manager.h>


/*************************
* Queue Manager Functions
*************************/
fsl_shw_return_t sah_Queue_Manager_Init(void);
void             sah_Queue_Manager_Close(void);
void             sah_Queue_Manager_Reset_Entries(void);
void             sah_Queue_Manager_Append_Entry(sah_Head_Desc *entry);
void             sah_Queue_Manager_Remove_Entry(sah_Head_Desc *entry);


/*************************
* Queue Functions
*************************/
sah_Queue *sah_Queue_Construct(void);
void      sah_Queue_Destroy(sah_Queue *this);
void      sah_Queue_Append_Entry(sah_Queue *this, sah_Head_Desc *entry);
void      sah_Queue_Remove_Entry(sah_Queue *this);
void      sah_Queue_Remove_Any_Entry(sah_Queue *this, sah_Head_Desc *entry);
void      sah_postprocess_queue(unsigned long reset_flag);


/*************************
* Misc Releated Functions
*************************/

int              sah_blocking_mode(struct sah_Head_Desc *entry);
fsl_shw_return_t sah_convert_error_status(uint32_t error_status);


#endif  /* SAH_QUEUE_MANAGER_H */

/* End of sah_queue_manager.h */
