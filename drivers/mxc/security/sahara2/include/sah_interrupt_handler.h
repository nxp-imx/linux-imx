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
* @file sah_interrupt_handler.h
*
* @brief Provides a hardware interrupt handling mechanism for device driver.
*
*/
/******************************************************************************
*
* CAUTION:
*
* MODIFICATION HISTORY:
*
* Date      Person     Change
* 30/07/03  MW         Initial Creation
*******************************************************************
*/

#ifndef SAH_INTERRUPT_HANDLER_H
#define SAH_INTERRUPT_HANDLER_H

#include <sah_driver_common.h>

/******************************************************************************
* External function declarations
******************************************************************************/
int sah_Intr_Init (wait_queue_head_t *wait_queue);
void sah_Intr_Release (void);

#endif  /* SAH_INTERRUPT_HANDLER_H */
