/*
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
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
* @file sah_status_manager.h
*
* @brief SAHARA Status Manager Types and Function Prototypes
*
* @author Stuart Holloway (SH)
*
*/

#ifndef STATUS_MANAGER_H
#define STATUS_MANAGER_H
#include "sah_driver_common.h"
#include "sahara.h"


/******************************************************************************
* User defined data types
******************************************************************************/
/**
******************************************************************************
* sah_Execute_Status
* Types read from SAHARA Status Register, with additional state for Op Status
******************************************************************************/
typedef enum sah_Execute_Status
{
    /** Sahara is Idle. */
    SAH_EXEC_IDLE = 0,
    /** SAHARA is busy performing a resest or processing a decriptor chain. */
    SAH_EXEC_BUSY = 1,
    /** An error occurred while SAHARA executed the first descriptor. */
    SAH_EXEC_ERROR1 = 2,
    /** SAHARA has failed internally. */
    SAH_EXEC_FAULT = 3,
    /** SAHARA has finished processing a descriptor chain and is idle. */
    SAH_EXEC_DONE1 = 4,
    /** SAHARA has finished processing a descriptor chain, and is processing a
     *   second chain.  */
    SAH_EXEC_DONE1_BUSY2 = 5,
    /** SAHARA has finished processing a descriptor chain, but has generated an
     *   error while processing a second descriptor chain. */
    SAH_EXEC_DONE1_ERROR2 = 6,
    /** SAHARA has finished two descriptors. */
    SAH_EXEC_DONE1_DONE2 = 7,
    /** SAHARA has stopped, and first descriptor has Op Status, not Err  */
    SAH_EXEC_OPSTAT1 = 0x20,
} sah_Execute_Status;

/**
 * When this bit is on in a #sah_Execute_Status, it means that DONE1 is true.
 */
#define SAH_EXEC_DONE1_BIT  4

/**
 * Bits which make up the Sahara State
 */
#define SAH_EXEC_STATE_MASK 0x00000007

/**
*******************************************************************************
* sah_Execute_Error
* Types read from SAHARA Error Status Register
******************************************************************************/
typedef enum sah_Execute_Error
{
    /** No Error */
    SAH_ERR_NONE = 0,
    /** Header is not valid. */
    SAH_ERR_HEADER = 1,
    /** Descriptor length is not correct. */
    SAH_ERR_DESC_LENGTH = 2,
    /** Length or pointer field is zero while the other is non-zero.  */
    SAH_ERR_DESC_POINTER = 3,
    /** Length of the link is not a multiple of 4 and is not the last link */
    SAH_ERR_LINK_LENGTH = 4,
    /** The data pointer in a link is zero */
    SAH_ERR_LINK_POINTER = 5,
    /** Input Buffer reported an overflow */
    SAH_ERR_INPUT_BUFFER = 6,
    /** Output Buffer reported an underflow */
    SAH_ERR_OUTPUT_BUFFER = 7,
    /** Incorrect data in output buffer after CHA's has signalled 'done'. */
    SAH_ERR_OUTPUT_BUFFER_STARVATION = 8,
    /** Internal Hardware Failure. */
    SAH_ERR_INTERNAL_STATE = 9,
    /** Current Descriptor was not legal, but cause is unknown. */
    SAH_ERR_GENERAL_DESCRIPTOR = 10,
    /** Reserved pointer fields have been set to 1. */
    SAH_ERR_RESERVED_FIELDS = 11,
    /** Descriptor address error */
    SAH_ERR_DESCRIPTOR_ADDRESS = 12,
    /** Link address error */
    SAH_ERR_LINK_ADDRESS = 13,
    /** Processing error in CHA module */
    SAH_ERR_CHA = 14,
    /** Processing error during DMA */
    SAH_ERR_DMA = 15
} sah_Execute_Error;


/**
*******************************************************************************
* sah_CHA_Error_Source
* Types read from SAHARA Error Status Register for CHA Error Source
*
******************************************************************************/
typedef enum sah_CHA_Error_Source
{
    /** No Error indicated in Source CHA Error. */
    SAH_CHA_NO_ERROR = 0,
    /** Error in SKHA module. */
    SAH_CHA_SKHA_ERROR = 1,
    /** Error in MDHA module. */
    SAH_CHA_MDHA_ERROR = 2,
    /** Error in RNG module.  */
    SAH_CHA_RNG_ERROR = 3,
    /** Error in PKHA module.  */
    SAH_CHA_PKHA_ERROR = 4,
} sah_CHA_Error_Source;

/**
******************************************************************************
* sah_CHA_Error_Status
* Types read from SAHARA Error Status Register for CHA Error Status
*
******************************************************************************/
typedef enum sah_CHA_Error_Status
{
    /** No CHA error detected */
    SAH_CHA_NO_ERR = 0x000,
    /** Non-empty input buffer when complete. */
    SAH_CHA_IP_BUF = 0x001,
    /** Illegal Address Error. */
    SAH_CHA_ADD_ERR = 0x002,
    /** Illegal Mode Error.  */
    SAH_CHA_MODE_ERR = 0x004,
    /** Illegal Data Size Error. */
    SAH_CHA_DATA_SIZE_ERR = 0x008,
    /** Illegal Key Size Error. */
    SAH_CHA_KEY_SIZE_ERR = 0x010,
    /** Mode/Context/Key written during processing. */
    SAH_CHA_PROC_ERR = 0x020,
    /** Context Read During Processing. */
    SAH_CHA_CTX_READ_ERR = 0x040,
    /** Internal Hardware Error. */
    SAH_CHA_INTERNAL_HW_ERR = 0x080,
    /** Input Buffer not enabled or underflow. */
    SAH_CHA_IP_BUFF_ERR = 0x100,
    /** Output Buffer not enabled or overflow. */
    SAH_CHA_OP_BUFF_ERR = 0x200,
    /** DES key parity error (SKHA) */
    SAH_CHA_DES_KEY_ERR = 0x400,
    /** Reserved error code. */
    SAH_CHA_RES = 0x800
} sah_CHA_Error_Status;

/**
*****************************************************************************
* sah_DMA_Error_Status
* Types read from SAHARA Error Status Register for DMA Error Status
******************************************************************************/
typedef enum sah_DMA_Error_Status
{
    /** No DMA Error Code. */
    SAH_DMA_NO_ERR = 0,
    /** AHB terminated a bus cycle with an error. */
    SAH_DMA_AHB_ERR = 2,
    /** Internal IP bus cycle was terminated with an error termination. */
    SAH_DMA_IP_ERR = 4,
    /** Parity error detected on DMA command. */
    SAH_DMA_PARITY_ERR = 6,
    /** DMA was requested to cross a 256 byte internal address boundary. */
    SAH_DMA_BOUNDRY_ERR = 8,
    /** DMA controller is busy */
    SAH_DMA_BUSY_ERR = 10,
    /** Memory Bounds Error */
    SAH_DMA_RESERVED_ERR = 12,
    /** Internal DMA hardware error detected */
    SAH_DMA_INT_ERR = 14
} sah_DMA_Error_Status;

/**
*****************************************************************************
* sah_DMA_Error_Size
* Types read from SAHARA Error Status Register for DMA Error Size
*
******************************************************************************/
typedef enum sah_DMA_Error_Size
{
    /** Error during Byte transfer.  */
    SAH_DMA_SIZE_BYTE = 0,
    /** Error during Half-word transfer. */
    SAH_DMA_SIZE_HALF_WORD = 1,
    /** Error during Word transfer. */
    SAH_DMA_SIZE_WORD = 2,
    /** Reserved DMA word size. */
    SAH_DMA_SIZE_RES = 3
} sah_DMA_Error_Size;




extern bool sah_dpm_flag;

/*************************
* Status Manager Functions
*************************/

unsigned long sah_Handle_Interrupt(sah_Execute_Status hw_status);
sah_Head_Desc *sah_Find_With_State (sah_Queue_Status status);
int sah_dpm_init(void);
void sah_dpm_close(void);
void sah_Queue_Manager_Prime (sah_Head_Desc *entry);


#endif  /* STATUS_MANAGER_H */
