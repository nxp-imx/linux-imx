/******************************************************************************
 *
 * Copyright 2007-2008 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 *
 ******************************************************************************
 *
 * File: iapiLow.c
 *
 * $Id iapiLow.c $
 *
 * Description:
 *  This library is written in C to guarantee functionality and integrity in
 * the usage of SDMA virtual DMA channels. This API (Application Programming
 * Interface)  allow SDMA channels' access in an OPEN, READ, WRITE, CLOSE
 * fashion.
 *  These are the LOW level functions of the I.API.
 *
 *
 * /
 *
 * $Log iapiLow.c $
 *
 *****************************************************************************/

/* ****************************************************************************
 * Include File Section
 *****************************************************************************/
#include "epm.h"
#include "iapiLow.h"

/**
 * Function Section
 */


/* ***************************************************************************/
/**Records an ISR callback function pointer into the ISR callback
 * function table
 *
 * @param cd_p channel descriptor to attach callback to
 * @param func_p pointer to the callback function to be registered
 *
 * @return none
 */
void
iapi_AttachCallbackISR (channelDescriptor * cd_p,
                  void (* func_p)(channelDescriptor * cd_p, void * arg))

{
  if (cd_p->callbackSynch == CALLBACK_ISR) {
   iapi_DisableInterrupts();
   callbackIsrTable[cd_p->channelNumber] = func_p;
   iapi_EnableInterrupts();
  } else   if (cd_p->callbackSynch == DEFAULT_POLL) {
   callbackIsrTable[cd_p->channelNumber] = NULL;
  } else {
   iapi_errno = IAPI_ERR_CALLBACKSYNCH_UNKNOWN | IAPI_ERR_CH_AVAILABLE |
     cd_p->channelNumber;
  }
}


/* ***************************************************************************/
/**Detaches (removes) an ISR callback function pointer from the ISR callback
 * function table
 *
 * <b>Algorithm:</b>\n
 *    - Attach a null function to replace the original one.
 *
 * @param cd_p channel descriptor to detach callback from
 *
 * @return none
 */
void
iapi_DetachCallbackISR (channelDescriptor * cd_p)

{
  iapi_AttachCallbackISR (cd_p, NULL);
}

/* ***************************************************************************/
/**Updates an ISR callback function pointer into the ISR callback function
 * table
 *
 * <b>Algorithm:</b>\n
 *    - Detach the old function pointer (if any) and attach the new one
 *
 * @param cd_p channel descriptor to attach callback to
 * @param func_p pointer to the callback function to be registered
 *
 * @return none
 */
void
iapi_ChangeCallbackISR (channelDescriptor * cd_p,
                  void (* func_p)(channelDescriptor * cd_p, void * arg))
{
  iapi_DetachCallbackISR(cd_p);
  iapi_AttachCallbackISR(cd_p, func_p);
}

/* ***************************************************************************/
/**Loop while the channel is not done on the SDMA
 *
 * <b>Algorithm:</b>\n
 *    - Loop doing nothing but checking the I.API global variable to indicate
 * that the channel has been completed (interrupt from SDMA)
 *
 * <b>Notes:</b>\n
 *    - The ISR must update the I.API global variable iapi_SDMAIntr.
 *
 * @param channel channel number to poll on
 *
 * @return none
 */
void
iapi_lowSynchChannel (unsigned char channel)
{
  while (!((1UL << channel) & iapi_SDMAIntr)) ;
  iapi_SDMAIntr &= ~(1UL << channel);
}

/* ***************************************************************************/
/**Fill the buffer descriptor with the values given in parameter.
 *
 * @return none
 */
void
iapi_SetBufferDescriptor( bufferDescriptor * bd_p, unsigned char command,
                    unsigned char status, unsigned short count,
                    void * buffAddr, void * extBufferAddr)
{
  bd_p->mode.command = command;
  bd_p->mode.status = status;
  bd_p->mode.count = count;
  if (buffAddr != NULL) {
    bd_p->bufferAddr = iapi_Virt2Phys(buffAddr);
  } else {
    bd_p->bufferAddr = buffAddr;
  }
  bd_p->extBufferAddr = extBufferAddr;
}

