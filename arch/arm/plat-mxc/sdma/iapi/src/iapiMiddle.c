/******************************************************************************
 *
 * Copyright 2007 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * File: iapiMiddle.c
 *
 * $Id iapiMiddle.c $
 *
 * Description:
 *  This library is written in C to guarantee functionality and integrity in
 * the usage of SDMA virtual DMA channels. This API (Application Programming
 * Interface)  allow SDMA channels' access in an OPEN, READ, WRITE, CLOSE
 * fashion.
 *  These are the MIDDLE level functions of the I.API.
 *
 *
 *
 *
 * $Log iapiMiddle.c $
 *
 *****************************************************************************/


/* ****************************************************************************
 * Include File Section
 *****************************************************************************/
#include "epm.h"
#include <string.h>

#include "iapiLow.h"
#include "iapiMiddle.h"

/* ****************************************************************************
 * Global Variable Section
 *****************************************************************************/


/* ****************************************************************************
 * Function Section
 *****************************************************************************/

/* ***************************************************************************/
/**Allocates one Buffer Descriptor structure using information present in the
 * channel descriptor.
 *
 * @param *ccb_p channel control block used to get the channel descriptor
 *
 * @return
 *       - pointer on the new Buffer Descriptor
 *       - NULL if allocation failed
 *
 */
bufferDescriptor *
iapi_AllocBD (channelControlBlock * ccb_p)

{
  bufferDescriptor * ptrBD = NULL;

  if (ccb_p->channelDescriptor->bufferDescNumber != 0){
#ifdef CONFIG_SDMA_IRAM
     channelDescriptor * cd_p = ccb_p->channelDescriptor;
     if(cd_p->channelNumber >= MXC_DMA_CHANNEL_IRAM) {
         ptrBD = (bufferDescriptor *)
           MALLOC( ccb_p->channelDescriptor->bufferDescNumber *
               sizeof(bufferDescriptor), SDMA_IRAM);
     } else
#endif /*CONFIG_SDMA_IRAM*/
     {
     	ptrBD = (bufferDescriptor *)
       		MALLOC( ccb_p->channelDescriptor->bufferDescNumber * 
           		sizeof(bufferDescriptor), SDMA_ERAM);   
     }
  }
  if (ptrBD != NULL) {
   ptrBD->mode.command = 0;
   ptrBD->mode.status = 0;
   ptrBD->mode.count = 0;
   ptrBD->bufferAddr = NULL;
  }

  return ptrBD;
}

/* ***************************************************************************/
/**Allocate one channel context data structure.
 *
 * @param **ctxd_p  pointer to context data to be allocated
 * @param channel channel number of context data structure
 *
 * @return
 *     - IAPI_SUCCESS
 *     - -iapi_errno if allocation failed
 */
int
iapi_AllocContext(contextData ** ctxd_p, unsigned char channel)
{
  contextData * ctxData;
  int result = IAPI_SUCCESS;

  if (*ctxd_p != NULL){
   result = IAPI_ERR_CC_ALREADY_DEFINED | IAPI_ERR_CH_AVAILABLE | channel;
   iapi_errno = result;
   return -result;
  }

  ctxData = (contextData *)MALLOC(sizeof(contextData), SDMA_ERAM);

  if (ctxData !=NULL) {
   *ctxd_p = ctxData;
   return IAPI_SUCCESS;

  } else {
   *ctxd_p = NULL;
   result = IAPI_ERR_CC_ALLOC_FAILED | IAPI_ERR_CH_AVAILABLE | channel;
   iapi_errno = result;
   return -result;
  }
}

/* ***************************************************************************/
/**Allocates channel description and fill in with default values.
 *
 * <b>Algorithm:</b>\n
 *  - Check channel properties.
 *  - Then modifies the properties of the channel description with default
 *
 * @param **cd_p  pointer to channel descriptor to be allocated
 * @param channel channel number of channel descriptor
 *
 * @return
 *     - IAPI_SUCCESS
 *     - -iapi_errno if allocation failed
 *
 */
int
iapi_AllocChannelDesc (channelDescriptor ** cd_p, unsigned char channel)
{
#ifdef MCU
  volatile unsigned long * chPriorities = &SDMA_CHNPRI_0;
#endif /* MCU */
  channelDescriptor * tmpCDptr;
  int result = IAPI_SUCCESS;


  if (*cd_p != NULL){
   result = IAPI_ERR_CD_ALREADY_DEFINED | IAPI_ERR_CH_AVAILABLE | channel;
   iapi_errno = result;
   return -result;
  }

  tmpCDptr = (channelDescriptor *)MALLOC(sizeof(channelDescriptor), SDMA_ERAM);

  if (tmpCDptr != NULL){
   iapi_memcpy(tmpCDptr, &iapi_ChannelDefaults, sizeof (channelDescriptor));
   tmpCDptr->channelNumber = channel;
#ifdef MCU
   if (chPriorities[channel] != 0) {
      tmpCDptr->priority = chPriorities[channel];
   } else {
      chPriorities[channel] = tmpCDptr->priority;
   }
#endif
   * cd_p = tmpCDptr ;
   return IAPI_SUCCESS;
  } else {
   * cd_p = NULL;
   result = IAPI_ERR_CD_ALLOC_FAILED | IAPI_ERR_CH_AVAILABLE | channel;
   iapi_errno = result;
   return -result;
  }
}

/* ***************************************************************************/
/**Changes channel description information after performing sanity checks.
 *
 * <b>Algorithm:</b>\n
 *    - Check channel properties.
 *    - Then modifies the properties of the channel description.
 *
 * @param *cd_p channel descriptor of the channel to change
 * @param whatToChange control code indicating the desired change
 * @param newval new value
 *
 * @return
 *     - IAPI_SUCCESS
 *     - IAPI_FAILURE if change failed
 *
 */
int
iapi_ChangeChannelDesc (channelDescriptor * cd_p, unsigned char whatToChange,
                  unsigned long newval)
{
  bufferDescriptor * tmpBDptr;
  unsigned char index = 0;
  int result = IAPI_SUCCESS;

  /* verify parameter validity */
  if (cd_p == NULL) {
   result = IAPI_ERR_CD_UNINITIALIZED;
   iapi_errno = result;
   return -result;
  }

  /* verify channel descriptor initialization */
  if (cd_p->ccb_ptr == NULL){
   result = IAPI_ERR_CCB_UNINITIALIZED | IAPI_ERR_CH_AVAILABLE |
     cd_p->channelNumber;
   iapi_errno = result;
   return -result;
  }

  /* verify channel is not in use */
  tmpBDptr = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
  for (index=cd_p->bufferDescNumber ; index>0 ; index--){
   if (tmpBDptr->mode.status & BD_DONE){
     result = IAPI_ERR_CH_IN_USE | IAPI_ERR_CH_AVAILABLE |
      cd_p->channelNumber;
     iapi_errno = result;
     return -result;
   }
   tmpBDptr++;
  }

  /* Select the change accorded to the selector given in parameter */
  switch (whatToChange){

   /*
    * Channel Number
    */
  case IAPI_CHANNELNUMBER:
   /* Channel number can not be changed (description remains attached) */
   result = IAPI_ERR_CD_CHANGE_CH_NUMBER | IAPI_ERR_CH_AVAILABLE |
     cd_p->channelNumber;
   iapi_errno = result;
   return -result;

   /*
    * Buffer Descriptor Number
    */
  case IAPI_BUFFERDESCNUMBER:
   if (newval < MAX_BD_NUM){
     if (newval != cd_p->bufferDescNumber){
      /* Free memory used for previous old data */
      if (cd_p->ccb_ptr->baseBDptr != NULL){
        tmpBDptr = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
        for (index=0 ; index < cd_p->bufferDescNumber ; index++){
         if (tmpBDptr->bufferAddr != NULL){
           if (cd_p->trust == FALSE) {
            FREE(iapi_Phys2Virt(tmpBDptr->bufferAddr));
           }
         }
         tmpBDptr ++ ;
        }
        FREE((bufferDescriptor *)iapi_Phys2Virt((cd_p->ccb_ptr)->baseBDptr));
      }
      (cd_p->ccb_ptr)->baseBDptr = NULL;
      (cd_p->ccb_ptr)->currentBDptr = NULL;
      /* Allocate and initialize structures */
      cd_p->bufferDescNumber = (unsigned char)newval;
      cd_p->ccb_ptr->status.openedInit = FALSE;
	  if (IAPI_SUCCESS !=iapi_InitializeMemory (cd_p->ccb_ptr))
	  {
		  result = IAPI_ERR_BD_ALLOCATION | cd_p->channelNumber;
		  iapi_errno = result;
		  return -result;
	  }
      cd_p->ccb_ptr->status.openedInit = TRUE;
     }
     break;
   } else {
     result = IAPI_ERR_INVALID_PARAMETER | IAPI_ERR_CH_AVAILABLE |
      cd_p->channelNumber;
     iapi_errno = result;
     return -result;
   }

   /*
    * Buffer size
    */
  case IAPI_BUFFERSIZE:
   if (newval < MAX_BD_SIZE)
   {
     if (newval != cd_p->bufferSize)
     {
       /* Free memory used for previous old data */
       if (cd_p->ccb_ptr->baseBDptr != NULL)
       {
         tmpBDptr = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
         for (index=0 ; index < cd_p->bufferDescNumber ; index++)
         {
           if (cd_p->trust == FALSE)
           {
             FREE(iapi_Phys2Virt(tmpBDptr->bufferAddr));
           }
           tmpBDptr ++ ;
         }
         FREE((bufferDescriptor *)iapi_Phys2Virt((cd_p->ccb_ptr)->baseBDptr));
       }
       (cd_p->ccb_ptr)->baseBDptr = NULL;
       (cd_p->ccb_ptr)->currentBDptr = NULL;
       /* Allocate and initialize structures */
       cd_p->bufferSize = (unsigned short)newval;
       cd_p->ccb_ptr->status.openedInit = FALSE;
	   if (IAPI_SUCCESS !=iapi_InitializeMemory (cd_p->ccb_ptr))
	   {
		   result = IAPI_ERR_BD_ALLOCATION | cd_p->channelNumber;
		   iapi_errno = result;
		   return -result;
	   }
       cd_p->ccb_ptr->status.openedInit = TRUE;
     }
     break;
   }
   else
   {
     result = IAPI_ERR_INVALID_PARAMETER | IAPI_ERR_CH_AVAILABLE |
      cd_p->channelNumber;
     iapi_errno = result;
     return -result;
   }


   /*
    * Blocking / non blocking feature
    */
  case IAPI_BLOCKING:
   if (newval < MAX_BLOCKING){
     cd_p->blocking = newval;
     break;
   } else {
     result = IAPI_ERR_INVALID_PARAMETER | IAPI_ERR_CH_AVAILABLE |
      cd_p->channelNumber;
     iapi_errno = result;
     return -result;
   }

   /*
    * Synchronization method
    */
  case IAPI_CALLBACKSYNCH:
   if (newval < MAX_SYNCH){
     cd_p->callbackSynch = newval;
     iapi_ChangeCallbackISR( cd_p, cd_p->callbackISR_ptr);
     break;
   } else {
     result = IAPI_ERR_INVALID_PARAMETER | IAPI_ERR_CH_AVAILABLE |
      cd_p->channelNumber;
     iapi_errno = result;
     return -result;
   }


   /*
    *  Ownership of the channel
    */
  case IAPI_OWNERSHIP:
#ifdef DSP
   result = IAPI_ERR_NOT_ALLOWED | cd_p->channelNumber;
   iapi_errno = result;
   return -result;
#endif /* DSP */
#ifdef MCU
   if (newval < MAX_OWNERSHIP){
     cd_p->ownership = newval;
     iapi_ChannelConfig( cd_p->channelNumber,
                    ( newval >> CH_OWNSHP_OFFSET_EVT ) & 1,
                    ( newval >> CH_OWNSHP_OFFSET_MCU ) & 1,
                    ( newval >> CH_OWNSHP_OFFSET_DSP ) & 1);
     break;
   } else {
     result = IAPI_ERR_INVALID_PARAMETER | IAPI_ERR_CH_AVAILABLE |
      cd_p->channelNumber;
     iapi_errno = result;
     return -result;
   }

#endif /* MCU */

   /*
    * Priority
    */
  case IAPI_PRIORITY:
#ifdef DSP
   result = IAPI_ERR_NOT_ALLOWED | cd_p->channelNumber;
   iapi_errno = result;
   return -result;
#endif /* DSP */

#ifdef MCU
   if (newval < MAX_CH_PRIORITY){
     volatile unsigned long * ChannelPriorities = &SDMA_CHNPRI_0;
     ChannelPriorities[ cd_p->channelNumber ] = newval;
     break;
   } else {
     result = IAPI_ERR_INVALID_PARAMETER | IAPI_ERR_CH_AVAILABLE |
      cd_p->channelNumber;
     iapi_errno = result;
     return -result;
   }
#endif /* MCU */


   /*
    * "Trust" property
    */
  case IAPI_TRUST:
   if (newval < MAX_TRUST){
     if (cd_p->trust != newval){
      cd_p->trust = newval;
      if (newval == FALSE) {
		if (IAPI_SUCCESS !=iapi_InitializeMemory (cd_p->ccb_ptr))
		{
			result = IAPI_ERR_BD_ALLOCATION | cd_p->channelNumber;
			iapi_errno = result;
			return -result;
		}
      }
     }
     break;
   } else {
     result = IAPI_ERR_INVALID_PARAMETER | IAPI_ERR_CH_AVAILABLE |
      cd_p->channelNumber;
     iapi_errno = result;
     return -result;
   }

   /*
    * Callback function pointer
    */
  case IAPI_CALLBACKISR_PTR:
   if ( (void *)newval != NULL){
     {
       union {
         void * voidstar;
         void (* funcptr)(channelDescriptor * cd_p, void * arg);
       } value;
       value.voidstar = (void*) newval;
       cd_p->callbackISR_ptr = value.funcptr;
     }
     iapi_ChangeCallbackISR( cd_p, cd_p->callbackISR_ptr);
     break;
   } else {
     result = IAPI_ERR_INVALID_PARAMETER | IAPI_ERR_CH_AVAILABLE |
      cd_p->channelNumber;
     iapi_errno = result;
     return -result;
   }


   /*
    * Channel Control Block pointer
    */
  case IAPI_CCB_PTR:
   cd_p->ccb_ptr = (channelControlBlock *)newval;
   cd_p->ccb_ptr->channelDescriptor = cd_p;
   break;

   /*
    * WRAP/UNWRAP
    */
  case IAPI_BDWRAP:
     /* point to first BD */
     tmpBDptr = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
     /* to point to last BD */
     tmpBDptr += cd_p->bufferDescNumber - 1;
    if (newval == TRUE){
     /* wrap last BD */
      tmpBDptr->mode.status |= BD_WRAP;
      break;
   }
   else if (newval == FALSE){
     /* unwrap last BD */
      tmpBDptr->mode.status &= ~BD_WRAP;
      break;
   }
    else {
      result = IAPI_ERR_INVALID_PARAMETER | IAPI_ERR_CH_AVAILABLE |
                 cd_p->channelNumber;
      iapi_errno = result;
      return -result;
   }


   /*
    * Watermark level
    */
  case IAPI_WML:
#ifdef DSP
   result = IAPI_ERR_NOT_ALLOWED | cd_p->channelNumber;
   iapi_errno = result;
   return -result;
#endif /* DSP */
#ifdef MCU
    if (newval < MAX_WML){
     if (cd_p->watermarkLevel != newval){
       cd_p->watermarkLevel = newval;
     }
   break;
   }
   else {
     result = IAPI_ERR_INVALID_PARAMETER | IAPI_ERR_CH_AVAILABLE |
      cd_p->channelNumber;
     iapi_errno = result;
     return -result;
   }
#endif /* MCU */

   /*
    * Detect errors
    */
  default:
   result = IAPI_ERR_CD_CHANGE_UNKNOWN | IAPI_ERR_CH_AVAILABLE |
     cd_p->channelNumber;
   iapi_errno = result;
   return -result;
  }

  return IAPI_SUCCESS;
}


/* ***************************************************************************/
/**Initialize a table of function pointers that contain the interrupt Service
 * Routine callback pointers for the SDMA channels with a default value
 *
 * <b>Algorithm:</b>\n
 *    - Loop on each element of the global IAPI variable callbackIsrTable
 *
 * @param *func_p default callback functon for all SDMA channels
 *
 * @return none
 */
void
iapi_InitializeCallbackISR(void(* func_p)(channelDescriptor * cd_p, void * arg))
{
  unsigned long chCnt;

  for (chCnt = 0 ; chCnt < CH_NUM ; chCnt++){
    callbackIsrTable[chCnt] = func_p;
  }
}

/* ***************************************************************************/
/**For the specified  channel control block, attach the array of buffer
 * descriptors, the channel description structure and initialize channel's
 * status using information in the channel descriptor.
 *
 * @param *ccb_p pointer to channel control block
 *
 * @return none
 *
 */
int
iapi_InitializeMemory (channelControlBlock * ccb_p)
{
  bufferDescriptor * bd_p;
  unsigned char index;
  int result = IAPI_SUCCESS;

  /* Attach the array of Buffer descriptors */
  bd_p = iapi_AllocBD( ccb_p );
  if (bd_p != NULL)
  {
  ccb_p->baseBDptr = (bufferDescriptor *)iapi_Virt2Phys(bd_p);
  ccb_p->currentBDptr = ccb_p->baseBDptr;
   for(index=0 ;index < ccb_p->channelDescriptor->bufferDescNumber-1 ; index++)
   {
     if (ccb_p->channelDescriptor->trust == TRUE)
     {
       iapi_SetBufferDescriptor(bd_p,
                    (unsigned char)ccb_p->channelDescriptor->dataSize,
                    BD_CONT|BD_EXTD, ccb_p->channelDescriptor->bufferSize,
                    NULL, NULL);
     }
     else
     {
      if (ccb_p->channelDescriptor->bufferSize != 0)
      {
        iapi_SetBufferDescriptor(bd_p,
                      (unsigned char)ccb_p->channelDescriptor->dataSize,
                      BD_CONT|BD_EXTD, ccb_p->channelDescriptor->bufferSize,
                      MALLOC(ccb_p->channelDescriptor->bufferSize, SDMA_ERAM), NULL);
      }
     }
     bd_p++;
   }

   if (ccb_p->channelDescriptor->trust == TRUE)
   {
     iapi_SetBufferDescriptor(bd_p,
                   (unsigned char)ccb_p->channelDescriptor->dataSize,
                   BD_EXTD|BD_WRAP|BD_INTR, ccb_p->channelDescriptor->bufferSize,
                   NULL, NULL);
   }
   else
   {
     if (ccb_p->channelDescriptor->bufferSize != 0)
     {
      iapi_SetBufferDescriptor(bd_p,
                    (unsigned char)ccb_p->channelDescriptor->dataSize,
                    BD_EXTD|BD_WRAP|BD_INTR,
                    ccb_p->channelDescriptor->bufferSize,
                    MALLOC(ccb_p->channelDescriptor->bufferSize, SDMA_ERAM), NULL);
     }
   }
  }
  else
  {
	  result = IAPI_ERR_BD_ALLOCATION;
	  return -result;
  }
  return result;
}
