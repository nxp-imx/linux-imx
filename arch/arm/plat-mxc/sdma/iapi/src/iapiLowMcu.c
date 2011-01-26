/******************************************************************************
 *
 * Copyright 2007-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * File: iapiLowMcu.c
 *
 * $Id iapiLowMcu.c $
 *
 * Description:
 *  This library is written in C to guarantee functionality and integrity in
 * the usage of SDMA virtual DMA channels. This API (Application Programming
 * Interface)  allow SDMA channels' access in an OPEN, READ, WRITE, CLOSE
 * fashion.
 *  These are the LOW level functions of the I.API specific to MCU.
 *
 *
 *  http://compass/mot.com/go/115342679
 *
 * $Log iapiLowMcu.c $
 *
 *****************************************************************************/

/* ****************************************************************************
 * Include File Section
 *****************************************************************************/
#include <string.h>
#include <io.h>

#include "epm.h"
#include "iapiLow.h"

/* ****************************************************************************
 * Function Section
 *****************************************************************************/
#ifdef MCU

/* ***************************************************************************/
/**Send a command on SDMA's channel zero.
 * Check if buffer descriptor is already used by the sdma, if yes return
 * an error as c0BDNum is wrong.
 *
 * <b>Notes</b>\n
 *  There is an upgrade in the script on the Context load command and
 * the fact that the context structure has a fixed length of 20 or 24
 * depending on SDMA versions.
 *
 * @return
 *   - IAPI_SUCCESS
 *   - -iapi_errno if failure
 */
int
iapi_Channel0Command(channelDescriptor *cd_p, void *buf,
		     unsigned short nbyte, unsigned char command)
{
	channelControlBlock *ccb_p;
	bufferDescriptor *bd_p;
	int result = IAPI_SUCCESS;
	unsigned char chNum;

	/*
	 * Check data structures are properly initialized
	 */
	/* Channel descriptor validity */
	if (cd_p == NULL) {
		result = IAPI_ERR_CD_UNINITIALIZED;
		iapi_errno = result;
		return -result;
	}

	/* Channel control block validity */
	if (cd_p->ccb_ptr == NULL) {
		result = IAPI_ERR_CCB_UNINITIALIZED;
		iapi_errno = result;
		return -result;
	}

	/* Control block & Descriptpor associated with the channel being worked on */
	chNum = cd_p->channelNumber;
	ccb_p = cd_p->ccb_ptr;

	/* Is channel already in use ? */
	if (ccb_p->baseBDptr != NULL) {
		result = IAPI_ERR_BD_ALLOCATED | IAPI_ERR_CH_AVAILABLE | chNum;
		iapi_errno = result;
		return -result;
	}

	/* Allocation of buffer descriptors */
	bd_p = (bufferDescriptor *) MALLOC(sizeof(bufferDescriptor), SDMA_ERAM);
	if (bd_p != NULL) {
		ccb_p->baseBDptr = (bufferDescriptor *) iapi_Virt2Phys(bd_p);
	} else {
		result = IAPI_ERR_BD_ALLOCATION | IAPI_ERR_CH_AVAILABLE | chNum;
		iapi_errno = result;
		return -result;
	}

	/* Buffer descriptor setting */
	iapi_SetBufferDescriptor(bd_p, command, BD_WRAP | BD_DONE | BD_INTR,
				 nbyte, buf, NULL);

	/* Actually the transfer */
	iapi_lowStartChannel(cd_p->channelNumber);
	iapi_lowSynchChannel(cd_p->channelNumber);

	/* Cleaning of allocation */
	FREE(bd_p);
	ccb_p->baseBDptr = NULL;

	return IAPI_SUCCESS;

}

/* ***************************************************************************/
/**Starts the channel (core specific register)
 *
 * <b>Algorithm:</b>\n
 *   - Bit numbered "channel" of HostEnStartReg register is set
 *
 * @param channel channel to start
 *
 * @return none
 */
void iapi_lowStartChannel(unsigned char channel)
{
	/* HSTART is a 'write-ones' register */
	writel(1UL << channel, SDMA_H_START_ADDR);
}

/* ***************************************************************************/
/**Stops the channel (core specific register)
 *
 * <b>Algorithm:</b>
 *   - Bit numbered "channel" of HostEnStopReg register is cleared
 *
 * <b>Notes:</b>\n
 *   - This is a write one to clear register
 *
 * @param channel channel to stop
 *
 * @return none
 */
void iapi_lowStopChannel(unsigned char channel)
{
	/* Another 'write-ones' register */
	writel(1UL << channel, SDMA_H_STATSTOP_ADDR);
}

/* ***************************************************************************/
/**Initialize the initial priority of registers and channel enable
 * RAM from the MCU side. No channels are enabled, all priorities are set to 0.
 *
 * @return none
 */
void iapi_InitChannelTables(void)
{

	/* No channel is enabled */
	iapi_memset((void *)&SDMA_CHNENBL_0, 0x00,
		    sizeof(unsigned long) * EVENTS_NUM);
	/* All channels have priority 0 */
	iapi_memset((void *)&SDMA_CHNPRI_0, 0x00,
		    sizeof(unsigned long) * CH_NUM);
}

/* ***************************************************************************/
/** The host enable (HE), hosts override (HO), dsp enable (DE), dsp override
 * (DO) registers are involved here.
 * Host and Dsp enable registers are here to signify that the MCU or DSP side
 * have prepared the appropriate buffers and are now ready. If the channel is
 * owned by the MCU the override bit for that channel needs to be cleared :
 * the host allows the channel to be used.\n
 *
 * Then the override bits can define (mcuOverride dspOverride):\n
 *  - 0 0 channel is public: transfer to/from MCU to DSP
 *  - 0 1 channel if owned by DSP
 *  - 1 0 channel if owned by MCU
 *  - 1 1 channel zero config
 *
 * See also :\n
 *  IAPI Table 1.1 "Channel configuration properties"
 *
 * @param channel channel to configure
 * @param eventOverride event ownership
 * @param mcuOverride ARM ownership
 * @param dspOverride DSP ownership
 *
 * @return
 *   - -iapi_errno if the 3 override parameters are all set
 *   - IAPI_SUCCESS in other cases (valid cases)
 */
int
iapi_ChannelConfig(unsigned char channel, unsigned eventOverride,
		   unsigned mcuOverride, unsigned dspOverride)
{
	int result = IAPI_SUCCESS;

	if ((eventOverride == 1) && (mcuOverride == 1) && (dspOverride == 1)) {
		result = IAPI_ERR_CONFIG_OVERRIDE;
		iapi_errno = result;
		return -result;
	} else {
		/*
		 * DSP side
		 */
		if (dspOverride) {
			SDMA_H_DSPOVR &= ~(1 << channel);
		} else {
			SDMA_H_DSPOVR |= (1 << channel);
		}
		/*
		 * Event
		 */
		if (eventOverride) {
			SDMA_H_EVTOVR &= ~(1 << channel);
		} else {
			SDMA_H_EVTOVR |= (1 << channel);
		}
		/*
		 * MCU side
		 */
		if (mcuOverride) {
			SDMA_H_HOSTOVR &= ~(1 << channel);
		} else {
			SDMA_H_HOSTOVR |= (1 << channel);
		}
	}
	return IAPI_SUCCESS;
}

/* ***************************************************************************/
/**Load the context data of a channel from SDMA
 *
 * <b>Algorithm:</b>\n
 *    - Setup BD with appropiate parameters
 *    - Start channel
 *    - Poll for answer
 *
 * @param *cd_p  channel descriptor for channel 0
 * @param *buf pointer to receive context data
 * @param channel channel for which the context data is requested
 *
 * @return none
 */
void
iapi_lowGetContext(channelDescriptor *cd_p, void *buf, unsigned char channel)
{
	bufferDescriptor *bd_p;

	bd_p = (bufferDescriptor *) iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);

	/*Setup buffer descriptor with channel 0 command */
	iapi_SetBufferDescriptor(&bd_p[0],
				 C0_GETDM,
				 (unsigned char)(BD_DONE | BD_INTR | BD_WRAP |
						 BD_EXTD),
				 (unsigned short)sizeof(contextData) / 4, buf,
				 (void *)(CHANNEL_CONTEXT_BASE_ADDRESS +
					  (sizeof(contextData) * channel / 4)));
	/* Receive, polling method */
	iapi_lowStartChannel(cd_p->channelNumber);
	iapi_lowSynchChannel(cd_p->channelNumber);
}

/* ***************************************************************************/
/**Read "size" byte /2 at SDMA address (address) and write them in buf
 *
 * <b>Algorithm:</b>\n
 *    - Setup BD with appropiate parameters (C0_GETPM)
 *    - Start channel
 *    - Poll for answer
 *
 * <b>Notes</b>\n
 *   - Parameter "size" is in bytes, it represents the size of "buf", e.g.
 * the size in bytes of the script to be loaded.
 *   - Parameter "address" denotes the RAM address for the script in SDMA
 *
 * @param *cd_p  channel descriptor for channel 0
 * @param *buf pointer to receive the data
 * @param size number of bytes to read
 * @param address address in SDMA RAM to start reading from
 *
 * @return none
 */
void
iapi_lowGetScript(channelDescriptor *cd_p, void *buf, unsigned short size,
		  unsigned long address)
{
	bufferDescriptor *bd_p;

	bd_p = (bufferDescriptor *) iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);

	/*Setup buffer descriptor with channel 0 command */
	iapi_SetBufferDescriptor(&bd_p[0], C0_GETPM, (unsigned char)(BD_DONE | BD_INTR | BD_WRAP | BD_EXTD), (unsigned short)size / 2,	/*count in shorts */
				 buf, (void *)address);
	/* Receive, polling method */
	iapi_lowStartChannel(cd_p->channelNumber);
	iapi_lowSynchChannel(cd_p->channelNumber);
}

/* ***************************************************************************/
/**Load a SDMA script to SDMA
 *
 * <b>Algorithm:</b>\n
 *    - Setup BD with appropiate parameters (C0_SETPM)
 *    - Start channel
 *    - Poll for answer
 *
 * <b>Notes</b>\b
 *    - Parameter "size" is in bytes, it represents the size of "buf", e.g.
 *  the size in bytes of the script to be uploaded.
 *    - Parameter "address" denotes the RAM address for the script in SDMA
 *
 * @param *cd_p  channel descriptor for channel 0
 * @param *buf pointer to the script
 * @param size size of the script, in bytes
 * @param address address in SDMA RAM to place the script
 *
 * @return none
 */
void
iapi_lowSetScript(channelDescriptor *cd_p, void *buf, unsigned short size,
		  unsigned long address)
{
	bufferDescriptor *bd_p;

	bd_p = (bufferDescriptor *) iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);

	/*Setup buffer descriptor with channel 0 command */
	iapi_SetBufferDescriptor(&bd_p[0], C0_SETPM, (unsigned char)(BD_DONE | BD_INTR | BD_WRAP | BD_EXTD), (unsigned short)size / 2,	/*count in  shorts */
				 buf, (void *)(address));
	/* Receive, polling method */
	iapi_lowStartChannel(cd_p->channelNumber);
	iapi_lowSynchChannel(cd_p->channelNumber);
}

/* ***************************************************************************/
/**Load the context for a channel to SDMA
 *
 * <b>Algorithm:</b>\n
 *   - Send context and poll for answer.
 *
 * @param *cd_p  channel descriptor for channel 0
 * @param *buf pointer to context data
 * @param channel channel to place the context for
 *
 * @return none
 */
void
iapi_lowSetContext(channelDescriptor *cd_p, void *buf, unsigned char channel)
{

	bufferDescriptor *local_bd_p;
#ifdef SDMA_SKYE

	unsigned char command = 0;

	local_bd_p =
	    (bufferDescriptor *) iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);

	command = channel << 3;
	command = command | C0_SETCTX;
	iapi_SetBufferDescriptor(&local_bd_p[0],
				 command,
				 (unsigned char)(BD_DONE | BD_INTR | BD_WRAP),
				 (unsigned short)(sizeof(contextData) / 4),
				 buf, NULL);
#else

	local_bd_p =
	    (bufferDescriptor *) iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);

	iapi_SetBufferDescriptor(&local_bd_p[0],
				 C0_SETDM,
				 (unsigned char)(BD_DONE | BD_INTR | BD_WRAP |
						 BD_EXTD),
				 (unsigned short)(sizeof(contextData) / 4), buf,
				 (void *)(2048 +
					  (sizeof(contextData) / 4) * channel));
#endif
	/* Send */
	iapi_lowStartChannel(cd_p->channelNumber);
	iapi_lowSynchChannel(cd_p->channelNumber);

}

/* ***************************************************************************/
/**Associate specified channel with the script starting at the
 * specified address. Channel 0 command is used to load the set-up context
 * for the channel. The address used must be generated by the GUI tool
 * used to create RAM images for SDMA.
 *
 * <b>Algorithm:</b>\n
 *    - Set-up and load the context.
 *
 *  @param *cd_p  pointer to the channel descriptor of the channel
 *  @param *data_p: pointer to the data identifying the script to be associated
 *          with the channel
 *
 * @return
 *     - IAPI_SUCCESS : OK
 *     - -iapi_errno : operation failed, return negated value of iapi_errno
 */

int iapi_lowAssignScript(channelDescriptor *cd_p, script_data * data_p)
{
	contextData *chContext;	/* context to be loaded for the channel */
	channelDescriptor *cd0_p;	/* pointer to channel descriptor of channel 0 */
	int result = IAPI_SUCCESS;

	/*Verify passed data */
	if (cd_p == NULL || data_p == NULL) {
		result = IAPI_ERR_INVALID_PARAMETER;
		iapi_errno = result;
		return -result;
	}

	/* Allocate context and initialize PC to required script start adress */
	chContext = (contextData *) MALLOC(sizeof(contextData), SDMA_ERAM);
	if (chContext == NULL) {
		result = IAPI_ERR_B_ALLOC_FAILED | cd_p->channelNumber;
		iapi_errno = result;
		return -result;
	}

	iapi_memset(chContext, 0x00, sizeof(contextData));
	chContext->channelState.pc = data_p->load_address;

	/* Send by context the event mask,base address for peripheral
	 * and watermark level
	 */
	chContext->gReg[0] = data_p->event_mask2;
	chContext->gReg[1] = data_p->event_mask1;
	chContext->gReg[6] = data_p->shp_addr;
	chContext->gReg[7] = data_p->wml;
	if (data_p->per_addr)
		chContext->gReg[2] = data_p->per_addr;

	/* Set transmited data to the CD */
	cd_p->watermarkLevel = data_p->wml;
	cd_p->eventMask1 = data_p->event_mask1;
	cd_p->eventMask2 = data_p->event_mask2;

	/* Get the cd0_p */
	cd0_p = (cd_p->ccb_ptr - cd_p->channelNumber)->channelDescriptor;

	/*load the context */
	iapi_lowSetContext(cd0_p, chContext, cd_p->channelNumber);

	/* release allocated memory */
	FREE(chContext);

	return IAPI_SUCCESS;
}

/* ***************************************************************************/
/**  Set the channels to be triggered by an event. The for every channel that
 *must be triggered by the event, the corresponding bit from channel_map
 *parameter must be set to 1. (e.g. for the event to trigger channels 31 and
 *0 one must pass 0x80000001)
 *
 *
 * <b>Algorithm:</b>\n
 *   - Update the register from Channel Enable RAM with the channel_map
 *
 * @param event event for which to set the channel association
 * @param channel_map channels to be triggered by event. Put the corresponding
 *          bit from this 32-bit value to 1 for every channel that should be
 *          triggered by the event.
 *
 * @return
 *     - IAPI_SUCCESS : OK
 *     - -iapi_errno : operation failed, return negated value of iapi_errno
 */
int
iapi_lowSetChannelEventMapping(unsigned char event, unsigned long channel_map)
{
	volatile unsigned long *channelEnableMatx;
	int result = IAPI_SUCCESS;

	/* Check validity of event */
	if (event < EVENTS_NUM) {
		channelEnableMatx = &SDMA_CHNENBL_0;
		channelEnableMatx[event] |= channel_map;
		return result;
	} else {
		result = IAPI_ERR_INVALID_PARAMETER | event;
		iapi_errno = result;
		return -result;
	}
}

#endif				/* MCU */
