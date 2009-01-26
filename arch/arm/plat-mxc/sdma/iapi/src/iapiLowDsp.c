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
 * File: iapiLowDsp.c
 *
 * $Id iapiLowDsp.c $
 *
 * Description:
 *  This library is written in C to guarantee functionality and integrity in
 * the usage of SDMA virtual DMA channels. This API (Application Programming
 * Interface)  allow SDMA channels' access in an OPEN, READ, WRITE, CLOSE
 * fashion.
 *  These are the LOW level functions of the I.API specific to MCU.
 *
 *
 *
 *
 * $Log iapiLowDsp.c $
 *
 *****************************************************************************/

/* ****************************************************************************
 * Include File Section
 *****************************************************************************/
#include "epm.h"
#include "iapiLow.h"

/* ****************************************************************************
 * Function Section
 *****************************************************************************/
#ifdef DSP

/* ***************************************************************************/
/**Starts the channel (core specific register)
 *
 * <b>Algorithm:</b>\n
 *   - Bit numbered "channel" of DspEnStartReg register is set
 *
 * @param channel channel to start
 *
 * @return none
 */
void
iapi_lowStartChannel (unsigned char channel)
{
  SDMA_D_START |= (1 << channel);
}

/* ***************************************************************************/
/**Stops the channel (core specific register)
 *
 * <b>Algorithm:</b>
 *   - Bit numbered "channel" of DspEnStopReg register is cleared
 *
 * <b>Notes:</b>\n
 *   - This is a write one to clear register
 *
 * @param channel channel to stop
 *
 * @return none
 */
void
iapi_lowStopChannel (unsigned char channel)
{
  SDMA_D_STATSTOP &= (1 << channel);
}

#endif /* DSP */
