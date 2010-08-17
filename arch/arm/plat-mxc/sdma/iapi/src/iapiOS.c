/******************************************************************************
 *
 * Copyright 2007-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * File: iapiOS.c
 *
 * $Id iapiOS.c $
 *
 * Description:
 *  This library is written in C to guarantee functionality and integrity in
 * the usage of SDMA virtual DMA channels. This API (Application Programming
 * Interface)  allow SDMA channels' access in an OPEN, READ, WRITE, CLOSE
 * fashion.
 *  These are the OS level functions of the I.API - are OS dependant and must
 * be provided by the user of I.API.
 *
 *
 * /
 *
 * $Log iapiOS.c $
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
#ifdef CONFIG_SDMA_IRAM
void *(*iapi_iram_Malloc) (size_t size);
#endif				/*CONFIG_SDMA_IRAM */

void *(*iapi_Malloc) (size_t size);
void (*iapi_Free) (void *ptr);

void *(*iapi_Virt2Phys) (void *ptr);
void *(*iapi_Phys2Virt) (void *ptr);

void (*iapi_WakeUp) (int);
void (*iapi_GotoSleep) (int);
void (*iapi_InitSleep) (int);

void *(*iapi_memcpy) (void *dest, const void *src, size_t count);
void *(*iapi_memset) (void *dest, int c, size_t count);

void (*iapi_EnableInterrupts) (void);
void (*iapi_DisableInterrupts) (void);

int (*iapi_GetChannel) (int);
int (*iapi_ReleaseChannel) (int);
