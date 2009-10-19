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
 * File: iapiMiddleMcu.c
 *
 * $Id iapiMiddleMcu.c $
 *
 * Description:
 *  This library is written in C to guarantee functionality and integrity in
 * the usage of SDMA virtual DMA channels. This API (Application Programming
 * Interface)  allow SDMA channels' access in an OPEN, READ, WRITE, CLOSE
 * fashion.
 *  These are the MIDDLE level functions of the I.API specific to MCU.
 *
 *
 *
 *
 * $Log iapiMiddleMcu.c $
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

/*extern void * __HEAP_START;
extern void * __HEAP_END;
*/

/* ****************************************************************************
 * Function Section
 *****************************************************************************/
