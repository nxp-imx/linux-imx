/*
 * Copyright (c) 2004-2008 Atheros Communications Inc.
 * All rights reserved.
 *
 * This file is the include file for Atheros PS and patch parser.
 * It implements APIs to parse data buffer with patch and PS information and convert it to HCI commands.
 *
 *
 *
 * ar3kpsparser.h
 *
 *
 *
 * The software source and binaries included in this development package are
 * licensed, not sold. You, or your company, received the package under one
 * or more license agreements. The rights granted to you are specifically
 * listed in these license agreement(s). All other rights remain with Atheros
 * Communications, Inc., its subsidiaries, or the respective owner including
 * those listed on the included copyright notices..  Distribution of any
 * portion of this package must be in strict compliance with the license
 * agreement(s) terms.
 *
 *
 *
 */

/*------------------------------------------------------------------------------
 *
 * <copyright file="File name" company="Atheros">
 * Copyright (c) 2004-2008 Atheros Corporation.  All rights reserved.
 * 
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation;
//
// Software distributed under the License is distributed on an "AS
// IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
// implied. See the License for the specific language governing
// rights and limitations under the License.
//
//
 *------------------------------------------------------------------------------
 *
 *
 *
 * This file is the include file for Atheros PS and patch parser.
 * It implements APIs to parse data buffer with patch and PS information and convert it to HCI commands.
 *
 *
 *
 *
 *
 */
#ifndef __AR3KPSPARSER_H
#define __AR3KPSPARSER_H




#include <linux/fs.h>
#include <linux/slab.h>
#include "athdefs.h"
#ifdef HCI_TRANSPORT_SDIO
#include "a_config.h"
#include "a_types.h"
#include "a_osapi.h"
#define ATH_MODULE_NAME misc
#include "a_debug.h"
#include "common_drv.h"
#include "hci_transport_api.h"
#include "ar3kconfig.h"
#else
#ifndef A_PRINTF
#define A_PRINTF(args...)     printk(KERN_ALERT args)
#endif /* A_PRINTF */
#include "debug_linux.h"

/* Helper data type declaration */

#ifndef A_UINT32
#define A_UCHAR                 unsigned char
#define A_UINT32                unsigned long
#define A_UINT16                unsigned short
#define A_UINT8                 unsigned char
#define A_BOOL                  unsigned char
#endif /* A_UINT32 */

#define ATH_DEBUG_ERR          (1 << 0)
#define ATH_DEBUG_WARN         (1 << 1)
#define ATH_DEBUG_INFO         (1 << 2)



#define FALSE   0
#define TRUE    1

#ifndef A_MALLOC
#define A_MALLOC(size)  kmalloc((size),GFP_KERNEL)
#endif /* A_MALLOC */


#ifndef A_FREE
#define A_FREE(addr)  kfree((addr))
#endif /* A_MALLOC */
#endif /* HCI_TRANSPORT_UART */

/* String manipulation APIs */
#ifndef A_STRTOUL
#define A_STRTOUL               simple_strtoul
#endif  /* A_STRTOL */

#ifndef A_STRTOL 
#define A_STRTOL                simple_strtol
#endif /* A_STRTOL */


/* The maximum number of bytes possible in a patch entry */
#define MAX_PATCH_SIZE                    20000

/* Maximum HCI packets that will be formed from the Patch file */
#define MAX_NUM_PATCH_ENTRY               (MAX_PATCH_SIZE/MAX_BYTE_LENGTH) + 1







typedef struct PSCmdPacket
{
    A_UCHAR *Hcipacket;
    int packetLen;
} PSCmdPacket;

/* Parses a Patch information buffer and store it in global structure */
A_STATUS AthDoParsePatch(A_UCHAR *, A_UINT32);

/* parses a PS information buffer and stores it in a global structure */
A_STATUS AthDoParsePS(A_UCHAR *, A_UINT32);

/* 
 *  Uses the output of Both AthDoParsePS and AthDoParsePatch APIs to form HCI command array with
 *  all the PS and patch commands.
 *  The list will have the below mentioned commands in order.
 *  CRC command packet
 *  Download patch command(s)
 *  Enable patch Command
 *  PS Reset Command
 *  PS Tag Command(s)
 *
 */  
int AthCreateCommandList(PSCmdPacket **, A_UINT32 *);

/* Cleanup the dynamically allicated HCI command list */
A_STATUS AthFreeCommandList(PSCmdPacket **HciPacketList, A_UINT32 numPackets);
#endif /* __AR3KPSPARSER_H */
