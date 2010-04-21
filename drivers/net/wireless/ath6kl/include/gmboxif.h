//------------------------------------------------------------------------------
// <copyright file="gmboxif.h" company="Atheros">
//    Copyright (c) 2009 Atheros Corporation.  All rights reserved.
// 
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
//------------------------------------------------------------------------------
//==============================================================================
// Author(s): ="Atheros"
//==============================================================================

#ifndef __GMBOXIF_H__
#define __GMBOXIF_H__

#ifndef ATH_TARGET
#include "athstartpack.h"
#endif

/* GMBOX interface definitions */
    
#define AR6K_GMBOX_CREDIT_COUNTER       1   /* we use credit counter 1 to track credits */
#define AR6K_GMBOX_CREDIT_SIZE_COUNTER  2   /* credit counter 2 is used to pass the size of each credit */


    /* HCI UART transport definitions when used over GMBOX interface */
#define HCI_UART_COMMAND_PKT 0x01
#define HCI_UART_ACL_PKT     0x02
#define HCI_UART_SCO_PKT     0x03
#define HCI_UART_EVENT_PKT   0x04

    /* definitions for BT HCI packets */
typedef PREPACK struct {
    A_UINT16 Flags_ConnHandle;
    A_UINT16 Length;
} POSTPACK BT_HCI_ACL_HEADER;

typedef PREPACK struct {
    A_UINT16 Flags_ConnHandle;
    A_UINT8  Length;
} POSTPACK BT_HCI_SCO_HEADER;

typedef PREPACK struct {
    A_UINT16 OpCode;
    A_UINT8  ParamLength;
} POSTPACK BT_HCI_COMMAND_HEADER;

typedef PREPACK struct {
    A_UINT8  EventCode;
    A_UINT8  ParamLength;
} POSTPACK BT_HCI_EVENT_HEADER;

/* MBOX host interrupt signal assignments */

#define MBOX_SIG_HCI_BRIDGE_MAX      8
#define MBOX_SIG_HCI_BRIDGE_BT_ON    0
#define MBOX_SIG_HCI_BRIDGE_BT_OFF   1
#define MBOX_SIG_HCI_BRIDGE_BAUD_SET 2


#ifndef ATH_TARGET
#include "athendpack.h"
#endif

#endif /* __GMBOXIF_H__ */

