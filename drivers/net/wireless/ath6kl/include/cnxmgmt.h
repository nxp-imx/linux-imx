//------------------------------------------------------------------------------
// <copyright file="cnxmgmt.h" company="Atheros">
//    Copyright (c) 2004-2007 Atheros Corporation.  All rights reserved.
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

#ifndef _CNXMGMT_H_
#define _CNXMGMT_H_

typedef enum {
    CM_CONNECT_WITHOUT_SCAN             = 0x0001,
    CM_CONNECT_ASSOC_POLICY_USER        = 0x0002,
    CM_CONNECT_SEND_REASSOC             = 0x0004,
    CM_CONNECT_WITHOUT_ROAMTABLE_UPDATE = 0x0008,
    CM_CONNECT_DO_WPA_OFFLOAD           = 0x0010,
    CM_CONNECT_DO_NOT_DEAUTH            = 0x0020,
} CM_CONNECT_TYPE;

#endif  /* _CNXMGMT_H_ */
