//------------------------------------------------------------------------------

// <copyright file="ar6kap_common.h" company="Atheros">
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

// This file contains the definitions of common AP mode data structures.
//
// Author(s): ="Atheros"
//==============================================================================

#ifndef _AR6KAP_COMMON_H_
#define _AR6KAP_COMMON_H_
/*
 * Used with AR6000_XIOCTL_AP_GET_STA_LIST
 */
typedef struct {
    A_UINT8     mac[ATH_MAC_LEN];
    A_UINT8     aid;
    A_UINT8     keymgmt;
    A_UINT8     ucipher;
    A_UINT8     auth;
} station_t;
typedef struct {
    station_t sta[AP_MAX_NUM_STA];
} ap_get_sta_t;
#endif /* _AR6KAP_COMMON_H_ */
