//------------------------------------------------------------------------------
// <copyright file="wlan_dset.h" company="Atheros">
//    Copyright (c) 2007 Atheros Corporation.  All rights reserved.
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

#ifndef __WLAN_DSET_H__
#define __WLAN_DSET_H__

typedef PREPACK struct wow_config_dset {

    A_UINT8 valid_dset;
    A_UINT8 gpio_enable;
    A_UINT16 gpio_pin;
} POSTPACK WOW_CONFIG_DSET;

#endif
