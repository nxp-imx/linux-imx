//------------------------------------------------------------------------------
// <copyright file="miscdrv.h" company="Atheros">
//    Copyright (c) 2004-2008 Atheros Corporation.  All rights reserved.
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
#ifndef _MISCDRV_H
#define _MISCDRV_H


#define HOST_INTEREST_ITEM_ADDRESS(target, item)    \
(((target) == TARGET_TYPE_AR6001) ?     \
   AR6001_HOST_INTEREST_ITEM_ADDRESS(item) :    \
   AR6002_HOST_INTEREST_ITEM_ADDRESS(item))

A_UINT32 ar6kRev2Array[][128]   = {
                                    {0xFFFF, 0xFFFF},      // No Patches
                               };

#define CFG_REV2_ITEMS                0     // no patches so far
#define AR6K_RESET_ADDR               0x4000
#define AR6K_RESET_VAL                0x100

#define EEPROM_SZ                     768
#define EEPROM_WAIT_LIMIT             4

#endif

