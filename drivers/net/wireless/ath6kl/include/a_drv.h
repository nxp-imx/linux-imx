//------------------------------------------------------------------------------
// <copyright file="a_drv.h" company="Atheros">
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
// This file contains the definitions of the basic atheros data types.
// It is used to map the data types in atheros files to a platform specific
// type.
//
// Author(s): ="Atheros"
//==============================================================================
#ifndef _A_DRV_H_
#define _A_DRV_H_

#if defined(__linux__) && !defined(LINUX_EMULATION)
#include "../os/linux/include/athdrv_linux.h"
#endif

#ifdef UNDER_NWIFI
#include "../os/windows/include/athdrv.h"
#endif

#ifdef ATHR_CE_LEGACY
#include "../os/windows/include/athdrv.h"
#endif

#ifdef REXOS
#include "../os/rexos/include/common/athdrv_rexos.h"
#endif

#ifdef WIN_NWF
#include "../os/windows/include/athdrv.h"
#endif

#endif /* _ADRV_H_ */
