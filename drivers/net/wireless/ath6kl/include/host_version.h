//------------------------------------------------------------------------------
// <copyright file="host_version.h" company="Atheros">
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
// This file contains version information for the sample host driver for the
// AR6000 chip
//
// Author(s): ="Atheros"
//==============================================================================
#ifndef _HOST_VERSION_H_
#define _HOST_VERSION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <AR6002/AR6K_version.h>

/*
 * The version number is made up of major, minor, patch and build
 * numbers. These are 16 bit numbers.  The build and release script will
 * set the build number using a Perforce counter.  Here the build number is
 * set to 9999 so that builds done without the build-release script are easily
 * identifiable.
 */

#define ATH_SW_VER_MAJOR      __VER_MAJOR_
#define ATH_SW_VER_MINOR      __VER_MINOR_
#define ATH_SW_VER_PATCH      __VER_PATCH_
#define ATH_SW_VER_BUILD      __BUILD_NUMBER_ 

#ifdef __cplusplus
}
#endif

#endif /* _HOST_VERSION_H_ */
