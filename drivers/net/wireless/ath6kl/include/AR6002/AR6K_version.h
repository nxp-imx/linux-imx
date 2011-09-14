//------------------------------------------------------------------------------
// <copyright file="AR6K_version.h" company="Atheros">
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

#define __VER_MAJOR_ 3
#define __VER_MINOR_ 0 
#define __VER_PATCH_ 0

/* The makear6ksdk script (used for release builds) modifies the following line. */
#define __BUILD_NUMBER_ 1057


/* Format of the version number. */
#define VER_MAJOR_BIT_OFFSET        28
#define VER_MINOR_BIT_OFFSET        24
#define VER_PATCH_BIT_OFFSET        16
#define VER_BUILD_NUM_BIT_OFFSET    0


/*
 * The version has the following format:
 * Bits 28-31: Major version
 * Bits 24-27: Minor version
 * Bits 16-23: Patch version
 * Bits 0-15:  Build number (automatically generated during build process )
 * E.g. Build 1.1.3.7 would be represented as 0x11030007.
 *
 * DO NOT split the following macro into multiple lines as this may confuse the build scripts.
 */
#define AR6K_SW_VERSION     ( ( __VER_MAJOR_ << VER_MAJOR_BIT_OFFSET ) + ( __VER_MINOR_ << VER_MINOR_BIT_OFFSET ) + ( __VER_PATCH_ << VER_PATCH_BIT_OFFSET ) + ( __BUILD_NUMBER_ << VER_BUILD_NUM_BIT_OFFSET ) )


