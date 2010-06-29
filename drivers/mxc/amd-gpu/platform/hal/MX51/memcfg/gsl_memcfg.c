/* Copyright (c) 2008-2010, Advanced Micro Devices. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include "kos_libapi.h"

//
//  Return the maximum amount of memory that can be allocated to the Z430.  This number
//  will be constrained to 2MB as a minimum and the original hardcoded value for the caller
//  as a maximum.  If the return value is outside of this range, then the original value in 
//  the caller will be used.  For this reason, returning 0 is used to signify to use the
//  original value as the default.
//
KOS_DLLEXPORT unsigned long kgsl_get_z430_memory_amount(void)
{
    return(0);
}
