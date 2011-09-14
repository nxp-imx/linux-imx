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

#include "gsl.h"
#include "gsl_hal.h"
#include "gsl_context.h"

//////////////////////////////////////////////////////////////////////////////
// functions
//////////////////////////////////////////////////////////////////////////////

KGSL_API int
kgsl_context_create(gsl_deviceid_t device_id, gsl_context_type_t type, unsigned int *drawctxt_id, gsl_flags_t flags)
{
    gsl_device_t* device  = &gsl_driver.device[device_id-1];
    int status;

    GSL_API_MUTEX_LOCK();

    if (device->ftbl.context_create)
    {
        status = device->ftbl.context_create(device, type, drawctxt_id, flags);
    }
    else
    {
        status = GSL_FAILURE;
    }

    GSL_API_MUTEX_UNLOCK();

    return status;
}

//----------------------------------------------------------------------------

KGSL_API int
kgsl_context_destroy(gsl_deviceid_t device_id, unsigned int drawctxt_id)
{
    gsl_device_t* device  = &gsl_driver.device[device_id-1];
    int status;

    GSL_API_MUTEX_LOCK();

    if (device->ftbl.context_destroy)
    {
        status = device->ftbl.context_destroy(device, drawctxt_id);
    }
    else
    {
        status = GSL_FAILURE;
    }

    GSL_API_MUTEX_UNLOCK();

    return status;
}

//----------------------------------------------------------------------------

