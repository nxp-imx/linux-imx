/* Copyright (c) 2002,2007-2010, Code Aurora Forum. All rights reserved.
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

#ifdef GSL_BLD_G12

//////////////////////////////////////////////////////////////////////////////
//  defines                    
//////////////////////////////////////////////////////////////////////////////
#define GSL_CMDWINDOW_TARGET_MASK       0x000000FF
#define GSL_CMDWINDOW_ADDR_MASK         0x00FFFF00
#define GSL_CMDWINDOW_TARGET_SHIFT      0
#define GSL_CMDWINDOW_ADDR_SHIFT        8


//////////////////////////////////////////////////////////////////////////////
// macros
//////////////////////////////////////////////////////////////////////////////
#ifdef GSL_LOCKING_FINEGRAIN
#define GSL_CMDWINDOW_MUTEX_CREATE()        device->cmdwindow_mutex = kos_mutex_create("gsl_cmdwindow"); \
                                            if (!device->cmdwindow_mutex) return (GSL_FAILURE);
#define GSL_CMDWINDOW_MUTEX_LOCK()          kos_mutex_lock(device->cmdwindow_mutex)
#define GSL_CMDWINDOW_MUTEX_UNLOCK()        kos_mutex_unlock(device->cmdwindow_mutex)
#define GSL_CMDWINDOW_MUTEX_FREE()          kos_mutex_free(device->cmdwindow_mutex); device->cmdwindow_mutex = 0;
#else
#define GSL_CMDWINDOW_MUTEX_CREATE()
#define GSL_CMDWINDOW_MUTEX_LOCK()
#define GSL_CMDWINDOW_MUTEX_UNLOCK()
#define GSL_CMDWINDOW_MUTEX_FREE()
#endif


//////////////////////////////////////////////////////////////////////////////
// functions
//////////////////////////////////////////////////////////////////////////////

int
kgsl_cmdwindow_init(gsl_device_t *device)
{
    GSL_CMDWINDOW_MUTEX_CREATE();

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

int 
kgsl_cmdwindow_close(gsl_device_t *device)
{
    GSL_CMDWINDOW_MUTEX_FREE();

    return (GSL_SUCCESS);
}

#endif // GSL_BLD_G12

//----------------------------------------------------------------------------

int
kgsl_cmdwindow_write0(gsl_deviceid_t device_id, gsl_cmdwindow_t target, unsigned int addr, unsigned int data)
{
#ifdef GSL_BLD_G12
    gsl_device_t  *device;
    unsigned int  cmdwinaddr;
    unsigned int  cmdstream;

    kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE,
                    "--> int kgsl_cmdwindow_write( gsl_device_id_t device_id=%d, gsl_cmdwindow_t target=%d, unsigned int addr=0x%08x, unsigned int data=0x%08x)\n", device_id, target, addr, data );

    device = &gsl_driver.device[device_id-1];       // device_id is 1 based

    if (target < GSL_CMDWINDOW_MIN || target > GSL_CMDWINDOW_MAX)
    {
        kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_ERROR, "ERROR: Invalid target.\n" );
        kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_cmdwindow_write. Return value %B\n", GSL_FAILURE );
        return (GSL_FAILURE);
    }

    if ((!(device->flags & GSL_FLAGS_INITIALIZED) && target == GSL_CMDWINDOW_MMU) ||
        (!(device->flags & GSL_FLAGS_STARTED)     && target != GSL_CMDWINDOW_MMU))
    {
        kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_ERROR, "ERROR: Invalid device state to write to selected targer.\n" );
        kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_cmdwindow_write. Return value %B\n", GSL_FAILURE );
        return (GSL_FAILURE);
    }

    // set command stream
    if (target == GSL_CMDWINDOW_MMU)
    {
#ifdef GSL_NO_MMU
        return (GSL_SUCCESS);
#endif
        cmdstream = ADDR_VGC_MMUCOMMANDSTREAM;
    }
    else
    {
        cmdstream = ADDR_VGC_COMMANDSTREAM;
    }


    // set command window address
    cmdwinaddr  = ((target << GSL_CMDWINDOW_TARGET_SHIFT) & GSL_CMDWINDOW_TARGET_MASK);
    cmdwinaddr |= ((addr   << GSL_CMDWINDOW_ADDR_SHIFT)   & GSL_CMDWINDOW_ADDR_MASK);

    GSL_CMDWINDOW_MUTEX_LOCK();

#ifndef GSL_NO_MMU
    // set mmu pagetable
	kgsl_mmu_setpagetable(device, GSL_CALLER_PROCESSID_GET());
#endif

    // write command window address
    device->ftbl.device_regwrite(device, (cmdstream)>>2, cmdwinaddr);

    // write data
    device->ftbl.device_regwrite(device, (cmdstream)>>2, data);

    GSL_CMDWINDOW_MUTEX_UNLOCK();

    kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_cmdwindow_write. Return value %B\n", GSL_SUCCESS );

    return (GSL_SUCCESS);
#else
    // unreferenced formal parameter
    (void) device_id;
    (void) target;
    (void) addr;
    (void) data;

    return (GSL_FAILURE);
#endif // GSL_BLD_G12
}

//----------------------------------------------------------------------------

KGSL_API int
kgsl_cmdwindow_write(gsl_deviceid_t device_id, gsl_cmdwindow_t target, unsigned int addr, unsigned int data)
{
	int status = GSL_SUCCESS;
	GSL_API_MUTEX_LOCK();
	status = kgsl_cmdwindow_write0(device_id, target, addr, data);
	GSL_API_MUTEX_UNLOCK();
	return status;
}
