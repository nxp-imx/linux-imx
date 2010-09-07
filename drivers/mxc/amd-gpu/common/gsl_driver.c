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


//////////////////////////////////////////////////////////////////////////////
//  defines
//////////////////////////////////////////////////////////////////////////////
#define GSL_PROCESSID_NONE          0x00000000

#define GSL_DRVFLAGS_EXTERNAL       0x10000000
#define GSL_DRVFLAGS_INTERNAL       0x20000000


//////////////////////////////////////////////////////////////////////////////
// globals
//////////////////////////////////////////////////////////////////////////////
#ifndef KGSL_USER_MODE
static gsl_flags_t  gsl_driver_initialized = 0;
gsl_driver_t        gsl_driver;
#else
extern gsl_flags_t  gsl_driver_initialized;
extern gsl_driver_t gsl_driver;
#endif


//////////////////////////////////////////////////////////////////////////////
// functions
//////////////////////////////////////////////////////////////////////////////

int
kgsl_driver_init0(gsl_flags_t flags, gsl_flags_t flags_debug)
{
    int  status = GSL_SUCCESS;

    if (!(gsl_driver_initialized & GSL_FLAGS_INITIALIZED0))
    {
#ifdef GSL_LOG
        // Uncomment these to enable logging.
        //kgsl_log_init();
        //kgsl_log_open_stdout( KGSL_LOG_GROUP_ALL | KGSL_LOG_LEVEL_ALL | KGSL_LOG_TIMESTAMP
        //                      | KGSL_LOG_THREAD_ID | KGSL_LOG_PROCESS_ID );
        //kgsl_log_open_file( "c:\\kgsl_log.txt", KGSL_LOG_GROUP_ALL | KGSL_LOG_LEVEL_ALL | KGSL_LOG_TIMESTAMP
        //                      | KGSL_LOG_THREAD_ID | KGSL_LOG_PROCESS_ID );
#endif
        kos_memset(&gsl_driver, 0, sizeof(gsl_driver_t));

        GSL_API_MUTEX_CREATE();
    }

#ifdef _DEBUG
    // set debug flags on every entry, and prior to hal initialization
    gsl_driver.flags_debug |= flags_debug;
#else
    (void) flags_debug;     // unref formal parameter
#endif // _DEBUG


    KGSL_DEBUG(GSL_DBGFLAGS_DUMPX,
    {
        KGSL_DEBUG_DUMPX_OPEN("dumpx.tb", 0);
        KGSL_DEBUG_DUMPX( BB_DUMP_ENABLE, 0, 0, 0, " ");
    });

    KGSL_DEBUG_TBDUMP_OPEN("tbdump.txt");

    if (!(gsl_driver_initialized & GSL_FLAGS_INITIALIZED0))
    {
        GSL_API_MUTEX_LOCK();

        // init hal
        status = kgsl_hal_init();

        if (status == GSL_SUCCESS)
        {
            gsl_driver_initialized |= flags;
            gsl_driver_initialized |= GSL_FLAGS_INITIALIZED0;
        }

        GSL_API_MUTEX_UNLOCK();
    }

    return (status);
}

//----------------------------------------------------------------------------

int
kgsl_driver_close0(gsl_flags_t flags)
{
    int  status = GSL_SUCCESS;

    if ((gsl_driver_initialized & GSL_FLAGS_INITIALIZED0) && (gsl_driver_initialized & flags))
    {
        GSL_API_MUTEX_LOCK();

        // close hall
        status = kgsl_hal_close();

        GSL_API_MUTEX_UNLOCK();

        GSL_API_MUTEX_FREE();

#ifdef GSL_LOG
        kgsl_log_close();
#endif

        gsl_driver_initialized &= ~flags;
        gsl_driver_initialized &= ~GSL_FLAGS_INITIALIZED0;

        KGSL_DEBUG(GSL_DBGFLAGS_DUMPX,
        {
            KGSL_DEBUG_DUMPX_CLOSE();
        });

        KGSL_DEBUG_TBDUMP_CLOSE();
    }

    return (status);
}

//----------------------------------------------------------------------------

KGSL_API int
kgsl_driver_init()
{
    // only an external (platform specific device driver) component should call this

    return(kgsl_driver_init0(GSL_DRVFLAGS_EXTERNAL, 0));
}

//----------------------------------------------------------------------------

KGSL_API int
kgsl_driver_close()
{
    // only an external (platform specific device driver) component should call this

    return(kgsl_driver_close0(GSL_DRVFLAGS_EXTERNAL));
}

//----------------------------------------------------------------------------

KGSL_API int
kgsl_driver_entry(gsl_flags_t flags)
{
    int           status = GSL_FAILURE;
    int           index, i;
    unsigned int  pid;

    if (kgsl_driver_init0(GSL_DRVFLAGS_INTERNAL, flags) != GSL_SUCCESS)
    {
        return (GSL_FAILURE);
    }

    kgsl_log_write( KGSL_LOG_GROUP_DRIVER | KGSL_LOG_LEVEL_TRACE, "--> int kgsl_driver_entry( gsl_flags_t flags=%d )\n", flags );

    GSL_API_MUTEX_LOCK();

    pid = GSL_CALLER_PROCESSID_GET();

    // if caller process has not already opened access
    status = kgsl_driver_getcallerprocessindex(pid, &index);
    if (status != GSL_SUCCESS)
    {
        // then, add caller pid to process table
        status = kgsl_driver_getcallerprocessindex(GSL_PROCESSID_NONE, &index);
        if (status == GSL_SUCCESS)
        {
            gsl_driver.callerprocess[index] = pid;
            gsl_driver.refcnt++;
        }
    }

    if (status == GSL_SUCCESS)
    {
        if (!(gsl_driver_initialized & GSL_FLAGS_INITIALIZED))
        {
            // init memory apertures
            status = kgsl_sharedmem_init(&gsl_driver.shmem);
            if (status == GSL_SUCCESS)
            {
                // init devices
		status = GSL_FAILURE;
                for (i = 0; i < GSL_DEVICE_MAX; i++)
                {
		    if (kgsl_device_init(&gsl_driver.device[i], (gsl_deviceid_t)(i + 1)) == GSL_SUCCESS) {
			status = GSL_SUCCESS;
		    }
                }
            }

            if (status == GSL_SUCCESS)
            {
                gsl_driver_initialized |= GSL_FLAGS_INITIALIZED;
            }
        }

        // walk through process attach callbacks
        if (status == GSL_SUCCESS)
        {
            for (i = 0; i < GSL_DEVICE_MAX; i++)
            {
                status = kgsl_device_attachcallback(&gsl_driver.device[i], pid);
                if (status != GSL_SUCCESS)
                {
                    break;
                }
            }
        }

        // if something went wrong
        if (status != GSL_SUCCESS)
        {
            // then, remove caller pid from process table
            if (kgsl_driver_getcallerprocessindex(pid, &index) == GSL_SUCCESS)
            {
                gsl_driver.callerprocess[index] = GSL_PROCESSID_NONE;
                gsl_driver.refcnt--;
            }
        }
    }

    GSL_API_MUTEX_UNLOCK();

    kgsl_log_write( KGSL_LOG_GROUP_DRIVER | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_driver_entry. Return value: %B\n", status );

    return (status);
}

//----------------------------------------------------------------------------

int
kgsl_driver_exit0(unsigned int pid)
{
    int  status = GSL_SUCCESS;
    int  index, i;

    GSL_API_MUTEX_LOCK();

    if (gsl_driver_initialized & GSL_FLAGS_INITIALIZED)
    {
        if (kgsl_driver_getcallerprocessindex(pid, &index) == GSL_SUCCESS)
        {
            // walk through process detach callbacks
            for (i = 0; i < GSL_DEVICE_MAX; i++)
            {
                // Empty the freememqueue of this device
                kgsl_cmdstream_memqueue_drain(&gsl_driver.device[i]);

                // Detach callback
                status = kgsl_device_detachcallback(&gsl_driver.device[i], pid);
                if (status != GSL_SUCCESS)
                {
                    break;
                }
            }

            // last running caller process
            if (gsl_driver.refcnt - 1 == 0)
            {
                // close devices
                for (i = 0; i < GSL_DEVICE_MAX; i++)
                {
                    kgsl_device_close(&gsl_driver.device[i]);
                }

                // shutdown memory apertures
                kgsl_sharedmem_close(&gsl_driver.shmem);

                gsl_driver_initialized &= ~GSL_FLAGS_INITIALIZED;
            }

            // remove caller pid from process table
            gsl_driver.callerprocess[index] = GSL_PROCESSID_NONE;
            gsl_driver.refcnt--;
        }
    }

    GSL_API_MUTEX_UNLOCK();

    if (!(gsl_driver_initialized & GSL_FLAGS_INITIALIZED))
    {
        kgsl_driver_close0(GSL_DRVFLAGS_INTERNAL);
    }

    return (status);
}

//----------------------------------------------------------------------------

KGSL_API int
kgsl_driver_exit(void)
{
    int status;

    kgsl_log_write( KGSL_LOG_GROUP_DRIVER | KGSL_LOG_LEVEL_TRACE, "--> int kgsl_driver_exit()\n" );

    status = kgsl_driver_exit0(GSL_CALLER_PROCESSID_GET());

    kgsl_log_write( KGSL_LOG_GROUP_DRIVER | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_driver_exit(). Return value: %B\n", status );

    return (status);
}

//----------------------------------------------------------------------------

KGSL_API int
kgsl_driver_destroy(unsigned int pid)
{
    return (kgsl_driver_exit0(pid));
}
