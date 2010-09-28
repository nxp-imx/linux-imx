/* Copyright (c) 2008-2010, Advanced Micro Devices. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Advanced Micro Devices nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __GSL_DRIVER_H
#define __GSL_DRIVER_H


/////////////////////////////////////////////////////////////////////////////
// macros
//////////////////////////////////////////////////////////////////////////////
#ifdef GSL_DEDICATED_PROCESS
#define GSL_CALLER_PROCESSID_GET()      kos_callerprocess_getid()
#else
#define GSL_CALLER_PROCESSID_GET()      kos_process_getid()
#endif // GSL_DEDICATED_PROCESS

#ifdef GSL_LOCKING_COARSEGRAIN
#define GSL_API_MUTEX_CREATE()          gsl_driver.mutex = kos_mutex_create("gsl_global"); \
                                        if (!gsl_driver.mutex) {return (GSL_FAILURE);}
#define GSL_API_MUTEX_LOCK()            kos_mutex_lock(gsl_driver.mutex)
#define GSL_API_MUTEX_UNLOCK()          kos_mutex_unlock(gsl_driver.mutex)
#define GSL_API_MUTEX_FREE()            kos_mutex_free(gsl_driver.mutex); gsl_driver.mutex = 0;
#else
#define GSL_API_MUTEX_CREATE()
#define GSL_API_MUTEX_LOCK()        
#define GSL_API_MUTEX_UNLOCK()      
#define GSL_API_MUTEX_FREE()
#endif


//////////////////////////////////////////////////////////////////////////////
// types
//////////////////////////////////////////////////////////////////////////////

// -------------
// driver object 
// -------------
typedef struct _gsl_driver_t {
    gsl_flags_t      flags_debug;
    int              refcnt;
    unsigned int     callerprocess[GSL_CALLER_PROCESS_MAX]; // caller process table
    oshandle_t       mutex;                                 // global API mutex
    void             *hal;
    gsl_sharedmem_t  shmem;
    gsl_device_t     device[GSL_DEVICE_MAX];
    int              dmi_state;     //  OS_TRUE = enabled, OS_FALSE otherwise
    gsl_flags_t      dmi_mode;      //  single, double, or triple buffering
    int              dmi_frame;     //  set to -1 when DMI is enabled
    int              dmi_max_frame; //  indicates the maximum frame # that we will support
    int              enable_mmu;
} gsl_driver_t;


//////////////////////////////////////////////////////////////////////////////
// external variable declarations
//////////////////////////////////////////////////////////////////////////////
extern gsl_driver_t  gsl_driver;


//////////////////////////////////////////////////////////////////////////////
//  inline functions
//////////////////////////////////////////////////////////////////////////////
OSINLINE int
kgsl_driver_getcallerprocessindex(unsigned int pid, int *index)
{
    int  i;

    // obtain index in caller process table
    for (i = 0; i < GSL_CALLER_PROCESS_MAX; i++)
    {
        if (gsl_driver.callerprocess[i] == pid)
        {
            *index = i;            
            return (GSL_SUCCESS);
        }
    }

    return (GSL_FAILURE);
}

#endif // __GSL_DRIVER_H
