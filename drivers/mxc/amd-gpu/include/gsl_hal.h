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

#ifndef __GSL_HALAPI_H
#define __GSL_HALAPI_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/*
#include "gsl_buildconfig.h"
#include "kos_libapi.h"
#include "gsl_klibapi.h"
#ifdef GSL_BLD_YAMATO
#include <reg/yamato.h>
#endif
#ifdef GSL_BLD_G12
#include <reg/g12_reg.h>
#endif
#include "gsl_hwaccess.h"
*/

#include "gsl.h"
#include "gsl_hwaccess.h"


//////////////////////////////////////////////////////////////////////////////
// linkage
//////////////////////////////////////////////////////////////////////////////
#ifdef __KGSLHAL_EXPORTS
#define KGSLHAL_API                 OS_DLLEXPORT
#else
#define KGSLHAL_API
#endif // __KGSLLIB_EXPORTS


//////////////////////////////////////////////////////////////////////////////
//  version control                    
//////////////////////////////////////////////////////////////////////////////
#define KGSLHAL_NAME            "AMD GSL Kernel HAL"
#define KGSLHAL_VERSION         "0.1"


//////////////////////////////////////////////////////////////////////////////
//  macros
//////////////////////////////////////////////////////////////////////////////
#define GSL_HAL_REG_READ(device_id, gpubase, offsetwords, value)    kgsl_hwaccess_regread(device_id, gpubase, (offsetwords), (value))
#define GSL_HAL_REG_WRITE(device_id, gpubase, offsetwords, value)   kgsl_hwaccess_regwrite(device_id, gpubase, (offsetwords), (value))

#define GSL_HAL_MEM_READ(dst, gpubase, gpuoffset, sizebytes, touserspace)        kgsl_hwaccess_memread(dst, gpubase, (gpuoffset), (sizebytes), touserspace)
#define GSL_HAL_MEM_WRITE(gpubase, gpuoffset, src, sizebytes, fromuserspace)     kgsl_hwaccess_memwrite(gpubase, (gpuoffset), src, (sizebytes), fromuserspace)
#define GSL_HAL_MEM_SET(gpubase, gpuoffset, value, sizebytes)                    kgsl_hwaccess_memset(gpubase, (gpuoffset), (value), (sizebytes))


//////////////////////////////////////////////////////////////////////////////
//  types
//////////////////////////////////////////////////////////////////////////////

// -------------
// device config
// -------------
typedef struct _gsl_devconfig_t {

    gsl_memregion_t  regspace;

    unsigned int     mmu_config;
    gpuaddr_t        mpu_base;
    int              mpu_range;
    gpuaddr_t        va_base;
    unsigned int     va_range;

#ifdef GSL_BLD_YAMATO
    gsl_memregion_t  gmemspace;
#endif // GSL_BLD_YAMATO

} gsl_devconfig_t;

// ----------------------
// memory aperture config
// ----------------------
typedef struct _gsl_apertureconfig_t
{
    gsl_apertureid_t  id;
    gsl_channelid_t   channel;
    unsigned int      hostbase;
    unsigned int      gpubase;
    unsigned int      sizebytes;
} gsl_apertureconfig_t;

// --------------------
// shared memory config
// --------------------
typedef struct _gsl_shmemconfig_t 
{
    int                   numapertures;
    gsl_apertureconfig_t  apertures[GSL_SHMEM_MAX_APERTURES];
} gsl_shmemconfig_t;

typedef struct _gsl_hal_t {
     gsl_memregion_t z160_regspace;
     gsl_memregion_t z430_regspace;
     gsl_memregion_t memchunk;
     gsl_memregion_t memspace[GSL_SHMEM_MAX_APERTURES];
     unsigned int has_z160;
     unsigned int has_z430;
} gsl_hal_t;


//////////////////////////////////////////////////////////////////////////////
//  HAL API
//////////////////////////////////////////////////////////////////////////////
KGSLHAL_API int             kgsl_hal_init(void);
KGSLHAL_API int             kgsl_hal_close(void);
KGSLHAL_API int             kgsl_hal_getshmemconfig(gsl_shmemconfig_t *config);
KGSLHAL_API int             kgsl_hal_getdevconfig(gsl_deviceid_t device_id, gsl_devconfig_t *config);
KGSLHAL_API int             kgsl_hal_setpowerstate(gsl_deviceid_t device_id, int state, unsigned int value);
KGSLHAL_API gsl_chipid_t    kgsl_hal_getchipid(gsl_deviceid_t device_id);
KGSLHAL_API int             kgsl_hal_allocphysical(unsigned int virtaddr, unsigned int numpages, unsigned int scattergatterlist[]);
KGSLHAL_API int             kgsl_hal_freephysical(unsigned int virtaddr, unsigned int numpages, unsigned int scattergatterlist[]);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif  // __GSL_HALAPI_H
