/* Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
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

#ifndef __GSL_KLIBAPI_H
#define __GSL_KLIBAPI_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include "gsl_types.h"
#include "gsl_properties.h"


//////////////////////////////////////////////////////////////////////////////
//  entrypoints
//////////////////////////////////////////////////////////////////////////////
#ifdef __KGSLLIB_EXPORTS
#define KGSL_API                    OS_DLLEXPORT
#else
#ifdef __KERNEL_MODE__
#define KGSL_API                    extern
#else
#define KGSL_API                    OS_DLLIMPORT
#endif
#endif // __KGSLLIB_EXPORTS


//////////////////////////////////////////////////////////////////////////////
//  version control                    
//////////////////////////////////////////////////////////////////////////////
#define KGSLLIB_NAME            "AMD GSL Kernel Library"
#define KGSLLIB_VERSION         "0.1"


//////////////////////////////////////////////////////////////////////////////
//  library API
//////////////////////////////////////////////////////////////////////////////
KGSL_API int                kgsl_driver_init(void);
KGSL_API int                kgsl_driver_close(void);
KGSL_API int                kgsl_driver_entry(gsl_flags_t flags);
KGSL_API int                kgsl_driver_exit(void);
KGSL_API int                kgsl_driver_destroy(unsigned int pid);


////////////////////////////////////////////////////////////////////////////
//  device API
////////////////////////////////////////////////////////////////////////////
KGSL_API int                kgsl_device_start(gsl_deviceid_t device_id, gsl_flags_t flags);
KGSL_API int                kgsl_device_stop(gsl_deviceid_t device_id);
KGSL_API int                kgsl_device_idle(gsl_deviceid_t device_id, unsigned int timeout);
KGSL_API int                kgsl_device_isidle(gsl_deviceid_t device_id);
KGSL_API int                kgsl_device_getproperty(gsl_deviceid_t device_id, gsl_property_type_t type, void *value, unsigned int sizebytes);
KGSL_API int                kgsl_device_setproperty(gsl_deviceid_t device_id, gsl_property_type_t type, void *value, unsigned int sizebytes);
KGSL_API int                kgsl_device_regread(gsl_deviceid_t device_id, unsigned int offsetwords, unsigned int *value);
KGSL_API int                kgsl_device_regwrite(gsl_deviceid_t device_id, unsigned int offsetwords, unsigned int value);
KGSL_API int                kgsl_device_waitirq(gsl_deviceid_t device_id, gsl_intrid_t intr_id, unsigned int *count, unsigned int timeout);


////////////////////////////////////////////////////////////////////////////
//  command API
////////////////////////////////////////////////////////////////////////////
KGSL_API int                kgsl_cmdstream_issueibcmds(gsl_deviceid_t device_id, int drawctxt_index, gpuaddr_t ibaddr, int sizedwords, gsl_timestamp_t *timestamp, gsl_flags_t flags);
KGSL_API gsl_timestamp_t    kgsl_cmdstream_readtimestamp(gsl_deviceid_t device_id, gsl_timestamp_type_t type);
KGSL_API int                kgsl_cmdstream_freememontimestamp(gsl_deviceid_t device_id, gsl_memdesc_t *memdesc, gsl_timestamp_t timestamp, gsl_timestamp_type_t type);
KGSL_API int                kgsl_cmdstream_waittimestamp(gsl_deviceid_t device_id, gsl_timestamp_t timestamp, unsigned int timeout);
KGSL_API int                kgsl_cmdwindow_write(gsl_deviceid_t device_id, gsl_cmdwindow_t target, unsigned int addr, unsigned int data);
KGSL_API int                kgsl_add_timestamp(gsl_deviceid_t device_id, gsl_timestamp_t *timestamp);
KGSL_API int                kgsl_cmdstream_check_timestamp(gsl_deviceid_t device_id, gsl_timestamp_t timestamp);

////////////////////////////////////////////////////////////////////////////
//  context API
////////////////////////////////////////////////////////////////////////////
KGSL_API int                kgsl_context_create(gsl_deviceid_t device_id, gsl_context_type_t type, unsigned int *drawctxt_id, gsl_flags_t flags);
KGSL_API int                kgsl_context_destroy(gsl_deviceid_t device_id, unsigned int drawctxt_id);
KGSL_API int                kgsl_drawctxt_bind_gmem_shadow(gsl_deviceid_t device_id, unsigned int drawctxt_id, const gsl_rect_t* gmem_rect, unsigned int shadow_x, unsigned int shadow_y, const gsl_buffer_desc_t* shadow_buffer, unsigned int buffer_id);


////////////////////////////////////////////////////////////////////////////
//  sharedmem API
////////////////////////////////////////////////////////////////////////////
KGSL_API int                kgsl_sharedmem_alloc(gsl_deviceid_t device_id, gsl_flags_t flags, int sizebytes, gsl_memdesc_t *memdesc);
KGSL_API int                kgsl_sharedmem_free(gsl_memdesc_t *memdesc);
KGSL_API int                kgsl_sharedmem_read(const gsl_memdesc_t *memdesc, void *dst, unsigned int offsetbytes, unsigned int sizebytes, unsigned int touserspace);
KGSL_API int                kgsl_sharedmem_write(const gsl_memdesc_t *memdesc, unsigned int offsetbytes, void *src, unsigned int sizebytes, unsigned int fromuserspace);
KGSL_API int                kgsl_sharedmem_set(const gsl_memdesc_t *memdesc, unsigned int offsetbytes, unsigned int value, unsigned int sizebytes);
KGSL_API unsigned int       kgsl_sharedmem_largestfreeblock(gsl_deviceid_t device_id, gsl_flags_t flags);
KGSL_API int                kgsl_sharedmem_map(gsl_deviceid_t device_id, gsl_flags_t flags, const gsl_scatterlist_t *scatterlist, gsl_memdesc_t *memdesc);
KGSL_API int                kgsl_sharedmem_unmap(gsl_memdesc_t *memdesc);
KGSL_API int                kgsl_sharedmem_getmap(const gsl_memdesc_t *memdesc, gsl_scatterlist_t *scatterlist);
KGSL_API int                kgsl_sharedmem_cacheoperation(const gsl_memdesc_t *memdesc, unsigned int offsetbytes, unsigned int sizebytes, unsigned int operation);
KGSL_API int                kgsl_sharedmem_fromhostpointer(gsl_deviceid_t device_id, gsl_memdesc_t *memdesc, void* hostptr);


////////////////////////////////////////////////////////////////////////////
//  interrupt API
////////////////////////////////////////////////////////////////////////////
KGSL_API void               kgsl_intr_isr(void);


////////////////////////////////////////////////////////////////////////////
//  TB dump API
////////////////////////////////////////////////////////////////////////////
KGSL_API int                kgsl_tbdump_waitirq(void);
KGSL_API int                kgsl_tbdump_exportbmp(const void* addr, unsigned int format, unsigned int stride, unsigned int width, unsigned int height);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif  // __GSL_KLIBAPI_H
