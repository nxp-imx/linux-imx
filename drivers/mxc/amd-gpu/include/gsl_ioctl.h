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

#ifndef _GSL_IOCTL_H
#define _GSL_IOCTL_H

#include "gsl_types.h"
#include "gsl_properties.h"

//////////////////////////////////////////////////////////////////////////////
// types
//////////////////////////////////////////////////////////////////////////////

typedef struct _kgsl_device_start_t {
    gsl_deviceid_t  device_id;
    gsl_flags_t flags;
} kgsl_device_start_t;

typedef struct _kgsl_device_stop_t {
    gsl_deviceid_t  device_id;
} kgsl_device_stop_t;

typedef struct _kgsl_device_idle_t {
    gsl_deviceid_t  device_id;
    unsigned int    timeout;
} kgsl_device_idle_t;

typedef struct _kgsl_device_isidle_t {
    gsl_deviceid_t  device_id;
} kgsl_device_isidle_t;

typedef struct _kgsl_device_getproperty_t {
    gsl_deviceid_t  device_id;
    gsl_property_type_t type;
    unsigned int    *value;
    unsigned int    sizebytes;
} kgsl_device_getproperty_t;

typedef struct _kgsl_device_setproperty_t {
    gsl_deviceid_t  device_id;
    gsl_property_type_t type;
    void        *value;
    unsigned int    sizebytes;
} kgsl_device_setproperty_t;

typedef struct _kgsl_device_regread_t {
    gsl_deviceid_t  device_id;
    unsigned int    offsetwords;
    unsigned int    *value;
} kgsl_device_regread_t;

typedef struct _kgsl_device_regwrite_t {
    gsl_deviceid_t  device_id;
    unsigned int    offsetwords;
    unsigned int    value;
} kgsl_device_regwrite_t;

typedef struct _kgsl_device_waitirq_t {
    gsl_deviceid_t  device_id;
    gsl_intrid_t    intr_id;
    unsigned int    *count;
    unsigned int    timeout;
} kgsl_device_waitirq_t;

typedef struct _kgsl_cmdstream_issueibcmds_t {
    gsl_deviceid_t  device_id;
    int     drawctxt_index;
    gpuaddr_t   ibaddr;
    int     sizedwords;
    gsl_timestamp_t *timestamp;
    gsl_flags_t flags;
} kgsl_cmdstream_issueibcmds_t;

typedef struct _kgsl_cmdstream_readtimestamp_t {
    gsl_deviceid_t  device_id;
    gsl_timestamp_type_t    type;
    gsl_timestamp_t *timestamp;
} kgsl_cmdstream_readtimestamp_t;

typedef struct _kgsl_cmdstream_freememontimestamp_t {
    gsl_deviceid_t  device_id;
    gsl_memdesc_t   *memdesc;
    gsl_timestamp_t timestamp;
    gsl_timestamp_type_t    type;
} kgsl_cmdstream_freememontimestamp_t;

typedef struct _kgsl_cmdstream_waittimestamp_t {
    gsl_deviceid_t  device_id;
	gsl_timestamp_t timestamp;
    unsigned int    timeout;
} kgsl_cmdstream_waittimestamp_t;

typedef struct _kgsl_cmdwindow_write_t {
    gsl_deviceid_t  device_id;
    gsl_cmdwindow_t target;
    unsigned int    addr;
    unsigned int    data;
} kgsl_cmdwindow_write_t;

typedef struct _kgsl_context_create_t {
    gsl_deviceid_t  device_id;
    gsl_context_type_t  type;
    unsigned int    *drawctxt_id;
    gsl_flags_t flags;
} kgsl_context_create_t;

typedef struct _kgsl_context_destroy_t {
    gsl_deviceid_t  device_id;
    unsigned int    drawctxt_id;
} kgsl_context_destroy_t;

typedef struct _kgsl_drawctxt_bind_gmem_shadow_t {
    gsl_deviceid_t device_id;
    unsigned int drawctxt_id;
    const gsl_rect_t* gmem_rect;
    unsigned int shadow_x;
    unsigned int shadow_y;
    const gsl_buffer_desc_t* shadow_buffer;
    unsigned int buffer_id;
} kgsl_drawctxt_bind_gmem_shadow_t;

typedef struct _kgsl_sharedmem_alloc_t {
    gsl_deviceid_t  device_id;
    gsl_flags_t flags;
    int     sizebytes;
    gsl_memdesc_t   *memdesc;
} kgsl_sharedmem_alloc_t;

typedef struct _kgsl_sharedmem_free_t {
    gsl_memdesc_t   *memdesc;
} kgsl_sharedmem_free_t;

typedef struct _kgsl_sharedmem_read_t {
    const gsl_memdesc_t *memdesc;
    unsigned int    *dst;
    unsigned int    offsetbytes;
    unsigned int    sizebytes;
} kgsl_sharedmem_read_t;

typedef struct _kgsl_sharedmem_write_t {
    const gsl_memdesc_t *memdesc;
    unsigned int    offsetbytes;
    unsigned int    *src;
    unsigned int    sizebytes;
} kgsl_sharedmem_write_t;

typedef struct _kgsl_sharedmem_set_t {
    const gsl_memdesc_t *memdesc;
    unsigned int    offsetbytes;
    unsigned int    value;
    unsigned int    sizebytes;
} kgsl_sharedmem_set_t;

typedef struct _kgsl_sharedmem_largestfreeblock_t {
    gsl_deviceid_t  device_id;
    gsl_flags_t flags;
    unsigned int    *largestfreeblock;
} kgsl_sharedmem_largestfreeblock_t;

typedef struct _kgsl_sharedmem_cacheoperation_t {
    const gsl_memdesc_t *memdesc;
    unsigned int    offsetbytes;
    unsigned int    sizebytes;
    unsigned int    operation;
} kgsl_sharedmem_cacheoperation_t;

typedef struct _kgsl_sharedmem_fromhostpointer_t {
    gsl_deviceid_t  device_id;
    gsl_memdesc_t   *memdesc;
    void        *hostptr;
} kgsl_sharedmem_fromhostpointer_t;

typedef struct _kgsl_add_timestamp_t {
    gsl_deviceid_t device_id;
    gsl_timestamp_t *timestamp;
} kgsl_add_timestamp_t;

typedef struct _kgsl_device_clock_t {
    gsl_deviceid_t device; /* GSL_DEVICE_YAMATO = 1, GSL_DEVICE_G12 = 2 */
    int enable; /* 0: disable, 1: enable */
} kgsl_device_clock_t;

//////////////////////////////////////////////////////////////////////////////
// ioctl numbers
//////////////////////////////////////////////////////////////////////////////

#define GSL_MAGIC                               0xF9
#define IOCTL_KGSL_DEVICE_START                 _IOW(GSL_MAGIC, 0x20, struct _kgsl_device_start_t)
#define IOCTL_KGSL_DEVICE_STOP                  _IOW(GSL_MAGIC, 0x21, struct _kgsl_device_stop_t)
#define IOCTL_KGSL_DEVICE_IDLE                  _IOW(GSL_MAGIC, 0x22, struct _kgsl_device_idle_t)
#define IOCTL_KGSL_DEVICE_ISIDLE                _IOR(GSL_MAGIC, 0x23, struct _kgsl_device_isidle_t)
#define IOCTL_KGSL_DEVICE_GETPROPERTY           _IOWR(GSL_MAGIC, 0x24, struct _kgsl_device_getproperty_t)
#define IOCTL_KGSL_DEVICE_SETPROPERTY           _IOW(GSL_MAGIC, 0x25, struct _kgsl_device_setproperty_t)
#define IOCTL_KGSL_DEVICE_REGREAD               _IOWR(GSL_MAGIC, 0x26, struct _kgsl_device_regread_t)
#define IOCTL_KGSL_DEVICE_REGWRITE              _IOW(GSL_MAGIC, 0x27, struct _kgsl_device_regwrite_t)
#define IOCTL_KGSL_DEVICE_WAITIRQ               _IOWR(GSL_MAGIC, 0x28, struct _kgsl_device_waitirq_t)
#define IOCTL_KGSL_CMDSTREAM_ISSUEIBCMDS        _IOWR(GSL_MAGIC, 0x29, struct _kgsl_cmdstream_issueibcmds_t)
#define IOCTL_KGSL_CMDSTREAM_READTIMESTAMP      _IOWR(GSL_MAGIC, 0x2A, struct _kgsl_cmdstream_readtimestamp_t)
#define IOCTL_KGSL_CMDSTREAM_FREEMEMONTIMESTAMP _IOW(GSL_MAGIC, 0x2B, struct _kgsl_cmdstream_freememontimestamp_t)
#define IOCTL_KGSL_CMDSTREAM_WAITTIMESTAMP      _IOW(GSL_MAGIC, 0x2C, struct _kgsl_cmdstream_waittimestamp_t)
#define IOCTL_KGSL_CMDWINDOW_WRITE              _IOW(GSL_MAGIC, 0x2D, struct _kgsl_cmdwindow_write_t)
#define IOCTL_KGSL_CONTEXT_CREATE               _IOWR(GSL_MAGIC, 0x2E, struct _kgsl_context_create_t)
#define IOCTL_KGSL_CONTEXT_DESTROY              _IOW(GSL_MAGIC, 0x2F, struct _kgsl_context_destroy_t)
#define IOCTL_KGSL_DRAWCTXT_BIND_GMEM_SHADOW    _IOW(GSL_MAGIC, 0x30, struct _kgsl_drawctxt_bind_gmem_shadow_t)
#define IOCTL_KGSL_SHAREDMEM_ALLOC              _IOWR(GSL_MAGIC, 0x31, struct _kgsl_sharedmem_alloc_t)
#define IOCTL_KGSL_SHAREDMEM_FREE               _IOW(GSL_MAGIC, 0x32, struct _kgsl_sharedmem_free_t)
#define IOCTL_KGSL_SHAREDMEM_READ               _IOWR(GSL_MAGIC, 0x33, struct _kgsl_sharedmem_read_t)
#define IOCTL_KGSL_SHAREDMEM_WRITE              _IOW(GSL_MAGIC, 0x34, struct _kgsl_sharedmem_write_t)
#define IOCTL_KGSL_SHAREDMEM_SET                _IOW(GSL_MAGIC, 0x35, struct _kgsl_sharedmem_set_t)
#define IOCTL_KGSL_SHAREDMEM_LARGESTFREEBLOCK   _IOWR(GSL_MAGIC, 0x36, struct _kgsl_sharedmem_largestfreeblock_t)
#define IOCTL_KGSL_SHAREDMEM_CACHEOPERATION     _IOW(GSL_MAGIC, 0x37, struct _kgsl_sharedmem_cacheoperation_t)
#define IOCTL_KGSL_SHAREDMEM_FROMHOSTPOINTER    _IOW(GSL_MAGIC, 0x38, struct _kgsl_sharedmem_fromhostpointer_t)
#define IOCTL_KGSL_ADD_TIMESTAMP                _IOWR(GSL_MAGIC, 0x39, struct _kgsl_add_timestamp_t)
#define IOCTL_KGSL_DRIVER_EXIT		        _IOWR(GSL_MAGIC, 0x3A, NULL)
#define IOCTL_KGSL_DEVICE_CLOCK			_IOWR(GSL_MAGIC, 0x60, struct _kgsl_device_clock_t)


#endif
