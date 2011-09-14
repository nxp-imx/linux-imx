/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __GSL_CMDSTREAM_H
#define __GSL_CMDSTREAM_H


//////////////////////////////////////////////////////////////////////////////
//  defines
//////////////////////////////////////////////////////////////////////////////

#ifdef VG_HDK
#define GSL_CMDSTREAM_GET_SOP_TIMESTAMP(device, data)
#else
#define GSL_CMDSTREAM_GET_SOP_TIMESTAMP(device, data)   kgsl_sharedmem_read0(&device->memstore, (data), GSL_DEVICE_MEMSTORE_OFFSET(soptimestamp), 4, false)
#endif

#ifdef VG_HDK
#define GSL_CMDSTREAM_GET_EOP_TIMESTAMP(device, data)   (*((int*)data) = (gsl_driver.device[GSL_DEVICE_G12-1]).timestamp)
#else
#define GSL_CMDSTREAM_GET_EOP_TIMESTAMP(device, data)   kgsl_sharedmem_read0(&device->memstore, (data), GSL_DEVICE_MEMSTORE_OFFSET(eoptimestamp), 4, false)
#endif


//////////////////////////////////////////////////////////////////////////////
// types
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// functions
//////////////////////////////////////////////////////////////////////////////
gsl_timestamp_t kgsl_cmdstream_readtimestamp0(gsl_deviceid_t device_id, gsl_timestamp_type_t type);
void kgsl_cmdstream_memqueue_drain(gsl_device_t *device);
int kgsl_cmdstream_init(gsl_device_t *device);
int kgsl_cmdstream_close(gsl_device_t *device);

#endif  // __GSL_CMDSTREAM_H
