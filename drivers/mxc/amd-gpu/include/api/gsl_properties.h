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

#ifndef __GSL_PROPERTIES_H
#define __GSL_PROPERTIES_H


//////////////////////////////////////////////////////////////////////////////
// types
//////////////////////////////////////////////////////////////////////////////

// --------------
// property types
// --------------
typedef enum _gsl_property_type_t
{
    GSL_PROP_DEVICE_INFO      = 0x00000001,
    GSL_PROP_DEVICE_SHADOW    = 0x00000002,
    GSL_PROP_DEVICE_POWER     = 0x00000003,
    GSL_PROP_SHMEM            = 0x00000004,
    GSL_PROP_SHMEM_APERTURES  = 0x00000005,
    GSL_PROP_DEVICE_DMI       = 0x00000006
} gsl_property_type_t;

// -----------------
// aperture property
// -----------------
typedef struct _gsl_apertureprop_t {
    unsigned int  gpuaddr;
    unsigned int  hostaddr;
} gsl_apertureprop_t;

// --------------
// shmem property
// --------------
typedef struct _gsl_shmemprop_t {
    int                 numapertures;
    unsigned int        aperture_mask;
    unsigned int        aperture_shift;
    gsl_apertureprop_t  *aperture;
} gsl_shmemprop_t;

// -----------------------------
// device shadow memory property
// -----------------------------
typedef struct _gsl_shadowprop_t {
    unsigned int  hostaddr;
    unsigned int  size;
    gsl_flags_t   flags;
} gsl_shadowprop_t;

// ---------------------
// device power property
// ---------------------
typedef struct _gsl_powerprop_t {
    unsigned int  value;
    gsl_flags_t   flags;
} gsl_powerprop_t;


// ---------------------
// device DMI property
// ---------------------
typedef struct _gsl_dmiprop_t {
    unsigned int  value;
    gsl_flags_t   flags;
} gsl_dmiprop_t;

#endif  // __GSL_PROPERTIES_H
