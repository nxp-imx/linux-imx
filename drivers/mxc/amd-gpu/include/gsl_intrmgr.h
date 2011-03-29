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

#ifndef __GSL_INTRMGR_H
#define __GSL_INTRMGR_H


//////////////////////////////////////////////////////////////////////////////
//  types
//////////////////////////////////////////////////////////////////////////////

// -------------------------------------
// block which can generate an interrupt
// -------------------------------------
typedef enum _gsl_intrblock_t
{
    GSL_INTR_BLOCK_YDX_MH = 0,
    GSL_INTR_BLOCK_YDX_CP,
    GSL_INTR_BLOCK_YDX_RBBM,
    GSL_INTR_BLOCK_YDX_SQ,
    GSL_INTR_BLOCK_G12,
    GSL_INTR_BLOCK_G12_MH,

    GSL_INTR_BLOCK_COUNT,
} gsl_intrblock_t;

// ------------------------
// interrupt block register
// ------------------------
typedef struct _gsl_intrblock_reg_t
{
    gsl_intrblock_t  id;
    gsl_intrid_t     first_id;
    gsl_intrid_t     last_id;
    unsigned int     status_reg;
    unsigned int     clear_reg;
    unsigned int     mask_reg;
} gsl_intrblock_reg_t;

// --------
// callback
// --------
typedef void (*gsl_intr_callback_t)(gsl_intrid_t id, void *cookie);

// -----------------
// interrupt routine
// -----------------
typedef struct _gsl_intr_handler_t
{
    gsl_intr_callback_t callback;
    void *              cookie;
} gsl_intr_handler_t;

// -----------------
// interrupt manager
// -----------------
typedef struct _gsl_intr_t
{
    gsl_flags_t         flags;
    gsl_device_t        *device;
    unsigned int        enabled[GSL_INTR_BLOCK_COUNT];
    gsl_intr_handler_t  handler[GSL_INTR_COUNT];
    oshandle_t          evnt[GSL_INTR_COUNT];
} gsl_intr_t;


//////////////////////////////////////////////////////////////////////////////
//  prototypes
//////////////////////////////////////////////////////////////////////////////
int             kgsl_intr_init(gsl_device_t *device);
int             kgsl_intr_close(gsl_device_t *device);
int             kgsl_intr_attach(gsl_intr_t *intr, gsl_intrid_t id, gsl_intr_callback_t callback, void *cookie);
int             kgsl_intr_detach(gsl_intr_t *intr, gsl_intrid_t id);
int             kgsl_intr_enable(gsl_intr_t *intr, gsl_intrid_t id);
int             kgsl_intr_disable(gsl_intr_t *intr, gsl_intrid_t id);
int             kgsl_intr_isenabled(gsl_intr_t *intr, gsl_intrid_t id);
void            kgsl_intr_decode(gsl_device_t *device, gsl_intrblock_t block_id);

#endif  // __GSL_INTMGR_H
