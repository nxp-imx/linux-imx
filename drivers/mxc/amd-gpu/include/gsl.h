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

#ifndef __GSL_H
#define __GSL_H

//#define __KGSLLIB_EXPORTS
#define __KERNEL_MODE__


//////////////////////////////////////////////////////////////////////////////
//  forward typedefs
//////////////////////////////////////////////////////////////////////////////
//struct _gsl_device_t;
typedef struct _gsl_device_t    gsl_device_t;


//////////////////////////////////////////////////////////////////////////////
//  includes
//////////////////////////////////////////////////////////////////////////////
#include "gsl_buildconfig.h"

#include "kos_libapi.h"

#include "gsl_klibapi.h"

#ifdef GSL_BLD_YAMATO
#include <reg/yamato.h>

#include "gsl_pm4types.h"
#include "gsl_utils.h"
#include "gsl_drawctxt.h"
#include "gsl_ringbuffer.h"
#endif

#ifdef GSL_BLD_G12
#include <reg/g12_reg.h>

#include "gsl_cmdwindow.h"
#endif

#include "gsl_debug.h"
#include "gsl_mmu.h"
#include "gsl_memmgr.h"
#include "gsl_sharedmem.h"
#include "gsl_intrmgr.h"
#include "gsl_cmdstream.h"
#include "gsl_device.h"
#include "gsl_driver.h"
#include "gsl_log.h"

#include "gsl_config.h"

#endif // __GSL_H
