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

#ifndef __GSL_HWACCESS_LINUX_H
#define __GSL_HWACCESS_LINUX_H

#ifdef _LINUX
#include "gsl_linux_map.h"
#endif

#include <linux/io.h>
#include <asm/system.h>
#include <asm/uaccess.h>

OSINLINE void
kgsl_hwaccess_memread(void *dst, unsigned int gpubase, unsigned int gpuoffset, unsigned int sizebytes, unsigned int touserspace)
{
    if (gsl_driver.enable_mmu && (gpubase >= GSL_LINUX_MAP_RANGE_START) && (gpubase < GSL_LINUX_MAP_RANGE_END)) {
        gsl_linux_map_read(dst, gpubase+gpuoffset, sizebytes, touserspace);
    } else {
        if (touserspace)
        {
            if (copy_to_user(dst, (void *)(gpubase + gpuoffset), sizebytes))
            {
                return;
            }
        }
        else
        {
            kos_memcpy(dst, (void *) (gpubase + gpuoffset), sizebytes);
        }
    }
}

//----------------------------------------------------------------------------

OSINLINE void
kgsl_hwaccess_memwrite(unsigned int gpubase, unsigned int gpuoffset, void *src, unsigned int sizebytes, unsigned int fromuserspace)
{
    if (gsl_driver.enable_mmu && (gpubase >= GSL_LINUX_MAP_RANGE_START) && (gpubase < GSL_LINUX_MAP_RANGE_END)) {
        gsl_linux_map_write(src, gpubase+gpuoffset, sizebytes, fromuserspace);
    } else {
        if (fromuserspace)
        {
            if (copy_from_user((void *)(gpubase + gpuoffset), src, sizebytes))
            {
                return;
            }
        }
        else
        {
            kos_memcpy((void *)(gpubase + gpuoffset), src, sizebytes);
        }
    }
}

//----------------------------------------------------------------------------

OSINLINE void
kgsl_hwaccess_memset(unsigned int gpubase, unsigned int gpuoffset, unsigned int value, unsigned int sizebytes)
{
    if (gsl_driver.enable_mmu && (gpubase >= GSL_LINUX_MAP_RANGE_START) && (gpubase < GSL_LINUX_MAP_RANGE_END)) {
	gsl_linux_map_set(gpuoffset+gpubase, value, sizebytes);
    } else {
        kos_memset((void *)(gpubase + gpuoffset), value, sizebytes);
    }
}

//----------------------------------------------------------------------------

OSINLINE void
kgsl_hwaccess_regread(gsl_deviceid_t device_id, unsigned int gpubase, unsigned int offsetwords, unsigned int *data)
{
    unsigned int *reg;

    // unreferenced formal parameter
    (void) device_id;

    reg = (unsigned int *)(gpubase + (offsetwords << 2));
    
	*data = readl(reg);
}

//----------------------------------------------------------------------------

OSINLINE void
kgsl_hwaccess_regwrite(gsl_deviceid_t device_id, unsigned int gpubase, unsigned int offsetwords, unsigned int data)
{
    unsigned int *reg;

    // unreferenced formal parameter
    (void) device_id;

    reg = (unsigned int *)(gpubase + (offsetwords << 2));
	writel(data, reg);
}
#endif  // __GSL_HWACCESS_WINCE_MX51_H
