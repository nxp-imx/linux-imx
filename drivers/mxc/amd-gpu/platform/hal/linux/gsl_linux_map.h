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

#ifndef __GSL_LINUX_MAP_H__
#define __GSL_LINUX_MAP_H__

#include "gsl_halconfig.h"

#define GSL_LINUX_MAP_RANGE_START (1024*1024)
#define GSL_LINUX_MAP_RANGE_END (GSL_LINUX_MAP_RANGE_START+GSL_HAL_SHMEM_SIZE_EMEM1_MMU)

int gsl_linux_map_init(void);
void *gsl_linux_map_alloc(unsigned int gpu_addr, unsigned int size);
void gsl_linux_map_free(unsigned int gpu_addr);
void *gsl_linux_map_find(unsigned int gpu_addr);
void *gsl_linux_map_read(void *dst, unsigned int gpuoffset, unsigned int sizebytes, unsigned int touserspace);
void *gsl_linux_map_write(void *src, unsigned int gpuoffset, unsigned int sizebytes, unsigned int fromuserspace);
void *gsl_linux_map_set(unsigned int gpuoffset, unsigned int value, unsigned int sizebytes);
int gsl_linux_map_destroy(void);

#endif
