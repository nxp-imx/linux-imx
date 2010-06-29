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

#ifndef __GSL_HALCONFIG_H
#define __GSL_HALCONFIG_H



#define GSL_HAL_PLATFORM                "i.MX35G"


#define GSL_HAL_GPUBASE_GMEM            0x00100000          // 1MB
#define GSL_HAL_GPUBASE_GMEM_PHYS       0x20000000          // 1MB

#define GSL_HAL_GPUBASE_REG_YDX         0x30000000
#define GSL_HAL_GPUBASE_REG_G12         0x20000000

#define GSL_HAL_SIZE_REG_YDX            0x00020000            // 128KB
#define GSL_HAL_SIZE_REG_G12            0x00001000            // 4KB
#define GSL_HAL_SIZE_GMEM               0x00040000          // 256KB - 0 to 384KB in 128KB increments

#if defined(_LINUX) && defined(GSL_MMU_TRANSLATION_ENABLED)
#define GSL_HAL_SHMEM_SIZE_EMEM1        0x02400000            // 36MB
#define GSL_HAL_SHMEM_SIZE_EMEM2        0x00400000            // 4MB
#define GSL_HAL_SHMEM_SIZE_PHYS         0x00400000            // 4MB
#elif defined(_LINUX) //MX35 Linux can able to allocate only 4MB
#define GSL_HAL_SHMEM_SIZE_EMEM1        0x00400000          // 4MB
#define GSL_HAL_SHMEM_SIZE_EMEM2        0x00200000          // 2MB
#define GSL_HAL_SHMEM_SIZE_PHYS         0x00200000          // 2MB
#else //Not possible to allocate 24 MB on WinCE
#define GSL_HAL_SHMEM_SIZE_EMEM1        0x00D00000          // 13MB
#define GSL_HAL_SHMEM_SIZE_EMEM2        0x00200000          // 2MB
#define GSL_HAL_SHMEM_SIZE_PHYS         0x00100000          // 1MB
#endif

#define MX35_G12_INTERRUPT              16

#endif  // __GSL_HALCONFIG_H
