/* Copyright (c) 2002,2007-2009, Code Aurora Forum. All rights reserved.
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

#ifndef _YAMATO_H
#define _YAMATO_H

#ifndef qLittleEndian
#define qLittleEndian
#endif

#if defined(_YDX14)
#if defined(_WIN32) && !defined(__SYMBIAN32__)
#pragma message("YDX 14 header files\r\n")
#endif
#include "yamato/14/yamato_enum.h"
#include "yamato/14/yamato_ipt.h"
#include "yamato/14/yamato_mask.h"
#include "yamato/14/yamato_offset.h"
#include "yamato/14/yamato_registers.h"
#include "yamato/14/yamato_shift.h"
#include "yamato/14/yamato_struct.h"
#include "yamato/14/yamato_typedef.h"
#define _YAMATO_GENENUM_H           "reg/yamato/14/yamato_genenum.h"
#define _YAMATO_GENREG_H            "reg/yamato/14/yamato_genreg.h"
#else
#if defined(_WIN32) && !defined(__SYMBIAN32__)
#pragma message("YDX 22 header files\r\n")
#endif
#include "yamato/22/yamato_enum.h"
#include "yamato/22/yamato_ipt.h"
#include "yamato/22/yamato_mask.h"
#include "yamato/22/yamato_offset.h"
#include "yamato/22/yamato_registers.h"
#include "yamato/22/yamato_shift.h"
#include "yamato/22/yamato_struct.h"
#include "yamato/22/yamato_typedef.h"
#define _YAMATO_GENENUM_H           "reg/yamato/22/yamato_genenum.h"
#define _YAMATO_GENREG_H            "reg/yamato/22/yamato_genreg.h"
#endif

#endif // _YAMATO_H
