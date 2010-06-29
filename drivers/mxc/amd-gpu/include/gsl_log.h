/* Copyright (c) 2002,2008-2009, Code Aurora Forum. All rights reserved.
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

#ifndef __GSL_LOG_H
#define __GSL_LOG_H

#define KGSL_LOG_GROUP_DRIVER       0x00000001
#define KGSL_LOG_GROUP_DEVICE       0x00000002
#define KGSL_LOG_GROUP_COMMAND      0x00000004
#define KGSL_LOG_GROUP_CONTEXT      0x00000008
#define KGSL_LOG_GROUP_MEMORY       0x00000010
#define KGSL_LOG_GROUP_ALL          0x000000ff

#define KGSL_LOG_LEVEL_ALL          0x0000ff00
#define KGSL_LOG_LEVEL_TRACE        0x00003f00
#define KGSL_LOG_LEVEL_DEBUG        0x00001f00
#define KGSL_LOG_LEVEL_INFO         0x00000f00
#define KGSL_LOG_LEVEL_WARN         0x00000700
#define KGSL_LOG_LEVEL_ERROR        0x00000300
#define KGSL_LOG_LEVEL_FATAL        0x00000100

#define KGSL_LOG_TIMESTAMP          0x00010000
#define KGSL_LOG_THREAD_ID          0x00020000
#define KGSL_LOG_PROCESS_ID         0x00040000

#ifdef GSL_LOG

int kgsl_log_init(void);
int kgsl_log_close(void);
int kgsl_log_open_stdout( unsigned int log_flags );
int kgsl_log_write( unsigned int log_flags, char* format, ... );
int kgsl_log_open_membuf( int* memBufId, unsigned int log_flags );
int kgsl_log_open_file( char* filename, unsigned int log_flags );
int kgsl_log_flush_membuf( char* filename, int memBufId );

#else

// Empty function definitions
OSINLINE int kgsl_log_init(void) { return GSL_SUCCESS; }
OSINLINE int kgsl_log_close(void) { return GSL_SUCCESS; }
OSINLINE int kgsl_log_open_stdout( unsigned int log_flags ) { (void)log_flags; return GSL_SUCCESS; }
OSINLINE int kgsl_log_write( unsigned int log_flags, char* format, ... ) { (void)log_flags; (void)format; return GSL_SUCCESS; }
OSINLINE int kgsl_log_open_membuf( int* memBufId, unsigned int log_flags ) { (void)memBufId; (void)log_flags; return GSL_SUCCESS; }
OSINLINE int kgsl_log_open_file( char* filename, unsigned int log_flags ) { (void)filename; (void)log_flags; return GSL_SUCCESS; }
OSINLINE int kgsl_log_flush_membuf( char* filename, int memBufId ) { (void) filename; (void) memBufId; return GSL_SUCCESS; }

#endif

#endif // __GSL_LOG_H
