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

#ifndef __GSL_DEBUG_H
#define __GSL_DEBUG_H

#ifdef BB_DUMPX
#include "dumpx.h"
#endif

#ifdef TBDUMP
#include "gsl_tbdump.h"
#endif


//////////////////////////////////////////////////////////////////////////////
//  macros
//////////////////////////////////////////////////////////////////////////////
#ifdef _DEBUG
#define KGSL_DEBUG(flag, action)                            if (gsl_driver.flags_debug & flag) {action;}
#ifdef GSL_BLD_YAMATO
#define KGSL_DEBUG_DUMPPM4(cmds, sizedwords)                Yamato_DumpPM4((cmds), (sizedwords))
#define KGSL_DEBUG_DUMPREGWRITE(addr, value)                Yamato_DumpRegisterWrite((addr), (value))
#define KGSL_DEBUG_DUMPMEMWRITE(addr, sizebytes, data)      Yamato_DumpWriteMemory(addr, sizebytes, data)
#define KGSL_DEBUG_DUMPMEMSET(addr, sizebytes, value)       Yamato_DumpSetMemory(addr, sizebytes, value)
#define KGSL_DEBUG_DUMPFBSTART(device)                      Yamato_DumpFbStart(device)
#define KGSL_DEBUG_DUMPREGSPACE(device)                     Yamato_DumpRegSpace(device)
#define KGSL_DEBUG_DUMPWINDOW(addr, width, height)          Yamato_DumpWindow(addr, width, height)
#else
#define KGSL_DEBUG_DUMPPM4(cmds, sizedwords)                
#define KGSL_DEBUG_DUMPREGWRITE(addr, value)                
#define KGSL_DEBUG_DUMPMEMWRITE(addr, sizebytes, data)      
#define KGSL_DEBUG_DUMPMEMSET(addr, sizebytes, value)       
#define KGSL_DEBUG_DUMPFBSTART(device)                      
#define KGSL_DEBUG_DUMPREGSPACE(device)
#define KGSL_DEBUG_DUMPWINDOW(addr, width, height)          
#endif
#ifdef TBDUMP

#define KGSL_DEBUG_TBDUMP_OPEN(filename)                    tbdump_open(filename)
#define KGSL_DEBUG_TBDUMP_CLOSE()                           tbdump_close()
#define KGSL_DEBUG_TBDUMP_SYNCMEM(addr, src, sizebytes)     tbdump_syncmem((unsigned int)addr, (unsigned int)src, sizebytes)
#define KGSL_DEBUG_TBDUMP_SETMEM(addr, value, sizebytes)    tbdump_setmem((unsigned int)addr, value, sizebytes)
#define KGSL_DEBUG_TBDUMP_SLAVEWRITE(addr, value)           tbdump_slavewrite(addr, value)
#define KGSL_DEBUG_TBDUMP_WAITIRQ()                         tbdump_waitirq()

#else
#define KGSL_DEBUG_TBDUMP_OPEN(file)
#define KGSL_DEBUG_TBDUMP_CLOSE()
#define KGSL_DEBUG_TBDUMP_SYNCMEM(addr, src, sizebytes)
#define KGSL_DEBUG_TBDUMP_SETMEM(addr, value, sizebytes)
#define KGSL_DEBUG_TBDUMP_SLAVEWRITE(addr, value)
#define KGSL_DEBUG_TBDUMP_WAITIRQ()
#endif
#ifdef BB_DUMPX
#define KGSL_DEBUG_DUMPX_OPEN(filename, param)              dumpx_open((filename), (param))
#define KGSL_DEBUG_DUMPX(cmd, par1, par2, par3, comment)    dumpx(cmd, (par1), (par2), (par3), (comment))
#define KGSL_DEBUG_DUMPX_CLOSE()                            dumpx_close()
#else
#define KGSL_DEBUG_DUMPX_OPEN(filename, param)
#define KGSL_DEBUG_DUMPX(cmd, par1, par2, par3, comment)
#define KGSL_DEBUG_DUMPX_CLOSE()
#endif
#else
#define KGSL_DEBUG(flag, action)
#define KGSL_DEBUG_DUMPPM4(cmds, sizedwords)
#define KGSL_DEBUG_DUMPREGWRITE(addr, value)
#define KGSL_DEBUG_DUMPMEMWRITE(addr, sizebytes, data)
#define KGSL_DEBUG_DUMPMEMSET(addr, sizebytes, value)
#define KGSL_DEBUG_DUMPFBSTART(device)
#define KGSL_DEBUG_DUMPREGSPACE(device)
#define KGSL_DEBUG_DUMPWINDOW(addr, width, height)
#define KGSL_DEBUG_DUMPX(cmd, par1, par2, par3, comment)

#define KGSL_DEBUG_TBDUMP_OPEN(file)
#define KGSL_DEBUG_TBDUMP_CLOSE()
#define KGSL_DEBUG_TBDUMP_SYNCMEM(addr, src, sizebytes)
#define KGSL_DEBUG_TBDUMP_SETMEM(addr, value, sizebytes)
#define KGSL_DEBUG_TBDUMP_SLAVEWRITE(addr, value)
#define KGSL_DEBUG_TBDUMP_WAITIRQ()
#endif // _DEBUG


//////////////////////////////////////////////////////////////////////////////
//  prototypes
//////////////////////////////////////////////////////////////////////////////
#ifdef GSL_BLD_YAMATO
void            Yamato_DumpPM4(unsigned int *cmds, unsigned int sizedwords);
void            Yamato_DumpRegisterWrite(unsigned int dwAddress, unsigned int value);
void            Yamato_DumpWriteMemory(unsigned int dwAddress, unsigned int dwSize, void* pData);
void            Yamato_DumpSetMemory(unsigned int dwAddress, unsigned int dwSize, unsigned int pData);
void            Yamato_DumpFbStart(gsl_device_t *device);
void            Yamato_DumpRegSpace(gsl_device_t *device);
#ifdef _WIN32
void            Yamato_DumpWindow(unsigned int addr, unsigned int width, unsigned int height);
#endif
#endif
#ifdef _DEBUG
int             kgsl_dumpx_parse_ibs(gpuaddr_t gpuaddr, int sizedwords);
#endif //_DEBUG
#endif // __GSL_DRIVER_H
