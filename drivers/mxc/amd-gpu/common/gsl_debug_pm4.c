/* Copyright (c) 2008-2010, Advanced Micro Devices. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include "gsl.h"
#include "gsl_hal.h"

#if defined(_WIN32) && defined (GSL_BLD_YAMATO)

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

//#define PM4_DEBUG_USE_MEMBUF

#ifdef PM4_DEBUG_USE_MEMBUF

#define MEMBUF_SIZE 100000
#define BUFFER_END_MARGIN 1000
char memBuf[MEMBUF_SIZE];
static int writePtr = 0;
static unsigned int lineNumber = 0;
//#define fprintf(A,...); writePtr += sprintf( memBuf+writePtr, __VA_ARGS__ ); sprintf( memBuf+writePtr, "###" ); if( writePtr > MEMBUF_SIZE-BUFFER_END_MARGIN ) { memset(memBuf+writePtr, '#', MEMBUF_SIZE-writePtr); writePtr = 0; }
#define FILE char
#define fopen(X,Y) 0
#define fclose(X)

int printString( FILE *_File, const char * _Format, ...)
{
    int ret;
    va_list ap;
    (void)_File;

    va_start(ap, _Format);
    if( writePtr > 0 && memBuf[writePtr-1] == '\n' )
    {
        // Add line number if last written character was newline
        writePtr += sprintf( memBuf+writePtr, "%d: ", lineNumber++ );
    }
    ret = vsprintf(memBuf+writePtr, _Format, ap);
    writePtr += ret;
    sprintf( memBuf+writePtr, "###" );
    if( writePtr > MEMBUF_SIZE-BUFFER_END_MARGIN ) 
    { 
        memset(memBuf+writePtr, '#', MEMBUF_SIZE-writePtr); 
        writePtr = 0;
    }

    va_end(ap);

    return ret;
}

#else

int printString( FILE *_File, const char * _Format, ...)
{
    int ret;
    va_list ap;
    va_start(ap, _Format);
    ret = vfprintf(_File, _Format, ap);
    va_end(ap);
    fflush(_File);
    return ret;
}

#endif

#ifndef    _WIN32_WCE
#define PM4_DUMPFILE    "pm4dump.txt"
#else
#define PM4_DUMPFILE    "\\Release\\pm4dump.txt"
#endif

//////////////////////////////////////////////////////////////////////////////
// defines
//////////////////////////////////////////////////////////////////////////////
#define EXPAND_OPCODE(opcode) ((opcode << 8) | PM4_PKT_MASK)

#define GetString_uint                       GetString_int
#define GetString_fixed12_4(val, szValue)    GetString_fixed(val, 12, 4, szValue)
#define GetString_signedint15(val, szValue)  GetString_signedint(val, 15, szValue)

// Need a prototype for this function
void WritePM4Packet_Type3(FILE* pFile, unsigned int dwHeader, unsigned int** ppBuffer);

static int indirectionLevel = 0;


//////////////////////////////////////////////////////////////////////////////
// functions
//////////////////////////////////////////////////////////////////////////////

void WriteDWORD(FILE* pFile, unsigned int dwValue)
{
    printString(pFile, "    0x%08x", dwValue);
}

void WriteDWORD2(FILE* pFile, unsigned int dwValue)
{
    printString(pFile, "    0x%08x\n", dwValue);
}

//----------------------------------------------------------------------------

// Generate the GetString_## functions for enumerated types
#define START_ENUMTYPE(__type)                      \
void GetString_##__type(unsigned int val, char* szValue)   \
{                                                   \
    switch(val)                                     \
    {

#define GENERATE_ENUM(__enumname, __val)            \
        case __val:                                 \
            kos_strcpy(szValue, #__enumname);        \
            break;

#define END_ENUMTYPE(__type)                        \
        default:                                    \
            sprintf(szValue, "Unknown: %d", val);   \
            break;                                  \
    }                                               \
}

#include _YAMATO_GENENUM_H

//----------------------------------------------------------------------------

void 
GetString_hex(unsigned int val, char* szValue)
{
    sprintf(szValue, "0x%x", val);
}

//----------------------------------------------------------------------------

void 
GetString_float(unsigned int val, char* szValue)
{
    float fval = *((float*) &val);
    sprintf(szValue, "%.4f", fval);
}

//----------------------------------------------------------------------------

void 
GetString_bool(unsigned int val, char* szValue)
{
    if (val)
    {
        kos_strcpy(szValue, "TRUE");
    }
    else
    {
        kos_strcpy(szValue, "FALSE");
    }
}

//----------------------------------------------------------------------------

void GetString_int(unsigned int val, char* szValue)
{
    sprintf(szValue, "%d", val);
}

//----------------------------------------------------------------------------

void 
GetString_intMinusOne(unsigned int val, char* szValue)
{
    sprintf(szValue, "%d+1", val);
}

//----------------------------------------------------------------------------

void 
GetString_signedint(unsigned int val, unsigned int dwNumBits, char* szValue)
{
    int nValue = val;

    if (val & (1<<(dwNumBits-1)))
    {
        nValue |= 0xffffffff << dwNumBits;
    }

    sprintf(szValue, "%d", nValue);
}

//----------------------------------------------------------------------------

void 
GetString_fixed(unsigned int val, unsigned int dwNumInt, unsigned int dwNumFrac, char* szValue)
{

   (void) dwNumInt;     // unreferenced formal parameter

    if (val>>dwNumFrac == 0)
    {
        // Integer part is 0 - just print out the fractional part
        sprintf(szValue, "%d/%d",
            val&((1<<dwNumFrac)-1),
            1<<dwNumFrac);
    }
    else
    {
        // Print out as a mixed fraction
        sprintf(szValue, "%d %d/%d",
            val>>dwNumFrac,
            val&((1<<dwNumFrac)-1),
            1<<dwNumFrac);
    }
}

//----------------------------------------------------------------------------

void 
GetString_Register(unsigned int dwBaseIndex, unsigned int dwValue, char* pszString)
{
    char szValue[64];
    char szField[128];

    // Empty the string
    pszString[0] = '\0';

    switch(dwBaseIndex)
    {
#define START_REGISTER(__reg)               \
        case mm##__reg:                     \
            {                               \
                reg##__reg reg;             \
                reg.u32All = dwValue;       \
                strcat(pszString, #__reg ", (");

#define GENERATE_FIELD(__name, __type)                                \
                GetString_##__type(reg.bitfields.__name, szValue);    \
                sprintf(szField, #__name " = %s, ", szValue);         \
                strcat(pszString, szField);

#define END_REGISTER(__reg)                           \
                pszString[strlen(pszString)-2]='\0';  \
                strcat(pszString, ")");               \
            }                                         \
            break;

#include _YAMATO_GENREG_H

        default:
            break;
    }
}

//----------------------------------------------------------------------------

void 
GetString_Type3Opcode(unsigned int opcode, char* pszValue)
{
switch(EXPAND_OPCODE(opcode))
    {
#define TYPE3SWITCH(__opcode)                  \
        case PM4_PACKET3_##__opcode:           \
            kos_strcpy(pszValue, #__opcode);    \
            break;

        TYPE3SWITCH(NOP)
        TYPE3SWITCH(IB_PREFETCH_END)
        TYPE3SWITCH(SUBBLK_PREFETCH)

        TYPE3SWITCH(INSTR_PREFETCH)
        TYPE3SWITCH(REG_RMW)
        TYPE3SWITCH(DRAW_INDX)
        TYPE3SWITCH(VIZ_QUERY)
        TYPE3SWITCH(SET_STATE)
        TYPE3SWITCH(WAIT_FOR_IDLE)
        TYPE3SWITCH(IM_LOAD)
        TYPE3SWITCH(IM_LOAD_IMMEDIATE)
        TYPE3SWITCH(SET_CONSTANT)
        TYPE3SWITCH(LOAD_CONSTANT_CONTEXT)
        TYPE3SWITCH(LOAD_ALU_CONSTANT)

        TYPE3SWITCH(DRAW_INDX_BIN)
        TYPE3SWITCH(3D_DRAW_INDX_2_BIN)
        TYPE3SWITCH(3D_DRAW_INDX_2)
        TYPE3SWITCH(INDIRECT_BUFFER_PFD)
        TYPE3SWITCH(INVALIDATE_STATE)
        TYPE3SWITCH(WAIT_REG_MEM)
        TYPE3SWITCH(MEM_WRITE)
        TYPE3SWITCH(REG_TO_MEM)
        TYPE3SWITCH(INDIRECT_BUFFER)

        TYPE3SWITCH(CP_INTERRUPT)
        TYPE3SWITCH(COND_EXEC)
        TYPE3SWITCH(COND_WRITE)
        TYPE3SWITCH(EVENT_WRITE)
        TYPE3SWITCH(INSTR_MATCH)
        TYPE3SWITCH(ME_INIT)
        TYPE3SWITCH(CONST_PREFETCH)
        TYPE3SWITCH(MEM_WRITE_CNTR)

        TYPE3SWITCH(SET_BIN_MASK)
        TYPE3SWITCH(SET_BIN_SELECT)
        TYPE3SWITCH(WAIT_REG_EQ)
        TYPE3SWITCH(WAIT_REG_GTE)
        TYPE3SWITCH(INCR_UPDT_STATE)
        TYPE3SWITCH(INCR_UPDT_CONST)
        TYPE3SWITCH(INCR_UPDT_INSTR)
        TYPE3SWITCH(EVENT_WRITE_SHD)
        TYPE3SWITCH(EVENT_WRITE_CFL)
        TYPE3SWITCH(EVENT_WRITE_ZPD)
        TYPE3SWITCH(WAIT_UNTIL_READ)
        TYPE3SWITCH(WAIT_IB_PFD_COMPLETE)
        TYPE3SWITCH(CONTEXT_UPDATE)

        default:
            sprintf(pszValue, "Unknown: %d", opcode);
            break;
    }
}

//----------------------------------------------------------------------------

void 
WritePM4Packet_Type0(FILE* pFile, unsigned int dwHeader, unsigned int** ppBuffer)
{
    pm4_type0 header = *((pm4_type0*) &dwHeader);
    unsigned int* pBuffer = *ppBuffer;
    unsigned int dwIndex;

    WriteDWORD(pFile, dwHeader);
    printString(pFile, "    // Type-0 packet (BASE_INDEX = 0x%x, ONE_REG_WR = %d, COUNT = %d+1)\n",
        header.base_index, header.one_reg_wr, header.count);   

    // Now go through and write the dwNumDWORDs 
    for (dwIndex = 0; dwIndex < header.count+1; dwIndex++)
    {
        char szRegister[1024];
        unsigned int dwRegIndex;
        unsigned int dwRegValue = *(pBuffer++);

        if (header.one_reg_wr)
        {
            dwRegIndex = header.base_index;
        }
        else
        {
            dwRegIndex = header.base_index + dwIndex;
        }

        WriteDWORD(pFile, dwRegValue);
        // Write register string based on fields
        GetString_Register(dwRegIndex, dwRegValue, szRegister);
        printString(pFile, "    // %s\n", szRegister);

        // Write actual unsigned int
        
    }

    *ppBuffer = pBuffer;
}

//----------------------------------------------------------------------------

void 
WritePM4Packet_Type2(FILE* pFile, unsigned int dwHeader, unsigned int** ppBuffer)
{
    unsigned int* pBuffer = *ppBuffer;

    WriteDWORD(pFile, dwHeader);
    printString(pFile, "    // Type-2 packet\n");
    
    *ppBuffer = pBuffer;
}

//----------------------------------------------------------------------------

void
AnalyzePacketType(FILE *pFile, unsigned int dwHeader, unsigned int**ppBuffer)
{
    switch(dwHeader & PM4_PKT_MASK)
    {
    case PM4_TYPE0_PKT:
        WritePM4Packet_Type0(pFile, dwHeader, ppBuffer);
        break;

    case PM4_TYPE1_PKT:
        break;

    case PM4_TYPE2_PKT:
        WritePM4Packet_Type2(pFile, dwHeader, ppBuffer);
        break;

    case PM4_TYPE3_PKT:
        WritePM4Packet_Type3(pFile, dwHeader, ppBuffer);
        break;
    }
}

void 
WritePM4Packet_Type3(FILE* pFile, unsigned int dwHeader, unsigned int** ppBuffer)
{
    pm4_type3 header = *((pm4_type3*) &dwHeader);
    unsigned int* pBuffer = *ppBuffer;
    unsigned int dwIndex;
    char szOpcode[64];

    if((EXPAND_OPCODE(header.it_opcode) == PM4_PACKET3_INDIRECT_BUFFER) ||
       (EXPAND_OPCODE(header.it_opcode) == PM4_PACKET3_INDIRECT_BUFFER_PFD))
    {
        unsigned int *pIndirectBuffer = (unsigned int *) *(pBuffer++); // ordinal 2 of IB packet is an address
        unsigned int *pIndirectBufferEnd = pIndirectBuffer + *(pBuffer++); // ordinal 3 of IB packet is size
        unsigned int gpuaddr = kgsl_sharedmem_convertaddr((unsigned int) pIndirectBuffer, 1);

        indirectionLevel++;

        WriteDWORD2(pFile, dwHeader);
        WriteDWORD2(pFile, gpuaddr);
        WriteDWORD2(pFile, (unsigned int) (pIndirectBufferEnd-pIndirectBuffer));

        if (indirectionLevel == 1)
        {
            printString(pFile, "Start_IB1, base=0x%x, size=%d\n", gpuaddr, (unsigned int)(pIndirectBufferEnd - pIndirectBuffer));
        }
        else
        {
            printString(pFile, "Start_IB2, base=0x%x, size=%d\n", gpuaddr, (unsigned int)(pIndirectBufferEnd - pIndirectBuffer));
        }

        while(pIndirectBuffer < pIndirectBufferEnd)
        {
            unsigned int _dwHeader = *(pIndirectBuffer++);

            AnalyzePacketType(pFile, _dwHeader, &pIndirectBuffer);
        }

        if (indirectionLevel == 1)
        {
            printString(pFile, "End_IB1\n");
        }
        else
        {
            printString(pFile, "End_IB2\n");
        }

        indirectionLevel--;
    }
    else
    {
        unsigned int registerAddr = 0xffffffff;
        char szRegister[1024];

        GetString_Type3Opcode(header.it_opcode, szOpcode);

        WriteDWORD(pFile, dwHeader);
        printString(pFile, "    // Type-3 packet (PREDICATE = %d, IT_OPCODE = %s, COUNT = %d+1)\n",
            header.predicate, szOpcode, header.count);
        
        // Go through each command
        for (dwIndex = 0; dwIndex < header.count+1; dwIndex++)
        {
            // Check for a register write
            if((EXPAND_OPCODE(header.it_opcode) == PM4_PACKET3_SET_CONSTANT) && (((*pBuffer) >> 16) == 0x4))
                registerAddr = (*pBuffer) & 0xffff;

            // Write unsigned int
            WriteDWORD(pFile, *pBuffer);

            // Starting at Ordinal 2 is actual register values
            if((dwIndex > 0) && (registerAddr != 0xffffffff))
            {
                // Write register string based on address
                GetString_Register(registerAddr + 0x2000, *pBuffer, szRegister);
                printString(pFile, "    // %s\n", szRegister);
                registerAddr++;
            }
            else
            {
                // Write out newline if we aren't augmenting with register fields
                printString(pFile, "\n");
            }

            pBuffer++;
        }
    }
    *ppBuffer = pBuffer;
}

//----------------------------------------------------------------------------

void
Yamato_DumpInitParams(unsigned int dwEDRAMBase, unsigned int dwEDRAMSize)
{
    FILE* pFile = fopen(PM4_DUMPFILE, "a");

    printString(pFile, "InitParams, edrambase=0x%x, edramsize=%d\n",
        dwEDRAMBase, dwEDRAMSize);

    fclose(pFile);
}

//----------------------------------------------------------------------------

void
Yamato_DumpSwapBuffers(unsigned int dwAddress, unsigned int dwWidth,
    unsigned int dwHeight, unsigned int dwPitch, unsigned int dwAlignedHeight, unsigned int dwBitsPerPixel)
{
    // Open file
    FILE* pFile = fopen(PM4_DUMPFILE, "a");

    printString(pFile, "SwapBuffers, address=0x%08x, width=%d, height=%d, pitch=%d, alignedheight=%d, bpp=%d\n",
        dwAddress, dwWidth, dwHeight, dwPitch, dwAlignedHeight, dwBitsPerPixel);

    fclose(pFile);
}

//----------------------------------------------------------------------------

void
Yamato_DumpRegSpace(gsl_device_t *device)
{
    int           regsPerLine = 0x20;
    unsigned int  dwOffset;
    unsigned int  value;

    FILE* pFile = fopen(PM4_DUMPFILE, "a");

    printString(pFile, "Start_RegisterSpace\n");
    
    for (dwOffset = 0; dwOffset < device->regspace.sizebytes; dwOffset += 4)
    {
        if (dwOffset % regsPerLine == 0)
        {
           printString(pFile, "    0x%08x   ", dwOffset);
        }

        GSL_HAL_REG_READ(device->id, (unsigned int) device->regspace.mmio_virt_base, (dwOffset >> 2), &value);

        printString(pFile, " 0x%08x", value);

        if (((dwOffset + 4) % regsPerLine == 0) && ((dwOffset + 4) < device->regspace.sizebytes))
        {
           printString(pFile, "\n");
        }
    }

    printString(pFile, "\nEnd_RegisterSpace\n");

    fclose(pFile);
}

//----------------------------------------------------------------------------

void 
Yamato_DumpAllocateMemory(unsigned int dwSize, unsigned int dwFlags, unsigned int dwAddress,
    unsigned int dwActualSize)
{
    // Open file
    FILE* pFile = fopen(PM4_DUMPFILE, "a");

    printString(pFile, "AllocateMemory, size=%d, flags=0x%x, address=0x%x, actualSize=%d\n",
        dwSize, dwFlags, dwAddress, dwActualSize);

    fclose(pFile);
}

//----------------------------------------------------------------------------

void
Yamato_DumpFreeMemory(unsigned int dwAddress)
{
    // Open file
    FILE* pFile = fopen(PM4_DUMPFILE, "a");

    printString(pFile, "FreeMemory, address=0x%x\n", dwAddress);

    fclose(pFile);
}

//----------------------------------------------------------------------------

void 
Yamato_DumpWriteMemory(unsigned int dwAddress, unsigned int dwSize, void* pData)
{
    // Open file
    FILE* pFile = fopen(PM4_DUMPFILE, "a");
    unsigned int dwNumDWORDs;
    unsigned int dwIndex;
    unsigned int *pDataPtr;

    printString(pFile, "StartWriteMemory, address=0x%x, size=%d\n", dwAddress, dwSize);

    // Now write the data, in dwNumDWORDs
    dwNumDWORDs = dwSize >> 2;

    // If there are spillover bytes into the next dword, increment the amount dumped out here.
    // The reader needs to take care of not overwriting the nonvalid bytes
    if((dwSize % 4) != 0)
        dwNumDWORDs++;

    for (dwIndex = 0, pDataPtr = (unsigned int *)pData; dwIndex < dwNumDWORDs; dwIndex++, pDataPtr++)
    {
        WriteDWORD2(pFile, *pDataPtr);
    }

    printString(pFile, "EndWriteMemory\n");

    fclose(pFile);
}

void 
Yamato_DumpSetMemory(unsigned int dwAddress, unsigned int dwSize, unsigned int pData)
{
    // Open file
    FILE* pFile = fopen(PM4_DUMPFILE, "a");
//    unsigned int* pDataPtr;

    printString(pFile, "SetMemory, address=0x%x, size=%d, value=0x%x\n",
        dwAddress, dwSize, pData);

    fclose(pFile);
}

//----------------------------------------------------------------------------
void 
Yamato_ConvertIBAddr(unsigned int dwHeader, unsigned int *pBuffer, int gpuToHost)
{
    unsigned int hostaddr;
    unsigned int *ibend;
    unsigned int *addr;
    unsigned int *ib    = pBuffer;
    pm4_type3    header = *((pm4_type3*) &dwHeader);

    // convert ib1 base address
    if((EXPAND_OPCODE(header.it_opcode) == PM4_PACKET3_INDIRECT_BUFFER) ||
       (EXPAND_OPCODE(header.it_opcode) == PM4_PACKET3_INDIRECT_BUFFER_PFD))
    {
        if (gpuToHost)
        {
            // from gpu to host
            *ib = kgsl_sharedmem_convertaddr(*ib, 0);

            hostaddr = *ib;
        }
        else
        {
            // from host to gpu
            hostaddr = *ib;
            *ib = kgsl_sharedmem_convertaddr(*ib, 1);
        }

        // walk through ib1 and convert any ib2 base address

        ib    = (unsigned int *) hostaddr;
        ibend = (unsigned int *) (ib + *(++pBuffer));

        while (ib < ibend)
        {
            dwHeader = *(ib);
            header   = *((pm4_type3*) (&dwHeader));

            switch(dwHeader & PM4_PKT_MASK)
            {
            case PM4_TYPE0_PKT:
                ib += header.count + 2;
                break;

            case PM4_TYPE1_PKT:
                break;

            case PM4_TYPE2_PKT:
                ib++;
                break;

            case PM4_TYPE3_PKT:
                if((EXPAND_OPCODE(header.it_opcode) == PM4_PACKET3_INDIRECT_BUFFER) ||
                   (EXPAND_OPCODE(header.it_opcode) == PM4_PACKET3_INDIRECT_BUFFER_PFD))
                {
                    addr = ib + 1;
                    if (gpuToHost)
                    {
                        // from gpu to host
                        *addr = kgsl_sharedmem_convertaddr(*addr, 0);
                    }
                    else
                    {
                        // from host to gpu
                        *addr = kgsl_sharedmem_convertaddr(*addr, 1);
                    }
                }
                ib += header.count + 2;
                break;
            }
        }
    }
}

//----------------------------------------------------------------------------

void
Yamato_DumpPM4(unsigned int* pBuffer, unsigned int sizeDWords)
{
    unsigned int *pBufferEnd = pBuffer + sizeDWords;
    unsigned int *tmp;

    // Open file
    FILE* pFile = fopen(PM4_DUMPFILE, "a");

    printString(pFile, "Start_PM4Buffer\n");//, count=%d\n", sizeDWords);

    // So look at the first unsigned int - should be a header
    while(pBuffer < pBufferEnd)
    {
        unsigned int dwHeader = *(pBuffer++);

        //printString(pFile, "  Start_Packet\n");
        switch(dwHeader & PM4_PKT_MASK)
        {
            case PM4_TYPE0_PKT:
                WritePM4Packet_Type0(pFile, dwHeader, &pBuffer);
                break;

            case PM4_TYPE1_PKT:
                break;

            case PM4_TYPE2_PKT:
                WritePM4Packet_Type2(pFile, dwHeader, &pBuffer);
                break;

            case PM4_TYPE3_PKT:
                indirectionLevel = 0;
                tmp = pBuffer;
                Yamato_ConvertIBAddr(dwHeader, tmp, 1);
                WritePM4Packet_Type3(pFile, dwHeader, &pBuffer);
                Yamato_ConvertIBAddr(dwHeader, tmp, 0);
                break;
        }
        //printString(pFile, "  End_Packet\n");
    }

    printString(pFile, "End_PM4Buffer\n");
    fclose(pFile);
}

//----------------------------------------------------------------------------

void
Yamato_DumpRegisterWrite(unsigned int dwAddress, unsigned int value)
{
    FILE *pFile;

    // Build a Type-0 packet that maps to this register write
    unsigned int pBuffer[100], *pBuf = &pBuffer[1];

    // Don't dump CP_RB_WPTR (switch statement may be necessary here for future additions)
    if(dwAddress == mmCP_RB_WPTR)
        return;

    pFile = fopen(PM4_DUMPFILE, "a");

    pBuffer[0] = dwAddress;
    pBuffer[1] = value;

    printString(pFile, "StartRegisterWrite\n");
    WritePM4Packet_Type0(pFile, pBuffer[0], &pBuf);
    printString(pFile, "EndRegisterWrite\n");

    fclose(pFile);
}

//----------------------------------------------------------------------------

void
Yamato_DumpFbStart(gsl_device_t *device)
{
    FILE *pFile;

    static int firstCall = 0;

    // We only want to call this once
    if(firstCall)
        return;

    pFile = fopen(PM4_DUMPFILE, "a");

    printString(pFile, "FbStart, value=0x%x\n", device->mmu.mpu_base);
    printString(pFile, "FbSize, value=0x%x\n", device->mmu.mpu_range);

    fclose(pFile);

    firstCall = 1;
}

//----------------------------------------------------------------------------

void
Yamato_DumpWindow(unsigned int addr, unsigned int width, unsigned int height)
{
    FILE *pFile;

    pFile = fopen(PM4_DUMPFILE, "a");

    printString(pFile, "DumpWindow, addr=0x%x, width=0x%x, height=0x%x\n", addr, width, height);

    fclose(pFile);
}

//----------------------------------------------------------------------------
#ifdef _DEBUG

#define ADDRESS_STACK_SIZE 256
#define GET_PM4_TYPE3_OPCODE(x) ((*(x) >> 8) & 0xFF)
#define IF_REGISTER_IN_RANGE(reg, base, count)         \
    offset = (reg) - (base);                           \
    if(offset >= 0 && offset <= (count) - 2)
#define GET_CP_CONSTANT_DATA(x) (*((x) + offset + 2))

static const char format2bpp[] =
{
    2,  // COLORX_4_4_4_4 
    2,  // COLORX_1_5_5_5 
    2,  // COLORX_5_6_5 
    1,  // COLORX_8 
    2,  // COLORX_8_8 
    4,  // COLORX_8_8_8_8 
    4,  // COLORX_S8_8_8_8 
    2,  // COLORX_16_FLOAT 
    4,  // COLORX_16_16_FLOAT 
    8,  // COLORX_16_16_16_16_FLOAT 
    4,  // COLORX_32_FLOAT 
    8,  // COLORX_32_32_FLOAT 
    16, // COLORX_32_32_32_32_FLOAT ,
    1,  // COLORX_2_3_3 
    3,  // COLORX_8_8_8
};

static unsigned int kgsl_dumpx_addr_count = 0; //unique command buffer addresses encountered
static int kgsl_dumpx_handle_type3(unsigned int* hostaddr, int count)
{
    // For swap detection we need to find the below declared static values, and detect DI during EDRAM copy
    static unsigned int width = 0, height = 0, format = 0, baseaddr = 0, iscopy = 0;

    static unsigned int addr_stack[ADDRESS_STACK_SIZE];
    static unsigned int size_stack[ADDRESS_STACK_SIZE];
    int swap = 0; // have we encountered a swap during recursion (return value)

    switch(GET_PM4_TYPE3_OPCODE(hostaddr))
    {
        case PM4_INDIRECT_BUFFER_PFD:
        case PM4_INDIRECT_BUFFER:
        {
            // traverse indirect buffers
            unsigned int i;
            unsigned int ibaddr = *(hostaddr+1);
            unsigned int ibsize = *(hostaddr+2);
            
            // is this address already in encountered?
            for(i = 0; i < kgsl_dumpx_addr_count && addr_stack[i] != ibaddr; i++);
            
            if(kgsl_dumpx_addr_count == i)
            {
                // yes it was, store the address so we don't dump this buffer twice
                addr_stack[kgsl_dumpx_addr_count] = ibaddr;
                // just for sanity checking
                size_stack[kgsl_dumpx_addr_count++] = ibsize; 
                KOS_ASSERT(kgsl_dumpx_addr_count < ADDRESS_STACK_SIZE);

                // recursively follow the indirect link and update swap if indirect buffer had resolve
                swap |= kgsl_dumpx_parse_ibs(ibaddr, ibsize); 
            }
            else
            {
                KOS_ASSERT(size_stack[i] == ibsize);
            }
        } 
        break;

        case PM4_SET_CONSTANT:
        if((*(hostaddr+1) >> 16) == 0x4)
        {   
            // parse register writes, and figure out framebuffer configuration

            unsigned int regaddr = (*(hostaddr + 1) & 0xFFFF) + 0x2000; //dword address in register space
            int offset; // used by the macros

            IF_REGISTER_IN_RANGE(mmPA_SC_WINDOW_SCISSOR_BR, regaddr, count)
            {
                // found write to PA_SC_WINDOW_SCISSOR_BR, we use this to detect current
                // width and height of the framebuffer (TODO: find more reliable way of achieving this)
                unsigned int data = GET_CP_CONSTANT_DATA(hostaddr);
                width  = data & 0xFFFF;
                height = data >> 16;
            }
            
            IF_REGISTER_IN_RANGE(mmRB_MODECONTROL, regaddr, count)
            {
                // found write to RB_MODECONTROL, we use this to find out if next DI is resolve
                unsigned int data = GET_CP_CONSTANT_DATA(hostaddr);
                iscopy = (data & RB_MODECONTROL__EDRAM_MODE_MASK) == (EDRAM_COPY << RB_MODECONTROL__EDRAM_MODE__SHIFT);
            }

            IF_REGISTER_IN_RANGE(mmRB_COPY_DEST_BASE, regaddr, count)
            {
                // found write to RB_COPY_DEST_BASE, we use this to find out the framebuffer base address
                unsigned int data = GET_CP_CONSTANT_DATA(hostaddr);
                baseaddr = (data & RB_COPY_DEST_BASE__COPY_DEST_BASE_MASK);
            }

            IF_REGISTER_IN_RANGE(mmRB_COPY_DEST_INFO, regaddr, count)
            {
               // found write to RB_COPY_DEST_INFO, we use this to find out the framebuffer format
                unsigned int data = GET_CP_CONSTANT_DATA(hostaddr);
                format = (data & RB_COPY_DEST_INFO__COPY_DEST_FORMAT_MASK) >> RB_COPY_DEST_INFO__COPY_DEST_FORMAT__SHIFT;
            }             
        }
        break;

        case PM4_DRAW_INDX:
        case PM4_DRAW_INDX_2:
        {
            // DI found
            // check if it is resolve
            if(iscopy && !swap)
            {
                // printf("resolve: %ix%i @ 0x%08x, format = 0x%08x\n", width, height, baseaddr, format);
                KOS_ASSERT(format < 15);

                // yes it was and we need to update color buffer config because this is the first bin
                // dumpx framebuffer base address, and dimensions
                KGSL_DEBUG_DUMPX( BB_DUMP_CBUF_AWH, (unsigned int)baseaddr, width, height, " ");

                // find aligned width
                width = (width + 31) & ~31;

                //dump bytes-per-pixel and aligned width
                KGSL_DEBUG_DUMPX( BB_DUMP_CBUF_FS, format2bpp[format], width, 0, " ");
                swap = 1;
            }
            
        }
        break;

        default:
        break;
    }
    return swap;
}
           
// Traverse IBs and dump them to test vector. Detect swap by inspecting register 
// writes, keeping note of the current state, and dump framebuffer config to test vector 
int kgsl_dumpx_parse_ibs(gpuaddr_t gpuaddr, int sizedwords)
{
    static unsigned int level = 0; //recursion level

    int swap = 0; // have we encountered a swap during recursion (return value)
    unsigned int *hostaddr;
    int dwords_left = sizedwords; //dwords left in the current command buffer

    level++;

    KOS_ASSERT(sizeof(unsigned int *) == sizeof(unsigned int));
    KOS_ASSERT(level <= 2);
    hostaddr = (unsigned int *)kgsl_sharedmem_convertaddr(gpuaddr, 0);    

    // dump the IB to test vector
    KGSL_DEBUG(GSL_DBGFLAGS_DUMPX, KGSL_DEBUG_DUMPX(BB_DUMP_MEMWRITE, gpuaddr, (unsigned int)hostaddr, sizedwords*4, "kgsl_dumpx_write_ibs"));

    while(dwords_left)
    {
        int count = 0; //dword count including packet header

        switch(*hostaddr >> 30)
        {
        case 0x0: // type-0
            count = (*hostaddr >> 16)+2;
            break;
        case 0x1: // type-1
            count = 2;
            break;
        case 0x3: // type-3
            count = ((*hostaddr >> 16) & 0x3fff) + 2;
            swap |= kgsl_dumpx_handle_type3(hostaddr, count);
            break; // type-3
        default:
            KOS_ASSERT(!"unknown packet type");
        }
        
        // jump to next packet
        dwords_left -= count;
        hostaddr += count;
        KOS_ASSERT(dwords_left >= 0 && "PM4 parsing error");
    }

    level--;

    // if this is the starting level of recursion, we are done. clean-up
    if(level == 0) kgsl_dumpx_addr_count = 0;

    return swap;
}
#endif

#endif // WIN32

