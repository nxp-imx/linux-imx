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

#ifndef __GSL_TYPES_H
#define __GSL_TYPES_H

#include "stddef.h"


//////////////////////////////////////////////////////////////////////////////
// status
//////////////////////////////////////////////////////////////////////////////
#define GSL_SUCCESS                     OS_SUCCESS                  
#define GSL_FAILURE                     OS_FAILURE                  
#define GSL_FAILURE_SYSTEMERROR         OS_FAILURE_SYSTEMERROR      
#define GSL_FAILURE_DEVICEERROR         OS_FAILURE_DEVICEERROR  
#define GSL_FAILURE_OUTOFMEM            OS_FAILURE_OUTOFMEM         
#define GSL_FAILURE_BADPARAM            OS_FAILURE_BADPARAM         
#define GSL_FAILURE_OFFSETINVALID       OS_FAILURE_OFFSETINVALID
#define GSL_FAILURE_NOTSUPPORTED        OS_FAILURE_NOTSUPPORTED     
#define GSL_FAILURE_NOMOREAVAILABLE     OS_FAILURE_NOMOREAVAILABLE  
#define GSL_FAILURE_NOTINITIALIZED      OS_FAILURE_NOTINITIALIZED 
#define GSL_FAILURE_ALREADYINITIALIZED  OS_FAILURE_ALREADYINITIALIZED
#define GSL_FAILURE_TIMEOUT             OS_FAILURE_TIMEOUT


//////////////////////////////////////////////////////////////////////////////
// memory allocation flags
//////////////////////////////////////////////////////////////////////////////
#define GSL_MEMFLAGS_ANY                0x00000000      // dont care

#define GSL_MEMFLAGS_CHANNELANY         0x00000000
#define GSL_MEMFLAGS_CHANNEL1           0x00000000
#define GSL_MEMFLAGS_CHANNEL2           0x00000001
#define GSL_MEMFLAGS_CHANNEL3           0x00000002
#define GSL_MEMFLAGS_CHANNEL4           0x00000003
                                        
#define GSL_MEMFLAGS_BANKANY            0x00000000
#define GSL_MEMFLAGS_BANK1              0x00000010
#define GSL_MEMFLAGS_BANK2              0x00000020
#define GSL_MEMFLAGS_BANK3              0x00000040
#define GSL_MEMFLAGS_BANK4              0x00000080
                                        
#define GSL_MEMFLAGS_DIRANY             0x00000000
#define GSL_MEMFLAGS_DIRTOP             0x00000100
#define GSL_MEMFLAGS_DIRBOT             0x00000200
                                        
#define GSL_MEMFLAGS_APERTUREANY        0x00000000
#define GSL_MEMFLAGS_EMEM               0x00000000
#define GSL_MEMFLAGS_CONPHYS            0x00001000
                                        
#define GSL_MEMFLAGS_ALIGNANY           0x00000000      // minimum alignment is 32 bytes 
#define GSL_MEMFLAGS_ALIGN32            0x00000000
#define GSL_MEMFLAGS_ALIGN64            0x00060000
#define GSL_MEMFLAGS_ALIGN128           0x00070000
#define GSL_MEMFLAGS_ALIGN256           0x00080000
#define GSL_MEMFLAGS_ALIGN512           0x00090000
#define GSL_MEMFLAGS_ALIGN1K            0x000A0000
#define GSL_MEMFLAGS_ALIGN2K            0x000B0000
#define GSL_MEMFLAGS_ALIGN4K            0x000C0000
#define GSL_MEMFLAGS_ALIGN8K            0x000D0000
#define GSL_MEMFLAGS_ALIGN16K           0x000E0000
#define GSL_MEMFLAGS_ALIGN32K           0x000F0000
#define GSL_MEMFLAGS_ALIGN64K           0x00100000
#define GSL_MEMFLAGS_ALIGNPAGE          GSL_MEMFLAGS_ALIGN4K

#define GSL_MEMFLAGS_GPUREADWRITE       0x00000000
#define GSL_MEMFLAGS_GPUREADONLY        0x01000000
#define GSL_MEMFLAGS_GPUWRITEONLY       0x02000000
#define GSL_MEMFLAGS_GPUNOACCESS        0x04000000

#define GSL_MEMFLAGS_FORCEPAGESIZE      0x40000000
#define GSL_MEMFLAGS_STRICTREQUEST      0x80000000      // fail the alloc if the flags cannot be honored 
                    
#define GSL_MEMFLAGS_CHANNEL_MASK       0x0000000F
#define GSL_MEMFLAGS_BANK_MASK          0x000000F0
#define GSL_MEMFLAGS_DIR_MASK           0x00000F00
#define GSL_MEMFLAGS_APERTURE_MASK      0x0000F000
#define GSL_MEMFLAGS_ALIGN_MASK         0x00FF0000
#define GSL_MEMFLAGS_GPUAP_MASK         0x0F000000

#define GSL_MEMFLAGS_CHANNEL_SHIFT      0
#define GSL_MEMFLAGS_BANK_SHIFT         4
#define GSL_MEMFLAGS_DIR_SHIFT          8
#define GSL_MEMFLAGS_APERTURE_SHIFT     12
#define GSL_MEMFLAGS_ALIGN_SHIFT        16
#define GSL_MEMFLAGS_GPUAP_SHIFT        24


//////////////////////////////////////////////////////////////////////////////
// debug flags
//////////////////////////////////////////////////////////////////////////////
#define GSL_DBGFLAGS_ALL                0xFFFFFFFF
#define GSL_DBGFLAGS_DEVICE             0x00000001
#define GSL_DBGFLAGS_CTXT               0x00000002
#define GSL_DBGFLAGS_MEMMGR             0x00000004
#define GSL_DBGFLAGS_MMU                0x00000008
#define GSL_DBGFLAGS_POWER              0x00000010
#define GSL_DBGFLAGS_IRQ                0x00000020
#define GSL_DBGFLAGS_BIST               0x00000040
#define GSL_DBGFLAGS_PM4                0x00000080
#define GSL_DBGFLAGS_PM4MEM             0x00000100
#define GSL_DBGFLAGS_PM4CHECK           0x00000200
#define GSL_DBGFLAGS_DUMPX              0x00000400
#define GSL_DBGFLAGS_DUMPX_WITHOUT_IFH  0x00000800
#define GSL_DBGFLAGS_IFH                0x00001000
#define GSL_DBGFLAGS_NULL               0x00002000


//////////////////////////////////////////////////////////////////////////////
// generic flag values
//////////////////////////////////////////////////////////////////////////////
#define GSL_FLAGS_NORMALMODE            0x00000000
#define GSL_FLAGS_SAFEMODE              0x00000001
#define GSL_FLAGS_INITIALIZED0          0x00000002
#define GSL_FLAGS_INITIALIZED           0x00000004
#define GSL_FLAGS_STARTED               0x00000008
#define GSL_FLAGS_ACTIVE                0x00000010
#define GSL_FLAGS_RESERVED0             0x00000020
#define GSL_FLAGS_RESERVED1             0x00000040
#define GSL_FLAGS_RESERVED2             0x00000080


//////////////////////////////////////////////////////////////////////////////
// power flags
//////////////////////////////////////////////////////////////////////////////
#define GSL_PWRFLAGS_POWER_OFF          0x00000001
#define GSL_PWRFLAGS_POWER_ON           0x00000002
#define GSL_PWRFLAGS_CLK_ON             0x00000004
#define GSL_PWRFLAGS_CLK_OFF            0x00000008
#define GSL_PWRFLAGS_OVERRIDE_ON        0x00000010
#define GSL_PWRFLAGS_OVERRIDE_OFF       0x00000020

//////////////////////////////////////////////////////////////////////////////
// DMI flags
//////////////////////////////////////////////////////////////////////////////
#define GSL_DMIFLAGS_ENABLE_SINGLE      0x00000001  //  Single buffered DMI
#define GSL_DMIFLAGS_ENABLE_DOUBLE      0x00000002  //  Double buffered DMI
#define GSL_DMIFLAGS_ENABLE_TRIPLE      0x00000004  //  Triple buffered DMI
#define GSL_DMIFLAGS_DISABLE            0x00000008
#define GSL_DMIFLAGS_NEXT_BUFFER        0x00000010

//////////////////////////////////////////////////////////////////////////////
// cache flags
//////////////////////////////////////////////////////////////////////////////
#define GSL_CACHEFLAGS_CLEAN            0x00000001  /* flush cache          */
#define GSL_CACHEFLAGS_INVALIDATE       0x00000002  /* invalidate cache     */
#define GSL_CACHEFLAGS_WRITECLEAN       0x00000004  /* flush write cache    */


//////////////////////////////////////////////////////////////////////////////
//  context
//////////////////////////////////////////////////////////////////////////////
#define GSL_CONTEXT_MAX             20
#define GSL_CONTEXT_NONE            0
#define GSL_CONTEXT_SAVE_GMEM       1
#define GSL_CONTEXT_NO_GMEM_ALLOC   2


//////////////////////////////////////////////////////////////////////////////
// other
//////////////////////////////////////////////////////////////////////////////
#define GSL_TIMEOUT_NONE                        0
#define GSL_TIMEOUT_DEFAULT                     0xFFFFFFFF

#define GSL_PAGESIZE                            0x1000
#define GSL_PAGESIZE_SHIFT                      12

#define GSL_TIMESTAMP_EPSILON           20000

//////////////////////////////////////////////////////////////////////////////
// types
//////////////////////////////////////////////////////////////////////////////
typedef unsigned int        gsl_devhandle_t;
typedef unsigned int        gsl_ctxthandle_t;
typedef int                 gsl_timestamp_t;
typedef unsigned int        gsl_flags_t;
typedef unsigned int        gpuaddr_t;

// ---------
// device id
// ---------
typedef enum _gsl_deviceid_t
{
    GSL_DEVICE_ANY    = 0,
    GSL_DEVICE_YAMATO = 1,
    GSL_DEVICE_G12    = 2,
    GSL_DEVICE_MAX    = 2,

    GSL_DEVICE_FOOBAR = 0x7FFFFFFF
} gsl_deviceid_t;

// ----------------
// chip revision id
// ----------------
//
// coreid:8 majorrev:8 minorrev:8 patch:8
// 
// coreid = 0x00 = YAMATO_DX
// coreid = 0x80 = G12
//

#define COREID(x)   ((((unsigned int)x & 0xFF) << 24))
#define MAJORID(x)  ((((unsigned int)x & 0xFF) << 16))
#define MINORID(x)  ((((unsigned int)x & 0xFF) <<  8))
#define PATCHID(x)  ((((unsigned int)x & 0xFF) <<  0))

typedef enum _gsl_chipid_t
{
    GSL_CHIPID_YAMATODX_REV13   = (COREID(0x00) | MAJORID(0x01) | MINORID(0x03) | PATCHID(0x00)),
    GSL_CHIPID_YAMATODX_REV14   = (COREID(0x00) | MAJORID(0x01) | MINORID(0x04) | PATCHID(0x00)),
    GSL_CHIPID_YAMATODX_REV20   = (COREID(0x00) | MAJORID(0x02) | MINORID(0x00) | PATCHID(0x00)),
    GSL_CHIPID_YAMATODX_REV21   = (COREID(0x00) | MAJORID(0x02) | MINORID(0x01) | PATCHID(0x00)),
    GSL_CHIPID_YAMATODX_REV211  = (COREID(0x00) | MAJORID(0x02) | MINORID(0x01) | PATCHID(0x01)),
    GSL_CHIPID_YAMATODX_REV22   = (COREID(0x00) | MAJORID(0x02) | MINORID(0x02) | PATCHID(0x00)),
    GSL_CHIPID_YAMATODX_REV23   = (COREID(0x00) | MAJORID(0x02) | MINORID(0x03) | PATCHID(0x00)),
    GSL_CHIPID_YAMATODX_REV231  = (COREID(0x00) | MAJORID(0x02) | MINORID(0x03) | PATCHID(0x01)),
    GSL_CHIPID_YAMATODX_REV24   = (COREID(0x00) | MAJORID(0x02) | MINORID(0x04) | PATCHID(0x00)),
    GSL_CHIPID_YAMATODX_REV25   = (COREID(0x00) | MAJORID(0x02) | MINORID(0x05) | PATCHID(0x00)),
    GSL_CHIPID_YAMATODX_REV251  = (COREID(0x00) | MAJORID(0x02) | MINORID(0x05) | PATCHID(0x01)),
    GSL_CHIPID_G12_REV00        = (int)(COREID(0x80) | MAJORID(0x00) | MINORID(0x00) | PATCHID(0x00)),
    GSL_CHIPID_ERROR            = (int)0xFFFFFFFF

} gsl_chipid_t;

#undef COREID
#undef MAJORID
#undef MINORID
#undef PATCHID

// -----------
// device info
// -----------
typedef struct _gsl_devinfo_t {

    gsl_deviceid_t  device_id;          // ID of this device
    gsl_chipid_t    chip_id;
    int             mmu_enabled;        // mmu address translation enabled
    unsigned int    gmem_gpubaseaddr;
    void *          gmem_hostbaseaddr;  // if gmem_hostbaseaddr is NULL, we would know its not mapped into mmio space
    unsigned int    gmem_sizebytes;
    unsigned int    high_precision; /* mx50 z160 has higher gradient/texture precision */

} gsl_devinfo_t;

// -------------------
// device memory store
// -------------------
typedef struct _gsl_devmemstore_t {
    volatile unsigned int  soptimestamp;
    unsigned int           sbz;
    volatile unsigned int  eoptimestamp;
    unsigned int           sbz2;
} gsl_devmemstore_t;

#define GSL_DEVICE_MEMSTORE_OFFSET(field)       offsetof(gsl_devmemstore_t, field)

// -----------
// aperture id
// -----------
typedef enum _gsl_apertureid_t
{
    GSL_APERTURE_EMEM   = (GSL_MEMFLAGS_EMEM),
    GSL_APERTURE_PHYS   = (GSL_MEMFLAGS_CONPHYS >> GSL_MEMFLAGS_APERTURE_SHIFT),
    GSL_APERTURE_MMU    = (GSL_APERTURE_EMEM | 0x10000000),
    GSL_APERTURE_MAX    = 2,

    GSL_APERTURE_FOOBAR = 0x7FFFFFFF
} gsl_apertureid_t;

// ----------
// channel id
// ----------
typedef enum _gsl_channelid_t
{
    GSL_CHANNEL_1      = (GSL_MEMFLAGS_CHANNEL1 >> GSL_MEMFLAGS_CHANNEL_SHIFT),
    GSL_CHANNEL_2      = (GSL_MEMFLAGS_CHANNEL2 >> GSL_MEMFLAGS_CHANNEL_SHIFT),
    GSL_CHANNEL_3      = (GSL_MEMFLAGS_CHANNEL3 >> GSL_MEMFLAGS_CHANNEL_SHIFT),
    GSL_CHANNEL_4      = (GSL_MEMFLAGS_CHANNEL4 >> GSL_MEMFLAGS_CHANNEL_SHIFT),
    GSL_CHANNEL_MAX    = 4,

    GSL_CHANNEL_FOOBAR = 0x7FFFFFFF
} gsl_channelid_t;

// ----------------------
// page access permission
// ----------------------
typedef enum _gsl_ap_t
{
    GSL_AP_NULL   = 0x0,
    GSL_AP_R      = 0x1,
    GSL_AP_W      = 0x2,
    GSL_AP_RW     = 0x3,
    GSL_AP_X      = 0x4,
    GSL_AP_RWX    = 0x5,
    GSL_AP_MAX    = 0x6,

    GSL_AP_FOOBAR = 0x7FFFFFFF
} gsl_ap_t;

// -------------
// memory region
// -------------
typedef struct _gsl_memregion_t {
    unsigned char  *mmio_virt_base;
    unsigned int   mmio_phys_base;
    gpuaddr_t      gpu_base;
    unsigned int   sizebytes;
} gsl_memregion_t;

// ------------------------
// shared memory allocation
// ------------------------
typedef struct _gsl_memdesc_t {
    void          *hostptr;
    gpuaddr_t      gpuaddr;
    int            size;
    unsigned int   priv;                // private
    unsigned int   priv2;               // private

} gsl_memdesc_t;

// ---------------------------------
// physical page scatter/gatter list
// ---------------------------------
typedef struct _gsl_scatterlist_t {
    int           contiguous;       // flag whether pages on the list are physically contiguous
    unsigned int  num;
    unsigned int  *pages;
} gsl_scatterlist_t;

// --------------
// mem free queue
// --------------
//
// this could be compressed down into the just the memdesc for the node
//
typedef struct _gsl_memnode_t {
    gsl_timestamp_t       timestamp;
    gsl_memdesc_t         memdesc;
    unsigned int          pid;
    struct _gsl_memnode_t *next;
} gsl_memnode_t;

typedef struct _gsl_memqueue_t {
    gsl_memnode_t   *head;
    gsl_memnode_t   *tail;
} gsl_memqueue_t;

// ------------
// timestamp id
// ------------
typedef enum _gsl_timestamp_type_t
{
    GSL_TIMESTAMP_CONSUMED = 1, // start-of-pipeline timestamp
    GSL_TIMESTAMP_RETIRED  = 2, // end-of-pipeline timestamp
    GSL_TIMESTAMP_MAX      = 2,

    GSL_TIMESTAMP_FOOBAR   = 0x7FFFFFFF
} gsl_timestamp_type_t;

// ------------
// context type
// ------------
typedef enum _gsl_context_type_t
{
    GSL_CONTEXT_TYPE_GENERIC = 1,
    GSL_CONTEXT_TYPE_OPENGL  = 2,
    GSL_CONTEXT_TYPE_OPENVG  = 3,

    GSL_CONTEXT_TYPE_FOOBAR  = 0x7FFFFFFF
} gsl_context_type_t;

// ---------
// rectangle
// ---------
typedef struct _gsl_rect_t {
    unsigned int x;
    unsigned int y;
    unsigned int width;
    unsigned int height;
	unsigned int pitch;
} gsl_rect_t;

// -----------------------
// pixel buffer descriptor
// -----------------------
typedef struct _gsl_buffer_desc_t {
    gsl_memdesc_t data;
	unsigned int width;
	unsigned int height;
	unsigned int pitch;
	unsigned int format;
    unsigned int enabled;
} gsl_buffer_desc_t;

// ---------------------
// command window target
// ---------------------
typedef enum _gsl_cmdwindow_t
{
    GSL_CMDWINDOW_MIN     = 0x00000000,
    GSL_CMDWINDOW_2D      = 0x00000000,
    GSL_CMDWINDOW_3D      = 0x00000001,     // legacy
    GSL_CMDWINDOW_MMU     = 0x00000002,
    GSL_CMDWINDOW_ARBITER = 0x000000FF,
    GSL_CMDWINDOW_MAX     = 0x000000FF,

    GSL_CMDWINDOW_FOOBAR  = 0x7FFFFFFF
} gsl_cmdwindow_t;

// ------------
// interrupt id
// ------------
typedef enum _gsl_intrid_t
{
  GSL_INTR_YDX_MH_AXI_READ_ERROR = 0,
  GSL_INTR_YDX_MH_AXI_WRITE_ERROR,
  GSL_INTR_YDX_MH_MMU_PAGE_FAULT,

  GSL_INTR_YDX_CP_SW_INT,
  GSL_INTR_YDX_CP_T0_PACKET_IN_IB,
  GSL_INTR_YDX_CP_OPCODE_ERROR,
  GSL_INTR_YDX_CP_PROTECTED_MODE_ERROR,
  GSL_INTR_YDX_CP_RESERVED_BIT_ERROR,
  GSL_INTR_YDX_CP_IB_ERROR,
  GSL_INTR_YDX_CP_IB2_INT,
  GSL_INTR_YDX_CP_IB1_INT,
  GSL_INTR_YDX_CP_RING_BUFFER,

  GSL_INTR_YDX_RBBM_READ_ERROR,
  GSL_INTR_YDX_RBBM_DISPLAY_UPDATE,
  GSL_INTR_YDX_RBBM_GUI_IDLE,

  GSL_INTR_YDX_SQ_PS_WATCHDOG,
  GSL_INTR_YDX_SQ_VS_WATCHDOG,

  GSL_INTR_G12_MH,
  GSL_INTR_G12_G2D,
  GSL_INTR_G12_FIFO,
#ifndef _Z180
  GSL_INTR_G12_FBC,
#endif // _Z180

  GSL_INTR_G12_MH_AXI_READ_ERROR,
  GSL_INTR_G12_MH_AXI_WRITE_ERROR,
  GSL_INTR_G12_MH_MMU_PAGE_FAULT,

  GSL_INTR_COUNT,

  GSL_INTR_FOOBAR = 0x7FFFFFFF
} gsl_intrid_t;

#endif  // __GSL_TYPES_H
