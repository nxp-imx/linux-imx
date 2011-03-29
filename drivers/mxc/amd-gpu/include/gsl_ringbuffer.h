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

#ifndef __GSL_RINGBUFFER_H
#define __GSL_RINGBUFFER_H


//////////////////////////////////////////////////////////////////////////////
//  defines
//////////////////////////////////////////////////////////////////////////////

// ringbuffer sizes log2quadword
#define     GSL_RB_SIZE_8                        0
#define     GSL_RB_SIZE_16                       1
#define     GSL_RB_SIZE_32                       2
#define     GSL_RB_SIZE_64                       3
#define     GSL_RB_SIZE_128                      4
#define     GSL_RB_SIZE_256                      5
#define     GSL_RB_SIZE_512                      6
#define     GSL_RB_SIZE_1K                       7
#define     GSL_RB_SIZE_2K                       8
#define     GSL_RB_SIZE_4K                       9
#define     GSL_RB_SIZE_8K                      10
#define     GSL_RB_SIZE_16K                     11
#define     GSL_RB_SIZE_32K                     12
#define     GSL_RB_SIZE_64K                     13
#define     GSL_RB_SIZE_128K                    14
#define     GSL_RB_SIZE_256K                    15
#define     GSL_RB_SIZE_512K                    16
#define     GSL_RB_SIZE_1M                      17
#define     GSL_RB_SIZE_2M                      18
#define     GSL_RB_SIZE_4M                      19

// offsets into memptrs
#define GSL_RB_MEMPTRS_RPTR_OFFSET              0
#define GSL_RB_MEMPTRS_WPTRPOLL_OFFSET          (GSL_RB_MEMPTRS_RPTR_OFFSET + sizeof(unsigned int))

// dword base address of the GFX decode space
#define GSL_HAL_SUBBLOCK_OFFSET(reg)            ((unsigned int)((reg) - (0x2000)))

// CP timestamp register
#define mmCP_TIMESTAMP                          mmSCRATCH_REG0


//////////////////////////////////////////////////////////////////////////////
// types
//////////////////////////////////////////////////////////////////////////////

#ifdef _DEBUG
// ----------------
// ringbuffer debug
// ----------------
typedef struct _gsl_rb_debug_t {
    unsigned int        pm4_ucode_rel;
    unsigned int        pfp_ucode_rel;
    unsigned int        cp_rb_base;
    cp_rb_cntl_u        cp_rb_cntl;
    unsigned int        cp_rb_rptr_addr;
    unsigned int        cp_rb_rptr;
    unsigned int        cp_rb_wptr;
    unsigned int        cp_rb_wptr_base;
    scratch_umsk_u      scratch_umsk;
    unsigned int        scratch_addr;
    cp_me_cntl_u        cp_me_cntl;
    cp_me_status_u      cp_me_status;
    cp_debug_u          cp_debug;
    cp_stat_u           cp_stat;
    rbbm_status_u       rbbm_status;
    unsigned int        sop_timestamp;
    unsigned int        eop_timestamp;
} gsl_rb_debug_t;
#endif // _DEBUG

// -------------------
// ringbuffer watchdog
// -------------------
typedef struct _gsl_rbwatchdog_t {
    gsl_flags_t   flags;
    unsigned int  rptr_sample;
} gsl_rbwatchdog_t;

// ------------------
// memory ptr objects
// ------------------
#ifdef __GNUC__
#pragma pack(push, 1)
#else
#pragma pack(push)
#pragma pack(1)
#endif
typedef struct _gsl_rbmemptrs_t {
    volatile int  rptr;
    int           wptr_poll;
} gsl_rbmemptrs_t;
#pragma pack(pop)

// -----
// stats
// -----
typedef struct _gsl_rbstats_t {
    __int64  wraps;
    __int64  issues;
    __int64  wordstotal;
} gsl_rbstats_t;


// -----------------
// ringbuffer object
// -----------------
typedef struct _gsl_ringbuffer_t {

    gsl_device_t      *device;
    gsl_flags_t       flags;
#ifdef GSL_LOCKING_FINEGRAIN
    oshandle_t        mutex;
#endif
    gsl_memdesc_t     buffer_desc;              // allocated memory descriptor
    gsl_memdesc_t     memptrs_desc;

    gsl_rbmemptrs_t   *memptrs;

    unsigned int      sizedwords;               // ring buffer size dwords
    unsigned int      blksizequadwords;

    unsigned int      wptr;                     // write pointer offset in dwords from baseaddr
    unsigned int      rptr;                     // read pointer  offset in dwords from baseaddr
    gsl_timestamp_t   timestamp;


    gsl_rbwatchdog_t  watchdog;

#ifdef GSL_STATS_RINGBUFFER
    gsl_rbstats_t     stats;
#endif // GSL_STATS_RINGBUFFER

} gsl_ringbuffer_t;


//////////////////////////////////////////////////////////////////////////////
// macros
//////////////////////////////////////////////////////////////////////////////

#ifdef GSL_LOCKING_FINEGRAIN
#define GSL_RB_MUTEX_CREATE()               rb->mutex = kos_mutex_create("gsl_ringbuffer"); \
                                            if (!rb->mutex) {return (GSL_FAILURE);}
#define GSL_RB_MUTEX_LOCK()                 kos_mutex_lock(rb->mutex)
#define GSL_RB_MUTEX_UNLOCK()               kos_mutex_unlock(rb->mutex)
#define GSL_RB_MUTEX_FREE()                 kos_mutex_free(rb->mutex); rb->mutex = 0;
#else
#define GSL_RB_MUTEX_CREATE()
#define GSL_RB_MUTEX_LOCK()
#define GSL_RB_MUTEX_UNLOCK()
#define GSL_RB_MUTEX_FREE()
#endif

// ----------
// ring write
// ----------
#define GSL_RB_WRITE(ring, data)        \
      KGSL_DEBUG(GSL_DBGFLAGS_DUMPX, KGSL_DEBUG_DUMPX(BB_DUMP_RINGBUF_WRT, (unsigned int)ring, data, 0, "GSL_RB_WRITE")); \
      *(unsigned int *)(ring)++ = (unsigned int)(data);

// ---------
// timestamp
// ---------
#ifdef GSL_DEVICE_SHADOW_MEMSTORE_TO_USER
#define GSL_RB_USE_MEM_TIMESTAMP
#endif //GSL_DEVICE_SHADOW_MEMSTORE_TO_USER

#ifdef  GSL_RB_USE_MEM_TIMESTAMP
#define GSL_RB_MEMPTRS_SCRATCH_MASK         0x1     // enable timestamp (...scratch0) memory shadowing
#define GSL_RB_INIT_TIMESTAMP(rb)

#else
#define GSL_RB_MEMPTRS_SCRATCH_MASK         0x0     // disable
#define GSL_RB_INIT_TIMESTAMP(rb)           kgsl_device_regwrite((rb)->device->id, mmCP_TIMESTAMP, 0);
#endif // GSL_RB_USE_MEMTIMESTAMP

// --------
// mem rptr
// --------
#ifdef  GSL_RB_USE_MEM_RPTR
#define GSL_RB_CNTL_NO_UPDATE               0x0     // enable
#define GSL_RB_GET_READPTR(rb, data)        kgsl_sharedmem_read0(&(rb)->memptrs_desc, (data), GSL_RB_MEMPTRS_RPTR_OFFSET, 4, false)
#else
#define GSL_RB_CNTL_NO_UPDATE               0x1     // disable
#define GSL_RB_GET_READPTR(rb, data)        (rb)->device->fbtl.device_regread((rb)->device, mmCP_RB_RPTR,(data))
#endif // GSL_RB_USE_MEMRPTR

// ------------
// wptr polling
// ------------
#ifdef  GSL_RB_USE_WPTR_POLLING
#define GSL_RB_CNTL_POLL_EN                 0x1     // enable
#define GSL_RB_UPDATE_WPTR_POLLING(rb)      (rb)->memptrs->wptr_poll = (rb)->wptr
#else
#define GSL_RB_CNTL_POLL_EN                 0x0     // disable
#define GSL_RB_UPDATE_WPTR_POLLING(rb)
#endif  // GSL_RB_USE_WPTR_POLLING

// -----
// stats
// -----
#ifdef GSL_STATS_RINGBUFFER
#define GSL_RB_STATS(x) x
#else
#define GSL_RB_STATS(x)
#endif // GSL_STATS_RINGBUFFER


//////////////////////////////////////////////////////////////////////////////
//  prototypes
//////////////////////////////////////////////////////////////////////////////
int             kgsl_ringbuffer_init(gsl_device_t *device);
int             kgsl_ringbuffer_close(gsl_ringbuffer_t *rb);
int             kgsl_ringbuffer_start(gsl_ringbuffer_t *rb);
int             kgsl_ringbuffer_stop(gsl_ringbuffer_t *rb);
gsl_timestamp_t	kgsl_ringbuffer_issuecmds(gsl_device_t *device, int pmodeoff, unsigned int *cmdaddr, int sizedwords, unsigned int pid);
int             kgsl_ringbuffer_issueibcmds(gsl_device_t *device, int drawctxt_index, gpuaddr_t ibaddr, int sizedwords, gsl_timestamp_t *timestamp, gsl_flags_t flags);
void            kgsl_ringbuffer_watchdog(void);

int             kgsl_ringbuffer_querystats(gsl_ringbuffer_t *rb, gsl_rbstats_t *stats);
int             kgsl_ringbuffer_bist(gsl_ringbuffer_t *rb);

#endif  // __GSL_RINGBUFFER_H
