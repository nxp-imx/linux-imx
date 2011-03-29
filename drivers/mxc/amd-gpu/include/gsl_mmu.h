/* Copyright (c) 2002,2007-2010, Code Aurora Forum. All rights reserved.
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

#ifndef __GSL_MMU_H
#define __GSL_MMU_H


//////////////////////////////////////////////////////////////////////////////
// defines
//////////////////////////////////////////////////////////////////////////////
#ifdef GSL_STATS_MMU
#define GSL_MMU_STATS(x) x
#else
#define GSL_MMU_STATS(x)
#endif // GSL_STATS_MMU

#ifdef GSL_MMU_PAGETABLE_PERPROCESS
#define GSL_MMU_PAGETABLE_MAX               GSL_CALLER_PROCESS_MAX      // all device mmu's share a single page table per process
#else
#define GSL_MMU_PAGETABLE_MAX               1                           // all device mmu's share a single global page table
#endif // GSL_MMU_PAGETABLE_PERPROCESS

#define GSL_PT_SUPER_PTE                    8


//////////////////////////////////////////////////////////////////////////////
//  types
//////////////////////////////////////////////////////////////////////////////

#ifdef _DEBUG
// ---------
// mmu debug
// ---------
typedef struct _gsl_mmu_debug_t {
    unsigned int  config;
    unsigned int  mpu_base;
    unsigned int  mpu_end;
    unsigned int  va_range;
    unsigned int  pt_base;
    unsigned int  page_fault;
    unsigned int  trans_error;
    unsigned int  invalidate;
} gsl_mmu_debug_t;
#endif // _DEBUG

// ------------
// mmu register
// ------------
typedef struct _gsl_mmu_reg_t
{
    unsigned int  CONFIG;
    unsigned int  MPU_BASE;
    unsigned int  MPU_END;
    unsigned int  VA_RANGE;
    unsigned int  PT_BASE;
    unsigned int  PAGE_FAULT;
    unsigned int  TRAN_ERROR;
    unsigned int  INVALIDATE;
} gsl_mmu_reg_t;

// ------------
// mh interrupt
// ------------
typedef struct _gsl_mh_intr_t
{
    gsl_intrid_t  AXI_READ_ERROR;
    gsl_intrid_t  AXI_WRITE_ERROR;
    gsl_intrid_t  MMU_PAGE_FAULT;
} gsl_mh_intr_t;

// ----------------
// page table stats
// ----------------
typedef struct _gsl_ptstats_t {
    __int64  maps;
    __int64  unmaps;
	__int64  switches;
} gsl_ptstats_t;

// ---------
// mmu stats
// ---------
typedef struct _gsl_mmustats_t {
	gsl_ptstats_t  pt;
	__int64        tlbflushes;
} gsl_mmustats_t;

// -----------------
// page table object
// -----------------
typedef struct _gsl_pagetable_t {
	unsigned int   pid;
    unsigned int   refcnt;
    gsl_memdesc_t  base;
    gpuaddr_t      va_base;
    unsigned int   va_range;
    unsigned int   last_superpte;
    unsigned int   max_entries;
} gsl_pagetable_t;

// -------------------------
// tlb flush filter object
// -------------------------
typedef struct _gsl_tlbflushfilter_t {
    unsigned int  *base;
    unsigned int  size;
} gsl_tlbflushfilter_t;

// ----------
// mmu object
// ----------
typedef struct _gsl_mmu_t {
#ifdef GSL_LOCKING_FINEGRAIN
    oshandle_t            mutex;
#endif
    unsigned int          refcnt;
    gsl_flags_t           flags;
    gsl_device_t          *device;
    unsigned int          config;
    gpuaddr_t             mpu_base;
    int                   mpu_range;
    gpuaddr_t             va_base;
    unsigned int          va_range;
    gsl_memdesc_t         dummyspace;
    gsl_tlbflushfilter_t  tlbflushfilter;
    gsl_pagetable_t       *hwpagetable;                     // current page table object being used by device mmu
    gsl_pagetable_t       *pagetable[GSL_MMU_PAGETABLE_MAX];    // page table object table
#ifdef GSL_STATS_MMU
	gsl_mmustats_t        stats;
#endif  // GSL_STATS_MMU
} gsl_mmu_t;


//////////////////////////////////////////////////////////////////////////////
//  inline functions
//////////////////////////////////////////////////////////////////////////////
OSINLINE int
kgsl_mmu_isenabled(gsl_mmu_t *mmu)
{
    // address translation enabled
    int enabled = ((mmu)->flags & GSL_FLAGS_STARTED) ? 1 : 0;

    return (enabled);
}


//////////////////////////////////////////////////////////////////////////////
//  prototypes
//////////////////////////////////////////////////////////////////////////////
int    kgsl_mmu_init(gsl_device_t *device);
int    kgsl_mmu_close(gsl_device_t *device);
int    kgsl_mmu_attachcallback(gsl_mmu_t *mmu, unsigned int pid);
int    kgsl_mmu_detachcallback(gsl_mmu_t *mmu, unsigned int pid);
int    kgsl_mmu_setpagetable(gsl_device_t *device, unsigned int pid);
int    kgsl_mmu_map(gsl_mmu_t *mmu, gpuaddr_t gpubaseaddr, const gsl_scatterlist_t *scatterlist, gsl_flags_t flags, unsigned int pid);
int    kgsl_mmu_unmap(gsl_mmu_t *mmu, gpuaddr_t gpubaseaddr, int range, unsigned int pid);
int    kgsl_mmu_getmap(gsl_mmu_t *mmu, gpuaddr_t gpubaseaddr, int range, gsl_scatterlist_t *scatterlist, unsigned int pid);
int    kgsl_mmu_querystats(gsl_mmu_t *mmu, gsl_mmustats_t *stats);
int    kgsl_mmu_bist(gsl_mmu_t *mmu);

#endif // __GSL_MMU_H
