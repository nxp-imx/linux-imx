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

#ifndef __GSL_MEMMGR_H
#define __GSL_MEMMGR_H


//////////////////////////////////////////////////////////////////////////////
// defines
//////////////////////////////////////////////////////////////////////////////
#define GSL_MEMARENA_NODE_POOL_MAX      32                              // max is 32

#define GSL_MEMARENA_PAGE_DIST_MAX      12                              // 4MB

//#define GSL_MEMARENA_NODE_POOL_ENABLED


//////////////////////////////////////////////////////////////////////////////
//  types
//////////////////////////////////////////////////////////////////////////////

// ------------------
// memory arena stats
// ------------------
typedef struct _gsl_memarena_stats_t {
    __int64  bytes_read;
    __int64  bytes_written;
    __int64  allocs_success;
    __int64  allocs_fail;
    __int64  frees;
    __int64  allocs_pagedistribution[GSL_MEMARENA_PAGE_DIST_MAX]; // 0=0--(4K-1), 1=4--(8K-1), 2=8--(16K-1),... max-1=(GSL_PAGESIZE<<(max-1))--infinity
    __int64  frees_pagedistribution[GSL_MEMARENA_PAGE_DIST_MAX];
} gsl_memarena_stats_t;

// ------------
// memory block
// ------------
typedef struct _memblk_t {
    unsigned int      blkaddr;
    unsigned int      blksize;
    struct _memblk_t  *next;
    struct _memblk_t  *prev;
    int               nodepoolindex;
} memblk_t;

// ----------------------
// memory block free list
// ----------------------
typedef struct _gsl_freelist_t {
    memblk_t  *head;
    memblk_t  *allocrover;
    memblk_t  *freerover;
} gsl_freelist_t;

// ----------------------
// memory block node pool
// ----------------------
typedef struct _gsl_nodepool_t {
    unsigned int            priv;
    memblk_t                memblk[GSL_MEMARENA_NODE_POOL_MAX];
    struct _gsl_nodepool_t  *next;
    struct _gsl_nodepool_t  *prev;
} gsl_nodepool_t;

// -------------------
// memory arena object
// -------------------
typedef struct _gsl_memarena_t {
    oshandle_t      mutex;
    unsigned int    gpubaseaddr;
    unsigned int    hostbaseaddr;
    unsigned int    sizebytes;
    gsl_nodepool_t  *nodepool;
    gsl_freelist_t  freelist;
    unsigned int    priv;

#ifdef GSL_STATS_MEM
    gsl_memarena_stats_t  stats;
#endif // GSL_STATS_MEM

} gsl_memarena_t;


//////////////////////////////////////////////////////////////////////////////
//  prototypes
//////////////////////////////////////////////////////////////////////////////
gsl_memarena_t* kgsl_memarena_create(int aperture_id, int mmu_virtualized, unsigned int hostbaseaddr, gpuaddr_t gpubaseaddr, int sizebytes);
int             kgsl_memarena_destroy(gsl_memarena_t *memarena);
int             kgsl_memarena_isvirtualized(gsl_memarena_t *memarena);
int             kgsl_memarena_querystats(gsl_memarena_t *memarena, gsl_memarena_stats_t *stats);
int             kgsl_memarena_alloc(gsl_memarena_t *memarena, gsl_flags_t flags, int size, gsl_memdesc_t *memdesc);
void            kgsl_memarena_free(gsl_memarena_t *memarena, gsl_memdesc_t *memdesc);
void*           kgsl_memarena_gethostptr(gsl_memarena_t *memarena, gpuaddr_t gpuaddr);
unsigned int    kgsl_memarena_getgpuaddr(gsl_memarena_t *memarena, void *hostptr);
unsigned int    kgsl_memarena_getlargestfreeblock(gsl_memarena_t *memarena, gsl_flags_t flags);

#endif  // __GSL_MEMMGR_H
