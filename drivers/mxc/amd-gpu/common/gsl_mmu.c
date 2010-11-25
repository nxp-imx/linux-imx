/* Copyright (c) 2002,2007-2010, Code Aurora Forum. All rights reserved.
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


//////////////////////////////////////////////////////////////////////////////
// types
//////////////////////////////////////////////////////////////////////////////

// ---------
// pte debug
// ---------

typedef struct _gsl_pte_debug_t
{
    unsigned int    write     :1;
    unsigned int    read      :1;
    unsigned int    reserved  :10;
    unsigned int    phyaddr   :20;
} gsl_pte_debug_t;


//////////////////////////////////////////////////////////////////////////////
//  defines
//////////////////////////////////////////////////////////////////////////////
#define GSL_PT_ENTRY_SIZEBYTES              4
#define GSL_PT_EXTRA_ENTRIES                16

#define GSL_PT_PAGE_WRITE                   0x00000001
#define GSL_PT_PAGE_READ                    0x00000002

#define GSL_PT_PAGE_AP_MASK                 0x00000003
#define GSL_PT_PAGE_ADDR_MASK               ~(GSL_PAGESIZE-1)

#define GSL_MMUFLAGS_TLBFLUSH               0x80000000

#define GSL_TLBFLUSH_FILTER_ENTRY_NUMBITS   (sizeof(unsigned char) * 8)


//////////////////////////////////////////////////////////////////////////////
// constants
//////////////////////////////////////////////////////////////////////////////
const unsigned int GSL_PT_PAGE_AP[4] = {(GSL_PT_PAGE_READ | GSL_PT_PAGE_WRITE), GSL_PT_PAGE_READ, GSL_PT_PAGE_WRITE, 0};


/////////////////////////////////////////////////////////////////////////////
// macros
//////////////////////////////////////////////////////////////////////////////
#ifdef GSL_LOCKING_FINEGRAIN
#define GSL_MMU_MUTEX_CREATE()              mmu->mutex = kos_mutex_create("gsl_mmu"); \
                                            if (!mmu->mutex) {return (GSL_FAILURE);}
#define GSL_MMU_LOCK()                      kos_mutex_lock(mmu->mutex)
#define GSL_MMU_UNLOCK()                    kos_mutex_unlock(mmu->mutex)
#define GSL_MMU_MUTEX_FREE()                kos_mutex_free(mmu->mutex); mmu->mutex = 0;
#else
#define GSL_MMU_MUTEX_CREATE()
#define GSL_MMU_LOCK()
#define GSL_MMU_UNLOCK()
#define GSL_MMU_MUTEX_FREE()
#endif

#define GSL_PT_ENTRY_GET(va)                ((va - pagetable->va_base) >> GSL_PAGESIZE_SHIFT)
#define GSL_PT_VIRT_GET(pte)                (pagetable->va_base + (pte * GSL_PAGESIZE))

#define GSL_PT_MAP_APDEFAULT                GSL_PT_PAGE_AP[0]

#define GSL_PT_MAP_GET(pte)                 *((unsigned int *)(((unsigned int)pagetable->base.hostptr) + ((pte) * GSL_PT_ENTRY_SIZEBYTES)))
#define GSL_PT_MAP_GETADDR(pte)             (GSL_PT_MAP_GET(pte) & GSL_PT_PAGE_ADDR_MASK)

#define GSL_PT_MAP_DEBUG(pte)               ((gsl_pte_debug_t*) &GSL_PT_MAP_GET(pte))

#define GSL_PT_MAP_SETBITS(pte, bits)       (GSL_PT_MAP_GET(pte) |= (((unsigned int) bits) & GSL_PT_PAGE_AP_MASK))
#define GSL_PT_MAP_SETADDR(pte, pageaddr)   (GSL_PT_MAP_GET(pte)  = (GSL_PT_MAP_GET(pte) & ~GSL_PT_PAGE_ADDR_MASK) | (((unsigned int) pageaddr) & GSL_PT_PAGE_ADDR_MASK))

#define GSL_PT_MAP_RESET(pte)               (GSL_PT_MAP_GET(pte)  = 0)
#define GSL_PT_MAP_RESETBITS(pte, bits)     (GSL_PT_MAP_GET(pte) &= ~(((unsigned int) bits) & GSL_PT_PAGE_AP_MASK))

#define GSL_MMU_VIRT_TO_PAGE(va)            *((unsigned int *)(pagetable->base.gpuaddr + (GSL_PT_ENTRY_GET(va) * GSL_PT_ENTRY_SIZEBYTES)))
#define GSL_MMU_VIRT_TO_PHYS(va)            ((GSL_MMU_VIRT_TO_PAGE(va) & GSL_PT_PAGE_ADDR_MASK) + (va & (GSL_PAGESIZE-1)))

#define GSL_TLBFLUSH_FILTER_GET(superpte)       *((unsigned char *)(((unsigned int)mmu->tlbflushfilter.base) + (superpte / GSL_TLBFLUSH_FILTER_ENTRY_NUMBITS)))
#define GSL_TLBFLUSH_FILTER_SETDIRTY(superpte)  (GSL_TLBFLUSH_FILTER_GET((superpte)) |= 1 << (superpte % GSL_TLBFLUSH_FILTER_ENTRY_NUMBITS))
#define GSL_TLBFLUSH_FILTER_ISDIRTY(superpte)   (GSL_TLBFLUSH_FILTER_GET((superpte)) & (1 << (superpte % GSL_TLBFLUSH_FILTER_ENTRY_NUMBITS)))
#define GSL_TLBFLUSH_FILTER_RESET()             kos_memset(mmu->tlbflushfilter.base, 0, mmu->tlbflushfilter.size)


//////////////////////////////////////////////////////////////////////////////
// process index in pagetable object table
//////////////////////////////////////////////////////////////////////////////
OSINLINE int
kgsl_mmu_getprocessindex(unsigned int pid, int *pindex)
{
    int status = GSL_SUCCESS;
#ifdef GSL_MMU_PAGETABLE_PERPROCESS
    if (kgsl_driver_getcallerprocessindex(pid, pindex) != GSL_SUCCESS)
    {
        status = GSL_FAILURE;
    }
#else
    (void) pid;      // unreferenced formal parameter
    *pindex = 0;
#endif // GSL_MMU_PAGETABLE_PERPROCESS
    return (status);
}

//////////////////////////////////////////////////////////////////////////////
// pagetable object for current caller process
//////////////////////////////////////////////////////////////////////////////
OSINLINE gsl_pagetable_t*
kgsl_mmu_getpagetableobject(gsl_mmu_t *mmu, unsigned int pid)
{
    int pindex = 0;
    if (kgsl_mmu_getprocessindex(pid, &pindex) == GSL_SUCCESS)
    {
        return (mmu->pagetable[pindex]);
    }
    else
    {
        return (NULL);
    }
}


//////////////////////////////////////////////////////////////////////////////
//  functions
//////////////////////////////////////////////////////////////////////////////

void
kgsl_mh_intrcallback(gsl_intrid_t id, void *cookie)
{
    gsl_mmu_t     *mmu = (gsl_mmu_t *) cookie;
    unsigned int   devindex = mmu->device->id-1;        // device_id is 1 based

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE,
                    "--> void kgsl_mh_ntrcallback(gsl_intrid_t id=%I, void *cookie=0x%08x)\n", id, cookie );

    // error condition interrupt
    if (id == gsl_cfg_mh_intr[devindex].AXI_READ_ERROR  ||
        id == gsl_cfg_mh_intr[devindex].AXI_WRITE_ERROR ||
        id == gsl_cfg_mh_intr[devindex].MMU_PAGE_FAULT)
    {
	printk(KERN_ERR "GPU: AXI Read/Write Error or MMU page fault\n");
	schedule_work(&mmu->device->irq_err_work);
    }

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_mh_intrcallback.\n" );
}

//----------------------------------------------------------------------------

#ifdef _DEBUG
static void
kgsl_mmu_debug(gsl_mmu_t *mmu, gsl_mmu_debug_t *regs)
{
    unsigned int  devindex = mmu->device->id-1;         // device_id is 1 based

    kos_memset(regs, 0, sizeof(gsl_mmu_debug_t));

    mmu->device->ftbl.device_regread(mmu->device, gsl_cfg_mmu_reg[devindex].CONFIG,     &regs->config);
    mmu->device->ftbl.device_regread(mmu->device, gsl_cfg_mmu_reg[devindex].MPU_BASE,   &regs->mpu_base);
    mmu->device->ftbl.device_regread(mmu->device, gsl_cfg_mmu_reg[devindex].MPU_END,    &regs->mpu_end);
    mmu->device->ftbl.device_regread(mmu->device, gsl_cfg_mmu_reg[devindex].VA_RANGE,   &regs->va_range);
    mmu->device->ftbl.device_regread(mmu->device, gsl_cfg_mmu_reg[devindex].PT_BASE,    &regs->pt_base);
    mmu->device->ftbl.device_regread(mmu->device, gsl_cfg_mmu_reg[devindex].PAGE_FAULT, &regs->page_fault);
    mmu->device->ftbl.device_regread(mmu->device, gsl_cfg_mmu_reg[devindex].TRAN_ERROR, &regs->trans_error);
    mmu->device->ftbl.device_regread(mmu->device, gsl_cfg_mmu_reg[devindex].INVALIDATE, &regs->invalidate);
}
#endif

//----------------------------------------------------------------------------

int
kgsl_mmu_checkconsistency(gsl_pagetable_t *pagetable)
{
    unsigned int     pte;
    unsigned int     data;
    gsl_pte_debug_t  *pte_debug;

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE,
                    "--> int kgsl_mmu_checkconsistency(gsl_pagetable_t *pagetable=0x%08x)\n", pagetable );

    if (pagetable->last_superpte % GSL_PT_SUPER_PTE != 0)
    {
        KOS_ASSERT(0);
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_mmu_checkconsistency. Return value %B\n", GSL_FAILURE );
        return (GSL_FAILURE);
    }

    // go through page table and make sure there are no detectable errors
    pte = 0;
    while (pte < pagetable->max_entries)
    {
        pte_debug = GSL_PT_MAP_DEBUG(pte);

        if (GSL_PT_MAP_GETADDR(pte) != 0)
        {
            // pte is in use

            // access first couple bytes of a page
            data = *((unsigned int *)GSL_PT_VIRT_GET(pte));
        }

        pte++;
    }

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_mmu_checkconsistency. Return value %B\n", GSL_SUCCESS );
    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

int
kgsl_mmu_destroypagetableobject(gsl_mmu_t *mmu, unsigned int pid)
{
    gsl_deviceid_t   tmp_id;
    gsl_device_t     *tmp_device;
    int              pindex;
    gsl_pagetable_t  *pagetable;

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "--> gsl_pagetable_t* kgsl_mmu_destroypagetableobject(gsl_mmu_t *mmu=0x%08x, unsigned int pid=0x%08x)\n", mmu, pid );

    if (kgsl_mmu_getprocessindex(pid, &pindex) != GSL_SUCCESS)
    {
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_mmu_destroypagetableobject. Return value 0x%08x\n", GSL_SUCCESS );
        return (GSL_FAILURE);
    }

    pagetable = mmu->pagetable[pindex];

    // if pagetable object exists for current "current device mmu"/"current caller process" combination
    if (pagetable)
    {
        // no more "device mmu"/"caller process" combinations attached to current pagetable object
        if (pagetable->refcnt == 0)
        {
#ifdef _DEBUG
            // memory leak check
            if (pagetable->last_superpte != 0 || GSL_PT_MAP_GETADDR(pagetable->last_superpte))
            {
                /* many dumpx test cases forcefully exit, and thus trigger this assert. */
                /* Because it is an annoyance for HW guys, it is disabled for dumpx */
                if(!gsl_driver.flags_debug & GSL_DBGFLAGS_DUMPX)
                {
                    KOS_ASSERT(0);
                    return (GSL_FAILURE);
                }
            }
#endif // _DEBUG

            if (pagetable->base.gpuaddr)
            {
                kgsl_sharedmem_free0(&pagetable->base, GSL_CALLER_PROCESSID_GET());
            }

            kos_free(pagetable);

            // clear pagetable object reference for all "device mmu"/"current caller process" combinations
            for (tmp_id = GSL_DEVICE_ANY + 1; tmp_id <= GSL_DEVICE_MAX; tmp_id++)
            {
                tmp_device = &gsl_driver.device[tmp_id-1];

                if (tmp_device->mmu.flags & GSL_FLAGS_STARTED)
                {
                    tmp_device->mmu.pagetable[pindex] = NULL;
                }
            }
        }
    }

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_mmu_destroypagetableobject. Return value 0x%08x\n", GSL_SUCCESS );

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

gsl_pagetable_t*
kgsl_mmu_createpagetableobject(gsl_mmu_t *mmu, unsigned int pid)
{
    //
    // create pagetable object for "current device mmu"/"current caller
    // process" combination. If none exists, setup a new pagetable object.
    //
    int              status         = GSL_SUCCESS;
    gsl_pagetable_t  *tmp_pagetable = NULL;
    gsl_deviceid_t   tmp_id;
    gsl_device_t     *tmp_device;
    int              pindex;
    gsl_flags_t      flags;

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "--> gsl_pagetable_t* kgsl_mmu_createpagetableobject(gsl_mmu_t *mmu=0x%08x, unsigned int pid=0x%08x)\n", mmu, pid );

    status = kgsl_mmu_getprocessindex(pid, &pindex);
    if (status != GSL_SUCCESS)
    {
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_mmu_createpagetableobject. Return value 0x%08x\n", NULL );
        return (NULL);
    }
    // if pagetable object does not already exists for "current device mmu"/"current caller process" combination
    if (!mmu->pagetable[pindex])
    {
        // then, check if pagetable object already exists for any "other device mmu"/"current caller process" combination
        for (tmp_id = GSL_DEVICE_ANY + 1; tmp_id <= GSL_DEVICE_MAX; tmp_id++)
        {
            tmp_device = &gsl_driver.device[tmp_id-1];

            if (tmp_device->mmu.flags & GSL_FLAGS_STARTED)
            {
                if (tmp_device->mmu.pagetable[pindex])
                {
                    tmp_pagetable = tmp_device->mmu.pagetable[pindex];
                    break;
                }
            }
        }

        // pagetable object exists
        if (tmp_pagetable)
        {
            KOS_ASSERT(tmp_pagetable->va_base  == mmu->va_base);
            KOS_ASSERT(tmp_pagetable->va_range == mmu->va_range);

            // set pagetable object reference
            mmu->pagetable[pindex] = tmp_pagetable;
        }
        // create new pagetable object
        else
        {
            mmu->pagetable[pindex] = (void *)kos_malloc(sizeof(gsl_pagetable_t));
            if (!mmu->pagetable[pindex])
            {
                kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_ERROR, "ERROR: Unable to allocate pagetable object.\n" );
                kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_mmu_createpagetableobject. Return value 0x%08x\n", NULL );
                return (NULL);
            }

            kos_memset(mmu->pagetable[pindex], 0, sizeof(gsl_pagetable_t));

			mmu->pagetable[pindex]->pid           = pid;
            mmu->pagetable[pindex]->refcnt        = 0;
            mmu->pagetable[pindex]->va_base       = mmu->va_base;
            mmu->pagetable[pindex]->va_range      = mmu->va_range;
            mmu->pagetable[pindex]->last_superpte = 0;
            mmu->pagetable[pindex]->max_entries   = (mmu->va_range >> GSL_PAGESIZE_SHIFT) + GSL_PT_EXTRA_ENTRIES;

            // allocate page table memory
            flags  = (GSL_MEMFLAGS_ALIGN4K | GSL_MEMFLAGS_CONPHYS | GSL_MEMFLAGS_STRICTREQUEST);
            status = kgsl_sharedmem_alloc0(mmu->device->id, flags, mmu->pagetable[pindex]->max_entries * GSL_PT_ENTRY_SIZEBYTES, &mmu->pagetable[pindex]->base);

            if (status == GSL_SUCCESS)
            {
                // reset page table entries
                kgsl_sharedmem_set0(&mmu->pagetable[pindex]->base, 0, 0, mmu->pagetable[pindex]->base.size);

                KGSL_DEBUG(GSL_DBGFLAGS_DUMPX, KGSL_DEBUG_DUMPX(BB_DUMP_MMU_TBLADDR, mmu->pagetable[pindex]->base.gpuaddr, 0, mmu->pagetable[pindex]->base.size, "kgsl_mmu_init"));
            }
            else
            {
                kgsl_mmu_destroypagetableobject(mmu, pid);
            }
        }
    }

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_mmu_createpagetableobject. Return value 0x%08x\n", mmu->pagetable[pindex] );

    return (mmu->pagetable[pindex]);
}

//----------------------------------------------------------------------------

int
kgsl_mmu_setpagetable(gsl_device_t *device, unsigned int pid)
{
    //
    // set device mmu to use current caller process's page table
    //
    int              status   = GSL_SUCCESS;
    unsigned int     devindex = device->id-1;       // device_id is 1 based
    gsl_mmu_t        *mmu     = &device->mmu;

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE,
                    "--> gsl_pagetable_t* kgsl_mmu_setpagetable(gsl_device_t *device=0x%08x)\n", device );

    GSL_MMU_LOCK();

    if (mmu->flags & GSL_FLAGS_STARTED)
    {
#ifdef GSL_MMU_PAGETABLE_PERPROCESS
		// page table not current, then setup mmu to use new specified page table
		if (mmu->hwpagetable->pid != pid)
		{
			gsl_pagetable_t  *pagetable = kgsl_mmu_getpagetableobject(mmu, pid);
			if (pagetable)
			{
				mmu->hwpagetable = pagetable;

				// flag tlb flush	
				mmu->flags |= GSL_MMUFLAGS_TLBFLUSH;

				status = mmu->device->ftbl.mmu_setpagetable(mmu->device, gsl_cfg_mmu_reg[devindex].PT_BASE, pagetable->base.gpuaddr, pid);

				GSL_MMU_STATS(mmu->stats.pt.switches++);
			}
			else
			{
				status = GSL_FAILURE;
			}
		}
#endif // GSL_MMU_PAGETABLE_PERPROCESS

        // if needed, invalidate device specific tlb
		if ((mmu->flags & GSL_MMUFLAGS_TLBFLUSH) && status == GSL_SUCCESS)
        {
            mmu->flags &= ~GSL_MMUFLAGS_TLBFLUSH;

            GSL_TLBFLUSH_FILTER_RESET();

			status = mmu->device->ftbl.mmu_tlbinvalidate(mmu->device, gsl_cfg_mmu_reg[devindex].INVALIDATE, pid);

			GSL_MMU_STATS(mmu->stats.tlbflushes++);
		}
	}

    GSL_MMU_UNLOCK();

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_mmu_setpagetable. Return value %B\n", status );

    return (status);
}

//----------------------------------------------------------------------------

int
kgsl_mmu_init(gsl_device_t *device)
{
    //
    // intialize device mmu
    //
    // call this with the global lock held
    //
    int              status;
    gsl_flags_t      flags;
    gsl_pagetable_t  *pagetable;
    unsigned int     devindex = device->id-1;       // device_id is 1 based
    gsl_mmu_t        *mmu     = &device->mmu;
#ifdef _DEBUG
    gsl_mmu_debug_t  regs;
#endif // _DEBUG

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE,
                    "--> int kgsl_mmu_init(gsl_device_t *device=0x%08x)\n", device );

    if (device->ftbl.mmu_tlbinvalidate == NULL || device->ftbl.mmu_setpagetable == NULL ||
        !(device->flags & GSL_FLAGS_INITIALIZED))
    {
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_mmu_init. Return value %B\n", GSL_FAILURE );
        return (GSL_FAILURE);
    }

    if (mmu->flags & GSL_FLAGS_INITIALIZED0)
    {
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_INFO, "MMU already initialized.\n" );
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_mmu_init. Return value %B\n", GSL_SUCCESS );
        return (GSL_SUCCESS);
    }

    // setup backward reference
    mmu->device = device;

    // disable MMU when running in safe mode
    if (device->flags & GSL_FLAGS_SAFEMODE)
    {
        mmu->config = 0x00000000;
    }

    // setup MMU and sub-client behavior
    device->ftbl.device_regwrite(device, gsl_cfg_mmu_reg[devindex].CONFIG, mmu->config);

    // enable axi interrupts
    kgsl_intr_attach(&device->intr, gsl_cfg_mh_intr[devindex].AXI_READ_ERROR, kgsl_mh_intrcallback, (void *) mmu);
    kgsl_intr_attach(&device->intr, gsl_cfg_mh_intr[devindex].AXI_WRITE_ERROR, kgsl_mh_intrcallback, (void *) mmu);
    kgsl_intr_enable(&device->intr, gsl_cfg_mh_intr[devindex].AXI_READ_ERROR);
    kgsl_intr_enable(&device->intr, gsl_cfg_mh_intr[devindex].AXI_WRITE_ERROR);

    mmu->refcnt  = 0;
    mmu->flags  |= GSL_FLAGS_INITIALIZED0;

    // MMU enabled
    if (mmu->config & 0x1)
    {
        // idle device
        device->ftbl.device_idle(device, GSL_TIMEOUT_DEFAULT);

        // make sure aligned to pagesize
        KOS_ASSERT((mmu->mpu_base & ((1 << GSL_PAGESIZE_SHIFT)-1)) == 0);
        KOS_ASSERT(((mmu->mpu_base + mmu->mpu_range) & ((1 << GSL_PAGESIZE_SHIFT)-1)) == 0);

        // define physical memory range accessible by the core
        device->ftbl.device_regwrite(device, gsl_cfg_mmu_reg[devindex].MPU_BASE, mmu->mpu_base);
        device->ftbl.device_regwrite(device, gsl_cfg_mmu_reg[devindex].MPU_END,  mmu->mpu_base + mmu->mpu_range);

        // enable page fault interrupt
        kgsl_intr_attach(&device->intr, gsl_cfg_mh_intr[devindex].MMU_PAGE_FAULT, kgsl_mh_intrcallback, (void *) mmu);
        kgsl_intr_enable(&device->intr, gsl_cfg_mh_intr[devindex].MMU_PAGE_FAULT);

        mmu->flags |= GSL_FLAGS_INITIALIZED;

        // sub-client MMU lookups require address translation
        if ((mmu->config & ~0x1) > 0)
        {
            GSL_MMU_MUTEX_CREATE();

            // make sure virtual address range is a multiple of 64Kb
            KOS_ASSERT((mmu->va_range & ((1 << 16)-1)) == 0);

            // setup pagetable object
            pagetable = kgsl_mmu_createpagetableobject(mmu, GSL_CALLER_PROCESSID_GET());
            if (!pagetable)
            {
                kgsl_mmu_close(device);
                kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_mmu_init. Return value %B\n", GSL_FAILURE );
                return (GSL_FAILURE);
            }

            mmu->hwpagetable = pagetable;

            // create tlb flush filter to track dirty superPTE's -- one bit per superPTE
            mmu->tlbflushfilter.size = (mmu->va_range / (GSL_PAGESIZE * GSL_PT_SUPER_PTE * 8)) + 1;
            mmu->tlbflushfilter.base = (unsigned int *)kos_malloc(mmu->tlbflushfilter.size);
            if (!mmu->tlbflushfilter.base)
            {
                kgsl_mmu_close(device);
                kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_mmu_init. Return value %B\n", GSL_FAILURE );
                return (GSL_FAILURE);
            }

            GSL_TLBFLUSH_FILTER_RESET();

            // set page table base
            device->ftbl.device_regwrite(device, gsl_cfg_mmu_reg[devindex].PT_BASE, mmu->hwpagetable->base.gpuaddr);

            // define virtual address range
            device->ftbl.device_regwrite(device, gsl_cfg_mmu_reg[devindex].VA_RANGE, (mmu->hwpagetable->va_base | (mmu->hwpagetable->va_range >> 16)));

            // allocate memory used for completing r/w operations that cannot be mapped by the MMU
            flags  = (GSL_MEMFLAGS_ALIGN32 | GSL_MEMFLAGS_CONPHYS | GSL_MEMFLAGS_STRICTREQUEST);
            status = kgsl_sharedmem_alloc0(device->id, flags, 32, &mmu->dummyspace);
            if (status != GSL_SUCCESS)
            {
                kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_ERROR, "ERROR: Unable to allocate dummy space memory.\n" );
                kgsl_mmu_close(device);
                kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_mmu_init. Return value %B\n", status );
                return (status);
            }

            device->ftbl.device_regwrite(device, gsl_cfg_mmu_reg[devindex].TRAN_ERROR, mmu->dummyspace.gpuaddr);

            // call device specific tlb invalidate
			device->ftbl.mmu_tlbinvalidate(device, gsl_cfg_mmu_reg[devindex].INVALIDATE, mmu->hwpagetable->pid);

			GSL_MMU_STATS(mmu->stats.tlbflushes++);

            mmu->flags |= GSL_FLAGS_STARTED;
        }
    }

    KGSL_DEBUG(GSL_DBGFLAGS_MMU, kgsl_mmu_debug(&device->mmu, &regs));

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_mmu_init. Return value %B\n", GSL_SUCCESS );

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

int
kgsl_mmu_map(gsl_mmu_t *mmu, gpuaddr_t gpubaseaddr, const gsl_scatterlist_t *scatterlist, gsl_flags_t flags, unsigned int pid)
{
    //
    // map physical pages into the gpu page table
    //
    int              status = GSL_SUCCESS;
    unsigned int     i, phyaddr, ap;
    unsigned int     pte, ptefirst, ptelast, superpte;
    int              flushtlb;
    gsl_pagetable_t  *pagetable;

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE,
                    "--> int kgsl_mmu_map(gsl_mmu_t *mmu=0x%08x, gpuaddr_t gpubaseaddr=0x%08x, gsl_scatterlist_t *scatterlist=%M, gsl_flags_t flags=%d, unsigned int pid=%d)\n",
                    mmu, gpubaseaddr, scatterlist, flags, pid );

    KOS_ASSERT(scatterlist);

    if (scatterlist->num <= 0)
    {
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_ERROR, "ERROR: num pages is too small.\n" );
        KOS_ASSERT(0);
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_mmu_map. Return value %B\n", GSL_FAILURE );
        return (GSL_FAILURE);
    }

    // get gpu access permissions
    ap = GSL_PT_PAGE_AP[((flags & GSL_MEMFLAGS_GPUAP_MASK) >> GSL_MEMFLAGS_GPUAP_SHIFT)];

    GSL_MMU_LOCK();

    pagetable = kgsl_mmu_getpagetableobject(mmu, pid);
    if (!pagetable)
    {
        GSL_MMU_UNLOCK();
        return (GSL_FAILURE);
    }

    // check consistency, debug only
    KGSL_DEBUG(GSL_DBGFLAGS_MMU, kgsl_mmu_checkconsistency(pagetable));

    ptefirst = GSL_PT_ENTRY_GET(gpubaseaddr);
    ptelast  = GSL_PT_ENTRY_GET(gpubaseaddr + (GSL_PAGESIZE * (scatterlist->num-1)));
    flushtlb = 0;

    if (!GSL_PT_MAP_GETADDR(ptefirst))
    {
        // tlb needs to be flushed when the first and last pte are not at superpte boundaries
        if ((ptefirst & (GSL_PT_SUPER_PTE-1)) != 0 || ((ptelast+1) & (GSL_PT_SUPER_PTE-1)) != 0)
        {
            flushtlb = 1;
        }

        // create page table entries
        for (pte = ptefirst; pte <= ptelast; pte++)
        {
            if (scatterlist->contiguous)
            {
                phyaddr = scatterlist->pages[0] + ((pte-ptefirst) * GSL_PAGESIZE);
            }
            else
            {
                phyaddr = scatterlist->pages[pte-ptefirst];
            }

            GSL_PT_MAP_SETADDR(pte, phyaddr);
            GSL_PT_MAP_SETBITS(pte, ap);

            // tlb needs to be flushed when a dirty superPTE gets backed
            if ((pte & (GSL_PT_SUPER_PTE-1)) == 0)
            {
                if (GSL_TLBFLUSH_FILTER_ISDIRTY(pte / GSL_PT_SUPER_PTE))
                {
                    flushtlb = 1;
                }
            }

            KGSL_DEBUG(GSL_DBGFLAGS_DUMPX, KGSL_DEBUG_DUMPX(BB_DUMP_SET_MMUTBL, pte , *(unsigned int*)(((char*)pagetable->base.hostptr) + (pte * GSL_PT_ENTRY_SIZEBYTES)), 0, "kgsl_mmu_map"));
        }

        if (flushtlb)
        {
            // every device's tlb needs to be flushed because the current page table is shared among all devices
            for (i = 0; i < GSL_DEVICE_MAX; i++)
            {
                if (gsl_driver.device[i].flags & GSL_FLAGS_INITIALIZED)
                {
                    gsl_driver.device[i].mmu.flags |= GSL_MMUFLAGS_TLBFLUSH;
                }
            }
        }

        // determine new last mapped superPTE 
        superpte = ptelast - (ptelast & (GSL_PT_SUPER_PTE-1));
        if (superpte > pagetable->last_superpte)
        {
            pagetable->last_superpte = superpte;
        }

		GSL_MMU_STATS(mmu->stats.pt.maps++);
    }
    else
    {
        // this should never happen
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_FATAL, "FATAL: This should never happen.\n" );
        KOS_ASSERT(0);
        status = GSL_FAILURE;
    }

    GSL_MMU_UNLOCK();

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_mmu_map. Return value %B\n", GSL_SUCCESS );

    return (status);
}

//----------------------------------------------------------------------------

int
kgsl_mmu_unmap(gsl_mmu_t *mmu, gpuaddr_t gpubaseaddr, int range, unsigned int pid)
{
    //
    // remove mappings in the specified address range from the gpu page table
    //
    int              status = GSL_SUCCESS;
    gsl_pagetable_t  *pagetable;
    unsigned int     numpages;
    unsigned int     pte, ptefirst, ptelast, superpte;

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE,
                    "--> int kgsl_mmu_unmap(gsl_mmu_t *mmu=0x%08x, gpuaddr_t gpubaseaddr=0x%08x, int range=%d, unsigned int pid=%d)\n",
                    mmu, gpubaseaddr, range, pid );

    if (range <= 0)
    {
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_ERROR, "ERROR: Range is too small.\n" );
        KOS_ASSERT(0);
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_mmu_unmap. Return value %B\n", GSL_FAILURE );
        return (GSL_FAILURE);
    }

    numpages = (range >> GSL_PAGESIZE_SHIFT);
    if (range & (GSL_PAGESIZE-1))
    {
        numpages++;
    }

    GSL_MMU_LOCK();

    pagetable = kgsl_mmu_getpagetableobject(mmu, pid);
    if (!pagetable)
    {
        GSL_MMU_UNLOCK();
        return (GSL_FAILURE);
    }

    // check consistency, debug only
    KGSL_DEBUG(GSL_DBGFLAGS_MMU, kgsl_mmu_checkconsistency(pagetable));

    ptefirst = GSL_PT_ENTRY_GET(gpubaseaddr);
    ptelast  = GSL_PT_ENTRY_GET(gpubaseaddr + (GSL_PAGESIZE * (numpages-1)));

    if (GSL_PT_MAP_GETADDR(ptefirst))
    {
        superpte = ptefirst - (ptefirst & (GSL_PT_SUPER_PTE-1));
        GSL_TLBFLUSH_FILTER_SETDIRTY(superpte / GSL_PT_SUPER_PTE);

        // remove page table entries
        for (pte = ptefirst; pte <= ptelast; pte++)
        {
            GSL_PT_MAP_RESET(pte);

            superpte = pte - (pte & (GSL_PT_SUPER_PTE-1));
            if (pte == superpte)
            {
                GSL_TLBFLUSH_FILTER_SETDIRTY(superpte / GSL_PT_SUPER_PTE);
            }

			KGSL_DEBUG(GSL_DBGFLAGS_DUMPX, KGSL_DEBUG_DUMPX(BB_DUMP_SET_MMUTBL, pte, *(unsigned int*)(((char*)pagetable->base.hostptr) + (pte * GSL_PT_ENTRY_SIZEBYTES)), 0, "kgsl_mmu_unmap, reset superPTE"));
        }

        // determine new last mapped superPTE 
        superpte = ptelast - (ptelast & (GSL_PT_SUPER_PTE-1));
        if (superpte == pagetable->last_superpte && pagetable->last_superpte >= GSL_PT_SUPER_PTE)
        {
            do
            {
                pagetable->last_superpte -= GSL_PT_SUPER_PTE;
            } while (!GSL_PT_MAP_GETADDR(pagetable->last_superpte) && pagetable->last_superpte >= GSL_PT_SUPER_PTE);
        }

		GSL_MMU_STATS(mmu->stats.pt.unmaps++);
    }
    else
    {
        // this should never happen
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_FATAL, "FATAL: This should never happen.\n" );
        KOS_ASSERT(0);
        status = GSL_FAILURE;
    }

    // invalidate tlb, debug only
	KGSL_DEBUG(GSL_DBGFLAGS_MMU, mmu->device->ftbl.mmu_tlbinvalidate(mmu->device, gsl_cfg_mmu_reg[mmu->device->id-1].INVALIDATE, pagetable->pid));

    GSL_MMU_UNLOCK();

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_mmu_unmap. Return value %B\n", GSL_SUCCESS );

    return (status);
}

//----------------------------------------------------------------------------

int         
kgsl_mmu_getmap(gsl_mmu_t *mmu, gpuaddr_t gpubaseaddr, int range, gsl_scatterlist_t *scatterlist, unsigned int pid)
{
    //
    // obtain scatter list of physical pages for the given gpu address range.
    // if all pages are physically contiguous they are coalesced into a single
    // scatterlist entry.
    //
    gsl_pagetable_t  *pagetable;
    unsigned int     numpages;
    unsigned int     pte, ptefirst, ptelast;
    unsigned int     contiguous = 1;

    numpages = (range >> GSL_PAGESIZE_SHIFT);
    if (range & (GSL_PAGESIZE-1))
    {
        numpages++;
    }

    if (range <= 0 || scatterlist->num != numpages)
    {
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_ERROR, "ERROR: Range is too small.\n" );
        KOS_ASSERT(0);
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_mmu_getmap. Return value %B\n", GSL_FAILURE );
        return (GSL_FAILURE);
    }

    GSL_MMU_LOCK();

    pagetable = kgsl_mmu_getpagetableobject(mmu, pid);
    if (!pagetable)
    {
        GSL_MMU_UNLOCK();
        return (GSL_FAILURE);
    }

    ptefirst = GSL_PT_ENTRY_GET(gpubaseaddr);
    ptelast  = GSL_PT_ENTRY_GET(gpubaseaddr + (GSL_PAGESIZE * (numpages-1)));

    // determine whether pages are physically contiguous
    if (numpages > 1)
    {
        for (pte = ptefirst; pte <= ptelast-1; pte++)
        {
            if (GSL_PT_MAP_GETADDR(pte) + GSL_PAGESIZE != GSL_PT_MAP_GETADDR(pte+1))
            {
                contiguous = 0;
                break;
            }
        }
    }

    if (!contiguous)
    {
        // populate scatter list
        for (pte = ptefirst; pte <= ptelast; pte++)
        {
            scatterlist->pages[pte-ptefirst] = GSL_PT_MAP_GETADDR(pte);
        }
    }
    else
    {
        // coalesce physically contiguous pages into a single scatter list entry
        scatterlist->pages[0] = GSL_PT_MAP_GETADDR(ptefirst);
    }

    GSL_MMU_UNLOCK();

    scatterlist->contiguous = contiguous;

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

int
kgsl_mmu_close(gsl_device_t *device)
{
    //
    // close device mmu
    //
    // call this with the global lock held
    //
    gsl_mmu_t     *mmu     = &device->mmu;
    unsigned int  devindex = mmu->device->id-1;         // device_id is 1 based
#ifdef _DEBUG
    int           i;
#endif // _DEBUG

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "--> int kgsl_mmu_close(gsl_device_t *device=0x%08x)\n", device );

    if (mmu->flags & GSL_FLAGS_INITIALIZED0)
    {
        if (mmu->flags & GSL_FLAGS_STARTED)
        {
            // terminate pagetable object
            kgsl_mmu_destroypagetableobject(mmu, GSL_CALLER_PROCESSID_GET());
        }

        // no more processes attached to current device mmu
        if (mmu->refcnt == 0)
        {
#ifdef _DEBUG
            // check if there are any orphaned pagetable objects lingering around
            for (i = 0; i < GSL_MMU_PAGETABLE_MAX; i++)
            {
                if (mmu->pagetable[i])
                {
                    /* many dumpx test cases forcefully exit, and thus trigger this assert. */
                    /* Because it is an annoyance for HW guys, it is disabled for dumpx */
                    if(!gsl_driver.flags_debug & GSL_DBGFLAGS_DUMPX)
                    {
                        KOS_ASSERT(0);
                        return (GSL_FAILURE);
                    }
                }
            }
#endif // _DEBUG

            // disable mh interrupts
            kgsl_intr_detach(&device->intr, gsl_cfg_mh_intr[devindex].AXI_READ_ERROR);
            kgsl_intr_detach(&device->intr, gsl_cfg_mh_intr[devindex].AXI_WRITE_ERROR);
            kgsl_intr_detach(&device->intr, gsl_cfg_mh_intr[devindex].MMU_PAGE_FAULT);

            // disable MMU
            device->ftbl.device_regwrite(device, gsl_cfg_mmu_reg[devindex].CONFIG, 0x00000000);

            if (mmu->tlbflushfilter.base)
            {
                kos_free(mmu->tlbflushfilter.base);
            }

            if (mmu->dummyspace.gpuaddr)
            {
                kgsl_sharedmem_free0(&mmu->dummyspace, GSL_CALLER_PROCESSID_GET());
            }

            GSL_MMU_MUTEX_FREE();

            mmu->flags &= ~GSL_FLAGS_STARTED;
            mmu->flags &= ~GSL_FLAGS_INITIALIZED;
            mmu->flags &= ~GSL_FLAGS_INITIALIZED0;
        }
    }

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_mmu_close. Return value %B\n", GSL_SUCCESS );

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

int
kgsl_mmu_attachcallback(gsl_mmu_t *mmu, unsigned int pid)
{
    //
    // attach process
    //
    // call this with the global lock held
    //
    int              status = GSL_SUCCESS;
    gsl_pagetable_t  *pagetable;

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "--> int kgsl_mmu_attachcallback(gsl_mmu_t *mmu=0x%08x, unsigned int pid=0x%08x)\n", mmu, pid );

    GSL_MMU_LOCK();

    if (mmu->flags & GSL_FLAGS_INITIALIZED0)
    {
        // attach to current device mmu
        mmu->refcnt++;

        if (mmu->flags & GSL_FLAGS_STARTED)
        {
            // attach to pagetable object
            pagetable = kgsl_mmu_createpagetableobject(mmu, pid);
            if(pagetable)
            {
                pagetable->refcnt++;
            }
            else
            {
                status = GSL_FAILURE;
            }
        }
    }

    GSL_MMU_UNLOCK();

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_mmu_attachcallback. Return value %B\n", status );

    return (status);
}

//----------------------------------------------------------------------------

int
kgsl_mmu_detachcallback(gsl_mmu_t *mmu, unsigned int pid)
{
    //
    // detach process
    //
    int              status = GSL_SUCCESS;
    gsl_pagetable_t  *pagetable;

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "--> int kgsl_mmu_detachcallback(gsl_mmu_t *mmu=0x%08x, unsigned int pid=0x%08x)\n", mmu, pid );

    GSL_MMU_LOCK();

    if (mmu->flags & GSL_FLAGS_INITIALIZED0)
    {
        // detach from current device mmu
        mmu->refcnt--;

        if (mmu->flags & GSL_FLAGS_STARTED)
        {
            // detach from pagetable object
            pagetable = kgsl_mmu_getpagetableobject(mmu, pid);
            if(pagetable)
            {
                pagetable->refcnt--;
            }
            else
            {
                status = GSL_FAILURE;
            }
        }
    }

    GSL_MMU_UNLOCK();

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_mmu_detachcallback. Return value %B\n", status );

    return (status);
}

//----------------------------------------------------------------------------

int
kgsl_mmu_querystats(gsl_mmu_t *mmu, gsl_mmustats_t *stats)
{
#ifdef GSL_STATS_MMU
    int              status = GSL_SUCCESS;

    KOS_ASSERT(stats);

    GSL_MMU_LOCK();

    if (mmu->flags & GSL_FLAGS_STARTED)
    {
		kos_memcpy(stats, &mmu->stats, sizeof(gsl_mmustats_t));
    }
    else
    {
		kos_memset(stats, 0, sizeof(gsl_mmustats_t));
    }

    GSL_MMU_UNLOCK();

    return (status);
#else
    // unreferenced formal parameters
    (void) mmu;
    (void) stats;

    return (GSL_FAILURE_NOTSUPPORTED);
#endif // GSL_STATS_MMU
}

//----------------------------------------------------------------------------

int
kgsl_mmu_bist(gsl_mmu_t *mmu)
{
    // unreferenced formal parameter
    (void) mmu;

    return (GSL_SUCCESS);
}
