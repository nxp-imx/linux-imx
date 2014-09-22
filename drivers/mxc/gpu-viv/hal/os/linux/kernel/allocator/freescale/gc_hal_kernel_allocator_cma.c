/****************************************************************************
*
*    Copyright (C) 2005 - 2014 by Vivante Corp.
*
*    This program is free software; you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation; either version 2 of the license, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program; if not write to the Free Software
*    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*
*****************************************************************************/


#include "gc_hal_kernel_linux.h"
#include "gc_hal_kernel_allocator.h"

#include <linux/pagemap.h>
#include <linux/seq_file.h>
#include <linux/mman.h>
#include <asm/atomic.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

typedef struct _gcsCMA_PRIV * gcsCMA_PRIV_PTR;
typedef struct _gcsCMA_PRIV {
    gctUINT32 cmasize;
}
gcsCMA_PRIV;

struct mdl_cma_priv {
    gctPOINTER kvaddr;
    dma_addr_t physical;
};

int gc_cma_usage_show(struct seq_file* m, void* data)
{
    gcsINFO_NODE *node = m->private;
    gckALLOCATOR Allocator = node->device;
    gcsCMA_PRIV_PTR priv = Allocator->privateData;

    seq_printf(m, "cma:  %u bytes\n", priv->cmasize);

    return 0;
}

static gcsINFO InfoList[] =
{
    {"cmausage", gc_cma_usage_show},
};

static void
_DefaultAllocatorDebugfsInit(
    IN gckALLOCATOR Allocator,
    IN gckDEBUGFS_DIR Root
    )
{
    gcmkVERIFY_OK(
        gckDEBUGFS_DIR_Init(&Allocator->debugfsDir, Root->root, "cma"));

    gcmkVERIFY_OK(gckDEBUGFS_DIR_CreateFiles(
        &Allocator->debugfsDir,
        InfoList,
        gcmCOUNTOF(InfoList),
        Allocator
        ));
}

static void
_DefaultAllocatorDebugfsCleanup(
    IN gckALLOCATOR Allocator
    )
{
    gcmkVERIFY_OK(gckDEBUGFS_DIR_RemoveFiles(
        &Allocator->debugfsDir,
        InfoList,
        gcmCOUNTOF(InfoList)
        ));

    gckDEBUGFS_DIR_Deinit(&Allocator->debugfsDir);
}

static gceSTATUS
_CMAFSLAlloc(
    IN gckALLOCATOR Allocator,
    INOUT PLINUX_MDL Mdl,
    IN gctSIZE_T NumPages,
    IN gctUINT32 Flags
    )
{
    gceSTATUS status;
    gctPOINTER addr = gcvNULL;
    gcsCMA_PRIV_PTR priv = (gcsCMA_PRIV_PTR)Allocator->privateData;

    struct mdl_cma_priv *mdl_priv=gcvNULL;
    gckOS os = Allocator->os;

    gcmkHEADER_ARG("Mdl=%p NumPages=%d", Mdl, NumPages);

    gcmkONERROR(gckOS_Allocate(os, sizeof(struct mdl_cma_priv), (gctPOINTER *)&mdl_priv));

    mdl_priv->kvaddr = dma_alloc_writecombine(gcvNULL,
            NumPages * PAGE_SIZE,
            &mdl_priv->physical,
            GFP_KERNEL | gcdNOWARN);

    if (addr == gcvNULL)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    Mdl->priv = mdl_priv;
    priv->cmasize += NumPages * PAGE_SIZE;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if(mdl_priv)
        gckOS_Free(os, mdl_priv);
    gcmkFOOTER();
    return status;
}

static void
_CMAFSLFree(
    IN gckALLOCATOR Allocator,
    IN OUT PLINUX_MDL Mdl
    )
{
    gckOS os = Allocator->os;
    struct mdl_cma_priv *mdl_priv=(struct mdl_cma_priv *)Mdl->priv;
    gcsCMA_PRIV_PTR priv = (gcsCMA_PRIV_PTR)Allocator->privateData;
    dma_free_writecombine(gcvNULL,
            Mdl->numPages * PAGE_SIZE,
            mdl_priv->kvaddr,
            mdl_priv->physical);
     gckOS_Free(os, mdl_priv);
    priv->cmasize -= Mdl->numPages * PAGE_SIZE;
}

gctINT
_CMAFSLMapUser(
    gckALLOCATOR Allocator,
    PLINUX_MDL Mdl,
    PLINUX_MDL_MAP MdlMap,
    gctBOOL Cacheable
    )
{

    PLINUX_MDL      mdl = Mdl;
    PLINUX_MDL_MAP  mdlMap = MdlMap;
    struct mdl_cma_priv *mdl_priv=(struct mdl_cma_priv *)Mdl->priv;

    gcmkHEADER_ARG("Allocator=%p Mdl=%p MdlMap=%p gctBOOL=%d", Allocator, Mdl, MdlMap, Cacheable);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)
    mdlMap->vmaAddr = (gctSTRING)vm_mmap(gcvNULL,
                    0L,
                    mdl->numPages * PAGE_SIZE,
                    PROT_READ | PROT_WRITE,
                    MAP_SHARED,
                    0);
#else
    down_write(&current->mm->mmap_sem);

    mdlMap->vmaAddr = (gctSTRING)do_mmap_pgoff(gcvNULL,
                    0L,
                    mdl->numPages * PAGE_SIZE,
                    PROT_READ | PROT_WRITE,
                    MAP_SHARED,
                    0);

    up_write(&current->mm->mmap_sem);
#endif

    gcmkTRACE_ZONE(
        gcvLEVEL_INFO, gcvZONE_OS,
        "%s(%d): vmaAddr->0x%X for phys_addr->0x%X",
        __FUNCTION__, __LINE__,
        (gctUINT32)(gctUINTPTR_T)mdlMap->vmaAddr,
        (gctUINT32)(gctUINTPTR_T)mdl
        );

    if (IS_ERR(mdlMap->vmaAddr))
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_INFO, gcvZONE_OS,
            "%s(%d): do_mmap_pgoff error",
            __FUNCTION__, __LINE__
            );

        mdlMap->vmaAddr = gcvNULL;

        gcmkFOOTER_ARG("*status=%d", gcvSTATUS_OUT_OF_MEMORY);
        return gcvSTATUS_OUT_OF_MEMORY;
    }

    down_write(&current->mm->mmap_sem);

    mdlMap->vma = find_vma(current->mm, (unsigned long)mdlMap->vmaAddr);

    if (mdlMap->vma == gcvNULL)
    {
        up_write(&current->mm->mmap_sem);

        gcmkTRACE_ZONE(
            gcvLEVEL_INFO, gcvZONE_OS,
            "%s(%d): find_vma error",
            __FUNCTION__, __LINE__
            );

        mdlMap->vmaAddr = gcvNULL;

        gcmkFOOTER_ARG("*status=%d", gcvSTATUS_OUT_OF_RESOURCES);
        return gcvSTATUS_OUT_OF_RESOURCES;
    }

    /* Now map all the vmalloc pages to this user address. */
    if (mdl->contiguous)
    {
        /* map kernel memory to user space.. */
        if (dma_mmap_writecombine(gcvNULL,
                mdlMap->vma,
                mdl_priv->kvaddr,
                mdl_priv->physical,
                mdl->numPages * PAGE_SIZE) < 0)
        {
            up_write(&current->mm->mmap_sem);

            gcmkTRACE_ZONE(
                gcvLEVEL_WARNING, gcvZONE_OS,
                "%s(%d): dma_mmap_attrs error",
                __FUNCTION__, __LINE__
                );

             mdlMap->vmaAddr = gcvNULL;

            gcmkFOOTER_ARG("*status=%d", gcvSTATUS_OUT_OF_MEMORY);
            return gcvSTATUS_OUT_OF_MEMORY;
        }
    }
    else
    {
        gckOS_Print("incorrect mdl:conti%d\n",mdl->contiguous);
    }

    up_write(&current->mm->mmap_sem);

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

void
_CMAUnmapUser(
    IN gckALLOCATOR Allocator,
    IN gctPOINTER Logical,
    IN gctUINT32 Size
    )
{
    if (unlikely(current->mm == gcvNULL))
    {
        /* Do nothing if process is exiting. */
        return;
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0)
    if (vm_munmap((unsigned long)Logical, Size) < 0)
    {
        gcmkTRACE_ZONE(
                gcvLEVEL_WARNING, gcvZONE_OS,
                "%s(%d): vm_munmap failed",
                __FUNCTION__, __LINE__
                );
    }
#else
    down_write(&current->mm->mmap_sem);
    if (do_munmap(current->mm, (unsigned long)Logical, Size) < 0)
    {
        gcmkTRACE_ZONE(
                gcvLEVEL_WARNING, gcvZONE_OS,
                "%s(%d): do_munmap failed",
                __FUNCTION__, __LINE__
                );
    }
    up_write(&current->mm->mmap_sem);
#endif
}

gceSTATUS
_CMAMapKernel(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    OUT gctPOINTER *Logical
    )
{
    struct mdl_cma_priv *mdl_priv=(struct mdl_cma_priv *)Mdl->priv;
    *Logical =mdl_priv->kvaddr;
    return gcvSTATUS_OK;
}

gceSTATUS
_CMAUnmapKernel(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctPOINTER Logical
    )
{
    return gcvSTATUS_OK;
}

extern gceSTATUS
_DefaultLogicalToPhysical(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctPOINTER Logical,
    IN gctUINT32 ProcessID,
    OUT gctUINT32_PTR Physical
    );

extern gceSTATUS
_DefaultCache(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctPOINTER Logical,
    IN gctUINT32 Physical,
    IN gctUINT32 Bytes,
    IN gceCACHEOPERATION Operation
    );

gceSTATUS
_CMAPhysical(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctUINT32 Offset,
    OUT gctUINT32_PTR Physical
    )
{
    struct mdl_cma_priv *mdl_priv=(struct mdl_cma_priv *)Mdl->priv;
     gcmkASSERT(!Offset);
    *Physical = mdl_priv->physical;

    return gcvSTATUS_OK;
}


extern void
_DefaultAllocatorDestructor(
    IN void* PrivateData
    );

/* Default allocator operations. */
gcsALLOCATOR_OPERATIONS CMAFSLAllocatorOperations = {
    .Alloc              = _CMAFSLAlloc,
    .Free               = _CMAFSLFree,
    .MapUser            = _CMAFSLMapUser,
    .UnmapUser          = _CMAUnmapUser,
    .MapKernel          = _CMAMapKernel,
    .UnmapKernel        = _CMAUnmapKernel,
    .LogicalToPhysical  = _DefaultLogicalToPhysical,
    .Cache              = _DefaultCache,
    .Physical           = _CMAPhysical,
};

/* Default allocator entry. */
gceSTATUS
_CMAFSLAlloctorInit(
    IN gckOS Os,
    OUT gckALLOCATOR * Allocator
    )
{
    gceSTATUS status;
    gckALLOCATOR allocator;
    gcsCMA_PRIV_PTR priv = gcvNULL;

    gcmkONERROR(
        gckALLOCATOR_Construct(Os, &CMAFSLAllocatorOperations, &allocator));

    priv = kzalloc(gcmSIZEOF(gcsCMA_PRIV), GFP_KERNEL | gcdNOWARN);

    if (!priv)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    /* Register private data. */
    allocator->privateData = priv;
    allocator->privateDataDestructor = _DefaultAllocatorDestructor;

    allocator->debugfsInit = _DefaultAllocatorDebugfsInit;
    allocator->debugfsCleanup = _DefaultAllocatorDebugfsCleanup;

    allocator->capability = gcvALLOC_FLAG_CONTIGUOUS;

    *Allocator = allocator;

    return gcvSTATUS_OK;

OnError:
    return status;
}

