/****************************************************************************
*
*    Copyright (C) 2005 - 2012 by Vivante Corp.
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




#include "gc_hal_kernel_precomp.h"

#define _GC_OBJ_ZONE    gcvZONE_MMU

typedef enum _gceMMU_TYPE
{
    gcvMMU_USED     = (0 << 4),
    gcvMMU_SINGLE   = (1 << 4),
    gcvMMU_FREE     = (2 << 4),
}
gceMMU_TYPE;

#define gcmENTRY_TYPE(x) (x & 0xF0)

#define gcdMMU_TABLE_DUMP       0

/*
    gcdMMU_CLEAR_VALUE

        The clear value for the entry of the old MMU.
*/
#ifndef gcdMMU_CLEAR_VALUE
#   define gcdMMU_CLEAR_VALUE                   0x00000ABC
#endif

typedef struct _gcsMMU_STLB *gcsMMU_STLB_PTR;

typedef struct _gcsMMU_STLB
{
    gctPHYS_ADDR    physical;
    gctUINT32_PTR   logical;
    gctSIZE_T       size;
    gctUINT32       physBase;
    gctSIZE_T       pageCount;
    gctUINT32       mtlbIndex;
    gctUINT32       mtlbEntryNum;
    gcsMMU_STLB_PTR next;
} gcsMMU_STLB;

#if gcdSHARED_PAGETABLE
typedef struct _gcsSharedPageTable * gcsSharedPageTable_PTR;
typedef struct _gcsSharedPageTable
{
    /* Shared gckMMU object. */
    gckMMU          mmu;

    /* Hardwares which use this shared pagetable. */
    gckHARDWARE     hardwares[gcdCORE_COUNT];

    /* Number of cores use this shared pagetable. */
    gctUINT32       reference;
}
gcsSharedPageTable;

static gcsSharedPageTable_PTR sharedPageTable = gcvNULL;
#endif

static gceSTATUS
_Link(
    IN gckMMU Mmu,
    IN gctUINT32 Index,
    IN gctUINT32 Next
    )
{
    if (Index >= Mmu->pageTableEntries)
    {
        /* Just move heap pointer. */
        Mmu->heapList = Next;
    }
    else
    {
        /* Address page table. */
        gctUINT32_PTR pageTable = Mmu->pageTableLogical;

        /* Dispatch on node type. */
        switch (gcmENTRY_TYPE(pageTable[Index]))
        {
        case gcvMMU_SINGLE:
            /* Set single index. */
            pageTable[Index] = (Next << 8) | gcvMMU_SINGLE;
            break;

        case gcvMMU_FREE:
            /* Set index. */
            pageTable[Index + 1] = Next;
            break;

        default:
            gcmkFATAL("MMU table correcupted at index %u!", Index);
            return gcvSTATUS_HEAP_CORRUPTED;
        }
    }

    /* Success. */
    return gcvSTATUS_OK;
}

static gceSTATUS
_AddFree(
    IN gckMMU Mmu,
    IN gctUINT32 Index,
    IN gctUINT32 Node,
    IN gctUINT32 Count
    )
{
    gctUINT32_PTR pageTable = Mmu->pageTableLogical;

    if (Count == 1)
    {
        /* Initialize a single page node. */
        pageTable[Node] = (~((1U<<8)-1)) | gcvMMU_SINGLE;
    }
    else
    {
        /* Initialize the node. */
        pageTable[Node + 0] = (Count << 8) | gcvMMU_FREE;
        pageTable[Node + 1] = ~0U;
    }

    /* Append the node. */
    return _Link(Mmu, Index, Node);
}

static gceSTATUS
_Collect(
    IN gckMMU Mmu
    )
{
    gctUINT32_PTR pageTable = Mmu->pageTableLogical;
    gceSTATUS status;
    gctUINT32 i, previous, start = 0, count = 0;

    previous = Mmu->heapList = ~0U;
    Mmu->freeNodes = gcvFALSE;

    /* Walk the entire page table. */
    for (i = 0; i < Mmu->pageTableEntries; ++i)
    {
        /* Dispatch based on type of page. */
        switch (gcmENTRY_TYPE(pageTable[i]))
        {
        case gcvMMU_USED:
            /* Used page, so close any open node. */
            if (count > 0)
            {
                /* Add the node. */
                gcmkONERROR(_AddFree(Mmu, previous, start, count));

                /* Reset the node. */
                previous = start;
                count    = 0;
            }
            break;

        case gcvMMU_SINGLE:
            /* Single free node. */
            if (count++ == 0)
            {
                /* Start a new node. */
                start = i;
            }
            break;

        case gcvMMU_FREE:
            /* A free node. */
            if (count == 0)
            {
                /* Start a new node. */
                start = i;
            }

            /* Advance the count. */
            count += pageTable[i] >> 8;

            /* Advance the index into the page table. */
            i     += (pageTable[i] >> 8) - 1;
            break;

        default:
            gcmkFATAL("MMU page table correcupted at index %u!", i);
            return gcvSTATUS_HEAP_CORRUPTED;
        }
    }

    /* See if we have an open node left. */
    if (count > 0)
    {
        /* Add the node to the list. */
        gcmkONERROR(_AddFree(Mmu, previous, start, count));
    }

    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_MMU,
                   "Performed a garbage collection of the MMU heap.");

    /* Success. */
    return gcvSTATUS_OK;

OnError:
    /* Return the staus. */
    return status;
}

static gctUINT32
_SetPage(gctUINT32 PageAddress)
{
    return PageAddress
           /* writable */
           | (1 << 2)
           /* Ignore exception */
           | (0 << 1)
           /* Present */
           | (1 << 0);
}

static gceSTATUS
_FillFlatMapping(
    IN gckMMU Mmu,
    IN gctUINT32 PhysBase,
    OUT gctSIZE_T Size
    )
{
    gceSTATUS status;
    gctBOOL mutex = gcvFALSE;
    gcsMMU_STLB_PTR head = gcvNULL, pre = gcvNULL;
    gctUINT32 start = PhysBase & (~gcdMMU_PAGE_64K_MASK);
    gctUINT32 end = (PhysBase + Size - 1) & (~gcdMMU_PAGE_64K_MASK);
    gctUINT32 mStart = start >> gcdMMU_MTLB_SHIFT;
    gctUINT32 mEnd = end >> gcdMMU_MTLB_SHIFT;
    gctUINT32 sStart = (start & gcdMMU_STLB_64K_MASK) >> gcdMMU_STLB_64K_SHIFT;
    gctUINT32 sEnd = (end & gcdMMU_STLB_64K_MASK) >> gcdMMU_STLB_64K_SHIFT;

    /* Grab the mutex. */
    gcmkONERROR(gckOS_AcquireMutex(Mmu->os, Mmu->pageTableMutex, gcvINFINITE));
    mutex = gcvTRUE;

    while (mStart <= mEnd)
    {
        gcmkASSERT(mStart < gcdMMU_MTLB_ENTRY_NUM);
        if (*(Mmu->mtlbLogical + mStart) == 0)
        {
            gcsMMU_STLB_PTR stlb;
            gctPOINTER pointer = gcvNULL;
            gctUINT32 last = (mStart == mEnd) ? sEnd : (gcdMMU_STLB_64K_ENTRY_NUM - 1);

            gcmkONERROR(gckOS_Allocate(Mmu->os, sizeof(struct _gcsMMU_STLB), &pointer));
            stlb = pointer;

            stlb->mtlbEntryNum = 0;
            stlb->next = gcvNULL;
            stlb->physical = gcvNULL;
            stlb->logical = gcvNULL;
            stlb->size = gcdMMU_STLB_64K_SIZE;
            stlb->pageCount = 0;

            if (pre == gcvNULL)
            {
                pre = head = stlb;
            }
            else
            {
                gcmkASSERT(pre->next == gcvNULL);
                pre->next = stlb;
                pre = stlb;
            }

            gcmkONERROR(
                    gckOS_AllocateContiguous(Mmu->os,
                                             gcvFALSE,
                                             &stlb->size,
                                             &stlb->physical,
                                             (gctPOINTER)&stlb->logical));

            gcmkONERROR(gckOS_ZeroMemory(stlb->logical, stlb->size));

            gcmkONERROR(gckOS_GetPhysicalAddress(
                Mmu->os,
                stlb->logical,
                &stlb->physBase));

            if (stlb->physBase & (gcdMMU_STLB_64K_SIZE - 1))
            {
                gcmkONERROR(gcvSTATUS_NOT_ALIGNED);
            }

            *(Mmu->mtlbLogical + mStart)
                      = stlb->physBase
                        /* 64KB page size */
                        | (1 << 2)
                        /* Ignore exception */
                        | (0 << 1)
                        /* Present */
                        | (1 << 0);
#if gcdMMU_TABLE_DUMP
            gckOS_Print("%s(%d): insert MTLB[%d]: %08x\n",
                __FUNCTION__, __LINE__,
                mStart,
                *(Mmu->mtlbLogical + mStart));
#endif

            stlb->mtlbIndex = mStart;
            stlb->mtlbEntryNum = 1;
#if gcdMMU_TABLE_DUMP
            gckOS_Print("%s(%d): STLB: logical:%08x -> physical:%08x\n",
                    __FUNCTION__, __LINE__,
                    stlb->logical,
                    stlb->physBase);
#endif

            while (sStart <= last)
            {
                gcmkASSERT(!(start & gcdMMU_PAGE_64K_MASK));
                *(stlb->logical + sStart) = _SetPage(start);
#if gcdMMU_TABLE_DUMP
                gckOS_Print("%s(%d): insert STLB[%d]: %08x\n",
                    __FUNCTION__, __LINE__,
                    sStart,
                    *(stlb->logical + sStart));
#endif
                /* next page. */
                start += gcdMMU_PAGE_64K_SIZE;
                sStart++;
                stlb->pageCount++;
            }

            sStart = 0;
            ++mStart;
        }
        else
        {
            gcmkONERROR(gcvSTATUS_INVALID_REQUEST);
        }
    }

    /* Insert the stlb into staticSTLB. */
    if (Mmu->staticSTLB == gcvNULL)
    {
        Mmu->staticSTLB = head;
    }
    else
    {
        gcmkASSERT(pre == gcvNULL);
        gcmkASSERT(pre->next == gcvNULL);
        pre->next = Mmu->staticSTLB;
        Mmu->staticSTLB = head;
    }

    /* Release the mutex. */
    gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->pageTableMutex));

    return gcvSTATUS_OK;

OnError:

    /* Roll back. */
    while (head != gcvNULL)
    {
        pre = head;
        head = head->next;

        if (pre->physical != gcvNULL)
        {
            gcmkVERIFY_OK(
                gckOS_FreeContiguous(Mmu->os,
                    pre->physical,
                    pre->logical,
                    pre->size));
        }

        if (pre->mtlbEntryNum != 0)
        {
            gcmkASSERT(pre->mtlbEntryNum == 1);
            *(Mmu->mtlbLogical + pre->mtlbIndex) = 0;
        }

        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Mmu->os, pre));
    }

    if (mutex)
    {
        /* Release the mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->pageTableMutex));
    }

    return status;
}

static gceSTATUS
_SetupDynamicSpace(
    IN gckMMU Mmu
    )
{
    gceSTATUS status;
    gctINT i;
    gctUINT32 physical;
    gctINT numEntries;
    gctUINT32_PTR pageTable;
    gctBOOL acquired = gcvFALSE;

    /* find the start of dynamic address space. */
    for (i = 0; i < gcdMMU_MTLB_ENTRY_NUM; i++)
    {
        if (!Mmu->mtlbLogical[i])
        {
            break;
        }
    }

    Mmu->dynamicMappingStart = i;

    /* Number of entries in Master TLB for dynamic mapping. */
    numEntries = gcdMMU_MTLB_ENTRY_NUM - i;

    Mmu->pageTableSize = numEntries * 4096;

    Mmu->pageTableEntries = Mmu->pageTableSize / gcmSIZEOF(gctUINT32);

    /* Construct Slave TLB. */
    gcmkONERROR(gckOS_AllocateContiguous(Mmu->os,
                gcvFALSE,
                &Mmu->pageTableSize,
                &Mmu->pageTablePhysical,
                (gctPOINTER)&Mmu->pageTableLogical));

    /* Invalidate all entries. */
    gcmkONERROR(gckOS_ZeroMemory(Mmu->pageTableLogical,
                Mmu->pageTableSize));

    /* Initilization. */
    pageTable      = Mmu->pageTableLogical;
    pageTable[0]   = (Mmu->pageTableEntries << 8) | gcvMMU_FREE;
    pageTable[1]   = ~0U;
    Mmu->heapList  = 0;
    Mmu->freeNodes = gcvFALSE;

    gcmkONERROR(gckOS_GetPhysicalAddress(Mmu->os,
                Mmu->pageTableLogical,
                &physical));

    /* Grab the mutex. */
    gcmkONERROR(gckOS_AcquireMutex(Mmu->os, Mmu->pageTableMutex, gcvINFINITE));
    acquired = gcvTRUE;

    /* Map to Master TLB. */
    for (; i < gcdMMU_MTLB_ENTRY_NUM; i++)
    {
        Mmu->mtlbLogical[i] = physical
                            /* 4KB page size */
                            | (0 << 2)
                            /* Ignore exception */
                            | (0 << 1)
                            /* Present */
                            | (1 << 0);
#if gcdMMU_TABLE_DUMP
        gckOS_Print("%s(%d): insert MTLB[%d]: %08x\n",
                __FUNCTION__, __LINE__,
                i,
                *(Mmu->mtlbLogical + i));
#endif
        physical += gcdMMU_STLB_4K_SIZE;
    }

    /* Release the mutex. */
    gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->pageTableMutex));

    return gcvSTATUS_OK;

OnError:
    if (Mmu->pageTableLogical)
    {
        /* Free the page table. */
        gcmkVERIFY_OK(
                gckOS_FreeContiguous(Mmu->os,
                    Mmu->pageTablePhysical,
                    (gctPOINTER) Mmu->pageTableLogical,
                    Mmu->pageTableSize));
    }

    if (acquired)
    {
        /* Release the mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->pageTableMutex));
    }

    return status;
}

/*******************************************************************************
**
**  _Construct
**
**  Construct a new gckMMU object.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctSIZE_T MmuSize
**          Number of bytes for the page table.
**
**  OUTPUT:
**
**      gckMMU * Mmu
**          Pointer to a variable that receives the gckMMU object pointer.
*/
gceSTATUS
_Construct(
    IN gckKERNEL Kernel,
    IN gctSIZE_T MmuSize,
    OUT gckMMU * Mmu
    )
{
    gckOS os;
    gckHARDWARE hardware;
    gceSTATUS status;
    gckMMU mmu = gcvNULL;
    gctUINT32_PTR pageTable;
    gctPOINTER pointer = gcvNULL;

    gcmkHEADER_ARG("Kernel=0x%x MmuSize=%lu", Kernel, MmuSize);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(MmuSize > 0);
    gcmkVERIFY_ARGUMENT(Mmu != gcvNULL);

    /* Extract the gckOS object pointer. */
    os = Kernel->os;
    gcmkVERIFY_OBJECT(os, gcvOBJ_OS);

    /* Extract the gckHARDWARE object pointer. */
    hardware = Kernel->hardware;
    gcmkVERIFY_OBJECT(hardware, gcvOBJ_HARDWARE);

    /* Allocate memory for the gckMMU object. */
    gcmkONERROR(gckOS_Allocate(os, sizeof(struct _gckMMU), &pointer));

    mmu = pointer;

    /* Initialize the gckMMU object. */
    mmu->object.type      = gcvOBJ_MMU;
    mmu->os               = os;
    mmu->hardware         = hardware;
    mmu->pageTableMutex   = gcvNULL;
    mmu->pageTableLogical = gcvNULL;
    mmu->mtlbLogical      = gcvNULL;
    mmu->staticSTLB       = gcvNULL;
    mmu->enabled          = gcvFALSE;
#ifdef __QNXNTO__
    mmu->nodeList         = gcvNULL;
    mmu->nodeMutex        = gcvNULL;
#endif

    /* Create the page table mutex. */
    gcmkONERROR(gckOS_CreateMutex(os, &mmu->pageTableMutex));

#ifdef __QNXNTO__
    /* Create the node list mutex. */
    gcmkONERROR(gckOS_CreateMutex(os, &mmu->nodeMutex));
#endif

    if (hardware->mmuVersion == 0)
    {
        mmu->pageTableSize = MmuSize;

        gcmkONERROR(
            gckOS_AllocateContiguous(os,
                                     gcvFALSE,
                                     &mmu->pageTableSize,
                                     &mmu->pageTablePhysical,
                                     &pointer));

        mmu->pageTableLogical = pointer;

        /* Compute number of entries in page table. */
        mmu->pageTableEntries = mmu->pageTableSize / sizeof(gctUINT32);

        /* Mark all pages as free. */
        pageTable      = mmu->pageTableLogical;

#if gcdMMU_CLEAR_VALUE
        {
            gctUINT32 i;

            for (i = 0; i < mmu->pageTableEntries; ++i)
            {
                pageTable[i] = gcdMMU_CLEAR_VALUE;
            }
        }
#endif

        pageTable[0]   = (mmu->pageTableEntries << 8) | gcvMMU_FREE;
        pageTable[1]   = ~0U;
        mmu->heapList  = 0;
        mmu->freeNodes = gcvFALSE;

        /* Set page table address. */
        gcmkONERROR(
            gckHARDWARE_SetMMU(hardware, (gctPOINTER) mmu->pageTableLogical));
    }
    else
    {
        /* Allocate the 4K mode MTLB table. */
        mmu->mtlbSize = gcdMMU_MTLB_SIZE + 64;

        gcmkONERROR(
            gckOS_AllocateContiguous(os,
                                     gcvFALSE,
                                     &mmu->mtlbSize,
                                     &mmu->mtlbPhysical,
                                     &pointer));

        mmu->mtlbLogical = pointer;

        /* Invalid all the entries. */
        gcmkONERROR(
            gckOS_ZeroMemory(pointer, mmu->mtlbSize));
    }

    /* Return the gckMMU object pointer. */
    *Mmu = mmu;

    /* Success. */
    gcmkFOOTER_ARG("*Mmu=0x%x", *Mmu);
    return gcvSTATUS_OK;

OnError:
    /* Roll back. */
    if (mmu != gcvNULL)
    {
        if (mmu->pageTableLogical != gcvNULL)
        {
            /* Free the page table. */
            gcmkVERIFY_OK(
                gckOS_FreeContiguous(os,
                                     mmu->pageTablePhysical,
                                     (gctPOINTER) mmu->pageTableLogical,
                                     mmu->pageTableSize));

        }

        if (mmu->mtlbLogical != gcvNULL)
        {
            gcmkVERIFY_OK(
                gckOS_FreeContiguous(os,
                                     mmu->mtlbPhysical,
                                     (gctPOINTER) mmu->mtlbLogical,
                                     mmu->mtlbSize));
        }

        if (mmu->pageTableMutex != gcvNULL)
        {
            /* Delete the mutex. */
            gcmkVERIFY_OK(
                gckOS_DeleteMutex(os, mmu->pageTableMutex));
        }

#ifdef __QNXNTO__
        if (mmu->nodeMutex != gcvNULL)
        {
            /* Delete the mutex. */
            gcmkVERIFY_OK(
                gckOS_DeleteMutex(os, mmu->nodeMutex));
        }
#endif

        /* Mark the gckMMU object as unknown. */
        mmu->object.type = gcvOBJ_UNKNOWN;

        /* Free the allocates memory. */
        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(os, mmu));
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  _Destroy
**
**  Destroy a gckMMU object.
**
**  INPUT:
**
**      gckMMU Mmu
**          Pointer to an gckMMU object.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
_Destroy(
    IN gckMMU Mmu
    )
{
#ifdef __QNXNTO__
    gcuVIDMEM_NODE_PTR node, next;
#endif

    gcmkHEADER_ARG("Mmu=0x%x", Mmu);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Mmu, gcvOBJ_MMU);

#ifdef __QNXNTO__
    /* Free all associated virtual memory. */
    for (node = Mmu->nodeList; node != gcvNULL; node = next)
    {
        next = node->Virtual.next;
        gcmkVERIFY_OK(gckVIDMEM_Free(node));
    }
#endif

    while (Mmu->staticSTLB != gcvNULL)
    {
        gcsMMU_STLB_PTR pre = Mmu->staticSTLB;
        Mmu->staticSTLB = pre->next;

        if (pre->physical != gcvNULL)
        {
            gcmkVERIFY_OK(
                gckOS_FreeContiguous(Mmu->os,
                    pre->physical,
                    pre->logical,
                    pre->size));
        }

        if (pre->mtlbEntryNum != 0)
        {
            gcmkASSERT(pre->mtlbEntryNum == 1);
            *(Mmu->mtlbLogical + pre->mtlbIndex) = 0;
#if gcdMMU_TABLE_DUMP
            gckOS_Print("%s(%d): clean MTLB[%d]\n",
                __FUNCTION__, __LINE__,
                pre->mtlbIndex);
#endif
        }

        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Mmu->os, pre));
    }

    if (Mmu->hardware->mmuVersion != 0)
    {
        gcmkVERIFY_OK(
                gckOS_FreeContiguous(Mmu->os,
                    Mmu->mtlbPhysical,
                    (gctPOINTER) Mmu->mtlbLogical,
                    Mmu->mtlbSize));
    }

    /* Free the page table. */
    gcmkVERIFY_OK(
            gckOS_FreeContiguous(Mmu->os,
                Mmu->pageTablePhysical,
                (gctPOINTER) Mmu->pageTableLogical,
                Mmu->pageTableSize));

#ifdef __QNXNTO__
    /* Delete the node list mutex. */
    gcmkVERIFY_OK(gckOS_DeleteMutex(Mmu->os, Mmu->nodeMutex));
#endif

    /* Delete the page table mutex. */
    gcmkVERIFY_OK(gckOS_DeleteMutex(Mmu->os, Mmu->pageTableMutex));

    /* Mark the gckMMU object as unknown. */
    Mmu->object.type = gcvOBJ_UNKNOWN;

    /* Free the gckMMU object. */
    gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Mmu->os, Mmu));

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

gceSTATUS
gckMMU_Construct(
    IN gckKERNEL Kernel,
    IN gctSIZE_T MmuSize,
    OUT gckMMU * Mmu
    )
{
#if gcdSHARED_PAGETABLE
    gceSTATUS status;
    gctPOINTER pointer;

    gcmkHEADER_ARG("Kernel=0x%08x", Kernel);

    if (sharedPageTable == gcvNULL)
    {
        gcmkONERROR(
                gckOS_Allocate(Kernel->os,
                               sizeof(struct _gcsSharedPageTable),
                               &pointer));
        sharedPageTable = pointer;

        gcmkONERROR(
                gckOS_ZeroMemory(sharedPageTable,
                    sizeof(struct _gcsSharedPageTable)));

        gcmkONERROR(_Construct(Kernel, MmuSize, &sharedPageTable->mmu));
    }

    *Mmu = sharedPageTable->mmu;

    sharedPageTable->hardwares[sharedPageTable->reference] = Kernel->hardware;

    sharedPageTable->reference++;

    gcmkFOOTER_ARG("sharedPageTable->reference=%lu", sharedPageTable->reference);
    return gcvSTATUS_OK;

OnError:
    if (sharedPageTable)
    {
        if (sharedPageTable->mmu)
        {
            gcmkVERIFY_OK(gckMMU_Destroy(sharedPageTable->mmu));
        }

        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Kernel->os, sharedPageTable));
    }

    gcmkFOOTER();
    return status;
#else
    return _Construct(Kernel, MmuSize, Mmu);
#endif
}

gceSTATUS
gckMMU_Destroy(
    IN gckMMU Mmu
    )
{
#if gcdSHARED_PAGETABLE
    sharedPageTable->reference--;

    if (sharedPageTable->reference == 0)
    {
        if (sharedPageTable->mmu)
        {
            gcmkVERIFY_OK(_Destroy(Mmu));
        }

        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Mmu->os, sharedPageTable));
    }

    return gcvSTATUS_OK;
#else
    return _Destroy(Mmu);
#endif
}

/*******************************************************************************
**
**  gckMMU_AllocatePages
**
**  Allocate pages inside the page table.
**
**  INPUT:
**
**      gckMMU Mmu
**          Pointer to an gckMMU object.
**
**      gctSIZE_T PageCount
**          Number of pages to allocate.
**
**  OUTPUT:
**
**      gctPOINTER * PageTable
**          Pointer to a variable that receives the base address of the page
**          table.
**
**      gctUINT32 * Address
**          Pointer to a variable that receives the hardware specific address.
*/
gceSTATUS
gckMMU_AllocatePages(
    IN gckMMU Mmu,
    IN gctSIZE_T PageCount,
    OUT gctPOINTER * PageTable,
    OUT gctUINT32 * Address
    )
{
    gceSTATUS status;
    gctBOOL mutex = gcvFALSE;
    gctUINT32 index = 0, previous = ~0U, left;
    gctUINT32_PTR pageTable;
    gctBOOL gotIt;
    gctUINT32 address;

    gcmkHEADER_ARG("Mmu=0x%x PageCount=%lu", Mmu, PageCount);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Mmu, gcvOBJ_MMU);
    gcmkVERIFY_ARGUMENT(PageCount > 0);
    gcmkVERIFY_ARGUMENT(PageTable != gcvNULL);

    if (PageCount > Mmu->pageTableEntries)
    {
        /* Not enough pages avaiable. */
        gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
    }

    /* Grab the mutex. */
    gcmkONERROR(gckOS_AcquireMutex(Mmu->os, Mmu->pageTableMutex, gcvINFINITE));
    mutex = gcvTRUE;

    /* Cast pointer to page table. */
    for (pageTable = Mmu->pageTableLogical, gotIt = gcvFALSE; !gotIt;)
    {
        /* Walk the heap list. */
        for (index = Mmu->heapList; !gotIt && (index < Mmu->pageTableEntries);)
        {
            /* Check the node type. */
            switch (gcmENTRY_TYPE(pageTable[index]))
            {
            case gcvMMU_SINGLE:
                /* Single odes are valid if we only need 1 page. */
                if (PageCount == 1)
                {
                    gotIt = gcvTRUE;
                }
                else
                {
                    /* Move to next node. */
                    previous = index;
                    index    = pageTable[index] >> 8;
                }
                break;

            case gcvMMU_FREE:
                /* Test if the node has enough space. */
                if (PageCount <= (pageTable[index] >> 8))
                {
                    gotIt = gcvTRUE;
                }
                else
                {
                    /* Move to next node. */
                    previous = index;
                    index    = pageTable[index + 1];
                }
                break;

            default:
                gcmkFATAL("MMU table correcupted at index %u!", index);
                gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
            }
        }

        /* Test if we are out of memory. */
        if (index >= Mmu->pageTableEntries)
        {
            if (Mmu->freeNodes)
            {
                /* Time to move out the trash! */
                gcmkONERROR(_Collect(Mmu));
            }
            else
            {
                /* Out of resources. */
                gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
            }
        }
    }

    switch (gcmENTRY_TYPE(pageTable[index]))
    {
    case gcvMMU_SINGLE:
        /* Unlink single node from free list. */
        gcmkONERROR(
            _Link(Mmu, previous, pageTable[index] >> 8));
        break;

    case gcvMMU_FREE:
        /* Check how many pages will be left. */
        left = (pageTable[index] >> 8) - PageCount;
        switch (left)
        {
        case 0:
            /* The entire node is consumed, just unlink it. */
            gcmkONERROR(
                _Link(Mmu, previous, pageTable[index + 1]));
            break;

        case 1:
            /* One page will remain.  Convert the node to a single node and
            ** advance the index. */
            pageTable[index] = (pageTable[index + 1] << 8) | gcvMMU_SINGLE;
            index ++;
            break;

        default:
            /* Enough pages remain for a new node.  However, we will just adjust
            ** the size of the current node and advance the index. */
            pageTable[index] = (left << 8) | gcvMMU_FREE;
            index += left;
            break;
        }
        break;
    }

    /* Mark node as used. */
    pageTable[index] = gcvMMU_USED;

    /* Return pointer to page table. */
    *PageTable = &pageTable[index];

    /* Build virtual address. */
    if (Mmu->hardware->mmuVersion == 0)
    {
        gcmkONERROR(
                gckHARDWARE_BuildVirtualAddress(Mmu->hardware, index, 0, &address));
    }
    else
    {
        gctUINT32 masterOffset = index / gcdMMU_STLB_4K_ENTRY_NUM
                               + Mmu->dynamicMappingStart;
        gctUINT32 slaveOffset = index % gcdMMU_STLB_4K_ENTRY_NUM;

        address = (masterOffset << gcdMMU_MTLB_SHIFT)
                | (slaveOffset << gcdMMU_STLB_4K_SHIFT);
    }

    if (Address != gcvNULL)
    {
        *Address = address;
    }

    /* Release the mutex. */
    gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->pageTableMutex));

    /* Success. */
    gcmkFOOTER_ARG("*PageTable=0x%x *Address=%08x",
                   *PageTable, gcmOPT_VALUE(Address));
    return gcvSTATUS_OK;

OnError:

    if (mutex)
    {
        /* Release the mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->pageTableMutex));
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckMMU_FreePages
**
**  Free pages inside the page table.
**
**  INPUT:
**
**      gckMMU Mmu
**          Pointer to an gckMMU object.
**
**      gctPOINTER PageTable
**          Base address of the page table to free.
**
**      gctSIZE_T PageCount
**          Number of pages to free.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckMMU_FreePages(
    IN gckMMU Mmu,
    IN gctPOINTER PageTable,
    IN gctSIZE_T PageCount
    )
{
    gctUINT32_PTR pageTable;

    gcmkHEADER_ARG("Mmu=0x%x PageTable=0x%x PageCount=%lu",
                   Mmu, PageTable, PageCount);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Mmu, gcvOBJ_MMU);
    gcmkVERIFY_ARGUMENT(PageTable != gcvNULL);
    gcmkVERIFY_ARGUMENT(PageCount > 0);

    /* Convert the pointer. */
    pageTable = (gctUINT32_PTR) PageTable;

#if gcdMMU_CLEAR_VALUE
    {
        gctUINT32 i;

        for (i = 0; i < PageCount; ++i)
        {
            pageTable[i] = gcdMMU_CLEAR_VALUE;
        }
    }
#endif

    if (PageCount == 1)
    {
        /* Single page node. */
        pageTable[0] = (~((1U<<8)-1)) | gcvMMU_SINGLE;
    }
    else
    {
        /* Mark the node as free. */
        pageTable[0] = (PageCount << 8) | gcvMMU_FREE;
        pageTable[1] = ~0U;
    }

    /* We have free nodes. */
    Mmu->freeNodes = gcvTRUE;

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

gceSTATUS
gckMMU_Enable(
    IN gckMMU Mmu,
    IN gctUINT32 PhysBaseAddr,
    IN gctUINT32 PhysSize
    )
{
    gceSTATUS status;

    gcmkHEADER_ARG("Mmu=0x%x", Mmu);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Mmu, gcvOBJ_MMU);

#if gcdSHARED_PAGETABLE
    if (Mmu->enabled)
    {
        gcmkFOOTER_ARG("Status=%d", gcvSTATUS_SKIP);
        return gcvSTATUS_SKIP;
    }
#endif

    if (Mmu->hardware->mmuVersion == 0)
    {
        /* Success. */
        gcmkFOOTER_ARG("Status=%d", gcvSTATUS_SKIP);
        return gcvSTATUS_SKIP;
    }
    else
    {
        if (PhysSize != 0)
        {
            gcmkONERROR(_FillFlatMapping(
                Mmu,
                PhysBaseAddr,
                PhysSize
                ));
        }

        gcmkONERROR(_SetupDynamicSpace(Mmu));

        gcmkONERROR(
            gckHARDWARE_SetMMUv2(
                Mmu->hardware,
                gcvTRUE,
                Mmu->mtlbLogical,
                gcvMMU_MODE_4K,
                (gctUINT8_PTR)Mmu->mtlbLogical + gcdMMU_MTLB_SIZE,
                gcvFALSE
                ));

        Mmu->enabled = gcvTRUE;

        /* Success. */
        gcmkFOOTER_NO();
        return gcvSTATUS_OK;
    }

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckMMU_SetPage(
    IN gckMMU Mmu,
    IN gctUINT32 PageAddress,
    IN gctUINT32 *PageEntry
    )
{
    gcmkHEADER_ARG("Mmu=0x%x", Mmu);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Mmu, gcvOBJ_MMU);
    gcmkVERIFY_ARGUMENT(PageEntry != gcvNULL);
    gcmkVERIFY_ARGUMENT(!(PageAddress & 0xFFF));

    if (Mmu->hardware->mmuVersion == 0)
    {
        *PageEntry = PageAddress;
    }
    else
    {
        *PageEntry = _SetPage(PageAddress);
    }

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

#ifdef __QNXNTO__
gceSTATUS
gckMMU_InsertNode(
    IN gckMMU Mmu,
    IN gcuVIDMEM_NODE_PTR Node)
{
    gceSTATUS status;
    gctBOOL mutex = gcvFALSE;

    gcmkHEADER_ARG("Mmu=0x%x Node=0x%x", Mmu, Node);

    gcmkVERIFY_OBJECT(Mmu, gcvOBJ_MMU);

    gcmkONERROR(gckOS_AcquireMutex(Mmu->os, Mmu->nodeMutex, gcvINFINITE));
    mutex = gcvTRUE;

    Node->Virtual.next = Mmu->nodeList;
    Mmu->nodeList = Node;

    gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->nodeMutex));

    gcmkFOOTER();
    return gcvSTATUS_OK;

OnError:
    if (mutex)
    {
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->nodeMutex));
    }

    gcmkFOOTER();
    return status;
}

gceSTATUS
gckMMU_RemoveNode(
    IN gckMMU Mmu,
    IN gcuVIDMEM_NODE_PTR Node)
{
    gceSTATUS status;
    gctBOOL mutex = gcvFALSE;
    gcuVIDMEM_NODE_PTR *iter;

    gcmkHEADER_ARG("Mmu=0x%x Node=0x%x", Mmu, Node);

    gcmkVERIFY_OBJECT(Mmu, gcvOBJ_MMU);

    gcmkONERROR(gckOS_AcquireMutex(Mmu->os, Mmu->nodeMutex, gcvINFINITE));
    mutex = gcvTRUE;

    for (iter = &Mmu->nodeList; *iter; iter = &(*iter)->Virtual.next)
    {
        if (*iter == Node)
        {
            *iter = Node->Virtual.next;
            break;
        }
    }

    gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->nodeMutex));

    gcmkFOOTER();
    return gcvSTATUS_OK;

OnError:
    if (mutex)
    {
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->nodeMutex));
    }

    gcmkFOOTER();
    return status;
}

gceSTATUS
gckMMU_FreeHandleMemory(
    IN gckKERNEL Kernel,
    IN gckMMU Mmu,
    IN gctUINT32 Pid
    )
{
    gceSTATUS status;
    gctBOOL acquired = gcvFALSE;
    gcuVIDMEM_NODE_PTR curr, next;

    gcmkHEADER_ARG("Kernel=0x%x, Mmu=0x%x Pid=%u", Kernel, Mmu, Pid);

    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_OBJECT(Mmu, gcvOBJ_MMU);

    gcmkONERROR(gckOS_AcquireMutex(Mmu->os, Mmu->nodeMutex, gcvINFINITE));
    acquired = gcvTRUE;

    for (curr = Mmu->nodeList; curr != gcvNULL; curr = next)
    {
        next = curr->Virtual.next;

        if (curr->Virtual.processID == Pid)
        {
            while (curr->Virtual.unlockPendings[Kernel->core] == 0 && curr->Virtual.lockeds[Kernel->core] > 0)
            {
                gcmkONERROR(gckVIDMEM_Unlock(Kernel, curr, gcvSURF_TYPE_UNKNOWN, gcvNULL));
            }

            gcmkVERIFY_OK(gckVIDMEM_Free(curr));
        }
    }

    gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->nodeMutex));

    gcmkFOOTER();
    return gcvSTATUS_OK;

OnError:
    if (acquired)
    {
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->nodeMutex));
    }

    gcmkFOOTER();
    return status;
}
#endif

gceSTATUS
gckMMU_Flush(
    IN gckMMU Mmu
    )
{
    gckHARDWARE hardware;
#if gcdSHARED_PAGETABLE
    gctINT i;
    for (i = 0; i < gcdCORE_COUNT; i++)
    {
#if gcdENABLE_VG
        if (i == gcvCORE_VG)
        {
            continue;
        }
#endif
        hardware = sharedPageTable->hardwares[i];
        if (hardware)
        {
            /* Notify cores who use this page table. */
            gcmkVERIFY_OK(
                gckOS_AtomSet(hardware->os, hardware->pageTableDirty, 1));
        }
    }
#else
    hardware = Mmu->hardware;
    gcmkVERIFY_OK(
        gckOS_AtomSet(hardware->os, hardware->pageTableDirty, 1));
#endif

    return gcvSTATUS_OK;
}

/******************************************************************************
****************************** T E S T   C O D E ******************************
******************************************************************************/

