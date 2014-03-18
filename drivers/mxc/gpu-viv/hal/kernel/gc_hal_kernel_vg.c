/****************************************************************************
*
*    Copyright (C) 2005 - 2013 by Vivante Corp.
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

#if gcdENABLE_VG

#define ENABLE_VG_TRY_VIRTUAL_MEMORY 0

#define _GC_OBJ_ZONE            gcvZONE_VG

/******************************************************************************\
******************************* gckKERNEL API Code ******************************
\******************************************************************************/

/*******************************************************************************
**
**  gckKERNEL_Construct
**
**  Construct a new gckKERNEL object.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      IN gctPOINTER Context
**          Pointer to a driver defined context.
**
**  OUTPUT:
**
**      gckKERNEL * Kernel
**          Pointer to a variable that will hold the pointer to the gckKERNEL
**          object.
*/
gceSTATUS gckVGKERNEL_Construct(
    IN gckOS Os,
    IN gctPOINTER Context,
    IN gckKERNEL  inKernel,
    OUT gckVGKERNEL * Kernel
    )
{
    gceSTATUS status;
    gckVGKERNEL kernel = gcvNULL;

    gcmkHEADER_ARG("Os=0x%x Context=0x%x", Os, Context);
    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Kernel != gcvNULL);

    do
    {
        /* Allocate the gckKERNEL object. */
        gcmkERR_BREAK(gckOS_Allocate(
            Os,
            sizeof(struct _gckVGKERNEL),
            (gctPOINTER *) &kernel
            ));

        /* Initialize the gckKERNEL object. */
        kernel->object.type = gcvOBJ_KERNEL;
        kernel->os          = Os;
        kernel->context     = Context;
        kernel->hardware    = gcvNULL;
        kernel->interrupt   = gcvNULL;
        kernel->command     = gcvNULL;
        kernel->mmu         = gcvNULL;
        kernel->kernel      = inKernel;

        /* Construct the gckVGHARDWARE object. */
        gcmkERR_BREAK(gckVGHARDWARE_Construct(
            Os, &kernel->hardware
            ));

        /* Set pointer to gckKERNEL object in gckVGHARDWARE object. */
        kernel->hardware->kernel = kernel;

        /* Construct the gckVGINTERRUPT object. */
        gcmkERR_BREAK(gckVGINTERRUPT_Construct(
            kernel, &kernel->interrupt
            ));

        /* Construct the gckVGCOMMAND object. */
        gcmkERR_BREAK(gckVGCOMMAND_Construct(
            kernel, gcmKB2BYTES(8), gcmKB2BYTES(2), &kernel->command
            ));

        /* Construct the gckVGMMU object. */
        gcmkERR_BREAK(gckVGMMU_Construct(
            kernel, gcmKB2BYTES(32), &kernel->mmu
            ));

        /* Return pointer to the gckKERNEL object. */
        *Kernel = kernel;

        gcmkFOOTER_ARG("*Kernel=0x%x", *Kernel);
        /* Success. */
        return gcvSTATUS_OK;
    }
    while (gcvFALSE);

    /* Roll back. */
    if (kernel != gcvNULL)
    {
        if (kernel->mmu != gcvNULL)
        {
            gcmkVERIFY_OK(gckVGMMU_Destroy(kernel->mmu));
        }

        if (kernel->command != gcvNULL)
        {
            gcmkVERIFY_OK(gckVGCOMMAND_Destroy(kernel->command));
        }

        if (kernel->interrupt != gcvNULL)
        {
            gcmkVERIFY_OK(gckVGINTERRUPT_Destroy(kernel->interrupt));
        }

        if (kernel->hardware != gcvNULL)
        {
            gcmkVERIFY_OK(gckVGHARDWARE_Destroy(kernel->hardware));
        }

        gcmkVERIFY_OK(gckOS_Free(Os, kernel));
    }

    gcmkFOOTER();
    /* Return status. */
    return status;
}

/*******************************************************************************
**
**  gckKERNEL_Destroy
**
**  Destroy an gckKERNEL object.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object to destroy.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS gckVGKERNEL_Destroy(
    IN gckVGKERNEL Kernel
    )
{
    gceSTATUS status;

    gcmkHEADER_ARG("Kernel=0x%x", Kernel);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);

    do
    {
        /* Destroy the gckVGMMU object. */
        if (Kernel->mmu != gcvNULL)
        {
            gcmkERR_BREAK(gckVGMMU_Destroy(Kernel->mmu));
            Kernel->mmu = gcvNULL;
        }

        /* Destroy the gckVGCOMMAND object. */
        if (Kernel->command != gcvNULL)
        {
            gcmkERR_BREAK(gckVGCOMMAND_Destroy(Kernel->command));
            Kernel->command = gcvNULL;
        }

        /* Destroy the gckVGINTERRUPT object. */
        if (Kernel->interrupt != gcvNULL)
        {
            gcmkERR_BREAK(gckVGINTERRUPT_Destroy(Kernel->interrupt));
            Kernel->interrupt = gcvNULL;
        }

        /* Destroy the gckVGHARDWARE object. */
        if (Kernel->hardware != gcvNULL)
        {
            gcmkERR_BREAK(gckVGHARDWARE_Destroy(Kernel->hardware));
            Kernel->hardware = gcvNULL;
        }

        /* Mark the gckKERNEL object as unknown. */
        Kernel->object.type = gcvOBJ_UNKNOWN;

        /* Free the gckKERNEL object. */
        gcmkERR_BREAK(gckOS_Free(Kernel->os, Kernel));
    }
    while (gcvFALSE);

    gcmkFOOTER();

    /* Return status. */
    return status;
}

/*******************************************************************************
**
**  gckKERNEL_AllocateLinearMemory
**
**  Function walks all required memory pools and allocates the requested
**  amount of video memory.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gcePOOL * Pool
**          Pointer the desired memory pool.
**
**      gctSIZE_T Bytes
**          Number of bytes to allocate.
**
**      gctSIZE_T Alignment
**          Required buffer alignment.
**
**      gceSURF_TYPE Type
**          Surface type.
**
**  OUTPUT:
**
**      gcePOOL * Pool
**          Pointer to the actual pool where the memory was allocated.
**
**      gcuVIDMEM_NODE_PTR * Node
**          Allocated node.
*/
gceSTATUS
gckKERNEL_AllocateLinearMemory(
    IN gckKERNEL Kernel,
    IN OUT gcePOOL * Pool,
    IN gctSIZE_T Bytes,
    IN gctSIZE_T Alignment,
    IN gceSURF_TYPE Type,
    OUT gcuVIDMEM_NODE_PTR * Node
    )
{
    gcePOOL pool;
    gceSTATUS status;
    gckVIDMEM videoMemory;

    /* Get initial pool. */
    switch (pool = *Pool)
    {
    case gcvPOOL_DEFAULT:
    case gcvPOOL_LOCAL:
        pool = gcvPOOL_LOCAL_INTERNAL;
        break;

    case gcvPOOL_UNIFIED:
        pool = gcvPOOL_SYSTEM;
        break;

    default:
        break;
    }

    do
    {
        /* Verify the number of bytes to allocate. */
        if (Bytes == 0)
        {
            status = gcvSTATUS_INVALID_ARGUMENT;
            break;
        }

        if (pool == gcvPOOL_VIRTUAL)
        {
            /* Create a gcuVIDMEM_NODE for virtual memory. */
            gcmkERR_BREAK(gckVIDMEM_ConstructVirtual(Kernel, gcvFALSE, Bytes, Node));

            /* Success. */
            break;
        }

        else
        {
            /* Get pointer to gckVIDMEM object for pool. */
            status = gckKERNEL_GetVideoMemoryPool(Kernel, pool, &videoMemory);

            if (status == gcvSTATUS_OK)
            {
                if(*Pool == gcvPOOL_SYSTEM)
                    Type |= gcvSURF_VG;
                /* Allocate memory. */
                status = gckVIDMEM_AllocateLinear(Kernel,
                                                  videoMemory,
                                                  Bytes,
                                                  Alignment,
                                                  Type,
                                                  Node);

                if (status == gcvSTATUS_OK)
                {
                    /* Memory allocated. */
                    break;
                }
            }
        }

        if (pool == gcvPOOL_LOCAL_INTERNAL)
        {
            /* Advance to external memory. */
            pool = gcvPOOL_LOCAL_EXTERNAL;
        }
        else if (pool == gcvPOOL_LOCAL_EXTERNAL)
        {
            /* Advance to contiguous system memory. */
            pool = gcvPOOL_SYSTEM;
        }
        else if (pool == gcvPOOL_SYSTEM)
        {
            /* Advance to virtual memory. */
#if ENABLE_VG_TRY_VIRTUAL_MEMORY
            pool = gcvPOOL_VIRTUAL;
#else
            /*VG non-contiguous memory support is not ready yet, disable it temporary*/
            status = gcvSTATUS_OUT_OF_MEMORY;
            break;
#endif
        }
        else
        {
            /* Out of pools. */
            status = gcvSTATUS_OUT_OF_MEMORY;
            break;
        }
    }
    /* Loop only for multiple selection pools. */
    while ((*Pool == gcvPOOL_DEFAULT)
    ||     (*Pool == gcvPOOL_LOCAL)
    ||     (*Pool == gcvPOOL_UNIFIED)
    );

    if (gcmIS_SUCCESS(status))
    {
        /* Return pool used for allocation. */
        *Pool = pool;
    }

    /* Return status. */
    return status;
}

/*******************************************************************************
**
**  gckKERNEL_Dispatch
**
**  Dispatch a command received from the user HAL layer.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gcsHAL_INTERFACE * Interface
**          Pointer to a gcsHAL_INTERFACE structure that defines the command to
**          be dispatched.
**
**  OUTPUT:
**
**      gcsHAL_INTERFACE * Interface
**          Pointer to a gcsHAL_INTERFACE structure that receives any data to be
**          returned.
*/
gceSTATUS gckVGKERNEL_Dispatch(
    IN gckKERNEL Kernel,
    IN gctBOOL FromUser,
    IN OUT gcsHAL_INTERFACE * Interface
    )
{
    gceSTATUS status;
    gcsHAL_INTERFACE * kernelInterface = Interface;
    gcuVIDMEM_NODE_PTR node;
    gctUINT32 processID;
    gckKERNEL kernel = Kernel;
    gctPOINTER info = gcvNULL;
    gctPHYS_ADDR physical = gcvNULL;
    gctPOINTER logical = gcvNULL;
    gctSIZE_T bytes = 0;

    gcmkHEADER_ARG("Kernel=0x%x Interface=0x%x ", Kernel, Interface);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(Interface != gcvNULL);

    gcmkONERROR(gckOS_GetProcessID(&processID));

    /* Dispatch on command. */
    switch (Interface->command)
    {
    case gcvHAL_QUERY_VIDEO_MEMORY:
        /* Query video memory size. */
        gcmkERR_BREAK(gckKERNEL_QueryVideoMemory(
            Kernel, kernelInterface
            ));
        break;

    case gcvHAL_QUERY_CHIP_IDENTITY:
        /* Query chip identity. */
        gcmkERR_BREAK(gckVGHARDWARE_QueryChipIdentity(
            Kernel->vg->hardware,
            &kernelInterface->u.QueryChipIdentity.chipModel,
            &kernelInterface->u.QueryChipIdentity.chipRevision,
            &kernelInterface->u.QueryChipIdentity.chipFeatures,
            &kernelInterface->u.QueryChipIdentity.chipMinorFeatures,
            &kernelInterface->u.QueryChipIdentity.chipMinorFeatures2
            ));
        break;

    case gcvHAL_QUERY_COMMAND_BUFFER:
        /* Query command buffer information. */
        gcmkERR_BREAK(gckKERNEL_QueryCommandBuffer(
            Kernel,
            &kernelInterface->u.QueryCommandBuffer.information
            ));
        break;
    case gcvHAL_ALLOCATE_NON_PAGED_MEMORY:
        bytes = (gctSIZE_T) kernelInterface->u.AllocateNonPagedMemory.bytes;
        /* Allocate non-paged memory. */
        gcmkERR_BREAK(gckOS_AllocateContiguous(
            Kernel->os,
            gcvTRUE,
            &bytes,
            &physical,
            &logical
            ));

        kernelInterface->u.AllocateNonPagedMemory.bytes    = bytes;
        kernelInterface->u.AllocateNonPagedMemory.logical  = gcmPTR_TO_UINT64(logical);
        kernelInterface->u.AllocateNonPagedMemory.physical = gcmPTR_TO_NAME(physical);
        break;

    case gcvHAL_FREE_NON_PAGED_MEMORY:
        physical = gcmNAME_TO_PTR(kernelInterface->u.AllocateNonPagedMemory.physical);

        /* Unmap user logical out of physical memory first. */
        gcmkERR_BREAK(gckOS_UnmapUserLogical(
            Kernel->os,
            physical,
            (gctSIZE_T) kernelInterface->u.AllocateNonPagedMemory.bytes,
            gcmUINT64_TO_PTR(kernelInterface->u.AllocateNonPagedMemory.logical)
            ));

        /* Free non-paged memory. */
        gcmkERR_BREAK(gckOS_FreeNonPagedMemory(
            Kernel->os,
            (gctSIZE_T) kernelInterface->u.AllocateNonPagedMemory.bytes,
            physical,
            gcmUINT64_TO_PTR(kernelInterface->u.AllocateNonPagedMemory.logical)
            ));

        gcmRELEASE_NAME(kernelInterface->u.AllocateNonPagedMemory.physical);
        break;

    case gcvHAL_ALLOCATE_CONTIGUOUS_MEMORY:
        bytes = (gctSIZE_T) kernelInterface->u.AllocateNonPagedMemory.bytes;
        /* Allocate contiguous memory. */
        gcmkERR_BREAK(gckOS_AllocateContiguous(
            Kernel->os,
            gcvTRUE,
            &bytes,
            &physical,
            &logical
            ));

        kernelInterface->u.AllocateNonPagedMemory.bytes    = bytes;
        kernelInterface->u.AllocateNonPagedMemory.logical  = gcmPTR_TO_UINT64(logical);
        kernelInterface->u.AllocateNonPagedMemory.physical = gcmPTR_TO_NAME(physical);
        break;

    case gcvHAL_FREE_CONTIGUOUS_MEMORY:
        physical = gcmNAME_TO_PTR(kernelInterface->u.AllocateNonPagedMemory.physical);
        /* Unmap user logical out of physical memory first. */
        gcmkERR_BREAK(gckOS_UnmapUserLogical(
            Kernel->os,
            physical,
            (gctSIZE_T) kernelInterface->u.AllocateNonPagedMemory.bytes,
            gcmUINT64_TO_PTR(kernelInterface->u.AllocateNonPagedMemory.logical)
            ));

        /* Free contiguous memory. */
        gcmkERR_BREAK(gckOS_FreeContiguous(
            Kernel->os,
            physical,
            gcmUINT64_TO_PTR(kernelInterface->u.AllocateNonPagedMemory.logical),
            (gctSIZE_T) kernelInterface->u.AllocateNonPagedMemory.bytes
            ));

        gcmRELEASE_NAME(kernelInterface->u.AllocateNonPagedMemory.physical);
        break;

    case gcvHAL_ALLOCATE_VIDEO_MEMORY:
        {
            gctSIZE_T bytes;
            gctUINT32 bitsPerPixel;
            gctUINT32 bits;

            /* Align width and height to tiles. */
            gcmkERR_BREAK(gckVGHARDWARE_AlignToTile(
                Kernel->vg->hardware,
                kernelInterface->u.AllocateVideoMemory.type,
                &kernelInterface->u.AllocateVideoMemory.width,
                &kernelInterface->u.AllocateVideoMemory.height
                ));

            /* Convert format into bytes per pixel and bytes per tile. */
            gcmkERR_BREAK(gckVGHARDWARE_ConvertFormat(
                Kernel->vg->hardware,
                kernelInterface->u.AllocateVideoMemory.format,
                &bitsPerPixel,
                gcvNULL
                ));

            /* Compute number of bits for the allocation. */
            bits
                = kernelInterface->u.AllocateVideoMemory.width
                * kernelInterface->u.AllocateVideoMemory.height
                * kernelInterface->u.AllocateVideoMemory.depth
                * bitsPerPixel;

            /* Compute number of bytes for the allocation. */
            bytes = gcmALIGN(bits, 8) / 8;

            /* Allocate memory. */
            gcmkERR_BREAK(gckKERNEL_AllocateLinearMemory(
                Kernel,
                &kernelInterface->u.AllocateVideoMemory.pool,
                bytes,
                64,
                kernelInterface->u.AllocateVideoMemory.type,
                &node
                ));

            kernelInterface->u.AllocateVideoMemory.node = gcmPTR_TO_UINT64(node);
        }
        break;

    case gcvHAL_ALLOCATE_LINEAR_VIDEO_MEMORY:
        /* Allocate memory. */
        gcmkERR_BREAK(gckKERNEL_AllocateLinearMemory(
            Kernel,
            &kernelInterface->u.AllocateLinearVideoMemory.pool,
            kernelInterface->u.AllocateLinearVideoMemory.bytes,
            kernelInterface->u.AllocateLinearVideoMemory.alignment,
            kernelInterface->u.AllocateLinearVideoMemory.type,
            &node
            ));

        gcmkERR_BREAK(gckKERNEL_AddProcessDB(Kernel,
           processID, gcvDB_VIDEO_MEMORY,
           node,
           gcvNULL,
           kernelInterface->u.AllocateLinearVideoMemory.bytes
           ));

        kernelInterface->u.AllocateLinearVideoMemory.node = gcmPTR_TO_UINT64(node);
        break;

    case gcvHAL_FREE_VIDEO_MEMORY:
        node = gcmUINT64_TO_PTR(Interface->u.FreeVideoMemory.node);
#ifdef __QNXNTO__
        /* Unmap the video memory */

        if ((node->VidMem.memory->object.type == gcvOBJ_VIDMEM) &&
            (node->VidMem.logical != gcvNULL))
        {
            gckKERNEL_UnmapVideoMemory(Kernel,
                                       node->VidMem.logical,
                                       processID,
                                       node->VidMem.bytes);
            node->VidMem.logical = gcvNULL;
        }
#endif /* __QNXNTO__ */

        /* Free video memory. */
        gcmkERR_BREAK(gckVIDMEM_Free(Kernel,
            node
            ));

        gcmkERR_BREAK(gckKERNEL_RemoveProcessDB(
            Kernel,
            processID, gcvDB_VIDEO_MEMORY,
            node
            ));

        break;

    case gcvHAL_MAP_MEMORY:
        /* Map memory. */
        gcmkERR_BREAK(gckKERNEL_MapMemory(
            Kernel,
            gcmINT2PTR(kernelInterface->u.MapMemory.physical),
            (gctSIZE_T) kernelInterface->u.MapMemory.bytes,
            &logical
            ));
        kernelInterface->u.MapMemory.logical = gcmPTR_TO_UINT64(logical);
        break;

    case gcvHAL_UNMAP_MEMORY:
        /* Unmap memory. */
        gcmkERR_BREAK(gckKERNEL_UnmapMemory(
            Kernel,
            gcmINT2PTR(kernelInterface->u.MapMemory.physical),
            (gctSIZE_T) kernelInterface->u.MapMemory.bytes,
            gcmUINT64_TO_PTR(kernelInterface->u.MapMemory.logical)
            ));
        break;

    case gcvHAL_MAP_USER_MEMORY:
        /* Map user memory to DMA. */
        gcmkERR_BREAK(gckOS_MapUserMemory(
            Kernel->os,
            gcvCORE_VG,
            gcmUINT64_TO_PTR(kernelInterface->u.MapUserMemory.memory),
            kernelInterface->u.MapUserMemory.physical,
            (gctSIZE_T) kernelInterface->u.MapUserMemory.size,
            &info,
            &kernelInterface->u.MapUserMemory.address
            ));

        kernelInterface->u.MapUserMemory.info = gcmPTR_TO_NAME(info);
        break;

    case gcvHAL_UNMAP_USER_MEMORY:
        /* Unmap user memory. */
        gcmkERR_BREAK(gckOS_UnmapUserMemory(
            Kernel->os,
            gcvCORE_VG,
            gcmUINT64_TO_PTR(kernelInterface->u.UnmapUserMemory.memory),
            (gctSIZE_T) kernelInterface->u.UnmapUserMemory.size,
            gcmNAME_TO_PTR(kernelInterface->u.UnmapUserMemory.info),
            kernelInterface->u.UnmapUserMemory.address
            ));
        gcmRELEASE_NAME(kernelInterface->u.UnmapUserMemory.info);
        break;
    case gcvHAL_LOCK_VIDEO_MEMORY:
        node = gcmUINT64_TO_PTR(Interface->u.LockVideoMemory.node);

        /* Lock video memory. */
        gcmkERR_BREAK(
            gckVIDMEM_Lock(Kernel,
                           node,
						   gcvFALSE,
                           &Interface->u.LockVideoMemory.address));

        if (node->VidMem.memory->object.type == gcvOBJ_VIDMEM)
        {
            /* Map video memory address into user space. */
#ifdef __QNXNTO__
        if (node->VidMem.logical == gcvNULL)
        {
            gcmkONERROR(
                gckKERNEL_MapVideoMemory(Kernel,
                                         FromUser,
                                         Interface->u.LockVideoMemory.address,
                                         processID,
                                         node->VidMem.bytes,
                                         &node->VidMem.logical));
        }

        Interface->u.LockVideoMemory.memory = gcmPTR_TO_UINT64(node->VidMem.logical);
#else
            gcmkERR_BREAK(
                gckKERNEL_MapVideoMemoryEx(Kernel,
                                         gcvCORE_VG,
                                         FromUser,
                                         Interface->u.LockVideoMemory.address,
                                         &logical));
            Interface->u.LockVideoMemory.memory = gcmPTR_TO_UINT64(logical);
#endif
        }
        else
        {
            Interface->u.LockVideoMemory.memory = gcmPTR_TO_UINT64(node->Virtual.logical);

            /* Success. */
            status = gcvSTATUS_OK;
        }

#if gcdSECURE_USER
        /* Return logical address as physical address. */
        Interface->u.LockVideoMemory.address =
            (gctUINT32)(Interface->u.LockVideoMemory.memory);
#endif
        gcmkERR_BREAK(
            gckKERNEL_AddProcessDB(Kernel,
                                   processID, gcvDB_VIDEO_MEMORY_LOCKED,
                                   node,
                                   gcvNULL,
                                   0));
        break;

    case gcvHAL_UNLOCK_VIDEO_MEMORY:
        /* Unlock video memory. */
        node = gcmUINT64_TO_PTR(Interface->u.UnlockVideoMemory.node);

#if gcdSECURE_USER
        /* Save node information before it disappears. */
        if (node->VidMem.memory->object.type == gcvOBJ_VIDMEM)
        {
            logical = gcvNULL;
            bytes   = 0;
        }
        else
        {
            logical = node->Virtual.logical;
            bytes   = node->Virtual.bytes;
        }
#endif

        /* Unlock video memory. */
        gcmkERR_BREAK(
            gckVIDMEM_Unlock(Kernel,
                             node,
                             Interface->u.UnlockVideoMemory.type,
                             &Interface->u.UnlockVideoMemory.asynchroneous));

#if gcdSECURE_USER
        /* Flush the translation cache for virtual surfaces. */
        if (logical != gcvNULL)
        {
            gcmkVERIFY_OK(gckKERNEL_FlushTranslationCache(Kernel,
                                                          cache,
                                                          logical,
                                                          bytes));
        }
#endif

        if (Interface->u.UnlockVideoMemory.asynchroneous == gcvFALSE)
        {
            /* There isn't a event to unlock this node, remove record now */
            gcmkERR_BREAK(
                    gckKERNEL_RemoveProcessDB(Kernel,
                        processID, gcvDB_VIDEO_MEMORY_LOCKED,
                        node));
        }

        break;
    case gcvHAL_USER_SIGNAL:
#if !USE_NEW_LINUX_SIGNAL
        /* Dispatch depends on the user signal subcommands. */
        switch(Interface->u.UserSignal.command)
        {
        case gcvUSER_SIGNAL_CREATE:
            /* Create a signal used in the user space. */
            gcmkERR_BREAK(
                gckOS_CreateUserSignal(Kernel->os,
                                       Interface->u.UserSignal.manualReset,
                                       &Interface->u.UserSignal.id));

            gcmkVERIFY_OK(
                gckKERNEL_AddProcessDB(Kernel,
                                       processID, gcvDB_SIGNAL,
                                       gcmINT2PTR(Interface->u.UserSignal.id),
                                       gcvNULL,
                                       0));
            break;

        case gcvUSER_SIGNAL_DESTROY:
            /* Destroy the signal. */
            gcmkERR_BREAK(
                gckOS_DestroyUserSignal(Kernel->os,
                                        Interface->u.UserSignal.id));

            gcmkVERIFY_OK(gckKERNEL_RemoveProcessDB(
                Kernel,
                processID, gcvDB_SIGNAL,
                gcmINT2PTR(Interface->u.UserSignal.id)));
            break;

        case gcvUSER_SIGNAL_SIGNAL:
            /* Signal the signal. */
            gcmkERR_BREAK(
                gckOS_SignalUserSignal(Kernel->os,
                                       Interface->u.UserSignal.id,
                                       Interface->u.UserSignal.state));
            break;

        case gcvUSER_SIGNAL_WAIT:
            /* Wait on the signal. */
            status = gckOS_WaitUserSignal(Kernel->os,
                                          Interface->u.UserSignal.id,
                                          Interface->u.UserSignal.wait);
            break;

        default:
            /* Invalid user signal command. */
            gcmkERR_BREAK(gcvSTATUS_INVALID_ARGUMENT);
        }
#endif
        break;

    case gcvHAL_COMMIT:
        /* Commit a command and context buffer. */
        gcmkERR_BREAK(gckVGCOMMAND_Commit(
            Kernel->vg->command,
            gcmUINT64_TO_PTR(kernelInterface->u.VGCommit.context),
            gcmUINT64_TO_PTR(kernelInterface->u.VGCommit.queue),
            kernelInterface->u.VGCommit.entryCount,
            gcmUINT64_TO_PTR(kernelInterface->u.VGCommit.taskTable)
            ));
        break;
    case gcvHAL_VERSION:
        kernelInterface->u.Version.major = gcvVERSION_MAJOR;
        kernelInterface->u.Version.minor = gcvVERSION_MINOR;
        kernelInterface->u.Version.patch = gcvVERSION_PATCH;
        kernelInterface->u.Version.build = gcvVERSION_BUILD;
        status = gcvSTATUS_OK;
        break;

    case gcvHAL_GET_BASE_ADDRESS:
        /* Get base address. */
        gcmkERR_BREAK(
            gckOS_GetBaseAddress(Kernel->os,
                                 &kernelInterface->u.GetBaseAddress.baseAddress));
        break;
    default:
        /* Invalid command. */
        status = gcvSTATUS_INVALID_ARGUMENT;
    }

OnError:
    /* Save status. */
    kernelInterface->status = status;

    gcmkFOOTER();

    /* Return the status. */
    return status;
}

/*******************************************************************************
**
**  gckKERNEL_QueryCommandBuffer
**
**  Query command buffer attributes.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckVGHARDWARE object.
**
**  OUTPUT:
**
**      gcsCOMMAND_BUFFER_INFO_PTR Information
**          Pointer to the information structure to receive buffer attributes.
*/
gceSTATUS
gckKERNEL_QueryCommandBuffer(
    IN gckKERNEL Kernel,
    OUT gcsCOMMAND_BUFFER_INFO_PTR Information
    )
{
    gceSTATUS status;

    gcmkHEADER_ARG("Kernel=0x%x *Pool=0x%x",
                   Kernel, Information);
    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);

    /* Get the information. */
    status = gckVGCOMMAND_QueryCommandBuffer(Kernel->vg->command, Information);

    gcmkFOOTER();
    /* Return status. */
    return status;
}

#endif /* gcdENABLE_VG */
