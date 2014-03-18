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

#define _GC_OBJ_ZONE    gcvZONE_KERNEL

/*******************************************************************************
***** Version Signature *******************************************************/

#define _gcmTXT2STR(t) #t
#define gcmTXT2STR(t) _gcmTXT2STR(t)
const char * _VERSION = "\n\0$VERSION$"
                        gcmTXT2STR(gcvVERSION_MAJOR) "."
                        gcmTXT2STR(gcvVERSION_MINOR) "."
                        gcmTXT2STR(gcvVERSION_PATCH) ":"
                        gcmTXT2STR(gcvVERSION_BUILD) "$\n";

/******************************************************************************\
******************************* gckKERNEL API Code ******************************
\******************************************************************************/

#if gcmIS_DEBUG(gcdDEBUG_TRACE)
#define gcmDEFINE2TEXT(d) #d
gctCONST_STRING _DispatchText[] =
{
    gcmDEFINE2TEXT(gcvHAL_QUERY_VIDEO_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_QUERY_CHIP_IDENTITY),
    gcmDEFINE2TEXT(gcvHAL_ALLOCATE_NON_PAGED_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_FREE_NON_PAGED_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_ALLOCATE_CONTIGUOUS_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_FREE_CONTIGUOUS_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_ALLOCATE_VIDEO_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_ALLOCATE_LINEAR_VIDEO_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_FREE_VIDEO_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_MAP_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_UNMAP_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_MAP_USER_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_UNMAP_USER_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_LOCK_VIDEO_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_UNLOCK_VIDEO_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_EVENT_COMMIT),
    gcmDEFINE2TEXT(gcvHAL_USER_SIGNAL),
    gcmDEFINE2TEXT(gcvHAL_SIGNAL),
    gcmDEFINE2TEXT(gcvHAL_WRITE_DATA),
    gcmDEFINE2TEXT(gcvHAL_COMMIT),
    gcmDEFINE2TEXT(gcvHAL_STALL),
    gcmDEFINE2TEXT(gcvHAL_READ_REGISTER),
    gcmDEFINE2TEXT(gcvHAL_WRITE_REGISTER),
    gcmDEFINE2TEXT(gcvHAL_GET_PROFILE_SETTING),
    gcmDEFINE2TEXT(gcvHAL_SET_PROFILE_SETTING),
    gcmDEFINE2TEXT(gcvHAL_READ_ALL_PROFILE_REGISTERS),
#if VIVANTE_PROFILER_PERDRAW
    gcmDEFINE2TEXT(gcvHAL_READ_PROFILER_REGISTER_SETTING),
#endif
    gcmDEFINE2TEXT(gcvHAL_PROFILE_REGISTERS_2D),
    gcmDEFINE2TEXT(gcvHAL_SET_POWER_MANAGEMENT_STATE),
    gcmDEFINE2TEXT(gcvHAL_QUERY_POWER_MANAGEMENT_STATE),
    gcmDEFINE2TEXT(gcvHAL_GET_BASE_ADDRESS),
    gcmDEFINE2TEXT(gcvHAL_SET_IDLE),
    gcmDEFINE2TEXT(gcvHAL_QUERY_KERNEL_SETTINGS),
    gcmDEFINE2TEXT(gcvHAL_RESET),
    gcmDEFINE2TEXT(gcvHAL_MAP_PHYSICAL),
    gcmDEFINE2TEXT(gcvHAL_DEBUG),
    gcmDEFINE2TEXT(gcvHAL_CACHE),
    gcmDEFINE2TEXT(gcvHAL_TIMESTAMP),
    gcmDEFINE2TEXT(gcvHAL_DATABASE),
    gcmDEFINE2TEXT(gcvHAL_VERSION),
    gcmDEFINE2TEXT(gcvHAL_CHIP_INFO),
    gcmDEFINE2TEXT(gcvHAL_ATTACH),
    gcmDEFINE2TEXT(gcvHAL_DETACH)
};
#endif

#if gcdENABLE_RECOVERY
void
_ResetFinishFunction(
    gctPOINTER Data
    )
{
    gckKERNEL kernel = (gckKERNEL)Data;

    gckOS_AtomSet(kernel->os, kernel->resetAtom, 0);
}
#endif

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
**      gceCORE Core
**          Specified core.
**
**      IN gctPOINTER Context
**          Pointer to a driver defined context.
**
**      IN gckDB SharedDB,
**          Pointer to a shared DB.
**
**  OUTPUT:
**
**      gckKERNEL * Kernel
**          Pointer to a variable that will hold the pointer to the gckKERNEL
**          object.
*/

gceSTATUS
gckKERNEL_Construct(
    IN gckOS Os,
    IN gceCORE Core,
    IN gctPOINTER Context,
    IN gckDB SharedDB,
    OUT gckKERNEL * Kernel
    )
{
    gckKERNEL kernel = gcvNULL;
    gceSTATUS status;
    gctSIZE_T i;
    gctPOINTER pointer = gcvNULL;

    gcmkHEADER_ARG("Os=0x%x Context=0x%x", Os, Context);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Kernel != gcvNULL);

    /* Allocate the gckKERNEL object. */
    gcmkONERROR(gckOS_Allocate(Os,
                               gcmSIZEOF(struct _gckKERNEL),
                               &pointer));

    kernel = pointer;

    /* Zero the object pointers. */
    kernel->hardware     = gcvNULL;
    kernel->command      = gcvNULL;
    kernel->eventObj     = gcvNULL;
    kernel->mmu          = gcvNULL;
#if gcdDVFS
    kernel->dvfs         = gcvNULL;
#endif

    kernel->vidmemMutex  = gcvNULL;

    /* Initialize the gckKERNEL object. */
    kernel->object.type = gcvOBJ_KERNEL;
    kernel->os          = Os;
    kernel->core        = Core;


    if (SharedDB == gcvNULL)
    {
        gcmkONERROR(gckOS_Allocate(Os,
                                   gcmSIZEOF(struct _gckDB),
                                   &pointer));

        kernel->db               = pointer;
        kernel->dbCreated        = gcvTRUE;
        kernel->db->freeDatabase = gcvNULL;
        kernel->db->freeRecord   = gcvNULL;
        kernel->db->dbMutex      = gcvNULL;
        kernel->db->lastDatabase = gcvNULL;
        kernel->db->idleTime     = 0;
        kernel->db->lastIdle     = 0;
        kernel->db->lastSlowdown = 0;

        for (i = 0; i < gcmCOUNTOF(kernel->db->db); ++i)
        {
            kernel->db->db[i] = gcvNULL;
        }

        /* Construct a database mutex. */
        gcmkONERROR(gckOS_CreateMutex(Os, &kernel->db->dbMutex));

        /* Construct a id-pointer database. */
        gcmkONERROR(gckKERNEL_CreateIntegerDatabase(kernel, &kernel->db->pointerDatabase));

        /* Construct a id-pointer database mutex. */
        gcmkONERROR(gckOS_CreateMutex(Os, &kernel->db->pointerDatabaseMutex));
    }
    else
    {
        kernel->db               = SharedDB;
        kernel->dbCreated        = gcvFALSE;
    }

    for (i = 0; i < gcmCOUNTOF(kernel->timers); ++i)
    {
        kernel->timers[i].startTime = 0;
        kernel->timers[i].stopTime = 0;
    }

    kernel->timeOut      = gcdGPU_TIMEOUT;

    /* Save context. */
    kernel->context = Context;

#if gcdVIRTUAL_COMMAND_BUFFER
    kernel->virtualBufferHead =
    kernel->virtualBufferTail = gcvNULL;

    gcmkONERROR(
        gckOS_CreateMutex(Os, (gctPOINTER)&kernel->virtualBufferLock));
#endif

    /* Construct atom holding number of clients. */
    kernel->atomClients = gcvNULL;
    gcmkONERROR(gckOS_AtomConstruct(Os, &kernel->atomClients));

#if gcdENABLE_VG
    kernel->vg = gcvNULL;

    if (Core == gcvCORE_VG)
    {
        /* Construct the gckMMU object. */
        gcmkONERROR(
            gckVGKERNEL_Construct(Os, Context, kernel, &kernel->vg));
    }
    else
#endif
    {
        /* Construct the gckHARDWARE object. */
        gcmkONERROR(
            gckHARDWARE_Construct(Os, kernel->core, &kernel->hardware));

        /* Set pointer to gckKERNEL object in gckHARDWARE object. */
        kernel->hardware->kernel = kernel;

        /* Initialize the hardware. */
        gcmkONERROR(
            gckHARDWARE_InitializeHardware(kernel->hardware));

        /* Construct the gckCOMMAND object. */
        gcmkONERROR(
            gckCOMMAND_Construct(kernel, &kernel->command));

        /* Construct the gckEVENT object. */
        gcmkONERROR(
            gckEVENT_Construct(kernel, &kernel->eventObj));

        /* Construct the gckMMU object. */
        gcmkONERROR(
            gckMMU_Construct(kernel, gcdMMU_SIZE, &kernel->mmu));

#if gcdENABLE_RECOVERY
        gcmkONERROR(
            gckOS_AtomConstruct(Os, &kernel->resetAtom));

        gcmkVERIFY_OK(
            gckOS_CreateTimer(Os,
                              (gctTIMERFUNCTION)_ResetFinishFunction,
                              (gctPOINTER)kernel,
                              &kernel->resetFlagClearTimer));
        kernel->resetTimeStamp = 0;
#endif

#if gcdDVFS
        if (gckHARDWARE_IsFeatureAvailable(kernel->hardware,
                                           gcvFEATURE_DYNAMIC_FREQUENCY_SCALING))
        {
            gcmkONERROR(gckDVFS_Construct(kernel->hardware, &kernel->dvfs));
            gcmkONERROR(gckDVFS_Start(kernel->dvfs));
        }
#endif
    }

#if VIVANTE_PROFILER
    /* Initialize profile setting */
    kernel->profileEnable = gcvFALSE;
    kernel->profileCleanRegister = gcvTRUE;
#endif

#if gcdANDROID_NATIVE_FENCE_SYNC
    gcmkONERROR(gckOS_CreateSyncTimeline(Os, &kernel->timeline));
#endif

    /* Construct a video memory mutex. */
    gcmkONERROR(gckOS_CreateMutex(Os, &kernel->vidmemMutex));

    /* Return pointer to the gckKERNEL object. */
    *Kernel = kernel;

    /* Success. */
    gcmkFOOTER_ARG("*Kernel=0x%x", *Kernel);
    return gcvSTATUS_OK;

OnError:
    if (kernel != gcvNULL)
    {
#if gcdENABLE_VG
        if (Core != gcvCORE_VG)
#endif
        {
            if (kernel->eventObj != gcvNULL)
            {
                gcmkVERIFY_OK(gckEVENT_Destroy(kernel->eventObj));
            }

            if (kernel->command != gcvNULL)
            {
            gcmkVERIFY_OK(gckCOMMAND_Destroy(kernel->command));
            }

            if (kernel->hardware != gcvNULL)
            {
                /* Turn off the power. */
                gcmkVERIFY_OK(gckOS_SetGPUPower(kernel->hardware->os,
                                                kernel->hardware->core,
                                                gcvFALSE,
                                                gcvFALSE));
                gcmkVERIFY_OK(gckHARDWARE_Destroy(kernel->hardware));
            }
        }

        if (kernel->atomClients != gcvNULL)
        {
            gcmkVERIFY_OK(gckOS_AtomDestroy(Os, kernel->atomClients));
        }

#if gcdENABLE_RECOVERY
        if (kernel->resetAtom != gcvNULL)
        {
            gcmkVERIFY_OK(gckOS_AtomDestroy(Os, kernel->resetAtom));
        }

        if (kernel->resetFlagClearTimer)
        {
            gcmkVERIFY_OK(gckOS_StopTimer(Os, kernel->resetFlagClearTimer));
            gcmkVERIFY_OK(gckOS_DestroyTimer(Os, kernel->resetFlagClearTimer));
        }
#endif

        if (kernel->dbCreated && kernel->db != gcvNULL)
        {
            if (kernel->db->dbMutex != gcvNULL)
            {
                /* Destroy the database mutex. */
                gcmkVERIFY_OK(gckOS_DeleteMutex(Os, kernel->db->dbMutex));
            }

            gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Os, kernel->db));
        }

#if gcdVIRTUAL_COMMAND_BUFFER
        if (kernel->virtualBufferLock != gcvNULL)
        {
            /* Destroy the virtual command buffer mutex. */
            gcmkVERIFY_OK(gckOS_DeleteMutex(Os, kernel->virtualBufferLock));
        }
#endif

#if gcdDVFS
        if (kernel->dvfs)
        {
            gcmkVERIFY_OK(gckDVFS_Stop(kernel->dvfs));
            gcmkVERIFY_OK(gckDVFS_Destroy(kernel->dvfs));
        }
#endif

#if gcdANDROID_NATIVE_FENCE_SYNC
        if (kernel->timeline)
        {
            gcmkVERIFY_OK(gckOS_DestroySyncTimeline(Os, kernel->timeline));
        }
#endif

        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Os, kernel));
    }

    /* Return the error. */
    gcmkFOOTER();
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
gceSTATUS
gckKERNEL_Destroy(
    IN gckKERNEL Kernel
    )
{
    gctSIZE_T i;
    gcsDATABASE_PTR database, databaseNext;
    gcsDATABASE_RECORD_PTR record, recordNext;

    gcmkHEADER_ARG("Kernel=0x%x", Kernel);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
#if QNX_SINGLE_THREADED_DEBUGGING
    gcmkVERIFY_OK(gckOS_DeleteMutex(Kernel->os, Kernel->debugMutex));
#endif

    /* Destroy the database. */
    if (Kernel->dbCreated)
    {
        for (i = 0; i < gcmCOUNTOF(Kernel->db->db); ++i)
        {
            if (Kernel->db->db[i] != gcvNULL)
            {
                gcmkVERIFY_OK(
                    gckKERNEL_DestroyProcessDB(Kernel, Kernel->db->db[i]->processID));
            }
        }

        /* Free all databases. */
        for (database = Kernel->db->freeDatabase;
             database != gcvNULL;
             database = databaseNext)
        {
            databaseNext = database->next;
            gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Kernel->os, database));
        }

        if (Kernel->db->lastDatabase != gcvNULL)
        {
            gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Kernel->os, Kernel->db->lastDatabase));
        }

        /* Free all database records. */
        for (record = Kernel->db->freeRecord; record != gcvNULL; record = recordNext)
        {
            recordNext = record->next;
            gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Kernel->os, record));
        }

        /* Destroy the database mutex. */
        gcmkVERIFY_OK(gckOS_DeleteMutex(Kernel->os, Kernel->db->dbMutex));


        /* Destroy id-pointer database. */
        gcmkVERIFY_OK(gckKERNEL_DestroyIntegerDatabase(Kernel, Kernel->db->pointerDatabase));

        /* Destroy id-pointer database mutex. */
        gcmkVERIFY_OK(gckOS_DeleteMutex(Kernel->os, Kernel->db->pointerDatabaseMutex));
    }

#if gcdENABLE_VG
    if (Kernel->vg)
    {
        gcmkVERIFY_OK(gckVGKERNEL_Destroy(Kernel->vg));
    }
    else
#endif
    {
        /* Destroy the gckMMU object. */
        gcmkVERIFY_OK(gckMMU_Destroy(Kernel->mmu));

        /* Destroy the gckCOMMNAND object. */
        gcmkVERIFY_OK(gckCOMMAND_Destroy(Kernel->command));

        /* Destroy the gckEVENT object. */
        gcmkVERIFY_OK(gckEVENT_Destroy(Kernel->eventObj));

        /* Destroy the gckHARDWARE object. */
        gcmkVERIFY_OK(gckHARDWARE_Destroy(Kernel->hardware));

#if gcdENABLE_RECOVERY
        gcmkVERIFY_OK(gckOS_AtomDestroy(Kernel->os, Kernel->resetAtom));

        if (Kernel->resetFlagClearTimer)
        {
            gcmkVERIFY_OK(gckOS_StopTimer(Kernel->os, Kernel->resetFlagClearTimer));
            gcmkVERIFY_OK(gckOS_DestroyTimer(Kernel->os, Kernel->resetFlagClearTimer));
        }
#endif
    }

    /* Detsroy the client atom. */
    gcmkVERIFY_OK(gckOS_AtomDestroy(Kernel->os, Kernel->atomClients));

#if gcdVIRTUAL_COMMAND_BUFFER
    gcmkVERIFY_OK(gckOS_DeleteMutex(Kernel->os, Kernel->virtualBufferLock));
#endif

#if gcdDVFS
    if (Kernel->dvfs)
    {
        gcmkVERIFY_OK(gckDVFS_Stop(Kernel->dvfs));
        gcmkVERIFY_OK(gckDVFS_Destroy(Kernel->dvfs));
    }
#endif

#if gcdANDROID_NATIVE_FENCE_SYNC
    gcmkVERIFY_OK(gckOS_DestroySyncTimeline(Kernel->os, Kernel->timeline));
#endif

    gcmkVERIFY_OK(gckOS_DeleteMutex(Kernel->os, Kernel->vidmemMutex));

    /* Mark the gckKERNEL object as unknown. */
    Kernel->object.type = gcvOBJ_UNKNOWN;

    /* Free the gckKERNEL object. */
    gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Kernel->os, Kernel));

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

#ifdef CONFIG_ANDROID_RESERVED_MEMORY_ACCOUNT
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/oom.h>
#include <linux/sched.h>
#include <linux/notifier.h>

extern struct task_struct *lowmem_deathpending;
static unsigned long lowmem_deathpending_timeout;

static int force_contiguous_lowmem_shrink(IN gckKERNEL Kernel)
{
	struct task_struct *p;
	struct task_struct *selected = NULL;
	int tasksize;
        int ret = -1;
	int min_adj = 0;
	int selected_tasksize = 0;
	int selected_oom_adj;
	/*
	 * If we already have a death outstanding, then
	 * bail out right away; indicating to vmscan
	 * that we have nothing further to offer on
	 * this pass.
	 *
	 */
	if (lowmem_deathpending &&
	    time_before_eq(jiffies, lowmem_deathpending_timeout))
		return 0;
	selected_oom_adj = min_adj;

	read_lock(&tasklist_lock);
	for_each_process(p) {
		struct mm_struct *mm;
		struct signal_struct *sig;
                gcuDATABASE_INFO info;
		int oom_adj;

		task_lock(p);
		mm = p->mm;
		sig = p->signal;
		if (!mm || !sig) {
			task_unlock(p);
			continue;
		}
		oom_adj = sig->oom_adj;
		if (oom_adj < min_adj) {
			task_unlock(p);
			continue;
		}

		tasksize = 0;
		if (gckKERNEL_QueryProcessDB(Kernel, p->pid, gcvFALSE, gcvDB_VIDEO_MEMORY, &info) == gcvSTATUS_OK){
			tasksize += info.counters.bytes / PAGE_SIZE;
		}
		if (gckKERNEL_QueryProcessDB(Kernel, p->pid, gcvFALSE, gcvDB_CONTIGUOUS, &info) == gcvSTATUS_OK){
			tasksize += info.counters.bytes / PAGE_SIZE;
		}

		task_unlock(p);

		if (tasksize <= 0)
			continue;

		gckOS_Print("<gpu> pid %d (%s), adj %d, size %d \n", p->pid, p->comm, oom_adj, tasksize);

		if (selected) {
			if (oom_adj < selected_oom_adj)
				continue;
			if (oom_adj == selected_oom_adj &&
			    tasksize <= selected_tasksize)
				continue;
		}
		selected = p;
		selected_tasksize = tasksize;
		selected_oom_adj = oom_adj;
	}
	if (selected) {
		gckOS_Print("<gpu> send sigkill to %d (%s), adj %d, size %d\n",
			     selected->pid, selected->comm,
			     selected_oom_adj, selected_tasksize);
		lowmem_deathpending = selected;
		lowmem_deathpending_timeout = jiffies + HZ;
		force_sig(SIGKILL, selected);
		ret = 0;
	}
	read_unlock(&tasklist_lock);
	return ret;
}

#endif

/*******************************************************************************
**
**  _AllocateMemory
**
**  Private function to walk all required memory pools to allocate the requested
**  amount of video memory.
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
static gceSTATUS
_AllocateMemory(
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
    gctINT loopCount;
    gcuVIDMEM_NODE_PTR node = gcvNULL;
    gctBOOL tileStatusInVirtual;
    gctBOOL forceContiguous = gcvFALSE;

    gcmkHEADER_ARG("Kernel=0x%x *Pool=%d Bytes=%lu Alignment=%lu Type=%d",
                   Kernel, *Pool, Bytes, Alignment, Type);

    gcmkVERIFY_ARGUMENT(Pool != gcvNULL);
    gcmkVERIFY_ARGUMENT(Bytes != 0);

#ifdef CONFIG_ANDROID_RESERVED_MEMORY_ACCOUNT
_AllocateMemory_Retry:
#endif
    /* Get initial pool. */
    switch (pool = *Pool)
    {
    case gcvPOOL_DEFAULT_FORCE_CONTIGUOUS:
        forceContiguous = gcvTRUE;
    case gcvPOOL_DEFAULT:
    case gcvPOOL_LOCAL:
        pool      = gcvPOOL_LOCAL_INTERNAL;
        loopCount = (gctINT) gcvPOOL_NUMBER_OF_POOLS;
        break;

    case gcvPOOL_UNIFIED:
        pool      = gcvPOOL_SYSTEM;
        loopCount = (gctINT) gcvPOOL_NUMBER_OF_POOLS;
        break;

    case gcvPOOL_CONTIGUOUS:
        loopCount = (gctINT) gcvPOOL_NUMBER_OF_POOLS;
        break;

    case gcvPOOL_DEFAULT_FORCE_CONTIGUOUS_CACHEABLE:
        pool      = gcvPOOL_CONTIGUOUS;
        loopCount = 1;
        forceContiguous = gcvTRUE;
        break;

    default:
        loopCount = 1;
        break;
    }

    while (loopCount-- > 0)
    {
        if (pool == gcvPOOL_VIRTUAL)
        {
            /* Create a gcuVIDMEM_NODE for virtual memory. */
            gcmkONERROR(
                gckVIDMEM_ConstructVirtual(Kernel, gcvFALSE, Bytes, &node));

            /* Success. */
            break;
        }

        else
        if (pool == gcvPOOL_CONTIGUOUS)
        {
#if gcdCONTIGUOUS_SIZE_LIMIT
            if (Bytes > gcdCONTIGUOUS_SIZE_LIMIT && forceContiguous == gcvFALSE)
            {
                status = gcvSTATUS_OUT_OF_MEMORY;
            }
            else
#endif
            {
                /* Create a gcuVIDMEM_NODE from contiguous memory. */
                status = gckVIDMEM_ConstructVirtual(Kernel, gcvTRUE, Bytes, &node);
            }

            if (gcmIS_SUCCESS(status) || forceContiguous == gcvTRUE)
            {
                /* Memory allocated. */
                if(node && forceContiguous == gcvTRUE)
                {
                    gctUINT32 physAddr=0;
                    gctUINT32 baseAddress = 0;

                    gcmkONERROR(
                        gckOS_LockPages(Kernel->os,
                                        node->Virtual.physical,
                                        node->Virtual.bytes,
                                        gcvFALSE,
                                        &node->Virtual.logical,
                                        &node->Virtual.pageCount));

                    /* Convert logical address into a physical address. */
                    gcmkONERROR(
                        gckOS_GetPhysicalAddress(Kernel->os,
                                                 node->Virtual.logical,
                                                 &physAddr));

                    gcmkONERROR(
                        gckOS_UnlockPages(Kernel->os,
                                          node->Virtual.physical,
                                          node->Virtual.bytes,
                                          node->Virtual.logical));

                    gcmkONERROR(gckOS_GetBaseAddress(Kernel->os, &baseAddress));

                    gcmkASSERT(physAddr >= baseAddress);

                    /* Subtract baseAddress to get a GPU address used for programming. */
                    physAddr -= baseAddress;

                    if((physAddr & 0x80000000) || ((physAddr + Bytes) & 0x80000000))
                    {
                        gckOS_Print("gpu virtual memory 0x%x cannot be allocated for external use !\n", physAddr);

                        gcmkONERROR(gckVIDMEM_Free(Kernel, node));

                        node = gcvNULL;
                    }
                }

                break;
            }
        }

        else
        {
            /* Get pointer to gckVIDMEM object for pool. */
#if gcdUSE_VIDMEM_PER_PID
            gctUINT32 pid;
            gckOS_GetProcessID(&pid);

            status = gckKERNEL_GetVideoMemoryPoolPid(Kernel, pool, pid, &videoMemory);
            if (status == gcvSTATUS_NOT_FOUND)
            {
                /* Create VidMem pool for this process. */
                status = gckKERNEL_CreateVideoMemoryPoolPid(Kernel, pool, pid, &videoMemory);
            }
#else
            status = gckKERNEL_GetVideoMemoryPool(Kernel, pool, &videoMemory);
#endif

            if (gcmIS_SUCCESS(status))
            {
                /* Allocate memory. */
                status = gckVIDMEM_AllocateLinear(Kernel,
                                                  videoMemory,
                                                  Bytes,
                                                  Alignment,
                                                  Type,
                                                  &node);

                if (gcmIS_SUCCESS(status))
                {
                    /* Memory allocated. */
                    node->VidMem.pool = pool;
                    break;
                }
            }
        }

        if (pool == gcvPOOL_LOCAL_INTERNAL)
        {
            /* Advance to external memory. */
            pool = gcvPOOL_LOCAL_EXTERNAL;
        }

        else
        if (pool == gcvPOOL_LOCAL_EXTERNAL)
        {
            /* Advance to contiguous system memory. */
            pool = gcvPOOL_SYSTEM;
        }

        else
        if (pool == gcvPOOL_SYSTEM)
        {
            /* Advance to contiguous memory. */
            pool = gcvPOOL_CONTIGUOUS;
        }

        else
        if (pool == gcvPOOL_CONTIGUOUS)
        {
            tileStatusInVirtual =
                gckHARDWARE_IsFeatureAvailable(Kernel->hardware,
                                               gcvFEATURE_MC20);

            if (Type == gcvSURF_TILE_STATUS && tileStatusInVirtual != gcvTRUE)
            {
                gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
            }

            /* Advance to virtual memory. */
            pool = gcvPOOL_VIRTUAL;
        }

        else
        {
            /* Out of pools. */
            gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
        }
    }

    if (node == gcvNULL)
    {

#ifdef CONFIG_ANDROID_RESERVED_MEMORY_ACCOUNT
        if(forceContiguous == gcvTRUE)
        {
            if(force_contiguous_lowmem_shrink(Kernel) == 0)
            {
                 /* Sleep 1 millisecond. */
                 gckOS_Delay(gcvNULL, 1);
                 goto _AllocateMemory_Retry;
            }
        }
#endif
        /* Nothing allocated. */
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    /* Return node and pool used for allocation. */
    *Node = node;
    *Pool = pool;

    /* Return status. */
    gcmkFOOTER_ARG("*Pool=%d *Node=0x%x", *Pool, *Node);
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
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
**      gctBOOL FromUser
**          whether the call is from the user space.
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

gceSTATUS
gckKERNEL_Dispatch(
    IN gckKERNEL Kernel,
    IN gctBOOL FromUser,
    IN OUT gcsHAL_INTERFACE * Interface
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gctSIZE_T bytes;
    gcuVIDMEM_NODE_PTR node;
    gctBOOL locked = gcvFALSE;
    gctPHYS_ADDR physical = gcvNULL;
    gctPOINTER logical = gcvNULL;
    gctPOINTER info = gcvNULL;
    gckCONTEXT context = gcvNULL;
    gctUINT32 address;
    gctUINT32 processID;
    gckKERNEL kernel = Kernel;
#if gcdSECURE_USER
    gcskSECURE_CACHE_PTR cache;
#endif
    gctBOOL asynchronous;
    gctPOINTER paddr = gcvNULL;
#if !USE_NEW_LINUX_SIGNAL
    gctSIGNAL   signal;
#endif
    gceSURF_TYPE type;

    gcmkHEADER_ARG("Kernel=0x%x FromUser=%d Interface=0x%x",
                   Kernel, FromUser, Interface);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(Interface != gcvNULL);

#if gcmIS_DEBUG(gcdDEBUG_TRACE)
    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_KERNEL,
                   "Dispatching command %d (%s)",
                   Interface->command, _DispatchText[Interface->command]);
#endif
#if QNX_SINGLE_THREADED_DEBUGGING
    gckOS_AcquireMutex(Kernel->os, Kernel->debugMutex, gcvINFINITE);
#endif

    /* Get the current process ID. */
    gcmkONERROR(gckOS_GetProcessID(&processID));

#if gcdSECURE_USER
    gcmkONERROR(gckKERNEL_GetProcessDBCache(Kernel, processID, &cache));
#endif

    /* Dispatch on command. */
    switch (Interface->command)
    {
    case gcvHAL_GET_BASE_ADDRESS:
        /* Get base address. */
        gcmkONERROR(
            gckOS_GetBaseAddress(Kernel->os,
                                 &Interface->u.GetBaseAddress.baseAddress));
        break;

    case gcvHAL_QUERY_VIDEO_MEMORY:
        /* Query video memory size. */
        gcmkONERROR(gckKERNEL_QueryVideoMemory(Kernel, Interface));
        break;

    case gcvHAL_QUERY_CHIP_IDENTITY:
        /* Query chip identity. */
        gcmkONERROR(
            gckHARDWARE_QueryChipIdentity(
                Kernel->hardware,
                &Interface->u.QueryChipIdentity));
        break;

    case gcvHAL_MAP_MEMORY:
        physical = gcmINT2PTR(Interface->u.MapMemory.physical);

        /* Map memory. */
        gcmkONERROR(
            gckKERNEL_MapMemory(Kernel,
                                physical,
                                (gctSIZE_T) Interface->u.MapMemory.bytes,
                                &logical));

        Interface->u.MapMemory.logical = gcmPTR_TO_UINT64(logical);

        gcmkVERIFY_OK(
            gckKERNEL_AddProcessDB(Kernel,
                                   processID, gcvDB_MAP_MEMORY,
                                   logical,
                                   physical,
                                   (gctSIZE_T) Interface->u.MapMemory.bytes));
        break;

    case gcvHAL_UNMAP_MEMORY:
        physical = gcmINT2PTR(Interface->u.UnmapMemory.physical);

        /* Unmap memory. */
        gcmkONERROR(
            gckKERNEL_UnmapMemory(Kernel,
                                  physical,
                                  (gctSIZE_T) Interface->u.UnmapMemory.bytes,
                                  gcmUINT64_TO_PTR(Interface->u.UnmapMemory.logical)));
        gcmkVERIFY_OK(
            gckKERNEL_RemoveProcessDB(Kernel,
                                      processID, gcvDB_MAP_MEMORY,
                                      gcmUINT64_TO_PTR(Interface->u.UnmapMemory.logical)));
        break;

    case gcvHAL_ALLOCATE_NON_PAGED_MEMORY:
        bytes = (gctSIZE_T) Interface->u.AllocateNonPagedMemory.bytes;

        /* Allocate non-paged memory. */
        gcmkONERROR(
            gckOS_AllocateNonPagedMemory(
                Kernel->os,
                FromUser,
                &bytes,
                &physical,
                &logical));

        Interface->u.AllocateNonPagedMemory.bytes    = bytes;
        Interface->u.AllocateNonPagedMemory.logical  = gcmPTR_TO_UINT64(logical);
        Interface->u.AllocateNonPagedMemory.physical = gcmPTR_TO_NAME(physical);

        gcmkVERIFY_OK(
            gckKERNEL_AddProcessDB(Kernel,
                                   processID, gcvDB_NON_PAGED,
                                   logical,
                                   gcmINT2PTR(Interface->u.AllocateNonPagedMemory.physical),
                                   bytes));

        break;

    case gcvHAL_ALLOCATE_VIRTUAL_COMMAND_BUFFER:
#if gcdVIRTUAL_COMMAND_BUFFER
        bytes = (gctSIZE_T) Interface->u.AllocateVirtualCommandBuffer.bytes;

        gcmkONERROR(
            gckKERNEL_AllocateVirtualCommandBuffer(
                Kernel,
                FromUser,
                &bytes,
                &physical,
                &logical));

        Interface->u.AllocateVirtualCommandBuffer.bytes    = bytes;
        Interface->u.AllocateVirtualCommandBuffer.logical  = gcmPTR_TO_UINT64(logical);
        Interface->u.AllocateVirtualCommandBuffer.physical = gcmPTR_TO_NAME(physical);

        gcmkVERIFY_OK(
            gckKERNEL_AddProcessDB(Kernel,
                                   processID, gcvDB_COMMAND_BUFFER,
                                   logical,
                                   gcmINT2PTR(Interface->u.AllocateVirtualCommandBuffer.physical),
                                   bytes));
#else
        status = gcvSTATUS_NOT_SUPPORTED;
#endif
        break;

    case gcvHAL_FREE_NON_PAGED_MEMORY:
        physical = gcmNAME_TO_PTR(Interface->u.FreeNonPagedMemory.physical);

        /* Unmap user logical out of physical memory first. */
        gcmkONERROR(gckOS_UnmapUserLogical(Kernel->os,
                                           physical,
                                           (gctSIZE_T) Interface->u.FreeNonPagedMemory.bytes,
                                           gcmUINT64_TO_PTR(Interface->u.FreeNonPagedMemory.logical)));

        /* Free non-paged memory. */
        gcmkONERROR(
            gckOS_FreeNonPagedMemory(Kernel->os,
                                     (gctSIZE_T) Interface->u.FreeNonPagedMemory.bytes,
                                     physical,
                                     gcmUINT64_TO_PTR(Interface->u.FreeNonPagedMemory.logical)));

        gcmkVERIFY_OK(
            gckKERNEL_RemoveProcessDB(Kernel,
                                      processID, gcvDB_NON_PAGED,
                                      gcmUINT64_TO_PTR(Interface->u.FreeNonPagedMemory.logical)));

#if gcdSECURE_USER
        gcmkVERIFY_OK(gckKERNEL_FlushTranslationCache(
            Kernel,
            cache,
            gcmUINT64_TO_PTR(Interface->u.FreeNonPagedMemory.logical),
            Interface->u.FreeNonPagedMemory.bytes));
#endif

        gcmRELEASE_NAME(Interface->u.FreeNonPagedMemory.physical);

        break;

    case gcvHAL_ALLOCATE_CONTIGUOUS_MEMORY:
        bytes = (gctSIZE_T) Interface->u.AllocateContiguousMemory.bytes;

        /* Allocate contiguous memory. */
        gcmkONERROR(gckOS_AllocateContiguous(
            Kernel->os,
            FromUser,
            &bytes,
            &physical,
            &logical));

        Interface->u.AllocateContiguousMemory.bytes    = bytes;
        Interface->u.AllocateContiguousMemory.logical  = gcmPTR_TO_UINT64(logical);
        Interface->u.AllocateContiguousMemory.physical = gcmPTR_TO_NAME(physical);

        gcmkONERROR(gckHARDWARE_ConvertLogical(
            Kernel->hardware,
            gcmUINT64_TO_PTR(Interface->u.AllocateContiguousMemory.logical),
            &Interface->u.AllocateContiguousMemory.address));

        gcmkVERIFY_OK(gckKERNEL_AddProcessDB(
            Kernel,
            processID, gcvDB_CONTIGUOUS,
            logical,
            gcmINT2PTR(Interface->u.AllocateContiguousMemory.physical),
            bytes));

        break;

    case gcvHAL_FREE_CONTIGUOUS_MEMORY:
        physical = gcmNAME_TO_PTR(Interface->u.FreeContiguousMemory.physical);

        /* Unmap user logical out of physical memory first. */
        gcmkONERROR(gckOS_UnmapUserLogical(Kernel->os,
                                           physical,
                                           (gctSIZE_T) Interface->u.FreeContiguousMemory.bytes,
                                           gcmUINT64_TO_PTR(Interface->u.FreeContiguousMemory.logical)));

        /* Free contiguous memory. */
        gcmkONERROR(
            gckOS_FreeContiguous(Kernel->os,
                                 physical,
                                 gcmUINT64_TO_PTR(Interface->u.FreeContiguousMemory.logical),
                                 (gctSIZE_T) Interface->u.FreeContiguousMemory.bytes));

        gcmkVERIFY_OK(
            gckKERNEL_RemoveProcessDB(Kernel,
                                      processID, gcvDB_CONTIGUOUS,
                                      gcmUINT64_TO_PTR(Interface->u.FreeNonPagedMemory.logical)));

#if gcdSECURE_USER
        gcmkVERIFY_OK(gckKERNEL_FlushTranslationCache(
            Kernel,
            cache,
            gcmUINT64_TO_PTR(Interface->u.FreeContiguousMemory.logical),
            Interface->u.FreeContiguousMemory.bytes));
#endif

        gcmRELEASE_NAME(Interface->u.FreeContiguousMemory.physical);

        break;

    case gcvHAL_ALLOCATE_VIDEO_MEMORY:

        gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);

        break;

    case gcvHAL_ALLOCATE_LINEAR_VIDEO_MEMORY:
        type = Interface->u.AllocateLinearVideoMemory.type;

        /* Allocate memory. */
        gcmkONERROR(
            _AllocateMemory(Kernel,
                            &Interface->u.AllocateLinearVideoMemory.pool,
                            Interface->u.AllocateLinearVideoMemory.bytes,
                            Interface->u.AllocateLinearVideoMemory.alignment,
                            Interface->u.AllocateLinearVideoMemory.type,
                            &node));

        if (node->VidMem.memory->object.type == gcvOBJ_VIDMEM)
        {
            bytes = node->VidMem.bytes;
            node->VidMem.type = type;

            gcmkONERROR(
                gckKERNEL_AddProcessDB(Kernel,
                                   processID, gcvDB_VIDEO_MEMORY_RESERVED,
                                   node,
                                   gcvNULL,
                                   bytes));
        }
        else
        {
            bytes = node->Virtual.bytes;
            node->Virtual.type = type;

            if(node->Virtual.contiguous)
            {
                gcmkONERROR(
                    gckKERNEL_AddProcessDB(Kernel,
                                   processID, gcvDB_VIDEO_MEMORY_CONTIGUOUS,
                                   node,
                                   gcvNULL,
                                   bytes));
            }
            else
            {
                gcmkONERROR(
                    gckKERNEL_AddProcessDB(Kernel,
                                   processID, gcvDB_VIDEO_MEMORY_VIRTUAL,
                                   node,
                                   gcvNULL,
                                   bytes));
            }

        }

        gcmkONERROR(
            gckKERNEL_AddProcessDB(Kernel,
                                   processID, gcvDB_VIDEO_MEMORY,
                                   node,
                                   gcvNULL,
                                   bytes));

        /* Get the node. */
        Interface->u.AllocateLinearVideoMemory.node = gcmPTR_TO_UINT64(node);
        break;

    case gcvHAL_FREE_VIDEO_MEMORY:
        node = gcmUINT64_TO_PTR(Interface->u.FreeVideoMemory.node);
#ifdef __QNXNTO__
        if (node->VidMem.memory->object.type == gcvOBJ_VIDMEM
         && node->VidMem.logical != gcvNULL)
        {
            gcmkONERROR(
                    gckKERNEL_UnmapVideoMemory(Kernel,
                                               node->VidMem.logical,
                                               processID,
                                               node->VidMem.bytes));
            node->VidMem.logical = gcvNULL;
        }
#endif
        /* Free video memory. */
        gcmkONERROR(
            gckVIDMEM_Free(Kernel, node));

        gcmkONERROR(
            gckKERNEL_RemoveProcessDB(Kernel,
                                      processID, gcvDB_VIDEO_MEMORY,
                                      node));

        if (node->VidMem.memory->object.type == gcvOBJ_VIDMEM)
        {
           gcmkONERROR(
                gckKERNEL_RemoveProcessDB(Kernel,
                                      processID, gcvDB_VIDEO_MEMORY_RESERVED,
                                      node));
        }
        else if(node->Virtual.contiguous)
        {
            gcmkONERROR(
                gckKERNEL_RemoveProcessDB(Kernel,
                                      processID, gcvDB_VIDEO_MEMORY_CONTIGUOUS,
                                      node));
        }
        else
        {
            gcmkONERROR(
                gckKERNEL_RemoveProcessDB(Kernel,
                                      processID, gcvDB_VIDEO_MEMORY_VIRTUAL,
                                      node));
        }

        break;

    case gcvHAL_LOCK_VIDEO_MEMORY:
        node = gcmUINT64_TO_PTR(Interface->u.LockVideoMemory.node);

        /* Lock video memory. */
        gcmkONERROR(
            gckVIDMEM_Lock(Kernel,
                           node,
                           Interface->u.LockVideoMemory.cacheable,
                           &Interface->u.LockVideoMemory.address));

        locked = gcvTRUE;

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
        gcmkASSERT(node->VidMem.logical != gcvNULL);

        Interface->u.LockVideoMemory.memory = gcmPTR_TO_UINT64(node->VidMem.logical);
#else
            gcmkONERROR(
                gckKERNEL_MapVideoMemory(Kernel,
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
            Interface->u.LockVideoMemory.memory;
#endif
        gcmkONERROR(
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
        gcmkONERROR(
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
            gcmkONERROR(
                gckKERNEL_RemoveProcessDB(Kernel,
                                          processID, gcvDB_VIDEO_MEMORY_LOCKED,
                                          node));
        }
        break;

    case gcvHAL_EVENT_COMMIT:
        /* Commit an event queue. */
        gcmkONERROR(
            gckEVENT_Commit(Kernel->eventObj,
                            gcmUINT64_TO_PTR(Interface->u.Event.queue)));
        break;

    case gcvHAL_COMMIT:
        /* Commit a command and context buffer. */
        gcmkONERROR(
            gckCOMMAND_Commit(Kernel->command,
                              Interface->u.Commit.context ?
                                  gcmNAME_TO_PTR(Interface->u.Commit.context) : gcvNULL,
                              gcmUINT64_TO_PTR(Interface->u.Commit.commandBuffer),
                              gcmUINT64_TO_PTR(Interface->u.Commit.delta),
                              gcmUINT64_TO_PTR(Interface->u.Commit.queue),
                              processID));
        break;

    case gcvHAL_STALL:
        /* Stall the command queue. */
        gcmkONERROR(gckCOMMAND_Stall(Kernel->command, gcvFALSE));
        break;

    case gcvHAL_MAP_USER_MEMORY:
        /* Map user memory to DMA. */
        gcmkONERROR(
            gckOS_MapUserMemory(Kernel->os,
                                Kernel->core,
                                gcmUINT64_TO_PTR(Interface->u.MapUserMemory.memory),
                                Interface->u.MapUserMemory.physical,
                                (gctSIZE_T) Interface->u.MapUserMemory.size,
                                &info,
                                &Interface->u.MapUserMemory.address));

        Interface->u.MapUserMemory.info = gcmPTR_TO_NAME(info);

        gcmkVERIFY_OK(
            gckKERNEL_AddProcessDB(Kernel,
                                   processID, gcvDB_MAP_USER_MEMORY,
                                   gcmINT2PTR(Interface->u.MapUserMemory.info),
                                   gcmUINT64_TO_PTR(Interface->u.MapUserMemory.memory),
                                   (gctSIZE_T) Interface->u.MapUserMemory.size));
        break;

    case gcvHAL_UNMAP_USER_MEMORY:
        address = Interface->u.UnmapUserMemory.address;
        info = gcmNAME_TO_PTR(Interface->u.UnmapUserMemory.info);

        /* Unmap user memory. */
        gcmkONERROR(
            gckOS_UnmapUserMemory(Kernel->os,
                                  Kernel->core,
                                  gcmUINT64_TO_PTR(Interface->u.UnmapUserMemory.memory),
                                  (gctSIZE_T) Interface->u.UnmapUserMemory.size,
                                  info,
                                  address));

#if gcdSECURE_USER
        gcmkVERIFY_OK(gckKERNEL_FlushTranslationCache(
            Kernel,
            cache,
            gcmUINT64_TO_PTR(Interface->u.UnmapUserMemory.memory),
            Interface->u.UnmapUserMemory.size));
#endif
        gcmkVERIFY_OK(
            gckKERNEL_RemoveProcessDB(Kernel,
                                      processID, gcvDB_MAP_USER_MEMORY,
                                      gcmINT2PTR(Interface->u.UnmapUserMemory.info)));

        gcmRELEASE_NAME(Interface->u.UnmapUserMemory.info);

        break;

#if !USE_NEW_LINUX_SIGNAL
    case gcvHAL_USER_SIGNAL:
        /* Dispatch depends on the user signal subcommands. */
        switch(Interface->u.UserSignal.command)
        {
        case gcvUSER_SIGNAL_CREATE:
            /* Create a signal used in the user space. */
            gcmkONERROR(
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
            gcmkONERROR(
                gckOS_DestroyUserSignal(Kernel->os,
                                        Interface->u.UserSignal.id));

            gcmkVERIFY_OK(gckKERNEL_RemoveProcessDB(
                Kernel,
                processID, gcvDB_SIGNAL,
                gcmINT2PTR(Interface->u.UserSignal.id)));
            break;

        case gcvUSER_SIGNAL_SIGNAL:
            /* Signal the signal. */
            gcmkONERROR(
                gckOS_SignalUserSignal(Kernel->os,
                                       Interface->u.UserSignal.id,
                                       Interface->u.UserSignal.state));
            break;

        case gcvUSER_SIGNAL_WAIT:
#if gcdGPU_TIMEOUT
            if (Interface->u.UserSignal.wait == gcvINFINITE)
            {
                gckHARDWARE hardware;
                gctUINT32 timer = 0;

                for(;;)
                {
                    /* Wait on the signal. */
                    status = gckOS_WaitUserSignal(Kernel->os,
                                                  Interface->u.UserSignal.id,
                                                  gcdGPU_ADVANCETIMER);

                    if (status == gcvSTATUS_TIMEOUT)
                    {
                        gcmkONERROR(
                            gckOS_SignalQueryHardware(Kernel->os,
                                                      (gctSIGNAL)(gctUINTPTR_T)Interface->u.UserSignal.id,
                                                      &hardware));

                        if (hardware)
                        {
                            /* This signal is bound to a hardware,
                            ** so the timeout is limited by Kernel->timeOut.
                            */
                            timer += gcdGPU_ADVANCETIMER;
                        }

                        if (timer >= Kernel->timeOut)
                        {
                            gcmkONERROR(
                                gckOS_Broadcast(Kernel->os,
                                                hardware,
                                                gcvBROADCAST_GPU_STUCK));

                            timer = 0;

                            /* If a few process try to reset GPU, only one
                            ** of them can do the real reset, other processes
                            ** still need to wait for this signal is triggered,
                            ** which menas reset is finished.
                            */
                            continue;
                        }
                    }
                    else
                    {
                        /* Bail out on other error. */
                        gcmkONERROR(status);

                        /* Wait for signal successfully. */
                        break;
                    }
                }
            }
            else
#endif
            {
                /* Wait on the signal. */
                status = gckOS_WaitUserSignal(Kernel->os,
                                              Interface->u.UserSignal.id,
                                              Interface->u.UserSignal.wait);
            }

            break;

        case gcvUSER_SIGNAL_MAP:
            gcmkONERROR(
                gckOS_MapSignal(Kernel->os,
                               (gctSIGNAL)(gctUINTPTR_T)Interface->u.UserSignal.id,
                               (gctHANDLE)(gctUINTPTR_T)processID,
                               &signal));

            gcmkVERIFY_OK(
                gckKERNEL_AddProcessDB(Kernel,
                                       processID, gcvDB_SIGNAL,
                                       gcmINT2PTR(Interface->u.UserSignal.id),
                                       gcvNULL,
                                       0));
            break;

        case gcvUSER_SIGNAL_UNMAP:
            /* Destroy the signal. */
            gcmkONERROR(
                gckOS_DestroyUserSignal(Kernel->os,
                                        Interface->u.UserSignal.id));

            gcmkVERIFY_OK(gckKERNEL_RemoveProcessDB(
                Kernel,
                processID, gcvDB_SIGNAL,
                gcmINT2PTR(Interface->u.UserSignal.id)));
            break;

        default:
            /* Invalid user signal command. */
            gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        }
        break;
#endif

    case gcvHAL_SET_POWER_MANAGEMENT_STATE:
        /* Set the power management state. */
        gcmkONERROR(
            gckHARDWARE_SetPowerManagementState(
                Kernel->hardware,
                Interface->u.SetPowerManagement.state));
        break;

    case gcvHAL_QUERY_POWER_MANAGEMENT_STATE:
        /* Chip is not idle. */
        Interface->u.QueryPowerManagement.isIdle = gcvFALSE;

        /* Query the power management state. */
        gcmkONERROR(gckHARDWARE_QueryPowerManagementState(
            Kernel->hardware,
            &Interface->u.QueryPowerManagement.state));

        /* Query the idle state. */
        gcmkONERROR(
            gckHARDWARE_QueryIdle(Kernel->hardware,
                                  &Interface->u.QueryPowerManagement.isIdle));
        break;

    case gcvHAL_READ_REGISTER:
#if gcdREGISTER_ACCESS_FROM_USER
        {
            gceCHIPPOWERSTATE power;

            gckOS_AcquireMutex(Kernel->os, Kernel->hardware->powerMutex, gcvINFINITE);
            gcmkONERROR(gckHARDWARE_QueryPowerManagementState(Kernel->hardware,
                                                              &power));
            if (power == gcvPOWER_ON)
            {
                /* Read a register. */
                gcmkONERROR(gckOS_ReadRegisterEx(
                    Kernel->os,
                    Kernel->core,
                    Interface->u.ReadRegisterData.address,
                    &Interface->u.ReadRegisterData.data));
            }
            else
            {
                /* Chip is in power-state. */
                Interface->u.ReadRegisterData.data = 0;
                status = gcvSTATUS_CHIP_NOT_READY;
            }
            gcmkONERROR(gckOS_ReleaseMutex(Kernel->os, Kernel->hardware->powerMutex));
        }
#else
        /* No access from user land to read registers. */
        Interface->u.ReadRegisterData.data = 0;
        status = gcvSTATUS_NOT_SUPPORTED;
#endif
        break;

    case gcvHAL_WRITE_REGISTER:
#if gcdREGISTER_ACCESS_FROM_USER
        {
            gceCHIPPOWERSTATE power;

            gckOS_AcquireMutex(Kernel->os, Kernel->hardware->powerMutex, gcvINFINITE);
            gcmkONERROR(gckHARDWARE_QueryPowerManagementState(Kernel->hardware,
                                                                  &power));
            if (power == gcvPOWER_ON)
            {
                /* Write a register. */
                gcmkONERROR(
                    gckOS_WriteRegisterEx(Kernel->os,
                                          Kernel->core,
                                          Interface->u.WriteRegisterData.address,
                                          Interface->u.WriteRegisterData.data));
            }
            else
            {
                /* Chip is in power-state. */
                Interface->u.WriteRegisterData.data = 0;
                status = gcvSTATUS_CHIP_NOT_READY;
            }
            gcmkONERROR(gckOS_ReleaseMutex(Kernel->os, Kernel->hardware->powerMutex));
        }
#else
        /* No access from user land to write registers. */
        status = gcvSTATUS_NOT_SUPPORTED;
#endif
        break;

    case gcvHAL_READ_ALL_PROFILE_REGISTERS:
#if VIVANTE_PROFILER && VIVANTE_PROFILER_CONTEXT
        /* Read profile data according to the context. */
        gcmkONERROR(
            gckHARDWARE_QueryContextProfile(
                Kernel->hardware,
                Kernel->profileCleanRegister,
                gcmNAME_TO_PTR(Interface->u.RegisterProfileData.context),
                &Interface->u.RegisterProfileData.counters));
#elif VIVANTE_PROFILER
        /* Read all 3D profile registers. */
        gcmkONERROR(
            gckHARDWARE_QueryProfileRegisters(
                Kernel->hardware,
                Kernel->profileCleanRegister,
                &Interface->u.RegisterProfileData.counters));
#else
        status = gcvSTATUS_OK;
#endif
        break;

    case gcvHAL_PROFILE_REGISTERS_2D:
#if VIVANTE_PROFILER
        /* Read all 2D profile registers. */
        gcmkONERROR(
            gckHARDWARE_ProfileEngine2D(
                Kernel->hardware,
                gcmUINT64_TO_PTR(Interface->u.RegisterProfileData2D.hwProfile2D)));
#else
        status = gcvSTATUS_OK;
#endif
        break;

    case gcvHAL_GET_PROFILE_SETTING:
#if VIVANTE_PROFILER
        /* Get profile setting */
        Interface->u.GetProfileSetting.enable = Kernel->profileEnable;
#endif

        status = gcvSTATUS_OK;
        break;
    case gcvHAL_SET_PROFILE_SETTING:
#if VIVANTE_PROFILER
        /* Set profile setting */
        if(Kernel->hardware->gpuProfiler)
            Kernel->profileEnable = Interface->u.SetProfileSetting.enable;
        else
        {
            status = gcvSTATUS_NOT_SUPPORTED;
            break;
        }
#endif

        status = gcvSTATUS_OK;
        break;

#if VIVANTE_PROFILER_PERDRAW
    case gcvHAL_READ_PROFILER_REGISTER_SETTING:
    #if VIVANTE_PROFILER
        Kernel->profileCleanRegister = Interface->u.SetProfilerRegisterClear.bclear;
    #endif
        status = gcvSTATUS_OK;
        break;
#endif

    case gcvHAL_QUERY_KERNEL_SETTINGS:
        /* Get kernel settings. */
        gcmkONERROR(
            gckKERNEL_QuerySettings(Kernel,
                                    &Interface->u.QueryKernelSettings.settings));
        break;

    case gcvHAL_RESET:
        /* Reset the hardware. */
        gckKERNEL_Recovery(Kernel);
        break;

    case gcvHAL_DEBUG:
        /* Set debug level and zones. */
        if (Interface->u.Debug.set)
        {
            gckOS_SetDebugLevel(Interface->u.Debug.level);
            gckOS_SetDebugZones(Interface->u.Debug.zones,
                                Interface->u.Debug.enable);
        }

        if (Interface->u.Debug.message[0] != '\0')
        {
            /* Print a message to the debugger. */
            if (Interface->u.Debug.type == gcvMESSAGE_TEXT)
            {
               gckOS_CopyPrint(Interface->u.Debug.message);
            }
            else
            {
               gckOS_DumpBuffer(Kernel->os,
                                Interface->u.Debug.message,
                                Interface->u.Debug.messageSize,
                                gceDUMP_BUFFER_FROM_USER,
                                gcvTRUE);
            }
        }
        status = gcvSTATUS_OK;
        break;

    case gcvHAL_DUMP_GPU_STATE:
        /* Dump GPU state */
        {
            gceCHIPPOWERSTATE power;
            gcmkONERROR(gckHARDWARE_QueryPowerManagementState(Kernel->hardware,
                                                              &power));
            if (power == gcvPOWER_ON)
            {
                Interface->u.ReadRegisterData.data = 1;
                gcmkVERIFY_OK(
                    gckHARDWARE_DumpGPUState(Kernel->hardware));
#if gcdVIRTUAL_COMMAND_BUFFER
                gcmkVERIFY_OK(
                    gckCOMMAND_DumpExecutingBuffer(Kernel->command));
#endif
            }
            else
            {
                Interface->u.ReadRegisterData.data = 0;
                status = gcvSTATUS_CHIP_NOT_READY;
            }
        }
        break;

    case gcvHAL_DUMP_EVENT:
        /* Dump GPU event */
        gcmkVERIFY_OK(gckEVENT_Dump(Kernel->eventObj));

        /* Dump Process DB. */
        gcmkVERIFY_OK(gckKERNEL_DumpProcessDB(Kernel));
        break;

    case gcvHAL_CACHE:
        node = gcmUINT64_TO_PTR(Interface->u.Cache.node);
        if (node == gcvNULL)
        {
            /* FIXME Surface wrap some memory which is not allocated by us,
            ** So we don't have physical address to handle outer cache, ignore it*/
            status = gcvSTATUS_OK;
            break;
        }
        else if (node->VidMem.memory->object.type == gcvOBJ_VIDMEM)
        {
            /* Video memory has no physical handles. */
            physical = gcvNULL;
        }
        else
        {
            /* Grab physical handle. */
            physical = node->Virtual.physical;
        }

        logical = gcmUINT64_TO_PTR(Interface->u.Cache.logical);
        bytes = (gctSIZE_T) Interface->u.Cache.bytes;
        switch(Interface->u.Cache.operation)
        {
        case gcvCACHE_FLUSH:
            /* Clean and invalidate the cache. */
            status = gckOS_CacheFlush(Kernel->os,
                                      processID,
                                      physical,
                                      paddr,
                                      logical,
                                      bytes);
            break;
        case gcvCACHE_CLEAN:
            /* Clean the cache. */
            status = gckOS_CacheClean(Kernel->os,
                                      processID,
                                      physical,
                                      paddr,
                                      logical,
                                      bytes);
            break;
        case gcvCACHE_INVALIDATE:
            /* Invalidate the cache. */
            status = gckOS_CacheInvalidate(Kernel->os,
                                           processID,
                                           physical,
                                           paddr,
                                           logical,
                                           bytes);
            break;

	case gcvCACHE_MEMORY_BARRIER:
	   status = gckOS_MemoryBarrier(Kernel->os,
                                        logical);
	   break;
        default:
            status = gcvSTATUS_INVALID_ARGUMENT;
            break;
        }
        break;

    case gcvHAL_TIMESTAMP:
        /* Check for invalid timer. */
        if ((Interface->u.TimeStamp.timer >= gcmCOUNTOF(Kernel->timers))
        ||  (Interface->u.TimeStamp.request != 2))
        {
            Interface->u.TimeStamp.timeDelta = 0;
            gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        }

        /* Return timer results and reset timer. */
        {
            gcsTIMER_PTR timer = &(Kernel->timers[Interface->u.TimeStamp.timer]);
            gctUINT64 timeDelta = 0;

            if (timer->stopTime < timer->startTime )
            {
                Interface->u.TimeStamp.timeDelta = 0;
                gcmkONERROR(gcvSTATUS_TIMER_OVERFLOW);
            }

            timeDelta = timer->stopTime - timer->startTime;

            /* Check truncation overflow. */
            Interface->u.TimeStamp.timeDelta = (gctINT32) timeDelta;
			/*bit0~bit30 is available*/
            if (timeDelta>>31)
            {
                Interface->u.TimeStamp.timeDelta = 0;
                gcmkONERROR(gcvSTATUS_TIMER_OVERFLOW);
            }

            status = gcvSTATUS_OK;
        }
        break;

    case gcvHAL_DATABASE:
        /* Query video memory. */
        gcmkONERROR(
            gckKERNEL_QueryProcessDB(Kernel,
                                     Interface->u.Database.processID,
                                     !Interface->u.Database.validProcessID,
                                     gcvDB_VIDEO_MEMORY,
                                     &Interface->u.Database.vidMem));

        /* Query non-paged memory. */
        gcmkONERROR(
            gckKERNEL_QueryProcessDB(Kernel,
                                     Interface->u.Database.processID,
                                     !Interface->u.Database.validProcessID,
                                     gcvDB_NON_PAGED,
                                     &Interface->u.Database.nonPaged));

        /* Query contiguous memory. */
        gcmkONERROR(
            gckKERNEL_QueryProcessDB(Kernel,
                                     Interface->u.Database.processID,
                                     !Interface->u.Database.validProcessID,
                                     gcvDB_CONTIGUOUS,
                                     &Interface->u.Database.contiguous));

        /* Query GPU idle time. */
        gcmkONERROR(
            gckKERNEL_QueryProcessDB(Kernel,
                                     Interface->u.Database.processID,
                                     !Interface->u.Database.validProcessID,
                                     gcvDB_IDLE,
                                     &Interface->u.Database.gpuIdle));
        break;

    case gcvHAL_VIDMEM_DATABASE:
        /* Query reserved video memory. */
        gcmkONERROR(
            gckKERNEL_QueryProcessDB(Kernel,
                                     Interface->u.VidMemDatabase.processID,
                                     !Interface->u.VidMemDatabase.validProcessID,
                                     gcvDB_VIDEO_MEMORY_RESERVED,
                                     &Interface->u.VidMemDatabase.vidMemResv));

        /* Query contiguous video memory. */
        gcmkONERROR(
            gckKERNEL_QueryProcessDB(Kernel,
                                     Interface->u.VidMemDatabase.processID,
                                     !Interface->u.VidMemDatabase.validProcessID,
                                     gcvDB_VIDEO_MEMORY_CONTIGUOUS,
                                     &Interface->u.VidMemDatabase.vidMemCont));

        /* Query virtual video memory. */
        gcmkONERROR(
            gckKERNEL_QueryProcessDB(Kernel,
                                     Interface->u.VidMemDatabase.processID,
                                     !Interface->u.VidMemDatabase.validProcessID,
                                     gcvDB_VIDEO_MEMORY_VIRTUAL,
                                     &Interface->u.VidMemDatabase.vidMemVirt));

        break;

    case gcvHAL_VERSION:
        Interface->u.Version.major = gcvVERSION_MAJOR;
        Interface->u.Version.minor = gcvVERSION_MINOR;
        Interface->u.Version.patch = gcvVERSION_PATCH;
        Interface->u.Version.build = gcvVERSION_BUILD;
#if gcmIS_DEBUG(gcdDEBUG_TRACE)
        gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_KERNEL,
                       "KERNEL version %d.%d.%d build %u %s %s",
                       gcvVERSION_MAJOR, gcvVERSION_MINOR, gcvVERSION_PATCH,
                       gcvVERSION_BUILD, gcvVERSION_DATE, gcvVERSION_TIME);
#endif
        break;

    case gcvHAL_CHIP_INFO:
        /* Only if not support multi-core */
        Interface->u.ChipInfo.count = 1;
        Interface->u.ChipInfo.types[0] = Kernel->hardware->type;
        break;

    case gcvHAL_ATTACH:
        /* Attach user process. */
        gcmkONERROR(
            gckCOMMAND_Attach(Kernel->command,
                              &context,
                              &bytes,
                              processID));

        Interface->u.Attach.stateCount = bytes;
        Interface->u.Attach.context = gcmPTR_TO_NAME(context);

        gcmkVERIFY_OK(
            gckKERNEL_AddProcessDB(Kernel,
                                   processID, gcvDB_CONTEXT,
                                   gcmINT2PTR(Interface->u.Attach.context),
                                   gcvNULL,
                                   0));
        break;

    case gcvHAL_DETACH:
        /* Detach user process. */
        gcmkONERROR(
            gckCOMMAND_Detach(Kernel->command,
                              gcmNAME_TO_PTR(Interface->u.Detach.context)));

        gcmkVERIFY_OK(
            gckKERNEL_RemoveProcessDB(Kernel,
                              processID, gcvDB_CONTEXT,
                              gcmINT2PTR(Interface->u.Detach.context)));

        gcmRELEASE_NAME(Interface->u.Detach.context);
        break;

    case gcvHAL_COMPOSE:
        Interface->u.Compose.physical = gcmPTR_TO_UINT64(gcmNAME_TO_PTR(Interface->u.Compose.physical));
        /* Start composition. */
        gcmkONERROR(
            gckEVENT_Compose(Kernel->eventObj,
                             &Interface->u.Compose));
        break;

    case gcvHAL_SET_TIMEOUT:
         /* set timeOut value from user */
         gckKERNEL_SetTimeOut(Kernel, Interface->u.SetTimeOut.timeOut);
        break;

#if gcdFRAME_DB
    case gcvHAL_GET_FRAME_INFO:
        gcmkONERROR(gckHARDWARE_GetFrameInfo(
            Kernel->hardware,
            gcmUINT64_TO_PTR(Interface->u.GetFrameInfo.frameInfo)));
        break;
#endif

    case gcvHAL_GET_SHARED_INFO:
        if (Interface->u.GetSharedInfo.data == gcvNULL)
        {
            gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        }
        else
        {
            gctUINT32 pid    = Interface->u.GetSharedInfo.pid;
            gctUINT32 dataId = Interface->u.GetSharedInfo.dataId;
            gctSIZE_T bytes  = Interface->u.GetSharedInfo.bytes;
            gctPOINTER data  = Interface->u.GetSharedInfo.data;
            gcsDATABASE_RECORD record;

            /* Find record. */
            gcmkONERROR(
                gckKERNEL_FindProcessDB(Kernel,
                                        pid,
                                        0,
                                        gcvDB_SHARED_INFO,
                                        gcmINT2PTR(dataId),
                                        &record));

            /* Check memory size. */
            if (bytes < record.bytes)
            {
                /* Insufficient memory to hold shared data. */
                gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
            }

            /* Copy to user. */
            status = gckOS_CopyToUserData(Kernel->os,
                                          record.physical,
                                          data,
                                          record.bytes);

            /*
             * Remove from process db.
             * Every time when shared info is taken, the record is erased in
             * kernel side.
             */
            gcmkVERIFY_OK(
                gckKERNEL_RemoveProcessDB(Kernel,
                                          pid,
                                          gcvDB_SHARED_INFO,
                                          gcmINT2PTR(dataId)));
            /* Free existed data. */
            gcmkVERIFY_OK(
                gckOS_FreeMemory(Kernel->os, record.physical));
        }
        break;

    case gcvHAL_SET_SHARED_INFO:
        {
            gctUINT32 dataId = Interface->u.SetSharedInfo.dataId;
            gctPOINTER data  = Interface->u.SetSharedInfo.data;
            gctUINT32 bytes  = Interface->u.SetSharedInfo.bytes;
            gctPOINTER memory = gcvNULL;
            gcsDATABASE_RECORD record;

            if (gcmIS_SUCCESS(gckKERNEL_FindProcessDB(Kernel,
                                                     processID,
                                                     0,
                                                     gcvDB_SHARED_INFO,
                                                     gcmINT2PTR(dataId),
                                                     &record)))
            {
                /* Find a record with the same id. */
                if (bytes != record.bytes)
                {
                    /* Remove from process db. */
                    gcmkVERIFY_OK(
                        gckKERNEL_RemoveProcessDB(Kernel,
                                                  processID,
                                                  gcvDB_SHARED_INFO,
                                                  gcmINT2PTR(dataId)));

                    /* Free existed data. */
                    gcmkVERIFY_OK(
                        gckOS_FreeMemory(Kernel->os, record.physical));
                }
                else
                {
                    /* Re-use allocated memory. */
                    memory = record.physical;
                }
            }

            if ((data == gcvNULL) || (bytes == 0))
            {
                /* Nothing to record. */
                break;
            }

            if (bytes > 1024)
            {
                /* Limite data size. */
                gcmkONERROR(gcvSTATUS_TOO_COMPLEX);
            }

            if (memory == gcvNULL)
            {
                /* Allocate memory for holding shared data. */
                gcmkONERROR(
                    gckOS_AllocateMemory(Kernel->os, bytes, &memory));

                /* Add to process db. */
                status = gckKERNEL_AddProcessDB(Kernel,
                                                processID,
                                                gcvDB_SHARED_INFO,
                                                gcmINT2PTR(dataId),
                                                memory,
                                                bytes);

                if (gcmIS_ERROR(status))
                {
                    /* Failed to add process db. Free allocated memory. */
                    gcmkVERIFY_OK(gckOS_FreeMemory(Kernel->os, memory));
                    break;
                }
            }

            /* Copy shared data to kernel memory. */
            gcmkONERROR(
                gckOS_CopyFromUserData(Kernel->os,
                                       memory,
                                       data,
                                       bytes));
        }
        break;

    case gcvHAL_SET_FSCALE_VALUE:
#if gcdENABLE_FSCALE_VAL_ADJUST
        status = gckHARDWARE_SetFscaleValue(Kernel->hardware,
                                            Interface->u.SetFscaleValue.value);
#else
        status = gcvSTATUS_NOT_SUPPORTED;
#endif
        break;
    case gcvHAL_GET_FSCALE_VALUE:
#if gcdENABLE_FSCALE_VAL_ADJUST
        status = gckHARDWARE_GetFscaleValue(Kernel->hardware,
                                            &Interface->u.GetFscaleValue.value,
                                            &Interface->u.GetFscaleValue.minValue,
                                            &Interface->u.GetFscaleValue.maxValue);
#else
        status = gcvSTATUS_NOT_SUPPORTED;
#endif
        break;

    case gcvHAL_QUERY_RESET_TIME_STAMP:
#if gcdENABLE_RECOVERY
        Interface->u.QueryResetTimeStamp.timeStamp = Kernel->resetTimeStamp;
#else
        Interface->u.QueryResetTimeStamp.timeStamp = 0;
#endif
        break;

#if gcdANDROID_NATIVE_FENCE_SYNC
    case gcvHAL_SYNC_POINT:
        {
            gctSYNC_POINT syncPoint;

            switch (Interface->u.SyncPoint.command)
            {
            case gcvSYNC_POINT_CREATE:
                gcmkONERROR(gckOS_CreateSyncPoint(Kernel->os, &syncPoint));

                Interface->u.SyncPoint.syncPoint = gcmPTR_TO_UINT64(syncPoint);

                gcmkVERIFY_OK(
                    gckKERNEL_AddProcessDB(Kernel,
                                           processID, gcvDB_SYNC_POINT,
                                           syncPoint,
                                           gcvNULL,
                                           0));
                break;

            case gcvSYNC_POINT_DESTROY:
                syncPoint = gcmUINT64_TO_PTR(Interface->u.SyncPoint.syncPoint);

                gcmkONERROR(gckOS_DestroySyncPoint(Kernel->os, syncPoint));

                gcmkVERIFY_OK(
                    gckKERNEL_RemoveProcessDB(Kernel,
                                              processID, gcvDB_SYNC_POINT,
                                              syncPoint));
                break;

            default:
                gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
                break;
            }
        }
        break;

    case gcvHAL_CREATE_NATIVE_FENCE:
        {
            gctINT fenceFD;
            gctSYNC_POINT syncPoint =
                gcmUINT64_TO_PTR(Interface->u.CreateNativeFence.syncPoint);

            gcmkONERROR(
                gckOS_CreateNativeFence(Kernel->os,
                                        Kernel->timeline,
                                        syncPoint,
                                        &fenceFD));

            Interface->u.CreateNativeFence.fenceFD = fenceFD;
        }
        break;
#endif

    default:
        /* Invalid command. */
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

OnError:
    /* Save status. */
    Interface->status = status;

    if (gcmIS_ERROR(status))
    {
        if (locked)
        {
            /* Roll back the lock. */
            gcmkVERIFY_OK(
                gckVIDMEM_Unlock(Kernel,
                                 gcmUINT64_TO_PTR(Interface->u.LockVideoMemory.node),
                                 gcvSURF_TYPE_UNKNOWN,
                                 &asynchronous));

            if (gcvTRUE == asynchronous)
            {
                /* Bottom Half */
                gcmkVERIFY_OK(
                    gckVIDMEM_Unlock(Kernel,
                                     gcmUINT64_TO_PTR(Interface->u.LockVideoMemory.node),
                                     gcvSURF_TYPE_UNKNOWN,
                                     gcvNULL));
            }
        }
    }

#if QNX_SINGLE_THREADED_DEBUGGING
    gckOS_ReleaseMutex(Kernel->os, Kernel->debugMutex);
#endif

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**  gckKERNEL_AttachProcess
**
**  Attach or detach a process.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctBOOL Attach
**          gcvTRUE if a new process gets attached or gcFALSE when a process
**          gets detatched.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckKERNEL_AttachProcess(
    IN gckKERNEL Kernel,
    IN gctBOOL Attach
    )
{
    gceSTATUS status;
    gctUINT32 processID;

    gcmkHEADER_ARG("Kernel=0x%x Attach=%d", Kernel, Attach);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);

    /* Get current process ID. */
    gcmkONERROR(gckOS_GetProcessID(&processID));

    gcmkONERROR(gckKERNEL_AttachProcessEx(Kernel, Attach, processID));

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**  gckKERNEL_AttachProcessEx
**
**  Attach or detach a process with the given PID. Can be paired with gckKERNEL_AttachProcess
**     provided the programmer is aware of the consequences.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctBOOL Attach
**          gcvTRUE if a new process gets attached or gcFALSE when a process
**          gets detatched.
**
**      gctUINT32 PID
**          PID of the process to attach or detach.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckKERNEL_AttachProcessEx(
    IN gckKERNEL Kernel,
    IN gctBOOL Attach,
    IN gctUINT32 PID
    )
{
    gceSTATUS status;
    gctINT32 old;

    gcmkHEADER_ARG("Kernel=0x%x Attach=%d PID=%d", Kernel, Attach, PID);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);

    if (Attach)
    {
        /* Increment the number of clients attached. */
        gcmkONERROR(
            gckOS_AtomIncrement(Kernel->os, Kernel->atomClients, &old));

        if (old == 0)
        {
#if gcdENABLE_VG
            if (Kernel->vg == gcvNULL)
#endif
            {
                gcmkONERROR(gckOS_Broadcast(Kernel->os,
                                            Kernel->hardware,
                                            gcvBROADCAST_FIRST_PROCESS));
            }
        }

        if (Kernel->dbCreated)
        {
            /* Create the process database. */
            gcmkONERROR(gckKERNEL_CreateProcessDB(Kernel, PID));
        }
    }
    else
    {
        if (Kernel->dbCreated)
        {
            /* Clean up the process database. */
            gcmkONERROR(gckKERNEL_DestroyProcessDB(Kernel, PID));

            /* Save the last know process ID. */
            Kernel->db->lastProcessID = PID;
        }

#if gcdENABLE_VG
        if (Kernel->vg == gcvNULL)
#endif
        {
            status = gckEVENT_Submit(Kernel->eventObj, gcvTRUE, gcvFALSE);

            if (status == gcvSTATUS_INTERRUPTED && Kernel->eventObj->submitTimer)
            {
                gcmkONERROR(gckOS_StartTimer(Kernel->os,
                                             Kernel->eventObj->submitTimer,
                                             1));
            }
            else
            {
                gcmkONERROR(status);
            }
        }

        /* Decrement the number of clients attached. */
        gcmkONERROR(
            gckOS_AtomDecrement(Kernel->os, Kernel->atomClients, &old));

        if (old == 1)
        {
#if gcdENABLE_VG
            if (Kernel->vg == gcvNULL)
#endif
            {
                /* Last client detached, switch to SUSPEND power state. */
                gcmkONERROR(gckOS_Broadcast(Kernel->os,
                                            Kernel->hardware,
                                            gcvBROADCAST_LAST_PROCESS));
            }

            /* Flush the debug cache. */
            gcmkDEBUGFLUSH(~0U);
        }
    }

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

#if gcdSECURE_USER
gceSTATUS
gckKERNEL_MapLogicalToPhysical(
    IN gckKERNEL Kernel,
    IN gcskSECURE_CACHE_PTR Cache,
    IN OUT gctPOINTER * Data
    )
{
    gceSTATUS status;
    static gctBOOL baseAddressValid = gcvFALSE;
    static gctUINT32 baseAddress;
    gctBOOL needBase;
    gcskLOGICAL_CACHE_PTR slot;

    gcmkHEADER_ARG("Kernel=0x%x Cache=0x%x *Data=0x%x",
                   Kernel, Cache, gcmOPT_POINTER(Data));

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);

    if (!baseAddressValid)
    {
        /* Get base address. */
        gcmkONERROR(gckHARDWARE_GetBaseAddress(Kernel->hardware, &baseAddress));

        baseAddressValid = gcvTRUE;
    }

    /* Does this state load need a base address? */
    gcmkONERROR(gckHARDWARE_NeedBaseAddress(Kernel->hardware,
                                            ((gctUINT32_PTR) Data)[-1],
                                            &needBase));

#if gcdSECURE_CACHE_METHOD == gcdSECURE_CACHE_LRU
    {
        gcskLOGICAL_CACHE_PTR next;
        gctINT i;

        /* Walk all used cache slots. */
        for (i = 1, slot = Cache->cache[0].next, next = gcvNULL;
             (i <= gcdSECURE_CACHE_SLOTS) && (slot->logical != gcvNULL);
             ++i, slot = slot->next
        )
        {
            if (slot->logical == *Data)
            {
                /* Bail out. */
                next = slot;
                break;
            }
        }

        /* See if we had a miss. */
        if (next == gcvNULL)
        {
            /* Use the tail of the cache. */
            slot = Cache->cache[0].prev;

            /* Initialize the cache line. */
            slot->logical = *Data;

            /* Map the logical address to a DMA address. */
            gcmkONERROR(
                gckOS_GetPhysicalAddress(Kernel->os, *Data, &slot->dma));
        }

        /* Move slot to head of list. */
        if (slot != Cache->cache[0].next)
        {
            /* Unlink. */
            slot->prev->next = slot->next;
            slot->next->prev = slot->prev;

            /* Move to head of chain. */
            slot->prev       = &Cache->cache[0];
            slot->next       = Cache->cache[0].next;
            slot->prev->next = slot;
            slot->next->prev = slot;
        }
    }
#elif gcdSECURE_CACHE_METHOD == gcdSECURE_CACHE_LINEAR
    {
        gctINT i;
        gcskLOGICAL_CACHE_PTR next = gcvNULL;
        gcskLOGICAL_CACHE_PTR oldestSlot = gcvNULL;
        slot = gcvNULL;

        if (Cache->cacheIndex != gcvNULL)
        {
            /* Walk the cache forwards. */
            for (i = 1, slot = Cache->cacheIndex;
                 (i <= gcdSECURE_CACHE_SLOTS) && (slot->logical != gcvNULL);
                 ++i, slot = slot->next)
            {
                if (slot->logical == *Data)
                {
                    /* Bail out. */
                    next = slot;
                    break;
                }

                /* Determine age of this slot. */
                if ((oldestSlot       == gcvNULL)
                ||  (oldestSlot->stamp > slot->stamp)
                )
                {
                    oldestSlot = slot;
                }
            }

            if (next == gcvNULL)
            {
                /* Walk the cache backwards. */
                for (slot = Cache->cacheIndex->prev;
                     (i <= gcdSECURE_CACHE_SLOTS) && (slot->logical != gcvNULL);
                     ++i, slot = slot->prev)
                {
                    if (slot->logical == *Data)
                    {
                        /* Bail out. */
                        next = slot;
                        break;
                    }

                    /* Determine age of this slot. */
                    if ((oldestSlot       == gcvNULL)
                    ||  (oldestSlot->stamp > slot->stamp)
                    )
                    {
                        oldestSlot = slot;
                    }
                }
            }
        }

        /* See if we had a miss. */
        if (next == gcvNULL)
        {
            if (Cache->cacheFree != 0)
            {
                slot = &Cache->cache[Cache->cacheFree];
                gcmkASSERT(slot->logical == gcvNULL);

                ++ Cache->cacheFree;
                if (Cache->cacheFree >= gcmCOUNTOF(Cache->cache))
                {
                    Cache->cacheFree = 0;
                }
            }
            else
            {
                /* Use the oldest cache slot. */
                gcmkASSERT(oldestSlot != gcvNULL);
                slot = oldestSlot;

                /* Unlink from the chain. */
                slot->prev->next = slot->next;
                slot->next->prev = slot->prev;

                /* Append to the end. */
                slot->prev       = Cache->cache[0].prev;
                slot->next       = &Cache->cache[0];
                slot->prev->next = slot;
                slot->next->prev = slot;
            }

            /* Initialize the cache line. */
            slot->logical = *Data;

            /* Map the logical address to a DMA address. */
            gcmkONERROR(
                gckOS_GetPhysicalAddress(Kernel->os, *Data, &slot->dma));
        }

        /* Save time stamp. */
        slot->stamp = ++ Cache->cacheStamp;

        /* Save current slot for next lookup. */
        Cache->cacheIndex = slot;
    }
#elif gcdSECURE_CACHE_METHOD == gcdSECURE_CACHE_HASH
    {
        gctINT i;
        gctUINT32 data = gcmPTR2INT(*Data);
        gctUINT32 key, index;
        gcskLOGICAL_CACHE_PTR hash;

        /* Generate a hash key. */
        key   = (data >> 24) + (data >> 16) + (data >> 8) + data;
        index = key % gcmCOUNTOF(Cache->hash);

        /* Get the hash entry. */
        hash = &Cache->hash[index];

        for (slot = hash->nextHash, i = 0;
             (slot != gcvNULL) && (i < gcdSECURE_CACHE_SLOTS);
             slot = slot->nextHash, ++i
        )
        {
            if (slot->logical == (*Data))
            {
                break;
            }
        }

        if (slot == gcvNULL)
        {
            /* Grab from the tail of the cache. */
            slot = Cache->cache[0].prev;

            /* Unlink slot from any hash table it is part of. */
            if (slot->prevHash != gcvNULL)
            {
                slot->prevHash->nextHash = slot->nextHash;
            }
            if (slot->nextHash != gcvNULL)
            {
                slot->nextHash->prevHash = slot->prevHash;
            }

            /* Initialize the cache line. */
            slot->logical = *Data;

            /* Map the logical address to a DMA address. */
            gcmkONERROR(
                gckOS_GetPhysicalAddress(Kernel->os, *Data, &slot->dma));

            if (hash->nextHash != gcvNULL)
            {
                gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_KERNEL,
                               "Hash Collision: logical=0x%x key=0x%08x",
                               *Data, key);
            }

            /* Insert the slot at the head of the hash list. */
            slot->nextHash     = hash->nextHash;
            if (slot->nextHash != gcvNULL)
            {
                slot->nextHash->prevHash = slot;
            }
            slot->prevHash     = hash;
            hash->nextHash     = slot;
        }

        /* Move slot to head of list. */
        if (slot != Cache->cache[0].next)
        {
            /* Unlink. */
            slot->prev->next = slot->next;
            slot->next->prev = slot->prev;

            /* Move to head of chain. */
            slot->prev       = &Cache->cache[0];
            slot->next       = Cache->cache[0].next;
            slot->prev->next = slot;
            slot->next->prev = slot;
        }
    }
#elif gcdSECURE_CACHE_METHOD == gcdSECURE_CACHE_TABLE
    {
        gctUINT32 index = (gcmPTR2INT(*Data) % gcdSECURE_CACHE_SLOTS) + 1;

        /* Get cache slot. */
        slot = &Cache->cache[index];

        /* Check for cache miss. */
        if (slot->logical != *Data)
        {
            /* Initialize the cache line. */
            slot->logical = *Data;

            /* Map the logical address to a DMA address. */
            gcmkONERROR(
                gckOS_GetPhysicalAddress(Kernel->os, *Data, &slot->dma));
        }
    }
#endif

    /* Return DMA address. */
    *Data = gcmINT2PTR(slot->dma + (needBase ? baseAddress : 0));

    /* Success. */
    gcmkFOOTER_ARG("*Data=0x%08x", *Data);
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckKERNEL_FlushTranslationCache(
    IN gckKERNEL Kernel,
    IN gcskSECURE_CACHE_PTR Cache,
    IN gctPOINTER Logical,
    IN gctSIZE_T Bytes
    )
{
    gctINT i;
    gcskLOGICAL_CACHE_PTR slot;
    gctUINT8_PTR ptr;

    gcmkHEADER_ARG("Kernel=0x%x Cache=0x%x Logical=0x%x Bytes=%lu",
                   Kernel, Cache, Logical, Bytes);

    /* Do we need to flush the entire cache? */
    if (Logical == gcvNULL)
    {
        /* Clear all cache slots. */
        for (i = 1; i <= gcdSECURE_CACHE_SLOTS; ++i)
        {
            Cache->cache[i].logical  = gcvNULL;

#if gcdSECURE_CACHE_METHOD == gcdSECURE_CACHE_HASH
            Cache->cache[i].nextHash = gcvNULL;
            Cache->cache[i].prevHash = gcvNULL;
#endif
}

#if gcdSECURE_CACHE_METHOD == gcdSECURE_CACHE_HASH
        /* Zero the hash table. */
        for (i = 0; i < gcmCOUNTOF(Cache->hash); ++i)
        {
            Cache->hash[i].nextHash = gcvNULL;
        }
#endif

        /* Reset the cache functionality. */
        Cache->cacheIndex = gcvNULL;
        Cache->cacheFree  = 1;
        Cache->cacheStamp = 0;
    }

    else
    {
        gctUINT8_PTR low  = (gctUINT8_PTR) Logical;
        gctUINT8_PTR high = low + Bytes;

#if gcdSECURE_CACHE_METHOD == gcdSECURE_CACHE_LRU
        gcskLOGICAL_CACHE_PTR next;

        /* Walk all used cache slots. */
        for (i = 1, slot = Cache->cache[0].next;
             (i <= gcdSECURE_CACHE_SLOTS) && (slot->logical != gcvNULL);
             ++i, slot = next
        )
        {
            /* Save pointer to next slot. */
            next = slot->next;

            /* Test if this slot falls within the range to flush. */
            ptr = (gctUINT8_PTR) slot->logical;
            if ((ptr >= low) && (ptr < high))
            {
                /* Unlink slot. */
                slot->prev->next = slot->next;
                slot->next->prev = slot->prev;

                /* Append slot to tail of cache. */
                slot->prev       = Cache->cache[0].prev;
                slot->next       = &Cache->cache[0];
                slot->prev->next = slot;
                slot->next->prev = slot;

                /* Mark slot as empty. */
                slot->logical = gcvNULL;
            }
        }

#elif gcdSECURE_CACHE_METHOD == gcdSECURE_CACHE_LINEAR
        gcskLOGICAL_CACHE_PTR next;

        for (i = 1, slot = Cache->cache[0].next;
             (i <= gcdSECURE_CACHE_SLOTS) && (slot->logical != gcvNULL);
             ++i, slot = next)
        {
            /* Save pointer to next slot. */
            next = slot->next;

            /* Test if this slot falls within the range to flush. */
            ptr = (gctUINT8_PTR) slot->logical;
            if ((ptr >= low) && (ptr < high))
            {
                /* Test if this slot is the current slot. */
                if (slot == Cache->cacheIndex)
                {
                    /* Move to next or previous slot. */
                    Cache->cacheIndex = (slot->next->logical != gcvNULL)
                                      ? slot->next
                                      : (slot->prev->logical != gcvNULL)
                                      ? slot->prev
                                      : gcvNULL;
                }

                /* Unlink slot from cache. */
                slot->prev->next = slot->next;
                slot->next->prev = slot->prev;

                /* Insert slot to head of cache. */
                slot->prev       = &Cache->cache[0];
                slot->next       = Cache->cache[0].next;
                slot->prev->next = slot;
                slot->next->prev = slot;

                /* Mark slot as empty. */
                slot->logical = gcvNULL;
                slot->stamp   = 0;
            }
        }

#elif gcdSECURE_CACHE_METHOD == gcdSECURE_CACHE_HASH
        gctINT j;
        gcskLOGICAL_CACHE_PTR hash, next;

        /* Walk all hash tables. */
        for (i = 0, hash = Cache->hash;
             i < gcmCOUNTOF(Cache->hash);
             ++i, ++hash)
        {
            /* Walk all slots in the hash. */
            for (j = 0, slot = hash->nextHash;
                 (j < gcdSECURE_CACHE_SLOTS) && (slot != gcvNULL);
                 ++j, slot = next)
            {
                /* Save pointer to next slot. */
                next = slot->next;

                /* Test if this slot falls within the range to flush. */
                ptr = (gctUINT8_PTR) slot->logical;
                if ((ptr >= low) && (ptr < high))
                {
                    /* Unlink slot from hash table. */
                    if (slot->prevHash == hash)
                    {
                        hash->nextHash = slot->nextHash;
                    }
                    else
                    {
                        slot->prevHash->nextHash = slot->nextHash;
                    }

                    if (slot->nextHash != gcvNULL)
                    {
                        slot->nextHash->prevHash = slot->prevHash;
                    }

                    /* Unlink slot from cache. */
                    slot->prev->next = slot->next;
                    slot->next->prev = slot->prev;

                    /* Append slot to tail of cache. */
                    slot->prev       = Cache->cache[0].prev;
                    slot->next       = &Cache->cache[0];
                    slot->prev->next = slot;
                    slot->next->prev = slot;

                    /* Mark slot as empty. */
                    slot->logical  = gcvNULL;
                    slot->prevHash = gcvNULL;
                    slot->nextHash = gcvNULL;
                }
            }
        }

#elif gcdSECURE_CACHE_METHOD == gcdSECURE_CACHE_TABLE
        gctUINT32 index;

        /* Loop while inside the range. */
        for (i = 1; (low < high) && (i <= gcdSECURE_CACHE_SLOTS); ++i)
        {
            /* Get index into cache for this range. */
            index = (gcmPTR2INT(low) % gcdSECURE_CACHE_SLOTS) + 1;
            slot  = &Cache->cache[index];

            /* Test if this slot falls within the range to flush. */
            ptr = (gctUINT8_PTR) slot->logical;
            if ((ptr >= low) && (ptr < high))
            {
                /* Remove entry from cache. */
                slot->logical = gcvNULL;
            }

            /* Next block. */
            low += gcdSECURE_CACHE_SLOTS;
        }
#endif
    }

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}
#endif

/*******************************************************************************
**
**  gckKERNEL_Recovery
**
**  Try to recover the GPU from a fatal error.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckKERNEL_Recovery(
    IN gckKERNEL Kernel
    )
{
#if gcdENABLE_RECOVERY
#define gcdEVENT_MASK 0x3FFFFFFF
    gceSTATUS status;
    gckEVENT eventObj;
    gckHARDWARE hardware;
#if gcdSECURE_USER
    gctUINT32 processID;
    gcskSECURE_CACHE_PTR cache;
#endif
    gctUINT32 oldValue;
    gcmkHEADER_ARG("Kernel=0x%x", Kernel);

    /* Validate the arguemnts. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);

    /* Grab gckEVENT object. */
    eventObj = Kernel->eventObj;
    gcmkVERIFY_OBJECT(eventObj, gcvOBJ_EVENT);

    /* Grab gckHARDWARE object. */
    hardware = Kernel->hardware;
    gcmkVERIFY_OBJECT(hardware, gcvOBJ_HARDWARE);

#if gcdSECURE_USER
    /* Flush the secure mapping cache. */
    gcmkONERROR(gckOS_GetProcessID(&processID));
    gcmkONERROR(gckKERNEL_GetProcessDBCache(Kernel, processID, &cache));
    gcmkONERROR(gckKERNEL_FlushTranslationCache(Kernel, cache, gcvNULL, 0));
#endif

    gcmkONERROR(
        gckOS_AtomicExchange(Kernel->os, Kernel->resetAtom, 1, &oldValue));

    if (oldValue)
    {
        /* Some one else will recovery GPU. */
        return gcvSTATUS_OK;
    }

    gcmkPRINT("[galcore]: GPU[%d] hang, automatic recovery.", Kernel->core);

    /* Start a timer to clear reset flag, before timer is expired,
    ** other recovery request is ignored. */
    gcmkVERIFY_OK(
        gckOS_StartTimer(Kernel->os,
                         Kernel->resetFlagClearTimer,
                         gcdGPU_TIMEOUT - 500));


    /* Try issuing a soft reset for the GPU. */
    status = gckHARDWARE_Reset(hardware);
    if (status == gcvSTATUS_NOT_SUPPORTED)
    {
        /* Switch to OFF power.  The next submit should return the GPU to ON
        ** state. */
        gcmkONERROR(
            gckHARDWARE_SetPowerManagementState(hardware,
                                                gcvPOWER_OFF_RECOVERY));
    }
    else
    {
        /* Bail out on reset error. */
        gcmkONERROR(status);
    }

    /* Handle all outstanding events now. */
#if gcdSMP
    gcmkONERROR(gckOS_AtomSet(Kernel->os, eventObj->pending, gcdEVENT_MASK));
#else
    eventObj->pending = gcdEVENT_MASK;
#endif
    gcmkONERROR(gckEVENT_Notify(eventObj, 1));

    /* Again in case more events got submitted. */
#if gcdSMP
    gcmkONERROR(gckOS_AtomSet(Kernel->os, eventObj->pending, gcdEVENT_MASK));
#else
    eventObj->pending = gcdEVENT_MASK;
#endif
    gcmkONERROR(gckEVENT_Notify(eventObj, 2));

    Kernel->resetTimeStamp++;

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
#else
    return gcvSTATUS_OK;
#endif
}

/*******************************************************************************
**
**  gckKERNEL_OpenUserData
**
**  Get access to the user data.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctBOOL NeedCopy
**          The flag indicating whether or not the data should be copied.
**
**      gctPOINTER StaticStorage
**          Pointer to the kernel storage where the data is to be copied if
**          NeedCopy is gcvTRUE.
**
**      gctPOINTER UserPointer
**          User pointer to the data.
**
**      gctSIZE_T Size
**          Size of the data.
**
**  OUTPUT:
**
**      gctPOINTER * KernelPointer
**          Pointer to the kernel pointer that will be pointing to the data.
*/
gceSTATUS
gckKERNEL_OpenUserData(
    IN gckKERNEL Kernel,
    IN gctBOOL NeedCopy,
    IN gctPOINTER StaticStorage,
    IN gctPOINTER UserPointer,
    IN gctSIZE_T Size,
    OUT gctPOINTER * KernelPointer
    )
{
    gceSTATUS status;

    gcmkHEADER_ARG(
        "Kernel=0x%08X NeedCopy=%d StaticStorage=0x%08X "
        "UserPointer=0x%08X Size=%lu KernelPointer=0x%08X",
        Kernel, NeedCopy, StaticStorage, UserPointer, Size, KernelPointer
        );

    /* Validate the arguemnts. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(!NeedCopy || (StaticStorage != gcvNULL));
    gcmkVERIFY_ARGUMENT(UserPointer != gcvNULL);
    gcmkVERIFY_ARGUMENT(KernelPointer != gcvNULL);
    gcmkVERIFY_ARGUMENT(Size > 0);

    if (NeedCopy)
    {
        /* Copy the user data to the static storage. */
        gcmkONERROR(gckOS_CopyFromUserData(
            Kernel->os, StaticStorage, UserPointer, Size
            ));

        /* Set the kernel pointer. */
        * KernelPointer = StaticStorage;
    }
    else
    {
        gctPOINTER pointer = gcvNULL;

        /* Map the user pointer. */
        gcmkONERROR(gckOS_MapUserPointer(
            Kernel->os, UserPointer, Size, &pointer
            ));

        /* Set the kernel pointer. */
        * KernelPointer = pointer;
    }

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckKERNEL_CloseUserData
**
**  Release resources associated with the user data connection opened by
**  gckKERNEL_OpenUserData.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctBOOL NeedCopy
**          The flag indicating whether or not the data should be copied.
**
**      gctBOOL FlushData
**          If gcvTRUE, the data is written back to the user.
**
**      gctPOINTER UserPointer
**          User pointer to the data.
**
**      gctSIZE_T Size
**          Size of the data.
**
**  OUTPUT:
**
**      gctPOINTER * KernelPointer
**          Kernel pointer to the data.
*/
gceSTATUS
gckKERNEL_CloseUserData(
    IN gckKERNEL Kernel,
    IN gctBOOL NeedCopy,
    IN gctBOOL FlushData,
    IN gctPOINTER UserPointer,
    IN gctSIZE_T Size,
    OUT gctPOINTER * KernelPointer
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gctPOINTER pointer;

    gcmkHEADER_ARG(
        "Kernel=0x%08X NeedCopy=%d FlushData=%d "
        "UserPointer=0x%08X Size=%lu KernelPointer=0x%08X",
        Kernel, NeedCopy, FlushData, UserPointer, Size, KernelPointer
        );

    /* Validate the arguemnts. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(UserPointer != gcvNULL);
    gcmkVERIFY_ARGUMENT(KernelPointer != gcvNULL);
    gcmkVERIFY_ARGUMENT(Size > 0);

    /* Get a shortcut to the kernel pointer. */
    pointer = * KernelPointer;

    if (pointer != gcvNULL)
    {
        if (NeedCopy)
        {
            if (FlushData)
            {
                gcmkONERROR(gckOS_CopyToUserData(
                    Kernel->os, * KernelPointer, UserPointer, Size
                    ));
            }
        }
        else
        {
            /* Unmap record from kernel memory. */
            gcmkONERROR(gckOS_UnmapUserPointer(
                Kernel->os,
                UserPointer,
                Size,
                * KernelPointer
                ));
        }

        /* Reset the kernel pointer. */
        * KernelPointer = gcvNULL;
    }

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

void
gckKERNEL_SetTimeOut(
    IN gckKERNEL Kernel,
    IN gctUINT32 timeOut
    )
{
    gcmkHEADER_ARG("Kernel=0x%x timeOut=%d", Kernel, timeOut);
#if gcdGPU_TIMEOUT
    Kernel->timeOut = timeOut;
#endif
    gcmkFOOTER_NO();
}

#if gcdVIRTUAL_COMMAND_BUFFER
gceSTATUS
gckKERNEL_AllocateVirtualCommandBuffer(
    IN gckKERNEL Kernel,
    IN gctBOOL InUserSpace,
    IN OUT gctSIZE_T * Bytes,
    OUT gctPHYS_ADDR * Physical,
    OUT gctPOINTER * Logical
    )
{
    gckOS os = Kernel->os;
    gceSTATUS status;
    gctPOINTER logical;
    gctSIZE_T pageCount;
    gctSIZE_T bytes = *Bytes;
    gckVIRTUAL_COMMAND_BUFFER_PTR buffer;

    gcmkHEADER_ARG("Os=0x%X InUserSpace=%d *Bytes=%lu",
                   os, InUserSpace, gcmOPT_VALUE(Bytes));

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Bytes != gcvNULL);
    gcmkVERIFY_ARGUMENT(*Bytes > 0);
    gcmkVERIFY_ARGUMENT(Physical != gcvNULL);
    gcmkVERIFY_ARGUMENT(Logical != gcvNULL);

    gcmkONERROR(gckOS_Allocate(os,
                               sizeof(gckVIRTUAL_COMMAND_BUFFER),
                               (gctPOINTER)&buffer));

    gcmkONERROR(gckOS_ZeroMemory(buffer, sizeof(gckVIRTUAL_COMMAND_BUFFER)));

    gcmkONERROR(gckOS_AllocatePagedMemoryEx(os,
                                            gcvFALSE,
                                            bytes,
                                            &buffer->physical));

    if (InUserSpace)
    {
        gcmkONERROR(gckOS_LockPages(os,
                                    buffer->physical,
                                    bytes,
                                    gcvFALSE,
                                    &logical,
                                    &pageCount));

        *Logical =
        buffer->userLogical = logical;
    }
    else
    {
        gcmkONERROR(
            gckOS_CreateKernelVirtualMapping(buffer->physical,
                                             &pageCount,
                                             &logical));
        *Logical =
        buffer->kernelLogical = logical;
    }

    buffer->pageCount = pageCount;
    buffer->kernel = Kernel;

    gcmkONERROR(gckOS_GetProcessID(&buffer->pid));

    gcmkONERROR(gckMMU_AllocatePages(Kernel->mmu,
                                     pageCount,
                                     &buffer->pageTable,
                                     &buffer->gpuAddress));

    gcmkONERROR(gckOS_MapPagesEx(os,
                                 Kernel->core,
                                 buffer->physical,
                                 pageCount,
                                 buffer->pageTable));

    gcmkONERROR(gckMMU_Flush(Kernel->mmu));

    *Physical = buffer;

    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_KERNEL,
                   "gpuAddress = %x pageCount = %d kernelLogical = %x userLogical=%x",
                   buffer->gpuAddress, buffer->pageCount,
                   buffer->kernelLogical, buffer->userLogical);

    gcmkVERIFY_OK(gckOS_AcquireMutex(os, Kernel->virtualBufferLock, gcvINFINITE));

    if (Kernel->virtualBufferHead == gcvNULL)
    {
        Kernel->virtualBufferHead =
        Kernel->virtualBufferTail = buffer;
    }
    else
    {
        buffer->prev = Kernel->virtualBufferTail;
        Kernel->virtualBufferTail->next = buffer;
        Kernel->virtualBufferTail = buffer;
    }

    gcmkVERIFY_OK(gckOS_ReleaseMutex(os, Kernel->virtualBufferLock));

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (buffer->gpuAddress)
    {
        gcmkVERIFY_OK(
            gckMMU_FreePages(Kernel->mmu, buffer->pageTable, buffer->pageCount));
    }

    if (buffer->userLogical)
    {
        gcmkVERIFY_OK(
            gckOS_UnlockPages(os, buffer->physical, bytes, buffer->userLogical));
    }

    if (buffer->kernelLogical)
    {
        gcmkVERIFY_OK(
            gckOS_DestroyKernelVirtualMapping(buffer->kernelLogical));
    }

    if (buffer->physical)
    {
        gcmkVERIFY_OK(gckOS_FreePagedMemory(os, buffer->physical, bytes));
    }

    gcmkVERIFY_OK(gckOS_Free(os, buffer));

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckKERNEL_DestroyVirtualCommandBuffer(
    IN gckKERNEL Kernel,
    IN gctSIZE_T Bytes,
    IN gctPHYS_ADDR Physical,
    IN gctPOINTER Logical
    )
{
    gckOS os;
    gckKERNEL kernel;
    gckVIRTUAL_COMMAND_BUFFER_PTR buffer = (gckVIRTUAL_COMMAND_BUFFER_PTR)Physical;

    gcmkHEADER();
    gcmkVERIFY_ARGUMENT(buffer != gcvNULL);

    kernel = buffer->kernel;
    os = kernel->os;

    if (buffer->userLogical)
    {
        gcmkVERIFY_OK(gckOS_UnlockPages(os, buffer->physical, Bytes, Logical));
    }
    else
    {
        gcmkVERIFY_OK(gckOS_DestroyKernelVirtualMapping(Logical));
    }

    gcmkVERIFY_OK(
        gckMMU_FreePages(kernel->mmu, buffer->pageTable, buffer->pageCount));

    gcmkVERIFY_OK(gckOS_FreePagedMemory(os, buffer->physical, Bytes));

    gcmkVERIFY_OK(gckOS_AcquireMutex(os, kernel->virtualBufferLock, gcvINFINITE));

    if (buffer == kernel->virtualBufferHead)
    {
        if ((kernel->virtualBufferHead = buffer->next) == gcvNULL)
        {
            kernel->virtualBufferTail = gcvNULL;
        }
    }
    else
    {
        buffer->prev->next = buffer->next;

        if (buffer == kernel->virtualBufferTail)
        {
            kernel->virtualBufferTail = buffer->prev;
        }
        else
        {
            buffer->next->prev = buffer->prev;
        }
    }

    gcmkVERIFY_OK(gckOS_ReleaseMutex(os, kernel->virtualBufferLock));

    gcmkVERIFY_OK(gckOS_Free(os, buffer));

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

gceSTATUS
gckKERNEL_GetGPUAddress(
    IN gckKERNEL Kernel,
    IN gctPOINTER Logical,
    OUT gctUINT32 * Address
    )
{
    gceSTATUS status;
    gckVIRTUAL_COMMAND_BUFFER_PTR buffer;
    gctPOINTER start;
    gctINT pid;

    gcmkHEADER_ARG("Logical = %x", Logical);

    gckOS_GetProcessID(&pid);

    status = gcvSTATUS_INVALID_ADDRESS;

    gcmkVERIFY_OK(gckOS_AcquireMutex(Kernel->os, Kernel->virtualBufferLock, gcvINFINITE));

    /* Walk all command buffer. */
    for (buffer = Kernel->virtualBufferHead; buffer != gcvNULL; buffer = buffer->next)
    {
        if (buffer->userLogical)
        {
            start = buffer->userLogical;
        }
        else
        {
            start = buffer->kernelLogical;
        }

        if (Logical >= start
        && (Logical < (start + buffer->pageCount * 4096))
        && pid == buffer->pid
        )
        {
            * Address = buffer->gpuAddress + (Logical - start);
            status = gcvSTATUS_OK;
            break;
        }
    }

    gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, Kernel->virtualBufferLock));

    gcmkFOOTER_NO();
    return status;
}

gceSTATUS
gckKERNEL_QueryGPUAddress(
    IN gckKERNEL Kernel,
    IN gctUINT32 GpuAddress,
    OUT gckVIRTUAL_COMMAND_BUFFER_PTR * Buffer
    )
{
    gckVIRTUAL_COMMAND_BUFFER_PTR buffer;
    gctUINT32 start;
    gceSTATUS status = gcvSTATUS_NOT_SUPPORTED;

    gcmkVERIFY_OK(gckOS_AcquireMutex(Kernel->os, Kernel->virtualBufferLock, gcvINFINITE));

    /* Walk all command buffers. */
    for (buffer = Kernel->virtualBufferHead; buffer != gcvNULL; buffer = buffer->next)
    {
        start = (gctUINT32)buffer->gpuAddress;

        if (GpuAddress >= start && GpuAddress < (start + buffer->pageCount * 4096))
        {
            /* Find a range matched. */
            *Buffer = buffer;
            status = gcvSTATUS_OK;
            break;
        }
    }

    gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, Kernel->virtualBufferLock));

    return status;
}
#endif

#if gcdLINK_QUEUE_SIZE
static void
gckLINKQUEUE_Dequeue(
    IN gckLINKQUEUE LinkQueue
    )
{
    gcmkASSERT(LinkQueue->count == gcdLINK_QUEUE_SIZE);

    LinkQueue->count--;
    LinkQueue->front = (LinkQueue->front + 1) % gcdLINK_QUEUE_SIZE;
}

void
gckLINKQUEUE_Enqueue(
    IN gckLINKQUEUE LinkQueue,
    IN gctUINT32 start,
    IN gctUINT32 end
    )
{
    if (LinkQueue->count == gcdLINK_QUEUE_SIZE)
    {
        gckLINKQUEUE_Dequeue(LinkQueue);
    }

    gcmkASSERT(LinkQueue->count < gcdLINK_QUEUE_SIZE);

    LinkQueue->count++;

    LinkQueue->data[LinkQueue->rear].start = start;
    LinkQueue->data[LinkQueue->rear].end = end;

    gcmkVERIFY_OK(
        gckOS_GetProcessID(&LinkQueue->data[LinkQueue->rear].pid));

    LinkQueue->rear = (LinkQueue->rear + 1) % gcdLINK_QUEUE_SIZE;
}

void
gckLINKQUEUE_GetData(
    IN gckLINKQUEUE LinkQueue,
    IN gctUINT32 Index,
    OUT gckLINKDATA * Data
    )
{
    gcmkASSERT(Index >= 0 && Index < gcdLINK_QUEUE_SIZE);

    *Data = &LinkQueue->data[(Index + LinkQueue->front) % gcdLINK_QUEUE_SIZE];
}
#endif

/******************************************************************************\
*************************** Pointer - ID translation ***************************
\******************************************************************************/
#define gcdID_TABLE_LENGTH 1024
typedef struct _gcsINTEGERDB * gckINTEGERDB;
typedef struct _gcsINTEGERDB
{
    gckOS                       os;
    gctPOINTER*                 table;
    gctPOINTER                  mutex;
    gctUINT32                   tableLen;
    gctUINT32                   currentID;
    gctUINT32                   unused;
}
gcsINTEGERDB;

gceSTATUS
gckKERNEL_CreateIntegerDatabase(
    IN gckKERNEL Kernel,
    OUT gctPOINTER * Database
    )
{
    gceSTATUS status;
    gckINTEGERDB database = gcvNULL;

    gcmkHEADER_ARG("Kernel=0x%08X Datbase=0x%08X", Kernel, Database);

    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(Database != gcvNULL);

    /* Allocate a database. */
    gcmkONERROR(gckOS_Allocate(
        Kernel->os, gcmSIZEOF(gcsINTEGERDB), (gctPOINTER *)&database));

    gckOS_ZeroMemory(database, gcmSIZEOF(gcsINTEGERDB));

    /* Allocate a pointer table. */
    gcmkONERROR(gckOS_Allocate(
        Kernel->os, gcmSIZEOF(gctPOINTER) * gcdID_TABLE_LENGTH, (gctPOINTER *)&database->table));

    gckOS_ZeroMemory(database->table, gcmSIZEOF(gctPOINTER) * gcdID_TABLE_LENGTH);

    /* Allocate a database mutex. */
    gcmkONERROR(gckOS_CreateMutex(Kernel->os, &database->mutex));

    /* Initialize. */
    database->currentID = 0;
    database->unused = gcdID_TABLE_LENGTH;
    database->os = Kernel->os;
    database->tableLen = gcdID_TABLE_LENGTH;

    *Database = database;

    gcmkFOOTER_ARG("*Database=0x%08X", *Database);
    return gcvSTATUS_OK;

OnError:
    /* Rollback. */
    if (database)
    {
        if (database->table)
        {
            gcmkOS_SAFE_FREE(Kernel->os, database->table);
        }

        gcmkOS_SAFE_FREE(Kernel->os, database);
    }

    gcmkFOOTER();
    return status;
}

gceSTATUS
gckKERNEL_DestroyIntegerDatabase(
    IN gckKERNEL Kernel,
    IN gctPOINTER Database
    )
{
    gckINTEGERDB database = Database;

    gcmkHEADER_ARG("Kernel=0x%08X Datbase=0x%08X", Kernel, Database);

    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(Database != gcvNULL);

    /* Destroy pointer table. */
    gcmkOS_SAFE_FREE(Kernel->os, database->table);

    /* Destroy database mutex. */
    gcmkVERIFY_OK(gckOS_DeleteMutex(Kernel->os, database->mutex));

    /* Destroy database. */
    gcmkOS_SAFE_FREE(Kernel->os, database);

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

gceSTATUS
gckKERNEL_AllocateIntegerId(
    IN gctPOINTER Database,
    IN gctPOINTER Pointer,
    OUT gctUINT32 * Id
    )
{
    gceSTATUS status;
    gckINTEGERDB database = Database;
    gctUINT32 i, unused, currentID, tableLen;
    gctPOINTER * table;
    gckOS os = database->os;
    gctBOOL acquired = gcvFALSE;

    gcmkHEADER_ARG("Database=0x%08X Pointer=0x%08X", Database, Pointer);

    gcmkVERIFY_ARGUMENT(Id != gcvNULL);

    gcmkVERIFY_OK(gckOS_AcquireMutex(os, database->mutex, gcvINFINITE));
    acquired = gcvTRUE;

    if (database->unused < 1)
    {
        /* Extend table. */
        gcmkONERROR(
            gckOS_Allocate(os,
                           gcmSIZEOF(gctPOINTER) * (database->tableLen + gcdID_TABLE_LENGTH),
                           (gctPOINTER *)&table));

        gckOS_ZeroMemory(table + database->tableLen,
                         gcmSIZEOF(gctPOINTER) * gcdID_TABLE_LENGTH);

        /* Copy data from old table. */
        gckOS_MemCopy(table,
                      database->table,
                      database->tableLen * gcmSIZEOF(gctPOINTER));

        gcmkOS_SAFE_FREE(os, database->table);

        /* Update databse with new allocated table. */
        database->table = table;
        database->currentID = database->tableLen;
        database->tableLen += gcdID_TABLE_LENGTH;
        database->unused += gcdID_TABLE_LENGTH;
    }

    table = database->table;
    currentID = database->currentID;
    tableLen = database->tableLen;
    unused = database->unused;

    /* Connect id with pointer. */
    table[currentID] = Pointer;

    *Id = currentID + 1;

    /* Update the currentID. */
    if (--unused > 0)
    {
        for (i = 0; i < tableLen; i++)
        {
            if (++currentID >= tableLen)
            {
                /* Wrap to the begin. */
                currentID = 0;
            }

            if (table[currentID] == gcvNULL)
            {
                break;
            }
        }
    }

    database->table = table;
    database->currentID = currentID;
    database->tableLen = tableLen;
    database->unused = unused;

    gcmkVERIFY_OK(gckOS_ReleaseMutex(os, database->mutex));
    acquired = gcvFALSE;

    gcmkFOOTER_ARG("*Id=%d", *Id);
    return gcvSTATUS_OK;

OnError:
    if (acquired)
    {
        gcmkVERIFY_OK(gckOS_ReleaseMutex(os, database->mutex));
    }

    gcmkFOOTER();
    return status;
}

gceSTATUS
gckKERNEL_FreeIntegerId(
    IN gctPOINTER Database,
    IN gctUINT32 Id
    )
{
    gceSTATUS status;
    gckINTEGERDB database = Database;
    gckOS os = database->os;
    gctBOOL acquired = gcvFALSE;

    gcmkHEADER_ARG("Database=0x%08X Id=%d", Database, Id);

    gcmkVERIFY_OK(gckOS_AcquireMutex(os, database->mutex, gcvINFINITE));
    acquired = gcvTRUE;

    if (!(Id > 0 && Id <= database->tableLen))
    {
        gcmkONERROR(gcvSTATUS_NOT_FOUND);
    }

    Id -= 1;

    database->table[Id] = gcvNULL;

    if (database->unused++ == 0)
    {
        database->currentID = Id;
    }

    gcmkVERIFY_OK(gckOS_ReleaseMutex(os, database->mutex));
    acquired = gcvFALSE;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (acquired)
    {
        gcmkVERIFY_OK(gckOS_ReleaseMutex(os, database->mutex));
    }

    gcmkFOOTER();
    return status;
}

gceSTATUS
gckKERNEL_QueryIntegerId(
    IN gctPOINTER Database,
    IN gctUINT32 Id,
    OUT gctPOINTER * Pointer
    )
{
    gceSTATUS status;
    gckINTEGERDB database = Database;
    gctPOINTER pointer;
    gckOS os = database->os;
    gctBOOL acquired = gcvFALSE;

    gcmkHEADER_ARG("Database=0x%08X Id=%d", Database, Id);
    gcmkVERIFY_ARGUMENT(Pointer != gcvNULL);

    gcmkVERIFY_OK(gckOS_AcquireMutex(os, database->mutex, gcvINFINITE));
    acquired = gcvTRUE;

    if (!(Id > 0 && Id <= database->tableLen))
    {
        gcmkONERROR(gcvSTATUS_NOT_FOUND);
    }

    Id -= 1;

    pointer = database->table[Id];

    gcmkVERIFY_OK(gckOS_ReleaseMutex(os, database->mutex));
    acquired = gcvFALSE;

    if (pointer)
    {
        *Pointer = pointer;
    }
    else
    {
        gcmkONERROR(gcvSTATUS_NOT_FOUND);
    }

    gcmkFOOTER_ARG("*Pointer=0x%08X", *Pointer);
    return gcvSTATUS_OK;

OnError:
    if (acquired)
    {
        gcmkVERIFY_OK(gckOS_ReleaseMutex(os, database->mutex));
    }

    gcmkFOOTER();
    return status;
}


gctUINT32
gckKERNEL_AllocateNameFromPointer(
    IN gckKERNEL Kernel,
    IN gctPOINTER Pointer
    )
{
    gceSTATUS status;
    gctUINT32 name;
    gctPOINTER database = Kernel->db->pointerDatabase;

    gcmkHEADER_ARG("Kernel=0x%X Pointer=0x%X", Kernel, Pointer);

    gcmkONERROR(
        gckKERNEL_AllocateIntegerId(database, Pointer, &name));

    gcmkFOOTER_ARG("name=%d", name);
    return name;

OnError:
    gcmkFOOTER();
    return 0;
}

gctPOINTER
gckKERNEL_QueryPointerFromName(
    IN gckKERNEL Kernel,
    IN gctUINT32 Name
    )
{
    gceSTATUS status;
    gctPOINTER pointer = gcvNULL;
    gctPOINTER database = Kernel->db->pointerDatabase;

    gcmkHEADER_ARG("Kernel=0x%X Name=%d", Kernel, Name);

    /* Lookup in database to get pointer. */
    gcmkONERROR(gckKERNEL_QueryIntegerId(database, Name, &pointer));

    gcmkFOOTER_ARG("pointer=0x%X", pointer);
    return pointer;

OnError:
    gcmkFOOTER();
    return gcvNULL;
}

gceSTATUS
gckKERNEL_DeleteName(
    IN gckKERNEL Kernel,
    IN gctUINT32 Name
    )
{
    gctPOINTER database = Kernel->db->pointerDatabase;

    gcmkHEADER_ARG("Kernel=0x%X Name=0x%X", Kernel, Name);

    /* Free name if exists. */
    gcmkVERIFY_OK(gckKERNEL_FreeIntegerId(database, Name));

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}
/*******************************************************************************
***** Test Code ****************************************************************
*******************************************************************************/

