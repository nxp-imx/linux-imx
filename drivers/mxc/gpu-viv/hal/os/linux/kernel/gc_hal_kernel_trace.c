/****************************************************************************
*
*    The MIT License (MIT)
*
*    Copyright (c) 2014 - 2024 Vivante Corporation
*
*    Permission is hereby granted, free of charge, to any person obtaining a
*    copy of this software and associated documentation files (the "Software"),
*    to deal in the Software without restriction, including without limitation
*    the rights to use, copy, modify, merge, publish, distribute, sublicense,
*    and/or sell copies of the Software, and to permit persons to whom the
*    Software is furnished to do so, subject to the following conditions:
*
*    The above copyright notice and this permission notice shall be included in
*    all copies or substantial portions of the Software.
*
*    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
*    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
*    DEALINGS IN THE SOFTWARE.
*
*****************************************************************************
*
*    The GPL License (GPL)
*
*    Copyright (C) 2014 - 2024 Vivante Corporation
*
*    This program is free software; you can redistribute it and/or
*    modify it under the terms of the GNU General Public License
*    as published by the Free Software Foundation; either version 2
*    of the License, or (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program; if not, write to the Free Software Foundation,
*    Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*****************************************************************************
*
*    Note: This software is released under dual MIT and GPL licenses. A
*    recipient may use this file under the terms of either the MIT license or
*    GPL License. If you wish to use only one license not the other, you can
*    indicate your decision by deleting one of the above license notices in your
*    version of this file.
*
*****************************************************************************/

#include "gc_hal_kernel_precomp.h"

#define _GC_OBJ_ZONE                     gcvZONE_TRACEPOINT

#define ANDROID_FIRST_APPLICATION_UID    10000

#if gcdENABLE_GPU_WORK_PERIOD_TRACE
#define CREATE_TRACE_POINTS
#include "gc_hal_kernel_trace_gpu_work.h"

static void
_GpuWorkTraceTimerFunction(gctPOINTER Data)
{
    gckGPUWORK gpuWork = (gckGPUWORK)Data;
    gcsUID_INFO_PTR uid_info = gcvNULL;
    gctUINT64 currentTime = 0, activeTime = 0;

    gcmkVERIFY(gckOS_AcquireMutex(gpuWork->os, gpuWork->uidInfoListMutex, gcvINFINITE));

    gckOS_GetTime(&currentTime);

    if (currentTime < gpuWork->lastTime)
        return;

    activeTime = (currentTime - gpuWork->lastTime) * 3 / 4;

    if (activeTime > 0) {
        for (uid_info = gpuWork->uidInfoList; uid_info != gcvNULL; uid_info = uid_info->next) {
            trace_gpu_work_period(gpuWork->kernel->core, uid_info->uid, gpuWork->lastTime,
                                  currentTime, activeTime);
        }
    }

    gpuWork->lastTime = currentTime;

    gcmkVERIFY(gckOS_ReleaseMutex(gpuWork->os, gpuWork->uidInfoListMutex));

    gcmkVERIFY(gckOS_StartTimer(gpuWork->os, gpuWork->gpuWorkTimer, 1000));
}

gceSTATUS
gckGPUWORK_Construct(gckKERNEL Kernel, gckGPUWORK *GpuWork)
{
    gckOS os;
    gckGPUWORK gpuWorkObj = gcvNULL;
    gcsUID_INFO_PTR uid_info = gcvNULL;
    gctPOINTER pointer = gcvNULL;
    gceSTATUS status = gcvSTATUS_OK;
    gctUINT i;

    gcmkHEADER_ARG("Kernel=%p", Kernel);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);

    /* Extract the pointer to the gckOS object. */
    os = Kernel->os;
    gcmkVERIFY_OBJECT(os, gcvOBJ_OS);

    /* Allocate the gckGPUWORK object. */
    gcmkONERROR(gckOS_Allocate(os, gcmSIZEOF(struct _gcsGPUWORK), &pointer));

    gpuWorkObj = pointer;

    /* Reset the object. */
    gcmkONERROR(gckOS_ZeroMemory(gpuWorkObj, gcmSIZEOF(struct _gcsGPUWORK)));

    /* Initialize the gckGPUWORK object. */
    gpuWorkObj->kernel = Kernel;
    gpuWorkObj->os = os;
    gpuWorkObj->uidInfoList = gcvNULL;
    gpuWorkObj->uidInfoCount = 0;
    gpuWorkObj->freeList = gcvNULL;
    gpuWorkObj->freeCount = 0;

    /* Create the mutexes. */
    gcmkONERROR(gckOS_CreateMutex(os, &gpuWorkObj->uidInfoListMutex));
    gcmkONERROR(gckOS_CreateMutex(os, &gpuWorkObj->freeListMutex));

    /* Create a bunch of zero uid_infos. */
    for (i = 0; i < 10; i++) {
        /* Allocate an uid info structure. */
        gcmkONERROR(gckOS_Allocate(os, gcmSIZEOF(gcsUID_INFO), &pointer));

        uid_info = pointer;

        uid_info->next = gpuWorkObj->freeList;
        gpuWorkObj->freeList = uid_info;
        gpuWorkObj->freeCount += 1;
    }

    /* Create GPU Work Trace Timer. */
    gcmkONERROR(gckOS_CreateTimer(os, _GpuWorkTraceTimerFunction,
                                    (gctPOINTER)gpuWorkObj,
                                    &gpuWorkObj->gpuWorkTimer));

    /* Initial the last time and start GPU Work Trace Timer. */
    gcmkONERROR(gckOS_GetTime(&gpuWorkObj->lastTime));

    /* Return pointer to the gckGPUWORK object. */
    *GpuWork = gpuWorkObj;

    /* Success. */
    gcmkFOOTER_ARG("*GpuWork=0x%x", *GpuWork);
    return gcvSTATUS_OK;

OnError:
    /* Roll back. */
    if (gpuWorkObj != gcvNULL) {
        if (gpuWorkObj->gpuWorkTimer != gcvNULL) {
            gcmkVERIFY_OK(gckOS_StopTimer(os, gpuWorkObj->gpuWorkTimer));

            gcmkVERIFY_OK(gckOS_DestroyTimer(os, gpuWorkObj->gpuWorkTimer));
            gpuWorkObj->gpuWorkTimer = gcvNULL;
        }

        if (gpuWorkObj->uidInfoListMutex != gcvNULL)
            gcmkVERIFY_OK(gckOS_DeleteMutex(os, gpuWorkObj->uidInfoListMutex));

        if (gpuWorkObj->freeListMutex != gcvNULL)
            gcmkVERIFY_OK(gckOS_DeleteMutex(os, gpuWorkObj->freeListMutex));

        while (gpuWorkObj->freeList != gcvNULL) {
            uid_info = gpuWorkObj->freeList;
            gpuWorkObj->freeList = uid_info->next;

            gcmkVERIFY_OK(gcmkOS_SAFE_FREE(os, uid_info));
        }

        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(os, gpuWorkObj));
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckGPUWORK_Destroy(gckGPUWORK GpuWork)
{
    gcsUID_INFO_PTR uid_info = gcvNULL;

    gcmkHEADER_ARG("GpuWork=%p", GpuWork);

    /* Vefiry the arguments. */
    gcmkVERIFY_ARGUMENT(GpuWork != gcvNULL);

    /* Stop and destroy timer. */
    if (GpuWork->gpuWorkTimer != gcvNULL) {
        gcmkVERIFY_OK(gckOS_StopTimer(GpuWork->os, GpuWork->gpuWorkTimer));

        gcmkVERIFY_OK(gckOS_DestroyTimer(GpuWork->os, GpuWork->gpuWorkTimer));
        GpuWork->gpuWorkTimer = gcvNULL;
    }

    /* Free the uidInfoList. */
    while (GpuWork->uidInfoList != gcvNULL) {
        uid_info = GpuWork->uidInfoList;
        GpuWork->uidInfoList = uid_info->next;

        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(GpuWork->os, uid_info));
    }

    /* Delete the uid info list mutex. */
    if (GpuWork->uidInfoListMutex) {
        gcmkVERIFY_OK(gckOS_DeleteMutex(GpuWork->os, GpuWork->uidInfoListMutex));
        GpuWork->uidInfoListMutex = gcvNULL;
    }

    /* Free the freeList. */
    while (GpuWork->freeList != gcvNULL) {
        uid_info = GpuWork->freeList;
        GpuWork->freeList = uid_info->next;

        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(GpuWork->os, uid_info));
    }

    /* Delete the free list mutex. */
    if (GpuWork->freeListMutex) {
        gcmkVERIFY_OK(gckOS_DeleteMutex(GpuWork->os, GpuWork->freeListMutex));
        GpuWork->freeListMutex = gcvNULL;
    }

    /* Free the gckGpuWork object. */
    gcmkVERIFY_OK(gcmkOS_SAFE_FREE(GpuWork->os, GpuWork));

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

gceSTATUS
gckGPUWORK_Attach(gckGPUWORK GpuWork, gctUINT32 UserID)
{
    gctUINT32 userId = UserID;
    gcsUID_INFO_PTR uid_info = gcvNULL;
    gctBOOL acquired = gcvFALSE;
    gceSTATUS status = gcvSTATUS_OK;

    /* Verify the arguments. */
    gcmkVERIFY_ARGUMENT(GpuWork != gcvNULL);

    /* Only trace android application uid. */
    if (userId < ANDROID_FIRST_APPLICATION_UID)
        return status;

    /* Acquire the mutex. */
    gcmkONERROR(gckOS_AcquireMutex(GpuWork->os, GpuWork->uidInfoListMutex, gcvINFINITE));
    acquired = gcvTRUE;

    for (uid_info = GpuWork->uidInfoList; uid_info != gcvNULL; uid_info = uid_info->next) {
        /* Try to find the uid. */
        if (uid_info->uid == userId) {
            uid_info->reference += 1;
            goto OnError;
        }
    }

    /* Release the mutex. */
    gcmkONERROR(gckOS_ReleaseMutex(GpuWork->os, GpuWork->uidInfoListMutex));
    acquired = gcvFALSE;

    if (uid_info == gcvNULL) {
        /* Get a new uid info of the current UID. */
        gckGPUWORK_AllocateUidInfo(GpuWork, &uid_info);

        uid_info->uid = userId;
        uid_info->reference = 1;

        /* Acquire the mutex. */
        gcmkONERROR(gckOS_AcquireMutex(GpuWork->os, GpuWork->uidInfoListMutex, gcvINFINITE));
        acquired = gcvTRUE;

        /* Add the uid info to uidInfoList. */
        uid_info->next = GpuWork->uidInfoList;
        GpuWork->uidInfoList = uid_info;
        GpuWork->uidInfoCount += 1;

        /* Release the mutex. */
        gcmkONERROR(gckOS_ReleaseMutex(GpuWork->os, GpuWork->uidInfoListMutex));
        acquired = gcvFALSE;
    }

    _GpuWorkTraceTimerFunction((gctPOINTER)GpuWork);

OnError:
    if (acquired)
        gckOS_ReleaseMutex(GpuWork->os, GpuWork->uidInfoListMutex);

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckGPUWORK_Dettach(gckGPUWORK GpuWork, gctUINT32 UserID)
{
    gctUINT32 userId = UserID;
    gcsUID_INFO_PTR previous = gcvNULL, uid_info = gcvNULL;
    gctBOOL acquired = gcvFALSE, needFree = gcvFALSE;
    gceSTATUS status = gcvSTATUS_OK;

    /* Verify the arguments. */
    gcmkVERIFY_ARGUMENT(GpuWork != gcvNULL);

    /* Only trace android application uid. */
    if (userId < ANDROID_FIRST_APPLICATION_UID)
        return status;

    /* Acquire the mutex. */
    gcmkONERROR(gckOS_AcquireMutex(GpuWork->os, GpuWork->uidInfoListMutex, gcvINFINITE));
    acquired = gcvTRUE;

    if (GpuWork->uidInfoList == gcvNULL)
        goto OnError;

    for (previous = gcvNULL, uid_info = GpuWork->uidInfoList;
         uid_info != gcvNULL;
         uid_info = uid_info->next) {
        if (uid_info->uid == userId) {
            /* Found it! */
            uid_info->reference -= 1;

            if (uid_info->reference == 0) {
                /* Remove the uid_info. */
                if (previous == gcvNULL)
                    GpuWork->uidInfoList = uid_info->next;
                else
                    previous->next = uid_info->next;

                GpuWork->uidInfoCount -= 1;
                needFree = gcvTRUE;
            }
            break;
        }

        previous = uid_info;
    }

    /* Release the mutex. */
    gcmkONERROR(gckOS_ReleaseMutex(GpuWork->os, GpuWork->uidInfoListMutex));
    acquired = gcvFALSE;

    if (uid_info == gcvNULL)
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);

    if (needFree) {
        gcmkONERROR(gckGPUWORK_FreeUidInfo(GpuWork, uid_info));

        _GpuWorkTraceTimerFunction((gctPOINTER)GpuWork);
    }

    /* Success. */
    gcmkFOOTER();
    return gcvSTATUS_OK;

OnError:
    if (acquired)
        gckOS_ReleaseMutex(GpuWork->os, GpuWork->uidInfoListMutex);

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckGPUWORK_AllocateUidInfo(gckGPUWORK GpuWork, gcsUID_INFO_PTR *UidInfo)
{
    gcsUID_INFO_PTR uid_info;
    gctPOINTER pointer = gcvNULL;
    gctBOOL acquired = gcvFALSE;
    gceSTATUS status = gcvSTATUS_OK;

    /* Verify the arguments. */
    gcmkVERIFY_ARGUMENT(GpuWork != gcvNULL);

    /* Acquire the mutex. */
    gcmkONERROR(gckOS_AcquireMutex(GpuWork->os, GpuWork->freeListMutex, gcvINFINITE));
    acquired = gcvTRUE;

    if  (GpuWork->freeList == gcvNULL && GpuWork->freeCount == 0) {
        /* Allocate a new uid info. */
        gcmkONERROR(gckOS_Allocate(GpuWork->os, gcmSIZEOF(struct _gcsUID_INFO), &pointer));

        uid_info = pointer;

        /* Reset. */
        gcmkONERROR(gckOS_ZeroMemory(uid_info, gcmSIZEOF(struct _gcsUID_INFO)));

        /* Push it on the free list. */
        uid_info->next = GpuWork->freeList;
        GpuWork->freeList = uid_info;
        GpuWork->freeCount += 1;
    }

    *UidInfo = GpuWork->freeList;
    GpuWork->freeList = GpuWork->freeList->next;
    GpuWork->freeCount -= 1;

    /* Release the mutex. */
    gcmkONERROR(gckOS_ReleaseMutex(GpuWork->os, GpuWork->freeListMutex));
    acquired = gcvFALSE;

    /* Success. */
    gcmkFOOTER_ARG("*UidInfo=0x%x", gcmOPT_POINTER(UidInfo));
    return gcvSTATUS_OK;

OnError:
    if (acquired)
        gckOS_ReleaseMutex(GpuWork->os, GpuWork->freeListMutex);

    /* Return the status. */
    gcmkFOOTER();
    return status;

}

gceSTATUS
gckGPUWORK_FreeUidInfo(gckGPUWORK GpuWork, gcsUID_INFO_PTR UidInfo)
{
    gceSTATUS status = gcvSTATUS_OK;
    gctBOOL acquired = gcvFALSE;

    gcmkHEADER_ARG("GpuWork=%p UidInfo=%p", GpuWork, UidInfo);

    /* Verify the arguments. */
    gcmkVERIFY_ARGUMENT(GpuWork != gcvNULL);
    gcmkVERIFY_ARGUMENT(UidInfo != gcvNULL);

    /* Reset. */
    gcmkONERROR(gckOS_ZeroMemory(UidInfo, gcmSIZEOF(struct _gcsUID_INFO)));

    /* Acquire the mutex. */
    gcmkONERROR(gckOS_AcquireMutex(GpuWork->os, GpuWork->freeListMutex, gcvINFINITE));
    acquired = gcvTRUE;

    /* Push it on the free list. */
    UidInfo->next = GpuWork->freeList;
    GpuWork->freeList = UidInfo;
    GpuWork->freeCount += 1;

    /* Release the mutex. */
    gcmkONERROR(gckOS_ReleaseMutex(GpuWork->os, GpuWork->freeListMutex));
    acquired = gcvFALSE;

    /* Success. */
    gcmkFOOTER();
    return gcvSTATUS_OK;

OnError:
    if (acquired)
        gckOS_ReleaseMutex(GpuWork->os, GpuWork->freeListMutex);

    /* Return the status. */
    gcmkFOOTER();
    return status;
}
#endif
