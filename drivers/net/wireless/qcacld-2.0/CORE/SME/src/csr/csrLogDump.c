/*
 * Copyright (c) 2013, 2016 The Linux Foundation. All rights reserved.
 *
 * Previously licensed under the ISC license by Qualcomm Atheros, Inc.
 *
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * This file was originally distributed by Qualcomm Atheros, Inc.
 * under proprietary terms before Copyright ownership was assigned
 * to the Linux Foundation.
 */

/*============================================================================
csrLogDump.c
Implements the dump commands specific to the csr module.
============================================================================*/
#include "aniGlobal.h"
#include "csrApi.h"
#include "logDump.h"
#include "smsDebug.h"
#include "smeInside.h"
#include "csrInsideApi.h"
#if defined(ANI_LOGDUMP)
static char *
dump_csr( tpAniSirGlobal pMac, tANI_U32 arg1, tANI_U32 arg2, tANI_U32 arg3, tANI_U32 arg4, char *p )
{
    static tCsrRoamProfile x;
    static tSirMacSSid ssid;   //To be allocated for array of SSIDs
    static tANI_U8 sessionId; // Defined for fixed session ID
    vos_mem_set((void*)&x, sizeof(x), 0);
    x.SSIDs.numOfSSIDs=1 ;
    x.SSIDs.SSIDList[0].SSID = ssid ;
    ssid.length=6 ;
    vos_mem_copy(ssid.ssId, "AniNet", 6);
    if(HAL_STATUS_SUCCESS(sme_AcquireGlobalLock( &pMac->sme )))
    {
        (void)csrRoamConnect(pMac, sessionId, &x, NULL, NULL);
        sme_ReleaseGlobalLock( &pMac->sme );
    }
    return p;
}

static char* dump_csrApConcScanParams( tpAniSirGlobal pMac, tANI_U32 arg1,
                               tANI_U32 arg2, tANI_U32 arg3, tANI_U32 arg4, char *p )
{
    if( arg1 )
    {
        pMac->roam.configParam.nRestTimeConc = arg1;
    }
    if( arg2 )
    {
        pMac->roam.configParam.nActiveMinChnTimeConc = arg2;
    }
    if( arg3 )
    {
        pMac->roam.configParam.nActiveMaxChnTimeConc = arg3;
    }

    smsLog(pMac, LOGE, FL(" Working %d %d %d"), (int) pMac->roam.configParam.nRestTimeConc,
        (int)pMac->roam.configParam.nActiveMinChnTimeConc, (int) pMac->roam.configParam.nActiveMaxChnTimeConc);
    return p;
}

static tDumpFuncEntry csrMenuDumpTable[] = {
    {0,     "CSR (850-860)",                                    NULL},
    {851,   "CSR: CSR testing connection to AniNet",            dump_csr},
    {853,   "CSR: Split Scan related params",                   dump_csrApConcScanParams},
};

void csrDumpInit(tHalHandle hHal)
{
    logDumpRegisterTable( (tpAniSirGlobal)hHal, &csrMenuDumpTable[0],
                          sizeof(csrMenuDumpTable)/sizeof(csrMenuDumpTable[0]) );
}

#endif //#if defined(ANI_LOGDUMP)
