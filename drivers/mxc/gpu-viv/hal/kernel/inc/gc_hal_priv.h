/****************************************************************************
*
*    The MIT License (MIT)
*
*    Copyright (c) 2014 - 2016 Vivante Corporation
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
*    Copyright (C) 2014 - 2016 Vivante Corporation
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

#ifndef __gc_hal_priv_h_
#define __gc_hal_priv_h_

#include "gc_hal_base.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _gcsSYMBOLSLIST *        gcsSYMBOLSLIST_PTR;
/******************************************************************************
**
** Patch defines which should be moved to dedicate file later
**
** !!! ALWAYS ADD new ID in the TAIL, otherwise will break exising TRACE FILE
*******************************************************************************/
typedef enum _gcePATCH_ID
{
    gcvPATCH_NOTINIT = -1,
    gcvPATCH_INVALID = 0,

    gcvPATCH_DEBUG,
    gcvPATCH_GTFES30,
    gcvPATCH_CTGL11,
    gcvPATCH_CTGL20,
    gcvPATCH_GLBM11,
    gcvPATCH_GLBM21,
    gcvPATCH_GLBM25,
    gcvPATCH_GLBM27,
    gcvPATCH_GLBMGUI,
    gcvPATCH_GFXBENCH,
    gcvPATCH_ANTUTU,        /* Antutu 3.x */
    gcvPATCH_ANTUTU4X,      /* Antutu 4.x */
    gcvPATCH_ANTUTU5X,      /* Antutu 5.x */
    gcvPATCH_ANTUTUGL3,     /* Antutu 3D Rating */
    gcvPATCH_QUADRANT,
    gcvPATCH_GPUBENCH,
    gcvPATCH_GLOFTSXHM,
    gcvPATCH_XRUNNER,
    gcvPATCH_BUSPARKING3D,
    gcvPATCH_SIEGECRAFT,
    gcvPATCH_PREMIUM,
    gcvPATCH_RACEILLEGAL,
    gcvPATCH_MEGARUN,
    gcvPATCH_BMGUI,
    gcvPATCH_NENAMARK,
    gcvPATCH_NENAMARK2,
    gcvPATCH_FISHNOODLE,
    gcvPATCH_MM06,
    gcvPATCH_MM07,
    gcvPATCH_BM21,
    gcvPATCH_SMARTBENCH,
    gcvPATCH_JPCT,
    gcvPATCH_NEOCORE,
    gcvPATCH_RTESTVA,
    gcvPATCH_NBA2013,
    gcvPATCH_BARDTALE,
    gcvPATCH_F18,
    gcvPATCH_CARPARK,
    gcvPATCH_CARCHALLENGE,
    gcvPATCH_HEROESCALL,
    gcvPATCH_GLOFTF3HM,
    gcvPATCH_CRAZYRACING,
    gcvPATCH_FIREFOX,
    gcvPATCH_CHROME,
    gcvPATCH_MONOPOLY,
    gcvPATCH_SNOWCOLD,
    gcvPATCH_BM3,
    gcvPATCH_BASEMARKX,
    gcvPATCH_DEQP,
    gcvPATCH_SF4,
    gcvPATCH_MGOHEAVEN2,
    gcvPATCH_SILIBILI,
    gcvPATCH_ELEMENTSDEF,
    gcvPATCH_GLOFTKRHM,
    gcvPATCH_OCLCTS,
    gcvPATCH_A8HP,
    gcvPATCH_A8CN,
    gcvPATCH_WISTONESG,
    gcvPATCH_SPEEDRACE,
    gcvPATCH_FSBHAWAIIF,
    gcvPATCH_AIRNAVY,
    gcvPATCH_F18NEW,
    gcvPATCH_CKZOMBIES2,
    gcvPATCH_EADGKEEPER,
    gcvPATCH_BASEMARK2V2,
    gcvPATCH_RIPTIDEGP2,
    gcvPATCH_OESCTS,
    gcvPATCH_GANGSTAR,
    gcvPATCH_TRIAL,
    gcvPATCH_WHRKYZIXOVAN,
    gcvPATCH_GMMY16MAPFB,
    gcvPATCH_UIMARK,
    gcvPATCH_NAMESGAS,
    gcvPATCH_AFTERBURNER,
    gcvPATCH_ANDROID_CTS_MEDIA,
    gcvPATCH_FM_OES_PLAYER,
    gcvPATCH_SUMSUNG_BENCH,
    gcvPATCH_ROCKSTAR_MAXPAYNE,
    gcvPATCH_TITANPACKING,
    gcvPATCH_OES20SFT,
    gcvPATCH_OES30SFT,
    gcvPATCH_BASEMARKOSIICN,
    gcvPATCH_ANDROID_WEBGL,
    gcvPATCH_ANDROID_COMPOSITOR,
    gcvPATCH_CTS_TEXTUREVIEW,
    gcvPATCH_WATER2_CHUKONG,
    gcvPATCH_GOOGLEEARTH,
    gcvPATCH_LEANBACK,
    gcvPATCH_YOUTUBE_TV,
    gcvPATCH_NETFLIX,
    gcvPATCH_ANGRYBIRDS,
    gcvPATCH_REALRACING,
    gcvPATCH_TEMPLERUN,
    gcvPATCH_SBROWSER,
    gcvPATCH_CLASHOFCLAN,
    gcvPATCH_YOUILABS_SHADERTEST,
    gcvPATCH_AXX_SAMPLE,
    gcvPATCH_3DMARKSS,
    gcvPATCH_GFXBENCH4,
    gcvPATCH_BATCHCOUNT,
    gcePATCH_ANDROID_CTS_GRAPHICS_GLVERSION,

    gcvPATCH_COUNT
} gcePATCH_ID;


#if gcdENABLE_3D
#define gcdPROC_IS_WEBGL(patchId) ((patchId) == gcvPATCH_CHROME || (patchId) == gcvPATCH_FIREFOX || (patchId) == gcvPATCH_ANDROID_WEBGL)
#endif /* gcdENABLE_3D */


/******************************************************************************\
******************************* Process local storage *************************
\******************************************************************************/

typedef struct _gcsPLS * gcsPLS_PTR;

typedef void (* gctPLS_DESTRUCTOR) (
    gcsPLS_PTR
    );

typedef struct _gcsPLS
{
    /* Global objects. */
    gcoOS                       os;
    gcoHAL                      hal;

    /* Internal memory pool. */
    gctSIZE_T                   internalSize;
    gctPHYS_ADDR                internalPhysical;
    gctPOINTER                  internalLogical;

    /* External memory pool. */
    gctSIZE_T                   externalSize;
    gctPHYS_ADDR                externalPhysical;
    gctPOINTER                  externalLogical;

    /* Contiguous memory pool. */
    gctSIZE_T                   contiguousSize;
    gctPHYS_ADDR                contiguousPhysical;
    gctPOINTER                  contiguousLogical;

    /* EGL-specific process-wide objects. */
    gctPOINTER                  eglDisplayInfo;
    gctPOINTER                  eglSurfaceInfo;
    gceSURF_FORMAT              eglConfigFormat;

    /* PLS reference count */
    gcsATOM_PTR                 reference;

    /* PorcessID of the constrcutor process */
    gctUINT32                   processID;

    /* ThreadID of the constrcutor process. */
    gctSIZE_T                   threadID;
    /* Flag for calling module destructor. */
    gctBOOL                     exiting;

    gctBOOL                     bNeedSupportNP2Texture;

    gctPLS_DESTRUCTOR           destructor;
    /* Mutex to guard PLS access. currently it's for EGL.
    ** We can use this mutex for every PLS access.
    */
    gctPOINTER                  accessLock;

    /* Global patchID to overwrite the detection */
    gcePATCH_ID                 patchID;
}
gcsPLS;

extern gcsPLS gcPLS;

gceSTATUS
gcoHAL_SetPatchID(
    IN  gcoHAL Hal,
    IN  gcePATCH_ID PatchID
    );

/* Get Patch ID based on process name */
gceSTATUS
gcoHAL_GetPatchID(
    IN  gcoHAL Hal,
    OUT gcePATCH_ID * PatchID
    );

gceSTATUS
gcoHAL_SetGlobalPatchID(
    IN  gcoHAL Hal,
    IN  gcePATCH_ID PatchID
    );

/* Detect if the current process is the executable specified. */
gceSTATUS
gcoOS_DetectProcessByName(
    IN gctCONST_STRING Name
    );

gceSTATUS
gcoOS_DetectProcessByEncryptedName(
    IN gctCONST_STRING Name
    );

gceSTATUS
gcoOS_DetectProgrameByEncryptedSymbols(
    IN gcsSYMBOLSLIST_PTR Symbols
    );

#ifdef __cplusplus
}
#endif

#endif /* __gc_hal_priv_h_ */
