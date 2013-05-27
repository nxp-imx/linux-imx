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


#ifndef __gc_hal_kernel_hardware_vg_h_
#define __gc_hal_kernel_hardware_vg_h_

/* gckHARDWARE object. */
struct _gckVGHARDWARE
{
    /* Object. */
    gcsOBJECT                   object;

    /* Pointer to gckKERNEL object. */
    gckVGKERNEL                 kernel;

    /* Pointer to gckOS object. */
    gckOS                       os;

    /* Chip characteristics. */
    gceCHIPMODEL                chipModel;
    gctUINT32                   chipRevision;
    gctUINT32                   chipFeatures;
    gctUINT32                   chipMinorFeatures;
    gctUINT32                   chipMinorFeatures2;
    gctBOOL                     allowFastClear;

    /* Features. */
    gctBOOL                     fe20;
    gctBOOL                     vg20;
    gctBOOL                     vg21;

    /* Event mask. */
    gctUINT32                   eventMask;

    gctBOOL                     clockState;
    gctBOOL                     powerState;
    gctPOINTER                  powerMutex;
    gctSIGNAL                   idleSignal;
    gctUINT32                   powerProcess;
    gctUINT32                   powerThread;
    gceCHIPPOWERSTATE           chipPowerState;
    gceCHIPPOWERSTATE           chipPowerStateGlobal;
    gctISRMANAGERFUNC           startIsr;
    gctISRMANAGERFUNC           stopIsr;
    gctPOINTER                  isrContext;
    gctUINT32                   powerOffTime;
    gctUINT32                   powerOffTimeout;
    gctTHREAD                   timeIdleThread;
    gctBOOL                     killThread;
    gctPOINTER                  pageTableDirty;

    gctBOOL                     powerManagement;
};

#endif /* __gc_hal_kernel_hardware_h_ */

