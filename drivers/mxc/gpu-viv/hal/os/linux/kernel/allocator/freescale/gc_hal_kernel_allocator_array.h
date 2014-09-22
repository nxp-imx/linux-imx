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


extern gceSTATUS
_DefaultAlloctorInit(
    IN gckOS Os,
    OUT gckALLOCATOR * Allocator
    );

#if LINUX_CMA_FSL
gceSTATUS
_CMAFSLAlloctorInit(
    IN gckOS Os,
    OUT gckALLOCATOR * Allocator
    );
#endif

gcsALLOCATOR_DESC allocatorArray[] =
{
#if LINUX_CMA_FSL
    gcmkDEFINE_ALLOCATOR_DESC("cmafsl", _CMAFSLAlloctorInit),
#endif
    /* Default allocator. */
    gcmkDEFINE_ALLOCATOR_DESC("default", _DefaultAlloctorInit),
};


