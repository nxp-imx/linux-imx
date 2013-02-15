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


#include <stdarg.h>

#ifndef __gc_hal_kernel_debugfs_h_
#define __gc_hal_kernel_debugfs_h_

 #define MAX_LINE_SIZE 768  	     /* Max bytes for a line of debug info */


 typedef struct _gcsDebugFileSystemNode gcsDebugFileSystemNode ;


/*******************************************************************************
 **
 **                             System Related
 **
 *******************************************************************************/

gctINT    gckDebugFileSystemIsEnabled(void);

gctINT   gckDebugFileSystemInitialize(void);

gctINT   gckDebugFileSystemTerminate(void);


/*******************************************************************************
 **
 **                             Node Related
 **
 *******************************************************************************/

gctINT gckDebugFileSystemCreateNode(
 			IN gctINT SizeInKB,
                        IN gctCONST_STRING  ParentName ,
                        IN gctCONST_STRING  NodeName,
                        OUT gcsDebugFileSystemNode  **Node
                        );


void gckDebugFileSystemFreeNode(
			IN gcsDebugFileSystemNode  * Node
			);



void gckDebugFileSystemSetCurrentNode(
			IN gcsDebugFileSystemNode  * Node
			);



void gckDebugFileSystemGetCurrentNode(
			OUT gcsDebugFileSystemNode  ** Node
			);


void gckDebugFileSystemPrint(
    			IN gctCONST_STRING  Message,
    			...
   			 );

#endif


