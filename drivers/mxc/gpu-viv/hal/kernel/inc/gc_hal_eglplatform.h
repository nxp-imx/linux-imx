/****************************************************************************
*
*    Copyright (C) 2005 - 2011 by Vivante Corp.
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




#ifndef __gc_hal_eglplatform_h_
#define __gc_hal_eglplatform_h_

/* Include VDK types. */
#include <EGL/egl.h>
#include "gc_hal_types.h"
#include "gc_hal_base.h"
#ifdef __cplusplus
extern "C" {
#endif


/*******************************************************************************
** Display. ********************************************************************
*/

EGLNativeDisplayType
gcoOS_GetDisplay(
	void
    );

EGLNativeDisplayType
gcoOS_GetDisplayByIndex(
    int DisplayIndex
    );

int
gcoOS_GetDisplayInfo(
    EGLNativeDisplayType Display,
    int * Width,
    int * Height,
    unsigned long * Physical,
    int * Stride,
    int * BitsPerPixel
    );

/* VFK_DISPLAY_INFO structure defining information returned by
   vdkGetDisplayInfoEx. */
typedef struct _halDISPLAY_INFO
{
    /* The size of the display in pixels. */
    int                         width;
    int                         height;

    /* The stride of the dispay. -1 is returned if the stride is not known
    ** for the specified display.*/
    int                         stride;

    /* The color depth of the display in bits per pixel. */
    int                         bitsPerPixel;

    /* The logical pointer to the display memory buffer. NULL is returned
    ** if the pointer is not known for the specified display. */
    void *                      logical;

    /* The physical address of the display memory buffer. ~0 is returned
    ** if the address is not known for the specified display. */
    unsigned long               physical;

    /* The color info of the display. */
    unsigned int                alphaLength;
    unsigned int                alphaOffset;
    unsigned int                redLength;
    unsigned int                redOffset;
    unsigned int                greenLength;
    unsigned int                greenOffset;
    unsigned int                blueLength;
    unsigned int                blueOffset;

    /* Display flip support. */
    int                         flip;
}
halDISPLAY_INFO;

int
gcoOS_GetDisplayInfoEx(
    EGLNativeDisplayType Display,
#ifdef __QNXNTO__
    EGLNativeWindowType Window,
#endif
    unsigned int DisplayInfoSize,
    halDISPLAY_INFO * DisplayInfo
    );

int
gcoOS_GetDisplayVirtual(
    EGLNativeDisplayType Display,
    int * Width,
    int * Height
    );

int
gcoOS_GetDisplayBackbuffer(
    EGLNativeDisplayType Display,
    NativeWindowType Window,
    gctPOINTER    context,
    gcoSURF       surface,
    unsigned int * Offset,
    int * X,
    int * Y
    );

int
gcoOS_SetDisplayVirtual(
    EGLNativeDisplayType Display,
#ifdef __QNXNTO__
    EGLNativeWindowType Window,
#endif
    unsigned int Offset,
    int X,
    int Y
    );

void
gcoOS_DestroyDisplay(
    EGLNativeDisplayType Display
    );

/*******************************************************************************
** Windows. ********************************************************************
*/

EGLNativeWindowType
gcoOS_CreateWindow(
    EGLNativeDisplayType Display,
    int X,
    int Y,
    int Width,
    int Height
    );

int
gcoOS_GetWindowInfo(
#if (defined(LINUX) || defined(__APPLE__)) && !defined(EGL_API_FB)
    EGLNativeDisplayType Display,
#endif
    EGLNativeWindowType Window,
    int * X,
    int * Y,
    int * Width,
    int * Height,
    int * BitsPerPixel,
    unsigned int * Offset
    );

void
gcoOS_DestroyWindow(
#if (defined(LINUX) || defined(__APPLE__)) && !defined(EGL_API_FB)
    EGLNativeDisplayType Display,
#endif
    EGLNativeWindowType Window
    );

int
gcoOS_DrawImage(
#if (defined(LINUX) || defined(__APPLE__)) && !defined(EGL_API_FB)
    EGLNativeDisplayType Display,
#endif
    EGLNativeWindowType Window,
    int Left,
    int Top,
    int Right,
    int Bottom,
    int Width,
    int Height,
    int BitsPerPixel,
    void * Bits
    );

int
gcoOS_GetImage(
    EGLNativeWindowType Window,
    int Left,
    int Top,
    int Right,
    int Bottom,
    int * BitsPerPixel,
    void ** Bits
    );

/*******************************************************************************
** Pixmaps. ********************************************************************
*/

EGLNativePixmapType
gcoOS_CreatePixmap(
    EGLNativeDisplayType Display,
    int Width,
    int Height,
    int BitsPerPixel
    );

int
gcoOS_GetPixmapInfo(
#if (defined(LINUX) || defined(__APPLE__)) && !defined(EGL_API_FB)
    EGLNativeDisplayType Display,
#endif
    EGLNativePixmapType Pixmap,
    int * Width,
    int * Height,
    int * BitsPerPixel,
    int * Stride,
    void ** Bits
    );

int
gcoOS_DrawPixmap(
#if (defined(LINUX) || defined(__APPLE__)) && !defined(EGL_API_FB)
    EGLNativeDisplayType Display,
#endif
    EGLNativePixmapType Pixmap,
    int Left,
    int Top,
    int Right,
    int Bottom,
    int Width,
    int Height,
    int BitsPerPixel,
    void * Bits
    );

void
gcoOS_DestroyPixmap(
#if (defined(LINUX) || defined(__APPLE__)) && !defined(EGL_API_FB)
    EGLNativeDisplayType Display,
#endif
    EGLNativePixmapType Pixmap
    );

#ifdef __cplusplus
}
#endif

#endif /* __gc_hal_eglplatform_h_ */
