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




#ifndef __gc_hal_eglplatform_h_
#define __gc_hal_eglplatform_h_

/* Include VDK types. */
#include "gc_hal_types.h"
#include "gc_hal_base.h"
#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
** Events. *********************************************************************
*/

typedef enum _halEventType
{
	/* Keyboard event. */
    HAL_KEYBOARD,

	/* Mouse move event. */
    HAL_POINTER,

	/* Mouse button event. */
    HAL_BUTTON,

	/* Application close event. */
	HAL_CLOSE,

	/* Application window has been updated. */
	HAL_WINDOW_UPDATE
}
halEventType;

/* Scancodes for keyboard. */
typedef enum _halKeys
{
    HAL_UNKNOWN = -1,

    HAL_BACKSPACE = 0x08,
    HAL_TAB,
    HAL_ENTER = 0x0D,
    HAL_ESCAPE = 0x1B,

    HAL_SPACE = 0x20,
    HAL_SINGLEQUOTE = 0x27,
    HAL_PAD_ASTERISK = 0x2A,
    HAL_COMMA = 0x2C,
    HAL_HYPHEN,
    HAL_PERIOD,
    HAL_SLASH,
    HAL_0,
    HAL_1,
    HAL_2,
    HAL_3,
    HAL_4,
    HAL_5,
    HAL_6,
    HAL_7,
    HAL_8,
    HAL_9,
    HAL_SEMICOLON = 0x3B,
    HAL_EQUAL = 0x3D,
    HAL_A = 0x41,
    HAL_B,
    HAL_C,
    HAL_D,
    HAL_E,
    HAL_F,
    HAL_G,
    HAL_H,
    HAL_I,
    HAL_J,
    HAL_K,
    HAL_L,
    HAL_M,
    HAL_N,
    HAL_O,
    HAL_P,
    HAL_Q,
    HAL_R,
    HAL_S,
    HAL_T,
    HAL_U,
    HAL_V,
    HAL_W,
    HAL_X,
    HAL_Y,
    HAL_Z,
    HAL_LBRACKET,
    HAL_BACKSLASH,
    HAL_RBRACKET,
    HAL_BACKQUOTE = 0x60,

    HAL_F1 = 0x80,
    HAL_F2,
    HAL_F3,
    HAL_F4,
    HAL_F5,
    HAL_F6,
    HAL_F7,
    HAL_F8,
    HAL_F9,
    HAL_F10,
    HAL_F11,
    HAL_F12,

    HAL_LCTRL,
    HAL_RCTRL,
    HAL_LSHIFT,
    HAL_RSHIFT,
    HAL_LALT,
    HAL_RALT,
    HAL_CAPSLOCK,
    HAL_NUMLOCK,
    HAL_SCROLLLOCK,
    HAL_PAD_0,
    HAL_PAD_1,
    HAL_PAD_2,
    HAL_PAD_3,
    HAL_PAD_4,
    HAL_PAD_5,
    HAL_PAD_6,
    HAL_PAD_7,
    HAL_PAD_8,
    HAL_PAD_9,
    HAL_PAD_HYPHEN,
    HAL_PAD_PLUS,
    HAL_PAD_SLASH,
    HAL_PAD_PERIOD,
    HAL_PAD_ENTER,
    HAL_SYSRQ,
    HAL_PRNTSCRN,
    HAL_BREAK,
    HAL_UP,
    HAL_LEFT,
    HAL_RIGHT,
    HAL_DOWN,
    HAL_HOME,
    HAL_END,
    HAL_PGUP,
    HAL_PGDN,
    HAL_INSERT,
    HAL_DELETE,
    HAL_LWINDOW,
    HAL_RWINDOW,
    HAL_MENU,
    HAL_POWER,
    HAL_SLEEP,
    HAL_WAKE
}
halKeys;

/* Structure that defined keyboard mapping. */
typedef struct _halKeyMap
{
	/* Normal key. */
    halKeys normal;

	/* Extended key. */
    halKeys extended;
}
halKeyMap;

/* Event structure. */
typedef struct _halEvent
{
	/* Event type. */
    halEventType type;

	/* Event data union. */
    union _halEventData
    {
		/* Event data for keyboard. */
        struct _halKeyboard
        {
			/* Scancode. */
            halKeys	scancode;

			/* ASCII characte of the key pressed. */
            gctCHAR	key;

			/* Flag whether the key was pressed (1) or released (0). */
            gctCHAR	pressed;
        }
        keyboard;

		/* Event data for pointer. */
        struct _halPointer
        {
			/* Current pointer coordinate. */
            gctINT		x;
            gctINT		y;
        }
        pointer;

		/* Event data for mouse buttons. */
        struct _halButton
        {
			/* Left button state. */
            gctINT		left;

			/* Middle button state. */
            gctINT		middle;

			/* Right button state. */
            gctINT		right;

			/* Current pointer coordinate. */
			gctINT		x;
			gctINT		y;
        }
        button;
    }
    data;
}
halEvent;

#if defined(_WIN32) || defined(__VC32__) && !defined(__CYGWIN__) && !defined(__SCITECH_SNAP__)
/* Win32 and Windows CE platforms. */
#include <windows.h>
typedef HDC             HALNativeDisplayType;
typedef HWND            HALNativeWindowType;
typedef HBITMAP         HALNativePixmapType;

typedef struct __BITFIELDINFO{
    BITMAPINFO    bmi;
    RGBQUAD       bmiColors[2];
} BITFIELDINFO;

#elif defined(LINUX) && defined(EGL_API_FB) && !defined(__APPLE__)
/* Linux platform for FBDEV. */
typedef struct _FBDisplay * HALNativeDisplayType;
typedef struct _FBWindow *  HALNativeWindowType;
typedef struct _FBPixmap *  HALNativePixmapType;

#elif defined(__ANDROID__) || defined(ANDROID)

struct egl_native_pixmap_t;

#if ANDROID_SDK_VERSION >= 9
    #include <android/native_window.h>

    typedef struct ANativeWindow*           HALNativeWindowType;
    typedef struct egl_native_pixmap_t*     HALNativePixmapType;
    typedef void*                           HALNativeDisplayType;
#else
    struct android_native_window_t;
    typedef struct android_native_window_t*    HALNativeWindowType;
    typedef struct egl_native_pixmap_t *        HALNativePixmapType;
    typedef void*                               HALNativeDisplayType;
#endif

#elif defined(LINUX) || defined(__APPLE__)
/* X11 platform. */
#include <X11/Xlib.h>
#include <X11/Xutil.h>

typedef Display *   HALNativeDisplayType;
typedef Window      HALNativeWindowType;

#ifdef CUSTOM_PIXMAP
typedef void *      HALNativePixmapType;
#else
typedef Pixmap      HALNativePixmapType;
#endif /* CUSTOM_PIXMAP */

/* Rename some badly named X defines. */
#ifdef Status
#   define XStatus      int
#   undef Status
#endif
#ifdef Always
#   define XAlways      2
#   undef Always
#endif
#ifdef CurrentTime
#   undef CurrentTime
#   define XCurrentTime 0
#endif

#elif defined(__QNXNTO__)
#include <screen/screen.h>

/* VOID */
typedef int              HALNativeDisplayType;
typedef screen_window_t  HALNativeWindowType;
typedef screen_pixmap_t  HALNativePixmapType;

#else

#error "Platform not recognized"

/* VOID */
typedef void *  HALNativeDisplayType;
typedef void *  HALNativeWindowType;
typedef void *  HALNativePixmapType;

#endif



/*******************************************************************************
** Display. ********************************************************************
*/

gceSTATUS
gcoOS_GetDisplay(
    OUT HALNativeDisplayType * Display
    );

gceSTATUS
gcoOS_GetDisplayByIndex(
    IN gctINT DisplayIndex,
    OUT HALNativeDisplayType * Display
    );

gceSTATUS
gcoOS_GetDisplayInfo(
    IN HALNativeDisplayType Display,
    OUT gctINT * Width,
    OUT gctINT * Height,
    OUT gctSIZE_T * Physical,
    OUT gctINT * Stride,
    OUT gctINT * BitsPerPixel
    );

/* VFK_DISPLAY_INFO structure defining information returned by
   vdkGetDisplayInfoEx. */
typedef struct _halDISPLAY_INFO
{
    /* The size of the display in pixels. */
    gctINT                         width;
    gctINT                         height;

    /* The stride of the dispay. -1 is returned if the stride is not known
    ** for the specified display.*/
    gctINT                         stride;

    /* The color depth of the display in bits per pixel. */
    gctINT                         bitsPerPixel;

    /* The logical pointer to the display memory buffer. NULL is returned
    ** if the pointer is not known for the specified display. */
    gctPOINTER                      logical;

    /* The physical address of the display memory buffer. ~0 is returned
    ** if the address is not known for the specified display. */
    gctSIZE_T               physical;

#ifndef __QNXNTO__
    /* 355_FB_MULTI_BUFFER */
    gctINT                      multiBuffer;
    gctINT                      backBufferY;
#endif

    /* The color info of the display. */
    gctUINT                alphaLength;
    gctUINT                alphaOffset;
    gctUINT                redLength;
    gctUINT                redOffset;
    gctUINT                greenLength;
    gctUINT                greenOffset;
    gctUINT                blueLength;
    gctUINT                blueOffset;

    /* Display flip support. */
    gctINT                         flip;
}
halDISPLAY_INFO;

gceSTATUS
gcoOS_GetDisplayInfoEx(
    IN HALNativeDisplayType Display,
    IN HALNativeWindowType Window,
    IN gctUINT DisplayInfoSize,
    OUT halDISPLAY_INFO * DisplayInfo
    );

gceSTATUS
gcoOS_GetNextDisplayInfoEx(
    IN HALNativeDisplayType Display,
    IN HALNativeWindowType Window,
    IN gctUINT DisplayInfoSize,
    OUT halDISPLAY_INFO * DisplayInfo
    );

gceSTATUS
gcoOS_GetDisplayVirtual(
    IN HALNativeDisplayType Display,
    OUT gctINT * Width,
    OUT gctINT * Height
    );

gceSTATUS
gcoOS_GetDisplayBackbuffer(
    IN HALNativeDisplayType Display,
    IN HALNativeWindowType Window,
    OUT gctPOINTER  *  context,
    OUT gcoSURF     *  surface,
    OUT gctUINT * Offset,
    OUT gctINT * X,
    OUT gctINT * Y
    );

gceSTATUS
gcoOS_SetDisplayVirtual(
    IN HALNativeDisplayType Display,
    IN HALNativeWindowType Window,
    IN gctUINT Offset,
    IN gctINT X,
    IN gctINT Y
    );

gceSTATUS
gcoOS_DisplayBufferRegions(
    IN HALNativeDisplayType Display,
    IN HALNativeWindowType Window,
    IN gctINT NumRects,
    IN gctINT_PTR Rects
    );

gceSTATUS
gcoOS_DestroyDisplay(
    IN HALNativeDisplayType Display
    );

gceSTATUS
gcoOS_InitLocalDisplayInfo(
    IN OUT gctPOINTER * localDisplay
    );

gceSTATUS
gcoOS_DeinitLocalDisplayInfo(
    IN OUT gctPOINTER * localDisplay
    );

gceSTATUS
gcoOS_GetDisplayInfoEx2(
    IN HALNativeDisplayType Display,
    IN HALNativeWindowType Window,
    IN gctPOINTER  localDisplay,
    IN gctUINT DisplayInfoSize,
    OUT halDISPLAY_INFO * DisplayInfo
    );

gceSTATUS
gcoOS_GetDisplayBackbufferEx(
    IN HALNativeDisplayType Display,
    IN HALNativeWindowType Window,
    IN gctPOINTER  localDisplay,
    OUT gctPOINTER  *  context,
    OUT gcoSURF     *  surface,
    OUT gctUINT * Offset,
    OUT gctINT * X,
    OUT gctINT * Y
    );

gceSTATUS
gcoOS_IsValidDisplay(
    IN HALNativeDisplayType Display
    );

gceSTATUS
gcoOS_GetNativeVisualId(
    IN HALNativeDisplayType Display,
    OUT gctINT* nativeVisualId
    );


/*******************************************************************************
** Windows. ********************************************************************
*/

gceSTATUS
gcoOS_CreateWindow(
    IN HALNativeDisplayType Display,
    IN gctINT X,
    IN gctINT Y,
    IN gctINT Width,
    IN gctINT Height,
    OUT HALNativeWindowType * Window
    );

gceSTATUS
gcoOS_GetWindowInfo(
    IN HALNativeDisplayType Display,
    IN HALNativeWindowType Window,
    OUT gctINT * X,
    OUT gctINT * Y,
    OUT gctINT * Width,
    OUT gctINT * Height,
    OUT gctINT * BitsPerPixel,
    OUT gctUINT * Offset
    );

gceSTATUS
gcoOS_DestroyWindow(
    IN HALNativeDisplayType Display,
    IN HALNativeWindowType Window
    );

gceSTATUS
gcoOS_DrawImage(
    IN HALNativeDisplayType Display,
    IN HALNativeWindowType Window,
    IN gctINT Left,
    IN gctINT Top,
    IN gctINT Right,
    IN gctINT Bottom,
    IN gctINT Width,
    IN gctINT Height,
    IN gctINT BitsPerPixel,
    IN gctPOINTER Bits
    );

gceSTATUS
gcoOS_GetImage(
    IN HALNativeWindowType Window,
    IN gctINT Left,
    IN gctINT Top,
    IN gctINT Right,
    IN gctINT Bottom,
    OUT gctINT * BitsPerPixel,
    OUT gctPOINTER * Bits
    );

gceSTATUS
gcoOS_GetWindowInfoEx(
    IN HALNativeDisplayType Display,
    IN HALNativeWindowType Window,
    OUT gctINT * X,
    OUT gctINT * Y,
    OUT gctINT * Width,
    OUT gctINT * Height,
    OUT gctINT * BitsPerPixel,
    OUT gctUINT * Offset,
    OUT gceSURF_FORMAT * Format
    );

gceSTATUS
gcoOS_DrawImageEx(
    IN HALNativeDisplayType Display,
    IN HALNativeWindowType Window,
    IN gctINT Left,
    IN gctINT Top,
    IN gctINT Right,
    IN gctINT Bottom,
    IN gctINT Width,
    IN gctINT Height,
    IN gctINT BitsPerPixel,
    IN gctPOINTER Bits,
    IN gceSURF_FORMAT  Format
    );

/*******************************************************************************
** Pixmaps. ********************************************************************
*/

gceSTATUS
gcoOS_CreatePixmap(
    IN HALNativeDisplayType Display,
    IN gctINT Width,
    IN gctINT Height,
    IN gctINT BitsPerPixel,
    OUT HALNativePixmapType * Pixmap
    );

gceSTATUS
gcoOS_GetPixmapInfo(
    IN HALNativeDisplayType Display,
    IN HALNativePixmapType Pixmap,
    OUT gctINT * Width,
    OUT gctINT * Height,
    OUT gctINT * BitsPerPixel,
    OUT gctINT * Stride,
    OUT gctPOINTER * Bits
    );

gceSTATUS
gcoOS_DrawPixmap(
    IN HALNativeDisplayType Display,
    IN HALNativePixmapType Pixmap,
    IN gctINT Left,
    IN gctINT Top,
    IN gctINT Right,
    IN gctINT Bottom,
    IN gctINT Width,
    IN gctINT Height,
    IN gctINT BitsPerPixel,
    IN gctPOINTER Bits
    );

gceSTATUS
gcoOS_DestroyPixmap(
    IN HALNativeDisplayType Display,
    IN HALNativePixmapType Pixmap
    );

gceSTATUS
gcoOS_GetPixmapInfoEx(
    IN HALNativeDisplayType Display,
    IN HALNativePixmapType Pixmap,
    OUT gctINT * Width,
    OUT gctINT * Height,
    OUT gctINT * BitsPerPixel,
    OUT gctINT * Stride,
    OUT gctPOINTER * Bits,
    OUT gceSURF_FORMAT * Format
    );

gceSTATUS
gcoOS_CopyPixmapBits(
    IN HALNativeDisplayType Display,
    IN HALNativePixmapType Pixmap,
    IN gctUINT DstWidth,
    IN gctUINT DstHeight,
    IN gctINT DstStride,
    IN gceSURF_FORMAT DstFormat,
    OUT gctPOINTER DstBits
    );

/*******************************************************************************
** OS relative. ****************************************************************
*/
gceSTATUS
gcoOS_LoadEGLLibrary(
    OUT gctHANDLE * Handle
    );

gceSTATUS
gcoOS_FreeEGLLibrary(
    IN gctHANDLE Handle
    );

gceSTATUS
gcoOS_ShowWindow(
    IN HALNativeDisplayType Display,
    IN HALNativeWindowType Window
    );

gceSTATUS
gcoOS_HideWindow(
    IN HALNativeDisplayType Display,
    IN HALNativeWindowType Window
    );

gceSTATUS
gcoOS_SetWindowTitle(
    IN HALNativeDisplayType Display,
    IN HALNativeWindowType Window,
    IN gctCONST_STRING Title
    );

gceSTATUS
gcoOS_CapturePointer(
    IN HALNativeDisplayType Display,
    IN HALNativeWindowType Window
    );

gceSTATUS
gcoOS_GetEvent(
    IN HALNativeDisplayType Display,
    IN HALNativeWindowType Window,
    OUT halEvent * Event
    );

gceSTATUS
gcoOS_CreateClientBuffer(
    IN gctINT Width,
    IN gctINT Height,
    IN gctINT Format,
    IN gctINT Type,
    OUT gctPOINTER * ClientBuffer
    );

gceSTATUS
gcoOS_GetClientBufferInfo(
    IN gctPOINTER ClientBuffer,
    OUT gctINT * Width,
    OUT gctINT * Height,
    OUT gctINT * Stride,
    OUT gctPOINTER * Bits
    );

gceSTATUS
gcoOS_DestroyClientBuffer(
    IN gctPOINTER ClientBuffer
    );


#ifdef __cplusplus
}
#endif

#endif /* __gc_hal_eglplatform_h_ */
