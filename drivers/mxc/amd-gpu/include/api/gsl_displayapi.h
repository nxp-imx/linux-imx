/* Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
	
#ifndef __GSL_DISPLAYAPI_H
#define __GSL_DISPLAYAPI_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

//////////////////////////////////////////////////////////////////////////////
//  entrypoints
//////////////////////////////////////////////////////////////////////////////
#ifdef __GSLDISPLAY_EXPORTS
#define DISP_API                    OS_DLLEXPORT
#else
#define DISP_API                    OS_DLLIMPORT
#endif // __GSLDISPLAY_EXPORTS


//////////////////////////////////////////////////////////////////////////////
//  defines                    
//////////////////////////////////////////////////////////////////////////////
#define GSL_DISPLAY_PANEL_TOSHIBA_640x480     0
#define GSL_DISPLAY_PANEL_HITACHI_240x320     1
#define GSL_DISPLAY_PANEL_DEFAULT             GSL_DISPLAY_PANEL_TOSHIBA_640x480


//////////////////////////////////////////////////////////////////////////////
//  types
//////////////////////////////////////////////////////////////////////////////
typedef int     gsl_display_id_t;
typedef int     gsl_surface_id_t;

typedef struct _gsl_displaymode_t {
    int panel_id;
    int width;
    int height;
    int bpp;
    int orientation;
    int frequency;
} gsl_displaymode_t;


//////////////////////////////////////////////////////////////////////////////
//  prototypes
//////////////////////////////////////////////////////////////////////////////
DISP_API gsl_display_id_t   gsl_display_open(gsl_devhandle_t devhandle, int panel_id);
DISP_API int                gsl_display_close(gsl_display_id_t display_id);
DISP_API int                gsl_display_getcount(void);
DISP_API int                gsl_display_setmode(gsl_display_id_t display_id, gsl_displaymode_t displaymode);
DISP_API int                gsl_display_getmode(gsl_display_id_t display_id, gsl_displaymode_t *displaymode);
DISP_API gsl_surface_id_t   gsl_display_setsurface(gsl_display_id_t display_id, void *buffer);
DISP_API int                gsl_display_getactivesurface(gsl_display_id_t display_id, void **buffer);
DISP_API int                gsl_display_flipsurface(gsl_display_id_t display_id, gsl_surface_id_t surface_id);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // __GSL_DISPLAYAPI_H
