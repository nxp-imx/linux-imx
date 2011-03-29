/* Copyright (c) 2002,2007-2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __GSL_DRAWCTXT_H
#define __GSL_DRAWCTXT_H

//////////////////////////////////////////////////////////////////////////////
// Flags
//////////////////////////////////////////////////////////////////////////////

#define CTXT_FLAGS_NOT_IN_USE       0x00000000
#define CTXT_FLAGS_IN_USE           0x00000001

#define CTXT_FLAGS_STATE_SHADOW     0x00000010  // state shadow memory allocated

#define CTXT_FLAGS_GMEM_SHADOW      0x00000100  // gmem shadow memory allocated
#define CTXT_FLAGS_GMEM_SAVE        0x00000200  // gmem must be copied to shadow
#define CTXT_FLAGS_GMEM_RESTORE     0x00000400  // gmem can be restored from shadow

#define CTXT_FLAGS_SHADER_SAVE      0x00002000  // shader must be copied to shadow
#define CTXT_FLAGS_SHADER_RESTORE   0x00004000  // shader can be restored from shadow

//////////////////////////////////////////////////////////////////////////////
//  types
//////////////////////////////////////////////////////////////////////////////

// ------------
// draw context
// ------------

typedef struct _gmem_shadow_t
{
    gsl_memdesc_t   gmemshadow;     // Shadow buffer address

    // 256 KB GMEM surface = 4 bytes-per-pixel x 256 pixels/row x 256 rows.
    // width & height must be a multiples of 32, in case tiled textures are used.
    unsigned int    size;           // Size of surface used to store GMEM
    unsigned int    width;          // Width of surface used to store GMEM
    unsigned int    height;         // Height of surface used to store GMEM
    unsigned int    pitch;          // Pitch of surface used to store GMEM
	unsigned int    format;         // Format of surface used to store GMEM

    int             offset;

    unsigned int    offset_x;    
	unsigned int    offset_y;

	unsigned int	gmem_width;     // GMEM width
	unsigned int	gmem_height;    // GMEM height
	unsigned int    gmem_pitch;     // GMEM pitch

    unsigned int    gmem_offset_x;
    unsigned int    gmem_offset_y;

    unsigned int*   gmem_save_commands;
    unsigned int*   gmem_restore_commands;
    unsigned int    gmem_save[3];
    unsigned int    gmem_restore[3];

    gsl_memdesc_t   quad_vertices;
    gsl_memdesc_t   quad_texcoords;
} gmem_shadow_t;

#define GSL_MAX_GMEM_SHADOW_BUFFERS 2

typedef struct _gsl_drawctxt_t {
	unsigned int        pid;
    gsl_flags_t         flags;
    gsl_context_type_t  type;
    gsl_memdesc_t       gpustate;
    
    unsigned int        reg_save[3];
    unsigned int        reg_restore[3];
    unsigned int        shader_save[3];
    unsigned int        shader_fixup[3];
    unsigned int        shader_restore[3];
    unsigned int        chicken_restore[3];
    gmem_shadow_t       context_gmem_shadow;    // Information of the GMEM shadow that is created in context create
    gmem_shadow_t       user_gmem_shadow[GSL_MAX_GMEM_SHADOW_BUFFERS]; // User defined GMEM shadow buffers
} gsl_drawctxt_t;


//////////////////////////////////////////////////////////////////////////////
//  prototypes
//////////////////////////////////////////////////////////////////////////////
int     kgsl_drawctxt_init(gsl_device_t *device);
int     kgsl_drawctxt_close(gsl_device_t *device);
int     kgsl_drawctxt_destroyall(gsl_device_t *device);
void    kgsl_drawctxt_switch(gsl_device_t *device, gsl_drawctxt_t *drawctxt, gsl_flags_t flags);
int     kgsl_drawctxt_create(gsl_device_t* device, gsl_context_type_t type, unsigned int *drawctxt_id, gsl_flags_t flags);
int     kgsl_drawctxt_destroy(gsl_device_t* device, unsigned int drawctxt_id);

#endif  // __GSL_DRAWCTXT_H
