/* Copyright (c) 2002,2007-2009, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include "gsl.h"
#include "gsl_hal.h"
#ifdef _LINUX
#include <asm/div64.h>
#endif

#ifdef GSL_BLD_YAMATO

//////////////////////////////////////////////////////////////////////////////
//
// Memory Map for Register, Constant & Instruction Shadow, and Command Buffers (34.5KB)
//
// +---------------------+------------+-------------+---+---------------------+
// | ALU Constant Shadow | Reg Shadow | C&V Buffers |Tex| Shader Instr Shadow |
// +---------------------+------------+-------------+---+---------------------+
//   ________________________________'               '___________________
//  '                                                                    '
// +--------------+-----------+------+-----------+------------------------+
// | Restore Regs | Save Regs | Quad | Gmem Save | Gmem Restore | unused  |
// +--------------+-----------+------+-----------+------------------------+
//
//       8K - ALU Constant Shadow (8K aligned)
//       4K - H/W Register Shadow (8K aligned)
//       9K - Command and Vertex Buffers
//              - Indirect command buffer : Const/Reg restore
//                      - includes Loop & Bool const shadows
//              - Indirect command buffer : Const/Reg save
//              - Quad vertices & texture coordinates
//              - Indirect command buffer : Gmem save
//              - Indirect command buffer : Gmem restore
//              - Unused (padding to 8KB boundary)
//      <1K - Texture Constant Shadow (768 bytes) (8K aligned)
//      18K - Shader Instruction Shadow
//              - 6K vertex (32 byte aligned)
//              - 6K pixel  (32 byte aligned)
//              - 6K shared (32 byte aligned)
//
// Note: Reading constants into a shadow, one at a time using REG_TO_MEM, takes
// 3 DWORDS per DWORD transfered, plus 1 DWORD for the shadow, for a total of
// 16 bytes per constant.  If the texture constants were transfered this way,
// the Command & Vertex Buffers section would extend past the 16K boundary.
// By moving the texture constant shadow area to start at 16KB boundary, we
// only require approximately 40 bytes more memory, but are able to use the
// LOAD_CONSTANT_CONTEXT shadowing feature for the textures, speeding up
// context switching.
//
// [Using LOAD_CONSTANT_CONTEXT shadowing feature for the Loop and/or Bool
// constants would require an additional 8KB each, for alignment.]
//
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Constants
//////////////////////////////////////////////////////////////////////////////

#define ALU_CONSTANTS           2048                // DWORDS
#define NUM_REGISTERS           1024                // DWORDS
#ifdef DISABLE_SHADOW_WRITES
    #define CMD_BUFFER_LEN          9216            // DWORDS
#else
    #define CMD_BUFFER_LEN          3072            // DWORDS
#endif
#define TEX_CONSTANTS           (32*6)              // DWORDS
#define BOOL_CONSTANTS          8                   // DWORDS
#define LOOP_CONSTANTS          56                  // DWORDS
#define SHADER_INSTRUCT_LOG2    9U                  // 2^n == SHADER_INSTRUCTIONS

#if defined(PM4_IM_STORE)
#define SHADER_INSTRUCT         (1<<SHADER_INSTRUCT_LOG2)   // 96-bit instructions
#else
#define SHADER_INSTRUCT         0
#endif

// LOAD_CONSTANT_CONTEXT shadow size
#define LCC_SHADOW_SIZE         0x2000              // 8KB

#define ALU_SHADOW_SIZE         LCC_SHADOW_SIZE     // 8KB
#define REG_SHADOW_SIZE         0x1000              // 4KB
#ifdef DISABLE_SHADOW_WRITES
    #define CMD_BUFFER_SIZE         0x9000          // 36KB
#else
    #define CMD_BUFFER_SIZE         0x3000          // 12KB
#endif
#define TEX_SHADOW_SIZE         (TEX_CONSTANTS*4)   // 768 bytes
#define SHADER_SHADOW_SIZE      (SHADER_INSTRUCT*12)// 6KB

#define REG_OFFSET              LCC_SHADOW_SIZE
#define CMD_OFFSET              (REG_OFFSET + REG_SHADOW_SIZE)
#define TEX_OFFSET              (CMD_OFFSET + CMD_BUFFER_SIZE)
#define SHADER_OFFSET           ((TEX_OFFSET + TEX_SHADOW_SIZE + 32) & ~31)

#define CONTEXT_SIZE            (SHADER_OFFSET + 3 * SHADER_SHADOW_SIZE)


/////////////////////////////////////////////////////////////////////////////
// macros
//////////////////////////////////////////////////////////////////////////////
#ifdef GSL_LOCKING_FINEGRAIN
#define GSL_CONTEXT_MUTEX_CREATE()          device->drawctxt_mutex = kos_mutex_create("gsl_drawctxt"); \
                                            if (!device->drawctxt_mutex) {return (GSL_FAILURE);}
#define GSL_CONTEXT_MUTEX_LOCK()            kos_mutex_lock(device->drawctxt_mutex)
#define GSL_CONTEXT_MUTEX_UNLOCK()          kos_mutex_unlock(device->drawctxt_mutex)
#define GSL_CONTEXT_MUTEX_FREE()            kos_mutex_free(device->drawctxt_mutex); device->drawctxt_mutex = 0;
#else
#define GSL_CONTEXT_MUTEX_CREATE()
#define GSL_CONTEXT_MUTEX_LOCK()
#define GSL_CONTEXT_MUTEX_UNLOCK()
#define GSL_CONTEXT_MUTEX_FREE()
#endif


//////////////////////////////////////////////////////////////////////////////
// temporary work structure
//////////////////////////////////////////////////////////////////////////////

typedef struct
{
    unsigned int    *start;         // Command & Vertex buffer start
    unsigned int    *cmd;           // Next available dword in C&V buffer

    // address of buffers, needed when creating IB1 command buffers.
    gpuaddr_t       bool_shadow;    // Address where bool constants are shadowed
    gpuaddr_t       loop_shadow;    // Address where loop constants are shadowed

#if defined(PM4_IM_STORE)
    gpuaddr_t       shader_shared;  // Address of shared shader instruction shadow
    gpuaddr_t       shader_vertex;  // Address of vertex shader instruction shadow
    gpuaddr_t       shader_pixel;   // Address of pixel shader instruction shadow
#endif

    gpuaddr_t       reg_values[2];  // Addresses in command buffer where separately handled registers are saved
    gpuaddr_t       chicken_restore;// Address where the TP0_CHICKEN register value is written
    gpuaddr_t       gmem_base;      // Base gpu address of GMEM
}
ctx_t;

//////////////////////////////////////////////////////////////////////////////
// Helper function to calculate IEEE754 single precision float values without FPU
//////////////////////////////////////////////////////////////////////////////
unsigned int uint2float( unsigned int uintval )
{
    unsigned int exp = 0;
    unsigned int frac = 0;
    unsigned int u = uintval;

    // Handle zero separately
    if( uintval == 0 ) return 0;

    // Find log2 of u
    if(u>=0x10000) { exp+=16; u>>=16; }
    if(u>=0x100  ) { exp+=8;  u>>=8;  }
    if(u>=0x10   ) { exp+=4;  u>>=4;  }
    if(u>=0x4    ) { exp+=2;  u>>=2;  }
    if(u>=0x2    ) { exp+=1;  u>>=1;  }

    // Calculate fraction
    frac = ( uintval & ( ~( 1 << exp ) ) ) << ( 23 - exp );

    // Exp is biased by 127 and shifted 23 bits
    exp = ( exp + 127 ) << 23;

    return ( exp | frac );
}

//////////////////////////////////////////////////////////////////////////////
// Helper function to divide two unsigned ints and return the result as a floating point value
//////////////////////////////////////////////////////////////////////////////
unsigned int uintdivide(unsigned int a, unsigned int b)
{
#ifdef _LINUX
    uint64_t a_fixed = a << 16;
    uint64_t b_fixed = b << 16;
#else
    unsigned int a_fixed = a << 16;
    unsigned int b_fixed = b << 16;
#endif
    // Assume the result is 0.fraction
    unsigned int fraction;
    unsigned int exp = 126;

    if( b == 0 ) return 0;

#ifdef _LINUX
    a_fixed = a_fixed << 32;
	do_div(a_fixed, b_fixed);
    fraction = (unsigned int)a_fixed;
#else
    fraction = ((unsigned int)((((__int64)a_fixed) << 32) / (__int64)b_fixed));
#endif

    if( fraction == 0 ) return 0;

    // Normalize
    while( !(fraction & (1<<31)) )
    {
        fraction <<= 1;
        exp--;
    }
    // Remove hidden bit
    fraction <<= 1;

    // Round
    if( ( fraction & 0x1ff ) > 256 )
    {
        int rounded = 0;
        int i = 9;

        // Do the bit addition
        while( !rounded )
        {
            if( fraction & (1<<i) )
            {
                // 1b + 1b = 0b, carry = 1
                fraction &= ~(1<<i);
                i++;
            }
            else
            {
                fraction |= (1<<i);
                rounded = 1;
            }
        }
    }

    // Use 23 most significant bits for the fraction
    fraction >>= 9;

    return ( ( exp << 23 ) | fraction );
}



//////////////////////////////////////////////////////////////////////////////
//                  context save (gmem -> sys)
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
// pre-compiled vertex shader program
//
// attribute vec4  P;
// void main(void)
// {
//   gl_Position = P;
// }
//
//////////////////////////////////////////////////////////////////////////////

#define GMEM2SYS_VTX_PGM_LEN    0x12

static const unsigned int gmem2sys_vtx_pgm[GMEM2SYS_VTX_PGM_LEN] = {
    0x00011003, 0x00001000, 0xc2000000,
    0x00001004, 0x00001000, 0xc4000000,
    0x00001005, 0x00002000, 0x00000000,
    0x1cb81000, 0x00398a88, 0x00000003,
    0x140f803e, 0x00000000, 0xe2010100,
    0x14000000, 0x00000000, 0xe2000000
};


//////////////////////////////////////////////////////////////////////////////
// pre-compiled fragment shader program
//
// precision highp float;
// uniform   vec4  clear_color;
// void main(void)
// {
//    gl_FragColor = clear_color;
// }
//
//////////////////////////////////////////////////////////////////////////////

#define GMEM2SYS_FRAG_PGM_LEN   0x0c

static const unsigned int gmem2sys_frag_pgm[GMEM2SYS_FRAG_PGM_LEN] = {
    0x00000000, 0x1002c400, 0x10000000,
    0x00001003, 0x00002000, 0x00000000,
    0x140f8000, 0x00000000, 0x22000000,
    0x14000000, 0x00000000, 0xe2000000
};


//////////////////////////////////////////////////////////////////////////////
//                  context restore (sys -> gmem)
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
// pre-compiled vertex shader program
//
// attribute vec4 position;
// attribute vec4 texcoord;
// varying   vec4 texcoord0;
// void main()
// {
//    gl_Position = position;
//    texcoord0 = texcoord;
// }
//
//////////////////////////////////////////////////////////////////////////////

#define SYS2GMEM_VTX_PGM_LEN    0x18

static const unsigned int sys2gmem_vtx_pgm[SYS2GMEM_VTX_PGM_LEN] = {
    0x00052003, 0x00001000, 0xc2000000, 0x00001005,
    0x00001000, 0xc4000000, 0x00001006, 0x10071000,
    0x20000000, 0x18981000, 0x0039ba88, 0x00000003,
    0x12982000, 0x40257b08, 0x00000002, 0x140f803e,
    0x00000000, 0xe2010100, 0x140f8000, 0x00000000,
    0xe2020200, 0x14000000, 0x00000000, 0xe2000000
};


//////////////////////////////////////////////////////////////////////////////
// pre-compiled fragment shader program
//
// precision mediump   float;
// uniform   sampler2D tex0;
// varying   vec4      texcoord0;
// void main()
// {
//    gl_FragColor = texture2D(tex0, texcoord0.xy);
// }
//
//////////////////////////////////////////////////////////////////////////////

#define SYS2GMEM_FRAG_PGM_LEN   0x0f

static const unsigned int sys2gmem_frag_pgm[SYS2GMEM_FRAG_PGM_LEN] = {
    0x00011002, 0x00001000, 0xc4000000, 0x00001003,
    0x10041000, 0x20000000, 0x10000001, 0x1ffff688,
    0x00000002, 0x140f8000, 0x00000000, 0xe2000000,
    0x14000000, 0x00000000, 0xe2000000
};


//////////////////////////////////////////////////////////////////////////////
// shader texture constants (sysmem -> gmem)
//////////////////////////////////////////////////////////////////////////////

#define SYS2GMEM_TEX_CONST_LEN  6

static unsigned int sys2gmem_tex_const[SYS2GMEM_TEX_CONST_LEN] =
{
    // Texture, FormatXYZW=Unsigned, ClampXYZ=Wrap/Repeat,RFMode=ZeroClamp-1,Dim=1:2d
    0x00000002,  // Pitch = TBD

    // Format=6:8888_WZYX, EndianSwap=0:None, ReqSize=0:256bit, DimHi=0, NearestClamp=1:OGL Mode
    0x00000806,  // Address[31:12] = TBD

    // Width, Height, EndianSwap=0:None
    0, // Width & Height = TBD

    // NumFormat=0:RF, DstSelXYZW=XYZW, ExpAdj=0, MagFilt=MinFilt=0:Point, Mip=2:BaseMap
    0 << 1 | 1 << 4 | 2 << 7 | 3 << 10 | 2 << 23,

    // VolMag=VolMin=0:Point, MinMipLvl=0, MaxMipLvl=1, LodBiasH=V=0, Dim3d=0
    0,

    // BorderColor=0:ABGRBlack, ForceBC=0:diable, TriJuice=0, Aniso=0, Dim=1:2d, MipPacking=0
    1 << 9  // Mip Address[31:12] = TBD
};


//////////////////////////////////////////////////////////////////////////////
// quad for copying GMEM to context shadow
//////////////////////////////////////////////////////////////////////////////

#define QUAD_LEN                12

static unsigned int gmem_copy_quad[QUAD_LEN] = {
    0x00000000, 0x00000000, 0x3f800000,
    0x00000000, 0x00000000, 0x3f800000,
    0x00000000, 0x00000000, 0x3f800000,
    0x00000000, 0x00000000, 0x3f800000
};

#define TEXCOORD_LEN            8

static unsigned int gmem_copy_texcoord[TEXCOORD_LEN] = {
    0x00000000, 0x3f800000,
    0x3f800000, 0x3f800000,
    0x00000000, 0x00000000,
    0x3f800000, 0x00000000
};

#define NUM_COLOR_FORMATS   13

static SurfaceFormat surface_format_table[NUM_COLOR_FORMATS] =
{
	FMT_4_4_4_4,		   // COLORX_4_4_4_4
	FMT_1_5_5_5,		   // COLORX_1_5_5_5
	FMT_5_6_5,		       // COLORX_5_6_5
	FMT_8,			       // COLORX_8
	FMT_8_8,		       // COLORX_8_8
	FMT_8_8_8_8,		   // COLORX_8_8_8_8
	FMT_8_8_8_8,		   // COLORX_S8_8_8_8
	FMT_16_FLOAT,		   // COLORX_16_FLOAT
	FMT_16_16_FLOAT,	   // COLORX_16_16_FLOAT
	FMT_16_16_16_16_FLOAT, // COLORX_16_16_16_16_FLOAT
	FMT_32_FLOAT,		   // COLORX_32_FLOAT
	FMT_32_32_FLOAT,	   // COLORX_32_32_FLOAT
	FMT_32_32_32_32_FLOAT, // COLORX_32_32_32_32_FLOAT
};

static unsigned int format2bytesperpixel[NUM_COLOR_FORMATS] =
{
    2,  // COLORX_4_4_4_4
	2,  // COLORX_1_5_5_5
	2,  // COLORX_5_6_5
	1,  // COLORX_8
	2,  // COLORX_8_8_8
	4,  // COLORX_8_8_8_8
	4,  // COLORX_S8_8_8_8
	2,  // COLORX_16_FLOAT
	4,  // COLORX_16_16_FLOAT
	8,  // COLORX_16_16_16_16_FLOAT
	4,  // COLORX_32_FLOAT
	8,  // COLORX_32_32_FLOAT
	16, // COLORX_32_32_32_32_FLOAT
};

//////////////////////////////////////////////////////////////////////////////
// shader linkage info
//////////////////////////////////////////////////////////////////////////////

#define SHADER_CONST_ADDR       (11 * 6 + 3)


//////////////////////////////////////////////////////////////////////////////
// gmem command buffer length
//////////////////////////////////////////////////////////////////////////////

#define PM4_REG(reg)            ((0x4 << 16) | (GSL_HAL_SUBBLOCK_OFFSET(reg)))

//////////////////////////////////////////////////////////////////////////////
// functions
//////////////////////////////////////////////////////////////////////////////

static void
config_gmemsize(gmem_shadow_t *shadow, int gmem_size)
{
    int w=64, h=64; // 16KB surface, minimum

    // convert from bytes to 32-bit words
    gmem_size = (gmem_size + 3)/4;

    // find the right surface size, close to a square.
    while (w * h < gmem_size)
        if (w < h)
            w *= 2;
        else
            h *= 2;

    shadow->width  = w;
    shadow->height = h;
    shadow->pitch  = w;
	shadow->format = COLORX_8_8_8_8;
    shadow->size   = shadow->pitch * shadow->height * 4;

	shadow->gmem_width  = w;
	shadow->gmem_height = h;
	shadow->gmem_pitch  = w;
}


//////////////////////////////////////////////////////////////////////////////

static unsigned int
gpuaddr(unsigned int *cmd, gsl_memdesc_t *memdesc)
{
    return memdesc->gpuaddr + ((char *)cmd - (char *)memdesc->hostptr);
}


//////////////////////////////////////////////////////////////////////////////

static void
create_ib1(gsl_drawctxt_t *drawctxt, unsigned int *cmd, unsigned int *start, unsigned int *end)
{
    cmd[0] = PM4_HDR_INDIRECT_BUFFER_PFD;
    cmd[1] = gpuaddr(start, &drawctxt->gpustate);
    cmd[2] = end - start;
}


//////////////////////////////////////////////////////////////////////////////

static unsigned int *
program_shader(unsigned int *cmds, int vtxfrag, const unsigned int *shader_pgm, int dwords)
{
    // load the patched vertex shader stream
    *cmds++ = pm4_type3_packet(PM4_IM_LOAD_IMMEDIATE, 2 + dwords);
    *cmds++ = vtxfrag;                      // 0=vertex shader, 1=fragment shader
    *cmds++ = ( (0 << 16) | dwords );       // instruction start & size (in 32-bit words)

    kos_memcpy(cmds, shader_pgm, dwords<<2);
    cmds += dwords;

    return cmds;
}


//////////////////////////////////////////////////////////////////////////////

static unsigned int *
reg_to_mem(unsigned int *cmds, gpuaddr_t dst, gpuaddr_t src, int dwords)
{
    while (dwords-- > 0)
    {
        *cmds++ = pm4_type3_packet(PM4_REG_TO_MEM, 2);
        *cmds++ = src++;
        *cmds++ = dst;
        dst += 4;
    }

    return cmds;
}



#ifdef DISABLE_SHADOW_WRITES

static void build_reg_to_mem_range(unsigned int start, unsigned int end, unsigned int** cmd, gsl_drawctxt_t *drawctxt)
{
    unsigned int i = start;

    for(i=start; i<=end; i++)
    {
        *(*cmd)++ = pm4_type3_packet(PM4_REG_TO_MEM, 2);
        *(*cmd)++ = i | (1<<30);
        *(*cmd)++ = ((drawctxt->gpustate.gpuaddr + REG_OFFSET) & 0xFFFFE000) + (i-0x2000)*4;
    }
}

#endif

//////////////////////////////////////////////////////////////////////////////
// chicken restore
//////////////////////////////////////////////////////////////////////////////
static unsigned int*
build_chicken_restore_cmds(gsl_drawctxt_t *drawctxt, ctx_t *ctx)
{
    unsigned int *start = ctx->cmd;
    unsigned int *cmds = start;

    *cmds++ = pm4_type3_packet(PM4_WAIT_FOR_IDLE, 1);
    *cmds++ = 0;

    *cmds++ = pm4_type0_packet(mmTP0_CHICKEN, 1);
    ctx->chicken_restore = gpuaddr(cmds, &drawctxt->gpustate);
    *cmds++ = 0x00000000;


    // create indirect buffer command for above command sequence
    create_ib1(drawctxt, drawctxt->chicken_restore, start, cmds);

    return cmds;
}



//////////////////////////////////////////////////////////////////////////////
// context save
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
// save h/w regs, alu constants, texture contants, etc. ...
//   requires: bool_shadow_gpuaddr, loop_shadow_gpuaddr
//////////////////////////////////////////////////////////////////////////////

static void
build_regsave_cmds(gsl_drawctxt_t *drawctxt, ctx_t *ctx)
{
    unsigned int *start = ctx->cmd;
    unsigned int *cmd = start;

#ifdef DISABLE_SHADOW_WRITES
    // Write HW registers into shadow
    build_reg_to_mem_range(mmRB_SURFACE_INFO, mmRB_DEPTH_INFO, &cmd, drawctxt);
    build_reg_to_mem_range(mmCOHER_DEST_BASE_0, mmPA_SC_SCREEN_SCISSOR_BR, &cmd, drawctxt);
    build_reg_to_mem_range(mmPA_SC_WINDOW_OFFSET, mmPA_SC_WINDOW_SCISSOR_BR, &cmd, drawctxt);
    build_reg_to_mem_range(mmVGT_MAX_VTX_INDX, mmRB_FOG_COLOR, &cmd, drawctxt);
    build_reg_to_mem_range(mmRB_STENCILREFMASK_BF, mmPA_CL_VPORT_ZOFFSET, &cmd, drawctxt);
    build_reg_to_mem_range(mmSQ_PROGRAM_CNTL, mmSQ_WRAPPING_1, &cmd, drawctxt);
    build_reg_to_mem_range(mmRB_DEPTHCONTROL, mmRB_MODECONTROL, &cmd, drawctxt);
    build_reg_to_mem_range(mmPA_SU_POINT_SIZE, mmPA_SC_LINE_STIPPLE, &cmd, drawctxt);
    build_reg_to_mem_range(mmPA_SC_VIZ_QUERY, mmPA_SC_VIZ_QUERY, &cmd, drawctxt);
    build_reg_to_mem_range(mmPA_SC_LINE_CNTL, mmSQ_PS_CONST, &cmd, drawctxt);
    build_reg_to_mem_range(mmPA_SC_AA_MASK, mmPA_SC_AA_MASK, &cmd, drawctxt);
    build_reg_to_mem_range(mmVGT_VERTEX_REUSE_BLOCK_CNTL, mmRB_DEPTH_CLEAR, &cmd, drawctxt);
    build_reg_to_mem_range(mmRB_SAMPLE_COUNT_CTL, mmRB_COLOR_DEST_MASK, &cmd, drawctxt);
    build_reg_to_mem_range(mmPA_SU_POLY_OFFSET_FRONT_SCALE, mmPA_SU_POLY_OFFSET_BACK_OFFSET, &cmd, drawctxt);

    // Copy ALU constants
    cmd = reg_to_mem(cmd, (drawctxt->gpustate.gpuaddr) & 0xFFFFE000, mmSQ_CONSTANT_0, ALU_CONSTANTS);

    // Copy Tex constants
    cmd = reg_to_mem(cmd, (drawctxt->gpustate.gpuaddr + TEX_OFFSET) & 0xFFFFE000, mmSQ_FETCH_0, TEX_CONSTANTS);
#else
    // H/w registers are already shadowed; just need to disable shadowing to prevent corruption.
    *cmd++ = pm4_type3_packet(PM4_LOAD_CONSTANT_CONTEXT, 3);
    *cmd++ = (drawctxt->gpustate.gpuaddr + REG_OFFSET) & 0xFFFFE000;
    *cmd++ = 4 << 16;                       // regs, start=0
    *cmd++ = 0x0;                           // count = 0

    // ALU constants are already shadowed; just need to disable shadowing to prevent corruption.
    *cmd++ = pm4_type3_packet(PM4_LOAD_CONSTANT_CONTEXT, 3);
    *cmd++ = drawctxt->gpustate.gpuaddr & 0xFFFFE000;
    *cmd++ = 0 << 16;                       // ALU, start=0
    *cmd++ = 0x0;                           // count = 0

    // Tex constants are already shadowed; just need to disable shadowing to prevent corruption.
    *cmd++ = pm4_type3_packet(PM4_LOAD_CONSTANT_CONTEXT, 3);
    *cmd++ = (drawctxt->gpustate.gpuaddr + TEX_OFFSET) & 0xFFFFE000;
    *cmd++ = 1 << 16;                       // Tex, start=0
    *cmd++ = 0x0;                           // count = 0
#endif




    // Need to handle some of the registers separately
    *cmd++ = pm4_type3_packet(PM4_REG_TO_MEM, 2);
    *cmd++ = mmSQ_GPR_MANAGEMENT;
    *cmd++ = ctx->reg_values[0];
    *cmd++ = pm4_type3_packet(PM4_REG_TO_MEM, 2);
    *cmd++ = mmTP0_CHICKEN;
    *cmd++ = ctx->reg_values[1];

    // Copy Boolean constants
    cmd = reg_to_mem(cmd, ctx->bool_shadow, mmSQ_CF_BOOLEANS, BOOL_CONSTANTS);

    // Copy Loop constants
    cmd = reg_to_mem(cmd, ctx->loop_shadow, mmSQ_CF_LOOP, LOOP_CONSTANTS);

    // create indirect buffer command for above command sequence
    create_ib1(drawctxt, drawctxt->reg_save, start, cmd);

    ctx->cmd = cmd;
}


//////////////////////////////////////////////////////////////////////////////
// copy colour, depth, & stencil buffers from graphics memory to system memory
//////////////////////////////////////////////////////////////////////////////

static unsigned int*
build_gmem2sys_cmds(gsl_drawctxt_t *drawctxt, ctx_t* ctx, gmem_shadow_t *shadow)
{
    unsigned int *cmds = shadow->gmem_save_commands;
    unsigned int *start = cmds;

    // Store TP0_CHICKEN register
    *cmds++ = pm4_type3_packet(PM4_REG_TO_MEM, 2);
    *cmds++ = mmTP0_CHICKEN;
    if( ctx )
        *cmds++ = ctx->chicken_restore;
    else
        cmds++;

    *cmds++ = pm4_type3_packet(PM4_WAIT_FOR_IDLE, 1);
    *cmds++ = 0;

    // Set TP0_CHICKEN to zero
    *cmds++ = pm4_type0_packet(mmTP0_CHICKEN, 1);
    *cmds++ = 0x00000000;

    // --------------
    // program shader
    // --------------

    // load shader vtx constants ... 5 dwords
    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 4);
    *cmds++ = (0x1 << 16) | SHADER_CONST_ADDR;
    *cmds++ = 0;
    *cmds++ = shadow->quad_vertices.gpuaddr | 0x3;      // valid(?) vtx constant flag & addr
    *cmds++ = 0x00000030;                   // limit = 12 dwords

    // Invalidate L2 cache to make sure vertices are updated
    *cmds++ = pm4_type0_packet(mmTC_CNTL_STATUS, 1);
    *cmds++ = 0x1;

    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 4);
    *cmds++ = PM4_REG(mmVGT_MAX_VTX_INDX);
    *cmds++ = 0x00ffffff; //mmVGT_MAX_VTX_INDX
    *cmds++ = 0x0;        //mmVGT_MIN_VTX_INDX
    *cmds++ = 0x00000000; //mmVGT_INDX_OFFSET

    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
    *cmds++ = PM4_REG(mmPA_SC_AA_MASK);
    *cmds++ = 0x0000ffff; //mmPA_SC_AA_MASK


    // load the patched vertex shader stream
    cmds = program_shader(cmds, 0, gmem2sys_vtx_pgm, GMEM2SYS_VTX_PGM_LEN);

    // Load the patched fragment shader stream
    cmds = program_shader(cmds, 1, gmem2sys_frag_pgm, GMEM2SYS_FRAG_PGM_LEN);

    // SQ_PROGRAM_CNTL / SQ_CONTEXT_MISC
    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 3);
    *cmds++ = PM4_REG(mmSQ_PROGRAM_CNTL);
    *cmds++ = 0x10010001;
    *cmds++ = 0x00000008;


    // --------------
    // resolve
    // --------------

    // PA_CL_VTE_CNTL
    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
    *cmds++ = PM4_REG(mmPA_CL_VTE_CNTL);
    *cmds++ = 0x00000b00;               // disable X/Y/Z transforms, X/Y/Z are premultiplied by W

    // change colour buffer to RGBA8888, MSAA = 1, and matching pitch
    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 3);
    *cmds++ = PM4_REG(mmRB_SURFACE_INFO);
    *cmds++ = shadow->gmem_pitch;

    // RB_COLOR_INFO        Endian=none, Linear, Format=RGBA8888, Swap=0, Base=gmem_base
    if( ctx )
    {
        KOS_ASSERT((ctx->gmem_base & 0xFFF) == 0);   // gmem base assumed 4K aligned.
        *cmds++ = (shadow->format << RB_COLOR_INFO__COLOR_FORMAT__SHIFT) | ctx->gmem_base;
    }
    else
    {
		unsigned int temp = *cmds;
		*cmds++ = (temp & ~RB_COLOR_INFO__COLOR_FORMAT_MASK) | (shadow->format << RB_COLOR_INFO__COLOR_FORMAT__SHIFT);
    }

    // disable Z
    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
    *cmds++ = PM4_REG(mmRB_DEPTHCONTROL);
    *cmds++ = 0;

    // set mmPA_SU_SC_MODE_CNTL
    //      Front_ptype = draw triangles
    //      Back_ptype = draw triangles
    //      Provoking vertex = last
    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
    *cmds++ = PM4_REG(mmPA_SU_SC_MODE_CNTL);
    *cmds++ = 0x00080240;

    // set the scissor to the extents of the draw surface
    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 3);
    *cmds++ = PM4_REG(mmPA_SC_SCREEN_SCISSOR_TL);
    *cmds++ = (shadow->gmem_offset_y << 16) | shadow->gmem_offset_x;
    *cmds++ = (shadow->gmem_height << 16) | shadow->gmem_width;
    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 3);
    *cmds++ = PM4_REG(mmPA_SC_WINDOW_SCISSOR_TL);
    *cmds++ = (1U << 31) | (0 << 16) | 0;
    *cmds++ = (shadow->height << 16) | shadow->width;

    // load the viewport so that z scale = clear depth and z offset = 0.0f
    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 3);
    *cmds++ = PM4_REG(mmPA_CL_VPORT_ZSCALE);
    *cmds++ = 0xbf800000; // -1.0f
    *cmds++ = 0x0;

    // load the COPY state
    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 6);
    *cmds++ = PM4_REG(mmRB_COPY_CONTROL);
    *cmds++ = 0;                                                // RB_COPY_CONTROL

    {
        // Calculate the new offset based on the adjusted base
		unsigned int bytesperpixel = format2bytesperpixel[shadow->format];
		unsigned int addr = (shadow->gmemshadow.gpuaddr + shadow->offset * bytesperpixel);
		unsigned int offset = (addr - (addr & 0xfffff000)) / bytesperpixel;

		*cmds++ = addr & 0xfffff000;   // RB_COPY_DEST_BASE
		*cmds++ = shadow->pitch >> 5;  // RB_COPY_DEST_PITCH
		*cmds++ = 0x0003c008 | (shadow->format << RB_COPY_DEST_INFO__COPY_DEST_FORMAT__SHIFT); // Endian=none, Linear, Format=RGBA8888,Swap=0,!Dither,MaskWrite:R=G=B=A=1

        KOS_ASSERT( (offset & 0xfffff000) == 0 ); // Make sure we stay in offsetx field.
		*cmds++ = offset;
    }

    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
    *cmds++ = PM4_REG(mmRB_MODECONTROL);
    *cmds++ = 0x6;                          // EDRAM copy

    // queue the draw packet
    *cmds++ = pm4_type3_packet(PM4_DRAW_INDX, 2);
    *cmds++ = 0;                            // viz query info.
    *cmds++ = 0x00030088;                   // PrimType=RectList, NumIndices=3, SrcSel=AutoIndex

    // create indirect buffer command for above command sequence
    create_ib1(drawctxt, shadow->gmem_save, start, cmds);

    return cmds;
}


//////////////////////////////////////////////////////////////////////////////
// context restore
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
// copy colour, depth, & stencil buffers from system memory to graphics memory
//////////////////////////////////////////////////////////////////////////////

static unsigned int*
build_sys2gmem_cmds(gsl_drawctxt_t *drawctxt, ctx_t* ctx, gmem_shadow_t *shadow)
{
    unsigned int *cmds = shadow->gmem_restore_commands;
    unsigned int *start = cmds;

    // Store TP0_CHICKEN register
    *cmds++ = pm4_type3_packet(PM4_REG_TO_MEM, 2);
    *cmds++ = mmTP0_CHICKEN;
    if( ctx )
        *cmds++ = ctx->chicken_restore;
    else
        cmds++;

    *cmds++ = pm4_type3_packet(PM4_WAIT_FOR_IDLE, 1);
    *cmds++ = 0;

    // Set TP0_CHICKEN to zero
    *cmds++ = pm4_type0_packet(mmTP0_CHICKEN, 1);
    *cmds++ = 0x00000000;

    // ----------------
    // shader constants
    // ----------------

    // vertex buffer constants
    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 7);

    *cmds++ = (0x1 << 16) | (9 * 6);
    *cmds++ = shadow->quad_vertices.gpuaddr | 0x3;          // valid(?) vtx constant flag & addr
    *cmds++ = 0x00000030;                       // limit = 12 dwords
    *cmds++ = shadow->quad_texcoords.gpuaddr | 0x3;         // valid(?) vtx constant flag & addr
    *cmds++ = 0x00000020;                       // limit = 8 dwords
    *cmds++ = 0;
    *cmds++ = 0;

    // Invalidate L2 cache to make sure vertices and texture coordinates are updated
    *cmds++ = pm4_type0_packet(mmTC_CNTL_STATUS, 1);
    *cmds++ = 0x1;

    // load the patched vertex shader stream
    cmds = program_shader(cmds, 0, sys2gmem_vtx_pgm, SYS2GMEM_VTX_PGM_LEN);

    // Load the patched fragment shader stream
    cmds = program_shader(cmds, 1, sys2gmem_frag_pgm, SYS2GMEM_FRAG_PGM_LEN);

    // SQ_PROGRAM_CNTL / SQ_CONTEXT_MISC
    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 3);
    *cmds++ = PM4_REG(mmSQ_PROGRAM_CNTL);
    *cmds++ = 0x10030002;
    *cmds++ = 0x00000008;


    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
    *cmds++ = PM4_REG(mmPA_SC_AA_MASK);
    *cmds++ = 0x0000ffff; //mmPA_SC_AA_MASK

    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
    *cmds++ = PM4_REG(mmPA_SC_VIZ_QUERY);
    *cmds++ = 0x0;        //mmPA_SC_VIZ_QUERY


    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
    *cmds++ = PM4_REG(mmRB_COLORCONTROL);
    *cmds++ = 0x00000c20; // RB_COLORCONTROL

    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 4);
    *cmds++ = PM4_REG(mmVGT_MAX_VTX_INDX);
    *cmds++ = 0x00ffffff; //mmVGT_MAX_VTX_INDX
    *cmds++ = 0x0;        //mmVGT_MIN_VTX_INDX
    *cmds++ = 0x00000000; //mmVGT_INDX_OFFSET

    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 3);
    *cmds++ = PM4_REG(mmVGT_VERTEX_REUSE_BLOCK_CNTL);
    *cmds++ = 0x00000002; //mmVGT_VERTEX_REUSE_BLOCK_CNTL
    *cmds++ = 0x00000002; //mmVGT_OUT_DEALLOC_CNTL

    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
    *cmds++ = PM4_REG(mmSQ_INTERPOLATOR_CNTL);
    *cmds++ = 0xffffffff; //mmSQ_INTERPOLATOR_CNTL

    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
    *cmds++ = PM4_REG(mmPA_SC_AA_CONFIG);
    *cmds++ = 0x00000000; //mmPA_SC_AA_CONFIG


    // set mmPA_SU_SC_MODE_CNTL
    //      Front_ptype = draw triangles
    //      Back_ptype = draw triangles
    //      Provoking vertex = last
    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
    *cmds++ = PM4_REG(mmPA_SU_SC_MODE_CNTL);
    *cmds++ = 0x00080240;

    // texture constants
    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, (SYS2GMEM_TEX_CONST_LEN + 1));
    *cmds++ = (0x1 << 16) | (0 * 6);
    kos_memcpy(cmds, sys2gmem_tex_const, SYS2GMEM_TEX_CONST_LEN<<2);
    cmds[0] |= (shadow->pitch >> 5) << 22;
    cmds[1] |= shadow->gmemshadow.gpuaddr | surface_format_table[shadow->format];
    cmds[2] |= (shadow->width+shadow->offset_x-1) | (shadow->height+shadow->offset_y-1) << 13;
    cmds += SYS2GMEM_TEX_CONST_LEN;

    // change colour buffer to RGBA8888, MSAA = 1, and matching pitch
    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 3);
    *cmds++ = PM4_REG(mmRB_SURFACE_INFO);
    *cmds++ = shadow->gmem_pitch; // GMEM pitch is equal to context GMEM shadow pitch

    // RB_COLOR_INFO        Endian=none, Linear, Format=RGBA8888, Swap=0, Base=gmem_base
    if( ctx )
	{
        *cmds++ = (shadow->format << RB_COLOR_INFO__COLOR_FORMAT__SHIFT) | ctx->gmem_base;
	}
    else
	{
		unsigned int temp = *cmds;
		*cmds++ = (temp & ~RB_COLOR_INFO__COLOR_FORMAT_MASK) | (shadow->format << RB_COLOR_INFO__COLOR_FORMAT__SHIFT);
	}

    // RB_DEPTHCONTROL
    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
    *cmds++ = PM4_REG(mmRB_DEPTHCONTROL);
    *cmds++ = 0;                            // disable Z

    // set the scissor to the extents of the draw surface
    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 3);
    *cmds++ = PM4_REG(mmPA_SC_SCREEN_SCISSOR_TL);
    *cmds++ = (0 << 16) | 0;
    *cmds++ = (shadow->height << 16) | shadow->width;
    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 3);
    *cmds++ = PM4_REG(mmPA_SC_WINDOW_SCISSOR_TL);
    *cmds++ = (1U << 31) | (shadow->gmem_offset_y << 16) | shadow->gmem_offset_x;
    *cmds++ = (shadow->gmem_height << 16) | shadow->gmem_width;

    // PA_CL_VTE_CNTL
    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
    *cmds++ = PM4_REG(mmPA_CL_VTE_CNTL);
    *cmds++ = 0x00000b00;                   // disable X/Y/Z transforms, X/Y/Z are premultiplied by W

    // load the viewport so that z scale = clear depth and z offset = 0.0f
    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 3);
    *cmds++ = PM4_REG(mmPA_CL_VPORT_ZSCALE);
    *cmds++ = 0xbf800000;
    *cmds++ = 0x0;

    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
    *cmds++ = PM4_REG(mmRB_COLOR_MASK);
    *cmds++ = 0x0000000f;                   // R = G = B = 1:enabled

    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
    *cmds++ = PM4_REG(mmRB_COLOR_DEST_MASK);
    *cmds++ = 0xffffffff;

    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 3);
    *cmds++ = PM4_REG(mmSQ_WRAPPING_0);
    *cmds++ = 0x00000000;
    *cmds++ = 0x00000000;

    *cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
    *cmds++ = PM4_REG(mmRB_MODECONTROL);
    *cmds++ = 0x4;                          // draw pixels with color and depth/stencil component

    // queue the draw packet
    *cmds++ = pm4_type3_packet(PM4_DRAW_INDX, 2);
    *cmds++ = 0;                            // viz query info.
    *cmds++ = 0x00030088;                   // PrimType=RectList, NumIndices=3, SrcSel=AutoIndex

    // create indirect buffer command for above command sequence
    create_ib1(drawctxt, shadow->gmem_restore, start, cmds);

    return cmds;
}


//////////////////////////////////////////////////////////////////////////////
// restore h/w regs, alu constants, texture constants, etc. ...
//////////////////////////////////////////////////////////////////////////////

static unsigned *
reg_range(unsigned int *cmd, unsigned int start, unsigned int end)
{
    *cmd++ = PM4_REG(start);                // h/w regs, start addr
    *cmd++ = end - start + 1;               // count
    return cmd;
}


//////////////////////////////////////////////////////////////////////////////

static void
build_regrestore_cmds(gsl_drawctxt_t *drawctxt, ctx_t *ctx)
{
    unsigned int *start = ctx->cmd;
    unsigned int *cmd = start;

    // H/W Registers
    cmd++; // deferred pm4_type3_packet(PM4_LOAD_CONSTANT_CONTEXT, ???);
#ifdef DISABLE_SHADOW_WRITES
    *cmd++ = ((drawctxt->gpustate.gpuaddr + REG_OFFSET) & 0xFFFFE000) | 1; // Force mismatch
#else
    *cmd++ = (drawctxt->gpustate.gpuaddr + REG_OFFSET) & 0xFFFFE000;
#endif

    cmd = reg_range(cmd, mmRB_SURFACE_INFO,                 mmPA_SC_SCREEN_SCISSOR_BR);
    cmd = reg_range(cmd, mmPA_SC_WINDOW_OFFSET,             mmPA_SC_WINDOW_SCISSOR_BR);
    cmd = reg_range(cmd, mmVGT_MAX_VTX_INDX,                mmPA_CL_VPORT_ZOFFSET);
    cmd = reg_range(cmd, mmSQ_PROGRAM_CNTL,                 mmSQ_WRAPPING_1);
    cmd = reg_range(cmd, mmRB_DEPTHCONTROL,                 mmRB_MODECONTROL);
    cmd = reg_range(cmd, mmPA_SU_POINT_SIZE,                mmPA_SC_VIZ_QUERY);
    cmd = reg_range(cmd, mmPA_SC_LINE_CNTL,                 mmRB_COLOR_DEST_MASK);
    cmd = reg_range(cmd, mmPA_SU_POLY_OFFSET_FRONT_SCALE,   mmPA_SU_POLY_OFFSET_BACK_OFFSET);

    // Now we know how many register blocks we have, we can compute command length
    start[0] = pm4_type3_packet(PM4_LOAD_CONSTANT_CONTEXT, (cmd-start)-1);
#ifdef DISABLE_SHADOW_WRITES
    start[2] |= (0<<24) | (4 << 16);    // Disable shadowing.
#else
    start[2] |= (1<<24) | (4 << 16);    // Enable shadowing for the entire register block.
#endif

    // Need to handle some of the registers separately
    *cmd++ = pm4_type0_packet(mmSQ_GPR_MANAGEMENT, 1);
    ctx->reg_values[0] = gpuaddr(cmd, &drawctxt->gpustate);
    *cmd++ = 0x00040400;

    *cmd++ = pm4_type3_packet(PM4_WAIT_FOR_IDLE, 1);
    *cmd++ = 0;
    *cmd++ = pm4_type0_packet(mmTP0_CHICKEN, 1);
    ctx->reg_values[1] = gpuaddr(cmd, &drawctxt->gpustate);
    *cmd++ = 0x00000000;

    // ALU Constants
    *cmd++ = pm4_type3_packet(PM4_LOAD_CONSTANT_CONTEXT, 3);
    *cmd++ = drawctxt->gpustate.gpuaddr & 0xFFFFE000;
#ifdef DISABLE_SHADOW_WRITES
    *cmd++ = (0<<24) | (0<<16) | 0; // Disable shadowing
#else
    *cmd++ = (1<<24) | (0<<16) | 0;
#endif
    *cmd++ = ALU_CONSTANTS;


    // Texture Constants
    *cmd++ = pm4_type3_packet(PM4_LOAD_CONSTANT_CONTEXT, 3);
    *cmd++ = (drawctxt->gpustate.gpuaddr + TEX_OFFSET) & 0xFFFFE000;
#ifdef DISABLE_SHADOW_WRITES
    *cmd++ = (0<<24) | (1<<16) | 0; // Disable shadowing
#else
    *cmd++ = (1<<24) | (1<<16) | 0;
#endif
    *cmd++ = TEX_CONSTANTS;


    // Boolean Constants
    *cmd++ = pm4_type3_packet(PM4_SET_CONSTANT, 1 + BOOL_CONSTANTS);
    *cmd++ = (2<<16) | 0;

    // the next BOOL_CONSTANT dwords is the shadow area for boolean constants.
    ctx->bool_shadow = gpuaddr(cmd, &drawctxt->gpustate);
    cmd += BOOL_CONSTANTS;


    // Loop Constants
    *cmd++ = pm4_type3_packet(PM4_SET_CONSTANT, 1 + LOOP_CONSTANTS);
    *cmd++ = (3<<16) | 0;

    // the next LOOP_CONSTANTS dwords is the shadow area for loop constants.
    ctx->loop_shadow = gpuaddr(cmd, &drawctxt->gpustate);
    cmd += LOOP_CONSTANTS;

    // create indirect buffer command for above command sequence
    create_ib1(drawctxt, drawctxt->reg_restore, start, cmd);

    ctx->cmd = cmd;
}


//////////////////////////////////////////////////////////////////////////////
// quad for saving/restoring gmem
//////////////////////////////////////////////////////////////////////////////

static void set_gmem_copy_quad( gmem_shadow_t* shadow )
{
    unsigned int tex_offset[2];

    // set vertex buffer values

    gmem_copy_quad[1] = uint2float( shadow->gmem_height + shadow->gmem_offset_y );
    gmem_copy_quad[3] = uint2float( shadow->gmem_width + shadow->gmem_offset_x );
    gmem_copy_quad[4] = uint2float( shadow->gmem_height + shadow->gmem_offset_y );
    gmem_copy_quad[9] = uint2float( shadow->gmem_width + shadow->gmem_offset_x );

    gmem_copy_quad[0] = uint2float( shadow->gmem_offset_x );
    gmem_copy_quad[6] = uint2float( shadow->gmem_offset_x );
    gmem_copy_quad[7] = uint2float( shadow->gmem_offset_y );
    gmem_copy_quad[10] = uint2float( shadow->gmem_offset_y );

    tex_offset[0] = uintdivide( shadow->offset_x, (shadow->offset_x+shadow->width) );
    tex_offset[1] = uintdivide( shadow->offset_y, (shadow->offset_y+shadow->height) );

    gmem_copy_texcoord[0] = gmem_copy_texcoord[4] = tex_offset[0];
    gmem_copy_texcoord[5] = gmem_copy_texcoord[7] = tex_offset[1];

    // copy quad data to vertex buffer
    kos_memcpy(shadow->quad_vertices.hostptr, gmem_copy_quad, QUAD_LEN << 2);

    // copy tex coord data to tex coord buffer
    kos_memcpy(shadow->quad_texcoords.hostptr, gmem_copy_texcoord, TEXCOORD_LEN << 2);
}


static void
build_quad_vtxbuff(gsl_drawctxt_t *drawctxt, ctx_t *ctx, gmem_shadow_t* shadow)
{
    unsigned int *cmd = ctx->cmd;

    // quad vertex buffer location
    shadow->quad_vertices.hostptr = cmd;
    shadow->quad_vertices.gpuaddr = gpuaddr(cmd, &drawctxt->gpustate);
    cmd += QUAD_LEN;

    // tex coord buffer location (in GPU space)
    shadow->quad_texcoords.hostptr = cmd;
    shadow->quad_texcoords.gpuaddr = gpuaddr(cmd, &drawctxt->gpustate);


    cmd += TEXCOORD_LEN;

    set_gmem_copy_quad(shadow);


    ctx->cmd = cmd;
}


//////////////////////////////////////////////////////////////////////////////

static void
build_shader_save_restore_cmds(gsl_drawctxt_t *drawctxt, ctx_t *ctx)
{
    unsigned int *cmd = ctx->cmd;
    unsigned int *save, *restore, *fixup;
#if defined(PM4_IM_STORE)
    unsigned int *startSizeVtx, *startSizePix, *startSizeShared;
#endif
    unsigned int *partition1;
    unsigned int *shaderBases, *partition2;

#if defined(PM4_IM_STORE)
    // compute vertex, pixel and shared instruction shadow GPU addresses
    ctx->shader_vertex = drawctxt->gpustate.gpuaddr + SHADER_OFFSET;
    ctx->shader_pixel  = ctx->shader_vertex + SHADER_SHADOW_SIZE;
    ctx->shader_shared = ctx->shader_pixel  + SHADER_SHADOW_SIZE;
#endif


    //-------------------------------------------------------------------
    // restore shader partitioning and instructions
    //-------------------------------------------------------------------

    restore = cmd;  // start address

    // Invalidate Vertex & Pixel instruction code address and sizes
    *cmd++ = pm4_type3_packet(PM4_INVALIDATE_STATE, 1);
    *cmd++ = 0x00000300;                        // 0x100 = Vertex, 0x200 = Pixel

    // Restore previous shader vertex & pixel instruction bases.
    *cmd++ = pm4_type3_packet(PM4_SET_SHADER_BASES, 1);
    shaderBases = cmd++;                        // TBD #5: shader bases (from fixup)

    // write the shader partition information to a scratch register
    *cmd++ = pm4_type0_packet(mmSQ_INST_STORE_MANAGMENT, 1);
    partition1 = cmd++;                         // TBD #4a: partition info (from save)

#if defined(PM4_IM_STORE)
    // load vertex shader instructions from the shadow.
    *cmd++ = pm4_type3_packet(PM4_IM_LOAD, 2);
    *cmd++ = ctx->shader_vertex + 0x0;          // 0x0 = Vertex
    startSizeVtx = cmd++;                       // TBD #1: start/size (from save)

    // load pixel shader instructions from the shadow.
    *cmd++ = pm4_type3_packet(PM4_IM_LOAD, 2);
    *cmd++ = ctx->shader_pixel + 0x1;           // 0x1 = Pixel
    startSizePix = cmd++;                       // TBD #2: start/size (from save)

    // load shared shader instructions from the shadow.
    *cmd++ = pm4_type3_packet(PM4_IM_LOAD, 2);
    *cmd++ = ctx->shader_shared + 0x2;          // 0x2 = Shared
    startSizeShared = cmd++;                    // TBD #3: start/size (from save)
#endif

    // create indirect buffer command for above command sequence
    create_ib1(drawctxt, drawctxt->shader_restore, restore, cmd);


    //-------------------------------------------------------------------
    // fixup SET_SHADER_BASES data
    //
    // since self-modifying PM4 code is being used here, a seperate
    // command buffer is used for this fixup operation, to ensure the
    // commands are not read by the PM4 engine before the data fields
    // have been written.
    //-------------------------------------------------------------------

    fixup = cmd;        // start address

    // write the shader partition information to a scratch register
    *cmd++ = pm4_type0_packet(mmSCRATCH_REG2, 1);
    partition2 = cmd++;                                 // TBD #4b: partition info (from save)

    // mask off unused bits, then OR with shader instruction memory size
    *cmd++ = pm4_type3_packet(PM4_REG_RMW, 3);
    *cmd++ = mmSCRATCH_REG2;
    *cmd++ = 0x0FFF0FFF;                                // AND off invalid bits.
    *cmd++ = (unsigned int)((SHADER_INSTRUCT_LOG2-5U) << 29);               // OR in instruction memory size

    // write the computed value to the SET_SHADER_BASES data field
    *cmd++ = pm4_type3_packet(PM4_REG_TO_MEM, 2);
    *cmd++ = mmSCRATCH_REG2;
    *cmd++ = gpuaddr(shaderBases, &drawctxt->gpustate); // TBD #5: shader bases (to restore)

    // create indirect buffer command for above command sequence
    create_ib1(drawctxt, drawctxt->shader_fixup, fixup, cmd);


    //-------------------------------------------------------------------
    // save shader partitioning and instructions
    //-------------------------------------------------------------------

    save = cmd;     // start address

    *cmd++ = pm4_type3_packet(PM4_WAIT_FOR_IDLE, 1);
    *cmd++ = 0;

    // Fetch the SQ_INST_STORE_MANAGMENT register value,
    // Store the value in the data fields of the SET_CONSTANT commands above.
    *cmd++ = pm4_type3_packet(PM4_REG_TO_MEM, 2);
    *cmd++ = mmSQ_INST_STORE_MANAGMENT;
    *cmd++ = gpuaddr(partition1, &drawctxt->gpustate);      // TBD #4a: partition info (to restore)
    *cmd++ = pm4_type3_packet(PM4_REG_TO_MEM, 2);
    *cmd++ = mmSQ_INST_STORE_MANAGMENT;
    *cmd++ = gpuaddr(partition2, &drawctxt->gpustate);      // TBD #4b: partition info (to fixup)

#if defined(PM4_IM_STORE)
    // Store the vertex shader instructions
    *cmd++ = pm4_type3_packet(PM4_IM_STORE, 2);
    *cmd++ = ctx->shader_vertex + 0x0;                      // 0x0 = Vertex
    *cmd++ = gpuaddr(startSizeVtx, &drawctxt->gpustate);    // TBD #1: start/size (to restore)

    // store the pixel shader instructions
    *cmd++ = pm4_type3_packet(PM4_IM_STORE, 2);
    *cmd++ = ctx->shader_pixel + 0x1;                       // 0x1 = Pixel
    *cmd++ = gpuaddr(startSizePix, &drawctxt->gpustate);    // TBD #2: start/size (to restore)

    // Store the shared shader instructions
    *cmd++ = pm4_type3_packet(PM4_IM_STORE, 2);
    *cmd++ = ctx->shader_shared + 0x2;                      // 0x2 = Shared
    *cmd++ = gpuaddr(startSizeShared, &drawctxt->gpustate); // TBD #3: start/size (to restore)
#endif

    *cmd++ = pm4_type3_packet(PM4_WAIT_FOR_IDLE, 1);
    *cmd++ = 0;



    // Create indirect buffer command for above command sequence
    create_ib1(drawctxt, drawctxt->shader_save, save, cmd);


    ctx->cmd = cmd;
}



//////////////////////////////////////////////////////////////////////////////
// create buffers for saving/restoring registers and constants
//////////////////////////////////////////////////////////////////////////////

static int
create_gpustate_shadow(gsl_device_t *device, gsl_drawctxt_t *drawctxt, ctx_t *ctx)
{
    gsl_flags_t flags;

    flags = (GSL_MEMFLAGS_CONPHYS | GSL_MEMFLAGS_ALIGN8K);
    KGSL_DEBUG(GSL_DBGFLAGS_DUMPX, flags = (GSL_MEMFLAGS_EMEM | GSL_MEMFLAGS_ALIGN8K));

    // allocate memory to allow HW to save sub-blocks for efficient context save/restore
    if (kgsl_sharedmem_alloc0(device->id, flags, CONTEXT_SIZE, &drawctxt->gpustate) != GSL_SUCCESS)
        return GSL_FAILURE;

    drawctxt->flags |= CTXT_FLAGS_STATE_SHADOW;

    // Blank out h/w register, constant, and command buffer shadows.
    kgsl_sharedmem_set0(&drawctxt->gpustate, 0, 0, CONTEXT_SIZE);

    // set-up command and vertex buffer pointers
    ctx->cmd = ctx->start = (unsigned int *) ((char *)drawctxt->gpustate.hostptr + CMD_OFFSET);

    // build indirect command buffers to save & restore regs/constants
    build_regrestore_cmds(drawctxt, ctx);
    build_regsave_cmds(drawctxt, ctx);

    build_shader_save_restore_cmds(drawctxt, ctx);

    return GSL_SUCCESS;
}


//////////////////////////////////////////////////////////////////////////////
// Allocate GMEM shadow buffer
//////////////////////////////////////////////////////////////////////////////
static int
allocate_gmem_shadow_buffer(gsl_device_t *device, gsl_drawctxt_t *drawctxt)
{
    // allocate memory for GMEM shadow
    if (kgsl_sharedmem_alloc0(device->id, (GSL_MEMFLAGS_CONPHYS | GSL_MEMFLAGS_ALIGN8K),
        drawctxt->context_gmem_shadow.size, &drawctxt->context_gmem_shadow.gmemshadow) != GSL_SUCCESS)
        return GSL_FAILURE;

    // blank out gmem shadow.
    kgsl_sharedmem_set0(&drawctxt->context_gmem_shadow.gmemshadow, 0, 0, drawctxt->context_gmem_shadow.size);

    return GSL_SUCCESS;
}


//////////////////////////////////////////////////////////////////////////////
// create GMEM save/restore specific stuff
//////////////////////////////////////////////////////////////////////////////

static int
create_gmem_shadow(gsl_device_t *device, gsl_drawctxt_t *drawctxt, ctx_t *ctx)
{
    unsigned int i;
    config_gmemsize(&drawctxt->context_gmem_shadow, device->gmemspace.sizebytes);
    ctx->gmem_base = device->gmemspace.gpu_base;

    if( drawctxt->flags & CTXT_FLAGS_GMEM_SHADOW )
    {
        if( allocate_gmem_shadow_buffer(device, drawctxt) != GSL_SUCCESS )
            return GSL_FAILURE;
    }
    else
    {
        kos_memset( &drawctxt->context_gmem_shadow.gmemshadow, 0, sizeof( gsl_memdesc_t ) );
    }

    // build quad vertex buffer
    build_quad_vtxbuff(drawctxt, ctx, &drawctxt->context_gmem_shadow);

    // build TP0_CHICKEN register restore command buffer
    ctx->cmd = build_chicken_restore_cmds(drawctxt, ctx);

    // build indirect command buffers to save & restore gmem
    drawctxt->context_gmem_shadow.gmem_save_commands = ctx->cmd;
    ctx->cmd = build_gmem2sys_cmds(drawctxt, ctx, &drawctxt->context_gmem_shadow);
    drawctxt->context_gmem_shadow.gmem_restore_commands = ctx->cmd;
    ctx->cmd = build_sys2gmem_cmds(drawctxt, ctx, &drawctxt->context_gmem_shadow);

    for( i = 0; i < GSL_MAX_GMEM_SHADOW_BUFFERS; i++ )
    {
        // build quad vertex buffer
        build_quad_vtxbuff(drawctxt, ctx, &drawctxt->user_gmem_shadow[i]);

        // build indirect command buffers to save & restore gmem
        drawctxt->user_gmem_shadow[i].gmem_save_commands = ctx->cmd;
        ctx->cmd = build_gmem2sys_cmds(drawctxt, ctx, &drawctxt->user_gmem_shadow[i]);

        drawctxt->user_gmem_shadow[i].gmem_restore_commands = ctx->cmd;
        ctx->cmd = build_sys2gmem_cmds(drawctxt, ctx, &drawctxt->user_gmem_shadow[i]);
    }

    return GSL_SUCCESS;
}


//////////////////////////////////////////////////////////////////////////////
// init draw context
//////////////////////////////////////////////////////////////////////////////

int
kgsl_drawctxt_init(gsl_device_t *device)
{
    GSL_CONTEXT_MUTEX_CREATE();

    return (GSL_SUCCESS);
}


//////////////////////////////////////////////////////////////////////////////
// close draw context
//////////////////////////////////////////////////////////////////////////////

int
kgsl_drawctxt_close(gsl_device_t *device)
{
    GSL_CONTEXT_MUTEX_FREE();

    return (GSL_SUCCESS);
}


//////////////////////////////////////////////////////////////////////////////
// create a new drawing context
//////////////////////////////////////////////////////////////////////////////

int
kgsl_drawctxt_create(gsl_device_t* device, gsl_context_type_t type, unsigned int *drawctxt_id, gsl_flags_t flags)
{
    gsl_drawctxt_t  *drawctxt;
    int             index;
    ctx_t           ctx;

    kgsl_device_active(device);
    
    GSL_CONTEXT_MUTEX_LOCK();
    if (device->drawctxt_count >= GSL_CONTEXT_MAX)
    {
        GSL_CONTEXT_MUTEX_UNLOCK();
        return (GSL_FAILURE);
    }

    // find a free context slot
    index = 0;
    while (index < GSL_CONTEXT_MAX)
    {
        if (device->drawctxt[index].flags == CTXT_FLAGS_NOT_IN_USE)
            break;

        index++;
    }

    if (index >= GSL_CONTEXT_MAX)
    {
        GSL_CONTEXT_MUTEX_UNLOCK();
        return (GSL_FAILURE);
    }

    drawctxt = &device->drawctxt[index];

    kos_memset( &drawctxt->context_gmem_shadow, 0, sizeof( gmem_shadow_t ) );

	drawctxt->pid   = GSL_CALLER_PROCESSID_GET();
    drawctxt->flags = CTXT_FLAGS_IN_USE;
    drawctxt->type  = type;

    device->drawctxt_count++;

    // create context shadows, when not running in safe mode
    if (!(device->flags & GSL_FLAGS_SAFEMODE))
    {
        if (create_gpustate_shadow(device, drawctxt, &ctx) != GSL_SUCCESS)
        {
            kgsl_drawctxt_destroy(device, index);
            GSL_CONTEXT_MUTEX_UNLOCK();
            return (GSL_FAILURE);
        }

        // Save the shader instruction memory & GMEM on context switching
        drawctxt->flags |= ( CTXT_FLAGS_SHADER_SAVE | CTXT_FLAGS_GMEM_SHADOW );

        // Clear out user defined GMEM shadow buffer structs
        kos_memset( drawctxt->user_gmem_shadow, 0, sizeof(gmem_shadow_t)*GSL_MAX_GMEM_SHADOW_BUFFERS );

        // create gmem shadow
        if (create_gmem_shadow(device, drawctxt, &ctx) != GSL_SUCCESS)
        {
            kgsl_drawctxt_destroy(device, index);
            GSL_CONTEXT_MUTEX_UNLOCK();
            return (GSL_FAILURE);
        }

        KOS_ASSERT(ctx.cmd - ctx.start <= CMD_BUFFER_LEN);
    }

    *drawctxt_id = index;

    GSL_CONTEXT_MUTEX_UNLOCK();
    return (GSL_SUCCESS);
}


//////////////////////////////////////////////////////////////////////////////
// destroy a drawing context
//////////////////////////////////////////////////////////////////////////////

int
kgsl_drawctxt_destroy(gsl_device_t* device, unsigned int drawctxt_id)
{
    gsl_drawctxt_t *drawctxt;

    GSL_CONTEXT_MUTEX_LOCK();

    drawctxt = &device->drawctxt[drawctxt_id];

    if (drawctxt->flags != CTXT_FLAGS_NOT_IN_USE)
    {
        // deactivate context
        if (device->drawctxt_active == drawctxt)
        {
            // no need to save GMEM or shader, the context is being destroyed.
            drawctxt->flags &= ~(CTXT_FLAGS_GMEM_SAVE | CTXT_FLAGS_SHADER_SAVE);

            kgsl_drawctxt_switch(device, GSL_CONTEXT_NONE, 0);
        }

        device->ftbl.device_idle(device, GSL_TIMEOUT_DEFAULT);

        // destroy state shadow, if allocated
        if (drawctxt->flags & CTXT_FLAGS_STATE_SHADOW)
            kgsl_sharedmem_free0(&drawctxt->gpustate, GSL_CALLER_PROCESSID_GET());


        // destroy gmem shadow, if allocated
        if (drawctxt->context_gmem_shadow.gmemshadow.size > 0)
        {
            kgsl_sharedmem_free0(&drawctxt->context_gmem_shadow.gmemshadow, GSL_CALLER_PROCESSID_GET());
            drawctxt->context_gmem_shadow.gmemshadow.size = 0;
        }

        drawctxt->flags = CTXT_FLAGS_NOT_IN_USE;
		drawctxt->pid   = 0;

        device->drawctxt_count--;
        KOS_ASSERT(device->drawctxt_count >= 0);
    }

    GSL_CONTEXT_MUTEX_UNLOCK();

    return (GSL_SUCCESS);
}

//////////////////////////////////////////////////////////////////////////////
// Binds a user specified buffer as GMEM shadow area
//
// gmem_rect:   defines the rectangle that is copied from GMEM. X and Y
//              coordinates need to be multiples of 8 after conversion to 32bpp.
//              X, Y, width, and height need to be at 32-bit boundary to avoid
//              rounding.
//
// shadow_x & shadow_y: Position in GMEM shadow buffer where the contents of
//                      gmem_rect is copied. Both must be multiples of 8 after
//                      conversion to 32bpp. They also need to be at 32-bit
//                      boundary to avoid rounding.
//
// shadow_buffer:   Description of the GMEM shadow buffer. BPP needs to be
//                  8, 16, 32, 64, or 128. Enabled tells if the buffer is
//                  used or not (values 0 and 1). All the other buffer
//                  parameters are ignored when enabled=0.
//
// buffer_id: Two different buffers can be defined. Use buffer IDs 0 and 1.
//
//
//////////////////////////////////////////////////////////////////////////////
KGSL_API int kgsl_drawctxt_bind_gmem_shadow(gsl_deviceid_t device_id, unsigned int drawctxt_id, const gsl_rect_t* gmem_rect, unsigned int shadow_x, unsigned int shadow_y, const gsl_buffer_desc_t* shadow_buffer, unsigned int buffer_id)
{
    gsl_device_t   *device = &gsl_driver.device[device_id-1];
    gsl_drawctxt_t *drawctxt = &device->drawctxt[drawctxt_id];
    gmem_shadow_t  *shadow = &drawctxt->user_gmem_shadow[buffer_id];
    unsigned int    i;

    GSL_API_MUTEX_LOCK();
    GSL_CONTEXT_MUTEX_LOCK();

	if( !shadow_buffer->enabled )
    {
        // Disable shadow
        shadow->gmemshadow.size = 0;
    }
    else
    {
		// Sanity checks
		KOS_ASSERT((gmem_rect->x % 2) == 0);      // Needs to be a multiple of 2
		KOS_ASSERT((gmem_rect->y % 2) == 0);      // Needs to be a multiple of 2
		KOS_ASSERT((gmem_rect->width % 2) == 0);  // Needs to be a multiple of 2
		KOS_ASSERT((gmem_rect->height % 2) == 0); // Needs to be a multiple of 2
		KOS_ASSERT((gmem_rect->pitch % 32) == 0); // Needs to be a multiple of 32

		KOS_ASSERT((shadow_x % 2) == 0);  // Needs to be a multiple of 2
		KOS_ASSERT((shadow_y % 2) == 0);  // Needs to be a multiple of 2

		KOS_ASSERT(shadow_buffer->format >= COLORX_4_4_4_4);
		KOS_ASSERT(shadow_buffer->format <= COLORX_32_32_32_32_FLOAT);
		KOS_ASSERT((shadow_buffer->pitch % 32) == 0); // Needs to be a multiple of 32
		KOS_ASSERT(buffer_id >= 0);
		KOS_ASSERT(buffer_id < GSL_MAX_GMEM_SHADOW_BUFFERS);

		// Set up GMEM shadow regions
        kos_memcpy( &shadow->gmemshadow, &shadow_buffer->data, sizeof( gsl_memdesc_t ) );
        shadow->size = shadow->gmemshadow.size;

		shadow->width = shadow_buffer->width;
		shadow->height = shadow_buffer->height;
		shadow->pitch = shadow_buffer->pitch;
		shadow->format = shadow_buffer->format;

        shadow->offset = shadow->pitch * (shadow_y - gmem_rect->y) + shadow_x - gmem_rect->x;

        shadow->offset_x = shadow_x;
        shadow->offset_y = shadow_y;

		shadow->gmem_width = gmem_rect->width;
		shadow->gmem_height = gmem_rect->height;
		shadow->gmem_pitch = gmem_rect->pitch;

        shadow->gmem_offset_x = gmem_rect->x;
        shadow->gmem_offset_y = gmem_rect->y;

        // Modify quad vertices
        set_gmem_copy_quad(shadow);

        // Modify commands
        build_gmem2sys_cmds(drawctxt, NULL, shadow);
        build_sys2gmem_cmds(drawctxt, NULL, shadow);

        // Release context GMEM shadow if found
        if (drawctxt->context_gmem_shadow.gmemshadow.size > 0)
        {
            kgsl_sharedmem_free0(&drawctxt->context_gmem_shadow.gmemshadow, GSL_CALLER_PROCESSID_GET());
            drawctxt->context_gmem_shadow.gmemshadow.size = 0;
        }
    }

    // Enable GMEM shadowing if we have any of the user buffers enabled
    drawctxt->flags &= ~CTXT_FLAGS_GMEM_SHADOW;
    for( i = 0; i < GSL_MAX_GMEM_SHADOW_BUFFERS; i++ )
    {
        if( drawctxt->user_gmem_shadow[i].gmemshadow.size > 0 )
        {
            drawctxt->flags |= CTXT_FLAGS_GMEM_SHADOW;
        }
    }

    GSL_CONTEXT_MUTEX_UNLOCK();
    GSL_API_MUTEX_UNLOCK();

    return (GSL_SUCCESS);
}



//////////////////////////////////////////////////////////////////////////////
// switch drawing contexts
//////////////////////////////////////////////////////////////////////////////

void
kgsl_drawctxt_switch(gsl_device_t *device, gsl_drawctxt_t *drawctxt, gsl_flags_t flags)
{
    gsl_drawctxt_t *active_ctxt = device->drawctxt_active;

	if (drawctxt != GSL_CONTEXT_NONE)
	{
		if( flags & GSL_CONTEXT_SAVE_GMEM )
		{
			// Set the flag in context so that the save is done when this context is switched out.
			drawctxt->flags |= CTXT_FLAGS_GMEM_SAVE;
		}
		else
		{
			// Remove GMEM saving flag from the context
			drawctxt->flags &= ~CTXT_FLAGS_GMEM_SAVE;
		}
	}

    // already current?
    if (active_ctxt == drawctxt)
    {
        return;
    }

    // save old context, when not running in safe mode
    if (active_ctxt != GSL_CONTEXT_NONE && !(device->flags & GSL_FLAGS_SAFEMODE))
    {
        // save registers and constants.
        kgsl_ringbuffer_issuecmds(device, 0, active_ctxt->reg_save, 3, active_ctxt->pid);

        if (active_ctxt->flags & CTXT_FLAGS_SHADER_SAVE)
        {
            // save shader partitioning and instructions.
			kgsl_ringbuffer_issuecmds(device, 1, active_ctxt->shader_save, 3, active_ctxt->pid);

            // fixup shader partitioning parameter for SET_SHADER_BASES.
			kgsl_ringbuffer_issuecmds(device, 0, active_ctxt->shader_fixup, 3, active_ctxt->pid);

            active_ctxt->flags |= CTXT_FLAGS_SHADER_RESTORE;
        }

        if (active_ctxt->flags & CTXT_FLAGS_GMEM_SHADOW && active_ctxt->flags & CTXT_FLAGS_GMEM_SAVE )
        {
            // save gmem.  (note: changes shader.  shader must already be saved.)

            unsigned int i, numbuffers = 0;

            for( i = 0; i < GSL_MAX_GMEM_SHADOW_BUFFERS; i++ )
            {
                if( active_ctxt->user_gmem_shadow[i].gmemshadow.size > 0 )
                {
					kgsl_ringbuffer_issuecmds(device, 1, active_ctxt->user_gmem_shadow[i].gmem_save, 3, active_ctxt->pid);

                    // Restore TP0_CHICKEN
					kgsl_ringbuffer_issuecmds(device, 0, active_ctxt->chicken_restore, 3, active_ctxt->pid);
                    numbuffers++;
                }
            }
            if( numbuffers == 0 )
            {
                // No user defined buffers -> use context default
				kgsl_ringbuffer_issuecmds(device, 1, active_ctxt->context_gmem_shadow.gmem_save, 3, active_ctxt->pid);
                // Restore TP0_CHICKEN
				kgsl_ringbuffer_issuecmds(device, 0, active_ctxt->chicken_restore, 3, active_ctxt->pid);
            }

            active_ctxt->flags |= CTXT_FLAGS_GMEM_RESTORE;
        }
    }

    device->drawctxt_active = drawctxt;

    // restore new context, when not running in safe mode
    if (drawctxt != GSL_CONTEXT_NONE && !(device->flags & GSL_FLAGS_SAFEMODE))
    {
        KGSL_DEBUG(GSL_DBGFLAGS_DUMPX, KGSL_DEBUG_DUMPX(BB_DUMP_MEMWRITE, drawctxt->gpustate.gpuaddr, (unsigned int)drawctxt->gpustate.hostptr, LCC_SHADOW_SIZE + REG_SHADOW_SIZE + CMD_BUFFER_SIZE + TEX_SHADOW_SIZE , "kgsl_drawctxt_switch"));

        // restore gmem.  (note: changes shader.  shader must not already be restored.)
        if (drawctxt->flags & CTXT_FLAGS_GMEM_RESTORE)
        {
            unsigned int i, numbuffers = 0;

            for( i = 0; i < GSL_MAX_GMEM_SHADOW_BUFFERS; i++ )
            {
                if( drawctxt->user_gmem_shadow[i].gmemshadow.size > 0 )
                {
					kgsl_ringbuffer_issuecmds(device, 1, drawctxt->user_gmem_shadow[i].gmem_restore, 3, drawctxt->pid);

                    // Restore TP0_CHICKEN
					kgsl_ringbuffer_issuecmds(device, 0, drawctxt->chicken_restore, 3, drawctxt->pid);
                    numbuffers++;
                }
            }
            if( numbuffers == 0 )
            {
                // No user defined buffers -> use context default
				kgsl_ringbuffer_issuecmds(device, 1, drawctxt->context_gmem_shadow.gmem_restore, 3, drawctxt->pid);
                // Restore TP0_CHICKEN
				kgsl_ringbuffer_issuecmds(device, 0, drawctxt->chicken_restore, 3, drawctxt->pid);
            }

            drawctxt->flags &= ~CTXT_FLAGS_GMEM_RESTORE;
        }

        // restore registers and constants.
		kgsl_ringbuffer_issuecmds(device, 0, drawctxt->reg_restore, 3, drawctxt->pid);

        // restore shader instructions & partitioning.
        if (drawctxt->flags & CTXT_FLAGS_SHADER_RESTORE)
        {
			kgsl_ringbuffer_issuecmds(device, 0, drawctxt->shader_restore, 3, drawctxt->pid);
        }
    }
}


//////////////////////////////////////////////////////////////////////////////
// destroy all drawing contexts
//////////////////////////////////////////////////////////////////////////////
int
kgsl_drawctxt_destroyall(gsl_device_t *device)
{
    int             i;
    gsl_drawctxt_t  *drawctxt;

    GSL_CONTEXT_MUTEX_LOCK();

    for (i = 0; i < GSL_CONTEXT_MAX; i++)
    {
        drawctxt = &device->drawctxt[i];

        if (drawctxt->flags != CTXT_FLAGS_NOT_IN_USE)
        {
            // destroy state shadow, if allocated
            if (drawctxt->flags & CTXT_FLAGS_STATE_SHADOW)
                kgsl_sharedmem_free0(&drawctxt->gpustate, GSL_CALLER_PROCESSID_GET());

            // destroy gmem shadow, if allocated
            if (drawctxt->context_gmem_shadow.gmemshadow.size > 0)
            {
                kgsl_sharedmem_free0(&drawctxt->context_gmem_shadow.gmemshadow, GSL_CALLER_PROCESSID_GET());
                drawctxt->context_gmem_shadow.gmemshadow.size = 0;
            }

            drawctxt->flags = CTXT_FLAGS_NOT_IN_USE;

            device->drawctxt_count--;
            KOS_ASSERT(device->drawctxt_count >= 0);
        }
    }

    GSL_CONTEXT_MUTEX_UNLOCK();

    return (GSL_SUCCESS);
}

#endif
