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

/*
**	Include file the defines the front- and back-end compilers, as well as the
**	objects they use.
*/

#ifndef __gc_hal_compiler_h_
#define __gc_hal_compiler_h_

#ifndef VIVANTE_NO_3D
#include "gc_hal_types.h"
#include "gc_hal_engine.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef GC_ENABLE_LOADTIME_OPT
#define GC_ENABLE_LOADTIME_OPT           1
#endif

#define TEMP_OPT_CONSTANT_TEXLD_COORD    0

#define TEMP_SHADER_PATCH                1

#define TEMP_INLINE_ALL_EXPANSION            1
/******************************* IR VERSION ******************/
#define gcdSL_IR_VERSION gcmCC('\0','\0','\0','\1')

/******************************************************************************\
|******************************* SHADER LANGUAGE ******************************|
\******************************************************************************/

    /* allocator/deallocator function pointer */
typedef gceSTATUS (*gctAllocatorFunc)(
    IN gctSIZE_T Bytes,
    OUT gctPOINTER * Memory
    );

typedef gceSTATUS (*gctDeallocatorFunc)(
    IN gctPOINTER Memory
    );

typedef gctBOOL (*compareFunc) (
     IN void *    data,
     IN void *    key
     );

typedef struct _gcsListNode gcsListNode;
struct _gcsListNode
{
    gcsListNode *       next;
    void *              data;
};

typedef struct _gcsAllocator
{
    gctAllocatorFunc    allocate;
    gctDeallocatorFunc  deallocate;
} gcsAllocator;

/* simple map structure */
typedef struct _SimpleMap SimpleMap;
struct _SimpleMap
{
    gctUINT32     key;
    gctUINT32     val;
    SimpleMap    *next;
    gcsAllocator *allocator;

};

/* SimpleMap Operations */
/* return -1 if not found, otherwise return the mapped value */
gctUINT32
gcSimpleMap_Find(
     IN SimpleMap *Map,
     IN gctUINT32    Key
     );

gceSTATUS
gcSimpleMap_Destory(
     IN SimpleMap *    Map,
     IN gcsAllocator * Allocator
     );

/* Add a pair <Key, Val> to the Map head, the user should be aware that the
 * map pointer is always changed when adding a new node :
 *
 *   gcSimpleMap_AddNode(&theMap, key, val, allocator);
 *
 */
gceSTATUS
gcSimpleMap_AddNode(
     IN SimpleMap **   Map,
     IN gctUINT32      Key,
     IN gctUINT32      Val,
     IN gcsAllocator * Allocator
     );

/* gcsList data structure and related operations */
typedef struct _gcsList
{
    gcsListNode  *head;
    gcsListNode  *tail;
    gctINT        count;
    gcsAllocator *allocator;
} gcsList;

/* List operations */
void
gcList_Init(
    IN gcsList *list,
    IN gcsAllocator *allocator
    );

gceSTATUS
gcList_CreateNode(
    IN void *             Data,
    IN gctAllocatorFunc   Allocator,
    OUT gcsListNode **    ListNode
    );

gceSTATUS
gcList_Clean(
    IN gcsList *          List,
    IN gctBOOL            FreeData
    );

gcsListNode *
gcList_FindNode(
    IN gcsList *      List,
    IN void *         Key,
    IN compareFunc    compare
    );

gceSTATUS
gcList_AddNode(
    IN gcsList *          List,
    IN void *             Data
    );

gceSTATUS
gcList_RemoveNode(
    IN gcsList *          List,
    IN gcsListNode *      Node
    );

/*  link list structure for code list */
typedef gcsList gcsCodeList;
typedef gcsCodeList * gctCodeList;
typedef gcsListNode gcsCodeListNode;

/* Possible shader language opcodes. */
typedef enum _gcSL_OPCODE
{
	gcSL_NOP,							/* 0x00 */
	gcSL_MOV,							/* 0x01 */
	gcSL_SAT,							/* 0x02 */
	gcSL_DP3,							/* 0x03 */
	gcSL_DP4,							/* 0x04 */
	gcSL_ABS,							/* 0x05 */
	gcSL_JMP,							/* 0x06 */
	gcSL_ADD,							/* 0x07 */
	gcSL_MUL,							/* 0x08 */
	gcSL_RCP,							/* 0x09 */
	gcSL_SUB,							/* 0x0A */
	gcSL_KILL,							/* 0x0B */
	gcSL_TEXLD,							/* 0x0C */
	gcSL_CALL,							/* 0x0D */
	gcSL_RET,							/* 0x0E */
	gcSL_NORM,							/* 0x0F */
	gcSL_MAX,							/* 0x10 */
	gcSL_MIN,							/* 0x11 */
	gcSL_POW,							/* 0x12 */
	gcSL_RSQ,							/* 0x13 */
	gcSL_LOG,							/* 0x14 */
	gcSL_FRAC,							/* 0x15 */
	gcSL_FLOOR,							/* 0x16 */
	gcSL_CEIL,							/* 0x17 */
	gcSL_CROSS,							/* 0x18 */
	gcSL_TEXLDP,						/* 0x19 */
	gcSL_TEXBIAS,						/* 0x1A */
	gcSL_TEXGRAD,						/* 0x1B */
	gcSL_TEXLOD,						/* 0x1C */
	gcSL_SIN,							/* 0x1D */
	gcSL_COS,							/* 0x1E */
	gcSL_TAN,							/* 0x1F */
	gcSL_EXP,							/* 0x20 */
	gcSL_SIGN,							/* 0x21 */
	gcSL_STEP,							/* 0x22 */
	gcSL_SQRT,							/* 0x23 */
	gcSL_ACOS,							/* 0x24 */
	gcSL_ASIN,							/* 0x25 */
	gcSL_ATAN,							/* 0x26 */
	gcSL_SET,							/* 0x27 */
	gcSL_DSX,							/* 0x28 */
	gcSL_DSY,							/* 0x29 */
	gcSL_FWIDTH,						/* 0x2A */
	gcSL_DIV,   						/* 0x2B */
	gcSL_MOD,   						/* 0x2C */
	gcSL_AND_BITWISE,					/* 0x2D */
	gcSL_OR_BITWISE,					/* 0x2E */
	gcSL_XOR_BITWISE,					/* 0x2F */
	gcSL_NOT_BITWISE,					/* 0x30 */
	gcSL_LSHIFT,						/* 0x31 */
	gcSL_RSHIFT,						/* 0x32 */
	gcSL_ROTATE,						/* 0x33 */
	gcSL_BITSEL,						/* 0x34 */
	gcSL_LEADZERO,						/* 0x35 */
	gcSL_LOAD,							/* 0x36 */
	gcSL_STORE,							/* 0x37 */
	gcSL_BARRIER,						/* 0x38 */
	gcSL_STORE1,						/* 0x39 */
	gcSL_ATOMADD,						/* 0x3A */
	gcSL_ATOMSUB,						/* 0x3B */
	gcSL_ATOMXCHG,						/* 0x3C */
	gcSL_ATOMCMPXCHG,					/* 0x3D */
	gcSL_ATOMMIN,						/* 0x3E */
	gcSL_ATOMMAX,						/* 0x3F */
	gcSL_ATOMOR,						/* 0x40 */
	gcSL_ATOMAND,						/* 0x41 */
	gcSL_ATOMXOR,						/* 0x42 */
	/*gcSL_UNUSED,						 0x43 */
	/*gcSL_UNUSED,						 0x44 */
	/*gcSL_UNUSED,						 0x45 */
	/*gcSL_UNUSED,						 0x46 */
	/*gcSL_UNUSED,						 0x47 */
	/*gcSL_UNUSED,						 0x48 */
	/*gcSL_UNUSED,						 0x49 */
	/*gcSL_UNUSED,						 0x4A */
	/*gcSL_UNUSED,						 0x4B */
	/*gcSL_UNUSED,					 	 0x4C */
	/*gcSL_UNUSED,						 0x4D */
	/*gcSL_UNUSED,						 0x4E */
	/*gcSL_UNUSED,						 0x4F */
	/*gcSL_UNUSED,						 0x50 */
	/*gcSL_UNUSED,						 0x51 */
	/*gcSL_UNUSED,						 0x52 */
	gcSL_ADDLO = 0x53,					/* 0x53 */  /* Float only. */
	gcSL_MULLO,							/* 0x54 */  /* Float only. */
	gcSL_CONV,							/* 0x55 */
	gcSL_GETEXP,						/* 0x56 */
	gcSL_GETMANT,						/* 0x57 */
	gcSL_MULHI,							/* 0x58 */  /* Integer only. */
	gcSL_CMP,							/* 0x59 */
	gcSL_I2F,							/* 0x5A */
	gcSL_F2I,							/* 0x5B */
	gcSL_ADDSAT,						/* 0x5C */  /* Integer only. */
	gcSL_SUBSAT,						/* 0x5D */  /* Integer only. */
	gcSL_MULSAT,						/* 0x5E */  /* Integer only. */
	gcSL_DP2,							/* 0x5F */
	gcSL_MAXOPCODE
}
gcSL_OPCODE;

typedef enum _gcSL_FORMAT
{
	gcSL_FLOAT = 0,						/* 0 */
	gcSL_INTEGER = 1,				    /* 1 */
	gcSL_INT32 = 1,					    /* 1 */
	gcSL_BOOLEAN = 2,					/* 2 */
	gcSL_UINT32 = 3,					/* 3 */
	gcSL_INT8,						    /* 4 */
	gcSL_UINT8,						    /* 5 */
	gcSL_INT16,						    /* 6 */
	gcSL_UINT16,						/* 7 */
	gcSL_INT64,						    /* 8 */     /* Reserved for future enhancement. */
	gcSL_UINT64,						/* 9 */     /* Reserved for future enhancement. */
	gcSL_INT128,					    /* 10 */    /* Reserved for future enhancement. */
	gcSL_UINT128,						/* 11 */    /* Reserved for future enhancement. */
	gcSL_FLOAT16,					    /* 12 */
	gcSL_FLOAT64,						/* 13 */    /* Reserved for future enhancement. */
	gcSL_FLOAT128,						/* 14 */    /* Reserved for future enhancement. */
}
gcSL_FORMAT;

/* Destination write enable bits. */
typedef enum _gcSL_ENABLE
{
    gcSL_ENABLE_NONE                    = 0x0,     /* none is enabled, error/uninitialized state */
	gcSL_ENABLE_X						= 0x1,
	gcSL_ENABLE_Y						= 0x2,
	gcSL_ENABLE_Z						= 0x4,
	gcSL_ENABLE_W						= 0x8,
	/* Combinations. */
	gcSL_ENABLE_XY						= gcSL_ENABLE_X | gcSL_ENABLE_Y,
	gcSL_ENABLE_XYZ						= gcSL_ENABLE_X | gcSL_ENABLE_Y | gcSL_ENABLE_Z,
	gcSL_ENABLE_XYZW					= gcSL_ENABLE_X | gcSL_ENABLE_Y | gcSL_ENABLE_Z | gcSL_ENABLE_W,
	gcSL_ENABLE_XYW						= gcSL_ENABLE_X | gcSL_ENABLE_Y | gcSL_ENABLE_W,
	gcSL_ENABLE_XZ						= gcSL_ENABLE_X | gcSL_ENABLE_Z,
	gcSL_ENABLE_XZW						= gcSL_ENABLE_X | gcSL_ENABLE_Z | gcSL_ENABLE_W,
	gcSL_ENABLE_XW						= gcSL_ENABLE_X | gcSL_ENABLE_W,
	gcSL_ENABLE_YZ						= gcSL_ENABLE_Y | gcSL_ENABLE_Z,
	gcSL_ENABLE_YZW						= gcSL_ENABLE_Y | gcSL_ENABLE_Z | gcSL_ENABLE_W,
	gcSL_ENABLE_YW						= gcSL_ENABLE_Y | gcSL_ENABLE_W,
	gcSL_ENABLE_ZW						= gcSL_ENABLE_Z | gcSL_ENABLE_W,
}
gcSL_ENABLE;

/* Possible indices. */
typedef enum _gcSL_INDEXED
{
	gcSL_NOT_INDEXED,					/* 0 */
	gcSL_INDEXED_X,						/* 1 */
	gcSL_INDEXED_Y,						/* 2 */
	gcSL_INDEXED_Z,						/* 3 */
	gcSL_INDEXED_W,						/* 4 */
}
gcSL_INDEXED;

/* Opcode conditions. */
typedef enum _gcSL_CONDITION
{
	gcSL_ALWAYS,						/* 0x0 */
	gcSL_NOT_EQUAL,						/* 0x1 */
	gcSL_LESS_OR_EQUAL,					/* 0x2 */
	gcSL_LESS,							/* 0x3 */
	gcSL_EQUAL,							/* 0x4 */
	gcSL_GREATER,						/* 0x5 */
	gcSL_GREATER_OR_EQUAL,				/* 0x6 */
	gcSL_AND,							/* 0x7 */
	gcSL_OR,							/* 0x8 */
	gcSL_XOR,							/* 0x9 */
    gcSL_NOT_ZERO,                      /* 0xA */
}
gcSL_CONDITION;

/* Possible source operand types. */
typedef enum _gcSL_TYPE
{
	gcSL_NONE,							/* 0x0 */
	gcSL_TEMP,							/* 0x1 */
	gcSL_ATTRIBUTE,						/* 0x2 */
	gcSL_UNIFORM,						/* 0x3 */
	gcSL_SAMPLER,						/* 0x4 */
	gcSL_CONSTANT,						/* 0x5 */
	gcSL_OUTPUT,						/* 0x6 */
	gcSL_PHYSICAL,						/* 0x7 */
}
gcSL_TYPE;

/* Swizzle generator macro. */
#define gcmSWIZZLE(Component1, Component2, Component3, Component4) \
( \
	(gcSL_SWIZZLE_ ## Component1 << 0) | \
	(gcSL_SWIZZLE_ ## Component2 << 2) | \
	(gcSL_SWIZZLE_ ## Component3 << 4) | \
	(gcSL_SWIZZLE_ ## Component4 << 6)   \
)

#define gcmExtractSwizzle(Swizzle, Index) \
    ((gcSL_SWIZZLE) ((((Swizzle) >> (Index * 2)) & 0x3)))

#define gcmComposeSwizzle(SwizzleX, SwizzleY, SwizzleZ, SwizzleW) \
( \
	((SwizzleX) << 0) | \
	((SwizzleY) << 2) | \
	((SwizzleZ) << 4) | \
	((SwizzleW) << 6)   \
)

/* Possible swizzle values. */
typedef enum _gcSL_SWIZZLE
{
	gcSL_SWIZZLE_X,						/* 0x0 */
	gcSL_SWIZZLE_Y,						/* 0x1 */
	gcSL_SWIZZLE_Z,						/* 0x2 */
	gcSL_SWIZZLE_W,						/* 0x3 */
	/* Combinations. */
	gcSL_SWIZZLE_XXXX = gcmSWIZZLE(X, X, X, X),
	gcSL_SWIZZLE_YYYY = gcmSWIZZLE(Y, Y, Y, Y),
	gcSL_SWIZZLE_ZZZZ = gcmSWIZZLE(Z, Z, Z, Z),
	gcSL_SWIZZLE_WWWW = gcmSWIZZLE(W, W, W, W),
	gcSL_SWIZZLE_XYYY = gcmSWIZZLE(X, Y, Y, Y),
	gcSL_SWIZZLE_XZZZ = gcmSWIZZLE(X, Z, Z, Z),
	gcSL_SWIZZLE_XWWW = gcmSWIZZLE(X, W, W, W),
	gcSL_SWIZZLE_YZZZ = gcmSWIZZLE(Y, Z, Z, Z),
	gcSL_SWIZZLE_YWWW = gcmSWIZZLE(Y, W, W, W),
	gcSL_SWIZZLE_ZWWW = gcmSWIZZLE(Z, W, W, W),
	gcSL_SWIZZLE_XYZZ = gcmSWIZZLE(X, Y, Z, Z),
	gcSL_SWIZZLE_XYWW = gcmSWIZZLE(X, Y, W, W),
	gcSL_SWIZZLE_XZWW = gcmSWIZZLE(X, Z, W, W),
	gcSL_SWIZZLE_YZWW = gcmSWIZZLE(Y, Z, W, W),
	gcSL_SWIZZLE_XXYZ = gcmSWIZZLE(X, X, Y, Z),
	gcSL_SWIZZLE_XYZW = gcmSWIZZLE(X, Y, Z, W),
	gcSL_SWIZZLE_XYXY = gcmSWIZZLE(X, Y, X, Y),
	gcSL_SWIZZLE_YYZZ = gcmSWIZZLE(Y, Y, Z, Z),
	gcSL_SWIZZLE_YYWW = gcmSWIZZLE(Y, Y, W, W),
	gcSL_SWIZZLE_ZZZW = gcmSWIZZLE(Z, Z, Z, W),
	gcSL_SWIZZLE_XZZW = gcmSWIZZLE(X, Z, Z, W),
	gcSL_SWIZZLE_YYZW = gcmSWIZZLE(Y, Y, Z, W),

    gcSL_SWIZZLE_INVALID = 0x7FFFFFFF
}
gcSL_SWIZZLE;

typedef enum _gcSL_COMPONENT
{
	gcSL_COMPONENT_X,               /* 0x0 */
	gcSL_COMPONENT_Y,               /* 0x1 */
	gcSL_COMPONENT_Z,               /* 0x2 */
	gcSL_COMPONENT_W,               /* 0x3 */
    gcSL_COMPONENT_COUNT            /* 0x4 */
} gcSL_COMPONENT;

#define gcmIsComponentEnabled(Enable, Component) (((Enable) & (1 << (Component))) != 0)

/******************************************************************************\
|*********************************** SHADERS **********************************|
\******************************************************************************/

/* Shader types. */
typedef enum _gcSHADER_KIND {
    gcSHADER_TYPE_UNKNOWN = 0,
    gcSHADER_TYPE_VERTEX,
    gcSHADER_TYPE_FRAGMENT,
    gcSHADER_TYPE_CL,
    gcSHADER_TYPE_PRECOMPILED,
    gcSHADER_KIND_COUNT
} gcSHADER_KIND;

typedef enum _gcGL_DRIVER_VERSION {
    gcGL_DRIVER_ES11,    /* OpenGL ES 1.1 */
    gcGL_DRIVER_ES20,    /* OpenGL ES 2.0 */
    gcGL_DRIVER_ES30     /* OpenGL ES 3.0 */
} gcGL_DRIVER_VERSION;

/* gcSHADER objects. */
typedef struct _gcSHADER *              gcSHADER;
typedef struct _gcATTRIBUTE *			gcATTRIBUTE;
typedef struct _gcUNIFORM *             gcUNIFORM;
typedef struct _gcOUTPUT *              gcOUTPUT;
typedef struct _gcsFUNCTION *			gcFUNCTION;
typedef struct _gcsKERNEL_FUNCTION *	gcKERNEL_FUNCTION;
typedef struct _gcsHINT *               gcsHINT_PTR;
typedef struct _gcSHADER_PROFILER *     gcSHADER_PROFILER;
typedef struct _gcVARIABLE *			gcVARIABLE;
typedef struct _gcSHADER_LIST *         gcSHADER_LIST;

struct _gcsHINT
{
    /* Numbr of data transfers for Vertex Shader output. */
    gctUINT32   vsOutputCount;

    /* Flag whether the VS has point size or not. */
    gctBOOL     vsHasPointSize;

#if gcdUSE_WCLIP_PATCH
    /* Flag whether the VS gl_position.z depends on gl_position.w
       it's a hint for wclipping */
    gctBOOL     vsPositionZDependsOnW;
#endif

    gctBOOL     clipW;

    /* Flag whether or not the shader has a KILL instruction. */
    gctBOOL     hasKill;

    /* Element count. */
    gctUINT32   elementCount;

    /* Component count. */
    gctUINT32   componentCount;

    /* Number of data transfers for Fragment Shader input. */
    gctUINT32   fsInputCount;

    /* Maximum number of temporary registers used in FS. */
    gctUINT32   fsMaxTemp;

	/* Maximum number of temporary registers used in VS. */
	gctUINT32   vsMaxTemp;

    /* Balance minimum. */
    gctUINT32   balanceMin;

    /* Balance maximum. */
    gctUINT32   balanceMax;

    /* Auto-shift balancing. */
    gctBOOL     autoShift;

    /* Flag whether the PS outputs the depth value or not. */
    gctBOOL     psHasFragDepthOut;

	/* Flag whether the ThreadWalker is in PS. */
	gctBOOL		threadWalkerInPS;

    /* HW reg number for position of VS */
    gctUINT32   hwRegNoOfSIVPos;

#if gcdALPHA_KILL_IN_SHADER
    /* States to set when alpha kill is enabled. */
    gctUINT32   killStateAddress;
    gctUINT32   alphaKillStateValue;
    gctUINT32   colorKillStateValue;

    /* Shader instructiuon. */
    gctUINT32   killInstructionAddress;
    gctUINT32   alphaKillInstruction[3];
    gctUINT32   colorKillInstruction[3];
#endif

#if TEMP_SHADER_PATCH
	gctUINT32	pachedShaderIdentifier;
#endif

#if gcdUSE_WCLIP_PATCH
    /* Strict WClip match. */
    gctBOOL     strictWClipMatch;
#endif
};

#if TEMP_SHADER_PATCH
#define INVALID_SHADER_IDENTIFIER 0xFFFFFFFF
#endif

/* gcSHADER_TYPE enumeration. */
typedef enum _gcSHADER_TYPE
{
    gcSHADER_FLOAT_X1   = 0,        /* 0x00 */
    gcSHADER_FLOAT_X2,				/* 0x01 */
	gcSHADER_FLOAT_X3,				/* 0x02 */
	gcSHADER_FLOAT_X4,				/* 0x03 */
	gcSHADER_FLOAT_2X2,				/* 0x04 */
	gcSHADER_FLOAT_3X3,				/* 0x05 */
	gcSHADER_FLOAT_4X4,				/* 0x06 */
	gcSHADER_BOOLEAN_X1,			/* 0x07 */
	gcSHADER_BOOLEAN_X2,			/* 0x08 */
	gcSHADER_BOOLEAN_X3,			/* 0x09 */
	gcSHADER_BOOLEAN_X4,			/* 0x0A */
	gcSHADER_INTEGER_X1,			/* 0x0B */
	gcSHADER_INTEGER_X2,			/* 0x0C */
	gcSHADER_INTEGER_X3,			/* 0x0D */
	gcSHADER_INTEGER_X4,			/* 0x0E */
	gcSHADER_SAMPLER_1D,			/* 0x0F */
	gcSHADER_SAMPLER_2D,			/* 0x10 */
	gcSHADER_SAMPLER_3D,			/* 0x11 */
	gcSHADER_SAMPLER_CUBIC,			/* 0x12 */
	gcSHADER_FIXED_X1,				/* 0x13 */
	gcSHADER_FIXED_X2,				/* 0x14 */
	gcSHADER_FIXED_X3,				/* 0x15 */
	gcSHADER_FIXED_X4,				/* 0x16 */
	gcSHADER_IMAGE_2D,				/* 0x17 */  /* For OCL. */
	gcSHADER_IMAGE_3D,				/* 0x18 */  /* For OCL. */
	gcSHADER_SAMPLER,				/* 0x19 */  /* For OCL. */
	gcSHADER_FLOAT_2X3,				/* 0x1A */
	gcSHADER_FLOAT_2X4,				/* 0x1B */
	gcSHADER_FLOAT_3X2,				/* 0x1C */
	gcSHADER_FLOAT_3X4,				/* 0x1D */
	gcSHADER_FLOAT_4X2,				/* 0x1E */
	gcSHADER_FLOAT_4X3,				/* 0x1F */
	gcSHADER_ISAMPLER_2D,			/* 0x20 */
	gcSHADER_ISAMPLER_3D,			/* 0x21 */
	gcSHADER_ISAMPLER_CUBIC,		/* 0x22 */
	gcSHADER_USAMPLER_2D,			/* 0x23 */
	gcSHADER_USAMPLER_3D,			/* 0x24 */
	gcSHADER_USAMPLER_CUBIC,		/* 0x25 */
	gcSHADER_SAMPLER_EXTERNAL_OES,		/* 0x26 */

	gcSHADER_UINT_X1,			/* 0x27 */
	gcSHADER_UINT_X2,			/* 0x28 */
	gcSHADER_UINT_X3,			/* 0x29 */
	gcSHADER_UINT_X4,			/* 0x2A */

    gcSHADER_UNKONWN_TYPE,      /* do not add type after this */
    gcSHADER_TYPE_COUNT         /* must to change gcvShaderTypeInfo at the
                                 * same time if you add any new type! */}
gcSHADER_TYPE;

typedef enum _gcSHADER_TYPE_KIND
{
    gceTK_UNKOWN,
    gceTK_FLOAT,
    gceTK_INT,
    gceTK_UINT,
    gceTK_BOOL,
    gceTK_FIXED,
    gceTK_SAMPLER,
    gceTK_IMAGE,
    gceTK_OTHER
} gcSHADER_TYPE_KIND;

typedef struct _gcSHADER_TYPEINFO
{
    gcSHADER_TYPE      type;              /* e.g. gcSHADER_FLOAT_2X4 */
    gctINT             components;        /* e.g. 4 components       */
    gctINT             rows;              /* e.g. 2 rows             */
    gcSHADER_TYPE      componentType;     /* e.g. gcSHADER_FLOAT_X4  */
    gcSHADER_TYPE_KIND kind;              /* e.g. gceTK_FLOAT */
    gctCONST_STRING    name;              /* e.g. "FLOAT_2X4" */
} gcSHADER_TYPEINFO;

extern gcSHADER_TYPEINFO gcvShaderTypeInfo[];

#define gcmType_Comonents(Type)    (gcvShaderTypeInfo[Type].components)
#define gcmType_Rows(Type)         (gcvShaderTypeInfo[Type].rows)
#define gcmType_ComonentType(Type) (gcvShaderTypeInfo[Type].componentType)
#define gcmType_Kind(Type)         (gcvShaderTypeInfo[Type].kind)
#define gcmType_Name(Type)         (gcvShaderTypeInfo[Type].name)

#define gcmType_isMatrix(type) (gcmType_Rows(type) > 1)

typedef enum _gcSHADER_VAR_CATEGORY
{
    gcSHADER_VAR_CATEGORY_NORMAL  =  0, /* primitive type and its array */
    gcSHADER_VAR_CATEGORY_STRUCT  =  1  /* structure */
}
gcSHADER_VAR_CATEGORY;

typedef enum _gceTYPE_QUALIFIER
{
    gcvTYPE_QUALIFIER_NONE         = 0x0, /* unqualified */
    gcvTYPE_QUALIFIER_VOLATILE     = 0x1, /* volatile */
}gceTYPE_QUALIFIER;

typedef gctUINT16  gctTYPE_QUALIFIER;

#if GC_ENABLE_LOADTIME_OPT
typedef struct _gcSHADER_TYPE_INFO
{
    gcSHADER_TYPE    type;        /* eg. gcSHADER_FLOAT_2X3 is the type */
    gctCONST_STRING  name;        /* the name of the type: "gcSHADER_FLOAT_2X3" */
    gcSHADER_TYPE    baseType;    /* its base type is gcSHADER_FLOAT_2 */
    gctINT           components;  /* it has 2 components */
    gctINT           rows;        /* and 3 rows */
    gctINT           size;        /* the size in byte */
} gcSHADER_TYPE_INFO;

extern gcSHADER_TYPE_INFO shader_type_info[];

enum gceLTCDumpOption {
    gceLTC_DUMP_UNIFORM      = 0x0001,
    gceLTC_DUMP_EVALUATION   = 0x0002,
    gceLTC_DUMP_EXPESSION    = 0x0004,
    gceLTC_DUMP_COLLECTING   = 0x0008,
};

gctBOOL gcDumpOption(gctINT Opt);

#endif /* GC_ENABLE_LOADTIME_OPT */

#define IS_MATRIX_TYPE(type) \
    (((type >= gcSHADER_FLOAT_2X2) && (type <= gcSHADER_FLOAT_4X4)) || \
     ((type >= gcSHADER_FLOAT_2X3) && (type <= gcSHADER_FLOAT_4X3)))

/* gcSHADER_PRECISION enumeration. */
typedef enum _gcSHADER_PRECISION
{
	gcSHADER_PRECISION_DEFAULT,				/* 0x00 */
	gcSHADER_PRECISION_HIGH,				/* 0x01 */
	gcSHADER_PRECISION_MEDIUM,				/* 0x02 */
	gcSHADER_PRECISION_LOW,				    /* 0x03 */
}
gcSHADER_PRECISION;

/* Shader flags. */
typedef enum _gceSHADER_FLAGS
{
    gcvSHADER_NO_OPTIMIZATION           = 0x00,
	gcvSHADER_DEAD_CODE					= 0x01,
	gcvSHADER_RESOURCE_USAGE			= 0x02,
	gcvSHADER_OPTIMIZER					= 0x04,
	gcvSHADER_USE_GL_Z					= 0x08,
    /*
        The GC family of GPU cores model GC860 and under require the Z
        to be from 0 <= z <= w.
        However, OpenGL specifies the Z to be from -w <= z <= w.  So we
        have to a conversion here:

            z = (z + w) / 2.

        So here we append two instructions to the vertex shader.
    */
	gcvSHADER_USE_GL_POSITION			= 0x10,
	gcvSHADER_USE_GL_FACE				= 0x20,
	gcvSHADER_USE_GL_POINT_COORD		= 0x40,
	gcvSHADER_LOADTIME_OPTIMIZER		= 0x80,
#if gcdALPHA_KILL_IN_SHADER
    gcvSHADER_USE_ALPHA_KILL            = 0x100,
#endif

#if gcdPRE_ROTATION && (ANDROID_SDK_VERSION >= 14)
    gcvSHADER_VS_PRE_ROTATION           = 0x200,
#endif

#if TEMP_INLINE_ALL_EXPANSION
    gcvSHADER_INLINE_ALL_EXPANSION      = 0x400,
#endif
}
gceSHADER_FLAGS;

gceSTATUS
gcSHADER_CheckClipW(
    IN gctCONST_STRING VertexSource,
    IN gctCONST_STRING FragmentSource,
    OUT gctBOOL * clipW);

/*******************************************************************************
**  gcSHADER_GetUniformVectorCount
**
**  Get the number of vectors used by uniforms for this shader.
**
**  INPUT:
**
**      gcSHADER Shader
**          Pointer to a gcSHADER object.
**
**  OUTPUT:
**
**      gctSIZE_T * Count
**          Pointer to a variable receiving the number of vectors.
*/
gceSTATUS
gcSHADER_GetUniformVectorCount(
    IN gcSHADER Shader,
    OUT gctSIZE_T * Count
    );

/*******************************************************************************
**							gcOptimizer Data Structures
*******************************************************************************/
typedef enum _gceSHADER_OPTIMIZATION
{
    /*  No optimization. */
	gcvOPTIMIZATION_NONE,

    /*  Flow graph construction. */
	gcvOPTIMIZATION_CONSTRUCTION                = 1 << 0,

    /*  Dead code elimination. */
	gcvOPTIMIZATION_DEAD_CODE                   = 1 << 1,

    /*  Redundant move instruction elimination. */
	gcvOPTIMIZATION_REDUNDANT_MOVE              = 1 << 2,

    /*  Inline expansion. */
	gcvOPTIMIZATION_INLINE_EXPANSION            = 1 << 3,

    /*  Constant propagation. */
	gcvOPTIMIZATION_CONSTANT_PROPAGATION        = 1 << 4,

    /*  Redundant bounds/checking elimination. */
	gcvOPTIMIZATION_REDUNDANT_CHECKING          = 1 << 5,

    /*  Loop invariant movement. */
	gcvOPTIMIZATION_LOOP_INVARIANT              = 1 << 6,

    /*  Induction variable removal. */
	gcvOPTIMIZATION_INDUCTION_VARIABLE          = 1 << 7,

    /*  Common subexpression elimination. */
	gcvOPTIMIZATION_COMMON_SUBEXPRESSION        = 1 << 8,

    /*  Control flow/banch optimization. */
	gcvOPTIMIZATION_CONTROL_FLOW                = 1 << 9,

    /*  Vector component operation merge. */
	gcvOPTIMIZATION_VECTOR_INSTRUCTION_MERGE    = 1 << 10,

    /*  Algebra simplificaton. */
	gcvOPTIMIZATION_ALGEBRAIC_SIMPLIFICATION    = 1 << 11,

    /*  Pattern matching and replacing. */
	gcvOPTIMIZATION_PATTERN_MATCHING            = 1 << 12,

    /*  Interprocedural constant propagation. */
	gcvOPTIMIZATION_IP_CONSTANT_PROPAGATION     = 1 << 13,

    /*  Interprecedural register optimization. */
	gcvOPTIMIZATION_IP_REGISTRATION             = 1 << 14,

    /*  Optimization option number. */
	gcvOPTIMIZATION_OPTION_NUMBER               = 1 << 15,

	/*  Loadtime constant. */
    gcvOPTIMIZATION_LOADTIME_CONSTANT           = 1 << 16,

    /*  MAD instruction optimization. */
	gcvOPTIMIZATION_MAD_INSTRUCTION             = 1 << 17,

    /*  Special optimization for LOAD SW workaround. */
	gcvOPTIMIZATION_LOAD_SW_WORKAROUND          = 1 << 18,

    /* move code into conditional block if possile */
	gcvOPTIMIZATION_CONDITIONALIZE              = 1 << 19,

    /* expriemental: power optimization mode
        1. add extra dummy texld to tune performance
        2. insert NOP after high power instrucitons
        3. split high power vec3/vec4 instruciton to vec2/vec1 operation
        4. ...
     */
	gcvOPTIMIZATION_POWER_OPTIMIZATION           = 1 << 20,

    /* optimize varying packing */
    gcvOPTIMIZATION_VARYINGPACKING              = 1 << 22,

#if TEMP_INLINE_ALL_EXPANSION
	gcvOPTIMIZATION_INLINE_ALL_EXPANSION        = 1 << 23,
#endif

    /*  Full optimization. */
    /*  Note that gcvOPTIMIZATION_LOAD_SW_WORKAROUND is off. */
	gcvOPTIMIZATION_FULL                        = 0x7FFFFFFF &
                                                  ~gcvOPTIMIZATION_LOAD_SW_WORKAROUND &
                                                  ~gcvOPTIMIZATION_INLINE_ALL_EXPANSION &
                                                  ~gcvOPTIMIZATION_POWER_OPTIMIZATION,

	/* Optimization Unit Test flag. */
    gcvOPTIMIZATION_UNIT_TEST                   = 1 << 31
}
gceSHADER_OPTIMIZATION;

typedef enum _gceOPTIMIZATION_VaryingPaking
{
    gcvOPTIMIZATION_VARYINGPACKING_NONE = 0,
    gcvOPTIMIZATION_VARYINGPACKING_NOSPLIT,
    gcvOPTIMIZATION_VARYINGPACKING_SPLIT
} gceOPTIMIZATION_VaryingPaking;

typedef struct _gcOPTIMIZER_OPTION
{
    gceSHADER_OPTIMIZATION     optFlags;

    /* debug & dump options:

         VC_OPTION=-DUMP:SRC:OPT|:OPTV|:CG|:CGV:|ALL|ALLV

         SRC:  dump shader source code
         OPT:  dump incoming and final IR
         OPTV: dump result IR in each optimization phase
         CG:   dump generated machine code
         CGV:  dump BE tree and optimization detail

         ALL = SRC|OPT|CG
         ALLV = SRC|OPT|OPTV|CG|CGV
     */
    gctBOOL     dumpShaderSource;      /* dump shader source code */
    gctBOOL     dumpOptimizer;         /* dump incoming and final IR */
    gctBOOL     dumpOptimizerVerbose;  /* dump result IR in each optimization phase */
    gctBOOL     dumpBEGenertedCode;    /* dump generated machine code */
    gctBOOL     dumpBEVerbose;         /* dump BE tree and optimization detail */
    gctBOOL     dumpBEFinalIR;         /* dump BE final IR */

    /* Code generation */

    /* Varying Packing:

          VC_OPTION=-PACKVARYING:[0-2]|:T[-]m[,n]|:LshaderIdx,min,max

          0: turn off varying packing
          1: pack varyings, donot split any varying
          2: pack varyings, may split to make fully packed output

          Tm:    only packing shader pair which vertex shader id is m
          Tm,n:  only packing shader pair which vertex shader id
                   is in range of [m, n]
          T-m:   do not packing shader pair which vertex shader id is m
          T-m,n: do not packing shader pair which vertex shader id
                   is in range of [m, n]

          LshaderIdx,min,max : set  load balance (min, max) for shaderIdx
                               if shaderIdx is -1, all shaders are impacted
                               newMin = origMin * (min/100.);
                               newMax = origMax * (max/100.);
     */
    gceOPTIMIZATION_VaryingPaking    packVarying;
    gctINT                           _triageStart;
    gctINT                           _triageEnd;
    gctINT                           _loadBalanceShaderIdx;
    gctINT                           _loadBalanceMin;
    gctINT                           _loadBalanceMax;

    /* Do not generate immdeiate

          VC_OPTION=-NOIMM

       Force generate immediate even the machine model don't support it,
       for testing purpose only

          VC_OPTION=-FORCEIMM
     */
    gctBOOL     noImmediate;
    gctBOOL     forceImmediate;

    /* Power reduction mode options */
    gctBOOL   needPowerOptimization;

    /* Patch TEXLD instruction by adding dummy texld
       (can be used to tune GPU power usage):
         for every TEXLD we seen, add n dummy TEXLD

        it can be enabled by environment variable:

          VC_OPTION=-PATCH_TEXLD:M:N

        (for each M texld, add N dummy texld)
     */
    gctINT      patchEveryTEXLDs;
    gctINT      patchDummyTEXLDs;

    /* Insert NOP after high power consumption instructions

         VC_OPTION="-INSERTNOP:MUL:MULLO:DP3:DP4:SEENTEXLD"
     */
    gctBOOL     insertNOP;
    gctBOOL     insertNOPAfterMUL;
    gctBOOL     insertNOPAfterMULLO;
    gctBOOL     insertNOPAfterDP3;
    gctBOOL     insertNOPAfterDP4;
    gctBOOL     insertNOPOnlyWhenTexldSeen;

    /* split MAD to MUL and ADD:

         VC_OPTION=-SPLITMAD
     */
    gctBOOL     splitMAD;

    /* Convert vect3/vec4 operations to multiple vec2/vec1 operations

         VC_OPTION=-SPLITVEC:MUL:MULLO:DP3:DP4
     */
    gctBOOL     splitVec;
    gctBOOL     splitVec4MUL;
    gctBOOL     splitVec4MULLO;
    gctBOOL     splitVec4DP3;
    gctBOOL     splitVec4DP4;

    /* turn/off features:

          VC_OPTION=-F:n,[0|1]
          Note: n must be decimal number
     */
    gctUINT     featureBits;

    /* inline level (default 2 at O1):

          VC_OPTION=-INLINELEVEL:[0-3]
             0:  no inline
             1:  only inline the function only called once or small function
             2:  inline functions be called less than 5 times or medium size function
             3:  inline everything possible
     */
    gctUINT     inlineLevel;
} gcOPTIMIZER_OPTION;

extern gcOPTIMIZER_OPTION theOptimizerOption;
#define gcmGetOptimizerOption() gcGetOptimizerOption()

#define gcmOPT_DUMP_SHADER_SRC()         \
             (gcmGetOptimizerOption()->dumpShaderSource != 0)
#define gcmOPT_DUMP_OPTIMIZER()          \
             (gcmGetOptimizerOption()->dumpOptimizer != 0 || \
              gcmOPT_DUMP_OPTIMIZER_VERBOSE() )
#define gcmOPT_DUMP_OPTIMIZER_VERBOSE()  \
             (gcmGetOptimizerOption()->dumpOptimizerVerbose != 0)
#define gcmOPT_DUMP_CODEGEN()            \
             (gcmGetOptimizerOption()->dumpBEGenertedCode != 0 || \
              gcmOPT_DUMP_CODEGEN_VERBOSE() )
#define gcmOPT_DUMP_CODEGEN_VERBOSE()    \
             (gcmGetOptimizerOption()->dumpBEVerbose != 0)
#define gcmOPT_DUMP_FINAL_IR()    \
             (gcmGetOptimizerOption()->dumpBEFinalIR != 0)

#define gcmOPT_SET_DUMP_SHADER_SRC(v)   \
             gcmGetOptimizerOption()->dumpShaderSource = (v)

#define gcmOPT_PATCH_TEXLD()  (gcmGetOptimizerOption()->patchDummyTEXLDs != 0)
#define gcmOPT_INSERT_NOP()   (gcmGetOptimizerOption()->insertNOP == gcvTRUE)
#define gcmOPT_SPLITMAD()     (gcmGetOptimizerOption()->splitMAD == gcvTRUE)
#define gcmOPT_SPLITVEC()     (gcmGetOptimizerOption()->splitVec == gcvTRUE)

#define gcmOPT_NOIMMEDIATE()  (gcmGetOptimizerOption()->noImmediate == gcvTRUE)
#define gcmOPT_FORCEIMMEDIATE()  (gcmGetOptimizerOption()->forceImmediate == gcvTRUE)

#define gcmOPT_PACKVARYING()     (gcmGetOptimizerOption()->packVarying)
#define gcmOPT_PACKVARYING_triageStart()   (gcmGetOptimizerOption()->_triageStart)
#define gcmOPT_PACKVARYING_triageEnd()     (gcmGetOptimizerOption()->_triageEnd)

#define gcmOPT_INLINELEVEL()     (gcmGetOptimizerOption()->inlineLevel)

/* Setters */
#define gcmOPT_SetPatchTexld(m,n) (gcmGetOptimizerOption()->patchEveryTEXLDs = (m),\
                                   gcmGetOptimizerOption()->patchDummyTEXLDs = (n))
#define gcmOPT_SetSplitVecMUL() (gcmGetOptimizerOption()->splitVec = gcvTRUE, \
                                 gcmGetOptimizerOption()->splitVec4MUL = gcvTRUE)
#define gcmOPT_SetSplitVecMULLO() (gcmGetOptimizerOption()->splitVec = gcvTRUE, \
                                  gcmGetOptimizerOption()->splitVec4MULLO = gcvTRUE)
#define gcmOPT_SetSplitVecDP3() (gcmGetOptimizerOption()->splitVec = gcvTRUE, \
                                 gcmGetOptimizerOption()->splitVec4DP3 = gcvTRUE)
#define gcmOPT_SetSplitVecDP4() (gcmGetOptimizerOption()->splitVec = gcvTRUE, \
                                 gcmGetOptimizerOption()->splitVec4DP4 = gcvTRUE)

#define gcmOPT_SetPackVarying(v)     (gcmGetOptimizerOption()->packVarying = v)

#define FB_LIVERANGE_FIX1     0x0001


#define PredefinedDummySamplerId       8

/* Function argument qualifier */
typedef enum _gceINPUT_OUTPUT
{
	gcvFUNCTION_INPUT,
	gcvFUNCTION_OUTPUT,
	gcvFUNCTION_INOUT
}
gceINPUT_OUTPUT;

/* Kernel function property flags. */
typedef enum _gcePROPERTY_FLAGS
{
	gcvPROPERTY_REQD_WORK_GRP_SIZE	= 0x01
}
gceKERNEL_FUNCTION_PROPERTY_FLAGS;

/* Uniform flags. */
typedef enum _gceUNIFORM_FLAGS
{
	gcvUNIFORM_KERNEL_ARG			= 0x01,
	gcvUNIFORM_KERNEL_ARG_LOCAL		= 0x02,
	gcvUNIFORM_KERNEL_ARG_SAMPLER		= 0x04,
	gcvUNIFORM_LOCAL_ADDRESS_SPACE		= 0x08,
	gcvUNIFORM_PRIVATE_ADDRESS_SPACE	= 0x10,
	gcvUNIFORM_CONSTANT_ADDRESS_SPACE	= 0x20,
	gcvUNIFORM_GLOBAL_SIZE			= 0x40,
	gcvUNIFORM_LOCAL_SIZE			= 0x80,
	gcvUNIFORM_NUM_GROUPS			= 0x100,
	gcvUNIFORM_GLOBAL_OFFSET		= 0x200,
	gcvUNIFORM_WORK_DIM			= 0x400,
	gcvUNIFORM_KERNEL_ARG_CONSTANT		= 0x800,
	gcvUNIFORM_KERNEL_ARG_LOCAL_MEM_SIZE	= 0x1000,
	gcvUNIFORM_KERNEL_ARG_PRIVATE		= 0x2000,
	gcvUNIFORM_LOADTIME_CONSTANT		= 0x4000,
    gcvUNIFORM_IS_ARRAY                 = 0x8000,
}
gceUNIFORM_FLAGS;

#define gcdUNIFORM_KERNEL_ARG_MASK  (gcvUNIFORM_KERNEL_ARG         | \
                                     gcvUNIFORM_KERNEL_ARG_LOCAL   | \
									 gcvUNIFORM_KERNEL_ARG_SAMPLER | \
									 gcvUNIFORM_KERNEL_ARG_PRIVATE | \
									 gcvUNIFORM_KERNEL_ARG_CONSTANT)

typedef enum _gceVARIABLE_UPDATE_FLAGS
{
    gcvVARIABLE_UPDATE_NOUPDATE = 0,
    gcvVARIABLE_UPDATE_TEMPREG,
    gcvVARIABLE_UPDATE_TYPE_QUALIFIER,
}gceVARIABLE_UPDATE_FLAGS;

typedef struct _gcMACHINE_INST
{
    gctUINT        state0;
    gctUINT        state1;
    gctUINT        state2;
    gctUINT        state3;
}gcMACHINE_INST, *gcMACHINE_INST_PTR;

typedef struct _gcMACHINECODE
{
    gcMACHINE_INST_PTR   pCode;          /* machine code  */
    gctUINT              instCount;      /* 128-bit count */
    gctUINT              maxConstRegNo;
    gctUINT              maxTempRegNo;
    gctUINT              endPCOfMainRoutine;
}gcMACHINECODE, *gcMACHINECODE_PTR;

typedef enum NP2_ADDRESS_MODE
{
    NP2_ADDRESS_MODE_CLAMP  = 0,
    NP2_ADDRESS_MODE_REPEAT = 1,
    NP2_ADDRESS_MODE_MIRROR = 2
}NP2_ADDRESS_MODE;

typedef struct _gcNPOT_PATCH_PARAM
{
    gctINT               samplerSlot;
    NP2_ADDRESS_MODE     addressMode[3];
    gctINT               texDimension;    /* 2 or 3 */
}gcNPOT_PATCH_PARAM, *gcNPOT_PATCH_PARAM_PTR;

typedef struct _gcZBIAS_PATCH_PARAM
{
    /* Driver uses this to program uniform that designating zbias */
    gctINT               uniformAddr;
    gctINT               channel;
}gcZBIAS_PATCH_PARAM, *gcZBIAS_PATCH_PARAM_PTR;

void
gcGetOptionFromEnv(
    IN OUT gcOPTIMIZER_OPTION * Option
    );

void
gcSetOptimizerOption(
    IN gceSHADER_FLAGS Flags
    );

gcOPTIMIZER_OPTION *
gcGetOptimizerOption();

/*******************************************************************************
**  gcSHADER_SetCompilerVersion
**
**  Set the compiler version of a gcSHADER object.
**
**  INPUT:
**
**      gcSHADER Shader
**          Pointer to gcSHADER object
**
**      gctINT *Version
**          Pointer to a two word version
*/
gceSTATUS
gcSHADER_SetCompilerVersion(
    IN gcSHADER Shader,
    IN gctUINT32 *Version
    );

/*******************************************************************************
**  gcSHADER_GetCompilerVersion
**
**  Get the compiler version of a gcSHADER object.
**
**  INPUT:
**
**      gcSHADER Shader
**          Pointer to a gcSHADER object.
**
**  OUTPUT:
**
**      gctUINT32_PTR *CompilerVersion.
**          Pointer to holder of returned compilerVersion pointer
*/
gceSTATUS
gcSHADER_GetCompilerVersion(
    IN gcSHADER Shader,
    OUT gctUINT32_PTR *CompilerVersion
    );

/*******************************************************************************
**  gcSHADER_GetType
**
**  Get the gcSHADER object's type.
**
**  INPUT:
**
**      gcSHADER Shader
**          Pointer to a gcSHADER object.
**
**  OUTPUT:
**
**      gctINT *Type.
**          Pointer to return shader type.
*/
gceSTATUS
gcSHADER_GetType(
    IN gcSHADER Shader,
    OUT gctINT *Type
    );

gctUINT
gcSHADER_NextId();
/*******************************************************************************
**                             gcSHADER_Construct
********************************************************************************
**
**	Construct a new gcSHADER object.
**
**	INPUT:
**
**		gcoOS Hal
**			Pointer to an gcoHAL object.
**
**		gctINT ShaderType
**			Type of gcSHADER object to cerate.  'ShaderType' can be one of the
**			following:
**
**				gcSHADER_TYPE_VERTEX	Vertex shader.
**				gcSHADER_TYPE_FRAGMENT	Fragment shader.
**
**	OUTPUT:
**
**		gcSHADER * Shader
**			Pointer to a variable receiving the gcSHADER object pointer.
*/
gceSTATUS
gcSHADER_Construct(
	IN gcoHAL Hal,
	IN gctINT ShaderType,
	OUT gcSHADER * Shader
	);

/*******************************************************************************
**                              gcSHADER_Destroy
********************************************************************************
**
**	Destroy a gcSHADER object.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**	OUTPUT:
**
**		Nothing.
*/
gceSTATUS
gcSHADER_Destroy(
	IN gcSHADER Shader
	);

/*******************************************************************************
**                              gcSHADER_Copy
********************************************************************************
**
**	Copy a gcSHADER object.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**      gcSHADER Source
**          Pointer to a gcSHADER object that will be copied.
**
**	OUTPUT:
**
**		Nothing.
*/
gceSTATUS
gcSHADER_Copy(
	IN gcSHADER Shader,
	IN gcSHADER Source
	);

/*******************************************************************************
**  gcSHADER_LoadHeader
**
**  Load a gcSHADER object from a binary buffer.  The binary buffer is layed out
**  as follows:
**      // Six word header
**      // Signature, must be 'S','H','D','R'.
**      gctINT8             signature[4];
**      gctUINT32           binFileVersion;
**      gctUINT32           compilerVersion[2];
**      gctUINT32           gcSLVersion;
**      gctUINT32           binarySize;
**
**  INPUT:
**
**      gcSHADER Shader
**          Pointer to a gcSHADER object.
**          Shader type will be returned if type in shader object is not gcSHADER_TYPE_PRECOMPILED
**
**      gctPOINTER Buffer
**          Pointer to a binary buffer containing the shader data to load.
**
**      gctSIZE_T BufferSize
**          Number of bytes inside the binary buffer pointed to by 'Buffer'.
**
**  OUTPUT:
**      nothing
**
*/
gceSTATUS
gcSHADER_LoadHeader(
    IN gcSHADER Shader,
    IN gctPOINTER Buffer,
    IN gctSIZE_T BufferSize,
    OUT gctUINT32 * ShaderVersion
    );

/*******************************************************************************
**  gcSHADER_LoadKernel
**
**  Load a kernel function given by name into gcSHADER object
**
**  INPUT:
**
**      gcSHADER Shader
**          Pointer to a gcSHADER object.
**
**      gctSTRING KernelName
**          Pointer to a kernel function name
**
**  OUTPUT:
**      nothing
**
*/
gceSTATUS
gcSHADER_LoadKernel(
    IN gcSHADER Shader,
    IN gctSTRING KernelName
    );

/*******************************************************************************
**                                gcSHADER_Load
********************************************************************************
**
**	Load a gcSHADER object from a binary buffer.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gctPOINTER Buffer
**			Pointer to a binary buffer containg the shader data to load.
**
**		gctSIZE_T BufferSize
**			Number of bytes inside the binary buffer pointed to by 'Buffer'.
**
**	OUTPUT:
**
**		Nothing.
*/
gceSTATUS
gcSHADER_Load(
	IN gcSHADER Shader,
	IN gctPOINTER Buffer,
	IN gctSIZE_T BufferSize
	);

/*******************************************************************************
**                                gcSHADER_Save
********************************************************************************
**
**	Save a gcSHADER object to a binary buffer.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gctPOINTER Buffer
**			Pointer to a binary buffer to be used as storage for the gcSHADER
**			object.  If 'Buffer' is gcvNULL, the gcSHADER object will not be saved,
**			but the number of bytes required to hold the binary output for the
**			gcSHADER object will be returned.
**
**		gctSIZE_T * BufferSize
**			Pointer to a variable holding the number of bytes allocated in
**			'Buffer'.  Only valid if 'Buffer' is not gcvNULL.
**
**	OUTPUT:
**
**		gctSIZE_T * BufferSize
**			Pointer to a variable receiving the number of bytes required to hold
**			the binary form of the gcSHADER object.
*/
gceSTATUS
gcSHADER_Save(
	IN gcSHADER Shader,
	IN gctPOINTER Buffer,
	IN OUT gctSIZE_T * BufferSize
	);

/*******************************************************************************
**                                gcSHADER_LoadEx
********************************************************************************
**
**	Load a gcSHADER object from a binary buffer.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gctPOINTER Buffer
**			Pointer to a binary buffer containg the shader data to load.
**
**		gctSIZE_T BufferSize
**			Number of bytes inside the binary buffer pointed to by 'Buffer'.
**
**	OUTPUT:
**
**		Nothing.
*/
gceSTATUS
gcSHADER_LoadEx(
	IN gcSHADER Shader,
	IN gctPOINTER Buffer,
	IN gctSIZE_T BufferSize
	);

/*******************************************************************************
**                                gcSHADER_SaveEx
********************************************************************************
**
**	Save a gcSHADER object to a binary buffer.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gctPOINTER Buffer
**			Pointer to a binary buffer to be used as storage for the gcSHADER
**			object.  If 'Buffer' is gcvNULL, the gcSHADER object will not be saved,
**			but the number of bytes required to hold the binary output for the
**			gcSHADER object will be returned.
**
**		gctSIZE_T * BufferSize
**			Pointer to a variable holding the number of bytes allocated in
**			'Buffer'.  Only valid if 'Buffer' is not gcvNULL.
**
**	OUTPUT:
**
**		gctSIZE_T * BufferSize
**			Pointer to a variable receiving the number of bytes required to hold
**			the binary form of the gcSHADER object.
*/
gceSTATUS
gcSHADER_SaveEx(
	IN gcSHADER Shader,
	IN gctPOINTER Buffer,
	IN OUT gctSIZE_T * BufferSize
	);

/*******************************************************************************
**  gcSHADER_ReallocateAttributes
**
**  Reallocate an array of pointers to gcATTRIBUTE objects.
**
**  INPUT:
**
**      gcSHADER Shader
**          Pointer to a gcSHADER object.
**
**      gctSIZE_T Count
**          Array count to reallocate.  'Count' must be at least 1.
*/
gceSTATUS
gcSHADER_ReallocateAttributes(
    IN gcSHADER Shader,
    IN gctSIZE_T Count
    );

/*******************************************************************************
**							  gcSHADER_AddAttribute
********************************************************************************
**
**	Add an attribute to a gcSHADER object.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gctCONST_STRING Name
**			Name of the attribute to add.
**
**		gcSHADER_TYPE Type
**			Type of the attribute to add.
**
**		gctSIZE_T Length
**			Array length of the attribute to add.  'Length' must be at least 1.
**
**		gctBOOL IsTexture
**			gcvTRUE if the attribute is used as a texture coordinate, gcvFALSE if not.
**
**	OUTPUT:
**
**		gcATTRIBUTE * Attribute
**			Pointer to a variable receiving the gcATTRIBUTE object pointer.
*/
gceSTATUS
gcSHADER_AddAttribute(
	IN gcSHADER Shader,
	IN gctCONST_STRING Name,
	IN gcSHADER_TYPE Type,
	IN gctSIZE_T Length,
	IN gctBOOL IsTexture,
	OUT gcATTRIBUTE * Attribute
	);

/*******************************************************************************
**                         gcSHADER_GetAttributeCount
********************************************************************************
**
**	Get the number of attributes for this shader.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**	OUTPUT:
**
**		gctSIZE_T * Count
**			Pointer to a variable receiving the number of attributes.
*/
gceSTATUS
gcSHADER_GetAttributeCount(
	IN gcSHADER Shader,
	OUT gctSIZE_T * Count
	);

/*******************************************************************************
**                            gcSHADER_GetAttribute
********************************************************************************
**
**	Get the gcATTRIBUTE object poniter for an indexed attribute for this shader.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gctUINT Index
**			Index of the attribute to retrieve.
**
**	OUTPUT:
**
**		gcATTRIBUTE * Attribute
**			Pointer to a variable receiving the gcATTRIBUTE object pointer.
*/
gceSTATUS
gcSHADER_GetAttribute(
	IN gcSHADER Shader,
	IN gctUINT Index,
	OUT gcATTRIBUTE * Attribute
	);

/*******************************************************************************
**  gcSHADER_ReallocateUniforms
**
**  Reallocate an array of pointers to gcUNIFORM objects.
**
**  INPUT:
**
**      gcSHADER Shader
**          Pointer to a gcSHADER object.
**
**      gctSIZE_T Count
**          Array count to reallocate.  'Count' must be at least 1.
*/
gceSTATUS
gcSHADER_ReallocateUniforms(
    IN gcSHADER Shader,
    IN gctSIZE_T Count
    );

/*******************************************************************************
**							   gcSHADER_AddUniform
********************************************************************************
**
**	Add an uniform to a gcSHADER object.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gctCONST_STRING Name
**			Name of the uniform to add.
**
**		gcSHADER_TYPE Type
**			Type of the uniform to add.
**
**		gctSIZE_T Length
**			Array length of the uniform to add.  'Length' must be at least 1.
**
**	OUTPUT:
**
**		gcUNIFORM * Uniform
**			Pointer to a variable receiving the gcUNIFORM object pointer.
*/
gceSTATUS
gcSHADER_AddUniform(
	IN gcSHADER Shader,
	IN gctCONST_STRING Name,
	IN gcSHADER_TYPE Type,
	IN gctSIZE_T Length,
	OUT gcUNIFORM * Uniform
	);

/*******************************************************************************
**							   gcSHADER_AddPreRotationUniform
********************************************************************************
**
**	Add an uniform to a gcSHADER object.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gctCONST_STRING Name
**			Name of the uniform to add.
**
**		gcSHADER_TYPE Type
**			Type of the uniform to add.
**
**		gctSIZE_T Length
**			Array length of the uniform to add.  'Length' must be at least 1.
**
**		gctINT col
**			Which uniform.
**
**	OUTPUT:
**
**		gcUNIFORM * Uniform
**			Pointer to a variable receiving the gcUNIFORM object pointer.
*/
gceSTATUS
gcSHADER_AddPreRotationUniform(
	IN gcSHADER Shader,
	IN gctCONST_STRING Name,
	IN gcSHADER_TYPE Type,
	IN gctSIZE_T Length,
    IN gctINT col,
	OUT gcUNIFORM * Uniform
	);

/*******************************************************************************
**							   gcSHADER_AddUniformEx
********************************************************************************
**
**	Add an uniform to a gcSHADER object.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gctCONST_STRING Name
**			Name of the uniform to add.
**
**		gcSHADER_TYPE Type
**			Type of the uniform to add.
**
**      gcSHADER_PRECISION precision
**          Precision of the uniform to add.
**
**		gctSIZE_T Length
**			Array length of the uniform to add.  'Length' must be at least 1.
**
**	OUTPUT:
**
**		gcUNIFORM * Uniform
**			Pointer to a variable receiving the gcUNIFORM object pointer.
*/
gceSTATUS
gcSHADER_AddUniformEx(
	IN gcSHADER Shader,
	IN gctCONST_STRING Name,
	IN gcSHADER_TYPE Type,
    IN gcSHADER_PRECISION precision,
	IN gctSIZE_T Length,
	OUT gcUNIFORM * Uniform
	);

/*******************************************************************************
**							   gcSHADER_AddUniformEx1
********************************************************************************
**
**	Add an uniform to a gcSHADER object.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gctCONST_STRING Name
**			Name of the uniform to add.
**
**		gcSHADER_TYPE Type
**			Type of the uniform to add.
**
**      gcSHADER_PRECISION precision
**          Precision of the uniform to add.
**
**		gctSIZE_T Length
**			Array length of the uniform to add.  'Length' must be at least 1.
**
**      gcSHADER_VAR_CATEGORY varCategory
**          Variable category, normal or struct.
**
**      gctUINT16 numStructureElement
**          If struct, its element number.
**
**      gctINT16 parent
**          If struct, parent index in gcSHADER.variables.
**
**      gctINT16 prevSibling
**          If struct, previous sibling index in gcSHADER.variables.
**
**	OUTPUT:
**
**		gcUNIFORM * Uniform
**			Pointer to a variable receiving the gcUNIFORM object pointer.
**
**      gctINT16* ThisUniformIndex
**          Returned value about uniform index in gcSHADER.
*/
gceSTATUS
gcSHADER_AddUniformEx1(
	IN gcSHADER Shader,
	IN gctCONST_STRING Name,
	IN gcSHADER_TYPE Type,
    IN gcSHADER_PRECISION precision,
	IN gctSIZE_T Length,
    IN gctINT    IsArray,
    IN gcSHADER_VAR_CATEGORY varCategory,
    IN gctUINT16 numStructureElement,
    IN gctINT16 parent,
    IN gctINT16 prevSibling,
    OUT gctINT16* ThisUniformIndex,
	OUT gcUNIFORM * Uniform
	);

/*******************************************************************************
**                          gcSHADER_GetUniformCount
********************************************************************************
**
**	Get the number of uniforms for this shader.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**	OUTPUT:
**
**		gctSIZE_T * Count
**			Pointer to a variable receiving the number of uniforms.
*/
gceSTATUS
gcSHADER_GetUniformCount(
	IN gcSHADER Shader,
	OUT gctSIZE_T * Count
	);

/*******************************************************************************
**                         gcSHADER_GetPreRotationUniform
********************************************************************************
**
**	Get the preRotate Uniform.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**	OUTPUT:
**
**		gcUNIFORM ** pUniform
**			Pointer to a preRotation uniforms array.
*/
gceSTATUS
gcSHADER_GetPreRotationUniform(
	IN gcSHADER Shader,
	OUT gcUNIFORM ** pUniform
	);

/*******************************************************************************
**                             gcSHADER_GetUniform
********************************************************************************
**
**	Get the gcUNIFORM object pointer for an indexed uniform for this shader.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gctUINT Index
**			Index of the uniform to retrieve.
**
**	OUTPUT:
**
**		gcUNIFORM * Uniform
**			Pointer to a variable receiving the gcUNIFORM object pointer.
*/
gceSTATUS
gcSHADER_GetUniform(
	IN gcSHADER Shader,
	IN gctUINT Index,
	OUT gcUNIFORM * Uniform
	);


/*******************************************************************************
**                             gcSHADER_GetUniformIndexingRange
********************************************************************************
**
**	Get the gcUNIFORM object pointer for an indexed uniform for this shader.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gctINT uniformIndex
**			Index of the start uniform.
**
**		gctINT offset
**			Offset to indexing.
**
**	OUTPUT:
**
**		gctINT * LastUniformIndex
**			Pointer to index of last uniform in indexing range.
**
**		gctINT * OffsetUniformIndex
**			Pointer to index of uniform that indexing at offset.
**
**		gctINT * DeviationInOffsetUniform
**			Pointer to offset in uniform picked up.
*/
gceSTATUS
gcSHADER_GetUniformIndexingRange(
	IN gcSHADER Shader,
	IN gctINT uniformIndex,
    IN gctINT offset,
	OUT gctINT * LastUniformIndex,
    OUT gctINT * OffsetUniformIndex,
    OUT gctINT * DeviationInOffsetUniform
	);

/*******************************************************************************
**  gcSHADER_GetKernelFucntion
**
**  Get the gcKERNEL_FUNCTION object pointer for an indexed kernel function for this shader.
**
**  INPUT:
**
**      gcSHADER Shader
**          Pointer to a gcSHADER object.
**
**      gctUINT Index
**          Index of kernel function to retreive the name for.
**
**  OUTPUT:
**
**      gcKERNEL_FUNCTION * KernelFunction
**          Pointer to a variable receiving the gcKERNEL_FUNCTION object pointer.
*/
gceSTATUS
gcSHADER_GetKernelFunction(
    IN gcSHADER Shader,
    IN gctUINT Index,
    OUT gcKERNEL_FUNCTION * KernelFunction
    );

gceSTATUS
gcSHADER_GetKernelFunctionByName(
	IN gcSHADER Shader,
    IN gctSTRING KernelName,
    OUT gcKERNEL_FUNCTION * KernelFunction
    );
/*******************************************************************************
**  gcSHADER_GetKernelFunctionCount
**
**  Get the number of kernel functions for this shader.
**
**  INPUT:
**
**      gcSHADER Shader
**          Pointer to a gcSHADER object.
**
**  OUTPUT:
**
**      gctSIZE_T * Count
**          Pointer to a variable receiving the number of kernel functions.
*/
gceSTATUS
gcSHADER_GetKernelFunctionCount(
    IN gcSHADER Shader,
    OUT gctSIZE_T * Count
    );

/*******************************************************************************
**  gcSHADER_ReallocateOutputs
**
**  Reallocate an array of pointers to gcOUTPUT objects.
**
**  INPUT:
**
**      gcSHADER Shader
**          Pointer to a gcSHADER object.
**
**      gctSIZE_T Count
**          Array count to reallocate.  'Count' must be at least 1.
*/
gceSTATUS
gcSHADER_ReallocateOutputs(
    IN gcSHADER Shader,
    IN gctSIZE_T Count
    );

/*******************************************************************************
**							   gcSHADER_AddOutput
********************************************************************************
**
**	Add an output to a gcSHADER object.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gctCONST_STRING Name
**			Name of the output to add.
**
**		gcSHADER_TYPE Type
**			Type of the output to add.
**
**		gctSIZE_T Length
**			Array length of the output to add.  'Length' must be at least 1.
**
**		gctUINT16 TempRegister
**			Temporary register index that holds the output value.
**
**	OUTPUT:
**
**		Nothing.
*/
gceSTATUS
gcSHADER_AddOutput(
	IN gcSHADER Shader,
	IN gctCONST_STRING Name,
	IN gcSHADER_TYPE Type,
	IN gctSIZE_T Length,
	IN gctUINT16 TempRegister
	);

gceSTATUS
gcSHADER_AddOutputIndexed(
	IN gcSHADER Shader,
	IN gctCONST_STRING Name,
	IN gctSIZE_T Index,
	IN gctUINT16 TempIndex
	);

/*******************************************************************************
**							 gcSHADER_GetOutputCount
********************************************************************************
**
**	Get the number of outputs for this shader.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**	OUTPUT:
**
**		gctSIZE_T * Count
**			Pointer to a variable receiving the number of outputs.
*/
gceSTATUS
gcSHADER_GetOutputCount(
	IN gcSHADER Shader,
	OUT gctSIZE_T * Count
	);

/*******************************************************************************
**							   gcSHADER_GetOutput
********************************************************************************
**
**	Get the gcOUTPUT object pointer for an indexed output for this shader.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gctUINT Index
**			Index of output to retrieve.
**
**	OUTPUT:
**
**		gcOUTPUT * Output
**			Pointer to a variable receiving the gcOUTPUT object pointer.
*/
gceSTATUS
gcSHADER_GetOutput(
	IN gcSHADER Shader,
	IN gctUINT Index,
	OUT gcOUTPUT * Output
	);


/*******************************************************************************
**							   gcSHADER_GetOutputByName
********************************************************************************
**
**	Get the gcOUTPUT object pointer for this shader by output name.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gctSTRING name
**			Name of output to retrieve.
**
**      gctSIZE_T nameLength
**          Length of name to retrieve
**
**	OUTPUT:
**
**		gcOUTPUT * Output
**			Pointer to a variable receiving the gcOUTPUT object pointer.
*/
gceSTATUS
gcSHADER_GetOutputByName(
	IN gcSHADER Shader,
	IN gctSTRING name,
    IN gctSIZE_T nameLength,
	OUT gcOUTPUT * Output
	);

/*******************************************************************************
**  gcSHADER_ReallocateVariables
**
**  Reallocate an array of pointers to gcVARIABLE objects.
**
**  INPUT:
**
**      gcSHADER Shader
**          Pointer to a gcSHADER object.
**
**      gctSIZE_T Count
**          Array count to reallocate.  'Count' must be at least 1.
*/
gceSTATUS
gcSHADER_ReallocateVariables(
    IN gcSHADER Shader,
    IN gctSIZE_T Count
    );

/*******************************************************************************
**							   gcSHADER_AddVariable
********************************************************************************
**
**	Add a variable to a gcSHADER object.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gctCONST_STRING Name
**			Name of the variable to add.
**
**		gcSHADER_TYPE Type
**			Type of the variable to add.
**
**		gctSIZE_T Length
**			Array length of the variable to add.  'Length' must be at least 1.
**
**		gctUINT16 TempRegister
**			Temporary register index that holds the variable value.
**
**	OUTPUT:
**
**		Nothing.
*/
gceSTATUS
gcSHADER_AddVariable(
	IN gcSHADER Shader,
	IN gctCONST_STRING Name,
	IN gcSHADER_TYPE Type,
	IN gctSIZE_T Length,
	IN gctUINT16 TempRegister
	);


/*******************************************************************************
**  gcSHADER_AddVariableEx
********************************************************************************
**
**  Add a variable to a gcSHADER object.
**
**  INPUT:
**
**      gcSHADER Shader
**          Pointer to a gcSHADER object.
**
**      gctCONST_STRING Name
**          Name of the variable to add.
**
**      gcSHADER_TYPE Type
**          Type of the variable to add.
**
**      gctSIZE_T Length
**          Array length of the variable to add.  'Length' must be at least 1.
**
**      gctUINT16 TempRegister
**          Temporary register index that holds the variable value.
**
**      gcSHADER_VAR_CATEGORY varCategory
**          Variable category, normal or struct.
**
**      gctUINT16 numStructureElement
**          If struct, its element number.
**
**      gctINT16 parent
**          If struct, parent index in gcSHADER.variables.
**
**      gctINT16 prevSibling
**          If struct, previous sibling index in gcSHADER.variables.
**
**  OUTPUT:
**
**      gctINT16* ThisVarIndex
**          Returned value about variable index in gcSHADER.
*/
gceSTATUS
gcSHADER_AddVariableEx(
    IN gcSHADER Shader,
    IN gctCONST_STRING Name,
    IN gcSHADER_TYPE Type,
    IN gctSIZE_T Length,
    IN gctUINT16 TempRegister,
    IN gcSHADER_VAR_CATEGORY varCategory,
    IN gctUINT16 numStructureElement,
    IN gctINT16 parent,
    IN gctINT16 prevSibling,
    OUT gctINT16* ThisVarIndex
    );

/*******************************************************************************
**  gcSHADER_UpdateVariable
********************************************************************************
**
**  Update a variable to a gcSHADER object.
**
**  INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gctUINT Index
**			Index of variable to retrieve.
**
**		gceVARIABLE_UPDATE_FLAGS flag
**			Flag which property of variable will be updated.
**
**      gctUINT newValue
**          New value to update.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gcSHADER_UpdateVariable(
    IN gcSHADER Shader,
    IN gctUINT Index,
    IN gceVARIABLE_UPDATE_FLAGS flag,
    IN gctUINT newValue
    );

/*******************************************************************************
**							 gcSHADER_GetVariableCount
********************************************************************************
**
**	Get the number of variables for this shader.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**	OUTPUT:
**
**		gctSIZE_T * Count
**			Pointer to a variable receiving the number of variables.
*/
gceSTATUS
gcSHADER_GetVariableCount(
	IN gcSHADER Shader,
	OUT gctSIZE_T * Count
	);

/*******************************************************************************
**							   gcSHADER_GetVariable
********************************************************************************
**
**	Get the gcVARIABLE object pointer for an indexed variable for this shader.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gctUINT Index
**			Index of variable to retrieve.
**
**	OUTPUT:
**
**		gcVARIABLE * Variable
**			Pointer to a variable receiving the gcVARIABLE object pointer.
*/
gceSTATUS
gcSHADER_GetVariable(
	IN gcSHADER Shader,
	IN gctUINT Index,
	OUT gcVARIABLE * Variable
	);

/*******************************************************************************
**							   gcSHADER_GetVariableIndexingRange
********************************************************************************
**
**	Get the gcVARIABLE indexing range.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gcVARIABLE variable
**			Start variable.
**
**		gctBOOL whole
**			Indicate whether maximum indexing range is queried
**
**	OUTPUT:
**
**		gctUINT *Start
**			Pointer to range start (temp register index).
**
**		gctUINT *End
**			Pointer to range end (temp register index).
*/
gceSTATUS
gcSHADER_GetVariableIndexingRange(
	IN gcSHADER Shader,
    IN gcVARIABLE variable,
    IN gctBOOL whole,
    OUT gctUINT *Start,
    OUT gctUINT *End
	);

/*******************************************************************************
**							   gcSHADER_AddOpcode
********************************************************************************
**
**	Add an opcode to a gcSHADER object.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gcSL_OPCODE Opcode
**			Opcode to add.
**
**		gctUINT16 TempRegister
**			Temporary register index that acts as the target of the opcode.
**
**		gctUINT8 Enable
**			Write enable bits for the temporary register that acts as the target
**			of the opcode.
**
**		gcSL_FORMAT Format
**			Format of the temporary register.
**
**	OUTPUT:
**
**		Nothing.
*/
gceSTATUS
gcSHADER_AddOpcode(
	IN gcSHADER Shader,
	IN gcSL_OPCODE Opcode,
	IN gctUINT16 TempRegister,
	IN gctUINT8 Enable,
	IN gcSL_FORMAT Format
	);

gceSTATUS
gcSHADER_AddOpcode2(
	IN gcSHADER Shader,
	IN gcSL_OPCODE Opcode,
	IN gcSL_CONDITION Condition,
	IN gctUINT16 TempRegister,
	IN gctUINT8 Enable,
	IN gcSL_FORMAT Format
	);

/*******************************************************************************
**							gcSHADER_AddOpcodeIndexed
********************************************************************************
**
**	Add an opcode to a gcSHADER object that writes to an dynamically indexed
**	target.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gcSL_OPCODE Opcode
**			Opcode to add.
**
**		gctUINT16 TempRegister
**			Temporary register index that acts as the target of the opcode.
**
**		gctUINT8 Enable
**			Write enable bits  for the temporary register that acts as the
**			target of the opcode.
**
**		gcSL_INDEXED Mode
**			Location of the dynamic index inside the temporary register.  Valid
**			values can be:
**
**				gcSL_INDEXED_X - Use x component of the temporary register.
**				gcSL_INDEXED_Y - Use y component of the temporary register.
**				gcSL_INDEXED_Z - Use z component of the temporary register.
**				gcSL_INDEXED_W - Use w component of the temporary register.
**
**		gctUINT16 IndexRegister
**			Temporary register index that holds the dynamic index.
**
**		gcSL_FORMAT Format
**			Format of the temporary register.
**
**	OUTPUT:
**
**		Nothing.
*/
gceSTATUS
gcSHADER_AddOpcodeIndexed(
	IN gcSHADER Shader,
	IN gcSL_OPCODE Opcode,
	IN gctUINT16 TempRegister,
	IN gctUINT8 Enable,
	IN gcSL_INDEXED Mode,
	IN gctUINT16 IndexRegister,
	IN gcSL_FORMAT Format
	);

/*******************************************************************************
**  gcSHADER_AddOpcodeConditionIndexed
**
**  Add an opcode to a gcSHADER object that writes to an dynamically indexed
**  target.
**
**  INPUT:
**
**      gcSHADER Shader
**          Pointer to a gcSHADER object.
**
**      gcSL_OPCODE Opcode
**          Opcode to add.
**
**      gcSL_CONDITION Condition
**          Condition to check.
**
**      gctUINT16 TempRegister
**          Temporary register index that acts as the target of the opcode.
**
**      gctUINT8 Enable
**          Write enable bits  for the temporary register that acts as the
**          target of the opcode.
**
**      gcSL_INDEXED Indexed
**          Location of the dynamic index inside the temporary register.  Valid
**          values can be:
**
**              gcSL_INDEXED_X - Use x component of the temporary register.
**              gcSL_INDEXED_Y - Use y component of the temporary register.
**              gcSL_INDEXED_Z - Use z component of the temporary register.
**              gcSL_INDEXED_W - Use w component of the temporary register.
**
**      gctUINT16 IndexRegister
**          Temporary register index that holds the dynamic index.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gcSHADER_AddOpcodeConditionIndexed(
    IN gcSHADER Shader,
    IN gcSL_OPCODE Opcode,
    IN gcSL_CONDITION Condition,
    IN gctUINT16 TempRegister,
    IN gctUINT8 Enable,
    IN gcSL_INDEXED Indexed,
    IN gctUINT16 IndexRegister,
    IN gcSL_FORMAT Format
    );

/*******************************************************************************
**						  gcSHADER_AddOpcodeConditional
********************************************************************************
**
**	Add an conditional opcode to a gcSHADER object.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gcSL_OPCODE Opcode
**			Opcode to add.
**
**		gcSL_CONDITION Condition
**			Condition that needs to evaluate to gcvTRUE in order for the opcode to
**			execute.
**
**		gctUINT Label
**			Target label if 'Condition' evaluates to gcvTRUE.
**
**	OUTPUT:
**
**		Nothing.
*/
gceSTATUS
gcSHADER_AddOpcodeConditional(
	IN gcSHADER Shader,
	IN gcSL_OPCODE Opcode,
	IN gcSL_CONDITION Condition,
	IN gctUINT Label
	);

/*******************************************************************************
**  gcSHADER_AddOpcodeConditionalFormatted
**
**  Add an conditional jump or call opcode to a gcSHADER object.
**
**  INPUT:
**
**      gcSHADER Shader
**          Pointer to a gcSHADER object.
**
**      gcSL_OPCODE Opcode
**          Opcode to add.
**
**      gcSL_CONDITION Condition
**          Condition that needs to evaluate to gcvTRUE in order for the opcode to
**          execute.
**
**      gcSL_FORMAT Format
**          Format of conditional operands
**
**      gctUINT Label
**          Target label if 'Condition' evaluates to gcvTRUE.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gcSHADER_AddOpcodeConditionalFormatted(
    IN gcSHADER Shader,
    IN gcSL_OPCODE Opcode,
    IN gcSL_CONDITION Condition,
    IN gcSL_FORMAT Format,
    IN gctUINT Label
    );

/*******************************************************************************
**  gcSHADER_AddOpcodeConditionalFormattedEnable
**
**  Add an conditional jump or call opcode to a gcSHADER object.
**
**  INPUT:
**
**      gcSHADER Shader
**          Pointer to a gcSHADER object.
**
**      gcSL_OPCODE Opcode
**          Opcode to add.
**
**      gcSL_CONDITION Condition
**          Condition that needs to evaluate to gcvTRUE in order for the opcode to
**          execute.
**
**      gcSL_FORMAT Format
**          Format of conditional operands
**
**      gctUINT8 Enable
**          Write enable value for the target of the opcode.
**
**      gctUINT Label
**          Target label if 'Condition' evaluates to gcvTRUE.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gcSHADER_AddOpcodeConditionalFormattedEnable(
    IN gcSHADER Shader,
    IN gcSL_OPCODE Opcode,
    IN gcSL_CONDITION Condition,
    IN gcSL_FORMAT Format,
    IN gctUINT8 Enable,
    IN gctUINT Label
    );

/*******************************************************************************
**								gcSHADER_AddLabel
********************************************************************************
**
**	Define a label at the current instruction of a gcSHADER object.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gctUINT Label
**			Label to define.
**
**	OUTPUT:
**
**		Nothing.
*/
gceSTATUS
gcSHADER_AddLabel(
	IN gcSHADER Shader,
	IN gctUINT Label
	);

/*******************************************************************************
**							   gcSHADER_AddSource
********************************************************************************
**
**	Add a source operand to a gcSHADER object.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gcSL_TYPE Type
**			Type of the source operand.
**
**		gctUINT16 SourceIndex
**			Index of the source operand.
**
**		gctUINT8 Swizzle
**			x, y, z, and w swizzle values packed into one 8-bit value.
**
**		gcSL_FORMAT Format
**			Format of the source operand.
**
**	OUTPUT:
**
**		Nothing.
*/
gceSTATUS
gcSHADER_AddSource(
	IN gcSHADER Shader,
	IN gcSL_TYPE Type,
	IN gctUINT16 SourceIndex,
	IN gctUINT8 Swizzle,
	IN gcSL_FORMAT Format
	);

/*******************************************************************************
**							gcSHADER_AddSourceIndexed
********************************************************************************
**
**	Add a dynamically indexed source operand to a gcSHADER object.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gcSL_TYPE Type
**			Type of the source operand.
**
**		gctUINT16 SourceIndex
**			Index of the source operand.
**
**		gctUINT8 Swizzle
**			x, y, z, and w swizzle values packed into one 8-bit value.
**
**		gcSL_INDEXED Mode
**			Addressing mode for the index.
**
**		gctUINT16 IndexRegister
**			Temporary register index that holds the dynamic index.
**
**		gcSL_FORMAT Format
**			Format of the source operand.
**
**	OUTPUT:
**
**		Nothing.
*/
gceSTATUS
gcSHADER_AddSourceIndexed(
	IN gcSHADER Shader,
	IN gcSL_TYPE Type,
	IN gctUINT16 SourceIndex,
	IN gctUINT8 Swizzle,
	IN gcSL_INDEXED Mode,
	IN gctUINT16 IndexRegister,
	IN gcSL_FORMAT Format
	);

/*******************************************************************************
**						   gcSHADER_AddSourceAttribute
********************************************************************************
**
**	Add an attribute as a source operand to a gcSHADER object.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gcATTRIBUTE Attribute
**			Pointer to a gcATTRIBUTE object.
**
**		gctUINT8 Swizzle
**			x, y, z, and w swizzle values packed into one 8-bit value.
**
**		gctINT Index
**			Static index into the attribute in case the attribute is a matrix
**			or array.
**
**	OUTPUT:
**
**		Nothing.
*/
gceSTATUS
gcSHADER_AddSourceAttribute(
	IN gcSHADER Shader,
	IN gcATTRIBUTE Attribute,
	IN gctUINT8 Swizzle,
	IN gctINT Index
	);

/*******************************************************************************
**						   gcSHADER_AddSourceAttributeIndexed
********************************************************************************
**
**	Add an indexed attribute as a source operand to a gcSHADER object.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gcATTRIBUTE Attribute
**			Pointer to a gcATTRIBUTE object.
**
**		gctUINT8 Swizzle
**			x, y, z, and w swizzle values packed into one 8-bit value.
**
**		gctINT Index
**			Static index into the attribute in case the attribute is a matrix
**			or array.
**
**		gcSL_INDEXED Mode
**			Addressing mode of the dynamic index.
**
**		gctUINT16 IndexRegister
**			Temporary register index that holds the dynamic index.
**
**	OUTPUT:
**
**		Nothing.
*/
gceSTATUS
gcSHADER_AddSourceAttributeIndexed(
	IN gcSHADER Shader,
	IN gcATTRIBUTE Attribute,
	IN gctUINT8 Swizzle,
	IN gctINT Index,
	IN gcSL_INDEXED Mode,
	IN gctUINT16 IndexRegister
	);

/*******************************************************************************
**							gcSHADER_AddSourceUniform
********************************************************************************
**
**	Add a uniform as a source operand to a gcSHADER object.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gcUNIFORM Uniform
**			Pointer to a gcUNIFORM object.
**
**		gctUINT8 Swizzle
**			x, y, z, and w swizzle values packed into one 8-bit value.
**
**		gctINT Index
**			Static index into the uniform in case the uniform is a matrix or
**			array.
**
**	OUTPUT:
**
**		Nothing.
*/
gceSTATUS
gcSHADER_AddSourceUniform(
	IN gcSHADER Shader,
	IN gcUNIFORM Uniform,
	IN gctUINT8 Swizzle,
	IN gctINT Index
	);

/*******************************************************************************
**						gcSHADER_AddSourceUniformIndexed
********************************************************************************
**
**	Add an indexed uniform as a source operand to a gcSHADER object.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gcUNIFORM Uniform
**			Pointer to a gcUNIFORM object.
**
**		gctUINT8 Swizzle
**			x, y, z, and w swizzle values packed into one 8-bit value.
**
**		gctINT Index
**			Static index into the uniform in case the uniform is a matrix or
**			array.
**
**		gcSL_INDEXED Mode
**			Addressing mode of the dynamic index.
**
**		gctUINT16 IndexRegister
**			Temporary register index that holds the dynamic index.
**
**	OUTPUT:
**
**		Nothing.
*/
gceSTATUS
gcSHADER_AddSourceUniformIndexed(
	IN gcSHADER Shader,
	IN gcUNIFORM Uniform,
	IN gctUINT8 Swizzle,
	IN gctINT Index,
	IN gcSL_INDEXED Mode,
	IN gctUINT16 IndexRegister
	);

gceSTATUS
gcSHADER_AddSourceSamplerIndexed(
	IN gcSHADER Shader,
	IN gctUINT8 Swizzle,
	IN gcSL_INDEXED Mode,
	IN gctUINT16 IndexRegister
	);

gceSTATUS
gcSHADER_AddSourceAttributeFormatted(
    IN gcSHADER Shader,
    IN gcATTRIBUTE Attribute,
    IN gctUINT8 Swizzle,
    IN gctINT Index,
    IN gcSL_FORMAT Format
    );

gceSTATUS
gcSHADER_AddSourceAttributeIndexedFormatted(
    IN gcSHADER Shader,
    IN gcATTRIBUTE Attribute,
    IN gctUINT8 Swizzle,
    IN gctINT Index,
    IN gcSL_INDEXED Mode,
    IN gctUINT16 IndexRegister,
    IN gcSL_FORMAT Format
    );

gceSTATUS
gcSHADER_AddSourceUniformFormatted(
    IN gcSHADER Shader,
    IN gcUNIFORM Uniform,
    IN gctUINT8 Swizzle,
    IN gctINT Index,
    IN gcSL_FORMAT Format
    );

gceSTATUS
gcSHADER_AddSourceUniformIndexedFormatted(
    IN gcSHADER Shader,
    IN gcUNIFORM Uniform,
    IN gctUINT8 Swizzle,
    IN gctINT Index,
    IN gcSL_INDEXED Mode,
    IN gctUINT16 IndexRegister,
    IN gcSL_FORMAT Format
    );

gceSTATUS
gcSHADER_AddSourceSamplerIndexedFormatted(
    IN gcSHADER Shader,
    IN gctUINT8 Swizzle,
    IN gcSL_INDEXED Mode,
    IN gctUINT16 IndexRegister,
    IN gcSL_FORMAT Format
    );

/*******************************************************************************
**						   gcSHADER_AddSourceConstant
********************************************************************************
**
**	Add a constant floating point value as a source operand to a gcSHADER
**	object.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gctFLOAT Constant
**			Floating point constant.
**
**	OUTPUT:
**
**		Nothing.
*/
gceSTATUS
gcSHADER_AddSourceConstant(
	IN gcSHADER Shader,
	IN gctFLOAT Constant
	);

/*******************************************************************************
**			                   gcSHADER_AddSourceConstantFormatted
********************************************************************************
**
**	Add a constant value as a source operand to a gcSHADER
**	object.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		void * Constant
**			Pointer to constant.
**
**		gcSL_FORMAT Format
**
**	OUTPUT:
**
**		Nothing.
*/
gceSTATUS
gcSHADER_AddSourceConstantFormatted(
	IN gcSHADER Shader,
	IN void *Constant,
	IN gcSL_FORMAT Format
	);

/*******************************************************************************
**								  gcSHADER_Pack
********************************************************************************
**
**	Pack a dynamically created gcSHADER object by trimming the allocated arrays
**	and resolving all the labeling.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**	OUTPUT:
**
**		Nothing.
*/
gceSTATUS
gcSHADER_Pack(
	IN gcSHADER Shader
	);

/*******************************************************************************
**								gcSHADER_SetOptimizationOption
********************************************************************************
**
**	Set optimization option of a gcSHADER object.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object.
**
**		gctUINT OptimizationOption
**			Optimization option.  Can be one of the following:
**
**				0						- No optimization.
**				1						- Full optimization.
**				Other value				- For optimizer testing.
**
**	OUTPUT:
**
**		Nothing.
*/
gceSTATUS
gcSHADER_SetOptimizationOption(
	IN gcSHADER Shader,
	IN gctUINT OptimizationOption
	);

/*******************************************************************************
**  gcSHADER_ReallocateFunctions
**
**  Reallocate an array of pointers to gcFUNCTION objects.
**
**  INPUT:
**
**      gcSHADER Shader
**          Pointer to a gcSHADER object.
**
**      gctSIZE_T Count
**          Array count to reallocate.  'Count' must be at least 1.
*/
gceSTATUS
gcSHADER_ReallocateFunctions(
    IN gcSHADER Shader,
    IN gctSIZE_T Count
    );

gceSTATUS
gcSHADER_AddFunction(
	IN gcSHADER Shader,
	IN gctCONST_STRING Name,
	OUT gcFUNCTION * Function
	);

gceSTATUS
gcSHADER_ReallocateKernelFunctions(
    IN gcSHADER Shader,
    IN gctSIZE_T Count
    );

gceSTATUS
gcSHADER_AddKernelFunction(
	IN gcSHADER Shader,
	IN gctCONST_STRING Name,
	OUT gcKERNEL_FUNCTION * KernelFunction
	);

gceSTATUS
gcSHADER_BeginFunction(
	IN gcSHADER Shader,
	IN gcFUNCTION Function
	);

gceSTATUS
gcSHADER_EndFunction(
	IN gcSHADER Shader,
	IN gcFUNCTION Function
	);

gceSTATUS
gcSHADER_BeginKernelFunction(
	IN gcSHADER Shader,
	IN gcKERNEL_FUNCTION KernelFunction
	);

gceSTATUS
gcSHADER_EndKernelFunction(
	IN gcSHADER Shader,
	IN gcKERNEL_FUNCTION KernelFunction,
	IN gctSIZE_T LocalMemorySize
	);

gceSTATUS
gcSHADER_SetMaxKernelFunctionArgs(
    IN gcSHADER Shader,
    IN gctUINT32 MaxKernelFunctionArgs
    );

/*******************************************************************************
**  gcSHADER_SetConstantMemorySize
**
**  Set the constant memory address space size of a gcSHADER object.
**
**  INPUT:
**
**      gcSHADER Shader
**          Pointer to a gcSHADER object.
**
**      gctSIZE_T ConstantMemorySize
**          Constant memory size in bytes
**
**      gctCHAR *ConstantMemoryBuffer
**          Constant memory buffer
*/
gceSTATUS
gcSHADER_SetConstantMemorySize(
    IN gcSHADER Shader,
    IN gctSIZE_T ConstantMemorySize,
    IN gctCHAR * ConstantMemoryBuffer
    );

/*******************************************************************************
**  gcSHADER_GetConstantMemorySize
**
**  Set the constant memory address space size of a gcSHADER object.
**
**  INPUT:
**
**      gcSHADER Shader
**          Pointer to a gcSHADER object.
**
**  OUTPUT:
**
**      gctSIZE_T * ConstantMemorySize
**          Pointer to a variable receiving constant memory size in bytes
**
**      gctCHAR **ConstantMemoryBuffer.
**          Pointer to a variable for returned shader constant memory buffer.
*/
gceSTATUS
gcSHADER_GetConstantMemorySize(
    IN gcSHADER Shader,
    OUT gctSIZE_T * ConstantMemorySize,
    OUT gctCHAR ** ConstantMemoryBuffer
    );

/*******************************************************************************
**  gcSHADER_SetPrivateMemorySize
**
**  Set the private memory address space size of a gcSHADER object.
**
**  INPUT:
**
**      gcSHADER Shader
**          Pointer to a gcSHADER object.
**
**      gctSIZE_T PrivateMemorySize
**          Private memory size in bytes
*/
gceSTATUS
gcSHADER_SetPrivateMemorySize(
    IN gcSHADER Shader,
    IN gctSIZE_T PrivateMemorySize
    );

/*******************************************************************************
**  gcSHADER_GetPrivateMemorySize
**
**  Set the private memory address space size of a gcSHADER object.
**
**  INPUT:
**
**      gcSHADER Shader
**          Pointer to a gcSHADER object.
**
**  OUTPUT:
**
**      gctSIZE_T * PrivateMemorySize
**          Pointer to a variable receiving private memory size in bytes
*/
gceSTATUS
gcSHADER_GetPrivateMemorySize(
    IN gcSHADER Shader,
    OUT gctSIZE_T * PrivateMemorySize
    );

/*******************************************************************************
**  gcSHADER_SetLocalMemorySize
**
**  Set the local memory address space size of a gcSHADER object.
**
**  INPUT:
**
**      gcSHADER Shader
**          Pointer to a gcSHADER object.
**
**      gctSIZE_T LocalMemorySize
**          Local memory size in bytes
*/
gceSTATUS
gcSHADER_SetLocalMemorySize(
    IN gcSHADER Shader,
    IN gctSIZE_T LocalMemorySize
    );

/*******************************************************************************
**  gcSHADER_GetLocalMemorySize
**
**  Set the local memory address space size of a gcSHADER object.
**
**  INPUT:
**
**      gcSHADER Shader
**          Pointer to a gcSHADER object.
**
**  OUTPUT:
**
**      gctSIZE_T * LocalMemorySize
**          Pointer to a variable receiving lcoal memory size in bytes
*/
gceSTATUS
gcSHADER_GetLocalMemorySize(
    IN gcSHADER Shader,
    OUT gctSIZE_T * LocalMemorySize
    );


/*******************************************************************************
**  gcSHADER_CheckValidity
**
**  Check validity for a gcSHADER object.
**
**  INPUT:
**
**      gcSHADER Shader
**          Pointer to a gcSHADER object.
**
*/
gceSTATUS
gcSHADER_CheckValidity(
    IN gcSHADER Shader
    );

#if gcdUSE_WCLIP_PATCH
gceSTATUS
gcATTRIBUTE_IsPosition(
        IN gcATTRIBUTE Attribute,
        OUT gctBOOL * IsPosition
        );
#endif

/*******************************************************************************
**                             gcATTRIBUTE_GetType
********************************************************************************
**
**	Get the type and array length of a gcATTRIBUTE object.
**
**	INPUT:
**
**		gcATTRIBUTE Attribute
**			Pointer to a gcATTRIBUTE object.
**
**	OUTPUT:
**
**		gcSHADER_TYPE * Type
**			Pointer to a variable receiving the type of the attribute.  'Type'
**			can be gcvNULL, in which case no type will be returned.
**
**		gctSIZE_T * ArrayLength
**			Pointer to a variable receiving the length of the array if the
**			attribute was declared as an array.  If the attribute was not
**			declared as an array, the array length will be 1.  'ArrayLength' can
**			be gcvNULL, in which case no array length will be returned.
*/
gceSTATUS
gcATTRIBUTE_GetType(
	IN gcATTRIBUTE Attribute,
	OUT gcSHADER_TYPE * Type,
	OUT gctSIZE_T * ArrayLength
	);

/*******************************************************************************
**                            gcATTRIBUTE_GetName
********************************************************************************
**
**	Get the name of a gcATTRIBUTE object.
**
**	INPUT:
**
**		gcATTRIBUTE Attribute
**			Pointer to a gcATTRIBUTE object.
**
**	OUTPUT:
**
**		gctSIZE_T * Length
**			Pointer to a variable receiving the length of the attribute name.
**			'Length' can be gcvNULL, in which case no length will be returned.
**
**		gctCONST_STRING * Name
**			Pointer to a variable receiving the pointer to the attribute name.
**			'Name' can be gcvNULL, in which case no name will be returned.
*/
gceSTATUS
gcATTRIBUTE_GetName(
	IN gcATTRIBUTE Attribute,
	OUT gctSIZE_T * Length,
	OUT gctCONST_STRING * Name
	);

/*******************************************************************************
**                            gcATTRIBUTE_IsEnabled
********************************************************************************
**
**	Query the enabled state of a gcATTRIBUTE object.
**
**	INPUT:
**
**		gcATTRIBUTE Attribute
**			Pointer to a gcATTRIBUTE object.
**
**	OUTPUT:
**
**		gctBOOL * Enabled
**			Pointer to a variable receiving the enabled state of the attribute.
*/
gceSTATUS
gcATTRIBUTE_IsEnabled(
	IN gcATTRIBUTE Attribute,
	OUT gctBOOL * Enabled
	);

gceSTATUS
gcATTRIBUTE_GetIndex(
    IN gcATTRIBUTE Attribute,
    OUT gctUINT16 * Index
    );

/*******************************************************************************
**                              gcUNIFORM_GetType
********************************************************************************
**
**	Get the type and array length of a gcUNIFORM object.
**
**	INPUT:
**
**		gcUNIFORM Uniform
**			Pointer to a gcUNIFORM object.
**
**	OUTPUT:
**
**		gcSHADER_TYPE * Type
**			Pointer to a variable receiving the type of the uniform.  'Type' can
**			be gcvNULL, in which case no type will be returned.
**
**		gctSIZE_T * ArrayLength
**			Pointer to a variable receiving the length of the array if the
**			uniform was declared as an array.  If the uniform was not declared
**			as an array, the array length will be 1.  'ArrayLength' can be gcvNULL,
**			in which case no array length will be returned.
*/
gceSTATUS
gcUNIFORM_GetType(
	IN gcUNIFORM Uniform,
	OUT gcSHADER_TYPE * Type,
	OUT gctSIZE_T * ArrayLength
	);

/*******************************************************************************
**                              gcUNIFORM_GetTypeEx
********************************************************************************
**
**	Get the type and array length of a gcUNIFORM object.
**
**	INPUT:
**
**		gcUNIFORM Uniform
**			Pointer to a gcUNIFORM object.
**
**	OUTPUT:
**
**		gcSHADER_TYPE * Type
**			Pointer to a variable receiving the type of the uniform.  'Type' can
**			be gcvNULL, in which case no type will be returned.
**
**		gcSHADER_PRECISION * Precision
**			Pointer to a variable receiving the precision of the uniform.  'Precision' can
**			be gcvNULL, in which case no type will be returned.
**
**		gctSIZE_T * ArrayLength
**			Pointer to a variable receiving the length of the array if the
**			uniform was declared as an array.  If the uniform was not declared
**			as an array, the array length will be 1.  'ArrayLength' can be gcvNULL,
**			in which case no array length will be returned.
*/
gceSTATUS
gcUNIFORM_GetTypeEx(
	IN gcUNIFORM Uniform,
	OUT gcSHADER_TYPE * Type,
    OUT gcSHADER_PRECISION * Precision,
	OUT gctSIZE_T * ArrayLength
	);

/*******************************************************************************
**                              gcUNIFORM_GetFlags
********************************************************************************
**
**	Get the flags of a gcUNIFORM object.
**
**	INPUT:
**
**		gcUNIFORM Uniform
**			Pointer to a gcUNIFORM object.
**
**	OUTPUT:
**
**		gceUNIFORM_FLAGS * Flags
**			Pointer to a variable receiving the flags of the uniform.
**
*/
gceSTATUS
gcUNIFORM_GetFlags(
	IN gcUNIFORM Uniform,
	OUT gceUNIFORM_FLAGS * Flags
	);

/*******************************************************************************
**                              gcUNIFORM_SetFlags
********************************************************************************
**
**	Set the flags of a gcUNIFORM object.
**
**	INPUT:
**
**		gcUNIFORM Uniform
**			Pointer to a gcUNIFORM object.
**
**		gceUNIFORM_FLAGS Flags
**			Flags of the uniform to be set.
**
**	OUTPUT:
**			Nothing.
**
*/
gceSTATUS
gcUNIFORM_SetFlags(
	IN gcUNIFORM Uniform,
	IN gceUNIFORM_FLAGS Flags
	);

/*******************************************************************************
**                              gcUNIFORM_GetName
********************************************************************************
**
**	Get the name of a gcUNIFORM object.
**
**	INPUT:
**
**		gcUNIFORM Uniform
**			Pointer to a gcUNIFORM object.
**
**	OUTPUT:
**
**		gctSIZE_T * Length
**			Pointer to a variable receiving the length of the uniform name.
**			'Length' can be gcvNULL, in which case no length will be returned.
**
**		gctCONST_STRING * Name
**			Pointer to a variable receiving the pointer to the uniform name.
**			'Name' can be gcvNULL, in which case no name will be returned.
*/
gceSTATUS
gcUNIFORM_GetName(
	IN gcUNIFORM Uniform,
	OUT gctSIZE_T * Length,
	OUT gctCONST_STRING * Name
	);

/*******************************************************************************
**                              gcUNIFORM_GetSampler
********************************************************************************
**
**	Get the physical sampler number for a sampler gcUNIFORM object.
**
**	INPUT:
**
**		gcUNIFORM Uniform
**			Pointer to a gcUNIFORM object.
**
**	OUTPUT:
**
**		gctUINT32 * Sampler
**			Pointer to a variable receiving the physical sampler.
*/
gceSTATUS
gcUNIFORM_GetSampler(
	IN gcUNIFORM Uniform,
	OUT gctUINT32 * Sampler
	);

gceSTATUS
gcUNIFORM_GetIndex(
    IN gcUNIFORM Uniform,
    OUT gctUINT16 * Index
    );

/*******************************************************************************
**  gcUNIFORM_GetFormat
**
**  Get the type and array length of a gcUNIFORM object.
**
**  INPUT:
**
**      gcUNIFORM Uniform
**          Pointer to a gcUNIFORM object.
**
**  OUTPUT:
**
**      gcSL_FORMAT * Format
**          Pointer to a variable receiving the format of element of the uniform.
**          'Type' can be gcvNULL, in which case no type will be returned.
**
**      gctBOOL * IsPointer
**          Pointer to a variable receiving the state wheter the uniform is a pointer.
**          'IsPointer' can be gcvNULL, in which case no array length will be returned.
*/
gceSTATUS
gcUNIFORM_GetFormat(
    IN gcUNIFORM Uniform,
    OUT gcSL_FORMAT * Format,
    OUT gctBOOL * IsPointer
    );

/*******************************************************************************
**  gcUNIFORM_SetFormat
**
**  Set the format and isPointer of a uniform.
**
**  INPUT:
**
**      gcUNIFORM Uniform
**          Pointer to a gcUNIFORM object.
**
**      gcSL_FORMAT Format
**          Format of element of the uniform shaderType.
**
**      gctBOOL IsPointer
**          Wheter the uniform is a pointer.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gcUNIFORM_SetFormat(
    IN gcUNIFORM Uniform,
    IN gcSL_FORMAT Format,
    IN gctBOOL IsPointer
    );

/*******************************************************************************
**							   gcUNIFORM_SetValue
********************************************************************************
**
**	Set the value of a uniform in integer.
**
**	INPUT:
**
**		gcUNIFORM Uniform
**			Pointer to a gcUNIFORM object.
**
**		gctSIZE_T Count
**			Number of entries to program if the uniform has been declared as an
**			array.
**
**		const gctINT * Value
**			Pointer to a buffer holding the integer values for the uniform.
**
**	OUTPUT:
**
**		Nothing.
*/
gceSTATUS
gcUNIFORM_SetValue(
	IN gcUNIFORM Uniform,
	IN gctSIZE_T Count,
	IN const gctINT * Value
	);

/*******************************************************************************
**							   gcUNIFORM_SetValueX
********************************************************************************
**
**	Set the value of a uniform in fixed point.
**
**	INPUT:
**
**		gcUNIFORM Uniform
**			Pointer to a gcUNIFORM object.
**
**		gctSIZE_T Count
**			Number of entries to program if the uniform has been declared as an
**			array.
**
**		const gctFIXED_POINT * Value
**			Pointer to a buffer holding the fixed point values for the uniform.
**
**	OUTPUT:
**
**		Nothing.
*/
gceSTATUS
gcUNIFORM_SetValueX(
	IN gcUNIFORM Uniform,
	IN gctSIZE_T Count,
	IN gctFIXED_POINT * Value
	);

/*******************************************************************************
**							   gcUNIFORM_SetValueF
********************************************************************************
**
**	Set the value of a uniform in floating point.
**
**	INPUT:
**
**		gcUNIFORM Uniform
**			Pointer to a gcUNIFORM object.
**
**		gctSIZE_T Count
**			Number of entries to program if the uniform has been declared as an
**			array.
**
**		const gctFLOAT * Value
**			Pointer to a buffer holding the floating point values for the
**			uniform.
**
**	OUTPUT:
**
**		Nothing.
*/
gceSTATUS
gcUNIFORM_SetValueF(
	IN gcUNIFORM Uniform,
	IN gctSIZE_T Count,
	IN const gctFLOAT * Value
	);

/*******************************************************************************
**  gcUNIFORM_ProgramF
**
**  Set the value of a uniform in floating point.
**
**  INPUT:
**
**      gctUINT32 Address
**          Address of Uniform.
**
**      gctSIZE_T Row/Col
**
**      const gctFLOAT * Value
**          Pointer to a buffer holding the floating point values for the
**          uniform.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gcUNIFORM_ProgramF(
    IN gctUINT32 Address,
    IN gctSIZE_T Row,
    IN gctSIZE_T Col,
    IN const gctFLOAT * Value
    );

/*******************************************************************************
**						 gcUNIFORM_GetModelViewProjMatrix
********************************************************************************
**
**	Get the value of uniform modelViewProjMatrix ID if present.
**
**	INPUT:
**
**		gcUNIFORM Uniform
**			Pointer to a gcUNIFORM object.
**
**	OUTPUT:
**
**		Nothing.
*/
gctUINT
gcUNIFORM_GetModelViewProjMatrix(
    IN gcUNIFORM Uniform
    );

/*******************************************************************************
**								gcOUTPUT_GetType
********************************************************************************
**
**	Get the type and array length of a gcOUTPUT object.
**
**	INPUT:
**
**		gcOUTPUT Output
**			Pointer to a gcOUTPUT object.
**
**	OUTPUT:
**
**		gcSHADER_TYPE * Type
**			Pointer to a variable receiving the type of the output.  'Type' can
**			be gcvNULL, in which case no type will be returned.
**
**		gctSIZE_T * ArrayLength
**			Pointer to a variable receiving the length of the array if the
**			output was declared as an array.  If the output was not declared
**			as an array, the array length will be 1.  'ArrayLength' can be gcvNULL,
**			in which case no array length will be returned.
*/
gceSTATUS
gcOUTPUT_GetType(
	IN gcOUTPUT Output,
	OUT gcSHADER_TYPE * Type,
	OUT gctSIZE_T * ArrayLength
	);

/*******************************************************************************
**							   gcOUTPUT_GetIndex
********************************************************************************
**
**	Get the index of a gcOUTPUT object.
**
**	INPUT:
**
**		gcOUTPUT Output
**			Pointer to a gcOUTPUT object.
**
**	OUTPUT:
**
**		gctUINT * Index
**			Pointer to a variable receiving the temporary register index of the
**			output.  'Index' can be gcvNULL,. in which case no index will be
**			returned.
*/
gceSTATUS
gcOUTPUT_GetIndex(
	IN gcOUTPUT Output,
	OUT gctUINT * Index
	);

/*******************************************************************************
**								gcOUTPUT_GetName
********************************************************************************
**
**	Get the name of a gcOUTPUT object.
**
**	INPUT:
**
**		gcOUTPUT Output
**			Pointer to a gcOUTPUT object.
**
**	OUTPUT:
**
**		gctSIZE_T * Length
**			Pointer to a variable receiving the length of the output name.
**			'Length' can be gcvNULL, in which case no length will be returned.
**
**		gctCONST_STRING * Name
**			Pointer to a variable receiving the pointer to the output name.
**			'Name' can be gcvNULL, in which case no name will be returned.
*/
gceSTATUS
gcOUTPUT_GetName(
	IN gcOUTPUT Output,
	OUT gctSIZE_T * Length,
	OUT gctCONST_STRING * Name
	);

/*******************************************************************************
*********************************************************** F U N C T I O N S **
*******************************************************************************/

/*******************************************************************************
**  gcFUNCTION_ReallocateArguments
**
**  Reallocate an array of gcsFUNCTION_ARGUMENT objects.
**
**  INPUT:
**
**      gcFUNCTION Function
**          Pointer to a gcFUNCTION object.
**
**      gctSIZE_T Count
**          Array count to reallocate.  'Count' must be at least 1.
*/
gceSTATUS
gcFUNCTION_ReallocateArguments(
    IN gcFUNCTION Function,
    IN gctSIZE_T Count
    );

gceSTATUS
gcFUNCTION_AddArgument(
	IN gcFUNCTION Function,
	IN gctUINT16 TempIndex,
	IN gctUINT8 Enable,
	IN gctUINT8 Qualifier
	);

gceSTATUS
gcFUNCTION_GetArgument(
	IN gcFUNCTION Function,
	IN gctUINT16 Index,
	OUT gctUINT16_PTR Temp,
	OUT gctUINT8_PTR Enable,
	OUT gctUINT8_PTR Swizzle
	);

gceSTATUS
gcFUNCTION_GetLabel(
	IN gcFUNCTION Function,
	OUT gctUINT_PTR Label
	);

/*******************************************************************************
************************* K E R N E L    P R O P E R T Y    F U N C T I O N S **
*******************************************************************************/
/*******************************************************************************/
gceSTATUS
gcKERNEL_FUNCTION_AddKernelFunctionProperties(
	    IN gcKERNEL_FUNCTION KernelFunction,
		IN gctINT propertyType,
		IN gctSIZE_T propertySize,
		IN gctINT * values
		);

gceSTATUS
gcKERNEL_FUNCTION_GetPropertyCount(
    IN gcKERNEL_FUNCTION KernelFunction,
    OUT gctSIZE_T * Count
    );

gceSTATUS
gcKERNEL_FUNCTION_GetProperty(
    IN gcKERNEL_FUNCTION KernelFunction,
    IN gctUINT Index,
	OUT gctSIZE_T * propertySize,
	OUT gctINT * propertyType,
	OUT gctINT * propertyValues
    );


/*******************************************************************************
*******************************I M A G E   S A M P L E R    F U N C T I O N S **
*******************************************************************************/
/*******************************************************************************
**  gcKERNEL_FUNCTION_ReallocateImageSamplers
**
**  Reallocate an array of pointers to image sampler pair.
**
**  INPUT:
**
**      gcKERNEL_FUNCTION KernelFunction
**          Pointer to a gcKERNEL_FUNCTION object.
**
**      gctSIZE_T Count
**          Array count to reallocate.  'Count' must be at least 1.
*/
gceSTATUS
gcKERNEL_FUNCTION_ReallocateImageSamplers(
    IN gcKERNEL_FUNCTION KernelFunction,
    IN gctSIZE_T Count
    );

gceSTATUS
gcKERNEL_FUNCTION_AddImageSampler(
    IN gcKERNEL_FUNCTION KernelFunction,
    IN gctUINT8 ImageNum,
    IN gctBOOL IsConstantSamplerType,
    IN gctUINT32 SamplerType
    );

gceSTATUS
gcKERNEL_FUNCTION_GetImageSamplerCount(
    IN gcKERNEL_FUNCTION KernelFunction,
    OUT gctSIZE_T * Count
    );

gceSTATUS
gcKERNEL_FUNCTION_GetImageSampler(
    IN gcKERNEL_FUNCTION KernelFunction,
    IN gctUINT Index,
    OUT gctUINT8 *ImageNum,
    OUT gctBOOL *IsConstantSamplerType,
    OUT gctUINT32 *SamplerType
    );

/*******************************************************************************
*********************************************K E R N E L    F U N C T I O N S **
*******************************************************************************/

/*******************************************************************************
**  gcKERNEL_FUNCTION_ReallocateArguments
**
**  Reallocate an array of gcsFUNCTION_ARGUMENT objects.
**
**  INPUT:
**
**      gcKERNEL_FUNCTION Function
**          Pointer to a gcKERNEL_FUNCTION object.
**
**      gctSIZE_T Count
**          Array count to reallocate.  'Count' must be at least 1.
*/
gceSTATUS
gcKERNEL_FUNCTION_ReallocateArguments(
    IN gcKERNEL_FUNCTION Function,
    IN gctSIZE_T Count
    );

gceSTATUS
gcKERNEL_FUNCTION_AddArgument(
	IN gcKERNEL_FUNCTION Function,
	IN gctUINT16 TempIndex,
	IN gctUINT8 Enable,
	IN gctUINT8 Qualifier
	);

gceSTATUS
gcKERNEL_FUNCTION_GetArgument(
	IN gcKERNEL_FUNCTION Function,
	IN gctUINT16 Index,
	OUT gctUINT16_PTR Temp,
	OUT gctUINT8_PTR Enable,
	OUT gctUINT8_PTR Swizzle
	);

gceSTATUS
gcKERNEL_FUNCTION_GetLabel(
	IN gcKERNEL_FUNCTION Function,
	OUT gctUINT_PTR Label
	);

gceSTATUS
gcKERNEL_FUNCTION_GetName(
    IN gcKERNEL_FUNCTION KernelFunction,
    OUT gctSIZE_T * Length,
    OUT gctCONST_STRING * Name
    );

gceSTATUS
gcKERNEL_FUNCTION_ReallocateUniformArguments(
    IN gcKERNEL_FUNCTION KernelFunction,
    IN gctSIZE_T Count
    );

gceSTATUS
gcKERNEL_FUNCTION_AddUniformArgument(
    IN gcKERNEL_FUNCTION KernelFunction,
    IN gctCONST_STRING Name,
    IN gcSHADER_TYPE Type,
    IN gctSIZE_T Length,
    OUT gcUNIFORM * UniformArgument
    );

gceSTATUS
gcKERNEL_FUNCTION_GetUniformArgumentCount(
    IN gcKERNEL_FUNCTION KernelFunction,
    OUT gctSIZE_T * Count
    );

gceSTATUS
gcKERNEL_FUNCTION_GetUniformArgument(
    IN gcKERNEL_FUNCTION KernelFunction,
    IN gctUINT Index,
    OUT gcUNIFORM * UniformArgument
    );

gceSTATUS
gcKERNEL_FUNCTION_SetCodeEnd(
    IN gcKERNEL_FUNCTION KernelFunction
    );

/*******************************************************************************
**                              gcCompileShader
********************************************************************************
**
**	Compile a shader.
**
**	INPUT:
**
**		gcoOS Hal
**			Pointer to an gcoHAL object.
**
**		gctINT ShaderType
**			Shader type to compile.  Can be one of the following values:
**
**				gcSHADER_TYPE_VERTEX
**					Compile a vertex shader.
**
**				gcSHADER_TYPE_FRAGMENT
**					Compile a fragment shader.
**
**		gctSIZE_T SourceSize
**			Size of the source buffer in bytes.
**
**		gctCONST_STRING Source
**			Pointer to the buffer containing the shader source code.
**
**	OUTPUT:
**
**		gcSHADER * Binary
**			Pointer to a variable receiving the pointer to a gcSHADER object
**			containg the compiled shader code.
**
**		gctSTRING * Log
**			Pointer to a variable receiving a string pointer containging the
**			compile log.
*/
gceSTATUS
gcCompileShader(
	IN gcoHAL Hal,
	IN gctINT ShaderType,
	IN gctSIZE_T SourceSize,
	IN gctCONST_STRING Source,
	OUT gcSHADER * Binary,
	OUT gctSTRING * Log
	);

/*******************************************************************************
**                              gcOptimizeShader
********************************************************************************
**
**	Optimize a shader.
**
**	INPUT:
**
**		gcSHADER Shader
**			Pointer to a gcSHADER object holding information about the compiled
**			shader.
**
**		gctFILE LogFile
**			Pointer to an open FILE object.
*/
gceSTATUS
gcOptimizeShader(
	IN gcSHADER Shader,
	IN gctFILE LogFile
	);

/*******************************************************************************
**                                gcLinkShaders
********************************************************************************
**
**	Link two shaders and generate a harwdare specific state buffer by compiling
**	the compiler generated code through the resource allocator and code
**	generator.
**
**	INPUT:
**
**		gcSHADER VertexShader
**			Pointer to a gcSHADER object holding information about the compiled
**			vertex shader.
**
**		gcSHADER FragmentShader
**			Pointer to a gcSHADER object holding information about the compiled
**			fragment shader.
**
**		gceSHADER_FLAGS Flags
**			Compiler flags.  Can be any of the following:
**
**				gcvSHADER_DEAD_CODE       - Dead code elimination.
**				gcvSHADER_RESOURCE_USAGE  - Resource usage optimizaion.
**				gcvSHADER_OPTIMIZER       - Full optimization.
**				gcvSHADER_USE_GL_Z        - Use OpenGL ES Z coordinate.
**				gcvSHADER_USE_GL_POSITION - Use OpenGL ES gl_Position.
**				gcvSHADER_USE_GL_FACE     - Use OpenGL ES gl_FaceForward.
**
**	OUTPUT:
**
**		gctSIZE_T * StateBufferSize
**			Pointer to a variable receicing the number of bytes in the buffer
**			returned in 'StateBuffer'.
**
**		gctPOINTER * StateBuffer
**			Pointer to a variable receiving a buffer pointer that contains the
**			states required to download the shaders into the hardware.
**
**		gcsHINT_PTR * Hints
**			Pointer to a variable receiving a gcsHINT structure pointer that
**			contains information required when loading the shader states.
*/
gceSTATUS
gcLinkShaders(
	IN gcSHADER VertexShader,
	IN gcSHADER FragmentShader,
	IN gceSHADER_FLAGS Flags,
	OUT gctSIZE_T * StateBufferSize,
	OUT gctPOINTER * StateBuffer,
	OUT gcsHINT_PTR * Hints,
    OUT gcMACHINECODE_PTR *ppVsMachineCode,
    OUT gcMACHINECODE_PTR *ppFsMachineCode
	);

/*******************************************************************************
**                                gcLoadShaders
********************************************************************************
**
**	Load a pre-compiled and pre-linked shader program into the hardware.
**
**	INPUT:
**
**		gcoHAL Hal
**			Pointer to a gcoHAL object.
**
**		gctSIZE_T StateBufferSize
**			The number of bytes in the 'StateBuffer'.
**
**		gctPOINTER StateBuffer
**			Pointer to the states that make up the shader program.
**
**		gcsHINT_PTR Hints
**			Pointer to a gcsHINT structure that contains information required
**			when loading the shader states.
*/
gceSTATUS
gcLoadShaders(
	IN gcoHAL Hal,
	IN gctSIZE_T StateBufferSize,
	IN gctPOINTER StateBuffer,
	IN gcsHINT_PTR Hints
	);

gceSTATUS
gcRecompileShaders(
    IN gcoHAL Hal,
    IN gcMACHINECODE_PTR pVsMachineCode,
    IN gcMACHINECODE_PTR pPsMachineCode,
    /*Recompile variables*/
    IN OUT gctPOINTER *ppRecompileStateBuffer,
    IN OUT gctSIZE_T *pRecompileStateBufferSize,
    IN OUT gcsHINT_PTR *ppRecompileHints,
    /* natvie state*/
    IN gctPOINTER pNativeStateBuffer,
    IN gctSIZE_T nativeStateBufferSize,
    IN gcsHINT_PTR pNativeHints,
    /* npt info */
    IN gctUINT32 Samplers,
    IN gctUINT32 *SamplerWrapS,
    IN gctUINT32 *SamplerWrapT
    );

gceSTATUS
gcRecompileDepthBias(
    IN gcoHAL Hal,
    IN gcMACHINECODE_PTR pVsMachineCode,
    /*Recompile variables*/
    IN OUT gctPOINTER *ppRecompileStateBuffer,
    IN OUT gctSIZE_T *pRecompileStateBufferSize,
    IN OUT gcsHINT_PTR *ppRecompileHints,
    /* natvie state*/
    IN gctPOINTER pNativeStateBuffer,
    IN gctSIZE_T nativeStateBufferSize,
    IN gcsHINT_PTR pNativeHints,
	OUT gctINT * uniformAddr,
	OUT gctINT * uniformChannel
    );

/*******************************************************************************
**                                gcSaveProgram
********************************************************************************
**
**	Save pre-compiled shaders and pre-linked programs to a binary file.
**
**	INPUT:
**
**		gcSHADER VertexShader
**			Pointer to vertex shader object.
**
**		gcSHADER FragmentShader
**			Pointer to fragment shader object.
**
**		gctSIZE_T ProgramBufferSize
**			Number of bytes in 'ProgramBuffer'.
**
**		gctPOINTER ProgramBuffer
**			Pointer to buffer containing the program states.
**
**		gcsHINT_PTR Hints
**			Pointer to HINTS structure for program states.
**
**	OUTPUT:
**
**		gctPOINTER * Binary
**			Pointer to a variable receiving the binary data to be saved.
**
**		gctSIZE_T * BinarySize
**			Pointer to a variable receiving the number of bytes inside 'Binary'.
*/
gceSTATUS
gcSaveProgram(
	IN gcSHADER VertexShader,
	IN gcSHADER FragmentShader,
	IN gctSIZE_T ProgramBufferSize,
	IN gctPOINTER ProgramBuffer,
	IN gcsHINT_PTR Hints,
	OUT gctPOINTER * Binary,
	OUT gctSIZE_T * BinarySize
	);

/*******************************************************************************
**                                gcLoadProgram
********************************************************************************
**
**	Load pre-compiled shaders and pre-linked programs from a binary file.
**
**	INPUT:
**
**		gctPOINTER Binary
**			Pointer to the binary data loaded.
**
**		gctSIZE_T BinarySize
**			Number of bytes in 'Binary'.
**
**	OUTPUT:
**
**		gcSHADER VertexShader
**			Pointer to a vertex shader object.
**
**		gcSHADER FragmentShader
**			Pointer to a fragment shader object.
**
**		gctSIZE_T * ProgramBufferSize
**			Pointer to a variable receicing the number of bytes in the buffer
**			returned in 'ProgramBuffer'.
**
**		gctPOINTER * ProgramBuffer
**			Pointer to a variable receiving a buffer pointer that contains the
**			states required to download the shaders into the hardware.
**
**		gcsHINT_PTR * Hints
**			Pointer to a variable receiving a gcsHINT structure pointer that
**			contains information required when loading the shader states.
*/
gceSTATUS
gcLoadProgram(
	IN gctPOINTER Binary,
	IN gctSIZE_T BinarySize,
	OUT gcSHADER VertexShader,
	OUT gcSHADER FragmentShader,
	OUT gctSIZE_T * ProgramBufferSize,
	OUT gctPOINTER * ProgramBuffer,
	OUT gcsHINT_PTR * Hints
	);

/*******************************************************************************
**                              gcCompileKernel
********************************************************************************
**
**	Compile a OpenCL kernel shader.
**
**	INPUT:
**
**		gcoOS Hal
**			Pointer to an gcoHAL object.
**
**		gctSIZE_T SourceSize
**			Size of the source buffer in bytes.
**
**		gctCONST_STRING Source
**			Pointer to the buffer containing the shader source code.
**
**	OUTPUT:
**
**		gcSHADER * Binary
**			Pointer to a variable receiving the pointer to a gcSHADER object
**			containg the compiled shader code.
**
**		gctSTRING * Log
**			Pointer to a variable receiving a string pointer containging the
**			compile log.
*/
gceSTATUS
gcCompileKernel(
	IN gcoHAL Hal,
	IN gctSIZE_T SourceSize,
	IN gctCONST_STRING Source,
	IN gctCONST_STRING Options,
	OUT gcSHADER * Binary,
	OUT gctSTRING * Log
	);

/*******************************************************************************
**                                gcLinkKernel
********************************************************************************
**
**	Link OpenCL kernel and generate a harwdare specific state buffer by compiling
**	the compiler generated code through the resource allocator and code
**	generator.
**
**	INPUT:
**
**		gcSHADER Kernel
**			Pointer to a gcSHADER object holding information about the compiled
**			OpenCL kernel.
**
**		gceSHADER_FLAGS Flags
**			Compiler flags.  Can be any of the following:
**
**				gcvSHADER_DEAD_CODE       - Dead code elimination.
**				gcvSHADER_RESOURCE_USAGE  - Resource usage optimizaion.
**				gcvSHADER_OPTIMIZER       - Full optimization.
**				gcvSHADER_USE_GL_Z        - Use OpenGL ES Z coordinate.
**				gcvSHADER_USE_GL_POSITION - Use OpenGL ES gl_Position.
**				gcvSHADER_USE_GL_FACE     - Use OpenGL ES gl_FaceForward.
**
**	OUTPUT:
**
**		gctSIZE_T * StateBufferSize
**			Pointer to a variable receiving the number of bytes in the buffer
**			returned in 'StateBuffer'.
**
**		gctPOINTER * StateBuffer
**			Pointer to a variable receiving a buffer pointer that contains the
**			states required to download the shaders into the hardware.
**
**		gcsHINT_PTR * Hints
**			Pointer to a variable receiving a gcsHINT structure pointer that
**			contains information required when loading the shader states.
*/
gceSTATUS
gcLinkKernel(
	IN gcSHADER Kernel,
	IN gceSHADER_FLAGS Flags,
	OUT gctSIZE_T * StateBufferSize,
	OUT gctPOINTER * StateBuffer,
	OUT gcsHINT_PTR * Hints
	);

/*******************************************************************************
**                                gcLoadKernel
********************************************************************************
**
**  Load a pre-compiled and pre-linked kernel program into the hardware.
**
**  INPUT:
**
**      gctSIZE_T StateBufferSize
**          The number of bytes in the 'StateBuffer'.
**
**      gctPOINTER StateBuffer
**          Pointer to the states that make up the shader program.
**
**      gcsHINT_PTR Hints
**          Pointer to a gcsHINT structure that contains information required
**          when loading the shader states.
*/
gceSTATUS
gcLoadKernel(
    IN gctSIZE_T StateBufferSize,
    IN gctPOINTER StateBuffer,
    IN gcsHINT_PTR Hints
    );

gceSTATUS
gcInvokeThreadWalker(
    IN gcsTHREAD_WALKER_INFO_PTR Info
    );

void
gcTYPE_GetTypeInfo(
    IN gcSHADER_TYPE      Type,
    OUT gctINT *          Components,
    OUT gctINT *          Rows,
    OUT gctCONST_STRING * Name
    );

gctBOOL
gcOPT_doVaryingPackingForShader(
	IN gcSHADER Shader
    );

gceSTATUS
gcSHADER_PatchNPOTForMachineCode(
    IN     gcSHADER_KIND          shaderType,
    IN     gcMACHINECODE_PTR      pMachineCode,
    IN     gcNPOT_PATCH_PARAM_PTR pPatchParam,
    IN     gctUINT                countOfPatchParam,
    IN     gctUINT                hwSupportedInstCount,
    OUT    gctPOINTER*            ppCmdBuffer,
    OUT    gctUINT32*             pByteSizeOfCmdBuffer,
    IN OUT gcsHINT_PTR            pHints /* User needs copy original hints to this one, then passed this one in */
    );

gceSTATUS
gcSHADER_PatchZBiasForMachineCodeVS(
    IN     gcMACHINECODE_PTR       pMachineCode,
    IN OUT gcZBIAS_PATCH_PARAM_PTR pPatchParam,
    IN     gctUINT                 hwSupportedInstCount,
    OUT    gctPOINTER*             ppCmdBuffer,
    OUT    gctUINT32*              pByteSizeOfCmdBuffer,
    IN OUT gcsHINT_PTR             pHints /* User needs copy original hints to this one, then passed this one in */
    );

gceSTATUS
gcSHADER_InsertList(
    IN gcSHADER                    Shader,
    IN gcSHADER_LIST *             Root,
    IN gctINT                      Index,
    IN gctINT                      Data0,
    IN gctINT                      Data1
    );

gceSTATUS
gcSHADER_UpdateList(
    IN gcSHADER                    Shader,
    IN gcSHADER_LIST               Root,
    IN gctINT                      Index,
    IN gctINT                      NewIndex
    );

gceSTATUS
gcSHADER_DeleteList(
    IN gcSHADER                    Shader,
    IN gcSHADER_LIST *             Root,
    IN gctINT                      Index
    );

gceSTATUS
gcSHADER_FindList(
    IN gcSHADER                    Shader,
    IN gcSHADER_LIST               Root,
    IN gctINT                      Index,
    IN gcSHADER_LIST *             List
    );

gceSTATUS
gcSHADER_InsertWClipList(
    IN gcSHADER                    Shader,
    IN gctINT                      Index,
    IN gctINT                      Data0,
    IN gctINT                      Data1
    );

#ifdef __cplusplus
}
#endif

#endif /* VIVANTE_NO_3D */
#endif /* __gc_hal_compiler_h_ */
