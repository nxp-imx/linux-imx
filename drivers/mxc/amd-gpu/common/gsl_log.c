/* Copyright (c) 2002,2008-2009, Code Aurora Forum. All rights reserved.
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

#ifdef GSL_LOG

#define _CRT_SECURE_NO_WARNINGS

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "gsl.h"

#define KGSL_OUTPUT_TYPE_MEMBUF 0
#define KGSL_OUTPUT_TYPE_STDOUT 1
#define KGSL_OUTPUT_TYPE_FILE   2

#define REG_OUTPUT( X ) case X: b += sprintf( b, "%s", #X ); break;
#define INTRID_OUTPUT( X ) case X: b += sprintf( b, "%s", #X ); break;

typedef struct log_output
{
    unsigned char       type;
    unsigned int        flags;
    oshandle_t         file;

    struct log_output*  next;
} log_output_t;

static log_output_t* outputs = NULL;

static oshandle_t   log_mutex = NULL;
static char         buffer[256];
static char         buffer2[256];
static int          log_initialized = 0;

//----------------------------------------------------------------------------

int kgsl_log_init()
{
    log_mutex = kos_mutex_create( "log_mutex" );

    log_initialized = 1;
    
    return GSL_SUCCESS;
}

//----------------------------------------------------------------------------

int kgsl_log_close()
{
    if( !log_initialized ) return GSL_SUCCESS;

    // Go throught output list and free every node
    while( outputs != NULL )
    {
        log_output_t* temp = outputs->next;
        
        switch( outputs->type )
        {
            case KGSL_OUTPUT_TYPE_FILE:
                kos_fclose( outputs->file );
            break;
        }

        kos_free( outputs );
        outputs = temp;
    }

    kos_mutex_free( log_mutex );

    log_initialized = 0;

    return GSL_SUCCESS;
}

//----------------------------------------------------------------------------

int kgsl_log_open_stdout( unsigned int log_flags )
{
    log_output_t* output;
    
    if( !log_initialized ) return GSL_SUCCESS;
    
    output = kos_malloc( sizeof( log_output_t ) );
    output->type = KGSL_OUTPUT_TYPE_STDOUT;
    output->flags = log_flags;

    // Add to the list
    if( outputs == NULL )    
    {
        // First node in the list.
        outputs = output;
        output->next = NULL;
    }
    else
    {
        // Add to the start of the list
        output->next = outputs;
        outputs = output;
    }

    return GSL_SUCCESS;
}

//----------------------------------------------------------------------------

int kgsl_log_open_membuf( int* memBufId, unsigned int log_flags )
{
    // TODO

    return GSL_SUCCESS;
}

//----------------------------------------------------------------------------

int kgsl_log_open_file( char* filename, unsigned int log_flags )
{
    log_output_t* output;
    
    if( !log_initialized ) return GSL_SUCCESS;
    
    output = kos_malloc( sizeof( log_output_t ) );
    output->type = KGSL_OUTPUT_TYPE_FILE;
    output->flags = log_flags;
    output->file = kos_fopen( filename, "w" );

    // Add to the list
    if( outputs == NULL )    
    {
        // First node in the list.
        outputs = output;
        output->next = NULL;
    }
    else
    {
        // Add to the start of the list
        output->next = outputs;
        outputs = output;
    }

    return GSL_SUCCESS;
}

//----------------------------------------------------------------------------

int kgsl_log_flush_membuf( char* filename, int memBufId )
{
    // TODO

    return GSL_SUCCESS;
}
//----------------------------------------------------------------------------

int kgsl_log_write( unsigned int log_flags, char* format, ... )
{
    char            *c = format;
    char            *b = buffer;
    char            *p1, *p2;
    log_output_t*   output;
    va_list         arguments;

    if( !log_initialized ) return GSL_SUCCESS;

    // Acquire mutex lock as we are using shared buffer for the string parsing
    kos_mutex_lock( log_mutex );

    // Add separator
    *(b++) = '|'; *(b++) = ' ';

    va_start( arguments, format );

    while( 1 )
    {
        // Find the first occurence of %
        p1 = strchr( c, '%' );
        if( !p1 ) 
        {
            // No more % characters -> copy rest of the string
            strcpy( b, c );

            break;
        }

        // Find the second occurence of % and handle the string until that point
        p2 = strchr( p1+1, '%' );

        // If not found, just use the end of the buffer
        if( !p2 ) p2 = strchr( p1+1, '\0' );

        // Break the string to this point
        kos_memcpy( buffer2, c, p2-c );
        *(buffer2+(unsigned int)(p2-c)) = '\0';

        switch( *(p1+1) )
        {
            // gsl_memdesc_t
            case 'M':
            {
                gsl_memdesc_t val = va_arg( arguments, gsl_memdesc_t );
                // Handle string before %M
                kos_memcpy( b, c, p1-c );
                b += (unsigned int)p1-(unsigned int)c;
                // Replace %M
                b += sprintf( b, "[hostptr=0x%08x, gpuaddr=0x%08x]", val.hostptr, val.gpuaddr );
                // Handle string after %M
                kos_memcpy( b, p1+2, p2-(p1+2) );
                b += (unsigned int)p2-(unsigned int)(p1+2);
                *b = '\0';
            }
            break;

            // GSL_SUCCESS/GSL_FAILURE
            case 'B':
            {
                int val = va_arg( arguments, int );
                // Handle string before %B
                kos_memcpy( b, c, p1-c );
                b += (unsigned int)p1-(unsigned int)c;
                // Replace %B
                if( val == GSL_SUCCESS )
                    b += sprintf( b, "%s", "GSL_SUCCESS" );
                else
                    b += sprintf( b, "%s", "GSL_FAILURE" );
                // Handle string after %B
                kos_memcpy( b, p1+2, p2-(p1+2) );
                b += (unsigned int)p2-(unsigned int)(p1+2);
                *b = '\0';
            }
            break;

            // gsl_deviceid_t
            case 'D':
            {
                gsl_deviceid_t val = va_arg( arguments, gsl_deviceid_t );
                // Handle string before %D
                kos_memcpy( b, c, p1-c );
                b += (unsigned int)p1-(unsigned int)c;
                // Replace %D
                switch( val )
                {
                    case GSL_DEVICE_ANY:
                        b += sprintf( b, "%s", "GSL_DEVICE_ANY" );
                    break;
                    case GSL_DEVICE_YAMATO:
                        b += sprintf( b, "%s", "GSL_DEVICE_YAMATO" );
                    break;
                    case GSL_DEVICE_G12:
                        b += sprintf( b, "%s", "GSL_DEVICE_G12" );
                    break;
                    default:
                        b += sprintf( b, "%s", "UNKNOWN DEVICE" );
                    break;
                }
                // Handle string after %D
                kos_memcpy( b, p1+2, p2-(p1+2) );
                b += (unsigned int)p2-(unsigned int)(p1+2);
                *b = '\0';
            }
            break;
            
            // gsl_intrid_t
            case 'I':
            {
                unsigned int val = va_arg( arguments, unsigned int );
                // Handle string before %I
                kos_memcpy( b, c, p1-c );
                b += (unsigned int)p1-(unsigned int)c;
                // Replace %I
                switch( val )
                {
                    INTRID_OUTPUT( GSL_INTR_YDX_MH_AXI_READ_ERROR );
                    INTRID_OUTPUT( GSL_INTR_YDX_MH_AXI_WRITE_ERROR );
                    INTRID_OUTPUT( GSL_INTR_YDX_MH_MMU_PAGE_FAULT );
                    INTRID_OUTPUT( GSL_INTR_YDX_CP_SW_INT );
                    INTRID_OUTPUT( GSL_INTR_YDX_CP_T0_PACKET_IN_IB );
                    INTRID_OUTPUT( GSL_INTR_YDX_CP_OPCODE_ERROR );
                    INTRID_OUTPUT( GSL_INTR_YDX_CP_PROTECTED_MODE_ERROR );
                    INTRID_OUTPUT( GSL_INTR_YDX_CP_RESERVED_BIT_ERROR );
                    INTRID_OUTPUT( GSL_INTR_YDX_CP_IB_ERROR );
                    INTRID_OUTPUT( GSL_INTR_YDX_CP_IB2_INT );
                    INTRID_OUTPUT( GSL_INTR_YDX_CP_IB1_INT );
                    INTRID_OUTPUT( GSL_INTR_YDX_CP_RING_BUFFER );
                    INTRID_OUTPUT( GSL_INTR_YDX_RBBM_READ_ERROR );
                    INTRID_OUTPUT( GSL_INTR_YDX_RBBM_DISPLAY_UPDATE );
                    INTRID_OUTPUT( GSL_INTR_YDX_RBBM_GUI_IDLE );
                    INTRID_OUTPUT( GSL_INTR_YDX_SQ_PS_WATCHDOG );
                    INTRID_OUTPUT( GSL_INTR_YDX_SQ_VS_WATCHDOG );
                    INTRID_OUTPUT( GSL_INTR_G12_MH );
                    INTRID_OUTPUT( GSL_INTR_G12_G2D );
                    INTRID_OUTPUT( GSL_INTR_G12_FIFO );
#ifndef _Z180
                    INTRID_OUTPUT( GSL_INTR_G12_FBC );
#endif // _Z180
                    INTRID_OUTPUT( GSL_INTR_G12_MH_AXI_READ_ERROR );
                    INTRID_OUTPUT( GSL_INTR_G12_MH_AXI_WRITE_ERROR );
                    INTRID_OUTPUT( GSL_INTR_G12_MH_MMU_PAGE_FAULT );
                    INTRID_OUTPUT( GSL_INTR_COUNT );
                    INTRID_OUTPUT( GSL_INTR_FOOBAR );

                    default:
                        b += sprintf( b, "%s", "UNKNOWN INTERRUPT ID" );
                    break;
                }
                // Handle string after %I
                kos_memcpy( b, p1+2, p2-(p1+2) );
                b += (unsigned int)p2-(unsigned int)(p1+2);
                *b = '\0';
            }
            break;

            // Register offset
            case 'R':
            {
                unsigned int val = va_arg( arguments, unsigned int );
                
                // Handle string before %R
                kos_memcpy( b, c, p1-c );
                b += (unsigned int)p1-(unsigned int)c;
                // Replace %R
                switch( val )
                {
                    REG_OUTPUT( mmPA_CL_VPORT_XSCALE ); REG_OUTPUT( mmPA_CL_VPORT_XOFFSET ); REG_OUTPUT( mmPA_CL_VPORT_YSCALE );
                    REG_OUTPUT( mmPA_CL_VPORT_YOFFSET ); REG_OUTPUT( mmPA_CL_VPORT_ZSCALE ); REG_OUTPUT( mmPA_CL_VPORT_ZOFFSET );
                    REG_OUTPUT( mmPA_CL_VTE_CNTL ); REG_OUTPUT( mmPA_CL_CLIP_CNTL ); REG_OUTPUT( mmPA_CL_GB_VERT_CLIP_ADJ );
                    REG_OUTPUT( mmPA_CL_GB_VERT_DISC_ADJ ); REG_OUTPUT( mmPA_CL_GB_HORZ_CLIP_ADJ ); REG_OUTPUT( mmPA_CL_GB_HORZ_DISC_ADJ );
                    REG_OUTPUT( mmPA_CL_ENHANCE ); REG_OUTPUT( mmPA_SC_ENHANCE ); REG_OUTPUT( mmPA_SU_VTX_CNTL );
                    REG_OUTPUT( mmPA_SU_POINT_SIZE ); REG_OUTPUT( mmPA_SU_POINT_MINMAX ); REG_OUTPUT( mmPA_SU_LINE_CNTL );
                    REG_OUTPUT( mmPA_SU_FACE_DATA ); REG_OUTPUT( mmPA_SU_SC_MODE_CNTL ); REG_OUTPUT( mmPA_SU_POLY_OFFSET_FRONT_SCALE );
                    REG_OUTPUT( mmPA_SU_POLY_OFFSET_FRONT_OFFSET ); REG_OUTPUT( mmPA_SU_POLY_OFFSET_BACK_SCALE ); REG_OUTPUT( mmPA_SU_POLY_OFFSET_BACK_OFFSET );
                    REG_OUTPUT( mmPA_SU_PERFCOUNTER0_SELECT ); REG_OUTPUT( mmPA_SU_PERFCOUNTER1_SELECT ); REG_OUTPUT( mmPA_SU_PERFCOUNTER2_SELECT );
                    REG_OUTPUT( mmPA_SU_PERFCOUNTER3_SELECT ); REG_OUTPUT( mmPA_SU_PERFCOUNTER0_LOW ); REG_OUTPUT( mmPA_SU_PERFCOUNTER0_HI );
                    REG_OUTPUT( mmPA_SU_PERFCOUNTER1_LOW ); REG_OUTPUT( mmPA_SU_PERFCOUNTER1_HI ); REG_OUTPUT( mmPA_SU_PERFCOUNTER2_LOW );
                    REG_OUTPUT( mmPA_SU_PERFCOUNTER2_HI ); REG_OUTPUT( mmPA_SU_PERFCOUNTER3_LOW ); REG_OUTPUT( mmPA_SU_PERFCOUNTER3_HI );
                    REG_OUTPUT( mmPA_SC_WINDOW_OFFSET ); REG_OUTPUT( mmPA_SC_AA_CONFIG ); REG_OUTPUT( mmPA_SC_AA_MASK );
                    REG_OUTPUT( mmPA_SC_LINE_STIPPLE ); REG_OUTPUT( mmPA_SC_LINE_CNTL ); REG_OUTPUT( mmPA_SC_WINDOW_SCISSOR_TL );
                    REG_OUTPUT( mmPA_SC_WINDOW_SCISSOR_BR ); REG_OUTPUT( mmPA_SC_SCREEN_SCISSOR_TL ); REG_OUTPUT( mmPA_SC_SCREEN_SCISSOR_BR );
                    REG_OUTPUT( mmPA_SC_VIZ_QUERY ); REG_OUTPUT( mmPA_SC_VIZ_QUERY_STATUS ); REG_OUTPUT( mmPA_SC_LINE_STIPPLE_STATE );
                    REG_OUTPUT( mmPA_SC_PERFCOUNTER0_SELECT ); REG_OUTPUT( mmPA_SC_PERFCOUNTER0_LOW ); REG_OUTPUT( mmPA_SC_PERFCOUNTER0_HI );
                    REG_OUTPUT( mmPA_CL_CNTL_STATUS ); REG_OUTPUT( mmPA_SU_CNTL_STATUS ); REG_OUTPUT( mmPA_SC_CNTL_STATUS );
                    REG_OUTPUT( mmPA_SU_DEBUG_CNTL ); REG_OUTPUT( mmPA_SU_DEBUG_DATA ); REG_OUTPUT( mmPA_SC_DEBUG_CNTL );
                    REG_OUTPUT( mmPA_SC_DEBUG_DATA ); REG_OUTPUT( mmGFX_COPY_STATE ); REG_OUTPUT( mmVGT_DRAW_INITIATOR );
                    REG_OUTPUT( mmVGT_EVENT_INITIATOR ); REG_OUTPUT( mmVGT_DMA_BASE ); REG_OUTPUT( mmVGT_DMA_SIZE );
                    REG_OUTPUT( mmVGT_BIN_BASE ); REG_OUTPUT( mmVGT_BIN_SIZE ); REG_OUTPUT( mmVGT_CURRENT_BIN_ID_MIN );
                    REG_OUTPUT( mmVGT_CURRENT_BIN_ID_MAX ); REG_OUTPUT( mmVGT_IMMED_DATA ); REG_OUTPUT( mmVGT_MAX_VTX_INDX );
                    REG_OUTPUT( mmVGT_MIN_VTX_INDX ); REG_OUTPUT( mmVGT_INDX_OFFSET ); REG_OUTPUT( mmVGT_VERTEX_REUSE_BLOCK_CNTL );
                    REG_OUTPUT( mmVGT_OUT_DEALLOC_CNTL ); REG_OUTPUT( mmVGT_MULTI_PRIM_IB_RESET_INDX ); REG_OUTPUT( mmVGT_ENHANCE );
                    REG_OUTPUT( mmVGT_VTX_VECT_EJECT_REG ); REG_OUTPUT( mmVGT_LAST_COPY_STATE ); REG_OUTPUT( mmVGT_DEBUG_CNTL );
                    REG_OUTPUT( mmVGT_DEBUG_DATA ); REG_OUTPUT( mmVGT_CNTL_STATUS ); REG_OUTPUT( mmVGT_CRC_SQ_DATA );
                    REG_OUTPUT( mmVGT_CRC_SQ_CTRL ); REG_OUTPUT( mmVGT_PERFCOUNTER0_SELECT ); REG_OUTPUT( mmVGT_PERFCOUNTER1_SELECT );
                    REG_OUTPUT( mmVGT_PERFCOUNTER2_SELECT ); REG_OUTPUT( mmVGT_PERFCOUNTER3_SELECT ); REG_OUTPUT( mmVGT_PERFCOUNTER0_LOW );
                    REG_OUTPUT( mmVGT_PERFCOUNTER1_LOW ); REG_OUTPUT( mmVGT_PERFCOUNTER2_LOW ); REG_OUTPUT( mmVGT_PERFCOUNTER3_LOW );
                    REG_OUTPUT( mmVGT_PERFCOUNTER0_HI ); REG_OUTPUT( mmVGT_PERFCOUNTER1_HI ); REG_OUTPUT( mmVGT_PERFCOUNTER2_HI );
                    REG_OUTPUT( mmVGT_PERFCOUNTER3_HI ); REG_OUTPUT( mmTC_CNTL_STATUS ); REG_OUTPUT( mmTCR_CHICKEN );
                    REG_OUTPUT( mmTCF_CHICKEN ); REG_OUTPUT( mmTCM_CHICKEN ); REG_OUTPUT( mmTCR_PERFCOUNTER0_SELECT );
                    REG_OUTPUT( mmTCR_PERFCOUNTER1_SELECT ); REG_OUTPUT( mmTCR_PERFCOUNTER0_HI ); REG_OUTPUT( mmTCR_PERFCOUNTER1_HI );
                    REG_OUTPUT( mmTCR_PERFCOUNTER0_LOW ); REG_OUTPUT( mmTCR_PERFCOUNTER1_LOW ); REG_OUTPUT( mmTP_TC_CLKGATE_CNTL );
                    REG_OUTPUT( mmTPC_CNTL_STATUS ); REG_OUTPUT( mmTPC_DEBUG0 ); REG_OUTPUT( mmTPC_DEBUG1 );
                    REG_OUTPUT( mmTPC_CHICKEN ); REG_OUTPUT( mmTP0_CNTL_STATUS ); REG_OUTPUT( mmTP0_DEBUG );
                    REG_OUTPUT( mmTP0_CHICKEN ); REG_OUTPUT( mmTP0_PERFCOUNTER0_SELECT ); REG_OUTPUT( mmTP0_PERFCOUNTER0_HI );
                    REG_OUTPUT( mmTP0_PERFCOUNTER0_LOW ); REG_OUTPUT( mmTP0_PERFCOUNTER1_SELECT ); REG_OUTPUT( mmTP0_PERFCOUNTER1_HI );
                    REG_OUTPUT( mmTP0_PERFCOUNTER1_LOW ); REG_OUTPUT( mmTCM_PERFCOUNTER0_SELECT ); REG_OUTPUT( mmTCM_PERFCOUNTER1_SELECT );
                    REG_OUTPUT( mmTCM_PERFCOUNTER0_HI ); REG_OUTPUT( mmTCM_PERFCOUNTER1_HI ); REG_OUTPUT( mmTCM_PERFCOUNTER0_LOW );
                    REG_OUTPUT( mmTCM_PERFCOUNTER1_LOW ); REG_OUTPUT( mmTCF_PERFCOUNTER0_SELECT ); REG_OUTPUT( mmTCF_PERFCOUNTER1_SELECT );
                    REG_OUTPUT( mmTCF_PERFCOUNTER2_SELECT ); REG_OUTPUT( mmTCF_PERFCOUNTER3_SELECT ); REG_OUTPUT( mmTCF_PERFCOUNTER4_SELECT );
                    REG_OUTPUT( mmTCF_PERFCOUNTER5_SELECT ); REG_OUTPUT( mmTCF_PERFCOUNTER6_SELECT ); REG_OUTPUT( mmTCF_PERFCOUNTER7_SELECT );
                    REG_OUTPUT( mmTCF_PERFCOUNTER8_SELECT ); REG_OUTPUT( mmTCF_PERFCOUNTER9_SELECT ); REG_OUTPUT( mmTCF_PERFCOUNTER10_SELECT );
                    REG_OUTPUT( mmTCF_PERFCOUNTER11_SELECT ); REG_OUTPUT( mmTCF_PERFCOUNTER0_HI ); REG_OUTPUT( mmTCF_PERFCOUNTER1_HI );
                    REG_OUTPUT( mmTCF_PERFCOUNTER2_HI ); REG_OUTPUT( mmTCF_PERFCOUNTER3_HI ); REG_OUTPUT( mmTCF_PERFCOUNTER4_HI );
                    REG_OUTPUT( mmTCF_PERFCOUNTER5_HI ); REG_OUTPUT( mmTCF_PERFCOUNTER6_HI ); REG_OUTPUT( mmTCF_PERFCOUNTER7_HI );
                    REG_OUTPUT( mmTCF_PERFCOUNTER8_HI ); REG_OUTPUT( mmTCF_PERFCOUNTER9_HI ); REG_OUTPUT( mmTCF_PERFCOUNTER10_HI );
                    REG_OUTPUT( mmTCF_PERFCOUNTER11_HI ); REG_OUTPUT( mmTCF_PERFCOUNTER0_LOW ); REG_OUTPUT( mmTCF_PERFCOUNTER1_LOW );
                    REG_OUTPUT( mmTCF_PERFCOUNTER2_LOW ); REG_OUTPUT( mmTCF_PERFCOUNTER3_LOW ); REG_OUTPUT( mmTCF_PERFCOUNTER4_LOW );
                    REG_OUTPUT( mmTCF_PERFCOUNTER5_LOW ); REG_OUTPUT( mmTCF_PERFCOUNTER6_LOW ); REG_OUTPUT( mmTCF_PERFCOUNTER7_LOW );
                    REG_OUTPUT( mmTCF_PERFCOUNTER8_LOW ); REG_OUTPUT( mmTCF_PERFCOUNTER9_LOW ); REG_OUTPUT( mmTCF_PERFCOUNTER10_LOW );
                    REG_OUTPUT( mmTCF_PERFCOUNTER11_LOW ); REG_OUTPUT( mmTCF_DEBUG ); REG_OUTPUT( mmTCA_FIFO_DEBUG );
                    REG_OUTPUT( mmTCA_PROBE_DEBUG ); REG_OUTPUT( mmTCA_TPC_DEBUG ); REG_OUTPUT( mmTCB_CORE_DEBUG );
                    REG_OUTPUT( mmTCB_TAG0_DEBUG ); REG_OUTPUT( mmTCB_TAG1_DEBUG ); REG_OUTPUT( mmTCB_TAG2_DEBUG );
                    REG_OUTPUT( mmTCB_TAG3_DEBUG ); REG_OUTPUT( mmTCB_FETCH_GEN_SECTOR_WALKER0_DEBUG ); REG_OUTPUT( mmTCB_FETCH_GEN_WALKER_DEBUG );
                    REG_OUTPUT( mmTCB_FETCH_GEN_PIPE0_DEBUG ); REG_OUTPUT( mmTCD_INPUT0_DEBUG ); REG_OUTPUT( mmTCD_DEGAMMA_DEBUG );
                    REG_OUTPUT( mmTCD_DXTMUX_SCTARB_DEBUG ); REG_OUTPUT( mmTCD_DXTC_ARB_DEBUG ); REG_OUTPUT( mmTCD_STALLS_DEBUG );
                    REG_OUTPUT( mmTCO_STALLS_DEBUG ); REG_OUTPUT( mmTCO_QUAD0_DEBUG0 ); REG_OUTPUT( mmTCO_QUAD0_DEBUG1 );
                    REG_OUTPUT( mmSQ_GPR_MANAGEMENT ); REG_OUTPUT( mmSQ_FLOW_CONTROL ); REG_OUTPUT( mmSQ_INST_STORE_MANAGMENT );
                    REG_OUTPUT( mmSQ_RESOURCE_MANAGMENT ); REG_OUTPUT( mmSQ_EO_RT ); REG_OUTPUT( mmSQ_DEBUG_MISC );
                    REG_OUTPUT( mmSQ_ACTIVITY_METER_CNTL ); REG_OUTPUT( mmSQ_ACTIVITY_METER_STATUS ); REG_OUTPUT( mmSQ_INPUT_ARB_PRIORITY );
                    REG_OUTPUT( mmSQ_THREAD_ARB_PRIORITY ); REG_OUTPUT( mmSQ_VS_WATCHDOG_TIMER ); REG_OUTPUT( mmSQ_PS_WATCHDOG_TIMER );
                    REG_OUTPUT( mmSQ_INT_CNTL ); REG_OUTPUT( mmSQ_INT_STATUS ); REG_OUTPUT( mmSQ_INT_ACK );
                    REG_OUTPUT( mmSQ_DEBUG_INPUT_FSM ); REG_OUTPUT( mmSQ_DEBUG_CONST_MGR_FSM ); REG_OUTPUT( mmSQ_DEBUG_TP_FSM );
                    REG_OUTPUT( mmSQ_DEBUG_FSM_ALU_0 ); REG_OUTPUT( mmSQ_DEBUG_FSM_ALU_1 ); REG_OUTPUT( mmSQ_DEBUG_EXP_ALLOC );
                    REG_OUTPUT( mmSQ_DEBUG_PTR_BUFF ); REG_OUTPUT( mmSQ_DEBUG_GPR_VTX ); REG_OUTPUT( mmSQ_DEBUG_GPR_PIX );
                    REG_OUTPUT( mmSQ_DEBUG_TB_STATUS_SEL ); REG_OUTPUT( mmSQ_DEBUG_VTX_TB_0 ); REG_OUTPUT( mmSQ_DEBUG_VTX_TB_1 );
                    REG_OUTPUT( mmSQ_DEBUG_VTX_TB_STATUS_REG ); REG_OUTPUT( mmSQ_DEBUG_VTX_TB_STATE_MEM ); REG_OUTPUT( mmSQ_DEBUG_PIX_TB_0 );
                    REG_OUTPUT( mmSQ_DEBUG_PIX_TB_STATUS_REG_0 ); REG_OUTPUT( mmSQ_DEBUG_PIX_TB_STATUS_REG_1 ); REG_OUTPUT( mmSQ_DEBUG_PIX_TB_STATUS_REG_2 );
                    REG_OUTPUT( mmSQ_DEBUG_PIX_TB_STATUS_REG_3 ); REG_OUTPUT( mmSQ_DEBUG_PIX_TB_STATE_MEM ); REG_OUTPUT( mmSQ_PERFCOUNTER0_SELECT );
                    REG_OUTPUT( mmSQ_PERFCOUNTER1_SELECT ); REG_OUTPUT( mmSQ_PERFCOUNTER2_SELECT ); REG_OUTPUT( mmSQ_PERFCOUNTER3_SELECT );
                    REG_OUTPUT( mmSQ_PERFCOUNTER0_LOW ); REG_OUTPUT( mmSQ_PERFCOUNTER0_HI ); REG_OUTPUT( mmSQ_PERFCOUNTER1_LOW );
                    REG_OUTPUT( mmSQ_PERFCOUNTER1_HI ); REG_OUTPUT( mmSQ_PERFCOUNTER2_LOW ); REG_OUTPUT( mmSQ_PERFCOUNTER2_HI );
                    REG_OUTPUT( mmSQ_PERFCOUNTER3_LOW ); REG_OUTPUT( mmSQ_PERFCOUNTER3_HI ); REG_OUTPUT( mmSX_PERFCOUNTER0_SELECT );
                    REG_OUTPUT( mmSX_PERFCOUNTER0_LOW ); REG_OUTPUT( mmSX_PERFCOUNTER0_HI ); REG_OUTPUT( mmSQ_INSTRUCTION_ALU_0 );
                    REG_OUTPUT( mmSQ_INSTRUCTION_ALU_1 ); REG_OUTPUT( mmSQ_INSTRUCTION_ALU_2 ); REG_OUTPUT( mmSQ_INSTRUCTION_CF_EXEC_0 );
                    REG_OUTPUT( mmSQ_INSTRUCTION_CF_EXEC_1 ); REG_OUTPUT( mmSQ_INSTRUCTION_CF_EXEC_2 ); REG_OUTPUT( mmSQ_INSTRUCTION_CF_LOOP_0 );
                    REG_OUTPUT( mmSQ_INSTRUCTION_CF_LOOP_1 ); REG_OUTPUT( mmSQ_INSTRUCTION_CF_LOOP_2 ); REG_OUTPUT( mmSQ_INSTRUCTION_CF_JMP_CALL_0 );
                    REG_OUTPUT( mmSQ_INSTRUCTION_CF_JMP_CALL_1 ); REG_OUTPUT( mmSQ_INSTRUCTION_CF_JMP_CALL_2 ); REG_OUTPUT( mmSQ_INSTRUCTION_CF_ALLOC_0 );
                    REG_OUTPUT( mmSQ_INSTRUCTION_CF_ALLOC_1 ); REG_OUTPUT( mmSQ_INSTRUCTION_CF_ALLOC_2 ); REG_OUTPUT( mmSQ_INSTRUCTION_TFETCH_0 );
                    REG_OUTPUT( mmSQ_INSTRUCTION_TFETCH_1 ); REG_OUTPUT( mmSQ_INSTRUCTION_TFETCH_2 ); REG_OUTPUT( mmSQ_INSTRUCTION_VFETCH_0 );
                    REG_OUTPUT( mmSQ_INSTRUCTION_VFETCH_1 ); REG_OUTPUT( mmSQ_INSTRUCTION_VFETCH_2 ); REG_OUTPUT( mmSQ_CONSTANT_0 );
                    REG_OUTPUT( mmSQ_CONSTANT_1 ); REG_OUTPUT( mmSQ_CONSTANT_2 ); REG_OUTPUT( mmSQ_CONSTANT_3 );
                    REG_OUTPUT( mmSQ_FETCH_0 ); REG_OUTPUT( mmSQ_FETCH_1 ); REG_OUTPUT( mmSQ_FETCH_2 );
                    REG_OUTPUT( mmSQ_FETCH_3 ); REG_OUTPUT( mmSQ_FETCH_4 ); REG_OUTPUT( mmSQ_FETCH_5 );
                    REG_OUTPUT( mmSQ_CONSTANT_VFETCH_0 ); REG_OUTPUT( mmSQ_CONSTANT_VFETCH_1 ); REG_OUTPUT( mmSQ_CONSTANT_T2 );
                    REG_OUTPUT( mmSQ_CONSTANT_T3 ); REG_OUTPUT( mmSQ_CF_BOOLEANS ); REG_OUTPUT( mmSQ_CF_LOOP );
                    REG_OUTPUT( mmSQ_CONSTANT_RT_0 ); REG_OUTPUT( mmSQ_CONSTANT_RT_1 ); REG_OUTPUT( mmSQ_CONSTANT_RT_2 );
                    REG_OUTPUT( mmSQ_CONSTANT_RT_3 ); REG_OUTPUT( mmSQ_FETCH_RT_0 ); REG_OUTPUT( mmSQ_FETCH_RT_1 );
                    REG_OUTPUT( mmSQ_FETCH_RT_2 ); REG_OUTPUT( mmSQ_FETCH_RT_3 ); REG_OUTPUT( mmSQ_FETCH_RT_4 );
                    REG_OUTPUT( mmSQ_FETCH_RT_5 ); REG_OUTPUT( mmSQ_CF_RT_BOOLEANS ); REG_OUTPUT( mmSQ_CF_RT_LOOP );
                    REG_OUTPUT( mmSQ_VS_PROGRAM ); REG_OUTPUT( mmSQ_PS_PROGRAM ); REG_OUTPUT( mmSQ_CF_PROGRAM_SIZE );
                    REG_OUTPUT( mmSQ_INTERPOLATOR_CNTL ); REG_OUTPUT( mmSQ_PROGRAM_CNTL ); REG_OUTPUT( mmSQ_WRAPPING_0 );
                    REG_OUTPUT( mmSQ_WRAPPING_1 ); REG_OUTPUT( mmSQ_VS_CONST ); REG_OUTPUT( mmSQ_PS_CONST );
                    REG_OUTPUT( mmSQ_CONTEXT_MISC ); REG_OUTPUT( mmSQ_CF_RD_BASE ); REG_OUTPUT( mmSQ_DEBUG_MISC_0 );
                    REG_OUTPUT( mmSQ_DEBUG_MISC_1 ); REG_OUTPUT( mmMH_ARBITER_CONFIG ); REG_OUTPUT( mmMH_CLNT_AXI_ID_REUSE );
                    REG_OUTPUT( mmMH_INTERRUPT_MASK ); REG_OUTPUT( mmMH_INTERRUPT_STATUS ); REG_OUTPUT( mmMH_INTERRUPT_CLEAR );
                    REG_OUTPUT( mmMH_AXI_ERROR ); REG_OUTPUT( mmMH_PERFCOUNTER0_SELECT ); REG_OUTPUT( mmMH_PERFCOUNTER1_SELECT );
                    REG_OUTPUT( mmMH_PERFCOUNTER0_CONFIG ); REG_OUTPUT( mmMH_PERFCOUNTER1_CONFIG ); REG_OUTPUT( mmMH_PERFCOUNTER0_LOW );
                    REG_OUTPUT( mmMH_PERFCOUNTER1_LOW ); REG_OUTPUT( mmMH_PERFCOUNTER0_HI ); REG_OUTPUT( mmMH_PERFCOUNTER1_HI );
                    REG_OUTPUT( mmMH_DEBUG_CTRL ); REG_OUTPUT( mmMH_DEBUG_DATA ); REG_OUTPUT( mmMH_AXI_HALT_CONTROL );
                    REG_OUTPUT( mmMH_MMU_CONFIG ); REG_OUTPUT( mmMH_MMU_VA_RANGE ); REG_OUTPUT( mmMH_MMU_PT_BASE );
                    REG_OUTPUT( mmMH_MMU_PAGE_FAULT ); REG_OUTPUT( mmMH_MMU_TRAN_ERROR ); REG_OUTPUT( mmMH_MMU_INVALIDATE );
                    REG_OUTPUT( mmMH_MMU_MPU_BASE ); REG_OUTPUT( mmMH_MMU_MPU_END ); REG_OUTPUT( mmWAIT_UNTIL );
                    REG_OUTPUT( mmRBBM_ISYNC_CNTL ); REG_OUTPUT( mmRBBM_STATUS ); REG_OUTPUT( mmRBBM_DSPLY );
                    REG_OUTPUT( mmRBBM_RENDER_LATEST ); REG_OUTPUT( mmRBBM_RTL_RELEASE ); REG_OUTPUT( mmRBBM_PATCH_RELEASE );
                    REG_OUTPUT( mmRBBM_AUXILIARY_CONFIG ); REG_OUTPUT( mmRBBM_PERIPHID0 ); REG_OUTPUT( mmRBBM_PERIPHID1 );
                    REG_OUTPUT( mmRBBM_PERIPHID2 ); REG_OUTPUT( mmRBBM_PERIPHID3 ); REG_OUTPUT( mmRBBM_CNTL );
                    REG_OUTPUT( mmRBBM_SKEW_CNTL ); REG_OUTPUT( mmRBBM_SOFT_RESET ); REG_OUTPUT( mmRBBM_PM_OVERRIDE1 );
                    REG_OUTPUT( mmRBBM_PM_OVERRIDE2 ); REG_OUTPUT( mmGC_SYS_IDLE ); REG_OUTPUT( mmNQWAIT_UNTIL );
                    REG_OUTPUT( mmRBBM_DEBUG_OUT ); REG_OUTPUT( mmRBBM_DEBUG_CNTL ); REG_OUTPUT( mmRBBM_DEBUG );
                    REG_OUTPUT( mmRBBM_READ_ERROR ); REG_OUTPUT( mmRBBM_WAIT_IDLE_CLOCKS ); REG_OUTPUT( mmRBBM_INT_CNTL );
                    REG_OUTPUT( mmRBBM_INT_STATUS ); REG_OUTPUT( mmRBBM_INT_ACK ); REG_OUTPUT( mmMASTER_INT_SIGNAL );
                    REG_OUTPUT( mmRBBM_PERFCOUNTER1_SELECT ); REG_OUTPUT( mmRBBM_PERFCOUNTER1_LO ); REG_OUTPUT( mmRBBM_PERFCOUNTER1_HI );
                    REG_OUTPUT( mmCP_RB_BASE ); REG_OUTPUT( mmCP_RB_CNTL ); REG_OUTPUT( mmCP_RB_RPTR_ADDR );
                    REG_OUTPUT( mmCP_RB_RPTR ); REG_OUTPUT( mmCP_RB_RPTR_WR ); REG_OUTPUT( mmCP_RB_WPTR );
                    REG_OUTPUT( mmCP_RB_WPTR_DELAY ); REG_OUTPUT( mmCP_RB_WPTR_BASE ); REG_OUTPUT( mmCP_IB1_BASE );
                    REG_OUTPUT( mmCP_IB1_BUFSZ ); REG_OUTPUT( mmCP_IB2_BASE ); REG_OUTPUT( mmCP_IB2_BUFSZ );
                    REG_OUTPUT( mmCP_ST_BASE ); REG_OUTPUT( mmCP_ST_BUFSZ ); REG_OUTPUT( mmCP_QUEUE_THRESHOLDS );
                    REG_OUTPUT( mmCP_MEQ_THRESHOLDS ); REG_OUTPUT( mmCP_CSQ_AVAIL ); REG_OUTPUT( mmCP_STQ_AVAIL );
                    REG_OUTPUT( mmCP_MEQ_AVAIL ); REG_OUTPUT( mmCP_CSQ_RB_STAT ); REG_OUTPUT( mmCP_CSQ_IB1_STAT );
                    REG_OUTPUT( mmCP_CSQ_IB2_STAT ); REG_OUTPUT( mmCP_NON_PREFETCH_CNTRS ); REG_OUTPUT( mmCP_STQ_ST_STAT );
                    REG_OUTPUT( mmCP_MEQ_STAT ); REG_OUTPUT( mmCP_MIU_TAG_STAT ); REG_OUTPUT( mmCP_CMD_INDEX );
                    REG_OUTPUT( mmCP_CMD_DATA ); REG_OUTPUT( mmCP_ME_CNTL ); REG_OUTPUT( mmCP_ME_STATUS );
                    REG_OUTPUT( mmCP_ME_RAM_WADDR ); REG_OUTPUT( mmCP_ME_RAM_RADDR ); REG_OUTPUT( mmCP_ME_RAM_DATA );
                    REG_OUTPUT( mmCP_ME_RDADDR ); REG_OUTPUT( mmCP_DEBUG ); REG_OUTPUT( mmSCRATCH_REG0 );
                    REG_OUTPUT( mmSCRATCH_REG1 ); REG_OUTPUT( mmSCRATCH_REG2 ); REG_OUTPUT( mmSCRATCH_REG3 );
                    REG_OUTPUT( mmSCRATCH_REG4 ); REG_OUTPUT( mmSCRATCH_REG5 ); REG_OUTPUT( mmSCRATCH_REG6 );
                    REG_OUTPUT( mmSCRATCH_REG7 );
                    REG_OUTPUT( mmSCRATCH_UMSK ); REG_OUTPUT( mmSCRATCH_ADDR ); REG_OUTPUT( mmCP_ME_VS_EVENT_SRC );
                    REG_OUTPUT( mmCP_ME_VS_EVENT_ADDR ); REG_OUTPUT( mmCP_ME_VS_EVENT_DATA ); REG_OUTPUT( mmCP_ME_VS_EVENT_ADDR_SWM );
                    REG_OUTPUT( mmCP_ME_VS_EVENT_DATA_SWM ); REG_OUTPUT( mmCP_ME_PS_EVENT_SRC ); REG_OUTPUT( mmCP_ME_PS_EVENT_ADDR );
                    REG_OUTPUT( mmCP_ME_PS_EVENT_DATA ); REG_OUTPUT( mmCP_ME_PS_EVENT_ADDR_SWM ); REG_OUTPUT( mmCP_ME_PS_EVENT_DATA_SWM );
                    REG_OUTPUT( mmCP_ME_CF_EVENT_SRC ); REG_OUTPUT( mmCP_ME_CF_EVENT_ADDR ); REG_OUTPUT( mmCP_ME_CF_EVENT_DATA );
                    REG_OUTPUT( mmCP_ME_NRT_ADDR ); REG_OUTPUT( mmCP_ME_NRT_DATA ); REG_OUTPUT( mmCP_ME_VS_FETCH_DONE_SRC );
                    REG_OUTPUT( mmCP_ME_VS_FETCH_DONE_ADDR ); REG_OUTPUT( mmCP_ME_VS_FETCH_DONE_DATA ); REG_OUTPUT( mmCP_INT_CNTL );
                    REG_OUTPUT( mmCP_INT_STATUS ); REG_OUTPUT( mmCP_INT_ACK ); REG_OUTPUT( mmCP_PFP_UCODE_ADDR );
                    REG_OUTPUT( mmCP_PFP_UCODE_DATA ); REG_OUTPUT( mmCP_PERFMON_CNTL ); REG_OUTPUT( mmCP_PERFCOUNTER_SELECT );
                    REG_OUTPUT( mmCP_PERFCOUNTER_LO ); REG_OUTPUT( mmCP_PERFCOUNTER_HI ); REG_OUTPUT( mmCP_BIN_MASK_LO );
                    REG_OUTPUT( mmCP_BIN_MASK_HI ); REG_OUTPUT( mmCP_BIN_SELECT_LO ); REG_OUTPUT( mmCP_BIN_SELECT_HI );
                    REG_OUTPUT( mmCP_NV_FLAGS_0 ); REG_OUTPUT( mmCP_NV_FLAGS_1 ); REG_OUTPUT( mmCP_NV_FLAGS_2 );
                    REG_OUTPUT( mmCP_NV_FLAGS_3 ); REG_OUTPUT( mmCP_STATE_DEBUG_INDEX ); REG_OUTPUT( mmCP_STATE_DEBUG_DATA );
                    REG_OUTPUT( mmCP_PROG_COUNTER ); REG_OUTPUT( mmCP_STAT ); REG_OUTPUT( mmBIOS_0_SCRATCH );
                    REG_OUTPUT( mmBIOS_1_SCRATCH ); REG_OUTPUT( mmBIOS_2_SCRATCH ); REG_OUTPUT( mmBIOS_3_SCRATCH );
                    REG_OUTPUT( mmBIOS_4_SCRATCH ); REG_OUTPUT( mmBIOS_5_SCRATCH ); REG_OUTPUT( mmBIOS_6_SCRATCH );
                    REG_OUTPUT( mmBIOS_7_SCRATCH ); REG_OUTPUT( mmBIOS_8_SCRATCH ); REG_OUTPUT( mmBIOS_9_SCRATCH );
                    REG_OUTPUT( mmBIOS_10_SCRATCH ); REG_OUTPUT( mmBIOS_11_SCRATCH ); REG_OUTPUT( mmBIOS_12_SCRATCH );
                    REG_OUTPUT( mmBIOS_13_SCRATCH ); REG_OUTPUT( mmBIOS_14_SCRATCH ); REG_OUTPUT( mmBIOS_15_SCRATCH );
                    REG_OUTPUT( mmCOHER_SIZE_PM4 ); REG_OUTPUT( mmCOHER_BASE_PM4 ); REG_OUTPUT( mmCOHER_STATUS_PM4 );
                    REG_OUTPUT( mmCOHER_SIZE_HOST ); REG_OUTPUT( mmCOHER_BASE_HOST ); REG_OUTPUT( mmCOHER_STATUS_HOST );
                    REG_OUTPUT( mmCOHER_DEST_BASE_0 ); REG_OUTPUT( mmCOHER_DEST_BASE_1 ); REG_OUTPUT( mmCOHER_DEST_BASE_2 );
                    REG_OUTPUT( mmCOHER_DEST_BASE_3 ); REG_OUTPUT( mmCOHER_DEST_BASE_4 ); REG_OUTPUT( mmCOHER_DEST_BASE_5 );
                    REG_OUTPUT( mmCOHER_DEST_BASE_6 ); REG_OUTPUT( mmCOHER_DEST_BASE_7 ); REG_OUTPUT( mmRB_SURFACE_INFO );
                    REG_OUTPUT( mmRB_COLOR_INFO ); REG_OUTPUT( mmRB_DEPTH_INFO ); REG_OUTPUT( mmRB_STENCILREFMASK );
                    REG_OUTPUT( mmRB_ALPHA_REF ); REG_OUTPUT( mmRB_COLOR_MASK ); REG_OUTPUT( mmRB_BLEND_RED );
                    REG_OUTPUT( mmRB_BLEND_GREEN ); REG_OUTPUT( mmRB_BLEND_BLUE ); REG_OUTPUT( mmRB_BLEND_ALPHA );
                    REG_OUTPUT( mmRB_FOG_COLOR ); REG_OUTPUT( mmRB_STENCILREFMASK_BF ); REG_OUTPUT( mmRB_DEPTHCONTROL );
                    REG_OUTPUT( mmRB_BLENDCONTROL ); REG_OUTPUT( mmRB_COLORCONTROL ); REG_OUTPUT( mmRB_MODECONTROL );
                    REG_OUTPUT( mmRB_COLOR_DEST_MASK ); REG_OUTPUT( mmRB_COPY_CONTROL ); REG_OUTPUT( mmRB_COPY_DEST_BASE );
                    REG_OUTPUT( mmRB_COPY_DEST_PITCH ); REG_OUTPUT( mmRB_COPY_DEST_INFO ); REG_OUTPUT( mmRB_COPY_DEST_PIXEL_OFFSET );
                    REG_OUTPUT( mmRB_DEPTH_CLEAR ); REG_OUTPUT( mmRB_SAMPLE_COUNT_CTL ); REG_OUTPUT( mmRB_SAMPLE_COUNT_ADDR );
                    REG_OUTPUT( mmRB_BC_CONTROL ); REG_OUTPUT( mmRB_EDRAM_INFO ); REG_OUTPUT( mmRB_CRC_RD_PORT );
                    REG_OUTPUT( mmRB_CRC_CONTROL ); REG_OUTPUT( mmRB_CRC_MASK ); REG_OUTPUT( mmRB_PERFCOUNTER0_SELECT );
                    REG_OUTPUT( mmRB_PERFCOUNTER0_LOW ); REG_OUTPUT( mmRB_PERFCOUNTER0_HI ); REG_OUTPUT( mmRB_TOTAL_SAMPLES );
                    REG_OUTPUT( mmRB_ZPASS_SAMPLES ); REG_OUTPUT( mmRB_ZFAIL_SAMPLES ); REG_OUTPUT( mmRB_SFAIL_SAMPLES );
                    REG_OUTPUT( mmRB_DEBUG_0 ); REG_OUTPUT( mmRB_DEBUG_1 ); REG_OUTPUT( mmRB_DEBUG_2 );
                    REG_OUTPUT( mmRB_DEBUG_3 ); REG_OUTPUT( mmRB_DEBUG_4 ); REG_OUTPUT( mmRB_FLAG_CONTROL );
                    REG_OUTPUT( mmRB_BC_SPARES ); REG_OUTPUT( mmBC_DUMMY_CRAYRB_ENUMS ); REG_OUTPUT( mmBC_DUMMY_CRAYRB_MOREENUMS );

                    default:
                        b += sprintf( b, "%s", "UNKNOWN REGISTER OFFSET" );
                    break;
                }
                // Handle string after %R
                kos_memcpy( b, p1+2, p2-(p1+2) );
                b += (unsigned int)p2-(unsigned int)(p1+2);
                *b = '\0';
            }
            break;


            default:
            {
                int val = va_arg( arguments, int );
                // Standard format. Use vsprintf.
                b += sprintf( b, buffer2, val );
            }
            break;
        }
        

        c = p2;
    }

    // Add this string to all outputs
    output = outputs;

    while( output != NULL )
    {
        // Filter according to the flags
        if( ( output->flags & log_flags ) == log_flags )
        {
            // Passed the filter. Now commit this message.
            switch( output->type )
            {
                case KGSL_OUTPUT_TYPE_MEMBUF:
                    // TODO
                break;

                case KGSL_OUTPUT_TYPE_STDOUT:
                    // Write timestamp if enabled
                    if( output->flags & KGSL_LOG_TIMESTAMP )
                        printf( "[Timestamp: %d] ", kos_timestamp() );
                    // Write process id if enabled
                    if( output->flags & KGSL_LOG_PROCESS_ID )
                        printf( "[Process ID: %d] ", kos_process_getid() );
                    // Write thread id if enabled
                    if( output->flags & KGSL_LOG_THREAD_ID )
                        printf( "[Thread ID: %d] ", kos_thread_getid() );

                    // Write the message
                    printf( buffer );
                break;
                
                case KGSL_OUTPUT_TYPE_FILE:
                    // Write timestamp if enabled
                    if( output->flags & KGSL_LOG_TIMESTAMP )
                        kos_fprintf( output->file, "[Timestamp: %d] ", kos_timestamp() );
                    // Write process id if enabled
                    if( output->flags & KGSL_LOG_PROCESS_ID )
                        kos_fprintf( output->file, "[Process ID: %d] ", kos_process_getid() );
                    // Write thread id if enabled
                    if( output->flags & KGSL_LOG_THREAD_ID )
                        kos_fprintf( output->file, "[Thread ID: %d] ", kos_thread_getid() );

                    // Write the message
                    kos_fprintf( output->file, buffer );
                break;
            }
        }
            
        output = output->next;
    }

    va_end( arguments );

    kos_mutex_unlock( log_mutex );

    return GSL_SUCCESS;
}

//----------------------------------------------------------------------------
#endif
