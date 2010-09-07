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

#ifndef __GSL_PM4TYPES_H
#define __GSL_PM4TYPES_H


//////////////////////////////////////////////////////////////////////////////
// packet mask
//////////////////////////////////////////////////////////////////////////////
#define PM4_PKT_MASK                0xc0000000


//////////////////////////////////////////////////////////////////////////////
// packet types
//////////////////////////////////////////////////////////////////////////////
#define PM4_TYPE0_PKT               ((unsigned int)0 << 30)
#define PM4_TYPE1_PKT               ((unsigned int)1 << 30)
#define PM4_TYPE2_PKT               ((unsigned int)2 << 30)
#define PM4_TYPE3_PKT               ((unsigned int)3 << 30)


//////////////////////////////////////////////////////////////////////////////
// type3 packets
//////////////////////////////////////////////////////////////////////////////
#define PM4_ME_INIT                 0x48    // initialize CP's micro-engine

#define PM4_NOP                     0x10    // skip N 32-bit words to get to the next packet

#define PM4_INDIRECT_BUFFER         0x3f    // indirect buffer dispatch.  prefetch parser uses this packet type to determine whether to pre-fetch the IB
#define PM4_INDIRECT_BUFFER_PFD     0x37    // indirect buffer dispatch.  same as IB, but init is pipelined

#define PM4_WAIT_FOR_IDLE           0x26    // wait for the IDLE state of the engine
#define PM4_WAIT_REG_MEM            0x3c    // wait until a register or memory location is a specific value
#define PM4_WAIT_REG_EQ             0x52    // wait until a register location is equal to a specific value
#define PM4_WAT_REG_GTE             0x53    // wait until a register location is >= a specific value
#define PM4_WAIT_UNTIL_READ         0x5c    // wait until a read completes
#define PM4_WAIT_IB_PFD_COMPLETE    0x5d    // wait until all base/size writes from an IB_PFD packet have completed

#define PM4_REG_RMW                 0x21    // register read/modify/write
#define PM4_REG_TO_MEM              0x3e    // reads register in chip and writes to memory
#define PM4_MEM_WRITE               0x3d    // write N 32-bit words to memory
#define PM4_MEM_WRITE_CNTR          0x4f    // write CP_PROG_COUNTER value to memory
#define PM4_COND_EXEC               0x44    // conditional execution of a sequence of packets
#define PM4_COND_WRITE              0x45    // conditional write to memory or register

#define PM4_EVENT_WRITE             0x46    // generate an event that creates a write to memory when completed
#define PM4_EVENT_WRITE_SHD         0x58    // generate a VS|PS_done event
#define PM4_EVENT_WRITE_CFL         0x59    // generate a cache flush done event
#define PM4_EVENT_WRITE_ZPD         0x5b    // generate a z_pass done event

#define PM4_DRAW_INDX               0x22    // initiate fetch of index buffer and draw
#define PM4_DRAW_INDX_2             0x36    // draw using supplied indices in packet
#define PM4_DRAW_INDX_BIN           0x34    // initiate fetch of index buffer and binIDs and draw
#define PM4_DRAW_INDX_2_BIN         0x35    // initiate fetch of bin IDs and draw using supplied indices

#define PM4_VIZ_QUERY               0x23    // begin/end initiator for viz query extent processing
#define PM4_SET_STATE               0x25    // fetch state sub-blocks and initiate shader code DMAs
#define PM4_SET_CONSTANT            0x2d    // load constant into chip and to memory
#define PM4_IM_LOAD                 0x27    // load sequencer instruction memory (pointer-based)
#define PM4_IM_LOAD_IMMEDIATE       0x2b    // load sequencer instruction memory (code embedded in packet)
#define PM4_LOAD_CONSTANT_CONTEXT   0x2e    // load constants from a location in memory
#define PM4_INVALIDATE_STATE        0x3b    // selective invalidation of state pointers

#define PM4_SET_SHADER_BASES        0x4A    // dynamically changes shader instruction memory partition
#define PM4_SET_BIN_BASE_OFFSET     0x4B    // program an offset that will added to the BIN_BASE value of the 3D_DRAW_INDX_BIN packet
#define PM4_SET_BIN_MASK            0x50    // sets the 64-bit BIN_MASK register in the PFP
#define PM4_SET_BIN_SELECT          0x51    // sets the 64-bit BIN_SELECT register in the PFP

#define PM4_CONTEXT_UPDATE          0x5e    // updates the current context, if needed
#define PM4_INTERRUPT               0x40    // generate interrupt from the command stream

#define PM4_IM_STORE                0x2c    // copy sequencer instruction memory to system memory


//////////////////////////////////////////////////////////////////////////////
// packet header building macros
//////////////////////////////////////////////////////////////////////////////
#define pm4_type0_packet(regindx, cnt)                      (PM4_TYPE0_PKT | (((cnt)-1) << 16)  | ((regindx) & 0x7FFF))
#define pm4_type0_packet_for_sameregister(regindx, cnt)     (PM4_TYPE0_PKT | (((cnt)-1) << 16)  | ((1 << 15) | ((regindx) & 0x7FFF))
#define pm4_type1_packet(reg0, reg1)                        (PM4_TYPE1_PKT                      | ((reg1) << 12) | (reg0))
#define pm4_type3_packet(opcode, cnt)                       (PM4_TYPE3_PKT | (((cnt)-1) << 16)  | (((opcode) & 0xFF) << 8))
#define pm4_predicated_type3_packet(opcode, cnt)            (PM4_TYPE3_PKT | (((cnt)-1) << 16)  | (((opcode) & 0xFF) << 8) | 0x1))
#define pm4_nop_packet(cnt)                                 (PM4_TYPE3_PKT | (((cnt)-1) << 16)  | (PM4_NOP << 8))


//////////////////////////////////////////////////////////////////////////////
// packet headers
//////////////////////////////////////////////////////////////////////////////
#define PM4_HDR_ME_INIT                 pm4_type3_packet(PM4_ME_INIT, 18)
#define PM4_HDR_INDIRECT_BUFFER_PFD     pm4_type3_packet(PM4_INDIRECT_BUFFER_PFD, 2)
#define PM4_HDR_INDIRECT_BUFFER         pm4_type3_packet(PM4_INDIRECT_BUFFER, 2)


//////////////////////////////////////////////////////////////////////////////
// types
//////////////////////////////////////////////////////////////////////////////

// -----------------------
// pm4 type0 packet header
// -----------------------
typedef struct __pm4_type0
{
    unsigned int       base_index  :15;
    unsigned int       one_reg_wr  :1;
    unsigned int       count       :14;
    unsigned int       type        :2;
} pm4_type0;

// -----------------------
// pm4 type2 packet header
// -----------------------
typedef struct __pm4_type2
{
    unsigned int       reserved    :30;
    unsigned int       type        :2;
} pm4_type2;

// -----------------------
// pm4 type3 packet header
// -----------------------
typedef struct __pm4_type3
{
    unsigned int       predicate   :1;
    unsigned int       reserved1   :7;
    unsigned int       it_opcode   :7;
    unsigned int       reserved2   :1;
    unsigned int       count       :14;
    unsigned int       type        :2;
} pm4_type3;

#endif  // __GSL_PM4TYPES_H
