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

#ifndef _R400IPT_H_
#define _R400IPT_H_

// Hand-generated list from Yamato_PM4_Spec.doc

#define PM4_PACKET0_NOP     0x00000000          // Empty type-0 packet header
#define PM4_PACKET1_NOP     0x40000000          // Empty type-1 packet header
#define PM4_PACKET2_NOP     0x80000000          // Empty type-2 packet header (reserved)

#define PM4_COUNT_SHIFT     16  
#define PM4_COUNT_MASK      
#define PM4_PACKET_COUNT(__x)   ((((__x)-1) << PM4_COUNT_SHIFT) & 0x3fff0000)
// Type 3 packet headers

#define PM4_PACKET3_NOP                         0xC0001000  // Do nothing.
#define PM4_PACKET3_IB_PREFETCH_END             0xC0001700  // Internal Packet Used Only by CP
#define PM4_PACKET3_SUBBLK_PREFETCH             0xC0001F00  // Internal Packet Used Only by CP

#define PM4_PACKET3_INSTR_PREFETCH              0xC0002000  // Internal Packet Used Only by CP
#define PM4_PACKET3_REG_RMW                     0xC0002100  // Register Read-Modify-Write                               New for R400
#define PM4_PACKET3_DRAW_INDX                   0xC0002200  // Initiate fetch of index buffer                           New for R400
#define PM4_PACKET3_VIZ_QUERY                   0xC0002300  // Begin/End initiator for Viz Query extent processing      New for R400
#define PM4_PACKET3_SET_STATE                   0xC0002500  // Fetch State Sub-Blocks and Initiate Shader Code DMAs     New for R400
#define PM4_PACKET3_WAIT_FOR_IDLE               0xC0002600  // Wait for the engine to be idle. 
#define PM4_PACKET3_IM_LOAD                     0xC0002700  // Load Sequencer Instruction Memory for a Specific Shader  New for R400
#define PM4_PACKET3_IM_LOAD_IMMEDIATE           0xC0002B00  // Load Sequencer Instruction Memory for a Specific Shader  New for R400
#define PM4_PACKET3_SET_CONSTANT                0xC0002D00  // Load Constant Into Chip & Shadow to Memory               New for R400
#define PM4_PACKET3_LOAD_CONSTANT_CONTEXT       0xC0002E00  // Load All Constants from a Location in Memory             New for R400
#define PM4_PACKET3_LOAD_ALU_CONSTANT           0xC0002F00  // Load ALu constants from a location in memory - similar to SET_CONSTANT but tuned for performance when loading only ALU constants

#define PM4_PACKET3_DRAW_INDX_BIN               0xC0003400  // Initiate fetch of index buffer and BIN info used for visibility test
#define PM4_PACKET3_3D_DRAW_INDX_2_BIN          0xC0003500  // Draw using supplied indices and initiate fetch of BIN info for visibility test
#define PM4_PACKET3_3D_DRAW_INDX_2              0xC0003600  // Draw primitives using vertex buf and Indices in this packet. Pkt does NOT contain vtx fmt
#define PM4_PACKET3_INDIRECT_BUFFER_PFD         0xC0003700
#define PM4_PACKET3_INVALIDATE_STATE            0xC0003B00  // Selective Invalidation of State Pointers                 New for R400
#define PM4_PACKET3_WAIT_REG_MEM                0xC0003C00  // Wait Until a Register or Memory Location is a Specific Value.   New for R400
#define PM4_PACKET3_MEM_WRITE                   0xC0003D00  // Write DWORD to Memory For Synchronization                New for R400
#define PM4_PACKET3_REG_TO_MEM                  0xC0003E00  // Reads Register in Chip and Writes to Memory              New for R400
#define PM4_PACKET3_INDIRECT_BUFFER             0xC0003F00  // Indirect Buffer Dispatch - Pre-fetch parser uses this packet type in determining to pre-fetch the indirect buffer.  Supported

#define PM4_PACKET3_CP_INTERRUPT                0xC0004000  // Generate Interrupt from the Command Stream               New for R400
#define PM4_PACKET3_COND_EXEC                   0xC0004400  // Conditional execution of a sequence of packets
#define PM4_PACKET3_COND_WRITE                  0xC0004500  // Conditional Write to Memory                              New for R400
#define PM4_PACKET3_EVENT_WRITE                 0xC0004600  // Generate An Event that Creates a Write to Memory when Completed  New for R400
#define PM4_PACKET3_INSTR_MATCH                 0xC0004700  // Internal Packet Used Only by CP
#define PM4_PACKET3_ME_INIT                     0xC0004800  // Initialize CP's Micro Engine                             New for R400
#define PM4_PACKET3_CONST_PREFETCH              0xC0004900  // Internal packet used only by CP
#define PM4_PACKET3_MEM_WRITE_CNTR              0xC0004F00

#define PM4_PACKET3_SET_BIN_MASK                0xC0005000  // Sets the 64-bit BIN_MASK register in the PFP
#define PM4_PACKET3_SET_BIN_SELECT              0xC0005100  // Sets the 64-bit BIN_SELECT register in the PFP
#define PM4_PACKET3_WAIT_REG_EQ                 0xC0005200  // Wait until a register location is equal to a specific value
#define PM4_PACKET3_WAIT_REG_GTE                0xC0005300  // Wait until a register location is greater than or equal to a specific value
#define PM4_PACKET3_INCR_UPDT_STATE             0xC0005500  // Internal Packet Used Only by CP  
#define PM4_PACKET3_INCR_UPDT_CONST             0xC0005600  // Internal Packet Used Only by CP
#define PM4_PACKET3_INCR_UPDT_INSTR             0xC0005700  // Internal Packet Used Only by CP
#define PM4_PACKET3_EVENT_WRITE_SHD             0xC0005800  // Generate a VS|PS_Done Event.
#define PM4_PACKET3_EVENT_WRITE_CFL             0xC0005900  // Generate a Cach Flush Done Event
#define PM4_PACKET3_EVENT_WRITE_ZPD             0xC0005B00  // Generate a Cach Flush Done Event
#define PM4_PACKET3_WAIT_UNTIL_READ             0xC0005C00  // Wait Until a Read completes.
#define PM4_PACKET3_WAIT_IB_PFD_COMPLETE        0xC0005D00  // Wait Until all Base/Size writes from an IB_PFD packet have completed.
#define PM4_PACKET3_CONTEXT_UPDATE              0xC0005E00  // Updates the current context if needed.

    /****** New Opcodes For R400 (all decode values are TBD) ******/


#endif //  _R400IPT_H_
