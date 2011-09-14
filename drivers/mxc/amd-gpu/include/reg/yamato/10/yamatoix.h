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

#ifndef _yamatoix_h
#define _yamatoix_h

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

// [SUDEBUGIND] : Indirect Registers

#define ixCLIPPER_DEBUG_REG00                      0x0000
#define ixCLIPPER_DEBUG_REG01                      0x0001
#define ixCLIPPER_DEBUG_REG02                      0x0002
#define ixCLIPPER_DEBUG_REG03                      0x0003
#define ixCLIPPER_DEBUG_REG04                      0x0004
#define ixCLIPPER_DEBUG_REG05                      0x0005
#define ixCLIPPER_DEBUG_REG09                      0x0009
#define ixCLIPPER_DEBUG_REG10                      0x000A
#define ixCLIPPER_DEBUG_REG11                      0x000B
#define ixCLIPPER_DEBUG_REG12                      0x000C
#define ixCLIPPER_DEBUG_REG13                      0x000D
#define ixSXIFCCG_DEBUG_REG0                       0x0011
#define ixSXIFCCG_DEBUG_REG1                       0x0012
#define ixSXIFCCG_DEBUG_REG2                       0x0013
#define ixSXIFCCG_DEBUG_REG3                       0x0014
#define ixSETUP_DEBUG_REG0                         0x0015
#define ixSETUP_DEBUG_REG1                         0x0016
#define ixSETUP_DEBUG_REG2                         0x0017
#define ixSETUP_DEBUG_REG3                         0x0018
#define ixSETUP_DEBUG_REG4                         0x0019
#define ixSETUP_DEBUG_REG5                         0x001A

// [SCDEBUGIND] : Indirect Registers

#define ixSC_DEBUG_0                               0x0000
#define ixSC_DEBUG_1                               0x0001
#define ixSC_DEBUG_2                               0x0002
#define ixSC_DEBUG_3                               0x0003
#define ixSC_DEBUG_4                               0x0004
#define ixSC_DEBUG_5                               0x0005
#define ixSC_DEBUG_6                               0x0006
#define ixSC_DEBUG_7                               0x0007
#define ixSC_DEBUG_8                               0x0008
#define ixSC_DEBUG_9                               0x0009
#define ixSC_DEBUG_10                              0x000A
#define ixSC_DEBUG_11                              0x000B
#define ixSC_DEBUG_12                              0x000C

// [VGTDEBUGIND] : Indirect Registers

#define ixVGT_DEBUG_REG0                           0x0000
#define ixVGT_DEBUG_REG1                           0x0001
#define ixVGT_DEBUG_REG3                           0x0003
#define ixVGT_DEBUG_REG6                           0x0006
#define ixVGT_DEBUG_REG7                           0x0007
#define ixVGT_DEBUG_REG8                           0x0008
#define ixVGT_DEBUG_REG9                           0x0009
#define ixVGT_DEBUG_REG10                          0x000A
#define ixVGT_DEBUG_REG12                          0x000C
#define ixVGT_DEBUG_REG13                          0x000D
#define ixVGT_DEBUG_REG14                          0x000E
#define ixVGT_DEBUG_REG15                          0x000F
#define ixVGT_DEBUG_REG16                          0x0010
#define ixVGT_DEBUG_REG17                          0x0011
#define ixVGT_DEBUG_REG18                          0x0012
#define ixVGT_DEBUG_REG20                          0x0014
#define ixVGT_DEBUG_REG21                          0x0015

// [MHDEBUGIND] : Indirect Registers

#define ixMH_DEBUG_REG00                           0x0000
#define ixMH_DEBUG_REG01                           0x0001
#define ixMH_DEBUG_REG02                           0x0002
#define ixMH_DEBUG_REG03                           0x0003
#define ixMH_DEBUG_REG04                           0x0004
#define ixMH_DEBUG_REG05                           0x0005
#define ixMH_DEBUG_REG06                           0x0006
#define ixMH_DEBUG_REG07                           0x0007
#define ixMH_DEBUG_REG08                           0x0008
#define ixMH_DEBUG_REG09                           0x0009
#define ixMH_DEBUG_REG10                           0x000A
#define ixMH_DEBUG_REG11                           0x000B
#define ixMH_DEBUG_REG12                           0x000C
#define ixMH_DEBUG_REG13                           0x000D
#define ixMH_DEBUG_REG14                           0x000E
#define ixMH_DEBUG_REG15                           0x000F
#define ixMH_DEBUG_REG16                           0x0010
#define ixMH_DEBUG_REG17                           0x0011
#define ixMH_DEBUG_REG18                           0x0012
#define ixMH_DEBUG_REG19                           0x0013
#define ixMH_DEBUG_REG20                           0x0014
#define ixMH_DEBUG_REG21                           0x0015
#define ixMH_DEBUG_REG22                           0x0016
#define ixMH_DEBUG_REG23                           0x0017
#define ixMH_DEBUG_REG24                           0x0018
#define ixMH_DEBUG_REG25                           0x0019
#define ixMH_DEBUG_REG26                           0x001A
#define ixMH_DEBUG_REG27                           0x001B
#define ixMH_DEBUG_REG28                           0x001C
#define ixMH_DEBUG_REG29                           0x001D
#define ixMH_DEBUG_REG30                           0x001E
#define ixMH_DEBUG_REG31                           0x001F
#define ixMH_DEBUG_REG32                           0x0020
#define ixMH_DEBUG_REG33                           0x0021
#define ixMH_DEBUG_REG34                           0x0022
#define ixMH_DEBUG_REG35                           0x0023
#define ixMH_DEBUG_REG36                           0x0024
#define ixMH_DEBUG_REG37                           0x0025
#define ixMH_DEBUG_REG38                           0x0026
#define ixMH_DEBUG_REG39                           0x0027
#define ixMH_DEBUG_REG40                           0x0028
#define ixMH_DEBUG_REG41                           0x0029
#define ixMH_DEBUG_REG42                           0x002A
#define ixMH_DEBUG_REG43                           0x002B
#define ixMH_DEBUG_REG44                           0x002C
#define ixMH_DEBUG_REG45                           0x002D
#define ixMH_DEBUG_REG46                           0x002E
#define ixMH_DEBUG_REG47                           0x002F
#define ixMH_DEBUG_REG48                           0x0030
#define ixMH_DEBUG_REG49                           0x0031
#define ixMH_DEBUG_REG50                           0x0032
#define ixMH_DEBUG_REG51                           0x0033
#define ixMH_DEBUG_REG52                           0x0034
#define ixMH_DEBUG_REG53                           0x0035
#define ixMH_DEBUG_REG54                           0x0036
#define ixMH_DEBUG_REG55                           0x0037
#define ixMH_DEBUG_REG56                           0x0038
#define ixMH_DEBUG_REG57                           0x0039
#define ixMH_DEBUG_REG58                           0x003A
#define ixMH_DEBUG_REG59                           0x003B
#define ixMH_DEBUG_REG60                           0x003C
#define ixMH_DEBUG_REG61                           0x003D
#define ixMH_DEBUG_REG62                           0x003E
#define ixMH_DEBUG_REG63                           0x003F


#ifdef __cplusplus
}
#endif  // __cplusplus

#endif // _yamatob_h

