/*
 * Freescale OCOTP Register Definitions
 *
 * Copyright 2008-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 * This file is created by xml file. Don't Edit it.
 *
 * Xml Revision: 1.12
 * Template revision: 1.3
 */

#ifndef __ARCH_ARM___OCOTP_H
#define __ARCH_ARM___OCOTP_H


#define HW_OCOTP_CTRL	(0x00000000)
#define HW_OCOTP_CTRL_SET	(0x00000004)
#define HW_OCOTP_CTRL_CLR	(0x00000008)
#define HW_OCOTP_CTRL_TOG	(0x0000000c)

#define BP_OCOTP_CTRL_WR_UNLOCK      16
#define BM_OCOTP_CTRL_WR_UNLOCK 0xFFFF0000
#define BF_OCOTP_CTRL_WR_UNLOCK(v) \
	(((v) << 16) & BM_OCOTP_CTRL_WR_UNLOCK)
#define BV_OCOTP_CTRL_WR_UNLOCK__KEY 0x3E77
#define BP_OCOTP_CTRL_RSRVD2      14
#define BM_OCOTP_CTRL_RSRVD2 0x0000C000
#define BF_OCOTP_CTRL_RSRVD2(v)  \
	(((v) << 14) & BM_OCOTP_CTRL_RSRVD2)
#define BM_OCOTP_CTRL_RELOAD_SHADOWS 0x00002000
#define BM_OCOTP_CTRL_RD_BANK_OPEN 0x00001000
#define BP_OCOTP_CTRL_RSRVD1      10
#define BM_OCOTP_CTRL_RSRVD1 0x00000C00
#define BF_OCOTP_CTRL_RSRVD1(v)  \
	(((v) << 10) & BM_OCOTP_CTRL_RSRVD1)
#define BM_OCOTP_CTRL_ERROR 0x00000200
#define BM_OCOTP_CTRL_BUSY 0x00000100
#define BP_OCOTP_CTRL_RSRVD0      6
#define BM_OCOTP_CTRL_RSRVD0 0x000000C0
#define BF_OCOTP_CTRL_RSRVD0(v)  \
	(((v) << 6) & BM_OCOTP_CTRL_RSRVD0)
#define BP_OCOTP_CTRL_ADDR      0
#define BM_OCOTP_CTRL_ADDR 0x0000003F
#define BF_OCOTP_CTRL_ADDR(v)  \
	(((v) << 0) & BM_OCOTP_CTRL_ADDR)

#define HW_OCOTP_TIMING	(0x00000010)

#define BP_OCOTP_TIMING_RSRVD0      22
#define BM_OCOTP_TIMING_RSRVD0 0xFFC00000
#define BF_OCOTP_TIMING_RSRVD0(v) \
	(((v) << 22) & BM_OCOTP_TIMING_RSRVD0)
#define BP_OCOTP_TIMING_RD_BUSY      16
#define BM_OCOTP_TIMING_RD_BUSY 0x003F0000
#define BF_OCOTP_TIMING_RD_BUSY(v)  \
	(((v) << 16) & BM_OCOTP_TIMING_RD_BUSY)
#define BP_OCOTP_TIMING_RELAX      12
#define BM_OCOTP_TIMING_RELAX 0x0000F000
#define BF_OCOTP_TIMING_RELAX(v)  \
	(((v) << 12) & BM_OCOTP_TIMING_RELAX)
#define BP_OCOTP_TIMING_SCLK_COUNT      0
#define BM_OCOTP_TIMING_SCLK_COUNT 0x00000FFF
#define BF_OCOTP_TIMING_SCLK_COUNT(v)  \
	(((v) << 0) & BM_OCOTP_TIMING_SCLK_COUNT)

#define HW_OCOTP_DATA	(0x00000020)

#define BP_OCOTP_DATA_DATA      0
#define BM_OCOTP_DATA_DATA 0xFFFFFFFF
#define BF_OCOTP_DATA_DATA(v)   (v)

#define HW_OCOTP_LOCK	(0x00000030)

#define BP_OCOTP_LOCK_UNALLOCATED      26
#define BM_OCOTP_LOCK_UNALLOCATED 0xFC000000
#define BF_OCOTP_LOCK_UNALLOCATED(v) \
	(((v) << 26) & BM_OCOTP_LOCK_UNALLOCATED)
#define BM_OCOTP_LOCK_PIN 0x02000000
#define BM_OCOTP_LOCK_DCPKEY_ALT 0x01000000
#define BM_OCOTP_LOCK_SCC_ALT 0x00800000
#define BM_OCOTP_LOCK_DCPKEY 0x00400000
#define BM_OCOTP_LOCK_SWCAP_SHADOW 0x00200000
#define BM_OCOTP_LOCK_SWCAP 0x00100000
#define BM_OCOTP_LOCK_HWCAP_SHADOW 0x00080000
#define BM_OCOTP_LOCK_HWCAP 0x00040000
#define BM_OCOTP_LOCK_MAC_SHADOW 0x00020000
#define BM_OCOTP_LOCK_MAC 0x00010000
#define BM_OCOTP_LOCK_SJC_SHADOW 0x00008000
#define BM_OCOTP_LOCK_SJC 0x00004000
#define BM_OCOTP_LOCK_SRK_SHADOW 0x00002000
#define BM_OCOTP_LOCK_SRK 0x00001000
#define BM_OCOTP_LOCK_SCC_SHADOW 0x00000800
#define BM_OCOTP_LOCK_SCC 0x00000400
#define BM_OCOTP_LOCK_GP_SHADOW 0x00000200
#define BM_OCOTP_LOCK_GP 0x00000100
#define BM_OCOTP_LOCK_MEM_MISC_SHADOW 0x00000080
#define BM_OCOTP_LOCK_MEM_TRIM_SHADOW 0x00000040
#define BM_OCOTP_LOCK_MEM_TRIM 0x00000020
#define BM_OCOTP_LOCK_CFG_MISC_SHADOW 0x00000010
#define BM_OCOTP_LOCK_CFG_BOOT_SHADOW 0x00000008
#define BM_OCOTP_LOCK_CFG_BOOT 0x00000004
#define BM_OCOTP_LOCK_CFG_TESTER_SHADOW 0x00000002
#define BM_OCOTP_LOCK_CFG_TESTER 0x00000001

/*
 *  multi-register-define name HW_OCOTP_CFGn
 *              base 0x00000040
 *              count 7
 *              offset 0x10
 */
#define HW_OCOTP_CFGn(n)	(0x00000040 + (n) * 0x10)
#define BP_OCOTP_CFGn_BITS      0
#define BM_OCOTP_CFGn_BITS 0xFFFFFFFF
#define BF_OCOTP_CFGn_BITS(v)   (v)

/*
 *  multi-register-define name HW_OCOTP_MEMn
 *              base 0x000000B0
 *              count 6
 *              offset 0x10
 */
#define HW_OCOTP_MEMn(n)	(0x000000b0 + (n) * 0x10)
#define BP_OCOTP_MEMn_BITS      0
#define BM_OCOTP_MEMn_BITS 0xFFFFFFFF
#define BF_OCOTP_MEMn_BITS(v)   (v)

/*
 *  multi-register-define name HW_OCOTP_GPn
 *              base 0x00000110
 *              count 2
 *              offset 0x10
 */
#define HW_OCOTP_GPn(n)	(0x00000110 + (n) * 0x10)
#define BP_OCOTP_GPn_BITS      0
#define BM_OCOTP_GPn_BITS 0xFFFFFFFF
#define BF_OCOTP_GPn_BITS(v)   (v)

/*
 *  multi-register-define name HW_OCOTP_SCCn
 *              base 0x00000130
 *              count 8
 *              offset 0x10
 */
#define HW_OCOTP_SCCn(n)	(0x00000130 + (n) * 0x10)
#define BP_OCOTP_SCCn_BITS      0
#define BM_OCOTP_SCCn_BITS 0xFFFFFFFF
#define BF_OCOTP_SCCn_BITS(v)   (v)

/*
 *  multi-register-define name HW_OCOTP_SRKn
 *              base 0x000001B0
 *              count 8
 *              offset 0x10
 */
#define HW_OCOTP_SRKn(n)	(0x000001b0 + (n) * 0x10)
#define BP_OCOTP_SRKn_BITS      0
#define BM_OCOTP_SRKn_BITS 0xFFFFFFFF
#define BF_OCOTP_SRKn_BITS(v)   (v)

/*
 *  multi-register-define name HW_OCOTP_SJC_RESPn
 *              base 0x00000230
 *              count 2
 *              offset 0x10
 */
#define HW_OCOTP_SJC_RESPn(n)	(0x00000230 + (n) * 0x10)
#define BP_OCOTP_SJC_RESPn_BITS      0
#define BM_OCOTP_SJC_RESPn_BITS 0xFFFFFFFF
#define BF_OCOTP_SJC_RESPn_BITS(v)   (v)

/*
 *  multi-register-define name HW_OCOTP_MACn
 *              base 0x00000250
 *              count 2
 *              offset 0x10
 */
#define HW_OCOTP_MACn(n)	(0x00000250 + (n) * 0x10)
#define BP_OCOTP_MACn_BITS      0
#define BM_OCOTP_MACn_BITS 0xFFFFFFFF
#define BF_OCOTP_MACn_BITS(v)   (v)

/*
 *  multi-register-define name HW_OCOTP_HWCAPn
 *              base 0x00000270
 *              count 3
 *              offset 0x10
 */
#define HW_OCOTP_HWCAPn(n)	(0x00000270 + (n) * 0x10)
#define BP_OCOTP_HWCAPn_BITS      0
#define BM_OCOTP_HWCAPn_BITS 0xFFFFFFFF
#define BF_OCOTP_HWCAPn_BITS(v)   (v)

#define HW_OCOTP_SWCAP	(0x000002a0)

#define BP_OCOTP_SWCAP_BITS      0
#define BM_OCOTP_SWCAP_BITS 0xFFFFFFFF
#define BF_OCOTP_SWCAP_BITS(v)   (v)

#define HW_OCOTP_SCS	(0x000002b0)
#define HW_OCOTP_SCS_SET	(0x000002b4)
#define HW_OCOTP_SCS_CLR	(0x000002b8)
#define HW_OCOTP_SCS_TOG	(0x000002bc)

#define BM_OCOTP_SCS_LOCK 0x80000000
#define BP_OCOTP_SCS_SPARE      1
#define BM_OCOTP_SCS_SPARE 0x7FFFFFFE
#define BF_OCOTP_SCS_SPARE(v)  \
	(((v) << 1) & BM_OCOTP_SCS_SPARE)
#define BM_OCOTP_SCS_HAB_JDE 0x00000001

#define HW_OCOTP_VERSION	(0x000002c0)

#define BP_OCOTP_VERSION_MAJOR      24
#define BM_OCOTP_VERSION_MAJOR 0xFF000000
#define BF_OCOTP_VERSION_MAJOR(v) \
	(((v) << 24) & BM_OCOTP_VERSION_MAJOR)
#define BP_OCOTP_VERSION_MINOR      16
#define BM_OCOTP_VERSION_MINOR 0x00FF0000
#define BF_OCOTP_VERSION_MINOR(v)  \
	(((v) << 16) & BM_OCOTP_VERSION_MINOR)
#define BP_OCOTP_VERSION_STEP      0
#define BM_OCOTP_VERSION_STEP 0x0000FFFF
#define BF_OCOTP_VERSION_STEP(v)  \
	(((v) << 0) & BM_OCOTP_VERSION_STEP)
#endif /* __ARCH_ARM___OCOTP_H */
