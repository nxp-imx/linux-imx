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
#define BP_OCOTP_CTRL_RSVD1      13
#define BM_OCOTP_CTRL_RSVD1 0x0000E000
#define BF_OCOTP_CTRL_RSVD1(v)  \
	(((v) << 13) & BM_OCOTP_CTRL_RSVD1)
#define BM_OCOTP_CTRL_CRC_FAIL 0x00001000
#define BM_OCOTP_CTRL_CRC_TEST 0x00000800
#define BM_OCOTP_CTRL_RELOAD_SHADOWS 0x00000400
#define BM_OCOTP_CTRL_ERROR 0x00000200
#define BM_OCOTP_CTRL_BUSY 0x00000100
#define BM_OCOTP_CTRL_RSVD0 0x00000080
#define BP_OCOTP_CTRL_ADDR      0
#define BM_OCOTP_CTRL_ADDR 0x0000007F
#define BF_OCOTP_CTRL_ADDR(v)  \
	(((v) << 0) & BM_OCOTP_CTRL_ADDR)

#define HW_OCOTP_TIMING	(0x00000010)

#define BP_OCOTP_TIMING_RSRVD0      28
#define BM_OCOTP_TIMING_RSRVD0 0xF0000000
#define BF_OCOTP_TIMING_RSRVD0(v) \
	(((v) << 28) & BM_OCOTP_TIMING_RSRVD0)
#define BP_OCOTP_TIMING_WAIT      22
#define BM_OCOTP_TIMING_WAIT 0x0FC00000
#define BF_OCOTP_TIMING_WAIT(v)  \
	(((v) << 22) & BM_OCOTP_TIMING_WAIT)
#define BP_OCOTP_TIMING_STROBE_READ      16
#define BM_OCOTP_TIMING_STROBE_READ 0x003F0000
#define BF_OCOTP_TIMING_STROBE_READ(v)  \
	(((v) << 16) & BM_OCOTP_TIMING_STROBE_READ)
#define BP_OCOTP_TIMING_RELAX      12
#define BM_OCOTP_TIMING_RELAX 0x0000F000
#define BF_OCOTP_TIMING_RELAX(v)  \
	(((v) << 12) & BM_OCOTP_TIMING_RELAX)
#define BP_OCOTP_TIMING_STROBE_PROG      0
#define BM_OCOTP_TIMING_STROBE_PROG 0x00000FFF
#define BF_OCOTP_TIMING_STROBE_PROG(v)  \
	(((v) << 0) & BM_OCOTP_TIMING_STROBE_PROG)

#define HW_OCOTP_DATA	(0x00000020)

#define BP_OCOTP_DATA_DATA      0
#define BM_OCOTP_DATA_DATA 0xFFFFFFFF
#define BF_OCOTP_DATA_DATA(v)   (v)

#define HW_OCOTP_READ_CTRL	(0x00000030)

#define BP_OCOTP_READ_CTRL_RSVD0      1
#define BM_OCOTP_READ_CTRL_RSVD0 0xFFFFFFFE
#define BF_OCOTP_READ_CTRL_RSVD0(v) \
	(((v) << 1) & BM_OCOTP_READ_CTRL_RSVD0)
#define BM_OCOTP_READ_CTRL_READ_FUSE 0x00000001

#define HW_OCOTP_READ_FUSE_DATA	(0x00000040)

#define BP_OCOTP_READ_FUSE_DATA_DATA      0
#define BM_OCOTP_READ_FUSE_DATA_DATA 0xFFFFFFFF
#define BF_OCOTP_READ_FUSE_DATA_DATA(v)   (v)

#define HW_OCOTP_SW_STICKY	(0x00000050)

#define BP_OCOTP_SW_STICKY_RSVD0      5
#define BM_OCOTP_SW_STICKY_RSVD0 0xFFFFFFE0
#define BF_OCOTP_SW_STICKY_RSVD0(v) \
	(((v) << 5) & BM_OCOTP_SW_STICKY_RSVD0)
#define BM_OCOTP_SW_STICKY_JTAG_BLOCK_RELEASE 0x00000010
#define BM_OCOTP_SW_STICKY_BLOCK_ROM_PART 0x00000008
#define BM_OCOTP_SW_STICKY_FIELD_RETURN_LOCK 0x00000004
#define BM_OCOTP_SW_STICKY_SRK_REVOKE_LOCK 0x00000002
#define BM_OCOTP_SW_STICKY_BLOCK_DTCP_KEY 0x00000001

#define HW_OCOTP_SCS	(0x00000060)
#define HW_OCOTP_SCS_SET	(0x00000064)
#define HW_OCOTP_SCS_CLR	(0x00000068)
#define HW_OCOTP_SCS_TOG	(0x0000006c)

#define BM_OCOTP_SCS_LOCK 0x80000000
#define BP_OCOTP_SCS_SPARE      1
#define BM_OCOTP_SCS_SPARE 0x7FFFFFFE
#define BF_OCOTP_SCS_SPARE(v)  \
	(((v) << 1) & BM_OCOTP_SCS_SPARE)
#define BM_OCOTP_SCS_HAB_JDE 0x00000001

#define HW_OCOTP_CRC_ADDR	(0x00000070)

#define BP_OCOTP_CRC_ADDR_RSVD0      19
#define BM_OCOTP_CRC_ADDR_RSVD0 0xFFF80000
#define BF_OCOTP_CRC_ADDR_RSVD0(v) \
	(((v) << 19) & BM_OCOTP_CRC_ADDR_RSVD0)
#define BP_OCOTP_CRC_ADDR_CRC_ADDR      16
#define BM_OCOTP_CRC_ADDR_CRC_ADDR 0x00070000
#define BF_OCOTP_CRC_ADDR_CRC_ADDR(v)  \
	(((v) << 16) & BM_OCOTP_CRC_ADDR_CRC_ADDR)
#define BP_OCOTP_CRC_ADDR_DATA_END_ADDR      8
#define BM_OCOTP_CRC_ADDR_DATA_END_ADDR 0x0000FF00
#define BF_OCOTP_CRC_ADDR_DATA_END_ADDR(v)  \
	(((v) << 8) & BM_OCOTP_CRC_ADDR_DATA_END_ADDR)
#define BP_OCOTP_CRC_ADDR_DATA_START_ADDR      0
#define BM_OCOTP_CRC_ADDR_DATA_START_ADDR 0x000000FF
#define BF_OCOTP_CRC_ADDR_DATA_START_ADDR(v)  \
	(((v) << 0) & BM_OCOTP_CRC_ADDR_DATA_START_ADDR)

#define HW_OCOTP_CRC_VALUE	(0x00000080)

#define BP_OCOTP_CRC_VALUE_DATA      0
#define BM_OCOTP_CRC_VALUE_DATA 0xFFFFFFFF
#define BF_OCOTP_CRC_VALUE_DATA(v)   (v)

#define HW_OCOTP_VERSION	(0x00000090)

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

#define HW_OCOTP_LOCK	(0x00000400)

#define BP_OCOTP_LOCK_UNALLOCATED      30
#define BM_OCOTP_LOCK_UNALLOCATED 0xC0000000
#define BF_OCOTP_LOCK_UNALLOCATED(v) \
	(((v) << 30) & BM_OCOTP_LOCK_UNALLOCATED)
#define BP_OCOTP_LOCK_CRC_GP_HI_LOCK      28
#define BM_OCOTP_LOCK_CRC_GP_HI_LOCK 0x30000000
#define BF_OCOTP_LOCK_CRC_GP_HI_LOCK(v)  \
	(((v) << 28) & BM_OCOTP_LOCK_CRC_GP_HI_LOCK)
#define BP_OCOTP_LOCK_CRC_GP_LO_LOCK      26
#define BM_OCOTP_LOCK_CRC_GP_LO_LOCK 0x0C000000
#define BF_OCOTP_LOCK_CRC_GP_LO_LOCK(v)  \
	(((v) << 26) & BM_OCOTP_LOCK_CRC_GP_LO_LOCK)
#define BM_OCOTP_LOCK_PIN 0x02000000
#define BM_OCOTP_LOCK_RSVD2 0x01000000
#define BM_OCOTP_LOCK_DTCP_DEV_CERT 0x00800000
#define BM_OCOTP_LOCK_MISC_CONF 0x00400000
#define BM_OCOTP_LOCK_HDCP_KEYS 0x00200000
#define BM_OCOTP_LOCK_HDCP_KSV 0x00100000
#define BP_OCOTP_LOCK_ANALOG      18
#define BM_OCOTP_LOCK_ANALOG 0x000C0000
#define BF_OCOTP_LOCK_ANALOG(v)  \
	(((v) << 18) & BM_OCOTP_LOCK_ANALOG)
#define BM_OCOTP_LOCK_OTPMK 0x00020000
#define BM_OCOTP_LOCK_DTCP_KEY 0x00010000
#define BM_OCOTP_LOCK_RSVD1 0x00008000
#define BM_OCOTP_LOCK_SRK 0x00004000
#define BP_OCOTP_LOCK_GP2      12
#define BM_OCOTP_LOCK_GP2 0x00003000
#define BF_OCOTP_LOCK_GP2(v)  \
	(((v) << 12) & BM_OCOTP_LOCK_GP2)
#define BP_OCOTP_LOCK_GP1      10
#define BM_OCOTP_LOCK_GP1 0x00000C00
#define BF_OCOTP_LOCK_GP1(v)  \
	(((v) << 10) & BM_OCOTP_LOCK_GP1)
#define BP_OCOTP_LOCK_MAC_ADDR      8
#define BM_OCOTP_LOCK_MAC_ADDR 0x00000300
#define BF_OCOTP_LOCK_MAC_ADDR(v)  \
	(((v) << 8) & BM_OCOTP_LOCK_MAC_ADDR)
#define BM_OCOTP_LOCK_RSVD0 0x00000080
#define BM_OCOTP_LOCK_SJC_RESP 0x00000040
#define BP_OCOTP_LOCK_MEM_TRIM      4
#define BM_OCOTP_LOCK_MEM_TRIM 0x00000030
#define BF_OCOTP_LOCK_MEM_TRIM(v)  \
	(((v) << 4) & BM_OCOTP_LOCK_MEM_TRIM)
#define BP_OCOTP_LOCK_BOOT_CFG      2
#define BM_OCOTP_LOCK_BOOT_CFG 0x0000000C
#define BF_OCOTP_LOCK_BOOT_CFG(v)  \
	(((v) << 2) & BM_OCOTP_LOCK_BOOT_CFG)
#define BP_OCOTP_LOCK_TESTER      0
#define BM_OCOTP_LOCK_TESTER 0x00000003
#define BF_OCOTP_LOCK_TESTER(v)  \
	(((v) << 0) & BM_OCOTP_LOCK_TESTER)

/*
 *  multi-register-define name HW_OCOTP_CFGn
 *              base 0x00000410
 *              count 7
 *              offset 0x10
 */
#define HW_OCOTP_CFGn(n)	(0x00000410 + (n) * 0x10)
#define BP_OCOTP_CFGn_BITS      0
#define BM_OCOTP_CFGn_BITS 0xFFFFFFFF
#define BF_OCOTP_CFGn_BITS(v)   (v)

/*
 *  multi-register-define name HW_OCOTP_MEMn
 *              base 0x00000480
 *              count 5
 *              offset 0x10
 */
#define HW_OCOTP_MEMn(n)	(0x00000480 + (n) * 0x10)
#define BP_OCOTP_MEMn_BITS      0
#define BM_OCOTP_MEMn_BITS 0xFFFFFFFF
#define BF_OCOTP_MEMn_BITS(v)   (v)

/*
 *  multi-register-define name HW_OCOTP_ANAn
 *              base 0x000004D0
 *              count 3
 *              offset 0x10
 */
#define HW_OCOTP_ANAn(n)	(0x000004d0 + (n) * 0x10)
#define BP_OCOTP_ANAn_BITS      0
#define BM_OCOTP_ANAn_BITS 0xFFFFFFFF
#define BF_OCOTP_ANAn_BITS(v)   (v)

/*
 *  multi-register-define name HW_OCOTP_OTPMKn
 *              base 0x00000500
 *              count 8
 *              offset 0x10
 */
#define HW_OCOTP_OTPMKn(n)	(0x00000500 + (n) * 0x10)
#define BP_OCOTP_OTPMKn_BITS      0
#define BM_OCOTP_OTPMKn_BITS 0xFFFFFFFF
#define BF_OCOTP_OTPMKn_BITS(v)   (v)

/*
 *  multi-register-define name HW_OCOTP_SRKn
 *              base 0x00000580
 *              count 8
 *              offset 0x10
 */
#define HW_OCOTP_SRKn(n)	(0x00000580 + (n) * 0x10)
#define BP_OCOTP_SRKn_BITS      0
#define BM_OCOTP_SRKn_BITS 0xFFFFFFFF
#define BF_OCOTP_SRKn_BITS(v)   (v)

/*
 *  multi-register-define name HW_OCOTP_SJC_RESPn
 *              base 0x00000600
 *              count 2
 *              offset 0x10
 */
#define HW_OCOTP_SJC_RESPn(n)	(0x00000600 + (n) * 0x10)
#define BP_OCOTP_SJC_RESPn_BITS      0
#define BM_OCOTP_SJC_RESPn_BITS 0xFFFFFFFF
#define BF_OCOTP_SJC_RESPn_BITS(v)   (v)

/*
 *  multi-register-define name HW_OCOTP_MACn
 *              base 0x00000620
 *              count 2
 *              offset 0x10
 */
#define HW_OCOTP_MACn(n)	(0x00000620 + (n) * 0x10)
#define BP_OCOTP_MACn_BITS      0
#define BM_OCOTP_MACn_BITS 0xFFFFFFFF
#define BF_OCOTP_MACn_BITS(v)   (v)

/*
 *  multi-register-define name HW_OCOTP_HDCP_KSVn
 *              base 0x00000640
 *              count 2
 *              offset 0x10
 */
#define HW_OCOTP_HDCP_KSVn(n)	(0x00000640 + (n) * 0x10)
#define BP_OCOTP_HDCP_KSVn_BITS      0
#define BM_OCOTP_HDCP_KSVn_BITS 0xFFFFFFFF
#define BF_OCOTP_HDCP_KSVn_BITS(v)   (v)

#define HW_OCOTP_GP1	(0x00000660)

#define BP_OCOTP_GP1_BITS      0
#define BM_OCOTP_GP1_BITS 0xFFFFFFFF
#define BF_OCOTP_GP1_BITS(v)   (v)

#define HW_OCOTP_GP2	(0x00000670)

#define BP_OCOTP_GP2_BITS      0
#define BM_OCOTP_GP2_BITS 0xFFFFFFFF
#define BF_OCOTP_GP2_BITS(v)   (v)

/*
 *  multi-register-define name HW_OCOTP_DTCP_KEYn
 *              base 0x00000680
 *              count 5
 *              offset 0x10
 */
#define HW_OCOTP_DTCP_KEYn(n)	(0x00000680 + (n) * 0x10)
#define BP_OCOTP_DTCP_KEYn_BITS      0
#define BM_OCOTP_DTCP_KEYn_BITS 0xFFFFFFFF
#define BF_OCOTP_DTCP_KEYn_BITS(v)   (v)

#define HW_OCOTP_MISC_CONF	(0x000006d0)

#define BP_OCOTP_MISC_CONF_BITS      0
#define BM_OCOTP_MISC_CONF_BITS 0xFFFFFFFF
#define BF_OCOTP_MISC_CONF_BITS(v)   (v)

#define HW_OCOTP_FIELD_RETURN	(0x000006e0)

#define BP_OCOTP_FIELD_RETURN_BITS      0
#define BM_OCOTP_FIELD_RETURN_BITS 0xFFFFFFFF
#define BF_OCOTP_FIELD_RETURN_BITS(v)   (v)

#define HW_OCOTP_SRK_REVOKE	(0x000006f0)

#define BP_OCOTP_SRK_REVOKE_BITS      0
#define BM_OCOTP_SRK_REVOKE_BITS 0xFFFFFFFF
#define BF_OCOTP_SRK_REVOKE_BITS(v)   (v)

/*
 *  multi-register-define name HW_OCOTP_HDCP_KEYn
 *              base 0x00000800
 *              count 72
 *              offset 0x10
 */
#define HW_OCOTP_HDCP_KEYn(n)	(0x00000800 + (n) * 0x10)
#define BP_OCOTP_HDCP_KEYn_BITS      0
#define BM_OCOTP_HDCP_KEYn_BITS 0xFFFFFFFF
#define BF_OCOTP_HDCP_KEYn_BITS(v)   (v)

/*
 *  multi-register-define name HW_OCOTP_CRCn
 *              base 0x00000D00
 *              count 8
 *              offset 0x10
 */
#define HW_OCOTP_CRCn(n)	(0x00000d00 + (n) * 0x10)
#define BP_OCOTP_CRCn_BITS      0
#define BM_OCOTP_CRCn_BITS 0xFFFFFFFF
#define BF_OCOTP_CRCn_BITS(v)   (v)
#endif /* __ARCH_ARM___OCOTP_H */
