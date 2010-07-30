/*
 * Freescale OCOTP Register Definitions
 *
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * Xml Revision: 1.21
 * Template revision: 26195
 */

#ifndef __ARCH_ARM___OCOTP_H
#define __ARCH_ARM___OCOTP_H


#define HW_OCOTP_CTRL	(0x00000000)
#define HW_OCOTP_CTRL_SET	(0x00000004)
#define HW_OCOTP_CTRL_CLR	(0x00000008)
#define HW_OCOTP_CTRL_TOG	(0x0000000c)

#define BP_OCOTP_CTRL_WR_UNLOCK	16
#define BM_OCOTP_CTRL_WR_UNLOCK	0xFFFF0000
#define BF_OCOTP_CTRL_WR_UNLOCK(v) \
		(((v) << 16) & BM_OCOTP_CTRL_WR_UNLOCK)
#define BV_OCOTP_CTRL_WR_UNLOCK__KEY 0x3E77
#define BP_OCOTP_CTRL_RSRVD2	14
#define BM_OCOTP_CTRL_RSRVD2	0x0000C000
#define BF_OCOTP_CTRL_RSRVD2(v)  \
		(((v) << 14) & BM_OCOTP_CTRL_RSRVD2)
#define BM_OCOTP_CTRL_RELOAD_SHADOWS	0x00002000
#define BM_OCOTP_CTRL_RD_BANK_OPEN	0x00001000
#define BP_OCOTP_CTRL_RSRVD1	10
#define BM_OCOTP_CTRL_RSRVD1	0x00000C00
#define BF_OCOTP_CTRL_RSRVD1(v)  \
		(((v) << 10) & BM_OCOTP_CTRL_RSRVD1)
#define BM_OCOTP_CTRL_ERROR	0x00000200
#define BM_OCOTP_CTRL_BUSY	0x00000100
#define BP_OCOTP_CTRL_RSRVD0	6
#define BM_OCOTP_CTRL_RSRVD0	0x000000C0
#define BF_OCOTP_CTRL_RSRVD0(v)  \
		(((v) << 6) & BM_OCOTP_CTRL_RSRVD0)
#define BP_OCOTP_CTRL_ADDR	0
#define BM_OCOTP_CTRL_ADDR	0x0000003F
#define BF_OCOTP_CTRL_ADDR(v)  \
		(((v) << 0) & BM_OCOTP_CTRL_ADDR)

#define HW_OCOTP_DATA	(0x00000010)

#define BP_OCOTP_DATA_DATA	0
#define BM_OCOTP_DATA_DATA	0xFFFFFFFF
#define BF_OCOTP_DATA_DATA(v)	(v)

/*
 *  multi-register-define name HW_OCOTP_CUSTn
 *              base 0x00000020
 *              count 4
 *              offset 0x10
 */
#define HW_OCOTP_CUSTn(n)	(0x00000020 + (n) * 0x10)
#define BP_OCOTP_CUSTn_BITS	0
#define BM_OCOTP_CUSTn_BITS	0xFFFFFFFF
#define BF_OCOTP_CUSTn_BITS(v)	(v)

/*
 *  multi-register-define name HW_OCOTP_CRYPTOn
 *              base 0x00000060
 *              count 4
 *              offset 0x10
 */
#define HW_OCOTP_CRYPTOn(n)	(0x00000060 + (n) * 0x10)
#define BP_OCOTP_CRYPTOn_BITS	0
#define BM_OCOTP_CRYPTOn_BITS	0xFFFFFFFF
#define BF_OCOTP_CRYPTOn_BITS(v)	(v)

/*
 *  multi-register-define name HW_OCOTP_HWCAPn
 *              base 0x000000A0
 *              count 6
 *              offset 0x10
 */
#define HW_OCOTP_HWCAPn(n)	(0x000000a0 + (n) * 0x10)
#define BP_OCOTP_HWCAPn_BITS	0
#define BM_OCOTP_HWCAPn_BITS	0xFFFFFFFF
#define BF_OCOTP_HWCAPn_BITS(v)	(v)

#define HW_OCOTP_SWCAP	(0x00000100)

#define BP_OCOTP_SWCAP_BITS	0
#define BM_OCOTP_SWCAP_BITS	0xFFFFFFFF
#define BF_OCOTP_SWCAP_BITS(v)	(v)

#define HW_OCOTP_CUSTCAP	(0x00000110)

#define BP_OCOTP_CUSTCAP_RSRVD1	3
#define BM_OCOTP_CUSTCAP_RSRVD1	0xFFFFFFF8
#define BF_OCOTP_CUSTCAP_RSRVD1(v) \
		(((v) << 3) & BM_OCOTP_CUSTCAP_RSRVD1)
#define BM_OCOTP_CUSTCAP_RTC_XTAL_32768_PRESENT	0x00000004
#define BM_OCOTP_CUSTCAP_RTC_XTAL_32000_PRESENT	0x00000002
#define BM_OCOTP_CUSTCAP_RSRVD0	0x00000001

#define HW_OCOTP_LOCK	(0x00000120)

#define BM_OCOTP_LOCK_ROM7	0x80000000
#define BM_OCOTP_LOCK_ROM6	0x40000000
#define BM_OCOTP_LOCK_ROM5	0x20000000
#define BM_OCOTP_LOCK_ROM4	0x10000000
#define BM_OCOTP_LOCK_ROM3	0x08000000
#define BM_OCOTP_LOCK_ROM2	0x04000000
#define BM_OCOTP_LOCK_ROM1	0x02000000
#define BM_OCOTP_LOCK_ROM0	0x01000000
#define BM_OCOTP_LOCK_HWSW_SHADOW_ALT	0x00800000
#define BM_OCOTP_LOCK_CRYPTODCP_ALT	0x00400000
#define BM_OCOTP_LOCK_CRYPTOKEY_ALT	0x00200000
#define BM_OCOTP_LOCK_PIN	0x00100000
#define BM_OCOTP_LOCK_OPS	0x00080000
#define BM_OCOTP_LOCK_UN2	0x00040000
#define BM_OCOTP_LOCK_UN1	0x00020000
#define BM_OCOTP_LOCK_UN0	0x00010000
#define BM_OCOTP_LOCK_SRK	0x00008000
#define BP_OCOTP_LOCK_UNALLOCATED	12
#define BM_OCOTP_LOCK_UNALLOCATED	0x00007000
#define BF_OCOTP_LOCK_UNALLOCATED(v)  \
		(((v) << 12) & BM_OCOTP_LOCK_UNALLOCATED)
#define BM_OCOTP_LOCK_SRK_SHADOW	0x00000800
#define BM_OCOTP_LOCK_ROM_SHADOW	0x00000400
#define BM_OCOTP_LOCK_CUSTCAP	0x00000200
#define BM_OCOTP_LOCK_HWSW	0x00000100
#define BM_OCOTP_LOCK_CUSTCAP_SHADOW	0x00000080
#define BM_OCOTP_LOCK_HWSW_SHADOW	0x00000040
#define BM_OCOTP_LOCK_CRYPTODCP	0x00000020
#define BM_OCOTP_LOCK_CRYPTOKEY	0x00000010
#define BM_OCOTP_LOCK_CUST3	0x00000008
#define BM_OCOTP_LOCK_CUST2	0x00000004
#define BM_OCOTP_LOCK_CUST1	0x00000002
#define BM_OCOTP_LOCK_CUST0	0x00000001

/*
 *  multi-register-define name HW_OCOTP_OPSn
 *              base 0x00000130
 *              count 4
 *              offset 0x10
 */
#define HW_OCOTP_OPSn(n)	(0x00000130 + (n) * 0x10)
#define BP_OCOTP_OPSn_BITS	0
#define BM_OCOTP_OPSn_BITS	0xFFFFFFFF
#define BF_OCOTP_OPSn_BITS(v)	(v)

/*
 *  multi-register-define name HW_OCOTP_UNn
 *              base 0x00000170
 *              count 3
 *              offset 0x10
 */
#define HW_OCOTP_UNn(n)	(0x00000170 + (n) * 0x10)
#define BP_OCOTP_UNn_BITS	0
#define BM_OCOTP_UNn_BITS	0xFFFFFFFF
#define BF_OCOTP_UNn_BITS(v)	(v)

/*
 *  multi-register-define name HW_OCOTP_ROMn
 *              base 0x000001A0
 *              count 8
 *              offset 0x10
 */
#define HW_OCOTP_ROMn(n)	(0x000001a0 + (n) * 0x10)
#define BP_OCOTP_ROMn_BOOT_MODE	24
#define BM_OCOTP_ROMn_BOOT_MODE	0xFF000000
#define BF_OCOTP_ROMn_BOOT_MODE(v) \
		(((v) << 24) & BM_OCOTP_ROMn_BOOT_MODE)
#define BP_OCOTP_ROMn_SD_MMC_MODE	22
#define BM_OCOTP_ROMn_SD_MMC_MODE	0x00C00000
#define BF_OCOTP_ROMn_SD_MMC_MODE(v)  \
		(((v) << 22) & BM_OCOTP_ROMn_SD_MMC_MODE)
#define BP_OCOTP_ROMn_SD_POWER_GATE_GPIO	20
#define BM_OCOTP_ROMn_SD_POWER_GATE_GPIO	0x00300000
#define BF_OCOTP_ROMn_SD_POWER_GATE_GPIO(v)  \
		(((v) << 20) & BM_OCOTP_ROMn_SD_POWER_GATE_GPIO)
#define BP_OCOTP_ROMn_SD_POWER_UP_DELAY	14
#define BM_OCOTP_ROMn_SD_POWER_UP_DELAY	0x000FC000
#define BF_OCOTP_ROMn_SD_POWER_UP_DELAY(v)  \
		(((v) << 14) & BM_OCOTP_ROMn_SD_POWER_UP_DELAY)
#define BP_OCOTP_ROMn_SD_BUS_WIDTH	12
#define BM_OCOTP_ROMn_SD_BUS_WIDTH	0x00003000
#define BF_OCOTP_ROMn_SD_BUS_WIDTH(v)  \
		(((v) << 12) & BM_OCOTP_ROMn_SD_BUS_WIDTH)
#define BP_OCOTP_ROMn_SSP_SCK_INDEX	8
#define BM_OCOTP_ROMn_SSP_SCK_INDEX	0x00000F00
#define BF_OCOTP_ROMn_SSP_SCK_INDEX(v)  \
		(((v) << 8) & BM_OCOTP_ROMn_SSP_SCK_INDEX)
#define BM_OCOTP_ROMn_EMMC_USE_DDR	0x00000080
#define BM_OCOTP_ROMn_DISABLE_SPI_NOR_FAST_READ	0x00000040
#define BM_OCOTP_ROMn_ENABLE_USB_BOOT_SERIAL_NUM	0x00000020
#define BM_OCOTP_ROMn_ENABLE_UNENCRYPTED_BOOT	0x00000010
#define BM_OCOTP_ROMn_SD_MBR_BOOT	0x00000008
#define BM_OCOTP_ROMn_RSRVD2	0x00000004
#define BM_OCOTP_ROMn_RSRVD1	0x00000002
#define BM_OCOTP_ROMn_RSRVD0	0x00000001

/*
 *  multi-register-define name HW_OCOTP_SRKn
 *              base 0x00000220
 *              count 8
 *              offset 0x10
 */
#define HW_OCOTP_SRKn(n)	(0x00000220 + (n) * 0x10)
#define BP_OCOTP_SRKn_BITS	0
#define BM_OCOTP_SRKn_BITS	0xFFFFFFFF
#define BF_OCOTP_SRKn_BITS(v)	(v)

#define HW_OCOTP_VERSION	(0x000002a0)

#define BP_OCOTP_VERSION_MAJOR	24
#define BM_OCOTP_VERSION_MAJOR	0xFF000000
#define BF_OCOTP_VERSION_MAJOR(v) \
		(((v) << 24) & BM_OCOTP_VERSION_MAJOR)
#define BP_OCOTP_VERSION_MINOR	16
#define BM_OCOTP_VERSION_MINOR	0x00FF0000
#define BF_OCOTP_VERSION_MINOR(v)  \
		(((v) << 16) & BM_OCOTP_VERSION_MINOR)
#define BP_OCOTP_VERSION_STEP	0
#define BM_OCOTP_VERSION_STEP	0x0000FFFF
#define BF_OCOTP_VERSION_STEP(v)  \
		(((v) << 0) & BM_OCOTP_VERSION_STEP)
#endif /* __ARCH_ARM___OCOTP_H */
