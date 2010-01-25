/*
 * Freescale PWM Register Definitions
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
 * Xml Revision: 1.30
 * Template revision: 26195
 */

#ifndef __ARCH_ARM___PWM_H
#define __ARCH_ARM___PWM_H

#include <mach/mx28.h>

#define REGS_PWM_BASE IO_ADDRESS(PWM_PHYS_ADDR)
#define REGS_PWM_PHYS (0x80064000)
#define REGS_PWM_SIZE 0x00002000

#define HW_PWM_CTRL	(0x00000000)
#define HW_PWM_CTRL_SET	(0x00000004)
#define HW_PWM_CTRL_CLR	(0x00000008)
#define HW_PWM_CTRL_TOG	(0x0000000c)

#define BM_PWM_CTRL_SFTRST	0x80000000
#define BM_PWM_CTRL_CLKGATE	0x40000000
#define BM_PWM_CTRL_PWM7_PRESENT	0x20000000
#define BM_PWM_CTRL_PWM6_PRESENT	0x10000000
#define BM_PWM_CTRL_PWM5_PRESENT	0x08000000
#define BM_PWM_CTRL_PWM4_PRESENT	0x04000000
#define BM_PWM_CTRL_PWM3_PRESENT	0x02000000
#define BM_PWM_CTRL_PWM2_PRESENT	0x01000000
#define BM_PWM_CTRL_PWM1_PRESENT	0x00800000
#define BM_PWM_CTRL_PWM0_PRESENT	0x00400000
#define BP_PWM_CTRL_RSRVD1	10
#define BM_PWM_CTRL_RSRVD1	0x003FFC00
#define BF_PWM_CTRL_RSRVD1(v)  \
		(((v) << 10) & BM_PWM_CTRL_RSRVD1)
#define BM_PWM_CTRL_OUTPUT_CUTOFF_EN	0x00000200
#define BM_PWM_CTRL_RSRVD2	0x00000100
#define BM_PWM_CTRL_PWM7_ENABLE	0x00000080
#define BM_PWM_CTRL_PWM6_ENABLE	0x00000040
#define BM_PWM_CTRL_PWM5_ENABLE	0x00000020
#define BM_PWM_CTRL_PWM4_ENABLE	0x00000010
#define BM_PWM_CTRL_PWM3_ENABLE	0x00000008
#define BM_PWM_CTRL_PWM2_ENABLE	0x00000004
#define BM_PWM_CTRL_PWM1_ENABLE	0x00000002
#define BM_PWM_CTRL_PWM0_ENABLE	0x00000001

/*
 *  multi-register-define name HW_PWM_ACTIVEn
 *              base 0x00000010
 *              count 8
 *              offset 0x20
 */
#define HW_PWM_ACTIVEn(n)	(0x00000010 + (n) * 0x20)
#define HW_PWM_ACTIVEn_SET(n)	(0x00000014 + (n) * 0x20)
#define HW_PWM_ACTIVEn_CLR(n)	(0x00000018 + (n) * 0x20)
#define HW_PWM_ACTIVEn_TOG(n)	(0x0000001c + (n) * 0x20)
#define BP_PWM_ACTIVEn_INACTIVE	16
#define BM_PWM_ACTIVEn_INACTIVE	0xFFFF0000
#define BF_PWM_ACTIVEn_INACTIVE(v) \
		(((v) << 16) & BM_PWM_ACTIVEn_INACTIVE)
#define BP_PWM_ACTIVEn_ACTIVE	0
#define BM_PWM_ACTIVEn_ACTIVE	0x0000FFFF
#define BF_PWM_ACTIVEn_ACTIVE(v)  \
		(((v) << 0) & BM_PWM_ACTIVEn_ACTIVE)

/*
 *  multi-register-define name HW_PWM_PERIODn
 *              base 0x00000020
 *              count 8
 *              offset 0x20
 */
#define HW_PWM_PERIODn(n)	(0x00000020 + (n) * 0x20)
#define HW_PWM_PERIODn_SET(n)	(0x00000024 + (n) * 0x20)
#define HW_PWM_PERIODn_CLR(n)	(0x00000028 + (n) * 0x20)
#define HW_PWM_PERIODn_TOG(n)	(0x0000002c + (n) * 0x20)
#define BP_PWM_PERIODn_RSRVD2	27
#define BM_PWM_PERIODn_RSRVD2	0xF8000000
#define BF_PWM_PERIODn_RSRVD2(v) \
		(((v) << 27) & BM_PWM_PERIODn_RSRVD2)
#define BM_PWM_PERIODn_HSADC_OUT	0x04000000
#define BM_PWM_PERIODn_HSADC_CLK_SEL	0x02000000
#define BM_PWM_PERIODn_MATT_SEL	0x01000000
#define BM_PWM_PERIODn_MATT	0x00800000
#define BP_PWM_PERIODn_CDIV	20
#define BM_PWM_PERIODn_CDIV	0x00700000
#define BF_PWM_PERIODn_CDIV(v)  \
		(((v) << 20) & BM_PWM_PERIODn_CDIV)
#define BV_PWM_PERIODn_CDIV__DIV_1    0x0
#define BV_PWM_PERIODn_CDIV__DIV_2    0x1
#define BV_PWM_PERIODn_CDIV__DIV_4    0x2
#define BV_PWM_PERIODn_CDIV__DIV_8    0x3
#define BV_PWM_PERIODn_CDIV__DIV_16   0x4
#define BV_PWM_PERIODn_CDIV__DIV_64   0x5
#define BV_PWM_PERIODn_CDIV__DIV_256  0x6
#define BV_PWM_PERIODn_CDIV__DIV_1024 0x7
#define BP_PWM_PERIODn_INACTIVE_STATE	18
#define BM_PWM_PERIODn_INACTIVE_STATE	0x000C0000
#define BF_PWM_PERIODn_INACTIVE_STATE(v)  \
		(((v) << 18) & BM_PWM_PERIODn_INACTIVE_STATE)
#define BV_PWM_PERIODn_INACTIVE_STATE__HI_Z 0x0
#define BV_PWM_PERIODn_INACTIVE_STATE__0    0x2
#define BV_PWM_PERIODn_INACTIVE_STATE__1    0x3
#define BP_PWM_PERIODn_ACTIVE_STATE	16
#define BM_PWM_PERIODn_ACTIVE_STATE	0x00030000
#define BF_PWM_PERIODn_ACTIVE_STATE(v)  \
		(((v) << 16) & BM_PWM_PERIODn_ACTIVE_STATE)
#define BV_PWM_PERIODn_ACTIVE_STATE__HI_Z 0x0
#define BV_PWM_PERIODn_ACTIVE_STATE__0    0x2
#define BV_PWM_PERIODn_ACTIVE_STATE__1    0x3
#define BP_PWM_PERIODn_PERIOD	0
#define BM_PWM_PERIODn_PERIOD	0x0000FFFF
#define BF_PWM_PERIODn_PERIOD(v)  \
		(((v) << 0) & BM_PWM_PERIODn_PERIOD)

#define HW_PWM_VERSION	(0x00000110)

#define BP_PWM_VERSION_MAJOR	24
#define BM_PWM_VERSION_MAJOR	0xFF000000
#define BF_PWM_VERSION_MAJOR(v) \
		(((v) << 24) & BM_PWM_VERSION_MAJOR)
#define BP_PWM_VERSION_MINOR	16
#define BM_PWM_VERSION_MINOR	0x00FF0000
#define BF_PWM_VERSION_MINOR(v)  \
		(((v) << 16) & BM_PWM_VERSION_MINOR)
#define BP_PWM_VERSION_STEP	0
#define BM_PWM_VERSION_STEP	0x0000FFFF
#define BF_PWM_VERSION_STEP(v)  \
		(((v) << 0) & BM_PWM_VERSION_STEP)
#endif /* __ARCH_ARM___PWM_H */
