/*
 * Copyright 2004-2007, 2011-2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2008 Juergen Beisert (kernel@pengutronix.de)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */

#ifndef __ASM_ARCH_MXC_H__
#define __ASM_ARCH_MXC_H__

#include <linux/types.h>

#ifndef __ASM_ARCH_MXC_HARDWARE_H__
#error "Do not include directly."
#endif

#define MXC_CPU_MX1		1
#define MXC_CPU_MX21		21
#define MXC_CPU_MX25		25
#define MXC_CPU_MX27		27
#define MXC_CPU_MX31		31
#define MXC_CPU_MX35		35
#define MXC_CPU_MX37            37
#define MXC_CPU_MX50		50
#define MXC_CPU_MX51		51
#define MXC_CPU_MX53		53
#define MXC_CPU_MX6Q		63
#define MXC_CPU_MX6DL		61
#define MXC_CPU_MX6SL		60

#define IMX_CHIP_REVISION_1_0		0x10
#define IMX_CHIP_REVISION_1_1		0x11
#define IMX_CHIP_REVISION_1_2		0x12
#define IMX_CHIP_REVISION_1_3		0x13
#define IMX_CHIP_REVISION_2_0		0x20
#define IMX_CHIP_REVISION_2_1		0x21
#define IMX_CHIP_REVISION_2_2		0x22
#define IMX_CHIP_REVISION_2_3		0x23
#define IMX_CHIP_REVISION_3_0		0x30
#define IMX_CHIP_REVISION_3_1		0x31
#define IMX_CHIP_REVISION_3_2		0x32
#define IMX_CHIP_REVISION_3_3		0x33
#define IMX_CHIP_REVISION_UNKNOWN	0xff

#define IMX_CHIP_REVISION_1_0_STRING		"1.0"
#define IMX_CHIP_REVISION_1_1_STRING		"1.1"
#define IMX_CHIP_REVISION_1_2_STRING		"1.2"
#define IMX_CHIP_REVISION_1_3_STRING		"1.3"
#define IMX_CHIP_REVISION_2_0_STRING		"2.0"
#define IMX_CHIP_REVISION_2_1_STRING		"2.1"
#define IMX_CHIP_REVISION_2_2_STRING		"2.2"
#define IMX_CHIP_REVISION_2_3_STRING		"2.3"
#define IMX_CHIP_REVISION_3_0_STRING		"3.0"
#define IMX_CHIP_REVISION_3_1_STRING		"3.1"
#define IMX_CHIP_REVISION_3_2_STRING		"3.2"
#define IMX_CHIP_REVISION_3_3_STRING		"3.3"
#define IMX_CHIP_REVISION_UNKNOWN_STRING	"unknown"
#define IMX_BOARD_REV_1		0x000
#define IMX_BOARD_REV_2		0x100
#define IMX_BOARD_REV_3		0x200
#define IMX_BOARD_REV_4		0x300
#define IMX_BOARD_REV_5		0x400

#ifndef __ASSEMBLY__
extern unsigned int system_rev;
#define board_is_rev(rev)      (((system_rev & 0x0F00) == rev) ? 1 : 0)
#define imx_cpu_ver()		(system_rev & 0xFF)

#endif

#ifdef CONFIG_ARCH_MX5
#define board_is_mx53_loco_mc34708() (cpu_is_mx53() &&	\
	(board_is_rev(IMX_BOARD_REV_2) || board_is_rev(IMX_BOARD_REV_4)))
#define board_is_mx53_arm2() (cpu_is_mx53() && board_is_rev(IMX_BOARD_REV_2))
#define board_is_mx53_evk_a()    (cpu_is_mx53() && board_is_rev(IMX_BOARD_REV_1))
#define board_is_mx53_evk_b()    (cpu_is_mx53() && board_is_rev(IMX_BOARD_REV_3))
#define fuse_dev_is_available(int)  (1)
#endif

#ifdef CONFIG_SOC_IMX6Q
#define board_is_mx6q_arm2() (cpu_is_mx6q() && \
	board_is_rev(IMX_BOARD_REV_1))
#define board_is_mx6q_sabre_lite() (cpu_is_mx6q() && \
	board_is_rev(IMX_BOARD_REV_2))
#define board_is_mx6q_sabre_auto() (cpu_is_mx6q() && \
	board_is_rev(IMX_BOARD_REV_3))
#define board_is_mx6sl_evk() (cpu_is_mx6sl() && \
	board_is_rev(IMX_BOARD_REV_3))

#define board_is_mx6_unknown() \
	board_is_rev(IMX_BOARD_REV_1)
#define board_is_mx6_reva() \
	board_is_rev(IMX_BOARD_REV_2)
#define board_is_mx6_revb() \
	board_is_rev(IMX_BOARD_REV_3)
#define board_is_mx6_revc() \
	board_is_rev(IMX_BOARD_REV_4)
#endif

#ifndef __ASSEMBLY__
extern unsigned int __mxc_cpu_type;
#endif

#ifdef CONFIG_ARCH_MX1
# ifdef mxc_cpu_type
#  undef mxc_cpu_type
#  define mxc_cpu_type __mxc_cpu_type
# else
#  define mxc_cpu_type MXC_CPU_MX1
# endif
# define cpu_is_mx1()		(mxc_cpu_type == MXC_CPU_MX1)
#else
# define cpu_is_mx1()		(0)
#endif

#ifdef CONFIG_MACH_MX21
# ifdef mxc_cpu_type
#  undef mxc_cpu_type
#  define mxc_cpu_type __mxc_cpu_type
# else
#  define mxc_cpu_type MXC_CPU_MX21
# endif
# define cpu_is_mx21()		(mxc_cpu_type == MXC_CPU_MX21)
#else
# define cpu_is_mx21()		(0)
#endif

#ifdef CONFIG_ARCH_MX25
# ifdef mxc_cpu_type
#  undef mxc_cpu_type
#  define mxc_cpu_type __mxc_cpu_type
# else
#  define mxc_cpu_type MXC_CPU_MX25
# endif
# define cpu_is_mx25()		(mxc_cpu_type == MXC_CPU_MX25)
#else
# define cpu_is_mx25()		(0)
#endif

#ifdef CONFIG_MACH_MX27
# ifdef mxc_cpu_type
#  undef mxc_cpu_type
#  define mxc_cpu_type __mxc_cpu_type
# else
#  define mxc_cpu_type MXC_CPU_MX27
# endif
# define cpu_is_mx27()		(mxc_cpu_type == MXC_CPU_MX27)
#else
# define cpu_is_mx27()		(0)
#endif

#ifdef CONFIG_SOC_IMX31
# ifdef mxc_cpu_type
#  undef mxc_cpu_type
#  define mxc_cpu_type __mxc_cpu_type
# else
#  define mxc_cpu_type MXC_CPU_MX31
# endif
# define cpu_is_mx31()		(mxc_cpu_type == MXC_CPU_MX31)
#else
# define cpu_is_mx31()		(0)
#endif

# define cpu_is_mx32()		(0)

#ifdef CONFIG_SOC_IMX35
# ifdef mxc_cpu_type
#  undef mxc_cpu_type
#  define mxc_cpu_type __mxc_cpu_type
# else
#  define mxc_cpu_type MXC_CPU_MX35
# endif
# define cpu_is_mx35()		(mxc_cpu_type == MXC_CPU_MX35)
#else
# define cpu_is_mx35()		(0)
#endif

#ifdef CONFIG_SOC_IMX37
# ifdef mxc_cpu_type
#  undef mxc_cpu_type
#  define mxc_cpu_type __mxc_cpu_type
# else
#  define mxc_cpu_type MXC_CPU_MX37
# endif
# define cpu_is_mx37()          (mxc_cpu_type == MXC_CPU_MX37)
#else
# define cpu_is_mx37()          (0)
#endif

#ifdef CONFIG_SOC_IMX50
# ifdef mxc_cpu_type
#  undef mxc_cpu_type
#  define mxc_cpu_type __mxc_cpu_type
# else
#  define mxc_cpu_type MXC_CPU_MX50
# endif
# define cpu_is_mx50()		(mxc_cpu_type == MXC_CPU_MX50)
#else
# define cpu_is_mx50()		(0)
#endif

#ifdef CONFIG_SOC_IMX51
# ifdef mxc_cpu_type
#  undef mxc_cpu_type
#  define mxc_cpu_type __mxc_cpu_type
# else
#  define mxc_cpu_type MXC_CPU_MX51
# endif
# define cpu_is_mx51()		(mxc_cpu_type == MXC_CPU_MX51)
#else
# define cpu_is_mx51()		(0)
#endif

#ifdef CONFIG_SOC_IMX53
# ifdef mxc_cpu_type
#  undef mxc_cpu_type
#  define mxc_cpu_type __mxc_cpu_type
# else
#  define mxc_cpu_type MXC_CPU_MX53
# endif
# define cpu_is_mx53()		(mxc_cpu_type == MXC_CPU_MX53)
#else
# define cpu_is_mx53()		(0)
#endif

#ifdef CONFIG_SOC_IMX6Q
#  define mxc_cpu_type __mxc_cpu_type
# define cpu_is_mx6q()		(mxc_cpu_type == MXC_CPU_MX6Q)
# define cpu_is_mx6dl()		(mxc_cpu_type == MXC_CPU_MX6DL)
# define cpu_is_mx6sl()		(mxc_cpu_type == MXC_CPU_MX6SL)
#else
# define cpu_is_mx6q()		(0)
# define cpu_is_mx6dl()		(0)
# define cpu_is_mx6sl()		(0)
# define mx6q_revision(void)	(0)
# define mx6dl_revision(void)	(0)
#endif

#ifndef __ASSEMBLY__

struct cpu_op {
	u32 pll_reg;
	u32 pll_rate;
	u32 pll_lpm_rate;
	u32 cpu_rate;
	u32 pdr0_reg;
	u32 pdf;
	u32 mfi;
	u32 mfd;
	u32 mfn;
	u32 pu_voltage;
	u32 soc_voltage;
	u32 cpu_voltage;
	u32 cpu_podf;
};

enum mxc_cpu_pwr_mode {
	WAIT_CLOCKED,		/* wfi only */
	WAIT_UNCLOCKED,		/* WAIT */
	WAIT_UNCLOCKED_POWER_OFF,	/* WAIT + SRPG */
	STOP_XTAL_ON,		/* STOP + SRPG + XTAL_ON*/
	STOP_POWER_OFF,		/* STOP + XTAL_OFF */
	ARM_POWER_OFF,		/* STOP + SRPG + ARM power off */
};

int tzic_enable_wake(int is_idle);

extern void mxc_cpu_lp_set(enum mxc_cpu_pwr_mode mode);
extern int tzic_enable_wake(int is_idle);

/* available disableable devices in fuse */
enum mxc_dev_type {
	MXC_DEV_PXP,
	MXC_DEV_OVG,
	MXC_DEV_DSI_CSI2,
	MXC_DEV_ENET,
	MXC_DEV_MLB,
	MXC_DEV_EPDC,
	MXC_DEV_HDMI,
	MXC_DEV_PCIE,
	MXC_DEV_SATA,
	MXC_DEV_DTCP,
	MXC_DEV_2D,
	MXC_DEV_3D,
	MXC_DEV_VPU,
	MXC_DEV_DIVX3,
	MXC_DEV_RV,
	MXC_DEV_SORENSEN,
};
#ifdef CONFIG_ARCH_MX6
extern int fuse_dev_is_available(enum mxc_dev_type dev);
#endif

#endif

#if defined(CONFIG_ARCH_MX3) || defined(CONFIG_ARCH_MX2)
/* These are deprecated, use mx[23][157]_setup_weimcs instead. */
#define CSCR_U(n) (IO_ADDRESS(WEIM_BASE_ADDR + n * 0x10))
#define CSCR_L(n) (IO_ADDRESS(WEIM_BASE_ADDR + n * 0x10 + 0x4))
#define CSCR_A(n) (IO_ADDRESS(WEIM_BASE_ADDR + n * 0x10 + 0x8))
#endif

#define cpu_is_mx5()    (cpu_is_mx51() || cpu_is_mx53() || cpu_is_mx50())
#define cpu_is_mx3()	(cpu_is_mx31() || cpu_is_mx35())
#define cpu_is_mx2()	(cpu_is_mx21() || cpu_is_mx27())
#define cpu_is_mx6()	(cpu_is_mx6q() || cpu_is_mx6dl() || cpu_is_mx6sl())

#define MXC_PGCR_PCR		1
#define MXC_SRPGCR_PCR		1
#define MXC_EMPGCR_PCR		1
#define MXC_PGSR_PSR		1
#endif /*  __ASM_ARCH_MXC_H__ */
