/*
 * Copyright 2004-2010 Freescale Semiconductor, Inc.
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

#ifndef __ASM_ARCH_MXC_HARDWARE_H__
#error "Do not include directly."
#endif

#define MXC_CPU_MX1		1
#define MXC_CPU_MX21		21
#define MXC_CPU_MX25		25
#define MXC_CPU_MX27		27
#define MXC_CPU_MX31		31
#define MXC_CPU_MX32		32
#define MXC_CPU_MX35		35
#define MXC_CPU_MX37		37
#define MXC_CPU_MX50		50
#define MXC_CPU_MX51		51
#define MXC_CPU_MX53		53
#define MXC_CPU_MXC91231	91231

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

#ifdef CONFIG_ARCH_MX31
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

#ifdef CONFIG_ARCH_MX35
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

#ifdef CONFIG_ARCH_MX37
# ifdef mxc_cpu_type
#  undef mxc_cpu_type
#  define mxc_cpu_type __mxc_cpu_type
# else
#  define mxc_cpu_type MXC_CPU_MX37
# endif
# define cpu_is_mx37()		(mxc_cpu_type == MXC_CPU_MX37)
#else
# define cpu_is_mx37()		(0)
#endif

#ifdef CONFIG_ARCH_MX51
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

#ifdef CONFIG_ARCH_MX53
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

#ifdef CONFIG_ARCH_MXC91231
# ifdef mxc_cpu_type
#  undef mxc_cpu_type
#  define mxc_cpu_type __mxc_cpu_type
# else
#  define mxc_cpu_type MXC_CPU_MXC91231
# endif
# define cpu_is_mxc91231()	(mxc_cpu_type == MXC_CPU_MXC91231)
#else
# define cpu_is_mxc91231()	(0)
#endif

#ifdef CONFIG_ARCH_MX50
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

#define cpu_is_mx32()		(0)

/*
 * Create inline functions to test for cpu revision
 * Function name is cpu_is_<cpu name>_rev(rev)
 *
 * Returns:
 *	 0 - not the cpu queried
 *	 1 - cpu and revision match
 *	 2 - cpu matches, but cpu revision is greater than queried rev
 *	-1 - cpu matches, but cpu revision is less than queried rev
 */
#ifndef __ASSEMBLY__
extern unsigned int system_rev;
#define mxc_set_system_rev(part, rev) ({	\
	system_rev = (part << 12) | rev;	\
})
#define mxc_cpu()		(system_rev >> 12)
#define mxc_cpu_rev()		(system_rev & 0xFF)
#define mxc_cpu_rev_major()	((system_rev >> 4) & 0xF)
#define mxc_cpu_rev_minor()	(system_rev & 0xF)
#define mxc_cpu_is_rev(rev)	\
	((mxc_cpu_rev() == rev) ? 1 : ((mxc_cpu_rev() < rev) ? -1 : 2))
#define cpu_rev(type, rev) (cpu_is_##type() ? mxc_cpu_is_rev(rev) : 0)

#define cpu_is_mx21_rev(rev) cpu_rev(mx21, rev)
#define cpu_is_mx25_rev(rev) cpu_rev(mx25, rev)
#define cpu_is_mx27_rev(rev) cpu_rev(mx27, rev)
#define cpu_is_mx31_rev(rev) cpu_rev(mx31, rev)
#define cpu_is_mx35_rev(rev) cpu_rev(mx35, rev)
#define cpu_is_mx37_rev(rev) cpu_rev(mx37, rev)
#define cpu_is_mx50_rev(rev) cpu_rev(mx50, rev)
#define cpu_is_mx51_rev(rev) cpu_rev(mx51, rev)
#define cpu_is_mx53_rev(rev) cpu_rev(mx53, rev)


#include <linux/types.h>

extern void mxc_wd_reset(void);

int mxc_snoop_set_config(u32 num, unsigned long base, int size);
int mxc_snoop_get_status(u32 num, u32 *statl, u32 *stath);

struct platform_device;
void mxc_pg_enable(struct platform_device *pdev);
void mxc_pg_disable(struct platform_device *pdev);

struct mxc_unifi_platform_data *get_unifi_plat_data(void);

struct fsl_otp_data {
	char 		**fuse_name;
	char		*regulator_name;
	unsigned int 	fuse_num;
};

struct mxs_dma_plat_data {
	unsigned int burst8:1;
	unsigned int burst:1;
	unsigned int chan_base;
	unsigned int chan_num;
};
#endif				/* __ASSEMBLY__ */

/* DMA driver defines */
#define MXC_IDE_DMA_WATERMARK	32	/* DMA watermark level in bytes */
#define MXC_IDE_DMA_BD_NR	(512/3/4)	/* Number of BDs per channel */

/*!
 * DPTC GP and LP ID
 */
#define DPTC_GP_ID 0
#define DPTC_LP_ID 1

#define MUX_IO_P		29
#define MUX_IO_I		24

#if defined(CONFIG_ARCH_MX5) || defined(CONFIG_ARCH_MX25)
#define IOMUX_TO_GPIO(pin) 	((((unsigned int)pin >> MUX_IO_P) * 32) + ((pin >> MUX_IO_I) & ((1 << (MUX_IO_P - MUX_IO_I)) - 1)))
#define IOMUX_TO_IRQ(pin)	(MXC_GPIO_IRQ_START + IOMUX_TO_GPIO(pin))
#endif

#ifndef __ASSEMBLY__

struct cpu_wp {
	u32 pll_reg;
	u32 pll_rate;
	u32 cpu_rate;
	u32 pdr0_reg;
	u32 pdf;
	u32 mfi;
	u32 mfd;
	u32 mfn;
	u32 cpu_voltage;
	u32 cpu_podf;
};

#ifndef CONFIG_ARCH_MX5
struct cpu_wp *get_cpu_wp(int *wp);
#endif

enum mxc_cpu_pwr_mode {
	WAIT_CLOCKED,		/* wfi only */
	WAIT_UNCLOCKED,		/* WAIT */
	WAIT_UNCLOCKED_POWER_OFF,	/* WAIT + SRPG */
	STOP_POWER_ON,		/* just STOP */
	STOP_POWER_OFF,		/* STOP + SRPG */
};

void mxc_cpu_lp_set(enum mxc_cpu_pwr_mode mode);
int tzic_enable_wake(int is_idle);
void gpio_activate_audio_ports(void);
void gpio_inactivate_audio_ports(void);
void gpio_activate_bt_audio_port(void);
void gpio_inactivate_bt_audio_port(void);
void gpio_activate_esai_ports(void);
void gpio_deactivate_esai_ports(void);

#endif

#if defined(CONFIG_ARCH_MX3) || defined(CONFIG_ARCH_MX2)
/* These are deprecated, use mx[23][157]_setup_weimcs instead. */
#define CSCR_U(n) (IO_ADDRESS(WEIM_BASE_ADDR + n * 0x10))
#define CSCR_L(n) (IO_ADDRESS(WEIM_BASE_ADDR + n * 0x10 + 0x4))
#define CSCR_A(n) (IO_ADDRESS(WEIM_BASE_ADDR + n * 0x10 + 0x8))
#endif

#define cpu_is_mx5()	(cpu_is_mx51() || cpu_is_mx53() || cpu_is_mx50())
#define cpu_is_mx3()	(cpu_is_mx31() || cpu_is_mx35() || cpu_is_mxc91231())
#define cpu_is_mx2()	(cpu_is_mx21() || cpu_is_mx27())

#endif /*  __ASM_ARCH_MXC_H__ */
