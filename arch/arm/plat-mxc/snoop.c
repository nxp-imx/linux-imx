/*
 * Copyright 2005-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <mach/hardware.h>
#include <linux/io.h>

#ifdef M4IF_BASE_ADDR
#define SNOOP_V2
#define MAX_SNOOP	2
#define g_snoop_base	(IO_ADDRESS(M4IF_BASE_ADDR) + 0x4C)
#elif defined(M3IF_BASE_ADDR)
#define MAX_SNOOP	1
#define g_snoop_base	(IO_ADDRESS(M3IF_BASE_ADDR) + 0x28)
#else
#define MAX_SNOOP	0
#define g_snoop_base	0
#endif

/* M3IF Snooping Configuration Register 0 (M3IFSCFG0) READ/WRITE*/
#define SBAR(x)		(x * 0x14)
/* M3IF Snooping Configuration Register 1 (M3IFSCFG1) READ/WRITE*/
#define SERL(x)		((x * 0x14) + 0x4)
/* M3IF Snooping Configuration Register 2 (M3IFSCFG2) READ/WRITE*/
#define SERH(x)		((x * 0x14) + 0x8)
/* M3IF Snooping Status Register 0 (M3IFSSR0) READ/WRITE */
#define SSRL(x)		((x * 0x14) + 0xC)
/* M3IF Snooping Status Register 1 (M3IFSSR1) */
#define SSRH(x)		((x * 0x14) + 0x10)

#if MAX_SNOOP

int mxc_snoop_set_config(u32 num, unsigned long base, int size)
{
	u32 reg;
	uint32_t msb;
	uint32_t seg_size;
	uint32_t window_size = 0;
	int i;

	if (num >= MAX_SNOOP) {
		return -EINVAL;
	}

	/* Setup M3IF for snooping */
	if (size) {

		if (base == 0) {
			return -EINVAL;
		}

		msb = fls(size);
		if (!(size & ((1UL << msb) - 1)))
			msb--;	/* Already aligned to power 2 */
		if (msb < 11)
			msb = 11;

		window_size = (1UL << msb);
		seg_size = window_size / 64;

		msb -= 11;

		reg = base & ~((1UL << msb) - 1);
		reg |= msb << 1;
		reg |= 1;	/* enable snooping */
		reg |= 0x80;	/* Set pulse width to default (M4IF only) */
		__raw_writel(reg, g_snoop_base + SBAR(num));

		reg = 0;
		for (i = 0; i < 32; i++) {
			if (i * seg_size >= size)
				break;
			reg |= 1UL << i;
		}
		__raw_writel(reg, g_snoop_base + SERL(num));

		reg = 0;
		for (i = 32; i < 64; i++) {
			if (i * seg_size >= size)
				break;
			reg |= 1UL << (i - 32);
		}
		__raw_writel(reg, g_snoop_base + SERH(num));

		pr_debug
		    ("Snooping unit # %d enabled: window size = 0x%X, M3IFSCFG0=0x%08X, M3IFSCFG1=0x%08X, M3IFSCFG2=0x%08X\n",
		     num, window_size, __raw_readl(g_snoop_base + SBAR(num)),
		     __raw_readl(g_snoop_base + SERL(num)),
		     __raw_readl(g_snoop_base + SERH(num)));
	} else {
		__raw_writel(0, g_snoop_base + SBAR(num));
	}

	return window_size;
}

EXPORT_SYMBOL(mxc_snoop_set_config);

int mxc_snoop_get_status(u32 num, u32 *statl, u32 *stath)
{
	if (num >= MAX_SNOOP) {
		return -EINVAL;
	}

	*statl = __raw_readl(g_snoop_base + SSRL(num));
	*stath = __raw_readl(g_snoop_base + SSRH(num));
	/* DPRINTK("status = 0x%08X%08X\n", stat[1], stat[0]); */

#ifdef SNOOP_V2
	__raw_writel(*statl, g_snoop_base + SSRL(num));
	__raw_writel(*stath, g_snoop_base + SSRH(num));
#else
	__raw_writel(0x0, g_snoop_base + SSRL(num));
	__raw_writel(0x0, g_snoop_base + SSRH(num));
#endif
	return 0;
}

EXPORT_SYMBOL(mxc_snoop_get_status);

#endif				/* MAX_SNOOP */
