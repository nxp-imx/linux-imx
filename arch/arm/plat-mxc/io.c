/*
 *  Copyright 2007-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 * mxc custom ioremap implementation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <mach/hardware.h>
#include <linux/io.h>

void *__iomem __mxc_ioremap(unsigned long cookie, size_t size,
			    unsigned int mtype)
{
#ifdef CONFIG_ARCH_MX3
	if (cpu_is_mx3() && (mtype == MT_DEVICE) && (cookie < 0x80000000 && cookie != MX3x_L2CC_BASE_ADDR))
		mtype = MT_DEVICE_NONSHARED;
#endif
	return __arm_ioremap(cookie, size, mtype);
}

EXPORT_SYMBOL(__mxc_ioremap);

void __mxc_iounmap(void __iomem *addr)
{
	extern void __iounmap(volatile void __iomem *addr);

	__iounmap(addr);
}

EXPORT_SYMBOL(__mxc_iounmap);
