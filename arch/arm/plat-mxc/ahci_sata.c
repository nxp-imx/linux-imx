/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/io.h>
#include <linux/clk.h>
#include <linux/kernel.h>
#include <mach/ahci_sata.h>

/* AHCI module Initialization, if return 0, initialization is successful. */
int sata_init(void __iomem *addr, unsigned long timer1ms)
{
	u32 tmpdata;

	/* Reset HBA */
	writel(HOST_RESET, addr + HOST_CTL);

	tmpdata = readl(addr + HOST_VERSIONR);
	tmpdata = 0;
	while (readl(addr + HOST_VERSIONR) == 0) {
		tmpdata++;
		if (tmpdata > 100000) {
			printk(KERN_ERR "Can't recover from RESET HBA!\n");
			break;
		}
	}

	tmpdata = readl(addr + HOST_CAP);
	if (!(tmpdata & HOST_CAP_SSS)) {
		tmpdata |= HOST_CAP_SSS;
		writel(tmpdata, addr + HOST_CAP);
	}
	tmpdata = readl(addr + HOST_CAP);

	if (!(readl(addr + HOST_PORTS_IMPL) & 0x1))
		writel((readl(addr + HOST_PORTS_IMPL) | 0x1),
			addr + HOST_PORTS_IMPL);

	writel(timer1ms, addr + HOST_TIMER1MS);

	return 0;
}
