/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <asm/errno.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#include <mach/hardware.h>
#include <mach/ahci_sata.h>

int write_phy_ctl_ack_polling(u32 data, void __iomem *mmio,
		int max_iterations, u32 exp_val)
{
	u32 i, val;

	writel(data, mmio + PORT_PHY_CTL);

	for (i = 0; i < max_iterations + 1; i++) {
		val = readl(mmio + PORT_PHY_SR);
		val =  (val >> PORT_PHY_STAT_ACK_LOC) & 0x1;
		if (val == exp_val)
			return 0;
		if (i == max_iterations) {
			pr_err("Wait for CR ACK error!\n");
			return 1;
		}
		usleep_range(100, 200);
	}
	return 0;
}

int sata_phy_cr_addr(u32 addr, void __iomem *mmio)
{
	u32 temp_wr_data;

	/* write addr */
	temp_wr_data = addr;
	writel(temp_wr_data, mmio + PORT_PHY_CTL);

	/* capture addr */
	temp_wr_data |= PORT_PHY_CTL_CAP_ADR_LOC;

	/* wait for ack */
	if (write_phy_ctl_ack_polling(temp_wr_data, mmio, 100, 1))
		return 1;

	/* deassert cap addr */
	temp_wr_data &= 0xffff;

	/* wait for ack de-assetion */
	if (write_phy_ctl_ack_polling(temp_wr_data, mmio, 100, 0))
		return 1;

	return 0;
}

int sata_phy_cr_write(u32 data, void __iomem *mmio)
{
	u32 temp_wr_data;

	/* write data */
	temp_wr_data = data;

	/* capture data */
	temp_wr_data |= PORT_PHY_CTL_CAP_DAT_LOC;

	/* wait for ack */
	if (write_phy_ctl_ack_polling(temp_wr_data, mmio, 100, 1))
		return 1;

	/* deassert cap data */
	temp_wr_data &= 0xffff;

	/* wait for ack de-assetion */
	if (write_phy_ctl_ack_polling(temp_wr_data, mmio, 100, 0))
		return 1;

	/* assert wr signal */
	temp_wr_data |= PORT_PHY_CTL_WRITE_LOC;

	/* wait for ack */
	if (write_phy_ctl_ack_polling(temp_wr_data, mmio, 100, 1))
		return 1;

	/* deassert wr _signal */
	temp_wr_data = 0x0;

	/* wait for ack de-assetion */
	if (write_phy_ctl_ack_polling(temp_wr_data, mmio, 100, 0))
		return 1;

	return 0;
}

int sata_phy_cr_read(u32 *data, void __iomem *mmio)
{
	u32 temp_rd_data, temp_wr_data;

	/* assert rd signal */
	temp_wr_data = PORT_PHY_CTL_READ_LOC;

	/* wait for ack */
	if (write_phy_ctl_ack_polling(temp_wr_data, mmio, 100, 1))
		return 1;

	/* after got ack return data */
	temp_rd_data = readl(mmio + PORT_PHY_SR);
	*data = (temp_rd_data & 0xffff);

	/* deassert rd _signal */
	temp_wr_data = 0x0 ;

	/* wait for ack de-assetion */
	if (write_phy_ctl_ack_polling(temp_wr_data, mmio, 100, 0))
		return 1;

	return 0;
}

/* AHCI module Initialization, if return 0, initialization is successful. */
int sata_init(void __iomem *addr, unsigned long timer1ms)
{
	u32 tmpdata;
	int iterations = 20;

	/*
	 * Make sure that SATA PHY is enabled
	 * The PDDQ mode is disabled.
	 */
	tmpdata = readl(addr + PORT_PHY_CTL);
	writel(tmpdata & (~PORT_PHY_CTL_PDDQ_LOC), addr + PORT_PHY_CTL);

	/* Reset HBA */
	writel(HOST_RESET, addr + HOST_CTL);

	tmpdata = 0;
	while (readl(addr + HOST_CAP) == 0) {
		tmpdata++;
		if (tmpdata > 100000) {
			pr_err("Can't recover from RESET HBA!\n");
			break;
		}
	}

	tmpdata = readl(addr + HOST_CAP);
	if (!(tmpdata & HOST_CAP_SSS)) {
		tmpdata |= HOST_CAP_SSS;
		writel(tmpdata, addr + HOST_CAP);
	}

	if (!(readl(addr + HOST_PORTS_IMPL) & 0x1))
		writel((readl(addr + HOST_PORTS_IMPL) | 0x1),
			addr + HOST_PORTS_IMPL);

	writel(timer1ms, addr + HOST_TIMER1MS);

	/* Release resources when there is no device on the port */
	do {
		if ((readl(addr + PORT_SATA_SR) & 0xF) == 0)
			usleep_range(1000, 2000);
		else
			break;

		if (iterations == 0) {
			pr_info("No sata disk.\n");
			/* Enter into PDDQ mode, save power */
			tmpdata = readl(addr + PORT_PHY_CTL);
			writel(tmpdata | PORT_PHY_CTL_PDDQ_LOC,
					addr + PORT_PHY_CTL);
			return -ENODEV;
		}
	} while (iterations-- > 0);

	return 0;
}
