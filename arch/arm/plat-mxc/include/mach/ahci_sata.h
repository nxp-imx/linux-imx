/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __PLAT_MXC_AHCI_SATA_H__
#define __PLAT_MXC_AHCI_SATA_H__

enum {
	HOST_CAP = 0x00,
	HOST_CAP_SSS = (1 << 27), /* Staggered Spin-up */
	HOST_CTL		= 0x04, /* global host control */
	HOST_RESET		= (1 << 0),  /* reset controller; self-clear */
	HOST_PORTS_IMPL	= 0x0c,
	HOST_TIMER1MS = 0xe0, /* Timer 1-ms */
	HOST_VERSIONR = 0xfc, /* host version register*/

	/* Offest used to control the MPLL input clk */
	PHY_CR_CLOCK_FREQ_OVRD = 0x12,
	/* Port0 SATA Status */
	PORT_SATA_SR = 0x128,
	/* Port0 PHY Control */
	PORT_PHY_CTL = 0x178,
	/* PORT_PHY_CTL bits */
	PORT_PHY_CTL_CAP_ADR_LOC = 0x10000,
	PORT_PHY_CTL_CAP_DAT_LOC = 0x20000,
	PORT_PHY_CTL_WRITE_LOC = 0x40000,
	PORT_PHY_CTL_READ_LOC = 0x80000,
	PORT_PHY_CTL_PDDQ_LOC = 0x100000,
	/* Port0 PHY Status */
	PORT_PHY_SR = 0x17c,
	/* PORT_PHY_SR */
	PORT_PHY_STAT_DATA_LOC = 0,
	PORT_PHY_STAT_ACK_LOC = 18,
	/* SATA PHY Register */
	SATA_PHY_CR_CLOCK_CRCMP_LT_LIMIT = 0x0001,
	SATA_PHY_CR_CLOCK_DAC_CTL = 0x0008,
	SATA_PHY_CR_CLOCK_RTUNE_CTL = 0x0009,
	SATA_PHY_CR_CLOCK_ADC_OUT = 0x000A,
	SATA_PHY_CR_CLOCK_MPLL_TST = 0x0017,
};

extern int write_phy_ctl_ack_polling(u32 data, void __iomem *mmio,
		int max_iterations, u32 exp_val);
extern int sata_phy_cr_addr(u32 addr, void __iomem *mmio);
extern int sata_phy_cr_write(u32 data, void __iomem *mmio);
extern int sata_phy_cr_read(u32 *data, void __iomem *mmio);
extern int sata_init(void __iomem *addr, unsigned long timer1ms);
#endif /* __PLAT_MXC_AHCI_SATA_H__ */
