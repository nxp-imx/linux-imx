/*
 * arch/arm/mach-mx6/pcie.c
 *
 * PCIe host controller driver for IMX6 SOCs
 *
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * Bits taken from arch/arm/mach-dove/pcie.c
 *
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

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/time.h>

#include <mach/pcie.h>

#include <asm/sizes.h>
#include <asm/signal.h>

#include "crm_regs.h"
#ifdef CONFIG_PCI_MSI
#include "msi.h"
#endif

/* Register Definitions */
#define PRT_LOG_R_BaseAddress 0x700

/* Register DB_R0 */
/* Debug Register 0 */
#define DB_R0 (PRT_LOG_R_BaseAddress + 0x28)
#define DB_R0_RegisterSize 32
#define DB_R0_RegisterResetValue 0x0
#define DB_R0_RegisterResetMask 0xFFFFFFFF
/* End of Register Definition for DB_R0 */

/* Register DB_R1 */
/* Debug Register 1 */
#define DB_R1 (PRT_LOG_R_BaseAddress + 0x2c)
#define DB_R1_RegisterSize 32
#define DB_R1_RegisterResetValue 0x0
#define DB_R1_RegisterResetMask 0xFFFFFFFF
/* End of Register Definition for DB_R1 */

#define ATU_R_BaseAddress 0x900
#define ATU_VIEWPORT_R (ATU_R_BaseAddress + 0x0)
#define ATU_REGION_CTRL1_R (ATU_R_BaseAddress + 0x4)
#define ATU_REGION_CTRL2_R (ATU_R_BaseAddress + 0x8)
#define ATU_REGION_LOWBASE_R (ATU_R_BaseAddress + 0xC)
#define ATU_REGION_UPBASE_R (ATU_R_BaseAddress + 0x10)
#define ATU_REGION_LIMIT_ADDR_R (ATU_R_BaseAddress + 0x14)
#define ATU_REGION_LOW_TRGT_ADDR_R (ATU_R_BaseAddress + 0x18)
#define ATU_REGION_UP_TRGT_ADDR_R (ATU_R_BaseAddress + 0x1C)

/* Registers of PHY */
/* Register PHY_STS_R */
/* PHY Status Register */
#define PHY_STS_R (PRT_LOG_R_BaseAddress + 0x110)

/* Register PHY_CTRL_R */
/* PHY Control Register */
#define PHY_CTRL_R (PRT_LOG_R_BaseAddress + 0x114)

#define SSP_CR_SUP_DIG_MPLL_OVRD_IN_LO 0x0011
/* FIELD: RES_ACK_IN_OVRD [15:15]
// FIELD: RES_ACK_IN [14:14]
// FIELD: RES_REQ_IN_OVRD [13:13]
// FIELD: RES_REQ_IN [12:12]
// FIELD: RTUNE_REQ_OVRD [11:11]
// FIELD: RTUNE_REQ [10:10]
// FIELD: MPLL_MULTIPLIER_OVRD [9:9]
// FIELD: MPLL_MULTIPLIER [8:2]
// FIELD: MPLL_EN_OVRD [1:1]
// FIELD: MPLL_EN [0:0]
*/

#define SSP_CR_SUP_DIG_ATEOVRD 0x0010
/* FIELD: ateovrd_en [2:2]
// FIELD: ref_usb2_en [1:1]
// FIELD: ref_clkdiv2 [0:0]
*/

#define SSP_CR_LANE0_DIG_RX_OVRD_IN_LO 0x1005
/* FIELD: RX_LOS_EN_OVRD [13:13]
// FIELD: RX_LOS_EN [12:12]
// FIELD: RX_TERM_EN_OVRD [11:11]
// FIELD: RX_TERM_EN [10:10]
// FIELD: RX_BIT_SHIFT_OVRD [9:9]
// FIELD: RX_BIT_SHIFT [8:8]
// FIELD: RX_ALIGN_EN_OVRD [7:7]
// FIELD: RX_ALIGN_EN [6:6]
// FIELD: RX_DATA_EN_OVRD [5:5]
// FIELD: RX_DATA_EN [4:4]
// FIELD: RX_PLL_EN_OVRD [3:3]
// FIELD: RX_PLL_EN [2:2]
// FIELD: RX_INVERT_OVRD [1:1]
// FIELD: RX_INVERT [0:0]
*/

#define SSP_CR_LANE0_DIG_RX_ASIC_OUT 0x100D
/* FIELD: LOS [2:2]
// FIELD: PLL_STATE [1:1]
// FIELD: VALID [0:0]
*/

/* control bus bit definition */
#define PCIE_CR_CTL_DATA_LOC 0
#define PCIE_CR_CTL_CAP_ADR_LOC 16
#define PCIE_CR_CTL_CAP_DAT_LOC 17
#define PCIE_CR_CTL_WR_LOC 18
#define PCIE_CR_CTL_RD_LOC 19
#define PCIE_CR_STAT_DATA_LOC 0
#define PCIE_CR_STAT_ACK_LOC 16

#define PCIE_CAP_STRUC_BaseAddress 0x70

/* Register LNK_CAP */
/* PCIE Link cap */
#define LNK_CAP (PCIE_CAP_STRUC_BaseAddress + 0xc)
#define LNK_CAP_RegisterSize 32
#define LNK_CAP_RegisterResetValue 0x011cc12
#define LNK_CAP_RegisterResetMask 0xffffffff

#ifdef CONFIG_PCI_MSI
#define PCIE_RC_MSI_CAP   0x50

#define PCIE_PL_MSICA    0x820
#define PCIE_PL_MSICUA    0x824
#define PCIE_PL_MSIC_INT  0x828

#define MSIC_INT_EN  0x0
#define MSIC_INT_MASK  0x4
#define MSIC_INT_STATUS  0x8
#endif

/* End of Register Definitions */

#define PCIE_DBI_BASE_ADDR	(PCIE_ARB_END_ADDR - SZ_16K + 1)

#define  PCIE_CONF_BUS(b)		(((b) & 0xFF) << 16)
#define  PCIE_CONF_DEV(d)		(((d) & 0x1F) << 11)
#define  PCIE_CONF_FUNC(f)		(((f) & 0x7) << 8)
#define  PCIE_CONF_REG(r)		((r) & ~0x3)

/*
 * The default values of the RC's reserved ddr memory
 * used to verify EP mode.
 * BTW, here is the layout of the 1G ddr on SD boards
 * 0x1000_0000 ~ 0x4FFF_FFFF
 */
static u32 rc_ddr_test_region = 0x40000000;
static u32 rc_ddr_test_region_size = (SZ_16M - SZ_16K);

#ifdef EP_SELF_IO_TEST
static void *rc_ddr_test_reg1, *rc_ddr_test_reg2;
static void __iomem *pcie_arb_base_addr;
static struct timeval tv1, tv2, tv3;
static u32 tv_count1, tv_count2;
#endif
static void __iomem *base;
static void __iomem *dbi_base;

enum {
	MemRdWr = 0,
	MemRdLk = 1,
	IORdWr = 2,
	CfgRdWr0 = 4,
	CfgRdWr1 = 5
};

struct imx_pcie_port {
	u8			index;
	u8			root_bus_nr;
	void __iomem		*base;
	void __iomem		*dbi_base;
	spinlock_t		conf_lock;

	char			io_space_name[16];
	char			mem_space_name[16];

	struct resource		res[2];
};

static struct imx_pcie_port imx_pcie_port[1];
static int num_pcie_ports;

static int pcie_phy_cr_read(int addr, int *data);
static int pcie_phy_cr_write(int addr, int data);
static void change_field(int *in, int start, int end, int val);

/* IMX PCIE GPR configure routines */
static inline void imx_pcie_clrset(u32 mask, u32 val, void __iomem *addr)
{
	writel(((readl(addr) & ~mask) | (val & mask)), addr);
}

static struct imx_pcie_port *bus_to_port(int bus)
{
	int i;

	for (i = num_pcie_ports - 1; i >= 0; i--) {
		int rbus = imx_pcie_port[i].root_bus_nr;
		if (rbus != -1 && rbus == bus)
			break;
	}

	return i >= 0 ? imx_pcie_port + i : NULL;
}

static int __init imx_pcie_setup(int nr, struct pci_sys_data *sys)
{
	struct imx_pcie_port *pp;

	if (nr >= num_pcie_ports)
		return 0;

	pp = &imx_pcie_port[nr];
	pp->root_bus_nr = sys->busnr;

	/*
	 * IORESOURCE_IO
	 */
	snprintf(pp->io_space_name, sizeof(pp->io_space_name),
		 "PCIe %d I/O", pp->index);
	pp->io_space_name[sizeof(pp->io_space_name) - 1] = 0;
	pp->res[0].name = pp->io_space_name;
	if (pp->index == 0) {
		pp->res[0].start = PCIE_ARB_BASE_ADDR + SZ_16M - SZ_2M;
		pp->res[0].end = pp->res[0].start + SZ_1M - 1;
	}
	pp->res[0].flags = IORESOURCE_IO;
	if (request_resource(&ioport_resource, &pp->res[0]))
		panic("Request PCIe IO resource failed\n");
	sys->resource[0] = &pp->res[0];

	/*
	 * IORESOURCE_MEM
	 */
	snprintf(pp->mem_space_name, sizeof(pp->mem_space_name),
			"PCIe %d MEM", pp->index);
	pp->mem_space_name[sizeof(pp->mem_space_name) - 1] = 0;
	pp->res[1].name = pp->mem_space_name;
	if (pp->index == 0) {
		pp->res[1].start = PCIE_ARB_BASE_ADDR;
		pp->res[1].end = pp->res[1].start + SZ_16M - SZ_2M - 1;
	}
	pp->res[1].flags = IORESOURCE_MEM;
	if (request_resource(&iomem_resource, &pp->res[1]))
		panic("Request PCIe Memory resource failed\n");
	sys->resource[1] = &pp->res[1];

	sys->resource[2] = NULL;

	return 1;
}

static int imx_pcie_link_up(void __iomem *dbi_base)
{
	/* Check the pcie link up or link down */
	int iterations = 200;
	u32 rc, ltssm, rx_valid, temp;

	do {
		/* link is debug bit 36 debug 1 start in bit 32 */
		rc = readl(dbi_base + DB_R1) & (0x1 << (36 - 32)) ;
		iterations--;
		usleep_range(2000, 3000);

		/* From L0, initiate MAC entry to gen2 if EP/RC supports gen2.
		 * Wait 2ms (LTSSM timeout is 24ms, PHY lock is ~5us in gen2).
		 * If (MAC/LTSSM.state == Recovery.RcvrLock)
		 * && (PHY/rx_valid==0) then pulse PHY/rx_reset. Transition
		 * to gen2 is stuck
		 */
		pcie_phy_cr_read(SSP_CR_LANE0_DIG_RX_ASIC_OUT, &rx_valid);
		ltssm = readl(dbi_base + DB_R0) & 0x3F;
		if ((ltssm == 0x0D) && ((rx_valid & 0x01) == 0)) {
			pr_info("Transition to gen2 is stuck, reset PHY!\n");
			pcie_phy_cr_read(SSP_CR_LANE0_DIG_RX_OVRD_IN_LO, &temp);
			change_field(&temp, 3, 3, 0x1);
			change_field(&temp, 5, 5, 0x1);
			pcie_phy_cr_write(SSP_CR_LANE0_DIG_RX_OVRD_IN_LO,
					0x0028);
			usleep_range(2000, 3000);
			pcie_phy_cr_read(SSP_CR_LANE0_DIG_RX_OVRD_IN_LO, &temp);
			change_field(&temp, 3, 3, 0x0);
			change_field(&temp, 5, 5, 0x0);
			pcie_phy_cr_write(SSP_CR_LANE0_DIG_RX_OVRD_IN_LO,
					0x0000);
		}

		if (iterations == 0)
			pr_info("link up failed, DB_R0:0x%08x, DB_R1:0x%08x!\n"
					, readl(dbi_base + DB_R0)
					, readl(dbi_base + DB_R1));
	} while (!rc && iterations);

	if (!rc)
		return 0;
	return 1;
}

static void imx_pcie_regions_setup(struct device *dev, void __iomem *dbi_base)
{
	struct imx_pcie_platform_data *pdata = dev->platform_data;
#ifdef CONFIG_PCI_MSI
	unsigned int i;
	void __iomem *p = dbi_base + PCIE_PL_MSIC_INT;
#endif

	/*
	 * i.MX6 defines 16MB in the AXI address map for PCIe.
	 *
	 * That address space excepted the pcie registers is
	 * split and defined into different regions by iATU,
	 * with sizes and offsets as follows:
	 *
	 * RC:
	 * 0x0100_0000 --- 0x01DF_FFFF 14MB IORESOURCE_MEM
	 * 0x01E0_0000 --- 0x01EF_FFFF 1MB IORESOURCE_IO
	 * 0x01F0_0000 --- 0x01FF_FFFF 1MB Cfg + MSI + Registers
	 *
	 * EP (default value):
	 * 0x0100_0000 --- 0x01FF_C000 16MB - 16KB IORESOURCE_MEM
	 */

	/* CMD reg:I/O space, MEM space, and Bus Master Enable */
	writel(readl(dbi_base + PCI_COMMAND)
			| PCI_COMMAND_IO
			| PCI_COMMAND_MEMORY
			| PCI_COMMAND_MASTER,
			dbi_base + PCI_COMMAND);

	if (pdata->type_ep) {
		/*
		 * configure the class_rev(emaluate one memory ram ep device),
		 * bar0 and bar1 of ep
		 */
		writel(0xdeadbeaf, dbi_base + PCI_VENDOR_ID);
		writel(readl(dbi_base + PCI_CLASS_REVISION)
				| (PCI_CLASS_MEMORY_RAM	<< 16),
				dbi_base + PCI_CLASS_REVISION);
		writel(0xdeadbeaf, dbi_base + PCI_SUBSYSTEM_VENDOR_ID);

		/* 32bit none-prefetchable 8M bytes memory on bar0 */
		writel(0x0, dbi_base + PCI_BASE_ADDRESS_0);
		writel(SZ_8M - 1, dbi_base + (1 << 12) + PCI_BASE_ADDRESS_0);

		/* None used bar1 */
		writel(0x0, dbi_base + PCI_BASE_ADDRESS_1);
		writel(0, dbi_base + (1 << 12) + PCI_BASE_ADDRESS_1);

		/* 4K bytes IO on bar2 */
		writel(0x1, dbi_base + PCI_BASE_ADDRESS_2);
		writel(SZ_4K - 1, dbi_base + (1 << 12) + PCI_BASE_ADDRESS_2);

		/*
		 * 32bit prefetchable 1M bytes memory on bar3
		 * FIXME BAR MASK3 is not changable, the size
		 * is fixed to 256 bytes.
		 */
		writel(0x8, dbi_base + PCI_BASE_ADDRESS_3);
		writel(SZ_1M - 1, dbi_base + (1 << 12) + PCI_BASE_ADDRESS_3);

		/*
		 * 64bit prefetchable 1M bytes memory on bar4-5.
		 * FIXME BAR4,5 are not enabled yet
		 */
		writel(0xc, dbi_base + PCI_BASE_ADDRESS_4);
		writel(SZ_1M - 1, dbi_base + (1 << 12) + PCI_BASE_ADDRESS_4);
		writel(0, dbi_base + (1 << 12) + PCI_BASE_ADDRESS_5);

		/*
		 * region0 outbound used to access RC's reserved ddr memory
		 */
		writel(0, dbi_base + ATU_VIEWPORT_R);
		writel(PCIE_ARB_BASE_ADDR, dbi_base + ATU_REGION_LOWBASE_R);
		writel(0, dbi_base + ATU_REGION_UPBASE_R);
		writel(PCIE_ARB_BASE_ADDR + rc_ddr_test_region_size,
				dbi_base + ATU_REGION_LIMIT_ADDR_R);

		writel(rc_ddr_test_region,
				dbi_base + ATU_REGION_LOW_TRGT_ADDR_R);
		writel(0, dbi_base + ATU_REGION_UP_TRGT_ADDR_R);
		writel(MemRdWr, dbi_base + ATU_REGION_CTRL1_R);
		writel((1<<31), dbi_base + ATU_REGION_CTRL2_R);
	} else {
		/* Set the CLASS_REV of RC CFG header to PCI_CLASS_BRIDGE_PCI */
		writel(readl(dbi_base + PCI_CLASS_REVISION)
				| (PCI_CLASS_BRIDGE_PCI << 16),
				dbi_base + PCI_CLASS_REVISION);

		/*
		 * region0 outbound used to access target cfg
		 */
		writel(0, dbi_base + ATU_VIEWPORT_R);
		writel(PCIE_ARB_END_ADDR - SZ_1M + 1,
				dbi_base + ATU_REGION_LOWBASE_R);
		writel(PCIE_ARB_END_ADDR - SZ_64K,
				dbi_base + ATU_REGION_LIMIT_ADDR_R);
		writel(0, dbi_base + ATU_REGION_UPBASE_R);

		writel(0, dbi_base + ATU_REGION_LOW_TRGT_ADDR_R);
		writel(0, dbi_base + ATU_REGION_UP_TRGT_ADDR_R);
		writel(CfgRdWr0, dbi_base + ATU_REGION_CTRL1_R);
		writel((1<<31), dbi_base + ATU_REGION_CTRL2_R);
	}

#ifdef CONFIG_PCI_MSI
	writel(MSI_MATCH_ADDR, dbi_base + PCIE_PL_MSICA);
	writel(0, dbi_base + PCIE_PL_MSICUA);
	for (i = 0; i < 8 ; i++) {
		writel(0, p + MSIC_INT_EN);
		writel(0xFFFFFFFF, p + MSIC_INT_MASK);
		writel(0xFFFFFFFF, p + MSIC_INT_STATUS);
		p += 12;
	}
#endif
}

#ifdef CONFIG_PCI_MSI
void imx_pcie_mask_irq(unsigned int pos, int set)
{
	unsigned int mask = 1 << (pos & 0x1F);
	unsigned int val, newval;
	void __iomem *p;

	p = dbi_base + PCIE_PL_MSIC_INT + MSIC_INT_MASK + ((pos >> 5) * 12);

	if (pos >= (8 * 32))
		return;
	val = readl(p);
	if (set)
		newval = val | mask;
	else
		newval = val & ~mask;
	if (val != newval)
		writel(newval, p);
}

void imx_pcie_enable_irq(unsigned int pos, int set)
{
	unsigned int mask = 1 << (pos & 0x1F);
	unsigned int val, newval;
	void __iomem *p;

	p = dbi_base + PCIE_PL_MSIC_INT + MSIC_INT_EN + ((pos >> 5) * 12);

	/* RC: MSI CAP enable */
	if (set) {
		val = readl(dbi_base + PCIE_RC_MSI_CAP);
		val |= (PCI_MSI_FLAGS_ENABLE << 16);
		writel(val, dbi_base + PCIE_RC_MSI_CAP);
	}

	if (pos >= (8 * 32))
		return;
	val = readl(p);
	if (set)
		newval = val | mask;
	else
		newval = val & ~mask;
	if (val != newval)
		writel(newval, p);
	if (set && (val != newval))
		imx_pcie_mask_irq(pos, 0);  /* unmask when enabled */
	}

unsigned int imx_pcie_msi_pending(unsigned int index)
{
	unsigned int val, mask;
	void __iomem *p = dbi_base + PCIE_PL_MSIC_INT + (index * 12);

	if (index >= 8)
		return 0;
	val = readl(p + MSIC_INT_STATUS);
	mask = readl(p + MSIC_INT_MASK);
	val &= ~mask;
	writel(val, p + MSIC_INT_STATUS);
	return val;
}
#endif

static int imx_pcie_rd_conf(struct pci_bus *bus, u32 devfn, int where,
			int size, u32 *val)
{
	struct imx_pcie_port *pp = bus_to_port(bus->number);
	u32 va_address;

	/*  Added to change transaction TYPE  */
	if (bus->number < 2) {
		writel(0, dbi_base + ATU_VIEWPORT_R);
		writel(CfgRdWr0, dbi_base + ATU_REGION_CTRL1_R);
	} else {
		writel(0, dbi_base + ATU_VIEWPORT_R);
		writel(CfgRdWr1, dbi_base + ATU_REGION_CTRL1_R);
	}

	if (pp) {
		if (devfn != 0) {
			*val = 0xFFFFFFFF;
			return PCIBIOS_DEVICE_NOT_FOUND;
		}

		va_address = (u32)dbi_base + (where & ~0x3);
	} else {
		writel(0, dbi_base + ATU_VIEWPORT_R);

		writel((((PCIE_CONF_BUS(bus->number)
				+ PCIE_CONF_DEV(PCI_SLOT(devfn))
				+ PCIE_CONF_FUNC(PCI_FUNC(devfn)))) << 8),
				dbi_base + ATU_REGION_LOW_TRGT_ADDR_R);
		va_address = (u32)base + PCIE_CONF_REG(where);
	}

	*val = readl(va_address);

	if (size == 1)
		*val = (*val >> (8 * (where & 3))) & 0xFF;
	else if (size == 2)
		*val = (*val >> (8 * (where & 3))) & 0xFFFF;

	return PCIBIOS_SUCCESSFUL;
}

static int imx_pcie_wr_conf(struct pci_bus *bus, u32 devfn,
			int where, int size, u32 val)
{
	struct imx_pcie_port *pp = bus_to_port(bus->number);
	u32 va_address = 0, mask = 0, tmp = 0;
	int ret = PCIBIOS_SUCCESSFUL;

	/*  Added to change transaction TYPE  */
	if (bus->number < 2) {
		writel(0, dbi_base + ATU_VIEWPORT_R);
		writel(CfgRdWr0, dbi_base + ATU_REGION_CTRL1_R);
	} else {
		writel(0, dbi_base + ATU_VIEWPORT_R);
		writel(CfgRdWr1, dbi_base + ATU_REGION_CTRL1_R);
	}

	if (pp) {
		if (devfn != 0)
			return PCIBIOS_DEVICE_NOT_FOUND;

		va_address = (u32)dbi_base + (where & ~0x3);
	} else {
		writel(0, dbi_base + ATU_VIEWPORT_R);

		writel((((PCIE_CONF_BUS(bus->number)
				+ PCIE_CONF_DEV(PCI_SLOT(devfn))
				+ PCIE_CONF_FUNC(PCI_FUNC(devfn)))) << 8),
				dbi_base + ATU_REGION_LOW_TRGT_ADDR_R);
		va_address = (u32)base + PCIE_CONF_REG(where);
	}

	if (size == 4) {
		writel(val, va_address);
		goto exit;
	}

	if (size == 2)
		mask = ~(0xFFFF << ((where & 0x3) * 8));
	else if (size == 1)
		mask = ~(0xFF << ((where & 0x3) * 8));
	else
		ret = PCIBIOS_BAD_REGISTER_NUMBER;

	tmp = readl(va_address) & mask;
	tmp |= val << ((where & 0x3) * 8);
	writel(tmp, va_address);
exit:
	return ret;
}

static struct pci_ops imx_pcie_ops = {
	.read = imx_pcie_rd_conf,
	.write = imx_pcie_wr_conf,
};

static struct pci_bus __init *
imx_pcie_scan_bus(int nr, struct pci_sys_data *sys)
{
	struct pci_bus *bus;

	if (nr < num_pcie_ports) {
		bus = pci_scan_bus(sys->busnr, &imx_pcie_ops, sys);
	} else {
		bus = NULL;
		BUG();
	}

	return bus;
}

static int __init imx_pcie_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
       switch (pin) {
       case 1: return MXC_INT_PCIE_3;
       case 2: return MXC_INT_PCIE_2;
       case 3: return MXC_INT_PCIE_1;
       case 4: return MXC_INT_PCIE_0;
       default: return -1;
       }
}

static struct hw_pci imx_pci __initdata = {
	.nr_controllers	= 1,
	.swizzle	= pci_std_swizzle,
	.setup		= imx_pcie_setup,
	.scan		= imx_pcie_scan_bus,
	.map_irq	= imx_pcie_map_irq,
};

/* PHY CR bus acess routines */
static int pcie_phy_cr_ack_polling(int max_iterations, int exp_val)
{
	u32 temp_rd_data, wait_counter = 0;

	do {
		temp_rd_data = readl(dbi_base + PHY_STS_R);
		temp_rd_data = (temp_rd_data >> PCIE_CR_STAT_ACK_LOC) & 0x1;
		wait_counter++;
	} while ((wait_counter < max_iterations) && (temp_rd_data != exp_val));

	if (temp_rd_data != exp_val)
		return 0 ;
	return 1 ;
}

static int pcie_phy_cr_cap_addr(int addr)
{
	u32 temp_wr_data;

	/* write addr */
	temp_wr_data = addr << PCIE_CR_CTL_DATA_LOC ;
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* capture addr */
	temp_wr_data |= (0x1 << PCIE_CR_CTL_CAP_ADR_LOC);
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* wait for ack */
	if (!pcie_phy_cr_ack_polling(100, 1))
		return 0;

	/* deassert cap addr */
	temp_wr_data = addr << PCIE_CR_CTL_DATA_LOC;
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* wait for ack de-assetion */
	if (!pcie_phy_cr_ack_polling(100, 0))
		return 0 ;

	return 1 ;
}

static int pcie_phy_cr_read(int addr , int *data)
{
	u32 temp_rd_data, temp_wr_data;

	/*  write addr */
	/* cap addr */
	if (!pcie_phy_cr_cap_addr(addr))
		return 0;

	/* assert rd signal */
	temp_wr_data = 0x1 << PCIE_CR_CTL_RD_LOC;
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* wait for ack */
	if (!pcie_phy_cr_ack_polling(100, 1))
		return 0;

	/* after got ack return data */
	temp_rd_data = readl(dbi_base + PHY_STS_R);
	*data = (temp_rd_data & (0xFFFF << PCIE_CR_STAT_DATA_LOC)) ;

	/* deassert rd signal */
	temp_wr_data = 0x0;
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* wait for ack de-assetion */
	if (!pcie_phy_cr_ack_polling(100, 0))
		return 0 ;

	return 1 ;

}

static int pcie_phy_cr_write(int addr, int data)
{
	u32 temp_wr_data;

	/* write addr */
	/* cap addr */
	if (!pcie_phy_cr_cap_addr(addr))
		return 0 ;

	temp_wr_data = data << PCIE_CR_CTL_DATA_LOC;
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* capture data */
	temp_wr_data |= (0x1 << PCIE_CR_CTL_CAP_DAT_LOC);
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* wait for ack */
	if (!pcie_phy_cr_ack_polling(100, 1))
		return 0 ;

	/* deassert cap data */
	temp_wr_data = data << PCIE_CR_CTL_DATA_LOC;
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* wait for ack de-assetion */
	if (!pcie_phy_cr_ack_polling(100, 0))
		return 0;

	/* assert wr signal */
	temp_wr_data = 0x1 << PCIE_CR_CTL_WR_LOC;
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* wait for ack */
	if (!pcie_phy_cr_ack_polling(100, 1))
		return 0;

	/* deassert wr signal */
	temp_wr_data = data << PCIE_CR_CTL_DATA_LOC;
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* wait for ack de-assetion */
	if (!pcie_phy_cr_ack_polling(100, 0))
		return 0;

	temp_wr_data = 0x0 ;
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	return 1 ;
}

static void change_field(int *in, int start, int end, int val)
{
	int mask;

	mask = ((0xFFFFFFFF << start) ^ (0xFFFFFFFF << (end + 1))) & 0xFFFFFFFF;
	*in = (*in & ~mask) | (val << start);
}

static void imx_pcie_enable_controller(struct device *dev)
{
	struct clk *pcie_clk;
	struct imx_pcie_platform_data *pdata = dev->platform_data;

	/* Enable PCIE power */
	gpio_request(pdata->pcie_pwr_en, "PCIE POWER_EN");

	/* activate PCIE_PWR_EN */
	gpio_direction_output(pdata->pcie_pwr_en, 1);

	imx_pcie_clrset(IOMUXC_GPR1_TEST_POWERDOWN, 0 << 18, IOMUXC_GPR1);


	/* enable the clks */
	if (pdata->type_ep) {
		pcie_clk = clk_get(NULL, "pcie_ep_clk");
		if (IS_ERR(pcie_clk))
			pr_err("no pcie_ep clock.\n");

		if (clk_enable(pcie_clk)) {
			pr_err("can't enable pcie_ep clock.\n");
			clk_put(pcie_clk);
		}
	} else {
		pcie_clk = clk_get(NULL, "pcie_clk");
		if (IS_ERR(pcie_clk))
			pr_err("no pcie clock.\n");

		if (clk_enable(pcie_clk)) {
			pr_err("can't enable pcie clock.\n");
			clk_put(pcie_clk);
		}
	}
	imx_pcie_clrset(IOMUXC_GPR1_PCIE_REF_CLK_EN, 1 << 16, IOMUXC_GPR1);
}

static void card_reset(struct device *dev)
{
	struct imx_pcie_platform_data *pdata = dev->platform_data;

	/* PCIE RESET */
	gpio_request(pdata->pcie_rst, "PCIE RESET");

	/* activate PERST_B */
	gpio_direction_output(pdata->pcie_rst, 0);

	/* Add one reset to the pcie external device */
	msleep(100);

	/* deactive PERST_B */
	gpio_direction_output(pdata->pcie_rst, 1);
}

static void __init add_pcie_port(void __iomem *base, void __iomem *dbi_base,
		struct imx_pcie_platform_data *pdata)
{
	struct clk *pcie_clk;

	if (imx_pcie_link_up(dbi_base)) {
		struct imx_pcie_port *pp = &imx_pcie_port[num_pcie_ports++];

		pr_info("IMX PCIe port: link up.\n");

		pp->index = 0;
		pp->root_bus_nr = -1;
		pp->base = base;
		pp->dbi_base = dbi_base;
		spin_lock_init(&pp->conf_lock);
		memset(pp->res, 0, sizeof(pp->res));
	} else {
		pr_info("IMX PCIe port: link down with power supply %d!\n",
				pdata->pcie_power_always_on);
		/* Release the clocks, and disable the power */

		/* Release the clocks, and disable the power */
		pcie_clk = clk_get(NULL, "pcie_clk");
		if (IS_ERR(pcie_clk))
			pr_err("no pcie clock.\n");

		clk_disable(pcie_clk);
		clk_put(pcie_clk);

		imx_pcie_clrset(IOMUXC_GPR1_PCIE_REF_CLK_EN, 0 << 16,
				IOMUXC_GPR1);

		if (!pdata->pcie_power_always_on) {
			/* Disable PCIE power */
			gpio_request(pdata->pcie_pwr_en, "PCIE POWER_EN");

			/* activate PCIE_PWR_EN */
			gpio_direction_output(pdata->pcie_pwr_en, 0);
		}

		imx_pcie_clrset(IOMUXC_GPR1_TEST_POWERDOWN, 1 << 18,
				IOMUXC_GPR1);
	}
}

/*  Added for PCI abort handling */
static int imx6q_pcie_abort_handler(unsigned long addr,
		unsigned int fsr, struct pt_regs *regs)
{
	/*
	 * If it was an imprecise abort, then we need to correct the
	 * return address to be _after_ the instruction.
	 */
	if (fsr & (1 << 10))
		regs->ARM_pc += 4;
	return 0;
}

#ifdef CONFIG_IMX_PCIE_EP_MODE_IN_EP_RC_SYS
static ssize_t imx_pcie_rc_memw_info(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "imx-pcie-rc-memw-info start 0x%08x, size 0x%08x\n",
			rc_ddr_test_region, rc_ddr_test_region_size);
}

static ssize_t
imx_pcie_rc_memw_start(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	u32 memw_start;

	sscanf(buf, "%x\n", &memw_start);

	if (memw_start < 0x10000000) {
		dev_err(dev, "Invalid memory start address.\n");
		dev_info(dev, "For example: echo 0x41000000 > /sys/...");
		return -1;
	}

	if (rc_ddr_test_region != memw_start) {
		rc_ddr_test_region = memw_start;
		/* Re-setup the iATU */
		imx_pcie_regions_setup(dev, dbi_base);
	}

	return count;
}

static ssize_t
imx_pcie_rc_memw_size(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	u32 memw_size;

	sscanf(buf, "%x\n", &memw_size);

	if ((memw_size > (SZ_16M - SZ_16K)) || (memw_size < SZ_64K)) {
		dev_err(dev, "Invalid, should be [SZ_64K,SZ_16M - SZ_16KB].\n");
		dev_info(dev, "For example: echo 0x800000 > /sys/...");
		return -1;
	}

	if (rc_ddr_test_region_size != memw_size) {
		rc_ddr_test_region_size = memw_size;
		/* Re-setup the iATU */
		imx_pcie_regions_setup(dev, dbi_base);
	}

	return count;
}

static DEVICE_ATTR(rc_memw_info, S_IRUGO, imx_pcie_rc_memw_info, NULL);
static DEVICE_ATTR(rc_memw_start_set, S_IWUGO, NULL, imx_pcie_rc_memw_start);
static DEVICE_ATTR(rc_memw_size_set, S_IWUGO, NULL, imx_pcie_rc_memw_size);

static struct attribute *imx_pcie_attrs[] = {
	/*
	 * The start address, and the limitation (64KB ~ (16MB - 16KB))
	 * of the ddr mem window reserved by RC, and used for EP to access.
	 * BTW, these attrs are only configured at EP side.
	 */
	&dev_attr_rc_memw_info.attr,
	&dev_attr_rc_memw_start_set.attr,
	&dev_attr_rc_memw_size_set.attr,
	NULL
};

static struct attribute_group imx_pcie_attrgroup = {
	.attrs	= imx_pcie_attrs,
};
#endif

static int __devinit imx_pcie_pltfm_probe(struct platform_device *pdev)
{
#ifdef EP_SELF_IO_TEST
	int i;
#endif
	int ret = 0;
	struct resource *mem;
	struct device *dev = &pdev->dev;
	struct imx_pcie_platform_data *pdata = dev->platform_data;

	pr_info("iMX6 PCIe %s mode %s entering.\n",
			pdata->type_ep ? "PCIe EP" : "PCIe RC", __func__);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(dev, "no mmio space\n");
		return -EINVAL;
	}

#ifdef CONFIG_IMX_PCIE_EP_MODE_IN_EP_RC_SYS
	/* add attributes for device */
	ret = sysfs_create_group(&pdev->dev.kobj, &imx_pcie_attrgroup);
	if (ret)
		return -EINVAL;
#endif

	/*  Added for PCI abort handling */
	hook_fault_code(16 + 6, imx6q_pcie_abort_handler, SIGBUS, 0,
			"imprecise external abort");

	base = ioremap_nocache(PCIE_ARB_END_ADDR - SZ_1M + 1, SZ_1M - SZ_16K);
	if (!base) {
		pr_err("error with ioremap in function %s\n", __func__);
		ret = PTR_ERR(base);
#ifdef CONFIG_IMX_PCIE_EP_MODE_IN_EP_RC_SYS
		sysfs_remove_group(&pdev->dev.kobj, &imx_pcie_attrgroup);
#endif
		return ret;
	}

	dbi_base = devm_ioremap(dev, mem->start, resource_size(mem));
	if (!dbi_base) {
		dev_err(dev, "can't map %pR\n", mem);
		ret = PTR_ERR(dbi_base);
		goto err_base;
	}

	/* FIXME the field name should be aligned to RM */
	imx_pcie_clrset(IOMUXC_GPR12_APP_LTSSM_ENABLE, 0 << 10, IOMUXC_GPR12);

	/* configure constant input signal to the pcie ctrl and phy */
	if (pdata->type_ep & 1)
		/* EP */
		imx_pcie_clrset(IOMUXC_GPR12_DEVICE_TYPE,
				PCI_EXP_TYPE_ENDPOINT	<< 12, IOMUXC_GPR12);
	else
		/* RC */
		imx_pcie_clrset(IOMUXC_GPR12_DEVICE_TYPE,
				PCI_EXP_TYPE_ROOT_PORT << 12, IOMUXC_GPR12);
	imx_pcie_clrset(IOMUXC_GPR12_LOS_LEVEL, 9 << 4, IOMUXC_GPR12);

	imx_pcie_clrset(IOMUXC_GPR8_TX_DEEMPH_GEN1, 0 << 0, IOMUXC_GPR8);
	imx_pcie_clrset(IOMUXC_GPR8_TX_DEEMPH_GEN2_3P5DB, 0 << 6, IOMUXC_GPR8);
	imx_pcie_clrset(IOMUXC_GPR8_TX_DEEMPH_GEN2_6DB, 20 << 12, IOMUXC_GPR8);
	imx_pcie_clrset(IOMUXC_GPR8_TX_SWING_FULL, 127 << 18, IOMUXC_GPR8);
	imx_pcie_clrset(IOMUXC_GPR8_TX_SWING_LOW, 127 << 25, IOMUXC_GPR8);

	/* Enable the pwr, clks and so on */
	imx_pcie_enable_controller(dev);
	if (!(pdata->type_ep)) {
		/*Only RC: togle the external card's reset */
		card_reset(dev) ;

		imx_pcie_regions_setup(dev, dbi_base);
	}

	pr_info("PCIE: %s start link up.\n", __func__);
	/* start link up */
	imx_pcie_clrset(IOMUXC_GPR12_APP_LTSSM_ENABLE, 1 << 10, IOMUXC_GPR12);

	if (pdata->type_ep) {
#ifdef EP_SELF_IO_TEST
		/* Prepare the test regions and data */
		rc_ddr_test_reg1 = kzalloc(rc_ddr_test_region_size, GFP_KERNEL);
		if (!rc_ddr_test_reg1)
			pr_err("PCIe EP: can't alloc the test region1.\n");

		rc_ddr_test_reg2 = kzalloc(rc_ddr_test_region_size, GFP_KERNEL);
		if (!rc_ddr_test_reg2) {
			kfree(rc_ddr_test_reg1);
			pr_err("PCIe EP: can't alloc the test region2.\n");
		}

		pcie_arb_base_addr = ioremap_cached(PCIE_ARB_BASE_ADDR,
				rc_ddr_test_region_size);

		if (!pcie_arb_base_addr) {
			pr_err("error with ioremap in function %s\n", __func__);
			ret = PTR_ERR(pcie_arb_base_addr);
			kfree(rc_ddr_test_reg2);
			kfree(rc_ddr_test_reg1);
			goto err_base;
		}

		for (i = 0; i < rc_ddr_test_region_size; i = i + 4) {
			writel(0xE6600D00 + i, rc_ddr_test_reg1 + i);
			writel(0xDEADBEAF, rc_ddr_test_reg2 + i);
		}
#endif

		pr_info("PCIe EP: waiting for link up...\n");
		/* link is debug bit 36 debug 1 start in bit 32 */
		do {
			usleep_range(10, 20);
		} while ((readl(dbi_base + DB_R1) & 0x10) == 0);
		/* Make sure that the PCIe link is up */
		if (imx_pcie_link_up(dbi_base)) {
			pr_info("PCIe EP: link up.\n");
		} else {
			pr_info("PCIe EP: ERROR link is down, exit!\n");
#ifdef EP_SELF_IO_TEST
			kfree(rc_ddr_test_reg2);
			kfree(rc_ddr_test_reg1);
			iounmap(pcie_arb_base_addr);
#endif
			goto err_link_down;
		}

		imx_pcie_regions_setup(dev, dbi_base);
#ifdef EP_SELF_IO_TEST
		/* PCIe EP start the data transfer after link up */
		pr_info("PCIe EP: Starting data transfer...\n");
		do_gettimeofday(&tv1);

		memcpy((unsigned long *)pcie_arb_base_addr,
				(unsigned long *)rc_ddr_test_reg1, 0xFFC000);

		do_gettimeofday(&tv2);

		memcpy((unsigned long *)rc_ddr_test_reg2,
				(unsigned long *)pcie_arb_base_addr, 0xFFC000);

		do_gettimeofday(&tv3);

		if (memcmp(rc_ddr_test_reg2, rc_ddr_test_reg1, 0xFFC000) != 0) {
			pr_info("PCIe EP: Data transfer is failed.\n");
		} else {
			tv_count1 = (tv2.tv_sec - tv1.tv_sec) * USEC_PER_SEC
				+ tv2.tv_usec - tv1.tv_usec;
			tv_count2 = (tv3.tv_sec - tv2.tv_sec) * USEC_PER_SEC
				+ tv3.tv_usec - tv2.tv_usec;

			pr_info("PCIe EP: Data transfer is successful."
					"tv_count1 %dus, tv_count2 %dus.\n",
					tv_count1, tv_count2);
			pr_info("PCIe EP: Data write speed is %ldMB/s.\n",
					((((0xFFC000/1024) * MSEC_PER_SEC)))
					/(tv_count1));
			pr_info("PCIe EP: Data read speed is %ldMB/s.\n",
					((((0xFFC000/1024) * MSEC_PER_SEC)))
					/(tv_count2));
		}
#endif

	} else {
		/* add the pcie port */
		add_pcie_port(base, dbi_base, pdata);

		pci_common_init(&imx_pci);
	}
	return 0;

err_link_down:
	iounmap(dbi_base);

err_base:
	iounmap(base);

	return ret;
}

static struct platform_driver imx_pcie_pltfm_driver = {
	.driver = {
		.name	= "imx-pcie",
		.owner	= THIS_MODULE,
	},
	.probe		= imx_pcie_pltfm_probe,
};

/*****************************************************************************\
 *                                                                           *
 * Driver init/exit                                                          *
 *                                                                           *
\*****************************************************************************/

static int __init imx_pcie_drv_init(void)
{
	return platform_driver_register(&imx_pcie_pltfm_driver);
}

static void __exit imx_pcie_drv_exit(void)
{
	platform_driver_unregister(&imx_pcie_pltfm_driver);
}

module_init(imx_pcie_drv_init);
module_exit(imx_pcie_drv_exit);

MODULE_DESCRIPTION("i.MX PCIE platform driver");
MODULE_LICENSE("GPL v2");
