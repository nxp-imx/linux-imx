// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host controller driver for Freescale i.MX6 SoCs
 *
 * Copyright (C) 2013 Kosagi
 *		http://www.kosagi.com
 *
 * Author: Sean Cross <xobs@kosagi.com>
 */

#include <dt-bindings/soc/imx8_hsio.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/mfd/syscon/imx7-iomuxc-gpr.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <linux/signal.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/reset.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include "../../pci.h"

#include "pcie-designware.h"

#define IMX8MQ_PCIE_LINK_CAP_REG_OFFSET		0x7c
#define IMX8MQ_PCIE_LINK_CAP_L1EL_64US		(BIT(18) | BIT(17))
#define IMX8MQ_PCIE_L1SUB_CTRL1_REG_EN_MASK	0xf
#define IMX8MQ_GPR_PCIE_REF_USE_PAD		BIT(9)
#define IMX8MQ_GPR_PCIE_CLK_REQ_OVERRIDE_EN	BIT(10)
#define IMX8MQ_GPR_PCIE_CLK_REQ_OVERRIDE	BIT(11)
#define IMX8MQ_GPR12_PCIE2_CTRL_DEVICE_TYPE	GENMASK(11, 8)
#define IMX8MQ_PCIE2_BASE_ADDR			0x33c00000
#define IMX8_HSIO_PCIEB_BASE_ADDR		0x5f010000

#define to_imx6_pcie(x)	dev_get_drvdata((x)->dev)

enum imx6_pcie_variants {
	IMX6Q,
	IMX6SX,
	IMX6QP,
	IMX7D,
	IMX8MQ,
	IMX8MM,
	IMX8QM,
	IMX8QXP,
};

#define IMX6_PCIE_FLAG_IMX6_PHY			BIT(0)
#define IMX6_PCIE_FLAG_IMX6_SPEED_CHANGE	BIT(1)
#define IMX6_PCIE_FLAG_SUPPORTS_SUSPEND		BIT(2)
#define IMX6_PCIE_FLAG_IMX6_CPU_ADDR_FIXUP	BIT(3)
#define IMX6_PCIE_FLAG_SUPPORTS_L1SS		BIT(4)

struct imx6_pcie_drvdata {
	enum imx6_pcie_variants variant;
	u32 flags;
	int dbi_length;
};

struct imx6_pcie {
	struct dw_pcie		*pci;
	int			clkreq_gpio;
	int			dis_gpio;
	int			reset_gpio;
	bool			gpio_active_high;
	struct clk		*pcie_bus;
	struct clk		*pcie_phy;
	struct clk		*pcie_per;
	struct clk		*pcie_inbound_axi;
	struct clk		*pcie;
	struct clk		*pcie_aux;
	struct clk		*phy_per;
	struct clk		*misc_per;
	struct regmap		*iomuxc_gpr;
	u32			controller_id;
	struct reset_control	*pciephy_reset;
	struct reset_control	*apps_reset;
	struct reset_control	*turnoff_reset;
	struct reset_control	*clkreq_reset;
	u32			tx_deemph_gen1;
	u32			tx_deemph_gen2_3p5db;
	u32			tx_deemph_gen2_6db;
	u32			tx_swing_full;
	u32			tx_swing_low;
	u32			hsio_cfg;
	u32			ext_osc;
	u32			local_addr;
	u32			hard_wired;
	u32			dma_unroll_offset;
	int			link_gen;
	struct regulator	*vpcie;
	void __iomem		*phy_base;

	/* power domain for pcie */
	struct device		*pd_pcie;
	/* power domain for pcie phy */
	struct device		*pd_pcie_phy;
	/* power domain for hsio gpio used by pcie */
	struct device		*pd_hsio_gpio;
	const struct imx6_pcie_drvdata *drvdata;
	struct regulator	*epdev_on;
};

/* Parameters for the waiting for PCIe PHY PLL to lock on i.MX7 */
#define PHY_PLL_LOCK_WAIT_MAX_RETRIES	2000
#define PHY_PLL_LOCK_WAIT_USLEEP_MIN	50
#define PHY_PLL_LOCK_WAIT_USLEEP_MAX	200
#define PHY_PLL_LOCK_WAIT_TIMEOUT	(2000 * PHY_PLL_LOCK_WAIT_USLEEP_MAX)

/* PCIe Root Complex registers (memory-mapped) */
#define PCIE_RC_IMX6_MSI_CAP			0x50
#define PCIE_RC_LCR				0x7c
#define PCIE_RC_LCR_MAX_LINK_SPEEDS_GEN1	0x1
#define PCIE_RC_LCR_MAX_LINK_SPEEDS_GEN2	0x2
#define PCIE_RC_LCR_MAX_LINK_SPEEDS_MASK	0xf

#define PCIE_RC_LCSR				0x80

/* PCIe Port Logic registers (memory-mapped) */
#define PL_OFFSET 0x700

#define PCIE_PHY_DEBUG_R1 (PL_OFFSET + 0x2c)

#define PCIE_PHY_CTRL (PL_OFFSET + 0x114)
#define PCIE_PHY_CTRL_DATA(x)		FIELD_PREP(GENMASK(15, 0), (x))
#define PCIE_PHY_CTRL_CAP_ADR		BIT(16)
#define PCIE_PHY_CTRL_CAP_DAT		BIT(17)
#define PCIE_PHY_CTRL_WR		BIT(18)
#define PCIE_PHY_CTRL_RD		BIT(19)

#define PCIE_PHY_STAT (PL_OFFSET + 0x110)
#define PCIE_PHY_STAT_ACK		BIT(16)

#define PCIE_LINK_WIDTH_SPEED_CONTROL	0x80C

/* DMA registers */
#define MAX_PCIE_DMA_CHANNELS	8
#define DMA_UNROLL_CDM_OFFSET	(0x7 << 19)
#define DMA_REG_OFFSET		0x970
#define DMA_CTRL_VIEWPORT_OFF	(DMA_REG_OFFSET + 0x8)
#define DMA_WRITE_ENGINE_EN_OFF	(DMA_REG_OFFSET + 0xC)
#define DMA_WRITE_ENGINE_EN	BIT(0)
#define DMA_WRITE_DOORBELL	(DMA_REG_OFFSET + 0x10)
#define DMA_READ_ENGINE_EN_OFF	(DMA_REG_OFFSET + 0x2C)
#define DMA_READ_ENGINE_EN	BIT(0)
#define DMA_READ_DOORBELL	(DMA_REG_OFFSET + 0x30)
#define DMA_WRITE_INT_STS	(DMA_REG_OFFSET + 0x4C)
#define DMA_WRITE_INT_MASK	(DMA_REG_OFFSET + 0x54)
#define DMA_WRITE_INT_CLR	(DMA_REG_OFFSET + 0x58)
#define DMA_READ_INT_STS	(DMA_REG_OFFSET + 0xA0)
#define DMA_READ_INT_MASK	(DMA_REG_OFFSET + 0xA8)
#define DMA_READ_INT_CLR	(DMA_REG_OFFSET + 0xAC)
#define DMA_DONE_INT_STS	0xFF
#define DMA_ABORT_INT_STS	(0xFF << 16)
#define DMA_VIEWPOT_SEL_OFF	(DMA_REG_OFFSET + 0xFC)
#define DMA_CHANNEL_CTRL_1	(DMA_REG_OFFSET + 0x100)
#define DMA_CHANNEL_CTRL_1_LIE	BIT(3)
#define DMA_CHANNEL_CTRL_2	(DMA_REG_OFFSET + 0x104)
#define DMA_TRANSFER_SIZE	(DMA_REG_OFFSET + 0x108)
#define DMA_SAR_LOW		(DMA_REG_OFFSET + 0x10C)
#define DMA_SAR_HIGH		(DMA_REG_OFFSET + 0x110)
#define DMA_DAR_LOW		(DMA_REG_OFFSET + 0x114)
#define DMA_DAR_HIGH		(DMA_REG_OFFSET + 0x118)

/* PHY registers (not memory-mapped) */
#define PCIE_PHY_ATEOVRD			0x10
#define  PCIE_PHY_ATEOVRD_EN			BIT(2)
#define  PCIE_PHY_ATEOVRD_REF_CLKDIV_SHIFT	0
#define  PCIE_PHY_ATEOVRD_REF_CLKDIV_MASK	0x1

#define PCIE_PHY_MPLL_OVRD_IN_LO		0x11
#define  PCIE_PHY_MPLL_MULTIPLIER_SHIFT		2
#define  PCIE_PHY_MPLL_MULTIPLIER_MASK		0x7f
#define  PCIE_PHY_MPLL_MULTIPLIER_OVRD		BIT(9)

#define PCIE_PHY_RX_ASIC_OUT 0x100D
#define PCIE_PHY_RX_ASIC_OUT_VALID	(1 << 0)

/* iMX7 PCIe PHY registers */
#define PCIE_PHY_CMN_REG4		0x14
/* These are probably the bits that *aren't* DCC_FB_EN */
#define PCIE_PHY_CMN_REG4_DCC_FB_EN	0x29

#define PCIE_PHY_CMN_REG15	        0x54
#define PCIE_PHY_CMN_REG15_DLY_4	BIT(2)
#define PCIE_PHY_CMN_REG15_PLL_PD	BIT(5)
#define PCIE_PHY_CMN_REG15_OVRD_PLL_PD	BIT(7)

#define PCIE_PHY_CMN_REG24		0x90
#define PCIE_PHY_CMN_REG24_RX_EQ	BIT(6)
#define PCIE_PHY_CMN_REG24_RX_EQ_SEL	BIT(3)

#define PCIE_PHY_CMN_REG26		0x98
#define PCIE_PHY_CMN_REG26_ATT_MODE	0xBC

#define PCIE_PHY_CMN_REG62			0x188
#define PCIE_PHY_CMN_REG62_PLL_CLK_OUT		0x08
#define PCIE_PHY_CMN_REG64			0x190
#define PCIE_PHY_CMN_REG64_AUX_RX_TX_TERM	0x8C
#define PCIE_PHY_CMN_REG75			0x1D4
#define PCIE_PHY_CMN_REG75_PLL_DONE		0x3
#define PCIE_PHY_TRSV_REG5			0x414
#define PCIE_PHY_TRSV_REG5_GEN1_DEEMP		0x2D
#define PCIE_PHY_TRSV_REG6			0x418
#define PCIE_PHY_TRSV_REG6_GEN2_DEEMP		0xF

#define PHY_RX_OVRD_IN_LO 0x1005
#define PHY_RX_OVRD_IN_LO_RX_DATA_EN		BIT(5)
#define PHY_RX_OVRD_IN_LO_RX_PLL_EN		BIT(3)

/* iMX8 HSIO registers */
#define IMX8QM_CSR_PHYX2_OFFSET			0x00000
#define IMX8QM_CSR_PHYX1_OFFSET			0x10000
#define IMX8QM_CSR_PHYX_STTS0_OFFSET		0x4
#define IMX8QM_CSR_PCIEA_OFFSET			0x20000
#define IMX8QM_CSR_PCIEB_OFFSET			0x30000
#define IMX8QM_CSR_PCIE_CTRL1_OFFSET		0x4
#define IMX8QM_CSR_PCIE_CTRL2_OFFSET		0x8
#define IMX8QM_CSR_PCIE_STTS0_OFFSET		0xC
#define IMX8QM_CSR_MISC_OFFSET			0x50000

#define IMX8QM_CTRL_LTSSM_ENABLE		BIT(4)
#define IMX8QM_CTRL_READY_ENTR_L23		BIT(5)
#define IMX8QM_CTRL_PM_XMT_TURNOFF		BIT(9)
#define IMX8QM_CTRL_BUTTON_RST_N		BIT(21)
#define IMX8QM_CTRL_PERST_N			BIT(22)
#define IMX8QM_CTRL_POWER_UP_RST_N		BIT(23)

#define IMX8QM_CTRL_STTS0_PM_LINKST_IN_L2	BIT(13)
#define IMX8QM_CTRL_STTS0_PM_REQ_CORE_RST	BIT(19)
#define IMX8QM_STTS0_LANE0_TX_PLL_LOCK		BIT(4)
#define IMX8QM_STTS0_LANE1_TX_PLL_LOCK		BIT(12)

#define IMX8QM_PCIE_TYPE_MASK			(0xF << 24)

#define IMX8QM_PHYX2_CTRL0_APB_MASK		0x3
#define IMX8QM_PHY_APB_RSTN_0			BIT(0)
#define IMX8QM_PHY_APB_RSTN_1			BIT(1)

#define IMX8QM_MISC_IOB_RXENA			BIT(0)
#define IMX8QM_MISC_IOB_TXENA			BIT(1)
#define IMX8QM_CSR_MISC_IOB_A_0_TXOE		BIT(2)
#define IMX8QM_CSR_MISC_IOB_A_0_M1M0_MASK	(0x3 << 3)
#define IMX8QM_CSR_MISC_IOB_A_0_M1M0_2		BIT(4)
#define IMX8QM_MISC_PHYX1_EPCS_SEL		BIT(12)
#define IMX8QM_MISC_PCIE_AB_SELECT		BIT(13)

#define IMX8MM_GPR_PCIE_REF_CLK_SEL		(0x3 << 24)
#define IMX8MM_GPR_PCIE_REF_CLK_PLL		(0x3 << 24)
#define IMX8MM_GPR_PCIE_REF_CLK_EXT		(0x2 << 24)
#define IMX8MM_GPR_PCIE_AUX_EN			BIT(19)
#define IMX8MM_GPR_PCIE_CMN_RST			BIT(18)
#define IMX8MM_GPR_PCIE_POWER_OFF		BIT(17)
#define IMX8MM_GPR_PCIE_SSC_EN			BIT(16)

/*
 * The default value of the reserved ddr memory
 * used to verify EP/RC memory space access operations.
 * The layout of the 1G ddr on SD boards
 * [imx6qdl-sd-ard boards]0x1000_0000 ~ 0x4FFF_FFFF
 * [imx6sx,imx7d platforms]0x8000_0000 ~ 0xBFFF_FFFF
 *
 */
static u32 ddr_test_region = 0, test_region_size = SZ_2M;
static bool dma_w_end, dma_r_end, dma_en;

static bool imx6_pcie_readable_reg(struct device *dev, unsigned int reg)
{
	enum imx6_pcie_variants variant;
	struct imx6_pcie *imx6_pcie = dev_get_drvdata(dev);

	variant = imx6_pcie->drvdata->variant;
	if (variant == IMX8QXP) {
		switch (reg) {
		case IMX8QM_CSR_PHYX1_OFFSET:
		case IMX8QM_CSR_PCIEB_OFFSET:
		case IMX8QM_CSR_MISC_OFFSET:
		case IMX8QM_CSR_PHYX1_OFFSET + IMX8QM_CSR_PHYX_STTS0_OFFSET:
		case IMX8QM_CSR_PCIEB_OFFSET + IMX8QM_CSR_PCIE_CTRL1_OFFSET:
		case IMX8QM_CSR_PCIEB_OFFSET + IMX8QM_CSR_PCIE_CTRL2_OFFSET:
		case IMX8QM_CSR_PCIEB_OFFSET + IMX8QM_CSR_PCIE_STTS0_OFFSET:
			return true;

		default:
			return false;
		}
	} else {
		switch (reg) {
		case IMX8QM_CSR_PHYX2_OFFSET:
		case IMX8QM_CSR_PCIEA_OFFSET:
		case IMX8QM_CSR_MISC_OFFSET:
		case IMX8QM_CSR_PHYX2_OFFSET + IMX8QM_CSR_PHYX_STTS0_OFFSET:
		case IMX8QM_CSR_PCIEA_OFFSET + IMX8QM_CSR_PCIE_CTRL1_OFFSET:
		case IMX8QM_CSR_PCIEA_OFFSET + IMX8QM_CSR_PCIE_CTRL2_OFFSET:
		case IMX8QM_CSR_PCIEA_OFFSET + IMX8QM_CSR_PCIE_STTS0_OFFSET:
			return true;
		default:
			return false;
		}
	}
}

static bool imx6_pcie_writeable_reg(struct device *dev, unsigned int reg)
{
	enum imx6_pcie_variants variant;
	struct imx6_pcie *imx6_pcie = dev_get_drvdata(dev);

	variant = imx6_pcie->drvdata->variant;
	if (variant == IMX8QXP) {
		switch (reg) {
		case IMX8QM_CSR_PHYX1_OFFSET:
		case IMX8QM_CSR_PCIEB_OFFSET:
		case IMX8QM_CSR_MISC_OFFSET:
		case IMX8QM_CSR_PCIEB_OFFSET + IMX8QM_CSR_PCIE_CTRL1_OFFSET:
		case IMX8QM_CSR_PCIEB_OFFSET + IMX8QM_CSR_PCIE_CTRL2_OFFSET:
			return true;

		default:
			return false;
		}
	} else {
		switch (reg) {
		case IMX8QM_CSR_PHYX2_OFFSET:
		case IMX8QM_CSR_PCIEA_OFFSET:
		case IMX8QM_CSR_MISC_OFFSET:
		case IMX8QM_CSR_PCIEA_OFFSET + IMX8QM_CSR_PCIE_CTRL1_OFFSET:
		case IMX8QM_CSR_PCIEA_OFFSET + IMX8QM_CSR_PCIE_CTRL2_OFFSET:
			return true;
		default:
			return false;
		}
	}
}

static const struct regmap_config imx6_pcie_regconfig = {
	.max_register = IMX8QM_CSR_MISC_OFFSET,
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.num_reg_defaults_raw =  IMX8QM_CSR_MISC_OFFSET / sizeof(uint32_t) + 1,
	.readable_reg = imx6_pcie_readable_reg,
	.writeable_reg = imx6_pcie_writeable_reg,
	.cache_type = REGCACHE_NONE,
};

static int pcie_phy_poll_ack(struct imx6_pcie *imx6_pcie, bool exp_val)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	bool val;
	u32 max_iterations = 10;
	u32 wait_counter = 0;

	do {
		val = dw_pcie_readl_dbi(pci, PCIE_PHY_STAT) &
			PCIE_PHY_STAT_ACK;
		wait_counter++;

		if (val == exp_val)
			return 0;

		udelay(1);
	} while (wait_counter < max_iterations);

	return -ETIMEDOUT;
}

static int pcie_phy_wait_ack(struct imx6_pcie *imx6_pcie, int addr)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	u32 val;
	int ret;

	val = PCIE_PHY_CTRL_DATA(addr);
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, val);

	val |= PCIE_PHY_CTRL_CAP_ADR;
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, val);

	ret = pcie_phy_poll_ack(imx6_pcie, true);
	if (ret)
		return ret;

	val = PCIE_PHY_CTRL_DATA(addr);
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, val);

	return pcie_phy_poll_ack(imx6_pcie, false);
}

/* Read from the 16-bit PCIe PHY control registers (not memory-mapped) */
static int pcie_phy_read(struct imx6_pcie *imx6_pcie, int addr, u16 *data)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	u32 phy_ctl;
	int ret;

	ret = pcie_phy_wait_ack(imx6_pcie, addr);
	if (ret)
		return ret;

	/* assert Read signal */
	phy_ctl = PCIE_PHY_CTRL_RD;
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, phy_ctl);

	ret = pcie_phy_poll_ack(imx6_pcie, true);
	if (ret)
		return ret;

	*data = dw_pcie_readl_dbi(pci, PCIE_PHY_STAT);

	/* deassert Read signal */
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, 0x00);

	return pcie_phy_poll_ack(imx6_pcie, false);
}

static int pcie_phy_write(struct imx6_pcie *imx6_pcie, int addr, u16 data)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	u32 var;
	int ret;

	/* write addr */
	/* cap addr */
	ret = pcie_phy_wait_ack(imx6_pcie, addr);
	if (ret)
		return ret;

	var = PCIE_PHY_CTRL_DATA(data);
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, var);

	/* capture data */
	var |= PCIE_PHY_CTRL_CAP_DAT;
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, var);

	ret = pcie_phy_poll_ack(imx6_pcie, true);
	if (ret)
		return ret;

	/* deassert cap data */
	var = PCIE_PHY_CTRL_DATA(data);
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, var);

	/* wait for ack de-assertion */
	ret = pcie_phy_poll_ack(imx6_pcie, false);
	if (ret)
		return ret;

	/* assert wr signal */
	var = PCIE_PHY_CTRL_WR;
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, var);

	/* wait for ack */
	ret = pcie_phy_poll_ack(imx6_pcie, true);
	if (ret)
		return ret;

	/* deassert wr signal */
	var = PCIE_PHY_CTRL_DATA(data);
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, var);

	/* wait for ack de-assertion */
	ret = pcie_phy_poll_ack(imx6_pcie, false);
	if (ret)
		return ret;

	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, 0x0);

	return 0;
}

static void imx6_pcie_reset_phy(struct imx6_pcie *imx6_pcie)
{
	u16 tmp;

	if (!(imx6_pcie->drvdata->flags & IMX6_PCIE_FLAG_IMX6_PHY))
		return;

	pcie_phy_read(imx6_pcie, PHY_RX_OVRD_IN_LO, &tmp);
	tmp |= (PHY_RX_OVRD_IN_LO_RX_DATA_EN |
		PHY_RX_OVRD_IN_LO_RX_PLL_EN);
	pcie_phy_write(imx6_pcie, PHY_RX_OVRD_IN_LO, tmp);

	usleep_range(2000, 3000);

	pcie_phy_read(imx6_pcie, PHY_RX_OVRD_IN_LO, &tmp);
	tmp &= ~(PHY_RX_OVRD_IN_LO_RX_DATA_EN |
		  PHY_RX_OVRD_IN_LO_RX_PLL_EN);
	pcie_phy_write(imx6_pcie, PHY_RX_OVRD_IN_LO, tmp);
}

#ifdef CONFIG_ARM
/*  Added for PCI abort handling */
static int imx6q_pcie_abort_handler(unsigned long addr,
		unsigned int fsr, struct pt_regs *regs)
{
	unsigned long pc = instruction_pointer(regs);
	unsigned long instr;
	int reg ;

	/* if the abort from user-space, just return and report it */
	if (user_mode(regs))
		return 1;

	instr = *(unsigned long *)pc;
	reg = (instr >> 12) & 15;

	/*
	 * If the instruction being executed was a read,
	 * make it look like it read all-ones.
	 */
	if ((instr & 0x0c100000) == 0x04100000) {
		unsigned long val;

		if (instr & 0x00400000)
			val = 255;
		else
			val = -1;

		regs->uregs[reg] = val;
		regs->ARM_pc += 4;
		return 0;
	}

	if ((instr & 0x0e100090) == 0x00100090) {
		regs->uregs[reg] = -1;
		regs->ARM_pc += 4;
		return 0;
	}

	return 1;
}
#endif

static int imx6_pcie_attach_pd(struct device *dev)
{
	struct imx6_pcie *imx6_pcie = dev_get_drvdata(dev);
	struct device_link *link;

	/* Do nothing when in a single power domain */
	if (dev->pm_domain)
		return 0;

	imx6_pcie->pd_pcie = dev_pm_domain_attach_by_name(dev, "pcie");
	if (IS_ERR(imx6_pcie->pd_pcie))
		return PTR_ERR(imx6_pcie->pd_pcie);
	/* Do nothing when power domain missing */
	if (!imx6_pcie->pd_pcie)
		return 0;
	link = device_link_add(dev, imx6_pcie->pd_pcie,
			DL_FLAG_STATELESS |
			DL_FLAG_PM_RUNTIME |
			DL_FLAG_RPM_ACTIVE);
	if (!link) {
		dev_err(dev, "Failed to add device_link to pcie pd.\n");
		return -EINVAL;
	}

	imx6_pcie->pd_pcie_phy = dev_pm_domain_attach_by_name(dev, "pcie_phy");
	if (IS_ERR(imx6_pcie->pd_pcie_phy))
		return PTR_ERR(imx6_pcie->pd_pcie_phy);

	link = device_link_add(dev, imx6_pcie->pd_pcie_phy,
			DL_FLAG_STATELESS |
			DL_FLAG_PM_RUNTIME |
			DL_FLAG_RPM_ACTIVE);
	if (!link) {
		dev_err(dev, "Failed to add device_link to pcie_phy pd.\n");
		return -EINVAL;
	}

	switch (imx6_pcie->drvdata->variant) {
	case IMX8QM:
	case IMX8QXP:
		imx6_pcie->pd_hsio_gpio = dev_pm_domain_attach_by_name(dev, "hsio_gpio");
		if (IS_ERR(imx6_pcie->pd_hsio_gpio))
			return PTR_ERR(imx6_pcie->pd_hsio_gpio);

		link = device_link_add(dev, imx6_pcie->pd_hsio_gpio,
				DL_FLAG_STATELESS |
				DL_FLAG_PM_RUNTIME |
				DL_FLAG_RPM_ACTIVE);
		if (!link) {
			dev_err(dev, "Failed to add device_link to hsio_gpio pd.\n");
			return -EINVAL;
		}

		break;
	default:
		break;
	}

	return 0;
}

static unsigned int imx6_pcie_grp_offset(const struct imx6_pcie *imx6_pcie)
{
	return imx6_pcie->controller_id == 1 ? IOMUXC_GPR16 : IOMUXC_GPR14;
}

static int imx6_pcie_enable_ref_clk(struct imx6_pcie *imx6_pcie)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	struct device *dev = pci->dev;
	unsigned int offset;
	int ret = 0;

	switch (imx6_pcie->drvdata->variant) {
	case IMX6SX:
		ret = clk_prepare_enable(imx6_pcie->pcie_inbound_axi);
		if (ret) {
			dev_err(dev, "unable to enable pcie_axi clock\n");
			break;
		}

		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6SX_GPR12_PCIE_TEST_POWERDOWN, 0);
		break;
	case IMX6QP:		/* FALLTHROUGH */
	case IMX6Q:
		/* power up core phy and enable ref clock */
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_TEST_PD, 0 << 18);
		/*
		 * the async reset input need ref clock to sync internally,
		 * when the ref clock comes after reset, internal synced
		 * reset time is too short, cannot meet the requirement.
		 * add one ~10us delay here.
		 */
		usleep_range(10, 100);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_REF_CLK_EN, 1 << 16);
		break;
	case IMX7D:
		break;
	case IMX8MQ:
	case IMX8MM:
		ret = clk_prepare_enable(imx6_pcie->pcie_aux);
		if (ret) {
			dev_err(dev, "unable to enable pcie_aux clock\n");
			break;
		}

		offset = imx6_pcie_grp_offset(imx6_pcie);
		/*
		 * Set the over ride low and enabled
		 * make sure that REF_CLK is turned on.
		 */
		regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
				   IMX8MQ_GPR_PCIE_CLK_REQ_OVERRIDE,
				   0);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
				   IMX8MQ_GPR_PCIE_CLK_REQ_OVERRIDE_EN,
				   IMX8MQ_GPR_PCIE_CLK_REQ_OVERRIDE_EN);
		break;
	case IMX8QXP:
	case IMX8QM:
		ret = clk_prepare_enable(imx6_pcie->pcie_inbound_axi);
		if (ret) {
			dev_err(dev, "unable to enable pcie_axi clock\n");
			break;
		}
		ret = clk_prepare_enable(imx6_pcie->pcie_per);
		if (ret) {
			dev_err(dev, "unable to enable pcie_per clock\n");
			clk_disable_unprepare(imx6_pcie->pcie_inbound_axi);
			break;
		}

		ret = clk_prepare_enable(imx6_pcie->phy_per);
		if (unlikely(ret)) {
			clk_disable_unprepare(imx6_pcie->pcie_per);
			clk_disable_unprepare(imx6_pcie->pcie_inbound_axi);
			dev_err(dev, "unable to enable phy per clock\n");
		}
		ret = clk_prepare_enable(imx6_pcie->misc_per);
		if (unlikely(ret)) {
			clk_disable_unprepare(imx6_pcie->phy_per);
			clk_disable_unprepare(imx6_pcie->pcie_per);
			clk_disable_unprepare(imx6_pcie->pcie_inbound_axi);
			dev_err(dev, "unable to enable misc per clock\n");
		}
		break;
	}

	return ret;
}

static void imx7d_pcie_wait_for_phy_pll_lock(struct imx6_pcie *imx6_pcie)
{
	u32 val;
	struct device *dev = imx6_pcie->pci->dev;

	if (regmap_read_poll_timeout(imx6_pcie->iomuxc_gpr,
				     IOMUXC_GPR22, val,
				     val & IMX7D_GPR22_PCIE_PHY_PLL_LOCKED,
				     PHY_PLL_LOCK_WAIT_USLEEP_MAX,
				     PHY_PLL_LOCK_WAIT_TIMEOUT))
		dev_err(dev, "PCIe PLL lock timeout\n");
}
static void imx8_pcie_wait_for_phy_pll_lock(struct imx6_pcie *imx6_pcie)
{
	u32 val, retries = 0, tmp = 0, orig = 0;
	struct dw_pcie *pci = imx6_pcie->pci;
	struct device *dev = pci->dev;

	switch (imx6_pcie->drvdata->variant) {
	case IMX8MM:
		for (retries = 0; retries < PHY_PLL_LOCK_WAIT_MAX_RETRIES;
		     retries++) {
			tmp = readl(imx6_pcie->phy_base + PCIE_PHY_CMN_REG75);
			if (tmp == PCIE_PHY_CMN_REG75_PLL_DONE)
				break;
			udelay(10);
		}
		break;

	case IMX8QXP:
	case IMX8QM:
		for (retries = 0; retries < PHY_PLL_LOCK_WAIT_MAX_RETRIES;
		     retries++) {
			if (imx6_pcie->hsio_cfg == PCIEAX1PCIEBX1SATA) {
				regmap_read(imx6_pcie->iomuxc_gpr,
					    IMX8QM_CSR_PHYX2_OFFSET + 0x4,
					    &tmp);
				if (imx6_pcie->controller_id == 0) /* pciea 1 lanes */
					orig = IMX8QM_STTS0_LANE0_TX_PLL_LOCK;
				else /* pcieb 1 lanes */
					orig = IMX8QM_STTS0_LANE1_TX_PLL_LOCK;
			} else if (imx6_pcie->hsio_cfg == PCIEAX2PCIEBX1) {
				val = IMX8QM_CSR_PHYX2_OFFSET
					+ imx6_pcie->controller_id * SZ_64K;
				regmap_read(imx6_pcie->iomuxc_gpr,
					    val + IMX8QM_CSR_PHYX_STTS0_OFFSET,
					    &tmp);
				orig = IMX8QM_STTS0_LANE0_TX_PLL_LOCK;
				if (imx6_pcie->controller_id == 0) /* pciea 2 lanes */
					orig |= IMX8QM_STTS0_LANE1_TX_PLL_LOCK;
			} else if (imx6_pcie->hsio_cfg == PCIEAX2SATA) {
				regmap_read(imx6_pcie->iomuxc_gpr,
					    IMX8QM_CSR_PHYX2_OFFSET + 0x4,
					    &tmp);
				orig = IMX8QM_STTS0_LANE0_TX_PLL_LOCK;
				orig |= IMX8QM_STTS0_LANE1_TX_PLL_LOCK;
			}
			tmp &= orig;
			if (tmp == orig)
				break;
			udelay(10);
		}
		break;

	default:
		break;
	}

	if (retries >= PHY_PLL_LOCK_WAIT_MAX_RETRIES)
		dev_err(dev, "PCIe PLL lock timeout\n");
}

static void imx6_pcie_clk_enable(struct imx6_pcie *imx6_pcie)
{
	int ret;
	struct dw_pcie *pci = imx6_pcie->pci;
	struct device *dev = pci->dev;

	ret = clk_prepare_enable(imx6_pcie->pcie_phy);
	if (ret)
		dev_err(dev, "unable to enable pcie_phy clock\n");

	ret = clk_prepare_enable(imx6_pcie->pcie_bus);
	if (ret)
		dev_err(dev, "unable to enable pcie_bus clock\n");

	ret = clk_prepare_enable(imx6_pcie->pcie);
	if (ret)
		dev_err(dev, "unable to enable pcie clock\n");

	ret = imx6_pcie_enable_ref_clk(imx6_pcie);
	if (ret)
		dev_err(dev, "unable to enable pcie ref clock\n");

	/* allow the clocks to stabilize */
	usleep_range(200, 500);
}

static void imx6_pcie_clk_disable(struct imx6_pcie *imx6_pcie)
{
	clk_disable_unprepare(imx6_pcie->pcie);
	clk_disable_unprepare(imx6_pcie->pcie_phy);
	clk_disable_unprepare(imx6_pcie->pcie_bus);

	switch (imx6_pcie->drvdata->variant) {
	case IMX6Q:
	case IMX6QP:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				IMX6Q_GPR1_PCIE_REF_CLK_EN, 0);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				IMX6Q_GPR1_PCIE_TEST_PD,
				IMX6Q_GPR1_PCIE_TEST_PD);
		break;
	case IMX6SX:
		clk_disable_unprepare(imx6_pcie->pcie_inbound_axi);
		break;
	case IMX7D:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX7D_GPR12_PCIE_PHY_REFCLK_SEL,
				   IMX7D_GPR12_PCIE_PHY_REFCLK_SEL);
		break;
	case IMX8MQ:
	case IMX8MM:
		clk_disable_unprepare(imx6_pcie->pcie_aux);
		break;
	case IMX8QXP:
	case IMX8QM:
		clk_disable_unprepare(imx6_pcie->pcie_per);
		clk_disable_unprepare(imx6_pcie->pcie_inbound_axi);
		clk_disable_unprepare(imx6_pcie->phy_per);
		clk_disable_unprepare(imx6_pcie->misc_per);
		break;
	default:
		break;
	}
}

static void imx6_pcie_assert_core_reset(struct imx6_pcie *imx6_pcie)
{
	u32 val;
	int i;
	struct device *dev = imx6_pcie->pci->dev;

	switch (imx6_pcie->drvdata->variant) {
	case IMX7D:
	case IMX8MQ:
	case IMX8MM:
		reset_control_assert(imx6_pcie->pciephy_reset);
		reset_control_assert(imx6_pcie->apps_reset);
		break;
	case IMX6SX:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6SX_GPR12_PCIE_TEST_POWERDOWN,
				   IMX6SX_GPR12_PCIE_TEST_POWERDOWN);
		/* Force PCIe PHY reset */
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR5,
				   IMX6SX_GPR5_PCIE_BTNRST_RESET,
				   IMX6SX_GPR5_PCIE_BTNRST_RESET);
		break;
	case IMX6QP:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_SW_RST,
				   IMX6Q_GPR1_PCIE_SW_RST);
		break;
	case IMX6Q:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_TEST_PD, 1 << 18);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_REF_CLK_EN, 0 << 16);
		break;
	case IMX8QXP:
		imx6_pcie_clk_enable(imx6_pcie);
		val = IMX8QM_CSR_PCIEB_OFFSET;
		regmap_update_bits(imx6_pcie->iomuxc_gpr,
				val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
				IMX8QM_CTRL_BUTTON_RST_N,
				IMX8QM_CTRL_BUTTON_RST_N);
		regmap_update_bits(imx6_pcie->iomuxc_gpr,
				val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
				IMX8QM_CTRL_PERST_N,
				IMX8QM_CTRL_PERST_N);
		regmap_update_bits(imx6_pcie->iomuxc_gpr,
				val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
				IMX8QM_CTRL_POWER_UP_RST_N,
				IMX8QM_CTRL_POWER_UP_RST_N);
		break;
	case IMX8QM:
		imx6_pcie_clk_enable(imx6_pcie);
		for (i = 0; i <= imx6_pcie->controller_id; i++) {
			val = IMX8QM_CSR_PCIEA_OFFSET + i * SZ_64K;
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
					val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
					IMX8QM_CTRL_BUTTON_RST_N,
					IMX8QM_CTRL_BUTTON_RST_N);
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
					val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
					IMX8QM_CTRL_PERST_N,
					IMX8QM_CTRL_PERST_N);
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
					val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
					IMX8QM_CTRL_POWER_UP_RST_N,
					IMX8QM_CTRL_POWER_UP_RST_N);
		}
		break;
	}

	if (imx6_pcie->vpcie && regulator_is_enabled(imx6_pcie->vpcie) > 0) {
		int ret = regulator_disable(imx6_pcie->vpcie);

		if (ret)
			dev_err(dev, "failed to disable vpcie regulator: %d\n",
				ret);
	}
}

static void imx6_pcie_deassert_core_reset(struct imx6_pcie *imx6_pcie)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	struct device *dev = pci->dev;
	int ret, i;
	u32 val, tmp;

	if (imx6_pcie->vpcie && !regulator_is_enabled(imx6_pcie->vpcie)) {
		ret = regulator_enable(imx6_pcie->vpcie);
		if (ret) {
			dev_err(dev, "failed to enable vpcie regulator: %d\n",
				ret);
			return;
		}
	}

	switch (imx6_pcie->drvdata->variant) {
	case IMX8QXP:
	case IMX8QM:
		/* ClKs had been enabled */
		break;
	default:
		imx6_pcie_clk_enable(imx6_pcie);
		break;
	}

	/* Some boards don't have PCIe reset GPIO. */
	if (gpio_is_valid(imx6_pcie->reset_gpio)) {
		gpio_set_value_cansleep(imx6_pcie->reset_gpio,
					imx6_pcie->gpio_active_high);
		msleep(100);
		gpio_set_value_cansleep(imx6_pcie->reset_gpio,
					!imx6_pcie->gpio_active_high);
	}

	switch (imx6_pcie->drvdata->variant) {
	case IMX8QXP:
	case IMX8QM:
		val = IMX8QM_CSR_PCIEA_OFFSET
			+ imx6_pcie->controller_id * SZ_64K;
		/* bit19 PM_REQ_CORE_RST of pciex#_stts0 should be cleared. */
		for (i = 0; i < 100; i++) {
			regmap_read(imx6_pcie->iomuxc_gpr,
					val + IMX8QM_CSR_PCIE_STTS0_OFFSET,
					&tmp);
			if ((tmp & IMX8QM_CTRL_STTS0_PM_REQ_CORE_RST) == 0)
				break;
			udelay(10);
		}

		if ((tmp & IMX8QM_CTRL_STTS0_PM_REQ_CORE_RST) != 0)
			dev_err(dev, "ERROR PM_REQ_CORE_RST is still set.\n");

		/* wait for phy pll lock firstly. */
		imx8_pcie_wait_for_phy_pll_lock(imx6_pcie);
		break;
	case IMX8MQ:
	case IMX8MM:
		reset_control_deassert(imx6_pcie->pciephy_reset);

		imx8_pcie_wait_for_phy_pll_lock(imx6_pcie);
		/*
		 * Set the over ride low and enabled
		 * make sure that REF_CLK is turned on.
		 */
		val = imx6_pcie_grp_offset(imx6_pcie);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, val,
				   IMX8MQ_GPR_PCIE_CLK_REQ_OVERRIDE,
				   0);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, val,
				   IMX8MQ_GPR_PCIE_CLK_REQ_OVERRIDE_EN,
				   IMX8MQ_GPR_PCIE_CLK_REQ_OVERRIDE_EN);

		if (imx6_pcie->drvdata->flags & IMX6_PCIE_FLAG_SUPPORTS_L1SS) {
			/*
			 * Configure the CLK_REQ# high, let the L1SS
			 * automatically controlled by HW later.
			 */
			reset_control_deassert(imx6_pcie->clkreq_reset);
			/*
			 * Configure the L1 latency of rc to less than 64us
			 * Otherwise, the L1/L1SUB wouldn't be enable by ASPM.
			 */
			dw_pcie_dbi_ro_wr_en(pci);
			val = readl(pci->dbi_base + SZ_1M +
					IMX8MQ_PCIE_LINK_CAP_REG_OFFSET);
			val &= ~PCI_EXP_LNKCAP_L1EL;
			val |= IMX8MQ_PCIE_LINK_CAP_L1EL_64US;
			writel(val, pci->dbi_base + SZ_1M +
					IMX8MQ_PCIE_LINK_CAP_REG_OFFSET);
			dw_pcie_dbi_ro_wr_dis(pci);
		}
		break;
	case IMX7D:
		reset_control_deassert(imx6_pcie->pciephy_reset);

		/* Workaround for ERR010728, failure of PCI-e PLL VCO to
		 * oscillate, especially when cold.  This turns off "Duty-cycle
		 * Corrector" and other mysterious undocumented things.
		 */
		if (likely(imx6_pcie->phy_base)) {
			/* De-assert DCC_FB_EN */
			writel(PCIE_PHY_CMN_REG4_DCC_FB_EN,
			       imx6_pcie->phy_base + PCIE_PHY_CMN_REG4);
			/* Assert RX_EQS and RX_EQS_SEL */
			writel(PCIE_PHY_CMN_REG24_RX_EQ_SEL
				| PCIE_PHY_CMN_REG24_RX_EQ,
			       imx6_pcie->phy_base + PCIE_PHY_CMN_REG24);
			/* Assert ATT_MODE */
			writel(PCIE_PHY_CMN_REG26_ATT_MODE,
			       imx6_pcie->phy_base + PCIE_PHY_CMN_REG26);
		} else {
			dev_warn(dev, "Unable to apply ERR010728 workaround. DT missing fsl,imx7d-pcie-phy phandle ?\n");
		}

		imx7d_pcie_wait_for_phy_pll_lock(imx6_pcie);
		break;
	case IMX6SX:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR5,
				   IMX6SX_GPR5_PCIE_BTNRST_RESET, 0);
		break;
	case IMX6QP:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_SW_RST, 0);

		usleep_range(200, 500);
		break;
	case IMX6Q:		/* Nothing to do */
		break;
	}

	return;
}

static void imx6_pcie_configure_type(struct imx6_pcie *imx6_pcie)
{
	unsigned int mask, val;

	if (IS_ENABLED(CONFIG_EP_MODE_IN_EP_RC_SYS)
			&& (imx6_pcie->hard_wired == 0)) {
		if (imx6_pcie->drvdata->variant == IMX8QM
				|| imx6_pcie->drvdata->variant == IMX8QXP) {
			val = IMX8QM_CSR_PCIEA_OFFSET
				+ imx6_pcie->controller_id * SZ_64K;
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
					val, IMX8QM_PCIE_TYPE_MASK,
					PCI_EXP_TYPE_ENDPOINT << 24);
		} else {
			if (unlikely(imx6_pcie->controller_id))
				/* iMX8MQ second PCIE */
				regmap_update_bits(imx6_pcie->iomuxc_gpr,
						IOMUXC_GPR12,
						IMX6Q_GPR12_DEVICE_TYPE >> 4,
						PCI_EXP_TYPE_ENDPOINT << 8);
			else
				regmap_update_bits(imx6_pcie->iomuxc_gpr,
						IOMUXC_GPR12,
						IMX6Q_GPR12_DEVICE_TYPE,
						PCI_EXP_TYPE_ENDPOINT << 12);
		}
	} else {
		if (imx6_pcie->drvdata->variant == IMX8QM ||
		    imx6_pcie->drvdata->variant == IMX8QXP) {
			val = IMX8QM_CSR_PCIEA_OFFSET
				+ imx6_pcie->controller_id * SZ_64K;
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
					val, IMX8QM_PCIE_TYPE_MASK,
					PCI_EXP_TYPE_ROOT_PORT << 24);
		} else if (imx6_pcie->drvdata->variant == IMX8MQ &&
		    imx6_pcie->controller_id == 1) {
			mask   = IMX8MQ_GPR12_PCIE2_CTRL_DEVICE_TYPE;
			val    = FIELD_PREP(IMX8MQ_GPR12_PCIE2_CTRL_DEVICE_TYPE,
					    PCI_EXP_TYPE_ROOT_PORT);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
					mask, val);
		} else {
			mask = IMX6Q_GPR12_DEVICE_TYPE;
			val  = FIELD_PREP(IMX6Q_GPR12_DEVICE_TYPE,
					  PCI_EXP_TYPE_ROOT_PORT);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
					mask, val);
		}
	}
}

static void imx6_pcie_init_phy(struct imx6_pcie *imx6_pcie)
{
	unsigned int offset;

	switch (imx6_pcie->drvdata->variant) {
	case IMX8QXP:
	case IMX8QM:
		if (imx6_pcie->hsio_cfg == PCIEAX2SATA) {
			/*
			 * bit 0 rx ena 1.
			 * bit12 PHY_X1_EPCS_SEL 1.
			 * bit13 phy_ab_select 0.
			 */
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_PHYX2_OFFSET,
				IMX8QM_PHYX2_CTRL0_APB_MASK,
				IMX8QM_PHY_APB_RSTN_0
				| IMX8QM_PHY_APB_RSTN_1);

			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_PHYX1_EPCS_SEL,
				IMX8QM_MISC_PHYX1_EPCS_SEL);
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_PCIE_AB_SELECT,
				0);
		} else if (imx6_pcie->hsio_cfg == PCIEAX1PCIEBX1SATA) {
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_PHYX2_OFFSET,
				IMX8QM_PHYX2_CTRL0_APB_MASK,
				IMX8QM_PHY_APB_RSTN_0
				| IMX8QM_PHY_APB_RSTN_1);

			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_PHYX1_EPCS_SEL,
				IMX8QM_MISC_PHYX1_EPCS_SEL);
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_PCIE_AB_SELECT,
				IMX8QM_MISC_PCIE_AB_SELECT);
		} else if (imx6_pcie->hsio_cfg == PCIEAX2PCIEBX1) {
			/*
			 * bit 0 rx ena 1.
			 * bit12 PHY_X1_EPCS_SEL 0.
			 * bit13 phy_ab_select 1.
			 */
			if (imx6_pcie->controller_id)
				regmap_update_bits(imx6_pcie->iomuxc_gpr,
					IMX8QM_CSR_PHYX1_OFFSET,
					IMX8QM_PHY_APB_RSTN_0,
					IMX8QM_PHY_APB_RSTN_0);
			else
				regmap_update_bits(imx6_pcie->iomuxc_gpr,
					IMX8QM_CSR_PHYX2_OFFSET,
					IMX8QM_PHYX2_CTRL0_APB_MASK,
					IMX8QM_PHY_APB_RSTN_0
					| IMX8QM_PHY_APB_RSTN_1);

			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_PHYX1_EPCS_SEL,
				0);
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_PCIE_AB_SELECT,
				IMX8QM_MISC_PCIE_AB_SELECT);
		}

		if (imx6_pcie->ext_osc) {
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_IOB_RXENA,
				IMX8QM_MISC_IOB_RXENA);
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_IOB_TXENA,
				0);
		} else {
			/* Try to used the internal pll as ref clk */
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_IOB_RXENA,
				0);
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_IOB_TXENA,
				IMX8QM_MISC_IOB_TXENA);
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_CSR_MISC_IOB_A_0_TXOE
				| IMX8QM_CSR_MISC_IOB_A_0_M1M0_MASK,
				IMX8QM_CSR_MISC_IOB_A_0_TXOE
				| IMX8QM_CSR_MISC_IOB_A_0_M1M0_2);
		}

		break;
	case IMX8MM:
		offset = imx6_pcie_grp_offset(imx6_pcie);

		dev_info(imx6_pcie->pci->dev, "%s REF_CLK is used!.\n",
			 imx6_pcie->ext_osc ? "EXT" : "PLL");
		if (imx6_pcie->ext_osc) {
			regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
					   IMX8MQ_GPR_PCIE_REF_USE_PAD, 0);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
					   IMX8MM_GPR_PCIE_REF_CLK_SEL,
					   IMX8MM_GPR_PCIE_REF_CLK_SEL);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
					   IMX8MM_GPR_PCIE_AUX_EN,
					   IMX8MM_GPR_PCIE_AUX_EN);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
					   IMX8MM_GPR_PCIE_POWER_OFF, 0);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
					   IMX8MM_GPR_PCIE_SSC_EN, 0);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
					   IMX8MM_GPR_PCIE_REF_CLK_SEL,
					   IMX8MM_GPR_PCIE_REF_CLK_EXT);
			udelay(100);
			/* Do the PHY common block reset */
			regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
					   IMX8MM_GPR_PCIE_CMN_RST,
					   IMX8MM_GPR_PCIE_CMN_RST);
			udelay(200);
		} else {
			/* Configure the internal PLL as REF clock */
			regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
					   IMX8MQ_GPR_PCIE_REF_USE_PAD, 0);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
					   IMX8MM_GPR_PCIE_REF_CLK_SEL,
					   IMX8MM_GPR_PCIE_REF_CLK_SEL);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
					   IMX8MM_GPR_PCIE_AUX_EN,
					   IMX8MM_GPR_PCIE_AUX_EN);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
					   IMX8MM_GPR_PCIE_POWER_OFF, 0);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
					   IMX8MM_GPR_PCIE_SSC_EN, 0);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
					   IMX8MM_GPR_PCIE_REF_CLK_SEL,
					   IMX8MM_GPR_PCIE_REF_CLK_PLL);
			udelay(100);
			/* Configure the PHY */
			writel(PCIE_PHY_CMN_REG62_PLL_CLK_OUT,
			       imx6_pcie->phy_base + PCIE_PHY_CMN_REG62);
			writel(PCIE_PHY_CMN_REG64_AUX_RX_TX_TERM,
			       imx6_pcie->phy_base + PCIE_PHY_CMN_REG64);
			/* Do the PHY common block reset */
			regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
					   IMX8MM_GPR_PCIE_CMN_RST,
					   IMX8MM_GPR_PCIE_CMN_RST);
			udelay(200);
		}

		/*
		 * In order to pass the compliance tests.
		 * Configure the TRSV regiser of iMX8MM PCIe PHY.
		 */
		writel(PCIE_PHY_TRSV_REG5_GEN1_DEEMP,
		       imx6_pcie->phy_base + PCIE_PHY_TRSV_REG5);
		writel(PCIE_PHY_TRSV_REG6_GEN2_DEEMP,
		       imx6_pcie->phy_base + PCIE_PHY_TRSV_REG6);

		break;
	case IMX8MQ:
		/*
		 * TODO: Currently this code assumes external
		 * oscillator is being used
		 */
		regmap_update_bits(imx6_pcie->iomuxc_gpr,
				   imx6_pcie_grp_offset(imx6_pcie),
				   IMX8MQ_GPR_PCIE_REF_USE_PAD,
				   IMX8MQ_GPR_PCIE_REF_USE_PAD);
		break;
	case IMX7D:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX7D_GPR12_PCIE_PHY_REFCLK_SEL, 0);
		break;
	case IMX6SX:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6SX_GPR12_PCIE_RX_EQ_MASK,
				   IMX6SX_GPR12_PCIE_RX_EQ_2);
		/* FALLTHROUGH */
	default:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6Q_GPR12_PCIE_CTL_2, 0 << 10);

		/* configure constant input signal to the pcie ctrl and phy */
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6Q_GPR12_LOS_LEVEL, 9 << 4);

		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR8,
				   IMX6Q_GPR8_TX_DEEMPH_GEN1,
				   imx6_pcie->tx_deemph_gen1 << 0);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR8,
				   IMX6Q_GPR8_TX_DEEMPH_GEN2_3P5DB,
				   imx6_pcie->tx_deemph_gen2_3p5db << 6);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR8,
				   IMX6Q_GPR8_TX_DEEMPH_GEN2_6DB,
				   imx6_pcie->tx_deemph_gen2_6db << 12);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR8,
				   IMX6Q_GPR8_TX_SWING_FULL,
				   imx6_pcie->tx_swing_full << 18);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR8,
				   IMX6Q_GPR8_TX_SWING_LOW,
				   imx6_pcie->tx_swing_low << 25);
		break;
	}

	imx6_pcie_configure_type(imx6_pcie);
}

static int imx6_setup_phy_mpll(struct imx6_pcie *imx6_pcie)
{
	unsigned long phy_rate = clk_get_rate(imx6_pcie->pcie_phy);
	int mult, div;
	u16 val;

	if (!(imx6_pcie->drvdata->flags & IMX6_PCIE_FLAG_IMX6_PHY))
		return 0;

	switch (phy_rate) {
	case 125000000:
		/*
		 * The default settings of the MPLL are for a 125MHz input
		 * clock, so no need to reconfigure anything in that case.
		 */
		return 0;
	case 100000000:
		mult = 25;
		div = 0;
		break;
	case 200000000:
		mult = 25;
		div = 1;
		break;
	default:
		dev_err(imx6_pcie->pci->dev,
			"Unsupported PHY reference clock rate %lu\n", phy_rate);
		return -EINVAL;
	}

	pcie_phy_read(imx6_pcie, PCIE_PHY_MPLL_OVRD_IN_LO, &val);
	val &= ~(PCIE_PHY_MPLL_MULTIPLIER_MASK <<
		 PCIE_PHY_MPLL_MULTIPLIER_SHIFT);
	val |= mult << PCIE_PHY_MPLL_MULTIPLIER_SHIFT;
	val |= PCIE_PHY_MPLL_MULTIPLIER_OVRD;
	pcie_phy_write(imx6_pcie, PCIE_PHY_MPLL_OVRD_IN_LO, val);

	pcie_phy_read(imx6_pcie, PCIE_PHY_ATEOVRD, &val);
	val &= ~(PCIE_PHY_ATEOVRD_REF_CLKDIV_MASK <<
		 PCIE_PHY_ATEOVRD_REF_CLKDIV_SHIFT);
	val |= div << PCIE_PHY_ATEOVRD_REF_CLKDIV_SHIFT;
	val |= PCIE_PHY_ATEOVRD_EN;
	pcie_phy_write(imx6_pcie, PCIE_PHY_ATEOVRD, val);

	return 0;
}

static int imx6_pcie_wait_for_speed_change(struct imx6_pcie *imx6_pcie)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	struct device *dev = pci->dev;
	u32 tmp;
	unsigned int retries;

	for (retries = 0; retries < 200; retries++) {
		tmp = dw_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
		/* Test if the speed change finished. */
		if (!(tmp & PORT_LOGIC_SPEED_CHANGE))
			return 0;
		usleep_range(100, 1000);
	}

	dev_err(dev, "Speed change timeout\n");
	return -ETIMEDOUT;
}

static void imx6_pcie_ltssm_enable(struct device *dev)
{
	u32 val;
	struct imx6_pcie *imx6_pcie = dev_get_drvdata(dev);

	switch (imx6_pcie->drvdata->variant) {
	case IMX6Q:
	case IMX6SX:
	case IMX6QP:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6Q_GPR12_PCIE_CTL_2,
				   IMX6Q_GPR12_PCIE_CTL_2);
		break;
	case IMX7D:
	case IMX8MQ:
	case IMX8MM:
		reset_control_deassert(imx6_pcie->apps_reset);
		break;
	case IMX8QXP:
	case IMX8QM:
		/* Bit4 of the CTRL2 */
		val = IMX8QM_CSR_PCIEA_OFFSET
			+ imx6_pcie->controller_id * SZ_64K;
		regmap_update_bits(imx6_pcie->iomuxc_gpr,
				val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
				IMX8QM_CTRL_LTSSM_ENABLE,
				IMX8QM_CTRL_LTSSM_ENABLE);
		break;
	}
}

static int imx6_pcie_establish_link(struct imx6_pcie *imx6_pcie)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	struct device *dev = pci->dev;
	u32 tmp;
	int ret;

	/*
	 * Force Gen1 operation when starting the link.  In case the link is
	 * started in Gen2 mode, there is a possibility the devices on the
	 * bus will not be detected at all.  This happens with PCIe switches.
	 */
	tmp = dw_pcie_readl_dbi(pci, PCIE_RC_LCR);
	tmp &= ~PCIE_RC_LCR_MAX_LINK_SPEEDS_MASK;
	tmp |= PCIE_RC_LCR_MAX_LINK_SPEEDS_GEN1;
	dw_pcie_writel_dbi(pci, PCIE_RC_LCR, tmp);

	/* Start LTSSM. */
	imx6_pcie_ltssm_enable(dev);

	ret = dw_pcie_wait_for_link(pci);
	if (ret)
		goto err_reset_phy;

	if (imx6_pcie->link_gen >= 2) {
		/* Allow Gen2 mode after the link is up. */
		tmp = dw_pcie_readl_dbi(pci, PCIE_RC_LCR);
		tmp &= ~PCIE_RC_LCR_MAX_LINK_SPEEDS_MASK;
		tmp |= imx6_pcie->link_gen;
		dw_pcie_writel_dbi(pci, PCIE_RC_LCR, tmp);

		/*
		 * Start Directed Speed Change so the best possible
		 * speed both link partners support can be negotiated.
		 */
		tmp = dw_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
		tmp |= PORT_LOGIC_SPEED_CHANGE;
		dw_pcie_writel_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL, tmp);

		if (imx6_pcie->drvdata->flags &
		    IMX6_PCIE_FLAG_IMX6_SPEED_CHANGE) {
			/*
			 * On i.MX7, DIRECT_SPEED_CHANGE behaves differently
			 * from i.MX6 family when no link speed transition
			 * occurs and we go Gen1 -> yep, Gen1. The difference
			 * is that, in such case, it will not be cleared by HW
			 * which will cause the following code to report false
			 * failure.
			 */

			ret = imx6_pcie_wait_for_speed_change(imx6_pcie);
			if (ret) {
				dev_err(dev, "Failed to bring link up!\n");
				goto err_reset_phy;
			}
		}

		/* Make sure link training is finished as well! */
		ret = dw_pcie_wait_for_link(pci);
		if (ret) {
			dev_err(dev, "Failed to bring link up!\n");
			goto err_reset_phy;
		}
	} else {
		dev_info(dev, "Link: Gen2 disabled\n");
	}

	tmp = dw_pcie_readl_dbi(pci, PCIE_RC_LCSR);
	dev_info(dev, "Link up, Gen%i\n", (tmp >> 16) & 0xf);
	return 0;

err_reset_phy:
	dev_dbg(dev, "PHY DEBUG_R0=0x%08x DEBUG_R1=0x%08x\n",
		dw_pcie_readl_dbi(pci, PCIE_PORT_DEBUG0),
		dw_pcie_readl_dbi(pci, PCIE_PORT_DEBUG1));
	imx6_pcie_reset_phy(imx6_pcie);
	if (!IS_ENABLED(CONFIG_PCI_IMX6_COMPLIANCE_TEST)) {
		imx6_pcie_clk_disable(imx6_pcie);
		if (imx6_pcie->vpcie != NULL)
			regulator_disable(imx6_pcie->vpcie);
		if (imx6_pcie->epdev_on != NULL)
			regulator_disable(imx6_pcie->epdev_on);
	}

	return ret;
}

static void pci_imx_set_msi_en(struct pcie_port *pp)
{
	u16 val;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	if (pci_msi_enabled()) {
		dw_pcie_dbi_ro_wr_en(pci);
		val = dw_pcie_readw_dbi(pci, PCIE_RC_IMX6_MSI_CAP +
					PCI_MSI_FLAGS);
		val |= PCI_MSI_FLAGS_ENABLE;
		dw_pcie_writew_dbi(pci, PCIE_RC_IMX6_MSI_CAP + PCI_MSI_FLAGS,
				   val);
		dw_pcie_dbi_ro_wr_dis(pci);
	}
}

static int imx6_pcie_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct imx6_pcie *imx6_pcie = to_imx6_pcie(pci);

	if (gpio_is_valid(imx6_pcie->dis_gpio))
		gpio_set_value_cansleep(imx6_pcie->dis_gpio, 1);

	imx6_pcie_assert_core_reset(imx6_pcie);
	imx6_pcie_init_phy(imx6_pcie);
	imx6_pcie_deassert_core_reset(imx6_pcie);
	imx6_setup_phy_mpll(imx6_pcie);
	if (!(IS_ENABLED(CONFIG_EP_MODE_IN_EP_RC_SYS)
			&& (imx6_pcie->hard_wired == 0))) {
		dw_pcie_setup_rc(pp);
		pci_imx_set_msi_en(pp);
		if (imx6_pcie_establish_link(imx6_pcie))
			return -ENODEV;

		if (IS_ENABLED(CONFIG_PCI_MSI))
			dw_pcie_msi_init(pp);
	}

	return 0;
}

static const struct dw_pcie_host_ops imx6_pcie_host_ops = {
	.host_init = imx6_pcie_host_init,
};

static int imx6_add_pcie_port(struct imx6_pcie *imx6_pcie,
			      struct platform_device *pdev)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	struct pcie_port *pp = &pci->pp;
	struct device *dev = &pdev->dev;
	int ret;

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		pp->msi_irq = platform_get_irq_byname(pdev, "msi");
		if (pp->msi_irq <= 0) {
			dev_err(dev, "failed to get MSI irq\n");
			return -ENODEV;
		}
	}

	pp->ops = &imx6_pcie_host_ops;

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static u64 imx6_pcie_cpu_addr_fixup(struct dw_pcie *pcie, u64 cpu_addr)
{
	struct pcie_port *pp = &pcie->pp;
	struct imx6_pcie *imx6_pcie = to_imx6_pcie(pcie);

	if (imx6_pcie->drvdata->flags & IMX6_PCIE_FLAG_IMX6_CPU_ADDR_FIXUP)
		return (cpu_addr + imx6_pcie->local_addr - pp->mem_base);
	else
		return cpu_addr;
}

static const struct dw_pcie_ops dw_pcie_ops = {
	/* No special ops needed, but pcie-designware still expects this struct */
	.cpu_addr_fixup = imx6_pcie_cpu_addr_fixup,
};

static ssize_t ep_bar0_addr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct imx6_pcie *imx6_pcie = dev_get_drvdata(dev);
	struct dw_pcie *pci = imx6_pcie->pci;

	return sprintf(buf, "imx-pcie-bar0-addr-info start 0x%08x\n",
			readl(pci->dbi_base + PCI_BASE_ADDRESS_0));
}

static ssize_t ep_bar0_addr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	u32 bar_start;
	struct imx6_pcie *imx6_pcie = dev_get_drvdata(dev);
	struct dw_pcie *pci = imx6_pcie->pci;

	if (sscanf(buf, "%x\n", &bar_start) != 1)
		return -EINVAL;
	writel(bar_start, pci->dbi_base + PCI_BASE_ADDRESS_0);

	return count;
}

static void imx6_pcie_regions_setup(struct device *dev)
{
	struct imx6_pcie *imx6_pcie = dev_get_drvdata(dev);
	struct dw_pcie *pci = imx6_pcie->pci;
	struct pcie_port *pp = &pci->pp;

	dw_pcie_dbi_ro_wr_en(pci);

	switch (imx6_pcie->drvdata->variant) {
	case IMX8QM:
	case IMX8QXP:
	case IMX8MQ:
	case IMX8MM:
		/*
		 * RPMSG reserved 4Mbytes, but only used up to 2Mbytes.
		 * The left 2Mbytes can be used here.
		 */
		if (ddr_test_region == 0)
			dev_err(dev, "invalid ddr test region.\n");
		break;
	case IMX6SX:
	case IMX7D:
		ddr_test_region = 0xb0000000;
		break;

	case IMX6Q:
	case IMX6QP:
		ddr_test_region = 0x40000000;
		break;
	}
	dev_info(dev, "ddr_test_region is 0x%08x.\n", ddr_test_region);

	dw_pcie_prog_outbound_atu(pci, 0, 0, pp->mem_base,
				  ddr_test_region, test_region_size);
}

static DEVICE_ATTR_RW(ep_bar0_addr);

static struct attribute *imx6_pcie_ep_attrs[] = {
	&dev_attr_ep_bar0_addr.attr,
	NULL
};

static struct attribute_group imx6_pcie_attrgroup = {
	.attrs	= imx6_pcie_ep_attrs,
};

static void imx6_pcie_setup_ep(struct dw_pcie *pci, unsigned int bar_offset)
{
	int ret;
	u32 val;
	u32 lanes;
	struct device_node *np = pci->dev->of_node;

	ret = of_property_read_u32(np, "num-lanes", &lanes);
	if (ret)
		lanes = 0;

	/* set the number of lanes */
	val = dw_pcie_readl_dbi(pci, PCIE_PORT_LINK_CONTROL);
	val &= ~PORT_LINK_MODE_MASK;
	switch (lanes) {
	case 1:
		val |= PORT_LINK_MODE_1_LANES;
		break;
	default:
		dev_err(pci->dev, "num-lanes %u: invalid value\n", lanes);
		return;
	}
	dw_pcie_writel_dbi(pci, PCIE_PORT_LINK_CONTROL, val);

	/* set link width speed control register */
	val = dw_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
	val &= ~PORT_LOGIC_LINK_WIDTH_MASK;
	switch (lanes) {
	case 1:
		val |= PORT_LOGIC_LINK_WIDTH_1_LANES;
		break;
	default:
		dev_err(pci->dev, "num-lanes %u: invalid value\n", lanes);
		return;
	}
	dw_pcie_writel_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL, val);

	/* get iATU unroll support */
	val = dw_pcie_readl_dbi(pci, PCIE_ATU_VIEWPORT);
	if (val == 0xffffffff) {
		pci->iatu_unroll_enabled = 1;
		if (!pci->atu_base)
			pci->atu_base = pci->dbi_base + DEFAULT_DBI_ATU_OFFSET;
	}
	dev_info(pci->dev, "iATU unroll: %s\n",
		pci->iatu_unroll_enabled ? "enabled" : "disabled");

	/* CMD reg:I/O space, MEM space, and Bus Master Enable */
	writel(readl(pci->dbi_base + PCI_COMMAND)
			| PCI_COMMAND_IO
			| PCI_COMMAND_MEMORY
			| PCI_COMMAND_MASTER,
			pci->dbi_base + PCI_COMMAND);

	/*
	 * configure the class_rev(emaluate one memory ram ep device),
	 * bar0 and bar1 of ep
	 */
	writel(0xdeadbeaf, pci->dbi_base + PCI_VENDOR_ID);
	writel((readl(pci->dbi_base + PCI_CLASS_REVISION) & 0xFFFF)
			| (PCI_CLASS_MEMORY_RAM	<< 16),
			pci->dbi_base + PCI_CLASS_REVISION);
	writel(0xdeadbeaf, pci->dbi_base
			+ PCI_SUBSYSTEM_VENDOR_ID);

	/* 32bit none-prefetchable 8M bytes memory on bar0 */
	writel(0x0, pci->dbi_base + PCI_BASE_ADDRESS_0);
	writel(SZ_8M - 1, pci->dbi_base + bar_offset
			+ PCI_BASE_ADDRESS_0);

	/* None used bar1 */
	writel(0x0, pci->dbi_base + PCI_BASE_ADDRESS_1);
	writel(0, pci->dbi_base + bar_offset + PCI_BASE_ADDRESS_1);

	/* 4K bytes IO on bar2 */
	writel(0x1, pci->dbi_base + PCI_BASE_ADDRESS_2);
	writel(SZ_4K - 1, pci->dbi_base + bar_offset +
			PCI_BASE_ADDRESS_2);

	/*
	 * 32bit prefetchable 1M bytes memory on bar3
	 * FIXME BAR MASK3 is not changeable, the size
	 * is fixed to 256 bytes.
	 */
	writel(0x8, pci->dbi_base + PCI_BASE_ADDRESS_3);
	writel(SZ_1M - 1, pci->dbi_base + bar_offset
			+ PCI_BASE_ADDRESS_3);

	/*
	 * 64bit prefetchable 1M bytes memory on bar4-5.
	 * FIXME BAR4,5 are not enabled yet
	 */
	writel(0xc, pci->dbi_base + PCI_BASE_ADDRESS_4);
	writel(SZ_1M - 1, pci->dbi_base + bar_offset
			+ PCI_BASE_ADDRESS_4);
	writel(0, pci->dbi_base + bar_offset + PCI_BASE_ADDRESS_5);
}

static irqreturn_t imx6_pcie_dma_isr(int irq, void *param)
{
	u32 irqs, offset;
	struct pcie_port *pp = (struct pcie_port *)param;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct imx6_pcie *imx6_pcie = to_imx6_pcie(pci);

	offset = imx6_pcie->dma_unroll_offset;

	/* check write isr */
	irqs = readl(pci->dbi_base + offset + DMA_WRITE_INT_STS);
	if (irqs & DMA_DONE_INT_STS) {
		/* write 1 clear */
		writel(irqs & DMA_DONE_INT_STS,
				pci->dbi_base + offset + DMA_WRITE_INT_CLR);
		dma_w_end = 1;
	} else if (irqs & DMA_ABORT_INT_STS) {
		pr_info("imx pcie dma write error 0x%0x.\n", irqs);
	}
	/* check read isr */
	irqs = readl(pci->dbi_base + offset + DMA_READ_INT_STS);
	if (irqs & DMA_DONE_INT_STS) {
		/* write 1 clear */
		writel(irqs & DMA_DONE_INT_STS,
				pci->dbi_base + offset + DMA_READ_INT_CLR);
		dma_r_end = 1;
	} else if (irqs & DMA_ABORT_INT_STS) {
		pr_info("imx pcie dma read error 0x%0x.", irqs);
	}
	return IRQ_HANDLED;
}

/**
 * imx6_pcie_local_dma_start - Start one local iMX PCIE DMA.
 * @pp: the port start the dma transmission.
 * @dir: direction of the dma, 1 read, 0 write;
 * @chl: the channel num of the iMX PCIE DMA(0 - 7).
 * @src: source DMA address.
 * @dst: destination DMA address.
 * @len: transfer length.
 */
static int imx6_pcie_local_dma_start(struct pcie_port *pp, bool dir,
		unsigned int chl, dma_addr_t src, dma_addr_t dst,
		unsigned int len)
{
	u32 offset, doorbell, unroll_cal;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct imx6_pcie *imx6_pcie = to_imx6_pcie(pci);

	if (pp == NULL)
		return -EINVAL;
	if (chl > MAX_PCIE_DMA_CHANNELS)
		return -EINVAL;

	offset = imx6_pcie->dma_unroll_offset;
	/* enable dma engine, dir 1:read. 0:write. */
	if (dir)
		writel(DMA_READ_ENGINE_EN,
				pci->dbi_base + offset
				+ DMA_READ_ENGINE_EN_OFF);
	else
		writel(DMA_WRITE_ENGINE_EN,
				pci->dbi_base + offset
				+ DMA_WRITE_ENGINE_EN_OFF);
	writel(0x0, pci->dbi_base + offset + DMA_WRITE_INT_MASK);
	writel(0x0, pci->dbi_base + offset + DMA_READ_INT_MASK);
	 /* ch dir and ch num */
	if (offset == 0) {
		writel((dir << 31) | chl, pci->dbi_base + DMA_VIEWPOT_SEL_OFF);
		writel(DMA_CHANNEL_CTRL_1_LIE,
				pci->dbi_base + DMA_CHANNEL_CTRL_1);
		writel(0x0, pci->dbi_base + DMA_CHANNEL_CTRL_2);
		writel(len, pci->dbi_base + DMA_TRANSFER_SIZE);
		writel((u32)src, pci->dbi_base + DMA_SAR_LOW);
		writel(0x0, pci->dbi_base + DMA_SAR_HIGH);
		writel((u32)dst, pci->dbi_base + DMA_DAR_LOW);
		writel(0x0, pci->dbi_base + DMA_DAR_HIGH);
	} else {
		unroll_cal = DMA_UNROLL_CDM_OFFSET
			+ 0x200 * (chl + 1) + 0x100 * dir;
		writel(DMA_CHANNEL_CTRL_1_LIE, pci->dbi_base + unroll_cal);
		writel(0x0, pci->dbi_base + unroll_cal + 0x4);
		writel(len, pci->dbi_base + unroll_cal + 0x8);
		writel((u32)src, pci->dbi_base + unroll_cal + 0xc);
		writel(0x0, pci->dbi_base + unroll_cal + 0x10);
		writel((u32)dst, pci->dbi_base + unroll_cal + 0x14);
		writel(0x0, pci->dbi_base + unroll_cal + 0x18);
	}

	doorbell = dir ? DMA_READ_DOORBELL : DMA_WRITE_DOORBELL;
	writel(chl, pci->dbi_base + offset + doorbell);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static void imx6_pcie_ltssm_disable(struct device *dev)
{
	u32 val;
	struct imx6_pcie *imx6_pcie = dev_get_drvdata(dev);

	switch (imx6_pcie->drvdata->variant) {
	case IMX6SX:
	case IMX6QP:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6Q_GPR12_PCIE_CTL_2, 0);
		break;
	case IMX7D:
	case IMX8MQ:
	case IMX8MM:
		reset_control_assert(imx6_pcie->apps_reset);
		break;
	case IMX8QXP:
	case IMX8QM:
		/* Bit4 of the CTRL2 */
		val = IMX8QM_CSR_PCIEA_OFFSET
			+ imx6_pcie->controller_id * SZ_64K;
		regmap_update_bits(imx6_pcie->iomuxc_gpr,
				val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
				IMX8QM_CTRL_LTSSM_ENABLE, 0);
		regmap_update_bits(imx6_pcie->iomuxc_gpr,
				val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
				IMX8QM_CTRL_READY_ENTR_L23, 0);
		break;
	default:
		dev_err(dev, "ltssm_disable not supported\n");
	}
}

static void imx6_pcie_pm_turnoff(struct imx6_pcie *imx6_pcie)
{
	int i;
	u32 dst, val;
	struct device *dev = imx6_pcie->pci->dev;

	/* Some variants have a turnoff reset in DT */
	if (imx6_pcie->turnoff_reset) {
		reset_control_assert(imx6_pcie->turnoff_reset);
		reset_control_deassert(imx6_pcie->turnoff_reset);
		goto pm_turnoff_sleep;
	}

	/* Others poke directly at IOMUXC registers */
	switch (imx6_pcie->drvdata->variant) {
	case IMX6SX:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				IMX6SX_GPR12_PCIE_PM_TURN_OFF,
				IMX6SX_GPR12_PCIE_PM_TURN_OFF);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				IMX6SX_GPR12_PCIE_PM_TURN_OFF, 0);
		break;
	case IMX6QP:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6SX_GPR12_PCIE_PM_TURN_OFF,
				   IMX6SX_GPR12_PCIE_PM_TURN_OFF);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6SX_GPR12_PCIE_PM_TURN_OFF, 0);
		break;
	case IMX8QXP:
	case IMX8QM:
		dst = IMX8QM_CSR_PCIEA_OFFSET + imx6_pcie->controller_id * SZ_64K;
		regmap_update_bits(imx6_pcie->iomuxc_gpr,
				dst + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
				IMX8QM_CTRL_PM_XMT_TURNOFF,
				IMX8QM_CTRL_PM_XMT_TURNOFF);
		regmap_update_bits(imx6_pcie->iomuxc_gpr,
				dst + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
				IMX8QM_CTRL_PM_XMT_TURNOFF,
				0);
		regmap_update_bits(imx6_pcie->iomuxc_gpr,
				dst + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
				IMX8QM_CTRL_READY_ENTR_L23,
				IMX8QM_CTRL_READY_ENTR_L23);
		/* check the L2 is entered or not. */
		for (i = 0; i < 10000; i++) {
			regmap_read(imx6_pcie->iomuxc_gpr,
					dst + IMX8QM_CSR_PCIE_STTS0_OFFSET,
					&val);
			if (val & IMX8QM_CTRL_STTS0_PM_LINKST_IN_L2)
				break;
			udelay(10);
		}
		if ((val & IMX8QM_CTRL_STTS0_PM_LINKST_IN_L2) == 0)
			dev_err(dev, "PCIE%d can't enter into L2.\n",
					imx6_pcie->controller_id);
		break;
	default:
		dev_err(dev, "PME_Turn_Off not implemented\n");
		return;
	}

	/*
	 * Components with an upstream port must respond to
	 * PME_Turn_Off with PME_TO_Ack but we can't check.
	 *
	 * The standard recommends a 1-10ms timeout after which to
	 * proceed anyway as if acks were received.
	 */
pm_turnoff_sleep:
	usleep_range(1000, 10000);
}

static int imx6_pcie_suspend_noirq(struct device *dev)
{
	struct imx6_pcie *imx6_pcie = dev_get_drvdata(dev);

	if (!(imx6_pcie->drvdata->flags & IMX6_PCIE_FLAG_SUPPORTS_SUSPEND))
		return 0;
	if (unlikely(imx6_pcie->drvdata->variant == IMX6Q)) {
		/*
		 * L2 can exit by 'reset' or Inband beacon (from remote EP)
		 * toggling phy_powerdown has same effect as 'inband beacon'
		 * So, toggle bit18 of GPR1, used as a workaround of errata
		 * ERR005723 "PCIe PCIe does not support L2 Power Down"
		 */
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_TEST_PD,
				   IMX6Q_GPR1_PCIE_TEST_PD);
	} else {
		imx6_pcie_pm_turnoff(imx6_pcie);
		imx6_pcie_ltssm_disable(dev);
		imx6_pcie_clk_disable(imx6_pcie);
	}

	return 0;
}

static int imx6_pcie_resume_noirq(struct device *dev)
{
	int ret;
	struct imx6_pcie *imx6_pcie = dev_get_drvdata(dev);
	struct pcie_port *pp = &imx6_pcie->pci->pp;

	if (!(imx6_pcie->drvdata->flags & IMX6_PCIE_FLAG_SUPPORTS_SUSPEND))
		return 0;
	if (unlikely(imx6_pcie->drvdata->variant == IMX6Q)) {
		/*
		 * L2 can exit by 'reset' or Inband beacon (from remote EP)
		 * toggling phy_powerdown has same effect as 'inband beacon'
		 * So, toggle bit18 of GPR1, used as a workaround of errata
		 * ERR005723 "PCIe PCIe does not support L2 Power Down"
		 */
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				IMX6Q_GPR1_PCIE_TEST_PD, 0);
	} else {
		imx6_pcie_assert_core_reset(imx6_pcie);
		imx6_pcie_init_phy(imx6_pcie);
		imx6_pcie_deassert_core_reset(imx6_pcie);
		dw_pcie_setup_rc(pp);
		pci_imx_set_msi_en(pp);

		ret = imx6_pcie_establish_link(imx6_pcie);
		if (ret < 0)
			dev_info(dev, "pcie link is down after resume.\n");
	}

	return 0;
}
#endif

static const struct dev_pm_ops imx6_pcie_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(imx6_pcie_suspend_noirq,
				      imx6_pcie_resume_noirq)
};

static int imx6_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dw_pcie *pci;
	struct imx6_pcie *imx6_pcie;
	struct device_node *np;
	struct resource *dbi_base, *hsio_res;
	struct device_node *node = dev->of_node;
	void __iomem *iomem;
	struct regmap_config regconfig = imx6_pcie_regconfig;
	int ret;

	imx6_pcie = devm_kzalloc(dev, sizeof(*imx6_pcie), GFP_KERNEL);
	if (!imx6_pcie)
		return -ENOMEM;

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	pci->dev = dev;
	pci->ops = &dw_pcie_ops;

	imx6_pcie->pci = pci;
	imx6_pcie->drvdata = of_device_get_match_data(dev);

	/* Find the PHY if one is defined, only imx7d uses it */
	np = of_parse_phandle(node, "fsl,imx7d-pcie-phy", 0);
	if (np) {
		struct resource res;

		ret = of_address_to_resource(np, 0, &res);
		if (ret) {
			dev_err(dev, "Unable to map PCIe PHY\n");
			return ret;
		}
		imx6_pcie->phy_base = devm_ioremap_resource(dev, &res);
		if (IS_ERR(imx6_pcie->phy_base)) {
			dev_err(dev, "Unable to map PCIe PHY\n");
			return PTR_ERR(imx6_pcie->phy_base);
		}
	}

	dbi_base = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pci->dbi_base = devm_ioremap_resource(dev, dbi_base);
	if (IS_ERR(pci->dbi_base))
		return PTR_ERR(pci->dbi_base);

	if (of_property_read_u32(node, "hsio-cfg", &imx6_pcie->hsio_cfg))
		imx6_pcie->hsio_cfg = 0;
	if (of_property_read_u32(node, "ext_osc", &imx6_pcie->ext_osc) < 0)
		imx6_pcie->ext_osc = 0;

	if (of_property_read_u32(node, "local-addr", &imx6_pcie->local_addr))
		imx6_pcie->local_addr = 0;
	if (of_property_read_u32(node, "hard-wired", &imx6_pcie->hard_wired))
		imx6_pcie->hard_wired = 0;

	np = of_parse_phandle(node, "reserved-region", 0);
	if (np) {
		struct resource res;

		if (of_address_to_resource(np, 0, &res)) {
			dev_err(dev, "failed to get reserved region address\n");
			of_node_put(np);
			return -EINVAL;
		}
		ddr_test_region = res.start + SZ_2M;
		of_node_put(np);
	}

	/* Fetch GPIOs */
	imx6_pcie->clkreq_gpio = of_get_named_gpio(node, "clkreq-gpio", 0);
	if (gpio_is_valid(imx6_pcie->clkreq_gpio)) {
		devm_gpio_request_one(&pdev->dev, imx6_pcie->clkreq_gpio,
				      GPIOF_OUT_INIT_LOW, "PCIe CLKREQ");
	} else if (imx6_pcie->clkreq_gpio == -EPROBE_DEFER) {
		return imx6_pcie->clkreq_gpio;
	}

	imx6_pcie->dis_gpio = of_get_named_gpio(node, "disable-gpio", 0);
	if (gpio_is_valid(imx6_pcie->dis_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev, imx6_pcie->dis_gpio,
					    GPIOF_OUT_INIT_LOW, "PCIe DIS");
		if (ret) {
			dev_err(&pdev->dev, "unable to get disable gpio\n");
			return ret;
		}
	} else if (imx6_pcie->dis_gpio == -EPROBE_DEFER) {
		return imx6_pcie->dis_gpio;
	}

	imx6_pcie->epdev_on = devm_regulator_get(&pdev->dev,
						 "epdev_on");
	if (IS_ERR(imx6_pcie->epdev_on))
		return -EPROBE_DEFER;

	imx6_pcie->reset_gpio = of_get_named_gpio(node, "reset-gpio", 0);
	imx6_pcie->gpio_active_high = of_property_read_bool(node,
						"reset-gpio-active-high");
	if (gpio_is_valid(imx6_pcie->reset_gpio)) {
		ret = devm_gpio_request_one(dev, imx6_pcie->reset_gpio,
				imx6_pcie->gpio_active_high ?
					GPIOF_OUT_INIT_HIGH :
					GPIOF_OUT_INIT_LOW,
				"PCIe reset");
		if (ret) {
			dev_err(dev, "unable to get reset gpio\n");
			return ret;
		}
	} else if (imx6_pcie->reset_gpio == -EPROBE_DEFER) {
		return imx6_pcie->reset_gpio;
	}

	/* Fetch clocks */
	imx6_pcie->pcie_phy = devm_clk_get(dev, "pcie_phy");
	if (IS_ERR(imx6_pcie->pcie_phy)) {
		dev_err(dev, "pcie_phy clock source missing or invalid\n");
		return PTR_ERR(imx6_pcie->pcie_phy);
	}

	imx6_pcie->pcie_bus = devm_clk_get(dev, "pcie_bus");
	if (IS_ERR(imx6_pcie->pcie_bus)) {
		dev_err(dev, "pcie_bus clock source missing or invalid\n");
		return PTR_ERR(imx6_pcie->pcie_bus);
	}

	imx6_pcie->pcie = devm_clk_get(dev, "pcie");
	if (IS_ERR(imx6_pcie->pcie)) {
		dev_err(dev, "pcie clock source missing or invalid\n");
		return PTR_ERR(imx6_pcie->pcie);
	}

	switch (imx6_pcie->drvdata->variant) {
	case IMX6SX:
		imx6_pcie->pcie_inbound_axi = devm_clk_get(dev,
							   "pcie_inbound_axi");
		if (IS_ERR(imx6_pcie->pcie_inbound_axi)) {
			dev_err(dev, "pcie_inbound_axi clock missing or invalid\n");
			return PTR_ERR(imx6_pcie->pcie_inbound_axi);
		}
		break;
	case IMX8MQ:
	case IMX8MM:
		imx6_pcie->pcie_aux = devm_clk_get(dev, "pcie_aux");
		if (IS_ERR(imx6_pcie->pcie_aux)) {
			dev_err(dev, "pcie_aux clock source missing or invalid\n");
			return PTR_ERR(imx6_pcie->pcie_aux);
		}
		/* fall through */
	case IMX7D:
		if (dbi_base->start == IMX8MQ_PCIE2_BASE_ADDR)
			imx6_pcie->controller_id = 1;

		imx6_pcie->pciephy_reset = devm_reset_control_get_exclusive(dev,
									    "pciephy");
		if (IS_ERR(imx6_pcie->pciephy_reset)) {
			dev_err(dev, "Failed to get PCIEPHY reset control\n");
			return PTR_ERR(imx6_pcie->pciephy_reset);
		}

		imx6_pcie->apps_reset = devm_reset_control_get_exclusive(dev,
									 "apps");
		if (IS_ERR(imx6_pcie->apps_reset)) {
			dev_err(dev, "Failed to get PCIE APPS reset control\n");
			return PTR_ERR(imx6_pcie->apps_reset);
		}
		break;
	case IMX8QM:
	case IMX8QXP:
		if (dbi_base->start == IMX8_HSIO_PCIEB_BASE_ADDR)
			imx6_pcie->controller_id = 1;

		imx6_pcie->pcie_per = devm_clk_get(dev, "pcie_per");
		if (IS_ERR(imx6_pcie->pcie_per)) {
			dev_err(dev, "pcie_per clock source missing or invalid\n");
			return PTR_ERR(imx6_pcie->pcie_per);
		}

		imx6_pcie->pcie_inbound_axi = devm_clk_get(&pdev->dev,
				"pcie_inbound_axi");
		if (IS_ERR(imx6_pcie->pcie_inbound_axi)) {
			dev_err(&pdev->dev,
				"pcie clock source missing or invalid\n");
			return PTR_ERR(imx6_pcie->pcie_inbound_axi);
		}

		imx6_pcie->phy_per = devm_clk_get(dev, "phy_per");
		if (IS_ERR(imx6_pcie->phy_per)) {
			dev_err(dev, "failed to get per clock.\n");
			return PTR_ERR(imx6_pcie->phy_per);
		}

		imx6_pcie->misc_per = devm_clk_get(dev, "misc_per");
		if (IS_ERR(imx6_pcie->misc_per)) {
			dev_err(dev, "failed to get per clock.\n");
			return PTR_ERR(imx6_pcie->misc_per);
		}

		hsio_res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
							"hsio");
		if (hsio_res) {
			iomem = devm_ioremap_resource(dev, hsio_res);
			if (IS_ERR(iomem))
				return PTR_ERR(iomem);
			imx6_pcie->iomuxc_gpr =
				devm_regmap_init_mmio(dev, iomem, &regconfig);
			if (IS_ERR(imx6_pcie->iomuxc_gpr)) {
				dev_err(dev, "failed to init register map\n");
				return PTR_ERR(imx6_pcie->iomuxc_gpr);
			}
		} else {
			dev_err(dev, "missing *hsio* reg space\n");
		}
		break;
	default:
		break;
	}

	/* Grab turnoff reset */
	imx6_pcie->turnoff_reset = devm_reset_control_get_optional_exclusive(dev, "turnoff");
	if (IS_ERR(imx6_pcie->turnoff_reset)) {
		dev_err(dev, "Failed to get TURNOFF reset control\n");
		return PTR_ERR(imx6_pcie->turnoff_reset);
	}

	imx6_pcie->clkreq_reset = devm_reset_control_get_optional_exclusive(dev, "clkreq");
	if (IS_ERR(imx6_pcie->clkreq_reset)) {
		dev_err(dev, "Failed to get CLKREQ reset control\n");
		return PTR_ERR(imx6_pcie->clkreq_reset);
	}

	/* Grab GPR config register range */
	if (imx6_pcie->iomuxc_gpr == NULL) {
		imx6_pcie->iomuxc_gpr =
			syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
		if (IS_ERR(imx6_pcie->iomuxc_gpr)) {
			dev_err(dev, "unable to find iomuxc registers\n");
			return PTR_ERR(imx6_pcie->iomuxc_gpr);
		}
	}

	/* Grab PCIe PHY Tx Settings */
	if (of_property_read_u32(node, "fsl,tx-deemph-gen1",
				 &imx6_pcie->tx_deemph_gen1))
		imx6_pcie->tx_deemph_gen1 = 0;

	if (of_property_read_u32(node, "fsl,tx-deemph-gen2-3p5db",
				 &imx6_pcie->tx_deemph_gen2_3p5db))
		imx6_pcie->tx_deemph_gen2_3p5db = 0;

	if (of_property_read_u32(node, "fsl,tx-deemph-gen2-6db",
				 &imx6_pcie->tx_deemph_gen2_6db))
		imx6_pcie->tx_deemph_gen2_6db = 20;

	if (of_property_read_u32(node, "fsl,tx-swing-full",
				 &imx6_pcie->tx_swing_full))
		imx6_pcie->tx_swing_full = 127;

	if (of_property_read_u32(node, "fsl,tx-swing-low",
				 &imx6_pcie->tx_swing_low))
		imx6_pcie->tx_swing_low = 127;

	/* Limit link speed */
	ret = of_property_read_u32(node, "fsl,max-link-speed",
				   &imx6_pcie->link_gen);
	if (ret)
		imx6_pcie->link_gen = 1;

	imx6_pcie->vpcie = devm_regulator_get_optional(&pdev->dev, "vpcie");
	if (IS_ERR(imx6_pcie->vpcie)) {
		if (PTR_ERR(imx6_pcie->vpcie) != -ENODEV)
			return PTR_ERR(imx6_pcie->vpcie);
		imx6_pcie->vpcie = NULL;
	}

	platform_set_drvdata(pdev, imx6_pcie);

	ret = imx6_pcie_attach_pd(dev);
	if (ret)
		return ret;

	ret = regulator_enable(imx6_pcie->epdev_on);
	if (ret)
		dev_err(dev, "failed to enable the epdev_on regulator\n");

	if (IS_ENABLED(CONFIG_EP_MODE_IN_EP_RC_SYS)
			&& (imx6_pcie->hard_wired == 0)) {
		int i = 0, irq;
		u32 val, tv_count1, tv_count2;
		dma_addr_t test_reg1_dma, test_reg2_dma;
		void *test_reg1, *test_reg2;
		void __iomem *pcie_arb_base_addr;
		struct timespec64 tv1s, tv1e, tv2s, tv2e;
		struct resource_entry *win, *tmp;
		LIST_HEAD(res);
		struct pcie_port *pp = &pci->pp;
		unsigned long timeout = jiffies + msecs_to_jiffies(300000);

		/* add attributes for device */
		imx6_pcie_attrgroup.attrs = imx6_pcie_ep_attrs;
		ret = sysfs_create_group(&pdev->dev.kobj, &imx6_pcie_attrgroup);
		if (ret)
			return -EINVAL;

		ret = devm_of_pci_get_host_bridge_resources(dev, 0, 0xff, &res,
							    &pp->io_base);
		if (ret)
			return ret;

		ret = devm_request_pci_bus_resources(&pdev->dev, &res);
		if (ret) {
			dev_err(dev, "missing ranges property\n");
			pci_free_resource_list(&res);
			return ret;
		}

		/* Get the I/O and memory ranges from DT */
		resource_list_for_each_entry_safe(win, tmp, &res) {
			switch (resource_type(win->res)) {
			case IORESOURCE_MEM:
				pp->mem = win->res;
				pp->mem->name = "MEM";
				pp->mem_size = resource_size(pp->mem);
				pp->mem_bus_addr = pp->mem->start - win->offset;
				break;
			}
		}

		pp->mem_base = pp->mem->start;
		pp->ops = &imx6_pcie_host_ops;
		dev_info(dev, " try to initialize pcie ep.\n");
		ret = imx6_pcie_host_init(pp);
		if (ret) {
			dev_info(dev, " fail to initialize pcie ep.\n");
			return ret;
		}

		dw_pcie_dbi_ro_wr_en(pci);
		switch (imx6_pcie->drvdata->variant) {
		case IMX8MQ:
		case IMX8MM:
			imx6_pcie_setup_ep(pci, SZ_1M);
			break;
		default:
			imx6_pcie_setup_ep(pci, SZ_4K);
			break;
		}
		pci_imx_set_msi_en(pp);
		platform_set_drvdata(pdev, imx6_pcie);
		imx6_pcie_regions_setup(dev);

		/*
		 * iMX6SX PCIe has the stand-alone power domain.
		 * refer to the initialization for iMX6SX PCIe,
		 * release the PCIe PHY reset here,
		 * before LTSSM enable is set.
		 */
		if (imx6_pcie->drvdata->variant == IMX6SX)
			regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR5,
					BIT(19), 0 << 19);

		/* assert LTSSM enable */
		imx6_pcie_ltssm_enable(dev);

		dev_info(dev, "PCIe EP: waiting for link up...\n");
		/* link is indicated by the bit4 of DB_R1 register */
		do {
			usleep_range(10, 20);
			if (time_after(jiffies, timeout)) {
				dev_info(dev, "PCIe EP: link down.\n");
				return 0;
			}
			val = readl(pci->dbi_base + PCIE_PHY_DEBUG_R1);
		} while ((val & 0x10) == 0);

		/* self io test */
		/* Check the DMA INT exist or not */
		irq =  platform_get_irq_byname(pdev, "dma");
		if (irq > 0)
			dma_en = 1;
		else
			dma_en = 0;
		if (dma_en) {
			/* configure the DMA INT ISR */
			ret = request_irq(irq, imx6_pcie_dma_isr,
					  IRQF_SHARED, "imx-pcie-dma", pp);
			if (ret) {
				pr_err("register interrupt %d failed, rc %d\n",
					irq, ret);
				dma_en = 0;
			}
			test_reg1 = dma_alloc_coherent(dev, test_region_size,
					&test_reg1_dma, GFP_KERNEL);
			test_reg2 = dma_alloc_coherent(dev, test_region_size,
					&test_reg2_dma, GFP_KERNEL);
			if (!(test_reg1 && test_reg2))
				dma_en = 0; /* Roll back to PIO. */
			dma_r_end = dma_w_end = 0;

			val = readl(pci->dbi_base + DMA_CTRL_VIEWPORT_OFF);
			if (val == 0xffffffff)
				imx6_pcie->dma_unroll_offset =
					DMA_UNROLL_CDM_OFFSET - DMA_REG_OFFSET;
			else
				imx6_pcie->dma_unroll_offset = 0;
		}

		if (unlikely(dma_en == 0)) {
			test_reg1 = devm_kzalloc(&pdev->dev,
					test_region_size, GFP_KERNEL);
			if (!test_reg1) {
				ret = -ENOMEM;
				return ret;
			}

			test_reg2 = devm_kzalloc(&pdev->dev,
					test_region_size, GFP_KERNEL);
			if (!test_reg2) {
				ret = -ENOMEM;
				return ret;
			}
		}

		pcie_arb_base_addr = ioremap_nocache(pp->mem_base,
					test_region_size);
		if (!pcie_arb_base_addr) {
			dev_err(dev, "ioremap error in ep io test\n");
			ret = -ENOMEM;
			return ret;
		}

		for (i = 0; i < test_region_size; i = i + 4) {
			writel(0xE6600D00 + i, test_reg1 + i);
			writel(0xDEADBEAF, test_reg2 + i);
		}

		/* PCIe EP start the data transfer after link up */
		dev_info(dev, "pcie ep: Starting data transfer...\n");
		ktime_get_real_ts64(&tv1s);


		/* EP write the test region to remote RC's DDR memory */
		if (dma_en) {
			imx6_pcie_local_dma_start(pp, 0, 0, test_reg1_dma,
			imx6_pcie_cpu_addr_fixup(imx6_pcie->pci, pp->mem_base),
					test_region_size);
			timeout = jiffies + msecs_to_jiffies(300);
			do {
				udelay(1);
				if (time_after(jiffies, timeout)) {
					dev_info(dev, "dma write no end ...\n");
					break;
				}
			} while (!dma_w_end);
		} else {
			memcpy((unsigned int *)pcie_arb_base_addr,
					(unsigned int *)test_reg1,
					test_region_size);
		}

		ktime_get_real_ts64(&tv1e);

		ktime_get_real_ts64(&tv2s);
		/* EP read the test region back from remote RC's DDR memory */
		if (dma_en) {
			imx6_pcie_local_dma_start(pp, 1, 0,
			imx6_pcie_cpu_addr_fixup(imx6_pcie->pci, pp->mem_base),
					test_reg2_dma, test_region_size);
			timeout = jiffies + msecs_to_jiffies(300);
			do {
				udelay(1);
				if (time_after(jiffies, timeout)) {
					dev_info(dev, "dma read no end\n");
					break;
				}
			} while (!dma_r_end);
		} else {
			memcpy((unsigned int *)test_reg2,
					(unsigned int *)pcie_arb_base_addr,
					test_region_size);
		}

		ktime_get_real_ts64(&tv2e);
		if (memcmp(test_reg2, test_reg1, test_region_size) == 0) {
			tv_count1 = (tv1e.tv_sec - tv1s.tv_sec)
				* USEC_PER_SEC
				+ (tv1e.tv_nsec - tv1s.tv_nsec) / 1000;
			tv_count2 = (tv2e.tv_sec - tv2s.tv_sec)
				* USEC_PER_SEC
				+ (tv2e.tv_nsec - tv2s.tv_nsec) / 1000;

			dev_info(dev, "ep: Data %s transfer is successful.\n",
					dma_en ? "DMA" : "PIO");
			dev_info(dev, "ep: Data write %dus speed:%ldMB/s.\n",
					tv_count1,
					((test_region_size/1024)
					   * MSEC_PER_SEC)
					/(tv_count1));
			dev_info(dev, "ep: Data read %dus speed:%ldMB/s.\n",
					tv_count2,
					((test_region_size/1024)
					   * MSEC_PER_SEC)
					/(tv_count2));
		} else {
			dev_info(dev, "ep: Data transfer is failed.\n");
		} /* end of self io test. */
	} else {
		ret = imx6_add_pcie_port(imx6_pcie, pdev);
		if (ret < 0) {
			if (IS_ENABLED(CONFIG_PCI_IMX6_COMPLIANCE_TEST)) {
				/* The PCIE clocks wouldn't be turned off */
				dev_info(dev, "To do the compliance tests.\n");
				ret = 0;
			} else {
				dev_err(dev, "unable to add pcie port.\n");
			}
			return ret;
		}
		pci_imx_set_msi_en(&imx6_pcie->pci->pp);

		if (IS_ENABLED(CONFIG_RC_MODE_IN_EP_RC_SYS)
				&& (imx6_pcie->hard_wired == 0))
			imx6_pcie_regions_setup(&pdev->dev);

		/*
		 * If the L1SS is enabled, disable the over ride after link up.
		 * Let the the CLK_REQ# controlled by HW L1SS automatically.
		 */
		ret = imx6_pcie->drvdata->flags & IMX6_PCIE_FLAG_SUPPORTS_L1SS;
		if (IS_ENABLED(CONFIG_PCIEASPM_POWER_SUPERSAVE) && (ret > 0)) {
			switch (imx6_pcie->drvdata->variant) {
			case IMX8MQ:
			case IMX8MM:
				regmap_update_bits(imx6_pcie->iomuxc_gpr,
					imx6_pcie_grp_offset(imx6_pcie),
					IMX8MQ_GPR_PCIE_CLK_REQ_OVERRIDE_EN,
					0);
				break;
			default:
				break;
			};
		}
	}

	return 0;
}

static void imx6_pcie_shutdown(struct platform_device *pdev)
{
	struct imx6_pcie *imx6_pcie = platform_get_drvdata(pdev);

	/* bring down link, so bootloader gets clean state in case of reboot */
	imx6_pcie_assert_core_reset(imx6_pcie);
}

static const struct imx6_pcie_drvdata drvdata[] = {
	[IMX6Q] = {
		.variant = IMX6Q,
		.flags = IMX6_PCIE_FLAG_IMX6_PHY |
			 IMX6_PCIE_FLAG_SUPPORTS_SUSPEND |
			 IMX6_PCIE_FLAG_IMX6_SPEED_CHANGE,
		.dbi_length = 0x200,
	},
	[IMX6SX] = {
		.variant = IMX6SX,
		.flags = IMX6_PCIE_FLAG_IMX6_PHY |
			 IMX6_PCIE_FLAG_IMX6_SPEED_CHANGE |
			 IMX6_PCIE_FLAG_SUPPORTS_SUSPEND,
	},
	[IMX6QP] = {
		.variant = IMX6QP,
		.flags = IMX6_PCIE_FLAG_IMX6_PHY |
			 IMX6_PCIE_FLAG_IMX6_SPEED_CHANGE |
			 IMX6_PCIE_FLAG_SUPPORTS_SUSPEND,
	},
	[IMX7D] = {
		.variant = IMX7D,
		.flags = IMX6_PCIE_FLAG_SUPPORTS_SUSPEND,
	},
	[IMX8MQ] = {
		.variant = IMX8MQ,
		.flags = IMX6_PCIE_FLAG_SUPPORTS_SUSPEND |
			 IMX6_PCIE_FLAG_SUPPORTS_L1SS,
	},
	[IMX8MM] = {
		.variant = IMX8MM,
		.flags = IMX6_PCIE_FLAG_SUPPORTS_SUSPEND |
			 IMX6_PCIE_FLAG_SUPPORTS_L1SS,
	},
	[IMX8QM] = {
		.variant = IMX8QM,
		.flags = IMX6_PCIE_FLAG_SUPPORTS_SUSPEND |
			 IMX6_PCIE_FLAG_IMX6_CPU_ADDR_FIXUP,
	},
	[IMX8QXP] = {
		.variant = IMX8QXP,
		.flags = IMX6_PCIE_FLAG_SUPPORTS_SUSPEND |
			 IMX6_PCIE_FLAG_IMX6_CPU_ADDR_FIXUP,
	},
};

static const struct of_device_id imx6_pcie_of_match[] = {
	{ .compatible = "fsl,imx6q-pcie",  .data = &drvdata[IMX6Q],  },
	{ .compatible = "fsl,imx6sx-pcie", .data = &drvdata[IMX6SX], },
	{ .compatible = "fsl,imx6qp-pcie", .data = &drvdata[IMX6QP], },
	{ .compatible = "fsl,imx7d-pcie",  .data = &drvdata[IMX7D],  },
	{ .compatible = "fsl,imx8mq-pcie", .data = &drvdata[IMX8MQ], },
	{ .compatible = "fsl,imx8mm-pcie", .data = &drvdata[IMX8MM], },
	{ .compatible = "fsl,imx8qm-pcie", .data = &drvdata[IMX8QM], },
	{ .compatible = "fsl,imx8qxp-pcie", .data = &drvdata[IMX8QXP], },
	{},
};

static struct platform_driver imx6_pcie_driver = {
	.driver = {
		.name	= "imx6q-pcie",
		.of_match_table = imx6_pcie_of_match,
		.suppress_bind_attrs = true,
		.pm = &imx6_pcie_pm_ops,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe    = imx6_pcie_probe,
	.shutdown = imx6_pcie_shutdown,
};

static void imx6_pcie_quirk(struct pci_dev *dev)
{
	struct pci_bus *bus = dev->bus;
	struct pcie_port *pp = bus->sysdata;

	/* Bus parent is the PCI bridge, its parent is this platform driver */
	if (!bus->dev.parent || !bus->dev.parent->parent)
		return;

	/* Make sure we only quirk devices associated with this driver */
	if (bus->dev.parent->parent->driver != &imx6_pcie_driver.driver)
		return;

	if (bus->number == pp->root_bus_nr) {
		struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
		struct imx6_pcie *imx6_pcie = to_imx6_pcie(pci);

		/*
		 * Limit config length to avoid the kernel reading beyond
		 * the register set and causing an abort on i.MX 6Quad
		 */
		if (imx6_pcie->drvdata->dbi_length) {
			dev->cfg_size = imx6_pcie->drvdata->dbi_length;
			dev_info(&dev->dev, "Limiting cfg_size to %d\n",
					dev->cfg_size);
		}
	}
}
DECLARE_PCI_FIXUP_CLASS_HEADER(PCI_VENDOR_ID_SYNOPSYS, 0xabcd,
			PCI_CLASS_BRIDGE_PCI, 8, imx6_pcie_quirk);

static int __init imx6_pcie_init(void)
{
#ifdef CONFIG_ARM
	/*
	 * Since probe() can be deferred we need to make sure that
	 * hook_fault_code is not called after __init memory is freed
	 * by kernel and since imx6q_pcie_abort_handler() is a no-op,
	 * we can install the handler here without risking it
	 * accessing some uninitialized driver state.
	 */
	hook_fault_code(8, imx6q_pcie_abort_handler, SIGBUS, 0,
			"external abort on non-linefetch");
#endif

	return platform_driver_register(&imx6_pcie_driver);
}
device_initcall(imx6_pcie_init);
