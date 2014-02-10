/*
 * PCIe host controller driver for Freescale i.MX6 SoCs
 *
 * Copyright (C) 2014 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2013 Kosagi
 *		http://www.kosagi.com
 *
 * Author: Sean Cross <xobs@kosagi.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/resource.h>
#include <linux/signal.h>
#include <linux/types.h>
#include <linux/busfreq-imx6.h>

#include "pcie-designware.h"

#define to_imx6_pcie(x)	container_of(x, struct imx6_pcie, pp)

/*
 * The default value of the reserved ddr memory
 * used to verify EP/RC memory space access operations.
 * BTW, here is the layout of the 1G ddr on SD boards
 * 0x1000_0000 ~ 0x4FFF_FFFF
 */
static u32 ddr_test_region = 0x40000000;
static u32 test_region_size = SZ_2M;

struct imx6_pcie {
	int			reset_gpio;
	int			power_on_gpio;
	int			wake_up_gpio;
	int			disable_gpio;
	struct clk		*lvds_gate;
	struct clk		*sata_ref_100m;
	struct clk		*pcie_ref_125m;
	struct clk		*pcie_axi;
	struct pcie_port	pp;
	struct regmap		*iomuxc_gpr;
	void __iomem		*mem_base;
};

/* PCIe Port Logic registers (memory-mapped) */
#define PL_OFFSET 0x700
#define PCIE_PHY_DEBUG_R0 (PL_OFFSET + 0x28)
#define PCIE_PHY_DEBUG_R1 (PL_OFFSET + 0x2c)

#define PCIE_PHY_CTRL (PL_OFFSET + 0x114)
#define PCIE_PHY_CTRL_DATA_LOC 0
#define PCIE_PHY_CTRL_CAP_ADR_LOC 16
#define PCIE_PHY_CTRL_CAP_DAT_LOC 17
#define PCIE_PHY_CTRL_WR_LOC 18
#define PCIE_PHY_CTRL_RD_LOC 19

#define PCIE_PHY_STAT (PL_OFFSET + 0x110)
#define PCIE_PHY_STAT_ACK_LOC 16

/* PHY registers (not memory-mapped) */
#define PCIE_PHY_RX_ASIC_OUT 0x100D

#define PHY_RX_OVRD_IN_LO 0x1005
#define PHY_RX_OVRD_IN_LO_RX_DATA_EN (1 << 5)
#define PHY_RX_OVRD_IN_LO_RX_PLL_EN (1 << 3)

static int pcie_phy_poll_ack(void __iomem *dbi_base, int exp_val)
{
	u32 val;
	u32 max_iterations = 10;
	u32 wait_counter = 0;

	do {
		val = readl(dbi_base + PCIE_PHY_STAT);
		val = (val >> PCIE_PHY_STAT_ACK_LOC) & 0x1;
		wait_counter++;

		if (val == exp_val)
			return 0;

		udelay(1);
	} while (wait_counter < max_iterations);

	return -ETIMEDOUT;
}

static int pcie_phy_wait_ack(void __iomem *dbi_base, int addr)
{
	u32 val;
	int ret;

	val = addr << PCIE_PHY_CTRL_DATA_LOC;
	writel(val, dbi_base + PCIE_PHY_CTRL);

	val |= (0x1 << PCIE_PHY_CTRL_CAP_ADR_LOC);
	writel(val, dbi_base + PCIE_PHY_CTRL);

	ret = pcie_phy_poll_ack(dbi_base, 1);
	if (ret)
		return ret;

	val = addr << PCIE_PHY_CTRL_DATA_LOC;
	writel(val, dbi_base + PCIE_PHY_CTRL);

	ret = pcie_phy_poll_ack(dbi_base, 0);
	if (ret)
		return ret;

	return 0;
}

/* Read from the 16-bit PCIe PHY control registers (not memory-mapped) */
static int pcie_phy_read(void __iomem *dbi_base, int addr , int *data)
{
	u32 val, phy_ctl;
	int ret;

	ret = pcie_phy_wait_ack(dbi_base, addr);
	if (ret)
		return ret;

	/* assert Read signal */
	phy_ctl = 0x1 << PCIE_PHY_CTRL_RD_LOC;
	writel(phy_ctl, dbi_base + PCIE_PHY_CTRL);

	ret = pcie_phy_poll_ack(dbi_base, 1);
	if (ret)
		return ret;

	val = readl(dbi_base + PCIE_PHY_STAT);
	*data = val & 0xffff;

	/* deassert Read signal */
	writel(0x00, dbi_base + PCIE_PHY_CTRL);

	ret = pcie_phy_poll_ack(dbi_base, 0);
	if (ret)
		return ret;

	return 0;
}

static int pcie_phy_write(void __iomem *dbi_base, int addr, int data)
{
	u32 var;
	int ret;

	/* write addr */
	/* cap addr */
	ret = pcie_phy_wait_ack(dbi_base, addr);
	if (ret)
		return ret;

	var = data << PCIE_PHY_CTRL_DATA_LOC;
	writel(var, dbi_base + PCIE_PHY_CTRL);

	/* capture data */
	var |= (0x1 << PCIE_PHY_CTRL_CAP_DAT_LOC);
	writel(var, dbi_base + PCIE_PHY_CTRL);

	ret = pcie_phy_poll_ack(dbi_base, 1);
	if (ret)
		return ret;

	/* deassert cap data */
	var = data << PCIE_PHY_CTRL_DATA_LOC;
	writel(var, dbi_base + PCIE_PHY_CTRL);

	/* wait for ack de-assertion */
	ret = pcie_phy_poll_ack(dbi_base, 0);
	if (ret)
		return ret;

	/* assert wr signal */
	var = 0x1 << PCIE_PHY_CTRL_WR_LOC;
	writel(var, dbi_base + PCIE_PHY_CTRL);

	/* wait for ack */
	ret = pcie_phy_poll_ack(dbi_base, 1);
	if (ret)
		return ret;

	/* deassert wr signal */
	var = data << PCIE_PHY_CTRL_DATA_LOC;
	writel(var, dbi_base + PCIE_PHY_CTRL);

	/* wait for ack de-assertion */
	ret = pcie_phy_poll_ack(dbi_base, 0);
	if (ret)
		return ret;

	writel(0x0, dbi_base + PCIE_PHY_CTRL);

	return 0;
}

/*  Added for PCI abort handling */
static int imx6q_pcie_abort_handler(unsigned long addr,
		unsigned int fsr, struct pt_regs *regs)
{
	return 0;
}

static int imx6_pcie_deassert_core_reset(struct pcie_port *pp)
{
	struct imx6_pcie *imx6_pcie = to_imx6_pcie(pp);
	int ret;

	if (gpio_is_valid(imx6_pcie->power_on_gpio))
		gpio_set_value(imx6_pcie->power_on_gpio, 1);

	regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
			IMX6Q_GPR1_PCIE_TEST_PD, 0 << 18);
	regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
			IMX6Q_GPR1_PCIE_REF_CLK_EN, 1 << 16);
	request_bus_freq(BUS_FREQ_HIGH);

	ret = clk_prepare_enable(imx6_pcie->sata_ref_100m);
	if (ret) {
		dev_err(pp->dev, "unable to enable sata_ref_100m\n");
		goto err_sata_ref;
	}

	ret = clk_prepare_enable(imx6_pcie->pcie_ref_125m);
	if (ret) {
		dev_err(pp->dev, "unable to enable pcie_ref_125m\n");
		goto err_pcie_ref;
	}

	if (!IS_ENABLED(CONFIG_EP_MODE_IN_EP_RC_SYS)
			&& !IS_ENABLED(CONFIG_RC_MODE_IN_EP_RC_SYS)) {
		ret = clk_prepare_enable(imx6_pcie->lvds_gate);
		if (ret) {
			dev_err(pp->dev, "unable to enable lvds_gate\n");
			goto err_lvds_gate;
		}
	}

	ret = clk_prepare_enable(imx6_pcie->pcie_axi);
	if (ret) {
		dev_err(pp->dev, "unable to enable pcie_axi\n");
		goto err_pcie_axi;
	}

	/* allow the clocks to stabilize */
	usleep_range(200, 500);

	if (gpio_is_valid(imx6_pcie->reset_gpio)) {
		gpio_set_value(imx6_pcie->reset_gpio, 0);
		msleep(100);
		gpio_set_value(imx6_pcie->reset_gpio, 1);
	}

	return 0;

err_pcie_axi:
	clk_disable_unprepare(imx6_pcie->lvds_gate);
err_lvds_gate:
	clk_disable_unprepare(imx6_pcie->pcie_ref_125m);
err_pcie_ref:
	clk_disable_unprepare(imx6_pcie->sata_ref_100m);
err_sata_ref:
	release_bus_freq(BUS_FREQ_HIGH);
	return ret;

}

static void imx6_pcie_init_phy(struct pcie_port *pp)
{
	struct imx6_pcie *imx6_pcie = to_imx6_pcie(pp);

	regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
			IMX6Q_GPR12_PCIE_CTL_2, 0 << 10);

	/* configure constant input signal to the pcie ctrl and phy */
	if (IS_ENABLED(CONFIG_EP_MODE_IN_EP_RC_SYS))
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				IMX6Q_GPR12_DEVICE_TYPE,
				PCI_EXP_TYPE_ENDPOINT << 12);
	else
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				IMX6Q_GPR12_DEVICE_TYPE,
				PCI_EXP_TYPE_ROOT_PORT << 12);
	regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
			IMX6Q_GPR12_LOS_LEVEL, 9 << 4);

	regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR8,
			IMX6Q_GPR8_TX_DEEMPH_GEN1, 0 << 0);
	regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR8,
			IMX6Q_GPR8_TX_DEEMPH_GEN2_3P5DB, 0 << 6);
	regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR8,
			IMX6Q_GPR8_TX_DEEMPH_GEN2_6DB, 20 << 12);
	regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR8,
			IMX6Q_GPR8_TX_SWING_FULL, 127 << 18);
	regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR8,
			IMX6Q_GPR8_TX_SWING_LOW, 127 << 25);
}

static irqreturn_t imx_pcie_msi_irq_handler(int irq, void *arg)
{
	struct pcie_port *pp = arg;

	dw_handle_msi_irq(pp);

	return IRQ_HANDLED;
}

static void imx6_pcie_host_init(struct pcie_port *pp)
{
	int count = 0;
	struct imx6_pcie *imx6_pcie = to_imx6_pcie(pp);

	imx6_pcie_init_phy(pp);

	imx6_pcie_deassert_core_reset(pp);

	dw_pcie_setup_rc(pp);

	regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
			IMX6Q_GPR12_PCIE_CTL_2, 1 << 10);

	while (!dw_pcie_link_up(pp)) {
		usleep_range(100, 1000);
		count++;
		if (count >= 200) {
			dev_err(pp->dev, "phy link never came up\n");
			dev_dbg(pp->dev,
				"DEBUG_R0: 0x%08x, DEBUG_R1: 0x%08x\n",
				readl(pp->dbi_base + PCIE_PHY_DEBUG_R0),
				readl(pp->dbi_base + PCIE_PHY_DEBUG_R1));
			return;
		}
	}

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		pp->quirks |= DW_PCIE_QUIRK_NO_MSI_VEC;
		pp->quirks |= DW_PCIE_QUIRK_MSI_SELF_EN;
		dw_pcie_msi_init(pp);
	}

	return;
}

static int imx6_pcie_link_up(struct pcie_port *pp)
{
	u32 rc, ltssm, rx_valid, temp;

	/* link is debug bit 36, debug register 1 starts at bit 32 */
	rc = readl(pp->dbi_base + PCIE_PHY_DEBUG_R1) & (0x1 << (36 - 32));
	if (rc)
		return -EAGAIN;

	/*
	 * From L0, initiate MAC entry to gen2 if EP/RC supports gen2.
	 * Wait 2ms (LTSSM timeout is 24ms, PHY lock is ~5us in gen2).
	 * If (MAC/LTSSM.state == Recovery.RcvrLock)
	 * && (PHY/rx_valid==0) then pulse PHY/rx_reset. Transition
	 * to gen2 is stuck
	 */
	pcie_phy_read(pp->dbi_base, PCIE_PHY_RX_ASIC_OUT, &rx_valid);
	ltssm = readl(pp->dbi_base + PCIE_PHY_DEBUG_R0) & 0x3F;

	if (rx_valid & 0x01)
		return 0;

	if (ltssm != 0x0d)
		return 0;

	dev_err(pp->dev, "transition to gen2 is stuck, reset PHY!\n");

	pcie_phy_read(pp->dbi_base,
		PHY_RX_OVRD_IN_LO, &temp);
	temp |= (PHY_RX_OVRD_IN_LO_RX_DATA_EN
		| PHY_RX_OVRD_IN_LO_RX_PLL_EN);
	pcie_phy_write(pp->dbi_base,
		PHY_RX_OVRD_IN_LO, temp);

	usleep_range(2000, 3000);

	pcie_phy_read(pp->dbi_base,
		PHY_RX_OVRD_IN_LO, &temp);
	temp &= ~(PHY_RX_OVRD_IN_LO_RX_DATA_EN
		| PHY_RX_OVRD_IN_LO_RX_PLL_EN);
	pcie_phy_write(pp->dbi_base,
		PHY_RX_OVRD_IN_LO, temp);

	return 0;
}

static struct pcie_host_ops imx6_pcie_host_ops = {
	.link_up = imx6_pcie_link_up,
	.host_init = imx6_pcie_host_init,
};

static int imx6_add_pcie_port(struct pcie_port *pp,
			struct platform_device *pdev)
{
	int ret;

	pp->irq = platform_get_irq(pdev, 0);
	if (!pp->irq) {
		dev_err(&pdev->dev, "failed to get irq\n");
		return -ENODEV;
	}

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		pp->msi_irq = pp->irq - 3;
		if (!pp->msi_irq) {
			dev_err(&pdev->dev, "failed to get msi irq\n");
			return -ENODEV;
		}

		ret = devm_request_irq(&pdev->dev, pp->msi_irq,
					imx_pcie_msi_irq_handler,
					IRQF_SHARED, "imx6q-pcie", pp);
		if (ret) {
			dev_err(&pdev->dev, "failed to request msi irq\n");
			return ret;
		}
	}

	pp->root_bus_nr = -1;
	pp->ops = &imx6_pcie_host_ops;

	spin_lock_init(&pp->conf_lock);
	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(&pdev->dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static ssize_t imx_pcie_bar0_addr_info(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	struct imx6_pcie *imx6_pcie = dev_get_drvdata(dev);
	struct pcie_port *pp = &imx6_pcie->pp;

	return sprintf(buf, "imx-pcie-bar0-addr-info start 0x%08x\n",
			readl(pp->dbi_base + PCI_BASE_ADDRESS_0));
}

static ssize_t imx_pcie_bar0_addr_start(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	u32 bar_start;
	struct imx6_pcie *imx6_pcie = dev_get_drvdata(dev);
	struct pcie_port *pp = &imx6_pcie->pp;

	sscanf(buf, "%x\n", &bar_start);
	writel(bar_start, pp->dbi_base + PCI_BASE_ADDRESS_0);

	return count;
}

static void imx_pcie_regions_setup(struct device *dev)
{
	struct imx6_pcie *imx6_pcie = dev_get_drvdata(dev);
	struct pcie_port *pp = &imx6_pcie->pp;

	if (IS_ENABLED(CONFIG_EP_MODE_IN_EP_RC_SYS)) {
		/*
		 * region2 outbound used to access rc mem
		 * in imx6 pcie ep/rc validation system
		 */
		writel(0, pp->dbi_base + PCIE_ATU_VIEWPORT);
		writel(0x01000000, pp->dbi_base + PCIE_ATU_LOWER_BASE);
		writel(0, pp->dbi_base + PCIE_ATU_UPPER_BASE);
		writel(0x01000000 + test_region_size,
				pp->dbi_base + PCIE_ATU_LIMIT);

		writel(ddr_test_region,
				pp->dbi_base + PCIE_ATU_LOWER_TARGET);
		writel(0, pp->dbi_base + PCIE_ATU_UPPER_TARGET);
		writel(PCIE_ATU_TYPE_MEM, pp->dbi_base + PCIE_ATU_CR1);
		writel(PCIE_ATU_ENABLE, pp->dbi_base + PCIE_ATU_CR2);
	}

	if (IS_ENABLED(CONFIG_RC_MODE_IN_EP_RC_SYS)) {
		/*
		 * region2 outbound used to access ep mem
		 * in imx6 pcie ep/rc validation system
		 */
		writel(2, pp->dbi_base + PCIE_ATU_VIEWPORT);
		writel(0x01000000, pp->dbi_base + PCIE_ATU_LOWER_BASE);
		writel(0, pp->dbi_base + PCIE_ATU_UPPER_BASE);
		writel(0x01000000 + test_region_size,
				pp->dbi_base + PCIE_ATU_LIMIT);

		writel(ddr_test_region,
				pp->dbi_base + PCIE_ATU_LOWER_TARGET);
		writel(0, pp->dbi_base + PCIE_ATU_UPPER_TARGET);
		writel(PCIE_ATU_TYPE_MEM, pp->dbi_base + PCIE_ATU_CR1);
		writel(PCIE_ATU_ENABLE, pp->dbi_base + PCIE_ATU_CR2);
	}
}

static ssize_t imx_pcie_memw_info(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "imx-pcie-rc-memw-info start 0x%08x, size 0x%08x\n",
			ddr_test_region, test_region_size);
}

static ssize_t
imx_pcie_memw_start(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	u32 memw_start;

	sscanf(buf, "%x\n", &memw_start);

	if (memw_start < 0x10000000) {
		dev_err(dev, "Invalid memory start address.\n");
		dev_info(dev, "For example: echo 0x41000000 > /sys/...");
		return -1;
	}

	if (ddr_test_region != memw_start) {
		ddr_test_region = memw_start;
		/* Re-setup the iATU */
		imx_pcie_regions_setup(dev);
	}

	return count;
}

static ssize_t
imx_pcie_memw_size(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	u32 memw_size;

	sscanf(buf, "%x\n", &memw_size);

	if ((memw_size > (SZ_16M - SZ_1M)) || (memw_size < SZ_64K)) {
		dev_err(dev, "Invalid, should be [SZ_64K,SZ_16M - SZ_1MB].\n");
		dev_info(dev, "For example: echo 0x800000 > /sys/...");
		return -1;
	}

	if (test_region_size != memw_size) {
		test_region_size = memw_size;
		/* Re-setup the iATU */
		imx_pcie_regions_setup(dev);
	}

	return count;
}

static DEVICE_ATTR(memw_info, S_IRUGO, imx_pcie_memw_info, NULL);
static DEVICE_ATTR(memw_start_set, S_IWUGO, NULL, imx_pcie_memw_start);
static DEVICE_ATTR(memw_size_set, S_IWUGO, NULL, imx_pcie_memw_size);
static DEVICE_ATTR(ep_bar0_addr, S_IRWXUGO, imx_pcie_bar0_addr_info,
		imx_pcie_bar0_addr_start);

static struct attribute *imx_pcie_attrs[] = {
	/*
	 * The start address, and the limitation (64KB ~ (16MB - 1MB))
	 * of the ddr mem window reserved by RC, and used for EP to access.
	 * BTW, these attrs are only configured at EP side.
	 */
	&dev_attr_memw_info.attr,
	&dev_attr_memw_start_set.attr,
	&dev_attr_memw_size_set.attr,
	&dev_attr_ep_bar0_addr.attr,
	NULL
};

static struct attribute_group imx_pcie_attrgroup = {
	.attrs	= imx_pcie_attrs,
};

static int __init imx6_pcie_probe(struct platform_device *pdev)
{
	struct imx6_pcie *imx6_pcie;
	struct pcie_port *pp;
	struct device_node *np = pdev->dev.of_node;
	struct resource *dbi_base;
	int ret, i;
	void *test_reg1, *test_reg2;
	void __iomem *pcie_arb_base_addr;
	struct timeval tv1, tv2, tv3;
	u32 tv_count1, tv_count2;

	imx6_pcie = devm_kzalloc(&pdev->dev, sizeof(*imx6_pcie), GFP_KERNEL);
	if (!imx6_pcie)
		return -ENOMEM;

	pp = &imx6_pcie->pp;
	pp->dev = &pdev->dev;

	if (IS_ENABLED(CONFIG_EP_MODE_IN_EP_RC_SYS)) {
		/* add attributes for device */
		ret = sysfs_create_group(&pdev->dev.kobj, &imx_pcie_attrgroup);
		if (ret)
			return -EINVAL;
	}

	/* Added for PCI abort handling */
	hook_fault_code(16 + 6, imx6q_pcie_abort_handler, SIGBUS, 0,
		"imprecise external abort");

	dbi_base = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!dbi_base) {
		dev_err(&pdev->dev, "dbi_base memory resource not found\n");
		return -ENODEV;
	}

	pp->dbi_base = devm_ioremap_resource(&pdev->dev, dbi_base);
	if (IS_ERR(pp->dbi_base)) {
		ret = PTR_ERR(pp->dbi_base);
		goto err;
	}

	/* Fetch GPIOs */
	imx6_pcie->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (gpio_is_valid(imx6_pcie->reset_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev,
					imx6_pcie->reset_gpio,
					GPIOF_OUT_INIT_LOW,
					"PCIe reset");
		if (ret) {
			dev_err(&pdev->dev, "unable to get reset gpio\n");
			goto err;
		}
	}

	imx6_pcie->power_on_gpio = of_get_named_gpio(np, "power-on-gpio", 0);
	if (gpio_is_valid(imx6_pcie->power_on_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev,
					imx6_pcie->power_on_gpio,
					GPIOF_OUT_INIT_LOW,
					"PCIe power enable");
		if (ret) {
			dev_err(&pdev->dev, "unable to get power-on gpio\n");
			goto err;
		}
	}

	imx6_pcie->wake_up_gpio = of_get_named_gpio(np, "wake-up-gpio", 0);
	if (gpio_is_valid(imx6_pcie->wake_up_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev,
					imx6_pcie->wake_up_gpio,
					GPIOF_IN,
					"PCIe wake up");
		if (ret) {
			dev_err(&pdev->dev, "unable to get wake-up gpio\n");
			goto err;
		}
	}

	imx6_pcie->disable_gpio = of_get_named_gpio(np, "disable-gpio", 0);
	if (gpio_is_valid(imx6_pcie->disable_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev,
					imx6_pcie->disable_gpio,
					GPIOF_OUT_INIT_HIGH,
					"PCIe disable endpoint");
		if (ret) {
			dev_err(&pdev->dev, "unable to get disable-ep gpio\n");
			goto err;
		}
	}

	/* Fetch clocks */
	imx6_pcie->lvds_gate = devm_clk_get(&pdev->dev, "lvds_gate");
	if (IS_ERR(imx6_pcie->lvds_gate)) {
		dev_err(&pdev->dev,
			"lvds_gate clock select missing or invalid\n");
		ret = PTR_ERR(imx6_pcie->lvds_gate);
		goto err;
	}

	imx6_pcie->sata_ref_100m = devm_clk_get(&pdev->dev, "sata_ref_100m");
	if (IS_ERR(imx6_pcie->sata_ref_100m)) {
		dev_err(&pdev->dev,
			"sata_ref_100m clock source missing or invalid\n");
		ret = PTR_ERR(imx6_pcie->sata_ref_100m);
		goto err;
	}

	imx6_pcie->pcie_ref_125m = devm_clk_get(&pdev->dev, "pcie_ref_125m");
	if (IS_ERR(imx6_pcie->pcie_ref_125m)) {
		dev_err(&pdev->dev,
			"pcie_ref_125m clock source missing or invalid\n");
		ret = PTR_ERR(imx6_pcie->pcie_ref_125m);
		goto err;
	}

	imx6_pcie->pcie_axi = devm_clk_get(&pdev->dev, "pcie_axi");
	if (IS_ERR(imx6_pcie->pcie_axi)) {
		dev_err(&pdev->dev,
			"pcie_axi clock source missing or invalid\n");
		ret = PTR_ERR(imx6_pcie->pcie_axi);
		goto err;
	}

	/* Grab GPR config register range */
	imx6_pcie->iomuxc_gpr =
		 syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (IS_ERR(imx6_pcie->iomuxc_gpr)) {
		dev_err(&pdev->dev, "unable to find iomuxc registers\n");
		ret = PTR_ERR(imx6_pcie->iomuxc_gpr);
		goto err;
	}

	if (IS_ENABLED(CONFIG_EP_MODE_IN_EP_RC_SYS)) {
		if (IS_ENABLED(CONFIG_EP_SELF_IO_TEST)) {
			/* Prepare the test regions and data */
			test_reg1 = devm_kzalloc(&pdev->dev,
					test_region_size, GFP_KERNEL);
			if (!test_reg1) {
				pr_err("pcie ep: can't alloc the test reg1.\n");
				ret = PTR_ERR(test_reg1);
				goto err;
			}

			test_reg2 = devm_kzalloc(&pdev->dev,
					test_region_size, GFP_KERNEL);
			if (!test_reg2) {
				pr_err("pcie ep: can't alloc the test reg2.\n");
				ret = PTR_ERR(test_reg1);
				goto err;
			}

			pcie_arb_base_addr = ioremap_cached(0x01000000,
					test_region_size);

			if (!pcie_arb_base_addr) {
				pr_err("error with ioremap in ep selftest\n");
				ret = PTR_ERR(pcie_arb_base_addr);
				goto err;
			}

			for (i = 0; i < test_region_size; i = i + 4) {
				writel(0xE6600D00 + i, test_reg1 + i);
				writel(0xDEADBEAF, test_reg2 + i);
			}
		}

		imx6_pcie_init_phy(pp);

		imx6_pcie_deassert_core_reset(pp);

		/* assert LTSSM enable */
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				IMX6Q_GPR12_PCIE_CTL_2, 1 << 10);


		dev_info(&pdev->dev, "PCIe EP: waiting for link up...\n");

		platform_set_drvdata(pdev, imx6_pcie);
		/* link is indicated by the bit4 of DB_R1 register */
		do {
			usleep_range(10, 20);
		} while ((readl(pp->dbi_base + PCIE_PHY_DEBUG_R1) & 0x10) == 0);

		/* CMD reg:I/O space, MEM space, and Bus Master Enable */
		writel(readl(pp->dbi_base + PCI_COMMAND)
				| PCI_COMMAND_IO
				| PCI_COMMAND_MEMORY
				| PCI_COMMAND_MASTER,
				pp->dbi_base + PCI_COMMAND);

		/*
		 * configure the class_rev(emaluate one memory ram ep device),
		 * bar0 and bar1 of ep
		 */
		writel(0xdeadbeaf, pp->dbi_base + PCI_VENDOR_ID);
		writel(readl(pp->dbi_base + PCI_CLASS_REVISION)
				| (PCI_CLASS_MEMORY_RAM	<< 16),
				pp->dbi_base + PCI_CLASS_REVISION);
		writel(0xdeadbeaf, pp->dbi_base
				+ PCI_SUBSYSTEM_VENDOR_ID);

		/* 32bit none-prefetchable 8M bytes memory on bar0 */
		writel(0x0, pp->dbi_base + PCI_BASE_ADDRESS_0);
		writel(SZ_8M - 1, pp->dbi_base + (1 << 12)
				+ PCI_BASE_ADDRESS_0);

		/* None used bar1 */
		writel(0x0, pp->dbi_base + PCI_BASE_ADDRESS_1);
		writel(0, pp->dbi_base + (1 << 12) + PCI_BASE_ADDRESS_1);

		/* 4K bytes IO on bar2 */
		writel(0x1, pp->dbi_base + PCI_BASE_ADDRESS_2);
		writel(SZ_4K - 1, pp->dbi_base + (1 << 12) +
				PCI_BASE_ADDRESS_2);

		/*
		 * 32bit prefetchable 1M bytes memory on bar3
		 * FIXME BAR MASK3 is not changable, the size
		 * is fixed to 256 bytes.
		 */
		writel(0x8, pp->dbi_base + PCI_BASE_ADDRESS_3);
		writel(SZ_1M - 1, pp->dbi_base + (1 << 12)
				+ PCI_BASE_ADDRESS_3);

		/*
		 * 64bit prefetchable 1M bytes memory on bar4-5.
		 * FIXME BAR4,5 are not enabled yet
		 */
		writel(0xc, pp->dbi_base + PCI_BASE_ADDRESS_4);
		writel(SZ_1M - 1, pp->dbi_base + (1 << 12)
				+ PCI_BASE_ADDRESS_4);
		writel(0, pp->dbi_base + (1 << 12) + PCI_BASE_ADDRESS_5);

		/* Re-setup the iATU */
		imx_pcie_regions_setup(&pdev->dev);

		if (IS_ENABLED(CONFIG_EP_SELF_IO_TEST)) {
			/* PCIe EP start the data transfer after link up */
			pr_info("pcie ep: Starting data transfer...\n");
			do_gettimeofday(&tv1);

			memcpy((unsigned long *)pcie_arb_base_addr,
					(unsigned long *)test_reg1,
					test_region_size);

			do_gettimeofday(&tv2);

			memcpy((unsigned long *)test_reg2,
					(unsigned long *)pcie_arb_base_addr,
					test_region_size);

			do_gettimeofday(&tv3);

			if (memcmp(test_reg2, test_reg1, test_region_size) == 0) {
				tv_count1 = (tv2.tv_sec - tv1.tv_sec)
					* USEC_PER_SEC
					+ tv2.tv_usec - tv1.tv_usec;
				tv_count2 = (tv3.tv_sec - tv2.tv_sec)
					* USEC_PER_SEC
					+ tv3.tv_usec - tv2.tv_usec;

				pr_info("pcie ep: Data transfer is successful."
						" tv_count1 %dus,"
						" tv_count2 %dus.\n",
						tv_count1, tv_count2);
				pr_info("pcie ep: Data write speed:%ldMB/s.\n",
						((test_region_size/1024)
						   * MSEC_PER_SEC)
						/(tv_count1));
				pr_info("pcie ep: Data read speed:%ldMB/s.\n",
						((test_region_size/1024)
						   * MSEC_PER_SEC)
						/(tv_count2));
			} else {
				pr_info("pcie ep: Data transfer is failed.\n");
			}
		}
	} else {
		ret = imx6_add_pcie_port(pp, pdev);
		if (ret < 0)
			goto err;

		platform_set_drvdata(pdev, imx6_pcie);

		/* Re-setup the iATU */
		imx_pcie_regions_setup(&pdev->dev);
	}
	return 0;

err:
	return ret;
}

static const struct of_device_id imx6_pcie_of_match[] = {
	{ .compatible = "fsl,imx6q-pcie", },
	{},
};
MODULE_DEVICE_TABLE(of, imx6_pcie_of_match);

static struct platform_driver imx6_pcie_driver = {
	.driver = {
		.name	= "imx6q-pcie",
		.owner	= THIS_MODULE,
		.of_match_table = imx6_pcie_of_match,
	},
};

/* Freescale PCIe driver does not allow module unload */

static int __init imx6_pcie_init(void)
{
	return platform_driver_probe(&imx6_pcie_driver, imx6_pcie_probe);
}
fs_initcall(imx6_pcie_init);

MODULE_AUTHOR("Sean Cross <xobs@kosagi.com>");
MODULE_DESCRIPTION("Freescale i.MX6 PCIe host controller driver");
MODULE_LICENSE("GPL v2");
