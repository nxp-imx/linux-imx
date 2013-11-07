/*
 * PCIe host controller driver for IMX SOCs
 *
 * Copyright (C) 2013 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/signal.h>
#include <linux/busfreq-imx6.h>

#include "pcie-designware.h"

#define to_imx_pcie(x)	container_of(x, struct imx_pcie, pp)

struct imx_pcie {
	int			irq;
	int			rst_gpio;
	int			pwr_gpio;
	/*
	 * imx pcie interface derives the dbi_base from the
	 * reg property of the DT, stored here temply, and replace the
	 * dbi_base of pcie_port structure later.
	 */
	void __iomem		*dbi_base;
	struct regmap		*gpr;
	struct clk		*pcie_clk;
	struct clk		*pcie_ref_clk;
	struct clk		*pcie_bus_in;
	struct clk		*pcie_bus_out;
	struct pcie_port	pp;
};

/* Register Definitions */
#define PRT_LOG_R_BaseAddress 0x700
#define DB_R0 (PRT_LOG_R_BaseAddress + 0x28)
#define DB_R1 (PRT_LOG_R_BaseAddress + 0x2c)
#define PHY_STS_R (PRT_LOG_R_BaseAddress + 0x110)
#define PHY_CTRL_R (PRT_LOG_R_BaseAddress + 0x114)
/* control bus bit definition */
#define PCIE_CR_CTL_DATA_LOC 0
#define PCIE_CR_CTL_CAP_ADR_LOC 16
#define PCIE_CR_CTL_CAP_DAT_LOC 17
#define PCIE_CR_CTL_WR_LOC 18
#define PCIE_CR_CTL_RD_LOC 19
#define PCIE_CR_STAT_DATA_LOC 0
#define PCIE_CR_STAT_ACK_LOC 16

/*
 * FIELD: RX_LOS_EN_OVRD [13:13]
 * FIELD: RX_LOS_EN [12:12]
 * FIELD: RX_TERM_EN_OVRD [11:11]
 * FIELD: RX_TERM_EN [10:10]
 * FIELD: RX_BIT_SHIFT_OVRD [9:9]
 * FIELD: RX_BIT_SHIFT [8:8]
 * FIELD: RX_ALIGN_EN_OVRD [7:7]
 * FIELD: RX_ALIGN_EN [6:6]
 * FIELD: RX_DATA_EN_OVRD [5:5]
 * FIELD: RX_DATA_EN [4:4]
 * FIELD: RX_PLL_EN_OVRD [3:3]
 * FIELD: RX_PLL_EN [2:2]
 * FIELD: RX_INVERT_OVRD [1:1]
 * FIELD: RX_INVERT [0:0]
*/
#define SSP_CR_LANE0_DIG_RX_OVRD_IN_LO 0x1005

/*
 * FIELD: LOS [2:2]
 * FIELD: PLL_STATE [1:1]
 * FIELD: VALID [0:0]
*/
#define SSP_CR_LANE0_DIG_RX_ASIC_OUT 0x100D
/* End of Register Definitions */

/* PHY CR bus acess routines */
static int
pcie_phy_cr_ack_polling(void __iomem *dbi_base, u32 max_iterations, u32 exp_val)
{
	u32 temp_rd_data, wait_counter = 0;

	do {
		temp_rd_data = readl(dbi_base + PHY_STS_R);
		temp_rd_data = (temp_rd_data >> PCIE_CR_STAT_ACK_LOC) & 0x1;
		wait_counter++;
	} while ((wait_counter < max_iterations) && (temp_rd_data != exp_val));

	if (temp_rd_data != exp_val)
		return 0;
	return 1;
}

static int pcie_phy_cr_cap_addr(void __iomem *dbi_base, u32 addr)
{
	u32 temp_wr_data;

	/* write addr */
	temp_wr_data = addr << PCIE_CR_CTL_DATA_LOC;
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* capture addr */
	temp_wr_data |= (0x1 << PCIE_CR_CTL_CAP_ADR_LOC);
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* wait for ack */
	if (!pcie_phy_cr_ack_polling(dbi_base, 100, 1))
		return 0;

	/* deassert cap addr */
	temp_wr_data = addr << PCIE_CR_CTL_DATA_LOC;
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* wait for ack de-assetion */
	if (!pcie_phy_cr_ack_polling(dbi_base, 100, 0))
		return 0;

	return 1;
}

static int pcie_phy_cr_read(void __iomem *dbi_base, u32 addr , u32 *data)
{
	u32 temp_rd_data, temp_wr_data;

	/*  write addr */
	/* cap addr */
	if (!pcie_phy_cr_cap_addr(dbi_base, addr))
		return 0;

	/* assert rd signal */
	temp_wr_data = 0x1 << PCIE_CR_CTL_RD_LOC;
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* wait for ack */
	if (!pcie_phy_cr_ack_polling(dbi_base, 100, 1))
		return 0;

	/* after got ack return data */
	temp_rd_data = readl(dbi_base + PHY_STS_R);
	*data = (temp_rd_data & (0xffff << PCIE_CR_STAT_DATA_LOC));

	/* deassert rd signal */
	temp_wr_data = 0x0;
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* wait for ack de-assetion */
	if (!pcie_phy_cr_ack_polling(dbi_base, 100, 0))
		return 0;

	return 1;
}

static int pcie_phy_cr_write(void __iomem *dbi_base, u32 addr , u32 data)
{
	u32 temp_wr_data;

	/* write addr */
	/* cap addr */
	if (!pcie_phy_cr_cap_addr(dbi_base, addr))
		return 0;

	temp_wr_data = data << PCIE_CR_CTL_DATA_LOC;
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* capture data */
	temp_wr_data |= (0x1 << PCIE_CR_CTL_CAP_DAT_LOC);
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* wait for ack */
	if (!pcie_phy_cr_ack_polling(dbi_base, 100, 1))
		return 0;

	/* deassert cap data */
	temp_wr_data = data << PCIE_CR_CTL_DATA_LOC;
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* wait for ack de-assetion */
	if (!pcie_phy_cr_ack_polling(dbi_base, 100, 0))
		return 0;

	/* assert wr signal */
	temp_wr_data = 0x1 << PCIE_CR_CTL_WR_LOC;
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* wait for ack */
	if (!pcie_phy_cr_ack_polling(dbi_base, 100, 1))
		return 0;

	/* deassert wr signal */
	temp_wr_data = data << PCIE_CR_CTL_DATA_LOC;
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* wait for ack de-assetion */
	if (!pcie_phy_cr_ack_polling(dbi_base, 100, 0))
		return 0;

	temp_wr_data = 0x0;
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	return 1;
}
/* End of the PHY CR bus acess routines */

static void change_field(int *in, int start, int end, int val)
{
	int mask;

	mask = ((0xFFFFFFFF << start) ^ (0xFFFFFFFF << (end + 1))) & 0xFFFFFFFF;
	*in = (*in & ~mask) | (val << start);
}

static int imx_pcie_link_up(struct pcie_port *pp)
{
	/* link is indicated by the bit4 of DB_R1 register */
	u32 val = readl(pp->dbi_base + DB_R1);

	if (val & (0x1 << 4))
		return 1;

	return 0;
}

static int imx_pcie_establish_link(struct pcie_port *pp)
{
	u32 val, ltssm, rx_valid;
	int count = 0;
	struct imx_pcie *imx_pcie = to_imx_pcie(pp);

	/* setup root complex */
	dw_pcie_setup_rc(pp);

	/* activate PERST_B */
	gpio_set_value(imx_pcie->rst_gpio, 0);
	msleep(100);
	/* deactive PERST_B */
	gpio_set_value(imx_pcie->rst_gpio, 1);

	/* assert LTSSM enable */
	regmap_update_bits(imx_pcie->gpr, IOMUXC_GPR12,
			IMX6Q_GPR12_PCIE_APP_LTSSM_EN, 1 << 10);

	/* check if the link is up or not */
	while (!dw_pcie_link_up(pp)) {
		usleep_range(2000, 3000);
		count++;

		/* From L0, initiate MAC entry to gen2 if EP/RC supports gen2.
		 * Wait 2ms (LTSSM timeout is 24ms, PHY lock is ~5us in gen2).
		 * If (MAC/LTSSM.state == Recovery.RcvrLock)
		 * && (PHY/rx_valid==0) then pulse PHY/rx_reset. Transition
		 * to gen2 is stuck
		 */
		pcie_phy_cr_read(pp->dbi_base, SSP_CR_LANE0_DIG_RX_ASIC_OUT,
				&rx_valid);
		ltssm = readl(pp->dbi_base + DB_R0) & 0x3F;
		if ((ltssm == 0x0D) && ((rx_valid & 0x01) == 0)) {
			pr_info("Transition to gen2 is stuck, reset PHY!\n");
			pcie_phy_cr_read(pp->dbi_base,
					SSP_CR_LANE0_DIG_RX_OVRD_IN_LO, &val);
			change_field(&val, 3, 3, 0x1);
			change_field(&val, 5, 5, 0x1);
			pcie_phy_cr_write(pp->dbi_base,
					SSP_CR_LANE0_DIG_RX_OVRD_IN_LO, val);
			usleep_range(2000, 3000);
			pcie_phy_cr_read(pp->dbi_base,
					SSP_CR_LANE0_DIG_RX_OVRD_IN_LO, &val);
			change_field(&val, 3, 3, 0x0);
			change_field(&val, 5, 5, 0x0);
			pcie_phy_cr_write(pp->dbi_base,
					SSP_CR_LANE0_DIG_RX_OVRD_IN_LO, val);
		}

		if ((count > 100)) {
			dev_err(pp->dev, "link up failed, DB_R0:0x%08x,"
					"DB_R1:0x%08x!\n"
					, readl(pp->dbi_base + DB_R0)
					, readl(pp->dbi_base + DB_R1));
			return -EINVAL;
		}
	}

	dev_info(pp->dev, "Link up\n");

	return 0;
}

static void imx_pcie_host_init(struct pcie_port *pp)
{
	/* replace the dbi_base of pp by the dbi_base of imx_pcie */
	struct imx_pcie *imx_pcie = to_imx_pcie(pp);
	pp->dbi_base = imx_pcie->dbi_base;

	imx_pcie_establish_link(pp);
}

static struct pcie_host_ops imx_pcie_host_ops = {
	.link_up = imx_pcie_link_up,
	.host_init = imx_pcie_host_init,
};

static int imx_add_pcie_port(struct pcie_port *pp, struct platform_device *pdev)
{
	int ret;

	pp->root_bus_nr = -1;
	pp->ops = &imx_pcie_host_ops;

	spin_lock_init(&pp->conf_lock);
	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(&pdev->dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static int __init imx_pcie_probe(struct platform_device *pdev)
{
	struct imx_pcie *imx_pcie;
	struct pcie_port *pp;
	struct device_node *np = pdev->dev.of_node;
	struct resource *dbi_mem;
	int ret = 0;

	if (!np)
		return -ENODEV;

	imx_pcie = devm_kzalloc(&pdev->dev, sizeof(*imx_pcie), GFP_KERNEL);
	if (!imx_pcie) {
		dev_err(&pdev->dev, "no memory for imx_pcie port\n");
		return -ENOMEM;
	}

	pp = &imx_pcie->pp;

	pp->dev = &pdev->dev;
	pp->irq = platform_get_irq(pdev, 0);
	dev_info(&pdev->dev, "legacy_irq %d\n", pp->irq);
	if (pp->irq <= 0) {
		dev_err(&pdev->dev, "no legacy_irq\n");
		return -EINVAL;
	}

	dbi_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!dbi_mem) {
		dev_err(&pdev->dev, "no mmio space\n");
		return -ENOMEM;
	}

	dev_info(&pdev->dev, "map %pR\n", dbi_mem);
	imx_pcie->dbi_base = devm_ioremap(&pdev->dev, dbi_mem->start,
			resource_size(dbi_mem));
	if (!imx_pcie->dbi_base) {
		dev_err(&pdev->dev, "can't map %pR\n", dbi_mem);
		return -EIO;
	}

	imx_pcie->gpr =
		syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (IS_ERR(imx_pcie->gpr)) {
		dev_err(&pdev->dev, "can't find fsl,imx6q-iomux-gpr regmap\n");
		ret = -EIO;
	}

	/* configure constant input signal to the pcie ctrl and phy */
	regmap_update_bits(imx_pcie->gpr, IOMUXC_GPR12,
			IMX6Q_GPR12_PCIE_APP_LTSSM_EN, 0 << 10);
	regmap_update_bits(imx_pcie->gpr, IOMUXC_GPR12,
			IMX6Q_GPR12_PCIE_DEV_TYPE_MASK, 4 << 12);
	regmap_update_bits(imx_pcie->gpr, IOMUXC_GPR12,
			IMX6Q_GPR12_PCIE_LOS_LEVEL_MASK, 9 << 4);
	regmap_update_bits(imx_pcie->gpr, IOMUXC_GPR8,
			IMX6Q_GPR8_PCIE_TX_DEEM_GEN1_MASK, 0 << 0);
	regmap_update_bits(imx_pcie->gpr, IOMUXC_GPR8,
			IMX6Q_GPR8_PCIE_TX_DEEM_GEN2_3P5DB_MASK, 0 << 6);
	regmap_update_bits(imx_pcie->gpr, IOMUXC_GPR8,
			IMX6Q_GPR8_PCIE_TX_DEEM_GEN2_6DB_MASK, 20 << 12);
	regmap_update_bits(imx_pcie->gpr, IOMUXC_GPR8,
			IMX6Q_GPR8_PCIE_TX_SWING_FULL_MASK, 127 << 18);
	regmap_update_bits(imx_pcie->gpr, IOMUXC_GPR8,
			IMX6Q_GPR8_PCIE_TX_SWING_LOW_MASK, 127 << 25);

	/* Enable the pwr, clks and so on */
	imx_pcie->pwr_gpio = of_get_named_gpio(np, "pwr-gpios", 0);
	if (gpio_is_valid(imx_pcie->pwr_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev, imx_pcie->pwr_gpio,
				GPIOF_OUT_INIT_HIGH, "PCIE_PWR_EN");
		if (ret)
			dev_warn(&pdev->dev, "no pwr_en pin available!\n");
	}

	regmap_update_bits(imx_pcie->gpr, IOMUXC_GPR1,
			IMX6Q_GPR1_PCIE_TEST_PD, 0 << 18);

	request_bus_freq(BUS_FREQ_HIGH);
	imx_pcie->pcie_clk = devm_clk_get(&pdev->dev, "pcie_axi");
	if (IS_ERR(imx_pcie->pcie_clk)) {
		dev_err(&pdev->dev, "no pcie clock.\n");
		ret = PTR_ERR(imx_pcie->pcie_clk);
		goto err_release_pwr;
	}
	ret = clk_prepare_enable(imx_pcie->pcie_clk);
	if (ret < 0) {
		dev_err(&pdev->dev, "can't prepare-enable pcie clock\n");
		goto err_release_pwr;
	}

	imx_pcie->pcie_ref_clk = devm_clk_get(&pdev->dev, "pcie_ref");
	if (IS_ERR(imx_pcie->pcie_ref_clk)) {
		dev_err(&pdev->dev, "no pcie ref clock.\n");
		ret = PTR_ERR(imx_pcie->pcie_ref_clk);
		goto err_release_clk;
	}
	ret = clk_prepare_enable(imx_pcie->pcie_ref_clk);
	if (ret < 0) {
		dev_err(&pdev->dev, "can't prepare-enable pcie ref clock\n");
		goto err_release_clk;
	}

	/*
	 * Route pcie bus clock out to EP
	 * disable pcie bus in.
	 * enable pcie bus out.
	 */
	imx_pcie->pcie_bus_in = devm_clk_get(&pdev->dev, "pcie_bus_in");
	if (IS_ERR(imx_pcie->pcie_bus_in)) {
		dev_err(&pdev->dev, "no pcie bus in clock.\n");
		ret = PTR_ERR(imx_pcie->pcie_bus_in);
		goto err_release_ref_clk;
	}
	clk_prepare_enable(imx_pcie->pcie_bus_in);
	clk_disable_unprepare(imx_pcie->pcie_bus_in);

	imx_pcie->pcie_bus_out = devm_clk_get(&pdev->dev, "pcie_bus_out");
	if (IS_ERR(imx_pcie->pcie_bus_out)) {
		dev_err(&pdev->dev, "no pcie bus out clock.\n");
		ret = PTR_ERR(imx_pcie->pcie_bus_out);
		goto err_release_ref_clk;
	}
	ret = clk_prepare_enable(imx_pcie->pcie_bus_out);
	if (ret < 0) {
		dev_err(&pdev->dev, "can't enable pcie bus clock out.\n");
		goto err_release_ref_clk;
	}

	regmap_update_bits(imx_pcie->gpr, IOMUXC_GPR1,
			IMX6Q_GPR1_PCIE_REF_CLK_EN, 1 << 16);

	/* PCIE RESET, togle the external card's reset */
	imx_pcie->rst_gpio = of_get_named_gpio(np, "rst-gpios", 0);
	if (gpio_is_valid(imx_pcie->rst_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev, imx_pcie->rst_gpio,
				GPIOF_DIR_OUT, "PCIE RESET");
		if (ret)
			dev_warn(&pdev->dev, "no pcie rst pin available!\n");
	}

	dev_info(&pdev->dev, "starting to link pcie port.\n");
	ret = imx_add_pcie_port(pp, pdev);
	if (ret < 0)
		goto err_link_down;

	platform_set_drvdata(pdev, imx_pcie);
	return 0;

err_link_down:
	clk_disable_unprepare(imx_pcie->pcie_bus_out);
err_release_ref_clk:
	clk_disable_unprepare(imx_pcie->pcie_ref_clk);
err_release_clk:
	clk_disable_unprepare(imx_pcie->pcie_clk);
err_release_pwr:
	release_bus_freq(BUS_FREQ_HIGH);
	if (gpio_is_valid(imx_pcie->pwr_gpio))
		gpio_set_value(imx_pcie->pwr_gpio, 0);

	return ret;
}

static struct of_device_id imx_pcie_of_match[] = {
	{ .compatible = "fsl,imx-pcie", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_pcie_of_match);

static struct platform_driver imx_pcie_driver = {
	.driver = {
		.name	= "imx-pcie",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(imx_pcie_of_match),
	},
};

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

static int __init imx_pcie_drv_init(void)
{
	/*  Added for PCI abort handling */
	hook_fault_code(16 + 6, imx6q_pcie_abort_handler, SIGBUS, 0,
			"imprecise external abort");

	return platform_driver_probe(&imx_pcie_driver,
			imx_pcie_probe);
}
subsys_initcall(imx_pcie_drv_init);

MODULE_AUTHOR("Richard Zhu <Hong-Xing.Zhu@freescale.com>");
MODULE_DESCRIPTION("i.MX PCIe platform driver");
MODULE_LICENSE("GPL v2");
