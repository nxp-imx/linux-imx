/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/ahci_platform.h>
#include <mach/common.h>
#include <mach/hardware.h>

/*****************************************************************************\
 *                                                                           *
 * FSL SATA AHCI low level functions                                         *
 * return value 1 failure, 0 success                                         *
 *                                                                           *
\*****************************************************************************/
enum {
	HOST_CAP = 0x00,
	HOST_CAP_SSS = (1 << 27), /* Staggered Spin-up */
	HOST_PORTS_IMPL	= 0x0c,
	HOST_TIMER1MS = 0xe0, /* Timer 1-ms */
	/* Offest used to control the MPLL input clk */
	PHY_CR_CLOCK_FREQ_OVRD = 0x12,
	/* Port0 PHY Control */
	PORT_PHY_CTL = 0x178,
	/* PORT_PHY_CTL bits */
	PORT_PHY_CTL_CAP_ADR_LOC = 0x10000,
	PORT_PHY_CTL_CAP_DAT_LOC = 0x20000,
	PORT_PHY_CTL_WRITE_LOC = 0x40000,
	PORT_PHY_CTL_READ_LOC = 0x80000,
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

static int write_phy_ctl_ack_polling(u32 data, void __iomem *mmio,
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
			printk(KERN_ERR "Wait for CR ACK error!\n");
			return 1;
		}
		msleep(1);
	}
	return 0;
}

static int sata_phy_cr_addr(u32 addr, void __iomem *mmio)
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

static int sata_phy_cr_write(u32 data, void __iomem *mmio)
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

static int sata_phy_cr_read(u32 *data, void __iomem *mmio)
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

/* SATA AHCI temperature monitor */
static ssize_t sata_ahci_current_tmp(struct device *dev, struct device_attribute
			      *devattr, char *buf)
{
	void __iomem *mmio;
	u32 mpll_test_reg, rtune_ctl_reg, dac_ctl_reg, adc_out_reg;
	u32 str1, str2, str3, str4, read_sum, index;
	int m1, m2, a, temp;

	/* map the IO addr */
	mmio = ioremap(MX53_SATA_BASE_ADDR, SZ_2K);
	if (mmio == NULL) {
		printk(KERN_ERR "Failed to map SATA REGS\n");
		return 1;
	}

	/* check rd-wr to reg */
	read_sum = 0;
	sata_phy_cr_addr(SATA_PHY_CR_CLOCK_CRCMP_LT_LIMIT, mmio);
	sata_phy_cr_write(read_sum, mmio);
	sata_phy_cr_read(&read_sum, mmio);
	if ((read_sum & 0xffff) != 0)
		printk(KERN_ERR "Read/Write REG error, 0x%x!\n", read_sum);

	sata_phy_cr_write(0x5A5A, mmio);
	sata_phy_cr_read(&read_sum, mmio);
	if ((read_sum & 0xffff) != 0x5A5A)
		printk(KERN_ERR "Read/Write REG error, 0x%x!\n", read_sum);

	sata_phy_cr_write(0x1234, mmio);
	sata_phy_cr_read(&read_sum, mmio);
	if ((read_sum & 0xffff) != 0x1234)
		printk(KERN_ERR "Read/Write REG error, 0x%x!\n", read_sum);

	/* stat temperature test */
	sata_phy_cr_addr(SATA_PHY_CR_CLOCK_MPLL_TST, mmio);
	sata_phy_cr_read(&mpll_test_reg, mmio);
	sata_phy_cr_addr(SATA_PHY_CR_CLOCK_RTUNE_CTL, mmio);
	sata_phy_cr_read(&rtune_ctl_reg, mmio);
	sata_phy_cr_addr(SATA_PHY_CR_CLOCK_DAC_CTL, mmio);
	sata_phy_cr_read(&dac_ctl_reg, mmio);

	/* mpll_tst.meas_iv   ([12:2]) */
	str1 = (mpll_test_reg >> 2) & 0x7FF;
	/* rtune_ctl.mode     ([1:0]) */
	str2 = (rtune_ctl_reg) & 0x3;
	/* dac_ctl.dac_mode   ([14:12]) */
	str3 = (dac_ctl_reg >> 12)  & 0x7;
	/* rtune_ctl.sel_atbp ([4]) */
	str4 = (rtune_ctl_reg >> 4);

	/* Caculate the m1 */
	/* mpll_tst.meas_iv */
	mpll_test_reg = (mpll_test_reg & 0xE03) | (512) << 2;
	/* rtune_ctl.mode */
	rtune_ctl_reg = (rtune_ctl_reg & 0xFFC) | (1);
	/* dac_ctl.dac_mode */
	dac_ctl_reg = (dac_ctl_reg & 0x8FF) | (4) << 12;
	/* rtune_ctl.sel_atbp */
	rtune_ctl_reg = (rtune_ctl_reg & 0xFEF) | (0) << 4;

	sata_phy_cr_addr(SATA_PHY_CR_CLOCK_MPLL_TST, mmio);
	sata_phy_cr_write(mpll_test_reg, mmio);
	sata_phy_cr_addr(SATA_PHY_CR_CLOCK_DAC_CTL, mmio);
	sata_phy_cr_write(dac_ctl_reg, mmio);
	sata_phy_cr_addr(SATA_PHY_CR_CLOCK_RTUNE_CTL, mmio);
	sata_phy_cr_write(rtune_ctl_reg, mmio);

	/* two dummy read */
	index = 0;
	read_sum = 0;
	adc_out_reg = 0;
	sata_phy_cr_addr(SATA_PHY_CR_CLOCK_ADC_OUT, mmio);
	while (index < 2) {
		sata_phy_cr_read(&adc_out_reg, mmio);
		/* check if valid */
		if (adc_out_reg & 0x400)
			index = index + 1;
		read_sum++;
		if (read_sum > 100000) {
			printk(KERN_ERR "Read REG more than 100000 times!\n");
			break;
		}
	}

	index = 0;
	read_sum = 0;
	while (index < 80) {
		sata_phy_cr_read(&adc_out_reg, mmio);
		if (adc_out_reg & 0x400) {
			read_sum = read_sum + (adc_out_reg & 0x3FF);
			index = index + 1;
		}
	}
	/* Use the U32 to make 1000 precision */
	m1 = (read_sum * 1000) / 80;

	/* Caculate the m2 */
	/* rtune_ctl.sel_atbp */
	rtune_ctl_reg = (rtune_ctl_reg & 0xFEF) | (1) << 4;
	sata_phy_cr_addr(SATA_PHY_CR_CLOCK_RTUNE_CTL, mmio);
	sata_phy_cr_write(rtune_ctl_reg, mmio);

	/* two dummy read */
	index = 0;
	read_sum = 0;
	sata_phy_cr_addr(SATA_PHY_CR_CLOCK_ADC_OUT, mmio);
	while (index < 2) {
		sata_phy_cr_read(&adc_out_reg, mmio);
		/* check if valid */
		if (adc_out_reg & 0x400)
			index = index + 1;
		read_sum++;
		if (read_sum > 100000) {
			printk(KERN_ERR "Read REG more than 100000 times!\n");
			break;
		}
	}

	index = 0;
	read_sum = 0;
	while (index < 80) {
		/* FIX ME dead loop protection??? */
		sata_phy_cr_read(&adc_out_reg, mmio);
		if (adc_out_reg & 0x400) {
			read_sum = read_sum + (adc_out_reg & 0x3FF);
			index = index + 1;
		}
	}
	/* Use the U32 to make 1000 precision */
	m2 = (read_sum * 1000) / 80;

	/* restore the status  */
	/* mpll_tst.meas_iv */
	mpll_test_reg = (mpll_test_reg & 0xE03) | (str1) << 2;
	/* rtune_ctl.mode */
	rtune_ctl_reg = (rtune_ctl_reg & 0xFFC) | (str2);
	/* dac_ctl.dac_mode */
	dac_ctl_reg = (dac_ctl_reg & 0x8FF) | (str3) << 12;
	/* rtune_ctl.sel_atbp */
	rtune_ctl_reg = (rtune_ctl_reg & 0xFEF) | (str4) << 4;

	sata_phy_cr_addr(SATA_PHY_CR_CLOCK_MPLL_TST, mmio);
	sata_phy_cr_write(mpll_test_reg, mmio);
	sata_phy_cr_addr(SATA_PHY_CR_CLOCK_DAC_CTL, mmio);
	sata_phy_cr_write(dac_ctl_reg, mmio);
	sata_phy_cr_addr(SATA_PHY_CR_CLOCK_RTUNE_CTL, mmio);
	sata_phy_cr_write(rtune_ctl_reg, mmio);

	/* Compute temperature */
	if (!(m2 / 1000))
		m2 = 1000;
	a = (m2 - m1) / (m2 / 1000);
	temp = ((((-559) * a) / 1000) * a) / 1000 + (1379) * a / 1000 + (-458);

	iounmap(mmio);
	return sprintf(buf, "SATA AHCI current temperature:%d\n", temp);
}

static DEVICE_ATTR(temperature, S_IRUGO, sata_ahci_current_tmp, NULL);

static struct attribute *fsl_sata_ahci_attr[] = {
	&dev_attr_temperature.attr,
	NULL
};

static const struct attribute_group fsl_sata_ahci_group = {
	.attrs = fsl_sata_ahci_attr,
};

/* HW Initialization, if return 1, initialization is failed. */
static int sata_init(struct device *dev)
{
	void __iomem *mmio;
	struct clk *clk;
	int ret, rc = 0;
	u32 tmpdata;

	clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(clk))
		printk(KERN_ERR "IMX SATA can't get clock.\n");
	clk_enable(clk);

	mmio = ioremap(MX53_SATA_BASE_ADDR, SZ_2K);
	if (mmio == NULL) {
		printk(KERN_ERR "Failed to map SATA REGS\n");
		return 1;
	}

	tmpdata = readl(mmio + HOST_CAP);
	if (!(tmpdata & HOST_CAP_SSS)) {
		tmpdata |= HOST_CAP_SSS;
		writel(tmpdata, mmio + HOST_CAP);
	}

	if (!(readl(mmio + HOST_PORTS_IMPL) & 0x1))
		writel((readl(mmio + HOST_PORTS_IMPL) | 0x1),
			mmio + HOST_PORTS_IMPL);

	/* Get the AHB clock rate, and configure the TIMER1MS reg */
	clk = clk_get(NULL, "ahb_clk");
	if (IS_ERR(clk))
		printk(KERN_ERR "IMX SATA can't get AHB clock.\n");
	tmpdata = clk_get_rate(clk) / 1000;
	writel(tmpdata, mmio + HOST_TIMER1MS);

	/* write addr */
	tmpdata = PHY_CR_CLOCK_FREQ_OVRD;
	writel(tmpdata, mmio + PORT_PHY_CTL);
	/* capture addr */
	tmpdata |= PORT_PHY_CTL_CAP_ADR_LOC;
	/* Wait for ack */
	if (write_phy_ctl_ack_polling(tmpdata, mmio, 100, 1)) {
		rc = 1;
		goto err0;
	}

	/* deassert cap data */
	tmpdata &= 0xFFFF;
	/* wait for ack de-assertion */
	if (write_phy_ctl_ack_polling(tmpdata, mmio, 100, 0)) {
		rc = 1;
		goto err0;
	}

	/* write data */
	/* Configure the PHY CLK input refer to different OSC
	 * For 25MHz, pre[13,14]:01, ncy[12,8]:06,
	 * ncy5[7,6]:02, int_ctl[5,3]:0, prop_ctl[2,0]:7.
	 * For 50MHz, pre:00, ncy:06, ncy5:02, int_ctl:0, prop_ctl:7.
	 */
	/* EVK revA */
	if (board_is_mx53_evk_a())
		tmpdata = (0x1 << 15) | (0x1 << 13) | (0x6 << 8)
			| (0x2 << 6) | 0x7;
	/* Others are 50MHz */
	else
		tmpdata = (0x1 << 15) | (0x0 << 13) | (0x6 << 8)
			| (0x2 << 6) | 0x7;

	writel(tmpdata, mmio + PORT_PHY_CTL);
	/* capture data */
	tmpdata |= PORT_PHY_CTL_CAP_DAT_LOC;
	/* wait for ack */
	if (write_phy_ctl_ack_polling(tmpdata, mmio, 100, 1)) {
		rc = 1;
		goto err0;
	}

	/* deassert cap data */
	tmpdata &= 0xFFFF;
	/* wait for ack de-assertion */
	if (write_phy_ctl_ack_polling(tmpdata, mmio, 100, 0)) {
		rc = 1;
		goto err0;
	}

	/* assert wr signal and wait for ack */
	if (write_phy_ctl_ack_polling(PORT_PHY_CTL_WRITE_LOC, mmio, 100, 1)) {
		rc = 1;
		goto err0;
	}
	/* deassert rd _signal and wait for ack de-assertion */
	if (write_phy_ctl_ack_polling(0, mmio, 100, 0)) {
		rc = 1;
		goto err0;
	}

	msleep(10);

	/* Add the temperature monitor */
	ret = sysfs_create_group(&dev->kobj, &fsl_sata_ahci_group);
	if (ret)
		sysfs_remove_group(&dev->kobj, &fsl_sata_ahci_group);

err0:
	iounmap(mmio);
	return rc;
}

static void sata_exit(struct device *dev)
{
	struct clk *clk;

	sysfs_remove_group(&dev->kobj, &fsl_sata_ahci_group);
	clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(clk))
		printk(KERN_ERR "IMX SATA can't get clock.\n");
	clk_disable(clk);
	clk_put(clk);
}

struct ahci_platform_data sata_data = {
	.init = sata_init,
	.exit = sata_exit,
};

