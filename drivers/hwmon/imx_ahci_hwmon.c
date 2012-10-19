/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/hwmon.h>

#include <mach/mx53.h>
#include <mach/ahci_sata.h>

/**
 * struct imx_ahci_hwmon - hwmon information
 * @lock: Access lock to serialise the conversions.
 * @hwmon_dev: The hwmon device we created.
*/
struct imx_ahci_hwmon {
	struct mutex		lock;
	struct device		*hwmon_dev;
};

static ssize_t imx_ahci_hwmon_name_show(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "imx-ahci-hwmon\n");
}

/**
 * imx_ahci_hwmon_temp_show - show value of the temperature
 * @dev: The device that the attribute belongs to.
 * @attr: The attribute being read.
 * @buf: The result buffer.
 *
 * Read a value from the IMX AHCI temperature monitor.
 */
static ssize_t imx_ahci_hwmon_temp_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	void __iomem *mmio;
	u32 mpll_test_reg, rtune_ctl_reg, dac_ctl_reg, adc_out_reg;
	u32 str1, str2, str3, str4, read_sum, index, port_phy_ctl;
	int m1, m2, a, temp, ret;
	struct clk *sata_clk, *sata_ref_clk;
	struct imx_ahci_hwmon *hwmon;

	hwmon = platform_get_drvdata(to_platform_device(dev));

	ret = mutex_lock_interruptible(&hwmon->lock);
	if (ret < 0)
		return ret;

	/* initialize the HW.(kinds of clocks) */
	sata_clk = clk_get(NULL, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		dev_err(dev, "no sata clock.\n");
		return PTR_ERR(sata_clk);
	}
	clk_enable(sata_clk);
	sata_ref_clk = clk_get(NULL, "usb_phy1_clk");
	if (IS_ERR(sata_ref_clk)) {
		dev_err(dev, "no sata clock.\n");
		return PTR_ERR(sata_ref_clk);
	}
	clk_enable(sata_ref_clk);

	/* map the IO addr */
	mmio = ioremap(MX53_SATA_BASE_ADDR, SZ_2K);
	if (mmio == NULL) {
		dev_err(dev, "Failed to map SATA REGS\n");
		return -1;
	}

	/* Disable PDDQ mode if it is enabled */
	port_phy_ctl = readl(mmio + PORT_PHY_CTL);
	if (port_phy_ctl & PORT_PHY_CTL_PDDQ_LOC)
		writel(port_phy_ctl & ~PORT_PHY_CTL_PDDQ_LOC, mmio + PORT_PHY_CTL);

	/* check rd-wr to reg */
	read_sum = 0;
	sata_phy_cr_addr(SATA_PHY_CR_CLOCK_CRCMP_LT_LIMIT, mmio);
	sata_phy_cr_write(read_sum, mmio);
	sata_phy_cr_read(&read_sum, mmio);
	if ((read_sum & 0xffff) != 0)
		dev_err(dev, "Read/Write REG error, 0x%x!\n", read_sum);

	sata_phy_cr_write(0x5A5A, mmio);
	sata_phy_cr_read(&read_sum, mmio);
	if ((read_sum & 0xffff) != 0x5A5A)
		dev_err(dev, "Read/Write REG error, 0x%x!\n", read_sum);

	sata_phy_cr_write(0x1234, mmio);
	sata_phy_cr_read(&read_sum, mmio);
	if ((read_sum & 0xffff) != 0x1234)
		dev_err(dev, "Read/Write REG error, 0x%x!\n", read_sum);

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
			dev_err(dev, "Read REG more than 100000 times!\n");
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
			dev_err(dev, "Read REG more than 100000 times!\n");
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

	/* Set the Port Phy ctl back */
	writel(port_phy_ctl, mmio + PORT_PHY_CTL);

	iounmap(mmio);

	/* Release the clocks */
	clk_disable(sata_ref_clk);
	clk_put(sata_ref_clk);
	clk_disable(sata_clk);
	clk_put(sata_clk);
	mutex_unlock(&hwmon->lock);

	return sprintf(buf, "%d\n", temp * 1000);
}

static DEVICE_ATTR(name, S_IRUGO, imx_ahci_hwmon_name_show, NULL);
static DEVICE_ATTR(temp1_input, S_IRUGO, imx_ahci_hwmon_temp_show, NULL);

static struct attribute *imx_ahci_hwmon_attrs[] = {
	&dev_attr_name.attr,
	&dev_attr_temp1_input.attr,
	NULL
};

static struct attribute_group imx_ahci_hwmon_attrgroup = {
	.attrs	= imx_ahci_hwmon_attrs,
};

/**
 * imx_ahci_hwmon_probe - device probe entry.
 * @dev: The device being probed.
*/
static int __devinit imx_ahci_hwmon_probe(struct platform_device *dev)
{
	struct imx_ahci_hwmon *hwmon;
	int ret = 0;

	hwmon = kzalloc(sizeof(struct imx_ahci_hwmon), GFP_KERNEL);
	if (hwmon == NULL) {
		dev_err(&dev->dev, "no memory\n");
		return -ENOMEM;
	}

	platform_set_drvdata(dev, hwmon);

	/* add attributes for device. */
	ret = sysfs_create_group(&dev->dev.kobj, &imx_ahci_hwmon_attrgroup);
	if (ret)
		goto err_mem;

	/* register with the hwmon core */

	hwmon->hwmon_dev = hwmon_device_register(&dev->dev);
	if (IS_ERR(hwmon->hwmon_dev)) {
		dev_err(&dev->dev, "error registering with hwmon\n");
		ret = PTR_ERR(hwmon->hwmon_dev);
		goto err_raw_attribute;
	}

	mutex_init(&hwmon->lock);

	return 0;

err_raw_attribute:
       sysfs_remove_group(&dev->dev.kobj, &imx_ahci_hwmon_attrgroup);

err_mem:
	platform_set_drvdata(dev, NULL);
	kfree(hwmon);
	return ret;
}

static int __devexit imx_ahci_hwmon_remove(struct platform_device *dev)
{
	struct imx_ahci_hwmon *hwmon = platform_get_drvdata(dev);

	mutex_destroy(&hwmon->lock);
	sysfs_remove_group(&dev->dev.kobj, &imx_ahci_hwmon_attrgroup);
	hwmon_device_unregister(hwmon->hwmon_dev);
	platform_set_drvdata(dev, NULL);
	kfree(hwmon);

	return 0;
}

static struct platform_driver imx_ahci_hwmon_driver = {
	.driver	= {
		.name		= "imx-ahci-hwmon",
		.owner		= THIS_MODULE,
	},
	.probe		= imx_ahci_hwmon_probe,
	.remove		= __devexit_p(imx_ahci_hwmon_remove),
};

static int __init imx_ahci_hwmon_init(void)
{
	return platform_driver_register(&imx_ahci_hwmon_driver);
}

static void __exit imx_ahci_hwmon_exit(void)
{
	platform_driver_unregister(&imx_ahci_hwmon_driver);
}

module_init(imx_ahci_hwmon_init);
module_exit(imx_ahci_hwmon_exit);

MODULE_DESCRIPTION("FSL IMX AHCI HWMon driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-ahci-hwmon");
