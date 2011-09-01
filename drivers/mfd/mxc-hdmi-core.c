/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/clk.h>

#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <asm/mach-types.h>

#include <mach/mxc_hdmi.h>
#include <linux/mfd/mxc-hdmi-core.h>

struct mxc_hdmi_data {
	struct clk *isfr_clk;
	struct platform_device *pdev;
	unsigned long __iomem *reg_base;
	unsigned long reg_phys_base;
	struct device *dev;
};

static unsigned long hdmi_base;

int mxc_hdmi_pixel_clk;
int mxc_hdmi_ratio;

u8 hdmi_readb(unsigned int reg)
{
	u8 value;

	value = __raw_readb(hdmi_base + reg);

	pr_debug("hdmi rd: 0x%04x = 0x%02x\n", reg, value);

	return value;
}

void hdmi_writeb(u8 value, unsigned int reg)
{
	pr_debug("hdmi wr: 0x%04x = 0x%02x\n", reg, value);
	__raw_writeb(value, hdmi_base + reg);
}

void hdmi_mask_writeb(u8 data, unsigned int reg, u8 shift, u8 mask)
{
	u8 value = hdmi_readb(reg) & ~mask;
	value |= (data << shift) & mask;
	hdmi_writeb(value, reg);
}

unsigned int hdmi_read4(unsigned int reg)
{
	/* read a four byte address from registers */
	return (hdmi_readb(reg + 3) << 24) |
		(hdmi_readb(reg + 2) << 16) |
		(hdmi_readb(reg + 1) << 8) |
		hdmi_readb(reg);
}

void hdmi_write4(unsigned int value, unsigned int reg)
{
	/* write a four byte address to hdmi regs */
	hdmi_writeb(value & 0xff, reg);
	hdmi_writeb((value >> 8) & 0xff, reg + 1);
	hdmi_writeb((value >> 16) & 0xff, reg + 2);
	hdmi_writeb((value >> 24) & 0xff, reg + 3);
}

static void initialize_hdmi_ih_mutes(void)
{
	u8 ih_mute;

	/*
	 * Boot up defaults are:
	 * HDMI_IH_MUTE   = 0x03 (disabled)
	 * HDMI_IH_MUTE_* = 0x00 (enabled)
	 */

	/* Disable top level interrupt bits in HDMI block */
	ih_mute = hdmi_readb(HDMI_IH_MUTE) |
		  HDMI_IH_MUTE_MUTE_WAKEUP_INTERRUPT |
		  HDMI_IH_MUTE_MUTE_ALL_INTERRUPT;

	hdmi_writeb(ih_mute, HDMI_IH_MUTE);

	/* Disable interrupts in the IH_MUTE_* registers */
	hdmi_writeb(0xff, HDMI_IH_MUTE_FC_STAT0);
	hdmi_writeb(0xff, HDMI_IH_MUTE_FC_STAT1);
	hdmi_writeb(0xff, HDMI_IH_MUTE_FC_STAT2);
	hdmi_writeb(0xff, HDMI_IH_MUTE_AS_STAT0);
	hdmi_writeb(0xff, HDMI_IH_MUTE_PHY_STAT0);
	hdmi_writeb(0xff, HDMI_IH_MUTE_I2CM_STAT0);
	hdmi_writeb(0xff, HDMI_IH_MUTE_CEC_STAT0);
	hdmi_writeb(0xff, HDMI_IH_MUTE_VP_STAT0);
	hdmi_writeb(0xff, HDMI_IH_MUTE_I2CMPHY_STAT0);
	hdmi_writeb(0xff, HDMI_IH_MUTE_AHBDMAAUD_STAT0);

	/* Enable top level interrupt bits in HDMI block */
	ih_mute &= ~(HDMI_IH_MUTE_MUTE_WAKEUP_INTERRUPT |
		    HDMI_IH_MUTE_MUTE_ALL_INTERRUPT);
	hdmi_writeb(ih_mute, HDMI_IH_MUTE);
}

static int mxc_hdmi_core_probe(struct platform_device *pdev)
{
	struct mxc_hdmi_data *hdmi_data;
	struct resource *res;
	int ret = 0;

	/* 100% for 8 bit pixels */
	mxc_hdmi_ratio = 100;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENOENT;

	hdmi_data = kzalloc(sizeof(struct mxc_hdmi_data), GFP_KERNEL);
	if (!hdmi_data) {
		dev_err(&pdev->dev, "Couldn't allocate mxc hdmi mfd device\n");
		return -ENOMEM;
	}
	hdmi_data->pdev = pdev;

	hdmi_data->isfr_clk = clk_get(&hdmi_data->pdev->dev, "hdmi_isfr_clk");
	if (IS_ERR(hdmi_data->isfr_clk)) {
		ret = PTR_ERR(hdmi_data->isfr_clk);
		dev_err(&hdmi_data->pdev->dev,
			"Unable to get HDMI isfr clk: %d\n", ret);
		goto eclkg;
	}

	ret = clk_enable(hdmi_data->isfr_clk);
	if (ret < 0) {
		dev_err(&pdev->dev, "Cannot enable HDMI clock: %d\n", ret);
		goto eclke;
	}

	pr_debug("%s isfr_clk:%d\n", __func__,
		(int)clk_get_rate(hdmi_data->isfr_clk));

	hdmi_data->reg_phys_base = res->start;
	if (!request_mem_region(res->start, resource_size(res),
				dev_name(&pdev->dev))) {
		dev_err(&pdev->dev, "request_mem_region failed\n");
		ret = -EBUSY;
		goto emem;
	}

	hdmi_data->reg_base = ioremap(res->start, resource_size(res));
	if (!hdmi_data->reg_base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto eirq;
	}
	hdmi_base = (unsigned long)hdmi_data->reg_base;

	pr_debug("\n%s hdmi hw base = 0x%08x\n\n", __func__, (int)res->start);

	initialize_hdmi_ih_mutes();

	/* Replace platform data coming in with a local struct */
	platform_set_drvdata(pdev, hdmi_data);

	return ret;

eirq:
	release_mem_region(res->start, resource_size(res));
emem:
	clk_disable(hdmi_data->isfr_clk);
eclke:
	clk_put(hdmi_data->isfr_clk);
eclkg:
	kfree(hdmi_data);
	return ret;
}


static int __exit mxc_hdmi_core_remove(struct platform_device *pdev)
{
	struct mxc_hdmi_data *hdmi_data = platform_get_drvdata(pdev);
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	iounmap(hdmi_data->reg_base);
	release_mem_region(res->start, resource_size(res));

	kfree(hdmi_data);

	return 0;
}

static struct platform_driver mxc_hdmi_core_driver = {
	.driver = {
		.name = "mxc_hdmi_core",
		.owner = THIS_MODULE,
	},
	.remove = __exit_p(mxc_hdmi_core_remove),
};

static int __init mxc_hdmi_core_init(void)
{
	return platform_driver_probe(&mxc_hdmi_core_driver,
				     mxc_hdmi_core_probe);
}

static void __exit mxc_hdmi_core_exit(void)
{
	platform_driver_unregister(&mxc_hdmi_core_driver);
}

subsys_initcall(mxc_hdmi_core_init);
module_exit(mxc_hdmi_core_exit);

MODULE_DESCRIPTION("Core driver for Freescale i.Mx on-chip HDMI");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
