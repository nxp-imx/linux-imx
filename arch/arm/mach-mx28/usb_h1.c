/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <mach/arc_otg.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/irqs.h>
#include "usb.h"

static void usb_host_phy_resume(struct fsl_usb2_platform_data *plat)
{
	fsl_platform_set_usb_phy_dis(plat, 0);
}

static int fsl_usb_host_init_ext(struct platform_device *pdev)
{
	struct clk *usb_clk;

	usb_clk = clk_get(NULL, "usb_clk1");
	clk_enable(usb_clk);
	clk_put(usb_clk);

	return fsl_usb_host_init(pdev);
}

static struct fsl_usb2_platform_data usbh1_config = {
	.name = "Host 1",
	.platform_init = fsl_usb_host_init_ext,
	.platform_uninit = fsl_usb_host_uninit,
	.operating_mode = FSL_USB2_MPH_HOST,
	.phy_mode = FSL_USB2_PHY_UTMI_WIDE,
	.power_budget = 500,	/* 500 mA max power */
	.platform_resume = usb_host_phy_resume,
	.transceiver = "utmi",
	.phy_regs = USBPHY1_PHYS_ADDR,
};

static struct resource usbh1_resources[] = {
	[0] = {
	       .start = (u32) (USBCTRL1_PHYS_ADDR),
	       .end = (u32) (USBCTRL1_PHYS_ADDR + 0x1ff),
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = IRQ_USB1,
	       .flags = IORESOURCE_IRQ,
	       },
};

static int __init usbh1_init(void)
{
	pr_debug("%s: \n", __func__);

	host_pdev_register(usbh1_resources,
			ARRAY_SIZE(usbh1_resources), &usbh1_config);

	return 0;
}

module_init(usbh1_init);
