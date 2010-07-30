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
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/gpio.h>

#include <mach/irqs.h>
#include "usb.h"
#include "mx28_pins.h"

#define USB_POWER_ENABLE MXS_PIN_TO_GPIO(PINID_AUART2_TX)

static void usb_host_phy_resume(struct fsl_usb2_platform_data *plat)
{
	fsl_platform_set_usb_phy_dis(plat, 0);
}

static int usbotg_init_ext(struct platform_device *pdev)
{
	struct clk *usb_clk;

	usb_clk = clk_get(NULL, "usb_clk0");
	clk_enable(usb_clk);
	clk_put(usb_clk);

	return usbotg_init(pdev);
}

/*
 * platform data structs
 * 	- Which one to use is determined by CONFIG options in usb.h
 * 	- operating_mode plugged at run time
 */
static struct fsl_usb2_platform_data __maybe_unused dr_utmi_config = {
	.name              = "DR",
	.platform_init     = usbotg_init_ext,
	.platform_uninit   = usbotg_uninit,
	.phy_mode          = FSL_USB2_PHY_UTMI_WIDE,
	.power_budget      = 500,	/* 500 mA max power */
	.platform_resume = usb_host_phy_resume,
	.transceiver       = "utmi",
	.phy_regs          = USBPHY0_PHYS_ADDR,
};

/*
 * resources
 */
static struct resource otg_resources[] = {
	[0] = {
		.start	= (u32)USBCTRL0_PHYS_ADDR,
		.end	= (u32)(USBCTRL0_PHYS_ADDR + 0x1ff),
		.flags	= IORESOURCE_MEM,
	},

	[1] = {
		.start	= IRQ_USB0,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 dr_udc_dmamask = ~(u32) 0;
static void dr_udc_release(struct device *dev)
{
}

/*
 * platform device structs
 * 	dev.platform_data field plugged at run time
 */
static struct platform_device dr_udc_device = {
	.name = "fsl-usb2-udc",
	.id   = -1,
	.dev  = {
		.release           = dr_udc_release,
		.dma_mask          = &dr_udc_dmamask,
		.coherent_dma_mask = 0xffffffff,
	},
	.resource      = otg_resources,
	.num_resources = ARRAY_SIZE(otg_resources),
};

static u64 dr_otg_dmamask = ~(u32) 0;
static void dr_otg_release(struct device *dev)
{}

static struct platform_device __maybe_unused dr_otg_device = {
	.name = "fsl-usb2-otg",
	.id = -1,
	.dev  = {
		.release           = dr_otg_release,
		.dma_mask          = &dr_otg_dmamask,
		.coherent_dma_mask = 0xffffffff,
	},
	.resource      = otg_resources,
	.num_resources = ARRAY_SIZE(otg_resources),
};


static int __init usb_dr_init(void)
{
	pr_debug("%s: \n", __func__);

	dr_register_otg();
	dr_register_host(otg_resources, ARRAY_SIZE(otg_resources));
	dr_register_udc();

	PDATA->change_ahb_burst = 1;
	PDATA->ahb_burst_mode = 0;
	return 0;
}

void fsl_phy_usb_utmi_init(struct fsl_xcvr_ops *this)
{
}

void fsl_phy_usb_utmi_uninit(struct fsl_xcvr_ops *this)
{
}

/*!
 * set vbus power
 *
 * @param       view  viewport register
 * @param       on    power on or off
 */
void fsl_phy_set_power(struct fsl_xcvr_ops *this,
			struct fsl_usb2_platform_data *pdata, int on)
{
	/* USB_POWER_ENABLE_PIN have request at pin init*/
	if (pdata->phy_regs != USBPHY1_PHYS_ADDR) {
		gpio_direction_output(USB_POWER_ENABLE, on);
		gpio_set_value(USB_POWER_ENABLE, on);
	}
}

#ifdef CONFIG_MXS_VBUS_CURRENT_DRAW
	fs_initcall(usb_dr_init);
#else
	subsys_initcall(usb_dr_init);
#endif
