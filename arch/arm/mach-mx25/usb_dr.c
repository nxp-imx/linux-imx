/*
 * Copyright (C) 2005-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <mach/hardware.h>
#include <mach/arc_otg.h>
#include "usb.h"

static void _wake_up_enable(struct fsl_usb2_platform_data *pdata, bool enable);

/*
 * platform data structs
 *	- Which one to use is determined by CONFIG options in usb.h
 *	- operating_mode plugged at run time
 */
static struct fsl_usb2_platform_data __maybe_unused dr_utmi_config = {
	.name              = "DR",
	.platform_init     = usbotg_init,
	.platform_uninit   = usbotg_uninit,
	.phy_mode          = FSL_USB2_PHY_UTMI_WIDE,
	.power_budget      = 500,		/* via RT9706 */
	.gpio_usb_active   = gpio_usbotg_utmi_active,
	.gpio_usb_inactive = gpio_usbotg_utmi_inactive,
	.transceiver       = "utmi",
	.wake_up_enable = _wake_up_enable,
};

/*
 * OTG resources
 */
static struct resource otg_resources[] = {
	[0] = {
	       .start = (u32)(USB_OTGREGS_BASE),
	       .end   = (u32)(USB_OTGREGS_BASE + 0x1ff),
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MX25_INT_USB_OTG,
	       .flags = IORESOURCE_IRQ,
	       },
};

/*
 * UDC resources (same as OTG resource)
 */
static struct resource udc_resources[] = {
	[0] = {
	       .start = (u32)(USB_OTGREGS_BASE),
	       .end   = (u32)(USB_OTGREGS_BASE + 0x1ff),
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_USB_OTG,
	       .flags = IORESOURCE_IRQ,
	       },
};


static u64 dr_udc_dmamask = ~(u32) 0;
static void dr_udc_release(struct device *dev)
{
}

static u64 dr_otg_dmamask = ~(u32) 0;
static void dr_otg_release(struct device *dev)
{
}

/*
 * platform device structs
 *	dev.platform_data field plugged at run time
 */
static struct platform_device __maybe_unused dr_udc_device = {
	.name = "fsl-usb2-udc",
	.id   = -1,
	.dev  = {
		.release           = dr_udc_release,
		.dma_mask          = &dr_udc_dmamask,
		.coherent_dma_mask = 0xffffffff,
	},
	.resource      = udc_resources,
	.num_resources = ARRAY_SIZE(udc_resources),
};

static struct platform_device __maybe_unused dr_otg_device = {
	.name = "fsl-usb2-otg",
	.id = -1,
	.dev = {
		.release           = dr_otg_release,
		.dma_mask          = &dr_otg_dmamask,
		.coherent_dma_mask = 0xffffffff,
	},
	.resource      = otg_resources,
	.num_resources = ARRAY_SIZE(otg_resources),
};

static void _wake_up_enable(struct fsl_usb2_platform_data *pdata, bool enable)
{
	if (get_usb_mode(pdata) == FSL_USB_DR_DEVICE) {
		if (enable)
			USBCTRL |= (UCTRL_OWIE | UCTRL_VBUS_WKUP_EN);
		else {
			USBCTRL &= ~UCTRL_OWIE;
			USBCTRL &= ~UCTRL_VBUS_WKUP_EN;
		}
	} else {
		if (enable)
			USBCTRL |= UCTRL_OWIE;
		else
			USBCTRL &= ~UCTRL_OWIE;
	}
}

static int __init usb_dr_init(void)
{
	pr_debug("%s: \n", __func__);

	dr_register_otg();
	dr_register_host(otg_resources, ARRAY_SIZE(otg_resources));
	dr_register_udc();

	return 0;
}

module_init(usb_dr_init);
