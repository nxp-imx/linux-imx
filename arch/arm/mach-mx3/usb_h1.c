/*
 * Copyright 2005-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <mach/arc_otg.h>
#include <mach/hardware.h>
#include "usb.h"

static struct fsl_usb2_platform_data usbh1_config = {
	.name              = "Host 1",
	.platform_init     = fsl_usb_host_init,
	.platform_uninit   = fsl_usb_host_uninit,
	.operating_mode    = FSL_USB2_MPH_HOST,
	.phy_mode          = FSL_USB2_PHY_SERIAL,
	.power_budget      = 500,		/* 500 mA max power */
	.gpio_usb_active   = gpio_usbh1_active,
	.gpio_usb_inactive = gpio_usbh1_inactive,
	.transceiver       = "serial",
};

static struct resource usbh1_resources[] = {
	[0] = {
		.start = (u32) (USB_H1REGS_BASE),
		.end   = (u32) (USB_H1REGS_BASE + 0x1ff),
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = MXC_INT_USB1,
		.flags = IORESOURCE_IRQ,
	},
};

static int __init usbh1_init(void)
{
	pr_debug("%s: \n", __func__);

	host_pdev_register(usbh1_resources, ARRAY_SIZE(usbh1_resources),
			   &usbh1_config);
	return 0;
}
module_init(usbh1_init);
