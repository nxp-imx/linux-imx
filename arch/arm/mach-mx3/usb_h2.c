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
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/usb/fsl_xcvr.h>
#include <linux/regulator/consumer.h>
#include <mach/hardware.h>
#include <mach/arc_otg.h>
#include "usb.h"

static struct fsl_usb2_platform_data usbh2_config = {
	.name              = "Host 2",
	.platform_init     = fsl_usb_host_init,
	.platform_uninit   = fsl_usb_host_uninit,
	.operating_mode    = FSL_USB2_MPH_HOST,
	.phy_mode          = FSL_USB2_PHY_ULPI,
	.power_budget      = 500,		/* 500 mA max power */
	.gpio_usb_active   = gpio_usbh2_active,
	.gpio_usb_inactive = gpio_usbh2_inactive,
	.transceiver       = "isp1504",
};

static struct resource usbh2_resources[] = {
	[0] = {
		.start = (u32) (USB_H2REGS_BASE),
		.end   = (u32) (USB_H2REGS_BASE + 0x1ff),
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = MXC_INT_USB2,
		.flags = IORESOURCE_IRQ,
	},
};

static int __init usbh2_init(void)
{
	pr_debug("%s: \n", __func__);

	if (machine_is_mx31_3ds()) {
		struct regulator *usbh2_regux;
		usbh2_config.xcvr_pwr =
			kmalloc(sizeof(struct fsl_xcvr_power), GFP_KERNEL);
		if (!(usbh2_config.xcvr_pwr))
			return -ENOMEM;

		usbh2_regux = regulator_get(NULL, "GPO1");
		usbh2_config.xcvr_pwr->regu1 = usbh2_regux;
		usbh2_regux = regulator_get(NULL, "GPO3");
		usbh2_config.xcvr_pwr->regu2 = usbh2_regux;
	}

	host_pdev_register(usbh2_resources, ARRAY_SIZE(usbh2_resources),
			   &usbh2_config);
	return 0;
}
module_init(usbh2_init);
