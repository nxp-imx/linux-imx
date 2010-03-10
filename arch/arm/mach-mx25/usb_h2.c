/*
 * Copyright 2005-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
	.phy_mode          = FSL_USB2_PHY_SERIAL,
	.power_budget      = 500,		/* via RT9702 */
	.gpio_usb_active   = gpio_usbh2_active,
	.gpio_usb_inactive = gpio_usbh2_inactive,
	.transceiver       = "serial",		/* on-chip */
};

static struct resource usbh2_resources[] = {
	[0] = {
	       .start = (u32) (USB_H2REGS_BASE),
	       .end   = (u32) (USB_H2REGS_BASE + 0x1ff),
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_USB_HTG,
	       .flags = IORESOURCE_IRQ,
	       },
};

void usbh2_get_xcvr_power(struct device *dev)
{
	struct regulator *usbh2_regux;

	usbh2_regux = regulator_get(dev, "GPO1");
	regulator_enable(usbh2_regux);
	((struct fsl_usb2_platform_data *)dev->platform_data)->
	    xcvr_pwr->regu1 = usbh2_regux;

	usbh2_regux = regulator_get(dev, "GPO3");
	regulator_enable(usbh2_regux);
	((struct fsl_usb2_platform_data *)dev->platform_data)->
	    xcvr_pwr->regu2 = usbh2_regux;
}
EXPORT_SYMBOL(usbh2_get_xcvr_power);

void usbh2_put_xcvr_power(struct device *dev)
{
	struct regulator *usbh2_regux;

	usbh2_regux = ((struct fsl_usb2_platform_data *)dev->
		       platform_data)->xcvr_pwr->regu2;
	regulator_disable(usbh2_regux);
	regulator_put(usbh2_regux);

	usbh2_regux = ((struct fsl_usb2_platform_data *)dev->
		       platform_data)->xcvr_pwr->regu1;
	regulator_disable(usbh2_regux);
	regulator_put(usbh2_regux);
}
EXPORT_SYMBOL(usbh2_put_xcvr_power);

static int __init usbh2_init(void)
{
	pr_debug("%s: \n", __func__);

	host_pdev_register(usbh2_resources, ARRAY_SIZE(usbh2_resources),
			   &usbh2_config);
	return 0;
}
module_init(usbh2_init);
