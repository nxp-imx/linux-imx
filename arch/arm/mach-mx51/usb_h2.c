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
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <asm/mach-types.h>
#include <mach/arc_otg.h>
#include "usb.h"
#include "iomux.h"

/*
 * USB Host2 HS port
 */
static int gpio_usbh2_active(void)
{
	/* Set USBH2_STP to GPIO and toggle it */
	mxc_request_iomux(MX51_PIN_EIM_A26, IOMUX_CONFIG_GPIO);
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_EIM_A26), "eim_a26");
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_EIM_A26), 0);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_A26), 1);

	msleep(100);

	return 0;
}

void gpio_usbh2_setback_stp(void)
{
	/* setback USBH2_STP to be function */
	mxc_request_iomux(MX51_PIN_EIM_A26, IOMUX_CONFIG_ALT2);
}
EXPORT_SYMBOL(gpio_usbh2_setback_stp);

static void gpio_usbh2_inactive(void)
{
	gpio_free(IOMUX_TO_GPIO(MX51_PIN_EIM_A26));
	mxc_free_iomux(MX51_PIN_EIM_A26, IOMUX_CONFIG_GPIO);
}

static struct fsl_usb2_platform_data usbh2_config = {
	.name = "Host 2",
	.platform_init = fsl_usb_host_init,
	.platform_uninit = fsl_usb_host_uninit,
	.operating_mode = FSL_USB2_MPH_HOST,
	.phy_mode = FSL_USB2_PHY_ULPI,
	.power_budget = 500,	/* 500 mA max power */
	.gpio_usb_active = gpio_usbh2_active,
	.gpio_usb_inactive = gpio_usbh2_inactive,
	.transceiver = "isp1504",
};

static struct resource usbh2_resources[] = {
	[0] = {
	       .start = (u32) (USB_H2REGS_BASE),
	       .end = (u32) (USB_H2REGS_BASE + 0x1ff),
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_USB_H2,
	       .flags = IORESOURCE_IRQ,
	       },
};

static int __init usbh2_init(void)
{
	pr_debug("%s: \n", __func__);

	if (machine_is_mx51_3ds() ||
	    (machine_is_mx51_babbage() && (cpu_is_mx51_rev(CHIP_REV_2_0) >= 1)))
		return 0;

	host_pdev_register(usbh2_resources, ARRAY_SIZE(usbh2_resources),
			   &usbh2_config);
	return 0;
}

module_init(usbh2_init);
