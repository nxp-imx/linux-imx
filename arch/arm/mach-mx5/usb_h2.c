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
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <mach/arc_otg.h>
#include "usb.h"
#include "iomux.h"
#include "mx51_pins.h"

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

static void gpio_usbh2_inactive(void)
{
	gpio_free(IOMUX_TO_GPIO(MX51_PIN_EIM_A26));
	mxc_free_iomux(MX51_PIN_EIM_A26, IOMUX_CONFIG_GPIO);
}

static int fsl_usb_host_init_ext(struct platform_device *pdev)
{
	int ret = fsl_usb_host_init(pdev);
	if (ret)
		return ret;

	/* setback USBH2_STP to be function */
	mxc_request_iomux(MX51_PIN_EIM_A26, IOMUX_CONFIG_ALT2);

	return 0;
}

static struct fsl_usb2_platform_data usbh2_config = {
	.name = "Host 2",
	.platform_init = fsl_usb_host_init_ext,
	.platform_uninit = fsl_usb_host_uninit,
	.operating_mode = FSL_USB2_MPH_HOST,
	.phy_mode = FSL_USB2_PHY_ULPI,
	.power_budget = 500,	/* 500 mA max power */
	.gpio_usb_active = gpio_usbh2_active,
	.gpio_usb_inactive = gpio_usbh2_inactive,
	.transceiver = "isp1504",
};

void __init mx51_usbh2_init(void)
{
	mxc_register_device(&mxc_usbh2_device, &usbh2_config);
}

