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
#include <mach/arc_otg.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include "usb.h"
#include "iomux.h"


/*
 * USB Host1 HS port
 */
static int gpio_usbh1_active(void)
{
	/* Set USBH1_STP to GPIO and toggle it */
	mxc_request_iomux(MX51_PIN_USBH1_STP, IOMUX_CONFIG_GPIO |
			  IOMUX_CONFIG_SION);
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_USBH1_STP), "usbh1_stp");
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_USBH1_STP), 0);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_USBH1_STP), 1);

	/* Signal only used on MX51-3DS for reset to PHY.*/
	if (machine_is_mx51_3ds()) {
		mxc_request_iomux(MX51_PIN_EIM_D17, IOMUX_CONFIG_ALT1);
		mxc_iomux_set_pad(MX51_PIN_EIM_D17, PAD_CTL_DRV_HIGH |
			  PAD_CTL_HYS_NONE | PAD_CTL_PUE_KEEPER |
			  PAD_CTL_100K_PU | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		gpio_request(IOMUX_TO_GPIO(MX51_PIN_EIM_D17), "eim_d17");
		gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_EIM_D17), 0);
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_D17), 1);
	}

	msleep(100);

	return 0;
}

void gpio_usbh1_setback_stp(void)
{
	/* setback USBH1_STP to be function */
	mxc_request_iomux(MX51_PIN_USBH1_STP, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_USBH1_STP, PAD_CTL_SRE_FAST |
			  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE |
			  PAD_CTL_HYS_ENABLE | PAD_CTL_DDR_INPUT_CMOS |
			  PAD_CTL_DRV_VOT_LOW);
	gpio_free(IOMUX_TO_GPIO(MX51_PIN_USBH1_STP));
}
EXPORT_SYMBOL(gpio_usbh1_setback_stp);

static void gpio_usbh1_inactive(void)
{
	/* Signal only used on MX51-3DS for reset to PHY.*/
	if (machine_is_mx51_3ds()) {
		gpio_free(IOMUX_TO_GPIO(MX51_PIN_EIM_D17));
		mxc_free_iomux(MX51_PIN_EIM_D17, IOMUX_CONFIG_GPIO);
	}

	mxc_free_iomux(MX51_PIN_USBH1_STP, IOMUX_CONFIG_GPIO);
	gpio_free(IOMUX_TO_GPIO(MX51_PIN_USBH1_STP));
}

static struct fsl_usb2_platform_data usbh1_config = {
	.name = "Host 1",
	.platform_init = fsl_usb_host_init,
	.platform_uninit = fsl_usb_host_uninit,
	.operating_mode = FSL_USB2_MPH_HOST,
	.phy_mode = FSL_USB2_PHY_ULPI,
	.power_budget = 500,	/* 500 mA max power */
	.gpio_usb_active = gpio_usbh1_active,
	.gpio_usb_inactive = gpio_usbh1_inactive,
	.transceiver = "isp1504",
};

static struct resource usbh1_resources[] = {
	[0] = {
	       .start = (u32) (USB_H1REGS_BASE),
	       .end = (u32) (USB_H1REGS_BASE + 0x1ff),
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_USB_H1,
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
