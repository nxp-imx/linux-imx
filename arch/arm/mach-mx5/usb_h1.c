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
#include <linux/clk.h>
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

static void _wake_up_enable(struct fsl_usb2_platform_data *pdata, bool enable)
{
	if (enable)
		USBCTRL |= UCTRL_H1WIE;
	else
		USBCTRL &= ~UCTRL_H1WIE;
}

static void usbotg_clock_gate(bool on)
{
	struct clk *usboh3_clk = clk_get(NULL, "usboh3_clk");
	struct clk *usb_ahb_clk = clk_get(NULL, "usb_ahb_clk");

	if (on) {
		clk_enable(usb_ahb_clk);
		clk_enable(usboh3_clk);
	} else {
		clk_disable(usboh3_clk);
		clk_disable(usb_ahb_clk);
	}

	clk_put(usboh3_clk);
	clk_put(usb_ahb_clk);
}

static int fsl_usb_host_init_ext(struct platform_device *pdev)
{
	int ret = fsl_usb_host_init(pdev);
	if (ret)
		return ret;

	/* setback USBH1_STP to be function */
	mxc_request_iomux(MX51_PIN_USBH1_STP, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_USBH1_STP, PAD_CTL_SRE_FAST |
			  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE |
			  PAD_CTL_HYS_ENABLE | PAD_CTL_DDR_INPUT_CMOS |
			  PAD_CTL_DRV_VOT_LOW);
	gpio_free(IOMUX_TO_GPIO(MX51_PIN_USBH1_STP));

	/* disable remote wakeup irq */
	USBCTRL &= ~UCTRL_H1WIE;
	return 0;
}

static struct fsl_usb2_platform_data usbh1_config = {
	.name = "Host 1",
	.platform_init = fsl_usb_host_init_ext,
	.platform_uninit = fsl_usb_host_uninit,
	.operating_mode = FSL_USB2_MPH_HOST,
	.phy_mode = FSL_USB2_PHY_ULPI,
	.power_budget = 500,	/* 500 mA max power */
	.gpio_usb_active = gpio_usbh1_active,
	.gpio_usb_inactive = gpio_usbh1_inactive,
	.wake_up_enable = _wake_up_enable,
	.usb_clock_for_pm  = usbotg_clock_gate,
	.transceiver = "isp1504",
};

void __init mx51_usbh1_init(void)
{
	mxc_register_device(&mxc_usbh1_device, &usbh1_config);
}

