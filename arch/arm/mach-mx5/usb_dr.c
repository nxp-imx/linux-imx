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
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <mach/arc_otg.h>
#include <mach/hardware.h>
#include "usb.h"

static int usbotg_init_ext(struct platform_device *pdev);
static void usbotg_uninit_ext(struct fsl_usb2_platform_data *pdata);
static void _wake_up_enable(struct fsl_usb2_platform_data *pdata, bool enable);
static void usbotg_clock_gate(bool on);

/*
 * platform data structs
 * 	- Which one to use is determined by CONFIG options in usb.h
 * 	- operating_mode plugged at run time
 */
static struct fsl_usb2_platform_data dr_utmi_config = {
	.name              = "DR",
	.platform_init     = usbotg_init_ext,
	.platform_uninit   = usbotg_uninit_ext,
	.phy_mode          = FSL_USB2_PHY_UTMI_WIDE,
	.power_budget      = 500,		/* 500 mA max power */
	.gpio_usb_active   = gpio_usbotg_hs_active,
	.gpio_usb_inactive = gpio_usbotg_hs_inactive,
	.usb_clock_for_pm  = usbotg_clock_gate,
	.wake_up_enable = _wake_up_enable,
	.transceiver       = "utmi",
};

/* Notes: configure USB clock*/
static int usbotg_init_ext(struct platform_device *pdev)
{
	struct clk *usb_clk;

	usb_clk = clk_get(NULL, "usboh3_clk");
	clk_enable(usb_clk);
	clk_put(usb_clk);

	usb_clk = clk_get(&pdev->dev, "usb_phy1_clk");
	clk_enable(usb_clk);
	clk_put(usb_clk);

	/*derive clock from oscillator */
	usb_clk = clk_get(NULL, "usb_utmi_clk");
	clk_disable(usb_clk);
	clk_put(usb_clk);

	return usbotg_init(pdev);
}

static void usbotg_uninit_ext(struct fsl_usb2_platform_data *pdata)
{
	struct clk *usb_clk;

	usb_clk = clk_get(NULL, "usboh3_clk");
	clk_disable(usb_clk);
	clk_put(usb_clk);

	usb_clk = clk_get(&pdata->pdev->dev, "usb_phy1_clk");
	clk_disable(usb_clk);
	clk_put(usb_clk);

	usbotg_uninit(pdata);
}

static void _wake_up_enable(struct fsl_usb2_platform_data *pdata, bool enable)
{
	if (get_usb_mode(pdata) == FSL_USB_DR_DEVICE) {
		if (enable) {
			USBCTRL |= UCTRL_OWIE;
			USBCTRL_HOST2 |= UCTRL_H2OVBWK_EN;
			USB_PHY_CTR_FUNC |= USB_UTMI_PHYCTRL_CONF2;
		} else {
			USBCTRL &= ~UCTRL_OWIE;
			USBCTRL_HOST2 &= ~UCTRL_H2OVBWK_EN;
			USB_PHY_CTR_FUNC &= ~USB_UTMI_PHYCTRL_CONF2;
		}
	} else {
		if (enable) {
			USBCTRL |= UCTRL_OWIE;
			USBCTRL_HOST2 |= (1 << 5);
		} else {
			USBCTRL &= ~UCTRL_OWIE;
			USBCTRL_HOST2 &= ~(1 << 5);
		}
	}
}

static void usbotg_clock_gate(bool on)
{
	struct clk *usb_clk;

	if (on) {
		usb_clk = clk_get(NULL, "usb_ahb_clk");
		clk_enable(usb_clk);
		clk_put(usb_clk);

		usb_clk = clk_get(NULL, "usboh3_clk");
		clk_enable(usb_clk);
		clk_put(usb_clk);

		usb_clk = clk_get(NULL, "usb_phy1_clk");
		clk_enable(usb_clk);
		clk_put(usb_clk);

		/*derive clock from oscillator */
		usb_clk = clk_get(NULL, "usb_utmi_clk");
		clk_disable(usb_clk);
		clk_put(usb_clk);
	} else {
		usb_clk = clk_get(NULL, "usboh3_clk");
		clk_disable(usb_clk);
		clk_put(usb_clk);

		usb_clk = clk_get(NULL, "usb_phy1_clk");
		clk_disable(usb_clk);
		clk_put(usb_clk);

		usb_clk = clk_get(NULL, "usb_ahb_clk");
		clk_disable(usb_clk);
		clk_put(usb_clk);
	}
}

void __init mx5_usb_dr_init(void)
{
#ifdef CONFIG_USB_OTG
	dr_utmi_config.operating_mode = FSL_USB2_DR_OTG;
	platform_device_add_data(&mxc_usbdr_otg_device, &dr_utmi_config, sizeof(dr_utmi_config));
	platform_device_register(&mxc_usbdr_otg_device);
#endif
#ifdef CONFIG_USB_EHCI_ARC_OTG
	dr_utmi_config.operating_mode = DR_HOST_MODE;
	platform_device_add_data(&mxc_usbdr_host_device, &dr_utmi_config, sizeof(dr_utmi_config));
	platform_device_register(&mxc_usbdr_host_device);
#endif
#ifdef CONFIG_USB_GADGET_ARC
	dr_utmi_config.operating_mode = DR_UDC_MODE;
	platform_device_add_data(&mxc_usbdr_udc_device, &dr_utmi_config, sizeof(dr_utmi_config));
	platform_device_register(&mxc_usbdr_udc_device);
#endif
}
