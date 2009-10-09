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
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <mach/hardware.h>
#include <mach/arc_otg.h>
#include "usb.h"

static void usbotg_pm_clock(bool on);
static void _wake_up_enable(struct fsl_usb2_platform_data *pdata, bool enable);

/*
 * platform data structs
 * 	- Which one to use is determined by CONFIG options in usb.h
 * 	- operating_mode plugged at run time
 */
static struct fsl_usb2_platform_data __maybe_unused dr_utmi_config = {
	.name              = "DR",
	.platform_init     = usbotg_init_ext,
	.platform_uninit   = usbotg_uninit_ext,
	.phy_mode          = FSL_USB2_PHY_UTMI_WIDE,
	.power_budget      = 500,	/* 500 mA max power */
	.gpio_usb_active   = gpio_usbotg_hs_active,
	.gpio_usb_inactive = gpio_usbotg_hs_inactive,
	.usb_clock_for_pm  = usbotg_pm_clock,
	.wake_up_enable = _wake_up_enable,
	.transceiver       = "utmi",
};


/*
 * resources
 */
static struct resource otg_resources[] = {
	[0] = {
		.start = (u32)(OTG_BASE_ADDR),
		.end   = (u32)(OTG_BASE_ADDR + 0x620),
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

/* Notes: configure USB clock*/
static int usbotg_init_ext(struct platform_device *pdev)
{
	struct clk *usb_clk, *usboh2_clk;
	int ret;

	usboh2_clk = clk_get(NULL, "usboh2_clk");
	clk_enable(usboh2_clk);

	usb_clk = clk_get(NULL, "usb_phy_clk");
	clk_enable(usb_clk);
	clk_put(usb_clk);

	ret = usbotg_init(pdev);

	/* this clock is no use after set portsc PTS bit */
	clk_disable(usboh2_clk);
	clk_put(usboh2_clk);

	return ret;
}

static void usbotg_pm_clock(bool on)
{
	struct clk *usb_clk;

	usb_clk = clk_get(NULL, "usb_phy_clk");
	/* close and open usb phy clock for suspend and resume */
	if (on) {
		clk_enable(usb_clk);
	} else {
		clk_disable(usb_clk);
	}
	clk_put(usb_clk);
}

static void usbotg_uninit_ext(struct fsl_usb2_platform_data *pdata)
{
	usbotg_pm_clock(false);
	usbotg_uninit(pdata);
}

static void _wake_up_enable(struct fsl_usb2_platform_data *pdata, bool enable)
{
	if (get_usb_mode(pdata) == FSL_USB_DR_DEVICE) {
		if (enable) {
			USBCTRL |= (UCTRL_OWIE | UCTRL_VBUS_WKUP_EN);
			USB_PHY_CTR_FUNC |= USB_UTMI_PHYCTRL_CONF2;
		} else {
			USBCTRL &= ~UCTRL_OWIE;
			USBCTRL &= ~UCTRL_VBUS_WKUP_EN;
			USB_PHY_CTR_FUNC &= ~USB_UTMI_PHYCTRL_CONF2;
		}
	}
}

static int __init usb_dr_init(void)
{
	pr_debug("%s: \n", __func__);

	dr_register_otg();
	dr_register_host(otg_resources, ARRAY_SIZE(otg_resources));
	dr_register_udc();
	device_init_wakeup(&(dr_udc_device.dev), 1);
	return 0;
}

module_init(usb_dr_init);
