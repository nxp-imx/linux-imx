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
#include <asm/delay.h>
#include "usb.h"
static int usbotg_init_ext(struct platform_device *pdev);
static void usbotg_uninit_ext(struct fsl_usb2_platform_data *pdata);
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

/* Below two macros are used at otg mode to indicate usb mode*/
#define ENABLED_BY_HOST   (0x1 << 0)
#define ENABLED_BY_DEVICE (0x1 << 1)
static u32 wakeup_irq_enable_src; /* only useful at otg mode */
static void __wakeup_irq_enable(bool on, int source)
 {
	/* otg host and device share the OWIE bit, only when host and device
	 * all enable the wakeup irq, we can enable the OWIE bit
	 */
	if (on) {
#ifdef CONFIG_USB_OTG
		wakeup_irq_enable_src |= source;
		if (wakeup_irq_enable_src == (ENABLED_BY_HOST | ENABLED_BY_DEVICE)) {
			USBCTRL |= UCTRL_OWIE;
			USB_PHY_CTR_FUNC |= USB_UTMI_PHYCTRL_CONF2;
		}
#else
		USBCTRL |= UCTRL_OWIE;
		USB_PHY_CTR_FUNC |= USB_UTMI_PHYCTRL_CONF2;
#endif
	} else {
		USB_PHY_CTR_FUNC &= ~USB_UTMI_PHYCTRL_CONF2;
		USBCTRL &= ~UCTRL_OWIE;
		wakeup_irq_enable_src &= ~source;
		/* The interrupt must be disabled for at least 3 clock
		 * cycles of the standby clock(32k Hz) , that is 0.094 ms*/
		udelay(100);
	}
}

static void _host_wakeup_enable(struct fsl_usb2_platform_data *pdata, bool enable)
{
	__wakeup_irq_enable(enable, ENABLED_BY_HOST);
	/* host only care the ID change wakeup event */
	if (enable) {
		pr_debug("host wakeup enable\n");
		USBCTRL_HOST2 |= UCTRL_H2OIDWK_EN;
	} else {
		pr_debug("host wakeup disable\n");
		USBCTRL_HOST2 &= ~UCTRL_H2OIDWK_EN;
		/* The interrupt must be disabled for at least 3 clock
		 * cycles of the standby clock(32k Hz) , that is 0.094 ms*/
		udelay(100);
	}
}

static void _device_wakeup_enable(struct fsl_usb2_platform_data *pdata, bool enable)
{
	__wakeup_irq_enable(enable, ENABLED_BY_DEVICE);
	/* if udc is not used by any gadget, we can not enable the vbus wakeup */
	if (!pdata->port_enables) {
		USBCTRL_HOST2 &= ~UCTRL_H2OVBWK_EN;
		return;
	}
	if (enable) {
		pr_debug("device wakeup enable\n");
		USBCTRL_HOST2 |= UCTRL_H2OVBWK_EN;
	} else {
		pr_debug("device wakeup disable\n");
		USBCTRL_HOST2 &= ~UCTRL_H2OVBWK_EN;
	}
}

static u32 low_power_enable_src; /* only useful at otg mode */
static void __phy_lowpower_suspend(bool enable, int source)
{
	if (enable) {
		low_power_enable_src |= source;
#ifdef CONFIG_USB_OTG
		if (low_power_enable_src == (ENABLED_BY_HOST | ENABLED_BY_DEVICE)) {
			pr_debug("phy lowpower enabled\n");
			UOG_PORTSC1 |= PORTSC_PHCD;
		}
#else
		UOG_PORTSC1 |= PORTSC_PHCD;
#endif
	} else {
		pr_debug("phy lowpower disable\n");
		UOG_PORTSC1 &= ~PORTSC_PHCD;
		low_power_enable_src &= ~source;
	}
}

static void _host_phy_lowpower_suspend(bool enable)
{
	__phy_lowpower_suspend(enable, ENABLED_BY_HOST);
}

static void _device_phy_lowpower_suspend(bool enable)
{
	__phy_lowpower_suspend(enable, ENABLED_BY_DEVICE);
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
		usb_clk = clk_get(NULL, "usb_phy1_clk");
		clk_disable(usb_clk);
		clk_put(usb_clk);

		usb_clk = clk_get(NULL, "usboh3_clk");
		clk_disable(usb_clk);
		clk_put(usb_clk);

		usb_clk = clk_get(NULL, "usb_ahb_clk");
		clk_disable(usb_clk);
		clk_put(usb_clk);
	}
}

void mx5_set_otghost_vbus_func(driver_vbus_func driver_vbus)
{
	dr_utmi_config.platform_driver_vbus = driver_vbus;
}

void __init mx5_usb_dr_init(void)
{
#ifdef CONFIG_USB_OTG
	/* wake_up_enalbe is useless, just for usb_register_remote_wakeup execution*/
	dr_utmi_config.wake_up_enable = _device_wakeup_enable;
	dr_utmi_config.operating_mode = FSL_USB2_DR_OTG;
	platform_device_add_data(&mxc_usbdr_otg_device, &dr_utmi_config, sizeof(dr_utmi_config));
	platform_device_register(&mxc_usbdr_otg_device);
#endif
#ifdef CONFIG_USB_EHCI_ARC_OTG
	dr_utmi_config.operating_mode = DR_HOST_MODE;
	dr_utmi_config.wake_up_enable = _host_wakeup_enable;
	dr_utmi_config.phy_lowpower_suspend = _host_phy_lowpower_suspend;
	platform_device_add_data(&mxc_usbdr_host_device, &dr_utmi_config, sizeof(dr_utmi_config));
	platform_device_register(&mxc_usbdr_host_device);
#endif
#ifdef CONFIG_USB_GADGET_ARC
	dr_utmi_config.operating_mode = DR_UDC_MODE;
	dr_utmi_config.wake_up_enable = _device_wakeup_enable;
	dr_utmi_config.phy_lowpower_suspend = _device_phy_lowpower_suspend;
	platform_device_add_data(&mxc_usbdr_udc_device, &dr_utmi_config, sizeof(dr_utmi_config));
	platform_device_register(&mxc_usbdr_udc_device);
#endif
}
