/*
 * Copyright (C) 2005-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <asm/delay.h>
#include <mach/arc_otg.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include "usb.h"
#include "iomux.h"
#include "mx51_pins.h"
static struct clk *usb_phy2_clk;
static struct clk *usb_oh3_clk;
static struct clk *usb_ahb_clk;
extern int clk_get_usecount(struct clk *clk);

#ifdef CONFIG_USB_EHCI_ARC
extern void fsl_usb_recover_hcd(struct platform_device *pdev);
#else
static void fsl_usb_recover_hcd(struct platform_device *pdev)
{; }
#endif
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
	pr_debug("host1, %s, enable is %d\n", __func__, enable);
	if (enable)
		USBCTRL |= UCTRL_H1WIE;
	else {
		USBCTRL &= ~UCTRL_H1WIE;
		/* The interrupt must be disabled for at least 3
		* cycles of the standby clock(32k Hz) , that is 0.094 ms*/
		udelay(100);
	}
}

static void _phy_lowpower_suspend(struct fsl_usb2_platform_data *pdata, bool enable)
{
	pr_debug("host1, %s, enable is %d\n", __func__, enable);
	if (enable) {
		UH1_PORTSC1 |= PORTSC_PHCD;
	} else {
		UH1_PORTSC1 &= ~PORTSC_PHCD;
	}
}

static void usbh1_clock_gate(bool on)
{
	pr_debug("%s: on is %d\n", __func__, on);
	if (on) {
		clk_enable(usb_ahb_clk);
		clk_enable(usb_oh3_clk);
		clk_enable(usb_phy2_clk);
	} else {
		clk_disable(usb_phy2_clk);
		clk_disable(usb_oh3_clk);
		clk_disable(usb_ahb_clk);
	}
}

static enum usb_wakeup_event _is_usbh1_wakeup(struct fsl_usb2_platform_data *pdata)
{
	int wakeup_req = USBCTRL & UCTRL_H1WIR;

	if (wakeup_req)
		return !WAKEUP_EVENT_INVALID;

	return WAKEUP_EVENT_INVALID;
}

static void h1_wakeup_handler(struct fsl_usb2_platform_data *pdata)
{
	_wake_up_enable(pdata, false);
	_phy_lowpower_suspend(pdata, false);
	fsl_usb_recover_hcd(&mxc_usbh1_device);
}

static void usbh1_wakeup_event_clear(void)
{
	int wakeup_req = USBCTRL & UCTRL_H1WIR;

	if (wakeup_req != 0) {
		printk(KERN_INFO "Unknown wakeup.(OTGSC 0x%x)\n", UOG_OTGSC);
		/* Disable H1WIE to clear H1WIR, wait 3 clock
		 * cycles of standly clock(32KHz)
		 */
		USBCTRL &= ~UCTRL_H1WIE;
		udelay(100);
		USBCTRL |= UCTRL_H1WIE;
	}
}
static int fsl_usb_host_init_ext(struct platform_device *pdev)
{
	int ret;
	struct clk *usb_clk;

	/* the usb_ahb_clk will be enabled in usb_otg_init */
	usb_ahb_clk = clk_get(NULL, "usb_ahb_clk");

	if (cpu_is_mx53()) {
		usb_clk = clk_get(NULL, "usboh3_clk");
		clk_enable(usb_clk);
		usb_oh3_clk = usb_clk;

		usb_clk = clk_get(NULL, "usb_phy2_clk");
		clk_enable(usb_clk);
		usb_phy2_clk = usb_clk;
	} else if (cpu_is_mx50()) {
		usb_clk = clk_get(NULL, "usb_phy2_clk");
		clk_enable(usb_clk);
		usb_phy2_clk = usb_clk;
	} else if (cpu_is_mx51()) {
		usb_clk = clk_get(NULL, "usboh3_clk");
		clk_enable(usb_clk);
		usb_oh3_clk = usb_clk;
	}

	ret = fsl_usb_host_init(pdev);
	if (ret)
		return ret;

	if (cpu_is_mx51()) {
		/* setback USBH1_STP to be function */
		mxc_request_iomux(MX51_PIN_USBH1_STP, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_USBH1_STP, PAD_CTL_SRE_FAST |
				  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
				  PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE |
				  PAD_CTL_HYS_ENABLE | PAD_CTL_DDR_INPUT_CMOS |
				  PAD_CTL_DRV_VOT_LOW);
		gpio_free(IOMUX_TO_GPIO(MX51_PIN_USBH1_STP));
	}

	/* disable remote wakeup irq */
	USBCTRL &= ~UCTRL_H1WIE;
	return 0;
}

static void fsl_usb_host_uninit_ext(struct fsl_usb2_platform_data *pdata)
{
	if (cpu_is_mx53()) {
		clk_disable(usb_oh3_clk);
		clk_put(usb_oh3_clk);

		clk_disable(usb_phy2_clk);
		clk_put(usb_phy2_clk);
	} else if (cpu_is_mx50()) {
		clk_disable(usb_phy2_clk);
		clk_put(usb_phy2_clk);
	} else if (cpu_is_mx51()) {
		clk_disable(usb_oh3_clk);
		clk_put(usb_oh3_clk);
	}

	fsl_usb_host_uninit(pdata);
	/* usb_ahb_clk will be disabled at usb_common.c */
	clk_put(usb_ahb_clk);
}

static struct fsl_usb2_platform_data usbh1_config = {
	.name = "Host 1",
	.platform_init = fsl_usb_host_init_ext,
	.platform_uninit = fsl_usb_host_uninit_ext,
	.operating_mode = FSL_USB2_MPH_HOST,
	.phy_mode = FSL_USB2_PHY_UTMI_WIDE,
	.power_budget = 500,	/* 500 mA max power */
	.wake_up_enable = _wake_up_enable,
	.usb_clock_for_pm  = usbh1_clock_gate,
	.phy_lowpower_suspend = _phy_lowpower_suspend,
	.is_wakeup_event = _is_usbh1_wakeup,
	.wakeup_handler = h1_wakeup_handler,
	.transceiver = "utmi",
};
static struct fsl_usb2_wakeup_platform_data usbh1_wakeup_config = {
		.name = "USBH1 wakeup",
		.usb_clock_for_pm  = usbh1_clock_gate,
		.usb_pdata = {&usbh1_config, NULL, NULL},
		.usb_wakeup_exhandle = usbh1_wakeup_event_clear,
};

void mx5_set_host1_vbus_func(driver_vbus_func driver_vbus)
{
	usbh1_config.platform_driver_vbus = driver_vbus;
}

void __init mx5_usbh1_init(void)
{
	if (cpu_is_mx51()) {
		usbh1_config.phy_mode = FSL_USB2_PHY_ULPI;
		usbh1_config.transceiver = "isp1504";
		usbh1_config.gpio_usb_active = gpio_usbh1_active;
		usbh1_config.gpio_usb_inactive = gpio_usbh1_inactive;
	}
	mxc_register_device(&mxc_usbh1_device, &usbh1_config);
	usbh1_config.wakeup_pdata = &usbh1_wakeup_config;
	mxc_register_device(&mxc_usbh1_wakeup_device, &usbh1_wakeup_config);
}

