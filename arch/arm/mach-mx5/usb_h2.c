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
#include <mach/arc_otg.h>
#include "usb.h"
#include "iomux.h"
#include "mx51_pins.h"

extern void fsl_usb_recover_hcd(struct platform_device *pdev);
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

static void _wake_up_enable(struct fsl_usb2_platform_data *pdata, bool enable)
{
	printk(KERN_DEBUG "host2, %s, enable is %d\n", __func__, enable);
	if (enable)
		USBCTRL_HOST2 |= UCTRL_H2WIE;
	else {
		USBCTRL_HOST2 &= ~UCTRL_H2WIE;
		/* The interrupt must be disabled for at least 3
		* cycles of the standby clock(32k Hz) , that is 0.094 ms*/
		udelay(100);
	}
}

static void _phy_lowpower_suspend(struct fsl_usb2_platform_data *pdata, bool enable)
{
	printk(KERN_DEBUG "host2, %s, enable is %d\n", __func__, enable);
	if (enable) {
		UH2_PORTSC1 |= PORTSC_PHCD;
	} else {
		UH2_PORTSC1 &= ~PORTSC_PHCD;
	}
}

static void fsl_usbh2_clock_gate(bool on)
{
	struct clk *usb_clk;
	if (on) {
		usb_clk = clk_get(NULL, "usb_ahb_clk");
		clk_enable(usb_clk);
		clk_put(usb_clk);

		usb_clk = clk_get(NULL, "usboh3_clk");
		clk_enable(usb_clk);
		clk_put(usb_clk);
	} else {
		usb_clk = clk_get(NULL, "usb_ahb_clk");
		clk_disable(usb_clk);
		clk_put(usb_clk);

		usb_clk = clk_get(NULL, "usboh3_clk");
		clk_disable(usb_clk);
		clk_put(usb_clk);
	}
}

static enum usb_wakeup_event _is_usbh2_wakeup(struct fsl_usb2_platform_data *pdata)
{
	int wakeup_req = USBCTRL & UCTRL_H2WIR;

	if (wakeup_req)
		return !WAKEUP_EVENT_INVALID;

	return WAKEUP_EVENT_INVALID;
}

static void h2_wakeup_handler(struct fsl_usb2_platform_data *pdata)
{
	_wake_up_enable(pdata, false);
	_phy_lowpower_suspend(pdata, false);
	fsl_usb_recover_hcd(&mxc_usbh2_device);
}

static void usbh2_wakeup_event_clear(void)
{
	int wakeup_req = USBCTRL & UCTRL_H2WIR;

	if (wakeup_req != 0) {
		printk(KERN_INFO "Unknown wakeup.(OTGSC 0x%x)\n", UOG_OTGSC);
		/* Disable H2WIE to clear H2WIR, wait 3 clock
		 * cycles of standly clock(32KHz)
		 */
		USBCTRL &= ~UCTRL_H2WIE;
		udelay(100);
		USBCTRL |= UCTRL_H2WIE;
	}
}
static int fsl_usb_host_init_ext(struct platform_device *pdev)
{
	int ret = 0;
	struct clk *usb_clk;

	usb_clk = clk_get(NULL, "usboh3_clk");
	clk_enable(usb_clk);
	clk_put(usb_clk);

	/* on mx53, there is a hardware limitation that when switch the host2's clk mode
	 * ,usb phy1 clk must be on, after finish switching this clk can be off */
	if (cpu_is_mx53()) {
		usb_clk = clk_get(NULL, "usb_phy1_clk");
		clk_enable(usb_clk);
		clk_put(usb_clk);
	}

	ret = fsl_usb_host_init(pdev);

	if (cpu_is_mx53()) {
		usb_clk = clk_get(NULL, "usb_phy1_clk");
		clk_disable(usb_clk);
		clk_put(usb_clk);
	}

	/* setback USBH2_STP to be function */
	mxc_request_iomux(MX51_PIN_EIM_A26, IOMUX_CONFIG_ALT2);

	return ret;
}

static void fsl_usb_host_uninit_ext(struct fsl_usb2_platform_data *pdata)
{
	struct clk *usb_clk;

	usb_clk = clk_get(NULL, "usboh3_clk");
	clk_disable(usb_clk);
	clk_put(usb_clk);

	fsl_usb_host_uninit(pdata);
}

static struct fsl_usb2_platform_data usbh2_config = {
	.name = "Host 2",
	.platform_init = fsl_usb_host_init_ext,
	.platform_uninit = fsl_usb_host_uninit_ext,
	.operating_mode = FSL_USB2_MPH_HOST,
	.phy_mode = FSL_USB2_PHY_ULPI,
	.power_budget = 500,	/* 500 mA max power */
	.wake_up_enable = _wake_up_enable,
	.usb_clock_for_pm  = fsl_usbh2_clock_gate,
	.phy_lowpower_suspend = _phy_lowpower_suspend,
	.gpio_usb_active = gpio_usbh2_active,
	.gpio_usb_inactive = gpio_usbh2_inactive,
	.is_wakeup_event = _is_usbh2_wakeup,
	.wakeup_handler = h2_wakeup_handler,
	.transceiver = "isp1504",
};
static struct fsl_usb2_wakeup_platform_data usbh2_wakeup_config = {
	.name = "USBH2 wakeup",
	.usb_clock_for_pm  = fsl_usbh2_clock_gate,
	.usb_pdata = {&usbh2_config, NULL, NULL},
	.usb_wakeup_exhandle = usbh2_wakeup_event_clear,
};
void __init mx5_usbh2_init(void)
{
	usbh2_config.wakeup_pdata = &usbh2_wakeup_config;
	mxc_register_device(&mxc_usbh2_device, &usbh2_config);
	mxc_register_device(&mxc_usbh2_wakeup_device, &usbh2_wakeup_config);
}
