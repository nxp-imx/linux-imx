/*
 * Copyright (C) 2005-2012 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>

#include <mach/arc_otg.h>
#include <mach/iomux-mx51.h>
#include <mach/iomux-mx53.h>
#include "usb.h"

#define MX5X_USBH2_STP	IMX_GPIO_NR(2, 20)

/*
 * USB Host2 HS port
 */
static int gpio_usbh2_active(void)
{
	iomux_v3_cfg_t usbh2stp_gpio = MX51_PAD_EIM_A26__GPIO2_20;
	int ret = 0;

	/* Set USBH2_STP to GPIO and toggle it */
	mxc_iomux_v3_setup_pad(usbh2stp_gpio);
	ret = gpio_request(MX5X_USBH2_STP, "eim_a26");
	if (ret) {
		pr_debug("failed to get MX51_PAD_EIM_A26__GPIO2_20: %d\n", ret);
		return ret;
	}

	gpio_direction_output(MX5X_USBH2_STP, 0);
	gpio_set_value(MX5X_USBH2_STP, 1);
	msleep(100);

	return 0;
}

static void gpio_usbh2_inactive(void)
{
	gpio_free(MX5X_USBH2_STP);
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
		return WAKEUP_EVENT_DPDM;

	return WAKEUP_EVENT_INVALID;
}

static void h2_wakeup_handler(struct fsl_usb2_platform_data *pdata)
{
	_wake_up_enable(pdata, false);
	_phy_lowpower_suspend(pdata, false);
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
	mxc_iomux_v3_setup_pad(MX51_PAD_EIM_A26__USBH2_STP);

	return ret;
}

static void fsl_usb_host_uninit_ext(struct platform_device *pdev)
{
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;
	struct clk *usb_clk;

	usb_clk = clk_get(NULL, "usboh3_clk");
	clk_disable(usb_clk);
	clk_put(usb_clk);

	fsl_usb_host_uninit(pdata);
}

static struct fsl_usb2_platform_data usbh2_config = {
	.name		= "Host 2",
	.init		= fsl_usb_host_init_ext,
	.exit		= fsl_usb_host_uninit_ext,
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

#define MX53_OFFSET	0x20000000
void __init mx5_usbh2_init(void)
{
	if (cpu_is_mx53() || cpu_is_mx50()) {
		mxc_usbh2_device.resource[0].start	-= MX53_OFFSET;
		mxc_usbh2_device.resource[0].end	-= MX53_OFFSET;
	}
	usbh2_config.wakeup_pdata = &usbh2_wakeup_config;
	mxc_register_device(&mxc_usbh2_device, &usbh2_config);
	mxc_register_device(&mxc_usbh2_wakeup_device, &usbh2_wakeup_config);
}
