/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * High Speed Inter Chip code for i.MX6, this file is for HSIC port 1
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <mach/arc_otg.h>
#include <mach/hardware.h>
#include <mach/iomux-mx6q.h>
#include <mach/iomux-mx6dl.h>
#include <mach/iomux-mx6sl.h>
#include "devices-imx6q.h"
#include "regs-anadig.h"
#include "usb.h"

static struct clk *usb_oh3_clk;
static struct clk *usb_phy3_clk;

extern int clk_get_usecount(struct clk *clk);
static struct fsl_usb2_platform_data usbh2_config;

static void usbh2_internal_phy_clock_gate(bool on)
{
	if (on) {
		/* must turn on the 480M clock, otherwise
		 * there will be a 10ms delay before host
		 * controller send out resume signal*/
		USB_H2_CTRL |= UCTRL_UTMI_ON_CLOCK;
		USB_UH2_HSIC_CTRL |= HSIC_CLK_ON;
	} else {
		USB_UH2_HSIC_CTRL &= ~HSIC_CLK_ON;
		/* can't turn off this clock, otherwise
		 * there will be a 10ms delay before host
		 * controller send out resume signal*/
		/*USB_H2_CTRL &= ~UCTRL_UTMI_ON_CLOCK*/;
	}
}

static int fsl_usb_host_init_ext(struct platform_device *pdev)
{
	int ret;
	struct clk *usb_clk;
	usb_clk = clk_get(NULL, "usboh3_clk");
	clk_enable(usb_clk);
	usb_oh3_clk = usb_clk;

	usb_clk = clk_get(NULL, "usb_phy3_clk");
	clk_enable(usb_clk);
	usb_phy3_clk = usb_clk;

	ret = fsl_usb_host_init(pdev);
	if (ret) {
		printk(KERN_ERR "host1 init fails......\n");
		return ret;
	}
	usbh2_internal_phy_clock_gate(true);
	 /* Host2 HSIC enable */
	USB_UH2_HSIC_CTRL |= HSIC_EN;

	return 0;
}

static void fsl_usb_host_uninit_ext(struct platform_device *pdev)
{
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;

	fsl_usb_host_uninit(pdata);

	clk_put(usb_phy3_clk);
	clk_put(usb_oh3_clk);

}

static void usbh2_clock_gate(bool on)
{
	pr_debug("%s: on is %d\n", __func__, on);
	if (on) {
		clk_enable(usb_oh3_clk);
		clk_enable(usb_phy3_clk);
		usbh2_internal_phy_clock_gate(true);
	} else {
		usbh2_internal_phy_clock_gate(false);
		clk_disable(usb_phy3_clk);
		clk_disable(usb_oh3_clk);
	}
}

void mx6_set_host2_vbus_func(driver_vbus_func driver_vbus)
{
	usbh2_config.platform_driver_vbus = driver_vbus;
}

static void _wake_up_enable(struct fsl_usb2_platform_data *pdata, bool enable)
{
	pr_debug("host2, %s, enable is %d\n", __func__, enable);
	/* for HSIC, no disconnect nor connect
	 * we need to disable the WKDS, WKCN */
	UH2_PORTSC1 &= ~(PORTSC_WKDC | PORTSC_WKCN);

	if (enable) {
		USB_H2_CTRL |= (UCTRL_OWIE);
	} else {
		USB_H2_CTRL &= ~(UCTRL_OWIE);
		/* The interrupt must be disabled for at least 3
		* cycles of the standby clock(32k Hz) , that is 0.094 ms*/
		udelay(100);
	}
}

static void _phy_lowpower_suspend(struct fsl_usb2_platform_data *pdata, bool enable)
{
	pr_debug("host2, %s, enable is %d\n", __func__, enable);
	if (enable)
		UH2_PORTSC1 |= PORTSC_PHCD;
	else
		UH2_PORTSC1 &= ~PORTSC_PHCD;

}

static enum usb_wakeup_event _is_usbh2_wakeup(struct fsl_usb2_platform_data *pdata)
{
	u32 wakeup_req = USB_H2_CTRL & UCTRL_OWIR;

	if (wakeup_req)
		return WAKEUP_EVENT_DPDM;
	pr_err("host2, %s, invalid wake up\n", __func__);
	return WAKEUP_EVENT_INVALID;
}

static void h2_wakeup_handler(struct fsl_usb2_platform_data *pdata)
{
	_wake_up_enable(pdata, false);
	_phy_lowpower_suspend(pdata, false);
}

static void usbh2_wakeup_event_clear(void)
{
	u32 wakeup_req = USB_H2_CTRL & UCTRL_OWIR;
	pr_debug("%s\n", __func__);

	if (wakeup_req != 0) {
		/* Disable H2 wakeup enable to clear H2 wakeup request, wait 3 clock
		 * cycles of standly clock(32KHz)
		 */
		USB_H2_CTRL &= ~UCTRL_OWIE;
		udelay(100);
		USB_H2_CTRL |= UCTRL_OWIE;
	}
}

static void hsic_start(void)
{
	pr_debug("%s\n", __func__);
	/* strobe 47K pull up */
	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_pad(
				MX6Q_PAD_RGMII_TX_CTL__USBOH3_H2_STROBE_START);
	else if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_pad(
				MX6DL_PAD_RGMII_TX_CTL__USBOH3_H2_STROBE_START);
	else if (cpu_is_mx6sl())
		mxc_iomux_v3_setup_pad(
				MX6SL_PAD_HSIC_STROBE__USB_H_STROBE_START);
}

static void hsic_device_connected(void)
{
	pr_debug("%s\n", __func__);
	if (!(USB_UH2_HSIC_CTRL & HSIC_DEV_CONN))
		USB_UH2_HSIC_CTRL |= HSIC_DEV_CONN;
}

static struct fsl_usb2_platform_data usbh2_config = {
	.name		= "Host 2",
	.init		= fsl_usb_host_init_ext,
	.exit		= fsl_usb_host_uninit_ext,
	.operating_mode = FSL_USB2_MPH_HOST,
	.phy_mode = FSL_USB2_PHY_HSIC,
	.power_budget = 500,	/* 500 mA max power */
	.wake_up_enable = _wake_up_enable,
	.usb_clock_for_pm  = usbh2_clock_gate,
	.phy_lowpower_suspend = _phy_lowpower_suspend,
	.is_wakeup_event = _is_usbh2_wakeup,
	.wakeup_handler = h2_wakeup_handler,
	.transceiver = "hsic_xcvr",
	.hsic_post_ops = hsic_start,
	.hsic_device_connected = hsic_device_connected,
};

static struct fsl_usb2_wakeup_platform_data usbh2_wakeup_config = {
		.name = "usbh2 wakeup",
		.usb_clock_for_pm  = usbh2_clock_gate,
		.usb_pdata = {&usbh2_config, NULL, NULL},
		.usb_wakeup_exhandle = usbh2_wakeup_event_clear,
};

void __init mx6_usb_h2_init(void)
{
	struct platform_device *pdev, *pdev_wakeup;
	static void __iomem *anatop_base_addr = MX6_IO_ADDRESS(ANATOP_BASE_ADDR);
	usbh2_config.wakeup_pdata = &usbh2_wakeup_config;
	if (cpu_is_mx6sl())
		pdev = imx6sl_add_fsl_ehci_hs(2, &usbh2_config);
	else
		pdev = imx6q_add_fsl_ehci_hs(2, &usbh2_config);

	usbh2_wakeup_config.usb_pdata[0] = pdev->dev.platform_data;
	if (cpu_is_mx6sl())
		pdev_wakeup = imx6sl_add_fsl_usb2_hs_wakeup(2, &usbh2_wakeup_config);
	else
		pdev_wakeup = imx6q_add_fsl_usb2_hs_wakeup(2, &usbh2_wakeup_config);
	platform_device_add(pdev);
	((struct fsl_usb2_platform_data *)(pdev->dev.platform_data))->wakeup_pdata =
		pdev_wakeup->dev.platform_data;
	/* Some phy and power's special controls for host2
	 * 1. Its 480M is from OTG's 480M
	 * 2. EN_USB_CLKS should always be opened
	 */
	__raw_writel(BM_ANADIG_USB1_PLL_480_CTRL_EN_USB_CLKS,
			anatop_base_addr + HW_ANADIG_USB1_PLL_480_CTRL_SET);
	/* must change the clkgate delay to 2 or 3 to avoid
	 * 24M OSCI clock not stable issue */
	__raw_writel(BF_ANADIG_ANA_MISC0_CLKGATE_DELAY(3),
			anatop_base_addr + HW_ANADIG_ANA_MISC0);
}
