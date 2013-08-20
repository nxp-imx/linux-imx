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
#include "devices-imx6q.h"
#include "regs-anadig.h"
#include "usb.h"

static struct clk *usb_oh3_clk;
extern int clk_get_usecount(struct clk *clk);
static struct fsl_usb2_platform_data usbh1_config;

static void fsl_platform_h1_set_usb_phy_dis(
		struct fsl_usb2_platform_data *pdata, bool enable)
{
	u32 usb_phy_ctrl_dcdt = 0;
	void __iomem *anatop_base_addr = MX6_IO_ADDRESS(ANATOP_BASE_ADDR);
	usb_phy_ctrl_dcdt = __raw_readl(
			MX6_IO_ADDRESS(pdata->phy_regs) + HW_USBPHY_CTRL) &
			BM_USBPHY_CTRL_ENHOSTDISCONDETECT;
	if (enable) {
		if (usb_phy_ctrl_dcdt == 0) {
			__raw_writel(BM_ANADIG_USB2_PLL_480_CTRL_EN_USB_CLKS,
					anatop_base_addr + HW_ANADIG_USB2_PLL_480_CTRL_CLR);

			__raw_writel(BM_USBPHY_PWD_RXPWDENV,
					MX6_IO_ADDRESS(pdata->phy_regs) + HW_USBPHY_PWD_SET);

			udelay(300);

			__raw_writel(BM_USBPHY_CTRL_ENHOSTDISCONDETECT,
				MX6_IO_ADDRESS(pdata->phy_regs)
				+ HW_USBPHY_CTRL_SET);

			UH1_USBSTS |= (1 << 7);

			while ((UH1_USBSTS & (1 << 7)) == 0)
				;

			udelay(2);

			__raw_writel(BM_USBPHY_PWD_RXPWDENV,
					MX6_IO_ADDRESS(pdata->phy_regs) + HW_USBPHY_PWD_CLR);

			__raw_writel(BM_ANADIG_USB2_PLL_480_CTRL_EN_USB_CLKS,
					anatop_base_addr + HW_ANADIG_USB2_PLL_480_CTRL_SET);

		}
	} else {
		if (usb_phy_ctrl_dcdt
				== BM_USBPHY_CTRL_ENHOSTDISCONDETECT)
			__raw_writel(BM_USBPHY_CTRL_ENHOSTDISCONDETECT,
				MX6_IO_ADDRESS(pdata->phy_regs)
				+ HW_USBPHY_CTRL_CLR);
	}
}

static void usbh1_internal_phy_clock_gate(bool on)
{
	void __iomem *phy_reg = MX6_IO_ADDRESS(USB_PHY1_BASE_ADDR);
	if (on) {
		__raw_writel(BM_USBPHY_CTRL_CLKGATE, phy_reg + HW_USBPHY_CTRL_CLR);
	} else {
		__raw_writel(BM_USBPHY_CTRL_CLKGATE, phy_reg + HW_USBPHY_CTRL_SET);
	}
}

static void usbh1_platform_phy_power_on(void)
{
	void __iomem *anatop_base_addr = MX6_IO_ADDRESS(ANATOP_BASE_ADDR);
	__raw_writel(BM_ANADIG_ANA_MISC0_STOP_MODE_CONFIG,
				anatop_base_addr + HW_ANADIG_ANA_MISC0_SET);
}

static int usb_phy_enable(struct fsl_usb2_platform_data *pdata)
{
	u32 tmp;
	void __iomem *phy_reg = MX6_IO_ADDRESS(USB_PHY1_BASE_ADDR);
	void __iomem *phy_ctrl;

	/* Stop then Reset */
	UH1_USBCMD &= ~UCMD_RUN_STOP;
	while (UH1_USBCMD & UCMD_RUN_STOP)
		;

	UH1_USBCMD |= UCMD_RESET;
	while ((UH1_USBCMD) & (UCMD_RESET))
		;

	/*
	 * If the controller reset does not put the PHY be out of
	 * low power mode, do it manually.
	 */
	if (UH1_PORTSC1 & PORTSC_PHCD) {
		UH1_PORTSC1 &= ~PORTSC_PHCD;
		mdelay(1);
	}

	/* Reset USBPHY module */
	phy_ctrl = phy_reg + HW_USBPHY_CTRL;
	tmp = __raw_readl(phy_ctrl);
	tmp |= BM_USBPHY_CTRL_SFTRST;
	__raw_writel(tmp, phy_ctrl);
	udelay(10);

	/* Remove CLKGATE and SFTRST */
	tmp = __raw_readl(phy_ctrl);
	tmp &= ~(BM_USBPHY_CTRL_CLKGATE | BM_USBPHY_CTRL_SFTRST);
	__raw_writel(tmp, phy_ctrl);
	udelay(10);

	/* Power up the PHY */
	__raw_writel(0, phy_reg + HW_USBPHY_PWD);
	/* enable FS/LS device */
	tmp = __raw_readl(phy_reg + HW_USBPHY_CTRL);
	tmp |= (BM_USBPHY_CTRL_ENUTMILEVEL2 | BM_USBPHY_CTRL_ENUTMILEVEL3);
	__raw_writel(tmp, phy_reg + HW_USBPHY_CTRL);

	if (!usb_icbug_swfix_need())
		__raw_writel((1 << 17), phy_reg + HW_USBPHY_IP_SET);
	if (cpu_is_mx6sl())
		__raw_writel((1 << 18), phy_reg + HW_USBPHY_IP_SET);
	return 0;
}
static int fsl_usb_host_init_ext(struct platform_device *pdev)
{
	int ret;
	struct clk *usb_clk;
	usb_clk = clk_get(NULL, "usboh3_clk");
	clk_enable(usb_clk);
	usb_oh3_clk = usb_clk;

	ret = fsl_usb_host_init(pdev);
	if (ret) {
		printk(KERN_ERR "host1 init fails......\n");
		clk_disable(usb_oh3_clk);
		clk_put(usb_oh3_clk);
		return ret;
	}
	usbh1_internal_phy_clock_gate(true);
	usb_phy_enable(pdev->dev.platform_data);

	return 0;
}

static void fsl_usb_host_uninit_ext(struct platform_device *pdev)
{
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;

	fsl_usb_host_uninit(pdata);

	clk_put(usb_oh3_clk);

}

static void usbh1_clock_gate(bool on)
{
	pr_debug("%s: on is %d\n", __func__, on);
	if (on) {
		clk_enable(usb_oh3_clk);
	} else {
		clk_disable(usb_oh3_clk);
	}
}

static void _wake_up_enable(struct fsl_usb2_platform_data *pdata, bool enable)
{
	void __iomem *phy_reg = MX6_IO_ADDRESS(USB_PHY1_BASE_ADDR);

	pr_debug("host1, %s, enable is %d\n", __func__, enable);
	if (enable) {
		__raw_writel(BM_USBPHY_CTRL_ENIDCHG_WKUP | BM_USBPHY_CTRL_ENVBUSCHG_WKUP
				| BM_USBPHY_CTRL_ENDPDMCHG_WKUP
				| BM_USBPHY_CTRL_ENAUTOSET_USBCLKS
				| BM_USBPHY_CTRL_ENAUTOCLR_PHY_PWD
				| BM_USBPHY_CTRL_ENAUTOCLR_CLKGATE
				| BM_USBPHY_CTRL_ENAUTOCLR_USBCLKGATE
				| BM_USBPHY_CTRL_ENAUTO_PWRON_PLL , phy_reg + HW_USBPHY_CTRL_SET);
		USB_H1_CTRL |= (UCTRL_OWIE);
	} else {
		__raw_writel(BM_USBPHY_CTRL_ENIDCHG_WKUP | BM_USBPHY_CTRL_ENVBUSCHG_WKUP
				| BM_USBPHY_CTRL_ENDPDMCHG_WKUP
				| BM_USBPHY_CTRL_ENAUTOSET_USBCLKS
				| BM_USBPHY_CTRL_ENAUTOCLR_PHY_PWD
				| BM_USBPHY_CTRL_ENAUTOCLR_CLKGATE
				| BM_USBPHY_CTRL_ENAUTOCLR_USBCLKGATE
				| BM_USBPHY_CTRL_ENAUTO_PWRON_PLL , phy_reg + HW_USBPHY_CTRL_CLR);
		USB_H1_CTRL &= ~(UCTRL_OWIE);
		/* The interrupt must be disabled for at least 3
		* cycles of the standby clock(32k Hz) , that is 0.094 ms*/
		udelay(100);
	}
}

static void usbh1_platform_rh_suspend_swfix(struct fsl_usb2_platform_data *pdata)
{
	void __iomem *phy_reg = MX6_IO_ADDRESS(USB_PHY1_BASE_ADDR);
	u32 tmp;
	u32 index = 0;

	/* before we set and then clear PWD bit,
	 * we must wait LS to be J */
	if ((UH1_PORTSC1 & (3 << 26)) != (1 << 26)) {
		while (((UH1_PORTSC1 & PORTSC_LS_MASK) != PORTSC_LS_J_STATE) &&
				(index < 1000)) {
			index++;
			udelay(4);
		}
	} else {
		while (((UH1_PORTSC1 & PORTSC_LS_MASK) != PORTSC_LS_K_STATE) &&
				(index < 1000)) {
			index++;
			udelay(4);
		}
	}

	if (index >= 1000)
		printk(KERN_INFO "%s big error\n", __func__);

	tmp = (BM_USBPHY_PWD_TXPWDFS
		| BM_USBPHY_PWD_TXPWDIBIAS
		| BM_USBPHY_PWD_TXPWDV2I
		| BM_USBPHY_PWD_RXPWDENV
		| BM_USBPHY_PWD_RXPWD1PT1
		| BM_USBPHY_PWD_RXPWDDIFF
		| BM_USBPHY_PWD_RXPWDRX);
	__raw_writel(tmp, phy_reg + HW_USBPHY_PWD_SET);

	__raw_writel(tmp, phy_reg + HW_USBPHY_PWD_CLR);

	fsl_platform_h1_set_usb_phy_dis(pdata, 0);
}

static void usbh1_platform_rh_resume_swfix(struct fsl_usb2_platform_data *pdata)
{
	u32 index = 0;

	if ((UH1_PORTSC1 & (PORTSC_PORT_SPEED_MASK)) != PORTSC_PORT_SPEED_HIGH)
		return ;
	while ((UH1_PORTSC1 & PORTSC_PORT_FORCE_RESUME)
			&& (index < 1000)) {
		udelay(500);
		index++;
	}
	if (index >= 1000)
		printk(KERN_ERR "failed to wait for the resume finished in %s() line:%d\n",
		__func__, __LINE__);
	/* We should add some delay to wait for the device switch to
	  * High-Speed 45ohm termination resistors mode. */
	udelay(500);
	fsl_platform_h1_set_usb_phy_dis(pdata, 1);
}

static void usbh1_platform_rh_suspend(struct fsl_usb2_platform_data *pdata)
{
	/*for mx6sl ,we do not need any sw fix*/
	if (cpu_is_mx6sl())
		return ;
	__raw_writel(BM_USBPHY_CTRL_ENHOSTDISCONDETECT,
		MX6_IO_ADDRESS(pdata->phy_regs)
		+ HW_USBPHY_CTRL_CLR);
}

static void usbh1_platform_rh_resume(struct fsl_usb2_platform_data *pdata)
{
	u32 index = 0;

	/*for mx6sl ,we do not need any sw fix*/
	if (cpu_is_mx6sl())
		return ;
	if ((UH1_PORTSC1 & (PORTSC_PORT_SPEED_MASK)) != PORTSC_PORT_SPEED_HIGH)
		return ;
	while ((UH1_PORTSC1 & PORTSC_PORT_FORCE_RESUME)
			&& (index < 1000)) {
		udelay(500);
		index++;
	}
	if (index >= 1000)
		printk(KERN_ERR "failed to wait for the resume finished in %s() line:%d\n",
		__func__, __LINE__);
	/* We should add some delay to wait for the device switch to
	  * High-Speed 45ohm termination resistors mode. */
	udelay(500);
	__raw_writel(BM_USBPHY_CTRL_ENHOSTDISCONDETECT,
		MX6_IO_ADDRESS(pdata->phy_regs)
		+ HW_USBPHY_CTRL_SET);
}

static void _phy_lowpower_suspend(struct fsl_usb2_platform_data *pdata, bool enable)
{
	u32 tmp;
	void __iomem *phy_reg = MX6_IO_ADDRESS(USB_PHY1_BASE_ADDR);
	pr_debug("host1, %s, enable is %d\n", __func__, enable);
	if (enable) {
		UH1_PORTSC1 |= PORTSC_PHCD;

		pr_debug("%s, Poweroff UTMI \n", __func__);

		tmp = (BM_USBPHY_PWD_TXPWDFS
			| BM_USBPHY_PWD_TXPWDIBIAS
			| BM_USBPHY_PWD_TXPWDV2I
			| BM_USBPHY_PWD_RXPWDENV
			| BM_USBPHY_PWD_RXPWD1PT1
			| BM_USBPHY_PWD_RXPWDDIFF
			| BM_USBPHY_PWD_RXPWDRX);
		__raw_writel(tmp, phy_reg + HW_USBPHY_PWD_SET);

		usbh1_internal_phy_clock_gate(false);
	} else {
		if (UH1_PORTSC1 & PORTSC_PHCD)
			UH1_PORTSC1 &= ~PORTSC_PHCD;

		/* Wait PHY clock stable */
		mdelay(1);

		usbh1_internal_phy_clock_gate(true);
		udelay(2);
		tmp = (BM_USBPHY_PWD_TXPWDFS
			| BM_USBPHY_PWD_TXPWDIBIAS
			| BM_USBPHY_PWD_TXPWDV2I
			| BM_USBPHY_PWD_RXPWDENV
			| BM_USBPHY_PWD_RXPWD1PT1
			| BM_USBPHY_PWD_RXPWDDIFF
			| BM_USBPHY_PWD_RXPWDRX);
		__raw_writel(tmp, phy_reg + HW_USBPHY_PWD_CLR);
		/*
		 * The PHY works at 32Khz clock when it is at low power mode,
		 * it needs 10 clocks from 32Khz to normal work state, so
		 * 500us is the safe value for PHY enters stable status
		 * according to IC engineer.
		 */
		udelay(500);

	}
}

static enum usb_wakeup_event _is_usbh1_wakeup(struct fsl_usb2_platform_data *pdata)
{
	u32 wakeup_req = USB_H1_CTRL & UCTRL_OWIR;

	if (wakeup_req)
		return WAKEUP_EVENT_DPDM;
	pr_err("host1, %s, invalid wake up\n", __func__);
	return WAKEUP_EVENT_INVALID;
}

static void h1_wakeup_handler(struct fsl_usb2_platform_data *pdata)
{
	_wake_up_enable(pdata, false);
	_phy_lowpower_suspend(pdata, false);
}

static void usbh1_wakeup_event_clear(void)
{
	int wakeup_req = USB_H1_CTRL & UCTRL_OWIR;

	if (wakeup_req != 0) {
		printk(KERN_INFO "Unknown wakeup.(H1 OTGSC 0x%x)\n", UH1_OTGSC);
		/* Disable OWIE to clear OWIR, wait 3 clock
		 * cycles of standly clock(32KHz)
		 */
		USB_H1_CTRL &= ~UCTRL_OWIE;
		udelay(100);
		USB_H1_CTRL |= UCTRL_OWIE;
	}
}

static struct fsl_usb2_platform_data usbh1_config = {
	.name		= "Host 1",
	.init		= fsl_usb_host_init_ext,
	.exit		= fsl_usb_host_uninit_ext,
	.operating_mode = FSL_USB2_MPH_HOST,
	.phy_mode = FSL_USB2_PHY_UTMI_WIDE,
	.power_budget = 500,	/* 500 mA max power */
	.wake_up_enable = _wake_up_enable,
	.usb_clock_for_pm  = usbh1_clock_gate,
	.platform_set_disconnect_det = fsl_platform_h1_set_usb_phy_dis,
	.phy_lowpower_suspend = _phy_lowpower_suspend,
	.is_wakeup_event = _is_usbh1_wakeup,
	.wakeup_handler = h1_wakeup_handler,
	.platform_phy_power_on = usbh1_platform_phy_power_on,
	.transceiver = "utmi",
	.phy_regs = USB_PHY1_BASE_ADDR,
};
static struct fsl_usb2_wakeup_platform_data usbh1_wakeup_config = {
		.name = "USBH1 wakeup",
		.usb_clock_for_pm  = usbh1_clock_gate,
		.usb_pdata = {&usbh1_config, NULL, NULL},
		.usb_wakeup_exhandle = usbh1_wakeup_event_clear,
};

static struct platform_device *pdev, *pdev_wakeup;
static driver_vbus_func  mx6_set_usb_host1_vbus;

static int  __init mx6_usb_h1_init(void)
{
	static void __iomem *anatop_base_addr = MX6_IO_ADDRESS(ANATOP_BASE_ADDR);
	struct imx_fsl_usb2_wakeup_data imx6q_fsl_hs_wakeup_data[] = {
		imx_fsl_usb2_wakeup_data_entry_single(MX6Q, 1, HS1)};
	struct imx_fsl_usb2_wakeup_data  imx6sl_fsl_hs_wakeup_data[] = {
		imx_fsl_usb2_wakeup_data_entry_single(MX6SL, 1, HS1)};
	struct imx_mxc_ehci_data imx6q_mxc_ehci_hs_data[] = {
		imx_mxc_ehci_data_entry_single(MX6Q, 1, HS1)};
	struct imx_mxc_ehci_data imx6sl_mxc_ehci_hs_data[] = {
		imx_mxc_ehci_data_entry_single(MX6SL, 1, HS1)};

	mx6_get_host1_vbus_func(&mx6_set_usb_host1_vbus);
	usbh1_config.platform_driver_vbus = mx6_set_usb_host1_vbus;

	/* Some phy and power's special controls for host1
	 * 1. The external charger detector needs to be disabled
	 * or the signal at DP will be poor
	 * 2. The PLL's power and output to usb for host 1
	 * is totally controlled by IC, so the Software only needs
	 * to enable them at initializtion.
	 */
	__raw_writel(BM_ANADIG_USB2_CHRG_DETECT_EN_B  \
			| BM_ANADIG_USB2_CHRG_DETECT_CHK_CHRG_B, \
			anatop_base_addr + HW_ANADIG_USB2_CHRG_DETECT);
	__raw_writel(BM_ANADIG_USB2_PLL_480_CTRL_BYPASS,
			anatop_base_addr + HW_ANADIG_USB2_PLL_480_CTRL_CLR);
	__raw_writel(BM_ANADIG_USB2_PLL_480_CTRL_ENABLE  \
			| BM_ANADIG_USB2_PLL_480_CTRL_POWER \
			| BM_ANADIG_USB2_PLL_480_CTRL_EN_USB_CLKS, \
			anatop_base_addr + HW_ANADIG_USB2_PLL_480_CTRL_SET);

	usbh1_config.wakeup_pdata = &usbh1_wakeup_config;
	if (usb_icbug_swfix_need()) {
		usbh1_config.platform_rh_suspend = usbh1_platform_rh_suspend_swfix;
		usbh1_config.platform_rh_resume  = usbh1_platform_rh_resume_swfix;
	} else {
		usbh1_config.platform_rh_suspend = usbh1_platform_rh_suspend;
		usbh1_config.platform_rh_resume  = usbh1_platform_rh_resume;
	}
	if (cpu_is_mx6sl())
		pdev = imx6sl_add_fsl_ehci_hs(1, &usbh1_config);
	else
		pdev = imx6q_add_fsl_ehci_hs(1, &usbh1_config);
	usbh1_wakeup_config.usb_pdata[0] = pdev->dev.platform_data;
	if (cpu_is_mx6sl())
		pdev_wakeup = imx6sl_add_fsl_usb2_hs_wakeup(1, &usbh1_wakeup_config);
	else
		pdev_wakeup = imx6q_add_fsl_usb2_hs_wakeup(1, &usbh1_wakeup_config);
	platform_device_add(pdev);
	((struct fsl_usb2_platform_data *)(pdev->dev.platform_data))->wakeup_pdata =
		(struct fsl_usb2_wakeup_platform_data *)(pdev_wakeup->dev.platform_data);
	return 0;
}
module_init(mx6_usb_h1_init);

static void __exit mx6_usb_h1_exit(void)
{
	static void __iomem *anatop_base_addr = MX6_IO_ADDRESS(ANATOP_BASE_ADDR);

	platform_device_unregister(pdev);
	platform_device_unregister(pdev_wakeup);
	__raw_writel(BM_ANADIG_USB2_PLL_480_CTRL_BYPASS,
			anatop_base_addr + HW_ANADIG_USB2_PLL_480_CTRL_SET);
	__raw_writel(BM_ANADIG_USB2_PLL_480_CTRL_ENABLE  \
			| BM_ANADIG_USB2_PLL_480_CTRL_POWER \
			| BM_ANADIG_USB2_PLL_480_CTRL_EN_USB_CLKS, \
			anatop_base_addr + HW_ANADIG_USB2_PLL_480_CTRL_CLR);

	return ;
}
module_exit(mx6_usb_h1_exit);

MODULE_AUTHOR("Freescale Semiconductor");
MODULE_LICENSE("GPL");
