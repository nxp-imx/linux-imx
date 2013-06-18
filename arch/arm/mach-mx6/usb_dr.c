/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <mach/arc_otg.h>
#include <mach/hardware.h>
#include "devices-imx6q.h"
#include "regs-anadig.h"
#include "usb.h"

DEFINE_MUTEX(otg_wakeup_enable_mutex);
static int usbotg_init_ext(struct platform_device *pdev);
static void usbotg_uninit_ext(struct platform_device *pdev);
static void usbotg_clock_gate(bool on);
static void _dr_discharge_line(bool enable);
extern bool usb_icbug_swfix_need(void);
static void enter_phy_lowpower_suspend(struct fsl_usb2_platform_data *pdata, \
								bool enable);
static u32 wakeup_irq_enable_src; /* only useful at otg mode */
static u32 low_power_enable_src; /* only useful at otg mode */

/* The usb_phy1_clk do not have enable/disable function at clock.c
 * and PLL output for usb1's phy should be always enabled.
 * usb_phy1_clk only stands for usb uses pll3 as its parent.
 */
static struct clk *usb_phy1_clk;
static struct clk *usb_oh3_clk;
static u8 otg_used;

static void usbotg_wakeup_event_clear(void);
extern int clk_get_usecount(struct clk *clk);
/* Beginning of Common operation for DR port */

/*
 * platform data structs
 * 	- Which one to use is determined by CONFIG options in usb.h
 * 	- operating_mode plugged at run time
 */
static struct fsl_usb2_platform_data dr_utmi_config = {
	.name              = "DR",
	.init              = usbotg_init_ext,
	.exit              = usbotg_uninit_ext,
	.phy_mode          = FSL_USB2_PHY_UTMI_WIDE,
	.power_budget      = 500,		/* 500 mA max power */
	.usb_clock_for_pm  = usbotg_clock_gate,
	.transceiver       = "utmi",
	.phy_regs = USB_PHY0_BASE_ADDR,
	.dr_discharge_line = _dr_discharge_line,
	.lowpower	   = true, /* Default driver low power is true */
};

/* Platform data for wakeup operation */
static struct fsl_usb2_wakeup_platform_data dr_wakeup_config = {
	.name = "DR wakeup",
	.usb_clock_for_pm  = usbotg_clock_gate,
	.usb_wakeup_exhandle = usbotg_wakeup_event_clear,
};

static void usbotg_internal_phy_clock_gate(bool on)
{
	void __iomem *phy_reg = MX6_IO_ADDRESS(USB_PHY0_BASE_ADDR);
	if (on) {
		__raw_writel(BM_USBPHY_CTRL_CLKGATE, phy_reg + HW_USBPHY_CTRL_CLR);
	} else {
		__raw_writel(BM_USBPHY_CTRL_CLKGATE, phy_reg + HW_USBPHY_CTRL_SET);
	}
}

static int usb_phy_enable(struct fsl_usb2_platform_data *pdata)
{
	u32 tmp;
	void __iomem *phy_reg = MX6_IO_ADDRESS(USB_PHY0_BASE_ADDR);
	void __iomem *phy_ctrl;

	/* Stop then Reset */
	UOG_USBCMD &= ~UCMD_RUN_STOP;
	while (UOG_USBCMD & UCMD_RUN_STOP)
		;

	UOG_USBCMD |= UCMD_RESET;
	while ((UOG_USBCMD) & (UCMD_RESET))
		;

	/*
	 * If the controller reset does not put the PHY be out of
	 * low power mode, do it manually.
	 */
	if (UOG_PORTSC1 & PORTSC_PHCD) {
		UOG_PORTSC1 &= ~PORTSC_PHCD;
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
	if ((pdata->operating_mode == FSL_USB2_DR_HOST) ||
			(pdata->operating_mode == FSL_USB2_DR_OTG)) {
		/* enable FS/LS device */
		__raw_writel(BM_USBPHY_CTRL_ENUTMILEVEL2 | BM_USBPHY_CTRL_ENUTMILEVEL3
				, phy_reg + HW_USBPHY_CTRL_SET);
	}

	if (!usb_icbug_swfix_need())
		__raw_writel((1 << 17), phy_reg + HW_USBPHY_IP_SET);
	if (cpu_is_mx6sl())
		__raw_writel((1 << 18), phy_reg + HW_USBPHY_IP_SET);
	return 0;
}
/* Notes: configure USB clock*/
static int usbotg_init_ext(struct platform_device *pdev)
{
	struct clk *usb_clk;
	u32 ret;

	/* at mx6q: this clock is AHB clock for usb core */
	usb_clk = clk_get(NULL, "usboh3_clk");
	clk_enable(usb_clk);
	usb_oh3_clk = usb_clk;

	usb_clk = clk_get(NULL, "usb_phy1_clk");
	clk_enable(usb_clk);
	usb_phy1_clk = usb_clk;

	ret = usbotg_init(pdev);
	if (ret) {
		clk_disable(usb_oh3_clk);
		clk_put(usb_oh3_clk);
		clk_disable(usb_phy1_clk);
		clk_put(usb_phy1_clk);
		printk(KERN_ERR "otg init fails......\n");
		return ret;
	}
	if (!otg_used) {
		wakeup_irq_enable_src = 0;
		low_power_enable_src = 0;
		usb_phy_enable(pdev->dev.platform_data);
		enter_phy_lowpower_suspend(pdev->dev.platform_data, false);
		/*after the phy reset,can not read the readingvalue for id/vbus at
		* the register of otgsc ,cannot  read at once ,need delay 3 ms
		*/
		mdelay(3);
	}
	otg_used++;

	return ret;
}

static void usbotg_uninit_ext(struct platform_device *pdev)
{
	otg_used--;
	if (!otg_used) {
		clk_put(usb_phy1_clk);
		clk_put(usb_oh3_clk);
	}
}

static void usbotg_clock_gate(bool on)
{
	pr_debug("%s: on is %d\n", __func__, on);
	if (on) {
		clk_enable(usb_oh3_clk);
		clk_enable(usb_phy1_clk);
	} else {
		clk_disable(usb_phy1_clk);
		clk_disable(usb_oh3_clk);
	}
	pr_debug("usb_oh3_clk:%d, usb_phy_clk1_ref_count:%d\n", clk_get_usecount(usb_oh3_clk), clk_get_usecount(usb_phy1_clk));
}

static void dr_platform_phy_power_on(void)
{
	void __iomem *anatop_base_addr = MX6_IO_ADDRESS(ANATOP_BASE_ADDR);
	__raw_writel(BM_ANADIG_ANA_MISC0_STOP_MODE_CONFIG,
				anatop_base_addr + HW_ANADIG_ANA_MISC0_SET);
}


static void _dr_discharge_line(bool enable)
{
	void __iomem *anatop_base_addr = MX6_IO_ADDRESS(ANATOP_BASE_ADDR);
	void __iomem *phy_reg = MX6_IO_ADDRESS(USB_PHY0_BASE_ADDR);

	pr_debug("DR: %s  enable is %d\n", __func__, enable);
	if (enable) {
		__raw_writel(BM_USBPHY_DEBUG_CLKGATE, phy_reg + HW_USBPHY_DEBUG_CLR);
		__raw_writel(BM_ANADIG_USB1_LOOPBACK_UTMI_DIG_TST1
					|BM_ANADIG_USB1_LOOPBACK_TSTI_TX_EN,
			anatop_base_addr + HW_ANADIG_USB1_LOOPBACK);
	} else {
		__raw_writel(0x0,
			anatop_base_addr + HW_ANADIG_USB1_LOOPBACK);
		__raw_writel(BM_USBPHY_DEBUG_CLKGATE, phy_reg + HW_USBPHY_DEBUG_SET);
	}
}

/* Below two macros are used at otg mode to indicate usb mode*/
#define ENABLED_BY_HOST   (0x1 << 0)
#define ENABLED_BY_DEVICE (0x1 << 1)
static void enter_phy_lowpower_suspend(struct fsl_usb2_platform_data *pdata, bool enable)
{
	void __iomem *phy_reg = MX6_IO_ADDRESS(USB_PHY0_BASE_ADDR);
	u32 tmp;
	pr_debug("DR: %s begins, enable is %d\n", __func__, enable);

	if (enable) {
		UOG_PORTSC1 |= PORTSC_PHCD;
		tmp = (BM_USBPHY_PWD_TXPWDFS
			| BM_USBPHY_PWD_TXPWDIBIAS
			| BM_USBPHY_PWD_TXPWDV2I
			| BM_USBPHY_PWD_RXPWDENV
			| BM_USBPHY_PWD_RXPWD1PT1
			| BM_USBPHY_PWD_RXPWDDIFF
			| BM_USBPHY_PWD_RXPWDRX);
		__raw_writel(tmp, phy_reg + HW_USBPHY_PWD_SET);
		usbotg_internal_phy_clock_gate(false);

	} else {
		if (UOG_PORTSC1 & PORTSC_PHCD) {
			UOG_PORTSC1 &= ~PORTSC_PHCD;
			mdelay(1);
		}
		usbotg_internal_phy_clock_gate(true);
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
		 *
		 * Besides, the digital value needs 1ms debounce time to
		 * wait the value to be stable. We have expected the
		 * value from OTGSC is correct after calling this API.
		 *
		 * So delay 2ms is a save value.
		 */
		mdelay(2);

	}
	pr_debug("DR: %s ends, enable is %d\n", __func__, enable);
}

static void __phy_lowpower_suspend(struct fsl_usb2_platform_data *pdata, bool enable, int source)
{
	if (enable) {
		low_power_enable_src |= source;
#ifdef CONFIG_USB_OTG
		if (low_power_enable_src == (ENABLED_BY_HOST | ENABLED_BY_DEVICE)) {
			pr_debug("phy lowpower enabled\n");
			enter_phy_lowpower_suspend(pdata, enable);
		}
#else
		enter_phy_lowpower_suspend(pdata, enable);
#endif
	} else {
		pr_debug("phy lowpower disable\n");
		enter_phy_lowpower_suspend(pdata, enable);
		low_power_enable_src &= ~source;
	}
}

static void otg_wake_up_enable(struct fsl_usb2_platform_data *pdata, bool enable)
{
	void __iomem *phy_reg = MX6_IO_ADDRESS(USB_PHY0_BASE_ADDR);

	pr_debug("%s, enable is %d\n", __func__, enable);
	if (enable) {
		__raw_writel(BM_USBPHY_CTRL_ENDPDMCHG_WKUP
				| BM_USBPHY_CTRL_ENAUTOSET_USBCLKS
				| BM_USBPHY_CTRL_ENAUTOCLR_PHY_PWD
				| BM_USBPHY_CTRL_ENAUTOCLR_CLKGATE
				| BM_USBPHY_CTRL_ENAUTOCLR_USBCLKGATE
				| BM_USBPHY_CTRL_ENAUTO_PWRON_PLL , phy_reg + HW_USBPHY_CTRL_SET);
		USB_OTG_CTRL |= UCTRL_OWIE;
	} else {
		__raw_writel(BM_USBPHY_CTRL_ENDPDMCHG_WKUP
				| BM_USBPHY_CTRL_ENAUTOSET_USBCLKS
				| BM_USBPHY_CTRL_ENAUTOCLR_PHY_PWD
				| BM_USBPHY_CTRL_ENAUTOCLR_CLKGATE
				| BM_USBPHY_CTRL_ENAUTOCLR_USBCLKGATE
				| BM_USBPHY_CTRL_ENAUTO_PWRON_PLL , phy_reg + HW_USBPHY_CTRL_CLR);
		USB_OTG_CTRL &= ~UCTRL_OWIE;
		/* The interrupt must be disabled for at least 3 clock
		 * cycles of the standby clock(32k Hz) , that is 0.094 ms*/
		udelay(100);
	}
}

static void __wakeup_irq_enable(struct fsl_usb2_platform_data *pdata, bool on, int source)
 {
	/* otg host and device share the OWIE bit, only when host and device
	 * all enable the wakeup irq, we can enable the OWIE bit
	 */
	mutex_lock(&otg_wakeup_enable_mutex);
	if (on) {
#ifdef CONFIG_USB_OTG
		wakeup_irq_enable_src |= source;
		if (wakeup_irq_enable_src == (ENABLED_BY_HOST | ENABLED_BY_DEVICE)) {
			otg_wake_up_enable(pdata, on);
		}
#else
		otg_wake_up_enable(pdata, on);
#endif
	} else {
		otg_wake_up_enable(pdata, on);
		wakeup_irq_enable_src &= ~source;
		/* The interrupt must be disabled for at least 3 clock
		 * cycles of the standby clock(32k Hz) , that is 0.094 ms*/
		udelay(100);
	}
	mutex_unlock(&otg_wakeup_enable_mutex);
}

/* The wakeup operation for DR port, it will clear the wakeup irq status
 * and re-enable the wakeup
 */
static void usbotg_wakeup_event_clear(void)
{
	int wakeup_req = USB_OTG_CTRL & UCTRL_OWIR;

	if (wakeup_req != 0) {
		printk(KERN_INFO "Unknown wakeup.(OTGSC 0x%x)\n", UOG_OTGSC);
		/* Disable OWIE to clear OWIR, wait 3 clock
		 * cycles of standly clock(32KHz)
		 */
		USB_OTG_CTRL &= ~UCTRL_OWIE;
		udelay(100);
		USB_OTG_CTRL |= UCTRL_OWIE;
	}
}

/* End of Common operation for DR port */

#ifdef CONFIG_USB_EHCI_ARC_OTG
/* Beginning of host related operation for DR port */
static void fsl_platform_otg_set_usb_phy_dis(
		struct fsl_usb2_platform_data *pdata, bool enable)
{
	u32 usb_phy_ctrl_dcdt = 0;
	void __iomem *anatop_base_addr = MX6_IO_ADDRESS(ANATOP_BASE_ADDR);
	usb_phy_ctrl_dcdt = __raw_readl(
			MX6_IO_ADDRESS(pdata->phy_regs) + HW_USBPHY_CTRL) &
			BM_USBPHY_CTRL_ENHOSTDISCONDETECT;
	if (enable) {
		if (usb_phy_ctrl_dcdt == 0) {
			__raw_writel(BM_ANADIG_USB1_PLL_480_CTRL_EN_USB_CLKS,
					anatop_base_addr + HW_ANADIG_USB1_PLL_480_CTRL_CLR);

			__raw_writel(BM_USBPHY_PWD_RXPWDENV,
					MX6_IO_ADDRESS(pdata->phy_regs) + HW_USBPHY_PWD_SET);

			udelay(300);

			__raw_writel(BM_USBPHY_CTRL_ENHOSTDISCONDETECT,
				MX6_IO_ADDRESS(pdata->phy_regs)
				+ HW_USBPHY_CTRL_SET);

			UOG_USBSTS |= (1 << 7);

			while ((UOG_USBSTS & (1 << 7)) == 0)
				;

			udelay(2);

			__raw_writel(BM_USBPHY_PWD_RXPWDENV,
					MX6_IO_ADDRESS(pdata->phy_regs) + HW_USBPHY_PWD_CLR);

			__raw_writel(BM_ANADIG_USB1_PLL_480_CTRL_EN_USB_CLKS,
					anatop_base_addr + HW_ANADIG_USB1_PLL_480_CTRL_SET);

		}
	} else {
		if (usb_phy_ctrl_dcdt
				== BM_USBPHY_CTRL_ENHOSTDISCONDETECT)
			__raw_writel(BM_USBPHY_CTRL_ENHOSTDISCONDETECT,
				MX6_IO_ADDRESS(pdata->phy_regs)
				+ HW_USBPHY_CTRL_CLR);
	}
}

static void _host_platform_rh_suspend_swfix(struct fsl_usb2_platform_data *pdata)
{
	void __iomem *phy_reg = MX6_IO_ADDRESS(USB_PHY0_BASE_ADDR);
	u32 tmp;
	u32 index = 0;

	/* before we set and then clear PWD bit,
	 * we must wait LS to be suspend */
	if ((UOG_PORTSC1 & (3 << 26)) != (1 << 26)) {
		while (((UOG_PORTSC1 & PORTSC_LS_MASK) != PORTSC_LS_J_STATE) &&
				(index < 1000)) {
			index++;
			udelay(4);
		}
	} else {
		while (((UOG_PORTSC1 & PORTSC_LS_MASK) != PORTSC_LS_K_STATE) &&
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

	fsl_platform_otg_set_usb_phy_dis(pdata, 0);
}

static void _host_platform_rh_resume_swfix(struct fsl_usb2_platform_data *pdata)
{
	u32 index = 0;

	if ((UOG_PORTSC1 & (PORTSC_PORT_SPEED_MASK)) != PORTSC_PORT_SPEED_HIGH)
		return ;
	while ((UOG_PORTSC1 & PORTSC_PORT_FORCE_RESUME)
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
	fsl_platform_otg_set_usb_phy_dis(pdata, 1);
}
static void _host_platform_rh_suspend(struct fsl_usb2_platform_data *pdata)
{
	/*for mx6sl ,we do not need any sw fix*/
	if (cpu_is_mx6sl())
		return ;
	__raw_writel(BM_USBPHY_CTRL_ENHOSTDISCONDETECT,
		MX6_IO_ADDRESS(pdata->phy_regs)
		+ HW_USBPHY_CTRL_CLR);
}

static void _host_platform_rh_resume(struct fsl_usb2_platform_data *pdata)
{
	u32 index = 0;

	/*for mx6sl ,we do not need any sw fix*/
	if (cpu_is_mx6sl())
		return ;
	if ((UOG_PORTSC1 & (PORTSC_PORT_SPEED_MASK)) != PORTSC_PORT_SPEED_HIGH)
		return ;
	while ((UOG_PORTSC1 & PORTSC_PORT_FORCE_RESUME)
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

static void _host_phy_lowpower_suspend(struct fsl_usb2_platform_data *pdata, bool enable)
{
	__phy_lowpower_suspend(pdata, enable, ENABLED_BY_HOST);
}

static void _host_wakeup_enable(struct fsl_usb2_platform_data *pdata, bool enable)
{
	void __iomem *phy_reg = MX6_IO_ADDRESS(USB_PHY0_BASE_ADDR);
	__wakeup_irq_enable(pdata, enable, ENABLED_BY_HOST);
	if (enable) {
		pr_debug("host wakeup enable\n");
		USB_OTG_CTRL |= UCTRL_WKUP_ID_EN;
		__raw_writel(BM_USBPHY_CTRL_ENIDCHG_WKUP, phy_reg + HW_USBPHY_CTRL_SET);
	} else {
		pr_debug("host wakeup disable\n");
		__raw_writel(BM_USBPHY_CTRL_ENIDCHG_WKUP, phy_reg + HW_USBPHY_CTRL_CLR);
		USB_OTG_CTRL &= ~UCTRL_WKUP_ID_EN;
		/* The interrupt must be disabled for at least 3 clock
		 * cycles of the standby clock(32k Hz) , that is 0.094 ms*/
		udelay(100);
	}
	pr_debug("the otgsc is 0x%x, usbsts is 0x%x, portsc is 0x%x, otgctrl: 0x%x\n", UOG_OTGSC, UOG_USBSTS, UOG_PORTSC1, USB_OTG_CTRL);
}

static enum usb_wakeup_event _is_host_wakeup(struct fsl_usb2_platform_data *pdata)
{
	u32 wakeup_req = USB_OTG_CTRL & UCTRL_OWIR;
	u32 otgsc = UOG_OTGSC;

	if (wakeup_req) {
		pr_debug("the otgsc is 0x%x, usbsts is 0x%x, portsc is 0x%x, wakeup_irq is 0x%x\n", UOG_OTGSC, UOG_USBSTS, UOG_PORTSC1, wakeup_req);
	}
	/* if ID change sts, it is a host wakeup event */
	if (otgsc & OTGSC_IS_USB_ID) {
		pr_debug("otg host ID wakeup\n");
		return WAKEUP_EVENT_ID;
	}
	if (wakeup_req  && (!(otgsc & OTGSC_STS_USB_ID))) {
		pr_debug("otg host Remote wakeup\n");
		return WAKEUP_EVENT_DPDM;
	}
	return WAKEUP_EVENT_INVALID;
}

static void host_wakeup_handler(struct fsl_usb2_platform_data *pdata)
{
	_host_phy_lowpower_suspend(pdata, false);
	_host_wakeup_enable(pdata, false);
}
/* End of host related operation for DR port */
#endif /* CONFIG_USB_EHCI_ARC_OTG */


#ifdef CONFIG_USB_GADGET_ARC
/* Beginning of device related operation for DR port */
static void _device_phy_lowpower_suspend(struct fsl_usb2_platform_data *pdata, bool enable)
{
	__phy_lowpower_suspend(pdata, enable, ENABLED_BY_DEVICE);
}

static void _device_wakeup_enable(struct fsl_usb2_platform_data *pdata, bool enable)
{
	void __iomem *phy_reg = MX6_IO_ADDRESS(USB_PHY0_BASE_ADDR);
	__wakeup_irq_enable(pdata, enable, ENABLED_BY_DEVICE);
	/* if udc is not used by any gadget, we can not enable the vbus wakeup */
	if (!pdata->port_enables) {
		USB_OTG_CTRL &= ~UCTRL_WKUP_VBUS_EN;
		return;
	}
	if (enable) {
		pr_debug("device wakeup enable\n");
		USB_OTG_CTRL |= UCTRL_WKUP_VBUS_EN;
		__raw_writel(BM_USBPHY_CTRL_ENVBUSCHG_WKUP, phy_reg + HW_USBPHY_CTRL_SET);
	} else {
		pr_debug("device wakeup disable\n");
		__raw_writel(BM_USBPHY_CTRL_ENVBUSCHG_WKUP, phy_reg + HW_USBPHY_CTRL_CLR);
		USB_OTG_CTRL &= ~UCTRL_WKUP_VBUS_EN;
	}
}

static enum usb_wakeup_event _is_device_wakeup(struct fsl_usb2_platform_data *pdata)
{
	int wakeup_req = USB_OTG_CTRL & UCTRL_OWIR;
	pr_debug("%s\n", __func__);

	/* if ID=1, it is a device wakeup event */
	if (wakeup_req && (UOG_OTGSC & OTGSC_STS_USB_ID) && (UOG_USBSTS & USBSTS_URI)) {
		printk(KERN_INFO "otg udc wakeup, host sends reset signal\n");
		return WAKEUP_EVENT_DPDM;
	}
	if (wakeup_req && (UOG_OTGSC & OTGSC_STS_USB_ID) &&  \
		((UOG_USBSTS & USBSTS_PCI) || (UOG_PORTSC1 & PORTSC_PORT_FORCE_RESUME))) {
		/*
		 * When the line state from J to K, the Port Change Detect bit
		 * in the USBSTS register is also set to '1'.
		 */
		printk(KERN_INFO "otg udc wakeup, host sends resume signal\n");
		return WAKEUP_EVENT_DPDM;
	}
	if (wakeup_req && (UOG_OTGSC & OTGSC_STS_USB_ID) && (UOG_OTGSC & OTGSC_STS_A_VBUS_VALID) \
		&& (UOG_OTGSC & OTGSC_IS_B_SESSION_VALID)) {
		printk(KERN_INFO "otg udc vbus rising wakeup\n");
		return WAKEUP_EVENT_VBUS;
	}
	if (wakeup_req && (UOG_OTGSC & OTGSC_STS_USB_ID) && !(UOG_OTGSC & OTGSC_STS_A_VBUS_VALID)) {
		printk(KERN_INFO "otg udc vbus falling wakeup\n");
		return WAKEUP_EVENT_VBUS;
	}

	return WAKEUP_EVENT_INVALID;
}

static void device_wakeup_handler(struct fsl_usb2_platform_data *pdata)
{
	_device_phy_lowpower_suspend(pdata, false);
	_device_wakeup_enable(pdata, false);
}

/* end of device related operation for DR port */
#endif /* CONFIG_USB_GADGET_ARC */

static struct platform_device *pdev[3], *pdev_wakeup;
static driver_vbus_func  mx6_set_usb_otg_vbus;
static int devnum;
static int  __init mx6_usb_dr_init(void)
{
	int i = 0;
	void __iomem *anatop_base_addr = MX6_IO_ADDRESS(ANATOP_BASE_ADDR);
	struct imx_fsl_usb2_wakeup_data imx6q_fsl_otg_wakeup_data =
		imx_fsl_usb2_wakeup_data_entry_single(MX6Q, 0, OTG);
	struct imx_mxc_ehci_data __maybe_unused imx6q_mxc_ehci_otg_data =
		imx_mxc_ehci_data_entry_single(MX6Q, 0, OTG);
	struct imx_fsl_usb2_udc_data __maybe_unused imx6q_fsl_usb2_udc_data =
		imx_fsl_usb2_udc_data_entry_single(MX6Q);
	struct imx_fsl_usb2_otg_data __maybe_unused imx6q_fsl_usb2_otg_data  =
		imx_fsl_usb2_otg_data_entry_single(MX6Q);

	/* Some phy and power's special controls for otg
	 * 1. The external charger detector needs to be disabled
	 * or the signal at DP will be poor
	 * 2. The EN_USB_CLKS is always enabled.
	 * The PLL's power is controlled by usb and others who
	 * use pll3 too.
	 */
	__raw_writel(BM_ANADIG_USB1_CHRG_DETECT_EN_B  \
			| BM_ANADIG_USB1_CHRG_DETECT_CHK_CHRG_B,  \
			anatop_base_addr + HW_ANADIG_USB1_CHRG_DETECT);
	__raw_writel(BM_ANADIG_USB1_PLL_480_CTRL_EN_USB_CLKS,
			anatop_base_addr + HW_ANADIG_USB1_PLL_480_CTRL_SET);
	mx6_get_otghost_vbus_func(&mx6_set_usb_otg_vbus);
	dr_utmi_config.platform_driver_vbus = mx6_set_usb_otg_vbus;

#ifdef CONFIG_USB_OTG
	/* wake_up_enable is useless, just for usb_register_remote_wakeup execution*/
	dr_utmi_config.wake_up_enable = _device_wakeup_enable;
	dr_utmi_config.operating_mode = FSL_USB2_DR_OTG;
	dr_utmi_config.wakeup_pdata = &dr_wakeup_config;
	pdev[i] = imx6q_add_fsl_usb2_otg(&dr_utmi_config);
	dr_wakeup_config.usb_pdata[i] = pdev[i]->dev.platform_data;
	i++;
#endif
#ifdef CONFIG_USB_EHCI_ARC_OTG
	dr_utmi_config.operating_mode = DR_HOST_MODE;
	dr_utmi_config.wake_up_enable = _host_wakeup_enable;
	if (usb_icbug_swfix_need()) {
		dr_utmi_config.platform_rh_suspend = _host_platform_rh_suspend_swfix;
		dr_utmi_config.platform_rh_resume  = _host_platform_rh_resume_swfix;
	} else {
		dr_utmi_config.platform_rh_suspend = _host_platform_rh_suspend;
		dr_utmi_config.platform_rh_resume  = _host_platform_rh_resume;
	}
	dr_utmi_config.platform_set_disconnect_det = fsl_platform_otg_set_usb_phy_dis;
	dr_utmi_config.phy_lowpower_suspend = _host_phy_lowpower_suspend;
	dr_utmi_config.is_wakeup_event = _is_host_wakeup;
	dr_utmi_config.wakeup_pdata = &dr_wakeup_config;
	dr_utmi_config.wakeup_handler = host_wakeup_handler;
	dr_utmi_config.platform_phy_power_on = dr_platform_phy_power_on;
	pdev[i] = imx6q_add_fsl_ehci_otg(&dr_utmi_config);
	dr_wakeup_config.usb_pdata[i] = pdev[i]->dev.platform_data;
	i++;
#endif
#ifdef CONFIG_USB_GADGET_ARC
	dr_utmi_config.operating_mode = DR_UDC_MODE;
	dr_utmi_config.wake_up_enable = _device_wakeup_enable;
	dr_utmi_config.platform_rh_suspend = NULL;
	dr_utmi_config.platform_rh_resume  = NULL;
	dr_utmi_config.platform_set_disconnect_det = NULL;
	dr_utmi_config.phy_lowpower_suspend = _device_phy_lowpower_suspend;
	dr_utmi_config.is_wakeup_event = _is_device_wakeup;
	dr_utmi_config.wakeup_pdata = &dr_wakeup_config;
	dr_utmi_config.wakeup_handler = device_wakeup_handler;
	dr_utmi_config.charger_base_addr = anatop_base_addr;
	dr_utmi_config.platform_phy_power_on = dr_platform_phy_power_on;
	pdev[i] = imx6q_add_fsl_usb2_udc(&dr_utmi_config);
	dr_wakeup_config.usb_pdata[i] = pdev[i]->dev.platform_data;
	i++;
#endif
	devnum = i;
	/* register wakeup device */
	pdev_wakeup = imx6q_add_fsl_usb2_otg_wakeup(&dr_wakeup_config);
	for (i = 0; i < devnum; i++) {
		platform_device_add(pdev[i]);
		((struct fsl_usb2_platform_data *)(pdev[i]->dev.platform_data))->wakeup_pdata =
			(struct fsl_usb2_wakeup_platform_data *)(pdev_wakeup->dev.platform_data);
	}

	return 0;
}
module_init(mx6_usb_dr_init);

static void __exit mx6_usb_dr_exit(void)
{
	int i;
	void __iomem *anatop_base_addr = MX6_IO_ADDRESS(ANATOP_BASE_ADDR);

	for (i = 0; i < devnum; i++)
		platform_device_del(pdev[devnum-i-1]);
	platform_device_unregister(pdev_wakeup);
	otg_used = 0;

	__raw_writel(BM_ANADIG_USB1_PLL_480_CTRL_EN_USB_CLKS,
			anatop_base_addr + HW_ANADIG_USB1_PLL_480_CTRL_CLR);
	return ;
}
module_exit(mx6_usb_dr_exit);

MODULE_AUTHOR("Freescale Semiconductor");
MODULE_LICENSE("GPL");
