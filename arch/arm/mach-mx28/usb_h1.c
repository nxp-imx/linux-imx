/*
 * Copyright (C) 2009-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/fsl_devices.h>
#include <mach/arc_otg.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/irqs.h>
#include "usb.h"

extern int clk_get_usecount(struct clk *clk);
static struct clk *usb_clk;
static struct clk *usb_phy_clk;
static struct platform_device *h1_pdev;

#ifdef CONFIG_WORKAROUND_ARCUSB_REG_RW
static void fsl_safe_writel(u32 val32, volatile u32 *addr)
{
	__asm__ ("swp %0, %0, [%1]" : : "r"(val32), "r"(addr));
}
#endif
static void usbh1_internal_phy_clock_gate(bool on)
{
	void __iomem *phy_reg = IO_ADDRESS(USBPHY1_PHYS_ADDR);

	pr_debug ("%s, Clock %s UTMI \n", __func__, on ? "on" : "off");

	if (on)
		__raw_writel(BM_USBPHY_CTRL_CLKGATE, phy_reg + HW_USBPHY_CTRL_CLR);
	else
		__raw_writel(BM_USBPHY_CTRL_CLKGATE, phy_reg + HW_USBPHY_CTRL_SET);
}

static int fsl_usb_host_init_ext(struct platform_device *pdev)
{
	usb_clk = clk_get(NULL, "usb_clk1");
	clk_enable(usb_clk);
	clk_put(usb_clk);

	usb_phy_clk = clk_get(NULL, "usb_phy_clk1");
	clk_enable(usb_phy_clk);
	clk_put(usb_phy_clk);

	usbh1_internal_phy_clock_gate(true);
	return fsl_usb_host_init(pdev);
}

static void fsl_usb_host_uninit_ext(struct fsl_usb2_platform_data *pdata)
{
	fsl_usb_host_uninit(pdata);
	usbh1_internal_phy_clock_gate(false);
	clk_disable(usb_phy_clk);
	clk_disable(usb_clk);
}

static void usbh1_clock_gate(bool on)
{
	pr_debug("%s: on is %d\n", __func__, on);
	if (on) {
		clk_enable(usb_clk);
		clk_enable(usb_phy_clk);
	} else {
		clk_disable(usb_phy_clk);
		clk_disable(usb_clk);
	}
	pr_debug("usb_clk1_ref_count:%d, usb_phy_clk1_ref_count:%d\n", clk_get_usecount(usb_clk), clk_get_usecount(usb_phy_clk));
}

static void _wake_up_enable(struct fsl_usb2_platform_data *pdata, bool enable)
{
	u32 tmp;
	void __iomem *phy_reg = IO_ADDRESS(pdata->phy_regs);

	pr_debug("host1, %s, enable is %d\n", __func__, enable);
	if (enable) {
		tmp = BM_USBPHY_CTRL_RESUME_IRQ | BM_USBPHY_CTRL_WAKEUP_IRQ;
		__raw_writel(tmp, phy_reg + HW_USBPHY_CTRL_CLR);

		tmp = (BM_USBPHY_CTRL_ENIRQWAKEUP
			| BM_USBPHY_CTRL_ENIDCHG_WKUP
			| BM_USBPHY_CTRL_ENDPDMCHG_WKUP);
		__raw_writel(tmp, phy_reg + HW_USBPHY_CTRL_SET);
	} else {
		tmp = BM_USBPHY_CTRL_RESUME_IRQ | BM_USBPHY_CTRL_WAKEUP_IRQ;
		__raw_writel(tmp, phy_reg + HW_USBPHY_CTRL_CLR);

		tmp = (BM_USBPHY_CTRL_ENIRQWAKEUP
			| BM_USBPHY_CTRL_ENIDCHG_WKUP
			| BM_USBPHY_CTRL_ENDPDMCHG_WKUP);

		__raw_writel(tmp, phy_reg + HW_USBPHY_CTRL_CLR);
		/* The interrupt must be disabled for at least 3
		* cycles of the standby clock(32k Hz) , that is 0.094 ms*/
		udelay(100);
	}
}

static void _phy_lowpower_suspend(struct fsl_usb2_platform_data *pdata, bool enable)
{
	u32 tmp;
	void __iomem *phy_reg = IO_ADDRESS(pdata->phy_regs);
	void __iomem *usb_reg = pdata->regs;
	pr_debug("host1, %s, enable is %d\n", __func__, enable);
	if (enable) {
		tmp = __raw_readl(usb_reg + UOG_PORTSC1);
		tmp |= PORTSC_PHCD;
		fsl_safe_writel(tmp, usb_reg + UOG_PORTSC1);

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
		tmp = __raw_readl(usb_reg + UOG_PORTSC1);
		if (tmp & PORTSC_PHCD) {
			tmp &= ~PORTSC_PHCD;
			fsl_safe_writel(tmp, usb_reg + UOG_PORTSC1);
			mdelay(1);
		}

		usbh1_internal_phy_clock_gate(true);

		tmp = (BM_USBPHY_PWD_TXPWDFS
			| BM_USBPHY_PWD_TXPWDIBIAS
			| BM_USBPHY_PWD_TXPWDV2I
			| BM_USBPHY_PWD_RXPWDENV
			| BM_USBPHY_PWD_RXPWD1PT1
			| BM_USBPHY_PWD_RXPWDDIFF
			| BM_USBPHY_PWD_RXPWDRX);
		__raw_writel(tmp, phy_reg + HW_USBPHY_PWD_CLR);
	}
}

static enum usb_wakeup_event _is_usbh1_wakeup(struct fsl_usb2_platform_data *pdata)
{
	void __iomem *phy_reg = IO_ADDRESS(pdata->phy_regs);
	u32 tmp;

	pr_debug("host1, %s\n", __func__);
	tmp = BM_USBPHY_CTRL_RESUME_IRQ | BM_USBPHY_CTRL_WAKEUP_IRQ;
	if (__raw_readl(phy_reg + HW_USBPHY_CTRL) && tmp) {
		__raw_writel(tmp, phy_reg + HW_USBPHY_CTRL_CLR);
		return WAKEUP_EVENT_DPDM;
	} else
		return WAKEUP_EVENT_INVALID;
}

static void h1_wakeup_handler(struct fsl_usb2_platform_data *pdata)
{
	_phy_lowpower_suspend(pdata, false);
	_wake_up_enable(pdata, false);
}

static void usbh1_wakeup_event_clear(void)
{
	void __iomem *phy_reg = IO_ADDRESS(USBPHY1_PHYS_ADDR);
	u32 wakeup_irq_bits;

	wakeup_irq_bits = BM_USBPHY_CTRL_RESUME_IRQ | BM_USBPHY_CTRL_WAKEUP_IRQ;
	if (__raw_readl(phy_reg + HW_USBPHY_CTRL) && wakeup_irq_bits) {
		/* clear the wakeup interrupt status */
		__raw_writel(wakeup_irq_bits, phy_reg + HW_USBPHY_CTRL_CLR);
	}
}

static void fsl_platform_h1_set_usb_phy_dis(
		struct fsl_usb2_platform_data *pdata, bool enable)
{
	u32 usb_phy_ctrl_dcdt = 0;
	usb_phy_ctrl_dcdt = __raw_readl(
			IO_ADDRESS(pdata->phy_regs) + HW_USBPHY_CTRL) &
			BM_USBPHY_CTRL_ENHOSTDISCONDETECT;
	if (enable) {
		if (usb_phy_ctrl_dcdt == 0) {
			__raw_writel(BM_USBPHY_CTRL_ENHOSTDISCONDETECT,
				IO_ADDRESS(pdata->phy_regs)
				+ HW_USBPHY_CTRL_SET);
		}
	} else {
		if (usb_phy_ctrl_dcdt
				== BM_USBPHY_CTRL_ENHOSTDISCONDETECT)
			__raw_writel(BM_USBPHY_CTRL_ENHOSTDISCONDETECT,
				IO_ADDRESS(pdata->phy_regs)
				+ HW_USBPHY_CTRL_CLR);
	}
}

static void usb_host_phy_suspend(struct fsl_usb2_platform_data *pdata)
{
	void __iomem *phy_reg = IO_ADDRESS(USBPHY1_PHYS_ADDR);
	void __iomem *usb_reg = pdata->regs;
	u32 tmp;
	u32 index = 0;
	tmp = __raw_readl(usb_reg + UOG_PORTSC1);

	/* before we set and then clear PWD bit,
	 * we must wait LS to be J */
	if ((tmp & (3 << 26)) != (1 << 26)) {
		while (((tmp & PORTSC_LS_MASK) != PORTSC_LS_J_STATE) &&
				(index < 4)) {
			index++;
			msleep(1);
			tmp = __raw_readl(usb_reg + UOG_PORTSC1);
		}
	} else {
		while (((tmp & PORTSC_LS_MASK) != PORTSC_LS_K_STATE) &&
				(index < 4)) {
			index++;
			msleep(1);
			tmp = __raw_readl(usb_reg + UOG_PORTSC1);
		}
	}

	if (index >= 4)
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

static void usb_host_phy_resume(struct fsl_usb2_platform_data *pdata)
{
	u32 index = 0;
	u32 tmp;
	void __iomem *usb_reg = pdata->regs;
	tmp = __raw_readl(usb_reg + UOG_PORTSC1);

	if ((tmp & (3 << 26)) != (2 << 26))
		return ;

	while ((tmp & PORTSC_PORT_FORCE_RESUME)
			&& (index < 500)) {
		msleep(1);
		index++;
		tmp = __raw_readl(usb_reg + UOG_PORTSC1);
	}

	if (index >= 500)
		printk(KERN_ERR "%s big error\n", __func__);

	udelay(500);

	fsl_platform_h1_set_usb_phy_dis(pdata, 1);
}

static struct fsl_usb2_platform_data usbh1_config = {
	.name = "Host 1",
	.platform_init = fsl_usb_host_init_ext,
	.platform_uninit = fsl_usb_host_uninit_ext,
	.operating_mode = FSL_USB2_MPH_HOST,
	.phy_mode = FSL_USB2_PHY_UTMI_WIDE,
	.power_budget = 500,	/* 500 mA max power */
	.platform_suspend = usb_host_phy_suspend,
	.platform_resume = usb_host_phy_resume,
	.transceiver = "utmi",
	.usb_clock_for_pm  = usbh1_clock_gate,
	.wake_up_enable = _wake_up_enable,
	.phy_lowpower_suspend = _phy_lowpower_suspend,
	.is_wakeup_event = _is_usbh1_wakeup,
	.wakeup_handler = h1_wakeup_handler,
	.phy_regs = USBPHY1_PHYS_ADDR,
};

static struct fsl_usb2_wakeup_platform_data usbh1_wakeup_config = {
	.name = "USBH1 wakeup",
	.usb_clock_for_pm  = usbh1_clock_gate,
	.usb_pdata = {&usbh1_config, NULL, NULL},
	.usb_wakeup_exhandle = usbh1_wakeup_event_clear,
};

/* The resources for kinds of usb devices */
static struct resource usbh1_resources[] = {
	[0] = {
	       .start = (u32) (USBCTRL1_PHYS_ADDR),
	       .end = (u32) (USBCTRL1_PHYS_ADDR + 0x1ff),
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = IRQ_USB1,
	       .flags = IORESOURCE_IRQ,
	       },
};

static struct resource usbh1_wakeup_resources[] = {
	{
		.start = IRQ_USB1_WAKEUP, /*wakeup irq*/
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = IRQ_USB1,
		.flags = IORESOURCE_IRQ,/* usb core irq */
	},
};

struct platform_device mxs_usbh1_wakeup_device = {
	.name = "usb_wakeup",
	.id   = 2,
	.num_resources = ARRAY_SIZE(usbh1_wakeup_resources),
	.resource = usbh1_wakeup_resources,
};

static int __init usbh1_init(void)
{
	struct platform_device *pdev;

	usbh1_config.wakeup_pdata = &usbh1_wakeup_config;
	pdev = host_pdev_register(usbh1_resources,
			ARRAY_SIZE(usbh1_resources), &usbh1_config);

	h1_pdev = pdev;
	pr_debug("%s: \n", __func__);

	/* the platform device(usb h1)'s pdata address has changed */
	usbh1_wakeup_config.usb_pdata[0] = pdev->dev.platform_data;
	mxs_usbh1_wakeup_device.dev.platform_data = &usbh1_wakeup_config;

	if (platform_device_register(&mxs_usbh1_wakeup_device))
		printk(KERN_ERR "usb h1 wakeup device\n");
	else
		printk(KERN_INFO "usb h1 wakeup device is registered\n");

	return 0;
}
module_init(usbh1_init);
