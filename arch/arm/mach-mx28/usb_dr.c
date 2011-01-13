/*
 * Copyright (C) 2009-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/gpio.h>

#include <mach/irqs.h>
#include <mach/arc_otg.h>
#include "usb.h"
#include "mx28_pins.h"
#define USB_POWER_ENABLE MXS_PIN_TO_GPIO(PINID_AUART2_TX)

extern int clk_get_usecount(struct clk *clk);
static struct clk *usb_clk;
static struct clk *usb_phy_clk;
static struct platform_device *otg_host_pdev;

/* Beginning of Common operation for DR port */
void fsl_phy_usb_utmi_init(struct fsl_xcvr_ops *this)
{
}

void fsl_phy_usb_utmi_uninit(struct fsl_xcvr_ops *this)
{
}

/*!
 * set vbus power
 *
 * @param       view  viewport register
 * @param       on    power on or off
 */
void fsl_phy_set_power(struct fsl_xcvr_ops *this,
			struct fsl_usb2_platform_data *pdata, int on)
{
	/* USB_POWER_ENABLE_PIN have request at pin init*/
	if (pdata->phy_regs != USBPHY1_PHYS_ADDR) {
		pr_debug("%s: on is %d\n", __func__, on);
		gpio_direction_output(USB_POWER_ENABLE, on);
		gpio_set_value(USB_POWER_ENABLE, on);
	}
}

static void usb_host_phy_resume(struct fsl_usb2_platform_data *plat)
{
	fsl_platform_set_usb_phy_dis(plat, 0);
}

static int internal_phy_clk_already_on;
static void usbotg_internal_phy_clock_gate(bool on)
{
	u32 tmp;
	void __iomem *phy_reg = IO_ADDRESS(USBPHY0_PHYS_ADDR);
	if (on) {
		internal_phy_clk_already_on += 1;
		if (internal_phy_clk_already_on == 1) {
			pr_debug ("%s, Clock on UTMI \n", __func__);
			tmp = BM_USBPHY_CTRL_SFTRST | BM_USBPHY_CTRL_CLKGATE;
			__raw_writel(tmp, phy_reg + HW_USBPHY_CTRL_CLR);
		}
	} else {
		internal_phy_clk_already_on -= 1;
		if (internal_phy_clk_already_on == 0) {
			pr_debug ("%s, Clock off UTMI \n", __func__);
			tmp = BM_USBPHY_CTRL_CLKGATE;
			__raw_writel(tmp, phy_reg + HW_USBPHY_CTRL_SET);
		}
	}
	if (internal_phy_clk_already_on < 0)
		printk(KERN_ERR "please check internal phy clock ON/OFF sequence \n");
}

static int usbotg_init_ext(struct platform_device *pdev)
{
	usb_clk = clk_get(NULL, "usb_clk0");
	clk_enable(usb_clk);
	clk_put(usb_clk);

	usb_phy_clk = clk_get(NULL, "usb_phy_clk0");
	clk_enable(usb_phy_clk);
	clk_put(usb_phy_clk);

	usbotg_internal_phy_clock_gate(true);
	return usbotg_init(pdev);
}

static void usbotg_uninit_ext(struct fsl_usb2_platform_data *pdata)
{
	usbotg_uninit(pdata);

	usbotg_internal_phy_clock_gate(false);
	clk_disable(usb_phy_clk);
	clk_disable(usb_clk);
}

static void usbotg_clock_gate(bool on)
{
	pr_debug("%s: on is %d\n", __func__, on);
	if (on) {
		clk_enable(usb_clk);
		clk_enable(usb_phy_clk);
		usbotg_internal_phy_clock_gate(on);
	} else {
		usbotg_internal_phy_clock_gate(on);
		clk_disable(usb_phy_clk);
		clk_disable(usb_clk);
	}

	pr_debug("usb_clk0_ref_count:%d, usb_phy_clk0_ref_count:%d\n", clk_get_usecount(usb_clk), clk_get_usecount(usb_phy_clk));
}

/* Below two macros are used at otg mode to indicate usb mode*/
#define ENABLED_BY_HOST   (0x1 << 0)
#define ENABLED_BY_DEVICE (0x1 << 1)
static u32 low_power_enable_src; /* only useful at otg mode */
static void enter_phy_lowpower_suspend(struct fsl_usb2_platform_data *pdata, bool enable)
{
	void __iomem *phy_reg = IO_ADDRESS(pdata->phy_regs);
	void __iomem *usb_reg = pdata->regs;
	u32 tmp;
	pr_debug("DR: %s, enable is %d\n", __func__, enable);

	if (enable) {
		tmp = __raw_readl(usb_reg + UOG_PORTSC1);
		tmp |= PORTSC_PHCD;
		__raw_writel(tmp, usb_reg + UOG_PORTSC1);

		pr_debug("%s, Poweroff UTMI \n", __func__);

		tmp = (BM_USBPHY_PWD_TXPWDFS
			| BM_USBPHY_PWD_TXPWDIBIAS
			| BM_USBPHY_PWD_TXPWDV2I
			| BM_USBPHY_PWD_RXPWDENV
			| BM_USBPHY_PWD_RXPWD1PT1
			| BM_USBPHY_PWD_RXPWDDIFF
			| BM_USBPHY_PWD_RXPWDRX);
		__raw_writel(tmp, phy_reg + HW_USBPHY_PWD_SET);

		pr_debug ("%s, Polling UTMI enter suspend \n", __func__);
		while (tmp & BM_USBPHY_CTRL_UTMI_SUSPENDM)
			tmp = __raw_readl(phy_reg + HW_USBPHY_CTRL);
	} else {
		tmp = (BM_USBPHY_PWD_TXPWDFS
			| BM_USBPHY_PWD_TXPWDIBIAS
			| BM_USBPHY_PWD_TXPWDV2I
			| BM_USBPHY_PWD_RXPWDENV
			| BM_USBPHY_PWD_RXPWD1PT1
			| BM_USBPHY_PWD_RXPWDDIFF
			| BM_USBPHY_PWD_RXPWDRX);
		__raw_writel(tmp, phy_reg + HW_USBPHY_PWD_CLR);

		tmp = __raw_readl(usb_reg + UOG_PORTSC1);
		tmp &= ~PORTSC_PHCD;
		__raw_writel(tmp, usb_reg + UOG_PORTSC1);
	}
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
	u32 tmp;
	void __iomem *phy_reg = IO_ADDRESS(pdata->phy_regs);

	pr_debug("%s, enable is %d\n", __func__, enable);
	if (enable) {
		tmp = BM_USBPHY_CTRL_RESUME_IRQ | BM_USBPHY_CTRL_WAKEUP_IRQ;
		__raw_writel(tmp, phy_reg + HW_USBPHY_CTRL_CLR);

		__raw_writel(BM_USBPHY_CTRL_ENIRQWAKEUP, phy_reg + HW_USBPHY_CTRL_SET);
	} else {
		tmp = BM_USBPHY_CTRL_RESUME_IRQ | BM_USBPHY_CTRL_WAKEUP_IRQ;
		__raw_writel(tmp, phy_reg + HW_USBPHY_CTRL_CLR);

		__raw_writel(BM_USBPHY_CTRL_ENIRQWAKEUP, phy_reg + HW_USBPHY_CTRL_CLR);
		/* The interrupt must be disabled for at least 3
		* cycles of the standby clock(32k Hz) , that is 0.094 ms*/
		udelay(100);
	}
}

static u32 wakeup_irq_enable_src; /* only useful at otg mode */
static void __wakeup_irq_enable(struct fsl_usb2_platform_data *pdata, bool on, int source)
 {
	/* otg host and device share the OWIE bit, only when host and device
	 * all enable the wakeup irq, we can enable the OWIE bit
	 */
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
}

/* The wakeup operation for DR port, it will clear the wakeup irq status
 * and re-enable the wakeup
 */
static void usbotg_wakeup_event_clear(void)
{
	void __iomem *phy_reg = IO_ADDRESS(USBPHY0_PHYS_ADDR);
	u32 wakeup_irq_bits;

	wakeup_irq_bits = BM_USBPHY_CTRL_RESUME_IRQ | BM_USBPHY_CTRL_WAKEUP_IRQ;
	if (__raw_readl(phy_reg + HW_USBPHY_CTRL) && wakeup_irq_bits) {
		/* clear the wakeup interrupt status */
		__raw_writel(wakeup_irq_bits, phy_reg + HW_USBPHY_CTRL_CLR);
	}
}

/* End of Common operation for DR port */


#ifdef CONFIG_USB_EHCI_ARC_OTG
extern void fsl_usb_recover_hcd(struct platform_device *pdev);
/* Beginning of host related operation for DR port */
static void _host_phy_lowpower_suspend(struct fsl_usb2_platform_data *pdata, bool enable)
{
	__phy_lowpower_suspend(pdata, enable, ENABLED_BY_HOST);
}

static void _host_wakeup_enable(struct fsl_usb2_platform_data *pdata, bool enable)
{
	void __iomem *phy_reg = IO_ADDRESS(pdata->phy_regs);
	u32 tmp;

	__wakeup_irq_enable(pdata, enable, ENABLED_BY_HOST);
	tmp = BM_USBPHY_CTRL_ENIDCHG_WKUP | BM_USBPHY_CTRL_ENDPDMCHG_WKUP;
	/* host only care the ID change wakeup event */
	if (enable) {
		pr_debug("DR: host wakeup enable\n");
		__raw_writel(tmp, phy_reg + HW_USBPHY_CTRL_SET);
	} else {
		pr_debug("DR: host wakeup disable\n");
		__raw_writel(tmp, phy_reg + HW_USBPHY_CTRL_CLR);
		/* The interrupt must be disabled for at least 3 clock
		 * cycles of the standby clock(32k Hz) , that is 0.094 ms*/
		udelay(100);
	}
}

static enum usb_wakeup_event _is_host_wakeup(struct fsl_usb2_platform_data *pdata)
{
	void __iomem *phy_reg = IO_ADDRESS(pdata->phy_regs);
	void __iomem *usb_reg = pdata->regs;
	u32 wakeup_irq_bits, wakeup_req, otgsc;

	pr_debug("%s\n", __func__);
	wakeup_irq_bits = BM_USBPHY_CTRL_RESUME_IRQ | BM_USBPHY_CTRL_WAKEUP_IRQ;
	otgsc = __raw_readl(usb_reg + UOG_OTGSC);

	if (__raw_readl(phy_reg + HW_USBPHY_CTRL) && wakeup_irq_bits)
		wakeup_req = 1;

	/* if ID change sts, it is a host wakeup event */
	if (wakeup_req && (otgsc & OTGSC_IS_USB_ID)) {
		pr_debug("otg host ID wakeup\n");
		/* if host ID wakeup, we must clear the b session change sts */
		__raw_writel(wakeup_irq_bits, phy_reg + HW_USBPHY_CTRL_CLR);
		__raw_writel(otgsc & (~OTGSC_IS_USB_ID), usb_reg + UOG_OTGSC);
		return WAKEUP_EVENT_ID;
	}
	if (wakeup_req  && (!(otgsc & OTGSC_STS_USB_ID))) {
		__raw_writel(wakeup_irq_bits, phy_reg + HW_USBPHY_CTRL_CLR);
		pr_debug("otg host Remote wakeup\n");
		return WAKEUP_EVENT_DPDM;
	}
	return WAKEUP_EVENT_INVALID;
}

static void host_wakeup_handler(struct fsl_usb2_platform_data *pdata)
{
	_host_wakeup_enable(pdata, false);
	_host_phy_lowpower_suspend(pdata, false);
	fsl_usb_recover_hcd(otg_host_pdev);
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
	void __iomem *phy_reg = IO_ADDRESS(pdata->phy_regs);
	u32 tmp;

	tmp = BM_USBPHY_CTRL_ENVBUSCHG_WKUP | BM_USBPHY_CTRL_ENDPDMCHG_WKUP;
	__wakeup_irq_enable(pdata, enable, ENABLED_BY_DEVICE);
	/* if udc is not used by any gadget, we can not enable the vbus wakeup */
	if (!pdata->port_enables) {
		__raw_writel(BM_USBPHY_CTRL_ENVBUSCHG_WKUP, phy_reg + HW_USBPHY_CTRL_CLR);
		return;
	}
	if (enable) {
		pr_debug("device wakeup enable\n");
		__raw_writel(tmp, phy_reg + HW_USBPHY_CTRL_SET);
	} else {
		pr_debug("device wakeup disable\n");
		__raw_writel(tmp, phy_reg + HW_USBPHY_CTRL_CLR);
	}
}

static enum usb_wakeup_event _is_device_wakeup(struct fsl_usb2_platform_data *pdata)
{
	void __iomem *phy_reg = IO_ADDRESS(pdata->phy_regs);
	void __iomem *usb_reg = pdata->regs;
	u32 wakeup_irq_bits, wakeup_req, otgsc;

	pr_debug("%s\n", __func__);
	wakeup_irq_bits = BM_USBPHY_CTRL_RESUME_IRQ | BM_USBPHY_CTRL_WAKEUP_IRQ;
	otgsc = __raw_readl(usb_reg + UOG_OTGSC);
	if (__raw_readl(phy_reg + HW_USBPHY_CTRL) && wakeup_irq_bits) {
		wakeup_req = 1;
	}

	/* if ID change sts, it is a host wakeup event */
	if (wakeup_req && (otgsc & OTGSC_STS_USB_ID) && (otgsc & OTGSC_IS_B_SESSION_VALID)) {
		pr_debug("otg device wakeup\n");
		/* if host ID wakeup, we must clear the b session change sts */
		__raw_writel(wakeup_irq_bits, phy_reg + HW_USBPHY_CTRL_CLR);
		return WAKEUP_EVENT_VBUS;
	}

	return WAKEUP_EVENT_INVALID;
}

static void device_wakeup_handler(struct fsl_usb2_platform_data *pdata)
{
	_device_wakeup_enable(pdata, false);
	_device_phy_lowpower_suspend(pdata, false);
}

/* end of device related operation for DR port */
#endif /* CONFIG_USB_GADGET_ARC */

/*
 * platform data structs
 * 	- Which one to use is determined by CONFIG options in usb.h
 * 	- operating_mode plugged at run time
 */
static struct fsl_usb2_platform_data __maybe_unused dr_utmi_config = {
	.name              = "DR",
	.platform_init     = usbotg_init_ext,
	.platform_uninit   = usbotg_uninit_ext,
	.usb_clock_for_pm  = usbotg_clock_gate,
	.phy_mode          = FSL_USB2_PHY_UTMI_WIDE,
	.power_budget      = 500,	/* 500 mA max power */
	.platform_resume = usb_host_phy_resume,
	.transceiver       = "utmi",
	.phy_regs          = USBPHY0_PHYS_ADDR,
};

/*
 * OTG resources
 */
static struct resource otg_resources[] = {
	[0] = {
		.start	= (u32)USBCTRL0_PHYS_ADDR,
		.end	= (u32)(USBCTRL0_PHYS_ADDR + 0x1ff),
		.flags	= IORESOURCE_MEM,
	},

	[1] = {
		.start	= IRQ_USB0,
		.flags	= IORESOURCE_IRQ,
	},
};

/*
 * UDC resources (same as OTG resource)
 */
static struct resource udc_resources[] = {
	[0] = {
		.start	= (u32)USBCTRL0_PHYS_ADDR,
		.end	= (u32)(USBCTRL0_PHYS_ADDR + 0x1ff),
		.flags	= IORESOURCE_MEM,
	},

	[1] = {
		.start	= IRQ_USB0,
		.flags	= IORESOURCE_IRQ,
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
	.resource      = udc_resources,
	.num_resources = ARRAY_SIZE(udc_resources),
};

static struct resource usbotg_wakeup_resources[] = {
	{
		.start = IRQ_USB0_WAKEUP, /*wakeup irq*/
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = IRQ_USB0, /* usb core irq */
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxs_usbotg_wakeup_device = {
	.name = "usb_wakeup",
	.id   = 1,
	.num_resources = ARRAY_SIZE(usbotg_wakeup_resources),
	.resource = usbotg_wakeup_resources,
};

static struct fsl_usb2_wakeup_platform_data usbdr_wakeup_config = {
	.name = "DR wakeup",
	.usb_clock_for_pm  = usbotg_clock_gate,
	.usb_wakeup_exhandle = usbotg_wakeup_event_clear,
};

static int __init usb_dr_init(void)
{
	struct platform_device *pdev;

	pr_debug("%s: \n", __func__);
	dr_utmi_config.change_ahb_burst = 1;
	dr_utmi_config.ahb_burst_mode = 0;

#ifdef CONFIG_USB_OTG
	dr_utmi_config.operating_mode = FSL_USB2_DR_OTG;
	/* wake_up_enalbe is useless, just for usb_register_remote_wakeup execution*/
	dr_utmi_config.wake_up_enable = _device_wakeup_enable;
	dr_utmi_config.irq_delay = 0;
	dr_utmi_config.wakeup_pdata = &usbdr_wakeup_config;

	if (platform_device_register(&dr_otg_device))
		printk(KERN_ERR "usb DR: can't register otg device\n");
	else {
		platform_device_add_data(&dr_otg_device, &dr_utmi_config, sizeof(dr_utmi_config));
		usbdr_wakeup_config.usb_pdata[0] = dr_otg_device.dev.platform_data;
	}
#endif

#ifdef CONFIG_USB_EHCI_ARC_OTG
	dr_utmi_config.operating_mode = DR_HOST_MODE;
	dr_utmi_config.wake_up_enable = _host_wakeup_enable;
	dr_utmi_config.phy_lowpower_suspend = _host_phy_lowpower_suspend;
	dr_utmi_config.wakeup_handler = host_wakeup_handler;
	dr_utmi_config.is_wakeup_event = _is_host_wakeup;
	dr_utmi_config.irq_delay = 0;
	dr_utmi_config.wakeup_pdata = &usbdr_wakeup_config;
	pdev = host_pdev_register(otg_resources,
			ARRAY_SIZE(otg_resources), &dr_utmi_config);
	if (pdev) {
		usbdr_wakeup_config.usb_pdata[1] = pdev->dev.platform_data;
		otg_host_pdev = pdev;
	} else
		printk(KERN_ERR "usb DR: can't alloc platform device for host\n");
#endif

#ifdef CONFIG_USB_GADGET_ARC
	dr_utmi_config.operating_mode = DR_UDC_MODE;
	dr_utmi_config.wake_up_enable = _device_wakeup_enable;
	dr_utmi_config.phy_lowpower_suspend = _device_phy_lowpower_suspend;
	dr_utmi_config.is_wakeup_event = _is_device_wakeup;
	dr_utmi_config.wakeup_handler = device_wakeup_handler;
	dr_utmi_config.irq_delay = 0;
	dr_utmi_config.wakeup_pdata = &usbdr_wakeup_config;

	if (platform_device_register(&dr_udc_device))
		printk(KERN_ERR "usb DR: can't register udc device\n");
	else {
		platform_device_add_data(&dr_udc_device, &dr_utmi_config, sizeof(dr_utmi_config));
		usbdr_wakeup_config.usb_pdata[2] = dr_udc_device.dev.platform_data;
	}
#endif

	mxs_usbotg_wakeup_device.dev.platform_data = &usbdr_wakeup_config;

	if (platform_device_register(&mxs_usbotg_wakeup_device))
		printk(KERN_ERR "usb DR wakeup device\n");
	else
		printk(KERN_INFO "usb DR wakeup device is registered\n");

	return 0;
}
#ifdef CONFIG_MXS_VBUS_CURRENT_DRAW
	fs_initcall(usb_dr_init);
#else
	subsys_initcall(usb_dr_init);
#endif
