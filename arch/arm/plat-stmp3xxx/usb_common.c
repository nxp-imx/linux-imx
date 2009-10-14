/*
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *	otg_{get,set}_transceiver() are from arm/plat-omap/usb.c.
 *	which is Copyright (C) 2004 Texas Instruments, Inc.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 *@defgroup USB ARC OTG USB Driver
 */

/*!
 * @file usb_common.c
 *
 * @brief platform related part of usb driver.
 * @ingroup USB
 */

/*!
 *Include files
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/usb/otg.h>
#include <linux/usb/fsl_xcvr.h>
#include <mach/arc_otg.h>
#include <mach/platform.h>
#include <mach/regs-power.h>
#include <mach/regs-digctl.h>
#include <mach/regs-clkctrl.h>
#include <mach/regs-usbctrl.h>
#include <mach/regs-usbphy.h>
#include <mach/pinmux.h>

#define MXC_NUMBER_USB_TRANSCEIVER 6
struct fsl_xcvr_ops *g_xc_ops[MXC_NUMBER_USB_TRANSCEIVER] = { NULL };

void fsl_usb_xcvr_register(struct fsl_xcvr_ops *xcvr_ops)
{
	int i;

	pr_debug("%s\n", __func__);
	for (i = 0; i < MXC_NUMBER_USB_TRANSCEIVER; i++) {
		if (g_xc_ops[i] == NULL) {
			g_xc_ops[i] = xcvr_ops;
			return;
		}
	}

	pr_debug("Failed %s\n", __func__);
}
EXPORT_SYMBOL(fsl_usb_xcvr_register);

void fsl_usb_xcvr_unregister(struct fsl_xcvr_ops *xcvr_ops)
{
	int i;

	pr_debug("%s\n", __func__);
	for (i = 0; i < MXC_NUMBER_USB_TRANSCEIVER; i++) {
		if (g_xc_ops[i] == xcvr_ops) {
			g_xc_ops[i] = NULL;
			return;
		}
	}

	pr_debug("Failed %s\n", __func__);
}
EXPORT_SYMBOL(fsl_usb_xcvr_unregister);

#if defined(CONFIG_USB_OTG)
static struct otg_transceiver *xceiv;

/**
 * otg_get_transceiver - find the (single) OTG transceiver driver
 *
 * Returns the transceiver driver, after getting a refcount to it; or
 * null if there is no such transceiver.  The caller is responsible for
 * releasing that count.
 */
struct otg_transceiver *otg_get_transceiver(void)
{
	pr_debug("%s xceiv=0x%p\n", __func__, xceiv);
	if (xceiv)
		get_device(xceiv->dev);
	return xceiv;
}
EXPORT_SYMBOL(otg_get_transceiver);

int otg_set_transceiver(struct otg_transceiver *x)
{
	pr_debug("%s xceiv=0x%p  x=0x%p\n", __func__, xceiv, x);
	/*
	if (x == NULL)
		stmp3xxx_release_pin_group(&usb_mux_pins, "usb");
		*/
	if (xceiv && x)
		return -EBUSY;
	xceiv = x;
	return 0;
}
EXPORT_SYMBOL(otg_set_transceiver);

static struct resource *otg_resources;

struct resource *otg_get_resources(void)
{
	pr_debug("otg_get_resources\n");
	return otg_resources;
}
EXPORT_SYMBOL(otg_get_resources);

int otg_set_resources(struct resource *resources)
{
	//stmp3xxx_request_pin_group(&usb_mux_pins, "usb");
	otg_resources = resources;
	return 0;
}
EXPORT_SYMBOL(otg_set_resources);
#endif

static struct fsl_xcvr_ops *fsl_usb_get_xcvr(char *name)
{
	int i;

	pr_debug("%s\n", __func__);
	if (name == NULL) {
		printk(KERN_ERR "get_xcvr(): No tranceiver name\n");
		return NULL;
	}

	for (i = 0; i < MXC_NUMBER_USB_TRANSCEIVER; i++) {
		if (strcmp(g_xc_ops[i]->name, name) == 0) {
			return g_xc_ops[i];
		}
	}
	pr_debug("Failed %s\n", __func__);
	return NULL;
}

/* The dmamask must be set for EHCI to work */
static u64 ehci_dmamask = ~(u32) 0;

static int instance_id;
struct platform_device *host_pdev_register(struct resource *res, int n_res,
					   struct fsl_usb2_platform_data *config)
{
	struct platform_device *pdev;
	int rc;
	instance_id = 0;

	pr_debug("register host res=0x%p, size=%d\n", res, n_res);

	pdev = platform_device_register_simple("fsl-ehci",
					       instance_id, res, n_res);
	if (IS_ERR(pdev)) {
		pr_debug("can't register %s Host, %ld\n",
			 config->name, PTR_ERR(pdev));
		return NULL;
	}

	pdev->dev.coherent_dma_mask = 0xffffffff;
	pdev->dev.dma_mask = &ehci_dmamask;

	/*
	 * platform_device_add_data() makes a copy of
	 * the platform_data passed in.  That makes it
	 * impossible to share the same config struct for
	 * all OTG devices (host,gadget,otg).  So, just
	 * set the platorm_data pointer ourselves.
	 */
	rc = platform_device_add_data(pdev, config,
				      sizeof(struct fsl_usb2_platform_data));
	if (rc) {
		platform_device_unregister(pdev);
		return NULL;
	}

	pr_debug(KERN_INFO "usb: %s host (%s) registered\n", config->name,
	       config->transceiver);
	pr_debug("pdev=0x%p  dev=0x%p  resources=0x%p  pdata=0x%p\n",
		 pdev, &pdev->dev, pdev->resource, pdev->dev.platform_data);

	instance_id++;

	return pdev;
}

int usb_phy_enable(void)
{
	u32 tmp;
	/*
	* Set these bits so that we can force the OTG bits high
	* so the ARC core operates properly
	*/
	stmp3xxx_clearl(BM_POWER_CTRL_CLKGATE,
		      REGS_POWER_BASE + HW_POWER_CTRL);
	stmp3xxx_setl(BM_POWER_DEBUG_VBUSVALIDPIOLOCK |
			 BM_POWER_DEBUG_AVALIDPIOLOCK |
			 BM_POWER_DEBUG_BVALIDPIOLOCK,
			 REGS_POWER_BASE + HW_POWER_DEBUG);
	tmp = __raw_readl(REGS_POWER_BASE + HW_POWER_STS);
	tmp |= BM_POWER_STS_BVALID | BM_POWER_STS_AVALID |
		       BM_POWER_STS_VBUSVALID;
	__raw_writel(tmp, REGS_POWER_BASE + HW_POWER_STS);

	/* Reset USBPHY module */
	stmp3xxx_setl(BM_USBPHY_CTRL_SFTRST,
		      REGS_USBPHY_BASE + HW_USBPHY_CTRL);
	udelay(10);

	/* Remove CLKGATE and SFTRST */
	stmp3xxx_clearl(BM_USBPHY_CTRL_CLKGATE | BM_USBPHY_CTRL_SFTRST,
		      REGS_USBPHY_BASE + HW_USBPHY_CTRL);

	/* Turn on the USB clocks */
	stmp3xxx_setl(BM_CLKCTRL_PLLCTRL0_EN_USB_CLKS,
		     REGS_CLKCTRL_BASE + HW_CLKCTRL_PLLCTRL0);
	stmp3xxx_clearl(BM_DIGCTL_CTRL_USB_CLKGATE,
		      REGS_DIGCTL_BASE + HW_DIGCTL_CTRL);

	/* set UTMI xcvr */
	/* Workaround an IC issue for ehci driver:
	 * when turn off root hub port power, EHCI set
	 * PORTSC reserved bits to be 0, but PTW with 0
	 * means 8 bits tranceiver width, here change
	 * it back to be 16 bits and do PHY diable and
	 * then enable.
	 */
	tmp = __raw_readl(REGS_USBCTRL_BASE + HW_USBCTRL_PORTSC1) & ~PORTSC_PTS_MASK;
	tmp |= (PORTSC_PTS_UTMI | PORTSC_PTW);
	__raw_writel(tmp, REGS_USBCTRL_BASE + HW_USBCTRL_PORTSC1);

	/* Power up the PHY */
	__raw_writel(0, REGS_USBPHY_BASE + HW_USBPHY_PWD);

	/*
	* Set precharge bit to cure overshoot problems at the
	* start of packets
	*/
	stmp3xxx_setl(1, REGS_USBPHY_BASE + HW_USBPHY_CTRL);

#if defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_EHCI_HCD_MODULE)
	/* enable disconnect detector */
	/* enable disconnect detector must be after entry high speed mode*/
	/*HW_USBPHY_CTRL_SET(BM_USBPHY_CTRL_ENHOSTDISCONDETECT);
	*/
#endif
	return 0;
}
EXPORT_SYMBOL(usb_phy_enable);

static int otg_used;

int usbotg_init(struct platform_device *pdev)
{
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;
	struct fsl_xcvr_ops *xops;

	pr_debug("%s: pdev=0x%p  pdata=0x%p\n", __func__, pdev, pdata);

	xops = fsl_usb_get_xcvr(pdata->transceiver);
	if (!xops) {
		printk(KERN_ERR "DR transceiver ops missing\n");
		return -EINVAL;
	}
	pdata->xcvr_ops = xops;
	pdata->xcvr_type = xops->xcvr_type;
	pdata->pdev = pdev;

	otg_used = 0;
	if (!otg_used) {
		pr_debug("%s: grab pins\n", __func__);
		if (xops->init)
			xops->init(xops);
		usb_phy_enable();
	}

	otg_used++;
	pr_debug("%s: success\n", __func__);
	return 0;
}
EXPORT_SYMBOL(usbotg_init);

void usbotg_uninit(struct fsl_usb2_platform_data *pdata)
{
	pr_debug("%s\n", __func__);

	if (pdata->xcvr_ops && pdata->xcvr_ops->uninit)
		pdata->xcvr_ops->uninit(pdata->xcvr_ops);

	pdata->regs = NULL;
	otg_used--;
}
EXPORT_SYMBOL(usbotg_uninit);
