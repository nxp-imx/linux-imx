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
#include <mach/hardware.h>
#include <linux/io.h>
#define MXC_NUMBER_USB_TRANSCEIVER 6
struct fsl_xcvr_ops *g_xc_ops[MXC_NUMBER_USB_TRANSCEIVER] = { NULL };
#ifdef CONFIG_WORKAROUND_ARCUSB_REG_RW
static void fsl_safe_writel(u32 val32, volatile u32 *addr)
{
	__asm__ ("swp %0, %0, [%1]" : : "r"(val32), "r"(addr));
}
#endif

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

void fsl_platform_set_test_mode(
				struct fsl_usb2_platform_data *pdata,
				enum usb_test_mode mode)
{
}
EXPORT_SYMBOL(fsl_platform_set_test_mode);

/* enable/disable high-speed disconnect detector of phy ctrl */
void fsl_platform_set_usb_phy_dis(struct fsl_usb2_platform_data *pdata,
				  bool enable)
{
	if (enable)
		__raw_writel(BM_USBPHY_CTRL_ENHOSTDISCONDETECT,
			IO_ADDRESS(pdata->phy_regs) + HW_USBPHY_CTRL_SET);
	else
		__raw_writel(BM_USBPHY_CTRL_ENHOSTDISCONDETECT,
			IO_ADDRESS(pdata->phy_regs) + HW_USBPHY_CTRL_CLR);
}
EXPORT_SYMBOL(fsl_platform_set_usb_phy_dis);


#if defined(CONFIG_USB_OTG)
static struct resource *otg_resources;

struct resource *otg_get_resources(void)
{
	pr_debug("otg_get_resources\n");
	return otg_resources;
}
EXPORT_SYMBOL(otg_get_resources);

int otg_set_resources(struct resource *resources)
{
	otg_resources = resources;
	return 0;
}
EXPORT_SYMBOL(otg_set_resources);
#endif

/*!
 * Register remote wakeup by this usb controller
 *
 * @param pdev: platform_device for this usb controller
 *
 * @return 0 or negative error code in case not supportted.
 */
static int usb_register_remote_wakeup(struct platform_device *pdev)
{
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;
	struct resource *res;
	int irq;

	pr_debug("%s: pdev=0x%p \n", __func__, pdev);
	if (!(pdata->wake_up_enable))
		return -ECANCELED;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev,
		"Found HC with no IRQ. Check %s setup!\n",
		dev_name(&pdev->dev));
		return -ENODEV;
	}
	irq = res->start;
	pdev->dev.power.can_wakeup = 1;
	enable_irq_wake(irq);

	return 0;
}

static struct fsl_xcvr_ops *fsl_usb_get_xcvr(char *name)
{
	int i;

	pr_debug("%s\n", __func__);
	if (name == NULL) {
		printk(KERN_ERR "get_xcvr(): No tranceiver name\n");
		return NULL;
	}

	for (i = 0; i < MXC_NUMBER_USB_TRANSCEIVER; i++) {
		if (strcmp(g_xc_ops[i]->name, name) == 0)
			return g_xc_ops[i];
	}
	pr_debug("Failed %s\n", __func__);
	return NULL;
}

/* The dmamask must be set for EHCI to work */
static u64 ehci_dmamask = ~(u32) 0;

static int instance_id = ~(u32) 0;
struct platform_device *host_pdev_register(struct resource *res, int n_res,
						struct fsl_usb2_platform_data
						*config)
{
	struct platform_device *pdev;
	int rc;

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

int usb_phy_enable(struct fsl_usb2_platform_data *pdata)
{
	u32 tmp;
	void __iomem *phy_reg = IO_ADDRESS(pdata->phy_regs);
	void __iomem *usb_reg = pdata->regs;
	void __iomem *usbcmd, *phy_ctrl, *portsc;

	/* Reset USB IP */
	usbcmd = usb_reg + UOG_USBCMD;
	tmp = __raw_readl(usbcmd); /* usb command */
	tmp &= ~UCMD_RUN_STOP;
	fsl_safe_writel(tmp, usbcmd);
	while (__raw_readl(usbcmd) & UCMD_RUN_STOP)
		;

	tmp |= UCMD_RESET;
	fsl_safe_writel(tmp, usbcmd);
	while (__raw_readl(usbcmd) & UCMD_RESET)
		;
	mdelay(10);

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

	/* set UTMI xcvr */
	/* Workaround an IC issue for ehci driver:
	 * when turn off root hub port power, EHCI set
	 * PORTSC reserved bits to be 0, but PTW with 0
	 * means 8 bits tranceiver width, here change
	 * it back to be 16 bits and do PHY diable and
	 * then enable.
	 */
	portsc = usb_reg + UOG_PORTSC1;
	tmp = __raw_readl(portsc);
	tmp &=  ~PORTSC_PTS_MASK;
	tmp |= (PORTSC_PTS_UTMI | PORTSC_PTW);
	fsl_safe_writel(tmp, portsc);

	/* Power up the PHY */
	__raw_writel(0, phy_reg + HW_USBPHY_PWD);

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

	if (!otg_used) {
		pr_debug("%s: grab pins\n", __func__);
		if (xops->init)
			xops->init(xops);
		usb_phy_enable(pdata);
	}

	if ((pdata->operating_mode == FSL_USB2_DR_HOST) ||
			(pdata->operating_mode == FSL_USB2_DR_OTG)) {
		/* enable FS/LS device */
		__raw_writel(BM_USBPHY_CTRL_ENUTMILEVEL2 | BM_USBPHY_CTRL_ENUTMILEVEL3
				, IO_ADDRESS(pdata->phy_regs) + HW_USBPHY_CTRL_SET);
	}

	if (usb_register_remote_wakeup(pdev))
		pr_debug("%s port is not a wakeup source.\n", pdata->name);

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

int fsl_usb_host_init(struct platform_device *pdev)
{
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;
	struct fsl_xcvr_ops *xops;
	u32 tmp;
	void __iomem *phy_reg = IO_ADDRESS(pdata->phy_regs);

	pr_debug("%s: pdev=0x%p  pdata=0x%p\n", __func__, pdev, pdata);

	xops = fsl_usb_get_xcvr(pdata->transceiver);
	if (!xops) {
		printk(KERN_ERR "%s transceiver ops missing\n", pdata->name);
		return -EINVAL;
	}
	pdata->xcvr_ops = xops;
	pdata->xcvr_type = xops->xcvr_type;
	pdata->pdev = pdev;

	if (xops->init)
		xops->init(xops);
	usb_phy_enable(pdata);
	/* enable FS/LS device */
	tmp = __raw_readl(phy_reg + HW_USBPHY_CTRL);
	tmp |= (BM_USBPHY_CTRL_ENUTMILEVEL2 | BM_USBPHY_CTRL_ENUTMILEVEL3);
	__raw_writel(tmp, phy_reg + HW_USBPHY_CTRL);

	if (usb_register_remote_wakeup(pdev))
		pr_debug("%s port is not a wakeup source.\n", pdata->name);

	pr_debug("%s: %s success\n", __func__, pdata->name);
	return 0;
}
EXPORT_SYMBOL(fsl_usb_host_init);

void fsl_usb_host_uninit(struct fsl_usb2_platform_data *pdata)
{
	pr_debug("%s\n", __func__);

	if (pdata->xcvr_ops && pdata->xcvr_ops->uninit)
		pdata->xcvr_ops->uninit(pdata->xcvr_ops);

	pdata->regs = NULL;
}
EXPORT_SYMBOL(fsl_usb_host_uninit);

void usb_debounce_id_pin(void)
{

	/* Because the IC design needs to remove the glitch on ID so the otgsc bit 8 will
	 * be delayed max 2 ms to show the real ID pin value
	 */
	mdelay(3);
}
EXPORT_SYMBOL(usb_debounce_id_pin);
int usb_host_wakeup_irq(struct device *wkup_dev)
{
	return 0;
}
EXPORT_SYMBOL(usb_host_wakeup_irq);

void usb_host_set_wakeup(struct device *wkup_dev, bool para)
{
	struct fsl_usb2_platform_data *pdata = wkup_dev->platform_data;
	if (pdata->wake_up_enable)
		pdata->wake_up_enable(pdata, para);
}
EXPORT_SYMBOL(usb_host_set_wakeup);

#ifdef CONFIG_ARCH_MX28
#define USBPHY_PHYS_ADDR USBPHY0_PHYS_ADDR
#endif

int usb_event_is_otg_wakeup(void)
{
	u32 wakeup_irq_bits;
	wakeup_irq_bits = BM_USBPHY_CTRL_RESUME_IRQ | BM_USBPHY_CTRL_WAKEUP_IRQ;

	if (__raw_readl(IO_ADDRESS(USBPHY_PHYS_ADDR) + HW_USBPHY_STATUS) && wakeup_irq_bits) {
		return true;
	}
	return false;
}
EXPORT_SYMBOL(usb_event_is_otg_wakeup);

int fsl_is_usb_plugged(void)
{
	return __raw_readl(IO_ADDRESS(USBPHY_PHYS_ADDR) + HW_USBPHY_STATUS) & \
		BM_USBPHY_STATUS_DEVPLUGIN_STATUS;
}
EXPORT_SYMBOL(fsl_is_usb_plugged);

void fsl_enable_usb_plugindetect(void)
{
	__raw_writel(BM_USBPHY_CTRL_ENDEVPLUGINDETECT,
			IO_ADDRESS(USBPHY_PHYS_ADDR) + HW_USBPHY_CTRL_SET);
}
EXPORT_SYMBOL(fsl_enable_usb_plugindetect);

