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
#include <linux/regulator/consumer.h>
#include <linux/io.h>
#include <asm/mach-types.h>
#include <mach/arc_otg.h>
#include <mach/hardware.h>
#include <mach/mxc.h>
typedef void (*driver_vbus_func)(bool);

void __iomem *imx_otg_base;
static  driver_vbus_func s_h1_driver_vbus;
static  driver_vbus_func s_otg_driver_vbus;

EXPORT_SYMBOL(imx_otg_base);

#define MXC_NUMBER_USB_TRANSCEIVER 6
struct fsl_xcvr_ops *g_xc_ops[MXC_NUMBER_USB_TRANSCEIVER] = { NULL };

bool usb_icbug_swfix_need(void)
{
	if (cpu_is_mx6sl())
		return false;
	else if ((mx6q_revision() > IMX_CHIP_REVISION_1_1))
		return false;
	else if ((mx6dl_revision() > IMX_CHIP_REVISION_1_0))
		return false;
	return true;
}
EXPORT_SYMBOL(usb_icbug_swfix_need);

void mx6_set_host1_vbus_func(driver_vbus_func driver_vbus)
{
	s_h1_driver_vbus = driver_vbus;
}

void mx6_get_host1_vbus_func(driver_vbus_func *driver_vbus)
{
	*driver_vbus = s_h1_driver_vbus;
}
EXPORT_SYMBOL(mx6_get_host1_vbus_func);

void mx6_set_otghost_vbus_func(driver_vbus_func driver_vbus)
{
	s_otg_driver_vbus = driver_vbus;
}

void mx6_get_otghost_vbus_func(driver_vbus_func *driver_vbus)
{
	*driver_vbus = s_otg_driver_vbus;
}
EXPORT_SYMBOL(mx6_get_otghost_vbus_func);



enum fsl_usb2_modes get_usb_mode(struct fsl_usb2_platform_data *pdata)
{
	enum fsl_usb2_modes mode;
	mode = FSL_USB_UNKNOWN;

	if (!strcmp("DR", pdata->name)) {
		if ((UOG_USBMODE & 0x3) == 0x2)
			mode = FSL_USB_DR_DEVICE;
		else if ((UOG_USBMODE & 0x3) == 0x3)
			mode = FSL_USB_DR_HOST;
	} else if (!strcmp("Host 1", pdata->name))
		mode = FSL_USB_MPH_HOST1;
	else if (!strcmp("Host 2", pdata->name))
		mode = FSL_USB_MPH_HOST2;

	if (mode == FSL_USB_UNKNOWN)
		printk(KERN_ERR "unknow usb mode,name is %s\n", pdata->name);
	return mode;
}

static struct clk *usb_clk;
static struct clk *usb_ahb_clk;


/*
 * make sure USB_CLK is running at 60 MHz +/- 1000 Hz
 */
static int fsl_check_usbclk(void)
{
	unsigned long freq;

	usb_ahb_clk = clk_get(NULL, "usb_ahb_clk");
	if (clk_enable(usb_ahb_clk)) {
		if (cpu_is_mx6())
			return 0; /* there is no ahb clock at mx6 */
		printk(KERN_ERR "clk_enable(usb_ahb_clk) failed\n");
		return -EINVAL;
	}
	clk_put(usb_ahb_clk);

	usb_clk = clk_get(NULL, "usb_clk");
	if (clk_enable(usb_clk)) {
		if (cpu_is_mx6())
			return 0; /* there is usb_clk at mx6 */
		printk(KERN_ERR "clk_enable(usb_clk) failed\n");
		return -EINVAL;
	}
	freq = clk_get_rate(usb_clk);
	clk_put(usb_clk);
	if ((freq < 59999000) || (freq > 60001000)) {
		printk(KERN_ERR "USB_CLK=%lu, should be 60MHz\n", freq);
		return -1;
	}

	return 0;
}

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

void fsl_platform_set_test_mode (struct fsl_usb2_platform_data *pdata, enum usb_test_mode mode)
{
	if (pdata->xcvr_ops && pdata->xcvr_ops->set_test_mode)
		pdata->xcvr_ops->set_test_mode((u32 *)(pdata->regs + ULPIVW_OFF), mode);
}
EXPORT_SYMBOL(fsl_platform_set_test_mode);

void fsl_usb_xcvr_unregister(struct fsl_xcvr_ops *xcvr_ops)
{
	int i;

	pr_debug("%s\n", __func__);
	for (i = 0; i < MXC_NUMBER_USB_TRANSCEIVER; i++) {
		if (g_xc_ops[i] && (g_xc_ops[i] == xcvr_ops)) {
			g_xc_ops[i] = NULL;
			return;
		}
	}

	pr_debug("Failed %s\n", __func__);
}
EXPORT_SYMBOL(fsl_usb_xcvr_unregister);

static struct fsl_xcvr_ops *fsl_usb_get_xcvr(char *name)
{
	int i;

	pr_debug("%s\n", __func__);
	if (name == NULL) {
		printk(KERN_ERR "get_xcvr(): No tranceiver name\n");
		return NULL;
	}

	for (i = 0; i < MXC_NUMBER_USB_TRANSCEIVER; i++) {
		if (g_xc_ops[i] && (strcmp(g_xc_ops[i]->name, name) == 0)) {
			return g_xc_ops[i];
		}
	}
	pr_debug("Failed %s\n", __func__);
	return NULL;
}

/* The dmamask must be set for EHCI to work */
static u64 ehci_dmamask = ~(u32) 0;

/*!
 * Register an instance of a USB host platform device.
 *
 * @param	res:	resource pointer
 * @param       n_res:	number of resources
 * @param       config: config pointer
 *
 * @return      newly-registered platform_device
 *
 * The USB controller supports 3 host interfaces, and the
 * kernel can be configured to support some number of them.
 * Each supported host interface is registered as an instance
 * of the "fsl-ehci" device.  Call this function multiple times
 * to register each host interface.
 */
static int usb_mxc_instance_id;
__init struct platform_device *host_pdev_register(struct resource *res, int n_res,
					   struct fsl_usb2_platform_data *config)
{
	struct platform_device *pdev;
	int rc;

	pr_debug("register host res=0x%p, size=%d\n", res, n_res);

	pdev = platform_device_register_simple("fsl-ehci",
			 usb_mxc_instance_id, res, n_res);
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

	printk(KERN_INFO "usb: %s host (%s) registered\n", config->name,
	       config->transceiver);
	pr_debug("pdev=0x%p  dev=0x%p  resources=0x%p  pdata=0x%p\n",
		 pdev, &pdev->dev, pdev->resource, pdev->dev.platform_data);

	usb_mxc_instance_id++;

	return pdev;
}

static void usbh1_set_serial_xcvr(void)
{
	pr_debug("%s: \n", __func__);
	USBCTRL &= ~(UCTRL_H1SIC_MASK | UCTRL_BPE); /* disable bypass mode */
	USBCTRL |= UCTRL_H1SIC_SU6 |		/* single-ended / unidir. */
		   UCTRL_H1WIE | UCTRL_H1DT |	/* disable H1 TLL */
		   UCTRL_H1PM;			/* power mask */
}

static void usbh1_set_ulpi_xcvr(void)
{
	pr_debug("%s: \n", __func__);

	/* Stop then Reset */
	UH1_USBCMD &= ~UCMD_RUN_STOP;
	while (UH1_USBCMD & UCMD_RUN_STOP)
		;

	UH1_USBCMD |= UCMD_RESET;
	while (UH1_USBCMD & UCMD_RESET)
		;

	/* Select the clock from external PHY */
	USB_CTRL_1 |= USB_CTRL_UH1_EXT_CLK_EN;

	/* select ULPI PHY PTS=2 */
	UH1_PORTSC1 = (UH1_PORTSC1 & ~PORTSC_PTS_MASK) | PORTSC_PTS_ULPI;

	USBCTRL &= ~UCTRL_H1WIE; /* HOST1 wakeup intr disable */
	USBCTRL &= ~UCTRL_H1UIE; /* Host1 ULPI interrupt disable */
	USBCTRL |= UCTRL_H1PM; /* HOST1 power mask */
	USB_PHY_CTR_FUNC |= USB_UH1_OC_DIS; /* OC is not used */

	/* Interrupt Threshold Control:Immediate (no threshold) */
	UH1_USBCMD &= UCMD_ITC_NO_THRESHOLD;

	UH1_USBCMD |= UCMD_RESET;       /* reset the controller */

	/* allow controller to reset, and leave time for
	* the ULPI transceiver to reset too.
	*/
	msleep(100);

	/* Turn off the usbpll for ulpi tranceivers */
	clk_disable(usb_clk);
}

static void usbh1_set_utmi_xcvr(void)
{
	u32 tmp;

	/* Stop then Reset */
	UH1_USBCMD &= ~UCMD_RUN_STOP;
	while (UH1_USBCMD & UCMD_RUN_STOP)
		;

	UH1_USBCMD |= UCMD_RESET;
	while ((UH1_USBCMD) & (UCMD_RESET))
		;

	/* For OC and PWR, it is board level setting
	 * The default setting is for mx53 evk
	 */
	USBCTRL &= ~UCTRL_H1PM;	/* Host1 Power Mask */
	USBCTRL &= ~UCTRL_H1WIE; /* Host1 Wakeup Intr Disable */
	USB_PHY_CTR_FUNC |= USB_UH1_OC_DIS; /* Over current disable */

	if (cpu_is_mx50()) {
		USBCTRL |= UCTRL_H1PM; /* Host1 Power Mask */
		USB_PHY_CTR_FUNC &= ~USB_UH1_OC_DIS; /* Over current enable */
		/* Over current polarity low active */
		USB_PHY_CTR_FUNC |= USB_UH1_OC_POL;
	}
	/* set UTMI xcvr */
	tmp = UH1_PORTSC1 & ~PORTSC_PTS_MASK;
	tmp |= PORTSC_PTS_UTMI;
	UH1_PORTSC1 = tmp;

	/* Set the PHY clock to 24MHz */
	USBH1_PHY_CTRL1 &= ~USB_UTMI_PHYCTRL2_PLLDIV_MASK;
	USBH1_PHY_CTRL1 |= 0x01;

	/* Workaround an IC issue for ehci driver:
	 * when turn off root hub port power, EHCI set
	 * PORTSC reserved bits to be 0, but PTW with 0
	 * means 8 bits tranceiver width, here change
	 * it back to be 16 bits and do PHY diable and
	 * then enable.
	 */
	UH1_PORTSC1 |= PORTSC_PTW;

	/* need to reset the controller here so that the ID pin
	 * is correctly detected.
	 */
	/* Stop then Reset */
	UH1_USBCMD &= ~UCMD_RUN_STOP;
	while (UH1_USBCMD & UCMD_RUN_STOP)
		;

	UH1_USBCMD |= UCMD_RESET;
	while ((UH1_USBCMD) & (UCMD_RESET))
		;

	/* allow controller to reset, and leave time for
	 * the ULPI transceiver to reset too.
	 */
	msleep(100);

	/* Turn off the usbpll for UTMI tranceivers */
	clk_disable(usb_clk);
}

static void usbh2_set_ulpi_xcvr(void)
{
	u32 tmp;

	pr_debug("%s\n", __func__);

	UH2_USBCMD &= ~UCMD_RUN_STOP;
	while (UH2_USBCMD & UCMD_RUN_STOP)
		;

	UH2_USBCMD |= UCMD_RESET;
	while (UH2_USBCMD & UCMD_RESET)

	USBCTRL_HOST2 &= ~(UCTRL_H2SIC_MASK | UCTRL_BPE);
	USBCTRL_HOST2 &= ~UCTRL_H2WIE;	/* wakeup intr enable */
	USBCTRL_HOST2 &= ~UCTRL_H2UIE;	/* ULPI intr enable */
	USB_CTRL_1 |= USB_CTRL_UH2_EXT_CLK_EN;
	if (cpu_is_mx53())
		USB_CTRL_1 |= USB_CTRL_UH2_CLK_FROM_ULPI_PHY;
	if (cpu_is_mx51())/* not tested */
		USBCTRL_HOST2 |= (1 << 12);
	/* must set ULPI phy before turning off clock */
	tmp = UH2_PORTSC1 & ~PORTSC_PTS_MASK;
	tmp |= PORTSC_PTS_ULPI;
	UH2_PORTSC1 = tmp;
	if (cpu_is_mx53()) {
		/* turn off the internal 60MHZ clk  */
		USB_CLKONOFF_CTRL |= (1 << 21);
	}
	UH2_USBCMD |= UCMD_RESET;	/* reset the controller */

	/* allow controller to reset, and leave time for
	 * the ULPI transceiver to reset too.
	 */
	msleep(100);

	/* Turn off the usbpll for ulpi tranceivers */
	clk_disable(usb_clk);
}

static void usbh2_set_serial_xcvr(void)
{
	pr_debug("%s: \n", __func__);

	/* Stop then Reset */
	UH2_USBCMD &= ~UCMD_RUN_STOP;
	while (UH2_USBCMD & UCMD_RUN_STOP)
		;

	UH2_USBCMD |= UCMD_RESET;
	while (UH2_USBCMD & UCMD_RESET)
		;

	USBCTRL &= ~(UCTRL_H2SIC_MASK);	/* Disable bypass mode */
	USBCTRL &= ~(UCTRL_H2PM);	/* Power Mask */
	USBCTRL &= ~UCTRL_H2OCPOL;	/* OverCurrent Polarity is Low Active */
	USBCTRL &= ~UCTRL_H2WIE;	/* Wakeup intr disable */
	USBCTRL |= UCTRL_IP_PUE_DOWN |	/* ipp_pue_pulldwn_dpdm */
	    UCTRL_USBTE |	/* USBT is enabled */
	    UCTRL_H2DT;		/* Disable H2 TLL */

	if (cpu_is_mx25()) {
		/*
		 * USBH2_PWR and USBH2_OC are active high.
		 * Must force xcvr clock to "internal" so that
		 * we can write to PTS field after it's been
		 * cleared by ehci_turn_off_all_ports().
		 */
		USBCTRL |= UCTRL_H2PP | UCTRL_H2OCPOL | UCTRL_XCSH2;
		/* Disable Host2 bus Lock */
		USBCTRL |= UCTRL_H2LOCKD;
	}

	USBCTRL &= ~(UCTRL_PP);
	UH2_PORTSC1 = (UH2_PORTSC1 & (~PORTSC_PTS_MASK)) | PORTSC_PTS_SERIAL;

	if (UH2_HCSPARAMS & HCSPARAMS_PPC)
		UH2_PORTSC1 |= PORTSC_PORT_POWER;

	/* Reset controller before set host mode */
	UH2_USBCMD |= UCMD_RESET;
	while (UH2_USBCMD & UCMD_RESET)
		;

	msleep(100);
}

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
	device_set_wakeup_capable(&pdev->dev, true);
	enable_irq_wake(irq);

	return 0;
}

int fsl_usb_host_init(struct platform_device *pdev)
{
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;
	struct fsl_xcvr_ops *xops;

	pr_debug("%s: pdev=0x%p  pdata=0x%p\n", __func__, pdev, pdata);

	xops = fsl_usb_get_xcvr(pdata->transceiver);
	if (!xops) {
		printk(KERN_ERR "%s transceiver ops missing\n", pdata->name);
		return -EINVAL;
	}
	pdata->xcvr_ops = xops;
	pdata->xcvr_type = xops->xcvr_type;
	pdata->pdev = pdev;

	if (fsl_check_usbclk() != 0)
		return -EINVAL;

	pr_debug("%s: grab pins\n", __func__);
	if (pdata->gpio_usb_active && pdata->gpio_usb_active())
		return -EINVAL;

	if (cpu_is_mx50())
		/* Turn on AHB CLK for H1*/
		USB_CLKONOFF_CTRL &= ~H1_AHBCLK_OFF;

	/* enable board power supply for xcvr */
	if (pdata->xcvr_pwr) {
		if (pdata->xcvr_pwr->regu1)
			regulator_enable(pdata->xcvr_pwr->regu1);
		if (pdata->xcvr_pwr->regu2)
			regulator_enable(pdata->xcvr_pwr->regu2);
	}

	if (xops->init)
		xops->init(xops);

	if (usb_register_remote_wakeup(pdev))
		pr_debug("%s port is not a wakeup source.\n", pdata->name);
	if (!(cpu_is_mx6())) {
		if (xops->xcvr_type == PORTSC_PTS_SERIAL) {
			if (cpu_is_mx35()) {
				usbh2_set_serial_xcvr();
				/* Close the internal 60Mhz */
				USBCTRL &= ~UCTRL_XCSH2;
			} else if (cpu_is_mx25())
				usbh2_set_serial_xcvr();
			else
				usbh1_set_serial_xcvr();
		} else if (xops->xcvr_type == PORTSC_PTS_ULPI) {
			if (!strcmp("Host 1", pdata->name))
				usbh1_set_ulpi_xcvr();
			if (!strcmp("Host 2", pdata->name))
				usbh2_set_ulpi_xcvr();
		} else if (xops->xcvr_type == PORTSC_PTS_UTMI) {
			usbh1_set_utmi_xcvr();
		}
	} else {
#ifdef CONFIG_ARCH_MX6
		if (!strcmp("Host 1", pdata->name)) {
			if (machine_is_mx6q_arm2())
				USB_H1_CTRL &= ~UCTRL_OVER_CUR_POL;
			else if (machine_is_mx6q_sabrelite())
				USB_H1_CTRL |= UCTRL_OVER_CUR_POL;
			USB_H1_CTRL |= UCTRL_OVER_CUR_DIS;
		}
#endif
	}

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

	if (pdata->gpio_usb_inactive)
		pdata->gpio_usb_inactive();
	if (pdata->xcvr_type == PORTSC_PTS_SERIAL) {
		/* Workaround an IC issue for ehci driver.
		 * when turn off root hub port power, EHCI set
		 * PORTSC reserved bits to be 0, but PTS with 0
		 * means UTMI interface, so here force the Host2
		 * port use the internal 60Mhz.
		 */
		if (cpu_is_mx35())
			USBCTRL |= UCTRL_XCSH2;
		clk_disable(usb_clk);
	}

	/* disable board power supply for xcvr */
	if (pdata->xcvr_pwr) {
		if (pdata->xcvr_pwr->regu1)
			regulator_disable(pdata->xcvr_pwr->regu1);
		if (pdata->xcvr_pwr->regu2)
			regulator_disable(pdata->xcvr_pwr->regu2);
	}

	clk_disable(usb_ahb_clk);
}
EXPORT_SYMBOL(fsl_usb_host_uninit);

static void otg_set_serial_xcvr(void)
{
	pr_debug("%s\n", __func__);
}

void otg_set_serial_host(void)
{
	pr_debug("%s\n", __func__);
	/* set USBCTRL for host operation
	 * disable: bypass mode,
	 * set: single-ended/unidir/6 wire, OTG wakeup intr enable,
	 *      power mask
	 */
	USBCTRL &= ~UCTRL_OSIC_MASK;
#if defined(CONFIG_ARCH_MX27) || defined(CONFIG_ARCH_MX3)
	USBCTRL &= ~UCTRL_BPE;
#endif

#if defined(CONFIG_MXC_USB_SB3)
	USBCTRL |= UCTRL_OSIC_SB3 | UCTRL_OWIE | UCTRL_OPM;
#elif defined(CONFIG_MXC_USB_SU6)
	USBCTRL |= UCTRL_OSIC_SU6 | UCTRL_OWIE | UCTRL_OPM;
#elif defined(CONFIG_MXC_USB_DB4)
	USBCTRL |= UCTRL_OSIC_DB4 | UCTRL_OWIE | UCTRL_OPM;
#else
	USBCTRL |= UCTRL_OSIC_DU6 | UCTRL_OWIE | UCTRL_OPM;
#endif

	USB_OTG_MIRROR = OTGM_VBUSVAL | OTGM_ASESVLD;	/* 0xa */
}
EXPORT_SYMBOL(otg_set_serial_host);

void otg_set_serial_peripheral(void)
{
	/* set USBCTRL for device operation
	 * disable: bypass mode
	 * set: differential/unidir/6 wire, OTG wakeup intr enable,
	 *      power mask
	 */
	USBCTRL &= ~UCTRL_OSIC_MASK;
#if defined(CONFIG_ARCH_MX27) || defined(CONFIG_ARCH_MX3)
	USBCTRL &= ~UCTRL_BPE;
#endif

#if defined(CONFIG_MXC_USB_SB3)
	USBCTRL |= UCTRL_OSIC_SB3 | UCTRL_OWIE | UCTRL_OPM;
#elif defined(CONFIG_MXC_USB_SU6)
	USBCTRL |= UCTRL_OSIC_SU6 | UCTRL_OWIE | UCTRL_OPM;
#elif defined(CONFIG_MXC_USB_DB4)
	USBCTRL |= UCTRL_OSIC_DB4 | UCTRL_OWIE | UCTRL_OPM;
#else
	USBCTRL |= UCTRL_OSIC_DU6 | UCTRL_OWIE | UCTRL_OPM;
#endif

	USB_OTG_MIRROR = OTGM_VBUSVAL | OTGM_BSESVLD | OTGM_IDIDG;	/* oxd */
}
EXPORT_SYMBOL(otg_set_serial_peripheral);

static void otg_set_ulpi_xcvr(void)
{
	u32 tmp;

	pr_debug("%s\n", __func__);
	USBCTRL &= ~UCTRL_OSIC_MASK;
#if defined(CONFIG_ARCH_MX27) || defined(CONFIG_ARCH_MX3)
	USBCTRL &= ~UCTRL_BPE;
#endif
	USBCTRL |= UCTRL_OUIE |	/* ULPI intr enable */
	    UCTRL_OWIE |	/* OTG wakeup intr enable */
	    UCTRL_OPM;		/* power mask */

	/* must set ULPI phy before turning off clock */
	tmp = UOG_PORTSC1 & ~PORTSC_PTS_MASK;
	tmp |= PORTSC_PTS_ULPI;
	UOG_PORTSC1 = tmp;

	/* need to reset the controller here so that the ID pin
	 * is correctly detected.
	 */
	UOG_USBCMD |= UCMD_RESET;

	/* allow controller to reset, and leave time for
	 * the ULPI transceiver to reset too.
	 */
	msleep(100);

	/* Turn off the usbpll for ulpi tranceivers */
	clk_disable(usb_clk);
}

int fsl_usb_xcvr_suspend(struct fsl_xcvr_ops *xcvr_ops)
{
	if (!machine_is_mx31_3ds())
		return -ECANCELED;

	if (xcvr_ops->xcvr_type == PORTSC_PTS_ULPI) {
		if (fsl_check_usbclk() != 0)
			return -EINVAL;
		clk_enable(usb_clk);

		otg_set_ulpi_xcvr();

		if (xcvr_ops->suspend)
			/* suspend transceiver */
			xcvr_ops->suspend(xcvr_ops);

		clk_disable(usb_clk);
	}
	return 0;
}
EXPORT_SYMBOL(fsl_usb_xcvr_suspend);

static void otg_set_utmi_xcvr(void)
{
	u32 tmp;

	/* Stop then Reset */
	UOG_USBCMD &= ~UCMD_RUN_STOP;
	while (UOG_USBCMD & UCMD_RUN_STOP)
		;

	UOG_USBCMD |= UCMD_RESET;
	while ((UOG_USBCMD) & (UCMD_RESET))
		;

	if (cpu_is_mx53())
		USB_PHY_CTR_FUNC |= USB_UTMI_PHYCTRL_OC_DIS;

	if (cpu_is_mx51()) {
		if (machine_is_mx51_3ds()) {
			/* OTG Polarity of Overcurrent is Low active */
			USB_PHY_CTR_FUNC |= USB_UTMI_PHYCTRL_OC_POL;
			/* Enable OTG Overcurrent Event */
			USB_PHY_CTR_FUNC &= ~USB_UTMI_PHYCTRL_OC_DIS;
		} else {
			/* BBG is not using OC */
			USB_PHY_CTR_FUNC |= USB_UTMI_PHYCTRL_OC_DIS;
		}
	} else if (cpu_is_mx25()) {
		USBCTRL |= UCTRL_OCPOL;
		USBCTRL &= ~UCTRL_PP;
	} else if (cpu_is_mx50()) {
		USB_PHY_CTR_FUNC |= USB_UTMI_PHYCTRL_OC_DIS;
	} else {
		/* USBOTG_PWR low active */
		USBCTRL &= ~UCTRL_PP;
		/* OverCurrent Polarity is Low Active */
		USBCTRL &= ~UCTRL_OCPOL;

		if (cpu_is_mx35() && (imx_cpu_ver() < IMX_CHIP_REVISION_2_0))
			/* OTG Lock Disable */
			USBCTRL |= UCTRL_OLOCKD;
	}

	if (cpu_is_mx51())
		USBCTRL &= ~UCTRL_OPM;	/* OTG Power Mask */

	USBCTRL &= ~UCTRL_OWIE;	/* OTG Wakeup Intr Disable */

	/* set UTMI xcvr */
	tmp = UOG_PORTSC1 & ~PORTSC_PTS_MASK;
	tmp |= PORTSC_PTS_UTMI;
	UOG_PORTSC1 = tmp;

	if (cpu_is_mx51()) {
		/* Set the PHY clock to 19.2MHz */
		USB_PHY_CTR_FUNC2 &= ~USB_UTMI_PHYCTRL2_PLLDIV_MASK;
		USB_PHY_CTR_FUNC2 |= 0x01;
	}

	/* Workaround an IC issue for ehci driver:
	 * when turn off root hub port power, EHCI set
	 * PORTSC reserved bits to be 0, but PTW with 0
	 * means 8 bits tranceiver width, here change
	 * it back to be 16 bits and do PHY diable and
	 * then enable.
	 */
	UOG_PORTSC1 |= PORTSC_PTW;

	if (cpu_is_mx35() || cpu_is_mx25()) {
		/* Enable UTMI interface in PHY control Reg */
		USB_PHY_CTR_FUNC &= ~USB_UTMI_PHYCTRL_UTMI_ENABLE;
		USB_PHY_CTR_FUNC |= USB_UTMI_PHYCTRL_UTMI_ENABLE;
	}

	/* need to reset the controller here so that the ID pin
	 * is correctly detected.
	 */
	/* Stop then Reset */
	UOG_USBCMD &= ~UCMD_RUN_STOP;
	while (UOG_USBCMD & UCMD_RUN_STOP)
		;

	UOG_USBCMD |= UCMD_RESET;
	while ((UOG_USBCMD) & (UCMD_RESET))
		;

	/* allow controller to reset, and leave time for
	 * the ULPI transceiver to reset too.
	 */
	msleep(100);

	if (cpu_is_mx37()) {
		/* fix USB PHY Power Gating leakage issue for i.MX37 */
		USB_PHY_CTR_FUNC &= ~USB_UTMI_PHYCTRL_CHGRDETON;
		USB_PHY_CTR_FUNC &= ~USB_UTMI_PHYCTRL_CHGRDETEN;
	}

	/* Turn off the usbpll for UTMI tranceivers */
	clk_disable(usb_clk);
}

static int mxc_otg_used;

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

	if (fsl_check_usbclk() != 0)
		return -EINVAL;
	if (!mxc_otg_used) {
		if (cpu_is_mx50())
			/* Turn on AHB CLK for OTG*/
			USB_CLKONOFF_CTRL &= ~OTG_AHBCLK_OFF;

		pr_debug("%s: grab pins\n", __func__);
		if (pdata->gpio_usb_active && pdata->gpio_usb_active())
			return -EINVAL;
		if (xops->init)
			xops->init(xops);
		if (!(cpu_is_mx6())) {
			UOG_PORTSC1 = UOG_PORTSC1 & ~PORTSC_PHCD;


			if (xops->xcvr_type == PORTSC_PTS_SERIAL) {
				if (pdata->operating_mode == FSL_USB2_DR_HOST) {
					otg_set_serial_host();
					/* need reset */
					UOG_USBCMD |= UCMD_RESET;
					msleep(100);
				} else if (pdata->operating_mode == FSL_USB2_DR_DEVICE)
					otg_set_serial_peripheral();
				otg_set_serial_xcvr();
			} else if (xops->xcvr_type == PORTSC_PTS_ULPI) {
				otg_set_ulpi_xcvr();
			} else if (xops->xcvr_type == PORTSC_PTS_UTMI) {
				otg_set_utmi_xcvr();
			}
		} else {
#ifdef CONFIG_ARCH_MX6
			if (machine_is_mx6q_arm2())
				USB_OTG_CTRL &= ~UCTRL_OVER_CUR_POL;
			else if (machine_is_mx6q_sabrelite())
				USB_OTG_CTRL |= UCTRL_OVER_CUR_POL;
			USB_OTG_CTRL |= UCTRL_OVER_CUR_DIS;
#endif
		}
	}

	if (usb_register_remote_wakeup(pdev))
		pr_debug("DR is not a wakeup source.\n");

	mxc_otg_used++;
	pr_debug("%s: success\n", __func__);
	return 0;
}
EXPORT_SYMBOL(usbotg_init);

void usbotg_uninit(struct fsl_usb2_platform_data *pdata)
{
	pr_debug("%s\n", __func__);

	mxc_otg_used--;
	if (!mxc_otg_used) {
		if (pdata->xcvr_ops && pdata->xcvr_ops->uninit)
			pdata->xcvr_ops->uninit(pdata->xcvr_ops);

		pdata->regs = NULL;

		if (machine_is_mx31_3ds()) {
			if (pdata->xcvr_ops && pdata->xcvr_ops->suspend)
				pdata->xcvr_ops->suspend(pdata->xcvr_ops);
			clk_disable(usb_clk);
		}
		msleep(1);
		UOG_PORTSC1 = UOG_PORTSC1 | PORTSC_PHCD;
		if (pdata->gpio_usb_inactive)
			pdata->gpio_usb_inactive();
		if (pdata->xcvr_type == PORTSC_PTS_SERIAL)
			clk_disable(usb_clk);
		clk_disable(usb_ahb_clk);
	}
}
EXPORT_SYMBOL(usbotg_uninit);

/*
 * This function is used to debounce the reading value for id/vbus at
 * the register of otgsc
 */
void usb_debounce_id_vbus(void)
{
	msleep(3);
}
EXPORT_SYMBOL(usb_debounce_id_vbus);

int usb_event_is_otg_wakeup(struct fsl_usb2_platform_data *pdata)
{
	return (USBCTRL & UCTRL_OWIR) ? true : false;
}
EXPORT_SYMBOL(usb_event_is_otg_wakeup);

#ifdef CONFIG_ARCH_MX6
/* enable/disable high-speed disconnect detector of phy ctrl */
void fsl_platform_set_usb_phy_dis(struct fsl_usb2_platform_data *pdata,
				  bool enable)
{
	u32 usb_phy_ctrl_dcdt = 0;
	/* for HSIC, we do not need to enable disconnect detection */
	if (pdata->phy_mode == FSL_USB2_PHY_HSIC)
		return;
	usb_phy_ctrl_dcdt = __raw_readl(
			MX6_IO_ADDRESS(pdata->phy_regs) + HW_USBPHY_CTRL) &
			BM_USBPHY_CTRL_ENHOSTDISCONDETECT;
	if (enable) {
		if (usb_phy_ctrl_dcdt == 0)
			__raw_writel(BM_USBPHY_CTRL_ENHOSTDISCONDETECT,
				MX6_IO_ADDRESS(pdata->phy_regs)
				+ HW_USBPHY_CTRL_SET);
	} else {
		if (usb_phy_ctrl_dcdt
				== BM_USBPHY_CTRL_ENHOSTDISCONDETECT)
			__raw_writel(BM_USBPHY_CTRL_ENHOSTDISCONDETECT,
				MX6_IO_ADDRESS(pdata->phy_regs)
				+ HW_USBPHY_CTRL_CLR);
	}
}
EXPORT_SYMBOL(fsl_platform_set_usb_phy_dis);
#endif

void usb_host_set_wakeup(struct device *wkup_dev, bool para)
{
	struct fsl_usb2_platform_data *pdata = wkup_dev->platform_data;
	if (pdata->wake_up_enable)
		pdata->wake_up_enable(pdata, para);
}
EXPORT_SYMBOL(usb_host_set_wakeup);
