/*
 * Copyright 2005-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 * USB Host side, platform-specific functionality.
 */

#include <linux/usb/fsl_xcvr.h>
#include <mach/arc_otg.h>

static void fsl_setup_phy(struct ehci_hcd *ehci,
			  enum fsl_usb2_phy_modes phy_mode,
			  int port_offset);

static inline void fsl_platform_usb_setup(struct ehci_hcd *ehci)
{
	struct fsl_usb2_platform_data *pdata;

	pdata = ehci_to_hcd(ehci)->self.controller->platform_data;
	fsl_setup_phy(ehci, pdata->phy_mode, 0);
}

static inline void fsl_platform_set_host_mode(struct usb_hcd *hcd)
{
	unsigned int temp;
	struct fsl_usb2_platform_data *pdata;

	pdata = hcd->self.controller->platform_data;

	if (pdata->xcvr_ops && pdata->xcvr_ops->set_host)
		pdata->xcvr_ops->set_host();

	/* set host mode */
	temp = readl(hcd->regs + 0x1a8);
	writel(temp | USBMODE_CM_HOST, hcd->regs + 0x1a8);
}

/* Needed for enable PP and i2c/serial transceivers */
static inline void
fsl_platform_set_vbus_power(struct fsl_usb2_platform_data *pdata, int on)
{
	u32 temp;

	/* HCSPARAMS */
	temp = readl(pdata->regs + 0x104);
	/* Port Power Control */
	if (temp & HCSPARAMS_PPC) {
		temp = readl(pdata->regs + FSL_SOC_USB_PORTSC1);
		if (on)
			temp |= PORT_POWER;
		else
			temp &= ~PORT_POWER;

		writel(temp, pdata->regs + FSL_SOC_USB_PORTSC1);
	}

	if (pdata->xcvr_ops && pdata->xcvr_ops->set_vbus_power)
		pdata->xcvr_ops->set_vbus_power(pdata->xcvr_ops, pdata, on);
}

/* Set USB AHB burst length for host */
static inline void fsl_platform_set_ahb_burst(struct usb_hcd *hcd)
{
	struct fsl_usb2_platform_data *pdata;
	unsigned int temp;

	pdata = hcd->self.controller->platform_data;
	if (pdata->change_ahb_burst) {
		temp = readl(hcd->regs + FSL_SOC_USB_SBUSCFG);
		writel((temp & (~(0x7))) | pdata->ahb_burst_mode,
			hcd->regs + FSL_SOC_USB_SBUSCFG);
	}

	/* Increase TX fifo threshold for USB+ATA for i.mx35 2.0 */
	if (cpu_is_mx35() && (imx_cpu_ver() >= IMX_CHIP_REVISION_2_0)) {
		temp = readl(hcd->regs + FSL_SOC_USB_TXFILLTUNING);
		/* Change TX FIFO threshold to be 0x20 */
		writel((temp & (~(0x3f << 16))) | (0x20 << 16),
			hcd->regs + FSL_SOC_USB_TXFILLTUNING);
	}

	/* Increase TX fifo threshold for USB+SD in Hostx */
	if (cpu_is_mx53() && (strcmp("DR", pdata->name))) {
		temp = readl(hcd->regs + FSL_SOC_USB_TXFILLTUNING);
		/* Change TX FIFO threshold to be 0x08 */
		writel((temp & (~(0x3f << 16))) | (0x08 << 16),
				hcd->regs + FSL_SOC_USB_TXFILLTUNING);
	}

	/* Increase TX fifo threshold for in Hostx */
	if (cpu_is_mx6()) {
		temp = readl(hcd->regs + FSL_SOC_USB_TXFILLTUNING);
		/* Change TX FIFO threshold to be 0x08 */
		writel((temp & (~(0x3f << 16))) | (0x08 << 16),
				hcd->regs + FSL_SOC_USB_TXFILLTUNING);
	}
}
