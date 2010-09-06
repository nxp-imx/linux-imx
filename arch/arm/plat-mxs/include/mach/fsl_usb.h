/*
 * Copyright (C) 1999 ARM Limited
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc.
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

/*
 * USB Host side, platform-specific functionality.
 */

#include <linux/usb/fsl_xcvr.h>
#include <mach/arc_otg.h>

/* ehci_arc_hc_driver.flags value */
#define FSL_PLATFORM_HC_FLAGS (HCD_USB2 | HCD_MEMORY)

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
	temp = readl(hcd->regs + UOG_USBMODE);
	writel(temp | USBMODE_CM_HOST, hcd->regs + UOG_USBMODE);
}

/* Needed for i2c/serial transceivers */
static inline void
fsl_platform_set_vbus_power(struct fsl_usb2_platform_data *pdata, int on)
{
	if (pdata->xcvr_ops && pdata->xcvr_ops->set_vbus_power)
		pdata->xcvr_ops->set_vbus_power(pdata->xcvr_ops, pdata, on);
}

/* Set USB AHB burst length for host */
static inline void fsl_platform_set_ahb_burst(struct usb_hcd *hcd)
{
}

void fsl_phy_usb_utmi_init(struct fsl_xcvr_ops *this);
void fsl_phy_usb_utmi_uninit(struct fsl_xcvr_ops *this);
void fsl_phy_set_power(struct fsl_xcvr_ops *this,
			struct fsl_usb2_platform_data *pdata, int on);

