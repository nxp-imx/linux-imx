/*
 * Copyright 2005-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
	temp = readl(hcd->regs + 0x1a8);
	writel(temp | USBMODE_CM_HOST, hcd->regs + 0x1a8);
}

/* Needed for i2c/serial transceivers */
static inline void
fsl_platform_set_vbus_power(struct fsl_usb2_platform_data *pdata, int on)
{
}

/* Set USB AHB burst length for host */
static inline void fsl_platform_set_ahb_burst(struct usb_hcd *hcd)
{
}
