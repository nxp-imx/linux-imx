/*
 * host.c - ChipIdea USB host controller driver
 *
 * Copyright (c) 2012 Intel Corporation
 *
 * Author: Alexander Shishkin
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/usb/chipidea.h>
#include <linux/regulator/consumer.h>

#include "../host/ehci.h"

#include "ci.h"
#include "bits.h"
#include "host.h"

static struct hc_driver __read_mostly ci_ehci_hc_driver;
static int (*orig_bus_suspend)(struct usb_hcd *hcd);

static int ci_ehci_bus_suspend(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int port;
	u32 tmp;

	int ret = orig_bus_suspend(hcd);

	if (ret)
		return ret;

	port = HCS_N_PORTS(ehci->hcs_params);
	while (port--) {
		u32 __iomem *reg = &ehci->regs->port_status[port];
		u32 portsc = ehci_readl(ehci, reg);

		if (portsc & PORT_CONNECT) {
			/*
			 * For chipidea, the resume signal will be ended
			 * automatically, so for remote wakeup case, the
			 * usbcmd.rs may not be set before the resume has
			 * ended if other resume path consumes too much
			 * time (~23ms-24ms), in that case, the SOF will not
			 * send out within 3ms after resume ends, then the
			 * device will enter suspend again.
			 */
			if (hcd->self.root_hub->do_remote_wakeup) {
				ehci_dbg(ehci,
					"Remote wakeup is enabled, "
					"and device is on the port\n");

				tmp = ehci_readl(ehci, &ehci->regs->command);
				tmp |= CMD_RUN;
				ehci_writel(ehci, tmp, &ehci->regs->command);
				/*
				 * It needs a short delay between set RUNSTOP
				 * and set PHCD.
				 */
				udelay(125);
			}
		}
	}

	return 0;
}

static irqreturn_t host_irq(struct ci_hdrc *ci)
{
	return usb_hcd_irq(ci->irq, ci->hcd);
}

static int host_start(struct ci_hdrc *ci)
{
	struct usb_hcd *hcd;
	struct ehci_hcd *ehci;
	int ret;

	if (usb_disabled())
		return -ENODEV;

	hcd = usb_create_hcd(&ci_ehci_hc_driver, ci->dev, dev_name(ci->dev));
	if (!hcd)
		return -ENOMEM;

	dev_set_drvdata(ci->dev, ci);
	hcd->rsrc_start = ci->hw_bank.phys;
	hcd->rsrc_len = ci->hw_bank.size;
	hcd->regs = ci->hw_bank.abs;
	hcd->has_tt = 1;

	hcd->power_budget = ci->platdata->power_budget;
	hcd->phy = ci->transceiver;

	ehci = hcd_to_ehci(hcd);
	ehci->caps = ci->hw_bank.cap;
	ehci->has_hostpc = ci->hw_bank.lpm;
	ehci->imx28_write_fix = ci->imx28_write_fix;

	if (ci->platdata->reg_vbus) {
		ret = regulator_enable(ci->platdata->reg_vbus);
		if (ret) {
			dev_err(ci->dev,
				"Failed to enable vbus regulator, ret=%d\n",
				ret);
			goto put_hcd;
		}
	}

	ret = usb_add_hcd(hcd, 0, 0);
	if (ret)
		goto disable_reg;
	else
		ci->hcd = hcd;

	if (ci->platdata->flags & CI_HDRC_DISABLE_STREAMING)
		hw_write(ci, OP_USBMODE, USBMODE_CI_SDIS, USBMODE_CI_SDIS);

	return ret;

disable_reg:
	regulator_disable(ci->platdata->reg_vbus);

put_hcd:
	usb_put_hcd(hcd);

	return ret;
}

static void host_stop(struct ci_hdrc *ci)
{
	struct usb_hcd *hcd = ci->hcd;

	if (hcd) {
		usb_remove_hcd(hcd);
		usb_put_hcd(hcd);
		if (ci->platdata->reg_vbus)
			regulator_disable(ci->platdata->reg_vbus);
	}
}


void ci_hdrc_host_destroy(struct ci_hdrc *ci)
{
	if (ci->role == CI_ROLE_HOST && ci->hcd)
		host_stop(ci);
}

int ci_hdrc_host_init(struct ci_hdrc *ci)
{
	struct ci_role_driver *rdrv;

	if (!hw_read(ci, CAP_DCCPARAMS, DCCPARAMS_HC))
		return -ENXIO;

	rdrv = devm_kzalloc(ci->dev, sizeof(struct ci_role_driver), GFP_KERNEL);
	if (!rdrv)
		return -ENOMEM;

	rdrv->start	= host_start;
	rdrv->stop	= host_stop;
	rdrv->irq	= host_irq;
	rdrv->name	= "host";
	ci->roles[CI_ROLE_HOST] = rdrv;

	ehci_init_driver(&ci_ehci_hc_driver, NULL);

	orig_bus_suspend = ci_ehci_hc_driver.bus_suspend;

	ci_ehci_hc_driver.bus_suspend = ci_ehci_bus_suspend;

	return 0;
}
