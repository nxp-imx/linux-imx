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
static int (*orig_bus_resume)(struct usb_hcd *hcd);
static int (*orig_hub_control)(struct usb_hcd *hcd,
				u16 typeReq, u16 wValue, u16 wIndex,
				char *buf, u16 wLength);

/* This function is used to override WKCN, WKDN, and WKOC */
static void ci_ehci_override_wakeup_flag(struct ehci_hcd *ehci,
		u32 __iomem *reg, u32 flags, bool set)
{
	u32 val = ehci_readl(ehci, reg);

	if (set)
		val |= flags;
	else
		val &= ~flags;

	ehci_writel(ehci, val, reg);
}

static int ci_ehci_bus_suspend(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int port;
	u32 tmp;
	struct device *dev = hcd->self.controller;
	struct ci_hdrc *ci = dev_get_drvdata(dev);

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
			if (hcd->phy && test_bit(port, &ehci->bus_suspended)
				&& (ehci_port_speed(ehci, portsc) ==
					USB_PORT_STAT_HIGH_SPEED))
				/*
				 * notify the USB PHY, it is for global
				 * suspend case.
				 */
				usb_phy_notify_suspend(hcd->phy,
					USB_SPEED_HIGH);
		}
		if (ci->platdata->flags & CI_HDRC_IMX_IS_HSIC)
			ci_ehci_override_wakeup_flag(ehci, reg,
				PORT_WKDISC_E | PORT_WKCONN_E, false);
	}

	return 0;
}

static int ci_imx_ehci_bus_resume(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int port;

	int ret = orig_bus_resume(hcd);

	if (ret)
		return ret;

	port = HCS_N_PORTS(ehci->hcs_params);
	while (port--) {
		u32 __iomem *reg = &ehci->regs->port_status[port];
		u32 portsc = ehci_readl(ehci, reg);
		/*
		 * Notify PHY after resume signal has finished, it is
		 * for global suspend case.
		 */
		if (hcd->phy
			&& test_bit(port, &ehci->bus_suspended)
			&& (portsc & PORT_CONNECT)
			&& (ehci_port_speed(ehci, portsc) ==
				USB_PORT_STAT_HIGH_SPEED))
			/* notify the USB PHY */
			usb_phy_notify_resume(hcd->phy, USB_SPEED_HIGH);
	}

	return 0;
}

/* The below code is based on tegra ehci driver */
static int ci_imx_ehci_hub_control(
	struct usb_hcd	*hcd,
	u16		typeReq,
	u16		wValue,
	u16		wIndex,
	char		*buf,
	u16		wLength
)
{
	struct ehci_hcd	*ehci = hcd_to_ehci(hcd);
	u32 __iomem	*status_reg;
	u32		temp;
	unsigned long	flags;
	int		retval = 0;

	status_reg = &ehci->regs->port_status[(wIndex & 0xff) - 1];

	spin_lock_irqsave(&ehci->lock, flags);

	if (typeReq == SetPortFeature && wValue == USB_PORT_FEAT_SUSPEND) {
		struct device *dev = hcd->self.controller;
		struct ci_hdrc *ci = dev_get_drvdata(dev);
		temp = ehci_readl(ehci, status_reg);
		if ((temp & PORT_PE) == 0 || (temp & PORT_RESET) != 0) {
			retval = -EPIPE;
			goto done;
		}

		temp &= ~(PORT_RWC_BITS | PORT_WKCONN_E);
		temp |= PORT_WKDISC_E | PORT_WKOC_E;
		ehci_writel(ehci, temp | PORT_SUSPEND, status_reg);

		if (ci->platdata->flags & CI_HDRC_IMX_IS_HSIC) {
			if (ci->platdata->notify_event)
				ci->platdata->notify_event
					(ci, CI_HDRC_IMX_HSIC_SUSPEND_EVENT);
			ci_ehci_override_wakeup_flag(ehci, status_reg,
				PORT_WKDISC_E | PORT_WKCONN_E, false);
		}
		/*
		 * If a transaction is in progress, there may be a delay in
		 * suspending the port. Poll until the port is suspended.
		 */
		if (ehci_handshake(ehci, status_reg, PORT_SUSPEND,
						PORT_SUSPEND, 5000))
			ehci_err(ehci, "timeout waiting for SUSPEND\n");

		spin_unlock_irqrestore(&ehci->lock, flags);
		if (ehci_port_speed(ehci, temp) ==
				USB_PORT_STAT_HIGH_SPEED && hcd->phy) {
			/* notify the USB PHY */
			usb_phy_notify_suspend(hcd->phy, USB_SPEED_HIGH);
		}
		spin_lock_irqsave(&ehci->lock, flags);

		set_bit((wIndex & 0xff) - 1, &ehci->suspended_ports);
		goto done;
	}

	/*
	 * After resume has finished, it needs do some post resume
	 * operation for some SoCs.
	 */
	else if (typeReq == ClearPortFeature &&
					wValue == USB_PORT_FEAT_C_SUSPEND) {

		/* Make sure the resume has finished, it should be finished */
		if (ehci_handshake(ehci, status_reg, PORT_RESUME, 0, 25000))
			ehci_err(ehci, "timeout waiting for resume\n");

		temp = ehci_readl(ehci, status_reg);

		if (ehci_port_speed(ehci, temp) ==
				USB_PORT_STAT_HIGH_SPEED && hcd->phy) {
			/* notify the USB PHY */
			usb_phy_notify_resume(hcd->phy, USB_SPEED_HIGH);
		}
	}

	spin_unlock_irqrestore(&ehci->lock, flags);

	/* Handle the hub control events here */
	return orig_hub_control(hcd, typeReq, wValue, wIndex, buf, wLength);
done:
	spin_unlock_irqrestore(&ehci->lock, flags);
	return retval;
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

	if (ci->platdata->notify_event &&
		(ci->platdata->flags & CI_HDRC_IMX_IS_HSIC))
		ci->platdata->notify_event
			(ci, CI_HDRC_IMX_HSIC_ACTIVE_EVENT);

	if (ci->platdata->flags & CI_HDRC_DISABLE_STREAMING)
		hw_write(ci, OP_USBMODE, USBMODE_CI_SDIS, USBMODE_CI_SDIS);

	return ret;

disable_reg:
	if (ci->platdata->reg_vbus)
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
	orig_bus_resume = ci_ehci_hc_driver.bus_resume;
	orig_hub_control = ci_ehci_hc_driver.hub_control;

	ci_ehci_hc_driver.bus_suspend = ci_ehci_bus_suspend;
	if (ci->platdata->flags & CI_HDRC_IMX_EHCI_QUIRK) {
		ci_ehci_hc_driver.bus_resume = ci_imx_ehci_bus_resume;
		ci_ehci_hc_driver.hub_control = ci_imx_ehci_hub_control;
	}

	return 0;
}
