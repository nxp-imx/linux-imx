/*
 * Copyright (c) 2005 MontaVista Software
 * Copyright (C) 2013-2014 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Ported to 834x by Randy Vinson <rvinson@mvista.com> using code provided
 * by Hunter Wu.
 */

#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/usb/otg.h>
#include <linux/usb/hcd.h>

#include "../core/usb.h"
#include "ehci-fsl.h"
#include <mach/fsl_usb.h>
extern void usb_host_set_wakeup(struct device *wkup_dev, bool para);
static void fsl_usb_lowpower_mode(struct fsl_usb2_platform_data *pdata, bool enable)
{
	unsigned long flags;

	spin_lock_irqsave(&pdata->lock, flags);
	if (enable) {
		if (pdata->phy_lowpower_suspend)
			pdata->phy_lowpower_suspend(pdata, true);
	} else {
		if (pdata->phy_lowpower_suspend)
			pdata->phy_lowpower_suspend(pdata, false);
	}
	pdata->lowpower = enable;
	spin_unlock_irqrestore(&pdata->lock, flags);
}

static void fsl_usb_clk_gate(struct fsl_usb2_platform_data *pdata, bool enable)
{
	if (pdata->usb_clock_for_pm)
		pdata->usb_clock_for_pm(enable);
}
#undef EHCI_PROC_PTC
#ifdef EHCI_PROC_PTC		/* /proc PORTSC:PTC support */
/*
 * write a PORTSC:PTC value to /proc/driver/ehci-ptc
 * to put the controller into test mode.
 */
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#define EFPSL 3			/* ehci fsl proc string length */

static int ehci_fsl_proc_read(char *page, char **start, off_t off, int count,
			      int *eof, void *data)
{
	return 0;
}

static int ehci_fsl_proc_write(struct file *file, const char __user *buffer,
			       unsigned long count, void *data)
{
	int ptc;
	u32 portsc;
	struct ehci_hcd *ehci = (struct ehci_hcd *) data;
	char str[EFPSL] = {0};

	if (count > EFPSL-1)
		return -EINVAL;

	if (copy_from_user(str, buffer, count))
		return -EFAULT;

	str[count] = '\0';

	ptc = simple_strtoul(str, NULL, 0);

	portsc = ehci_readl(ehci, &ehci->regs->port_status[0]);
	portsc &= ~(0xf << 16);
	portsc |= (ptc << 16);
	printk(KERN_INFO "PTC %x  portsc %08x\n", ptc, portsc);

	ehci_writel(ehci, portsc, &ehci->regs->port_status[0]);

	return count;
}

static int ehci_testmode_init(struct ehci_hcd *ehci)
{
	struct proc_dir_entry *entry;

	entry = create_proc_read_entry("driver/ehci-ptc", 0644, NULL,
				       ehci_fsl_proc_read, ehci);
	if (!entry)
		return -ENODEV;

	entry->write_proc = ehci_fsl_proc_write;
	return 0;
}
#else
static int ehci_testmode_init(struct ehci_hcd *ehci)
{
	return 0;
}
#endif	/* /proc PORTSC:PTC support */

/**
 * The hcd operation need to be done during the wakeup irq
 */
void fsl_usb_recover_hcd(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	u32 cmd = 0;

	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	/* After receive remote wakeup signaling. Must restore
	 * CMDRUN bit in 20ms to keep port status.
	 */
	cmd = ehci_readl(ehci, &ehci->regs->command);
	if (!(cmd & CMD_RUN) || (hcd->state == HC_STATE_SUSPENDED)) {
		ehci_writel(ehci, ehci->command, &ehci->regs->command);
		/* Resume root hub here? */
		usb_hcd_resume_root_hub(hcd);
	}

	/* disable all interrupt, will re-enable in resume */
	ehci_writel(ehci, 0, &ehci->regs->intr_enable);
}

/**
 * This irq is used to open the hw access and let usb_hcd_irq process the usb event
 * ehci_fsl_pre_irq will be called before usb_hcd_irq
 * The hcd operation need to be done during the wakeup irq
 */
static irqreturn_t ehci_fsl_pre_irq(int irq, void *dev)
{
	struct platform_device *pdev = (struct platform_device *)dev;
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct fsl_usb2_platform_data *pdata;

	pdata = hcd->self.controller->platform_data;

	if (!test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags)) {
		if (pdata->irq_delay || (pdata->wakeup_event == WAKEUP_EVENT_VBUS)
				|| (pdata->wakeup_event == WAKEUP_EVENT_INVALID))
			return IRQ_NONE;

		pr_debug("%s\n", __func__);
		pdata->wakeup_event = WAKEUP_EVENT_INVALID;
		fsl_usb_recover_hcd(pdev);
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

/**
 * usb_hcd_fsl_probe - initialize FSL-based HCDs
 * @drvier: Driver to be used for this HCD
 * @pdev: USB Host Controller being probed
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller.
 *
 */
int usb_hcd_fsl_probe(const struct hc_driver *driver,
		      struct platform_device *pdev)
{
	struct fsl_usb2_platform_data *pdata;
	struct usb_hcd *hcd;
	struct resource *res;
	struct ehci_hcd *ehci;
	int irq;
	int retval;

	pr_debug("initializing FSL-SOC USB Controller\n");

	/* Need platform data for setup */
	pdata = (struct fsl_usb2_platform_data *)pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev,
			"No platform data for %s.\n", dev_name(&pdev->dev));
		return -ENODEV;
	}

	/*
	 * This is a host mode driver, verify that we're supposed to be
	 * in host mode.
	 */
	if (!((pdata->operating_mode == FSL_USB2_DR_HOST) ||
	      (pdata->operating_mode == FSL_USB2_MPH_HOST) ||
	      (pdata->operating_mode == FSL_USB2_DR_OTG))) {
		dev_err(&pdev->dev,
			"Non Host Mode configured for %s. Wrong driver linked.\n",
			dev_name(&pdev->dev));
		return -ENODEV;
	}

	hcd = usb_create_hcd(driver, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		retval = -ENOMEM;
		goto err1;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no IRQ. Check %s setup!\n",
			dev_name(&pdev->dev));
		return -ENODEV;
	}
	irq = res->start;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);

	if (pdata->operating_mode != FSL_USB2_DR_OTG) {
		if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len,
					driver->description)) {
			dev_dbg(&pdev->dev, "controller already in use\n");
			retval = -EBUSY;
			goto err2;
		}
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);

	if (hcd->regs == NULL) {
		dev_dbg(&pdev->dev, "error mapping memory\n");
		retval = -EFAULT;
		goto err3;
	}
	pdata->regs = hcd->regs;

	/*
	 * do platform specific init: check the clock, grab/config pins, etc.
	 */
	if (pdata->init && pdata->init(pdev)) {
		retval = -ENODEV;
		goto err4;
	}
	pdata->lowpower = false;

	spin_lock_init(&pdata->lock);

	fsl_platform_set_host_mode(hcd);
	hcd->power_budget = pdata->power_budget;
	ehci = hcd_to_ehci(hcd);
	if (pdata->operating_mode == FSL_USB2_DR_OTG) {

		dbg("pdev=0x%p  hcd=0x%p  ehci=0x%p\n", pdev, hcd, ehci);

		ehci->transceiver = otg_get_transceiver();
		dbg("ehci->transceiver=0x%p\n", ehci->transceiver);

		if (!ehci->transceiver) {
			printk(KERN_ERR "can't find transceiver\n");
			retval = -ENODEV;
			goto err4;
		}

		retval = otg_set_host(ehci->transceiver, &ehci_to_hcd(ehci)->self);
		if (retval)
			otg_put_transceiver(ehci->transceiver);
	} else if ((pdata->operating_mode == FSL_USB2_MPH_HOST) || \
			(pdata->operating_mode == FSL_USB2_DR_HOST))
		fsl_platform_set_vbus_power(pdata, 1);

	/*
	 * The ehci_fsl_pre_irq must be registered before usb_hcd_irq, in that case
	 * it can be called before usb_hcd_irq when irq occurs
	 */
	retval = request_irq(irq, ehci_fsl_pre_irq, IRQF_SHARED,
			"fsl ehci pre interrupt", (void *)pdev);
	if (retval != 0)
		goto err5;

	retval = usb_add_hcd(hcd, irq, IRQF_DISABLED | IRQF_SHARED);
	if (retval != 0)
		goto err6;

	fsl_platform_set_ahb_burst(hcd);
	ehci_testmode_init(hcd_to_ehci(hcd));
	/*
	 * Only for HSIC host controller, let HSCI controller
	 * connect with device, call it after EHCI initialization
	 * finishes.
	 */
	if (pdata->hsic_post_ops)
		pdata->hsic_post_ops();

	ehci = hcd_to_ehci(hcd);
	pdata->pm_command = ehci->command;
	return retval;
err6:
	free_irq(irq, (void *)pdev);
err5:
	otg_put_transceiver(ehci->transceiver);
err4:
	iounmap(hcd->regs);
err3:
	if (pdata->operating_mode != FSL_USB2_DR_OTG)
		release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err2:
	usb_put_hcd(hcd);
err1:
	dev_err(&pdev->dev, "init %s fail, %d\n", dev_name(&pdev->dev), retval);
	fsl_usb_lowpower_mode(pdata, true);
	if (pdata->usb_clock_for_pm)
		pdata->usb_clock_for_pm(false);
	if (pdata->exit && pdata->pdev)
		pdata->exit(pdata->pdev);
	return retval;
}

/**
 * usb_hcd_fsl_remove - shutdown processing for FSL-based HCDs
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_hcd_fsl_probe().
 *
 */
static void usb_hcd_fsl_remove(struct usb_hcd *hcd,
			       struct platform_device *pdev)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;

	if (!test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags)) {
		/* Need open clock for register access */
		if (pdata->usb_clock_for_pm)
			pdata->usb_clock_for_pm(true);

		/*disable the wakeup to avoid an abnormal wakeup interrupt*/
		usb_host_set_wakeup(hcd->self.controller, false);

		/* Put the PHY out of low power mode */
		fsl_usb_lowpower_mode(pdata, false);
	}

	/* disable the host wakeup */
	usb_host_set_wakeup(hcd->self.controller, false);
	/*free the ehci_fsl_pre_irq  */
	free_irq(hcd->irq, (void *)pdev);

	usb_remove_hcd(hcd);

	ehci_port_power(ehci, 0);

	iounmap(hcd->regs);

	if (ehci->transceiver) {
		(void)otg_set_host(ehci->transceiver, 0);
		otg_put_transceiver(ehci->transceiver);
	} else {
		release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	}

	usb_put_hcd(hcd);

	fsl_usb_lowpower_mode(pdata, true);

	/* Close the VBUS */
	if (pdata->xcvr_ops && pdata->xcvr_ops->set_vbus_power)
		pdata->xcvr_ops->set_vbus_power(pdata->xcvr_ops, pdata, 0);

	if (pdata->usb_clock_for_pm)
		pdata->usb_clock_for_pm(false);

	/*
	 * do platform specific un-initialization
	 */
	if (pdata->exit && pdata->pdev)
		pdata->exit(pdata->pdev);
}

static void fsl_setup_phy(struct ehci_hcd *ehci,
			  enum fsl_usb2_phy_modes phy_mode, int port_offset)
{
	u32 portsc;

	portsc = ehci_readl(ehci, &ehci->regs->port_status[port_offset]);
	portsc &= ~(PORT_PTS_MSK | PORT_PTS_PTW);

	switch (phy_mode) {
	case FSL_USB2_PHY_ULPI:
		portsc |= PORT_PTS_ULPI;
		break;
	case FSL_USB2_PHY_SERIAL:
		portsc |= PORT_PTS_SERIAL;
		break;
	case FSL_USB2_PHY_UTMI_WIDE:
		portsc |= PORT_PTS_PTW;
		/* fall through */
	case FSL_USB2_PHY_UTMI:
		portsc |= PORT_PTS_UTMI;
		break;
		/* HSIC */
	case FSL_USB2_PHY_HSIC:
		portsc |= PORT_PTS_HSIC;
		break;
	case FSL_USB2_PHY_NONE:
		break;
	}
	ehci_writel(ehci, portsc, &ehci->regs->port_status[port_offset]);
}

static void ehci_fsl_stream_disable(struct ehci_hcd *ehci)
{
	u32 __iomem	*reg_ptr;
	u32		tmp;

	reg_ptr = (u32 __iomem *)(((u8 __iomem *)ehci->regs) + USBMODE);
	tmp = ehci_readl(ehci, reg_ptr);
	tmp |= CI_USBMODE_SDIS;
	ehci_writel(ehci, tmp, reg_ptr);
}

/* called after powerup, by probe or system-pm "wakeup" */
static int ehci_fsl_reinit(struct ehci_hcd *ehci)
{
	fsl_platform_usb_setup(ehci);
	ehci_port_power(ehci, 0);
	ehci_fsl_stream_disable(ehci);

	return 0;
}

static int ehci_fsl_bus_suspend(struct usb_hcd *hcd)
{
	int ret = 0;
	struct fsl_usb2_platform_data *pdata;
	u32 tmp, portsc, cmd;
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);

	pdata = hcd->self.controller->platform_data;
	printk(KERN_DEBUG "%s begins, %s\n", __func__, pdata->name);

	/* the host is already at low power mode */
	if (!test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags)) {
		return 0;
	}

	portsc = ehci_readl(ehci, &ehci->regs->port_status[0]);
	ret = ehci_bus_suspend(hcd);
	if (ret != 0)
		return ret;

	cmd = ehci_readl(ehci, &ehci->regs->command);
	if ((portsc & PORT_CONNECT) && ((cmd & CMD_RUN) == 0)) {
		tmp = ehci_readl(ehci, &ehci->regs->command);
		tmp |= CMD_RUN;
		ehci_writel(ehci, tmp, &ehci->regs->command);
		/* on MX6Q, it need a short delay between set RUNSTOP
		 * and set PHCD
		 */
		udelay(125);
	}
	if (pdata->platform_suspend)
		pdata->platform_suspend(pdata);
	usb_host_set_wakeup(hcd->self.controller, true);
	fsl_usb_lowpower_mode(pdata, true);
	fsl_usb_clk_gate(hcd->self.controller->platform_data, false);
	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	printk(KERN_DEBUG "%s ends, %s\n", __func__, pdata->name);

	return ret;
}

static int ehci_fsl_bus_resume(struct usb_hcd *hcd)
{
	int ret = 0;
	struct fsl_usb2_platform_data *pdata;

	pdata = hcd->self.controller->platform_data;
	printk(KERN_DEBUG "%s begins, %s\n", __func__, pdata->name);

	/*
	 * At otg mode, it should not call host resume for usb gadget device
	 * Otherwise, this usb device can't be recognized as a gadget
	 */
	if (hcd->self.is_b_host) {
		return -ESHUTDOWN;
	}

	if (!test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags)) {
		fsl_usb_clk_gate(hcd->self.controller->platform_data, true);
		usb_host_set_wakeup(hcd->self.controller, false);
		fsl_usb_lowpower_mode(pdata, false);
		set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	}

	if (pdata->platform_resume)
		pdata->platform_resume(pdata);

	ret = ehci_bus_resume(hcd);
	if (ret)
		return ret;
	printk(KERN_DEBUG "%s ends, %s\n", __func__, pdata->name);

	return ret;
}

static void ehci_fsl_shutdown(struct usb_hcd *hcd)
{
	if (!test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags)) {
		set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
		fsl_usb_clk_gate(hcd->self.controller->platform_data, true);
	}
	/* Disable wakeup event first */
	usb_host_set_wakeup(hcd->self.controller, false);

	ehci_shutdown(hcd);
	if (test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags)) {
		clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
		fsl_usb_clk_gate(hcd->self.controller->platform_data, false);
	}
}

/* called during probe() after chip reset completes */
static int ehci_fsl_setup(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int retval;

	/* EHCI registers start at offset 0x100 */
	ehci->caps = hcd->regs + 0x100;
	ehci->regs = hcd->regs + 0x100 +
	    HC_LENGTH(ehci, ehci_readl(ehci, &ehci->caps->hc_capbase));
	dbg_hcs_params(ehci, "reset");
	dbg_hcc_params(ehci, "reset");

	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = ehci_readl(ehci, &ehci->caps->hcs_params);

	retval = ehci_halt(ehci);
	if (retval)
		return retval;

	/* data structure init */
	retval = ehci_init(hcd);
	if (retval)
		return retval;

	hcd->has_tt = 1;

	ehci->sbrn = 0x20;

	ehci_reset(ehci);

	retval = ehci_fsl_reinit(ehci);
	return retval;
}

/* called after hcd send port_reset cmd */
static int ehci_fsl_reset_device(struct usb_hcd *hcd, struct usb_device *udev)
{
	struct fsl_usb2_platform_data *pdata;

	pdata = hcd->self.controller->platform_data;

	if (pdata->hsic_device_connected)
		pdata->hsic_device_connected();

	return 0;
}

static const struct hc_driver ehci_fsl_hc_driver = {
	.description = hcd_name,
	.product_desc = "Freescale On-Chip EHCI Host Controller",
	.hcd_priv_size = sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq = ehci_irq,
	.flags = HCD_USB2,

	/*
	 * basic lifecycle operations
	 */
	.reset = ehci_fsl_setup,
	.start = ehci_run,
	.stop = ehci_stop,
	.shutdown = ehci_fsl_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue = ehci_urb_enqueue,
	.urb_dequeue = ehci_urb_dequeue,
	.endpoint_disable = ehci_endpoint_disable,
	.endpoint_reset = ehci_endpoint_reset,

	/*
	 * scheduling support
	 */
	.get_frame_number = ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data = ehci_hub_status_data,
	.hub_control = ehci_hub_control,
	.bus_suspend = ehci_fsl_bus_suspend,
	.bus_resume = ehci_fsl_bus_resume,
	.relinquish_port = ehci_relinquish_port,
	.port_handed_over = ehci_port_handed_over,

	.clear_tt_buffer_complete = ehci_clear_tt_buffer_complete,
	.reset_device = ehci_fsl_reset_device,
};

static int ehci_fsl_drv_probe(struct platform_device *pdev)
{
	if (usb_disabled())
		return -ENODEV;

	/* FIXME we only want one one probe() not two */
	return usb_hcd_fsl_probe(&ehci_fsl_hc_driver, pdev);
}

static int ehci_fsl_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	/* FIXME we only want one one remove() not two */
	usb_hcd_fsl_remove(hcd, pdev);
	return 0;
}

#ifdef CONFIG_PM

static bool host_can_wakeup_system(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;

	if (pdata->operating_mode == FSL_USB2_DR_OTG)
		if (device_may_wakeup(ehci->transceiver->dev))
			return true;
		else
			return false;
	else
		if (device_may_wakeup(&(pdev->dev)))
			return true;
		else
			return false;
}

/* suspend/resume, section 4.3 */

/* These routines rely on the bus (pci, platform, etc)
 * to handle powerdown and wakeup, and currently also on
 * transceivers that don't need any software attention to set up
 * the right sort of wakeup.
 *
 * They're also used for turning on/off the port when doing OTG.
 */
static int ehci_fsl_drv_suspend(struct platform_device *pdev,
				pm_message_t message)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	struct usb_device *roothub = hcd->self.root_hub;
	u32 port_status;
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;

	printk(KERN_DEBUG "USB Host suspend begins\n");
	/* Only handles OTG mode switch event, system suspend event will be done in bus suspend */
	if (pdata->pmflags == 0) {
		printk(KERN_DEBUG "%s, pm event \n", __func__);
		disable_irq(hcd->irq);
		if (!host_can_wakeup_system(pdev)) {
			/* Need open clock for register access */
			fsl_usb_clk_gate(hcd->self.controller->platform_data, true);
			/*
			 * Disable wakeup interrupt, since there is wakeup
			 * when phcd from 1->0 if wakeup interrupt is enabled
			 */
			usb_host_set_wakeup(hcd->self.controller, false);

			/*
			 * Open PHY's clock, then the wakeup settings
			 * can be wroten correctly
			 */
			fsl_usb_lowpower_mode(pdata, false);

			usb_host_set_wakeup(hcd->self.controller, false);

			fsl_usb_lowpower_mode(pdata, true);

			fsl_usb_clk_gate(hcd->self.controller->platform_data, false);
		} else {
			if (pdata->platform_phy_power_on)
				pdata->platform_phy_power_on();
		}
		enable_irq(hcd->irq);

		printk(KERN_DEBUG "host suspend ends\n");
		return 0;
	}

	/* only the otg host can go here */
	/* wait for all usb device on the hcd dettached */
	usb_lock_device(roothub);
	if (roothub->children[0] != NULL) {
		int old = hcd->self.is_b_host;
		printk(KERN_DEBUG "will resume roothub and its children\n");
		hcd->self.is_b_host = 0;
		/* resume the roothub, so that it can test the children is disconnected */
		if (roothub->state == USB_STATE_SUSPENDED)
			usb_resume(&roothub->dev, PMSG_USER_SUSPEND);
		/* we must do unlock here, the hubd thread will hold the same lock
		 * here release the lock, so that the hubd thread can process the usb
		 * disconnect event and set the children[0] be NULL, or there will be
		 * a deadlock */
		usb_unlock_device(roothub);
		while (roothub->children[0] != NULL)
			msleep(1);
		usb_lock_device(roothub);
		hcd->self.is_b_host = old;
	}
	usb_unlock_device(roothub);

	if (!(hcd->state & HC_STATE_SUSPENDED)) {
		printk(KERN_DEBUG "will suspend roothub and its children\n");
		usb_lock_device(roothub);
		usb_suspend(&roothub->dev, PMSG_USER_SUSPEND);
		usb_unlock_device(roothub);
	}

	if (!test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags)) {
		fsl_usb_clk_gate(hcd->self.controller->platform_data, true);
		set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	}

	port_status = ehci_readl(ehci, &ehci->regs->port_status[0]);
	/* save EHCI registers */
	pdata->pm_command = ehci_readl(ehci, &ehci->regs->command);
	pdata->pm_command &= ~CMD_RUN;
	pdata->pm_status  = ehci_readl(ehci, &ehci->regs->status);
	pdata->pm_intr_enable  = ehci_readl(ehci, &ehci->regs->intr_enable);
	pdata->pm_frame_index  = ehci_readl(ehci, &ehci->regs->frame_index);
	pdata->pm_segment  = ehci_readl(ehci, &ehci->regs->segment);
	pdata->pm_frame_list  = ehci_readl(ehci, &ehci->regs->frame_list);
	pdata->pm_async_next  = ehci_readl(ehci, &ehci->regs->async_next);
	pdata->pm_configured_flag  =
		ehci_readl(ehci, &ehci->regs->configured_flag);
	pdata->pm_portsc = ehci_readl(ehci, &ehci->regs->port_status[0]);

	/* clear the W1C bits */
	pdata->pm_portsc &= cpu_to_hc32(ehci, ~PORT_RWC_BITS);

	/* clear PHCD bit */
	pdata->pm_portsc &= ~PORT_PTS_PHCD;

	usb_host_set_wakeup(hcd->self.controller, true);
	fsl_usb_lowpower_mode(pdata, true);

	if (test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags)) {
		clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
		fsl_usb_clk_gate(hcd->self.controller->platform_data, false);
	}
	pdata->pmflags = 0;
	printk(KERN_DEBUG "host suspend ends\n");
	return 0;
}

#define OTGSC_OFFSET 		0x64
#define OTGSC_ID_VALUE		(1 << 8)
#define OTGSC_ID_INT_STS	(1 << 16)
static int ehci_fsl_drv_resume(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	struct usb_device *roothub = hcd->self.root_hub;
	unsigned long flags;
	u32 tmp, otgsc;
	bool id_changed;
	int id_value;
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;
	struct fsl_usb2_wakeup_platform_data *wake_up_pdata = pdata->wakeup_pdata;
	/* Only handles OTG mode switch event */
	printk(KERN_DEBUG "ehci fsl drv resume begins: %s\n", pdata->name);
	if (pdata->pmflags == 0) {
		printk(KERN_DEBUG "%s,pm event, wait for wakeup irq if needed\n", __func__);
		wait_event_interruptible(wake_up_pdata->wq, !wake_up_pdata->usb_wakeup_is_pending);
		disable_irq(hcd->irq);
		if (!host_can_wakeup_system(pdev)) {
			/* Need open clock for register access */
			u32 __iomem	*reg_ptr;
			fsl_usb_clk_gate(hcd->self.controller->platform_data, true);

			reg_ptr = (u32 __iomem *)(((u8 __iomem *)ehci->regs) + USBMODE);
			tmp = ehci_readl(ehci, reg_ptr);

			/* quit, if not in host mode */
			if ((tmp & USBMODE_CM_HC) != USBMODE_CM_HC) {
				usb_host_set_wakeup(hcd->self.controller, true);
				fsl_usb_clk_gate(hcd->self.controller->platform_data, false);
				enable_irq(hcd->irq);
				return 0;
			}
			fsl_usb_lowpower_mode(pdata, false);

			tmp = ehci_readl(ehci, &ehci->regs->port_status[0]);
			if (pdata->operating_mode == FSL_USB2_DR_OTG) {
				otgsc = ehci_readl(ehci, (u32 __iomem *)ehci->regs + OTGSC_OFFSET / 4);
				id_changed = !!(otgsc & OTGSC_ID_INT_STS);
				id_value = !!(otgsc & OTGSC_ID_VALUE);
				if (((tmp & PORT_CONNECT) && !id_value) || id_changed) {
					set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
				} else if (!(tmp & PORT_CONNECT)) {
					usb_host_set_wakeup(hcd->self.controller, true);
					fsl_usb_lowpower_mode(pdata, true);
					fsl_usb_clk_gate(hcd->self.controller->platform_data, false);
				}
			} else {
				if (tmp & PORT_CONNECT) {
					set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
				} else {
					usb_host_set_wakeup(hcd->self.controller, true);
					fsl_usb_lowpower_mode(pdata, true);
					fsl_usb_clk_gate(hcd->self.controller->platform_data, false);
				}

			}
		}
		enable_irq(hcd->irq);
		return 0;
	}
	if (!test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags)) {
		set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
		fsl_usb_clk_gate(hcd->self.controller->platform_data, true);
		usb_host_set_wakeup(hcd->self.controller, false);
		fsl_usb_lowpower_mode(pdata, false);
	}

	spin_lock_irqsave(&ehci->lock, flags);
	/* set host mode */
	fsl_platform_set_host_mode(hcd);

	/* restore EHCI registers */
	ehci_writel(ehci, pdata->pm_portsc, &ehci->regs->port_status[0]);
	ehci_writel(ehci, pdata->pm_command, &ehci->regs->command);
	ehci_writel(ehci, pdata->pm_intr_enable, &ehci->regs->intr_enable);
	ehci_writel(ehci, pdata->pm_frame_index, &ehci->regs->frame_index);
	ehci_writel(ehci, pdata->pm_segment, &ehci->regs->segment);
	ehci_writel(ehci, pdata->pm_frame_list, &ehci->regs->frame_list);
	ehci_writel(ehci, pdata->pm_async_next, &ehci->regs->async_next);
	ehci_writel(ehci, pdata->pm_configured_flag,
		    &ehci->regs->configured_flag);

	ehci_fsl_stream_disable(ehci);

	tmp = ehci_readl(ehci, &ehci->regs->command);
	tmp |= CMD_RUN;
	ehci_writel(ehci, tmp, &ehci->regs->command);
	spin_unlock_irqrestore(&ehci->lock, flags);

	if ((hcd->state & HC_STATE_SUSPENDED)) {
		printk(KERN_DEBUG "will resume roothub and its children\n");
		usb_lock_device(roothub);
		usb_resume(&roothub->dev, PMSG_USER_RESUME);
		usb_unlock_device(roothub);
	}
	pdata->pmflags = 0;
	printk(KERN_DEBUG "ehci fsl drv resume ends: %s\n", pdata->name);

	return 0;
}
#endif
MODULE_ALIAS("platform:fsl-ehci");

static struct platform_driver ehci_fsl_driver = {
	.probe = ehci_fsl_drv_probe,
	.remove = ehci_fsl_drv_remove,
	.shutdown = usb_hcd_platform_shutdown,
#ifdef CONFIG_PM
	.suspend = ehci_fsl_drv_suspend,
	.resume = ehci_fsl_drv_resume,
#endif
	.driver = {
		   .name = "fsl-ehci",
	},
};
