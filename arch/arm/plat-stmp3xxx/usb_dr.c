/*
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <mach/arc_otg.h>
#include <mach/irqs.h>
#include <mach/platform.h>
#include <mach/regs-usbctrl.h>
#include <mach/regs-usbphy.h>
#include "usb.h"

/*
 * platform data structs
 * 	- Which one to use is determined by CONFIG options in usb.h
 * 	- operating_mode plugged at run time
 */
static struct fsl_usb2_platform_data __maybe_unused dr_utmi_config = {
	.name              = "DR",
	.platform_init     = usbotg_init,
	.platform_uninit   = usbotg_uninit,
	.phy_mode          = FSL_USB2_PHY_UTMI_WIDE,
	.power_budget      = 500,	/* 500 mA max power */
	.platform_resume = usb_host_phy_resume,
	.transceiver       = "utmi",
};

/*
 * resources
 */
static struct resource otg_resources[] = {
	[0] = {
		.start	= (u32)REGS_USBCTRL_PHYS,
		.end	= (u32)(REGS_USBCTRL_PHYS + SZ_4K),
		.flags	= IORESOURCE_MEM,
	},

	[1] = {
		.start	= IRQ_USB_CTRL,
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
	.resource      = otg_resources,
	.num_resources = ARRAY_SIZE(otg_resources),
};

static void usb_host_phy_resume(struct fsl_usb2_platform_data *plat)
{
	stmp3xxx_clearl(BM_USBPHY_CTRL_ENHOSTDISCONDETECT, 
			REGS_USBPHY_BASE + HW_USBPHY_CTRL);
}

static int __init usb_dr_init(void)
{
	pr_debug("%s: \n", __func__);

	dr_register_otg();
	dr_register_host(otg_resources, ARRAY_SIZE(otg_resources));
	dr_register_udc();

	return 0;
}

module_init(usb_dr_init);
