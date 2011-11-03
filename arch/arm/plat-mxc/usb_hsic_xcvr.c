/*
 * Copyright (C) 2005-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 * For HSIC, there is no usb phy, so the operation at this file is noop
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/pmic_external.h>
#include <linux/usb/fsl_xcvr.h>

#include <mach/hardware.h>
#include <mach/arc_otg.h>
#include <asm/mach-types.h>

static struct fsl_xcvr_ops hsic_xcvr_ops = {
	.name = "hsic_xcvr",
	.xcvr_type = PORTSC_HSIC_MODE,
};

extern void fsl_usb_xcvr_register(struct fsl_xcvr_ops *xcvr_ops);

static int __init hsic_xcvr_init(void)
{
	fsl_usb_xcvr_register(&hsic_xcvr_ops);
	return 0;
}

extern void fsl_usb_xcvr_unregister(struct fsl_xcvr_ops *xcvr_ops);

static void __exit hsic_xcvr_exit(void)
{
	fsl_usb_xcvr_unregister(&hsic_xcvr_ops);
}

subsys_initcall(hsic_xcvr_init);
module_exit(hsic_xcvr_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("nop xcvr driver");
MODULE_LICENSE("GPL");
