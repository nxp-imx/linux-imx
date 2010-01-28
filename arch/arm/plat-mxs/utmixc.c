/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/usb/fsl_xcvr.h>
#include <linux/pmic_external.h>

#include <mach/hardware.h>
#include <mach/arc_otg.h>
#include <asm/mach-types.h>


static void usb_utmi_init(struct fsl_xcvr_ops *this)
{
}

static void usb_utmi_uninit(struct fsl_xcvr_ops *this)
{
}

/*!
 * set vbus power
 *
 * @param       view  viewport register
 * @param       on    power on or off
 */
static void set_power(struct fsl_xcvr_ops *this,
		      struct fsl_usb2_platform_data *pdata, int on)
{

}

static struct fsl_xcvr_ops utmi_ops = {
	.name = "utmi",
	.xcvr_type = PORTSC_PTS_UTMI,
	.init = usb_utmi_init,
	.uninit = usb_utmi_uninit,
	.set_vbus_power = set_power,
};

extern void fsl_usb_xcvr_register(struct fsl_xcvr_ops *xcvr_ops);

static int __init utmixc_init(void)
{
	fsl_usb_xcvr_register(&utmi_ops);
	return 0;
}

extern void fsl_usb_xcvr_unregister(struct fsl_xcvr_ops *xcvr_ops);

static void __exit utmixc_exit(void)
{
	fsl_usb_xcvr_unregister(&utmi_ops);
}

module_init(utmixc_init);
module_exit(utmixc_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("utmi xcvr driver");
MODULE_LICENSE("GPL");
