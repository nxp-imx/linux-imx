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

extern void fsl_phy_usb_utmi_init(struct fsl_xcvr_ops *this);
extern void fsl_phy_usb_utmi_uninit(struct fsl_xcvr_ops *this);
extern void fsl_phy_set_power(struct fsl_xcvr_ops *this,
			struct fsl_usb2_platform_data *pdata, int on);
#include <mach/regs-power.h>
#include <asm/io.h>


static void set_vbus_draw(struct fsl_xcvr_ops *this,
		struct fsl_usb2_platform_data *pdata, unsigned mA)
{
#ifdef CONFIG_MXS_VBUS_CURRENT_DRAW
	if ((__raw_readl(REGS_POWER_BASE + HW_POWER_5VCTRL)
		& BM_POWER_5VCTRL_CHARGE_4P2_ILIMIT) == 0x20000) {
		printk(KERN_INFO "USB enumeration done,current limitation release\r\n");
		__raw_writel(__raw_readl(REGS_POWER_BASE + HW_POWER_5VCTRL) |
		BM_POWER_5VCTRL_CHARGE_4P2_ILIMIT, REGS_POWER_BASE +
		HW_POWER_5VCTRL);
	}
#endif
}
static struct fsl_xcvr_ops utmi_ops = {
	.name = "utmi",
	.xcvr_type = PORTSC_PTS_UTMI,
	.init = fsl_phy_usb_utmi_init,
	.uninit = fsl_phy_usb_utmi_uninit,
	.set_vbus_power = fsl_phy_set_power,
	.set_vbus_draw = set_vbus_draw,
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

#ifdef CONFIG_MXS_VBUS_CURRENT_DRAW
	fs_initcall(utmixc_init);
#else
	subsys_initcall(utmixc_init);
#endif
module_exit(utmixc_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("utmi xcvr driver");
MODULE_LICENSE("GPL");
