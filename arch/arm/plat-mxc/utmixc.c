/*
 * Copyright (C) 2005-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
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

static struct regulator *usbotg_regux;

static void usb_utmi_init(struct fsl_xcvr_ops *this)
{
#if defined(CONFIG_MXC_PMIC_MC13892_MODULE) || defined(CONFIG_MXC_PMIC_MC13892)
	if (machine_is_mx51_3ds()) {
		unsigned int value;

		/* VUSBIN */
		pmic_read_reg(REG_USB1, &value, 0xffffff);
		value |= 0x1;
		value |= (0x1 << 3);
		pmic_write_reg(REG_USB1, value, 0xffffff);
	}
#endif
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
	struct device *dev = &pdata->pdev->dev;

	pr_debug("real %s(on=%d) pdata=0x%p\n", __func__, on, pdata);
	if (machine_is_mx37_3ds()) {
		if (on) {
			if (!board_is_rev(BOARD_REV_2))
				usbotg_regux = regulator_get(dev, "DCDC2");
			else
				usbotg_regux = regulator_get(dev, "SWBST");

			regulator_enable(usbotg_regux);
		} else {
			regulator_disable(usbotg_regux);
			regulator_put(usbotg_regux);
		}
	}
	if (pdata && pdata->platform_driver_vbus)
		pdata->platform_driver_vbus(on);
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

subsys_initcall(utmixc_init);
module_exit(utmixc_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("utmi xcvr driver");
MODULE_LICENSE("GPL");
