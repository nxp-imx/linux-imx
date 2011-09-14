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
 */

#include <mach/common.h>
#include "devices.h"

extern int usbotg_init(struct platform_device *pdev);
extern void usbotg_uninit(struct fsl_usb2_platform_data *pdata);
extern struct platform_device *host_pdev_register(struct resource *res,
		  int n_res, struct fsl_usb2_platform_data *config);

extern int fsl_usb_host_init(struct platform_device *pdev);
extern void fsl_usb_host_uninit(struct fsl_usb2_platform_data *pdata);
extern int gpio_usbotg_utmi_active(void);
extern void gpio_usbotg_utmi_inactive(void);

extern void __init mx5_usb_dr_init(void);
extern void __init mx5_usbh1_init(void);
extern void __init mx5_usbh2_init(void);

typedef void (*driver_vbus_func)(bool);
extern void mx5_set_host1_vbus_func(driver_vbus_func);
extern void mx5_set_otghost_vbus_func(driver_vbus_func);
/*
 * Used to set pdata->operating_mode before registering the platform_device.
 * If OTG is configured, the controller operates in OTG mode,
 * otherwise it's either host or device.
 */
#ifdef CONFIG_USB_OTG
#define DR_UDC_MODE	FSL_USB2_DR_OTG
#define DR_HOST_MODE	FSL_USB2_DR_OTG
#else
#define DR_UDC_MODE	FSL_USB2_DR_DEVICE
#define DR_HOST_MODE	FSL_USB2_DR_HOST
#endif

extern void __iomem *imx_otg_base;
