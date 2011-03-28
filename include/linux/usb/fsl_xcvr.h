/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU Lesser General
 * Public License.  You may obtain a copy of the GNU Lesser General
 * Public License Version 2.1 or later at the following locations:
 *
 * http://www.opensource.org/licenses/lgpl-license.html
 * http://www.gnu.org/copyleft/lgpl.html
 */
#ifndef __LINUX_USB_FSL_XCVR_H
#define __LINUX_USB_FSL_XCVR_H
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

struct fsl_usb2_platform_data;

enum usb_test_mode{
	USB_TEST_J = 1,
	USB_TEST_K = 2,
};

/**
 * @name: transceiver name
 * @xcvr_type: one of PORTSC_PTS_{UTMI,SERIAL,ULPI}
 * @init: transceiver- and board-specific initialization function
 * @uninit: transceiver- and board-specific uninitialization function
 * @set_host:
 * @set_device:
 * @pullup: enable or disable D+ pullup
 *
 */
struct fsl_xcvr_ops {
	char *name;
	u32 xcvr_type;

	void (*init)(struct fsl_xcvr_ops *ops);
	void (*uninit)(struct fsl_xcvr_ops *ops);
	void (*suspend)(struct fsl_xcvr_ops *ops);
	void (*set_host)(void);
	void (*set_device)(void);
	void (*set_vbus_power)(struct fsl_xcvr_ops *ops,
			       struct fsl_usb2_platform_data *pdata, int on);
	void (*set_vbus_draw)(struct fsl_xcvr_ops *ops,
			struct fsl_usb2_platform_data *pdata, unsigned mA);
	void (*set_remote_wakeup)(u32 *view);
	void (*pullup)(int on);
	void(*set_test_mode)(u32 *view, enum usb_test_mode mode);
};

struct fsl_xcvr_power {
	struct platform_device *usb_pdev;
	struct regulator *regu1;
	struct regulator *regu2;
};
#endif
