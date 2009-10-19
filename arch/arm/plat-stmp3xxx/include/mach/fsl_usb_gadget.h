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

/*
 * USB Gadget side, platform-specific functionality.
 */

#include <linux/usb/fsl_xcvr.h>

/* Needed for i2c/serial transceivers */
static inline void
fsl_platform_set_device_mode(struct fsl_usb2_platform_data *pdata)
{
}

static inline void
fsl_platform_pullup_enable(struct fsl_usb2_platform_data *pdata)
{
}

static inline void
fsl_platform_pullup_disable(struct fsl_usb2_platform_data *pdata)
{
}

static inline void
fsl_platform_set_test_mode(struct fsl_usb2_platform_data *pdata,
		enum usb_test_mode mode)
{
}
