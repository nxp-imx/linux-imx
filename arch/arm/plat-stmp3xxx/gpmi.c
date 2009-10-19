/*
 * Freescale STMP37XX/STMP378X GPMI module pin multiplexing
 *
 * Author: dmitry pervushin <dimka@embeddedalley.com>
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
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
#include <linux/io.h>
#include <linux/module.h>
#include <mach/platform.h>

int gpmi_pinmux_request(char *title)
{
	int err = 0;

	err = stmp3xxx_request_pin_group(&gpmi_pins, title);

	return err;
}
EXPORT_SYMBOL_GPL(gpmi_pinmux_request);

void gpmi_pinmux_free(char *title)
{
	stmp3xxx_release_pin_group(&gpmi_pins, title);
}
EXPORT_SYMBOL_GPL(gpmi_pinmux_free);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("dmitry pervushin <dimka@embeddedalley.com>");
