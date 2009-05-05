/*
 * Pin multiplexing for SPDIF transmitter on STMP3780 dev. board
 *
 * Author: Vladimir Barinov <vbarinov@embeddedalley.com>
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
#include "common.h"

int spdif_pinmux_request(void)
{
	return stmp3xxx_request_pin_group(&spdif_pins, "spdif");
}
EXPORT_SYMBOL_GPL(spdif_pinmux_request);

void spdif_pinmux_free(void)
{
	stmp3xxx_release_pin_group(&spdif_pins, "spdif");
}
EXPORT_SYMBOL_GPL(spdif_pinmux_free);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vladimir Barinov <vbarinov@embeddedalley.com>");
