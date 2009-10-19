/*
 * Freescale STMP378X Rotary Encoder module pin multiplexing
 *
 * Author: Drew Benedetti <drewb@embeddedalley.com>
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
#include <mach/pins.h>
#include <mach/pinmux.h>

#define ROTARY_FUN	PIN_FUN1

#define TITLE	"stmp3xxx-rotdec"

int rotdec_pinmux_request(void)
{
	int rc = 0;

	rc |= stmp3xxx_request_pin(PINID_ROTARYA, ROTARY_FUN, TITLE);
	rc |= stmp3xxx_request_pin(PINID_ROTARYB, ROTARY_FUN, TITLE);

	return rc;
}
EXPORT_SYMBOL_GPL(rotdec_pinmux_request);

void rotdec_pinmux_free(void)
{
	stmp3xxx_release_pin(PINID_ROTARYA, TITLE);
	stmp3xxx_release_pin(PINID_ROTARYB, TITLE);
}
EXPORT_SYMBOL_GPL(rotdec_pinmux_free);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Drew Benedetti <drewb@embeddedalley.com>");
