/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/init.h>
#include <linux/string.h>
#include <linux/kernel.h>

int __initdata primary_di = { -1 };
static int __init di1_setup(char *__unused)
{
	primary_di = 1;
	return 1;
}
__setup("di1_primary", di1_setup);

static int __init di0_setup(char *__unused)
{
	primary_di = 0;
	return 1;
}
__setup("di0_primary", di0_setup);

int __initdata lcdif_sel_lcd = { 0 };
static int __init lcd_setup(char *str)
{
	int s, ret;

	s = *str;
	if (s == '=') {

		str++;
		ret = strict_strtoul(str, 0, &lcdif_sel_lcd);
		if (ret < 0)
			return 0;
		return 1;
	} else
		return 0;
}
__setup("lcd", lcd_setup);

