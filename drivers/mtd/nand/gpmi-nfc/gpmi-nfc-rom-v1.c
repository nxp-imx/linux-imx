/*
 * Freescale GPMI NFC NAND Flash Driver
 *
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
 * Copyright (C) 2008 Embedded Alley Solutions, Inc.
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

#include "gpmi-nfc.h"

/**
 * set_geometry() - Sets geometry for the Boot ROM Helper.
 *
 * @this:  Per-device data.
 */
static int set_geometry(struct gpmi_nfc_data *this)
{
	struct gpmi_nfc_platform_data  *pdata    =  this->pdata;
	struct boot_rom_geometry       *geometry = &this->rom_geometry;
	int                            error;

	/* Version-independent geometry. */

	error = gpmi_nfc_rom_helper_set_geometry(this);

	if (error)
		return error;

	/*
	 * Check if the platform data indicates we are to protect the boot area.
	 */

	if (!pdata->boot_area_size_in_bytes) {
		geometry->boot_area_count         = 0;
		geometry->boot_area_size_in_bytes = 0;
		return 0;
	}

	/*
	 * If control arrives here, we are supposed to set up partitions to
	 * protect the boot areas. In this version of the ROM, we support only
	 * one boot area.
	 */

	geometry->boot_area_count = 1;

	/*
	 * Use the platform's boot area size.
	 */

	geometry->boot_area_size_in_bytes = pdata->boot_area_size_in_bytes;

	/* Return success. */

	return 0;

}

/* This structure represents the Boot ROM Helper for this version. */

struct boot_rom_helper  gpmi_nfc_boot_rom_helper_v1 = {
	.version                   = 1,
	.description               = "Single-chip boot area, "
						"block mark swapping supported",
	.swap_block_mark           = true,
	.set_geometry              = set_geometry,
};
