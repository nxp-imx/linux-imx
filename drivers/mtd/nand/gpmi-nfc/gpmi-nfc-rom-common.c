/*
 * Freescale GPMI NFC NAND Flash Driver
 *
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
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
 * gpmi_nfc_rom_helper_set_geometry() - Sets geometry for the Boot ROM Helper.
 *
 * @this:  Per-device data.
 */
int gpmi_nfc_rom_helper_set_geometry(struct gpmi_nfc_data *this)
{
	struct boot_rom_geometry  *geometry = &this->rom_geometry;

	/*
	 * Set the boot block stride size.
	 *
	 * In principle, we should be reading this from the OTP bits, since
	 * that's where the ROM is going to get it. In fact, we don't have any
	 * way to read the OTP bits, so we go with the default and hope for the
	 * best.
	 */

	geometry->stride_size_in_pages = 64;

	/*
	 * Set the search area stride exponent.
	 *
	 * In principle, we should be reading this from the OTP bits, since
	 * that's where the ROM is going to get it. In fact, we don't have any
	 * way to read the OTP bits, so we go with the default and hope for the
	 * best.
	 */

	geometry->search_area_stride_exponent = 2;

	/* Return success. */

	return 0;

}
