/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <mach/sdma.h>

#include "mx31_sdma_script_code_v1.h"
#include "mx31_sdma_script_code_v2.h"
#include "mx35_sdma_script_code_v1.h"
#include "mx35_sdma_script_code_v2.h"

void mx31_sdma_get_script_info(sdma_script_start_addrs *sdma_script_addr)
{
	sdma_script_start_addrs *d = sdma_script_addr;

	if (cpu_is_mx31_rev(CHIP_REV_1_0) == 1) {
		d->mxc_sdma_app_2_mcu_addr = mx31_app_2_mcu_ADDR;
		d->mxc_sdma_ap_2_ap_addr = mx31_ap_2_ap_ADDR;
		d->mxc_sdma_ap_2_bp_addr = -1;
		d->mxc_sdma_bp_2_ap_addr = -1;
		d->mxc_sdma_loopback_on_dsp_side_addr = -1;
		d->mxc_sdma_mcu_2_app_addr = mx31_mcu_2_app_ADDR;
		d->mxc_sdma_mcu_2_shp_addr = mx31_mcu_2_shp_ADDR;
		d->mxc_sdma_mcu_interrupt_only_addr = -1;
		d->mxc_sdma_shp_2_mcu_addr = mx31_shp_2_mcu_ADDR;
		d->mxc_sdma_start_addr = (unsigned short *)mx31_sdma_code;
		d->mxc_sdma_uartsh_2_mcu_addr = mx31_uartsh_2_mcu_ADDR;
		d->mxc_sdma_uart_2_mcu_addr = mx31_uart_2_mcu_ADDR;
		d->mxc_sdma_ram_code_size = MX31_RAM_CODE_SIZE;
		d->mxc_sdma_ram_code_start_addr = MX31_RAM_CODE_START_ADDR;
		d->mxc_sdma_dptc_dvfs_addr = mx31_dptc_dvfs_ADDR;
		d->mxc_sdma_firi_2_mcu_addr = mx31_firi_2_mcu_ADDR;
		d->mxc_sdma_firi_2_per_addr = -1;
		d->mxc_sdma_mshc_2_mcu_addr = mx31_mshc_2_mcu_ADDR;
		d->mxc_sdma_per_2_app_addr = -1;
		d->mxc_sdma_per_2_firi_addr = -1;
		d->mxc_sdma_per_2_shp_addr = -1;
		d->mxc_sdma_mcu_2_ata_addr = mx31_mcu_2_ata_ADDR;
		d->mxc_sdma_mcu_2_firi_addr = mx31_mcu_2_firi_ADDR;
		d->mxc_sdma_mcu_2_mshc_addr = mx31_mcu_2_mshc_ADDR;
		d->mxc_sdma_ata_2_mcu_addr = mx31_ata_2_mcu_ADDR;
		d->mxc_sdma_uartsh_2_per_addr = -1;
		d->mxc_sdma_shp_2_per_addr = -1;
		d->mxc_sdma_uart_2_per_addr = -1;
		d->mxc_sdma_app_2_per_addr = -1;
	} else {
		d->mxc_sdma_app_2_mcu_addr = mx31_app_2_mcu_ADDR_2;
		d->mxc_sdma_ap_2_ap_addr = mx31_ap_2_ap_ADDR_2;
		d->mxc_sdma_ap_2_ap_fixed_addr = mx31_ap_2_ap_fixed_addr_ADDR_2;
		d->mxc_sdma_ap_2_bp_addr = mx31_ap_2_bp_ADDR_2;
		d->mxc_sdma_ap_2_ap_fixed_addr = mx31_ap_2_ap_fixed_addr_ADDR_2;
		d->mxc_sdma_bp_2_ap_addr = mx31_bp_2_ap_ADDR_2;
		d->mxc_sdma_loopback_on_dsp_side_addr = -1;
		d->mxc_sdma_mcu_2_app_addr = mx31_mcu_2_app_ADDR_2;
		d->mxc_sdma_mcu_2_shp_addr = mx31_mcu_2_shp_ADDR_2;
		d->mxc_sdma_mcu_interrupt_only_addr = -1;
		d->mxc_sdma_shp_2_mcu_addr = mx31_shp_2_mcu_ADDR_2;
		d->mxc_sdma_start_addr = (unsigned short *)mx31_sdma_code_2;
		d->mxc_sdma_uartsh_2_mcu_addr = mx31_uartsh_2_mcu_ADDR_2;
		d->mxc_sdma_uart_2_mcu_addr = mx31_uart_2_mcu_ADDR_2;
		d->mxc_sdma_ram_code_size = MX31_RAM_CODE_SIZE_2;
		d->mxc_sdma_ram_code_start_addr = MX31_RAM_CODE_START_ADDR_2;
		d->mxc_sdma_dptc_dvfs_addr = -1;
		d->mxc_sdma_firi_2_mcu_addr = mx31_firi_2_mcu_ADDR_2;
		d->mxc_sdma_firi_2_per_addr = -1;
		d->mxc_sdma_mshc_2_mcu_addr = mx31_mshc_2_mcu_ADDR_2;
		d->mxc_sdma_per_2_app_addr = mx31_per_2_app_ADDR_2;
		d->mxc_sdma_per_2_firi_addr = -1;
		d->mxc_sdma_per_2_shp_addr = mx31_per_2_shp_ADDR_2;
		d->mxc_sdma_mcu_2_ata_addr = mx31_mcu_2_ata_ADDR_2;
		d->mxc_sdma_mcu_2_firi_addr = mx31_mcu_2_firi_ADDR_2;
		d->mxc_sdma_mcu_2_mshc_addr = mx31_mcu_2_mshc_ADDR_2;
		d->mxc_sdma_ata_2_mcu_addr = mx31_ata_2_mcu_ADDR_2;
		d->mxc_sdma_uartsh_2_per_addr = -1;
		d->mxc_sdma_shp_2_per_addr = mx31_shp_2_per_ADDR_2;
		d->mxc_sdma_uart_2_per_addr = -1;
		d->mxc_sdma_app_2_per_addr = mx31_app_2_per_ADDR_2;
	}
}

void mx35_sdma_get_script_info(sdma_script_start_addrs *sdma_script_addr)
{
	sdma_script_start_addrs *d = sdma_script_addr;

	if (cpu_is_mx35_rev(CHIP_REV_2_0) < 1) {
		d->mxc_sdma_ap_2_ap_addr = mx35_ap_2_ap_ADDR;
		d->mxc_sdma_ap_2_bp_addr = -1;
		d->mxc_sdma_bp_2_ap_addr = -1;
		d->mxc_sdma_loopback_on_dsp_side_addr = -1;
		d->mxc_sdma_mcu_interrupt_only_addr = -1;

		d->mxc_sdma_firi_2_per_addr = -1;
		d->mxc_sdma_firi_2_mcu_addr = -1;
		d->mxc_sdma_per_2_firi_addr = -1;
		d->mxc_sdma_mcu_2_firi_addr = -1;

		d->mxc_sdma_uart_2_per_addr = mx35_uart_2_per_ADDR;
		d->mxc_sdma_uart_2_mcu_addr = mx35_uart_2_mcu_ADDR;
		d->mxc_sdma_per_2_app_addr = mx35_per_2_app_ADDR;
		d->mxc_sdma_mcu_2_app_addr = mx35_mcu_2_app_ADDR;
		d->mxc_sdma_per_2_per_addr = mx35_p_2_p_ADDR;
		d->mxc_sdma_uartsh_2_per_addr = mx35_uartsh_2_per_ADDR;
		d->mxc_sdma_uartsh_2_mcu_addr = mx35_uartsh_2_mcu_ADDR;
		d->mxc_sdma_per_2_shp_addr = mx35_per_2_shp_ADDR;
		d->mxc_sdma_mcu_2_shp_addr = mx35_mcu_2_shp_ADDR;
		d->mxc_sdma_ata_2_mcu_addr = mx35_ata_2_mcu_ADDR;
		d->mxc_sdma_mcu_2_ata_addr = mx35_mcu_2_ata_ADDR;
		d->mxc_sdma_app_2_per_addr = mx35_app_2_per_ADDR;
		d->mxc_sdma_app_2_mcu_addr = mx35_app_2_mcu_ADDR;
		d->mxc_sdma_shp_2_per_addr = mx35_shp_2_per_ADDR;
		d->mxc_sdma_shp_2_mcu_addr = mx35_shp_2_mcu_ADDR;
		d->mxc_sdma_mshc_2_mcu_addr = -1;
		d->mxc_sdma_mcu_2_mshc_addr = -1;

		d->mxc_sdma_spdif_2_mcu_addr = mx35_spdif_2_mcu_ADDR;
		d->mxc_sdma_mcu_2_spdif_addr = mx35_mcu_2_spdif_ADDR;
		d->mxc_sdma_asrc_2_mcu_addr = mx35_asrc__mcu_ADDR;

		d->mxc_sdma_dptc_dvfs_addr = -1;
		d->mxc_sdma_ext_mem_2_ipu_addr = mx35_ext_mem__ipu_ram_ADDR;
		d->mxc_sdma_descrambler_addr = -1;

		d->mxc_sdma_start_addr = (unsigned short *)mx35_sdma_code;
		d->mxc_sdma_ram_code_size = MX35_RAM_CODE_SIZE;
		d->mxc_sdma_ram_code_start_addr = MX35_RAM_CODE_START_ADDR;
	} else {
		d->mxc_sdma_ap_2_ap_addr = mx35_ap_2_ap_ADDR_V2;
		d->mxc_sdma_ap_2_bp_addr = -1;
		d->mxc_sdma_bp_2_ap_addr = -1;
		d->mxc_sdma_loopback_on_dsp_side_addr = -1;
		d->mxc_sdma_mcu_interrupt_only_addr = -1;

		d->mxc_sdma_firi_2_per_addr = -1;
		d->mxc_sdma_firi_2_mcu_addr = -1;
		d->mxc_sdma_per_2_firi_addr = -1;
		d->mxc_sdma_mcu_2_firi_addr = -1;

		d->mxc_sdma_uart_2_per_addr = mx35_uart_2_per_ADDR_V2;
		d->mxc_sdma_uart_2_mcu_addr = mx35_uart_2_mcu_ADDR_V2;
		d->mxc_sdma_per_2_app_addr = mx35_per_2_app_ADDR_V2;
		d->mxc_sdma_mcu_2_app_addr = mx35_mcu_2_app_ADDR_V2;
		d->mxc_sdma_per_2_per_addr = mx35_p_2_p_ADDR_V2;
		d->mxc_sdma_uartsh_2_per_addr = mx35_uartsh_2_per_ADDR_V2;
		d->mxc_sdma_uartsh_2_mcu_addr = mx35_uartsh_2_mcu_ADDR_V2;
		d->mxc_sdma_per_2_shp_addr = mx35_per_2_shp_ADDR_V2;
		d->mxc_sdma_mcu_2_shp_addr = mx35_mcu_2_shp_ADDR_V2;

		d->mxc_sdma_ata_2_mcu_addr = mx35_ata_2_mcu_ADDR_V2;
		d->mxc_sdma_mcu_2_ata_addr = mx35_mcu_2_ata_ADDR_V2;

		d->mxc_sdma_app_2_per_addr = mx35_app_2_per_ADDR_V2;
		d->mxc_sdma_app_2_mcu_addr = mx35_app_2_mcu_ADDR_V2;
		d->mxc_sdma_shp_2_per_addr = mx35_shp_2_per_ADDR_V2;
		d->mxc_sdma_shp_2_mcu_addr = mx35_shp_2_mcu_ADDR_V2;

		d->mxc_sdma_mshc_2_mcu_addr = -1;
		d->mxc_sdma_mcu_2_mshc_addr = -1;

		d->mxc_sdma_spdif_2_mcu_addr = mx35_spdif_2_mcu_ADDR_V2;
		d->mxc_sdma_mcu_2_spdif_addr = mx35_mcu_2_spdif_ADDR_V2;

		d->mxc_sdma_asrc_2_mcu_addr = mx35_asrc__mcu_ADDR_V2;

		d->mxc_sdma_dptc_dvfs_addr = -1;
		d->mxc_sdma_ext_mem_2_ipu_addr = mx35_ext_mem__ipu_ram_ADDR_V2;
		d->mxc_sdma_descrambler_addr = -1;

		d->mxc_sdma_start_addr = (unsigned short *)mx35_sdma_code_v2;
		d->mxc_sdma_ram_code_size = MX35_RAM_CODE_SIZE;
		d->mxc_sdma_ram_code_start_addr = MX35_RAM_CODE_START_ADDR_V2;
	}
}

void mxc_sdma_get_script_info(sdma_script_start_addrs *sdma_script_addr)
{
	sdma_script_start_addrs *d = sdma_script_addr;

	if (cpu_is_mx31())
		mx31_sdma_get_script_info(d);
	else if (cpu_is_mx35())
		mx35_sdma_get_script_info(d);
}
