/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <asm/sizes.h>
#include <linux/mtd/nand.h>

#include "nand_device_info.h"

/*
 * Type 2
 */
static struct nand_device_info nand_device_info_table_type_2[] __initdata = {
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x20,
	.device_code              = 0xf1,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 128LL*SZ_1M,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 30,
	.data_hold_in_ns          = 20,
	.address_setup_in_ns      = 25,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"NAND01GW3",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xad,
	.device_code              = 0xf1,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 128LL*SZ_1M,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 45,
	.data_hold_in_ns          = 30,
	.address_setup_in_ns      = 25,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	NULL,
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x2c,
	.device_code              = 0xf1,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 128LL*SZ_1M,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 30,
	.data_hold_in_ns          = 20,
	.address_setup_in_ns      = 10,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	NULL,
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xec,
	.device_code              = 0xf1,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 128LL*SZ_1M,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 35,
	.data_hold_in_ns          = 25,
	.address_setup_in_ns      = 0,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"K9F1F08",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x98,
	.device_code              = 0xf1,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 128LL*SZ_1M,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 30,
	.data_hold_in_ns          = 20,
	.address_setup_in_ns      = 0,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"TC58NVG0S3",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x45,
	.device_code              = 0xf1,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 128LL*SZ_1M,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 45,
	.data_hold_in_ns          = 32,
	.address_setup_in_ns      = 0,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	NULL,
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x20,
	.device_code              = 0xda,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 256LL*SZ_1M,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 20,
	.data_hold_in_ns          = 30,
	.address_setup_in_ns      = 0,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"NAND02GW3",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xad,
	.device_code              = 0xda,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 256LL*SZ_1M,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 30,
	.data_hold_in_ns          = 25,
	.address_setup_in_ns      = 10,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"HY27UF082G2M, HY27UG082G2M, HY27UG082G1M",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x2c,
	.device_code              = 0xda,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 256LL*SZ_1M,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 20,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 10,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"MT29F2G08",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xec,
	.device_code              = 0xda,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 256LL*SZ_1M,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 20,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 20,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"K9F2G08U0M",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x98,
	.device_code              = 0xda,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 256LL*SZ_1M,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 20,
	.data_hold_in_ns          = 30,
	.address_setup_in_ns      = 0,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"TC58NVG1S3",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x45,
	.device_code              = 0xda,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 256LL*SZ_1M,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 45,
	.data_hold_in_ns          = 32,
	.address_setup_in_ns      = 0,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	NULL,
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x20,
	.device_code              = 0xdc,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 512LL*SZ_1M,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 45,
	.data_hold_in_ns          = 30,
	.address_setup_in_ns      = 10,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	NULL,
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xad,
	.device_code              = 0xdc,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 512LL*SZ_1M,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 45,
	.data_hold_in_ns          = 30,
	.address_setup_in_ns      = 10,
	.gpmi_sample_delay_in_ns  = 10,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"HY27UH084G2M, HY27UG084G2M, HY27UH084G1M",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x2c,
	.device_code              = 0xdc,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 512LL*SZ_1M,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 20,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 10,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"MT29F4G08",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xec,
	.device_code              = 0xdc,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 512LL*SZ_1M,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 25,
	.data_hold_in_ns          = 25,
	.address_setup_in_ns      = 20,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	NULL,
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x98,
	.device_code              = 0xdc,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 512LL*SZ_1M,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 25,
	.data_hold_in_ns          = 25,
	.address_setup_in_ns      = 0,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"TH58NVG2S3",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x45,
	.device_code              = 0xdc,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 512LL*SZ_1M,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 45,
	.data_hold_in_ns          = 32,
	.address_setup_in_ns      = 0,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	NULL,
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xad,
	.device_code              = 0xd3,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 1LL*SZ_1G,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 30,
	.data_hold_in_ns          = 25,
	.address_setup_in_ns      = 20,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"HY27UH088G2M",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x20,
	.device_code              = 0xd3,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 1LL*SZ_1G,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 45,
	.data_hold_in_ns          = 30,
	.address_setup_in_ns      = 10,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"NAND08GW3BxANx",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x2c,
	.device_code              = 0xd3,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 1LL*SZ_1G,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 25,
	.data_hold_in_ns          = 15,
	.address_setup_in_ns      = 10,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"MT29F8G08FABWG",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x98,
	.device_code              = 0xd3,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 1LL*SZ_1G,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 45,
	.data_hold_in_ns          = 32,
	.address_setup_in_ns      = 0,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	NULL,
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x20,
	.device_code              = 0xd5,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 2LL*SZ_1G,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 45,
	.data_hold_in_ns          = 30,
	.address_setup_in_ns      = 10,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	NULL,
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xad,
	.device_code              = 0xd5,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 2LL*SZ_1G,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 25,
	.data_hold_in_ns          = 30,
	.address_setup_in_ns      = 10,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	NULL,
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x2c,
	.device_code              = 0xd5,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 2LL*SZ_1G,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 45,
	.data_hold_in_ns          = 32,
	.address_setup_in_ns      = 0,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	NULL,
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x2c,
	.device_code              = 0x48,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 2LL*SZ_1G,
	.block_size_in_pages      = 128,
	/* TODO: The actual oob size for MT29F16G08ABACA is
	224 bytes. Use oob 218 bytes since MX53 NFC controller
	mentions the spare area size must be less or equal 218
	byte if ECC is enabled */
	.page_total_size_in_bytes = 4*SZ_1K + 218,
	.ecc_strength_in_bits     = 8,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 15,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 20,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = 20,
	.tRLOH_in_ns              = 5,
	.tRHOH_in_ns              = 15,
	"MT29F16G08ABACA(2GB)",
	},
	{true}
};

/*
 * Large MLC
 */
static struct nand_device_info nand_device_info_table_large_mlc[] __initdata = {
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x98,
	.device_code              = 0xda,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 256LL*SZ_1M,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 20,
	.data_hold_in_ns          = 30,
	.address_setup_in_ns      = 0,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"TC58NVG1D4BFT00",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x45,
	.device_code              = 0xda,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 256LL*SZ_1M,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 20,
	.data_hold_in_ns          = 30,
	.address_setup_in_ns      = 0,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	NULL,
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x45,
	.device_code              = 0xdc,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 512LL*SZ_1M,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 20,
	.data_hold_in_ns          = 30,
	.address_setup_in_ns      = 0,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	NULL,
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x98,
	.device_code              = 0xd3,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 1LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 35,
	.data_hold_in_ns          = 30,
	.address_setup_in_ns      = 0,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"TH58NVG3D4xFT00",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x45,
	.device_code              = 0xd3,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 1LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 35,
	.data_hold_in_ns          = 20,
	.address_setup_in_ns      = 0,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	NULL,
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x98,
	.device_code              = 0xd5,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 2LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 35,
	.data_hold_in_ns          = 15,
	.address_setup_in_ns      = 0,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"TH58NVG4D4xFT00",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x45,
	.device_code              = 0xd5,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 2LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 35,
	.data_hold_in_ns          = 15,
	.address_setup_in_ns      = 0,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	NULL,
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x98,
	.device_code              = 0xdc,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 512LL*SZ_1M,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 20,
	.data_hold_in_ns          = 30,
	.address_setup_in_ns      = 0,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"TC58NVG2D4BFT00",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xec,
	.device_code              = 0xdc,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 512LL*SZ_1M,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 25,
	.data_hold_in_ns          = 15,
	.address_setup_in_ns      = 25,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"K9G4G08U0M",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xad,
	.device_code              = 0xdc,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 512LL*SZ_1M,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 45,
	.data_hold_in_ns          = 25,
	.address_setup_in_ns      = 50,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"HY27UT084G2M, HY27UU088G5M",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x20,
	.device_code              = 0xdc,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 512LL*SZ_1M,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 40,
	.data_hold_in_ns          = 20,
	.address_setup_in_ns      = 30,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"NAND04GW3C2AN1E",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xec,
	.device_code              = 0xd3,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 1LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 20,
	.data_hold_in_ns          = 15,
	.address_setup_in_ns      = 20,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"K9G8G08U0M, K9HAG08U1M",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xad,
	.device_code              = 0xd3,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 1LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 60,
	.data_hold_in_ns          = 30,
	.address_setup_in_ns      = 50,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"HY27UV08AG5M",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x2c,
	.device_code              = 0xd3,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 1LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 15,
	.data_hold_in_ns          = 15,
	.address_setup_in_ns      = 15,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"Intel JS29F08G08AAMiB1 and Micron MT29F8G08MAA; "
	"Intel JS29F08G08CAMiB1 and Micron MT29F16G08QAA",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xec,
	.device_code              = 0xd5,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 2LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 20,
	.data_hold_in_ns          = 15,
	.address_setup_in_ns      = 20,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"K9LAG08U0M K9HBG08U1M K9GAG08U0M",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x2c,
	.device_code              = 0xd5,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 2LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 15,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 15,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"Intel JS29F32G08FAMiB1 and Micron MT29F32G08TAA",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x2c,
	.device_code              = 0xdc,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 512LL*SZ_1M,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 20,
	.data_hold_in_ns          = 20,
	.address_setup_in_ns      = 20,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"MT29F4G08",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x89,
	.device_code              = 0xd3,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 1LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 15,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 15,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"JS29F08G08AAMiB2, JS29F08G08CAMiB2",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x89,
	.device_code              = 0xd5,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 2LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 15,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 15,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"JS29F32G08FAMiB2",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xad,
	.device_code              = 0xd5,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 2LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 15,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 20,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"HY27UW08CGFM",
	},
	{true}
};

/*
 * Type 7
 */
static struct nand_device_info nand_device_info_table_type_7[] __initdata = {
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x2c,
	.device_code              = 0xd3,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 1LL*SZ_1G,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 25,
	.data_hold_in_ns          = 15,
	.address_setup_in_ns      = 10,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"MT29F8G08FABWG",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x2c,
	.device_code              = 0xdc,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 512LL*SZ_1M,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 20,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 10,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"MT29F4G08AAA",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xec,
	.device_code              = 0xdc,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 512LL*SZ_1M,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 15,
	.data_hold_in_ns          = 12,
	.address_setup_in_ns      = 25,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"K9F4G08",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xec,
	.device_code              = 0xd3,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 1LL*SZ_1G,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 25,
	.data_hold_in_ns          = 15,
	.address_setup_in_ns      = 35,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"K9K8G08UXM, K9NBG08U5A, K9WAG08U1A",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xec,
	.device_code              = 0xd5,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 2LL*SZ_1G,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 15,
	.data_hold_in_ns          = 12,
	.address_setup_in_ns      = 25,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"K9WAG08UXM",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xec,
	.device_code              = 0xda,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 256LL*SZ_1M,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 20,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 20,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"K9F2G08U0A",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xec,
	.device_code              = 0xf1,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 128LL*SZ_1M,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 2*SZ_1K + 64,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 15,
	.data_hold_in_ns          = 12,
	.address_setup_in_ns      = 20,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"K9F1F08",
	},
	{true}
};

/*
 * Type 8
 */
static struct nand_device_info nand_device_info_table_type_8[] __initdata = {
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xec,
	.device_code              = 0xd5,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 2LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 4*SZ_1K + 128,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 15,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 20,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"K9GAG08U0M",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xec,
	.device_code              = 0xd7,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 4LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 4*SZ_1K + 128,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 15,
	.data_hold_in_ns          = 15,
	.address_setup_in_ns      = 25,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"K9LBG08U0M (32Gb), K9HCG08U1M (64Gb), K9MDG08U5M (128Gb)",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xad,
	.device_code              = 0xd5,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 2LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 4*SZ_1K + 128,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 20,
	.data_hold_in_ns          = 20,
	.address_setup_in_ns      = 20,
	.gpmi_sample_delay_in_ns  = 0,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"H27UAG, H27UBG",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xad,
	.device_code              = 0xd7,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 4LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 4*SZ_1K + 128,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 23,
	.data_hold_in_ns          = 20,
	.address_setup_in_ns      = 25,
	.gpmi_sample_delay_in_ns  = 0,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"H27UCG",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xad,
	.device_code              = 0xd3,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 8LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 4*SZ_1K + 128,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 15,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 20,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"H27U8G8T2B",
	},
	{true}
};

/*
 * Type 9
 */
static struct nand_device_info nand_device_info_table_type_9[] __initdata = {
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x98,
	.device_code              = 0xd3,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 1LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 4*SZ_1K + 218,
	.ecc_strength_in_bits     = 8,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 15,
	.data_hold_in_ns          = 15,
	.address_setup_in_ns      = 10,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"TC58NVG3D1DTG00",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x98,
	.device_code              = 0xd5,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 2LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 4*SZ_1K + 218,
	.ecc_strength_in_bits     = 8,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 15,
	.data_hold_in_ns          = 15,
	.address_setup_in_ns      = 10,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"TC58NVG4D1DTG00",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x98,
	.device_code              = 0xd7,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 4LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 4*SZ_1K + 218,
	.ecc_strength_in_bits     = 8,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 15,
	.data_hold_in_ns          = 15,
	.address_setup_in_ns      = 10,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"TH58NVG6D1DTG20",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x89,
	.device_code              = 0xd5,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 2LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 4*SZ_1K + 218,
	.ecc_strength_in_bits     = 8,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 10,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 15,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"JS29F16G08AAMC1, JS29F32G08CAMC1",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x2c,
	.device_code              = 0xd5,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 2LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 4*SZ_1K + 218,
	.ecc_strength_in_bits     = 8,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 15,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 15,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"MT29F16G08MAA, MT29F32G08QAA",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x2c,
	.device_code              = 0xd7,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 4LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 4*SZ_1K + 218,
	.ecc_strength_in_bits     = 8,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 15,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 15,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"MT29F64G08TAA (32Gb), MT29F32G08CBAAA (32Gb) MT29F64G08CFAAA (64Gb)",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x2c,
	.device_code              = 0xd9,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 8LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 4*SZ_1K + 218,
	.ecc_strength_in_bits     = 8,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 10,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 15,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"MT29F128G08CJAAA",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x89,
	.device_code              = 0xd7,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 4LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 4*SZ_1K + 218,
	.ecc_strength_in_bits     = 8,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 10,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 15,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"JSF64G08FAMC1",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xec,
	.device_code              = 0xd7,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 4LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 4*SZ_1K + 218,
	.ecc_strength_in_bits     = 8,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 20,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 25,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = 20,
	.tRLOH_in_ns              = 5,
	.tRHOH_in_ns              = 15,
	"K9LBG08U0D",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xec,
	.device_code              = 0xd5,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 2LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 4*SZ_1K + 218,
	.ecc_strength_in_bits     = 8,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 20,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 20,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"K9GAG08U0D, K9LBG08U1D, K9HCG08U5D",
	},
	{true}
};

/*
 * Type 10
 */
static struct nand_device_info nand_device_info_table_type_10[] __initdata = {
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xec,
	.device_code              = 0xd3,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 1LL*SZ_1G,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 4*SZ_1K + 128,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 15,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 20,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	NULL,
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xec,
	.device_code              = 0xd5,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 2LL*SZ_1G,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 4*SZ_1K + 128,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 25,
	.data_hold_in_ns          = 15,
	.address_setup_in_ns      = 30,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	"K9NCG08U5M",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xec,
	.device_code              = 0xd7,
	.cell_technology          = NAND_DEVICE_CELL_TECH_SLC,
	.chip_size_in_bytes       = 4LL*SZ_1G,
	.block_size_in_pages      = 64,
	.page_total_size_in_bytes = 4*SZ_1K + 128,
	.ecc_strength_in_bits     = 4,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 15,
	.data_hold_in_ns          = 15,
	.address_setup_in_ns      = 25,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = -1,
	.tRLOH_in_ns              = -1,
	.tRHOH_in_ns              = -1,
	NULL,
	},
	{true}
};

/*
 * Type 11
 */
static struct nand_device_info nand_device_info_table_type_11[] __initdata = {
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x98,
	.device_code              = 0xd7,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 4LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 8*SZ_1K + 376,
	.ecc_strength_in_bits     = 14,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 15,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 8,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = 20,
	.tRLOH_in_ns              = 5,
	.tRHOH_in_ns              = 25,
	"TC58NVG5D2ELAM8 (4GB), TH58NVG6D2ELAM8 (8GB)",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x98,
	.device_code              = 0xde,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 8LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 8*SZ_1K + 376,
	.ecc_strength_in_bits     = 14,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 15,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 8,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = 20,
	.tRLOH_in_ns              = 5,
	.tRHOH_in_ns              = 25,
	"TH58NVG7D2ELAM8",
	},
	{true}
};

/*
 * Type 15
 */
static struct nand_device_info nand_device_info_table_type_15[] __initdata = {
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xec,
	.device_code              = 0xd7,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 4LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 8*SZ_1K + 436,
	.ecc_strength_in_bits     = 16,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 20,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 25,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = 25,
	.tRLOH_in_ns              = 5,
	.tRHOH_in_ns              = 15,
	"K9GBG08U0M (4GB, 1CE); K9LCG08U1M (8GB, 2CE); K9HDG08U5M (16GB, 4CE)",
	},
	{true}
};

static struct nand_device_info nand_device_info_table_type_16[] __initdata = {
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xec,
	.device_code              = 0xd7,
	.is_ddr_ok		  = true,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 4LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 8*SZ_1K + 512,
	.ecc_strength_in_bits     = 24,
	.ecc_size_in_bytes        = 1024,
	.data_setup_in_ns         = 20,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 25,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = 25,
	.tRLOH_in_ns              = 5,
	.tRHOH_in_ns              = 15,
	"K9LCGD88X1M(8GB, 1CE); K9HDGD8X5M(8GB, 2CE); K9PFGD8X7M(16GB, 4CE)",
	},
	{true}
};
/*
 * BCH ECC12
 */
static struct nand_device_info nand_device_info_table_bch_ecc12[] __initdata = {
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xad,
	.device_code              = 0xd5,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 2LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 4*SZ_1K + 224,
	.ecc_strength_in_bits     = 12,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 15,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 20,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = 20,
	.tRLOH_in_ns              = 5,
	.tRHOH_in_ns              = 15,
	"H27UAG8T2ATR (2GB, 1CE)",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xad,
	.device_code              = 0xd7,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 4LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 4*SZ_1K + 224,
	.ecc_strength_in_bits     = 12,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 15,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 20,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = 20,
	.tRLOH_in_ns              = 5,
	.tRHOH_in_ns              = 15,
	"H27UBG8T2M (4GB, 1CE), H27UCG8UDM (8GB, 2CE), H27UDG8VEM (16GB, 4CE)",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0xad,
	.device_code              = 0xde,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 8LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 4*SZ_1K + 224,
	.ecc_strength_in_bits     = 12,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 15,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 20,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = 20,
	.tRLOH_in_ns              = 5,
	.tRHOH_in_ns              = 15,
	"H27UEG8YEM (32GB, 4CE)",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x2c,
	.device_code              = 0xd7,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 4LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 4*SZ_1K + 218,
	.ecc_strength_in_bits     = 12,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 10,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 15,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = 16,
	.tRLOH_in_ns              = 5,
	.tRHOH_in_ns              = 15,
	"MT29F32G08CBAAA (4GB, 1CE), MT29F64G08CFAAA (8GB, 2CE)",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x2c,
	.device_code              = 0xd9,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 8LL*SZ_1G,
	.block_size_in_pages      = 128,
	.page_total_size_in_bytes = 4*SZ_1K + 218,
	.ecc_strength_in_bits     = 12,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 10,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 15,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = 16,
	.tRLOH_in_ns              = 5,
	.tRHOH_in_ns              = 15,
	"MT29F128G08CJAAA (16GB, 2CE)",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x2c,
	.device_code              = 0x48,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 2LL*SZ_1G,
	.block_size_in_pages      = 256,
	.page_total_size_in_bytes = 4*SZ_1K + 224,
	.ecc_strength_in_bits     = 12,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 15,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 20,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = 20,
	.tRLOH_in_ns              = 5,
	.tRHOH_in_ns              = 15,
	"MT29F16G08CBABA (2GB, 1CE)",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x2c,
	.device_code              = 0x68,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 4LL*SZ_1G,
	.block_size_in_pages      = 256,
	.page_total_size_in_bytes = 4*SZ_1K + 224,
	.ecc_strength_in_bits     = 12,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 15,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 20,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = 20,
	.tRLOH_in_ns              = 5,
	.tRHOH_in_ns              = 15,
	"MT29F32G08CBABA (4GB, 1CE); "
	"MT29F64G08CEABA (8GB, 2CE); "
	"MT29F64G08CFABA (8GB, 2CE)",
	},
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x2c,
	.device_code              = 0x88,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 8LL*SZ_1G,
	.block_size_in_pages      = 256,
	.page_total_size_in_bytes = 4*SZ_1K + 224,
	.ecc_strength_in_bits     = 12,
	.ecc_size_in_bytes        = 512,
	.data_setup_in_ns         = 15,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 20,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = 20,
	.tRLOH_in_ns              = 5,
	.tRHOH_in_ns              = 15,
	"MT29F128G08CJABA (16GB, 2CE); "
	"MT29F128G08CKABA (16GB, 2CE); "
	"MT29F256G08CUABA (32GB, 4CE)",
	},
	{true}
};

/*
 * BCH ECC24
 */
static struct nand_device_info nand_device_info_table_bch_ecc24[] __initdata = {
	{
	.end_of_table             = false,
	.manufacturer_code        = 0x2c,
	.device_code              = 0x88,
	.is_ddr_ok		  = true,
	.cell_technology          = NAND_DEVICE_CELL_TECH_MLC,
	.chip_size_in_bytes       = 8LL * SZ_1G,
	.block_size_in_pages      = 256,
	.page_total_size_in_bytes = 8 * SZ_1K + 448,
	.ecc_strength_in_bits     = 24,
	.ecc_size_in_bytes        = 1024,
	.data_setup_in_ns         = 15,
	.data_hold_in_ns          = 10,
	.address_setup_in_ns      = 20,
	.gpmi_sample_delay_in_ns  = 6,
	.tREA_in_ns               = 20,
	.tRLOH_in_ns              = 5,
	.tRHOH_in_ns              = 15,
	"MT29F64G08CBAAA(8GB, 2CE) ",
	},
	{true}
};

/*
 * The following macros make it convenient to extract information from an ID
 * byte array. All these macros begin with the prefix "ID_".
 *
 * Macros of the form:
 *
 *         ID_GET_[<manufacturer>_[<modifier>_]]<field>
 *
 * extract the given field from an ID byte array. Macros of the form:
 *
 *         ID_[<manufacturer>_[<modifier>_]]<field>_<meaning>
 *
 * contain the value for the given field that has the given meaning.
 *
 * If the <manufacturer> appears, it means this macro represents a view of this
 * field that is specific to the given manufacturer.
 *
 * If the <modifier> appears, it means this macro represents a view of this
 * field that the given manufacturer applies only under specific conditions.
 *
 * Here is a simple example:
 *
 *         ID_PAGE_SIZE_CODE_2K
 *
 * This macro has the value of the "Page Size" field that indicates the page
 * size is 2K.
 *
 * A more complicated example:
 *
 *         ID_SAMSUNG_6_BYTE_PAGE_SIZE_CODE_8K  (0x2)
 *
 * This macro has the value of the "Page Size" field for Samsung parts that
 * indicates the page size is 8K. However, this interpretation is only correct
 * for devices that return 6 ID bytes.
 */

/* Byte 1 ------------------------------------------------------------------- */

#define ID_GET_BYTE_1(id)    ((id)[0])

#define ID_GET_MFR_CODE(id)  ID_GET_BYTE_1(id)

/* Byte 2 ------------------------------------------------------------------- */

#define ID_GET_BYTE_2(id)                           ((id)[1])

#define ID_GET_DEVICE_CODE(id)                      ID_GET_BYTE_2(id)
    #define ID_SAMSUNG_DEVICE_CODE_1_GBIT           (0xf1)
    #define ID_SAMSUNG_DEVICE_CODE_2_GBIT           (0xda)
    #define ID_HYNIX_DEVICE_CODE_ECC12              (0xd7)
    #define ID_HYNIX_DEVICE_CODE_ECC12_ODD	    (0xd5)
    #define ID_HYNIX_DEVICE_CODE_ECC12_LARGE        (0xde)
    #define ID_MICRON_DEVICE_CODE_ECC12             (0xd7) /* ECC12        */
    #define ID_MICRON_DEVICE_CODE_ECC12_LARGE       (0xd9) /* ECC12 8GB/CE */
    #define ID_MICRON_DEVICE_CODE_ECC12_2GB_PER_CE  (0x48) /* L63B  2GB/CE */
    #define ID_MICRON_DEVICE_CODE_ECC12_4GB_PER_CE  (0x68) /* L63B  4GB/CE */
    #define ID_MICRON_DEVICE_CODE_ECC12_8GB_PER_CE  (0x88) /* L63B  8GB/CE */
    #define ID_MICRON_DEVICE_CODE_ECC24_8GB_PER_CE  (0x88) /* L63B  8GB/CE */

/* Byte 3 ------------------------------------------------------------------- */

#define ID_GET_BYTE_3(id)               ((id)[2])

#define ID_GET_DIE_COUNT_CODE(id)       ((ID_GET_BYTE_3(id) >> 0) & 0x3)

#define ID_GET_CELL_TYPE_CODE(id)       ((ID_GET_BYTE_3(id) >> 2) & 0x3)
    #define ID_CELL_TYPE_CODE_SLC       (0x0) /* All others => MLC. */

#define ID_GET_SAMSUNG_SIMUL_PROG(id)   ((ID_GET_BYTE_3(id) >> 4) & 0x3)

#define ID_GET_MICRON_SIMUL_PROG(id)    ((ID_GET_BYTE_3(id) >> 4) & 0x3)

#define ID_GET_CACHE_PROGRAM(id)        ((ID_GET_BYTE_3(id) >> 7) & 0x1)

#define ID_ONFI_NAND_BYTE3		(0x04)
/* Byte 4 ------------------------------------------------------------------- */

#define ID_GET_BYTE_4(id)                       ((id)[3])
    #define ID_HYNIX_BYTE_4_ECC12_DEVICE        (0x25)

#define ID_GET_PAGE_SIZE_CODE(id)               ((ID_GET_BYTE_4(id) >> 0) & 0x3)
    #define ID_PAGE_SIZE_CODE_1K                (0x0)
    #define ID_PAGE_SIZE_CODE_2K                (0x1)
    #define ID_PAGE_SIZE_CODE_4K                (0x2)
    #define ID_PAGE_SIZE_CODE_8K                (0x3)
    #define ID_SAMSUNG_6_BYTE_PAGE_SIZE_CODE_8K (0x2)

#define ID_GET_OOB_SIZE_CODE(id)                ((ID_GET_BYTE_4(id) >> 2) & 0x1)

#define ID_GET_BLOCK_SIZE_CODE(id)              ((ID_GET_BYTE_4(id) >> 4) & 0x3)

/* Byte 5 ------------------------------------------------------------------- */

#define ID_GET_BYTE_5(id)                  ((id)[4])
    #define ID_MICRON_BYTE_5_ECC12         (0x84)

#define ID_GET_SAMSUNG_ECC_LEVEL_CODE(id)  ((ID_GET_BYTE_5(id) >> 4) & 0x7)
    #define ID_SAMSUNG_ECC_LEVEL_CODE_8    (0x03)
    #define ID_SAMSUNG_ECC_LEVEL_CODE_24   (0x05)

#define ID_GET_PLANE_COUNT_CODE(id)        ((ID_GET_BYTE_5(id) >> 2) & 0x3)

/* Byte 6 ------------------------------------------------------------------- */

#define ID_GET_BYTE_6(id)                        ((id)[5])
    #define ID_TOSHIBA_BYTE_6_PAGE_SIZE_CODE_8K  (0x54)
    #define ID_TOSHIBA_BYTE_6_PAGE_SIZE_CODE_4K  (0x13)

#define ID_GET_SAMSUNG_DEVICE_VERSION_CODE(id)   ((ID_GET_BYTE_6(id)>>0) & 0x7)
    #define ID_SAMSUNG_DEVICE_VERSION_CODE_40NM  (0x01)

#define ID_GET_TOGGLE_NAND(id)		((ID_GET_BYTE_6(id) >> 7) & 0x1)

/* -------------------------------------------------------------------------- */

void nand_device_print_info(struct nand_device_info *info)
{
	unsigned    i;
	const char  *mfr_name;
	const char  *cell_technology_name;
	uint64_t    chip_size;
	const char  *chip_size_units;
	unsigned    page_data_size_in_bytes;
	unsigned    page_oob_size_in_bytes;

	/* Check for nonsense. */

	if (!info)
		return;

	/* Prepare the manufacturer name. */

	mfr_name = "Unknown";

	for (i = 0; nand_manuf_ids[i].id; i++) {
		if (nand_manuf_ids[i].id == info->manufacturer_code) {
			mfr_name = nand_manuf_ids[i].name;
			break;
		}
	}

	/* Prepare the name of the cell technology. */

	switch (info->cell_technology) {
	case NAND_DEVICE_CELL_TECH_SLC:
		cell_technology_name = "SLC";
		break;
	case NAND_DEVICE_CELL_TECH_MLC:
		cell_technology_name = "MLC";
		break;
	default:
		cell_technology_name = "Unknown";
		break;
	}

	/* Prepare the chip size. */

	if ((info->chip_size_in_bytes >= SZ_1G) &&
					!(info->chip_size_in_bytes % SZ_1G)) {
		chip_size       = info->chip_size_in_bytes / ((uint64_t) SZ_1G);
		chip_size_units = "GiB";
	} else if ((info->chip_size_in_bytes >= SZ_1M) &&
					!(info->chip_size_in_bytes % SZ_1M)) {
		chip_size       = info->chip_size_in_bytes / ((uint64_t) SZ_1M);
		chip_size_units = "MiB";
	} else {
		chip_size       = info->chip_size_in_bytes;
		chip_size_units = "B";
	}

	/* Prepare the page geometry. */

	page_data_size_in_bytes = (1<<(fls(info->page_total_size_in_bytes)-1));
	page_oob_size_in_bytes  = info->page_total_size_in_bytes -
							page_data_size_in_bytes;

	/* Print the information. */

	printk(KERN_INFO "Manufacturer      : %s (0x%02x)\n",  mfr_name,
						info->manufacturer_code);
	printk(KERN_INFO "Device Code       : 0x%02x\n", info->device_code);
	printk(KERN_INFO "Cell Technology   : %s\n", cell_technology_name);
	printk(KERN_INFO "Chip Size         : %llu %s\n", chip_size,
							chip_size_units);
	printk(KERN_INFO "Pages per Block   : %u\n",
						info->block_size_in_pages);
	printk(KERN_INFO "Page Geometry     : %u+%u\n", page_data_size_in_bytes,
						page_oob_size_in_bytes);
	printk(KERN_INFO "ECC Strength      : %u bits\n",
						info->ecc_strength_in_bits);
	printk(KERN_INFO "ECC Size          : %u B\n", info->ecc_size_in_bytes);
	printk(KERN_INFO "Data Setup Time   : %u ns\n", info->data_setup_in_ns);
	printk(KERN_INFO "Data Hold Time    : %u ns\n", info->data_hold_in_ns);
	printk(KERN_INFO "Address Setup Time: %u ns\n",
						info->address_setup_in_ns);
	printk(KERN_INFO "GPMI Sample Delay : %u ns\n",
						info->gpmi_sample_delay_in_ns);
	if (info->tREA_in_ns >= 0)
		printk(KERN_INFO "tREA              : %u ns\n",
							info->tREA_in_ns);
	else
		printk(KERN_INFO "tREA              : Unknown\n");
	if (info->tREA_in_ns >= 0)
		printk(KERN_INFO "tRLOH             : %u ns\n",
							info->tRLOH_in_ns);
	else
		printk(KERN_INFO "tRLOH             : Unknown\n");
	if (info->tREA_in_ns >= 0)
		printk(KERN_INFO "tRHOH             : %u ns\n",
							info->tRHOH_in_ns);
	else
		printk(KERN_INFO "tRHOH             : Unknown\n");
	if (info->description)
		printk(KERN_INFO "Description       : %s\n", info->description);
	else
		printk(KERN_INFO "Description       : <None>\n");

}

static struct nand_device_info *nand_device_info_search(
	struct nand_device_info *table, uint8_t mfr_code, uint8_t device_code)
{

	for (; !table->end_of_table; table++) {
		if (table->manufacturer_code != mfr_code)
			continue;
		if (table->device_code != device_code)
			continue;
		return table;
	}

	return 0;

}

static struct nand_device_info * __init nand_device_info_fn_toshiba(const uint8_t id[])
{
	struct nand_device_info  *table;

	/* Check for an SLC device. */

	if (ID_GET_CELL_TYPE_CODE(id) == ID_CELL_TYPE_CODE_SLC) {
		/* Type 2 */
		return nand_device_info_search(nand_device_info_table_type_2,
				ID_GET_MFR_CODE(id), ID_GET_DEVICE_CODE(id));
	}

	/*
	 * Look for 8K page Toshiba MLC devices.
	 *
	 * The page size field in byte 4 can't be used because the field was
	 * redefined in the 8K parts so the value meaning "8K page" is the same
	 * as the value meaning "4K page" on the 4K page devices.
	 *
	 * The only identifiable difference between the 4K and 8K page Toshiba
	 * devices with a device code of 0xd7 is the undocumented 6th ID byte.
	 * The 4K device returns a value of 0x13 and the 8K a value of 0x54.
	 * Toshiba has verified that this is an acceptable method to distinguish
	 * the two device families.
	 */

	if (ID_GET_BYTE_6(id) == ID_TOSHIBA_BYTE_6_PAGE_SIZE_CODE_8K) {
		/* Type 11 */
		table = nand_device_info_table_type_11;
	} else if (ID_GET_PAGE_SIZE_CODE(id) == ID_PAGE_SIZE_CODE_4K) {
		/* Type 9 */
		table = nand_device_info_table_type_9;
	} else {
		/* Large MLC */
		table = nand_device_info_table_large_mlc;
	}

	return nand_device_info_search(table, ID_GET_MFR_CODE(id),
							ID_GET_DEVICE_CODE(id));

}

static struct nand_device_info * __init nand_device_info_fn_samsung(const uint8_t id[])
{
	struct nand_device_info  *table;

	/* Check for an MLC device. */

	if (ID_GET_CELL_TYPE_CODE(id) != ID_CELL_TYPE_CODE_SLC) {

		/* Is this a Samsung 8K Page MLC device with 16 bit ECC? */
		if ((ID_GET_SAMSUNG_ECC_LEVEL_CODE(id) ==
					ID_SAMSUNG_ECC_LEVEL_CODE_24) &&
		    (ID_GET_PAGE_SIZE_CODE(id) ==
					ID_SAMSUNG_6_BYTE_PAGE_SIZE_CODE_8K)) {
			if (ID_GET_TOGGLE_NAND(id))
				table = nand_device_info_table_type_16;
			else
				/* Type 15 */
				table = nand_device_info_table_type_15;
		}
		/* Is this a Samsung 42nm ECC8 device with a 6 byte ID? */
		else if ((ID_GET_SAMSUNG_ECC_LEVEL_CODE(id) ==
					ID_SAMSUNG_ECC_LEVEL_CODE_8) &&
			(ID_GET_SAMSUNG_DEVICE_VERSION_CODE(id) ==
					ID_SAMSUNG_DEVICE_VERSION_CODE_40NM)) {
			/* Type 9 */
			table = nand_device_info_table_type_9;
		} else if (ID_GET_PAGE_SIZE_CODE(id) == ID_PAGE_SIZE_CODE_4K) {
			/* Type 8 */
			table = nand_device_info_table_type_8;
		} else {
			/* Large MLC */
			table = nand_device_info_table_large_mlc;
		}

	} else {

		/* Check the page size first. */
		if (ID_GET_PAGE_SIZE_CODE(id) == ID_PAGE_SIZE_CODE_4K) {
			/* Type 10 */
			table = nand_device_info_table_type_10;
		}
		/* Check the chip size. */
		else if (ID_GET_DEVICE_CODE(id) ==
						ID_SAMSUNG_DEVICE_CODE_1_GBIT) {
			if (!ID_GET_CACHE_PROGRAM(id)) {
				/*
				 * 128 MiB Samsung chips without cache program
				 * are Type 7.
				 *
				 * The K9F1G08U0B does not support multi-plane
				 * program, so the if statement below cannot be
				 * used to identify it.
				 */
				table = nand_device_info_table_type_7;

			} else {
				/* Smaller sizes are Type 2 by default. */
				table = nand_device_info_table_type_2;
			}
		} else {
			/* Check number of simultaneously programmed pages. */
			if (ID_GET_SAMSUNG_SIMUL_PROG(id) &&
						ID_GET_PLANE_COUNT_CODE(id)) {
				/* Type 7 */
				table = nand_device_info_table_type_7;
			} else {
				/* Type 2 */
				table = nand_device_info_table_type_2;
			}

		}

	}

	return nand_device_info_search(table, ID_GET_MFR_CODE(id),
							ID_GET_DEVICE_CODE(id));

}

static struct nand_device_info * __init nand_device_info_fn_stmicro(const uint8_t id[])
{
	struct nand_device_info  *table;

	/* Check for an SLC device. */

	if (ID_GET_CELL_TYPE_CODE(id) == ID_CELL_TYPE_CODE_SLC)
		/* Type 2 */
		table = nand_device_info_table_type_2;
	else
		/* Large MLC */
		table = nand_device_info_table_large_mlc;

	return nand_device_info_search(table, ID_GET_MFR_CODE(id),
							ID_GET_DEVICE_CODE(id));

}

static struct nand_device_info * __init nand_device_info_fn_hynix(const uint8_t id[])
{
	struct nand_device_info  *table;

	/* Check for an SLC device. */

	if (ID_GET_CELL_TYPE_CODE(id) == ID_CELL_TYPE_CODE_SLC) {
		/* Type 2 */
		return nand_device_info_search(nand_device_info_table_type_2,
				ID_GET_MFR_CODE(id), ID_GET_DEVICE_CODE(id));
	}

	/*
	 * Check for ECC12 devices.
	 *
	 * We look at the 4th ID byte to distinguish some Hynix ECC12 devices
	 * from the similar ECC8 part. For example H27UBG8T2M (ECC12) 4th byte
	 * is 0x25, whereas H27UDG8WFM (ECC8) 4th byte is 0xB6.
	 */

	if ((((ID_GET_DEVICE_CODE(id) == ID_HYNIX_DEVICE_CODE_ECC12) ||
		(ID_GET_DEVICE_CODE(id) == ID_HYNIX_DEVICE_CODE_ECC12_ODD)) &&
			ID_GET_BYTE_4(id) == ID_HYNIX_BYTE_4_ECC12_DEVICE) ||
	    (ID_GET_DEVICE_CODE(id) == ID_HYNIX_DEVICE_CODE_ECC12_LARGE)) {
		/* BCH ECC 12 */
		table = nand_device_info_table_bch_ecc12;
	} else if (ID_GET_PAGE_SIZE_CODE(id) == ID_PAGE_SIZE_CODE_4K) {
		/*
		 * So far, all other Samsung and Hynix 4K page devices are
		 * Type 8.
		 */
		table = nand_device_info_table_type_8;
	} else
		/* Large MLC */
		table = nand_device_info_table_large_mlc;

	return nand_device_info_search(table, ID_GET_MFR_CODE(id),
							ID_GET_DEVICE_CODE(id));

}

static struct nand_device_info * __init nand_device_info_fn_micron(const uint8_t id[])
{
	struct nand_device_info  *table;

	/* Check for an SLC device. */

	if (ID_GET_CELL_TYPE_CODE(id) == ID_CELL_TYPE_CODE_SLC) {

		/* Check number of simultaneously programmed pages. */

		if (ID_GET_MICRON_SIMUL_PROG(id)) {
			/* Type 7 */
			table = nand_device_info_table_type_7;
		} else {
			/* Zero simultaneously programmed pages means Type 2. */
			table = nand_device_info_table_type_2;
		}

		return nand_device_info_search(table, ID_GET_MFR_CODE(id),
							ID_GET_DEVICE_CODE(id));

	}

	if (ID_GET_DEVICE_CODE(id) == ID_MICRON_DEVICE_CODE_ECC24_8GB_PER_CE
		&& ID_GET_BYTE_3(id) == ID_ONFI_NAND_BYTE3) {
		/* BCH ECC 24 */
		table = nand_device_info_table_bch_ecc24;
	} else
	/*
	 * We look at the 5th ID byte to distinguish some Micron ECC12 NANDs
	 * from the similar ECC8 part.
	 *
	 * For example MT29F64G08CFAAA (ECC12) 5th byte is 0x84, whereas
	 * MT29F64G08TAA (ECC8) 5th byte is 0x78.
	 *
	 * We also have a special case for the Micron L63B family
	 * (256 page/block), which has unique device codes but no ID fields that
	 * can easily be used to distinguish the family.
	 */

	if ((ID_GET_DEVICE_CODE(id) == ID_MICRON_DEVICE_CODE_ECC12 &&
				ID_GET_BYTE_5(id) == ID_MICRON_BYTE_5_ECC12)  ||
	   (ID_GET_DEVICE_CODE(id) == ID_MICRON_DEVICE_CODE_ECC12_LARGE)      ||
	   (ID_GET_DEVICE_CODE(id) == ID_MICRON_DEVICE_CODE_ECC12_2GB_PER_CE) ||
	   (ID_GET_DEVICE_CODE(id) == ID_MICRON_DEVICE_CODE_ECC12_4GB_PER_CE) ||
	   (ID_GET_DEVICE_CODE(id) == ID_MICRON_DEVICE_CODE_ECC12_8GB_PER_CE)) {
		/* BCH ECC 12 */
		table = nand_device_info_table_bch_ecc12;
	} else if (ID_GET_PAGE_SIZE_CODE(id) == ID_PAGE_SIZE_CODE_4K) {
		/* Toshiba devices with 4K pages are Type 9. */
		table = nand_device_info_table_type_9;
	} else {
		/* Large MLC */
		table = nand_device_info_table_large_mlc;
	}

	return nand_device_info_search(table, ID_GET_MFR_CODE(id),
							ID_GET_DEVICE_CODE(id));

}

static struct nand_device_info * __init nand_device_info_fn_sandisk(const uint8_t id[])
{
	struct nand_device_info  *table;

	if (ID_GET_CELL_TYPE_CODE(id) != ID_CELL_TYPE_CODE_SLC) {
		/* Large MLC */
		table = nand_device_info_table_large_mlc;
	} else {
		/* Type 2 */
		table = nand_device_info_table_type_2;
	}

	return nand_device_info_search(table, ID_GET_MFR_CODE(id),
							ID_GET_DEVICE_CODE(id));

}

static struct nand_device_info * __init nand_device_info_fn_intel(const uint8_t id[])
{
	struct nand_device_info  *table;

	/* Check for an SLC device. */

	if (ID_GET_CELL_TYPE_CODE(id) == ID_CELL_TYPE_CODE_SLC) {
		/* Type 2 */
		return nand_device_info_search(nand_device_info_table_type_2,
				ID_GET_MFR_CODE(id), ID_GET_DEVICE_CODE(id));
	}

	if (ID_GET_PAGE_SIZE_CODE(id) == ID_PAGE_SIZE_CODE_4K) {
		/* Type 9 */
		table = nand_device_info_table_type_9;
	} else {
		/* Large MLC */
		table = nand_device_info_table_large_mlc;
	}

	return nand_device_info_search(table, ID_GET_MFR_CODE(id),
							ID_GET_DEVICE_CODE(id));

}

/**
 * struct nand_device_type_info - Information about a NAND Flash type.
 *
 * @name:   A human-readable name for this type.
 * @table:  The device info table for this type.
 */

struct nand_device_type_info {
	struct nand_device_info  *table;
	const char               *name;
};

/*
 * A table that maps manufacturer IDs to device information tables.
 */

static struct nand_device_type_info  nand_device_type_directory[] __initdata = {
	{nand_device_info_table_type_2,    "Type 2"   },
	{nand_device_info_table_large_mlc, "Large MLC"},
	{nand_device_info_table_type_7,    "Type 7"   },
	{nand_device_info_table_type_8,    "Type 8"   },
	{nand_device_info_table_type_9,    "Type 9"   },
	{nand_device_info_table_type_10,   "Type 10"  },
	{nand_device_info_table_type_11,   "Type 11"  },
	{nand_device_info_table_type_15,   "Type 15"  },
	{nand_device_info_table_type_16,   "Type 16"  },
	{nand_device_info_table_bch_ecc12, "BCH ECC12"},
	{nand_device_info_table_bch_ecc24, "BCH ECC24"},
	{0, 0},
};

/**
 * struct nand_device_mfr_info - Information about a NAND Flash manufacturer.
 *
 * @id:     The value of the first NAND Flash ID byte, which identifies the
 *          manufacturer.
 * @fn:     A pointer to a function to use for identifying devices from the
 *          given manufacturer.
 */

struct nand_device_mfr_info {
	uint8_t                  id;
	struct nand_device_info  *(*fn)(const uint8_t id[]);
};

/*
 * A table that maps manufacturer IDs to device information tables.
 */

static struct nand_device_mfr_info  nand_device_mfr_directory[] __initdata = {
	{
	.id = NAND_MFR_TOSHIBA,
	.fn = nand_device_info_fn_toshiba,
	},
	{
	.id = NAND_MFR_SAMSUNG,
	.fn = nand_device_info_fn_samsung,
	},
	{
	.id = NAND_MFR_FUJITSU,
	.fn = 0,
	},
	{
	.id = NAND_MFR_NATIONAL,
	.fn = 0,
	},
	{
	.id = NAND_MFR_RENESAS,
	.fn = 0,
	},
	{
	.id = NAND_MFR_STMICRO,
	.fn = nand_device_info_fn_stmicro,
	},
	{
	.id = NAND_MFR_HYNIX,
	.fn = nand_device_info_fn_hynix,
	},
	{
	.id = NAND_MFR_MICRON,
	.fn = nand_device_info_fn_micron,
	},
	{
	.id = NAND_MFR_AMD,
	.fn = 0,
	},
	{
	.id = NAND_MFR_SANDISK,
	.fn = nand_device_info_fn_sandisk,
	},
	{
	.id = NAND_MFR_INTEL,
	.fn = nand_device_info_fn_intel,
	},
	{0, 0}
};

/**
 * nand_device_info_test_table - Validate a device info table.
 *
 * This function runs tests on the given device info table to check that it
 * meets the current assumptions.
 */

static void __init nand_device_info_test_table(
			struct nand_device_info *table, const char *name)
{
	unsigned  i;
	unsigned  j;
	uint8_t   mfr_code;
	uint8_t   device_code;

	/* Loop over entries in this table. */

	for (i = 0; !table[i].end_of_table; i++) {

		/* Get discriminating attributes of the current device. */

		mfr_code    = table[i].manufacturer_code;
		device_code = table[i].device_code;

		/* Compare with the remaining devices in this table. */

		for (j = i + 1; !table[j].end_of_table; j++) {
			if ((mfr_code    == table[j].manufacturer_code) &&
			    (device_code == table[j].device_code))
				goto error;
		}

	}

	return;

error:

	printk(KERN_EMERG
		"\n== NAND Flash device info table failed validity check ==\n");

	printk(KERN_EMERG "\nDevice Info Table: %s\n", name);
	printk(KERN_EMERG "\nTable Index %u\n", i);
	nand_device_print_info(table + i);
	printk(KERN_EMERG "\nTable Index %u\n", j);
	nand_device_print_info(table + j);
	printk(KERN_EMERG "\n");

	BUG();

}

/**
 * nand_device_info_test_data - Test the NAND Flash device data.
 */

static void __init nand_device_info_test_data(void)
{

	unsigned  i;

	for (i = 0; nand_device_type_directory[i].name; i++) {
		nand_device_info_test_table(
					nand_device_type_directory[i].table,
					nand_device_type_directory[i].name);
	}

}

struct nand_device_info * __init nand_device_get_info(const uint8_t id[])
{
	unsigned                 i;
	uint8_t                  mfr_id = ID_GET_MFR_CODE(id);
	struct nand_device_info  *(*fn)(const uint8_t id[]) = 0;

	/* Test the data. */

	nand_device_info_test_data();

	/* Look for information about this manufacturer. */

	for (i = 0; nand_device_mfr_directory[i].id; i++) {
		if (nand_device_mfr_directory[i].id == mfr_id) {
			fn = nand_device_mfr_directory[i].fn;
			break;
		}
	}

	if (!fn)
		return 0;

	/*
	 * If control arrives here, we found both a table of device information,
	 * and a function we can use to identify the current device. Attempt to
	 * identify the device and return the result.
	 */

	return fn(id);

}
