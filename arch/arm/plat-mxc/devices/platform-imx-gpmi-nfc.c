/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
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
#include <linux/mtd/gpmi-nand.h>
#include <asm/sizes.h>
#include <mach/hardware.h>
#include <mach/devices-common.h>

#ifdef CONFIG_SOC_IMX50
struct platform_device *__init
imx_add_gpmi(const struct gpmi_nfc_platform_data *platform_data)
{
	struct resource res[] = {
	{ /* GPMI */
		 .name  = GPMI_NFC_GPMI_REGS_ADDR_RES_NAME,
		 .flags = IORESOURCE_MEM,
		 .start = MX50_GPMI_BASE_ADDR,
		 .end   = MX50_GPMI_BASE_ADDR + SZ_8K - 1,
	}, {
		 .name  = GPMI_NFC_GPMI_INTERRUPT_RES_NAME,
		 .flags = IORESOURCE_IRQ,
		 .start = MX50_INT_RAWNAND_GPMI,
		 .end   = MX50_INT_RAWNAND_GPMI,
	}, { /* BCH */
		 .name  = GPMI_NFC_BCH_REGS_ADDR_RES_NAME,
		 .flags = IORESOURCE_MEM,
		 .start = MX50_BCH_BASE_ADDR,
		 .end   = MX50_BCH_BASE_ADDR + SZ_8K - 1,
	}, {
		 .name  = GPMI_NFC_BCH_INTERRUPT_RES_NAME,
		 .flags = IORESOURCE_IRQ,
		 .start = MX50_INT_RAWNAND_BCH,
		 .end   = MX50_INT_RAWNAND_BCH,
	}, { /* DMA */
		 .name  = GPMI_NFC_DMA_CHANNELS_RES_NAME,
		 .flags = IORESOURCE_DMA,
		 .start	= MX50_DMA_CHANNEL_AHB_APBH_GPMI0,
		 .end	= MX50_DMA_CHANNEL_AHB_APBH_GPMI7,
	}, {
		 .name  = GPMI_NFC_DMA_INTERRUPT_RES_NAME,
		 .flags = IORESOURCE_IRQ,
		 .start = MX50_INT_APBHDMA_CHAN0,
		 .end   = MX50_INT_APBHDMA_CHAN7,
	}, };

	return imx_add_platform_device_dmamask("imx50-gpmi-nfc", 0,
			res, ARRAY_SIZE(res), platform_data,
			sizeof(*platform_data), DMA_BIT_MASK(32));
}
#endif /* ifdef CONFIG_SOC_IMX50 */

#ifdef CONFIG_SOC_IMX6Q
struct platform_device *__init
imx_add_gpmi(const struct gpmi_nand_platform_data *platform_data)
{
	struct resource res[] = {
	{ /* GPMI */
		 .name  = GPMI_NAND_GPMI_REGS_ADDR_RES_NAME,
		 .flags = IORESOURCE_MEM,
		 .start = MX6Q_GPMI_BASE_ADDR,
		 .end   = MX6Q_GPMI_BASE_ADDR + SZ_8K - 1,
	}, {
		 .name  = GPMI_NAND_GPMI_INTERRUPT_RES_NAME,
		 .flags = IORESOURCE_IRQ,
		 .start	= MXC_INT_RAWNAND_GPMI,
		 .end	= MXC_INT_RAWNAND_GPMI,
	}, { /* BCH */
		 .name  = GPMI_NAND_BCH_REGS_ADDR_RES_NAME,
		 .flags = IORESOURCE_MEM,
		 .start = MX6Q_BCH_BASE_ADDR,
		 .end   = MX6Q_BCH_BASE_ADDR + SZ_8K - 1,
	}, {
		 .name  = GPMI_NAND_BCH_INTERRUPT_RES_NAME,
		 .flags = IORESOURCE_IRQ,
		 .start	= MXC_INT_RAWNAND_BCH,
		 .end	= MXC_INT_RAWNAND_BCH,
	}, { /* DMA */
		 .name  = GPMI_NAND_DMA_CHANNELS_RES_NAME,
		 .flags = IORESOURCE_DMA,
		 .start	= MX6Q_DMA_CHANNEL_AHB_APBH_GPMI0,
		 .end	= MX6Q_DMA_CHANNEL_AHB_APBH_GPMI7,
	}, {
		 .name  = GPMI_NAND_DMA_INTERRUPT_RES_NAME,
		 .flags = IORESOURCE_IRQ,
		 .start	= MXC_INT_APBHDMA_DMA,
		 .end	= MXC_INT_APBHDMA_DMA,
	}, };

	return imx_add_platform_device_dmamask("imx6q-gpmi-nand", 0,
			res, ARRAY_SIZE(res), platform_data,
			sizeof(*platform_data), DMA_BIT_MASK(32));
}
#endif
