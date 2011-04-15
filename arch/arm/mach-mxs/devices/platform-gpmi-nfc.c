/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <asm/sizes.h>
#include <mach/mx23.h>
#include <mach/mx28.h>
#include <mach/devices-common.h>

#define RES_MEM(soc, _id, _s, _n)				\
	{							\
		.start = soc ##_## _id ## _BASE_ADDR,		\
		.end   = soc ##_## _id ## _BASE_ADDR + (_s) - 1,\
		.name  = (_n),					\
		.flags = IORESOURCE_MEM,			\
	}

#define RES_IRQ(soc, _id, _n)			\
	{					\
		.start = soc ##_INT_## _id,	\
		.end   = soc ##_INT_## _id,	\
		.name  = (_n),			\
		.flags = IORESOURCE_IRQ,	\
	}

#define RES_DMA(soc, _i_s, _i_e, _n)		\
	{					\
		.start = soc ##_## _i_s,	\
		.end   = soc ##_## _i_e,	\
		.name  = (_n),			\
		.flags = IORESOURCE_DMA,	\
	}

#ifdef CONFIG_SOC_IMX23
const struct mxs_gpmi_nfc_data mx23_gpmi_nfc_data __initconst = {
	.devid = "imx23-gpmi-nfc",
	.res = {
		/* GPMI */
		RES_MEM(MX23, GPMI, SZ_8K, GPMI_NFC_GPMI_REGS_ADDR_RES_NAME),
		RES_IRQ(MX23, GPMI_ATTENTION, GPMI_NFC_GPMI_INTERRUPT_RES_NAME),
		/* BCH */
		RES_MEM(MX23, BCH, SZ_8K, GPMI_NFC_BCH_REGS_ADDR_RES_NAME),
		RES_IRQ(MX23, BCH, GPMI_NFC_BCH_INTERRUPT_RES_NAME),
		/* DMA */
		RES_DMA(MX23, DMA_GPMI0, DMA_GPMI3,
					GPMI_NFC_DMA_CHANNELS_RES_NAME),
		RES_IRQ(MX23, GPMI_DMA, GPMI_NFC_DMA_INTERRUPT_RES_NAME),
	},
};
#endif

#ifdef CONFIG_SOC_IMX28
const struct mxs_gpmi_nfc_data mx28_gpmi_nfc_data __initconst = {
	.devid = "imx28-gpmi-nfc",
	.res = {
		/* GPMI */
		RES_MEM(MX28, GPMI, SZ_8K, GPMI_NFC_GPMI_REGS_ADDR_RES_NAME),
		RES_IRQ(MX28, GPMI, GPMI_NFC_GPMI_INTERRUPT_RES_NAME),
		/* BCH */
		RES_MEM(MX28, BCH, SZ_8K, GPMI_NFC_BCH_REGS_ADDR_RES_NAME),
		RES_IRQ(MX28, BCH, GPMI_NFC_BCH_INTERRUPT_RES_NAME),
		/* DMA */
		RES_DMA(MX28, DMA_GPMI0, DMA_GPMI7,
					GPMI_NFC_DMA_CHANNELS_RES_NAME),
		RES_IRQ(MX28, GPMI_DMA, GPMI_NFC_DMA_INTERRUPT_RES_NAME),
	},
};
#endif

struct platform_device *__init
mxs_add_gpmi_nfc(const struct gpmi_nfc_platform_data *pdata,
		const struct mxs_gpmi_nfc_data *data)
{
	return mxs_add_platform_device_dmamask(data->devid, -1,
				data->res, RES_SIZE,
				pdata, sizeof(*pdata), DMA_BIT_MASK(32));
}
