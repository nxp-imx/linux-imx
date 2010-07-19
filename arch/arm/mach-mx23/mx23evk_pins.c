/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <mach/pinctrl.h>

#include "mx23_pins.h"

static struct pin_desc mx23evk_fixed_pins[] = {
	{
	 .name = "DUART.RX",
	 .id = PINID_PWM0,
	 .fun = PIN_FUN3,
	 },
	{
	 .name = "DUART.TX",
	 .id = PINID_PWM1,
	 .fun = PIN_FUN3,
	 },
#ifdef CONFIG_MXS_AUART1_DEVICE_ENABLE
	{
	 .name  = "AUART1.RX",
	 .id    = PINID_AUART1_RX,
	 .fun   = PIN_FUN1,
	 },
	{
	 .name  = "AUART1.TX",
	 .id    = PINID_AUART1_TX,
	 .fun   = PIN_FUN1,
	 },
	{
	 .name  = "AUART1.CTS",
	 .id    = PINID_AUART1_CTS,
	 .fun   = PIN_FUN1,
	 },
	{
	 .name  = "AUART1.RTS",
	 .id    = PINID_AUART1_RTS,
	 .fun   = PIN_FUN1,
	 },
#endif

#ifdef CONFIG_MXS_AUART2_DEVICE_ENABLE
	{
	 .name  = "AUART2.RX",
	 .id    = PINID_GPMI_D14,
	 .fun   = PIN_FUN2,
	},
	{
	 .name  = "AUART2.TX",
	 .id    = PINID_GPMI_D15,
	 .fun   = PIN_FUN2,
	},
	{
	 .name  = "AUART2.CTS",
	 .id    = PINID_ROTARYB,
	 .fun   = PIN_FUN2,
	},
	{
	 .name  = "AUART2.RTS",
	 .id    = PINID_ROTARYA,
	 .fun   = PIN_FUN2,
	},
#endif
#if defined(CONFIG_I2C_MXS) || \
	defined(CONFIG_I2C_MXS_MODULE)
	{
	 .name = "I2C_SCL",
	 .id = PINID_I2C_SCL,
	 .fun = PIN_FUN1,
	 .strength = PAD_4MA,
	 .voltage = PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name = "I2C_SDA",
	 .id = PINID_I2C_SDA,
	 .fun = PIN_FUN1,
	 .strength = PAD_4MA,
	 .voltage = PAD_3_3V,
	 .drive	= 1,
	 },
#endif
#if defined(CONFIG_FB_MXS) || defined(CONFIG_FB_MXS_MODULE)
	{
	 .name  = "LCD_D00",
	 .id	= PINID_LCD_D00,
	 .fun	= PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name  = "LCD_D01",
	 .id	= PINID_LCD_D01,
	 .fun	= PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name  = "LCD_D02",
	 .id	= PINID_LCD_D02,
	 .fun	= PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name  = "LCD_D03",
	 .id	= PINID_LCD_D03,
	 .fun	= PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name  = "LCD_D04",
	 .id	= PINID_LCD_D04,
	 .fun	= PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name  = "LCD_D05",
	 .id	= PINID_LCD_D05,
	 .fun	= PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name  = "LCD_D06",
	 .id	= PINID_LCD_D06,
	 .fun	= PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name  = "LCD_D07",
	 .id	= PINID_LCD_D07,
	 .fun	= PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name  = "LCD_D08",
	 .id	= PINID_LCD_D08,
	 .fun	= PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name  = "LCD_D09",
	 .id	= PINID_LCD_D09,
	 .fun	= PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name  = "LCD_D10",
	 .id	= PINID_LCD_D10,
	 .fun	= PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name  = "LCD_D11",
	 .id	= PINID_LCD_D11,
	 .fun	= PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name  = "LCD_D12",
	 .id	= PINID_LCD_D12,
	 .fun	= PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name  = "LCD_D13",
	 .id	= PINID_LCD_D13,
	 .fun	= PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name  = "LCD_D14",
	 .id	= PINID_LCD_D14,
	 .fun	= PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name  = "LCD_D15",
	 .id	= PINID_LCD_D15,
	 .fun	= PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name  = "LCD_D16",
	 .id	= PINID_LCD_D16,
	 .fun	= PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name  = "LCD_D17",
	 .id	= PINID_LCD_D17,
	 .fun	= PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name  = "LCD_D18",
	 .id	= PINID_GPMI_D08,
	 .fun	= PIN_FUN2,
	 .strength = PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name  = "LCD_D19",
	 .id	= PINID_GPMI_D09,
	 .fun	= PIN_FUN2,
	 .strength = PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name  = "LCD_D20",
	 .id	= PINID_GPMI_D10,
	 .fun	= PIN_FUN2,
	 .strength = PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name  = "LCD_D21",
	 .id	= PINID_GPMI_D11,
	 .fun	= PIN_FUN2,
	 .strength = PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name  = "LCD_D22",
	 .id	= PINID_GPMI_D12,
	 .fun	= PIN_FUN2,
	 .strength = PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name  = "LCD_D23",
	 .id	= PINID_GPMI_D13,
	 .fun	= PIN_FUN2,
	 .strength = PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name = "LCD_RESET",
	 .id = PINID_LCD_RESET,
	 .fun = PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage = PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name = "LCD_VSYNC",
	 .id   = PINID_LCD_VSYNC,
	 .fun  = PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage = PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name = "LCD_HSYNC",
	 .id = PINID_LCD_HSYNC,
	 .fun = PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage = PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name = "LCD_ENABLE",
	 .id = PINID_LCD_ENABLE,
	 .fun = PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage = PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name = "LCD_DOTCLK",
	 .id = PINID_LCD_DOTCK,
	 .fun = PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage = PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name = "LCD_BACKLIGHT",
	 .id = PINID_PWM2,
	 .fun = PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage = PAD_3_3V,
	 .drive	= 1,
	 },
#endif

#if defined(CONFIG_FEC) || defined(CONFIG_FEC_MODULE)
	{
	 .name = "ENET0_MDC",
	 .id = PINID_ENET0_MDC,
	 .fun = PIN_FUN1,
	 .strength = PAD_8MA,
	 .pull = 1,
	 .pullup = 1,
	 .voltage = PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name = "ENET0_MDIO",
	 .id = PINID_ENET0_MDIO,
	 .fun = PIN_FUN1,
	 .strength = PAD_8MA,
	 .pull = 1,
	 .pullup = 1,
	 .voltage = PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name = "ENET0_RX_EN",
	 .id = PINID_ENET0_RX_EN,
	 .fun = PIN_FUN1,
	 .strength = PAD_8MA,
	 .pull = 1,
	 .pullup = 1,
	 .voltage = PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name = "ENET0_RXD0",
	 .id = PINID_ENET0_RXD0,
	 .fun = PIN_FUN1,
	 .strength = PAD_8MA,
	 .pull = 1,
	 .pullup = 1,
	 .voltage = PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name = "ENET0_RXD1",
	 .id = PINID_ENET0_RXD1,
	 .fun = PIN_FUN1,
	 .strength = PAD_8MA,
	 .pull = 1,
	 .pullup = 1,
	 .voltage = PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name = "ENET0_TX_EN",
	 .id = PINID_ENET0_TX_EN,
	 .fun = PIN_FUN1,
	 .strength = PAD_8MA,
	 .pull = 1,
	 .pullup = 1,
	 .voltage = PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name = "ENET0_TXD0",
	 .id = PINID_ENET0_TXD0,
	 .fun = PIN_FUN1,
	 .strength = PAD_8MA,
	 .pull = 1,
	 .pullup = 1,
	 .voltage = PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name = "ENET0_TXD1",
	 .id = PINID_ENET0_TXD1,
	 .fun = PIN_FUN1,
	 .strength = PAD_8MA,
	 .pull = 1,
	 .pullup = 1,
	 .voltage = PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name = "ENET_CLK",
	 .id = PINID_ENET_CLK,
	 .fun = PIN_FUN1,
	 .strength = PAD_8MA,
	 .pull = 1,
	 .pullup = 1,
	 .voltage = PAD_3_3V,
	 .drive	= 1,
	 },
#endif
#if defined(CONFIG_USB_OTG)
	{
	 .name = "USB_OTG_ID",
	 .id   = PINID_ROTARYA,
	 .fun  = PIN_GPIO,
	 .pull = 1,
	 .pullup = 1,
	 .voltage = PAD_3_3V,
	},
#endif
#if defined(CONFIG_SND_SOC_MXS_SPDIF) || \
       defined(CONFIG_SND_SOC_MXS_SPDIF_MODULE)
	{
	 .name		= "SPDIF",
	 .id		= PINID_ROTARYA,
	 .fun		= PIN_FUN3,
	 .strength	= PAD_12MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 1,
	 .drive 	= 1,
	 .pull 		= 1,
	},
#endif

#if defined(CONFIG_MTD_NAND_GPMI_NFC) || \
       defined(CONFIG_MTD_NAND_GPMI_NFC_MODULE)
	{
	 .name     = "GPMI D0",
	 .id       = PINID_GPMI_D00,
	 .fun      = PIN_FUN1,
	 .strength = PAD_4MA,
	 .voltage  = PAD_3_3V,
	 .pullup   = 0,
	 .drive    = !0
	 },
	{
	.name     = "GPMI D1",
	.id       = PINID_GPMI_D01,
	.fun      = PIN_FUN1,
	.strength = PAD_4MA,
	.voltage  = PAD_3_3V,
	.pullup   = 0,
	.drive    = !0
	 },
	{
	 .name     = "GPMI D2",
	 .id       = PINID_GPMI_D02,
	 .fun      = PIN_FUN1,
	 .strength = PAD_4MA,
	 .voltage  = PAD_3_3V,
	 .pullup   = 0,
	 .drive    = !0
	 },
	{
	 .name     = "GPMI D3",
	 .id       = PINID_GPMI_D03,
	 .fun      = PIN_FUN1,
	 .strength = PAD_4MA,
	 .voltage  = PAD_3_3V,
	 .pullup   = 0,
	 .drive    = !0
	 },
	{
	 .name     = "GPMI D4",
	 .id       = PINID_GPMI_D04,
	 .fun      = PIN_FUN1,
	 .strength = PAD_4MA,
	 .voltage  = PAD_3_3V,
	 .pullup   = 0,
	 .drive    = !0
	 },
	{
	 .name     = "GPMI D5",
	 .id       = PINID_GPMI_D05,
	 .fun      = PIN_FUN1,
	 .strength = PAD_4MA,
	 .voltage  = PAD_3_3V,
	 .pullup   = 0,
	 .drive    = !0
	 },
	{
	 .name     = "GPMI D6",
	 .id       = PINID_GPMI_D06,
	 .fun      = PIN_FUN1,
	 .strength = PAD_4MA,
	 .voltage  = PAD_3_3V,
	 .pullup   = 0,
	 .drive    = !0
	 },
	{
	 .name     = "GPMI D7",
	 .id       = PINID_GPMI_D07,
	 .fun      = PIN_FUN1,
	 .strength = PAD_4MA,
	 .voltage  = PAD_3_3V,
	 .pullup   = 0,
	 .drive    = !0
	 },
	{
	 .name     = "GPMI CLE",
	 .id       = PINID_GPMI_CLE,
	 .fun      = PIN_FUN1,
	 .strength = PAD_4MA,
	 .voltage  = PAD_3_3V,
	 .pullup   = 0,
	 .drive    = !0
	 },
	{
	 .name     = "GPMI ALE",
	 .id       = PINID_GPMI_ALE,
	 .fun      = PIN_FUN1,
	 .strength = PAD_4MA,
	 .voltage  = PAD_3_3V,
	 .pullup   = 0,
	 .drive    = !0
	 },
	{
	 .name     = "GPMI WPN-",
	 .id       = PINID_GPMI_WPN,
	 .fun      = PIN_FUN1,
	 .strength = PAD_12MA,
	 .voltage  = PAD_3_3V,
	 .pullup   = 0,
	 .drive    = !0
	 },
	{
	 .name     = "GPMI WR-",
	 .id       = PINID_GPMI_WRN,
	 .fun      = PIN_FUN1,
	 .strength = PAD_12MA,
	 .voltage  = PAD_3_3V,
	 .pullup   = 0,
	 .drive    = !0
	 },
	{
	 .name     = "GPMI RD-",
	 .id       = PINID_GPMI_RDN,
	 .fun      = PIN_FUN1,
	 .strength = PAD_12MA,
	 .voltage  = PAD_3_3V,
	 .pullup   = 0,
	 .drive    = !0
	 },
	{
	 .name     = "GPMI RDY0",
	 .id       = PINID_GPMI_RDY0,
	 .fun      = PIN_FUN1,
	 .strength = PAD_4MA,
	 .voltage  = PAD_3_3V,
	 .pullup   = 0,
	 .drive    = !0
	 },
	{
	 .name     = "GPMI RDY1",
	 .id       = PINID_GPMI_RDY1,
	 .fun      = PIN_FUN1,
	 .strength = PAD_4MA,
	 .voltage  = PAD_3_3V,
	 .pullup   = 0,
	 .drive    = !0
	 },
	{
	 .name     = "GPMI CE0-",
	 .id       = PINID_GPMI_CE0N,
	 .fun      = PIN_FUN1,
	 .strength = PAD_4MA,
	 .voltage  = PAD_3_3V,
	 .pullup   = 0,
	 .drive    = !0
	 },
	{
	 .name     = "GPMI CE1-",
	 .id       = PINID_GPMI_CE1N,
	 .fun      = PIN_FUN1,
	 .strength = PAD_4MA,
	 .voltage  = PAD_3_3V,
	 .pullup   = 0,
	 .drive    = !0
	 },
#endif

};

#if defined(CONFIG_MMC_MXS) || defined(CONFIG_MMC_MXS_MODULE)
static struct pin_desc mx23evk_mmc_pins[] = {
	/* Configurations of SSP0 SD/MMC port pins */
	{
	 .name = "SSP1_DATA0",
	 .id = PINID_SSP1_DATA0,
	 .fun = PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage = PAD_3_3V,
	 .pullup = 1,
	 .drive = 1,
	 .pull = 1,
	 },
	{
	 .name = "SSP1_DATA1",
	 .id = PINID_SSP1_DATA1,
	 .fun = PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage = PAD_3_3V,
	 .pullup = 1,
	 .drive = 1,
	 .pull = 1,
	 },
	{
	 .name = "SSP1_DATA2",
	 .id = PINID_SSP1_DATA2,
	 .fun = PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage = PAD_3_3V,
	 .pullup = 1,
	 .drive = 1,
	 .pull = 1,
	 },
	{
	 .name = "SSP1_DATA3",
	 .id = PINID_SSP1_DATA3,
	 .fun = PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage = PAD_3_3V,
	 .pullup = 1,
	 .drive = 1,
	 .pull = 1,
	 },
	{
	 .name = "SSP1_CMD",
	 .id = PINID_SSP1_CMD,
	 .fun = PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage = PAD_3_3V,
	 .pullup = 1,
	 .drive = 1,
	 .pull = 1,
	 },
	{
	 .name = "SSP1_DETECT",
	 .id = PINID_SSP1_DETECT,
	 .fun = PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage = PAD_3_3V,
	 .pullup = 0,
	 .drive = 1,
	 .pull = 0,
	 },
	{
	 .name = "SSP1_SCK",
	 .id = PINID_SSP1_SCK,
	 .fun = PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage = PAD_3_3V,
	 .pullup = 0,
	 .drive = 1,
	 .pull = 0,
	 },
};
#endif

#if defined(CONFIG_SPI_MXS) || defined(CONFIG_SPI_MXS_MODULE)
static struct pin_desc mx23evk_spi_pins[] = {
	{
	 .name	= "SSP1_DATA0",
	 .id	= PINID_SSP1_DATA0,
	 .fun	= PIN_FUN1,
	 .strength	= PAD_4MA,
	 .voltage	= PAD_3_3V,
	 .drive 	= 1,
	 },
	{
	 .name	= "SSP1_DATA3",
	 .id	= PINID_SSP1_DATA3,
	 .fun	= PIN_FUN1,
	 .strength	= PAD_4MA,
	 .voltage	= PAD_3_3V,
	 .drive 	= 1,
	 },
	{
	 .name	= "SSP1_CMD",
	 .id	= PINID_SSP1_CMD,
	 .fun	= PIN_FUN1,
	 .strength	= PAD_4MA,
	 .voltage	= PAD_3_3V,
	 .drive 	= 1,
	 },
	{
	 .name	= "SSP1_SCK",
	 .id	= PINID_SSP1_SCK,
	 .fun	= PIN_FUN1,
	 .strength	= PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive 	= 1,
	 },
};
#endif


static void mxs_request_pins(struct pin_desc *pins, int nr)
{
	int i;
	struct pin_desc *pin;

	/* configure the pins */
	for (i = 0; i < nr; i++) {
		pin = &pins[i];
		if (pin->fun == PIN_GPIO)
			gpio_request(MXS_PIN_TO_GPIO(pin->id), pin->name);
		else
			mxs_request_pin(pin->id, pin->fun, pin->name);
		if (pin->drive) {
			mxs_set_strength(pin->id, pin->strength, pin->name);
			mxs_set_voltage(pin->id, pin->voltage, pin->name);
		}
		if (pin->pull)
			mxs_set_pullup(pin->id, pin->pullup, pin->name);
		if (pin->fun == PIN_GPIO) {
			if (pin->output)
				gpio_direction_output(MXS_PIN_TO_GPIO(pin->id),
						      pin->data);
			else
				gpio_direction_input(MXS_PIN_TO_GPIO(pin->id));
		}
	}
}

static void mxs_release_pins(struct pin_desc *pins, int nr)
{
	int i;
	struct pin_desc *pin;

	/* release the pins */
	for (i = 0; i < nr; i++) {
		pin = &pins[i];
		if (pin->fun == PIN_GPIO)
			gpio_free(MXS_PIN_TO_GPIO(pin->id));
		else
			mxs_release_pin(pin->id, pin->name);
	}
}

#if defined(CONFIG_MXC_MMA7450) || defined(CONFIG_MXC_MMA7450_MODULE)
int mx23evk_mma7450_pin_init(void)
{
	/* intr */
	gpio_request(MXS_PIN_TO_GPIO(PINID_GPMI_D14), "MMA7450_INTR1");
	gpio_direction_input(MXS_PIN_TO_GPIO(PINID_GPMI_D14));
	gpio_request(MXS_PIN_TO_GPIO(PINID_GPMI_D15), "MMA7450_INTR2");
	gpio_direction_input(MXS_PIN_TO_GPIO(PINID_GPMI_D15));
	return 0;
}
int mx23evk_mma7450_pin_release(void)
{
	return 0;
}
#else
int mx23evk_mma7450_pin_init(void)
{
	return 0;
}
int mx23evk_mma7450_pin_release(void)
{
	return 0;
}
#endif

#if defined(CONFIG_MMC_MXS) || defined(CONFIG_MMC_MXS_MODULE)
#define MMC0_POWER	MXS_PIN_TO_GPIO(PINID_PWM3)
#define MMC0_WP		MXS_PIN_TO_GPIO(PINID_PWM4)

int mxs_mmc_get_wp_mmc0(void)
{
	return gpio_get_value(MMC0_WP);
}

int mxs_mmc_hw_init_mmc0(void)
{
	int ret = 0;

	mxs_request_pins(mx23evk_mmc_pins, ARRAY_SIZE(mx23evk_mmc_pins));

	/* Configure write protect GPIO pin */
	ret = gpio_request(MMC0_WP, "mmc0_wp");
	if (ret) {
		pr_err("wp\n");
		goto out_wp;
	}
	gpio_set_value(MMC0_WP, 0);
	gpio_direction_input(MMC0_WP);

	/* Configure POWER pin as gpio to drive power to MMC slot */
	ret = gpio_request(MMC0_POWER, "mmc0_power");
	if (ret) {
		pr_err("power\n");
		goto out_power;
	}
	gpio_direction_output(MMC0_POWER, 0);
	mdelay(100);

	return 0;

out_power:
	gpio_free(MMC0_WP);
out_wp:
	mxs_release_pins(mx23evk_mmc_pins, ARRAY_SIZE(mx23evk_mmc_pins));
	return ret;
}

void mxs_mmc_hw_release_mmc0(void)
{
	gpio_free(MMC0_POWER);
	gpio_free(MMC0_WP);

	mxs_release_pins(mx23evk_mmc_pins, ARRAY_SIZE(mx23evk_mmc_pins));
}

void mxs_mmc_cmd_pullup_mmc0(int enable)
{
	mxs_set_pullup(PINID_SSP1_CMD, enable, "mmc0_cmd");
}
#else
int mxs_mmc_get_wp_mmc0(void)
{
	return 0;
}
int mxs_mmc_hw_init_mmc0(void)
{
	return 0;
}

void mxs_mmc_hw_release_mmc0(void)
{
}

void mxs_mmc_cmd_pullup_mmc0(int enable)
{
}
#endif

#if defined(CONFIG_ENC28J60) || defined(CONFIG_ENC28J60_MODULE)
int mxs_spi_enc_pin_init(void)
{
	unsigned gpio = MXS_PIN_TO_GPIO(PINID_SSP1_DATA1);

	mxs_request_pins(mx23evk_spi_pins, ARRAY_SIZE(mx23evk_spi_pins));

	gpio_request(gpio, "ENC28J60_INTR");
	gpio_direction_input(gpio);
	set_irq_type(gpio_to_irq(gpio), IRQ_TYPE_EDGE_FALLING);

	return 0;
}
int mxs_spi_enc_pin_release(void)
{
	unsigned gpio = MXS_PIN_TO_GPIO(PINID_SSP1_DATA1);


	gpio_free(gpio);
	set_irq_type(gpio_to_irq(gpio), IRQ_TYPE_NONE);

	/* release the pins */
	mxs_release_pins(mx23evk_spi_pins, ARRAY_SIZE(mx23evk_spi_pins));

	return 0;
}
#else
int mxs_spi_enc_pin_init(void)
{
	return 0;
}
int mxs_spi_enc_pin_release(void)
{
	return 0;
}
#endif

#if defined(CONFIG_FEC) || defined(CONFIG_FEC_MODULE)
int mx23evk_enet_gpio_init(void)
{
	/* pwr */
	gpio_request(MXS_PIN_TO_GPIO(PINID_SSP1_DATA3), "ENET_PWR");
	gpio_direction_output(MXS_PIN_TO_GPIO(PINID_SSP1_DATA3), 0);

	/* reset phy */
	gpio_request(MXS_PIN_TO_GPIO(PINID_ENET0_RX_CLK), "PHY_RESET");
	gpio_direction_output(MXS_PIN_TO_GPIO(PINID_ENET0_RX_CLK), 0);
	gpio_direction_output(MXS_PIN_TO_GPIO(PINID_ENET0_RX_CLK), 1);

	return 0;
}
#else
int mx23evk_enet_gpio_init(void)
{
	return 0;
}
#endif

void __init mx23evk_pins_init(void)
{
	mxs_request_pins(mx23evk_fixed_pins, ARRAY_SIZE(mx23evk_fixed_pins));
}
