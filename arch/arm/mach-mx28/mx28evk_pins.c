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

#include "mx28_pins.h"

static struct pin_desc mx28evk_fixed_pins[] = {
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
#ifdef CONFIG_MXS_AUART0_DEVICE_ENABLE
	{
	 .name  = "AUART0.RX",
	 .id    = PINID_AUART0_RX,
	 .fun   = PIN_FUN1,
	 },
	{
	 .name  = "AUART0.TX",
	 .id    = PINID_AUART0_TX,
	 .fun   = PIN_FUN1,
	 },
	{
	 .name  = "AUART0.CTS",
	 .id    = PINID_AUART0_CTS,
	 .fun   = PIN_FUN1,
	 },
	{
	 .name  = "AUART0.RTS",
	 .id    = PINID_AUART0_RTS,
	 .fun   = PIN_FUN1,
	 },
#endif
#ifdef CONFIG_MXS_AUART3_DEVICE_ENABLE
	{
	 .name  = "AUART3.RX",
	 .id    = PINID_AUART3_RX,
	 .fun   = PIN_FUN1,
	 },
	{
	 .name  = "AUART3.TX",
	 .id    = PINID_AUART3_TX,
	 .fun   = PIN_FUN1,
	 },
	{
	 .name  = "AUART3.CTS",
	 .id    = PINID_AUART3_CTS,
	 .fun   = PIN_FUN1,
	 },
	{
	 .name  = "AUART3.RTS",
	 .id    = PINID_AUART3_RTS,
	 .fun   = PIN_FUN1,
	 },
#endif
	{
	 .name = "usb0",
	 .id = PINID_AUART2_TX, /* Power enable pin*/
	 .fun = PIN_GPIO,
	 .data = 0,
	 .output = 1,
	 },
	 {
	 .name  = "usb1",
	 .id    = PINID_AUART2_RX,
	 .fun   = PIN_GPIO,
	 .data  = 1,
	 .output = 1,
	 },

#if defined(CONFIG_USB_OTG)
	 {
	 .name 	= "usb0_id",
	 .id 	= PINID_AUART1_RTS,
	 .fun	= PIN_FUN2,
	 .data 	= 1,
	 .pull 	= 1,
	 .pullup = 1,
	 },
#endif

#if defined(CONFIG_CAN_FLEXCAN) || defined(CONFIG_CAN_FLEXCAN_MODULE)
	{
	 .name	= "CAN1_TX",
	 .id	= PINID_GPMI_CE2N,
	 .fun	= PIN_FUN2,
	 .strength	= PAD_4MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 0,
	 .drive 	= 1,
	 .pull 		= 0,
	 },
	{
	 .name	= "CAN1_RX",
	 .id	= PINID_GPMI_CE3N,
	 .fun	= PIN_FUN2,
	 .strength	= PAD_4MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 0,
	 .drive 	= 1,
	 .pull 		= 0,
	 },
	{
	 .name	= "CAN0_TX",
	 .id	= PINID_GPMI_RDY2,
	 .fun	= PIN_FUN2,
	 .strength	= PAD_4MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 0,
	 .drive 	= 1,
	 .pull 		= 0,
	 },
	{
	 .name	= "CAN0_RX",
	 .id	= PINID_GPMI_RDY3,
	 .fun	= PIN_FUN2,
	 .strength	= PAD_4MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 0,
	 .drive 	= 1,
	 .pull 		= 0,
	 },
	{
	 .name	= "CAN_PWDN",
	 .id	= PINID_SSP1_CMD,
	 .fun	= PIN_GPIO,
	 .strength	= PAD_4MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 0,
	 .drive 	= 1,
	 .pull 		= 0,
	 .data		= 0,
	 .output	= 1,
	 },

#endif

#if defined(CONFIG_I2C_MXS) || \
	defined(CONFIG_I2C_MXS_MODULE)
	{
	 .name = "I2C0_SCL",
	 .id = PINID_I2C0_SCL,
	 .fun = PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage = PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name = "I2C0_SDA",
	 .id = PINID_I2C0_SDA,
	 .fun = PIN_FUN1,
	 .strength = PAD_8MA,
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
	 .id	= PINID_LCD_D18,
	 .fun	= PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name  = "LCD_D19",
	 .id	= PINID_LCD_D19,
	 .fun	= PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name  = "LCD_D20",
	 .id	= PINID_LCD_D20,
	 .fun	= PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name  = "LCD_D21",
	 .id	= PINID_LCD_D21,
	 .fun	= PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name  = "LCD_D22",
	 .id	= PINID_LCD_D22,
	 .fun	= PIN_FUN1,
	 .strength = PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name  = "LCD_D23",
	 .id	= PINID_LCD_D23,
	 .fun	= PIN_FUN1,
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
	 .id   = PINID_LCD_RD_E,
	 .fun  = PIN_FUN2,
	 .strength = PAD_8MA,
	 .voltage = PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name = "LCD_HSYNC",
	 .id = PINID_LCD_WR_RWN,
	 .fun = PIN_FUN2,
	 .strength = PAD_8MA,
	 .voltage = PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name = "LCD_ENABLE",
	 .id = PINID_LCD_CS,
	 .fun = PIN_FUN2,
	 .strength = PAD_8MA,
	 .voltage = PAD_3_3V,
	 .drive	= 1,
	 },
	{
	 .name = "LCD_DOTCLK",
	 .id = PINID_LCD_RS,
	 .fun = PIN_FUN2,
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
#if defined(CONFIG_MMC_MXS) || defined(CONFIG_MMC_MXS_MODULE)
	/* Configurations of SSP0 SD/MMC port pins */
	{
	 .name	= "SSP0_DATA0",
	 .id	= PINID_SSP0_DATA0,
	 .fun	= PIN_FUN1,
	 .strength	= PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 1,
	 .drive 	= 1,
	 .pull 		= 1,
	 },
	{
	 .name	= "SSP0_DATA1",
	 .id	= PINID_SSP0_DATA1,
	 .fun	= PIN_FUN1,
	 .strength	= PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 1,
	 .drive 	= 1,
	 .pull 		= 1,
	 },
	{
	 .name	= "SSP0_DATA2",
	 .id	= PINID_SSP0_DATA2,
	 .fun	= PIN_FUN1,
	 .strength	= PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 1,
	 .drive 	= 1,
	 .pull 		= 1,
	 },
	{
	 .name	= "SSP0_DATA3",
	 .id	= PINID_SSP0_DATA3,
	 .fun	= PIN_FUN1,
	 .strength	= PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 1,
	 .drive 	= 1,
	 .pull 		= 1,
	 },
	{
	 .name	= "SSP0_DATA4",
	 .id	= PINID_SSP0_DATA4,
	 .fun	= PIN_FUN1,
	 .strength	= PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 1,
	 .drive 	= 1,
	 .pull 		= 1,
	 },
	{
	 .name	= "SSP0_DATA5",
	 .id	= PINID_SSP0_DATA5,
	 .fun	= PIN_FUN1,
	 .strength	= PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 1,
	 .drive 	= 1,
	 .pull 		= 1,
	 },
	{
	 .name	= "SSP0_DATA6",
	 .id	= PINID_SSP0_DATA6,
	 .fun	= PIN_FUN1,
	 .strength	= PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 1,
	 .drive 	= 1,
	 .pull 		= 1,
	 },
	{
	 .name	= "SSP0_DATA7",
	 .id	= PINID_SSP0_DATA7,
	 .fun	= PIN_FUN1,
	 .strength	= PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 1,
	 .drive 	= 1,
	 .pull 		= 1,
	 },
	{
	 .name	= "SSP0_CMD",
	 .id	= PINID_SSP0_CMD,
	 .fun	= PIN_FUN1,
	 .strength	= PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 1,
	 .drive 	= 1,
	 .pull 		= 1,
	 },
	{
	 .name	= "SSP0_DETECT",
	 .id	= PINID_SSP0_DETECT,
	 .fun	= PIN_FUN1,
	 .strength	= PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 0,
	 .drive 	= 1,
	 .pull 		= 0,
	 },
	{
	 .name	= "SSP0_SCK",
	 .id	= PINID_SSP0_SCK,
	 .fun	= PIN_FUN1,
	 .strength	= PAD_12MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 0,
	 .drive 	= 2,
	 .pull 		= 0,
	 },
#endif
#if defined(CONFIG_LEDS_MXS) || defined(CONFIG_LEDS_MXS_MODULE)
	{
	 .name = "LEDS_PWM0",
	 .id = PINID_AUART1_RX,
	 .fun           = PIN_FUN3,
	 .strength      = PAD_8MA,
	 .voltage       = PAD_3_3V,
	 .pullup        = 1,
	 .drive         = 1,
	 .pull          = 1,
	 },
	{
	 .name = "LEDS_PWM1",
	 .id = PINID_AUART1_TX,
	 .fun           = PIN_FUN3,
	 .strength      = PAD_8MA,
	 .voltage       = PAD_3_3V,
	 .pullup        = 1,
	 .drive         = 1,
	 .pull          = 1,
	 },
#endif
#if defined(CONFIG_SND_MXS_SOC_DAI) || defined(CONFIG_SND_MXS_SOC_DAI_MODULE)
	/* Configurations of SAIF0 port pins */
	{
	 .name	= "SAIF0_MCLK",
	 .id	= PINID_SAIF0_MCLK,
	 .fun	= PIN_FUN1,
	 .strength	= PAD_12MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 1,
	 .drive 	= 1,
	 .pull 		= 1,
	 },
	{
	 .name	= "SAIF0_LRCLK",
	 .id	= PINID_SAIF0_LRCLK,
	 .fun	= PIN_FUN1,
	 .strength	= PAD_12MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 1,
	 .drive 	= 1,
	 .pull 		= 1,
	 },
	{
	 .name	= "SAIF0_BITCLK",
	 .id	= PINID_SAIF0_BITCLK,
	 .fun	= PIN_FUN1,
	 .strength	= PAD_12MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 1,
	 .drive 	= 1,
	 .pull 		= 1,
	 },
	{
	 .name	= "SAIF0_SDATA0",
	 .id	= PINID_SAIF0_SDATA0,
	 .fun	= PIN_FUN1,
	 .strength	= PAD_12MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 1,
	 .drive 	= 1,
	 .pull 		= 1,
	 },
	{
	 .name	= "SAIF1_SDATA0",
	 .id	= PINID_SAIF1_SDATA0,
	 .fun	= PIN_FUN1,
	 .strength	= PAD_12MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 1,
	 .drive 	= 1,
	 .pull 		= 1,
	 },
#endif
#if defined(CONFIG_SND_SOC_MXS_SPDIF) || \
       defined(CONFIG_SND_SOC_MXS_SPDIF_MODULE)
	{
	 .name	= "SPDIF",
	 .id	= PINID_SPDIF,
	 .fun	= PIN_FUN1,
	 .strength	= PAD_12MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 1,
	 .drive 	= 1,
	 .pull 		= 1,
	},
#endif
};

#if defined(CONFIG_FEC) || defined(CONFIG_FEC_MODULE)\
	|| defined(CONFIG_FEC_L2SWITCH)
static struct pin_desc mx28evk_eth_pins[] = {
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
	 .name = "ENET1_RX_EN",
	 .id = PINID_ENET0_CRS,
	 .fun = PIN_FUN2,
	 .strength = PAD_8MA,
	 .pull = 1,
	 .pullup = 1,
	 .voltage = PAD_3_3V,
	 .drive = 1,
	 },
	{
	 .name = "ENET1_RXD0",
	 .id = PINID_ENET0_RXD2,
	 .fun = PIN_FUN2,
	 .strength = PAD_8MA,
	 .pull = 1,
	 .pullup = 1,
	 .voltage = PAD_3_3V,
	 .drive = 1,
	 },
	{
	 .name = "ENET1_RXD1",
	 .id = PINID_ENET0_RXD3,
	 .fun = PIN_FUN2,
	 .strength = PAD_8MA,
	 .pull = 1,
	 .pullup = 1,
	 .voltage = PAD_3_3V,
	 .drive = 1,
	 },
	{
	 .name = "ENET1_TX_EN",
	 .id = PINID_ENET0_COL,
	 .fun = PIN_FUN2,
	 .strength = PAD_8MA,
	 .pull = 1,
	 .pullup = 1,
	 .voltage = PAD_3_3V,
	 .drive = 1,
	 },
	{
	 .name = "ENET1_TXD0",
	 .id = PINID_ENET0_TXD2,
	 .fun = PIN_FUN2,
	 .strength = PAD_8MA,
	 .pull = 1,
	 .pullup = 1,
	 .voltage = PAD_3_3V,
	 .drive = 1,
	 },
	{
	 .name = "ENET1_TXD1",
	 .id = PINID_ENET0_TXD3,
	 .fun = PIN_FUN2,
	 .strength = PAD_8MA,
	 .pull = 1,
	 .pullup = 1,
	 .voltage = PAD_3_3V,
	 .drive = 1,
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
};
#endif

static int __initdata enable_ssp1 = { 0 };
static int __init ssp1_setup(char *__unused)
{
	enable_ssp1 = 1;
	return 1;
}

__setup("ssp1", ssp1_setup);

static struct pin_desc mx28evk_ssp1_pins[] = {
	{
	 .name	= "SSP1_DATA0",
	 .id	= PINID_GPMI_D00,
	 .fun	= PIN_FUN2,
	 .strength	= PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 1,
	 .drive 	= 1,
	 .pull 		= 1,
	 },
	{
	 .name	= "SSP1_DATA1",
	 .id	= PINID_GPMI_D01,
	 .fun	= PIN_FUN2,
	 .strength	= PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 1,
	 .drive 	= 1,
	 .pull 		= 1,
	 },
	{
	 .name	= "SSP1_DATA2",
	 .id	= PINID_GPMI_D02,
	 .fun	= PIN_FUN2,
	 .strength	= PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 1,
	 .drive 	= 1,
	 .pull 		= 1,
	 },
	{
	 .name	= "SSP1_DATA3",
	 .id	= PINID_GPMI_D03,
	 .fun	= PIN_FUN2,
	 .strength	= PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 1,
	 .drive 	= 1,
	 .pull 		= 1,
	 },
	{
	 .name	= "SSP1_DATA4",
	 .id	= PINID_GPMI_D04,
	 .fun	= PIN_FUN2,
	 .strength	= PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 1,
	 .drive 	= 1,
	 .pull 		= 1,
	 },
	{
	 .name	= "SSP1_DATA5",
	 .id	= PINID_GPMI_D05,
	 .fun	= PIN_FUN2,
	 .strength	= PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 1,
	 .drive 	= 1,
	 .pull 		= 1,
	 },
	{
	 .name	= "SSP1_DATA6",
	 .id	= PINID_GPMI_D06,
	 .fun	= PIN_FUN2,
	 .strength	= PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 1,
	 .drive 	= 1,
	 .pull 		= 1,
	 },
	{
	 .name	= "SSP1_DATA7",
	 .id	= PINID_GPMI_D07,
	 .fun	= PIN_FUN2,
	 .strength	= PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 1,
	 .drive 	= 1,
	 .pull 		= 1,
	 },
	{
	 .name	= "SSP1_CMD",
	 .id	= PINID_GPMI_RDY1,
	 .fun	= PIN_FUN2,
	 .strength	= PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 1,
	 .drive 	= 1,
	 .pull 		= 1,
	 },
	{
	 .name	= "SSP1_DETECT",
	 .id	= PINID_GPMI_RDY0,
	 .fun	= PIN_FUN1,
	 .strength	= PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 0,
	 .drive 	= 1,
	 .pull 		= 0,
	 },
	{
	 .name	= "SSP1_SCK",
	 .id	= PINID_GPMI_WRN,
	 .fun	= PIN_FUN2,
	 .strength	= PAD_12MA,
	 .voltage	= PAD_3_3V,
	 .pullup	= 0,
	 .drive 	= 2,
	 .pull 		= 0,
	 },
};


int enable_gpmi = { 0 };
static int __init gpmi_setup(char *__unused)
{
	enable_gpmi = 1;
	return 1;
}

__setup("gpmi", gpmi_setup);

static struct pin_desc mx28evk_gpmi_pins[] = {
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
	 .name     = "GPMI RD-",
	 .id       = PINID_GPMI_RDN,
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
	 .name     = "GPMI ALE",
	 .id       = PINID_GPMI_ALE,
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
	 .name     = "GPMI RST-",
	 .id       = PINID_GPMI_RESETN,
	 .fun      = PIN_FUN1,
	 .strength = PAD_12MA,
	 .voltage  = PAD_3_3V,
	 .pullup   = 0,
	 .drive    = !0
	 },
};

#if defined(CONFIG_SPI_MXS) || defined(CONFIG_SPI_MXS_MODULE)
static struct pin_desc mx28evk_spi_pins[] = {
	{
	 .name	= "SSP2 MOSI",
	 .id	= PINID_SSP2_MOSI,
	 .fun	= PIN_FUN1,
	 .strength	= PAD_4MA,
	 .voltage	= PAD_3_3V,
	 .drive 	= 1,
	 },
	{
	 .name	= "SSP2 MISO",
	 .id	= PINID_SSP2_MISO,
	 .fun	= PIN_FUN1,
	 .strength	= PAD_4MA,
	 .voltage	= PAD_3_3V,
	 .drive 	= 1,
	 },
	{
	 .name	= "SSP2 SCK",
	 .id	= PINID_SSP2_SCK,
	 .fun	= PIN_FUN1,
	 .strength	= PAD_4MA,
	 .voltage	= PAD_3_3V,
	 .drive 	= 1,
	 },
	{
	 .name	= "SSP2 SS0",
	 .id	= PINID_SSP2_SS0,
	 .fun	= PIN_FUN1,
	 .strength	= PAD_8MA,
	 .voltage	= PAD_3_3V,
	 .drive 	= 1,
	 },
};
#endif

#if defined(CONFIG_FEC) || defined(CONFIG_FEC_MODULE)\
	|| defined(CONFIG_FEC_L2SWITCH)
int mx28evk_enet_gpio_init(void)
{
	/* pwr */
	gpio_request(MXS_PIN_TO_GPIO(PINID_SSP1_DATA3), "ENET_PWR");
	gpio_direction_output(MXS_PIN_TO_GPIO(PINID_SSP1_DATA3), 0);

	/* reset phy */
	gpio_request(MXS_PIN_TO_GPIO(PINID_ENET0_RX_CLK), "PHY_RESET");
	gpio_direction_output(MXS_PIN_TO_GPIO(PINID_ENET0_RX_CLK), 0);

	/*
	 * Before timer bug fix(set wrong match value of timer),
	 * mdelay(10) delay 50ms actually.
	 * So change delay to 50ms after timer issue fix.
	 */
	mdelay(50);
	gpio_direction_output(MXS_PIN_TO_GPIO(PINID_ENET0_RX_CLK), 1);

	return 0;
}

void mx28evk_enet_io_lowerpower_enter(void)
{
	int i;
	gpio_direction_output(MXS_PIN_TO_GPIO(PINID_SSP1_DATA3), 1);
	gpio_direction_output(MXS_PIN_TO_GPIO(PINID_ENET0_RX_CLK), 0);
	gpio_request(MXS_PIN_TO_GPIO(PINID_ENET0_TX_CLK), "ETH_INT");
	gpio_direction_output(MXS_PIN_TO_GPIO(PINID_ENET0_TX_CLK), 0);

	for (i = 0; i < ARRAY_SIZE(mx28evk_eth_pins); i++) {
		mxs_release_pin(mx28evk_eth_pins[i].id,
			mx28evk_eth_pins[i].name);
		gpio_request(MXS_PIN_TO_GPIO(mx28evk_eth_pins[i].id),
			mx28evk_eth_pins[i].name);
		gpio_direction_output(
			MXS_PIN_TO_GPIO(mx28evk_eth_pins[i].id), 0);
	}

}

void mx28evk_enet_io_lowerpower_exit(void)
{
	int i;
	gpio_direction_output(MXS_PIN_TO_GPIO(PINID_SSP1_DATA3), 0);
	gpio_direction_output(MXS_PIN_TO_GPIO(PINID_ENET0_RX_CLK), 1);
	gpio_free(MXS_PIN_TO_GPIO(PINID_ENET0_TX_CLK));
	for (i = 0; i < ARRAY_SIZE(mx28evk_eth_pins); i++) {
		gpio_free(MXS_PIN_TO_GPIO(mx28evk_eth_pins[i].id));
		mxs_request_pin(mx28evk_eth_pins[i].id,
			mx28evk_eth_pins[i].fun,
			mx28evk_eth_pins[i].name);
	}
}

#else
int mx28evk_enet_gpio_init(void)
{
	return 0;
}
void mx28evk_enet_io_lowerpower_enter(void)
{}
void mx28evk_enet_io_lowerpower_exit(void)
{}
#endif

void __init mx28evk_init_pin_group(struct pin_desc *pins, unsigned count)
{
	int i;
	struct pin_desc *pin;
	for (i = 0; i < count; i++) {
		pin = pins + i;
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

void __init mx28evk_pins_init(void)
{

	mx28evk_init_pin_group(mx28evk_fixed_pins,
						ARRAY_SIZE(mx28evk_fixed_pins));

	if (enable_ssp1) {
		pr_info("Initializing SSP1 pins\n");
		mx28evk_init_pin_group(mx28evk_ssp1_pins,
						ARRAY_SIZE(mx28evk_ssp1_pins));
	} else if (enable_gpmi) {
		pr_info("Initializing GPMI pins\n");
		mx28evk_init_pin_group(mx28evk_gpmi_pins,
						ARRAY_SIZE(mx28evk_gpmi_pins));
	}

#if defined(CONFIG_SPI_MXS) || defined(CONFIG_SPI_MXS_MODULE)
	mx28evk_init_pin_group(mx28evk_spi_pins,
					ARRAY_SIZE(mx28evk_spi_pins));
#endif

#if defined(CONFIG_FEC) || defined(CONFIG_FEC_MODULE)\
	|| defined(CONFIG_FEC_L2SWITCH)
		mx28evk_init_pin_group(mx28evk_eth_pins,
						ARRAY_SIZE(mx28evk_eth_pins));
#endif
}
