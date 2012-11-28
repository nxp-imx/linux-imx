/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
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

#ifndef _BOARD_MX6DL_HDMIDONGLE_H
#define _BOARD_MX6DL_HDMIDONGLE_H
#include <mach/iomux-mx6dl.h>

static iomux_v3_cfg_t mx6dl_hdmidongle_rev_a_pads[] = {
	/* SPI2 for PMIC communication port */
	MX6DL_PAD_EIM_OE__ECSPI2_MISO,
	MX6DL_PAD_EIM_RW__ECSPI2_SS0,
	MX6DL_PAD_EIM_CS0__ECSPI2_SCLK,
	MX6DL_PAD_EIM_CS1__ECSPI2_MOSI,

	/*USB_OTG_DET(USB OTG cable plug detect) */
	MX6DL_PAD_EIM_A16__GPIO_2_22,
	/*BT_WAKEUP_HOST(Combo module BT wake-up output) */
	MX6DL_PAD_EIM_A25__GPIO_5_2,
	/* HOST_WAKEUP_BT(CPU wakeup BT signal)*/
	MX6DL_PAD_EIM_D16__GPIO_3_16,

	/* I2C3  */
	MX6DL_PAD_EIM_D17__I2C3_SCL,
	MX6DL_PAD_EIM_D18__I2C3_SDA,

	/* USB OC pin */
	MX6DL_PAD_EIM_D21__USBOH3_USBOTG_OC,
	/* WLAN_WAKEUP_HOST(Combo module WLAN host wake-up output) */
	MX6DL_PAD_EIM_D22__GPIO_3_22,

	/* BT_UART2  */
	MX6DL_PAD_EIM_D26__UART2_TXD,
	MX6DL_PAD_EIM_D27__UART2_RXD,
	MX6DL_PAD_EIM_D28__UART2_CTS,
	MX6DL_PAD_EIM_D29__UART2_RTS,

	/*BT_nRST(Combo module BT reset signal)*/
	MX6DL_PAD_EIM_DA7__GPIO_3_7,

	/*BT_REG_ON(Combo module BT Internal regulators power enable/disable)*/
	MX6DL_PAD_EIM_DA9__GPIO_3_9,

	/*WL_REG_ON(Combo module WLAN Internal regulators power enable/disable)*/
	MX6DL_PAD_EIM_DA10__GPIO_3_10,

	/* GPIO2 */
	MX6DL_PAD_EIM_A22__GPIO_2_16,	/* Boot Mode Select */
	MX6DL_PAD_EIM_A21__GPIO_2_17,	/* Boot Mode Select */
	MX6DL_PAD_EIM_A20__GPIO_2_18,	/* Boot Mode Select */
	MX6DL_PAD_EIM_A19__GPIO_2_19,	/* Boot Mode Select */
	MX6DL_PAD_EIM_A18__GPIO_2_20,	/* Boot Mode Select */
	MX6DL_PAD_EIM_A17__GPIO_2_21,	/* Boot Mode Select */
	MX6DL_PAD_EIM_RW__GPIO_2_26,		/* Boot Mode Select */
	MX6DL_PAD_EIM_LBA__GPIO_2_27,	/* Boot Mode Select */
	MX6DL_PAD_EIM_EB0__GPIO_2_28,	/* Boot Mode Select */
	MX6DL_PAD_EIM_EB1__GPIO_2_29,	/* Boot Mode Select */
	MX6DL_PAD_EIM_EB2__GPIO_2_30,	/* Boot Mode Select */
	MX6DL_PAD_EIM_EB3__GPIO_2_31,	/* Boot Mode Select */

	/* GPIO3 */
	MX6DL_PAD_EIM_DA0__GPIO_3_0,	/*  Boot Mode Select */
	MX6DL_PAD_EIM_DA1__GPIO_3_1,	/*  Boot Mode Select */
	MX6DL_PAD_EIM_DA2__GPIO_3_2,	/*  Boot Mode Select */
	MX6DL_PAD_EIM_DA3__GPIO_3_3,	/*  Boot Mode Select */
	MX6DL_PAD_EIM_DA4__GPIO_3_4,	/*  Boot Mode Select */
	MX6DL_PAD_EIM_DA5__GPIO_3_5,	/*  Boot Mode Select */
	MX6DL_PAD_EIM_DA6__GPIO_3_6,	/*  Boot Mode Select */
	MX6DL_PAD_EIM_DA8__GPIO_3_8,	/*  Boot Mode Select */
	MX6DL_PAD_EIM_DA11__GPIO_3_11,	/*  Boot Mode Select */
	MX6DL_PAD_EIM_DA12__GPIO_3_12,	/*  Boot Mode Select */
	MX6DL_PAD_EIM_DA13__GPIO_3_13,	/*  Boot Mode Select */
	MX6DL_PAD_EIM_DA14__GPIO_3_14,	/*  Boot Mode Select */
	MX6DL_PAD_EIM_DA15__GPIO_3_15,	/*  Boot Mode Select */

	/* GPIO5 */
	MX6DL_PAD_EIM_WAIT__GPIO_5_0,	/* Boot Mode Select */
	MX6DL_PAD_EIM_A24__GPIO_5_4,	/* Boot Mode Select */

	/* GPIO6 */
	MX6DL_PAD_EIM_A23__GPIO_6_6,	/* Boot Mode Select */

	/* I2C1  */
	MX6DL_PAD_CSI0_DAT8__I2C1_SDA,
	MX6DL_PAD_CSI0_DAT9__I2C1_SCL,

	/* UART1 for debug */
	MX6DL_PAD_CSI0_DAT10__UART1_TXD,
	MX6DL_PAD_CSI0_DAT11__UART1_RXD,

	/* SD1 (Combo module WLAN SDIO )*/
	MX6DL_PAD_SD1_CLK__USDHC1_CLK,
	MX6DL_PAD_SD1_CMD__USDHC1_CMD,
	MX6DL_PAD_SD1_DAT0__USDHC1_DAT0,
	MX6DL_PAD_SD1_DAT1__USDHC1_DAT1,
	MX6DL_PAD_SD1_DAT2__USDHC1_DAT2,
	MX6DL_PAD_SD1_DAT3__USDHC1_DAT3,

	/* SD2 (MicroSD SDIO CMD)*/
	MX6DL_PAD_SD2_CLK__USDHC2_CLK,
	MX6DL_PAD_SD2_CMD__USDHC2_CMD,
	MX6DL_PAD_SD2_DAT0__USDHC2_DAT0,
	MX6DL_PAD_SD2_DAT1__USDHC2_DAT1,
	MX6DL_PAD_SD2_DAT2__USDHC2_DAT2,
	MX6DL_PAD_SD2_DAT3__USDHC2_DAT3,
	/*SD_DET (SD plug-in detect interrupt)*/
	MX6DL_PAD_GPIO_4__GPIO_1_4,

	/* SD3 (eMMC SDIO)*/
	MX6DL_PAD_SD3_CLK__USDHC3_CLK_50MHZ,
	MX6DL_PAD_SD3_CMD__USDHC3_CMD_50MHZ,
	MX6DL_PAD_SD3_DAT0__USDHC3_DAT0_50MHZ,
	MX6DL_PAD_SD3_DAT1__USDHC3_DAT1_50MHZ,
	MX6DL_PAD_SD3_DAT2__USDHC3_DAT2_50MHZ,
	MX6DL_PAD_SD3_DAT3__USDHC3_DAT3_50MHZ,
	MX6DL_PAD_SD3_DAT4__USDHC3_DAT4_50MHZ,
	MX6DL_PAD_SD3_DAT5__USDHC3_DAT5_50MHZ,
	MX6DL_PAD_SD3_DAT6__USDHC3_DAT6_50MHZ,
	MX6DL_PAD_SD3_DAT7__USDHC3_DAT7_50MHZ,
	MX6DL_PAD_SD3_RST__USDHC3_RST,

	/* UART4 for debug */
	MX6DL_PAD_KEY_COL0__UART4_TXD,
	MX6DL_PAD_KEY_ROW0__UART4_RXD,

	/*HDMI CEC communication PIN*/
	MX6DL_PAD_KEY_ROW2__HDMI_TX_CEC_LINE,

	/* I2C2 */
	MX6DL_PAD_KEY_COL3__I2C2_SCL,	/* GPIO4[12] */
	MX6DL_PAD_KEY_ROW3__I2C2_SDA,	/* GPIO4[13] */

	/*DCDC5V_PWREN(5V DCDC BOOST control signal)*/
	MX6DL_PAD_KEY_COL4__GPIO_4_14,

	/*USB_OTG_PWREN(USB OTG power change control signal)*/
	MX6DL_PAD_KEY_ROW4__GPIO_4_15,

	/* USBOTG ID pin */
	MX6DL_PAD_GPIO_1__USBOTG_ID,

	/* I2C3 */
	MX6DL_PAD_GPIO_5__I2C3_SCL,
	MX6DL_PAD_GPIO_16__I2C3_SDA,

	/*WDOG(Watch dog output)*/
	MX6DL_PAD_GPIO_9__GPIO_1_9,

	/*PMIC_nINT(PMIC interrupt signal)*/
	MX6DL_PAD_NANDF_CS0__GPIO_6_11,

	/*GPIO_nRST(GPIO shutdown control)*/
	MX6DL_PAD_NANDF_CS1__GPIO_6_14,

	/*PWRKEY_DET(Power key press detection)*/
	MX6DL_PAD_NANDF_CS3__GPIO_6_16,

	/*CHG_SYS_ON(Charger auto power on control signal)*/
	MX6DL_PAD_NANDF_D6__GPIO_2_6,

	/*IR_RC*/
	MX6DL_PAD_SD4_DAT6__GPIO_2_14,
};

static iomux_v3_cfg_t mx6dl_hdmidongle_rev_b_pads[] = {
	/* SPI2 for PMIC communication port */
	MX6DL_PAD_EIM_OE__ECSPI2_MISO,
	MX6DL_PAD_EIM_RW__ECSPI2_SS0,
	MX6DL_PAD_EIM_CS0__ECSPI2_SCLK,
	MX6DL_PAD_EIM_CS1__ECSPI2_MOSI,

	/*USB_OTG_DET(USB OTG cable plug detect) */
	MX6DL_PAD_EIM_A16__GPIO_2_22,
	/*WLAN_CLKREQn*/
	MX6DL_PAD_EIM_A25__GPIO_5_2,

	/* WLAN_ACT */
	MX6DL_PAD_EIM_D17__GPIO_3_17,

	/*BT_PERI*/
	MX6DL_PAD_EIM_D18__GPIO_3_18,

	/* USB OC pin */
	MX6DL_PAD_EIM_D21__USBOH3_USBOTG_OC,

	/* WLAN_WAKEn (PM event, OD, used to reactivate the PCIe main PWR and REF CLK) */
	MX6DL_PAD_EIM_D22__GPIO_3_22,


	/*WLAN_PERSTn (PCIe rst signal, avtive LOW)*/
	MX6DL_PAD_EIM_DA9__GPIO_3_9,

	/*WLAN_PDn (Externally shutdown RTL8192)*/
	MX6DL_PAD_EIM_DA10__GPIO_3_10,

	/* GPIO2 */
	MX6DL_PAD_EIM_A22__GPIO_2_16,	/* Boot Mode Select */
	MX6DL_PAD_EIM_A21__GPIO_2_17,	/* Boot Mode Select */
	MX6DL_PAD_EIM_A20__GPIO_2_18,	/* Boot Mode Select */
	MX6DL_PAD_EIM_A19__GPIO_2_19,	/* Boot Mode Select */
	MX6DL_PAD_EIM_A18__GPIO_2_20,	/* Boot Mode Select */
	MX6DL_PAD_EIM_A17__GPIO_2_21,	/* Boot Mode Select */
	MX6DL_PAD_EIM_RW__GPIO_2_26,		/* Boot Mode Select */
	MX6DL_PAD_EIM_LBA__GPIO_2_27,	/* Boot Mode Select */
	MX6DL_PAD_EIM_EB0__GPIO_2_28,	/* Boot Mode Select */
	MX6DL_PAD_EIM_EB1__GPIO_2_29,	/* Boot Mode Select */
	MX6DL_PAD_EIM_EB2__GPIO_2_30,	/* Boot Mode Select */
	MX6DL_PAD_EIM_EB3__GPIO_2_31,	/* Boot Mode Select */

	/* GPIO3 */
	MX6DL_PAD_EIM_DA0__GPIO_3_0,	/*  Boot Mode Select */
	MX6DL_PAD_EIM_DA1__GPIO_3_1,	/*  Boot Mode Select */
	MX6DL_PAD_EIM_DA2__GPIO_3_2,	/*  Boot Mode Select */
	MX6DL_PAD_EIM_DA3__GPIO_3_3,	/*  Boot Mode Select */
	MX6DL_PAD_EIM_DA4__GPIO_3_4,	/*  Boot Mode Select */
	MX6DL_PAD_EIM_DA5__GPIO_3_5,	/*  Boot Mode Select */
	MX6DL_PAD_EIM_DA6__GPIO_3_6,	/*  Boot Mode Select */
	MX6DL_PAD_EIM_DA8__GPIO_3_8,	/*  Boot Mode Select */
	MX6DL_PAD_EIM_DA11__GPIO_3_11,	/*  Boot Mode Select */
	MX6DL_PAD_EIM_DA12__GPIO_3_12,	/*  Boot Mode Select */
	MX6DL_PAD_EIM_DA13__GPIO_3_13,	/*  Boot Mode Select */
	MX6DL_PAD_EIM_DA14__GPIO_3_14,	/*  Boot Mode Select */
	MX6DL_PAD_EIM_DA15__GPIO_3_15,	/*  Boot Mode Select */

	/* GPIO5 */
	MX6DL_PAD_EIM_WAIT__GPIO_5_0,	/* Boot Mode Select */
	MX6DL_PAD_EIM_A24__GPIO_5_4,		/* Boot Mode Select */

	/* GPIO6 */
	MX6DL_PAD_EIM_A23__GPIO_6_6,	/* Boot Mode Select */

	/* SD2 (MicroSD SDIO CMD)*/
	MX6DL_PAD_SD2_CLK__USDHC2_CLK,
	MX6DL_PAD_SD2_CMD__USDHC2_CMD,
	MX6DL_PAD_SD2_DAT0__USDHC2_DAT0,
	MX6DL_PAD_SD2_DAT1__USDHC2_DAT1,
	MX6DL_PAD_SD2_DAT2__USDHC2_DAT2,
	MX6DL_PAD_SD2_DAT3__USDHC2_DAT3,
	/*SD_DET (SD plug-in detect interrupt)*/
	MX6DL_PAD_GPIO_4__GPIO_1_4,

	/* SD3 (eMMC SDIO)*/
	MX6DL_PAD_SD3_CLK__USDHC3_CLK_50MHZ,
	MX6DL_PAD_SD3_CMD__USDHC3_CMD_50MHZ,
	MX6DL_PAD_SD3_DAT0__USDHC3_DAT0_50MHZ,
	MX6DL_PAD_SD3_DAT1__USDHC3_DAT1_50MHZ,
	MX6DL_PAD_SD3_DAT2__USDHC3_DAT2_50MHZ,
	MX6DL_PAD_SD3_DAT3__USDHC3_DAT3_50MHZ,
	MX6DL_PAD_SD3_DAT4__USDHC3_DAT4_50MHZ,
	MX6DL_PAD_SD3_DAT5__USDHC3_DAT5_50MHZ,
	MX6DL_PAD_SD3_DAT6__USDHC3_DAT6_50MHZ,
	MX6DL_PAD_SD3_DAT7__USDHC3_DAT7_50MHZ,
	MX6DL_PAD_SD3_RST__USDHC3_RST,

	/* UART4 for debug */
	MX6DL_PAD_KEY_COL0__UART4_TXD,
	MX6DL_PAD_KEY_ROW0__UART4_RXD,

	/*SD2_VSELECT (SD2 SDXC power exchange control signal)*/
	MX6DL_PAD_KEY_ROW1__GPIO_4_9,

	/*HDMI CEC communication PIN*/
	MX6DL_PAD_KEY_ROW2__HDMI_TX_CEC_LINE,

	/*PWRKEY_DET (Pwr button detection interrupt)*/
	MX6DL_PAD_ENET_RXD0__GPIO_1_27,

	/*USB_OTG_PWREN(USB OTG power change control signal)*/
	MX6DL_PAD_KEY_ROW4__GPIO_4_15,

	/* USBOTG ID pin */
	MX6DL_PAD_GPIO_1__USBOTG_ID,

	/* I2C2 */
	MX6DL_PAD_KEY_COL3__I2C2_SCL,	/* GPIO4[12] */
	MX6DL_PAD_KEY_ROW3__I2C2_SDA,	/* GPIO4[13] */

	/* I2C3 */
	MX6DL_PAD_GPIO_5__I2C3_SCL,
	MX6DL_PAD_GPIO_16__I2C3_SDA,

	/*WDOG(Watch dog output)*/
	MX6DL_PAD_GPIO_9__GPIO_1_9,

	/*PMIC_nINT(PMIC interrupt signal)*/
	MX6DL_PAD_SD4_DAT3__GPIO_2_11,

	/*GPIO_nRST(GPIO shutdown control)*/
	MX6DL_PAD_SD4_DAT1__GPIO_2_9,

	/*CHG_SYS_ON(Charger auto power on control signal)*/
	MX6DL_PAD_SD4_DAT6__GPIO_2_14,
};

/* The GPMI is conflicted with SD3, so init this in the driver. */
static iomux_v3_cfg_t mx6dl_gpmi_nand[] __initdata = {
	MX6DL_PAD_NANDF_CLE__RAWNAND_CLE,
	MX6DL_PAD_NANDF_ALE__RAWNAND_ALE,
	MX6DL_PAD_NANDF_CS0__RAWNAND_CE0N,
	MX6DL_PAD_NANDF_CS1__RAWNAND_CE1N,
	MX6DL_PAD_NANDF_CS2__RAWNAND_CE2N,
	MX6DL_PAD_NANDF_CS3__RAWNAND_CE3N,
	MX6DL_PAD_NANDF_RB0__RAWNAND_READY0,
	MX6DL_PAD_SD4_DAT0__RAWNAND_DQS,
	MX6DL_PAD_NANDF_D0__RAWNAND_D0,
	MX6DL_PAD_NANDF_D1__RAWNAND_D1,
	MX6DL_PAD_NANDF_D2__RAWNAND_D2,
	MX6DL_PAD_NANDF_D3__RAWNAND_D3,
	MX6DL_PAD_NANDF_D4__RAWNAND_D4,
	MX6DL_PAD_NANDF_D5__RAWNAND_D5,
	MX6DL_PAD_NANDF_D6__RAWNAND_D6,
	MX6DL_PAD_NANDF_D7__RAWNAND_D7,
	MX6DL_PAD_SD4_CMD__RAWNAND_RDN,
	MX6DL_PAD_SD4_CLK__RAWNAND_WRN,
	MX6DL_PAD_NANDF_WP_B__RAWNAND_RESETN,
};

static iomux_v3_cfg_t mx6dl_hdmidongle_hdmi_ddc_pads[] = {
	MX6DL_PAD_KEY_COL3__HDMI_TX_DDC_SCL, /* HDMI DDC SCL */
	MX6DL_PAD_KEY_ROW3__HDMI_TX_DDC_SDA, /* HDMI DDC SDA */
};

static iomux_v3_cfg_t mx6dl_hdmidongle_i2c2_pads[] = {
	MX6DL_PAD_KEY_COL3__I2C2_SCL,	/* I2C2 SCL */
	MX6DL_PAD_KEY_ROW3__I2C2_SDA,	/* I2C2 SDA */
};

#endif
