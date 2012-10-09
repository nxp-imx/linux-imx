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
 *
 * Auto Generate file, please don't edit it
 *
 */

#ifndef __MACH_IOMUX_MX6SL_H__
#define __MACH_IOMUX_MX6SL_H__

#include <mach/iomux-v3.h>

#define MX6SL_HIGH_DRV		PAD_CTL_DSE_120ohm
#define MX6SL_DISP_PAD_CLT	MX6SL_HIGH_DRV

#define MX6SL_CCM_CLKO_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
		PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |		\
		PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define MX6SL_UART_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE  |		\
		PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
		PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define MX6SL_USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
		PAD_CTL_PUS_22K_UP  | PAD_CTL_SPEED_LOW |		\
		PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST | PAD_CTL_HYS)

#define MX6SL_USDHC_PAD_CTRL_100MHZ (PAD_CTL_PKE | PAD_CTL_PUE |	\
		PAD_CTL_PUS_22K_UP  | PAD_CTL_SPEED_MED |		\
		PAD_CTL_DSE_34ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define MX6SL_USDHC_PAD_CTRL_200MHZ (PAD_CTL_PKE | PAD_CTL_PUE |	\
		PAD_CTL_PUS_22K_UP  | PAD_CTL_SPEED_HIGH |		\
		PAD_CTL_DSE_34ohm   | PAD_CTL_SRE_FAST   | PAD_CTL_HYS)

#define MX6SL_ENET_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE  |		\
		PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
		PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

#define MX6SL_I2C_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE   |		\
		PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
		PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  |		\
		PAD_CTL_HYS         | PAD_CTL_ODE)

#define MX6SL_ECSPI_PAD_CTRL (PAD_CTL_SRE_FAST | PAD_CTL_SPEED_MED |	\
		PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

#define MX6SL_USB_HSIC_PAD_CTRL	(PAD_CTL_PKE | PAD_CTL_PUE |		\
		PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

#define MX6SL_HP_DET_PAD_CTRL	(PAD_CTL_PKE | PAD_CTL_PUE |		\
		PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
		PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)
#define MX6SL_LCDIF_PAD_CTRL	(PAD_CTL_HYS | PAD_CTL_PUS_100K_UP |	\
				PAD_CTL_PUE | PAD_CTL_PKE |		\
				PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm)

#define MX6SL_KEYPAD_CTRL	(PAD_CTL_HYS | PAD_CTL_PKE | PAD_CTL_PUE | \
				PAD_CTL_PUS_100K_UP | PAD_CTL_DSE_120ohm)

#define MX6SL_TSPAD_CTRL	(PAD_CTL_HYS | PAD_CTL_PKE | PAD_CTL_PUE | \
				PAD_CTL_PUS_47K_UP)
#define MX6SL_ADU_PAD_CTRL	(PAD_CTL_PKE | PAD_CTL_PUE |		\
		PAD_CTL_DSE_40ohm | PAD_CTL_PUS_100K_DOWN |		\
		PAD_CTL_HYS | PAD_CTL_SPEED_MED)
#define MX6SL_CHG_PAD_CTRL	(PAD_CTL_HYS | PAD_CTL_PKE | PAD_CTL_PUE | \
		PAD_CTL_PUS_47K_UP)


#define MX6SL_PAD_AUD_MCLK		0x02A4
#define MX6SL_PAD_AUD_RXD		0x02AC
#define MX6SL_PAD_AUD_TXC		0x02B4
#define MX6SL_PAD_AUD_TXD		0x02B8
#define MX6SL_PAD_AUD_TXFS		0x02BC
#define MX6SL_PAD_HSIC_DAT		0x0444
#define MX6SL_PAD_HSIC_STROBE		0x0448

#define MX6SL_PAD_AUD_MCLK__AUDMUX_AUDIO_CLK_OUT                              \
		IOMUX_PAD(0x02A4, 0x004C, 0, 0x0000, 0, MX6SL_ADU_PAD_CTRL)
#define MX6SL_PAD_AUD_MCLK__PWM4_PWMO                                         \
		IOMUX_PAD(0x02A4, 0x004C, 1, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_MCLK__ECSPI3_RDY                                        \
		IOMUX_PAD(0x02A4, 0x004C, 2, 0x06B4, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_MCLK__FEC_MDC                                           \
		IOMUX_PAD(0x02A4, 0x004C, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_MCLK__WDOG2_WDOG_RST_B_DEB                              \
		IOMUX_PAD(0x02A4, 0x004C, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_MCLK__GPIO_1_6                                          \
		IOMUX_PAD(0x02A4, 0x004C, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_MCLK__SPDIF_SPDIF_EXT_CLK                               \
		IOMUX_PAD(0x02A4, 0x004C, 6, 0x07F4, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_MCLK__TPSMP_HDATA_27                                    \
		IOMUX_PAD(0x02A4, 0x004C, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_AUD_RXC__AUDMUX_AUD3_RXC                                    \
		IOMUX_PAD(0x02A8, 0x0050, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_RXC__I2C1_SDA                                           \
		IOMUX_PAD(0x02A8, 0x0050, 1 | IOMUX_CONFIG_SION, 0x0720, 0, MX6SL_I2C_PAD_CTRL)
#define MX6SL_PAD_AUD_RXC__UART3_TXD                                          \
		IOMUX_PAD(0x02A8, 0x0050, 2, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_AUD_RXC__UART3_RXD                                          \
		IOMUX_PAD(0x02A8, 0x0050, 2, 0x080C, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_AUD_RXC__FEC_TX_CLK                                         \
		IOMUX_PAD(0x02A8, 0x0050, 3, 0x070C, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_RXC__I2C3_SDA                                           \
		IOMUX_PAD(0x02A8, 0x0050, 4 | IOMUX_CONFIG_SION, 0x0730, 0, MX6SL_I2C_PAD_CTRL)
#define MX6SL_PAD_AUD_RXC__GPIO_1_1                                           \
		IOMUX_PAD(0x02A8, 0x0050, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_RXC__ECSPI3_SS1                                         \
		IOMUX_PAD(0x02A8, 0x0050, 6, 0x06C4, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_RXC__PL301_SIM_MX6SL_PER1_HREADYOUT                     \
		IOMUX_PAD(0x02A8, 0x0050, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_AUD_RXD__AUDMUX_AUD3_RXD                                    \
		IOMUX_PAD(0x02AC, 0x0054, 0, 0x0000, 0, MX6SL_ADU_PAD_CTRL)
#define MX6SL_PAD_AUD_RXD__ECSPI3_MOSI                                        \
		IOMUX_PAD(0x02AC, 0x0054, 1, 0x06BC, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_RXD__UART4_TXD                                          \
		IOMUX_PAD(0x02AC, 0x0054, 2, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_AUD_RXD__UART4_RXD                                          \
		IOMUX_PAD(0x02AC, 0x0054, 2, 0x0814, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_AUD_RXD__FEC_RX_ER                                          \
		IOMUX_PAD(0x02AC, 0x0054, 3, 0x0708, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_RXD__USDHC1_LCTL                                        \
		IOMUX_PAD(0x02AC, 0x0054, 4, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_AUD_RXD__GPIO_1_2                                           \
		IOMUX_PAD(0x02AC, 0x0054, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_RXD__SRC_INT_BOOT                                       \
		IOMUX_PAD(0x02AC, 0x0054, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_RXD__PL301_SIM_MX6SL_PER1_HRESP                         \
		IOMUX_PAD(0x02AC, 0x0054, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_AUD_RXFS__AUDMUX_AUD3_RXFS                                  \
		IOMUX_PAD(0x02B0, 0x0058, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_RXFS__I2C1_SCL                                          \
		IOMUX_PAD(0x02B0, 0x0058, 1 | IOMUX_CONFIG_SION, 0x071C, 0, MX6SL_I2C_PAD_CTRL)
#define MX6SL_PAD_AUD_RXFS__UART3_TXD                                         \
		IOMUX_PAD(0x02B0, 0x0058, 2, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_AUD_RXFS__UART3_RXD                                         \
		IOMUX_PAD(0x02B0, 0x0058, 2, 0x080C, 1, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_AUD_RXFS__FEC_MDIO                                          \
		IOMUX_PAD(0x02B0, 0x0058, 3, 0x06F4, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_RXFS__I2C3_SCL                                          \
		IOMUX_PAD(0x02B0, 0x0058, 4 | IOMUX_CONFIG_SION, 0x072C, 0, MX6SL_I2C_PAD_CTRL)
#define MX6SL_PAD_AUD_RXFS__GPIO_1_0                                          \
		IOMUX_PAD(0x02B0, 0x0058, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_RXFS__ECSPI3_SS0                                        \
		IOMUX_PAD(0x02B0, 0x0058, 6, 0x06C0, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_RXFS__PL301_SIM_MX6SL_PER1_HPROT_1                      \
		IOMUX_PAD(0x02B0, 0x0058, 7, 0x07EC, 0, NO_PAD_CTRL)

#define MX6SL_PAD_AUD_TXC__AUDMUX_AUD3_TXC                                    \
		IOMUX_PAD(0x02B4, 0x005C, 0, 0x0000, 0, MX6SL_ADU_PAD_CTRL)
#define MX6SL_PAD_AUD_TXC__ECSPI3_MISO                                        \
		IOMUX_PAD(0x02B4, 0x005C, 1, 0x06B8, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_TXC__UART4_TXD                                          \
		IOMUX_PAD(0x02B4, 0x005C, 2, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_AUD_TXC__UART4_RXD                                          \
		IOMUX_PAD(0x02B4, 0x005C, 2, 0x0814, 1, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_AUD_TXC__FEC_RX_DV                                          \
		IOMUX_PAD(0x02B4, 0x005C, 3, 0x0704, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_TXC__USDHC2_LCTL                                        \
		IOMUX_PAD(0x02B4, 0x005C, 4, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_AUD_TXC__GPIO_1_3                                           \
		IOMUX_PAD(0x02B4, 0x005C, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_TXC__SRC_SYSTEM_RST                                     \
		IOMUX_PAD(0x02B4, 0x005C, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_TXC__TPSMP_HDATA_24                                     \
		IOMUX_PAD(0x02B4, 0x005C, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_AUD_TXD__AUDMUX_AUD3_TXD                                    \
		IOMUX_PAD(0x02B8, 0x0060, 0, 0x0000, 0, MX6SL_ADU_PAD_CTRL)
#define MX6SL_PAD_AUD_TXD__ECSPI3_SCLK                                        \
		IOMUX_PAD(0x02B8, 0x0060, 1, 0x06B0, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_TXD__UART4_CTS                                          \
		IOMUX_PAD(0x02B8, 0x0060, 2, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_AUD_TXD__UART4_RTS                                          \
		IOMUX_PAD(0x02B8, 0x0060, 2, 0x0810, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_AUD_TXD__FEC_TDATA_0                                        \
		IOMUX_PAD(0x02B8, 0x0060, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_TXD__USDHC4_LCTL                                        \
		IOMUX_PAD(0x02B8, 0x0060, 4, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_AUD_TXD__GPIO_1_5                                           \
		IOMUX_PAD(0x02B8, 0x0060, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_TXD__ANATOP_ANATOP_TESTI_1                              \
		IOMUX_PAD(0x02B8, 0x0060, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_TXD__TPSMP_HDATA_26                                     \
		IOMUX_PAD(0x02B8, 0x0060, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_AUD_TXFS__AUDMUX_AUD3_TXFS                                  \
		IOMUX_PAD(0x02BC, 0x0064, 0, 0x0000, 0, MX6SL_ADU_PAD_CTRL)
#define MX6SL_PAD_AUD_TXFS__PWM3_PWMO                                         \
		IOMUX_PAD(0x02BC, 0x0064, 1, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_TXFS__UART4_CTS                                         \
		IOMUX_PAD(0x02BC, 0x0064, 2, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_AUD_TXFS__UART4_RTS                                         \
		IOMUX_PAD(0x02BC, 0x0064, 2, 0x0810, 1, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_AUD_TXFS__FEC_RDATA_1                                       \
		IOMUX_PAD(0x02BC, 0x0064, 3, 0x06FC, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_TXFS__USDHC3_LCTL                                       \
		IOMUX_PAD(0x02BC, 0x0064, 4, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_AUD_TXFS__GPIO_1_4                                          \
		IOMUX_PAD(0x02BC, 0x0064, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_TXFS__ANATOP_ANATOP_TESTI_0                             \
		IOMUX_PAD(0x02BC, 0x0064, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_AUD_TXFS__TPSMP_HDATA_25                                    \
		IOMUX_PAD(0x02BC, 0x0064, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_BOOT_MODE0__SRC_BOOT_MODE_0                                 \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_BOOT_MODE1__SRC_BOOT_MODE_1                                 \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_A0__MMDC_DRAM_A_0                                      \
		IOMUX_PAD(0x02C0, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_A1__MMDC_DRAM_A_1                                      \
		IOMUX_PAD(0x02C4, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_A10__MMDC_DRAM_A_10                                    \
		IOMUX_PAD(0x02C8, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_A11__MMDC_DRAM_A_11                                    \
		IOMUX_PAD(0x02CC, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_A12__MMDC_DRAM_A_12                                    \
		IOMUX_PAD(0x02D0, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_A13__MMDC_DRAM_A_13                                    \
		IOMUX_PAD(0x02D4, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_A14__MMDC_DRAM_A_14                                    \
		IOMUX_PAD(0x02D8, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_A15__MMDC_DRAM_A_15                                    \
		IOMUX_PAD(0x02DC, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_A2__MMDC_DRAM_A_2                                      \
		IOMUX_PAD(0x02E0, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_A3__MMDC_DRAM_A_3                                      \
		IOMUX_PAD(0x02E4, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_A4__MMDC_DRAM_A_4                                      \
		IOMUX_PAD(0x02E8, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_A5__MMDC_DRAM_A_5                                      \
		IOMUX_PAD(0x02EC, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_A6__MMDC_DRAM_A_6                                      \
		IOMUX_PAD(0x02F0, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_A7__MMDC_DRAM_A_7                                      \
		IOMUX_PAD(0x02F4, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_A8__MMDC_DRAM_A_8                                      \
		IOMUX_PAD(0x02F8, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_A9__MMDC_DRAM_A_9                                      \
		IOMUX_PAD(0x02FC, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_CAS__MMDC_DRAM_CAS                                     \
		IOMUX_PAD(0x0300, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_CS0__MMDC_DRAM_CS_0                                    \
		IOMUX_PAD(0x0304, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_CS1__MMDC_DRAM_CS_1                                    \
		IOMUX_PAD(0x0308, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D0__MMDC_DRAM_D_0                                      \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D1__MMDC_DRAM_D_1                                      \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D10__MMDC_DRAM_D_10                                    \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D11__MMDC_DRAM_D_11                                    \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D12__MMDC_DRAM_D_12                                    \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D13__MMDC_DRAM_D_13                                    \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D14__MMDC_DRAM_D_14                                    \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D15__MMDC_DRAM_D_15                                    \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D16__MMDC_DRAM_D_16                                    \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D17__MMDC_DRAM_D_17                                    \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D18__MMDC_DRAM_D_18                                    \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D19__MMDC_DRAM_D_19                                    \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D2__MMDC_DRAM_D_2                                      \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D20__MMDC_DRAM_D_20                                    \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D21__MMDC_DRAM_D_21                                    \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D22__MMDC_DRAM_D_22                                    \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D23__MMDC_DRAM_D_23                                    \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D24__MMDC_DRAM_D_24                                    \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D25__MMDC_DRAM_D_25                                    \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D26__MMDC_DRAM_D_26                                    \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D27__MMDC_DRAM_D_27                                    \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D28__MMDC_DRAM_D_28                                    \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D29__MMDC_DRAM_D_29                                    \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D3__MMDC_DRAM_D_3                                      \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D30__MMDC_DRAM_D_30                                    \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D31__MMDC_DRAM_D_31                                    \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D4__MMDC_DRAM_D_4                                      \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D5__MMDC_DRAM_D_5                                      \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D6__MMDC_DRAM_D_6                                      \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D7__MMDC_DRAM_D_7                                      \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D8__MMDC_DRAM_D_8                                      \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_D9__MMDC_DRAM_D_9                                      \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_DQM0__MMDC_DRAM_DQM_0                                  \
		IOMUX_PAD(0x030C, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_DQM1__MMDC_DRAM_DQM_1                                  \
		IOMUX_PAD(0x0310, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_DQM2__MMDC_DRAM_DQM_2                                  \
		IOMUX_PAD(0x0314, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_DQM3__MMDC_DRAM_DQM_3                                  \
		IOMUX_PAD(0x0318, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_RAS__MMDC_DRAM_RAS                                     \
		IOMUX_PAD(0x031C, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_RESET__MMDC_DRAM_RESET                                 \
		IOMUX_PAD(0x0320, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_SDBA0__MMDC_DRAM_SDBA_0                                \
		IOMUX_PAD(0x0324, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_SDBA1__MMDC_DRAM_SDBA_1                                \
		IOMUX_PAD(0x0328, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_SDBA2__MMDC_DRAM_SDBA_2                                \
		IOMUX_PAD(0x032C, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_SDCKE0__MMDC_DRAM_SDCKE_0                              \
		IOMUX_PAD(0x0330, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_SDCKE1__MMDC_DRAM_SDCKE_1                              \
		IOMUX_PAD(0x0334, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_SDCLK_0__MMDC_DRAM_SDCLK0                              \
		IOMUX_PAD(0x0338, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_SDODT0__MMDC_DRAM_ODT_0                                \
		IOMUX_PAD(0x033C, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_SDODT1__MMDC_DRAM_ODT_1                                \
		IOMUX_PAD(0x0340, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_SDQS0__MMDC_DRAM_SDQS_0                                \
		IOMUX_PAD(0x0344, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_SDQS1__MMDC_DRAM_SDQS_1                                \
		IOMUX_PAD(0x0348, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_SDQS2__MMDC_DRAM_SDQS_2                                \
		IOMUX_PAD(0x034C, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_SDQS3__MMDC_DRAM_SDQS_3                                \
		IOMUX_PAD(0x0350, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_DRAM_SDWE__MMDC_DRAM_SDWE                                   \
		IOMUX_PAD(0x0354, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_ECSPI1_MISO__ECSPI1_MISO                                    \
		IOMUX_PAD(0x0358, 0x0068, 0, 0x0684, 0, MX6SL_ECSPI_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_MISO__AUDMUX_AUD4_TXFS                               \
		IOMUX_PAD(0x0358, 0x0068, 1, 0x05F8, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_MISO__UART5_CTS                                      \
		IOMUX_PAD(0x0358, 0x0068, 2, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_MISO__UART5_RTS                                      \
		IOMUX_PAD(0x0358, 0x0068, 2, 0x0818, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_MISO__EPDC_BDR_0                                     \
		IOMUX_PAD(0x0358, 0x0068, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_MISO__USDHC2_WP                                      \
		IOMUX_PAD(0x0358, 0x0068, 4, 0x0834, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_MISO__GPIO_4_10                                      \
		IOMUX_PAD(0x0358, 0x0068, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_MISO__CCM_PLL3_BYP                                   \
		IOMUX_PAD(0x0358, 0x0068, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_MISO__MMDC_MMDC_DEBUG_40                             \
		IOMUX_PAD(0x0358, 0x0068, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_ECSPI1_MOSI__ECSPI1_MOSI                                    \
		IOMUX_PAD(0x035C, 0x006C, 0, 0x0688, 0, MX6SL_ECSPI_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_MOSI__AUDMUX_AUD4_TXC                                \
		IOMUX_PAD(0x035C, 0x006C, 1, 0x05F4, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_MOSI__UART5_TXD                                      \
		IOMUX_PAD(0x035C, 0x006C, 2, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_MOSI__UART5_RXD                                      \
		IOMUX_PAD(0x035C, 0x006C, 2, 0x081C, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_MOSI__EPDC_VCOM_1                                    \
		IOMUX_PAD(0x035C, 0x006C, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_MOSI__USDHC2_VSELECT                                 \
		IOMUX_PAD(0x035C, 0x006C, 4, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_MOSI__GPIO_4_9                                       \
		IOMUX_PAD(0x035C, 0x006C, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_MOSI__CCM_PLL2_BYP                                   \
		IOMUX_PAD(0x035C, 0x006C, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_MOSI__MMDC_MMDC_DEBUG_49                             \
		IOMUX_PAD(0x035C, 0x006C, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_ECSPI1_SCLK__ECSPI1_SCLK                                    \
		IOMUX_PAD(0x0360, 0x0070, 0, 0x067C, 0, MX6SL_ECSPI_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_SCLK__AUDMUX_AUD4_TXD                                \
		IOMUX_PAD(0x0360, 0x0070, 1, 0x05E8, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_SCLK__UART5_TXD                                      \
		IOMUX_PAD(0x0360, 0x0070, 2, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_SCLK__UART5_RXD                                      \
		IOMUX_PAD(0x0360, 0x0070, 2, 0x081C, 1, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_SCLK__EPDC_VCOM_0                                    \
		IOMUX_PAD(0x0360, 0x0070, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_SCLK__USDHC2_RST                                     \
		IOMUX_PAD(0x0360, 0x0070, 4, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_SCLK__GPIO_4_8                                       \
		IOMUX_PAD(0x0360, 0x0070, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_SCLK__USB_USBOTG2_OC                                 \
		IOMUX_PAD(0x0360, 0x0070, 6, 0x0820, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_SCLK__TPSMP_HDATA_18                                 \
		IOMUX_PAD(0x0360, 0x0070, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_ECSPI1_SS0__ECSPI1_SS0                                      \
		IOMUX_PAD(0x0364, 0x0074, 0, 0x068C, 0, MX6SL_ECSPI_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_SS0__AUDMUX_AUD4_RXD                                 \
		IOMUX_PAD(0x0364, 0x0074, 1, 0x05E4, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_SS0__UART5_CTS                                       \
		IOMUX_PAD(0x0364, 0x0074, 2, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_SS0__UART5_RTS                                       \
		IOMUX_PAD(0x0364, 0x0074, 2, 0x0818, 1, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_SS0__EPDC_BDR_1                                      \
		IOMUX_PAD(0x0364, 0x0074, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_SS0__USDHC2_CD                                       \
		IOMUX_PAD(0x0364, 0x0074, 4, 0x0830, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_SS0__GPIO_4_11                                       \
		IOMUX_PAD(0x0364, 0x0074, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_SS0__USB_USBOTG2_PWR                                 \
		IOMUX_PAD(0x0364, 0x0074, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI1_SS0__PL301_SIM_MX6SL_PER1_HADDR_23                   \
		IOMUX_PAD(0x0364, 0x0074, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_ECSPI2_MISO__GPIO_4_14                                      \
		IOMUX_PAD(0x0368, 0x0078, 5, 0x0000, 0, MX6SL_CHG_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_MISO__USB_USBOTG1_OC                                 \
		IOMUX_PAD(0x0368, 0x0078, 6, 0x0824, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_MISO__TPSMP_HDATA_23                                 \
		IOMUX_PAD(0x0368, 0x0078, 7, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_MISO__ECSPI2_MISO                                    \
		IOMUX_PAD(0x0368, 0x0078, 0, 0x06A0, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_MISO__SDMA_SDMA_EXT_EVENT_0                          \
		IOMUX_PAD(0x0368, 0x0078, 1, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_MISO__UART3_CTS                                      \
		IOMUX_PAD(0x0368, 0x0078, 2, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_MISO__UART3_RTS                                      \
		IOMUX_PAD(0x0368, 0x0078, 2, 0x0808, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_MISO__CSI_MCLK                                       \
		IOMUX_PAD(0x0368, 0x0078, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_MISO__USDHC1_WP                                      \
		IOMUX_PAD(0x0368, 0x0078, 4, 0x082C, 0, MX6SL_USDHC_PAD_CTRL)

#define MX6SL_PAD_ECSPI2_MOSI__ECSPI2_MOSI                                    \
		IOMUX_PAD(0x036C, 0x007C, 0, 0x06A4, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_MOSI__SDMA_SDMA_EXT_EVENT_1                          \
		IOMUX_PAD(0x036C, 0x007C, 1, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_MOSI__UART3_TXD                                      \
		IOMUX_PAD(0x036C, 0x007C, 2, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_MOSI__UART3_RXD                                      \
		IOMUX_PAD(0x036C, 0x007C, 2, 0x080C, 2, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_MOSI__CSI_HSYNC                                      \
		IOMUX_PAD(0x036C, 0x007C, 3, 0x0670, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_MOSI__USDHC1_VSELECT                                 \
		IOMUX_PAD(0x036C, 0x007C, 4, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_MOSI__GPIO_4_13                                      \
		IOMUX_PAD(0x036C, 0x007C, 5, 0x0000, 0, MX6SL_CHG_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_MOSI__ANATOP_ANATOP_TESTO_1                          \
		IOMUX_PAD(0x036C, 0x007C, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_MOSI__TPSMP_HDATA_22                                 \
		IOMUX_PAD(0x036C, 0x007C, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_ECSPI2_SCLK__ECSPI2_SCLK                                    \
		IOMUX_PAD(0x0370, 0x0080, 0, 0x069C, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_SCLK__SPDIF_SPDIF_EXT_CLK                            \
		IOMUX_PAD(0x0370, 0x0080, 1, 0x07F4, 1, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_SCLK__UART3_TXD                                      \
		IOMUX_PAD(0x0370, 0x0080, 2, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_SCLK__UART3_RXD                                      \
		IOMUX_PAD(0x0370, 0x0080, 2, 0x080C, 3, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_SCLK__CSI_PIXCLK                                     \
		IOMUX_PAD(0x0370, 0x0080, 3, 0x0674, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_SCLK__USDHC1_RST                                     \
		IOMUX_PAD(0x0370, 0x0080, 4, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_SCLK__GPIO_4_12                                      \
		IOMUX_PAD(0x0370, 0x0080, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_SCLK__USB_USBOTG2_OC                                 \
		IOMUX_PAD(0x0370, 0x0080, 6, 0x0820, 1, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_SCLK__TPSMP_HDATA_21                                 \
		IOMUX_PAD(0x0370, 0x0080, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_ECSPI2_SS0__ECSPI2_SS0                                      \
		IOMUX_PAD(0x0374, 0x0084, 0, 0x06A8, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_SS0__ECSPI1_SS3                                      \
		IOMUX_PAD(0x0374, 0x0084, 1, 0x0698, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_SS0__UART3_CTS                                       \
		IOMUX_PAD(0x0374, 0x0084, 2, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_SS0__UART3_RTS                                       \
		IOMUX_PAD(0x0374, 0x0084, 2, 0x0808, 1, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_SS0__CSI_VSYNC                                       \
		IOMUX_PAD(0x0374, 0x0084, 3, 0x0678, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_SS0__USDHC1_CD                                       \
		IOMUX_PAD(0x0374, 0x0084, 4, 0x0828, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_SS0__GPIO_4_15                                       \
		IOMUX_PAD(0x0374, 0x0084, 5, 0x0000, 0, MX6SL_CHG_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_SS0__USB_USBOTG1_PWR                                 \
		IOMUX_PAD(0x0374, 0x0084, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_ECSPI2_SS0__PL301_SIM_MX6SL_PER1_HADDR_24                   \
		IOMUX_PAD(0x0374, 0x0084, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_BDR0__EPDC_BDR_0                                       \
		IOMUX_PAD(0x0378, 0x0088, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_BDR0__USDHC4_CLK                                       \
		IOMUX_PAD(0x0378, 0x0088, 1, 0x0850, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_EPDC_BDR0__UART3_CTS                                        \
		IOMUX_PAD(0x0378, 0x0088, 2, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_EPDC_BDR0__UART3_RTS                                        \
		IOMUX_PAD(0x0378, 0x0088, 2, 0x0808, 2, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_EPDC_BDR0__WEIM_WEIM_A_26                                   \
		IOMUX_PAD(0x0378, 0x0088, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_BDR0__TCON_RL                                          \
		IOMUX_PAD(0x0378, 0x0088, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_BDR0__GPIO_2_5                                         \
		IOMUX_PAD(0x0378, 0x0088, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_BDR0__EPDC_SDCE_7                                      \
		IOMUX_PAD(0x0378, 0x0088, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_BDR0__MMDC_MMDC_DEBUG_9                                \
		IOMUX_PAD(0x0378, 0x0088, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_BDR1__EPDC_BDR_1                                       \
		IOMUX_PAD(0x037C, 0x008C, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_BDR1__USDHC4_CMD                                       \
		IOMUX_PAD(0x037C, 0x008C, 1, 0x0858, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_EPDC_BDR1__UART3_CTS                                        \
		IOMUX_PAD(0x037C, 0x008C, 2, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_EPDC_BDR1__UART3_RTS                                        \
		IOMUX_PAD(0x037C, 0x008C, 2, 0x0808, 3, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_EPDC_BDR1__WEIM_WEIM_CRE                                    \
		IOMUX_PAD(0x037C, 0x008C, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_BDR1__TCON_UD                                          \
		IOMUX_PAD(0x037C, 0x008C, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_BDR1__GPIO_2_6                                         \
		IOMUX_PAD(0x037C, 0x008C, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_BDR1__EPDC_SDCE_8                                      \
		IOMUX_PAD(0x037C, 0x008C, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_BDR1__MMDC_MMDC_DEBUG_8                                \
		IOMUX_PAD(0x037C, 0x008C, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_D0__EPDC_SDDO_0                                        \
		IOMUX_PAD(0x0380, 0x0090, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D0__ECSPI4_MOSI                                        \
		IOMUX_PAD(0x0380, 0x0090, 1, 0x06D8, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D0__LCDIF_DAT_24                                       \
		IOMUX_PAD(0x0380, 0x0090, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D0__CSI_D_0                                            \
		IOMUX_PAD(0x0380, 0x0090, 3, 0x0630, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D0__TCON_E_DATA_0                                      \
		IOMUX_PAD(0x0380, 0x0090, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D0__GPIO_1_7                                           \
		IOMUX_PAD(0x0380, 0x0090, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D0__ANATOP_USBPHY1_TSTI_TX_HS_MODE                     \
		IOMUX_PAD(0x0380, 0x0090, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D0__OBSERVE_MUX_OUT_0                                  \
		IOMUX_PAD(0x0380, 0x0090, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_D1__EPDC_SDDO_1                                        \
		IOMUX_PAD(0x0384, 0x0094, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D1__ECSPI4_MISO                                        \
		IOMUX_PAD(0x0384, 0x0094, 1, 0x06D4, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D1__LCDIF_DAT_25                                       \
		IOMUX_PAD(0x0384, 0x0094, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D1__CSI_D_1                                            \
		IOMUX_PAD(0x0384, 0x0094, 3, 0x0634, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D1__TCON_E_DATA_1                                      \
		IOMUX_PAD(0x0384, 0x0094, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D1__GPIO_1_8                                           \
		IOMUX_PAD(0x0384, 0x0094, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D1__ANATOP_USBPHY1_TSTI_TX_LS_MODE                     \
		IOMUX_PAD(0x0384, 0x0094, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D1__OBSERVE_MUX_OUT_1                                  \
		IOMUX_PAD(0x0384, 0x0094, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_D10__EPDC_SDDO_10                                      \
		IOMUX_PAD(0x0388, 0x0098, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D10__ECSPI3_SS0                                        \
		IOMUX_PAD(0x0388, 0x0098, 1, 0x06C0, 1, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D10__EPDC_PWRCTRL_2                                    \
		IOMUX_PAD(0x0388, 0x0098, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D10__WEIM_WEIM_A_18                                    \
		IOMUX_PAD(0x0388, 0x0098, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D10__TCON_E_DATA_10                                    \
		IOMUX_PAD(0x0388, 0x0098, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D10__GPIO_1_17                                         \
		IOMUX_PAD(0x0388, 0x0098, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D10__USDHC4_WP                                         \
		IOMUX_PAD(0x0388, 0x0098, 6, 0x087C, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_EPDC_D10__MMDC_MMDC_DEBUG_29                                \
		IOMUX_PAD(0x0388, 0x0098, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_D11__EPDC_SDDO_11                                      \
		IOMUX_PAD(0x038C, 0x009C, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D11__ECSPI3_SCLK                                       \
		IOMUX_PAD(0x038C, 0x009C, 1, 0x06B0, 1, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D11__EPDC_PWRCTRL_3                                    \
		IOMUX_PAD(0x038C, 0x009C, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D11__WEIM_WEIM_A_19                                    \
		IOMUX_PAD(0x038C, 0x009C, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D11__TCON_E_DATA_11                                    \
		IOMUX_PAD(0x038C, 0x009C, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D11__GPIO_1_18                                         \
		IOMUX_PAD(0x038C, 0x009C, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D11__USDHC4_CD                                         \
		IOMUX_PAD(0x038C, 0x009C, 6, 0x0854, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_EPDC_D11__MMDC_MMDC_DEBUG_28                                \
		IOMUX_PAD(0x038C, 0x009C, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_D12__EPDC_SDDO_12                                      \
		IOMUX_PAD(0x0390, 0x00A0, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D12__UART2_TXD                                         \
		IOMUX_PAD(0x0390, 0x00A0, 1, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_EPDC_D12__UART2_RXD                                         \
		IOMUX_PAD(0x0390, 0x00A0, 1, 0x0804, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_EPDC_D12__EPDC_PWRCOM                                       \
		IOMUX_PAD(0x0390, 0x00A0, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D12__WEIM_WEIM_A_20                                    \
		IOMUX_PAD(0x0390, 0x00A0, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D12__TCON_E_DATA_12                                    \
		IOMUX_PAD(0x0390, 0x00A0, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D12__GPIO_1_19                                         \
		IOMUX_PAD(0x0390, 0x00A0, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D12__ECSPI3_SS1                                        \
		IOMUX_PAD(0x0390, 0x00A0, 6, 0x06C4, 1, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D12__MMDC_MMDC_DEBUG_27                                \
		IOMUX_PAD(0x0390, 0x00A0, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_D13__EPDC_SDDO_13                                      \
		IOMUX_PAD(0x0394, 0x00A4, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D13__UART2_TXD                                         \
		IOMUX_PAD(0x0394, 0x00A4, 1, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_EPDC_D13__UART2_RXD                                         \
		IOMUX_PAD(0x0394, 0x00A4, 1, 0x0804, 1, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_EPDC_D13__EPDC_PWRIRQ                                       \
		IOMUX_PAD(0x0394, 0x00A4, 2, 0x06E8, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D13__WEIM_WEIM_A_21                                    \
		IOMUX_PAD(0x0394, 0x00A4, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D13__TCON_E_DATA_13                                    \
		IOMUX_PAD(0x0394, 0x00A4, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D13__GPIO_1_20                                         \
		IOMUX_PAD(0x0394, 0x00A4, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D13__ECSPI3_SS2                                        \
		IOMUX_PAD(0x0394, 0x00A4, 6, 0x06C8, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D13__MMDC_MMDC_DEBUG_26                                \
		IOMUX_PAD(0x0394, 0x00A4, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_D14__EPDC_SDDO_14                                      \
		IOMUX_PAD(0x0398, 0x00A8, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D14__UART2_CTS                                         \
		IOMUX_PAD(0x0398, 0x00A8, 1, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_EPDC_D14__UART2_RTS                                         \
		IOMUX_PAD(0x0398, 0x00A8, 1, 0x0800, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_EPDC_D14__EPDC_PWRSTAT                                      \
		IOMUX_PAD(0x0398, 0x00A8, 2, 0x06EC, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D14__WEIM_WEIM_A_22                                    \
		IOMUX_PAD(0x0398, 0x00A8, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D14__TCON_E_DATA_14                                    \
		IOMUX_PAD(0x0398, 0x00A8, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D14__GPIO_1_21                                         \
		IOMUX_PAD(0x0398, 0x00A8, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D14__ECSPI3_SS3                                        \
		IOMUX_PAD(0x0398, 0x00A8, 6, 0x06CC, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D14__MMDC_MMDC_DEBUG_25                                \
		IOMUX_PAD(0x0398, 0x00A8, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_D15__EPDC_SDDO_15                                      \
		IOMUX_PAD(0x039C, 0x00AC, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D15__UART2_CTS                                         \
		IOMUX_PAD(0x039C, 0x00AC, 1, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_EPDC_D15__UART2_RTS                                         \
		IOMUX_PAD(0x039C, 0x00AC, 1, 0x0800, 1, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_EPDC_D15__EPDC_PWRWAKE                                      \
		IOMUX_PAD(0x039C, 0x00AC, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D15__WEIM_WEIM_A_23                                    \
		IOMUX_PAD(0x039C, 0x00AC, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D15__TCON_E_DATA_15                                    \
		IOMUX_PAD(0x039C, 0x00AC, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D15__GPIO_1_22                                         \
		IOMUX_PAD(0x039C, 0x00AC, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D15__ECSPI3_RDY                                        \
		IOMUX_PAD(0x039C, 0x00AC, 6, 0x06B4, 1, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D15__MMDC_MMDC_DEBUG_24                                \
		IOMUX_PAD(0x039C, 0x00AC, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_D2__EPDC_SDDO_2                                        \
		IOMUX_PAD(0x03A0, 0x00B0, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D2__ECSPI4_SS0                                         \
		IOMUX_PAD(0x03A0, 0x00B0, 1, 0x06DC, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D2__LCDIF_DAT_26                                       \
		IOMUX_PAD(0x03A0, 0x00B0, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D2__CSI_D_2                                            \
		IOMUX_PAD(0x03A0, 0x00B0, 3, 0x0638, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D2__TCON_E_DATA_2                                      \
		IOMUX_PAD(0x03A0, 0x00B0, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D2__GPIO_1_9                                           \
		IOMUX_PAD(0x03A0, 0x00B0, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D2__ANATOP_USBPHY1_TSTI_TX_DN                          \
		IOMUX_PAD(0x03A0, 0x00B0, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D2__TPSMP_HDATA_28                                     \
		IOMUX_PAD(0x03A0, 0x00B0, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_D3__EPDC_SDDO_3                                        \
		IOMUX_PAD(0x03A4, 0x00B4, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D3__ECSPI4_SCLK                                        \
		IOMUX_PAD(0x03A4, 0x00B4, 1, 0x06D0, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D3__LCDIF_DAT_27                                       \
		IOMUX_PAD(0x03A4, 0x00B4, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D3__CSI_D_3                                            \
		IOMUX_PAD(0x03A4, 0x00B4, 3, 0x063C, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D3__TCON_E_DATA_3                                      \
		IOMUX_PAD(0x03A4, 0x00B4, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D3__GPIO_1_10                                          \
		IOMUX_PAD(0x03A4, 0x00B4, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D3__ANATOP_USBPHY1_TSTI_TX_DP                          \
		IOMUX_PAD(0x03A4, 0x00B4, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D3__TPSMP_HDATA_29                                     \
		IOMUX_PAD(0x03A4, 0x00B4, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_D4__EPDC_SDDO_4                                        \
		IOMUX_PAD(0x03A8, 0x00B8, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D4__ECSPI4_SS1                                         \
		IOMUX_PAD(0x03A8, 0x00B8, 1, 0x06E0, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D4__LCDIF_DAT_28                                       \
		IOMUX_PAD(0x03A8, 0x00B8, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D4__CSI_D_4                                            \
		IOMUX_PAD(0x03A8, 0x00B8, 3, 0x0640, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D4__TCON_E_DATA_4                                      \
		IOMUX_PAD(0x03A8, 0x00B8, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D4__GPIO_1_11                                          \
		IOMUX_PAD(0x03A8, 0x00B8, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D4__ANATOP_USBPHY1_TSTI_TX_EN                          \
		IOMUX_PAD(0x03A8, 0x00B8, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D4__TPSMP_HDATA_30                                     \
		IOMUX_PAD(0x03A8, 0x00B8, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_D5__EPDC_SDDO_5                                        \
		IOMUX_PAD(0x03AC, 0x00BC, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D5__ECSPI4_SS2                                         \
		IOMUX_PAD(0x03AC, 0x00BC, 1, 0x06E4, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D5__LCDIF_DAT_29                                       \
		IOMUX_PAD(0x03AC, 0x00BC, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D5__CSI_D_5                                            \
		IOMUX_PAD(0x03AC, 0x00BC, 3, 0x0644, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D5__TCON_E_DATA_5                                      \
		IOMUX_PAD(0x03AC, 0x00BC, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D5__GPIO_1_12                                          \
		IOMUX_PAD(0x03AC, 0x00BC, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D5__ANATOP_USBPHY1_TSTI_TX_HIZ                         \
		IOMUX_PAD(0x03AC, 0x00BC, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D5__TPSMP_HDATA_31                                     \
		IOMUX_PAD(0x03AC, 0x00BC, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_D6__EPDC_SDDO_6                                        \
		IOMUX_PAD(0x03B0, 0x00C0, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D6__ECSPI4_SS3                                         \
		IOMUX_PAD(0x03B0, 0x00C0, 1, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D6__LCDIF_DAT_30                                       \
		IOMUX_PAD(0x03B0, 0x00C0, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D6__CSI_D_6                                            \
		IOMUX_PAD(0x03B0, 0x00C0, 3, 0x0648, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D6__TCON_E_DATA_6                                      \
		IOMUX_PAD(0x03B0, 0x00C0, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D6__GPIO_1_13                                          \
		IOMUX_PAD(0x03B0, 0x00C0, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D6__ANATOP_USBPHY2_TSTO_RX_DISCON_DET                  \
		IOMUX_PAD(0x03B0, 0x00C0, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D6__TPSMP_HDATA_20                                     \
		IOMUX_PAD(0x03B0, 0x00C0, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_D7__EPDC_SDDO_7                                        \
		IOMUX_PAD(0x03B4, 0x00C4, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D7__ECSPI4_RDY                                         \
		IOMUX_PAD(0x03B4, 0x00C4, 1, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D7__LCDIF_DAT_31                                       \
		IOMUX_PAD(0x03B4, 0x00C4, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D7__CSI_D_7                                            \
		IOMUX_PAD(0x03B4, 0x00C4, 3, 0x064C, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D7__TCON_E_DATA_7                                      \
		IOMUX_PAD(0x03B4, 0x00C4, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D7__GPIO_1_14                                          \
		IOMUX_PAD(0x03B4, 0x00C4, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D7__ANATOP_USBPHY2_TSTO_RX_FS_RXD                      \
		IOMUX_PAD(0x03B4, 0x00C4, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D7__MMDC_MMDC_DEBUG_32                                 \
		IOMUX_PAD(0x03B4, 0x00C4, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_D8__EPDC_SDDO_8                                        \
		IOMUX_PAD(0x03B8, 0x00C8, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D8__ECSPI3_MOSI                                        \
		IOMUX_PAD(0x03B8, 0x00C8, 1, 0x06BC, 1, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D8__EPDC_PWRCTRL_0                                     \
		IOMUX_PAD(0x03B8, 0x00C8, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D8__WEIM_WEIM_A_16                                     \
		IOMUX_PAD(0x03B8, 0x00C8, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D8__TCON_E_DATA_8                                      \
		IOMUX_PAD(0x03B8, 0x00C8, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D8__GPIO_1_15                                          \
		IOMUX_PAD(0x03B8, 0x00C8, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D8__USDHC4_RST                                         \
		IOMUX_PAD(0x03B8, 0x00C8, 6, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_EPDC_D8__MMDC_MMDC_DEBUG_31                                 \
		IOMUX_PAD(0x03B8, 0x00C8, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_D9__EPDC_SDDO_9                                        \
		IOMUX_PAD(0x03BC, 0x00CC, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D9__ECSPI3_MISO                                        \
		IOMUX_PAD(0x03BC, 0x00CC, 1, 0x06B8, 1, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D9__EPDC_PWRCTRL_1                                     \
		IOMUX_PAD(0x03BC, 0x00CC, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D9__WEIM_WEIM_A_17                                     \
		IOMUX_PAD(0x03BC, 0x00CC, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D9__TCON_E_DATA_9                                      \
		IOMUX_PAD(0x03BC, 0x00CC, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D9__GPIO_1_16                                          \
		IOMUX_PAD(0x03BC, 0x00CC, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_D9__USDHC4_VSELECT                                     \
		IOMUX_PAD(0x03BC, 0x00CC, 6, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_EPDC_D9__MMDC_MMDC_DEBUG_30                                 \
		IOMUX_PAD(0x03BC, 0x00CC, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_GDCLK__EPDC_GDCLK                                      \
		IOMUX_PAD(0x03C0, 0x00D0, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_GDCLK__ECSPI2_SS2                                      \
		IOMUX_PAD(0x03C0, 0x00D0, 1, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_GDCLK__TCON_YCKR                                       \
		IOMUX_PAD(0x03C0, 0x00D0, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_GDCLK__CSI_PIXCLK                                      \
		IOMUX_PAD(0x03C0, 0x00D0, 3, 0x0674, 1, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_GDCLK__TCON_YCKL                                       \
		IOMUX_PAD(0x03C0, 0x00D0, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_GDCLK__GPIO_1_31                                       \
		IOMUX_PAD(0x03C0, 0x00D0, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_GDCLK__USDHC2_RST                                      \
		IOMUX_PAD(0x03C0, 0x00D0, 6, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_EPDC_GDCLK__MMDC_MMDC_DEBUG_15                              \
		IOMUX_PAD(0x03C0, 0x00D0, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_GDOE__EPDC_GDOE                                        \
		IOMUX_PAD(0x03C4, 0x00D4, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_GDOE__ECSPI2_SS3                                       \
		IOMUX_PAD(0x03C4, 0x00D4, 1, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_GDOE__TCON_YOER                                        \
		IOMUX_PAD(0x03C4, 0x00D4, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_GDOE__CSI_HSYNC                                        \
		IOMUX_PAD(0x03C4, 0x00D4, 3, 0x0670, 1, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_GDOE__TCON_YOEL                                        \
		IOMUX_PAD(0x03C4, 0x00D4, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_GDOE__GPIO_2_0                                         \
		IOMUX_PAD(0x03C4, 0x00D4, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_GDOE__USDHC2_VSELECT                                   \
		IOMUX_PAD(0x03C4, 0x00D4, 6, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_EPDC_GDOE__MMDC_MMDC_DEBUG_14                               \
		IOMUX_PAD(0x03C4, 0x00D4, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_GDRL__EPDC_GDRL                                        \
		IOMUX_PAD(0x03C8, 0x00D8, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_GDRL__ECSPI2_RDY                                       \
		IOMUX_PAD(0x03C8, 0x00D8, 1, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_GDRL__TCON_YDIOUR                                      \
		IOMUX_PAD(0x03C8, 0x00D8, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_GDRL__CSI_MCLK                                         \
		IOMUX_PAD(0x03C8, 0x00D8, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_GDRL__TCON_YDIOUL                                      \
		IOMUX_PAD(0x03C8, 0x00D8, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_GDRL__GPIO_2_1                                         \
		IOMUX_PAD(0x03C8, 0x00D8, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_GDRL__USDHC2_WP                                        \
		IOMUX_PAD(0x03C8, 0x00D8, 6, 0x0834, 1, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_EPDC_GDRL__MMDC_MMDC_DEBUG_13                               \
		IOMUX_PAD(0x03C8, 0x00D8, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_GDSP__EPDC_GDSP                                        \
		IOMUX_PAD(0x03CC, 0x00DC, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_GDSP__PWM4_PWMO                                        \
		IOMUX_PAD(0x03CC, 0x00DC, 1, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_GDSP__TCON_YDIODR                                      \
		IOMUX_PAD(0x03CC, 0x00DC, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_GDSP__CSI_VSYNC                                        \
		IOMUX_PAD(0x03CC, 0x00DC, 3, 0x0678, 1, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_GDSP__TCON_YDIODL                                      \
		IOMUX_PAD(0x03CC, 0x00DC, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_GDSP__GPIO_2_2                                         \
		IOMUX_PAD(0x03CC, 0x00DC, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_GDSP__USDHC2_CD                                        \
		IOMUX_PAD(0x03CC, 0x00DC, 6, 0x0830, 1, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_EPDC_GDSP__MMDC_MMDC_DEBUG_12                               \
		IOMUX_PAD(0x03CC, 0x00DC, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_PWRCOM__EPDC_PWRCOM                                    \
		IOMUX_PAD(0x03D0, 0x00E0, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCOM__USDHC4_DAT0                                    \
		IOMUX_PAD(0x03D0, 0x00E0, 1, 0x085C, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCOM__LCDIF_DAT_20                                   \
		IOMUX_PAD(0x03D0, 0x00E0, 2, 0x07C8, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCOM__WEIM_WEIM_BCLK                                 \
		IOMUX_PAD(0x03D0, 0x00E0, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCOM__ANATOP_USBOTG1_ID                              \
		IOMUX_PAD(0x03D0, 0x00E0, 4, 0x05DC, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCOM__GPIO_2_11                                      \
		IOMUX_PAD(0x03D0, 0x00E0, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCOM__USDHC3_RST                                     \
		IOMUX_PAD(0x03D0, 0x00E0, 6, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCOM__MMDC_MMDC_DEBUG_3                              \
		IOMUX_PAD(0x03D0, 0x00E0, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_PWRCTRL0__EPDC_PWRCTRL_0                               \
		IOMUX_PAD(0x03D4, 0x00E4, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCTRL0__AUDMUX_AUD5_RXC                              \
		IOMUX_PAD(0x03D4, 0x00E4, 1, 0x0604, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCTRL0__LCDIF_DAT_16                                 \
		IOMUX_PAD(0x03D4, 0x00E4, 2, 0x07B8, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCTRL0__WEIM_WEIM_RW                                 \
		IOMUX_PAD(0x03D4, 0x00E4, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCTRL0__TCON_YCKL                                    \
		IOMUX_PAD(0x03D4, 0x00E4, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCTRL0__GPIO_2_7                                     \
		IOMUX_PAD(0x03D4, 0x00E4, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCTRL0__USDHC4_RST                                   \
		IOMUX_PAD(0x03D4, 0x00E4, 6, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCTRL0__MMDC_MMDC_DEBUG_7                            \
		IOMUX_PAD(0x03D4, 0x00E4, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_PWRCTRL1__EPDC_PWRCTRL_1                               \
		IOMUX_PAD(0x03D8, 0x00E8, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCTRL1__AUDMUX_AUD5_TXFS                             \
		IOMUX_PAD(0x03D8, 0x00E8, 1, 0x0610, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCTRL1__LCDIF_DAT_17                                 \
		IOMUX_PAD(0x03D8, 0x00E8, 2, 0x07BC, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCTRL1__WEIM_WEIM_OE                                 \
		IOMUX_PAD(0x03D8, 0x00E8, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCTRL1__TCON_YOEL                                    \
		IOMUX_PAD(0x03D8, 0x00E8, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCTRL1__GPIO_2_8                                     \
		IOMUX_PAD(0x03D8, 0x00E8, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCTRL1__USDHC4_VSELECT                               \
		IOMUX_PAD(0x03D8, 0x00E8, 6, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCTRL1__MMDC_MMDC_DEBUG_6                            \
		IOMUX_PAD(0x03D8, 0x00E8, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_PWRCTRL2__EPDC_PWRCTRL_2                               \
		IOMUX_PAD(0x03DC, 0x00EC, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCTRL2__AUDMUX_AUD5_TXD                              \
		IOMUX_PAD(0x03DC, 0x00EC, 1, 0x0600, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCTRL2__LCDIF_DAT_18                                 \
		IOMUX_PAD(0x03DC, 0x00EC, 2, 0x07C0, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCTRL2__WEIM_WEIM_CS_0                               \
		IOMUX_PAD(0x03DC, 0x00EC, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCTRL2__TCON_YDIOUL                                  \
		IOMUX_PAD(0x03DC, 0x00EC, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCTRL2__GPIO_2_9                                     \
		IOMUX_PAD(0x03DC, 0x00EC, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCTRL2__USDHC4_WP                                    \
		IOMUX_PAD(0x03DC, 0x00EC, 6, 0x087C, 1, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCTRL2__MMDC_MMDC_DEBUG_5                            \
		IOMUX_PAD(0x03DC, 0x00EC, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_PWRCTRL3__EPDC_PWRCTRL_3                               \
		IOMUX_PAD(0x03E0, 0x00F0, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCTRL3__AUDMUX_AUD5_TXC                              \
		IOMUX_PAD(0x03E0, 0x00F0, 1, 0x060C, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCTRL3__LCDIF_DAT_19                                 \
		IOMUX_PAD(0x03E0, 0x00F0, 2, 0x07C4, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCTRL3__WEIM_WEIM_CS_1                               \
		IOMUX_PAD(0x03E0, 0x00F0, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCTRL3__TCON_YDIODL                                  \
		IOMUX_PAD(0x03E0, 0x00F0, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCTRL3__GPIO_2_10                                    \
		IOMUX_PAD(0x03E0, 0x00F0, 5, 0x0000, 0, MX6SL_TSPAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCTRL3__USDHC4_CD                                    \
		IOMUX_PAD(0x03E0, 0x00F0, 6, 0x0854, 1, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRCTRL3__MMDC_MMDC_DEBUG_4                            \
		IOMUX_PAD(0x03E0, 0x00F0, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_PWRINT__EPDC_PWRIRQ                                    \
		IOMUX_PAD(0x03E4, 0x00F4, 0, 0x06E8, 1, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRINT__USDHC4_DAT1                                    \
		IOMUX_PAD(0x03E4, 0x00F4, 1, 0x0860, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRINT__LCDIF_DAT_21                                   \
		IOMUX_PAD(0x03E4, 0x00F4, 2, 0x07CC, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRINT__WEIM_ACLK_FREERUN                              \
		IOMUX_PAD(0x03E4, 0x00F4, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRINT__ANATOP_USBOTG2_ID                              \
		IOMUX_PAD(0x03E4, 0x00F4, 4, 0x05E0, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRINT__GPIO_2_12                                      \
		IOMUX_PAD(0x03E4, 0x00F4, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRINT__USDHC3_VSELECT                                 \
		IOMUX_PAD(0x03E4, 0x00F4, 6, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRINT__MMDC_MMDC_DEBUG_2                              \
		IOMUX_PAD(0x03E4, 0x00F4, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_PWRSTAT__EPDC_PWRSTAT                                  \
		IOMUX_PAD(0x03E8, 0x00F8, 0, 0x06EC, 1, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRSTAT__USDHC4_DAT2                                   \
		IOMUX_PAD(0x03E8, 0x00F8, 1, 0x0864, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRSTAT__LCDIF_DAT_22                                  \
		IOMUX_PAD(0x03E8, 0x00F8, 2, 0x07D0, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRSTAT__WEIM_WEIM_WAIT                                \
		IOMUX_PAD(0x03E8, 0x00F8, 3, 0x0884, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRSTAT__KITTEN_EVENTI                                 \
		IOMUX_PAD(0x03E8, 0x00F8, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRSTAT__GPIO_2_13                                     \
		IOMUX_PAD(0x03E8, 0x00F8, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRSTAT__USDHC3_WP                                     \
		IOMUX_PAD(0x03E8, 0x00F8, 6, 0x084C, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRSTAT__MMDC_MMDC_DEBUG_1                             \
		IOMUX_PAD(0x03E8, 0x00F8, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_PWRWAKEUP__EPDC_PWRWAKE                                \
		IOMUX_PAD(0x03EC, 0x00FC, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRWAKEUP__USDHC4_DAT3                                 \
		IOMUX_PAD(0x03EC, 0x00FC, 1, 0x0868, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRWAKEUP__LCDIF_DAT_23                                \
		IOMUX_PAD(0x03EC, 0x00FC, 2, 0x07D4, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRWAKEUP__WEIM_WEIM_DTACK_B                           \
		IOMUX_PAD(0x03EC, 0x00FC, 3, 0x0880, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRWAKEUP__KITTEN_EVENTO                               \
		IOMUX_PAD(0x03EC, 0x00FC, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRWAKEUP__GPIO_2_14                                   \
		IOMUX_PAD(0x03EC, 0x00FC, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRWAKEUP__USDHC3_CD                                   \
		IOMUX_PAD(0x03EC, 0x00FC, 6, 0x0838, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_EPDC_PWRWAKEUP__MMDC_MMDC_DEBUG_0                           \
		IOMUX_PAD(0x03EC, 0x00FC, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_SDCE0__EPDC_SDCE_0                                     \
		IOMUX_PAD(0x03F0, 0x0100, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCE0__ECSPI2_SS1                                      \
		IOMUX_PAD(0x03F0, 0x0100, 1, 0x06AC, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCE0__PWM3_PWMO                                       \
		IOMUX_PAD(0x03F0, 0x0100, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCE0__WEIM_WEIM_CS_2                                  \
		IOMUX_PAD(0x03F0, 0x0100, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCE0__TCON_YCKR                                       \
		IOMUX_PAD(0x03F0, 0x0100, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCE0__GPIO_1_27                                       \
		IOMUX_PAD(0x03F0, 0x0100, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCE0__ANATOP_USBPHY1_TSTO_PLL_CLK20DIV                \
		IOMUX_PAD(0x03F0, 0x0100, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCE0__MMDC_MMDC_DEBUG_19                              \
		IOMUX_PAD(0x03F0, 0x0100, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_SDCE1__EPDC_SDCE_1                                     \
		IOMUX_PAD(0x03F4, 0x0104, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCE1__WDOG2_WDOG_B                                    \
		IOMUX_PAD(0x03F4, 0x0104, 1, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCE1__PWM4_PWMO                                       \
		IOMUX_PAD(0x03F4, 0x0104, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCE1__WEIM_WEIM_LBA                                   \
		IOMUX_PAD(0x03F4, 0x0104, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCE1__TCON_YOER                                       \
		IOMUX_PAD(0x03F4, 0x0104, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCE1__GPIO_1_28                                       \
		IOMUX_PAD(0x03F4, 0x0104, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCE1__ANATOP_USBPHY1_TSTO_RX_FS_RXD                   \
		IOMUX_PAD(0x03F4, 0x0104, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCE1__MMDC_MMDC_DEBUG_18                              \
		IOMUX_PAD(0x03F4, 0x0104, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_SDCE2__EPDC_SDCE_2                                     \
		IOMUX_PAD(0x03F8, 0x0108, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCE2__I2C3_SCL                                        \
		IOMUX_PAD(0x03F8, 0x0108, 1 | IOMUX_CONFIG_SION, 0x072C, 1, MX6SL_I2C_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCE2__PWM1_PWMO                                       \
		IOMUX_PAD(0x03F8, 0x0108, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCE2__WEIM_WEIM_EB_0                                  \
		IOMUX_PAD(0x03F8, 0x0108, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCE2__TCON_YDIOUR                                     \
		IOMUX_PAD(0x03F8, 0x0108, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCE2__GPIO_1_29                                       \
		IOMUX_PAD(0x03F8, 0x0108, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCE2__ANATOP_USBPHY1_TSTO_RX_HS_RXD                   \
		IOMUX_PAD(0x03F8, 0x0108, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCE2__MMDC_MMDC_DEBUG_17                              \
		IOMUX_PAD(0x03F8, 0x0108, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_SDCE3__EPDC_SDCE_3                                     \
		IOMUX_PAD(0x03FC, 0x010C, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCE3__I2C3_SDA                                        \
		IOMUX_PAD(0x03FC, 0x010C, 1 | IOMUX_CONFIG_SION, 0x0730, 1, MX6SL_I2C_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCE3__PWM2_PWMO                                       \
		IOMUX_PAD(0x03FC, 0x010C, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCE3__WEIM_WEIM_EB_1                                  \
		IOMUX_PAD(0x03FC, 0x010C, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCE3__TCON_YDIODR                                     \
		IOMUX_PAD(0x03FC, 0x010C, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCE3__GPIO_1_30                                       \
		IOMUX_PAD(0x03FC, 0x010C, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCE3__ANATOP_USBPHY1_TSTO_RX_SQUELCH                  \
		IOMUX_PAD(0x03FC, 0x010C, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCE3__MMDC_MMDC_DEBUG_16                              \
		IOMUX_PAD(0x03FC, 0x010C, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_SDCLK__EPDC_SDCLK                                      \
		IOMUX_PAD(0x0400, 0x0110, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCLK__ECSPI2_MOSI                                     \
		IOMUX_PAD(0x0400, 0x0110, 1, 0x06A4, 1, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCLK__I2C2_SCL                                        \
		IOMUX_PAD(0x0400, 0x0110, 2 | IOMUX_CONFIG_SION, 0x0724, 0, MX6SL_I2C_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCLK__CSI_D_8                                         \
		IOMUX_PAD(0x0400, 0x0110, 3, 0x0650, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCLK__TCON_CL                                         \
		IOMUX_PAD(0x0400, 0x0110, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCLK__GPIO_1_23                                       \
		IOMUX_PAD(0x0400, 0x0110, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCLK__ANATOP_USBPHY2_TSTO_RX_HS_RXD                   \
		IOMUX_PAD(0x0400, 0x0110, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDCLK__MMDC_MMDC_DEBUG_23                              \
		IOMUX_PAD(0x0400, 0x0110, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_SDLE__EPDC_SDLE                                        \
		IOMUX_PAD(0x0404, 0x0114, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDLE__ECSPI2_MISO                                      \
		IOMUX_PAD(0x0404, 0x0114, 1, 0x06A0, 1, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDLE__I2C2_SDA                                         \
		IOMUX_PAD(0x0404, 0x0114, 2 | IOMUX_CONFIG_SION, 0x0728, 0, MX6SL_I2C_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDLE__CSI_D_9                                          \
		IOMUX_PAD(0x0404, 0x0114, 3, 0x0654, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDLE__TCON_LD                                          \
		IOMUX_PAD(0x0404, 0x0114, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDLE__GPIO_1_24                                        \
		IOMUX_PAD(0x0404, 0x0114, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDLE__ANATOP_USBPHY2_TSTO_RX_SQUELCH                   \
		IOMUX_PAD(0x0404, 0x0114, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDLE__MMDC_MMDC_DEBUG_22                               \
		IOMUX_PAD(0x0404, 0x0114, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_SDOE__EPDC_SDOE                                        \
		IOMUX_PAD(0x0408, 0x0118, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDOE__ECSPI2_SS0                                       \
		IOMUX_PAD(0x0408, 0x0118, 1, 0x06A8, 1, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDOE__TCON_XDIOR                                       \
		IOMUX_PAD(0x0408, 0x0118, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDOE__CSI_D_10                                         \
		IOMUX_PAD(0x0408, 0x0118, 3, 0x0658, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDOE__TCON_XDIOL                                       \
		IOMUX_PAD(0x0408, 0x0118, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDOE__GPIO_1_25                                        \
		IOMUX_PAD(0x0408, 0x0118, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDOE__ANATOP_USBPHY2_TSTO_PLL_CLK20DIV                 \
		IOMUX_PAD(0x0408, 0x0118, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDOE__MMDC_MMDC_DEBUG_21                               \
		IOMUX_PAD(0x0408, 0x0118, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_SDSHR__EPDC_SDSHR                                      \
		IOMUX_PAD(0x040C, 0x011C, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDSHR__ECSPI2_SCLK                                     \
		IOMUX_PAD(0x040C, 0x011C, 1, 0x069C, 1, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDSHR__EPDC_SDCE_4                                     \
		IOMUX_PAD(0x040C, 0x011C, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDSHR__CSI_D_11                                        \
		IOMUX_PAD(0x040C, 0x011C, 3, 0x065C, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDSHR__TCON_XDIOR                                      \
		IOMUX_PAD(0x040C, 0x011C, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDSHR__GPIO_1_26                                       \
		IOMUX_PAD(0x040C, 0x011C, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDSHR__ANATOP_USBPHY1_TSTO_RX_DISCON_DET               \
		IOMUX_PAD(0x040C, 0x011C, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_SDSHR__MMDC_MMDC_DEBUG_20                              \
		IOMUX_PAD(0x040C, 0x011C, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_VCOM0__EPDC_VCOM_0                                     \
		IOMUX_PAD(0x0410, 0x0120, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_VCOM0__AUDMUX_AUD5_RXFS                                \
		IOMUX_PAD(0x0410, 0x0120, 1, 0x0608, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_VCOM0__UART3_TXD                                       \
		IOMUX_PAD(0x0410, 0x0120, 2, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_EPDC_VCOM0__UART3_RXD                                       \
		IOMUX_PAD(0x0410, 0x0120, 2, 0x080C, 4, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_EPDC_VCOM0__WEIM_WEIM_A_24                                  \
		IOMUX_PAD(0x0410, 0x0120, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_VCOM0__TCON_VCOM_0                                     \
		IOMUX_PAD(0x0410, 0x0120, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_VCOM0__GPIO_2_3                                        \
		IOMUX_PAD(0x0410, 0x0120, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_VCOM0__EPDC_SDCE_5                                     \
		IOMUX_PAD(0x0410, 0x0120, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_VCOM0__MMDC_MMDC_DEBUG_11                              \
		IOMUX_PAD(0x0410, 0x0120, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_EPDC_VCOM1__EPDC_VCOM_1                                     \
		IOMUX_PAD(0x0414, 0x0124, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_VCOM1__AUDMUX_AUD5_RXD                                 \
		IOMUX_PAD(0x0414, 0x0124, 1, 0x05FC, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_VCOM1__UART3_TXD                                       \
		IOMUX_PAD(0x0414, 0x0124, 2, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_EPDC_VCOM1__UART3_RXD                                       \
		IOMUX_PAD(0x0414, 0x0124, 2, 0x080C, 5, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_EPDC_VCOM1__WEIM_WEIM_A_25                                  \
		IOMUX_PAD(0x0414, 0x0124, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_VCOM1__TCON_VCOM_1                                     \
		IOMUX_PAD(0x0414, 0x0124, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_VCOM1__GPIO_2_4                                        \
		IOMUX_PAD(0x0414, 0x0124, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_VCOM1__EPDC_SDCE_6                                     \
		IOMUX_PAD(0x0414, 0x0124, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_EPDC_VCOM1__MMDC_MMDC_DEBUG_10                              \
		IOMUX_PAD(0x0414, 0x0124, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_FEC_CRS_DV__FEC_RX_DV                                       \
		IOMUX_PAD(0x0418, 0x0128, 0, 0x0704, 1, MX6SL_ENET_PAD_CTRL)
#define MX6SL_PAD_FEC_CRS_DV__USDHC4_DAT1                                     \
		IOMUX_PAD(0x0418, 0x0128, 1, 0x0860, 1, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_FEC_CRS_DV__AUDMUX_AUD6_TXC                                 \
		IOMUX_PAD(0x0418, 0x0128, 2, 0x0624, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_CRS_DV__ECSPI4_MISO                                     \
		IOMUX_PAD(0x0418, 0x0128, 3, 0x06D4, 1, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_CRS_DV__GPT_CMPOUT2                                     \
		IOMUX_PAD(0x0418, 0x0128, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_CRS_DV__GPIO_4_25                                       \
		IOMUX_PAD(0x0418, 0x0128, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_CRS_DV__KITTEN_TRACE_31                                 \
		IOMUX_PAD(0x0418, 0x0128, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_CRS_DV__PL301_SIM_MX6SL_PER1_HADDR_3                    \
		IOMUX_PAD(0x0418, 0x0128, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_FEC_MDC__FEC_MDC                                            \
		IOMUX_PAD(0x041C, 0x012C, 0, 0x0000, 0, MX6SL_ENET_PAD_CTRL)
#define MX6SL_PAD_FEC_MDC__USDHC4_DAT4                                        \
		IOMUX_PAD(0x041C, 0x012C, 1, 0x086C, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_FEC_MDC__AUDMUX_AUDIO_CLK_OUT                               \
		IOMUX_PAD(0x041C, 0x012C, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_MDC__USDHC1_RST                                         \
		IOMUX_PAD(0x041C, 0x012C, 3, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_FEC_MDC__USDHC3_RST                                         \
		IOMUX_PAD(0x041C, 0x012C, 4, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_FEC_MDC__GPIO_4_23                                          \
		IOMUX_PAD(0x041C, 0x012C, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_MDC__KITTEN_TRACE_29                                    \
		IOMUX_PAD(0x041C, 0x012C, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_MDC__PL301_SIM_MX6SL_PER1_HADDR_8                       \
		IOMUX_PAD(0x041C, 0x012C, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_FEC_MDIO__FEC_MDIO                                          \
		IOMUX_PAD(0x0420, 0x0130, 0, 0x06F4, 1, MX6SL_ENET_PAD_CTRL)
#define MX6SL_PAD_FEC_MDIO__USDHC4_CLK                                        \
		IOMUX_PAD(0x0420, 0x0130, 1, 0x0850, 1, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_FEC_MDIO__AUDMUX_AUD6_RXFS                                  \
		IOMUX_PAD(0x0420, 0x0130, 2, 0x0620, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_MDIO__ECSPI4_SS0                                        \
		IOMUX_PAD(0x0420, 0x0130, 3, 0x06DC, 1, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_MDIO__GPT_CAPIN1                                        \
		IOMUX_PAD(0x0420, 0x0130, 4, 0x0710, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_MDIO__GPIO_4_20                                         \
		IOMUX_PAD(0x0420, 0x0130, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_MDIO__KITTEN_TRACE_26                                   \
		IOMUX_PAD(0x0420, 0x0130, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_MDIO__PL301_SIM_MX6SL_PER1_HADDR_15                     \
		IOMUX_PAD(0x0420, 0x0130, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_FEC_REF_CLK__FEC_REF_OUT                                    \
		IOMUX_PAD(0x0424, 0x0134, 0x10, 0x0000, 0, MX6SL_ENET_PAD_CTRL)
#define MX6SL_PAD_FEC_REF_CLK__USDHC4_RST                                     \
		IOMUX_PAD(0x0424, 0x0134, 1, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_FEC_REF_CLK__WDOG1_WDOG_B                                   \
		IOMUX_PAD(0x0424, 0x0134, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_REF_CLK__PWM4_PWMO                                      \
		IOMUX_PAD(0x0424, 0x0134, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_REF_CLK__CCM_PMIC_RDY                                   \
		IOMUX_PAD(0x0424, 0x0134, 4, 0x062C, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_REF_CLK__GPIO_4_26                                      \
		IOMUX_PAD(0x0424, 0x0134, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_REF_CLK__SPDIF_SPDIF_EXT_CLK                            \
		IOMUX_PAD(0x0424, 0x0134, 6, 0x07F4, 2, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_REF_CLK__PL301_SIM_MX6SL_PER1_HADDR_0                   \
		IOMUX_PAD(0x0424, 0x0134, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_FEC_RX_ER__GPIO_4_19                                        \
		IOMUX_PAD(0x0428, 0x0138, 5, 0x0000, 0, MX6SL_HP_DET_PAD_CTRL)
#define MX6SL_PAD_FEC_RX_ER__KITTEN_TRACE_25                                  \
		IOMUX_PAD(0x0428, 0x0138, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_RX_ER__PL301_SIM_MX6SL_PER1_HADDR_5                     \
		IOMUX_PAD(0x0428, 0x0138, 7, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_RX_ER__FEC_RX_ER                                        \
		IOMUX_PAD(0x0428, 0x0138, 0, 0x0708, 1, MX6SL_ENET_PAD_CTRL)
#define MX6SL_PAD_FEC_RX_ER__USDHC4_DAT0                                      \
		IOMUX_PAD(0x0428, 0x0138, 1, 0x085C, 1, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_FEC_RX_ER__AUDMUX_AUD6_RXD                                  \
		IOMUX_PAD(0x0428, 0x0138, 2, 0x0614, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_RX_ER__ECSPI4_MOSI                                      \
		IOMUX_PAD(0x0428, 0x0138, 3, 0x06D8, 1, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_RX_ER__GPT_CMPOUT1                                      \
		IOMUX_PAD(0x0428, 0x0138, 4, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_FEC_RXD0__FEC_RDATA_0                                       \
		IOMUX_PAD(0x042C, 0x013C, 0, 0x06F8, 0, MX6SL_ENET_PAD_CTRL)
#define MX6SL_PAD_FEC_RXD0__USDHC4_DAT5                                       \
		IOMUX_PAD(0x042C, 0x013C, 1, 0x0870, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_FEC_RXD0__ANATOP_USBOTG1_ID                                 \
		IOMUX_PAD(0x042C, 0x013C, 2, 0x05DC, 1, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_RXD0__USDHC1_VSELECT                                    \
		IOMUX_PAD(0x042C, 0x013C, 3, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_FEC_RXD0__USDHC3_VSELECT                                    \
		IOMUX_PAD(0x042C, 0x013C, 4, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_FEC_RXD0__GPIO_4_17                                         \
		IOMUX_PAD(0x042C, 0x013C, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_RXD0__KITTEN_TRACE_24                                   \
		IOMUX_PAD(0x042C, 0x013C, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_RXD0__PL301_SIM_MX6SL_PER1_HADDR_7                      \
		IOMUX_PAD(0x042C, 0x013C, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_FEC_RXD1__FEC_RDATA_1                                       \
		IOMUX_PAD(0x0430, 0x0140, 0, 0x06FC, 1, MX6SL_ENET_PAD_CTRL)
#define MX6SL_PAD_FEC_RXD1__USDHC4_DAT2                                       \
		IOMUX_PAD(0x0430, 0x0140, 1, 0x0864, 1, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_FEC_RXD1__AUDMUX_AUD6_TXFS                                  \
		IOMUX_PAD(0x0430, 0x0140, 2, 0x0628, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_RXD1__ECSPI4_SS1                                        \
		IOMUX_PAD(0x0430, 0x0140, 3, 0x06E0, 1, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_RXD1__GPT_CMPOUT3                                       \
		IOMUX_PAD(0x0430, 0x0140, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_RXD1__GPIO_4_18                                         \
		IOMUX_PAD(0x0430, 0x0140, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_RXD1__FEC_COL                                           \
		IOMUX_PAD(0x0430, 0x0140, 6, 0x06F0, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_RXD1__PL301_SIM_MX6SL_PER1_HADDR_9                      \
		IOMUX_PAD(0x0430, 0x0140, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_FEC_TX_CLK__FEC_TX_CLK                                      \
		IOMUX_PAD(0x0434, 0x0144, 0, 0x070C, 1, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_TX_CLK__USDHC4_CMD                                      \
		IOMUX_PAD(0x0434, 0x0144, 1, 0x0858, 1, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_FEC_TX_CLK__AUDMUX_AUD6_RXC                                 \
		IOMUX_PAD(0x0434, 0x0144, 2, 0x061C, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_TX_CLK__ECSPI4_SCLK                                     \
		IOMUX_PAD(0x0434, 0x0144, 3, 0x06D0, 1, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_TX_CLK__GPT_CAPIN2                                      \
		IOMUX_PAD(0x0434, 0x0144, 4, 0x0714, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_TX_CLK__GPIO_4_21                                       \
		IOMUX_PAD(0x0434, 0x0144, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_TX_CLK__KITTEN_TRACE_27                                 \
		IOMUX_PAD(0x0434, 0x0144, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_TX_CLK__PL301_SIM_MX6SL_PER1_HADDR_4                    \
		IOMUX_PAD(0x0434, 0x0144, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_FEC_TX_EN__FEC_TX_EN                                        \
		IOMUX_PAD(0x0438, 0x0148, 0, 0x0000, 0, MX6SL_ENET_PAD_CTRL)
#define MX6SL_PAD_FEC_TX_EN__USDHC4_DAT6                                      \
		IOMUX_PAD(0x0438, 0x0148, 1, 0x0874, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_FEC_TX_EN__SPDIF_IN1                                        \
		IOMUX_PAD(0x0438, 0x0148, 2, 0x07F0, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_TX_EN__USDHC1_WP                                        \
		IOMUX_PAD(0x0438, 0x0148, 3, 0x082C, 1, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_FEC_TX_EN__USDHC3_WP                                        \
		IOMUX_PAD(0x0438, 0x0148, 4, 0x084C, 1, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_FEC_TX_EN__GPIO_4_22                                        \
		IOMUX_PAD(0x0438, 0x0148, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_TX_EN__KITTEN_TRACE_28                                  \
		IOMUX_PAD(0x0438, 0x0148, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_TX_EN__PL301_SIM_MX6SL_PER1_HADDR_1                     \
		IOMUX_PAD(0x0438, 0x0148, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_FEC_TXD0__FEC_TDATA_0                                       \
		IOMUX_PAD(0x043C, 0x014C, 0, 0x0000, 0, MX6SL_ENET_PAD_CTRL)
#define MX6SL_PAD_FEC_TXD0__USDHC4_DAT3                                       \
		IOMUX_PAD(0x043C, 0x014C, 1, 0x0868, 1, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_FEC_TXD0__AUDMUX_AUD6_TXD                                   \
		IOMUX_PAD(0x043C, 0x014C, 2, 0x0618, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_TXD0__ECSPI4_SS2                                        \
		IOMUX_PAD(0x043C, 0x014C, 3, 0x06E4, 1, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_TXD0__GPT_CLKIN                                         \
		IOMUX_PAD(0x043C, 0x014C, 4, 0x0718, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_TXD0__GPIO_4_24                                         \
		IOMUX_PAD(0x043C, 0x014C, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_TXD0__KITTEN_TRACE_30                                   \
		IOMUX_PAD(0x043C, 0x014C, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_TXD0__PL301_SIM_MX6SL_PER1_HADDR_2                      \
		IOMUX_PAD(0x043C, 0x014C, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_FEC_TXD1__FEC_TDATA_1                                       \
		IOMUX_PAD(0x0440, 0x0150, 0, 0x0000, 0, MX6SL_ENET_PAD_CTRL)
#define MX6SL_PAD_FEC_TXD1__USDHC4_DAT7                                       \
		IOMUX_PAD(0x0440, 0x0150, 1, 0x0878, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_FEC_TXD1__SPDIF_OUT1                                        \
		IOMUX_PAD(0x0440, 0x0150, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_TXD1__USDHC1_CD                                         \
		IOMUX_PAD(0x0440, 0x0150, 3, 0x0828, 1, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_FEC_TXD1__USDHC3_CD                                         \
		IOMUX_PAD(0x0440, 0x0150, 4, 0x0838, 1, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_FEC_TXD1__GPIO_4_16                                         \
		IOMUX_PAD(0x0440, 0x0150, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_TXD1__FEC_RX_CLK                                        \
		IOMUX_PAD(0x0440, 0x0150, 6, 0x0700, 0, NO_PAD_CTRL)
#define MX6SL_PAD_FEC_TXD1__PL301_SIM_MX6SL_PER1_HADDR_6                      \
		IOMUX_PAD(0x0440, 0x0150, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_HSIC_DAT__USB_H_DATA                                        \
		IOMUX_PAD(0x0444, 0x0154, 0, 0x0000, 0, MX6SL_USB_HSIC_PAD_CTRL)
#define MX6SL_PAD_HSIC_DAT__I2C1_SCL                                          \
		IOMUX_PAD(0x0444, 0x0154, 1 | IOMUX_CONFIG_SION, 0x071C, 1, MX6SL_I2C_PAD_CTRL)
#define MX6SL_PAD_HSIC_DAT__PWM1_PWMO                                         \
		IOMUX_PAD(0x0444, 0x0154, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_HSIC_DAT__ANATOP_ANATOP_24M_OUT                             \
		IOMUX_PAD(0x0444, 0x0154, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_HSIC_DAT__OSC32K_32K_OUT                                    \
		IOMUX_PAD(0x0444, 0x0154, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_HSIC_DAT__GPIO_3_19                                         \
		IOMUX_PAD(0x0444, 0x0154, 5, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_HSIC_STROBE__USB_H_STROBE                                   \
		IOMUX_PAD(0x0448, 0x0158, 0, 0x0000, 0, MX6SL_USB_HSIC_PAD_CTRL)
#define MX6SL_PAD_HSIC_STROBE__USB_H_STROBE_START                                   \
		IOMUX_PAD(0x0448, 0x0158, 0, 0x0000, 0, MX6SL_USB_HSIC_PAD_CTRL | PAD_CTL_PUS_47K_UP)
#define MX6SL_PAD_HSIC_STROBE__I2C1_SDA                                       \
		IOMUX_PAD(0x0448, 0x0158, 1 | IOMUX_CONFIG_SION, 0x0720, 1, MX6SL_I2C_PAD_CTRL)
#define MX6SL_PAD_HSIC_STROBE__PWM2_PWMO                                      \
		IOMUX_PAD(0x0448, 0x0158, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_HSIC_STROBE__ANATOP_ANATOP_32K_OUT                          \
		IOMUX_PAD(0x0448, 0x0158, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_HSIC_STROBE__GPIO_3_20                                      \
		IOMUX_PAD(0x0448, 0x0158, 5, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_I2C1_SCL__I2C1_SCL                                          \
		IOMUX_PAD(0x044C, 0x015C, 0 | IOMUX_CONFIG_SION, 0x071C, 2, MX6SL_I2C_PAD_CTRL)
#define MX6SL_PAD_I2C1_SCL__UART1_CTS                                         \
		IOMUX_PAD(0x044C, 0x015C, 1, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_I2C1_SCL__UART1_RTS                                         \
		IOMUX_PAD(0x044C, 0x015C, 1, 0x07F8, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_I2C1_SCL__ECSPI3_SS2                                        \
		IOMUX_PAD(0x044C, 0x015C, 2, 0x06C8, 1, NO_PAD_CTRL)
#define MX6SL_PAD_I2C1_SCL__FEC_RDATA_0                                       \
		IOMUX_PAD(0x044C, 0x015C, 3, 0x06F8, 1, NO_PAD_CTRL)
#define MX6SL_PAD_I2C1_SCL__USDHC3_RST                                        \
		IOMUX_PAD(0x044C, 0x015C, 4, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_I2C1_SCL__GPIO_3_12                                         \
		IOMUX_PAD(0x044C, 0x015C, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_I2C1_SCL__ECSPI1_SS1                                        \
		IOMUX_PAD(0x044C, 0x015C, 6, 0x0690, 0, NO_PAD_CTRL)
#define MX6SL_PAD_I2C1_SCL__PL301_SIM_MX6SL_PER1_HSIZE_0                      \
		IOMUX_PAD(0x044C, 0x015C, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_I2C1_SDA__I2C1_SDA                                          \
		IOMUX_PAD(0x0450, 0x0160, 0 | IOMUX_CONFIG_SION, 0x0720, 2, MX6SL_I2C_PAD_CTRL)
#define MX6SL_PAD_I2C1_SDA__UART1_CTS                                         \
		IOMUX_PAD(0x0450, 0x0160, 1, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_I2C1_SDA__UART1_RTS                                         \
		IOMUX_PAD(0x0450, 0x0160, 1, 0x07F8, 1, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_I2C1_SDA__ECSPI3_SS3                                        \
		IOMUX_PAD(0x0450, 0x0160, 2, 0x06CC, 1, NO_PAD_CTRL)
#define MX6SL_PAD_I2C1_SDA__FEC_TX_EN                                         \
		IOMUX_PAD(0x0450, 0x0160, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_I2C1_SDA__USDHC3_VSELECT                                    \
		IOMUX_PAD(0x0450, 0x0160, 4, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_I2C1_SDA__GPIO_3_13                                         \
		IOMUX_PAD(0x0450, 0x0160, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_I2C1_SDA__ECSPI1_SS2                                        \
		IOMUX_PAD(0x0450, 0x0160, 6, 0x0694, 0, NO_PAD_CTRL)
#define MX6SL_PAD_I2C1_SDA__PL301_SIM_MX6SL_PER1_HSIZE_1                      \
		IOMUX_PAD(0x0450, 0x0160, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_I2C2_SCL__I2C2_SCL                                          \
		IOMUX_PAD(0x0454, 0x0164, 0 | IOMUX_CONFIG_SION, 0x0724, 1, MX6SL_I2C_PAD_CTRL)
#define MX6SL_PAD_I2C2_SCL__AUDMUX_AUD4_RXFS                                  \
		IOMUX_PAD(0x0454, 0x0164, 1, 0x05F0, 0, NO_PAD_CTRL)
#define MX6SL_PAD_I2C2_SCL__SPDIF_IN1                                         \
		IOMUX_PAD(0x0454, 0x0164, 2, 0x07F0, 1, NO_PAD_CTRL)
#define MX6SL_PAD_I2C2_SCL__FEC_TDATA_1                                       \
		IOMUX_PAD(0x0454, 0x0164, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_I2C2_SCL__USDHC3_WP                                         \
		IOMUX_PAD(0x0454, 0x0164, 4, 0x084C, 2, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_I2C2_SCL__GPIO_3_14                                         \
		IOMUX_PAD(0x0454, 0x0164, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_I2C2_SCL__ECSPI1_RDY                                        \
		IOMUX_PAD(0x0454, 0x0164, 6, 0x0680, 0, NO_PAD_CTRL)
#define MX6SL_PAD_I2C2_SCL__PL301_SIM_MX6SL_PER1_HSIZE_2                      \
		IOMUX_PAD(0x0454, 0x0164, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_I2C2_SDA__I2C2_SDA                                          \
		IOMUX_PAD(0x0458, 0x0168, 0 | IOMUX_CONFIG_SION, 0x0728, 1, MX6SL_I2C_PAD_CTRL)
#define MX6SL_PAD_I2C2_SDA__AUDMUX_AUD4_RXC                                   \
		IOMUX_PAD(0x0458, 0x0168, 1, 0x05EC, 0, NO_PAD_CTRL)
#define MX6SL_PAD_I2C2_SDA__SPDIF_OUT1                                        \
		IOMUX_PAD(0x0458, 0x0168, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_I2C2_SDA__FEC_REF_OUT                                       \
		IOMUX_PAD(0x0458, 0x0168, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_I2C2_SDA__USDHC3_CD                                         \
		IOMUX_PAD(0x0458, 0x0168, 4, 0x0838, 2, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_I2C2_SDA__GPIO_3_15                                         \
		IOMUX_PAD(0x0458, 0x0168, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_I2C2_SDA__ANATOP_ANATOP_TESTO_0                             \
		IOMUX_PAD(0x0458, 0x0168, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_I2C2_SDA__PL301_SIM_MX6SL_PER1_HWRITE                       \
		IOMUX_PAD(0x0458, 0x0168, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_JTAG_MOD__SJC_MOD                                           \
		IOMUX_PAD(0x045C, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_JTAG_TCK__SJC_TCK                                           \
		IOMUX_PAD(0x0460, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_JTAG_TDI__SJC_TDI                                           \
		IOMUX_PAD(0x0464, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_JTAG_TDO__SJC_TDO                                           \
		IOMUX_PAD(0x0468, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_JTAG_TMS__SJC_TMS                                           \
		IOMUX_PAD(0x046C, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_JTAG_TRSTB__SJC_TRSTB                                       \
		IOMUX_PAD(0x0470, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_KEY_COL0__KPP_COL_0                                         \
		IOMUX_PAD(0x0474, 0x016C, 0, 0x0734, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL0__I2C2_SCL                                          \
		IOMUX_PAD(0x0474, 0x016C, 1 | IOMUX_CONFIG_SION, 0x0724, 2, MX6SL_I2C_PAD_CTRL)
#define MX6SL_PAD_KEY_COL0__LCDIF_DAT_0                                       \
		IOMUX_PAD(0x0474, 0x016C, 2, 0x0778, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL0__WEIM_WEIM_DA_A_0                                  \
		IOMUX_PAD(0x0474, 0x016C, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL0__USDHC1_CD                                         \
		IOMUX_PAD(0x0474, 0x016C, 4, 0x0828, 2, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_KEY_COL0__GPIO_3_24                                         \
		IOMUX_PAD(0x0474, 0x016C, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL0__MSHC_SCLK                                         \
		IOMUX_PAD(0x0474, 0x016C, 6, 0x07E8, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL0__TPSMP_HDATA_0                                     \
		IOMUX_PAD(0x0474, 0x016C, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_KEY_COL1__KPP_COL_1                                         \
		IOMUX_PAD(0x0478, 0x0170, 0, 0x0738, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL1__ECSPI4_MOSI                                       \
		IOMUX_PAD(0x0478, 0x0170, 1, 0x06D8, 2, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL1__LCDIF_DAT_2                                       \
		IOMUX_PAD(0x0478, 0x0170, 2, 0x0780, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL1__WEIM_WEIM_DA_A_2                                  \
		IOMUX_PAD(0x0478, 0x0170, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL1__USDHC3_DAT4                                       \
		IOMUX_PAD(0x0478, 0x0170, 4, 0x083C, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_KEY_COL1__GPIO_3_26                                         \
		IOMUX_PAD(0x0478, 0x0170, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL1__MSHC_DATA_0                                       \
		IOMUX_PAD(0x0478, 0x0170, 6, 0x07D8, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL1__TPSMP_HDATA_2                                     \
		IOMUX_PAD(0x0478, 0x0170, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_KEY_COL2__KPP_COL_2                                         \
		IOMUX_PAD(0x047C, 0x0174, 0, 0x073C, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL2__ECSPI4_SS0                                        \
		IOMUX_PAD(0x047C, 0x0174, 1, 0x06DC, 2, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL2__LCDIF_DAT_4                                       \
		IOMUX_PAD(0x047C, 0x0174, 2, 0x0788, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL2__WEIM_WEIM_DA_A_4                                  \
		IOMUX_PAD(0x047C, 0x0174, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL2__USDHC3_DAT6                                       \
		IOMUX_PAD(0x047C, 0x0174, 4, 0x0844, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_KEY_COL2__GPIO_3_28                                         \
		IOMUX_PAD(0x047C, 0x0174, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL2__MSHC_DATA_2                                       \
		IOMUX_PAD(0x047C, 0x0174, 6, 0x07E0, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL2__TPSMP_HDATA_4                                     \
		IOMUX_PAD(0x047C, 0x0174, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_KEY_COL3__KPP_COL_3                                         \
		IOMUX_PAD(0x0480, 0x0178, 0, 0x0740, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL3__AUDMUX_AUD6_RXFS                                  \
		IOMUX_PAD(0x0480, 0x0178, 1, 0x0620, 1, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL3__LCDIF_DAT_6                                       \
		IOMUX_PAD(0x0480, 0x0178, 2, 0x0790, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL3__WEIM_WEIM_DA_A_6                                  \
		IOMUX_PAD(0x0480, 0x0178, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL3__USDHC4_DAT6                                       \
		IOMUX_PAD(0x0480, 0x0178, 4, 0x0874, 1, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_KEY_COL3__GPIO_3_30                                         \
		IOMUX_PAD(0x0480, 0x0178, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL3__USDHC1_RST                                        \
		IOMUX_PAD(0x0480, 0x0178, 6, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_KEY_COL3__TPSMP_HDATA_6                                     \
		IOMUX_PAD(0x0480, 0x0178, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_KEY_COL4__KPP_COL_4                                         \
		IOMUX_PAD(0x0484, 0x017C, 0, 0x0744, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL4__AUDMUX_AUD6_RXD                                   \
		IOMUX_PAD(0x0484, 0x017C, 1, 0x0614, 1, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL4__LCDIF_DAT_8                                       \
		IOMUX_PAD(0x0484, 0x017C, 2, 0x0798, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL4__WEIM_WEIM_DA_A_8                                  \
		IOMUX_PAD(0x0484, 0x017C, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL4__USDHC4_CLK                                        \
		IOMUX_PAD(0x0484, 0x017C, 4, 0x0850, 2, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_KEY_COL4__GPIO_4_0                                          \
		IOMUX_PAD(0x0484, 0x017C, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL4__USB_USBOTG1_PWR                                   \
		IOMUX_PAD(0x0484, 0x017C, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL4__TPSMP_HDATA_8                                     \
		IOMUX_PAD(0x0484, 0x017C, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_KEY_COL5__KPP_COL_5                                         \
		IOMUX_PAD(0x0488, 0x0180, 0, 0x0748, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL5__AUDMUX_AUD6_TXFS                                  \
		IOMUX_PAD(0x0488, 0x0180, 1, 0x0628, 1, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL5__LCDIF_DAT_10                                      \
		IOMUX_PAD(0x0488, 0x0180, 2, 0x07A0, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL5__WEIM_WEIM_DA_A_10                                 \
		IOMUX_PAD(0x0488, 0x0180, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL5__USDHC4_DAT0                                       \
		IOMUX_PAD(0x0488, 0x0180, 4, 0x085C, 2, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_KEY_COL5__GPIO_4_2                                          \
		IOMUX_PAD(0x0488, 0x0180, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL5__USB_USBOTG2_PWR                                   \
		IOMUX_PAD(0x0488, 0x0180, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL5__TPSMP_HDATA_10                                    \
		IOMUX_PAD(0x0488, 0x0180, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_KEY_COL6__KPP_COL_6                                         \
		IOMUX_PAD(0x048C, 0x0184, 0, 0x074C, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL6__UART4_TXD                                         \
		IOMUX_PAD(0x048C, 0x0184, 1, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_KEY_COL6__UART4_RXD                                         \
		IOMUX_PAD(0x048C, 0x0184, 1, 0x0814, 2, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_KEY_COL6__LCDIF_DAT_12                                      \
		IOMUX_PAD(0x048C, 0x0184, 2, 0x07A8, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL6__WEIM_WEIM_DA_A_12                                 \
		IOMUX_PAD(0x048C, 0x0184, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL6__USDHC4_DAT2                                       \
		IOMUX_PAD(0x048C, 0x0184, 4, 0x0864, 2, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_KEY_COL6__GPIO_4_4                                          \
		IOMUX_PAD(0x048C, 0x0184, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL6__USDHC3_RST                                        \
		IOMUX_PAD(0x048C, 0x0184, 6, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_KEY_COL6__TPSMP_HDATA_12                                    \
		IOMUX_PAD(0x048C, 0x0184, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_KEY_COL7__KPP_COL_7                                         \
		IOMUX_PAD(0x0490, 0x0188, 0, 0x0750, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL7__UART4_CTS                                         \
		IOMUX_PAD(0x0490, 0x0188, 1, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_KEY_COL7__UART4_RTS                                         \
		IOMUX_PAD(0x0490, 0x0188, 1, 0x0810, 2, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_KEY_COL7__LCDIF_DAT_14                                      \
		IOMUX_PAD(0x0490, 0x0188, 2, 0x07B0, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL7__WEIM_WEIM_DA_A_14                                 \
		IOMUX_PAD(0x0490, 0x0188, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_COL7__USDHC4_DAT4                                       \
		IOMUX_PAD(0x0490, 0x0188, 4, 0x086C, 1, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_KEY_COL7__GPIO_4_6                                          \
		IOMUX_PAD(0x0490, 0x0188, 5, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_KEY_COL7__USDHC1_WP                                         \
		IOMUX_PAD(0x0490, 0x0188, 6, 0x082C, 2, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_KEY_COL7__TPSMP_HDATA_14                                    \
		IOMUX_PAD(0x0490, 0x0188, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_KEY_ROW0__KPP_ROW_0                                         \
		IOMUX_PAD(0x0494, 0x018C, 0, 0x0754, 0, MX6SL_KEYPAD_CTRL)
#define MX6SL_PAD_KEY_ROW0__I2C2_SDA                                          \
		IOMUX_PAD(0x0494, 0x018C, 1 | IOMUX_CONFIG_SION, 0x0728, 2, MX6SL_I2C_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW0__LCDIF_DAT_1                                       \
		IOMUX_PAD(0x0494, 0x018C, 2, 0x077C, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW0__WEIM_WEIM_DA_A_1                                  \
		IOMUX_PAD(0x0494, 0x018C, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW0__USDHC1_WP                                         \
		IOMUX_PAD(0x0494, 0x018C, 4, 0x082C, 3, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW0__GPIO_3_25                                         \
		IOMUX_PAD(0x0494, 0x018C, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW0__MSHC_BS                                           \
		IOMUX_PAD(0x0494, 0x018C, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW0__TPSMP_HDATA_1                                     \
		IOMUX_PAD(0x0494, 0x018C, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_KEY_ROW1__KPP_ROW_1                                         \
		IOMUX_PAD(0x0498, 0x0190, 0, 0x0758, 0, MX6SL_KEYPAD_CTRL)
#define MX6SL_PAD_KEY_ROW1__ECSPI4_MISO                                       \
		IOMUX_PAD(0x0498, 0x0190, 1, 0x06D4, 2, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW1__LCDIF_DAT_3                                       \
		IOMUX_PAD(0x0498, 0x0190, 2, 0x0784, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW1__WEIM_WEIM_DA_A_3                                  \
		IOMUX_PAD(0x0498, 0x0190, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW1__USDHC3_DAT5                                       \
		IOMUX_PAD(0x0498, 0x0190, 4, 0x0840, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW1__GPIO_3_27                                         \
		IOMUX_PAD(0x0498, 0x0190, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW1__MSHC_DATA_1                                       \
		IOMUX_PAD(0x0498, 0x0190, 6, 0x07DC, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW1__TPSMP_HDATA_3                                     \
		IOMUX_PAD(0x0498, 0x0190, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_KEY_ROW2__KPP_ROW_2                                         \
		IOMUX_PAD(0x049C, 0x0194, 0, 0x075C, 0, MX6SL_KEYPAD_CTRL)
#define MX6SL_PAD_KEY_ROW2__ECSPI4_SCLK                                       \
		IOMUX_PAD(0x049C, 0x0194, 1, 0x06D0, 2, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW2__LCDIF_DAT_5                                       \
		IOMUX_PAD(0x049C, 0x0194, 2, 0x078C, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW2__WEIM_WEIM_DA_A_5                                  \
		IOMUX_PAD(0x049C, 0x0194, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW2__USDHC3_DAT7                                       \
		IOMUX_PAD(0x049C, 0x0194, 4, 0x0848, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW2__GPIO_3_29                                         \
		IOMUX_PAD(0x049C, 0x0194, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW2__MSHC_DATA_3                                       \
		IOMUX_PAD(0x049C, 0x0194, 6, 0x07E4, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW2__TPSMP_HDATA_5                                     \
		IOMUX_PAD(0x049C, 0x0194, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_KEY_ROW3__KPP_ROW_3                                         \
		IOMUX_PAD(0x04A0, 0x0198, 0, 0x0760, 0, MX6SL_KEYPAD_CTRL)
#define MX6SL_PAD_KEY_ROW3__AUDMUX_AUD6_RXC                                   \
		IOMUX_PAD(0x04A0, 0x0198, 1, 0x061C, 1, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW3__LCDIF_DAT_7                                       \
		IOMUX_PAD(0x04A0, 0x0198, 2, 0x0794, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW3__WEIM_WEIM_DA_A_7                                  \
		IOMUX_PAD(0x04A0, 0x0198, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW3__USDHC4_DAT7                                       \
		IOMUX_PAD(0x04A0, 0x0198, 4, 0x0878, 1, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW3__GPIO_3_31                                         \
		IOMUX_PAD(0x04A0, 0x0198, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW3__USDHC1_VSELECT                                    \
		IOMUX_PAD(0x04A0, 0x0198, 6, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW3__TPSMP_HDATA_7                                     \
		IOMUX_PAD(0x04A0, 0x0198, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_KEY_ROW4__KPP_ROW_4                                         \
		IOMUX_PAD(0x04A4, 0x019C, 0, 0x0764, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW4__AUDMUX_AUD6_TXC                                   \
		IOMUX_PAD(0x04A4, 0x019C, 1, 0x0624, 1, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW4__LCDIF_DAT_9                                       \
		IOMUX_PAD(0x04A4, 0x019C, 2, 0x079C, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW4__WEIM_WEIM_DA_A_9                                  \
		IOMUX_PAD(0x04A4, 0x019C, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW4__USDHC4_CMD                                        \
		IOMUX_PAD(0x04A4, 0x019C, 4, 0x0858, 2, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW4__GPIO_4_1                                          \
		IOMUX_PAD(0x04A4, 0x019C, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW4__USB_USBOTG1_OC                                    \
		IOMUX_PAD(0x04A4, 0x019C, 6, 0x0824, 1, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW4__TPSMP_HDATA_9                                     \
		IOMUX_PAD(0x04A4, 0x019C, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_KEY_ROW5__KPP_ROW_5                                         \
		IOMUX_PAD(0x04A8, 0x01A0, 0, 0x0768, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW5__AUDMUX_AUD6_TXD                                   \
		IOMUX_PAD(0x04A8, 0x01A0, 1, 0x0618, 1, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW5__LCDIF_DAT_11                                      \
		IOMUX_PAD(0x04A8, 0x01A0, 2, 0x07A4, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW5__WEIM_WEIM_DA_A_11                                 \
		IOMUX_PAD(0x04A8, 0x01A0, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW5__USDHC4_DAT1                                       \
		IOMUX_PAD(0x04A8, 0x01A0, 4, 0x0860, 2, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW5__GPIO_4_3                                          \
		IOMUX_PAD(0x04A8, 0x01A0, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW5__USB_USBOTG2_OC                                    \
		IOMUX_PAD(0x04A8, 0x01A0, 6, 0x0820, 2, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW5__TPSMP_HDATA_11                                    \
		IOMUX_PAD(0x04A8, 0x01A0, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_KEY_ROW6__KPP_ROW_6                                         \
		IOMUX_PAD(0x04AC, 0x01A4, 0, 0x076C, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW6__UART4_TXD                                         \
		IOMUX_PAD(0x04AC, 0x01A4, 1, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW6__UART4_RXD                                         \
		IOMUX_PAD(0x04AC, 0x01A4, 1, 0x0814, 3, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW6__LCDIF_DAT_13                                      \
		IOMUX_PAD(0x04AC, 0x01A4, 2, 0x07AC, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW6__WEIM_WEIM_DA_A_13                                 \
		IOMUX_PAD(0x04AC, 0x01A4, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW6__USDHC4_DAT3                                       \
		IOMUX_PAD(0x04AC, 0x01A4, 4, 0x0868, 2, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW6__GPIO_4_5                                          \
		IOMUX_PAD(0x04AC, 0x01A4, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW6__USDHC3_VSELECT                                    \
		IOMUX_PAD(0x04AC, 0x01A4, 6, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW6__TPSMP_HDATA_13                                    \
		IOMUX_PAD(0x04AC, 0x01A4, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_KEY_ROW7__KPP_ROW_7                                         \
		IOMUX_PAD(0x04B0, 0x01A8, 0, 0x0770, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW7__UART4_CTS                                         \
		IOMUX_PAD(0x04B0, 0x01A8, 1, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW7__UART4_RTS                                         \
		IOMUX_PAD(0x04B0, 0x01A8, 1, 0x0810, 3, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW7__LCDIF_DAT_15                                      \
		IOMUX_PAD(0x04B0, 0x01A8, 2, 0x07B4, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW7__WEIM_WEIM_DA_A_15                                 \
		IOMUX_PAD(0x04B0, 0x01A8, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW7__USDHC4_DAT5                                       \
		IOMUX_PAD(0x04B0, 0x01A8, 4, 0x0870, 1, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW7__GPIO_4_7                                          \
		IOMUX_PAD(0x04B0, 0x01A8, 5, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW7__USDHC1_CD                                         \
		IOMUX_PAD(0x04B0, 0x01A8, 6, 0x0828, 3, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_KEY_ROW7__TPSMP_HDATA_15                                    \
		IOMUX_PAD(0x04B0, 0x01A8, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_LCD_CLK__LCDIF_CLK                                          \
		IOMUX_PAD(0x04B4, 0x01AC, 0, 0x0000, 0, MX6SL_LCDIF_PAD_CTRL)
#define MX6SL_PAD_LCD_CLK__USDHC4_DAT4                                        \
		IOMUX_PAD(0x04B4, 0x01AC, 1, 0x086C, 2, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_LCD_CLK__LCDIF_WR_RWN                                       \
		IOMUX_PAD(0x04B4, 0x01AC, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_CLK__WEIM_WEIM_RW                                       \
		IOMUX_PAD(0x04B4, 0x01AC, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_CLK__PWM4_PWMO                                          \
		IOMUX_PAD(0x04B4, 0x01AC, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_CLK__GPIO_2_15                                          \
		IOMUX_PAD(0x04B4, 0x01AC, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_CLK__SRC_EARLY_RST                                      \
		IOMUX_PAD(0x04B4, 0x01AC, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_CLK__TPSMP_HTRANS_0                                     \
		IOMUX_PAD(0x04B4, 0x01AC, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_LCD_DAT0__LCDIF_DAT_0                                       \
		IOMUX_PAD(0x04B8, 0x01B0, 0, 0x0778, 1, MX6SL_LCDIF_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT0__ECSPI1_MOSI                                       \
		IOMUX_PAD(0x04B8, 0x01B0, 1, 0x0688, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT0__ANATOP_USBOTG2_ID                                 \
		IOMUX_PAD(0x04B8, 0x01B0, 2, 0x05E0, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT0__PWM1_PWMO                                         \
		IOMUX_PAD(0x04B8, 0x01B0, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT0__UART5_DTR                                         \
		IOMUX_PAD(0x04B8, 0x01B0, 4, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT0__GPIO_2_20                                         \
		IOMUX_PAD(0x04B8, 0x01B0, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT0__KITTEN_TRACE_0                                    \
		IOMUX_PAD(0x04B8, 0x01B0, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT0__SRC_BT_CFG_0                                      \
		IOMUX_PAD(0x04B8, 0x01B0, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_LCD_DAT1__LCDIF_DAT_1                                       \
		IOMUX_PAD(0x04BC, 0x01B4, 0, 0x077C, 1, MX6SL_LCDIF_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT1__ECSPI1_MISO                                       \
		IOMUX_PAD(0x04BC, 0x01B4, 1, 0x0684, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT1__ANATOP_USBOTG1_ID                                 \
		IOMUX_PAD(0x04BC, 0x01B4, 2, 0x05DC, 2, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT1__PWM2_PWMO                                         \
		IOMUX_PAD(0x04BC, 0x01B4, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT1__AUDMUX_AUD4_RXFS                                  \
		IOMUX_PAD(0x04BC, 0x01B4, 4, 0x05F0, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT1__GPIO_2_21                                         \
		IOMUX_PAD(0x04BC, 0x01B4, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT1__KITTEN_TRACE_1                                    \
		IOMUX_PAD(0x04BC, 0x01B4, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT1__SRC_BT_CFG_1                                      \
		IOMUX_PAD(0x04BC, 0x01B4, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_LCD_DAT10__LCDIF_DAT_10                                     \
		IOMUX_PAD(0x04C0, 0x01B8, 0, 0x07A0, 1, MX6SL_LCDIF_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT10__KPP_COL_1                                        \
		IOMUX_PAD(0x04C0, 0x01B8, 1, 0x0738, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT10__CSI_D_7                                          \
		IOMUX_PAD(0x04C0, 0x01B8, 2, 0x064C, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT10__WEIM_WEIM_D_4                                    \
		IOMUX_PAD(0x04C0, 0x01B8, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT10__ECSPI2_MISO                                      \
		IOMUX_PAD(0x04C0, 0x01B8, 4, 0x06A0, 2, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT10__GPIO_2_30                                        \
		IOMUX_PAD(0x04C0, 0x01B8, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT10__KITTEN_TRACE_10                                  \
		IOMUX_PAD(0x04C0, 0x01B8, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT10__SRC_BT_CFG_10                                    \
		IOMUX_PAD(0x04C0, 0x01B8, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_LCD_DAT11__LCDIF_DAT_11                                     \
		IOMUX_PAD(0x04C4, 0x01BC, 0, 0x07A4, 1, MX6SL_LCDIF_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT11__KPP_ROW_1                                        \
		IOMUX_PAD(0x04C4, 0x01BC, 1, 0x0758, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT11__CSI_D_6                                          \
		IOMUX_PAD(0x04C4, 0x01BC, 2, 0x0648, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT11__WEIM_WEIM_D_5                                    \
		IOMUX_PAD(0x04C4, 0x01BC, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT11__ECSPI2_SS1                                       \
		IOMUX_PAD(0x04C4, 0x01BC, 4, 0x06AC, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT11__GPIO_2_31                                        \
		IOMUX_PAD(0x04C4, 0x01BC, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT11__KITTEN_TRACE_11                                  \
		IOMUX_PAD(0x04C4, 0x01BC, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT11__SRC_BT_CFG_11                                    \
		IOMUX_PAD(0x04C4, 0x01BC, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_LCD_DAT12__LCDIF_DAT_12                                     \
		IOMUX_PAD(0x04C8, 0x01C0, 0, 0x07A8, 1, MX6SL_LCDIF_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT12__KPP_COL_2                                        \
		IOMUX_PAD(0x04C8, 0x01C0, 1, 0x073C, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT12__CSI_D_5                                          \
		IOMUX_PAD(0x04C8, 0x01C0, 2, 0x0644, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT12__WEIM_WEIM_D_6                                    \
		IOMUX_PAD(0x04C8, 0x01C0, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT12__UART5_CTS                                        \
		IOMUX_PAD(0x04C8, 0x01C0, 4, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT12__UART5_RTS                                        \
		IOMUX_PAD(0x04C8, 0x01C0, 4, 0x0818, 2, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT12__GPIO_3_0                                         \
		IOMUX_PAD(0x04C8, 0x01C0, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT12__KITTEN_TRACE_12                                  \
		IOMUX_PAD(0x04C8, 0x01C0, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT12__SRC_BT_CFG_12                                    \
		IOMUX_PAD(0x04C8, 0x01C0, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_LCD_DAT13__LCDIF_DAT_13                                     \
		IOMUX_PAD(0x04CC, 0x01C4, 0, 0x07AC, 1, MX6SL_LCDIF_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT13__KPP_ROW_2                                        \
		IOMUX_PAD(0x04CC, 0x01C4, 1, 0x075C, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT13__CSI_D_4                                          \
		IOMUX_PAD(0x04CC, 0x01C4, 2, 0x0640, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT13__WEIM_WEIM_D_7                                    \
		IOMUX_PAD(0x04CC, 0x01C4, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT13__UART5_CTS                                        \
		IOMUX_PAD(0x04CC, 0x01C4, 4, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT13__UART5_RTS                                        \
		IOMUX_PAD(0x04CC, 0x01C4, 4, 0x0818, 3, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT13__GPIO_3_1                                         \
		IOMUX_PAD(0x04CC, 0x01C4, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT13__KITTEN_TRACE_13                                  \
		IOMUX_PAD(0x04CC, 0x01C4, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT13__SRC_BT_CFG_13                                    \
		IOMUX_PAD(0x04CC, 0x01C4, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_LCD_DAT14__LCDIF_DAT_14                                     \
		IOMUX_PAD(0x04D0, 0x01C8, 0, 0x07B0, 1, MX6SL_LCDIF_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT14__KPP_COL_3                                        \
		IOMUX_PAD(0x04D0, 0x01C8, 1, 0x0740, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT14__CSI_D_3                                          \
		IOMUX_PAD(0x04D0, 0x01C8, 2, 0x063C, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT14__WEIM_WEIM_D_8                                    \
		IOMUX_PAD(0x04D0, 0x01C8, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT14__UART5_TXD                                        \
		IOMUX_PAD(0x04D0, 0x01C8, 4, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT14__UART5_RXD                                        \
		IOMUX_PAD(0x04D0, 0x01C8, 4, 0x081C, 2, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT14__GPIO_3_2                                         \
		IOMUX_PAD(0x04D0, 0x01C8, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT14__KITTEN_TRACE_14                                  \
		IOMUX_PAD(0x04D0, 0x01C8, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT14__SRC_BT_CFG_14                                    \
		IOMUX_PAD(0x04D0, 0x01C8, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_LCD_DAT15__LCDIF_DAT_15                                     \
		IOMUX_PAD(0x04D4, 0x01CC, 0, 0x07B4, 1, MX6SL_LCDIF_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT15__KPP_ROW_3                                        \
		IOMUX_PAD(0x04D4, 0x01CC, 1, 0x0760, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT15__CSI_D_2                                          \
		IOMUX_PAD(0x04D4, 0x01CC, 2, 0x0638, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT15__WEIM_WEIM_D_9                                    \
		IOMUX_PAD(0x04D4, 0x01CC, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT15__UART5_TXD                                        \
		IOMUX_PAD(0x04D4, 0x01CC, 4, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT15__UART5_RXD                                        \
		IOMUX_PAD(0x04D4, 0x01CC, 4, 0x081C, 3, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT15__GPIO_3_3                                         \
		IOMUX_PAD(0x04D4, 0x01CC, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT15__KITTEN_TRACE_15                                  \
		IOMUX_PAD(0x04D4, 0x01CC, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT15__SRC_BT_CFG_15                                    \
		IOMUX_PAD(0x04D4, 0x01CC, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_LCD_DAT16__LCDIF_DAT_16                                     \
		IOMUX_PAD(0x04D8, 0x01D0, 0, 0x07B8, 1, MX6SL_LCDIF_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT16__KPP_COL_4                                        \
		IOMUX_PAD(0x04D8, 0x01D0, 1, 0x0744, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT16__CSI_D_1                                          \
		IOMUX_PAD(0x04D8, 0x01D0, 2, 0x0634, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT16__WEIM_WEIM_D_10                                   \
		IOMUX_PAD(0x04D8, 0x01D0, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT16__I2C2_SCL                                         \
		IOMUX_PAD(0x04D8, 0x01D0, 4 | IOMUX_CONFIG_SION, 0x0724, 3, MX6SL_I2C_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT16__GPIO_3_4                                         \
		IOMUX_PAD(0x04D8, 0x01D0, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT16__KITTEN_TRACE_16                                  \
		IOMUX_PAD(0x04D8, 0x01D0, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT16__SRC_BT_CFG_24                                    \
		IOMUX_PAD(0x04D8, 0x01D0, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_LCD_DAT17__LCDIF_DAT_17                                     \
		IOMUX_PAD(0x04DC, 0x01D4, 0, 0x07BC, 1, MX6SL_LCDIF_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT17__KPP_ROW_4                                        \
		IOMUX_PAD(0x04DC, 0x01D4, 1, 0x0764, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT17__CSI_D_0                                          \
		IOMUX_PAD(0x04DC, 0x01D4, 2, 0x0630, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT17__WEIM_WEIM_D_11                                   \
		IOMUX_PAD(0x04DC, 0x01D4, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT17__I2C2_SDA                                         \
		IOMUX_PAD(0x04DC, 0x01D4, 4 | IOMUX_CONFIG_SION, 0x0728, 3, MX6SL_I2C_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT17__GPIO_3_5                                         \
		IOMUX_PAD(0x04DC, 0x01D4, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT17__KITTEN_TRACE_17                                  \
		IOMUX_PAD(0x04DC, 0x01D4, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT17__SRC_BT_CFG_25                                    \
		IOMUX_PAD(0x04DC, 0x01D4, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_LCD_DAT18__LCDIF_DAT_18                                     \
		IOMUX_PAD(0x04E0, 0x01D8, 0, 0x07C0, 1, MX6SL_LCDIF_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT18__KPP_COL_5                                        \
		IOMUX_PAD(0x04E0, 0x01D8, 1, 0x0748, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT18__CSI_D_15                                         \
		IOMUX_PAD(0x04E0, 0x01D8, 2, 0x066C, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT18__WEIM_WEIM_D_12                                   \
		IOMUX_PAD(0x04E0, 0x01D8, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT18__GPT_CAPIN1                                       \
		IOMUX_PAD(0x04E0, 0x01D8, 4, 0x0710, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT18__GPIO_3_6                                         \
		IOMUX_PAD(0x04E0, 0x01D8, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT18__KITTEN_TRACE_18                                  \
		IOMUX_PAD(0x04E0, 0x01D8, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT18__SRC_BT_CFG_26                                    \
		IOMUX_PAD(0x04E0, 0x01D8, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_LCD_DAT19__LCDIF_DAT_19                                     \
		IOMUX_PAD(0x04E4, 0x01DC, 0, 0x07C4, 1, MX6SL_LCDIF_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT19__KPP_ROW_5                                        \
		IOMUX_PAD(0x04E4, 0x01DC, 1, 0x0768, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT19__CSI_D_14                                         \
		IOMUX_PAD(0x04E4, 0x01DC, 2, 0x0668, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT19__WEIM_WEIM_D_13                                   \
		IOMUX_PAD(0x04E4, 0x01DC, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT19__GPT_CAPIN2                                       \
		IOMUX_PAD(0x04E4, 0x01DC, 4, 0x0714, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT19__GPIO_3_7                                         \
		IOMUX_PAD(0x04E4, 0x01DC, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT19__KITTEN_TRACE_19                                  \
		IOMUX_PAD(0x04E4, 0x01DC, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT19__SRC_BT_CFG_27                                    \
		IOMUX_PAD(0x04E4, 0x01DC, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_LCD_DAT2__LCDIF_DAT_2                                       \
		IOMUX_PAD(0x04E8, 0x01E0, 0, 0x0780, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT2__ECSPI1_SS0                                        \
		IOMUX_PAD(0x04E8, 0x01E0, 1, 0x068C, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT2__EPIT2_EPITO                                       \
		IOMUX_PAD(0x04E8, 0x01E0, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT2__PWM3_PWMO                                         \
		IOMUX_PAD(0x04E8, 0x01E0, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT2__AUDMUX_AUD4_RXC                                   \
		IOMUX_PAD(0x04E8, 0x01E0, 4, 0x05EC, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT2__GPIO_2_22                                         \
		IOMUX_PAD(0x04E8, 0x01E0, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT2__KITTEN_TRACE_2                                    \
		IOMUX_PAD(0x04E8, 0x01E0, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT2__SRC_BT_CFG_2                                      \
		IOMUX_PAD(0x04E8, 0x01E0, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_LCD_DAT20__LCDIF_DAT_20                                     \
		IOMUX_PAD(0x04EC, 0x01E4, 0, 0x07C8, 1, MX6SL_LCDIF_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT20__KPP_COL_6                                        \
		IOMUX_PAD(0x04EC, 0x01E4, 1, 0x074C, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT20__CSI_D_13                                         \
		IOMUX_PAD(0x04EC, 0x01E4, 2, 0x0664, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT20__WEIM_WEIM_D_14                                   \
		IOMUX_PAD(0x04EC, 0x01E4, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT20__GPT_CMPOUT1                                      \
		IOMUX_PAD(0x04EC, 0x01E4, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT20__GPIO_3_8                                         \
		IOMUX_PAD(0x04EC, 0x01E4, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT20__KITTEN_TRACE_20                                  \
		IOMUX_PAD(0x04EC, 0x01E4, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT20__SRC_BT_CFG_28                                    \
		IOMUX_PAD(0x04EC, 0x01E4, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_LCD_DAT21__LCDIF_DAT_21                                     \
		IOMUX_PAD(0x04F0, 0x01E8, 0, 0x07CC, 1, MX6SL_LCDIF_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT21__KPP_ROW_6                                        \
		IOMUX_PAD(0x04F0, 0x01E8, 1, 0x076C, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT21__CSI_D_12                                         \
		IOMUX_PAD(0x04F0, 0x01E8, 2, 0x0660, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT21__WEIM_WEIM_D_15                                   \
		IOMUX_PAD(0x04F0, 0x01E8, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT21__GPT_CMPOUT2                                      \
		IOMUX_PAD(0x04F0, 0x01E8, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT21__GPIO_3_9                                         \
		IOMUX_PAD(0x04F0, 0x01E8, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT21__KITTEN_TRACE_21                                  \
		IOMUX_PAD(0x04F0, 0x01E8, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT21__SRC_BT_CFG_29                                    \
		IOMUX_PAD(0x04F0, 0x01E8, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_LCD_DAT22__LCDIF_DAT_22                                     \
		IOMUX_PAD(0x04F4, 0x01EC, 0, 0x07D0, 1, MX6SL_LCDIF_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT22__KPP_COL_7                                        \
		IOMUX_PAD(0x04F4, 0x01EC, 1, 0x0750, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT22__CSI_D_11                                         \
		IOMUX_PAD(0x04F4, 0x01EC, 2, 0x065C, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT22__WEIM_WEIM_EB_3                                   \
		IOMUX_PAD(0x04F4, 0x01EC, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT22__GPT_CMPOUT3                                      \
		IOMUX_PAD(0x04F4, 0x01EC, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT22__GPIO_3_10                                        \
		IOMUX_PAD(0x04F4, 0x01EC, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT22__KITTEN_TRACE_22                                  \
		IOMUX_PAD(0x04F4, 0x01EC, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT22__SRC_BT_CFG_30                                    \
		IOMUX_PAD(0x04F4, 0x01EC, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_LCD_DAT23__LCDIF_DAT_23                                     \
		IOMUX_PAD(0x04F8, 0x01F0, 0, 0x07D4, 1, MX6SL_LCDIF_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT23__KPP_ROW_7                                        \
		IOMUX_PAD(0x04F8, 0x01F0, 1, 0x0770, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT23__CSI_D_10                                         \
		IOMUX_PAD(0x04F8, 0x01F0, 2, 0x0658, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT23__WEIM_WEIM_EB_2                                   \
		IOMUX_PAD(0x04F8, 0x01F0, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT23__GPT_CLKIN                                        \
		IOMUX_PAD(0x04F8, 0x01F0, 4, 0x0718, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT23__GPIO_3_11                                        \
		IOMUX_PAD(0x04F8, 0x01F0, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT23__KITTEN_TRACE_23                                  \
		IOMUX_PAD(0x04F8, 0x01F0, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT23__SRC_BT_CFG_31                                    \
		IOMUX_PAD(0x04F8, 0x01F0, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_LCD_DAT3__LCDIF_DAT_3                                       \
		IOMUX_PAD(0x04FC, 0x01F4, 0, 0x0784, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT3__ECSPI1_SCLK                                       \
		IOMUX_PAD(0x04FC, 0x01F4, 1, 0x067C, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT3__UART5_DSR                                         \
		IOMUX_PAD(0x04FC, 0x01F4, 2, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT3__PWM4_PWMO                                         \
		IOMUX_PAD(0x04FC, 0x01F4, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT3__AUDMUX_AUD4_RXD                                   \
		IOMUX_PAD(0x04FC, 0x01F4, 4, 0x05E4, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT3__GPIO_2_23                                         \
		IOMUX_PAD(0x04FC, 0x01F4, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT3__KITTEN_TRACE_3                                    \
		IOMUX_PAD(0x04FC, 0x01F4, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT3__SRC_BT_CFG_3                                      \
		IOMUX_PAD(0x04FC, 0x01F4, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_LCD_DAT4__LCDIF_DAT_4                                       \
		IOMUX_PAD(0x0500, 0x01F8, 0, 0x0788, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT4__ECSPI1_SS1                                        \
		IOMUX_PAD(0x0500, 0x01F8, 1, 0x0690, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT4__CSI_VSYNC                                         \
		IOMUX_PAD(0x0500, 0x01F8, 2, 0x0678, 2, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT4__WDOG2_WDOG_RST_B_DEB                              \
		IOMUX_PAD(0x0500, 0x01F8, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT4__AUDMUX_AUD4_TXC                                   \
		IOMUX_PAD(0x0500, 0x01F8, 4, 0x05F4, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT4__GPIO_2_24                                         \
		IOMUX_PAD(0x0500, 0x01F8, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT4__KITTEN_TRACE_4                                    \
		IOMUX_PAD(0x0500, 0x01F8, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT4__SRC_BT_CFG_4                                      \
		IOMUX_PAD(0x0500, 0x01F8, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_LCD_DAT5__LCDIF_DAT_5                                       \
		IOMUX_PAD(0x0504, 0x01FC, 0, 0x078C, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT5__ECSPI1_SS2                                        \
		IOMUX_PAD(0x0504, 0x01FC, 1, 0x0694, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT5__CSI_HSYNC                                         \
		IOMUX_PAD(0x0504, 0x01FC, 2, 0x0670, 2, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT5__WEIM_WEIM_CS_3                                    \
		IOMUX_PAD(0x0504, 0x01FC, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT5__AUDMUX_AUD4_TXFS                                  \
		IOMUX_PAD(0x0504, 0x01FC, 4, 0x05F8, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT5__GPIO_2_25                                         \
		IOMUX_PAD(0x0504, 0x01FC, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT5__KITTEN_TRACE_5                                    \
		IOMUX_PAD(0x0504, 0x01FC, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT5__SRC_BT_CFG_5                                      \
		IOMUX_PAD(0x0504, 0x01FC, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_LCD_DAT6__LCDIF_DAT_6                                       \
		IOMUX_PAD(0x0508, 0x0200, 0, 0x0790, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT6__ECSPI1_SS3                                        \
		IOMUX_PAD(0x0508, 0x0200, 1, 0x0698, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT6__CSI_PIXCLK                                        \
		IOMUX_PAD(0x0508, 0x0200, 2, 0x0674, 2, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT6__WEIM_WEIM_D_0                                     \
		IOMUX_PAD(0x0508, 0x0200, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT6__AUDMUX_AUD4_TXD                                   \
		IOMUX_PAD(0x0508, 0x0200, 4, 0x05E8, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT6__GPIO_2_26                                         \
		IOMUX_PAD(0x0508, 0x0200, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT6__KITTEN_TRACE_6                                    \
		IOMUX_PAD(0x0508, 0x0200, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT6__SRC_BT_CFG_6                                      \
		IOMUX_PAD(0x0508, 0x0200, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_LCD_DAT7__LCDIF_DAT_7                                       \
		IOMUX_PAD(0x050C, 0x0204, 0, 0x0794, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT7__ECSPI1_RDY                                        \
		IOMUX_PAD(0x050C, 0x0204, 1, 0x0680, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT7__CSI_MCLK                                          \
		IOMUX_PAD(0x050C, 0x0204, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT7__WEIM_WEIM_D_1                                     \
		IOMUX_PAD(0x050C, 0x0204, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT7__AUDMUX_AUDIO_CLK_OUT                              \
		IOMUX_PAD(0x050C, 0x0204, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT7__GPIO_2_27                                         \
		IOMUX_PAD(0x050C, 0x0204, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT7__KITTEN_TRACE_7                                    \
		IOMUX_PAD(0x050C, 0x0204, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT7__SRC_BT_CFG_7                                      \
		IOMUX_PAD(0x050C, 0x0204, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_LCD_DAT8__LCDIF_DAT_8                                       \
		IOMUX_PAD(0x0510, 0x0208, 0, 0x0798, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT8__KPP_COL_0                                         \
		IOMUX_PAD(0x0510, 0x0208, 1, 0x0734, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT8__CSI_D_9                                           \
		IOMUX_PAD(0x0510, 0x0208, 2, 0x0654, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT8__WEIM_WEIM_D_2                                     \
		IOMUX_PAD(0x0510, 0x0208, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT8__ECSPI2_SCLK                                       \
		IOMUX_PAD(0x0510, 0x0208, 4, 0x069C, 2, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT8__GPIO_2_28                                         \
		IOMUX_PAD(0x0510, 0x0208, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT8__KITTEN_TRACE_8                                    \
		IOMUX_PAD(0x0510, 0x0208, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT8__SRC_BT_CFG_8                                      \
		IOMUX_PAD(0x0510, 0x0208, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_LCD_DAT9__LCDIF_DAT_9                                       \
		IOMUX_PAD(0x0514, 0x020C, 0, 0x079C, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT9__KPP_ROW_0                                         \
		IOMUX_PAD(0x0514, 0x020C, 1, 0x0754, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT9__CSI_D_8                                           \
		IOMUX_PAD(0x0514, 0x020C, 2, 0x0650, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT9__WEIM_WEIM_D_3                                     \
		IOMUX_PAD(0x0514, 0x020C, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT9__ECSPI2_MOSI                                       \
		IOMUX_PAD(0x0514, 0x020C, 4, 0x06A4, 2, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT9__GPIO_2_29                                         \
		IOMUX_PAD(0x0514, 0x020C, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT9__KITTEN_TRACE_9                                    \
		IOMUX_PAD(0x0514, 0x020C, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_DAT9__SRC_BT_CFG_9                                      \
		IOMUX_PAD(0x0514, 0x020C, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_LCD_ENABLE__LCDIF_ENABLE                                    \
		IOMUX_PAD(0x0518, 0x0210, 0, 0x0000, 0, MX6SL_LCDIF_PAD_CTRL)
#define MX6SL_PAD_LCD_ENABLE__USDHC4_DAT5                                     \
		IOMUX_PAD(0x0518, 0x0210, 1, 0x0870, 2, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_LCD_ENABLE__LCDIF_RD_E                                      \
		IOMUX_PAD(0x0518, 0x0210, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_ENABLE__WEIM_WEIM_OE                                    \
		IOMUX_PAD(0x0518, 0x0210, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_ENABLE__UART2_TXD                                       \
		IOMUX_PAD(0x0518, 0x0210, 4, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_LCD_ENABLE__UART2_RXD                                       \
		IOMUX_PAD(0x0518, 0x0210, 4, 0x0804, 2, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_LCD_ENABLE__GPIO_2_16                                       \
		IOMUX_PAD(0x0518, 0x0210, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_ENABLE__OCOTP_CTRL_WRAPPER_FUSE_LATCHED                 \
		IOMUX_PAD(0x0518, 0x0210, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_ENABLE__TPSMP_HTRANS_1                                  \
		IOMUX_PAD(0x0518, 0x0210, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_LCD_HSYNC__LCDIF_HSYNC                                      \
		IOMUX_PAD(0x051C, 0x0214, 0, 0x0774, 0, MX6SL_LCDIF_PAD_CTRL)
#define MX6SL_PAD_LCD_HSYNC__USDHC4_DAT6                                      \
		IOMUX_PAD(0x051C, 0x0214, 1, 0x0874, 2, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_LCD_HSYNC__LCDIF_CS                                         \
		IOMUX_PAD(0x051C, 0x0214, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_HSYNC__WEIM_WEIM_CS_0                                   \
		IOMUX_PAD(0x051C, 0x0214, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_HSYNC__UART2_TXD                                        \
		IOMUX_PAD(0x051C, 0x0214, 4, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_LCD_HSYNC__UART2_RXD                                        \
		IOMUX_PAD(0x051C, 0x0214, 4, 0x0804, 3, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_LCD_HSYNC__GPIO_2_17                                        \
		IOMUX_PAD(0x051C, 0x0214, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_HSYNC__KITTEN_TRCLK                                     \
		IOMUX_PAD(0x051C, 0x0214, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_HSYNC__TPSMP_HDATA_16                                   \
		IOMUX_PAD(0x051C, 0x0214, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_LCD_RESET__LCDIF_RESET                                      \
		IOMUX_PAD(0x0520, 0x0218, 0, 0x0000, 0, MX6SL_LCDIF_PAD_CTRL)
#define MX6SL_PAD_LCD_RESET__WEIM_WEIM_DTACK_B                                \
		IOMUX_PAD(0x0520, 0x0218, 1, 0x0880, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_RESET__LCDIF_BUSY                                       \
		IOMUX_PAD(0x0520, 0x0218, 2, 0x0774, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_RESET__WEIM_WEIM_WAIT                                   \
		IOMUX_PAD(0x0520, 0x0218, 3, 0x0884, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_RESET__UART2_CTS                                        \
		IOMUX_PAD(0x0520, 0x0218, 4, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_LCD_RESET__UART2_RTS                                        \
		IOMUX_PAD(0x0520, 0x0218, 4, 0x0800, 2, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_LCD_RESET__GPIO_2_19                                        \
		IOMUX_PAD(0x0520, 0x0218, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_RESET__CCM_PMIC_RDY                                     \
		IOMUX_PAD(0x0520, 0x0218, 6, 0x062C, 1, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_RESET__TPSMP_HDATA_DIR                                  \
		IOMUX_PAD(0x0520, 0x0218, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_LCD_VSYNC__LCDIF_VSYNC                                      \
		IOMUX_PAD(0x0524, 0x021C, 0, 0x0000, 0, MX6SL_LCDIF_PAD_CTRL)
#define MX6SL_PAD_LCD_VSYNC__USDHC4_DAT7                                      \
		IOMUX_PAD(0x0524, 0x021C, 1, 0x0878, 2, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_LCD_VSYNC__LCDIF_RS                                         \
		IOMUX_PAD(0x0524, 0x021C, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_VSYNC__WEIM_WEIM_CS_1                                   \
		IOMUX_PAD(0x0524, 0x021C, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_VSYNC__UART2_CTS                                        \
		IOMUX_PAD(0x0524, 0x021C, 4, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_LCD_VSYNC__UART2_RTS                                        \
		IOMUX_PAD(0x0524, 0x021C, 4, 0x0800, 3, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_LCD_VSYNC__GPIO_2_18                                        \
		IOMUX_PAD(0x0524, 0x021C, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_VSYNC__KITTEN_TRCTL                                     \
		IOMUX_PAD(0x0524, 0x021C, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_LCD_VSYNC__TPSMP_HDATA_17                                   \
		IOMUX_PAD(0x0524, 0x021C, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_PMIC_ON_REQ__SNVS_LP_WRAPPER_SNVS_WAKEUP_ALARM              \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_PMIC_STBY_REQ__CCM_PMIC_VSTBY_REQ                           \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_POR_B__SRC_POR_B                                            \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_PWM1__PWM1_PWMO                                             \
		IOMUX_PAD(0x0528, 0x0220, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_PWM1__CCM_CLKO                                              \
		IOMUX_PAD(0x0528, 0x0220, 1, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_PWM1__AUDMUX_AUDIO_CLK_OUT                                  \
		IOMUX_PAD(0x0528, 0x0220, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_PWM1__FEC_REF_OUT                                           \
		IOMUX_PAD(0x0528, 0x0220, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_PWM1__CSI_MCLK                                              \
		IOMUX_PAD(0x0528, 0x0220, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_PWM1__GPIO_3_23                                             \
		IOMUX_PAD(0x0528, 0x0220, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_PWM1__EPIT1_EPITO                                           \
		IOMUX_PAD(0x0528, 0x0220, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_PWM1__OBSERVE_MUX_OUT_4                                     \
		IOMUX_PAD(0x0528, 0x0220, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_REF_CLK_24M__ANATOP_ANATOP_24M_OUT                          \
		IOMUX_PAD(0x052C, 0x0224, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_REF_CLK_24M__I2C3_SCL                                       \
		IOMUX_PAD(0x052C, 0x0224, 1 | IOMUX_CONFIG_SION, 0x072C, 2, MX6SL_I2C_PAD_CTRL)
#define MX6SL_PAD_REF_CLK_24M__PWM3_PWMO                                      \
		IOMUX_PAD(0x052C, 0x0224, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_REF_CLK_24M__ANATOP_USBOTG2_ID                              \
		IOMUX_PAD(0x052C, 0x0224, 3, 0x05E0, 2, NO_PAD_CTRL)
#define MX6SL_PAD_REF_CLK_24M__CCM_PMIC_RDY                                   \
		IOMUX_PAD(0x052C, 0x0224, 4, 0x062C, 2, NO_PAD_CTRL)
#define MX6SL_PAD_REF_CLK_24M__GPIO_3_21                                      \
		IOMUX_PAD(0x052C, 0x0224, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_REF_CLK_24M__USDHC3_WP                                      \
		IOMUX_PAD(0x052C, 0x0224, 6, 0x084C, 3, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_REF_CLK_24M__TPSMP_HDATA_19                                 \
		IOMUX_PAD(0x052C, 0x0224, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_REF_CLK_32K__ANATOP_ANATOP_32K_OUT                          \
		IOMUX_PAD(0x0530, 0x0228, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_REF_CLK_32K__I2C3_SDA                                       \
		IOMUX_PAD(0x0530, 0x0228, 1 | IOMUX_CONFIG_SION, 0x0730, 2, MX6SL_I2C_PAD_CTRL)
#define MX6SL_PAD_REF_CLK_32K__PWM4_PWMO                                      \
		IOMUX_PAD(0x0530, 0x0228, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_REF_CLK_32K__ANATOP_USBOTG1_ID                              \
		IOMUX_PAD(0x0530, 0x0228, 3, 0x05DC, 3, NO_PAD_CTRL)
#define MX6SL_PAD_REF_CLK_32K__USDHC1_LCTL                                    \
		IOMUX_PAD(0x0530, 0x0228, 4, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_REF_CLK_32K__GPIO_3_22                                      \
		IOMUX_PAD(0x0530, 0x0228, 5, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_REF_CLK_32K__USDHC3_CD                                      \
		IOMUX_PAD(0x0530, 0x0228, 6, 0x0838, 3, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_REF_CLK_32K__OBSERVE_MUX_OUT_3                              \
		IOMUX_PAD(0x0530, 0x0228, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_RESET_IN_B__SRC_RESET_B                                     \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_SD1_CLK__USDHC1_CLK_50MHZ                                   \
		IOMUX_PAD(0x0534, 0x022C, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD1_CLK__USDHC1_CLK_100MHZ                                  \
	IOMUX_PAD(0x0534, 0x022C, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_100MHZ)
#define MX6SL_PAD_SD1_CLK__USDHC1_CLK_200MHZ                                  \
	IOMUX_PAD(0x0534, 0x022C, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_200MHZ)
#define MX6SL_PAD_SD1_CLK__FEC_MDIO                                           \
		IOMUX_PAD(0x0534, 0x022C, 1, 0x06F4, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_CLK__KPP_COL_0                                          \
		IOMUX_PAD(0x0534, 0x022C, 2, 0x0734, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_CLK__EPDC_SDCE_4                                        \
		IOMUX_PAD(0x0534, 0x022C, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_CLK__MSHC_SCLK                                          \
		IOMUX_PAD(0x0534, 0x022C, 4, 0x07E8, 1, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_CLK__GPIO_5_15                                          \
		IOMUX_PAD(0x0534, 0x022C, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_CLK__ANATOP_ANATOP_TESTO_2                              \
		IOMUX_PAD(0x0534, 0x022C, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_CLK__PL301_SIM_MX6SL_PER1_HADDR_25                      \
		IOMUX_PAD(0x0534, 0x022C, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_SD1_CMD__USDHC1_CMD_50MHZ                                   \
		IOMUX_PAD(0x0538, 0x0230, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD1_CMD__USDHC1_CMD_100MHZ                                  \
	IOMUX_PAD(0x0538, 0x0230, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_100MHZ)
#define MX6SL_PAD_SD1_CMD__USDHC1_CMD_200MHZ                                  \
	IOMUX_PAD(0x0538, 0x0230, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_200MHZ)
#define MX6SL_PAD_SD1_CMD__FEC_TX_CLK                                         \
		IOMUX_PAD(0x0538, 0x0230, 1, 0x070C, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_CMD__KPP_ROW_0                                          \
		IOMUX_PAD(0x0538, 0x0230, 2, 0x0754, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_CMD__EPDC_SDCE_5                                        \
		IOMUX_PAD(0x0538, 0x0230, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_CMD__MSHC_BS                                            \
		IOMUX_PAD(0x0538, 0x0230, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_CMD__GPIO_5_14                                          \
		IOMUX_PAD(0x0538, 0x0230, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_CMD__ANATOP_ANATOP_TESTO_3                              \
		IOMUX_PAD(0x0538, 0x0230, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_CMD__PL301_SIM_MX6SL_PER1_HADDR_26                      \
		IOMUX_PAD(0x0538, 0x0230, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_SD1_DAT0__USDHC1_DAT0_50MHZ                                 \
		IOMUX_PAD(0x053C, 0x0234, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT0__USDHC1_DAT0_100MHZ                                \
	IOMUX_PAD(0x053C, 0x0234, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_100MHZ)
#define MX6SL_PAD_SD1_DAT0__USDHC1_DAT0_200MHZ                                \
	IOMUX_PAD(0x053C, 0x0234, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_200MHZ)
#define MX6SL_PAD_SD1_DAT0__FEC_RX_ER                                         \
		IOMUX_PAD(0x053C, 0x0234, 1, 0x0708, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT0__KPP_COL_1                                         \
		IOMUX_PAD(0x053C, 0x0234, 2, 0x0738, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT0__EPDC_SDCE_6                                       \
		IOMUX_PAD(0x053C, 0x0234, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT0__MSHC_DATA_0                                       \
		IOMUX_PAD(0x053C, 0x0234, 4, 0x07D8, 1, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT0__GPIO_5_11                                         \
		IOMUX_PAD(0x053C, 0x0234, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT0__ANATOP_ANATOP_TESTO_4                             \
		IOMUX_PAD(0x053C, 0x0234, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT0__PL301_SIM_MX6SL_PER1_HADDR_27                     \
		IOMUX_PAD(0x053C, 0x0234, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_SD1_DAT1__USDHC1_DAT1_50MHZ                                 \
		IOMUX_PAD(0x0540, 0x0238, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT1__USDHC1_DAT1_100MHZ                                \
	IOMUX_PAD(0x0540, 0x0238, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_100MHZ)
#define MX6SL_PAD_SD1_DAT1__USDHC1_DAT1_200MHZ                                \
	IOMUX_PAD(0x0540, 0x0238, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_200MHZ)
#define MX6SL_PAD_SD1_DAT1__FEC_RX_DV                                         \
		IOMUX_PAD(0x0540, 0x0238, 1, 0x0704, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT1__KPP_ROW_1                                         \
		IOMUX_PAD(0x0540, 0x0238, 2, 0x0758, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT1__EPDC_SDCE_7                                       \
		IOMUX_PAD(0x0540, 0x0238, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT1__MSHC_DATA_1                                       \
		IOMUX_PAD(0x0540, 0x0238, 4, 0x07DC, 1, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT1__GPIO_5_8                                          \
		IOMUX_PAD(0x0540, 0x0238, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT1__ANATOP_ANATOP_TESTO_5                             \
		IOMUX_PAD(0x0540, 0x0238, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT1__PL301_SIM_MX6SL_PER1_HADDR_28                     \
		IOMUX_PAD(0x0540, 0x0238, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_SD1_DAT2__USDHC1_DAT2_50MHZ                                 \
		IOMUX_PAD(0x0544, 0x023C, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT2__USDHC1_DAT2_100MHZ                                \
	IOMUX_PAD(0x0544, 0x023C, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_100MHZ)
#define MX6SL_PAD_SD1_DAT2__USDHC1_DAT2_200MHZ                                \
	IOMUX_PAD(0x0544, 0x023C, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_200MHZ)
#define MX6SL_PAD_SD1_DAT2__FEC_RDATA_1                                       \
		IOMUX_PAD(0x0544, 0x023C, 1, 0x06FC, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT2__KPP_COL_2                                         \
		IOMUX_PAD(0x0544, 0x023C, 2, 0x073C, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT2__EPDC_SDCE_8                                       \
		IOMUX_PAD(0x0544, 0x023C, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT2__MSHC_DATA_2                                       \
		IOMUX_PAD(0x0544, 0x023C, 4, 0x07E0, 1, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT2__GPIO_5_13                                         \
		IOMUX_PAD(0x0544, 0x023C, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT2__ANATOP_ANATOP_TESTO_6                             \
		IOMUX_PAD(0x0544, 0x023C, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT2__PL301_SIM_MX6SL_PER1_HADDR_29                     \
		IOMUX_PAD(0x0544, 0x023C, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_SD1_DAT3__USDHC1_DAT3_50MHZ                                 \
		IOMUX_PAD(0x0548, 0x0240, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT3__USDHC1_DAT3_100MHZ                                \
	IOMUX_PAD(0x0548, 0x0240, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_100MHZ)
#define MX6SL_PAD_SD1_DAT3__USDHC1_DAT3_200MHZ                                \
	IOMUX_PAD(0x0548, 0x0240, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_200MHZ)
#define MX6SL_PAD_SD1_DAT3__FEC_TDATA_0                                       \
		IOMUX_PAD(0x0548, 0x0240, 1, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT3__KPP_ROW_2                                         \
		IOMUX_PAD(0x0548, 0x0240, 2, 0x075C, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT3__EPDC_SDCE_9                                       \
		IOMUX_PAD(0x0548, 0x0240, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT3__MSHC_DATA_3                                       \
		IOMUX_PAD(0x0548, 0x0240, 4, 0x07E4, 1, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT3__GPIO_5_6                                          \
		IOMUX_PAD(0x0548, 0x0240, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT3__ANATOP_ANATOP_TESTO_7                             \
		IOMUX_PAD(0x0548, 0x0240, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT3__PL301_SIM_MX6SL_PER1_HADDR_30                     \
		IOMUX_PAD(0x0548, 0x0240, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_SD1_DAT4__USDHC1_DAT4_50MHZ                                 \
		IOMUX_PAD(0x054C, 0x0244, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT4__USDHC1_DAT4_100MHZ                                \
	IOMUX_PAD(0x054C, 0x0244, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_100MHZ)
#define MX6SL_PAD_SD1_DAT4__USDHC1_DAT4_200MHZ                                \
	IOMUX_PAD(0x054C, 0x0244, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_200MHZ)
#define MX6SL_PAD_SD1_DAT4__FEC_MDC                                           \
		IOMUX_PAD(0x054C, 0x0244, 1, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT4__KPP_COL_3                                         \
		IOMUX_PAD(0x054C, 0x0244, 2, 0x0740, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT4__EPDC_SDCLKN                                       \
		IOMUX_PAD(0x054C, 0x0244, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT4__UART4_TXD                                         \
		IOMUX_PAD(0x054C, 0x0244, 4, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT4__UART4_RXD                                         \
		IOMUX_PAD(0x054C, 0x0244, 4, 0x0814, 4, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT4__GPIO_5_12                                         \
		IOMUX_PAD(0x054C, 0x0244, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT4__ANATOP_ANATOP_TESTO_8                             \
		IOMUX_PAD(0x054C, 0x0244, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT4__PL301_SIM_MX6SL_PER1_HADDR_31                     \
		IOMUX_PAD(0x054C, 0x0244, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_SD1_DAT5__USDHC1_DAT5_50MHZ                                 \
		IOMUX_PAD(0x0550, 0x0248, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT5__USDHC1_DAT5_100MHZ                                \
	IOMUX_PAD(0x0550, 0x0248, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_100MHZ)
#define MX6SL_PAD_SD1_DAT5__USDHC1_DAT5_200MHZ                                \
	IOMUX_PAD(0x0550, 0x0248, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_200MHZ)
#define MX6SL_PAD_SD1_DAT5__FEC_RDATA_0                                       \
		IOMUX_PAD(0x0550, 0x0248, 1, 0x06F8, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT5__KPP_ROW_3                                         \
		IOMUX_PAD(0x0550, 0x0248, 2, 0x0760, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT5__EPDC_SDOED                                        \
		IOMUX_PAD(0x0550, 0x0248, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT5__UART4_TXD                                         \
		IOMUX_PAD(0x0550, 0x0248, 4, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT5__UART4_RXD                                         \
		IOMUX_PAD(0x0550, 0x0248, 4, 0x0814, 5, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT5__GPIO_5_9                                          \
		IOMUX_PAD(0x0550, 0x0248, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT5__ANATOP_ANATOP_TESTO_9                             \
		IOMUX_PAD(0x0550, 0x0248, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT5__PL301_SIM_MX6SL_PER1_HPROT_3                      \
		IOMUX_PAD(0x0550, 0x0248, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_SD1_DAT6__USDHC1_DAT6_50MHZ                                 \
		IOMUX_PAD(0x0554, 0x024C, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT6__USDHC1_DAT6_100MHZ                                \
	IOMUX_PAD(0x0554, 0x024C, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_100MHZ)
#define MX6SL_PAD_SD1_DAT6__USDHC1_DAT6_200MHZ                                \
	IOMUX_PAD(0x0554, 0x024C, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_200MHZ)
#define MX6SL_PAD_SD1_DAT6__FEC_TX_EN                                         \
		IOMUX_PAD(0x0554, 0x024C, 1, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT6__KPP_COL_4                                         \
		IOMUX_PAD(0x0554, 0x024C, 2, 0x0744, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT6__EPDC_SDOEZ                                        \
		IOMUX_PAD(0x0554, 0x024C, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT6__UART4_CTS                                         \
		IOMUX_PAD(0x0554, 0x024C, 4, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT6__UART4_RTS                                         \
		IOMUX_PAD(0x0554, 0x024C, 4, 0x0810, 4, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT6__GPIO_5_7                                          \
		IOMUX_PAD(0x0554, 0x024C, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT6__ANATOP_ANATOP_TESTO_10                            \
		IOMUX_PAD(0x0554, 0x024C, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT6__PL301_SIM_MX6SL_PER1_HPROT_2                      \
		IOMUX_PAD(0x0554, 0x024C, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_SD1_DAT7__USDHC1_DAT7_50MHZ                                 \
		IOMUX_PAD(0x0558, 0x0250, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT7__USDHC1_DAT7_100MHZ                                \
	IOMUX_PAD(0x0558, 0x0250, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_100MHZ)
#define MX6SL_PAD_SD1_DAT7__USDHC1_DAT7_200MHZ                                \
	IOMUX_PAD(0x0558, 0x0250, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_200MHZ)
#define MX6SL_PAD_SD1_DAT7__FEC_TDATA_1                                       \
		IOMUX_PAD(0x0558, 0x0250, 1, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT7__KPP_ROW_4                                         \
		IOMUX_PAD(0x0558, 0x0250, 2, 0x0764, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT7__CCM_PMIC_RDY                                      \
		IOMUX_PAD(0x0558, 0x0250, 3, 0x062C, 3, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT7__UART4_CTS                                         \
		IOMUX_PAD(0x0558, 0x0250, 4, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT7__UART4_RTS                                         \
		IOMUX_PAD(0x0558, 0x0250, 4, 0x0810, 5, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT7__GPIO_5_10                                         \
		IOMUX_PAD(0x0558, 0x0250, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT7__ANATOP_ANATOP_TESTO_11                            \
		IOMUX_PAD(0x0558, 0x0250, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD1_DAT7__PL301_SIM_MX6SL_PER1_HMASTLOCK                    \
		IOMUX_PAD(0x0558, 0x0250, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_SD2_CLK__USDHC2_CLK_50MHZ                                   \
		IOMUX_PAD(0x055C, 0x0254, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD2_CLK__USDHC2_CLK_100MHZ                                  \
	IOMUX_PAD(0x055C, 0x0254, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_100MHZ)
#define MX6SL_PAD_SD2_CLK__USDHC2_CLK_200MHZ                                  \
	IOMUX_PAD(0x055C, 0x0254, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_200MHZ)
#define MX6SL_PAD_SD2_CLK__AUDMUX_AUD4_RXFS                                   \
		IOMUX_PAD(0x055C, 0x0254, 1, 0x05F0, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_CLK__ECSPI3_SCLK                                        \
		IOMUX_PAD(0x055C, 0x0254, 2, 0x06B0, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_CLK__CSI_D_0                                            \
		IOMUX_PAD(0x055C, 0x0254, 3, 0x0630, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_CLK__OSC32K_32K_OUT                                     \
		IOMUX_PAD(0x055C, 0x0254, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_CLK__GPIO_5_5                                           \
		IOMUX_PAD(0x055C, 0x0254, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_CLK__ANATOP_ANATOP_TESTO_13                             \
		IOMUX_PAD(0x055C, 0x0254, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_CLK__PL301_SIM_MX6SL_PER1_HPROT_1                       \
		IOMUX_PAD(0x055C, 0x0254, 7, 0x07EC, 1, NO_PAD_CTRL)

#define MX6SL_PAD_SD2_CMD__USDHC2_CMD_50MHZ                                   \
		IOMUX_PAD(0x0560, 0x0258, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD2_CMD__USDHC2_CMD_100MHZ                                  \
	IOMUX_PAD(0x0560, 0x0258, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_100MHZ)
#define MX6SL_PAD_SD2_CMD__USDHC2_CMD_200MHZ                                  \
	IOMUX_PAD(0x0560, 0x0258, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_200MHZ)
#define MX6SL_PAD_SD2_CMD__AUDMUX_AUD4_RXC                                    \
		IOMUX_PAD(0x0560, 0x0258, 1, 0x05EC, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_CMD__ECSPI3_SS0                                         \
		IOMUX_PAD(0x0560, 0x0258, 2, 0x06C0, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_CMD__CSI_D_1                                            \
		IOMUX_PAD(0x0560, 0x0258, 3, 0x0634, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_CMD__EPIT1_EPITO                                        \
		IOMUX_PAD(0x0560, 0x0258, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_CMD__GPIO_5_4                                           \
		IOMUX_PAD(0x0560, 0x0258, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_CMD__ANATOP_ANATOP_TESTO_14                             \
		IOMUX_PAD(0x0560, 0x0258, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_CMD__PL301_SIM_MX6SL_PER1_HADDR_21                      \
		IOMUX_PAD(0x0560, 0x0258, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_SD2_DAT0__USDHC2_DAT0_50MHZ                                 \
		IOMUX_PAD(0x0564, 0x025C, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT0__USDHC2_DAT0_100MHZ                                \
	IOMUX_PAD(0x0564, 0x025C, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_100MHZ)
#define MX6SL_PAD_SD2_DAT0__USDHC2_DAT0_200MHZ                                \
	IOMUX_PAD(0x0564, 0x025C, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_200MHZ)
#define MX6SL_PAD_SD2_DAT0__AUDMUX_AUD4_RXD                                   \
		IOMUX_PAD(0x0564, 0x025C, 1, 0x05E4, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT0__ECSPI3_MOSI                                       \
		IOMUX_PAD(0x0564, 0x025C, 2, 0x06BC, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT0__CSI_D_2                                           \
		IOMUX_PAD(0x0564, 0x025C, 3, 0x0638, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT0__UART5_CTS                                         \
		IOMUX_PAD(0x0564, 0x025C, 4, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT0__UART5_RTS                                         \
		IOMUX_PAD(0x0564, 0x025C, 4, 0x0818, 4, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT0__GPIO_5_1                                          \
		IOMUX_PAD(0x0564, 0x025C, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT0__ANATOP_ANATOP_TESTO_15                            \
		IOMUX_PAD(0x0564, 0x025C, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT0__PL301_SIM_MX6SL_PER1_HPROT_0                      \
		IOMUX_PAD(0x0564, 0x025C, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_SD2_DAT1__USDHC2_DAT1_50MHZ                                 \
		IOMUX_PAD(0x0568, 0x0260, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT1__USDHC2_DAT1_100MHZ                                \
	IOMUX_PAD(0x0568, 0x0260, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_100MHZ)
#define MX6SL_PAD_SD2_DAT1__USDHC2_DAT1_200MHZ                                \
	IOMUX_PAD(0x0568, 0x0260, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_200MHZ)
#define MX6SL_PAD_SD2_DAT1__AUDMUX_AUD4_TXC                                   \
		IOMUX_PAD(0x0568, 0x0260, 1, 0x05F4, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT1__ECSPI3_MISO                                       \
		IOMUX_PAD(0x0568, 0x0260, 2, 0x06B8, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT1__CSI_D_3                                           \
		IOMUX_PAD(0x0568, 0x0260, 3, 0x063C, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT1__UART5_CTS                                         \
		IOMUX_PAD(0x0568, 0x0260, 4, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT1__UART5_RTS                                         \
		IOMUX_PAD(0x0568, 0x0260, 4, 0x0818, 5, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT1__GPIO_4_30                                         \
		IOMUX_PAD(0x0568, 0x0260, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT1__MMDC_MMDC_DEBUG_39                                \
		IOMUX_PAD(0x0568, 0x0260, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT1__PL301_SIM_MX6SL_PER1_HBURST_1                     \
		IOMUX_PAD(0x0568, 0x0260, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_SD2_DAT2__USDHC2_DAT2_50MHZ                                 \
		IOMUX_PAD(0x056C, 0x0264, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT2__USDHC2_DAT2_100MHZ                                \
	IOMUX_PAD(0x056C, 0x0264, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_100MHZ)
#define MX6SL_PAD_SD2_DAT2__USDHC2_DAT2_200MHZ                                \
	IOMUX_PAD(0x056C, 0x0264, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_200MHZ)
#define MX6SL_PAD_SD2_DAT2__AUDMUX_AUD4_TXFS                                  \
		IOMUX_PAD(0x056C, 0x0264, 1, 0x05F8, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT2__FEC_COL                                           \
		IOMUX_PAD(0x056C, 0x0264, 2, 0x06F0, 1, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT2__CSI_D_4                                           \
		IOMUX_PAD(0x056C, 0x0264, 3, 0x0640, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT2__UART5_TXD                                         \
		IOMUX_PAD(0x056C, 0x0264, 4, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT2__UART5_RXD                                         \
		IOMUX_PAD(0x056C, 0x0264, 4, 0x081C, 4, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT2__GPIO_5_3                                          \
		IOMUX_PAD(0x056C, 0x0264, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT2__MMDC_MMDC_DEBUG_38                                \
		IOMUX_PAD(0x056C, 0x0264, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT2__PL301_SIM_MX6SL_PER1_HADDR_22                     \
		IOMUX_PAD(0x056C, 0x0264, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_SD2_DAT3__USDHC2_DAT3_50MHZ                                 \
		IOMUX_PAD(0x0570, 0x0268, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT3__USDHC2_DAT3_100MHZ                                \
	IOMUX_PAD(0x0570, 0x0268, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_100MHZ)
#define MX6SL_PAD_SD2_DAT3__USDHC2_DAT3_200MHZ                                \
	IOMUX_PAD(0x0570, 0x0268, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_200MHZ)
#define MX6SL_PAD_SD2_DAT3__AUDMUX_AUD4_TXD                                   \
		IOMUX_PAD(0x0570, 0x0268, 1, 0x05E8, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT3__FEC_RX_CLK                                        \
		IOMUX_PAD(0x0570, 0x0268, 2, 0x0700, 1, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT3__CSI_D_5                                           \
		IOMUX_PAD(0x0570, 0x0268, 3, 0x0644, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT3__UART5_TXD                                         \
		IOMUX_PAD(0x0570, 0x0268, 4, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT3__UART5_RXD                                         \
		IOMUX_PAD(0x0570, 0x0268, 4, 0x081C, 5, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT3__GPIO_4_28                                         \
		IOMUX_PAD(0x0570, 0x0268, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT3__MMDC_MMDC_DEBUG_37                                \
		IOMUX_PAD(0x0570, 0x0268, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT3__PL301_SIM_MX6SL_PER1_HBURST_0                     \
		IOMUX_PAD(0x0570, 0x0268, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_SD2_DAT4__USDHC2_DAT4_50MHZ                                 \
		IOMUX_PAD(0x0574, 0x026C, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT4__USDHC2_DAT4_100MHZ                                \
	IOMUX_PAD(0x0574, 0x026C, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_100MHZ)
#define MX6SL_PAD_SD2_DAT4__USDHC2_DAT4_200MHZ                                \
	IOMUX_PAD(0x0574, 0x026C, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_200MHZ)
#define MX6SL_PAD_SD2_DAT4__USDHC3_DAT4                                       \
		IOMUX_PAD(0x0574, 0x026C, 1, 0x083C, 1, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT4__UART2_TXD                                         \
		IOMUX_PAD(0x0574, 0x026C, 2, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT4__UART2_RXD                                         \
		IOMUX_PAD(0x0574, 0x026C, 2, 0x0804, 4, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT4__CSI_D_6                                           \
		IOMUX_PAD(0x0574, 0x026C, 3, 0x0648, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT4__SPDIF_OUT1                                        \
		IOMUX_PAD(0x0574, 0x026C, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT4__GPIO_5_2                                          \
		IOMUX_PAD(0x0574, 0x026C, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT4__MMDC_MMDC_DEBUG_36                                \
		IOMUX_PAD(0x0574, 0x026C, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT4__PL301_SIM_MX6SL_PER1_HADDR_10                     \
		IOMUX_PAD(0x0574, 0x026C, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_SD2_DAT5__USDHC2_DAT5_50MHZ                                 \
		IOMUX_PAD(0x0578, 0x0270, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT5__USDHC2_DAT5_100MHZ                                \
	IOMUX_PAD(0x0578, 0x0270, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_100MHZ)
#define MX6SL_PAD_SD2_DAT5__USDHC2_DAT5_200MHZ                                \
	IOMUX_PAD(0x0578, 0x0270, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_200MHZ)
#define MX6SL_PAD_SD2_DAT5__USDHC3_DAT5                                       \
		IOMUX_PAD(0x0578, 0x0270, 1, 0x0840, 1, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT5__UART2_TXD                                         \
		IOMUX_PAD(0x0578, 0x0270, 2, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT5__UART2_RXD                                         \
		IOMUX_PAD(0x0578, 0x0270, 2, 0x0804, 5, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT5__CSI_D_7                                           \
		IOMUX_PAD(0x0578, 0x0270, 3, 0x064C, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT5__SPDIF_IN1                                         \
		IOMUX_PAD(0x0578, 0x0270, 4, 0x07F0, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT5__GPIO_4_31                                         \
		IOMUX_PAD(0x0578, 0x0270, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT5__MMDC_MMDC_DEBUG_35                                \
		IOMUX_PAD(0x0578, 0x0270, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT5__PL301_SIM_MX6SL_PER1_HADDR_20                     \
		IOMUX_PAD(0x0578, 0x0270, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_SD2_DAT6__USDHC2_DAT6_50MHZ                                 \
		IOMUX_PAD(0x057C, 0x0274, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT6__USDHC2_DAT6_100MHZ                                \
	IOMUX_PAD(0x057C, 0x0274, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_100MHZ)
#define MX6SL_PAD_SD2_DAT6__USDHC2_DAT6_200MHZ                                \
	IOMUX_PAD(0x057C, 0x0274, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_200MHZ)
#define MX6SL_PAD_SD2_DAT6__USDHC3_DAT6                                       \
		IOMUX_PAD(0x057C, 0x0274, 1, 0x0844, 1, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT6__UART2_CTS                                         \
		IOMUX_PAD(0x057C, 0x0274, 2, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT6__UART2_RTS                                         \
		IOMUX_PAD(0x057C, 0x0274, 2, 0x0800, 4, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT6__CSI_D_8                                           \
		IOMUX_PAD(0x057C, 0x0274, 3, 0x0650, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT6__USDHC2_WP                                         \
		IOMUX_PAD(0x057C, 0x0274, 4, 0x0834, 2, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT6__GPIO_4_29                                         \
		IOMUX_PAD(0x057C, 0x0274, 5, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT6__MMDC_MMDC_DEBUG_34                                \
		IOMUX_PAD(0x057C, 0x0274, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT6__PL301_SIM_MX6SL_PER1_HADDR_19                     \
		IOMUX_PAD(0x057C, 0x0274, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_SD2_DAT7__USDHC2_DAT7_50MHZ                                 \
		IOMUX_PAD(0x0580, 0x0278, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT7__USDHC2_DAT7_100MHZ                                \
	IOMUX_PAD(0x0580, 0x0278, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_100MHZ)
#define MX6SL_PAD_SD2_DAT7__USDHC2_DAT7_200MHZ                                \
	IOMUX_PAD(0x0580, 0x0278, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_200MHZ)
#define MX6SL_PAD_SD2_DAT7__USDHC3_DAT7                                       \
		IOMUX_PAD(0x0580, 0x0278, 1, 0x0848, 1, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT7__UART2_CTS                                         \
		IOMUX_PAD(0x0580, 0x0278, 2, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT7__UART2_RTS                                         \
		IOMUX_PAD(0x0580, 0x0278, 2, 0x0800, 5, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT7__CSI_D_9                                           \
		IOMUX_PAD(0x0580, 0x0278, 3, 0x0654, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT7__USDHC2_CD                                         \
		IOMUX_PAD(0x0580, 0x0278, 4, 0x0830, 2, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT7__GPIO_5_0                                          \
		IOMUX_PAD(0x0580, 0x0278, 5, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT7__MMDC_MMDC_DEBUG_33                                \
		IOMUX_PAD(0x0580, 0x0278, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_DAT7__PL301_SIM_MX6SL_PER1_HADDR_16                     \
		IOMUX_PAD(0x0580, 0x0278, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_SD2_RST__USDHC2_RST                                         \
		IOMUX_PAD(0x0584, 0x027C, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD2_RST__FEC_REF_OUT                                        \
		IOMUX_PAD(0x0584, 0x027C, 1, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_RST__WDOG2_WDOG_B                                       \
		IOMUX_PAD(0x0584, 0x027C, 2, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_RST__SPDIF_OUT1                                         \
		IOMUX_PAD(0x0584, 0x027C, 3, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_RST__CSI_MCLK                                           \
		IOMUX_PAD(0x0584, 0x027C, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_RST__GPIO_4_27                                          \
		IOMUX_PAD(0x0584, 0x027C, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_RST__ANATOP_ANATOP_TESTO_12                             \
		IOMUX_PAD(0x0584, 0x027C, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD2_RST__PL301_SIM_MX6SL_PER1_HBURST_2                      \
		IOMUX_PAD(0x0584, 0x027C, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_SD3_CLK__USDHC3_CLK_50MHZ                                   \
		IOMUX_PAD(0x0588, 0x0280, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD3_CLK__USDHC3_CLK_100MHZ                                  \
	IOMUX_PAD(0x0588, 0x0280, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_100MHZ)
#define MX6SL_PAD_SD3_CLK__USDHC3_CLK_200MHZ                                  \
	IOMUX_PAD(0x0588, 0x0280, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_200MHZ)
#define MX6SL_PAD_SD3_CLK__AUDMUX_AUD5_RXFS                                   \
		IOMUX_PAD(0x0588, 0x0280, 1, 0x0608, 1, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_CLK__KPP_COL_5                                          \
		IOMUX_PAD(0x0588, 0x0280, 2, 0x0748, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_CLK__CSI_D_10                                           \
		IOMUX_PAD(0x0588, 0x0280, 3, 0x0658, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_CLK__WDOG1_WDOG_RST_B_DEB                               \
		IOMUX_PAD(0x0588, 0x0280, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_CLK__GPIO_5_18                                          \
		IOMUX_PAD(0x0588, 0x0280, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_CLK__USB_USBOTG1_PWR                                    \
		IOMUX_PAD(0x0588, 0x0280, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_CLK__PL301_SIM_MX6SL_PER1_HADDR_13                      \
		IOMUX_PAD(0x0588, 0x0280, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_SD3_CMD__USDHC3_CMD_50MHZ                                   \
		IOMUX_PAD(0x058C, 0x0284, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD3_CMD__USDHC3_CMD_100MHZ                                  \
	IOMUX_PAD(0x058C, 0x0284, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_100MHZ)
#define MX6SL_PAD_SD3_CMD__USDHC3_CMD_200MHZ                                  \
	IOMUX_PAD(0x058C, 0x0284, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_200MHZ)
#define MX6SL_PAD_SD3_CMD__AUDMUX_AUD5_RXC                                    \
		IOMUX_PAD(0x058C, 0x0284, 1, 0x0604, 1, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_CMD__KPP_ROW_5                                          \
		IOMUX_PAD(0x058C, 0x0284, 2, 0x0768, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_CMD__CSI_D_11                                           \
		IOMUX_PAD(0x058C, 0x0284, 3, 0x065C, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_CMD__ANATOP_USBOTG2_ID                                  \
		IOMUX_PAD(0x058C, 0x0284, 4, 0x05E0, 3, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_CMD__GPIO_5_21                                          \
		IOMUX_PAD(0x058C, 0x0284, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_CMD__USB_USBOTG2_PWR                                    \
		IOMUX_PAD(0x058C, 0x0284, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_CMD__PL301_SIM_MX6SL_PER1_HADDR_18                      \
		IOMUX_PAD(0x058C, 0x0284, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_SD3_DAT0__USDHC3_DAT0_50MHZ                                 \
		IOMUX_PAD(0x0590, 0x0288, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD3_DAT0__USDHC3_DAT0_100MHZ                                \
	IOMUX_PAD(0x0590, 0x0288, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_100MHZ)
#define MX6SL_PAD_SD3_DAT0__USDHC3_DAT0_200MHZ                                \
	IOMUX_PAD(0x0590, 0x0288, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_200MHZ)
#define MX6SL_PAD_SD3_DAT0__AUDMUX_AUD5_RXD                                   \
		IOMUX_PAD(0x0590, 0x0288, 1, 0x05FC, 1, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_DAT0__KPP_COL_6                                         \
		IOMUX_PAD(0x0590, 0x0288, 2, 0x074C, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_DAT0__CSI_D_12                                          \
		IOMUX_PAD(0x0590, 0x0288, 3, 0x0660, 1, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_DAT0__ANATOP_USBOTG1_ID                                 \
		IOMUX_PAD(0x0590, 0x0288, 4, 0x05DC, 4, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_DAT0__GPIO_5_19                                         \
		IOMUX_PAD(0x0590, 0x0288, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_DAT0__SJC_JTAG_ACT                                      \
		IOMUX_PAD(0x0590, 0x0288, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_DAT0__PL301_SIM_MX6SL_PER1_HADDR_11                     \
		IOMUX_PAD(0x0590, 0x0288, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_SD3_DAT1__USDHC3_DAT1_50MHZ                                 \
		IOMUX_PAD(0x0594, 0x028C, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD3_DAT1__USDHC3_DAT1_100MHZ                                \
	IOMUX_PAD(0x0594, 0x028C, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_100MHZ)
#define MX6SL_PAD_SD3_DAT1__USDHC3_DAT1_200MHZ                                \
	IOMUX_PAD(0x0594, 0x028C, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_200MHZ)
#define MX6SL_PAD_SD3_DAT1__AUDMUX_AUD5_TXC                                   \
		IOMUX_PAD(0x0594, 0x028C, 1, 0x060C, 1, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_DAT1__KPP_ROW_6                                         \
		IOMUX_PAD(0x0594, 0x028C, 2, 0x076C, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_DAT1__CSI_D_13                                          \
		IOMUX_PAD(0x0594, 0x028C, 3, 0x0664, 1, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_DAT1__USDHC1_VSELECT                                    \
		IOMUX_PAD(0x0594, 0x028C, 4, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD3_DAT1__GPIO_5_20                                         \
		IOMUX_PAD(0x0594, 0x028C, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_DAT1__SJC_DE_B                                          \
		IOMUX_PAD(0x0594, 0x028C, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_DAT1__PL301_SIM_MX6SL_PER1_HADDR_17                     \
		IOMUX_PAD(0x0594, 0x028C, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_SD3_DAT2__USDHC3_DAT2_50MHZ                                 \
		IOMUX_PAD(0x0598, 0x0290, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD3_DAT2__USDHC3_DAT2_100MHZ                                \
	IOMUX_PAD(0x0598, 0x0290, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_100MHZ)
#define MX6SL_PAD_SD3_DAT2__USDHC3_DAT2_200MHZ                                \
	IOMUX_PAD(0x0598, 0x0290, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_200MHZ)
#define MX6SL_PAD_SD3_DAT2__AUDMUX_AUD5_TXFS                                  \
		IOMUX_PAD(0x0598, 0x0290, 1, 0x0610, 1, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_DAT2__KPP_COL_7                                         \
		IOMUX_PAD(0x0598, 0x0290, 2, 0x0750, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_DAT2__CSI_D_14                                          \
		IOMUX_PAD(0x0598, 0x0290, 3, 0x0668, 1, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_DAT2__EPIT1_EPITO                                       \
		IOMUX_PAD(0x0598, 0x0290, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_DAT2__GPIO_5_16                                         \
		IOMUX_PAD(0x0598, 0x0290, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_DAT2__USB_USBOTG2_OC                                    \
		IOMUX_PAD(0x0598, 0x0290, 6, 0x0820, 3, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_DAT2__PL301_SIM_MX6SL_PER1_HADDR_14                     \
		IOMUX_PAD(0x0598, 0x0290, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_SD3_DAT3__USDHC3_DAT3_50MHZ                                 \
		IOMUX_PAD(0x059C, 0x0294, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL)
#define MX6SL_PAD_SD3_DAT3__USDHC3_DAT3_100MHZ                                \
	IOMUX_PAD(0x059C, 0x0294, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_100MHZ)
#define MX6SL_PAD_SD3_DAT3__USDHC3_DAT3_200MHZ                                \
	IOMUX_PAD(0x059C, 0x0294, 0, 0x0000, 0, MX6SL_USDHC_PAD_CTRL_200MHZ)
#define MX6SL_PAD_SD3_DAT3__AUDMUX_AUD5_TXD                                   \
		IOMUX_PAD(0x059C, 0x0294, 1, 0x0600, 1, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_DAT3__KPP_ROW_7                                         \
		IOMUX_PAD(0x059C, 0x0294, 2, 0x0770, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_DAT3__CSI_D_15                                          \
		IOMUX_PAD(0x059C, 0x0294, 3, 0x066C, 1, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_DAT3__EPIT2_EPITO                                       \
		IOMUX_PAD(0x059C, 0x0294, 4, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_DAT3__GPIO_5_17                                         \
		IOMUX_PAD(0x059C, 0x0294, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_DAT3__USB_USBOTG1_OC                                    \
		IOMUX_PAD(0x059C, 0x0294, 6, 0x0824, 2, NO_PAD_CTRL)
#define MX6SL_PAD_SD3_DAT3__PL301_SIM_MX6SL_PER1_HADDR_12                     \
		IOMUX_PAD(0x059C, 0x0294, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_TAMPER__SNVS_LP_WRAPPER_SNVS_TD1                            \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_TEST_MODE__TCU_TEST_MODE                                    \
		IOMUX_PAD(NO_PAD_I, NO_MUX_I, 0, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_UART1_RXD__UART1_TXD                                        \
		IOMUX_PAD(0x05A0, 0x0298, 0, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_UART1_RXD__UART1_RXD                                        \
		IOMUX_PAD(0x05A0, 0x0298, 0, 0x07FC, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_UART1_RXD__PWM1_PWMO                                        \
		IOMUX_PAD(0x05A0, 0x0298, 1, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_UART1_RXD__UART4_TXD                                        \
		IOMUX_PAD(0x05A0, 0x0298, 2, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_UART1_RXD__UART4_RXD                                        \
		IOMUX_PAD(0x05A0, 0x0298, 2, 0x0814, 6, NO_PAD_CTRL)
#define MX6SL_PAD_UART1_RXD__FEC_COL                                          \
		IOMUX_PAD(0x05A0, 0x0298, 3, 0x06F0, 2, NO_PAD_CTRL)
#define MX6SL_PAD_UART1_RXD__UART5_TXD                                        \
		IOMUX_PAD(0x05A0, 0x0298, 4, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_UART1_RXD__UART5_RXD                                        \
		IOMUX_PAD(0x05A0, 0x0298, 4, 0x081C, 6, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_UART1_RXD__GPIO_3_16                                        \
		IOMUX_PAD(0x05A0, 0x0298, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_UART1_RXD__ANATOP_ANATOP_TESTI_2                            \
		IOMUX_PAD(0x05A0, 0x0298, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_UART1_RXD__TPSMP_CLK                                        \
		IOMUX_PAD(0x05A0, 0x0298, 7, 0x0000, 0, NO_PAD_CTRL)

#define MX6SL_PAD_UART1_TXD__UART1_TXD                                        \
		IOMUX_PAD(0x05A4, 0x029C, 0, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_UART1_TXD__UART1_RXD                                        \
		IOMUX_PAD(0x05A4, 0x029C, 0, 0x07FC, 1, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_UART1_TXD__PWM2_PWMO                                        \
		IOMUX_PAD(0x05A4, 0x029C, 1, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_UART1_TXD__UART4_TXD                                        \
		IOMUX_PAD(0x05A4, 0x029C, 2, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_UART1_TXD__UART4_RXD                                        \
		IOMUX_PAD(0x05A4, 0x029C, 2, 0x0814, 7, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_UART1_TXD__FEC_RX_CLK                                       \
		IOMUX_PAD(0x05A4, 0x029C, 3, 0x0700, 2, NO_PAD_CTRL)
#define MX6SL_PAD_UART1_TXD__UART5_TXD                                        \
		IOMUX_PAD(0x05A4, 0x029C, 4, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_UART1_TXD__UART5_RXD                                        \
		IOMUX_PAD(0x05A4, 0x029C, 4, 0x081C, 7, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_UART1_TXD__GPIO_3_17                                        \
		IOMUX_PAD(0x05A4, 0x029C, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_UART1_TXD__ANATOP_ANATOP_TESTI_3                            \
		IOMUX_PAD(0x05A4, 0x029C, 6, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_UART1_TXD__UART5_DCD                                        \
		IOMUX_PAD(0x05A4, 0x029C, 7, 0x0000, 0, MX6SL_UART_PAD_CTRL)

#define MX6SL_PAD_WDOG_B__WDOG1_WDOG_B                                        \
		IOMUX_PAD(0x05A8, 0x02A0, 0, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_WDOG_B__WDOG1_WDOG_RST_B_DEB                                \
		IOMUX_PAD(0x05A8, 0x02A0, 1, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_WDOG_B__UART5_RI                                            \
		IOMUX_PAD(0x05A8, 0x02A0, 2, 0x0000, 0, MX6SL_UART_PAD_CTRL)
#define MX6SL_PAD_WDOG_B__GPIO_3_18                                           \
		IOMUX_PAD(0x05A8, 0x02A0, 5, 0x0000, 0, NO_PAD_CTRL)
#define MX6SL_PAD_WDOG_B__OBSERVE_MUX_OUT_2                                   \
		IOMUX_PAD(0x05A8, 0x02A0, 7, 0x0000, 0, NO_PAD_CTRL)

#endif /* __MACH_IOMUX_MX6SL_H__*/
