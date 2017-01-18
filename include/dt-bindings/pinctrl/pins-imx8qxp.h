/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _SC_PINS_H
#define _SC_PINS_H

#define SC_P_ALL            UINT16_MAX

/*
 * @name Pin Definitions
 */
#define SC_P_PCIE_CTRL0_CLKREQ_B                 0
#define SC_P_PCIE_CTRL0_WAKE_B                   1
#define SC_P_PCIE_CTRL0_PERST_B                  2
#define SC_P_COMP_CTL_GPIO_1V8_3V3_PCIESEP       3
#define SC_P_USB_SS3_TC0                         4
#define SC_P_USB_SS3_TC1                         5
#define SC_P_USB_SS3_TC2                         6
#define SC_P_USB_SS3_TC3                         7
#define SC_P_COMP_CTL_GPIO_3V3_USB3IO            8
#define SC_P_EMMC0_CLK                           9
#define SC_P_EMMC0_CMD                           10
#define SC_P_EMMC0_DATA0                         11
#define SC_P_EMMC0_DATA1                         12
#define SC_P_EMMC0_DATA2                         13
#define SC_P_EMMC0_DATA3                         14
#define SC_P_EMMC0_DATA4                         15
#define SC_P_EMMC0_DATA5                         16
#define SC_P_EMMC0_DATA6                         17
#define SC_P_EMMC0_DATA7                         18
#define SC_P_EMMC0_STROBE                        19
#define SC_P_EMMC0_RESET_B                       20
#define SC_P_COMP_CTL_GPIO_1V8_3V3_SD1FIX        21
#define SC_P_USDHC1_RESET_B                      22
#define SC_P_USDHC1_VSELECT                      23
#define SC_P_USDHC1_WP                           24
#define SC_P_USDHC1_CD_B                         25
#define SC_P_COMP_CTL_GPIO_1V8_3V3_VSELSEP       26
#define SC_P_USDHC1_CLK                          27
#define SC_P_USDHC1_CMD                          28
#define SC_P_USDHC1_DATA0                        29
#define SC_P_USDHC1_DATA1                        30
#define SC_P_USDHC1_DATA2                        31
#define SC_P_USDHC1_DATA3                        32
#define SC_P_COMP_CTL_GPIO_1V8_3V3_VSEL3         33
#define SC_P_ENET0_RGMII_TXC                     34
#define SC_P_ENET0_RGMII_TX_CTL                  35
#define SC_P_ENET0_RGMII_TXD0                    36
#define SC_P_ENET0_RGMII_TXD1                    37
#define SC_P_ENET0_RGMII_TXD2                    38
#define SC_P_ENET0_RGMII_TXD3                    39
#define SC_P_ENET0_RGMII_RXC                     40
#define SC_P_ENET0_RGMII_RX_CTL                  41
#define SC_P_ENET0_RGMII_RXD0                    42
#define SC_P_ENET0_RGMII_RXD1                    43
#define SC_P_ENET0_RGMII_RXD2                    44
#define SC_P_ENET0_RGMII_RXD3                    45
#define SC_P_COMP_CTL_GPIO_1V8_3V3_ENET_ENETB    46
#define SC_P_ENET0_REFCLK_125M_25M               47
#define SC_P_ENET0_MDIO                          48
#define SC_P_ENET0_MDC                           49
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIOCT        50
#define SC_P_FLEXCAN0_RX                         51
#define SC_P_FLEXCAN0_TX                         52
#define SC_P_FLEXCAN1_RX                         53
#define SC_P_FLEXCAN1_TX                         54
#define SC_P_UART0_RX                            55
#define SC_P_UART0_TX                            56
#define SC_P_UART0_RTS_B                         57
#define SC_P_UART0_CTS_B                         58
#define SC_P_UART1_TX                            59
#define SC_P_UART1_RX                            60
#define SC_P_UART1_RTS_B                         61
#define SC_P_UART1_CTS_B                         62
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIOLH        63
#define SC_P_SPI0_SCK                            64
#define SC_P_SPI0_SDO                            65
#define SC_P_SPI0_SDI                            66
#define SC_P_SPI0_CS0                            67
#define SC_P_SPI0_CS1                            68
#define SC_P_SPI2_SCK                            69
#define SC_P_SPI2_SDO                            70
#define SC_P_SPI2_SDI                            71
#define SC_P_SPI2_CS0                            72
#define SC_P_SPI2_CS1                            73
#define SC_P_SAI1_RXC                            74
#define SC_P_SAI1_RXD                            75
#define SC_P_SAI1_RXFS                           76
#define SC_P_SAI1_TXC                            77
#define SC_P_SAI1_TXD                            78
#define SC_P_SAI1_TXFS                           79
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIORHT       80
#define SC_P_ESAI0_FSR                           81
#define SC_P_ESAI0_FST                           82
#define SC_P_ESAI0_SCKR                          83
#define SC_P_ESAI0_SCKT                          84
#define SC_P_ESAI0_TX0                           85
#define SC_P_ESAI0_TX1                           86
#define SC_P_ESAI0_TX2_RX3                       87
#define SC_P_ESAI0_TX3_RX2                       88
#define SC_P_ESAI0_TX4_RX1                       89
#define SC_P_ESAI0_TX5_RX0                       90
#define SC_P_SPDIF0_RX                           91
#define SC_P_SPDIF0_TX                           92
#define SC_P_SPDIF0_EXT_CLK                      93
#define SC_P_SPI3_SCK                            94
#define SC_P_SPI3_SDO                            95
#define SC_P_SPI3_SDI                            96
#define SC_P_SPI3_CS0                            97
#define SC_P_SPI3_CS1                            98
#define SC_P_MCLK_IN0                            99
#define SC_P_MCLK_OUT0                           100
#define SC_P_FTM0                                101
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIORHB       102
#define SC_P_ADC_IN1                             103
#define SC_P_ADC_IN0                             104
#define SC_P_ADC_IN3                             105
#define SC_P_ADC_IN2                             106
#define SC_P_CSI_D00                             107
#define SC_P_CSI_D01                             108
#define SC_P_CSI_D02                             109
#define SC_P_CSI_D03                             110
#define SC_P_CSI_D04                             111
#define SC_P_CSI_D05                             112
#define SC_P_CSI_D06                             113
#define SC_P_CSI_D07                             114
#define SC_P_CSI_HSYNC                           115
#define SC_P_CSI_VSYNC                           116
#define SC_P_CSI_PCLK                            117
#define SC_P_CSI_MCLK                            118
#define SC_P_CSI_EN                              119
#define SC_P_CSI_RESET                           120
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIORHD       121
#define SC_P_PMIC_I2C_SCL                        122
#define SC_P_PMIC_I2C_SDA                        123
#define SC_P_PMIC_INT_B                          124
#define SC_P_SCU_GPIO0_00                        125
#define SC_P_SCU_GPIO0_01                        126
#define SC_P_SCU_BOOT_MODE0                      127
#define SC_P_SCU_BOOT_MODE1                      128
#define SC_P_SCU_BOOT_MODE2                      129
#define SC_P_SCU_BOOT_MODE3                      130
#define SC_P_MIPI_DSI0_I2C0_SCL                  131
#define SC_P_MIPI_DSI0_I2C0_SDA                  132
#define SC_P_MIPI_DSI0_GPIO0_00                  133
#define SC_P_MIPI_DSI0_GPIO0_01                  134
#define SC_P_MIPI_DSI1_I2C0_SCL                  135
#define SC_P_MIPI_DSI1_I2C0_SDA                  136
#define SC_P_MIPI_DSI1_GPIO0_00                  137
#define SC_P_MIPI_DSI1_GPIO0_01                  138
#define SC_P_COMP_CTL_GPIO_1V8_3V3_MIPIDSIGPIO   139
#define SC_P_MIPI_CSI0_MCLK_OUT                  140
#define SC_P_MIPI_CSI0_I2C0_SCL                  141
#define SC_P_MIPI_CSI0_I2C0_SDA                  142
#define SC_P_MIPI_CSI0_GPIO0_00                  143
#define SC_P_MIPI_CSI0_GPIO0_01                  144
#define SC_P_QSPI0A_DATA0                        145
#define SC_P_QSPI0A_DATA1                        146
#define SC_P_QSPI0A_DATA2                        147
#define SC_P_QSPI0A_DATA3                        148
#define SC_P_QSPI0A_DQS                          149
#define SC_P_QSPI0A_SS0_B                        150
#define SC_P_QSPI0A_SS1_B                        151
#define SC_P_QSPI0A_SCLK                         152
#define SC_P_QSPI0B_SCLK                         153
#define SC_P_QSPI0B_DATA0                        154
#define SC_P_QSPI0B_DATA1                        155
#define SC_P_QSPI0B_DATA2                        156
#define SC_P_QSPI0B_DATA3                        157
#define SC_P_QSPI0B_DQS                          158
#define SC_P_QSPI0B_SS0_B                        159
#define SC_P_QSPI0B_SS1_B                        160
#define SC_P_COMP_CTL_GPIO_1V8_3V3_QSPI0         161
#define SC_P_XTALI                               162
#define SC_P_XTALO                               163
#define SC_P_ANA_TEST_OUT_P                      164
#define SC_P_ANA_TEST_OUT_N                      165
#define SC_P_RTC_XTALI                           166
#define SC_P_RTC_XTALO                           167
#define SC_P_PMIC_ON_REQ                         168
#define SC_P_ON_OFF_BUTTON                       169

#endif /* _SC_PINS_H */
