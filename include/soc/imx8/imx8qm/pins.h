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

/*!
 * Header file used to configure SoC pin list.
 */

#ifndef _SC_PINS_H
#define _SC_PINS_H

/* Includes */

#include <soc/imx8/sc/scfw.h>

/* Defines */

#define SC_P_ALL            UINT16_MAX      //!< All pins

/*!
 * @name Pin Definitions
 */
/*@{*/
#define SC_P_SIM0_CLK                            0    //!< DMA.SIM0.CLK, LSIO.GPIO0.IO00
#define SC_P_SIM0_RST                            1    //!< DMA.SIM0.RST, LSIO.GPIO0.IO01
#define SC_P_SIM0_IO                             2    //!< DMA.SIM0.IO, LSIO.GPIO0.IO02
#define SC_P_SIM0_PD                             3    //!< DMA.SIM0.PD, DMA.I2C3.SCL, LSIO.GPIO0.IO03
#define SC_P_SIM0_POWER_EN                       4    //!< DMA.SIM0.POWER_EN, DMA.I2C3.SDA, LSIO.GPIO0.IO04
#define SC_P_SIM0_GPIO0_00                       5    //!< DMA.SIM0.POWER_EN, LSIO.GPIO0.IO05
#define SC_P_COMP_CTL_GPIO_1V8_3V3_SIM           6    //!<
#define SC_P_M40_I2C0_SCL                        7    //!< M40.I2C0.SCL, M40.UART0.RX, M40.GPIO0.IO02, LSIO.GPIO0.IO06
#define SC_P_M40_I2C0_SDA                        8    //!< M40.I2C0.SDA, M40.UART0.TX, M40.GPIO0.IO03, LSIO.GPIO0.IO07
#define SC_P_M40_GPIO0_00                        9    //!< M40.GPIO0.IO00, M40.TPM0.CH0, DMA.UART4.RX, LSIO.GPIO0.IO08
#define SC_P_M40_GPIO0_01                        10   //!< M40.GPIO0.IO01, M40.TPM0.CH1, DMA.UART4.TX, LSIO.GPIO0.IO09
#define SC_P_M41_I2C0_SCL                        11   //!< M41.I2C0.SCL, M41.UART0.RX, M41.GPIO0.IO02, LSIO.GPIO0.IO10
#define SC_P_M41_I2C0_SDA                        12   //!< M41.I2C0.SDA, M41.UART0.TX, M41.GPIO0.IO03, LSIO.GPIO0.IO11
#define SC_P_M41_GPIO0_00                        13   //!< M41.GPIO0.IO00, M41.TPM0.CH0, DMA.UART3.RX, LSIO.GPIO0.IO12
#define SC_P_M41_GPIO0_01                        14   //!< M41.GPIO0.IO01, M41.TPM0.CH1, DMA.UART3.TX, LSIO.GPIO0.IO13
#define SC_P_GPT0_CLK                            15   //!< LSIO.GPT0.CLK, DMA.I2C1.SCL, LSIO.KPP0.COL4, LSIO.GPIO0.IO14
#define SC_P_GPT0_CAPTURE                        16   //!< LSIO.GPT0.CAPTURE, DMA.I2C1.SDA, LSIO.KPP0.COL5, LSIO.GPIO0.IO15
#define SC_P_GPT0_COMPARE                        17   //!< LSIO.GPT0.COMPARE, LSIO.PWM3.OUT, LSIO.KPP0.COL6, LSIO.GPIO0.IO16
#define SC_P_GPT1_CLK                            18   //!< LSIO.GPT1.CLK, DMA.I2C2.SCL, LSIO.KPP0.COL7, LSIO.GPIO0.IO17
#define SC_P_GPT1_CAPTURE                        19   //!< LSIO.GPT1.CAPTURE, DMA.I2C2.SDA, LSIO.KPP0.ROW4, LSIO.GPIO0.IO18
#define SC_P_GPT1_COMPARE                        20   //!< LSIO.GPT1.COMPARE, LSIO.PWM2.OUT, LSIO.KPP0.ROW5, LSIO.GPIO0.IO19
#define SC_P_UART0_RX                            21   //!< DMA.UART0.RX, LSIO.GPIO0.IO20
#define SC_P_UART0_TX                            22   //!< DMA.UART0.TX, LSIO.GPIO0.IO21
#define SC_P_UART0_RTS_B                         23   //!< DMA.UART0.RTS_B, LSIO.PWM0.OUT, DMA.UART2.RX, LSIO.GPIO0.IO22
#define SC_P_UART0_CTS_B                         24   //!< DMA.UART0.CTS_B, LSIO.PWM1.OUT, DMA.UART2.TX, LSIO.GPIO0.IO23
#define SC_P_UART1_TX                            25   //!< DMA.UART1.TX, DMA.SPI3.SCK, LSIO.GPIO0.IO24
#define SC_P_UART1_RX                            26   //!< DMA.UART1.RX, DMA.SPI3.SDO, LSIO.GPIO0.IO25
#define SC_P_UART1_RTS_B                         27   //!< DMA.UART1.RTS_B, DMA.SPI3.SDI, DMA.UART1.CTS_B, LSIO.GPIO0.IO26
#define SC_P_UART1_CTS_B                         28   //!< DMA.UART1.CTS_B, DMA.SPI3.CS0, DMA.UART1.RTS_B, LSIO.GPIO0.IO27
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIOLH        29   //!<
#define SC_P_SCU_PMIC_MEMC_ON                    30   //!< SCU.GPIO0.IOXX_PMIC_MEMC_ON
#define SC_P_SCU_WDOG_OUT                        31   //!< SCU.WDOG0.WDOG_OUT
#define SC_P_JTAG_TMS                            32   //!< SCU.JTAG.TMS
#define SC_P_JTAG_TCK                            33   //!< SCU.JTAG.TCK
#define SC_P_JTAG_TDO                            34   //!< SCU.JTAG.TDO
#define SC_P_JTAG_TDI                            35   //!< SCU.JTAG.TDI
#define SC_P_JTAG_TRST_B                         36   //!< SCU.JTAG.TRST_B
#define SC_P_TEST_MODE_SELECT                    37   //!< SCU.TCU.TEST_MODE_SELECT
#define SC_P_SCU_PMIC_STANDBY                    38   //!< SCU.DSC.PMIC_STANDBY
#define SC_P_PMIC_I2C_SDA                        39   //!< SCU.PMIC_I2C.SDA
#define SC_P_PMIC_I2C_SCL                        40   //!< SCU.PMIC_I2C.SCL
#define SC_P_PMIC_EARLY_WARNING                  41   //!< SCU.PMIC_EARLY_WARNING
#define SC_P_POR_B                               42   //!< SCU.DSC.POR_B
#define SC_P_PMIC_INT_B                          43   //!< SCU.DSC.PMIC_INT_B
#define SC_P_SCU_GPIO0_00                        44   //!< SCU.GPIO0.IO00, SCU.UART0.RX, LSIO.GPIO0.IO28
#define SC_P_SCU_GPIO0_01                        45   //!< SCU.GPIO0.IO01, SCU.UART0.TX, LSIO.GPIO0.IO29
#define SC_P_SCU_GPIO0_02                        46   //!< SCU.GPIO0.IO02, SCU.GPIO0.IOXX_PMIC_GPU0_ON, LSIO.GPIO0.IO30
#define SC_P_SCU_GPIO0_03                        47   //!< SCU.GPIO0.IO03, SCU.GPIO0.IOXX_PMIC_GPU1_ON, LSIO.GPIO0.IO31
#define SC_P_SCU_GPIO0_04                        48   //!< SCU.GPIO0.IO04, SCU.GPIO0.IOXX_PMIC_A72_ON, LSIO.GPIO1.IO00
#define SC_P_SCU_GPIO0_05                        49   //!< SCU.GPIO0.IO05, SCU.GPIO0.IOXX_PMIC_A53_ON, LSIO.GPIO1.IO01
#define SC_P_SCU_GPIO0_06                        50   //!< SCU.GPIO0.IO06, SCU.TPM0.CH0, LSIO.GPIO1.IO02
#define SC_P_SCU_GPIO0_07                        51   //!< SCU.GPIO0.IO07, SCU.TPM0.CH1, SCU.DSC.RTC_CLOCK_OUTPUT_32K, LSIO.GPIO1.IO03
#define SC_P_SCU_BOOT_MODE0                      52   //!< SCU.DSC.BOOT_MODE0
#define SC_P_SCU_BOOT_MODE1                      53   //!< SCU.DSC.BOOT_MODE1
#define SC_P_SCU_BOOT_MODE2                      54   //!< SCU.DSC.BOOT_MODE2
#define SC_P_SCU_BOOT_MODE3                      55   //!< SCU.DSC.BOOT_MODE3
#define SC_P_SCU_BOOT_MODE4                      56   //!< SCU.DSC.BOOT_MODE4, SCU.PMIC_I2C.SCL
#define SC_P_SCU_BOOT_MODE5                      57   //!< SCU.DSC.BOOT_MODE5, SCU.PMIC_I2C.SDA
#define SC_P_LVDS0_GPIO00                        58   //!< LVDS0.GPIO0.IO00, LVDS0.PWM0.OUT, LSIO.GPIO1.IO04
#define SC_P_LVDS0_GPIO01                        59   //!< LVDS0.GPIO0.IO01, LSIO.GPIO1.IO05
#define SC_P_LVDS0_I2C0_SCL                      60   //!< LVDS0.I2C0.SCL, LVDS0.GPIO0.IO02, LSIO.GPIO1.IO06
#define SC_P_LVDS0_I2C0_SDA                      61   //!< LVDS0.I2C0.SDA, LVDS0.GPIO0.IO03, LSIO.GPIO1.IO07
#define SC_P_LVDS0_I2C1_SCL                      62   //!< LVDS0.I2C1.SCL, DMA.UART2.TX, LSIO.GPIO1.IO08
#define SC_P_LVDS0_I2C1_SDA                      63   //!< LVDS0.I2C1.SDA, DMA.UART2.RX, LSIO.GPIO1.IO09
#define SC_P_LVDS1_GPIO00                        64   //!< LVDS1.GPIO0.IO00, LVDS1.PWM0.OUT, LSIO.GPIO1.IO10
#define SC_P_LVDS1_GPIO01                        65   //!< LVDS1.GPIO0.IO01, LSIO.GPIO1.IO11
#define SC_P_LVDS1_I2C0_SCL                      66   //!< LVDS1.I2C0.SCL, LVDS1.GPIO0.IO02, LSIO.GPIO1.IO12
#define SC_P_LVDS1_I2C0_SDA                      67   //!< LVDS1.I2C0.SDA, LVDS1.GPIO0.IO03, LSIO.GPIO1.IO13
#define SC_P_LVDS1_I2C1_SCL                      68   //!< LVDS1.I2C1.SCL, DMA.UART3.TX, LSIO.GPIO1.IO14
#define SC_P_LVDS1_I2C1_SDA                      69   //!< LVDS1.I2C1.SDA, DMA.UART3.RX, LSIO.GPIO1.IO15
#define SC_P_COMP_CTL_GPIO_1V8_3V3_LVDSGPIO      70   //!<
#define SC_P_MIPI_DSI0_I2C0_SCL                  71   //!< MIPI_DSI0.I2C0.SCL, LSIO.GPIO1.IO16
#define SC_P_MIPI_DSI0_I2C0_SDA                  72   //!< MIPI_DSI0.I2C0.SDA, LSIO.GPIO1.IO17
#define SC_P_MIPI_DSI0_GPIO0_00                  73   //!< MIPI_DSI0.GPIO0.IO00, MIPI_DSI0.PWM0.OUT, LSIO.GPIO1.IO18
#define SC_P_MIPI_DSI0_GPIO0_01                  74   //!< MIPI_DSI0.GPIO0.IO01, LSIO.GPIO1.IO19
#define SC_P_MIPI_DSI1_I2C0_SCL                  75   //!< MIPI_DSI1.I2C0.SCL, LSIO.GPIO1.IO20
#define SC_P_MIPI_DSI1_I2C0_SDA                  76   //!< MIPI_DSI1.I2C0.SDA, LSIO.GPIO1.IO21
#define SC_P_MIPI_DSI1_GPIO0_00                  77   //!< MIPI_DSI1.GPIO0.IO00, MIPI_DSI1.PWM0.OUT, LSIO.GPIO1.IO22
#define SC_P_MIPI_DSI1_GPIO0_01                  78   //!< MIPI_DSI1.GPIO0.IO01, LSIO.GPIO1.IO23
#define SC_P_COMP_CTL_GPIO_1V8_3V3_MIPIDSIGPIO   79   //!<
#define SC_P_MIPI_CSI0_MCLK_OUT                  80   //!< MIPI_CSI0.ACM.MCLK_OUT, LSIO.GPIO1.IO24
#define SC_P_MIPI_CSI0_I2C0_SCL                  81   //!< MIPI_CSI0.I2C0.SCL, LSIO.GPIO1.IO25
#define SC_P_MIPI_CSI0_I2C0_SDA                  82   //!< MIPI_CSI0.I2C0.SDA, LSIO.GPIO1.IO26
#define SC_P_MIPI_CSI0_GPIO0_00                  83   //!< MIPI_CSI0.GPIO0.IO00, DMA.I2C0.SCL, LSIO.GPIO1.IO27
#define SC_P_MIPI_CSI0_GPIO0_01                  84   //!< MIPI_CSI0.GPIO0.IO01, DMA.I2C0.SDA, LSIO.GPIO1.IO28
#define SC_P_MIPI_CSI1_MCLK_OUT                  85   //!< MIPI_CSI1.ACM.MCLK_OUT, LSIO.GPIO1.IO29
#define SC_P_MIPI_CSI1_GPIO0_00                  86   //!< MIPI_CSI1.GPIO0.IO00, DMA.UART4.RX, LSIO.GPIO1.IO30
#define SC_P_MIPI_CSI1_GPIO0_01                  87   //!< MIPI_CSI1.GPIO0.IO01, DMA.UART4.TX, LSIO.GPIO1.IO31
#define SC_P_MIPI_CSI1_I2C0_SCL                  88   //!< MIPI_CSI1.I2C0.SCL, LSIO.GPIO2.IO00
#define SC_P_MIPI_CSI1_I2C0_SDA                  89   //!< MIPI_CSI1.I2C0.SDA, LSIO.GPIO2.IO01
#define SC_P_HDMI_TX0_TS_SCL                     90   //!< HDMI_TX0.I2C0.SCL, DMA.I2C0.SCL, LSIO.GPIO2.IO02
#define SC_P_HDMI_TX0_TS_SDA                     91   //!< HDMI_TX0.I2C0.SDA, DMA.I2C0.SDA, LSIO.GPIO2.IO03
#define SC_P_COMP_CTL_GPIO_3V3_HDMIGPIO          92   //!<
#define SC_P_ESAI1_FSR                           93   //!< AUD.ESAI1.FSR, LSIO.GPIO2.IO04
#define SC_P_ESAI1_FST                           94   //!< AUD.ESAI1.FST, LSIO.GPIO2.IO05
#define SC_P_ESAI1_SCKR                          95   //!< AUD.ESAI1.SCKR, LSIO.GPIO2.IO06
#define SC_P_ESAI1_SCKT                          96   //!< AUD.ESAI1.SCKT, AUD.SAI2.RXC, LSIO.GPIO2.IO07
#define SC_P_ESAI1_TX0                           97   //!< AUD.ESAI1.TX0, AUD.SAI2.RXD, LSIO.GPIO2.IO08
#define SC_P_ESAI1_TX1                           98   //!< AUD.ESAI1.TX1, AUD.SAI2.RXFS, LSIO.GPIO2.IO09
#define SC_P_ESAI1_TX2_RX3                       99   //!< AUD.ESAI1.TX2_RX3, LSIO.GPIO2.IO10
#define SC_P_ESAI1_TX3_RX2                       100  //!< AUD.ESAI1.TX3_RX2, LSIO.GPIO2.IO11
#define SC_P_ESAI1_TX4_RX1                       101  //!< AUD.ESAI1.TX4_RX1, LSIO.GPIO2.IO12
#define SC_P_ESAI1_TX5_RX0                       102  //!< AUD.ESAI1.TX5_RX0, LSIO.GPIO2.IO13
#define SC_P_SPDIF0_RX                           103  //!< AUD.SPDIF0.RX, AUD.MQS.R, AUD.ACM.MCLK_IN1, LSIO.GPIO2.IO14
#define SC_P_SPDIF0_TX                           104  //!< AUD.SPDIF0.TX, AUD.MQS.L, AUD.ACM.MCLK_OUT1, LSIO.GPIO2.IO15
#define SC_P_SPDIF0_EXT_CLK                      105  //!< AUD.SPDIF0.EXT_CLK, DMA.DMA0.REQ_IN0, LSIO.GPIO2.IO16
#define SC_P_SPI3_SCK                            106  //!< DMA.SPI3.SCK, LSIO.GPIO2.IO17
#define SC_P_SPI3_SDO                            107  //!< DMA.SPI3.SDO, DMA.FTM.CH0, LSIO.GPIO2.IO18
#define SC_P_SPI3_SDI                            108  //!< DMA.SPI3.SDI, DMA.FTM.CH1, LSIO.GPIO2.IO19
#define SC_P_SPI3_CS0                            109  //!< DMA.SPI3.CS0, DMA.FTM.CH2, LSIO.GPIO2.IO20
#define SC_P_SPI3_CS1                            110  //!< DMA.SPI3.CS1, LSIO.GPIO2.IO21
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIORHB       111  //!<
#define SC_P_ESAI0_FSR                           112  //!< AUD.ESAI0.FSR, LSIO.GPIO2.IO22
#define SC_P_ESAI0_FST                           113  //!< AUD.ESAI0.FST, LSIO.GPIO2.IO23
#define SC_P_ESAI0_SCKR                          114  //!< AUD.ESAI0.SCKR, LSIO.GPIO2.IO24
#define SC_P_ESAI0_SCKT                          115  //!< AUD.ESAI0.SCKT, LSIO.GPIO2.IO25
#define SC_P_ESAI0_TX0                           116  //!< AUD.ESAI0.TX0, LSIO.GPIO2.IO26
#define SC_P_ESAI0_TX1                           117  //!< AUD.ESAI0.TX1, LSIO.GPIO2.IO27
#define SC_P_ESAI0_TX2_RX3                       118  //!< AUD.ESAI0.TX2_RX3, LSIO.GPIO2.IO28
#define SC_P_ESAI0_TX3_RX2                       119  //!< AUD.ESAI0.TX3_RX2, LSIO.GPIO2.IO29
#define SC_P_ESAI0_TX4_RX1                       120  //!< AUD.ESAI0.TX4_RX1, LSIO.GPIO2.IO30
#define SC_P_ESAI0_TX5_RX0                       121  //!< AUD.ESAI0.TX5_RX0, LSIO.GPIO2.IO31
#define SC_P_MCLK_IN0                            122  //!< AUD.ACM.MCLK_IN0, AUD.ESAI0.RX_HF_CLK, LSIO.GPIO3.IO00
#define SC_P_MCLK_OUT0                           123  //!< AUD.ACM.MCLK_OUT0, AUD.ESAI0.TX_HF_CLK, LSIO.GPIO3.IO01
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIORHC       124  //!<
#define SC_P_SPI0_SCK                            125  //!< DMA.SPI0.SCK, AUD.SAI0.RXC, LSIO.GPIO3.IO02
#define SC_P_SPI0_SDO                            126  //!< DMA.SPI0.SDO, AUD.SAI0.TXD, LSIO.GPIO3.IO03
#define SC_P_SPI0_SDI                            127  //!< DMA.SPI0.SDI, AUD.SAI0.RXD, LSIO.GPIO3.IO04
#define SC_P_SPI0_CS0                            128  //!< DMA.SPI0.CS0, AUD.SAI0.RXFS, LSIO.GPIO3.IO05
#define SC_P_SPI0_CS1                            129  //!< DMA.SPI0.CS1, AUD.SAI0.TXC, LSIO.GPIO3.IO06
#define SC_P_SPI2_SCK                            130  //!< DMA.SPI2.SCK, LSIO.GPIO3.IO07
#define SC_P_SPI2_SDO                            131  //!< DMA.SPI2.SDO, LSIO.GPIO3.IO08
#define SC_P_SPI2_SDI                            132  //!< DMA.SPI2.SDI, LSIO.GPIO3.IO09
#define SC_P_SPI2_CS0                            133  //!< DMA.SPI2.CS0, LSIO.GPIO3.IO10
#define SC_P_SPI2_CS1                            134  //!< DMA.SPI2.CS1, AUD.SAI0.TXFS, LSIO.GPIO3.IO11
#define SC_P_SAI1_RXC                            135  //!< AUD.SAI1.RXC, AUD.SAI0.TXD, LSIO.GPIO3.IO12
#define SC_P_SAI1_RXD                            136  //!< AUD.SAI1.RXD, AUD.SAI0.TXFS, LSIO.GPIO3.IO13
#define SC_P_SAI1_RXFS                           137  //!< AUD.SAI1.RXFS, AUD.SAI0.RXD, LSIO.GPIO3.IO14
#define SC_P_SAI1_TXC                            138  //!< AUD.SAI1.TXC, AUD.SAI0.TXC, LSIO.GPIO3.IO15
#define SC_P_SAI1_TXD                            139  //!< AUD.SAI1.TXD, AUD.SAI1.RXC, LSIO.GPIO3.IO16
#define SC_P_SAI1_TXFS                           140  //!< AUD.SAI1.TXFS, AUD.SAI1.RXFS, LSIO.GPIO3.IO17
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIORHT       141  //!<
#define SC_P_ADC_IN7                             142  //!< DMA.ADC1.IN3, DMA.SPI1.CS1, LSIO.KPP0.ROW3, LSIO.GPIO3.IO25
#define SC_P_ADC_IN6                             143  //!< DMA.ADC1.IN2, DMA.SPI1.CS0, LSIO.KPP0.ROW2, LSIO.GPIO3.IO24
#define SC_P_ADC_IN5                             144  //!< DMA.ADC1.IN1, DMA.SPI1.SDI, LSIO.KPP0.ROW1, LSIO.GPIO3.IO23
#define SC_P_ADC_IN4                             145  //!< DMA.ADC1.IN0, DMA.SPI1.SDO, LSIO.KPP0.ROW0, LSIO.GPIO3.IO22
#define SC_P_ADC_IN3                             146  //!< DMA.ADC0.IN3, DMA.SPI1.SCK, LSIO.KPP0.COL3, LSIO.GPIO3.IO21
#define SC_P_ADC_IN2                             147  //!< DMA.ADC0.IN2, LSIO.KPP0.COL2, LSIO.GPIO3.IO20
#define SC_P_ADC_IN1                             148  //!< DMA.ADC0.IN1, LSIO.KPP0.COL1, LSIO.GPIO3.IO19
#define SC_P_ADC_IN0                             149  //!< DMA.ADC0.IN0, LSIO.KPP0.COL0, LSIO.GPIO3.IO18
#define SC_P_MLB_SIG                             150  //!< CONN.MLB.SIG, AUD.SAI3.RXC, LSIO.GPIO3.IO26
#define SC_P_MLB_CLK                             151  //!< CONN.MLB.CLK, AUD.SAI3.RXFS, LSIO.GPIO3.IO27
#define SC_P_MLB_DATA                            152  //!< CONN.MLB.DATA, AUD.SAI3.RXD, LSIO.GPIO3.IO28
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIOLHT       153  //!<
#define SC_P_FLEXCAN0_RX                         154  //!< DMA.FLEXCAN0.RX, LSIO.GPIO3.IO29
#define SC_P_FLEXCAN0_TX                         155  //!< DMA.FLEXCAN0.TX, LSIO.GPIO3.IO30
#define SC_P_FLEXCAN1_RX                         156  //!< DMA.FLEXCAN1.RX, LSIO.GPIO3.IO31
#define SC_P_FLEXCAN1_TX                         157  //!< DMA.FLEXCAN1.TX, LSIO.GPIO4.IO00
#define SC_P_FLEXCAN2_RX                         158  //!< DMA.FLEXCAN2.RX, LSIO.GPIO4.IO01
#define SC_P_FLEXCAN2_TX                         159  //!< DMA.FLEXCAN2.TX, LSIO.GPIO4.IO02
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIOTHR       160  //!<
#define SC_P_USB_SS3_TC0                         161  //!< DMA.I2C1.SCL, CONN.USB_OTG1.PWR, LSIO.GPIO4.IO03
#define SC_P_USB_SS3_TC1                         162  //!< DMA.I2C1.SCL, CONN.USB_OTG2.PWR, LSIO.GPIO4.IO04
#define SC_P_USB_SS3_TC2                         163  //!< DMA.I2C1.SDA, CONN.USB_OTG1.OC, LSIO.GPIO4.IO05
#define SC_P_USB_SS3_TC3                         164  //!< DMA.I2C1.SDA, CONN.USB_OTG2.OC, LSIO.GPIO4.IO06
#define SC_P_COMP_CTL_GPIO_3V3_USB3IO            165  //!<
#define SC_P_USDHC1_RESET_B                      166  //!< CONN.USDHC1.RESET_B, LSIO.GPIO4.IO07
#define SC_P_USDHC1_VSELECT                      167  //!< CONN.USDHC1.VSELECT, LSIO.GPIO4.IO08
#define SC_P_USDHC2_RESET_B                      168  //!< CONN.USDHC2.RESET_B, LSIO.GPIO4.IO09
#define SC_P_USDHC2_VSELECT                      169  //!< CONN.USDHC2.VSELECT, LSIO.GPIO4.IO10
#define SC_P_USDHC2_WP                           170  //!< CONN.USDHC2.WP, LSIO.GPIO4.IO11
#define SC_P_USDHC2_CD_B                         171  //!< CONN.USDHC2.CD_B, LSIO.GPIO4.IO12
#define SC_P_COMP_CTL_GPIO_1V8_3V3_VSELSEP       172  //!<
#define SC_P_ENET0_MDIO                          173  //!< CONN.ENET0.MDIO, DMA.I2C4.SDA, LSIO.GPIO4.IO13
#define SC_P_ENET0_MDC                           174  //!< CONN.ENET0.MDC, DMA.I2C4.SCL, LSIO.GPIO4.IO14
#define SC_P_ENET0_REFCLK_125M_25M               175  //!< CONN.ENET0.REFCLK_125M_25M, CONN.ENET1.PPS, LSIO.GPIO4.IO15
#define SC_P_ENET1_REFCLK_125M_25M               176  //!< CONN.ENET1.REFCLK_125M_25M, CONN.ENET0.PPS, LSIO.GPIO4.IO16
#define SC_P_ENET1_MDIO                          177  //!< CONN.ENET1.MDIO, DMA.I2C4.SDA, LSIO.GPIO4.IO17
#define SC_P_ENET1_MDC                           178  //!< CONN.ENET1.MDC, DMA.I2C4.SCL, LSIO.GPIO4.IO18
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIOCT        179  //!<
#define SC_P_QSPI1A_SS0_B                        180  //!< LSIO.QSPI1A.SS0_B, LSIO.GPIO4.IO19
#define SC_P_QSPI1A_SS1_B                        181  //!< LSIO.QSPI1A.SS1_B, LSIO.GPIO4.IO20
#define SC_P_QSPI1A_SCLK                         182  //!< LSIO.QSPI1A.SCLK, LSIO.GPIO4.IO21
#define SC_P_QSPI1A_DQS                          183  //!< LSIO.QSPI1A.DQS, LSIO.GPIO4.IO22
#define SC_P_QSPI1A_DATA3                        184  //!< LSIO.QSPI1A.DATA3, LSIO.GPIO4.IO23
#define SC_P_QSPI1A_DATA2                        185  //!< LSIO.QSPI1A.DATA2, LSIO.GPIO4.IO24
#define SC_P_QSPI1A_DATA1                        186  //!< LSIO.QSPI1A.DATA1, LSIO.GPIO4.IO25
#define SC_P_QSPI1A_DATA0                        187  //!< LSIO.QSPI1A.DATA0, LSIO.GPIO4.IO26
#define SC_P_COMP_CTL_GPIO_1V8_3V3_QSPI1         188  //!<
#define SC_P_QSPI0A_DATA0                        189  //!< LSIO.QSPI0A.DATA0
#define SC_P_QSPI0A_DATA1                        190  //!< LSIO.QSPI0A.DATA1
#define SC_P_QSPI0A_DATA2                        191  //!< LSIO.QSPI0A.DATA2
#define SC_P_QSPI0A_DATA3                        192  //!< LSIO.QSPI0A.DATA3
#define SC_P_QSPI0A_DQS                          193  //!< LSIO.QSPI0A.DQS
#define SC_P_QSPI0A_SS0_B                        194  //!< LSIO.QSPI0A.SS0_B
#define SC_P_QSPI0A_SS1_B                        195  //!< LSIO.QSPI0A.SS1_B
#define SC_P_QSPI0A_SCLK                         196  //!< LSIO.QSPI0A.SCLK
#define SC_P_QSPI0B_SCLK                         197  //!< LSIO.QSPI0B.SCLK
#define SC_P_QSPI0B_DATA0                        198  //!< LSIO.QSPI0B.DATA0
#define SC_P_QSPI0B_DATA1                        199  //!< LSIO.QSPI0B.DATA1
#define SC_P_QSPI0B_DATA2                        200  //!< LSIO.QSPI0B.DATA2
#define SC_P_QSPI0B_DATA3                        201  //!< LSIO.QSPI0B.DATA3
#define SC_P_QSPI0B_DQS                          202  //!< LSIO.QSPI0B.DQS
#define SC_P_QSPI0B_SS0_B                        203  //!< LSIO.QSPI0B.SS0_B
#define SC_P_QSPI0B_SS1_B                        204  //!< LSIO.QSPI0B.SS1_B
#define SC_P_COMP_CTL_GPIO_1V8_3V3_QSPI0         205  //!<
#define SC_P_PCIE_CTRL0_CLKREQ_B                 206  //!< HSIO.PCIE0.CLKREQ_B, LSIO.GPIO4.IO27
#define SC_P_PCIE_CTRL0_WAKE_B                   207  //!< HSIO.PCIE0.WAKE_B, LSIO.GPIO4.IO28
#define SC_P_PCIE_CTRL0_PERST_B                  208  //!< HSIO.PCIE0.PERST_B, LSIO.GPIO4.IO29
#define SC_P_PCIE_CTRL1_CLKREQ_B                 209  //!< HSIO.PCIE1.CLKREQ_B, LSIO.GPIO4.IO30
#define SC_P_PCIE_CTRL1_WAKE_B                   210  //!< HSIO.PCIE1.WAKE_B, LSIO.GPIO4.IO31
#define SC_P_PCIE_CTRL1_PERST_B                  211  //!< HSIO.PCIE1.PERST_B, LSIO.GPIO5.IO00
#define SC_P_COMP_CTL_GPIO_1V8_3V3_PCIESEP       212  //!<
#define SC_P_USB_HSIC0_DATA                      213  //!< CONN.USB_HSIC0.DATA, DMA.I2C1.SDA, LSIO.GPIO5.IO01
#define SC_P_USB_HSIC0_STROBE                    214  //!< CONN.USB_HSIC0.STROBE, DMA.I2C1.SCL, LSIO.GPIO5.IO02
#define SC_P_CALIBRATION_0_HSIC                  215  //!<
#define SC_P_CALIBRATION_1_HSIC                  216  //!<
#define SC_P_EMMC0_CLK                           217  //!< CONN.EMMC0.CLK, CONN.NAND.READY_B
#define SC_P_EMMC0_CMD                           218  //!< CONN.EMMC0.CMD, CONN.NAND.DQS, LSIO.GPIO5.IO03
#define SC_P_EMMC0_DATA0                         219  //!< CONN.EMMC0.DATA0, CONN.NAND.DATA00, LSIO.GPIO5.IO04
#define SC_P_EMMC0_DATA1                         220  //!< CONN.EMMC0.DATA1, CONN.NAND.DATA01, LSIO.GPIO5.IO05
#define SC_P_EMMC0_DATA2                         221  //!< CONN.EMMC0.DATA2, CONN.NAND.DATA02, LSIO.GPIO5.IO06
#define SC_P_EMMC0_DATA3                         222  //!< CONN.EMMC0.DATA3, CONN.NAND.DATA03, LSIO.GPIO5.IO07
#define SC_P_EMMC0_DATA4                         223  //!< CONN.EMMC0.DATA4, CONN.NAND.DATA04, LSIO.GPIO5.IO08
#define SC_P_EMMC0_DATA5                         224  //!< CONN.EMMC0.DATA5, CONN.NAND.DATA05, LSIO.GPIO5.IO09
#define SC_P_EMMC0_DATA6                         225  //!< CONN.EMMC0.DATA6, CONN.NAND.DATA06, LSIO.GPIO5.IO10
#define SC_P_EMMC0_DATA7                         226  //!< CONN.EMMC0.DATA7, CONN.NAND.DATA07, LSIO.GPIO5.IO11
#define SC_P_EMMC0_STROBE                        227  //!< CONN.EMMC0.STROBE, CONN.NAND.CLE, LSIO.GPIO5.IO12
#define SC_P_EMMC0_RESET_B                       228  //!< CONN.EMMC0.RESET_B, CONN.NAND.WP_B, LSIO.GPIO5.IO13
#define SC_P_COMP_CTL_GPIO_1V8_3V3_SD1FIX        229  //!<
#define SC_P_USDHC1_CLK                          230  //!< CONN.USDHC1.CLK
#define SC_P_USDHC1_CMD                          231  //!< CONN.USDHC1.CMD, LSIO.GPIO5.IO14
#define SC_P_USDHC1_DATA0                        232  //!< CONN.USDHC1.DATA0, CONN.NAND.RE_N, LSIO.GPIO5.IO15
#define SC_P_USDHC1_DATA1                        233  //!< CONN.USDHC1.DATA1, CONN.NAND.RE_P, LSIO.GPIO5.IO16
#define SC_P_USDHC1_DATA2                        234  //!< CONN.USDHC1.DATA2, CONN.NAND.DQS_N, LSIO.GPIO5.IO17
#define SC_P_USDHC1_DATA3                        235  //!< CONN.USDHC1.DATA3, CONN.NAND.DQS_P, LSIO.GPIO5.IO18
#define SC_P_USDHC1_DATA4                        236  //!< CONN.USDHC1.DATA4, CONN.NAND.CE0_B, LSIO.GPIO5.IO19
#define SC_P_USDHC1_DATA5                        237  //!< CONN.USDHC1.DATA5, CONN.NAND.RE_B, LSIO.GPIO5.IO20
#define SC_P_USDHC1_DATA6                        238  //!< CONN.USDHC1.DATA6, CONN.NAND.WE_B, CONN.USDHC1.WP, LSIO.GPIO5.IO21
#define SC_P_USDHC1_DATA7                        239  //!< CONN.USDHC1.DATA7, CONN.NAND.ALE, CONN.USDHC1.CD_B, LSIO.GPIO5.IO22
#define SC_P_USDHC1_STROBE                       240  //!< CONN.USDHC1.STROBE, CONN.NAND.CE1_B, LSIO.GPIO5.IO23
#define SC_P_COMP_CTL_GPIO_1V8_3V3_VSEL2         241  //!<
#define SC_P_USDHC2_CLK                          242  //!< CONN.USDHC2.CLK, AUD.MQS.R, LSIO.GPIO5.IO24
#define SC_P_USDHC2_CMD                          243  //!< CONN.USDHC2.CMD, AUD.MQS.L, LSIO.GPIO5.IO25
#define SC_P_USDHC2_DATA0                        244  //!< CONN.USDHC2.DATA0, DMA.UART4.RX, LSIO.GPIO5.IO26
#define SC_P_USDHC2_DATA1                        245  //!< CONN.USDHC2.DATA1, DMA.UART4.TX, LSIO.GPIO5.IO27
#define SC_P_USDHC2_DATA2                        246  //!< CONN.USDHC2.DATA2, DMA.UART4.CTS_B, LSIO.GPIO5.IO28
#define SC_P_USDHC2_DATA3                        247  //!< CONN.USDHC2.DATA3, DMA.UART4.RTS_B, LSIO.GPIO5.IO29
#define SC_P_COMP_CTL_GPIO_1V8_3V3_VSEL3         248  //!<
#define SC_P_ENET0_RGMII_TXC                     249  //!< CONN.ENET0.RGMII_TXC, CONN.ENET0.RCLK50M_OUT, LSIO.GPIO5.IO30
#define SC_P_ENET0_RGMII_TX_CTL                  250  //!< CONN.ENET0.RGMII_TX_CTL, LSIO.GPIO5.IO31
#define SC_P_ENET0_RGMII_TXD0                    251  //!< CONN.ENET0.RGMII_TXD0, LSIO.GPIO6.IO00
#define SC_P_ENET0_RGMII_TXD1                    252  //!< CONN.ENET0.RGMII_TXD1, LSIO.GPIO6.IO01
#define SC_P_ENET0_RGMII_TXD2                    253  //!< CONN.ENET0.RGMII_TXD2, LSIO.GPIO6.IO02
#define SC_P_ENET0_RGMII_TXD3                    254  //!< CONN.ENET0.RGMII_TXD3, LSIO.GPIO6.IO03
#define SC_P_ENET0_RGMII_RXC                     255  //!< CONN.ENET0.RGMII_RXC, LSIO.GPIO6.IO04
#define SC_P_ENET0_RGMII_RX_CTL                  256  //!< CONN.ENET0.RGMII_RX_CTL, LSIO.GPIO6.IO05
#define SC_P_ENET0_RGMII_RXD0                    257  //!< CONN.ENET0.RGMII_RXD0, LSIO.GPIO6.IO06
#define SC_P_ENET0_RGMII_RXD1                    258  //!< CONN.ENET0.RGMII_RXD1, LSIO.GPIO6.IO07
#define SC_P_ENET0_RGMII_RXD2                    259  //!< CONN.ENET0.RGMII_RXD2, CONN.ENET0.RMII_RX_ER, LSIO.GPIO6.IO08
#define SC_P_ENET0_RGMII_RXD3                    260  //!< CONN.ENET0.RGMII_RXD3, LSIO.GPIO6.IO09
#define SC_P_COMP_CTL_GPIO_1V8_3V3_ENET_ENETB    261  //!<
#define SC_P_ENET1_RGMII_TXC                     262  //!< CONN.ENET1.RGMII_TXC, CONN.ENET1.RCLK50M_OUT, LSIO.GPIO6.IO10
#define SC_P_ENET1_RGMII_TX_CTL                  263  //!< CONN.ENET1.RGMII_TX_CTL, LSIO.GPIO6.IO11
#define SC_P_ENET1_RGMII_TXD0                    264  //!< CONN.ENET1.RGMII_TXD0, LSIO.GPIO6.IO12
#define SC_P_ENET1_RGMII_TXD1                    265  //!< CONN.ENET1.RGMII_TXD1, LSIO.GPIO6.IO13
#define SC_P_ENET1_RGMII_TXD2                    266  //!< CONN.ENET1.RGMII_TXD2, DMA.UART3.TX, VPU.TSI_S1.VID, LSIO.GPIO6.IO14
#define SC_P_ENET1_RGMII_TXD3                    267  //!< CONN.ENET1.RGMII_TXD3, DMA.UART3.RTS_B, VPU.TSI_S1.SYNC, LSIO.GPIO6.IO15
#define SC_P_ENET1_RGMII_RXC                     268  //!< CONN.ENET1.RGMII_RXC, DMA.UART3.CTS_B, VPU.TSI_S1.DATA, LSIO.GPIO6.IO16
#define SC_P_ENET1_RGMII_RX_CTL                  269  //!< CONN.ENET1.RGMII_RX_CTL, VPU.TSI_S0.VID, LSIO.GPIO6.IO17
#define SC_P_ENET1_RGMII_RXD0                    270  //!< CONN.ENET1.RGMII_RXD0, VPU.TSI_S0.SYNC, LSIO.GPIO6.IO18
#define SC_P_ENET1_RGMII_RXD1                    271  //!< CONN.ENET1.RGMII_RXD1, VPU.TSI_S0.DATA, LSIO.GPIO6.IO19
#define SC_P_ENET1_RGMII_RXD2                    272  //!< CONN.ENET1.RGMII_RXD2, CONN.ENET1.RMII_RX_ER, VPU.TSI_S0.CLK, LSIO.GPIO6.IO20
#define SC_P_ENET1_RGMII_RXD3                    273  //!< CONN.ENET1.RGMII_RXD3, DMA.UART3.RX, VPU.TSI_S1.CLK, LSIO.GPIO6.IO21
#define SC_P_COMP_CTL_GPIO_1V8_3V3_ENET_ENETA    274  //!<
#define SC_P_ANA_TEST_OUT_P                      275  //!< SCU.DSC.TEST_OUT_P
#define SC_P_ANA_TEST_OUT_N                      276  //!< SCU.DSC.TEST_OUT_N
#define SC_P_XTALI                               277  //!< SCU.DSC.XTALI
#define SC_P_XTALO                               278  //!< SCU.DSC.XTALO
#define SC_P_RTC_XTALI                           279  //!< SNVS.RTC_XTALI
#define SC_P_RTC_XTALO                           280  //!< SNVS.RTC_XTALO
#define SC_P_PMIC_ON_REQ                         281  //!< SNVS.PMIC_ON_REQ
#define SC_P_ON_OFF_BUTTON                       282  //!< SNVS.ON_OFF_BUTTON
#define SC_P_SNVS_TAMPER_OUT0                    283  //!< SNVS.TAMPER_OUT0
#define SC_P_SNVS_TAMPER_OUT1                    284  //!< SNVS.TAMPER_OUT1
#define SC_P_SNVS_TAMPER_IN0                     285  //!< SNVS.TAMPER_IN0
#define SC_P_SNVS_TAMPER_IN1                     286  //!< SNVS.TAMPER_IN1
#define SC_P_VDD_SNVS_1P8_IN                     287  //!< SNVS.VDD_SNVS_IN
/*@}*/

#endif /* _SC_PINS_H */

