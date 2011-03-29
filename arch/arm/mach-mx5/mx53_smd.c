/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/i2c/mpr.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/ahci_platform.h>
#include <linux/gpio_keys.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/keypad.h>
#include <linux/spi/flash.h>
#include <mach/memory.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/mxc_dvfs.h>
#include <mach/iomux-mx53.h>
#include <mach/i2c.h>
#include <mach/mxc_iim.h>
#include <mach/mxc_rfkill.h>

#include "crm_regs.h"
#include "devices.h"
#include "usb.h"

/*!
 * @file mach-mx5/mx53_smd.c
 *
 * @brief This file contains MX53 smd board specific initialization routines.
 *
 * @ingroup MSL_MX53
 */

/* MX53 SMD GPIO PIN configurations */
#define MX53_SMD_KEY_RESET			(0*32 + 2)	/* GPIO1_2 */
#define MX53_SMD_SATA_CLK_GPEN		(0*32 + 4)	/* GPIO1_4 */
#define MX53_SMD_PMIC_FAULT			(0*32 + 5)	/* GPIO1_5 */
#define MX53_SMD_SYS_ON_OFF_CTL		(0*32 + 7)	/* GPIO1_7 */
#define MX53_SMD_PMIC_ON_OFF_REQ	(0*32 + 8)	/* GPIO1_8 */

#define MX53_SMD_FEC_INT			(1*32 + 4)	/* GPIO2_4 */
#define MX53_SMD_HEADPHONE_DEC		(1*32 + 5)	/* GPIO2_5 */
#define MX53_SMD_ZIGBEE_INT			(1*32 + 6)	/* GPIO2_6 */
#define MX53_SMD_ZIGBEE_RESET_B		(1*32 + 7)	/* GPIO2_7 */
#define MX53_SMD_GPS_RESET_B		(1*32 + 12)	/* GPIO2_12 */
#define MX53_SMD_WAKEUP_ZIGBEE		(1*32 + 13)	/* GPIO2_13 */
#define MX53_SMD_KEY_VOL_DOWN		(1*32 + 14)	/* GPIO2_14 */
#define MX53_SMD_KEY_VOL_UP			(1*32 + 15)	/* GPIO2_15 */
#define MX53_SMD_FEC_PWR_EN			(1*32 + 16)	/* GPIO_2_16 */
#define MX53_SMD_LID_OPN_CLS_SW		(1*32 + 23)	/* GPIO_2_23 */
#define MX53_SMD_GPS_PPS		(1*32 + 24)	/* GPIO_2_24 */

#define MX53_SMD_DCDC1V8_EN			(2*32 + 1)	/*GPIO_3_1 */
#define MX53_SMD_AUD_AMP_STBY_B		(2*32 + 2)	/*GPIO_3_2 */
#define MX53_SMD_SATA_PWR_EN		(2*32 + 3)	/*GPIO_3_3 */
#define MX53_SMD_TPM_OSC_EN			(2*32 + 4)	/*GPIO_3_4 */
#define MX53_SMD_WLAN_PD			(2*32 + 5)	/*GPIO_3_5 */
#define MX53_SMD_WiFi_BT_PWR_EN		(2*32 + 10)	/*GPIO_3_10 */
#define MX53_SMD_RECOVERY_MODE_SW	(2*32 + 11)	/*GPIO_3_11 */
#define MX53_SMD_USB_OTG_OC			(2*32 + 12)	/*GPIO_3_12 */
#define MX53_SMD_SD1_CD				(2*32 + 13)	/*GPIO_3_13 */
#define MX53_SMD_USB_HUB_RESET_B	(2*32 + 14)	/*GPIO_3_14 */
#define MX53_SMD_eCOMPASS_INT		(2*32 + 15)	/*GPIO_3_15 */
#define MX53_SMD_CAP_TCH_INT1		(2*32 + 20)	/* GPIO_3_20 */
#define MX53_SMD_BT_PRIORITY		(2*32 + 21)	/* GPIO_3_21 */
#define MX53_SMD_ALS_INT			(2*32 + 22)	/* GPIO_3_22 */
#define MX53_SMD_TPM_INT			(2*32 + 26)	/* GPIO_3_26 */
#define MX53_SMD_MODEM_WKUP			(2*32 + 27)	/* GPIO_3_27 */
#define MX53_SMD_BT_RESET			(2*32 + 28)	/* GPIO_3_28 */
#define MX53_SMD_TPM_RST_B			(2*32 + 29)	/* GPIO_3_29 */
#define MX53_SMD_CHRG_OR_CMOS		(2*32 + 30)	/* GPIO_3_30 */
#define MX53_SMD_CAP_TCH_INT0		(2*32 + 31)	/* GPIO_3_31 */

#define MX53_SMD_CHA_ISET			(3*32 + 2)	/* GPIO_4_2 */
#define MX53_SMD_SYS_EJECT			(3*32 + 3)	/* GPIO_4_3 */
#define MX53_SMD_HDMI_CEC_D			(3*32 + 4)	/* GPIO_4_4 */

#define MX53_SMD_MODEM_DISABLE_B	(3*32 + 10)	/* GPIO_4_10 */
#define MX53_SMD_SD1_WP				(3*32 + 11)	/* GPIO_4_11 */
#define MX53_SMD_DCDC5V_BB_EN		(3*32 + 14)	/* GPIO_4_14 */
#define MX53_SMD_WLAN_HOST_WAKE		(3*32 + 15)	/* GPIO_4_15 */

#define MX53_SMD_HDMI_RESET_B		(4*32 + 0)	/* GPIO_5_0 */
#define MX53_SMD_MODEM_RESET_B		(4*32 + 2)	/* GPIO_5_2 */
#define MX53_SMD_KEY_INT			(4*32 + 4)	/* GPIO_5_4 */

#define MX53_SMD_CAP_TCH_FUN0		(5*32 + 6)	/* GPIO_6_6 */
#define MX53_SMD_CSI0_RST			(5*32 + 9)	/* GPIO_6_9 */
#define MX53_SMD_CSI0_PWN			(5*32 + 10)	/* GPIO_6_10 */
#define MX53_SMD_OSC_CKIH1_EN		(5*32 + 11)	/* GPIO_6_11 */
#define MX53_SMD_HDMI_INT			(5*32 + 12)	/* GPIO_6_12 */
#define MX53_SMD_LCD_PWR_EN			(5*32 + 13)	/* GPIO_6_13 */
#define MX53_SMD_ACCL_INT1_IN		(5*32 + 15)	/* GPIO_6_15 */
#define MX53_SMD_ACCL_INT2_IN		(5*32 + 16)	/* GPIO_6_16 */
#define MX53_SMD_AC_IN				(5*32 + 17)	/* GPIO_6_17 */
#define MX53_SMD_PWR_GOOD			(5*32 + 18)	/* GPIO_6_18 */

#define MX53_SMD_CABC_EN0			(6*32 + 2)	/* GPIO_7_2 */
#define MX53_SMD_DOCK_DECTECT		(6*32 + 3)	/* GPIO_7_3 */
#define MX53_SMD_FEC_RST			(6*32 + 6)	/* GPIO_7_6 */
#define MX53_SMD_USER_DEG_CHG_NONE	(6*32 + 7)	/* GPIO_7_7 */
#define MX53_SMD_USB_OTG_PWR_EN		(6*32 + 8)	/* GPIO_7_8 */
#define MX53_SMD_DEVELOP_MODE_SW	(6*32 + 9)	/* GPIO_7_9 */
#define MX53_SMD_CABC_EN1			(6*32 + 10)	/* GPIO_7_10 */
#define MX53_SMD_PMIC_INT			(6*32 + 11)	/* GPIO_7_11 */
#define MX53_SMD_CAP_TCH_FUN1		(6*32 + 13)	/* GPIO_7_13 */

extern int __init mx53_smd_init_da9052(void);

static iomux_v3_cfg_t mx53_smd_pads[] = {
	/* DI_VGA_HSYNC */
	MX53_PAD_EIM_OE__IPU_DI1_PIN7,
	/* HDMI reset */
	MX53_PAD_EIM_WAIT__GPIO5_0,
	/* DI_VGA_VSYNC */
	MX53_PAD_EIM_RW__IPU_DI1_PIN8,
	/* CSPI1 */
	MX53_PAD_EIM_EB2__ECSPI1_SS0,
	MX53_PAD_EIM_D16__ECSPI1_SCLK,
	MX53_PAD_EIM_D17__ECSPI1_MISO,
	MX53_PAD_EIM_D18__ECSPI1_MOSI,
	MX53_PAD_EIM_D19__ECSPI1_SS1,
	/* BT: UART3*/
	MX53_PAD_EIM_D24__UART3_TXD_MUX,
	MX53_PAD_EIM_D25__UART3_RXD_MUX,
	MX53_PAD_EIM_EB3__UART3_RTS,
	MX53_PAD_EIM_D23__UART3_CTS,
	/* LID_OPN_CLS_SW*/
	MX53_PAD_EIM_CS0__GPIO2_23,
	/* GPS_PPS */
	MX53_PAD_EIM_CS1__GPIO2_24,
	/* FEC_PWR_EN */
	MX53_PAD_EIM_A22__GPIO2_16,
	/* CAP_TCH_FUN0*/
	MX53_PAD_EIM_A23__GPIO6_6,
	/* KEY_INT */
	MX53_PAD_EIM_A24__GPIO5_4,
	/* MODEM_RESET_B */
	MX53_PAD_EIM_A25__GPIO5_2,
	/* CAP_TCH_INT1 */
	MX53_PAD_EIM_D20__GPIO3_20,
	/* BT_PRIORITY */
	MX53_PAD_EIM_D21__GPIO3_21,
	/* ALS_INT */
	MX53_PAD_EIM_D22__GPIO3_22,
	/* TPM_INT */
	MX53_PAD_EIM_D26__GPIO3_26,
	/* MODEM_WKUP */
	MX53_PAD_EIM_D27__GPIO3_27,
	/* BT_RESET */
	MX53_PAD_EIM_D28__GPIO3_28,
	/* TPM_RST_B */
	MX53_PAD_EIM_D29__GPIO3_29,
	/* CHARGER_NOW_OR_CMOS_RUN */
	MX53_PAD_EIM_D30__GPIO3_30,
	/* CAP_TCH_INT0 */
	MX53_PAD_EIM_D31__GPIO3_31,
	/* DCDC1V8_EN */
	MX53_PAD_EIM_DA1__GPIO3_1,
	/* AUD_AMP_STBY_B */
	MX53_PAD_EIM_DA2__GPIO3_2,
	/* SATA_PWR_EN */
	MX53_PAD_EIM_DA3__GPIO3_3,
	/* TPM_OSC_EN */
	MX53_PAD_EIM_DA4__GPIO3_4,
	/* WLAN_PD */
	MX53_PAD_EIM_DA5__GPIO3_5,
	/* WiFi_BT_PWR_EN */
	MX53_PAD_EIM_DA10__GPIO3_10,
	/* RECOVERY_MODE_SW */
	MX53_PAD_EIM_DA11__GPIO3_11,
	/* USB_OTG_OC */
	MX53_PAD_EIM_DA12__GPIO3_12,
	/* SD1_CD */
	MX53_PAD_EIM_DA13__GPIO3_13,
	/* USB_HUB_RESET_B */
	MX53_PAD_EIM_DA14__GPIO3_14,
	/* eCOMPASS_IN */
	MX53_PAD_EIM_DA15__GPIO3_15,
	/* HDMI_INT */
	MX53_PAD_NANDF_WE_B__GPIO6_12,
	/* LCD_PWR_EN */
	MX53_PAD_NANDF_RE_B__GPIO6_13,
	/* CSI0_RST */
	MX53_PAD_NANDF_WP_B__GPIO6_9,
	/* CSI0_PWN */
	MX53_PAD_NANDF_RB0__GPIO6_10,
	/* OSC_CKIH1_EN */
	MX53_PAD_NANDF_CS0__GPIO6_11,
	/* ACCL_INT1_IN */
	MX53_PAD_NANDF_CS2__GPIO6_15,
	/* ACCL_INT2_IN */
	MX53_PAD_NANDF_CS3__GPIO6_16,
	/* AUDMUX */
	MX53_PAD_CSI0_DAT4__AUDMUX_AUD3_TXC,
	MX53_PAD_CSI0_DAT5__AUDMUX_AUD3_TXD,
	MX53_PAD_CSI0_DAT6__AUDMUX_AUD3_TXFS,
	MX53_PAD_CSI0_DAT7__AUDMUX_AUD3_RXD,
	/* I2C1 */
	MX53_PAD_CSI0_DAT8__I2C1_SDA,
	MX53_PAD_CSI0_DAT9__I2C1_SCL,
	/* UART1 */
	MX53_PAD_CSI0_DAT10__UART1_TXD_MUX,
	MX53_PAD_CSI0_DAT11__UART1_RXD_MUX,
	/* CSI0 */
	MX53_PAD_CSI0_DAT12__IPU_CSI0_D_12,
	MX53_PAD_CSI0_DAT13__IPU_CSI0_D_13,
	MX53_PAD_CSI0_DAT14__IPU_CSI0_D_14,
	MX53_PAD_CSI0_DAT15__IPU_CSI0_D_15,
	MX53_PAD_CSI0_DAT16__IPU_CSI0_D_16,
	MX53_PAD_CSI0_DAT17__IPU_CSI0_D_17,
	MX53_PAD_CSI0_DAT18__IPU_CSI0_D_18,
	MX53_PAD_CSI0_DAT19__IPU_CSI0_D_19,
	MX53_PAD_CSI0_VSYNC__IPU_CSI0_VSYNC,
	MX53_PAD_CSI0_MCLK__IPU_CSI0_HSYNC,
	MX53_PAD_CSI0_PIXCLK__IPU_CSI0_PIXCLK,
	/* DISPLAY */
	MX53_PAD_DI0_DISP_CLK__IPU_DI0_DISP_CLK,
	MX53_PAD_DI0_PIN15__IPU_DI0_PIN15,
	MX53_PAD_DI0_PIN2__IPU_DI0_PIN2,
	MX53_PAD_DI0_PIN3__IPU_DI0_PIN3,
	MX53_PAD_DISP0_DAT0__IPU_DISP0_DAT_0,
	MX53_PAD_DISP0_DAT1__IPU_DISP0_DAT_1,
	MX53_PAD_DISP0_DAT2__IPU_DISP0_DAT_2,
	MX53_PAD_DISP0_DAT3__IPU_DISP0_DAT_3,
	MX53_PAD_DISP0_DAT4__IPU_DISP0_DAT_4,
	MX53_PAD_DISP0_DAT5__IPU_DISP0_DAT_5,
	MX53_PAD_DISP0_DAT6__IPU_DISP0_DAT_6,
	MX53_PAD_DISP0_DAT7__IPU_DISP0_DAT_7,
	MX53_PAD_DISP0_DAT8__IPU_DISP0_DAT_8,
	MX53_PAD_DISP0_DAT9__IPU_DISP0_DAT_9,
	MX53_PAD_DISP0_DAT10__IPU_DISP0_DAT_10,
	MX53_PAD_DISP0_DAT11__IPU_DISP0_DAT_11,
	MX53_PAD_DISP0_DAT12__IPU_DISP0_DAT_12,
	MX53_PAD_DISP0_DAT13__IPU_DISP0_DAT_13,
	MX53_PAD_DISP0_DAT14__IPU_DISP0_DAT_14,
	MX53_PAD_DISP0_DAT15__IPU_DISP0_DAT_15,
	MX53_PAD_DISP0_DAT16__IPU_DISP0_DAT_16,
	MX53_PAD_DISP0_DAT17__IPU_DISP0_DAT_17,
	MX53_PAD_DISP0_DAT18__IPU_DISP0_DAT_18,
	MX53_PAD_DISP0_DAT19__IPU_DISP0_DAT_19,
	MX53_PAD_DISP0_DAT20__IPU_DISP0_DAT_20,
	MX53_PAD_DISP0_DAT21__IPU_DISP0_DAT_21,
	MX53_PAD_DISP0_DAT22__IPU_DISP0_DAT_22,
	MX53_PAD_DISP0_DAT23__IPU_DISP0_DAT_23,
	/* FEC */
	MX53_PAD_FEC_MDC__FEC_MDC,
	MX53_PAD_FEC_MDIO__FEC_MDIO,
	MX53_PAD_FEC_REF_CLK__FEC_TX_CLK,
	MX53_PAD_FEC_RX_ER__FEC_RX_ER,
	MX53_PAD_FEC_CRS_DV__FEC_RX_DV,
	MX53_PAD_FEC_RXD1__FEC_RDATA_1,
	MX53_PAD_FEC_RXD0__FEC_RDATA_0,
	MX53_PAD_FEC_TX_EN__FEC_TX_EN,
	MX53_PAD_FEC_TXD1__FEC_TDATA_1,
	MX53_PAD_FEC_TXD0__FEC_TDATA_0,
	/* AUDMUX5 */
	MX53_PAD_KEY_COL0__AUDMUX_AUD5_TXC,
	MX53_PAD_KEY_ROW0__AUDMUX_AUD5_TXD,
	MX53_PAD_KEY_COL1__AUDMUX_AUD5_TXFS,
	MX53_PAD_KEY_ROW1__AUDMUX_AUD5_RXD,
	/* MODEM_DISABLE_B */
	MX53_PAD_KEY_COL2__GPIO4_10,
	/* SD1_WP */
	MX53_PAD_KEY_ROW2__GPIO4_11,
	/* I2C2 */
	MX53_PAD_KEY_COL3__I2C2_SCL,
	MX53_PAD_KEY_ROW3__I2C2_SDA,
	/* DCDC5V_BB_EN */
	MX53_PAD_KEY_COL4__GPIO4_14,
	/* WLAN_HOST_WAKE */
	MX53_PAD_KEY_ROW4__GPIO4_15,
	/* SD1 */
	MX53_PAD_SD1_CMD__ESDHC1_CMD,
	MX53_PAD_SD1_CLK__ESDHC1_CLK,
	MX53_PAD_SD1_DATA0__ESDHC1_DAT0,
	MX53_PAD_SD1_DATA1__ESDHC1_DAT1,
	MX53_PAD_SD1_DATA2__ESDHC1_DAT2,
	MX53_PAD_SD1_DATA3__ESDHC1_DAT3,
	/* SD2 */
	MX53_PAD_SD2_CMD__ESDHC2_CMD,
	MX53_PAD_SD2_CLK__ESDHC2_CLK,
	MX53_PAD_SD2_DATA0__ESDHC2_DAT0,
	MX53_PAD_SD2_DATA1__ESDHC2_DAT1,
	MX53_PAD_SD2_DATA2__ESDHC2_DAT2,
	MX53_PAD_SD2_DATA3__ESDHC2_DAT3,
	/* UART2 */
	MX53_PAD_PATA_BUFFER_EN__UART2_RXD_MUX,
	MX53_PAD_PATA_DMARQ__UART2_TXD_MUX,
	/* DEVELOP_MODE_SW */
	MX53_PAD_PATA_CS_0__GPIO7_9,
	/* CABC_EN1 */
	MX53_PAD_PATA_CS_1__GPIO7_10,
	/* FEC_nRST */
	MX53_PAD_PATA_DA_0__GPIO7_6,
	/* USER_DEBUG_OR_CHARGER_DONE */
	MX53_PAD_PATA_DA_1__GPIO7_7,
	/* USB_OTG_PWR_EN */
	MX53_PAD_PATA_DA_2__GPIO7_8,
	/* SD3 */
	MX53_PAD_PATA_DATA8__ESDHC3_DAT0,
	MX53_PAD_PATA_DATA9__ESDHC3_DAT1,
	MX53_PAD_PATA_DATA10__ESDHC3_DAT2,
	MX53_PAD_PATA_DATA11__ESDHC3_DAT3,
	MX53_PAD_PATA_DATA0__ESDHC3_DAT4,
	MX53_PAD_PATA_DATA1__ESDHC3_DAT5,
	MX53_PAD_PATA_DATA2__ESDHC3_DAT6,
	MX53_PAD_PATA_DATA3__ESDHC3_DAT7,
	MX53_PAD_PATA_IORDY__ESDHC3_CLK,
	MX53_PAD_PATA_RESET_B__ESDHC3_CMD,
	/* FEC_nINT */
	MX53_PAD_PATA_DATA4__GPIO2_4,
	/* HEADPHONE DET*/
	MX53_PAD_PATA_DATA5__GPIO2_5,
	/* ZigBee_INT*/
	MX53_PAD_PATA_DATA6__GPIO2_6,
	/* ZigBee_RESET_B */
	MX53_PAD_PATA_DATA7__GPIO2_7,
	/* GPS_RESET_B*/
	MX53_PAD_PATA_DATA12__GPIO2_12,
	/* WAKEUP_ZigBee */
	MX53_PAD_PATA_DATA13__GPIO2_13,
	/* KEY_VOL- */
	MX53_PAD_PATA_DATA14__GPIO2_14,
	/* KEY_VOL+ */
	MX53_PAD_PATA_DATA15__GPIO2_15,
	/* DOCK_DECTECT */
	MX53_PAD_PATA_DIOR__GPIO7_3,
	/* AC_IN */
	MX53_PAD_PATA_DIOW__GPIO6_17,
	/* PWR_GOOD */
	MX53_PAD_PATA_DMACK__GPIO6_18,
	/* CABC_EN0 */
	MX53_PAD_PATA_INTRQ__GPIO7_2,
	MX53_PAD_GPIO_0__CCM_SSI_EXT1_CLK,
	MX53_PAD_GPIO_1__PWM2_PWMO,
	/* KEY_RESET */
	MX53_PAD_GPIO_2__GPIO1_2,
	/* I2C3 */
	MX53_PAD_GPIO_3__I2C3_SCL,
	MX53_PAD_GPIO_6__I2C3_SDA,
	/* SATA_CLK_GPEN */
	MX53_PAD_GPIO_4__GPIO1_4,
	/* PMIC_FAULT */
	MX53_PAD_GPIO_5__GPIO1_5,
	/* SYS_ON_OFF_CTL */
	MX53_PAD_GPIO_7__GPIO1_7,
	/* PMIC_ON_OFF_REQ */
	MX53_PAD_GPIO_8__GPIO1_8,
	/* CHA_ISET */
	MX53_PAD_GPIO_12__GPIO4_2,
	/* SYS_EJECT */
	MX53_PAD_GPIO_13__GPIO4_3,
	/* HDMI_CEC_D */
	MX53_PAD_GPIO_14__GPIO4_4,
	/* PMIC_INT */
	MX53_PAD_GPIO_16__GPIO7_11,
	MX53_PAD_GPIO_17__SPDIF_OUT1,
	/* CAP_TCH_FUN1 */
	MX53_PAD_GPIO_18__GPIO7_13,
	/* LVDS */
	MX53_PAD_LVDS0_TX3_P__LDB_LVDS0_TX3,
	MX53_PAD_LVDS0_CLK_P__LDB_LVDS0_CLK,
	MX53_PAD_LVDS0_TX2_P__LDB_LVDS0_TX2,
	MX53_PAD_LVDS0_TX1_P__LDB_LVDS0_TX1,
	MX53_PAD_LVDS0_TX0_P__LDB_LVDS0_TX0,
	MX53_PAD_LVDS1_TX3_P__LDB_LVDS1_TX3,
	MX53_PAD_LVDS1_TX2_P__LDB_LVDS1_TX2,
	MX53_PAD_LVDS1_CLK_P__LDB_LVDS1_CLK,
	MX53_PAD_LVDS1_TX1_P__LDB_LVDS1_TX1,
	MX53_PAD_LVDS1_TX0_P__LDB_LVDS1_TX0,
};

static struct fb_videomode video_modes[] = {
	{
	 /* 800x480 @ 57 Hz , pixel clk @ 27MHz */
	 "CLAA-WVGA", 57, 800, 480, 37037, 40, 60, 10, 10, 20, 10,
	 FB_SYNC_CLK_LAT_FALL,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	/* 800x480 @ 60 Hz , pixel clk @ 32MHz */
	"SEIKO-WVGA", 60, 800, 480, 29850, 89, 164, 23, 10, 10, 10,
	FB_SYNC_CLK_LAT_FALL,
	FB_VMODE_NONINTERLACED,
	0,},
	{
	/* 1600x1200 @ 60 Hz 162M pixel clk*/
	"UXGA", 60, 1600, 1200, 6172,
	304, 64,
	1, 46,
	192, 3,
	FB_SYNC_HOR_HIGH_ACT|FB_SYNC_VERT_HIGH_ACT,
	FB_VMODE_NONINTERLACED,
	0,},
};

static struct platform_pwm_backlight_data mxc_pwm_backlight_data = {
	.pwm_id = 1,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

extern void mx5_ipu_reset(void);
static struct mxc_ipu_config mxc_ipu_data = {
	.rev = 3,
	.reset = mx5_ipu_reset,
};

extern void mx5_vpu_reset(void);
static struct mxc_vpu_platform_data mxc_vpu_data = {
	.iram_enable = true,
	.iram_size = 0x14000,
	.reset = mx5_vpu_reset,
};

static struct fec_platform_data fec_data = {
	.phy = PHY_INTERFACE_MODE_RMII,
};


static struct mxc_spi_master mxcspi1_data = {
	.maxchipselect = 4,
	.spi_version = 23,
	.chipselect_active = NULL,
	.chipselect_inactive = NULL,
};

static struct mxc_dvfs_platform_data dvfs_core_data = {
	.reg_id = "DA9052_BUCK_CORE",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.gpc_vcr_offset = MXC_GPC_VCR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 30,
};

static struct mxc_bus_freq_platform_data bus_freq_data = {
	.gp_reg_id = "DA9052_BUCK_CORE",
	.lp_reg_id = "DA9052_BUCK_PRO",
};

static struct tve_platform_data tve_data = {
	.dac_reg = "DA9052_LDO7",
};

static struct ldb_platform_data ldb_data = {
	.ext_ref = 1,
	.boot_enable = MXC_LDBDI1,
};

static void mxc_iim_enable_fuse(void)
{
	u32 reg;

	if (!ccm_base)
		return;

	/* enable fuse blown */
	reg = readl(ccm_base + 0x64);
	reg |= 0x10;
	writel(reg, ccm_base + 0x64);
}

static void mxc_iim_disable_fuse(void)
{
	u32 reg;

	if (!ccm_base)
		return;
	/* enable fuse blown */
	reg = readl(ccm_base + 0x64);
	reg &= ~0x10;
	writel(reg, ccm_base + 0x64);
}

static struct mxc_iim_data iim_data = {
	.bank_start = MXC_IIM_MX53_BANK_START_ADDR,
	.bank_end   = MXC_IIM_MX53_BANK_END_ADDR,
	.enable_fuse = mxc_iim_enable_fuse,
	.disable_fuse = mxc_iim_disable_fuse,
};

static struct resource mxcfb_resources[] = {
	[0] = {
	       .flags = IORESOURCE_MEM,
	       },
};

static struct mxc_fb_platform_data fb_data[] = {
	{
	 .interface_pix_fmt = IPU_PIX_FMT_RGB24,
	 .mode_str = "1024x768M-16@60",
	 .mode = video_modes,
	 .num_modes = ARRAY_SIZE(video_modes),
	 },
	{
	 .interface_pix_fmt = IPU_PIX_FMT_RGB666,
	 .mode_str = "XGA",
	 .mode = video_modes,
	 .num_modes = ARRAY_SIZE(video_modes),
	 },
};

extern int primary_di;
static int __init mxc_init_fb(void)
{
	if (!machine_is_mx53_smd())
		return 0;

	/*for smd board, set default display as LDB*/
	if (primary_di < 0)
		primary_di = 1;

	if (primary_di) {
		printk(KERN_INFO "DI1 is primary\n");
		/* DI1 -> DP-BG channel: */
		mxc_fb_devices[1].num_resources = ARRAY_SIZE(mxcfb_resources);
		mxc_fb_devices[1].resource = mxcfb_resources;
		mxc_register_device(&mxc_fb_devices[1], &fb_data[1]);

		/* DI0 -> DC channel: */
		mxc_register_device(&mxc_fb_devices[0], &fb_data[0]);
	} else {
		printk(KERN_INFO "DI0 is primary\n");

		/* DI0 -> DP-BG channel: */
		mxc_fb_devices[0].num_resources = ARRAY_SIZE(mxcfb_resources);
		mxc_fb_devices[0].resource = mxcfb_resources;
		mxc_register_device(&mxc_fb_devices[0], &fb_data[0]);

		/* DI1 -> DC channel: */
		mxc_register_device(&mxc_fb_devices[1], &fb_data[1]);
	}

	/*
	 * DI0/1 DP-FG channel:
	 */
	mxc_register_device(&mxc_fb_devices[2], NULL);

	return 0;
}
device_initcall(mxc_init_fb);

static void sii902x_hdmi_reset(void)
{
	gpio_set_value(MX53_SMD_HDMI_RESET_B, 0);
	msleep(10);
	gpio_set_value(MX53_SMD_HDMI_RESET_B, 1);
	msleep(10);
}

static struct mxc_lcd_platform_data sii902x_hdmi_data = {
	.reset = sii902x_hdmi_reset,
	.analog_reg = "DA9052_LDO2",
	.boot_enable = 1,
};

static struct imxi2c_platform_data mxci2c_data = {
       .bitrate = 100000,
};

static struct mxc_camera_platform_data camera_data = {
	.analog_regulator = "DA9052_LDO7",
	.core_regulator = "DA9052_LDO9",
	.mclk = 24000000,
	.csi = 0,
};

static struct mxc_lightsensor_platform_data ls_data = {
	.rext = 700,	/* calibration: 499K->700K */
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
	.type = "mma8451",
	.addr = 0x1C,
	 },
	{
	.type = "ov3640",
	.addr = 0x3C,
	.platform_data = (void *)&camera_data,
	 },
};

static u16 smd_touchkey_martix[4] = {
	KEY_BACK, KEY_HOME, KEY_MENU, KEY_SEARCH,
};

static struct mpr121_platform_data mpr121_keyboard_platdata = {
	.keycount = ARRAY_SIZE(smd_touchkey_martix),
	.vdd_uv = 3300000,
	.matrix = smd_touchkey_martix,
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
	 .type = "sgtl5000-i2c",
	 .addr = 0x0a,
	 },
	{
	.type = "mpr121_touchkey",
	.addr = 0x5a,
	.irq = gpio_to_irq(MX53_SMD_KEY_INT),
	.platform_data = &mpr121_keyboard_platdata,
	},
};

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake)	\
{								\
	.gpio		= gpio_num,				\
	.type		= EV_KEY,				\
	.code		= ev_code,				\
	.active_low	= act_low,				\
	.desc		= "btn " descr,				\
	.wakeup		= wake,					\
}

static struct gpio_keys_button smd_buttons[] = {
	GPIO_BUTTON(MX53_SMD_KEY_VOL_UP, KEY_VOLUMEUP, 1, "volume-up", 0),
	GPIO_BUTTON(MX53_SMD_KEY_VOL_DOWN, KEY_VOLUMEDOWN, 1, "volume-down", 0),
};

static struct gpio_keys_platform_data smd_button_data = {
	.buttons	= smd_buttons,
	.nbuttons	= ARRAY_SIZE(smd_buttons),
};

static struct platform_device smd_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &smd_button_data,
	}
};

static void __init smd_add_device_buttons(void)
{
	platform_device_register(&smd_button_device);
}
#else
static void __init smd_add_device_buttons(void) {}
#endif

static int p1003_ts_hw_status(void)
{
	return gpio_get_value(MX53_SMD_CAP_TCH_INT1);
}

static struct p1003_ts_platform_data p1003_ts_data = {
	.hw_status = p1003_ts_hw_status,
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
	.type = "sii902x",
	.addr = 0x39,
	.irq = gpio_to_irq(MX53_SMD_HDMI_INT),
	.platform_data = &sii902x_hdmi_data,
	},
	{
	.type = "p1003_fwv33",
	.addr = 0x41,
	.irq  = gpio_to_irq(MX53_SMD_CAP_TCH_INT1),
	.platform_data = &p1003_ts_data,
	},
	{
	.type = "isl29023",
	.addr = 0x44,
	.irq  = gpio_to_irq(MX53_SMD_ALS_INT),
	.platform_data = &ls_data,
	},
};

#if defined(CONFIG_MTD_MXC_M25P80) || defined(CONFIG_MTD_MXC_M25P80_MODULE)
static struct mtd_partition m25p32_partitions[] = {
	{
	.name = "bootloader",
	.offset = 0,
	.size = 0x00040000,
	},
	{
	.name = "kernel",
	.offset = MTDPART_OFS_APPEND,
	.size = MTDPART_SIZ_FULL,
	},
};

static struct flash_platform_data m25p32_spi_flash_data = {
	.name = "m25p32",
	.parts = m25p32_partitions,
	.nr_parts = ARRAY_SIZE(m25p32_partitions),
	.type = "m25p32",
};
#endif

static struct spi_board_info m25p32_spi1_board_info[] __initdata = {
#if defined(CONFIG_MTD_MXC_M25P80) || defined(CONFIG_MTD_MXC_M25P80_MODULE)
	{
		/* the modalias must be the same as spi device driver name */
		.modalias = "m25p32",           /* Name of spi_driver for this device */
		.max_speed_hz = 20000000,       /* max spi SCK clock speed in HZ */
		.bus_num = 1,                   /* Framework bus number */
		.chip_select = 1,               /* Framework chip select. */
		.platform_data = &m25p32_spi_flash_data,
	}
#endif
};

static void spi_device_init(void)
{
	spi_register_board_info(m25p32_spi1_board_info,
				ARRAY_SIZE(m25p32_spi1_board_info));
}

static void mx53_gpio_usbotg_driver_vbus(bool on)
{
	if (on)
		gpio_set_value(MX53_SMD_USB_OTG_PWR_EN, 1);
	else
		gpio_set_value(MX53_SMD_USB_OTG_PWR_EN, 0);
}

static int sdhc_write_protect(struct device *dev)
{
	int ret = 0;

	if (to_platform_device(dev)->id == 0)
		ret = gpio_get_value(MX53_SMD_SD1_WP);

	return ret;
}

static unsigned int sdhc_get_card_det_status(struct device *dev)
{
	int ret = 0;

	if (to_platform_device(dev)->id == 0)
		ret = gpio_get_value(MX53_SMD_SD1_CD);

	return ret;
}

static struct mxc_mmc_platform_data mmc1_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
		| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 0,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};

static struct mxc_mmc_platform_data mmc2_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
		| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 1,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};

static struct mxc_mmc_platform_data mmc3_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
		| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA | MMC_CAP_DATA_DDR,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 1,
	.clock_mmc = "esdhc_clk",
};


static int mxc_sgtl5000_amp_enable(int enable)
{
	gpio_request(MX53_SMD_AUD_AMP_STBY_B, "amp-standby");
	if (enable)
		gpio_direction_output(MX53_SMD_AUD_AMP_STBY_B, 1);
	else
		gpio_direction_output(MX53_SMD_AUD_AMP_STBY_B, 0);
	gpio_free(MX53_SMD_AUD_AMP_STBY_B);
	return 0;
}

static int headphone_det_status(void)
{
	return (gpio_get_value(MX53_SMD_HEADPHONE_DEC) == 0);
}

static int mxc_sgtl5000_init(void);

static struct mxc_audio_platform_data sgtl5000_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 5,
	.hp_irq = gpio_to_irq(MX53_SMD_HEADPHONE_DEC),
	.hp_status = headphone_det_status,
	.amp_enable = mxc_sgtl5000_amp_enable,
	.init = mxc_sgtl5000_init,
};

static int mxc_sgtl5000_init(void)
{
	sgtl5000_data.sysclk = 22579200;
	return 0;
}

static struct platform_device mxc_sgtl5000_device = {
	.name = "imx-3stack-sgtl5000",
};

static struct mxc_asrc_platform_data mxc_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx = 1,
	.spdif_rx = 0,
	.spdif_clk_44100 = 0,	/* Souce from CKIH1 for 44.1K */
	/* Source from CCM spdif_clk (24M) for 48k and 32k
	 * It's not accurate
	 */
	.spdif_clk_48000 = 1,
	.spdif_clkid = 0,
	.spdif_clk = NULL,	/* spdif bus clk */
};

static void mx53_smd_bt_reset(void)
{
	gpio_request(MX53_SMD_BT_RESET, "bt-reset");
	gpio_direction_output(MX53_SMD_BT_RESET, 0);
	/* pull down reset pin at least >5ms */
	mdelay(6);
	/* pull up after power supply BT */
	gpio_set_value(MX53_SMD_BT_RESET, 1);
	gpio_free(MX53_SMD_BT_RESET);
	msleep(100);
	/* Bluetooth need some time to reset */
}

static int mx53_smd_bt_power_change(int status)
{
	if (status)
		mx53_smd_bt_reset();

	return 0;
}

static struct platform_device mxc_bt_rfkill = {
	.name = "mxc_bt_rfkill",
};

static struct mxc_bt_rfkill_platform_data mxc_bt_rfkill_data = {
	.power_change = mx53_smd_bt_power_change,
};

static void mx53_smd_power_off(void)
{
	gpio_request(MX53_SMD_SYS_ON_OFF_CTL, "power-off");
	gpio_set_value(MX53_SMD_SYS_ON_OFF_CTL, 0);
}

#if defined(CONFIG_BATTERY_MAX17085) || defined(CONFIG_BATTERY_MAX17085_MODULE)
static struct resource smd_batt_resource[] = {
	{
	.flags = IORESOURCE_IO,
	.name = "pwr-good",
	.start = MX53_SMD_PWR_GOOD,
	.end = MX53_SMD_PWR_GOOD,
	},
	{
	.flags = IORESOURCE_IO,
	.name = "ac-in",
	.start = MX53_SMD_AC_IN,
	.end = MX53_SMD_AC_IN,
	},
	{
	.flags = IORESOURCE_IO,
	.name = "charge-now",
	.start = MX53_SMD_CHRG_OR_CMOS,
	.end = MX53_SMD_CHRG_OR_CMOS,
	},
	{
	.flags = IORESOURCE_IO,
	.name = "charge-done",
	.start = MX53_SMD_USER_DEG_CHG_NONE,
	.end = MX53_SMD_USER_DEG_CHG_NONE,
	},
};

static struct platform_device smd_battery_device = {
	.name           = "max17085_bat",
	.resource	= smd_batt_resource,
	.num_resources  = ARRAY_SIZE(smd_batt_resource),
};

static void __init smd_add_device_battery(void)
{
	platform_device_register(&smd_battery_device);
}
#else
static void __init smd_add_device_battery(void)
{
}
#endif

/*!
 * Board specific fixup function. It is called by \b setup_arch() in
 * setup.c file very early on during kernel starts. It allows the user to
 * statically fill in the proper values for the passed-in parameters. None of
 * the parameters is used currently.
 *
 * @param  desc         pointer to \b struct \b machine_desc
 * @param  tags         pointer to \b struct \b tag
 * @param  cmdline      pointer to the command line
 * @param  mi           pointer to \b struct \b meminfo
 */
static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	struct tag *t;
	struct tag *mem_tag = 0;
	int total_mem = SZ_1G;
	int left_mem = 0;
	int gpu_mem = SZ_128M;
	int fb_mem = SZ_32M;
	char *str;

	mxc_set_cpu_type(MXC_CPU_MX53);

	for_each_tag(mem_tag, tags) {
		if (mem_tag->hdr.tag == ATAG_MEM) {
			total_mem = mem_tag->u.mem.size;
			left_mem = total_mem - gpu_mem - fb_mem;
			break;
		}
	}

	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "mem=");
			if (str != NULL) {
				str += 4;
				left_mem = memparse(str, &str);
				if (left_mem == 0 || left_mem > total_mem)
					left_mem = total_mem - gpu_mem - fb_mem;
			}

			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpu_memory=");
			if (str != NULL) {
				str += 11;
				gpu_mem = memparse(str, &str);
			}

			break;
		}
	}

	if (mem_tag) {
		fb_mem = total_mem - left_mem - gpu_mem;
		if (fb_mem < 0) {
			gpu_mem = total_mem - left_mem;
			fb_mem = 0;
		}
		mem_tag->u.mem.size = left_mem;

		/*reserve memory for gpu*/
		gpu_device.resource[5].start =
				mem_tag->u.mem.start + left_mem;
		gpu_device.resource[5].end =
				gpu_device.resource[5].start + gpu_mem - 1;
#if defined(CONFIG_FB_MXC_SYNC_PANEL) || \
	defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
		if (fb_mem) {
			mxcfb_resources[0].start =
				gpu_device.resource[5].end + 1;
			mxcfb_resources[0].end =
				mxcfb_resources[0].start + fb_mem - 1;
		} else {
			mxcfb_resources[0].start = 0;
			mxcfb_resources[0].end = 0;
		}
#endif
	}
}

static void __init mx53_smd_io_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx53_smd_pads,
					ARRAY_SIZE(mx53_smd_pads));

	/* SD1 CD */
	gpio_request(MX53_SMD_SD1_CD, "sd1-cd");
	gpio_direction_input(MX53_SMD_SD1_CD);
	/* SD1 WP */
	gpio_request(MX53_SMD_SD1_WP, "sd1-wp");
	gpio_direction_input(MX53_SMD_SD1_WP);

	/* reset FEC PHY */
	gpio_request(MX53_SMD_FEC_RST, "fec-rst");
	gpio_direction_output(MX53_SMD_FEC_RST, 0);
	gpio_set_value(MX53_SMD_FEC_RST, 0);
	msleep(1);
	gpio_set_value(MX53_SMD_FEC_RST, 1);

	/* headphone_det_b */
	gpio_request(MX53_SMD_HEADPHONE_DEC, "headphone-dec");
	gpio_direction_input(MX53_SMD_HEADPHONE_DEC);

	/* USB PWR enable */
	gpio_request(MX53_SMD_USB_OTG_PWR_EN, "usb-pwr");
	gpio_direction_output(MX53_SMD_USB_OTG_PWR_EN, 0);

	/* Enable MX53_SMD_DCDC1V8_EN */
	gpio_request(MX53_SMD_DCDC1V8_EN, "dcdc1v8-en");
	gpio_direction_output(MX53_SMD_DCDC1V8_EN, 1);
	gpio_set_value(MX53_SMD_DCDC1V8_EN, 1);

	/* Enable OSC_CKIH1_EN for audio */
	gpio_request(MX53_SMD_OSC_CKIH1_EN, "osc-en");
	gpio_direction_output(MX53_SMD_OSC_CKIH1_EN, 1);
	gpio_set_value(MX53_SMD_OSC_CKIH1_EN, 1);

	/* Sii902x HDMI controller */
	gpio_request(MX53_SMD_HDMI_RESET_B, "disp0-pwr-en");
	gpio_direction_output(MX53_SMD_HDMI_RESET_B, 0);
	gpio_request(MX53_SMD_HDMI_INT, "disp0-det-int");
	gpio_direction_input(MX53_SMD_HDMI_INT);

	/* MPR121 capacitive button */
	gpio_request(MX53_SMD_KEY_INT, "cap-button-irq");
	gpio_direction_input(MX53_SMD_KEY_INT);
	gpio_free(MX53_SMD_KEY_INT);

	/* Camera reset */
	gpio_request(MX53_SMD_CSI0_RST, "cam-reset");
	gpio_set_value(MX53_SMD_CSI0_RST, 1);

	/* Camera power down */
	gpio_request(MX53_SMD_CSI0_PWN, "cam-pwdn");
	gpio_direction_output(MX53_SMD_CSI0_PWN, 1);
	msleep(1);
	gpio_set_value(MX53_SMD_CSI0_PWN, 0);

	/* Enable WiFi/BT Power*/
	gpio_request(MX53_SMD_WiFi_BT_PWR_EN, "bt-wifi-pwren");
	gpio_direction_output(MX53_SMD_WiFi_BT_PWR_EN, 1);

	/* WiFi Power up sequence */
	gpio_request(MX53_SMD_WLAN_PD, "wifi-pd");
	gpio_direction_output(MX53_SMD_WLAN_PD, 1);
	mdelay(1);
	gpio_set_value(MX53_SMD_WLAN_PD, 0);
	mdelay(5);
	gpio_set_value(MX53_SMD_WLAN_PD, 1);
	gpio_free(MX53_SMD_WLAN_PD);

	/* battery */
	gpio_request(MX53_SMD_AC_IN, "ac-in");
	gpio_direction_input(MX53_SMD_AC_IN);
	gpio_request(MX53_SMD_PWR_GOOD, "pwr-good");
	gpio_direction_input(MX53_SMD_PWR_GOOD);
	gpio_request(MX53_SMD_CHRG_OR_CMOS, "charger now");
	gpio_direction_output(MX53_SMD_CHRG_OR_CMOS, 0);
	gpio_request(MX53_SMD_USER_DEG_CHG_NONE, "charger done");
	gpio_direction_output(MX53_SMD_USER_DEG_CHG_NONE, 0);

	/* ambient light sensor */
	gpio_request(MX53_SMD_ALS_INT, "als int");
	gpio_direction_input(MX53_SMD_ALS_INT);

	gpio_request(MX53_SMD_LCD_PWR_EN, "lcd-pwr-en");
	gpio_direction_output(MX53_SMD_LCD_PWR_EN, 1);
}

/*!
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	mxc_ipu_data.di_clk[0] = clk_get(NULL, "ipu_di0_clk");
	mxc_ipu_data.di_clk[1] = clk_get(NULL, "ipu_di1_clk");
	mxc_ipu_data.csi_clk[0] = clk_get(NULL, "ssi_ext1_clk");
	mxc_spdif_data.spdif_core_clk = clk_get(NULL, "spdif_xtal_clk");
	clk_put(mxc_spdif_data.spdif_core_clk);

	mxcsdhc1_device.resource[2].start = gpio_to_irq(MX53_SMD_SD1_CD);
	mxcsdhc1_device.resource[2].end = gpio_to_irq(MX53_SMD_SD1_CD);

	mxc_cpu_common_init();
	mx53_smd_io_init();

	pm_power_off = mx53_smd_power_off;
	mxc_register_device(&mxc_dma_device, NULL);
	mxc_register_device(&mxc_wdt_device, NULL);
	mxc_register_device(&mxcspi1_device, &mxcspi1_data);
	mxc_register_device(&mxci2c_devices[0], &mxci2c_data);
	mxc_register_device(&mxci2c_devices[1], &mxci2c_data);
	mxc_register_device(&mxci2c_devices[2], &mxci2c_data);
	mx53_smd_init_da9052();

	mxc_register_device(&mxc_rtc_device, NULL);
	mxc_register_device(&mxc_ipu_device, &mxc_ipu_data);
	mxc_register_device(&mxc_ldb_device, &ldb_data);
	mxc_register_device(&mxc_tve_device, &tve_data);
	mxc_register_device(&mxcvpu_device, &mxc_vpu_data);
	mxc_register_device(&gpu_device, &z160_revision);
	mxc_register_device(&mxcscc_device, NULL);
	mxc_register_device(&mxc_dvfs_core_device, &dvfs_core_data);
	mxc_register_device(&busfreq_device, &bus_freq_data);
	mxc_register_device(&mxc_iim_device, &iim_data);
	mxc_register_device(&mxc_pwm2_device, NULL);
	mxc_register_device(&mxc_pwm1_backlight_device, &mxc_pwm_backlight_data);
	mxc_register_device(&mxcsdhc1_device, &mmc1_data);
	mxc_register_device(&mxcsdhc2_device, &mmc2_data);
	mxc_register_device(&mxcsdhc3_device, &mmc3_data);
	mxc_register_device(&mxc_ssi1_device, NULL);
	mxc_register_device(&mxc_ssi2_device, NULL);
	mxc_register_device(&mxc_alsa_spdif_device, &mxc_spdif_data);
	mxc_register_device(&ahci_fsl_device, &sata_data);
	/* AHCI SATA PWR EN(DCDC_5V, DCDC_3V3_BB) on SATA bus */
	gpio_request(MX53_SMD_SATA_PWR_EN, "sata-pwr-en");
	gpio_direction_output(MX53_SMD_SATA_PWR_EN, 1);

	mxc_register_device(&mxc_fec_device, &fec_data);
	mxc_register_device(&mxc_ptp_device, NULL);
	/* ASRC is only available for MX53 TO2.0 */
	if (mx53_revision() >= IMX_CHIP_REVISION_2_0) {
		mxc_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
		clk_put(mxc_asrc_data.asrc_core_clk);
		mxc_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
		clk_put(mxc_asrc_data.asrc_audio_clk);
		mxc_register_device(&mxc_asrc_device, &mxc_asrc_data);
	}

	spi_device_init();

	i2c_register_board_info(0, mxc_i2c0_board_info,
				ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
				ARRAY_SIZE(mxc_i2c2_board_info));

	mxc_register_device(&mxc_sgtl5000_device, &sgtl5000_data);
	mx5_set_otghost_vbus_func(mx53_gpio_usbotg_driver_vbus);
	mx5_usb_dr_init();
	mx5_usbh1_init();
	mxc_register_device(&mxc_v4l2_device, NULL);
	mxc_register_device(&mxc_v4l2out_device, NULL);
	mxc_register_device(&mxc_bt_rfkill, &mxc_bt_rfkill_data);
	smd_add_device_buttons();
	smd_add_device_battery();
}

static void __init mx53_smd_timer_init(void)
{
	struct clk *uart_clk;

	mx53_clocks_init(32768, 24000000, 22579200, 0);

	uart_clk = clk_get_sys("mxcintuart.0", NULL);
	early_console_setup(MX53_BASE_ADDR(UART1_BASE_ADDR), uart_clk);
}

static struct sys_timer mxc_timer = {
	.init	= mx53_smd_timer_init,
};

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX53_SMD data structure.
 */
MACHINE_START(MX53_SMD, "Freescale MX53 SMD Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.fixup = fixup_mxc_board,
	.map_io = mx5_map_io,
	.init_irq = mx5_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END
