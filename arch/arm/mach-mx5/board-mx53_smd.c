/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <linux/init.h>
#include <linux/clk.h>
#include <linux/fec.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c/mpr121_touchkey.h>
#include <linux/fsl_devices.h>
#include <linux/ahci_platform.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_ANDROID_PMEM
#include <linux/android_pmem.h>
#endif
#ifdef CONFIG_ION
#include <linux/ion.h>
#endif
#include <linux/pwm_backlight.h>
#include <linux/mxcfb.h>
#include <linux/ipu.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/mfd/da9052/da9052.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/memblock.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/ipu-v3.h>
#include <mach/imx-uart.h>
#include <mach/iomux-mx53.h>
#include <mach/ahci_sata.h>
#include <mach/imx_rfkill.h>
#include <mach/mxc_asrc.h>
#include <mach/mxc_dvfs.h>
#include <mach/check_fuse.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>

#include "crm_regs.h"
#include "devices-imx53.h"
#include "devices.h"
#include "usb.h"
#include "pmic.h"

/* MX53 SMD GPIO PIN configurations */
#define MX53_SMD_KEY_RESET	IMX_GPIO_NR(1, 2)
#define MX53_SMD_SATA_CLK_GPEN	IMX_GPIO_NR(1, 4)
#define MX53_SMD_PMIC_FAULT	IMX_GPIO_NR(1, 5)
#define MX53_SMD_SYS_ON_OFF_CTL	IMX_GPIO_NR(1, 7)
#define MX53_SMD_PMIC_ON_OFF_REQ	IMX_GPIO_NR(1, 8)

#define MX53_SMD_FEC_INT	IMX_GPIO_NR(2, 4)
#define MX53_SMD_HEADPHONE_DEC	IMX_GPIO_NR(2, 5)
#define MX53_SMD_ZIGBEE_INT	IMX_GPIO_NR(2, 6)
#define MX53_SMD_ZIGBEE_RESET_B	IMX_GPIO_NR(2, 7)
#define MX53_SMD_GPS_RESET_B	IMX_GPIO_NR(2, 12)
#define MX53_SMD_WAKEUP_ZIGBEE	IMX_GPIO_NR(2, 13)
#define MX53_SMD_UI2		IMX_GPIO_NR(2, 14)
#define MX53_SMD_UI1		IMX_GPIO_NR(2, 15)
#define MX53_SMD_FEC_PWR_EN	IMX_GPIO_NR(2, 16)
#define MX53_SMD_LID_OPN_CLS_SW	IMX_GPIO_NR(2, 23)
#define MX53_SMD_GPS_PPS	IMX_GPIO_NR(2, 24)
#define MX53_SMD_ECSPI1_CS0	IMX_GPIO_NR(2, 30)

#define MX53_SMD_DCDC1V8_EN	IMX_GPIO_NR(3, 1)
#define MX53_SMD_AUD_AMP_STBY_B	IMX_GPIO_NR(3, 2)
#define MX53_SMD_SATA_PWR_EN	IMX_GPIO_NR(3, 3)
#define MX53_SMD_TPM_OSC_EN	IMX_GPIO_NR(3, 4)
#define MX53_SMD_WLAN_PD	IMX_GPIO_NR(3, 5)
#define MX53_SMD_WiFi_BT_PWR_EN	IMX_GPIO_NR(3, 10)
#define MX53_SMD_RECOVERY_MODE_SW	IMX_GPIO_NR(3, 11)
#define MX53_SMD_USB_OTG_OC	IMX_GPIO_NR(3, 12)
#define MX53_SMD_SD1_CD         IMX_GPIO_NR(3, 13)
#define MX53_SMD_USB_HUB_RESET_B	IMX_GPIO_NR(3, 14)
#define MX53_SMD_eCOMPASS_INT	IMX_GPIO_NR(3, 15)
#define MX53_SMD_ECSPI1_CS1	IMX_GPIO_NR(3, 19)
#define MX53_SMD_CAP_TCH_INT1	IMX_GPIO_NR(3, 20)
#define MX53_SMD_BT_PRIORITY	IMX_GPIO_NR(3, 21)
#define MX53_SMD_ALS_INT	IMX_GPIO_NR(3, 22)
#define MX53_SMD_TPM_INT	IMX_GPIO_NR(3, 26)
#define MX53_SMD_MODEM_WKUP	IMX_GPIO_NR(3, 27)
#define MX53_SMD_BT_RESET	IMX_GPIO_NR(3, 28)
#define MX53_SMD_TPM_RST_B	IMX_GPIO_NR(3, 29)
#define MX53_SMD_CHRG_OR_CMOS	IMX_GPIO_NR(3, 30)
#define MX53_SMD_CAP_TCH_INT0	IMX_GPIO_NR(3, 31)

#define MX53_SMD_MODEM_DISABLE_B	IMX_GPIO_NR(4, 10)
#define MX53_SMD_SD1_WP         IMX_GPIO_NR(4, 11)
#define MX53_SMD_DCDC5V_BB_EN	IMX_GPIO_NR(4, 14)
#define MX53_SMD_WLAN_HOST_WAKE	IMX_GPIO_NR(4, 15)

#define MX53_SMD_HDMI_RESET_B   IMX_GPIO_NR(5, 0)
#define MX53_SMD_MODEM_RESET_B  IMX_GPIO_NR(5, 2)
#define MX53_SMD_KEY_INT	IMX_GPIO_NR(5, 4)

#define MX53_SMD_CAP_TCH_FUN0	IMX_GPIO_NR(6, 6)
#define MX53_SMD_CSI0_RST       IMX_GPIO_NR(6, 9)
#define MX53_SMD_CSI0_PWN       IMX_GPIO_NR(6, 10)
#define MX53_SMD_OSC_CKIH1_EN	IMX_GPIO_NR(6, 11)
#define MX53_SMD_HDMI_INT	IMX_GPIO_NR(6, 12)
#define MX53_SMD_LCD_PWR_EN	IMX_GPIO_NR(6, 13)
#define MX53_SMD_ACCL_INT1_IN	IMX_GPIO_NR(6, 15)
#define MX53_SMD_ACCL_INT2_IN	IMX_GPIO_NR(6, 16)
#define MX53_SMD_AC_IN		IMX_GPIO_NR(6, 17)
#define MX53_SMD_PWR_GOOD	IMX_GPIO_NR(6, 18)

#define MX53_SMD_CABC_EN0	IMX_GPIO_NR(7, 2)
#define MX53_SMD_DOCK_DECTECT	IMX_GPIO_NR(7, 3)
#define SMD_FEC_PHY_RST		IMX_GPIO_NR(7, 6)
#define MX53_SMD_USER_DEG_CHG_NONE	IMX_GPIO_NR(7, 7)
#define MX53_SMD_OTG_VBUS	IMX_GPIO_NR(7, 8)
#define MX53_SMD_DEVELOP_MODE_SW	IMX_GPIO_NR(7, 9)
#define MX53_SMD_CABC_EN1	IMX_GPIO_NR(7, 10)
#define MX53_SMD_PMIC_INT	IMX_GPIO_NR(7, 11)
#define MX53_SMD_CAP_TCH_FUN1	IMX_GPIO_NR(7, 13)

#define TZIC_WAKEUP0_OFFSET	0x0E00
#define TZIC_WAKEUP1_OFFSET	0x0E04
#define TZIC_WAKEUP2_OFFSET	0x0E08
#define TZIC_WAKEUP3_OFFSET	0x0E0C
#define GPIO7_0_11_IRQ_BIT	(0x1<<11)

void __init early_console_setup(unsigned long base, struct clk *clk);
static struct clk *sata_clk, *sata_ref_clk;
static int fs_in_sdcard;

#ifdef CONFIG_ANDROID_PMEM
extern struct platform_device mxc_android_pmem_device;
extern struct platform_device mxc_android_pmem_gpu_device;
#endif

extern char *lp_reg_id;
extern char *gp_reg_id;
extern void mx5_cpu_regulator_init(void);
extern int mx53_smd_init_da9052(void);

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

	MX53_PAD_GPIO_17__SPDIF_OUT1,
};

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake, debounce_ms) \
{                                                               \
	.gpio           = gpio_num,                             \
	.type           = EV_KEY,                               \
	.code           = ev_code,                              \
	.active_low     = act_low,                              \
	.desc           = "btn " descr,                         \
	.wakeup         = wake,                                 \
	.debounce_interval = debounce_ms,                       \
}

static struct gpio_keys_button smd_buttons[] = {
	GPIO_BUTTON(MX53_SMD_PMIC_ON_OFF_REQ, KEY_POWER, 0, "power", 0, 100),
	GPIO_BUTTON(MX53_SMD_UI1, KEY_VOLUMEUP, 1, "volume-up", 0, 0),
	GPIO_BUTTON(MX53_SMD_UI2, KEY_VOLUMEDOWN, 1, "volume-down", 0, 0),
};

static struct gpio_keys_platform_data smd_button_data = {
	.buttons        = smd_buttons,
	.nbuttons       = ARRAY_SIZE(smd_buttons),
};

static struct platform_device smd_button_device = {
	.name           = "gpio-keys",
	.id             = -1,
	.num_resources  = 0,
	.dev            = {
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

static const struct imxuart_platform_data mx53_smd_uart_data __initconst = {
	.flags = IMXUART_HAVE_RTSCTS,
	.dma_req_rx = MX53_DMA_REQ_UART3_RX,
	.dma_req_tx = MX53_DMA_REQ_UART3_TX,
};

static inline void mx53_smd_init_uart(void)
{
	imx53_add_imx_uart(0, NULL);
	imx53_add_imx_uart(1, NULL);
	imx53_add_imx_uart(2, &mx53_smd_uart_data);
}

static inline void mx53_smd_fec_reset(void)
{
	int ret;

	/* reset FEC PHY */
	ret = gpio_request(SMD_FEC_PHY_RST, "fec-phy-reset");
	if (ret) {
		printk(KERN_ERR"failed to get GPIO_FEC_PHY_RESET: %d\n", ret);
		return;
	}
	gpio_direction_output(SMD_FEC_PHY_RST, 0);
	msleep(1);
	gpio_set_value(SMD_FEC_PHY_RST, 1);
}

static struct fec_platform_data mx53_smd_fec_data = {
	.phy = PHY_INTERFACE_MODE_RMII,
};

static const struct imxi2c_platform_data mx53_smd_i2c_data __initconst = {
	.bitrate = 100000,
};

extern void __iomem *tzic_base;
static void smd_da9053_irq_wakeup_only_fixup(void)
{
	if (NULL == tzic_base) {
		pr_err("fail to map MX53_TZIC_BASE_ADDR\n");
		return;
	}
	__raw_writel(0, tzic_base + TZIC_WAKEUP0_OFFSET);
	__raw_writel(0, tzic_base + TZIC_WAKEUP1_OFFSET);
	__raw_writel(0, tzic_base + TZIC_WAKEUP2_OFFSET);
	/* only enable irq wakeup for da9053 */
	__raw_writel(GPIO7_0_11_IRQ_BIT, tzic_base + TZIC_WAKEUP3_OFFSET);
	pr_info("only da9053 irq is wakeup-enabled\n");
}

static void smd_suspend_enter(void)
{
	if (board_is_rev(IMX_BOARD_REV_4)) {
		smd_da9053_irq_wakeup_only_fixup();
		da9053_suspend_cmd_sw();
	} else {
		if (da9053_get_chip_version() != DA9053_VERSION_BB)
			smd_da9053_irq_wakeup_only_fixup();

		da9053_suspend_cmd_hw();
	}
}

static void smd_suspend_exit(void)
{
	if (da9053_get_chip_version())
		da9053_restore_volt_settings();
}

static struct mxc_pm_platform_data smd_pm_data = {
	.suspend_enter = smd_suspend_enter,
	.suspend_exit = smd_suspend_exit,
};

/* SDIO Card Slot */
static const struct esdhc_platform_data mx53_smd_sd1_data __initconst = {
	.cd_gpio = MX53_SMD_SD1_CD,
	.wp_gpio = MX53_SMD_SD1_WP,
	.keep_power_at_suspend = 1,
	.delay_line = 0,
	.cd_type = ESDHC_CD_CONTROLLER,
};

/* SDIO Wifi */
static const struct esdhc_platform_data mx53_smd_sd2_data __initconst = {
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.delay_line = 0,
	.cd_type = ESDHC_CD_PERMANENT,
};

/* SDIO Internal eMMC */
static const struct esdhc_platform_data mx53_smd_sd3_data __initconst = {
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.support_8bit = 1,
	.delay_line = 0,
	.cd_type = ESDHC_CD_PERMANENT,
};

static void mx53_smd_csi0_cam_powerdown(int powerdown)
{
	struct clk *clk = clk_get(NULL, "ssi_ext1_clk");
	if (!clk)
		printk(KERN_DEBUG "Failed to get ssi_ext1_clk\n");

	if (powerdown) {
		/* Power off */
		gpio_set_value(MX53_SMD_CSI0_PWN, 1);
		if (clk)
			clk_disable(clk);
	} else {
		if (clk)
			clk_enable(clk);
		/* Power Up */
		gpio_set_value(MX53_SMD_CSI0_PWN, 0);
		msleep(2);
	}
}

static void mx53_smd_csi0_io_init(void)
{
	struct clk *clk;
	uint32_t freq = 0;

	clk = clk_get(NULL, "ssi_ext1_clk");
	if (clk) {
		freq = clk_round_rate(clk, 24000000);
		clk_set_rate(clk, freq);
		clk_enable(clk);
	} else
		printk(KERN_DEBUG "Failed to get ssi_ext1_clk\n");

	/* Camera reset */
	gpio_request(MX53_SMD_CSI0_RST, "cam-reset");
	gpio_direction_output(MX53_SMD_CSI0_RST, 1);

	/* Camera power down */
	gpio_request(MX53_SMD_CSI0_PWN, "cam-pwdn");
	gpio_direction_output(MX53_SMD_CSI0_PWN, 1);
	mx53_smd_csi0_cam_powerdown(1);
	msleep(5);
	mx53_smd_csi0_cam_powerdown(0);
	msleep(5);
	gpio_set_value(MX53_SMD_CSI0_RST, 0);
	msleep(1);
	gpio_set_value(MX53_SMD_CSI0_RST, 1);
	msleep(5);
	mx53_smd_csi0_cam_powerdown(1);
}

static struct fsl_mxc_camera_platform_data camera_data = {
	.analog_regulator = "DA9052_LDO7",
	.core_regulator = "DA9052_LDO9",
	.mclk = 24000000,
	.mclk_source = 0,
	.csi = 0,
	.io_init = mx53_smd_csi0_io_init,
	.pwdn = mx53_smd_csi0_cam_powerdown,
};

static struct fsl_mxc_capture_platform_data capture_data = {
	.csi = 0,
	.ipu = 0,
	.mclk_source = 0,
	.is_mipi = 0,
};

static struct fsl_mxc_lightsensor_platform_data ls_data = {
	.rext = 700,    /* calibration: 499K->700K */
};

static int mma8451_position = 4;

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("mma8451", 0x1c),
		.platform_data = (void *)&mma8451_position,
	},
	{
		I2C_BOARD_INFO("ov5642", 0x3c),
		.platform_data = (void *)&camera_data,
	},

};

static unsigned short smd_touchkey_martix[4] = {
	KEY_BACK, KEY_HOME, KEY_MENU, KEY_SEARCH,
};

static struct mpr121_platform_data mpr121_keyboard_platdata = {
	.keymap_size = ARRAY_SIZE(smd_touchkey_martix),
	.vdd_uv = 3300000,
	.keymap = smd_touchkey_martix,
};

static int mag3110_position = 6;

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("sgtl5000", 0x0a)
	},
	{
		I2C_BOARD_INFO("mpr121_touchkey", 0x5a),
		.irq = gpio_to_irq(MX53_SMD_KEY_INT),
		.platform_data = &mpr121_keyboard_platdata,
	},
	{
		I2C_BOARD_INFO("mag3110", 0x0e),
		.irq = gpio_to_irq(MX53_SMD_eCOMPASS_INT),
		.platform_data = (void *)&mag3110_position,
	},
};

static int mx53_smd_spi_cs[] = {
	MX53_SMD_ECSPI1_CS0,
	MX53_SMD_ECSPI1_CS1,
};

static struct spi_imx_master mx53_smd_spi_data = {
	.chipselect = mx53_smd_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx53_smd_spi_cs),
};


#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
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

static struct spi_board_info m25p32_spi0_board_info[] __initdata = {
#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
	{
		/* The modalias must be the same as spi device driver name */
		.modalias = "m25p80",
		.max_speed_hz = 20000000,
		.bus_num = 0,
		.chip_select = 1,
		.platform_data = &m25p32_spi_flash_data,
	}
#endif
};

static void spi_device_init(void)
{
	spi_register_board_info(m25p32_spi0_board_info,
				ARRAY_SIZE(m25p32_spi0_board_info));
}

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


static struct mxc_iim_platform_data iim_data = {
	.bank_start = MXC_IIM_MX53_BANK_START_ADDR,
	.bank_end   = MXC_IIM_MX53_BANK_END_ADDR,
	.enable_fuse = mxc_iim_enable_fuse,
	.disable_fuse = mxc_iim_disable_fuse,
};


static void sii9022_hdmi_reset(void)
{
	gpio_set_value(MX53_SMD_HDMI_RESET_B, 0);
	msleep(10);
	gpio_set_value(MX53_SMD_HDMI_RESET_B, 1);
	msleep(10);
}

static struct fsl_mxc_lcd_platform_data sii902x_hdmi_data = {
	.reset = sii9022_hdmi_reset,
	.analog_reg = "DA9052_LDO2",
};

#ifdef CONFIG_ANDROID_PMEM
static struct android_pmem_platform_data android_pmem_data = {
	.name = "pmem_adsp",
	.size = SZ_64M,
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_gpu_data = {
	.name = "pmem_gpu",
	.size = SZ_64M,
	.cached = 1,
};
#endif

#ifdef CONFIG_ION
#define	ION_VPU	0
#define	ION_GPU	1
static struct ion_platform_data imx_ion_data = {
	.nr = 2,
	.heaps = {
		{
		.id = ION_VPU,
		.type = ION_HEAP_TYPE_CARVEOUT,
		.name = "vpu_ion",
		.size = SZ_64M,
		},
		{
		.id = ION_GPU,
		.type = ION_HEAP_TYPE_CARVEOUT,
		.name = "gpu_ion",
		.size = SZ_64M,
		},
	},
};
#endif

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("sii902x", 0x39),
		.irq = gpio_to_irq(MX53_SMD_HDMI_INT),
		.platform_data = &sii902x_hdmi_data,
	},
	{
		I2C_BOARD_INFO("p1003_fwv33", 0x41),
		.irq  = gpio_to_irq(MX53_SMD_CAP_TCH_INT1),
	},
	{
		I2C_BOARD_INFO("egalax_ts", 0x4),
		.irq = gpio_to_irq(MX53_SMD_CAP_TCH_INT1),
	},
	{
		I2C_BOARD_INFO("isl29023", 0x44),
		.irq  = gpio_to_irq(MX53_SMD_ALS_INT),
		.platform_data = &ls_data,
	},
};

/* HW Initialization, if return 0, initialization is successful. */
static int mx53_smd_sata_init(struct device *dev, void __iomem *addr)
{
	u32 tmpdata;
	int ret = 0;
	struct clk *clk;

	/* Enable SATA PWR  */
	ret = gpio_request(MX53_SMD_SATA_PWR_EN, "ahci-sata-pwr");
	if (ret) {
		printk(KERN_ERR "failed to get SATA_PWR_EN: %d\n", ret);
		return ret;
	}
	gpio_direction_output(MX53_SMD_SATA_PWR_EN, 1);

	sata_clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		dev_err(dev, "no sata clock.\n");
		return PTR_ERR(sata_clk);
	}
	ret = clk_enable(sata_clk);
	if (ret) {
		dev_err(dev, "can't enable sata clock.\n");
		goto put_sata_clk;
	}

	sata_ref_clk = clk_get(NULL, "usb_phy1_clk");
	if (IS_ERR(sata_ref_clk)) {
		dev_err(dev, "no sata ref clock.\n");
		ret = PTR_ERR(sata_ref_clk);
		goto release_sata_clk;
	}
	ret = clk_enable(sata_ref_clk);
	if (ret) {
		dev_err(dev, "can't enable sata ref clock.\n");
		goto put_sata_ref_clk;
	}

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb_clk");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_sata_ref_clk;
	}
	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;

release_sata_ref_clk:
	clk_disable(sata_ref_clk);
put_sata_ref_clk:
	clk_put(sata_ref_clk);
release_sata_clk:
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);

	return ret;
}

static void mx53_smd_sata_exit(struct device *dev)
{
	clk_disable(sata_ref_clk);
	clk_put(sata_ref_clk);

	clk_disable(sata_clk);
	clk_put(sata_clk);
}

static struct ahci_platform_data mx53_smd_sata_data = {
	.init = mx53_smd_sata_init,
	.exit = mx53_smd_sata_exit,
};

static void mx53_smd_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(MX53_SMD_OTG_VBUS, 1);
	else
		gpio_set_value(MX53_SMD_OTG_VBUS, 0);
}

static void __init mx53_smd_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX53_IO_ADDRESS(MX53_OTG_BASE_ADDR);
	ret = gpio_request(MX53_SMD_OTG_VBUS, "usb-pwr");
	if (ret) {
		printk(KERN_ERR"failed to get GPIO SMD_OTG_VBUS: %d\n", ret);
		return;
	}
	gpio_direction_output(MX53_SMD_OTG_VBUS, 0);

	mx5_set_otghost_vbus_func(mx53_smd_usbotg_vbus);
	mx5_usb_dr_init();
	mx5_usbh1_init();
}

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

static struct platform_device imx_bt_rfkill = {
	.name = "imx_bt_rfkill",
};

static struct imx_bt_rfkill_platform_data imx_bt_rfkill_data = {
	.power_change = mx53_smd_bt_power_change,
};

static struct mxc_audio_platform_data smd_audio_data;

static int smd_sgtl5000_init(void)
{
	smd_audio_data.sysclk = 22579200;

	/* Enable OSC_CKIH1_EN for audio */
	gpio_request(MX53_SMD_OSC_CKIH1_EN, "osc-en");
	gpio_direction_output(MX53_SMD_OSC_CKIH1_EN, 1);
	return 0;
}

static int smd_sgtl5000_amp_enable(int enable)
{
	gpio_request(MX53_SMD_AUD_AMP_STBY_B, "amp-standby");
	if (enable)
		gpio_direction_output(MX53_SMD_AUD_AMP_STBY_B, 1);
	else
		gpio_direction_output(MX53_SMD_AUD_AMP_STBY_B, 0);
	gpio_free(MX53_SMD_AUD_AMP_STBY_B);
	return 0;
}

static struct mxc_audio_platform_data smd_audio_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 5,
	.init = smd_sgtl5000_init,
	.amp_enable = smd_sgtl5000_amp_enable,
	.hp_gpio = MX53_SMD_HEADPHONE_DEC,
	.hp_active_low = 1,
};

static struct platform_device smd_audio_device = {
	.name = "imx-sgtl5000",
};

static struct imx_ssi_platform_data smd_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_RGB565,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static struct ipuv3_fb_platform_data smd_fb_data[] = {
	{
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 32,
	.int_clk = false,
	.late_init = false,
	.panel_width_mm = 203,
	.panel_height_mm = 152,
	}, {
	.disp_dev = "sii902x_hdmi",
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	.mode_str = "1024x768M-32@60",
	.default_bpp = 32,
	.int_clk = false,
	.late_init = false,
	},
};

static struct imx_ipuv3_platform_data ipu_data = {
	.rev = 3,
	.csi_clk[0] = "ssi_ext1_clk",
	.bypass_reset = false,
};

static struct platform_pwm_backlight_data mxc_pwm_backlight_data = {
	.pwm_id = 1,
	.max_brightness = 248,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static struct mxc_gpu_platform_data mx53_smd_gpu_pdata __initdata = {
	.enable_mmu = 0,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 0,
	.disp_id = 1,
	.ext_ref = 1,
	.mode = LDB_SIN1,
};

static struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx = 1,
	.spdif_rx = 0,
	.spdif_clk_44100 = 0,	/* Souce from CKIH1 for 44.1K */
	/* Source from CCM spdif_clk (24M) for 48k and 32k
	 * It's not accurate
	 */
	.spdif_clk_48000 = 1,
	.spdif_div_44100 = 8,
	.spdif_div_48000 = 8,
	.spdif_div_32000 = 12,
	.spdif_rx_clk = 0,	/* rx clk from spdif stream */
	.spdif_clk = NULL,	/* spdif bus clk */
};

static struct mxc_dvfs_platform_data smd_dvfs_core_data = {
	.reg_id = "cpu_vddgp",
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

static struct mxc_regulator_platform_data smd_regulator_data = {
	.cpu_reg_id = "cpu_vddgp",
};

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

extern struct imx_mxc_gpu_data imx53_gpu_data;

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	char *str;
	struct tag *t;
	int i = 0;

	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
#ifdef CONFIG_ANDROID_PMEM
			str = t->u.cmdline.cmdline;
			str = strstr(str, "pmem=");
			if (str != NULL) {
				str += 5;
				android_pmem_gpu_data.size =
						memparse(str, &str);
				if (*str == ',') {
					str++;
					android_pmem_data.size =
						memparse(str, &str);
				}
			}
#endif
#ifdef CONFIG_ION
			str = t->u.cmdline.cmdline;
			str = strstr(str, "ion=");
			if (str != NULL) {
				str += 4;
				imx_ion_data.heaps[ION_GPU].size =
						memparse(str, &str);
				if (*str == ',') {
					str++;
					imx_ion_data.heaps[ION_VPU].size =
						memparse(str, &str);
				}
			}
#endif

			str = t->u.cmdline.cmdline;
			str = strstr(str, "fbmem=");
			if (str != NULL) {
				str += 6;
				smd_fb_data[i++].res_size[0] =
						memparse(str, &str);
				while (*str == ',' &&
					i < ARRAY_SIZE(smd_fb_data)) {
					str++;
					smd_fb_data[i++].res_size[0] =
						memparse(str, &str);
				}
			}

			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpu_memory=");
			if (str != NULL) {
				str += 11;
				imx53_gpu_data.gmem_reserved_size =
						memparse(str, &str);
			}

			str = t->u.cmdline.cmdline;
			str = strstr(str, "fs_sdcard=");
			if (str != NULL) {
				str += 10;
				fs_in_sdcard = memparse(str, &str);
			}
			break;
		}
	}
}

static void mx53_smd_power_off(void)
{
	/* Drive DCDC5V_BB_EN low to disable LVDS0/1 power */
	gpio_direction_output(MX53_SMD_DCDC5V_BB_EN, 0);
	/* Drive DCDC1V8_BB_EN low to disable 1V8 voltage */
	gpio_direction_output(MX53_SMD_DCDC1V8_EN, 0);

	/* Disable the Audio AMP to avoid noise after shutdown */
	gpio_request(MX53_SMD_AUD_AMP_STBY_B, "amp-standby");
	gpio_direction_output(MX53_SMD_AUD_AMP_STBY_B, 0);

	/* power off by sending shutdown command to da9053*/
	da9053_power_off();
}

static int __init mx53_smd_power_init(void)
{
	/* cpu get regulator needs to be in lateinit so that
	   regulator list gets updated for i2c da9052 regulators */
	mx5_cpu_regulator_init();

	if (machine_is_mx53_smd())
		pm_power_off = mx53_smd_power_off;

	return 0;
}
late_initcall(mx53_smd_power_init);

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct resource ram_console_resource = {
	.name = "android ram console",
	.flags = IORESOURCE_MEM,
};

static struct platform_device android_ram_console = {
	.name = "ram_console",
	.num_resources = 1,
	.resource = &ram_console_resource,
};

static int __init imx5x_add_ram_console(void)
{
	return platform_device_register(&android_ram_console);
}
#else
#define imx5x_add_ram_console() do {} while (0)
#endif

static void __init mx53_smd_board_init(void)
{
	int i;

	mxc_iomux_v3_setup_multiple_pads(mx53_smd_pads,
					ARRAY_SIZE(mx53_smd_pads));

	/* Enable MX53_SMD_DCDC1V8_EN */
	gpio_request(MX53_SMD_DCDC1V8_EN, "dcdc1v8-en");
	gpio_direction_output(MX53_SMD_DCDC1V8_EN, 1);

	/* Enable MX53_SMD_DCDC5V_EN */
	gpio_request(MX53_SMD_DCDC5V_BB_EN, "dcdc5v_bb_en");
	gpio_direction_output(MX53_SMD_DCDC5V_BB_EN, 1);

	/* Sii902x HDMI controller */
	gpio_request(MX53_SMD_HDMI_RESET_B, "disp0-pwr-en");
	gpio_direction_output(MX53_SMD_HDMI_RESET_B, 0);
	gpio_request(MX53_SMD_HDMI_INT, "disp0-det-int");
	gpio_direction_input(MX53_SMD_HDMI_INT);

	/* MPR121 capacitive button */
	gpio_request(MX53_SMD_KEY_INT, "cap-button-irq");
	gpio_direction_input(MX53_SMD_KEY_INT);
	gpio_free(MX53_SMD_KEY_INT);

	/* Enable WiFi/BT Power*/
	gpio_request(MX53_SMD_WiFi_BT_PWR_EN, "bt-wifi-pwren");
	gpio_direction_output(MX53_SMD_WiFi_BT_PWR_EN, 1);
	gpio_free(MX53_SMD_WiFi_BT_PWR_EN);

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
	gpio_request(MX53_SMD_ALS_INT, "lightsensor");
	gpio_direction_input(MX53_SMD_ALS_INT);

	gpio_request(MX53_SMD_LCD_PWR_EN, "lcd-pwr-en");
	gpio_direction_output(MX53_SMD_LCD_PWR_EN, 1);

	/* mag3110 magnetometer sensor */
	gpio_request(MX53_SMD_eCOMPASS_INT, "ecompass int");
	gpio_direction_input(MX53_SMD_eCOMPASS_INT);

	gp_reg_id = smd_regulator_data.cpu_reg_id;
	lp_reg_id = smd_regulator_data.vcc_reg_id;

	mx53_smd_init_uart();
	imx5x_add_ram_console();
	mx53_smd_fec_reset();
	mxc_register_device(&mxc_pm_device, &smd_pm_data);
	imx53_add_fec(&mx53_smd_fec_data);
	imx53_add_imx2_wdt(0, NULL);
	imx53_add_srtc();
	imx53_add_imx_i2c(0, &mx53_smd_i2c_data);
	imx53_add_imx_i2c(1, &mx53_smd_i2c_data);
	imx53_add_imx_i2c(2, &mx53_smd_i2c_data);
	imx53_add_ecspi(0, &mx53_smd_spi_data);

	imx53_add_ipuv3(0, &ipu_data);
	for (i = 0; i < ARRAY_SIZE(smd_fb_data); i++)
		imx53_add_ipuv3fb(i, &smd_fb_data[i]);
	imx53_add_lcdif(&lcdif_data);
	if (!mxc_fuse_get_vpu_status())
		imx53_add_vpu();
	imx53_add_ldb(&ldb_data);
	imx53_add_v4l2_output(0);
	imx53_add_v4l2_capture(0, &capture_data);


	/*
	 * Disable HannStar touch panel CABC function,
	 * this function turns the panel's backlight automatically
	 * according to the content shown on the panel which
	 * may cause annoying unstable backlight issue.
	 */
	gpio_request(MX53_SMD_CABC_EN0, "cabc-en0");
	gpio_direction_output(MX53_SMD_CABC_EN0, 0);
	gpio_request(MX53_SMD_CABC_EN1, "cabc-en1");
	gpio_direction_output(MX53_SMD_CABC_EN1, 0);

	imx53_add_mxc_pwm(1);
	imx53_add_mxc_pwm_backlight(0, &mxc_pwm_backlight_data);

	if (fs_in_sdcard == 1) {
		imx53_add_sdhci_esdhc_imx(0, &mx53_smd_sd1_data);
		imx53_add_sdhci_esdhc_imx(1, &mx53_smd_sd2_data);
		imx53_add_sdhci_esdhc_imx(2, &mx53_smd_sd3_data);
	} else {
		imx53_add_sdhci_esdhc_imx(2, &mx53_smd_sd3_data);
		imx53_add_sdhci_esdhc_imx(1, &mx53_smd_sd2_data);
		imx53_add_sdhci_esdhc_imx(0, &mx53_smd_sd1_data);
	}

	imx53_add_ahci(0, &mx53_smd_sata_data);
	mxc_register_device(&imx_ahci_device_hwmon, NULL);
	mx53_smd_init_usb();
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx53_add_asrc(&imx_asrc_data);

	imx53_add_iim(&iim_data);
	smd_add_device_buttons();

	mx53_smd_init_da9052();

	spi_device_init();

	i2c_register_board_info(0, mxc_i2c0_board_info,
				ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
				ARRAY_SIZE(mxc_i2c2_board_info));

	mxc_register_device(&imx_bt_rfkill, &imx_bt_rfkill_data);

	imx53_add_imx_ssi(1, &smd_ssi_pdata);

	mxc_register_device(&smd_audio_device, &smd_audio_data);

	mxc_spdif_data.spdif_core_clk = clk_get(NULL, "spdif_xtal_clk");
	clk_put(mxc_spdif_data.spdif_core_clk);
	imx53_add_spdif(&mxc_spdif_data);
	imx53_add_spdif_dai();
	imx53_add_spdif_audio_device();

#ifdef CONFIG_ANDROID_PMEM
	mxc_register_device(&mxc_android_pmem_device, &android_pmem_data);
	mxc_register_device(&mxc_android_pmem_gpu_device,
				&android_pmem_gpu_data);
#endif
#ifdef CONFIG_ION
	imx53_add_ion(0, &imx_ion_data,
		sizeof(imx_ion_data) + (imx_ion_data.nr * sizeof(struct ion_platform_heap)));
#endif

	/*GPU*/
	if (mx53_revision() >= IMX_CHIP_REVISION_2_0)
		mx53_smd_gpu_pdata.z160_revision = 1;
	else
		mx53_smd_gpu_pdata.z160_revision = 0;

	if (!mxc_fuse_get_gpu_status())
		imx53_add_mxc_gpu(&mx53_smd_gpu_pdata);

	/* this call required to release SCC RAM partition held by ROM
	  * during boot, even if SCC2 driver is not part of the image
	  */
	imx53_add_mxc_scc2();
	smd_add_device_battery();
	pm_i2c_init(MX53_I2C1_BASE_ADDR);

	imx53_add_dvfs_core(&smd_dvfs_core_data);
	imx53_add_busfreq();
}

static void __init mx53_smd_timer_init(void)
{
	struct clk *uart_clk;

	mx53_clocks_init(32768, 24000000, 22579200, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(MX53_UART1_BASE_ADDR, uart_clk);
}

static struct sys_timer mx53_smd_timer = {
	.init	= mx53_smd_timer_init,
};

#define SZ_TRIPLE_1080P	ALIGN((1920*ALIGN(1080, 128)*2*3), SZ_4K)
static void __init mx53_smd_reserve(void)
{
	phys_addr_t phys;
	int i;

#ifdef CONFIG_ANDROID_RAM_CONSOLE
	phys = memblock_alloc(SZ_1M, SZ_4K);
	memblock_remove(phys, SZ_1M);
	ram_console_resource.start = phys;
	ram_console_resource.end   = phys + SZ_1M - 1;
#endif

	if (imx53_gpu_data.gmem_reserved_size) {
		phys = memblock_alloc(imx53_gpu_data.gmem_reserved_size,
					   SZ_4K);
		memblock_remove(phys, imx53_gpu_data.gmem_reserved_size);
		imx53_gpu_data.gmem_reserved_base = phys;
	}
#ifdef CONFIG_ANDROID_PMEM
	if (android_pmem_data.size) {
		phys = memblock_alloc(android_pmem_data.size, SZ_4K);
		memblock_remove(phys, android_pmem_data.size);
		android_pmem_data.start = phys;
	}

	if (android_pmem_gpu_data.size) {
		phys = memblock_alloc(android_pmem_gpu_data.size, SZ_4K);
		memblock_remove(phys, android_pmem_gpu_data.size);
		android_pmem_gpu_data.start = phys;
	}
#endif
#ifdef CONFIG_ION
	if (imx_ion_data.heaps[ION_VPU].size) {
		phys = memblock_alloc(imx_ion_data.heaps[ION_VPU].size, SZ_4K);
		memblock_remove(phys, imx_ion_data.heaps[ION_VPU].size);
		imx_ion_data.heaps[ION_VPU].base = phys;
	}

	if (imx_ion_data.heaps[ION_GPU].size) {
		phys = memblock_alloc(imx_ion_data.heaps[ION_GPU].size, SZ_4K);
		memblock_remove(phys, imx_ion_data.heaps[ION_GPU].size);
		imx_ion_data.heaps[ION_GPU].base = phys;
	}
#endif

	for (i = 0; i < ARRAY_SIZE(smd_fb_data); i++)
		if (smd_fb_data[i].res_size[0]) {
			/* reserve for background buffer */
			phys = memblock_alloc(smd_fb_data[i].res_size[0],
						SZ_4K);
			memblock_remove(phys, smd_fb_data[i].res_size[0]);
			smd_fb_data[i].res_base[0] = phys;

			/* reserve for overlay buffer */
			phys = memblock_alloc(SZ_TRIPLE_1080P, SZ_4K);
			memblock_remove(phys, SZ_TRIPLE_1080P);
			smd_fb_data[i].res_base[1] = phys;
			smd_fb_data[i].res_size[1] = SZ_TRIPLE_1080P;
		}
}

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX53_SMD data structure.
 */
MACHINE_START(MX53_SMD, "Freescale iMX53 SMD Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.fixup = fixup_mxc_board,
	.map_io = mx53_map_io,
	.init_early = imx53_init_early,
	.init_irq = mx53_init_irq,
	.timer = &mx53_smd_timer,
	.init_machine = mx53_smd_board_init,
	.reserve = mx53_smd_reserve,
MACHINE_END
