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
#include <linux/powerkey.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/keypad.h>
#include <asm/mach/flash.h>
#include <mach/memory.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/mxc_dvfs.h>
#include <mach/iomux-mx53.h>
#include <mach/i2c.h>
#include <mach/mxc_iim.h>

#include "crm_regs.h"
#include "devices.h"
#include "usb.h"

#define ARM2_SD1_CD			(0*32 + 1)	/* GPIO_1_1 */

#define MX53_HP_DETECT			(1*32 + 5)	/* GPIO_2_5 */

#define EVK_SD3_CD			(2*32 + 11)	/* GPIO_3_11 */
#define EVK_SD3_WP			(2*32 + 12)	/* GPIO_3_12 */
#define EVK_SD1_CD			(2*32 + 13)	/* GPIO_3_13 */
#define EVK_SD1_WP			(2*32 + 14)	/* GPIO_3_14 */
#define ARM2_OTG_VBUS			(2*32 + 22)	/* GPIO_3_22 */
#define MX53_DVI_PD			(2*32 + 24)	/* GPIO_3_24 */
#define EVK_TS_INT			(2*32 + 26)	/* GPIO_3_26 */
#define MX53_DVI_I2C			(2*32 + 28)	/* GPIO_3_28 */
#define MX53_DVI_DETECT		(2*32 + 31)	/* GPIO_3_31 */

#define MX53_CAM_RESET			(3*32 + 0)	/* GPIO_4_0 */
#define MX53_ESAI_RESET			(3*32 + 2)	/* GPIO_4_2 */
#define MX53_CAN2_EN2			(3*32 + 4)	/* GPIO_4_4 */
#define MX53_12V_EN			(3*32 + 5)	/* GPIO_4_5 */
#define ARM2_LCD_CONTRAST		(3*32 + 20)	/* GPIO_4_20 */

#define MX53_DVI_RESET			(4*32 + 0)	/* GPIO_5_0 */
#define EVK_USB_HUB_RESET		(4*32 + 20)	/* GPIO_5_20 */
#define MX53_TVIN_PWR			(4*32 + 23)	/* GPIO_5_23 */
#define MX53_CAN2_EN1			(4*32 + 24)	/* GPIO_5_24 */
#define MX53_TVIN_RESET			(4*32 + 25)	/* GPIO_5_25 */

#define EVK_OTG_VBUS			(5*32 + 6)	/* GPIO_6_6 */

#define EVK_FEC_PHY_RESET		(6*32 + 6)	/* GPIO_7_6 */
#define EVK_USBH1_VBUS			(6*32 + 8)	/* GPIO_7_8 */
#define MX53_PMIC_INT			(6*32 + 11)	/* GPIO_7_11 */
#define MX53_CAN1_EN1			(6*32 + 12)	/* GPIO_7_12 */
#define MX53_CAN1_EN2			(6*32 + 13)	/* GPIO_7_13 */

/*!
 * @file mach-mx53/mx53_evk.c
 *
 * @brief This file contains the board specific initialization routines.
 *
 * @ingroup MSL_MX53
 */
extern int __init mx53_evk_init_mc13892(void);
extern struct cpu_wp *(*get_cpu_wp)(int *wp);
extern void (*set_num_cpu_wp)(int num);
static int num_cpu_wp = 3;

static struct pad_desc mx53common_pads[] = {
	MX53_PAD_EIM_WAIT__GPIO_5_0,

	MX53_PAD_EIM_OE__DI1_PIN7,
	MX53_PAD_EIM_RW__DI1_PIN8,

	MX53_PAD_EIM_A25__DI0_D1_CS,

	MX53_PAD_EIM_D16__CSPI1_SCLK,
	MX53_PAD_EIM_D17__CSPI1_MISO,
	MX53_PAD_EIM_D18__CSPI1_MOSI,

	MX53_PAD_EIM_D20__SER_DISP0_CS,

	MX53_PAD_EIM_D23__DI0_D0_CS,

	MX53_PAD_EIM_D24__GPIO_3_24,
	MX53_PAD_EIM_D26__GPIO_3_26,

	MX53_PAD_EIM_D29__DISPB0_SER_RS,

	MX53_PAD_EIM_D30__DI0_PIN11,
	MX53_PAD_EIM_D31__DI0_PIN12,

	MX53_PAD_ATA_DA_1__GPIO_7_7,
	MX53_PAD_ATA_DATA4__GPIO_2_4,
	MX53_PAD_ATA_DATA5__GPIO_2_5,
	MX53_PAD_ATA_DATA6__GPIO_2_6,

	MX53_PAD_SD2_CLK__SD2_CLK,
	MX53_PAD_SD2_CMD__SD2_CMD,
	MX53_PAD_SD2_DATA0__SD2_DAT0,
	MX53_PAD_SD2_DATA1__SD2_DAT1,
	MX53_PAD_SD2_DATA2__SD2_DAT2,
	MX53_PAD_SD2_DATA3__SD2_DAT3,
	MX53_PAD_ATA_DATA12__SD2_DAT4,
	MX53_PAD_ATA_DATA13__SD2_DAT5,
	MX53_PAD_ATA_DATA14__SD2_DAT6,
	MX53_PAD_ATA_DATA15__SD2_DAT7,

	MX53_PAD_CSI0_D10__UART1_TXD,
	MX53_PAD_CSI0_D11__UART1_RXD,

	MX53_PAD_ATA_BUFFER_EN__UART2_RXD,
	MX53_PAD_ATA_DMARQ__UART2_TXD,
	MX53_PAD_ATA_DIOR__UART2_RTS,
	MX53_PAD_ATA_INTRQ__UART2_CTS,

	MX53_PAD_ATA_CS_0__UART3_TXD,
	MX53_PAD_ATA_CS_1__UART3_RXD,

	MX53_PAD_KEY_COL0__AUD5_TXC,
	MX53_PAD_KEY_ROW0__AUD5_TXD,
	MX53_PAD_KEY_COL1__AUD5_TXFS,
	MX53_PAD_KEY_ROW1__AUD5_RXD,

	MX53_PAD_CSI0_D7__GPIO_5_25,

	MX53_PAD_GPIO_2__MLBDAT,
	MX53_PAD_GPIO_3__MLBCLK,

	MX53_PAD_GPIO_6__MLBSIG,

	MX53_PAD_GPIO_4__GPIO_1_4,
	MX53_PAD_GPIO_7__GPIO_1_7,
	MX53_PAD_GPIO_8__GPIO_1_8,

	MX53_PAD_GPIO_10__GPIO_4_0,

	MX53_PAD_KEY_COL2__TXCAN1,
	MX53_PAD_KEY_ROW2__RXCAN1,

	/* CAN1 -- EN */
	MX53_PAD_GPIO_18__GPIO_7_13,
	/* CAN1 -- STBY */
	MX53_PAD_GPIO_17__GPIO_7_12,
	/* CAN1 -- NERR */
	MX53_PAD_GPIO_5__GPIO_1_5,

	MX53_PAD_KEY_COL4__TXCAN2,
	MX53_PAD_KEY_ROW4__RXCAN2,

	/* CAN2 -- EN */
	MX53_PAD_CSI0_D6__GPIO_5_24,
	/* CAN2 -- STBY */
	MX53_PAD_GPIO_14__GPIO_4_4,
	/* CAN2 -- NERR */
	MX53_PAD_CSI0_D4__GPIO_5_22,

	MX53_PAD_GPIO_11__GPIO_4_1,
	MX53_PAD_GPIO_12__GPIO_4_2,
	MX53_PAD_GPIO_13__GPIO_4_3,
	MX53_PAD_GPIO_16__GPIO_7_11,
	MX53_PAD_GPIO_19__GPIO_4_5,

	/* DI0 display clock */
	MX53_PAD_DI0_DISP_CLK__DI0_DISP_CLK,

	/* DI0 data enable */
	MX53_PAD_DI0_PIN15__DI0_PIN15,
	/* DI0 HSYNC */
	MX53_PAD_DI0_PIN2__DI0_PIN2,
	/* DI0 VSYNC */
	MX53_PAD_DI0_PIN3__DI0_PIN3,

	MX53_PAD_DISP0_DAT0__DISP0_DAT0,
	MX53_PAD_DISP0_DAT1__DISP0_DAT1,
	MX53_PAD_DISP0_DAT2__DISP0_DAT2,
	MX53_PAD_DISP0_DAT3__DISP0_DAT3,
	MX53_PAD_DISP0_DAT4__DISP0_DAT4,
	MX53_PAD_DISP0_DAT5__DISP0_DAT5,
	MX53_PAD_DISP0_DAT6__DISP0_DAT6,
	MX53_PAD_DISP0_DAT7__DISP0_DAT7,
	MX53_PAD_DISP0_DAT8__DISP0_DAT8,
	MX53_PAD_DISP0_DAT9__DISP0_DAT9,
	MX53_PAD_DISP0_DAT10__DISP0_DAT10,
	MX53_PAD_DISP0_DAT11__DISP0_DAT11,
	MX53_PAD_DISP0_DAT12__DISP0_DAT12,
	MX53_PAD_DISP0_DAT13__DISP0_DAT13,
	MX53_PAD_DISP0_DAT14__DISP0_DAT14,
	MX53_PAD_DISP0_DAT15__DISP0_DAT15,
	MX53_PAD_DISP0_DAT16__DISP0_DAT16,
	MX53_PAD_DISP0_DAT17__DISP0_DAT17,
	MX53_PAD_DISP0_DAT18__DISP0_DAT18,
	MX53_PAD_DISP0_DAT19__DISP0_DAT19,
	MX53_PAD_DISP0_DAT20__DISP0_DAT20,
	MX53_PAD_DISP0_DAT21__DISP0_DAT21,
	MX53_PAD_DISP0_DAT22__DISP0_DAT22,
	MX53_PAD_DISP0_DAT23__DISP0_DAT23,

	MX53_PAD_LVDS0_TX3_P__LVDS0_TX3,
	MX53_PAD_LVDS0_CLK_P__LVDS0_CLK,
	MX53_PAD_LVDS0_TX2_P__LVDS0_TX2,
	MX53_PAD_LVDS0_TX1_P__LVDS0_TX1,
	MX53_PAD_LVDS0_TX0_P__LVDS0_TX0,

	MX53_PAD_LVDS1_TX3_P__LVDS1_TX3,
	MX53_PAD_LVDS1_CLK_P__LVDS1_CLK,
	MX53_PAD_LVDS1_TX2_P__LVDS1_TX2,
	MX53_PAD_LVDS1_TX1_P__LVDS1_TX1,
	MX53_PAD_LVDS1_TX0_P__LVDS1_TX0,

	/* audio and CSI clock out */
	MX53_PAD_GPIO_0__SSI_EXT1_CLK,

	MX53_PAD_CSI0_D12__CSI0_D12,
	MX53_PAD_CSI0_D13__CSI0_D13,
	MX53_PAD_CSI0_D14__CSI0_D14,
	MX53_PAD_CSI0_D15__CSI0_D15,
	MX53_PAD_CSI0_D16__CSI0_D16,
	MX53_PAD_CSI0_D17__CSI0_D17,
	MX53_PAD_CSI0_D18__CSI0_D18,
	MX53_PAD_CSI0_D19__CSI0_D19,

	MX53_PAD_CSI0_VSYNC__CSI0_VSYNC,
	MX53_PAD_CSI0_MCLK__CSI0_HSYNC,
	MX53_PAD_CSI0_PIXCLK__CSI0_PIXCLK,
	/* Camera low power */
	MX53_PAD_CSI0_D5__GPIO_5_23,

	/* esdhc1 */
	MX53_PAD_SD1_CMD__SD1_CMD,
	MX53_PAD_SD1_CLK__SD1_CLK,
	MX53_PAD_SD1_DATA0__SD1_DATA0,
	MX53_PAD_SD1_DATA1__SD1_DATA1,
	MX53_PAD_SD1_DATA2__SD1_DATA2,
	MX53_PAD_SD1_DATA3__SD1_DATA3,

	/* esdhc3 */
	MX53_PAD_ATA_DATA8__SD3_DAT0,
	MX53_PAD_ATA_DATA9__SD3_DAT1,
	MX53_PAD_ATA_DATA10__SD3_DAT2,
	MX53_PAD_ATA_DATA11__SD3_DAT3,
	MX53_PAD_ATA_DATA0__SD3_DAT4,
	MX53_PAD_ATA_DATA1__SD3_DAT5,
	MX53_PAD_ATA_DATA2__SD3_DAT6,
	MX53_PAD_ATA_DATA3__SD3_DAT7,
	MX53_PAD_ATA_RESET_B__SD3_CMD,
	MX53_PAD_ATA_IORDY__SD3_CLK,

	/* FEC pins */
	MX53_PAD_FEC_MDIO__FEC_MDIO,
	MX53_PAD_FEC_REF_CLK__FEC_REF_CLK,
	MX53_PAD_FEC_RX_ER__FEC_RX_ER,
	MX53_PAD_FEC_CRS_DV__FEC_CRS_DV,
	MX53_PAD_FEC_RXD1__FEC_RXD1,
	MX53_PAD_FEC_RXD0__FEC_RXD0,
	MX53_PAD_FEC_TX_EN__FEC_TX_EN,
	MX53_PAD_FEC_TXD1__FEC_TXD1,
	MX53_PAD_FEC_TXD0__FEC_TXD0,
	MX53_PAD_FEC_MDC__FEC_MDC,

	MX53_PAD_CSI0_D8__I2C1_SDA,
	MX53_PAD_CSI0_D9__I2C1_SCL,

	MX53_PAD_KEY_COL3__I2C2_SCL,
	MX53_PAD_KEY_ROW3__I2C2_SDA,
};

static struct pad_desc mx53evk_pads[] = {
	/* USB OTG USB_OC */
	MX53_PAD_EIM_A24__GPIO_5_4,

	/* USB OTG USB_PWR */
	MX53_PAD_EIM_A23__GPIO_6_6,

	/* DISPB0_SER_CLK */
	MX53_PAD_EIM_D21__DISPB0_SER_CLK,

	/* DI0_PIN1 */
	MX53_PAD_EIM_D22__DISPB0_SER_DIN,

	/* DVI I2C ENABLE */
	MX53_PAD_EIM_D28__GPIO_3_28,

	/* DVI DET */
	MX53_PAD_EIM_D31__GPIO_3_31,

	/* SDHC1 SD_CD */
	MX53_PAD_EIM_DA13__GPIO_3_13,

	/* SDHC1 SD_WP */
	MX53_PAD_EIM_DA14__GPIO_3_14,

	/* SDHC3 SD_CD */
	MX53_PAD_EIM_DA11__GPIO_3_11,

	/* SDHC3 SD_WP */
	MX53_PAD_EIM_DA12__GPIO_3_12,

	/* PWM backlight */
	MX53_PAD_GPIO_1__PWMO,

	/* USB HOST USB_PWR */
	MX53_PAD_ATA_DA_2__GPIO_7_8,

	/* USB HOST USB_RST */
	MX53_PAD_CSI0_DATA_EN__GPIO_5_20,

	/* USB HOST CARD_ON */
	MX53_PAD_EIM_DA15__GPIO_3_15,

	/* USB HOST CARD_RST */
	MX53_PAD_ATA_DATA7__GPIO_2_7,

	/* USB HOST WAN_WAKE */
	MX53_PAD_EIM_D25__GPIO_3_25,

	/* FEC_RST */
	MX53_PAD_ATA_DA_0__GPIO_7_6,
};

static struct pad_desc mx53arm2_pads[] = {
	/* USB OTG USB_OC */
	MX53_PAD_EIM_D21__GPIO_3_21,

	/* USB OTG USB_PWR */
	MX53_PAD_EIM_D22__GPIO_3_22,

	/* SDHC1 SD_CD */
	MX53_PAD_GPIO_1__GPIO_1_1,

	/* gpio backlight */
	MX53_PAD_DI0_PIN4__GPIO_4_20,
};

static struct pad_desc mx53_nand_pads[] = {
	MX53_PAD_NANDF_CLE__NANDF_CLE,
	MX53_PAD_NANDF_ALE__NANDF_ALE,
	MX53_PAD_NANDF_WP_B__NANDF_WP_B,
	MX53_PAD_NANDF_WE_B__NANDF_WE_B,
	MX53_PAD_NANDF_RE_B__NANDF_RE_B,
	MX53_PAD_NANDF_RB0__NANDF_RB0,
	MX53_PAD_NANDF_CS0__NANDF_CS0,
	MX53_PAD_NANDF_CS1__NANDF_CS1	,
	MX53_PAD_NANDF_CS2__NANDF_CS2,
	MX53_PAD_NANDF_CS3__NANDF_CS3	,
	MX53_PAD_EIM_DA0__EIM_DA0,
	MX53_PAD_EIM_DA1__EIM_DA1,
	MX53_PAD_EIM_DA2__EIM_DA2,
	MX53_PAD_EIM_DA3__EIM_DA3,
	MX53_PAD_EIM_DA4__EIM_DA4,
	MX53_PAD_EIM_DA5__EIM_DA5,
	MX53_PAD_EIM_DA6__EIM_DA6,
	MX53_PAD_EIM_DA7__EIM_DA7,
};

/* working point(wp): 0 - 800MHz; 1 - 166.25MHz; */
static struct cpu_wp cpu_wp_auto[] = {
	{
	 .pll_rate = 1000000000,
	 .cpu_rate = 1000000000,
	 .pdf = 0,
	 .mfi = 10,
	 .mfd = 11,
	 .mfn = 5,
	 .cpu_podf = 0,
	 .cpu_voltage = 1150000,},
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 800000000,
	 .pdf = 0,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 0,
	 .cpu_voltage = 1050000,},
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 160000000,
	 .pdf = 4,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 4,
	 .cpu_voltage = 850000,},
};

static struct fb_videomode video_modes[] = {
	{
	 /* NTSC TV output */
	 "TV-NTSC", 60, 720, 480, 74074,
	 122, 15,
	 18, 26,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_INTERLACED,
	 0,},
	{
	 /* PAL TV output */
	 "TV-PAL", 50, 720, 576, 74074,
	 132, 11,
	 22, 26,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_INTERLACED | FB_VMODE_ODD_FLD_FIRST,
	 0,},
	{
	 /* 1080i50 TV output */
	 "1080I50", 50, 1920, 1080, 13468,
	 192, 527,
	 20, 24,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_INTERLACED | FB_VMODE_ODD_FLD_FIRST,
	 0,},
	{
	 /* 1080i60 TV output */
	 "1080I60", 60, 1920, 1080, 13468,
	 192, 87,
	 20, 24,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_INTERLACED | FB_VMODE_ODD_FLD_FIRST,
	 0,},
	{
	 /* 800x480 @ 57 Hz , pixel clk @ 27MHz */
	 "CLAA-WVGA", 57, 800, 480, 37037, 40, 60, 10, 10, 20, 10,
	 FB_SYNC_CLK_LAT_FALL,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	 "XGA", 60, 1024, 768, 15385,
	 220, 40,
	 21, 7,
	 60, 10,
	 0,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	 /* 720p30 TV output */
	 "720P30", 30, 1280, 720, 13468,
	 260, 1759,
	 25, 4,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	 "720P60", 60, 1280, 720, 13468,
	 260, 109,
	 25, 4,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	/* VGA 1280x1024 108M pixel clk output */
	"SXGA", 60, 1280, 1024, 9259,
	48, 248,
	1, 38,
	112, 3,
	0,
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
	{
	 /* 1080p24 TV output */
	 "1080P24", 24, 1920, 1080, 13468,
	 192, 637,
	 38, 6,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	 /* 1080p25 TV output */
	 "1080P25", 25, 1920, 1080, 13468,
	 192, 527,
	 38, 6,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	 /* 1080p30 TV output */
	 "1080P30", 30, 1920, 1080, 13468,
	 192, 87,
	 38, 6,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	 "1080P60", 60, 1920, 1080, 7692,
	 100, 40,
	 30, 3,
	 10, 2,
	 0,
	 FB_VMODE_NONINTERLACED,
	 0,},
};

struct cpu_wp *mx53_evk_get_cpu_wp(int *wp)
{
	*wp = num_cpu_wp;
	return cpu_wp_auto;
}

void mx53_evk_set_num_cpu_wp(int num)
{
	num_cpu_wp = num;
	return;
}

static struct mxc_w1_config mxc_w1_data = {
	.search_rom_accelerator = 1,
};

static struct platform_pwm_backlight_data mxc_pwm_backlight_data = {
	.pwm_id = 1,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static void flexcan_xcvr_enable(int id, int en)
{
	static int pwdn;
	if (id < 0 || id > 1)
		return;

	if (en) {
		if (!(pwdn++))
			gpio_set_value(MX53_12V_EN, 1);

		if (id == 0) {
			gpio_set_value(MX53_CAN1_EN1, 1);
			gpio_set_value(MX53_CAN1_EN2, 1);
		} else {
			gpio_set_value(MX53_CAN2_EN1, 1);
			gpio_set_value(MX53_CAN2_EN2, 1);
		}

	} else {
		if (!(--pwdn))
			gpio_set_value(MX53_12V_EN, 0);

		if (id == 0) {
			gpio_set_value(MX53_CAN1_EN1, 0);
			gpio_set_value(MX53_CAN1_EN2, 0);
		} else {
			gpio_set_value(MX53_CAN2_EN1, 0);
			gpio_set_value(MX53_CAN2_EN2, 0);
		}
	}
}

static struct flexcan_platform_data flexcan0_data = {
	.core_reg = NULL,
	.io_reg = NULL,
	.root_clk_id = "lp_apm", /*lp_apm is 24MHz */
	.xcvr_enable = flexcan_xcvr_enable,
	.br_clksrc = 0,
	.br_rjw = 2,
	.br_presdiv = 3,
	.br_propseg = 2,
	.br_pseg1 = 3,
	.br_pseg2 = 3,
	.bcc = 1,
	.srx_dis = 1,
	.smp = 1,
	.boff_rec = 1,
	.ext_msg = 1,
	.std_msg = 1,
};
static struct flexcan_platform_data flexcan1_data = {
	.core_reg = NULL,
	.io_reg = NULL,
	.root_clk_id = "lp_apm", /*lp_apm is 24MHz */
	.xcvr_enable = flexcan_xcvr_enable,
	.br_clksrc = 0,
	.br_rjw = 2,
	.br_presdiv = 3,
	.br_propseg = 2,
	.br_pseg1 = 3,
	.br_pseg2 = 3,
	.bcc = 1,
	.srx_dis = 1,
	.boff_rec = 1,
	.ext_msg = 1,
	.std_msg = 1,
};


extern void mx5_ipu_reset(void);
static struct mxc_ipu_config mxc_ipu_data = {
	.rev = 3,
	.reset = mx5_ipu_reset,
};

extern void mx5_vpu_reset(void);
static struct mxc_vpu_platform_data mxc_vpu_data = {
	.reset = mx5_vpu_reset,
};

static struct fec_platform_data fec_data = {
	.phy = PHY_INTERFACE_MODE_RMII,
};

/* workaround for ecspi chipselect pin may not keep correct level when idle */
static void mx53_evk_gpio_spi_chipselect_active(int cspi_mode, int status,
					     int chipselect)
{
	switch (cspi_mode) {
	case 1:
		switch (chipselect) {
		case 0x1:
			{
			struct pad_desc eim_d19_gpio = MX53_PAD_EIM_D19__GPIO_3_19;
			struct pad_desc cspi_ss0 = MX53_PAD_EIM_EB2__CSPI_SS0;

			/* de-select SS1 of instance: ecspi1. */
			mxc_iomux_v3_setup_pad(&eim_d19_gpio);
			mxc_iomux_v3_setup_pad(&cspi_ss0);
			}
			break;
		case 0x2:
			{
			struct pad_desc eim_eb2_gpio = MX53_PAD_EIM_EB2__GPIO_2_30;
			struct pad_desc cspi_ss1 = MX53_PAD_EIM_D19__CSPI_SS1;

			/* de-select SS0 of instance: ecspi1. */
			mxc_iomux_v3_setup_pad(&eim_eb2_gpio);
			mxc_iomux_v3_setup_pad(&cspi_ss1);
			}
			break;
		default:
			break;
		}
		break;
	case 2:
		break;
	case 3:
		break;
	default:
		break;
	}
}

static void mx53_evk_gpio_spi_chipselect_inactive(int cspi_mode, int status,
					       int chipselect)
{
	switch (cspi_mode) {
	case 1:
		switch (chipselect) {
		case 0x1:
			break;
		case 0x2:
			break;
		default:
			break;
		}
		break;
	case 2:
		break;
	case 3:
		break;
	default:
		break;
	}
}

static struct mxc_spi_master mxcspi1_data = {
	.maxchipselect = 4,
	.spi_version = 23,
	.chipselect_active = mx53_evk_gpio_spi_chipselect_active,
	.chipselect_inactive = mx53_evk_gpio_spi_chipselect_inactive,
};

static struct imxi2c_platform_data mxci2c_data = {
	.bitrate = 100000,
};

static struct mxc_srtc_platform_data srtc_data = {
	.srtc_sec_mode_addr = 0x83F98840,
};

static struct mxc_dvfs_platform_data dvfs_core_data = {
	.reg_id = "SW1",
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
	.num_wp = 3,
};

static struct mxc_bus_freq_platform_data bus_freq_data = {
	.gp_reg_id = "SW1",
	.lp_reg_id = "SW2",
};

static struct tve_platform_data tve_data = {
	.dac_reg = "VVIDEO",
};

static struct ldb_platform_data ldb_data = {
	.lvds_bg_reg = "VAUDIO",
	.ext_ref = 1,
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

static struct pad_desc mx53esai_pads[] = {
	MX53_PAD_FEC_MDIO__ESAI_SCKR,
	MX53_PAD_FEC_REF_CLK__ESAI_FSR,
	MX53_PAD_FEC_RX_ER__ESAI_HCKR,
	MX53_PAD_FEC_CRS_DV__ESAI_SCKT,
	MX53_PAD_FEC_RXD1__ESAI_FST,
	MX53_PAD_FEC_RXD0__ESAI_HCKT,
	MX53_PAD_FEC_TX_EN__ESAI_TX3_RX2,
	MX53_PAD_FEC_TXD1__ESAI_TX2_RX3,
	MX53_PAD_FEC_TXD0__ESAI_TX4_RX1,
	MX53_PAD_FEC_MDC__ESAI_TX5_RX0,
	MX53_PAD_NANDF_CS2__ESAI_TX0,
	MX53_PAD_NANDF_CS3__ESAI_TX1,
};

void gpio_activate_esai_ports(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx53esai_pads,
					ARRAY_SIZE(mx53esai_pads));
}

static struct mxc_esai_platform_data esai_data = {
	.activate_esai_ports = gpio_activate_esai_ports,
};

void gpio_cs42888_pdwn(int pdwn)
{
	if (pdwn)
		gpio_set_value(MX53_ESAI_RESET, 0);
	else
		gpio_set_value(MX53_ESAI_RESET, 1);
}

static void gpio_usbotg_vbus_active(void)
{
	if (board_is_mx53_arm2()) {
		/* MX53 ARM2 CPU board */
		/* Enable OTG VBus with GPIO low */
		gpio_set_value(ARM2_OTG_VBUS, 0);
	} else  if (board_is_mx53_evk_a()) {
		/* MX53 EVK board ver A*/
		/* Enable OTG VBus with GPIO low */
		gpio_set_value(EVK_OTG_VBUS, 0);
	} else  if (board_is_mx53_evk_b()) {
		/* MX53 EVK board ver B*/
		/* Enable OTG VBus with GPIO high */
		gpio_set_value(EVK_OTG_VBUS, 1);
	}
}

static void gpio_usbotg_vbus_inactive(void)
{
	if (board_is_mx53_arm2()) {
		/* MX53 ARM2 CPU board */
		/* Disable OTG VBus with GPIO high */
		gpio_set_value(ARM2_OTG_VBUS, 1);
	} else  if (board_is_mx53_evk_a()) {
		/* MX53 EVK board ver A*/
		/* Disable OTG VBus with GPIO high */
		gpio_set_value(EVK_OTG_VBUS, 1);
	} else  if (board_is_mx53_evk_b()) {
		/* MX53 EVK board ver B*/
		/* Disable OTG VBus with GPIO low */
		gpio_set_value(EVK_OTG_VBUS, 0);
	}
}

static void mx53_gpio_usbotg_driver_vbus(bool on)
{
	if (on)
		gpio_usbotg_vbus_active();
	else
		gpio_usbotg_vbus_inactive();
}

static void mx53_gpio_host1_driver_vbus(bool on)
{
	if (on)
		gpio_set_value(EVK_USBH1_VBUS, 1);
	else
		gpio_set_value(EVK_USBH1_VBUS, 0);
}

static void adv7180_pwdn(int pwdn)
{
	gpio_request(MX53_TVIN_PWR, "tvin-pwr");
	if (pwdn)
		gpio_set_value(MX53_TVIN_PWR, 0);
	else
		gpio_set_value(MX53_TVIN_PWR, 1);
	gpio_free(MX53_TVIN_PWR);
}

static struct mxc_tvin_platform_data adv7180_data = {
	.dvddio_reg = NULL,
	.dvdd_reg = NULL,
	.avdd_reg = NULL,
	.pvdd_reg = NULL,
	.pwdn = adv7180_pwdn,
	.reset = NULL,
};

static struct resource mxcfb_resources[] = {
	[0] = {
	       .flags = IORESOURCE_MEM,
	       },
};

static struct mxc_fb_platform_data fb_data[] = {
	{
	 .interface_pix_fmt = IPU_PIX_FMT_RGB565,
	 .mode_str = "CLAA-WVGA",
	 .mode = video_modes,
	 .num_modes = ARRAY_SIZE(video_modes),
	 },
	{
	 .interface_pix_fmt = IPU_PIX_FMT_GBR24,
	 .mode_str = "1024x768M-16@60",
	 .mode = video_modes,
	 .num_modes = ARRAY_SIZE(video_modes),
	 },
};

extern int primary_di;
static int __init mxc_init_fb(void)
{
	if (!machine_is_mx53_evk())
		return 0;

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

static void camera_pwdn(int pwdn)
{
	gpio_request(MX53_TVIN_PWR, "tvin-pwr");
	gpio_set_value(MX53_TVIN_PWR, pwdn);
	gpio_free(MX53_TVIN_PWR);
}

static struct mxc_camera_platform_data camera_data = {
	.analog_regulator = "VSD",
	.gpo_regulator = "VVIDEO",
	.mclk = 24000000,
	.csi = 0,
	.pwdn = camera_pwdn,
};

static struct mxc_audio_codec_platform_data cs42888_data = {
	.analog_regulator = "VSD",
	.pwdn = gpio_cs42888_pdwn,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
	.type = "ov3640",
	.addr = 0x3C,
	.platform_data = (void *)&camera_data,
	 },
	{
	.type = "adv7180",
	.addr = 0x21,
	.platform_data = (void *)&adv7180_data,
	 },
	{
	 .type = "cs42888",
	 .addr = 0x48,
	 .platform_data = &cs42888_data,
	 },
};

static void sii9022_hdmi_reset(void)
{
	gpio_set_value(MX53_DVI_RESET, 0);
	msleep(10);
	gpio_set_value(MX53_DVI_RESET, 1);
	msleep(10);
}

static struct mxc_lcd_platform_data sii9022_hdmi_data = {
	.reset = sii9022_hdmi_reset,
};

/* TO DO add platform data */
static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
	 .type = "sgtl5000-i2c",
	 .addr = 0x0a,
	 },
	{
	 .type = "tsc2007",
	 .addr = 0x48,
	 .irq  = IOMUX_TO_IRQ_V3(EVK_TS_INT),
	},
	{
	 .type = "backlight-i2c",
	 .addr = 0x2c,
	 },
	{
	 .type = "vga-ddc",
	 .addr = 0x1f,
	 },
	{
	 .type = "eeprom",
	 .addr = 0x50,
	 },
	{
	.type = "sii9022",
	.addr = 0x39,
	.irq = IOMUX_TO_IRQ_V3(MX53_DVI_DETECT),
	.platform_data = &sii9022_hdmi_data,
	},
};

static struct mtd_partition mxc_dataflash_partitions[] = {
	{
	 .name = "bootloader",
	 .offset = 0,
	 .size = 0x000100000,},
	{
	 .name = "kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL,},
};

static struct flash_platform_data mxc_spi_flash_data[] = {
	{
	 .name = "mxc_dataflash",
	 .parts = mxc_dataflash_partitions,
	 .nr_parts = ARRAY_SIZE(mxc_dataflash_partitions),
	 .type = "at45db321d",}
};


static struct spi_board_info mxc_dataflash_device[] __initdata = {
	{
	 .modalias = "mxc_dataflash",
	 .max_speed_hz = 25000000,	/* max spi clock (SCK) speed in HZ */
	 .bus_num = 1,
	 .chip_select = 1,
	 .platform_data = &mxc_spi_flash_data[0],},
};

static int sdhc_write_protect(struct device *dev)
{
	unsigned short rc = 0;

	if (!board_is_mx53_arm2()) {
		if (to_platform_device(dev)->id == 0)
			rc = gpio_get_value(EVK_SD1_WP);
		else
			rc = gpio_get_value(EVK_SD3_WP);
	}

	return rc;
}

static unsigned int sdhc_get_card_det_status(struct device *dev)
{
	int ret;
	if (board_is_mx53_arm2()) {
		if (to_platform_device(dev)->id == 0)
			ret = gpio_get_value(ARM2_SD1_CD);
		else
			ret = 1;
	} else {
		if (to_platform_device(dev)->id == 0) {
			ret = gpio_get_value(EVK_SD1_CD);
		} else{		/* config the det pin for SDHC3 */
			ret = gpio_get_value(EVK_SD3_CD);
			}
	}

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

static struct mxc_mmc_platform_data mmc3_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
		| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA
		| MMC_CAP_DATA_DDR,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 0,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
};

static int mxc_sgtl5000_amp_enable(int enable)
{
/* TO DO */
return 0;
}

static int headphone_det_status(void)
{
	return (gpio_get_value(MX53_HP_DETECT) == 0);
}

static int mxc_sgtl5000_init(void);

static struct mxc_audio_platform_data sgtl5000_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 5,
	.hp_irq = IOMUX_TO_IRQ(MX53_HP_DETECT),
	.hp_status = headphone_det_status,
	.amp_enable = mxc_sgtl5000_amp_enable,
	.init = mxc_sgtl5000_init,
};

static int mxc_sgtl5000_init(void)
{
	struct clk *ssi_ext1;
	int rate;

	if (board_is_mx53_arm2()) {
		sgtl5000_data.sysclk = 12000000;
	} else {
		ssi_ext1 = clk_get(NULL, "ssi_ext1_clk");
		if (IS_ERR(ssi_ext1))
			return -1;

		rate = clk_round_rate(ssi_ext1, 24000000);
		if (rate < 8000000 || rate > 27000000) {
			printk(KERN_ERR "Error: SGTL5000 mclk freq %d out of range!\n",
			       rate);
			clk_put(ssi_ext1);
			return -1;
		}

		clk_set_rate(ssi_ext1, rate);
		clk_enable(ssi_ext1);
		sgtl5000_data.sysclk = rate;
	}

	return 0;
}

static struct platform_device mxc_sgtl5000_device = {
	.name = "imx-3stack-sgtl5000",
};

static struct mxc_mlb_platform_data mlb_data = {
	.reg_nvcc = "VCAM",
	.mlb_clk = "mlb_clk",
};

static void mxc_register_powerkey(key_press_call_back kp_cb, void *param)
{
	pmic_event_callback_t power_key_event;

	power_key_event.param = param;
	power_key_event.func = (void *)kp_cb;
	pmic_event_subscribe(EVENT_PWRONI, power_key_event);
}

static struct power_key_platform_data pwrkey_data = {
	.register_key_press_handler = mxc_register_powerkey,
};

/* NAND Flash Partitions */
#ifdef CONFIG_MTD_PARTITIONS
static struct mtd_partition nand_flash_partitions[] = {
/* MX53 ROM require the boot FCB/DBBT support which need
 * more space to store such info on NAND boot partition.
 * 16M should cover all kind of NAND boot support on MX53.
 */
	{
	 .name = "bootloader",
	 .offset = 0,
	 .size = 16 * 1024 * 1024},
	{
	 .name = "nand.kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 5 * 1024 * 1024},
	{
	 .name = "nand.rootfs",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 256 * 1024 * 1024},
	{
	 .name = "nand.userfs1",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 256 * 1024 * 1024},
	{
	 .name = "nand.userfs2",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL},
};
#endif

static int nand_init(void)
{
	u32 i, reg;
	void __iomem *base;

	#define M4IF_GENP_WEIM_MM_MASK          0x00000001
	#define WEIM_GCR2_MUX16_BYP_GRANT_MASK  0x00001000

	base = ioremap(MX53_BASE_ADDR(M4IF_BASE_ADDR), SZ_4K);
	reg = __raw_readl(base + 0xc);
	reg &= ~M4IF_GENP_WEIM_MM_MASK;
	__raw_writel(reg, base + 0xc);

	iounmap(base);

	base = ioremap(MX53_BASE_ADDR(WEIM_BASE_ADDR), SZ_4K);
	for (i = 0x4; i < 0x94; i += 0x18) {
		reg = __raw_readl((u32)base + i);
		reg &= ~WEIM_GCR2_MUX16_BYP_GRANT_MASK;
		__raw_writel(reg, (u32)base + i);
	}

	iounmap(base);

	return 0;
}

static struct flash_platform_data mxc_nand_data = {
#ifdef CONFIG_MTD_PARTITIONS
	.parts = nand_flash_partitions,
	.nr_parts = ARRAY_SIZE(nand_flash_partitions),
#endif
	.width = 1,
	.init = nand_init,
};

static struct mxc_asrc_platform_data mxc_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx = 1,
	.spdif_rx = 0,
	.spdif_clk_44100 = 0,	/* Souce from CKIH1 for 44.1K */
	.spdif_clk_48000 = 7,	/* Source from CKIH2 for 48k and 32k */
	.spdif_clkid = 0,
	.spdif_clk = NULL,	/* spdif bus clk */
};

static struct mxc_audio_platform_data mxc_surround_audio_data = {
	.ext_ram = 1,
	.sysclk = 22579200,
};


static struct platform_device mxc_alsa_surround_device = {
	.name = "imx-3stack-cs42888",
};

static int __initdata mxc_apc_on = { 0 };	/* OFF: 0 (default), ON: 1 */
static int __init apc_setup(char *__unused)
{
	mxc_apc_on = 1;
	printk(KERN_INFO "Automotive Port Card is Plugged on\n");
	return 1;
}
__setup("apc", apc_setup);

static int __initdata enable_w1 = { 0 };
static int __init w1_setup(char *__unused)
{
	enable_w1 = 1;
	return cpu_is_mx53();
}
__setup("w1", w1_setup);


static int __initdata enable_spdif = { 0 };
static int __init spdif_setup(char *__unused)
{
	enable_spdif = 1;
	return 1;
}
__setup("spdif", spdif_setup);

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

	get_cpu_wp = mx53_evk_get_cpu_wp;
	set_num_cpu_wp = mx53_evk_set_num_cpu_wp;

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

static void __init mx53_evk_io_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx53common_pads,
					ARRAY_SIZE(mx53common_pads));

	if (board_is_mx53_arm2()) {
		/* MX53 ARM2 CPU board */
		pr_info("MX53 ARM2 board \n");
		mxc_iomux_v3_setup_multiple_pads(mx53arm2_pads,
					ARRAY_SIZE(mx53arm2_pads));

		/* Config GPIO for OTG VBus */
		gpio_request(ARM2_OTG_VBUS, "otg-vbus");
		gpio_direction_output(ARM2_OTG_VBUS, 1);

		gpio_request(ARM2_SD1_CD, "sdhc1-cd");
		gpio_direction_input(ARM2_SD1_CD);	/* SD1 CD */

		gpio_request(ARM2_LCD_CONTRAST, "lcd-contrast");
		gpio_direction_output(ARM2_LCD_CONTRAST, 1);
	} else {
		/* MX53 EVK board */
		pr_info("MX53 EVK board \n");
		mxc_iomux_v3_setup_multiple_pads(mx53evk_pads,
					ARRAY_SIZE(mx53evk_pads));

		/* Host1 Vbus with GPIO high */
		gpio_request(EVK_USBH1_VBUS, "usbh1-vbus");
		gpio_direction_output(EVK_USBH1_VBUS, 1);
		/* shutdown the Host1 Vbus when system bring up,
		* Vbus will be opened in Host1 driver's probe function */
		gpio_set_value(EVK_USBH1_VBUS, 0);

		/* USB HUB RESET - De-assert USB HUB RESET_N */
		gpio_request(EVK_USB_HUB_RESET, "usb-hub-reset");
		gpio_direction_output(EVK_USB_HUB_RESET, 0);
		msleep(1);
		gpio_set_value(EVK_USB_HUB_RESET, 1);

		/* Config GPIO for OTG VBus */
		gpio_request(EVK_OTG_VBUS, "otg-vbus");
		gpio_direction_output(EVK_OTG_VBUS, 0);
		if (board_is_mx53_evk_a()) /*rev A,"1" disable, "0" enable vbus*/
			gpio_set_value(EVK_OTG_VBUS, 1);
		else if (board_is_mx53_evk_b()) /* rev B,"0" disable,"1" enable Vbus*/
			gpio_set_value(EVK_OTG_VBUS, 0);

		gpio_request(EVK_SD1_CD, "sdhc1-cd");
		gpio_direction_input(EVK_SD1_CD);	/* SD1 CD */
		gpio_request(EVK_SD1_WP, "sdhc1-wp");
		gpio_direction_input(EVK_SD1_WP);	/* SD1 WP */

		/* SD3 CD */
		gpio_request(EVK_SD3_CD, "sdhc3-cd");
		gpio_direction_input(EVK_SD3_CD);

		/* SD3 WP */
		gpio_request(EVK_SD3_WP, "sdhc3-wp");
		gpio_direction_input(EVK_SD3_WP);

		/* reset FEC PHY */
		gpio_request(EVK_FEC_PHY_RESET, "fec-phy-reset");
		gpio_direction_output(EVK_FEC_PHY_RESET, 0);
		msleep(1);
		gpio_set_value(EVK_FEC_PHY_RESET, 1);

		gpio_request(MX53_ESAI_RESET, "fesai-reset");
		gpio_direction_output(MX53_ESAI_RESET, 0);
	}

	/* DVI Detect */
	gpio_request(MX53_DVI_DETECT, "dvi-detect");
	gpio_direction_input(MX53_DVI_DETECT);
	/* DVI Reset - Assert for i2c disabled mode */
	gpio_request(MX53_DVI_RESET, "dvi-reset");
	gpio_direction_output(MX53_DVI_RESET, 0);

	/* DVI Power-down */
	gpio_request(MX53_DVI_PD, "dvi-pd");
	gpio_direction_output(MX53_DVI_PD, 1);

	/* DVI I2C enable */
	gpio_request(MX53_DVI_I2C, "dvi-i2c");
	gpio_direction_output(MX53_DVI_I2C, 0);

	mxc_iomux_v3_setup_multiple_pads(mx53_nand_pads,
					ARRAY_SIZE(mx53_nand_pads));

	gpio_request(MX53_PMIC_INT, "pmic-int");
	gpio_direction_input(MX53_PMIC_INT);	/*PMIC_INT*/

	/* headphone_det_b */
	gpio_request(MX53_HP_DETECT, "hp-detect");
	gpio_direction_input(MX53_HP_DETECT);

	/* power key */

	/* LCD related gpio */

	/* Camera reset */
	gpio_request(MX53_CAM_RESET, "cam-reset");
	gpio_direction_output(MX53_CAM_RESET, 1);

	/* TVIN reset */
	gpio_request(MX53_TVIN_RESET, "tvin-reset");
	gpio_direction_output(MX53_TVIN_RESET, 0);
	msleep(5);
	gpio_set_value(MX53_TVIN_RESET, 1);

	/* TVin power down */
	gpio_request(MX53_TVIN_PWR, "tvin-pwr");
	gpio_direction_output(MX53_TVIN_PWR, 0);

	/* CAN1 enable GPIO*/
	gpio_request(MX53_CAN1_EN1, "can1-en1");
	gpio_direction_output(MX53_CAN1_EN1, 0);

	gpio_request(MX53_CAN1_EN2, "can1-en2");
	gpio_direction_output(MX53_CAN1_EN2, 0);

	/* CAN2 enable GPIO*/
	gpio_request(MX53_CAN2_EN1, "can2-en1");
	gpio_direction_output(MX53_CAN2_EN1, 0);

	gpio_request(MX53_CAN2_EN2, "can2-en2");
	gpio_direction_output(MX53_CAN2_EN2, 0);

	if (enable_spdif) {
		struct pad_desc spdif_pin = MX53_PAD_GPIO_19__SPDIF_TX1;
		mxc_iomux_v3_setup_pad(&spdif_pin);
	} else {
		/* GPIO for 12V */
		gpio_request(MX53_12V_EN, "12v-en");
		gpio_direction_output(MX53_12V_EN, 0);
	}
}

extern void mx53_gpio_usbotg_driver_vbus(bool on);
extern void mx53_gpio_host1_driver_vbus(bool on);
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

	/* SD card detect irqs */
	if (board_is_mx53_arm2()) {
		mxcsdhc1_device.resource[2].start = IOMUX_TO_IRQ_V3(ARM2_SD1_CD);
		mxcsdhc1_device.resource[2].end = IOMUX_TO_IRQ_V3(ARM2_SD1_CD);
		mmc3_data.card_inserted_state = 1;
		mmc3_data.status = NULL;
		mmc3_data.wp_status = NULL;
		mmc1_data.wp_status = NULL;
	} else {
		mxcsdhc3_device.resource[2].start = IOMUX_TO_IRQ_V3(EVK_SD3_CD);
		mxcsdhc3_device.resource[2].end = IOMUX_TO_IRQ_V3(EVK_SD3_CD);
		mxcsdhc1_device.resource[2].start = IOMUX_TO_IRQ_V3(EVK_SD1_CD);
		mxcsdhc1_device.resource[2].end = IOMUX_TO_IRQ_V3(EVK_SD1_CD);
	}

	mxc_cpu_common_init();
	mx53_evk_io_init();

	mxc_register_device(&mxc_dma_device, NULL);
	mxc_register_device(&mxc_wdt_device, NULL);
	mxc_register_device(&mxcspi1_device, &mxcspi1_data);
	mxc_register_device(&mxci2c_devices[0], &mxci2c_data);
	mxc_register_device(&mxci2c_devices[1], &mxci2c_data);
	mxc_register_device(&mxci2c_devices[2], &mxci2c_data);

	mxc_register_device(&mxc_rtc_device, &srtc_data);
	mxc_register_device(&mxc_w1_master_device, &mxc_w1_data);
	mxc_register_device(&mxc_ipu_device, &mxc_ipu_data);
	mxc_register_device(&mxc_ldb_device, &ldb_data);
	mxc_register_device(&mxc_tve_device, &tve_data);
	mxc_register_device(&mxcvpu_device, &mxc_vpu_data);
	mxc_register_device(&gpu_device, &z160_revision);
	mxc_register_device(&mxcscc_device, NULL);
	/*
	mxc_register_device(&mx53_lpmode_device, NULL);
	mxc_register_device(&sdram_autogating_device, NULL);
	*/
	mxc_register_device(&mxc_dvfs_core_device, &dvfs_core_data);
	mxc_register_device(&busfreq_device, &bus_freq_data);

	/*
	mxc_register_device(&mxc_dvfs_per_device, &dvfs_per_data);
	*/

	mxc_register_device(&mxc_iim_device, &iim_data);
	if (!board_is_mx53_arm2()) {
		mxc_register_device(&mxc_pwm2_device, NULL);
		mxc_register_device(&mxc_pwm1_backlight_device,
			&mxc_pwm_backlight_data);
	}
	mxc_register_device(&mxc_flexcan0_device, &flexcan0_data);
	mxc_register_device(&mxc_flexcan1_device, &flexcan1_data);

/*	mxc_register_device(&mxc_keypad_device, &keypad_plat_data); */

	mxc_register_device(&mxcsdhc1_device, &mmc1_data);
	mxc_register_device(&mxcsdhc3_device, &mmc3_data);
	mxc_register_device(&mxc_ssi1_device, NULL);
	mxc_register_device(&mxc_ssi2_device, NULL);
	mxc_register_device(&ahci_fsl_device, &sata_data);

	/* ASRC is only available for MX53 TO2.0 */
	if (cpu_is_mx53_rev(CHIP_REV_2_0) >= 1) {
		mxc_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
		clk_put(mxc_asrc_data.asrc_core_clk);
		mxc_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
		clk_set_rate(mxc_asrc_data.asrc_audio_clk, 1190000);
		clk_put(mxc_asrc_data.asrc_audio_clk);
		mxc_register_device(&mxc_asrc_device, &mxc_asrc_data);
	}

	mxc_register_device(&mxc_alsa_spdif_device, &mxc_spdif_data);
	if (!mxc_apc_on) {
		mxc_register_device(&mxc_fec_device, &fec_data);
		mxc_register_device(&mxc_ptp_device, NULL);
	}
	spi_register_board_info(mxc_dataflash_device,
				ARRAY_SIZE(mxc_dataflash_device));
	i2c_register_board_info(0, mxc_i2c0_board_info,
				ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));

	mx53_evk_init_mc13892();
/*
	pm_power_off = mxc_power_off;
	*/
	mxc_register_device(&mxc_sgtl5000_device, &sgtl5000_data);
	mxc_register_device(&mxc_mlb_device, &mlb_data);
	mxc_register_device(&mxc_powerkey_device, &pwrkey_data);
	mx5_set_otghost_vbus_func(mx53_gpio_usbotg_driver_vbus);
	mx5_usb_dr_init();
	mx5_set_host1_vbus_func(mx53_gpio_host1_driver_vbus);
	mx5_usbh1_init();
	mxc_register_device(&mxc_nandv2_mtd_device, &mxc_nand_data);
	if (mxc_apc_on) {
		mxc_register_device(&mxc_esai_device, &esai_data);
		mxc_register_device(&mxc_alsa_surround_device,
			&mxc_surround_audio_data);
	}
	mxc_register_device(&mxc_v4l2_device, NULL);
	mxc_register_device(&mxc_v4l2out_device, NULL);
}

static void __init mx53_evk_timer_init(void)
{
	struct clk *uart_clk;

	mx53_clocks_init(32768, 24000000, 22579200, 24576000);

	uart_clk = clk_get_sys("mxcintuart.0", NULL);
	early_console_setup(MX53_BASE_ADDR(UART1_BASE_ADDR), uart_clk);
}

static struct sys_timer mxc_timer = {
	.init	= mx53_evk_timer_init,
};

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX53_EVK data structure.
 */
MACHINE_START(MX53_EVK, "Freescale MX53 EVK Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.fixup = fixup_mxc_board,
	.map_io = mx5_map_io,
	.init_irq = mx5_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END
