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
#include <linux/smsc911x.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
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

#define ARD_SD1_CD			(0*32 + 1)	/* GPIO_1_1 */
#define ARD_SD2_WP			(0*32 + 2)	/* GPIO_1_2 */
#define ARD_SD2_CD			(0*32 + 4)	/* GPIO_1_4 */
#define ARD_VIDEOIN_INT_B	(0*32 + 7)	/* GPIO_1_7 */
#define ARD_SD1_WP			(0*32 + 9)	/* GPIO_1_9 */
#define ARD_PMIC_RDY			(0*32 + 24)	/* GPIO_1_24 */


#define ARD_CAN1_NERR_B	(1*32 + 0)	/* GPIO_2_0 */
#define ARD_CAN2_NERR_B	(1*32 + 1)	/* GPIO_2_1 */
#define ARD_VIDEOIN_PWR	(1*32 + 2)	/* GPIO_2_2 */
#define ARD_I2CPORTEXP_B	(1*32 + 3)	/* GPIO_2_3 */
#define ARD_ESAI_INT			(1*32 + 4)	/* GPIO_2_4 */
#define ARD_GPS_PWREN		(1*32 + 5)	/* GPIO_2_5 */
#define ARD_GPS_INT_B		(1*32 + 6)	/* GPIO_2_6 */
#define ARD_MLB_INT			(1*32 + 7)	/* GPIO_2_7 */
#define ARD_ETHERNET_INT_B			(1*32 + 31)	/* GPIO_2_31 */

#define ARD_SATE_REQ			(3*32 + 0)	/* GPIO_4_0 */
#define ARD_USBH1_OC			(3*32 + 1)	/* GPIO_4_1 */
#define ARD_USBH2_OC			(3*32 + 2)	/* GPIO_4_2 */
#define ARD_USBH1_PWR			(3*32 + 3)	/* GPIO_4_3 */
#define ARD_USER_LED			(3*32 + 4)	/* GPIO_4_4 */
#define ARD_FPGA_INT_B			(3*32 + 5)	/* GPIO_4_5 */
#define ARD_USBH2_PHYRST_B	(3*32 + 14)	/* GPIO_4_14 */
#define ARD_CAN_STBY		(3*32 + 15)	/* GPIO_4_15 */
#define ARD_PWM1_OFF		(3*32 + 29)	/* GPIO_4_29 */
#define ARD_PWM2_OFF		(3*32 + 30)	/* GPIO_4_30 */

#define ARD_USBOTG_PWR		(4*32 + 2)	/* GPIO_5_2 */
#define ARD_USBOTG_OC			(4*32 + 4)	/* GPIO_5_4 */
#define ARD_PMIC_INT			(4*32 + 7)	/* GPIO_5_7 */
#define ARD_PMIC_PBSTAT		(4*32 + 8)	/* GPIO_5_8 */
#define ARD_MLB_PWRDN		(4*32 + 9)	/* GPIO_5_9 */

#define ARD_CAN_EN			(6*32 + 6)	/* GPIO_7_6 */
#define ARD_TS_INT			(6*32 + 12)	/* GPIO_7_12 */
#define ARD_SD1_LCTL		(6*32 + 13)	/* GPIO_7_13 */

/* Start directly after the CPU's GPIO*/
#define MAX7310_BASE_ADDR		224	/* 7x32 */
#define ARD_BACKLIGHT_ON		MAX7310_BASE_ADDR
#define ARD_SPARE			(MAX7310_BASE_ADDR + 1)
#define ARD_CPU_PER_RST_B		(MAX7310_BASE_ADDR + 2)
#define ARD_MAIN_PER_RST_B		(MAX7310_BASE_ADDR + 3)
#define ARD_IPOD_RST_B			(MAX7310_BASE_ADDR + 4)
#define ARD_MLB_RST_B			(MAX7310_BASE_ADDR + 5)
#define ARD_SSI_STEERING		(MAX7310_BASE_ADDR + 6)
#define ARD_GPS_RST_B			(MAX7310_BASE_ADDR + 7)

/*!
 * @file mach-mx53/mx53_ard.c
 *
 * @brief This file contains the board specific initialization routines.
 *
 * @ingroup MSL_MX53
 */

static struct pad_desc mx53ard_pads[] = {
	/* UART1 */
	MX53_PAD_ATA_DIOW__UART1_TXD,
	MX53_PAD_ATA_DMACK__UART1_RXD,

	/* UART2 */
	MX53_PAD_ATA_BUFFER_EN__UART2_RXD,
	MX53_PAD_ATA_DMARQ__UART2_TXD,
	MX53_PAD_ATA_DIOR__UART2_RTS,
	MX53_PAD_ATA_INTRQ__UART2_CTS,

	/* UART3 */
	MX53_PAD_ATA_CS_0__UART3_TXD,
	MX53_PAD_ATA_CS_1__UART3_RXD,
	MX53_PAD_ATA_DA_1__UART3_CTS,
	MX53_PAD_ATA_DA_2__UART3_RTS,

	/* PMIC */
	MX53_PAD_DISP0_DAT13__GPIO_5_7,
	MX53_PAD_FEC_RX_ER__GPIO_1_24,
	MX53_PAD_DISP0_DAT14__GPIO_5_8,

	/* USBOTG_OC and USBOTG_PWR */
	MX53_PAD_EIM_A24__GPIO_5_4,
	MX53_PAD_EIM_A25__GPIO_5_2,

	/* USBH */
	MX53_PAD_GPIO_11__GPIO_4_1,
	MX53_PAD_GPIO_12__GPIO_4_2,
	MX53_PAD_GPIO_13__GPIO_4_3,
	MX53_PAD_KEY_COL4__GPIO_4_14,

	/* MAINBRD_SPDIF_IN */
	MX53_PAD_KEY_COL3__SPDIF_IN1,

	/* CAN */
	MX53_PAD_KEY_COL2__TXCAN1,
	MX53_PAD_KEY_ROW2__RXCAN1,
	MX53_PAD_ATA_RESET_B__TXCAN2,
	MX53_PAD_ATA_IORDY__RXCAN2,

	/* CAN1, CAN2 -- EN */
	MX53_PAD_ATA_DA_0__GPIO_7_6,
	/* CAN1, CAN2 -- STBY */
	MX53_PAD_KEY_ROW4__GPIO_4_15,
	/* CAN1 -- NERR */
	MX53_PAD_ATA_DATA0__GPIO_2_0,
	/* CAN2 -- NERR */
	MX53_PAD_ATA_DATA1__GPIO_2_1,

	MX53_PAD_DISP0_DAT0__USBH2_DAT0,
	MX53_PAD_DISP0_DAT1__USBH2_DAT1,
	MX53_PAD_DISP0_DAT2__USBH2_DAT2,
	MX53_PAD_DISP0_DAT3__USBH2_DAT3,
	MX53_PAD_DISP0_DAT4__USBH2_DAT4,
	MX53_PAD_DISP0_DAT5__USBH2_DAT5,
	MX53_PAD_DISP0_DAT6__USBH2_DAT6,
	MX53_PAD_DISP0_DAT7__USBH2_DAT7,
	MX53_PAD_DISP0_DAT8__PWM1,
	MX53_PAD_DISP0_DAT9__PWM2,
	MX53_PAD_DISP0_DAT10__USBH2_STP,
	MX53_PAD_DISP0_DAT11__USBH2_NXT,
	MX53_PAD_DISP0_DAT12__USBH2_CLK,
	MX53_PAD_DI0_DISP_CLK__USBH2_DIR,

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

	/* Bluetooth */
	MX53_PAD_DISP0_DAT23__AUD4_SSI_RXD,
	MX53_PAD_DISP0_DAT21__AUD4_TXD,
	MX53_PAD_DISP0_DAT20__AUD4_TXC,
	MX53_PAD_DISP0_DAT22__AUD4_TXFS,

	/* Video in */
	MX53_PAD_CSI0_D4__CSI0_D4,
	MX53_PAD_CSI0_D5__CSI0_D5,
	MX53_PAD_CSI0_D6__CSI0_D6,
	MX53_PAD_CSI0_D7__CSI0_D7,
	MX53_PAD_CSI0_D8__CSI0_D8,
	MX53_PAD_CSI0_D9__CSI0_D9,
	MX53_PAD_CSI0_D10__CSI0_D10,
	MX53_PAD_CSI0_D11__CSI0_D11,
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

	/* VIDEO_ADC_PWRDN_B */
	MX53_PAD_GPIO_7__GPIO_1_7,
	MX53_PAD_ATA_DATA2__GPIO_2_2,

	/* MLB */
	MX53_PAD_FEC_TXD1__MLBCLK,
	MX53_PAD_FEC_MDC__MLBDAT,
	MX53_PAD_GPIO_6__MLBSIG,
	MX53_PAD_ATA_DATA7__GPIO_2_7,
	MX53_PAD_DISP0_DAT15__GPIO_5_9,

	/* esdhc1 */
	MX53_PAD_SD1_CMD__SD1_CMD,
	MX53_PAD_SD1_CLK__SD1_CLK,
	MX53_PAD_SD1_DATA0__SD1_DATA0,
	MX53_PAD_SD1_DATA1__SD1_DATA1,
	MX53_PAD_SD1_DATA2__SD1_DATA2,
	MX53_PAD_SD1_DATA3__SD1_DATA3,
	MX53_PAD_ATA_DATA8__SD1_DATA4,
	MX53_PAD_ATA_DATA9__SD1_DATA5,
	MX53_PAD_ATA_DATA10__SD1_DATA6,
	MX53_PAD_ATA_DATA11__SD1_DATA7,
	MX53_PAD_GPIO_1__GPIO_1_1,
	MX53_PAD_GPIO_9__GPIO_1_9,

	/* esdhc2 */
	MX53_PAD_SD2_DATA0__SD2_DAT0,
	MX53_PAD_SD2_DATA1__SD2_DAT1,
	MX53_PAD_SD2_DATA2__SD2_DAT2,
	MX53_PAD_SD2_DATA3__SD2_DAT3,
	MX53_PAD_ATA_DATA12__SD2_DAT4,
	MX53_PAD_ATA_DATA13__SD2_DAT5,
	MX53_PAD_ATA_DATA14__SD2_DAT6,
	MX53_PAD_ATA_DATA15__SD2_DAT7,
	MX53_PAD_SD2_CLK__SD2_CLK,
	MX53_PAD_SD2_CMD__SD2_CMD,
	MX53_PAD_GPIO_4__GPIO_1_4,
	MX53_PAD_GPIO_2__GPIO_1_2,

	/* WEIM for CS1 */
	/* ETHERNET_INT_B */
	MX53_PAD_EIM_EB3__GPIO_2_31,
	MX53_PAD_EIM_D16__EIM_D16,
	MX53_PAD_EIM_D17__EIM_D17,
	MX53_PAD_EIM_D18__EIM_D18,
	MX53_PAD_EIM_D19__EIM_D19,
	MX53_PAD_EIM_D20__EIM_D20,
	MX53_PAD_EIM_D21__EIM_D21,
	MX53_PAD_EIM_D22__EIM_D22,
	MX53_PAD_EIM_D23__EIM_D23,
	MX53_PAD_EIM_D24__EIM_D24,
	MX53_PAD_EIM_D25__EIM_D25,
	MX53_PAD_EIM_D26__EIM_D26,
	MX53_PAD_EIM_D27__EIM_D27,
	MX53_PAD_EIM_D28__EIM_D28,
	MX53_PAD_EIM_D29__EIM_D29,
	MX53_PAD_EIM_D30__EIM_D30,
	MX53_PAD_EIM_D31__EIM_D31,
	MX53_PAD_EIM_DA0__EIM_DA0,
	MX53_PAD_EIM_DA1__EIM_DA1,
	MX53_PAD_EIM_DA2__EIM_DA2,
	MX53_PAD_EIM_DA3__EIM_DA3,
	MX53_PAD_EIM_DA4__EIM_DA4,
	MX53_PAD_EIM_DA5__EIM_DA5,
	MX53_PAD_EIM_DA6__EIM_DA6,
	MX53_PAD_EIM_OE__EIM_OE,
	MX53_PAD_EIM_RW__EIM_RW,
	MX53_PAD_EIM_CS1__EIM_CS1,

	/* I2C2 */
	MX53_PAD_EIM_EB2__I2C2_SCL,
	MX53_PAD_KEY_ROW3__I2C2_SDA,

	/* I2C3 */
	MX53_PAD_GPIO_3__I2C3_SCL,
	MX53_PAD_GPIO_16__I2C3_SDA,

	/* TOUCH_INT_B */
	MX53_PAD_GPIO_17__GPIO_7_12,

	/* Tuner */
	MX53_PAD_DI0_PIN15__AUD6_TXC,
	MX53_PAD_DI0_PIN4__AUD6_RXD,
	MX53_PAD_DI0_PIN3__AUD6_TXFS,

	/* FPGA */
	MX53_PAD_EIM_A23__EIM_A23,
	MX53_PAD_GPIO_19__GPIO_4_5,

	/* eCSPI */
	MX53_PAD_DISP0_DAT16__ECSPI2_MOSI,
	MX53_PAD_DISP0_DAT17__ECSPI2_MISO,
	MX53_PAD_DISP0_DAT18__ECSPI2_SS0,
	MX53_PAD_DISP0_DAT19__ECSPI2_SCLK,

	/* NAND */
	MX53_PAD_NANDF_CLE__NANDF_CLE,
	MX53_PAD_NANDF_ALE__NANDF_ALE,
	MX53_PAD_NANDF_WP_B__NANDF_WP_B,
	MX53_PAD_NANDF_WE_B__NANDF_WE_B,
	MX53_PAD_NANDF_RE_B__NANDF_RE_B,
	MX53_PAD_NANDF_RB0__NANDF_RB0,
	MX53_PAD_NANDF_CS0__NANDF_CS0,
	MX53_PAD_NANDF_CS1__NANDF_CS1	,
	MX53_PAD_EIM_DA0__EIM_DA0,
	MX53_PAD_EIM_DA1__EIM_DA1,
	MX53_PAD_EIM_DA2__EIM_DA2,
	MX53_PAD_EIM_DA3__EIM_DA3,
	MX53_PAD_EIM_DA4__EIM_DA4,
	MX53_PAD_EIM_DA5__EIM_DA5,
	MX53_PAD_EIM_DA6__EIM_DA6,
	MX53_PAD_EIM_DA7__EIM_DA7,

	/* IO Port Expander */
	MX53_PAD_ATA_DATA3__GPIO_2_3,

	/* GPS */
	MX53_PAD_GPIO_0__CLKO,
	MX53_PAD_ATA_DATA5__GPIO_2_5,
	MX53_PAD_ATA_DATA6__GPIO_2_6,

	MX53_PAD_GPIO_10__GPIO_4_0,
	MX53_PAD_GPIO_14__GPIO_4_4,
	MX53_PAD_GPIO_18__GPIO_7_13,
	MX53_PAD_GPIO_19__GPIO_4_5,

	/* EIM_WAIT, EIM_OE ... */
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
	 "720P60", 60, 1280, 720, 13468,
	 260, 109,
	 25, 4,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	 /* 800x480 @ 57 Hz , pixel clk @ 27MHz */
	 "CLAA-WVGA", 57, 800, 480, 37037, 40, 60, 10, 10, 20, 10,
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
	{
	 "1080P60", 60, 1920, 1080, 7692,
	 100, 40,
	 30, 3,
	 10, 2,
	 0,
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
};

static struct pad_desc mx53_ard_pwm_pads[] = {
	MX53_PAD_DISP0_DAT8__PWM1,
	MX53_PAD_DISP0_DAT9__PWM2,
	MX53_PAD_DISP0_DAT8__GPIO_4_29,
	MX53_PAD_DISP0_DAT9__GPIO_4_30,
};

static void enable_pwm1_pad(void)
{
	mxc_iomux_v3_setup_pad(&mx53_ard_pwm_pads[0]);
}

static void disable_pwm1_pad(void)
{
	mxc_iomux_v3_setup_pad(&mx53_ard_pwm_pads[2]);

	gpio_request(ARD_PWM2_OFF, "pwm2-off");
	gpio_direction_output(ARD_PWM2_OFF, 1);
	gpio_free(ARD_PWM2_OFF);
}

static struct mxc_pwm_platform_data mxc_pwm1_platform_data = {
	.pwmo_invert = 1,
	.enable_pwm_pad = enable_pwm1_pad,
	.disable_pwm_pad = disable_pwm1_pad,
};

static struct platform_pwm_backlight_data mxc_pwm1_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 5000000,
};

static void enable_pwm2_pad(void)
{
	mxc_iomux_v3_setup_pad(&mx53_ard_pwm_pads[1]);
}

static void disable_pwm2_pad(void)
{
	mxc_iomux_v3_setup_pad(&mx53_ard_pwm_pads[3]);

	gpio_request(ARD_PWM2_OFF, "pwm2-off");
	gpio_direction_output(ARD_PWM2_OFF, 1);
	gpio_free(ARD_PWM2_OFF);
}

static struct mxc_pwm_platform_data mxc_pwm2_platform_data = {
	.pwmo_invert = 1,
	.enable_pwm_pad = enable_pwm2_pad,
	.disable_pwm_pad = disable_pwm2_pad,
};

static struct platform_pwm_backlight_data mxc_pwm2_backlight_data = {
	.pwm_id = 1,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 5000000,
};

static void flexcan_xcvr_enable(int id, int en)
{
	if (id < 0 || id > 1)
		return;

	if (en)
		gpio_set_value(ARD_CAN_EN, 1);
	else
		gpio_set_value(ARD_CAN_EN, 0);
}

static struct flexcan_platform_data flexcan0_data = {
	.core_reg = NULL,
	.io_reg = NULL,
	.xcvr_enable = flexcan_xcvr_enable,
	.br_clksrc = 1,
	.br_rjw = 2,
	.br_presdiv = 5,
	.br_propseg = 5,
	.br_pseg1 = 4,
	.br_pseg2 = 7,
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
	.xcvr_enable = flexcan_xcvr_enable,
	.br_clksrc = 1,
	.br_rjw = 2,
	.br_presdiv = 5,
	.br_propseg = 5,
	.br_pseg1 = 4,
	.br_pseg2 = 7,
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

static struct mxc_spi_master mxcspi1_data = {
	.maxchipselect = 4,
	.spi_version = 23,
};

static struct imxi2c_platform_data mxci2c1_data = {
	.bitrate = 50000,
};

static struct imxi2c_platform_data mxci2c2_data = {
	.bitrate = 400000,
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
};

static struct mxc_bus_freq_platform_data bus_freq_data = {
	.gp_reg_id = "SW1",
	.lp_reg_id = "SW2",
};

static struct ldb_platform_data ldb_data = {
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

static struct pad_desc mx53_ard_esai_pads[] = {
	MX53_PAD_FEC_MDIO__ESAI_SCKR,
	MX53_PAD_FEC_REF_CLK__ESAI_FSR,
	MX53_PAD_FEC_CRS_DV__ESAI_SCKT,
	MX53_PAD_FEC_RXD1__ESAI_FST,
	MX53_PAD_FEC_TX_EN__ESAI_TX3_RX2,
	MX53_PAD_GPIO_5__ESAI_TX2_RX3,
	MX53_PAD_FEC_TXD0__ESAI_TX4_RX1,
	MX53_PAD_GPIO_8__ESAI_TX5_RX0,
	MX53_PAD_NANDF_CS2__ESAI_TX0,
	MX53_PAD_NANDF_CS3__ESAI_TX1,
	MX53_PAD_ATA_DATA4__GPIO_2_4,
};


void ard_gpio_activate_esai_ports(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx53_ard_esai_pads,
					ARRAY_SIZE(mx53_ard_esai_pads));
	/* ESAI_INT */
	gpio_request(ARD_ESAI_INT, "esai-int");
	gpio_direction_input(ARD_ESAI_INT);

}

static struct mxc_esai_platform_data esai_data = {
	.activate_esai_ports = ard_gpio_activate_esai_ports,
};

static void mx53_ard_usbotg_driver_vbus(bool on)
{
	if (on)
		gpio_set_value(ARD_USBOTG_PWR, 1);
	else
		gpio_set_value(ARD_USBOTG_PWR, 0);
}

static void mx53_ard_host1_driver_vbus(bool on)
{
	if (on)
		gpio_set_value(ARD_USBH1_PWR, 1);
	else
		gpio_set_value(ARD_USBH1_PWR, 0);
}

static void adv7180_pwdn(int pwdn)
{
	gpio_request(ARD_VIDEOIN_PWR, "tvin-pwr");
	if (pwdn)
		gpio_set_value(ARD_VIDEOIN_PWR, 0);
	else
		gpio_set_value(ARD_VIDEOIN_PWR, 1);
	gpio_free(ARD_VIDEOIN_PWR);
}

static struct mxc_tvin_platform_data adv7180_data = {
	.dvddio_reg = NULL,
	.dvdd_reg = NULL,
	.avdd_reg = NULL,
	.pvdd_reg = NULL,
	.pwdn = adv7180_pwdn,
	.reset = NULL,
	.cvbs = true,
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
	 .interface_pix_fmt = IPU_PIX_FMT_BGR24,
	 .mode_str = "1024x768M-16@60",
	 .mode = video_modes,
	 .num_modes = ARRAY_SIZE(video_modes),
	 },
};

extern int primary_di;
static int __init mxc_init_fb(void)
{
	if (!machine_is_mx53_ard())
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

static struct mxc_audio_codec_platform_data cs42888_data = {
	.analog_regulator = NULL,
};

static int mx53_ard_max7310_setup(struct i2c_client *client,
			       unsigned gpio_base, unsigned ngpio,
			       void *context)
{
	static int max7310_gpio_value[] = {
		1, 1, 1, 1, 0, 0, 0, 0,
	};
	int n;

	for (n = 0; n < ARRAY_SIZE(max7310_gpio_value); ++n) {
		gpio_request(gpio_base + n, "MAX7310 GPIO Expander");
		if (max7310_gpio_value[n] < 0)
			gpio_direction_input(gpio_base + n);
		else
			gpio_direction_output(gpio_base + n,
					      max7310_gpio_value[n]);
		/* Export, direction locked down */
		gpio_export(gpio_base + n, 0);
	}

	return 0;
}

static struct pca953x_platform_data mx53_i2c_max7310_platdata = {
	.gpio_base	= MAX7310_BASE_ADDR,
	.invert		= 0, /* Do not invert */
	.setup		= mx53_ard_max7310_setup,
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
	.type = "cs42888",
	.addr = 0x48,
	.platform_data = &cs42888_data,
	},
	{
	.type = "ipod",
	.addr = 0x10,
	},
	{
	.type = "tuner",
	.addr = 0x62,
	},
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
	 .type = "max11801",
	 .addr = 0x49,
	 .irq  = IOMUX_TO_IRQ_V3(ARD_TS_INT),
	},
	{
	 .type = "max7310",
	 .addr = 0x18,
	 .platform_data = &mx53_i2c_max7310_platdata,
	},
	{
	 .type = "mlb",
	 .addr = 0x20,
	},
	{
	.type = "adv7180",
	.addr = 0x21,
	.platform_data = (void *)&adv7180_data,
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

	if (to_platform_device(dev)->id == 0)
		rc = gpio_get_value(ARD_SD1_WP);
	else
		rc = gpio_get_value(ARD_SD2_WP);

	return rc;
}

static unsigned int sdhc_get_card_det_status(struct device *dev)
{
	int ret;
	if (to_platform_device(dev)->id == 0) {
		ret = gpio_get_value(ARD_SD1_CD);
	} else{		/* config the det pin for SDHC2 */
		ret = gpio_get_value(ARD_SD2_CD);
	}

	return ret;
}

static struct mxc_mmc_platform_data mmc1_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
		| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
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
	.caps = MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 0,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
};

static struct resource ard_smsc911x_resources[] = {
	{
	 .start = MX53_CS1_BASE_ADDR,
	 .end = MX53_CS1_BASE_ADDR + SZ_4K - 1,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .start =  IOMUX_TO_IRQ_V3(ARD_ETHERNET_INT_B),
	 .end =  IOMUX_TO_IRQ_V3(ARD_ETHERNET_INT_B),
	 .flags = IORESOURCE_IRQ,
	 },
};

struct smsc911x_platform_config ard_smsc911x_config = {
	.irq_polarity = SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type = SMSC911X_IRQ_TYPE_PUSH_PULL,
	.flags = SMSC911X_USE_32BIT,
};

static struct platform_device ard_smsc_lan9220_device = {
	.name = "smsc911x",
	.id = 0,
	.num_resources = ARRAY_SIZE(ard_smsc911x_resources),
	.resource = ard_smsc911x_resources,
};

static struct mxc_mlb_platform_data mlb_data = {
	.reg_nvcc = NULL,
	.mlb_clk = "mlb_clk",
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
	.clk_map_ver = 2.
};

static struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx = 0,
	.spdif_rx = 1,
	.spdif_clk_44100 = 0,	/* Souce from CKIH1 for 44.1K */
	.spdif_clk_48000 = 7,	/* Source from CKIH2 for 48k and 32k */
	.spdif_clkid = 0,
	.spdif_clk = NULL,	/* spdif bus clk */
};

static struct mxc_audio_platform_data mxc_surround_audio_data = {
	.ext_ram = 1,
	.sysclk = 24576000,
};


static struct platform_device mxc_alsa_surround_device = {
	.name = "imx-3stack-cs42888",
};

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

static void __init mx53_ard_io_init(void)
{
	/* MX53 ARD board */
	pr_info("MX53 ARD board \n");
	mxc_iomux_v3_setup_multiple_pads(mx53ard_pads,
				ARRAY_SIZE(mx53ard_pads));

	/* USBOTG_OC */
	gpio_request(ARD_USBOTG_OC, "otg-oc");
	gpio_direction_input(ARD_USBOTG_OC);
	/* USBOTG_PWR */
	gpio_request(ARD_USBOTG_PWR, "otg-pwr");
	gpio_direction_output(ARD_USBOTG_PWR, 1);

	gpio_request(ARD_USBH1_OC, "usbh1-oc");
	gpio_direction_input(ARD_USBH1_OC);

	gpio_request(ARD_USBH2_OC, "usbh2-oc");
	gpio_direction_input(ARD_USBH2_OC);

	gpio_request(ARD_USBH1_PWR, "usbh1-pwr");
	gpio_direction_output(ARD_USBH1_PWR, 1);

	gpio_request(ARD_USBH2_PHYRST_B, "usbh2-phyrst");
	gpio_direction_output(ARD_USBH2_PHYRST_B, 1);

	gpio_request(ARD_SD1_CD, "sdhc1-cd");
	gpio_direction_input(ARD_SD1_CD);	/* SD1 CD */
	gpio_request(ARD_SD1_WP, "sdhc1-wp");
	gpio_direction_input(ARD_SD1_WP);	/* SD1 WP */

	/* SD2 CD */
	gpio_request(ARD_SD2_CD, "sdhc2-cd");
	gpio_direction_input(ARD_SD2_CD);

	/* SD2 WP */
	gpio_request(ARD_SD2_WP, "sdhc2-wp");
	gpio_direction_input(ARD_SD2_WP);

	gpio_request(ARD_PMIC_INT, "pmic-int");
	gpio_direction_input(ARD_PMIC_INT);	/*PMIC_INT*/
	gpio_request(ARD_PMIC_RDY, "pmic-rdy");
	gpio_direction_input(ARD_PMIC_RDY);	/*PMIC_RDY*/
	gpio_request(ARD_PMIC_PBSTAT, "pmic-pbstat");
	gpio_direction_input(ARD_PMIC_PBSTAT);	/*PMIC_PBSTAT*/

	/* CAN1 enable GPIO*/
	gpio_request(ARD_CAN_EN, "can-en");
	gpio_direction_output(ARD_CAN_EN, 0);

	gpio_request(ARD_CAN_STBY, "can-stby");
	gpio_direction_output(ARD_CAN_STBY, 0);

	gpio_request(ARD_CAN1_NERR_B, "can1-nerr");
	gpio_direction_input(ARD_CAN1_NERR_B);

	gpio_request(ARD_CAN2_NERR_B, "can2-nerr");
	gpio_direction_input(ARD_CAN2_NERR_B);

	gpio_request(ARD_VIDEOIN_PWR, "videoin-pwr");
	gpio_direction_output(ARD_VIDEOIN_PWR, 0);

	gpio_request(ARD_I2CPORTEXP_B, "i2cptexp-rst");
	gpio_direction_output(ARD_I2CPORTEXP_B, 1);

	gpio_request(ARD_GPS_PWREN, "gps-pwren");
	gpio_direction_output(ARD_GPS_PWREN, 1);

	gpio_request(ARD_GPS_INT_B, "gps-int");
	gpio_direction_input(ARD_GPS_INT_B);

	gpio_request(ARD_MLB_INT, "mlb-int");
	gpio_direction_input(ARD_MLB_INT);

	gpio_request(ARD_MLB_PWRDN, "mlb-pwrdn");
	gpio_direction_input(ARD_MLB_PWRDN);

	gpio_request(ARD_ETHERNET_INT_B, "eth-int-b");
	gpio_direction_input(ARD_ETHERNET_INT_B);

	gpio_request(ARD_FPGA_INT_B, "fpga-int");
	gpio_direction_input(ARD_FPGA_INT_B);

	gpio_request(ARD_TS_INT, "ts-int");
	gpio_direction_input(ARD_TS_INT);
	gpio_free(ARD_TS_INT);
}

/* Config CS1 settings for ethernet controller */
static void weim_cs_config(void)
{
	u32 reg;
	void __iomem *weim_base, *iomuxc_base;

	weim_base = ioremap(MX53_BASE_ADDR(WEIM_BASE_ADDR), SZ_4K);
	iomuxc_base = ioremap(MX53_BASE_ADDR(IOMUXC_BASE_ADDR), SZ_4K);

	/* CS1 timings for LAN9220 */
	writel(0x20001, (weim_base + 0x18));
	writel(0x0, (weim_base + 0x1C));
	writel(0x16000202, (weim_base + 0x20));
	writel(0x00000002, (weim_base + 0x24));
	writel(0x16002082, (weim_base + 0x28));
	writel(0x00000000, (weim_base + 0x2C));
	writel(0x00000000, (weim_base + 0x90));

	/* specify 64 MB on CS1 and CS0 on GPR1 */
	reg = readl(iomuxc_base + 0x4);
	reg &= ~0x3F;
	reg |= 0x1B;
	writel(reg, (iomuxc_base + 0x4));

	iounmap(iomuxc_base);
	iounmap(weim_base);
}

static int mxc_read_mac_iim(void)
{
	struct clk *iim_clk;
	void __iomem *iim_base = IO_ADDRESS(IIM_BASE_ADDR);
	void __iomem *iim_mac_base = iim_base + \
		MXC_IIM_MX53_BANK_AREA_1_OFFSET + \
		MXC_IIM_MX53_MAC_ADDR_OFFSET;
	int i;

	iim_clk = clk_get(NULL, "iim_clk");

	if (!iim_clk) {
		printk(KERN_ERR "Could not get IIM clk to read MAC fuses!\n");
		return ~EINVAL;
	}

	clk_enable(iim_clk);

	for (i = 0; i < 6; i++)
		ard_smsc911x_config.mac[i] = readl(iim_mac_base + (i*4));

	clk_disable(iim_clk);
	return 0;
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

	mxcsdhc2_device.resource[2].start = IOMUX_TO_IRQ_V3(ARD_SD2_CD);
	mxcsdhc2_device.resource[2].end = IOMUX_TO_IRQ_V3(ARD_SD2_CD);
	mxcsdhc1_device.resource[2].start = IOMUX_TO_IRQ_V3(ARD_SD1_CD);
	mxcsdhc1_device.resource[2].end = IOMUX_TO_IRQ_V3(ARD_SD1_CD);

	mxc_cpu_common_init();

	mx53_ard_io_init();
	weim_cs_config();
	mxc_read_mac_iim();
	mxc_register_device(&ard_smsc_lan9220_device, &ard_smsc911x_config);

	mxc_register_device(&mxc_dma_device, NULL);
	mxc_register_device(&mxc_wdt_device, NULL);
	mxc_register_device(&mxcspi1_device, &mxcspi1_data);
	mxc_register_device(&mxci2c_devices[1], &mxci2c1_data);
	mxc_register_device(&mxci2c_devices[2], &mxci2c2_data);

	mxc_register_device(&mxc_rtc_device, NULL);
	mxc_register_device(&mxc_ipu_device, &mxc_ipu_data);
	mxc_register_device(&mxc_ldb_device, &ldb_data);
	mxc_register_device(&mxcvpu_device, &mxc_vpu_data);
	mxc_register_device(&gpu_device, &z160_revision);
	mxc_register_device(&mxcscc_device, NULL);

	mxc_register_device(&mxc_dvfs_core_device, &dvfs_core_data);
	mxc_register_device(&busfreq_device, &bus_freq_data);

	mxc_register_device(&mxc_iim_device, &iim_data);

	mxc_register_device(&mxc_pwm1_device, &mxc_pwm1_platform_data);
	mxc_register_device(&mxc_pwm1_backlight_device,
		&mxc_pwm1_backlight_data);

	mxc_register_device(&mxc_pwm2_device, &mxc_pwm2_platform_data);
	mxc_register_device(&mxc_pwm2_backlight_device,
		&mxc_pwm2_backlight_data);

	mxc_register_device(&mxc_flexcan0_device, &flexcan0_data);
	mxc_register_device(&mxc_flexcan1_device, &flexcan1_data);

	mxc_register_device(&mxcsdhc1_device, &mmc1_data);
	mxc_register_device(&mxcsdhc2_device, &mmc2_data);

	mxc_register_device(&ahci_fsl_device, &sata_data);

	/* ASRC is only available for MX53 TO2.0 */
	if (cpu_is_mx53_rev(CHIP_REV_2_0) >= 1) {
		mxc_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
		clk_put(mxc_asrc_data.asrc_core_clk);
		mxc_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
		clk_put(mxc_asrc_data.asrc_audio_clk);
		mxc_register_device(&mxc_asrc_device, &mxc_asrc_data);
	}

	mxc_register_device(&mxc_alsa_spdif_device, &mxc_spdif_data);

	spi_register_board_info(mxc_dataflash_device,
				ARRAY_SIZE(mxc_dataflash_device));
	i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
				ARRAY_SIZE(mxc_i2c2_board_info));

	mxc_register_device(&mxc_mlb_device, &mlb_data);
	mx5_set_otghost_vbus_func(mx53_ard_usbotg_driver_vbus);
	mx5_usb_dr_init();
	mx5_set_host1_vbus_func(mx53_ard_host1_driver_vbus);
	mx5_usbh1_init();
	mx5_usbh2_init();
	mxc_register_device(&mxc_nandv2_mtd_device, &mxc_nand_data);
	mxc_register_device(&mxc_esai_device, &esai_data);
	mxc_register_device(&mxc_alsa_surround_device,
			&mxc_surround_audio_data);

	mxc_register_device(&mxc_v4l2out_device, NULL);
	mxc_register_device(&mxc_v4l2_device, NULL);
}

static void __init mx53_ard_timer_init(void)
{
	struct clk *uart_clk;

	mx53_clocks_init(32768, 24000000, 22579200, 24576000);

	uart_clk = clk_get_sys("mxcintuart.0", NULL);
	early_console_setup(MX53_BASE_ADDR(UART1_BASE_ADDR), uart_clk);
}

static struct sys_timer mxc_timer = {
	.init	= mx53_ard_timer_init,
};

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX53_ARD data structure.
 */
MACHINE_START(MX53_ARD, "Freescale MX53 ARD Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.fixup = fixup_mxc_board,
	.map_io = mx5_map_io,
	.init_irq = mx5_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END
