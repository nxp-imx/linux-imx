/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
 * Copyright (C) 2010 Yong Shen. <Yong.Shen@linaro.org>
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
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#include <linux/mxcfb.h>
#include <linux/ipu.h>
#include <linux/pwm_backlight.h>
#include <linux/regulator/consumer.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <mach/imx-uart.h>
#include <mach/iomux-mx53.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_dvfs.h>

#include "crm_regs.h"
#include "devices-imx53.h"
#include "devices.h"

#define MX53_EVK_FEC_PHY_RST    IMX_GPIO_NR(7, 6)
#define EVK_ECSPI1_CS0          IMX_GPIO_NR(2, 30)
#define EVK_ECSPI1_CS1          IMX_GPIO_NR(3, 19)
#define EVK_HP_DETECT			IMX_GPIO_NR(2, 5)	/* GPIO_2_5 */
#define EVK_ECSPI1_CS0			IMX_GPIO_NR(2, 30)
#define EVK_SD3_CD			IMX_GPIO_NR(3, 11)	/* GPIO_3_11 */
#define EVK_SD3_WP			IMX_GPIO_NR(3, 12)	/* GPIO_3_12 */
#define EVK_SD1_CD			IMX_GPIO_NR(3, 13)	/* GPIO_3_13 */
#define EVK_SD1_WP			IMX_GPIO_NR(3, 14)	/* GPIO_3_14 */
#define EVK_ECSPI1_CS1			IMX_GPIO_NR(3, 19)
#define EVK_DISP0_PD			IMX_GPIO_NR(3, 24)	/* GPIO_3_24 */
#define EVK_TS_INT			IMX_GPIO_NR(3, 26)	/* GPIO_3_26 */
#define EVK_DISP0_I2C_EN		IMX_GPIO_NR(3, 28)	/* GPIO_3_28 */
#define EVK_DISP0_DET_INT		IMX_GPIO_NR(3, 31)	/* GPIO_3_31 */
#define EVK_CAM_RESET			IMX_GPIO_NR(4, 0)	/* GPIO_4_0 */
#define EVK_ESAI_RESET			IMX_GPIO_NR(4, 2)	/* GPIO_4_2 */
#define EVK_CAN2_EN2			IMX_GPIO_NR(4, 4)	/* GPIO_4_4 */
#define EVK_12V_EN			IMX_GPIO_NR(4, 5)	/* GPIO_4_5 */
#define EVK_DISP0_RESET			IMX_GPIO_NR(5, 0)	/* GPIO_5_0 */
#define EVK_USB_HUB_RESET		IMX_GPIO_NR(5, 20)	/* GPIO_5_20 */
#define EVK_TVIN_PWR			IMX_GPIO_NR(5, 23)	/* GPIO_5_23 */
#define EVK_CAN2_EN1			IMX_GPIO_NR(5, 24)	/* GPIO_5_24 */
#define EVK_TVIN_RESET			IMX_GPIO_NR(5, 25)	/* GPIO_5_25 */
#define EVK_OTG_VBUS			IMX_GPIO_NR(6, 6)	/* GPIO_6_6 */
#define EVK_FEC_PHY_RST			IMX_GPIO_NR(7, 6)
#define EVK_USBH1_VBUS			IMX_GPIO_NR(7, 8)	/* GPIO_7_8 */
#define EVK_PMIC_INT			IMX_GPIO_NR(7, 11)	/* GPIO_7_11 */
#define EVK_CAN1_EN1			IMX_GPIO_NR(7, 12)	/* GPIO_7_12 */
#define EVK_CAN1_EN2			IMX_GPIO_NR(7, 13)	/* GPIO_7_13 */

#define ARM2_SD1_CD			IMX_GPIO_NR(1, 1)	/* GPIO_1_1 */
#define ARM2_OTG_VBUS			IMX_GPIO_NR(3, 22)	/* GPIO_3_22 */
#define ARM2_LCD_CONTRAST		IMX_GPIO_NR(4, 20)	/* GPIO_4_20 */

extern char *lp_reg_id;
extern char *gp_reg_id;
extern void mx5_cpu_regulator_init(void);

static iomux_v3_cfg_t mx53common_pads[] = {
	MX53_PAD_EIM_WAIT__GPIO5_0,

	MX53_PAD_EIM_OE__IPU_DI1_PIN7,
	MX53_PAD_EIM_RW__IPU_DI1_PIN8,

	MX53_PAD_EIM_A25__IPU_DI0_D1_CS,

	MX53_PAD_EIM_D16__ECSPI1_SCLK,
	MX53_PAD_EIM_D17__ECSPI1_MISO,
	MX53_PAD_EIM_D18__ECSPI1_MOSI,

	MX53_PAD_EIM_D20__IPU_SER_DISP0_CS,

	MX53_PAD_EIM_D23__IPU_DI0_D0_CS,

	MX53_PAD_EIM_D24__GPIO3_24,
	MX53_PAD_EIM_D26__GPIO3_26,

	MX53_PAD_EIM_D29__IPU_DISPB0_SER_RS,

	MX53_PAD_EIM_D30__IPU_DI0_PIN11,
	MX53_PAD_EIM_D31__IPU_DI0_PIN12,

	MX53_PAD_PATA_DA_1__GPIO7_7,
	MX53_PAD_PATA_DATA4__GPIO2_4,
	MX53_PAD_PATA_DATA5__GPIO2_5,
	MX53_PAD_PATA_DATA6__GPIO2_6,

	MX53_PAD_SD2_CLK__ESDHC2_CLK,
	MX53_PAD_SD2_CMD__ESDHC2_CMD,
	MX53_PAD_SD2_DATA0__ESDHC2_DAT0,
	MX53_PAD_SD2_DATA1__ESDHC2_DAT1,
	MX53_PAD_SD2_DATA2__ESDHC2_DAT2,
	MX53_PAD_SD2_DATA3__ESDHC2_DAT3,
	MX53_PAD_PATA_DATA12__ESDHC2_DAT4,
	MX53_PAD_PATA_DATA13__ESDHC2_DAT5,
	MX53_PAD_PATA_DATA14__ESDHC2_DAT6,
	MX53_PAD_PATA_DATA15__ESDHC2_DAT7,

	MX53_PAD_CSI0_DAT10__UART1_TXD_MUX,
	MX53_PAD_CSI0_DAT11__UART1_RXD_MUX,

	MX53_PAD_PATA_BUFFER_EN__UART2_RXD_MUX,
	MX53_PAD_PATA_DMARQ__UART2_TXD_MUX,
	MX53_PAD_PATA_DIOR__UART2_RTS,
	MX53_PAD_PATA_INTRQ__UART2_CTS,

	MX53_PAD_PATA_CS_0__UART3_TXD_MUX,
	MX53_PAD_PATA_CS_1__UART3_RXD_MUX,

	MX53_PAD_KEY_COL0__AUDMUX_AUD5_TXC,
	MX53_PAD_KEY_ROW0__AUDMUX_AUD5_TXD,
	MX53_PAD_KEY_COL1__AUDMUX_AUD5_TXFS,
	MX53_PAD_KEY_ROW1__AUDMUX_AUD5_RXD,

	MX53_PAD_CSI0_DAT7__GPIO5_25,

	MX53_PAD_GPIO_2__MLB_MLBDAT,
	MX53_PAD_GPIO_3__MLB_MLBCLK,

	MX53_PAD_GPIO_6__MLB_MLBSIG,

	MX53_PAD_GPIO_4__GPIO1_4,
	MX53_PAD_GPIO_7__GPIO1_7,
	MX53_PAD_GPIO_8__GPIO1_8,

	MX53_PAD_GPIO_10__GPIO4_0,

	MX53_PAD_KEY_COL2__CAN1_TXCAN,
	MX53_PAD_KEY_ROW2__CAN1_RXCAN,

	/* CAN1 -- EN */
	MX53_PAD_GPIO_18__GPIO7_13,
	/* CAN1 -- STBY */
	MX53_PAD_GPIO_17__GPIO7_12,
	/* CAN1 -- NERR */
	MX53_PAD_GPIO_5__GPIO1_5,

	MX53_PAD_KEY_COL4__CAN2_TXCAN,
	MX53_PAD_KEY_ROW4__CAN2_RXCAN,

	/* CAN2 -- EN */
	MX53_PAD_CSI0_DAT6__GPIO5_24,
	/* CAN2 -- STBY */
	MX53_PAD_GPIO_14__GPIO4_4,
	/* CAN2 -- NERR */
	MX53_PAD_CSI0_DAT4__GPIO5_22,

	MX53_PAD_GPIO_11__GPIO4_1,
	MX53_PAD_GPIO_12__GPIO4_2,
	MX53_PAD_GPIO_13__GPIO4_3,
	MX53_PAD_GPIO_16__GPIO7_11,
	MX53_PAD_GPIO_19__GPIO4_5,

	/* DI0 display clock */
	MX53_PAD_DI0_DISP_CLK__IPU_DI0_DISP_CLK,

	/* DI0 data enable */
	MX53_PAD_DI0_PIN15__IPU_DI0_PIN15,
	/* DI0 HSYNC */
	MX53_PAD_DI0_PIN2__IPU_DI0_PIN2,
	/* DI0 VSYNC */
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

	MX53_PAD_LVDS0_TX3_P__LDB_LVDS0_TX3,
	MX53_PAD_LVDS0_CLK_P__LDB_LVDS0_CLK,
	MX53_PAD_LVDS0_TX2_P__LDB_LVDS0_TX2,
	MX53_PAD_LVDS0_TX1_P__LDB_LVDS0_TX1,
	MX53_PAD_LVDS0_TX0_P__LDB_LVDS0_TX0,

	MX53_PAD_LVDS1_TX3_P__LDB_LVDS1_TX3,
	MX53_PAD_LVDS1_CLK_P__LDB_LVDS1_CLK,
	MX53_PAD_LVDS1_TX2_P__LDB_LVDS1_TX2,
	MX53_PAD_LVDS1_TX1_P__LDB_LVDS1_TX1,
	MX53_PAD_LVDS1_TX0_P__LDB_LVDS1_TX0,

	/* audio and CSI clock out */
	MX53_PAD_GPIO_0__CCM_SSI_EXT1_CLK,

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
	/* Camera low power */
	MX53_PAD_CSI0_DAT5__GPIO5_23,

	/* esdhc1 */
	MX53_PAD_SD1_CMD__ESDHC1_CMD,
	MX53_PAD_SD1_CLK__ESDHC1_CLK,
	MX53_PAD_SD1_DATA0__ESDHC1_DAT0,
	MX53_PAD_SD1_DATA1__ESDHC1_DAT1,
	MX53_PAD_SD1_DATA2__ESDHC1_DAT2,
	MX53_PAD_SD1_DATA3__ESDHC1_DAT3,

	/* esdhc3 */
	MX53_PAD_PATA_DATA8__ESDHC3_DAT0,
	MX53_PAD_PATA_DATA9__ESDHC3_DAT1,
	MX53_PAD_PATA_DATA10__ESDHC3_DAT2,
	MX53_PAD_PATA_DATA11__ESDHC3_DAT3,
	MX53_PAD_PATA_DATA0__ESDHC3_DAT4,
	MX53_PAD_PATA_DATA1__ESDHC3_DAT5,
	MX53_PAD_PATA_DATA2__ESDHC3_DAT6,
	MX53_PAD_PATA_DATA3__ESDHC3_DAT7,
	MX53_PAD_PATA_RESET_B__ESDHC3_CMD,
	MX53_PAD_PATA_IORDY__ESDHC3_CLK,

	/* FEC pins */
	MX53_PAD_FEC_MDIO__FEC_MDIO,
	MX53_PAD_FEC_REF_CLK__FEC_TX_CLK,
	MX53_PAD_FEC_RX_ER__FEC_RX_ER,
	MX53_PAD_FEC_CRS_DV__FEC_RX_DV,
	MX53_PAD_FEC_RXD1__FEC_RDATA_1,
	MX53_PAD_FEC_RXD0__FEC_RDATA_0,
	MX53_PAD_FEC_TX_EN__FEC_TX_EN,
	MX53_PAD_FEC_TXD1__FEC_TDATA_1,
	MX53_PAD_FEC_TXD0__FEC_TDATA_0,
	MX53_PAD_FEC_MDC__FEC_MDC,

	MX53_PAD_CSI0_DAT8__I2C1_SDA,
	MX53_PAD_CSI0_DAT9__I2C1_SCL,

	MX53_PAD_KEY_COL3__I2C2_SCL,
	MX53_PAD_KEY_ROW3__I2C2_SDA,
};

static iomux_v3_cfg_t mx53evk_pads[] = {
	/* USB OTG USB_OC */
	MX53_PAD_EIM_A24__GPIO5_4,

	/* USB OTG USB_PWR */
	MX53_PAD_EIM_A23__GPIO6_6,

	/* DISPB0_SER_CLK */
	MX53_PAD_EIM_D21__IPU_DISPB0_SER_CLK,

	/* DI0_PIN1 */
	MX53_PAD_EIM_D22__IPU_DISPB0_SER_DIN,

	/* DISP0 I2C ENABLE */
	MX53_PAD_EIM_D28__GPIO3_28,

	/* DISP0 DET */
	MX53_PAD_EIM_D31__GPIO3_31,

	/* SDHC1 SD_CD */
	MX53_PAD_EIM_DA13__GPIO3_13,

	/* SDHC1 SD_WP */
	MX53_PAD_EIM_DA14__GPIO3_14,

	/* SDHC3 SD_CD */
	MX53_PAD_EIM_DA11__GPIO3_11,

	/* SDHC3 SD_WP */
	MX53_PAD_EIM_DA12__GPIO3_12,

	/* PWM backlight */
	MX53_PAD_GPIO_1__PWM2_PWMO,

	/* USB HOST USB_PWR */
	MX53_PAD_PATA_DA_2__GPIO7_8,

	/* USB HOST USB_RST */
	MX53_PAD_CSI0_DATA_EN__GPIO5_20,

	/* USB HOST CARD_ON */
	MX53_PAD_EIM_DA15__GPIO3_15,

	/* USB HOST CARD_RST */
	MX53_PAD_PATA_DATA7__GPIO2_7,

	/* USB HOST WAN_WAKE */
	MX53_PAD_EIM_D25__GPIO3_25,

	/* FEC_RST */
	MX53_PAD_PATA_DA_0__GPIO7_6,
};

static iomux_v3_cfg_t mx53arm2_pads[] = {
	/* USB OTG USB_OC */
	MX53_PAD_EIM_D21__GPIO3_21,

	/* USB OTG USB_PWR */
	MX53_PAD_EIM_D22__GPIO3_22,

	/* SDHC1 SD_CD */
	MX53_PAD_GPIO_1__GPIO1_1,

	/* gpio backlight */
	MX53_PAD_DI0_PIN4__GPIO4_20,
};

static iomux_v3_cfg_t mx53_nand_pads[] = {
	MX53_PAD_NANDF_CLE__EMI_NANDF_CLE,
	MX53_PAD_NANDF_ALE__EMI_NANDF_ALE,
	MX53_PAD_NANDF_WP_B__EMI_NANDF_WP_B,
	MX53_PAD_NANDF_WE_B__EMI_NANDF_WE_B,
	MX53_PAD_NANDF_RE_B__EMI_NANDF_RE_B,
	MX53_PAD_NANDF_RB0__EMI_NANDF_RB_0,
	MX53_PAD_NANDF_CS0__EMI_NANDF_CS_0,
	MX53_PAD_NANDF_CS1__EMI_NANDF_CS_1	,
	MX53_PAD_NANDF_CS2__EMI_NANDF_CS_2,
	MX53_PAD_NANDF_CS3__EMI_NANDF_CS_3	,
	MX53_PAD_EIM_DA0__EMI_NAND_WEIM_DA_0,
	MX53_PAD_EIM_DA1__EMI_NAND_WEIM_DA_1,
	MX53_PAD_EIM_DA2__EMI_NAND_WEIM_DA_2,
	MX53_PAD_EIM_DA3__EMI_NAND_WEIM_DA_3,
	MX53_PAD_EIM_DA4__EMI_NAND_WEIM_DA_4,
	MX53_PAD_EIM_DA5__EMI_NAND_WEIM_DA_5,
	MX53_PAD_EIM_DA6__EMI_NAND_WEIM_DA_6,
	MX53_PAD_EIM_DA7__EMI_NAND_WEIM_DA_7,
};

static const struct imxuart_platform_data mx53_evk_uart_pdata __initconst = {
	.flags = IMXUART_HAVE_RTSCTS,
};

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_RGB565,
};

static struct ipuv3_fb_platform_data evk_fb_data[] = {
	{
	.disp_dev = "lcd",
	.interface_pix_fmt = IPU_PIX_FMT_RGB565,
	.mode_str = "CLAA-WVGA",
	.default_bpp = 16,
	.int_clk = false,
	}, {
	.disp_dev = "vga",
	.interface_pix_fmt = IPU_PIX_FMT_GBR24,
	.mode_str = "VGA-XGA",
	.default_bpp = 16,
	.int_clk = false,
	},
};

static struct imx_ipuv3_platform_data ipu_data = {
	.rev = 3,
};

static struct platform_pwm_backlight_data evk_pwm_backlight_data = {
	.pwm_id = 1,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static struct fsl_mxc_tve_platform_data tve_data = {
	.dac_reg = "VVIDEO",
};

static inline void mx53_evk_init_uart(void)
{
	imx53_add_imx_uart(0, NULL);
	imx53_add_imx_uart(1, &mx53_evk_uart_pdata);
	imx53_add_imx_uart(2, NULL);
}

static const struct imxi2c_platform_data mx53_evk_i2c_data __initconst = {
	.bitrate = 100000,
};

static inline void mx53_evk_fec_reset(void)
{
	int ret;

	/* reset FEC PHY */
	ret = gpio_request_one(MX53_EVK_FEC_PHY_RST, GPIOF_OUT_INIT_LOW,
							"fec-phy-reset");
	if (ret) {
		printk(KERN_ERR"failed to get GPIO_FEC_PHY_RESET: %d\n", ret);
		return;
	}
	msleep(1);
	gpio_set_value(MX53_EVK_FEC_PHY_RST, 1);
}

static struct fec_platform_data mx53_evk_fec_pdata = {
	.phy = PHY_INTERFACE_MODE_RMII,
};

static struct spi_board_info mx53_evk_spi_board_info[] __initdata = {
	{
		.modalias = "mtd_dataflash",
		.max_speed_hz = 25000000,
		.bus_num = 0,
		.chip_select = 1,
		.mode = SPI_MODE_0,
		.platform_data = NULL,
	},
};

static int mx53_evk_spi_cs[] = {
	EVK_ECSPI1_CS0,
	EVK_ECSPI1_CS1,
};

static const struct spi_imx_master mx53_evk_spi_data __initconst = {
	.chipselect     = mx53_evk_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx53_evk_spi_cs),
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
};

static void sii902x_hdmi_reset(void)
{
	gpio_set_value(EVK_DISP0_RESET, 0);
	msleep(10);
	gpio_set_value(EVK_DISP0_RESET, 1);
	msleep(10);
}

static struct fsl_mxc_lcd_platform_data sii902x_hdmi_data = {
	.reset = sii902x_hdmi_reset,
};

static void ddc_dvi_init(void)
{
	/* enable DVI I2C */
	gpio_set_value(EVK_DISP0_I2C_EN, 1);
}

static int ddc_dvi_update(void)
{
	/* DVI cable state */
	if (gpio_get_value(EVK_DISP0_DET_INT) == 1)
		return 1;
	else
		return 0;
}

static struct fsl_mxc_dvi_platform_data evk_ddc_dvi_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.init = ddc_dvi_init,
	.update = ddc_dvi_update,
	.analog_regulator = "VSD",
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
	 .irq  = gpio_to_irq(EVK_TS_INT),
	},
	{
	 .type = "backlight-i2c",
	 .addr = 0x2c,
	 },
	{
	 .type = "mxc_dvi",
	 .addr = 0x50,
	 .irq = gpio_to_irq(EVK_DISP0_DET_INT),
	 .platform_data = &evk_ddc_dvi_data,
	 },
	{
	.type = "sii902x",
	.addr = 0x39,
	.irq = gpio_to_irq(EVK_DISP0_DET_INT),
	.platform_data = &sii902x_hdmi_data,
	},
};

static struct mxc_dvfs_platform_data evk_dvfs_core_data = {
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

static const struct esdhc_platform_data mx53_evk_sd1_data __initconst = {
	.cd_gpio = EVK_SD1_CD,
	.wp_gpio = EVK_SD1_WP,
};

static const struct esdhc_platform_data mx53_evk_sd3_data __initconst = {
	.cd_gpio = EVK_SD3_CD,
	.wp_gpio = EVK_SD3_WP,
};

static int __initdata enable_spdif = { 0 };
static int __init spdif_setup(char *__unused)
{
	enable_spdif = 1;
	return 1;
}
__setup("spdif", spdif_setup);

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
		gpio_request(EVK_FEC_PHY_RST, "fec-phy-reset");
		gpio_direction_output(EVK_FEC_PHY_RST, 0);
		msleep(1);
		gpio_set_value(EVK_FEC_PHY_RST, 1);

		gpio_request(EVK_ESAI_RESET, "fesai-reset");
		gpio_direction_output(EVK_ESAI_RESET, 0);
	}

	/* DISP0 Detect */
	gpio_request(EVK_DISP0_DET_INT, "disp0-detect");
	gpio_direction_input(EVK_DISP0_DET_INT);
	/* DISP0 Reset - Assert for i2c disabled mode */
	gpio_request(EVK_DISP0_RESET, "disp0-reset");
	gpio_direction_output(EVK_DISP0_RESET, 0);

	/* DISP0 Power-down */
	gpio_request(EVK_DISP0_PD, "disp0-pd");
	gpio_direction_output(EVK_DISP0_PD, 1);

	/* DISP0 I2C enable */
	gpio_request(EVK_DISP0_I2C_EN, "disp0-i2c");
	gpio_direction_output(EVK_DISP0_I2C_EN, 0);

	mxc_iomux_v3_setup_multiple_pads(mx53_nand_pads,
					ARRAY_SIZE(mx53_nand_pads));

	gpio_request(EVK_PMIC_INT, "pmic-int");
	gpio_direction_input(EVK_PMIC_INT);	/*PMIC_INT*/

	/* headphone_det_b */
	gpio_request(EVK_HP_DETECT, "hp-detect");
	gpio_direction_input(EVK_HP_DETECT);

	/* power key */

	/* LCD related gpio */

	/* Camera reset */
	gpio_request(EVK_CAM_RESET, "cam-reset");
	gpio_direction_output(EVK_CAM_RESET, 1);

	/* TVIN reset */
	gpio_request(EVK_TVIN_RESET, "tvin-reset");
	gpio_direction_output(EVK_TVIN_RESET, 0);
	msleep(5);
	gpio_set_value(EVK_TVIN_RESET, 1);

	/* TVin power down */
	gpio_request(EVK_TVIN_PWR, "tvin-pwr");
	gpio_direction_output(EVK_TVIN_PWR, 0);

	/* CAN1 enable GPIO*/
	gpio_request(EVK_CAN1_EN1, "can1-en1");
	gpio_direction_output(EVK_CAN1_EN1, 0);

	gpio_request(EVK_CAN1_EN2, "can1-en2");
	gpio_direction_output(EVK_CAN1_EN2, 0);

	/* CAN2 enable GPIO*/
	gpio_request(EVK_CAN2_EN1, "can2-en1");
	gpio_direction_output(EVK_CAN2_EN1, 0);

	gpio_request(EVK_CAN2_EN2, "can2-en2");
	gpio_direction_output(EVK_CAN2_EN2, 0);

	if (enable_spdif) {
		iomux_v3_cfg_t spdif_pin = MX53_PAD_GPIO_19__SPDIF_OUT1;
		mxc_iomux_v3_setup_pad(spdif_pin);
	} else {
		/* GPIO for 12V */
		gpio_request(EVK_12V_EN, "12v-en");
		gpio_direction_output(EVK_12V_EN, 0);
	}
}

static struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx = 1,
	.spdif_rx = 0,
	.spdif_clk_44100 = 0,	/* tx clk from CKIH1 for 44.1K */
	.spdif_clk_48000 = 7,	/* tx clk from CKIH2 for 48k and 32k */
	.spdif_div_44100 = 8,
	.spdif_div_48000 = 8,
	.spdif_div_32000 = 12,
	.spdif_rx_clk = 0,	/* rx clk from spdif stream */
	.spdif_clk = NULL,	/* spdif bus clk */
};

static struct mxc_regulator_platform_data evk_regulator_data = {
	.cpu_reg_id = "SW1",
	.vcc_reg_id = "SW2",
};

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
}

static void __init mx53_evk_board_init(void)
{
	int i;

	mx53_evk_io_init();

	gp_reg_id = evk_regulator_data.cpu_reg_id;
	lp_reg_id = evk_regulator_data.vcc_reg_id;

	mxc_spdif_data.spdif_core_clk = clk_get(NULL, "spdif_xtal_clk");
	clk_put(mxc_spdif_data.spdif_core_clk);

	mx53_evk_init_uart();
	imx53_add_srtc();
	mx53_evk_fec_reset();
	imx53_add_fec(&mx53_evk_fec_pdata);

	imx53_add_ipuv3(0, &ipu_data);
	for (i = 0; i < ARRAY_SIZE(evk_fb_data); i++)
		imx53_add_ipuv3fb(i, &evk_fb_data[i]);
	imx53_add_vpu();
	imx53_add_tve(&tve_data);
	imx53_add_v4l2_output(0);

	if (!board_is_mx53_arm2()) {
		imx53_add_mxc_pwm(1);
		imx53_add_mxc_pwm_backlight(0, &evk_pwm_backlight_data);
	}

	imx53_add_dvfs_core(&evk_dvfs_core_data);
	imx53_add_busfreq();
	imx53_add_imx_i2c(0, &mx53_evk_i2c_data);
	imx53_add_imx_i2c(1, &mx53_evk_i2c_data);
	i2c_register_board_info(0, mxc_i2c0_board_info,
				ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));

	imx53_add_sdhci_esdhc_imx(0, &mx53_evk_sd1_data);
	imx53_add_sdhci_esdhc_imx(2, &mx53_evk_sd3_data);
	mxc_register_device(&imx_ahci_device_hwmon, NULL);

	spi_register_board_info(mx53_evk_spi_board_info,
		ARRAY_SIZE(mx53_evk_spi_board_info));
	imx53_add_ecspi(0, &mx53_evk_spi_data);
	imx53_add_imx2_wdt(0, NULL);

	imx53_add_spdif(&mxc_spdif_data);
	imx53_add_spdif_dai();
	imx53_add_spdif_audio_device();

	mx5_cpu_regulator_init();

	/* this call required to release SCC RAM partition held by ROM
	  * during boot, even if SCC2 driver is not part of the image
	  */
	imx53_add_mxc_scc2();
}

static void __init mx53_evk_timer_init(void)
{
	mx53_clocks_init(32768, 24000000, 22579200, 0);
}

static struct sys_timer mx53_evk_timer = {
	.init	= mx53_evk_timer_init,
};

MACHINE_START(MX53_EVK, "Freescale MX53 EVK Board")
	.fixup = fixup_mxc_board,
	.map_io = mx53_map_io,
	.init_early = imx53_init_early,
	.init_irq = mx53_init_irq,
	.timer = &mx53_evk_timer,
	.init_machine = mx53_evk_board_init,
MACHINE_END
