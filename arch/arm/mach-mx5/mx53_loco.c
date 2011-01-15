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

/*!
 * @file mach-mx5/mx53_loco.c
 *
 * @brief This file contains MX53 loco board specific initialization routines.
 *
 * @ingroup MSL_MX53
 */

/* MX53 LOCO GPIO PIN configurations */
#define NVDD_FAULT			(0*32 + 5)	/* GPIO1_5 */

#define FEC_INT				(1*32 + 4)	/* GPIO_2_4 */
#define HEADPHONE_DEC_B		(1*32 + 5)	/* GPIO_2_5 */
#define MIC_DEC_B			(1*32 + 6)	/* GPIO_2_6 */
#define USER_UI1			(1*32 + 14)	/* GPIO_2_14 */
#define USER_UI2			(1*32 + 15)	/* GPIO_2_15 */
#define MX53_nONKEY			(0*32 + 8)	/* GPIO_1_8 */

#define SD3_CD				(2*32 + 11)	/* GPIO_3_11 */
#define SD3_WP				(2*32 + 12)	/* GPIO_3_12 */
#define DISP0_POWER_EN		(2*32 + 24)	/* GPIO_3_24 */
#define DISP0_DET_INT		(2*32 + 31)	/* GPIO_3_31 */

#define DISP0_RESET			(4*32 + 0)	/* GPIO_5_0 */

#define CSI0_RTSB			(5*32 + 9)	/* GPIO_6_9 */
#define CSI0_PWDN			(5*32 + 10)	/* GPIO_6_10 */
#define ACCL_EN				(5*32 + 14)	/* GPIO_6_14 */
#define ACCL_INT1_IN		(5*32 + 15)	/* GPIO_6_15 */
#define ACCL_INT2_IN		(5*32 + 16)	/* GPIO_6_16 */

#define LCD_BLT_EN			(6*32 + 2)	/* GPIO_7_2 */
#define FEC_RST				(6*32 + 6)	/* GPIO_7_6 */
#define USER_LED_EN			(6*32 + 7)	/* GPIO_7_7 */
#define USB_PWREN			(6*32 + 8)	/* GPIO_7_8 */
#define NIRQ				(6*32 + 11)	/* GPIO7_11 */

extern int __init mx53_loco_init_da9052(void);

static struct pad_desc mx53_loco_pads[] = {
	/* FEC */
	MX53_PAD_FEC_MDC__FEC_MDC,
	MX53_PAD_FEC_MDIO__FEC_MDIO,
	MX53_PAD_FEC_REF_CLK__FEC_REF_CLK,
	MX53_PAD_FEC_RX_ER__FEC_RX_ER,
	MX53_PAD_FEC_CRS_DV__FEC_CRS_DV,
	MX53_PAD_FEC_RXD1__FEC_RXD1,
	MX53_PAD_FEC_RXD0__FEC_RXD0,
	MX53_PAD_FEC_TX_EN__FEC_TX_EN,
	MX53_PAD_FEC_TXD1__FEC_TXD1,
	MX53_PAD_FEC_TXD0__FEC_TXD0,
	/* FEC_nRST */
	MX53_PAD_ATA_DA_0__GPIO_7_6,
	/* FEC_nINT */
	MX53_PAD_ATA_DATA4__GPIO_2_4,
	/* AUDMUX5 */
	MX53_PAD_KEY_COL0__AUD5_TXC,
	MX53_PAD_KEY_ROW0__AUD5_TXD,
	MX53_PAD_KEY_COL1__AUD5_TXFS,
	MX53_PAD_KEY_ROW1__AUD5_RXD,
	/* I2C2 */
	MX53_PAD_KEY_COL3__I2C2_SCL,
	MX53_PAD_KEY_ROW3__I2C2_SDA,
	/* SD1 */
	MX53_PAD_SD1_CMD__SD1_CMD,
	MX53_PAD_SD1_CLK__SD1_CLK,
	MX53_PAD_SD1_DATA0__SD1_DATA0,
	MX53_PAD_SD1_DATA1__SD1_DATA1,
	MX53_PAD_SD1_DATA2__SD1_DATA2,
	MX53_PAD_SD1_DATA3__SD1_DATA3,
	/* SD3 */
	MX53_PAD_ATA_DATA8__SD3_DAT0,
	MX53_PAD_ATA_DATA9__SD3_DAT1,
	MX53_PAD_ATA_DATA10__SD3_DAT2,
	MX53_PAD_ATA_DATA11__SD3_DAT3,
	MX53_PAD_ATA_DATA0__SD3_DAT4,
	MX53_PAD_ATA_DATA1__SD3_DAT5,
	MX53_PAD_ATA_DATA2__SD3_DAT6,
	MX53_PAD_ATA_DATA3__SD3_DAT7,
	MX53_PAD_ATA_IORDY__SD3_CLK,
	MX53_PAD_ATA_RESET_B__SD3_CMD,
	/* SD3_CD */
	MX53_PAD_EIM_DA11__GPIO_3_11,
	/* SD3_WP */
	MX53_PAD_EIM_DA12__GPIO_3_12,
	/* VGA */
	MX53_PAD_EIM_OE__DI1_PIN7,
	MX53_PAD_EIM_RW__DI1_PIN8,
	/* DISPLB */
	MX53_PAD_EIM_D20__SER_DISP0_CS,
	MX53_PAD_EIM_D21__DISPB0_SER_CLK,
	MX53_PAD_EIM_D22__DISPB0_SER_DIN,
	MX53_PAD_EIM_D23__DI0_D0_CS,
	/* DISP0_POWER_EN */
	MX53_PAD_EIM_D24__GPIO_3_24,
	/* DISP0 DET INT */
	MX53_PAD_EIM_D31__GPIO_3_31,

	/* LVDS */
	MX53_PAD_LVDS0_TX3_P__LVDS0_TX3,
	MX53_PAD_LVDS0_CLK_P__LVDS0_CLK,
	MX53_PAD_LVDS0_TX2_P__LVDS0_TX2,
	MX53_PAD_LVDS0_TX1_P__LVDS0_TX1,
	MX53_PAD_LVDS0_TX0_P__LVDS0_TX0,
	MX53_PAD_LVDS1_TX3_P__LVDS1_TX3,
	MX53_PAD_LVDS1_TX2_P__LVDS1_TX2,
	MX53_PAD_LVDS1_CLK_P__LVDS1_CLK,
	MX53_PAD_LVDS1_TX1_P__LVDS1_TX1,
	MX53_PAD_LVDS1_TX0_P__LVDS1_TX0,
	/* I2C1 */
	MX53_PAD_CSI0_D8__I2C1_SDA,
	MX53_PAD_CSI0_D9__I2C1_SCL,
	/* UART1 */
	MX53_PAD_CSI0_D10__UART1_TXD,
	MX53_PAD_CSI0_D11__UART1_RXD,
	/* CSI0 */
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
	/* DISPLAY */
	MX53_PAD_DI0_DISP_CLK__DI0_DISP_CLK,
	MX53_PAD_DI0_PIN15__DI0_PIN15,
	MX53_PAD_DI0_PIN2__DI0_PIN2,
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
	/* Audio CLK*/
	MX53_PAD_GPIO_0__SSI_EXT1_CLK,
	/* PWM */
	MX53_PAD_GPIO_1__PWMO,
	/* SPDIF */
	MX53_PAD_GPIO_7__PLOCK,
	MX53_PAD_GPIO_17__SPDIF_OUT1,
	/* GPIO */
	MX53_PAD_ATA_DA_1__GPIO_7_7,
	MX53_PAD_ATA_DA_2__GPIO_7_8,
	MX53_PAD_ATA_DATA5__GPIO_2_5,
	MX53_PAD_ATA_DATA6__GPIO_2_6,
	MX53_PAD_ATA_DATA14__GPIO_2_14,
	MX53_PAD_ATA_DATA15__GPIO_2_15,
	MX53_PAD_ATA_INTRQ__GPIO_7_2,
	MX53_PAD_EIM_WAIT__GPIO_5_0,
	MX53_PAD_NANDF_WP_B__GPIO_6_9,
	MX53_PAD_NANDF_RB0__GPIO_6_10,
	MX53_PAD_NANDF_CS1__GPIO_6_14,
	MX53_PAD_NANDF_CS2__GPIO_6_15,
	MX53_PAD_NANDF_CS3__GPIO_6_16,
	MX53_PAD_GPIO_5__GPIO_1_5,
	MX53_PAD_GPIO_16__GPIO_7_11,
	MX53_PAD_GPIO_8__GPIO_1_8,
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
	.reset = mx5_vpu_reset,
};

static struct fec_platform_data fec_data = {
	.phy = PHY_INTERFACE_MODE_RMII,
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
	 .interface_pix_fmt = IPU_PIX_FMT_RGB565,
	 .mode_str = "CLAA-WVGA",
	 .mode = video_modes,
	 .num_modes = ARRAY_SIZE(video_modes),
	 },
	{
	 .interface_pix_fmt = IPU_PIX_FMT_BGR24,
	 .mode_str = "XGA",
	 .mode = video_modes,
	 .num_modes = ARRAY_SIZE(video_modes),
	 },
};

extern int primary_di;
static int __init mxc_init_fb(void)
{
	if (!machine_is_mx53_loco())
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

static void sii9022_hdmi_reset(void)
{
	gpio_set_value(DISP0_RESET, 0);
	msleep(10);
	gpio_set_value(DISP0_RESET, 1);
	msleep(10);
}

static struct mxc_lcd_platform_data sii9022_hdmi_data = {
       .reset = sii9022_hdmi_reset,
};

static struct imxi2c_platform_data mxci2c_data = {
       .bitrate = 100000,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
	.type = "mma8450",
	.addr = 0x1C,
	 },
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
	 .type = "sgtl5000-i2c",
	 .addr = 0x0a,
	 },
	{
	 .type = "sii9022",
	 .addr = 0x39,
	 .irq = IOMUX_TO_IRQ_V3(DISP0_DET_INT),
	 .platform_data = &sii9022_hdmi_data,
	},
};

static int sdhc_write_protect(struct device *dev)
{
	int ret = 0;

	if (to_platform_device(dev)->id == 2)
		ret = gpio_get_value(SD3_WP);

	return ret;
}

static unsigned int sdhc_get_card_det_status(struct device *dev)
{
	int ret = 0;

	if (to_platform_device(dev)->id == 2)
		ret = gpio_get_value(SD3_CD);

	return ret;
}

static struct mxc_mmc_platform_data mmc1_data = {
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
	.caps = MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA
		| MMC_CAP_DATA_DDR,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 0,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
};

static int headphone_det_status(void)
{
	return (gpio_get_value(HEADPHONE_DEC_B) == 0);
}

static int mxc_sgtl5000_init(void);

static struct mxc_audio_platform_data sgtl5000_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 5,
	.hp_irq = IOMUX_TO_IRQ_V3(HEADPHONE_DEC_B),
	.hp_status = headphone_det_status,
	.init = mxc_sgtl5000_init,
};

static int mxc_sgtl5000_init(void)
{
	struct clk *ssi_ext1;
	int rate;

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
	.spdif_clk_44100 = -1,	/* Souce from CKIH1 for 44.1K */
	/* Source from CCM spdif_clk (24M) for 48k and 32k
	 * It's not accurate
	 */
	.spdif_clk_48000 = 1,
	.spdif_clkid = 0,
	.spdif_clk = NULL,	/* spdif bus clk */
};

static void mx53_loco_usbh1_vbus(bool on)
{
	if (on)
		gpio_set_value(USB_PWREN, 1);
	else
		gpio_set_value(USB_PWREN, 0);
}

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

static struct gpio_keys_button loco_buttons[] = {
	GPIO_BUTTON(MX53_nONKEY, KEY_POWER, 1, "power", 1),
	GPIO_BUTTON(USER_UI1, KEY_VOLUMEUP, 1, "volume-up", 0),
	GPIO_BUTTON(USER_UI2, KEY_VOLUMEDOWN, 1, "volume-down", 0),
};

static struct gpio_keys_platform_data loco_button_data = {
	.buttons	= loco_buttons,
	.nbuttons	= ARRAY_SIZE(loco_buttons),
};

static struct platform_device loco_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &loco_button_data,
	}
};

static void __init loco_add_device_buttons(void)
{
	platform_device_register(&loco_button_device);
}
#else
static void __init loco_add_device_buttons(void) {}
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

static void __init mx53_loco_io_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx53_loco_pads,
					ARRAY_SIZE(mx53_loco_pads));

	/* SD3 */
	gpio_request(SD3_CD, "sd3-cd");
	gpio_direction_input(SD3_CD);
	gpio_request(SD3_WP, "sd3-wp");
	gpio_direction_input(SD3_WP);

	/* reset FEC PHY */
	gpio_request(FEC_RST, "fec-rst");
	gpio_direction_output(FEC_RST, 0);
	gpio_set_value(FEC_RST, 0);
	msleep(1);
	gpio_set_value(FEC_RST, 1);

	/* headphone_det_b */
	gpio_request(HEADPHONE_DEC_B, "headphone-dec");
	gpio_direction_input(HEADPHONE_DEC_B);

	/* USB PWR enable */
	gpio_request(USB_PWREN, "usb-pwr");
	gpio_direction_output(USB_PWREN, 0);

	/* Sii9022 HDMI controller */
	gpio_request(DISP0_RESET, "disp0-reset");
	gpio_direction_output(DISP0_RESET, 0);
	gpio_request(DISP0_DET_INT, "disp0-detect");
	gpio_direction_input(DISP0_DET_INT);
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

	mxcsdhc3_device.resource[2].start = IOMUX_TO_IRQ_V3(SD3_CD);
	mxcsdhc3_device.resource[2].end = IOMUX_TO_IRQ_V3(SD3_CD);

	mxc_cpu_common_init();
	mx53_loco_io_init();

	mxc_register_device(&mxc_dma_device, NULL);
	mxc_register_device(&mxc_wdt_device, NULL);
	mxc_register_device(&mxci2c_devices[0], &mxci2c_data);
	mxc_register_device(&mxci2c_devices[1], &mxci2c_data);

	mx53_loco_init_da9052();

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
	mxc_register_device(&mxcsdhc3_device, &mmc3_data);
	mxc_register_device(&mxc_ssi1_device, NULL);
	mxc_register_device(&mxc_ssi2_device, NULL);
	mxc_register_device(&mxc_alsa_spdif_device, &mxc_spdif_data);
	mxc_register_device(&ahci_fsl_device, &sata_data);
	mxc_register_device(&mxc_fec_device, &fec_data);
	/* ASRC is only available for MX53 TO2.0 */
	if (cpu_is_mx53_rev(CHIP_REV_2_0) >= 1) {
		mxc_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
		clk_put(mxc_asrc_data.asrc_core_clk);
		mxc_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
		clk_set_rate(mxc_asrc_data.asrc_audio_clk, 1190000);
		clk_put(mxc_asrc_data.asrc_audio_clk);
		mxc_register_device(&mxc_asrc_device, &mxc_asrc_data);
	}

	i2c_register_board_info(0, mxc_i2c0_board_info,
				ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));

	mxc_register_device(&mxc_sgtl5000_device, &sgtl5000_data);
	mx5_usb_dr_init();
	mx5_set_host1_vbus_func(mx53_loco_usbh1_vbus);
	mx5_usbh1_init();
	mxc_register_device(&mxc_v4l2_device, NULL);
	mxc_register_device(&mxc_v4l2out_device, NULL);
	loco_add_device_buttons();
}

static void __init mx53_loco_timer_init(void)
{
	struct clk *uart_clk;

	mx53_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("mxcintuart.0", NULL);
	early_console_setup(MX53_BASE_ADDR(UART1_BASE_ADDR), uart_clk);
}

static struct sys_timer mxc_timer = {
	.init	= mx53_loco_timer_init,
};

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX53_LOCO data structure.
 */
MACHINE_START(MX53_LOCO, "Freescale MX53 LOCO Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.fixup = fixup_mxc_board,
	.map_io = mx5_map_io,
	.init_irq = mx5_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END
