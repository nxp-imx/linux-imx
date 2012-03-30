/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/mxcfb.h>
#include <linux/ipu.h>
#include <linux/pwm_backlight.h>
#include <linux/smsc911x.h>
#include <linux/i2c/pca953x.h>
#include <linux/regulator/consumer.h>
#include <sound/pcm.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/imx-uart.h>
#include <mach/iomux-mx53.h>
#include <mach/ipu-v3.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "crm_regs.h"
#include "devices-imx53.h"

#define ARD_SD1_CD IMX_GPIO_NR(1, 1)
#define ARD_SD1_WP IMX_GPIO_NR(1, 9)
#define ARD_SD2_CD IMX_GPIO_NR(1, 4)
#define ARD_SD2_WP IMX_GPIO_NR(1, 2)
#define ARD_ESAI_INT IMX_GPIO_NR(2, 4)
#define ARD_TS_INT IMX_GPIO_NR(7, 12)
#define ARD_ETHERNET_INT_B IMX_GPIO_NR(2, 31)

/* Start directly after the CPU's GPIO*/
#define MAX7310_BASE_ADDR		224	/* 7x32 */
#define ARD_BACKLIGHT_ON		MAX7310_BASE_ADDR
#define ARD_SPARE			    (MAX7310_BASE_ADDR + 1)
#define ARD_CPU_PER_RST_B		(MAX7310_BASE_ADDR + 2)
#define ARD_MAIN_PER_RST_B		(MAX7310_BASE_ADDR + 3)
#define ARD_IPOD_RST_B			(MAX7310_BASE_ADDR + 4)
#define ARD_MLB_RST_B			(MAX7310_BASE_ADDR + 5)
#define ARD_SSI_STEERING		(MAX7310_BASE_ADDR + 6)
#define ARD_GPS_RST_B			(MAX7310_BASE_ADDR + 7)

extern char *gp_reg_id;
extern char *lp_reg_id;
extern void mx5_cpu_regulator_init(void);

static iomux_v3_cfg_t mx53_ard_pads[] = {
	/* UART */
	MX53_PAD_PATA_DIOW__UART1_TXD_MUX,
	MX53_PAD_PATA_DMACK__UART1_RXD_MUX,
	MX53_PAD_PATA_BUFFER_EN__UART2_RXD_MUX,
	MX53_PAD_PATA_DMARQ__UART2_TXD_MUX,
	MX53_PAD_PATA_DIOR__UART2_RTS,
	MX53_PAD_PATA_INTRQ__UART2_CTS,
	MX53_PAD_PATA_CS_0__UART3_TXD_MUX,
	MX53_PAD_PATA_CS_1__UART3_RXD_MUX,
	MX53_PAD_PATA_DA_1__UART3_CTS,
	MX53_PAD_PATA_DA_2__UART3_RTS,

	/* SDHC1 */
	MX53_PAD_SD1_CMD__ESDHC1_CMD,
	MX53_PAD_SD1_CLK__ESDHC1_CLK,
	MX53_PAD_SD1_DATA0__ESDHC1_DAT0,
	MX53_PAD_SD1_DATA1__ESDHC1_DAT1,
	MX53_PAD_SD1_DATA2__ESDHC1_DAT2,
	MX53_PAD_SD1_DATA3__ESDHC1_DAT3,
	MX53_PAD_PATA_DATA8__ESDHC1_DAT4,
	MX53_PAD_PATA_DATA9__ESDHC1_DAT5,
	MX53_PAD_PATA_DATA10__ESDHC1_DAT6,
	MX53_PAD_PATA_DATA11__ESDHC1_DAT7,
	MX53_PAD_GPIO_1__GPIO1_1,
	MX53_PAD_GPIO_9__GPIO1_9,

	/* SDHC2 */
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
	MX53_PAD_GPIO_4__GPIO1_4,
	MX53_PAD_GPIO_2__GPIO1_2,

	/* WEIM for CS1 */
	/* ETHERNET_INT_B */
	MX53_PAD_EIM_EB3__GPIO2_31,
	MX53_PAD_EIM_D16__EMI_WEIM_D_16,
	MX53_PAD_EIM_D17__EMI_WEIM_D_17,
	MX53_PAD_EIM_D18__EMI_WEIM_D_18,
	MX53_PAD_EIM_D19__EMI_WEIM_D_19,
	MX53_PAD_EIM_D20__EMI_WEIM_D_20,
	MX53_PAD_EIM_D21__EMI_WEIM_D_21,
	MX53_PAD_EIM_D22__EMI_WEIM_D_22,
	MX53_PAD_EIM_D23__EMI_WEIM_D_23,
	MX53_PAD_EIM_D24__EMI_WEIM_D_24,
	MX53_PAD_EIM_D25__EMI_WEIM_D_25,
	MX53_PAD_EIM_D26__EMI_WEIM_D_26,
	MX53_PAD_EIM_D27__EMI_WEIM_D_27,
	MX53_PAD_EIM_D28__EMI_WEIM_D_28,
	MX53_PAD_EIM_D29__EMI_WEIM_D_29,
	MX53_PAD_EIM_D30__EMI_WEIM_D_30,
	MX53_PAD_EIM_D31__EMI_WEIM_D_31,
	MX53_PAD_EIM_DA0__EMI_NAND_WEIM_DA_0,
	MX53_PAD_EIM_DA1__EMI_NAND_WEIM_DA_1,
	MX53_PAD_EIM_DA2__EMI_NAND_WEIM_DA_2,
	MX53_PAD_EIM_DA3__EMI_NAND_WEIM_DA_3,
	MX53_PAD_EIM_DA4__EMI_NAND_WEIM_DA_4,
	MX53_PAD_EIM_DA5__EMI_NAND_WEIM_DA_5,
	MX53_PAD_EIM_DA6__EMI_NAND_WEIM_DA_6,
	MX53_PAD_EIM_OE__EMI_WEIM_OE,
	MX53_PAD_EIM_RW__EMI_WEIM_RW,
	MX53_PAD_EIM_CS1__EMI_WEIM_CS_1,
	/* I2C2 */
	MX53_PAD_EIM_EB2__I2C2_SCL,
	MX53_PAD_KEY_ROW3__I2C2_SDA,

	/* I2C3 */
	MX53_PAD_GPIO_3__I2C3_SCL,
	MX53_PAD_GPIO_16__I2C3_SDA,

	/* TOUCH_INT_B */
	MX53_PAD_GPIO_17__GPIO7_12,

	/* MAINBRD_SPDIF_IN */
	MX53_PAD_KEY_COL3__SPDIF_IN1,
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
	/* PWM */
	MX53_PAD_DISP0_DAT8__PWM1_PWMO,
	MX53_PAD_DISP0_DAT9__PWM2_PWMO
};

/* Config CS1 settings for ethernet controller */
static void weim_cs_config(void)
{
	u32 reg;
	void __iomem *weim_base, *iomuxc_base;
	weim_base = ioremap(MX53_WEIM_BASE_ADDR, SZ_4K);
	iomuxc_base = ioremap(MX53_IOMUXC_BASE_ADDR, SZ_4K);

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

static struct resource ard_smsc911x_resources[] = {
	{
	 .start = MX53_CS1_BASE_ADDR,
	 .end = MX53_CS1_BASE_ADDR + SZ_4K - 1,
	 .flags = IORESOURCE_MEM,
	},
	{
	 .start =  gpio_to_irq(ARD_ETHERNET_INT_B),
	 .end =  gpio_to_irq(ARD_ETHERNET_INT_B),
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

static iomux_v3_cfg_t mx53_ard_esai_pads[] = {
	MX53_PAD_FEC_MDIO__ESAI1_SCKR,
	MX53_PAD_FEC_REF_CLK__ESAI1_FSR,
	MX53_PAD_FEC_CRS_DV__ESAI1_SCKT,
	MX53_PAD_FEC_RXD1__ESAI1_FST,
	MX53_PAD_FEC_TX_EN__ESAI1_TX3_RX2,
	MX53_PAD_GPIO_5__ESAI1_TX2_RX3,
	MX53_PAD_FEC_TXD0__ESAI1_TX4_RX1,
	MX53_PAD_GPIO_8__ESAI1_TX5_RX0,
	MX53_PAD_NANDF_CS2__ESAI1_TX0,
	MX53_PAD_NANDF_CS3__ESAI1_TX1,
	MX53_PAD_PATA_DATA4__GPIO2_4,
};

void ard_gpio_activate_esai_ports(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx53_ard_esai_pads,
					ARRAY_SIZE(mx53_ard_esai_pads));
	/* ESAI_INT */
	gpio_request(ARD_ESAI_INT, "esai-int");
	gpio_direction_input(ARD_ESAI_INT);

}

static struct imx_esai_platform_data esai_data = {
	.flags = IMX_ESAI_NET,
};

static struct platform_device mxc_alsa_surround_device = {
	.name = "imx-cs42888",
};

static struct mxc_audio_platform_data mxc_surround_audio_data = {
	.ext_ram = 1,
	.sysclk = 24576000,
};

static struct mxc_audio_codec_platform_data cs42888_data = {
	.rates = (SNDRV_PCM_RATE_48000 |
			SNDRV_PCM_RATE_96000 |
			SNDRV_PCM_RATE_192000),
};

static int imx53_init_audio(void)
{
	ard_gpio_activate_esai_ports();

	mxc_register_device(&mxc_alsa_surround_device,
		&mxc_surround_audio_data);
	imx53_add_imx_esai(0, &esai_data);

    return 0;
}

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

static const struct imxuart_platform_data mx53_ard_uart_data __initconst = {
	.flags = IMXUART_HAVE_RTSCTS,
};

static const struct esdhc_platform_data mx53_ard_sd1_data __initconst = {
	.cd_gpio = ARD_SD1_CD,
	.wp_gpio = ARD_SD1_WP,
};

static const struct esdhc_platform_data mx53_ard_sd2_data __initconst = {
	.cd_gpio = ARD_SD2_CD,
	.wp_gpio = ARD_SD2_WP,
};

static struct imxi2c_platform_data mx53_ard_i2c1_data = {
	.bitrate = 50000,
};

static struct imxi2c_platform_data mx53_ard_i2c2_data = {
	.bitrate = 400000,
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
    .type = "cs42888",
	.addr = 0x48,
	.platform_data = (void *)&cs42888_data,
	},
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("max11801", 0x49),
		.irq  = gpio_to_irq(ARD_TS_INT),
	},
	{
	 .type = "max7310",
	 .addr = 0x18,
	 .platform_data = &mx53_i2c_max7310_platdata,
	},
};

static struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx = 0,
	.spdif_rx = 1,
	.spdif_clk_44100 = 0,	/* tx clk from CKIH1 for 44.1K */
	.spdif_clk_48000 = 7,	/* tx clk from CKIH2 for 48k and 32k */
	.spdif_div_44100 = 8,
	.spdif_div_48000 = 8,
	.spdif_div_32000 = 12,
	.spdif_rx_clk = 0,	/* rx clk from spdif stream */
	.spdif_clk = NULL,	/* spdif bus clk */
};

static struct mxc_regulator_platform_data ard_regulator_data = {
	.cpu_reg_id = "SW1",
};

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
}

static inline void mx53_ard_init_uart(void)
{
	imx53_add_imx_uart(0, NULL);
	imx53_add_imx_uart(1, &mx53_ard_uart_data);
	imx53_add_imx_uart(2, &mx53_ard_uart_data);
}

static struct ipuv3_fb_platform_data ard_fb_data[] = {
	{
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	},
	{
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

static struct platform_pwm_backlight_data ard_pwm1_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static struct platform_pwm_backlight_data ard_pwm2_backlight_data = {
	.pwm_id = 1,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static struct fsl_mxc_tve_platform_data tve_data = {
	.dac_reg = "LDO4",
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.ext_ref = 1,
	.mode = LDB_SIN0,
};

static void __init mx53_ard_io_init(void)
{
	/* MX53 ARD board */
	pr_info("MX53 ARD board \n");
	gpio_request(ARD_ETHERNET_INT_B, "eth-int-b");
	gpio_direction_input(ARD_ETHERNET_INT_B);
}

static int __initdata enable_ard_vga = { 0 };
static int __init ard_vga_setup(char *__unused)
{
	enable_ard_vga = 1;
	printk(KERN_INFO "Enable MX53 ARD VGA\n");
	return cpu_is_mx53();
}
__setup("ard-vga", ard_vga_setup);

static void __init mx53_ard_board_init(void)
{
	int i;
	mxc_iomux_v3_setup_multiple_pads(mx53_ard_pads,
					ARRAY_SIZE(mx53_ard_pads));

	/* setup VGA PINs */
	if (enable_ard_vga) {
			iomux_v3_cfg_t vga;
			vga = MX53_PAD_EIM_OE__IPU_DI1_PIN7;
			mxc_iomux_v3_setup_pad(vga);
			vga = MX53_PAD_EIM_RW__IPU_DI1_PIN8;
			mxc_iomux_v3_setup_pad(vga);
	}

	gp_reg_id = ard_regulator_data.cpu_reg_id;
	lp_reg_id = ard_regulator_data.vcc_reg_id;

	mxc_spdif_data.spdif_core_clk = clk_get(NULL, "spdif_xtal_clk");
	clk_put(mxc_spdif_data.spdif_core_clk);
	mx53_ard_init_uart();

	imx53_add_ipuv3(0, &ipu_data);
	for (i = 0; i < ARRAY_SIZE(ard_fb_data); i++)
			imx53_add_ipuv3fb(i, &ard_fb_data[i]);

	imx53_add_vpu();
	imx53_add_ldb(&ldb_data);
	imx53_add_tve(&tve_data);
	imx53_add_v4l2_output(0);

	imx53_add_mxc_pwm(0);
	imx53_add_mxc_pwm_backlight(0, &ard_pwm1_backlight_data);
	imx53_add_mxc_pwm(1);
	imx53_add_mxc_pwm_backlight(1, &ard_pwm2_backlight_data);

	imx53_add_srtc();
	imx53_add_imx2_wdt(0, NULL);
	imx53_add_sdhci_esdhc_imx(0, &mx53_ard_sd1_data);
	imx53_add_sdhci_esdhc_imx(1, &mx53_ard_sd2_data);
	mxc_register_device(&imx_ahci_device_hwmon, NULL);

	weim_cs_config();
	mx53_ard_io_init();
	mxc_register_device(&ard_smsc_lan9220_device, &ard_smsc911x_config);
	imx53_add_imx_i2c(1, &mx53_ard_i2c1_data);
	imx53_add_imx_i2c(2, &mx53_ard_i2c2_data);

	imx53_add_spdif(&mxc_spdif_data);
	imx53_add_spdif_dai();
	imx53_add_spdif_audio_device();

	i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
				ARRAY_SIZE(mxc_i2c2_board_info));

	imx53_init_audio();

	mx5_cpu_regulator_init();

	/* this call required to release SCC RAM partition held by ROM
	  * during boot, even if SCC2 driver is not part of the image
	  */
	imx53_add_mxc_scc2();
}

static void __init mx53_ard_timer_init(void)
{
	mx53_clocks_init(32768, 24000000, 22579200, 0);
}

static struct sys_timer mx53_ard_timer = {
	.init	= mx53_ard_timer_init,
};

MACHINE_START(MX53_ARD, "Freescale MX53 ARD Board")
	.fixup = fixup_mxc_board,
	.map_io = mx53_map_io,
	.init_early = imx53_init_early,
	.init_irq = mx53_init_irq,
	.timer = &mx53_ard_timer,
	.init_machine = mx53_ard_board_init,
MACHINE_END
