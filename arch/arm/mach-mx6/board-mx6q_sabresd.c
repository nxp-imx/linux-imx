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
#include <linux/spi/flash.h>
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
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/etherdevice.h>
#include <linux/power/max8903_charger.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/wm8994/pdata.h>
#include <linux/mfd/wm8994/gpio.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-mx6q_sabresd.h"

#define SABRESD_SD2_CD		IMX_GPIO_NR(2, 2)
#define SABRESD_SD2_WP		IMX_GPIO_NR(2, 3)
#define SABRESD_SD3_CD		IMX_GPIO_NR(2, 0)
#define SABRESD_SD3_WP		IMX_GPIO_NR(2, 1)
#define SABRESD_ECSPI1_CS1	IMX_GPIO_NR(3, 19)
#define SABRESD_USB_OTG_PWR	IMX_GPIO_NR(3, 22)
#define SABRESD_CAP_TCH_INT1	IMX_GPIO_NR(6, 7)
#define SABRESD_CAP_TCH_INT0	IMX_GPIO_NR(6, 8)
#define SABRESD_USB_HUB_RESET	IMX_GPIO_NR(7, 12)
#define SABRESD_CAN1_STBY	IMX_GPIO_NR(1, 2)
#define SABRESD_CAN1_EN		IMX_GPIO_NR(1, 4)
#define SABRESD_VOLUME_UP	IMX_GPIO_NR(1, 4)
#define SABRESD_VOLUME_DN	IMX_GPIO_NR(1, 5)
#define SABRESD_POWER_OFF	IMX_GPIO_NR(3, 29)
#define SABRESD_PFUZE_INT	IMX_GPIO_NR(7, 13)
#define SABRESD_CSI0_RST	IMX_GPIO_NR(1, 17)
#define SABRESD_CSI0_PWN	IMX_GPIO_NR(1, 16)
#define SABRESD_MIPICSI_RST	IMX_GPIO_NR(1, 20)
#define SABRESD_MIPICSI_PWN	IMX_GPIO_NR(1, 19)
#define SABRESD_AUX_5V_EN	IMX_GPIO_NR(6, 10)
#define SABRESD_SENSOR_EN	IMX_GPIO_NR(2, 31)
#define SABRESD_eCOMPASS_INT	IMX_GPIO_NR(3, 16)
#define SABRESD_ALS_INT		IMX_GPIO_NR(3, 9)

#define SABRESD_CHARGE_FLT_1_B	IMX_GPIO_NR(5, 2)
#define SABRESD_CHARGE_CHG_1_B	IMX_GPIO_NR(3, 23)
#define SABRESD_CHARGE_FLT_2_B	IMX_GPIO_NR(3, 14)
#define SABRESD_CHARGE_CHG_2_B	IMX_GPIO_NR(3, 13)
#define SABRESD_CHARGE_UOK_B	IMX_GPIO_NR(1, 27)
#define SABRESD_CHARGE_DOK_B	IMX_GPIO_NR(2, 24)
#define SABRESD_WM8958_4V2_EN	IMX_GPIO_NR(4, 10)
#define SABRESD_WM8958_HP_DET	IMX_GPIO_NR(7, 8)
#define SABRESD_GPS_EN		IMX_GPIO_NR(3, 0)
#define SABRESD_AUX_3V15_EN	IMX_GPIO_NR(6, 9)

static struct clk *sata_clk;
static int mma8451_position = 3;
static int mag3110_position;

extern char *gp_reg_id;

static const struct esdhc_platform_data mx6q_sabresd_sd2_data __initconst = {
	.cd_gpio = SABRESD_SD2_CD,
	.wp_gpio = SABRESD_SD2_WP,
	.keep_power_at_suspend = 1,
	.support_8bit = 1,
	.delay_line = 0,
};

static const struct esdhc_platform_data mx6q_sabresd_sd3_data __initconst = {
	.cd_gpio = SABRESD_SD3_CD,
	.wp_gpio = SABRESD_SD3_WP,
	.keep_power_at_suspend = 1,
	.support_8bit = 1,
	.delay_line = 0,
};

static const struct esdhc_platform_data mx6q_sabresd_sd4_data __initconst = {
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.support_8bit = 1,
	.delay_line = 0,
};

static const struct anatop_thermal_platform_data
	mx6q_sabresd_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

static inline void mx6q_sabresd_init_uart(void)
{
	imx6q_add_imx_uart(2, NULL);
	imx6q_add_imx_uart(0, NULL);
}

static int mx6q_sabresd_fec_phy_init(struct phy_device *phydev)
{
	/* prefer master mode, disable 1000 Base-T capable */
	phy_write(phydev, 0x9, 0x1c00);

	/* min rx data delay */
	phy_write(phydev, 0x0b, 0x8105);
	phy_write(phydev, 0x0c, 0x0000);

	/* max rx/tx clock delay, min rx/tx control delay */
	phy_write(phydev, 0x0b, 0x8104);
	phy_write(phydev, 0x0c, 0xf0f0);
	phy_write(phydev, 0x0b, 0x104);

	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init = mx6q_sabresd_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RGMII,
};

static int mx6q_sabresd_spi_cs[] = {
	SABRESD_ECSPI1_CS1,
};

static const struct spi_imx_master mx6q_sabresd_spi_data __initconst = {
	.chipselect     = mx6q_sabresd_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6q_sabresd_spi_cs),
};

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct mtd_partition imx6_sabresd_spi_nor_partitions[] = {
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

static struct flash_platform_data imx6_sabresd__spi_flash_data = {
	.name = "m25p80",
	.parts = imx6_sabresd_spi_nor_partitions,
	.nr_parts = ARRAY_SIZE(imx6_sabresd_spi_nor_partitions),
	.type = "sst25vf016b",
};
#endif

static struct spi_board_info imx6_sabresd_spi_nor_device[] __initdata = {
#if defined(CONFIG_MTD_M25P80)
	{
		.modalias = "m25p80",
		.max_speed_hz = 20000000, /* max spi clock (SCK) speed in HZ */
		.bus_num = 0,
		.chip_select = 0,
		.platform_data = &imx6_sabresd__spi_flash_data,
	},
#endif
};

static void spi_device_init(void)
{
	spi_register_board_info(imx6_sabresd_spi_nor_device,
				ARRAY_SIZE(imx6_sabresd_spi_nor_device));
}

static struct imx_ssi_platform_data mx6_sabresd_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct platform_device mx6_sabresd_audio_device = {
	.name = "imx-wm8958",
};

static struct mxc_audio_platform_data wm8958_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.hp_gpio = SABRESD_WM8958_HP_DET,
	.hp_active_low = 1,
};

static struct wm8994_pdata wm8958_pdata = {
	.gpio_defaults = {
		[0] = WM8994_GP_FN_GPIO | WM8994_GPN_DB,
		[1] = WM8994_GP_FN_GPIO | WM8994_GPN_DB | WM8994_GPN_PD,
		[2] = WM8994_GP_FN_GPIO | WM8994_GPN_DB | WM8994_GPN_PD,
		[3] = WM8994_GP_FN_GPIO | WM8994_GPN_DB | WM8994_GPN_PD,
		[4] = WM8994_GP_FN_GPIO | WM8994_GPN_DB | WM8994_GPN_PD,
		[5] = WM8994_GP_FN_GPIO | WM8994_GPN_DB | WM8994_GPN_PD,
		[7] = WM8994_GP_FN_GPIO | WM8994_GPN_DB | WM8994_GPN_PD,
		[8] = WM8994_GP_FN_GPIO | WM8994_GPN_DB | WM8994_GPN_PD,
		[9] = WM8994_GP_FN_GPIO | WM8994_GPN_DB | WM8994_GPN_PD,
		[10] = WM8994_GP_FN_GPIO | WM8994_GPN_DB | WM8994_GPN_PD,
	},
};

static int mxc_wm8958_init(void)
{
	struct clk *clko;
	int rate;

	clko = clk_get(NULL, "clko_clk");
	if (IS_ERR(clko)) {
		pr_err("can't get CLKO clock.\n");
		return PTR_ERR(clko);
	}
	/* both audio codec and comera use CLKO clk*/
	rate = clk_round_rate(clko, 22000000);

	wm8958_data.sysclk = rate;
	clk_set_rate(clko, rate);

	/* enable wm8958 4.2v power supply */
	gpio_request(SABRESD_WM8958_4V2_EN, "aud_4v2");
	gpio_direction_output(SABRESD_WM8958_4V2_EN, 1);
	msleep(1);
	gpio_set_value(SABRESD_WM8958_4V2_EN, 1);

	return 0;
}

static void mx6q_csi0_cam_powerdown(int powerdown)
{
	if (powerdown)
		gpio_set_value(SABRESD_CSI0_PWN, 1);
	else
		gpio_set_value(SABRESD_CSI0_PWN, 0);
}

static void mx6q_csi0_io_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx6q_sabresd_csi0_sensor_pads,
			ARRAY_SIZE(mx6q_sabresd_csi0_sensor_pads));

	/* Camera reset */
	gpio_request(SABRESD_CSI0_RST, "cam-reset");
	gpio_direction_output(SABRESD_MIPICSI_RST, 1);

	/* Camera power down */
	gpio_request(SABRESD_CSI0_PWN, "cam-pwdn");
	gpio_direction_output(SABRESD_CSI0_PWN, 1);
	msleep(1);
	gpio_set_value(SABRESD_CSI0_PWN, 0);

	/* For MX6Q GPR1 bit19 and bit20 meaning:
	 * Bit19:       0 - Enable mipi to IPU1 CSI0
	 *                      virtual channel is fixed to 0
	 *              1 - Enable parallel interface to IPU1 CSI0
	 * Bit20:       0 - Enable mipi to IPU2 CSI1
	 *                      virtual channel is fixed to 3
	 *              1 - Enable parallel interface to IPU2 CSI1
	 * IPU1 CSI1 directly connect to mipi csi2,
	 *      virtual channel is fixed to 1
	 * IPU2 CSI0 directly connect to mipi csi2,
	 *      virtual channel is fixed to 2
	 */
	mxc_iomux_set_gpr_register(1, 19, 1, 1);
}

static struct fsl_mxc_camera_platform_data camera_data = {
	.mclk = 22000000,
	.csi = 0,
	.io_init = mx6q_csi0_io_init,
	.pwdn = mx6q_csi0_cam_powerdown,
};

static void mx6q_mipi_powerdown(int powerdown)
{
	if (powerdown)
		gpio_set_value(SABRESD_MIPICSI_PWN, 1);
	else
		gpio_set_value(SABRESD_MIPICSI_PWN, 0);
}

static void mx6q_mipi_sensor_io_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx6q_sabresd_mipi_sensor_pads,
			ARRAY_SIZE(mx6q_sabresd_mipi_sensor_pads));

	/* Camera reset */
	gpio_request(SABRESD_MIPICSI_RST, "cam-reset");
	gpio_direction_output(SABRESD_MIPICSI_RST, 1);

	/* Camera power down */
	gpio_request(SABRESD_MIPICSI_PWN, "cam-pwdn");
	gpio_direction_output(SABRESD_MIPICSI_PWN, 1);
	msleep(1);
	gpio_set_value(SABRESD_MIPICSI_PWN, 0);

	mxc_iomux_set_gpr_register(1, 19, 1, 0);
}

static struct fsl_mxc_camera_platform_data mipi_csi2_data = {
	.mclk = 24000000,
	.csi = 0,
	.io_init = mx6q_mipi_sensor_io_init,
	.pwdn = mx6q_mipi_powerdown,
};

static struct imxi2c_platform_data mx6q_sabresd_i2c_data = {
	.bitrate = 100000,
};

static struct fsl_mxc_lightsensor_platform_data ls_data = {
	.rext = 499,	/* calibration: 499K->700K */
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("wm8958", 0x1a),
		.platform_data = &wm8958_pdata,
	},
	{
		I2C_BOARD_INFO("ov5642", 0x3c),
		.platform_data = (void *)&camera_data,
	},
	{
		I2C_BOARD_INFO("mma8451", 0x1c),
		.platform_data = (void *)&mma8451_position,
	},
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
	{
		I2C_BOARD_INFO("ov5640_mipi", 0x3c),
		.platform_data = (void *)&mipi_csi2_data,
	},
	{
		I2C_BOARD_INFO("egalax_ts", 0x4),
		.irq = gpio_to_irq(SABRESD_CAP_TCH_INT0),
	},
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("egalax_ts", 0x4),
		.irq = gpio_to_irq(SABRESD_CAP_TCH_INT1),
	},
	{
		I2C_BOARD_INFO("mag3110", 0x0e),
		.irq = gpio_to_irq(SABRESD_eCOMPASS_INT),
		.platform_data = (void *)&mag3110_position,
	},
	{
		I2C_BOARD_INFO("isl29023", 0x44),
		.irq  = gpio_to_irq(SABRESD_ALS_INT),
		.platform_data = &ls_data,
	},
};

static void imx6q_sabresd_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(SABRESD_USB_OTG_PWR, 1);
	else
		gpio_set_value(SABRESD_USB_OTG_PWR, 0);
}

static void __init imx6q_sabresd_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/* disable external charger detect,
	 * or it will affect signal quality at dp .
	 */
	ret = gpio_request(SABRESD_USB_OTG_PWR, "usb-pwr");
	if (ret) {
		pr_err("failed to get GPIO SABRESD_USB_OTG_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(SABRESD_USB_OTG_PWR, 0);
	mxc_iomux_set_gpr_register(1, 13, 1, 1);

	mx6_set_otghost_vbus_func(imx6q_sabresd_usbotg_vbus);
	mx6_usb_dr_init();
	mx6_usb_h1_init();
}

/* HW Initialization, if return 0, initialization is successful. */
static int mx6q_sabresd_sata_init(struct device *dev, void __iomem *addr)
{
	u32 tmpdata;
	int ret = 0;
	struct clk *clk;

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

	/* Set PHY Paremeters, two steps to configure the GPR13,
	 * one write for rest of parameters, mask of first write is 0x07FFFFFD,
	 * and the other one write for setting the mpll_clk_off_b
	 *.rx_eq_val_0(iomuxc_gpr13[26:24]),
	 *.los_lvl(iomuxc_gpr13[23:19]),
	 *.rx_dpll_mode_0(iomuxc_gpr13[18:16]),
	 *.sata_speed(iomuxc_gpr13[15]),
	 *.mpll_ss_en(iomuxc_gpr13[14]),
	 *.tx_atten_0(iomuxc_gpr13[13:11]),
	 *.tx_boost_0(iomuxc_gpr13[10:7]),
	 *.tx_lvl(iomuxc_gpr13[6:2]),
	 *.mpll_ck_off(iomuxc_gpr13[1]),
	 *.tx_edgerate_0(iomuxc_gpr13[0]),
	 */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x07FFFFFD) | 0x0593A044), IOMUXC_GPR13);

	/* enable SATA_PHY PLL */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x2) | 0x2), IOMUXC_GPR13);

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_sata_clk;
	}
	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;

release_sata_clk:
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);

	return ret;
}

static void mx6q_sabresd_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);
}

static struct ahci_platform_data mx6q_sabresd_sata_data = {
	.init = mx6q_sabresd_sata_init,
	.exit = mx6q_sabresd_sata_exit,
};

static void mx6q_sabresd_flexcan0_switch(int enable)
{
	if (enable) {
		gpio_set_value(SABRESD_CAN1_EN, 1);
		gpio_set_value(SABRESD_CAN1_STBY, 1);
	} else {
		gpio_set_value(SABRESD_CAN1_EN, 0);
		gpio_set_value(SABRESD_CAN1_STBY, 0);
	}
}

static const struct flexcan_platform_data
	mx6q_sabresd_flexcan0_pdata __initconst = {
	.transceiver_switch = mx6q_sabresd_flexcan0_switch,
};

static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static struct ipuv3_fb_platform_data sabresd_fb_data[] = {
	{ /*fb0*/
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	}, {
	.disp_dev = "lcd",
	.interface_pix_fmt = IPU_PIX_FMT_RGB565,
	.mode_str = "CLAA-WVGA",
	.default_bpp = 16,
	.int_clk = false,
	}, {
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-SVGA",
	.default_bpp = 16,
	.int_clk = false,
	}, {
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-VGA",
	.default_bpp = 16,
	.int_clk = false,
	},
};

static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;

	if ((ipu_id > 1) || (ipu_id < 0)) {
		pr_err("Invalid IPU select for HDMI: %d. Set to 0\n", ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0)) {
		pr_err("Invalid DI select for HDMI: %d. Set to 0\n", disp_id);
		disp_id = 0;
	}

	/* Configure the connection between IPU1/2 and HDMI */
	hdmi_mux_setting = 2*ipu_id + disp_id;

	/* GPR3, bits 2-3 = HDMI_MUX_CTL */
	mxc_iomux_set_gpr_register(3, 2, 2, hdmi_mux_setting);
}

static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init = hdmi_init,
};

static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data = {
	.ipu_id = 0,
	.disp_id = 0,
};

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_RGB565,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 1,
	.disp_id = 0,
	.ext_ref = 1,
	.mode = LDB_SEP0,
	.sec_ipu_id = 1,
	.sec_disp_id = 1,
};

static struct max8903_pdata charger1_data = {
	.dok = SABRESD_CHARGE_DOK_B,
	.uok = SABRESD_CHARGE_UOK_B,
	.chg = SABRESD_CHARGE_CHG_1_B,
	.flt = SABRESD_CHARGE_FLT_1_B,
	.dcm_always_high = true,
	.dc_valid = true,
	.usb_valid = true,
};

static struct platform_device sabresd_max8903_charger_1 = {
	.name	= "max8903-charger",
	.id	= 1,
	.dev	= {
		.platform_data = &charger1_data,
	},
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	}, {
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	},
};

static void sabresd_suspend_enter(void)
{
	/* suspend preparation */
	/* Disable AUX 5V */
	gpio_set_value(SABRESD_AUX_5V_EN, 0);
}

static void sabresd_suspend_exit(void)
{
	/* resume restore */
	/* Enable AUX 5V */
	gpio_set_value(SABRESD_AUX_5V_EN, 1);
}
static const struct pm_platform_data mx6q_sabresd_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = sabresd_suspend_enter,
	.suspend_exit = sabresd_suspend_exit,
};

static struct regulator_consumer_supply sabresd_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data sabresd_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(sabresd_vmmc_consumers),
	.consumer_supplies = sabresd_vmmc_consumers,
};

static struct fixed_voltage_config sabresd_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &sabresd_vmmc_init,
};

static struct platform_device sabresd_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &sabresd_vmmc_reg_config,
	},
};

static int imx6q_init_audio(void)
{
	mxc_register_device(&mx6_sabresd_audio_device,
			    &wm8958_data);
	imx6q_add_imx_ssi(1, &mx6_sabresd_ssi_pdata);

	mxc_wm8958_init();
	return 0;
}

static void gps_power_on(bool on)
{
	/* Enable/disable aux_3v15 */
	gpio_request(SABRESD_AUX_3V15_EN, "aux_3v15_en");
	gpio_direction_output(SABRESD_AUX_3V15_EN, 1);
	gpio_set_value(SABRESD_AUX_3V15_EN, on);
	gpio_free(SABRESD_AUX_3V15_EN);
	/*Enable/disable gps_en*/
	gpio_request(SABRESD_GPS_EN, "gps_en");
	gpio_direction_output(SABRESD_GPS_EN, 1);
	gpio_set_value(SABRESD_GPS_EN, on);
	gpio_free(SABRESD_GPS_EN);

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

static struct gpio_keys_button imx6q_buttons[] = {
	GPIO_BUTTON(SABRESD_VOLUME_UP, KEY_VOLUMEUP, 1, "volume-up", 0),
	GPIO_BUTTON(SABRESD_VOLUME_DN, KEY_VOLUMEDOWN, 1, "volume-down", 0),
	GPIO_BUTTON(SABRESD_POWER_OFF, KEY_POWER, 1, "power", 1),
};

static struct gpio_keys_platform_data imx6q_button_data = {
	.buttons	= imx6q_buttons,
	.nbuttons	= ARRAY_SIZE(imx6q_buttons),
};

static struct platform_device imx6q_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &imx6q_button_data,
	}
};

static void __init imx6q_add_device_buttons(void)
{
	platform_device_register(&imx6q_button_device);
}
#else
static void __init imx6q_add_device_buttons(void) {}
#endif

static struct platform_pwm_backlight_data mx6_sabresd_pwm_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static struct mxc_dvfs_platform_data sabresd_dvfscore_data = {
	.reg_id = "cpu_vddgp",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
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
	.delay_time = 80,
};

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
}

static struct mipi_csi2_platform_data mipi_csi2_pdata = {
	.ipu_id	 = 0,
	.csi_id = 0,
	.v_channel = 0,
	.lanes = 2,
	.dphy_clk = "mipi_pllref_clk",
	.pixel_clk = "emi_clk",
};

/*!
 * Board specific initialization.
 */
static void __init mx6_sabresd_board_init(void)
{
	int i;
	int ret;
	struct clk *clko2;
	struct clk *new_parent;
	int rate;

	mxc_iomux_v3_setup_multiple_pads(mx6q_sabresd_pads,
					ARRAY_SIZE(mx6q_sabresd_pads));

#ifdef CONFIG_FEC_1588
	/* Set GPIO_16 input for IEEE-1588 ts_clk and RMII reference clock
	 * For MX6 GPR1 bit21 meaning:
	 * Bit21:       0 - GPIO_16 pad output
	 *              1 - GPIO_16 pad input
	 */
	 mxc_iomux_set_gpr_register(1, 21, 1, 1);
#endif

	gp_reg_id = sabresd_dvfscore_data.reg_id;
	mx6q_sabresd_init_uart();
	imx6q_add_mxc_hdmi_core(&hdmi_core_data);

	imx6q_add_ipuv3(0, &ipu_data[0]);
	imx6q_add_ipuv3(1, &ipu_data[1]);

	for (i = 0; i < ARRAY_SIZE(sabresd_fb_data); i++)
		imx6q_add_ipuv3fb(i, &sabresd_fb_data[i]);

	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
	imx6q_add_v4l2_output(0);
	imx6q_add_v4l2_capture(0);
	imx6q_add_mipi_csi2(&mipi_csi2_pdata);
	imx6q_add_imx_snvs_rtc();

	imx6q_add_imx_i2c(0, &mx6q_sabresd_i2c_data);
	imx6q_add_imx_i2c(1, &mx6q_sabresd_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_sabresd_i2c_data);
	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));
	ret = gpio_request(SABRESD_PFUZE_INT, "pFUZE-int");
	if (ret) {
		printk(KERN_ERR"request pFUZE-int error!!\n");
		return;
	} else {
		gpio_direction_input(SABRESD_PFUZE_INT);
		mx6q_sabresd_init_pfuze100(SABRESD_PFUZE_INT);
	}

	/* SPI */
	imx6q_add_ecspi(0, &mx6q_sabresd_spi_data);
	spi_device_init();

	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &mx6q_sabresd_anatop_thermal_data);
	imx6_init_fec(fec_data);
	imx6q_add_pm_imx(0, &mx6q_sabresd_pm_data);
	imx6q_add_sdhci_usdhc_imx(1, &mx6q_sabresd_sd2_data);
	imx6q_add_sdhci_usdhc_imx(2, &mx6q_sabresd_sd3_data);
	imx6q_add_sdhci_usdhc_imx(3, &mx6q_sabresd_sd4_data);
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	imx6q_sabresd_init_usb();
	imx6q_add_ahci(0, &mx6q_sabresd_sata_data);
	imx6q_add_vpu();
	imx6q_init_audio();
	platform_device_register(&sabresd_vmmc_reg_devices);
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	/* release USB Hub reset */
	gpio_set_value(SABRESD_USB_HUB_RESET, 1);

	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm(1);
	imx6q_add_mxc_pwm(2);
	imx6q_add_mxc_pwm(3);
	imx6q_add_mxc_pwm_backlight(0, &mx6_sabresd_pwm_backlight_data);

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

	imx6q_add_dvfs_core(&sabresd_dvfscore_data);
	mx6_cpu_regulator_init();

	imx6q_add_device_buttons();

	/* enable sensor 3v3 and 1v8 */
	gpio_request(SABRESD_SENSOR_EN, "sensor-en");
	gpio_direction_output(SABRESD_SENSOR_EN, 1);

	/* enable ecompass intr */
	gpio_request(SABRESD_eCOMPASS_INT, "ecompass-int");
	gpio_direction_input(SABRESD_eCOMPASS_INT);
	/* enable light sensor intr */
	gpio_request(SABRESD_ALS_INT, "als-int");
	gpio_direction_input(SABRESD_ALS_INT);

	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();

	/*
	ret = gpio_request_array(mx6q_sabresd_flexcan_gpios,
			ARRAY_SIZE(mx6q_sabresd_flexcan_gpios));
	if (ret)
		pr_err("failed to request flexcan1-gpios: %d\n", ret);
	else
		imx6q_add_flexcan0(&mx6q_sabresd_flexcan0_pdata);
	*/

	clko2 = clk_get(NULL, "clko2_clk");
	if (IS_ERR(clko2))
		pr_err("can't get CLKO2 clock.\n");

	new_parent = clk_get(NULL, "osc_clk");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko2, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko2, 24000000);
	clk_set_rate(clko2, rate);
	clk_enable(clko2);

	/* Enable Aux_5V */
	gpio_request(SABRESD_AUX_5V_EN, "aux_5v_en");
	gpio_direction_output(SABRESD_AUX_5V_EN, 1);
	gpio_set_value(SABRESD_AUX_5V_EN, 1);

	gps_power_on(true);
	/* Register charger chips */
	platform_device_register(&sabresd_max8903_charger_1);
}

extern void __iomem *twd_base;
static void __init mx6_sabresd_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART1_BASE_ADDR, uart_clk);
}

static struct sys_timer mx6_sabresd_timer = {
	.init   = mx6_sabresd_timer_init,
};

static void __init mx6q_sabresd_reserve()
{
	phys_addr_t phys;

	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, SZ_1G);
		memblock_free(phys, imx6q_gpu_pdata.reserved_mem_size);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
}

/*
 * initialize __mach_desc_MX6Q_SABRESD data structure.
 */
MACHINE_START(MX6Q_SABRESD, "Freescale i.MX 6Quad/DualLite/Solo Sabre-SD Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = mx6_sabresd_board_init,
	.timer = &mx6_sabresd_timer,
	.reserve = mx6q_sabresd_reserve,
MACHINE_END
