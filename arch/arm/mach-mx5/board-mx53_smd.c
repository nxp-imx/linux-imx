/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/i2c/mpr.h>
#include <linux/fsl_devices.h>
#include <linux/ahci_platform.h>
#include <linux/regulator/consumer.h>

#include <linux/pwm_backlight.h>
#include <linux/mxcfb.h>
#include <linux/ipu.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/ipu-v3.h>
#include <mach/imx-uart.h>
#include <mach/iomux-mx53.h>
#include <mach/ahci_sata.h>
#include <mach/imx_rfkill.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "crm_regs.h"
#include "devices-imx53.h"
#include "devices.h"
#include "usb.h"


#define SMD_FEC_PHY_RST		IMX_GPIO_NR(7, 6)
#define MX53_SMD_SD1_CD         IMX_GPIO_NR(3, 13)
#define MX53_SMD_SD1_WP         IMX_GPIO_NR(4, 11)
#define MX53_SMD_HDMI_RESET_B   IMX_GPIO_NR(5, 0)
#define MX53_SMD_MODEM_RESET_B  IMX_GPIO_NR(5, 2)
#define MX53_SMD_KEY_INT	IMX_GPIO_NR(5, 4)
#define MX53_SMD_HDMI_INT	IMX_GPIO_NR(6, 12)
#define MX53_SMD_CAP_TCH_INT1	IMX_GPIO_NR(3, 20)
#define MX53_SMD_SATA_PWR_EN	IMX_GPIO_NR(3, 3)
#define MX53_SMD_OTG_VBUS	IMX_GPIO_NR(7, 8)
#define MX53_SMD_NONKEY		IMX_GPIO_NR(1, 8)
#define MX53_SMD_UI1		IMX_GPIO_NR(2, 14)
#define MX53_SMD_UI2		IMX_GPIO_NR(2, 15)
#define MX53_SMD_HEADPHONE_DEC	IMX_GPIO_NR(2, 5)
#define MX53_SMD_OSC_CKIH1_EN	IMX_GPIO_NR(6, 11)
#define MX53_SMD_DCDC1V8_EN	IMX_GPIO_NR(3, 1)
#define MX53_SMD_DCDC5V_BB_EN	IMX_GPIO_NR(4, 14)
#define MX53_SMD_ALS_INT 	IMX_GPIO_NR(3, 22)
#define MX53_SMD_BT_RESET	IMX_GPIO_NR(3, 28)
#define MX53_SMD_CSI0_RST       IMX_GPIO_NR(6, 9)
#define MX53_SMD_CSI0_PWN       IMX_GPIO_NR(6, 10)
#define MX53_SMD_ECSPI1_CS0	IMX_GPIO_NR(2, 30)
#define MX53_SMD_ECSPI1_CS1	IMX_GPIO_NR(3, 19)

static struct clk *sata_clk, *sata_ref_clk;

extern char *lp_reg_id;
extern char *gp_reg_id;
extern void mx5_cpu_regulator_init(void);
extern int mx53_smd_init_da9052(void);
extern void mx5_cpu_regulator_init(void);

static iomux_v3_cfg_t mx53_smd_pads[] = {
	MX53_PAD_CSI0_DAT10__UART1_TXD_MUX,
	MX53_PAD_CSI0_DAT11__UART1_RXD_MUX,

	MX53_PAD_PATA_BUFFER_EN__UART2_RXD_MUX,
	MX53_PAD_PATA_DMARQ__UART2_TXD_MUX,

	MX53_PAD_PATA_CS_0__UART3_TXD_MUX,
	MX53_PAD_PATA_CS_1__UART3_RXD_MUX,
	MX53_PAD_PATA_DA_1__UART3_CTS,
	MX53_PAD_PATA_DA_2__UART3_RTS,
	/* I2C1 */
	MX53_PAD_CSI0_DAT8__I2C1_SDA,
	MX53_PAD_CSI0_DAT9__I2C1_SCL,
	/* I2C2 */
	MX53_PAD_KEY_COL3__I2C2_SCL,
	MX53_PAD_KEY_ROW3__I2C2_SDA,
	/* I2C3 */
	MX53_PAD_GPIO_3__I2C3_SCL,
	MX53_PAD_GPIO_6__I2C3_SDA,

	/* CSPI1 */
	MX53_PAD_EIM_EB2__ECSPI1_SS0,
	MX53_PAD_EIM_D16__ECSPI1_SCLK,
	MX53_PAD_EIM_D17__ECSPI1_MISO,
	MX53_PAD_EIM_D18__ECSPI1_MOSI,
	MX53_PAD_EIM_D19__ECSPI1_SS1,
	MX53_PAD_EIM_EB2__GPIO2_30,
	MX53_PAD_EIM_D19__GPIO3_19,

	/* SD1 */
	MX53_PAD_SD1_CMD__ESDHC1_CMD,
	MX53_PAD_SD1_CLK__ESDHC1_CLK,
	MX53_PAD_SD1_DATA0__ESDHC1_DAT0,
	MX53_PAD_SD1_DATA1__ESDHC1_DAT1,
	MX53_PAD_SD1_DATA2__ESDHC1_DAT2,
	MX53_PAD_SD1_DATA3__ESDHC1_DAT3,
	/* SD1_CD */
	MX53_PAD_EIM_DA13__GPIO3_13,
	/* SD1_WP */
	MX53_PAD_KEY_ROW2__GPIO4_11,

	/* SD2 */
	MX53_PAD_SD2_CMD__ESDHC2_CMD,
	MX53_PAD_SD2_CLK__ESDHC2_CLK,
	MX53_PAD_SD2_DATA0__ESDHC2_DAT0,
	MX53_PAD_SD2_DATA1__ESDHC2_DAT1,
	MX53_PAD_SD2_DATA2__ESDHC2_DAT2,
	MX53_PAD_SD2_DATA3__ESDHC2_DAT3,

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

	/* SATA_PWR_EN */
	MX53_PAD_EIM_DA3__GPIO3_3,

	/* USB_OTG_OC */
	MX53_PAD_EIM_DA12__GPIO3_12,
	/* USB_HUB_RESET_B */
	MX53_PAD_EIM_DA14__GPIO3_14,
	/* USB_OTG_PWR_EN */
	MX53_PAD_PATA_DA_2__GPIO7_8,

	/* OSC_CKIH1_EN, for audio codec clk */
	MX53_PAD_NANDF_CS0__GPIO6_11,

	/* AUDMUX3 */
	MX53_PAD_CSI0_DAT4__AUDMUX_AUD3_TXC,
	MX53_PAD_CSI0_DAT5__AUDMUX_AUD3_TXD,
	MX53_PAD_CSI0_DAT6__AUDMUX_AUD3_TXFS,
	MX53_PAD_CSI0_DAT7__AUDMUX_AUD3_RXD,

	/* AUDMUX5 */
	MX53_PAD_KEY_COL0__AUDMUX_AUD5_TXC,
	MX53_PAD_KEY_ROW0__AUDMUX_AUD5_TXD,
	MX53_PAD_KEY_COL1__AUDMUX_AUD5_TXFS,
	MX53_PAD_KEY_ROW1__AUDMUX_AUD5_RXD,

	/* AUD_AMP_STBY_B */
	MX53_PAD_EIM_DA2__GPIO3_2,

	/* DCDC1V8_EN */
	MX53_PAD_EIM_DA1__GPIO3_1,
	/* DCDC5V_BB_EN */
	MX53_PAD_KEY_COL4__GPIO4_14,
	/*SSI_EXT1_CLK*/
	MX53_PAD_GPIO_0__CCM_SSI_EXT1_CLK,
	/* PWM */
	MX53_PAD_GPIO_1__PWM2_PWMO,
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
	/* LDVS */
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
#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake)    \
{                                                               \
	.gpio           = gpio_num,                             \
	.type           = EV_KEY,                               \
	.code           = ev_code,                              \
	.active_low     = act_low,                              \
	.desc           = "btn " descr,                         \
	.wakeup         = wake,                                 \
}

static struct gpio_keys_button smd_buttons[] = {
	GPIO_BUTTON(MX53_SMD_NONKEY, KEY_POWER, 1, "power", 0),
	GPIO_BUTTON(MX53_SMD_UI1, KEY_VOLUMEUP, 1, "volume-up", 0),
	GPIO_BUTTON(MX53_SMD_UI2, KEY_VOLUMEDOWN, 1, "volume-down", 0),
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

static void smd_suspend_enter(void)
{
	/* da9053 suspend preparation */
}

static void smd_suspend_exit(void)
{
	/*clear the EMPGC0/1 bits */
	__raw_writel(0, MXC_SRPG_EMPGC0_SRPGCR);
	__raw_writel(0, MXC_SRPG_EMPGC1_SRPGCR);
	/* da9053 resmue resore */
}

static struct mxc_pm_platform_data smd_pm_data = {
	.suspend_enter = smd_suspend_enter,
	.suspend_exit = smd_suspend_exit,
};


static const struct esdhc_platform_data mx53_smd_sd1_data __initconst = {
	.cd_gpio = MX53_SMD_SD1_CD,
	.wp_gpio = MX53_SMD_SD1_WP,
};

static const struct esdhc_platform_data mx53_smd_sd2_data __initconst = {
	.always_present = 1,
};

static const struct esdhc_platform_data mx53_smd_sd3_data __initconst = {
	.always_present = 1,
};

static struct fsl_mxc_camera_platform_data camera_data = {
	.analog_regulator = "DA9052_LDO7",
	.core_regulator = "DA9052_LDO9",
	.mclk = 24000000,
	.csi = 0,
};

static struct fsl_mxc_lightsensor_platform_data ls_data = {
	.rext = 700,    /* calibration: 499K->700K */
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
	.type = "sgtl5000",
	.addr = 0x0a,
	},
	{
	.type = "mpr121_touchkey",
	.addr = 0x5a,
	.irq = gpio_to_irq(MX53_SMD_KEY_INT),
	.platform_data = &mpr121_keyboard_platdata,
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

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
	.type = "sii902x",
	.addr = 0x39,
	.irq = gpio_to_irq(MX53_SMD_HDMI_INT),
	.platform_data = &sii902x_hdmi_data,
	},
	{
		I2C_BOARD_INFO("p1003_ts", 0x41),
		.irq = gpio_to_irq(MX53_SMD_CAP_TCH_INT1),
	},
	{
	.type = "isl29023",
	.addr = 0x44,
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
	struct regulator *wifi_bt_pwren;

	wifi_bt_pwren = regulator_get(NULL, "wifi_bt");
	if (IS_ERR(wifi_bt_pwren)) {
		printk(KERN_ERR "%s: regulator_get error\n", __func__);
		return -1;
	}

	if (status) {
		regulator_enable(wifi_bt_pwren);
		mx53_smd_bt_reset();
	} else
		regulator_disable(wifi_bt_pwren);

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

static struct mxc_audio_platform_data smd_audio_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 5,
	.init = smd_sgtl5000_init,
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

static struct ipuv3_fb_platform_data smd_fb_data[] = {
	{
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	}, {
	.disp_dev = "hdmi",
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	.mode_str = "1024x768M-16@60",
	.default_bpp = 16,
	.int_clk = false,
	},
};

static struct imx_ipuv3_platform_data ipu_data = {
	.rev = 3,
	.csi_clk[0] = "ssi_ext1_clk",
};

static struct platform_pwm_backlight_data mxc_pwm_backlight_data = {
	.pwm_id = 1,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
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

static struct mxc_regulator_platform_data smd_regulator_data = {
	.cpu_reg_id = "DA9052_BUCK_CORE",
};

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
}

static void __init mx53_smd_board_init(void)
{
	int i;

	mxc_iomux_v3_setup_multiple_pads(mx53_smd_pads,
					ARRAY_SIZE(mx53_smd_pads));

	gp_reg_id = smd_regulator_data.cpu_reg_id;
	lp_reg_id = smd_regulator_data.vcc_reg_id;

	mxc_spdif_data.spdif_core_clk = clk_get(NULL, "spdif_xtal_clk");
	clk_put(mxc_spdif_data.spdif_core_clk);

	mx53_smd_init_uart();
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
	imx53_add_vpu();
	imx53_add_ldb(&ldb_data);
	imx53_add_v4l2_output(0);
	imx53_add_v4l2_capture(0);
	imx53_add_mxc_pwm(1);
	imx53_add_mxc_pwm_backlight(0, &mxc_pwm_backlight_data);
	imx53_add_sdhci_esdhc_imx(0, &mx53_smd_sd1_data);
	imx53_add_sdhci_esdhc_imx(1, &mx53_smd_sd2_data);
	imx53_add_sdhci_esdhc_imx(2, &mx53_smd_sd3_data);
	imx53_add_ahci(0, &mx53_smd_sata_data);
	mxc_register_device(&imx_ahci_device_hwmon, NULL);

	mx53_smd_init_usb();
	imx53_add_iim(&iim_data);
	smd_add_device_buttons();

	mx53_smd_init_da9052();

	/* Camera reset */
	gpio_request(MX53_SMD_CSI0_RST, "cam-reset");
	gpio_set_value(MX53_SMD_CSI0_RST, 1);

	/* Camera power down */
	gpio_request(MX53_SMD_CSI0_PWN, "cam-pwdn");
	gpio_direction_output(MX53_SMD_CSI0_PWN, 1);
	msleep(1);
	gpio_set_value(MX53_SMD_CSI0_PWN, 0);

	spi_device_init();

	i2c_register_board_info(0, mxc_i2c0_board_info,
				ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
				ARRAY_SIZE(mxc_i2c2_board_info));


	gpio_request(MX53_SMD_DCDC1V8_EN, "dcdc1v8-en");
	gpio_direction_output(MX53_SMD_DCDC1V8_EN, 1);

	/* ambient light sensor */
	gpio_request(MX53_SMD_ALS_INT, "als int");
	gpio_direction_input(MX53_SMD_ALS_INT);

	mxc_register_device(&smd_audio_device, &smd_audio_data);
	mxc_register_device(&imx_bt_rfkill, &imx_bt_rfkill_data);
	imx53_add_imx_ssi(1, &smd_ssi_pdata);

	imx53_add_spdif(&mxc_spdif_data);
	imx53_add_spdif_dai();
	imx53_add_spdif_audio_device();

	/* this call required to release SCC RAM partition held by ROM
	  * during boot, even if SCC2 driver is not part of the image
	  */
	imx53_add_mxc_scc2();

	mx5_cpu_regulator_init();
}

static void __init mx53_smd_timer_init(void)
{
	mx53_clocks_init(32768, 24000000, 22579200, 0);
}

static struct sys_timer mx53_smd_timer = {
	.init	= mx53_smd_timer_init,
};

MACHINE_START(MX53_SMD, "Freescale MX53 SMD Board")
	.fixup = fixup_mxc_board,
	.map_io = mx53_map_io,
	.init_early = imx53_init_early,
	.init_irq = mx53_init_irq,
	.timer = &mx53_smd_timer,
	.init_machine = mx53_smd_board_init,
MACHINE_END
