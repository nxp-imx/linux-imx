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
#include <linux/smsc911x.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/etherdevice.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/max17135.h>
#include <sound/wm8962.h>
#include <sound/pcm.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6sl.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-mx6sl_arm2.h"

#define MX6_ARM2_USBOTG1_PWR    IMX_GPIO_NR(4, 0)       /* KEY_COL4 */
#define MX6_ARM2_USBOTG2_PWR    IMX_GPIO_NR(4, 2)       /* KEY_COL5 */
#define MX6_ARM2_LCD_PWR_EN	IMX_GPIO_NR(4, 3)	/* KEY_ROW5 */
#define MX6_ARM2_SD1_WP		IMX_GPIO_NR(4, 6)	/* KEY_COL7 */
#define MX6_ARM2_SD1_CD		IMX_GPIO_NR(4, 7)	/* KEY_ROW7 */
#define MX6_ARM2_SD2_WP		IMX_GPIO_NR(4, 29)	/* SD2_DAT6 */
#define MX6_ARM2_SD2_CD		IMX_GPIO_NR(5, 0)	/* SD2_DAT7 */
#define MX6_ARM2_SD3_CD		IMX_GPIO_NR(3, 22)	/* REF_CLK_32K */
#define MX6_ARM2_FEC_PWR_EN	IMX_GPIO_NR(4, 21)	/* FEC_TX_CLK */

extern int __init mx6sl_arm2_init_pfuze100(u32 int_gpio);
static const struct esdhc_platform_data mx6_arm2_sd1_data __initconst = {
	.cd_gpio		= MX6_ARM2_SD1_CD,
	.wp_gpio		= MX6_ARM2_SD1_WP,
	.support_8bit		= 1,
	.keep_power_at_suspend	= 1,
	.delay_line		= 0,
};

static const struct esdhc_platform_data mx6_arm2_sd2_data __initconst = {
	.cd_gpio		= MX6_ARM2_SD2_CD,
	.wp_gpio		= MX6_ARM2_SD2_WP,
	.keep_power_at_suspend	= 1,
	.delay_line		= 0,
};

static const struct esdhc_platform_data mx6_arm2_sd3_data __initconst = {
	.cd_gpio		= MX6_ARM2_SD3_CD,
	.keep_power_at_suspend	= 1,
	.delay_line		= 0,
};

static struct imxi2c_platform_data mx6_arm2_i2c0_data = {
	.bitrate = 100000,
};

static struct imxi2c_platform_data mx6_arm2_i2c1_data = {
	.bitrate = 100000,
};

static struct imxi2c_platform_data mx6_arm2_i2c2_data = {
	.bitrate = 400000,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("max17135", 0x48),
		/*.platform_data = &max17135_pdata,*/
	},
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("wm8962", 0x1a),
		/*.platform_data = &wm8962_config_data,*/
	},
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
	},
};
void __init early_console_setup(unsigned long base, struct clk *clk);

static inline void mx6_arm2_init_uart(void)
{
	imx6q_add_imx_uart(0, NULL); /* DEBUG UART1 */

	imx6q_add_sdhci_usdhc_imx(0, &mx6_arm2_sd1_data);
	imx6q_add_sdhci_usdhc_imx(1, &mx6_arm2_sd2_data);
	imx6q_add_sdhci_usdhc_imx(2, &mx6_arm2_sd3_data);
}

static struct fec_platform_data fec_data __initdata = {
	.phy = PHY_INTERFACE_MODE_RMII,
};

static void imx6_arm2_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(MX6_ARM2_USBOTG1_PWR, 1);
	else
		gpio_set_value(MX6_ARM2_USBOTG1_PWR, 0);
}

static void __init mx6_arm2_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);

	/* disable external charger detect,
	 * or it will affect signal quality at dp.
	 */

	ret = gpio_request(MX6_ARM2_USBOTG1_PWR, "usbotg-pwr");
	if (ret) {
		pr_err("failed to get GPIO MX6_ARM2_USBOTG1_PWR:%d\n", ret);
		return;
	}
	gpio_direction_output(MX6_ARM2_USBOTG1_PWR, 0);

	ret = gpio_request(MX6_ARM2_USBOTG2_PWR, "usbh1-pwr");
	if (ret) {
		pr_err("failed to get GPIO MX6_ARM2_USBOTG2_PWR:%d\n", ret);
		return;
	}
	gpio_direction_output(MX6_ARM2_USBOTG2_PWR, 1);

	mx6_set_otghost_vbus_func(imx6_arm2_usbotg_vbus);
	mx6_usb_dr_init();
	mx6_usb_h1_init();
#ifdef CONFIG_USB_EHCI_ARC_HSIC
	mx6_usb_h2_init();
#endif
}

static struct platform_pwm_backlight_data mx6_arm2_pwm_backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= 255,
	.dft_brightness	= 128,
	.pwm_period_ns	= 50000,
};
static struct fb_videomode video_modes[] = {
	{
	 /* 800x480 @ 57 Hz , pixel clk @ 32MHz */
	 "SEIKO-WVGA", 60, 800, 480, 29850, 99, 164, 33, 10, 10, 10,
	 FB_SYNC_CLK_LAT_FALL,
	 FB_VMODE_NONINTERLACED,
	 0,},
};

static struct mxc_fb_platform_data fb_data[] = {
	{
	 .interface_pix_fmt = V4L2_PIX_FMT_RGB24,
	 .mode_str = "SEIKO-WVGA",
	 .mode = video_modes,
	 .num_modes = ARRAY_SIZE(video_modes),
	 },
};

static struct platform_device lcd_wvga_device = {
	.name = "lcd_seiko",
};
/*!
 * Board specific initialization.
 */
static void __init mx6_arm2_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx6sl_arm2_pads, ARRAY_SIZE(mx6sl_arm2_pads));

	gp_reg_id = "cpu_vddgp";
	mx6_cpu_regulator_init();

	imx6q_add_imx_i2c(0, &mx6_arm2_i2c0_data);
	imx6q_add_imx_i2c(1, &mx6_arm2_i2c1_data);
	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	imx6q_add_imx_i2c(2, &mx6_arm2_i2c2_data);
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));
	mx6sl_arm2_init_pfuze100(0);

	mx6_arm2_init_uart();
	/* get enet tx reference clk from FEC_REF_CLK pad.
	 * GPR1[14] = 0, GPR1[18:17] = 00
	 */
	mxc_iomux_set_gpr_register(1, 14, 1, 0);
	mxc_iomux_set_gpr_register(1, 17, 2, 0);

	/* power on FEC phy and reset phy */
	gpio_request(MX6_ARM2_FEC_PWR_EN, "fec-pwr");
	gpio_direction_output(MX6_ARM2_FEC_PWR_EN, 1);
	/* wait RC ms for hw reset */
	udelay(500);

	imx6_init_fec(fec_data);

	mx6_arm2_init_usb();

	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm_backlight(0, &mx6_arm2_pwm_backlight_data);
	imx6dl_add_imx_elcdif(&fb_data[0]);

	gpio_request(MX6_ARM2_LCD_PWR_EN, "elcdif-power-on");
	gpio_direction_output(MX6_ARM2_LCD_PWR_EN, 1);
	mxc_register_device(&lcd_wvga_device, NULL);
}

extern void __iomem *twd_base;
static void __init mx6_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6sl_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART1_BASE_ADDR, uart_clk);
}

static struct sys_timer mxc_timer = {
	.init   = mx6_timer_init,
};

static void __init mx6_arm2_reserve(void)
{

}

MACHINE_START(MX6SL_ARM2, "Freescale i.MX 6SoloLite Armadillo2 Board")
	.boot_params	= MX6SL_PHYS_OFFSET + 0x100,
	.map_io		= mx6_map_io,
	.init_irq	= mx6_init_irq,
	.init_machine	= mx6_arm2_init,
	.timer		= &mxc_timer,
	.reserve	= mx6_arm2_reserve,
MACHINE_END
