/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/ipu.h>
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
#include <sound/pcm.h>
#include <linux/mxc_asrc.h>
#include <linux/mfd/mxc-hdmi-core.h>


#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>
#include <mach/mipi_dsi.h>
#include <mach/mipi_csi2.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-mx6q_arm2.h"
#include "board-mx6dl_arm2.h"

/* GPIO PIN, sort by PORT/BIT */
#define MX6_ARM2_LDB_BACKLIGHT		IMX_GPIO_NR(1, 9)
#define MX6_ARM2_ECSPI1_CS0		IMX_GPIO_NR(2, 30)
#define MX6_ARM2_ECSPI1_CS1		IMX_GPIO_NR(3, 19)
#define MX6_ARM2_USB_OTG_PWR		IMX_GPIO_NR(3, 22)
#define MX6_ARM2_DISP0_PWR		IMX_GPIO_NR(3, 24)
#define MX6_ARM2_DISP0_I2C_EN		IMX_GPIO_NR(3, 28)
#define MX6_ARM2_CAP_TCH_INT		IMX_GPIO_NR(3, 31)
#define MX6_ARM2_DISP0_DET_INT		IMX_GPIO_NR(3, 31)
#define MX6_ARM2_CSI0_RST		IMX_GPIO_NR(4, 5)
#define MX6_ARM2_DISP0_RESET		IMX_GPIO_NR(5, 0)
#define MX6_ARM2_CSI0_PWN		IMX_GPIO_NR(5, 23)
#define MX6_ARM2_CAN2_EN		IMX_GPIO_NR(5, 24)
#define MX6_ARM2_CSI0_RST_TVIN		IMX_GPIO_NR(5, 25)
#define MX6_ARM2_SD3_CD			IMX_GPIO_NR(6, 11)
#define MX6_ARM2_SD3_WP			IMX_GPIO_NR(6, 14)
#define MX6_ARM2_CAN1_STBY		IMX_GPIO_NR(7, 12)
#define MX6_ARM2_CAN1_EN		IMX_GPIO_NR(7, 13)
#define MX6_ARM2_MAX7310_1_BASE_ADDR	IMX_GPIO_NR(8, 0)
#define MX6_ARM2_MAX7310_2_BASE_ADDR	IMX_GPIO_NR(8, 8)
#define MX6DL_ARM2_EPDC_SDDO_0		IMX_GPIO_NR(2, 22)
#define MX6DL_ARM2_EPDC_SDDO_1		IMX_GPIO_NR(3, 10)
#define MX6DL_ARM2_EPDC_SDDO_2		IMX_GPIO_NR(3, 12)
#define MX6DL_ARM2_EPDC_SDDO_3		IMX_GPIO_NR(3, 11)
#define MX6DL_ARM2_EPDC_SDDO_4		IMX_GPIO_NR(2, 27)
#define MX6DL_ARM2_EPDC_SDDO_5		IMX_GPIO_NR(2, 30)
#define MX6DL_ARM2_EPDC_SDDO_6		IMX_GPIO_NR(2, 23)
#define MX6DL_ARM2_EPDC_SDDO_7		IMX_GPIO_NR(2, 26)
#define MX6DL_ARM2_EPDC_SDDO_8		IMX_GPIO_NR(2, 24)
#define MX6DL_ARM2_EPDC_SDDO_9		IMX_GPIO_NR(3, 15)
#define MX6DL_ARM2_EPDC_SDDO_10		IMX_GPIO_NR(3, 16)
#define MX6DL_ARM2_EPDC_SDDO_11		IMX_GPIO_NR(3, 23)
#define MX6DL_ARM2_EPDC_SDDO_12		IMX_GPIO_NR(3, 19)
#define MX6DL_ARM2_EPDC_SDDO_13		IMX_GPIO_NR(3, 13)
#define MX6DL_ARM2_EPDC_SDDO_14		IMX_GPIO_NR(3, 14)
#define MX6DL_ARM2_EPDC_SDDO_15		IMX_GPIO_NR(5, 2)
#define MX6DL_ARM2_EPDC_GDCLK		IMX_GPIO_NR(2, 17)
#define MX6DL_ARM2_EPDC_GDSP		IMX_GPIO_NR(2, 16)
#define MX6DL_ARM2_EPDC_GDOE		IMX_GPIO_NR(6, 6)
#define MX6DL_ARM2_EPDC_GDRL		IMX_GPIO_NR(5, 4)
#define MX6DL_ARM2_EPDC_SDCLK		IMX_GPIO_NR(3, 31)
#define MX6DL_ARM2_EPDC_SDOEZ		IMX_GPIO_NR(3, 30)
#define MX6DL_ARM2_EPDC_SDOED		IMX_GPIO_NR(3, 26)
#define MX6DL_ARM2_EPDC_SDOE		IMX_GPIO_NR(3, 27)
#define MX6DL_ARM2_EPDC_SDLE		IMX_GPIO_NR(3, 1)
#define MX6DL_ARM2_EPDC_SDCLKN		IMX_GPIO_NR(3, 0)
#define MX6DL_ARM2_EPDC_SDSHR		IMX_GPIO_NR(2, 29)
#define MX6DL_ARM2_EPDC_PWRCOM		IMX_GPIO_NR(2, 28)
#define MX6DL_ARM2_EPDC_PWRSTAT		IMX_GPIO_NR(2, 21)
#define MX6DL_ARM2_EPDC_PWRCTRL0	IMX_GPIO_NR(2, 20)
#define MX6DL_ARM2_EPDC_PWRCTRL1	IMX_GPIO_NR(2, 19)
#define MX6DL_ARM2_EPDC_PWRCTRL2	IMX_GPIO_NR(2, 18)
#define MX6DL_ARM2_EPDC_PWRCTRL3	IMX_GPIO_NR(3, 28)
#define MX6DL_ARM2_EPDC_BDR0		IMX_GPIO_NR(3, 2)
#define MX6DL_ARM2_EPDC_BDR1		IMX_GPIO_NR(3, 3)
#define MX6DL_ARM2_EPDC_SDCE0		IMX_GPIO_NR(3, 4)
#define MX6DL_ARM2_EPDC_SDCE1		IMX_GPIO_NR(3, 5)
#define MX6DL_ARM2_EPDC_SDCE2		IMX_GPIO_NR(3, 6)
#define MX6DL_ARM2_EPDC_SDCE3		IMX_GPIO_NR(3, 7)
#define MX6DL_ARM2_EPDC_SDCE4		IMX_GPIO_NR(3, 8)
#define MX6DL_ARM2_EPDC_SDCE5		IMX_GPIO_NR(3, 9)
#define MX6DL_ARM2_EPDC_PMIC_WAKE	IMX_GPIO_NR(2, 31)
#define MX6DL_ARM2_EPDC_PMIC_INT	IMX_GPIO_NR(2, 25)
#define MX6DL_ARM2_EPDC_VCOM		IMX_GPIO_NR(3, 17)

#define MX6_ARM2_IO_EXP_GPIO1(x)	(MX6_ARM2_MAX7310_1_BASE_ADDR + (x))
#define MX6_ARM2_IO_EXP_GPIO2(x)	(MX6_ARM2_MAX7310_2_BASE_ADDR + (x))

#define MX6_ARM2_PCIE_PWR_EN		MX6_ARM2_IO_EXP_GPIO1(2)
#define MX6_ARM2_PCIE_RESET		MX6_ARM2_IO_EXP_GPIO2(2)

#define MX6_ARM2_CAN2_STBY		MX6_ARM2_IO_EXP_GPIO2(1)

#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
#define MX6_ENET_IRQ		IMX_GPIO_NR(1, 6)
#define IOMUX_OBSRV_MUX1_OFFSET	0x3c
#define OBSRV_MUX1_MASK			0x3f
#define OBSRV_MUX1_ENET_IRQ		0x9
#endif

#define BMCR_PDOWN			0x0800 /* PHY Powerdown */

void __init early_console_setup(unsigned long base, struct clk *clk);
static struct clk *sata_clk;
static int esai_record;
static int sgtl5000_en;
static int spdif_en;
static int gpmi_en;
static int flexcan_en;
static int disable_mipi_dsi;

extern struct regulator *(*get_cpu_regulator)(void);
extern void (*put_cpu_regulator)(void);
extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;
extern int epdc_enabled;
static int max17135_regulator_init(struct max17135 *max17135);

enum sd_pad_mode {
	SD_PAD_MODE_LOW_SPEED,
	SD_PAD_MODE_MED_SPEED,
	SD_PAD_MODE_HIGH_SPEED,
};

static int plt_sd_pad_change(unsigned int index, int clock)
{
	/* LOW speed is the default state of SD pads */
	static enum sd_pad_mode pad_mode = SD_PAD_MODE_LOW_SPEED;

	iomux_v3_cfg_t *sd_pads_200mhz = NULL;
	iomux_v3_cfg_t *sd_pads_100mhz = NULL;
	iomux_v3_cfg_t *sd_pads_50mhz = NULL;

	u32 sd_pads_200mhz_cnt;
	u32 sd_pads_100mhz_cnt;
	u32 sd_pads_50mhz_cnt;

	switch (index) {
	case 2:
		if (cpu_is_mx6q()) {
			sd_pads_200mhz = mx6q_sd3_200mhz;
			sd_pads_100mhz = mx6q_sd3_100mhz;
			sd_pads_50mhz = mx6q_sd3_50mhz;

			sd_pads_200mhz_cnt = ARRAY_SIZE(mx6q_sd3_200mhz);
			sd_pads_100mhz_cnt = ARRAY_SIZE(mx6q_sd3_100mhz);
			sd_pads_50mhz_cnt = ARRAY_SIZE(mx6q_sd3_50mhz);
		} else if (cpu_is_mx6dl()) {
			sd_pads_200mhz = mx6dl_sd3_200mhz;
			sd_pads_100mhz = mx6dl_sd3_100mhz;
			sd_pads_50mhz = mx6dl_sd3_50mhz;

			sd_pads_200mhz_cnt = ARRAY_SIZE(mx6dl_sd3_200mhz);
			sd_pads_100mhz_cnt = ARRAY_SIZE(mx6dl_sd3_100mhz);
			sd_pads_50mhz_cnt = ARRAY_SIZE(mx6dl_sd3_50mhz);
		}
		break;
	case 3:
		if (cpu_is_mx6q()) {
			sd_pads_200mhz = mx6q_sd4_200mhz;
			sd_pads_100mhz = mx6q_sd4_100mhz;
			sd_pads_50mhz = mx6q_sd4_50mhz;

			sd_pads_200mhz_cnt = ARRAY_SIZE(mx6q_sd4_200mhz);
			sd_pads_100mhz_cnt = ARRAY_SIZE(mx6q_sd4_100mhz);
			sd_pads_50mhz_cnt = ARRAY_SIZE(mx6q_sd4_50mhz);
		} else if (cpu_is_mx6dl()) {
			sd_pads_200mhz = mx6dl_sd4_200mhz;
			sd_pads_100mhz = mx6dl_sd4_100mhz;
			sd_pads_50mhz = mx6dl_sd4_50mhz;

			sd_pads_200mhz_cnt = ARRAY_SIZE(mx6dl_sd4_200mhz);
			sd_pads_100mhz_cnt = ARRAY_SIZE(mx6dl_sd4_100mhz);
			sd_pads_50mhz_cnt = ARRAY_SIZE(mx6dl_sd4_50mhz);
		}
		break;
	default:
		printk(KERN_ERR "no such SD host controller index %d\n", index);
		return -EINVAL;
	}

	if (clock > 100000000) {
		if (pad_mode == SD_PAD_MODE_HIGH_SPEED)
			return 0;
		BUG_ON(!sd_pads_200mhz);
		pad_mode = SD_PAD_MODE_HIGH_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_200mhz,
							sd_pads_200mhz_cnt);
	} else if (clock > 52000000) {
		if (pad_mode == SD_PAD_MODE_MED_SPEED)
			return 0;
		BUG_ON(!sd_pads_100mhz);
		pad_mode = SD_PAD_MODE_MED_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_100mhz,
							sd_pads_100mhz_cnt);
	} else {
		if (pad_mode == SD_PAD_MODE_LOW_SPEED)
			return 0;
		BUG_ON(!sd_pads_50mhz);
		pad_mode = SD_PAD_MODE_LOW_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_50mhz,
							sd_pads_50mhz_cnt);
	}
}

static const struct esdhc_platform_data mx6_arm2_sd3_data __initconst = {
	.cd_gpio		= MX6_ARM2_SD3_CD,
	.wp_gpio		= MX6_ARM2_SD3_WP,
	.support_18v		= 1,
	.support_8bit		= 1,
	.keep_power_at_suspend	= 1,
	.delay_line		= 0,
	.platform_pad_change	= plt_sd_pad_change,
};

/* No card detect signal for SD4 on ARM2 board*/
static const struct esdhc_platform_data mx6_arm2_sd4_data __initconst = {
	.always_present		= 1,
	.support_8bit		= 1,
	.keep_power_at_suspend	= 1,
	.platform_pad_change	= plt_sd_pad_change,
};

static int __init gpmi_nand_platform_init(void)
{
	iomux_v3_cfg_t *nand_pads = NULL;
	u32 nand_pads_cnt;

	if (cpu_is_mx6q()) {
		nand_pads = mx6q_gpmi_nand;
		nand_pads_cnt = ARRAY_SIZE(mx6dl_gpmi_nand);
	} else if (cpu_is_mx6dl()) {
		nand_pads = mx6dl_gpmi_nand;
		nand_pads_cnt = ARRAY_SIZE(mx6dl_gpmi_nand);

	}
	BUG_ON(!nand_pads);
	return mxc_iomux_v3_setup_multiple_pads(nand_pads, nand_pads_cnt);
}

static struct gpmi_nand_platform_data
mx6_gpmi_nand_platform_data __initdata = {
	.platform_init           = gpmi_nand_platform_init,
	.min_prop_delay_in_ns    = 5,
	.max_prop_delay_in_ns    = 9,
	.max_chip_count          = 1,
	.enable_bbt              = 1,
	.enable_ddr              = 0,
};

static int __init board_support_onfi_nand(char *p)
{
	mx6_gpmi_nand_platform_data.enable_ddr = 1;
	return 0;
}

early_param("onfi_support", board_support_onfi_nand);

static const struct anatop_thermal_platform_data
	mx6_arm2_anatop_thermal_data __initconst = {
	.name = "anatop_thermal",
};

static const struct imxuart_platform_data mx6_arm2_uart1_data __initconst = {
	.flags      = IMXUART_HAVE_RTSCTS | IMXUART_USE_DCEDTE | IMXUART_SDMA,
	.dma_req_rx = MX6Q_DMA_REQ_UART2_RX,
	.dma_req_tx = MX6Q_DMA_REQ_UART2_TX,
};

static inline void mx6_arm2_init_uart(void)
{
	imx6q_add_imx_uart(3, NULL);
	imx6q_add_imx_uart(1, &mx6_arm2_uart1_data);
}

static int mx6_arm2_fec_phy_init(struct phy_device *phydev)
{
	unsigned short val;

	/* Ar8031 phy SmartEEE feature cause link status generates glitch,
	 * which cause ethernet link down/up issue, so disable SmartEEE
	 */
	phy_write(phydev, 0xd, 0x3);
	phy_write(phydev, 0xe, 0x805d);
	phy_write(phydev, 0xd, 0x4003);
	val = phy_read(phydev, 0xe);
	val &= ~(0x1 << 8);
	phy_write(phydev, 0xe, val);

	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	phy_write(phydev, 0xd, 0x7);
	phy_write(phydev, 0xe, 0x8016);
	phy_write(phydev, 0xd, 0x4007);
	val = phy_read(phydev, 0xe);

	val &= 0xffe3;
	val |= 0x18;
	phy_write(phydev, 0xe, val);

	/* introduce tx clock delay */
	phy_write(phydev, 0x1d, 0x5);
	val = phy_read(phydev, 0x1e);
	val |= 0x0100;
	phy_write(phydev, 0x1e, val);

	/*check phy power*/
	val = phy_read(phydev, 0x0);
	if (val & BMCR_PDOWN)
		phy_write(phydev, 0x0, (val & ~BMCR_PDOWN));
	return 0;
}

static int mx6_arm2_fec_power_hibernate(struct phy_device *phydev)
{
	unsigned short val;

	/*set AR8031 debug reg 0xb to hibernate power*/
	phy_write(phydev, 0x1d, 0xb);
	val = phy_read(phydev, 0x1e);

	val |= 0x8000;
	phy_write(phydev, 0x1e, val);

	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init			= mx6_arm2_fec_phy_init,
	.power_hibernate	= mx6_arm2_fec_power_hibernate,
	.phy			= PHY_INTERFACE_MODE_RGMII,
#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
	.gpio_irq = MX6_ENET_IRQ,
#endif
};

static int mx6_arm2_spi_cs[] = {
	MX6_ARM2_ECSPI1_CS0,
	MX6_ARM2_ECSPI1_CS1,
};

static const struct spi_imx_master mx6_arm2_spi_data __initconst = {
	.chipselect     = mx6_arm2_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6_arm2_spi_cs),
};

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct mtd_partition m25p32_partitions[] = {
	{
		.name	= "bootloader",
		.offset	= 0,
		.size	= 0x00100000,
	}, {
		.name	= "kernel",
		.offset	= MTDPART_OFS_APPEND,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct flash_platform_data m25p32_spi_flash_data = {
	.name		= "m25p32",
	.parts		= m25p32_partitions,
	.nr_parts	= ARRAY_SIZE(m25p32_partitions),
	.type		= "m25p32",
};
#endif

static struct spi_board_info m25p32_spi0_board_info[] __initdata = {
#if defined(CONFIG_MTD_M25P80)
	{
	/* The modalias must be the same as spi device driver name */
	.modalias	= "m25p80",
	.max_speed_hz	= 20000000,
	.bus_num	= 0,
	.chip_select	= 1,
	.platform_data	= &m25p32_spi_flash_data,
	},
#endif
};

static void spi_device_init(void)
{
	spi_register_board_info(m25p32_spi0_board_info,
				ARRAY_SIZE(m25p32_spi0_board_info));
}

static int max7310_1_setup(struct i2c_client *client,
	unsigned gpio_base, unsigned ngpio,
	void *context)
{
	int max7310_gpio_value[] = { 0, 1, 0, 1, 0, 0, 0, 0 };

	int n;

	 for (n = 0; n < ARRAY_SIZE(max7310_gpio_value); ++n) {
		gpio_request(gpio_base + n, "MAX7310 1 GPIO Expander");
		if (max7310_gpio_value[n] < 0)
			gpio_direction_input(gpio_base + n);
		else
			gpio_direction_output(gpio_base + n,
						max7310_gpio_value[n]);
		gpio_export(gpio_base + n, 0);
	}

	return 0;
}

static struct pca953x_platform_data max7310_platdata = {
	.gpio_base	= MX6_ARM2_MAX7310_1_BASE_ADDR,
	.invert		= 0,
	.setup		= max7310_1_setup,
};

static int max7310_u48_setup(struct i2c_client *client,
	unsigned gpio_base, unsigned ngpio,
	void *context)
{
	int max7310_gpio_value[] = { 1, 1, 1, 1, 0, 1, 0, 0 };

	int n;

	 for (n = 0; n < ARRAY_SIZE(max7310_gpio_value); ++n) {
		gpio_request(gpio_base + n, "MAX7310 U48 GPIO Expander");
		if (max7310_gpio_value[n] < 0)
			gpio_direction_input(gpio_base + n);
		else
			gpio_direction_output(gpio_base + n,
						max7310_gpio_value[n]);
		gpio_export(gpio_base + n, 0);
	}

	return 0;
}

static struct pca953x_platform_data max7310_u48_platdata = {
	.gpio_base	= MX6_ARM2_MAX7310_2_BASE_ADDR,
	.invert		= 0,
	.setup		= max7310_u48_setup,
};

static void ddc_dvi_init(void)
{
	/* enable DVI I2C */
	gpio_set_value(MX6_ARM2_DISP0_I2C_EN, 1);

	/* DISP0 Detect */
	gpio_request(MX6_ARM2_DISP0_DET_INT, "disp0-detect");
	gpio_direction_input(MX6_ARM2_DISP0_DET_INT);
}

static int ddc_dvi_update(void)
{
	/* DVI cable state */
	if (gpio_get_value(MX6_ARM2_DISP0_DET_INT) == 1)
		return 1;
	return 0;
}

static struct fsl_mxc_dvi_platform_data sabr_ddc_dvi_data = {
	.ipu_id		= 0,
	.disp_id	= 0,
	.init		= ddc_dvi_init,
	.update		= ddc_dvi_update,
};

static void mx6_csi0_io_init(void)
{
	if (0 == sgtl5000_en) {
		iomux_v3_cfg_t *sensor_pads = NULL;
		u32 sensor_pads_cnt;

		if (cpu_is_mx6q()) {
			sensor_pads = mx6q_arm2_csi0_sensor_pads;
			sensor_pads_cnt = \
				ARRAY_SIZE(mx6q_arm2_csi0_sensor_pads);
		} else if (cpu_is_mx6dl()) {
			sensor_pads = mx6dl_arm2_csi0_sensor_pads;
			sensor_pads_cnt = \
				ARRAY_SIZE(mx6dl_arm2_csi0_sensor_pads);
		}

		BUG_ON(!sensor_pads);
		mxc_iomux_v3_setup_multiple_pads(sensor_pads, sensor_pads_cnt);
	}
	/* Camera reset */
	gpio_request(MX6_ARM2_CSI0_RST, "cam-reset");
	gpio_direction_output(MX6_ARM2_CSI0_RST, 1);

	/* Camera power down */
	gpio_request(MX6_ARM2_CSI0_PWN, "cam-pwdn");
	gpio_direction_output(MX6_ARM2_CSI0_PWN, 1);
	msleep(1);
	gpio_set_value(MX6_ARM2_CSI0_PWN, 0);

	/* For MX6Q:
	 * GPR1 bit19 and bit20 meaning:
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
	 *
	 * For MX6DL:
	 * GPR1 bit 21 and GPR13 bit 0-5, RM has detail information
	 */
	if (cpu_is_mx6q())
		mxc_iomux_set_gpr_register(1, 19, 1, 1);
	else if (cpu_is_mx6dl())
		mxc_iomux_set_gpr_register(13, 0, 3, 4);
}

static struct fsl_mxc_camera_platform_data camera_data = {
	.analog_regulator	= "DA9052_LDO7",
	.core_regulator		= "DA9052_LDO9",
	.mclk			= 24000000,
	.mclk_source = 0,
	.csi			= 0,
	.io_init		= mx6_csi0_io_init,
};

static void mx6_csi0_tvin_io_init(void)
{
	if (0 == sgtl5000_en) {
		iomux_v3_cfg_t *tvin_pads = NULL;
		u32 tvin_pads_cnt;

		if (cpu_is_mx6q()) {
			tvin_pads = mx6q_arm2_csi0_tvin_pads;
			tvin_pads_cnt = \
				ARRAY_SIZE(mx6q_arm2_csi0_tvin_pads);
		} else if (cpu_is_mx6dl()) {
			tvin_pads = mx6dl_arm2_csi0_tvin_pads;
			tvin_pads_cnt = \
				ARRAY_SIZE(mx6dl_arm2_csi0_tvin_pads);
		}

		BUG_ON(!tvin_pads);
		mxc_iomux_v3_setup_multiple_pads(tvin_pads, tvin_pads_cnt);
	}
	/* Tvin reset */
	gpio_request(MX6_ARM2_CSI0_RST_TVIN, "tvin-reset");
	gpio_direction_output(MX6_ARM2_CSI0_RST_TVIN, 1);

	/* Tvin power down */
	gpio_request(MX6_ARM2_CSI0_PWN, "cam-pwdn");
	gpio_direction_output(MX6_ARM2_CSI0_PWN, 0);
	msleep(1);
	gpio_set_value(MX6_ARM2_CSI0_PWN, 1);

	if (cpu_is_mx6q())
		mxc_iomux_set_gpr_register(1, 19, 1, 1);
	else if (cpu_is_mx6dl())
		mxc_iomux_set_gpr_register(13, 0, 3, 4);
}

static struct fsl_mxc_tvin_platform_data tvin_data = {
	.io_init = mx6_csi0_tvin_io_init,
	.cvbs = false,
};

static void mx6_mipi_sensor_io_init(void)
{
	iomux_v3_cfg_t *mipi_sensor_pads = NULL;
	u32 mipi_sensor_pads_cnt;

	if (cpu_is_mx6q()) {
		mipi_sensor_pads = mx6q_arm2_mipi_sensor_pads;
		mipi_sensor_pads_cnt = ARRAY_SIZE(mx6q_arm2_mipi_sensor_pads);
	} else if (cpu_is_mx6dl()) {
		mipi_sensor_pads = mx6dl_arm2_mipi_sensor_pads;
		mipi_sensor_pads_cnt = ARRAY_SIZE(mx6dl_arm2_mipi_sensor_pads);

	}
	BUG_ON(!mipi_sensor_pads);
	mxc_iomux_v3_setup_multiple_pads(mipi_sensor_pads,
					mipi_sensor_pads_cnt);

	/*for mx6dl, mipi virtual channel 1 connect to csi 1*/
	if (cpu_is_mx6dl())
		mxc_iomux_set_gpr_register(13, 3, 3, 1);
}

static struct fsl_mxc_camera_platform_data ov5640_mipi_data = {
	.mclk		= 24000000,
	.csi		= 1,
	.mclk_source = 0,
	.io_init	= mx6_mipi_sensor_io_init,
};

static struct mxc_audio_codec_platform_data cs42888_data = {
	.rates = (SNDRV_PCM_RATE_44100 |
			SNDRV_PCM_RATE_88200 |
			SNDRV_PCM_RATE_176400),
};

#define mV_to_uV(mV) (mV * 1000)
#define uV_to_mV(uV) (uV / 1000)
#define V_to_uV(V) (mV_to_uV(V * 1000))
#define uV_to_V(uV) (uV_to_mV(uV) / 1000)

static struct regulator_consumer_supply display_consumers[] = {
	{
		/* MAX17135 */
		.supply = "DISPLAY",
	},
};

static struct regulator_consumer_supply vcom_consumers[] = {
	{
		/* MAX17135 */
		.supply = "VCOM",
	},
};

static struct regulator_consumer_supply v3p3_consumers[] = {
	{
		/* MAX17135 */
		.supply = "V3P3",
	},
};

static struct regulator_init_data max17135_init_data[] = {
	{
		.constraints = {
			.name = "DISPLAY",
			.valid_ops_mask =  REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(display_consumers),
		.consumer_supplies = display_consumers,
	}, {
		.constraints = {
			.name = "GVDD",
			.min_uV = V_to_uV(20),
			.max_uV = V_to_uV(20),
		},
	}, {
		.constraints = {
			.name = "GVEE",
			.min_uV = V_to_uV(-22),
			.max_uV = V_to_uV(-22),
		},
	}, {
		.constraints = {
			.name = "HVINN",
			.min_uV = V_to_uV(-22),
			.max_uV = V_to_uV(-22),
		},
	}, {
		.constraints = {
			.name = "HVINP",
			.min_uV = V_to_uV(20),
			.max_uV = V_to_uV(20),
		},
	}, {
		.constraints = {
			.name = "VCOM",
			.min_uV = mV_to_uV(-4325),
			.max_uV = mV_to_uV(-500),
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(vcom_consumers),
		.consumer_supplies = vcom_consumers,
	}, {
		.constraints = {
			.name = "VNEG",
			.min_uV = V_to_uV(-15),
			.max_uV = V_to_uV(-15),
		},
	}, {
		.constraints = {
			.name = "VPOS",
			.min_uV = V_to_uV(15),
			.max_uV = V_to_uV(15),
		},
	}, {
		.constraints = {
			.name = "V3P3",
			.valid_ops_mask =  REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(v3p3_consumers),
		.consumer_supplies = v3p3_consumers,
	},
};

static struct platform_device max17135_sensor_device = {
	.name = "max17135_sensor",
	.id = 0,
};

static struct max17135_platform_data max17135_pdata __initdata = {
	.vneg_pwrup = 1,
	.gvee_pwrup = 1,
	.vpos_pwrup = 2,
	.gvdd_pwrup = 1,
	.gvdd_pwrdn = 1,
	.vpos_pwrdn = 2,
	.gvee_pwrdn = 1,
	.vneg_pwrdn = 1,
	.gpio_pmic_pwrgood = MX6DL_ARM2_EPDC_PWRSTAT,
	.gpio_pmic_vcom_ctrl = MX6DL_ARM2_EPDC_VCOM,
	.gpio_pmic_wakeup = MX6DL_ARM2_EPDC_PMIC_WAKE,
	.gpio_pmic_v3p3 = MX6DL_ARM2_EPDC_PWRCTRL0,
	.gpio_pmic_intr = MX6DL_ARM2_EPDC_PMIC_INT,
	.regulator_init = max17135_init_data,
	.init = max17135_regulator_init,
};

static int __init max17135_regulator_init(struct max17135 *max17135)
{
	struct max17135_platform_data *pdata = &max17135_pdata;
	int i, ret;

	if (!epdc_enabled) {
		printk(KERN_DEBUG
			"max17135_regulator_init abort: EPDC not enabled\n");
		return 0;
	}

	max17135->gvee_pwrup = pdata->gvee_pwrup;
	max17135->vneg_pwrup = pdata->vneg_pwrup;
	max17135->vpos_pwrup = pdata->vpos_pwrup;
	max17135->gvdd_pwrup = pdata->gvdd_pwrup;
	max17135->gvdd_pwrdn = pdata->gvdd_pwrdn;
	max17135->vpos_pwrdn = pdata->vpos_pwrdn;
	max17135->vneg_pwrdn = pdata->vneg_pwrdn;
	max17135->gvee_pwrdn = pdata->gvee_pwrdn;

	max17135->max_wait = pdata->vpos_pwrup + pdata->vneg_pwrup +
		pdata->gvdd_pwrup + pdata->gvee_pwrup;

	max17135->gpio_pmic_pwrgood = pdata->gpio_pmic_pwrgood;
	max17135->gpio_pmic_vcom_ctrl = pdata->gpio_pmic_vcom_ctrl;
	max17135->gpio_pmic_wakeup = pdata->gpio_pmic_wakeup;
	max17135->gpio_pmic_v3p3 = pdata->gpio_pmic_v3p3;
	max17135->gpio_pmic_intr = pdata->gpio_pmic_intr;

	gpio_request(max17135->gpio_pmic_wakeup, "epdc-pmic-wake");
	gpio_direction_output(max17135->gpio_pmic_wakeup, 0);

	gpio_request(max17135->gpio_pmic_vcom_ctrl, "epdc-vcom");
	gpio_direction_output(max17135->gpio_pmic_vcom_ctrl, 0);

	gpio_request(max17135->gpio_pmic_v3p3, "epdc-v3p3");
	gpio_direction_output(max17135->gpio_pmic_v3p3, 0);

	gpio_request(max17135->gpio_pmic_intr, "epdc-pmic-int");
	gpio_direction_input(max17135->gpio_pmic_intr);

	gpio_request(max17135->gpio_pmic_pwrgood, "epdc-pwrstat");
	gpio_direction_input(max17135->gpio_pmic_pwrgood);

	max17135->vcom_setup = false;
	max17135->init_done = false;

	for (i = 0; i < MAX17135_NUM_REGULATORS; i++) {
		ret = max17135_register_regulator(max17135, i,
			&pdata->regulator_init[i]);
		if (ret != 0) {
			printk(KERN_ERR"max17135 regulator init failed: %d\n",
				ret);
			return ret;
		}
	}

	regulator_has_full_constraints();

	return 0;
}

static int sii902x_get_pins(void)
{
	/* Sii902x HDMI controller */
	gpio_request(MX6_ARM2_DISP0_RESET, "disp0-reset");
	gpio_direction_output(MX6_ARM2_DISP0_RESET, 0);
	gpio_request(MX6_ARM2_DISP0_DET_INT, "disp0-detect");
	gpio_direction_input(MX6_ARM2_DISP0_DET_INT);
	return 1;
}

static void sii902x_put_pins(void)
{
	gpio_free(MX6_ARM2_DISP0_RESET);
	gpio_free(MX6_ARM2_DISP0_DET_INT);
}

static void sii902x_hdmi_reset(void)
{
       gpio_set_value(MX6_ARM2_DISP0_RESET, 0);
       msleep(10);
       gpio_set_value(MX6_ARM2_DISP0_RESET, 1);
       msleep(10);
}

static struct fsl_mxc_lcd_platform_data sii902x_hdmi_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.reset = sii902x_hdmi_reset,
	.get_pins = sii902x_get_pins,
	.put_pins = sii902x_put_pins,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("cs42888", 0x48),
		.platform_data = (void *)&cs42888_data,
	}, {
		I2C_BOARD_INFO("ov5640", 0x3c),
		.platform_data = (void *)&camera_data,
	}, {
		I2C_BOARD_INFO("adv7180", 0x21),
		.platform_data = (void *)&tvin_data,
	},
};

static struct imxi2c_platform_data mx6_arm2_i2c_data = {
	.bitrate = 100000,
};

static struct imxi2c_platform_data mx6_arm2_i2c2_data = {
	.bitrate = 400000,
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("max17135", 0x48),
		.platform_data = &max17135_pdata,
	}, {
		I2C_BOARD_INFO("max7310", 0x1F),
		.platform_data = &max7310_platdata,
	}, {
		I2C_BOARD_INFO("max7310", 0x1B),
		.platform_data = &max7310_u48_platdata,
	}, {
		I2C_BOARD_INFO("mxc_dvi", 0x50),
		.platform_data = &sabr_ddc_dvi_data,
		.irq = gpio_to_irq(MX6_ARM2_DISP0_DET_INT),
	}, {
		I2C_BOARD_INFO("egalax_ts", 0x4),
		.irq = gpio_to_irq(MX6_ARM2_CAP_TCH_INT),
	}, {
		I2C_BOARD_INFO("sii902x", 0x39),
		.platform_data = &sii902x_hdmi_data,
		.irq = gpio_to_irq(MX6_ARM2_DISP0_DET_INT),
	},
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("egalax_ts", 0x4),
		.irq = gpio_to_irq(MX6_ARM2_CAP_TCH_INT),
	}, {
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	}, {
		I2C_BOARD_INFO("ov5640_mipi", 0x3c),
		.platform_data = (void *)&ov5640_mipi_data,
	}, {
		I2C_BOARD_INFO("sgtl5000", 0x0a),
	},
};

static int epdc_get_pins(void)
{
	int ret = 0;

	/* Claim GPIOs for EPDC pins - used during power up/down */
	ret |= gpio_request(MX6DL_ARM2_EPDC_SDDO_0, "epdc_d0");
	ret |= gpio_request(MX6DL_ARM2_EPDC_SDDO_1, "epdc_d1");
	ret |= gpio_request(MX6DL_ARM2_EPDC_SDDO_2, "epdc_d2");
	ret |= gpio_request(MX6DL_ARM2_EPDC_SDDO_3, "epdc_d3");
	ret |= gpio_request(MX6DL_ARM2_EPDC_SDDO_4, "epdc_d4");
	ret |= gpio_request(MX6DL_ARM2_EPDC_SDDO_5, "epdc_d5");
	ret |= gpio_request(MX6DL_ARM2_EPDC_SDDO_6, "epdc_d6");
	ret |= gpio_request(MX6DL_ARM2_EPDC_SDDO_7, "epdc_d7");
	ret |= gpio_request(MX6DL_ARM2_EPDC_GDCLK, "epdc_gdclk");
	ret |= gpio_request(MX6DL_ARM2_EPDC_GDSP, "epdc_gdsp");
	ret |= gpio_request(MX6DL_ARM2_EPDC_GDOE, "epdc_gdoe");
	ret |= gpio_request(MX6DL_ARM2_EPDC_GDRL, "epdc_gdrl");
	ret |= gpio_request(MX6DL_ARM2_EPDC_SDCLK, "epdc_sdclk");
	ret |= gpio_request(MX6DL_ARM2_EPDC_SDOE, "epdc_sdoe");
	ret |= gpio_request(MX6DL_ARM2_EPDC_SDLE, "epdc_sdle");
	ret |= gpio_request(MX6DL_ARM2_EPDC_SDSHR, "epdc_sdshr");
	ret |= gpio_request(MX6DL_ARM2_EPDC_BDR0, "epdc_bdr0");
	ret |= gpio_request(MX6DL_ARM2_EPDC_SDCE0, "epdc_sdce0");
	ret |= gpio_request(MX6DL_ARM2_EPDC_SDCE1, "epdc_sdce1");
	ret |= gpio_request(MX6DL_ARM2_EPDC_SDCE2, "epdc_sdce2");

	return ret;
}

static void epdc_put_pins(void)
{
	gpio_free(MX6DL_ARM2_EPDC_SDDO_0);
	gpio_free(MX6DL_ARM2_EPDC_SDDO_1);
	gpio_free(MX6DL_ARM2_EPDC_SDDO_2);
	gpio_free(MX6DL_ARM2_EPDC_SDDO_3);
	gpio_free(MX6DL_ARM2_EPDC_SDDO_4);
	gpio_free(MX6DL_ARM2_EPDC_SDDO_5);
	gpio_free(MX6DL_ARM2_EPDC_SDDO_6);
	gpio_free(MX6DL_ARM2_EPDC_SDDO_7);
	gpio_free(MX6DL_ARM2_EPDC_GDCLK);
	gpio_free(MX6DL_ARM2_EPDC_GDSP);
	gpio_free(MX6DL_ARM2_EPDC_GDOE);
	gpio_free(MX6DL_ARM2_EPDC_GDRL);
	gpio_free(MX6DL_ARM2_EPDC_SDCLK);
	gpio_free(MX6DL_ARM2_EPDC_SDOE);
	gpio_free(MX6DL_ARM2_EPDC_SDLE);
	gpio_free(MX6DL_ARM2_EPDC_SDSHR);
	gpio_free(MX6DL_ARM2_EPDC_BDR0);
	gpio_free(MX6DL_ARM2_EPDC_SDCE0);
	gpio_free(MX6DL_ARM2_EPDC_SDCE1);
	gpio_free(MX6DL_ARM2_EPDC_SDCE2);
}

static iomux_v3_cfg_t mx6dl_epdc_pads_enabled[] = {
	MX6DL_PAD_EIM_A16__EPDC_SDDO_0,
	MX6DL_PAD_EIM_DA10__EPDC_SDDO_1,
	MX6DL_PAD_EIM_DA12__EPDC_SDDO_2,
	MX6DL_PAD_EIM_DA11__EPDC_SDDO_3,
	MX6DL_PAD_EIM_LBA__EPDC_SDDO_4,
	MX6DL_PAD_EIM_EB2__EPDC_SDDO_5,
	MX6DL_PAD_EIM_CS0__EPDC_SDDO_6,
	MX6DL_PAD_EIM_RW__EPDC_SDDO_7,
	MX6DL_PAD_EIM_CS1__EPDC_SDDO_8,
	MX6DL_PAD_EIM_DA15__EPDC_SDDO_9,
	MX6DL_PAD_EIM_D16__EPDC_SDDO_10,
	MX6DL_PAD_EIM_D23__EPDC_SDDO_11,
	MX6DL_PAD_EIM_D19__EPDC_SDDO_12,
	MX6DL_PAD_EIM_DA13__EPDC_SDDO_13,
	MX6DL_PAD_EIM_DA14__EPDC_SDDO_14,
	MX6DL_PAD_EIM_A25__EPDC_SDDO_15,
	MX6DL_PAD_EIM_A21__EPDC_GDCLK,
	MX6DL_PAD_EIM_A22__EPDC_GDSP,
	MX6DL_PAD_EIM_A23__EPDC_GDOE,
	MX6DL_PAD_EIM_A24__EPDC_GDRL,
	MX6DL_PAD_EIM_D31__EPDC_SDCLK,
	MX6DL_PAD_EIM_D27__EPDC_SDOE,
	MX6DL_PAD_EIM_DA1__EPDC_SDLE,
	MX6DL_PAD_EIM_EB1__EPDC_SDSHR,
	MX6DL_PAD_EIM_DA2__EPDC_BDR_0,
	MX6DL_PAD_EIM_DA4__EPDC_SDCE_0,
	MX6DL_PAD_EIM_DA5__EPDC_SDCE_1,
	MX6DL_PAD_EIM_DA6__EPDC_SDCE_2,
};

static iomux_v3_cfg_t mx6dl_epdc_pads_disabled[] = {
	MX6DL_PAD_EIM_A16__GPIO_2_22,
	MX6DL_PAD_EIM_DA10__GPIO_3_10,
	MX6DL_PAD_EIM_DA12__GPIO_3_12,
	MX6DL_PAD_EIM_DA11__GPIO_3_11,
	MX6DL_PAD_EIM_LBA__GPIO_2_27,
	MX6DL_PAD_EIM_EB2__GPIO_2_30,
	MX6DL_PAD_EIM_CS0__GPIO_2_23,
	MX6DL_PAD_EIM_RW__GPIO_2_26,
	MX6DL_PAD_EIM_CS1__GPIO_2_24,
	MX6DL_PAD_EIM_DA15__GPIO_3_15,
	MX6DL_PAD_EIM_D16__GPIO_3_16,
	MX6DL_PAD_EIM_D23__GPIO_3_23,
	MX6DL_PAD_EIM_D19__GPIO_3_19,
	MX6DL_PAD_EIM_DA13__GPIO_3_13,
	MX6DL_PAD_EIM_DA14__GPIO_3_14,
	MX6DL_PAD_EIM_A25__GPIO_5_2,
	MX6DL_PAD_EIM_A21__GPIO_2_17,
	MX6DL_PAD_EIM_A22__GPIO_2_16,
	MX6DL_PAD_EIM_A23__GPIO_6_6,
	MX6DL_PAD_EIM_A24__GPIO_5_4,
	MX6DL_PAD_EIM_D31__GPIO_3_31,
	MX6DL_PAD_EIM_D27__GPIO_3_27,
	MX6DL_PAD_EIM_DA1__GPIO_3_1,
	MX6DL_PAD_EIM_EB1__GPIO_2_29,
	MX6DL_PAD_EIM_DA2__GPIO_3_2,
	MX6DL_PAD_EIM_DA4__GPIO_3_4,
	MX6DL_PAD_EIM_DA5__GPIO_3_5,
	MX6DL_PAD_EIM_DA6__GPIO_3_6,
};
static void epdc_enable_pins(void)
{
	/* Configure MUX settings to enable EPDC use */
	mxc_iomux_v3_setup_multiple_pads(mx6dl_epdc_pads_enabled, \
				ARRAY_SIZE(mx6dl_epdc_pads_enabled));

	gpio_direction_input(MX6DL_ARM2_EPDC_SDDO_0);
	gpio_direction_input(MX6DL_ARM2_EPDC_SDDO_1);
	gpio_direction_input(MX6DL_ARM2_EPDC_SDDO_2);
	gpio_direction_input(MX6DL_ARM2_EPDC_SDDO_3);
	gpio_direction_input(MX6DL_ARM2_EPDC_SDDO_4);
	gpio_direction_input(MX6DL_ARM2_EPDC_SDDO_5);
	gpio_direction_input(MX6DL_ARM2_EPDC_SDDO_6);
	gpio_direction_input(MX6DL_ARM2_EPDC_SDDO_7);
	gpio_direction_input(MX6DL_ARM2_EPDC_GDCLK);
	gpio_direction_input(MX6DL_ARM2_EPDC_GDSP);
	gpio_direction_input(MX6DL_ARM2_EPDC_GDOE);
	gpio_direction_input(MX6DL_ARM2_EPDC_GDRL);
	gpio_direction_input(MX6DL_ARM2_EPDC_SDCLK);
	gpio_direction_input(MX6DL_ARM2_EPDC_SDOE);
	gpio_direction_input(MX6DL_ARM2_EPDC_SDLE);
	gpio_direction_input(MX6DL_ARM2_EPDC_SDSHR);
	gpio_direction_input(MX6DL_ARM2_EPDC_BDR0);
	gpio_direction_input(MX6DL_ARM2_EPDC_SDCE0);
	gpio_direction_input(MX6DL_ARM2_EPDC_SDCE1);
	gpio_direction_input(MX6DL_ARM2_EPDC_SDCE2);
}

static void epdc_disable_pins(void)
{
	/* Configure MUX settings for EPDC pins to
	 * GPIO and drive to 0. */
	mxc_iomux_v3_setup_multiple_pads(mx6dl_epdc_pads_disabled, \
				ARRAY_SIZE(mx6dl_epdc_pads_disabled));

	gpio_direction_output(MX6DL_ARM2_EPDC_SDDO_0, 0);
	gpio_direction_output(MX6DL_ARM2_EPDC_SDDO_1, 0);
	gpio_direction_output(MX6DL_ARM2_EPDC_SDDO_2, 0);
	gpio_direction_output(MX6DL_ARM2_EPDC_SDDO_3, 0);
	gpio_direction_output(MX6DL_ARM2_EPDC_SDDO_4, 0);
	gpio_direction_output(MX6DL_ARM2_EPDC_SDDO_5, 0);
	gpio_direction_output(MX6DL_ARM2_EPDC_SDDO_6, 0);
	gpio_direction_output(MX6DL_ARM2_EPDC_SDDO_7, 0);
	gpio_direction_output(MX6DL_ARM2_EPDC_GDCLK, 0);
	gpio_direction_output(MX6DL_ARM2_EPDC_GDSP, 0);
	gpio_direction_output(MX6DL_ARM2_EPDC_GDOE, 0);
	gpio_direction_output(MX6DL_ARM2_EPDC_GDRL, 0);
	gpio_direction_output(MX6DL_ARM2_EPDC_SDCLK, 0);
	gpio_direction_output(MX6DL_ARM2_EPDC_SDOE, 0);
	gpio_direction_output(MX6DL_ARM2_EPDC_SDLE, 0);
	gpio_direction_output(MX6DL_ARM2_EPDC_SDSHR, 0);
	gpio_direction_output(MX6DL_ARM2_EPDC_BDR0, 0);
	gpio_direction_output(MX6DL_ARM2_EPDC_SDCE0, 0);
	gpio_direction_output(MX6DL_ARM2_EPDC_SDCE1, 0);
	gpio_direction_output(MX6DL_ARM2_EPDC_SDCE2, 0);
}

static struct fb_videomode e60_v110_mode = {
	.name = "E60_V110",
	.refresh = 50,
	.xres = 800,
	.yres = 600,
	.pixclock = 18604700,
	.left_margin = 8,
	.right_margin = 178,
	.upper_margin = 4,
	.lower_margin = 10,
	.hsync_len = 20,
	.vsync_len = 4,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};
static struct fb_videomode e60_v220_mode = {
	.name = "E60_V220",
	.refresh = 85,
	.xres = 800,
	.yres = 600,
	.pixclock = 30000000,
	.left_margin = 8,
	.right_margin = 164,
	.upper_margin = 4,
	.lower_margin = 8,
	.hsync_len = 4,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
	.refresh = 85,
	.xres = 800,
	.yres = 600,
};
static struct fb_videomode e060scm_mode = {
	.name = "E060SCM",
	.refresh = 85,
	.xres = 800,
	.yres = 600,
	.pixclock = 26666667,
	.left_margin = 8,
	.right_margin = 100,
	.upper_margin = 4,
	.lower_margin = 8,
	.hsync_len = 4,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};
static struct fb_videomode e97_v110_mode = {
	.name = "E97_V110",
	.refresh = 50,
	.xres = 1200,
	.yres = 825,
	.pixclock = 32000000,
	.left_margin = 12,
	.right_margin = 128,
	.upper_margin = 4,
	.lower_margin = 10,
	.hsync_len = 20,
	.vsync_len = 4,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct imx_epdc_fb_mode panel_modes[] = {
	{
		&e60_v110_mode,
		4,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		428,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		1,      /* gdclk_offs */
		1,      /* num_ce */
	},
	{
		&e60_v220_mode,
		4,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		465,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		9,      /* gdclk_offs */
		1,      /* num_ce */
	},
	{
		&e060scm_mode,
		4,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		419,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		5,      /* gdclk_offs */
		1,      /* num_ce */
	},
	{
		&e97_v110_mode,
		8,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		632,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		1,      /* gdclk_offs */
		3,      /* num_ce */
	}
};

static struct imx_epdc_fb_platform_data epdc_data = {
	.epdc_mode = panel_modes,
	.num_modes = ARRAY_SIZE(panel_modes),
	.get_pins = epdc_get_pins,
	.put_pins = epdc_put_pins,
	.enable_pins = epdc_enable_pins,
	.disable_pins = epdc_disable_pins,
};

static void imx6_arm2_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(MX6_ARM2_USB_OTG_PWR, 1);
	else
		gpio_set_value(MX6_ARM2_USB_OTG_PWR, 0);
}

static void __init mx6_arm2_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);

	/* disable external charger detect,
	 * or it will affect signal quality at dp.
	 */

	ret = gpio_request(MX6_ARM2_USB_OTG_PWR, "usb-pwr");
	if (ret) {
		pr_err("failed to get GPIO MX6_ARM2_USB_OTG_PWR:%d\n", ret);
		return;
	}
	gpio_direction_output(MX6_ARM2_USB_OTG_PWR, 0);
	mxc_iomux_set_gpr_register(1, 13, 1, 1);

	mx6_set_otghost_vbus_func(imx6_arm2_usbotg_vbus);

#ifdef CONFIG_USB_EHCI_ARC_HSIC
	mx6_usb_h2_init();
	mx6_usb_h3_init();
#endif
}

static struct viv_gpu_platform_data imx6_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M,
};

/* HW Initialization, if return 0, initialization is successful. */
static int mx6_arm2_sata_init(struct device *dev, void __iomem *addr)
{
	u32 tmpdata;
	int ret = 0;
	struct clk *clk;

	/* Enable SATA PWR CTRL_0 of MAX7310 */
	gpio_request(MX6_ARM2_MAX7310_1_BASE_ADDR, "SATA_PWR_EN");
	gpio_direction_output(MX6_ARM2_MAX7310_1_BASE_ADDR, 1);

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

#ifdef CONFIG_SATA_AHCI_PLATFORM
	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;
#else
	usleep_range(1000, 2000);
	/* AHCI PHY enter into PDDQ mode if the AHCI module is not enabled */
	tmpdata = readl(addr + PORT_PHY_CTL);
	writel(tmpdata | PORT_PHY_CTL_PDDQ_LOC, addr + PORT_PHY_CTL);
	pr_info("No AHCI save PWR: PDDQ %s\n", ((readl(addr + PORT_PHY_CTL)
					>> 20) & 1) ? "enabled" : "disabled");
#endif

release_sata_clk:
	/* disable SATA_PHY PLL */
	writel((readl(IOMUXC_GPR13) & ~0x2), IOMUXC_GPR13);
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);
	/* Disable SATA PWR CTRL_0 of MAX7310 */
	gpio_request(MX6_ARM2_MAX7310_1_BASE_ADDR, "SATA_PWR_EN");
	gpio_direction_output(MX6_ARM2_MAX7310_1_BASE_ADDR, 0);

	return ret;
}

#ifdef CONFIG_SATA_AHCI_PLATFORM
static void mx6_arm2_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);

	/* Disable SATA PWR CTRL_0 of MAX7310 */
	gpio_request(MX6_ARM2_MAX7310_1_BASE_ADDR, "SATA_PWR_EN");
	gpio_direction_output(MX6_ARM2_MAX7310_1_BASE_ADDR, 0);

}

static struct ahci_platform_data mx6_arm2_sata_data = {
	.init	= mx6_arm2_sata_init,
	.exit	= mx6_arm2_sata_exit,
};
#endif

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits	= 4,
	.clk_map_ver	= 2,
};

static void mx6_arm2_reset_mipi_dsi(void)
{
	gpio_set_value(MX6_ARM2_DISP0_PWR, 1);
	gpio_set_value(MX6_ARM2_DISP0_RESET, 1);
	udelay(10);
	gpio_set_value(MX6_ARM2_DISP0_RESET, 0);
	udelay(50);
	gpio_set_value(MX6_ARM2_DISP0_RESET, 1);

	/*
	 * it needs to delay 120ms minimum for reset complete
	 */
	msleep(120);
}

static struct mipi_dsi_platform_data mipi_dsi_pdata = {
	.ipu_id		= 1,
	.disp_id	= 1,
	.lcd_panel	= "TRULY-WVGA",
	.reset		= mx6_arm2_reset_mipi_dsi,
};

static struct ipuv3_fb_platform_data sabr_fb_data[] = {
	{ /*fb0*/
	.disp_dev		= "ldb",
	.interface_pix_fmt	= IPU_PIX_FMT_RGB666,
	.mode_str		= "LDB-XGA",
	.default_bpp		= 16,
	.int_clk		= false,
	}, {
	.disp_dev		= "mipi_dsi",
	.interface_pix_fmt	= IPU_PIX_FMT_RGB24,
	.mode_str		= "TRULY-WVGA",
	.default_bpp		= 24,
	.int_clk		= false,
	}, {
	.disp_dev		= "ldb",
	.interface_pix_fmt	= IPU_PIX_FMT_RGB666,
	.mode_str		= "LDB-XGA",
	.default_bpp		= 16,
	.int_clk		= false,
	}, {
	.disp_dev		= "lcd",
	.interface_pix_fmt	= IPU_PIX_FMT_RGB565,
	.mode_str		= "CLAA-WVGA",
	.default_bpp		= 16,
	.int_clk		= false,
	}
};

static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;
	int max_ipu_id = cpu_is_mx6q() ? 1 : 0;

	if ((ipu_id > max_ipu_id) || (ipu_id < 0)) {
		pr_err("Invalid IPU select for HDMI: %d. Set to 0\n", ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0)) {
		pr_err("Invalid DI select for HDMI: %d. Set to 0\n", disp_id);
		disp_id = 0;
	}

	/* Configure the connection between IPU1/2 and HDMI */
	hdmi_mux_setting = 2 * ipu_id + disp_id;

	/* GPR3, bits 2-3 = HDMI_MUX_CTL */
	mxc_iomux_set_gpr_register(3, 2, 2, hdmi_mux_setting);

	/* Set HDMI event as SDMA event2 while Chip version later than TO1.2 */
	if (hdmi_SDMA_check())
		mxc_iomux_set_gpr_register(0, 0, 1, 1);
}

/* On mx6x arm2 board i2c2 iomux with hdmi ddc,
 * the pins default work at i2c2 function,
 when hdcp enable, the pins should work at ddc function */

static void hdmi_enable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_arm2_hdmi_ddc_pads,
			ARRAY_SIZE(mx6dl_arm2_hdmi_ddc_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_arm2_hdmi_ddc_pads,
			ARRAY_SIZE(mx6q_arm2_hdmi_ddc_pads));
}

static void hdmi_disable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_arm2_i2c2_pads,
			ARRAY_SIZE(mx6dl_arm2_i2c2_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_arm2_i2c2_pads,
			ARRAY_SIZE(mx6q_arm2_i2c2_pads));
}

static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init = hdmi_init,
	.enable_pins = hdmi_enable_ddc_pin,
	.disable_pins = hdmi_disable_ddc_pin,
};

static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data = {
	.ipu_id		= 0,
	.disp_id	= 0,
};

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id		= 0,
	.disp_id	= 0,
	.default_ifmt	= IPU_PIX_FMT_RGB565,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id		= 1,
	.disp_id	= 0,
	.ext_ref	= 1,
	.mode		= LDB_SEP0,
	.sec_ipu_id	= 0,
	.sec_disp_id	= 1,
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
	.rev		= 4,
	.csi_clk[0]	= "clko_clk",
	}, {
	.rev		= 4,
	.csi_clk[0]	= "clko_clk",
	},
};

static struct platform_pwm_backlight_data mx6_arm2_pwm_backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= 255,
	.dft_brightness	= 128,
	.pwm_period_ns	= 50000,
};

static struct gpio mx6_flexcan_gpios[] = {
	{ MX6_ARM2_CAN1_EN, GPIOF_OUT_INIT_LOW, "flexcan1-en" },
	{ MX6_ARM2_CAN1_STBY, GPIOF_OUT_INIT_LOW, "flexcan1-stby" },
	{ MX6_ARM2_CAN2_EN, GPIOF_OUT_INIT_LOW, "flexcan2-en" },
};

static void mx6_flexcan0_switch(int enable)
{
	if (enable) {
		gpio_set_value(MX6_ARM2_CAN1_EN, 1);
		gpio_set_value(MX6_ARM2_CAN1_STBY, 1);
	} else {
		gpio_set_value(MX6_ARM2_CAN1_EN, 0);
		gpio_set_value(MX6_ARM2_CAN1_STBY, 0);
	}
}

static void mx6_flexcan1_switch(int enable)
{
	if (enable) {
		gpio_set_value(MX6_ARM2_CAN2_EN, 1);
		gpio_set_value_cansleep(MX6_ARM2_CAN2_STBY, 1);
	} else {
		gpio_set_value(MX6_ARM2_CAN2_EN, 0);
		gpio_set_value_cansleep(MX6_ARM2_CAN2_STBY, 0);
	}
}

static const struct flexcan_platform_data
		mx6_arm2_flexcan_pdata[] __initconst = {
	{
		.transceiver_switch = mx6_flexcan0_switch,
	}, {
		.transceiver_switch = mx6_flexcan1_switch,
	}
};

static struct mipi_csi2_platform_data mipi_csi2_pdata = {
	.ipu_id		= 0,
	.csi_id		= 0,
	.v_channel	= 0,
	.lanes		= 2,
	.dphy_clk	= "mipi_pllref_clk",
	.pixel_clk	= "emi_clk",
};

static struct fsl_mxc_capture_platform_data capture_data[] = {
	{
		.csi = 0,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 0,
	}, {
		.csi = 1,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 1,
	},
};


static void arm2_suspend_enter(void)
{
	/* suspend preparation */
}

static void arm2_suspend_exit(void)
{
	/* resmue resore */
}
static const struct pm_platform_data mx6_arm2_pm_data __initconst = {
	.name		= "imx_pm",
	.suspend_enter	= arm2_suspend_enter,
	.suspend_exit	= arm2_suspend_exit,
};

static const struct asrc_p2p_params esai_p2p = {
       .p2p_rate = 44100,
       .p2p_width = ASRC_WIDTH_24_BIT,
};

static struct mxc_audio_platform_data sab_audio_data = {
	.sysclk	= 16934400,
	.priv = (void *)&esai_p2p,
};

static struct platform_device sab_audio_device = {
	.name	= "imx-cs42888",
};

static struct imx_esai_platform_data sab_esai_pdata = {
	.flags	= IMX_ESAI_NET,
};

static struct regulator_consumer_supply arm2_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data arm2_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(arm2_vmmc_consumers),
	.consumer_supplies = arm2_vmmc_consumers,
};

static struct fixed_voltage_config arm2_vmmc_reg_config = {
	.supply_name	= "vmmc",
	.microvolts	= 3300000,
	.gpio		= -1,
	.init_data	= &arm2_vmmc_init,
};

static struct platform_device arm2_vmmc_reg_devices = {
	.name		= "reg-fixed-voltage",
	.id		= 0,
	.dev		= {
		.platform_data = &arm2_vmmc_reg_config,
	},
};

#ifdef CONFIG_SND_SOC_CS42888

static struct regulator_consumer_supply cs42888_arm2_consumer_va = {
	.supply		= "VA",
	.dev_name	= "0-0048",
};

static struct regulator_consumer_supply cs42888_arm2_consumer_vd = {
	.supply		= "VD",
	.dev_name	= "0-0048",
};

static struct regulator_consumer_supply cs42888_arm2_consumer_vls = {
	.supply		= "VLS",
	.dev_name	= "0-0048",
};

static struct regulator_consumer_supply cs42888_arm2_consumer_vlc = {
	.supply		= "VLC",
	.dev_name	= "0-0048",
};

static struct regulator_init_data cs42888_arm2_va_reg_initdata = {
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &cs42888_arm2_consumer_va,
};

static struct regulator_init_data cs42888_arm2_vd_reg_initdata = {
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &cs42888_arm2_consumer_vd,
};

static struct regulator_init_data cs42888_arm2_vls_reg_initdata = {
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &cs42888_arm2_consumer_vls,
};

static struct regulator_init_data cs42888_arm2_vlc_reg_initdata = {
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &cs42888_arm2_consumer_vlc,
};

static struct fixed_voltage_config cs42888_arm2_va_reg_config = {
	.supply_name		= "VA",
	.microvolts		= 2800000,
	.gpio			= -1,
	.init_data		= &cs42888_arm2_va_reg_initdata,
};

static struct fixed_voltage_config cs42888_arm2_vd_reg_config = {
	.supply_name		= "VD",
	.microvolts		= 2800000,
	.gpio			= -1,
	.init_data		= &cs42888_arm2_vd_reg_initdata,
};

static struct fixed_voltage_config cs42888_arm2_vls_reg_config = {
	.supply_name		= "VLS",
	.microvolts		= 2800000,
	.gpio			= -1,
	.init_data		= &cs42888_arm2_vls_reg_initdata,
};

static struct fixed_voltage_config cs42888_arm2_vlc_reg_config = {
	.supply_name		= "VLC",
	.microvolts		= 2800000,
	.gpio			= -1,
	.init_data		= &cs42888_arm2_vlc_reg_initdata,
};

static struct platform_device cs42888_arm2_va_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &cs42888_arm2_va_reg_config,
	},
};

static struct platform_device cs42888_arm2_vd_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 4,
	.dev	= {
		.platform_data = &cs42888_arm2_vd_reg_config,
	},
};

static struct platform_device cs42888_arm2_vls_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 5,
	.dev	= {
		.platform_data = &cs42888_arm2_vls_reg_config,
	},
};

static struct platform_device cs42888_arm2_vlc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 6,
	.dev	= {
		.platform_data = &cs42888_arm2_vlc_reg_config,
	},
};

#endif /* CONFIG_SND_SOC_CS42888 */

#ifdef CONFIG_SND_SOC_SGTL5000

static struct regulator_consumer_supply sgtl5000_arm2_consumer_vdda = {
	.supply		= "VDDA",
	.dev_name	= "1-000a",
};

static struct regulator_consumer_supply sgtl5000_arm2_consumer_vddio = {
	.supply		= "VDDIO",
	.dev_name	= "1-000a",
};

static struct regulator_consumer_supply sgtl5000_arm2_consumer_vddd = {
	.supply		= "VDDD",
	.dev_name	= "1-000a",
};

static struct regulator_init_data sgtl5000_arm2_vdda_reg_initdata = {
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &sgtl5000_arm2_consumer_vdda,
};

static struct regulator_init_data sgtl5000_arm2_vddio_reg_initdata = {
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &sgtl5000_arm2_consumer_vddio,
};

static struct regulator_init_data sgtl5000_arm2_vddd_reg_initdata = {
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &sgtl5000_arm2_consumer_vddd,
};

static struct fixed_voltage_config sgtl5000_arm2_vdda_reg_config = {
	.supply_name		= "VDDA",
	.microvolts		= 1800000,
	.gpio			= -1,
	.init_data		= &sgtl5000_arm2_vdda_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_arm2_vddio_reg_config = {
	.supply_name		= "VDDIO",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &sgtl5000_arm2_vddio_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_arm2_vddd_reg_config = {
	.supply_name		= "VDDD",
	.microvolts		= 0,
	.gpio			= -1,
	.init_data		= &sgtl5000_arm2_vddd_reg_initdata,
};

static struct platform_device sgtl5000_arm2_vdda_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 7,
	.dev	= {
		.platform_data = &sgtl5000_arm2_vdda_reg_config,
	},
};

static struct platform_device sgtl5000_arm2_vddio_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 8,
	.dev	= {
		.platform_data = &sgtl5000_arm2_vddio_reg_config,
	},
};

static struct platform_device sgtl5000_arm2_vddd_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 9,
	.dev	= {
		.platform_data = &sgtl5000_arm2_vddd_reg_config,
	},
};

#endif /* CONFIG_SND_SOC_SGTL5000 */

static struct mxc_audio_platform_data mx6_arm2_audio_data;

static int mx6_arm2_sgtl5000_init(void)
{

	mx6_arm2_audio_data.sysclk = 12000000;

	return 0;
}

static struct imx_ssi_platform_data mx6_arm2_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct mxc_audio_platform_data mx6_arm2_audio_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.init = mx6_arm2_sgtl5000_init,
	.hp_gpio = -1,
};

static struct platform_device mx6_arm2_audio_device = {
	.name = "imx-sgtl5000",
};

static int __init mx6_arm2_init_audio(void)
{
	struct clk *pll3_pfd, *esai_clk;
	mxc_register_device(&sab_audio_device, &sab_audio_data);
	imx6q_add_imx_esai(0, &sab_esai_pdata);

	esai_clk = clk_get(NULL, "esai_clk");
	if (IS_ERR(esai_clk))
		return PTR_ERR(esai_clk);

	pll3_pfd = clk_get(NULL, "pll3_pfd_508M");
	if (IS_ERR(pll3_pfd))
		return PTR_ERR(pll3_pfd);

	clk_set_parent(esai_clk, pll3_pfd);
	clk_set_rate(esai_clk, 101647058);

#ifdef CONFIG_SND_SOC_CS42888
	platform_device_register(&cs42888_arm2_va_reg_devices);
	platform_device_register(&cs42888_arm2_vd_reg_devices);
	platform_device_register(&cs42888_arm2_vls_reg_devices);
	platform_device_register(&cs42888_arm2_vlc_reg_devices);
#endif

	if (sgtl5000_en) {
		/* SSI audio init part */
		mxc_register_device(&mx6_arm2_audio_device,
				&mx6_arm2_audio_data);
		imx6q_add_imx_ssi(1, &mx6_arm2_ssi_pdata);

		/*
		 * AUDMUX3 and CSI0_Camera use the same pin
		 * MX6x_PAD_CSI0_DAT5
		 */
		if (cpu_is_mx6q()) {
			mxc_iomux_v3_setup_multiple_pads(mx6q_arm2_audmux_pads,
					ARRAY_SIZE(mx6q_arm2_audmux_pads));
		} else if (cpu_is_mx6dl()) {
			mxc_iomux_v3_setup_multiple_pads(mx6dl_arm2_audmux_pads,
					ARRAY_SIZE(mx6dl_arm2_audmux_pads));
		}

#ifdef CONFIG_SND_SOC_SGTL5000
	platform_device_register(&sgtl5000_arm2_vdda_reg_devices);
	platform_device_register(&sgtl5000_arm2_vddio_reg_devices);
	platform_device_register(&sgtl5000_arm2_vddd_reg_devices);
#endif
	}

	return 0;
}

static int __init early_use_esai_record(char *p)
{
	esai_record = 1;
	return 0;
}

early_param("esai_record", early_use_esai_record);

static struct mxc_mlb_platform_data mx6_arm2_mlb150_data = {
	.reg_nvcc		= NULL,
	.mlb_clk		= "mlb150_clk",
	.mlb_pll_clk		= "pll6",
};

static struct mxc_dvfs_platform_data arm2_dvfscore_data = {
	.reg_id			= "cpu_vddgp",
	.soc_id			= "cpu_vddsoc",
	.pu_id			= "cpu_vddvpu",
	.clk1_id		= "cpu_clk",
	.clk2_id		= "gpc_dvfs_clk",
	.gpc_cntr_offset	= MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset	= MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset	= MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset	= MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask		= 0x1F800,
	.prediv_offset		= 11,
	.prediv_val		= 3,
	.div3ck_mask		= 0xE0000000,
	.div3ck_offset		= 29,
	.div3ck_val		= 2,
	.emac_val		= 0x08,
	.upthr_val		= 25,
	.dnthr_val		= 9,
	.pncthr_val		= 33,
	.upcnt_val		= 10,
	.dncnt_val		= 10,
	.delay_time		= 80,
};

static void __init mx6_arm2_fixup(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
}

static int __init early_enable_sgtl5000(char *p)
{
	sgtl5000_en = 1;
	return 0;
}

early_param("sgtl5000", early_enable_sgtl5000);

static int __init early_enable_spdif(char *p)
{
	spdif_en = 1;
	return 0;
}

early_param("spdif", early_enable_spdif);

static int __init early_enable_gpmi(char *p)
{
	gpmi_en = 1;
	return 0;
}

early_param("gpmi", early_enable_gpmi);

static int __init early_enable_can(char *p)
{
	flexcan_en = 1;
	return 0;
}

early_param("flexcan", early_enable_can);

static int spdif_clk_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long rate_actual;
	rate_actual = clk_round_rate(clk, rate);
	clk_set_rate(clk, rate_actual);
	return 0;
}

static struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx		= 1,		/* enable tx */
	.spdif_rx		= 1,		/* enable rx */
	/*
	 * spdif0_clk will be 454.7MHz divided by ccm dividers.
	 *
	 * 44.1KHz: 454.7MHz / 7 (ccm) / 23 (spdif) = 44,128 Hz ~ 0.06% error
	 * 48KHz:   454.7MHz / 4 (ccm) / 37 (spdif) = 48,004 Hz ~ 0.01% error
	 * 32KHz:   454.7MHz / 6 (ccm) / 37 (spdif) = 32,003 Hz ~ 0.01% error
	 */
	.spdif_clk_44100	= 1,    /* tx clk from spdif0_clk_root */
	.spdif_clk_48000	= 1,    /* tx clk from spdif0_clk_root */
	.spdif_div_44100	= 23,
	.spdif_div_48000	= 37,
	.spdif_div_32000	= 37,
	.spdif_rx_clk		= 0,    /* rx clk from spdif stream */
	.spdif_clk_set_rate	= spdif_clk_set_rate,
	.spdif_clk		= NULL, /* spdif bus clk */
};

static const struct imx_pcie_platform_data mx6_arm2_pcie_data  __initconst = {
	.pcie_pwr_en	= MX6_ARM2_PCIE_PWR_EN,
	.pcie_rst	= MX6_ARM2_PCIE_RESET,
	.pcie_wake_up	= -EINVAL,
	.pcie_dis	= -EINVAL,
};

static int __init early_disable_mipi_dsi(char *p)
{
	/*enable on board HDMI*/
	/*mulplex pin with mipi disp0_reset we should disable mipi reset*/
	disable_mipi_dsi = 1;
	return 0;
}

early_param("disable_mipi_dsi", early_disable_mipi_dsi);

/*!
 * Board specific initialization.
 */
static void __init mx6_arm2_init(void)
{
	int i;
	int ret;

	iomux_v3_cfg_t *common_pads = NULL;
	iomux_v3_cfg_t *esai_rec_pads = NULL;
	iomux_v3_cfg_t *spdif_pads = NULL;
	iomux_v3_cfg_t *flexcan_pads = NULL;
	iomux_v3_cfg_t *i2c3_pads = NULL;
	iomux_v3_cfg_t *epdc_pads = NULL;

	int common_pads_cnt;
	int esai_rec_pads_cnt;
	int spdif_pads_cnt;
	int flexcan_pads_cnt;
	int i2c3_pads_cnt;
	int epdc_pads_cnt;


	/*
	 * common pads: pads are non-shared with others on this board
	 * feature_pds: pads are shared with others on this board
	 */

	if (cpu_is_mx6q()) {
		common_pads = mx6q_arm2_pads;
		esai_rec_pads = mx6q_arm2_esai_record_pads;
		spdif_pads = mx6q_arm2_spdif_pads;
		flexcan_pads = mx6q_arm2_can_pads;
		i2c3_pads = mx6q_arm2_i2c3_pads;

		common_pads_cnt = ARRAY_SIZE(mx6q_arm2_pads);
		esai_rec_pads_cnt = ARRAY_SIZE(mx6q_arm2_esai_record_pads);
		spdif_pads_cnt =  ARRAY_SIZE(mx6q_arm2_spdif_pads);
		flexcan_pads_cnt = ARRAY_SIZE(mx6q_arm2_can_pads);
		i2c3_pads_cnt = ARRAY_SIZE(mx6q_arm2_i2c3_pads);
	} else if (cpu_is_mx6dl()) {
		common_pads = mx6dl_arm2_pads;
		esai_rec_pads = mx6dl_arm2_esai_record_pads;
		spdif_pads = mx6dl_arm2_spdif_pads;
		flexcan_pads = mx6dl_arm2_can_pads;
		i2c3_pads = mx6dl_arm2_i2c3_pads;
		epdc_pads = mx6dl_arm2_epdc_pads;

		common_pads_cnt = ARRAY_SIZE(mx6dl_arm2_pads);
		esai_rec_pads_cnt = ARRAY_SIZE(mx6dl_arm2_esai_record_pads);
		spdif_pads_cnt =  ARRAY_SIZE(mx6dl_arm2_spdif_pads);
		flexcan_pads_cnt = ARRAY_SIZE(mx6dl_arm2_can_pads);
		i2c3_pads_cnt = ARRAY_SIZE(mx6dl_arm2_i2c3_pads);
		epdc_pads_cnt = ARRAY_SIZE(mx6dl_arm2_epdc_pads);
	}

	BUG_ON(!common_pads);
	mxc_iomux_v3_setup_multiple_pads(common_pads, common_pads_cnt);

	if (esai_record) {
		BUG_ON(!esai_rec_pads);
		mxc_iomux_v3_setup_multiple_pads(esai_rec_pads,
						esai_rec_pads_cnt);
	}

	/*
	 * IEEE-1588 ts_clk, S/PDIF in and i2c3 are mutually exclusive
	 * because all of them use GPIO_16.
	 * S/PDIF out and can1 stby are mutually exclusive because both
	 * use GPIO_17.
	 */
#ifndef CONFIG_FEC_1588
	if (spdif_en) {
		BUG_ON(!spdif_pads);
		mxc_iomux_v3_setup_multiple_pads(spdif_pads, spdif_pads_cnt);
	} else {
		BUG_ON(!i2c3_pads);
		mxc_iomux_v3_setup_multiple_pads(i2c3_pads, i2c3_pads_cnt);
	}
#else
	/* Set GPIO_16 input for IEEE-1588 ts_clk and RMII reference clock
	 * For MX6 GPR1 bit21 meaning:
	 * Bit21:       0 - GPIO_16 pad output
	 *              1 - GPIO_16 pad input
	 */
	mxc_iomux_set_gpr_register(1, 21, 1, 1);
#endif

	if (!spdif_en && flexcan_en) {
		BUG_ON(!flexcan_pads);
		mxc_iomux_v3_setup_multiple_pads(flexcan_pads,
						flexcan_pads_cnt);
	}

	/*
	 * the following is the common devices support on the shared ARM2 boards
	 * Since i.MX6DQ/DL share the same memory/Register layout, we don't
	 * need to diff the i.MX6DQ or i.MX6DL here. We can simply use the
	 * mx6q_add_features() for the shared devices. For which only exist
	 * on each indivual SOC, we can use cpu_is_mx6q/6dl() to diff it.
	 */

	gp_reg_id = arm2_dvfscore_data.reg_id;
	soc_reg_id = arm2_dvfscore_data.soc_id;
	pu_reg_id = arm2_dvfscore_data.pu_id;
	mx6_arm2_init_uart();


	imx6q_add_mipi_csi2(&mipi_csi2_pdata);
	imx6q_add_mxc_hdmi_core(&hdmi_core_data);

	imx6q_add_ipuv3(0, &ipu_data[0]);
	if (cpu_is_mx6q())
		imx6q_add_ipuv3(1, &ipu_data[1]);

	if (cpu_is_mx6dl()) {
		mipi_dsi_pdata.ipu_id = 0;
		mipi_dsi_pdata.disp_id = 1;
		ldb_data.ipu_id = 0;
		ldb_data.disp_id = 0;
		for (i = 0; i < ARRAY_SIZE(sabr_fb_data) / 2; i++)
			imx6q_add_ipuv3fb(i, &sabr_fb_data[i]);
	} else {
		for (i = 0; i < ARRAY_SIZE(sabr_fb_data); i++)
			imx6q_add_ipuv3fb(i, &sabr_fb_data[i]);
	}

	if (!disable_mipi_dsi)
		imx6q_add_mipi_dsi(&mipi_dsi_pdata);
	imx6q_add_vdoa();
	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
	imx6q_add_v4l2_output(0);
	imx6q_add_v4l2_capture(0, &capture_data[0]);
	imx6q_add_v4l2_capture(1, &capture_data[1]);

	imx6q_add_imx_snvs_rtc();

	imx6q_add_imx_caam();

	imx6q_add_imx_i2c(0, &mx6_arm2_i2c_data);
	imx6q_add_imx_i2c(1, &mx6_arm2_i2c_data);
	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	if (!spdif_en) {
		if (disable_mipi_dsi)
			mx6_arm2_i2c2_data.bitrate = 100000;
		imx6q_add_imx_i2c(2, &mx6_arm2_i2c2_data);
		i2c_register_board_info(2, mxc_i2c2_board_info,
				ARRAY_SIZE(mxc_i2c2_board_info));
	}
	if (cpu_is_mx6dl())
		imx6q_add_imx_i2c(3, &mx6_arm2_i2c_data);

	/* SPI */
	imx6q_add_ecspi(0, &mx6_arm2_spi_data);
	spi_device_init();

	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &mx6_arm2_anatop_thermal_data);

	if (!esai_record) {
		imx6_init_fec(fec_data);
#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
	/* Make sure the IOMUX_OBSRV_MUX1 is set to ENET_IRQ. */
	mxc_iomux_set_specialbits_register(IOMUX_OBSRV_MUX1_OFFSET,
		OBSRV_MUX1_ENET_IRQ, OBSRV_MUX1_MASK);
#endif
	}

	imx6q_add_pm_imx(0, &mx6_arm2_pm_data);
	imx6q_add_sdhci_usdhc_imx(3, &mx6_arm2_sd4_data);
	imx6q_add_sdhci_usdhc_imx(2, &mx6_arm2_sd3_data);
	imx_add_viv_gpu(&imx6_gpu_data, &imx6_gpu_pdata);
	if (cpu_is_mx6q()) {
#ifdef CONFIG_SATA_AHCI_PLATFORM
		imx6q_add_ahci(0, &mx6_arm2_sata_data);
#else
		mx6_arm2_sata_init(NULL,
			(void __iomem *)ioremap(MX6Q_SATA_BASE_ADDR, SZ_4K));
#endif
	}
	imx6q_add_vpu();
	mx6_arm2_init_usb();
	mx6_arm2_init_audio();
	platform_device_register(&arm2_vmmc_reg_devices);

	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	/* DISP0 Reset - Assert for i2c disabled mode */
	gpio_request(MX6_ARM2_DISP0_RESET, "disp0-reset");
	gpio_direction_output(MX6_ARM2_DISP0_RESET, 0);

	/* DISP0 I2C enable */
	if (!disable_mipi_dsi) {
		gpio_request(MX6_ARM2_DISP0_I2C_EN, "disp0-i2c");
		gpio_direction_output(MX6_ARM2_DISP0_I2C_EN, 0);
	}
	gpio_request(MX6_ARM2_DISP0_PWR, "disp0-pwr");
	gpio_direction_output(MX6_ARM2_DISP0_PWR, 1);

	gpio_request(MX6_ARM2_LDB_BACKLIGHT, "ldb-backlight");
	gpio_direction_output(MX6_ARM2_LDB_BACKLIGHT, 1);
	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();
	if (gpmi_en)
		imx6q_add_gpmi(&mx6_gpmi_nand_platform_data);

	imx6q_add_dvfs_core(&arm2_dvfscore_data);

	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm_backlight(0, &mx6_arm2_pwm_backlight_data);

	if (spdif_en) {
		mxc_spdif_data.spdif_core_clk = clk_get_sys("mxc_spdif.0", NULL);
		clk_put(mxc_spdif_data.spdif_core_clk);
		imx6q_add_spdif(&mxc_spdif_data);
		imx6q_add_spdif_dai();
		imx6q_add_spdif_audio_device();
	} else if (flexcan_en) {
		ret = gpio_request_array(mx6_flexcan_gpios,
				ARRAY_SIZE(mx6_flexcan_gpios));
		if (ret) {
			pr_err("failed to request flexcan-gpios: %d\n", ret);
		} else {
			imx6q_add_flexcan0(&mx6_arm2_flexcan_pdata[0]);
			imx6q_add_flexcan1(&mx6_arm2_flexcan_pdata[1]);
		}
	}

	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);
	imx6q_add_mlb150(&mx6_arm2_mlb150_data);

	if (cpu_is_mx6dl() && epdc_enabled) {
		BUG_ON(!epdc_pads);
		mxc_iomux_v3_setup_multiple_pads(epdc_pads, epdc_pads_cnt);
		imx6dl_add_imx_pxp();
		imx6dl_add_imx_pxp_client();
		mxc_register_device(&max17135_sensor_device, NULL);
		imx6dl_add_imx_epdc(&epdc_data);
	}
	/* Add PCIe RC interface support */
	imx6q_add_pcie(&mx6_arm2_pcie_data);
	imx6q_add_busfreq();
}

extern void __iomem *twd_base;
static void __init mx6_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART4_BASE_ADDR, uart_clk);
}

static struct sys_timer mxc_timer = {
	.init   = mx6_timer_init,
};

static void __init mx6_arm2_reserve(void)
{
#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	phys_addr_t phys;

	if (imx6_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(
			imx6_gpu_pdata.reserved_mem_size, SZ_4K, SZ_2G);
		memblock_remove(phys, imx6_gpu_pdata.reserved_mem_size);
		imx6_gpu_pdata.reserved_mem_base = phys;
	}
#endif
}

MACHINE_START(MX6Q_ARM2, "Freescale i.MX 6Quad/Solo/DualLite Armadillo2 Board")
	.boot_params	= MX6_PHYS_OFFSET + 0x100,
	.fixup		= mx6_arm2_fixup,
	.map_io		= mx6_map_io,
	.init_irq	= mx6_init_irq,
	.init_machine	= mx6_arm2_init,
	.timer		= &mxc_timer,
	.reserve	= mx6_arm2_reserve,
MACHINE_END
