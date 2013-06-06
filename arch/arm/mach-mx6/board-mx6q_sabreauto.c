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
#include <linux/mtd/physmap.h>
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
#include <linux/ion.h>
#include <linux/etherdevice.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mxc_asrc.h>
#include <sound/pcm.h>
#include <linux/mfd/mxc-hdmi-core.h>

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
#include "board-mx6q_sabreauto.h"
#include "board-mx6solo_sabreauto.h"

/* sorted by GPIO_NR */
#define SABREAUTO_SD1_CD		IMX_GPIO_NR(1, 1)
#define SABREAUTO_ESAI_INT		IMX_GPIO_NR(1, 10)
#define SABREAUTO_ANDROID_HOME		IMX_GPIO_NR(1, 11)
#define SABREAUTO_ANDROID_BACK		IMX_GPIO_NR(1, 12)
#define SABREAUTO_SD3_WP		IMX_GPIO_NR(1, 13)
#define SABREAUTO_I2C_EXP_RST		IMX_GPIO_NR(1, 15)
#define SABREAUTO_USB_OTG_OC		IMX_GPIO_NR(2, 8)
#define SABREAUTO_LDB_BACKLIGHT3	IMX_GPIO_NR(2, 9)
#define SABREAUTO_LDB_BACKLIGHT4	IMX_GPIO_NR(2, 10)
#define SABREAUTO_ANDROID_POWER		IMX_GPIO_NR(2, 12)
#define SABREAUTO_ANDROID_VOLUP		IMX_GPIO_NR(2, 15)
#define SABREAUTO_CAP_TCH_INT		IMX_GPIO_NR(2, 28)
#define SABREAUTO_eCOMPASS_INT		IMX_GPIO_NR(2, 29)
#define SABREAUTO_ECSPI1_CS1		IMX_GPIO_NR(3, 19)
#define SABREAUTO_DISP0_PWR		IMX_GPIO_NR(3, 24)
#define SABREAUTO_DISP0_I2C_EN		IMX_GPIO_NR(3, 28)
#define SABREAUTO_DISP0_DET_INT		IMX_GPIO_NR(3, 31)
#define SABREAUTO_DISP0_RESET		IMX_GPIO_NR(5, 0)
#define SABREAUTO_I2C3_STEER		IMX_GPIO_NR(5, 4)
#define SABREAUTO_WEIM_NOR_WDOG1        IMX_GPIO_NR(4, 29)
#define SABREAUTO_ANDROID_VOLDOWN	IMX_GPIO_NR(5, 14)
#define SABREAUTO_PMIC_INT		IMX_GPIO_NR(5, 16)
#define SABREAUTO_ALS_INT		IMX_GPIO_NR(5, 17)
#define SABREAUTO_SD1_WP		IMX_GPIO_NR(5, 20)
#define SABREAUTO_USB_HOST1_OC		IMX_GPIO_NR(5, 0)
#define SABREAUTO_SD3_CD		IMX_GPIO_NR(6, 15)

#define SABREAUTO_MAX7310_1_BASE_ADDR	IMX_GPIO_NR(8, 0)
#define SABREAUTO_MAX7310_2_BASE_ADDR	IMX_GPIO_NR(8, 8)
#define SABREAUTO_MAX7310_3_BASE_ADDR	IMX_GPIO_NR(8, 16)

#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
#define MX6_ENET_IRQ		IMX_GPIO_NR(1, 6)
#define IOMUX_OBSRV_MUX1_OFFSET	0x3c
#define OBSRV_MUX1_MASK			0x3f
#define OBSRV_MUX1_ENET_IRQ		0x9
#endif

#define SABREAUTO_IO_EXP_GPIO1(x)	(SABREAUTO_MAX7310_1_BASE_ADDR + (x))
#define SABREAUTO_IO_EXP_GPIO2(x)	(SABREAUTO_MAX7310_2_BASE_ADDR + (x))
#define SABREAUTO_IO_EXP_GPIO3(x)	(SABREAUTO_MAX7310_3_BASE_ADDR + (x))

#define SABREAUTO_PCIE_RST_B_REVB	(SABREAUTO_MAX7310_1_BASE_ADDR + 2)
/*
 * CAN2 STBY and EN lines are the same as the CAN1. These lines are not
 * independent.
 */
#define SABREAUTO_PER_RST		SABREAUTO_IO_EXP_GPIO1(3)
#define SABREAUTO_VIDEOIN_PWR		SABREAUTO_IO_EXP_GPIO2(2)
#define SABREAUTO_CAN1_STEER		SABREAUTO_IO_EXP_GPIO2(3)
#define SABREAUTO_CAN_STBY		SABREAUTO_IO_EXP_GPIO2(5)
#define SABREAUTO_CAN_EN		SABREAUTO_IO_EXP_GPIO2(6)
#define SABREAUTO_USB_HOST1_PWR		SABREAUTO_IO_EXP_GPIO2(7)
#define SABREAUTO_USB_OTG_PWR		SABREAUTO_IO_EXP_GPIO3(1)
#define BMCR_PDOWN			0x0800 /* PHY Powerdown */

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;

static int mma8451_position = 7;
static int mag3110_position = 7;
static struct clk *sata_clk;
static int mipi_sensor;
static int can0_enable;
static int uart3_en;
static int tuner_en;
static int spinor_en;
static int weimnor_en;
static int caam_enabled;

static int __init spinor_enable(char *p)
{
       spinor_en = 1;
       return 0;
}
early_param("spi-nor", spinor_enable);

static int __init weimnor_enable(char *p)
{
       weimnor_en = 1;
       return 0;
}
early_param("weim-nor", weimnor_enable);

static int __init uart3_enable(char *p)
{
	uart3_en = 1;
	return 0;
}
early_param("uart3", uart3_enable);

static int __init tuner_enable(char *p)
{
	tuner_en = 1;
	return 0;
}
early_param("tuner", tuner_enable);

enum sd_pad_mode {
	SD_PAD_MODE_LOW_SPEED,
	SD_PAD_MODE_MED_SPEED,
	SD_PAD_MODE_HIGH_SPEED,
};

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

static struct gpio_keys_button ard_buttons[] = {
	GPIO_BUTTON(SABREAUTO_ANDROID_HOME,    KEY_HOME,       1, "home",        0),
	GPIO_BUTTON(SABREAUTO_ANDROID_BACK,    KEY_BACK,       1, "back",        0),
	GPIO_BUTTON(SABREAUTO_ANDROID_VOLUP,   KEY_VOLUMEUP,   1, "volume-up",   0),
	GPIO_BUTTON(SABREAUTO_ANDROID_VOLDOWN, KEY_VOLUMEDOWN, 1, "volume-down", 0),
	GPIO_BUTTON(SABREAUTO_ANDROID_POWER,   KEY_POWER,      1, "power-key",   1),
};

static struct gpio_keys_platform_data ard_android_button_data = {
	.buttons	= ard_buttons,
	.nbuttons	= ARRAY_SIZE(ard_buttons),
};

static struct platform_device ard_android_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &ard_android_button_data,
	}
};

static void __init imx6q_add_android_device_buttons(void)
{
	platform_device_register(&ard_android_button_device);
}
#else
static void __init imx6q_add_android_device_buttons(void) {}
#endif

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

	if (index != 2) {
		printk(KERN_ERR"no such SD host controller index %d\n", index);
		return -EINVAL;
	}

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

static const struct esdhc_platform_data mx6q_sabreauto_sd3_data __initconst = {
	.cd_gpio		= SABREAUTO_SD3_CD,
	.wp_gpio		= SABREAUTO_SD3_WP,
	.keep_power_at_suspend = 1,
	.support_18v		= 1,
	.support_8bit		= 1,
	.delay_line		= 0,
	.platform_pad_change	= plt_sd_pad_change,
	.cd_type = ESDHC_CD_CONTROLLER,
	.runtime_pm = 1,
};

static const struct esdhc_platform_data mx6q_sabreauto_sd1_data __initconst = {
	.cd_gpio = SABREAUTO_SD1_CD,
	.wp_gpio = SABREAUTO_SD1_WP,
	.keep_power_at_suspend = 1,
	.cd_type = ESDHC_CD_CONTROLLER,
	.runtime_pm = 1,
};


static int __init gpmi_nand_platform_init(void)
{
	iomux_v3_cfg_t *nand_pads = NULL;
	u32 nand_pads_cnt;

	if (cpu_is_mx6q()) {
		nand_pads = mx6q_gpmi_nand;
		nand_pads_cnt = ARRAY_SIZE(mx6q_gpmi_nand);
	} else if (cpu_is_mx6dl()) {
		nand_pads = mx6dl_gpmi_nand;
		nand_pads_cnt = ARRAY_SIZE(mx6dl_gpmi_nand);

	}
	BUG_ON(!nand_pads);
	return mxc_iomux_v3_setup_multiple_pads(nand_pads, nand_pads_cnt);
}

static const struct gpmi_nand_platform_data
mx6q_gpmi_nand_platform_data __initconst = {
	.platform_init           = gpmi_nand_platform_init,
	.min_prop_delay_in_ns    = 5,
	.max_prop_delay_in_ns    = 9,
	.max_chip_count          = 1,
};

static const struct anatop_thermal_platform_data
mx6q_sabreauto_anatop_thermal_data __initconst = {
	.name = "anatop_thermal",
};

static const struct imxuart_platform_data mx6_bt_uart_data __initconst = {
	.flags = IMXUART_HAVE_RTSCTS | IMXUART_SDMA,
	.dma_req_rx = MX6Q_DMA_REQ_UART3_RX,
	.dma_req_tx = MX6Q_DMA_REQ_UART3_TX,
};

static inline void mx6q_sabreauto_init_uart(void)
{
	imx6q_add_imx_uart(1, NULL);
	imx6q_add_imx_uart(2, &mx6_bt_uart_data);
	imx6q_add_imx_uart(3, NULL);
}

static int mx6q_sabreauto_fec_phy_init(struct phy_device *phydev)
{
	unsigned short val;

	if (!board_is_mx6_reva()) {
		/* Ar8031 phy SmartEEE feature cause link status generates
		 * glitch, which cause ethernet link down/up issue, so
		 * disable SmartEEE
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

		/* Introduce tx clock delay */
		phy_write(phydev, 0x1d, 0x5);
		val = phy_read(phydev, 0x1e);
		val |= 0x0100;
		phy_write(phydev, 0x1e, val);

		/*check phy power*/
		val = phy_read(phydev, 0x0);

		if (val & BMCR_PDOWN)
			phy_write(phydev, 0x0, (val & ~BMCR_PDOWN));
	} else {
		/* prefer master mode, 1000 Base-T capable */
		phy_write(phydev, 0x9, 0x0f00);

		/* min rx data delay */
		phy_write(phydev, 0x0b, 0x8105);
		phy_write(phydev, 0x0c, 0x0000);

		/* max rx/tx clock delay, min rx/tx control delay */
		phy_write(phydev, 0x0b, 0x8104);
		phy_write(phydev, 0x0c, 0xf0f0);
		phy_write(phydev, 0x0b, 0x104);
	}

	return 0;
}

static int mx6q_sabreauto_fec_power_hibernate(struct phy_device *phydev)
{
	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init			= mx6q_sabreauto_fec_phy_init,
	.power_hibernate	= mx6q_sabreauto_fec_power_hibernate,
	.phy			= PHY_INTERFACE_MODE_RGMII,
#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
	.gpio_irq = MX6_ENET_IRQ,
#endif
};

static int mx6q_sabreauto_spi_cs[] = {
	SABREAUTO_ECSPI1_CS1,
};

static const struct spi_imx_master mx6q_sabreauto_spi_data __initconst = {
	.chipselect     = mx6q_sabreauto_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6q_sabreauto_spi_cs),
};

static struct mtd_partition m25p32_partitions[] = {
	{
		.name	= "bootloader",
		.offset	= 0,
		.size	= SZ_256K,
	}, {
		.name	= "bootenv",
		.offset = MTDPART_OFS_APPEND,
		.size	= SZ_8K,
		.mask_flags = MTD_WRITEABLE,
	}, {
		.name	= "kernel",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct flash_platform_data m25p32_spi_flash_data = {
	.name		= "m25p32",
	.parts		= m25p32_partitions,
	.nr_parts 	= ARRAY_SIZE(m25p32_partitions),
	.type		= "m25p32",
};

static struct spi_board_info m25p32_spi0_board_info[] __initdata = {
	{
		/* The modalias must be the same as spi device driver name */
		.modalias	= "m25p80",
		.max_speed_hz	= 20000000,
		.bus_num	= 0,
		.chip_select	= 0,
		.platform_data	= &m25p32_spi_flash_data,
	},
};
static void spi_device_init(void)
{
	spi_register_board_info(m25p32_spi0_board_info,
				ARRAY_SIZE(m25p32_spi0_board_info));
}

static struct mtd_partition mxc_nor_partitions[] = {
	{
		.name	= "bootloader",
		.offset	= 0,
		.size	= SZ_256K,
	}, {
		.name = "bootenv",
		.offset = MTDPART_OFS_APPEND,
		.size = SZ_256K,
		.mask_flags = MTD_WRITEABLE,
	}, {
		.name	= "kernel",
		.offset	= MTDPART_OFS_APPEND,
		.size	= SZ_4M,
	}, {
		.name	= "rootfs",
		.offset = MTDPART_OFS_APPEND,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct resource nor_flash_resource = {
	.start		= CS0_BASE_ADDR,
	.end		= CS0_BASE_ADDR  +  0x02000000 - 1,
	.flags		= IORESOURCE_MEM,
};

static struct physmap_flash_data nor_flash_data = {
	.probe_type	= "cfi_probe",
	.width		= 2,
	.parts		= mxc_nor_partitions,
	.nr_parts	= ARRAY_SIZE(mxc_nor_partitions),
};

static struct platform_device physmap_flash_device = {
	.name	= "physmap-flash",
	.id	= 0,
	.dev	= {
		.platform_data = &nor_flash_data,
	},
	.resource	= &nor_flash_resource,
	.num_resources	= 1,
};

/* These registers settings are just valid for Numonyx M29W256GL7AN6E. */
static void mx6q_setup_weimcs(void)
{
	void __iomem *nor_reg = MX6_IO_ADDRESS(WEIM_BASE_ADDR);
	void __iomem *ccm_reg = MX6_IO_ADDRESS(CCM_BASE_ADDR);
	unsigned int reg;
	struct clk *clk;
	u32 rate;

	/* CLKCTL_CCGR6: Set emi_slow_clock to be on in all modes */
	reg = readl(ccm_reg + 0x80);
	reg |= 0x00000C00;
	writel(reg, ccm_reg + 0x80);

	/* Timing settings below based upon datasheet for M29W256GL7AN6E
	   These setting assume that the EIM_SLOW_CLOCK is set to 132 MHz */
	clk = clk_get(NULL, "emi_slow_clk");
	if (IS_ERR(clk))
		printk(KERN_ERR "emi_slow_clk not found\n");

	rate = clk_get_rate(clk);
	if (rate != 132000000)
		printk(KERN_ERR "Warning: emi_slow_clk not set to 132 MHz!"
		       " WEIM NOR timing may be incorrect!\n");

	/*
	 * For EIM General Configuration registers.
	 *
	 * CS0GCR1:
	 *	GBC = 0; CSREC = 6; DSZ = 2; BL = 0;
	 *	CREP = 1; CSEN = 1;
	 *
	 *	EIM Operation Mode: MUM = SRD = SWR = 0.
	 *		(Async write/Async page read, none multiplexed)
	 *
	 * CS0GCR2:
	 *	ADH = 1
	 */
	writel(0x00620081, nor_reg);
	writel(0x00000001, nor_reg + 0x00000004);

	/*
	 * For EIM Read Configuration registers.
	 *
	 * CS0RCR1:
	 *	RWSC = 1C;
	 *	RADVA = 0; RADVN = 2;
	 *	OEA = 2; OEN = 0;
	 *	RCSA = 0; RCSN = 0
	 *
	 * CS0RCR2:
	 *	APR = 1 (Async Page Read);
	 *	PAT = 4 (6 EIM clock sycles)
	 */
	writel(0x1C022000, nor_reg + 0x00000008);
	writel(0x0000C000, nor_reg + 0x0000000C);

	/*
	 * For EIM Write Configuration registers.
	 *
	 * CS0WCR1:
	 *	WWSC = 20;
	 *	WADVA = 0; WADVN = 1;
	 *	WBEA = 1; WBEN = 2;
	 *	WEA = 1; WEN = 6;
	 *	WCSA = 1; WCSN = 2;
	 *
	 * CS0WCR2:
	 *	WBCDD = 0
	 */
	writel(0x1404a38e, nor_reg + 0x00000010);
	writel(0x00000000, nor_reg + 0x00000014);
}

static int max7310_1_setup(struct i2c_client *client,
	unsigned gpio_base, unsigned ngpio,
	void *context)
{
	/* 0 BACKLITE_ON */
	/* 1 SAT_SHUTDN_B */
	/* 2 CPU_PER_RST_B */
	/* 3 MAIN_PER_RST_B */
	/* 4 IPOD_RST_B */
	/* 5 MLB_RST_B */
	/* 6 SSI_STEERING */
	/* 7 GPS_RST_B */

	int max7310_gpio_value[] = {
		0, 1, 1, 1, 0, 0, 1, 0,
	};

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
	.gpio_base	= SABREAUTO_MAX7310_1_BASE_ADDR,
	.invert		= 0,
	.setup		= max7310_1_setup,
};

static int max7310_u39_setup(struct i2c_client *client,
	unsigned gpio_base, unsigned ngpio,
	void *context)
{
	/* 0 not use  */
	/* 1 GPS_PWREN */
	/* 2 VIDEO_ADC_PWRDN_B */
	/* 3 ENET_CAN1_STEER */
	/* 4 EIMD30_BTUART3_STEER */
	/* 5 CAN_STBY */
	/* 6 CAN_EN */
	/* 7 USB_H1_PWR */

	int max7310_gpio_value[] = {
		0, 1, 0, 0, 0, 1, 1, 1,
	};

	int n;

	if (uart3_en)
		max7310_gpio_value[4] = 1;

	 for (n = 0; n < ARRAY_SIZE(max7310_gpio_value); ++n) {
		gpio_request(gpio_base + n, "MAX7310 U39 GPIO Expander");
		if (max7310_gpio_value[n] < 0)
			gpio_direction_input(gpio_base + n);
		else
			gpio_direction_output(gpio_base + n,
						max7310_gpio_value[n]);
		gpio_export(gpio_base + n, 0);
	}

	return 0;
}

static int max7310_u43_setup(struct i2c_client *client,
	unsigned gpio_base, unsigned ngpio,
	void *context)
{
	/*0 PORT_EXP_C0*/
	/*1 USB_OTG_PWR_ON  */
	/*2 SAT_RST_B*/
	/*3 NAND_BT_WIFI_STEER*/

	int max7310_gpio_value[] = {
		0, 0, 0, 0, 0, 0, 0, 0,
	};
	int n;

	if (uart3_en)
		max7310_gpio_value[3] = 1;

	for (n = 0; n < ARRAY_SIZE(max7310_gpio_value); ++n) {
		gpio_request(gpio_base + n, "MAX7310 U43 GPIO Expander");
		if (max7310_gpio_value[n] < 0)
			gpio_direction_input(gpio_base + n);
		else
			gpio_direction_output(gpio_base + n,
						max7310_gpio_value[n]);
		gpio_export(gpio_base + n, 0);
	}

	return 0;
}

static struct pca953x_platform_data max7310_u39_platdata = {
	.gpio_base	= SABREAUTO_MAX7310_2_BASE_ADDR,
	.invert		= 0,
	.setup		= max7310_u39_setup,
};

static struct pca953x_platform_data max7310_u43_platdata = {
	.gpio_base	= SABREAUTO_MAX7310_3_BASE_ADDR,
	.invert		= 0,
	.setup		= max7310_u43_setup,
};

static void adv7180_pwdn(int pwdn)
{
	if (pwdn)
		gpio_set_value_cansleep(SABREAUTO_VIDEOIN_PWR, 0);
	else
		gpio_set_value_cansleep(SABREAUTO_VIDEOIN_PWR, 1);
}

static void mx6q_csi0_io_init(void)
{
	if (cpu_is_mx6q())
		mxc_iomux_set_gpr_register(1, 19, 1, 1);
	else if (cpu_is_mx6dl())
		mxc_iomux_set_gpr_register(13, 0, 3, 4);
}

static struct fsl_mxc_tvin_platform_data adv7180_data = {
	.dvddio_reg	= NULL,
	.dvdd_reg	= NULL,
	.avdd_reg	= NULL,
	.pvdd_reg	= NULL,
	.pwdn		= adv7180_pwdn,
	.reset		= NULL,
	.cvbs		= true,
	.io_init	= mx6q_csi0_io_init,
};

static void mx6q_mipi_csi1_io_init(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_set_gpr_register(13, 3, 3, 1);
}

static struct fsl_mxc_tvin_platform_data adv7280_data = {
	.dvddio_reg	= NULL,
	.dvdd_reg	= NULL,
	.avdd_reg	= NULL,
	.pvdd_reg	= NULL,
	.pwdn		= NULL,
	.cvbs		= true,
	.io_init    = mx6q_mipi_csi1_io_init,
	/* csi slave reg address */
	.csi_tx_addr = 0x52,
};

static struct imxi2c_platform_data mx6q_sabreauto_i2c2_data = {
	.bitrate	= 400000,
};

static struct imxi2c_platform_data mx6q_sabreauto_i2c1_data = {
	.bitrate	= 100000,
};

static struct mxc_audio_codec_platform_data cs42888_data = {
	.rates = (
			SNDRV_PCM_RATE_48000 |
			SNDRV_PCM_RATE_96000 |
			SNDRV_PCM_RATE_192000),
};

static struct fsl_mxc_lightsensor_platform_data ls_data = {
	.rext = 499,
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("max7310", 0x30),
		.platform_data = &max7310_platdata,
	}, {
		I2C_BOARD_INFO("max7310", 0x32),
		.platform_data = &max7310_u39_platdata,
	}, {
		I2C_BOARD_INFO("max7310", 0x34),
		.platform_data = &max7310_u43_platdata,
	}, {
		I2C_BOARD_INFO("adv7180", 0x21),
		.platform_data = (void *)&adv7180_data,
	}, {
		I2C_BOARD_INFO("isl29023", 0x44),
		.irq  = gpio_to_irq(SABREAUTO_ALS_INT),
		.platform_data = &ls_data,
	},
	{
		I2C_BOARD_INFO("mag3110", 0x0e),
		.irq = gpio_to_irq(SABREAUTO_eCOMPASS_INT),
		.platform_data = (void *)&mag3110_position,
	},
	{
		I2C_BOARD_INFO("mma8451", 0x1c),
		.platform_data = (void *)&mma8451_position,
	},
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("egalax_ts", 0x04),
		.irq = gpio_to_irq(SABREAUTO_CAP_TCH_INT),
	}, {
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	}, {
		I2C_BOARD_INFO("cs42888", 0x48),
		.platform_data = (void *)&cs42888_data,
	}, {
		I2C_BOARD_INFO("si4763_i2c", 0x63),
	}, {
		I2C_BOARD_INFO("adv7280", 0x21),
		.platform_data = (void *)&adv7280_data,
	},
};

struct platform_device mxc_si4763_audio_device = {
	.name = "imx-tuner-si4763",
	.id = 0,
};

struct platform_device si4763_codec_device = {
	.name = "si4763",
	.id = 0,
};

static struct imx_ssi_platform_data mx6_sabreauto_ssi1_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};
static struct mxc_audio_platform_data si4763_audio_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 5,
};
static void imx6q_sabreauto_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value_cansleep(SABREAUTO_USB_OTG_PWR, 1);
	else
		gpio_set_value_cansleep(SABREAUTO_USB_OTG_PWR, 0);
}

static void imx6q_sabreauto_usbhost1_vbus(bool on)
{
	if (on)
		gpio_set_value_cansleep(SABREAUTO_USB_HOST1_PWR, 1);
	else
		gpio_set_value_cansleep(SABREAUTO_USB_HOST1_PWR, 0);
}

static void __init imx6q_sabreauto_init_usb(void)
{
	int ret = 0;
	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);

	ret = gpio_request(SABREAUTO_USB_OTG_OC, "otg-oc");
	if (ret) {
		printk(KERN_ERR"failed to get GPIO SABREAUTO_USB_OTG_OC:"
			" %d\n", ret);
		return;
	}
	gpio_direction_input(SABREAUTO_USB_OTG_OC);

	ret = gpio_request(SABREAUTO_USB_HOST1_OC, "usbh1-oc");
	if (ret) {
		printk(KERN_ERR"failed to get SABREAUTO_USB_HOST1_OC:"
			" %d\n", ret);
		return;
	}
	gpio_direction_input(SABREAUTO_USB_HOST1_OC);

	mxc_iomux_set_gpr_register(1, 13, 1, 0);
	mx6_set_otghost_vbus_func(imx6q_sabreauto_usbotg_vbus);
	mx6_set_host1_vbus_func(imx6q_sabreauto_usbhost1_vbus);
#ifdef CONFIG_USB_EHCI_ARC_HSIC
	mx6_usb_h2_init();
	mx6_usb_h3_init();
#endif
}

static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M + SZ_64M,
};

/* HW Initialization, if return 0, initialization is successful. */
static int mx6q_sabreauto_sata_init(struct device *dev, void __iomem *addr)
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

	return ret;
}

#ifdef CONFIG_SATA_AHCI_PLATFORM
static void mx6q_sabreauto_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);

}

static struct ahci_platform_data mx6q_sabreauto_sata_data = {
	.init = mx6q_sabreauto_sata_init,
	.exit = mx6q_sabreauto_sata_exit,
};
#endif

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits	= 4,
	.clk_map_ver	= 2,
};

static void mx6q_sabreauto_reset_mipi_dsi(void)
{
	gpio_set_value(SABREAUTO_DISP0_PWR, 1);
	gpio_set_value(SABREAUTO_DISP0_RESET, 1);
	udelay(10);
	gpio_set_value(SABREAUTO_DISP0_RESET, 0);
	udelay(50);
	gpio_set_value(SABREAUTO_DISP0_RESET, 1);

	/*
	 * it needs to delay 120ms minimum for reset complete
	 */
	msleep(120);
}

static struct mipi_dsi_platform_data mipi_dsi_pdata = {
	.ipu_id		= 0,
	.disp_id	= 0,
	.lcd_panel	= "TRULY-WVGA",
	.reset		= mx6q_sabreauto_reset_mipi_dsi,
};

static struct ipuv3_fb_platform_data sabr_fb_data[] = {
	{ /*fb0*/
		.disp_dev		= "ldb",
		.interface_pix_fmt	= IPU_PIX_FMT_RGB666,
		.mode_str		= "LDB-XGA",
		.default_bpp		= 16,
		.int_clk		= false,
	}, {
		.disp_dev		= "ldb",
		.interface_pix_fmt	= IPU_PIX_FMT_RGB666,
		.mode_str		= "LDB-XGA",
		.default_bpp		= 16,
		.int_clk		= false,
	}, {
		.disp_dev               = "lcd",
		.interface_pix_fmt      = IPU_PIX_FMT_RGB565,
		.mode_str               = "CLAA-WVGA",
		.default_bpp            = 16,
		.int_clk                = false,
	},
};

static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;
	char ipu_di_clk[] = "ipu1_di0_clk";
	struct clk *di_clk, *pll5_clk;

	if ((ipu_id > 1) || (ipu_id < 0)) {
		printk(KERN_ERR"Invalid IPU select for HDMI: %d. Set to 0\n",
			ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0)) {
		printk(KERN_ERR"Invalid DI select for HDMI: %d. Set to 0\n",
			disp_id);
		disp_id = 0;
	}

	/* Configure the connection between IPU1/2 and HDMI */
	hdmi_mux_setting = 2*ipu_id + disp_id;

	/* GPR3, bits 2-3 = HDMI_MUX_CTL */
	mxc_iomux_set_gpr_register(3, 2, 2, hdmi_mux_setting);

	/* Set HDMI event as SDMA event2 while Chip version later than TO1.2 */
	if (hdmi_SDMA_check())
		mxc_iomux_set_gpr_register(0, 0, 1, 1);

	ipu_di_clk[3] += ipu_id;
	ipu_di_clk[7] += disp_id;
	di_clk = clk_get(NULL, ipu_di_clk);
	if (IS_ERR(di_clk))
		printk(KERN_ERR "Cannot get %s clock\n", ipu_di_clk);
	pll5_clk = clk_get(NULL, "pll5");
	if (IS_ERR(pll5_clk))
		printk(KERN_ERR "Cannot get pll5 clock\n");
	clk_set_parent(di_clk, pll5_clk);
}

/* On mx6x sabreauto board i2c2 iomux with hdmi ddc,
 * the pins default work at i2c2 function,
 when hdcp enable, the pins should work at ddc function */

static void hdmi_enable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_sabreauto_hdmi_ddc_pads,
			ARRAY_SIZE(mx6dl_sabreauto_hdmi_ddc_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_sabreauto_hdmi_ddc_pads,
			ARRAY_SIZE(mx6q_sabreauto_hdmi_ddc_pads));
}

static void hdmi_disable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_sabreauto_i2c2_pads,
			ARRAY_SIZE(mx6dl_sabreauto_i2c2_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_sabreauto_i2c2_pads,
			ARRAY_SIZE(mx6q_sabreauto_i2c2_pads));
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
	.mode 		= LDB_SEP0,
	.sec_ipu_id	= 1,
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

/* Backlight PWM for CPU board lvds*/
static struct platform_pwm_backlight_data mx6_arm2_pwm_backlight_data3 = {
	.pwm_id			= 2,
	.max_brightness		= 255,
	.dft_brightness		= 128,
	.pwm_period_ns		= 50000,
};

static struct ion_platform_data imx_ion_data = {
	.nr = 1,
	.heaps = {
		{
		.type = ION_HEAP_TYPE_CARVEOUT,
		.name = "vpu_ion",
		.size = SZ_64M,
		},
	},
};

/* Backlight PWM for Main board lvds*/
static struct platform_pwm_backlight_data mx6_arm2_pwm_backlight_data4 = {
	.pwm_id			= 3,
	.max_brightness		= 255,
	.dft_brightness		= 128,
	.pwm_period_ns		= 50000,
};
static int flexcan0_en;
static int flexcan1_en;

static void mx6q_flexcan_switch(void)
{
  if (flexcan0_en || flexcan1_en) {
	gpio_set_value_cansleep(SABREAUTO_CAN_EN, 1);
	gpio_set_value_cansleep(SABREAUTO_CAN_STBY, 1);
	/* Enable STEER pin if CAN1 interface is required.
	 * STEER pin is used to switch between ENET_MDC
	 * and CAN1_TX functionality. By default ENET_MDC
	 * is active after reset.
	 */
	if (flexcan0_en)
		gpio_set_value_cansleep(SABREAUTO_CAN1_STEER, 1);

  } else {
    /* avoid to disable CAN xcvr if any of the CAN interfaces
    * are down. XCRV will be disabled only if both CAN2
    * interfaces are DOWN.
    */
    if (!flexcan0_en && !flexcan1_en) {
	gpio_set_value_cansleep(SABREAUTO_CAN_EN, 0);
	gpio_set_value_cansleep(SABREAUTO_CAN_STBY, 0);
    }
    /* turn down STEER pin only if CAN1 is DOWN */
    if (!flexcan0_en)
	gpio_set_value_cansleep(SABREAUTO_CAN1_STEER, 0);

  }
}
static void mx6q_flexcan0_switch(int enable)
{
    flexcan0_en = enable;
    mx6q_flexcan_switch();
}

static void mx6q_flexcan1_switch(int enable)
{
    flexcan1_en = enable;
    mx6q_flexcan_switch();
}

static const struct flexcan_platform_data
		mx6q_sabreauto_flexcan_pdata[] __initconst = {
	{
		.transceiver_switch = mx6q_flexcan0_switch,
	}, {
		.transceiver_switch = mx6q_flexcan1_switch,
	}
};

static struct mipi_csi2_platform_data mipi_csi2_pdata = {
	.ipu_id		= 0,
	.csi_id		= 1,
	.v_channel	= 1,
	.lanes		= 1,
	.dphy_clk	= "mipi_pllref_clk",
	.pixel_clk	= "emi_clk",
};

static void sabreauto_suspend_enter(void)
{
	/* suspend preparation */
}

static void sabreauto_suspend_exit(void)
{
	/* resmue resore */
}
static const struct pm_platform_data mx6q_sabreauto_pm_data __initconst = {
	.name		= "imx_pm",
	.suspend_enter	= sabreauto_suspend_enter,
	.suspend_exit	= sabreauto_suspend_exit,
};

static const struct asrc_p2p_params esai_p2p = {
	.p2p_rate = 48000,
	.p2p_width = ASRC_WIDTH_24_BIT,
};

static struct mxc_audio_platform_data sab_audio_data = {
	.sysclk		= 24576000,
	.codec_name	= "cs42888.1-0048",
	.priv = (void *)&esai_p2p,
};

static struct platform_device sab_audio_device = {
	.name		= "imx-cs42888",
};

static struct imx_esai_platform_data sab_esai_pdata = {
	.flags		= IMX_ESAI_NET,
};

static struct regulator_consumer_supply sabreauto_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data sabreauto_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(sabreauto_vmmc_consumers),
	.consumer_supplies = sabreauto_vmmc_consumers,
};

static struct fixed_voltage_config sabreauto_vmmc_reg_config = {
	.supply_name	= "vmmc",
	.microvolts	= 3300000,
	.gpio		= -1,
	.init_data	= &sabreauto_vmmc_init,
};

static struct platform_device sabreauto_vmmc_reg_devices = {
	.name		= "reg-fixed-voltage",
	.id		= 0,
	.dev		= {
				.platform_data = &sabreauto_vmmc_reg_config,
	},
};

static struct regulator_consumer_supply cs42888_sabreauto_consumer_va = {
	.supply		= "VA",
	.dev_name	= "1-0048",
};

static struct regulator_consumer_supply cs42888_sabreauto_consumer_vd = {
	.supply		= "VD",
	.dev_name	= "1-0048",
};

static struct regulator_consumer_supply cs42888_sabreauto_consumer_vls = {
	.supply		= "VLS",
	.dev_name	= "1-0048",
};

static struct regulator_consumer_supply cs42888_sabreauto_consumer_vlc = {
	.supply		= "VLC",
	.dev_name	= "1-0048",
};

static struct regulator_init_data cs42888_sabreauto_va_reg_initdata = {
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &cs42888_sabreauto_consumer_va,
};

static struct regulator_init_data cs42888_sabreauto_vd_reg_initdata = {
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &cs42888_sabreauto_consumer_vd,
};

static struct regulator_init_data cs42888_sabreauto_vls_reg_initdata = {
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &cs42888_sabreauto_consumer_vls,
};

static struct regulator_init_data cs42888_sabreauto_vlc_reg_initdata = {
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &cs42888_sabreauto_consumer_vlc,
};

static struct fixed_voltage_config cs42888_sabreauto_va_reg_config = {
	.supply_name		= "VA",
	.microvolts		= 2800000,
	.gpio			= -1,
	.init_data		= &cs42888_sabreauto_va_reg_initdata,
};

static struct fixed_voltage_config cs42888_sabreauto_vd_reg_config = {
	.supply_name		= "VD",
	.microvolts		= 2800000,
	.gpio			= -1,
	.init_data		= &cs42888_sabreauto_vd_reg_initdata,
};

static struct fixed_voltage_config cs42888_sabreauto_vls_reg_config = {
	.supply_name		= "VLS",
	.microvolts		= 2800000,
	.gpio			= -1,
	.init_data		= &cs42888_sabreauto_vls_reg_initdata,
};

static struct fixed_voltage_config cs42888_sabreauto_vlc_reg_config = {
	.supply_name		= "VLC",
	.microvolts		= 2800000,
	.gpio			= -1,
	.init_data		= &cs42888_sabreauto_vlc_reg_initdata,
};

static struct platform_device cs42888_sabreauto_va_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &cs42888_sabreauto_va_reg_config,
	},
};

static struct platform_device cs42888_sabreauto_vd_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 4,
	.dev	= {
		.platform_data = &cs42888_sabreauto_vd_reg_config,
	},
};

static struct platform_device cs42888_sabreauto_vls_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 5,
	.dev	= {
		.platform_data = &cs42888_sabreauto_vls_reg_config,
	},
};

static struct platform_device cs42888_sabreauto_vlc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 6,
	.dev	= {
		.platform_data = &cs42888_sabreauto_vlc_reg_config,
	},
};

static int __init imx6q_init_audio(void)
{
	struct clk *pll4_clk, *esai_clk, *anaclk_2;

	mxc_register_device(&sab_audio_device, &sab_audio_data);
	imx6q_add_imx_esai(0, &sab_esai_pdata);

	gpio_request(SABREAUTO_ESAI_INT, "esai-int");
	gpio_direction_input(SABREAUTO_ESAI_INT);

	anaclk_2 = clk_get(NULL, "anaclk_2");
	if (IS_ERR(anaclk_2))
		return PTR_ERR(anaclk_2);
	clk_set_rate(anaclk_2, 24576000);

	esai_clk = clk_get(NULL, "esai_clk");
	if (IS_ERR(esai_clk))
		return PTR_ERR(esai_clk);

	pll4_clk = clk_get(NULL, "pll4");
	if (IS_ERR(pll4_clk))
		return PTR_ERR(pll4_clk);

	clk_set_parent(pll4_clk, anaclk_2);
	clk_set_parent(esai_clk, pll4_clk);
	clk_set_rate(pll4_clk, 786432000);
	clk_set_rate(esai_clk, 24576000);

	platform_device_register(&cs42888_sabreauto_va_reg_devices);
	platform_device_register(&cs42888_sabreauto_vd_reg_devices);
	platform_device_register(&cs42888_sabreauto_vls_reg_devices);
	platform_device_register(&cs42888_sabreauto_vlc_reg_devices);
	return 0;
}

static struct mxc_mlb_platform_data mx6_sabreauto_mlb150_data = {
	.reg_nvcc		= NULL,
	.mlb_clk		= "mlb150_clk",
	.mlb_pll_clk		= "pll6",
};

static struct mxc_dvfs_platform_data sabreauto_dvfscore_data = {
	.reg_id			= "VDDCORE",
	.soc_id			= "VDDSOC",
	.clk1_id		= "cpu_clk",
	.clk2_id 		= "gpc_dvfs_clk",
	.gpc_cntr_offset 	= MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset 	= MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset 	= MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset 	= MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask 		= 0x1F800,
	.prediv_offset 		= 11,
	.prediv_val 		= 3,
	.div3ck_mask 		= 0xE0000000,
	.div3ck_offset 		= 29,
	.div3ck_val 		= 2,
	.emac_val 		= 0x08,
	.upthr_val 		= 25,
	.dnthr_val 		= 9,
	.pncthr_val 		= 33,
	.upcnt_val 		= 10,
	.dncnt_val 		= 10,
	.delay_time 		= 80,
};

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	char *str;
	struct tag *t;
	int i = 0;
	struct ipuv3_fb_platform_data *pdata_fb = sabr_fb_data;

	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "fbmem=");
			if (str != NULL) {
				str += 6;
				pdata_fb[i++].res_size[0] = memparse(str, &str);
				while (*str == ',' &&
					i < ARRAY_SIZE(sabr_fb_data)) {
					str++;
					pdata_fb[i++].res_size[0] = memparse(str, &str);
				}
			}
			/* GPU reserved memory */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpumem=");
			if (str != NULL) {
				str += 7;
				imx6q_gpu_pdata.reserved_mem_size = memparse(str, &str);
			}
			break;
		}
	}
}

static int __init caam_setup(char *__unused)
{
	caam_enabled = 1;
	return 1;
}
early_param("caam", caam_setup);

static int __init early_enable_mipi_sensor(char *p)
{
	mipi_sensor = 1;
	return 0;
}
early_param("mipi_sensor", early_enable_mipi_sensor);

static int __init early_enable_can0(char *p)
{
	can0_enable = 1;
	return 0;
}
early_param("can0", early_enable_can0);

static struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx	= 0,	/* disable tx */
	.spdif_rx	= 1,	/* enable rx */
	.spdif_rx_clk	= 0,	/* rx clk from spdif stream */
	.spdif_clk	= NULL,	/* spdif bus clk */
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

static const struct imx_pcie_platform_data mx6_sabreauto_pcie_data __initconst = {
	.pcie_pwr_en	= -EINVAL,
	.pcie_rst	= SABREAUTO_PCIE_RST_B_REVB,
	.pcie_wake_up	= -EINVAL,
	.pcie_dis	= -EINVAL,
};

/*!
 * Board specific initialization.
 */
static void __init mx6_board_init(void)
{
	int i;
	int ret;
	iomux_v3_cfg_t *common_pads = NULL;
	iomux_v3_cfg_t *can0_pads = NULL;
	iomux_v3_cfg_t *can1_pads = NULL;
	iomux_v3_cfg_t *mipi_sensor_pads = NULL;
	iomux_v3_cfg_t *i2c3_pads = NULL;
	iomux_v3_cfg_t *tuner_pads = NULL;
	iomux_v3_cfg_t *spinor_pads = NULL;
	iomux_v3_cfg_t *weimnor_pads = NULL;
	iomux_v3_cfg_t *bluetooth_pads = NULL;
	iomux_v3_cfg_t *extra_pads = NULL;

	int common_pads_cnt;
	int can0_pads_cnt;
	int can1_pads_cnt;
	int mipi_sensor_pads_cnt;
	int i2c3_pads_cnt;
	int tuner_pads_cnt;
	int spinor_pads_cnt;
	int weimnor_pads_cnt;
	int bluetooth_pads_cnt;
	int extra_pads_cnt;

	if (cpu_is_mx6q()) {
		common_pads = mx6q_sabreauto_pads;
		can0_pads = mx6q_sabreauto_can0_pads;
		can1_pads = mx6q_sabreauto_can1_pads;
		mipi_sensor_pads = mx6q_sabreauto_mipi_sensor_pads;
		tuner_pads = mx6q_tuner_pads;
		spinor_pads = mx6q_spinor_pads;
		weimnor_pads = mx6q_weimnor_pads;
		bluetooth_pads = mx6q_bluetooth_pads;

		common_pads_cnt = ARRAY_SIZE(mx6q_sabreauto_pads);
		can0_pads_cnt = ARRAY_SIZE(mx6q_sabreauto_can0_pads);
		can1_pads_cnt = ARRAY_SIZE(mx6q_sabreauto_can1_pads);
		mipi_sensor_pads_cnt = ARRAY_SIZE(mx6q_sabreauto_mipi_sensor_pads);
		tuner_pads_cnt = ARRAY_SIZE(mx6q_tuner_pads);
		spinor_pads_cnt = ARRAY_SIZE(mx6q_spinor_pads);
		weimnor_pads_cnt = ARRAY_SIZE(mx6q_weimnor_pads);
		bluetooth_pads_cnt = ARRAY_SIZE(mx6q_bluetooth_pads);
		if (board_is_mx6_reva()) {
			i2c3_pads = mx6q_i2c3_pads_rev_a;
			i2c3_pads_cnt = ARRAY_SIZE(mx6q_i2c3_pads_rev_a);
			mxc_iomux_v3_setup_multiple_pads(i2c3_pads,
					i2c3_pads_cnt);
		} else {
			i2c3_pads = mx6q_i2c3_pads_rev_b;
			i2c3_pads_cnt = ARRAY_SIZE(mx6q_i2c3_pads_rev_b);
			extra_pads = mx6q_extra_pads_rev_b;
			extra_pads_cnt = ARRAY_SIZE(mx6q_extra_pads_rev_b);
			mxc_iomux_v3_setup_multiple_pads(extra_pads,
					extra_pads_cnt);
		}
	} else if (cpu_is_mx6dl()) {
		common_pads = mx6dl_sabreauto_pads;
		can0_pads = mx6dl_sabreauto_can0_pads;
		can1_pads = mx6dl_sabreauto_can1_pads;
		mipi_sensor_pads = mx6dl_sabreauto_mipi_sensor_pads;
		tuner_pads = mx6dl_tuner_pads;
		spinor_pads = mx6dl_spinor_pads;
		weimnor_pads = mx6dl_weimnor_pads;
		bluetooth_pads = mx6dl_bluetooth_pads;

		common_pads_cnt = ARRAY_SIZE(mx6dl_sabreauto_pads);
		can0_pads_cnt = ARRAY_SIZE(mx6dl_sabreauto_can0_pads);
		can1_pads_cnt = ARRAY_SIZE(mx6dl_sabreauto_can1_pads);
		mipi_sensor_pads_cnt = ARRAY_SIZE(mx6dl_sabreauto_mipi_sensor_pads);
		tuner_pads_cnt = ARRAY_SIZE(mx6dl_tuner_pads);
		spinor_pads_cnt = ARRAY_SIZE(mx6dl_spinor_pads);
		weimnor_pads_cnt = ARRAY_SIZE(mx6dl_weimnor_pads);
		bluetooth_pads_cnt = ARRAY_SIZE(mx6dl_bluetooth_pads);

		if (board_is_mx6_reva()) {
			i2c3_pads = mx6dl_i2c3_pads_rev_a;
			i2c3_pads_cnt = ARRAY_SIZE(mx6dl_i2c3_pads_rev_a);
			mxc_iomux_v3_setup_multiple_pads(i2c3_pads,
					i2c3_pads_cnt);
		} else {
			i2c3_pads = mx6dl_i2c3_pads_rev_b;
			i2c3_pads_cnt = ARRAY_SIZE(mx6dl_i2c3_pads_rev_b);
			extra_pads = mx6dl_extra_pads_rev_b;
			extra_pads_cnt = ARRAY_SIZE(mx6dl_extra_pads_rev_b);
			mxc_iomux_v3_setup_multiple_pads(extra_pads,
					extra_pads_cnt);
		}
	}

	BUG_ON(!common_pads);
	mxc_iomux_v3_setup_multiple_pads(common_pads, common_pads_cnt);

	/*If at least one NOR memory is selected we don't
	 * configure IC23 PADS for rev B */
	if (spinor_en) {
		BUG_ON(!spinor_pads);
		mxc_iomux_v3_setup_multiple_pads(spinor_pads, spinor_pads_cnt);
	} else if (weimnor_en) {
		BUG_ON(!weimnor_pads);
		mxc_iomux_v3_setup_multiple_pads(weimnor_pads,
						weimnor_pads_cnt);
	} else {
		if (!board_is_mx6_reva()) {
			BUG_ON(!i2c3_pads);
			mxc_iomux_v3_setup_multiple_pads(i2c3_pads,
					i2c3_pads_cnt);
		}
		if (uart3_en) {
			BUG_ON(!bluetooth_pads);
			mxc_iomux_v3_setup_multiple_pads(bluetooth_pads,
					bluetooth_pads_cnt);
		}
	}

	if (can0_enable) {
		BUG_ON(!can0_pads);
		mxc_iomux_v3_setup_multiple_pads(can0_pads,
						can0_pads_cnt);
	}

	BUG_ON(!can1_pads);
	mxc_iomux_v3_setup_multiple_pads(can1_pads, can1_pads_cnt);

	if (tuner_en) {
		BUG_ON(!tuner_pads);
		mxc_iomux_v3_setup_multiple_pads(tuner_pads,
			tuner_pads_cnt);
	}

	/* assert i2c-rst  */
	gpio_request(SABREAUTO_I2C_EXP_RST, "i2c-rst");
	gpio_direction_output(SABREAUTO_I2C_EXP_RST, 1);

	if (!board_is_mx6_reva()) {
		/* enable either EIM_D18 or i2c3_sda route path */
		gpio_request(SABREAUTO_I2C3_STEER, "i2c3-steer");
		if (spinor_en)
			gpio_direction_output(SABREAUTO_I2C3_STEER, 0);
		else if (weimnor_en) {
			/*Put DISP0_DAT8 in ALT5 mode to prevent WDOG1 of
			resetting WEIM NOR*/
			gpio_direction_output(SABREAUTO_I2C3_STEER, 0);

			gpio_request(SABREAUTO_WEIM_NOR_WDOG1, "nor-reset");
			gpio_direction_output(SABREAUTO_WEIM_NOR_WDOG1, 1);
		} else
			gpio_direction_output(SABREAUTO_I2C3_STEER, 1);
		/* Set GPIO_16 input for IEEE-1588 ts_clk and
		 * RMII reference clk
		 * For MX6 GPR1 bit21 meaning:
		 * Bit21:   0 - GPIO_16 pad output
		 *          1 - GPIO_16 pad input
		 */
		mxc_iomux_set_gpr_register(1, 21, 1, 1);
	}

	if (mipi_sensor) {
		BUG_ON(!mipi_sensor_pads);
		mxc_iomux_v3_setup_multiple_pads(mipi_sensor_pads,
						mipi_sensor_pads_cnt);
	}

	gp_reg_id = sabreauto_dvfscore_data.reg_id;
	soc_reg_id = sabreauto_dvfscore_data.soc_id;
	mx6q_sabreauto_init_uart();
	imx6q_add_mipi_csi2(&mipi_csi2_pdata);
	if (cpu_is_mx6dl()) {
		mipi_dsi_pdata.ipu_id = 0;
		mipi_dsi_pdata.disp_id = 1;
		ldb_data.ipu_id = 0;
		ldb_data.disp_id = 0;
		ldb_data.sec_ipu_id = 0;
		ldb_data.sec_disp_id = 1;
		hdmi_core_data.disp_id = 1;
	}
	imx6q_add_mxc_hdmi_core(&hdmi_core_data);

	imx6q_add_ipuv3(0, &ipu_data[0]);
	if (cpu_is_mx6q()) {
		imx6q_add_ipuv3(1, &ipu_data[1]);
		for (i = 0; i < ARRAY_SIZE(sabr_fb_data); i++)
			imx6q_add_ipuv3fb(i, &sabr_fb_data[i]);
	} else if (cpu_is_mx6dl())
		for (i = 0; i < (ARRAY_SIZE(sabr_fb_data) + 1) / 2; i++)
			imx6q_add_ipuv3fb(i, &sabr_fb_data[i]);

	imx6q_add_vdoa();
	imx6q_add_mipi_dsi(&mipi_dsi_pdata);
	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
	imx6q_add_v4l2_output(0);
	imx6q_add_v4l2_capture(0, &capture_data[0]);
	imx6q_add_v4l2_capture(1, &capture_data[1]);
	imx6q_add_android_device_buttons();

	imx6q_add_imx_snvs_rtc();

	if (1 == caam_enabled)
		imx6q_add_imx_caam();

	imx6q_add_imx_i2c(1, &mx6q_sabreauto_i2c1_data);
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	imx6q_add_imx_i2c(2, &mx6q_sabreauto_i2c2_data);
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));

	ret = gpio_request(SABREAUTO_PMIC_INT, "pFUZE-int");
	if (ret) {
		printk(KERN_ERR"request pFUZE-int error!!\n");
		return;
	} else {
		gpio_direction_input(SABREAUTO_PMIC_INT);
		mx6q_sabreauto_init_pfuze100(SABREAUTO_PMIC_INT);
	}
	/* SPI */
	imx6q_add_ecspi(0, &mx6q_sabreauto_spi_data);
		if (spinor_en)
			spi_device_init();
		else if (weimnor_en) {
			mx6q_setup_weimcs();
			platform_device_register(&physmap_flash_device);
		}
	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &mx6q_sabreauto_anatop_thermal_data);

	if (!can0_enable) {
		imx6_init_fec(fec_data);
#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
		/* Make sure the IOMUX_OBSRV_MUX1 is set to ENET_IRQ. */
		mxc_iomux_set_specialbits_register(IOMUX_OBSRV_MUX1_OFFSET,
			OBSRV_MUX1_ENET_IRQ, OBSRV_MUX1_MASK);
#endif

	}
	imx6q_add_pm_imx(0, &mx6q_sabreauto_pm_data);

	imx6q_add_sdhci_usdhc_imx(2, &mx6q_sabreauto_sd3_data);
	imx6q_add_sdhci_usdhc_imx(0, &mx6q_sabreauto_sd1_data);

	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	imx6q_sabreauto_init_usb();
	if (cpu_is_mx6q()) {
#ifdef CONFIG_SATA_AHCI_PLATFORM
		imx6q_add_ahci(0, &mx6q_sabreauto_sata_data);
#else
		mx6q_sabreauto_sata_init(NULL,
			(void __iomem *)ioremap(MX6Q_SATA_BASE_ADDR, SZ_4K));
#endif
	}
	imx6q_add_vpu();
	imx6q_init_audio();
	platform_device_register(&sabreauto_vmmc_reg_devices);
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	/* DISP0 Detect */
	gpio_request(SABREAUTO_DISP0_DET_INT, "disp0-detect");
	gpio_direction_input(SABREAUTO_DISP0_DET_INT);

	/* DISP0 Reset - Assert for i2c disabled mode */
	gpio_request(SABREAUTO_DISP0_RESET, "disp0-reset");
	gpio_direction_output(SABREAUTO_DISP0_RESET, 0);

	/* DISP0 I2C enable */
	gpio_request(SABREAUTO_DISP0_I2C_EN, "disp0-i2c");
	gpio_direction_output(SABREAUTO_DISP0_I2C_EN, 0);

	gpio_request(SABREAUTO_DISP0_PWR, "disp0-pwr");
	gpio_direction_output(SABREAUTO_DISP0_PWR, 1);

	gpio_request(SABREAUTO_LDB_BACKLIGHT3, "ldb-backlight3");
	gpio_direction_output(SABREAUTO_LDB_BACKLIGHT3, 1);
	gpio_request(SABREAUTO_LDB_BACKLIGHT4, "ldb-backlight4");
	gpio_direction_output(SABREAUTO_LDB_BACKLIGHT4, 1);
	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();
	if (!uart3_en && !weimnor_en)
		imx6q_add_gpmi(&mx6q_gpmi_nand_platform_data);

	imx6q_add_dvfs_core(&sabreauto_dvfscore_data);

	imx6q_add_ion(0, &imx_ion_data,
		sizeof(imx_ion_data) + sizeof(struct ion_platform_heap));
	imx6q_add_mxc_pwm(2);
	imx6q_add_mxc_pwm(3);
	imx6q_add_mxc_pwm_backlight(2, &mx6_arm2_pwm_backlight_data3);
	imx6q_add_mxc_pwm_backlight(3, &mx6_arm2_pwm_backlight_data4);

	mxc_spdif_data.spdif_core_clk = clk_get_sys("mxc_spdif.0", NULL);
	clk_put(mxc_spdif_data.spdif_core_clk);
	imx6q_add_spdif(&mxc_spdif_data);
	imx6q_add_spdif_dai();
	imx6q_add_spdif_audio_device();

	if (can0_enable)
		imx6q_add_flexcan0(&mx6q_sabreauto_flexcan_pdata[0]);
	imx6q_add_flexcan1(&mx6q_sabreauto_flexcan_pdata[1]);
	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();
	imx6q_add_mlb150(&mx6_sabreauto_mlb150_data);

	/* Tuner audio interface */
	imx6q_add_imx_ssi(1, &mx6_sabreauto_ssi1_pdata);
	mxc_register_device(&si4763_codec_device, NULL);
	mxc_register_device(&mxc_si4763_audio_device, &si4763_audio_data);

	imx6q_add_busfreq();

	/* Add PCIe RC interface support */
	imx6q_add_pcie(&mx6_sabreauto_pcie_data);

	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);

	/* acquire this irq is want to let touch driver failed to
	 * request this irq gpio, and the driver will disable suspend
	 * funtion, to workaround sabreauto board have isssue on gpio
	 * of touch use. */
	gpio_request(SABREAUTO_CAP_TCH_INT, "touch irq");
	gpio_direction_input(SABREAUTO_CAP_TCH_INT);

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
	.init = mx6_timer_init,
};

static void __init mx6q_reserve(void)
{
	phys_addr_t phys;
	int i, fb0_reserved = 0, fb_array_size;

	/*
	 * Reserve primary framebuffer memory if its base address
	 * is set by kernel command line.
	 */
	fb_array_size = ARRAY_SIZE(sabr_fb_data);
	if (fb_array_size > 0 && sabr_fb_data[0].res_base[0] &&
	    sabr_fb_data[0].res_size[0]) {
		if (sabr_fb_data[0].res_base[0] > SZ_2G)
			printk(KERN_INFO"UI Performance downgrade with FB phys address %x!\n",
			    sabr_fb_data[0].res_base[0]);
		memblock_reserve(sabr_fb_data[0].res_base[0],
				 sabr_fb_data[0].res_size[0]);
		memblock_remove(sabr_fb_data[0].res_base[0],
				sabr_fb_data[0].res_size[0]);
		sabr_fb_data[0].late_init = true;
		ipu_data[ldb_data.ipu_id].bypass_reset = true;
		fb0_reserved = 1;
	}
	for (i = fb0_reserved; i < fb_array_size; i++)
		if (sabr_fb_data[i].res_size[0]) {
			/* Reserve for other background buffer. */
			phys = memblock_alloc_base(sabr_fb_data[i].res_size[0],
						SZ_4K, SZ_2G);
			memblock_remove(phys, sabr_fb_data[i].res_size[0]);
			sabr_fb_data[i].res_base[0] = phys;
		}

#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
			SZ_4K, SZ_2G);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif

#if defined(CONFIG_ION)
	if (imx_ion_data.heaps[0].size) {
		phys = memblock_alloc(imx_ion_data.heaps[0].size, SZ_4K);
		memblock_free(phys, imx_ion_data.heaps[0].size);
		memblock_remove(phys, imx_ion_data.heaps[0].size);
		imx_ion_data.heaps[0].base = phys;
	}
#endif
}

MACHINE_START(MX6Q_SABREAUTO, "Freescale i.MX 6Quad/DualLite/Solo Sabre Auto Board")
	.boot_params	= MX6_PHYS_OFFSET + 0x100,
	.fixup		= fixup_mxc_board,
	.map_io		= mx6_map_io,
	.init_irq	= mx6_init_irq,
	.init_machine	= mx6_board_init,
	.timer		= &mxc_timer,
	.reserve	= mx6q_reserve,
MACHINE_END
