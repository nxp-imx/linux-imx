/*
 *  Copyright (C) 2000 Deep Blue Solutions Ltd
 *  Copyright (C) 2002 Shane Nay (shane@minirl.com)
 *  Copyright 2005-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/serial_8250.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/ata.h>
#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <asm/mach/flash.h>
#endif

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/irq.h>
#include <asm/mach/keypad.h>
#include <asm/mach/time.h>
#include <mach/common.h>
#include <mach/memory.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/spba.h>

#include "board-mx31ads.h"
#include "crm_regs.h"
#include "iomux.h"
/*!
 * @file mach-mx3/mx31ads.c
 *
 * @brief This file contains the board-specific initialization routines.
 *
 * @ingroup MSL_MX31
 */

extern void mx31ads_gpio_init(void) __init;

static void mxc_nop_release(struct device *dev)
{
	/* Nothing */
}

#if defined(CONFIG_CS89x0) || defined(CONFIG_CS89x0_MODULE)
/*! Null terminated portlist used to probe for the CS8900A device on ISA Bus
 * Add 3 to reset the page window before probing (fixes eth probe when deployed
 * using nand_boot)
 */
unsigned int netcard_portlist[] = { CS8900A_BASE_ADDRESS + 3, 0 };

EXPORT_SYMBOL(netcard_portlist);
/*!
 * The CS8900A has 4 IRQ pins, which is software selectable, CS8900A interrupt
 * pin 0 is used for interrupt generation.
 */
unsigned int cs8900_irq_map[] = { CS8900AIRQ, 0, 0, 0 };

EXPORT_SYMBOL(cs8900_irq_map);
#endif

#if defined(CONFIG_KEYBOARD_MXC) || defined(CONFIG_KEYBOARD_MXC_MODULE)

/* Keypad keycodes for the EVB 8x8
 * keypad.  POWER and PTT keys don't generate
 * any interrupts via this driver so they are
 * not support. Change any keys as u like!
 */
static u16 keymapping[64] = {
	KEY_SELECT, KEY_LEFT, KEY_DOWN, KEY_RIGHT,
	KEY_UP, KEY_F12, KEY_END, KEY_BACK,
	KEY_F1, KEY_SENDFILE, KEY_HOME, KEY_F6,
	KEY_VOLUMEUP, KEY_F8, KEY_F9, KEY_F10,
	KEY_3, KEY_2, KEY_1, KEY_4,
	KEY_VOLUMEDOWN, KEY_7, KEY_5, KEY_6,
	KEY_9, KEY_LEFTSHIFT, KEY_8, KEY_0,
	KEY_KPASTERISK, KEY_RECORD, KEY_Q, KEY_W,
	KEY_A, KEY_S, KEY_D, KEY_E,
	KEY_F, KEY_R, KEY_T, KEY_Y,
	KEY_TAB, KEY_F7, KEY_CAPSLOCK, KEY_Z,
	KEY_X, KEY_C, KEY_V, KEY_G,
	KEY_B, KEY_H, KEY_N, KEY_M,
	KEY_J, KEY_K, KEY_U, KEY_I,
	KEY_SPACE, KEY_F2, KEY_DOT, KEY_ENTER,
	KEY_L, KEY_BACKSPACE, KEY_P, KEY_O,
};

static struct resource mxc_kpp_resources[] = {
	[0] = {
	       .start = MXC_INT_KPP,
	       .end = MXC_INT_KPP,
	       .flags = IORESOURCE_IRQ,
	       }
};

static struct keypad_data evb_8_by_8_keypad = {
	.rowmax = 8,
	.colmax = 8,
	.irq = MXC_INT_KPP,
	.learning = 0,
	.delay = 2,
	.matrix = keymapping,
};

/* mxc keypad driver */
static struct platform_device mxc_keypad_device = {
	.name = "mxc_keypad",
	.id = 0,
	.num_resources = ARRAY_SIZE(mxc_kpp_resources),
	.resource = mxc_kpp_resources,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &evb_8_by_8_keypad,
		},
};

static void mxc_init_keypad(void)
{
	(void)platform_device_register(&mxc_keypad_device);
}
#else
static inline void mxc_init_keypad(void)
{
}
#endif

#if defined(CONFIG_SERIAL_8250) || defined(CONFIG_SERIAL_8250_MODULE)
/*!
 * The serial port definition structure.
 */
static struct plat_serial8250_port serial_platform_data[] = {
	{
	 .membase = (void *)(PBC_BASE_ADDRESS + PBC_SC16C652_UARTA),
	 .mapbase = (unsigned long)(CS4_BASE_ADDR + PBC_SC16C652_UARTA),
	 .irq = EXPIO_INT_XUART_INTA,
	 .uartclk = 14745600,
	 .regshift = 0,
	 .iotype = UPIO_MEM,
	 .flags = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST | UPF_AUTO_IRQ,},
	{
	 .membase = (void *)(PBC_BASE_ADDRESS + PBC_SC16C652_UARTB),
	 .mapbase = (unsigned long)(CS4_BASE_ADDR + PBC_SC16C652_UARTB),
	 .irq = EXPIO_INT_XUART_INTB,
	 .uartclk = 14745600,
	 .regshift = 0,
	 .iotype = UPIO_MEM,
	 .flags = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST | UPF_AUTO_IRQ,},
	{},
};

static struct platform_device serial_device = {
	.name = "serial8250",
	.id = 0,
	.dev = {
		.platform_data = serial_platform_data,
		},
};

static int __init mxc_init_extuart(void)
{
	return platform_device_register(&serial_device);
}
#else
static inline int mxc_init_extuart(void)
{
	return 0;
}
#endif
/* MTD NOR flash */

#if defined(CONFIG_MTD_MXC) || defined(CONFIG_MTD_MXC_MODULE)

static struct mtd_partition mxc_nor_partitions[] = {
	{
	 .name = "Bootloader",
	 .size = 512 * 1024,
	 .offset = 0x00000000,
	 .mask_flags = MTD_WRITEABLE	/* force read-only */
	 },
	{
	 .name = "nor.Kernel",
	 .size = 2 * 1024 * 1024,
	 .offset = MTDPART_OFS_APPEND,
	 .mask_flags = 0},
	{
	 .name = "nor.userfs",
	 .size = 14 * 1024 * 1024,
	 .offset = MTDPART_OFS_APPEND,
	 .mask_flags = 0},
	{
	 .name = "nor.rootfs",
	 .size = 12 * 1024 * 1024,
	 .offset = MTDPART_OFS_APPEND,
	 .mask_flags = MTD_WRITEABLE},
	{
	 .name = "FIS directory",
	 .size = 12 * 1024,
	 .offset = 0x01FE0000,
	 .mask_flags = MTD_WRITEABLE	/* force read-only */
	 },
	{
	 .name = "Redboot config",
	 .size = MTDPART_SIZ_FULL,
	 .offset = 0x01FFF000,
	 .mask_flags = MTD_WRITEABLE	/* force read-only */
	 },
};

static struct flash_platform_data mxc_flash_data = {
	.map_name = "cfi_probe",
	.width = 2,
	.parts = mxc_nor_partitions,
	.nr_parts = ARRAY_SIZE(mxc_nor_partitions),
};

static struct resource mxc_flash_resource = {
	.start = 0xa0000000,
	.end = 0xa0000000 + 0x02000000 - 1,
	.flags = IORESOURCE_MEM,

};

static struct platform_device mxc_nor_mtd_device = {
	.name = "mxc_nor_flash",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_flash_data,
		},
	.num_resources = 1,
	.resource = &mxc_flash_resource,
};

static void mxc_init_nor_mtd(void)
{
	(void)platform_device_register(&mxc_nor_mtd_device);
}
#else
static void mxc_init_nor_mtd(void)
{
}
#endif

/* NAND Flash Partitions */
#ifdef CONFIG_MTD_PARTITIONS

static struct mtd_partition nand_flash_partitions[4] = {
	{
	 .name = "nand.bootloader",
	 .offset = 0,
	 .size = 1024 * 1024},
	{
	 .name = "nand.kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 5 * 1024 * 1024},
	{
	 .name = "nand.rootfs",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 22 * 1024 * 1024},
	{
	 .name = "nand.userfs",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL},
};

#endif

/* MTD NAND flash */

#if defined(CONFIG_MTD_NAND_MXC) \
	|| defined(CONFIG_MTD_NAND_MXC_MODULE) \
	|| defined(CONFIG_MTD_NAND_MXC_V2) \
	|| defined(CONFIG_MTD_NAND_MXC_V2_MODULE)

static struct flash_platform_data mxc_nand_data = {
	#ifdef CONFIG_MTD_PARTITIONS
		.parts = nand_flash_partitions,
		.nr_parts = ARRAY_SIZE(nand_flash_partitions),
	#endif
	.width = 1,
};

static struct platform_device mxc_nand_mtd_device = {
	.name = "mxc_nand_flash",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_nand_data,
		},
};

static struct platform_device mxc_nandv2_mtd_device = {
	.name = "mxc_nandv2_flash",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_nand_data,
		},
};

static void mxc_init_nand_mtd(void)
{
	if (__raw_readl(MXC_CCM_RCSR) & MXC_CCM_RCSR_NF16B) {
		mxc_nand_data.width = 2;
	}
	if (cpu_is_mx31()) {
		(void)platform_device_register(&mxc_nand_mtd_device);
	}
	if (cpu_is_mx32()) {
		(void)platform_device_register(&mxc_nandv2_mtd_device);
	}
}
#else
static inline void mxc_init_nand_mtd(void)
{
}
#endif

/* i.MX MTD NAND Flash Controller */

#if defined(CONFIG_MTD_NAND_IMX_NFC) || defined(CONFIG_MTD_NAND_IMX_NFC_MODULE)

/* Resources for this device. */

static struct resource imx_nfc_resources[] = {
	{
	 .flags = IORESOURCE_MEM,
	 .start = NFC_BASE_ADDR + 0x000,
	 .end   = NFC_BASE_ADDR + 0x840 - 1,
	 .name  = IMX_NFC_BUFFERS_ADDR_RES_NAME,
	 },
	{
	 .flags = IORESOURCE_MEM,
	 .start = NFC_BASE_ADDR + 0xE00,
	 .end   = NFC_BASE_ADDR + 0xE20 - 1,
	 .name  = IMX_NFC_PRIMARY_REGS_ADDR_RES_NAME,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = MXC_INT_NANDFC,
	 .end   = MXC_INT_NANDFC,
	 .name  = IMX_NFC_INTERRUPT_RES_NAME,
	 },
};

/**
 * imx_nfc_set_page_size() - Tells the hardware the page size.
 *
 * @data_size_in_bytes:  The page size in bytes (e.g., 512, 2048, etc.). This
 *                       size refers specifically to the the data bytes in the
 *                       page, *not* including out-of-band bytes. The return
 *                       value is zero if the operation succeeded. Do not
 *                       interpret a non-zero value as an error code - it only
 *                       indicates failure. The driver will decide what error
 *                       code to return to its caller.
 */
static int imx_nfc_set_page_size(unsigned int data_size_in_bytes)
{

	unsigned long x = __raw_readl(MXC_CCM_RCSR);

	switch (data_size_in_bytes) {

	case 512:
		x &= ~MXC_CCM_RCSR_NFMS;
		break;

	case 2048:
		x |= MXC_CCM_RCSR_NFMS;
		break;

	default:
		return !0;
		break;

	}

	__raw_writel(x, MXC_CCM_RCSR);

	return 0;

}

/*
 * Platform-specific information about this device. Some of the details depend
 * on the SoC. See imx_init_nfc() below for code that fills in the rest.
 */

static struct imx_nfc_platform_data imx_nfc_platform_data = {
	.force_ce           = false,
	.target_cycle_in_ns = 50,
	.clock_name         = "nfc_clk",
	.set_page_size      = imx_nfc_set_page_size,
	.interleave         = false,
	#ifdef CONFIG_MTD_PARTITIONS
		.partitions         = nand_flash_partitions,
		.partition_count    = ARRAY_SIZE(nand_flash_partitions),
	#endif
};

/* The structure that represents the NFC device. */

static struct platform_device imx_nfc_device = {
	.name = IMX_NFC_DRIVER_NAME,
	.id = 0,
	.dev = {
		.release       = mxc_nop_release,
		.platform_data = &imx_nfc_platform_data,
	 },
	.resource      = imx_nfc_resources,
	.num_resources = ARRAY_SIZE(imx_nfc_resources),
};

/**
 * imx_init_nfc() - Sets up the NFC for this platform.
 *
 * This function sets up data structures representing the NFC device on this
 * platform and registers the device with the platform management system.
 */

static void imx_nfc_init(void)
{

	/*
	 * A field in the Reset Control and Source Register register tells us
	 * the bus width.
	 */

	if (__raw_readl(MXC_CCM_RCSR) & MXC_CCM_RCSR_NF16B)
		imx_nfc_platform_data.bus_width_in_bits = 16;
	else
		imx_nfc_platform_data.bus_width_in_bits = 8;

	/*
	 * Discover the type of SoC we're running on and, based on that, fill in
	 * some details about the NFC.
	 */

	if (cpu_is_mx31()) {
		imx_nfc_platform_data.major_version = 1;
		imx_nfc_platform_data.minor_version = 0;
	} else if (cpu_is_mx32()) {
		imx_nfc_platform_data.major_version = 2;
		imx_nfc_platform_data.minor_version = 0;
	} else {
		pr_err("imx_nfc: Can't identify the SoC\n");
		BUG();
	}

	/* Register the NFC device. */

	(void)platform_device_register(&imx_nfc_device);

}

#else

static inline void imx_nfc_init(void)
{
}

#endif /* i.MX MTD NAND Flash Controller */

static struct spi_board_info mxc_spi_board_info[] __initdata = {
	{
	 .modalias = "pmic_spi",
	 .irq = IOMUX_TO_IRQ(MX31_PIN_GPIO1_3),
	 .max_speed_hz = 4000000,
	 .bus_num = 2,
	 .chip_select = 0,
	 },
};

#if defined(CONFIG_FB_MXC_SYNC_PANEL) || defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
static const char fb_default_mode[] = "Sharp-QVGA";

/* mxc lcd driver */
static struct platform_device mxc_fb_device = {
	.name = "mxc_sdc_fb",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &fb_default_mode,
		.coherent_dma_mask = 0xFFFFFFFF,
		},
};

static void mxc_init_fb(void)
{
	(void)platform_device_register(&mxc_fb_device);
}
#else
static inline void mxc_init_fb(void)
{
}
#endif

#if defined(CONFIG_BACKLIGHT_MXC)
static struct platform_device mxcbl_devices[] = {
#if defined(CONFIG_BACKLIGHT_MXC_PMIC) || defined(CONFIG_BACKLIGHT_MXC_PMIC_MODULE)
	{
	 .name = "mxc_pmic_bl",
	 .id = 0,
	 .dev = {
		 .platform_data = (void *)-1,	/* DISP # for this backlight */
		 },
	 },
	{
	 .name = "mxc_pmic_bl",
	 .id = 1,
	 .dev = {
		 .platform_data = (void *)0,	/* DISP # for this backlight */
		 },
	 },
#endif
#if defined(CONFIG_BACKLIGHT_MXC_IPU) || defined(CONFIG_BACKLIGHT_MXC_IPU_MODULE)
	{
	 .name = "mxc_ipu_bl",
	 .id = 0,
	 .dev = {
		 .platform_data = (void *)3,	/* DISP # for this backlight */
		 },
	 },
#endif
};
static inline void mxc_init_bl(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(mxcbl_devices); i++) {
		platform_device_register(&mxcbl_devices[i]);
	}
}
#else
static inline void mxc_init_bl(void)
{
}
#endif

/*!
 * Data structures and data for mt9v111 camera.
 */
static struct mxc_camera_platform_data camera_mt9v111_data = {
	.mclk = 27000000,
};

/*!
 * Data structures and data for ov2640 camera.
 */
static struct mxc_camera_platform_data camera_ov2640_data = {
	.core_regulator = NULL,
	.io_regulator = NULL,
	.analog_regulator = NULL,
	.gpo_regulator = NULL,
	.mclk = 24000000,
};

/*!
 * Info to register i2c devices.
 */
static struct i2c_board_info mxc_i2c_info[] __initdata = {
	{
	 .type = "mt9v111",
	 .addr = 0x48,
	 .platform_data = (void *)&camera_mt9v111_data,
	 },
	{
	 .type = "ov2640",
	 .addr = 0x30,
	 .platform_data = (void *)&camera_ov2640_data,
	 },
};

#if defined(CONFIG_MXC_FIR) || defined(CONFIG_MXC_FIR_MODULE)
/*!
 * Resource definition for the FIR
 */
static struct resource mxcir_resources[] = {
	[0] = {
	       .start = UART2_BASE_ADDR,
	       .end = UART2_BASE_ADDR + SZ_16K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_UART2,
	       .end = MXC_INT_UART2,
	       .flags = IORESOURCE_IRQ,
	       },
	[2] = {
	       .start = FIRI_BASE_ADDR,
	       .end = FIRI_BASE_ADDR + SZ_16K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[3] = {
	       .start = MXC_INT_FIRI,
	       .end = MXC_INT_FIRI,
	       .flags = IORESOURCE_IRQ,
	       },
	[4] = {
	       .start = MXC_INT_UART2,
	       .end = MXC_INT_UART2,
	       .flags = IORESOURCE_IRQ,
	       }
};

static struct mxc_ir_platform_data ir_data = {
	.uart_ir_mux = 1,
	.ir_rx_invert = MXC_IRDA_RX_INV,
	.ir_tx_invert = MXC_IRDA_TX_INV,
};

/*! Device Definition for MXC FIR */
static struct platform_device mxcir_device = {
	.name = "mxcir",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &ir_data,
		},
	.num_resources = ARRAY_SIZE(mxcir_resources),
	.resource = mxcir_resources,
};

static inline void mxc_init_ir(void)
{
	ir_data.uart_clk = clk_get(NULL, "uart_clk.1");;
	(void)platform_device_register(&mxcir_device);
}
#else
static inline void mxc_init_ir(void)
{
}
#endif

static void mxc_expio_irq_handler(u32 irq, struct irq_desc *desc)
{
	u32 imr_val;
	u32 int_valid;
	u32 expio_irq;

	desc->chip->mask(irq);	/* irq = gpio irq number */

	imr_val = __raw_readw(PBC_INTMASK_SET_REG);
	int_valid = __raw_readw(PBC_INTSTATUS_REG) & imr_val;

	if (unlikely(!int_valid)) {
		printk(KERN_ERR "\nEXPIO: Spurious interrupt:0x%0x\n\n",
		       int_valid);
		goto out;
	}

	expio_irq = MXC_EXP_IO_BASE;
	for (; int_valid != 0; int_valid >>= 1, expio_irq++) {
		struct irq_desc *d;
		if ((int_valid & 1) == 0)
			continue;
		d = irq_desc + expio_irq;
		if (unlikely(!(d->handle_irq))) {
			printk(KERN_ERR "\nEXPIO irq: %d unhandeled\n",
			       expio_irq);
			BUG();	/* oops */
		}
		d->handle_irq(expio_irq, d);
	}

      out:
	desc->chip->ack(irq);
	desc->chip->unmask(irq);
}

/*
 * Disable an expio pin's interrupt by setting the bit in the imr.
 * @param irq		an expio virtual irq number
 */
static void expio_mask_irq(u32 irq)
{
	u32 expio = MXC_IRQ_TO_EXPIO(irq);
	/* mask the interrupt */
	__raw_writew(1 << expio, PBC_INTMASK_CLEAR_REG);
}

/*
 * Acknowledge an expanded io pin's interrupt by clearing the bit in the isr.
 * @param irq		an expanded io virtual irq number
 */
static void expio_ack_irq(u32 irq)
{
	u32 expio = MXC_IRQ_TO_EXPIO(irq);
	/* clear the interrupt status */
	__raw_writew(1 << expio, PBC_INTSTATUS_REG);
	/* mask the interrupt */
	expio_mask_irq(irq);
}

/*
 * Enable a expio pin's interrupt by clearing the bit in the imr.
 * @param irq		a expio virtual irq number
 */
static void expio_unmask_irq(u32 irq)
{
	u32 expio = MXC_IRQ_TO_EXPIO(irq);
	/* unmask the interrupt */
	__raw_writew(1 << expio, PBC_INTMASK_SET_REG);
}

static struct irq_chip expio_irq_chip = {
	.ack = expio_ack_irq,
	.mask = expio_mask_irq,
	.unmask = expio_unmask_irq,
};

static int initialized = 0;

static int __init _mxc_expio_init(void)
{
	int i;

	initialized = 1;

	printk(KERN_INFO "MX31ADS EXPIO(CPLD) hardware\n");
	/*
	 * Configure INT line as GPIO input
	 */
	mxc_request_iomux(MX31_PIN_GPIO1_4, OUTPUTCONFIG_GPIO,
			  INPUTCONFIG_GPIO);
	mxc_set_gpio_direction(MX31_PIN_GPIO1_4, 1);

	/* disable the interrupt and clear the status */
	__raw_writew(0xFFFF, PBC_INTMASK_CLEAR_REG);
	__raw_writew(0xFFFF, PBC_INTSTATUS_REG);
	for (i = MXC_EXP_IO_BASE; i < (MXC_EXP_IO_BASE + MXC_MAX_EXP_IO_LINES);
	     i++) {
		set_irq_chip(i, &expio_irq_chip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}
	set_irq_type(EXPIO_PARENT_INT, IRQF_TRIGGER_HIGH);
	set_irq_chained_handler(EXPIO_PARENT_INT, mxc_expio_irq_handler);

	return 0;
}

/*
 * This may get called early from board specific init
 */
int mxc_expio_init(void)
{
	if (!initialized)
		return _mxc_expio_init();
	else
		return 0;
}

/* MMC device data */

#if defined(CONFIG_MMC_MXC) || defined(CONFIG_MMC_MXC_MODULE)
extern unsigned int sdhc_get_card_det_status(struct device *dev);
extern int sdhc_init_card_det(int id);

static struct mxc_mmc_platform_data mmc1_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30,
	.min_clk = 150000,
	.max_clk = 25000000,
	.card_inserted_state = 1,
	.status = sdhc_get_card_det_status,
	.power_mmc = "VMMC1",
};
static struct mxc_mmc_platform_data mmc2_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30,
	.min_clk = 150000,
	.max_clk = 25000000,
	.card_inserted_state = 1,
	.status = sdhc_get_card_det_status,
	.power_mmc = "VMMC2",
};

/*!
 * Resource definition for the SDHC1
 */
static struct resource mxcsdhc1_resources[] = {
	[0] = {
	       .start = MMC_SDHC1_BASE_ADDR,
	       .end = MMC_SDHC1_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_MMC_SDHC1,
	       .end = MXC_INT_MMC_SDHC1,
	       .flags = IORESOURCE_IRQ,
	       },
	[2] = {
	       .start = 0,
	       .end = 0,
	       .flags = IORESOURCE_IRQ,
	       },
};

/*!
 * Resource definition for the SDHC2
 */
static struct resource mxcsdhc2_resources[] = {
	[0] = {
	       .start = MMC_SDHC2_BASE_ADDR,
	       .end = MMC_SDHC2_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_MMC_SDHC2,
	       .end = MXC_INT_MMC_SDHC2,
	       .flags = IORESOURCE_IRQ,
	       },
	[2] = {
	       .start = 0,
	       .end = 0,
	       .flags = IORESOURCE_IRQ,
	       },
};

/*! Device Definition for MXC SDHC1 */
static struct platform_device mxcsdhc1_device = {
	.name = "mxcmci",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mmc1_data,
		},
	.num_resources = ARRAY_SIZE(mxcsdhc1_resources),
	.resource = mxcsdhc1_resources,
};

/*! Device Definition for MXC SDHC2 */
static struct platform_device mxcsdhc2_device = {
	.name = "mxcmci",
	.id = 1,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mmc2_data,
		},
	.num_resources = ARRAY_SIZE(mxcsdhc2_resources),
	.resource = mxcsdhc2_resources,
};

static inline void mxc_init_mmc(void)
{
	int cd_irq;

	cd_irq = sdhc_init_card_det(0);
	if (cd_irq) {
		mxcsdhc1_device.resource[2].start = cd_irq;
		mxcsdhc1_device.resource[2].end = cd_irq;
	}

	cd_irq = sdhc_init_card_det(1);
	if (cd_irq) {
		mxcsdhc2_device.resource[2].start = cd_irq;
		mxcsdhc2_device.resource[2].end = cd_irq;
	}

	spba_take_ownership(SPBA_SDHC1, SPBA_MASTER_A | SPBA_MASTER_C);
	(void)platform_device_register(&mxcsdhc1_device);
	spba_take_ownership(SPBA_SDHC2, SPBA_MASTER_A | SPBA_MASTER_C);
	(void)platform_device_register(&mxcsdhc2_device);
}
#else
static inline void mxc_init_mmc(void)
{
}
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
	mxc_cpu_init();
}

#if (defined(CONFIG_MXC_PMIC_MC13783) || \
	defined(CONFIG_MXC_PMIC_MC13783_MODULE)) \
	&& (defined(CONFIG_SND_MXC_PMIC) || defined(CONFIG_SND_MXC_PMIC_MODULE))
extern void gpio_activate_audio_ports(void);

static void __init mxc_init_pmic_audio(void)
{
	struct clk *ckih_clk;
	struct clk *cko_clk;

	/* Enable 26 mhz clock on CKO1 for PMIC audio */
	ckih_clk = clk_get(NULL, "ckih");
	cko_clk = clk_get(NULL, "cko1_clk");
	if (IS_ERR(ckih_clk) || IS_ERR(cko_clk)) {
		printk(KERN_ERR "Unable to set CKO1 output to CKIH\n");
	} else {
		clk_set_parent(cko_clk, ckih_clk);
		clk_set_rate(cko_clk, clk_get_rate(ckih_clk));
		clk_enable(cko_clk);
	}
	clk_put(ckih_clk);
	clk_put(cko_clk);

	gpio_activate_audio_ports();
}
#else
static void __inline mxc_init_pmic_audio(void)
{
}
#endif

/* IDE device data */
#if defined(CONFIG_BLK_DEV_IDE_MXC) || defined(CONFIG_BLK_DEV_IDE_MXC_MODULE)

/*! Platform Data for MXC IDE */
static struct mxc_ide_platform_data mxc_ide_data = {
	.power_drive = NULL,
	.power_io = NULL,
};

static struct platform_device mxc_ide_device = {
	.name = "mxc_ide",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_ide_data,
		},
};

static inline void mxc_init_ide(void)
{
	if (platform_device_register(&mxc_ide_device) < 0)
		printk(KERN_ERR "Error: Registering the ide.\n");
}
#else
static inline void mxc_init_ide(void)
{
}
#endif

#if defined(CONFIG_PATA_FSL) || defined(CONFIG_PATA_FSL_MODULE)
extern void gpio_ata_active(void);
extern void gpio_ata_inactive(void);

static int ata_init(struct platform_device *pdev)
{
	/* Configure the pins */
	gpio_ata_active();

	return 0;
}

static void ata_exit(void)
{
	/* Free the pins */
	gpio_ata_inactive();
}

static struct fsl_ata_platform_data ata_data = {
	.udma_mask = ATA_UDMA3,	/* board can handle up to UDMA3 */
	.mwdma_mask = ATA_MWDMA2,
	.pio_mask = ATA_PIO4,
	.fifo_alarm = MXC_IDE_DMA_WATERMARK / 2,
	.max_sg = MXC_IDE_DMA_BD_NR,
	.init = ata_init,
	.exit = ata_exit,
	.core_reg = NULL,	/*"LDO2", */
	.io_reg = NULL,		/*"LDO3", */
};

static struct resource pata_fsl_resources[] = {
	[0] = {			/* I/O */
	       .start = ATA_BASE_ADDR + 0x00,
	       .end = ATA_BASE_ADDR + 0xD8,
	       .flags = IORESOURCE_MEM,
	       },
	[2] = {			/* IRQ */
	       .start = MXC_INT_ATA,
	       .end = MXC_INT_ATA,
	       .flags = IORESOURCE_IRQ,
	       },
};

static struct platform_device pata_fsl_device = {
	.name = "pata_fsl",
	.id = -1,
	.num_resources = ARRAY_SIZE(pata_fsl_resources),
	.resource = pata_fsl_resources,
	.dev = {
		.platform_data = &ata_data,
		.coherent_dma_mask = ~0,
		},
};

static void __init mxc_init_pata(void)
{
	(void)platform_device_register(&pata_fsl_device);
}
#else				/* CONFIG_PATA_FSL */
static void __init mxc_init_pata(void)
{
}
#endif				/* CONFIG_PATA_FSL */

/*!
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	mxc_cpu_common_init();
	early_console_setup(saved_command_line);
	mxc_init_devices();
	mxc_init_pmic_audio();
	mxc_gpio_init();
	mx31ads_gpio_init();
	mxc_expio_init();
	mxc_init_keypad();
	mxc_init_extuart();
	mxc_init_nor_mtd();
	mxc_init_nand_mtd();
	imx_nfc_init();

	i2c_register_board_info(0, mxc_i2c_info, ARRAY_SIZE(mxc_i2c_info));
	spi_register_board_info(mxc_spi_board_info,
				ARRAY_SIZE(mxc_spi_board_info));

	mxc_init_fb();
	mxc_init_bl();
	mxc_init_ir();
	mxc_init_mmc();
	mxc_init_ide();
	mxc_init_pata();
}

unsigned long board_get_ckih_rate(void)
{
}
static void __init mx31ads_timer_init(void)
{
	unsigned long ckih = 26000000;

	if ((__raw_readw(PBC_BASE_ADDRESS + PBC_BSTAT) &
	     CKIH_27MHZ_BIT_SET) != 0) {
		ckih = 27000000;
	}

	mxc_clocks_init(32768, 0, ckih, 0);
	mxc_timer_init("gpt_clk");
}

static struct sys_timer mx31ads_timer = {
	.init	= mx31ads_timer_init,
};


#define PLL_PCTL_REG(pd, mfd, mfi, mfn)		\
	((((pd) - 1) << 26) + (((mfd) - 1) << 16) + ((mfi)  << 10) + mfn)

/* For 26MHz input clock */
#define PLL_532MHZ		PLL_PCTL_REG(1, 13, 10, 3)
#define PLL_399MHZ		PLL_PCTL_REG(1, 52, 7, 35)
#define PLL_133MHZ		PLL_PCTL_REG(2, 26, 5, 3)

/* For 27MHz input clock */
#define PLL_532_8MHZ		PLL_PCTL_REG(1, 15, 9, 13)
#define PLL_399_6MHZ		PLL_PCTL_REG(1, 18, 7, 7)
#define PLL_133_2MHZ		PLL_PCTL_REG(3, 5, 7, 2)

#define PDR0_REG(mcu, max, hsp, ipg, nfc)	\
	(MXC_CCM_PDR0_MCU_DIV_##mcu | MXC_CCM_PDR0_MAX_DIV_##max | \
	 MXC_CCM_PDR0_HSP_DIV_##hsp | MXC_CCM_PDR0_IPG_DIV_##ipg | \
	 MXC_CCM_PDR0_NFC_DIV_##nfc)

/* working point(wp): 0 - 133MHz; 1 - 266MHz; 2 - 399MHz; 3 - 532MHz */
/* 26MHz input clock table */
static struct cpu_wp cpu_wp_26[] = {
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 133000000,
	 .pdr0_reg = PDR0_REG(4, 4, 4, 2, 6),},
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 266000000,
	 .pdr0_reg = PDR0_REG(2, 4, 4, 2, 6),},
	{
	 .pll_reg = PLL_399MHZ,
	 .pll_rate = 399000000,
	 .cpu_rate = 399000000,
	 .pdr0_reg = PDR0_REG(1, 3, 3, 2, 6),},
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 532000000,
	 .pdr0_reg = PDR0_REG(1, 4, 4, 2, 6),},
};

/* 27MHz input clock table */
static struct cpu_wp cpu_wp_27[] = {
	{
	 .pll_reg = PLL_532_8MHZ,
	 .pll_rate = 532800000,
	 .cpu_rate = 133200000,
	 .pdr0_reg = PDR0_REG(4, 4, 4, 2, 6),},
	{
	 .pll_reg = PLL_532_8MHZ,
	 .pll_rate = 532800000,
	 .cpu_rate = 266400000,
	 .pdr0_reg = PDR0_REG(2, 4, 4, 2, 6),},
	{
	 .pll_reg = PLL_399_6MHZ,
	 .pll_rate = 399600000,
	 .cpu_rate = 399600000,
	 .pdr0_reg = PDR0_REG(1, 3, 3, 2, 6),},
	{
	 .pll_reg = PLL_532_8MHZ,
	 .pll_rate = 532800000,
	 .cpu_rate = 532800000,
	 .pdr0_reg = PDR0_REG(1, 4, 4, 2, 6),},
};

struct cpu_wp *get_cpu_wp(int *wp)
{
	*wp = 4;
	if ((__raw_readw(PBC_BASE_ADDRESS + PBC_BSTAT) &
	     CKIH_27MHZ_BIT_SET) != 0) {
		return cpu_wp_27;
	} else {
		return cpu_wp_26;
	}
}

/*
 * The following uses standard kernel macros defined in arch.h in order to
 * initialize __mach_desc_MX31ADS data structure.
 */
/* *INDENT-OFF* */
MACHINE_START(MX31ADS, "Freescale MX31/MX32 ADS")
	/* Maintainer: Freescale Semiconductor, Inc. */
#ifdef CONFIG_SERIAL_8250_CONSOLE
	.phys_io = CS4_BASE_ADDR,
	.io_pg_offst = ((CS4_BASE_ADDR_VIRT) >> 18) & 0xfffc,
#else
	.phys_io = AIPS1_BASE_ADDR,
	.io_pg_offst = ((AIPS1_BASE_ADDR_VIRT) >> 18) & 0xfffc,
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mxc_map_io,
	.init_irq = mxc_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mx31ads_timer,
MACHINE_END
