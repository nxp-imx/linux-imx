/*
 * Freescale STMP378X platform support
 *
 * Embedded Alley Solutions, Inc <source@embeddedalley.com>
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysdev.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <asm/dma.h>
#include <asm/setup.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <asm/mach/irq.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>

#include <mach/ocram-malloc.h>

#include <mach/lcdif.h>
#include <mach/system.h>
#include <mach/platform.h>
#include <mach/regs-icoll.h>
#include <mach/regs-apbh.h>
#include <mach/regs-apbx.h>
#include <mach/regs-ocotp.h>

#include "common.h"

/*
 * IRQ handling
 */
static void stmp378x_ack_irq(unsigned int irq)
{
	/* Tell ICOLL to release IRQ line */
	HW_ICOLL_VECTOR_WR(0x0);

	/* ACK current interrupt */
	HW_ICOLL_LEVELACK_WR(BV_ICOLL_LEVELACK_IRQLEVELACK__LEVEL0);

	/* Barrier */
	(void) HW_ICOLL_STAT_RD();
}

static void stmp378x_disable_irq(unsigned int irq)
{
	/* IRQ disable */
	HW_ICOLL_INTERRUPTn_CLR(irq, BM_ICOLL_INTERRUPTn_ENABLE);
}

static void stmp378x_mask_irq(unsigned int irq)
{
	/* IRQ disable */
	HW_ICOLL_INTERRUPTn_CLR(irq, BM_ICOLL_INTERRUPTn_ENABLE);
}

static void stmp378x_unmask_irq(unsigned int irq)
{
	/* IRQ enable */
	HW_ICOLL_INTERRUPTn_SET(irq, BM_ICOLL_INTERRUPTn_ENABLE);
}

static struct irq_chip stmp378x_chip = {
	.disable = stmp378x_disable_irq,
	.ack	= stmp378x_ack_irq,
	.mask	= stmp378x_mask_irq,
	.unmask = stmp378x_unmask_irq,
};

static void stmp378x_gpio_ack_irq(unsigned int irq)
{
	stmp3xxx_pin_ack_irq(irq);
	stmp378x_ack_irq(irq);
}

static struct irq_chip stmp378x_gpio_chip = {
	.disable = stmp378x_disable_irq,
	.ack	= stmp378x_gpio_ack_irq,
	.mask	= stmp378x_mask_irq,
	.unmask = stmp378x_unmask_irq,
};

static int stmp378x_irq_is_gpio(int irq)
{
	return irq == IRQ_GPIO0 || irq == IRQ_GPIO1 || irq == IRQ_GPIO2;
}

void __init stmp378x_init_irq(void)
{
	stmp3xxx_init_irq(&stmp378x_chip,
		&stmp378x_gpio_chip,
		stmp378x_irq_is_gpio);
}

/*
 * DMA interrupt handling
 */
void stmp3xxx_arch_dma_enable_interrupt(int channel)
{
	int dmabus = channel / 16;

	switch (dmabus) {
	case STMP3XXX_BUS_APBH:
		HW_APBH_CTRL1_SET(1 << (16 + (channel % 16)));
		HW_APBH_CTRL2_SET(1 << (16 + (channel % 16)));
		break;

	case STMP3XXX_BUS_APBX:
		HW_APBX_CTRL1_SET(1 << (16 + (channel % 16)));
		HW_APBX_CTRL2_SET(1 << (16 + (channel % 16)));
		break;
	}
}
EXPORT_SYMBOL(stmp3xxx_arch_dma_enable_interrupt);

void stmp3xxx_arch_dma_clear_interrupt(int channel)
{
	int dmabus = channel / 16;

	switch (dmabus) {
	case STMP3XXX_BUS_APBH:
		HW_APBH_CTRL1_CLR(1 << (channel % 16));
		HW_APBH_CTRL2_CLR(1 << (channel % 16));
		break;

	case STMP3XXX_BUS_APBX:
		HW_APBX_CTRL1_CLR(1 << (channel % 16));
		HW_APBX_CTRL2_CLR(1 << (channel % 16));
		break;
	}
}
EXPORT_SYMBOL(stmp3xxx_arch_dma_clear_interrupt);

int stmp3xxx_arch_dma_is_interrupt(int channel)
{
	int dmabus = channel / 16;
	int r = 0;

	switch (dmabus) {
	case STMP3XXX_BUS_APBH:
		r = HW_APBH_CTRL1_RD() & (1 << (channel % 16));
		break;

	case STMP3XXX_BUS_APBX:
		r = HW_APBX_CTRL1_RD() & (1 << (channel % 16));
		break;
	}
	return r;
}
EXPORT_SYMBOL(stmp3xxx_arch_dma_is_interrupt);

void stmp3xxx_arch_dma_reset_channel(int channel)
{
	int dmabus = channel / 16;
	unsigned chbit = 1 << (channel % 16);

	switch (dmabus) {
	case STMP3XXX_BUS_APBH:
		/* Reset channel and wait for it to complete */
		HW_APBH_CTRL0_SET(chbit <<
				 BP_APBH_CTRL0_RESET_CHANNEL);
		while (HW_APBH_CTRL0_RD() &
		       (chbit << BP_APBH_CTRL0_RESET_CHANNEL))
			continue;
		break;

	case STMP3XXX_BUS_APBX:
		/* Reset channel and wait for it to complete */
		HW_APBX_CHANNEL_CTRL_SET(
			BF_APBX_CHANNEL_CTRL_RESET_CHANNEL(chbit));
		while (HW_APBX_CHANNEL_CTRL_RD() &
			BF_APBX_CHANNEL_CTRL_RESET_CHANNEL(chbit))
			continue;
		break;
	}
}
EXPORT_SYMBOL(stmp3xxx_arch_dma_reset_channel);

void stmp3xxx_arch_dma_freeze(int channel)
{
	int dmabus = channel / 16;
	unsigned chbit = 1 << (channel % 16);

	switch (dmabus) {
	case STMP3XXX_BUS_APBH:
		HW_APBH_CTRL0_SET(1<<chbit);
		break;
	case STMP3XXX_BUS_APBX:
		HW_APBX_CHANNEL_CTRL_SET(1<<chbit);
		break;
	}
}
EXPORT_SYMBOL(stmp3xxx_arch_dma_freeze);

void stmp3xxx_arch_dma_unfreeze(int channel)
{
	int dmabus = channel / 16;
	unsigned chbit = 1 << (channel % 16);

	switch (dmabus) {
	case STMP3XXX_BUS_APBH:
		HW_APBH_CTRL0_CLR(1<<chbit);
		break;
	case STMP3XXX_BUS_APBX:
		HW_APBX_CHANNEL_CTRL_CLR(1<<chbit);
		break;
	}
}
EXPORT_SYMBOL(stmp3xxx_arch_dma_unfreeze);

/*
 * STMP378x dedicated pin groups
 */
static struct pin_desc i2c_pins_desc[] = {
	{ PINID_I2C_SCL, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_I2C_SDA, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
};

struct pin_group i2c_pins = {
	.pins		= i2c_pins_desc,
	.nr_pins	= ARRAY_SIZE(i2c_pins_desc),
};

static struct pin_desc gpmi_pins_desc[] = {
	{ PINID_GPMI_CE0N, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_CE1N, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GMPI_CE2N, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_CLE, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_ALE, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_WPN, PIN_FUN1, PIN_12MA, PIN_3_3V, 0 },
	{ PINID_GPMI_RDY1, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_D00, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_D01, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_D02, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_D03, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_D04, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_D05, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_D06, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_D07, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_RDY0, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_RDY2, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_RDY3, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_WRN, PIN_FUN1, PIN_12MA, PIN_3_3V, 0 },
	{ PINID_GPMI_RDN, PIN_FUN1, PIN_12MA, PIN_3_3V, 0 },
};

struct pin_group gpmi_pins = {
	.pins		= gpmi_pins_desc,
	.nr_pins	= ARRAY_SIZE(gpmi_pins_desc),
};

static struct pin_desc lcd_hx8238a_desc[] = {
	{ PINID_LCD_D00, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D01, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D02, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D03, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D04, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D05, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D06, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D07, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D08, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D09, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D10, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D11, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D12, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D13, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D14, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D15, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D16, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D17, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_RESET, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_VSYNC, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_HSYNC, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_ENABLE, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_DOTCK, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_GPMI_D13, PIN_FUN2, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_GPMI_D12, PIN_FUN2, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_GPMI_D11, PIN_FUN2, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_GPMI_D10, PIN_FUN2, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_GPMI_D09, PIN_FUN2, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_GPMI_D08, PIN_FUN2, PIN_8MA, PIN_3_3V, 0 },
};

struct pin_group stmp378x_lcd_pins = {
	.pins		= lcd_hx8238a_desc,
	.nr_pins	= ARRAY_SIZE(lcd_hx8238a_desc),
};

unsigned stmp378x_lcd_spi_pins[] = {
	[SPI_MOSI] = PINID_LCD_WR,
	[SPI_SCLK] = PINID_LCD_RS,
	[SPI_CS] = PINID_LCD_CS,
};

struct pin_desc appuart_pins_0[] = {
	{ PINID_AUART1_CTS, PIN_FUN1, PIN_4MA, PIN_1_8V, 0, },
	{ PINID_AUART1_RTS, PIN_FUN1, PIN_4MA, PIN_1_8V, 0, },
	{ PINID_AUART1_RX, PIN_FUN1, PIN_4MA, PIN_1_8V, 0, },
	{ PINID_AUART1_TX, PIN_FUN1, PIN_4MA, PIN_1_8V, 0, },
};

#if 0 /* temporary ? */
struct pin_desc appuart_pins_1[] = {
	{ PINID_AUART2_CTS, PIN_FUN1, PIN_4MA, PIN_1_8V, 0, },
	{ PINID_AUART2_RTS, PIN_FUN1, PIN_4MA, PIN_1_8V, 0, },
	{ PINID_AUART2_RX, PIN_FUN1, PIN_4MA, PIN_1_8V, 0, },
	{ PINID_AUART2_TX, PIN_FUN1, PIN_4MA, PIN_1_8V, 0, },
};
#endif

struct pin_group appuart_pins[] = {
	[0] = {
		.pins		= appuart_pins_0,
		.nr_pins	= ARRAY_SIZE(appuart_pins_0),
	},
	[1] = {
#if 0		/* undefined yet. */
		.pins		= appuart_pins_1,
		.nr_pins	= ARRAY_SIZE(appuart_pins_1),
#endif
	},
};

struct pin_desc dbguart_pins_0[] = {
	{ PINID_PWM0, PIN_FUN3, },
	{ PINID_PWM1, PIN_FUN3, },
};

struct pin_group dbguart_pins[] = {
	[0] = {
		.pins		= dbguart_pins_0,
		.nr_pins	= ARRAY_SIZE(dbguart_pins_0),
	},
};

static struct pin_desc usb_mux_pins_desc[] = {
	{ PINID_SSP1_DETECT,	PIN_FUN3, },
};

struct pin_group usb_mux_pins = {
	.pins 		= usb_mux_pins_desc,
	.nr_pins 	= ARRAY_SIZE(usb_mux_pins_desc),
};

static struct pin_desc spdif_pins_desc[] = {
	{ PINID_ROTARYA, PIN_FUN3, PIN_4MA, PIN_1_8V, 0, },
};

struct pin_group spdif_pins = {
	.pins		= spdif_pins_desc,
	.nr_pins	= ARRAY_SIZE(spdif_pins_desc),
};

/*
 * The registers are all very closely mapped, so we might as well map them all
 * with a single mapping
 *
 * Logical      Physical
 * f0000000	80000000	On-chip registers
 * f1000000	00000000	256k on-chip SRAM
 */

static struct map_desc stmp378x_io_desc[] __initdata = {
	{
		.virtual	= IO_ADDRESS(STMP378X_REGS_BASE),
		.pfn		= __phys_to_pfn(STMP378X_REGS_BASE),
		.length		= STMP378X_REGS_SIZE,
		.type		= MT_DEVICE,
	},
	{
		.virtual	= STMP3XXX_OCRAM_VA_BASE,
		.pfn		= __phys_to_pfn(STMP378X_OCRAM_BASE),
		.length		= STMP378X_OCRAM_SIZE,
		.type		= MT_DEVICE,
	},
};

void __init stmp378x_map_io(void)
{
	iotable_init(stmp378x_io_desc, ARRAY_SIZE(stmp378x_io_desc));
}

int get_evk_board_version()
{
	int boardid;
	boardid = HW_OCOTP_CUSTCAP_RD();
	boardid &= 0x30000000;
	boardid = boardid >> 28;

	return boardid;
}

EXPORT_SYMBOL_GPL(get_evk_board_version);

