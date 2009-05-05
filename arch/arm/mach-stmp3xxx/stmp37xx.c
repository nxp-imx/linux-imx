/*
 * Freescale STMP37XX platform support
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
#include <linux/spi/spi.h>
#include <linux/fsl_devices.h>

#include <mach/hardware.h>
#include <asm/dma.h>
#include <asm/setup.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <asm/mach/irq.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>

#include <mach/stmp3xxx.h>
#include <mach/gpmi.h>
#include <mach/ocram-malloc.h>

#include <mach/platform.h>
#include <mach/system.h>
#include <mach/lcdif.h>

#include <mach/regs-icoll.h>
#include <mach/regs-rtc.h>
#include <mach/regs-apbh.h>
#include <mach/regs-apbx.h>
#include <mach/regs-power.h>
#include <mach/regs-usbctrl.h>
#include <mach/regs-usbphy.h>
#include <mach/regs-digctl.h>
#include <mach/regs-ssp.h>

#include "common.h"

/*
 * IRQ handling
 */
static void stmp37xx_ack_irq(unsigned int irq)
{
	/* Disable IRQ */
	HW_ICOLL_PRIORITYn_CLR(irq / 4, 0x04 << ((irq % 4) * 8));

	/* ACK current interrupt */
	HW_ICOLL_LEVELACK_WR(1);

	/* Barrier */
	(void) HW_ICOLL_STAT_RD();
}

static void stmp37xx_mask_irq(unsigned int irq)
{
	/* IRQ disable */
	HW_ICOLL_PRIORITYn_CLR(irq / 4, 0x04 << ((irq % 4) * 8));
}

static void stmp37xx_unmask_irq(unsigned int irq)
{
	/* IRQ enable */
	HW_ICOLL_PRIORITYn_SET(irq / 4, 0x04 << ((irq % 4) * 8));
}

static struct irq_chip stmp37xx_chip = {
	.ack	= stmp37xx_ack_irq,
	.mask	= stmp37xx_mask_irq,
	.unmask = stmp37xx_unmask_irq,
};

void __init stmp37xx_init_irq(void)
{
	stmp3xxx_init_irq(&stmp37xx_chip, NULL, NULL);
}

/*
 * DMA interrupt handling
 */
void stmp3xxx_arch_dma_enable_interrupt(int channel)
{
	int dmabus = channel / 16;

	switch (dmabus) {
	case STMP3XXX_BUS_APBH:
		HW_APBH_CTRL1_SET(1 << (8 + (channel % 16)));
		break;

	case STMP3XXX_BUS_APBX:
		HW_APBX_CTRL1_SET(1 << (8 + (channel % 16)));
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
		break;

	case STMP3XXX_BUS_APBX:
		HW_APBX_CTRL1_CLR(1 << (channel % 16));
		break;
	}
}
EXPORT_SYMBOL(stmp3xxx_arch_dma_clear_interrupt);

int stmp3xxx_arch_dma_is_interrupt(int channel)
{
	int r = 0;

	int dmabus = channel / 16;

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
		HW_APBH_CTRL0_SET(chbit << BP_APBH_CTRL0_RESET_CHANNEL);
		while (HW_APBH_CTRL0_RD() &
		       (chbit << BP_APBH_CTRL0_RESET_CHANNEL))
			continue;
		break;

	case STMP3XXX_BUS_APBX:
		/* Reset channel and wait for it to complete */
		HW_APBX_CTRL0_SET(chbit << BP_APBX_CTRL0_RESET_CHANNEL);
		while (HW_APBX_CTRL0_RD() &
		       (chbit << BP_APBX_CTRL0_RESET_CHANNEL))
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
		HW_APBX_CTRL0_SET(1<<chbit);
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
		HW_APBX_CTRL0_CLR(1<<chbit);
		break;
	}
}
EXPORT_SYMBOL(stmp3xxx_arch_dma_unfreeze);

/*
 * STMP37xx dedicated pin groups
 */
static struct pin_desc gpmi_pins_desc[] = {
	{ PINID_GPMI_A0, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_A1, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_A2, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_RESETN, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_IRQ, PIN_FUN1, PIN_12MA, PIN_3_3V, 0 },
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
	{ PINID_EMI_CE3N, PIN_FUN2, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_EMI_CE2N, PIN_FUN2, PIN_4MA, PIN_3_3V, 0 },
};

struct pin_group gpmi_pins = {
	.pins		= gpmi_pins_desc,
	.nr_pins	= ARRAY_SIZE(gpmi_pins_desc),
};

static struct pin_desc lcd_hx8238a_desc[] = {
	{ PINID_LCD_D00, PIN_FUN1, PIN_12MA, PIN_3_3V, 0 },
	{ PINID_LCD_D01, PIN_FUN1, PIN_12MA, PIN_3_3V, 0 },
	{ PINID_LCD_D02, PIN_FUN1, PIN_12MA, PIN_3_3V, 0 },
	{ PINID_LCD_D03, PIN_FUN1, PIN_12MA, PIN_3_3V, 0 },
	{ PINID_LCD_D04, PIN_FUN1, PIN_12MA, PIN_3_3V, 0 },
	{ PINID_LCD_D05, PIN_FUN1, PIN_12MA, PIN_3_3V, 0 },
	{ PINID_LCD_D06, PIN_FUN1, PIN_12MA, PIN_3_3V, 0 },
	{ PINID_LCD_D07, PIN_FUN1, PIN_12MA, PIN_3_3V, 0 },
	{ PINID_LCD_D08, PIN_FUN1, PIN_12MA, PIN_3_3V, 0 },
	{ PINID_LCD_D09, PIN_FUN1, PIN_12MA, PIN_3_3V, 0 },
	{ PINID_LCD_D10, PIN_FUN1, PIN_12MA, PIN_3_3V, 0 },
	{ PINID_LCD_D11, PIN_FUN1, PIN_12MA, PIN_3_3V, 0 },
	{ PINID_LCD_D12, PIN_FUN1, PIN_12MA, PIN_3_3V, 0 },
	{ PINID_LCD_D13, PIN_FUN1, PIN_12MA, PIN_3_3V, 0 },
	{ PINID_LCD_D14, PIN_FUN1, PIN_12MA, PIN_3_3V, 0 },
	{ PINID_LCD_D15, PIN_FUN1, PIN_12MA, PIN_3_3V, 0 },
	{ PINID_LCD_RESET, PIN_FUN1, PIN_12MA, PIN_3_3V, 0 },
	{ PINID_LCD_RS, PIN_FUN1, PIN_12MA, PIN_3_3V, 0 },
	{ PINID_LCD_BUSY, PIN_FUN2, PIN_12MA, PIN_3_3V, 0 },
};

struct pin_group stmp37xx_lcd_pins = {
	.pins		= lcd_hx8238a_desc,
	.nr_pins	= ARRAY_SIZE(lcd_hx8238a_desc),
};

unsigned stmp37xx_lcd_spi_pins[] = {
	[SPI_MOSI] = PINID_LCD_WR_RWN,
	[SPI_SCLK] = PINID_LCD_RD_E,
	[SPI_CS] = PINID_LCD_CS
};

/*
 * The registers are all very closely mapped, so we might as well map them all
 * with a single mapping
 *
 * Logical      Physical
 * f0000000	80000000	On-chip registers
 * f1000000	00000000	256k on-chip SRAM
 */

static struct map_desc stmp37xx_io_desc[] __initdata = {
	{
		.virtual	= IO_ADDRESS(STMP37XX_REGS_BASE),
		.pfn		= __phys_to_pfn(STMP37XX_REGS_BASE),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	= STMP3XXX_OCRAM_VA_BASE,
		.pfn		= __phys_to_pfn(STMP37XX_OCRAM_BASE),
		.length		= STMP37XX_OCRAM_SIZE,
		.type		= MT_DEVICE,
	},
};

void __init stmp37xx_map_io(void)
{
	iotable_init(stmp37xx_io_desc, ARRAY_SIZE(stmp37xx_io_desc));
}

/*
 * STMP37xx specific devices
 */

static u64 common_dmamask = (u32)0xffffffffU;
#define COMMON_COHERENT_DMAMASK (u32)0xffffffffU

struct pin_desc appuart_pins_0[] = {
	{ PINID_UART2_CTS, PIN_FUN1, PIN_4MA, PIN_1_8V, 0, },
	{ PINID_UART2_RTS, PIN_FUN1, PIN_4MA, PIN_1_8V, 0, },
	{ PINID_UART2_RX, PIN_FUN1, PIN_4MA, PIN_1_8V, 0, },
	{ PINID_UART2_TX, PIN_FUN1, PIN_4MA, PIN_1_8V, 0, },
};

struct pin_group appuart_pins[] = {
	[0] = {
		.pins		= appuart_pins_0,
		.nr_pins	= ARRAY_SIZE(appuart_pins_0),
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

/* SPI */
static struct stmp37xx_spi_platform_data enc_data = {
	.irq_pin = PINID_GPMI_D01,
	.hw_init = stmp37xx_spi_enc_init,
	.hw_release = stmp37xx_spi_enc_release,
};

static struct spi_board_info spi_board_info[] __initdata = {
#ifdef CONFIG_ENC28J60
{
		.modalias	= "enc28j60",
		.irq		= IRQ_RESERVED_59,
		.max_speed_hz	= 20 * 1000 * 1000,
		.bus_num	= 2,
		.chip_select	= 0,
		.platform_data	= &enc_data,
	},
#endif
};

int stmp37xx_add_devices(void)
{
	if (ARRAY_SIZE(spi_board_info))
		spi_register_board_info(spi_board_info,
			ARRAY_SIZE(spi_board_info));

	return 0;
}
EXPORT_SYMBOL(stmp37xx_add_devices);
