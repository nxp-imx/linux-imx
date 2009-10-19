/*
 * Freescale STMP37XX/STMP378X SPI module pin multiplexing
 *
 * Author: dmitry pervushin <dimka@embeddedalley.com>
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
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <mach/stmp3xxx.h>
#include <mach/pinmux.h>

/*
   These pins are:
	SCK	(SSPx_SCK)
	MOSI	(SSPx_CMD)
	MISO	(SSPx_DATA0)
	SSn	(SSPx_DATA3)
   Please add new pins in the same order, thanks :)
*/
static struct pin_desc ssp_pins_desc[2][4] = {
	[0] = {
		{ PINID_SSP1_SCK,	PIN_FUN1, PIN_8MA, PIN_3_3V, 0, },
		{ PINID_SSP1_CMD,	PIN_FUN1, PIN_4MA, PIN_3_3V, 0, },
		{ PINID_SSP1_DATA0,	PIN_FUN1, PIN_4MA, PIN_3_3V, 0, },
		{ PINID_SSP1_DATA3,	PIN_FUN1, PIN_4MA, PIN_3_3V, 0, },
	},
	[1] = {
#if defined(CONFIG_ARCH_STMP37XX)
		{ PINID_GPMI_IRQ,	PIN_FUN3, PIN_8MA, PIN_3_3V, 0, },
		{ PINID_GPMI_RDY2,	PIN_FUN3, PIN_4MA, PIN_3_3V, 0, },
		{ PINID_EMI_CE2N,	PIN_FUN3, PIN_4MA, PIN_3_3V, 0, },
		{ PINID_GPMI_D03,	PIN_FUN3, PIN_4MA, PIN_3_3V, 0, },
#elif defined(CONFIG_ARCH_STMP378X)
		{ PINID_GPMI_WRN,	PIN_FUN3, PIN_8MA, PIN_3_3V, 0, },
		{ PINID_GPMI_RDY1,	PIN_FUN3, PIN_4MA, PIN_3_3V, 0, },
		{ PINID_GPMI_D00,	PIN_FUN3, PIN_4MA, PIN_3_3V, 0, },
		{ PINID_GPMI_D03,	PIN_FUN3, PIN_4MA, PIN_3_3V, 0, },
#endif
	},
};

static struct pin_group ssp_pins[2] = {
	[0] = {
		.pins = ssp_pins_desc[0],
		.nr_pins = ARRAY_SIZE(ssp_pins_desc[0]),
	},
	[1] = {
		.pins = ssp_pins_desc[1],
		.nr_pins = ARRAY_SIZE(ssp_pins_desc[1]),
	},
};

int stmp37xx_spi_pins_request(char *id, int ssp)
{
	return stmp3xxx_request_pin_group(&ssp_pins[ssp-1], id);
}
EXPORT_SYMBOL_GPL(stmp37xx_spi_pins_request);

void stmp37xx_spi_pins_release(char *id, int ssp)
{
	stmp3xxx_release_pin_group(&ssp_pins[ssp-1], id);
}
EXPORT_SYMBOL_GPL(stmp37xx_spi_pins_release);

int stmp37xx_spi_enc_init(void *spi_dev)
{
	struct spi_device *spi = spi_dev;
	struct stmp37xx_spi_platform_data *data = spi->dev.platform_data;

	gpio_request(data->irq_pin, dev_name(&spi->dev));
	gpio_direction_input(data->irq_pin);
	set_irq_type(gpio_to_irq(data->irq_pin), IRQ_TYPE_EDGE_FALLING);
	spi->irq = gpio_to_irq(data->irq_pin);
	dev_dbg(&spi->dev, "Assigned IRQ %d(%s)\n", spi->irq, __func__);
	return 0;
}

int stmp37xx_spi_enc_release(void *spi_dev)
{
	struct spi_device *spi = spi_dev;
	struct stmp37xx_spi_platform_data *data = spi->dev.platform_data;

	set_irq_type(data->irq_pin, IRQ_TYPE_NONE);
	gpio_free(data->irq_pin);
	return 0;
}

MODULE_AUTHOR("dmitry pervushin <dimka@embeddedalley.com>");
MODULE_LICENSE("GPL");
