/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/hardware.h>
#include <mach/device.h>
#include <mach/pinctrl.h>

#include "device.h"
#include "mx23evk.h"
#include "mx23_pins.h"

static struct mxs_mma7450_platform_data mma7450_platdata = {
	.reg_dvdd_io = "vddio",
	.reg_avdd = "vdda",
	.gpio_pin_get = mx23evk_mma7450_pin_init,
	.gpio_pin_put = mx23evk_mma7450_pin_release,
	/* int1 and int2 will be initialized in
	i2c_device_init */
	.int1 = 0,
	.int2 = 0,
};

static struct i2c_board_info __initdata mma7450_i2c_device = {
	I2C_BOARD_INFO("mma7450", 0x3A),
	.platform_data = (void *)&mma7450_platdata,
};

static void i2c_device_init(void)
{
	mma7450_platdata.int1 = gpio_to_irq(MXS_PIN_TO_GPIO(PINID_GPMI_D14));
	mma7450_platdata.int2 = gpio_to_irq(MXS_PIN_TO_GPIO(PINID_GPMI_D15));
	i2c_register_board_info(0, &mma7450_i2c_device, 1);
}

static void __init fixup_board(struct machine_desc *desc, struct tag *tags,
			       char **cmdline, struct meminfo *mi)
{
	mx23_set_input_clk(24000000, 24000000, 32000, 50000000);
}

#if defined(CONFIG_SND_MXS_SOC_ADC) || defined(CONFIG_SND_MXS_SOC_ADC_MODULE)
static void __init mx23evk_init_adc(void)
{
	struct platform_device *pdev;
	pdev = mxs_get_device("mxs-adc", 0);
	if (pdev == NULL)
		return;
	mxs_add_device(pdev, 3);
}
#else
static void __init mx23evk_init_adc(void)
{

}
#endif


static void __init mx23evk_device_init(void)
{
	/* Add mx23evk special code */
	i2c_device_init();
	mx23evk_init_adc();
}


static void __init mx23evk_init_machine(void)
{
	mx23_pinctrl_init();

	/* Init iram allocate */
	iram_init(MX23_OCRAM_PHBASE, MX23_OCRAM_SIZE);

	mx23_gpio_init();
	mx23evk_pins_init();
	mx23evk_mma7450_pin_init();
	mx23_device_init();
	mx23evk_device_init();
}

MACHINE_START(MX23EVK, "Freescale MX23EVK board")
	.phys_io	= 0x80000000,
	.io_pg_offst	= ((0xf0000000) >> 18) & 0xfffc,
	.boot_params	= 0x40000100,
	.fixup		= fixup_board,
	.map_io		= mx23_map_io,
	.init_irq	= mx23_irq_init,
	.init_machine	= mx23evk_init_machine,
	.timer		= &mx23_timer.timer,
MACHINE_END
