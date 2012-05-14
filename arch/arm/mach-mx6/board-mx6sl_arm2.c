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

void __init early_console_setup(unsigned long base, struct clk *clk);

static inline void mx6_arm2_init_uart(void)
{
	imx6q_add_imx_uart(0, NULL); /* DEBUG UART1 */
}

/*!
 * Board specific initialization.
 */
static void __init mx6_arm2_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx6sl_arm2_pads, ARRAY_SIZE(mx6sl_arm2_pads));

	mx6_arm2_init_uart();
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
