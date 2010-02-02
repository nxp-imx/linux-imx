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
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/mmc/host.h>
#include <linux/phy.h>
#include <linux/fec.h>

#include <asm/mach/map.h>

#include <mach/hardware.h>
#include <mach/regs-timrot.h>
#include <mach/regs-lradc.h>
#include <mach/device.h>
#include <mach/dma.h>
#include <mach/lradc.h>
#include <mach/lcdif.h>

#include "device.h"
#include "mx23_pins.h"
#include "mach/mx23.h"

#if defined(CONFIG_SERIAL_MXS_DUART) || \
	defined(CONFIG_SERIAL_MXS_DUART_MODULE)
static struct resource duart_resource[] = {
	{
	 .flags = IORESOURCE_MEM,
	 .start = DUART_PHYS_ADDR,
	 .end = DUART_PHYS_ADDR + 0x1000 - 1,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_DEBUG_UART ,
	 .end = IRQ_DEBUG_UART ,
	 },
};

static void __init mx23_init_duart(void)
{
	struct platform_device *pdev;
	pdev = mxs_get_device("mxs-duart", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->resource = duart_resource;
	pdev->num_resources = ARRAY_SIZE(duart_resource);
	mxs_add_device(pdev, 3);
}
#else
static void mx23_init_duart(void)
{
}
#endif

#if defined(CONFIG_MXS_DMA_ENGINE)
static struct resource mxs_ahb_apbh_res = {
	.flags = IORESOURCE_MEM,
	.start = APBH_DMA_PHYS_ADDR,
	.end = APBH_DMA_PHYS_ADDR + 0x2000 - 1,
};

static struct mxs_dma_plat_data mxs_ahb_apbh_data = {
	.chan_base = MXS_DMA_CHANNEL_AHB_APBH,
	.chan_num = 16,
};

static struct resource mxs_ahb_apbx_res = {
	.flags = IORESOURCE_MEM,
	.start = APBX_DMA_PHYS_ADDR,
	.end = APBX_DMA_PHYS_ADDR + 0x2000 - 1,
};

static struct mxs_dma_plat_data mxs_ahb_apbx_data = {
	.chan_base = MXS_DMA_CHANNEL_AHB_APBX,
	.chan_num = 16,
};

static void __init mx23_init_dma(void)
{
	int i;
	struct mxs_dev_lookup *lookup;
	struct platform_device *pdev;
	lookup = mxs_get_devices("mxs-dma");
	if (lookup == NULL || IS_ERR(lookup))
		return;
	for (i = 0; i < lookup->size; i++) {
		pdev = lookup->pdev + i;
		if (!strcmp(pdev->name, "mxs-dma-apbh")) {
			pdev->resource = &mxs_ahb_apbh_res;
			pdev->dev.platform_data = &mxs_ahb_apbh_data;
		} else if (!strcmp(pdev->name, "mxs-dma-apbx")) {
			pdev->resource = &mxs_ahb_apbx_res;
			pdev->dev.platform_data = &mxs_ahb_apbx_data;
		} else
			continue;
		pdev->num_resources = 1;
		mxs_add_device(pdev, 0);
	}
}
#else
static void mx23_init_dma(void)
{
	;
}
#endif

#if defined(CONFIG_FB_MXS) || defined(CONFIG_FB_MXS_MODULE)
static struct resource framebuffer_resource[] = {
	{
	 .flags = IORESOURCE_MEM,
	 .start = LCDIF_PHYS_ADDR,
	 .end   = LCDIF_PHYS_ADDR + 0x2000 - 1,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_LCDIF,
	 .end   = IRQ_LCDIF,
	 },
};

static struct mxs_platform_fb_data mxs_framebuffer_pdata = {
	.list = LIST_HEAD_INIT(mxs_framebuffer_pdata.list),
};

static void __init mx23_init_lcdif(void)
{
	struct platform_device *pdev;
	pdev = mxs_get_device("mxs-fb", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->resource = framebuffer_resource;
	pdev->num_resources = ARRAY_SIZE(framebuffer_resource);
	pdev->dev.platform_data = &mxs_framebuffer_pdata;
	mxs_add_device(pdev, 3);
}
#else
static void __init mx23_init_lcdif(void)
{
	;
}
#endif


#if defined(CONFIG_MXS_WATCHDOG) || defined(CONFIG_MXS_WATCHDOG_MODULE)
static struct resource mx23_wdt_res = {
	.flags = IORESOURCE_MEM,
	.start = RTC_PHYS_ADDR,
	.end   = RTC_PHYS_ADDR + 0x2000 - 1,
};

static void __init mx23_init_wdt(void)
{
	struct platform_device *pdev;
	pdev = mxs_get_device("mxs-wdt", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->resource = &mx23_wdt_res;
	pdev->num_resources = 1;
	mxs_add_device(pdev, 3);
}
#else
static void __init mx23_init_wdt(void)
{
	;
}
#endif

#if defined(CONFIG_RTC_DRV_MXS) || defined(CONFIG_RTC_DRV_MXS_MODULE)
static struct resource mx23_rtc_res[] = {
	{
	 .flags = IORESOURCE_MEM,
	 .start = RTC_PHYS_ADDR,
	 .end   = RTC_PHYS_ADDR + 0x2000 - 1,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_RTC_ALARM,
	 .end   = IRQ_RTC_ALARM,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_RTC_1MSEC,
	 .end   = IRQ_RTC_1MSEC,
	},
};

static void __init mx23_init_rtc(void)
{
	struct platform_device *pdev;

	pdev = mxs_get_device("mxs-rtc", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->resource = mx23_rtc_res;
	pdev->num_resources = ARRAY_SIZE(mx23_rtc_res);
	mxs_add_device(pdev, 3);
}
#else
static void __init mx23_init_rtc(void)
{
	;
}
#endif

#ifdef CONFIG_MXS_LRADC
struct mxs_lradc_plat_data mx23_lradc_data = {
	.vddio_voltage = BV_LRADC_CTRL4_LRADC6SELECT__CHANNEL10,
	.battery_voltage = BV_LRADC_CTRL4_LRADC7SELECT__CHANNEL7,
};

static struct resource mx23_lradc_res[] = {
	{
	 .flags = IORESOURCE_MEM,
	 .start = LRADC_PHYS_ADDR,
	 .end   = LRADC_PHYS_ADDR + 0x2000 - 1,
	 },
};

static void __init mx23_init_lradc(void)
{
	struct platform_device *pdev;

	pdev = mxs_get_device("mxs-lradc", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->resource = mx23_lradc_res;
	pdev->num_resources = ARRAY_SIZE(mx23_lradc_res);
	pdev->dev.platform_data = &mx23_lradc_data;
	mxs_add_device(pdev, 0);
}
#else
static void __init mx23_init_lradc(void)
{
	;
}
#endif

#if defined(CONFIG_KEYBOARD_MXS) || defined(CONFIG_KEYBOARD_MXS_MODULE)
static struct mxskbd_keypair keyboard_data[] = {
	{ 100, KEY_F4 },
	{ 306, KEY_F5 },
	{ 626, KEY_F6 },
	{ 932, KEY_F7 },
	{ 1260, KEY_F8 },
	{ 1584, KEY_F9 },
	{ 1907, KEY_F10 },
	{ 2207, KEY_F11 },
	{ 2525, KEY_F12 },
	{ 2831, KEY_F13},
	{ 3134, KEY_F14 },
	{ -1, 0 },
};

static struct mxs_kbd_plat_data mxs_kbd_data = {
	.keypair = keyboard_data,
	.channel = LRADC_CH1,
};

static struct resource mx23_kbd_res[] = {
	{
	 .flags = IORESOURCE_MEM,
	 .start = LRADC_PHYS_ADDR,
	 .end   = LRADC_PHYS_ADDR + 0x2000 - 1,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_LRADC_CH1,
	 .end   = IRQ_LRADC_CH1,
	 },
};

static void __init mx23_init_kbd(void)
{
	struct platform_device *pdev;
	pdev = mxs_get_device("mxs-kbd", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->resource = mx23_kbd_res;
	pdev->num_resources = ARRAY_SIZE(mx23_kbd_res);
	pdev->dev.platform_data = &mxs_kbd_data;
	mxs_add_device(pdev, 3);
}
#else
static void __init mx23_init_kbd(void)
{
	;
}
#endif

#if defined(CONFIG_TOUCHSCREEN_MXS) || defined(CONFIG_TOUCHSCREEN_MXS_MODULE)
static struct mxs_touchscreen_plat_data mx23_ts_data = {
	.x_plus_chan = LRADC_TOUCH_X_PLUS,
	.x_minus_chan = LRADC_TOUCH_X_MINUS,
	.y_plus_chan = LRADC_TOUCH_Y_PLUS,
	.y_minus_chan = LRADC_TOUCH_Y_MINUS,
	.x_plus_val = BM_LRADC_CTRL0_XPULSW,
	.x_minus_val = BF_LRADC_CTRL0_XNURSW(2),
	.y_plus_val = BF_LRADC_CTRL0_YPLLSW(1),
	.y_minus_val = BM_LRADC_CTRL0_YNLRSW,
	.x_plus_mask = BM_LRADC_CTRL0_XPULSW,
	.x_minus_mask = BM_LRADC_CTRL0_XNURSW,
	.y_plus_mask = BM_LRADC_CTRL0_YPLLSW,
	.y_minus_mask = BM_LRADC_CTRL0_YNLRSW,
};

static struct resource mx23_ts_res[] = {
	{
	 .flags = IORESOURCE_MEM,
	 .start = LRADC_PHYS_ADDR,
	 .end   = LRADC_PHYS_ADDR + 0x2000 - 1,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_LRADC_TOUCH,
	 .end   = IRQ_LRADC_TOUCH,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_LRADC_CH5,
	 .end   = IRQ_LRADC_CH5,
	 },
};

static void __init mx23_init_ts(void)
{
	struct platform_device *pdev;

	pdev = mxs_get_device("mxs-ts", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->resource = mx23_ts_res;
	pdev->num_resources = ARRAY_SIZE(mx23_ts_res);
	pdev->dev.platform_data = &mx23_ts_data;
	mxs_add_device(pdev, 3);
}
#else
static void __init mx23_init_ts(void)
{
	;
}
#endif
int __init mx23_device_init(void)
{
	mx23_init_duart();
	return 0;
}

static struct __initdata map_desc mx23_io_desc[] = {
	{
	 .virtual = MX23_SOC_IO_VIRT_BASE,
	 .pfn = __phys_to_pfn(MX23_SOC_IO_PHYS_BASE),
	 .length = MX23_SOC_IO_AREA_SIZE,
	 .type = MT_DEVICE,
	 },
};

void __init mx23_map_io(void)
{
	iotable_init(mx23_io_desc, ARRAY_SIZE(mx23_io_desc));
}

void __init mx23_irq_init(void)
{
	avic_init_irq(IO_ADDRESS(ICOLL_PHYS_ADDR), ARCH_NR_IRQS);
}

static void mx23_timer_init(void)
{
	int i, reg;
	mx23_clock_init();

	mx23_timer.clk = clk_get(NULL, "clk_32k");
	if (mx23_timer.clk == NULL || IS_ERR(mx23_timer.clk))
		return;
	__raw_writel(BM_TIMROT_ROTCTRL_SFTRST,
		     mx23_timer.base + HW_TIMROT_ROTCTRL_CLR);
	for (i = 0; i < 10000; i++) {
		reg = __raw_readl(mx23_timer.base + HW_TIMROT_ROTCTRL);
		if (!(reg & BM_TIMROT_ROTCTRL_SFTRST))
			break;
		udelay(2);
	}
	if (i >= 10000)
		return;
	__raw_writel(BM_TIMROT_ROTCTRL_CLKGATE,
		     mx23_timer.base + HW_TIMROT_ROTCTRL_CLR);

	reg = __raw_readl(mx23_timer.base + HW_TIMROT_ROTCTRL);

	mxs_nomatch_timer_init(&mx23_timer);
}

struct mxs_sys_timer mx23_timer = {
	.timer = {
		  .init = mx23_timer_init,
		  },
	.clk_sel = BV_TIMROT_TIMCTRLn_SELECT__32KHZ_XTAL,
	.base = IO_ADDRESS(TIMROT_PHYS_ADDR),
};
