/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
#ifndef __FREESCALE_OTP__
#define __FREESCALE_OTP__

#define log(a, ...) printk(KERN_INFO "[ %s : %.3d ] "a"\n", \
			__func__, __LINE__,  ## __VA_ARGS__)

static int otp_wait_busy(u32 flags);

/* IMX23 and IMX28 share most of the defination ========================= */
#if (defined(CONFIG_ARCH_MX23) || defined(CONFIG_ARCH_MX28))

#include <linux/regulator/consumer.h>
#include <mach/hardware.h>
#include <mach/device.h>
#include <mach/regs-ocotp.h>
#include <mach/regs-power.h>

#if defined(CONFIG_ARCH_MX23)
#include <mach/mx23.h>
#else
#include <mach/mx28.h>
#endif

#define REGS_OCOTP_BASE		(IO_ADDRESS(OCOTP_PHYS_ADDR))
#define BF(value, field)	(((value) << BP_##field) & BM_##field)

static unsigned long otp_hclk_saved;
static u32 otp_voltage_saved;
struct regulator *regu;

/* open the bank for the imx23/imx28 */
static int otp_read_prepare(void)
{
	int r;

	/* [1] set the HCLK */
	/* [2] check BUSY and ERROR bit */
	r = otp_wait_busy(0);
	if (r < 0)
		goto error;

	/* [3] open bank */
	__raw_writel(BM_OCOTP_CTRL_RD_BANK_OPEN,
			REGS_OCOTP_BASE + HW_OCOTP_CTRL_SET);
	udelay(10);

	/* [4] wait for BUSY */
	r = otp_wait_busy(0);
	return 0;
error:
	return r;
}

static int otp_read_post(void)
{
	/* [5] close bank */
	__raw_writel(BM_OCOTP_CTRL_RD_BANK_OPEN,
			REGS_OCOTP_BASE + HW_OCOTP_CTRL_CLR);
	return 0;
}

static int otp_write_prepare(struct fsl_otp_data *otp_data)
{
	struct clk *hclk;
	int err = 0;

	/* [1] HCLK to 24MHz. */
	hclk = clk_get(NULL, "hclk");
	if (IS_ERR(hclk)) {
		err = PTR_ERR(hclk);
		goto out;
	}
	/*
	   WARNING  ACHTUNG  UWAGA

	   the code below changes HCLK clock rate to 24M. This is
	   required to write OTP bits (7.2.2 in STMP378x Target
	   Specification), and might affect LCD operation, for example.
	   Moreover, this hacky code changes VDDIO to 2.8V; and resto-
	   res it only on otp_close(). This may affect... anything.

	   You are warned now.
	 */
	otp_hclk_saved = clk_get_rate(hclk);
	clk_set_rate(hclk, 24000);

	/* [2] The voltage is set to 2.8V */
	regu = regulator_get(NULL, otp_data->regulator_name);
	otp_voltage_saved = regulator_get_voltage(regu);
	regulator_set_voltage(regu, 2800000, 2800000);

	/* [3] wait for BUSY and ERROR */
	err = otp_wait_busy(BM_OCOTP_CTRL_RD_BANK_OPEN);
out:
	return err;
}

static int otp_write_post(void)
{
	struct clk *hclk;

	hclk = clk_get(NULL, "hclk");

	/* restore the clock and voltage */
	clk_set_rate(hclk, otp_hclk_saved);
	regulator_set_voltage(regu, otp_voltage_saved, otp_voltage_saved);
	otp_wait_busy(0);

	/* reset */
	__raw_writel(BM_OCOTP_CTRL_RELOAD_SHADOWS,
			REGS_OCOTP_BASE + HW_OCOTP_CTRL_SET);
	otp_wait_busy(BM_OCOTP_CTRL_RELOAD_SHADOWS);
	return 0;
}

static int __init map_ocotp_addr(struct platform_device *pdev)
{
	return 0;
}
static void unmap_ocotp_addr(void)
{
}

#elif defined(CONFIG_ARCH_MX5) /* IMX5 below ============================= */

#include <mach/hardware.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include "regs-ocotp-v2.h"

static void *otp_base;
#define REGS_OCOTP_BASE		((unsigned long)otp_base)
#define HW_OCOTP_CUSTn(n)	(0x00000030 + (n) * 0x10)
#define BF(value, field) 	(((value) << BP_##field) & BM_##field)

#define DEF_RELEX	(15)	/* > 10.5ns */

static int set_otp_timing(void)
{
	struct clk *apb_clk;
	unsigned long clk_rate = 0;
	unsigned long relex, sclk_count, rd_busy;
	u32 timing = 0;

	/* [1] get the clock. It needs the AHB clock,though doc writes APB.*/
	apb_clk = clk_get(NULL, "ahb_clk");
	if (IS_ERR(apb_clk)) {
		log("we can not find the clock");
		return -1;
	}
	clk_rate = clk_get_rate(apb_clk);

	/* do optimization for too many zeros */
	relex	= clk_rate / (1000000000 / DEF_RELEX) + 1;
	sclk_count = clk_rate / (1000000000 / 5000) + 1 + DEF_RELEX;
	rd_busy	= clk_rate / (1000000000 / 300)	+ 1;

	timing = BF(relex, OCOTP_TIMING_RELAX);
	timing |= BF(sclk_count, OCOTP_TIMING_SCLK_COUNT);
	timing |= BF(rd_busy, OCOTP_TIMING_RD_BUSY);

	__raw_writel(timing, REGS_OCOTP_BASE + HW_OCOTP_TIMING);
	return 0;
}

/* IMX5 does not need to open the bank anymore */
static int otp_read_prepare(void)
{
	return set_otp_timing();
}
static int otp_read_post(void)
{
	return 0;
}

static int otp_write_prepare(struct fsl_otp_data *otp_data)
{
	int ret = 0;

	/* [1] set timing */
	ret = set_otp_timing();
	if (ret)
		return ret;

	/* [2] wait */
	otp_wait_busy(0);
	return 0;
}
static int otp_write_post(void)
{
	/* Reload all the shadow registers */
	__raw_writel(BM_OCOTP_CTRL_RELOAD_SHADOWS,
			REGS_OCOTP_BASE + HW_OCOTP_CTRL_SET);
	udelay(1);
	otp_wait_busy(BM_OCOTP_CTRL_RELOAD_SHADOWS);
	return 0;
}

static int __init map_ocotp_addr(struct platform_device *pdev)
{
	struct resource *res;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	otp_base = ioremap(res->start, SZ_8K);
	if (!otp_base) {
		log("Can not remap the OTP iomem!");
		return -1;
	}
	return 0;
}

static void unmap_ocotp_addr(void)
{
	iounmap(otp_base);
	otp_base = NULL;
}
#endif /* CONFIG_ARCH_MX5 */
#endif
