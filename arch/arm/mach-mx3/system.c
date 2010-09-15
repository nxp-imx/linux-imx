/*
 * Copyright (C) 1999 ARM Limited
 * Copyright (C) 2000 Deep Blue Solutions Ltd
 * Copyright 2004-2010 Freescale Semiconductor, Inc.
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

#include <linux/clk.h>
#include <linux/io.h>
#include <mach/hardware.h>
#include <asm/proc-fns.h>
#include <asm/system.h>
#include <mach/clock.h>
#include "crm_regs.h"

/*!
 * @defgroup MSL_MX31 i.MX31 Machine Specific Layer (MSL)
 */

/*!
 * @file mach-mx3/system.c
 * @brief This file contains idle and reset functions.
 *
 * @ingroup MSL_MX31
 */

static int clks_initialized;
static struct clk *sdma_clk, *mbx_clk, *ipu_clk, *mpeg_clk, *vpu_clk, *usb_clk,
    *rtic_clk, *nfc_clk, *emi_clk;

extern int mxc_jtag_enabled;

/*!
 * This function puts the CPU into idle mode. It is called by default_idle()
 * in process.c file.
 */
void arch_idle(void)
{
	int emi_gated_off = 0;

	/*
	 * This should do all the clock switching
	 * and wait for interrupt tricks.
	 */
	if (!mxc_jtag_enabled) {
		if (clks_initialized == 0) {
			clks_initialized = 1;
			sdma_clk = clk_get(NULL, "sdma_ahb_clk");
			ipu_clk = clk_get(NULL, "ipu_clk");
			if (cpu_is_mx31()) {
				mpeg_clk = clk_get(NULL, "mpeg4_clk");
				mbx_clk = clk_get(NULL, "mbx_clk");
			} else {
				vpu_clk = clk_get(NULL, "vpu_clk");
			}
			usb_clk = clk_get(NULL, "usb_ahb_clk");
			rtic_clk = clk_get(NULL, "rtic_clk");
			nfc_clk = clk_get(NULL, "nfc_clk");
			emi_clk = clk_get(NULL, "emi_clk");
		}

		if ((clk_get_usecount(sdma_clk) == 0)
		    && (clk_get_usecount(ipu_clk) <= 1)
		    && (clk_get_usecount(usb_clk) == 0)
		    && (clk_get_usecount(rtic_clk) == 0)
		    && (clk_get_usecount(mpeg_clk) == 0)
		    && (clk_get_usecount(mbx_clk) == 0)
		    && (clk_get_usecount(nfc_clk) == 0)
		    && (clk_get_usecount(vpu_clk) == 0)) {
			emi_gated_off = 1;
			clk_disable(emi_clk);
		}

		cpu_do_idle();
		if (emi_gated_off == 1)
			clk_enable(emi_clk);

	}
}

