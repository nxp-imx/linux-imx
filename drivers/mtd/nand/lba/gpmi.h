/*
 * Freescale STMP37XX/STMP378X GPMI (General-Purpose-Media-Interface)
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
#ifndef __DRIVERS_GPMI_H
#define __DRIVERS_GPMI_H

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <mach/stmp3xxx.h>
#include <mach/dma.h>

#include <mach/gpmi.h>
#include <mach/regs-gpmi.h>
#include <mach/regs-apbh.h>
#ifdef CONFIG_MTD_NAND_GPMI_BCH
#include <mach/regs-bch.h>
#endif


struct gpmi_nand_timing {
	u8 data_setup;
	u8 data_hold;
	u8 address_setup;
	u8 dsample_time;
};

#define GPMI_DMA_MAX_CHAIN	20	/* max DMA commands in chain */

#define GPMI_CMD_BUF_SZ		10
#define GPMI_DATA_BUF_SZ	4096
#define GPMI_WRITE_BUF_SZ	4096
#define GPMI_OOB_BUF_SZ		218


struct gpmi_perchip_data {
	int                     valid;
	struct nand_chip	chip;
	struct platform_device  *dev;
	int index;

	spinlock_t lock;        /* protect chain operations */
	struct stmp37xx_circ_dma_chain chain;
	struct stmp3xxx_dma_descriptor d[GPMI_DMA_MAX_CHAIN];
	int d_tail;

	struct completion	done;

	u8			*cmd_buffer;
	dma_addr_t		cmd_buffer_handle;
	int			cmd_buffer_size, cmd_buffer_sz;

	u8			*cmdtail_buffer;
	dma_addr_t		cmdtail_buffer_handle;
	int			cmdtail_buffer_size, cmdtail_buffer_sz;

	u8			*write_buffer;
	dma_addr_t		write_buffer_handle;
	int			write_buffer_size, write_buffer_sz;

	u8			*data_buffer;
	dma_addr_t		data_buffer_handle;
	u8			*data_buffer_cptr;
	int			data_buffer_size, data_buffer_sz, bytes2read;

	u8			*oob_buffer;
	dma_addr_t		oob_buffer_handle;
	int			oob_buffer_size;

	int			cs;
	unsigned		dma_ch;

	int			ecc_oob_bytes, oob_free;

	struct gpmi_nand_timing timing;

	void *p2w, *oob2w, *p2r, *oob2r;
	size_t p2w_size, oob2w_size, p2r_size, oob2r_size;
	dma_addr_t p2w_dma, oob2w_dma, p2r_dma, oob2r_dma;
	unsigned read_memcpy:1, write_memcpy:1,
		 read_oob_memcpy:1, write_oob_memcpy:1;
};


extern struct gpmi_nand_timing gpmi_safe_timing;


#endif
