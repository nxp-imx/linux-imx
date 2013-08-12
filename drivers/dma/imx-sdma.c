/*
 * drivers/dma/imx-sdma.c
 *
 * This file contains a driver for the Freescale Smart DMA engine
 *
 * Copyright 2010 Sascha Hauer, Pengutronix <s.hauer@pengutronix.de>
 *
 * Based on code from Freescale:
 *
 * Copyright 2004-2013 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/init.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/dmaengine.h>
#include <linux/delay.h>
#include <linux/genalloc.h>

#include <asm/irq.h>
#include <mach/sdma.h>
#include <mach/dma.h>
#include <mach/hardware.h>
#include <mach/iram.h>


/* SDMA registers */
#define SDMA_H_C0PTR		0x000
#define SDMA_H_INTR		0x004
#define SDMA_H_STATSTOP		0x008
#define SDMA_H_START		0x00c
#define SDMA_H_EVTOVR		0x010
#define SDMA_H_DSPOVR		0x014
#define SDMA_H_HOSTOVR		0x018
#define SDMA_H_EVTPEND		0x01c
#define SDMA_H_DSPENBL		0x020
#define SDMA_H_RESET		0x024
#define SDMA_H_EVTERR		0x028
#define SDMA_H_INTRMSK		0x02c
#define SDMA_H_PSW		0x030
#define SDMA_H_EVTERRDBG	0x034
#define SDMA_H_CONFIG		0x038
#define SDMA_ONCE_ENB		0x040
#define SDMA_ONCE_DATA		0x044
#define SDMA_ONCE_INSTR		0x048
#define SDMA_ONCE_STAT		0x04c
#define SDMA_ONCE_CMD		0x050
#define SDMA_EVT_MIRROR		0x054
#define SDMA_ILLINSTADDR	0x058
#define SDMA_CHN0ADDR		0x05c
#define SDMA_ONCE_RTB		0x060
#define SDMA_XTRIG_CONF1	0x070
#define SDMA_XTRIG_CONF2	0x074
#define SDMA_CHNENBL0_V2	0x200
#define SDMA_CHNENBL0_V1	0x080
#define SDMA_CHNPRI_0		0x100

/*
 * Buffer descriptor status values.
 */
#define BD_DONE  0x01
#define BD_WRAP  0x02
#define BD_CONT  0x04
#define BD_INTR  0x08
#define BD_RROR  0x10
#define BD_LAST  0x20
#define BD_EXTD  0x80

/*
 * Data Node descriptor status values.
 */
#define DND_END_OF_FRAME  0x80
#define DND_END_OF_XFER   0x40
#define DND_DONE          0x20
#define DND_UNUSED        0x01

/*
 * IPCV2 descriptor status values.
 */
#define BD_IPCV2_END_OF_FRAME  0x40

#define IPCV2_MAX_NODES        50
/*
 * Error bit set in the CCB status field by the SDMA,
 * in setbd routine, in case of a transfer error
 */
#define DATA_ERROR  0x10000000

/*
 * Buffer descriptor commands.
 */
#define C0_ADDR             0x01
#define C0_LOAD             0x02
#define C0_DUMP             0x03
#define C0_SETCTX           0x07
#define C0_GETCTX           0x03
#define C0_SETDM            0x01
#define C0_SETPM            0x04
#define C0_GETDM            0x02
#define C0_GETPM            0x08
/*
 * Change endianness indicator in the BD command field
 */
#define CHANGE_ENDIANNESS   0x80

/*
 * Mode/Count of data node descriptors - IPCv2
 */
struct sdma_mode_count {
	u32 count   : 16; /* size of the buffer pointed by this BD */
	u32 status  :  8; /* E,R,I,C,W,D status bits stored here */
	u32 command :  8; /* command mostlky used for channel 0 */
};

/*
 * Buffer descriptor
 */
struct sdma_buffer_descriptor {
	struct sdma_mode_count  mode;
	u32 buffer_addr;	/* address of the buffer described */
	u32 ext_buffer_addr;	/* extended buffer address */
} __attribute__ ((packed));

/**
 * struct sdma_channel_control - Channel control Block
 *
 * @current_bd_ptr	current buffer descriptor processed
 * @base_bd_ptr		first element of buffer descriptor array
 * @unused		padding. The SDMA engine expects an array of 128 byte
 *			control blocks
 */
struct sdma_channel_control {
	u32 current_bd_ptr;
	u32 base_bd_ptr;
	u32 unused[2];
} __attribute__ ((packed));

/**
 * struct sdma_state_registers - SDMA context for a channel
 *
 * @pc:		program counter
 * @t:		test bit: status of arithmetic & test instruction
 * @rpc:	return program counter
 * @sf:		source fault while loading data
 * @spc:	loop start program counter
 * @df:		destination fault while storing data
 * @epc:	loop end program counter
 * @lm:		loop mode
 */
struct sdma_state_registers {
	u32 pc     :14;
	u32 unused1: 1;
	u32 t      : 1;
	u32 rpc    :14;
	u32 unused0: 1;
	u32 sf     : 1;
	u32 spc    :14;
	u32 unused2: 1;
	u32 df     : 1;
	u32 epc    :14;
	u32 lm     : 2;
} __attribute__ ((packed));

/**
 * struct sdma_context_data - sdma context specific to a channel
 *
 * @channel_state:	channel state bits
 * @gReg:		general registers
 * @mda:		burst dma destination address register
 * @msa:		burst dma source address register
 * @ms:			burst dma status register
 * @md:			burst dma data register
 * @pda:		peripheral dma destination address register
 * @psa:		peripheral dma source address register
 * @ps:			peripheral dma status register
 * @pd:			peripheral dma data register
 * @ca:			CRC polynomial register
 * @cs:			CRC accumulator register
 * @dda:		dedicated core destination address register
 * @dsa:		dedicated core source address register
 * @ds:			dedicated core status register
 * @dd:			dedicated core data register
 */
struct sdma_context_data {
	struct sdma_state_registers  channel_state;
	u32  gReg[8];
	u32  mda;
	u32  msa;
	u32  ms;
	u32  md;
	u32  pda;
	u32  psa;
	u32  ps;
	u32  pd;
	u32  ca;
	u32  cs;
	u32  dda;
	u32  dsa;
	u32  ds;
	u32  dd;
	u32  scratch0;
	u32  scratch1;
	u32  scratch2;
	u32  scratch3;
	u32  scratch4;
	u32  scratch5;
	u32  scratch6;
	u32  scratch7;
} __attribute__ ((packed));

#define NUM_BD (int)(PAGE_SIZE / sizeof(struct sdma_buffer_descriptor))

struct sdma_engine;

enum sdma_mode {
	SDMA_MODE_INVALID = 0,
	SDMA_MODE_LOOP,
	SDMA_MODE_NORMAL,
	SDMA_MODE_P2P,
	SDMA_MODE_NO_BD,
};

/**
 * struct sdma_channel - housekeeping for a SDMA channel
 *
 * @sdma		pointer to the SDMA engine for this channel
 * @channel		the channel number, matches dmaengine chan_id + 1
 * @direction		transfer type. Needed for setting SDMA script
 * @peripheral_type	Peripheral type. Needed for setting SDMA script
 * @event_id0		aka dma request line
 * @event_id1		for channels that use 2 events
 * @word_size		peripheral access size
 * @buf_tail		ID of the buffer that was processed
 * @done		channel completion
 * @num_bd		max NUM_BD. number of descriptors currently handling
 */
struct sdma_channel {
	struct sdma_engine		*sdma;
	unsigned int			channel;
	enum dma_transfer_direction		direction;
	enum sdma_peripheral_type	peripheral_type;
	unsigned int			event_id0;
	unsigned int			event_id1;
	enum dma_slave_buswidth		word_size;
	unsigned int			buf_tail;
	struct completion		done;
	unsigned int			num_bd;
	struct sdma_buffer_descriptor	*bd;
	dma_addr_t			bd_phys;
	unsigned int			pc_from_device;
	unsigned int			pc_to_device;
	unsigned int			device_to_device;
	unsigned int			other_script;
	unsigned int                    pc_to_pc;
	enum sdma_mode			mode;
	dma_addr_t			per_address, per_address2;
	u32				event_mask0, event_mask1;
	u32				watermark_level;
	u32				shp_addr, per_addr;
	u32				data_addr1, data_addr2;
	struct dma_chan			chan;
	spinlock_t			lock;
	struct dma_async_tx_descriptor	desc;
	dma_cookie_t			last_completed;
	enum dma_status			status;
	unsigned int			chn_count;
	unsigned int			chn_real_count;
	unsigned int			irq_handling;
};

#define MAX_DMA_CHANNELS 32
#define MXC_SDMA_DEFAULT_PRIORITY 1
#define MXC_SDMA_MIN_PRIORITY 1
#define MXC_SDMA_MAX_PRIORITY 7

#define SDMA_FIRMWARE_MAGIC 0x414d4453

/**
 * struct sdma_firmware_header - Layout of the firmware image
 *
 * @magic		"SDMA"
 * @version_major	increased whenever layout of struct sdma_script_start_addrs
 *			changes.
 * @version_minor	firmware minor version (for binary compatible changes)
 * @script_addrs_start	offset of struct sdma_script_start_addrs in this image
 * @num_script_addrs	Number of script addresses in this image
 * @ram_code_start	offset of SDMA ram image in this firmware image
 * @ram_code_size	size of SDMA ram image
 * @script_addrs	Stores the start address of the SDMA scripts
 *			(in SDMA memory space)
 */
struct sdma_firmware_header {
	u32	magic;
	u32	version_major;
	u32	version_minor;
	u32	script_addrs_start;
	u32	num_script_addrs;
	u32	ram_code_start;
	u32	ram_code_size;
};

struct sdma_engine {
	struct device			*dev;
	struct device_dma_parameters	dma_parms;
	struct sdma_channel		channel[MAX_DMA_CHANNELS];
	struct sdma_channel_control	*channel_control;
	void __iomem			*regs;
	unsigned int			version;
	unsigned int			num_events;
	struct sdma_context_data	*context;
	dma_addr_t			context_phys;
	struct dma_device		dma_device;
	struct clk			*clk;
	struct sdma_script_start_addrs	*script_addrs;
	spinlock_t			irq_reg_lock;
	spinlock_t			channel_0_lock;
};

#define SDMA_H_CONFIG_DSPDMA	(1 << 12) /* indicates if the DSPDMA is used */
#define SDMA_H_CONFIG_RTD_PINS	(1 << 11) /* indicates if Real-Time Debug pins are enabled */
#define SDMA_H_CONFIG_ACR	(1 << 4)  /* indicates if AHB freq /core freq = 2 or 1 */
#define SDMA_H_CONFIG_CSM	(3)       /* indicates which context switch mode is selected*/


static inline u32 chnenbl_ofs(struct sdma_engine *sdma, unsigned int event)
{
	u32 chnenbl0 = (sdma->version == 2 ? SDMA_CHNENBL0_V2 : SDMA_CHNENBL0_V1);

	return chnenbl0 + event * 4;
}

static int sdma_config_ownership(struct sdma_channel *sdmac,
		bool event_override, bool mcu_override, bool dsp_override)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;
	u32 evt, mcu, dsp;

	if (event_override && mcu_override && dsp_override)
		return -EINVAL;

	evt = readl_relaxed(sdma->regs + SDMA_H_EVTOVR);
	mcu = readl_relaxed(sdma->regs + SDMA_H_HOSTOVR);
	dsp = readl_relaxed(sdma->regs + SDMA_H_DSPOVR);

	if (dsp_override)
		dsp &= ~(1 << channel);
	else
		dsp |= (1 << channel);

	if (event_override)
		evt &= ~(1 << channel);
	else
		evt |= (1 << channel);

	if (mcu_override)
		mcu &= ~(1 << channel);
	else
		mcu |= (1 << channel);

	writel_relaxed(evt, sdma->regs + SDMA_H_EVTOVR);
	writel_relaxed(mcu, sdma->regs + SDMA_H_HOSTOVR);
	writel_relaxed(dsp, sdma->regs + SDMA_H_DSPOVR);

	return 0;
}

/*
 * sdma_run_channel - run a channel and wait till it's done
 */
static int sdma_run_channel(struct sdma_channel *sdmac)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;
	unsigned long timeout = 1000;
	int ret;

	writel(1 << channel, sdma->regs + SDMA_H_START);

	while (!(ret = readl_relaxed(sdma->regs + SDMA_H_INTR) & 1)) {
		if (timeout-- <= 0)
			break;
		udelay(1);
	}

	if (ret) {
		/* Clear the interrupt status */
		writel_relaxed(ret, sdma->regs + SDMA_H_INTR);
	} else {
		dev_err(sdma->dev, "Timeout waiting for CH0 ready\n");
	}

	return ret ? 0 : -ETIMEDOUT;
}

static int sdma_load_script(struct sdma_engine *sdma, void *buf, int size,
		u32 address)
{
	struct sdma_buffer_descriptor *bd0 = sdma->channel[0].bd;
	void *buf_virt;
	dma_addr_t buf_phys;
	unsigned long flags;
	int ret;

#ifdef CONFIG_SDMA_IRAM
	buf_virt = iram_alloc(size, (unsigned long *)&buf_phys);
#else
	buf_virt = dma_alloc_coherent(NULL,
			size,
			&buf_phys, GFP_KERNEL);
#endif
	if (!buf_virt)
		return -ENOMEM;

	spin_lock_irqsave(&sdma->channel_0_lock, flags);

	bd0->mode.command = C0_SETPM;
	bd0->mode.status = BD_DONE | BD_INTR | BD_WRAP | BD_EXTD;
	bd0->mode.count = size / 2;
	bd0->buffer_addr = buf_phys;
	bd0->ext_buffer_addr = address;

	memcpy(buf_virt, buf, size);

	ret = sdma_run_channel(&sdma->channel[0]);

	spin_unlock_irqrestore(&sdma->channel_0_lock, flags);
#ifdef CONFIG_SDMA_IRAM
	iram_free(buf_phys, size);
#else
	dma_free_coherent(NULL, size, buf_virt, buf_phys);
#endif

	return ret;
}

static void sdma_event_enable(struct sdma_channel *sdmac, unsigned int event)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;
	u32 val;
	u32 chnenbl = chnenbl_ofs(sdma, event);

	val = readl_relaxed(sdma->regs + chnenbl);
	val |= (1 << channel);
	writel_relaxed(val, sdma->regs + chnenbl);
}

static void sdma_event_disable(struct sdma_channel *sdmac, unsigned int event)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;
	u32 chnenbl = chnenbl_ofs(sdma, event);
	u32 val;

	val = readl_relaxed(sdma->regs + chnenbl);
	val &= ~(1 << channel);
	writel_relaxed(val, sdma->regs + chnenbl);
}

static void sdma_handle_channel_loop(struct sdma_channel *sdmac)
{
	struct sdma_buffer_descriptor *bd;
	/*
	 * loop mode. Iterate over descriptors, re-setup them and
	 * call callback function.
	 */
	while (1) {
		bd = &sdmac->bd[sdmac->buf_tail];

		if (bd->mode.status & BD_DONE)
			break;

		if (bd->mode.status & BD_RROR)
			sdmac->status = DMA_ERROR;
		else
			sdmac->status = DMA_IN_PROGRESS;

		bd->mode.status |= BD_DONE;
		sdmac->buf_tail++;
		sdmac->buf_tail %= sdmac->num_bd;

		if (sdmac->desc.callback)
			sdmac->desc.callback(sdmac->desc.callback_param);
	}
}

static void mxc_sdma_handle_channel_normal(struct sdma_channel *sdmac)
{
	struct sdma_buffer_descriptor *bd;
	int i, error = 0;

	sdmac->chn_real_count = 0;
	/*
	 * non loop mode. Iterate over all descriptors, collect
	 * errors and call callback function
	 */
	for (i = 0; i < sdmac->num_bd; i++) {
		bd = &sdmac->bd[i];

		 if (bd->mode.status & (BD_DONE | BD_RROR))
			error = -EIO;
		 sdmac->chn_real_count += bd->mode.count;
	}

	if (error)
		sdmac->status = DMA_ERROR;
	else
		sdmac->status = DMA_SUCCESS;

	sdmac->last_completed = sdmac->desc.cookie;
	if (sdmac->desc.callback)
		sdmac->desc.callback(sdmac->desc.callback_param);
}


static void sdma_handle_other_intr(struct sdma_channel *sdmac)
{
	sdmac->last_completed = sdmac->desc.cookie;

	if (sdmac->desc.callback)
		sdmac->desc.callback(sdmac->desc.callback_param);
}

static void mxc_sdma_handle_channel(struct sdma_channel *sdmac)
{
	complete(&sdmac->done);

	switch (sdmac->mode) {
	case SDMA_MODE_LOOP:
		sdma_handle_channel_loop(sdmac);
		break;
	case SDMA_MODE_NORMAL:
		mxc_sdma_handle_channel_normal(sdmac);
		break;
	case SDMA_MODE_NO_BD:
		sdma_handle_other_intr(sdmac);
		break;
	default:
		pr_err("Unvalid SDMA MODE!\n");
		break;
	}
}

static irqreturn_t sdma_int_handler(int irq, void *dev_id)
{
	struct sdma_engine *sdma = dev_id;
	struct sdma_channel *sdmac;
	unsigned long flag;
	int channel;
	u32 stat, stat_bak;

	spin_lock_irqsave(&sdma->irq_reg_lock, flag);
	stat = readl_relaxed(sdma->regs + SDMA_H_INTR);
	/* not interested in channel 0 interrupts */
	stat &= ~1;
	writel_relaxed(stat, sdma->regs + SDMA_H_INTR);
	spin_unlock_irqrestore(&sdma->irq_reg_lock, flag);

	stat_bak = stat;
	while (stat_bak) {
		channel = fls(stat_bak) - 1;
		sdmac = &sdma->channel[channel];
		sdmac->irq_handling = 1;
		stat_bak &= ~(1 << channel);
	}

	while (stat) {
		channel = fls(stat) - 1;
		sdmac = &sdma->channel[channel];

		if (sdmac->irq_handling)
			mxc_sdma_handle_channel(sdmac);

		stat &= ~(1 << channel);
		sdmac->irq_handling = 0;
	}

	return IRQ_HANDLED;
}

/*
 * sets the pc of SDMA script according to the peripheral type
 */
static void sdma_get_pc(struct sdma_channel *sdmac,
		enum sdma_peripheral_type peripheral_type)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int per_2_emi = 0, emi_2_per = 0;
	/*
	 * These are needed once we start to support transfers between
	 * two peripherals or memory-to-memory transfers
	 */
	int per_2_per = 0, emi_2_emi = 0;
	int other = 0;

	sdmac->pc_from_device = 0;
	sdmac->pc_to_device = 0;
	sdmac->device_to_device = 0;
	sdmac->other_script = 0;
	sdmac->pc_to_pc = 0;

	switch (peripheral_type) {
	case IMX_DMATYPE_MEMORY:
		emi_2_emi = sdma->script_addrs->ap_2_ap_addr;
		break;
	case IMX_DMATYPE_DSP:
		emi_2_per = sdma->script_addrs->bp_2_ap_addr;
		per_2_emi = sdma->script_addrs->ap_2_bp_addr;
		break;
	case IMX_DMATYPE_FIRI:
		per_2_emi = sdma->script_addrs->firi_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_firi_addr;
		break;
	case IMX_DMATYPE_UART:
		per_2_emi = sdma->script_addrs->uart_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_app_addr;
		break;
	case IMX_DMATYPE_UART_SP:
		per_2_emi = sdma->script_addrs->uartsh_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_shp_addr;
		break;
	case IMX_DMATYPE_ATA:
		per_2_emi = sdma->script_addrs->ata_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_ata_addr;
		break;
	case IMX_DMATYPE_CSPI:
	case IMX_DMATYPE_EXT:
	case IMX_DMATYPE_SSI:
		per_2_emi = sdma->script_addrs->app_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_app_addr;
		break;
	case IMX_DMATYPE_SSI_SP:
		per_2_emi = sdma->script_addrs->ssish_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_ssish_addr;
		break;
	case IMX_DMATYPE_MMC:
	case IMX_DMATYPE_SDHC:
	case IMX_DMATYPE_CSPI_SP:
	case IMX_DMATYPE_ESAI:
	case IMX_DMATYPE_MSHC_SP:
		per_2_emi = sdma->script_addrs->shp_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_shp_addr;
		break;
	case IMX_DMATYPE_ASRC:
		per_2_emi = sdma->script_addrs->shp_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_shp_addr;
		per_2_per = sdma->script_addrs->per_2_per_addr;
		break;
	case IMX_DMATYPE_MSHC:
		per_2_emi = sdma->script_addrs->mshc_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_mshc_addr;
		break;
	case IMX_DMATYPE_CCM:
		per_2_emi = sdma->script_addrs->dptc_dvfs_addr;
		break;
	case IMX_DMATYPE_SPDIF:
		per_2_emi = sdma->script_addrs->spdif_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_spdif_addr;
		break;
	case IMX_DMATYPE_IPU_MEMORY:
		emi_2_per = sdma->script_addrs->ext_mem_2_ipu_addr;
		break;
	case IMX_DMATYPE_HDMI:
		other = sdma->script_addrs->hdmi_dma_addr;
	default:
		break;
	}

	sdmac->pc_from_device = per_2_emi;
	sdmac->pc_to_device = emi_2_per;
	sdmac->device_to_device = per_2_per;
	sdmac->other_script = other;
	sdmac->pc_to_pc = emi_2_emi;
}

static int sdma_set_context_reg(struct sdma_channel *sdmac,
				struct sdma_context_data *context)
{
	switch (sdmac->peripheral_type) {
	case IMX_DMATYPE_HDMI:
		context->gReg[4] = sdmac->data_addr1;
		context->gReg[6] = sdmac->data_addr2;
		break;
	default:
		context->gReg[0] = sdmac->event_mask1;
		context->gReg[1] = sdmac->event_mask0;
		context->gReg[2] = sdmac->per_addr;
		context->gReg[6] = sdmac->shp_addr;
		context->gReg[7] = sdmac->watermark_level;
		break;
	}

	return 0;
}

static int sdma_load_context(struct sdma_channel *sdmac)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;
	int load_address;
	struct sdma_context_data *context = sdma->context;
	struct sdma_buffer_descriptor *bd0 = sdma->channel[0].bd;
	int ret;
	unsigned long flags;


	if (sdmac->direction == DMA_DEV_TO_MEM)
		load_address = sdmac->pc_from_device;
	else if (sdmac->direction == DMA_DEV_TO_DEV)
		load_address = sdmac->device_to_device;
	else if (sdmac->direction == DMA_MEM_TO_DEV)
		load_address = sdmac->pc_to_device;
	else if (sdmac->direction == DMA_MEM_TO_MEM)
		load_address = sdmac->pc_to_pc;
	else
		load_address = sdmac->other_script;


	if (load_address < 0)
		return load_address;

	dev_dbg(sdma->dev, "load_address = %d\n", load_address);
	dev_dbg(sdma->dev, "wml = 0x%08x\n", sdmac->watermark_level);
	dev_dbg(sdma->dev, "shp_addr = 0x%08x\n", sdmac->shp_addr);
	dev_dbg(sdma->dev, "per_addr = 0x%08x\n", sdmac->per_addr);
	dev_dbg(sdma->dev, "event_mask0 = 0x%08x\n", sdmac->event_mask0);
	dev_dbg(sdma->dev, "event_mask1 = 0x%08x\n", sdmac->event_mask1);

	spin_lock_irqsave(&sdma->channel_0_lock, flags);
	memset(context, 0, sizeof(*context));
	context->channel_state.pc = load_address;

	/* Send by context the event mask,base address for peripheral
	 * and watermark level
	 */
	sdma_set_context_reg(sdmac, context);

	bd0->mode.command = C0_SETDM;
	bd0->mode.status = BD_DONE | BD_INTR | BD_WRAP | BD_EXTD;
	bd0->mode.count = sizeof(*context) / 4;
	bd0->buffer_addr = sdma->context_phys;
	bd0->ext_buffer_addr = 2048 + (sizeof(*context) / 4) * channel;

	ret = sdma_run_channel(&sdma->channel[0]);

	spin_unlock_irqrestore(&sdma->channel_0_lock, flags);
	return ret;
}

static void sdma_disable_channel(struct sdma_channel *sdmac)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;

	writel_relaxed(1 << channel, sdma->regs + SDMA_H_STATSTOP);
	sdmac->status = DMA_ERROR;
}

static int sdma_set_chan_private_data(struct sdma_channel *sdmac)
{
	struct sdma_engine *sdma = sdmac->sdma;
	struct imx_dma_data *data = sdmac->chan.private;

	sdmac->shp_addr = 0;
	sdmac->per_addr = 0;
	sdmac->data_addr1 = 0;
	sdmac->data_addr2 = 0;


	if (sdmac->direction == DMA_DEV_TO_DEV) {
		sdmac->per_addr = sdmac->per_address;
		sdmac->shp_addr = sdmac->per_address2;
	} else if (sdmac->direction == DMA_TRANS_NONE) {
		switch (sdmac->peripheral_type) {
		case IMX_DMATYPE_HDMI:
			sdmac->data_addr1 = *(u32 *)data->private;;
			sdmac->data_addr2 = *((u32 *)data->private + 1);
			break;
		default:
			dev_dbg(sdma->dev,
			"periphal type not support for DMA_TRANS_NONE!\n");
			break;
		}
	} else {
		sdmac->shp_addr = sdmac->per_address;
	}

	return 0;
}

static int sdma_config_channel(struct sdma_channel *sdmac)
{
	int ret;

	sdma_disable_channel(sdmac);

	sdmac->event_mask0 = 0;
	sdmac->event_mask1 = 0;

	if (sdmac->event_id0)
		sdma_event_enable(sdmac, sdmac->event_id0);

	if (sdmac->event_id1)
		sdma_event_enable(sdmac, sdmac->event_id1);

	switch (sdmac->peripheral_type) {
	case IMX_DMATYPE_DSP:
		sdma_config_ownership(sdmac, false, true, true);
		break;
	case IMX_DMATYPE_MEMORY:
		sdma_config_ownership(sdmac, false, true, false);
		break;
	default:
		sdma_config_ownership(sdmac, true, true, false);
		break;
	}

	sdma_get_pc(sdmac, sdmac->peripheral_type);

	if ((sdmac->peripheral_type != IMX_DMATYPE_MEMORY) &&
			(sdmac->peripheral_type != IMX_DMATYPE_DSP) &&
			(sdmac->peripheral_type != IMX_DMATYPE_HDMI)) {
		/* Handle multiple event channels differently */
		if (sdmac->event_id1) {
			if (sdmac->event_id0 > 31) {
				sdmac->watermark_level |= 1 << 28;
				sdmac->event_mask0 |= 0;
				sdmac->event_mask1 |=
					1 << ((sdmac->event_id0)%32);
			} else {
				sdmac->event_mask0 |=
					1 << ((sdmac->event_id0)%32);
				sdmac->event_mask1 |= 0;
			}
			if (sdmac->event_id1 > 31) {
				sdmac->watermark_level |= 1 << 29;
				sdmac->event_mask0 |= 0;
				sdmac->event_mask1 |=
					1 << ((sdmac->event_id1)%32);
			} else {
				sdmac->event_mask0 |=
					1 << ((sdmac->event_id1)%32);
				sdmac->event_mask1 |= 0;
			}
			sdmac->watermark_level |= (unsigned int)(3<<11);
			sdmac->watermark_level |= (unsigned int)(1<<31);
			sdmac->watermark_level |= (unsigned int)(2<<24);
		} else {
			if (sdmac->event_id0 > 31) {
				sdmac->event_mask0 = 0;
				sdmac->event_mask1 =
					1 << ((sdmac->event_id0)%32);
			} else {
				sdmac->event_mask0 =
					1 << ((sdmac->event_id0)%32);
				sdmac->event_mask1 = 0;
			}
		}
		/* Watermark Level */
		sdmac->watermark_level |= sdmac->watermark_level;
	} else {
		sdmac->watermark_level = 0; /* FIXME: M3_BASE_ADDRESS */
	}
	sdma_set_chan_private_data(sdmac);

	ret = sdma_load_context(sdmac);

	return ret;
}

static int sdma_set_channel_priority(struct sdma_channel *sdmac,
		unsigned int priority)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;

	if (priority < MXC_SDMA_MIN_PRIORITY
	    || priority > MXC_SDMA_MAX_PRIORITY) {
		return -EINVAL;
	}

	writel_relaxed(priority, sdma->regs + SDMA_CHNPRI_0 + 4 * channel);

	return 0;
}

static int sdma_request_channel(struct sdma_channel *sdmac)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;
	int ret = -EBUSY;

#ifdef CONFIG_SDMA_IRAM
	sdmac->bd = iram_alloc(PAGE_SIZE, (unsigned long *)&sdmac->bd_phys);
#else
	sdmac->bd = dma_alloc_noncached(NULL, PAGE_SIZE, &sdmac->bd_phys, GFP_KERNEL);
#endif
	if (!sdmac->bd) {
		ret = -ENOMEM;
		goto out;
	}

	memset(sdmac->bd, 0, PAGE_SIZE);

	sdma->channel_control[channel].base_bd_ptr = sdmac->bd_phys;
	sdma->channel_control[channel].current_bd_ptr = sdmac->bd_phys;

	clk_enable(sdma->clk);

	sdma_set_channel_priority(sdmac, MXC_SDMA_DEFAULT_PRIORITY);

	init_completion(&sdmac->done);

	sdmac->buf_tail = 0;

	sdmac->irq_handling = 0;

	return 0;
out:

	return ret;
}

static void sdma_enable_channel(struct sdma_engine *sdma, int channel)
{
	wmb();
	writel(1 << channel, sdma->regs + SDMA_H_START);
}

static dma_cookie_t sdma_assign_cookie(struct sdma_channel *sdmac)
{
	dma_cookie_t cookie = sdmac->chan.cookie;

	if (++cookie < 0)
		cookie = 1;

	sdmac->chan.cookie = cookie;
	sdmac->desc.cookie = cookie;

	return cookie;
}

static struct sdma_channel *to_sdma_chan(struct dma_chan *chan)
{
	return container_of(chan, struct sdma_channel, chan);
}

static dma_cookie_t sdma_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct sdma_channel *sdmac = to_sdma_chan(tx->chan);
	struct sdma_engine *sdma = sdmac->sdma;
	dma_cookie_t cookie;
	unsigned long flag;

	spin_lock_irqsave(&sdmac->lock, flag);

	cookie = sdma_assign_cookie(sdmac);

	sdma_enable_channel(sdma, sdmac->channel);

	spin_unlock_irqrestore(&sdmac->lock, flag);

	return cookie;
}

static int sdma_alloc_chan_resources(struct dma_chan *chan)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	struct imx_dma_data *data = chan->private;
	int prio, ret;

	if (!data)
		return -EINVAL;

	switch (data->priority) {
	case DMA_PRIO_HIGH:
		prio = 3;
		break;
	case DMA_PRIO_MEDIUM:
		prio = 2;
		break;
	case DMA_PRIO_LOW:
	default:
		prio = 1;
		break;
	}

	sdmac->peripheral_type = data->peripheral_type;
	sdmac->event_id0 = data->dma_request;
	if (data->dma_request_p2p > 0)
		sdmac->event_id1 = data->dma_request_p2p;
	else
		sdmac->event_id1 = 0;
	ret = sdma_request_channel(sdmac);
	if (ret)
		return ret;

	ret = sdma_set_channel_priority(sdmac, prio);
	if (ret)
		return ret;

	dma_async_tx_descriptor_init(&sdmac->desc, chan);
	sdmac->desc.tx_submit = sdma_tx_submit;
	/* txd.flags will be overwritten in prep funcs */
	sdmac->desc.flags = DMA_CTRL_ACK;

	/* Set SDMA channel mode to unvalid to avoid misconfig */
	sdmac->mode = SDMA_MODE_INVALID;

	return 0;
}

static void sdma_irq_pending_check(struct sdma_channel *sdmac)
{
	struct sdma_engine *sdma = sdmac->sdma;
	unsigned long flag;
	u32 stat;

	spin_lock_irqsave(&sdma->irq_reg_lock, flag);
	stat = readl_relaxed(sdma->regs + SDMA_H_INTR);

	/*Check if the current channel's IRQ hasn't been responded*/
	if (stat & (1 << sdmac->channel)) {
		/*Handle the irq manually*/
		writel_relaxed(1 << sdmac->channel, sdma->regs + SDMA_H_INTR);
		spin_unlock_irqrestore(&sdma->irq_reg_lock, flag);

		/*Prevent irq_handler from doing handle_channel() again*/
		sdmac->irq_handling = 0;
		mxc_sdma_handle_channel(sdmac);
	} else {
		spin_unlock_irqrestore(&sdma->irq_reg_lock, flag);
	}

	/*Wait here until irq_handler's finished*/
	while (sdmac->irq_handling)
		udelay(100);
}

static void sdma_free_chan_resources(struct dma_chan *chan)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	struct sdma_engine *sdma = sdmac->sdma;

	/*Check if irq to the channel is still pending*/
	sdma_irq_pending_check(sdmac);

	sdma_disable_channel(sdmac);

	if (sdmac->event_id0)
		sdma_event_disable(sdmac, sdmac->event_id0);
	if (sdmac->event_id1)
		sdma_event_disable(sdmac, sdmac->event_id1);

	sdmac->event_id0 = 0;
	sdmac->event_id1 = 0;

	sdma_set_channel_priority(sdmac, 0);

#ifdef CONFIG_SDMA_IRAM
	iram_free(sdmac->bd_phys, PAGE_SIZE);
#else
	dma_free_coherent(NULL, PAGE_SIZE, sdmac->bd, sdmac->bd_phys);
#endif
	clk_disable(sdma->clk);
}

static struct dma_async_tx_descriptor *sdma_prep_slave_sg(
		struct dma_chan *chan, struct scatterlist *sgl,
		unsigned int sg_len, enum dma_transfer_direction direction,
		unsigned long flags)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	struct sdma_engine *sdma = sdmac->sdma;
	int ret, i, count;
	int channel = sdmac->channel;
	struct scatterlist *sg;

	/*
	 * For SDMA M2M use, we need 2 scatterlists, the src addresses are
	 * stored in the first sg, and the dst addresses are stored in the
	 * second sg. In the former code, when the first sg entered 'sdma_
	 * prep_slave_sg', 'sdmac->status' would be set to 'DMA_IN_PROGRESS',
	 * and the second sg would return 'NULL' when entered 'sdma_prep_slave
	 * _sg'. To avoid this error, in the code, we check if for M2M use,
	 * the second sg will not return 'NULL' when enters 'sdma_prep_slave
	 * _sg'.
	 */
	if (!((direction == DMA_MEM_TO_MEM) && (flags == 0))) {
		if (sdmac->status == DMA_IN_PROGRESS)
			return NULL;
	}
	sdmac->status = DMA_IN_PROGRESS;

	sdmac->mode = SDMA_MODE_NORMAL;

	dev_dbg(sdma->dev, "setting up %d entries for channel %d.\n",
			sg_len, channel);

	sdmac->direction = direction;
	ret = sdma_load_context(sdmac);
	if (ret)
		goto err_out;

	if (sg_len > NUM_BD) {
		dev_err(sdma->dev, "SDMA channel %d: maximum number of sg exceeded: %d > %d\n",
				channel, sg_len, NUM_BD);
		ret = -EINVAL;
		goto err_out;
	}

	sdmac->chn_count = 0;
	for_each_sg(sgl, sg, sg_len, i) {
		struct sdma_buffer_descriptor *bd = &sdmac->bd[i];
		int param;

		if (sdmac->direction == DMA_MEM_TO_MEM) {
			if (flags == 1)
				bd->buffer_addr = sg->dma_address;
			if (flags == 0)
				bd->ext_buffer_addr = sg->dma_address;
		} else
		bd->buffer_addr = sg->dma_address;

		count = sg->length;

		if (count > 0xffff) {
			dev_err(sdma->dev, "SDMA channel %d: maximum bytes for sg entry exceeded: %d > %d\n",
					channel, count, 0xffff);
			ret = -EINVAL;
			goto err_out;
		}

		bd->mode.count = count;
		sdmac->chn_count += count;

		if (sdmac->word_size > DMA_SLAVE_BUSWIDTH_4_BYTES) {
			ret =  -EINVAL;
			goto err_out;
		}

		switch (sdmac->word_size) {
		case DMA_SLAVE_BUSWIDTH_4_BYTES:
			bd->mode.command = 0;
			if (count & 3 || sg->dma_address & 3)
				return NULL;
			break;
		case DMA_SLAVE_BUSWIDTH_2_BYTES:
			bd->mode.command = 2;
			if (count & 1 || sg->dma_address & 1)
				return NULL;
			break;
		case DMA_SLAVE_BUSWIDTH_1_BYTE:
			bd->mode.command = 1;
			break;
		default:
			return NULL;
		}

		param = BD_DONE | BD_EXTD | BD_CONT;

		if (i + 1 == sg_len) {
			param |= BD_INTR;
			param |= BD_LAST;
			param &= ~BD_CONT;
		}

		dev_dbg(sdma->dev, "entry %d: count: %d dma: 0x%08x %s%s\n",
				i, count, sg->dma_address,
				param & BD_WRAP ? "wrap" : "",
				param & BD_INTR ? " intr" : "");

		bd->mode.status = param;
	}

	sdmac->num_bd = sg_len;
	sdma->channel_control[channel].current_bd_ptr = sdmac->bd_phys;

	return &sdmac->desc;
err_out:
	sdmac->status = DMA_ERROR;
	return NULL;
}

static struct dma_async_tx_descriptor *sdma_prep_dma_cyclic(
		struct dma_chan *chan, dma_addr_t dma_addr, size_t buf_len,
		size_t period_len, enum dma_transfer_direction direction)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	struct sdma_engine *sdma = sdmac->sdma;
	int num_periods;
	int channel = sdmac->channel;
	int ret, i = 0, buf = 0;

	dev_dbg(sdma->dev, "%s channel: %d\n", __func__, channel);

	if (sdmac->status == DMA_IN_PROGRESS)
		return NULL;

	sdmac->status = DMA_IN_PROGRESS;
	sdmac->direction = direction;

	switch (sdmac->direction) {
	case DMA_DEV_TO_DEV:
		sdmac->mode = SDMA_MODE_P2P;
		break;
	case DMA_TRANS_NONE:
		sdmac->mode = SDMA_MODE_NO_BD;
		break;
	case DMA_MEM_TO_DEV:
	case DMA_DEV_TO_MEM:
		sdmac->mode = SDMA_MODE_LOOP;
		break;
	default:
		pr_err("SDMA direction is not support!");
		return NULL;
	}

	ret = sdma_load_context(sdmac);
	if (ret)
		goto err_out;

	if (period_len)
		num_periods = buf_len / period_len;
	else
		return &sdmac->desc;

	if (num_periods > NUM_BD) {
		dev_err(sdma->dev, "SDMA channel %d: maximum number of sg exceeded: %d > %d\n",
				channel, num_periods, NUM_BD);
		goto err_out;
	}

	if (period_len > 0xffff) {
		dev_err(sdma->dev, "SDMA channel %d: maximum period size exceeded: %d > %d\n",
				channel, period_len, 0xffff);
		goto err_out;
	}

	while (buf < buf_len) {
		struct sdma_buffer_descriptor *bd = &sdmac->bd[i];
		int param;

		bd->buffer_addr = dma_addr;

		bd->mode.count = period_len;

		if (sdmac->word_size > DMA_SLAVE_BUSWIDTH_4_BYTES)
			goto err_out;
		if (sdmac->word_size == DMA_SLAVE_BUSWIDTH_4_BYTES)
			bd->mode.command = 0;
		else
			bd->mode.command = sdmac->word_size;

		param = BD_DONE | BD_EXTD | BD_CONT | BD_INTR;
		if (i + 1 == num_periods)
			param |= BD_WRAP;

		dev_dbg(sdma->dev, "entry %d: count: %d dma: 0x%08x %s%s\n",
				i, period_len, dma_addr,
				param & BD_WRAP ? "wrap" : "",
				param & BD_INTR ? " intr" : "");

		bd->mode.status = param;

		dma_addr += period_len;
		buf += period_len;

		i++;
	}

	sdmac->num_bd = num_periods;
	sdma->channel_control[channel].current_bd_ptr = sdmac->bd_phys;

	return &sdmac->desc;
err_out:
	sdmac->status = DMA_ERROR;
	return NULL;
}

static int sdma_control(struct dma_chan *chan, enum dma_ctrl_cmd cmd,
		unsigned long arg)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	struct dma_slave_config *dmaengine_cfg = (void *)arg;

	switch (cmd) {
	case DMA_TERMINATE_ALL:
		sdma_disable_channel(sdmac);
		return 0;
	case DMA_SLAVE_CONFIG:
		if (dmaengine_cfg->direction == DMA_DEV_TO_DEV) {
			sdmac->per_address = dmaengine_cfg->src_addr;
			sdmac->per_address2 = dmaengine_cfg->dst_addr;
			sdmac->watermark_level = 0;
			sdmac->watermark_level |=
				dmaengine_cfg->src_maxburst;
			sdmac->watermark_level |=
				dmaengine_cfg->dst_maxburst << 16;
			sdmac->word_size = dmaengine_cfg->dst_addr_width;
		} else if (dmaengine_cfg->direction == DMA_DEV_TO_MEM) {
			sdmac->per_address = dmaengine_cfg->src_addr;
			sdmac->watermark_level = dmaengine_cfg->src_maxburst;
			sdmac->word_size = dmaengine_cfg->src_addr_width;
		} else if (dmaengine_cfg->direction == DMA_MEM_TO_DEV) {
			sdmac->per_address = dmaengine_cfg->dst_addr;
			sdmac->watermark_level = dmaengine_cfg->dst_maxburst;
			sdmac->word_size = dmaengine_cfg->dst_addr_width;
		} else if (dmaengine_cfg->direction == DMA_MEM_TO_MEM) {
			sdmac->word_size = dmaengine_cfg->dst_addr_width;
		}
		sdmac->direction = dmaengine_cfg->direction;
		return sdma_config_channel(sdmac);
	default:
		return -ENOSYS;
	}

	return -EINVAL;
}

static enum dma_status sdma_tx_status(struct dma_chan *chan,
					    dma_cookie_t cookie,
					    struct dma_tx_state *txstate)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	dma_cookie_t last_used;

	last_used = chan->cookie;

	dma_set_tx_state(txstate, sdmac->last_completed, last_used,
			sdmac->chn_count - sdmac->chn_real_count);

	return sdmac->status;
}

static void sdma_issue_pending(struct dma_chan *chan)
{
	/*
	 * Nothing to do. We only have a single descriptor
	 */
}

void sdma_set_event_pending(struct dma_chan *chan)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	struct sdma_engine *sdma = sdmac->sdma;
	u32 reg;
	u32 channel;

	channel = sdmac->channel;
	reg = readl_relaxed(sdma->regs + SDMA_H_EVTPEND);
	reg |= 1 << channel;
	writel_relaxed(reg, sdma->regs + SDMA_H_EVTPEND);

	return;
}
EXPORT_SYMBOL(sdma_set_event_pending);

#define SDMA_SCRIPT_ADDRS_ARRAY_SIZE_V1	38

static void sdma_add_scripts(struct sdma_engine *sdma,
		const struct sdma_script_start_addrs *addr)
{
	s32 *addr_arr = (u32 *)addr;
	s32 *saddr_arr = (u32 *)sdma->script_addrs;
	int i;

	for (i = 0; i < SDMA_SCRIPT_ADDRS_ARRAY_SIZE_V1; i++)
		if (addr_arr[i] > 0)
			saddr_arr[i] = addr_arr[i];
}

static int __init sdma_get_firmware(struct sdma_engine *sdma,
		const char *cpu_name, int to_version)
{
	const struct firmware *fw;
	char *fwname;
	const struct sdma_firmware_header *header;
	int ret;
	const struct sdma_script_start_addrs *addr;
	unsigned short *ram_code;

	fwname = kasprintf(GFP_KERNEL, "imx/sdma/sdma-%s-to%d.bin",
				cpu_name, to_version);
	if (!fwname)
		return -ENOMEM;

	ret = request_firmware(&fw, fwname, sdma->dev);
	if (ret) {
		kfree(fwname);
		return ret;
	}
	kfree(fwname);

	if (fw->size < sizeof(*header))
		goto err_firmware;

	header = (struct sdma_firmware_header *)fw->data;

	if (header->magic != SDMA_FIRMWARE_MAGIC)
		goto err_firmware;
	if (header->ram_code_start + header->ram_code_size > fw->size)
		goto err_firmware;

	addr = (void *)header + header->script_addrs_start;
	ram_code = (void *)header + header->ram_code_start;

	/* download the RAM image for SDMA */
	sdma_load_script(sdma, ram_code,
			header->ram_code_size,
			addr->ram_code_start_addr);
	clk_disable(sdma->clk);

	sdma_add_scripts(sdma, addr);

	dev_info(sdma->dev, "loaded firmware %d.%d\n",
			header->version_major,
			header->version_minor);

err_firmware:
	release_firmware(fw);

	return ret;
}

static int __init sdma_init(struct sdma_engine *sdma)
{
	int i, ret;
	dma_addr_t ccb_phys;

	switch (sdma->version) {
	case 1:
		sdma->num_events = 32;
		break;
	case 2:
		sdma->num_events = 48;
		break;
	default:
		dev_err(sdma->dev, "Unknown version %d. aborting\n", sdma->version);
		return -ENODEV;
	}

	clk_enable(sdma->clk);

	/* Be sure SDMA has not started yet */
	writel_relaxed(0, sdma->regs + SDMA_H_C0PTR);

#ifdef CONFIG_SDMA_IRAM
	sdma->channel_control = iram_alloc(MAX_DMA_CHANNELS *
			sizeof(struct sdma_channel_control)
			+ sizeof(struct sdma_context_data),
			(unsigned long *)&ccb_phys);
#else
	sdma->channel_control = dma_alloc_coherent(NULL,
			MAX_DMA_CHANNELS * sizeof (struct sdma_channel_control) +
			sizeof(struct sdma_context_data),
			&ccb_phys, GFP_KERNEL);
#endif

	if (!sdma->channel_control) {
		ret = -ENOMEM;
		goto err_dma_alloc;
	}

	sdma->context = (void *)sdma->channel_control +
		MAX_DMA_CHANNELS * sizeof (struct sdma_channel_control);
	sdma->context_phys = ccb_phys +
		MAX_DMA_CHANNELS * sizeof (struct sdma_channel_control);

	/* Zero-out the CCB structures array just allocated */
	memset(sdma->channel_control, 0,
			MAX_DMA_CHANNELS * sizeof (struct sdma_channel_control));

	/* disable all channels */
	for (i = 0; i < sdma->num_events; i++)
		writel_relaxed(0, sdma->regs + chnenbl_ofs(sdma, i));

	/* All channels have priority 0 */
	for (i = 0; i < MAX_DMA_CHANNELS; i++)
		writel_relaxed(0, sdma->regs + SDMA_CHNPRI_0 + i * 4);

	ret = sdma_request_channel(&sdma->channel[0]);
	if (ret)
		goto err_dma_request;

	sdma_config_ownership(&sdma->channel[0], false, true, false);

	/* Set Command Channel (Channel Zero) */
	writel_relaxed(0x4050, sdma->regs + SDMA_CHN0ADDR);

	/* Set bits of CONFIG register but with static context switching */
	/* FIXME: Check whether to set ACR bit depending on clock ratios */
	writel_relaxed(0, sdma->regs + SDMA_H_CONFIG);

	writel_relaxed(ccb_phys, sdma->regs + SDMA_H_C0PTR);

	/* Set bits of CONFIG register with given context switching mode */
	writel_relaxed(SDMA_H_CONFIG_CSM, sdma->regs + SDMA_H_CONFIG);

	/* Initializes channel's priorities */
	sdma_set_channel_priority(&sdma->channel[0], 7);

	clk_disable(sdma->clk);

	return 0;

err_dma_request:
#ifdef CONFIG_SDMA_IRAM
	iram_free((unsigned long)ccb_phys, MAX_DMA_CHANNELS
			* sizeof(struct sdma_channel_control)
			+ sizeof(struct sdma_context_data));
#endif
err_dma_alloc:
	clk_disable(sdma->clk);
	dev_err(sdma->dev, "initialisation failed with %d\n", ret);
	return ret;
}

static int __init sdma_probe(struct platform_device *pdev)
{
	int ret;
	int irq;
	struct resource *iores;
	struct sdma_platform_data *pdata = pdev->dev.platform_data;
	int i;
	struct sdma_engine *sdma;

	sdma = kzalloc(sizeof(*sdma), GFP_KERNEL);
	if (!sdma)
		return -ENOMEM;

	spin_lock_init(&sdma->channel_0_lock);

	sdma->dev = &pdev->dev;

	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (!iores || irq < 0 || !pdata) {
		ret = -EINVAL;
		goto err_irq;
	}

	if (!request_mem_region(iores->start, resource_size(iores), pdev->name)) {
		ret = -EBUSY;
		goto err_request_region;
	}

	sdma->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(sdma->clk)) {
		ret = PTR_ERR(sdma->clk);
		goto err_clk;
	}

	sdma->regs = ioremap(iores->start, resource_size(iores));
	if (!sdma->regs) {
		ret = -ENOMEM;
		goto err_ioremap;
	}

	ret = request_irq(irq, sdma_int_handler, 0, "sdma", sdma);
	if (ret)
		goto err_request_irq;

	sdma->script_addrs = kzalloc(sizeof(*sdma->script_addrs), GFP_KERNEL);
	if (!sdma->script_addrs)
		goto err_alloc;

	sdma->version = pdata->sdma_version;

	dma_cap_set(DMA_SLAVE, sdma->dma_device.cap_mask);
	dma_cap_set(DMA_CYCLIC, sdma->dma_device.cap_mask);

	spin_lock_init(&sdma->irq_reg_lock);

	INIT_LIST_HEAD(&sdma->dma_device.channels);
	/* Initialize channel parameters */
	for (i = 0; i < MAX_DMA_CHANNELS; i++) {
		struct sdma_channel *sdmac = &sdma->channel[i];

		sdmac->sdma = sdma;
		spin_lock_init(&sdmac->lock);

		sdmac->chan.device = &sdma->dma_device;
		sdmac->channel = i;

		/*
		 * Add the channel to the DMAC list. Do not add channel 0 though
		 * because we need it internally in the SDMA driver. This also means
		 * that channel 0 in dmaengine counting matches sdma channel 1.
		 */
		if (i)
			list_add_tail(&sdmac->chan.device_node,
					&sdma->dma_device.channels);
	}

	ret = sdma_init(sdma);
	if (ret)
		goto err_init;

	if (pdata->script_addrs)
		sdma_add_scripts(sdma, pdata->script_addrs);

	sdma_get_firmware(sdma, pdata->cpu_name, pdata->to_version);

	sdma->dma_device.dev = &pdev->dev;

	sdma->dma_device.device_alloc_chan_resources = sdma_alloc_chan_resources;
	sdma->dma_device.device_free_chan_resources = sdma_free_chan_resources;
	sdma->dma_device.device_tx_status = sdma_tx_status;
	sdma->dma_device.device_prep_slave_sg = sdma_prep_slave_sg;
	sdma->dma_device.device_prep_dma_cyclic = sdma_prep_dma_cyclic;
	sdma->dma_device.device_control = sdma_control;
	sdma->dma_device.device_issue_pending = sdma_issue_pending;
	sdma->dma_device.dev->dma_parms = &sdma->dma_parms;
	dma_set_max_seg_size(sdma->dma_device.dev, 65535);

	ret = dma_async_device_register(&sdma->dma_device);
	if (ret) {
		dev_err(&pdev->dev, "unable to register\n");
		goto err_init;
	}

	dev_info(sdma->dev, "initialized\n");

	return 0;

err_init:
	kfree(sdma->script_addrs);
err_alloc:
	free_irq(irq, sdma);
err_request_irq:
	iounmap(sdma->regs);
err_ioremap:
	clk_put(sdma->clk);
err_clk:
	release_mem_region(iores->start, resource_size(iores));
err_request_region:
err_irq:
	kfree(sdma);
	return ret;
}

static int __exit sdma_remove(struct platform_device *pdev)
{
	return -EBUSY;
}

static struct platform_driver sdma_driver = {
	.driver		= {
		.name	= "imx-sdma",
	},
	.remove		= __exit_p(sdma_remove),
};

static int __init sdma_module_init(void)
{
	return platform_driver_probe(&sdma_driver, sdma_probe);
}
module_init(sdma_module_init);

MODULE_AUTHOR("Sascha Hauer, Pengutronix <s.hauer@pengutronix.de>");
MODULE_DESCRIPTION("i.MX SDMA driver");
MODULE_LICENSE("GPL");
