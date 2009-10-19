/*
 * Copyright (C) 2007 SigmaTel, Inc., Ioannis Kappas <ikappas@sigmatel.com>
 *
 * Portions copyright (C) 2003 Russell King, PXA MMCI Driver
 * Portions copyright (C) 2004-2005 Pierre Ossman, W83L51xD SD/MMC driver
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
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/highmem.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/completion.h>
#include <linux/mmc/host.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <mach/hardware.h>
#include <mach/dma.h>
#include <mach/regs-apbh.h>
#include <mach/regs-ssp.h>
#include <mach/regs-clkctrl.h>
#include <mach/stmp3xxx.h>
#include <mach/mmc.h>
#include <mach/platform.h>

#define DRIVER_NAME	"stmp3xxx-mmc"

#define CLOCKRATE_MIN 400000
#define CLOCKRATE_MAX 48000000

/*
 * Card detect polling timeout
 */
#define STMP37XX_MMC_DETECT_TIMEOUT		(HZ/2)

/* Max value supported for XFER_COUNT */
#define SSP_BUFFER_SIZE		(65536 - 512)

struct stmp3xxx_mmc_host {
	struct device *dev;
	struct mmc_host *mmc;

	struct clk *clk;
	unsigned int clkrt;

	struct mmc_request *mrq;
	struct mmc_command *cmd;
	struct mmc_data *data;

	/* Whether the card is capable of 4-bit data */
	int bus_width_4:1;

	/* Whether SD card is present */
	unsigned present:1;

	/* Polling timer */
	struct timer_list timer;

	/* SSP interface which MMC/SD card slot is attached to */
	void __iomem *ssp_base;

	/* DMA channel used for this host */
	unsigned int dmach;

	/* IRQs */
	int dmairq, errirq;

	/* DMA descriptor to transfer data over SSP interface */
	struct stmp3xxx_dma_descriptor dma_desc;

	/* DMA buffer */
	dma_addr_t dma_buf_phys;
	char *dma_buf;

	struct completion dma_done;
	/* status on last interrupt */
	u32 status;
	int read_uA, write_uA;
	struct regulator *regulator;
};

/* Return read only state of card */
static int stmp3xxx_mmc_get_ro(struct mmc_host *mmc)
{
	struct stmp3xxx_mmc_host *host = mmc_priv(mmc);
	struct stmp3xxxmmc_platform_data *pdata = host->dev->platform_data;

	if (pdata && pdata->get_wp)
		return pdata->get_wp();

	return 0;
}

/* Detect if card is plugged */
static inline int stmp3xxx_mmc_is_plugged(struct stmp3xxx_mmc_host *host)
{
	u32 status = __raw_readl(host->ssp_base + HW_SSP_STATUS);
	return !(status & BM_SSP_STATUS_CARD_DETECT);
}

/* Card detection polling function */
static void stmp3xxx_mmc_detect_poll(unsigned long arg)
{
	struct stmp3xxx_mmc_host *host = (struct stmp3xxx_mmc_host *)arg;
	int card_status;

	card_status = stmp3xxx_mmc_is_plugged(host);
	if (card_status != host->present) {
		host->present = card_status;
		mmc_detect_change(host->mmc, 0);
	}

	mod_timer(&host->timer, jiffies + STMP37XX_MMC_DETECT_TIMEOUT);
}

#define STMP3XXX_MMC_IRQ_BITS  (BM_SSP_CTRL1_SDIO_IRQ		| \
				BM_SSP_CTRL1_RESP_ERR_IRQ	| \
				BM_SSP_CTRL1_RESP_TIMEOUT_IRQ	| \
				BM_SSP_CTRL1_DATA_TIMEOUT_IRQ	| \
				BM_SSP_CTRL1_DATA_CRC_IRQ	| \
				BM_SSP_CTRL1_FIFO_UNDERRUN_IRQ	| \
				BM_SSP_CTRL1_RECV_TIMEOUT_IRQ   | \
				BM_SSP_CTRL1_FIFO_OVERRUN_IRQ)

/* SSP DMA interrupt handler */
static irqreturn_t mmc_irq_handler(int irq, void *dev_id)
{
	struct stmp3xxx_mmc_host *host = dev_id;
	u32 c1;

	c1 = __raw_readl(host->ssp_base + HW_SSP_CTRL1);
	stmp3xxx_clearl(c1 & STMP3XXX_MMC_IRQ_BITS,
			host->ssp_base + HW_SSP_CTRL1);
	if (irq == host->dmairq)
		stmp3xxx_dma_clear_interrupt(host->dmach);
	host->status =
	    __raw_readl(host->ssp_base + HW_SSP_STATUS);

	if (host->cmd)		/* else it is a bogus interrupt */
		complete(&host->dma_done);

	return IRQ_HANDLED;
}

/*
 * Check for MMC command errors
 * Returns error code or zerro if no errors
 */
static inline int stmp3xxx_mmc_cmd_error(u32 status)
{
	int err = 0;

	if (status & BM_SSP_STATUS_TIMEOUT)
		err = -ETIMEDOUT;
	else if (status & BM_SSP_STATUS_RESP_TIMEOUT)
		err = -ETIMEDOUT;
	else if (status & BM_SSP_STATUS_RESP_CRC_ERR)
		err = -EILSEQ;
	else if (status & BM_SSP_STATUS_RESP_ERR)
		err = -EIO;

	return err;
}

/* Send the BC command to the device */
static void stmp3xxx_mmc_bc(struct stmp3xxx_mmc_host *host)
{
	struct mmc_command *cmd = host->cmd;
	struct stmp3xxx_dma_descriptor *dma_desc = &host->dma_desc;

	dma_desc->command->cmd = BM_APBH_CHn_CMD_WAIT4ENDCMD | BM_APBH_CHn_CMD_SEMAPHORE | BM_APBH_CHn_CMD_IRQONCMPLT | BF(0, APBH_CHn_CMD_XFER_COUNT) | BF(3, APBH_CHn_CMD_CMDWORDS) | BF(0, APBH_CHn_CMD_COMMAND);	/* NO_DMA_XFER */

	dma_desc->command->pio_words[0] = BM_SSP_CTRL0_ENABLE |
	    BM_SSP_CTRL0_IGNORE_CRC;
	dma_desc->command->pio_words[1] = BF(cmd->opcode, SSP_CMD0_CMD) |
	    BM_SSP_CMD0_APPEND_8CYC;
	dma_desc->command->pio_words[2] = BF(cmd->arg, SSP_CMD1_CMD_ARG);

	init_completion(&host->dma_done);
	stmp3xxx_dma_reset_channel(host->dmach);
	stmp3xxx_dma_go(host->dmach, dma_desc, 1);
	wait_for_completion(&host->dma_done);

	cmd->error = stmp3xxx_mmc_cmd_error(host->status);

	if (stmp3xxx_dma_running(host->dmach))
		dev_dbg(host->dev, "DMA command not finished\n");

	if (cmd->error) {
		dev_dbg(host->dev, "Command error 0x%x\n", cmd->error);
		stmp3xxx_dma_reset_channel(host->dmach);
	}
}

/* Send the ac command to the device */
static void stmp3xxx_mmc_ac(struct stmp3xxx_mmc_host *host)
{
	struct mmc_command *cmd = host->cmd;
	struct stmp3xxx_dma_descriptor *dma_desc = &host->dma_desc;
	u32 ignore_crc, resp, long_resp;
	u32 ssp_ctrl0;
	u32 ssp_cmd0;
	u32 ssp_cmd1;

	ignore_crc = (mmc_resp_type(cmd) & MMC_RSP_CRC) ?
	    0 : BM_SSP_CTRL0_IGNORE_CRC;
	resp = (mmc_resp_type(cmd) & MMC_RSP_PRESENT) ?
	    BM_SSP_CTRL0_GET_RESP : 0;
	long_resp = (mmc_resp_type(cmd) & MMC_RSP_136) ?
	    BM_SSP_CTRL0_LONG_RESP : 0;

	dma_desc->command->cmd =
	    BM_APBH_CHn_CMD_WAIT4ENDCMD |
	    BM_APBH_CHn_CMD_SEMAPHORE |
	    BM_APBH_CHn_CMD_IRQONCMPLT |
	    BF(0, APBH_CHn_CMD_XFER_COUNT) |
	    BF(3, APBH_CHn_CMD_CMDWORDS) | BF(0, APBH_CHn_CMD_COMMAND);

	ssp_ctrl0 = BM_SSP_CTRL0_ENABLE | ignore_crc | long_resp | resp;
	ssp_cmd0 = BF(cmd->opcode, SSP_CMD0_CMD);
	ssp_cmd1 = BF(cmd->arg, SSP_CMD1_CMD_ARG);

	dma_desc->command->pio_words[0] = ssp_ctrl0;
	dma_desc->command->pio_words[1] = ssp_cmd0;
	dma_desc->command->pio_words[2] = ssp_cmd1;

	stmp3xxx_dma_reset_channel(host->dmach);
	init_completion(&host->dma_done);
	stmp3xxx_dma_go(host->dmach, dma_desc, 1);
	wait_for_completion(&host->dma_done);

	switch (mmc_resp_type(cmd)) {
	case MMC_RSP_NONE:
		while (__raw_readl(host->ssp_base + HW_SSP_CTRL0)
		       & BM_SSP_CTRL0_RUN)
			continue;
		break;
	case MMC_RSP_R1:
	case MMC_RSP_R1B:
	case MMC_RSP_R3:
		cmd->resp[0] =
		    __raw_readl(host->ssp_base + HW_SSP_SDRESP0);
		break;
	case MMC_RSP_R2:
		cmd->resp[3] =
		    __raw_readl(host->ssp_base + HW_SSP_SDRESP0);
		cmd->resp[2] =
		    __raw_readl(host->ssp_base + HW_SSP_SDRESP1);
		cmd->resp[1] =
		    __raw_readl(host->ssp_base + HW_SSP_SDRESP2);
		cmd->resp[0] =
		    __raw_readl(host->ssp_base + HW_SSP_SDRESP3);
		break;
	default:
		dev_warn(host->dev, "Unsupported response type 0x%x\n",
			 mmc_resp_type(cmd));
		BUG();
		break;
	}

	cmd->error = stmp3xxx_mmc_cmd_error(host->status);

	if (stmp3xxx_dma_running(host->dmach))
		dev_dbg(host->dev, "DMA command not finished\n");

	if (cmd->error) {
		dev_dbg(host->dev, "Command error 0x%x\n", cmd->error);
		stmp3xxx_dma_reset_channel(host->dmach);
	}
}

/* Copy data between sg list and dma buffer */
static unsigned int stmp3xxx_sg_dma_copy(struct stmp3xxx_mmc_host *host,
					 unsigned int size, int to_dma)
{
	struct mmc_data *data = host->cmd->data;
	unsigned int copy_size, bytes_copied = 0;
	struct scatterlist *sg;
	char *dmabuf = host->dma_buf;
	char *sgbuf;
	int len, i;

	sg = data->sg;
	len = data->sg_len;

	/*
	 * Just loop through all entries. Size might not
	 * be the entire list though so make sure that
	 * we do not transfer too much.
	 */
	for (i = 0; i < len; i++) {
		sgbuf = kmap_atomic(sg_page(&sg[i]), KM_BIO_SRC_IRQ) +
		    sg[i].offset;
		copy_size = size < sg[i].length ? size : sg[i].length;
		if (to_dma)
			memcpy(dmabuf, sgbuf, copy_size);
		else
			memcpy(sgbuf, dmabuf, copy_size);
		kunmap_atomic(sgbuf, KM_BIO_SRC_IRQ);

		dmabuf += sg[i].length;

		bytes_copied += copy_size;
		size -= copy_size;

		if (size == 0)
			break;
	}

	return bytes_copied;
}

/* Convert ns to tick count according to the current sclk speed */
static unsigned short stmp3xxx_ns_to_ssp_ticks(unsigned clock_rate, unsigned ns)
{
	const unsigned int ssp_timeout_mul = 4096;
	/*
	 * Calculate ticks in ms since ns are large numbers
	 * and might overflow
	 */
	const unsigned int clock_per_ms = clock_rate / 1000;
	const unsigned int ms = ns / 1000;
	const unsigned int ticks = ms * clock_per_ms;
	const unsigned int ssp_ticks = ticks / ssp_timeout_mul;

	BUG_ON(ssp_ticks == 0);
	return ssp_ticks;
}

static void __init_reg(struct device *dev, struct regulator **pp_reg)
{
	struct regulator *reg = *pp_reg;

	if (!reg) {
		reg = regulator_get(NULL, "mmc_ssp-1");
		if (reg && !IS_ERR(reg))
			regulator_set_mode(reg, REGULATOR_MODE_NORMAL);
		else
			reg = NULL;
		*pp_reg = reg;
	}
}

/* Send adtc command to the card */
static void stmp3xxx_mmc_adtc(struct stmp3xxx_mmc_host *host)
{
	struct mmc_command *cmd = host->cmd;
	struct stmp3xxx_dma_descriptor *dma_desc = &host->dma_desc;
	int ignore_crc, resp, long_resp;
	int is_reading = 0;
	unsigned int copy_size;

	u32 ssp_ctrl0;
	u32 ssp_cmd0;
	u32 ssp_cmd1;
	u32 timeout;
	u32 val;

	u32 data_size = cmd->data->blksz * cmd->data->blocks;
	u32 log2_block_size;

	ignore_crc = mmc_resp_type(cmd) & MMC_RSP_CRC ? 0 : 1;
	resp = mmc_resp_type(cmd) & MMC_RSP_PRESENT ? 1 : 0;
	long_resp = mmc_resp_type(cmd) & MMC_RSP_136 ? 1 : 0;

	dev_dbg(host->dev, "ADTC command:\n"
		"response: %d, ignore crc: %d\n"
		"data list: %u, blocksz: %u, blocks: %u, timeout: %uns %uclks, "
		"flags: 0x%x\n", resp, ignore_crc, cmd->data->sg_len,
		cmd->data->blksz, cmd->data->blocks, cmd->data->timeout_ns,
		cmd->data->timeout_clks, cmd->data->flags);

	if (cmd->data->flags & MMC_DATA_WRITE) {
		dev_dbg(host->dev, "Data Write\n");
		copy_size = stmp3xxx_sg_dma_copy(host, data_size, 1);
		BUG_ON(copy_size < data_size);
		is_reading = 0;
		if (!host->regulator)
			__init_reg(host->dev, &host->regulator);
		if (host->regulator)
			regulator_set_current_limit(host->regulator,
						    host->write_uA,
						    host->write_uA);
	} else if (cmd->data->flags & MMC_DATA_READ) {
		dev_dbg(host->dev, "Data Read\n");
		is_reading = 1;
		if (!host->regulator)
			__init_reg(host->dev, &host->regulator);
		if (host->regulator)
			regulator_set_current_limit(host->regulator,
						    host->read_uA,
						    host->read_uA);
	} else {
		dev_warn(host->dev, "Unsuspported data mode, 0x%x\n",
			 cmd->data->flags);
		BUG();
	}

	BUG_ON(cmd->data->flags & MMC_DATA_STREAM);
	BUG_ON((data_size % 8) > 0);

	dma_desc->command->cmd =
	    BM_APBH_CHn_CMD_WAIT4ENDCMD |
	    BM_APBH_CHn_CMD_SEMAPHORE |
	    BM_APBH_CHn_CMD_IRQONCMPLT |
	    BF(data_size, APBH_CHn_CMD_XFER_COUNT) |
	    BF(3, APBH_CHn_CMD_CMDWORDS);

	/* when is_reading is set, DMA controller performs WRITE operation. */
	dma_desc->command->cmd |=
	    BF(is_reading ? BV_APBH_CHn_CMD_COMMAND__DMA_WRITE :
			     BV_APBH_CHn_CMD_COMMAND__DMA_READ,
			     APBH_CHn_CMD_COMMAND);
	ssp_ctrl0 =
	    (ignore_crc ? BM_SSP_CTRL0_IGNORE_CRC : 0) | (resp ?
							  BM_SSP_CTRL0_GET_RESP
							  : 0) | (long_resp ?
								  BM_SSP_CTRL0_LONG_RESP
								  : 0) |
	    (is_reading ? BM_SSP_CTRL0_READ : 0) | BM_SSP_CTRL0_DATA_XFER |
	    BM_SSP_CTRL0_WAIT_FOR_IRQ | BM_SSP_CTRL0_ENABLE | BF(data_size,
								 SSP_CTRL0_XFER_COUNT)
	    | BF(host->bus_width_4 ? BV_SSP_CTRL0_BUS_WIDTH__FOUR_BIT :
				     BV_SSP_CTRL0_BUS_WIDTH__ONE_BIT,
				     SSP_CTRL0_BUS_WIDTH);

	/*
	 * We need to set the hardware register to the logarithm to base 2 of
	 * the block size.
	 */
	log2_block_size = ilog2(cmd->data->blksz);

	ssp_cmd0 =
	    BF(log2_block_size, SSP_CMD0_BLOCK_SIZE) |
	    BF(cmd->opcode, SSP_CMD0_CMD) |
	    BF(cmd->data->blocks - 1, SSP_CMD0_BLOCK_COUNT);

	if (cmd->opcode == 12)
		ssp_cmd0 |= BM_SSP_CMD0_APPEND_8CYC;

	ssp_cmd1 = BF(cmd->arg, SSP_CMD1_CMD_ARG);

	dma_desc->command->pio_words[0] = ssp_ctrl0;
	dma_desc->command->pio_words[1] = ssp_cmd0;
	dma_desc->command->pio_words[2] = ssp_cmd1;

	/* Set the timeout count */
	timeout = stmp3xxx_ns_to_ssp_ticks(host->clkrt, cmd->data->timeout_ns);
	val = __raw_readl(host->ssp_base + HW_SSP_TIMING);
	val &= ~(BM_SSP_TIMING_TIMEOUT);
	val |= BF(timeout, SSP_TIMING_TIMEOUT);
	__raw_writel(val, host->ssp_base + HW_SSP_TIMING);

	init_completion(&host->dma_done);
	stmp3xxx_dma_reset_channel(host->dmach);
	stmp3xxx_dma_go(host->dmach, dma_desc, 1);
	wait_for_completion(&host->dma_done);
	if (host->regulator)
		regulator_set_current_limit(host->regulator, 0, 0);

	switch (mmc_resp_type(cmd)) {
	case MMC_RSP_NONE:
		break;
	case MMC_RSP_R1:
	case MMC_RSP_R3:
		cmd->resp[0] =
		    __raw_readl(host->ssp_base + HW_SSP_SDRESP0);
		break;
	case MMC_RSP_R2:
		cmd->resp[3] =
		    __raw_readl(host->ssp_base + HW_SSP_SDRESP0);
		cmd->resp[2] =
		    __raw_readl(host->ssp_base + HW_SSP_SDRESP1);
		cmd->resp[1] =
		    __raw_readl(host->ssp_base + HW_SSP_SDRESP2);
		cmd->resp[0] =
		    __raw_readl(host->ssp_base + HW_SSP_SDRESP3);
		break;
	default:
		dev_warn(host->dev, "Unsupported response type 0x%x\n",
			 mmc_resp_type(cmd));
		BUG();
		break;
	}

	cmd->error = stmp3xxx_mmc_cmd_error(host->status);

	if (stmp3xxx_dma_running(host->dmach))
		dev_dbg(host->dev, "DMA command not finished\n");

	if (cmd->error) {
		dev_dbg(host->dev, "Command error 0x%x\n", cmd->error);
		stmp3xxx_dma_reset_channel(host->dmach);
	} else {
		if (is_reading)
			cmd->data->bytes_xfered =
			    stmp3xxx_sg_dma_copy(host, data_size, 0);
		else
			cmd->data->bytes_xfered = data_size;

		dev_dbg(host->dev, "Transferred %u bytes\n",
			cmd->data->bytes_xfered);
	}
}

/* Begin sedning a command to the card */
static void stmp3xxx_mmc_start_cmd(struct stmp3xxx_mmc_host *host,
				   struct mmc_command *cmd)
{
	dev_dbg(host->dev, "MMC command:\n"
		"type: 0x%x opcode: %u, arg: %u, flags 0x%x retries: %u\n",
		mmc_cmd_type(cmd), cmd->opcode, cmd->arg, cmd->flags,
		cmd->retries);

	host->cmd = cmd;

	switch (mmc_cmd_type(cmd)) {
	case MMC_CMD_BC:
		stmp3xxx_mmc_bc(host);
		break;
	case MMC_CMD_BCR:
		stmp3xxx_mmc_ac(host);
		break;
	case MMC_CMD_AC:
		stmp3xxx_mmc_ac(host);
		break;
	case MMC_CMD_ADTC:
		stmp3xxx_mmc_adtc(host);
		break;
	default:
		dev_warn(host->dev, "Unknown MMC command\n");
		BUG();
		break;
	}

	dev_dbg(host->dev, "response: %u %u %u %u errors: %u\n",
		cmd->resp[0], cmd->resp[1], cmd->resp[2], cmd->resp[3],
		cmd->error);
}

/* Handle MMC request */
static void stmp3xxx_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct stmp3xxx_mmc_host *host = mmc_priv(mmc);

	dev_dbg(host->dev, "MMC request\n");

	host->mrq = mrq;

	stmp3xxx_mmc_start_cmd(host, mrq->cmd);

	if (mrq->data && mrq->data->stop) {
		dev_dbg(host->dev, "Stop opcode is %u\n",
			mrq->data->stop->opcode);
		stmp3xxx_mmc_start_cmd(host, mrq->data->stop);
	}

	host->mrq = NULL;
	mmc_request_done(mmc, mrq);
}

/*
 * Change divisors to reflect the rate of 'hz'. Note that we should not
 * play with clock rate, because the same source is used to clock both
 * SSP ports.
 */
static void
stmp3xxx_set_sclk_speed(struct stmp3xxx_mmc_host *host, unsigned int hz)
{
	unsigned long ssp;
	u32 div1, div2;
	u32 val;
	struct stmp3xxxmmc_platform_data *pdata = host->dev->platform_data;

	if (get_evk_board_version() == 1) {
		/*EVK Ver1 max clock is 12M */
		if (hz > 12000000)
			hz = 12000000;
	}

	if (pdata && pdata->setclock) {
		/*
		   if the SSP is buggy and platform provides callback...
		   well, let it be.
		 */
		host->clkrt = pdata->setclock(hz);
		return;
	}

	/*
	   ...but the RightIdea(tm) is to set divisors to match
	   the requested clock.
	 */
	hz /= 1000;

	ssp = clk_get_rate(host->clk);

	for (div1 = 2; div1 < 254; div1 += 2) {
		div2 = ssp / hz / div1;
		if (div2 < 0x100)
			break;
	}
	if (div1 >= 254) {
		dev_err(host->dev, "Cannot set clock to %dkHz\n", hz);
		return;
	}

	dev_dbg(host->dev, "Setting clock rate to %ld kHz [%x+%x] "
		"(requested %d), source %ldk\n",
		ssp / div1 / div2, div1, div2, hz, ssp);

	val = __raw_readl(host->ssp_base + HW_SSP_TIMING);
	val &= ~(BM_SSP_TIMING_CLOCK_DIVIDE | BM_SSP_TIMING_CLOCK_RATE);
	val |= BF(div1, SSP_TIMING_CLOCK_DIVIDE) |
		      BF(div2 - 1, SSP_TIMING_CLOCK_RATE);
	__raw_writel(val, host->ssp_base + HW_SSP_TIMING);

	host->clkrt = ssp / div1 / div2 * 1000;
}

/* Configure card */
static void stmp3xxx_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct stmp3xxx_mmc_host *host = mmc_priv(mmc);
	struct stmp3xxxmmc_platform_data *pdata;

	dev_dbg(host->dev, "MMC set ios:\n"
		"Clock %u, vdd %u, bus_mode %u, chip_select %u, "
		"power mode %u, bus_width %u\n", ios->clock, ios->vdd,
		ios->bus_mode, ios->chip_select, ios->power_mode,
		ios->bus_width);

	pdata = host->dev->platform_data;

	if (pdata->cmd_pullup) {
		if (ios->bus_mode == MMC_BUSMODE_PUSHPULL)
			pdata->cmd_pullup(0);
		else
			pdata->cmd_pullup(1);
	} else
		dev_warn(host->dev,
			 "Platform does not support CMD pin pullup control\n");

	if (ios->bus_width == MMC_BUS_WIDTH_4)
		host->bus_width_4 = 1;
	else
		host->bus_width_4 = 0;

	if (ios->clock > 0)
		stmp3xxx_set_sclk_speed(host, ios->clock);
}

static const struct mmc_host_ops stmp3xxx_mmc_ops = {
	.request = stmp3xxx_mmc_request,
	.get_ro = stmp3xxx_mmc_get_ro,
	.set_ios = stmp3xxx_mmc_set_ios,
};

/*
 * STMP37XX MMC/SD driver initialization
 */

/* Reset ssp peripheral to default values */
static void stmp3xxx_mmc_reset(struct stmp3xxx_mmc_host *host)
{
	u32 ssp_ctrl0;
	u32 ssp_ctrl1;

	stmp3xxx_reset_block(host->ssp_base, 0);

	/* Configure SSP Control Register 0 */
	ssp_ctrl0 =
	    BM_SSP_CTRL0_IGNORE_CRC |
	    BF(BV_SSP_CTRL0_BUS_WIDTH__ONE_BIT, SSP_CTRL0_BUS_WIDTH);

	/* Configure SSP Control Register 1 */
	ssp_ctrl1 =
	    BM_SSP_CTRL1_DMA_ENABLE |
	    BM_SSP_CTRL1_POLARITY |
	    BM_SSP_CTRL1_RECV_TIMEOUT_IRQ_EN |
	    BM_SSP_CTRL1_DATA_CRC_IRQ_EN |
	    BM_SSP_CTRL1_DATA_TIMEOUT_IRQ_EN |
	    BM_SSP_CTRL1_RESP_TIMEOUT_IRQ_EN |
	    BM_SSP_CTRL1_RESP_ERR_IRQ_EN |
	    BF(BV_SSP_CTRL1_WORD_LENGTH__EIGHT_BITS, SSP_CTRL1_WORD_LENGTH) |
	    BF(BV_SSP_CTRL1_SSP_MODE__SD_MMC, SSP_CTRL1_SSP_MODE);

	__raw_writel(BF(0xFFFF, SSP_TIMING_TIMEOUT) |
		     BF(2, SSP_TIMING_CLOCK_DIVIDE) |
		     BF(0, SSP_TIMING_CLOCK_RATE),
		     host->ssp_base + HW_SSP_TIMING);

	/* Write the SSP Control Register 0 and 1 values out to the interface */
	__raw_writel(ssp_ctrl0, host->ssp_base + HW_SSP_CTRL0);
	__raw_writel(ssp_ctrl1, host->ssp_base + HW_SSP_CTRL1);
}

static void stmp3xxx_mmc_irq_release(struct stmp3xxx_mmc_host *host)
{
	free_irq(host->dmairq, host);
	free_irq(host->errirq, host);
}

static int __init stmp3xxx_mmc_irq_init(struct stmp3xxx_mmc_host *host)
{
	int ret;

	ret = request_irq(host->dmairq, mmc_irq_handler, 0,
			  DRIVER_NAME " dma", host);
	if (ret) {
		dev_err(host->dev, "Unable to set up DMA irq handler\n");
		goto out0;
	}

	ret = request_irq(host->errirq, mmc_irq_handler, IRQF_SHARED,
			  DRIVER_NAME " error", host);
	if (ret) {
		dev_err(host->dev, "Unable to set up SSP error irq handler\n");
		goto out1;
	}
	return 0;

out1:
	free_irq(host->dmairq, host);
out0:
	return ret;
}

/* Allocate and initialise the DMA chains */
static int stmp3xxx_mmc_dma_init(struct stmp3xxx_mmc_host *host, int reset)
{
	int ret;

	if (!reset) {
		/* Allocate DMA channel */
		ret = stmp3xxx_dma_request(host->dmach,
					   host->dev, "STMP37XX MMC/SD");
		if (ret) {
			dev_err(host->dev, "Unable to request DMA channel\n");
			return ret;
		}

		host->dma_buf = dma_alloc_coherent(host->dev, SSP_BUFFER_SIZE,
						   &host->dma_buf_phys,
						   GFP_DMA);
		if (host->dma_buf == NULL) {
			dev_err(host->dev, "Unable to allocate DMA memory\n");
			ret = -ENOMEM;
			goto out_mem;
		}

		ret = stmp3xxx_dma_allocate_command(host->dmach,
						    &host->dma_desc);
		if (ret) {
			dev_err(host->dev,
				"Unable to allocate DMA descriptor\n");
			goto out_cmd;
		}

		host->dma_desc.command->next = (u32) host->dma_desc.handle;
		host->dma_desc.command->buf_ptr = (u32) host->dma_buf_phys;
		host->dma_desc.virtual_buf_ptr = host->dma_buf;
	}

	/* Reset DMA channel */
	stmp3xxx_dma_reset_channel(host->dmach);

	/* Enable DMA interrupt */
	stmp3xxx_dma_clear_interrupt(host->dmach);
	stmp3xxx_dma_enable_interrupt(host->dmach);

	return 0;

out_cmd:
	dma_free_coherent(host->dev, SSP_BUFFER_SIZE, host->dma_buf,
			  host->dma_buf_phys);
out_mem:
	stmp3xxx_dma_release(host->dmach);

	return ret;
}

static void stmp3xxx_mmc_dma_release(struct stmp3xxx_mmc_host *host)
{
	stmp3xxx_dma_reset_channel(host->dmach);

	dma_free_coherent(host->dev, SSP_BUFFER_SIZE, host->dma_buf,
			  host->dma_buf_phys);

	stmp3xxx_dma_free_command(host->dmach, &host->dma_desc);
	stmp3xxx_dma_release(host->dmach);
}

/* Probe peripheral for connected cards */
static int __init stmp3xxx_mmc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct stmp3xxxmmc_platform_data *mmc_data;
	struct stmp3xxx_mmc_host *host;
	struct mmc_host *mmc;
	struct resource *r;
	int err = 0;

	mmc_data = dev->platform_data;
	if (mmc_data == NULL) {
		err = -EINVAL;
		dev_err(dev, "Missing platform data\n");
		goto out;
	}

	/* Allocate main MMC host structure */
	mmc = mmc_alloc_host(sizeof(struct stmp3xxx_mmc_host), dev);
	if (!mmc) {
		dev_err(dev, "Unable to allocate MMC host\n");
		err = -ENOMEM;
		goto out;
	}
	host = mmc_priv(mmc);

	host->read_uA = mmc_data->read_uA;
	host->write_uA = mmc_data->write_uA;
	host->regulator = regulator_get(NULL, "mmc_ssp-1");
	if (host->regulator && !IS_ERR(host->regulator))
		regulator_set_mode(host->regulator, REGULATOR_MODE_NORMAL);
	else
		host->regulator = NULL;

	/* get resources: */

	/*
	 *  1. io memory
	 */
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		dev_err(&pdev->dev, "failed to get IORESOURCE_MEM\n");
		err = -ENXIO;
		goto out_res;
	}
	host->ssp_base =
		r->start - STMP3XXX_REGS_PHBASE + STMP3XXX_REGS_BASE;

	/*
	 *  2. DMA channel
	 */
	r = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	if (!r) {
		dev_err(&pdev->dev, "failed to get IORESOURCE_DMA\n");
		err = -ENXIO;
		goto out_res;
	}
	host->dmach = r->start;

	/*
	 * 3. two IRQs
	 */
	host->dmairq = platform_get_irq(pdev, 0);
	if (host->dmairq < 0) {
		dev_err(&pdev->dev, "failed to get IORESOURCE_IRQ/0\n");
		err = host->dmairq;
		goto out_res;
	}

	host->errirq = platform_get_irq(pdev, 1);
	if (host->errirq < 0) {
		dev_err(&pdev->dev, "failed to get IORESOURCE_IRQ/1\n");
		err = host->errirq;
		goto out_res;
	}

	/* Set up MMC pins */
	if (mmc_data->hw_init) {
		err = mmc_data->hw_init();
		if (err) {
			dev_err(dev, "MMC HW configuration failed\n");
			goto out_res;
		}
	}

	host->mmc = mmc;
	host->dev = dev;

	/* Set minimal clock rate */
	host->clk = clk_get(dev, "ssp");
	if (IS_ERR(host->clk)) {
		err = PTR_ERR(host->clk);
		dev_err(dev, "Clocks initialization failed\n");
		goto out_clk;
	}

	clk_enable(host->clk);
	stmp3xxx_set_sclk_speed(host, CLOCKRATE_MIN);

	/* Reset MMC block */
	stmp3xxx_mmc_reset(host);

	/* Enable DMA */
	err = stmp3xxx_mmc_dma_init(host, 0);
	if (err) {
		dev_err(dev, "DMA init failed\n");
		goto out_dma;
	}

	/* Set up interrupt handlers */
	err = stmp3xxx_mmc_irq_init(host);
	if (err) {
		dev_err(dev, "IRQ initialization failed\n");
		goto out_irq;
	}

	/* Get current card status for further cnanges tracking */
	host->present = stmp3xxx_mmc_is_plugged(host);

	/* Add a card detection polling timer */
	init_timer(&host->timer);
	host->timer.function = stmp3xxx_mmc_detect_poll;
	host->timer.data = (unsigned long)host;
	host->timer.expires = jiffies + STMP37XX_MMC_DETECT_TIMEOUT;
	add_timer(&host->timer);

	mmc->ops = &stmp3xxx_mmc_ops;
	mmc->f_min = CLOCKRATE_MIN;
	mmc->f_max = CLOCKRATE_MAX;
	mmc->caps = MMC_CAP_4_BIT_DATA;

	/* Maximum block count requests. */
	mmc->max_blk_size = 512;
	mmc->max_blk_count = SSP_BUFFER_SIZE / 512;
	mmc->max_hw_segs = SSP_BUFFER_SIZE / 512;
	mmc->max_phys_segs = SSP_BUFFER_SIZE / 512;
	mmc->max_req_size = SSP_BUFFER_SIZE;
	mmc->max_seg_size = SSP_BUFFER_SIZE;

	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;

	platform_set_drvdata(pdev, mmc);

	err = mmc_add_host(mmc);
	if (err) {
		dev_err(dev, "Oh God. mmc_add_host failed\n");
		goto out_all;
	}

	return err;

out_all:

out_irq:
	stmp3xxx_mmc_dma_release(host);
out_dma:
	clk_disable(host->clk);
out_clk:
	if (mmc_data->hw_release)
		mmc_data->hw_release();
out_res:
	mmc_free_host(mmc);
out:
	return err;
}

static int __exit stmp3xxx_mmc_remove(struct platform_device *pdev)
{
	struct stmp3xxx_mmc_host *host;
	struct stmp3xxxmmc_platform_data *mmc_data;
	struct mmc_host *mmc;

	dev_info(&pdev->dev, "Removing\n");

	mmc_data = pdev->dev.platform_data;
	mmc = platform_get_drvdata(pdev);
	platform_set_drvdata(pdev, NULL);

	host = mmc_priv(mmc);
	mmc_remove_host(mmc);

	/* Disable SSP clock */
	clk_disable(host->clk);
	clk_put(host->clk);

	/* Release IRQs */
	stmp3xxx_mmc_irq_release(host);

	/* Delete card detection timer */
	del_timer(&host->timer);

	/* Release DMA */
	stmp3xxx_mmc_dma_release(host);
	if (host->regulator)
		regulator_put(host->regulator);

	mmc_free_host(mmc);

	if (mmc_data->hw_release)
		mmc_data->hw_release();

	return 0;
}

#ifdef CONFIG_PM
static int stmp3xxx_mmc_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	struct stmp3xxx_mmc_host *host;
	struct stmp3xxxmmc_platform_data *mmc_data;
	struct mmc_host *mmc;
	int ret = 0;

	dev_dbg(&pdev->dev, "Suspending\n");

	mmc_data = pdev->dev.platform_data;
	mmc = platform_get_drvdata(pdev);
	host = mmc_priv(mmc);

	ret = mmc_suspend_host(mmc, state);
	if (!ret) {
		if (mmc_data && mmc_data->hw_release)
			mmc_data->hw_release();
		clk_disable(host->clk);
	}
	return ret;
}

static int stmp3xxx_mmc_resume(struct platform_device *pdev)
{
	struct stmp3xxx_mmc_host *host;
	struct stmp3xxxmmc_platform_data *mmc_data;
	struct mmc_host *mmc;

	dev_dbg(&pdev->dev, "Resuming\n");

	mmc_data = pdev->dev.platform_data;
	mmc = platform_get_drvdata(pdev);
	host = mmc_priv(mmc);

	clk_enable(host->clk);

	if (mmc_data->hw_init)
		mmc_data->hw_init();
	stmp3xxx_mmc_reset(host);
	stmp3xxx_mmc_dma_init(host, 1);

	return mmc_resume_host(mmc);
}
#else
#define stmp3xxx_mmc_suspend	NULL
#define stmp3xxx_mmc_resume	NULL
#endif /* CONFIG_PM */

static struct platform_driver stmp3xxx_mmc_driver = {
	.probe = stmp3xxx_mmc_probe,
	.remove = __exit_p(stmp3xxx_mmc_remove),
	.suspend = stmp3xxx_mmc_suspend,
	.resume = stmp3xxx_mmc_resume,
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   },
};

static int __init stmp3xxx_mmc_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&stmp3xxx_mmc_driver);
	if (ret < 0)
		return ret;

	return ret;
}

static void __exit stmp3xxx_mmc_exit(void)
{
	platform_driver_unregister(&stmp3xxx_mmc_driver);
}

module_init(stmp3xxx_mmc_init);
module_exit(stmp3xxx_mmc_exit);

MODULE_DESCRIPTION("STMP37xx/378x MMC peripheral");
MODULE_LICENSE("GPL");
