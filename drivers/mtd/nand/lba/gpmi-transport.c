/*
 * Freescale STMP37XX/STMP378X GPMI transport layer for LBA driver
 *
 * Author: Dmitrij Frasenyak <sed@embeddedalley.com>
 * Clock settings and hw init comes from
 * gpmi driver by Dmitry Pervushin.
 *
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2009 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/ctype.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <asm/div64.h>
#include <mach/platform.h>
#include <mach/regs-apbh.h>
#include <mach/regs-gpmi.h>
#include <mach/stmp3xxx.h>
#include <mach/dma.h>
#include "gpmi.h"
#include "lba.h"

struct lba_data *g_data;
static int max_chips = 1;
static long clk = -1;

struct gpmi_nand_timing gpmi_safe_timing = {
	.address_setup = 25,
	.data_setup = 80,
	.data_hold = 60,
	.dsample_time = 6,
};

/******************************************************************************
 * HW init part
 ******************************************************************************/

/**
 * gpmi_irq - IRQ handler
 *
 * @irq:	irq no
 * @context:	IRQ context, pointer to gpmi_nand_data
 */
static irqreturn_t gpmi_irq(int irq, void *context)
{
	struct lba_data *data = context;
	int i;

	for (i = 0; i < max_chips; i++) {
		if (stmp3xxx_dma_is_interrupt(data->nand[i].dma_ch)) {
			stmp3xxx_dma_clear_interrupt(data->nand[i].dma_ch);
			complete(&data->nand[i].done);
		}

	}
	__raw_writel(BM_GPMI_CTRL1_DEV_IRQ | BM_GPMI_CTRL1_TIMEOUT_IRQ,
			REGS_GPMI_BASE + HW_GPMI_CTRL1_CLR);
	return IRQ_HANDLED;
}

static inline u32 gpmi_cycles_ceil(u32 ntime, u32 period)
{
	int k;

	k = (ntime + period - 1) / period;
	if (k == 0)
		k++;
	return k ;
}


/**
 * gpmi_set_timings - set GPMI timings
 * @pdev: pointer to GPMI platform device
 * @tm: pointer to structure &gpmi_nand_timing with new timings
 *
 * During initialization, GPMI uses safe sub-optimal timings, which
 * can be changed after reading boot control blocks
 */
void gpmi_set_timings(struct lba_data *data, struct gpmi_nand_timing *tm)
{
	u32 period_ns = 1000000 / clk_get_rate(data->clk) + 1;
	u32 address_cycles, data_setup_cycles;
	u32 data_hold_cycles, data_sample_cycles;
	u32 busy_timeout;
	u32 t0, reg;

	address_cycles = gpmi_cycles_ceil(tm->address_setup, period_ns);
	data_setup_cycles = gpmi_cycles_ceil(tm->data_setup, period_ns);
	data_hold_cycles = gpmi_cycles_ceil(tm->data_hold, period_ns);
	data_sample_cycles = gpmi_cycles_ceil(tm->dsample_time + period_ns / 4,
			period_ns / 2);
	busy_timeout = gpmi_cycles_ceil(10000000 / 4096, period_ns);

	t0 = address_cycles << BP_GPMI_TIMING0_ADDRESS_SETUP;
	t0 |= data_setup_cycles << BP_GPMI_TIMING0_DATA_SETUP;
	t0 |= data_hold_cycles << BP_GPMI_TIMING0_DATA_HOLD;
	__raw_writel(t0, REGS_GPMI_BASE + HW_GPMI_TIMING0);

	__raw_writel(busy_timeout, REGS_GPMI_BASE + HW_GPMI_TIMING1);

	reg = __raw_readl(REGS_GPMI_BASE + HW_GPMI_CTRL1);
#ifdef CONFIG_ARCH_STMP378X
	reg &= ~BM_GPMI_CTRL1_RDN_DELAY;
	reg |= data_sample_cycles << BP_GPMI_CTRL1_RDN_DELAY;
#else
	reg &= ~BM_GPMI_CTRL1_DSAMPLE_TIME;
	reg |= data_sample_cycles << BP_GPMI_CTRL1_DSAMPLE_TIME;
#endif
	__raw_writel(reg, REGS_GPMI_BASE + HW_GPMI_CTRL1);
}

void queue_plug(struct lba_data *data)
{
	u32 ctrl1;
	clk_enable(data->clk);
	if (clk <= 0)
		clk = 24000; /* safe setting, some chips do not work on
				    speeds >= 24kHz */
	clk_set_rate(data->clk, clk);

	clk = clk_get_rate(data->clk);


	stmp3xxx_reset_block(HW_GPMI_CTRL0 + REGS_GPMI_BASE, 1);

	ctrl1 = __raw_readl(REGS_GPMI_BASE + HW_GPMI_CTRL1);

	/* write protection OFF	*/
	ctrl1 |= BM_GPMI_CTRL1_DEV_RESET;

	/* IRQ polarity */
	ctrl1 |= BM_GPMI_CTRL1_ATA_IRQRDY_POLARITY;

	/* ...and ECC module */
	/*HW_GPMI_CTRL1_SET(bch_mode());*/

	/* choose NAND mode (1 means ATA, 0 - NAND */
	ctrl1 &= ~BM_GPMI_CTRL1_GPMI_MODE;

	__raw_writel(ctrl1, REGS_GPMI_BASE + HW_GPMI_CTRL1);

	gpmi_set_timings(data, &gpmi_safe_timing);
}

void queue_release(struct lba_data *data)
{
	__raw_writel(BM_GPMI_CTRL0_SFTRST, REGS_GPMI_BASE + HW_GPMI_CTRL0_SET);

	clk_disable(data->clk);
}


/**
 * gpmi_init_hw - initialize the hardware
 * @pdev: pointer to platform device
 *
 * Initialize GPMI hardware and set default (safe) timings for NAND access.
 * Returns error code or 0 on success
 */
static int gpmi_init_hw(struct platform_device *pdev, int request_pins)
{
	struct lba_data *data = platform_get_drvdata(pdev);
	struct gpmi_platform_data *gpd =
			(struct gpmi_platform_data *)pdev->dev.platform_data;
	int err = 0;

	data->clk = clk_get(NULL, "gpmi");
	if (IS_ERR(data->clk)) {
		err = PTR_ERR(data->clk);
		dev_err(&pdev->dev, "cannot set failsafe clockrate\n");
		goto out;
	}
	if (request_pins)
		gpd->pinmux(1);

	queue_plug(data);

out:
	return err;
}
/**
 * gpmi_release_hw - free the hardware
 * @pdev: pointer to platform device
 *
 * In opposite to gpmi_init_hw, release all acquired resources
 */
static void gpmi_release_hw(struct platform_device *pdev)
{
	struct gpmi_platform_data *gpd =
			(struct gpmi_platform_data *)pdev->dev.platform_data;
	struct lba_data *data = platform_get_drvdata(pdev);

	queue_release(data);
	clk_put(data->clk);
	gpd->pinmux(0);
}


/**
 * gpmi_alloc_buffers - allocate DMA buffers for one chip
 *
 * @pdev:	GPMI platform device
 * @g:		pointer to structure associated with NAND chip
 *
 * Allocate buffer using dma_alloc_coherent
 */
static int gpmi_alloc_buffers(struct platform_device *pdev,
			      struct gpmi_perchip_data *g)
{
	g->cmd_buffer = dma_alloc_coherent(&pdev->dev,
					   g->cmd_buffer_size,
					   &g->cmd_buffer_handle, GFP_DMA);
	if (!g->cmd_buffer)
		goto out1;

	g->write_buffer = dma_alloc_coherent(&pdev->dev,
					     g->write_buffer_size,
					     &g->write_buffer_handle, GFP_DMA);
	if (!g->write_buffer)
		goto out2;

	g->data_buffer = dma_alloc_coherent(&pdev->dev,
					    g->data_buffer_size,
					    &g->data_buffer_handle, GFP_DMA);
	if (!g->data_buffer)
		goto out3;

	g->oob_buffer = dma_alloc_coherent(&pdev->dev,
					   g->oob_buffer_size,
					   &g->oob_buffer_handle, GFP_DMA);
	if (!g->oob_buffer)
		goto out4;

	g->cmdtail_buffer = dma_alloc_coherent(&pdev->dev,
					   g->cmdtail_buffer_size,
					   &g->cmdtail_buffer_handle, GFP_DMA);
	if (!g->oob_buffer)
		goto out5;

	return 0;

out5:
	dma_free_coherent(&pdev->dev, g->oob_buffer_size,
			  g->oob_buffer, g->oob_buffer_handle);

out4:
	dma_free_coherent(&pdev->dev, g->data_buffer_size,
			  g->data_buffer, g->data_buffer_handle);
out3:
	dma_free_coherent(&pdev->dev, g->write_buffer_size,
			  g->write_buffer, g->write_buffer_handle);
out2:
	dma_free_coherent(&pdev->dev, g->cmd_buffer_size,
			  g->cmd_buffer, g->cmd_buffer_handle);
out1:
	return -ENOMEM;
}

/**
 * gpmi_free_buffers - free buffers allocated by gpmi_alloc_buffers
 *
 * @pdev:	platform device
 * @g:		pointer to structure associated with NAND chip
 *
 * Deallocate buffers on exit
 */
static void gpmi_free_buffers(struct platform_device *pdev,
			      struct gpmi_perchip_data *g)
{
	dma_free_coherent(&pdev->dev, g->oob_buffer_size,
			  g->oob_buffer, g->oob_buffer_handle);
	dma_free_coherent(&pdev->dev, g->write_buffer_size,
			  g->write_buffer, g->write_buffer_handle);
	dma_free_coherent(&pdev->dev, g->cmd_buffer_size,
			  g->cmd_buffer, g->cmd_buffer_handle);
	dma_free_coherent(&pdev->dev, g->cmdtail_buffer_size,
			  g->cmdtail_buffer, g->cmdtail_buffer_handle);
	dma_free_coherent(&pdev->dev, g->data_buffer_size,
			  g->data_buffer, g->data_buffer_handle);
}


/******************************************************************************
 * Arch specific chain_* callbaks
 ******************************************************************************/

/**
 * chain_w4r - Initialize descriptor to perform W4R operation
 *
 * @chain:	Descriptor to use
 * @cs:		CS for this operation
 *
 * Due to HW bug we have to put W4R into separate desc.
 */
static void chain_w4r(struct stmp3xxx_dma_descriptor *chain, int cs)
{
	chain->command->cmd =
		(4 << BP_APBH_CHn_CMD_CMDWORDS)	|
		BM_APBH_CHn_CMD_WAIT4ENDCMD	|
		BM_APBH_CHn_CMD_NANDWAIT4READY	|
		BM_APBH_CHn_CMD_NANDLOCK	|
		BM_APBH_CHn_CMD_CHAIN		|
		(BV_APBH_CHn_CMD_COMMAND__NO_DMA_XFER << BP_APBH_CHn_CMD_COMMAND);
	chain->command->pio_words[0] =
		(BV_GPMI_CTRL0_COMMAND_MODE__WAIT_FOR_READY << BP_GPMI_CTRL0_COMMAND_MODE) |
		BM_GPMI_CTRL0_WORD_LENGTH	|
		(BV_GPMI_CTRL0_ADDRESS__NAND_DATA << BP_GPMI_CTRL0_ADDRESS)   |
		(cs << BP_GPMI_CTRL0_CS);
	chain->command->pio_words[1] = 0;
	chain->command->pio_words[2] = 0;
	chain->command->pio_words[3] = 0;
	chain->command->buf_ptr = 0;
}

/**
 * chain_cmd - Initialize descriptor to push CMD to the bus
 *
 * @chain:	Descriptor to use
 * @cmd_handle: dma_addr_t pointer that holds the command
 * @lba_cmd:    flags and lenghth of this command.
 * @cs:		CS for this operation
 *
 * CLE || CLE+ALE
 */
static void chain_cmd(struct stmp3xxx_dma_descriptor *chain,
		      dma_addr_t cmd_handle,
		      struct lba_cmd *lba_cmd,
		      int cs)
{
	/* output command */
	chain->command->cmd =
		(lba_cmd->len << BP_APBH_CHn_CMD_XFER_COUNT) 	|
		(3 << BP_APBH_CHn_CMD_CMDWORDS) 		|
		BM_APBH_CHn_CMD_WAIT4ENDCMD			|
		BM_APBH_CHn_CMD_NANDLOCK			|
		(BV_APBH_CHn_CMD_COMMAND__DMA_READ << BP_APBH_CHn_CMD_COMMAND);
	chain->command->cmd |= BM_APBH_CHn_CMD_CHAIN;
	chain->command->pio_words[0] =
		(BV_GPMI_CTRL0_COMMAND_MODE__WRITE << BP_GPMI_CTRL0_COMMAND_MODE) |
		BM_GPMI_CTRL0_WORD_LENGTH			|
		BM_GPMI_CTRL0_LOCK_CS				|
		(cs << BP_GPMI_CTRL0_CS)			|
		(BV_GPMI_CTRL0_ADDRESS__NAND_CLE << BP_GPMI_CTRL0_ADDRESS)	|
		(lba_cmd->len << BP_GPMI_CTRL0_XFER_COUNT);
	chain->command->pio_words[1] = 0;
	chain->command->pio_words[2] = 0;
	chain->command->buf_ptr = cmd_handle;

	if (lba_cmd->flag & FE_CMD_INC)
		chain->command->pio_words[0] |=	BM_GPMI_CTRL0_ADDRESS_INCREMENT;
/*BUG 	if (lba_cmd->flag & FE_W4R) */
/* 		chain->command->cmd |= BM_APBH_CHn_CMD_NANDWAIT4READY; */
}

/**
 * chain_cmd - Initialize descriptor to read data from the bus
 *
 * @chain:	Descriptor to use
 * @data_handle: dma_addr_t pointer to buffer to store data
 * @data_len:    the size of the data buffer to read
 * @cmd_handle: dma_addr_t pointer that holds the command
 * @lba_cmd:    flags and lenghth of this command.
 * @cs:		CS for this operation
 */
static void chain_read_data(struct stmp3xxx_dma_descriptor *chain,
			     dma_addr_t data_handle,
			     dma_addr_t data_len,
			     struct lba_cmd *lba_cmd,
			     int cs)
{
	chain->command->cmd =
		(data_len << BP_APBH_CHn_CMD_XFER_COUNT)	|
		(4 << BP_APBH_CHn_CMD_CMDWORDS)			|
		BM_APBH_CHn_CMD_WAIT4ENDCMD			|
		BM_APBH_CHn_CMD_NANDLOCK			|
		BM_APBH_CHn_CMD_CHAIN				|
		(BV_APBH_CHn_CMD_COMMAND__DMA_WRITE << BP_APBH_CHn_CMD_COMMAND);
	chain->command->pio_words[0] =
		(BV_GPMI_CTRL0_COMMAND_MODE__READ << BP_GPMI_CTRL0_COMMAND_MODE)	|
		BM_GPMI_CTRL0_WORD_LENGTH			|
		BM_GPMI_CTRL0_LOCK_CS				|
		(cs << BP_GPMI_CTRL0_CS)			|
		(BV_GPMI_CTRL0_ADDRESS__NAND_DATA << BP_GPMI_CTRL0_ADDRESS) |
		(data_len << BP_GPMI_CTRL0_XFER_COUNT);

	chain->command->pio_words[1] = 0;
	chain->command->pio_words[2] = 0;
	chain->command->pio_words[3] = 0;
	chain->command->buf_ptr = data_handle;

	if (lba_cmd->flag & FE_CMD_INC)
		chain->command->pio_words[0] |=	BM_GPMI_CTRL0_ADDRESS_INCREMENT;
/*BUG 	if (lba_cmd->flag & FE_W4R) */
/* 		chain->command->cmd |= BM_APBH_CHn_CMD_NANDWAIT4READY; */

}

/**
 * chain_cmd - Initialize descriptor to read data from the bus
 *
 * @chain:	Descriptor to use
 * @data_handle: dma_addr_t pointer to buffer to read data from
 * @data_len:    the size of the data buffer to write
 * @cmd_handle: dma_addr_t pointer that holds the command
 * @lba_cmd:    flags and lenghth of this command.
 * @cs:		CS for this operation
 */
static void chain_write_data(struct stmp3xxx_dma_descriptor *chain,
			      dma_addr_t data_handle,
			      int data_len,
			      struct lba_cmd *lba_cmd,
			      int cs)
{

	chain->command->cmd =
		(data_len << BP_APBH_CHn_CMD_XFER_COUNT)	|
		(4 << BP_APBH_CHn_CMD_CMDWORDS)			|
		BM_APBH_CHn_CMD_WAIT4ENDCMD			|
		BM_APBH_CHn_CMD_NANDLOCK			|
		BM_APBH_CHn_CMD_CHAIN				|
		(BV_APBH_CHn_CMD_COMMAND__DMA_READ << BP_APBH_CHn_CMD_COMMAND);
	chain->command->pio_words[0] =
		(BV_GPMI_CTRL0_COMMAND_MODE__WRITE << BP_GPMI_CTRL0_COMMAND_MODE) |
		BM_GPMI_CTRL0_WORD_LENGTH			|
		BM_GPMI_CTRL0_LOCK_CS				|
		(cs << BP_GPMI_CTRL0_CS)			|
		(BV_GPMI_CTRL0_ADDRESS__NAND_DATA << BP_GPMI_CTRL0_ADDRESS) |
		(data_len << BP_GPMI_CTRL0_XFER_COUNT);

	chain->command->pio_words[1] = 0;
	chain->command->pio_words[2] = 0;
	chain->command->pio_words[3] = 0;
	chain->command->buf_ptr = data_handle;

	if (lba_cmd->flag & FE_CMD_INC)
		chain->command->pio_words[0] |=	BM_GPMI_CTRL0_ADDRESS_INCREMENT;
/*BUG 	if (lba_cmd->flag & FE_W4R) */
/* 		chain->command->cmd |= BM_APBH_CHn_CMD_NANDWAIT4READY; */

}


/******************************************************************************
 * Interface to arch independent part
 ******************************************************************************/
/**
 * queue_cmd - Setup a chain of descriptors
 *
 * @priv:	 private data passed
 * @cmd_buf:     pointer to command buffer (to be removed)
 * @cmd_handle:  dma_addr_t pointer that holds the command
 * @cmd_len:     the size of the command buffer (to be removed)
 * @data_handle: dma_addr_t pointer to a data buffer
 * @data_len:    the size of the data buffer
 * @cmd_flags:   commands flags
 */
int queue_cmd(void *priv,
	      uint8_t *cmd_buf, dma_addr_t cmd_handle, int cmd_len,
	      dma_addr_t data, int data_len,
	      struct lba_cmd *cmd_flags)
{

	struct gpmi_perchip_data *g = priv;
	unsigned long flags;
	int idx;
	int ret = 0;
	struct stmp3xxx_dma_descriptor *chain ;
	int i;

	if (!g || !(cmd_buf || cmd_handle))
		BUG();

	spin_lock_irqsave(&g->lock, flags);

	/* Keep it for debug purpose */
	chain = g->d;
	for (i = g->d_tail; i < GPMI_DMA_MAX_CHAIN; i++) {
		chain[i].command->cmd = 0;
		chain[i].command->buf_ptr = 0;
	}
	/* End */

	if (!cmd_handle) {
		if (!cmd_buf)
			BUG();
		memcpy(g->cmd_buffer, cmd_buf, cmd_len);
		cmd_handle = g->cmd_buffer_handle;
	}

	idx = g->d_tail;
	chain = &g->d[idx];

	do {
		if (!cmd_flags)
			BUG();

		if (cmd_flags->flag & FE_W4R) {
			/* there seems to be HW BUG with W4R flag.
			 * IRQ controller hangs forever when it's combined
			 * with real operation.
			 */
			chain_w4r(chain, g->cs);
			chain++; idx++;
		}


		switch (cmd_flags->flag & F_MASK) {

		case F_CMD:
			chain_cmd(chain, cmd_handle, cmd_flags, g->cs);
			break;
		case F_DATA_READ:
			chain_read_data(chain, data, data_len,
					cmd_flags, g->cs);
			break;
		case F_DATA_WRITE:
			chain_write_data(chain, data, data_len,
					 cmd_flags, g->cs);
			break;
		default:{
			if (cmd_flags->flag & FE_END)
				goto out;
			else{
				printk(KERN_ERR "uknown cmd\n");
				BUG();
			}
		}
		}


		chain++; idx++;
		cmd_handle += cmd_flags->len;

		if (idx >= GPMI_DMA_MAX_CHAIN) {
			printk(KERN_ERR "to many chains; idx is 0x%x\n", idx);
			BUG();
		}

	} while (!((cmd_flags++)->flag & FE_END));

out:
	if (idx < GPMI_DMA_MAX_CHAIN) {
		ret = idx;
		g->d_tail = idx;
	}
	spin_unlock_irqrestore(g->lock, flags);

	return ret;

}

dma_addr_t queue_get_cmd_handle(void *priv)
{
	struct gpmi_perchip_data *g = priv;
	return g->cmd_buffer_handle;
}

uint8_t *queue_get_cmd_ptr(void *priv)
{
	struct gpmi_perchip_data *g = priv;
	return g->cmd_buffer;
}

dma_addr_t queue_get_data_handle(void *priv)
{
	struct gpmi_perchip_data *g = priv;
	return g->data_buffer_handle;
}

uint8_t *queue_get_data_ptr(void *priv)
{
	struct gpmi_perchip_data *g = priv;
	return g->data_buffer;
}


/**
 * queue_run - run the chain
 *
 * @priv:	 private data.
 */
int queue_run(void *priv)
{
	struct gpmi_perchip_data *g = priv;

	if (!g->d_tail)
		return 0;
	stmp3xxx_dma_reset_channel(g->dma_ch);
	stmp3xxx_dma_clear_interrupt(g->dma_ch);
	stmp3xxx_dma_enable_interrupt(g->dma_ch);

	g->d[g->d_tail-1].command->cmd &= ~(BM_APBH_CHn_CMD_NANDLOCK |
				       BM_APBH_CHn_CMD_CHAIN);
	g->d[g->d_tail-1].command->cmd |= BM_APBH_CHn_CMD_IRQONCMPLT ;

	g->d[g->d_tail-1].command->pio_words[0] &= ~BM_GPMI_CTRL0_LOCK_CS;

#ifdef DEBUG
	/*stmp37cc_dma_print_chain(&g->chain);*/
#endif

	init_completion(&g->done);
	stmp3xxx_dma_go(g->dma_ch, g->d, 1);
	wait_for_completion(&g->done);

	g->d_tail = 0;

	return 0;

}

/******************************************************************************
 * Platform specific part / chard driver and misc functions
 ******************************************************************************/



static int __init lba_probe(struct platform_device *pdev)
{
	struct lba_data *data;
	struct resource *r;
	struct gpmi_perchip_data *g;
	int err;

	/* Allocate memory for the device structure (and zero it) */
	data = kzalloc(sizeof(*data) + sizeof(struct gpmi_perchip_data),
		       GFP_KERNEL);
	if (!data) {
		dev_err(&pdev->dev, "failed to allocate gpmi_nand_data\n");
		err = -ENOMEM;
		goto out1;
	}
	g_data = data;
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		dev_err(&pdev->dev, "failed to get resource\n");
		err = -ENXIO;
		goto out2;
	}
	data->io_base = ioremap(r->start, r->end - r->start + 1);
	if (!data->io_base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		err = -EIO;
		goto out2;
	}

	r = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!r) {
		err = -EIO;
		dev_err(&pdev->dev, "can't get IRQ resource\n");
		goto out3;
	}
	data->irq = r->start;

	platform_set_drvdata(pdev, data);
	err = gpmi_init_hw(pdev, 1);
	if (err)
		goto out3;


	err = request_irq(data->irq,
			  gpmi_irq, 0, dev_name(&pdev->dev), data);
	if (err) {
		dev_err(&pdev->dev, "can't request GPMI IRQ\n");
		goto out4;
	}

	g = data->nand;

	r = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	if (!r) {
		dev_err(&pdev->dev, "can't get DMA resource\n");
		goto out_res;
	}
	g->cs = 0;
	g->dma_ch = r->start;

	err = stmp3xxx_dma_request(g->dma_ch, NULL, dev_name(&pdev->dev));
	if (err) {
		dev_err(&pdev->dev, "can't request DMA channel 0x%x\n",
			g->dma_ch);
		goto out_res;
	}

	err = stmp3xxx_dma_make_chain(g->dma_ch, &g->chain,
				      g->d, ARRAY_SIZE(g->d));
	if (err) {
		dev_err(&pdev->dev, "can't setup DMA chain\n");
		stmp3xxx_dma_release(g->dma_ch);
		goto out_res;
	}

	g->cmd_buffer_size = GPMI_CMD_BUF_SZ;
	g->cmdtail_buffer_size = GPMI_CMD_BUF_SZ;
	g->write_buffer_size = GPMI_WRITE_BUF_SZ;
	g->data_buffer_size = GPMI_DATA_BUF_SZ;
	g->oob_buffer_size = GPMI_OOB_BUF_SZ;

	err = gpmi_alloc_buffers(pdev, g);
	if (err) {
		dev_err(&pdev->dev, "can't setup buffers\n");
		stmp3xxx_dma_free_chain(&g->chain);
		stmp3xxx_dma_release(g->dma_ch);
		goto out_res;
	}

	g->dev = pdev;
	g->chip.priv = g;
	g->index = 0;
	g->timing = gpmi_safe_timing;

	g->cmd_buffer_sz =
		g->write_buffer_sz =
		g->data_buffer_sz =
		0;
	g->valid = !0;	/* mark the data as valid */


	lba_core_init(data);

	return 0;

out_res:
	free_irq(data->irq, data);
out4:
	gpmi_release_hw(pdev);
out3:
	platform_set_drvdata(pdev, NULL);
	iounmap(data->io_base);
out2:
	kfree(data);
out1:
	return err;
}

static int gpmi_suspend(struct platform_device *pdev, pm_message_t pm)
{
	struct lba_data *data = platform_get_drvdata(pdev);
	int err;

	printk(KERN_INFO "%s: %d\n", __func__, __LINE__);
	err = lba_core_suspend(pdev, data);
	if (!err)
		gpmi_release_hw(pdev);

	return err;
}

static int gpmi_resume(struct platform_device *pdev)
{
	struct lba_data *data = platform_get_drvdata(pdev);
	int r;

	printk(KERN_INFO "%s: %d\n", __func__, __LINE__);
	r = gpmi_init_hw(pdev, 1);
	lba_core_resume(pdev, data);
	return r;
}

/**
 * gpmi_nand_remove - remove a GPMI device
 *
 */
static int __devexit lba_remove(struct platform_device *pdev)
{
	struct lba_data *data = platform_get_drvdata(pdev);
	int i;

	lba_core_remove(data);
	gpmi_release_hw(pdev);
	free_irq(data->irq, data);

	for (i = 0; i < max_chips; i++) {
		if (!data->nand[i].valid)
			continue;
		gpmi_free_buffers(pdev, &data->nand[i]);
		stmp3xxx_dma_free_chain(&data->nand[i].chain);
		stmp3xxx_dma_release(data->nand[i].dma_ch);
	}
	iounmap(data->io_base);
	kfree(data);

	return 0;
}

static struct platform_driver lba_driver = {
	.probe = lba_probe,
	.remove = __devexit_p(lba_remove),
	.driver = {
		   .name = "gpmi",
		   .owner = THIS_MODULE,
	},
	.suspend = gpmi_suspend,
	.resume = gpmi_resume,

};


static int __init lba_init(void)
{
	return platform_driver_register(&lba_driver);
}

static void lba_exit(void)
{
	platform_driver_unregister(&lba_driver);
}

module_init(lba_init);
module_exit(lba_exit);
MODULE_LICENSE("GPL");
