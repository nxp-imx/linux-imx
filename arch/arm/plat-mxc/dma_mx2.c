/*
 * Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*  Front-end to the DMA handling.  This handles the allocation/freeing
 *  of DMA channels, and provides a unified interface to the machines
 *  DMA facilities.
 */

/*!
 * @file plat-mxc/dma_mx2.c
 * @brief This file contains functions for DMA  API
 *
 * @ingroup DMA_MX27
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/mman.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/clk.h>

#include <linux/proc_fs.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <mach/dma.h>
#include <asm/delay.h>

#include <asm/atomic.h>

/* commented temperily for mx27 compilation
#define DMA_PM
*/
#ifdef DMA_PM
#include <linux/pm.h>
#include <mach/apmc.h>
struct apmc_user *dma_apmc_user;
struct pm_dev *dma_pm;
#define DMA_PMST_RESUME		0
#define DMA_PMST_STANDBY		1
#define DMA_PMST_SUSPEND		2
static unsigned int dma_pm_status = DMA_PMST_RESUME;
#endif

/*!
 * This variable is used to controll the clock of DMA.
 * It counts the number of actived channels
 */
static atomic_t g_dma_actived = ATOMIC_INIT(0);

/*!
 * This variable point a proc file which contains the information
 *	of DMA channels
 */
static struct proc_dir_entry *g_proc_dir;

/*!
 * The dma channels
 */
static mxc_dma_channel_t g_dma_channels[MAX_DMA_CHANNELS];
static mx2_dma_priv_t g_dma_privates[MXC_DMA_CHANNELS];
static mx2_dma_bd_t g_dma_bd_table[MXC_DMA_CHANNELS][MAX_BD_SIZE];

static DEFINE_SPINLOCK(dma_list_lock);

static struct clk *dma_clk;

/*!@brief flush buffer descriptor ring*/
#define flush_dma_bd(private) \
		{ \
			atomic_set(&(private->bd_used), 0); \
			private->bd_rd = private->bd_wr;\
		}

/*!@brief get next buffer discriptor */
#define next_dma_bd(private) \
		({ \
			int bd_next = (private->bd_rd+1)%MAX_BD_SIZE; \
			(bd_next ==  private->bd_wr) ? NULL: private->bd_ring+bd_next;\
		})

static inline int consume_dma_bd(mxc_dma_channel_t * dma, int error);
/*!
 *@brief allocate a dma channel.
 *
 *@param idx   Requested channel NO.
 *                @li MXC_INVLAID_CHANNEL     System allocates a free channel which is not statically allocated.
 *                @li Others     User requests a specific channel
 *@return        @li MXC_INVLAID_CHANNEL Failure
 *               @li Others      Success
 */
static inline int get_dma_channel(int idx)
{
	int i;
	mxc_dma_channel_t *p;

	if ((idx >= MAX_DMA_CHANNELS) && (idx != MXC_DMA_DYNAMIC_CHANNEL)) {
		return -1;
	}
	if (idx != MXC_DMA_DYNAMIC_CHANNEL) {
		p = g_dma_channels + idx;
		BUG_ON(p->dynamic != 0);
		if (xchg(&p->lock, 1) != 0) {
			return -1;
		}
		return idx;
	}

	p = g_dma_channels;
	for (i = 0; (i < MAX_DMA_CHANNELS); i++, p++) {
		if (p->dynamic && (xchg(&p->lock, 1) == 0)) {
			return i;
		}
	}
	return -1;
}

/*!
 *@brief release a dma channel.
 *
 *@param idx   channel number
 *@return        none;
 */
static inline void put_dma_channel(int idx)
{
	mxc_dma_channel_t *p;

	if ((idx < MAX_DMA_CHANNELS) && (idx >= 0)) {
		p = g_dma_channels + idx;
		(void)xchg(&p->lock, 0);
	}
}

/*!
 *@brief Get dma list for /proc/dma
 */
static int mxc_get_dma_list(char *buf)
{
	mxc_dma_channel_t *dma;
	char *p = buf;
	int i;

	for (i = 0, dma = g_dma_channels; i < MAX_DMA_CHANNELS; i++, dma++) {
		if (dma->lock) {
			p += sprintf(p, "dma channel %2d: %s\n", i,
				     dma->dev_name ? dma->dev_name : "unknown");
		} else {
			p += sprintf(p, "dma channel %2d: unused\n", i);
		}
	}

	return p - buf;
}

/*!@brief save the mask of dma interrupts*/
#define save_dma_interrupt(flags) \
		flags = __raw_readl(IO_ADDRESS(DMA_BASE_ADDR) + DMA_DIMR)

/*!@brief restore the mask of dma interrupts*/
#define restore_dma_interrupt(flags) \
		__raw_writel(flags, IO_ADDRESS(DMA_BASE_ADDR) + DMA_DIMR)

/*!@brief disable interrupt of dma channel*/
static inline void mask_dma_interrupt(int channel)
{
	unsigned long reg;
	save_dma_interrupt(reg);
	reg |= 1 << channel;	/*mask interrupt; */
	restore_dma_interrupt(reg);
}

/*!@brief enable interrupt of dma channel */
static inline void unmask_dma_interrupt(int channel)
{
	unsigned long reg;
	save_dma_interrupt(reg);
	reg &= ~(1 << channel);	/*unmask interrupt; */
	restore_dma_interrupt(reg);
}

/*!@brief get interrupt event of dma channel */
static unsigned long inline __get_dma_interrupt(int channel)
{
	unsigned long mode;
	mode = 0;
	if (__raw_readl(IO_ADDRESS(DMA_BASE_ADDR) + DMA_DISR) & (1 << channel))
		mode |= DMA_DONE;

	if (__raw_readl(IO_ADDRESS(DMA_BASE_ADDR) + DMA_DBTOSR) &
	    (1 << channel))
		mode |= DMA_BURST_TIMEOUT;
	if (__raw_readl(IO_ADDRESS(DMA_BASE_ADDR) + DMA_DSESR) & (1 << channel))
		mode |= DMA_TRANSFER_ERROR;

	if (__raw_readl(IO_ADDRESS(DMA_BASE_ADDR) + DMA_DBOSR) & (1 << channel))
		mode |= DMA_BUFFER_OVERFLOW;
	if (__raw_readl(IO_ADDRESS(DMA_BASE_ADDR) + DMA_DRTOSR) &
	    (1 << channel))
		mode |= DMA_REQUEST_TIMEOUT;
	return mode;
}

/*!
 *@brief clean all event of dma interrupt and return the valid event.
 */
static unsigned long inline __clear_dma_interrupt(int channel)
{
	unsigned long mode;
	mode = __get_dma_interrupt(channel);
	__raw_writel(1 << channel, IO_ADDRESS(DMA_BASE_ADDR) + DMA_DISR);
	__raw_writel(1 << channel, IO_ADDRESS(DMA_BASE_ADDR) + DMA_DBTOSR);
	__raw_writel(1 << channel, IO_ADDRESS(DMA_BASE_ADDR) + DMA_DRTOSR);
	__raw_writel(1 << channel, IO_ADDRESS(DMA_BASE_ADDR) + DMA_DSESR);
	__raw_writel(1 << channel, IO_ADDRESS(DMA_BASE_ADDR) + DMA_DBOSR);

	return mode;
}

/*!@brief This function enables dma clocks without lock */
static void inline __enable_dma_clk(void)
{
	unsigned long reg;
	clk_enable(dma_clk);
	reg = __raw_readl(IO_ADDRESS(DMA_BASE_ADDR) + DMA_DCR);
	reg |= 0x1;
	__raw_writel(reg, IO_ADDRESS(DMA_BASE_ADDR) + DMA_DCR);
}

/*!@brief This function disables dma clocks without lock */
static void inline __disable_dma_clk(void)
{
	unsigned long reg;
	reg = __raw_readl(IO_ADDRESS(DMA_BASE_ADDR) + DMA_DCR);
	reg &= ~0x1;
	__raw_writel(reg, IO_ADDRESS(DMA_BASE_ADDR) + DMA_DCR);
	clk_disable(dma_clk);
}

/*!@brief This function enables dma clocks with lock */
static void inline enable_dma_clk(void)
{
	unsigned long flags;
	spin_lock_irqsave(&dma_list_lock, flags);
	if (atomic_read(&g_dma_actived) == 0) {
		__enable_dma_clk();
	}
	spin_unlock_irqrestore(&dma_list_lock, flags);
	return;
}

/*!@brief This function disables dma clocks without locked */
static void inline disable_dma_clk(void)
{
	unsigned long flags;
	spin_lock_irqsave(&dma_list_lock, flags);
	if (atomic_read(&g_dma_actived) == 0) {
		__disable_dma_clk();
	}
	spin_unlock_irqrestore(&dma_list_lock, flags);
	return;
}

/*!@brief select a buffer to transfer and
 * 	setup dma channel for current transfer
 */
static void setup_dmac(mxc_dma_channel_t * dma)
{
	mx2_dma_priv_t *priv = (mx2_dma_priv_t *) dma->private;
	dma_regs_t *dma_base = (dma_regs_t *) (priv->dma_base);
	mx2_dma_bd_t *p, *q;
	unsigned long ctrl_val;

	if (dma->active == 0) {
		printk(KERN_ERR
		       "dma channel %d is not enabled, when receiving this channel 's interrupt\n",
		       dma->channel);
		return;
	}
	if (atomic_read(&(priv->bd_used)) <= 0) {
		printk(KERN_ERR "dma channel %d is empty\n", dma->channel);
		dma->active = 0;
		atomic_dec(&g_dma_actived);
		return;
	}
	/* BUSY: transfering
	 * PEND: Wait for set to DMAC.
	 * s1: no transfering:
	 *      set first(one BUSY). if there are more than one tranfer. set second &repeat is enabled(two BUSY).
	 *
	 * s2: transfering & just on transfer
	 *      one BUSY. set the tranesfer and set repeat bit(two BUSY)
	 * s3: transfering & repeat has set
	 *     has two BUSY.
	 */
	p = priv->bd_ring + priv->bd_rd;
	q = next_dma_bd(priv);
	if (!(p->state & DMA_BD_ST_BUSY)) {
		/*NOTICE:: This is first buffer or dma chain does not support chain-buffer. So CEN must clear & set again */
		ctrl_val =
		    __raw_readl(&(dma_base->Ctl)) &
		    (~(DMA_CTL_ACRPT | DMA_CTL_RPT | DMA_CTL_CEN));
		__raw_writel(ctrl_val, &(dma_base->Ctl));
		if (p->mode != dma->mode) {
			dma->mode = p->mode;	/* bi-dir channel do mode change */
			if (dma->mode == MXC_DMA_MODE_READ) {
				DMA_CTL_SET_SMOD(ctrl_val,
						 priv->dma_info->sourceType);
				DMA_CTL_SET_SSIZ(ctrl_val,
						 priv->dma_info->sourcePort);
				DMA_CTL_SET_DMOD(ctrl_val,
						 priv->dma_info->destType);
				DMA_CTL_SET_DSIZ(ctrl_val,
						 priv->dma_info->destPort);
			} else {
				DMA_CTL_SET_SMOD(ctrl_val,
						 priv->dma_info->destType);
				DMA_CTL_SET_SSIZ(ctrl_val,
						 priv->dma_info->destPort);
				DMA_CTL_SET_DMOD(ctrl_val,
						 priv->dma_info->sourceType);
				DMA_CTL_SET_DSIZ(ctrl_val,
						 priv->dma_info->sourcePort);
			}
		}
		__raw_writel(p->src_addr, &(dma_base->SourceAddr));
		__raw_writel(p->dst_addr, &(dma_base->DestAddr));
		__raw_writel(p->count, &(dma_base->Count));
		p->state |= DMA_BD_ST_BUSY;
		p->state &= ~(DMA_BD_ST_PEND);
		ctrl_val |= DMA_CTL_CEN;
		__raw_writel(ctrl_val, &(dma_base->Ctl));
		if (q && priv->dma_chaining) {	/*DO chain-buffer */
			__raw_writel(q->src_addr, &(dma_base->SourceAddr));
			__raw_writel(q->dst_addr, &(dma_base->DestAddr));
			__raw_writel(q->count, &(dma_base->Count));
			q->state |= DMA_BD_ST_BUSY;
			q->state &= ~(DMA_BD_ST_PEND);
			ctrl_val |= DMA_CTL_ACRPT | DMA_CTL_RPT | DMA_CTL_CEN;
			__raw_writel(ctrl_val, &(dma_base->Ctl));
		}
	} else {		/* Just dma channel which supports dma buffer can run to there */
		BUG_ON(!priv->dma_chaining);
		if (q) {	/* p is tranfering, then q must be set into dma controller */
			/*WARNING:: [1] dangerous area begin.
			 *      If the p is completed during MCU run in this erea, the dma channel is crashed.
			 */
			__raw_writel(q->src_addr, &(dma_base->SourceAddr));
			__raw_writel(q->dst_addr, &(dma_base->DestAddr));
			__raw_writel(q->count, &(dma_base->Count));
			/*WARNING:: [2] dangerous area end */
			ctrl_val =
			    __raw_readl(&(dma_base->Ctl)) | (DMA_CTL_ACRPT |
							     DMA_CTL_RPT |
							     DMA_CTL_CEN);
			__raw_writel(ctrl_val, &(dma_base->Ctl));

			/* WARNING:: This is workaround and it is dangerous:
			 *      the judgement is not safety.
			 */
			if (!__get_dma_interrupt(dma->channel)) {
				q->state |= DMA_BD_ST_BUSY;
				q->state &= ~(DMA_BD_ST_PEND);
			} else {
				/*Waiting re-enable is in ISR */
				printk(KERN_ERR
				       "Warning:: The privous transfer is completed. Maybe the chain buffer is stopped.");
			}
		} else {	/* Last buffer is transfering: just clear RPT bit */
			ctrl_val =
			    __raw_readl(&(dma_base->Ctl)) &
			    (~(DMA_CTL_ACRPT | DMA_CTL_RPT));
			__raw_writel(ctrl_val, &(dma_base->Ctl));
		}
	}
}

/*!
 * @brief interrupt handler of dma channel
 */
static irqreturn_t dma_irq_handler(int irq, void *dev_id)
{
	mxc_dma_channel_t *dma = (mxc_dma_channel_t *) dev_id;
	mx2_dma_priv_t *priv = (mx2_dma_priv_t *) (dma ? dma->private : NULL);
	dma_regs_t *dma_base;
	int state, error = MXC_DMA_DONE;

	BUG_ON(priv == NULL);

	dma_base = (dma_regs_t *) priv->dma_base;

	state = __clear_dma_interrupt(dma->channel);

	priv->trans_bytes += dma_base->transferd;
	if (state != DMA_DONE) {
		if (state & DMA_REQUEST_TIMEOUT) {
			error = MXC_DMA_REQUEST_TIMEOUT;
		} else {
			error = MXC_DMA_TRANSFER_ERROR;
		}
	}
	if (consume_dma_bd(dma, error)) {
		disable_dma_clk();
		if (dma->cb_fn) {
			dma->cb_fn(dma->cb_args, error, priv->trans_bytes);
		}
		priv->trans_bytes = 0;
	} else {
		disable_dma_clk();
	}
	return IRQ_HANDLED;
}

/*!
 *@brief	Set DMA channel parameters
 *
 *@param	dma  Requested channel NO.
 *@param	dma_info  Channel configuration
 *@return	@li 0        	Success
 *           	@li others 	Failure
 */
static int setup_dma_channel(mxc_dma_channel_t * dma, mx2_dma_info_t * dma_info)
{
	mx2_dma_priv_t *priv = (mx2_dma_priv_t *) (dma ? dma->private : NULL);
	dma_regs_t *dma_base;
	unsigned long reg;

	if (!dma_info || !priv) {
		return -1;
	}

	if (dma_info->sourceType > 3) {
		return -1;
	}
	if (dma_info->destType > 3) {
		return -1;
	}
	if (dma_info->destPort > 3) {
		return -1;
	}
	if (dma_info->sourcePort > 3) {
		return -1;
	}
	if (dma_info->M2D_Valid) {
		/*add for second dma */
		if (dma_info->W < dma_info->X) {
			return -1;
		}
	}

	priv->dma_chaining = dma_info->dma_chaining;
	priv->ren = dma_info->ren;

	if (dma_info->sourceType != DMA_TYPE_FIFO
	    && dma_info->destType != DMA_TYPE_FIFO) {
		if (dma_info->ren) {
			printk(KERN_INFO
			       "Warning:request enable just affect source or destination port is FIFO !\n");
			priv->ren = 0;
		}
	}

	if (dma_info->M2D_Valid) {
		if (dma_info->msel) {
			__raw_writel(dma_info->W,
				     IO_ADDRESS(DMA_BASE_ADDR) + DMA_WSRB);
			__raw_writel(dma_info->X,
				     IO_ADDRESS(DMA_BASE_ADDR) + DMA_XSRB);
			__raw_writel(dma_info->Y,
				     IO_ADDRESS(DMA_BASE_ADDR) + DMA_YSRB);

		} else {
			__raw_writel(dma_info->W,
				     IO_ADDRESS(DMA_BASE_ADDR) + DMA_WSRA);
			__raw_writel(dma_info->X,
				     IO_ADDRESS(DMA_BASE_ADDR) + DMA_XSRA);
			__raw_writel(dma_info->Y,
				     IO_ADDRESS(DMA_BASE_ADDR) + DMA_YSRA);
		}
	}

	dma_base = (dma_regs_t *) (priv->dma_base);

	__raw_writel(dma_info->burstLength, &(dma_base->BurstLength));
	__raw_writel(dma_info->request, &(dma_base->RequestSource));

	if (dma_info->ren) {
		reg = dma_info->busuntils & 0x1FFFF;
		if (dma_info->rto_en) {
			reg |= 0xE000;
		}
		__raw_writel(reg, &(dma_base->BusUtilt));
	} else {
		__raw_writel(dma_info->busuntils, &(dma_base->BusUtilt));
	}

	reg = __raw_readl(&(dma_base->Ctl)) & (~(DMA_CTL_ACRPT | DMA_CTL_RPT));

	if (dma_info->dir) {
		reg |= DMA_CTL_MDIR;
	} else {
		reg &= ~DMA_CTL_MDIR;
	}

	if (priv->ren) {
		reg |= DMA_CTL_REN;
	} else {
		reg &= ~DMA_CTL_REN;
	}

	if ((dma_info->M2D_Valid) && (dma_info->msel)) {
		reg |= DMA_CTL_MSEL;
	} else {
		reg &= ~DMA_CTL_MSEL;
	}

	if (dma_info->mode) {
		DMA_CTL_SET_SMOD(reg, dma_info->destType);
		DMA_CTL_SET_SSIZ(reg, dma_info->destPort);
		DMA_CTL_SET_DMOD(reg, dma_info->sourceType);
		DMA_CTL_SET_DSIZ(reg, dma_info->sourcePort);
	} else {
		DMA_CTL_SET_SMOD(reg, dma_info->sourceType);
		DMA_CTL_SET_SSIZ(reg, dma_info->sourcePort);
		DMA_CTL_SET_DMOD(reg, dma_info->destType);
		DMA_CTL_SET_DSIZ(reg, dma_info->destPort);
	}

	__raw_writel(reg, &(dma_base->Ctl));

	__clear_dma_interrupt(dma->channel);
	unmask_dma_interrupt(dma->channel);

	disable_dma_clk();
	return 0;
}

/*!@brief setup interrupt and setup dma channel by dma parameter  */
static inline int __init_dma_channel(mxc_dma_channel_t * chan,
				     mx2_dma_info_t * dma_info)
{
	mx2_dma_priv_t *dma_private = (mx2_dma_priv_t *) chan->private;
	dma_regs_t *dma_base;
	int ret;

	mask_dma_interrupt(chan->channel);
	ret =
	    request_irq(dma_private->dma_irq, dma_irq_handler,
			IRQF_DISABLED | IRQF_SHARED, chan->dev_name,
			(void *)chan);
	if (ret) {
		printk(KERN_ERR
		       "%s: unable to request IRQ %d for DMA channel\n",
		       chan->dev_name, dma_private->dma_irq);
		return ret;
	}

	enable_dma_clk();

	dma_base = (dma_regs_t *) (dma_private->dma_base);
	__raw_writel(0, &(dma_base->Ctl));

	ret = 0;
	if ((ret = setup_dma_channel(chan, dma_info))) {
		free_irq(dma_private->dma_irq, (void *)chan);
	}
	disable_dma_clk();
	return 0;
}

/*!@brief initialize buffer descriptor ring.*/
static inline void init_dma_bd(mx2_dma_priv_t * private)
{
	int i;
	mx2_dma_bd_t *pbd;
	private->bd_rd = private->bd_wr = 0;
	atomic_set(&(private->bd_used), 0);
	for (i = 0, pbd = private->bd_ring; i < MAX_BD_SIZE; i++, pbd++) {
		pbd->state = 0;
	}
}

/*!@brief add dma buffer into buffer descriptor ring */
static inline int fill_dma_bd(mxc_dma_channel_t * dma,
			      mxc_dma_requestbuf_t * buf, int num,
			      mxc_dma_mode_t mode)
{
	int i, wr;
	unsigned long flags, mask;
	mx2_dma_priv_t *priv = dma->private;
	mx2_dma_bd_t *p, *q;

	if ((atomic_read(&(priv->bd_used)) + num) > MAX_BD_SIZE) {
		return -EBUSY;
	}

	for (i = 0; i < num; i++) {
		wr = priv->bd_wr;
		p = priv->bd_ring + wr;
		p->mode = mode;
		p->count = buf[i].num_of_bytes;
		p->src_addr = buf[i].src_addr;
		p->dst_addr = buf[i].dst_addr;
		if (i == num - 1) {
			p->state = DMA_BD_ST_LAST | DMA_BD_ST_PEND;
		} else {
			p->state = DMA_BD_ST_PEND;
		}
		priv->bd_wr = (wr + 1) % MAX_BD_SIZE;
		atomic_inc(&(priv->bd_used));

		if (atomic_read(&(priv->bd_used)) != 2)
			continue;
		/* Disable interrupt of this channel */
		local_irq_save(flags);
		local_irq_disable();
		save_dma_interrupt(mask);
		mask_dma_interrupt(dma->channel);
		local_irq_restore(flags);
		/*TODO ::
		 *  If channel is transfering and supports chain_buffer,
		 *  when the new buffer is 2st buffer , repeat must be enabled
		 */
		if (priv->dma_chaining && dma->active) {
			q = priv->bd_ring + priv->bd_rd;
			if (q && (q->state & DMA_BD_ST_BUSY)) {
				if (atomic_read(&(priv->bd_used)) == 2) {
					setup_dmac(dma);
				}
			}
		}
		restore_dma_interrupt(mask);
	}
	return 0;
}

/*!@brief add sg-list into buffer descriptor ring */
static inline int fill_dma_bd_by_sg(mxc_dma_channel_t * dma,
				    struct scatterlist *sg, int num,
				    int real_bytes, mxc_dma_mode_t mode)
{
	int i, wr, total_bytes = real_bytes;
	unsigned long flags, mask;
	mx2_dma_priv_t *priv = dma->private;
	mx2_dma_bd_t *p, *q;
	if ((atomic_read(&(priv->bd_used)) + num) > MAX_BD_SIZE) {
		return -EBUSY;
	}

	for (i = 0; i < num && ((real_bytes <= 0) || (total_bytes > 0)); i++) {
		wr = priv->bd_wr;
		p = priv->bd_ring + wr;
		p->mode = mode;
		if (real_bytes > 0) {
			if (sg[i].length >= total_bytes) {
				p->count = total_bytes;
			} else {
				p->count = sg[i].length;
			}
			total_bytes -= p->count;
		} else {
			p->count = sg[i].length;
		}
		if (mode == MXC_DMA_MODE_READ) {
			p->src_addr = priv->dma_info->per_address;
			p->dst_addr = sg[i].dma_address;
		} else {
			p->dst_addr = priv->dma_info->per_address;
			p->src_addr = sg[i].dma_address;
		}
		if ((i == num - 1) || ((real_bytes > 0) && (total_bytes == 0))) {
			p->state = DMA_BD_ST_LAST | DMA_BD_ST_PEND;
		} else {
			p->state = DMA_BD_ST_PEND;
		}
		priv->bd_wr = (wr + 1) % MAX_BD_SIZE;
		atomic_inc(&(priv->bd_used));

		if (atomic_read(&(priv->bd_used)) != 2)
			continue;
		/* Disable interrupt of this channel */
		local_irq_save(flags);
		local_irq_disable();
		save_dma_interrupt(mask);
		mask_dma_interrupt(dma->channel);
		local_irq_restore(flags);
		/*TODO ::
		 *  If channel is transfering and supports chain_buffer,
		 *  when the new buffer is 2st buffer , repeat must be enabled
		 */
		if (priv->dma_chaining && dma->active) {
			q = next_dma_bd(priv);
			if (q && (q->state & DMA_BD_ST_BUSY)) {
				if ((atomic_read(&(priv->bd_used))) == 2) {
					setup_dmac(dma);
				}
			}
		}
		restore_dma_interrupt(mask);
	}
	return 0;
}

/*!@brief select next buffer descripter to transfer.
 *	return 1: need call call-back function. 0: Not need call call-back.
 *	it just is called in ISR
 */
static inline int consume_dma_bd(mxc_dma_channel_t * dma, int error)
{
	mx2_dma_priv_t *priv = dma->private;
	mx2_dma_bd_t *p;
	int notify = 0;
	if (priv == NULL) {
		printk(KERN_ERR
		       "request dma channel %d which is not initialize completed.!\n",
		       dma->channel);
		return 1;
	}
	if (error != MXC_DMA_DONE) {
		for (p = priv->bd_ring + priv->bd_rd;
		     atomic_read(&(priv->bd_used)) > 0;) {
			priv->bd_rd = (priv->bd_rd + 1) % MAX_BD_SIZE;
			atomic_dec(&(priv->bd_used));
			if (p->state & DMA_BD_ST_LAST) {
				p->state = 0;
				break;
			}
			p->state = 0;
		}
		notify = 1;
	} else {
		p = priv->bd_ring + priv->bd_rd;
		priv->bd_rd = (priv->bd_rd + 1) % MAX_BD_SIZE;
		atomic_dec(&(priv->bd_used));
		notify = (p->state & DMA_BD_ST_LAST) == DMA_BD_ST_LAST;
	}
	if (atomic_read(&(priv->bd_used)) <= 0) {
		dma->active = 0;
		atomic_dec(&g_dma_actived);
	} else {
		setup_dmac(dma);
	}
	return notify;
}

/*!
 * This function is generally called by the driver at open time.
 * The DMA driver would do any initialization steps that is required
 * to get the channel ready for data transfer.
 *
 * @param channel_id   a pre-defined id. The peripheral driver would specify
 *                     the id associated with its peripheral. This would be
 *                     used by the DMA driver to identify the peripheral
 *                     requesting DMA and do the necessary setup on the
 *                     channel associated with the particular peripheral.
 *                     The DMA driver could use static or dynamic DMA channel
 *                     allocation.
 * @param dev_name     module name or device name
 * @return returns a negative number on error if request for a DMA channel did not
 *         succeed, returns the channel number to be used on success.
 */
int mxc_dma_request_ext(mxc_dma_device_t channel_id, char *dev_name,
			struct dma_channel_info *info)
{
	mxc_dma_channel_t *dma;
	mx2_dma_priv_t *dma_private = NULL;
	mx2_dma_info_t *dma_info = mxc_dma_get_info(channel_id);
	int index;
	int ret;

	if (dma_info == NULL) {
		return -EINVAL;
	}

	if ((index = get_dma_channel(dma_info->dma_chan)) < 0) {
		return -ENODEV;
	}

	dma = g_dma_channels + index;
	dma_private = (mx2_dma_priv_t *) dma->private;
	if (dma_private == NULL) {
		printk(KERN_ERR
		       "request dma channel %d which is not initialize completed.!\n",
		       index);
		ret = -EFAULT;
		goto exit;
	}

	dma->active = 0;
	dma_private->dma_info = NULL;
	dma->cb_fn = NULL;
	dma->cb_args = NULL;
	dma->dev_name = dev_name;
	dma->mode = dma_info->mode ? MXC_DMA_MODE_WRITE : MXC_DMA_MODE_READ;
	init_dma_bd(dma_private);

	if (!(ret = __init_dma_channel(dma, dma_info))) {
		dma_private->dma_info = dma_info;
		return index;
	}
      exit:
	put_dma_channel(index);
	return ret;
}

/*!
 * This function is generally called by the driver at close time. The DMA
 * driver would do any cleanup associated with this channel.
 *
 * @param channel_num  the channel number returned at request time. This
 *                     would be used by the DMA driver to identify the calling
 *                     driver and do the necessary cleanup on the channel
 *                     associated with the particular peripheral
 * @return returns a negative number on error or 0 on success
 */
int mxc_dma_free(int channel_num)
{
	mxc_dma_channel_t *dma;
	mx2_dma_priv_t *dma_private;

	if ((channel_num >= MAX_DMA_CHANNELS) || (channel_num < 0)) {
		return -EINVAL;
	}

	dma = g_dma_channels + channel_num;
	dma_private = (mx2_dma_priv_t *) dma->private;
	if (dma_private == NULL) {
		printk(KERN_ERR
		       "Free dma %d which is not completed initialization \n",
		       channel_num);
		return -EFAULT;
	}
	if (dma->lock) {
		if (dma->active) {	/*Channel is busy */
			mxc_dma_disable(channel_num);
		}

		dma_private = (mx2_dma_priv_t *) dma->private;

		enable_dma_clk();
		mask_dma_interrupt(channel_num);
		disable_dma_clk();

		free_irq(dma_private->dma_irq, (void *)dma);
		put_dma_channel(channel_num);
	}
	return 0;
}

/*!
 * This function would just configure the buffers specified by the user into
 * dma channel. The caller must call mxc_dma_enable to start this transfer.
 *
 * @param channel_num  the channel number returned at request time. This
 *                     would be used by the DMA driver to identify the calling
 *                     driver and do the necessary cleanup on the channel
 *                     associated with the particular peripheral
 * @param dma_buf      an array of physical addresses to the user defined
 *                     buffers. The caller must guarantee the dma_buf is
 *                     available until the transfer is completed.
 * @param num_buf      number of buffers in the array
 * @param mode         specifies whether this is READ or WRITE operation
 * @return This function returns a negative number on error if buffer could not be
 *         added with DMA for transfer. On Success, it returns 0
 */
int mxc_dma_config(int channel_num, mxc_dma_requestbuf_t * dma_buf, int num_buf,
		   mxc_dma_mode_t mode)
{
	mxc_dma_channel_t *dma;
	mx2_dma_priv_t *dma_private;

	if ((dma_buf == NULL) || (num_buf < 1)) {
		return -EINVAL;
	}

	if ((channel_num >= MAX_DMA_CHANNELS) || (channel_num < 0)) {
		return -EINVAL;
	}

	dma = g_dma_channels + channel_num;
	dma_private = (mx2_dma_priv_t *) dma->private;
	if (dma_private == NULL) {
		printk(KERN_ERR
		       "config dma %d which is not completed initialization \n",
		       channel_num);
		return -EFAULT;
	}

	if (dma->lock == 0) {
		return -ENODEV;
	}

	/*TODO: dma chainning can not support on bi-dir channel */
	if (dma_private->dma_chaining && (dma->mode != mode)) {
		return -EINVAL;
	}

	/*TODO: fill dma buffer into driver .
	 * If driver is no enought buffer to save them , it will return -EBUSY
	 */
	if (fill_dma_bd(dma, dma_buf, num_buf, mode)) {
		return -EBUSY;
	}

	return 0;
}

/*!
 * This function would just configure the scatterlist specified by the
 * user into dma channel. This is a slight variation of mxc_dma_config(),
 * it is provided for the convenience of drivers that have a scatterlist
 * passed into them. It is the calling driver's responsibility to have the
 * correct physical address filled in the "dma_address" field of the
 * scatterlist.
 *
 * @param channel_num  the channel number returned at request time. This
 *                     would be used by the DMA driver to identify the calling
 *                     driver and do the necessary cleanup on the channel
 *                     associated with the particular peripheral
 * @param sg           a scatterlist of buffers. The caller must guarantee
 *                     the dma_buf is available until the transfer is
 *                     completed.
 * @param num_buf      number of buffers in the array
 * @param num_of_bytes total number of bytes to transfer. If set to 0, this
 *                     would imply to use the length field of the scatterlist
 *                     for each DMA transfer. Else it would calculate the size
 *                     for each DMA transfer.
 * @param mode         specifies whether this is READ or WRITE operation
 * @return This function returns a negative number on error if buffer could not
 *         be added with DMA for transfer. On Success, it returns 0
 */
int mxc_dma_sg_config(int channel_num, struct scatterlist *sg,
		      int num_buf, int num_of_bytes, mxc_dma_mode_t mode)
{
	mxc_dma_channel_t *dma;
	mx2_dma_priv_t *dma_private;

	if ((sg == NULL) || (num_buf < 1) || (num_of_bytes < 0)) {
		return -EINVAL;
	}

	if ((channel_num >= MAX_DMA_CHANNELS) || (channel_num < 0)) {
		return -EINVAL;
	}

	dma = g_dma_channels + channel_num;
	dma_private = (mx2_dma_priv_t *) dma->private;
	if (dma_private == NULL) {
		printk(KERN_ERR
		       "config_sg dma %d which is not completed initialization \n",
		       channel_num);
		return -EFAULT;
	}

	if (dma->lock == 0) {
		return -ENODEV;
	}

	/*TODO: dma chainning can not support on bi-dir channel */
	if (dma_private->dma_chaining && (dma->mode != mode)) {
		return -EINVAL;
	}

	/*TODO: fill dma buffer into driver .
	 * If driver is no enought buffer to save them , it will return -EBUSY
	 */
	if (fill_dma_bd_by_sg(dma, sg, num_buf, num_of_bytes, mode)) {
		return -EBUSY;
	}
	return 0;
}

/*!
 * This function is provided if the driver would like to set/change its
 * callback function.
 *
 * @param channel_num  the channel number returned at request time. This
 *                     would be used by the DMA driver to identify the calling
 *                     driver and do the necessary cleanup on the channel
 *                     associated with the particular peripheral
 * @param callback     a callback function to provide notification on transfer
 *                     completion, user could specify NULL if he does not wish
 *                     to be notified
 * @param arg          an argument that gets passed in to the callback
 *                     function, used by the user to do any driver specific
 *                     operations.
 * @return this function returns an error if the callback could not be set
 *         for the channel
 */
int mxc_dma_callback_set(int channel_num, mxc_dma_callback_t callback,
			 void *arg)
{
	mxc_dma_channel_t *dma;

	if ((channel_num >= MAX_DMA_CHANNELS) || (channel_num < 0)) {
		return -EINVAL;
	}
	dma = g_dma_channels + channel_num;

	if (!dma->lock) {
		return -ENODEV;
	}

	if (dma->active) {
		return -EBUSY;
	}
	dma->cb_fn = callback;
	dma->cb_args = arg;
	return 0;

}

/*!
 * This stops the DMA channel and any ongoing transfers. Subsequent use of
 * mxc_dma_enable() will restart the channel and restart the transfer.
 *
 * @param channel_num  the channel number returned at request time. This
 *                     would be used by the DMA driver to identify the calling
 *                     driver and do the necessary cleanup on the channel
 *                     associated with the particular peripheral
 * @return returns a negative number on error or 0 on success
 */
int mxc_dma_disable(int channel_num)
{
	mxc_dma_channel_t *dma;
	mx2_dma_priv_t *priv;
	unsigned long ctrl_val;

	if ((channel_num >= MAX_DMA_CHANNELS) || (channel_num < 0)) {
		return -EINVAL;
	}

	dma = g_dma_channels + channel_num;

	if (dma->lock == 0) {
		return -EINVAL;
	}

	if (!dma->active) {
		return -EINVAL;
	}

	priv = (mx2_dma_priv_t *) dma->private;
	if (priv == NULL) {
		printk(KERN_ERR "disable a uncompleted dma channel %d\n",
		       channel_num);
		return -EFAULT;
	}

	dma->active = 0;
	enable_dma_clk();

	__clear_dma_interrupt(channel_num);
	ctrl_val =
	    __raw_readl(IO_ADDRESS(DMA_BASE_ADDR) + DMA_CCR(channel_num));
	ctrl_val &= ~DMA_CTL_CEN;	/* clear CEN bit */
	__raw_writel(ctrl_val,
		     IO_ADDRESS(DMA_BASE_ADDR) + DMA_CCR(channel_num));
	disable_dma_clk();
	atomic_dec(&g_dma_actived);

	/*TODO: Clear all request buffers */
	flush_dma_bd(priv);
	return 0;
}

/*!
 * This starts DMA transfer. Or it restarts DMA on a stopped channel
 * previously stopped with mxc_dma_disable().
 *
 * @param channel_num  the channel number returned at request time. This
 *                     would be used by the DMA driver to identify the calling
 *                     driver and do the necessary cleanup on the channel
 *                     associated with the particular peripheral
 * @return returns a negative number on error or 0 on success
 */
int mxc_dma_enable(int channel_num)
{
	mxc_dma_channel_t *dma;
	mx2_dma_priv_t *priv;

	if ((channel_num >= MAX_DMA_CHANNELS) || (channel_num < 0)) {
		return -EINVAL;
	}

	dma = g_dma_channels + channel_num;

	if (dma->lock == 0) {
		return -EINVAL;
	}

	priv = (mx2_dma_priv_t *) dma->private;
	if (priv == NULL) {
		printk(KERN_ERR "enable a uncompleted dma channel %d\n",
		       channel_num);
		return -EFAULT;
	}

	if (dma->active) {
		return 0;
	}
	dma->active = 1;
	priv->trans_bytes = 0;

	enable_dma_clk();

	atomic_inc(&g_dma_actived);
	__clear_dma_interrupt(channel_num);

	setup_dmac(dma);
	disable_dma_clk();
	return 0;
}

/*!
*@brief Dump DMA registers
*
*@param channel  Requested channel NO.
*@return     none
*/

void mxc_dump_dma_register(int channel)
{
	mxc_dma_channel_t *dma = &g_dma_channels[channel];
	mx2_dma_priv_t *priv = (mx2_dma_priv_t *) dma->private;
	dma_regs_t *dma_base;

	printk(KERN_INFO "======== Dump dma channel %d \n", channel);
	if ((unsigned)channel >= MXC_DMA_CHANNELS) {
		printk(KERN_INFO "Channel number is invalid \n");
		return;
	}
	if (!dma->lock) {
		printk(KERN_INFO "Channel is not allocated \n");
		return;
	}

	printk(KERN_INFO "g_dma_actived = %d\n", atomic_read(&g_dma_actived));

	enable_dma_clk();
	dma_base = (dma_regs_t *) (priv->dma_base);
	printk(KERN_INFO "DMA COMMON REGISTER\n");
	printk(KERN_INFO "DMA CONTROL             DMA_DCR: %08x\n",
	       __raw_readl(IO_ADDRESS(DMA_BASE_ADDR) + DMA_DCR));
	printk(KERN_INFO "DMA Interrupt status    DMA_DISR: %08x\n",
	       __raw_readl(IO_ADDRESS(DMA_BASE_ADDR) + DMA_DISR));
	printk(KERN_INFO "DMA Interrupt Mask      DMA_DIMR: %08x\n",
	       __raw_readl(IO_ADDRESS(DMA_BASE_ADDR) + DMA_DIMR));
	printk(KERN_INFO "DMA Burst Time Out      DMA_DBTOSR: %08x\n",
	       __raw_readl(IO_ADDRESS(DMA_BASE_ADDR) + DMA_DBTOSR));
	printk(KERN_INFO "DMA request Time Out    DMA_DRTOSR: %08x\n",
	       __raw_readl(IO_ADDRESS(DMA_BASE_ADDR) + DMA_DRTOSR));
	printk(KERN_INFO "DMA Transfer Error      DMA_DSESR: %08x\n",
	       __raw_readl(IO_ADDRESS(DMA_BASE_ADDR) + DMA_DSESR));
	printk(KERN_INFO "DMA DMA_Overflow        DMA_DBOSR: %08x\n",
	       __raw_readl(IO_ADDRESS(DMA_BASE_ADDR) + DMA_DBOSR));
	printk(KERN_INFO "DMA Burst Time OutCtl   DMA_BurstTOCtl: %08x\n",
	       __raw_readl(IO_ADDRESS(DMA_BASE_ADDR) + DMA_DBTOCR));

	printk(KERN_INFO "DMA 2D X size: %08x\n",
	       __raw_readl(IO_ADDRESS(DMA_BASE_ADDR) + DMA_XSRA));
	printk(KERN_INFO "DMA 2D Y size: %08x\n",
	       __raw_readl(IO_ADDRESS(DMA_BASE_ADDR) + DMA_YSRA));
	printk(KERN_INFO "DMA 2D Z size: %08x\n",
	       __raw_readl(IO_ADDRESS(DMA_BASE_ADDR) + DMA_WSRA));

	printk(KERN_INFO "DMA Chan %2d  Sourc     SourceAddr: %08x\n", channel,
	       __raw_readl(&(dma_base->SourceAddr)));
	printk(KERN_INFO "DMA Chan %2d  dest      DestAddr: %08x\n", channel,
	       __raw_readl(&(dma_base->DestAddr)));
	printk(KERN_INFO "DMA Chan %2d  count     Count: %08x\n", channel,
	       __raw_readl(&(dma_base->Count)));
	printk(KERN_INFO "DMA Chan %2d  Ctl       Ctl: %08x\n", channel,
	       __raw_readl(&(dma_base->Ctl)));
	printk(KERN_INFO "DMA Chan %2d  request   RequestSource: %08x\n",
	       channel, __raw_readl(&(dma_base->RequestSource)));
	printk(KERN_INFO "DMA Chan %2d  burstL    BurstLength: %08x\n", channel,
	       __raw_readl(&(dma_base->BurstLength)));
	printk(KERN_INFO "DMA Chan %2d  requestTO ReqTimeout: %08x\n", channel,
	       __raw_readl(&(dma_base->ReqTimeout)));
	printk(KERN_INFO "DMA Chan %2d  BusUtilt  BusUtilt: %08x\n", channel,
	       __raw_readl(&(dma_base->BusUtilt)));

	disable_dma_clk();
}

#ifdef DMA_PM

static int channel_in_use(void)
{
	int i;
	for (i = 0; i < MXC_DMA_CHANNELS; i++) {
		if (dma_chan[i].lock)
			return 1;
	}
	return 0;
}

int mxc_dma_pm_standby(void)
{
	unsigned long reg;
	if (dma_pm_status == DMA_PMST_STANDBY)
		return 0;

	if (!channel_in_use()) {
		/*Disable DMA */
		__disable_dma_clk();
		dma_pm_status = DMA_PMST_STANDBY;
		return 0;
	}
	return -1;
}

int mxc_dma_pm_resume(void)
{
	unsigned long reg;
	if (dma_pm_status == DMA_PMST_RESUME)
		return 0;

	/*Enable HCLK_DMA and DMA(ipg clock) */
	dma_pm_status = DMA_PMST_RESUME;
	return 0;
}

int mxc_dma_pm_suspend(void)
{
	unsigned long reg;
	if (dma_pm_status == DMA_PMST_SUSPEND)
		return 0;

	if (!channel_in_use()) {
		/*Disable DMA */
		__disable_dma_clk();
		dma_pm_status = DMA_PMST_SUSPEND;
		return 0;
	}
	return -1;
}

int mxc_dma_pm_handler(struct pm_dev *dev, pm_request_t rqst, void *data)
{
	int ret = 0;
	switch (rqst) {
		/*APM doesn't send PM_STANDBY and PM_STANDBY_RESUME request now. */
	case PM_SUSPEND:
		ret = dma_pm_suspend();
		break;
	case PM_RESUME:
		ret = dma_pm_resume();
		break;
	}
	return ret;
}

#endif				/*DMA_PM */

int __init mxc_dma_init(void)
{
	int i;
	mxc_dma_channel_t *dma = g_dma_channels;
	mx2_dma_priv_t *private = g_dma_privates;

	memset(dma, 0, sizeof(mxc_dma_channel_t) * MXC_DMA_CHANNELS);
	for (i = 0; i < MXC_DMA_CHANNELS; i++, dma++, private++) {
		dma->channel = i;
		dma->private = private;
		private->dma_base =
		    (unsigned int)(IO_ADDRESS(DMA_BASE_ADDR + DMA_CH_BASE(i)));
		private->dma_irq = i + MXC_DMA_INTR_0;	/*Dma channel interrupt number */
		private->bd_ring = &g_dma_bd_table[i][0];
	}

	mxc_dma_load_info(g_dma_channels);

	dma_clk = clk_get(NULL, "dma_clk");
	clk_enable(dma_clk);

	__raw_writel(0x2, IO_ADDRESS(DMA_BASE_ADDR) + DMA_DCR);	/*reset DMA; */

	disable_dma_clk();

	/*use module init because create_proc after init_dma */
	g_proc_dir = create_proc_entry("dma", 0, NULL);
	g_proc_dir->read_proc = (read_proc_t *) mxc_get_dma_list;
	g_proc_dir->data = NULL;

#ifdef DMA_PM
	/* Register the device with power management. */
	dma_pm = pm_register(PM_DMA_DEV, PM_SYS_UNKNOWN, dma_pm_handler);
#endif

	return 0;
}

arch_initcall(mxc_dma_init);

EXPORT_SYMBOL(mxc_dma_request_ext);
EXPORT_SYMBOL(mxc_dma_free);
EXPORT_SYMBOL(mxc_dma_callback_set);
EXPORT_SYMBOL(mxc_dma_enable);
EXPORT_SYMBOL(mxc_dma_disable);
EXPORT_SYMBOL(mxc_dma_config);
EXPORT_SYMBOL(mxc_dma_sg_config);
EXPORT_SYMBOL(mxc_dump_dma_register);
