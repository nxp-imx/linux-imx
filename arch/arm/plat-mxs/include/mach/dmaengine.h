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

#ifndef __ASM_ARM_ARCH_DMA_H
#define __ASM_ARM_ARCH_DMA_H

#ifndef ARCH_DMA_PIO_WORDS
#define DMA_PIO_WORDS	15
#else
#define DMA_PIO_WORDS	ARCH_DMA_PIO_WORDS
#endif

#define MXS_DMA_ALIGNMENT	8

struct mxs_dma_cmd_bits {
	unsigned int command:2;
#define NO_DMA_XFER	0x00
#define DMA_WRITE	0x01
#define DMA_READ	0x02
#define DMA_SENSE	0x03

	unsigned int chain:1;
	unsigned int irq:1;
	unsigned int resv:2;
	unsigned int dec_sem:1;
	unsigned int wait4end:1;
	unsigned int halt_on_terminate:1;
	unsigned int terminate_flush:1;
	unsigned int resv2:2;
	unsigned int pio_words:4;
	unsigned int bytes:16;
};

struct mxs_dma_cmd {
	unsigned long next;
	union {
		unsigned long data;
		struct mxs_dma_cmd_bits bits;
	} cmd;
	union {
		dma_addr_t address;
		unsigned long alternate;
	};
	unsigned long pio_words[DMA_PIO_WORDS];
};

struct mxs_dma_desc {
	struct mxs_dma_cmd cmd;
	unsigned int flags;
#define MXS_DMA_DESC_READY 0x80000000
	/* address is desc physcial address */
	dma_addr_t address;
	/* buffer address */
	void *buffer;
	struct list_head node;
};

struct mxs_dma_chan {
	const char *name;
	unsigned long dev;
	spinlock_t lock;
	struct mxs_dma_device *dma;
	unsigned int flags;
#define MXS_DMA_FLAGS_IDLE	0x00000000
#define MXS_DMA_FLAGS_BUSY	0x00000001
#define MXS_DMA_FLAGS_FREE	0x00000000
#define MXS_DMA_FLAGS_ALLOCATED	0x00010000
#define MXS_DMA_FLAGS_VALID	0x80000000
	unsigned int active_num;
	unsigned int pending_num;
	struct list_head active;
	struct list_head done;
};

/* dma controller devices */
struct mxs_dma_device {
	struct list_head node;
	const char *name;
	void __iomem *base;
	unsigned int chan_base;
	unsigned int chan_num;
	unsigned int data;
	struct platform_device *pdev;

	/* operations */
	int (*enable) (struct mxs_dma_chan *, unsigned int);
	void (*disable) (struct mxs_dma_chan *, unsigned int);
	void (*reset) (struct mxs_dma_device *, unsigned int);
	void (*freeze) (struct mxs_dma_device *, unsigned int);
	void (*unfreeze) (struct mxs_dma_device *, unsigned int);
	int (*read_semaphore) (struct mxs_dma_device *, unsigned int);
	void (*add_semaphore) (struct mxs_dma_device *, unsigned int, unsigned);
	void (*enable_irq) (struct mxs_dma_device *, unsigned int, int);
	int (*irq_is_pending) (struct mxs_dma_device *, unsigned int);
	void (*ack_irq) (struct mxs_dma_device *, unsigned int);

	void (*set_target) (struct mxs_dma_device *, unsigned int, int);
};

extern int mxs_dma_device_register(struct mxs_dma_device *pdev);

/* request a dma channel */
extern int mxs_dma_request(int channel, struct device *dev, const char *name);
/* Release a dma channel */
extern void mxs_dma_release(int channel, struct device *dev);

/* Enable dma transfer */
extern int mxs_dma_enable(int channel);

/*
 * Disable dma transfer and the desc will be putted into done list
 * Before calling mxs_dma_release, mxs_dma_get_cooked must called to
 * take dma desc back.
 */
extern void mxs_dma_disable(int channel);
/* reset dma channel */
extern void mxs_dma_reset(int channel);
/* freeze dma channel */
extern void mxs_dma_freeze(int channel);
/* unfreeze dma channel */
extern void mxs_dma_unfreeze(int channel);
/*
 * Called to task back the done desc. if head is NULL, they will be
 * put into done list. And must called mxs_dma_get_cooked to take
 * them back
 */
extern int mxs_dma_cooked(int channel, struct list_head *head);

/* Read the dma semaphore to check if there are pending dma desc */
extern int mxs_dma_read_semaphore(int channel);
/* check dma irq is pending */
extern int mxs_dma_irq_is_pending(int channel);
/* enable or disable dma irq */
extern void mxs_dma_enable_irq(int channel, int en);
/* clear dma irq */
extern void mxs_dma_ack_irq(int channel);

/* Used to configure channel related device selection:NOUSED in i.MX28 */
extern void mxs_dma_set_target(int channel, int target);

/* mxs dma utility functions */
extern struct mxs_dma_desc *mxs_dma_alloc_desc(void);
extern void mxs_dma_free_desc(struct mxs_dma_desc *);

static inline unsigned int mxs_dma_cmd_address(struct mxs_dma_desc *desc)
{
	return desc->address += offsetof(struct mxs_dma_desc, cmd);
}

static inline int mxs_dma_desc_pending(struct mxs_dma_desc *pdesc)
{
	return pdesc->flags & MXS_DMA_DESC_READY;
}

/* Add a dma desc to channel*/
extern int mxs_dma_desc_append(int channel, struct mxs_dma_desc *pdesc);
/* Add a list of dma desc to channel*/
extern int mxs_dma_desc_add_list(int channel, struct list_head *head);
/* Take the working done desc back */
extern int mxs_dma_get_cooked(int channel, struct list_head *head);
#endif
