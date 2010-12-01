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

/**
 * struct mxs_dma_cmd_bits - MXS DMA hardware command bits.
 *
 * This structure describes the in-memory layout of the command bits in a DMA
 * command. See the appropriate reference manual for a detailed description
 * of what these bits mean to the DMA hardware.
 */
struct mxs_dma_cmd_bits {
	unsigned int command:2;
#define NO_DMA_XFER	0x00
#define DMA_WRITE	0x01
#define DMA_READ	0x02
#define DMA_SENSE	0x03

	unsigned int chain:1;
	unsigned int irq:1;
	unsigned int nand_lock:1;
	unsigned int nand_wait_4_ready:1;
	unsigned int dec_sem:1;
	unsigned int wait4end:1;
	unsigned int halt_on_terminate:1;
	unsigned int terminate_flush:1;
	unsigned int resv2:2;
	unsigned int pio_words:4;
	unsigned int bytes:16;
};

/* fill the word 1 of the DMA channel command structure */
#define fill_dma_word1(cmdp, _cmd, _chain, _irq, _lock, _wait_ready, _dec_sem,\
		_wait4end, _halt_on_term, _term_flush, _pio_words, _bytes)\
	do {							\
		struct mxs_dma_cmd_bits *bits = &(cmdp)->bits;	\
								\
		/* reset all the fields first */		\
		(cmdp)->data		= 0;			\
								\
		bits->command		= _cmd;			\
		bits->chain		= _chain;		\
		bits->irq		= _irq;			\
		bits->nand_lock		= _lock;		\
		bits->nand_wait_4_ready	= _wait_ready;		\
		bits->dec_sem		= _dec_sem;		\
		bits->wait4end		= _wait4end;		\
		bits->halt_on_terminate	= _halt_on_term;	\
		bits->terminate_flush	= _term_flush;		\
		bits->pio_words		= _pio_words;		\
		bits->bytes		= _bytes;		\
	} while (0)

/**
 * struct mxs_dma_cmd - MXS DMA hardware command.
 *
 * This structure describes the in-memory layout of an entire DMA command,
 * including space for the maximum number of PIO accesses. See the appropriate
 * reference manual for a detailed description of what these fields mean to the
 * DMA hardware.
 */
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

/**
 * struct mxs_dma_desc - MXS DMA command descriptor.
 *
 * This structure incorporates an MXS DMA hardware command structure, along
 * with metadata.
 *
 * @cmd:      The MXS DMA hardware command block.
 * @flags:    Flags that represent the state of this DMA descriptor.
 * @address:  The physical address of this descriptor.
 * @buffer:   A convenient place for software to put the virtual address of the
 *            associated data buffer (the physical address of the buffer
 *            appears in the DMA command). The MXS platform DMA software doesn't
 *            use this field -- it is provided as a convenience.
 * @node:     Links this structure into a list.
 */
struct mxs_dma_desc {
	struct mxs_dma_cmd cmd;
	unsigned int flags;
#define MXS_DMA_DESC_READY 0x80000000
#define MXS_DMA_DESC_FIRST 0x00000001
#define MXS_DMA_DESC_LAST  0x00000002
	dma_addr_t address;
	void *buffer;
	struct list_head node;
};

struct mxs_dma_info {
	unsigned int status;
#define MXS_DMA_INFO_ERR       0x00000001
#define MXS_DMA_INFO_ERR_STAT  0x00010000
	unsigned int buf_addr;
	unsigned int xfer_count;
};

/**
 * struct mxs_dma_chan - MXS DMA channel
 *
 * This structure represents a single DMA channel. The MXS platform code
 * maintains an array of these structures to represent every DMA channel in the
 * system (see mxs_dma_channels).
 *
 * @name:         A human-readable string that describes how this channel is
 *                being used or what software "owns" it. This field is set when
 *                when the channel is reserved by mxs_dma_request().
 * @dev:          A pointer to a struct device *, cast to an unsigned long, and
 *                representing the software that "owns" the channel. This field
 *                is set when when the channel is reserved by mxs_dma_request().
 * @lock:         Arbitrates access to this channel.
 * @dma:          A pointer to a struct mxs_dma_device representing the driver
 *                code that operates this channel.
 * @flags:        Flag bits that represent the state of this channel.
 * @active_num:   If the channel is not busy, this value is zero. If the channel
 *                is busy, this field contains the number of DMA command
 *                descriptors at the head of the active list that the hardware
 *                has been told to process. This value is set at the moment the
 *                channel is enabled by mxs_dma_enable(). More descriptors may
 *                arrive after the channel is enabled, so the number of
 *                descriptors on the active list may be greater than this value.
 *                In fact, it should always be active_num + pending_num.
 * @pending_num:  The number of DMA command descriptors at the tail of the
 *                active list that the hardware has not been told to process.
 * @active:       The list of DMA command descriptors either currently being
 *                processed by the hardware or waiting to be processed.
 *                Descriptors being processed appear at the head of the list,
 *                while pending descriptors appear at the tail. The total number
 *                should always be active_num + pending_num.
 * @done:         The list of DMA command descriptors that have either been
 *                processed by the DMA hardware or aborted by a call to
 *                mxs_dma_disable().
 */
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

/**
 * struct mxs_dma_device - DMA channel driver interface.
 *
 * This structure represents the driver that operates a DMA channel. Every
 * struct mxs_dma_chan contains a pointer to a structure of this type, which is
 * installed when the driver registers to "own" the channel (see
 * mxs_dma_device_register()).
 */
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
	void (*info)(struct mxs_dma_device *,
		unsigned int, struct mxs_dma_info *info);
	void (*enable_irq) (struct mxs_dma_device *, unsigned int, int);
	int (*irq_is_pending) (struct mxs_dma_device *, unsigned int);
	void (*ack_irq) (struct mxs_dma_device *, unsigned int);

	void (*set_target) (struct mxs_dma_device *, unsigned int, int);
};

/**
 * mxs_dma_device_register - Register a DMA driver.
 *
 * This function registers a driver for a contiguous group of DMA channels (the
 * ordering of DMA channels is specified by the globally unique DMA channel
 * numbers given in mach/dma.h).
 *
 * @pdev:  A pointer to a structure that represents the driver. This structure
 *         contains fields that specify the first DMA channel number and the
 *         number of channels.
 */
extern int mxs_dma_device_register(struct mxs_dma_device *pdev);

/**
 * mxs_dma_request - Request to reserve a DMA channel.
 *
 * @channel:  The channel number. This is one of the globally unique DMA channel
 *            numbers given in mach/dma.h.
 * @dev:      A pointer to a struct device representing the channel "owner."
 * @name:     A human-readable string that identifies the channel owner or the
 *            purpose of the channel.
 */
extern int mxs_dma_request(int channel, struct device *dev, const char *name);

/**
 * mxs_dma_release - Release a DMA channel.
 *
 * This function releases a DMA channel from its current owner.
 *
 * The channel will NOT be released if it's marked "busy" (see
 * mxs_dma_enable()).
 *
 * @channel:  The channel number. This is one of the globally unique DMA channel
 *            numbers given in mach/dma.h.
 * @dev:      A pointer to a struct device representing the channel "owner." If
 *            this doesn't match the owner given to mxs_dma_request(), the
 *            channel will NOT be released.
 */
extern void mxs_dma_release(int channel, struct device *dev);

/**
 * mxs_dma_enable - Enable a DMA channel.
 *
 * If the given channel has any DMA descriptors on its active list, this
 * function causes the DMA hardware to begin processing them.
 *
 * This function marks the DMA channel as "busy," whether or not there are any
 * descriptors to process.
 *
 * @channel:  The channel number. This is one of the globally unique DMA channel
 *            numbers given in mach/dma.h.
 */
extern int mxs_dma_enable(int channel);

/**
 * mxs_dma_disable - Disable a DMA channel.
 *
 * This function shuts down a DMA channel and marks it as "not busy." Any
 * descriptors on the active list are immediately moved to the head of the
 * "done" list, whether or not they have actually been processed by the
 * hardware. The "ready" flags of these descriptors are NOT cleared, so they
 * still appear to be active.
 *
 * This function immediately shuts down a DMA channel's hardware, aborting any
 * I/O that may be in progress, potentially leaving I/O hardware in an undefined
 * state. It is unwise to call this function if there is ANY chance the hardware
 * is still processing a command.
 *
 * @channel:  The channel number. This is one of the globally unique DMA channel
 *            numbers given in mach/dma.h.
 */
extern void mxs_dma_disable(int channel);

/**
 * mxs_dma_reset - Resets the DMA channel hardware.
 *
 * @channel:  The channel number. This is one of the globally unique DMA channel
 *            numbers given in mach/dma.h.
 */
extern void mxs_dma_reset(int channel);

/**
 * mxs_dma_freeze - Freeze a DMA channel.
 *
 * This function causes the channel to continuously fail arbitration for bus
 * access, which halts all forward progress without losing any state. A call to
 * mxs_dma_unfreeze() will cause the channel to continue its current operation
 * with no ill effect.
 *
 * @channel:  The channel number. This is one of the globally unique DMA channel
 *            numbers given in mach/dma.h.
 */
extern void mxs_dma_freeze(int channel);

/**
 * mxs_dma_unfreeze - Unfreeze a DMA channel.
 *
 * This function reverses the effect of mxs_dma_freeze(), enabling the DMA
 * channel to continue from where it was frozen.
 *
 * @channel:  The channel number. This is one of the globally unique DMA channel
 *            numbers given in mach/dma.h.
 */

extern void mxs_dma_unfreeze(int channel);

/* get dma channel information */
extern int mxs_dma_get_info(int channel, struct mxs_dma_info *info);

/**
 * mxs_dma_cooked - Clean up processed DMA descriptors.
 *
 * This function removes processed DMA descriptors from the "active" list. Pass
 * in a non-NULL list head to get the descriptors moved to your list. Pass NULL
 * to get the descriptors moved to the channel's "done" list. Descriptors on
 * the "done" list can be retrieved with mxs_dma_get_cooked().
 *
 * This function marks the DMA channel as "not busy" if no unprocessed
 * descriptors remain on the "active" list.
 *
 * @channel:  The channel number. This is one of the globally unique DMA channel
 *            numbers given in mach/dma.h.
 * @head:     If this is not NULL, it is the list to which the processed
 *            descriptors should be moved. If this list is NULL, the descriptors
 *            will be moved to the "done" list.
 */
extern int mxs_dma_cooked(int channel, struct list_head *head);

/**
 * mxs_dma_read_semaphore - Read a DMA channel's hardware semaphore.
 *
 * As used by the MXS platform's DMA software, the DMA channel's hardware
 * semaphore reflects the number of DMA commands the hardware will process, but
 * has not yet finished. This is a volatile value read directly from hardware,
 * so it must be be viewed as immediately stale.
 *
 * If the channel is not marked busy, or has finished processing all its
 * commands, this value should be zero.
 *
 * See mxs_dma_append() for details on how DMA command blocks must be configured
 * to maintain the expected behavior of the semaphore's value.
 *
 * @channel:  The channel number. This is one of the globally unique DMA channel
 *            numbers given in mach/dma.h.
 */
extern int mxs_dma_read_semaphore(int channel);

/**
 * mxs_dma_irq_is_pending - Check if a DMA interrupt is pending.
 *
 * @channel:  The channel number. This is one of the globally unique DMA channel
 *            numbers given in mach/dma.h.
 */
extern int mxs_dma_irq_is_pending(int channel);

/**
 * mxs_dma_enable_irq - Enable or disable DMA interrupt.
 *
 * This function enables the given DMA channel to interrupt the CPU.
 *
 * @channel:  The channel number. This is one of the globally unique DMA channel
 *            numbers given in mach/dma.h.
 * @en:       True if the interrupt for this channel should be enabled. False
 *            otherwise.
 */
extern void mxs_dma_enable_irq(int channel, int en);

/**
 * mxs_dma_ack_irq - Clear DMA interrupt.
 *
 * The software that is using the DMA channel must register to receive its
 * interrupts and, when they arrive, must call this function to clear them.
 *
 * @channel:  The channel number. This is one of the globally unique DMA channel
 *            numbers given in mach/dma.h.
 */
extern void mxs_dma_ack_irq(int channel);

/**
 * mxs_dma_set_target - Set the target for a DMA channel.
 *
 * This function is NOT used on the i.MX28.
 *
 * @channel:  The channel number. This is one of the globally unique DMA channel
 *            numbers given in mach/dma.h.
 * @target:   Indicates the target for the channel.
 */
extern void mxs_dma_set_target(int channel, int target);

/* mxs dma utility functions */
extern struct mxs_dma_desc *mxs_dma_alloc_desc(void);
extern void mxs_dma_free_desc(struct mxs_dma_desc *);

/**
 * mxs_dma_cmd_address - Return the address of the command within a descriptor.
 *
 * @desc:  The DMA descriptor of interest.
 */
static inline unsigned int mxs_dma_cmd_address(struct mxs_dma_desc *desc)
{
	return desc->address += offsetof(struct mxs_dma_desc, cmd);
}

/**
 * mxs_dma_desc_pending - Check if descriptor is on a channel's active list.
 *
 * This function returns the state of a descriptor's "ready" flag. This flag is
 * usually set only if the descriptor appears on a channel's active list. The
 * descriptor may or may not have already been processed by the hardware.
 *
 * The "ready" flag is set when the descriptor is submitted to a channel by a
 * call to mxs_dma_append() or mxs_dma_append_list(). The "ready" flag is
 * cleared when a processed descriptor is moved off the active list by a call
 * to mxs_dma_cooked(). The "ready" flag is NOT cleared if the descriptor is
 * aborted by a call to mxs_dma_disable().
 *
 * @desc:  The DMA descriptor of interest.
 */
static inline int mxs_dma_desc_pending(struct mxs_dma_desc *pdesc)
{
	return pdesc->flags & MXS_DMA_DESC_READY;
}

/**
 * mxs_dma_desc_append - Add a DMA descriptor to a channel.
 *
 * If the descriptor list for this channel is not empty, this function sets the
 * CHAIN bit and the NEXTCMD_ADDR fields in the last descriptor's DMA command so
 * it will chain to the new descriptor's command.
 *
 * Then, this function marks the new descriptor as "ready," adds it to the end
 * of the active descriptor list, and increments the count of pending
 * descriptors.
 *
 * The MXS platform DMA software imposes some rules on DMA commands to maintain
 * important invariants. These rules are NOT checked, but they must be carefully
 * applied by software that uses MXS DMA channels.
 *
 * Invariant:
 *     The DMA channel's hardware semaphore must reflect the number of DMA
 *     commands the hardware will process, but has not yet finished.
 *
 * Explanation:
 *     A DMA channel begins processing commands when its hardware semaphore is
 *     written with a value greater than zero, and it stops processing commands
 *     when the semaphore returns to zero.
 *
 *     When a channel finishes a DMA command, it will decrement its semaphore if
 *     the DECREMENT_SEMAPHORE bit is set in that command's flags bits.
 *
 *     In principle, it's not necessary for the DECREMENT_SEMAPHORE to be set,
 *     unless it suits the purposes of the software. For example, one could
 *     construct a series of five DMA commands, with the DECREMENT_SEMAPHORE
 *     bit set only in the last one. Then, setting the DMA channel's hardware
 *     semaphore to one would cause the entire series of five commands to be
 *     processed. However, this example would violate the invariant given above.
 *
 * Rule:
 *    ALL DMA commands MUST have the DECREMENT_SEMAPHORE bit set so that the DMA
 *    channel's hardware semaphore will be decremented EVERY time a command is
 *    processed.
 *
 * @channel:  The channel number. This is one of the globally unique DMA channel
 *            numbers given in mach/dma.h.
 * @pdesc:    A pointer to the new descriptor.
 */
extern int mxs_dma_desc_append(int channel, struct mxs_dma_desc *pdesc);

/**
 * mxs_dma_desc_add_list - Add a list of DMA descriptors to a channel.
 *
 * This function marks all the new descriptors as "ready," adds them to the end
 * of the active descriptor list, and adds the length of the list to the count
 * of pending descriptors.
 *
 * See mxs_dma_desc_append() for important rules that apply to incoming DMA
 * descriptors.
 *
 * @channel:  The channel number. This is one of the globally unique DMA channel
 *            numbers given in mach/dma.h.
 * @head:     A pointer to the head of the list of DMA descriptors to add.
 */
extern int mxs_dma_desc_add_list(int channel, struct list_head *head);

/**
 * mxs_dma_desc_get_cooked - Retrieve processed DMA descriptors.
 *
 * This function moves all the descriptors from the DMA channel's "done" list to
 * the head of the given list.
 *
 * @channel:  The channel number. This is one of the globally unique DMA channel
 *            numbers given in mach/dma.h.
 * @head:     A pointer to the head of the list that will receive the
 *            descriptors on the "done" list.
 */
extern int mxs_dma_get_cooked(int channel, struct list_head *head);

#endif
