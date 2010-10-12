/*
 * Copyright 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_MXC_DMA_H__
#define __ASM_ARCH_MXC_DMA_H__

#include <linux/scatterlist.h>

#define MXC_DMA_DYNAMIC_CHANNEL   255

#define MXC_DMA_DONE		  0x0
#define MXC_DMA_REQUEST_TIMEOUT   0x1
#define MXC_DMA_TRANSFER_ERROR    0x2

/*! This defines the list of device ID's for DMA */
typedef enum mxc_dma_device {
	MXC_DMA_UART1_RX,
	MXC_DMA_UART1_TX,
	MXC_DMA_UART2_RX,
	MXC_DMA_UART2_TX,
	MXC_DMA_UART3_RX,
	MXC_DMA_UART3_TX,
	MXC_DMA_UART4_RX,
	MXC_DMA_UART4_TX,
	MXC_DMA_UART5_RX,
	MXC_DMA_UART5_TX,
	MXC_DMA_UART6_RX,
	MXC_DMA_UART6_TX,
	MXC_DMA_MMC1_WIDTH_1,
	MXC_DMA_MMC1_WIDTH_4,
	MXC_DMA_MMC2_WIDTH_1,
	MXC_DMA_MMC2_WIDTH_4,
	MXC_DMA_SSI1_8BIT_RX0,
	MXC_DMA_SSI1_8BIT_TX0,
	MXC_DMA_SSI1_16BIT_RX0,
	MXC_DMA_SSI1_16BIT_TX0,
	MXC_DMA_SSI1_24BIT_RX0,
	MXC_DMA_SSI1_24BIT_TX0,
	MXC_DMA_SSI1_8BIT_RX1,
	MXC_DMA_SSI1_8BIT_TX1,
	MXC_DMA_SSI1_16BIT_RX1,
	MXC_DMA_SSI1_16BIT_TX1,
	MXC_DMA_SSI1_24BIT_RX1,
	MXC_DMA_SSI1_24BIT_TX1,
	MXC_DMA_SSI2_8BIT_RX0,
	MXC_DMA_SSI2_8BIT_TX0,
	MXC_DMA_SSI2_16BIT_RX0,
	MXC_DMA_SSI2_16BIT_TX0,
	MXC_DMA_SSI2_24BIT_RX0,
	MXC_DMA_SSI2_24BIT_TX0,
	MXC_DMA_SSI2_8BIT_RX1,
	MXC_DMA_SSI2_8BIT_TX1,
	MXC_DMA_SSI2_16BIT_RX1,
	MXC_DMA_SSI2_16BIT_TX1,
	MXC_DMA_SSI2_24BIT_RX1,
	MXC_DMA_SSI2_24BIT_TX1,
	MXC_DMA_SSI3_8BIT_RX0,
	MXC_DMA_SSI3_8BIT_TX0,
	MXC_DMA_SSI3_16BIT_RX0,
	MXC_DMA_SSI3_16BIT_TX0,
	MXC_DMA_SSI3_24BIT_RX0,
	MXC_DMA_SSI3_24BIT_TX0,
	MXC_DMA_SSI3_8BIT_RX1,
	MXC_DMA_SSI3_8BIT_TX1,
	MXC_DMA_SSI3_16BIT_RX1,
	MXC_DMA_SSI3_16BIT_TX1,
	MXC_DMA_SSI3_24BIT_RX1,
	MXC_DMA_SSI3_24BIT_TX1,
	MXC_DMA_FIR_RX,
	MXC_DMA_FIR_TX,
	MXC_DMA_CSPI1_RX,
	MXC_DMA_CSPI1_TX,
	MXC_DMA_CSPI2_RX,
	MXC_DMA_CSPI2_TX,
	MXC_DMA_CSPI3_RX,
	MXC_DMA_CSPI3_TX,
	MXC_DMA_ATA_RX,
	MXC_DMA_ATA_TX,
	MXC_DMA_MEMORY,
	MXC_DMA_FIFO_MEMORY,
	MXC_DMA_DSP_PACKET_DATA0_RD,
	MXC_DMA_DSP_PACKET_DATA0_WR,
	MXC_DMA_DSP_PACKET_DATA1_RD,
	MXC_DMA_DSP_PACKET_DATA1_WR,
	MXC_DMA_DSP_LOG0_CHNL,
	MXC_DMA_DSP_LOG1_CHNL,
	MXC_DMA_DSP_LOG2_CHNL,
	MXC_DMA_DSP_LOG3_CHNL,
	MXC_DMA_CSI_RX,
	MXC_DMA_SPDIF_16BIT_TX,
	MXC_DMA_SPDIF_16BIT_RX,
	MXC_DMA_SPDIF_32BIT_TX,
	MXC_DMA_SPDIF_32BIT_RX,
	MXC_DMA_ASRC_A_RX,
	MXC_DMA_ASRC_A_TX,
	MXC_DMA_ASRC_B_RX,
	MXC_DMA_ASRC_B_TX,
	MXC_DMA_ASRC_C_RX,
	MXC_DMA_ASRC_C_TX,
	MXC_DMA_ASRCA_ESAI,
	MXC_DMA_ASRCB_ESAI,
	MXC_DMA_ASRCC_ESAI,
	MXC_DMA_ASRCA_SSI1_TX0,
	MXC_DMA_ASRCA_SSI1_TX1,
	MXC_DMA_ASRCA_SSI2_TX0,
	MXC_DMA_ASRCA_SSI2_TX1,
	MXC_DMA_ASRCB_SSI1_TX0,
	MXC_DMA_ASRCB_SSI1_TX1,
	MXC_DMA_ASRCB_SSI2_TX0,
	MXC_DMA_ASRCB_SSI2_TX1,
	MXC_DMA_ESAI_16BIT_RX,
	MXC_DMA_ESAI_16BIT_TX,
	MXC_DMA_ESAI_24BIT_RX,
	MXC_DMA_ESAI_24BIT_TX,
	MXC_DMA_TEST_RAM2D2RAM,
	MXC_DMA_TEST_RAM2RAM2D,
	MXC_DMA_TEST_RAM2D2RAM2D,
	MXC_DMA_TEST_RAM2RAM,
	MXC_DMA_TEST_HW_CHAINING,
	MXC_DMA_TEST_SW_CHAINING
} mxc_dma_device_t;

/*! This defines the prototype of callback funtion registered by the drivers */
typedef void (*mxc_dma_callback_t) (void *arg, int error_status,
				    unsigned int count);

/*! This defines the type of DMA transfer requested */
typedef enum mxc_dma_mode {
	MXC_DMA_MODE_READ,
	MXC_DMA_MODE_WRITE,
} mxc_dma_mode_t;

/*! This defines the DMA channel parameters */
typedef struct mxc_dma_channel {
	unsigned int active:1;	/*!< When there has a active tranfer, it is set to 1 */
	unsigned int lock;	/*!< Defines the channel is allocated or not */
	int curr_buf;		/*!< Current buffer */
	mxc_dma_mode_t mode;	/*!< Read or Write */
	unsigned int channel;	/*!< Channel info */
	unsigned int dynamic:1;	/*!< Channel not statically allocated when 1 */
	char *dev_name;		/*!< Device name */
	void *private;		/*!< Private structure for platform */
	mxc_dma_callback_t cb_fn;	/*!< The callback function */
	void *cb_args;		/*!< The argument of callback function */
} mxc_dma_channel_t;

/*! This structure contains the information about a dma transfer */
typedef struct mxc_dma_requestbuf {
	dma_addr_t src_addr;	/*!< source address */
	dma_addr_t dst_addr;	/*!< destination address */
	int num_of_bytes;	/*!< the length of this transfer : bytes */
} mxc_dma_requestbuf_t;

/*! This struct contains the information for asrc special*/
struct dma_channel_asrc_info {
	u32 channs;		/*!< data channels in asrc */
};

/*! This struct contains  the information for device special*/
struct dma_channel_info {
	struct dma_channel_asrc_info asrc;	/*!< asrc special information */
};

#if defined(CONFIG_ARCH_MX27) || defined(CONFIG_ARCH_MX21)
#include <mach/mx2_dma.h>
#else
#include <mach/sdma.h>
#endif

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
 * @param data         the customized parameter for special channel.
 * @return returns a negative number on error if request for a DMA channel did not
 *         succeed, returns the channel number to be used on success.
 */
extern int mxc_dma_request_ext(mxc_dma_device_t channel_id, char *dev_name,
			       struct dma_channel_info *info);

static inline int mxc_dma_request(mxc_dma_device_t channel_id, char *dev_name)
{
	return mxc_dma_request_ext(channel_id, dev_name, NULL);
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
extern int mxc_dma_free(int channel_num);

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
extern int mxc_dma_config(int channel_num, mxc_dma_requestbuf_t *dma_buf,
			  int num_buf, mxc_dma_mode_t mode);

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
extern int mxc_dma_sg_config(int channel_num, struct scatterlist *sg,
			     int num_buf, int num_of_bytes,
			     mxc_dma_mode_t mode);

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
 * @return this function returns a negative number on error if the callback
 *         could not be set for the channel or 0 on success
 */
extern int mxc_dma_callback_set(int channel_num, mxc_dma_callback_t callback,
				void *arg);

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
extern int mxc_dma_disable(int channel_num);

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
extern int mxc_dma_enable(int channel_num);

#endif
