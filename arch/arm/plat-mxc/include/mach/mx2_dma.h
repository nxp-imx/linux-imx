/*
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __ASM_ARCH_MXC_MX2_H__
#define __ASM_ARCH_MXC_MX2_H__

/*!
 * @defgroup DMA_MX27 DMA driver for i.MX27
 */

/*!
 *@file arch-mxc/mx2_dma.h
 *@brief DMA driver header file
 *
 * @ingroup DMA_MX27
 *
 */

#include <mach/hardware.h>
#include <asm/dma.h>
#include <stdarg.h>

#define MXC_DMA_INTR_0		32

#define DMA_DCR		0x000	/*  32bit dma control reg */
#define DMA_DISR	0x004	/*  32bit dma interrupt status reg */
#define DMA_DIMR	0x008	/*  32bit dma interrupt mask reg */
#define DMA_DBTOSR    0x00c	/*  32bit dma burst timeout stat reg */
#define DMA_DRTOSR  0x010	/*  32bit dma req timeout status reg */
#define DMA_DSESR    0x014	/*  32bit dma transfer err status reg */
#define DMA_DBOSR    0x018	/*  32bit dma buffer overflow stat reg */
#define DMA_DBTOCR    0x01c	/*  32bit dma burst timeout ctrl reg */

#define DMA_WSRA		0x040	/*  32bit dma W-size A reg */
#define DMA_XSRA		0x044	/*  32bit dma X-size A reg */
#define DMA_YSRA		0x048	/*  32bit dma Y-size A reg */
#define DMA_WSRB		0x04C	/*  32bit dma W-size B reg */
#define DMA_XSRB		0x050	/*  32bit dma X-size B reg */
#define DMA_YSRB		0x054	/*  32bit dma Y-size B reg */

#define DMA_CH_BASE(x)	(0x080+0x040*(x))

#define DMA_SAR(x)		(DMA_CH_BASE(x)+0x000)
#define DMA_DAR(x)		(DMA_CH_BASE(x)+0x004)
#define DMA_CNTR(x)		(DMA_CH_BASE(x)+0x008)
#define DMA_CCR(x)		(DMA_CH_BASE(x)+0x00C)	/*  32bit dma ch0 control reg */
#define DMA_RSSR(x)		(DMA_CH_BASE(x)+0x010)	/*  32bit dma ch0 req source sel reg */
#define DMA_BLR(x)		(DMA_CH_BASE(x)+0x014)	/*  32bit dma ch0 burst lenght reg */
#define DMA_RTOR(x)		(DMA_CH_BASE(x)+0x018)	/*  32bit dma ch0 req time out reg */
#define DMA_BUCR(x)		(DMA_CH_BASE(x)+0x018)	/*  32bit dma ch0 bus utilization reg */
#define DMA_CCNR(x)		(DMA_CH_BASE(x)+0x01C)	/*  32bit dma ch0 */

#define DMA_TCR			0x480	/*32bit dma test control reg */
#define DMA_TFIFOA		0x484	/*  32bit dma test fifo A reg */
#define DMA_TDRR		0x488	/*  32bit dma test request reg */
#define DMA_TDIPR		0x48c	/*  32bit dma test in progress reg */
#define DMA_TFIFOB		0x490	/*  32bit dma test fifo B reg */

/*!
 * This defines maximum DMA address
 */
#define MAX_DMA_ADDRESS 0xffffffff

#define MXC_DMA_CHANNELS 16
#define MAX_DMA_CHANNELS MXC_DMA_CHANNELS

#define MX_DMA_CHANNELS		MXC_DMA_CHANNELS

/*!@def DMA_MEM_SIZE_8 DMA access port size, 8 bit*/
/*!@def DMA_MEM_SIZE_16 DMA access port size, 16 bit*/
/*!@def DMA_MEM_SIZE_32 DMA access port size, 32 bit*/
#define DMA_MEM_SIZE_8		0x1
#define DMA_MEM_SIZE_16	0x2
#define DMA_MEM_SIZE_32	0x0

/*!@def DMA_TYPE_LINEAR DMA transfer type, linear*/
/*!@def DMA_TYPE_2D DMA transfer type, 2D*/
/*!@def DMA_TYPE_FIFO DMA transfer type, FIFO*/
/*!@def DMA_TYPE_EBE DMA transfer type, end-of-burst enable FIFO*/
#define DMA_TYPE_LINEAR	0x0
#define DMA_TYPE_2D		0x01
#define DMA_TYPE_FIFO		0x2
#define DMA_TYPE_EBE		0x3

/*!@def DMA_DONE DMA transfer done*/
/*!@def DMA_BURST_TIMEOUT DMA transfer timeout error*/
/*!@def DMA_REQUEST_TIMEOUT DMA transfer request timeout error*/
/*!@def DMA_TRANSFER_ERROR DMA transfer error*/
/*!@def DMA_BUFFER_OVERFLOW DMA transfer buffer overflow error*/
#define DMA_DONE		0x1000
#define DMA_BURST_TIMEOUT 	0x1
#define DMA_REQUEST_TIMEOUT 	0x2
#define DMA_TRANSFER_ERROR	0x4
#define DMA_BUFFER_OVERFLOW	0x8

/*!@brief DMA control register*/
typedef struct {
	volatile u32 CEN:1;	/*!< Dma channel enable              */
	volatile u32 FRC:1;	/*!<  Force a dma cycle bit           */
	volatile u32 RPT:1;	/*!<  Repeat bit              */
	volatile u32 REN:1;	/*!<   Request enable bit      */
	volatile u32 SSIZ:2;	/*!<   Source port size, 2 bit in length               */
	volatile u32 DSIZ:2;	/*!<  Dest port size, 2 bit in length         */
	volatile u32 MSEL:1;	/*!<  2D memory register set  bit             */
	volatile u32 MDIR:1;	/*!< Transfer direction, inversed or normal          */
	volatile u32 SMOD:2;	/*!<  Source mode, 2 bit in length            */
	volatile u32 DMOD:2;	/*!<  Dest mode, 2 bit in length              */
	volatile u32 ACRPT:1;	/*!<  Auto clear repeat bit           */
	volatile u32 Reserver:17;	/*!<  Reserved bits           */

} dma_regs_control;

#define DMA_CTL_CEN 0x1
#define DMA_CTL_FRC 0x2
#define DMA_CTL_RPT 0x4
#define DMA_CTL_REN 0x8

#define DMA_CTL_MSEL 0x100
#define DMA_CTL_MDIR 0x200
#define DMA_CTL_ACRPT 0x4000

#define DMA_CTL_GET_SSIZ(x) (((x)>>4)&0x3)
#define DMA_CTL_GET_DSIZ(x) (((x)>>6)&0x3)
#define DMA_CTL_GET_SMOD(x)  (((x)>>10)&0x3)
#define DMA_CTL_GET_DMOD(x)  (((x)>>12)&0x3)

#define DMA_CTL_SET_SSIZ(x,value) 	do{ \
									(x)&=~(0x3<<4); \
									(x)|=(value)<<4; 		\
								}while(0)

#define DMA_CTL_SET_DSIZ(x,value) 	do{ \
									(x)&=~(0x3<<6); \
									(x)|=(value)<<6; 		\
								}while(0)

#define DMA_CTL_SET_SMOD(x,value) 	do{ \
									(x)&=~(0x3<<10); \
									(x)|=(value)<<10; 		\
								}while(0)

#define DMA_CTL_SET_DMOD(x,value) 	do{ \
									(x)&=~(0x3<<12); \
									(x)|=(value)<<12; 		\
								}while(0)

typedef struct {
	volatile u32 SourceAddr;
	volatile u32 DestAddr;
	volatile u32 Count;
	volatile u32 Ctl;
	volatile u32 RequestSource;
	volatile u32 BurstLength;
	union {
		volatile u32 ReqTimeout;
		volatile u32 BusUtilt;
	};
	volatile u32 transferd;
} dma_regs_t;

#ifndef TRANSFER_32BIT
/*!
 * This defines DMA access data size
 */

#define TRANSFER_8BIT       DMA_MEM_SIZE_8
#define TRANSFER_16BIT      DMA_MEM_SIZE_16
#define TRANSFER_32BIT      DMA_MEM_SIZE_32

#endif

/*!
 * This defines maximum device name length passed during mxc_request_dma().
 */
#define MAX_DEVNAME_LENGTH 32
#define MAX_BD_SIZE	32

/*!
 * Structure containing dma channel parameters.
 */
typedef struct {
	unsigned long dma_chan;	/*!< the dma channel information: dynamic or channel number */
	u32 mode:1;		/*!< the initialized dma mode, 0 for dma read, 1 for dma write */
	u32 rto_en:1;		/*!< enable request-timeout. It is valid when REN=1 */
	u32 dir:1;		/*!< Transfer direction, 0 for increment, 1 for decrement */
	u32 dma_chaining:1;	/*!< Autoclear bit for chainbuffer */
	u32 ren:1;		/*!< enable transfer based request signal */
	u32 M2D_Valid:1;	/*!< enable 2D address module. 0 for disable it. 1 for enabled it */
	u32 msel:1;		/*!<2D memory selection, 0 for set A, 1 for set B */
	u32 burstLength;	/*!<    Channel burst length    */
	u32 request;		/*!<   Request source. */
	u32 busuntils;		/*!<   when REN=0, Bus utilization, otherwise it it request timeout */
	u32 sourceType;		/*!<    Source type, see DMA_TYPE_* */
	u32 sourcePort;		/*!<    Source port size, see DMA_MEM_SIZE_*  */
	u32 destType;		/*!<    Destination type, see DMA_TYPE_*    */
	u32 destPort;		/*!<    Destination port size, see DMA_MEM_SIZE_*       */
	__u32 per_address;	/*< Peripheral source/destination
				 *   physical address
				 */
	u32 W;			/*!<    2D Wide-size            */
	u32 X;			/*!<    2D X-size               */
	u32 Y;			/*!<    2D Y-size               */
} mx2_dma_info_t;

/*!
 * Structure of dma buffer descriptor
 */
typedef struct {
	unsigned long state;	/*!< dma bd state */
	int mode;		/*!< the dma mode of this bd */
	unsigned long count;	/*!< the length of the dma transfer */
	unsigned long src_addr;	/*!< the source address of the dma transfer */
	unsigned long dst_addr;	/*!< the destination address of the dma transfer */
} mx2_dma_bd_t;

/*!
 * the states of dma buffer descriptor
 */
#define DMA_BD_ST_BUSY	0x20000000	/*!< dma bd is transfering or has be configured into controller */
#define DMA_BD_ST_PEND	0x10000000	/*!< dma bd is waiting to be configured into controller */
#define DMA_BD_ST_LAST	0x08000000	/*!< dma bd is the last dma bd which is built in one dma transfer request
					 *          When completed this bd, the callback function must be called.
					 */

/*!
 *  This structure containing the private information for MX2
 */
typedef struct mx2_dma_priv_s {
	unsigned int dma_chaining:1;	/* 1: using headware dma chaining feature */
	unsigned int ren:1;	/* 1: dma start besed on request signal */
	unsigned long trans_bytes;	/* To store the transfered data bytes in this transfer */
	mx2_dma_info_t *dma_info;	/* To store the pointer for dma parameter for reading and wirting */
	int bd_rd;		/* the read index of bd ring */
	int bd_wr;		/* the write index of bd ring */
	atomic_t bd_used;	/* the valid bd number in bd ring */
	mx2_dma_bd_t *bd_ring;	/* the pointer of bd ring */
	unsigned long dma_base;	/* register base address of this channel */
	int dma_irq;		/* irq number of this channel */
} mx2_dma_priv_t;

/*!
 * @brief get the dma info by channel_id
 */
extern mx2_dma_info_t *mxc_dma_get_info(mxc_dma_device_t channel_id);

/*!
 * @brief: scan dma parameter list . And collect information about which channels are dynamic .
 */
extern void mxc_dma_load_info(mxc_dma_channel_t * dma);

#endif
