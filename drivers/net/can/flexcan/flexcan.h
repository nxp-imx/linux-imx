/*
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file flexcan.h
 *
 * @brief FlexCan definitions.
 *
 * @ingroup can
 */

#ifndef __CAN_FLEXCAN_H__
#define __CAN_FLEXCAN_H__

#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/can.h>
#include <linux/can/core.h>
#include <linux/can/error.h>

#define FLEXCAN_DEVICE_NAME	"FlexCAN"

#define CAN_MB_RX_INACTIVE	0x0
#define CAN_MB_RX_EMPTY		0x4
#define CAN_MB_RX_FULL		0x2
#define CAN_MB_RX_OVERRUN	0x6
#define CAN_MB_RX_BUSY		0x1

#define CAN_MB_TX_INACTIVE	0x8
#define CAN_MB_TX_ABORT		0x9
#define CAN_MB_TX_ONCE		0xC
#define CAN_MB_TX_REMOTE	0xA

struct can_hw_mb {
	unsigned int mb_cs;
	unsigned int mb_id;
	unsigned char mb_data[8];
};

#define MB_CS_CODE_OFFSET	24
#define MB_CS_CODE_MASK		(0xF << MB_CS_CODE_OFFSET)
#define MB_CS_SRR_OFFSET	22
#define MB_CS_SRR_MASK		(0x1 << MB_CS_SRR_OFFSET)
#define MB_CS_IDE_OFFSET	21
#define MB_CS_IDE_MASK		(0x1 << MB_CS_IDE_OFFSET)
#define MB_CS_RTR_OFFSET	20
#define MB_CS_RTR_MASK		(0x1 << MB_CS_RTR_OFFSET)
#define MB_CS_LENGTH_OFFSET	16
#define MB_CS_LENGTH_MASK	(0xF << MB_CS_LENGTH_OFFSET)
#define MB_CS_TIMESTAMP_OFFSET	0
#define MB_CS_TIMESTAMP_MASK	(0xFF << MB_CS_TIMESTAMP_OFFSET)

#define CAN_HW_REG_MCR		0x00
#define CAN_HW_REG_CTRL		0x04
#define CAN_HW_REG_TIMER	0x08
#define CAN_HW_REG_RXGMASK	0x10
#define CAN_HW_REG_RX14MASK	0x14
#define CAN_HW_REG_RX15MASK	0x18
#define CAN_HW_REG_ECR		0x1C
#define CAN_HW_REG_ESR		0x20
#define CAN_HW_REG_IMASK2	0x24
#define CAN_HW_REG_IMASK1	0x28
#define CAN_HW_REG_IFLAG2	0x2C
#define CAN_HW_REG_IFLAG1	0x30

#define CAN_MB_BASE	0x0080
#define CAN_RXMASK_BASE	0x0880
#define CAN_FIFO_BASE	0xE0

#define __MCR_MDIS		(1 << 31)
#define __MCR_FRZ		(1 << 30)
#define __MCR_FEN		(1 << 29)
#define __MCR_HALT		(1 << 28)
#define __MCR_NOTRDY		(1 << 27)
#define __MCR_WAK_MSK		(1 << 26)
#define __MCR_SOFT_RST		(1 << 25)
#define __MCR_FRZ_ACK		(1 << 24)
#define __MCR_SLF_WAK		(1 << 22)
#define __MCR_WRN_EN		(1 << 21)
#define __MCR_LPM_ACK		(1 << 20)
#define __MCR_WAK_SRC		(1 << 19)
#define __MCR_DOZE		(1 << 18)
#define __MCR_SRX_DIS		(1 << 17)
#define __MCR_BCC		(1 << 16)
#define __MCR_LPRIO_EN		(1 << 13)
#define __MCR_AEN		(1 << 12)
#define __MCR_MAX_IDAM_OFFSET 	8
#define __MCR_MAX_IDAM_MASK 	(0x3 << __MCR_MAX_IDAM_OFFSET)
#define __MCR_MAX_IDAM_A	(0x0 << __MCR_MAX_IDAM_OFFSET)
#define __MCR_MAX_IDAM_B	(0x1 << __MCR_MAX_IDAM_OFFSET)
#define __MCR_MAX_IDAM_C	(0x2 << __MCR_MAX_IDAM_OFFSET)
#define __MCR_MAX_IDAM_D	(0x3 << __MCR_MAX_IDAM_OFFSET)
#define __MCR_MAX_MB_OFFSET 	0
#define __MCR_MAX_MB_MASK 	(0x3F)

#define __CTRL_PRESDIV_OFFSET	24
#define __CTRL_PRESDIV_MASK	(0xFF << __CTRL_PRESDIV_OFFSET)
#define __CTRL_RJW_OFFSET	22
#define __CTRL_RJW_MASK		(0x3 << __CTRL_RJW_OFFSET)
#define __CTRL_PSEG1_OFFSET	19
#define __CTRL_PSEG1_MASK	(0x7 << __CTRL_PSEG1_OFFSET)
#define __CTRL_PSEG2_OFFSET	16
#define __CTRL_PSEG2_MASK	(0x7 << __CTRL_PSEG2_OFFSET)
#define __CTRL_BOFF_MSK		(0x1 << 15)
#define __CTRL_ERR_MSK		(0x1 << 14)
#define __CTRL_CLK_SRC		(0x1 << 13)
#define __CTRL_LPB		(0x1 << 12)
#define __CTRL_TWRN_MSK		(0x1 << 11)
#define __CTRL_RWRN_MSK		(0x1 << 10)
#define __CTRL_SMP		(0x1 << 7)
#define __CTRL_BOFF_REC		(0x1 << 6)
#define __CTRL_TSYN		(0x1 << 5)
#define __CTRL_LBUF		(0x1 << 4)
#define __CTRL_LOM		(0x1 << 3)
#define __CTRL_PROPSEG_OFFSET	0
#define __CTRL_PROPSEG_MASK	(0x7)

#define __ECR_TX_ERR_COUNTER(x) ((x) & 0xFF)
#define __ECR_RX_ERR_COUNTER(x) (((x) >> 8) & 0xFF)
#define __ECR_PASSIVE_THRESHOLD	128
#define __ECR_ACTIVE_THRESHOLD	96

#define __ESR_TWRN_INT		(0x1 << 17)
#define __ESR_RWRN_INT		(0x1 << 16)
#define __ESR_BIT1_ERR		(0x1 << 15)
#define __ESR_BIT0_ERR		(0x1 << 14)
#define __ESR_ACK_ERR		(0x1 << 13)
#define __ESR_CRC_ERR		(0x1 << 12)
#define __ESR_FRM_ERR		(0x1 << 11)
#define __ESR_STF_ERR		(0x1 << 10)
#define __ESR_TX_WRN		(0x1 << 9)
#define __ESR_RX_WRN		(0x1 << 8)
#define __ESR_IDLE		(0x1 << 7)
#define __ESR_TXRX		(0x1 << 6)
#define __ESR_FLT_CONF_OFF	4
#define __ESR_FLT_CONF_MASK	(0x3 << __ESR_FLT_CONF_OFF)
#define __ESR_BOFF_INT		(0x1 << 2)
#define __ESR_ERR_INT		(0x1 << 1)
#define __ESR_WAK_INT		(0x1)

#define __ESR_INTERRUPTS	(__ESR_WAK_INT | __ESR_ERR_INT | \
				__ESR_BOFF_INT | __ESR_TWRN_INT | \
				__ESR_RWRN_INT)

#define __FIFO_OV_INT		0x0080
#define __FIFO_WARN_INT		0x0040
#define __FIFO_RDY_INT		0x0020

struct flexcan_device {
	struct mutex mutex;
	void *io_base;
	struct can_hw_mb *hwmb;
	unsigned int *rx_mask;
	unsigned int xmit_mb;
	unsigned int bitrate;
	/* word 1 */
	unsigned int br_presdiv:8;
	unsigned int br_rjw:2;
	unsigned int br_propseg:3;
	unsigned int br_pseg1:3;
	unsigned int br_pseg2:3;
	unsigned int maxmb:6;
	unsigned int xmit_maxmb:6;
	unsigned int wd1_resv:1;

	/* word 2 */
	unsigned int fifo:1;
	unsigned int wakeup:1;
	unsigned int srx_dis:1;
	unsigned int wak_src:1;
	unsigned int bcc:1;
	unsigned int lprio:1;
	unsigned int abort:1;
	unsigned int br_clksrc:1;
	unsigned int loopback:1;
	unsigned int smp:1;
	unsigned int boff_rec:1;
	unsigned int tsyn:1;
	unsigned int listen:1;

	unsigned int ext_msg:1;
	unsigned int std_msg:1;

	struct timer_list timer;
	struct platform_device *dev;
	struct regulator *core_reg;
	struct regulator *io_reg;
	struct clk *clk;
	int irq;
};

#define FLEXCAN_MAX_FIFO_MB	8
#define FLEXCAN_MAX_MB		64
#define FLEXCAN_MAX_PRESDIV	256
#define FLEXCAN_MAX_RJW		4
#define FLEXCAN_MAX_PSEG1	8
#define FLEXCAN_MAX_PSEG2	8
#define FLEXCAN_MAX_PROPSEG	8
#define FLEXCAN_MAX_BITRATE	1000000

extern struct net_device *flexcan_device_alloc(struct platform_device *pdev,
					       void (*setup) (struct net_device
							      *dev));
extern void flexcan_device_free(struct platform_device *pdev);

extern void flexcan_mbm_init(struct flexcan_device *flexcan);
extern void flexcan_mbm_isr(struct net_device *dev);
extern int flexcan_mbm_xmit(struct flexcan_device *flexcan,
			    struct can_frame *frame);
#endif				/* __CAN_FLEXCAN_H__ */
