/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/uaccess.h>
#include <linux/mxc_mlb.h>
#include <linux/iram_alloc.h>
#include <linux/fsl_devices.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/sched.h>

#define DRIVER_NAME "mxc_mlb150"

/*!
 * MLB module memory map registers define
 */
#define MLB150_REG_MLBC0		0x0
#define MLB150_MLBC0_MLBEN		(0x1)
#define MLB150_MLBC0_MLBCLK_MASK	(0x7 << 2)
#define MLB150_MLBC0_MLBCLK_SHIFT	(2)
#define MLB150_MLBC0_MLBPEN		(0x1 << 5)
#define MLB150_MLBC0_MLBLK		(0x1 << 7)
#define MLB150_MLBC0_ASYRETRY		(0x1 << 12)
#define MLB150_MLBC0_CTLRETRY		(0x1 << 12)
#define MLB150_MLBC0_FCNT_MASK		(0x7 << 15)
#define MLB150_MLBC0_FCNT_SHIFT		(15)

#define MLB150_REG_MLBPC0		0x8
#define MLB150_MLBPC0_MCLKHYS		(0x1 << 11)

#define MLB150_REG_MS0			0xC
#define MLB150_REG_MS1			0x14

#define MLB150_REG_MSS			0x20
#define MLB150_MSS_RSTSYSCMD		(0x1)
#define MLB150_MSS_LKSYSCMD		(0x1 << 1)
#define MLB150_MSS_ULKSYSCMD		(0x1 << 2)
#define MLB150_MSS_CSSYSCMD		(0x1 << 3)
#define MLB150_MSS_SWSYSCMD		(0x1 << 4)
#define MLB150_MSS_SERVREQ		(0x1 << 5)

#define MLB150_REG_MSD			0x24

#define MLB150_REG_MIEN			0x2C
#define MLB150_MIEN_ISOC_PE		(0x1)
#define MLB150_MIEN_ISOC_BUFO		(0x1 << 1)
#define MLB150_MIEN_SYNC_PE		(0x1 << 16)
#define MLB150_MIEN_ARX_DONE		(0x1 << 17)
#define MLB150_MIEN_ARX_PE		(0x1 << 18)
#define MLB150_MIEN_ARX_BREAK		(0x1 << 19)
#define MLB150_MIEN_ATX_DONE		(0x1 << 20)
#define MLB150_MIEN_ATX_PE		(0x1 << 21)
#define MLB150_MIEN_ATX_BREAK		(0x1 << 22)
#define MLB150_MIEN_CRX_DONE		(0x1 << 24)
#define MLB150_MIEN_CRX_PE		(0x1 << 25)
#define MLB150_MIEN_CRX_BREAK		(0x1 << 26)
#define MLB150_MIEN_CTX_DONE		(0x1 << 27)
#define MLB150_MIEN_CTX_PE		(0x1 << 28)
#define MLB150_MIEN_CTX_BREAK		(0x1 << 29)

#define MLB150_REG_MLBPC2		0x34
#define MLB150_REG_MLBPC1		0x38
#define MLB150_MLBPC1_VAL		(0x00000888)

#define MLB150_REG_MLBC1		0x3C
#define MLB150_MLBC1_LOCK		(0x1 << 6)
#define MLB150_MLBC1_CLKM		(0x1 << 7)
#define MLB150_MLBC1_NDA_MASK		(0xFF << 8)
#define MLB150_MLBC1_NDA_SHIFT		(8)

#define MLB150_REG_HCTL			0x80
#define MLB150_HCTL_RST0		(0x1)
#define MLB150_HCTL_RST1		(0x1 << 1)
#define MLB150_HCTL_EN			(0x1 << 15)

#define MLB150_REG_HCMR0		0x88
#define MLB150_REG_HCMR1		0x8C
#define MLB150_REG_HCER0		0x90
#define MLB150_REG_HCER1		0x94
#define MLB150_REG_HCBR0		0x98
#define MLB150_REG_HCBR1		0x9C

#define MLB150_REG_MDAT0		0xC0
#define MLB150_REG_MDAT1		0xC4
#define MLB150_REG_MDAT2		0xC8
#define MLB150_REG_MDAT3		0xCC

#define MLB150_REG_MDWE0		0xD0
#define MLB150_REG_MDWE1		0xD4
#define MLB150_REG_MDWE2		0xD8
#define MLB150_REG_MDWE3		0xDC

#define MLB150_REG_MCTL			0xE0
#define MLB150_MCTL_XCMP		(0x1)

#define MLB150_REG_MADR			0xE4
#define MLB150_MADR_WNR			(0x1 << 31)
#define MLB150_MADR_TB			(0x1 << 30)
#define MLB150_MADR_ADDR_MASK		(0x7f << 8)
#define MLB150_MADR_ADDR_SHIFT		(0)

#define MLB150_REG_ACTL			0x3C0
#define MLB150_ACTL_MPB			(0x1 << 4)
#define MLB150_ACTL_DMAMODE		(0x1 << 2)
#define MLB150_ACTL_SMX			(0x1 << 1)
#define MLB150_ACTL_SCE			(0x1)

#define MLB150_REG_ACSR0		0x3D0
#define MLB150_REG_ACSR1		0x3D4
#define MLB150_REG_ACMR0		0x3D8
#define MLB150_REG_ACMR1		0x3DC

#define MLB150_REG_CAT_MDATn(ch) (MLB150_REG_MDAT0 + ((ch % 8) >> 1) * 4)
#define MLB150_REG_CAT_MDWEn(ch) (MLB150_REG_MDWE0 + ((ch % 8) >> 1) * 4)

#define MLB150_LOGIC_CH_NUM		(64)
#define MLB150_BUF_CDT_OFFSET		(0x0)
#define MLB150_BUF_ADT_OFFSET		(0x40)
#define MLB150_BUF_CAT_MLB_OFFSET	(0x80)
#define MLB150_BUF_CAT_HBI_OFFSET	(0x88)
#define MLB150_BUF_CTR_END_OFFSET	(0x8F)

#define MLB150_CAT_MODE_RX		(0x1 << 0)
#define MLB150_CAT_MODE_TX		(0x1 << 1)
#define MLB150_CAT_MODE_INBOUND_DMA	(0x1 << 8)
#define MLB150_CAT_MODE_OUTBOUND_DMA	(0x1 << 9)

#define MLB150_CH_SYNC_BUF_DEP		(128 * 4 * 4)
#define MLB150_CH_CTRL_BUF_DEP		(64)
#define MLB150_CH_ASYNC_BUF_DEP		(2048)
#define MLB150_CH_ISOC_BLK_SIZE		(196)
#define MLB150_CH_ISOC_BLK_NUM		(3)
#define MLB150_CH_ISOC_BUF_DEP		(MLB150_CH_ISOC_BLK_SIZE * MLB150_CH_ISOC_BLK_NUM)

#define MLB150_CH_SYNC_DBR_BUF_OFFSET	(0x0)
#define MLB150_CH_CTRL_DBR_BUF_OFFSET	(MLB150_CH_SYNC_DBR_BUF_OFFSET + 2 * MLB150_CH_SYNC_BUF_DEP)
#define MLB150_CH_ASYNC_DBR_BUF_OFFSET	(MLB150_CH_CTRL_DBR_BUF_OFFSET + 2 * MLB150_CH_CTRL_BUF_DEP)
#define MLB150_CH_ISOC_DBR_BUF_OFFSET	(MLB150_CH_ASYNC_DBR_BUF_OFFSET + 2 * MLB150_CH_ASYNC_BUF_DEP)

static u32 mlb150_ch_packet_buf_size[4] = {
	MLB150_CH_SYNC_BUF_DEP,
	MLB150_CH_CTRL_BUF_DEP,
	MLB150_CH_ASYNC_BUF_DEP,
	MLB150_CH_ISOC_BUF_DEP
};

#define MLB150_DBR_BUF_START 0x00000

#define MLB150_CDT_LEN			(16)
#define MLB150_ADT_LEN			(16)
#define MLB150_CAT_LEN			(2)

#define MLB150_CDT_SZ		(MLB150_CDT_LEN * MLB150_LOGIC_CH_NUM)
#define MLB150_ADT_SZ		(MLB150_ADT_LEN * MLB150_LOGIC_CH_NUM)
#define MLB150_CAT_SZ		(MLB150_CAT_LEN * MLB150_LOGIC_CH_NUM * 2)

#define MLB150_CDT_BASE(base)		(base + MLB150_BUF_CDT_OFFSET)
#define MLB150_ADT_BASE(base)		(base + MLB150_BUF_ADT_OFFSET)
#define MLB150_CAT_MLB_BASE(base)	(base + MLB150_BUF_CAT_MLB_OFFSET)
#define MLB150_CAT_HBI_BASE(base)	(base + MLB150_BUF_CAT_HBI_OFFSET)

#define MLB150_CDTn_ADDR(base, n)	(base + MLB150_BUF_CDT_OFFSET + n * MLB150_CDT_LEN)
#define MLB150_ADTn_ADDR(base, n)	(base + MLB150_BUF_ADT_OFFSET + n * MLB150_ADT_LEN)
#define MLB150_CATn_MLB_ADDR(base, n)	(base + MLB150_BUF_CAT_MLB_OFFSET + n * MLB150_CAT_LEN)
#define MLB150_CATn_HBI_ADDR(base, n)	(base + MLB150_BUF_CAT_HBI_OFFSET + n * MLB150_CAT_LEN)

#define CAT_CL_SHIFT		(0x0)
#define CAT_CT_SHIFT		(8)
#define CAT_CE			(0x1 << 11)
#define CAT_RNW			(0x1 << 12)
#define CAT_MT			(0x1 << 13)
#define CAT_FCE			(0x1 << 14)
#define CAT_MFE			(0x1 << 14)

#define CDT_WSBC_SHIFT		(14)
#define CDT_WPC_SHIFT		(11)
#define CDT_RSBC_SHIFT		(30)
#define CDT_RPC_SHIFT		(27)
#define CDT_WPC_1_SHIFT		(12)
#define CDT_RPC_1_SHIFT		(28)
#define CDT_WPTR_SHIFT		(0)
#define CDT_SYNC_WSTS_MASK	(0x0000f000)
#define CDT_SYNC_WSTS_SHIFT	(12)
#define CDT_CTRL_ASYNC_WSTS_MASK	(0x0000f000)
#define CDT_CTRL_ASYNC_WSTS_SHIFT	(12)
#define CDT_ISOC_WSTS_MASK	(0x0000e000)
#define CDT_ISOC_WSTS_SHIFT	(13)
#define CDT_RPTR_SHIFT		(16)
#define CDT_SYNC_RSTS_MASK	(0xf0000000)
#define CDT_SYNC_RSTS_SHIFT	(28)
#define CDT_CTRL_ASYNC_RSTS_MASK	(0xf0000000)
#define CDT_CTRL_ASYNC_RSTS_SHIFT	(28)
#define CDT_ISOC_RSTS_MASK	(0xe0000000)
#define CDT_ISOC_RSTS_SHIFT	(29)
#define CDT_CTRL_ASYNC_WSTS_1	(0x1 << 14)
#define CDT_CTRL_ASYNC_RSTS_1	(0x1 << 15)
#define CDT_BD_SHIFT		(0)
#define CDT_BA_SHIFT		(16)
#define CDT_BS_SHIFT		(0)
#define CDT_BF_SHIFT		(31)

#define ADT_PG			(0x1 << 13)
#define ADT_LE			(0x1 << 14)
#define ADT_CE			(0x1 << 15)
#define ADT_BD1_SHIFT		(0)
#define ADT_ERR1		(0x1 << 13)
#define ADT_DNE1		(0x1 << 14)
#define ADT_RDY1		(0x1 << 15)
#define ADT_BD2_SHIFT		(16)
#define ADT_ERR2		(0x1 << 29)
#define ADT_DNE2		(0x1 << 30)
#define ADT_RDY2		(0x1 << 31)
#define ADT_BA1_SHIFT		(0x0)
#define ADT_BA2_SHIFT		(0x0)
#define ADT_PS1			(0x1 << 12)
#define ADT_PS2			(0x1 << 28)
#define ADT_MEP1		(0x1 << 11)
#define ADT_MEP2		(0x1 << 27)

#define MLB_CONTROL_TX_CHANN	(0 << 4)
#define MLB_CONTROL_RX_CHANN	(1 << 4)
#define MLB_ASYNC_TX_CHANN	(2 << 4)
#define MLB_ASYNC_RX_CHANN	(3 << 4)

#define MLB_MINOR_DEVICES	4
#define MLB_CONTROL_DEV_NAME	"ctrl"
#define MLB_ASYNC_DEV_NAME	"async"
#define MLB_SYNC_DEV_NAME	"sync"
#define MLB_ISOC_DEV_NAME	"isoc"

#define TX_CHANNEL		0
#define RX_CHANNEL		1
#define PING_BUF_MAX_SIZE	(2 * 1024)
#define PONG_BUF_MAX_SIZE	(2 * 1024)
/* max package data size */
#define ASYNC_PACKET_SIZE	1024
#define CTRL_PACKET_SIZE	64
#define TRANS_RING_NODES	10

#define MLB_IRAM_SIZE		(MLB_MINOR_DEVICES * (PING_BUF_MAX_SIZE + PONG_BUF_MAX_SIZE))
#define _get_txchan(dev)	mlb_devinfo[dev].channels[TX_CHANNEL]
#define _get_rxchan(dev)	mlb_devinfo[dev].channels[RX_CHANNEL]

enum MLB_CTYPE {
	MLB_CTYPE_SYNC,
	MLB_CTYPE_CTRL,
	MLB_CTYPE_ASYNC,
	MLB_CTYPE_ISOC,
};

enum MLB150_CLK_SPEED {
	MLB150_CLK_256FS,
	MLB150_CLK_512FS,
	MLB150_CLK_1024FS,
	MLB150_CLK_2048FS,
	MLB150_CLK_3072FS,
	MLB150_CLK_4096FS,
	MLB150_CLK_6144FS,
	MLB150_CLK_8192FS,
};

/*!
 * Ring buffer
 */
#define MLB_RING_BUF_INIT(r)	{	\
	r->wpos = 0;	\
	r->rpos = 0;	\
}

#define MLB_RING_BUF_IS_FULL(r) (((r->wpos + 1) % TRANS_RING_NODES) == r->rpos)
#define MLB_RING_BUF_IS_EMPTY(r) (r->rpos == r->wpos)
#define MLB_RING_BUF_ENQUE(r, buf) {	\
	memcpy(r->node[r->wpos].data, buf, r->node.size);	\
	r->wpos = (r->wpos + 1) % TRANS_RING_NODES;	\
}
#define MLB_RING_BUF_DEQUE(r, buf) {	\
	memcpy(buf, r->node[r->rpos].data, r->node.size);	\
	r->rpos = (r->rpos + 1) % TRANS_RING_NODES;	\
}

struct mlb_ringbuf {
	u32 wpos;
	u32 rpos;
	u32 size;
	/* Last buffer is for package drop */
	u8  *virt_bufs[TRANS_RING_NODES + 1];
	u32 phy_addrs[TRANS_RING_NODES + 1];
};

struct mlb_channel_info {

	/* channel address */
	s32 address;
	/* DBR buf head */
	u32 dbr_buf_head;
	/* ping buffer head */
	u32 ping_buf_head;
	/* pong buffer head */
	u32 pong_buf_head;
	/* ping buffer physical head */
	u32 ping_phy_head;
	/* pong buffer physical head */
	u32 pong_phy_head;
	/* channel buffer size */
	u32 buf_size;
	/* channel buffer current ptr */
	u32 buf_ptr;
	/* channel buffer phy addr */
	u32 buf_phy_addr;
	/* packet start indicator */
	u32 ps_ind;
	/* packet remain size */
	u32 pkt_remain_size;
	/* buffer spin lock */
	rwlock_t buf_lock;
};

struct mlb_dev_info {

	/* device node name */
	const char dev_name[20];
	/* channel type */
	const unsigned int channel_type;
	/* ch fps */
	enum MLB150_CLK_SPEED fps;
	/* channel info for tx/rx */
	struct mlb_channel_info channels[2];
	/* rx ring buffer */
	struct mlb_ringbuf rx_bufs;
	/* exception event */
	unsigned long ex_event;
	/* tx busy indicator */
	unsigned long tx_busy;
	/* channel started up or not */
	atomic_t on;
	/* device open count */
	atomic_t opencnt;
	/* wait queue head for channel */
	wait_queue_head_t rd_wq;
	wait_queue_head_t wt_wq;
	/* spinlock for event access */
	spinlock_t event_lock;
};

static struct mlb_dev_info mlb_devinfo[MLB_MINOR_DEVICES] = {
	{
	.dev_name = MLB_SYNC_DEV_NAME,
	.channel_type = MLB_CTYPE_SYNC,
	.channels = {
		[0] = {
			.buf_size = MLB150_CH_SYNC_BUF_DEP,
			.dbr_buf_head = MLB150_CH_SYNC_DBR_BUF_OFFSET,
			.buf_lock =
				__RW_LOCK_UNLOCKED(mlb_devinfo[0].channels[0].
					buf_lock),
		},
		[1] = {
			.buf_size = MLB150_CH_SYNC_BUF_DEP,
			.dbr_buf_head = MLB150_CH_SYNC_DBR_BUF_OFFSET
					+ MLB150_CH_SYNC_BUF_DEP,
			.buf_lock =
				__RW_LOCK_UNLOCKED(mlb_devinfo[0].channels[1].
					buf_lock),
		},
	},
	.on = ATOMIC_INIT(0),
	.opencnt = ATOMIC_INIT(0),
	.rd_wq = __WAIT_QUEUE_HEAD_INITIALIZER(mlb_devinfo[0].rd_wq),
	.wt_wq = __WAIT_QUEUE_HEAD_INITIALIZER(mlb_devinfo[0].wt_wq),
	.event_lock = __SPIN_LOCK_UNLOCKED(mlb_devinfo[0].event_lock),
	},
	{
	.dev_name = MLB_CONTROL_DEV_NAME,
	.channel_type = MLB_CTYPE_CTRL,
	.channels = {
		[0] = {
			.buf_size = MLB150_CH_CTRL_BUF_DEP,
			.dbr_buf_head = MLB150_CH_CTRL_DBR_BUF_OFFSET,
			.buf_lock =
			__RW_LOCK_UNLOCKED(mlb_devinfo[1].channels[0].
					buf_lock),
		},
		[1] = {
			.buf_size = MLB150_CH_CTRL_BUF_DEP,
			.dbr_buf_head = MLB150_CH_CTRL_DBR_BUF_OFFSET
					+ MLB150_CH_CTRL_BUF_DEP,
			.buf_lock =
			__RW_LOCK_UNLOCKED(mlb_devinfo[1].channels[1].
					buf_lock),
		},
	},
	.on = ATOMIC_INIT(0),
	.opencnt = ATOMIC_INIT(0),
	.rd_wq = __WAIT_QUEUE_HEAD_INITIALIZER(mlb_devinfo[1].rd_wq),
	.wt_wq = __WAIT_QUEUE_HEAD_INITIALIZER(mlb_devinfo[1].wt_wq),
	.event_lock = __SPIN_LOCK_UNLOCKED(mlb_devinfo[1].event_lock),
	},
	{
	.dev_name = MLB_ASYNC_DEV_NAME,
	.channel_type = MLB_CTYPE_ASYNC,
	.channels = {
		[0] = {
			.buf_size = MLB150_CH_ASYNC_BUF_DEP,
			.dbr_buf_head = MLB150_CH_ASYNC_DBR_BUF_OFFSET,
			.buf_lock =
			__RW_LOCK_UNLOCKED(mlb_devinfo[2].channels[0].
					buf_lock),
		},
		[1] = {
			.buf_size = MLB150_CH_ASYNC_BUF_DEP,
			.dbr_buf_head = MLB150_CH_ASYNC_DBR_BUF_OFFSET
					+ MLB150_CH_ASYNC_BUF_DEP,
			.buf_lock =
				__RW_LOCK_UNLOCKED(mlb_devinfo[2].channels[1].
					buf_lock),
		},
	},
	.on = ATOMIC_INIT(0),
	.opencnt = ATOMIC_INIT(0),
	.rd_wq = __WAIT_QUEUE_HEAD_INITIALIZER(mlb_devinfo[2].rd_wq),
	.wt_wq = __WAIT_QUEUE_HEAD_INITIALIZER(mlb_devinfo[2].wt_wq),
	.event_lock = __SPIN_LOCK_UNLOCKED(mlb_devinfo[2].event_lock),
	},
	{
	.dev_name = MLB_ISOC_DEV_NAME,
	.channel_type = MLB_CTYPE_ISOC,
	.channels = {
		[0] = {
			.buf_size = MLB150_CH_ISOC_BUF_DEP,
			.dbr_buf_head = MLB150_CH_ISOC_DBR_BUF_OFFSET,
			.buf_lock =
				__RW_LOCK_UNLOCKED(mlb_devinfo[3].channels[0].
					buf_lock),
		},
		[1] = {
			.buf_size = MLB150_CH_ISOC_BUF_DEP,
			.dbr_buf_head = MLB150_CH_ISOC_DBR_BUF_OFFSET
					+ MLB150_CH_ISOC_BUF_DEP,
			.buf_lock =
				__RW_LOCK_UNLOCKED(mlb_devinfo[3].channels[1].
					buf_lock),
		},
	},
	.on = ATOMIC_INIT(0),
	.opencnt = ATOMIC_INIT(0),
	.rd_wq = __WAIT_QUEUE_HEAD_INITIALIZER(mlb_devinfo[3].rd_wq),
	.wt_wq = __WAIT_QUEUE_HEAD_INITIALIZER(mlb_devinfo[3].wt_wq),
	.event_lock = __SPIN_LOCK_UNLOCKED(mlb_devinfo[3].event_lock),
	},
};

static struct regulator *reg_nvcc;	/* NVCC_MLB regulator */
static struct clk *mlb_clk;
static struct clk *mlb_pll_clk;
static dev_t dev;
static struct class *mlb_class;	/* device class */
static struct device *class_dev;
static u32 mlb_base;	/* mlb module base address */
static u32 ahb0_irq, ahb1_irq, mlb_irq;

DEFINE_SPINLOCK(ctr_lock);

#ifdef DEBUG

#define DUMP_REG(reg) pr_debug(#reg": 0x%08x\n", __raw_readl(mlb_base + reg))

static void mlb150_dev_dump_reg(void)
{
	pr_debug("mxc_mlb150: Dump registers:\n");
	DUMP_REG(MLB150_REG_MLBC0);
	DUMP_REG(MLB150_REG_MLBPC0);
	DUMP_REG(MLB150_REG_MS0);
	DUMP_REG(MLB150_REG_MS1);
	DUMP_REG(MLB150_REG_MSS);
	DUMP_REG(MLB150_REG_MSD);
	DUMP_REG(MLB150_REG_MIEN);
	DUMP_REG(MLB150_REG_MLBPC2);
	DUMP_REG(MLB150_REG_MLBPC1);
	DUMP_REG(MLB150_REG_MLBC1);
	DUMP_REG(MLB150_REG_HCTL);
	DUMP_REG(MLB150_REG_HCMR0);
	DUMP_REG(MLB150_REG_HCMR1);
	DUMP_REG(MLB150_REG_HCER0);
	DUMP_REG(MLB150_REG_HCER1);
	DUMP_REG(MLB150_REG_HCBR0);
	DUMP_REG(MLB150_REG_HCBR1);
	DUMP_REG(MLB150_REG_MDAT0);
	DUMP_REG(MLB150_REG_MDAT1);
	DUMP_REG(MLB150_REG_MDAT2);
	DUMP_REG(MLB150_REG_MDAT3);
	DUMP_REG(MLB150_REG_MDWE0);
	DUMP_REG(MLB150_REG_MDWE1);
	DUMP_REG(MLB150_REG_MDWE2);
	DUMP_REG(MLB150_REG_MDWE3);
	DUMP_REG(MLB150_REG_MCTL);
	DUMP_REG(MLB150_REG_MADR);
	DUMP_REG(MLB150_REG_ACTL);
	DUMP_REG(MLB150_REG_ACSR0);
	DUMP_REG(MLB150_REG_ACSR1);
	DUMP_REG(MLB150_REG_ACMR0);
	DUMP_REG(MLB150_REG_ACMR1);
}

static void mlb150_dev_dump_hex(const u8 *buf, u32 len)
{
	u32 i, remain, round_len;

	pr_debug("buf: 0x%08x, len: %d\n", (u32)buf, len);
	remain = len & 0x7;
	round_len = len - remain;
	for (i = 0; i < round_len; i += 8) {
		pr_debug("%02x %02x %02x %02x %02x %02x %02x %02x\n",
			*(buf + i),
			*(buf + i + 1),
			*(buf + i + 2),
			*(buf + i + 3),
			*(buf + i + 4),
			*(buf + i + 5),
			*(buf + i + 6),
			*(buf + i + 7));
	}

	if (remain) {
		i = round_len;
		switch (remain) {
		case 1:
			pr_debug("%02x\n",
				*(buf + i));
			break;
		case 2:
			pr_debug("%02x %02x\n",
				*(buf + i),
				*(buf + i + 1));
			break;
		case 3:
			pr_debug("%02x %02x %02x\n",
				*(buf + i),
				*(buf + i + 1),
				*(buf + i + 2));
			break;
		case 4:
			pr_debug("%02x %02x %02x %02x\n",
				*(buf + i),
				*(buf + i + 1),
				*(buf + i + 2),
				*(buf + i + 3));
			break;
		case 5:
			pr_debug("%02x %02x %02x %02x %02x\n",
				*(buf + i),
				*(buf + i + 1),
				*(buf + i + 2),
				*(buf + i + 3),
				*(buf + i + 4));
			break;
		case 6:
			pr_debug("%02x %02x %02x %02x %02x %02x\n",
				*(buf + i),
				*(buf + i + 1),
				*(buf + i + 2),
				*(buf + i + 3),
				*(buf + i + 4),
				*(buf + i + 5));
			break;
		case 7:
			pr_debug("%02x %02x %02x %02x %02x %02x %02x\n",
				*(buf + i),
				*(buf + i + 1),
				*(buf + i + 2),
				*(buf + i + 3),
				*(buf + i + 4),
				*(buf + i + 5),
				*(buf + i + 6));
			break;
		default:
			break;
		}
	}

	if (i % 8 != 0)
		pr_debug("\n");
}
#endif

static inline void mlb150_dev_enable_ctr_write(u32 mdat0_bits_en,
		u32 mdat1_bits_en, u32 mdat2_bits_en, u32 mdat3_bits_en)
{
	__raw_writel(mdat0_bits_en, mlb_base + MLB150_REG_MDWE0);
	__raw_writel(mdat1_bits_en, mlb_base + MLB150_REG_MDWE1);
	__raw_writel(mdat2_bits_en, mlb_base + MLB150_REG_MDWE2);
	__raw_writel(mdat3_bits_en, mlb_base + MLB150_REG_MDWE3);
}

static inline u8 mlb150_dev_dbr_read(u32 dbr_addr)
{
	s32 timeout = 1000;
	u8  dbr_val = 0;
	unsigned long flags;

	spin_lock_irqsave(&ctr_lock, flags);
	__raw_writel(MLB150_MADR_TB | dbr_addr,
		mlb_base + MLB150_REG_MADR);

	while ((!(__raw_readl(mlb_base + MLB150_REG_MCTL)
			& MLB150_MCTL_XCMP)) &&
			timeout--)
		;

	if (unlikely(0 == timeout))
		return -ETIME;

	dbr_val = __raw_readl(mlb_base + MLB150_REG_MDAT0) & 0x000000ff;

	__raw_writel(0, mlb_base + MLB150_REG_MCTL);
	spin_unlock_irqrestore(&ctr_lock, flags);

	return dbr_val;
}

static inline s32 mlb150_dev_dbr_write(u32 dbr_addr, u8 dbr_val)
{
	s32 timeout = 1000;
	u32 mdat0 = dbr_val & 0x000000ff;
	unsigned long flags;

	spin_lock_irqsave(&ctr_lock, flags);
	__raw_writel(mdat0, mlb_base + MLB150_REG_MDAT0);

	__raw_writel(MLB150_MADR_WNR | MLB150_MADR_TB | dbr_addr,
			mlb_base + MLB150_REG_MADR);

	while ((!(__raw_readl(mlb_base + MLB150_REG_MCTL)
			& MLB150_MCTL_XCMP)) &&
			timeout--)
		;

	if (unlikely(timeout <= 0))
		return -ETIME;

	__raw_writel(0, mlb_base + MLB150_REG_MCTL);
	spin_unlock_irqrestore(&ctr_lock, flags);

	return 0;
}

static s32 mlb150_dev_ctr_read(u32 ctr_offset, u32 *ctr_val)
{
	s32 timeout = 1000;
	unsigned long flags;

	spin_lock_irqsave(&ctr_lock, flags);
	__raw_writel(ctr_offset, mlb_base + MLB150_REG_MADR);

	while ((!(__raw_readl(mlb_base + MLB150_REG_MCTL)
			& MLB150_MCTL_XCMP)) &&
			timeout--)
		;

	if (unlikely(timeout <= 0)) {
		pr_debug("mxc_mlb150: Read CTR timeout\n");
		return -ETIME;
	}

	ctr_val[0] = __raw_readl(mlb_base + MLB150_REG_MDAT0);
	ctr_val[1] = __raw_readl(mlb_base + MLB150_REG_MDAT1);
	ctr_val[2] = __raw_readl(mlb_base + MLB150_REG_MDAT2);
	ctr_val[3] = __raw_readl(mlb_base + MLB150_REG_MDAT3);

	__raw_writel(0, mlb_base + MLB150_REG_MCTL);

	spin_unlock_irqrestore(&ctr_lock, flags);

	return 0;
}

static s32 mlb150_dev_ctr_write(u32 ctr_offset, const u32 *ctr_val)
{
	s32 timeout = 1000;
	unsigned long flags;

	spin_lock_irqsave(&ctr_lock, flags);

	__raw_writel(ctr_val[0], mlb_base + MLB150_REG_MDAT0);
	__raw_writel(ctr_val[1], mlb_base + MLB150_REG_MDAT1);
	__raw_writel(ctr_val[2], mlb_base + MLB150_REG_MDAT2);
	__raw_writel(ctr_val[3], mlb_base + MLB150_REG_MDAT3);

	__raw_writel(MLB150_MADR_WNR | ctr_offset,
			mlb_base + MLB150_REG_MADR);

	while ((!(__raw_readl(mlb_base + MLB150_REG_MCTL)
			& MLB150_MCTL_XCMP)) &&
			timeout--)
		;

	if (unlikely(timeout <= 0)) {
		pr_debug("mxc_mlb150: Write CTR timeout\n");
		return -ETIME;
	}

	__raw_writel(0, mlb_base + MLB150_REG_MCTL);

	spin_unlock_irqrestore(&ctr_lock, flags);

#ifdef DEBUG_CTR
	{
		u32 ctr_rd[4] = { 0 };

		if (!mlb150_dev_ctr_read(ctr_offset, ctr_rd)) {
			if (ctr_val[0] == ctr_rd[0] &&
				ctr_val[1] == ctr_rd[1] &&
				ctr_val[2] == ctr_rd[2] &&
				ctr_val[3] == ctr_rd[3])
				return 0;
			else {
				pr_debug("mxc_mlb150: ctr write failed\n");
				return -EBADE;
			}
		} else {
			pr_debug("mxc_mlb150: ctr read failed\n");
			return -EBADE;
		}
	}
#endif

	return 0;
}

static s32 mlb150_dev_get_adt_sts(u32 ch)
{
	s32 timeout = 1000;
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&ctr_lock, flags);
	__raw_writel(MLB150_BUF_ADT_OFFSET + ch,
			mlb_base + MLB150_REG_MADR);

	while ((!(__raw_readl(mlb_base + MLB150_REG_MCTL)
			& MLB150_MCTL_XCMP)) &&
			timeout--)
		;

	if (unlikely(timeout <= 0)) {
		pr_debug("mxc_mlb150: Read CTR timeout\n");
		return -ETIME;
	}

	reg = __raw_readl(mlb_base + MLB150_REG_MDAT1);

	__raw_writel(0, mlb_base + MLB150_REG_MCTL);
	spin_unlock_irqrestore(&ctr_lock, flags);

#ifdef DEBUG_ADT
	pr_debug("mxc_mlb150: Get ch %d adt sts: 0x%08x\n", ch, reg);
#endif

	return reg;
}

static s32 mlb150_dev_cat_write(u32 ctr_offset, u32 ch, const u16 cat_val)
{
	u16 ctr_val[8] = { 0 };

	if (unlikely(mlb150_dev_ctr_read(ctr_offset, (u32 *)ctr_val)))
		return -ETIME;

	ctr_val[ch % 8] = cat_val;
	if (unlikely(mlb150_dev_ctr_write(ctr_offset, (u32 *)ctr_val)))
		return -ETIME;

	return 0;
}

#define mlb150_dev_cat_mlb_read(ch, cat_val)	\
	mlb150_dev_cat_read(MLB150_BUF_CAT_MLB_OFFSET + (ch >> 3), ch, cat_val)
#define mlb150_dev_cat_mlb_write(ch, cat_val)	\
	mlb150_dev_cat_write(MLB150_BUF_CAT_MLB_OFFSET + (ch >> 3), ch, cat_val)
#define mlb150_dev_cat_hbi_read(ch, cat_val)	\
	mlb150_dev_cat_read(MLB150_BUF_CAT_HBI_OFFSET + (ch >> 3), ch, cat_val)
#define mlb150_dev_cat_hbi_write(ch, cat_val)	\
	mlb150_dev_cat_write(MLB150_BUF_CAT_HBI_OFFSET + (ch >> 3), ch, cat_val)

#define mlb150_dev_cdt_read(ch, cdt_val)	\
	mlb150_dev_ctr_read(MLB150_BUF_CDT_OFFSET + ch, cdt_val)
#define mlb150_dev_cdt_write(ch, cdt_val)	\
	mlb150_dev_ctr_write(MLB150_BUF_CDT_OFFSET + ch, cdt_val)
#define mlb150_dev_adt_read(ch, adt_val)	\
	mlb150_dev_ctr_read(MLB150_BUF_ADT_OFFSET + ch, adt_val)
#define mlb150_dev_adt_write(ch, adt_val)	\
	mlb150_dev_ctr_write(MLB150_BUF_ADT_OFFSET + ch, adt_val)

#ifdef DEBUG
static void mlb150_dev_dump_ctr_tbl(u32 ch_start, u32 ch_end)
{
	u32 i = 0;
	u32 ctr_val[4] = { 0 };

	pr_debug("mxc_mlb150: CDT Table");
	for (i = MLB150_BUF_CDT_OFFSET + ch_start;
			i < MLB150_BUF_CDT_OFFSET + ch_end;
			++i) {
		mlb150_dev_ctr_read(i, ctr_val);
		pr_debug("CTR 0x%02x: 0x%08x, 0x%08x, 0x%08x, 0x%08x\n",
			i, ctr_val[3], ctr_val[2], ctr_val[1], ctr_val[0]);
	}

	pr_debug("mxc_mlb150: ADT Table");
	for (i = MLB150_BUF_ADT_OFFSET + ch_start;
			i < MLB150_BUF_ADT_OFFSET + ch_end;
			++i) {
		mlb150_dev_ctr_read(i, ctr_val);
		pr_debug("CTR 0x%02x: 0x%08x, 0x%08x, 0x%08x, 0x%08x\n",
			i, ctr_val[3], ctr_val[2], ctr_val[1], ctr_val[0]);
	}

	pr_debug("mxc_mlb150: CAT MLB Table");
	for (i = MLB150_BUF_CAT_MLB_OFFSET + (ch_start >> 3);
			i < MLB150_BUF_CAT_MLB_OFFSET + (ch_end >> 3) + 1;
			++i) {
		mlb150_dev_ctr_read(i, ctr_val);
		pr_debug("CTR 0x%02x: 0x%08x, 0x%08x, 0x%08x, 0x%08x\n",
			i, ctr_val[3], ctr_val[2], ctr_val[1], ctr_val[0]);
	}

	pr_debug("mxc_mlb150: CAT HBI Table");
	for (i = MLB150_BUF_CAT_HBI_OFFSET + (ch_start >> 3);
			i < MLB150_BUF_CAT_HBI_OFFSET + (ch_end >> 3) + 1;
			++i) {
		mlb150_dev_ctr_read(i, ctr_val);
		pr_debug("CTR 0x%02x: 0x%08x, 0x%08x, 0x%08x, 0x%08x\n",
			i, ctr_val[3], ctr_val[2], ctr_val[1], ctr_val[0]);
	}
}
#endif

/*!
 * Initial the MLB module device
 */
static inline s32 mlb150_dev_enable_dma_irq(u32 enable)
{
	if (enable) {
		__raw_writel(0xffffffff, mlb_base + MLB150_REG_ACMR0);
		__raw_writel(0xffffffff, mlb_base + MLB150_REG_ACMR1);
	} else {
		__raw_writel(0x0, mlb_base + MLB150_REG_ACMR0);
		__raw_writel(0x0, mlb_base + MLB150_REG_ACMR1);
	}

	return 0;
}


static s32 mlb150_dev_init_ir_amba_ahb(void)
{
	u32 reg = 0;

	/* Step 1. Program the ACMRn registers to enable interrupts from all
	 * active DMA channels */
	mlb150_dev_enable_dma_irq(1);

	/* Step 2. Select the status clear method:
	 * ACTL.SCE = 0, hardware clears on read
	 * ACTL.SCE = 1, software writes a '1' to clear */
	/* We only support DMA MODE 1 */
	reg = __raw_readl(mlb_base + MLB150_REG_ACTL);
	reg |= MLB150_ACTL_DMAMODE;
#ifdef MLB150_MULTIPLE_PACKAGE_MODE
	reg |= MLB150_REG_ACTL_MPB;
#endif

	/* Step 3. Select 1 or 2 interrupt signals:
	 * ACTL.SMX = 0: one interrupt for channels 0 - 31 on ahb_init[0]
	 *	and another interrupt for channels 32 - 63 on ahb_init[1]
	 * ACTL.SMX = 1: singel interrupt all channels on ahb_init[0]
	 * */
	/*
	reg |= MLB150_ACTL_SMX;
	*/

	__raw_writel(reg, mlb_base + MLB150_REG_ACTL);

	return 0;
}

static inline s32 mlb150_dev_enable_ir_mlb(u32 enable)
{
	/* Step 1, Select the MSn to be cleared by software,
	 * writing a '0' to the appropriate bits */
	__raw_writel(0, mlb_base + MLB150_REG_MS0);
	__raw_writel(0, mlb_base + MLB150_REG_MS1);

	/* Step 1, Program MIEN to enable protocol error
	 * interrupts for all active MLB channels */
	if (enable)
		__raw_writel(MLB150_MIEN_CTX_PE |
			MLB150_MIEN_CRX_PE | MLB150_MIEN_ATX_PE |
			MLB150_MIEN_ARX_PE | MLB150_MIEN_SYNC_PE |
			MLB150_MIEN_ISOC_PE,
			mlb_base + MLB150_REG_MIEN);
	else
		__raw_writel(0, mlb_base + MLB150_REG_MIEN);

	return 0;
}

static inline int mlb150_enable_pll(void)
{
	u32 c0_val;

	__raw_writel(MLB150_MLBPC1_VAL,
			mlb_base + MLB150_REG_MLBPC1);

	c0_val = __raw_readl(mlb_base + MLB150_REG_MLBC0);
	if (c0_val & MLB150_MLBC0_MLBPEN) {
		c0_val &= ~MLB150_MLBC0_MLBPEN;
		__raw_writel(c0_val,
				mlb_base + MLB150_REG_MLBC0);
	}

	clk_enable(mlb_pll_clk);

	c0_val |= (MLB150_MLBC0_MLBPEN);
	__raw_writel(c0_val, mlb_base + MLB150_REG_MLBC0);

	return 0;
}

static inline int mlb150_disable_pll(void)
{
	u32 c0_val;

	clk_disable(mlb_pll_clk);

	c0_val = __raw_readl(mlb_base + MLB150_REG_MLBC0);

	__raw_writel(0x0, mlb_base + MLB150_REG_MLBPC1);

	c0_val &= ~MLB150_MLBC0_MLBPEN;
	__raw_writel(c0_val, mlb_base + MLB150_REG_MLBC0);

	return 0;
}

static void mlb150_dev_init(void)
{
	u32 c0_val, hctl_val;

	/* Disable EN bits */
	c0_val = __raw_readl(mlb_base + MLB150_REG_MLBC0);
	c0_val &= ~MLB150_MLBC0_MLBEN;
	__raw_writel(c0_val, mlb_base + MLB150_REG_MLBC0);

	hctl_val = __raw_readl(mlb_base + MLB150_REG_HCTL);
	hctl_val &= ~MLB150_HCTL_EN;
	__raw_writel(hctl_val, mlb_base + MLB150_REG_HCTL);

	/* Step 1, Configure the MediaLB interface */
	/* Select pin mode and clock, 3-pin and 256fs */
	c0_val = __raw_readl(mlb_base + MLB150_REG_MLBC0);
	c0_val &= ~(MLB150_MLBC0_MLBPEN | MLB150_MLBC0_MLBCLK_MASK);
	__raw_writel(c0_val, mlb_base + MLB150_REG_MLBC0);

	c0_val |= MLB150_MLBC0_MLBEN;
	__raw_writel(c0_val, mlb_base + MLB150_REG_MLBC0);

	/* Step 2, Configure the HBI interface */
	__raw_writel(0xffffffff, mlb_base + MLB150_REG_HCMR0);
	__raw_writel(0xffffffff, mlb_base + MLB150_REG_HCMR1);
	__raw_writel(MLB150_HCTL_EN, mlb_base + MLB150_REG_HCTL);

	mlb150_dev_init_ir_amba_ahb();

	mlb150_dev_enable_ir_mlb(1);
}

static s32 mlb150_dev_reset_cdt(void)
{
	int i = 0;
	u32 ctr_val[4] = { 0 };

	for (i = 0; i < (MLB150_LOGIC_CH_NUM); ++i)
		mlb150_dev_ctr_write(MLB150_BUF_CDT_OFFSET + i, ctr_val);

	return 0;
}

static s32 mlb150_dev_init_ch_cdt(u32 ch, enum MLB_CTYPE ctype, u32 ch_func)
{
	u32 cdt_val[4] = { 0 };

	/* a. Set the 14-bit base address (BA) */
	pr_debug("mxc_mlb150: ctype: %d, ch: %d, dbr_buf_head: 0x%08x",
		ctype, ch, mlb_devinfo[ctype].channels[ch_func].dbr_buf_head);
	cdt_val[3] = (mlb_devinfo[ctype].channels[ch_func].dbr_buf_head)
			<< CDT_BA_SHIFT;

	/* b. Set the 12-bit or 13-bit buffer depth (BD)
	 * BD = buffer depth in bytes - 1 */
	switch (ctype) {
	case MLB_CTYPE_SYNC:
		/* For synchronous channels: (BD + 1) = 4 * m * bpf */
		cdt_val[3] |= (MLB150_CH_SYNC_BUF_DEP - 1) << CDT_BD_SHIFT;
		break;
	case MLB_CTYPE_CTRL:
		/* For control channels: (BD + 1) >= max packet length (64) */
		/* BD */
		cdt_val[3] |= ((MLB150_CH_CTRL_BUF_DEP - 1) << CDT_BD_SHIFT);
		break;
	case MLB_CTYPE_ASYNC:
		/* For asynchronous channels: (BD + 1) >= max packet length
		 * 1024 for a MOST Data packet (MDP);
		 * 1536 for a MOST Ethernet Packet (MEP) */
		cdt_val[3] |= ((MLB150_CH_ASYNC_BUF_DEP - 1) << CDT_BD_SHIFT);
		break;
	case MLB_CTYPE_ISOC:
		/* For isochronous channels: (BD + 1) mod (BS + 1) = 0 */
		/* BS */
		cdt_val[1] |= (MLB150_CH_ISOC_BLK_SIZE - 1);
		/* BD */
		cdt_val[3] |= (MLB150_CH_ISOC_BUF_DEP - 1)
				<< CDT_BD_SHIFT;
		break;
	default:
		break;
	}

	pr_debug("mxc_mlb150: Set CDT val of channel %d, type: %d: "
		"0x%08x 0x%08x 0x%08x 0x%08x\n",
		ch, ctype, cdt_val[3], cdt_val[2], cdt_val[1], cdt_val[0]);

	if (unlikely(mlb150_dev_cdt_write(ch, cdt_val)))
		return -ETIME;

#ifdef DEBUG_CTR
	{
		u32 cdt_rd[4] = { 0 };
		if (likely(!mlb150_dev_cdt_read(ch, cdt_rd))) {
			pr_debug("mxc_mlb150: CDT val of channel %d: "
				"0x%08x 0x%08x 0x%08x 0x%08x\n",
				ch, cdt_rd[3], cdt_rd[2], cdt_rd[1], cdt_rd[0]);
			if (cdt_rd[3] == cdt_val[3] &&
				cdt_rd[2] == cdt_val[2] &&
				cdt_rd[1] == cdt_val[1] &&
				cdt_rd[0] == cdt_val[0]) {
				pr_debug("mxc_mlb150: set cdt succeed!\n");
				return 0;
			} else {
				pr_debug("mxc_mlb150: set cdt failed!\n");
				return -EBADE;
			}
		} else {
			pr_debug("mxc_mlb150: Read CDT val of channel %d failed\n",
					ch);
			return -EBADE;
		}
	}
#endif

	return 0;
}

static s32 mlb150_dev_init_ch_cat(u32 ch, u32 cat_mode, enum MLB_CTYPE ctype)
{
	u16 cat_val = 0;
#ifdef DEBUG_CTR
	u16 cat_rd = 0;
#endif

	cat_val = CAT_CE | (ctype << CAT_CT_SHIFT) | ch;

	if (cat_mode & MLB150_CAT_MODE_OUTBOUND_DMA)
		cat_val |= CAT_RNW;

	if (MLB_CTYPE_SYNC == ctype)
		cat_val |= CAT_MT;

	pr_debug("mxc_mlb150: set CAT val of channel %d, type: %d: 0x%04x\n",
			ch, ctype, cat_val);

	switch (cat_mode) {
	case MLB150_CAT_MODE_RX | MLB150_CAT_MODE_INBOUND_DMA:
	case MLB150_CAT_MODE_TX | MLB150_CAT_MODE_OUTBOUND_DMA:
		if (unlikely(mlb150_dev_cat_mlb_write(ch, cat_val)))
			return -ETIME;
#ifdef DEBUG_CTR
		if (likely(!mlb150_dev_cat_mlb_read(ch, &cat_rd)))
			pr_debug("mxc_mlb150: CAT val of mlb channel %d: 0x%04x",
					ch, cat_rd);
		else {
			pr_debug("mxc_mlb150: Read CAT of mlb channel %d failed\n",
					ch);
				return -EBADE;
		}
#endif
		break;
	case MLB150_CAT_MODE_TX | MLB150_CAT_MODE_INBOUND_DMA:
	case MLB150_CAT_MODE_RX | MLB150_CAT_MODE_OUTBOUND_DMA:
		if (unlikely(mlb150_dev_cat_hbi_write(ch, cat_val)))
			return -ETIME;
#ifdef DEBUG_CTR
		if (likely(!mlb150_dev_cat_hbi_read(ch, &cat_rd)))
			pr_debug("mxc_mlb150: CAT val of hbi channel %d: 0x%04x",
					ch, cat_rd);
		else {
			pr_debug("mxc_mlb150: Read CAT of hbi channel %d failed\n",
					ch);
				return -EBADE;
		}
#endif
		break;
	default:
		return EBADRQC;
	}

#ifdef DEBUG_CTR
	{
		if (cat_val == cat_rd) {
			pr_debug("mxc_mlb150: set cat succeed!\n");
			return 0;
		} else {
			pr_debug("mxc_mlb150: set cat failed!\n");
			return -EBADE;
		}
	}
#endif
	return 0;
}

static s32 mlb150_dev_reset_cat(void)
{
	int i = 0;
	u32 ctr_val[4] = { 0 };

	for (i = 0; i < (MLB150_LOGIC_CH_NUM >> 3); ++i) {
		mlb150_dev_ctr_write(MLB150_BUF_CAT_MLB_OFFSET + i, ctr_val);
		mlb150_dev_ctr_write(MLB150_BUF_CAT_HBI_OFFSET + i, ctr_val);
	}

	return 0;
}

static s32 mlb150_dev_init_rfb(u32 rx_ch, u32 tx_ch, enum MLB_CTYPE ctype)
{
	/* Step 1, Initialize all bits of CAT to '0' */
	mlb150_dev_reset_cat();
	mlb150_dev_reset_cdt();

	/* Step 2, Initialize logical channel */
	/* Step 3, Program the CDT for channel N */
	mlb150_dev_init_ch_cdt(rx_ch, ctype, RX_CHANNEL);
	mlb150_dev_init_ch_cdt(tx_ch, ctype, TX_CHANNEL);

	/* Step 4&5, Program the CAT for the inbound and outbound DMA */
	mlb150_dev_init_ch_cat(rx_ch,
			MLB150_CAT_MODE_RX | MLB150_CAT_MODE_INBOUND_DMA,
			ctype);
	mlb150_dev_init_ch_cat(rx_ch,
			MLB150_CAT_MODE_RX | MLB150_CAT_MODE_OUTBOUND_DMA,
			ctype);
	mlb150_dev_init_ch_cat(tx_ch,
			MLB150_CAT_MODE_TX | MLB150_CAT_MODE_INBOUND_DMA,
			ctype);
	mlb150_dev_init_ch_cat(tx_ch,
			MLB150_CAT_MODE_TX | MLB150_CAT_MODE_OUTBOUND_DMA,
			ctype);

	return 0;
}

static s32 mlb150_dev_reset_adt(void)
{
	int i = 0;
	u32 ctr_val[4] = { 0 };

	for (i = 0; i < (MLB150_LOGIC_CH_NUM); ++i)
		mlb150_dev_ctr_write(MLB150_BUF_ADT_OFFSET + i, ctr_val);

	return 0;
}

static inline s32 mlb150_dev_set_ch_amba_ahb(u32 ch, enum MLB_CTYPE ctype,
					u32 dne_sts, u32 buf_addr)
{
	u32 ctr_val[4] = { 0 };

	if (MLB_CTYPE_ASYNC == ctype ||
		MLB_CTYPE_CTRL == ctype) {
		ctr_val[1] |= ADT_PS1;
		ctr_val[1] |= ADT_PS2;
	}

	/* Clear DNE1 and ERR1 */
	/* Set the page ready bit (RDY1) */
	if (dne_sts & ADT_DNE1) {
		ctr_val[1] |= ADT_RDY2;
		ctr_val[3] = buf_addr;
	} else {
		ctr_val[1] |= ADT_RDY1;
		ctr_val[2] = buf_addr;
	}

#ifdef DEBUG_ADT
	pr_debug("mxc_mlb150: Set ADT val of channel %d, ctype: %d: "
		"0x%08x 0x%08x 0x%08x 0x%08x\n",
		ch, ctype, ctr_val[3], ctr_val[2], ctr_val[1], ctr_val[0]);
#endif

	if (unlikely(mlb150_dev_adt_write(ch, ctr_val)))
		return -ETIME;

#ifdef DEBUG_ADT_N
	{
		u32 ctr_rd[4] = { 0 };
		if (likely(!mlb150_dev_adt_read(ch, ctr_rd))) {
			pr_debug("mxc_mlb150: ADT val of channel %d: "
				"0x%08x 0x%08x 0x%08x 0x%08x\n",
				ch, ctr_rd[3], ctr_rd[2],
				ctr_rd[1], ctr_rd[0]);
			if (ctr_rd[3] == ctr_val[3] &&
				ctr_rd[2] == ctr_val[2] &&
				ctr_rd[1] == ctr_val[1] &&
				ctr_rd[0] == ctr_val[0]) {
				pr_debug("mxc_mlb150: set adt succeed!\n");
				return 0;
			} else {
				pr_debug("mxc_mlb150: set adt failed!\n");
				return -EBADE;
			}
		} else {
			pr_debug("mxc_mlb150: Read ADT val of channel %d failed\n",
					ch);
			return -EBADE;
		}
	}
#endif

      return 0;
}

static s32 mlb150_dev_init_ch_amba_ahb(struct mlb_channel_info *chinfo,
					enum MLB_CTYPE ctype)
{
	u32 ctr_val[4] = { 0 };

	/* a. Set the 32-bit base address (BA1) */
	ctr_val[3] = chinfo->pong_phy_head;
	ctr_val[2] = chinfo->ping_phy_head;
	ctr_val[1] = (chinfo->buf_size - 1) << ADT_BD1_SHIFT;
	ctr_val[1] |= (chinfo->buf_size - 1) << ADT_BD2_SHIFT;
	if (MLB_CTYPE_ASYNC == ctype ||
		MLB_CTYPE_CTRL == ctype) {
		ctr_val[1] |= ADT_PS1;
		ctr_val[1] |= ADT_PS2;
	}

	ctr_val[0] |= (ADT_LE | ADT_CE);

	pr_debug("mxc_mlb150: Set ADT val of channel %d, ctype: %d: "
		"0x%08x 0x%08x 0x%08x 0x%08x\n",
		chinfo->address, ctype, ctr_val[3], ctr_val[2],
		ctr_val[1], ctr_val[0]);

	if (unlikely(mlb150_dev_adt_write(chinfo->address, ctr_val)))
		return -ETIME;

#ifdef DEBUG_CTR
	{
		u32 ctr_rd[4] = { 0 };
		if (likely(!mlb150_dev_adt_read(chinfo->address, ctr_rd))) {
			pr_debug("mxc_mlb150: ADT val of channel %d: "
				"0x%08x 0x%08x 0x%08x 0x%08x\n",
				chinfo->address, ctr_rd[3], ctr_rd[2],
				ctr_rd[1], ctr_rd[0]);
			if (ctr_rd[3] == ctr_val[3] &&
				ctr_rd[2] == ctr_val[2] &&
				ctr_rd[1] == ctr_val[1] &&
				ctr_rd[0] == ctr_val[0]) {
				pr_debug("mxc_mlb150: set adt succeed!\n");
				return 0;
			} else {
				pr_debug("mxc_mlb150: set adt failed!\n");
				return -EBADE;
			}
		} else {
			pr_debug("mxc_mlb150: Read ADT val of channel %d failed\n",
					chinfo->address);
			return -EBADE;
		}
	}
#endif

	return 0;
}

static s32 mlb150_dev_init_amba_ahb(struct mlb_channel_info *rx_chinfo,
		struct mlb_channel_info *tx_chinfo, enum MLB_CTYPE ctype)
{
	/* Step 1, Initialize all bits of the ADT to '0' */
	mlb150_dev_reset_adt();

	/* Step 2, Select a logic channel */
	/* Step 3, Program the AMBA AHB block ping page for channel N */
	/* Step 4, Program the AMBA AHB block pong page for channel N */
	mlb150_dev_init_ch_amba_ahb(rx_chinfo, ctype);
	mlb150_dev_init_ch_amba_ahb(tx_chinfo, ctype);

	return 0;
}

static s32 mlb150_dev_unmute_syn_ch(u32 rx_ch, u32 tx_ch)
{
	u32 timeout = 10000;

	/* Check that MediaLB clock is running (MLBC1.CLKM = 0)
	 * If MLBC1.CLKM = 1, clear the register bit, wait one
	 * APB or I/O clock cycle and repeat the check */
	while ((__raw_readl(mlb_base + MLB150_REG_MLBC1) & MLB150_MLBC1_CLKM)
			|| timeout--)
		__raw_writel(~MLB150_MLBC1_CLKM, mlb_base + MLB150_REG_MLBC1);

	if (unlikely(0 == timeout))
		return -ETIME;

	timeout = 10000;
	/* Poll for MLB lock (MLBC0.MLBLK = 1) */
	while (!(__raw_readl(mlb_base + MLB150_REG_MLBC0) & MLB150_MLBC0_MLBLK)
			|| timeout--)
		;

	if (unlikely(0 == timeout))
		return -ETIME;

	/* Unmute synchronous channel(s) */
	mlb150_dev_cat_mlb_write(rx_ch, CAT_CE | rx_ch);
	mlb150_dev_cat_mlb_write(tx_ch,
			CAT_CE | tx_ch | CAT_RNW);
	mlb150_dev_cat_hbi_write(rx_ch,
			CAT_CE | rx_ch | CAT_RNW);
	mlb150_dev_cat_hbi_write(tx_ch, CAT_CE | tx_ch);

	return 0;
}

static void mlb150_dev_exit(void)
{
	mlb150_dev_enable_dma_irq(0);
	mlb150_dev_enable_ir_mlb(0);

	__raw_writel(0, mlb_base + MLB150_REG_HCTL);
	__raw_writel(0, mlb_base + MLB150_REG_MLBC0);
}

/*!
 * MLB receive start function
 *
 * load phy_head to next buf register to start next rx
 * here use single-packet buffer, set start=end
 */
static inline void mlb_start_rx(u32 ch, s32 ctype, u32 dne_sts, u32 buf_addr)
{
	/*  Set ADT for RX */
	mlb150_dev_set_ch_amba_ahb(ch, ctype, dne_sts, buf_addr);
}

/*!
 * MLB transmit start function
 * make sure aquiring the rw buf_lock, when calling this
 */
static inline void mlb_start_tx(u32 ch, s32 ctype, u32 dne_sts, u32 buf_addr)
{
	/*  Set ADT for TX */
	mlb150_dev_set_ch_amba_ahb(ch, ctype, dne_sts, buf_addr);
}

/*!
 * Enable the MLB channel
 */
static void mlb_channel_enable(int chan_dev_id, int on)
{
	struct mlb_dev_info *pdevinfo = &mlb_devinfo[chan_dev_id];
	/*!
	 * setup the direction, enable, channel type,
	 * mode select, channel address and mask buf start
	 */
	if (on) {
		u32 ctype = pdevinfo->channel_type;
		struct mlb_channel_info *tx_chinfo = &_get_txchan(chan_dev_id);
		struct mlb_channel_info *rx_chinfo = &_get_rxchan(chan_dev_id);
		u32 tx_ch = tx_chinfo->address;
		u32 rx_ch = rx_chinfo->address;

		mlb150_dev_enable_ctr_write(0xffffffff, 0xffffffff,
				0xffffffff, 0xffffffff);
		mlb150_dev_init_rfb(rx_ch, tx_ch, ctype);

		mlb150_dev_init_amba_ahb(rx_chinfo, tx_chinfo, ctype);

		/* Synchronize and unmute synchrouous channel */
		if (MLB_CTYPE_SYNC == ctype)
			mlb150_dev_unmute_syn_ch(rx_ch, tx_ch);

		mlb150_dev_enable_ctr_write(0x0, ADT_RDY1 | ADT_DNE1 |
				ADT_ERR1 | ADT_PS1 |
				ADT_MEP1 | ADT_RDY2 | ADT_DNE2 | ADT_ERR2 |
				ADT_PS2 | ADT_MEP2,
				0xffffffff, 0xffffffff);

		if (pdevinfo->fps >= MLB150_CLK_2048FS)
			mlb150_enable_pll();

		atomic_set(&pdevinfo->on, 1);

#ifdef DEBUG
		mlb150_dev_dump_reg();
		mlb150_dev_dump_ctr_tbl(0, tx_chinfo->address + 1);
#endif
		mlb_start_rx(rx_ch, ctype, ADT_DNE2,
				pdevinfo->rx_bufs.phy_addrs[0]);
	} else {
		mlb150_dev_enable_dma_irq(0);
		mlb150_dev_enable_ir_mlb(0);

		mlb150_dev_reset_cat();

		atomic_set(&mlb_devinfo[chan_dev_id].on, 0);

		if (mlb_devinfo[chan_dev_id].fps >= MLB150_CLK_2048FS)
			mlb150_disable_pll();
	}
}

/*!
 * MLB interrupt handler
 */
static void mlb_tx_isr(int minor)
{
	struct mlb_dev_info *pdevinfo = &mlb_devinfo[minor];

	pdevinfo->tx_busy = 0;

	wake_up_interruptible(&pdevinfo->wt_wq);
}

static void mlb_rx_isr(int minor)
{
	struct mlb_dev_info *pdevinfo = &mlb_devinfo[minor];
	struct mlb_channel_info *pchinfo = &_get_rxchan(minor);
	struct mlb_ringbuf *rx_rbuf = &pdevinfo->rx_bufs;
	s32 wpos, rpos, adt_sts;
	u32 rx_ring_buf = 0;
	s32 ctype = pdevinfo->channel_type;
	u32 ch_addr = pchinfo->address;

#ifdef DEBUG_RX
	pr_debug("mxc_mlb150: mlb_rx_isr\n");
#endif

	rpos = rx_rbuf->rpos;
	wpos = rx_rbuf->wpos;

#ifdef DEBUG_RX
	pr_debug("adt_buf_ptr: 0x%08x\n", (u32)adt_buf_ptr);
#endif

	/*!
	 * Copy packet from IRAM buf to ring buf.
	 * if the wpos++ == rpos, drop this packet
	 */
	if (((wpos + 1) % TRANS_RING_NODES) != rpos) {
		rx_ring_buf = rx_rbuf->phy_addrs[(wpos + 1) % TRANS_RING_NODES];
#ifdef DEBUG_RX
		if (len > mlb150_ch_packet_buf_size[ctype])
			pr_debug("mxc_mlb150: packet overflow, "
				"packet type: %d\n", ctype);
#endif

		/* update the ring wpos */
		rx_rbuf->wpos = (wpos + 1) % TRANS_RING_NODES;

		/* wake up the reader */
		wake_up_interruptible(&pdevinfo->rd_wq);

#ifdef DEBUG_RX
		pr_debug("recv package, len:%d, rx_rdpos: %d, rx_wtpos: %d\n",
			 len, rpos, pdevinfo->rx_bufs.wpos);
#endif
	} else {
		rx_ring_buf = pdevinfo->rx_bufs.phy_addrs[TRANS_RING_NODES];

		pr_debug
		    ("drop package, due to no space, (%d,%d)\n",
		     rpos, pdevinfo->rx_bufs.wpos);
	}

	adt_sts = mlb150_dev_get_adt_sts(ch_addr);
	mlb_start_rx(ch_addr, ctype, adt_sts, rx_ring_buf);
}

static irqreturn_t mlb_ahb_isr(int irq, void *dev_id)
{
	u32 rx_int_sts, tx_int_sts, acsr0,
		acsr1, rx_err, tx_err, hcer0, hcer1;
	struct mlb_dev_info *pdev = NULL;
	struct mlb_channel_info *ptxchinfo = NULL, *prxchinfo = NULL;
	int minor;

	/* Step 5, Read the ACSRn registers to determine which channel or
	 * channels are causing the interrupt */
	acsr0 = __raw_readl(mlb_base + MLB150_REG_ACSR0);
	acsr1 = __raw_readl(mlb_base + MLB150_REG_ACSR1);

	hcer0 = __raw_readl(mlb_base + MLB150_REG_HCER0);
	hcer1 = __raw_readl(mlb_base + MLB150_REG_HCER1);

	/* Step 6, If ACTL.SCE = 1, write the result of step 5 back to ACSR0
	 * and ACSR1 to clear the interrupt */
	if (MLB150_ACTL_SCE & __raw_readl(mlb_base + MLB150_REG_ACTL)) {
		__raw_writel(acsr0, mlb_base + MLB150_REG_ACSR0);
		__raw_writel(acsr1, mlb_base + MLB150_REG_ACSR1);
	}

	for (minor = 0; minor < MLB_MINOR_DEVICES; minor++) {
		pdev = &mlb_devinfo[minor];
		prxchinfo = &_get_rxchan(minor);
		ptxchinfo = &_get_txchan(minor);

		rx_int_sts = (prxchinfo->address < 31) ? acsr0 : acsr1;
		tx_int_sts = (ptxchinfo->address < 31) ? acsr0 : acsr1;
		rx_err = (prxchinfo->address < 31) ? hcer0 : hcer1;
		tx_err = (ptxchinfo->address < 31) ? hcer0 : hcer1;

		/* get tx channel interrupt status */
		if (tx_int_sts & (1 << (ptxchinfo->address % 32))) {
			if (!(tx_err & (1 << (ptxchinfo->address % 32))))
				mlb_tx_isr(minor);
			else {
				pr_debug("tx channel %d encountered an AHB error!\n",
					ptxchinfo->address);
			}
		}

		/* get rx channel interrupt status */
		if (rx_int_sts & (1 << (prxchinfo->address % 32))) {
			if (!(rx_err & (1 << (prxchinfo->address % 32))))
				mlb_rx_isr(minor);
			else {
				pr_debug("rx channel %d encountered an AHB error!\n",
					prxchinfo->address);
			}
		}
	}

	return IRQ_HANDLED;
}

static irqreturn_t mlb_isr(int irq, void *dev_id)
{
	u32 rx_int_sts, tx_int_sts, ms0,
		ms1, tx_cis, rx_cis, ctype;
	struct mlb_dev_info *pdev;
	int minor;
	u32 cdt_val[4] = { 0 };

	/* Step 4, Read the MSn register to determine which channel(s)
	 * are causing the interrupt */
	ms0 = __raw_readl(mlb_base + MLB150_REG_MS0);
	ms1 = __raw_readl(mlb_base + MLB150_REG_MS1);
	pr_debug("mxc_mlb150: mlb interrupt:0x%08x 0x%08x\n",
			(u32)ms0, (u32)ms1);

	for (minor = 0; minor < MLB_MINOR_DEVICES; minor++) {
		pdev = &mlb_devinfo[minor];
		tx_cis = rx_cis = 0;

		ctype = pdev->channel_type;
		rx_int_sts = (_get_rxchan(minor).address < 31) ? ms0 : ms1;
		tx_int_sts = (_get_txchan(minor).address < 31) ? ms0 : ms1;

		pr_debug("mxc_mlb150: channel interrupt: "
				"tx: 0x%08x, rx: 0x%08x\n",
			(u32)tx_int_sts, (u32)rx_int_sts);

		/* Get tx channel interrupt status */
		if (tx_int_sts & (1 << (_get_txchan(minor).address % 32))) {
			mlb150_dev_cdt_read(_get_txchan(minor).address,
					cdt_val);
			pr_debug("mxc_mlb150: cdt_val[3]: 0x%08x, "
					"cdt_val[2]: 0x%08x, "
					"cdt_val[1]: 0x%08x, "
					"cdt_val[0]: 0x%08x\n",
					cdt_val[3], cdt_val[2],
					cdt_val[1], cdt_val[0]);
			switch (ctype) {
			case MLB_CTYPE_SYNC:
				tx_cis = (cdt_val[2] & ~CDT_SYNC_WSTS_MASK)
					>> CDT_SYNC_WSTS_SHIFT;
				/* Clear RSTS/WSTS errors to resume
				 * channel operation */
				/* a. For synchronous channels: WSTS[3] = 0 */
				cdt_val[2] &= ~(0x8 << CDT_SYNC_WSTS_SHIFT);
				break;
			case MLB_CTYPE_CTRL:
			case MLB_CTYPE_ASYNC:
				tx_cis = (cdt_val[2] &
					~CDT_CTRL_ASYNC_WSTS_MASK)
					>> CDT_CTRL_ASYNC_WSTS_SHIFT;
				tx_cis = (cdt_val[3] & CDT_CTRL_ASYNC_WSTS_1) ?
					(tx_cis | (0x1 << 4)) : tx_cis;
				/* b. For async and ctrl channels:
				 * RSTS[4]/WSTS[4] = 0
				 * and RSTS[2]/WSTS[2] = 0 */
				cdt_val[3] &= ~CDT_CTRL_ASYNC_WSTS_1;
				cdt_val[2] &=
					~(0x4 << CDT_CTRL_ASYNC_WSTS_SHIFT);
				break;
			case MLB_CTYPE_ISOC:
				tx_cis = (cdt_val[2] & ~CDT_ISOC_WSTS_MASK)
					>> CDT_ISOC_WSTS_SHIFT;
				/* c. For isoc channels: WSTS[2:1] = 0x00 */
				cdt_val[2] &= ~(0x6 << CDT_ISOC_WSTS_SHIFT);
				break;
			default:
				break;
			}
			mlb150_dev_cdt_write(_get_txchan(minor).address,
					cdt_val);
		}

		/* Get rx channel interrupt status */
		if (rx_int_sts & (1 << (_get_rxchan(minor).address % 32))) {
			mlb150_dev_cdt_read(_get_rxchan(minor).address,
					cdt_val);
			switch (ctype) {
			case MLB_CTYPE_SYNC:
				tx_cis = (cdt_val[2] & ~CDT_SYNC_RSTS_MASK)
					>> CDT_SYNC_RSTS_SHIFT;
				cdt_val[2] &= ~(0x8 << CDT_SYNC_WSTS_SHIFT);
				break;
			case MLB_CTYPE_CTRL:
			case MLB_CTYPE_ASYNC:
				tx_cis =
					(cdt_val[2] & ~CDT_CTRL_ASYNC_RSTS_MASK)
					>> CDT_CTRL_ASYNC_RSTS_SHIFT;
				tx_cis = (cdt_val[3] & CDT_CTRL_ASYNC_RSTS_1) ?
					(tx_cis | (0x1 << 4)) : tx_cis;
				cdt_val[3] &= ~CDT_CTRL_ASYNC_RSTS_1;
				cdt_val[2] &=
					~(0x4 << CDT_CTRL_ASYNC_RSTS_SHIFT);
				break;
			case MLB_CTYPE_ISOC:
				tx_cis = (cdt_val[2] & ~CDT_ISOC_RSTS_MASK)
					>> CDT_ISOC_RSTS_SHIFT;
				cdt_val[2] &= ~(0x6 << CDT_ISOC_WSTS_SHIFT);
				break;
			default:
				break;
			}
			mlb150_dev_cdt_write(_get_rxchan(minor).address,
					cdt_val);
		}

		if (!tx_cis && !rx_cis)
			continue;

		/* fill exception event */
		spin_lock(&pdev->event_lock);
		pdev->ex_event |= (rx_cis << 16) | tx_cis;
		spin_unlock(&pdev->event_lock);
	}

	return IRQ_HANDLED;
}

static int mxc_mlb150_open(struct inode *inode, struct file *filp)
{
	int minor, ring_buf_size, buf_size, j, ret;
	void __iomem *buf_addr;
	ulong phyaddr;
	struct mxc_mlb_platform_data *plat_data;
	struct mlb_dev_info *pdevinfo = NULL;
	struct mlb_channel_info *pchinfo = NULL;

	plat_data = container_of(inode->i_cdev, struct mxc_mlb_platform_data,
				cdev);
	filp->private_data = plat_data;

	minor = MINOR(inode->i_rdev);

	if (unlikely(minor < 0 || minor >= MLB_MINOR_DEVICES))
		return -ENODEV;

	/* open for each channel device */
	if (unlikely(atomic_cmpxchg(&mlb_devinfo[minor].opencnt, 0, 1) != 0))
		return -EBUSY;

	pdevinfo = &mlb_devinfo[minor];
	pchinfo = &_get_txchan(minor);

	ring_buf_size = mlb150_ch_packet_buf_size[minor];
	buf_size = ring_buf_size * (TRANS_RING_NODES + 1) + PING_BUF_MAX_SIZE;
	buf_addr = iram_alloc(buf_size, &phyaddr);
	memset(buf_addr, 0, buf_size);
	if (unlikely(buf_addr == NULL)) {
		ret = -ENOMEM;
		dev_err(plat_data->dev, "can not alloc rx buffers\n");
		return ret;
	}

	dev_dbg(plat_data->dev, "ch_type: %d, RX ring buf virt base: 0x%08x "
			"phy base: 0x%08x\n",
			pdevinfo->channel_type, (u32)buf_addr, (u32)phyaddr);

	for (j = 0; j < TRANS_RING_NODES + 1;
		++j, buf_addr += ring_buf_size, phyaddr += ring_buf_size) {
		pdevinfo->rx_bufs.virt_bufs[j] = buf_addr;
		pdevinfo->rx_bufs.phy_addrs[j] = phyaddr;
		pdevinfo->rx_bufs.size = pchinfo->buf_size;
	}

	/* set the virtual and physical buf head address */
	pchinfo->ping_buf_head = pchinfo->pong_buf_head = (u32)buf_addr;
	pchinfo->ping_phy_head = pchinfo->pong_phy_head = phyaddr;

	pchinfo->buf_ptr = (u32)buf_addr;
	pchinfo->buf_phy_addr = phyaddr;

	dev_dbg(plat_data->dev, "ctype: %d, tx phy_head: 0x%08x, "
		"buf_head: 0x%08x\n",
		pchinfo->address,
		(u32)pchinfo->buf_phy_addr,
		(u32)pchinfo->buf_ptr);

	/* reset the buffer read/write ptr */
	pdevinfo->rx_bufs.rpos = pdevinfo->rx_bufs.wpos = 0;
	pdevinfo->ex_event = 0;

	return 0;
}

static int mxc_mlb150_release(struct inode *inode, struct file *filp)
{
	int minor;
	u32 buf_size;

	minor = MINOR(inode->i_rdev);

#ifdef DEBUG
	mlb150_dev_dump_reg();
	mlb150_dev_dump_ctr_tbl(0, _get_txchan(minor).address + 1);
	mlb150_dev_dump_hex((const u8 *)mlb_devinfo[minor].rx_bufs.virt_bufs[0],
			mlb_devinfo[minor].rx_bufs.size);
#endif

	/* clear channel settings and info */
	mlb_channel_enable(minor, 0);

	buf_size = mlb150_ch_packet_buf_size[minor] *
			(TRANS_RING_NODES + 1) + PING_BUF_MAX_SIZE;
	iram_free(mlb_devinfo[minor].rx_bufs.phy_addrs[0], buf_size);

	/* decrease the open count */
	atomic_set(&mlb_devinfo[minor].opencnt, 0);

	return 0;
}

static long mxc_mlb150_ioctl(struct file *filp,
			 unsigned int cmd, unsigned long arg)
{
	struct inode *inode = filp->f_dentry->d_inode;
	void __user *argp = (void __user *)arg;
	unsigned long flags, event;
	int minor;
	struct mxc_mlb_platform_data *plat_data = filp->private_data;

	minor = MINOR(inode->i_rdev);

	switch (cmd) {
	case MLB_CHAN_SETADDR:
		{
			unsigned int caddr;
			/* get channel address from user space */
			if (copy_from_user(&caddr, argp, sizeof(caddr))) {
				pr_err("mxc_mlb150: copy from user failed\n");
				return -EFAULT;
			}
			_get_txchan(minor).address = (caddr >> 16) & 0xFFFF;
			_get_rxchan(minor).address = caddr & 0xFFFF;
			pr_debug("mxc_mlb150: set ch addr, tx: %d, rx: %d\n",
					_get_txchan(minor).address,
					_get_rxchan(minor).address);
			break;
		}

	case MLB_CHAN_STARTUP:
		if (unlikely(atomic_read(&mlb_devinfo[minor].on))) {
			pr_debug("mxc_mlb150: channel areadly startup\n");
			break;
		}
		pr_debug("mxc_mlb150: start channel\n");
		mlb_channel_enable(minor, 1);
		break;
	case MLB_CHAN_SHUTDOWN:
		if (unlikely(atomic_read(&mlb_devinfo[minor].on) == 0)) {
			pr_debug("mxc_mlb150: channel areadly shutdown\n");
			break;
		}
		pr_debug("mxc_mlb150: shutdown channel\n");
		mlb_channel_enable(minor, 0);
		break;
	case MLB_CHAN_GETEVENT:
		/* get and clear the ex_event */
		spin_lock_irqsave(&mlb_devinfo[minor].event_lock, flags);
		event = mlb_devinfo[minor].ex_event;
		mlb_devinfo[minor].ex_event = 0;
		spin_unlock_irqrestore(&mlb_devinfo[minor].event_lock, flags);

		pr_debug("mxc_mlb150: get event\n");
		if (event) {
			if (copy_to_user(argp, &event, sizeof(event))) {
				pr_err("mxc_mlb150: copy to user failed\n");
				return -EFAULT;
			}
		} else {
			pr_debug("mxc_mlb150: no exception event now\n");
			return -EAGAIN;
		}
		break;
	case MLB_SET_FPS:
		{
			u32 fps, c0_val;

			/* get fps from user space */
			if (unlikely(copy_from_user(&fps, argp, sizeof(fps)))) {
				pr_err("mxc_mlb150: copy from user failed\n");
				return -EFAULT;
			}

			if (plat_data->fps_sel)
				plat_data->fps_sel(fps);

			c0_val = __raw_readl(mlb_base + MLB150_REG_MLBC0);
			c0_val &= ~MLB150_MLBC0_MLBCLK_MASK;

			/* check fps value */
			switch (fps) {
			case 256:
			case 512:
			case 1024:
				mlb_devinfo[minor].fps = fps >> 9;
				c0_val &= ~MLB150_MLBC0_MLBPEN;
				c0_val |= (fps >> 9)
					<< MLB150_MLBC0_MLBCLK_SHIFT;
				break;
			case 2048:
			case 3072:
			case 4096:
				mlb_devinfo[minor].fps = (fps >> 10) + 1;
				c0_val |= ((fps >> 10) + 1)
					<< MLB150_MLBC0_MLBCLK_SHIFT;
				break;
			case 6144:
				mlb_devinfo[minor].fps = fps >> 10;
				c0_val |= ((fps >> 10) + 1)
					<< MLB150_MLBC0_MLBCLK_SHIFT;
				break;
			case 8192:
				mlb_devinfo[minor].fps = (fps >> 10) - 1;
				c0_val |= ((fps >> 10) - 1)
						<< MLB150_MLBC0_MLBCLK_SHIFT;
				break;
			default:
				pr_debug("mxc_mlb150: invalid fps argument: %d\n",
						fps);
				return -EINVAL;
			}

			__raw_writel(c0_val, mlb_base + MLB150_REG_MLBC0);

			pr_debug("mxc_mlb150: set fps to %d, MLBC0: 0x%08x\n",
				fps,
				(u32)__raw_readl(mlb_base + MLB150_REG_MLBC0));

			break;
		}

	case MLB_GET_VER:
		{
			u32 version;

			/* get MLB device module version */
			version = 0x03030003;

			pr_debug("mxc_mlb150: get version: 0x%08x\n",
					version);

			if (copy_to_user(argp, &version, sizeof(version))) {
				pr_err("mxc_mlb150: copy to user failed\n");
				return -EFAULT;
			}
			break;
		}

	case MLB_SET_DEVADDR:
		{
			u32 c1_val;
			u8 devaddr;

			/* get MLB device address from user space */
			if (unlikely(copy_from_user
				(&devaddr, argp, sizeof(unsigned char)))) {
				pr_err("mxc_mlb150: copy from user failed\n");
				return -EFAULT;
			}

			c1_val = __raw_readl(mlb_base + MLB150_REG_MLBC1);
			c1_val &= ~MLB150_MLBC1_NDA_MASK;
			c1_val |= devaddr << MLB150_MLBC1_NDA_SHIFT;
			__raw_writel(c1_val, mlb_base + MLB150_REG_MLBC1);
			pr_debug("mxc_mlb150: set dev addr, dev addr: %d, "
				"MLBC1: 0x%08x\n", devaddr,
				(u32)__raw_readl(mlb_base + MLB150_REG_MLBC1));

			break;
		}
	default:
		pr_info("mxc_mlb150: Invalid ioctl command\n");
		return -EINVAL;
	}

	return 0;
}

/*!
 * MLB read routine
 *
 * Read the current received data from queued buffer,
 * and free this buffer for hw to fill ingress data.
 */
static ssize_t mxc_mlb150_read(struct file *filp, char __user *buf,
			    size_t count, loff_t *f_pos)
{
	int minor, ret;
	int size, rdpos;
	struct mlb_ringbuf *rx_rbuf = NULL;
	struct mlb_dev_info *pdevinfo = NULL;

#ifdef DEBUG_RX
	pr_debug("mxc_mlb150: mxc_mlb150_read\n");
#endif

	minor = MINOR(filp->f_dentry->d_inode->i_rdev);

	pdevinfo = &mlb_devinfo[minor];

	rdpos = pdevinfo->rx_bufs.rpos;
	rx_rbuf = &pdevinfo->rx_bufs;

	/* check the current rx buffer is available or not */
	if (rdpos == rx_rbuf->wpos) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		/* if !O_NONBLOCK, we wait for recv packet */
		ret = wait_event_interruptible(pdevinfo->rd_wq,
						(rx_rbuf->wpos != rdpos));
		if (ret < 0)
			return ret;
	}

	size = mlb150_ch_packet_buf_size[minor];
	if (unlikely(size > count)) {
		/* the user buffer is too small */
		pr_warning
			("mxc_mlb150: received data size is bigger than "
			"size: %d, count: %d\n", size, count);
		return -EINVAL;
	}

	/* copy rx buffer data to user buffer */
	if (likely(copy_to_user(buf, rx_rbuf->virt_bufs[rdpos], size))) {
		pr_err("mxc_mlb150: copy from user failed\n");
		return -EFAULT;
	}

	/* update the read ptr */
	rx_rbuf->rpos = (rdpos + 1) % TRANS_RING_NODES;

	*f_pos = 0;

	return size;
}

/*!
 * MLB write routine
 *
 * Copy the user data to tx channel buffer,
 * and prepare the channel current/next buffer ptr.
 */
static ssize_t mxc_mlb150_write(struct file *filp, const char __user *buf,
			     size_t count, loff_t *f_pos)
{
	s32 minor = 0, ret = 0;
	struct mlb_channel_info *pchinfo = NULL;
	struct mlb_dev_info *pdevinfo = NULL;
	u32 adt_sts = 0;

	minor = MINOR(filp->f_dentry->d_inode->i_rdev);
	pchinfo = &_get_txchan(minor);
	pdevinfo = &mlb_devinfo[minor];

	if (unlikely(count > pchinfo->buf_size)) {
		/* too many data to write */
		pr_warning("mxc_mlb150: overflow write data\n");
		return -EFBIG;
	}

	*f_pos = 0;

	/* check the current tx buffer is used or not */
	if (1 == pdevinfo->tx_busy) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		ret = wait_event_interruptible(pdevinfo->wt_wq,
				0 == pdevinfo->tx_busy);

		if (ret < 0)
			goto out;
	}

	if (copy_from_user((void *)pchinfo->buf_ptr, buf, count)) {
		pr_err("mxc_mlb: copy from user failed\n");
		ret = -EFAULT;
		goto out;
	}

	adt_sts = mlb150_dev_get_adt_sts(pchinfo->address);
	pdevinfo->tx_busy = 1;
	mlb_start_tx(pchinfo->address, pdevinfo->channel_type,
			adt_sts, pchinfo->buf_phy_addr);

	ret = count;

out:
	return ret;
}

static unsigned int mxc_mlb150_poll(struct file *filp,
				 struct poll_table_struct *wait)
{
	int minor;
	unsigned int ret = 0;
	struct mlb_dev_info *pdevinfo = NULL;

	minor = MINOR(filp->f_dentry->d_inode->i_rdev);

	pdevinfo = &mlb_devinfo[minor];

	poll_wait(filp, &pdevinfo->rd_wq, wait);
	poll_wait(filp, &pdevinfo->wt_wq, wait);

	/* check the tx buffer is avaiable or not */
	if (0 == pdevinfo->tx_busy)
		ret |= POLLOUT | POLLWRNORM;

	/* check the rx buffer filled or not */
	if (pdevinfo->rx_bufs.rpos != pdevinfo->rx_bufs.wpos)
		ret |= POLLIN | POLLRDNORM;

	/* check the exception event */
	if (pdevinfo->ex_event)
		ret |= POLLIN | POLLRDNORM;

	return ret;
}

/*!
 * char dev file operations structure
 */
static const struct file_operations mxc_mlb150_fops = {

	.owner = THIS_MODULE,
	.open = mxc_mlb150_open,
	.release = mxc_mlb150_release,
	.unlocked_ioctl = mxc_mlb150_ioctl,
	.poll = mxc_mlb150_poll,
	.read = mxc_mlb150_read,
	.write = mxc_mlb150_write,
};

/*!
 * This function is called whenever the MLB device is detected.
 */
static int __devinit mxc_mlb150_probe(struct platform_device *pdev)
{
	int ret, mlb_major, i;
	struct mxc_mlb_platform_data *plat_data;
	struct resource *res;
	void __iomem *base;

	plat_data =
		(struct mxc_mlb_platform_data *)pdev->dev.platform_data;
	plat_data->dev = &pdev->dev;

	/**
	 * Register MLB lld as four character devices
	 */
	ret = alloc_chrdev_region(&dev, 0, MLB_MINOR_DEVICES, "mxc_mlb150");
	mlb_major = MAJOR(dev);
	dev_dbg(plat_data->dev, "MLB device major: %d\n", mlb_major);

	if (unlikely(ret < 0)) {
		dev_err(plat_data->dev, "can't get major %d\n", mlb_major);
		goto err2;
	}

	cdev_init(&plat_data->cdev, &mxc_mlb150_fops);
	plat_data->cdev.owner = THIS_MODULE;

	ret = cdev_add(&plat_data->cdev, dev, MLB_MINOR_DEVICES);
	if (unlikely(ret)) {
		dev_err(plat_data->dev, "can't add cdev\n");
		goto err2;
	}

	/* create class and device for udev information */
	mlb_class = class_create(THIS_MODULE, "mlb150");
	if (unlikely(IS_ERR(mlb_class))) {
		dev_err(plat_data->dev, "failed to create mlb150 class\n");
		ret = -ENOMEM;
		goto err2;
	}

	for (i = 0; i < MLB_MINOR_DEVICES; i++) {
		class_dev = device_create(mlb_class, NULL, MKDEV(mlb_major, i),
					  NULL, mlb_devinfo[i].dev_name);
		if (unlikely(IS_ERR(class_dev))) {
			dev_err(plat_data->dev, "failed to create mlb150 %s"
				" class device\n", mlb_devinfo[i].dev_name);
			ret = -ENOMEM;
			goto err1;
		}
	}

	/* get irq line */
	/* AHB0 IRQ */
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
	if (unlikely(res == NULL)) {
		dev_err(plat_data->dev, "No mlb150 ahb0 irq line provided\n");
		goto err0;
	}

	ahb0_irq = res->start;
	dev_dbg(plat_data->dev, "ahb0_irq: %d\n", ahb0_irq);
	if (request_irq(ahb0_irq, mlb_ahb_isr, 0, "mlb_ahb0", NULL)) {
		dev_err(plat_data->dev, "failed to request irq\n");
		ret = -EBUSY;
		goto err0;
	}

	/* AHB1 IRQ */
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 2);
	if (unlikely(res == NULL)) {
		dev_err(plat_data->dev, "No mlb150 ahb0 irq line provided\n");
		goto err0;
	}

	ahb1_irq = res->start;
	dev_dbg(plat_data->dev, "ahb1_irq: %d\n", ahb1_irq);
	if (request_irq(ahb1_irq, mlb_ahb_isr, 0, "mlb_ahb1", NULL)) {
		dev_err(plat_data->dev, "failed to request irq\n");
		ret = -EBUSY;
		goto err0;
	}

	/* MLB IRQ */
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		dev_err(plat_data->dev, "No mlb150 irq line provided\n");
		goto err0;
	}

	mlb_irq = res->start;
	dev_dbg(plat_data->dev, "mlb_irq: %d\n", mlb_irq);
	if (request_irq(mlb_irq, mlb_isr, 0, "mlb", NULL)) {
		dev_err(plat_data->dev, "failed to request irq\n");
		ret = -EBUSY;
		goto err0;
	}

	/* ioremap from phy mlb to kernel space */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(res == NULL)) {
		dev_err(plat_data->dev, "No mlb150 base address provided\n");
		goto err0;
	}

	base = ioremap(res->start, res->end - res->start);
	dev_dbg(plat_data->dev, "mapped mlb150 base address: 0x%08x\n",
		(u32)base);

	if (unlikely(base == NULL)) {
		dev_err(plat_data->dev,
				"failed to do ioremap with mlb150 base\n");
		goto err0;
	}
	mlb_base = (u32)base;

	dev_dbg(plat_data->dev, "mlb reg base: 0x%08x\n", mlb_base);

	if (plat_data->reg_nvcc) {
		/* power on MLB */
		reg_nvcc = regulator_get(plat_data->dev, plat_data->reg_nvcc);
		if (unlikely(!IS_ERR(reg_nvcc))) {
			/* set MAX LDO6 for NVCC to 2.5V */
			regulator_set_voltage(reg_nvcc, 2500000, 2500000);
			regulator_enable(reg_nvcc);
		}
	}

	/* enable clock */
	if (likely(plat_data->mlb_clk)) {
		mlb_clk = clk_get(plat_data->dev, plat_data->mlb_clk);
		if (unlikely(IS_ERR(mlb_clk))) {
			dev_err(&pdev->dev, "unable to get mlb clock\n");
			ret = PTR_ERR(mlb_clk);
			goto err0;
		}
		clk_enable(mlb_clk);
	}

	if (likely(plat_data->mlb_pll_clk)) {
		mlb_pll_clk = clk_get(plat_data->dev, plat_data->mlb_pll_clk);
		if (unlikely(IS_ERR(mlb_pll_clk))) {
			dev_err(&pdev->dev, "unable to get mlb pll clock\n");
			ret = PTR_ERR(mlb_pll_clk);
			goto err0;
		}
	}

	/* initial MLB module */
	mlb150_dev_init();

	return 0;

err0:
	if (likely(ahb0_irq)) {
		free_irq(ahb0_irq, NULL);
		ahb0_irq = 0;
	}
	if (likely(ahb1_irq)) {
		free_irq(ahb1_irq, NULL);
		ahb1_irq = 0;
	}
	if (likely(mlb_irq)) {
		free_irq(mlb_irq, NULL);
		mlb_irq = 0;
	}
err1:
	for (--i; i >= 0; i--)
		device_destroy(mlb_class, MKDEV(mlb_major, i));

	class_destroy(mlb_class);
err2:
	unregister_chrdev_region(dev, MLB_MINOR_DEVICES);

	return ret;
}

static int __devexit mxc_mlb150_remove(struct platform_device *pdev)
{
	int i;
	struct mxc_mlb_platform_data *plat_data;

	plat_data = (struct mxc_mlb_platform_data *)pdev->dev.platform_data;

	mlb150_dev_exit();

	/* disable mlb clock */
	if (plat_data->mlb_clk) {
		clk_disable(mlb_clk);
		clk_put(mlb_clk);
	}

	if (plat_data->mlb_pll_clk)
		clk_put(mlb_pll_clk);

	/* disable mlb power */
	if (plat_data->reg_nvcc) {
		regulator_disable(reg_nvcc);
		regulator_put(reg_nvcc);
	}

	/* inactive GPIO */
	gpio_mlb_inactive();

	/* iounmap */
	if (mlb_base) {
		iounmap((void *)mlb_base);
		mlb_base = 0;
	}

	if (ahb0_irq)
		free_irq(ahb0_irq, NULL);
	if (ahb1_irq)
		free_irq(ahb1_irq, NULL);
	if (mlb_irq)
		free_irq(mlb_irq, NULL);
	ahb0_irq = ahb1_irq = mlb_irq = 0;

	/* destroy mlb device class */
	for (i = MLB_MINOR_DEVICES - 1; i >= 0; i--)
		device_destroy(mlb_class, MKDEV(MAJOR(dev), i));
	class_destroy(mlb_class);

	/* Unregister the two MLB devices */
	unregister_chrdev_region(dev, MLB_MINOR_DEVICES);

	return 0;
}

#ifdef CONFIG_PM
static int mxc_mlb150_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int mxc_mlb150_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define mxc_mlb150_suspend NULL
#define mxc_mlb150_resume NULL
#endif

/*!
 * platform driver structure for MLB
 */
static struct platform_driver mxc_mlb150_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner  = THIS_MODULE,
	},
	.probe = mxc_mlb150_probe,
	.remove = __devexit_p(mxc_mlb150_remove),
	.suspend = mxc_mlb150_suspend,
	.resume = mxc_mlb150_resume,
};

static int __init mxc_mlb150_init(void)
{
	return platform_driver_register(&mxc_mlb150_driver);
}

static void __exit mxc_mlb150_exit(void)
{
	platform_driver_unregister(&mxc_mlb150_driver);
}

module_init(mxc_mlb150_init);
module_exit(mxc_mlb150_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MLB150 low level driver");
MODULE_LICENSE("GPL");
