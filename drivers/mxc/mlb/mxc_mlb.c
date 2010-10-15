/*
 * linux/drivers/mxc/mlb/mxc_mlb.c
 *
 * Copyright (C) 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
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
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/mxc_mlb.h>
#include <linux/uaccess.h>
#include <linux/iram_alloc.h>
#include <linux/fsl_devices.h>

/*!
 * MLB module memory map registers define
 */
#define MLB_REG_DCCR	0x0
#define MLB_REG_SSCR	0x4
#define MLB_REG_SDCR	0x8
#define MLB_REG_SMCR	0xC
#define MLB_REG_VCCR	0x1C
#define MLB_REG_SBCR	0x20
#define MLB_REG_ABCR	0x24
#define MLB_REG_CBCR	0x28
#define MLB_REG_IBCR	0x2C
#define MLB_REG_CICR	0x30
#define MLB_REG_CECRn	0x40
#define MLB_REG_CSCRn	0x44
#define MLB_REG_CCBCRn	0x48
#define MLB_REG_CNBCRn	0x4C
#define MLB_REG_LCBCRn	0x280

#define MLB_DCCR_FS_OFFSET	28
#define MLB_DCCR_EN		(1 << 31)
#define MLB_DCCR_LBM_OFFSET	30
#define MLB_DCCR_RESET		(1 << 23)
#define MLB_CECR_CE		(1 << 31)
#define MLB_CECR_TR		(1 << 30)
#define MLB_CECR_CT_OFFSET	28
#define MLB_CECR_MBS		(1 << 19)
#define MLB_CSCR_CBPE		(1 << 0)
#define MLB_CSCR_CBDB		(1 << 1)
#define MLB_CSCR_CBD		(1 << 2)
#define MLB_CSCR_CBS		(1 << 3)
#define MLB_CSCR_BE		(1 << 4)
#define MLB_CSCR_ABE		(1 << 5)
#define MLB_CSCR_LFS		(1 << 6)
#define MLB_CSCR_PBPE		(1 << 8)
#define MLB_CSCR_PBDB		(1 << 9)
#define MLB_CSCR_PBD		(1 << 10)
#define MLB_CSCR_PBS		(1 << 11)
#define MLB_CSCR_RDY		(1 << 16)
#define MLB_CSCR_BM		(1 << 31)
#define MLB_CSCR_BF		(1 << 30)
#define MLB_SSCR_SDML		(1 << 5)

#define MLB_CONTROL_TX_CHANN		(0 << 4)
#define MLB_CONTROL_RX_CHANN		(1 << 4)
#define MLB_ASYNC_TX_CHANN		(2 << 4)
#define MLB_ASYNC_RX_CHANN		(3 << 4)

#define MLB_MINOR_DEVICES		2
#define MLB_CONTROL_DEV_NAME		"ctrl"
#define MLB_ASYNC_DEV_NAME		"async"

#define TX_CHANNEL			0
#define RX_CHANNEL			1
#define TX_CHANNEL_BUF_SIZE		1024
#define RX_CHANNEL_BUF_SIZE		(2*1024)
/* max package data size */
#define ASYNC_PACKET_SIZE			1024
#define CTRL_PACKET_SIZE			64
#define RX_RING_NODES			10

#define MLB_IRAM_SIZE			(MLB_MINOR_DEVICES * (TX_CHANNEL_BUF_SIZE + RX_CHANNEL_BUF_SIZE))
#define _get_txchan(dev)		mlb_devinfo[dev].channels[TX_CHANNEL]
#define _get_rxchan(dev)		mlb_devinfo[dev].channels[RX_CHANNEL]

enum {
	MLB_CTYPE_SYNC,
	MLB_CTYPE_ISOC,
	MLB_CTYPE_ASYNC,
	MLB_CTYPE_CTRL,
};

/*!
 * Rx ring buffer
 */
struct mlb_rx_ringnode {
	int size;
	char *data;
};

struct mlb_channel_info {

	/* channel offset in memmap */
	const unsigned int reg_offset;
	/* channel address */
	int address;
	/*!
	 * channel buffer start address
	 * for Rx, buf_head pointer to a loop ring buffer
	 */
	unsigned long buf_head;
	/* physical buffer head address */
	unsigned long phy_head;
	/* channel buffer size */
	unsigned int buf_size;
	/* channel buffer current ptr */
	unsigned long buf_ptr;
	/* buffer spin lock */
	rwlock_t buf_lock;
};

struct mlb_dev_info {

	/* device node name */
	const char dev_name[20];
	/* channel type */
	const unsigned int channel_type;
	/* channel info for tx/rx */
	struct mlb_channel_info channels[2];
	/* rx ring buffer */
	struct mlb_rx_ringnode rx_bufs[RX_RING_NODES];
	/* rx ring buffer read/write ptr */
	unsigned int rdpos, wtpos;
	/* exception event */
	unsigned long ex_event;
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
	 .dev_name = MLB_CONTROL_DEV_NAME,
	 .channel_type = MLB_CTYPE_CTRL,
	 .channels = {
		      [0] = {
			     .reg_offset = MLB_CONTROL_TX_CHANN,
			     .buf_size = TX_CHANNEL_BUF_SIZE,
			     .buf_lock =
			     __RW_LOCK_UNLOCKED(mlb_devinfo[0].channels[0].
						buf_lock),
			     },
		      [1] = {
			     .reg_offset = MLB_CONTROL_RX_CHANN,
			     .buf_size = RX_CHANNEL_BUF_SIZE,
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
	 .dev_name = MLB_ASYNC_DEV_NAME,
	 .channel_type = MLB_CTYPE_ASYNC,
	 .channels = {
		      [0] = {
			     .reg_offset = MLB_ASYNC_TX_CHANN,
			     .buf_size = TX_CHANNEL_BUF_SIZE,
			     .buf_lock =
			     __RW_LOCK_UNLOCKED(mlb_devinfo[1].channels[0].
						buf_lock),
			     },
		      [1] = {
			     .reg_offset = MLB_ASYNC_RX_CHANN,
			     .buf_size = RX_CHANNEL_BUF_SIZE,
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
};

static struct regulator *reg_nvcc;	/* NVCC_MLB regulator */
static struct clk *mlb_clk;
static struct cdev mxc_mlb_dev;	/* chareset device */
static dev_t dev;
static struct class *mlb_class;	/* device class */
static struct device *class_dev;
static unsigned long mlb_base;	/* mlb module base address */
static unsigned int irq;
static unsigned long iram_base;
static __iomem void *iram_addr;

/*!
 * Initial the MLB module device
 */
static void mlb_dev_init(void)
{
	unsigned long dccr_val;
	unsigned long phyaddr;

	/* reset the MLB module */
	__raw_writel(MLB_DCCR_RESET, mlb_base + MLB_REG_DCCR);
	while (__raw_readl(mlb_base + MLB_REG_DCCR)
	       & MLB_DCCR_RESET)
		;

	/*!
	 * Enable MLB device, disable loopback mode,
	 * set default fps to 512, set mlb device address to 0
	 */
	dccr_val = MLB_DCCR_EN;
	__raw_writel(dccr_val, mlb_base + MLB_REG_DCCR);

	/* disable all the system interrupt */
	__raw_writel(0x5F, mlb_base + MLB_REG_SMCR);

	/* write async, control tx/rx base address */
	phyaddr = _get_txchan(0).phy_head >> 16;
	__raw_writel(phyaddr << 16 | phyaddr, mlb_base + MLB_REG_CBCR);
	phyaddr = _get_txchan(1).phy_head >> 16;
	__raw_writel(phyaddr << 16 | phyaddr, mlb_base + MLB_REG_ABCR);

}

static void mlb_dev_exit(void)
{
	__raw_writel(0, mlb_base + MLB_REG_DCCR);
}

/*!
 * MLB receive start function
 *
 * load phy_head to next buf register to start next rx
 * here use single-packet buffer, set start=end
 */
static void mlb_start_rx(int cdev_id)
{
	struct mlb_channel_info *chinfo = &_get_rxchan(cdev_id);
	unsigned long next;

	next = chinfo->phy_head & 0xFFFC;
	/* load next buf */
	__raw_writel((next << 16) | next, mlb_base +
		     MLB_REG_CNBCRn + chinfo->reg_offset);
	/* set ready bit to start next rx */
	__raw_writel(MLB_CSCR_RDY, mlb_base + MLB_REG_CSCRn
		     + chinfo->reg_offset);
}

/*!
 * MLB transmit start function
 * make sure aquiring the rw buf_lock, when calling this
 */
static void mlb_start_tx(int cdev_id)
{
	struct mlb_channel_info *chinfo = &_get_txchan(cdev_id);
	unsigned long begin, end;

	begin = chinfo->phy_head;
	end = (chinfo->phy_head + chinfo->buf_ptr - chinfo->buf_head) & 0xFFFC;
	/* load next buf */
	__raw_writel((begin << 16) | end, mlb_base +
		     MLB_REG_CNBCRn + chinfo->reg_offset);
	/* set ready bit to start next tx */
	__raw_writel(MLB_CSCR_RDY, mlb_base + MLB_REG_CSCRn
		     + chinfo->reg_offset);
}

/*!
 * Enable the MLB channel
 */
static void mlb_channel_enable(int chan_dev_id, int on)
{
	unsigned long tx_regval = 0, rx_regval = 0;
	/*!
	 * setup the direction, enable, channel type,
	 * mode select, channel address and mask buf start
	 */
	if (on) {
		unsigned int ctype = mlb_devinfo[chan_dev_id].channel_type;
		tx_regval = MLB_CECR_CE | MLB_CECR_TR | MLB_CECR_MBS |
		    (ctype << MLB_CECR_CT_OFFSET) |
		    _get_txchan(chan_dev_id).address;
		rx_regval = MLB_CECR_CE | MLB_CECR_MBS |
		    (ctype << MLB_CECR_CT_OFFSET) |
		    _get_rxchan(chan_dev_id).address;

		atomic_set(&mlb_devinfo[chan_dev_id].on, 1);
	} else {
		atomic_set(&mlb_devinfo[chan_dev_id].on, 0);
	}

	/* update the rx/tx channel entry config */
	__raw_writel(tx_regval, mlb_base + MLB_REG_CECRn +
		     _get_txchan(chan_dev_id).reg_offset);
	__raw_writel(rx_regval, mlb_base + MLB_REG_CECRn +
		     _get_rxchan(chan_dev_id).reg_offset);

	if (on)
		mlb_start_rx(chan_dev_id);
}

/*!
 * MLB interrupt handler
 */
void mlb_tx_isr(int minor, unsigned int cis)
{
	struct mlb_channel_info *chinfo = &_get_txchan(minor);

	if (cis & MLB_CSCR_CBD) {
		/* buffer done, reset the buf_ptr */
		write_lock(&chinfo->buf_lock);
		chinfo->buf_ptr = chinfo->buf_head;
		write_unlock(&chinfo->buf_lock);
		/* wake up the writer */
		wake_up_interruptible(&mlb_devinfo[minor].wt_wq);
	}
}

void mlb_rx_isr(int minor, unsigned int cis)
{
	struct mlb_channel_info *chinfo = &_get_rxchan(minor);
	unsigned long end;
	unsigned int len;

	if (cis & MLB_CSCR_CBD) {

		int wpos, rpos;

		rpos = mlb_devinfo[minor].rdpos;
		wpos = mlb_devinfo[minor].wtpos;

		/* buffer done, get current buffer ptr */
		end =
		    __raw_readl(mlb_base + MLB_REG_CCBCRn + chinfo->reg_offset);
		end >>= 16;	/* end here is phy */
		len = end - (chinfo->phy_head & 0xFFFC);

		/*!
		 * copy packet from IRAM buf to ring buf.
		 * if the wpos++ == rpos, drop this packet
		 */
		if (((wpos + 1) % RX_RING_NODES) != rpos) {

#ifdef DEBUG
			if (mlb_devinfo[minor].channel_type == MLB_CTYPE_CTRL) {
				if (len > CTRL_PACKET_SIZE)
					pr_debug
					    ("mxc_mlb: ctrl packet"
					     "overflow\n");
			} else {
				if (len > ASYNC_PACKET_SIZE)
					pr_debug
					    ("mxc_mlb: async packet"
					     "overflow\n");
			}
#endif
			memcpy(mlb_devinfo[minor].rx_bufs[wpos].data,
			       (const void *)chinfo->buf_head, len);
			mlb_devinfo[minor].rx_bufs[wpos].size = len;

			/* update the ring wpos */
			mlb_devinfo[minor].wtpos = (wpos + 1) % RX_RING_NODES;

			/* wake up the reader */
			wake_up_interruptible(&mlb_devinfo[minor].rd_wq);

			pr_debug("recv package, len:%d, rdpos: %d, wtpos: %d\n",
				 len, rpos, mlb_devinfo[minor].wtpos);
		} else {
			pr_debug
			    ("drop package, due to no space, (%d,%d)\n",
			     rpos, mlb_devinfo[minor].wtpos);
		}

		/* start next rx */
		mlb_start_rx(minor);
	}
}

static irqreturn_t mlb_isr(int irq, void *dev_id)
{
	unsigned long int_status, sscr, tx_cis, rx_cis;
	struct mlb_dev_info *pdev;
	int minor;

	sscr = __raw_readl(mlb_base + MLB_REG_SSCR);
	pr_debug("mxc_mlb: system interrupt:%lx\n", sscr);
	__raw_writel(0x7F, mlb_base + MLB_REG_SSCR);

	int_status = __raw_readl(mlb_base + MLB_REG_CICR) & 0xFFFF;
	pr_debug("mxc_mlb: channel interrupt ids: %lx\n", int_status);

	for (minor = 0; minor < MLB_MINOR_DEVICES; minor++) {

		pdev = &mlb_devinfo[minor];
		tx_cis = rx_cis = 0;

		/* get tx channel interrupt status */
		if (int_status & (1 << (_get_txchan(minor).reg_offset >> 4)))
			tx_cis = __raw_readl(mlb_base + MLB_REG_CSCRn
					     + _get_txchan(minor).reg_offset);
		/* get rx channel interrupt status */
		if (int_status & (1 << (_get_rxchan(minor).reg_offset >> 4)))
			rx_cis = __raw_readl(mlb_base + MLB_REG_CSCRn
					     + _get_rxchan(minor).reg_offset);

		if (!tx_cis && !rx_cis)
			continue;

		pr_debug("tx/rx int status: 0x%08lx/0x%08lx\n", tx_cis, rx_cis);
		/* fill exception event */
		spin_lock(&pdev->event_lock);
		pdev->ex_event |= tx_cis & 0x303;
		pdev->ex_event |= (rx_cis & 0x303) << 16;
		spin_unlock(&pdev->event_lock);

		/* clear the interrupt status */
		__raw_writel(tx_cis & 0xFFFF, mlb_base + MLB_REG_CSCRn
			     + _get_txchan(minor).reg_offset);
		__raw_writel(rx_cis & 0xFFFF, mlb_base + MLB_REG_CSCRn
			     + _get_rxchan(minor).reg_offset);

		/* handel tx channel */
		if (tx_cis)
			mlb_tx_isr(minor, tx_cis);
		/* handle rx channel */
		if (rx_cis)
			mlb_rx_isr(minor, rx_cis);

	}

	return IRQ_HANDLED;
}

static int mxc_mlb_open(struct inode *inode, struct file *filp)
{
	int minor;

	minor = MINOR(inode->i_rdev);

	if (minor < 0 || minor >= MLB_MINOR_DEVICES)
		return -ENODEV;

	/* open for each channel device */
	if (atomic_cmpxchg(&mlb_devinfo[minor].opencnt, 0, 1) != 0)
		return -EBUSY;

	/* reset the buffer read/write ptr */
	_get_txchan(minor).buf_ptr = _get_txchan(minor).buf_head;
	_get_rxchan(minor).buf_ptr = _get_rxchan(minor).buf_head;
	mlb_devinfo[minor].rdpos = mlb_devinfo[minor].wtpos = 0;
	mlb_devinfo[minor].ex_event = 0;

	return 0;
}

static int mxc_mlb_release(struct inode *inode, struct file *filp)
{
	int minor;

	minor = MINOR(inode->i_rdev);

	/* clear channel settings and info */
	mlb_channel_enable(minor, 0);

	/* decrease the open count */
	atomic_set(&mlb_devinfo[minor].opencnt, 0);

	return 0;
}

static int mxc_mlb_ioctl(struct inode *inode, struct file *filp,
			 unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	unsigned long flags, event;
	int minor;

	minor = MINOR(inode->i_rdev);

	switch (cmd) {

	case MLB_CHAN_SETADDR:
		{
			unsigned int caddr;
			/* get channel address from user space */
			if (copy_from_user(&caddr, argp, sizeof(caddr))) {
				pr_err("mxc_mlb: copy from user failed\n");
				return -EFAULT;
			}
			_get_txchan(minor).address = (caddr >> 16) & 0xFFFF;
			_get_rxchan(minor).address = caddr & 0xFFFF;
			break;
		}

	case MLB_CHAN_STARTUP:
		if (atomic_read(&mlb_devinfo[minor].on)) {
			pr_debug("mxc_mlb: channel areadly startup\n");
			break;
		}
		mlb_channel_enable(minor, 1);
		break;
	case MLB_CHAN_SHUTDOWN:
		if (atomic_read(&mlb_devinfo[minor].on) == 0) {
			pr_debug("mxc_mlb: channel areadly shutdown\n");
			break;
		}
		mlb_channel_enable(minor, 0);
		break;
	case MLB_CHAN_GETEVENT:
		/* get and clear the ex_event */
		spin_lock_irqsave(&mlb_devinfo[minor].event_lock, flags);
		event = mlb_devinfo[minor].ex_event;
		mlb_devinfo[minor].ex_event = 0;
		spin_unlock_irqrestore(&mlb_devinfo[minor].event_lock, flags);

		if (event) {
			if (copy_to_user(argp, &event, sizeof(event))) {
				pr_err("mxc_mlb: copy to user failed\n");
				return -EFAULT;
			}
		} else {
			pr_debug("mxc_mlb: no exception event now\n");
			return -EAGAIN;
		}
		break;
	case MLB_SET_FPS:
		{
			unsigned int fps;
			unsigned long dccr_val;

			/* get fps from user space */
			if (copy_from_user(&fps, argp, sizeof(fps))) {
				pr_err("mxc_mlb: copy from user failed\n");
				return -EFAULT;
			}

			/* check fps value */
			if (fps != 256 && fps != 512 && fps != 1024) {
				pr_debug("mxc_mlb: invalid fps argument\n");
				return -EINVAL;
			}

			dccr_val = __raw_readl(mlb_base + MLB_REG_DCCR);
			dccr_val &= ~(0x3 << MLB_DCCR_FS_OFFSET);
			dccr_val |= (fps >> 9) << MLB_DCCR_FS_OFFSET;
			__raw_writel(dccr_val, mlb_base + MLB_REG_DCCR);
			break;
		}

	case MLB_GET_VER:
		{
			unsigned long version;

			/* get MLB device module version */
			version = __raw_readl(mlb_base + MLB_REG_VCCR);

			if (copy_to_user(argp, &version, sizeof(version))) {
				pr_err("mxc_mlb: copy to user failed\n");
				return -EFAULT;
			}
			break;
		}

	case MLB_SET_DEVADDR:
		{
			unsigned long dccr_val;
			unsigned char devaddr;

			/* get MLB device address from user space */
			if (copy_from_user
			    (&devaddr, argp, sizeof(unsigned char))) {
				pr_err("mxc_mlb: copy from user failed\n");
				return -EFAULT;
			}

			dccr_val = __raw_readl(mlb_base + MLB_REG_DCCR);
			dccr_val &= ~0xFF;
			dccr_val |= devaddr;
			__raw_writel(dccr_val, mlb_base + MLB_REG_DCCR);

			break;
		}
	default:
		pr_info("mxc_mlb: Invalid ioctl command\n");
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
static ssize_t mxc_mlb_read(struct file *filp, char __user *buf,
			    size_t count, loff_t *f_pos)
{
	int minor, ret;
	int size, rdpos;
	struct mlb_rx_ringnode *rxbuf;

	minor = MINOR(filp->f_dentry->d_inode->i_rdev);

	rdpos = mlb_devinfo[minor].rdpos;
	rxbuf = mlb_devinfo[minor].rx_bufs;

	/* check the current rx buffer is available or not */
	if (rdpos == mlb_devinfo[minor].wtpos) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		/* if !O_NONBLOCK, we wait for recv packet */
		ret = wait_event_interruptible(mlb_devinfo[minor].rd_wq,
						(mlb_devinfo[minor].wtpos !=
						rdpos));
		if (ret < 0)
			return ret;
	}

	size = rxbuf[rdpos].size;
	if (size > count) {
		/* the user buffer is too small */
		pr_warning
		    ("mxc_mlb: received data size is bigger than count\n");
		return -EINVAL;
	}

	/* copy rx buffer data to user buffer */
	if (copy_to_user(buf, rxbuf[rdpos].data, size)) {
		pr_err("mxc_mlb: copy from user failed\n");
		return -EFAULT;
	}

	/* update the read ptr */
	mlb_devinfo[minor].rdpos = (rdpos + 1) % RX_RING_NODES;

	*f_pos = 0;

	return size;
}

/*!
 * MLB write routine
 *
 * Copy the user data to tx channel buffer,
 * and prepare the channel current/next buffer ptr.
 */
static ssize_t mxc_mlb_write(struct file *filp, const char __user *buf,
			     size_t count, loff_t *f_pos)
{
	int minor;
	unsigned long flags;
	DEFINE_WAIT(__wait);
	int ret;

	minor = MINOR(filp->f_dentry->d_inode->i_rdev);

	if (count > _get_txchan(minor).buf_size) {
		/* too many data to write */
		pr_warning("mxc_mlb: overflow write data\n");
		return -EFBIG;
	}

	*f_pos = 0;

	/* check the current tx buffer is used or not */
	write_lock_irqsave(&_get_txchan(minor).buf_lock, flags);
	if (_get_txchan(minor).buf_ptr != _get_txchan(minor).buf_head) {
		write_unlock_irqrestore(&_get_txchan(minor).buf_lock, flags);

		/* there's already some datas being transmit now */
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		/* if !O_NONBLOCK, we wait for transmit finish */
		for (;;) {
			prepare_to_wait(&mlb_devinfo[minor].wt_wq,
					&__wait, TASK_INTERRUPTIBLE);

			write_lock_irqsave(&_get_txchan(minor).buf_lock, flags);
			if (_get_txchan(minor).buf_ptr ==
			    _get_txchan(minor).buf_head)
				break;

			write_unlock_irqrestore(&_get_txchan(minor).buf_lock,
						flags);
			if (!signal_pending(current)) {
				schedule();
				continue;
			}
			return -ERESTARTSYS;
		}
		finish_wait(&mlb_devinfo[minor].wt_wq, &__wait);
	}

	/* copy user buffer to tx buffer */
	if (copy_from_user((void *)_get_txchan(minor).buf_ptr, buf, count)) {
		pr_err("mxc_mlb: copy from user failed\n");
		ret = -EFAULT;
		goto out;
	}
	_get_txchan(minor).buf_ptr += count;

	/* set current/next buffer start/end */
	mlb_start_tx(minor);

	ret = count;

out:
	write_unlock_irqrestore(&_get_txchan(minor).buf_lock, flags);
	return ret;
}

static unsigned int mxc_mlb_poll(struct file *filp,
				 struct poll_table_struct *wait)
{
	int minor;
	unsigned int ret = 0;
	unsigned long flags;

	minor = MINOR(filp->f_dentry->d_inode->i_rdev);

	poll_wait(filp, &mlb_devinfo[minor].rd_wq, wait);
	poll_wait(filp, &mlb_devinfo[minor].wt_wq, wait);

	/* check the tx buffer is avaiable or not */
	read_lock_irqsave(&_get_txchan(minor).buf_lock, flags);
	if (_get_txchan(minor).buf_ptr == _get_txchan(minor).buf_head)
		ret |= POLLOUT | POLLWRNORM;
	read_unlock_irqrestore(&_get_txchan(minor).buf_lock, flags);

	/* check the rx buffer filled or not */
	if (mlb_devinfo[minor].rdpos != mlb_devinfo[minor].wtpos)
		ret |= POLLIN | POLLRDNORM;

	/* check the exception event */
	if (mlb_devinfo[minor].ex_event)
		ret |= POLLIN | POLLRDNORM;

	return ret;
}

/*!
 * char dev file operations structure
 */
static struct file_operations mxc_mlb_fops = {

	.owner = THIS_MODULE,
	.open = mxc_mlb_open,
	.release = mxc_mlb_release,
	.ioctl = mxc_mlb_ioctl,
	.poll = mxc_mlb_poll,
	.read = mxc_mlb_read,
	.write = mxc_mlb_write,
};

/*!
 * This function is called whenever the MLB device is detected.
 */
static int __devinit mxc_mlb_probe(struct platform_device *pdev)
{
	int ret, mlb_major, i, j;
	struct mxc_mlb_platform_data *plat_data;
	struct resource *res;
	void __iomem *base, *bufaddr;
	unsigned long phyaddr;

	/* malloc the Rx ring buffer firstly */
	for (i = 0; i < MLB_MINOR_DEVICES; i++) {
		char *buf;
		int bufsize;

		if (mlb_devinfo[i].channel_type == MLB_CTYPE_ASYNC)
			bufsize = ASYNC_PACKET_SIZE;
		else
			bufsize = CTRL_PACKET_SIZE;

		buf = kmalloc(bufsize * RX_RING_NODES, GFP_KERNEL);
		if (buf == NULL) {
			ret = -ENOMEM;
			dev_err(&pdev->dev, "can not alloc rx buffers\n");
			goto err4;
		}
		for (j = 0; j < RX_RING_NODES; j++) {
			mlb_devinfo[i].rx_bufs[j].data = buf;
			buf += bufsize;
		}
	}

	/**
	 * Register MLB lld as two character devices
	 * One for Packet date channel, the other for control data channel
	 */
	ret = alloc_chrdev_region(&dev, 0, MLB_MINOR_DEVICES, "mxc_mlb");
	mlb_major = MAJOR(dev);

	if (ret < 0) {
		dev_err(&pdev->dev, "can't get major %d\n", mlb_major);
		goto err3;
	}

	cdev_init(&mxc_mlb_dev, &mxc_mlb_fops);
	mxc_mlb_dev.owner = THIS_MODULE;

	ret = cdev_add(&mxc_mlb_dev, dev, MLB_MINOR_DEVICES);
	if (ret) {
		dev_err(&pdev->dev, "can't add cdev\n");
		goto err2;
	}

	/* create class and device for udev information */
	mlb_class = class_create(THIS_MODULE, "mlb");
	if (IS_ERR(mlb_class)) {
		dev_err(&pdev->dev, "failed to create mlb class\n");
		ret = -ENOMEM;
		goto err2;
	}

	for (i = 0; i < MLB_MINOR_DEVICES; i++) {

		class_dev = device_create(mlb_class, NULL, MKDEV(mlb_major, i),
					  NULL, mlb_devinfo[i].dev_name);
		if (IS_ERR(class_dev)) {
			dev_err(&pdev->dev, "failed to create mlb %s"
				" class device\n", mlb_devinfo[i].dev_name);
			ret = -ENOMEM;
			goto err1;
		}
	}

	/* get irq line */
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "No mlb irq line provided\n");
		goto err1;
	}

	irq = res->start;
	/* request irq */
	if (request_irq(irq, mlb_isr, 0, "mlb", NULL)) {
		dev_err(&pdev->dev, "failed to request irq\n");
		ret = -EBUSY;
		goto err1;
	}

	/* ioremap from phy mlb to kernel space */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "No mlb base address provided\n");
		goto err0;
	}

	base = ioremap(res->start, res->end - res->start);
	dev_dbg(&pdev->dev, "mapped mlb base address: 0x%08x\n",
		(unsigned int)base);

	if (base == NULL) {
		dev_err(&pdev->dev, "failed to do ioremap with mlb base\n");
		goto err0;
	}
	mlb_base = (unsigned long)base;

	/*!
	 * get rx/tx buffer address from platform data
	 * make sure the buf_address is 4bytes aligned
	 *
	 * -------------------   <-- plat_data->buf_address
	 * | minor 0 tx buf  |
	 *  -----------------
	 * | minor 0 rx buf  |
	 *  -----------------
	 * |    ....         |
	 *  -----------------
	 * | minor n tx buf  |
	 *  -----------------
	 * | minor n rx buf  |
	 * -------------------
	 */

	plat_data = (struct mxc_mlb_platform_data *)pdev->dev.platform_data;

	bufaddr = iram_addr = iram_alloc(MLB_IRAM_SIZE, &iram_base);
	phyaddr = iram_base;

	for (i = 0; i < MLB_MINOR_DEVICES; i++) {
		/* set the virtual and physical buf head address */
		_get_txchan(i).buf_head = bufaddr;
		_get_txchan(i).phy_head = phyaddr;

		bufaddr += TX_CHANNEL_BUF_SIZE;
		phyaddr += TX_CHANNEL_BUF_SIZE;

		_get_rxchan(i).buf_head = (unsigned long)bufaddr;
		_get_rxchan(i).phy_head = phyaddr;

		bufaddr += RX_CHANNEL_BUF_SIZE;
		phyaddr += RX_CHANNEL_BUF_SIZE;

		dev_dbg(&pdev->dev, "phy_head: tx(%lx), rx(%lx)\n",
			_get_txchan(i).phy_head, _get_rxchan(i).phy_head);
		dev_dbg(&pdev->dev, "buf_head: tx(%lx), rx(%lx)\n",
			_get_txchan(i).buf_head, _get_rxchan(i).buf_head);
	}

	/* enable GPIO */
	gpio_mlb_active();

	if (plat_data->reg_nvcc) {
		/* power on MLB */
		reg_nvcc = regulator_get(&pdev->dev, plat_data->reg_nvcc);
		if (!IS_ERR(reg_nvcc)) {
			/* set MAX LDO6 for NVCC to 2.5V */
			regulator_set_voltage(reg_nvcc, 2500000, 2500000);
			regulator_enable(reg_nvcc);
		}
	}

	/* enable clock */
	mlb_clk = clk_get(&pdev->dev, plat_data->mlb_clk);
	clk_enable(mlb_clk);

	/* initial MLB module */
	mlb_dev_init();

	return 0;

err0:
	free_irq(irq, NULL);
err1:
	for (--i; i >= 0; i--)
		device_destroy(mlb_class, MKDEV(mlb_major, i));

	class_destroy(mlb_class);
err2:
	cdev_del(&mxc_mlb_dev);
err3:
	unregister_chrdev_region(dev, MLB_MINOR_DEVICES);
err4:
	for (i = 0; i < MLB_MINOR_DEVICES; i++)
		kfree(mlb_devinfo[i].rx_bufs[0].data);

	return ret;
}

static int __devexit mxc_mlb_remove(struct platform_device *pdev)
{
	int i;

	mlb_dev_exit();

	/* disable mlb clock */
	clk_disable(mlb_clk);
	clk_put(mlb_clk);

	/* disable mlb power */
	regulator_disable(reg_nvcc);
	regulator_put(reg_nvcc);

	/* inactive GPIO */
	gpio_mlb_inactive();

	iram_free(iram_base, MLB_IRAM_SIZE);

	/* iounmap */
	iounmap((void *)mlb_base);

	free_irq(irq, NULL);

	/* destroy mlb device class */
	for (i = MLB_MINOR_DEVICES - 1; i >= 0; i--)
		device_destroy(mlb_class, MKDEV(MAJOR(dev), i));
	class_destroy(mlb_class);

	/* Unregister the two MLB devices */
	cdev_del(&mxc_mlb_dev);
	unregister_chrdev_region(dev, MLB_MINOR_DEVICES);

	for (i = 0; i < MLB_MINOR_DEVICES; i++)
		kfree(mlb_devinfo[i].rx_bufs[0].data);

	return 0;
}

#ifdef CONFIG_PM
static int mxc_mlb_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int mxc_mlb_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define mxc_mlb_suspend NULL
#define mxc_mlb_resume NULL
#endif

/*!
 * platform driver structure for MLB
 */
static struct platform_driver mxc_mlb_driver = {
	.driver = {
		   .name = "mxc_mlb"},
	.probe = mxc_mlb_probe,
	.remove = __devexit_p(mxc_mlb_remove),
	.suspend = mxc_mlb_suspend,
	.resume = mxc_mlb_resume,
};

static int __init mxc_mlb_init(void)
{
	return platform_driver_register(&mxc_mlb_driver);
}

static void __exit mxc_mlb_exit(void)
{
	platform_driver_unregister(&mxc_mlb_driver);
}

module_init(mxc_mlb_init);
module_exit(mxc_mlb_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MLB low level driver");
MODULE_LICENSE("GPL");
