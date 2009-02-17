/***************************************************************************
 *
 * Copyright (C) 2004-2007  SMSC
 * Copyright (C) 2005 ARM
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 ***************************************************************************
 * Rewritten, heavily based on smsc911x simple driver by SMSC.
 * Partly uses io macros from smc91x.c by Nicolas Pitre
 *
 * Supported devices:
 *   LAN9115, LAN9116, LAN9117, LAN9118
 *   LAN9215, LAN9216, LAN9217, LAN9218
 *
 * History:
 *   05/05/2005 bahadir.balban@arm.com
 *     - Transition to linux coding style
 *     - Platform driver and module interface
 *
 *   17/07/2006 steve.glendinning@smsc.com
 *     - Added support for LAN921x family
 *     - Added workaround for multicast filters
 *
 *   31/07/2006 steve.glendinning@smsc.com
 *     - Removed tasklet, using NAPI poll instead
 *     - Multiple device support
 *     - Large tidy-up following feedback from netdev list
 *
 *   03/08/2006 steve.glendinning@smsc.com
 *     - Added ethtool support
 *     - Convert to use generic MII interface
 *
 *   04/08/2006 bahadir.balban@arm.com
 *     - Added ethtool eeprom r/w support
 *
 *   17/06/2007 steve.glendinning@smsc.com
 *     - Incorporate changes from Bill Gatliff and Russell King
 *
 *   04/07/2007 steve.glendinning@smsc.com
 *     - move irq configuration to platform_device
 *     - fix link poller after interface is stopped and restarted
 *
 *   13/07/2007 bahadir.balban@arm.com
 *     - set irq polarity before requesting irq
 */

#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/version.h>
#include <linux/bug.h>
#include <linux/bitops.h>
#include <linux/irq.h>
#include <asm/io.h>
#include "smsc911x.h"

#define SMSC_CHIPNAME		"smsc911x"
#define SMSC_DRV_VERSION	"2007-07-13"

MODULE_LICENSE("GPL");

struct smsc911x_data {
	void __iomem *ioaddr;

	unsigned int idrev;
	unsigned int generation;	/* used to decide which workarounds apply */

	/* device configuration */
	unsigned int irq_polarity;
	unsigned int irq_type;

	/* This needs to be acquired before calling any of below:
	 * smsc911x_mac_read(), smsc911x_mac_write()
	 * smsc911x_phy_read(), smsc911x_phy_write()
	 */
	spinlock_t phy_lock;

	struct net_device_stats stats;
	struct mii_if_info mii;
	unsigned int using_extphy;
	u32 msg_enable;
#ifdef USE_LED1_WORK_AROUND
	unsigned int gpio_setting;
	unsigned int gpio_orig_setting;
#endif
	struct net_device *netdev;
	struct napi_struct napi;
	struct timer_list link_poll_timer;
	unsigned int stop_link_poll;

	unsigned int software_irq_signal;

#ifdef USE_PHY_WORK_AROUND
#define MIN_PACKET_SIZE (64)
	char loopback_tx_pkt[MIN_PACKET_SIZE];
	char loopback_rx_pkt[MIN_PACKET_SIZE];
	unsigned int resetcount;
#endif

	/* Members for Multicast filter workaround */
	unsigned int multicast_update_pending;
	unsigned int set_bits_mask;
	unsigned int clear_bits_mask;
	unsigned int hashhi;
	unsigned int hashlo;
};

#if SMSC_CAN_USE_32BIT

static inline u32 smsc911x_reg_read(struct smsc911x_data *pdata, u32 reg)
{
	return readl(pdata->ioaddr + reg);
}

static inline void smsc911x_reg_write(u32 val, struct smsc911x_data *pdata,
				      u32 reg)
{
	writel(val, pdata->ioaddr + reg);
}

#elif SMSC_CAN_USE_SPI

static u32 smsc911x_reg_read(struct smsc911x_data *pdata, u32 reg)
{
	u32 reg_val;
	unsigned long flags;

	local_irq_save(flags);
	reg_val = spi_cpld_read(reg);
	local_irq_restore(flags);

	return reg_val;
}

static void smsc911x_reg_write(u32 val, struct smsc911x_data *pdata,
				      u32 reg)
{
	unsigned long flags;

	local_irq_save(flags);
	spi_cpld_write(reg, val);
	local_irq_restore(flags);
}

#else				/* SMSC_CAN_USE_32BIT */

static inline u32 smsc911x_reg_read(struct smsc911x_data *pdata, u32 reg)
{
	u32 reg_val;
	unsigned long flags;

	/* these two 16-bit reads must be performed consecutively, so must
	 * not be interrupted by our own ISR (which would start another
	 * read operation) */
	local_irq_save(flags);
	reg_val = ((readw(pdata->ioaddr + reg) & 0xFFFF) |
		   ((readw(pdata->ioaddr + reg + 2) & 0xFFFF) << 16));
	local_irq_restore(flags);

	return reg_val;
}

static inline void smsc911x_reg_write(u32 val, struct smsc911x_data *pdata,
				      u32 reg)
{
	unsigned long flags;

	/* these two 16-bit writes must be performed consecutively, so must
	 * not be interrupted by our own ISR (which would start another
	 * read operation) */
	local_irq_save(flags);
	writew(val & 0xFFFF, pdata->ioaddr + reg);
	writew((val >> 16) & 0xFFFF, pdata->ioaddr + reg + 2);
	local_irq_restore(flags);
}

#endif				/* SMSC_CAN_USE_32BIT */

/* Writes a packet to the TX_DATA_FIFO */
static inline void
smsc911x_tx_writefifo(struct smsc911x_data *pdata, unsigned int *buf,
		      unsigned int wordcount)
{
	while (wordcount--)
		smsc911x_reg_write(*buf++, pdata, TX_DATA_FIFO);
}

/* Reads a packet out of the RX_DATA_FIFO */
static inline void
smsc911x_rx_readfifo(struct smsc911x_data *pdata, unsigned int *buf,
		     unsigned int wordcount)
{
	while (wordcount--)
		*buf++ = smsc911x_reg_read(pdata, RX_DATA_FIFO);
}

/* waits for MAC not busy, with timeout.  Only called by smsc911x_mac_read
 * and smsc911x_mac_write, so assumes phy_lock is held */
static int smsc911x_mac_notbusy(struct smsc911x_data *pdata)
{
	int i;
	u32 val;

	for (i = 0; i < 40; i++) {
		val = smsc911x_reg_read(pdata, MAC_CSR_CMD);
		if (!(val & MAC_CSR_CMD_CSR_BUSY_))
			return 1;
	}
	SMSC_WARNING("Timed out waiting for MAC not BUSY. "
		     "MAC_CSR_CMD: 0x%08X", val);
	return 0;
}

/* Fetches a MAC register value. Assumes phy_lock is acquired */
static u32 smsc911x_mac_read(struct smsc911x_data *pdata, unsigned int offset)
{
	unsigned int temp;

#ifdef CONFIG_DEBUG_SPINLOCK
	if (!spin_is_locked(&pdata->phy_lock))
		SMSC_WARNING("phy_lock not held");
#endif				/* CONFIG_DEBUG_SPINLOCK */

	temp = smsc911x_reg_read(pdata, MAC_CSR_CMD);
	if (unlikely(temp & MAC_CSR_CMD_CSR_BUSY_)) {
		SMSC_WARNING("smsc911x_mac_read failed, MAC busy at entry");
		return 0xFFFFFFFF;
	}

	/* Send the MAC cmd */
	smsc911x_reg_write(((offset & 0xFF) | MAC_CSR_CMD_CSR_BUSY_
			    | MAC_CSR_CMD_R_NOT_W_), pdata, MAC_CSR_CMD);

	/* Workaround for hardware read-after-write restriction */
	temp = smsc911x_reg_read(pdata, BYTE_TEST);

	/* Wait for the read to happen */
	if (likely(smsc911x_mac_notbusy(pdata)))
		return smsc911x_reg_read(pdata, MAC_CSR_DATA);

	SMSC_WARNING("smsc911x_mac_read failed, MAC busy after read");
	return 0xFFFFFFFF;
}

/* Set a mac register, phy_lock must be acquired before calling */
static void smsc911x_mac_write(struct smsc911x_data *pdata,
			       unsigned int offset, u32 val)
{
	unsigned int temp;

#ifdef CONFIG_DEBUG_SPINLOCK
	if (!spin_is_locked(&pdata->phy_lock))
		SMSC_WARNING("phy_lock not held");
#endif				/* CONFIG_DEBUG_SPINLOCK */

	temp = smsc911x_reg_read(pdata, MAC_CSR_CMD);
	if (unlikely(temp & MAC_CSR_CMD_CSR_BUSY_)) {
		SMSC_WARNING("smsc911x_mac_write failed, MAC busy at entry");
		return;
	}

	/* Send data to write */
	smsc911x_reg_write(val, pdata, MAC_CSR_DATA);

	/* Write the actual data */
	smsc911x_reg_write(((offset & 0xFF) | MAC_CSR_CMD_CSR_BUSY_), pdata,
			   MAC_CSR_CMD);

	/* Workaround for hardware read-after-write restriction */
	temp = smsc911x_reg_read(pdata, BYTE_TEST);

	/* Wait for the write to complete */
	if (likely(smsc911x_mac_notbusy(pdata)))
		return;

	SMSC_WARNING("smsc911x_mac_write failed, MAC busy after write");
}

/* Gets a phy register, phy_lock must be acquired before calling */
static u16 smsc911x_phy_read(struct smsc911x_data *pdata, unsigned int index)
{
	unsigned int addr;
	int i;

#ifdef CONFIG_DEBUG_SPINLOCK
	if (!spin_is_locked(&pdata->phy_lock))
		SMSC_WARNING("phy_lock not held");
#endif				/* CONFIG_DEBUG_SPINLOCK */

	/* Confirm MII not busy */
	if (unlikely(smsc911x_mac_read(pdata, MII_ACC) & MII_ACC_MII_BUSY_)) {
		SMSC_WARNING("MII is busy in smsc911x_phy_read???");
		return 0;
	}

	/* Set the address, index & direction (read from PHY) */
	addr = (((pdata->mii.phy_id) & 0x1F) << 11)
	    | ((index & 0x1F) << 6);
	smsc911x_mac_write(pdata, MII_ACC, addr);

	/* Wait for read to complete w/ timeout */
	for (i = 0; i < 100; i++) {
		/* See if MII is finished yet */
		if (!(smsc911x_mac_read(pdata, MII_ACC) & MII_ACC_MII_BUSY_)) {
			return smsc911x_mac_read(pdata, MII_DATA);
		}
	}
	SMSC_WARNING("Timed out waiting for MII write to finish");
	return 0xFFFF;
}

/* Sets a phy register, phy_lock must be acquired before calling */
static void smsc911x_phy_write(struct smsc911x_data *pdata,
			       unsigned int index, u16 val)
{
	unsigned int addr;
	int i;

#ifdef CONFIG_DEBUG_SPINLOCK
	if (!spin_is_locked(&pdata->phy_lock))
		SMSC_WARNING("phy_lock not held");
#endif				/* CONFIG_DEBUG_SPINLOCK */

	/* Confirm MII not busy */
	if (unlikely(smsc911x_mac_read(pdata, MII_ACC) & MII_ACC_MII_BUSY_)) {
		SMSC_WARNING("MII is busy in smsc911x_write_phy???");
		return;
	}

	/* Put the data to write in the MAC */
	smsc911x_mac_write(pdata, MII_DATA, val);

	/* Set the address, index & direction (write to PHY) */
	addr = (((pdata->mii.phy_id) & 0x1F) << 11) |
	    ((index & 0x1F) << 6) | MII_ACC_MII_WRITE_;
	smsc911x_mac_write(pdata, MII_ACC, addr);

	/* Wait for write to complete w/ timeout */
	for (i = 0; i < 100; i++) {
		/* See if MII is finished yet */
		if (!(smsc911x_mac_read(pdata, MII_ACC) & MII_ACC_MII_BUSY_))
			return;
	}
	SMSC_WARNING("Timed out waiting for MII write to finish");
}

static int smsc911x_mdio_read(struct net_device *dev, int phy_id, int location)
{
	struct smsc911x_data *pdata = netdev_priv(dev);
	unsigned long flags;
	int reg;

	spin_lock_irqsave(&pdata->phy_lock, flags);
	reg = smsc911x_phy_read(pdata, location);
	spin_unlock_irqrestore(&pdata->phy_lock, flags);

	return reg;
}

static void smsc911x_mdio_write(struct net_device *dev, int phy_id,
				int location, int val)
{
	struct smsc911x_data *pdata = netdev_priv(dev);
	unsigned long flags;

	spin_lock_irqsave(&pdata->phy_lock, flags);
	smsc911x_phy_write(pdata, location, val);
	spin_unlock_irqrestore(&pdata->phy_lock, flags);
}

/* Autodetects and initialises external phy for SMSC9115 and SMSC9117 flavors.
 * If something goes wrong, returns -ENODEV to revert back to internal phy.
 * Performed at initialisation only, so interrupts are enabled */
static int smsc911x_phy_initialise_external(struct smsc911x_data *pdata)
{
	unsigned int address;
	unsigned int hwcfg;
	unsigned int phyid1;
	unsigned int phyid2;

	hwcfg = smsc911x_reg_read(pdata, HW_CFG);

	/* External phy is requested, supported, and detected */
	if (hwcfg & HW_CFG_EXT_PHY_DET_) {

		/* Attempt to switch to external phy for auto-detecting
		 * its address. Assuming tx and rx are stopped because
		 * smsc911x_phy_initialise is called before
		 * smsc911x_rx_initialise and tx_initialise.
		 */

		/* Disable phy clocks to the MAC */
		hwcfg &= (~HW_CFG_PHY_CLK_SEL_);
		hwcfg |= HW_CFG_PHY_CLK_SEL_CLK_DIS_;
		smsc911x_reg_write(hwcfg, pdata, HW_CFG);
		udelay(10);	/* Enough time for clocks to stop */

		/* Switch to external phy */
		hwcfg |= HW_CFG_EXT_PHY_EN_;
		smsc911x_reg_write(hwcfg, pdata, HW_CFG);

		/* Enable phy clocks to the MAC */
		hwcfg &= (~HW_CFG_PHY_CLK_SEL_);
		hwcfg |= HW_CFG_PHY_CLK_SEL_EXT_PHY_;
		smsc911x_reg_write(hwcfg, pdata, HW_CFG);
		udelay(10);	/* Enough time for clocks to restart */

		hwcfg |= HW_CFG_SMI_SEL_;
		smsc911x_reg_write(hwcfg, pdata, HW_CFG);

		/* Auto-detect PHY */
		spin_lock_irq(&pdata->phy_lock);
		for (address = 0; address <= 31; address++) {
			pdata->mii.phy_id = address;
			phyid1 = smsc911x_phy_read(pdata, MII_PHYSID1);
			phyid2 = smsc911x_phy_read(pdata, MII_PHYSID2);
			if ((phyid1 != 0xFFFFU) || (phyid2 != 0xFFFFU)) {
				SMSC_TRACE("Detected PHY at address = "
					   "0x%02X = %d", address, address);
				break;
			}
		}
		spin_unlock_irq(&pdata->phy_lock);

		if ((phyid1 == 0xFFFFU) && (phyid2 == 0xFFFFU)) {
			SMSC_WARNING("External PHY is not accessable, "
				     "using internal PHY instead");
			/* Revert back to internal phy settings. */

			/* Disable phy clocks to the MAC */
			hwcfg &= (~HW_CFG_PHY_CLK_SEL_);
			hwcfg |= HW_CFG_PHY_CLK_SEL_CLK_DIS_;
			smsc911x_reg_write(hwcfg, pdata, HW_CFG);
			udelay(10);	/* Enough time for clocks to stop */

			/* Switch to internal phy */
			hwcfg &= (~HW_CFG_EXT_PHY_EN_);
			smsc911x_reg_write(hwcfg, pdata, HW_CFG);

			/* Enable phy clocks to the MAC */
			hwcfg &= (~HW_CFG_PHY_CLK_SEL_);
			hwcfg |= HW_CFG_PHY_CLK_SEL_INT_PHY_;
			smsc911x_reg_write(hwcfg, pdata, HW_CFG);
			udelay(10);	/* Enough time for clocks to restart */

			hwcfg &= (~HW_CFG_SMI_SEL_);
			smsc911x_reg_write(hwcfg, pdata, HW_CFG);
			/* Use internal phy */
			return -ENODEV;
		} else {
			SMSC_TRACE("Successfully switched to external PHY");
			pdata->using_extphy = 1;
		}
	} else {
		SMSC_WARNING("No external PHY detected.");
		SMSC_WARNING("Using internal PHY instead.");
		/* Use internal phy */
		return -ENODEV;
	}
	return 0;
}

/* called by phy_initialise and loopback test */
static int smsc911x_phy_reset(struct smsc911x_data *pdata)
{
	unsigned int temp;
	unsigned int i = 100000;
	unsigned long flags;

	SMSC_TRACE("Performing PHY BCR Reset");
	spin_lock_irqsave(&pdata->phy_lock, flags);
	smsc911x_phy_write(pdata, MII_BMCR, BMCR_RESET);
	do {
		udelay(10);
		temp = smsc911x_phy_read(pdata, MII_BMCR);
	} while ((i--) && (temp & BMCR_RESET));
	spin_unlock_irqrestore(&pdata->phy_lock, flags);

	if (temp & BMCR_RESET) {
		SMSC_WARNING("PHY reset failed to complete.");
		return 0;
	}
	/* Extra delay required because the phy may not be completed with
	 * its reset when BMCR_RESET is cleared. Specs say 256 uS is
	 * enough delay but using 1ms here to be safe
	 */
	msleep(1);

	return 1;
}

/* Fetches a tx status out of the status fifo */
static unsigned int smsc911x_tx_get_txstatus(struct smsc911x_data *pdata)
{
	unsigned int result =
	    smsc911x_reg_read(pdata, TX_FIFO_INF) & TX_FIFO_INF_TSUSED_;

	if (result != 0)
		result = smsc911x_reg_read(pdata, TX_STATUS_FIFO);

	return result;
}

/* Fetches the next rx status */
static unsigned int smsc911x_rx_get_rxstatus(struct smsc911x_data *pdata)
{
	unsigned int result =
	    smsc911x_reg_read(pdata, RX_FIFO_INF) & RX_FIFO_INF_RXSUSED_;

	if (result != 0)
		result = smsc911x_reg_read(pdata, RX_STATUS_FIFO);

	return result;
}

#ifdef USE_PHY_WORK_AROUND
static int smsc911x_phy_check_loopbackpkt(struct smsc911x_data *pdata)
{
	unsigned int tries;
	u32 wrsz;
	u32 rdsz;
	u32 bufp;

	for (tries = 0; tries < 10; tries++) {
		unsigned int txcmd_a;
		unsigned int txcmd_b;
		unsigned int status;
		unsigned int pktlength;
		unsigned int i;

		/* Zero-out rx packet memory */
		memset(pdata->loopback_rx_pkt, 0, MIN_PACKET_SIZE);

		/* Write tx packet to 118 */
		txcmd_a = (((unsigned int)pdata->loopback_tx_pkt)
			   & 0x03) << 16;
		txcmd_a |= TX_CMD_A_FIRST_SEG_ | TX_CMD_A_LAST_SEG_;
		txcmd_a |= MIN_PACKET_SIZE;

		txcmd_b = MIN_PACKET_SIZE << 16 | MIN_PACKET_SIZE;

		smsc911x_reg_write(txcmd_a, pdata, TX_DATA_FIFO);
		smsc911x_reg_write(txcmd_b, pdata, TX_DATA_FIFO);

		bufp = ((u32) pdata->loopback_tx_pkt) & 0xFFFFFFFC;
		wrsz = MIN_PACKET_SIZE + 3;
		wrsz += (((u32) pdata->loopback_tx_pkt) & 0x3);
		wrsz >>= 2;

		smsc911x_tx_writefifo(pdata, (unsigned int *)bufp, wrsz);

		/* Wait till transmit is done */
		i = 60;
		do {
			udelay(5);
			status = smsc911x_tx_get_txstatus(pdata);
		} while ((i--) && (!status));

		if (!status) {
			SMSC_WARNING("Failed to transmit during loopback test");
			continue;
		}
		if (status & TX_STS_ES_) {
			SMSC_WARNING("Transmit encountered errors during "
				     "loopback test");
			continue;
		}

		/* Wait till receive is done */
		i = 60;
		do {
			udelay(5);
			status = smsc911x_rx_get_rxstatus(pdata);
		} while ((i--) && (!status));

		if (!status) {
			SMSC_WARNING("Failed to receive during loopback test");
			continue;
		}
		if (status & RX_STS_ES_) {
			SMSC_WARNING("Receive encountered errors during "
				     "loopback test");
			continue;
		}

		pktlength = ((status & 0x3FFF0000UL) >> 16);
		bufp = (u32)pdata->loopback_rx_pkt;
		rdsz = pktlength + 3;
		rdsz += ((u32)pdata->loopback_rx_pkt) & 0x3;
		rdsz >>= 2;

		smsc911x_rx_readfifo(pdata, (unsigned int *)bufp, rdsz);

		if (pktlength != (MIN_PACKET_SIZE + 4)) {
			SMSC_WARNING("Unexpected packet size during "
				     "loop back test, size=%d, "
				     "will retry", pktlength);
		} else {
			unsigned int j;
			int mismatch = 0;
			for (j = 0; j < MIN_PACKET_SIZE; j++) {
				if (pdata->loopback_tx_pkt[j]
				    != pdata->loopback_rx_pkt[j]) {
					mismatch = 1;
					break;
				}
			}
			if (!mismatch) {
				SMSC_TRACE("Successfully verified "
					   "loopback packet");
				return 1;
			} else {
				SMSC_WARNING("Data miss match during "
					     "loop back test, will retry.");
			}
		}
	}

	return 0;
}

static int smsc911x_phy_loopbacktest(struct smsc911x_data *pdata)
{
	int result = 0;
	unsigned int i;
	unsigned int val;
	unsigned long flags;

	/* Initialise tx packet */
	for (i = 0; i < 6; i++) {
		/* Use broadcast destination address */
		pdata->loopback_tx_pkt[i] = (char)0xFF;
	}

	for (i = 6; i < 12; i++) {
		/* Use incrementing source address */
		pdata->loopback_tx_pkt[i] = (char)i;
	}

	/* Set length type field */
	pdata->loopback_tx_pkt[12] = 0x00;
	pdata->loopback_tx_pkt[13] = 0x00;
	for (i = 14; i < MIN_PACKET_SIZE; i++) {
		pdata->loopback_tx_pkt[i] = (char)i;
	}

	val = smsc911x_reg_read(pdata, HW_CFG);
	val &= HW_CFG_TX_FIF_SZ_;
	val |= HW_CFG_SF_;
	smsc911x_reg_write(val, pdata, HW_CFG);

	smsc911x_reg_write(TX_CFG_TX_ON_, pdata, TX_CFG);
	smsc911x_reg_write((((unsigned int)pdata->loopback_rx_pkt)
			    & 0x03) << 8, pdata, RX_CFG);

	for (i = 0; i < 10; i++) {
		/* Set PHY to 10/FD, no ANEG, and loopback mode */
		spin_lock_irqsave(&pdata->phy_lock, flags);
		smsc911x_phy_write(pdata, MII_BMCR, 0x4100);

		/* Enable MAC tx/rx, FD */
		smsc911x_mac_write(pdata, MAC_CR, MAC_CR_FDPX_
				   | MAC_CR_TXEN_ | MAC_CR_RXEN_);
		spin_unlock_irqrestore(&pdata->phy_lock, flags);

		if (smsc911x_phy_check_loopbackpkt(pdata)) {
			result = 1;
			break;
		}
		pdata->resetcount++;

		/* Disable MAC rx */
		spin_lock_irqsave(&pdata->phy_lock, flags);
		smsc911x_mac_write(pdata, MAC_CR, 0);
		spin_unlock_irqrestore(&pdata->phy_lock, flags);

		smsc911x_phy_reset(pdata);
	}

	/* Disable MAC */
	spin_lock_irqsave(&pdata->phy_lock, flags);
	smsc911x_mac_write(pdata, MAC_CR, 0);

	/* Cancel PHY loopback mode */
	smsc911x_phy_write(pdata, MII_BMCR, 0);
	spin_unlock_irqrestore(&pdata->phy_lock, flags);

	smsc911x_reg_write(0, pdata, TX_CFG);
	smsc911x_reg_write(0, pdata, RX_CFG);

	return result;
}
#endif				/* USE_PHY_WORK_AROUND */

/* assumes phy_lock is held */
static void smsc911x_phy_update_flowcontrol(struct smsc911x_data *pdata)
{
	unsigned int temp;

	if (pdata->mii.full_duplex) {
		unsigned int phy_adv;
		unsigned int phy_lpa;
		phy_adv = smsc911x_phy_read(pdata, MII_ADVERTISE);
		phy_lpa = smsc911x_phy_read(pdata, MII_LPA);
		if (phy_adv & phy_lpa & LPA_PAUSE_CAP) {
			/* Both ends support symmetric pause, enable
			 * PAUSE receive and transmit */
			smsc911x_mac_write(pdata, FLOW, 0xFFFF0002);
			temp = smsc911x_reg_read(pdata, AFC_CFG);
			temp |= 0xF;
			smsc911x_reg_write(temp, pdata, AFC_CFG);
		} else if (((phy_adv & ADVERTISE_PAUSE_ALL) ==
			    ADVERTISE_PAUSE_ALL) &&
			   ((phy_lpa & LPA_PAUSE_ALL) == LPA_PAUSE_ASYM)) {
			/* We support symmetric and asym pause, the
			 * other end only supports asym, Enable PAUSE
			 * receive, disable PAUSE transmit */
			smsc911x_mac_write(pdata, FLOW, 0xFFFF0002);
			temp = smsc911x_reg_read(pdata, AFC_CFG);
			temp &= ~0xF;
			smsc911x_reg_write(temp, pdata, AFC_CFG);
		} else {
			/* Disable PAUSE receive and transmit */
			smsc911x_mac_write(pdata, FLOW, 0);
			temp = smsc911x_reg_read(pdata, AFC_CFG);
			temp &= ~0xF;
			smsc911x_reg_write(temp, pdata, AFC_CFG);
		}
	} else {
		smsc911x_mac_write(pdata, FLOW, 0);
		temp = smsc911x_reg_read(pdata, AFC_CFG);
		temp |= 0xF;
		smsc911x_reg_write(temp, pdata, AFC_CFG);
	}
}

/* Update link mode if any thing has changed */
static void smsc911x_phy_update_linkmode(struct net_device *dev, int init)
{
	struct smsc911x_data *pdata = netdev_priv(dev);
	unsigned long flags;

	if (mii_check_media(&pdata->mii, netif_msg_link(pdata), init)) {
		/* duplex state has changed */
		unsigned int mac_cr;

		spin_lock_irqsave(&pdata->phy_lock, flags);
		mac_cr = smsc911x_mac_read(pdata, MAC_CR);
		if (pdata->mii.full_duplex) {
			SMSC_TRACE("configuring for full duplex mode");
			mac_cr |= MAC_CR_FDPX_;
		} else {
			SMSC_TRACE("configuring for half duplex mode");
			mac_cr &= ~MAC_CR_FDPX_;
		}
		smsc911x_mac_write(pdata, MAC_CR, mac_cr);

		smsc911x_phy_update_flowcontrol(pdata);

		spin_unlock_irqrestore(&pdata->phy_lock, flags);
	}
#ifdef USE_LED1_WORK_AROUND
	if (netif_carrier_ok(dev)) {
		if ((pdata->gpio_orig_setting & GPIO_CFG_LED1_EN_) &&
		    (!pdata->using_extphy)) {
			/* Restore orginal GPIO configuration */
			pdata->gpio_setting = pdata->gpio_orig_setting;
			smsc911x_reg_write(pdata->gpio_setting, pdata,
					   GPIO_CFG);
		}
	} else {
		/* Check global setting that LED1
		 * usage is 10/100 indicator */
		pdata->gpio_setting = smsc911x_reg_read(pdata, GPIO_CFG);
		if ((pdata->gpio_setting & GPIO_CFG_LED1_EN_)
		    && (!pdata->using_extphy)) {
			/* Force 10/100 LED off, after saving
			 * orginal GPIO configuration */
			pdata->gpio_orig_setting = pdata->gpio_setting;

			pdata->gpio_setting &= ~GPIO_CFG_LED1_EN_;
			pdata->gpio_setting |= (GPIO_CFG_GPIOBUF0_
						| GPIO_CFG_GPIODIR0_
						| GPIO_CFG_GPIOD0_);
			smsc911x_reg_write(pdata->gpio_setting, pdata,
					   GPIO_CFG);
		}
	}
#endif				/* USE_LED1_WORK_AROUND */
}

/* Entry point for the link poller */
static void smsc911x_phy_checklink(unsigned long ptr)
{
	struct net_device *dev = (struct net_device *)ptr;
	struct smsc911x_data *pdata = netdev_priv(dev);

	smsc911x_phy_update_linkmode(dev, 0);

	if (!(pdata->stop_link_poll)) {
		pdata->link_poll_timer.expires = jiffies + 2 * HZ;
		add_timer(&pdata->link_poll_timer);
	} else {
		pdata->stop_link_poll = 0;
	}
}

/* Initialises the PHY layer.  Called at initialisation by open() so
 * interrupts are enabled */
static int smsc911x_phy_initialise(struct net_device *dev)
{
	struct smsc911x_data *pdata = netdev_priv(dev);
	unsigned int phyid1 = 0;
	unsigned int phyid2 = 0;
	unsigned int temp;

	pdata->using_extphy = 0;

	switch (pdata->idrev & 0xFFFF0000) {
	case 0x01170000:
	case 0x01150000:
		/* External PHY supported, try to autodetect */
		if (smsc911x_phy_initialise_external(pdata) < 0) {
			SMSC_TRACE("External PHY is not detected, using "
				   "internal PHY instead");
			pdata->mii.phy_id = 1;
		}
		break;
	default:
		SMSC_TRACE("External PHY is not supported, using internal PHY "
			   "instead");
		pdata->mii.phy_id = 1;
		break;
	}

	spin_lock_irq(&pdata->phy_lock);
	phyid1 = smsc911x_phy_read(pdata, MII_PHYSID1);
	phyid2 = smsc911x_phy_read(pdata, MII_PHYSID2);
	spin_unlock_irq(&pdata->phy_lock);

	if ((phyid1 == 0xFFFF) && (phyid2 == 0xFFFF)) {
		SMSC_WARNING("Internal PHY not detected!");
		return 0;
	}

	/* Reset the phy */
	if (!smsc911x_phy_reset(pdata)) {
		SMSC_WARNING("PHY reset failed to complete.");
		return 0;
	}
#ifdef USE_PHY_WORK_AROUND
	if (!smsc911x_phy_loopbacktest(pdata)) {
		SMSC_WARNING("Failed Loop Back Test");
		return 0;
	} else {
		SMSC_TRACE("Passed Loop Back Test");
	}
#endif				/* USE_PHY_WORK_AROUND */

	/* Advertise all speeds and pause capabilities */
	spin_lock_irq(&pdata->phy_lock);
	temp = smsc911x_phy_read(pdata, MII_ADVERTISE);
	temp |= (ADVERTISE_ALL | ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM);
	smsc911x_phy_write(pdata, MII_ADVERTISE, temp);
	pdata->mii.advertising = temp;

	if (!pdata->using_extphy) {
		/* using internal phy, enable PHY interrupts */
		smsc911x_phy_read(pdata, MII_INTSTS);
		smsc911x_phy_write(pdata, MII_INTMSK, PHY_INTMSK_DEFAULT_);
	}

	/* begin to establish link */
	smsc911x_phy_write(pdata, MII_BMCR, BMCR_ANENABLE | BMCR_ANRESTART);
	spin_unlock_irq(&pdata->phy_lock);

	smsc911x_phy_update_linkmode(dev, 1);

	setup_timer(&pdata->link_poll_timer, smsc911x_phy_checklink,
		(unsigned long)dev);
	pdata->link_poll_timer.expires = jiffies + 2 * HZ;
	add_timer(&pdata->link_poll_timer);

	SMSC_TRACE("phy initialised succesfully");
	return 1;
}

/* Gets the number of tx statuses in the fifo */
static unsigned int smsc911x_tx_get_txstatcount(struct smsc911x_data *pdata)
{
	unsigned int result = (smsc911x_reg_read(pdata, TX_FIFO_INF)
			       & TX_FIFO_INF_TSUSED_) >> 16;
	return result;
}

/* Reads tx statuses and increments counters where necessary */
static void smsc911x_tx_update_txcounters(struct smsc911x_data *pdata)
{
	unsigned int tx_stat;

	while ((tx_stat = smsc911x_tx_get_txstatus(pdata)) != 0) {
		if (unlikely(tx_stat & 0x80000000)) {
			/* In this driver the packet tag is used as the packet
			 * length. Since a packet length can never reach the
			 * size of 0x8000, this bit is reserved. It is worth
			 * noting that the "reserved bit" in the warning above
			 * does not reference a hardware defined reserved bit
			 * but rather a driver defined one.
			 */
			SMSC_WARNING("Packet tag reserved bit is high");
		} else {
			if (unlikely(tx_stat & 0x00008000)) {
				pdata->stats.tx_errors++;
			} else {
				pdata->stats.tx_packets++;
				pdata->stats.tx_bytes += (tx_stat >> 16);
			}
			if (unlikely(tx_stat & 0x00000100)) {
				pdata->stats.collisions += 16;
				pdata->stats.tx_aborted_errors += 1;
			} else {
				pdata->stats.collisions +=
				    ((tx_stat >> 3) & 0xF);
			}
			if (unlikely(tx_stat & 0x00000800)) {
				pdata->stats.tx_carrier_errors += 1;
			}
			if (unlikely(tx_stat & 0x00000200)) {
				pdata->stats.collisions++;
				pdata->stats.tx_aborted_errors++;
			}
		}
	}
}

/* Increments the Rx error counters */
static void
smsc911x_rx_counterrors(struct smsc911x_data *pdata, unsigned int rxstat)
{
	int crc_err = 0;

	if (unlikely(rxstat & 0x00008000)) {
		pdata->stats.rx_errors++;
		if (unlikely(rxstat & 0x00000002)) {
			pdata->stats.rx_crc_errors++;
			crc_err = 1;
		}
	}
	if (likely(!crc_err)) {
		if (unlikely((rxstat & 0x00001020) == 0x00001020)) {
			/* Frame type indicates length,
			 * and length error is set */
			pdata->stats.rx_length_errors++;
		}
		if (rxstat & RX_STS_MCAST_)
			pdata->stats.multicast++;
	}
}

/* Quickly dumps bad packets */
static void
smsc911x_rx_fastforward(struct smsc911x_data *pdata, unsigned int pktbytes)
{
	unsigned int pktwords = (pktbytes + NET_IP_ALIGN + 3) >> 2;

	if (likely(pktwords >= 4)) {
		unsigned int timeout = 500;
		unsigned int val;
		smsc911x_reg_write(RX_DP_CTRL_RX_FFWD_, pdata, RX_DP_CTRL);
		do {
			udelay(1);
			val = smsc911x_reg_read(pdata, RX_DP_CTRL);
		} while (timeout-- && (val & RX_DP_CTRL_RX_FFWD_));

		if (unlikely(timeout == 0))
			SMSC_WARNING("Timed out waiting for RX FFWD "
				     "to finish, RX_DP_CTRL: 0x%08X", val);
	} else {
		unsigned int temp;
		while (pktwords--)
			temp = smsc911x_reg_read(pdata, RX_DATA_FIFO);
	}
}

/* NAPI poll function */
static int smsc911x_poll(struct napi_struct *napi, int budget)
{
	struct smsc911x_data *pdata = container_of(napi, struct smsc911x_data, napi);
	struct net_device *dev = pdata->netdev;
	int npackets = 0;

	while (npackets < budget) {
		unsigned int pktlength;
		unsigned int pktwords;
		unsigned int rxstat = smsc911x_rx_get_rxstatus(pdata);

		/* break out of while loop if there are no more packets waiting */
		if (!rxstat)
			break;

		pktlength = ((rxstat & 0x3FFF0000) >> 16);
		pktwords = (pktlength + NET_IP_ALIGN + 3) >> 2;
		smsc911x_rx_counterrors(pdata, rxstat);

		if (likely((rxstat & RX_STS_ES_) == 0)) {
			struct sk_buff *skb;
			skb = dev_alloc_skb(pktlength + NET_IP_ALIGN);
			if (likely(skb)) {
				skb->data = skb->head;
				skb->tail = skb->head;
				/* Align IP on 16B boundary */
				skb_reserve(skb, NET_IP_ALIGN);
				skb_put(skb, pktlength - 4);
				smsc911x_rx_readfifo(pdata,
						     (unsigned int *)skb->head,
						     pktwords);
				skb->dev = dev;
				skb->protocol = eth_type_trans(skb, dev);
				skb->ip_summed = CHECKSUM_NONE;
				netif_receive_skb(skb);

				/* Update counters */
				pdata->stats.rx_packets++;
				pdata->stats.rx_bytes += (pktlength - 4);
				dev->last_rx = jiffies;
				npackets++;
				continue;
			} else {
				SMSC_WARNING("Unable to allocate sk_buff "
					     "for rx packet, in PIO path");
				pdata->stats.rx_dropped++;
			}
		}
		/* At this point, the packet is to be read out
		 * of the fifo and discarded */
		smsc911x_rx_fastforward(pdata, pktlength);
	}

	pdata->stats.rx_dropped += smsc911x_reg_read(pdata, RX_DROP);
	smsc911x_reg_write(INT_STS_RSFL_, pdata, INT_STS);

	if (npackets < budget) {
		unsigned int temp;
		/* We processed all packets available.  Tell NAPI it can
		 * stop polling then re-enable rx interrupts */
		netif_rx_complete(dev, napi);
		temp = smsc911x_reg_read(pdata, INT_EN);
		temp |= INT_EN_RSFL_EN_;
		smsc911x_reg_write(temp, pdata, INT_EN);
	}

	/* There are still packets waiting */
	return npackets;
}

/* Returns hash bit number for given MAC address
 * Example:
 * 01 00 5E 00 00 01 -> returns bit number 31 */
static unsigned int smsc911x_hash(char addr[ETH_ALEN])
{
	unsigned int crc;
	unsigned int result;

	crc = ether_crc(ETH_ALEN, addr);
	result = (crc >> 26) & 0x3f;

	return result;
}

static void smsc911x_rx_multicast_update(struct smsc911x_data *pdata)
{
	/* Performs the multicast & mac_cr update.  This is called when
	 * safe on the current hardware, and with the phy_lock held */
	unsigned int mac_cr = smsc911x_mac_read(pdata, MAC_CR);
	mac_cr |= pdata->set_bits_mask;
	mac_cr &= ~(pdata->clear_bits_mask);
	smsc911x_mac_write(pdata, MAC_CR, mac_cr);
	smsc911x_mac_write(pdata, HASHH, pdata->hashhi);
	smsc911x_mac_write(pdata, HASHL, pdata->hashlo);
	SMSC_TRACE("maccr 0x%08X, HASHH 0x%08X, HASHL 0x%08X", mac_cr,
		   pdata->hashhi, pdata->hashlo);
}

static void smsc911x_rx_multicast_update_workaround(struct smsc911x_data *pdata)
{
	unsigned int mac_cr;

	/* This function is only called for older LAN911x devices
	 * (revA or revB), where MAC_CR, HASHH and HASHL should not
	 * be modified during Rx - newer devices immediately update the
	 * registers.
	 *
	 * This is called from interrupt context */

	spin_lock(&pdata->phy_lock);

	/* Check Rx has stopped */
	if (smsc911x_mac_read(pdata, MAC_CR) & MAC_CR_RXEN_)
		SMSC_WARNING("Rx not stopped\n");

	/* Perform the update - safe to do now Rx has stopped */
	smsc911x_rx_multicast_update(pdata);

	/* Re-enable Rx */
	mac_cr = smsc911x_mac_read(pdata, MAC_CR);
	mac_cr |= MAC_CR_RXEN_;
	smsc911x_mac_write(pdata, MAC_CR, mac_cr);

	pdata->multicast_update_pending = 0;

	spin_unlock(&pdata->phy_lock);
}

/* Sets the device MAC address to dev_addr, called with phy_lock held */
static void
smsc911x_set_mac_address(struct smsc911x_data *pdata, u8 dev_addr[6])
{
	u32 mac_high16 = (dev_addr[5] << 8) | dev_addr[4];
	u32 mac_low32 = (dev_addr[3] << 24) | (dev_addr[2] << 16) |
	    (dev_addr[1] << 8) | dev_addr[0];

	smsc911x_mac_write(pdata, ADDRH, mac_high16);
	smsc911x_mac_write(pdata, ADDRL, mac_low32);
}

static int smsc911x_open(struct net_device *dev)
{
	struct smsc911x_data *pdata = netdev_priv(dev);
	unsigned int timeout;
	unsigned int temp;
	unsigned int intcfg;

	spin_lock_init(&pdata->phy_lock);

	/* Reset the LAN911x */
	smsc911x_reg_write(HW_CFG_SRST_, pdata, HW_CFG);
	timeout = 10;
	do {
		udelay(10);
		temp = smsc911x_reg_read(pdata, HW_CFG);
	} while ((--timeout) && (temp & HW_CFG_SRST_));

	if (unlikely(temp & HW_CFG_SRST_)) {
		SMSC_WARNING("Failed to complete reset");
		return -ENODEV;
	}

	smsc911x_reg_write(0x00050000, pdata, HW_CFG);
	smsc911x_reg_write(0x006E3740, pdata, AFC_CFG);

	/* Make sure EEPROM has finished loading before setting GPIO_CFG */
	timeout = 50;
	while ((timeout--) &&
	       (smsc911x_reg_read(pdata, E2P_CMD) & E2P_CMD_EPC_BUSY_)) {
		udelay(10);
	}

	if (unlikely(timeout == 0)) {
		SMSC_WARNING("Timed out waiting for EEPROM "
			     "busy bit to clear");
	}
#if USE_DEBUG >= 1
	smsc911x_reg_write(0x00670700, pdata, GPIO_CFG);
#else
	smsc911x_reg_write(0x70070000, pdata, GPIO_CFG);
#endif

	/* Initialise irqs, but leave all sources disabled */
	smsc911x_reg_write(0, pdata, INT_EN);
	smsc911x_reg_write(0xFFFFFFFF, pdata, INT_STS);

	/* Set interrupt deassertion to 100uS */
	intcfg = ((10 << 24) | INT_CFG_IRQ_EN_);

	if (pdata->irq_polarity) {
		SMSC_TRACE("irq polarity: active high");
		intcfg |= INT_CFG_IRQ_POL_;
	} else {
		SMSC_TRACE("irq polarity: active low");
	}

	if (pdata->irq_type) {
		SMSC_TRACE("irq type: push-pull");
		intcfg |= INT_CFG_IRQ_TYPE_;
	} else {
		SMSC_TRACE("irq type: open drain");
	}

	smsc911x_reg_write(intcfg, pdata, INT_CFG);

	SMSC_TRACE("Testing irq handler using IRQ %d", dev->irq);
	pdata->software_irq_signal = 0;
	smp_wmb();

	temp = smsc911x_reg_read(pdata, INT_EN);
	temp |= INT_EN_SW_INT_EN_;
	smsc911x_reg_write(temp, pdata, INT_EN);

	timeout = 1000;
	while (timeout--) {
		smp_rmb();
		if (pdata->software_irq_signal)
			break;
		msleep(1);
	}

	if (!pdata->software_irq_signal) {
		printk(KERN_WARNING "%s: ISR failed signaling test (IRQ %d)\n",
		       dev->name, dev->irq);
		return -ENODEV;
	}
	SMSC_TRACE("IRQ handler passed test using IRQ %d", dev->irq);

	printk(KERN_INFO "%s: SMSC911x/921x identified at %#08lx, IRQ: %d\n",
	       dev->name, (unsigned long)pdata->ioaddr, dev->irq);

	netif_carrier_off(dev);

	if (!smsc911x_phy_initialise(dev)) {
		SMSC_WARNING("Failed to initialize PHY");
		return -ENODEV;
	}

	smsc911x_set_mac_address(pdata, dev->dev_addr);

	temp = smsc911x_reg_read(pdata, HW_CFG);
	temp &= HW_CFG_TX_FIF_SZ_;
	temp |= HW_CFG_SF_;
	smsc911x_reg_write(temp, pdata, HW_CFG);

	temp = smsc911x_reg_read(pdata, FIFO_INT);
	temp |= FIFO_INT_TX_AVAIL_LEVEL_;
	temp &= ~(FIFO_INT_RX_STS_LEVEL_);
	smsc911x_reg_write(temp, pdata, FIFO_INT);

	/* set RX Data offset to 2 bytes for alignment */
	smsc911x_reg_write((2 << 8), pdata, RX_CFG);

	napi_enable(&pdata->napi);

	temp = smsc911x_reg_read(pdata, INT_EN);
	temp |= (INT_EN_TDFA_EN_ | INT_EN_RSFL_EN_ | INT_EN_PHY_INT_EN_);
	smsc911x_reg_write(temp, pdata, INT_EN);

	spin_lock_irq(&pdata->phy_lock);
	temp = smsc911x_mac_read(pdata, MAC_CR);
	temp |= (MAC_CR_TXEN_ | MAC_CR_RXEN_ | MAC_CR_HBDIS_);
	smsc911x_mac_write(pdata, MAC_CR, temp);
	spin_unlock_irq(&pdata->phy_lock);

	smsc911x_reg_write(TX_CFG_TX_ON_, pdata, TX_CFG);

	netif_start_queue(dev);
	return 0;
}

/* Entry point for stopping the interface */
static int smsc911x_stop(struct net_device *dev)
{
	struct smsc911x_data *pdata = netdev_priv(dev);

	napi_disable(&pdata->napi);

	pdata->stop_link_poll = 1;
	del_timer_sync(&pdata->link_poll_timer);

	smsc911x_reg_write((smsc911x_reg_read(pdata, INT_CFG) &
			    (~INT_CFG_IRQ_EN_)), pdata, INT_CFG);
	netif_stop_queue(dev);

	/* At this point all Rx and Tx activity is stopped */
	pdata->stats.rx_dropped += smsc911x_reg_read(pdata, RX_DROP);
	smsc911x_tx_update_txcounters(pdata);

	SMSC_TRACE("Interface stopped");
	return 0;
}

/* Entry point for transmitting a packet */
static int smsc911x_hard_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct smsc911x_data *pdata = netdev_priv(dev);
	unsigned int freespace;
	unsigned int tx_cmd_a;
	unsigned int tx_cmd_b;
	unsigned int temp;
	u32 wrsz;
	u32 bufp;

	freespace = smsc911x_reg_read(pdata, TX_FIFO_INF) & TX_FIFO_INF_TDFREE_;

	if (unlikely(freespace < TX_FIFO_LOW_THRESHOLD))
		SMSC_WARNING("Tx data fifo low, space available: %d",
			     freespace);

	/* Word alignment adjustment */
	tx_cmd_a = ((((unsigned int)(skb->data)) & 0x03) << 16);
	tx_cmd_a |= TX_CMD_A_FIRST_SEG_ | TX_CMD_A_LAST_SEG_;
	tx_cmd_a |= (unsigned int)skb->len;

	tx_cmd_b = ((unsigned int)skb->len) << 16;
	tx_cmd_b |= (unsigned int)skb->len;

	smsc911x_reg_write(tx_cmd_a, pdata, TX_DATA_FIFO);
	smsc911x_reg_write(tx_cmd_b, pdata, TX_DATA_FIFO);

	bufp = ((u32)skb->data) & 0xFFFFFFFC;
	wrsz = (u32)skb->len + 3;
	wrsz += ((u32)skb->data) & 0x3;
	wrsz >>= 2;

	smsc911x_tx_writefifo(pdata, (unsigned int *)bufp, wrsz);
	freespace -= (skb->len + 32);
	dev_kfree_skb(skb);
	dev->trans_start = jiffies;

	if (unlikely(smsc911x_tx_get_txstatcount(pdata) >= 30))
		smsc911x_tx_update_txcounters(pdata);

	if (freespace < TX_FIFO_LOW_THRESHOLD) {
		netif_stop_queue(dev);
		temp = smsc911x_reg_read(pdata, FIFO_INT);
		temp &= 0x00FFFFFF;
		temp |= 0x32000000;
		smsc911x_reg_write(temp, pdata, FIFO_INT);
	}

	return NETDEV_TX_OK;
}

/* Entry point for getting status counters */
static struct net_device_stats *smsc911x_get_stats(struct net_device *dev)
{
	struct smsc911x_data *pdata = netdev_priv(dev);
	smsc911x_tx_update_txcounters(pdata);
	pdata->stats.rx_dropped += smsc911x_reg_read(pdata, RX_DROP);
	return &pdata->stats;
}

/* Entry point for setting addressing modes */
static void smsc911x_set_multicast_list(struct net_device *dev)
{
	struct smsc911x_data *pdata = netdev_priv(dev);
	unsigned long flags;

	if (dev->flags & IFF_PROMISC) {
		/* Enabling promiscuous mode */
		pdata->set_bits_mask = MAC_CR_PRMS_;
		pdata->clear_bits_mask = (MAC_CR_MCPAS_ | MAC_CR_HPFILT_);
		pdata->hashhi = 0;
		pdata->hashlo = 0;
	} else if (dev->flags & IFF_ALLMULTI) {
		/* Enabling all multicast mode */
		pdata->set_bits_mask = MAC_CR_MCPAS_;
		pdata->clear_bits_mask = (MAC_CR_PRMS_ | MAC_CR_HPFILT_);
		pdata->hashhi = 0;
		pdata->hashlo = 0;
	} else if (dev->mc_count > 0) {
		/* Enabling specific multicast addresses */
		unsigned int hash_high = 0;
		unsigned int hash_low = 0;
		unsigned int count = 0;
		struct dev_mc_list *mc_list = dev->mc_list;

		pdata->set_bits_mask = MAC_CR_HPFILT_;
		pdata->clear_bits_mask = (MAC_CR_PRMS_ | MAC_CR_MCPAS_);

		while (mc_list) {
			count++;
			if ((mc_list->dmi_addrlen) == ETH_ALEN) {
				unsigned int bitnum =
				    smsc911x_hash(mc_list->dmi_addr);
				unsigned int mask = 0x01 << (bitnum & 0x1F);
				if (bitnum & 0x20)
					hash_high |= mask;
				else
					hash_low |= mask;
			} else {
				SMSC_WARNING("dmi_addrlen != 6");
			}
			mc_list = mc_list->next;
		}
		if (count != (unsigned int)dev->mc_count)
			SMSC_WARNING("mc_count != dev->mc_count");

		pdata->hashhi = hash_high;
		pdata->hashlo = hash_low;
	} else {
		/* Enabling local MAC address only */
		pdata->set_bits_mask = 0;
		pdata->clear_bits_mask =
		    (MAC_CR_PRMS_ | MAC_CR_MCPAS_ | MAC_CR_HPFILT_);
		pdata->hashhi = 0;
		pdata->hashlo = 0;
	}

	spin_lock_irqsave(&pdata->phy_lock, flags);

	if (pdata->generation <= 1) {
		/* Older hardware revision - cannot change these flags while
		 * receiving data */
		if (!pdata->multicast_update_pending) {
			unsigned int temp;
			SMSC_TRACE("scheduling mcast update");
			pdata->multicast_update_pending = 1;

			/* Request the hardware to stop, then perform the
			 * update when we get an RX_STOP interrupt */
			smsc911x_reg_write(INT_STS_RXSTOP_INT_, pdata, INT_STS);
			temp = smsc911x_reg_read(pdata, INT_EN);
			temp |= INT_EN_RXSTOP_INT_EN_;
			smsc911x_reg_write(temp, pdata, INT_EN);

			temp = smsc911x_mac_read(pdata, MAC_CR);
			temp &= ~(MAC_CR_RXEN_);
			smsc911x_mac_write(pdata, MAC_CR, temp);
		} else {
			/* There is another update pending, this should now
			 * use the newer values */
		}
	} else {
		/* Newer hardware revision - can write immediately */
		smsc911x_rx_multicast_update(pdata);
	}

	spin_unlock_irqrestore(&pdata->phy_lock, flags);
}

static irqreturn_t smsc911x_irqhandler(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct smsc911x_data *pdata = netdev_priv(dev);
	unsigned int intsts;
	unsigned int inten;
	unsigned int temp;
	int serviced = IRQ_NONE;

	intsts = smsc911x_reg_read(pdata, INT_STS);
	inten = smsc911x_reg_read(pdata, INT_EN);

	if (unlikely(intsts & inten & INT_STS_SW_INT_)) {
		temp = smsc911x_reg_read(pdata, INT_EN);
		temp &= (~INT_EN_SW_INT_EN_);
		smsc911x_reg_write(temp, pdata, INT_EN);
		smsc911x_reg_write(INT_STS_SW_INT_, pdata, INT_STS);
		pdata->software_irq_signal = 1;
		smp_wmb();
		serviced = IRQ_HANDLED;
	}

	if (unlikely(intsts & inten & INT_STS_RXSTOP_INT_)) {
		/* Called when there is a multicast update scheduled and
		 * it is now safe to complete the update */
		SMSC_TRACE("RX Stop interrupt");
		temp = smsc911x_reg_read(pdata, INT_EN);
		temp &= (~INT_EN_RXSTOP_INT_EN_);
		smsc911x_reg_write(temp, pdata, INT_EN);
		smsc911x_reg_write(INT_STS_RXSTOP_INT_, pdata, INT_STS);
		smsc911x_rx_multicast_update_workaround(pdata);
		serviced = IRQ_HANDLED;
	}

	if (intsts & inten & INT_STS_TDFA_) {
		temp = smsc911x_reg_read(pdata, FIFO_INT);
		temp |= FIFO_INT_TX_AVAIL_LEVEL_;
		smsc911x_reg_write(temp, pdata, FIFO_INT);
		smsc911x_reg_write(INT_STS_TDFA_, pdata, INT_STS);
		netif_wake_queue(dev);
		serviced = IRQ_HANDLED;
	}

	if (unlikely(intsts & inten & INT_STS_RXE_)) {
		smsc911x_reg_write(INT_STS_RXE_, pdata, INT_STS);
		serviced = IRQ_HANDLED;
	}

	if (likely(intsts & inten & INT_STS_RSFL_)) {
		/* Disable Rx interrupts and schedule NAPI poll */
		temp = smsc911x_reg_read(pdata, INT_EN);
		temp &= (~INT_EN_RSFL_EN_);
		smsc911x_reg_write(temp, pdata, INT_EN);
		netif_rx_schedule(dev, &pdata->napi);
		serviced = IRQ_HANDLED;
	}

	if (unlikely(intsts & inten & INT_STS_PHY_INT_)) {
		smsc911x_reg_write(INT_STS_PHY_INT_, pdata, INT_STS);
		spin_lock(&pdata->phy_lock);
		temp = smsc911x_phy_read(pdata, MII_INTSTS);
		spin_unlock(&pdata->phy_lock);
		SMSC_TRACE("PHY interrupt, sts 0x%04X", (u16)temp);
		smsc911x_phy_update_linkmode(dev, 0);
		serviced = IRQ_HANDLED;
	}
	return serviced;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
void smsc911x_poll_controller(struct net_device *dev)
{
	disable_irq(dev->irq);
	smsc911x_irqhandler(0, dev);
	enable_irq(dev->irq);
}
#endif				/* CONFIG_NET_POLL_CONTROLLER */

/* Standard ioctls for mii-tool */
static int smsc911x_do_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct smsc911x_data *pdata = netdev_priv(dev);
	struct mii_ioctl_data *data = if_mii(ifr);
	unsigned long flags;

	SMSC_TRACE("ioctl cmd 0x%x", cmd);
	switch (cmd) {
	case SIOCGMIIPHY:
		data->phy_id = pdata->mii.phy_id;
		return 0;
	case SIOCGMIIREG:
		spin_lock_irqsave(&pdata->phy_lock, flags);
		data->val_out = smsc911x_phy_read(pdata, data->reg_num);
		spin_unlock_irqrestore(&pdata->phy_lock, flags);
		return 0;
	case SIOCSMIIREG:
		spin_lock_irqsave(&pdata->phy_lock, flags);
		smsc911x_phy_write(pdata, data->reg_num, data->val_in);
		spin_unlock_irqrestore(&pdata->phy_lock, flags);
		return 0;
	}

	SMSC_TRACE("unsupported ioctl cmd");
	return -1;
}

static int
smsc911x_ethtool_getsettings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct smsc911x_data *pdata = netdev_priv(dev);

	cmd->maxtxpkt = 1;
	cmd->maxrxpkt = 1;
	return mii_ethtool_gset(&pdata->mii, cmd);
}

static int
smsc911x_ethtool_setsettings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct smsc911x_data *pdata = netdev_priv(dev);

	return mii_ethtool_sset(&pdata->mii, cmd);
}

static void smsc911x_ethtool_getdrvinfo(struct net_device *dev,
					struct ethtool_drvinfo *info)
{
	strncpy(info->driver, SMSC_CHIPNAME, sizeof(info->driver));
	strncpy(info->version, SMSC_DRV_VERSION, sizeof(info->version));
	strncpy(info->bus_info, dev->dev.bus_id, sizeof(info->bus_info));
}

static int smsc911x_ethtool_nwayreset(struct net_device *dev)
{
	struct smsc911x_data *pdata = netdev_priv(dev);

	return mii_nway_restart(&pdata->mii);
}

static u32 smsc911x_ethtool_getmsglevel(struct net_device *dev)
{
	struct smsc911x_data *pdata = netdev_priv(dev);
	return pdata->msg_enable;
}

static void smsc911x_ethtool_setmsglevel(struct net_device *dev, u32 level)
{
	struct smsc911x_data *pdata = netdev_priv(dev);
	pdata->msg_enable = level;
}

static int smsc911x_ethtool_getregslen(struct net_device *dev)
{
	return (((E2P_CMD - ID_REV) / 4 + 1) + (WUCSR - MAC_CR) + 1 + 32) *
	    sizeof(u32);
}

static void
smsc911x_ethtool_getregs(struct net_device *dev, struct ethtool_regs *regs,
			 void *buf)
{
	struct smsc911x_data *pdata = netdev_priv(dev);
	unsigned long flags;
	unsigned int i;
	unsigned int j = 0;
	u32 *data = buf;

	regs->version = pdata->idrev;
	for (i = ID_REV; i <= E2P_CMD; i += (sizeof(u32)))
		data[j++] = smsc911x_reg_read(pdata, i);

	spin_lock_irqsave(&pdata->phy_lock, flags);
	for (i = MAC_CR; i <= WUCSR; i++)
		data[j++] = smsc911x_mac_read(pdata, i);
	for (i = 0; i <= 31; i++)
		data[j++] = smsc911x_phy_read(pdata, i);
	spin_unlock_irqrestore(&pdata->phy_lock, flags);
}

static void smsc911x_eeprom_enable_access(struct smsc911x_data *pdata)
{
	unsigned int temp = smsc911x_reg_read(pdata, GPIO_CFG);
	temp &= ~GPIO_CFG_EEPR_EN_;
	smsc911x_reg_write(temp, pdata, GPIO_CFG);
	msleep(1);
}

static int smsc911x_eeprom_send_cmd(struct smsc911x_data *pdata, u32 op)
{
	int timeout = 100;
	u32 e2cmd;

	SMSC_TRACE("op 0x%08x", op);
	if (smsc911x_reg_read(pdata, E2P_CMD) & E2P_CMD_EPC_BUSY_) {
		SMSC_WARNING("Busy at start");
		return -EBUSY;
	}

	e2cmd = op | E2P_CMD_EPC_BUSY_;
	smsc911x_reg_write(e2cmd, pdata, E2P_CMD);

	do {
		msleep(1);
		e2cmd = smsc911x_reg_read(pdata, E2P_CMD);
	} while ((e2cmd & E2P_CMD_EPC_BUSY_) && (timeout--));

	if (!timeout) {
		SMSC_TRACE("TIMED OUT");
		return -EAGAIN;
	}

	if (e2cmd & E2P_CMD_EPC_TIMEOUT_) {
		SMSC_TRACE("Error occured during eeprom operation");
		return -EINVAL;
	}

	return 0;
}

static int smsc911x_eeprom_read_location(struct smsc911x_data *pdata,
					 u8 address, u8 *data)
{
	u32 op = E2P_CMD_EPC_CMD_READ_ | address;
	int ret;

	SMSC_TRACE("address 0x%x", address);
	ret = smsc911x_eeprom_send_cmd(pdata, op);

	if (!ret)
		data[address] = smsc911x_reg_read(pdata, E2P_DATA);

	return ret;
}

static int smsc911x_eeprom_write_location(struct smsc911x_data *pdata,
					  u8 address, u8 data)
{
	u32 op = E2P_CMD_EPC_CMD_ERASE_ | address;
	int ret;

	SMSC_TRACE("address 0x%x, data 0x%x", address, data);
	ret = smsc911x_eeprom_send_cmd(pdata, op);

	if (!ret) {
		op = E2P_CMD_EPC_CMD_WRITE_ | address;
		smsc911x_reg_write((u32)data, pdata, E2P_DATA);
		ret = smsc911x_eeprom_send_cmd(pdata, op);
	}

	return ret;
}

static int smsc911x_ethtool_get_eeprom_len(struct net_device *dev)
{
	return SMSC911X_EEPROM_SIZE;
}

static int smsc911x_ethtool_get_eeprom(struct net_device *dev,
				       struct ethtool_eeprom *eeprom, u8 *data)
{
	struct smsc911x_data *pdata = netdev_priv(dev);
	u8 eeprom_data[SMSC911X_EEPROM_SIZE];
	int len;
	int i;

	smsc911x_eeprom_enable_access(pdata);

	len = min(eeprom->len, SMSC911X_EEPROM_SIZE);
	for (i = 0; i < len; i++) {
		int ret = smsc911x_eeprom_read_location(pdata, i, eeprom_data);
		if (ret < 0) {
			eeprom->len = 0;
			return ret;
		}
	}

	memcpy(data, &eeprom_data[eeprom->offset], len);
	eeprom->len = len;
	return 0;
}

static int smsc911x_ethtool_set_eeprom(struct net_device *dev,
				       struct ethtool_eeprom *eeprom, u8 *data)
{
	int ret;
	struct smsc911x_data *pdata = netdev_priv(dev);

	smsc911x_eeprom_enable_access(pdata);
	smsc911x_eeprom_send_cmd(pdata, E2P_CMD_EPC_CMD_EWEN_);
	ret = smsc911x_eeprom_write_location(pdata, eeprom->offset, *data);
	smsc911x_eeprom_send_cmd(pdata, E2P_CMD_EPC_CMD_EWDS_);

	/* Single byte write, according to man page */
	eeprom->len = 1;

	return ret;
}

static struct ethtool_ops smsc911x_ethtool_ops = {
	.get_settings = smsc911x_ethtool_getsettings,
	.set_settings = smsc911x_ethtool_setsettings,
	.get_link = ethtool_op_get_link,
	.get_drvinfo = smsc911x_ethtool_getdrvinfo,
	.nway_reset = smsc911x_ethtool_nwayreset,
	.get_msglevel = smsc911x_ethtool_getmsglevel,
	.set_msglevel = smsc911x_ethtool_setmsglevel,
	.get_regs_len = smsc911x_ethtool_getregslen,
	.get_regs = smsc911x_ethtool_getregs,
	.get_eeprom_len = smsc911x_ethtool_get_eeprom_len,
	.get_eeprom = smsc911x_ethtool_get_eeprom,
	.set_eeprom = smsc911x_ethtool_set_eeprom,
};

/* Initializing private device structures */
static int smsc911x_init(struct net_device *dev)
{
	u32 mac_high16;
	u32 mac_low32;
	struct smsc911x_data *pdata = netdev_priv(dev);

	SMSC_TRACE("Driver Parameters:");
	SMSC_TRACE("LAN base: 0x%08lX", (unsigned long)pdata->ioaddr);
	SMSC_TRACE("IRQ: %d", dev->irq);
	SMSC_TRACE("PHY will be autodetected.");

	if (pdata->ioaddr == 0) {
		SMSC_WARNING("pdata->ioaddr: 0x00000000");
		return -ENODEV;
	}

	/* Default generation to zero (all workarounds apply) */
	pdata->generation = 0;

	pdata->idrev = smsc911x_reg_read(pdata, ID_REV);
	if (((pdata->idrev >> 16) & 0xFFFF) == (pdata->idrev & 0xFFFF)) {
		SMSC_WARNING("idrev top 16 bits equal to bottom 16 bits, "
			     "idrev: 0x%08X", pdata->idrev);
		SMSC_TRACE("This may mean the chip is set for 32 bit while "
			   "the bus is reading as 16 bit");
		return -ENODEV;
	}
	switch (pdata->idrev & 0xFFFF0000) {
	case 0x01180000:
		switch (pdata->idrev & 0x0000FFFFUL) {
		case 0UL:
			SMSC_TRACE("LAN9118 Beacon identified, idrev: 0x%08X",
				   pdata->idrev);
			pdata->generation = 0;
			break;
		case 1UL:
			SMSC_TRACE
			    ("LAN9118 Concord A0 identified, idrev: 0x%08X",
			     pdata->idrev);
			pdata->generation = 1;
			break;
		case 2UL:
			SMSC_TRACE
			    ("LAN9118 Concord A1 identified, idrev: 0x%08X",
			     pdata->idrev);
			pdata->generation = 2;
			break;
		default:
			SMSC_TRACE
			    ("LAN9118 Concord A1 identified (NEW), idrev: 0x%08X",
			     pdata->idrev);
			pdata->generation = 2;
			break;
		}
		break;

	case 0x01170000:
		switch (pdata->idrev & 0x0000FFFFUL) {
		case 0UL:
			SMSC_TRACE("LAN9117 Beacon identified, idrev: 0x%08X",
				   pdata->idrev);
			pdata->generation = 0;
			break;
		case 1UL:
			SMSC_TRACE
			    ("LAN9117 Concord A0 identified, idrev: 0x%08X",
			     pdata->idrev);
			pdata->generation = 1;
			break;
		case 2UL:
			SMSC_TRACE
			    ("LAN9117 Concord A1 identified, idrev: 0x%08X",
			     pdata->idrev);
			pdata->generation = 2;
			break;
		default:
			SMSC_TRACE
			    ("LAN9117 Concord A1 identified (NEW), idrev: 0x%08X",
			     pdata->idrev);
			pdata->generation = 2;
			break;
		}
		break;

	case 0x01160000:
		switch (pdata->idrev & 0x0000FFFFUL) {
		case 0UL:
			SMSC_WARNING("LAN911x not identified, idrev: 0x%08X",
				     pdata->idrev);
			return -ENODEV;
		case 1UL:
			SMSC_TRACE
			    ("LAN9116 Concord A0 identified, idrev: 0x%08X",
			     pdata->idrev);
			pdata->generation = 1;
			break;
		case 2UL:
			SMSC_TRACE
			    ("LAN9116 Concord A1 identified, idrev: 0x%08X",
			     pdata->idrev);
			pdata->generation = 2;
			break;
		default:
			SMSC_TRACE
			    ("LAN9116 Concord A1 identified (NEW), idrev: 0x%08X",
			     pdata->idrev);
			pdata->generation = 2;
			break;
		}
		break;

	case 0x01150000:
		switch (pdata->idrev & 0x0000FFFFUL) {
		case 0UL:
			SMSC_WARNING("LAN911x not identified, idrev: 0x%08X",
				     pdata->idrev);
			return -ENODEV;
		case 1UL:
			SMSC_TRACE
			    ("LAN9115 Concord A0 identified, idrev: 0x%08X",
			     pdata->idrev);
			pdata->generation = 1;
			break;
		case 2UL:
			SMSC_TRACE
			    ("LAN9115 Concord A1 identified, idrev: 0x%08X",
			     pdata->idrev);
			pdata->generation = 2;
			break;
		default:
			SMSC_TRACE
			    ("LAN9115 Concord A1 identified (NEW), idrev: 0x%08X",
			     pdata->idrev);
			pdata->generation = 2;
			break;
		}
		break;

	case 0x118A0000UL:
		switch (pdata->idrev & 0x0000FFFFUL) {
		case 0UL:
			SMSC_TRACE
			    ("LAN9218 Boylston identified, idrev: 0x%08X",
			     pdata->idrev);
			pdata->generation = 3;
			break;
		default:
			SMSC_TRACE
			    ("LAN9218 Boylston identified (NEW), idrev: 0x%08X",
			     pdata->idrev);
			pdata->generation = 3;
			break;
		}
		break;

	case 0x117A0000UL:
		switch (pdata->idrev & 0x0000FFFFUL) {
		case 0UL:
			SMSC_TRACE
			    ("LAN9217 Boylston identified, idrev: 0x%08X",
			     pdata->idrev);
			pdata->generation = 3;
			break;
		default:
			SMSC_TRACE
			    ("LAN9217 Boylston identified (NEW), idrev: 0x%08X",
			     pdata->idrev);
			pdata->generation = 3;
			break;
		}
		break;

	case 0x116A0000UL:
		switch (pdata->idrev & 0x0000FFFFUL) {
		case 0UL:
			SMSC_TRACE
			    ("LAN9216 Boylston identified, idrev: 0x%08X",
			     pdata->idrev);
			pdata->generation = 3;
			break;
		default:
			SMSC_TRACE
			    ("LAN9216 Boylston identified (NEW), idrev: 0x%08X",
			     pdata->idrev);
			pdata->generation = 3;
			break;
		}
		break;

	case 0x115A0000UL:
		switch (pdata->idrev & 0x0000FFFFUL) {
		case 0UL:
			SMSC_TRACE
			    ("LAN9215 Boylston identified, idrev: 0x%08X",
			     pdata->idrev);
			pdata->generation = 3;
			break;
		default:
			SMSC_TRACE
			    ("LAN9215 Boylston identified (NEW), idrev: 0x%08X",
			     pdata->idrev);
			pdata->generation = 3;
			break;
		}
		break;

	case 0x92100000UL:
	case 0x92110000UL:
	case 0x92200000UL:
	case 0x92210000UL:
		/* LAN9210/LAN9211/LAN9220/LAN9221 */
		pdata->generation = 4;
		break;

	default:
		SMSC_WARNING("LAN911x not identified, idrev: 0x%08X",
			     pdata->idrev);
		return -ENODEV;
	}

	if (pdata->generation == 0)
		SMSC_WARNING("This driver is not intended "
			     "for this chip revision");

	ether_setup(dev);
	dev->open = smsc911x_open;
	dev->stop = smsc911x_stop;
	dev->hard_start_xmit = smsc911x_hard_start_xmit;
	dev->get_stats = smsc911x_get_stats;
	dev->set_multicast_list = smsc911x_set_multicast_list;
	dev->flags |= IFF_MULTICAST;
	dev->do_ioctl = smsc911x_do_ioctl;
	netif_napi_add(dev, &pdata->napi, smsc911x_poll, 64);
	dev->ethtool_ops = &smsc911x_ethtool_ops;

	/* Check if mac address has been specified when bringing interface up */
	if (is_valid_ether_addr(dev->dev_addr)) {
		smsc911x_set_mac_address(pdata, dev->dev_addr);
		SMSC_TRACE("MAC Address is specified by configuration");
	} else {
		/* Try reading mac address from device. if EEPROM is present
		 * it will already have been set */
		u32 mac_high16 = smsc911x_mac_read(pdata, ADDRH);
		u32 mac_low32 = smsc911x_mac_read(pdata, ADDRL);
		dev->dev_addr[0] = (u8)(mac_low32);
		dev->dev_addr[1] = (u8)(mac_low32 >> 8);
		dev->dev_addr[2] = (u8)(mac_low32 >> 16);
		dev->dev_addr[3] = (u8)(mac_low32 >> 24);
		dev->dev_addr[4] = (u8)(mac_high16);
		dev->dev_addr[5] = (u8)(mac_high16 >> 8);

		if (is_valid_ether_addr(dev->dev_addr)) {
			/* eeprom values are valid  so use them */
			SMSC_TRACE("Mac Address is read from LAN911x EEPROM");
		} else {
			/* eeprom values are invalid, generate random MAC */
			random_ether_addr(dev->dev_addr);
			smsc911x_set_mac_address(pdata, dev->dev_addr);
			SMSC_TRACE("MAC Address is set to random_ether_addr");
		}
	}

	printk(KERN_INFO
	       "%s: SMSC911x MAC Address: %02x:%02x:%02x:%02x:%02x:%02x\n",
	       dev->name, dev->dev_addr[0], dev->dev_addr[1], dev->dev_addr[2],
	       dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5]);

#ifdef CONFIG_NET_POLL_CONTROLLER
	dev->poll_controller = smsc911x_poll_controller;
#endif				/* CONFIG_NET_POLL_CONTROLLER */

	pdata->mii.phy_id_mask = 0x1f;
	pdata->mii.reg_num_mask = 0x1f;
	pdata->mii.force_media = 0;
	pdata->mii.full_duplex = 0;
	pdata->mii.dev = dev;
	pdata->mii.mdio_read = smsc911x_mdio_read;
	pdata->mii.mdio_write = smsc911x_mdio_write;

	pdata->msg_enable = NETIF_MSG_LINK;

	return 0;
}

static int smsc911x_drv_remove(struct platform_device *pdev)
{
	struct net_device *dev;
	struct smsc911x_data *pdata;
	struct resource *res;

	dev = platform_get_drvdata(pdev);
	BUG_ON(!dev);
	pdata = netdev_priv(dev);
	BUG_ON(!pdata);
	BUG_ON(!pdata->ioaddr);

	SMSC_TRACE("Stopping driver.");
	platform_set_drvdata(pdev, NULL);
	unregister_netdev(dev);
	free_irq(dev->irq, dev);
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "smsc911x-memory");
	if (!res)
		platform_get_resource(pdev, IORESOURCE_MEM, 0);

	release_mem_region(res->start, res->end - res->start);

	iounmap(pdata->ioaddr);

	free_netdev(dev);

	return 0;
}

static int smsc911x_drv_probe(struct platform_device *pdev)
{
	struct net_device *dev;
	struct smsc911x_data *pdata;
	struct resource *res;
	unsigned int intcfg = 0;
	int res_size;
	int retval;

	printk(KERN_INFO "%s: Driver version %s.\n", SMSC_CHIPNAME,
		SMSC_DRV_VERSION);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "smsc911x-memory");
	if (!res)
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		printk(KERN_WARNING "%s: Could not allocate resource.\n",
		       SMSC_CHIPNAME);
		retval = -ENODEV;
		goto out_0;
	}
	res_size = res->end - res->start;

	if (!request_mem_region(res->start, res_size, SMSC_CHIPNAME)) {
		retval = -EBUSY;
		goto out_0;
	}

	dev = alloc_etherdev(sizeof(struct smsc911x_data));
	if (!dev) {
		printk(KERN_WARNING "%s: Could not allocate device.\n",
		       SMSC_CHIPNAME);
		retval = -ENOMEM;
		goto out_release_io_1;
	}

	SET_NETDEV_DEV(dev, &pdev->dev);

	pdata = netdev_priv(dev);
	pdata->netdev = dev;

	dev->irq = platform_get_irq(pdev, 0);
	pdata->ioaddr = ioremap_nocache(res->start, res_size);

	/* copy config parameters across if present, otherwise pdata
	 * defaults to zeros */
	if (pdev->dev.platform_data) {
		struct smsc911x_platform_config *config = pdev->dev.platform_data;
		pdata->irq_polarity = config->irq_polarity;
		pdata->irq_type  = config->irq_type;
	}

	if (pdata->ioaddr == NULL) {
		SMSC_WARNING("Error smsc911x base address invalid");
		retval = -ENOMEM;
		goto out_free_netdev_2;
	}

	if ((retval = smsc911x_init(dev)) < 0)
		goto out_unmap_io_3;

	/* configure irq polarity and type before connecting isr */
	if (pdata->irq_polarity)
		intcfg |= INT_CFG_IRQ_POL_;

	if (pdata->irq_type)
		intcfg |= INT_CFG_IRQ_TYPE_;

	smsc911x_reg_write(intcfg, pdata, INT_CFG);

	retval = request_irq(dev->irq, smsc911x_irqhandler, IRQF_DISABLED,
			     SMSC_CHIPNAME, dev);
	if (retval) {
		SMSC_WARNING("Unable to claim requested irq: %d", dev->irq);
		goto out_unmap_io_3;
	}

	platform_set_drvdata(pdev, dev);

	retval = register_netdev(dev);
	if (retval) {
		SMSC_WARNING("Error %i registering device", retval);
		goto out_unset_drvdata_4;
	} else {
		SMSC_TRACE("Network interface: \"%s\"", dev->name);
	}

	return 0;

out_unset_drvdata_4:
	platform_set_drvdata(pdev, NULL);
	free_irq(dev->irq, dev);
out_unmap_io_3:
	iounmap(pdata->ioaddr);
out_free_netdev_2:
	free_netdev(dev);
out_release_io_1:
	release_mem_region(res->start, res->end - res->start);
out_0:
	return retval;
}

static int smsc911x_drv_suspend(struct platform_device *pdev,
				       pm_message_t state)
{
	unsigned long flags;
	struct net_device *dev;
	struct smsc911x_data *pdata;

	dev = platform_get_drvdata(pdev);
	BUG_ON(!dev);
	pdata = netdev_priv(dev);
	BUG_ON(!pdata);
	BUG_ON(!pdata->ioaddr);

	spin_lock_irqsave(&pdata->phy_lock, flags);
	smsc911x_phy_write(pdata, MII_BMCR, BMCR_PDOWN);
	spin_unlock_irqrestore(&pdata->phy_lock, flags);

	smsc911x_reg_write(PMT_CTRL_PM_MODE_D2_, pdata, PMT_CTRL);

	return 0;
}

static int smsc911x_drv_resume(struct platform_device *pdev)
{
	unsigned long flags;
	struct net_device *dev;
	struct smsc911x_data *pdata;
	unsigned int temp;

	dev = platform_get_drvdata(pdev);
	BUG_ON(!dev);
	pdata = netdev_priv(dev);
	BUG_ON(!pdata);
	BUG_ON(!pdata->ioaddr);

	smsc911x_reg_write(0xFFFFFFFF, pdata, BYTE_TEST);
	while (!(smsc911x_reg_read(pdata, PMT_CTRL) & PMT_CTRL_READY_))
		;

	spin_lock_irqsave(&pdata->phy_lock, flags);
	smsc911x_phy_write(pdata, MII_BMCR, BMCR_ANENABLE | BMCR_ANRESTART);
	spin_unlock_irqrestore(&pdata->phy_lock, flags);
	return 0;
}

static struct platform_driver smsc911x_driver = {
	.probe = smsc911x_drv_probe,
	.remove = smsc911x_drv_remove,
	.suspend = smsc911x_drv_suspend,
	.resume = smsc911x_drv_resume,
	.driver = {
		.name = SMSC_CHIPNAME,
	},
};

/* Entry point for loading the module */
static int __init smsc911x_init_module(void)
{
	return platform_driver_register(&smsc911x_driver);
}

/* entry point for unloading the module */
static void __exit smsc911x_cleanup_module(void)
{
	platform_driver_unregister(&smsc911x_driver);
}

module_init(smsc911x_init_module);
module_exit(smsc911x_cleanup_module);
