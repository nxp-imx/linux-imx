/*
 *  L2 switch Controller (Etheren switch) driver for Mx28.
 *
 *  Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *    Shrek Wu (B16972@freescale.com)
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/ptrace.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/fec.h>
#include <linux/phy.h>

#include <asm/irq.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <asm/pgtable.h>
#include <asm/cacheflush.h>

#include "fec_switch.h"

#define	SWITCH_MAX_PORTS	1

#if defined(CONFIG_ARCH_MXC) || defined(CONFIG_ARCH_MXS)
#include <mach/hardware.h>
#define FEC_ALIGNMENT   0xf
#else
#define FEC_ALIGNMENT   0x3
#endif

/* The number of Tx and Rx buffers.  These are allocated from the page
 * pool.  The code may assume these are power of two, so it it best
 * to keep them that size.
 * We don't need to allocate pages for the transmitter.  We just use
 * the skbuffer directly.
 */
#define FEC_ENET_RX_PAGES       8
#define FEC_ENET_RX_FRSIZE      2048
#define FEC_ENET_RX_FRPPG       (PAGE_SIZE / FEC_ENET_RX_FRSIZE)
#define FEC_ENET_TX_FRSIZE      2048
#define FEC_ENET_TX_FRPPG       (PAGE_SIZE / FEC_ENET_TX_FRSIZE)
#define TX_RING_SIZE            16      /* Must be power of two */
#define TX_RING_MOD_MASK        15      /*   for this to work */

#if (((RX_RING_SIZE + TX_RING_SIZE) * 8) > PAGE_SIZE)
#error "FEC: descriptor ring size constants too large"
#endif

/* Interrupt events/masks */
#define FEC_ENET_HBERR	((uint)0x80000000)	/* Heartbeat error */
#define FEC_ENET_BABR	((uint)0x40000000)	/* Babbling receiver */
#define FEC_ENET_BABT	((uint)0x20000000)	/* Babbling transmitter */
#define FEC_ENET_GRA	((uint)0x10000000)	/* Graceful stop complete */
#define FEC_ENET_TXF	((uint)0x08000000)	/* Full frame transmitted */
#define FEC_ENET_TXB	((uint)0x04000000)	/* A buffer was transmitted */
#define FEC_ENET_RXF	((uint)0x02000000)	/* Full frame received */
#define FEC_ENET_RXB	((uint)0x01000000)	/* A buffer was received */
#define FEC_ENET_MII	((uint)0x00800000)	/* MII interrupt */
#define FEC_ENET_EBERR	((uint)0x00400000)	/* SDMA bus error */

/* FEC MII MMFR bits definition */
#define FEC_MMFR_ST             (1 << 30)
#define FEC_MMFR_OP_READ        (2 << 28)
#define FEC_MMFR_OP_WRITE       (1 << 28)
#define FEC_MMFR_PA(v)          ((v & 0x1f) << 23)
#define FEC_MMFR_RA(v)          ((v & 0x1f) << 18)
#define FEC_MMFR_TA             (2 << 16)
#define FEC_MMFR_DATA(v)        (v & 0xffff)

#define FEC_MII_TIMEOUT		1000

static struct mii_bus *fec_mii_bus;

static int switch_enet_open(struct net_device *dev);
static int switch_enet_start_xmit(struct sk_buff *skb, struct net_device *dev);
static irqreturn_t switch_enet_interrupt(int irq, void *dev_id);
static void switch_enet_tx(struct net_device *dev);
static void switch_enet_rx(struct net_device *dev);
static int switch_enet_close(struct net_device *dev);
static void set_multicast_list(struct net_device *dev);
static void switch_restart(struct net_device *dev, int duplex);
static void switch_stop(struct net_device *dev);

#define		NMII	20

/* Transmitter timeout */
#define TX_TIMEOUT (2*HZ)

#ifdef CONFIG_ARCH_MXS
static void *swap_buffer(void *bufaddr, int len)
{
	int i;
	unsigned int *buf = bufaddr;

	for (i = 0; i < (len + 3) / 4; i++, buf++)
		*buf = __swab32(*buf);

	return bufaddr;
}
#endif

/*last read entry from learning interface*/
struct eswPortInfo g_info;

#ifdef USE_DEFAULT_SWITCH_PORT0_MAC
static unsigned char    switch_mac_default[] = {
	0x00, 0x08, 0x02, 0x6B, 0xA3, 0x1A,
};
#else
static unsigned char    switch_mac_default[ETH_ALEN];
#endif

static void switch_request_intrs(struct net_device *dev,
	irqreturn_t switch_net_irq_handler(int irq, void *private),
	void *irq_privatedata)
{
	struct switch_enet_private *fep;

	fep = netdev_priv(dev);

	/* Setup interrupt handlers */
	if (request_irq(dev->irq,
		switch_net_irq_handler, IRQF_DISABLED,
		"mxs-l2switch", irq_privatedata) != 0)
		printk(KERN_ERR "FEC: Could not alloc %s IRQ(%d)!\n",
			dev->name, dev->irq);
}

static void switch_set_mii(struct net_device *dev)
{
	struct switch_enet_private *fep = netdev_priv(dev);
	struct switch_t *fecp;

	fecp = (struct switch_t *)fep->hwp;

	writel(MCF_FEC_RCR_PROM | MCF_FEC_RCR_RMII_MODE |
			MCF_FEC_RCR_MAX_FL(1522) | MCF_FEC_RCR_CRC_FWD,
			fep->enet_addr + MCF_FEC_RCR0);
	writel(MCF_FEC_RCR_PROM | MCF_FEC_RCR_RMII_MODE |
			MCF_FEC_RCR_MAX_FL(1522) | MCF_FEC_RCR_CRC_FWD,
			fep->enet_addr + MCF_FEC_RCR1);
	/* TCR */
	writel(MCF_FEC_TCR_FDEN, fep->enet_addr + MCF_FEC_TCR0);
	writel(MCF_FEC_TCR_FDEN, fep->enet_addr + MCF_FEC_TCR1);

	/* ECR */
#ifdef L2SWITCH_ENHANCED_BUFFER
	writel(MCF_FEC_ECR_ETHER_EN | MCF_FEC_ECR_ENA_1588,
			fep->enet_addr + MCF_FEC_ECR0);
	writel(MCF_FEC_ECR_ETHER_EN | MCF_FEC_ECR_ENA_1588,
			fep->enet_addr + MCF_FEC_ECR1);
#else /*legac buffer*/
	writel(MCF_FEC_ECR_ETHER_EN, fep->enet_addr + MCF_FEC_ECR0);
	writel(MCF_FEC_ECR_ETHER_EN, fep->enet_addr + MCF_FEC_ECR1);
#endif

	writel(FEC_ENET_TXF | FEC_ENET_RXF | FEC_ENET_MII,
			fep->enet_addr + MCF_FEC_EIMR0);
	writel(FEC_ENET_TXF | FEC_ENET_RXF | FEC_ENET_MII,
			fep->enet_addr + MCF_FEC_EIMR1);

	/*
	* Set MII speed to 2.5 MHz
	*/
	writel(DIV_ROUND_UP(clk_get_rate(fep->clk), 5000000) << 1,
			fep->enet_addr + MCF_FEC_MSCR0);
	writel(DIV_ROUND_UP(clk_get_rate(fep->clk), 5000000) << 1,
			fep->enet_addr + MCF_FEC_MSCR1);

#ifdef CONFIG_ARCH_MXS
	/* Can't get phy(8720) ID when set to 2.5M on MX28, lower it*/
	fep->phy_speed = readl(fep->enet_addr + MCF_FEC_MSCR0) << 2;
	writel(fep->phy_speed, fep->enet_addr + MCF_FEC_MSCR0);
	writel(fep->phy_speed, fep->enet_addr + MCF_FEC_MSCR1);
#endif

}

static void switch_get_mac(struct net_device *dev)
{
	struct switch_enet_private *fep = netdev_priv(dev);
	unsigned char *iap, tmpaddr[ETH_ALEN];
	static int index;
#ifdef CONFIG_M5272
	if (FEC_FLASHMAC) {
		/*
		 * Get MAC address from FLASH.
		 * If it is all 1's or 0's, use the default.
		 */
		iap = (unsigned char *)FEC_FLASHMAC;
		if ((iap[0] == 0) && (iap[1] == 0) && (iap[2] == 0) &&
		    (iap[3] == 0) && (iap[4] == 0) && (iap[5] == 0))
			iap = switch_mac_default;
		if ((iap[0] == 0xff) && (iap[1] == 0xff) && (iap[2] == 0xff) &&
		    (iap[3] == 0xff) && (iap[4] == 0xff) && (iap[5] == 0xff))
			iap = switch_mac_default;
	}
#else
	if (is_valid_ether_addr(switch_mac_default))
		iap = switch_mac_default;
#endif
	else {
		*((unsigned long *) &tmpaddr[0]) =
			be32_to_cpu(readl(fep->enet_addr
			+ FEC_ADDR_LOW / sizeof(unsigned long)));
		*((unsigned short *) &tmpaddr[4]) =
			be16_to_cpu(readl(fep->enet_addr
			+ FEC_ADDR_HIGH / sizeof(unsigned long)) >> 16);
		iap = &tmpaddr[0];
	}

	memcpy(dev->dev_addr, iap, ETH_ALEN);

	/* Adjust MAC if using default MAC address */
	if (iap == switch_mac_default) {
		dev->dev_addr[ETH_ALEN-1] =
			switch_mac_default[ETH_ALEN-1] + index;
		index++;
	}
}

static void switch_enable_phy_intr(void)
{
}

static void switch_disable_phy_intr(void)
{
}

static void switch_phy_ack_intr(void)
{
}

static void switch_localhw_setup(void)
{
}

static void switch_uncache(unsigned long addr)
{
}

static void switch_platform_flush_cache(void)
{
}

/*
 * Calculate Galois Field Arithmetic CRC for Polynom x^8+x^2+x+1.
 * It omits the final shift in of 8 zeroes a "normal" CRC would do
 * (getting the remainder).
 *
 *  Examples (hexadecimal values):<br>
 *   10-11-12-13-14-15  => CRC=0xc2
 *   10-11-cc-dd-ee-00  => CRC=0xe6
 *
 *   param: pmacaddress
 *          A 6-byte array with the MAC address.
 *          The first byte is the first byte transmitted
 *   return The 8-bit CRC in bits 7:0
 */
static int crc8_calc(unsigned char *pmacaddress)
{
	/* byte index */
	int byt;
	/* bit index */
	int bit;
	int inval;
	int crc;
	/* preset */
	crc   = 0x12;
	for (byt = 0; byt < 6; byt++) {
		inval = (((int)pmacaddress[byt]) & 0xff);
		/*
		 * shift bit 0 to bit 8 so all our bits
		 * travel through bit 8
		 * (simplifies below calc)
		 */
		inval <<= 8;

		for (bit = 0; bit < 8; bit++) {
			/* next input bit comes into d7 after shift */
			crc |= inval & 0x100;
			if (crc & 0x01)
				/* before shift  */
				crc ^= 0x1c0;

			crc >>= 1;
			inval >>= 1;
		}

	}
	/* upper bits are clean as we shifted in zeroes! */
	return crc;
}

static void read_atable(struct switch_enet_private *fep,
	int index,
	unsigned long *read_lo, unsigned long *read_hi)
{
	unsigned long atable_base = (unsigned long)fep->hwentry;

	*read_lo = readl(atable_base + (index<<3));
	*read_hi = readl(atable_base + (index<<3) + 4);
}

static void write_atable(struct switch_enet_private *fep,
	int index,
	unsigned long write_lo, unsigned long write_hi)
{
	unsigned long atable_base = (unsigned long)fep->hwentry;

	writel(write_lo, atable_base + (index<<3));
	writel(write_hi, atable_base + (index<<3) + 4);
}

/* Read one element from the HW receive FIFO (Queue)
 * if available and return it.
 * return ms_HwPortInfo or null if no data is available
 */
static struct eswPortInfo *esw_portinfofifo_read(
	struct switch_enet_private *fep)
{
	struct switch_t  *fecp;
	unsigned long tmp;

	fecp = fep->hwp;
	if (readl(&fecp->ESW_LSR) == 0) {
		printk(KERN_ERR "%s: ESW_LSR = %lx\n",
			__func__, readl(&fecp->ESW_LSR));
		return NULL;
	}
	/* read word from FIFO */
	g_info.maclo = readl(&fecp->ESW_LREC0);
	/* but verify that we actually did so
	 * (0=no data available)
	 */
	if (g_info.maclo == 0) {
		printk(KERN_ERR "%s: mac lo %x\n",
			__func__, g_info.maclo);
		return NULL;
	}
	/* read 2nd word from FIFO */
	tmp = readl(&fecp->ESW_LREC1);
	g_info.machi = tmp & 0xffff;
	g_info.hash  = (tmp >> 16) & 0xff;
	g_info.port  = (tmp >> 24) & 0xf;

	return &g_info;
}


/*
 * Clear complete MAC Look Up Table
 */
static void esw_clear_atable(struct switch_enet_private *fep)
{
	int index;
	for (index = 0; index < 2048; index++)
		write_atable(fep, index, 0, 0);
}

/*
 * pdates MAC address lookup table with a static entry
 * Searches if the MAC address is already there in the block and replaces
 * the older entry with new one. If MAC address is not there then puts a
 * new entry in the first empty slot available in the block
 *
 * mac_addr Pointer to the array containing MAC address to
 *          be put as static entry
 * port     Port bitmask numbers to be added in static entry,
 *          valid values are 1-7
 * priority Priority for the static entry in table
 *
 * return 0 for a successful update else -1  when no slot available
 */
static int esw_update_atable_static(unsigned char *mac_addr,
	unsigned int port, unsigned int priority,
	struct switch_enet_private *fep)
{
	unsigned long block_index, entry, index_end;

	unsigned long read_lo, read_hi;
	unsigned long write_lo, write_hi;

	write_lo = (unsigned long)((mac_addr[3] << 24) |
			(mac_addr[2] << 16) |
			(mac_addr[1] << 8) |
			mac_addr[0]);
	write_hi = (unsigned long)(0 |
			(port << AT_SENTRY_PORTMASK_shift) |
			(priority << AT_SENTRY_PRIO_shift) |
			(AT_ENTRY_TYPE_STATIC << AT_ENTRY_TYPE_shift) |
			(AT_ENTRY_RECORD_VALID << AT_ENTRY_VALID_shift) |
			(mac_addr[5] << 8) | (mac_addr[4]));

	block_index = GET_BLOCK_PTR(crc8_calc(mac_addr));
	index_end = block_index + ATABLE_ENTRY_PER_SLOT;
	/* Now search all the entries in the selected block */
	for (entry = block_index; entry < index_end; entry++) {
		read_atable(fep, entry, &read_lo, &read_hi);
		/*
		 * MAC address matched, so update the
		 * existing entry
		 * even if its a dynamic one
		 */
		if ((read_lo == write_lo) &&
			((read_hi & 0x0000ffff) ==
			 (write_hi & 0x0000ffff))) {
			write_atable(fep, entry, write_lo, write_hi);
			return 0;
		} else if (!(read_hi & (1 << 16))) {
			/*
			 * Fill this empty slot (valid bit zero),
			 * assuming no holes in the block
			 */
			write_atable(fep, entry, write_lo, write_hi);
			fep->atCurrEntries++;
			return 0;
		}
	}

	/* No space available for this static entry */
	return -1;
}

static int esw_update_atable_dynamic1(unsigned long write_lo,
	unsigned long write_hi, int block_index,
	unsigned int port, unsigned int currTime,
	struct switch_enet_private *fep)
{
	unsigned long entry, index_end;
	unsigned long read_lo, read_hi;
	unsigned long tmp;
	int time, timeold, indexold;

	/* prepare update port and timestamp */
	tmp = AT_ENTRY_RECORD_VALID << AT_ENTRY_VALID_shift;
	tmp |= AT_ENTRY_TYPE_DYNAMIC << AT_ENTRY_TYPE_shift;
	tmp |= currTime << AT_DENTRY_TIME_shift;
	tmp |= port << AT_DENTRY_PORT_shift;
	tmp |= write_hi;

	/*
	* linear search through all slot
	* entries and update if found
	*/
	index_end = block_index + ATABLE_ENTRY_PER_SLOT;
	/* Now search all the entries in the selected block */
	for (entry = block_index; entry < index_end; entry++) {
		read_atable(fep, entry, &read_lo, &read_hi);
		if ((read_lo == write_lo) &&
			((read_hi & 0x0000ffff) ==
			(write_hi & 0x0000ffff))) {
			/* found correct address,
			 * update timestamp.
			 */
			write_atable(fep, entry, write_lo, tmp);
			return 0;
		} else if (!(read_hi & (1 << 16))) {
			/* slot is empty, then use it
			 * for new entry
			 * Note: There are no holes,
			 * therefore cannot be any
			 * more that need to be compared.
			 */
			write_atable(fep, entry, write_lo, tmp);
			/* statistics (we do it between writing
			 *  .hi an .lo due to
			 * hardware limitation...
			 */
			fep->atCurrEntries++;
			/* newly inserted */
			return 1;
		}
	}

	/*
	 * no more entry available in block ...
	 * overwrite oldest
	 */
	timeold = 0;
	indexold = 0;
	for (entry = block_index; entry < index_end; entry++) {
		read_atable(fep, entry, &read_lo, &read_hi);
		time = AT_EXTRACT_TIMESTAMP(read_hi);
		printk(KERN_ERR "%s : time %x currtime %x\n",
			__func__, time, currTime);
		time = TIMEDELTA(currTime, time);
		if (time > timeold) {
			/* is it older ? */
			timeold = time;
			indexold = entry;
		}
	}

	write_atable(fep, indexold, write_lo, tmp);
	/*
	 * Statistics (do it inbetween
	 *writing to .lo and .hi
	 */
	fep->atBlockOverflows++;
	printk(KERN_ERR "%s update time, atBlockOverflows %x\n",
		__func__, fep->atBlockOverflows);
	/* newly inserted */
	return 1;
}

/* dynamicms MAC address table learn and migration */
static int esw_atable_dynamicms_learn_migration(
	struct switch_enet_private *fep,
	int currTime)
{
	struct eswPortInfo *pESWPortInfo;
	int index;
	int inserted = 0;

	pESWPortInfo = esw_portinfofifo_read(fep);
	/* Anything to learn */
	if (pESWPortInfo != 0) {
		/* get block index from lookup table */
		index = GET_BLOCK_PTR(pESWPortInfo->hash);
		inserted = esw_update_atable_dynamic1(
			pESWPortInfo->maclo,
			pESWPortInfo->machi, index,
			pESWPortInfo->port, currTime, fep);
	} else {
		printk(KERN_ERR "%s:hav invalidate learned data \n", __func__);
		return -1;
	}

	return 0;

}

/*
 * esw_forced_forward
 * The frame is forwared to the forced destination ports.
 * It only replace the MAC lookup function,
 * all other filtering(eg.VLAN verification) act as normal
 */
static int esw_forced_forward(struct switch_enet_private *fep,
	int port1, int port2, int enable)
{
	unsigned long tmp = 0;
	struct switch_t  *fecp;

	fecp = fep->hwp;

	/* Enable Forced forwarding for port num */
	if ((port1 == 1) && (port2 == 1))
		tmp |= MCF_ESW_P0FFEN_FD(3);
	else if (port1 == 1)
		/* Enable Forced forwarding for port 1 only */
		tmp |= MCF_ESW_P0FFEN_FD(1);
	else if (port2 == 1)
		/* Enable Forced forwarding for port 2 only */
		tmp |= MCF_ESW_P0FFEN_FD(2);
	else {
		printk(KERN_ERR "%s:do not support "
			"the forced forward mode"
			"port1 %x port2 %x\n",
			__func__, port1, port2);
		return -1;
	}

	if (enable == 1)
		tmp |= MCF_ESW_P0FFEN_FEN;
	else if (enable == 0)
		tmp &= ~MCF_ESW_P0FFEN_FEN;
	else {
		printk(KERN_ERR "%s: the enable %x is error\n",
			__func__, enable);
		return -2;
	}

	writel(tmp, &fecp->ESW_P0FFEN);
	return 0;
}

static int esw_get_forced_forward(
	struct switch_enet_private *fep,
	unsigned long *ulForceForward)
{
	struct switch_t  *fecp;

	fecp = fep->hwp;
	*ulForceForward = readl(&fecp->ESW_P0FFEN);
#ifdef DEBUG_FORCED_FORWARD
	printk(KERN_INFO "%s  ESW_P0FFEN %#lx\n",
		__func__, readl(&fecp->ESW_P0FFEN));
#endif
	return 0;
}

static void esw_get_port_enable(
	struct switch_enet_private *fep,
	unsigned long *ulPortEnable)
{
	struct switch_t  *fecp;

	fecp = fep->hwp;
	*ulPortEnable = readl(&fecp->ESW_PER);
#ifdef DEBUG_PORT_ENABLE
	printk(KERN_INFO "%s  fecp->ESW_PER %#lx\n",
		__func__, readl(&fecp->ESW_PER));
#endif
}
/*
 * enable or disable port n tx or rx
 * tx_en 0 disable port n tx
 * tx_en 1 enable  port n tx
 * rx_en 0 disbale port n rx
 * rx_en 1 enable  port n rx
 */
static int esw_port_enable_config(struct switch_enet_private *fep,
	int port, int tx_en, int rx_en)
{
	unsigned long tmp = 0;
	struct switch_t  *fecp;

	fecp = fep->hwp;
	tmp = fecp->ESW_PER;
	if (tx_en == 1) {
		if (port == 0)
			tmp |= MCF_ESW_PER_TE0;
		else if (port == 1)
			tmp |= MCF_ESW_PER_TE1;
		else if (port == 2)
			tmp |= MCF_ESW_PER_TE2;
		else {
			printk(KERN_ERR "%s:do not support the"
				" port %x tx enable %d\n",
				__func__, port, tx_en);
			return -1;
		}
	} else if (tx_en == 0) {
		if (port == 0)
			tmp &= (~MCF_ESW_PER_TE0);
		else if (port == 1)
			tmp &= (~MCF_ESW_PER_TE1);
		else if (port == 2)
			tmp &= (~MCF_ESW_PER_TE2);
		else {
			printk(KERN_ERR "%s:do not support "
				"the port %x tx disable %d\n",
				__func__, port, tx_en);
			return -2;
		}
	} else {
		printk(KERN_ERR "%s:do not support the port %x"
			" tx op value %x\n",
			__func__, port, tx_en);
		return -3;
	}

	if (rx_en == 1) {
		if (port == 0)
			tmp |= MCF_ESW_PER_RE0;
		else if (port == 1)
			tmp |= MCF_ESW_PER_RE1;
		else if (port == 2)
			tmp |= MCF_ESW_PER_RE2;
		else {
			printk(KERN_ERR "%s:do not support the "
				"port %x rx enable %d\n",
				__func__, port, tx_en);
			return -4;
		}
	} else if (rx_en == 0) {
		if (port == 0)
			tmp &= (~MCF_ESW_PER_RE0);
		else if (port == 1)
			tmp &= (~MCF_ESW_PER_RE1);
		else if (port == 2)
			tmp &= (~MCF_ESW_PER_RE2);
		else {
			printk(KERN_ERR "%s:do not support the "
				"port %x rx disable %d\n",
				__func__, port, rx_en);
			return -5;
		}
	} else {
		printk(KERN_ERR "%s:do not support the port %x"
			" rx op value %x\n",
			__func__, port, tx_en);
		return -6;
	}

	writel(tmp, &fecp->ESW_PER);
	return 0;
}


static void esw_get_port_broadcast(
	struct switch_enet_private *fep,
	unsigned long *ulPortBroadcast)
{
	struct switch_t  *fecp;

	fecp = fep->hwp;
	*ulPortBroadcast = readl(&fecp->ESW_DBCR);
#ifdef DEBUG_PORT_BROADCAST
	printk(KERN_INFO "%s  fecp->ESW_DBCR %#lx\n",
		__func__, readl(&fecp->ESW_DBCR));
#endif
}

static int esw_port_broadcast_config(
	struct switch_enet_private *fep,
	int port, int enable)
{
	unsigned long tmp = 0;
	struct switch_t  *fecp;

	fecp = fep->hwp;

	if ((port > 2) || (port < 0)) {
		printk(KERN_ERR "%s:do not support the port %x"
			" default broadcast\n",
			__func__, port);
		return -1;
	}

	tmp = readl(&fecp->ESW_DBCR);
	if (enable == 1) {
		if (port == 0)
			tmp |= MCF_ESW_DBCR_P0;
		else if (port == 1)
			tmp |= MCF_ESW_DBCR_P1;
		else if (port == 2)
			tmp |= MCF_ESW_DBCR_P2;
	} else if (enable == 0) {
		if (port == 0)
			tmp &= ~MCF_ESW_DBCR_P0;
		else if (port == 1)
			tmp &= ~MCF_ESW_DBCR_P1;
		else if (port == 2)
			tmp &= ~MCF_ESW_DBCR_P2;
	}

	writel(tmp, &fecp->ESW_DBCR);
	return 0;
}


static void esw_get_port_multicast(
	struct switch_enet_private *fep,
	unsigned long *ulPortMulticast)
{
	struct switch_t  *fecp;

	fecp = fep->hwp;
	*ulPortMulticast = readl(&fecp->ESW_DMCR);
#ifdef DEBUG_PORT_MULTICAST
	printk(KERN_INFO "%s  fecp->ESW_DMCR %#lx\n",
		__func__, readl(&fecp->ESW_DMCR));
#endif
}

static int esw_port_multicast_config(
	struct switch_enet_private *fep,
	int port, int enable)
{
	unsigned long tmp = 0;
	struct switch_t  *fecp;

	fecp = fep->hwp;

	if ((port > 2) || (port < 0)) {
		printk(KERN_ERR "%s:do not support the port %x"
			" default broadcast\n",
			__func__, port);
		return -1;
	}

	tmp = readl(&fecp->ESW_DMCR);
	if (enable == 1) {
		if (port == 0)
			tmp |= MCF_ESW_DMCR_P0;
		else if (port == 1)
			tmp |= MCF_ESW_DMCR_P1;
		else if (port == 2)
			tmp |= MCF_ESW_DMCR_P2;
	} else if (enable == 0) {
		if (port == 0)
			tmp &= ~MCF_ESW_DMCR_P0;
		else if (port == 1)
			tmp &= ~MCF_ESW_DMCR_P1;
		else if (port == 2)
			tmp &= ~MCF_ESW_DMCR_P2;
	}

	writel(tmp, &fecp->ESW_DMCR);
	return 0;
}


static void esw_get_port_blocking(
	struct switch_enet_private *fep,
	unsigned long *ulPortBlocking)
{
	struct switch_t  *fecp;

	fecp = fep->hwp;
	*ulPortBlocking = (readl(&fecp->ESW_BKLR) & 0x00ff);
#ifdef DEBUG_PORT_BLOCKING
	printk(KERN_INFO "%s  fecp->ESW_BKLR %#lx\n",
		__func__, readl(&fecp->ESW_BKLR));
#endif
}

static int esw_port_blocking_config(
	struct switch_enet_private *fep,
	int port, int enable)
{
	unsigned long tmp = 0;
	struct switch_t  *fecp;

	fecp = fep->hwp;

	if ((port > 2) || (port < 0)) {
		printk(KERN_ERR "%s:do not support the port %x"
			" default broadcast\n",
			__func__, port);
		return -1;
	}

	tmp = readl(&fecp->ESW_BKLR);
	if (enable == 1) {
		if (port == 0)
			tmp |= MCF_ESW_BKLR_BE0;
		else if (port == 1)
			tmp |= MCF_ESW_BKLR_BE1;
		else if (port == 2)
			tmp |= MCF_ESW_BKLR_BE2;
	} else if (enable == 0) {
		if (port == 0)
			tmp &= ~MCF_ESW_BKLR_BE0;
		else if (port == 1)
			tmp &= ~MCF_ESW_BKLR_BE1;
		else if (port == 2)
			tmp &= ~MCF_ESW_BKLR_BE2;
	}

	writel(tmp, &fecp->ESW_BKLR);
	return 0;
}


static void esw_get_port_learning(
	struct switch_enet_private *fep,
	unsigned long *ulPortLearning)
{
	struct switch_t  *fecp;

	fecp = fep->hwp;
	*ulPortLearning = (readl(&fecp->ESW_BKLR) & 0xff0000) >> 16;
#ifdef DEBUG_PORT_LEARNING
	printk(KERN_INFO "%s  fecp->ESW_BKLR %#lx\n",
		__func__, readl(&fecp->ESW_BKLR));
#endif
}

static int esw_port_learning_config(
	struct switch_enet_private *fep,
	int port, int disable)
{
	unsigned long tmp = 0;
	struct switch_t  *fecp;

	fecp = fep->hwp;

	if ((port > 2) || (port < 0)) {
		printk(KERN_ERR "%s:do not support the port %x"
			" default broadcast\n",
			__func__, port);
		return -1;
	}

	tmp = readl(&fecp->ESW_BKLR);
	if (disable == 1) {
		fep->learning_irqhandle_enable = 0;
		fecp->switch_imask &= ~MCF_ESW_IMR_LRN;
		if (port == 0)
			tmp |= MCF_ESW_BKLR_LD0;
		else if (port == 1)
			tmp |= MCF_ESW_BKLR_LD1;
		else if (port == 2)
			tmp |= MCF_ESW_BKLR_LD2;
	} else if (disable == 0) {
		fep->learning_irqhandle_enable = 1;
		fecp->switch_imask |= MCF_ESW_IMR_LRN;
		if (port == 0)
			tmp &= ~MCF_ESW_BKLR_LD0;
		else if (port == 1)
			tmp &= ~MCF_ESW_BKLR_LD1;
		else if (port == 2)
			tmp &= ~MCF_ESW_BKLR_LD2;
	}

	writel(tmp, &fecp->ESW_BKLR);
#ifdef DEBUG_PORT_LEARNING
	printk(KERN_INFO "%s  ESW_BKLR %#lx, switch_imask %#lx\n",
		__func__, readl(&fecp->ESW_BKLR), readl(&fecp->switch_imask));
#endif
	return 0;
}

/*
 * Checks IP Snoop options of handling the snooped frame.
 * mode 0 : The snooped frame is forward only to management port
 * mode 1 : The snooped frame is copy to management port and
 *              normal forwarding is checked.
 * mode 2 : The snooped frame is discarded.
 * mode 3 : Disable the ip snoop function
 * ip_header_protocol : the IP header protocol field
 */
static int esw_ip_snoop_config(struct switch_enet_private *fep,
	int num, int mode, unsigned long ip_header_protocol)
{
	struct switch_t  *fecp;
	unsigned long tmp = 0, protocol_type = 0;

	fecp = fep->hwp;
	/* Config IP Snooping */
	if (mode == 0) {
		/* Enable IP Snooping */
		tmp = MCF_ESW_IPSNP_EN;
		tmp |= MCF_ESW_IPSNP_MODE(0);/*For Forward*/
	} else if (mode == 1) {
		/* Enable IP Snooping */
		tmp = MCF_ESW_IPSNP_EN;
		/*For Forward and copy_to_mangmnt_port*/
		tmp |= MCF_ESW_IPSNP_MODE(1);
	} else if (mode == 2) {
		/* Enable IP Snooping */
		tmp = MCF_ESW_IPSNP_EN;
		tmp |= MCF_ESW_IPSNP_MODE(2);/*discard*/
	} else if (mode == 3) {
		/* disable IP Snooping */
		tmp = MCF_ESW_IPSNP_EN;
		tmp &= ~MCF_ESW_IPSNP_EN;
	} else {
		printk(KERN_ERR "%s: the mode %x "
			"we do not support\n", __func__, mode);
		return -1;
	}

	protocol_type = ip_header_protocol;
	writel(tmp | MCF_ESW_IPSNP_PROTOCOL(protocol_type),
		 &fecp->ESW_IPSNP[num]);
	printk(KERN_INFO "%s : ESW_IPSNP[%d] %#lx\n",
		__func__, num, readl(&fecp->ESW_IPSNP[num]));
	return 0;
}

static void esw_get_ip_snoop_config(
	struct switch_enet_private *fep,
	unsigned long *ulpESW_IPSNP)
{
	int i;
	struct switch_t  *fecp;

	fecp = fep->hwp;
	for (i = 0; i < 8; i++)
		*(ulpESW_IPSNP + i) = readl(&fecp->ESW_IPSNP[i]);
#ifdef DEBUG_IP_SNOOP
	printk(KERN_INFO "%s  ", __func__);
	for (i = 0; i < 8; i++)
		printk(KERN_INFO " reg(%d) %#lx", readl(&fecp->ESW_IPSNP[i]));
	printk(KERN_INFO "\n");
#endif

}
/*
 * Checks TCP/UDP Port Snoop options of handling the snooped frame.
 * mode 0 : The snooped frame is forward only to management port
 * mode 1 : The snooped frame is copy to management port and
 *              normal forwarding is checked.
 * mode 2 : The snooped frame is discarded.
 * compare_port : port number in the TCP/UDP header
 * compare_num 1: TCP/UDP source port number is compared
 * compare_num 2: TCP/UDP destination port number is compared
 * compare_num 3: TCP/UDP source and destination port number is compared
 */
static int esw_tcpudp_port_snoop_config(struct switch_enet_private *fep,
	int num, int mode, int compare_port, int compare_num)
{
	struct switch_t  *fecp;
	unsigned long tmp = 0;

	fecp = fep->hwp;

	/* Enable TCP/UDP port Snooping */
	tmp = MCF_ESW_PSNP_EN;
	if (mode == 0)
		tmp |= MCF_ESW_PSNP_MODE(0);/* For Forward */
	else if (mode == 1)/*For Forward and copy_to_mangmnt_port*/
		tmp |= MCF_ESW_PSNP_MODE(1);
	else if (mode == 2)
		tmp |= MCF_ESW_PSNP_MODE(2);/* discard */
	else if (mode == 3) /* disable the port function */
		tmp &= (~MCF_ESW_PSNP_EN);
	else {
		printk(KERN_ERR "%s: the mode %x we do not support\n",
			__func__, mode);
		return -1;
	}

	if (compare_num == 1)
		tmp |= MCF_ESW_PSNP_CS;
	else if (compare_num == 2)
		tmp |= MCF_ESW_PSNP_CD;
	else if (compare_num == 3)
		tmp |= MCF_ESW_PSNP_CD | MCF_ESW_PSNP_CS;
	else {
		printk(KERN_ERR "%s: the compare port address %x"
			" we do not support\n",
			__func__, compare_num);
		return -1;
	}

	writel(tmp | MCF_ESW_PSNP_PORT_COMPARE(compare_port),
		 &fecp->ESW_PSNP[num]);
	printk(KERN_INFO "ESW_PSNP[%d] %#lx\n",
			num, fecp->ESW_PSNP[num]);
	return 0;
}

static void esw_get_tcpudp_port_snoop_config(
	struct switch_enet_private *fep,
	unsigned long *ulpESW_PSNP)
{
	int i;
	struct switch_t  *fecp;

	fecp = fep->hwp;
	for (i = 0; i < 8; i++)
		*(ulpESW_PSNP + i) = readl(&fecp->ESW_PSNP[i]);
#ifdef DEBUG_TCPUDP_PORT_SNOOP
	 printk(KERN_INFO "%s  ", __func__);
	 for (i = 0; i < 8; i++)
		printk(KERN_INFO " reg(%d) %#lx", readl(fecp->ESW_PSNP[i]));
	 printk(KERN_INFO "\n");
#endif

}

static void esw_get_port_mirroring(
	struct switch_enet_private *fep,
	struct eswIoctlPortMirrorStatus *pPortMirrorStatus)
{
	struct switch_t  *fecp;

	fecp = fep->hwp;
	pPortMirrorStatus->ESW_MCR    = readl(&fecp->ESW_MCR);
	pPortMirrorStatus->ESW_EGMAP  = readl(&fecp->ESW_EGMAP);
	pPortMirrorStatus->ESW_INGMAP = readl(&fecp->ESW_INGMAP);
	pPortMirrorStatus->ESW_INGSAL = readl(&fecp->ESW_INGSAL);
	pPortMirrorStatus->ESW_INGSAH = readl(&fecp->ESW_INGSAH);
	pPortMirrorStatus->ESW_INGDAL = readl(&fecp->ESW_INGDAL);
	pPortMirrorStatus->ESW_INGDAH = readl(&fecp->ESW_INGDAH);
	pPortMirrorStatus->ESW_ENGSAL = readl(&fecp->ESW_ENGSAL);
	pPortMirrorStatus->ESW_ENGSAH = readl(&fecp->ESW_ENGSAH);
	pPortMirrorStatus->ESW_ENGDAL = readl(&fecp->ESW_ENGDAL);
	pPortMirrorStatus->ESW_ENGDAH = readl(&fecp->ESW_ENGDAH);
	pPortMirrorStatus->ESW_MCVAL  = readl(&fecp->ESW_MCVAL);
#ifdef DEBUG_PORT_MIRROR
	printk(KERN_INFO "%s : ESW_MCR %#lx, ESW_EGMAP %#lx\n"
		"ESW_INGMAP %#lx, ESW_INGSAL %#lx, "
		"ESW_INGSAH %#lx ESW_INGDAL %#lx, ESW_INGDAH %#lx\n"
		"ESW_ENGSAL %#lx, ESW_ENGSAH%#lx, ESW_ENGDAL %#lx,"
		"ESW_ENGDAH %#lx, ESW_MCVAL %#lx\n",
		__func__, readl(&fecp->ESW_MCR),
		readl(&fecp->ESW_EGMAP),
		readl(&fecp->ESW_INGMAP),
		readl(&fecp->ESW_INGSAL),
		readl(&fecp->ESW_INGSAH),
		readl(&fecp->ESW_INGDAL),
		readl(&fecp->ESW_INGDAH),
		readl(&fecp->ESW_ENGSAL),
		readl(&fecp->ESW_ENGSAH),
		readl(&fecp->ESW_ENGDAL),
		readl(&fecp->ESW_ENGDAH),
		readl(&fecp->ESW_MCVAL));
#endif
}

static int esw_port_mirroring_config(struct switch_enet_private *fep,
	int mirror_port, int port, int mirror_enable,
	unsigned char *src_mac, unsigned char *des_mac,
	int egress_en, int ingress_en,
	int egress_mac_src_en, int egress_mac_des_en,
	int ingress_mac_src_en, int ingress_mac_des_en)
{
	struct switch_t  *fecp;
	unsigned long tmp = 0;

	fecp = fep->hwp;

	/*mirroring config*/
	tmp = 0;
	if (egress_en == 1) {
		tmp |= MCF_ESW_MCR_EGMAP;
		if (port == 0)
			fecp->ESW_EGMAP = MCF_ESW_EGMAP_EG0;
		else if (port == 1)
			fecp->ESW_EGMAP = MCF_ESW_EGMAP_EG1;
		else if (port == 2)
			fecp->ESW_EGMAP = MCF_ESW_EGMAP_EG2;
		else {
			printk(KERN_ERR "%s: the port %x we do not support\n",
					__func__, port);
			return -1;
		}
	} else if (egress_en == 0) {
		tmp &= (~MCF_ESW_MCR_EGMAP);
	} else {
		printk(KERN_ERR "%s: egress_en %x we do not support\n",
			__func__, egress_en);
		return -1;
	}

	if (ingress_en == 1) {
		tmp |= MCF_ESW_MCR_INGMAP;
		if (port == 0)
			fecp->ESW_INGMAP = MCF_ESW_INGMAP_ING0;
		else if (port == 1)
			fecp->ESW_INGMAP = MCF_ESW_INGMAP_ING1;
		else if (port == 2)
			fecp->ESW_INGMAP = MCF_ESW_INGMAP_ING2;
		else {
			printk(KERN_ERR "%s: the port %x we do not support\n",
				__func__, port);
			return -1;
		}
	} else if (ingress_en == 0) {
		tmp &= ~MCF_ESW_MCR_INGMAP;
	} else{
		printk(KERN_ERR "%s: ingress_en %x we do not support\n",
				__func__, ingress_en);
		return -1;
	}

	if (egress_mac_src_en == 1) {
		tmp |= MCF_ESW_MCR_EGSA;
		fecp->ESW_ENGSAH = (src_mac[5] << 8) | (src_mac[4]);
		fecp->ESW_ENGSAL = (unsigned long)((src_mac[3] << 24) |
					(src_mac[2] << 16) |
					(src_mac[1] << 8) |
					src_mac[0]);
	} else if (egress_mac_src_en == 0) {
		tmp &= ~MCF_ESW_MCR_EGSA;
	} else {
		printk(KERN_ERR "%s: egress_mac_src_en  %x we do not support\n",
			__func__, egress_mac_src_en);
		return -1;
	}

	if (egress_mac_des_en == 1) {
		tmp |= MCF_ESW_MCR_EGDA;
		fecp->ESW_ENGDAH = (des_mac[5] << 8) | (des_mac[4]);
		fecp->ESW_ENGDAL = (unsigned long)((des_mac[3] << 24) |
					(des_mac[2] << 16) |
					(des_mac[1] << 8) |
					des_mac[0]);
	} else if (egress_mac_des_en == 0) {
		tmp &= ~MCF_ESW_MCR_EGDA;
	} else {
		printk(KERN_ERR "%s: egress_mac_des_en  %x we do not support\n",
			__func__, egress_mac_des_en);
		return -1;
	}

	if (ingress_mac_src_en == 1) {
		tmp |= MCF_ESW_MCR_INGSA;
		fecp->ESW_INGSAH = (src_mac[5] << 8) | (src_mac[4]);
		fecp->ESW_INGSAL = (unsigned long)((src_mac[3] << 24) |
					(src_mac[2] << 16) |
					(src_mac[1] << 8) |
					src_mac[0]);
	} else if (ingress_mac_src_en == 0) {
		tmp &= ~MCF_ESW_MCR_INGSA;
	} else {
		printk(KERN_ERR "%s: ingress_mac_src_en  %x we do not support\n",
			__func__, ingress_mac_src_en);
		return -1;
	}

	if (ingress_mac_des_en == 1) {
		tmp |= MCF_ESW_MCR_INGDA;
		fecp->ESW_INGDAH = (des_mac[5] << 8) | (des_mac[4]);
		fecp->ESW_INGDAL = (unsigned long)((des_mac[3] << 24) |
					(des_mac[2] << 16) |
					(des_mac[1] << 8) |
					des_mac[0]);
	} else if (ingress_mac_des_en == 0) {
		tmp &= ~MCF_ESW_MCR_INGDA;
	} else {
		printk(KERN_ERR "%s: ingress_mac_des_en  %x we do not support\n",
			__func__, ingress_mac_des_en);
		return -1;
	}

	/*------------------------------------------------------------------*/
	if (mirror_enable == 1)
		tmp |= MCF_ESW_MCR_MEN | MCF_ESW_MCR_PORT(mirror_port);
	else if (mirror_enable == 0)
		tmp &= ~MCF_ESW_MCR_MEN;
	else
		printk(KERN_ERR "%s: the mirror enable %x is error\n",
			__func__, mirror_enable);


	writel(tmp, &fecp->ESW_MCR);
	printk(KERN_INFO "%s : MCR %#lx, EGMAP %#lx, INGMAP %#lx;\n"
		"ENGSAH %#lx, ENGSAL %#lx ;ENGDAH %#lx, ENGDAL %#lx;\n"
		"INGSAH %#lx, INGSAL %#lx\n;INGDAH %#lx, INGDAL %#lx;\n",
		__func__, readl(fecp->ESW_MCR),
		readl(&fecp->ESW_EGMAP),
		readl(&fecp->ESW_INGMAP),
		readl(&fecp->ESW_ENGSAH),
		readl(&fecp->ESW_ENGSAL),
		readl(&fecp->ESW_ENGDAH),
		readl(&fecp->ESW_ENGDAL),
		readl(&fecp->ESW_INGSAH),
		readl(&fecp->ESW_INGSAL),
		readl(&fecp->ESW_INGDAH),
		readl(&fecp->ESW_INGDAL));
	return 0;
}

static void esw_get_vlan_verification(
	struct switch_enet_private *fep,
	unsigned long *ulValue)
{
	struct switch_t  *fecp;
	fecp = fep->hwp;
	*ulValue = fecp->ESW_VLANV;

#ifdef DEBUG_VLAN_VERIFICATION_CONFIG
	printk(KERN_INFO "%s: ESW_VLANV %#lx\n",
		__func__, readl(&fecp->ESW_VLANV));
#endif
}

static int esw_set_vlan_verification(
	struct switch_enet_private *fep, int port,
	int vlan_domain_verify_en,
	int vlan_discard_unknown_en)
{
	struct switch_t  *fecp;

	fecp = fep->hwp;
	if ((port < 0) || (port > 2)) {
		printk(KERN_ERR "%s: do not support the port %d\n",
			__func__, port);
		return -1;
	}

	if (vlan_domain_verify_en == 1) {
		if (port == 0)
			writel(readl(&fecp->ESW_VLANV) | MCF_ESW_VLANV_VV0,
				&fecp->ESW_VLANV);
		else if (port == 1)
			writel(readl(&fecp->ESW_VLANV) | MCF_ESW_VLANV_VV1,
				&fecp->ESW_VLANV);
		else if (port == 2)
			writel(readl(&fecp->ESW_VLANV) | MCF_ESW_VLANV_VV2,
				&fecp->ESW_VLANV);
	} else if (vlan_domain_verify_en == 0) {
		if (port == 0)
			writel(readl(&fecp->ESW_VLANV) & ~MCF_ESW_VLANV_VV0,
				&fecp->ESW_VLANV);
		else if (port == 1)
			writel(readl(&fecp->ESW_VLANV) & ~MCF_ESW_VLANV_VV1,
				&fecp->ESW_VLANV);
		else if (port == 2)
			writel(readl(&fecp->ESW_VLANV) & ~MCF_ESW_VLANV_VV2,
				&fecp->ESW_VLANV);
	} else {
		printk(KERN_INFO "%s: donot support "
			"vlan_domain_verify %x\n",
			__func__, vlan_domain_verify_en);
		return -2;
	}

	if (vlan_discard_unknown_en == 1) {
		if (port == 0)
			writel(readl(&fecp->ESW_VLANV) | MCF_ESW_VLANV_DU0,
				&fecp->ESW_VLANV);
		else if (port == 1)
			writel(readl(&fecp->ESW_VLANV) | MCF_ESW_VLANV_DU1,
				&fecp->ESW_VLANV);
		else if (port == 2)
			writel(readl(&fecp->ESW_VLANV) | MCF_ESW_VLANV_DU2,
				&fecp->ESW_VLANV);
	} else if (vlan_discard_unknown_en == 0) {
		if (port == 0)
			writel(readl(&fecp->ESW_VLANV) & ~MCF_ESW_VLANV_DU0,
				&fecp->ESW_VLANV);
		else if (port == 1)
			writel(readl(&fecp->ESW_VLANV) & ~MCF_ESW_VLANV_DU1,
				&fecp->ESW_VLANV);
		else if (port == 2)
			writel(readl(&fecp->ESW_VLANV) & ~MCF_ESW_VLANV_DU2,
				&fecp->ESW_VLANV);
	} else {
		printk(KERN_INFO "%s: donot support "
			"vlan_discard_unknown %x\n",
			__func__, vlan_discard_unknown_en);
		return -3;
	}

#ifdef DEBUG_VLAN_VERIFICATION_CONFIG
	printk(KERN_INFO "%s: ESW_VLANV %#lx\n",
		__func__, fecp->ESW_VLANV);
#endif
	return 0;
}

static void esw_get_vlan_resolution_table(
	struct switch_enet_private *fep,
	int vlan_domain_num,
	unsigned long *ulValue)
{
	struct switch_t  *fecp;
	fecp = fep->hwp;

	*ulValue = fecp->ESW_VRES[vlan_domain_num];

#ifdef DEBUG_VLAN_DOMAIN_TABLE
	printk(KERN_INFO "%s: ESW_VRES[%d] = %#lx\n",
		__func__, vlan_domain_num,
		fecp->ESW_VRES[vlan_domain_num]);
#endif
}

int esw_set_vlan_resolution_table(
	struct switch_enet_private *fep,
	unsigned short port_vlanid,
	int vlan_domain_num,
	int vlan_domain_port)
{
	struct switch_t  *fecp;

	fecp = fep->hwp;
	if ((vlan_domain_num < 0)
		|| (vlan_domain_num > 31)) {
		printk(KERN_ERR "%s: do not support the "
			"vlan_domain_num %d\n",
		__func__, vlan_domain_num);
		return -1;
	}

	if ((vlan_domain_port < 0)
		|| (vlan_domain_port > 7)) {
		printk(KERN_ERR "%s: do not support the "
			"vlan_domain_port %d\n",
			__func__, vlan_domain_port);
		return -2;
	}

	writel(MCF_ESW_VRES_VLANID(port_vlanid) | vlan_domain_port,
		&fecp->ESW_VRES[vlan_domain_num]);

#ifdef DEBUG_VLAN_DOMAIN_TABLE
	printk(KERN_INFO "%s: ESW_VRES[%d] = %#lx\n",
		__func__, vlan_domain_num,
		readl(&fecp->ESW_VRES[vlan_domain_num]));
#endif
	return 0;
}

static void esw_get_vlan_input_config(
	struct switch_enet_private *fep,
	struct eswIoctlVlanInputStatus *pVlanInputConfig)
{
	struct switch_t  *fecp;
	int i;

	fecp = fep->hwp;
	for (i = 0; i < 3; i++)
		pVlanInputConfig->ESW_PID[i] = readl(&fecp->ESW_PID[i]);

	pVlanInputConfig->ESW_VLANV  = readl(&fecp->ESW_VLANV);
	pVlanInputConfig->ESW_VIMSEL = readl(&fecp->ESW_VIMSEL);
	pVlanInputConfig->ESW_VIMEN  = readl(&fecp->ESW_VIMEN);

	for (i = 0; i < 32; i++)
		pVlanInputConfig->ESW_VRES[i] = readl(&fecp->ESW_VRES[i]);
#ifdef DEBUG_VLAN_INTPUT_CONFIG
	printk(KERN_INFO "%s: ESW_VLANV %#lx, ESW_VIMSEL %#lx, "
		"ESW_VIMEN %#lx, ESW_PID[0], ESW_PID[1] %#lx, "
		"ESW_PID[2] %#lx", __func__,
		readl(&fecp->ESW_VLANV),
		readl(&fecp->ESW_VIMSEL),
		readl(&fecp->ESW_VIMEN),
		readl(&fecp->ESW_PID[0]),
		readl(&fecp->ESW_PID[1]),
		readl(&fecp->ESW_PID[2]);
#endif
}


static int esw_vlan_input_process(struct switch_enet_private *fep,
	int port, int mode, unsigned short port_vlanid,
	int vlan_verify_en, int vlan_domain_num,
	int vlan_domain_port)
{
	struct switch_t  *fecp;

	fecp = fep->hwp;

	/*we only support mode1 mode2 mode3 mode4*/
	if ((mode < 0) || (mode > 3)) {
		printk(KERN_ERR "%s: do not support the"
			" VLAN input processing mode %d\n",
			__func__, mode);
		return -1;
	}

	if ((port < 0) || (port > 3)) {
		printk(KERN_ERR "%s: do not support the port %d\n",
			__func__, mode);
		return -2;
	}

	if ((vlan_verify_en == 1) && ((vlan_domain_num < 0)
		|| (vlan_domain_num > 32))) {
		printk(KERN_ERR "%s: do not support the port %d\n",
			__func__, mode);
		return -3;
	}

	fecp->ESW_PID[port] = MCF_ESW_PID_VLANID(port_vlanid);
	if (port == 0) {
		if (vlan_verify_en == 1)
			writel(MCF_ESW_VRES_VLANID(port_vlanid) | MCF_ESW_VRES_P0,
				&fecp->ESW_VRES[vlan_domain_num]);

		writel(readl(&fecp->ESW_VIMEN) | MCF_ESW_VIMEN_EN0,
			&fecp->ESW_VIMEN);
		writel(readl(&fecp->ESW_VIMSEL) | MCF_ESW_VIMSEL_IM0(mode),
			&fecp->ESW_VIMSEL);
	} else if (port == 1) {
		if (vlan_verify_en == 1)
			writel(MCF_ESW_VRES_VLANID(port_vlanid)	| MCF_ESW_VRES_P1,
				&fecp->ESW_VRES[vlan_domain_num]);

		writel(readl(&fecp->ESW_VIMEN) | MCF_ESW_VIMEN_EN1,
			&fecp->ESW_VIMEN);
		writel(readl(&fecp->ESW_VIMSEL) | MCF_ESW_VIMSEL_IM1(mode),
			&fecp->ESW_VIMSEL);
	} else if (port == 2) {
		if (vlan_verify_en == 1)
			writel(MCF_ESW_VRES_VLANID(port_vlanid) | MCF_ESW_VRES_P2,
				&fecp->ESW_VRES[vlan_domain_num]);

		writel(readl(&fecp->ESW_VIMEN) | MCF_ESW_VIMEN_EN2,
			&fecp->ESW_VIMEN);
		writel(readl(&fecp->ESW_VIMSEL) | MCF_ESW_VIMSEL_IM2(mode),
			&fecp->ESW_VIMSEL);
	} else {
		printk(KERN_ERR "%s: do not support the port %d\n",
			__func__, port);
		return -2;
	}

	return 0;
}

static void esw_get_vlan_output_config(struct switch_enet_private *fep,
	unsigned long *ulVlanOutputConfig)
{
	struct switch_t  *fecp;

	fecp = fep->hwp;

	*ulVlanOutputConfig = readl(&fecp->ESW_VOMSEL);
#ifdef DEBUG_VLAN_OUTPUT_CONFIG
	printk(KERN_INFO "%s: ESW_VOMSEL %#lx", __func__,
			fecp->ESW_VOMSEL);
#endif
}

static int esw_vlan_output_process(struct switch_enet_private *fep,
	int port, int mode)
{
	struct switch_t  *fecp;

	fecp = fep->hwp;

	if ((port < 0) || (port > 2)) {
		printk(KERN_ERR "%s: do not support the port %d\n",
			__func__, mode);
		return -1;
	}

	if (port == 0) {
		writel(readl(&fecp->ESW_VOMSEL) | MCF_ESW_VOMSEL_OM0(mode),
			&fecp->ESW_VOMSEL);
	} else if (port == 1) {
		writel(readl(&fecp->ESW_VOMSEL) | MCF_ESW_VOMSEL_OM1(mode),
			&fecp->ESW_VOMSEL);
	} else if (port == 2) {
		writel(readl(&fecp->ESW_VOMSEL) | MCF_ESW_VOMSEL_OM2(mode),
			&fecp->ESW_VOMSEL);;
	} else {
		printk(KERN_ERR "%s: do not support the port %d\n",
			__func__, port);
		return -1;
	}
	return 0;
}

/* frame calssify and priority resolution */
/* vlan priority lookup */
static int esw_framecalssify_vlan_priority_lookup(
	struct switch_enet_private *fep,
	int port, int func_enable,
	int vlan_pri_table_num,
	int vlan_pri_table_value)
{
	struct switch_t  *fecp;

	fecp = fep->hwp;

	if ((port < 0) || (port > 3)) {
		printk(KERN_ERR "%s: do not support the port %d\n",
			__func__, port);
		return -1;
	}

	if (func_enable == 0) {
		writel(readl(&fecp->ESW_PRES[port]) & ~MCF_ESW_PRES_VLAN,
			&fecp->ESW_PRES[port]);
		printk(KERN_ERR "%s: disable port %d VLAN priority "
			"lookup function\n", __func__, port);
		return 0;
	}

	if ((vlan_pri_table_num < 0) || (vlan_pri_table_num > 7)) {
		printk(KERN_ERR "%s: do not support the priority %d\n",
			__func__, vlan_pri_table_num);
		return -1;
	}

	writel(readl(&fecp->ESW_PVRES[port]) | (((vlan_pri_table_value & 0x3)
		<< (vlan_pri_table_num*3))),
		&fecp->ESW_PVRES[port]);
	/* enable port  VLAN priority lookup function */
	writel(readl(&fecp->ESW_PRES[port]) | MCF_ESW_PRES_VLAN,
		&fecp->ESW_PRES[port]);

	return 0;
}

static int esw_framecalssify_ip_priority_lookup(
	struct switch_enet_private *fep,
	int port, int func_enable, int ipv4_en,
	int ip_priority_num,
	int ip_priority_value)
{
	struct switch_t  *fecp;
	unsigned long tmp = 0, tmp_prio = 0;

	fecp = fep->hwp;

	if ((port < 0) || (port > 3)) {
		printk(KERN_ERR "%s: do not support the port %d\n",
			__func__, port);
		return -1;
	}

	if (func_enable == 0) {
		writel(readl(&fecp->ESW_PRES[port]) & (~MCF_ESW_PRES_IP),
			&fecp->ESW_PRES[port]);
		printk(KERN_ERR "%s: disable port %d ip priority "
			"lookup function\n", __func__, port);
		return 0;
	}

	/* IPV4 priority 64 entry table lookup */
	/* IPv4 head 6 bit TOS field */
	if (ipv4_en == 1) {
		if ((ip_priority_num < 0) || (ip_priority_num > 63)) {
			printk(KERN_ERR "%s: do not support the table entry %d\n",
				__func__, ip_priority_num);
			return -2;
		}
	} else { /* IPV6 priority 256 entry table lookup */
		/* IPv6 head 8 bit COS field */
		if ((ip_priority_num < 0) || (ip_priority_num > 255)) {
			printk(KERN_ERR "%s: do not support the table entry %d\n",
				__func__, ip_priority_num);
			return -3;
		}
	}

	/* IP priority  table lookup : address */
	tmp = MCF_ESW_IPRES_ADDRESS(ip_priority_num);
	/* IP priority  table lookup : ipv4sel */
	if (ipv4_en == 1)
		tmp = tmp | MCF_ESW_IPRES_IPV4SEL;
	/* IP priority  table lookup : priority */
	if (port == 0)
		tmp |= MCF_ESW_IPRES_PRI0(ip_priority_value);
	else if (port == 1)
		tmp |= MCF_ESW_IPRES_PRI1(ip_priority_value);
	else if (port == 2)
		tmp |= MCF_ESW_IPRES_PRI2(ip_priority_value);

	/* configure */
	writel(MCF_ESW_IPRES_READ | MCF_ESW_IPRES_ADDRESS(ip_priority_num),
		&fecp->ESW_IPRES);
	tmp_prio = readl(&fecp->ESW_IPRES);

	writel(tmp | tmp_prio, &fecp->ESW_IPRES);

	writel(MCF_ESW_IPRES_READ |
		MCF_ESW_IPRES_ADDRESS(ip_priority_num),
		&fecp->ESW_IPRES);
	tmp_prio = readl(&fecp->ESW_IPRES);

	/* enable port  IP priority lookup function */
	writel(MCF_ESW_PRES_IP, &fecp->ESW_PRES[port]);

	return 0;
}

static int esw_framecalssify_mac_priority_lookup(
	struct switch_enet_private *fep,
	int port)
{
	struct switch_t  *fecp;

	if ((port < 0) || (port > 3)) {
		printk(KERN_ERR "%s: do not support the port %d\n",
			__func__, port);
		return -1;
	}

	fecp = fep->hwp;
	writel(readl(&fecp->ESW_PRES[port]) | MCF_ESW_PRES_MAC,
		&fecp->ESW_PRES[port]);

	return 0;
}

static int esw_frame_calssify_priority_init(
	struct switch_enet_private *fep,
	int port, unsigned char priority_value)
{
	struct switch_t  *fecp;

	fecp = fep->hwp;

	if ((port < 0) || (port > 3)) {
		printk(KERN_ERR "%s: do not support the port %d\n",
			__func__, port);
		return -1;
	}
	/* disable all priority lookup function */
	writel(0, &fecp->ESW_PRES[port]);
	writel(MCF_ESW_PRES_DFLT_PRI(priority_value & 0x7),
		&fecp->ESW_PRES[port]);

	return 0;
}

static int esw_get_statistics_status(
	struct switch_enet_private *fep,
	struct esw_statistics_status *pStatistics)
{
	struct switch_t  *fecp;
	fecp = fep->hwp;

	pStatistics->ESW_DISCN   = readl(&fecp->ESW_DISCN);
	pStatistics->ESW_DISCB   = readl(&fecp->ESW_DISCB);
	pStatistics->ESW_NDISCN  = readl(&fecp->ESW_NDISCN);
	pStatistics->ESW_NDISCB  = readl(&fecp->ESW_NDISCB);
#ifdef DEBUG_STATISTICS
	printk(KERN_ERR "%s:ESW_DISCN %#lx, ESW_DISCB %#lx,"
		"ESW_NDISCN %#lx, ESW_NDISCB %#lx\n",
		__func__,
		readl(&fecp->ESW_DISCN),
		readl(&fecp->ESW_DISCB),
		readl(&fecp->ESW_NDISCN),
		readl(&fecp->ESW_NDISCB));
#endif
	return 0;
}

static int esw_get_port_statistics_status(
	struct switch_enet_private *fep,
	int port,
	struct esw_port_statistics_status *pPortStatistics)
{
	struct switch_t  *fecp;

	if ((port < 0) || (port > 3)) {
		printk(KERN_ERR "%s: do not support the port %d\n",
			__func__, port);
		return -1;
	}

	fecp = fep->hwp;

	pPortStatistics->MCF_ESW_POQC   =
		readl(&fecp->port_statistics_status[port].MCF_ESW_POQC);
	pPortStatistics->MCF_ESW_PMVID  =
		readl(&fecp->port_statistics_status[port].MCF_ESW_PMVID);
	pPortStatistics->MCF_ESW_PMVTAG =
		readl(&fecp->port_statistics_status[port].MCF_ESW_PMVTAG);
	pPortStatistics->MCF_ESW_PBL    =
		readl(&fecp->port_statistics_status[port].MCF_ESW_PBL);
#ifdef DEBUG_PORT_STATISTICS
	printk(KERN_ERR "%s : port[%d].MCF_ESW_POQC %#lx, MCF_ESW_PMVID %#lx,"
		" MCF_ESW_PMVTAG %#lx, MCF_ESW_PBL %#lx\n",
		__func__, port,
		readl(&fecp->port_statistics_status[port].MCF_ESW_POQC),
		readl(&fecp->port_statistics_status[port].MCF_ESW_PMVID),
		readl(&fecp->port_statistics_status[port].MCF_ESW_PMVTAG),
		readl(&fecp->port_statistics_status[port].MCF_ESW_PBL));
#endif
	return 0;
}

static int esw_get_output_queue_status(
	struct switch_enet_private *fep,
	struct esw_output_queue_status *pOutputQueue)
{
	struct switch_t  *fecp;

	fecp = fep->hwp;
	pOutputQueue->ESW_MMSR  = readl(&fecp->ESW_MMSR);
	pOutputQueue->ESW_LMT   = readl(&fecp->ESW_LMT);
	pOutputQueue->ESW_LFC   = readl(&fecp->ESW_LFC);
	pOutputQueue->ESW_IOSR  = readl(&fecp->ESW_IOSR);
	pOutputQueue->ESW_PCSR  = readl(&fecp->ESW_PCSR);
	pOutputQueue->ESW_QWT   = readl(&fecp->ESW_QWT);
	pOutputQueue->ESW_P0BCT = readl(&fecp->ESW_P0BCT);
#ifdef DEBUG_OUTPUT_QUEUE
	printk(KERN_ERR "%s:ESW_MMSR %#lx, ESW_LMT %#lx, ESW_LFC %#lx, "
		"ESW_IOSR %#lx, ESW_PCSR %#lx, ESW_QWT %#lx, ESW_P0BCT %#lx\n",
		__func__,
		readl(&fecp->ESW_MMSR),
		readl(&fecp->ESW_LMT),
		readl(&fecp->ESW_LFC),
		readl(&fecp->ESW_IOSR),
		readl(&fecp->ESW_PCSR),
		readl(&fecp->ESW_QWT),
		readl(&fecp->ESW_P0BCT));
#endif
	return 0;
}

/* set output queue memory status and configure*/
static int esw_set_output_queue_memory(
	struct switch_enet_private *fep,
	int fun_num,
	struct esw_output_queue_status *pOutputQueue)
{
	struct switch_t  *fecp;

	fecp = fep->hwp;

	if (fun_num == 1) {
		/* memory manager status*/
		writel(pOutputQueue->ESW_MMSR, &fecp->ESW_MMSR);
	} else if (fun_num == 2) {
		/*low memory threshold*/
		writel(pOutputQueue->ESW_LMT, &fecp->ESW_LMT);
	} else if (fun_num == 3) {
		/*lowest number of free cells*/
		writel(pOutputQueue->ESW_LFC, &fecp->ESW_LFC);
	} else if (fun_num == 4) {
		/*queue weights*/
		writel(pOutputQueue->ESW_QWT, &fecp->ESW_QWT);
	} else if (fun_num == 5) {
		/*port 0 backpressure congenstion thresled*/
		writel(pOutputQueue->ESW_P0BCT, &fecp->ESW_P0BCT);
	} else {
		printk(KERN_INFO "%s: do not support the cmd %x\n",
			__func__, fun_num);
		return -1;
	}
#ifdef DEBUG_OUTPUT_QUEUE
	printk(KERN_ERR "%s:ESW_MMSR %#lx, ESW_LMT %#lx, ESW_LFC %#lx, "
		"ESW_IOSR %#lx, ESW_PCSR %#lx, ESW_QWT %#lx, ESW_P0BCT %#lx\n",
		__func__,
		readl(&fecp->ESW_MMSR),
		readl(&fecp->ESW_LMT),
		readl(&fecp->ESW_LFC),
		readl(&fecp->ESW_IOSR),
		readl(&fecp->ESW_PCSR),
		readl(&fecp->ESW_QWT),
		readl(&fecp->ESW_P0BCT));
#endif
	return 0;
}

int esw_set_irq_mask(
	struct switch_enet_private *fep,
	unsigned long mask, int enable)
{
	struct switch_t  *fecp;

	fecp = fep->hwp;
#ifdef DEBUG_IRQ
	printk(KERN_INFO "%s: irq event %#lx, irq mask %#lx "
		" mask %x, enable %x\n",
		__func__,
		readl(&fecp->switch_ievent),
		readl(&fecp->switch_imask), mask, enable);
#endif
	if (enable == 1)
		writel(readl(&fecp->switch_imask) | mask,
			&fecp->switch_imask);
	else if (enable == 1)
		writel(readl(&fecp->switch_imask) & (~mask),
			&fecp->switch_imask);
	else {
		printk(KERN_INFO "%s: enable %x is error value\n",
			__func__, enable);
		return -1;
	}
#ifdef DEBUG_IRQ
	printk(KERN_INFO "%s: irq event %#lx, irq mask %#lx, "
		"rx_des_start %#lx, tx_des_start %#lx, "
		"rx_buff_size %#lx, rx_des_active %#lx, "
		"tx_des_active %#lx\n",
		__func__,
		readl(&fecp->switch_ievent),
		readl(&fecp->switch_imask),
		readl(&fecp->fec_r_des_start),
		readl(&fecp->fec_x_des_start),
		readl(&fecp->fec_r_buff_size),
		readl(&fecp->fec_r_des_active),
		readl(&fecp->fec_x_des_active));
#endif
	return 0;
}

static void esw_get_switch_mode(
	struct switch_enet_private *fep,
	unsigned long *ulModeConfig)
{
	struct switch_t  *fecp;

	fecp = fep->hwp;
	*ulModeConfig = readl(&fecp->ESW_MODE);
#ifdef DEBUG_SWITCH_MODE
	printk(KERN_INFO "%s: mode %#lx \n",
		__func__, readl(&fecp->ESW_MODE));
#endif
}

static void esw_switch_mode_configure(
	struct switch_enet_private *fep,
	unsigned long configure)
{
	struct switch_t  *fecp;

	fecp = fep->hwp;
	writel(readl(&fecp->ESW_MODE) | configure, &fecp->ESW_MODE);
#ifdef DEBUG_SWITCH_MODE
	printk(KERN_INFO "%s: mode %#lx \n",
		__func__, reald(&fecp->ESW_MODE));
#endif
}


static void esw_get_bridge_port(
	struct switch_enet_private *fep,
	unsigned long *ulBMPConfig)
{
	struct switch_t  *fecp;

	fecp = fep->hwp;
	*ulBMPConfig = readl(&fecp->ESW_BMPC);
#ifdef DEBUG_BRIDGE_PORT
	printk(KERN_INFO "%s: bridge management port %#lx \n",
		__func__, readl(&fecp->ESW_BMPC));
#endif
}

static void  esw_bridge_port_configure(
	struct switch_enet_private *fep,
	unsigned long configure)
{
	struct switch_t  *fecp;

	fecp = fep->hwp;
	writel(readl(&fecp->ESW_BMPC) | configure,
		&fecp->ESW_BMPC);
#ifdef DEBUG_BRIDGE_PORT
	printk(KERN_INFO "%s: bridge management port %#lx \n",
		__func__, fecp->ESW_BMPC);
#endif
}

/* The timer should create an interrupt every 4 seconds*/
static void l2switch_aging_timer(unsigned long data)
{
	struct switch_enet_private *fep;

	fep = (struct switch_enet_private *)data;

	if (fep) {
		TIMEINCREMENT(fep->currTime);
		fep->timeChanged++;
	}

	mod_timer(&fep->timer_aging, jiffies + LEARNING_AGING_TIMER);
}

void esw_check_rxb_txb_interrupt(struct switch_enet_private *fep)
{
	struct switch_t  *fecp;

	fecp = fep->hwp;

	/*Enable Forced forwarding for port 1*/
	writel(MCF_ESW_P0FFEN_FEN | MCF_ESW_P0FFEN_FD(1),
		&fecp->ESW_P0FFEN);
	/*Disable learning for all ports*/

	writel(MCF_ESW_IMR_TXB | MCF_ESW_IMR_TXF |
		MCF_ESW_IMR_LRN | MCF_ESW_IMR_RXB | MCF_ESW_IMR_RXF,
		&fecp->switch_imask);
	printk(KERN_ERR "%s: fecp->ESW_DBCR %#lx, fecp->ESW_P0FFEN %#lx"
		" fecp->ESW_BKLR %#lx\n", __func__, fecp->ESW_DBCR,
		readl(&fecp->ESW_P0FFEN),
		readl(&fecp->ESW_BKLR));
}

static int esw_mac_addr_static(struct switch_enet_private *fep)
{
	struct switch_t  *fecp;

	fecp = fep->hwp;
	writel(MCF_ESW_DBCR_P1, &fecp->ESW_DBCR);

	if (is_valid_ether_addr(fep->netdev->dev_addr))
		esw_update_atable_static(fep->netdev->dev_addr, 7, 7, fep);
	else{
		printk(KERN_ERR "Can not add available mac address"
				" for switch!!\n");
		return -EFAULT;
	}

	return 0;
}

static void esw_main(struct switch_enet_private *fep)
{
	struct switch_t  *fecp;
	fecp = fep->hwp;

	esw_mac_addr_static(fep);
	writel(0, &fecp->ESW_BKLR);
	writel(MCF_ESW_IMR_TXB | MCF_ESW_IMR_TXF |
		MCF_ESW_IMR_LRN | MCF_ESW_IMR_RXB | MCF_ESW_IMR_RXF,
		&fecp->switch_imask);
	writel(0x70007, &fecp->ESW_PER);
	writel(MCF_ESW_DBCR_P1 | MCF_ESW_DBCR_P2, &fecp->ESW_DBCR);
}

static int switch_enet_ioctl(
	struct net_device *dev,
	struct ifreq *ifr, int cmd)
{
	struct switch_enet_private *fep;
	struct switch_t       *fecp;
	int ret = 0;

	printk(KERN_INFO "%s cmd %x\n", __func__, cmd);
	fep = netdev_priv(dev);
	fecp = (struct switch_t *)dev->base_addr;

	switch (cmd) {
	case ESW_SET_PORTENABLE_CONF:
	{
		struct eswIoctlPortEnableConfig configData;
		ret = copy_from_user(&configData,
			ifr->ifr_data,
			sizeof(struct eswIoctlPortEnableConfig));
		if (ret)
			return -EFAULT;

		ret = esw_port_enable_config(fep,
			configData.port,
			configData.tx_enable,
			configData.rx_enable);
	}
		break;
	case ESW_SET_BROADCAST_CONF:
	{
		struct eswIoctlPortConfig configData;
		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(struct eswIoctlPortConfig));
		if (ret)
			return -EFAULT;

		ret = esw_port_broadcast_config(fep,
			configData.port, configData.enable);
	}
		break;

	case ESW_SET_MULTICAST_CONF:
	{
		struct eswIoctlPortConfig configData;
		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(struct eswIoctlPortConfig));
		if (ret)
			return -EFAULT;

		ret = esw_port_multicast_config(fep,
			configData.port, configData.enable);
	}
		break;

	case ESW_SET_BLOCKING_CONF:
	{
		struct eswIoctlPortConfig configData;
		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(struct eswIoctlPortConfig));

		if (ret)
			return -EFAULT;

		ret = esw_port_blocking_config(fep,
			configData.port, configData.enable);
	}
		break;

	case ESW_SET_LEARNING_CONF:
	{
		struct eswIoctlPortConfig configData;
		printk(KERN_INFO "ESW_SET_LEARNING_CONF\n");
		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(struct eswIoctlPortConfig));
		if (ret)
			return -EFAULT;
		printk(KERN_INFO "ESW_SET_LEARNING_CONF: %x %x\n",
				configData.port, configData.enable);
		ret = esw_port_learning_config(fep,
			configData.port, configData.enable);
	}
		break;

	case ESW_SET_IP_SNOOP_CONF:
	{
		struct eswIoctlIpsnoopConfig configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(struct eswIoctlIpsnoopConfig));
		if (ret)
			return -EFAULT;
		printk(KERN_INFO "ESW_SET_IP_SNOOP_CONF:: %x %x %x\n",
				configData.num, configData.mode,
				configData.ip_header_protocol);
		ret = esw_ip_snoop_config(fep,
			configData.num, configData.mode,
			configData.ip_header_protocol);
	}
		break;

	case ESW_SET_PORT_SNOOP_CONF:
	{
		struct eswIoctlPortsnoopConfig configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(struct eswIoctlPortsnoopConfig));
		if (ret)
			return -EFAULT;
		printk(KERN_INFO "ESW_SET_PORT_SNOOP_CONF:: %x %x %x %x\n",
			configData.num, configData.mode,
			configData.compare_port, configData.compare_num);
		ret = esw_tcpudp_port_snoop_config(fep,
			configData.num, configData.mode,
			configData.compare_port,
			configData.compare_num);
	}
		break;

	case ESW_SET_PORT_MIRROR_CONF:
	{
		struct eswIoctlPortMirrorConfig configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(struct eswIoctlPortMirrorConfig));
		if (ret)
			return -EFAULT;
		printk(KERN_INFO "ESW_SET_PORT_MIRROR_CONF:: %x %x %x "
			"%s %s\n %x %x %x %x %x %x\n",
			configData.mirror_port, configData.port,
			configData.mirror_enable,
			configData.src_mac, configData.des_mac,
			configData.egress_en, configData.ingress_en,
			configData.egress_mac_src_en,
			configData.egress_mac_des_en,
			configData.ingress_mac_src_en,
			configData.ingress_mac_des_en);
		ret = esw_port_mirroring_config(fep,
			configData.mirror_port, configData.port,
			configData.mirror_enable,
			configData.src_mac, configData.des_mac,
			configData.egress_en, configData.ingress_en,
			configData.egress_mac_src_en,
			configData.egress_mac_des_en,
			configData.ingress_mac_src_en,
			configData.ingress_mac_des_en);
	}
		break;

	case ESW_SET_PIRORITY_VLAN:
	{
		struct eswIoctlPriorityVlanConfig configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data,
			sizeof(struct eswIoctlPriorityVlanConfig));
		if (ret)
			return -EFAULT;

		ret = esw_framecalssify_vlan_priority_lookup(fep,
			configData.port, configData.func_enable,
			configData.vlan_pri_table_num,
			configData.vlan_pri_table_value);
	}
		break;

	case ESW_SET_PIRORITY_IP:
	{
		struct eswIoctlPriorityIPConfig configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(struct eswIoctlPriorityIPConfig));
		if (ret)
			return -EFAULT;

		ret = esw_framecalssify_ip_priority_lookup(fep,
			configData.port, configData.func_enable,
			configData.ipv4_en, configData.ip_priority_num,
			configData.ip_priority_value);
	}
		break;

	case ESW_SET_PIRORITY_MAC:
	{
		struct eswIoctlPriorityMacConfig configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data,
			sizeof(struct eswIoctlPriorityMacConfig));
		if (ret)
			return -EFAULT;

		ret = esw_framecalssify_mac_priority_lookup(fep,
			configData.port);
	}
		break;

	case ESW_SET_PIRORITY_DEFAULT:
	{
		struct eswIoctlPriorityDefaultConfig configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data,
			sizeof(struct eswIoctlPriorityDefaultConfig));
		if (ret)
			return -EFAULT;

		ret = esw_frame_calssify_priority_init(fep,
			configData.port, configData.priority_value);
	}
		break;

	case ESW_SET_P0_FORCED_FORWARD:
	{
		struct eswIoctlP0ForcedForwardConfig configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data,
			sizeof(struct eswIoctlP0ForcedForwardConfig));
		if (ret)
			return -EFAULT;

		ret = esw_forced_forward(fep, configData.port1,
			configData.port2, configData.enable);
	}
		break;

	case ESW_SET_BRIDGE_CONFIG:
	{
		unsigned long configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(unsigned long));
		if (ret)
			return -EFAULT;

		esw_bridge_port_configure(fep, configData);
	}
		break;

	case ESW_SET_SWITCH_MODE:
	{
		unsigned long configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(unsigned long));
		if (ret)
			return -EFAULT;

		esw_switch_mode_configure(fep, configData);
	}
		break;

	case ESW_SET_OUTPUT_QUEUE_MEMORY:
	{
		struct eswIoctlOutputQueue configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(struct eswIoctlOutputQueue));
		if (ret)
			return -EFAULT;

		printk(KERN_INFO "ESW_SET_OUTPUT_QUEUE_MEMORY:: %#x \n"
			"%#lx %#lx %#lx %#lx\n"
			"%#lx %#lx %#lx\n",
			configData.fun_num,
			configData.sOutputQueue.ESW_MMSR,
			configData.sOutputQueue.ESW_LMT,
			configData.sOutputQueue.ESW_LFC,
			configData.sOutputQueue.ESW_PCSR,
			configData.sOutputQueue.ESW_IOSR,
			configData.sOutputQueue.ESW_QWT,
			configData.sOutputQueue.ESW_P0BCT);
		ret = esw_set_output_queue_memory(fep,
			configData.fun_num, &configData.sOutputQueue);
	}
		break;

	case ESW_SET_VLAN_OUTPUT_PROCESS:
	{
		struct eswIoctlVlanOutputConfig configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data, sizeof(struct eswIoctlVlanOutputConfig));
		if (ret)
			return -EFAULT;

		printk(KERN_INFO "ESW_SET_VLAN_OUTPUT_PROCESS: %x %x\n",
			configData.port, configData.mode);
		ret = esw_vlan_output_process(fep,
			configData.port, configData.mode);
	}
		break;

	case ESW_SET_VLAN_INPUT_PROCESS:
	{
		struct eswIoctlVlanInputConfig configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data,
			sizeof(struct eswIoctlVlanInputConfig));
		if (ret)
			return -EFAULT;

		printk(KERN_INFO "ESW_SET_VLAN_INPUT_PROCESS: %x %x"
				"%x %x %x %x\n",
			configData.port, configData.mode,
			configData.port_vlanid,
			configData.vlan_verify_en,
			configData.vlan_domain_num,
			configData.vlan_domain_port);
		ret = esw_vlan_input_process(fep, configData.port,
				configData.mode, configData.port_vlanid,
				configData.vlan_verify_en,
				configData.vlan_domain_num,
				configData.vlan_domain_port);
	}
		break;

	case ESW_SET_VLAN_DOMAIN_VERIFICATION:
	{
		struct eswIoctlVlanVerificationConfig configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data,
			sizeof(struct eswIoctlVlanVerificationConfig));
		if (ret)
			return -EFAULT;

		printk("ESW_SET_VLAN_DOMAIN_VERIFICATION: "
			"%x %x %x\n",
			configData.port,
			configData.vlan_domain_verify_en,
			configData.vlan_discard_unknown_en);
		ret = esw_set_vlan_verification(
			fep, configData.port,
			configData.vlan_domain_verify_en,
			configData.vlan_discard_unknown_en);
	}
		break;

	case ESW_SET_VLAN_RESOLUTION_TABLE:
	{
		struct eswIoctlVlanResoultionTable configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data,
			sizeof(struct eswIoctlVlanResoultionTable));
		if (ret)
			return -EFAULT;

		printk(KERN_INFO "ESW_SET_VLAN_RESOLUTION_TABLE: "
			"%x %x %x\n",
			configData.port_vlanid,
			configData.vlan_domain_num,
			configData.vlan_domain_port);

		ret = esw_set_vlan_resolution_table(
			fep, configData.port_vlanid,
			configData.vlan_domain_num,
			configData.vlan_domain_port);

	}
		break;
	case ESW_UPDATE_STATIC_MACTABLE:
	{
		struct eswIoctlUpdateStaticMACtable configData;

		ret = copy_from_user(&configData,
			ifr->ifr_data,
			sizeof(struct eswIoctlUpdateStaticMACtable));
		if (ret)
			return -EFAULT;

		printk(KERN_INFO "%s: ESW_UPDATE_STATIC_MACTABLE: mac %s, "
			"port %x, priority %x\n", __func__,
			configData.mac_addr,
			configData.port,
			configData.priority);
		ret = esw_update_atable_static(configData.mac_addr,
				configData.port, configData.priority, fep);
	}
		break;

	case ESW_CLEAR_ALL_MACTABLE:
	{
		esw_clear_atable(fep);
	}
		break;

	case ESW_GET_STATISTICS_STATUS:
	{
		struct esw_statistics_status Statistics;
		ret = esw_get_statistics_status(fep, &Statistics);
		if (ret != 0) {
			printk(KERN_ERR "%s: cmd %x fail\n",
				__func__, cmd);
			return -1;
		}

		ret = copy_to_user(ifr->ifr_data, &Statistics,
			sizeof(struct esw_statistics_status));
		if (ret)
			return -EFAULT;
	}
		break;

	case ESW_GET_PORT0_STATISTICS_STATUS:
	{
		struct esw_port_statistics_status PortStatistics;

		ret = esw_get_port_statistics_status(fep,
			0, &PortStatistics);
		if (ret != 0) {
			printk(KERN_ERR "%s: cmd %x fail\n",
				__func__, cmd);
			return -1;
		}

		ret = copy_to_user(ifr->ifr_data, &PortStatistics,
			sizeof(struct esw_port_statistics_status));
		if (ret)
			return -EFAULT;
	}
		break;

	case ESW_GET_PORT1_STATISTICS_STATUS:
	{
		struct esw_port_statistics_status PortStatistics;

		ret = esw_get_port_statistics_status(fep,
			1, &PortStatistics);
		if (ret != 0) {
			printk(KERN_ERR "%s: cmd %x fail\n",
				__func__, cmd);
			return -1;
		}

		ret = copy_to_user(ifr->ifr_data, &PortStatistics,
			sizeof(struct esw_port_statistics_status));
		if (ret)
			return -EFAULT;
	}
		break;

	case ESW_GET_PORT2_STATISTICS_STATUS:
	{
		struct esw_port_statistics_status PortStatistics;

		ret = esw_get_port_statistics_status(fep,
			2, &PortStatistics);
		if (ret != 0) {
			printk(KERN_ERR "%s: cmd %x fail\n",
				__func__, cmd);
			return -1;
		}

		ret = copy_to_user(ifr->ifr_data, &PortStatistics,
			sizeof(struct esw_port_statistics_status));
		if (ret)
			return -EFAULT;
	}
		break;

	case ESW_GET_LEARNING_CONF:
	{
		unsigned long PortLearning;

		esw_get_port_learning(fep, &PortLearning);
		ret = copy_to_user(ifr->ifr_data, &PortLearning,
			sizeof(unsigned long));
		if (ret)
			return -EFAULT;
	}
		break;

	case ESW_GET_BLOCKING_CONF:
	{
		unsigned long PortBlocking;

		esw_get_port_blocking(fep, &PortBlocking);
		ret = copy_to_user(ifr->ifr_data, &PortBlocking,
			sizeof(unsigned long));
		if (ret)
			return -EFAULT;
	}
		break;

	case ESW_GET_MULTICAST_CONF:
	{
		unsigned long PortMulticast;

		esw_get_port_multicast(fep, &PortMulticast);
		ret = copy_to_user(ifr->ifr_data, &PortMulticast,
			sizeof(unsigned long));
		if (ret)
			return -EFAULT;
	}
		break;

	case ESW_GET_BROADCAST_CONF:
	{
		unsigned long PortBroadcast;

		esw_get_port_broadcast(fep, &PortBroadcast);
		ret = copy_to_user(ifr->ifr_data, &PortBroadcast,
		sizeof(unsigned long));
		if (ret)
			return -EFAULT;
	}
		break;

	case ESW_GET_PORTENABLE_CONF:
	{
		unsigned long PortEnable;

		esw_get_port_enable(fep, &PortEnable);
		ret = copy_to_user(ifr->ifr_data, &PortEnable,
			sizeof(unsigned long));
		if (ret)
			return -EFAULT;
	}
		break;

	case ESW_GET_IP_SNOOP_CONF:
	{
		unsigned long ESW_IPSNP[8];

		esw_get_ip_snoop_config(fep, (unsigned long *)ESW_IPSNP);
		ret = copy_to_user(ifr->ifr_data, ESW_IPSNP,
			(8 * sizeof(unsigned long)));
		if (ret)
			return -EFAULT;
	}
		break;

	case ESW_GET_PORT_SNOOP_CONF:
	{
		unsigned long ESW_PSNP[8];

		esw_get_tcpudp_port_snoop_config(fep,
				(unsigned long *)ESW_PSNP);
		ret = copy_to_user(ifr->ifr_data, ESW_PSNP,
			(8 * sizeof(unsigned long)));
		if (ret)
			return -EFAULT;
	}
		break;

	case ESW_GET_PORT_MIRROR_CONF:
	{
		struct eswIoctlPortMirrorStatus PortMirrorStatus;

		esw_get_port_mirroring(fep, &PortMirrorStatus);
		ret = copy_to_user(ifr->ifr_data, &PortMirrorStatus,
			sizeof(struct eswIoctlPortMirrorStatus));
		if (ret)
			return -EFAULT;
	}
		break;

	case ESW_GET_P0_FORCED_FORWARD:
	{
		unsigned long ForceForward;

		esw_get_forced_forward(fep, &ForceForward);
		ret = copy_to_user(ifr->ifr_data, &ForceForward,
			sizeof(unsigned long));
		if (ret)
			return -EFAULT;
	}
		break;

	case ESW_GET_SWITCH_MODE:
	{
		unsigned long Config;

		esw_get_switch_mode(fep, &Config);
		ret = copy_to_user(ifr->ifr_data, &Config,
			sizeof(unsigned long));
		if (ret)
			return -EFAULT;
	}
		break;

	case ESW_GET_BRIDGE_CONFIG:
	{
		unsigned long Config;

		esw_get_bridge_port(fep, &Config);
		ret = copy_to_user(ifr->ifr_data, &Config,
			sizeof(unsigned long));
		if (ret)
			return -EFAULT;
	}
		break;
	case ESW_GET_OUTPUT_QUEUE_STATUS:
	{
		struct esw_output_queue_status Config;
		esw_get_output_queue_status(fep,
			&Config);
		ret = copy_to_user(ifr->ifr_data, &Config,
			sizeof(struct esw_output_queue_status));
		if (ret)
			return -EFAULT;
	}
		break;

	case ESW_GET_VLAN_OUTPUT_PROCESS:
	{
		unsigned long Config;

		esw_get_vlan_output_config(fep, &Config);
		ret = copy_to_user(ifr->ifr_data, &Config,
			sizeof(unsigned long));
		if (ret)
			return -EFAULT;
	}
		break;

	case ESW_GET_VLAN_INPUT_PROCESS:
	{
		struct eswIoctlVlanInputStatus Config;

		esw_get_vlan_input_config(fep, &Config);
		ret = copy_to_user(ifr->ifr_data, &Config,
			sizeof(struct eswIoctlVlanInputStatus));
		if (ret)
			return -EFAULT;
	}
		break;

	case ESW_GET_VLAN_RESOLUTION_TABLE:
	{
		unsigned long Config;
		unsigned char ConfigData;
		ret = copy_from_user(&ConfigData,
			ifr->ifr_data,
			sizeof(unsigned char));
		if (ret)
			return -EFAULT;

		printk(KERN_INFO "ESW_GET_VLAN_RESOLUTION_TABLE: %x \n",
			ConfigData);

		esw_get_vlan_resolution_table(fep, ConfigData, &Config);

		ret = copy_to_user(ifr->ifr_data, &Config,
			sizeof(unsigned long));
		if (ret)
			return -EFAULT;
	}
		break;

	case ESW_GET_VLAN_DOMAIN_VERIFICATION:
	{
		unsigned long Config;

		esw_get_vlan_verification(fep, &Config);
		ret = copy_to_user(ifr->ifr_data, &Config,
			sizeof(unsigned long));
		if (ret)
			return -EFAULT;
	}
		break;
	/*------------------------------------------------------------------*/
	default:
		return -EOPNOTSUPP;
	}


	return ret;
}

static int
switch_enet_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct switch_enet_private *fep;
	struct switch_t	*fecp;
	struct cbd_t	*bdp;
	void *bufaddr;
	unsigned short	status;
	unsigned long flags;

	fep = netdev_priv(dev);
	fecp = (struct switch_t *)fep->hwp;

	if (!fep->link[0] && !fep->link[1]) {
		/* Link is down or autonegotiation is in progress. */
		netif_stop_queue(dev);
		return NETDEV_TX_BUSY;
	}

	spin_lock_irqsave(&fep->hw_lock, flags);
	/* Fill in a Tx ring entry */
	bdp = fep->cur_tx;

	status = bdp->cbd_sc;

	if (status & BD_ENET_TX_READY) {
		/*
		 * Ooops.  All transmit buffers are full.  Bail out.
		 * This should not happen, since dev->tbusy should be set.
		 */
		printk(KERN_ERR "%s: tx queue full!.\n", dev->name);
		spin_unlock_irqrestore(&fep->hw_lock, flags);
		return NETDEV_TX_BUSY;
	}

	/* Clear all of the status flags */
	status &= ~BD_ENET_TX_STATS;

	/* Set buffer length and buffer pointer	*/
	bufaddr = skb->data;
	bdp->cbd_datlen = skb->len;

	/*
	 * On some FEC implementations data must be aligned on
	 * 4-byte boundaries. Use bounce buffers to copy data
	 * and get it aligned. Ugh.
	 */
	if ((unsigned long) bufaddr & FEC_ALIGNMENT) {
		unsigned int index;
		index = bdp - fep->tx_bd_base;
		memcpy(fep->tx_bounce[index],
		       (void *)skb->data, skb->len);
		bufaddr = fep->tx_bounce[index];
	}

#ifdef CONFIG_ARCH_MXS
	swap_buffer(bufaddr, skb->len);
#endif

	/* Save skb pointer. */
	fep->tx_skbuff[fep->skb_cur] = skb;

	dev->stats.tx_bytes += skb->len;
	fep->skb_cur = (fep->skb_cur+1) & TX_RING_MOD_MASK;

	/*
	 * Push the data cache so the CPM does not get stale memory
	 * data.
	 */
	bdp->cbd_bufaddr = dma_map_single(&dev->dev, bufaddr,
		FEC_ENET_TX_FRSIZE, DMA_TO_DEVICE);

	/*
	 * Send it on its way.  Tell FEC it's ready, interrupt when done,
	 * it's the last BD of the frame, and to put the CRC on the end.
	 */

	status |= (BD_ENET_TX_READY | BD_ENET_TX_INTR
			| BD_ENET_TX_LAST | BD_ENET_TX_TC);
	bdp->cbd_sc = status;
#ifdef L2SWITCH_ENHANCED_BUFFER
	bdp->bdu = 0x00000000;
	bdp->ebd_status = TX_BD_INT | TX_BD_TS;
#endif
	dev->trans_start = jiffies;

	/* Trigger transmission start */
	writel(MCF_ESW_TDAR_X_DES_ACTIVE, &fecp->fec_x_des_active);

	/* If this was the last BD in the ring,
	 * start at the beginning again.
	 */
	if (status & BD_ENET_TX_WRAP)
		bdp = fep->tx_bd_base;
	else
		bdp++;

	if (bdp == fep->dirty_tx) {
		fep->tx_full = 1;
		netif_stop_queue(dev);
		printk(KERN_ERR "%s:  net stop\n", __func__);
	}

	fep->cur_tx = bdp;

	spin_unlock_irqrestore(&fep->hw_lock, flags);

	return 0;
}

static void
switch_timeout(struct net_device *dev)
{
	struct switch_enet_private *fep = netdev_priv(dev);

	printk(KERN_INFO "%s: transmit timed out.\n", dev->name);
	dev->stats.tx_errors++;
	{
	int	i;
	struct cbd_t	*bdp;

	printk(KERN_INFO "Ring data dump: cur_tx %lx%s,"
		"dirty_tx %lx cur_rx: %lx\n",
		(unsigned long)fep->cur_tx, fep->tx_full ? " (full)" : "",
		(unsigned long)fep->dirty_tx,
		(unsigned long)fep->cur_rx);

	bdp = fep->tx_bd_base;
	printk(KERN_INFO " tx: %u buffers\n",  TX_RING_SIZE);
	for (i = 0 ; i < TX_RING_SIZE; i++) {
		printk(KERN_INFO "  %08x: %04x %04x %08x\n",
		       (uint) bdp,
		       bdp->cbd_sc,
		       bdp->cbd_datlen,
		       (int) bdp->cbd_bufaddr);
		bdp++;
	}

	bdp = fep->rx_bd_base;
	printk(KERN_INFO " rx: %lu buffers\n",
			(unsigned long) RX_RING_SIZE);
	for (i = 0 ; i < RX_RING_SIZE; i++) {
		printk(KERN_INFO "  %08x: %04x %04x %08x\n",
		       (uint) bdp,
		       bdp->cbd_sc,
		       bdp->cbd_datlen,
		       (int) bdp->cbd_bufaddr);
		bdp++;
	}
	}
	switch_restart(dev, fep->full_duplex);
	netif_wake_queue(dev);
}

/*
 * The interrupt handler.
 * This is called from the MPC core interrupt.
 */
static irqreturn_t
switch_enet_interrupt(int irq, void *dev_id)
{
	struct	net_device *dev = dev_id;
	struct switch_enet_private *fep = netdev_priv(dev);
	struct switch_t *fecp;
	uint	int_events;
	irqreturn_t ret = IRQ_NONE;

	fecp = (struct switch_t *)dev->base_addr;

	/* Get the interrupt events that caused us to be here */
	do {
		int_events = readl(&fecp->switch_ievent);
		writel(int_events, &fecp->switch_ievent);
		/* Handle receive event in its own function. */

		/* Transmit OK, or non-fatal error. Update the buffer
		 * descriptors. FEC handles all errors, we just discover
		 *  them as part of the transmit process.
		 */
		if (int_events & MCF_ESW_ISR_LRN) {
			if (readl(&fep->learning_irqhandle_enable))
				esw_atable_dynamicms_learn_migration(
					fep, fep->currTime);
			ret = IRQ_HANDLED;
		}

		if (int_events & MCF_ESW_ISR_OD0)
			ret = IRQ_HANDLED;

		if (int_events & MCF_ESW_ISR_OD1)
			ret = IRQ_HANDLED;

		if (int_events & MCF_ESW_ISR_OD2)
			ret = IRQ_HANDLED;

		if (int_events & MCF_ESW_ISR_RXB)
			ret = IRQ_HANDLED;

		if (int_events & MCF_ESW_ISR_RXF) {
			ret = IRQ_HANDLED;
			switch_enet_rx(dev);
		}

		if (int_events & MCF_ESW_ISR_TXB)
			ret = IRQ_HANDLED;

		if (int_events & MCF_ESW_ISR_TXF) {
			ret = IRQ_HANDLED;
			switch_enet_tx(dev);
		}

	} while (int_events);

	return ret;
}

static irqreturn_t mac0_enet_interrupt(int irq, void *dev_id)
{
	struct	net_device *dev = dev_id;
	struct switch_enet_private *fep = netdev_priv(dev);
	uint	int_events;
	irqreturn_t ret = IRQ_NONE;

	do {
		int_events = readl(fep->enet_addr + MCF_FEC_EIR0);
		writel(int_events, fep->enet_addr + MCF_FEC_EIR0);

		if (int_events & FEC_ENET_MII) {
			ret = IRQ_HANDLED;
			complete(&fep->mdio_done);
		}
	} while (int_events);

	return ret;
}

static irqreturn_t mac1_enet_interrupt(int irq, void *dev_id)
{
	struct	net_device *dev = dev_id;
	struct switch_enet_private *fep = netdev_priv(dev);
	uint	int_events;
	irqreturn_t ret = IRQ_NONE;

	do {
		int_events = readl(fep->enet_addr + MCF_FEC_EIR1);
		writel(int_events, fep->enet_addr + MCF_FEC_EIR1);

		if (int_events & FEC_ENET_MII) {
			ret = IRQ_HANDLED;
			complete(&fep->mdio_done);
		}
	} while (int_events);

	return ret;
}

static void
switch_enet_tx(struct net_device *dev)
{
	struct	switch_enet_private *fep;
	struct cbd_t	*bdp;
	unsigned short status;
	struct	sk_buff	*skb;

	fep = netdev_priv(dev);
	spin_lock(&fep->hw_lock);
	bdp = fep->dirty_tx;

	while (((status = bdp->cbd_sc) & BD_ENET_TX_READY) == 0) {
		if (bdp == fep->cur_tx && fep->tx_full == 0)
			break;

		dma_unmap_single(&dev->dev, bdp->cbd_bufaddr,
				FEC_ENET_TX_FRSIZE, DMA_TO_DEVICE);
		bdp->cbd_bufaddr = 0;
		skb = fep->tx_skbuff[fep->skb_dirty];
		/* Check for errors */
		if (status & (BD_ENET_TX_HB | BD_ENET_TX_LC |
				   BD_ENET_TX_RL | BD_ENET_TX_UN |
				   BD_ENET_TX_CSL)) {
			dev->stats.tx_errors++;
			if (status & BD_ENET_TX_HB)  /* No heartbeat */
				dev->stats.tx_heartbeat_errors++;
			if (status & BD_ENET_TX_LC)  /* Late collision */
				dev->stats.tx_window_errors++;
			if (status & BD_ENET_TX_RL)  /* Retrans limit */
				dev->stats.tx_aborted_errors++;
			if (status & BD_ENET_TX_UN)  /* Underrun */
				dev->stats.tx_fifo_errors++;
			if (status & BD_ENET_TX_CSL) /* Carrier lost */
				dev->stats.tx_carrier_errors++;
		} else {
			dev->stats.tx_packets++;
		}

		if (status & BD_ENET_TX_READY)
			printk(KERN_ERR "HEY! "
				"Enet xmit interrupt and TX_READY.\n");
		/*
		 * Deferred means some collisions occurred during transmit,
		 * but we eventually sent the packet OK.
		 */
		if (status & BD_ENET_TX_DEF)
			dev->stats.collisions++;

		/* Free the sk buffer associated with this last transmit */
		dev_kfree_skb_any(skb);
		fep->tx_skbuff[fep->skb_dirty] = NULL;
		fep->skb_dirty = (fep->skb_dirty + 1) & TX_RING_MOD_MASK;

		/* Update pointer to next buffer descriptor to be transmitted */
		if (status & BD_ENET_TX_WRAP)
			bdp = fep->tx_bd_base;
		else
			bdp++;

		/*
		 * Since we have freed up a buffer, the ring is no longer
		 * full.
		 */
		if (fep->tx_full) {
			fep->tx_full = 0;
			printk(KERN_ERR "%s: tx full is zero\n", __func__);
			if (netif_queue_stopped(dev))
				netif_wake_queue(dev);
		}
	}
	fep->dirty_tx = bdp;
	spin_unlock(&fep->hw_lock);
}


/*
 * During a receive, the cur_rx points to the current incoming buffer.
 * When we update through the ring, if the next incoming buffer has
 * not been given to the system, we just set the empty indicator,
 * effectively tossing the packet.
 */
static void
switch_enet_rx(struct net_device *dev)
{
	struct	switch_enet_private *fep;
	struct switch_t *fecp;
	struct cbd_t *bdp;
	unsigned short status;
	struct	sk_buff	*skb;
	ushort	pkt_len;
	__u8 *data;

#ifdef CONFIG_M532x
	flush_cache_all();
#endif

	fep = netdev_priv(dev);
	fecp = (struct switch_t *)fep->hwp;

	spin_lock(&fep->hw_lock);
	/*
	 * First, grab all of the stats for the incoming packet.
	 * These get messed up if we get called due to a busy condition.
	 */
	bdp = fep->cur_rx;
#ifdef L2SWITCH_ENHANCED_BUFFER
	printk(KERN_INFO "%s: cbd_sc %x cbd_datlen %x cbd_bufaddr %x "
		"ebd_status %x bdu %x length_proto_type %x "
		"payload_checksum %x\n",
		__func__, bdp->cbd_sc, bdp->cbd_datlen,
		bdp->cbd_bufaddr, bdp->ebd_status, bdp->bdu,
		bdp->length_proto_type, bdp->payload_checksum);
#endif

while (!((status = bdp->cbd_sc) & BD_ENET_RX_EMPTY)) {
	/*
	 * Since we have allocated space to hold a complete frame,
	 * the last indicator should be set.
	 */
	if ((status & BD_ENET_RX_LAST) == 0)
		printk(KERN_INFO "SWITCH ENET: rcv is not +last\n");

	if (!fep->opened)
		goto rx_processing_done;

	/* Check for errors. */
	if (status & (BD_ENET_RX_LG | BD_ENET_RX_SH | BD_ENET_RX_NO |
			   BD_ENET_RX_CR | BD_ENET_RX_OV)) {
		dev->stats.rx_errors++;
		if (status & (BD_ENET_RX_LG | BD_ENET_RX_SH)) {
			/* Frame too long or too short. */
			dev->stats.rx_length_errors++;
		}
		if (status & BD_ENET_RX_NO)	/* Frame alignment */
			dev->stats.rx_frame_errors++;
		if (status & BD_ENET_RX_CR)	/* CRC Error */
			dev->stats.rx_crc_errors++;
		if (status & BD_ENET_RX_OV)	/* FIFO overrun */
			dev->stats.rx_fifo_errors++;
	}

	/*
	 * Report late collisions as a frame error.
	 * On this error, the BD is closed, but we don't know what we
	 * have in the buffer.  So, just drop this frame on the floor.
	 */
	if (status & BD_ENET_RX_CL) {
		dev->stats.rx_errors++;
		dev->stats.rx_frame_errors++;
		goto rx_processing_done;
	}

	/* Process the incoming frame */
	dev->stats.rx_packets++;
	pkt_len = bdp->cbd_datlen;
	dev->stats.rx_bytes += pkt_len;
	data = (__u8 *)__va(bdp->cbd_bufaddr);

	dma_unmap_single(NULL, bdp->cbd_bufaddr, bdp->cbd_datlen,
			DMA_FROM_DEVICE);
#ifdef CONFIG_ARCH_MXS
	swap_buffer(data, pkt_len);
#endif
	/*
	 * This does 16 byte alignment, exactly what we need.
	 * The packet length includes FCS, but we don't want to
	 * include that when passing upstream as it messes up
	 * bridging applications.
	 */
	skb = dev_alloc_skb(pkt_len + NET_IP_ALIGN);
	if (unlikely(!skb)) {
		printk("%s: Memory squeeze, dropping packet.\n",
			dev->name);
		dev->stats.rx_dropped++;
	} else {
		skb_reserve(skb, NET_IP_ALIGN);
		skb_put(skb, pkt_len);      /* Make room */
		skb_copy_to_linear_data(skb, data, pkt_len);
		skb->protocol = eth_type_trans(skb, dev);
		netif_rx(skb);
	}

	bdp->cbd_bufaddr = dma_map_single(NULL, data, bdp->cbd_datlen,
		DMA_FROM_DEVICE);

rx_processing_done:

	/* Clear the status flags for this buffer */
	status &= ~BD_ENET_RX_STATS;

	/* Mark the buffer empty */
	status |= BD_ENET_RX_EMPTY;
	bdp->cbd_sc = status;

	/* Update BD pointer to next entry */
	if (status & BD_ENET_RX_WRAP)
		bdp = fep->rx_bd_base;
	else
		bdp++;

	/*
	 * Doing this here will keep the FEC running while we process
	 * incoming frames.  On a heavily loaded network, we should be
	 * able to keep up at the expense of system resources.
	 */
	writel(MCF_ESW_RDAR_R_DES_ACTIVE, &fecp->fec_r_des_active);
   } /* while (!((status = bdp->cbd_sc) & BD_ENET_RX_EMPTY)) */
	writel(bdp, &fep->cur_rx);

	spin_unlock(&fep->hw_lock);
}

/* ------------------------------------------------------------------------- */

/*
 * Phy section
 */
static void switch_adjust_link0(struct net_device *dev)
{
	struct switch_enet_private *fep = netdev_priv(dev);
	struct phy_device *phy_dev = fep->phy_dev[0];
	unsigned long flags;
	int status_change = 0;

	spin_lock_irqsave(&fep->hw_lock, flags);

	/* Prevent a state halted on mii error */
	if (fep->mii_timeout && phy_dev->state == PHY_HALTED) {
		phy_dev->state = PHY_RESUMING;
		goto spin_unlock;
	}

	/* Duplex link change */
	if (phy_dev->link) {
		if (fep->full_duplex != phy_dev->duplex)
			status_change = 1;
	}

	/* Link on or off change */
	if (phy_dev->link != fep->link[0]) {
		fep->link[0] = phy_dev->link;
		if (phy_dev->link) {
			/* if link becomes up and tx be stopped, start it */
			if (netif_queue_stopped(dev)) {
				netif_start_queue(dev);
				netif_wake_queue(dev);
			}
		}
		status_change = 1;
	}

spin_unlock:
	spin_unlock_irqrestore(&fep->hw_lock, flags);

	if (status_change)
		phy_print_status(phy_dev);
}

static void switch_adjust_link1(struct net_device *dev)
{
	struct switch_enet_private *fep = netdev_priv(dev);
	struct phy_device *phy_dev = fep->phy_dev[1];
	unsigned long flags;
	int status_change = 0;

	spin_lock_irqsave(&fep->hw_lock, flags);

	/* Prevent a state halted on mii error */
	if (fep->mii_timeout && phy_dev->state == PHY_HALTED) {
		phy_dev->state = PHY_RESUMING;
		goto spin_unlock;
	}

	/* Duplex link change */
	if (phy_dev->link) {
		if (fep->full_duplex != phy_dev->duplex)
			status_change = 1;
	}

	/* Link on or off change */
	if (phy_dev->link != fep->link[1]) {
		fep->link[1] = phy_dev->link;
		if (phy_dev->link) {
			/* if link becomes up and tx be stopped, start it */
			if (netif_queue_stopped(dev)) {
				netif_start_queue(dev);
				netif_wake_queue(dev);
			}
		}
		status_change = 1;
	}

spin_unlock:
	spin_unlock_irqrestore(&fep->hw_lock, flags);

	if (status_change)
		phy_print_status(phy_dev);
}

static int fec_enet_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	struct switch_enet_private *fep = bus->priv;
	unsigned long time_left;

	fep->mii_timeout = 0;
	init_completion(&fep->mdio_done);

	/* clear MII end of transfer bit */
	writel(FEC_ENET_MII, fep->enet_addr + FEC_IEVENT
			/ sizeof(unsigned long));

	/* start a read op */
	writel(FEC_MMFR_ST | FEC_MMFR_OP_READ |
		FEC_MMFR_PA(mii_id) | FEC_MMFR_RA(regnum) |
		FEC_MMFR_TA, fep->enet_addr + FEC_MII_DATA
		/ sizeof(unsigned long));

	/* wait for end of transfer */
	time_left = wait_for_completion_timeout(&fep->mdio_done,
		usecs_to_jiffies(FEC_MII_TIMEOUT));
	if (time_left == 0) {
		fep->mii_timeout = 1;
		printk(KERN_ERR "FEC: MDIO read timeout\n");
		return -ETIMEDOUT;
	}

	/* return value */
	return FEC_MMFR_DATA(readl(fep->enet_addr + FEC_MII_DATA
				/ sizeof(unsigned long)));
}

static int fec_enet_mdio_write(struct mii_bus *bus, int mii_id, int regnum,
			   u16 value)
{
	struct switch_enet_private *fep = bus->priv;
	unsigned long time_left;

	fep->mii_timeout = 0;
	init_completion(&fep->mdio_done);

	/* start a write op */
	writel(FEC_MMFR_ST | FEC_MMFR_OP_WRITE |
		FEC_MMFR_PA(mii_id) | FEC_MMFR_RA(regnum) |
		FEC_MMFR_TA | FEC_MMFR_DATA(value),
		fep->enet_addr + FEC_MII_DATA
		/ sizeof(unsigned long));

	/* wait for end of transfer */
	time_left = wait_for_completion_timeout(&fep->mdio_done,
		usecs_to_jiffies(FEC_MII_TIMEOUT));
	if (time_left == 0) {
		fep->mii_timeout = 1;
		printk(KERN_ERR "FEC: MDIO write timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int fec_enet_mdio_reset(struct mii_bus *bus)
{
	return 0;
}

static int fec_enet_mii_probe(struct net_device *dev)
{
	struct switch_enet_private *fep = netdev_priv(dev);
	struct phy_device *phy_dev[2] = {NULL};
	int phy_addr, i;

	fep->phy_dev[0] = NULL;
	fep->phy_dev[1] = NULL;
	i = 0;

	/* find the phy, assuming fec index corresponds to addr */
	for (phy_addr = 0; phy_addr < PHY_MAX_ADDR; phy_addr++) {
		if (fep->mii_bus->phy_map[phy_addr]) {
			phy_dev[i] = fep->mii_bus->phy_map[phy_addr];
			i += 1;
		}
		if (i > 1)
			break;
	}

	if (!phy_dev[0] && !phy_dev[1]) {
		printk(KERN_ERR "%s: no PHY found\n", dev->name);
		return -ENODEV;
	}

	/* attach the mac to the phy */
	phy_dev[0] = phy_connect(dev, dev_name(&phy_dev[0]->dev),
			     &switch_adjust_link0, 0,
			     fep->phy_interface);
	if (IS_ERR(phy_dev[0])) {
		printk(KERN_ERR "%s: Could not attach to PHY\n", dev->name);
		return PTR_ERR(phy_dev[0]);
	}

	phy_dev[1] = phy_connect(dev, dev_name(&phy_dev[1]->dev),
			     &switch_adjust_link1, 0,
			     fep->phy_interface);
	if (IS_ERR(phy_dev[1])) {
		printk(KERN_ERR "%s: Could not attach to PHY\n", dev->name);
		return PTR_ERR(phy_dev[1]);
	}

	/* mask with MAC supported features */
	for (i = 0; i < 2; i++) {
		phy_dev[i]->supported &= PHY_BASIC_FEATURES;
		phy_dev[i]->advertising = phy_dev[i]->supported;
		fep->phy_dev[i] = phy_dev[i];
	}

	fep->link[0] = 0;
	fep->link[1] = 0;
	fep->full_duplex = 0;

	printk(KERN_INFO "%s: Freescale FEC PHY driver [%s] "
		"(mii_bus:phy_addr=%s, irq=%d)\n", dev->name,
		fep->phy_dev[0]->drv->name, dev_name(&fep->phy_dev[0]->dev),
		fep->phy_dev[0]->irq);

	return 0;
}

static struct mii_bus *fec_enet_mii_init(struct net_device *dev,
		struct platform_device *pdev)
{
	struct switch_enet_private *fep = netdev_priv(dev);
	int err = -ENXIO, i;

	fep->mii_timeout = 0;

	/*
	 * Set MII speed to 2.5 MHz (= clk_get_rate() / 2 * phy_speed)
	 */
	fep->phy_speed = DIV_ROUND_UP(clk_get_rate(fep->clk), 5000000) << 1;
#ifdef CONFIG_ARCH_MXS
	/* Can't get phy(8720) ID when set to 2.5M on MX28, lower it */
	fep->phy_speed <<= 2;
#endif
	writel(fep->phy_speed, fep->enet_addr + FEC_MII_SPEED / 4);

	fep->mii_bus = mdiobus_alloc();
	if (fep->mii_bus == NULL) {
		err = -ENOMEM;
		goto err_out;
	}

	fep->mii_bus->name = "fec_enet_mii_bus";
	fep->mii_bus->read = fec_enet_mdio_read;
	fep->mii_bus->write = fec_enet_mdio_write;
	fep->mii_bus->reset = fec_enet_mdio_reset;
	snprintf(fep->mii_bus->id, MII_BUS_ID_SIZE, "%x", pdev->id);
	fep->mii_bus->priv = fep;
	fep->mii_bus->parent = &pdev->dev;

	fep->mii_bus->irq = kmalloc(sizeof(int) * PHY_MAX_ADDR, GFP_KERNEL);
	if (!fep->mii_bus->irq) {
		err = -ENOMEM;
		goto err_out_free_mdiobus;
	}

	for (i = 0; i < PHY_MAX_ADDR; i++)
		fep->mii_bus->irq[i] = PHY_POLL;

	platform_set_drvdata(dev, fep->mii_bus);

	if (mdiobus_register(fep->mii_bus))
		goto err_out_free_mdio_irq;

	return fep->mii_bus;

err_out_free_mdio_irq:
	kfree(fep->mii_bus->irq);
err_out_free_mdiobus:
	mdiobus_free(fep->mii_bus);
err_out:
	return ERR_PTR(err);
}

static void fec_enet_mii_remove(struct switch_enet_private *fep)
{
	int i;
	for (i = 0; i < 2; i++) {
		if (fep->phy_dev[i])
			phy_disconnect(fep->phy_dev[i]);
	}
	mdiobus_unregister(fep->mii_bus);
	kfree(fep->mii_bus->irq);
	mdiobus_free(fep->mii_bus);
}

static int fec_enet_get_settings(struct net_device *dev,
				  struct ethtool_cmd *cmd)
{
	struct switch_enet_private *fep = netdev_priv(dev);
	struct phy_device *phydev = fep->phy_dev[0];

	if (!phydev)
		return -ENODEV;

	return phy_ethtool_gset(phydev, cmd);
}

static int fec_enet_set_settings(struct net_device *dev,
				 struct ethtool_cmd *cmd)
{
	struct switch_enet_private *fep = netdev_priv(dev);
	struct phy_device *phydev = fep->phy_dev[0];

	if (!phydev)
		return -ENODEV;

	return phy_ethtool_sset(phydev, cmd);
}

static void fec_enet_get_drvinfo(struct net_device *dev,
				 struct ethtool_drvinfo *info)
{
	struct switch_enet_private *fep = netdev_priv(dev);

	strcpy(info->driver, fep->pdev->dev.driver->name);
	strcpy(info->version, "Revision: 1.0");
	strcpy(info->bus_info, dev_name(&dev->dev));
}

static void fec_enet_free_buffers(struct net_device *dev)
{
	struct switch_enet_private *fep = netdev_priv(dev);
	int i;
	struct sk_buff *skb;
	struct cbd_t	*bdp;

	bdp = fep->rx_bd_base;
	for (i = 0; i < RX_RING_SIZE; i++) {
		skb = fep->rx_skbuff[i];

		if (bdp->cbd_bufaddr)
			dma_unmap_single(&dev->dev, bdp->cbd_bufaddr,
					FEC_ENET_RX_FRSIZE, DMA_FROM_DEVICE);
		if (skb)
			dev_kfree_skb(skb);
		bdp++;
	}

	bdp = fep->tx_bd_base;
	for (i = 0; i < TX_RING_SIZE; i++)
		kfree(fep->tx_bounce[i]);
}

static int fec_enet_alloc_buffers(struct net_device *dev)
{
	struct switch_enet_private *fep = netdev_priv(dev);
	int i;
	struct sk_buff *skb;
	struct cbd_t	*bdp;

	bdp = fep->rx_bd_base;
	for (i = 0; i < RX_RING_SIZE; i++) {
		skb = dev_alloc_skb(SWITCH_ENET_RX_FRSIZE);
		if (!skb) {
			fec_enet_free_buffers(dev);
			return -ENOMEM;
		}
		fep->rx_skbuff[i] = skb;

		bdp->cbd_bufaddr = dma_map_single(&dev->dev, skb->data,
				SWITCH_ENET_RX_FRSIZE, DMA_FROM_DEVICE);
		bdp->cbd_sc = BD_ENET_RX_EMPTY;
#ifdef L2SWITCH_ENHANCED_BUFFER
		bdp->ebd_status = RX_BD_INT;
		bdp->bdu = 0x00000000;
#endif
		bdp++;
	}

	/* Set the last buffer to wrap. */
	bdp--;
	bdp->cbd_sc |= BD_SC_WRAP;

	bdp = fep->tx_bd_base;
	for (i = 0; i < TX_RING_SIZE; i++) {
		fep->tx_bounce[i] = kmalloc(SWITCH_ENET_TX_FRSIZE, GFP_KERNEL);

		bdp->cbd_sc = 0;
		bdp->cbd_bufaddr = 0;
#ifdef L2SWITCH_ENHANCED_BUFFER
		bdp->ebd_status = TX_BD_INT;
		bdp->bdu = 0x00000000;
#endif
		bdp++;
	}

	/* Set the last buffer to wrap. */
	bdp--;
	bdp->cbd_sc |= BD_SC_WRAP;

	return 0;
}

static int
switch_enet_open(struct net_device *dev)
{
	int ret;
	struct switch_enet_private *fep = netdev_priv(dev);
	/* I should reset the ring buffers here, but I don't yet know
	 * a simple way to do that.
	 */
	clk_enable(fep->clk);
	ret = fec_enet_alloc_buffers(dev);
	if (ret)
		return ret;

	fep->link[0] = 0;
	fep->link[1] = 0;
	/* Probe and connect to PHY when open the interface */
	ret = fec_enet_mii_probe(dev);
	if (ret) {
		fec_enet_free_buffers(dev);
		return ret;
	}
	phy_start(fep->phy_dev[0]);
	phy_start(fep->phy_dev[1]);
	fep->old_link = 0;
	fep->link[0] = 1;
	fep->link[1] = 1;

	switch_restart(dev, 1);

	fep->currTime = 0;
	fep->learning_irqhandle_enable = 1;

	esw_main(fep);

	netif_start_queue(dev);
	fep->opened = 1;

	return 0;		/* Success */
}

static int
switch_enet_close(struct net_device *dev)
{
	struct switch_enet_private *fep = netdev_priv(dev);
	int i;

	fep->opened = 0;

	switch_stop(dev);
	for (i = 0; i < 2; i++) {
		if (fep->phy_dev[i]) {
			phy_stop(fep->phy_dev[i]);
			phy_disconnect(fep->phy_dev[i]);
		}
		phy_write(fep->phy_dev[i], MII_BMCR, BMCR_PDOWN);
	}

	fec_enet_free_buffers(dev);
	clk_disable(fep->clk);

	return 0;
}

/*
 * Set or clear the multicast filter for this adaptor.
 * Skeleton taken from sunlance driver.
 * The CPM Ethernet implementation allows Multicast as well as individual
 * MAC address filtering.  Some of the drivers check to make sure it is
 * a group multicast address, and discard those that are not.  I guess I
 * will do the same for now, but just remove the test if you want
 * individual filtering as well (do the upper net layers want or support
 * this kind of feature?).
 */

/* bits in hash */
#define HASH_BITS	6
#define CRC32_POLY	0xEDB88320

static void set_multicast_list(struct net_device *dev)
{
	struct switch_enet_private *fep;
	struct switch_t *ep;
	unsigned int i, bit, data, crc;

	fep = netdev_priv(dev);
	ep = fep->hwp;

	if (dev->flags & IFF_PROMISC) {
		/* ep->fec_r_cntrl |= 0x0008; */
		printk(KERN_INFO "%s IFF_PROMISC\n", __func__);
	} else {

		/* ep->fec_r_cntrl &= ~0x0008; */

		if (dev->flags & IFF_ALLMULTI) {
			/*
			 * Catch all multicast addresses, so set the
			 * filter to all 1's.
			 */
			printk(KERN_INFO "%s IFF_ALLMULTI\n", __func__);
		} else {
			/*
			 * Clear filter and add the addresses
			 * in hash register
			 */
			/*
			 * ep->fec_grp_hash_table_high = 0;
			 * ep->fec_grp_hash_table_low = 0;
			 */
			struct netdev_hw_addr *ha;
			u_char *addrs;
			netdev_for_each_mc_addr(ha, dev) {
				addrs = ha->addr;
				/* Only support group multicast for now */
				if (!(*addrs & 1))
					continue;

				/* calculate crc32 value of mac address	*/
				crc = 0xffffffff;

				for (i = 0; i < 6; i++) {
					data = addrs[i];
					for (bit = 0; bit < 8; bit++,
						data >>= 1) {
						crc = (crc >> 1) ^
						(((crc ^ data) & 1) ?
						CRC32_POLY : 0);
					}
				}

			}
		}
	}
}

/* Set a MAC change in hardware */
static int
switch_set_mac_address(struct net_device *dev, void *p)
{
	struct switch_enet_private *fep = netdev_priv(dev);
	struct sockaddr *addr = p;
	struct switch_t  *fecp;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	fecp = fep->hwp;
	writel(MCF_ESW_DBCR_P1, &fecp->ESW_DBCR);

	memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);

	writel(dev->dev_addr[3] | (dev->dev_addr[2] << 8) |
		(dev->dev_addr[1] << 16) | (dev->dev_addr[0] << 24),
		fep->enet_addr + MCF_FEC_PAUR0);
	writel((dev->dev_addr[5] << 16)
		| ((dev->dev_addr[4]+(unsigned char)(0)) << 24),
		fep->enet_addr + MCF_FEC_PAUR0);

	writel(dev->dev_addr[3] | (dev->dev_addr[2] << 8) |
		(dev->dev_addr[1] << 16) | (dev->dev_addr[0] << 24),
		fep->enet_addr + MCF_FEC_PAUR1);
	writel((dev->dev_addr[5] << 16)
		| ((dev->dev_addr[4]+(unsigned char)(1)) << 24),
		fep->enet_addr + MCF_FEC_PAUR1);

	esw_update_atable_static(dev->dev_addr, 7, 7, fep);
	writel(MCF_ESW_DBCR_P1 | MCF_ESW_DBCR_P2, &fecp->ESW_DBCR);

	return 0;
}

static struct ethtool_ops fec_enet_ethtool_ops = {
	.get_settings           = fec_enet_get_settings,
	.set_settings           = fec_enet_set_settings,
	.get_drvinfo            = fec_enet_get_drvinfo,
	.get_link               = ethtool_op_get_link,
 };
static const struct net_device_ops fec_netdev_ops = {
       .ndo_open		= switch_enet_open,
       .ndo_stop		= switch_enet_close,
       .ndo_do_ioctl		= switch_enet_ioctl,
       .ndo_start_xmit		= switch_enet_start_xmit,
       .ndo_set_multicast_list	= set_multicast_list,
       .ndo_tx_timeout		= switch_timeout,
       .ndo_set_mac_address	= switch_set_mac_address,
};

static int switch_mac_addr_setup(char *mac_addr)
{
	char *ptr, *p = mac_addr;
	unsigned long tmp;
	int i = 0, ret = 0;

	while (p && (*p) && i < 6) {
		ptr = strchr(p, ':');
		if (ptr)
			*ptr++ = '\0';
		if (strlen(p)) {
			ret = strict_strtoul(p, 16, &tmp);
			if (ret < 0 || tmp > 0xff)
				break;
			switch_mac_default[i++] = tmp;
		}
		p = ptr;
	}

	return 0;
}

__setup("fec_mac=", switch_mac_addr_setup);

/* Initialize the FEC Ethernet */
static int __init switch_enet_init(struct net_device *dev,
	int slot, struct platform_device *pdev)
{
	struct switch_enet_private	*fep = netdev_priv(dev);
	struct resource 	*r;
	struct cbd_t		*bdp;
	struct cbd_t		*cbd_base;
	struct switch_t	*fecp;
	int	i;
	struct switch_platform_data *plat = pdev->dev.platform_data;

	/* Only allow us to be probed once. */
	if (slot >= SWITCH_MAX_PORTS)
		return -ENXIO;

	/* Allocate memory for buffer descriptors */
	cbd_base = dma_alloc_coherent(NULL, PAGE_SIZE, &fep->bd_dma,
			 GFP_KERNEL);
	if (!cbd_base) {
		printk(KERN_ERR "FEC: allocate descriptor memory failed?\n");
		return -ENOMEM;
	}

	spin_lock_init(&fep->hw_lock);
	spin_lock_init(&fep->mii_lock);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r)
		return -ENXIO;

	r = request_mem_region(r->start, resource_size(r), pdev->name);
	if (!r)
		return -EBUSY;

	fep->enet_addr = ioremap(r->start, resource_size(r));

	dev->irq = platform_get_irq(pdev, 0);
	fep->mac0_irq = platform_get_irq(pdev, 1);
	fep->mac1_irq = platform_get_irq(pdev, 2);

	/*
	 * Create an Ethernet device instance.
	 * The switch lookup address memory start 0x800FC000
	 */
	fecp = (struct switch_t *)(fep->enet_addr + ENET_SWI_PHYS_ADDR_OFFSET
		/ sizeof(unsigned long));
	plat->switch_hw[1] = (unsigned long)fecp + MCF_ESW_LOOKUP_MEM_OFFSET;

	fep->index = slot;
	fep->hwp = fecp;
	fep->hwentry = (struct eswAddrTable_t *)plat->switch_hw[1];
	fep->netdev = dev;
#ifdef CONFIG_FEC_SHARED_PHY
	fep->phy_hwp = (struct switch_t *) plat->switch_hw[slot & ~1];
#else
	fep->phy_hwp = fecp;
#endif

	fep->clk = clk_get(&pdev->dev, "fec_clk");
	if (IS_ERR(fep->clk))
		return PTR_ERR(fep->clk);
	clk_enable(fep->clk);


	/* PHY reset should be done during clock on */
	if (plat) {
		fep->phy_interface = plat->fec_enet->phy;
		if (plat->fec_enet->init && plat->fec_enet->init())
			return -EIO;

		/*
		 * The priority for getting MAC address is:
		 * (1) kernel command line fec_mac = xx:xx:xx...
		 * (2) platform data mac field got from fuse etc
		 * (3) bootloader set the FEC mac register
		 */

		if (!is_valid_ether_addr(switch_mac_default) &&
			plat->fec_enet->mac &&
			is_valid_ether_addr(plat->fec_enet->mac))
			memcpy(switch_mac_default, plat->fec_enet->mac,
						sizeof(switch_mac_default));
	} else
		fep->phy_interface = PHY_INTERFACE_MODE_MII;

	/* Enable MII mode */

	/*
	 * SWITCH CONFIGURATION
	 */
	writel(MCF_ESW_MODE_SW_RST, &fecp->ESW_MODE);
	udelay(10);

	/* enable switch*/
	writel(MCF_ESW_MODE_STATRST, &fecp->ESW_MODE);
	writel(MCF_ESW_MODE_SW_EN, &fecp->ESW_MODE);

	/* Enable transmit/receive on all ports */
	writel(0xffffffff, &fecp->ESW_PER);
	/* Management port configuration,
	 * make port 0 as management port
	 */
	writel(0, &fecp->ESW_BMPC);

	/* clear all switch irq */
	writel(0xffffffff, &fecp->switch_ievent);
	writel(0, &fecp->switch_imask);
	udelay(10);

	plat->request_intrs = switch_request_intrs;
	plat->set_mii = switch_set_mii;
	plat->get_mac = switch_get_mac;
	plat->enable_phy_intr = switch_enable_phy_intr;
	plat->disable_phy_intr = switch_disable_phy_intr;
	plat->phy_ack_intr = switch_phy_ack_intr;
	plat->localhw_setup = switch_localhw_setup;
	plat->uncache = switch_uncache;
	plat->platform_flush_cache = switch_platform_flush_cache;

	/*
	 * Set the Ethernet address.  If using multiple Enets on the 8xx,
	 * this needs some work to get unique addresses.
	 *
	 * This is our default MAC address unless the user changes
	 * it via eth_mac_addr (our dev->set_mac_addr handler).
	 */
	if (plat && plat->get_mac)
		plat->get_mac(dev);

	/* Set receive and transmit descriptor base */
	fep->rx_bd_base = cbd_base;
	fep->tx_bd_base = cbd_base + RX_RING_SIZE;

	/* Initialize the receive buffer descriptors */
	bdp = fep->rx_bd_base;
	for (i = 0; i < RX_RING_SIZE; i++) {
		bdp->cbd_sc = 0;

#ifdef L2SWITCH_ENHANCED_BUFFER
		bdp->bdu = 0x00000000;
		bdp->ebd_status = RX_BD_INT;
#endif
		bdp++;
	}

	/* Set the last buffer to wrap */
	bdp--;
	bdp->cbd_sc |= BD_SC_WRAP;

	/* ...and the same for transmmit */
	bdp = fep->tx_bd_base;
	for (i = 0; i < TX_RING_SIZE; i++) {
		/* Initialize the BD for every fragment in the page */
		bdp->cbd_sc = 0;
		bdp->cbd_bufaddr = 0;
		bdp++;
	}

	/* Set the last buffer to wrap */
	bdp--;
	bdp->cbd_sc |= BD_SC_WRAP;

	/*
	 * Install our interrupt handlers. This varies depending on
	 * the architecture.
	 */
	if (plat && plat->request_intrs)
		plat->request_intrs(dev, switch_enet_interrupt, dev);

	if (request_irq(fep->mac0_irq, mac0_enet_interrupt, IRQF_DISABLED,
			"enet-1588", dev))
		printk(KERN_ERR "IEEE1588: failed to request irq\n");
	if (request_irq(fep->mac1_irq, mac1_enet_interrupt, IRQF_DISABLED,
			"enet-1588", dev))
		printk(KERN_ERR "IEEE1588: failed to request irq\n");

	dev->base_addr = (unsigned long)fecp;

	/* The FEC Ethernet specific entries in the device structure. */
	dev->netdev_ops = &fec_netdev_ops;
	dev->ethtool_ops = &fec_enet_ethtool_ops;

	/* setup MII interface */
	if (plat && plat->set_mii)
		plat->set_mii(dev);


#ifndef CONFIG_FEC_SHARED_PHY
	fep->phy_addr = 0;
#else
	fep->phy_addr = fep->index;
#endif

	fep->sequence_done = 1;
	return 0;
}

static void enet_reset(struct net_device *dev, int duplex)
{
	struct switch_enet_private	*fep = netdev_priv(dev);

	/* ECR */
	writel(MCF_FEC_ECR_MAGIC_ENA,
			fep->enet_addr + MCF_FEC_ECR0);
	writel(MCF_FEC_ECR_MAGIC_ENA,
			fep->enet_addr + MCF_FEC_ECR1);

	/* EMRBR */
	writel(PKT_MAXBLR_SIZE, fep->enet_addr + MCF_FEC_EMRBR0);
	writel(PKT_MAXBLR_SIZE, fep->enet_addr + MCF_FEC_EMRBR1);

	/*
	 * set the receive and transmit BDs ring base to
	 * hardware registers(ERDSR & ETDSR)
	 */
	writel(fep->bd_dma, fep->enet_addr + MCF_FEC_ERDSR0);
	writel(fep->bd_dma, fep->enet_addr + MCF_FEC_ERDSR1);
	writel((unsigned long)fep->bd_dma + sizeof(struct cbd_t) * RX_RING_SIZE,
			fep->enet_addr + MCF_FEC_ETDSR0);
	writel((unsigned long)fep->bd_dma + sizeof(struct cbd_t) * RX_RING_SIZE,
			fep->enet_addr + MCF_FEC_ETDSR1);
#ifdef CONFIG_ARCH_MXS
	/* Can't get phy(8720) ID when set to 2.5M on MX28, lower it */
	writel(fep->phy_speed,
			fep->enet_addr + MCF_FEC_MSCR0);
	writel(fep->phy_speed,
			fep->enet_addr + MCF_FEC_MSCR1);
#endif
	fep->full_duplex = duplex;

	/* EIR */
	writel(0, fep->enet_addr + MCF_FEC_EIR0);
	writel(0, fep->enet_addr + MCF_FEC_EIR1);

	/* IAUR */
	writel(0, fep->enet_addr + MCF_FEC_IAUR0);
	writel(0, fep->enet_addr + MCF_FEC_IAUR1);

	/* IALR */
	writel(0, fep->enet_addr + MCF_FEC_IALR0);
	writel(0, fep->enet_addr + MCF_FEC_IALR1);

	/* GAUR */
	writel(0, fep->enet_addr + MCF_FEC_GAUR0);
	writel(0, fep->enet_addr + MCF_FEC_GAUR1);

	/* GALR */
	writel(0, fep->enet_addr + MCF_FEC_GALR0);
	writel(0, fep->enet_addr + MCF_FEC_GALR1);

	/* EMRBR */
	writel(PKT_MAXBLR_SIZE, fep->enet_addr + MCF_FEC_EMRBR0);
	writel(PKT_MAXBLR_SIZE, fep->enet_addr + MCF_FEC_EMRBR1);

	/* EIMR */
	writel(FEC_ENET_TXF | FEC_ENET_RXF | FEC_ENET_MII,
			fep->enet_addr + MCF_FEC_EIMR0);
	writel(FEC_ENET_TXF | FEC_ENET_RXF | FEC_ENET_MII,
			fep->enet_addr + MCF_FEC_EIMR1);

	/* PALR PAUR */
	/* Set the station address for the ENET Adapter */
	writel(dev->dev_addr[3] |
		dev->dev_addr[2]<<8 |
		dev->dev_addr[1]<<16 |
		dev->dev_addr[0]<<24, fep->enet_addr + MCF_FEC_PALR0);
	writel(dev->dev_addr[5]<<16 |
		(dev->dev_addr[4]+(unsigned char)(0))<<24,
		fep->enet_addr + MCF_FEC_PAUR0);
	writel(dev->dev_addr[3] |
		dev->dev_addr[2]<<8 |
		dev->dev_addr[1]<<16 |
		dev->dev_addr[0]<<24, fep->enet_addr + MCF_FEC_PALR1);
	writel(dev->dev_addr[5]<<16 |
		(dev->dev_addr[4]+(unsigned char)(1))<<24,
		fep->enet_addr + MCF_FEC_PAUR1);

	/* RCR */
	writel(readl(fep->enet_addr + MCF_FEC_RCR0)
		| MCF_FEC_RCR_FCE | MCF_FEC_RCR_PROM,
		fep->enet_addr + MCF_FEC_RCR0);
	writel(readl(fep->enet_addr + MCF_FEC_RCR1)
		| MCF_FEC_RCR_FCE | MCF_FEC_RCR_PROM,
		fep->enet_addr + MCF_FEC_RCR1);

	/* TCR */
	writel(0x1c, fep->enet_addr + MCF_FEC_TCR0);
	writel(0x1c, fep->enet_addr + MCF_FEC_TCR1);

	/* ECR */
	writel(readl(fep->enet_addr + MCF_FEC_ECR0) | MCF_FEC_ECR_ETHER_EN,
			fep->enet_addr + MCF_FEC_ECR0);
	writel(readl(fep->enet_addr + MCF_FEC_ECR1) | MCF_FEC_ECR_ETHER_EN,
			fep->enet_addr + MCF_FEC_ECR1);
}

/*
 * This function is called to start or restart the FEC during a link
 * change.  This only happens when switching between half and full
 * duplex.
 */
static void
switch_restart(struct net_device *dev, int duplex)
{
	struct switch_enet_private *fep;
	struct switch_t *fecp;
	int i;
	struct switch_platform_data *plat;

	fep = netdev_priv(dev);
	fecp = fep->hwp;
	plat = fep->pdev->dev.platform_data;
	/*
	 * Whack a reset.  We should wait for this.
	 */
	/* fecp->fec_ecntrl = 1; */
	writel(MCF_ESW_MODE_SW_RST, &fecp->ESW_MODE);
	udelay(10);
	writel(MCF_ESW_MODE_STATRST, &fecp->ESW_MODE);
	writel(MCF_ESW_MODE_SW_EN, &fecp->ESW_MODE);

	/* Enable transmit/receive on all ports */
	writel(0xffffffff, &fecp->ESW_PER);
	/*
	 * Management port configuration,
	 * make port 0 as management port
	 */
	writel(0, &fecp->ESW_BMPC);

	/* Clear any outstanding interrupt */
	writel(0xffffffff, &fecp->switch_ievent);
	/*if (plat && plat->enable_phy_intr)
	 *	plat->enable_phy_intr();
	 */

	/* Reset all multicast */
	/*
	 * fecp->fec_grp_hash_table_high = 0;
	 * fecp->fec_grp_hash_table_low = 0;
	 */

	/* Set maximum receive buffer size */
	writel(PKT_MAXBLR_SIZE, &fecp->fec_r_buff_size);

	if (plat && plat->localhw_setup)
		plat->localhw_setup();

	/* Set receive and transmit descriptor base */
	writel(fep->bd_dma, &fecp->fec_r_des_start);
	writel((unsigned long)fep->bd_dma
		+ sizeof(struct cbd_t) * RX_RING_SIZE,
		&fecp->fec_x_des_start);

	fep->dirty_tx = fep->cur_tx = fep->tx_bd_base;
	fep->cur_rx = fep->rx_bd_base;

	/* Reset SKB transmit buffers */
	fep->skb_cur = fep->skb_dirty = 0;
	for (i = 0; i <= TX_RING_MOD_MASK; i++) {
		if (fep->tx_skbuff[i] != NULL) {
			dev_kfree_skb_any(fep->tx_skbuff[i]);
			fep->tx_skbuff[i] = NULL;
		}
	}

	enet_reset(dev, duplex);
	esw_clear_atable(fep);

	/* And last, enable the transmit and receive processing */
	writel(MCF_ESW_RDAR_R_DES_ACTIVE, &fecp->fec_r_des_active);

	/* Enable interrupts we wish to service */
	writel(0xffffffff, &fecp->switch_ievent);
	writel(MCF_ESW_IMR_RXF | MCF_ESW_IMR_TXF |
		MCF_ESW_IMR_RXB | MCF_ESW_IMR_TXB,
		&fecp->switch_imask);

#ifdef SWITCH_DEBUG
	printk(KERN_INFO "%s: switch hw init over."
		"isr %x mask %x rx_addr %x %x tx_addr %x %x."
		"fec_r_buff_size %x\n", __func__,
		readl(&fecp->switch_ievent),
		readl(&fecp->switch_imask),
		readl(&fecp->fec_r_des_start),
		&fecp->fec_r_des_start,
		readl(&fecp->fec_x_des_start),
		&fecp->fec_x_des_start,
		readl(&fecp->fec_r_buff_size));
	printk(KERN_INFO "%s: fecp->ESW_DBCR %x, fecp->ESW_P0FFEN %x fecp->ESW_BKLR %x\n",
		__func__,
		readl(&fecp->ESW_DBCR),
		readl(&fecp->ESW_P0FFEN),
		readl(&fecp->ESW_BKLR));

	printk(KERN_INFO "fecp->portstats[0].MCF_ESW_POQC %x,"
		"fecp->portstats[0].MCF_ESW_PMVID %x,"
		"fecp->portstats[0].MCF_ESW_PMVTAG %x,"
		"fecp->portstats[0].MCF_ESW_PBL %x\n",
		readl(&fecp->port_statistics_status[0].MCF_ESW_POQC),
		readl(&fecp->port_statistics_status[0].MCF_ESW_PMVID),
		readl(&fecp->port_statistics_status[0].MCF_ESW_PMVTAG),
		readl(&fecp->port_statistics_status[0].MCF_ESW_PBL));

	printk(KERN_INFO "fecp->portstats[1].MCF_ESW_POQC %x,"
		"fecp->portstats[1].MCF_ESW_PMVID %x,"
		"fecp->portstats[1].MCF_ESW_PMVTAG %x,"
		"fecp->portstats[1].MCF_ESW_PBL %x\n",
		readl(&fecp->port_statistics_status[1].MCF_ESW_POQC),
		readl(&fecp->port_statistics_status[1].MCF_ESW_PMVID),
		readl(&fecp->port_statistics_status[1].MCF_ESW_PMVTAG),
		readl(&fecp->port_statistics_status[1].MCF_ESW_PBL));

	printk(KERN_INFO "fecp->portstats[2].MCF_ESW_POQC %x,"
		"fecp->portstats[2].MCF_ESW_PMVID %x,"
		"fecp->portstats[2].MCF_ESW_PMVTAG %x,"
		"fecp->portstats[2].MCF_ESW_PBL %x\n",
		readl(&fecp->port_statistics_status[2].MCF_ESW_POQC),
		readl(&fecp->port_statistics_status[2].MCF_ESW_PMVID),
		readl(&fecp->port_statistics_status[2].MCF_ESW_PMVTAG),
		readl(&fecp->port_statistics_status[2].MCF_ESW_PBL));
#endif
}

static void
switch_stop(struct net_device *dev)
{
	struct switch_t *fecp;
	struct switch_enet_private *fep;
	struct switch_platform_data *plat;

#ifdef SWITCH_DEBUG
	printk(KERN_ERR "%s\n", __func__);
#endif
	fep = netdev_priv(dev);
	fecp = fep->hwp;
	plat = fep->pdev->dev.platform_data;
	/* We cannot expect a graceful transmit stop without link !!! */
	if (fep->link[0] || fep->link[1])
		udelay(10);

	/* Whack a reset.  We should wait for this */
	netif_stop_queue(dev);
	fep->link[0] = 0;
	fep->link[1] = 0;
	udelay(10);
}

static int __init eth_switch_probe(struct platform_device *pdev)
{
	struct net_device *dev;
	int i, err;
	struct switch_enet_private *fep;
	struct switch_platform_private *chip;

	printk(KERN_INFO "Ethernet Switch Version 1.0\n");
	chip = kzalloc(sizeof(struct switch_platform_private) +
		sizeof(struct switch_enet_private *) * SWITCH_MAX_PORTS,
		GFP_KERNEL);
	if (!chip) {
		err = -ENOMEM;
		printk(KERN_ERR "%s: kzalloc fail %x\n", __func__,
			(unsigned int)chip);
		return err;
	}

	chip->pdev = pdev;
	chip->num_slots = SWITCH_MAX_PORTS;
	platform_set_drvdata(pdev, chip);

	for (i = 0; (i < chip->num_slots); i++) {
		dev = alloc_etherdev(sizeof(struct switch_enet_private));
		if (!dev) {
			printk(KERN_ERR "%s: ethernet switch\
				 alloc_etherdev fail\n",
				dev->name);
			return -ENOMEM;
		}

		fep = netdev_priv(dev);
		fep->pdev = pdev;
		printk(KERN_ERR "%s: ethernet switch port %d init\n",
			__func__, i);
		err = switch_enet_init(dev, i, pdev);
		if (err) {
			free_netdev(dev);
			platform_set_drvdata(pdev, NULL);
			kfree(chip);
			continue;
		}

		chip->fep_host[i] = fep;

		/* Init phy bus */
		if (pdev->id == 0) {
			fec_mii_bus = fec_enet_mii_init(dev, pdev);
			if (IS_ERR(fec_mii_bus))
				printk(KERN_ERR "can't init phy bus\n");
		} else
			fep->mii_bus = fec_mii_bus;

		/* setup timer for Learning  Aging function */
		/*
		 * setup_timer(&fep->timer_aging,
		 *	l2switch_aging_timer, (unsigned long)fep);
		 */
		init_timer(&fep->timer_aging);
		fep->timer_aging.function = l2switch_aging_timer;
		fep->timer_aging.data = (unsigned long) fep;
		fep->timer_aging.expires = jiffies + LEARNING_AGING_TIMER;

		/* register network device */
		if (register_netdev(dev) != 0) {
			free_netdev(dev);
			platform_set_drvdata(pdev, NULL);
			kfree(chip);
			printk(KERN_ERR "%s: ethernet switch register_netdev fail\n",
				dev->name);
			return -EIO;
		}
		printk(KERN_INFO "%s: ethernet switch %pM\n",
				dev->name, dev->dev_addr);
	}

	return 0;
}

static int eth_switch_remove(struct platform_device *pdev)
{
	int i;
	struct net_device *dev;
	struct switch_enet_private *fep;
	struct switch_platform_private *chip;

	chip = platform_get_drvdata(pdev);
	if (chip) {
		for (i = 0; i < chip->num_slots; i++) {
			fep = chip->fep_host[i];
			dev = fep->netdev;
			fep->sequence_done = 1;
			fec_enet_mii_remove(fep);
			unregister_netdev(dev);
			free_netdev(dev);

			del_timer_sync(&fep->timer_aging);
		}

		platform_set_drvdata(pdev, NULL);
		kfree(chip);

	} else
		printk(KERN_ERR "%s: can not get the "
			"switch_platform_private %x\n", __func__,
			(unsigned int)chip);

	return 0;
}

static struct platform_driver eth_switch_driver = {
	.probe          = eth_switch_probe,
	.remove         = eth_switch_remove,
	.driver         = {
		.name   = "mxs-l2switch",
		.owner  = THIS_MODULE,
	},
};

static int __init fec_l2switch_init(void)
{
	return platform_driver_register(&eth_switch_driver);;
}

static void __exit fec_l2_switch_exit(void)
{
	platform_driver_unregister(&eth_switch_driver);
}

module_init(fec_l2switch_init);
module_exit(fec_l2_switch_exit);
MODULE_LICENSE("GPL");
