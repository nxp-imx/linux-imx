/*
 * Copyright 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 * (c) 2005 MontaVista Software, Inc.
 *
 * This code is based on mtd_dataflash.c by adding FSL spi access.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/err.h>

#include <linux/spi/spi.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <asm/mach/flash.h>

/*
 * DataFlash is a kind of SPI flash.  Most AT45 chips have two buffers in
 * each chip, which may be used for double buffered I/O; but this driver
 * doesn't (yet) use these for any kind of i/o overlap or prefetching.
 *
 * Sometimes DataFlash is packaged in MMC-format cards, although the
 * MMC stack can't (yet?) distinguish between MMC and DataFlash
 * protocols during enumeration.
 */

/* reads can bypass the buffers */
#define OP_READ_CONTINUOUS	0xE8
#define OP_READ_PAGE		0xD2

/* group B requests can run even while status reports "busy" */
#define OP_READ_STATUS		0xD7	/* group B */

/* move data between host and buffer */
#define OP_READ_BUFFER1		0xD4	/* group B */
#define OP_READ_BUFFER2		0xD6	/* group B */
#define OP_WRITE_BUFFER1	0x84	/* group B */
#define OP_WRITE_BUFFER2	0x87	/* group B */

/* erasing flash */
#define OP_ERASE_PAGE		0x81
#define OP_ERASE_BLOCK		0x50

/* move data between buffer and flash */
#define OP_TRANSFER_BUF1	0x53
#define OP_TRANSFER_BUF2	0x55
#define OP_MREAD_BUFFER1	0xD4
#define OP_MREAD_BUFFER2	0xD6
#define OP_MWERASE_BUFFER1	0x83
#define OP_MWERASE_BUFFER2	0x86
#define OP_MWRITE_BUFFER1	0x88	/* sector must be pre-erased */
#define OP_MWRITE_BUFFER2	0x89	/* sector must be pre-erased */

/* write to buffer, then write-erase to flash */
#define OP_PROGRAM_VIA_BUF1	0x82
#define OP_PROGRAM_VIA_BUF2	0x85

/* compare buffer to flash */
#define OP_COMPARE_BUF1		0x60
#define OP_COMPARE_BUF2		0x61

/* read flash to buffer, then write-erase to flash */
#define OP_REWRITE_VIA_BUF1	0x58
#define OP_REWRITE_VIA_BUF2	0x59

/* newer chips report JEDEC manufacturer and device IDs; chip
 * serial number and OTP bits; and per-sector writeprotect.
 */
#define OP_READ_ID		0x9F
#define OP_READ_SECURITY	0x77
#define OP_WRITE_SECURITY_REVC	0x9A
#define OP_WRITE_SECURITY	0x9B	/* revision D */

#define	SPI_FIFOSIZE		24	/* Bust size in bytes */
#define	CMD_SIZE		4
#define DUMY_SIZE		4

struct dataflash {
	uint8_t command[4];
	char name[24];

	unsigned partitioned:1;

	unsigned short page_offset;	/* offset in flash address */
	unsigned int page_size;	/* of bytes per page */

	struct mutex lock;
	struct spi_device *spi;

	struct mtd_info mtd;
};

#ifdef CONFIG_MTD_PARTITIONS
#define	mtd_has_partitions()	(1)
#else
#define	mtd_has_partitions()	(0)
#endif

/* ......................................................................... */

/*
 * This function initializes the SPI device parameters.
 */
static inline int spi_nor_setup(struct spi_device *spi, u8 bst_len)
{
	spi->bits_per_word = bst_len << 3;

	return spi_setup(spi);
}

/*
 * This function perform spi read/write transfer.
 */
static int spi_read_write(struct spi_device *spi, u8 * buf, u32 len)
{
	struct spi_message m;
	struct spi_transfer t;

	if (len > SPI_FIFOSIZE || len <= 0)
		return -1;

	spi_nor_setup(spi, len);

	spi_message_init(&m);
	memset(&t, 0, sizeof t);

	t.tx_buf = buf;
	t.rx_buf = buf;
	t.len = ((len - 1) >> 2) + 1;

	spi_message_add_tail(&t, &m);

	if (spi_sync(spi, &m) != 0 || m.status != 0) {
		printk(KERN_ERR "%s: error\n", __func__);
		return -1;
	}

	DEBUG(MTD_DEBUG_LEVEL2, "%s: len: 0x%x success\n", __func__, len);

	return 0;

}

/*
 * Return the status of the DataFlash device.
 */
static inline int dataflash_status(struct spi_device *spi)
{
	/* NOTE:  at45db321c over 25 MHz wants to write
	 * a dummy byte after the opcode...
	 */
	ssize_t retval;

	u16 val = OP_READ_STATUS << 8;

	retval = spi_read_write(spi, (u8 *)&val, 2);

	if (retval < 0)
		return retval;

	DEBUG(MTD_DEBUG_LEVEL2, "%s: status: 0x%x\n", __func__, val & 0xff);

	return val & 0xff;
}

/*
 * Poll the DataFlash device until it is READY.
 * This usually takes 5-20 msec or so; more for sector erase.
 */
static int dataflash_waitready(struct spi_device *spi)
{
	int status;

	for (;;) {
		status = dataflash_status(spi);
		if (status < 0) {
			DEBUG(MTD_DEBUG_LEVEL1, "%s: status %d?\n",
			      dev_name(&spi->dev), status);
			status = 0;
		}

		if (status & (1 << 7))	/* RDY/nBSY */
			return status;

		msleep(3);
	}
}

/* ......................................................................... */

/*
 * Erase pages of flash.
 */
static int dataflash_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct dataflash *priv = (struct dataflash *)mtd->priv;
	struct spi_device *spi = priv->spi;
	unsigned blocksize = priv->page_size << 3;
	uint8_t *command;
	uint32_t		rem;

	DEBUG(MTD_DEBUG_LEVEL2, "%s: erase addr=0x%llx len 0x%llx\n",
	      dev_name(&spi->dev), (long long)instr->addr,
	      (long long)instr->len);

	/* Sanity checks */
	if (instr->addr + instr->len > mtd->size)
		return -EINVAL;
	div_u64_rem(instr->len, priv->page_size, &rem);
	if (rem)
		return -EINVAL;
	div_u64_rem(instr->addr, priv->page_size, &rem);
	if (rem)
		return -EINVAL;

	command = priv->command;

	mutex_lock(&priv->lock);
	while (instr->len > 0) {
		unsigned int pageaddr;
		int status;
		int do_block;

		/* Calculate flash page address; use block erase (for speed) if
		 * we're at a block boundary and need to erase the whole block.
		 */
		pageaddr = div_u64(instr->addr, priv->page_size);
		do_block = (pageaddr & 0x7) == 0 && instr->len >= blocksize;
		pageaddr = pageaddr << priv->page_offset;

		command[3] = do_block ? OP_ERASE_BLOCK : OP_ERASE_PAGE;
		command[2] = (uint8_t) (pageaddr >> 16);
		command[1] = (uint8_t) (pageaddr >> 8);
		command[0] = 0;

		DEBUG(MTD_DEBUG_LEVEL3, "ERASE %s: (%x) %x %x %x [%i]\n",
		      do_block ? "block" : "page",
		      command[0], command[1], command[2], command[3], pageaddr);

		status = spi_read_write(spi, command, 4);
		(void)dataflash_waitready(spi);

		if (status < 0) {
			printk(KERN_ERR "%s: erase %x, err %d\n",
			       dev_name(&spi->dev), pageaddr, status);
			/* REVISIT:  can retry instr->retries times; or
			 * giveup and instr->fail_addr = instr->addr;
			 */
			continue;
		}

		if (do_block) {
			instr->addr += blocksize;
			instr->len -= blocksize;
		} else {
			instr->addr += priv->page_size;
			instr->len -= priv->page_size;
		}
	}
	mutex_unlock(&priv->lock);

	/* Inform MTD subsystem that erase is complete */
	instr->state = MTD_ERASE_DONE;
	mtd_erase_callback(instr);

	return 0;
}

/*
 * Read from the DataFlash device.
 *   from   : Start offset in flash device
 *   len    : Amount to read
 *   retlen : About of data actually read
 *   buf    : Buffer containing the data
 */
static int dataflash_read(struct mtd_info *mtd, loff_t from, size_t len,
			  size_t *retlen, u_char *buf)
{
	struct dataflash *priv = mtd->priv;
	struct spi_device *spi = priv->spi;
	u32 addr;
	int rx_len = 0, count = 0, i = 0;
	u_char txer[SPI_FIFOSIZE];
	u_char *s = txer;
	u_char *d = buf;
	int cmd_len = CMD_SIZE + DUMY_SIZE;
	int status = 0;

	DEBUG(MTD_DEBUG_LEVEL2, "%s: read 0x%x..0x%x\n",
	      dev_name(&priv->spi->dev), (unsigned)from, (unsigned)(from + len));

	*retlen = 0;

	/* Sanity checks */
	if (!len)
		return 0;

	if (from + len > mtd->size)
		return -EINVAL;

	/* Calculate flash page/byte address */
	addr = (((unsigned)from / priv->page_size) << priv->page_offset)
	    + ((unsigned)from % priv->page_size);

	mutex_unlock(&priv->lock);

	while (len > 0) {

		rx_len = len > (SPI_FIFOSIZE - cmd_len) ?
		    SPI_FIFOSIZE - cmd_len : len;

		txer[3] = OP_READ_CONTINUOUS;
		txer[2] = (addr >> 16) & 0xff;
		txer[1] = (addr >> 8) & 0xff;
		txer[0] = addr & 0xff;

		status = spi_read_write(spi, txer,
					roundup(rx_len, 4) + cmd_len);
		if (status) {
			mutex_unlock(&priv->lock);
			return status;
		}

		s = txer + cmd_len;

		for (i = rx_len; i >= 0; i -= 4, s += 4) {
			if (i < 4) {
				if (i == 1) {
					*d = s[3];
				} else if (i == 2) {
					*d++ = s[3];
					*d++ = s[2];
				} else if (i == 3) {
					*d++ = s[3];
					*d++ = s[2];
					*d++ = s[1];
				}

				break;
			}

			*d++ = s[3];
			*d++ = s[2];
			*d++ = s[1];
			*d++ = s[0];
		}

		/* updaate */
		len -= rx_len;
		addr += rx_len;
		count += rx_len;

		DEBUG(MTD_DEBUG_LEVEL2,
		      "%s: left:0x%x, from:0x%08x, to:0x%p, done: 0x%x\n",
		      __func__, len, (u32) addr, d, count);
	}

	*retlen = count;

	DEBUG(MTD_DEBUG_LEVEL2, "%s: %d bytes read\n", __func__, count);

	mutex_unlock(&priv->lock);

	return status;
}

/*
 * Write to the DataFlash device.
 *   to     : Start offset in flash device
 *   len    : Amount to write
 *   retlen : Amount of data actually written
 *   buf    : Buffer containing the data
 */
static int dataflash_write(struct mtd_info *mtd, loff_t to, size_t len,
			   size_t *retlen, const u_char *buf)
{
	struct dataflash *priv = mtd->priv;
	struct spi_device *spi = priv->spi;
	u32 pageaddr, addr, offset, writelen;
	size_t remaining = len;
	u_char *writebuf = (u_char *) buf;
	int status = -EINVAL;
	u_char txer[SPI_FIFOSIZE] = { 0 };
	uint8_t *command = priv->command;
	u_char *d = txer;
	u_char *s = (u_char *) buf;
	int delta = 0, l = 0, i = 0, count = 0;

	DEBUG(MTD_DEBUG_LEVEL2, "%s: write 0x%x..0x%x\n",
	      dev_name(&spi->dev), (unsigned)to, (unsigned)(to + len));

	*retlen = 0;

	/* Sanity checks */
	if (!len)
		return 0;

	if ((to + len) > mtd->size)
		return -EINVAL;

	pageaddr = ((unsigned)to / priv->page_size);
	offset = ((unsigned)to % priv->page_size);
	if (offset + len > priv->page_size)
		writelen = priv->page_size - offset;
	else
		writelen = len;

	mutex_lock(&priv->lock);

	while (remaining > 0) {
		DEBUG(MTD_DEBUG_LEVEL3, "write @ %i:%i len=%i\n",
		      pageaddr, offset, writelen);

		addr = pageaddr << priv->page_offset;

		/* (1) Maybe transfer partial page to Buffer1 */
		if (writelen != priv->page_size) {
			command[3] = OP_TRANSFER_BUF1;
			command[2] = (addr & 0x00FF0000) >> 16;
			command[1] = (addr & 0x0000FF00) >> 8;
			command[0] = 0;

			DEBUG(MTD_DEBUG_LEVEL3, "TRANSFER: (%x) %x %x %x\n",
			      command[3], command[2], command[1], command[0]);

			status = spi_read_write(spi, command, CMD_SIZE);
			if (status) {
				mutex_unlock(&priv->lock);
				return status;

			}

			(void)dataflash_waitready(spi);
		}

		count = writelen;
		while (count) {
			d = txer;
			l = count > (SPI_FIFOSIZE - CMD_SIZE) ?
			    SPI_FIFOSIZE - CMD_SIZE : count;

			delta = l % 4;
			if (delta) {
				switch (delta) {
				case 1:
					d[0] = OP_WRITE_BUFFER1;
					d[6] = (offset >> 8) & 0xff;
					d[5] = offset & 0xff;
					d[4] = *s++;
					break;
				case 2:
					d[1] = OP_WRITE_BUFFER1;
					d[7] = (offset >> 8) & 0xff;
					d[6] = offset & 0xff;
					d[5] = *s++;
					d[4] = *s++;
					break;
				case 3:
					d[2] = OP_WRITE_BUFFER1;
					d[0] = (offset >> 8) & 0xff;
					d[7] = offset & 0xff;
					d[6] = *s++;
					d[5] = *s++;
					d[4] = *s++;
					break;
				default:
					break;
				}

				DEBUG(MTD_DEBUG_LEVEL3,
				      "WRITEBUF: (%x) %x %x %x\n",
				      txer[3], txer[2], txer[1], txer[0]);

				status = spi_read_write(spi, txer,
							delta + CMD_SIZE);
				if (status) {
					mutex_unlock(&priv->lock);
					return status;
				}

				/* update */
				count -= delta;
				offset += delta;
				l -= delta;
			}

			d[3] = OP_WRITE_BUFFER1;
			d[1] = (offset >> 8) & 0xff;
			d[0] = offset & 0xff;

			for (i = 0, d += 4; i < l / 4; i++, d += 4) {
				d[3] = *s++;
				d[2] = *s++;
				d[1] = *s++;
				d[0] = *s++;
			}

			DEBUG(MTD_DEBUG_LEVEL3, "WRITEBUF: (%x) %x %x %x\n",
			      txer[3], txer[2], txer[1], txer[0]);

			status = spi_read_write(spi, txer, l + CMD_SIZE);
			if (status) {
				mutex_unlock(&priv->lock);
				return status;
			}

			/* update */
			count -= l;
			offset += l;
		}

		/* (2) Program full page via Buffer1 */
		command[3] = OP_MWERASE_BUFFER1;
		command[2] = (addr >> 16) & 0xff;
		command[1] = (addr >> 8) & 0xff;

		DEBUG(MTD_DEBUG_LEVEL3, "PROGRAM: (%x) %x %x %x\n",
		      command[3], command[2], command[1], command[0]);

		status = spi_read_write(spi, command, CMD_SIZE);
		if (status) {
			mutex_unlock(&priv->lock);
			return status;
		}

		(void)dataflash_waitready(spi);

		remaining -= writelen;
		pageaddr++;
		offset = 0;
		writebuf += writelen;
		*retlen += writelen;

		if (remaining > priv->page_size)
			writelen = priv->page_size;
		else
			writelen = remaining;
	}
	mutex_unlock(&priv->lock);

	return status;
}

/* ......................................................................... */

#ifdef CONFIG_MTD_DATAFLASH_OTP

static int dataflash_get_otp_info(struct mtd_info *mtd,
				  struct otp_info *info, size_t len)
{
	/* Report both blocks as identical:  bytes 0..64, locked.
	 * Unless the user block changed from all-ones, we can't
	 * tell whether it's still writable; so we assume it isn't.
	 */
	info->start = 0;
	info->length = 64;
	info->locked = 1;
	return sizeof(*info);
}

static ssize_t otp_read(struct spi_device *spi, unsigned base,
			uint8_t *buf, loff_t off, size_t len)
{
	struct dataflash *priv = mtd->priv;
	struct spi_device *spi = priv->spi;
	int rx_len = 0, count = 0, i = 0;
	u_char txer[SPI_FIFOSIZE];
	u_char *s = txer;
	u_char *d = NULL;
	int cmd_len = CMD_SIZE;
	int status;

	if (off > 64)
		return -EINVAL;

	if ((off + len) > 64)
		len = 64 - off;
	if (len == 0)
		return len;

	/* to make simple, we read 64 out */
	l = base + 64;

	d = kzalloc(l, GFP_KERNEL);
	if (!d)
		return -ENOMEM;

	while (l > 0) {

		rx_len = l > (SPI_FIFOSIZE - cmd_len) ?
		    SPI_FIFOSIZE - cmd_len : l;

		txer[3] = OP_READ_SECURITY;

		status = spi_read_write(spi, txer, rx_len + cmd_len);
		if (status) {
			mutex_unlock(&priv->lock);
			return status;
		}

		s = txer + cmd_len;
		for (i = rx_len; i >= 0; i -= 4, s += 4) {

			*d++ = s[3];
			*d++ = s[2];
			*d++ = s[1];
			*d++ = s[0];
		}

		/* updaate */
		l -= rx_len;
		addr += rx_len;
		count += rx_len;

		DEBUG(MTD_DEBUG_LEVEL2,
		      "%s: left:0x%x, from:0x%08x, to:0x%p, done: 0x%x\n",
		      __func__, len, (u32) addr, d, count);
	}

	d -= count;
	memcpy(buf, d + base + off, len);

	mutex_unlock(&priv->lock);

	return len;
}

static int dataflash_read_fact_otp(struct mtd_info *mtd,
				   loff_t from, size_t len, size_t *retlen,
				   u_char *buf)
{
	struct dataflash *priv = (struct dataflash *)mtd->priv;
	int status;

	/* 64 bytes, from 0..63 ... start at 64 on-chip */
	mutex_lock(&priv->lock);
	status = otp_read(priv->spi, 64, buf, from, len);
	mutex_unlock(&priv->lock);

	if (status < 0)
		return status;
	*retlen = status;
	return 0;
}

static int dataflash_read_user_otp(struct mtd_info *mtd,
				   loff_t from, size_t len, size_t *retlen,
				   u_char *buf)
{
	struct dataflash *priv = (struct dataflash *)mtd->priv;
	int status;

	/* 64 bytes, from 0..63 ... start at 0 on-chip */
	mutex_lock(&priv->lock);
	status = otp_read(priv->spi, 0, buf, from, len);
	mutex_unlock(&priv->lock);

	if (status < 0)
		return status;
	*retlen = status;
	return 0;
}

static int dataflash_write_user_otp(struct mtd_info *mtd,
				    loff_t from, size_t len, size_t *retlen,
				    u_char *buf)
{
	printk(KERN_ERR "%s not support!!\n", __func__);
	return 0;
}

static char *otp_setup(struct mtd_info *device, char revision)
{
	device->get_fact_prot_info = dataflash_get_otp_info;
	device->read_fact_prot_reg = dataflash_read_fact_otp;
	device->get_user_prot_info = dataflash_get_otp_info;
	device->read_user_prot_reg = dataflash_read_user_otp;

	/* rev c parts (at45db321c and at45db1281 only!) use a
	 * different write procedure; not (yet?) implemented.
	 */
	if (revision > 'c')
		device->write_user_prot_reg = dataflash_write_user_otp;

	return ", OTP";
}

#else

static char *otp_setup(struct mtd_info *device, char revision)
{
	return " (OTP)";
}

#endif

/* ......................................................................... */

/*
 * Register DataFlash device with MTD subsystem.
 */
static int __devinit
add_dataflash_otp(struct spi_device *spi, char *name,
		  int nr_pages, int pagesize, int pageoffset, char revision)
{
	struct dataflash *priv;
	struct mtd_info *device;
	struct flash_platform_data *pdata = spi->dev.platform_data;
	char *otp_tag = "";

	priv = kzalloc(sizeof *priv, GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	mutex_init(&priv->lock);
	priv->spi = spi;
	priv->page_size = pagesize;
	priv->page_offset = pageoffset;

	/* name must be usable with cmdlinepart */
	sprintf(priv->name, "spi%d.%d-%s",
		spi->master->bus_num, spi->chip_select, name);

	device = &priv->mtd;
	device->name = (pdata && pdata->name) ? pdata->name : priv->name;
	device->size = nr_pages * pagesize;
	device->erasesize = pagesize;
	device->writesize = pagesize;
	device->owner = THIS_MODULE;
	device->type = MTD_DATAFLASH;
	device->flags = MTD_CAP_NORFLASH;
	device->erase = dataflash_erase;
	device->read = dataflash_read;
	device->write = dataflash_write;
	device->priv = priv;

	if (revision >= 'c')
		otp_tag = otp_setup(device, revision);

	dev_info(&spi->dev, "%s (%llx KBytes) pagesize %d bytes%s\n",
		 name, DIV_ROUND_UP(device->size, 1024), pagesize, otp_tag);
	dev_set_drvdata(&spi->dev, priv);

	if (mtd_has_partitions()) {
		struct mtd_partition *parts;
		int nr_parts = 0;

#ifdef CONFIG_MTD_CMDLINE_PARTS
		static const char *part_probes[] = { "cmdlinepart", NULL, };

		nr_parts = parse_mtd_partitions(device, part_probes, &parts, 0);
#endif

		if (nr_parts <= 0 && pdata && pdata->parts) {
			parts = pdata->parts;
			nr_parts = pdata->nr_parts;
		}

		if (nr_parts > 0) {
			priv->partitioned = 1;
			return add_mtd_partitions(device, parts, nr_parts);
		}
	} else if (pdata && pdata->nr_parts)
		dev_warn(&spi->dev, "ignoring %d default partitions on %s\n",
			 pdata->nr_parts, device->name);

	return add_mtd_device(device) == 1 ? -ENODEV : 0;
}

static inline int __devinit
add_dataflash(struct spi_device *spi, char *name,
	      int nr_pages, int pagesize, int pageoffset)
{
	return add_dataflash_otp(spi, name, nr_pages, pagesize, pageoffset, 0);
}

struct flash_info {
	char *name;

	/* JEDEC id has a high byte of zero plus three data bytes:
	 * the manufacturer id, then a two byte device id.
	 */
	uint32_t jedec_id;

	/* The size listed here is what works with OP_ERASE_PAGE. */
	unsigned nr_pages;
	uint16_t pagesize;
	uint16_t pageoffset;

	uint16_t flags;
#define SUP_POW2PS	0x0002	/* supports 2^N byte pages */
#define IS_POW2PS	0x0001	/* uses 2^N byte pages */
};

static struct flash_info __devinitdata dataflash_data[] = {

	/*
	 * NOTE:  chips with SUP_POW2PS (rev D and up) need two entries,
	 * one with IS_POW2PS and the other without.  The entry with the
	 * non-2^N byte page size can't name exact chip revisions without
	 * losing backwards compatibility for cmdlinepart.
	 *
	 * These newer chips also support 128-byte security registers (with
	 * 64 bytes one-time-programmable) and software write-protection.
	 */
	{"AT45DB011B", 0x1f2200, 512, 264, 9, SUP_POW2PS},
	{"at45db011d", 0x1f2200, 512, 256, 8, SUP_POW2PS | IS_POW2PS},

	{"AT45DB021B", 0x1f2300, 1024, 264, 9, SUP_POW2PS},
	{"at45db021d", 0x1f2300, 1024, 256, 8, SUP_POW2PS | IS_POW2PS},

	{"AT45DB041x", 0x1f2400, 2048, 264, 9, SUP_POW2PS},
	{"at45db041d", 0x1f2400, 2048, 256, 8, SUP_POW2PS | IS_POW2PS},

	{"AT45DB081B", 0x1f2500, 4096, 264, 9, SUP_POW2PS},
	{"at45db081d", 0x1f2500, 4096, 256, 8, SUP_POW2PS | IS_POW2PS},

	{"AT45DB161x", 0x1f2600, 4096, 528, 10, SUP_POW2PS},
	{"at45db161d", 0x1f2600, 4096, 512, 9, SUP_POW2PS | IS_POW2PS},

	{"AT45DB321x", 0x1f2700, 8192, 528, 10, 0},	/* rev C */

	{"AT45DB321x", 0x1f2701, 8192, 528, 10, SUP_POW2PS},
	{"at45db321d", 0x1f2701, 8192, 512, 9, SUP_POW2PS | IS_POW2PS},

	{"AT45DB642x", 0x1f2800, 8192, 1056, 11, SUP_POW2PS},
	{"at45db642d", 0x1f2800, 8192, 1024, 10, SUP_POW2PS | IS_POW2PS},
};

static struct flash_info *__devinit jedec_probe(struct spi_device *spi)
{
	int tmp;
	u32 code = OP_READ_ID << 24;
	u32 jedec;
	struct flash_info *info;
	int status;

	/* JEDEC also defines an optional "extended device information"
	 * string for after vendor-specific data, after the three bytes
	 * we use here.  Supporting some chips might require using it.
	 *
	 * If the vendor ID isn't Atmel's (0x1f), assume this call failed.
	 * That's not an error; only rev C and newer chips handle it, and
	 * only Atmel sells these chips.
	 */

	tmp = spi_read_write(spi, (u8 *)&code, 4);
	if (tmp < 0) {
		DEBUG(MTD_DEBUG_LEVEL0, "%s: error %d reading JEDEC ID\n",
		      dev_name(&spi->dev), tmp);
		return NULL;
	}

	jedec = code & 0xFFFFFF;

	for (tmp = 0, info = dataflash_data;
	     tmp < ARRAY_SIZE(dataflash_data); tmp++, info++) {
		if (info->jedec_id == jedec) {
			DEBUG(MTD_DEBUG_LEVEL1, "%s: OTP, sector protect%s\n",
			      dev_name(&spi->dev), (info->flags & SUP_POW2PS)
			      ? ", binary pagesize" : "");
			if (info->flags & SUP_POW2PS) {
				status = dataflash_status(spi);
				if (status < 0) {
					DEBUG(MTD_DEBUG_LEVEL1,
					      "%s: status error %d\n",
					      dev_name(&spi->dev), status);
					return ERR_PTR(status);
				}
				if (status & 0x1) {
					if (info->flags & IS_POW2PS)
						return info;
				} else {
					if (!(info->flags & IS_POW2PS))
						return info;
				}
			}
		}
	}

	/*
	 * Treat other chips as errors ... we won't know the right page
	 * size (it might be binary) even when we can tell which density
	 * class is involved (legacy chip id scheme).
	 */
	dev_warn(&spi->dev, "JEDEC id %06x not handled\n", jedec);
	return ERR_PTR(-ENODEV);
}

/*
 * Detect and initialize DataFlash device, using JEDEC IDs on newer chips
 * or else the ID code embedded in the status bits:
 *
 *   Device      Density         ID code          #Pages PageSize  Offset
 *   AT45DB011B  1Mbit   (128K)  xx0011xx (0x0c)    512    264      9
 *   AT45DB021B  2Mbit   (256K)  xx0101xx (0x14)   1024    264      9
 *   AT45DB041B  4Mbit   (512K)  xx0111xx (0x1c)   2048    264      9
 *   AT45DB081B  8Mbit   (1M)    xx1001xx (0x24)   4096    264      9
 *   AT45DB0161B 16Mbit  (2M)    xx1011xx (0x2c)   4096    528     10
 *   AT45DB0321B 32Mbit  (4M)    xx1101xx (0x34)   8192    528     10
 *   AT45DB0642  64Mbit  (8M)    xx111xxx (0x3c)   8192   1056     11
 *   AT45DB1282  128Mbit (16M)   xx0100xx (0x10)  16384   1056     11
 */
static int __devinit dataflash_probe(struct spi_device *spi)
{
	int status;
	struct flash_info *info;

	/*
	 * Try to detect dataflash by JEDEC ID.
	 * If it succeeds we know we have either a C or D part.
	 * D will support power of 2 pagesize option.
	 * Both support the security register, though with different
	 * write procedures.
	 */
	info = jedec_probe(spi);
	if (IS_ERR(info))
		return PTR_ERR(info);
	if (info != NULL)
		return add_dataflash_otp(spi, info->name, info->nr_pages,
					 info->pagesize, info->pageoffset,
					 (info->flags & SUP_POW2PS) ? 'd' :
					 'c');

	/*
	 * Older chips support only legacy commands, identifing
	 * capacity using bits in the status byte.
	 */
	status = dataflash_status(spi);
	if (status <= 0 || status == 0xff) {
		DEBUG(MTD_DEBUG_LEVEL1, "%s: status error %d\n",
		      dev_name(&spi->dev), status);
		if (status == 0 || status == 0xff)
			status = -ENODEV;
		return status;
	}

	/* if there's a device there, assume it's dataflash.
	 * board setup should have set spi->max_speed_max to
	 * match f(car) for continuous reads, mode 0 or 3.
	 */
	switch (status & 0x3c) {
	case 0x0c:		/* 0 0 1 1 x x */
		status = add_dataflash(spi, "AT45DB011B", 512, 264, 9);
		break;
	case 0x14:		/* 0 1 0 1 x x */
		status = add_dataflash(spi, "AT45DB021B", 1024, 264, 9);
		break;
	case 0x1c:		/* 0 1 1 1 x x */
		status = add_dataflash(spi, "AT45DB041x", 2048, 264, 9);
		break;
	case 0x24:		/* 1 0 0 1 x x */
		status = add_dataflash(spi, "AT45DB081B", 4096, 264, 9);
		break;
	case 0x2c:		/* 1 0 1 1 x x */
		status = add_dataflash(spi, "AT45DB161x", 4096, 528, 10);
		break;
	case 0x34:		/* 1 1 0 1 x x */
		status = add_dataflash(spi, "AT45DB321x", 8192, 528, 10);
		break;
	case 0x38:		/* 1 1 1 x x x */
	case 0x3c:
		status = add_dataflash(spi, "AT45DB642x", 8192, 1056, 11);
		break;
		/* obsolete AT45DB1282 not (yet?) supported */
	default:
		DEBUG(MTD_DEBUG_LEVEL1, "%s: unsupported device (%x)\n",
		      dev_name(&spi->dev), status & 0x3c);
		status = -ENODEV;
	}

	if (status < 0)
		DEBUG(MTD_DEBUG_LEVEL1, "%s: add_dataflash --> %d\n",
		      dev_name(&spi->dev), status);

	return status;
}

static int __devexit dataflash_remove(struct spi_device *spi)
{
	struct dataflash *flash = dev_get_drvdata(&spi->dev);
	int status;

	DEBUG(MTD_DEBUG_LEVEL1, "%s: remove\n", dev_name(&spi->dev));

	if (mtd_has_partitions() && flash->partitioned)
		status = del_mtd_partitions(&flash->mtd);
	else
		status = del_mtd_device(&flash->mtd);
	if (status == 0)
		kfree(flash);
	return status;
}

static struct spi_driver dataflash_driver = {
	.driver = {
		   .name = "mxc_dataflash",
		   .bus = &spi_bus_type,
		   .owner = THIS_MODULE,
		   },

	.probe = dataflash_probe,
	.remove = __devexit_p(dataflash_remove),

	/* FIXME:  investigate suspend and resume... */
};

static int __init dataflash_init(void)
{
	return spi_register_driver(&dataflash_driver);
}

module_init(dataflash_init);

static void __exit dataflash_exit(void)
{
	spi_unregister_driver(&dataflash_driver);
}

module_exit(dataflash_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MTD DataFlash driver");
