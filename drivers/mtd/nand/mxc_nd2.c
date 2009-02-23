/*
 * Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mtd/partitions.h>
#include <asm/mach/flash.h>
#include <asm/io.h>
#include "mxc_nd2.h"

#define DVR_VER "2.5"

/* Global address Variables */
static u32 nfc_axi_base, nfc_ip_base;

struct mxc_mtd_s {
	struct mtd_info mtd;
	struct nand_chip nand;
	struct mtd_partition *parts;
	struct device *dev;
};

static struct mxc_mtd_s *mxc_nand_data;

/*
 * Define delays in microsec for NAND device operations
 */
#define TROP_US_DELAY   2000

struct nand_info {
	bool bStatusRequest;
	u16 colAddr;
};

static struct nand_info g_nandfc_info;

#ifdef CONFIG_MTD_NAND_MXC_SWECC
static int hardware_ecc = 0;
#else
static int hardware_ecc = 1;
#endif

static u8 num_of_interleave = 1;

static u8 *data_buf;
static u8 *oob_buf;

static int g_page_mask;

static struct clk *nfc_clk;

/*
 * OOB placement block for use with hardware ecc generation
 */
static struct nand_ecclayout nand_hw_eccoob_512 = {
	.eccbytes = 9,
	.eccpos = {7, 8, 9, 10, 11, 12, 13, 14, 15},
	.oobavail = 4,
	.oobfree = {{0, 4}}
};

static struct nand_ecclayout nand_hw_eccoob_2k = {
	.eccbytes = 9,
	.eccpos = {7, 8, 9, 10, 11, 12, 13, 14, 15},
	.oobavail = 4,
	.oobfree = {{2, 4}}
};

static struct nand_ecclayout nand_hw_eccoob_4k = {
	.eccbytes = 9,
	.eccpos = {7, 8, 9, 10, 11, 12, 13, 14, 15},
	.oobavail = 4,
	.oobfree = {{2, 4}}
};

/*!
 * @defgroup NAND_MTD NAND Flash MTD Driver for MXC processors
 */

/*!
 * @file mxc_nd2.c
 *
 * @brief This file contains the hardware specific layer for NAND Flash on
 * MXC processor
 *
 * @ingroup NAND_MTD
 */

#ifdef CONFIG_MTD_PARTITIONS
static const char *part_probes[] = { "RedBoot", "cmdlinepart", NULL };
#endif

static wait_queue_head_t irq_waitq;

static irqreturn_t mxc_nfc_irq(int irq, void *dev_id)
{
	/* Disable Interuupt */
	raw_write(raw_read(REG_NFC_INTRRUPT) | NFC_INT_MSK, REG_NFC_INTRRUPT);
	wake_up(&irq_waitq);

	return IRQ_HANDLED;
}

/*
 * Functions to transfer data to/from spare erea.
 */
static void
copy_spare(struct mtd_info *mtd, void *pbuf, void *pspare, int len, bool bfrom)
{
	u16 i, j;
	u16 m = mtd->oobsize;
	u16 n = mtd->writesize >> 9;
	u8 *d = (u8 *) pbuf;
	u8 *s = (u8 *) pspare;
	u16 t = SPARE_LEN;

	m /= num_of_interleave;
	n /= num_of_interleave;

	j = (m / n >> 1) << 1;

	if (bfrom) {
		for (i = 0; i < n - 1; i++)
			memcpy(&d[i * j], &s[i * t], j);

		/* the last section */
		memcpy(&d[i * j], &s[i * t], len - i * j);
	} else {
		for (i = 0; i < n - 1; i++)
			memcpy(&s[i * t], &d[i * j], j);

		/* the last section */
		memcpy(&s[i * t], &d[i * j], len - i * j);
	}
}

/*!
 * This function polls the NFC to wait for the basic operation to complete by
 * checking the INT bit of config2 register.
 *
 * @param       maxRetries     number of retry attempts (separated by 1 us)
 * @param       useirq         True if IRQ should be used rather than polling
 */
static void wait_op_done(int maxRetries, bool useirq)
{
	if (useirq) {
		if ((raw_read(REG_NFC_OPS_STAT) & NFC_OPS_STAT) == 0) {
			/* Enable Interuupt */
			raw_write(raw_read(REG_NFC_INTRRUPT) & ~NFC_INT_MSK,
				  REG_NFC_INTRRUPT);
			wait_event(irq_waitq,
				   (raw_read(REG_NFC_OPS_STAT) & NFC_OPS_STAT));
		}
		WRITE_NFC_IP_REG((raw_read(REG_NFC_OPS_STAT) &
				  ~NFC_OPS_STAT), REG_NFC_OPS_STAT);
	} else {
		while (1) {
			maxRetries--;
			if (raw_read(REG_NFC_OPS_STAT) & NFC_OPS_STAT) {
				WRITE_NFC_IP_REG((raw_read(REG_NFC_OPS_STAT) &
						  ~NFC_OPS_STAT),
						 REG_NFC_OPS_STAT);
				break;
			}
			udelay(1);
			if (maxRetries <= 0) {
				DEBUG(MTD_DEBUG_LEVEL0, "%s(%d): INT not set\n",
				      __func__, __LINE__);
			}
		}
	}
}

static inline void send_atomic_cmd(u16 cmd, bool useirq)
{
	/* fill command */
	raw_write(cmd, REG_NFC_FLASH_CMD);

	/* clear status */
	ACK_OPS;

	/* send out command */
	raw_write(NFC_CMD, REG_NFC_OPS);

	/* Wait for operation to complete */
	wait_op_done(TROP_US_DELAY, useirq);
}

static void mxc_do_addr_cycle(struct mtd_info *mtd, int column, int page_addr);
static int mxc_check_ecc_status(struct mtd_info *mtd);

#ifdef NFC_AUTO_MODE_ENABLE
/*!
 * This function handle the interleave related work
 * @param	mtd	mtd info
 * @param	cmd	command
 */
static void auto_cmd_interleave(struct mtd_info *mtd, u16 cmd)
{
	u32 i;
	u32 j = num_of_interleave;
	struct nand_chip *this = mtd->priv;
	u32 addr_low = raw_read(NFC_FLASH_ADDR0);
	u32 addr_high = raw_read(NFC_FLASH_ADDR8);
	u32 page_addr = addr_low >> 16 | addr_high << 16;
	u8 *dbuf = data_buf;
	u8 *obuf = oob_buf;
	u32 dlen = mtd->writesize / j;
	u32 olen = mtd->oobsize / j;

	/* adjust the addr value
	 * since ADD_OP mode is 01
	 */
	if (j > 1)
		page_addr *= j;
	else
		page_addr *= this->numchips;

	switch (cmd) {
	case NAND_CMD_PAGEPROG:
		for (i = 0; i < j; i++) {
			/* reset addr cycle */
			if (j > 1)
				mxc_do_addr_cycle(mtd, 0, page_addr++);

			/* data transfer */
			memcpy(MAIN_AREA0, dbuf, dlen);
			copy_spare(mtd, obuf, SPARE_AREA0, olen, false);

			/* update the value */
			dbuf += dlen;
			obuf += olen;

			NFC_SET_RBA(0);
			ACK_OPS;
			raw_write(NFC_AUTO_PROG, REG_NFC_OPS);

			/* wait auto_prog_done bit set */
			while (!(raw_read(REG_NFC_OPS_STAT) & NFC_OP_DONE)) ;
		}

		wait_op_done(TROP_US_DELAY, false);
		while (!(raw_read(REG_NFC_OPS_STAT) & NFC_RB)) ;

		break;
	case NAND_CMD_READSTART:
		for (i = 0; i < j; i++) {
			/* reset addr cycle */
			if (j > 1)
				mxc_do_addr_cycle(mtd, 0, page_addr++);

			NFC_SET_RBA(0);
			ACK_OPS;
			raw_write(NFC_AUTO_READ, REG_NFC_OPS);
			wait_op_done(TROP_US_DELAY, false);

			/* check ecc error */
			mxc_check_ecc_status(mtd);

			/* data transfer */
			memcpy(dbuf, MAIN_AREA0, dlen);
			copy_spare(mtd, obuf, SPARE_AREA0, olen, true);

			/* update the value */
			dbuf += dlen;
			obuf += olen;
		}
		break;
	case NAND_CMD_ERASE2:
		for (i = 0; i < j; i++) {
			if (!i) {
				page_addr = addr_low;
				page_addr *= (j > 1 ? j : this->numchips);
			}
			mxc_do_addr_cycle(mtd, -1, page_addr++);
			ACK_OPS;
			raw_write(NFC_AUTO_ERASE, REG_NFC_OPS);
			wait_op_done(TROP_US_DELAY, true);
		}
		break;
	case NAND_CMD_RESET:
		for (i = 0; i < j; i++) {
			if (j > 1)
				NFC_SET_NFC_ACTIVE_CS(i);
			send_atomic_cmd(cmd, false);
		}
		break;
	default:
		break;
	}
}
#endif

static void send_addr(u16 addr, bool useirq);

/*!
 * This function issues the specified command to the NAND device and
 * waits for completion.
 *
 * @param       cmd     command for NAND Flash
 * @param       useirq  True if IRQ should be used rather than polling
 */
static void send_cmd(struct mtd_info *mtd, u16 cmd, bool useirq)
{
	DEBUG(MTD_DEBUG_LEVEL3, "send_cmd(0x%x, %d)\n", cmd, useirq);

#ifdef NFC_AUTO_MODE_ENABLE
	switch (cmd) {
	case NAND_CMD_READ0:
	case NAND_CMD_READOOB:
		raw_write(NAND_CMD_READ0, REG_NFC_FLASH_CMD);
		break;
	case NAND_CMD_SEQIN:
	case NAND_CMD_ERASE1:
		raw_write(cmd, REG_NFC_FLASH_CMD);
		break;
	case NAND_CMD_PAGEPROG:
	case NAND_CMD_ERASE2:
	case NAND_CMD_READSTART:
		raw_write(raw_read(REG_NFC_FLASH_CMD) | cmd << NFC_CMD_1_SHIFT,
			  REG_NFC_FLASH_CMD);
		auto_cmd_interleave(mtd, cmd);
		break;
	case NAND_CMD_READID:
		send_atomic_cmd(cmd, useirq);
		send_addr(0, false);
		break;
	case NAND_CMD_RESET:
		auto_cmd_interleave(mtd, cmd);
	case NAND_CMD_STATUS:
		break;
	default:
		break;
	}
#else
	send_atomic_cmd(cmd, useirq);
#endif
}

/*!
 * This function sends an address (or partial address) to the
 * NAND device.  The address is used to select the source/destination for
 * a NAND command.
 *
 * @param       addr    address to be written to NFC.
 * @param       useirq  True if IRQ should be used rather than polling
 */
static void send_addr(u16 addr, bool useirq)
{
	DEBUG(MTD_DEBUG_LEVEL3, "send_addr(0x%x %d)\n", addr, useirq);

	/* fill address */
	raw_write((addr << NFC_FLASH_ADDR_SHIFT), REG_NFC_FLASH_ADDR);

	/* clear status */
	ACK_OPS;

	/* send out address */
	raw_write(NFC_ADDR, REG_NFC_OPS);

	/* Wait for operation to complete */
	wait_op_done(TROP_US_DELAY, useirq);
}

/*!
 * This function requests the NFC to initate the transfer
 * of data currently in the NFC RAM buffer to the NAND device.
 *
 * @param	buf_id	      Specify Internal RAM Buffer number
 */
static void send_prog_page(u8 buf_id)
{
#ifndef NFC_AUTO_MODE_ENABLE
	DEBUG(MTD_DEBUG_LEVEL3, "%s\n", __FUNCTION__);

	/* set ram buffer id */
	NFC_SET_RBA(buf_id);

	/* clear status */
	ACK_OPS;

	/* transfer data from NFC ram to nand */
	raw_write(NFC_INPUT, REG_NFC_OPS);

	/* Wait for operation to complete */
	wait_op_done(TROP_US_DELAY, false);
#endif
}

/*!
 * This function requests the NFC to initated the transfer
 * of data from the NAND device into in the NFC ram buffer.
 *
 * @param  	buf_id		Specify Internal RAM Buffer number
 */
static void send_read_page(u8 buf_id)
{
#ifndef NFC_AUTO_MODE_ENABLE
	DEBUG(MTD_DEBUG_LEVEL3, "%s(%d)\n", __FUNCTION__, buf_id);

	/* set ram buffer id */
	NFC_SET_RBA(buf_id);

	/* clear status */
	ACK_OPS;

	/* transfer data from nand to NFC ram */
	raw_write(NFC_OUTPUT, REG_NFC_OPS);

	/* Wait for operation to complete */
	wait_op_done(TROP_US_DELAY, false);
#endif
}

/*!
 * This function requests the NFC to perform a read of the
 * NAND device ID.
 */
static void send_read_id(void)
{
	/* Set RBA bits for BUFFER0 */
	NFC_SET_RBA(0);

	/* clear status */
	ACK_OPS;

	/* Read ID into main buffer */
	raw_write(NFC_ID, REG_NFC_OPS);

	/* Wait for operation to complete */
	wait_op_done(TROP_US_DELAY, false);

}

#ifdef NFC_AUTO_MODE_ENABLE
static inline void read_dev_status(u16 *status)
{
	u32 mask = 0xFF << 16;

	/* clear status */
	ACK_OPS;

	do {
		/* send auto read status command */
		raw_write(NFC_AUTO_STATE, REG_NFC_OPS);
		if (cpu_is_mx51_rev(CHIP_REV_2_0) == 1)
			wait_op_done(TROP_US_DELAY, false);
		*status = (raw_read(NFC_CONFIG1) & mask) >> 16;
	} while ((*status & NAND_STATUS_READY) == 0);
}
#endif

/*!
 * This function requests the NFC to perform a read of the
 * NAND device status and returns the current status.
 *
 * @return  device status
 */
static u16 get_dev_status(void)
{
#ifdef NFC_AUTO_MODE_ENABLE
	int i;
	u16 status = 0;
	for (i = 0; i < num_of_interleave; i++) {

		/* set ative cs */
		NFC_SET_NFC_ACTIVE_CS(i);

		/* FIXME, NFC Auto erase may have
		 * problem, have to pollingit until
		 * the nand get idle, otherwise
		 * it may get error
		 */
		read_dev_status(&status);
		if (status & NAND_STATUS_FAIL)
			break;
	}

	return status;
#else
	volatile u16 *mainBuf = MAIN_AREA1;
	u8 val = 1;
	u16 ret;

	/* Set ram buffer id */
	NFC_SET_RBA(val);

	/* clear status */
	ACK_OPS;

	/* Read status into main buffer */
	raw_write(NFC_STATUS, REG_NFC_OPS);

	/* Wait for operation to complete */
	wait_op_done(TROP_US_DELAY, false);

	/* Status is placed in first word of main buffer */
	/* get status, then recovery area 1 data */
	ret = *mainBuf;

	return ret;
#endif
}

static void mxc_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
	raw_write((raw_read(REG_NFC_ECC_EN) | NFC_ECC_EN), REG_NFC_ECC_EN);
	return;
}

/*
 * Function to record the ECC corrected/uncorrected errors resulted
 * after a page read. This NFC detects and corrects upto to 4 symbols
 * of 9-bits each.
 */
static int mxc_check_ecc_status(struct mtd_info *mtd)
{
	u32 ecc_stat, err;
	int no_subpages = 1;
	int ret = 0;
	u8 ecc_bit_mask, err_limit;

	ecc_bit_mask = (IS_4BIT_ECC ? 0x7 : 0xf);
	err_limit = (IS_4BIT_ECC ? 0x4 : 0x8);

	no_subpages = mtd->writesize >> 9;

	no_subpages /= num_of_interleave;

	ecc_stat = GET_NFC_ECC_STATUS();
	do {
		err = ecc_stat & ecc_bit_mask;
		if (err > err_limit) {
			mtd->ecc_stats.failed++;
			printk(KERN_WARNING "UnCorrectable RS-ECC Error\n");
			return -1;
		} else {
			ret += err;
		}
		ecc_stat >>= 4;
	} while (--no_subpages);

	mtd->ecc_stats.corrected += ret;
	pr_debug("%d Symbol Correctable RS-ECC Error\n", ret);

	return ret;
}

/*
 * Function to correct the detected errors. This NFC corrects all the errors
 * detected. So this function just return 0.
 */
static int mxc_nand_correct_data(struct mtd_info *mtd, u_char * dat,
				 u_char * read_ecc, u_char * calc_ecc)
{
	return 0;
}

/*
 * Function to calculate the ECC for the data to be stored in the Nand device.
 * This NFC has a hardware RS(511,503) ECC engine together with the RS ECC
 * CONTROL blocks are responsible for detection  and correction of up to
 * 8 symbols of 9 bits each in 528 byte page.
 * So this function is just return 0.
 */

static int mxc_nand_calculate_ecc(struct mtd_info *mtd, const u_char * dat,
				  u_char * ecc_code)
{
	return 0;
}

/*!
 * This function id is used to read the data buffer from the NAND Flash. To
 * read the data from NAND Flash first the data output cycle is initiated by
 * the NFC, which copies the data to RAMbuffer. This data of length \b len is
 * then copied to buffer \b buf.
 *
 * @param       mtd     MTD structure for the NAND Flash
 * @param       buf     data to be read from NAND Flash
 * @param       len     number of bytes to be read
 */
static void mxc_nand_read_buf(struct mtd_info *mtd, u_char * buf, int len)
{
	u16 col = g_nandfc_info.colAddr;

	if (mtd->writesize) {

		int j = mtd->writesize - col;
		int n = mtd->oobsize + j;

		n = min(n, len);

		if (j > 0) {
			if (n > j) {
				memcpy(buf, &data_buf[col], j);
				memcpy(buf + j, &oob_buf[0], n - j);
			} else {
				memcpy(buf, &data_buf[col], n);
			}
		} else {
			col -= mtd->writesize;
			memcpy(buf, &oob_buf[col], len);
		}

		/* update */
		g_nandfc_info.colAddr += n;

	} else {
		/* At flash identify phase,
		 * mtd->writesize has not been
		 * set correctly, it should
		 * be zero.And len will less 2
		 */
		memcpy(buf, &data_buf[col], len);

		/* update */
		g_nandfc_info.colAddr += len;
	}

}

/*!
 * This function reads byte from the NAND Flash
 *
 * @param       mtd     MTD structure for the NAND Flash
 *
 * @return    data read from the NAND Flash
 */
static uint8_t mxc_nand_read_byte(struct mtd_info *mtd)
{
	uint8_t ret;

	/* Check for status request */
	if (g_nandfc_info.bStatusRequest) {
		return (get_dev_status() & 0xFF);
	}

	mxc_nand_read_buf(mtd, &ret, 1);

	return ret;
}

/*!
  * This function reads word from the NAND Flash
  *
  * @param     mtd     MTD structure for the NAND Flash
  *
  * @return    data read from the NAND Flash
  */
static u16 mxc_nand_read_word(struct mtd_info *mtd)
{
	u16 ret;

	mxc_nand_read_buf(mtd, (uint8_t *) &ret, sizeof(u16));

	return ret;
}

/*!
 * This function reads byte from the NAND Flash
 *
 * @param     mtd     MTD structure for the NAND Flash
 *
 * @return    data read from the NAND Flash
 */
static u_char mxc_nand_read_byte16(struct mtd_info *mtd)
{
	/* Check for status request */
	if (g_nandfc_info.bStatusRequest) {
		return (get_dev_status() & 0xFF);
	}

	return mxc_nand_read_word(mtd) & 0xFF;
}

/*!
 * This function writes data of length \b len from buffer \b buf to the NAND
 * internal RAM buffer's MAIN area 0.
 *
 * @param       mtd     MTD structure for the NAND Flash
 * @param       buf     data to be written to NAND Flash
 * @param       len     number of bytes to be written
 */
static void mxc_nand_write_buf(struct mtd_info *mtd,
			       const u_char * buf, int len)
{
	u16 col = g_nandfc_info.colAddr;
	int j = mtd->writesize - col;
	int n = mtd->oobsize + j;

	n = min(n, len);

	if (j > 0) {
		if (n > j) {
			memcpy(&data_buf[col], buf, j);
			memcpy(&oob_buf[0], buf + j, n - j);
		} else {
			memcpy(&data_buf[col], buf, n);
		}
	} else {
		col -= mtd->writesize;
		memcpy(&oob_buf[col], buf, len);
	}

	/* update */
	g_nandfc_info.colAddr += n;
}

/*!
 * This function is used by the upper layer to verify the data in NAND Flash
 * with the data in the \b buf.
 *
 * @param       mtd     MTD structure for the NAND Flash
 * @param       buf     data to be verified
 * @param       len     length of the data to be verified
 *
 * @return      -EFAULT if error else 0
 *
 */
static int mxc_nand_verify_buf(struct mtd_info *mtd, const u_char * buf,
			       int len)
{
	u_char *s = data_buf;

	const u_char *p = buf;

	for (; len > 0; len--) {
		if (*p++ != *s++)
			return -EFAULT;
	}

	return 0;
}

/*!
 * This function is used by upper layer for select and deselect of the NAND
 * chip
 *
 * @param       mtd     MTD structure for the NAND Flash
 * @param       chip    val indicating select or deselect
 */
static void mxc_nand_select_chip(struct mtd_info *mtd, int chip)
{

	switch (chip) {
	case -1:
		/* Disable the NFC clock */
		clk_disable(nfc_clk);
		break;
	case 0 ... 7:
		/* Enable the NFC clock */
		clk_enable(nfc_clk);

		NFC_SET_NFC_ACTIVE_CS(chip);
		break;

	default:
		break;
	}
}

/*
 * Function to perform the address cycles.
 */
static void mxc_do_addr_cycle(struct mtd_info *mtd, int column, int page_addr)
{
#ifdef NFC_AUTO_MODE_ENABLE

	if (page_addr != -1 && column != -1) {
		u32 mask = 0xFFFF;
		/* the column address */
		raw_write(column & mask, NFC_FLASH_ADDR0);
		raw_write((raw_read(NFC_FLASH_ADDR0) |
			   ((page_addr & mask) << 16)), NFC_FLASH_ADDR0);
		/* the row address */
		raw_write(((raw_read(NFC_FLASH_ADDR8) & (mask << 16)) |
			   ((page_addr & (mask << 16)) >> 16)),
			  NFC_FLASH_ADDR8);
	} else if (page_addr != -1) {
		raw_write(page_addr, NFC_FLASH_ADDR0);
	}

	DEBUG(MTD_DEBUG_LEVEL3,
	      "AutoMode:the ADDR REGS value is (0x%x, 0x%x)\n",
	      raw_read(NFC_FLASH_ADDR0), raw_read(NFC_FLASH_ADDR8));
#else

	u32 page_mask = g_page_mask;

	if (column != -1) {
		send_addr(column & 0xFF, false);
		if (IS_2K_PAGE_NAND) {
			/* another col addr cycle for 2k page */
			send_addr((column >> 8) & 0xF, false);
		} else if (IS_4K_PAGE_NAND) {
			/* another col addr cycle for 4k page */
			send_addr((column >> 8) & 0x1F, false);
		}
	}
	if (page_addr != -1) {
		do {
			send_addr((page_addr & 0xff), false);
			page_mask >>= 8;
			page_addr >>= 8;
		} while (page_mask != 0);
	}
#endif
}

/*!
 * This function is used by the upper layer to write command to NAND Flash for
 * different operations to be carried out on NAND Flash
 *
 * @param       mtd             MTD structure for the NAND Flash
 * @param       command         command for NAND Flash
 * @param       column          column offset for the page read
 * @param       page_addr       page to be read from NAND Flash
 */
static void mxc_nand_command(struct mtd_info *mtd, unsigned command,
			     int column, int page_addr)
{
	bool useirq = false;

	DEBUG(MTD_DEBUG_LEVEL3,
	      "mxc_nand_command (cmd = 0x%x, col = 0x%x, page = 0x%x)\n",
	      command, column, page_addr);
	/*
	 * Reset command state information
	 */
	g_nandfc_info.bStatusRequest = false;

	/*
	 * Command pre-processing step
	 */
	switch (command) {
	case NAND_CMD_STATUS:
		g_nandfc_info.colAddr = 0;
		g_nandfc_info.bStatusRequest = true;
		break;

	case NAND_CMD_READ0:
		g_nandfc_info.colAddr = column;
		break;

	case NAND_CMD_READOOB:
		g_nandfc_info.colAddr = column;
		command = NAND_CMD_READ0;
		break;

	case NAND_CMD_SEQIN:
		if (column != 0) {

			/* FIXME: before send SEQIN command for
			 * partial write,We need read one page out.
			 * FSL NFC does not support partial write
			 * It alway send out 512+ecc+512+ecc ...
			 * for large page nand flash. But for small
			 * page nand flash, it did support SPARE
			 * ONLY operation. But to make driver
			 * simple. We take the same as large page,read
			 * whole page out and update. As for MLC nand
			 * NOP(num of operation) = 1. Partial written
			 * on one programed page is not allowed! We
			 * can't limit it on the driver, it need the
			 * upper layer applicaiton take care it
			 */

			mxc_nand_command(mtd, NAND_CMD_READ0, 0, page_addr);
		}

		g_nandfc_info.colAddr = column;
		break;

	case NAND_CMD_PAGEPROG:
#ifndef NFC_AUTO_MODE_ENABLE
		/* FIXME:the NFC interal buffer
		 * access has some limitation, it
		 * does not allow byte access. To
		 * make the code simple and ease use
		 * not every time check the address
		 * alignment.Use the temp buffer
		 * to accomadate the data.since We
		 * know data_buf will be at leat 4
		 * byte alignment, so we can use
		 * memcpy safely
		 */
		memcpy(MAIN_AREA0, data_buf, mtd->writesize);
		copy_spare(mtd, oob_buf, SPARE_AREA0, mtd->oobsize, false);
#endif

		if (IS_LARGE_PAGE_NAND)
			PROG_PAGE();
		else
			send_prog_page(0);

		useirq = true;

		break;

	case NAND_CMD_ERASE1:
		break;
	case NAND_CMD_ERASE2:
		useirq = true;

		break;
	}

	/*
	 * Write out the command to the device.
	 */
	send_cmd(mtd, command, useirq);

	mxc_do_addr_cycle(mtd, column, page_addr);

	/*
	 * Command post-processing step
	 */
	switch (command) {

	case NAND_CMD_READOOB:
	case NAND_CMD_READ0:
		if (IS_LARGE_PAGE_NAND) {
			/* send read confirm command */
			send_cmd(mtd, NAND_CMD_READSTART, false);
			/* read for each AREA */
			READ_PAGE();
		} else {
			send_read_page(0);
		}

#ifndef NFC_AUTO_MODE_ENABLE
		/* FIXME, the NFC interal buffer
		 * access has some limitation, it
		 * does not allow byte access. To
		 * make the code simple and ease use
		 * not every time check the address
		 * alignment.Use the temp buffer
		 * to accomadate the data.since We
		 * know data_buf will be at leat 4
		 * byte alignment, so we can use
		 * memcpy safely
		 */
		memcpy(data_buf, MAIN_AREA0, mtd->writesize);
		copy_spare(mtd, oob_buf, SPARE_AREA0, mtd->oobsize, true);
#endif

		break;

	case NAND_CMD_READID:
		send_read_id();
		g_nandfc_info.colAddr = column;
		memcpy(data_buf, MAIN_AREA0, 2048);

		break;
	}
}

static int mxc_nand_read_oob(struct mtd_info *mtd,
			     struct nand_chip *chip, int page, int sndcmd)
{
	if (sndcmd) {

		chip->cmdfunc(mtd, NAND_CMD_READ0, 0x00, page);
		sndcmd = 0;
	}

	memcpy(chip->oob_poi, oob_buf, mtd->oobsize);

	return sndcmd;
}

static int mxc_nand_read_page(struct mtd_info *mtd, struct nand_chip *chip,
			      uint8_t * buf)
{

#ifndef NFC_AUTO_MODE_ENABLE
	mxc_check_ecc_status(mtd);
#endif

	memcpy(buf, data_buf, mtd->writesize);
	memcpy(chip->oob_poi, oob_buf, mtd->oobsize);

	return 0;
}

static void mxc_nand_write_page(struct mtd_info *mtd, struct nand_chip *chip,
				const uint8_t * buf)
{
	memcpy(data_buf, buf, mtd->writesize);
	memcpy(oob_buf, chip->oob_poi, mtd->oobsize);

}

/* Define some generic bad / good block scan pattern which are used
 * while scanning a device for factory marked good / bad blocks. */
static uint8_t scan_ff_pattern[] = { 0xff, 0xff };

static struct nand_bbt_descr smallpage_memorybased = {
	.options = NAND_BBT_SCAN2NDPAGE,
	.offs = 5,
	.len = 1,
	.pattern = scan_ff_pattern
};

static struct nand_bbt_descr largepage_memorybased = {
	.options = 0,
	.offs = 0,
	.len = 2,
	.pattern = scan_ff_pattern
};

/* Generic flash bbt decriptors
*/
static uint8_t bbt_pattern[] = { 'B', 'b', 't', '0' };
static uint8_t mirror_pattern[] = { '1', 't', 'b', 'B' };

static struct nand_bbt_descr bbt_main_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
	    | NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
	.offs = 0,
	.len = 4,
	.veroffs = 4,
	.maxblocks = 4,
	.pattern = bbt_pattern
};

static struct nand_bbt_descr bbt_mirror_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
	    | NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
	.offs = 0,
	.len = 4,
	.veroffs = 4,
	.maxblocks = 4,
	.pattern = mirror_pattern
};

static int mxc_nand_scan_bbt(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;

	g_page_mask = this->pagemask;

	if (IS_2K_PAGE_NAND) {
		NFC_SET_NFMS(1 << NFMS_NF_PG_SZ);
		this->ecc.layout = &nand_hw_eccoob_2k;
	} else if (IS_4K_PAGE_NAND) {
		NFC_SET_NFMS(1 << NFMS_NF_PG_SZ);
		this->ecc.layout = &nand_hw_eccoob_4k;
	} else {
		this->ecc.layout = &nand_hw_eccoob_512;
	}

	/* reconfig for interleave mode */
#ifdef NFC_AUTO_MODE_ENABLE
	if (this->numchips > 1) {
		num_of_interleave = this->numchips;
		this->numchips = 1;

		/* FIXEME:need remove it
		 * when kernel support
		 * 4G larger space
		 */
		mtd->size = this->chipsize;
		mtd->erasesize *= num_of_interleave;
		mtd->writesize *= num_of_interleave;
		mtd->oobsize *= num_of_interleave;
		this->page_shift = ffs(mtd->writesize) - 1;
		this->bbt_erase_shift =
		    this->phys_erase_shift = ffs(mtd->erasesize) - 1;
		this->chip_shift = ffs(this->chipsize) - 1;
		this->oob_poi = this->buffers->databuf + mtd->writesize;
	}
#endif
	/* propagate ecc.layout to mtd_info */
	mtd->ecclayout = this->ecc.layout;

	/* jffs2 not write oob */
	mtd->flags &= ~MTD_OOB_WRITEABLE;

	/* use flash based bbt */
	this->bbt_td = &bbt_main_descr;
	this->bbt_md = &bbt_mirror_descr;

	/* update flash based bbt */
	this->options |= NAND_USE_FLASH_BBT;

	if (!this->badblock_pattern) {
		this->badblock_pattern = (mtd->writesize > 512) ?
		    &largepage_memorybased : &smallpage_memorybased;
	}

	/* Build bad block table */
	return nand_scan_bbt(mtd, this->badblock_pattern);
}

static void mxc_nfc_init(void)
{
	/* Disable interrupt */
	raw_write((raw_read(REG_NFC_INTRRUPT) | NFC_INT_MSK), REG_NFC_INTRRUPT);

	/* disable spare enable */
	raw_write(raw_read(REG_NFC_SP_EN) & ~NFC_SP_EN, REG_NFC_SP_EN);

	/* Unlock the internal RAM Buffer */
	raw_write(NFC_SET_BLS(NFC_BLS_UNLCOKED), REG_NFC_BLS);

	/* Blocks to be unlocked */
	UNLOCK_ADDR(0x0, 0xFFFF);

	/* Unlock Block Command for given address range */
	raw_write(NFC_SET_WPC(NFC_WPC_UNLOCK), REG_NFC_WPC);

	/* Enable symetric mode by default except mx37TO1.0 */
	if (!(cpu_is_mx37_rev(CHIP_REV_1_0) == 1))
		raw_write(raw_read(REG_NFC_ONE_CYCLE) |
			  NFC_ONE_CYCLE, REG_NFC_ONE_CYCLE);

}

static int mxc_alloc_buf(void)
{
	int err = 0;

	data_buf = kzalloc(NAND_MAX_PAGESIZE, GFP_KERNEL);
	if (!data_buf) {
		printk(KERN_ERR "%s: failed to allocate data_buf\n", __func__);
		err = -ENOMEM;
		goto out;
	}
	oob_buf = kzalloc(NAND_MAX_OOBSIZE, GFP_KERNEL);
	if (!oob_buf) {
		printk(KERN_ERR "%s: failed to allocate oob_buf\n", __func__);
		err = -ENOMEM;
		goto out;
	}

      out:
	return err;
}

static void mxc_free_buf(void)
{
	kfree(data_buf);
	kfree(oob_buf);
}

/*!
 * This function is called during the driver binding process.
 *
 * @param   pdev  the device structure used to store device specific
 *                information that is used by the suspend, resume and
 *                remove functions
 *
 * @return  The function always returns 0.
 */
static int __init mxcnd_probe(struct platform_device *pdev)
{
	struct nand_chip *this;
	struct mtd_info *mtd;
	struct flash_platform_data *flash = pdev->dev.platform_data;
	int nr_parts = 0, err = 0;

	nfc_axi_base = IO_ADDRESS(NFC_AXI_BASE_ADDR);
	nfc_ip_base = IO_ADDRESS(NFC_BASE_ADDR);

	/* init the nfc */
	mxc_nfc_init();

	/* init data buf */
	if (mxc_alloc_buf())
		goto out;

	/* Allocate memory for MTD device structure and private data */
	mxc_nand_data = kzalloc(sizeof(struct mxc_mtd_s), GFP_KERNEL);
	if (!mxc_nand_data) {
		printk(KERN_ERR "%s: failed to allocate mtd_info\n",
		       __FUNCTION__);
		err = -ENOMEM;
		goto out;
	}

	memset((char *)&g_nandfc_info, 0, sizeof(g_nandfc_info));

	mxc_nand_data->dev = &pdev->dev;
	/* structures must be linked */
	this = &mxc_nand_data->nand;
	mtd = &mxc_nand_data->mtd;
	mtd->priv = this;
	mtd->owner = THIS_MODULE;

	this->priv = mxc_nand_data;
	this->cmdfunc = mxc_nand_command;
	this->select_chip = mxc_nand_select_chip;
	this->read_byte = mxc_nand_read_byte;
	this->read_word = mxc_nand_read_word;
	this->write_buf = mxc_nand_write_buf;
	this->read_buf = mxc_nand_read_buf;
	this->verify_buf = mxc_nand_verify_buf;
	this->scan_bbt = mxc_nand_scan_bbt;

	/* NAND bus width determines access funtions used by upper layer */
	if (flash->width == 2) {
		this->read_byte = mxc_nand_read_byte16;
		this->options |= NAND_BUSWIDTH_16;
		NFC_SET_NFMS(1 << NFMS_NF_DWIDTH);
	} else {
		NFC_SET_NFMS(0);
	}

	nfc_clk = clk_get(&pdev->dev, "nfc_clk");
	clk_enable(nfc_clk);

	init_waitqueue_head(&irq_waitq);
	err = request_irq(MXC_INT_NANDFC, mxc_nfc_irq, 0, "mxc_nd", NULL);
	if (err) {
		goto out_1;
	}

	if (hardware_ecc) {
		this->ecc.read_page = mxc_nand_read_page;
		this->ecc.write_page = mxc_nand_write_page;
		this->ecc.read_oob = mxc_nand_read_oob;
		this->ecc.layout = &nand_hw_eccoob_512;
		this->ecc.calculate = mxc_nand_calculate_ecc;
		this->ecc.hwctl = mxc_nand_enable_hwecc;
		this->ecc.correct = mxc_nand_correct_data;
		this->ecc.mode = NAND_ECC_HW;
		this->ecc.size = 512;
		this->ecc.bytes = 9;
		raw_write((raw_read(REG_NFC_ECC_EN) | NFC_ECC_EN),
			  REG_NFC_ECC_EN);
	} else {
		this->ecc.mode = NAND_ECC_SOFT;
		raw_write((raw_read(REG_NFC_ECC_EN) & ~NFC_ECC_EN),
			  REG_NFC_ECC_EN);
	}

	/* config the gpio */
	if (flash->init)
		flash->init();

	/* Reset NAND */
	this->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);

	/* Scan to find existence of the device */
	if (nand_scan(mtd, NFC_GET_MAXCHIP_SP())) {
		DEBUG(MTD_DEBUG_LEVEL0,
		      "MXC_ND2: Unable to find any NAND device.\n");
		err = -ENXIO;
		goto out_1;
	}

	/* Register the partitions */
#ifdef CONFIG_MTD_PARTITIONS
	nr_parts =
	    parse_mtd_partitions(mtd, part_probes, &mxc_nand_data->parts, 0);
	if (nr_parts > 0)
		add_mtd_partitions(mtd, mxc_nand_data->parts, nr_parts);
	else if (flash->parts)
		add_mtd_partitions(mtd, flash->parts, flash->nr_parts);
	else
#endif
	{
		pr_info("Registering %s as whole device\n", mtd->name);
		add_mtd_device(mtd);
	}

	platform_set_drvdata(pdev, mtd);

	return 0;

      out_1:
	kfree(mxc_nand_data);
      out:
	return err;

}

 /*!
  * Dissociates the driver from the device.
  *
  * @param   pdev  the device structure used to give information on which
  *
  * @return  The function always returns 0.
  */

static int __exit mxcnd_remove(struct platform_device *pdev)
{
	struct mtd_info *mtd = platform_get_drvdata(pdev);
	struct flash_platform_data *flash = pdev->dev.platform_data;

	if (flash->exit)
		flash->exit();

	mxc_free_buf();

	clk_disable(nfc_clk);
	clk_put(nfc_clk);
	platform_set_drvdata(pdev, NULL);

	if (mxc_nand_data) {
		nand_release(mtd);
		free_irq(MXC_INT_NANDFC, NULL);
		kfree(mxc_nand_data);
	}

	return 0;
}

#ifdef CONFIG_PM
/*!
 * This function is called to put the NAND in a low power state. Refer to the
 * document driver-model/driver.txt in the kernel source tree for more
 * information.
 *
 * @param   pdev  the device information structure
 *
 * @param   state the power state the device is entering
 *
 * @return  The function returns 0 on success and -1 on failure
 */

static int mxcnd_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct mtd_info *info = platform_get_drvdata(pdev);
	int ret = 0;

	DEBUG(MTD_DEBUG_LEVEL0, "MXC_ND2 : NAND suspend\n");
	if (info)
		ret = info->suspend(info);

	/* Disable the NFC clock */
	clk_disable(nfc_clk);

	/* Disable the NFC clock */
	clk_disable(nfc_clk);

	return ret;
}

/*!
 * This function is called to bring the NAND back from a low power state. Refer
 * to the document driver-model/driver.txt in the kernel source tree for more
 * information.
 *
 * @param   pdev  the device information structure
 *
 * @return  The function returns 0 on success and -1 on failure
 */
static int mxcnd_resume(struct platform_device *pdev)
{
	struct mtd_info *info = platform_get_drvdata(pdev);
	int ret = 0;

	DEBUG(MTD_DEBUG_LEVEL0, "MXC_ND2 : NAND resume\n");
	/* Enable the NFC clock */
	clk_enable(nfc_clk);

	if (info) {
		info->resume(info);
	}

	return ret;
}

#else
#define mxcnd_suspend   NULL
#define mxcnd_resume    NULL
#endif				/* CONFIG_PM */

/*!
 * This structure contains pointers to the power management callback functions.
 */
static struct platform_driver mxcnd_driver = {
	.driver = {
		   .name = "mxc_nandv2_flash",
		   },
	.probe = mxcnd_probe,
	.remove = __exit_p(mxcnd_remove),
	.suspend = mxcnd_suspend,
	.resume = mxcnd_resume,
};

/*!
 * Main initialization routine
 * @return  0 if successful; non-zero otherwise
 */
static int __init mxc_nd_init(void)
{
	/* Register the device driver structure. */
	pr_info("MXC MTD nand Driver %s\n", DVR_VER);
	if (platform_driver_register(&mxcnd_driver) != 0) {
		printk(KERN_ERR "Driver register failed for mxcnd_driver\n");
		return -ENODEV;
	}
	return 0;
}

/*!
 * Clean up routine
 */
static void __exit mxc_nd_cleanup(void)
{
	/* Unregister the device structure */
	platform_driver_unregister(&mxcnd_driver);
}

module_init(mxc_nd_init);
module_exit(mxc_nd_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC NAND MTD driver Version 2-5");
MODULE_LICENSE("GPL");
