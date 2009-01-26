/*
 * Copyright 2004-2008 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <asm/io.h>
#include <asm/mach/flash.h>

#include "mxc_nd.h"

#define DVR_VER "2.1"

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
/*
 * Macros to get half word and bit positions of ECC
 */
#define COLPOS(x) ((x) >> 4)
#define BITPOS(x) ((x) & 0xf)

/* Define single bit Error positions in Main & Spare area */
#define MAIN_SINGLEBIT_ERROR 0x4
#define SPARE_SINGLEBIT_ERROR 0x1

struct nand_info {
	bool bSpareOnly;
	bool bStatusRequest;
	u16 colAddr;
};

static struct nand_info g_nandfc_info;

#ifdef CONFIG_MTD_NAND_MXC_SWECC
static int hardware_ecc = 0;
#else
static int hardware_ecc = 1;
#endif

#ifndef CONFIG_MTD_NAND_MXC_ECC_CORRECTION_OPTION2
static int Ecc_disabled;
#endif

static int is2k_Pagesize = 0;

static struct clk *nfc_clk;

/*
 * OOB placement block for use with hardware ecc generation
 */
static struct nand_ecclayout nand_hw_eccoob_8 = {
	.eccbytes = 5,
	.eccpos = {6, 7, 8, 9, 10},
	.oobfree = {{0, 5}, {11, 5}}
};

static struct nand_ecclayout nand_hw_eccoob_16 = {
	.eccbytes = 5,
	.eccpos = {6, 7, 8, 9, 10},
	.oobfree = {{0, 6}, {12, 4}}
};

static struct nand_ecclayout nand_hw_eccoob_2k = {
	.eccbytes = 20,
	.eccpos = {6, 7, 8, 9, 10, 22, 23, 24, 25, 26,
		   38, 39, 40, 41, 42, 54, 55, 56, 57, 58},
	.oobfree = {
		    {.offset = 0,
		     .length = 5},

		    {.offset = 11,
		     .length = 10},

		    {.offset = 27,
		     .length = 10},

		    {.offset = 43,
		     .length = 10},

		    {.offset = 59,
		     .length = 5}
		    }
};

/*!
 * @defgroup NAND_MTD NAND Flash MTD Driver for MXC processors
 */

/*!
 * @file mxc_nd.c
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
	NFC_CONFIG1 |= NFC_INT_MSK;	/* Disable interrupt */
	wake_up(&irq_waitq);

	return IRQ_RETVAL(1);
}

/*!
 * This function polls the NANDFC to wait for the basic operation to complete by
 * checking the INT bit of config2 register.
 *
 * @param       maxRetries     number of retry attempts (separated by 1 us)
 * @param       param           parameter for debug
 * @param       useirq         True if IRQ should be used rather than polling
 */
static void wait_op_done(int maxRetries, u16 param, bool useirq)
{
	if (useirq) {
		if ((NFC_CONFIG2 & NFC_INT) == 0) {
			NFC_CONFIG1 &= ~NFC_INT_MSK;	/* Enable interrupt */
			wait_event(irq_waitq, NFC_CONFIG2 & NFC_INT);
		}
		NFC_CONFIG2 &= ~NFC_INT;
	} else {
		while (maxRetries-- > 0) {
			if (NFC_CONFIG2 & NFC_INT) {
				NFC_CONFIG2 &= ~NFC_INT;
				break;
			}
			udelay(1);
		}
		if (maxRetries <= 0)
			DEBUG(MTD_DEBUG_LEVEL0, "%s(%d): INT not set\n",
			      __FUNCTION__, param);
	}
}

/*!
 * This function issues the specified command to the NAND device and
 * waits for completion.
 *
 * @param       cmd     command for NAND Flash
 * @param       useirq  True if IRQ should be used rather than polling
 */
static void send_cmd(u16 cmd, bool useirq)
{
	DEBUG(MTD_DEBUG_LEVEL3, "send_cmd(0x%x, %d)\n", cmd, useirq);

	NFC_FLASH_CMD = (u16) cmd;
	NFC_CONFIG2 = NFC_CMD;

	/* Wait for operation to complete */
	wait_op_done(TROP_US_DELAY, cmd, useirq);
}

/*!
 * This function sends an address (or partial address) to the
 * NAND device.  The address is used to select the source/destination for
 * a NAND command.
 *
 * @param       addr    address to be written to NFC.
 * @param       islast  True if this is the last address cycle for command
 */
static void send_addr(u16 addr, bool islast)
{
	DEBUG(MTD_DEBUG_LEVEL3, "send_addr(0x%x %d)\n", addr, islast);

	NFC_FLASH_ADDR = addr;
	NFC_CONFIG2 = NFC_ADDR;

	/* Wait for operation to complete */
	wait_op_done(TROP_US_DELAY, addr, islast);
}

/*!
 * This function requests the NANDFC to initate the transfer
 * of data currently in the NANDFC RAM buffer to the NAND device.
 *
 * @param	buf_id	      Specify Internal RAM Buffer number (0-3)
 * @param       bSpareOnly    set true if only the spare area is transferred
 */
static void send_prog_page(u8 buf_id, bool bSpareOnly)
{
	DEBUG(MTD_DEBUG_LEVEL3, "send_prog_page (%d)\n", bSpareOnly);

	/* NANDFC buffer 0 is used for page read/write */

	NFC_BUF_ADDR = buf_id;

	/* Configure spare or page+spare access */
	if (!is2k_Pagesize) {
		if (bSpareOnly) {
			NFC_CONFIG1 |= NFC_SP_EN;
		} else {
			NFC_CONFIG1 &= ~(NFC_SP_EN);
		}
	}
	NFC_CONFIG2 = NFC_INPUT;

	/* Wait for operation to complete */
	wait_op_done(TROP_US_DELAY, bSpareOnly, true);
}

/*!
 * This function will correct the single bit ECC error
 *
 * @param  buf_id	Specify Internal RAM Buffer number (0-3)
 * @param  eccpos 	Ecc byte and bit position
 * @param  bSpareOnly  	set to true if only spare area needs correction
 */

static void mxc_nd_correct_error(u8 buf_id, u16 eccpos, bool bSpareOnly)
{
	u16 col;
	u8 pos;
	volatile u16 *buf;

	/* Get col & bit position of error
	   these macros works for both 8 & 16 bits */
	col = COLPOS(eccpos);	/* Get half-word position */
	pos = BITPOS(eccpos);	/* Get bit position */

	DEBUG(MTD_DEBUG_LEVEL3,
	      "mxc_nd_correct_error (col=%d pos=%d)\n", col, pos);

	/* Set the pointer for main / spare area */
	if (!bSpareOnly) {
		buf = (volatile u16 *)(MAIN_AREA0 + col + (256 * buf_id));
	} else {
		buf = (volatile u16 *)(SPARE_AREA0 + col + (8 * buf_id));
	}

	/* Fix the data */
	*buf ^= (1 << pos);
}

/*!
 * This function will maintains state of single bit Error
 * in Main & spare  area
 *
 * @param buf_id	Specify Internal RAM Buffer number (0-3)
 * @param spare  	set to true if only spare area needs correction
 */
static void mxc_nd_correct_ecc(u8 buf_id, bool spare)
{
#ifdef CONFIG_MTD_NAND_MXC_ECC_CORRECTION_OPTION2
	static int lastErrMain = 0, lastErrSpare = 0;	/* To maintain single bit
							   error in previous page */
#endif
	u16 value, ecc_status;
	/* Read the ECC result */
	ecc_status = NFC_ECC_STATUS_RESULT;
	DEBUG(MTD_DEBUG_LEVEL3,
	      "mxc_nd_correct_ecc (Ecc status=%x)\n", ecc_status);

#ifdef CONFIG_MTD_NAND_MXC_ECC_CORRECTION_OPTION2
	/* Check for Error in Mainarea */
	if ((ecc_status & 0xC) == MAIN_SINGLEBIT_ERROR) {
		/* Check for error in previous page */
		if (lastErrMain && !spare) {
			value = NFC_RSLTMAIN_AREA;
			/* Correct single bit error in Mainarea
			   NFC will not correct the error in
			   current page */
			mxc_nd_correct_error(buf_id, value, false);
		} else {
			/* Set if single bit error in current page */
			lastErrMain = 1;
		}
	} else {
		/* Reset if no single bit error in current page */
		lastErrMain = 0;
	}

	/* Check for Error in Sparearea */
	if ((ecc_status & 0x3) == SPARE_SINGLEBIT_ERROR) {
		/* Check for error in previous page */
		if (lastErrSpare) {
			value = NFC_RSLTSPARE_AREA;
			/* Correct single bit error in Mainarea
			   NFC will not correct the error in
			   current page */
			mxc_nd_correct_error(buf_id, value, true);
		} else {
			/* Set if single bit error in current page */
			lastErrSpare = 1;
		}
	} else {
		/* Reset if no single bit error in current page */
		lastErrSpare = 0;
	}
#else
	if (((ecc_status & 0xC) == MAIN_SINGLEBIT_ERROR)
	    || ((ecc_status & 0x3) == SPARE_SINGLEBIT_ERROR)) {
		if (Ecc_disabled) {
			if ((ecc_status & 0xC) == MAIN_SINGLEBIT_ERROR) {
				value = NFC_RSLTMAIN_AREA;
				/* Correct single bit error in Mainarea
				   NFC will not correct the error in
				   current page */
				mxc_nd_correct_error(buf_id, value, false);
			}
			if ((ecc_status & 0x3) == SPARE_SINGLEBIT_ERROR) {
				value = NFC_RSLTSPARE_AREA;
				/* Correct single bit error in Mainarea
				   NFC will not correct the error in
				   current page */
				mxc_nd_correct_error(buf_id, value, true);
			}

		} else {
			/* Disable ECC  */
			NFC_CONFIG1 &= ~(NFC_ECC_EN);
			Ecc_disabled = 1;
		}
	} else if (ecc_status == 0) {
		if (Ecc_disabled) {
			/* Enable ECC */
			NFC_CONFIG1 |= NFC_ECC_EN;
			Ecc_disabled = 0;
		}
	} else {
		/* 2-bit Error  Do nothing */
	}
#endif				/* CONFIG_MTD_NAND_MXC_ECC_CORRECTION_OPTION2 */

}

/*!
 * This function requests the NANDFC to initated the transfer
 * of data from the NAND device into in the NANDFC ram buffer.
 *
 * @param  	buf_id		Specify Internal RAM Buffer number (0-3)
 * @param       bSpareOnly    	set true if only the spare area is transferred
 */
static void send_read_page(u8 buf_id, bool bSpareOnly)
{
	DEBUG(MTD_DEBUG_LEVEL3, "send_read_page (%d)\n", bSpareOnly);

	/* NANDFC buffer 0 is used for page read/write */
	NFC_BUF_ADDR = buf_id;

	/* Configure spare or page+spare access */
	if (!is2k_Pagesize) {
		if (bSpareOnly) {
			NFC_CONFIG1 |= NFC_SP_EN;
		} else {
			NFC_CONFIG1 &= ~(NFC_SP_EN);
		}
	}

	NFC_CONFIG2 = NFC_OUTPUT;

	/* Wait for operation to complete */
	wait_op_done(TROP_US_DELAY, bSpareOnly, true);

	/* If there are single bit errors in
	   two consecutive page reads then
	   the error is not  corrected by the
	   NFC for the second page.
	   Correct single bit error in driver */

	mxc_nd_correct_ecc(buf_id, bSpareOnly);
}

/*!
 * This function requests the NANDFC to perform a read of the
 * NAND device ID.
 */
static void send_read_id(void)
{
	struct nand_chip *this = &mxc_nand_data->nand;

	/* NANDFC buffer 0 is used for device ID output */
	NFC_BUF_ADDR = 0x0;

	/* Read ID into main buffer */
	NFC_CONFIG1 &= (~(NFC_SP_EN));
	NFC_CONFIG2 = NFC_ID;

	/* Wait for operation to complete */
	wait_op_done(TROP_US_DELAY, 0, true);

	if (this->options & NAND_BUSWIDTH_16) {
		volatile u16 *mainBuf = MAIN_AREA0;

		/*
		 * Pack the every-other-byte result for 16-bit ID reads
		 * into every-byte as the generic code expects and various
		 * chips implement.
		 */

		mainBuf[0] = (mainBuf[0] & 0xff) | ((mainBuf[1] & 0xff) << 8);
		mainBuf[1] = (mainBuf[2] & 0xff) | ((mainBuf[3] & 0xff) << 8);
		mainBuf[2] = (mainBuf[4] & 0xff) | ((mainBuf[5] & 0xff) << 8);
	}
}

/*!
 * This function requests the NANDFC to perform a read of the
 * NAND device status and returns the current status.
 *
 * @return  device status
 */
static u16 get_dev_status(void)
{
	volatile u16 *mainBuf = MAIN_AREA1;
	u32 store;
	u16 ret;
	/* Issue status request to NAND device */

	/* store the main area1 first word, later do recovery */
	store = *((u32 *) mainBuf);
	/*
	 * NANDFC buffer 1 is used for device status to prevent
	 * corruption of read/write buffer on status requests.
	 */
	NFC_BUF_ADDR = 1;

	/* Read status into main buffer */
	NFC_CONFIG1 &= (~(NFC_SP_EN));
	NFC_CONFIG2 = NFC_STATUS;

	/* Wait for operation to complete */
	wait_op_done(TROP_US_DELAY, 0, true);

	/* Status is placed in first word of main buffer */
	/* get status, then recovery area 1 data */
	ret = mainBuf[0];
	*((u32 *) mainBuf) = store;

	return ret;
}

/*!
 * This functions is used by upper layer to checks if device is ready
 *
 * @param       mtd     MTD structure for the NAND Flash
 *
 * @return  0 if device is busy else 1
 */
static int mxc_nand_dev_ready(struct mtd_info *mtd)
{
	/*
	 * NFC handles R/B internally.Therefore,this function
	 * always returns status as ready.
	 */
	return 1;
}

static void mxc_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
	/*
	 * If HW ECC is enabled, we turn it on during init.  There is
	 * no need to enable again here.
	 */
}

static int mxc_nand_correct_data(struct mtd_info *mtd, u_char * dat,
				 u_char * read_ecc, u_char * calc_ecc)
{
	/*
	 * 1-Bit errors are automatically corrected in HW.  No need for
	 * additional correction.  2-Bit errors cannot be corrected by
	 * HW ECC, so we need to return failure
	 */
	u16 ecc_status = NFC_ECC_STATUS_RESULT;

	if (((ecc_status & 0x3) == 2) || ((ecc_status >> 2) == 2)) {
		DEBUG(MTD_DEBUG_LEVEL0,
		      "MXC_NAND: HWECC uncorrectable 2-bit ECC error\n");
		return -1;
	}

	return 0;
}

static int mxc_nand_calculate_ecc(struct mtd_info *mtd, const u_char * dat,
				  u_char * ecc_code)
{
	/*
	 * Just return success.  HW ECC does not read/write the NFC spare
	 * buffer.  Only the FLASH spare area contains the calcuated ECC.
	 */
	return 0;
}

/*!
 * This function reads byte from the NAND Flash
 *
 * @param       mtd     MTD structure for the NAND Flash
 *
 * @return    data read from the NAND Flash
 */
static u_char mxc_nand_read_byte(struct mtd_info *mtd)
{
	u_char retVal = 0;
	u16 col, rdWord;
	volatile u16 *mainBuf = MAIN_AREA0;
	volatile u16 *spareBuf = SPARE_AREA0;

	/* Check for status request */
	if (g_nandfc_info.bStatusRequest) {
		return (get_dev_status() & 0xFF);
	}

	/* Get column for 16-bit access */
	col = g_nandfc_info.colAddr >> 1;

	/* If we are accessing the spare region */
	if (g_nandfc_info.bSpareOnly) {
		rdWord = spareBuf[col];
	} else {
		rdWord = mainBuf[col];
	}

	/* Pick upper/lower byte of word from RAM buffer */
	if (g_nandfc_info.colAddr & 0x1) {
		retVal = (rdWord >> 8) & 0xFF;
	} else {
		retVal = rdWord & 0xFF;
	}

	/* Update saved column address */
	g_nandfc_info.colAddr++;

	return retVal;
}

/*!
  * This function reads word from the NAND Flash
  *
  * @param       mtd     MTD structure for the NAND Flash
  *
  * @return    data read from the NAND Flash
  */
static u16 mxc_nand_read_word(struct mtd_info *mtd)
{
	u16 col;
	u16 rdWord, retVal;
	volatile u16 *p;

	DEBUG(MTD_DEBUG_LEVEL3,
	      "mxc_nand_read_word(col = %d)\n", g_nandfc_info.colAddr);

	col = g_nandfc_info.colAddr;
	/* Adjust saved column address */
	if (col < mtd->writesize && g_nandfc_info.bSpareOnly)
		col += mtd->writesize;

	if (col < mtd->writesize)
		p = (MAIN_AREA0) + (col >> 1);
	else
		p = (SPARE_AREA0) + ((col - mtd->writesize) >> 1);

	if (col & 1) {
		rdWord = *p;
		retVal = (rdWord >> 8) & 0xff;
		rdWord = *(p + 1);
		retVal |= (rdWord << 8) & 0xff00;

	} else {
		retVal = *p;

	}

	/* Update saved column address */
	g_nandfc_info.colAddr = col + 2;

	return retVal;
}

/*!
 * This function writes data of length \b len to buffer \b buf. The data to be
 * written on NAND Flash is first copied to RAMbuffer. After the Data Input
 * Operation by the NFC, the data is written to NAND Flash
 *
 * @param       mtd     MTD structure for the NAND Flash
 * @param       buf     data to be written to NAND Flash
 * @param       len     number of bytes to be written
 */
static void mxc_nand_write_buf(struct mtd_info *mtd,
			       const u_char * buf, int len)
{
	int n;
	int col;
	int i = 0;

	DEBUG(MTD_DEBUG_LEVEL3,
	      "mxc_nand_write_buf(col = %d, len = %d)\n", g_nandfc_info.colAddr,
	      len);

	col = g_nandfc_info.colAddr;

	/* Adjust saved column address */
	if (col < mtd->writesize && g_nandfc_info.bSpareOnly)
		col += mtd->writesize;

	n = mtd->writesize + mtd->oobsize - col;
	n = min(len, n);

	DEBUG(MTD_DEBUG_LEVEL3,
	      "%s:%d: col = %d, n = %d\n", __FUNCTION__, __LINE__, col, n);

	while (n) {
		volatile u32 *p;
		if (col < mtd->writesize)
			p = (volatile u32 *)((ulong) (MAIN_AREA0) + (col & ~3));
		else
			p = (volatile u32 *)((ulong) (SPARE_AREA0) -
					     mtd->writesize + (col & ~3));

		DEBUG(MTD_DEBUG_LEVEL3, "%s:%d: p = %p\n", __FUNCTION__,
		      __LINE__, p);

		if (((col | (int)&buf[i]) & 3) || n < 16) {
			u32 data = 0;

			if (col & 3 || n < 4)
				data = *p;

			switch (col & 3) {
			case 0:
				if (n) {
					data = (data & 0xffffff00) |
					    (buf[i++] << 0);
					n--;
					col++;
				}
			case 1:
				if (n) {
					data = (data & 0xffff00ff) |
					    (buf[i++] << 8);
					n--;
					col++;
				}
			case 2:
				if (n) {
					data = (data & 0xff00ffff) |
					    (buf[i++] << 16);
					n--;
					col++;
				}
			case 3:
				if (n) {
					data = (data & 0x00ffffff) |
					    (buf[i++] << 24);
					n--;
					col++;
				}
			}

			*p = data;
		} else {
			int m = mtd->writesize - col;

			if (col >= mtd->writesize)
				m += mtd->oobsize;

			m = min(n, m) & ~3;

			DEBUG(MTD_DEBUG_LEVEL3,
			      "%s:%d: n = %d, m = %d, i = %d, col = %d\n",
			      __FUNCTION__, __LINE__, n, m, i, col);

			memcpy((void *)(p), &buf[i], m);
			col += m;
			i += m;
			n -= m;
		}
	}
	/* Update saved column address */
	g_nandfc_info.colAddr = col;

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

	int n;
	int col;
	int i = 0;

	DEBUG(MTD_DEBUG_LEVEL3,
	      "mxc_nand_read_buf(col = %d, len = %d)\n", g_nandfc_info.colAddr,
	      len);

	col = g_nandfc_info.colAddr;
	/* Adjust saved column address */
	if (col < mtd->writesize && g_nandfc_info.bSpareOnly)
		col += mtd->writesize;

	n = mtd->writesize + mtd->oobsize - col;
	n = min(len, n);

	while (n) {
		volatile u32 *p;

		if (col < mtd->writesize)
			p = (volatile u32 *)((ulong) (MAIN_AREA0) + (col & ~3));
		else
			p = (volatile u32 *)((ulong) (SPARE_AREA0) -
					     mtd->writesize + (col & ~3));

		if (((col | (int)&buf[i]) & 3) || n < 16) {
			u32 data;

			data = *p;
			switch (col & 3) {
			case 0:
				if (n) {
					buf[i++] = (u8) (data);
					n--;
					col++;
				}
			case 1:
				if (n) {
					buf[i++] = (u8) (data >> 8);
					n--;
					col++;
				}
			case 2:
				if (n) {
					buf[i++] = (u8) (data >> 16);
					n--;
					col++;
				}
			case 3:
				if (n) {
					buf[i++] = (u8) (data >> 24);
					n--;
					col++;
				}
			}
		} else {
			int m = mtd->writesize - col;

			if (col >= mtd->writesize)
				m += mtd->oobsize;

			m = min(n, m) & ~3;
			memcpy(&buf[i], (void *)(p), m);
			col += m;
			i += m;
			n -= m;
		}
	}
	/* Update saved column address */
	g_nandfc_info.colAddr = col;

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
static int
mxc_nand_verify_buf(struct mtd_info *mtd, const u_char * buf, int len)
{
	return -EFAULT;
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
#ifdef CONFIG_MTD_NAND_MXC_FORCE_CE
	if (chip > 0) {
		DEBUG(MTD_DEBUG_LEVEL0,
		      "ERROR:  Illegal chip select (chip = %d)\n", chip);
		return;
	}

	if (chip == -1) {
		NFC_CONFIG1 &= (~(NFC_CE));
		return;
	}

	NFC_CONFIG1 |= NFC_CE;
#endif

	switch (chip) {
	case -1:
		/* Disable the NFC clock */
		clk_disable(nfc_clk);
		break;
	case 0:
		/* Enable the NFC clock */
		clk_enable(nfc_clk);
		break;

	default:
		break;
	}
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
	bool useirq = true;

	DEBUG(MTD_DEBUG_LEVEL3,
	      "mxc_nand_command (cmd = 0x%x, col = 0x%x, page = 0x%x)\n",
	      command, column, page_addr);

	/*
	 * Reset command state information
	 */
	g_nandfc_info.bStatusRequest = false;

	/* Reset column address to 0 */
	g_nandfc_info.colAddr = 0;

	/*
	 * Command pre-processing step
	 */
	switch (command) {

	case NAND_CMD_STATUS:
		g_nandfc_info.bStatusRequest = true;
		break;

	case NAND_CMD_READ0:
		g_nandfc_info.colAddr = column;
		g_nandfc_info.bSpareOnly = false;
		useirq = false;
		break;

	case NAND_CMD_READOOB:
		g_nandfc_info.colAddr = column;
		g_nandfc_info.bSpareOnly = true;
		useirq = false;
		if (is2k_Pagesize)
			command = NAND_CMD_READ0;	/* only READ0 is valid */
		break;

	case NAND_CMD_SEQIN:
		if (column >= mtd->writesize) {
			if (is2k_Pagesize) {
				/**
				  * FIXME: before send SEQIN command for write OOB,
				  * We must read one page out.
				  * For K9F1GXX has no READ1 command to set current HW
				  * pointer to spare area, we must write the whole page including OOB together.
				  */
				/* call itself to read a page */
				mxc_nand_command(mtd, NAND_CMD_READ0, 0,
						 page_addr);
			}
			g_nandfc_info.colAddr = column - mtd->writesize;
			g_nandfc_info.bSpareOnly = true;
			/* Set program pointer to spare region */
			if (!is2k_Pagesize)
				send_cmd(NAND_CMD_READOOB, false);
		} else {
			g_nandfc_info.bSpareOnly = false;
			g_nandfc_info.colAddr = column;
			/* Set program pointer to page start */
			if (!is2k_Pagesize)
				send_cmd(NAND_CMD_READ0, false);
		}
		useirq = false;
		break;

	case NAND_CMD_PAGEPROG:
#ifndef CONFIG_MTD_NAND_MXC_ECC_CORRECTION_OPTION2
		if (Ecc_disabled) {
			/* Enable Ecc for page writes */
			NFC_CONFIG1 |= NFC_ECC_EN;
		}
#endif

		send_prog_page(0, g_nandfc_info.bSpareOnly);

		if (is2k_Pagesize) {
			/* data in 4 areas datas */
			send_prog_page(1, g_nandfc_info.bSpareOnly);
			send_prog_page(2, g_nandfc_info.bSpareOnly);
			send_prog_page(3, g_nandfc_info.bSpareOnly);
		}

		break;

	case NAND_CMD_ERASE1:
		useirq = false;
		break;
	}

	/*
	 * Write out the command to the device.
	 */
	send_cmd(command, useirq);

	/*
	 * Write out column address, if necessary
	 */
	if (column != -1) {
		/*
		 * MXC NANDFC can only perform full page+spare or
		 * spare-only read/write.  When the upper layers
		 * layers perform a read/write buf operation,
		 * we will used the saved column adress to index into
		 * the full page.
		 */
		send_addr(0, page_addr == -1);
		if (is2k_Pagesize)
			/* another col addr cycle for 2k page */
			send_addr(0, false);
	}

	/*
	 * Write out page address, if necessary
	 */
	if (page_addr != -1) {
		send_addr((page_addr & 0xff), false);	/* paddr_0 - p_addr_7 */

		if (is2k_Pagesize) {
			/* One more address cycle for higher density devices */
			if (mtd->size >= 0x10000000) {
				/* paddr_8 - paddr_15 */
				send_addr((page_addr >> 8) & 0xff, false);
				send_addr((page_addr >> 16) & 0xff, true);
			} else
				/* paddr_8 - paddr_15 */
				send_addr((page_addr >> 8) & 0xff, true);
		} else {
			/* One more address cycle for higher density devices */
			if (mtd->size >= 0x4000000) {
				/* paddr_8 - paddr_15 */
				send_addr((page_addr >> 8) & 0xff, false);
				send_addr((page_addr >> 16) & 0xff, true);
			} else
				/* paddr_8 - paddr_15 */
				send_addr((page_addr >> 8) & 0xff, true);
		}
	}

	/*
	 * Command post-processing step
	 */
	switch (command) {

	case NAND_CMD_RESET:
		break;

	case NAND_CMD_READOOB:
	case NAND_CMD_READ0:
		if (is2k_Pagesize) {
			/* send read confirm command */
			send_cmd(NAND_CMD_READSTART, true);
			/* read for each AREA */
			send_read_page(0, g_nandfc_info.bSpareOnly);
			send_read_page(1, g_nandfc_info.bSpareOnly);
			send_read_page(2, g_nandfc_info.bSpareOnly);
			send_read_page(3, g_nandfc_info.bSpareOnly);
		} else {
			send_read_page(0, g_nandfc_info.bSpareOnly);
		}
		break;

	case NAND_CMD_READID:
		send_read_id();
		break;

	case NAND_CMD_PAGEPROG:
#ifndef CONFIG_MTD_NAND_MXC_ECC_CORRECTION_OPTION2
		if (Ecc_disabled) {
			/* Disble Ecc after page writes */
			NFC_CONFIG1 &= ~(NFC_ECC_EN);
		}
#endif
		break;

	case NAND_CMD_STATUS:
		break;

	case NAND_CMD_ERASE2:
		break;
	}
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

	/* Config before scanning */
	/* Do not rely on NFMS_BIT, set/clear NFMS bit based on mtd->writesize */
	if (mtd->writesize == 2048) {
		NFMS |= (1 << NFMS_BIT);
		is2k_Pagesize = 1;
	} else {
		if ((NFMS >> NFMS_BIT) & 0x1) {	/* This case strangly happened on MXC91321 P1.2.2 */
			printk(KERN_INFO
			       "Oops... NFMS Bit set for 512B Page, resetting it. [RCSR: 0x%08x]\n",
			       NFMS);
			NFMS &= ~(1 << NFMS_BIT);
		}
		is2k_Pagesize = 0;
	}

	if (is2k_Pagesize)
		this->ecc.layout = &nand_hw_eccoob_2k;

	/* jffs2 not write oob */
	mtd->flags &= ~MTD_OOB_WRITEABLE;

	/* use flash based bbt */
	this->bbt_td = &bbt_main_descr;
	this->bbt_md = &bbt_mirror_descr;

	/* update flash based bbt */
	this->options |= NAND_USE_FLASH_BBT;

	if (!this->badblock_pattern) {
		if (mtd->writesize == 2048)
			this->badblock_pattern = &smallpage_memorybased;
		else
			this->badblock_pattern = (mtd->writesize > 512) ?
			    &largepage_memorybased : &smallpage_memorybased;
	}
	/* Build bad block table */
	return nand_scan_bbt(mtd, this->badblock_pattern);
}

#ifdef CONFIG_MXC_NAND_LOW_LEVEL_ERASE
static void mxc_low_erase(struct mtd_info *mtd)
{

	struct nand_chip *this = mtd->priv;
	unsigned int page_addr, addr;
	u_char status;

	DEBUG(MTD_DEBUG_LEVEL0, "MXC_ND : mxc_low_erase:Erasing NAND\n");
	for (addr = 0; addr < this->chipsize; addr += mtd->erasesize) {
		page_addr = addr / mtd->writesize;
		mxc_nand_command(mtd, NAND_CMD_ERASE1, -1, page_addr);
		mxc_nand_command(mtd, NAND_CMD_ERASE2, -1, -1);
		mxc_nand_command(mtd, NAND_CMD_STATUS, -1, -1);
		status = mxc_nand_read_byte(mtd);
		if (status & NAND_STATUS_FAIL) {
			printk(KERN_ERR
			       "ERASE FAILED(block = %d,status = 0x%x)\n",
			       addr / mtd->erasesize, status);
		}
	}

}
#endif
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
	int nr_parts = 0;

	int err = 0;
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

	/* 50 us command delay time */
	this->chip_delay = 5;

	this->priv = mxc_nand_data;
	this->dev_ready = mxc_nand_dev_ready;
	this->cmdfunc = mxc_nand_command;
	this->select_chip = mxc_nand_select_chip;
	this->read_byte = mxc_nand_read_byte;
	this->read_word = mxc_nand_read_word;
	this->write_buf = mxc_nand_write_buf;
	this->read_buf = mxc_nand_read_buf;
	this->verify_buf = mxc_nand_verify_buf;
	this->scan_bbt = mxc_nand_scan_bbt;

	nfc_clk = clk_get(&pdev->dev, "nfc_clk");
	clk_enable(nfc_clk);

	NFC_CONFIG1 |= NFC_INT_MSK;
	init_waitqueue_head(&irq_waitq);
	err = request_irq(MXC_INT_NANDFC, mxc_nfc_irq, 0, "mxc_nd", NULL);
	if (err) {
		goto out_1;
	}

	if (hardware_ecc) {
		this->ecc.calculate = mxc_nand_calculate_ecc;
		this->ecc.hwctl = mxc_nand_enable_hwecc;
		this->ecc.correct = mxc_nand_correct_data;
		this->ecc.mode = NAND_ECC_HW;
		this->ecc.size = 512;
		this->ecc.bytes = 3;
		this->ecc.layout = &nand_hw_eccoob_8;
		NFC_CONFIG1 |= NFC_ECC_EN;
	} else {
		this->ecc.mode = NAND_ECC_SOFT;
	}

	/* Reset NAND */
	this->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);

	/* preset operation */
	/* Unlock the internal RAM Buffer */
	NFC_CONFIG = 0x2;

	/* Blocks to be unlocked */
	NFC_UNLOCKSTART_BLKADDR = 0x0;
	NFC_UNLOCKEND_BLKADDR = 0x4000;

	/* Unlock Block Command for given address range */
	NFC_WRPROT = 0x4;

	/* NAND bus width determines access funtions used by upper layer */
	if (flash->width == 2) {
		this->options |= NAND_BUSWIDTH_16;
		this->ecc.layout = &nand_hw_eccoob_16;
	} else {
		this->options |= 0;
	}

	is2k_Pagesize = 0;

	/* Scan to find existence of the device */
	if (nand_scan(mtd, 1)) {
		DEBUG(MTD_DEBUG_LEVEL0,
		      "MXC_ND: Unable to find any NAND device.\n");
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
#ifdef CONFIG_MXC_NAND_LOW_LEVEL_ERASE
	/* Erase all the blocks of a NAND */
	mxc_low_erase(mtd);
#endif

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

	DEBUG(MTD_DEBUG_LEVEL0, "MXC_ND : NAND suspend\n");
	if (info)
		ret = info->suspend(info);

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

	DEBUG(MTD_DEBUG_LEVEL0, "MXC_ND : NAND resume\n");
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
		   .name = "mxc_nand_flash",
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
MODULE_DESCRIPTION("MXC NAND MTD driver");
MODULE_LICENSE("GPL");
