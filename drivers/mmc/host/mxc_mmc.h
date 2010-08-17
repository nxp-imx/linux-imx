/*
 * Copyright 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __MXC_MMC_REG_H__
#define __MXC_MMC_REG_H__

/*!
 * @defgroup MMC_SD MMC/SD Driver
 */

/*!
 * @file mxc_mmc.h
 *
 * @brief Driver for the Freescale Semiconductor MXC SDHC modules.
 *
 * This file defines offsets and bits of SDHC registers. SDHC is also referred as
 * MMC/SD controller
 *
 * @ingroup MMC_SD
 */

/*!
 * Number of SDHC modules
 */

#define SDHC_MMC_WML                    16
#define SDHC_SD_WML                     64
#define DRIVER_NAME                     "MXCMMC"
#define SDHC_MEM_SIZE                   16384
#define SDHC_REV_NO                     0x400
#define READ_TO_VALUE                   0x2db4

/* Address offsets of the SDHC registers */
#define MMC_STR_STP_CLK                 0x00	/* Clock Control Reg */
#define MMC_STATUS                      0x04	/* Status Reg */
#define MMC_CLK_RATE                    0x08	/* Clock Rate Reg */
#define MMC_CMD_DAT_CONT                0x0C	/* Command and Data Control Reg */
#define MMC_RES_TO                      0x10	/* Response Time-out Reg */
#define MMC_READ_TO                     0x14	/* Read Time-out Reg */
#define MMC_BLK_LEN                     0x18	/* Block Length Reg */
#define MMC_NOB                         0x1C	/* Number of Blocks Reg */
#define MMC_REV_NO                      0x20	/* Revision Number Reg */
#define MMC_INT_CNTR                    0x24	/* Interrupt Control Reg */
#define MMC_CMD                         0x28	/* Command Number Reg */
#define MMC_ARG                         0x2C	/* Command Argument Reg */
#define MMC_RES_FIFO                    0x34	/* Command Response Reg */
#define MMC_BUFFER_ACCESS               0x38	/* Data Buffer Access Reg */
#define MMC_REM_NOB                     0x40	/* Remaining NOB Reg */
#define MMC_REM_BLK_SIZE                0x44	/* Remaining Block Size Reg */

/* Bit definitions for STR_STP_CLK */
#define STR_STP_CLK_RESET               (1<<3)
#define STR_STP_CLK_START_CLK           (1<<1)
#define STR_STP_CLK_STOP_CLK            (1<<0)

/* Bit definitions for STATUS */
#define STATUS_CARD_INSERTION           (1<<31)
#define STATUS_CARD_REMOVAL             (1<<30)
#define STATUS_YBUF_EMPTY               (1<<29)
#define STATUS_XBUF_EMPTY               (1<<28)
#define STATUS_YBUF_FULL                (1<<27)
#define STATUS_XBUF_FULL                (1<<26)
#define STATUS_BUF_UND_RUN              (1<<25)
#define STATUS_BUF_OVFL                 (1<<24)
#define STATUS_SDIO_INT_ACTIVE          (1<<14)
#define STATUS_END_CMD_RESP             (1<<13)
#define STATUS_WRITE_OP_DONE            (1<<12)
#define STATUS_READ_OP_DONE             (1<<11)
#define STATUS_WR_CRC_ERROR_CODE_MASK   (3<<9)
#define STATUS_CARD_BUS_CLK_RUN         (1<<8)
#define STATUS_BUF_READ_RDY             (1<<7)
#define STATUS_BUF_WRITE_RDY            (1<<6)
#define STATUS_RESP_CRC_ERR             (1<<5)
#define STATUS_READ_CRC_ERR             (1<<3)
#define STATUS_WRITE_CRC_ERR            (1<<2)
#define STATUS_TIME_OUT_RESP            (1<<1)
#define STATUS_TIME_OUT_READ            (1<<0)
#define STATUS_ERR_MASK                 0x3f

/* Clock rate definitions */
#define CLK_RATE_PRESCALER(x)           ((x) & 0xF)
#define CLK_RATE_CLK_DIVIDER(x)         (((x) & 0xF) << 4)

/* Bit definitions for CMD_DAT_CONT */
#define CMD_DAT_CONT_CMD_RESP_LONG_OFF  (1<<12)
#define CMD_DAT_CONT_STOP_READWAIT      (1<<11)
#define CMD_DAT_CONT_START_READWAIT     (1<<10)
#define CMD_DAT_CONT_BUS_WIDTH_1        (0<<8)
#define CMD_DAT_CONT_BUS_WIDTH_4        (2<<8)
#define CMD_DAT_CONT_INIT               (1<<7)
#define CMD_DAT_CONT_WRITE              (1<<4)
#define CMD_DAT_CONT_DATA_ENABLE        (1<<3)
#define CMD_DAT_CONT_RESPONSE_FORMAT_R1 (1)
#define CMD_DAT_CONT_RESPONSE_FORMAT_R2 (2)
#define CMD_DAT_CONT_RESPONSE_FORMAT_R3 (3)
#define CMD_DAT_CONT_RESPONSE_FORMAT_R4 (4)
#define CMD_DAT_CONT_RESPONSE_FORMAT_R5 (5)
#define CMD_DAT_CONT_RESPONSE_FORMAT_R6 (6)

/* Bit definitions for INT_CNTR */
#define INT_CNTR_SDIO_INT_WKP_EN        (1<<18)
#define INT_CNTR_CARD_INSERTION_WKP_EN  (1<<17)
#define INT_CNTR_CARD_REMOVAL_WKP_EN    (1<<16)
#define INT_CNTR_CARD_INSERTION_EN      (1<<15)
#define INT_CNTR_CARD_REMOVAL_EN        (1<<14)
#define INT_CNTR_SDIO_IRQ_EN            (1<<13)
#define INT_CNTR_DAT0_EN                (1<<12)
#define INT_CNTR_BUF_READ_EN            (1<<4)
#define INT_CNTR_BUF_WRITE_EN           (1<<3)
#define INT_CNTR_END_CMD_RES            (1<<2)
#define INT_CNTR_WRITE_OP_DONE          (1<<1)
#define INT_CNTR_READ_OP_DONE           (1<<0)

#endif				/* __MXC_MMC_REG_H__ */
