/*
 * Copyright 2004-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mxc_nd2.h
 *
 * @brief This file contains the NAND Flash Controller register information.
 *
 *
 * @ingroup NAND_MTD
 */

#ifndef __MXC_ND2_H__
#define __MXC_ND2_H__

#include <mach/hardware.h>
#include <mach/common.h>

#define IS_2K_PAGE_NAND         ((mtd->writesize / num_of_interleave) \
						== NAND_PAGESIZE_2KB)
#define IS_4K_PAGE_NAND         ((mtd->writesize / num_of_interleave) \
						== NAND_PAGESIZE_4KB)
#define IS_LARGE_PAGE_NAND      ((mtd->writesize / num_of_interleave) > 512)

#define GET_NAND_OOB_SIZE	(mtd->oobsize / num_of_interleave)
#define GET_NAND_PAGE_SIZE      (mtd->writesize / num_of_interleave)

#define NAND_PAGESIZE_2KB	2048
#define NAND_PAGESIZE_4KB	4096

/*
 * main area for bad block marker is in the last data section
 * the spare area for swapped bad block marker is the second
 * byte of last spare section
 */
#define NAND_SECTIONS        (GET_NAND_PAGE_SIZE >> 9)
#define NAND_OOB_PER_SECTION (((GET_NAND_OOB_SIZE / NAND_SECTIONS) >> 1) << 1)
#define NAND_CHUNKS	     (GET_NAND_PAGE_SIZE / (512 + NAND_OOB_PER_SECTION))

#define BAD_BLK_MARKER_MAIN_OFFS \
	(GET_NAND_PAGE_SIZE - NAND_CHUNKS * NAND_OOB_PER_SECTION)

#define BAD_BLK_MARKER_SP_OFFS (NAND_CHUNKS * SPARE_LEN)

#define BAD_BLK_MARKER_OOB_OFFS (NAND_CHUNKS * NAND_OOB_PER_SECTION)

#define BAD_BLK_MARKER_MAIN  \
	((u32)MAIN_AREA0 + BAD_BLK_MARKER_MAIN_OFFS)

#define BAD_BLK_MARKER_SP  \
	((u32)SPARE_AREA0 + BAD_BLK_MARKER_SP_OFFS)

#ifdef CONFIG_ARCH_MXC_HAS_NFC_V3
/*
 * For V3 NFC registers Definition
 */

#if defined(CONFIG_ARCH_MXC_HAS_NFC_V3_1)	/* mx37 */
#define MXC_INT_NANDFC			MXC_INT_EMI
#define NFC_FLASH_ADDR_CMD		(nfc_axi_base + 0x1E00)
#define NFC_CONFIG1			(nfc_axi_base + 0x1E04)
#define NFC_ECC_STATUS_RESULT		(nfc_axi_base + 0x1E08)
#define LAUNCH_NFC			(nfc_axi_base + 0x1E0c)
#define NFC_WRPROT			(nfc_ip_base + 0x00)
#define NFC_WRPROT_UNLOCK_BLK_ADD0	(nfc_ip_base + 0x04)
#define NFC_CONFIG2			(nfc_ip_base + 0x14)
#define NFC_IPC				(nfc_ip_base + 0x18)
#elif defined(CONFIG_ARCH_MXC_HAS_NFC_V3_2)	/* mx51 */
#define MXC_INT_NANDFC			MXC_INT_NFC
#define NFC_AUTO_MODE_ENABLE
#define NFC_FLASH_CMD			(nfc_axi_base + 0x1E00)
#define NFC_FLASH_ADDR0      		(nfc_axi_base + 0x1E04)
#define NFC_FLASH_ADDR8			(nfc_axi_base + 0x1E24)
#define NFC_CONFIG1         		(nfc_axi_base + 0x1E34)
#define NFC_ECC_STATUS_RESULT		(nfc_axi_base + 0x1E38)
#define NFC_ECC_STATUS_SUM		(nfc_axi_base + 0x1E3C)
#define LAUNCH_NFC			(nfc_axi_base + 0x1E40)
#define NFC_WRPROT			(nfc_ip_base + 0x00)
#define NFC_WRPROT_UNLOCK_BLK_ADD0	(nfc_ip_base + 0x04)
#define NFC_CONFIG2			(nfc_ip_base + 0x24)
#define NFC_CONFIG3			(nfc_ip_base + 0x28)
#define NFC_IPC				(nfc_ip_base + 0x2C)
#define NFC_DELAY_LINE			(nfc_ip_base + 0x34)
#else				/* skye */
#define NFC_FLASH_ADDR_CMD		(nfc_axi_base + 0xE00)
#define NFC_CONFIG1			(nfc_axi_base + 0xE04)
#define NFC_ECC_STATUS_RESULT		(nfc_axi_base + 0xE08)
#define LAUNCH_NFC			(nfc_axi_base + 0xE0C)
#define NFC_WRPROT			(nfc_ip_base + 0x00)
#define NFC_WRPROT_UNLOCK_BLK_ADD0	(nfc_ip_base + 0x04)
#define NFC_CONFIG2			(nfc_ip_base + 0x14)
#define NFC_IPC				(nfc_ip_base + 0x18)
#endif
/*!
 * Addresses for NFC RAM BUFFER Main area 0
 */
#define MAIN_AREA0        		((u16 *)(nfc_axi_base + 0x000))
#define MAIN_AREA1        		((u16 *)(nfc_axi_base + 0x200))

/*!
 * Addresses for NFC SPARE BUFFER Spare area 0
 */
#if defined(CONFIG_ARCH_MXC_HAS_NFC_V3_1) ||	\
    defined(CONFIG_ARCH_MXC_HAS_NFC_V3_2)
#define SPARE_AREA0       		((u16 *)(nfc_axi_base + 0x1000))
#define SPARE_LEN			64
#define SPARE_COUNT			8
#define SPARE_SIZE			(SPARE_LEN * SPARE_COUNT)
#else
#define SPARE_AREA0       		((u16 *)(nfc_axi_base + 0x800))
#define SPARE_LEN			16
#define SPARE_COUNT			4
#define SPARE_SIZE			(SPARE_LEN * SPARE_COUNT)
#endif

#if defined(CONFIG_ARCH_MXC_HAS_NFC_V3_1) ||	\
    defined(CONFIG_ARCH_MXC_HAS_NFC_V3_2)
#define NFC_SPAS_WIDTH 8
#define NFC_SPAS_SHIFT 16

#define NFC_SET_SPAS(v)			\
	raw_write((((raw_read(NFC_CONFIG2) & \
	NFC_FIELD_RESET(NFC_SPAS_WIDTH, NFC_SPAS_SHIFT)) | ((v) << 16))), \
	NFC_CONFIG2)

#define NFC_SET_ECC_MODE(v)		\
do { \
	if (cpu_is_mx53() > 0) { \
		if ((v) == NFC_SPAS_218 || (v) == NFC_SPAS_112) \
			raw_write(((raw_read(NFC_CONFIG2) & \
					~(3 << 6)) | \
					NFC_ECC_MODE_16), NFC_CONFIG2); \
		else \
			raw_write(((raw_read(NFC_CONFIG2) & \
					~(3 << 6)) | \
					NFC_ECC_MODE_4), NFC_CONFIG2); \
	} else if (cpu_is_mx51()) { \
		if ((v) == NFC_SPAS_218 || (v) == NFC_SPAS_112) \
			raw_write(((raw_read(NFC_CONFIG2) & \
					~(1 << 6)) | \
					NFC_ECC_MODE_8), NFC_CONFIG2); \
		else \
			raw_write(((raw_read(NFC_CONFIG2) & \
					~(1 << 6)) | \
					NFC_ECC_MODE_4), NFC_CONFIG2); \
	} else { \
		if ((v) == NFC_SPAS_218 || (v) == NFC_SPAS_112) \
			raw_write(((raw_read(NFC_CONFIG2) & \
					~(1 << 6)) | \
					NFC_ECC_MODE_4), NFC_CONFIG2); \
		else \
			raw_write(((raw_read(NFC_CONFIG2) & \
					~(1 << 6)) | \
					NFC_ECC_MODE_8), NFC_CONFIG2); \
	} \
} while (0)

#define WRITE_NFC_IP_REG(val, reg) 			\
	do {	 					\
		raw_write(raw_read(NFC_IPC) | NFC_IPC_CREQ, NFC_IPC);	\
		while (!(raw_read(NFC_IPC) & NFC_IPC_ACK)) \
			; \
		raw_write(val, reg);			\
		raw_write(raw_read(NFC_IPC) & ~NFC_IPC_CREQ, NFC_IPC);	\
	} while (0)

#else
#define NFC_SET_SPAS(v)
#define NFC_SET_ECC_MODE(v)
#define NFC_SET_NFMS(v)	(NFMS |= (v))

#define WRITE_NFC_IP_REG(val, reg) \
	raw_write((raw_read(REG_NFC_OPS_STAT) & ~NFC_OPS_STAT),  \
	REG_NFC_OPS_STAT)
#endif

#define GET_NFC_ECC_STATUS() raw_read(REG_NFC_ECC_STATUS_RESULT);

/*!
 * Set 1 to specific operation bit, rest to 0 in LAUNCH_NFC Register for
 * Specific operation
 */
#define NFC_CMD            		0x1
#define NFC_ADDR           		0x2
#define NFC_INPUT          		0x4
#define NFC_OUTPUT         		0x8
#define NFC_ID             		0x10
#define NFC_STATUS         		0x20

#ifdef CONFIG_ARCH_MXC_HAS_NFC_V3_2	/* mx51 */
#define NFC_AUTO_PROG 			0x40
#define NFC_AUTO_READ           	0x80
#define NFC_AUTO_ERASE          	0x200
#define NFC_COPY_BACK_0			0x400
#define NFC_COPY_BACK_1         	0x800
#define NFC_AUTO_STATE          	0x1000
#endif

/* Bit Definitions for NFC_IPC*/
#define NFC_OPS_STAT			(1 << 31)

#ifdef CONFIG_ARCH_MXC_HAS_NFC_V3_2	/* mx51 */
#define NFC_OP_DONE			(1 << 30)
#define NFC_RB				(1 << 28)
#define NFC_PS_WIDTH 			2
#define NFC_PS_SHIFT 			0
#define NFC_PS_512	 		0
#define NFC_PS_2K	 		1
#define NFC_PS_4K    			2
#else
#define NFC_RB				(1 << 29)
#endif

#define NFC_ONE_CYCLE			(1 << 2)

#ifdef CONFIG_ARCH_MXC_HAS_NFC_V3_2	/* mx51 */
#define NFC_INT_MSK			(1 << 15)
#define NFC_AUTO_PROG_DONE_MSK 		(1 << 14)
#define NFC_NUM_ADDR_PHASE1_WIDTH   	2
#define NFC_NUM_ADDR_PHASE1_SHIFT  	12

#define NFC_NUM_ADDR_PHASE0_WIDTH 	1
#define NFC_NUM_ADDR_PHASE0_SHIFT  	5

#define NFC_ONE_LESS_PHASE1 		0
#define NFC_TWO_LESS_PHASE1 		1

#define NFC_FLASH_ADDR_SHIFT		0
#else
#define NFC_INT_MSK			(1 << 4)
#define NFC_BIG				(1 << 5)
#define NFC_FLASH_ADDR_SHIFT		16
#endif

#define NFC_UNLOCK_END_ADDR_SHIFT	16

/* Bit definition for NFC_CONFIGRATION_1 */
#define NFC_SP_EN			(1 << 0)
#define NFC_CE				(1 << 1)
#define NFC_RST				(1 << 2)
#define NFC_ECC_EN			(1 << 3)

#define NFC_FIELD_RESET(width, shift)	(~((((1 << (width)) - 1) << (shift))))

#define NFC_RBA_SHIFT       		4

#if defined(CONFIG_ARCH_MXC_HAS_NFC_V3_1) ||	\
    defined(CONFIG_ARCH_MXC_HAS_NFC_V3_2)	/* mx51 */
#define NFC_RBA_WIDTH			3
#else
#define NFC_RBA_WIDTH			2
#endif

#if defined(CONFIG_ARCH_MXC_HAS_NFC_V3_2)	/* mx51 */
#define NFC_ITERATION_SHIFT 8
#define NFC_ITERATION_WIDTH 4
#define NFC_ACTIVE_CS_SHIFT 12
#define NFC_ACTIVE_CS_WIDTH 3
/* bit definition for CONFIGRATION3 */
#define NFC_NO_SDMA			(1 << 20)
#define NFC_FMP_SHIFT 			16
#define NFC_FMP_WIDTH			4
#define NFC_RBB_MODE			(1 << 15)
#define NFC_NUM_OF_DEVICES_SHIFT 	12
#define NFC_NUM_OF_DEVICES_WIDTH 	4
#define NFC_DMA_MODE_SHIFT 		11
#define NFC_DMA_MODE_WIDTH  		1
#define NFC_SBB_SHIFT 			8
#define NFC_SBB_WIDTH 			3
#define NFC_BIG				(1 << 7)
#define NFC_SB2R_SHIFT 			4
#define NFC_SB2R_WIDTH			3
#define NFC_FW_SHIFT    		3
#define NFC_FW_WIDTH 			1
#define NFC_TOO				(1 << 2)
#define NFC_ADD_OP_SHIFT 		0
#define NFC_ADD_OP_WIDTH		2
#define NFC_FW_8 			1
#define NFC_FW_16			0
#define NFC_ST_CMD_SHITF		24
#define NFC_ST_CMD_WIDTH		8
#endif

#define NFC_PPB_32			(0 << 7)
#define NFC_PPB_64			(1 << 7)
#define NFC_PPB_128			(2 << 7)
#define NFC_PPB_256			(3 << 7)
#define NFC_PPB_RESET			(~(3 << 7))

#if defined(CONFIG_ARCH_MXC_HAS_NFC_V3_2)
#define NFC_BLS_LOCKED			(0 << 6)
#define NFC_BLS_LOCKED_DEFAULT		(1 << 6)
#define NFC_BLS_UNLCOKED		(2 << 6)
#define NFC_BLS_RESET			(~(3 << 6))
#else
#define NFC_BLS_LOCKED                  (0 << 16)
#define NFC_BLS_LOCKED_DEFAULT          (1 << 16)
#define NFC_BLS_UNLCOKED                (2 << 16)
#define NFC_BLS_RESET                   (~(3 << 16))
#endif

#define NFC_WPC_LOCK_TIGHT		1
#define NFC_WPC_LOCK			(1 << 1)
#define NFC_WPC_UNLOCK			(1 << 2)
#define NFC_WPC_RESET			(~(7))
#if defined(CONFIG_ARCH_MXC_HAS_NFC_V3_1) || \
    defined(CONFIG_ARCH_MXC_HAS_NFC_V3_2)
#define NFC_ECC_MODE_4    		(0x0 << 6)
#define NFC_ECC_MODE_8			(0x1 << 6)
#define NFC_ECC_MODE_16                 (0x3 << 6)
#define NFC_SPAS_16			8
#define NFC_SPAS_64		 	32
#define NFC_SPAS_128			64
#define NFC_SPAS_112			56
#define NFC_SPAS_218		 	109
#define NFC_IPC_CREQ			(1 << 0)
#define NFC_IPC_ACK			(1 << 1)
#endif

#define REG_NFC_OPS_STAT		NFC_IPC
#define REG_NFC_INTRRUPT		NFC_CONFIG2
#ifdef CONFIG_ARCH_MXC_HAS_NFC_V3_2
#define REG_NFC_FLASH_ADDR		NFC_FLASH_ADDR0
#define REG_NFC_FLASH_CMD		NFC_FLASH_CMD
#else
#define REG_NFC_FLASH_ADDR		NFC_FLASH_ADDR_CMD
#define REG_NFC_FLASH_CMD		NFC_FLASH_ADDR_CMD
#endif
#define REG_NFC_OPS			LAUNCH_NFC
#define REG_NFC_SET_RBA			NFC_CONFIG1
#define REG_NFC_RB			NFC_IPC
#define REG_NFC_ECC_EN			NFC_CONFIG2
#define REG_NFC_ECC_STATUS_RESULT	NFC_ECC_STATUS_RESULT
#define REG_NFC_CE			NFC_CONFIG1
#define REG_NFC_RST			NFC_CONFIG1
#define REG_NFC_PPB			NFC_CONFIG2
#define REG_NFC_SP_EN			NFC_CONFIG1
#define REG_NFC_BLS			NFC_WRPROT
#define REG_UNLOCK_BLK_ADD0		NFC_WRPROT_UNLOCK_BLK_ADD0
#define REG_UNLOCK_BLK_ADD1		NFC_WRPROT_UNLOCK_BLK_ADD1
#define REG_UNLOCK_BLK_ADD2		NFC_WRPROT_UNLOCK_BLK_ADD2
#define REG_UNLOCK_BLK_ADD3		NFC_WRPROT_UNLOCK_BLK_ADD3
#define REG_NFC_WPC			NFC_WRPROT
#define REG_NFC_ONE_CYCLE		NFC_CONFIG2

/* NFC V3 Specific MACRO functions definitions */
#define raw_write(v, a)		__raw_writel(v, a)
#define raw_read(a)		__raw_readl(a)

/* Set RBA buffer id*/
#define NFC_SET_RBA(val)       \
	raw_write((raw_read(REG_NFC_SET_RBA) & \
	(NFC_FIELD_RESET(NFC_RBA_WIDTH, NFC_RBA_SHIFT))) | \
	((val) << NFC_RBA_SHIFT), REG_NFC_SET_RBA);

#define NFC_SET_PS(val)       \
	raw_write((raw_read(NFC_CONFIG2) & \
	(NFC_FIELD_RESET(NFC_PS_WIDTH, NFC_PS_SHIFT))) | \
	((val) << NFC_PS_SHIFT), NFC_CONFIG2);

#ifdef CONFIG_ARCH_MXC_HAS_NFC_V3_2
#define UNLOCK_ADDR(start_addr, end_addr)     \
{ \
	int i = 0; \
	for (; i < NAND_MAX_CHIPS; i++)  \
	raw_write(start_addr | \
	(end_addr << NFC_UNLOCK_END_ADDR_SHIFT), \
	REG_UNLOCK_BLK_ADD0 + (i << 2)); \
}
#define NFC_SET_NFC_ACTIVE_CS(val) \
	raw_write((raw_read(NFC_CONFIG1) & \
	(NFC_FIELD_RESET(NFC_ACTIVE_CS_WIDTH, NFC_ACTIVE_CS_SHIFT))) | \
	((val) << NFC_ACTIVE_CS_SHIFT), NFC_CONFIG1);

#define NFC_GET_NFC_ACTIVE_CS() \
	((raw_read(NFC_CONFIG1) >> NFC_ACTIVE_CS_SHIFT) & \
	((1 << NFC_ACTIVE_CS_WIDTH) - 1))

#define NFC_GET_MAXCHIP_SP() 		8

#else
#define UNLOCK_ADDR(start_addr, end_addr)     \
	raw_write(start_addr | \
	(end_addr << NFC_UNLOCK_END_ADDR_SHIFT), REG_UNLOCK_BLK_ADD0);

#define NFC_SET_NFC_ACTIVE_CS(val)
#define NFC_GET_NFC_ACTIVE_CS()
#define NFC_GET_MAXCHIP_SP() 		1
#endif

#define NFC_SET_BLS(val) ((raw_read(REG_NFC_BLS) & NFC_BLS_RESET) | val)
#define NFC_SET_WPC(val) ((raw_read(REG_NFC_WPC) & NFC_WPC_RESET) | val)
#define CHECK_NFC_RB    (raw_read(REG_NFC_RB) & NFC_RB)

#if defined(CONFIG_ARCH_MXC_HAS_NFC_V3_2)
#define NFC_SET_NFC_NUM_ADDR_PHASE1(val) \
	raw_write((raw_read(NFC_CONFIG2) & \
	(NFC_FIELD_RESET(NFC_NUM_ADDR_PHASE1_WIDTH, \
	NFC_NUM_ADDR_PHASE1_SHIFT))) | \
	((val) << NFC_NUM_ADDR_PHASE1_SHIFT), NFC_CONFIG2);

#define NFC_SET_NFC_NUM_ADDR_PHASE0(val) \
	raw_write((raw_read(NFC_CONFIG2) & \
	(NFC_FIELD_RESET(NFC_NUM_ADDR_PHASE0_WIDTH, \
	NFC_NUM_ADDR_PHASE0_SHIFT))) | \
	((val) << NFC_NUM_ADDR_PHASE0_SHIFT), NFC_CONFIG2);

#define NFC_SET_NFC_ITERATION(val) \
	raw_write((raw_read(NFC_CONFIG1) & \
	(NFC_FIELD_RESET(NFC_ITERATION_WIDTH, NFC_ITERATION_SHIFT))) | \
	((val) << NFC_ITERATION_SHIFT), NFC_CONFIG1);

#define NFC_SET_FW(val) \
	raw_write((raw_read(NFC_CONFIG3) & \
	(NFC_FIELD_RESET(NFC_FW_WIDTH, NFC_FW_SHIFT))) | \
	((val) << NFC_FW_SHIFT), NFC_CONFIG3);

#define NFC_SET_NUM_OF_DEVICE(val) \
	raw_write((raw_read(NFC_CONFIG3) & \
	(NFC_FIELD_RESET(NFC_NUM_OF_DEVICES_WIDTH, \
	NFC_NUM_OF_DEVICES_SHIFT))) | \
	((val) << NFC_NUM_OF_DEVICES_SHIFT), NFC_CONFIG3);

#define NFC_SET_ADD_OP_MODE(val) \
	 raw_write((raw_read(NFC_CONFIG3) & \
	(NFC_FIELD_RESET(NFC_ADD_OP_WIDTH, NFC_ADD_OP_SHIFT))) | \
	((val) << NFC_ADD_OP_SHIFT), NFC_CONFIG3);

#define NFC_SET_ADD_CS_MODE(val) \
{ \
	NFC_SET_ADD_OP_MODE(val); \
	NFC_SET_NUM_OF_DEVICE(this->numchips - 1); \
}

#define NFC_SET_ST_CMD(val) \
	raw_write((raw_read(NFC_CONFIG2) & \
	(NFC_FIELD_RESET(NFC_ST_CMD_WIDTH, \
	NFC_ST_CMD_SHITF))) | \
	((val) << NFC_ST_CMD_SHITF), NFC_CONFIG2);

#define NFMS_NF_DWIDTH 0
#define NFMS_NF_PG_SZ  1
#define NFC_CMD_1_SHIFT 8

#define NUM_OF_ADDR_CYCLE (fls(g_page_mask) >> 3)
#define SET_NFC_DELAY_LINE(val) raw_write((val), NFC_DELAY_LINE)

/*should set the fw,ps,spas,ppb*/
#define NFC_SET_NFMS(v)	\
do {	\
	if (!(v)) \
		NFC_SET_FW(NFC_FW_8);	\
	if (((v) & (1 << NFMS_NF_DWIDTH)))	\
		NFC_SET_FW(NFC_FW_16);	\
	if (((v) & (1 << NFMS_NF_PG_SZ))) {	\
		if (IS_2K_PAGE_NAND) {	\
			NFC_SET_PS(NFC_PS_2K);	\
			NFC_SET_NFC_NUM_ADDR_PHASE1(NUM_OF_ADDR_CYCLE); \
			NFC_SET_NFC_NUM_ADDR_PHASE0(NFC_TWO_LESS_PHASE1); \
		} else if (IS_4K_PAGE_NAND) {       \
			NFC_SET_PS(NFC_PS_4K);	\
			NFC_SET_NFC_NUM_ADDR_PHASE1(NUM_OF_ADDR_CYCLE); \
			NFC_SET_NFC_NUM_ADDR_PHASE0(NFC_TWO_LESS_PHASE1); \
		} else {	\
			NFC_SET_PS(NFC_PS_512);	\
			NFC_SET_NFC_NUM_ADDR_PHASE1(NUM_OF_ADDR_CYCLE - 1); \
			NFC_SET_NFC_NUM_ADDR_PHASE0(NFC_ONE_LESS_PHASE1); \
		}	\
		NFC_SET_ADD_CS_MODE(1); \
		NFC_SET_SPAS(GET_NAND_OOB_SIZE >> 1);	\
		NFC_SET_ECC_MODE(GET_NAND_OOB_SIZE >> 1); \
		NFC_SET_ST_CMD(0x70); \
		raw_write(raw_read(NFC_CONFIG3) | NFC_NO_SDMA, NFC_CONFIG3); \
		raw_write(raw_read(NFC_CONFIG3) | NFC_RBB_MODE, NFC_CONFIG3); \
		if (cpu_is_mx51()) \
			SET_NFC_DELAY_LINE(0); \
	} \
} while (0)
#endif

#ifdef CONFIG_ARCH_MXC_HAS_NFC_V3_1
#define NFC_SET_NFMS(v)
#endif

#define READ_PAGE()	send_read_page(0)
#define PROG_PAGE() 	send_prog_page(0)

#elif defined(CONFIG_ARCH_MXC_HAS_NFC_V2)

/*
 * For V1/V2 NFC registers Definition
 */

/*
 * Addresses for NFC registers
 */
#ifdef CONFIG_ARCH_MXC_HAS_NFC_V2_1
#define NFC_REG_BASE			(nfc_axi_base + 0x1000)
#else
#define NFC_REG_BASE			nfc_axi_base
#endif
#define NFC_BUF_SIZE            	(NFC_REG_BASE + 0xE00)
#define NFC_BUF_ADDR            	(NFC_REG_BASE + 0xE04)
#define NFC_FLASH_ADDR          	(NFC_REG_BASE + 0xE06)
#define NFC_FLASH_CMD           	(NFC_REG_BASE + 0xE08)
#define NFC_CONFIG              	(NFC_REG_BASE + 0xE0A)
#ifdef CONFIG_ARCH_MXC_HAS_NFC_V2_1
#define NFC_ECC_STATUS_RESULT		(NFC_REG_BASE + 0xE0C)
#define NFC_ECC_STATUS_RESULT_1		(NFC_REG_BASE + 0xE0C)
#define NFC_ECC_STATUS_RESULT_2		(NFC_REG_BASE + 0xE0E)
#define NFC_SPAS			(NFC_REG_BASE + 0xE10)
#else
#define NFC_ECC_STATUS_RESULT   	(NFC_REG_BASE + 0xE0C)
#define NFC_RSLTMAIN_AREA       	(NFC_REG_BASE + 0xE0E)
#define NFC_RSLTSPARE_AREA      	(NFC_REG_BASE + 0xE10)
#endif
#define NFC_WRPROT              	(NFC_REG_BASE + 0xE12)
#ifdef CONFIG_ARCH_MXC_HAS_NFC_V2_1
#define NFC_UNLOCKSTART_BLKADDR  	(NFC_REG_BASE + 0xE20)
#define NFC_UNLOCKEND_BLKADDR    	(NFC_REG_BASE + 0xE22)
#define NFC_UNLOCKSTART_BLKADDR1 	(NFC_REG_BASE + 0xE24)
#define NFC_UNLOCKEND_BLKADDR1   	(NFC_REG_BASE + 0xE26)
#define NFC_UNLOCKSTART_BLKADDR2 	(NFC_REG_BASE + 0xE28)
#define NFC_UNLOCKEND_BLKADDR2   	(NFC_REG_BASE + 0xE2A)
#define NFC_UNLOCKSTART_BLKADDR3 	(NFC_REG_BASE + 0xE2C)
#define NFC_UNLOCKEND_BLKADDR3   	(NFC_REG_BASE + 0xE2E)
#else
#define NFC_UNLOCKSTART_BLKADDR  	(NFC_REG_BASE + 0xE14)
#define NFC_UNLOCKEND_BLKADDR    	(NFC_REG_BASE + 0xE16)
#endif
#define NFC_NF_WRPRST            	(NFC_REG_BASE + 0xE18)
#define NFC_CONFIG1              	(NFC_REG_BASE + 0xE1A)
#define NFC_CONFIG2              	(NFC_REG_BASE + 0xE1C)

/*!
 * Addresses for NFC RAM BUFFER Main area 0
 */
#define MAIN_AREA0      		(u16 *)(nfc_axi_base + 0x000)
#define MAIN_AREA1      		(u16 *)(nfc_axi_base + 0x200)

/*!
 * Addresses for NFC SPARE BUFFER Spare area 0
 */
#ifdef CONFIG_ARCH_MXC_HAS_NFC_V2_1
#define SPARE_AREA0     		(u16 *)(nfc_axi_base + 0x1000)
#define SPARE_LEN			64
#define SPARE_COUNT			8
#else
#define SPARE_AREA0     		(u16 *)(nfc_axi_base + 0x800)
#define SPARE_LEN       		16
#define SPARE_COUNT     		4
#endif
#define SPARE_SIZE      		(SPARE_LEN * SPARE_COUNT)

#ifdef CONFIG_ARCH_MXC_HAS_NFC_V2_1
#define REG_NFC_ECC_MODE NFC_CONFIG1
#define SPAS_SHIFT		(0)
#define REG_NFC_SPAS NFC_SPAS
#define SPAS_MASK	(0xFF00)

#define NFC_SET_SPAS(v)			\
	raw_write(((raw_read(REG_NFC_SPAS) & SPAS_MASK) | ((v<<SPAS_SHIFT))), \
								REG_NFC_SPAS)

#define NFC_SET_ECC_MODE(v) 		 \
do {	\
	if ((v) == NFC_SPAS_218 || (v) == NFC_SPAS_112)  {	\
		raw_write((raw_read(REG_NFC_ECC_MODE) & NFC_ECC_MODE_8), \
							REG_NFC_ECC_MODE); \
	} else {	\
		raw_write((raw_read(REG_NFC_ECC_MODE) | NFC_ECC_MODE_4), \
							REG_NFC_ECC_MODE); \
	}	\
} while (0)

#define GET_ECC_STATUS()  __raw_readl(REG_NFC_ECC_STATUS_RESULT);
#define NFC_SET_NFMS(v) \
do { \
	if (((v) & (1 << NFMS_NF_PG_SZ))) { \
		if (IS_2K_PAGE_NAND) { \
			(NFMS |= 0x00000100); \
			(NFMS &= ~0x00000200); \
			NFC_SET_SPAS(NFC_SPAS_64); \
		} else if (IS_4K_PAGE_NAND) { \
			(NFMS &= ~0x00000100); \
			(NFMS |= 0x00000200); \
			GET_NAND_OOB_SIZE == 128 ? \
			NFC_SET_SPAS(NFC_SPAS_128) : \
			NFC_SET_SPAS(NFC_SPAS_218); \
		} else { \
			printk(KERN_ERR "Err for setting page/oob size"); \
		} \
		NFC_SET_ECC_MODE(GET_NAND_OOB_SIZE >> 1); \
	} \
} while (0)
#else
#define NFC_SET_SPAS(v)
#define NFC_SET_ECC_MODE(v)
#define GET_ECC_STATUS()  raw_read(REG_NFC_ECC_STATUS_RESULT);
#define NFC_SET_NFMS(v)     (NFMS |= (v))
#endif

#define WRITE_NFC_IP_REG(val, reg) \
	raw_write((raw_read(REG_NFC_OPS_STAT) & ~NFC_OPS_STAT),  \
	REG_NFC_OPS_STAT)

#define GET_NFC_ECC_STATUS() raw_read(REG_NFC_ECC_STATUS_RESULT);

/*!
 * Set INT to 0, Set 1 to specific operation bit, rest to 0 in LAUNCH_NFC Register for
 * Specific operation
 */
#define NFC_CMD            		0x1
#define NFC_ADDR           		0x2
#define NFC_INPUT          		0x4
#define NFC_OUTPUT         		0x8
#define NFC_ID             		0x10
#define NFC_STATUS         		0x20

/* Bit Definitions */
#define NFC_OPS_STAT			(1 << 15)
#define NFC_SP_EN           		(1 << 2)
#define NFC_ECC_EN          		(1 << 3)
#define NFC_INT_MSK         		(1 << 4)
#define NFC_BIG             		(1 << 5)
#define NFC_RST             		(1 << 6)
#define NFC_CE              		(1 << 7)
#define NFC_ONE_CYCLE       		(1 << 8)
#define NFC_BLS_LOCKED			0
#define NFC_BLS_LOCKED_DEFAULT		1
#define NFC_BLS_UNLCOKED		2
#define NFC_WPC_LOCK_TIGHT		1
#define NFC_WPC_LOCK			(1 << 1)
#define NFC_WPC_UNLOCK			(1 << 2)
#define NFC_FLASH_ADDR_SHIFT 		0
#define NFC_UNLOCK_END_ADDR_SHIFT	0

#ifdef CONFIG_ARCH_MXC_HAS_NFC_V2_1
#define NFC_ECC_MODE_4    		 (1<<0)
#define NFC_ECC_MODE_8			 (~(1<<0))
#define NFC_SPAS_16			 8
#define NFC_SPAS_64			 32
#define NFC_SPAS_112			 56
#define NFC_SPAS_128			 64
#define NFC_SPAS_218			 109
#endif
/* NFC Register Mapping */
#define REG_NFC_OPS_STAT		NFC_CONFIG2
#define REG_NFC_INTRRUPT		NFC_CONFIG1
#define REG_NFC_FLASH_ADDR		NFC_FLASH_ADDR
#define REG_NFC_FLASH_CMD		NFC_FLASH_CMD
#define REG_NFC_OPS			NFC_CONFIG2
#define REG_NFC_SET_RBA			NFC_BUF_ADDR
#define REG_NFC_ECC_EN			NFC_CONFIG1
#define REG_NFC_ECC_STATUS_RESULT  	NFC_ECC_STATUS_RESULT
#define REG_NFC_CE			NFC_CONFIG1
#define REG_NFC_SP_EN			NFC_CONFIG1
#define REG_NFC_BLS			NFC_CONFIG
#define REG_NFC_WPC			NFC_WRPROT
#define REG_START_BLKADDR      		NFC_UNLOCKSTART_BLKADDR
#define REG_END_BLKADDR        		NFC_UNLOCKEND_BLKADDR
#define REG_NFC_RST			NFC_CONFIG1
#define REG_NFC_ONE_CYCLE		NFC_CONFIG1

/* NFC V1/V2 Specific MACRO functions definitions */

#define raw_write(v, a)                  __raw_writew(v, a)
#define raw_read(a)                     __raw_readw(a)

#define NFC_SET_BLS(val)  		val

#define UNLOCK_ADDR(start_addr, end_addr)		\
{							\
	raw_write(start_addr, REG_START_BLKADDR);	\
	raw_write(end_addr, REG_END_BLKADDR);		\
}

#define NFC_SET_NFC_ACTIVE_CS(val)
#define NFC_GET_NFC_ACTIVE_CS()
#define NFC_GET_MAXCHIP_SP() 		1
#define NFC_SET_WPC(val)                val

#define NFC_SET_RBA(val) raw_write(val, REG_NFC_SET_RBA);

#ifdef CONFIG_ARCH_MXC_HAS_NFC_V2_1
#define READ_PAGE()	send_read_page(0)
#define PROG_PAGE() 	send_prog_page(0)
#else
#define READ_PAGE()   \
do {                     \
	send_read_page(0);  \
	send_read_page(1);  \
	send_read_page(2);  \
	send_read_page(3);  \
} while (0)

#define PROG_PAGE()   \
do {                     \
	send_prog_page(0);  \
	send_prog_page(1);  \
	send_prog_page(2);  \
	send_prog_page(3);  \
} while (0)
#endif
#define CHECK_NFC_RB            1

#endif

#endif				/* __MXC_ND2_H__ */
