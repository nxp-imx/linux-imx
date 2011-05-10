/*
 *  linux/drivers/mmc/host/mx_sdhci.h - Secure Digital Host
 *  Controller Interface driver
 *
 *  Copyright (C) 2008-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

/*
 * Controller registers
 */

#define SDHCI_DMA_ADDRESS	0x00

#define SDHCI_BLOCK_SIZE	0x04
#define  SDHCI_MAKE_BLKSZ(dma, blksz) (((dma & 0x7) << 13) | (blksz & 0x1FFF))

#define SDHCI_BLOCK_COUNT	0x04

#define SDHCI_ARGUMENT		0x08

#define SDHCI_TRANSFER_MODE	0x0C
#define  SDHCI_TRNS_DMA		0x00000001
#define  SDHCI_TRNS_BLK_CNT_EN	0x00000002
#define  SDHCI_TRNS_ACMD12	0x00000004
#define  SDHCI_TRNS_DDR_EN 	0x00000008
#define  SDHCI_TRNS_READ	0x00000010
#define  SDHCI_TRNS_MULTI	0x00000020
#define  SDHCI_TRNS_DPSEL	0x00200000
#define  SDHCI_TRNS_ABORTCMD	0x00C00000
#define  SDHCI_TRNS_MASK	0xFFFF0000

#define SDHCI_COMMAND		0x0E
#define  SDHCI_CMD_RESP_MASK	0x03
#define  SDHCI_CMD_CRC		0x08
#define  SDHCI_CMD_INDEX	0x10
#define  SDHCI_CMD_DATA		0x20

#define  SDHCI_CMD_RESP_NONE	0x00
#define  SDHCI_CMD_RESP_LONG	0x01
#define  SDHCI_CMD_RESP_SHORT	0x02
#define  SDHCI_CMD_RESP_SHORT_BUSY 0x03

#define SDHCI_MAKE_CMD(c, f) ((((c & 0xff) << 8) | (f & 0xff)) << 16)

#define SDHCI_RESPONSE		0x10

#define SDHCI_BUFFER		0x20

#define SDHCI_PRESENT_STATE	0x24
#define  SDHCI_CMD_INHIBIT	0x00000001
#define  SDHCI_DATA_INHIBIT	0x00000002
#define  SDHCI_DATA_ACTIVE 	0x00000004
#define  SDHCI_DOING_WRITE	0x00000100
#define  SDHCI_DOING_READ	0x00000200
#define  SDHCI_SPACE_AVAILABLE	0x00000400
#define  SDHCI_DATA_AVAILABLE	0x00000800
#define  SDHCI_CARD_PRESENT	0x00010000
#define  SDHCI_WRITE_PROTECT	0x00080000
#define  SDHCI_DAT0_IDLE	0x01000000
#define  SDHCI_CARD_INT_MASK	0x0E000000
#define  SDHCI_CARD_INT_ID	0x0C000000

#define SDHCI_HOST_CONTROL 	0x28
#define  SDHCI_CTRL_LED		0x00000001
#define  SDHCI_CTRL_4BITBUS	0x00000002
#define  SDHCI_CTRL_8BITBUS	0x00000004
#define  SDHCI_CTRL_HISPD	0x00000004
#define  SDHCI_CTRL_CDSS       0x80
#define  SDHCI_CTRL_DMA_MASK	0x18
#define   SDHCI_CTRL_SDMA	0x00
#define   SDHCI_CTRL_ADMA1	0x08
#define   SDHCI_CTRL_ADMA32	0x10
#define   SDHCI_CTRL_ADMA64	0x18
#define  SDHCI_CTRL_D3CD 	0x00000008
#define  SDHCI_CTRL_ADMA 	0x00000100
/* wake up control */
#define  SDHCI_CTRL_WECINS 	0x04000000

#define SDHCI_POWER_CONTROL	0x29
#define  SDHCI_POWER_ON		0x01
#define  SDHCI_POWER_180	0x0A
#define  SDHCI_POWER_300	0x0C
#define  SDHCI_POWER_330	0x0E

#define SDHCI_BLOCK_GAP_CONTROL	0x2A

#define SDHCI_WAKE_UP_CONTROL	0x2B

#define SDHCI_CLOCK_CONTROL	0x2C
#define  SDHCI_DIVIDER_SHIFT	8
#define  SDHCI_CLOCK_SD_EN	0x00000008
#define  SDHCI_CLOCK_PER_EN	0x00000004
#define  SDHCI_CLOCK_HLK_EN	0x00000002
#define  SDHCI_CLOCK_IPG_EN	0x00000001
#define  SDHCI_CLOCK_SDCLKFS1 	0x00000100
#define  SDHCI_CLOCK_MASK 	0x0000FFFF

#define SDHCI_TIMEOUT_CONTROL	0x2E

#define SDHCI_SOFTWARE_RESET	0x2F
#define  SDHCI_RESET_ALL	0x01
#define  SDHCI_RESET_CMD	0x02
#define  SDHCI_RESET_DATA	0x04

#define SDHCI_INT_STATUS	0x30
#define SDHCI_INT_ENABLE	0x34
#define SDHCI_SIGNAL_ENABLE	0x38
#define  SDHCI_INT_RESPONSE	0x00000001
#define  SDHCI_INT_DATA_END	0x00000002
#define  SDHCI_INT_DMA_END	0x00000008
#define  SDHCI_INT_SPACE_AVAIL	0x00000010
#define  SDHCI_INT_DATA_AVAIL	0x00000020
#define  SDHCI_INT_CARD_INSERT	0x00000040
#define  SDHCI_INT_CARD_REMOVE	0x00000080
#define  SDHCI_INT_CARD_INT	0x00000100
#define  SDHCI_INT_ERROR	0x00008000
#define  SDHCI_INT_TIMEOUT	0x00010000
#define  SDHCI_INT_CRC		0x00020000
#define  SDHCI_INT_END_BIT	0x00040000
#define  SDHCI_INT_INDEX	0x00080000
#define  SDHCI_INT_DATA_TIMEOUT	0x00100000
#define  SDHCI_INT_DATA_CRC	0x00200000
#define  SDHCI_INT_DATA_END_BIT	0x00400000
#define  SDHCI_INT_BUS_POWER	0x00800000
#define  SDHCI_INT_ACMD12ERR	0x01000000
#define  SDHCI_INT_ADMA_ERROR	0x10000000

#define  SDHCI_INT_NORMAL_MASK	0x00007FFF
#define  SDHCI_INT_ERROR_MASK	0xFFFF8000

#define  SDHCI_INT_CMD_MASK	(SDHCI_INT_RESPONSE | SDHCI_INT_TIMEOUT | \
		SDHCI_INT_CRC | SDHCI_INT_END_BIT | SDHCI_INT_INDEX | \
		SDHCI_INT_ACMD12ERR)
#define  SDHCI_INT_DATA_MASK	(SDHCI_INT_DATA_END | SDHCI_INT_DMA_END | \
		SDHCI_INT_DATA_AVAIL | SDHCI_INT_SPACE_AVAIL | \
		SDHCI_INT_DATA_TIMEOUT | SDHCI_INT_DATA_CRC | \
		SDHCI_INT_DATA_END_BIT | SDHCI_INT_ADMA_ERROR)
#define  SDHCI_INT_DATA_RE_MASK	(SDHCI_INT_DMA_END | \
		SDHCI_INT_DATA_AVAIL | SDHCI_INT_SPACE_AVAIL)

#define SDHCI_ACMD12_ERR	0x3C
#define SDHCI_ACMD12_ERR_NE 	0x00000001
#define SDHCI_ACMD12_ERR_TOE 	0x00000002
#define SDHCI_ACMD12_ERR_EBE 	0x00000004
#define SDHCI_ACMD12_ERR_CE 	0x00000008
#define SDHCI_ACMD12_ERR_IE 	0x00000010
#define SDHCI_ACMD12_ERR_CNIBE 	0x00000080

/* 3E-3F reserved */

#define SDHCI_CAPABILITIES	0x40
#define  SDHCI_TIMEOUT_CLK_MASK	0x0000003F
#define  SDHCI_TIMEOUT_CLK_SHIFT 0
#define  SDHCI_TIMEOUT_CLK_UNIT	0x00000080
#define  SDHCI_CLOCK_BASE_MASK	0x00003F00
#define  SDHCI_CLOCK_BASE_SHIFT	8
#define  SDHCI_MAX_BLOCK_MASK	0x00030000
#define  SDHCI_MAX_BLOCK_SHIFT  16
#define  SDHCI_CAN_DO_ADMA2	0x00080000
#define  SDHCI_CAN_DO_ADMA1	0x00100000
#define  SDHCI_CAN_DO_HISPD	0x00200000
#define  SDHCI_CAN_DO_DMA	0x00400000
#define  SDHCI_CAN_VDD_330	0x01000000
#define  SDHCI_CAN_VDD_300	0x02000000
#define  SDHCI_CAN_VDD_180	0x04000000
#define  SDHCI_CAN_64BIT	0x10000000

/* 44-47 reserved for more caps */
#define SDHCI_WML 		0x44
#define  SDHCI_WML_4_WORDS 	0x00040004
#define  SDHCI_WML_16_WORDS 	0x00100010
#define  SDHCI_WML_64_WORDS 	0x00400040
#define  SDHCI_WML_128_WORDS 	0x00800080

#define SDHCI_MAX_CURRENT	0x48

/* 4C-4F reserved for more max current */

#define SDHCI_SET_ACMD12_ERROR	0x50
#define SDHCI_SET_INT_ERROR	0x52

#define SDHCI_ADMA_ERROR	0x54

/* 55-57 reserved */

#define SDHCI_ADMA_ADDRESS	0x58

/* 60-FB reserved */
#define SDHCI_DLL_CONTROL 	0x60
#define DLL_CTRL_ENABLE 	0x00000001
#define DLL_CTRL_RESET 		0x00000002
#define DLL_CTRL_SLV_FORCE_UPD 	0x00000004
#define DLL_CTRL_SLV_OVERRIDE	0x00000200
#define DLL_CTRL_SLV_DLY_TAR 	0x00000000
#define DLL_CTRL_SLV_UP_INT 	0x00200000
#define DLL_CTRL_REF_UP_INT 	0x20000000

#define SDHCI_DLL_STATUS 	0x64
#define DLL_STS_SLV_LOCK 	0x00000001
#define DLL_STS_REF_LOCK 	0x00000002

/* ADMA Addr Descriptor Attribute Filed */
enum {
	FSL_ADMA_DES_ATTR_VALID = 0x01,
	FSL_ADMA_DES_ATTR_END = 0x02,
	FSL_ADMA_DES_ATTR_INT = 0x04,
	FSL_ADMA_DES_ATTR_SET = 0x10,
	FSL_ADMA_DES_ATTR_TRAN = 0x20,
	FSL_ADMA_DES_ATTR_LINK = 0x30,
};

#define SDHCI_VENDOR_SPEC	0xC0

#define SDHCI_HOST_VERSION	0xFC
#define  SDHCI_VENDOR_VER_MASK	0xFF00
#define  SDHCI_VENDOR_VER_SHIFT	8
#define  SDHCI_SPEC_VER_MASK	0x00FF
#define  SDHCI_SPEC_VER_SHIFT	0
#define   SDHCI_SPEC_100	0
#define   SDHCI_SPEC_200	1
#define   ESDHC_VENDOR_V22 	0x12
#define   ESDHC_VENDOR_V3 	0x13

/* registers for usdhc */
#define USDHCI_MIXER_CTL        0x48
#define USDHCI_COMMAND		0x0C

/*
 * USDHCI_DLL_CONTROL
 */
#define USDHC_DLL_CTRL_SLV_OVERRIDE	0x100

struct sdhci_chip;

struct sdhci_host {
	struct sdhci_chip *chip;
	struct mmc_host *mmc;	/* MMC structure */

#ifdef CONFIG_LEDS_CLASS
	struct led_classdev led;	/* LED control */
#endif

	spinlock_t lock;	/* Mutex */

	int init_flag;		/* Host has been initialized */
	int flags;		/* Host attributes */
#define SDHCI_USE_DMA		(1<<0)	/* Host is DMA capable */
#define SDHCI_REQ_USE_DMA	(1<<1)	/* Use DMA for this req. */
#define SDHCI_USE_EXTERNAL_DMA	(1<<2)	/* Use the External DMA */
#define SDHCI_CD_PRESENT 	(1<<8)	/* CD present */
#define SDHCI_WP_ENABLED	(1<<9)	/* Write protect */
#define SDHCI_CD_TIMEOUT 	(1<<10)	/* cd timer is expired */

	unsigned int max_clk;	/* Max possible freq (MHz) */
	unsigned int min_clk;	/* Min possible freq (MHz) */
	unsigned int timeout_clk;	/* Timeout freq (KHz) */

	unsigned int clock;	/* Current clock (MHz) */
	unsigned short power;	/* Current voltage */
	struct regulator *regulator_mmc;	/*! Regulator */

	struct mmc_request *mrq;	/* Current request */
	struct mmc_command *cmd;	/* Current command */
	struct mmc_data *data;	/* Current data request */
	unsigned int data_early:1;	/* Data finished before cmd */

	unsigned int id;	/* Id for SD/MMC block */
	int mode;		/* SD/MMC mode */
	int dma;		/* DMA channel number. */
	unsigned int dma_size;	/* Number of Bytes in DMA */
	unsigned int dma_len;	/* Length of the s-g list */
	unsigned int dma_dir;	/* DMA transfer direction */

	struct scatterlist *cur_sg;	/* We're working on this */
	int num_sg;		/* Entries left */
	int offset;		/* Offset into current sg */
	int remain;		/* Bytes left in current */

	struct resource *res;	/* IO map memory */
	int irq;		/* Device IRQ */
	int detect_irq;		/* Card Detect IRQ number. */
	int sdio_enable;	/* sdio interrupt enable number. */
	struct clk *clk;	/* Clock id */
	int bar;		/* PCI BAR index */
	unsigned long addr;	/* Bus address */
	void __iomem *ioaddr;	/* Mapped address */

	struct tasklet_struct card_tasklet;	/* Tasklet structures */
	struct workqueue_struct	*workqueue;
	struct work_struct finish_wq;
	struct work_struct cd_wq;	/* card detection work queue */
	/* Platform specific data */
	struct mxc_mmc_platform_data *plat_data;

	unsigned int usdhc_en;	/* enable usdhc */
	struct timer_list timer;	/* Timer for timeouts */
	struct timer_list cd_timer;	/* Timer for cd */
};

struct sdhci_chip {
	struct platform_device *pdev;

	unsigned long quirks;

	int num_slots;		/* Slots on controller */
	struct sdhci_host *hosts[0];	/* Pointers to hosts */
};
