/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _IMX_SSI_H
#define _IMX_SSI_H

#include <mach/hardware.h>

/* SSI regs definition */
#define SSI1_IO_BASE_ADDR	IO_ADDRESS(SSI1_BASE_ADDR)
#define SSI2_IO_BASE_ADDR	IO_ADDRESS(SSI2_BASE_ADDR)

#define SSI1_STX0   ((SSI1_IO_BASE_ADDR) + 0x00)
#define SSI1_STX1   ((SSI1_IO_BASE_ADDR) + 0x04)
#define SSI1_SRX0   ((SSI1_IO_BASE_ADDR) + 0x08)
#define SSI1_SRX1   ((SSI1_IO_BASE_ADDR) + 0x0c)
#define SSI1_SCR    ((SSI1_IO_BASE_ADDR) + 0x10)
#define SSI1_SISR   ((SSI1_IO_BASE_ADDR) + 0x14)
#define SSI1_SIER   ((SSI1_IO_BASE_ADDR) + 0x18)
#define SSI1_STCR   ((SSI1_IO_BASE_ADDR) + 0x1c)
#define SSI1_SRCR   ((SSI1_IO_BASE_ADDR) + 0x20)
#define SSI1_STCCR  ((SSI1_IO_BASE_ADDR) + 0x24)
#define SSI1_SRCCR  ((SSI1_IO_BASE_ADDR) + 0x28)
#define SSI1_SFCSR  ((SSI1_IO_BASE_ADDR) + 0x2c)
#define SSI1_STR    ((SSI1_IO_BASE_ADDR) + 0x30)
#define SSI1_SOR    ((SSI1_IO_BASE_ADDR) + 0x34)
#define SSI1_SACNT  ((SSI1_IO_BASE_ADDR) + 0x38)
#define SSI1_SACADD ((SSI1_IO_BASE_ADDR) + 0x3c)
#define SSI1_SACDAT ((SSI1_IO_BASE_ADDR) + 0x40)
#define SSI1_SATAG  ((SSI1_IO_BASE_ADDR) + 0x44)
#define SSI1_STMSK  ((SSI1_IO_BASE_ADDR) + 0x48)
#define SSI1_SRMSK  ((SSI1_IO_BASE_ADDR) + 0x4c)
#define SSI1_SACCST ((SSI1_IO_BASE_ADDR) + 0x50)
#define SSI1_SACCEN ((SSI1_IO_BASE_ADDR) + 0x54)
#define SSI1_SACCDIS ((SSI1_IO_BASE_ADDR) + 0x58)

#define SSI2_STX0   ((SSI2_IO_BASE_ADDR) + 0x00)
#define SSI2_STX1   ((SSI2_IO_BASE_ADDR) + 0x04)
#define SSI2_SRX0   ((SSI2_IO_BASE_ADDR) + 0x08)
#define SSI2_SRX1   ((SSI2_IO_BASE_ADDR) + 0x0c)
#define SSI2_SCR    ((SSI2_IO_BASE_ADDR) + 0x10)
#define SSI2_SISR   ((SSI2_IO_BASE_ADDR) + 0x14)
#define SSI2_SIER   ((SSI2_IO_BASE_ADDR) + 0x18)
#define SSI2_STCR   ((SSI2_IO_BASE_ADDR) + 0x1c)
#define SSI2_SRCR   ((SSI2_IO_BASE_ADDR) + 0x20)
#define SSI2_STCCR  ((SSI2_IO_BASE_ADDR) + 0x24)
#define SSI2_SRCCR  ((SSI2_IO_BASE_ADDR) + 0x28)
#define SSI2_SFCSR  ((SSI2_IO_BASE_ADDR) + 0x2c)
#define SSI2_STR    ((SSI2_IO_BASE_ADDR) + 0x30)
#define SSI2_SOR    ((SSI2_IO_BASE_ADDR) + 0x34)
#define SSI2_SACNT  ((SSI2_IO_BASE_ADDR) + 0x38)
#define SSI2_SACADD ((SSI2_IO_BASE_ADDR) + 0x3c)
#define SSI2_SACDAT ((SSI2_IO_BASE_ADDR) + 0x40)
#define SSI2_SATAG  ((SSI2_IO_BASE_ADDR) + 0x44)
#define SSI2_STMSK  ((SSI2_IO_BASE_ADDR) + 0x48)
#define SSI2_SRMSK  ((SSI2_IO_BASE_ADDR) + 0x4c)
#define SSI2_SACCST ((SSI2_IO_BASE_ADDR) + 0x50)
#define SSI2_SACCEN ((SSI2_IO_BASE_ADDR) + 0x54)
#define SSI2_SACCDIS ((SSI2_IO_BASE_ADDR) + 0x58)

#define SSI_SCR_CLK_IST        (1 << 9)
#define SSI_SCR_TCH_EN         (1 << 8)
#define SSI_SCR_SYS_CLK_EN     (1 << 7)
#define SSI_SCR_I2S_MODE_NORM  (0 << 5)
#define SSI_SCR_I2S_MODE_MSTR  (1 << 5)
#define SSI_SCR_I2S_MODE_SLAVE (2 << 5)
#define SSI_SCR_SYN            (1 << 4)
#define SSI_SCR_NET            (1 << 3)
#define SSI_SCR_RE             (1 << 2)
#define SSI_SCR_TE             (1 << 1)
#define SSI_SCR_SSIEN          (1 << 0)
#define SSI_SCR_I2S_MODE_MASK  (3 << 5)

#define SSI_SISR_CMDAU         (1 << 18)
#define SSI_SISR_CMDDU         (1 << 17)
#define SSI_SISR_RXT           (1 << 16)
#define SSI_SISR_RDR1          (1 << 15)
#define SSI_SISR_RDR0          (1 << 14)
#define SSI_SISR_TDE1          (1 << 13)
#define SSI_SISR_TDE0          (1 << 12)
#define SSI_SISR_ROE1          (1 << 11)
#define SSI_SISR_ROE0          (1 << 10)
#define SSI_SISR_TUE1          (1 << 9)
#define SSI_SISR_TUE0          (1 << 8)
#define SSI_SISR_TFS           (1 << 7)
#define SSI_SISR_RFS           (1 << 6)
#define SSI_SISR_TLS           (1 << 5)
#define SSI_SISR_RLS           (1 << 4)
#define SSI_SISR_RFF1          (1 << 3)
#define SSI_SISR_RFF0          (1 << 2)
#define SSI_SISR_TFE1          (1 << 1)
#define SSI_SISR_TFE0          (1 << 0)

#define SSI_SIER_RDMAE         (1 << 22)
#define SSI_SIER_RIE           (1 << 21)
#define SSI_SIER_TDMAE         (1 << 20)
#define SSI_SIER_TIE           (1 << 19)
#define SSI_SIER_CMDAU_EN      (1 << 18)
#define SSI_SIER_CMDDU_EN      (1 << 17)
#define SSI_SIER_RXT_EN        (1 << 16)
#define SSI_SIER_RDR1_EN       (1 << 15)
#define SSI_SIER_RDR0_EN       (1 << 14)
#define SSI_SIER_TDE1_EN       (1 << 13)
#define SSI_SIER_TDE0_EN       (1 << 12)
#define SSI_SIER_ROE1_EN       (1 << 11)
#define SSI_SIER_ROE0_EN       (1 << 10)
#define SSI_SIER_TUE1_EN       (1 << 9)
#define SSI_SIER_TUE0_EN       (1 << 8)
#define SSI_SIER_TFS_EN        (1 << 7)
#define SSI_SIER_RFS_EN        (1 << 6)
#define SSI_SIER_TLS_EN        (1 << 5)
#define SSI_SIER_RLS_EN        (1 << 4)
#define SSI_SIER_RFF1_EN       (1 << 3)
#define SSI_SIER_RFF0_EN       (1 << 2)
#define SSI_SIER_TFE1_EN       (1 << 1)
#define SSI_SIER_TFE0_EN       (1 << 0)

#define SSI_STCR_TXBIT0        (1 << 9)
#define SSI_STCR_TFEN1         (1 << 8)
#define SSI_STCR_TFEN0         (1 << 7)
#define SSI_STCR_TFDIR         (1 << 6)
#define SSI_STCR_TXDIR         (1 << 5)
#define SSI_STCR_TSHFD         (1 << 4)
#define SSI_STCR_TSCKP         (1 << 3)
#define SSI_STCR_TFSI          (1 << 2)
#define SSI_STCR_TFSL          (1 << 1)
#define SSI_STCR_TEFS          (1 << 0)

#define SSI_SRCR_RXBIT0        (1 << 9)
#define SSI_SRCR_RFEN1         (1 << 8)
#define SSI_SRCR_RFEN0         (1 << 7)
#define SSI_SRCR_RFDIR         (1 << 6)
#define SSI_SRCR_RXDIR         (1 << 5)
#define SSI_SRCR_RSHFD         (1 << 4)
#define SSI_SRCR_RSCKP         (1 << 3)
#define SSI_SRCR_RFSI          (1 << 2)
#define SSI_SRCR_RFSL          (1 << 1)
#define SSI_SRCR_REFS          (1 << 0)

#define SSI_STCCR_DIV2         (1 << 18)
#define SSI_STCCR_PSR          (1 << 15)
#define SSI_STCCR_WL(x)        ((((x) - 2) >> 1) << 13)
#define SSI_STCCR_DC(x)        (((x) & 0x1f) << 8)
#define SSI_STCCR_PM(x)        (((x) & 0xff) << 0)
#define SSI_STCCR_WL_MASK        (0xf << 13)
#define SSI_STCCR_DC_MASK        (0x1f << 8)
#define SSI_STCCR_PM_MASK        (0xff << 0)

#define SSI_SRCCR_DIV2         (1 << 18)
#define SSI_SRCCR_PSR          (1 << 15)
#define SSI_SRCCR_WL(x)        ((((x) - 2) >> 1) << 13)
#define SSI_SRCCR_DC(x)        (((x) & 0x1f) << 8)
#define SSI_SRCCR_PM(x)        (((x) & 0xff) << 0)
#define SSI_SRCCR_WL_MASK        (0xf << 13)
#define SSI_SRCCR_DC_MASK        (0x1f << 8)
#define SSI_SRCCR_PM_MASK        (0xff << 0)


#define SSI_SFCSR_RFCNT1(x)   (((x) & 0xf) << 28)
#define SSI_SFCSR_TFCNT1(x)   (((x) & 0xf) << 24)
#define SSI_SFCSR_RFWM1(x)    (((x) & 0xf) << 20)
#define SSI_SFCSR_TFWM1(x)    (((x) & 0xf) << 16)
#define SSI_SFCSR_RFCNT0(x)   (((x) & 0xf) << 12)
#define SSI_SFCSR_TFCNT0(x)   (((x) & 0xf) <<  8)
#define SSI_SFCSR_RFWM0(x)    (((x) & 0xf) <<  4)
#define SSI_SFCSR_TFWM0(x)    (((x) & 0xf) <<  0)

#define SSI_STR_TEST          (1 << 15)
#define SSI_STR_RCK2TCK       (1 << 14)
#define SSI_STR_RFS2TFS       (1 << 13)
#define SSI_STR_RXSTATE(x)    (((x) & 0xf) << 8)
#define SSI_STR_TXD2RXD       (1 <<  7)
#define SSI_STR_TCK2RCK       (1 <<  6)
#define SSI_STR_TFS2RFS       (1 <<  5)
#define SSI_STR_TXSTATE(x)    (((x) & 0xf) << 0)

#define SSI_SOR_CLKOFF        (1 << 6)
#define SSI_SOR_RX_CLR        (1 << 5)
#define SSI_SOR_TX_CLR        (1 << 4)
#define SSI_SOR_INIT          (1 << 3)
#define SSI_SOR_WAIT(x)       (((x) & 0x3) << 1)
#define SSI_SOR_SYNRST        (1 << 0)

#define SSI_SACNT_FRDIV(x)    (((x) & 0x3f) << 5)
#define SSI_SACNT_WR          (1 << 4)
#define SSI_SACNT_RD          (1 << 3)
#define SSI_SACNT_TIF         (1 << 2)
#define SSI_SACNT_FV          (1 << 1)
#define SSI_SACNT_AC97EN      (1 << 0)

/* SDMA & SSI watermarks for FIFO's */
#define SDMA_TXFIFO_WATERMARK		0x4
#define SDMA_RXFIFO_WATERMARK		0x6
#define SSI_TXFIFO_WATERMARK		0x4
#define SSI_RXFIFO_WATERMARK		0x6

/* Maximum number of ssi channels (counting two channels per block) */
#define MAX_SSI_CHANNELS		12

/* i.MX DAI SSP ID's */
#define IMX_DAI_SSI0			0 /* SSI1 FIFO 0 */
#define IMX_DAI_SSI1			1 /* SSI1 FIFO 1 */
#define IMX_DAI_SSI2			2 /* SSI2 FIFO 0 */
#define IMX_DAI_SSI3			3 /* SSI2 FIFO 1 */
#define IMX_DAI_SSI4			4 /* SSI3 FIFO 0 */
#define IMX_DAI_SSI5			5 /* SSI3 FIFO 1 */

/* SSI clock sources */
#define IMX_SSP_SYS_CLK			0

/* SSI audio dividers */
#define IMX_SSI_TX_DIV_2		0
#define IMX_SSI_TX_DIV_PSR		1
#define IMX_SSI_TX_DIV_PM		2
#define IMX_SSI_RX_DIV_2		3
#define IMX_SSI_RX_DIV_PSR		4
#define IMX_SSI_RX_DIV_PM		5


/* SSI Div 2 */
#define IMX_SSI_DIV_2_OFF		(~SSI_STCCR_DIV2)
#define IMX_SSI_DIV_2_ON		SSI_STCCR_DIV2

#define IMX_DAI_AC97_1 0
#define IMX_DAI_AC97_2 1

/* private info */
struct imx_ssi {
	bool network_mode;
	bool sync_mode;
	unsigned int ac97_tx_slots;
	unsigned int ac97_rx_slots;
	void __iomem *ioaddr;
	unsigned long baseaddr;
	int irq;
	struct platform_device *pdev;
	struct clk *ssi_clk;
};

extern struct snd_soc_dai *imx_ssi_dai[];
extern struct snd_soc_dai imx_ac97_dai[];

#endif
