/*
 * flexcan.c - FLEXCAN CAN controller driver
 *
 * Copyright (c) 2005-2006 Varma Electronics Oy
 * Copyright (c) 2009 Sascha Hauer, Pengutronix
 * Copyright (c) 2010 Marc Kleine-Budde, Pengutronix
 *
 * Based on code originally by Andrey Volkov <avolkov@varma-el.com>
 *
 * LICENCE:
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/netdevice.h>
#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <linux/can/led.h>
#include <linux/can/platform/flexcan.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/regmap.h>

#define DRV_NAME			"flexcan"

/* 8 for RX fifo and 2 error handling */
#define FLEXCAN_NAPI_WEIGHT		(8 + 2)

/* FLEXCAN module configuration register (CANMCR) bits */
#define FLEXCAN_MCR_MDIS		BIT(31)
#define FLEXCAN_MCR_FRZ			BIT(30)
#define FLEXCAN_MCR_FEN			BIT(29)
#define FLEXCAN_MCR_HALT		BIT(28)
#define FLEXCAN_MCR_NOT_RDY		BIT(27)
#define FLEXCAN_MCR_WAK_MSK		BIT(26)
#define FLEXCAN_MCR_SOFTRST		BIT(25)
#define FLEXCAN_MCR_FRZ_ACK		BIT(24)
#define FLEXCAN_MCR_SUPV		BIT(23)
#define FLEXCAN_MCR_SLF_WAK		BIT(22)
#define FLEXCAN_MCR_WRN_EN		BIT(21)
#define FLEXCAN_MCR_LPM_ACK		BIT(20)
#define FLEXCAN_MCR_WAK_SRC		BIT(19)
#define FLEXCAN_MCR_DOZE		BIT(18)
#define FLEXCAN_MCR_SRX_DIS		BIT(17)
#define FLEXCAN_MCR_BCC			BIT(16)
#define FLEXCAN_MCR_LPRIO_EN		BIT(13)
#define FLEXCAN_MCR_AEN			BIT(12)
#define FLEXCAN_MCR_FDEN		BIT(11)
#define FLEXCAN_MCR_MAXMB(x)		((x) & 0x7f)
#define FLEXCAN_MCR_IDAM_A		(0x0 << 8)
#define FLEXCAN_MCR_IDAM_B		(0x1 << 8)
#define FLEXCAN_MCR_IDAM_C		(0x2 << 8)
#define FLEXCAN_MCR_IDAM_D		(0x3 << 8)

/* FLEXCAN control register (CANCTRL) bits */
#define FLEXCAN_CTRL_PRESDIV(x)		(((x) & 0xff) << 24)
#define FLEXCAN_CTRL_RJW(x)		(((x) & 0x03) << 22)
#define FLEXCAN_CTRL_PSEG1(x)		(((x) & 0x07) << 19)
#define FLEXCAN_CTRL_PSEG2(x)		(((x) & 0x07) << 16)
#define FLEXCAN_CTRL_BOFF_MSK		BIT(15)
#define FLEXCAN_CTRL_ERR_MSK		BIT(14)
#define FLEXCAN_CTRL_CLK_SRC		BIT(13)
#define FLEXCAN_CTRL_LPB		BIT(12)
#define FLEXCAN_CTRL_TWRN_MSK		BIT(11)
#define FLEXCAN_CTRL_RWRN_MSK		BIT(10)
#define FLEXCAN_CTRL_SMP		BIT(7)
#define FLEXCAN_CTRL_BOFF_REC		BIT(6)
#define FLEXCAN_CTRL_TSYN		BIT(5)
#define FLEXCAN_CTRL_LBUF		BIT(4)
#define FLEXCAN_CTRL_LOM		BIT(3)
#define FLEXCAN_CTRL_PROPSEG(x)		((x) & 0x07)
#define FLEXCAN_CTRL_ERR_BUS		(FLEXCAN_CTRL_ERR_MSK)
#define FLEXCAN_CTRL_ERR_STATE \
	(FLEXCAN_CTRL_TWRN_MSK | FLEXCAN_CTRL_RWRN_MSK | \
	 FLEXCAN_CTRL_BOFF_MSK)
#define FLEXCAN_CTRL_ERR_ALL \
	(FLEXCAN_CTRL_ERR_BUS | FLEXCAN_CTRL_ERR_STATE)

#define FLEXCAN_FDCBT_FPRESDIV(x)	(((x) & 0x3ff) << 20)
#define FLEXCAN_FDCBT_FRJW(x)		(((x) & 0x07) << 16)
#define FLEXCAN_FDCBT_FPROPSEG(x)	(((x) & 0x1f) << 10)
#define FLEXCAN_FDCBT_FPSEG1(x)		(((x) & 0x07) << 5)
#define FLEXCAN_FDCBT_FPSEG2(x)		((x) & 0x07)

/* FLEXCAN control register 2 (CTRL2) bits */
#define FLEXCAN_CTRL2_ECRWRE		BIT(29)
#define FLEXCAN_CTRL2_WRMFRZ		BIT(28)
#define FLEXCAN_CTRL2_RFFN(x)		(((x) & 0x0f) << 24)
#define FLEXCAN_CTRL2_TASD(x)		(((x) & 0x1f) << 19)
#define FLEXCAN_CTRL2_MRP		BIT(18)
#define FLEXCAN_CTRL2_RRS		BIT(17)
#define FLEXCAN_CTRL2_EACEN		BIT(16)

/* FLEXCAN memory error control register (MECR) bits */
#define FLEXCAN_MECR_ECRWRDIS		BIT(31)
#define FLEXCAN_MECR_HANCEI_MSK		BIT(19)
#define FLEXCAN_MECR_FANCEI_MSK		BIT(18)
#define FLEXCAN_MECR_CEI_MSK		BIT(16)
#define FLEXCAN_MECR_HAERRIE		BIT(15)
#define FLEXCAN_MECR_FAERRIE		BIT(14)
#define FLEXCAN_MECR_EXTERRIE		BIT(13)
#define FLEXCAN_MECR_RERRDIS		BIT(9)
#define FLEXCAN_MECR_ECCDIS		BIT(8)
#define FLEXCAN_MECR_NCEFAFRZ		BIT(7)

/* FLEXCAN error and status register (ESR) bits */
#define FLEXCAN_ESR_TWRN_INT		BIT(17)
#define FLEXCAN_ESR_RWRN_INT		BIT(16)
#define FLEXCAN_ESR_BIT1_ERR		BIT(15)
#define FLEXCAN_ESR_BIT0_ERR		BIT(14)
#define FLEXCAN_ESR_ACK_ERR		BIT(13)
#define FLEXCAN_ESR_CRC_ERR		BIT(12)
#define FLEXCAN_ESR_FRM_ERR		BIT(11)
#define FLEXCAN_ESR_STF_ERR		BIT(10)
#define FLEXCAN_ESR_TX_WRN		BIT(9)
#define FLEXCAN_ESR_RX_WRN		BIT(8)
#define FLEXCAN_ESR_IDLE		BIT(7)
#define FLEXCAN_ESR_TXRX		BIT(6)
#define FLEXCAN_EST_FLT_CONF_SHIFT	(4)
#define FLEXCAN_ESR_FLT_CONF_MASK	(0x3 << FLEXCAN_EST_FLT_CONF_SHIFT)
#define FLEXCAN_ESR_FLT_CONF_ACTIVE	(0x0 << FLEXCAN_EST_FLT_CONF_SHIFT)
#define FLEXCAN_ESR_FLT_CONF_PASSIVE	(0x1 << FLEXCAN_EST_FLT_CONF_SHIFT)
#define FLEXCAN_ESR_BOFF_INT		BIT(2)
#define FLEXCAN_ESR_ERR_INT		BIT(1)
#define FLEXCAN_ESR_WAK_INT		BIT(0)
#define FLEXCAN_ESR_ERR_BUS \
	(FLEXCAN_ESR_BIT1_ERR | FLEXCAN_ESR_BIT0_ERR | \
	 FLEXCAN_ESR_ACK_ERR | FLEXCAN_ESR_CRC_ERR | \
	 FLEXCAN_ESR_FRM_ERR | FLEXCAN_ESR_STF_ERR)
#define FLEXCAN_ESR_ERR_STATE \
	(FLEXCAN_ESR_TWRN_INT | FLEXCAN_ESR_RWRN_INT | FLEXCAN_ESR_BOFF_INT)
#define FLEXCAN_ESR_ERR_ALL \
	(FLEXCAN_ESR_ERR_BUS | FLEXCAN_ESR_ERR_STATE)
#define FLEXCAN_ESR_ALL_INT \
	(FLEXCAN_ESR_TWRN_INT | FLEXCAN_ESR_RWRN_INT | \
	 FLEXCAN_ESR_BOFF_INT | FLEXCAN_ESR_ERR_INT | \
	 FLEXCAN_ESR_WAK_INT)

#define FLEXCAN_RX_BUF_ID		0
/* FLEXCAN interrupt flag register (IFLAG) bits */
/* Errata ERR005829 step7: Reserve first valid MB */
#define FLEXCAN_TX_BUF_RESERVED		8
#define FLEXCAN_TX_BUF_ID		9
#define FLEXCAN_IFLAG_BUF(x)		BIT(x)
#define FLEXCAN_IFLAG_RX_FIFO_OVERFLOW	BIT(7)
#define FLEXCAN_IFLAG_RX_FIFO_WARN	BIT(6)
#define FLEXCAN_IFLAG_RX_FIFO_AVAILABLE	BIT(5)
/* FIFO mode using MB0 as the Message Output Buffer */
#define FLEXCAN_RX_BUF_FIFO		0

#define FLEXCAN_RX_BUF_INT \
	FLEXCAN_IFLAG_BUF(FLEXCAN_RX_BUF_ID)
#define FLEXCAN_TX_BUF_INT \
	FLEXCAN_IFLAG_BUF(FLEXCAN_TX_BUF_ID)
#define FLEXCAN_IFLAG_DEFAULT_FIFO \
	(FLEXCAN_IFLAG_RX_FIFO_OVERFLOW | FLEXCAN_IFLAG_RX_FIFO_AVAILABLE | \
	 FLEXCAN_TX_BUF_INT)
#define FLEXCAN_IFLAG_DEFAULT_MB \
	(FLEXCAN_RX_BUF_INT | FLEXCAN_TX_BUF_INT)

/* FLEXCAN message buffers */
#define FLEXCAN_MB_CNT_EDL		BIT(31)
#define FLEXCAN_MB_CNT_BRS		BIT(30)
#define FLEXCAN_MB_CNT_ESI		BIT(29)

#define FLEXCAN_MB_CODE_RX_INACTIVE	(0x0 << 24)
#define FLEXCAN_MB_CODE_RX_EMPTY	(0x4 << 24)
#define FLEXCAN_MB_CODE_RX_FULL		(0x2 << 24)
#define FLEXCAN_MB_CODE_RX_OVERRUN	(0x6 << 24)
#define FLEXCAN_MB_CODE_RX_RANSWER	(0xa << 24)
#define FLEXCAN_MB_CODE_TX_INACTIVE	(0x8 << 24)
#define FLEXCAN_MB_CODE_TX_ABORT	(0x9 << 24)
#define FLEXCAN_MB_CODE_TX_DATA		(0xc << 24)
#define FLEXCAN_MB_CODE_TX_TANSWER	(0xe << 24)

#define FLEXCAN_MB_CNT_SRR		BIT(22)
#define FLEXCAN_MB_CNT_IDE		BIT(21)
#define FLEXCAN_MB_CNT_RTR		BIT(20)
#define FLEXCAN_MB_CNT_LENGTH(x)	(((x) & 0xf) << 16)
#define FLEXCAN_MB_CNT_TIMESTAMP(x)	((x) & 0xffff)

#define FLEXCAN_TIMEOUT_US		(50)

#define FLEXCAN_FDCTRL_FDRATE		BIT(31)

/* FLEXCAN hardware feature flags
 *
 * Below is some version info we got:
 *    SOC   Version   IP-Version  Glitch- [TR]WRN_INT Memory err RTR re-
 *                                Filter? connected?  detection  ception in MB
 *   MX25  FlexCAN2  03.00.00.00     no        no         no        no
 *   MX28  FlexCAN2  03.00.04.00    yes       yes         no        no
 *   MX35  FlexCAN2  03.00.00.00     no        no         no        no
 *   MX53  FlexCAN2  03.00.00.00    yes        no         no        no
 *   MX6s  FlexCAN3  10.00.12.00    yes       yes         no       yes
 *   VF610 FlexCAN3  ?               no       yes        yes       yes?
 *
 * Some SOCs do not have the RX_WARN & TX_WARN interrupt line connected.
 */
#define FLEXCAN_QUIRK_BROKEN_ERR_STATE	BIT(1) /* [TR]WRN_INT not connected */
#define FLEXCAN_QUIRK_DISABLE_RXFG	BIT(2) /* Disable RX FIFO Global mask */
#define FLEXCAN_QUIRK_DISABLE_MECR	BIT(3) /* Disble Memory error detection */
#define FLEXCAN_QUIRK_DISABLE_RX_FIFO	BIT(4) /* Disable RX FIFO mode */
#define FLEXCAN_QUIRK_SUPPORT_CANFD	BIT(5) /* Support CAN FD mode */


/* Message Buffer */
#define FLEXCAN_MB_CTRL		0x0
#define FLEXCAN_MB_ID		0x4
#define FLEXCAN_MB_DATA(n)	(0x8 + ((n) << 2))

#define FLEXCAN_MB_NUM		64
#define FLEXCAN_MB_FD_NUM	14
#define FLEXCAN_MB_SIZE		16
#define FLEXCAN_MB_FD_SIZE	72

/* CAN FD Memory Partition
 *
 * When CAN FD is enabled, the FlexCAN RAM can be partitioned in
 * blocks of 512 bytes. Each block can accommodate a number of
 * Message Buffers which depends on the configuration provided
 * by CAN_FDCTRL[MBDSRn] bit fields where we all set to 64 bytes
 * per Message Buffer and 7 MBs per Block by default.
 *
 * There're two RAM blocks: RAM block 0,1
 */
#define FLEXCAN_CANFD_MB_OFFSET(n)	(((n) / 7) * 512 + ((n) % 7) * \
					 FLEXCAN_MB_FD_SIZE)
#define FLEXCAN_CANFD_MBDSR_MASK	0x6db0000
#define FLEXCAN_CANFD_MBDSR_SHIFT	16
#define FLEXCAN_CANFD_MBDSR_DEFAULT	0x6db

/* registers definition
 *
 * FIFO-MODE:
 *			MB
 * 0X080...0X08F	0	RX MESSAGE BUFFER
 * 0X090...0X0DF	1-5	RESERVERD
 * 0X0E0...0X0FF	6-7	8 ENTRY ID TABLE
 *				(MX25, MX28, MX35, MX53)
 * 0X0E0...0X2DF	6-7..37	8..128 ENTRY ID TABLE
 *				SIZE CONF'ED VIA CTRL2::RFFN
 *				(MX6, VF610)
 */

enum flexcan_reg {
	FLEXCAN_MCR		= 0x00,
	FLEXCAN_CTRL		= 0x04,
	FLEXCAN_TIMER		= 0x08,
	FLEXCAN_RXGMASK		= 0x10,
	FLEXCAN_RX14MASK	= 0x14,
	FLEXCAN_RX15MASK	= 0x18,
	FLEXCAN_ECR		= 0x1c,
	FLEXCAN_ESR		= 0x20,
	FLEXCAN_IMASK2		= 0x24,
	FLEXCAN_IMASK1		= 0x28,
	FLEXCAN_IFLAG2		= 0x2c,
	FLEXCAN_IFLAG1		= 0x30,
	FLEXCAN_CTRL2		= 0x34,
	FLEXCAN_ESR2		= 0x38,
	FLEXCAN_IMEUR		= 0x3c,
	FLEXCAN_LRFR		= 0x40,
	FLEXCAN_CRCR		= 0x44,
	FLEXCAN_RXFGMASK	= 0x48,
	FLEXCAN_RXFIR		= 0x4c,
	FLEXCAN_MB		= 0x80,
	FLEXCAN_MECR		= 0xae0,
	FLEXCAN_ERRIAR		= 0xae4,
	FLEXCAN_ERRIDPR		= 0xae8,
	FLEXCAN_ERRIPPR		= 0xaeC,
	FLEXCAN_RERRAR		= 0xaf0,
	FLEXCAN_RERRDR		= 0xaf4,
	FLEXCAN_RERRSYNR	= 0xaf8,
	FLEXCAN_ERRSR		= 0xafC,
	FLEXCAN_FDCTRL		= 0xc00,
	FLEXCAN_FDCBT		= 0xc04,
	FLEXCAN_FDCRC		= 0xc08,
};

struct flexcan_devtype_data {
	u32 quirks;		/* quirks needed for different IP cores */
};

struct flexcan_stop_mode {
	struct regmap *gpr;
	u8 req_gpr;
	u8 req_bit;
	u8 ack_gpr;
	u8 ack_bit;
};
struct flexcan_priv {
	struct can_priv can;
	struct napi_struct napi;

	void __iomem *base;
	u32 reg_esr;
	u32 reg_ctrl_default;

	struct clk *clk_ipg;
	struct clk *clk_per;
	struct flexcan_platform_data *pdata;
	const struct flexcan_devtype_data *devtype_data;
	struct regulator *reg_xceiver;
	int id;
	struct flexcan_stop_mode stm;

	bool mb_mode;
	u32 iflag_default;
	/* Rx interrupt can be either Rx fifo or Rx buffer interrupt */
	u32 rx_int;

	u32 mb_size;
	u32 mb_num;
};

static struct flexcan_devtype_data fsl_p1010_devtype_data = {
	.quirks = FLEXCAN_QUIRK_BROKEN_ERR_STATE,
};

static struct flexcan_devtype_data fsl_imx28_devtype_data;

static struct flexcan_devtype_data fsl_imx6q_devtype_data = {
	.quirks = FLEXCAN_QUIRK_DISABLE_RXFG,
};

static struct flexcan_devtype_data fsl_imx8qm_devtype_data = {
	.quirks = FLEXCAN_QUIRK_DISABLE_RXFG | FLEXCAN_QUIRK_DISABLE_RX_FIFO |
		  FLEXCAN_QUIRK_SUPPORT_CANFD,
};

static struct flexcan_devtype_data fsl_vf610_devtype_data = {
	.quirks = FLEXCAN_QUIRK_DISABLE_RXFG | FLEXCAN_QUIRK_DISABLE_MECR,
};

static const struct can_bittiming_const flexcan_bittiming_const = {
	.name = DRV_NAME,
	.tseg1_min = 4,
	.tseg1_max = 16,
	.tseg2_min = 2,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 256,
	.brp_inc = 1,
};

static const struct can_bittiming_const flexcan_data_bittiming_const = {
	.name = DRV_NAME,
	.tseg1_min = 1,
	.tseg1_max = 39,
	.tseg2_min = 1,
	.tseg2_max = 8,
	.sjw_max = 8,
	.brp_min = 1,
	.brp_max = 1024,
	.brp_inc = 1,
};

/* Abstract off the read/write for arm versus ppc. This
 * assumes that PPC uses big-endian registers and everything
 * else uses little-endian registers, independent of CPU
 * endianness.
 */
#if defined(CONFIG_PPC)
static inline u32 flexcan_read(const struct flexcan_priv *priv,
			       enum flexcan_reg reg)
{
	return in_be32(priv->base + reg);
}

static inline void flexcan_write(const struct flexcan_priv *priv,
				 enum flexcan_reg reg, u32 val)
{
	out_be32(priv->base + reg, val);
}

static inline u32 flexcan_mb_read(const struct flexcan_priv *priv,
				  u32 index, unsigned int offset)
{
	if (priv->can.ctrlmode & CAN_CTRLMODE_FD)
		return in_be32(priv->base + FLEXCAN_MB +
			     FLEXCAN_CANFD_MB_OFFSET(index) + offset);
	else
		return in_be32(priv->base + FLEXCAN_MB +
			       priv->mb_size * index + offset);
}

static inline void flexcan_mb_write(const struct flexcan_priv *priv,
				    u32 index, unsigned int offset,
				    u32 val)
{
	if (priv->can.ctrlmode & CAN_CTRLMODE_FD)
		out_be32(val, priv->base + FLEXCAN_MB +
			 FLEXCAN_CANFD_MB_OFFSET(index) + offset);
	else
		out_be32(val, priv->base + FLEXCAN_MB +
			 priv->mb_size * index + offset);
}
#else
static inline u32 flexcan_read(const struct flexcan_priv *priv,
			       enum flexcan_reg reg)
{
	return readl(priv->base + reg);
}

static inline void flexcan_write(const struct flexcan_priv *priv,
				 enum flexcan_reg reg, u32 val)
{
	writel(val, priv->base + reg);
}

static inline u32 flexcan_mb_read(const struct flexcan_priv *priv,
				  u32 index, unsigned int offset)
{
	if (priv->can.ctrlmode & CAN_CTRLMODE_FD)
		return readl(priv->base + FLEXCAN_MB +
			     FLEXCAN_CANFD_MB_OFFSET(index) + offset);
	else
		return readl(priv->base + FLEXCAN_MB +
			     priv->mb_size * index + offset);
}

static inline void flexcan_mb_write(const struct flexcan_priv *priv,
				    u32 index, unsigned int offset,
				    u32 val)
{
	if (priv->can.ctrlmode & CAN_CTRLMODE_FD)
		writel(val, priv->base + FLEXCAN_MB +
		       FLEXCAN_CANFD_MB_OFFSET(index) + offset);
	else
		writel(val, priv->base + FLEXCAN_MB +
		       priv->mb_size * index + offset);
}
#endif

static inline void flexcan_enter_stop_mode(struct flexcan_priv *priv)
{
	/* enable stop request */
	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_DISABLE_RXFG)
		regmap_update_bits(priv->stm.gpr, priv->stm.req_gpr,
			1 << priv->stm.req_bit, 1 << priv->stm.req_bit);
}

static inline void flexcan_exit_stop_mode(struct flexcan_priv *priv)
{
	/* remove stop request */
	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_DISABLE_RXFG)
		regmap_update_bits(priv->stm.gpr, priv->stm.req_gpr,
			1 << priv->stm.req_bit, 0);
}

static inline int flexcan_transceiver_enable(const struct flexcan_priv *priv)
{
	if (priv->pdata && priv->pdata->transceiver_switch) {
		priv->pdata->transceiver_switch(1);
		return 0;
	}

	if (!priv->reg_xceiver)
		return 0;

	return regulator_enable(priv->reg_xceiver);
}

static inline int flexcan_transceiver_disable(const struct flexcan_priv *priv)
{
	if (priv->pdata && priv->pdata->transceiver_switch) {
		priv->pdata->transceiver_switch(0);
		return 0;
	}

	if (!priv->reg_xceiver)
		return 0;

	return regulator_disable(priv->reg_xceiver);
}

static inline int flexcan_has_and_handle_berr(const struct flexcan_priv *priv,
					      u32 reg_esr)
{
	return (priv->can.ctrlmode & CAN_CTRLMODE_BERR_REPORTING) &&
		(reg_esr & FLEXCAN_ESR_ERR_BUS);
}

static int flexcan_chip_enable(struct flexcan_priv *priv)
{
	unsigned int timeout = FLEXCAN_TIMEOUT_US / 10;
	u32 reg;

	reg = flexcan_read(priv, FLEXCAN_MCR);
	reg &= ~FLEXCAN_MCR_MDIS;
	flexcan_write(priv, FLEXCAN_MCR, reg);

	while (timeout-- &&
	       (flexcan_read(priv, FLEXCAN_MCR) & FLEXCAN_MCR_LPM_ACK))
		udelay(10);

	if (flexcan_read(priv, FLEXCAN_MCR) & FLEXCAN_MCR_LPM_ACK)
		return -ETIMEDOUT;

	return 0;
}

static int flexcan_chip_disable(struct flexcan_priv *priv)
{
	unsigned int timeout = FLEXCAN_TIMEOUT_US / 10;
	u32 reg;

	reg = flexcan_read(priv, FLEXCAN_MCR);
	reg |= FLEXCAN_MCR_MDIS;
	flexcan_write(priv, FLEXCAN_MCR, reg);

	while (timeout-- &&
	       !(flexcan_read(priv, FLEXCAN_MCR) & FLEXCAN_MCR_LPM_ACK))
		udelay(10);

	if (!(flexcan_read(priv, FLEXCAN_MCR) & FLEXCAN_MCR_LPM_ACK))
		return -ETIMEDOUT;

	return 0;
}

static int flexcan_chip_freeze(struct flexcan_priv *priv)
{
	unsigned int timeout = 1000 * 1000 * 10 / priv->can.bittiming.bitrate;
	u32 reg;

	reg = flexcan_read(priv, FLEXCAN_MCR);
	reg |= FLEXCAN_MCR_HALT;
	flexcan_write(priv, FLEXCAN_MCR, reg);

	while (timeout-- &&
	       !(flexcan_read(priv, FLEXCAN_MCR) & FLEXCAN_MCR_FRZ_ACK))
		udelay(100);

	if (!(flexcan_read(priv, FLEXCAN_MCR) & FLEXCAN_MCR_FRZ_ACK))
		return -ETIMEDOUT;

	return 0;
}

static int flexcan_chip_unfreeze(struct flexcan_priv *priv)
{
	unsigned int timeout = FLEXCAN_TIMEOUT_US / 10;
	u32 reg;

	reg = flexcan_read(priv, FLEXCAN_MCR);
	reg &= ~FLEXCAN_MCR_HALT;
	flexcan_write(priv, FLEXCAN_MCR, reg);

	while (timeout-- &&
	       (flexcan_read(priv, FLEXCAN_MCR) & FLEXCAN_MCR_FRZ_ACK))
		udelay(20);

	if (flexcan_read(priv, FLEXCAN_MCR) & FLEXCAN_MCR_FRZ_ACK)
		return -ETIMEDOUT;

	return 0;
}

static int flexcan_chip_softreset(struct flexcan_priv *priv)
{
	unsigned int timeout = FLEXCAN_TIMEOUT_US / 10;

	flexcan_write(priv, FLEXCAN_MCR, FLEXCAN_MCR_SOFTRST);
	while (timeout-- &&
	       (flexcan_read(priv, FLEXCAN_MCR) & FLEXCAN_MCR_SOFTRST))
		udelay(10);

	if (flexcan_read(priv, FLEXCAN_MCR) & FLEXCAN_MCR_SOFTRST)
		return -ETIMEDOUT;

	return 0;
}

static int __flexcan_get_berr_counter(const struct net_device *dev,
				      struct can_berr_counter *bec)
{
	const struct flexcan_priv *priv = netdev_priv(dev);
	u32 reg = flexcan_read(priv, FLEXCAN_ECR);

	bec->txerr = (reg >> 0) & 0xff;
	bec->rxerr = (reg >> 8) & 0xff;

	return 0;
}

static int flexcan_get_berr_counter(const struct net_device *dev,
				    struct can_berr_counter *bec)
{
	const struct flexcan_priv *priv = netdev_priv(dev);
	int err;

	err = clk_prepare_enable(priv->clk_ipg);
	if (err)
		return err;

	err = clk_prepare_enable(priv->clk_per);
	if (err)
		goto out_disable_ipg;

	err = __flexcan_get_berr_counter(dev, bec);

	clk_disable_unprepare(priv->clk_per);
 out_disable_ipg:
	clk_disable_unprepare(priv->clk_ipg);

	return err;
}

static int flexcan_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	const struct flexcan_priv *priv = netdev_priv(dev);
	struct canfd_frame *cf = (struct canfd_frame *)skb->data;
	u32 ctrl = FLEXCAN_MB_CODE_TX_DATA | (can_len2dlc(cf->len) << 16);
	u32 can_id, reg_fdctrl;
	u32 data;
	int i;

	if (can_dropped_invalid_skb(dev, skb))
		return NETDEV_TX_OK;

	netif_stop_queue(dev);

	if (cf->can_id & CAN_EFF_FLAG) {
		can_id = cf->can_id & CAN_EFF_MASK;
		ctrl |= FLEXCAN_MB_CNT_IDE | FLEXCAN_MB_CNT_SRR;
	} else {
		can_id = (cf->can_id & CAN_SFF_MASK) << 18;
	}

	if (cf->can_id & CAN_RTR_FLAG)
		ctrl |= FLEXCAN_MB_CNT_RTR;

	for (i = 0; i < cf->len; i += 4) {
		data = be32_to_cpup((__be32 *)&cf->data[i]);
		flexcan_mb_write(priv, FLEXCAN_TX_BUF_ID,
				 FLEXCAN_MB_DATA(i / 4), data);
	}

	can_put_echo_skb(skb, dev, 0);

	if (priv->can.ctrlmode & CAN_CTRLMODE_FD) {
		if (can_is_canfd_skb(skb)) {
			reg_fdctrl = flexcan_read(priv, FLEXCAN_FDCTRL) &
						  ~FLEXCAN_FDCTRL_FDRATE;
			if (cf->flags & CANFD_BRS) {
				reg_fdctrl |= FLEXCAN_FDCTRL_FDRATE;
				ctrl |= FLEXCAN_MB_CNT_BRS;
			}
			flexcan_write(priv, FLEXCAN_FDCTRL, reg_fdctrl);
			ctrl |= FLEXCAN_MB_CNT_EDL;
		}
	}

	flexcan_mb_write(priv, FLEXCAN_TX_BUF_ID, FLEXCAN_MB_ID, can_id);
	flexcan_mb_write(priv, FLEXCAN_TX_BUF_ID, FLEXCAN_MB_CTRL, ctrl);

	/* Errata ERR005829 step8:
	 * Write twice INACTIVE(0x8) code to first MB.
	 */
	flexcan_mb_write(priv, FLEXCAN_TX_BUF_RESERVED, FLEXCAN_MB_CTRL,
			 FLEXCAN_MB_CODE_TX_INACTIVE);

	flexcan_mb_write(priv, FLEXCAN_TX_BUF_RESERVED, FLEXCAN_MB_CTRL,
			 FLEXCAN_MB_CODE_TX_INACTIVE);

	return NETDEV_TX_OK;
}

static void do_bus_err(struct net_device *dev,
		       struct can_frame *cf, u32 reg_esr)
{
	struct flexcan_priv *priv = netdev_priv(dev);
	int rx_errors = 0, tx_errors = 0;

	cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;

	if (reg_esr & FLEXCAN_ESR_BIT1_ERR) {
		netdev_dbg(dev, "BIT1_ERR irq\n");
		cf->data[2] |= CAN_ERR_PROT_BIT1;
		tx_errors = 1;
	}
	if (reg_esr & FLEXCAN_ESR_BIT0_ERR) {
		netdev_dbg(dev, "BIT0_ERR irq\n");
		cf->data[2] |= CAN_ERR_PROT_BIT0;
		tx_errors = 1;
	}
	if (reg_esr & FLEXCAN_ESR_ACK_ERR) {
		netdev_dbg(dev, "ACK_ERR irq\n");
		cf->can_id |= CAN_ERR_ACK;
		cf->data[3] = CAN_ERR_PROT_LOC_ACK;
		tx_errors = 1;
	}
	if (reg_esr & FLEXCAN_ESR_CRC_ERR) {
		netdev_dbg(dev, "CRC_ERR irq\n");
		cf->data[2] |= CAN_ERR_PROT_BIT;
		cf->data[3] = CAN_ERR_PROT_LOC_CRC_SEQ;
		rx_errors = 1;
	}
	if (reg_esr & FLEXCAN_ESR_FRM_ERR) {
		netdev_dbg(dev, "FRM_ERR irq\n");
		cf->data[2] |= CAN_ERR_PROT_FORM;
		rx_errors = 1;
	}
	if (reg_esr & FLEXCAN_ESR_STF_ERR) {
		netdev_dbg(dev, "STF_ERR irq\n");
		cf->data[2] |= CAN_ERR_PROT_STUFF;
		rx_errors = 1;
	}

	priv->can.can_stats.bus_error++;
	if (rx_errors)
		dev->stats.rx_errors++;
	if (tx_errors)
		dev->stats.tx_errors++;
}

static int flexcan_poll_bus_err(struct net_device *dev, u32 reg_esr)
{
	struct sk_buff *skb;
	struct can_frame *cf;

	skb = alloc_can_err_skb(dev, &cf);
	if (unlikely(!skb))
		return 0;

	do_bus_err(dev, cf, reg_esr);

	dev->stats.rx_packets++;
	dev->stats.rx_bytes += cf->can_dlc;
	netif_receive_skb(skb);

	return 1;
}

static int flexcan_poll_state(struct net_device *dev, u32 reg_esr)
{
	struct flexcan_priv *priv = netdev_priv(dev);
	struct sk_buff *skb;
	struct can_frame *cf;
	enum can_state new_state = 0, rx_state = 0, tx_state = 0;
	int flt;
	struct can_berr_counter bec;

	flt = reg_esr & FLEXCAN_ESR_FLT_CONF_MASK;
	if (likely(flt == FLEXCAN_ESR_FLT_CONF_ACTIVE)) {
		tx_state = unlikely(reg_esr & FLEXCAN_ESR_TX_WRN) ?
			CAN_STATE_ERROR_WARNING : CAN_STATE_ERROR_ACTIVE;
		rx_state = unlikely(reg_esr & FLEXCAN_ESR_RX_WRN) ?
			CAN_STATE_ERROR_WARNING : CAN_STATE_ERROR_ACTIVE;
		new_state = max(tx_state, rx_state);
	} else {
		__flexcan_get_berr_counter(dev, &bec);
		new_state = flt == FLEXCAN_ESR_FLT_CONF_PASSIVE ?
			CAN_STATE_ERROR_PASSIVE : CAN_STATE_BUS_OFF;
		rx_state = bec.rxerr >= bec.txerr ? new_state : 0;
		tx_state = bec.rxerr <= bec.txerr ? new_state : 0;
	}

	/* state hasn't changed */
	if (likely(new_state == priv->can.state))
		return 0;

	skb = alloc_can_err_skb(dev, &cf);
	if (unlikely(!skb))
		return 0;

	can_change_state(dev, cf, tx_state, rx_state);

	if (unlikely(new_state == CAN_STATE_BUS_OFF))
		can_bus_off(dev);

	dev->stats.rx_packets++;
	dev->stats.rx_bytes += cf->can_dlc;
	netif_receive_skb(skb);

	return 1;
}

static void flexcan_read_mb(const struct net_device *dev,
			    struct canfd_frame *cf)
{
	const struct flexcan_priv *priv = netdev_priv(dev);
	u32 reg_ctrl, reg_id;
	u32 index;
	int i;

	index = priv->mb_mode ? FLEXCAN_RX_BUF_ID : FLEXCAN_RX_BUF_FIFO;
	reg_ctrl = flexcan_mb_read(priv, index, FLEXCAN_MB_CTRL);
	reg_id = flexcan_mb_read(priv, index, FLEXCAN_MB_ID);
	if (reg_ctrl & FLEXCAN_MB_CNT_IDE)
		cf->can_id = ((reg_id >> 0) & CAN_EFF_MASK) | CAN_EFF_FLAG;
	else
		cf->can_id = (reg_id >> 18) & CAN_SFF_MASK;

	if (reg_ctrl & FLEXCAN_MB_CNT_EDL)
		cf->len = can_dlc2len((reg_ctrl >> 16) & 0x0F);
	else
		cf->len = get_can_dlc((reg_ctrl >> 16) & 0x0F);

	if (reg_ctrl & FLEXCAN_MB_CNT_ESI) {
		cf->flags |= CANFD_ESI;
		netdev_warn(dev, "ESI Error\n");
	}

	if (!(reg_ctrl & FLEXCAN_MB_CNT_EDL) && reg_ctrl & FLEXCAN_MB_CNT_RTR) {
		cf->can_id |= CAN_RTR_FLAG;
	} else {
		if (reg_ctrl & FLEXCAN_MB_CNT_BRS)
			cf->flags |= CANFD_BRS;

		for (i = 0; i < cf->len; i += 4)
			*(__be32 *)(cf->data + i) =
				cpu_to_be32(flexcan_mb_read(priv, index,
					    FLEXCAN_MB_DATA(i / 4)));
	}

	/* mark as read */
	flexcan_write(priv, FLEXCAN_IFLAG1, priv->rx_int);
	flexcan_read(priv, FLEXCAN_TIMER);
}

static int flexcan_read_frame(struct net_device *dev)
{
	struct net_device_stats *stats = &dev->stats;
	const struct flexcan_priv *priv = netdev_priv(dev);
	struct canfd_frame *cf;
	struct sk_buff *skb;
	u32 reg_ctrl;
	u32 index;

	index = priv->mb_mode ? FLEXCAN_RX_BUF_ID : FLEXCAN_RX_BUF_FIFO;
	reg_ctrl = flexcan_mb_read(priv, index, FLEXCAN_MB_CTRL);

	if (reg_ctrl & FLEXCAN_MB_CNT_EDL)
		skb = alloc_canfd_skb(dev, &cf);
	else
		skb = alloc_can_skb(dev, (struct can_frame **)&cf);

	if (unlikely(!skb)) {
		stats->rx_dropped++;
		return 0;
	}

	flexcan_read_mb(dev, cf);

	stats->rx_packets++;
	stats->rx_bytes += cf->len;
	netif_receive_skb(skb);

	can_led_event(dev, CAN_LED_EVENT_RX);

	return 1;
}

static int flexcan_poll(struct napi_struct *napi, int quota)
{
	struct net_device *dev = napi->dev;
	const struct flexcan_priv *priv = netdev_priv(dev);
	u32 reg_iflag1, reg_esr;
	int work_done = 0;

	/* The error bits are cleared on read,
	 * use saved value from irq handler.
	 */
	reg_esr = flexcan_read(priv, FLEXCAN_ESR) | priv->reg_esr;

	/* handle state changes */
	work_done += flexcan_poll_state(dev, reg_esr);

	/* handle RX MB */
	reg_iflag1 = flexcan_read(priv, FLEXCAN_IFLAG1);
	while (reg_iflag1 & priv->rx_int && work_done < quota) {
		work_done += flexcan_read_frame(dev);
		reg_iflag1 = flexcan_read(priv, FLEXCAN_IFLAG1);
	}

	/* report bus errors */
	if (flexcan_has_and_handle_berr(priv, reg_esr) && work_done < quota)
		work_done += flexcan_poll_bus_err(dev, reg_esr);

	if (work_done < quota) {
		napi_complete(napi);
		/* enable IRQs */
		flexcan_write(priv, FLEXCAN_IMASK1, priv->iflag_default);
		flexcan_write(priv, FLEXCAN_CTRL, priv->reg_ctrl_default);
	}

	return work_done;
}

static irqreturn_t flexcan_irq(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct net_device_stats *stats = &dev->stats;
	struct flexcan_priv *priv = netdev_priv(dev);
	u32 reg_iflag1, reg_esr;

	reg_iflag1 = flexcan_read(priv, FLEXCAN_IFLAG1);
	reg_esr = flexcan_read(priv, FLEXCAN_ESR);

	/* ACK all bus error and state change IRQ sources */
	if (reg_esr & FLEXCAN_ESR_ALL_INT)
		flexcan_write(priv, FLEXCAN_ESR, reg_esr & FLEXCAN_ESR_ALL_INT);

	if (reg_esr & FLEXCAN_ESR_WAK_INT)
		flexcan_exit_stop_mode(priv);

	/* schedule NAPI in case of:
	 * - rx IRQ
	 * - state change IRQ
	 * - bus error IRQ and bus error reporting is activated
	 */
	if ((reg_iflag1 & priv->rx_int) ||
	    (reg_esr & FLEXCAN_ESR_ERR_STATE) ||
	    flexcan_has_and_handle_berr(priv, reg_esr)) {
		/* The error bits are cleared on read,
		 * save them for later use.
		 */
		priv->reg_esr = reg_esr & FLEXCAN_ESR_ERR_BUS;
		flexcan_write(priv, FLEXCAN_IMASK1, priv->iflag_default &
			      ~priv->rx_int);
		flexcan_write(priv, FLEXCAN_CTRL, priv->reg_ctrl_default &
			      ~FLEXCAN_CTRL_ERR_ALL);
		napi_schedule(&priv->napi);
	}

	/* FIFO overflow */
	if (reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_OVERFLOW) {
		flexcan_write(priv, FLEXCAN_IFLAG1,
			      FLEXCAN_IFLAG_RX_FIFO_OVERFLOW);
		dev->stats.rx_over_errors++;
		dev->stats.rx_errors++;
	}

	/* transmission complete interrupt */
	if (reg_iflag1 & (1 << FLEXCAN_TX_BUF_ID)) {
		stats->tx_bytes += can_get_echo_skb(dev, 0);
		stats->tx_packets++;
		can_led_event(dev, CAN_LED_EVENT_TX);

		/* after sending a RTR frame MB is in RX mode */
		flexcan_mb_write(priv, FLEXCAN_TX_BUF_ID, FLEXCAN_MB_CTRL,
				 FLEXCAN_MB_CODE_TX_INACTIVE);
		flexcan_write(priv, FLEXCAN_IFLAG1, 1 << FLEXCAN_TX_BUF_ID);
		netif_wake_queue(dev);
	}

	return IRQ_HANDLED;
}

static void flexcan_set_bittiming(struct net_device *dev)
{
	const struct flexcan_priv *priv = netdev_priv(dev);
	const struct can_bittiming *bt = &priv->can.bittiming;
	const struct can_bittiming *dbt = &priv->can.data_bittiming;
	u32 reg;

	reg = flexcan_read(priv, FLEXCAN_CTRL);
	reg &= ~(FLEXCAN_CTRL_PRESDIV(0xff) |
		 FLEXCAN_CTRL_RJW(0x3) |
		 FLEXCAN_CTRL_PSEG1(0x7) |
		 FLEXCAN_CTRL_PSEG2(0x7) |
		 FLEXCAN_CTRL_PROPSEG(0x7) |
		 FLEXCAN_CTRL_LPB |
		 FLEXCAN_CTRL_SMP |
		 FLEXCAN_CTRL_LOM);

	reg |= FLEXCAN_CTRL_PRESDIV(bt->brp - 1) |
		FLEXCAN_CTRL_PSEG1(bt->phase_seg1 - 1) |
		FLEXCAN_CTRL_PSEG2(bt->phase_seg2 - 1) |
		FLEXCAN_CTRL_RJW(bt->sjw - 1) |
		FLEXCAN_CTRL_PROPSEG(bt->prop_seg - 1);

	if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK)
		reg |= FLEXCAN_CTRL_LPB;
	if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY)
		reg |= FLEXCAN_CTRL_LOM;
	if (priv->can.ctrlmode & CAN_CTRLMODE_3_SAMPLES)
		reg |= FLEXCAN_CTRL_SMP;

	netdev_dbg(dev, "writing ctrl=0x%08x\n", reg);
	flexcan_write(priv, FLEXCAN_CTRL, reg);

	if (priv->can.ctrlmode & CAN_CTRLMODE_FD) {
		reg = FLEXCAN_FDCBT_FPRESDIV(dbt->brp - 1) |
			FLEXCAN_FDCBT_FPSEG1(dbt->phase_seg1 - 1) |
			FLEXCAN_FDCBT_FPSEG2(dbt->phase_seg2 - 1) |
			FLEXCAN_FDCBT_FRJW(dbt->sjw - 1) |
			FLEXCAN_FDCBT_FPROPSEG(dbt->prop_seg);
		flexcan_write(priv, FLEXCAN_FDCBT, reg);
	}

	/* print chip status */
	netdev_dbg(dev, "%s: mcr=0x%08x ctrl=0x%08x\n", __func__,
		   flexcan_read(priv, FLEXCAN_MCR),
		   flexcan_read(priv, FLEXCAN_CTRL));
}

/* flexcan_chip_start
 *
 * this functions is entered with clocks enabled
 *
 */
static int flexcan_chip_start(struct net_device *dev)
{
	struct flexcan_priv *priv = netdev_priv(dev);
	u32 reg_mcr, reg_ctrl, reg_ctrl2, reg_mecr, reg_fdctrl;
	int err, i;

	/* enable module */
	err = flexcan_chip_enable(priv);
	if (err)
		return err;

	/* soft reset */
	err = flexcan_chip_softreset(priv);
	if (err)
		goto out_chip_disable;

	flexcan_set_bittiming(dev);

	/* MCR
	 *
	 * enable freeze
	 * enable fifo
	 * halt now
	 * only supervisor access
	 * enable warning int
	 * disable local echo
	 * choose format C
	 * set max mailbox number
	 * enable self wakeup
	 */
	reg_mcr = flexcan_read(priv, FLEXCAN_MCR);
	reg_mcr &= ~FLEXCAN_MCR_MAXMB(0xff);
	reg_mcr |= FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT |
		FLEXCAN_MCR_SUPV | FLEXCAN_MCR_WRN_EN |
		FLEXCAN_MCR_IDAM_C | FLEXCAN_MCR_SRX_DIS |
		FLEXCAN_MCR_WAK_MSK | FLEXCAN_MCR_SLF_WAK |
		FLEXCAN_MCR_MAXMB(FLEXCAN_TX_BUF_ID);

	if (!priv->mb_mode)
		reg_mcr |= FLEXCAN_MCR_FEN;

	netdev_dbg(dev, "%s: writing mcr=0x%08x", __func__, reg_mcr);
	flexcan_write(priv, FLEXCAN_MCR, reg_mcr);

	/* CTRL
	 *
	 * disable timer sync feature
	 *
	 * disable auto busoff recovery
	 * transmit lowest buffer first
	 *
	 * enable tx and rx warning interrupt
	 * enable bus off interrupt
	 * (== FLEXCAN_CTRL_ERR_STATE)
	 */
	reg_ctrl = flexcan_read(priv, FLEXCAN_CTRL);
	reg_ctrl &= ~FLEXCAN_CTRL_TSYN;
	reg_ctrl |= FLEXCAN_CTRL_BOFF_REC | FLEXCAN_CTRL_LBUF |
		FLEXCAN_CTRL_ERR_STATE;

	/* enable the "error interrupt" (FLEXCAN_CTRL_ERR_MSK),
	 * on most Flexcan cores, too. Otherwise we don't get
	 * any error warning or passive interrupts.
	 */
	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_BROKEN_ERR_STATE ||
	    priv->can.ctrlmode & CAN_CTRLMODE_BERR_REPORTING)
		reg_ctrl |= FLEXCAN_CTRL_ERR_MSK;
	else
		reg_ctrl &= ~FLEXCAN_CTRL_ERR_MSK;

	/* save for later use */
	priv->reg_ctrl_default = reg_ctrl;
	/* leave interrupts disabled for now */
	reg_ctrl &= ~FLEXCAN_CTRL_ERR_ALL;
	netdev_dbg(dev, "%s: writing ctrl=0x%08x", __func__, reg_ctrl);
	flexcan_write(priv, FLEXCAN_CTRL, reg_ctrl);

	/* CAN FD initialization
	 *
	 * disable BRS by default
	 * Message Buffer Data Size 64 bytes per MB
	 * disable Transceiver Delay Compensation
	 * Configure Message Buffer according to CAN FD mode enabled or not
	 */
	if (priv->can.ctrlmode & CAN_CTRLMODE_FD) {
		reg_fdctrl = flexcan_read(priv, FLEXCAN_FDCTRL) &
			     ~FLEXCAN_CANFD_MBDSR_MASK;
		reg_fdctrl |= FLEXCAN_CANFD_MBDSR_DEFAULT <<
			      FLEXCAN_CANFD_MBDSR_SHIFT;
		flexcan_write(priv, FLEXCAN_FDCTRL, reg_fdctrl);
		reg_mcr = flexcan_read(priv, FLEXCAN_MCR);
		flexcan_write(priv, FLEXCAN_MCR, reg_mcr | FLEXCAN_MCR_FDEN);

		priv->mb_size = FLEXCAN_MB_FD_SIZE;
		priv->mb_num = FLEXCAN_MB_FD_NUM;
	} else {
		priv->mb_size = FLEXCAN_MB_SIZE;
		priv->mb_num = FLEXCAN_MB_NUM;
	}

	/* clear and invalidate all mailboxes first */
	i = priv->mb_mode ? 0 : FLEXCAN_TX_BUF_ID;
	for (; i < priv->mb_num; i++) {
		flexcan_mb_write(priv, i, FLEXCAN_MB_CTRL,
				 FLEXCAN_MB_CODE_RX_INACTIVE);
	}

	/* Errata ERR005829: mark first TX mailbox as INACTIVE */
	flexcan_mb_write(priv, FLEXCAN_TX_BUF_RESERVED, FLEXCAN_MB_CTRL,
			 FLEXCAN_MB_CODE_TX_INACTIVE);

	/* mark TX mailbox as INACTIVE */
	flexcan_mb_write(priv, FLEXCAN_TX_BUF_ID, FLEXCAN_MB_CTRL,
			 FLEXCAN_MB_CODE_TX_INACTIVE);

	if (priv->mb_mode) {
		/* mark RX mailbox as INACTIVE */
		flexcan_mb_write(priv, FLEXCAN_RX_BUF_ID, FLEXCAN_MB_CTRL,
				 FLEXCAN_MB_CODE_RX_EMPTY);

		/* store Remote Request Frame */
		reg_ctrl2 = flexcan_read(priv, FLEXCAN_CTRL2);
		reg_ctrl2 |= FLEXCAN_CTRL2_RRS;
		/* enable Entire Frame Arbitration Field Comparison */
		reg_ctrl2 |= FLEXCAN_CTRL2_EACEN;
		flexcan_write(priv, FLEXCAN_CTRL2, reg_ctrl2);
	}

	/* acceptance mask/acceptance code (accept everything) */
	flexcan_write(priv, FLEXCAN_RXGMASK, 0x0);
	flexcan_write(priv, FLEXCAN_RX14MASK, 0x0);
	flexcan_write(priv, FLEXCAN_RX15MASK, 0x0);

	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_DISABLE_RXFG)
		flexcan_write(priv, FLEXCAN_RXFGMASK, 0x0);

	/* On Vybrid, disable memory error detection interrupts
	 * and freeze mode.
	 * This also works around errata e5295 which generates
	 * false positive memory errors and put the device in
	 * freeze mode.
	 */
	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_DISABLE_MECR) {
		/* Follow the protocol as described in "Detection
		 * and Correction of Memory Errors" to write to
		 * MECR register
		 */
		reg_ctrl2 = flexcan_read(priv, FLEXCAN_CTRL2);
		reg_ctrl2 |= FLEXCAN_CTRL2_ECRWRE;
		flexcan_write(priv, FLEXCAN_CTRL2, reg_ctrl2);

		reg_mecr = flexcan_read(priv, FLEXCAN_MECR);
		reg_mecr &= ~FLEXCAN_MECR_ECRWRDIS;
		flexcan_write(priv, FLEXCAN_MECR, reg_mecr);
		reg_mecr &= ~(FLEXCAN_MECR_NCEFAFRZ | FLEXCAN_MECR_HANCEI_MSK |
			      FLEXCAN_MECR_FANCEI_MSK);
		flexcan_write(priv, FLEXCAN_MECR, reg_mecr);
	}

	err = flexcan_transceiver_enable(priv);
	if (err)
		goto out_chip_disable;

	/* synchronize with the can bus */
	err = flexcan_chip_unfreeze(priv);
	if (err)
		goto out_transceiver_disable;

	priv->can.state = CAN_STATE_ERROR_ACTIVE;

	/* enable interrupts atomically */
	disable_irq(dev->irq);
	flexcan_write(priv, FLEXCAN_CTRL, priv->reg_ctrl_default);
	flexcan_write(priv, FLEXCAN_IMASK1, priv->iflag_default);
	enable_irq(dev->irq);

	/* print chip status */
	netdev_dbg(dev, "%s: reading mcr=0x%08x ctrl=0x%08x\n", __func__,
		   flexcan_read(priv, FLEXCAN_MCR),
		   flexcan_read(priv, FLEXCAN_CTRL));

	return 0;

 out_transceiver_disable:
	flexcan_transceiver_disable(priv);
 out_chip_disable:
	flexcan_chip_disable(priv);
	return err;
}

/* flexcan_chip_stop
 *
 * this functions is entered with clocks enabled
 */
static void flexcan_chip_stop(struct net_device *dev)
{
	struct flexcan_priv *priv = netdev_priv(dev);

	/* freeze + disable module */
	flexcan_chip_freeze(priv);
	flexcan_chip_disable(priv);

	/* Disable all interrupts */
	flexcan_write(priv, FLEXCAN_IMASK1, 0);
	flexcan_write(priv, FLEXCAN_CTRL, priv->reg_ctrl_default &
		      ~FLEXCAN_CTRL_ERR_ALL);

	flexcan_transceiver_disable(priv);
	priv->can.state = CAN_STATE_STOPPED;
}

static int flexcan_open(struct net_device *dev)
{
	struct flexcan_priv *priv = netdev_priv(dev);
	int err;

	err = clk_prepare_enable(priv->clk_ipg);
	if (err)
		return err;

	err = clk_prepare_enable(priv->clk_per);
	if (err)
		goto out_disable_ipg;

	err = open_candev(dev);
	if (err)
		goto out_disable_per;

	err = request_irq(dev->irq, flexcan_irq, IRQF_SHARED, dev->name, dev);
	if (err)
		goto out_close;

	/* start chip and queuing */
	err = flexcan_chip_start(dev);
	if (err)
		goto out_free_irq;

	can_led_event(dev, CAN_LED_EVENT_OPEN);

	napi_enable(&priv->napi);
	netif_start_queue(dev);

	return 0;

 out_free_irq:
	free_irq(dev->irq, dev);
 out_close:
	close_candev(dev);
 out_disable_per:
	clk_disable_unprepare(priv->clk_per);
 out_disable_ipg:
	clk_disable_unprepare(priv->clk_ipg);

	return err;
}

static int flexcan_close(struct net_device *dev)
{
	struct flexcan_priv *priv = netdev_priv(dev);

	netif_stop_queue(dev);
	napi_disable(&priv->napi);
	flexcan_chip_stop(dev);

	free_irq(dev->irq, dev);
	clk_disable_unprepare(priv->clk_per);
	clk_disable_unprepare(priv->clk_ipg);

	close_candev(dev);

	can_led_event(dev, CAN_LED_EVENT_STOP);

	return 0;
}

static int flexcan_set_mode(struct net_device *dev, enum can_mode mode)
{
	int err;

	switch (mode) {
	case CAN_MODE_START:
		err = flexcan_chip_start(dev);
		if (err)
			return err;

		netif_wake_queue(dev);
		break;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static const struct net_device_ops flexcan_netdev_ops = {
	.ndo_open	= flexcan_open,
	.ndo_stop	= flexcan_close,
	.ndo_start_xmit	= flexcan_start_xmit,
	.ndo_change_mtu = can_change_mtu,
};

static int register_flexcandev(struct net_device *dev)
{
	struct flexcan_priv *priv = netdev_priv(dev);
	u32 reg, err;

	err = clk_prepare_enable(priv->clk_ipg);
	if (err)
		return err;

	err = clk_prepare_enable(priv->clk_per);
	if (err)
		goto out_disable_ipg;

	/* select "bus clock", chip must be disabled */
	err = flexcan_chip_disable(priv);
	if (err)
		goto out_disable_per;
	reg = flexcan_read(priv, FLEXCAN_CTRL);
	reg |= FLEXCAN_CTRL_CLK_SRC;
	flexcan_write(priv, FLEXCAN_CTRL, reg);

	err = flexcan_chip_enable(priv);
	if (err)
		goto out_chip_disable;

	/* set freeze, halt and activate FIFO, restrict register access */
	reg = flexcan_read(priv, FLEXCAN_MCR);
	reg |= FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT |
	       FLEXCAN_MCR_SUPV;

	if (!priv->mb_mode)
		reg |= FLEXCAN_MCR_FEN;

	flexcan_write(priv, FLEXCAN_MCR, reg);

	/* Currently we only support newer versions of this core
	 * featuring a RX FIFO. Older cores found on some Coldfire
	 * derivates are not yet supported.
	 */
	if (!priv->mb_mode) {
		reg = flexcan_read(priv, FLEXCAN_MCR);
		if (!(reg & FLEXCAN_MCR_FEN)) {
			netdev_err(dev, "Could not enable RX FIFO, unsupported core\n");
			err = -ENODEV;
			goto out_chip_disable;
		}
	}

	err = register_candev(dev);

	/* disable core and turn off clocks */
 out_chip_disable:
	flexcan_chip_disable(priv);
 out_disable_per:
	clk_disable_unprepare(priv->clk_per);
 out_disable_ipg:
	clk_disable_unprepare(priv->clk_ipg);

	return err;
}

static void unregister_flexcandev(struct net_device *dev)
{
	unregister_candev(dev);
}

static int flexcan_of_parse_stop_mode(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct device_node *np = pdev->dev.of_node;
	struct device_node *node;
	struct flexcan_priv *priv;
	phandle phandle;
	u32 out_val[5];
	int ret;

	if (!np)
		return -EINVAL;
	/*
	 * stop mode property format is:
	 * <&gpr req_gpr req_bit ack_gpr ack_bit>.
	 */
	ret = of_property_read_u32_array(np, "stop-mode", out_val, 5);
	if (ret) {
		dev_dbg(&pdev->dev, "no stop-mode property\n");
		return ret;
	}
	phandle = *out_val;

	node = of_find_node_by_phandle(phandle);
	if (!node) {
		dev_dbg(&pdev->dev, "could not find gpr node by phandle\n");
		return PTR_ERR(node);
	}

	priv = netdev_priv(dev);
	priv->stm.gpr = syscon_node_to_regmap(node);
	if (IS_ERR(priv->stm.gpr)) {
		dev_dbg(&pdev->dev, "could not find gpr regmap\n");
		return PTR_ERR(priv->stm.gpr);
	}

	of_node_put(node);

	priv->stm.req_gpr = out_val[1];
	priv->stm.req_bit = out_val[2];
	priv->stm.ack_gpr = out_val[3];
	priv->stm.ack_bit = out_val[4];

	dev_dbg(&pdev->dev, "gpr %s req_gpr 0x%x req_bit %u ack_gpr 0x%x ack_bit %u\n",
			node->full_name, priv->stm.req_gpr,
			priv->stm.req_bit, priv->stm.ack_gpr,
			priv->stm.ack_bit);
	return 0;
}

static const struct of_device_id flexcan_of_match[] = {
	{ .compatible = "fsl,imx8qm-flexcan", .data = &fsl_imx8qm_devtype_data, },
	{ .compatible = "fsl,imx6q-flexcan", .data = &fsl_imx6q_devtype_data, },
	{ .compatible = "fsl,imx28-flexcan", .data = &fsl_imx28_devtype_data, },
	{ .compatible = "fsl,p1010-flexcan", .data = &fsl_p1010_devtype_data, },
	{ .compatible = "fsl,vf610-flexcan", .data = &fsl_vf610_devtype_data, },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, flexcan_of_match);

static const struct platform_device_id flexcan_id_table[] = {
	{ .name = "flexcan", .driver_data = (kernel_ulong_t)&fsl_p1010_devtype_data, },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(platform, flexcan_id_table);

static int flexcan_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id;
	const struct flexcan_devtype_data *devtype_data;
	struct net_device *dev;
	struct flexcan_priv *priv;
	struct regulator *reg_xceiver;
	struct resource *mem;
	struct clk *clk_ipg = NULL, *clk_per = NULL;
	void __iomem *regs;
	int err, irq;
	u32 clock_freq = 0;
	int wakeup = 1;

	reg_xceiver = devm_regulator_get(&pdev->dev, "xceiver");
	if (PTR_ERR(reg_xceiver) == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	else if (IS_ERR(reg_xceiver))
		reg_xceiver = NULL;

	if (pdev->dev.of_node)
		of_property_read_u32(pdev->dev.of_node,
				     "clock-frequency", &clock_freq);

	if (!clock_freq) {
		clk_ipg = devm_clk_get(&pdev->dev, "ipg");
		if (IS_ERR(clk_ipg)) {
			dev_err(&pdev->dev, "no ipg clock defined\n");
			return PTR_ERR(clk_ipg);
		}

		clk_per = devm_clk_get(&pdev->dev, "per");
		if (IS_ERR(clk_per)) {
			dev_err(&pdev->dev, "no per clock defined\n");
			return PTR_ERR(clk_per);
		}
		clock_freq = clk_get_rate(clk_per);
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (irq <= 0)
		return -ENODEV;

	regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	of_id = of_match_device(flexcan_of_match, &pdev->dev);
	if (of_id) {
		devtype_data = of_id->data;
	} else if (platform_get_device_id(pdev)->driver_data) {
		devtype_data = (struct flexcan_devtype_data *)
			platform_get_device_id(pdev)->driver_data;
	} else {
		return -ENODEV;
	}

	dev = alloc_candev(sizeof(struct flexcan_priv), 1);
	if (!dev)
		return -ENOMEM;

	dev->netdev_ops = &flexcan_netdev_ops;
	dev->irq = irq;
	dev->flags |= IFF_ECHO;

	priv = netdev_priv(dev);
	priv->can.clock.freq = clock_freq;
	priv->can.bittiming_const = &flexcan_bittiming_const;
	priv->can.data_bittiming_const = &flexcan_data_bittiming_const;
	priv->can.do_set_mode = flexcan_set_mode;
	priv->can.do_get_berr_counter = flexcan_get_berr_counter;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK |
		CAN_CTRLMODE_LISTENONLY	| CAN_CTRLMODE_3_SAMPLES |
		CAN_CTRLMODE_BERR_REPORTING;
	priv->base = regs;
	priv->clk_ipg = clk_ipg;
	priv->clk_per = clk_per;
	priv->pdata = dev_get_platdata(&pdev->dev);
	priv->devtype_data = devtype_data;
	priv->reg_xceiver = reg_xceiver;

	netif_napi_add(dev, &priv->napi, flexcan_poll, FLEXCAN_NAPI_WEIGHT);

	platform_set_drvdata(pdev, dev);
	SET_NETDEV_DEV(dev, &pdev->dev);

	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_DISABLE_RX_FIFO) {
		priv->mb_mode = true;
		priv->rx_int = FLEXCAN_RX_BUF_INT;
		priv->iflag_default = FLEXCAN_IFLAG_DEFAULT_MB;
	} else {
		priv->mb_mode = false;
		priv->rx_int = FLEXCAN_IFLAG_RX_FIFO_AVAILABLE;
		priv->iflag_default = FLEXCAN_IFLAG_DEFAULT_FIFO;
	}

	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_SUPPORT_CANFD) {
		priv->can.ctrlmode_supported |= CAN_CTRLMODE_FD;
		if (!(priv->devtype_data->quirks &
		      FLEXCAN_QUIRK_DISABLE_RX_FIFO)) {
			dev_err(&pdev->dev, "canfd mode can't work on fifo mode\n");
			err = -EINVAL;
			goto failed_register;
		}
	}

	err = register_flexcandev(dev);
	if (err) {
		dev_err(&pdev->dev, "registering netdev failed\n");
		goto failed_register;
	}

	devm_can_led_init(dev);

	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_DISABLE_RXFG) {
		err = flexcan_of_parse_stop_mode(pdev);
		if (err) {
			wakeup = 0;
			dev_dbg(&pdev->dev, "failed to parse stop-mode\n");
		}

	}

	device_set_wakeup_capable(&pdev->dev, wakeup);

	dev_info(&pdev->dev, "device registered (reg_base=%p, irq=%d)\n",
		 priv->base, dev->irq);

	return 0;

 failed_register:
	free_candev(dev);
	return err;
}

static int flexcan_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct flexcan_priv *priv = netdev_priv(dev);

	unregister_flexcandev(dev);
	netif_napi_del(&priv->napi);
	free_candev(dev);

	return 0;
}

static int __maybe_unused flexcan_suspend(struct device *device)
{
	struct net_device *dev = dev_get_drvdata(device);
	struct flexcan_priv *priv = netdev_priv(dev);

	if (netif_running(dev)) {
		netif_stop_queue(dev);
		netif_device_detach(dev);
		/*
		* if wakeup is enabled, enter stop mode
		* else enter disabled mode.
		*/
		if (device_may_wakeup(device)) {
			enable_irq_wake(dev->irq);
			flexcan_enter_stop_mode(priv);
		} else {
			flexcan_chip_stop(dev);
		}
	}
	priv->can.state = CAN_STATE_SLEEPING;

	pinctrl_pm_select_sleep_state(device);

	return 0;
}

static int __maybe_unused flexcan_resume(struct device *device)
{
	struct net_device *dev = dev_get_drvdata(device);
	struct flexcan_priv *priv = netdev_priv(dev);
	int err = 0;

	pinctrl_pm_select_default_state(device);

	priv->can.state = CAN_STATE_ERROR_ACTIVE;
	if (netif_running(dev)) {
		netif_device_attach(dev);
		netif_start_queue(dev);

		if (device_may_wakeup(device)) {
			disable_irq_wake(dev->irq);
			flexcan_exit_stop_mode(priv);
		}

		err = flexcan_chip_start(dev);
	}

	return err;
}

static SIMPLE_DEV_PM_OPS(flexcan_pm_ops, flexcan_suspend, flexcan_resume);

static struct platform_driver flexcan_driver = {
	.driver = {
		.name = DRV_NAME,
		.pm = &flexcan_pm_ops,
		.of_match_table = flexcan_of_match,
	},
	.probe = flexcan_probe,
	.remove = flexcan_remove,
	.id_table = flexcan_id_table,
};

module_platform_driver(flexcan_driver);

MODULE_AUTHOR("Sascha Hauer <kernel@pengutronix.de>, "
	      "Marc Kleine-Budde <kernel@pengutronix.de>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CAN port driver for flexcan based chip");
