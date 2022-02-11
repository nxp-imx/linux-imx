// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017 Pengutronix, Oleksij Rempel <kernel@pengutronix.de>
 * Copyright 2020 NXP, Peng Fan <peng.fan@nxp.com>
 */

#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/err.h>
#ifdef CONFIG_IMX_SCU
#include <linux/firmware/imx/sci.h>
#include <dt-bindings/firmware/imx/rsrc.h>
#endif
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mailbox_client.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/remoteproc.h>

#include <soc/imx/imx_sip.h>

#include "remoteproc_internal.h"

#define IMX7D_SRC_SCR			0x0C
#define IMX7D_ENABLE_M4			BIT(3)
#define IMX7D_SW_M4P_RST		BIT(2)
#define IMX7D_SW_M4C_RST		BIT(1)
#define IMX7D_SW_M4C_NON_SCLR_RST	BIT(0)

#define IMX7D_M4_RST_MASK		(IMX7D_ENABLE_M4 | IMX7D_SW_M4P_RST \
					 | IMX7D_SW_M4C_RST \
					 | IMX7D_SW_M4C_NON_SCLR_RST)

#define IMX7D_M4_START			(IMX7D_ENABLE_M4 | IMX7D_SW_M4P_RST \
					 | IMX7D_SW_M4C_RST)
#define IMX7D_M4_STOP			(IMX7D_ENABLE_M4 | IMX7D_SW_M4C_NON_SCLR_RST)

/* Address: 0x020D8000 */
#define IMX6SX_SRC_SCR			0x00
#define IMX6SX_ENABLE_M4		BIT(22)
#define IMX6SX_SW_M4P_RST		BIT(12)
#define IMX6SX_SW_M4C_NON_SCLR_RST	BIT(4)
#define IMX6SX_SW_M4C_RST		BIT(3)

#define IMX6SX_M4_START			(IMX6SX_ENABLE_M4 | IMX6SX_SW_M4P_RST \
					 | IMX6SX_SW_M4C_RST)
#define IMX6SX_M4_STOP			(IMX6SX_ENABLE_M4 | IMX6SX_SW_M4C_NON_SCLR_RST)
#define IMX6SX_M4_RST_MASK		(IMX6SX_ENABLE_M4 | IMX6SX_SW_M4P_RST \
					 | IMX6SX_SW_M4C_NON_SCLR_RST \
					 | IMX6SX_SW_M4C_RST)

#define IMX7D_RPROC_MEM_MAX		16

/*
 * 1: indicated that remote processor is ready from re-initialization.
 * Clear this bit after the RPMSG restore is finished at master side.
 */
#define REMOTE_IS_READY			BIT(0)

/*
 * The time consumption by remote ready is less than 1ms in the
 * evaluation. Set the max wait timeout as 50ms here.
 */
#define REMOTE_READY_WAIT_MAX_RETRIES	500

enum imx_rproc_variants {
	IMX8QM,
	IMX8QXP,
	IMX8MQ,
	IMX8MP,
	IMX8MN,
	IMX7ULP,
	IMX7D,
	IMX6SX,
};

/**
 * struct imx_rproc_mem - slim internal memory structure
 * @cpu_addr: MPU virtual address of the memory region
 * @sys_addr: Bus address used to access the memory region
 * @size: Size of the memory region
 */
struct imx_rproc_mem {
	void __iomem *cpu_addr;
	phys_addr_t sys_addr;
	size_t size;
};

/* att flags */
/* M4 own area. Can be mapped at probe */
#define ATT_OWN		BIT(31)
/* I = [0:7] */
#define ATT_CORE_MASK	0xffff
#define ATT_CORE(I)	BIT((I))

/* address translation table */
struct imx_rproc_att {
	u32 da;	/* device address (From Cortex M4 view)*/
	u32 sa;	/* system bus address */
	u32 size; /* size of reg range */
	int flags;
};

struct imx_rproc_dcfg {
	u32				src_reg;
	u32				src_mask;
	u32				src_start;
	u32				src_stop;
	const struct imx_rproc_att	*att;
	size_t				att_size;
	enum imx_rproc_variants		variant;
};

struct imx_rproc {
	struct device			*dev;
	struct regmap			*regmap;
	struct reset_control		*enable;
	struct reset_control		*non_sclr_rst;
	struct rproc			*rproc;
	const struct imx_rproc_dcfg	*dcfg;
	struct imx_rproc_mem		mem[IMX7D_RPROC_MEM_MAX];
	struct clk			*clk;
	bool				ipc_only;
	bool				early_boot;
	bool				skip_fw_load_recovery;
	void				*rsc_va;
	struct mbox_client		cl;
	struct mbox_client		cl_rxdb;
	struct mbox_client		cl_txdb;
	struct mbox_chan		*tx_ch;
	struct mbox_chan		*rx_ch;
	struct mbox_chan		*rxdb_ch;
	struct mbox_chan		*txdb_ch;
	struct delayed_work		rproc_work;
	u32				mub_partition;
	struct notifier_block		proc_nb;
	u32				flags;
	spinlock_t			mu_lock;
	u32				rsrc;
	u32				id;
	int				num_domains;
	struct device			**pm_devices;
	struct device_link		**pm_devices_link;
};

#ifdef CONFIG_IMX_SCU
static struct imx_sc_ipc *ipc_handle;
#endif

static const struct imx_rproc_att imx_rproc_att_imx8qm[] = {
	/* dev addr , sys addr  , size	    , flags */
	{ 0x08000000, 0x08000000, 0x10000000, 0},
	/* TCML */
	{ 0x1FFE0000, 0x34FE0000, 0x00020000, ATT_OWN | ATT_CORE(0)},
	{ 0x1FFE0000, 0x38FE0000, 0x00020000, ATT_OWN | ATT_CORE(1)},
	/* TCMU */
	{ 0x20000000, 0x35000000, 0x00020000, ATT_OWN | ATT_CORE(0)},
	{ 0x20000000, 0x39000000, 0x00020000, ATT_OWN | ATT_CORE(1)},
	/* DDR (Data) */
	{ 0x80000000, 0x80000000, 0x60000000, 0 },
};

static const struct imx_rproc_att imx_rproc_att_imx8qxp[] = {
	/* dev addr , sys addr  , size	    , flags */
	{ 0x08000000, 0x08000000, 0x10000000, 0},
	/* TCML */
	{ 0x1FFE0000, 0x34FE0000, 0x00020000, ATT_OWN },
	/* TCMU */
	{ 0x20000000, 0x35000000, 0x00020000, ATT_OWN },
	/* OCRAM(Low 96KB) */
	{ 0x21000000, 0x00100000, 0x00018000, 0},
	/* OCRAM */
	{ 0x21100000, 0x00100000, 0x00040000, 0},
	/* DDR (Data) */
	{ 0x80000000, 0x80000000, 0x60000000, 0 },
};

static const struct imx_rproc_att imx_rproc_att_imx8mn[] = {
	/* dev addr , sys addr  , size	    , flags */
	/* ITCM   */
	{ 0x00000000, 0x007E0000, 0x00020000, ATT_OWN },
	/* OCRAM_S */
	{ 0x00180000, 0x00180000, 0x00009000, 0 },
	/* OCRAM */
	{ 0x00900000, 0x00900000, 0x00020000, 0 },
	/* OCRAM */
	{ 0x00920000, 0x00920000, 0x00020000, 0 },
	/* OCRAM */
	{ 0x00940000, 0x00940000, 0x00050000, 0 },
	/* QSPI Code - alias */
	{ 0x08000000, 0x08000000, 0x08000000, 0 },
	/* DDR (Code) - alias */
	{ 0x10000000, 0x40000000, 0x0FFE0000, 0 },
	/* DTCM */
	{ 0x20000000, 0x00800000, 0x00020000, ATT_OWN },
	/* OCRAM_S - alias */
	{ 0x20180000, 0x00180000, 0x00008000, ATT_OWN },
	/* OCRAM */
	{ 0x20200000, 0x00900000, 0x00020000, ATT_OWN },
	/* OCRAM */
	{ 0x20220000, 0x00920000, 0x00020000, ATT_OWN },
	/* OCRAM */
	{ 0x20240000, 0x00940000, 0x00040000, ATT_OWN },
	/* DDR (Data) */
	{ 0x40000000, 0x40000000, 0x80000000, 0 },
};

static const struct imx_rproc_att imx_rproc_att_imx8mq[] = {
	/* dev addr , sys addr  , size	    , flags */
	/* TCML - alias */
	{ 0x00000000, 0x007e0000, 0x00020000, 0 },
	/* OCRAM_S */
	{ 0x00180000, 0x00180000, 0x00008000, 0 },
	/* OCRAM */
	{ 0x00900000, 0x00900000, 0x00020000, 0 },
	/* OCRAM */
	{ 0x00920000, 0x00920000, 0x00020000, 0 },
	/* QSPI Code - alias */
	{ 0x08000000, 0x08000000, 0x08000000, 0 },
	/* DDR (Code) - alias */
	{ 0x10000000, 0x40000000, 0x0FFE0000, 0 },
	/* TCML/U */
	{ 0x1FFE0000, 0x007E0000, 0x00040000, ATT_OWN },
	/* OCRAM_S */
	{ 0x20180000, 0x00180000, 0x00008000, ATT_OWN },
	/* OCRAM */
	{ 0x20200000, 0x00900000, 0x00020000, ATT_OWN },
	/* OCRAM */
	{ 0x20220000, 0x00920000, 0x00020000, ATT_OWN },
	/* DDR (Data) */
	{ 0x40000000, 0x40000000, 0x80000000, 0 },
};

static const struct imx_rproc_att imx_rproc_att_imx7ulp[] = {
	{0x1FFD0000, 0x1FFD0000, 0x30000, ATT_OWN},
	{0x20000000, 0x20000000, 0x10000, ATT_OWN},
	{0x2F000000, 0x2F000000, 0x20000, ATT_OWN},
	{0x2F020000, 0x2F020000, 0x20000, ATT_OWN},
	{0x60000000, 0x60000000, 0x40000000, 0}
};

static const struct imx_rproc_att imx_rproc_att_imx7d[] = {
	/* dev addr , sys addr  , size	    , flags */
	/* OCRAM_S (M4 Boot code) - alias */
	{ 0x00000000, 0x00180000, 0x00008000, 0 },
	/* OCRAM_S (Code) */
	{ 0x00180000, 0x00180000, 0x00008000, ATT_OWN },
	/* OCRAM (Code) - alias */
	{ 0x00900000, 0x00900000, 0x00020000, 0 },
	/* OCRAM_EPDC (Code) - alias */
	{ 0x00920000, 0x00920000, 0x00020000, 0 },
	/* OCRAM_PXP (Code) - alias */
	{ 0x00940000, 0x00940000, 0x00008000, 0 },
	/* TCML (Code) */
	{ 0x1FFF8000, 0x007F8000, 0x00008000, ATT_OWN },
	/* DDR (Code) - alias, first part of DDR (Data) */
	{ 0x10000000, 0x80000000, 0x0FFF0000, 0 },

	/* TCMU (Data) */
	{ 0x20000000, 0x00800000, 0x00008000, ATT_OWN },
	/* OCRAM (Data) */
	{ 0x20200000, 0x00900000, 0x00020000, 0 },
	/* OCRAM_EPDC (Data) */
	{ 0x20220000, 0x00920000, 0x00020000, 0 },
	/* OCRAM_PXP (Data) */
	{ 0x20240000, 0x00940000, 0x00008000, 0 },
	/* DDR (Data) */
	{ 0x80000000, 0x80000000, 0x60000000, 0 },
};

static const struct imx_rproc_att imx_rproc_att_imx6sx[] = {
	/* dev addr , sys addr  , size	    , flags */
	/* TCML (M4 Boot Code) - alias */
	{ 0x00000000, 0x007F8000, 0x00008000, 0 },
	/* OCRAM_S (Code) */
	{ 0x00180000, 0x008F8000, 0x00004000, 0 },
	/* OCRAM_S (Code) - alias */
	{ 0x00180000, 0x008FC000, 0x00004000, 0 },
	/* TCML (Code) */
	{ 0x1FFF8000, 0x007F8000, 0x00008000, ATT_OWN },
	/* DDR (Code) - alias, first part of DDR (Data) */
	{ 0x10000000, 0x80000000, 0x0FFF8000, 0 },

	/* TCMU (Data) */
	{ 0x20000000, 0x00800000, 0x00008000, ATT_OWN },
	/* OCRAM_S (Data) - alias? */
	{ 0x208F8000, 0x008F8000, 0x00004000, 0 },
	/* DDR (Data) */
	{ 0x80000000, 0x80000000, 0x60000000, 0 },
};

static const struct imx_rproc_dcfg imx_rproc_cfg_imx8mn = {
	.att		= imx_rproc_att_imx8mn,
	.att_size	= ARRAY_SIZE(imx_rproc_att_imx8mn),
	.variant	= IMX8MN,
};

static const struct imx_rproc_dcfg imx_rproc_cfg_imx8mq = {
	.src_reg	= IMX7D_SRC_SCR,
	.src_mask	= IMX7D_M4_RST_MASK,
	.src_start	= IMX7D_M4_START,
	.src_stop	= IMX7D_M4_STOP,
	.att		= imx_rproc_att_imx8mq,
	.att_size	= ARRAY_SIZE(imx_rproc_att_imx8mq),
	.variant	= IMX8MQ,
};

static const struct imx_rproc_dcfg imx_rproc_cfg_imx7ulp = {
	.att		= imx_rproc_att_imx7ulp,
	.att_size	= ARRAY_SIZE(imx_rproc_att_imx7ulp),
	.variant	= IMX7ULP,
};

static const struct imx_rproc_dcfg imx_rproc_cfg_imx7d = {
	.src_reg	= IMX7D_SRC_SCR,
	.src_mask	= IMX7D_M4_RST_MASK,
	.src_start	= IMX7D_M4_START,
	.src_stop	= IMX7D_M4_STOP,
	.att		= imx_rproc_att_imx7d,
	.att_size	= ARRAY_SIZE(imx_rproc_att_imx7d),
	.variant	= IMX7D,
};

static const struct imx_rproc_dcfg imx_rproc_cfg_imx6sx = {
	.src_reg	= IMX6SX_SRC_SCR,
	.src_mask	= IMX6SX_M4_RST_MASK,
	.src_start	= IMX6SX_M4_START,
	.src_stop	= IMX6SX_M4_STOP,
	.att		= imx_rproc_att_imx6sx,
	.att_size	= ARRAY_SIZE(imx_rproc_att_imx6sx),
	.variant	= IMX6SX,
};

static const struct imx_rproc_dcfg imx_rproc_cfg_imx8qxp = {
	.att		= imx_rproc_att_imx8qxp,
	.att_size	= ARRAY_SIZE(imx_rproc_att_imx8qxp),
	.variant	= IMX8QXP,
};

static const struct imx_rproc_dcfg imx_rproc_cfg_imx8qm = {
	.att		= imx_rproc_att_imx8qm,
	.att_size	= ARRAY_SIZE(imx_rproc_att_imx8qm),
	.variant	= IMX8QM,
};

bool imx_rproc_ready(struct rproc *rproc)
{
	struct imx_rproc *priv = rproc->priv;
	int i;

	for (i = 0; i < REMOTE_READY_WAIT_MAX_RETRIES; i++) {
		if (priv->flags & REMOTE_IS_READY)
			break;
		udelay(100);
	}

	return true;
}

static int imx_rproc_rebuild_channels(struct rproc *rproc)
{
	struct imx_rproc *priv = rproc->priv;
	struct mbox_client *cl = &priv->cl;
	struct device *dev = priv->dev;
	int ret = 0;

	if (!priv->tx_ch) {
		priv->tx_ch = mbox_request_channel_byname(cl, "tx");
		if (IS_ERR(priv->tx_ch)) {
			ret = PTR_ERR(priv->tx_ch);
			dev_err(dev, "failed to restart tx chan %d\n", ret);
			priv->tx_ch = NULL;

			goto err_exit;
		}
	}

	if (!priv->rx_ch) {
		priv->rx_ch = mbox_request_channel_byname(cl, "rx");
		if (IS_ERR(priv->rx_ch)) {
			ret = PTR_ERR(priv->rx_ch);
			dev_err(dev, "failed to restart rx chan %d\n", ret);
			priv->rx_ch = NULL;

			goto err_exit;
		}
	}

	if (!priv->rxdb_ch) {
		priv->rxdb_ch = mbox_request_channel_byname(cl, "rxdb");
		if (IS_ERR(priv->rxdb_ch)) {
			ret = PTR_ERR(priv->rxdb_ch);
			dev_err(dev, "failed to restart rxdb chan %d\n", ret);
			priv->rxdb_ch = NULL;

			goto err_exit;
		}
	}

	/* txdb is optional */
	if (!priv->txdb_ch) {
		priv->txdb_ch = mbox_request_channel_byname(cl, "txdb");
		if (IS_ERR(priv->txdb_ch))
			priv->txdb_ch = NULL;
	}

err_exit:
	return ret;
}

static void imx_rproc_free_channels(struct rproc *rproc)
{
	struct imx_rproc *priv = rproc->priv;
	__u32 mmsg;

	if (priv->txdb_ch)
		mbox_send_message(priv->txdb_ch, (void *)&mmsg);

	mbox_free_channel(priv->tx_ch);
	mbox_free_channel(priv->rx_ch);
	mbox_free_channel(priv->rxdb_ch);
	mbox_free_channel(priv->txdb_ch);

	priv->tx_ch = NULL;
	priv->rx_ch = NULL;
	priv->rxdb_ch = NULL;
	priv->txdb_ch = NULL;
}

static int imx_rproc_start(struct rproc *rproc)
{
	struct imx_rproc *priv = rproc->priv;
	const struct imx_rproc_dcfg *dcfg = priv->dcfg;
	struct device *dev = priv->dev;
	struct arm_smccc_res res;
	int ret;
	u32 val;

	if (priv->ipc_only) {
		dev_info(dev, "%s: IPC only\n", __func__);
		ret = imx_rproc_rebuild_channels(rproc);
		if (ret < 0)
			return -EINVAL;

		/* To partition M4, we need block userspace stop/start */
		if (priv->skip_fw_load_recovery) {
			priv->skip_fw_load_recovery = false;
			/* Wait remoteproc ready restart itself */
			imx_rproc_ready(rproc);
		}
		return 0;
	}

#ifdef CONFIG_IMX_SCU
	if (priv->dcfg->variant == IMX8QXP || priv->dcfg->variant == IMX8QM) {
		if (priv->id == 1)
			ret = imx_sc_pm_cpu_start(ipc_handle, priv->rsrc, true, 0x38fe0000);
		else if (!priv->id)
			ret = imx_sc_pm_cpu_start(ipc_handle, priv->rsrc, true, 0x34fe0000);
		else
			ret = -EINVAL;
		if (ret) {
			dev_err(dev, "Failed to enable M4!\n");
			return ret;
		}
		return 0;
	}
#endif

	if (priv->dcfg->variant == IMX8MN) {
		arm_smccc_smc(IMX_SIP_SRC, IMX_SIP_SRC_M4_STARTED, 0, 0, 0, 0, 0, 0, &res);
		if (!res.a0) {
			arm_smccc_smc(IMX_SIP_SRC, IMX_SIP_SRC_M4_START, 0, 0, 0, 0, 0, 0, &res);
			ret = res.a0;
		} else {
			ret = 0;
		}
	} else if (priv->enable) {
		if (!reset_control_status(priv->enable)) {
			dev_info(dev, "alreay started\n");
			return 0;
		}

		ret = reset_control_deassert(priv->enable);
		if (!ret)
			ret = reset_control_deassert(priv->non_sclr_rst);
	} else {
		if (!priv->regmap)
			return -ENOTSUPP;

		ret = regmap_read(priv->regmap, dcfg->src_reg, &val);
		if (!(val & dcfg->src_stop)) {
			dev_info(dev, "alreay started\n");
			return 0;
		}

		ret = regmap_update_bits(priv->regmap, dcfg->src_reg,
					 dcfg->src_mask, dcfg->src_start);
	}

	if (ret)
		dev_err(dev, "Failed to enable M4!\n");
	else
		imx_rproc_ready(rproc);

	return ret;
}

static int imx_rproc_stop(struct rproc *rproc)
{
	struct imx_rproc *priv = rproc->priv;
	const struct imx_rproc_dcfg *dcfg = priv->dcfg;
	struct device *dev = priv->dev;
	struct arm_smccc_res res;
	int ret;
	__u32 mmsg;

	if (priv->ipc_only) {
		dev_info(dev, "%s: IPC only\n", __func__);
		imx_rproc_free_channels(rproc);

		/*
		 * TO i.MX8 Paritioned M4, M4 reboot is handled by itself,
		 * so we still keep early boot and skip fw load flag
		 * To allow m4 partition reset trigger remoteproc crash
		 * handler.
		 */
		if (priv->skip_fw_load_recovery) {
			priv->flags &= ~REMOTE_IS_READY;
			return 0;
		}
		return -EBUSY;
	}

	if (priv->txdb_ch) {
		ret = mbox_send_message(priv->txdb_ch, (void *)&mmsg);
		if (ret) {
			dev_err(dev, "txdb send fail: %d\n", ret);
			return ret;
		}
	}
#ifdef CONFIG_IMX_SCU
	if (priv->dcfg->variant == IMX8QXP || priv->dcfg->variant == IMX8QM) {
		if (priv->id == 1)
			ret = imx_sc_pm_cpu_start(ipc_handle, priv->rsrc, false, 0x38fe0000);
		else if (!priv->id)
			ret = imx_sc_pm_cpu_start(ipc_handle, priv->rsrc, false, 0x34fe0000);
		else
			ret = -EINVAL;
		if (ret) {
			dev_err(dev, "Failed to stop M4!\n");
			return ret;
		}
		return 0;
	}
#endif

	if (priv->dcfg->variant == IMX8MN) {
		arm_smccc_smc(IMX_SIP_SRC, IMX_SIP_SRC_M4_STOP, 0, 0, 0, 0, 0, 0, &res);
		ret = res.a0;
		if (res.a1)
			dev_info(dev, "remotecore might not run into wfi, force stop: %ld %ld %ld\n", res.a0, res.a1, res.a2);
	} else if (priv->enable) {
		ret = reset_control_assert(priv->enable);
		if (!ret)
			ret = reset_control_assert(priv->non_sclr_rst);
	} else {
		if (!priv->regmap)
			return -ENOTSUPP;

		ret = regmap_update_bits(priv->regmap, dcfg->src_reg,
					 dcfg->src_mask, dcfg->src_stop);
	}

	if (ret) {
		dev_err(dev, "Failed to stop M4!\n");
	} else {
		priv->early_boot = false;
		priv->rproc->skip_fw_load = false;
		priv->flags &= ~REMOTE_IS_READY;
	}

	return ret;
}

static int imx_rproc_da_to_sys(struct imx_rproc *priv, u64 da,
			       int len, u64 *sys)
{
	const struct imx_rproc_dcfg *dcfg = priv->dcfg;
	int i;

	/* parse address translation table */
	for (i = 0; i < dcfg->att_size; i++) {
		const struct imx_rproc_att *att = &dcfg->att[i];

		if (att->flags & ATT_CORE_MASK) {
			if (!((1 << priv->id) & (att->flags & ATT_CORE_MASK)))
				continue;
		}

		if (da >= att->da && da + len < att->da + att->size) {
			unsigned int offset = da - att->da;

			*sys = att->sa + offset;
			return 0;
		}
	}

	dev_warn(priv->dev, "Translation failed: da = 0x%llx len = 0x%x\n",
		 da, len);
	return -ENOENT;
}

static void *imx_rproc_da_to_va(struct rproc *rproc, u64 da, int len)
{
	struct imx_rproc *priv = rproc->priv;
	void *va = NULL;
	u64 sys;
	int i;

	if (len <= 0)
		return NULL;

	/*
	 * On device side we have many aliases, so we need to convert device
	 * address (M4) to system bus address first.
	 */
	if (imx_rproc_da_to_sys(priv, da, len, &sys))
		return NULL;

	for (i = 0; i < IMX7D_RPROC_MEM_MAX; i++) {
		if (sys >= priv->mem[i].sys_addr && sys + len <
		    priv->mem[i].sys_addr +  priv->mem[i].size) {
			unsigned int offset = sys - priv->mem[i].sys_addr;
			/* __force to make sparse happy with type conversion */
			va = (__force void *)(priv->mem[i].cpu_addr + offset);
			break;
		}
	}

	dev_dbg(&rproc->dev, "da = 0x%llx len = 0x%x va = 0x%p\n", da, len, va);

	return va;
}

static int imx_rproc_elf_load_segments(struct rproc *rproc,
				       const struct firmware *fw)
{
	struct imx_rproc *priv = rproc->priv;

	if (!priv->early_boot) {
		if (!fw)
			return -EINVAL;
		return rproc_elf_load_segments(rproc, fw);
	}

	return 0;
}

static int imx_rproc_mem_alloc(struct rproc *rproc,
			       struct rproc_mem_entry *mem)
{
	struct device *dev = rproc->dev.parent;
	void *va;

	dev_dbg(dev, "map memory: %p+%x\n", &mem->dma, mem->len);
	va = ioremap_wc(mem->dma, mem->len);
	if (IS_ERR_OR_NULL(va)) {
		dev_err(dev, "Unable to map memory region: %p+%x\n",
			&mem->dma, mem->len);
		return -ENOMEM;
	}

	/* Update memory entry va */
	mem->va = va;

	return 0;
}

static int imx_rproc_mem_release(struct rproc *rproc,
				 struct rproc_mem_entry *mem)
{
	dev_dbg(rproc->dev.parent, "unmap memory: %pa\n", &mem->dma);
	iounmap(mem->va);

	return 0;
}

static int check_dt_rsc_table(struct rproc *rproc)
{
	struct imx_rproc *priv = rproc->priv;
	struct resource_table *resource_table;
	struct device_node *np = priv->dev->of_node;
	int elems;
	int ret;

	/*Parse device tree to get resource table */
	elems = of_property_count_u32_elems(np, "rsrc-table");
	if (elems < 0) {
		dev_err(&rproc->dev, "no dtb rsrc-table\n");
		return elems;
	}

	resource_table = kzalloc(elems * sizeof(u32), GFP_KERNEL);
	if (!resource_table)
		return PTR_ERR(resource_table);

	ret = of_property_read_u32_array(np, "rsrc-table",
					 (u32 *)resource_table, elems);
	if (ret)
		return ret;

	rproc->cached_table = resource_table;
	rproc->table_ptr = resource_table;
	rproc->table_sz = elems * sizeof(u32);

	return 0;
}

static int imx_rproc_parse_fw(struct rproc *rproc, const struct firmware *fw)
{
	struct imx_rproc *priv = rproc->priv;
	struct device_node *np = priv->dev->of_node;
	struct of_phandle_iterator it;
	struct rproc_mem_entry *mem;
	struct reserved_mem *rmem;
	int index = 0;
	u32 da;
	int ret;

	/* Register associated reserved memory regions */
	of_phandle_iterator_init(&it, np, "memory-region", NULL, 0);
	while (of_phandle_iterator_next(&it) == 0) {
		rmem = of_reserved_mem_lookup(it.node);
		if (!rmem) {
			dev_err(priv->dev, "unable to acquire memory-region\n");
			return -EINVAL;
		}

		/* No need to translate pa to da */
		da = rmem->base;

		if (strcmp(it.node->name, "vdevbuffer")) {
			/* Register memory region */
			mem = rproc_mem_entry_init(priv->dev, NULL,
						   (dma_addr_t)rmem->base,
						   rmem->size, da,
						   imx_rproc_mem_alloc,
						   imx_rproc_mem_release,
						   it.node->name);

			if (mem)
				rproc_coredump_add_segment(rproc, da,
							   rmem->size);
		} else {
			/* Register reserved memory for vdev buffer alloc */
			mem = rproc_of_resm_mem_entry_init(priv->dev, index,
							   rmem->size,
							   rmem->base,
							   it.node->name);
		}

		if (!mem)
			return -ENOMEM;

		rproc_add_carveout(rproc, mem);
		index++;
	}

	if (!check_dt_rsc_table(rproc))
		return 0;

	if (priv->early_boot) {
		struct resource_table *table = NULL;

		ret = of_property_read_u32(np, "rsc-da", &da);
		if (!ret)
			priv->rsc_va = rproc_da_to_va(rproc, (u64)da, SZ_1K);
		else
			return 0;

		if (!priv->rsc_va) {
			dev_err(priv->dev, "no map for rsc-da: %x\n", da);
			return 0;
		}

		table = (struct resource_table *)priv->rsc_va;

		if (table->ver != 1 || table->reserved[0] || table->reserved[1]) {
			dev_err(priv->dev, "Invalid rsrc table header\n");
			return 0;
		}

		/* Assuming that the resource table fits in 1kB is fair */
		rproc->cached_table = kmemdup(table, SZ_1K, GFP_KERNEL);
		if (!rproc->cached_table)
			return -ENOMEM;

		rproc->table_ptr = rproc->cached_table;
		rproc->table_sz = SZ_1K;
		return 0;
	}

	if (!priv->early_boot) {
		ret = rproc_elf_load_rsc_table(rproc, fw);
		if (ret)
			dev_info(priv->dev, "No resource table in elf\n");
	}

	return  0;
}

static struct resource_table *
imx_rproc_elf_find_loaded_rsc_table(struct rproc *rproc,
				    const struct firmware *fw)
{
	struct imx_rproc *priv = rproc->priv;

	if (!priv->early_boot)
		return rproc_elf_find_loaded_rsc_table(rproc, fw);

	/*
	 * Not use (struct resource_table *)priv->rsc_va;
	 * M4 use vring to publish resource table, for multiple vdev,
	 * the first vdev kick will overwrite the resource table if
	 * using priv->rsc_va, then 2nd vdev will not be able to get
	 * the correct info from loaded table.
	 */
	return NULL;
}

static int imx_rproc_elf_sanity_check(struct rproc *rproc,
				      const struct firmware *fw)
{
	struct imx_rproc *priv = rproc->priv;

	if (!priv->early_boot)
		return rproc_elf_sanity_check(rproc, fw);

	return 0;
}

static u32 imx_rproc_elf_get_boot_addr(struct rproc *rproc,
				       const struct firmware *fw)
{
	struct imx_rproc *priv = rproc->priv;

	if (!priv->early_boot)
		return rproc_elf_get_boot_addr(rproc, fw);

	return 0;
}

static void imx_rproc_kick(struct rproc *rproc, int vqid)
{
	struct imx_rproc *priv = rproc->priv;
	int err;
	__u32 mmsg;

	mmsg = vqid << 16;

	priv->cl.tx_tout = 50;
	err = mbox_send_message(priv->tx_ch, (void *)&mmsg);
	if (err < 0)
		dev_err(priv->dev, "%s: failed (%d, err:%d)\n",
			__func__, vqid, err);
}

static void *imx_rproc_memcpy(struct rproc *rproc, void *dest,
			      const void *src, size_t count, int flags)
{
	u32 *tmp = dest;
	const u32 *s = src;

	count = count / 4;
	while (count--)
		*tmp++ = *s++;

	return dest;
}

static const struct rproc_ops imx_rproc_ops = {
	.start		= imx_rproc_start,
	.stop		= imx_rproc_stop,
	.kick		= imx_rproc_kick,
	.da_to_va       = imx_rproc_da_to_va,
	.load		= imx_rproc_elf_load_segments,
	.parse_fw	= imx_rproc_parse_fw,
	.find_loaded_rsc_table = imx_rproc_elf_find_loaded_rsc_table,
	.sanity_check	= imx_rproc_elf_sanity_check,
	.get_boot_addr	= imx_rproc_elf_get_boot_addr,
	.memcpy		= imx_rproc_memcpy,
};

static int imx_rproc_addr_init(struct imx_rproc *priv,
			       struct platform_device *pdev)
{
	const struct imx_rproc_dcfg *dcfg = priv->dcfg;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int a, b = 0, err, nph;

	/* remap required addresses */
	for (a = 0; a < dcfg->att_size; a++) {
		const struct imx_rproc_att *att = &dcfg->att[a];

		if (!(att->flags & ATT_OWN))
			continue;

		if (att->flags & ATT_CORE_MASK) {
			if (!((1 << priv->id) & (att->flags & ATT_CORE_MASK)))
				continue;
		}

		if (b >= IMX7D_RPROC_MEM_MAX)
			break;

		priv->mem[b].cpu_addr = devm_ioremap(&pdev->dev,
						     att->sa, att->size);
		if (!priv->mem[b].cpu_addr) {
			dev_err(dev, "devm_ioremap_resource failed\n");
			return -ENOMEM;
		}
		priv->mem[b].sys_addr = att->sa;
		priv->mem[b].size = att->size;
		b++;
	}

	/* memory-region is optional property */
	nph = of_count_phandle_with_args(np, "memory-region", NULL);
	if (nph <= 0)
		return 0;

	/* remap optional addresses */
	for (a = 0; a < nph; a++) {
		struct device_node *node;
		struct resource res;

		node = of_parse_phandle(np, "memory-region", a);
		err = of_address_to_resource(node, 0, &res);
		if (err) {
			dev_err(dev, "unable to resolve memory region\n");
			return err;
		}

		if (b >= IMX7D_RPROC_MEM_MAX)
			break;

		/* Not use resource version, because we might share region*/
		priv->mem[b].cpu_addr = devm_ioremap(&pdev->dev, res.start, resource_size(&res));
		if (IS_ERR(priv->mem[b].cpu_addr)) {
			dev_err(dev, "devm_ioremap failed\n");
			err = PTR_ERR(priv->mem[b].cpu_addr);
			return err;
		}
		priv->mem[b].sys_addr = res.start;
		priv->mem[b].size = resource_size(&res);
		b++;
	}

	return 0;
}

static int imx_rproc_configure_mode(struct imx_rproc *priv)
{
	const struct imx_rproc_dcfg *dcfg = priv->dcfg;
	struct device *dev = priv->dev;
	struct arm_smccc_res res;
	int ret;
	u32 val;

	if (dcfg->variant == IMX8QXP || dcfg->variant == IMX8QM) {
		/*
		 * Check whether M4 is owned by Linux. If not owned,
		 * that means M4 is loaded by ROM, kicked by SCU
		 */
#ifdef CONFIG_IMX_SCU
		if (!imx_sc_rm_is_resource_owned(ipc_handle, priv->rsrc)) {
			priv->ipc_only = true;
			priv->early_boot = true;
		}
#endif
	} else if (dcfg->variant == IMX8MN) {
		arm_smccc_smc(IMX_SIP_SRC, IMX_SIP_SRC_M4_STARTED, 0, 0, 0, 0, 0, 0, &res);
		priv->early_boot = !!res.a0;
	} else if (of_get_property(dev->of_node, "ipc-only", NULL)) {
		priv->ipc_only = true;
		priv->early_boot = true;
	} else if (of_get_property(dev->of_node, "early-booted", NULL)) {
		priv->early_boot = true;
	} else {
		if (!priv->regmap)
			return -ENODEV;

		ret = regmap_read(priv->regmap, dcfg->src_reg, &val);
		if (ret) {
			dev_err(dev, "Failed to read src\n");
			return ret;
		}

		priv->early_boot = !(val & dcfg->src_stop);
	}

	if (priv->early_boot)
		priv->rproc->skip_fw_load = true;

	return 0;
}

static void imx_rproc_rxdb_callback(struct mbox_client *cl, void *msg)
{
	struct rproc *rproc = dev_get_drvdata(cl->dev);
	struct imx_rproc *priv = rproc->priv;
	unsigned long flags;

	spin_lock_irqsave(&priv->mu_lock, flags);
	priv->flags |= REMOTE_IS_READY;
	spin_unlock_irqrestore(&priv->mu_lock, flags);
}

static int imx_rproc_db_channel_init(struct rproc *rproc)
{
	struct imx_rproc *priv = rproc->priv;
	struct device *dev = priv->dev;
	struct mbox_client *cl;
	int ret = 0;

	cl = &priv->cl_rxdb;
	cl->dev = dev;
	cl->rx_callback = imx_rproc_rxdb_callback;

	/*
	 * RX door bell is used to receive the ready signal from remote
	 * after the partition reset of A core.
	 */
	priv->rxdb_ch = mbox_request_channel_byname(cl, "rxdb");
	if (IS_ERR(priv->rxdb_ch)) {
		ret = PTR_ERR(priv->rxdb_ch);
		dev_dbg(cl->dev, "failed to request mbox chan rxdb, ret %d\n",
			ret);
		return ret;
	}

	cl = &priv->cl_txdb;
	cl->dev = dev;
	cl->tx_block = true;
	cl->tx_tout = 50;
	cl->knows_txdone = false;

	/* txdb is optional */
	priv->txdb_ch = mbox_request_channel_byname(cl, "txdb");
	if (IS_ERR(priv->txdb_ch)) {
		dev_info(cl->dev, "No txdb, ret %d\n",
			ret);
		priv->txdb_ch = NULL;
	}

	return ret;
}

static void imx_rproc_vq_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct imx_rproc *priv = container_of(dwork, struct imx_rproc,
					      rproc_work);

	/* TODO: take message from rx_callback */
	rproc_vq_interrupt(priv->rproc, 0);
	rproc_vq_interrupt(priv->rproc, 1);
	rproc_vq_interrupt(priv->rproc, 2);
	rproc_vq_interrupt(priv->rproc, 3);
}

static void imx_rproc_rx_callback(struct mbox_client *cl, void *msg)
{
	struct rproc *rproc = dev_get_drvdata(cl->dev);
	struct imx_rproc *priv = rproc->priv;

	schedule_delayed_work(&(priv->rproc_work), 0);
}

static int imx_rproc_xtr_mbox_init(struct rproc *rproc)
{
	struct imx_rproc *priv = rproc->priv;
	struct device *dev = priv->dev;
	struct mbox_client *cl;
	int ret = 0;

	cl = &priv->cl;
	cl->dev = dev;
	cl->tx_block = true;
	cl->tx_tout = 50;
	cl->knows_txdone = false;
	cl->rx_callback = imx_rproc_rx_callback;

	priv->tx_ch = mbox_request_channel_byname(cl, "tx");
	if (IS_ERR(priv->tx_ch)) {
		if (PTR_ERR(priv->tx_ch) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		ret = PTR_ERR(priv->tx_ch);
		dev_dbg(cl->dev, "failed to request mbox tx chan, ret %d\n",
			ret);
		goto err_out;
	}

	priv->rx_ch = mbox_request_channel_byname(cl, "rx");
	if (IS_ERR(priv->rx_ch)) {
		ret = PTR_ERR(priv->rx_ch);
		dev_dbg(cl->dev, "failed to request mbox rx chan, ret %d\n",
			ret);
		goto err_out;
	}

	return ret;

err_out:
	if (!IS_ERR(priv->tx_ch))
		mbox_free_channel(priv->tx_ch);
	if (!IS_ERR(priv->rx_ch))
		mbox_free_channel(priv->rx_ch);

	return ret;
}

#ifdef CONFIG_IMX_SCU
static int imx_rproc_partition_notify(struct notifier_block *nb,
				      unsigned long event, void *group)
{
	struct imx_rproc *priv = container_of(nb, struct imx_rproc, proc_nb);

	/* Ignore other irqs */
	if (!((event & BIT(priv->mub_partition)) &&
	    (*(u8 *)group == 5)))
		return 0;

	priv->skip_fw_load_recovery = true;

	rproc_report_crash(priv->rproc, RPROC_WATCHDOG);

	pr_info("Patition%d reset!\n", priv->mub_partition);

	return 0;
}
#endif

static int imx_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct imx_rproc *priv;
	struct rproc *rproc;
	struct regmap_config config = { .name = "imx-rproc" };
	const struct imx_rproc_dcfg *dcfg;
	struct regmap *regmap = NULL;
	struct reset_control *non_sclr_rst, *enable;
	const char *fw_name = NULL;
	int ret;
	int i __maybe_unused;

	regmap = syscon_regmap_lookup_by_phandle(np, "syscon");
	if (IS_ERR(regmap)) {
		dev_err(dev, "failed to find syscon\n");
		regmap = NULL;
	} else {
		regmap_attach_dev(dev, regmap, &config);
	}

	non_sclr_rst = devm_reset_control_get_optional(dev, "non_sclr_rst");
	if (IS_ERR(non_sclr_rst)) {
		if (PTR_ERR(non_sclr_rst) == -EPROBE_DEFER)
			return PTR_ERR(non_sclr_rst);
	}

	enable = devm_reset_control_get_optional(dev, "enable");
	if (IS_ERR(enable))
		return PTR_ERR(enable);

	of_property_read_string(np, "fsl,rproc-fw-name", &fw_name);

	/* set some other name then imx */
	rproc = rproc_alloc(dev, "imx-rproc", &imx_rproc_ops,
			    fw_name, sizeof(*priv));
	if (!rproc)
		return -ENOMEM;

	dcfg = of_device_get_match_data(dev);
	if (!dcfg) {
		ret = -EINVAL;
		goto err_put_rproc;
	}

	priv = rproc->priv;
	priv->rproc = rproc;
	priv->regmap = regmap;
	priv->non_sclr_rst = non_sclr_rst;
	priv->enable = enable;
	priv->dcfg = dcfg;
	priv->dev = dev;
	priv->rsc_va = NULL;

	dev_set_drvdata(dev, rproc);

	spin_lock_init(&priv->mu_lock);

	INIT_DELAYED_WORK(&(priv->rproc_work), imx_rproc_vq_work);

	ret = imx_rproc_xtr_mbox_init(rproc);
	if (ret) {
		if (ret == -EPROBE_DEFER)
			goto err_put_rproc;
	}

	priv->clk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(priv->clk)) {
		dev_err(dev, "Failed to get clock\n");
		ret = PTR_ERR(priv->clk);
		goto err_put_rproc;
	}

	/*
	 * clk for M4 block including memory. Should be
	 * enabled before .start for FW transfer.
	 */
	ret = clk_prepare_enable(priv->clk);
	if (ret) {
		dev_err(&rproc->dev, "Failed to enable clock\n");
		goto err_put_rproc;
	}

	ret = imx_rproc_db_channel_init(rproc);
	if (ret)
		goto err_put_mbox;

#ifdef CONFIG_IMX_SCU
	priv->proc_nb.notifier_call = imx_rproc_partition_notify;


	priv->num_domains = of_count_phandle_with_args(dev->of_node,
						       "power-domains",
						       "#power-domain-cells");
	if (priv->num_domains < 0)
		priv->num_domains = 0;

	if (dcfg->variant == IMX8QXP || dcfg->variant == IMX8QM) {
		ret = imx_scu_get_handle(&ipc_handle);
		if (ret)
			goto err_put_mbox;

		ret = of_property_read_u32(np, "core-id", &priv->rsrc);
		if (ret) {
			dev_err(&rproc->dev, "No reg <core resource id>\n");
			goto err_put_mbox;
		}

		ret = of_property_read_u32(np, "core-index", &priv->id);
		if (ret) {
			dev_err(&rproc->dev, "No reg <core index id>\n");
			goto err_put_mbox;
		}

		/*
		 * Get muB partition id and enable irq in SCFW
		 * default partition 3
		 */
		if (of_property_read_u32(np, "mub-partition",
					 &priv->mub_partition))
			priv->mub_partition = 3;

		ret = imx_scu_irq_group_enable(IMX_SC_IRQ_GROUP_REBOOTED, BIT(priv->mub_partition),
					       true);
		if (ret) {
			dev_warn(dev, "Enable irq failed.\n");
			goto err_put_clk;
		}

		ret = imx_scu_irq_register_notifier(&priv->proc_nb);
		if (ret) {
			imx_scu_irq_group_enable(IMX_SC_IRQ_GROUP_REBOOTED, BIT(priv->mub_partition),
						 false);
			dev_warn(dev, "reqister scu notifier failed.\n");
			goto err_put_clk;
		}

		if (priv->num_domains) {
			priv->pm_devices = devm_kcalloc(dev, priv->num_domains,
							sizeof(struct device),
							GFP_KERNEL);
			if (!priv->pm_devices)
				goto err_put_clk;
			priv->pm_devices_link = devm_kcalloc(dev,
							     priv->num_domains,
							     sizeof(struct device_link),
							     GFP_KERNEL);
			if (!priv->pm_devices)
				goto err_put_clk;

			for (i = 0; i < priv->num_domains; i++) {
				priv->pm_devices[i] =
					genpd_dev_pm_attach_by_id(dev, i);
				if (!priv->pm_devices[i])
					goto err_put_scu;
				priv->pm_devices_link[i] =
					device_link_add(dev, priv->pm_devices[i],
							DL_FLAG_RPM_ACTIVE |
							DL_FLAG_PM_RUNTIME |
							DL_FLAG_STATELESS);
				if (!priv->pm_devices_link[i])
					goto err_put_scu;
			}
		}
	}
#endif
	ret = imx_rproc_configure_mode(priv);
	if (ret)
		goto err_put_scu;

	ret = imx_rproc_addr_init(priv, pdev);
	if (ret) {
		dev_err(dev, "failed on imx_rproc_addr_init\n");
		goto err_put_scu;
	}

	rproc->auto_boot = of_property_read_bool(np, "fsl,rproc-auto-boot");

	ret = rproc_add(rproc);
	if (ret) {
		dev_err(dev, "rproc_add failed\n");
		goto err_put_scu;
	}

	return 0;

err_put_scu:
#ifdef CONFIG_IMX_SCU
	for (i = 0; i < priv->num_domains; i++) {
		if (priv->pm_devices_link[i])
			device_link_del(priv->pm_devices_link[i]);
		if (priv->pm_devices[i])
			dev_pm_domain_detach(priv->pm_devices[i], true);
	}
	imx_scu_irq_group_enable(5, BIT(priv->mub_partition), false);
err_put_clk:
#endif
	if (!priv->early_boot)
		clk_disable_unprepare(priv->clk);
err_put_mbox:
	if (!IS_ERR(priv->tx_ch))
		mbox_free_channel(priv->tx_ch);
	if (!IS_ERR(priv->rx_ch))
		mbox_free_channel(priv->rx_ch);
err_put_rproc:
	rproc_free(rproc);

	return ret;
}

static int imx_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct imx_rproc *priv = rproc->priv;

	if (!priv->early_boot)
		clk_disable_unprepare(priv->clk);
	rproc_del(rproc);
	rproc_free(rproc);

	return 0;
}

static const struct of_device_id imx_rproc_of_match[] = {
	{ .compatible = "fsl,imx7ulp-cm4", .data = &imx_rproc_cfg_imx7ulp },
	{ .compatible = "fsl,imx7d-cm4", .data = &imx_rproc_cfg_imx7d },
	{ .compatible = "fsl,imx6sx-cm4", .data = &imx_rproc_cfg_imx6sx },
	{ .compatible = "fsl,imx8mq-cm4", .data = &imx_rproc_cfg_imx8mq },
	{ .compatible = "fsl,imx8mm-cm4", .data = &imx_rproc_cfg_imx8mq },
	{ .compatible = "fsl,imx8mn-cm7", .data = &imx_rproc_cfg_imx8mn },
	{ .compatible = "fsl,imx8mp-cm7", .data = &imx_rproc_cfg_imx8mn },
	{ .compatible = "fsl,imx8qxp-cm4", .data = &imx_rproc_cfg_imx8qxp },
	{ .compatible = "fsl,imx8qm-cm4", .data = &imx_rproc_cfg_imx8qm },
	{},
};
MODULE_DEVICE_TABLE(of, imx_rproc_of_match);

static struct platform_driver imx_rproc_driver = {
	.probe = imx_rproc_probe,
	.remove = imx_rproc_remove,
	.driver = {
		.name = "imx-rproc",
		.of_match_table = imx_rproc_of_match,
	},
};

module_platform_driver(imx_rproc_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("IMX6SX/7D remote processor control driver");
MODULE_AUTHOR("Oleksij Rempel <o.rempel@pengutronix.de>");
