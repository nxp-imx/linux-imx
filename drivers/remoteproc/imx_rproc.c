// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017 Pengutronix, Oleksij Rempel <kernel@pengutronix.de>
 */

#include <dt-bindings/firmware/imx/rsrc.h>
#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/firmware/imx/sci.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_client.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/regmap.h>
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
#define IMX7D_M4_STOP			IMX7D_SW_M4C_NON_SCLR_RST

/* Address: 0x020D8000 */
#define IMX6SX_SRC_SCR			0x00
#define IMX6SX_ENABLE_M4		BIT(22)
#define IMX6SX_SW_M4P_RST		BIT(12)
#define IMX6SX_SW_M4C_NON_SCLR_RST	BIT(4)
#define IMX6SX_SW_M4C_RST		BIT(3)

#define IMX6SX_M4_START			(IMX6SX_ENABLE_M4 | IMX6SX_SW_M4P_RST \
					 | IMX6SX_SW_M4C_RST)
#define IMX6SX_M4_STOP			IMX6SX_SW_M4C_NON_SCLR_RST
#define IMX6SX_M4_RST_MASK		(IMX6SX_ENABLE_M4 | IMX6SX_SW_M4P_RST \
					 | IMX6SX_SW_M4C_NON_SCLR_RST \
					 | IMX6SX_SW_M4C_RST)

#define IMX7D_RPROC_MEM_MAX		16

#define REMOTE_IS_READY			BIT(0)
#define REMOTE_READY_WAIT_MAX_RETRIES	500

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

enum imx_rproc_method {
	IMX_DIRECT_MMIO,
	IMX_ARM_SMCCC,
	IMX_IPC_ONLY,
	IMX_SCU_API,
};

struct imx_rproc_dcfg {
	u32				src_reg;
	u32				src_mask;
	u32				src_start;
	u32				src_stop;
	const struct imx_rproc_att	*att;
	size_t				att_size;
	bool				elf_mem_hook;
	enum imx_rproc_method		method;
};

struct imx_rproc {
	struct device			*dev;
	struct regmap			*regmap;
	struct rproc			*rproc;
	const struct imx_rproc_dcfg	*dcfg;
	struct imx_rproc_mem		mem[IMX7D_RPROC_MEM_MAX];
	struct clk			*clk;
	bool				early_boot;
	bool				ipc_only;
	void				*rsc_va;
	struct mbox_client		cl;
	struct mbox_client		cl_rxdb;
	struct mbox_client		cl_txdb;
	struct mbox_chan		*tx_ch;
	struct mbox_chan		*rx_ch;
	struct mbox_chan		*rxdb_ch;
	struct mbox_chan		*txdb_ch;
	u32				mub_partition;
	struct notifier_block		proc_nb;
	u32				flags;
	spinlock_t			mu_lock;
	struct delayed_work		rproc_work;
	u32				rsrc;
	u32				id;
	int				num_domains;
	struct device			**pm_devices;
	struct device_link		**pm_devices_link;
};

static struct imx_sc_ipc *ipc_handle;

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
	.elf_mem_hook	= true,
	.method		= IMX_ARM_SMCCC,
};

static const struct imx_rproc_dcfg imx_rproc_cfg_imx8mq = {
	.src_reg	= IMX7D_SRC_SCR,
	.src_mask	= IMX7D_M4_RST_MASK,
	.src_start	= IMX7D_M4_START,
	.src_stop	= IMX7D_M4_STOP,
	.att		= imx_rproc_att_imx8mq,
	.att_size	= ARRAY_SIZE(imx_rproc_att_imx8mq),
	.elf_mem_hook	= true,
	.method		= IMX_DIRECT_MMIO,
};

static const struct imx_rproc_dcfg imx_rproc_cfg_imx7ulp = {
	.att		= imx_rproc_att_imx7ulp,
	.att_size	= ARRAY_SIZE(imx_rproc_att_imx7ulp),
	.method		= IMX_IPC_ONLY,
};

static const struct imx_rproc_dcfg imx_rproc_cfg_imx7d = {
	.src_reg	= IMX7D_SRC_SCR,
	.src_mask	= IMX7D_M4_RST_MASK,
	.src_start	= IMX7D_M4_START,
	.src_stop	= IMX7D_M4_STOP,
	.att		= imx_rproc_att_imx7d,
	.att_size	= ARRAY_SIZE(imx_rproc_att_imx7d),
	.method		= IMX_DIRECT_MMIO,
};

static const struct imx_rproc_dcfg imx_rproc_cfg_imx6sx = {
	.src_reg	= IMX6SX_SRC_SCR,
	.src_mask	= IMX6SX_M4_RST_MASK,
	.src_start	= IMX6SX_M4_START,
	.src_stop	= IMX6SX_M4_STOP,
	.att		= imx_rproc_att_imx6sx,
	.att_size	= ARRAY_SIZE(imx_rproc_att_imx6sx),
	.method		= IMX_DIRECT_MMIO,
};

static const struct imx_rproc_dcfg imx_rproc_cfg_imx8qxp = {
	.att		= imx_rproc_att_imx8qxp,
	.att_size	= ARRAY_SIZE(imx_rproc_att_imx8qxp),
	.method		= IMX_SCU_API,
};

static const struct imx_rproc_dcfg imx_rproc_cfg_imx8qm = {
	.att		= imx_rproc_att_imx8qm,
	.att_size	= ARRAY_SIZE(imx_rproc_att_imx8qm),
	.method		= IMX_SCU_API,
};


static int imx_rproc_ready(struct rproc *rproc)
{
	struct imx_rproc *priv = rproc->priv;
	int i;

	if (!priv->rxdb_ch)
		return 0;

	for (i = 0; i < REMOTE_READY_WAIT_MAX_RETRIES; i++) {
		if (priv->flags & REMOTE_IS_READY)
			return 0;
		udelay(100);
	}

	/* Not return -ETIMEOUT, remote processor might not implement doorbell */
	return 0;
}

static int imx_rproc_start(struct rproc *rproc)
{
	struct imx_rproc *priv = rproc->priv;
	const struct imx_rproc_dcfg *dcfg = priv->dcfg;
	struct device *dev = priv->dev;
	struct arm_smccc_res res;
	int ret;

	switch (dcfg->method) {
	case IMX_DIRECT_MMIO:
		ret = regmap_update_bits(priv->regmap, dcfg->src_reg,
					 dcfg->src_mask, dcfg->src_start);
		break;
	case IMX_ARM_SMCCC:
		arm_smccc_smc(IMX_SIP_SRC, IMX_SIP_SRC_M4_START, 0, 0, 0, 0, 0, 0, &res);
		ret = res.a0;
		break;
	case IMX_SCU_API:
		if (priv->ipc_only)
			return imx_rproc_ready(rproc);
		if (priv->id == 1)
			ret = imx_sc_pm_cpu_start(ipc_handle, priv->rsrc, true, 0x38fe0000);
		else if (!priv->id)
			ret = imx_sc_pm_cpu_start(ipc_handle, priv->rsrc, true, 0x34fe0000);
		else
			ret = -EINVAL;
		break;
	case IMX_IPC_ONLY:
		return -ENOTSUPP;
	default:
		return -ENOENT;
	}

	if (ret)
		dev_err(dev, "Failed to enable M4!\n");
	else
		ret = imx_rproc_ready(rproc);

	return ret;
}

static int imx_rproc_stop(struct rproc *rproc)
{
	struct imx_rproc *priv = rproc->priv;
	const struct imx_rproc_dcfg *dcfg = priv->dcfg;
	struct device *dev = priv->dev;
	struct arm_smccc_res res;
	int ret = 0;
	__u32 mmsg;

	if (rproc->state == RPROC_CRASHED && priv->ipc_only) {
		priv->flags &= ~REMOTE_IS_READY;
		return 0;
	}

	if (dcfg->method == IMX_IPC_ONLY)
		return -ENOTSUPP;


	if (priv->txdb_ch) {
		ret = mbox_send_message(priv->txdb_ch, (void *)&mmsg);
		if (ret) {
			dev_err(dev, "txdb send fail: %d\n", ret);
			return ret;
		}
	}

	switch (dcfg->method) {
	case IMX_DIRECT_MMIO:
		ret = regmap_update_bits(priv->regmap, dcfg->src_reg,
					 dcfg->src_mask, dcfg->src_stop);
		break;
	case IMX_ARM_SMCCC:
		arm_smccc_smc(IMX_SIP_SRC, IMX_SIP_SRC_M4_STOP, 0, 0, 0, 0, 0, 0, &res);
		ret = res.a0;
		if (res.a1)
			dev_info(dev, "remotecore not run into wfi, force stop: %ld %ld %ld\n", res.a0, res.a1, res.a2);
		break;
	case IMX_SCU_API:
		if (priv->id == 1)
			ret = imx_sc_pm_cpu_start(ipc_handle, priv->rsrc, false, 0x38fe0000);
		else if (!priv->id)
			ret = imx_sc_pm_cpu_start(ipc_handle, priv->rsrc, false, 0x34fe0000);
		else
			ret = -EINVAL;
		break;
	default:
		return -ENOENT;
	}

	if (ret) {
		dev_err(dev, "Failed to stop M4!\n");
	} else {
		priv->early_boot = false;
		priv->flags &= ~REMOTE_IS_READY;
	}

	return ret;
}

static int imx_rproc_da_to_sys(struct imx_rproc *priv, u64 da,
			       size_t len, u64 *sys)
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

	dev_warn(priv->dev, "Translation failed: da = 0x%llx len = 0x%zx\n",
		 da, len);
	return -ENOENT;
}

static void *imx_rproc_da_to_va(struct rproc *rproc, u64 da, size_t len, bool *is_iomem)
{
	struct imx_rproc *priv = rproc->priv;
	void *va = NULL;
	u64 sys;
	int i;

	if (len == 0)
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

	dev_dbg(&rproc->dev, "da = 0x%llx len = 0x%zx va = 0x%p\n",
		da, len, va);

	return va;
}

static int imx_rproc_mem_alloc(struct rproc *rproc,
			       struct rproc_mem_entry *mem)
{
	struct device *dev = rproc->dev.parent;
	void *va;

	dev_dbg(dev, "map memory: %p+%zx\n", &mem->dma, mem->len);
	va = ioremap_wc(mem->dma, mem->len);
	if (IS_ERR_OR_NULL(va)) {
		dev_err(dev, "Unable to map memory region: %p+%zx\n",
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

static int imx_rproc_parse_memory_regions(struct rproc *rproc)
{
	struct imx_rproc *priv = rproc->priv;
	struct device_node *np = priv->dev->of_node;
	struct of_phandle_iterator it;
	struct rproc_mem_entry *mem;
	struct reserved_mem *rmem;
	int index = 0;
	u32 da;

	/* Register associated reserved memory regions */
	of_phandle_iterator_init(&it, np, "memory-region", NULL, 0);
	while (of_phandle_iterator_next(&it) == 0) {
		/*
		 * Ignore the first memory region which will be used vdev buffer.
		 * No need to do extra handlings, rproc_add_virtio_dev will handle it.
		 */
		if (!index && !strcmp(it.node->name, "vdevbuffer")) {
			index ++;
			continue;
		}

		rmem = of_reserved_mem_lookup(it.node);
		if (!rmem) {
			dev_err(priv->dev, "unable to acquire memory-region\n");
			return -EINVAL;
		}

		/* No need to translate pa to da, i.MX use same map */
		da = rmem->base;

		if (!strncmp(it.node->name, "rsc_table", strlen("rsc_table"))) {
			if (priv->rsc_va && priv->early_boot) {
				dev_err(priv->dev, "Found duplicated rsc_table\n");
				return -EINVAL;
			}
			priv->rsc_va = rproc_da_to_va(rproc, (u64)da, SZ_1K, NULL);
			if (!priv->rsc_va) {
				dev_err(priv->dev, "no map for rsc_table: %x\n", da);
				return -EINVAL;
			}

			continue;
		}

		/* Register memory region */
		mem = rproc_mem_entry_init(priv->dev, NULL, (dma_addr_t)rmem->base, rmem->size, da,
					   imx_rproc_mem_alloc, imx_rproc_mem_release,
					   it.node->name);

		if (mem)
			rproc_coredump_add_segment(rproc, da, rmem->size);
		else
			return -ENOMEM;

		rproc_add_carveout(rproc, mem);
		index++;
	}

	return  0;
}

static int imx_rproc_parse_fw(struct rproc *rproc, const struct firmware *fw)
{
	int ret = imx_rproc_parse_memory_regions(rproc);

	if (ret)
		return ret;

	ret = rproc_elf_load_rsc_table(rproc, fw);
	if (ret)
		dev_info(&rproc->dev, "No resource table in elf\n");

	return 0;
}

static int imx_rproc_get_loaded_rsc_table(struct device *dev,
					  struct rproc *rproc)
{
	struct imx_rproc *priv = rproc->priv;

	if (!priv->rsc_va)
		return 0;

#if 0
	rproc->table_ptr = (struct resource_table *)priv->rsc_va;
#else
	/*
	 * This is a hack workaround, this will not let M4 detect vdev status,
	 * because vring is conflict with resource table,
	 * because NXP M4 SDK not detect vdev status update, so we just use a copied
	 * table here, for future, m4 need use a new address for publishing resource table
	 * Then we could change to
	 * rproc->table_ptr = (struct resource_table *)priv->rsc_va;
	 */
	rproc->table_ptr = kmemdup(priv->rsc_va, SZ_1K, GFP_KERNEL);
#endif
	rproc->table_sz = SZ_1K;
	rproc->cached_table = NULL;

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

static void imx_rproc_kick(struct rproc *rproc, int vqid)
{
	struct imx_rproc *priv = rproc->priv;
	int err;
	__u32 mmsg;

	if (!priv->tx_ch) {
		dev_err(priv->dev, "No initialized mbox tx channel\n");
		return;
	}

	mmsg = vqid << 16;

	priv->cl.tx_tout = 100;
	err = mbox_send_message(priv->tx_ch, (void *)&mmsg);
	if (err < 0)
		dev_err(priv->dev, "%s: failed (%d, err:%d)\n",
			__func__, vqid, err);
}

static int imx_rproc_attach(struct rproc *rproc)
{
	return 0;
}

static int imx_rproc_elf_load_segments(struct rproc *rproc,
				       const struct firmware *fw)
{
	struct imx_rproc *priv = rproc->priv;

	if (priv->ipc_only || !fw)
		return -EINVAL;

	return rproc_elf_load_segments(rproc, fw);
}

static struct resource_table *
imx_rproc_elf_find_loaded_rsc_table(struct rproc *rproc, const struct firmware *fw)
{
	struct imx_rproc *priv = rproc->priv;


	if (priv->ipc_only)
		return NULL;

#if 0
	/*
	 * We not return this currently, because vring conflicts with resource table on NXP
	 * i.MX M4 SDK
	 */
	if (priv->rsc_va)
		return priv->rsc_va;
#endif

	return rproc_elf_find_loaded_rsc_table(rproc, fw);
}

static const struct rproc_ops imx_rproc_ops = {
	.start		= imx_rproc_start,
	.stop		= imx_rproc_stop,
	.attach		= imx_rproc_attach,
	.kick		= imx_rproc_kick,
	.da_to_va       = imx_rproc_da_to_va,
	.load		= imx_rproc_elf_load_segments,
	.parse_fw	= imx_rproc_parse_fw,
	.find_loaded_rsc_table = imx_rproc_elf_find_loaded_rsc_table,
	.sanity_check	= rproc_elf_sanity_check,
	.get_boot_addr	= rproc_elf_get_boot_addr,
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
			dev_err(dev, "devm_ioremap failed\n");
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
			dev_err(dev, "devm_ioremap %pR failed\n", &res);
			err = PTR_ERR(priv->mem[b].cpu_addr);
			return err;
		}
		priv->mem[b].sys_addr = res.start;
		priv->mem[b].size = resource_size(&res);
		b++;
	}

	return 0;
}

static void imx_rproc_memcpy(struct rproc *rproc, void *dest, const void *src, size_t count)
{
	memcpy_toio((void * __iomem)dest, src, count);
}

static void imx_rproc_memset(struct rproc *rproc, void *s, int c, size_t count)
{
	memset_io((void * __iomem)s, c, count);
}

static void imx_rproc_vq_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct imx_rproc *priv = container_of(dwork, struct imx_rproc,
					      rproc_work);

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
	int ret;

	if (!of_get_property(dev->of_node, "mbox-names", NULL))
		return 0;

	spin_lock_init(&priv->mu_lock);

	cl = &priv->cl;
	cl->dev = dev;
	cl->tx_block = true;
	cl->tx_tout = 20;
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
		goto err_out;
	}

	cl = &priv->cl_txdb;
	cl->dev = dev;
	cl->tx_block = true;
	cl->tx_tout = 20;
	cl->knows_txdone = false;

	/* txdb is optional */
	priv->txdb_ch = mbox_request_channel_byname(cl, "txdb");
	if (IS_ERR(priv->txdb_ch)) {
		ret = PTR_ERR(priv->txdb_ch);
		dev_info(cl->dev, "No txdb, ret %d\n", ret);
		priv->txdb_ch = NULL;
	}

	return 0;

err_out:
	if (!IS_ERR(priv->tx_ch))
		mbox_free_channel(priv->tx_ch);
	if (!IS_ERR(priv->rx_ch))
		mbox_free_channel(priv->rx_ch);
	if (!IS_ERR(priv->rxdb_ch))
		mbox_free_channel(priv->rxdb_ch);

	return ret;
}

static int imx_rproc_partition_notify(struct notifier_block *nb,
				      unsigned long event, void *group)
{
	struct imx_rproc *priv = container_of(nb, struct imx_rproc, proc_nb);

	/* Ignore other irqs */
	if (!((event & BIT(priv->mub_partition)) && (*(u8 *)group == 5)))
		return 0;

	rproc_report_crash(priv->rproc, RPROC_WATCHDOG);

	pr_info("Patition%d reset!\n", priv->mub_partition);

	return 0;
}

static int imx_rproc_detect_mode(struct imx_rproc *priv)
{
	const struct imx_rproc_dcfg *dcfg = priv->dcfg;
	struct device *dev = priv->dev;
	struct arm_smccc_res res;
	int ret;
	u32 val;
	int i;

	switch (dcfg->method) {
	case IMX_DIRECT_MMIO:
		ret = regmap_read(priv->regmap, dcfg->src_reg, &val);
		if (ret) {
			dev_err(dev, "Failed to read src\n");
			return ret;
		}
		priv->early_boot = !(val & dcfg->src_stop);
		break;
	case IMX_ARM_SMCCC:
		arm_smccc_smc(IMX_SIP_SRC, IMX_SIP_SRC_M4_STARTED, 0, 0, 0, 0, 0, 0, &res);
		priv->early_boot = !!res.a0;
		break;
	case IMX_IPC_ONLY:
		priv->early_boot = true;
		break;
	case IMX_SCU_API:
		ret = imx_scu_get_handle(&ipc_handle);
		if (ret)
			return ret;
		ret = of_property_read_u32(dev->of_node, "core-id", &priv->rsrc);
		if (ret) {
			dev_err(dev, "No reg <core resource id>\n");
			return ret;
		}
		ret = of_property_read_u32(dev->of_node, "core-index", &priv->id);
		if (ret) {
			dev_err(dev, "No reg <core index id>\n");
			return ret;
		}

		priv->proc_nb.notifier_call = imx_rproc_partition_notify;


		priv->num_domains = of_count_phandle_with_args(dev->of_node, "power-domains",
							       "#power-domain-cells");
		if (priv->num_domains < 0)
			priv->num_domains = 0;

		if (priv->num_domains) {
			priv->pm_devices = devm_kcalloc(dev, priv->num_domains,
							sizeof(struct device), GFP_KERNEL);
			if (!priv->pm_devices)
				return -ENOMEM;
			priv->pm_devices_link = devm_kcalloc(dev, priv->num_domains,
							     sizeof(struct device_link),
							     GFP_KERNEL);
			if (!priv->pm_devices)
				return -ENOMEM;

			for (i = 0; i < priv->num_domains; i++) {
				priv->pm_devices[i] = genpd_dev_pm_attach_by_id(dev, i);
				if (IS_ERR(priv->pm_devices[i]))
					goto err_put_pd;
				priv->pm_devices_link[i] = device_link_add(dev, priv->pm_devices[i],
									   DL_FLAG_RPM_ACTIVE |
									   DL_FLAG_PM_RUNTIME |
									   DL_FLAG_STATELESS);
				if (IS_ERR(priv->pm_devices_link[i]))
					goto err_put_pd;
			}
		}
		if (!imx_sc_rm_is_resource_owned(ipc_handle, priv->rsrc)) {
			priv->ipc_only = true;
			priv->early_boot = true;
			priv->rproc->skip_fw_recovery = true;
			/*
			 * Get muB partition id and enable irq in SCFW
			 * default partition 3
			 */
			if (of_property_read_u32(dev->of_node, "mub-partition",
						 &priv->mub_partition))
				priv->mub_partition = 3;

			ret = imx_scu_irq_group_enable(5, BIT(priv->mub_partition), true);
			if (ret) {
				dev_warn(dev, "Enable irq failed.\n");
				goto err_put_pd;
			}

			ret = imx_scu_irq_register_notifier(&priv->proc_nb);
			if (ret) {
				imx_scu_irq_group_enable(5, BIT(priv->mub_partition), false);
				dev_warn(dev, "reqister scu notifier failed.\n");
				goto err_put_pd;
			}
		}
		break;
	default:
		return -ENOENT;
	}

	if (priv->early_boot) {
		priv->rproc->state = RPROC_DETACHED;

		ret = imx_rproc_parse_memory_regions(priv->rproc);
		if (ret)
			return ret;

		ret = imx_rproc_get_loaded_rsc_table(dev, priv->rproc);
		if (ret)
			return ret;
	}

	return 0;

err_put_pd:
	for (i = 0; i < priv->num_domains; i++) {
		if (priv->pm_devices_link[i])
			device_link_del(priv->pm_devices_link[i]);
		if (priv->pm_devices[i])
			dev_pm_domain_detach(priv->pm_devices[i], true);
	}
	return ret;
}

static int imx_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct imx_rproc *priv;
	struct rproc *rproc;
	struct regmap_config config = { .name = "imx-rproc" };
	const struct imx_rproc_dcfg *dcfg;
	struct regmap *regmap;
	int ret;

	/* set some other name then imx */
	rproc = rproc_alloc(dev, "imx-rproc", &imx_rproc_ops,
			    NULL, sizeof(*priv));
	if (!rproc)
		return -ENOMEM;

	dcfg = of_device_get_match_data(dev);
	if (!dcfg) {
		ret = -EINVAL;
		goto err_put_rproc;
	}

	if (dcfg->elf_mem_hook) {
		rproc->ops->elf_memcpy = imx_rproc_memcpy;
		rproc->ops->elf_memset = imx_rproc_memset;
	}

	priv = rproc->priv;
	priv->rproc = rproc;
	priv->dcfg = dcfg;
	priv->dev = dev;

	if (dcfg->method == IMX_DIRECT_MMIO) {
		regmap = syscon_regmap_lookup_by_phandle(np, "syscon");
		if (IS_ERR(regmap)) {
			dev_err(dev, "failed to find syscon\n");
			ret = PTR_ERR(regmap);
			goto err_put_rproc;
		}
		regmap_attach_dev(dev, regmap, &config);
		priv->regmap = regmap;
	}

	dev_set_drvdata(dev, rproc);

	ret = imx_rproc_xtr_mbox_init(rproc);
	if (ret) {
		if (ret == -EPROBE_DEFER)
			goto err_put_rproc;
		/* mbox is optional, so not fail here */
	}

	ret = imx_rproc_addr_init(priv, pdev);
	if (ret) {
		dev_err(dev, "failed on imx_rproc_addr_init\n");
		goto err_put_mbox;
	}

	ret = imx_rproc_detect_mode(priv);
	if (ret)
		goto err_put_mbox;

	priv->clk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(priv->clk)) {
		dev_err(dev, "Failed to get clock\n");
		ret = PTR_ERR(priv->clk);
		goto err_put_mbox;
	}

	/*
	 * clk for M4 block including memory. Should be
	 * enabled before .start for FW transfer.
	 */
	ret = clk_prepare_enable(priv->clk);
	if (ret) {
		dev_err(&rproc->dev, "Failed to enable clock\n");
		goto err_put_mbox;
	}

	INIT_DELAYED_WORK(&(priv->rproc_work), imx_rproc_vq_work);

	rproc->auto_boot = false;
	if (priv->early_boot)
		rproc->auto_boot = true;

	ret = rproc_add(rproc);
	if (ret) {
		dev_err(dev, "rproc_add failed\n");
		goto err_put_clk;
	}

	return 0;

err_put_clk:
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
