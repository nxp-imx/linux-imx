// SPDX-License-Identifier: GPL-2.0-only
// Copyright 2021 NXP

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/interrupt.h>
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
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/workqueue.h>
#include <linux/firmware/imx/sci.h>
#include <dt-bindings/firmware/imx/rsrc.h>
#include <linux/arm-smccc.h>

#include "remoteproc_internal.h"
#include "remoteproc_elf_helpers.h"

enum imx_dsp_rp_mbox_messages {
	RP_MBOX_SUSPEND_SYSTEM  = 0xFF11,
	RP_MBOX_SUSPEND_ACK     = 0xFF12,
	RP_MBOX_RESUME_SYSTEM   = 0xFF13,
	RP_MBOX_RESUME_ACK      = 0xFF14,
};

#define DSP_RPROC_CLK_MAX 32
#define DSP_RPROC_MEM_MAX 32

#define REMOTE_IS_READY			BIT(0)
#define REMOTE_READY_WAIT_MAX_RETRIES	500

/* att flags */
/* DSP own area. Can be mapped at probe */
#define ATT_OWN		BIT(31)
#define ATT_IRAM	BIT(30)

/* i.MX8MP */
/* DAP registers */
#define IMX8M_DAP_DEBUG                0x28800000
#define IMX8M_DAP_DEBUG_SIZE           (64 * 1024)
#define IMX8M_DAP_PWRCTL               (0x4000 + 0x3020)
#define IMX8M_PWRCTL_CORERESET         BIT(16)

/* DSP audio mix registers */
#define IMX8M_AudioDSP_REG0   0x100
#define IMX8M_AudioDSP_REG1   0x104
#define IMX8M_AudioDSP_REG2   0x108
#define IMX8M_AudioDSP_REG3   0x10c

#define IMX8M_AudioDSP_REG2_RUNSTALL  BIT(5)
#define IMX8M_AudioDSP_REG2_PWAITMODE BIT(1)

/* i.MX8ULP */
#define IMX8ULP_SIM_LPAV_REG_SYSCTRL0       0x8
#define IMX8ULP_SYSCTRL0_DSP_DBG_RST        BIT(25)
#define IMX8ULP_SYSCTRL0_DSP_PLAT_CLK_EN    BIT(19)
#define IMX8ULP_SYSCTRL0_DSP_PBCLK_EN       BIT(18)
#define IMX8ULP_SYSCTRL0_DSP_CLK_EN         BIT(17)
#define IMX8ULP_SYSCTRL0_DSP_RST            BIT(16)
#define IMX8ULP_SYSCTRL0_DSP_OCD_HALT       BIT(14)
#define IMX8ULP_SYSCTRL0_DSP_STALL          BIT(13)

#define IMX8ULP_SIP_HIFI_XRDC         0xc200000e

/*
 * struct imx_dsp_rproc_mem - slim internal memory structure
 * @cpu_addr: MPU virtual address of the memory region
 * @sys_addr: Bus address used to access the memory region
 * @size: Size of the memory region
 */
struct imx_dsp_rproc_mem {
	void __iomem *cpu_addr;
	phys_addr_t sys_addr;
	size_t size;
};

/* address translation table */
struct imx_dsp_rproc_att {
	u32 da;	/* device address (From Cortex M4 view)*/
	u32 sa;	/* system bus address */
	u32 size; /* size of reg range */
	int flags;
};

/* Remote core start/stop method */
enum imx_dsp_rproc_method {
	/* Through syscon regmap */
	IMX_DSP_MMIO,
	IMX_DSP_SCU_API,
};

struct imx_dsp_rproc {
	struct device			*dev;
	struct regmap			*regmap;
	struct rproc			*rproc;
	const struct imx_dsp_rproc_dcfg	*dcfg;
	struct imx_dsp_rproc_mem	mem[DSP_RPROC_MEM_MAX];
	struct clk			*clks[DSP_RPROC_CLK_MAX];
	struct mbox_client		cl;
	struct mbox_client		cl_rxdb;
	struct mbox_chan		*tx_ch;
	struct mbox_chan		*rx_ch;
	struct mbox_chan		*rxdb_ch;
	struct device			**pd_dev;
	struct device_link		**pd_dev_link;
	struct imx_sc_ipc		*ipc_handle;
	struct work_struct		rproc_work;
	struct workqueue_struct		*workqueue;
	struct completion		pm_comp;
	spinlock_t			mbox_lock;    /* lock for mbox */
	int				num_domains;
	u32				flags;
};

struct imx_dsp_rproc_dcfg {
	u32				src_reg;
	u32				src_mask;
	u32				src_start;
	u32				src_stop;
	const struct imx_dsp_rproc_att	*att;
	size_t				att_size;
	enum imx_dsp_rproc_method	method;
	int (*reset)(struct imx_dsp_rproc *priv);
};

static const struct imx_dsp_rproc_att imx_dsp_rproc_att_imx8qm[] = {
	/* dev addr , sys addr  , size	    , flags */
	{ 0x596e8000, 0x556e8000, 0x00008000, ATT_OWN },
	{ 0x596f0000, 0x556f0000, 0x00008000, ATT_OWN },
	{ 0x596f8000, 0x556f8000, 0x00000800, ATT_OWN | ATT_IRAM},
	{ 0x55700000, 0x55700000, 0x00070000, ATT_OWN },
	/* DDR (Data) */
	{ 0x80000000, 0x80000000, 0x60000000, 0},
};

static const struct imx_dsp_rproc_att imx_dsp_rproc_att_imx8qxp[] = {
	/* dev addr , sys addr  , size	    , flags */
	{ 0x596e8000, 0x596e8000, 0x00008000, ATT_OWN },
	{ 0x596f0000, 0x596f0000, 0x00008000, ATT_OWN },
	{ 0x596f8000, 0x596f8000, 0x00000800, ATT_OWN | ATT_IRAM},
	{ 0x59700000, 0x59700000, 0x00070000, ATT_OWN },
	/* DDR (Data) */
	{ 0x80000000, 0x80000000, 0x60000000, 0},
};

static const struct imx_dsp_rproc_att imx_dsp_rproc_att_imx8mp[] = {
	/* dev addr , sys addr  , size	    , flags */
	{ 0x3b6e8000, 0x3b6e8000, 0x00008000, ATT_OWN },
	{ 0x3b6f0000, 0x3b6f0000, 0x00008000, ATT_OWN },
	{ 0x3b6f8000, 0x3b6f8000, 0x00000800, ATT_OWN | ATT_IRAM},
	{ 0x3b700000, 0x3b700000, 0x00040000, ATT_OWN },
	/* DDR (Data) */
	{ 0x40000000, 0x40000000, 0x80000000, 0},
};

static const struct imx_dsp_rproc_att imx_dsp_rproc_att_imx8ulp[] = {
	/* dev addr , sys addr  , size	    , flags */
	{ 0x21170000, 0x21170000, 0x00010000, ATT_OWN | ATT_IRAM},
	{ 0x21180000, 0x21180000, 0x00010000, ATT_OWN },
	/* DDR (Data) */
	{ 0x0c000000, 0x80000000, 0x10000000, 0},
	{ 0x30000000, 0x90000000, 0x10000000, 0},
};

static int imx8mp_dsp_reset(struct imx_dsp_rproc *priv)
{
	void __iomem *dap = ioremap_wc(IMX8M_DAP_DEBUG, IMX8M_DAP_DEBUG_SIZE);
	int pwrctl;

	/* put DSP into reset and stall */
	pwrctl = readl(dap + IMX8M_DAP_PWRCTL);
	pwrctl |= IMX8M_PWRCTL_CORERESET;
	writel(pwrctl, dap + IMX8M_DAP_PWRCTL);

	/* keep reset asserted for 10 cycles */
	usleep_range(1, 2);

	regmap_update_bits(priv->regmap, IMX8M_AudioDSP_REG2,
			   IMX8M_AudioDSP_REG2_RUNSTALL,
			   IMX8M_AudioDSP_REG2_RUNSTALL);

	/* take the DSP out of reset and keep stalled for FW loading */
	pwrctl = readl(dap + IMX8M_DAP_PWRCTL);
	pwrctl &= ~IMX8M_PWRCTL_CORERESET;
	writel(pwrctl, dap + IMX8M_DAP_PWRCTL);

	iounmap(dap);
	return 0;
}

static int imx8ulp_dsp_reset(struct imx_dsp_rproc *priv)
{
	struct arm_smccc_res res;

	regmap_update_bits(priv->regmap, IMX8ULP_SIM_LPAV_REG_SYSCTRL0,
			   IMX8ULP_SYSCTRL0_DSP_RST, IMX8ULP_SYSCTRL0_DSP_RST);
	regmap_update_bits(priv->regmap, IMX8ULP_SIM_LPAV_REG_SYSCTRL0,
			   IMX8ULP_SYSCTRL0_DSP_STALL,
			   IMX8ULP_SYSCTRL0_DSP_STALL);

	arm_smccc_smc(IMX8ULP_SIP_HIFI_XRDC, 0, 0, 0, 0, 0, 0, 0, &res);

	regmap_update_bits(priv->regmap, IMX8ULP_SIM_LPAV_REG_SYSCTRL0,
			   IMX8ULP_SYSCTRL0_DSP_RST, 0);
	regmap_update_bits(priv->regmap, IMX8ULP_SIM_LPAV_REG_SYSCTRL0,
			   IMX8ULP_SYSCTRL0_DSP_DBG_RST, 0);

	return 0;
}

static const struct imx_dsp_rproc_dcfg imx_dsp_rproc_cfg_imx8mp = {
	.src_reg	= IMX8M_AudioDSP_REG2,
	.src_mask	= IMX8M_AudioDSP_REG2_RUNSTALL,
	.src_start	= 0,
	.src_stop	= IMX8M_AudioDSP_REG2_RUNSTALL,
	.att		= imx_dsp_rproc_att_imx8mp,
	.att_size	= ARRAY_SIZE(imx_dsp_rproc_att_imx8mp),
	.method		= IMX_DSP_MMIO,
	.reset          = imx8mp_dsp_reset,
};

static const struct imx_dsp_rproc_dcfg imx_dsp_rproc_cfg_imx8ulp = {
	.src_reg	= IMX8ULP_SIM_LPAV_REG_SYSCTRL0,
	.src_mask	= IMX8ULP_SYSCTRL0_DSP_STALL,
	.src_start	= 0,
	.src_stop	= IMX8ULP_SYSCTRL0_DSP_STALL,
	.att		= imx_dsp_rproc_att_imx8ulp,
	.att_size	= ARRAY_SIZE(imx_dsp_rproc_att_imx8ulp),
	.method		= IMX_DSP_MMIO,
	.reset          = imx8ulp_dsp_reset,
};

static const struct imx_dsp_rproc_dcfg imx_dsp_rproc_cfg_imx8qxp = {
	.att		= imx_dsp_rproc_att_imx8qxp,
	.att_size	= ARRAY_SIZE(imx_dsp_rproc_att_imx8qxp),
	.method		= IMX_DSP_SCU_API,
};

static const struct imx_dsp_rproc_dcfg imx_dsp_rproc_cfg_imx8qm = {
	.att		= imx_dsp_rproc_att_imx8qm,
	.att_size	= ARRAY_SIZE(imx_dsp_rproc_att_imx8qm),
	.method		= IMX_DSP_SCU_API,
};

static int imx_dsp_rproc_ready(struct rproc *rproc)
{
	struct imx_dsp_rproc *priv = rproc->priv;
	int i;

	if (!priv->rxdb_ch)
		return 0;

	for (i = 0; i < REMOTE_READY_WAIT_MAX_RETRIES; i++) {
		if (priv->flags & REMOTE_IS_READY)
			return 0;
		usleep_range(100, 200);
	}

	return -ETIMEDOUT;
}

static int imx_dsp_rproc_start(struct rproc *rproc)
{
	struct imx_dsp_rproc *priv = rproc->priv;
	const struct imx_dsp_rproc_dcfg *dcfg = priv->dcfg;
	struct device *dev = priv->dev;
	int ret;

	switch (dcfg->method) {
	case IMX_DSP_MMIO:
		ret = regmap_update_bits(priv->regmap,
					 dcfg->src_reg,
					 dcfg->src_mask,
					 dcfg->src_start);
		break;
	case IMX_DSP_SCU_API:
		ret = imx_sc_pm_cpu_start(priv->ipc_handle,
					  IMX_SC_R_DSP,
					  true,
					  rproc->bootaddr);
		break;
	default:
		return -EOPNOTSUPP;
	}

	if (ret)
		dev_err(dev, "Failed to enable remote core!\n");
	else
		ret = imx_dsp_rproc_ready(rproc);

	return ret;
}

static int imx_dsp_rproc_stop(struct rproc *rproc)
{
	struct imx_dsp_rproc *priv = rproc->priv;
	const struct imx_dsp_rproc_dcfg *dcfg = priv->dcfg;
	struct device *dev = priv->dev;
	int ret = 0;

	flush_workqueue(priv->workqueue);

	if (rproc->state == RPROC_CRASHED) {
		priv->flags &= ~REMOTE_IS_READY;
		return 0;
	}

	switch (dcfg->method) {
	case IMX_DSP_MMIO:
		ret = regmap_update_bits(priv->regmap, dcfg->src_reg, dcfg->src_mask,
					 dcfg->src_stop);
		break;
	case IMX_DSP_SCU_API:
		ret = imx_sc_pm_cpu_start(priv->ipc_handle,
					  IMX_SC_R_DSP,
					  false,
					  rproc->bootaddr);
		break;
	default:
		return -EOPNOTSUPP;
	}

	if (ret)
		dev_err(dev, "Failed to stop remote core\n");
	else
		priv->flags &= ~REMOTE_IS_READY;

	return ret;
}

/*
 * Custom memory copy implementation for DSP Cores
 */
static int imx_dsp_rproc_memcpy(void *dest, const void *src, size_t count)
{
	const u32 *s = src;
	u32 *d = dest;
	size_t size = count / 4;
	u32 *tmp_src = NULL;

	/*
	 * TODO: relax limitation of 4-byte aligned dest addresses and copy
	 * sizes
	 */
	if ((long)dest % 4 || count % 4)
		return -EINVAL;

	/* src offsets in ELF firmware image can be non-aligned */
	if ((long)src % 4) {
		tmp_src = kmemdup(src, count, GFP_KERNEL);
		if (!tmp_src)
			return -ENOMEM;
		s = tmp_src;
	}

	while (size--)
		*d++ = *s++;

	kfree(tmp_src);

	return 0;
}

static int imx_dsp_rproc_load_elf_segments(struct rproc *rproc, const struct firmware *fw)
{
	struct device *dev = &rproc->dev;
	const void *ehdr, *phdr;
	int i, ret = 0;
	u16 phnum;
	const u8 *elf_data = fw->data;
	u8 class = fw_elf_get_class(fw);
	u32 elf_phdr_get_size = elf_size_of_phdr(class);

	ehdr = elf_data;
	phnum = elf_hdr_get_e_phnum(class, ehdr);
	phdr = elf_data + elf_hdr_get_e_phoff(class, ehdr);

	/* go through the available ELF segments */
	for (i = 0; i < phnum; i++, phdr += elf_phdr_get_size) {
		u64 da = elf_phdr_get_p_paddr(class, phdr);
		u64 memsz = elf_phdr_get_p_memsz(class, phdr);
		u64 filesz = elf_phdr_get_p_filesz(class, phdr);
		u64 offset = elf_phdr_get_p_offset(class, phdr);
		u32 type = elf_phdr_get_p_type(class, phdr);
		void *ptr;

		if (type != PT_LOAD || !filesz)
			continue;

		dev_dbg(dev, "phdr: type %d da 0x%llx memsz 0x%llx filesz 0x%llx\n",
			type, da, memsz, filesz);

		if (filesz > memsz) {
			dev_err(dev, "bad phdr filesz 0x%llx memsz 0x%llx\n",
				filesz, memsz);
			ret = -EINVAL;
			break;
		}

		if (offset + filesz > fw->size) {
			dev_err(dev, "truncated fw: need 0x%llx avail 0x%zx\n",
				offset + filesz, fw->size);
			ret = -EINVAL;
			break;
		}

		if (!rproc_u64_fit_in_size_t(memsz)) {
			dev_err(dev, "size (%llx) does not fit in size_t type\n",
				memsz);
			ret = -EOVERFLOW;
			break;
		}

		/* grab the kernel address for this device address */
		ptr = rproc_da_to_va(rproc, da, memsz, NULL);
		if (!ptr) {
			dev_err(dev, "bad phdr da 0x%llx mem 0x%llx\n", da,
				memsz);
			ret = -EINVAL;
			break;
		}

		/* put the segment where the remote processor expects it */
		ret = imx_dsp_rproc_memcpy(ptr, elf_data + offset, filesz);
		if (ret) {
			dev_err(dev, "memcpy failed for da 0x%llx mem 0x%llx\n",
				da, memsz);
			break;
		}
	}

	return ret;
}

static int imx_dsp_rproc_sys_to_da(struct imx_dsp_rproc *priv, u64 sys,
				   size_t len, u64 *da)
{
	const struct imx_dsp_rproc_dcfg *dcfg = priv->dcfg;
	int i;

	/* parse address translation table */
	for (i = 0; i < dcfg->att_size; i++) {
		const struct imx_dsp_rproc_att *att = &dcfg->att[i];

		if (sys >= att->sa && sys + len <= att->sa + att->size) {
			unsigned int offset = sys - att->sa;

			*da = att->da + offset;
			return 0;
		}
	}

	return -ENOENT;
}

static int imx_dsp_rproc_da_to_sys(struct imx_dsp_rproc *priv, u64 da,
				   size_t len, u64 *sys)
{
	const struct imx_dsp_rproc_dcfg *dcfg = priv->dcfg;
	int i;

	/* parse address translation table */
	for (i = 0; i < dcfg->att_size; i++) {
		const struct imx_dsp_rproc_att *att = &dcfg->att[i];

		if (da >= att->da && da + len <= att->da + att->size) {
			unsigned int offset = da - att->da;

			*sys = att->sa + offset;
			return 0;
		}
	}

	return -ENOENT;
}

static void *imx_dsp_rproc_da_to_va(struct rproc *rproc, u64 da, size_t len, bool *is_iomem)
{
	struct imx_dsp_rproc *priv = rproc->priv;
	void *va = NULL;
	u64 sys;
	int i;

	if (len == 0)
		return NULL;

	/*
	 * On device side we have many aliases, so we need to convert device
	 * address (DSP) to system bus address first.
	 */
	if (imx_dsp_rproc_da_to_sys(priv, da, len, &sys))
		return NULL;

	for (i = 0; i < DSP_RPROC_MEM_MAX; i++) {
		if (sys >= priv->mem[i].sys_addr && sys + len <=
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

static int imx_dsp_rproc_mem_alloc(struct rproc *rproc,
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

static int imx_dsp_rproc_mem_release(struct rproc *rproc,
				     struct rproc_mem_entry *mem)
{
	dev_dbg(rproc->dev.parent, "unmap memory: %pa\n", &mem->dma);
	iounmap(mem->va);

	return 0;
}

static void imx_dsp_rproc_vq_work(struct work_struct *work)
{
	struct imx_dsp_rproc *priv = container_of(work, struct imx_dsp_rproc,
						  rproc_work);

	rproc_vq_interrupt(priv->rproc, 0);
	rproc_vq_interrupt(priv->rproc, 1);
}

static void imx_dsp_rproc_rx_callback(struct mbox_client *cl, void *data)
{
	struct rproc *rproc = dev_get_drvdata(cl->dev);
	struct imx_dsp_rproc *priv = rproc->priv;
	u32 message = (u32)(*(u32 *)data);

	dev_dbg(priv->dev, "mbox msg: 0x%x\n", message);

	switch (message) {
	case RP_MBOX_SUSPEND_ACK:
		complete(&priv->pm_comp);
		break;
	case RP_MBOX_RESUME_ACK:
		complete(&priv->pm_comp);
		break;
	default:
		queue_work(priv->workqueue, &priv->rproc_work);
		break;
	}
}

static void imx_dsp_rproc_rxdb_callback(struct mbox_client *cl, void *msg)
{
	struct rproc *rproc = dev_get_drvdata(cl->dev);
	struct imx_dsp_rproc *priv = rproc->priv;
	unsigned long flags;

	spin_lock_irqsave(&priv->mbox_lock, flags);
	priv->flags |= REMOTE_IS_READY;
	spin_unlock_irqrestore(&priv->mbox_lock, flags);
}

static int imx_dsp_rproc_mbox_init(struct imx_dsp_rproc *priv)
{
	struct device *dev = priv->dev;
	struct mbox_client *cl;
	int ret;

	if (!of_get_property(dev->of_node, "mbox-names", NULL))
		return 0;

	spin_lock_init(&priv->mbox_lock);

	cl = &priv->cl;
	cl->dev = dev;
	cl->tx_block = true;
	cl->tx_tout = 100;
	cl->knows_txdone = false;
	cl->rx_callback = imx_dsp_rproc_rx_callback;

	priv->tx_ch = mbox_request_channel_byname(cl, "tx0");
	if (IS_ERR(priv->tx_ch)) {
		ret = PTR_ERR(priv->tx_ch);
		dev_dbg(cl->dev, "failed to request tx mailbox channel: %d\n",
			ret);
		goto err_out;
	}

	priv->rx_ch = mbox_request_channel_byname(cl, "rx0");
	if (IS_ERR(priv->rx_ch)) {
		ret = PTR_ERR(priv->rx_ch);
		dev_dbg(cl->dev, "failed to request rx mailbox channel: %d\n",
			ret);
		goto err_out;
	}

	cl = &priv->cl_rxdb;
	cl->dev = dev;
	cl->rx_callback = imx_dsp_rproc_rxdb_callback;

	/*
	 * RX door bell is used to receive the ready signal from remote
	 * after the partition reset of A core.
	 */
	priv->rxdb_ch = mbox_request_channel_byname(cl, "rxdb0");
	if (IS_ERR(priv->rxdb_ch)) {
		ret = PTR_ERR(priv->rxdb_ch);
		dev_dbg(cl->dev, "failed to request mbox chan rxdb, ret %d\n",
			ret);
		goto err_out;
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

static void imx_dsp_rproc_free_mbox(struct imx_dsp_rproc *priv)
{
	mbox_free_channel(priv->tx_ch);
	mbox_free_channel(priv->rx_ch);
	mbox_free_channel(priv->rxdb_ch);
}

static int imx_dsp_rproc_prepare(struct rproc *rproc)
{
	struct imx_dsp_rproc *priv = rproc->priv;
	struct device *dev = priv->dev;
	struct device_node *np = dev->of_node;
	struct of_phandle_iterator it;
	struct rproc_mem_entry *mem;
	struct reserved_mem *rmem;
	u64 da;
	int i;

	of_phandle_iterator_init(&it, np, "memory-region", NULL, 0);
	while (of_phandle_iterator_next(&it) == 0) {
		/*
		 * Ignore the first memory region which will be used vdev buffer.
		 * No need to do extra handlings, rproc_add_virtio_dev will handle it.
		 */
		if (!strcmp(it.node->name, "vdev0buffer"))
			continue;

		rmem = of_reserved_mem_lookup(it.node);
		if (!rmem) {
			dev_err(dev, "unable to acquire memory-region\n");
			return -EINVAL;
		}

		if (imx_dsp_rproc_sys_to_da(priv, rmem->base, rmem->size, &da))
			return -EINVAL;

		/* Register memory region */
		mem = rproc_mem_entry_init(dev, NULL, (dma_addr_t)rmem->base,
					   rmem->size, da,
					   imx_dsp_rproc_mem_alloc,
					   imx_dsp_rproc_mem_release,
					   it.node->name);

		if (mem)
			rproc_coredump_add_segment(rproc, da, rmem->size);
		else
			return -ENOMEM;

		rproc_add_carveout(rproc, mem);
	}

	pm_runtime_get_sync(dev);

	for (i = 0; i < DSP_RPROC_MEM_MAX; i++)
		if (priv->mem[i].cpu_addr)
			memset_io(priv->mem[i].cpu_addr, 0, priv->mem[i].size);

	return  0;
}

static int imx_dsp_rproc_unprepare(struct rproc *rproc)
{
	struct imx_dsp_rproc *priv = rproc->priv;

	pm_runtime_put_sync(priv->dev);

	return  0;
}

static void imx_dsp_rproc_kick(struct rproc *rproc, int vqid)
{
	struct imx_dsp_rproc *priv = rproc->priv;
	int err;
	__u32 mmsg;

	if (!priv->tx_ch) {
		dev_err(priv->dev, "No initialized mbox tx channel\n");
		return;
	}

	/*
	 * Send the index of the triggered virtqueue as the mu payload.
	 * Let remote processor know which virtqueue is used.
	 */
	mmsg = vqid;

	err = mbox_send_message(priv->tx_ch, (void *)&mmsg);
	if (err < 0)
		dev_err(priv->dev, "%s: failed (%d, err:%d)\n",
			__func__, vqid, err);
}

static const struct rproc_ops imx_dsp_rproc_ops = {
	.prepare	= imx_dsp_rproc_prepare,
	.unprepare	= imx_dsp_rproc_unprepare,
	.start		= imx_dsp_rproc_start,
	.stop		= imx_dsp_rproc_stop,
	.kick		= imx_dsp_rproc_kick,
	.da_to_va       = imx_dsp_rproc_da_to_va,
	.load		= imx_dsp_rproc_load_elf_segments,
	.parse_fw	= rproc_elf_load_rsc_table,
	.sanity_check	= rproc_elf_sanity_check,
	.get_boot_addr	= rproc_elf_get_boot_addr,
};

static int imx_dsp_rproc_addr_init(struct imx_dsp_rproc *priv)
{
	const struct imx_dsp_rproc_dcfg *dcfg = priv->dcfg;
	struct device *dev = priv->dev;
	struct device_node *np = dev->of_node;
	int a, b = 0, err, nph;

	/* remap required addresses */
	for (a = 0; a < dcfg->att_size; a++) {
		const struct imx_dsp_rproc_att *att = &dcfg->att[a];

		if (!(att->flags & ATT_OWN))
			continue;

		if (b >= DSP_RPROC_MEM_MAX)
			break;

		priv->mem[b].cpu_addr = devm_ioremap(priv->dev,
						     att->sa, att->size);
		if (!priv->mem[b].cpu_addr) {
			dev_err(dev, "failed to remap %#x bytes from %#x\n", att->size, att->sa);
			return -ENOMEM;
		}
		priv->mem[b].sys_addr = att->sa;
		priv->mem[b].size = att->size;
		b++;
	}

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

		of_node_put(node);

		if (b >= DSP_RPROC_MEM_MAX)
			break;

		/* Not use resource version, because we might share region */
		priv->mem[b].cpu_addr = devm_ioremap(priv->dev, res.start,
						     resource_size(&res));
		if (!priv->mem[b].cpu_addr) {
			dev_err(dev, "failed to remap %pr\n", &res);
			return -ENOMEM;
		}
		priv->mem[b].sys_addr = res.start;
		priv->mem[b].size = resource_size(&res);
		b++;
	}

	return 0;
}

static int imx_dsp_attach_pm_domains(struct imx_dsp_rproc *priv)
{
	struct device *dev = priv->dev;
	int ret, i;

	priv->num_domains = of_count_phandle_with_args(dev->of_node,
						       "power-domains",
						       "#power-domain-cells");
	if (priv->num_domains <= 1)
		return 0;

	priv->pd_dev = devm_kmalloc_array(dev, priv->num_domains,
					  sizeof(*priv->pd_dev),
					  GFP_KERNEL);
	if (!priv->pd_dev)
		return -ENOMEM;

	priv->pd_dev_link = devm_kmalloc_array(dev, priv->num_domains,
					       sizeof(*priv->pd_dev_link),
					       GFP_KERNEL);
	if (!priv->pd_dev_link)
		return -ENOMEM;

	for (i = 0; i < priv->num_domains; i++) {
		priv->pd_dev[i] = dev_pm_domain_attach_by_id(dev, i);
		if (IS_ERR(priv->pd_dev[i]))
			return PTR_ERR(priv->pd_dev[i]);

		priv->pd_dev_link[i] = device_link_add(dev,
						       priv->pd_dev[i],
						       DL_FLAG_STATELESS |
						       DL_FLAG_PM_RUNTIME);
		if (IS_ERR(priv->pd_dev_link[i])) {
			dev_pm_domain_detach(priv->pd_dev[i], false);
			ret = PTR_ERR(priv->pd_dev_link[i]);
			goto detach_pm;
		}
	}

	return 0;

detach_pm:
	while (--i >= 0) {
		device_link_del(priv->pd_dev_link[i]);
		dev_pm_domain_detach(priv->pd_dev[i], false);
	}

	return ret;
}

static int imx_dsp_detach_pm_domains(struct imx_dsp_rproc *priv)
{
	int i;

	if (priv->num_domains <= 1)
		return 0;

	for (i = 0; i < priv->num_domains; i++) {
		device_link_del(priv->pd_dev_link[i]);
		dev_pm_domain_detach(priv->pd_dev[i], false);
	}

	return 0;
}

static int imx_dsp_rproc_detect_mode(struct imx_dsp_rproc *priv)
{
	const struct imx_dsp_rproc_dcfg *dcfg = priv->dcfg;
	struct device *dev = priv->dev;
	struct regmap *regmap;
	int ret = 0;

	switch (dcfg->method) {
	case IMX_DSP_SCU_API:
		ret = imx_scu_get_handle(&priv->ipc_handle);
		if (ret)
			return ret;
		return 0;
	case IMX_DSP_MMIO:
		regmap = syscon_regmap_lookup_by_phandle(dev->of_node, "syscon");
		if (IS_ERR(regmap)) {
			dev_err(dev, "failed to find syscon\n");
			return PTR_ERR(regmap);
		}

		priv->regmap = regmap;
	default:
		break;
	}

	return ret;
}

static const char *imx_dsp_clks_names[DSP_RPROC_CLK_MAX] = {
	/* DSP clocks */
	"dsp_clk1", "dsp_clk2", "dsp_clk3", "dsp_clk4", "dsp_clk5", "dsp_clk6",
	"dsp_clk7", "dsp_clk8",

	/* mbox clocks */
	"mu_a", "mu_b",

	/* Peripheral clocks */
	"per_clk1", "per_clk2", "per_clk3", "per_clk4", "per_clk5", "per_clk6",
	"per_clk7", "per_clk8", "per_clk9", "per_clk10", "per_clk11", "per_clk12",
};

static int imx_dsp_rproc_clk_get(struct imx_dsp_rproc *priv)
{
	int i;

	for (i = 0; i < DSP_RPROC_CLK_MAX; i++) {
		priv->clks[i] = devm_clk_get_optional(priv->dev,
						      imx_dsp_clks_names[i]);
		if (IS_ERR(priv->clks[i])) {
			dev_err(priv->dev, "Failed to get clock %s\n",
				imx_dsp_clks_names[i]);
			return PTR_ERR(priv->clks[i]);
		}
	}

	return 0;
}

static int imx_dsp_rproc_clk_enable(struct imx_dsp_rproc *priv)
{
	int i, ret;

	for (i = 0; i < DSP_RPROC_CLK_MAX; i++) {
		ret = clk_prepare_enable(priv->clks[i]);
		if (ret < 0) {
			dev_err(priv->dev, "Failed to enable clk %s\n",
				imx_dsp_clks_names[i]);
			goto err_dsp_clks;
		}
	}

	return 0;

err_dsp_clks:
	while (--i >= 0)
		clk_disable_unprepare(priv->clks[i]);

	return ret;
}

static void imx_dsp_rproc_clk_disable(struct imx_dsp_rproc *priv)
{
	int i;

	for (i = 0; i < DSP_RPROC_CLK_MAX; i++)
		clk_disable_unprepare(priv->clks[i]);
}

static int imx_dsp_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct imx_dsp_rproc_dcfg *dcfg;
	struct imx_dsp_rproc *priv;
	struct rproc *rproc;
	const char *fw_name;
	int ret;

	dcfg = of_device_get_match_data(dev);
	if (!dcfg)
		return -ENODEV;

	ret = rproc_of_parse_firmware(dev, 0, &fw_name);
	if (ret) {
		dev_err(dev, "failed to parse firmware-name property, ret = %d\n",
			ret);
		return ret;
	}

	rproc = rproc_alloc(dev, "imx-dsp-rproc", &imx_dsp_rproc_ops, fw_name,
			    sizeof(*priv));
	if (!rproc)
		return -ENOMEM;

	priv = rproc->priv;
	priv->rproc = rproc;
	priv->dcfg = dcfg;
	priv->dev = dev;

	dev_set_drvdata(dev, rproc);

	priv->workqueue = create_workqueue(dev_name(dev));
	if (!priv->workqueue) {
		dev_err(dev, "cannot create workqueue\n");
		ret = -ENOMEM;
		goto err_wkq;
	}

	INIT_WORK(&priv->rproc_work, imx_dsp_rproc_vq_work);

	ret = imx_dsp_rproc_addr_init(priv);
	if (ret) {
		dev_err(dev, "failed on imx_dsp_rproc_addr_init\n");
		goto err_addr;
	}

	ret = imx_dsp_rproc_detect_mode(priv);
	if (ret) {
		dev_err(dev, "failed on imx_dsp_rproc_detect_mode\n");
		goto err_detect;
	}

	ret = imx_dsp_attach_pm_domains(priv);
	if (ret) {
		dev_err(dev, "failed on imx_dsp_attach_pm_domains\n");
		goto err_pm;
	}

	ret = imx_dsp_rproc_clk_get(priv);
	if (ret) {
		dev_err(dev, "failed on imx_dsp_rproc_clk_get\n");
		goto err_clk_get;
	}

	init_completion(&priv->pm_comp);
	rproc->auto_boot = false;
	ret = rproc_add(rproc);
	if (ret) {
		dev_err(dev, "rproc_add failed\n");
		goto err_rproc_add;
	}

	pm_runtime_enable(dev);

	return 0;

err_rproc_add:
err_clk_get:
	imx_dsp_detach_pm_domains(priv);
err_pm:
err_detect:
err_addr:
	destroy_workqueue(priv->workqueue);
err_wkq:
	rproc_free(rproc);

	return ret;
}

static int imx_dsp_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct imx_dsp_rproc *priv = rproc->priv;

	pm_runtime_disable(&pdev->dev);
	rproc_del(rproc);
	imx_dsp_detach_pm_domains(priv);
	destroy_workqueue(priv->workqueue);
	rproc_free(rproc);

	return 0;
}

/* pm runtime */
static int imx_dsp_runtime_resume(struct device *dev)
{
	struct rproc *rproc = dev_get_drvdata(dev);
	struct imx_dsp_rproc *priv = rproc->priv;
	const struct imx_dsp_rproc_dcfg *dcfg = priv->dcfg;
	int ret;

	ret = imx_dsp_rproc_mbox_init(priv);
	if (ret) {
		dev_err(dev, "failed on imx_dsp_rproc_mbox_init\n");
		return ret;
	}

	ret = imx_dsp_rproc_clk_enable(priv);
	if (ret) {
		dev_err(dev, "failed on imx_dsp_rproc_clk_enable\n");
		return ret;
	}

	/* reset DSP if needed */
	if (dcfg->reset)
		dcfg->reset(priv);

	return 0;
}

static int imx_dsp_runtime_suspend(struct device *dev)
{
	struct rproc *rproc = dev_get_drvdata(dev);
	struct imx_dsp_rproc *priv = rproc->priv;

	imx_dsp_rproc_clk_disable(priv);

	imx_dsp_rproc_free_mbox(priv);

	return 0;
}

static void imx_dsp_load_firmware(const struct firmware *fw, void *context)
{
	struct rproc *rproc = context;
	int ret;

	/* load the ELF segments to memory */
	ret = rproc_load_segments(rproc, fw);
	if (ret)
		goto out;

	/* power up the remote processor */
	ret = rproc->ops->start(rproc);
	if (ret)
		goto out;

	/* same flow as first start */
	rproc->ops->kick(rproc, 0);

out:
	release_firmware(fw);
}

static int imx_dsp_suspend(struct device *dev)
{
	struct rproc *rproc = dev_get_drvdata(dev);
	struct imx_dsp_rproc *priv = rproc->priv;
	__u32 mmsg = RP_MBOX_SUSPEND_SYSTEM;
	int ret;

	if (rproc->state == RPROC_RUNNING) {
		reinit_completion(&priv->pm_comp);

		ret = mbox_send_message(priv->tx_ch, (void *)&mmsg);
		if (ret < 0) {
			dev_err(dev, "PM mbox_send_message failed: %d\n", ret);
			return ret;
		}

		if (!wait_for_completion_timeout(&priv->pm_comp, msecs_to_jiffies(100)))
			return -EBUSY;
	}

	return pm_runtime_force_suspend(dev);
}

static int imx_dsp_resume(struct device *dev)
{
	struct rproc *rproc = dev_get_drvdata(dev);
	int ret = 0;

	ret = pm_runtime_force_resume(dev);
	if (ret)
		return ret;

	if (rproc->state == RPROC_RUNNING) {
		/*TODO: load firmware and start */
		ret = request_firmware_nowait(THIS_MODULE,
					      FW_ACTION_HOTPLUG,
					      rproc->firmware,
					      dev,
					      GFP_KERNEL,
					      rproc,
					      imx_dsp_load_firmware);
		if (ret < 0) {
			dev_err(dev, "load firmware failed: %d\n", ret);
			goto err;
		}
	}

	return 0;

err:
	pm_runtime_force_suspend(dev);

	return ret;
}

static const struct dev_pm_ops imx_dsp_rproc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(imx_dsp_suspend, imx_dsp_resume)
	SET_RUNTIME_PM_OPS(imx_dsp_runtime_suspend,
			   imx_dsp_runtime_resume, NULL)
};

static const struct of_device_id imx_dsp_rproc_of_match[] = {
	{ .compatible = "fsl,imx8qxp-hifi4", .data = &imx_dsp_rproc_cfg_imx8qxp },
	{ .compatible = "fsl,imx8qm-hifi4",  .data = &imx_dsp_rproc_cfg_imx8qm },
	{ .compatible = "fsl,imx8mp-hifi4",  .data = &imx_dsp_rproc_cfg_imx8mp },
	{ .compatible = "fsl,imx8ulp-hifi4", .data = &imx_dsp_rproc_cfg_imx8ulp },
	{},
};
MODULE_DEVICE_TABLE(of, imx_dsp_rproc_of_match);

static struct platform_driver imx_dsp_rproc_driver = {
	.probe = imx_dsp_rproc_probe,
	.remove = imx_dsp_rproc_remove,
	.driver = {
		.name = "imx-dsp-rproc",
		.of_match_table = imx_dsp_rproc_of_match,
		.pm = &imx_dsp_rproc_pm_ops,
	},
};
module_platform_driver(imx_dsp_rproc_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("i.MX HiFi Core Remote Processor Control Driver");
MODULE_AUTHOR("Shengjiu Wang <shengjiu.wang@nxp.com>");
