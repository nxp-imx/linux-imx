/*
 * Copyright 2005-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file ipu_common.c
 *
 * @brief This file contains the IPU driver common API functions.
 *
 * @ingroup IPU
 */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ipu.h>
#include <linux/clk.h>
#include <mach/clock.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>

#include "ipu_prv.h"
#include "ipu_regs.h"
#include "ipu_param_mem.h"

struct ipu_irq_node {
	irqreturn_t(*handler) (int, void *);	/*!< the ISR */
	const char *name;	/*!< device associated with the interrupt */
	void *dev_id;		/*!< some unique information for the ISR */
	__u32 flags;		/*!< not used */
};

/* Globals */
struct clk *g_ipu_clk;
bool g_ipu_clk_enabled;
struct clk *g_di_clk[2];
struct clk *g_pixel_clk[2];
struct clk *g_csi_clk[2];
unsigned char g_dc_di_assignment[10];
ipu_channel_t g_ipu_csi_channel[2];
int g_ipu_irq[2];
int g_ipu_hw_rev;
bool g_sec_chan_en[24];
bool g_thrd_chan_en[24];
bool g_chan_is_interlaced[52];
uint32_t g_channel_init_mask;
uint32_t g_channel_enable_mask;
DEFINE_SPINLOCK(ipu_lock);
struct device *g_ipu_dev;

static struct ipu_irq_node ipu_irq_list[IPU_IRQ_COUNT];
static const char driver_name[] = "mxc_ipu";

static int ipu_dc_use_count;
static int ipu_dp_use_count;
static int ipu_dmfc_use_count;
static int ipu_smfc_use_count;
static int ipu_ic_use_count;
static int ipu_rot_use_count;
static int ipu_vdi_use_count;
static int ipu_di_use_count[2];
static int ipu_csi_use_count[2];
/* Set to the follow using IC direct channel, default non */
static ipu_channel_t using_ic_dirct_ch;

/* for power gating */
static uint32_t ipu_conf_reg;
static uint32_t ic_conf_reg;
static uint32_t ipu_cha_db_mode_reg[4];
static uint32_t ipu_cha_cur_buf_reg[4];
static uint32_t idma_enable_reg[2];
static uint32_t buf_ready_reg[8];

u32 *ipu_cm_reg;
u32 *ipu_idmac_reg;
u32 *ipu_dp_reg;
u32 *ipu_ic_reg;
u32 *ipu_dc_reg;
u32 *ipu_dc_tmpl_reg;
u32 *ipu_dmfc_reg;
u32 *ipu_di_reg[2];
u32 *ipu_smfc_reg;
u32 *ipu_csi_reg[2];
u32 *ipu_cpmem_base;
u32 *ipu_tpmem_base;
u32 *ipu_disp_base[2];
u32 *ipu_vdi_reg;

/* Static functions */
static irqreturn_t ipu_irq_handler(int irq, void *desc);

static inline uint32_t channel_2_dma(ipu_channel_t ch, ipu_buffer_t type)
{
	return ((uint32_t) ch >> (6 * type)) & 0x3F;
};

static inline int _ipu_is_ic_chan(uint32_t dma_chan)
{
	return ((dma_chan >= 11) && (dma_chan <= 22) && (dma_chan != 17) && (dma_chan != 18));
}

static inline int _ipu_is_ic_graphic_chan(uint32_t dma_chan)
{
	return (dma_chan == 14 || dma_chan == 15);
}

/* Either DP BG or DP FG can be graphic window */
static inline int _ipu_is_dp_graphic_chan(uint32_t dma_chan)
{
	return (dma_chan == 23 || dma_chan == 27);
}

static inline int _ipu_is_irt_chan(uint32_t dma_chan)
{
	return ((dma_chan >= 45) && (dma_chan <= 50));
}

static inline int _ipu_is_dmfc_chan(uint32_t dma_chan)
{
	return ((dma_chan >= 23) && (dma_chan <= 29));
}

static inline int _ipu_is_smfc_chan(uint32_t dma_chan)
{
	return ((dma_chan >= 0) && (dma_chan <= 3));
}

#define idma_is_valid(ch)	(ch != NO_DMA)
#define idma_mask(ch)		(idma_is_valid(ch) ? (1UL << (ch & 0x1F)) : 0)
#define idma_is_set(reg, dma)	(__raw_readl(reg(dma)) & idma_mask(dma))

static unsigned long _ipu_pixel_clk_get_rate(struct clk *clk)
{
	u32 div = __raw_readl(DI_BS_CLKGEN0(clk->id));
	if (div == 0)
		return 0;
	return  (clk_get_rate(clk->parent) * 16) / div;
}

static unsigned long _ipu_pixel_clk_round_rate(struct clk *clk, unsigned long rate)
{
	u32 div, div1;
	u32 parent_rate = clk_get_rate(clk->parent) * 16;
	/*
	 * Calculate divider
	 * Fractional part is 4 bits,
	 * so simply multiply by 2^4 to get fractional part.
	 */
	div = parent_rate / rate;

	if (div < 0x10)            /* Min DI disp clock divider is 1 */
		div = 0x10;
	if (div & ~0xFEF)
		div &= 0xFF8;
	else {
		div1 = div & 0xFE0;
		if ((parent_rate / div1 - parent_rate / div) < rate / 4)
			div = div1;
		else
			div &= 0xFF8;
	}
	return parent_rate / div;
}

static int _ipu_pixel_clk_set_rate(struct clk *clk, unsigned long rate)
{
	u32 div = (clk_get_rate(clk->parent) * 16) / rate;

	__raw_writel(div, DI_BS_CLKGEN0(clk->id));

	/* Setup pixel clock timing */
	/* FIXME: needs to be more flexible */
	/* Down time is half of period */
	__raw_writel((div / 16) << 16, DI_BS_CLKGEN1(clk->id));

	return 0;
}

static int _ipu_pixel_clk_enable(struct clk *clk)
{
	u32 disp_gen = __raw_readl(IPU_DISP_GEN);
	disp_gen |= clk->id ? DI1_COUNTER_RELEASE : DI0_COUNTER_RELEASE;
	__raw_writel(disp_gen, IPU_DISP_GEN);

	start_dvfs_per();

	return 0;
}

static void _ipu_pixel_clk_disable(struct clk *clk)
{
	u32 disp_gen = __raw_readl(IPU_DISP_GEN);
	disp_gen &= clk->id ? ~DI1_COUNTER_RELEASE : ~DI0_COUNTER_RELEASE;
	__raw_writel(disp_gen, IPU_DISP_GEN);

	start_dvfs_per();
}

static int _ipu_pixel_clk_set_parent(struct clk *clk, struct clk *parent)
{
	u32 di_gen = __raw_readl(DI_GENERAL(clk->id));

	if (parent == g_ipu_clk)
		di_gen &= ~DI_GEN_DI_CLK_EXT;
	else if (!IS_ERR(g_di_clk[clk->id]) && parent == g_di_clk[clk->id])
		di_gen |= DI_GEN_DI_CLK_EXT;
	else
		return -EINVAL;

	__raw_writel(di_gen, DI_GENERAL(clk->id));
	return 0;
}

static struct clk pixel_clk[] = {
	{
	.id = 0,
	.get_rate = _ipu_pixel_clk_get_rate,
	.set_rate = _ipu_pixel_clk_set_rate,
	.round_rate = _ipu_pixel_clk_round_rate,
	.set_parent = _ipu_pixel_clk_set_parent,
	.enable = _ipu_pixel_clk_enable,
	.disable = _ipu_pixel_clk_disable,
	},
	{
	.id = 1,
	.get_rate = _ipu_pixel_clk_get_rate,
	.set_rate = _ipu_pixel_clk_set_rate,
	.round_rate = _ipu_pixel_clk_round_rate,
	.set_parent = _ipu_pixel_clk_set_parent,
	.enable = _ipu_pixel_clk_enable,
	.disable = _ipu_pixel_clk_disable,
	},
};

/*!
 * This function is called by the driver framework to initialize the IPU
 * hardware.
 *
 * @param	dev	The device structure for the IPU passed in by the
 *			driver framework.
 *
 * @return      Returns 0 on success or negative error code on error
 */
static int ipu_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct mxc_ipu_config *plat_data = pdev->dev.platform_data;
	unsigned long ipu_base;

	spin_lock_init(&ipu_lock);

	g_ipu_hw_rev = plat_data->rev;

	g_ipu_dev = &pdev->dev;

	/* Register IPU interrupts */
	g_ipu_irq[0] = platform_get_irq(pdev, 0);
	if (g_ipu_irq[0] < 0)
		return -EINVAL;

	if (request_irq(g_ipu_irq[0], ipu_irq_handler, 0, pdev->name, 0) != 0) {
		dev_err(g_ipu_dev, "request SYNC interrupt failed\n");
		return -EBUSY;
	}
	/* Some platforms have 2 IPU interrupts */
	g_ipu_irq[1] = platform_get_irq(pdev, 1);
	if (g_ipu_irq[1] >= 0) {
		if (request_irq
		    (g_ipu_irq[1], ipu_irq_handler, 0, pdev->name, 0) != 0) {
			dev_err(g_ipu_dev, "request ERR interrupt failed\n");
			return -EBUSY;
		}
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR(res))
		return -ENODEV;

	ipu_base = res->start;
	if (g_ipu_hw_rev == 3)	/* IPUv3M */
		ipu_base += IPUV3M_REG_BASE;
	else			/* IPUv3D, v3E, v3EX */
		ipu_base += IPU_REG_BASE;

	ipu_cm_reg = ioremap(ipu_base + IPU_CM_REG_BASE, PAGE_SIZE);
	ipu_ic_reg = ioremap(ipu_base + IPU_IC_REG_BASE, PAGE_SIZE);
	ipu_idmac_reg = ioremap(ipu_base + IPU_IDMAC_REG_BASE, PAGE_SIZE);
	/* DP Registers are accessed thru the SRM */
	ipu_dp_reg = ioremap(ipu_base + IPU_SRM_REG_BASE, PAGE_SIZE);
	ipu_dc_reg = ioremap(ipu_base + IPU_DC_REG_BASE, PAGE_SIZE);
	ipu_dmfc_reg = ioremap(ipu_base + IPU_DMFC_REG_BASE, PAGE_SIZE);
	ipu_di_reg[0] = ioremap(ipu_base + IPU_DI0_REG_BASE, PAGE_SIZE);
	ipu_di_reg[1] = ioremap(ipu_base + IPU_DI1_REG_BASE, PAGE_SIZE);
	ipu_smfc_reg = ioremap(ipu_base + IPU_SMFC_REG_BASE, PAGE_SIZE);
	ipu_csi_reg[0] = ioremap(ipu_base + IPU_CSI0_REG_BASE, PAGE_SIZE);
	ipu_csi_reg[1] = ioremap(ipu_base + IPU_CSI1_REG_BASE, PAGE_SIZE);
	ipu_cpmem_base = ioremap(ipu_base + IPU_CPMEM_REG_BASE, PAGE_SIZE);
	ipu_tpmem_base = ioremap(ipu_base + IPU_TPM_REG_BASE, SZ_64K);
	ipu_dc_tmpl_reg = ioremap(ipu_base + IPU_DC_TMPL_REG_BASE, SZ_128K);
	ipu_disp_base[1] = ioremap(ipu_base + IPU_DISP1_BASE, SZ_4K);
	ipu_vdi_reg = ioremap(ipu_base + IPU_VDI_REG_BASE, PAGE_SIZE);

	dev_dbg(g_ipu_dev, "IPU VDI Regs = %p\n", ipu_vdi_reg);
	dev_dbg(g_ipu_dev, "IPU CM Regs = %p\n", ipu_cm_reg);
	dev_dbg(g_ipu_dev, "IPU IC Regs = %p\n", ipu_ic_reg);
	dev_dbg(g_ipu_dev, "IPU IDMAC Regs = %p\n", ipu_idmac_reg);
	dev_dbg(g_ipu_dev, "IPU DP Regs = %p\n", ipu_dp_reg);
	dev_dbg(g_ipu_dev, "IPU DC Regs = %p\n", ipu_dc_reg);
	dev_dbg(g_ipu_dev, "IPU DMFC Regs = %p\n", ipu_dmfc_reg);
	dev_dbg(g_ipu_dev, "IPU DI0 Regs = %p\n", ipu_di_reg[0]);
	dev_dbg(g_ipu_dev, "IPU DI1 Regs = %p\n", ipu_di_reg[1]);
	dev_dbg(g_ipu_dev, "IPU SMFC Regs = %p\n", ipu_smfc_reg);
	dev_dbg(g_ipu_dev, "IPU CSI0 Regs = %p\n", ipu_csi_reg[0]);
	dev_dbg(g_ipu_dev, "IPU CSI1 Regs = %p\n", ipu_csi_reg[1]);
	dev_dbg(g_ipu_dev, "IPU CPMem = %p\n", ipu_cpmem_base);
	dev_dbg(g_ipu_dev, "IPU TPMem = %p\n", ipu_tpmem_base);
	dev_dbg(g_ipu_dev, "IPU DC Template Mem = %p\n", ipu_dc_tmpl_reg);
	dev_dbg(g_ipu_dev, "IPU Display Region 1 Mem = %p\n", ipu_disp_base[1]);

	g_pixel_clk[0] = &pixel_clk[0];
	g_pixel_clk[1] = &pixel_clk[1];

	/* Enable IPU and CSI clocks */
	/* Get IPU clock freq */
	g_ipu_clk = clk_get(&pdev->dev, "ipu_clk");
	dev_dbg(g_ipu_dev, "ipu_clk = %lu\n", clk_get_rate(g_ipu_clk));

	if (plat_data->reset)
		plat_data->reset();

	clk_set_parent(g_pixel_clk[0], g_ipu_clk);
	clk_set_parent(g_pixel_clk[1], g_ipu_clk);
	clk_enable(g_ipu_clk);

	g_di_clk[0] = plat_data->di_clk[0];
	g_di_clk[1] = plat_data->di_clk[1];

	g_csi_clk[0] = plat_data->csi_clk[0];
	g_csi_clk[1] = plat_data->csi_clk[1];

	__raw_writel(0x807FFFFF, IPU_MEM_RST);
	while (__raw_readl(IPU_MEM_RST) & 0x80000000)
		;

	_ipu_init_dc_mappings();

	/* Enable error interrupts by default */
	__raw_writel(0xFFFFFFFF, IPU_INT_CTRL(5));
	__raw_writel(0xFFFFFFFF, IPU_INT_CTRL(6));
	__raw_writel(0xFFFFFFFF, IPU_INT_CTRL(9));
	__raw_writel(0xFFFFFFFF, IPU_INT_CTRL(10));

	/* DMFC Init */
	_ipu_dmfc_init(DMFC_NORMAL, 1);

	/* Set sync refresh channels and CSI->mem channel as high priority */
	__raw_writel(0x18800001L, IDMAC_CHA_PRI(0));

	/* Set MCU_T to divide MCU access window into 2 */
	__raw_writel(0x00400000L | (IPU_MCU_T_DEFAULT << 18), IPU_DISP_GEN);

	clk_disable(g_ipu_clk);

	register_ipu_device();

	return 0;
}

int ipu_remove(struct platform_device *pdev)
{
	if (g_ipu_irq[0])
		free_irq(g_ipu_irq[0], 0);
	if (g_ipu_irq[1])
		free_irq(g_ipu_irq[1], 0);

	clk_put(g_ipu_clk);

	iounmap(ipu_cm_reg);
	iounmap(ipu_ic_reg);
	iounmap(ipu_idmac_reg);
	iounmap(ipu_dc_reg);
	iounmap(ipu_dp_reg);
	iounmap(ipu_dmfc_reg);
	iounmap(ipu_di_reg[0]);
	iounmap(ipu_di_reg[1]);
	iounmap(ipu_smfc_reg);
	iounmap(ipu_csi_reg[0]);
	iounmap(ipu_csi_reg[1]);
	iounmap(ipu_cpmem_base);
	iounmap(ipu_tpmem_base);
	iounmap(ipu_dc_tmpl_reg);
	iounmap(ipu_disp_base[1]);
	iounmap(ipu_vdi_reg);

	return 0;
}

void ipu_dump_registers(void)
{
	printk(KERN_DEBUG "IPU_CONF = \t0x%08X\n", __raw_readl(IPU_CONF));
	printk(KERN_DEBUG "IDMAC_CONF = \t0x%08X\n", __raw_readl(IDMAC_CONF));
	printk(KERN_DEBUG "IDMAC_CHA_EN1 = \t0x%08X\n",
	       __raw_readl(IDMAC_CHA_EN(0)));
	printk(KERN_DEBUG "IDMAC_CHA_EN2 = \t0x%08X\n",
	       __raw_readl(IDMAC_CHA_EN(32)));
	printk(KERN_DEBUG "IDMAC_CHA_PRI1 = \t0x%08X\n",
	       __raw_readl(IDMAC_CHA_PRI(0)));
	printk(KERN_DEBUG "IDMAC_CHA_PRI2 = \t0x%08X\n",
	       __raw_readl(IDMAC_CHA_PRI(32)));
	printk(KERN_DEBUG "IDMAC_BAND_EN1 = \t0x%08X\n",
	       __raw_readl(IDMAC_BAND_EN(0)));
	printk(KERN_DEBUG "IDMAC_BAND_EN2 = \t0x%08X\n",
	       __raw_readl(IDMAC_BAND_EN(32)));
	printk(KERN_DEBUG "IPU_CHA_DB_MODE_SEL0 = \t0x%08X\n",
	       __raw_readl(IPU_CHA_DB_MODE_SEL(0)));
	printk(KERN_DEBUG "IPU_CHA_DB_MODE_SEL1 = \t0x%08X\n",
	       __raw_readl(IPU_CHA_DB_MODE_SEL(32)));
	printk(KERN_DEBUG "DMFC_WR_CHAN = \t0x%08X\n",
	       __raw_readl(DMFC_WR_CHAN));
	printk(KERN_DEBUG "DMFC_WR_CHAN_DEF = \t0x%08X\n",
	       __raw_readl(DMFC_WR_CHAN_DEF));
	printk(KERN_DEBUG "DMFC_DP_CHAN = \t0x%08X\n",
	       __raw_readl(DMFC_DP_CHAN));
	printk(KERN_DEBUG "DMFC_DP_CHAN_DEF = \t0x%08X\n",
	       __raw_readl(DMFC_DP_CHAN_DEF));
	printk(KERN_DEBUG "DMFC_IC_CTRL = \t0x%08X\n",
	       __raw_readl(DMFC_IC_CTRL));
	printk(KERN_DEBUG "IPU_FS_PROC_FLOW1 = \t0x%08X\n",
	       __raw_readl(IPU_FS_PROC_FLOW1));
	printk(KERN_DEBUG "IPU_FS_PROC_FLOW2 = \t0x%08X\n",
	       __raw_readl(IPU_FS_PROC_FLOW2));
	printk(KERN_DEBUG "IPU_FS_PROC_FLOW3 = \t0x%08X\n",
	       __raw_readl(IPU_FS_PROC_FLOW3));
	printk(KERN_DEBUG "IPU_FS_DISP_FLOW1 = \t0x%08X\n",
	       __raw_readl(IPU_FS_DISP_FLOW1));
}

/*!
 * This function is called to initialize a logical IPU channel.
 *
 * @param       channel Input parameter for the logical channel ID to init.
 *
 * @param       params  Input parameter containing union of channel
 *                      initialization parameters.
 *
 * @return      Returns 0 on success or negative error code on fail
 */
int32_t ipu_init_channel(ipu_channel_t channel, ipu_channel_params_t *params)
{
	int ret = 0;
	uint32_t ipu_conf;
	uint32_t reg;
	unsigned long lock_flags;

	dev_dbg(g_ipu_dev, "init channel = %d\n", IPU_CHAN_ID(channel));

	/* re-enable error interrupts every time a channel is initialized */
	__raw_writel(0xFFFFFFFF, IPU_INT_CTRL(5));
	__raw_writel(0xFFFFFFFF, IPU_INT_CTRL(6));
	__raw_writel(0xFFFFFFFF, IPU_INT_CTRL(9));
	__raw_writel(0xFFFFFFFF, IPU_INT_CTRL(10));

	if (g_ipu_clk_enabled == false) {
		stop_dvfs_per();
		g_ipu_clk_enabled = true;
		clk_enable(g_ipu_clk);
	}

	spin_lock_irqsave(&ipu_lock, lock_flags);

	if (g_channel_init_mask & (1L << IPU_CHAN_ID(channel))) {
		dev_err(g_ipu_dev, "Warning: channel already initialized %d\n",
			IPU_CHAN_ID(channel));
	}

	ipu_conf = __raw_readl(IPU_CONF);

	switch (channel) {
	case CSI_MEM0:
	case CSI_MEM1:
	case CSI_MEM2:
	case CSI_MEM3:
		if (params->csi_mem.csi > 1) {
			ret = -EINVAL;
			goto err;
		}

		if (params->csi_mem.interlaced)
			g_chan_is_interlaced[channel_2_dma(channel,
				IPU_OUTPUT_BUFFER)] = true;
		else
			g_chan_is_interlaced[channel_2_dma(channel,
				IPU_OUTPUT_BUFFER)] = false;

		ipu_smfc_use_count++;
		g_ipu_csi_channel[params->csi_mem.csi] = channel;

		/*SMFC setting*/
		if (params->csi_mem.mipi_en) {
			ipu_conf |= (1 << (IPU_CONF_CSI0_DATA_SOURCE_OFFSET +
				params->csi_mem.csi));
			_ipu_smfc_init(channel, params->csi_mem.mipi_id,
				params->csi_mem.csi);
		} else {
			ipu_conf &= ~(1 << (IPU_CONF_CSI0_DATA_SOURCE_OFFSET +
				params->csi_mem.csi));
			_ipu_smfc_init(channel, 0, params->csi_mem.csi);
		}

		/*CSI data (include compander) dest*/
		_ipu_csi_init(channel, params->csi_mem.csi);
		break;
	case CSI_PRP_ENC_MEM:
		if (params->csi_prp_enc_mem.csi > 1) {
			ret = -EINVAL;
			goto err;
		}
		if (using_ic_dirct_ch == MEM_VDI_PRP_VF_MEM) {
			ret = -EINVAL;
			goto err;
		}
		using_ic_dirct_ch = CSI_PRP_ENC_MEM;

		ipu_ic_use_count++;
		g_ipu_csi_channel[params->csi_prp_enc_mem.csi] = channel;

		/*Without SMFC, CSI only support parallel data source*/
		ipu_conf &= ~(1 << (IPU_CONF_CSI0_DATA_SOURCE_OFFSET +
			params->csi_prp_enc_mem.csi));

		/*CSI0/1 feed into IC*/
		ipu_conf &= ~IPU_CONF_IC_INPUT;
		if (params->csi_prp_enc_mem.csi)
			ipu_conf |= IPU_CONF_CSI_SEL;
		else
			ipu_conf &= ~IPU_CONF_CSI_SEL;

		/*PRP skip buffer in memory, only valid when RWS_EN is true*/
		reg = __raw_readl(IPU_FS_PROC_FLOW1);
		__raw_writel(reg & ~FS_ENC_IN_VALID, IPU_FS_PROC_FLOW1);

		/*CSI data (include compander) dest*/
		_ipu_csi_init(channel, params->csi_prp_enc_mem.csi);
		_ipu_ic_init_prpenc(params, true);
		break;
	case CSI_PRP_VF_MEM:
		if (params->csi_prp_vf_mem.csi > 1) {
			ret = -EINVAL;
			goto err;
		}
		if (using_ic_dirct_ch == MEM_VDI_PRP_VF_MEM) {
			ret = -EINVAL;
			goto err;
		}
		using_ic_dirct_ch = CSI_PRP_VF_MEM;

		ipu_ic_use_count++;
		g_ipu_csi_channel[params->csi_prp_vf_mem.csi] = channel;

		/*Without SMFC, CSI only support parallel data source*/
		ipu_conf &= ~(1 << (IPU_CONF_CSI0_DATA_SOURCE_OFFSET +
			params->csi_prp_vf_mem.csi));

		/*CSI0/1 feed into IC*/
		ipu_conf &= ~IPU_CONF_IC_INPUT;
		if (params->csi_prp_vf_mem.csi)
			ipu_conf |= IPU_CONF_CSI_SEL;
		else
			ipu_conf &= ~IPU_CONF_CSI_SEL;

		/*PRP skip buffer in memory, only valid when RWS_EN is true*/
		reg = __raw_readl(IPU_FS_PROC_FLOW1);
		__raw_writel(reg & ~FS_VF_IN_VALID, IPU_FS_PROC_FLOW1);

		/*CSI data (include compander) dest*/
		_ipu_csi_init(channel, params->csi_prp_vf_mem.csi);
		_ipu_ic_init_prpvf(params, true);
		break;
	case MEM_PRP_VF_MEM:
		ipu_ic_use_count++;
		reg = __raw_readl(IPU_FS_PROC_FLOW1);
		__raw_writel(reg | FS_VF_IN_VALID, IPU_FS_PROC_FLOW1);

		if (params->mem_prp_vf_mem.graphics_combine_en)
			g_sec_chan_en[IPU_CHAN_ID(channel)] = true;
		if (params->mem_prp_vf_mem.alpha_chan_en)
			g_thrd_chan_en[IPU_CHAN_ID(channel)] = true;

		_ipu_ic_init_prpvf(params, false);
		break;
	case MEM_VDI_PRP_VF_MEM:
		if ((using_ic_dirct_ch == CSI_PRP_VF_MEM) ||
		     (using_ic_dirct_ch == CSI_PRP_ENC_MEM)) {
			ret = -EINVAL;
			goto err;
		}
		using_ic_dirct_ch = MEM_VDI_PRP_VF_MEM;
		ipu_ic_use_count++;
		ipu_vdi_use_count++;
		reg = __raw_readl(IPU_FS_PROC_FLOW1);
		reg &= ~FS_VDI_SRC_SEL_MASK;
		__raw_writel(reg , IPU_FS_PROC_FLOW1);

		if (params->mem_prp_vf_mem.graphics_combine_en)
			g_sec_chan_en[IPU_CHAN_ID(channel)] = true;
		_ipu_ic_init_prpvf(params, false);
		_ipu_vdi_init(channel, params);
		break;
	case MEM_VDI_PRP_VF_MEM_P:
		_ipu_vdi_init(channel, params);
		break;
	case MEM_VDI_PRP_VF_MEM_N:
		_ipu_vdi_init(channel, params);
		break;
	case MEM_ROT_VF_MEM:
		ipu_ic_use_count++;
		ipu_rot_use_count++;
		_ipu_ic_init_rotate_vf(params);
		break;
	case MEM_PRP_ENC_MEM:
		ipu_ic_use_count++;
		reg = __raw_readl(IPU_FS_PROC_FLOW1);
		__raw_writel(reg | FS_ENC_IN_VALID, IPU_FS_PROC_FLOW1);
		_ipu_ic_init_prpenc(params, false);
		break;
	case MEM_ROT_ENC_MEM:
		ipu_ic_use_count++;
		ipu_rot_use_count++;
		_ipu_ic_init_rotate_enc(params);
		break;
	case MEM_PP_MEM:
		if (params->mem_pp_mem.graphics_combine_en)
			g_sec_chan_en[IPU_CHAN_ID(channel)] = true;
		if (params->mem_pp_mem.alpha_chan_en)
			g_thrd_chan_en[IPU_CHAN_ID(channel)] = true;
		_ipu_ic_init_pp(params);
		ipu_ic_use_count++;
		break;
	case MEM_ROT_PP_MEM:
		_ipu_ic_init_rotate_pp(params);
		ipu_ic_use_count++;
		ipu_rot_use_count++;
		break;
	case MEM_DC_SYNC:
		if (params->mem_dc_sync.di > 1) {
			ret = -EINVAL;
			goto err;
		}

		g_dc_di_assignment[1] = params->mem_dc_sync.di;
		_ipu_dc_init(1, params->mem_dc_sync.di,
			     params->mem_dc_sync.interlaced,
			     params->mem_dc_sync.out_pixel_fmt);
		ipu_di_use_count[params->mem_dc_sync.di]++;
		ipu_dc_use_count++;
		ipu_dmfc_use_count++;
		break;
	case MEM_BG_SYNC:
		if (params->mem_dp_bg_sync.di > 1) {
			ret = -EINVAL;
			goto err;
		}

		if (params->mem_dp_bg_sync.alpha_chan_en)
			g_thrd_chan_en[IPU_CHAN_ID(channel)] = true;

		g_dc_di_assignment[5] = params->mem_dp_bg_sync.di;
		_ipu_dp_init(channel, params->mem_dp_bg_sync.in_pixel_fmt,
			     params->mem_dp_bg_sync.out_pixel_fmt);
		_ipu_dc_init(5, params->mem_dp_bg_sync.di,
			     params->mem_dp_bg_sync.interlaced,
			     params->mem_dp_bg_sync.out_pixel_fmt);
		ipu_di_use_count[params->mem_dp_bg_sync.di]++;
		ipu_dc_use_count++;
		ipu_dp_use_count++;
		ipu_dmfc_use_count++;
		break;
	case MEM_FG_SYNC:
		_ipu_dp_init(channel, params->mem_dp_fg_sync.in_pixel_fmt,
			     params->mem_dp_fg_sync.out_pixel_fmt);

		if (params->mem_dp_fg_sync.alpha_chan_en)
			g_thrd_chan_en[IPU_CHAN_ID(channel)] = true;

		ipu_dc_use_count++;
		ipu_dp_use_count++;
		ipu_dmfc_use_count++;
		break;
	case DIRECT_ASYNC0:
		if (params->direct_async.di > 1) {
			ret = -EINVAL;
			goto err;
		}

		g_dc_di_assignment[8] = params->direct_async.di;
		_ipu_dc_init(8, params->direct_async.di, false, IPU_PIX_FMT_GENERIC);
		ipu_di_use_count[params->direct_async.di]++;
		ipu_dc_use_count++;
		break;
	case DIRECT_ASYNC1:
		if (params->direct_async.di > 1) {
			ret = -EINVAL;
			goto err;
		}

		g_dc_di_assignment[9] = params->direct_async.di;
		_ipu_dc_init(9, params->direct_async.di, false, IPU_PIX_FMT_GENERIC);
		ipu_di_use_count[params->direct_async.di]++;
		ipu_dc_use_count++;
		break;
	default:
		dev_err(g_ipu_dev, "Missing channel initialization\n");
		break;
	}

	/* Enable IPU sub module */
	g_channel_init_mask |= 1L << IPU_CHAN_ID(channel);

	__raw_writel(ipu_conf, IPU_CONF);

err:
	spin_unlock_irqrestore(&ipu_lock, lock_flags);
	return ret;
}
EXPORT_SYMBOL(ipu_init_channel);

/*!
 * This function is called to uninitialize a logical IPU channel.
 *
 * @param       channel Input parameter for the logical channel ID to uninit.
 */
void ipu_uninit_channel(ipu_channel_t channel)
{
	unsigned long lock_flags;
	uint32_t reg;
	uint32_t in_dma, out_dma = 0;
	uint32_t ipu_conf;

	if ((g_channel_init_mask & (1L << IPU_CHAN_ID(channel))) == 0) {
		dev_err(g_ipu_dev, "Channel already uninitialized %d\n",
			IPU_CHAN_ID(channel));
		return;
	}

	/* Make sure channel is disabled */
	/* Get input and output dma channels */
	in_dma = channel_2_dma(channel, IPU_VIDEO_IN_BUFFER);
	out_dma = channel_2_dma(channel, IPU_OUTPUT_BUFFER);

	if (idma_is_set(IDMAC_CHA_EN, in_dma) ||
	    idma_is_set(IDMAC_CHA_EN, out_dma)) {
		dev_err(g_ipu_dev,
			"Channel %d is not disabled, disable first\n",
			IPU_CHAN_ID(channel));
		return;
	}

	spin_lock_irqsave(&ipu_lock, lock_flags);

	ipu_conf = __raw_readl(IPU_CONF);

	/* Reset the double buffer */
	reg = __raw_readl(IPU_CHA_DB_MODE_SEL(in_dma));
	__raw_writel(reg & ~idma_mask(in_dma), IPU_CHA_DB_MODE_SEL(in_dma));
	reg = __raw_readl(IPU_CHA_DB_MODE_SEL(out_dma));
	__raw_writel(reg & ~idma_mask(out_dma), IPU_CHA_DB_MODE_SEL(out_dma));

	if (_ipu_is_ic_chan(in_dma) || _ipu_is_dp_graphic_chan(in_dma)) {
		g_sec_chan_en[IPU_CHAN_ID(channel)] = false;
		g_thrd_chan_en[IPU_CHAN_ID(channel)] = false;
	}

	switch (channel) {
	case CSI_MEM0:
	case CSI_MEM1:
	case CSI_MEM2:
	case CSI_MEM3:
		ipu_smfc_use_count--;
		if (g_ipu_csi_channel[0] == channel) {
			g_ipu_csi_channel[0] = CHAN_NONE;
		} else if (g_ipu_csi_channel[1] == channel) {
			g_ipu_csi_channel[1] = CHAN_NONE;
		}
		break;
	case CSI_PRP_ENC_MEM:
		ipu_ic_use_count--;
		if (using_ic_dirct_ch == CSI_PRP_ENC_MEM)
			using_ic_dirct_ch = 0;
		_ipu_ic_uninit_prpenc();
		if (g_ipu_csi_channel[0] == channel) {
			g_ipu_csi_channel[0] = CHAN_NONE;
		} else if (g_ipu_csi_channel[1] == channel) {
			g_ipu_csi_channel[1] = CHAN_NONE;
		}
		break;
	case CSI_PRP_VF_MEM:
		ipu_ic_use_count--;
		if (using_ic_dirct_ch == CSI_PRP_VF_MEM)
			using_ic_dirct_ch = 0;
		_ipu_ic_uninit_prpvf();
		if (g_ipu_csi_channel[0] == channel) {
			g_ipu_csi_channel[0] = CHAN_NONE;
		} else if (g_ipu_csi_channel[1] == channel) {
			g_ipu_csi_channel[1] = CHAN_NONE;
		}
		break;
	case MEM_PRP_VF_MEM:
		ipu_ic_use_count--;
		_ipu_ic_uninit_prpvf();
		reg = __raw_readl(IPU_FS_PROC_FLOW1);
		__raw_writel(reg & ~FS_VF_IN_VALID, IPU_FS_PROC_FLOW1);
		break;
	case MEM_VDI_PRP_VF_MEM:
		ipu_ic_use_count--;
		ipu_vdi_use_count--;
		if (using_ic_dirct_ch == MEM_VDI_PRP_VF_MEM)
			using_ic_dirct_ch = 0;
		_ipu_ic_uninit_prpvf();
		_ipu_vdi_uninit();
		reg = __raw_readl(IPU_FS_PROC_FLOW1);
		__raw_writel(reg & ~FS_VF_IN_VALID, IPU_FS_PROC_FLOW1);
		break;
	case MEM_VDI_PRP_VF_MEM_P:
	case MEM_VDI_PRP_VF_MEM_N:
		break;
	case MEM_ROT_VF_MEM:
		ipu_rot_use_count--;
		ipu_ic_use_count--;
		_ipu_ic_uninit_rotate_vf();
		break;
	case MEM_PRP_ENC_MEM:
		ipu_ic_use_count--;
		_ipu_ic_uninit_prpenc();
		reg = __raw_readl(IPU_FS_PROC_FLOW1);
		__raw_writel(reg & ~FS_ENC_IN_VALID, IPU_FS_PROC_FLOW1);
		break;
	case MEM_ROT_ENC_MEM:
		ipu_rot_use_count--;
		ipu_ic_use_count--;
		_ipu_ic_uninit_rotate_enc();
		break;
	case MEM_PP_MEM:
		ipu_ic_use_count--;
		_ipu_ic_uninit_pp();
		break;
	case MEM_ROT_PP_MEM:
		ipu_rot_use_count--;
		ipu_ic_use_count--;
		_ipu_ic_uninit_rotate_pp();
		break;
	case MEM_DC_SYNC:
		_ipu_dc_uninit(1);
		ipu_di_use_count[g_dc_di_assignment[1]]--;
		ipu_dc_use_count--;
		ipu_dmfc_use_count--;
		break;
	case MEM_BG_SYNC:
		_ipu_dp_uninit(channel);
		_ipu_dc_uninit(5);
		ipu_di_use_count[g_dc_di_assignment[5]]--;
		ipu_dc_use_count--;
		ipu_dp_use_count--;
		ipu_dmfc_use_count--;
		break;
	case MEM_FG_SYNC:
		_ipu_dp_uninit(channel);
		ipu_dc_use_count--;
		ipu_dp_use_count--;
		ipu_dmfc_use_count--;
		break;
	case DIRECT_ASYNC0:
		_ipu_dc_uninit(8);
		ipu_di_use_count[g_dc_di_assignment[8]]--;
		ipu_dc_use_count--;
		break;
	case DIRECT_ASYNC1:
		_ipu_dc_uninit(9);
		ipu_di_use_count[g_dc_di_assignment[9]]--;
		ipu_dc_use_count--;
		break;
	default:
		break;
	}

	g_channel_init_mask &= ~(1L << IPU_CHAN_ID(channel));

	if (ipu_ic_use_count == 0)
		ipu_conf &= ~IPU_CONF_IC_EN;
	if (ipu_vdi_use_count == 0) {
		ipu_conf &= ~IPU_CONF_ISP_EN;
		ipu_conf &= ~IPU_CONF_VDI_EN;
		ipu_conf &= ~IPU_CONF_IC_INPUT;
	}
	if (ipu_rot_use_count == 0)
		ipu_conf &= ~IPU_CONF_ROT_EN;
	if (ipu_dc_use_count == 0)
		ipu_conf &= ~IPU_CONF_DC_EN;
	if (ipu_dp_use_count == 0)
		ipu_conf &= ~IPU_CONF_DP_EN;
	if (ipu_dmfc_use_count == 0)
		ipu_conf &= ~IPU_CONF_DMFC_EN;
	if (ipu_di_use_count[0] == 0) {
		ipu_conf &= ~IPU_CONF_DI0_EN;
	}
	if (ipu_di_use_count[1] == 0) {
		ipu_conf &= ~IPU_CONF_DI1_EN;
	}
	if (ipu_smfc_use_count == 0)
		ipu_conf &= ~IPU_CONF_SMFC_EN;

	__raw_writel(ipu_conf, IPU_CONF);

	spin_unlock_irqrestore(&ipu_lock, lock_flags);

	if (ipu_conf == 0) {
		clk_disable(g_ipu_clk);
		g_ipu_clk_enabled = false;
	}

	WARN_ON(ipu_ic_use_count < 0);
	WARN_ON(ipu_vdi_use_count < 0);
	WARN_ON(ipu_rot_use_count < 0);
	WARN_ON(ipu_dc_use_count < 0);
	WARN_ON(ipu_dp_use_count < 0);
	WARN_ON(ipu_dmfc_use_count < 0);
	WARN_ON(ipu_smfc_use_count < 0);
}
EXPORT_SYMBOL(ipu_uninit_channel);

/*!
 * This function is called to initialize a buffer for logical IPU channel.
 *
 * @param       channel         Input parameter for the logical channel ID.
 *
 * @param       type            Input parameter which buffer to initialize.
 *
 * @param       pixel_fmt       Input parameter for pixel format of buffer.
 *                              Pixel format is a FOURCC ASCII code.
 *
 * @param       width           Input parameter for width of buffer in pixels.
 *
 * @param       height          Input parameter for height of buffer in pixels.
 *
 * @param       stride          Input parameter for stride length of buffer
 *                              in pixels.
 *
 * @param       rot_mode        Input parameter for rotation setting of buffer.
 *                              A rotation setting other than
 *                              IPU_ROTATE_VERT_FLIP
 *                              should only be used for input buffers of
 *                              rotation channels.
 *
 * @param       phyaddr_0       Input parameter buffer 0 physical address.
 *
 * @param       phyaddr_1       Input parameter buffer 1 physical address.
 *                              Setting this to a value other than NULL enables
 *                              double buffering mode.
 *
 * @param       u		private u offset for additional cropping,
 *				zero if not used.
 *
 * @param       v		private v offset for additional cropping,
 *				zero if not used.
 *
 * @return      Returns 0 on success or negative error code on fail
 */
int32_t ipu_init_channel_buffer(ipu_channel_t channel, ipu_buffer_t type,
				uint32_t pixel_fmt,
				uint16_t width, uint16_t height,
				uint32_t stride,
				ipu_rotate_mode_t rot_mode,
				dma_addr_t phyaddr_0, dma_addr_t phyaddr_1,
				uint32_t u, uint32_t v)
{
	unsigned long lock_flags;
	uint32_t reg;
	uint32_t dma_chan;
	uint32_t burst_size;

	dma_chan = channel_2_dma(channel, type);
	if (!idma_is_valid(dma_chan))
		return -EINVAL;

	if (stride < width * bytes_per_pixel(pixel_fmt))
		stride = width * bytes_per_pixel(pixel_fmt);

	if (stride % 4) {
		dev_err(g_ipu_dev,
			"Stride not 32-bit aligned, stride = %d\n", stride);
		return -EINVAL;
	}
	/* IC & IRT channels' width must be multiple of 8 pixels */
	if ((_ipu_is_ic_chan(dma_chan) || _ipu_is_irt_chan(dma_chan))
		&& (width % 8)) {
		dev_err(g_ipu_dev, "Width must be 8 pixel multiple\n");
		return -EINVAL;
	}

	/* Build parameter memory data for DMA channel */
	_ipu_ch_param_init(dma_chan, pixel_fmt, width, height, stride, u, v, 0,
			   phyaddr_0, phyaddr_1);

	/* Set correlative channel parameter of local alpha channel */
	if ((_ipu_is_ic_graphic_chan(dma_chan) ||
	     _ipu_is_dp_graphic_chan(dma_chan)) &&
	    (g_thrd_chan_en[IPU_CHAN_ID(channel)] == true)) {
		_ipu_ch_param_set_alpha_use_separate_channel(dma_chan, true);
		_ipu_ch_param_set_alpha_buffer_memory(dma_chan);
		_ipu_ch_param_set_alpha_condition_read(dma_chan);
		/* fix alpha width as 8 and burst size as 16*/
		_ipu_ch_params_set_alpha_width(dma_chan, 8);
		_ipu_ch_param_set_burst_size(dma_chan, 16);
	} else if (_ipu_is_ic_graphic_chan(dma_chan) &&
		   ipu_pixel_format_has_alpha(pixel_fmt))
		_ipu_ch_param_set_alpha_use_separate_channel(dma_chan, false);

	if (rot_mode)
		_ipu_ch_param_set_rotation(dma_chan, rot_mode);

	/* IC and ROT channels have restriction of 8 or 16 pix burst length */
	if (_ipu_is_ic_chan(dma_chan)) {
		if ((width % 16) == 0)
			_ipu_ch_param_set_burst_size(dma_chan, 16);
		else
			_ipu_ch_param_set_burst_size(dma_chan, 8);
	} else if (_ipu_is_irt_chan(dma_chan)) {
		_ipu_ch_param_set_burst_size(dma_chan, 8);
		_ipu_ch_param_set_block_mode(dma_chan);
	} else if (_ipu_is_dmfc_chan(dma_chan)) {
		u32 dmfc_dp_chan, dmfc_wr_chan;
		/*
		 * non-interleaving format need enlarge burst size
		 * to work-around black flash issue.
		 */
		if (((dma_chan == 23) || (dma_chan == 27) || (dma_chan == 28))
			&& ((pixel_fmt == IPU_PIX_FMT_YUV420P) ||
			(pixel_fmt == IPU_PIX_FMT_YUV420P2) ||
			(pixel_fmt == IPU_PIX_FMT_YVU422P) ||
			(pixel_fmt == IPU_PIX_FMT_YUV422P) ||
			(pixel_fmt == IPU_PIX_FMT_NV12))) {
			if (dma_chan == 23) {
				dmfc_dp_chan = __raw_readl(DMFC_DP_CHAN);
				dmfc_dp_chan &= ~(0xc0);
				dmfc_dp_chan |= 0x40;
				__raw_writel(dmfc_dp_chan, DMFC_DP_CHAN);
			} else if (dma_chan == 27) {
				dmfc_dp_chan = __raw_readl(DMFC_DP_CHAN);
				dmfc_dp_chan &= ~(0xc000);
				dmfc_dp_chan |= 0x4000;
				__raw_writel(dmfc_dp_chan, DMFC_DP_CHAN);
			} else if (dma_chan == 28) {
				dmfc_wr_chan = __raw_readl(DMFC_WR_CHAN);
				dmfc_wr_chan &= ~(0xc0);
				dmfc_wr_chan |= 0x40;
				__raw_writel(dmfc_wr_chan, DMFC_WR_CHAN);
			}
			_ipu_ch_param_set_burst_size(dma_chan, 64);
		} else {
			if (dma_chan == 23) {
				dmfc_dp_chan = __raw_readl(DMFC_DP_CHAN);
				dmfc_dp_chan &= ~(0xc0);
				dmfc_dp_chan |= 0x80;
				__raw_writel(dmfc_dp_chan, DMFC_DP_CHAN);
			} else if (dma_chan == 27) {
				dmfc_dp_chan = __raw_readl(DMFC_DP_CHAN);
				dmfc_dp_chan &= ~(0xc000);
				dmfc_dp_chan |= 0x8000;
				__raw_writel(dmfc_dp_chan, DMFC_DP_CHAN);
			} else {
				dmfc_wr_chan = __raw_readl(DMFC_WR_CHAN);
				dmfc_wr_chan &= ~(0xc0);
				dmfc_wr_chan |= 0x80;
				__raw_writel(dmfc_wr_chan, DMFC_WR_CHAN);
			}
		}
		spin_lock_irqsave(&ipu_lock, lock_flags);
		_ipu_dmfc_set_wait4eot(dma_chan, width);
		spin_unlock_irqrestore(&ipu_lock, lock_flags);
	}

	if (_ipu_disp_chan_is_interlaced(channel) ||
		g_chan_is_interlaced[dma_chan])
		_ipu_ch_param_set_interlaced_scan(dma_chan);

	if (_ipu_is_ic_chan(dma_chan) || _ipu_is_irt_chan(dma_chan)) {
		burst_size = _ipu_ch_param_get_burst_size(dma_chan);
		_ipu_ic_idma_init(dma_chan, width, height, burst_size,
			rot_mode);
	} else if (_ipu_is_smfc_chan(dma_chan)) {
		burst_size = _ipu_ch_param_get_burst_size(dma_chan);
		if ((pixel_fmt == IPU_PIX_FMT_GENERIC) &&
			((_ipu_ch_param_get_bpp(dma_chan) == 5) ||
			(_ipu_ch_param_get_bpp(dma_chan) == 3)))
			burst_size = burst_size >> 4;
		else
			burst_size = burst_size >> 2;
		_ipu_smfc_set_burst_size(channel, burst_size-1);
	}

	if (idma_is_set(IDMAC_CHA_PRI, dma_chan) && !cpu_is_mx53())
		_ipu_ch_param_set_high_priority(dma_chan);

	_ipu_ch_param_dump(dma_chan);

	spin_lock_irqsave(&ipu_lock, lock_flags);

	reg = __raw_readl(IPU_CHA_DB_MODE_SEL(dma_chan));
	if (phyaddr_1)
		reg |= idma_mask(dma_chan);
	else
		reg &= ~idma_mask(dma_chan);
	__raw_writel(reg, IPU_CHA_DB_MODE_SEL(dma_chan));

	/* Reset to buffer 0 */
	__raw_writel(idma_mask(dma_chan), IPU_CHA_CUR_BUF(dma_chan));

	spin_unlock_irqrestore(&ipu_lock, lock_flags);

	return 0;
}
EXPORT_SYMBOL(ipu_init_channel_buffer);

/*!
 * This function is called to update the physical address of a buffer for
 * a logical IPU channel.
 *
 * @param       channel         Input parameter for the logical channel ID.
 *
 * @param       type            Input parameter which buffer to initialize.
 *
 * @param       bufNum          Input parameter for buffer number to update.
 *                              0 or 1 are the only valid values.
 *
 * @param       phyaddr         Input parameter buffer physical address.
 *
 * @return      This function returns 0 on success or negative error code on
 *              fail. This function will fail if the buffer is set to ready.
 */
int32_t ipu_update_channel_buffer(ipu_channel_t channel, ipu_buffer_t type,
				  uint32_t bufNum, dma_addr_t phyaddr)
{
	uint32_t reg;
	int ret = 0;
	unsigned long lock_flags;
	uint32_t dma_chan = channel_2_dma(channel, type);
	if (dma_chan == IDMA_CHAN_INVALID)
		return -EINVAL;

	spin_lock_irqsave(&ipu_lock, lock_flags);

	if (bufNum == 0)
		reg = __raw_readl(IPU_CHA_BUF0_RDY(dma_chan));
	else
		reg = __raw_readl(IPU_CHA_BUF1_RDY(dma_chan));

	if ((reg & idma_mask(dma_chan)) == 0)
		_ipu_ch_param_set_buffer(dma_chan, bufNum, phyaddr);
	else
		ret = -EACCES;

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
	return ret;
}
EXPORT_SYMBOL(ipu_update_channel_buffer);


/*!
 * This function is called to initialize a buffer for logical IPU channel.
 *
 * @param       channel         Input parameter for the logical channel ID.
 *
 * @param       type            Input parameter which buffer to initialize.
 *
 * @param       pixel_fmt       Input parameter for pixel format of buffer.
 *                              Pixel format is a FOURCC ASCII code.
 *
 * @param       width           Input parameter for width of buffer in pixels.
 *
 * @param       height          Input parameter for height of buffer in pixels.
 *
 * @param       stride          Input parameter for stride length of buffer
 *                              in pixels.
 *
 * @param       u		predefined private u offset for additional cropping,
 *								zero if not used.
 *
 * @param       v		predefined private v offset for additional cropping,
 *								zero if not used.
 *
 * @param			vertical_offset vertical offset for Y coordinate
 * 								in the existed frame
 *
 *
 * @param			horizontal_offset horizontal offset for X coordinate
 * 								in the existed frame
 *
 *
 * @return      Returns 0 on success or negative error code on fail
 *              This function will fail if any buffer is set to ready.
 */

int32_t ipu_update_channel_offset(ipu_channel_t channel, ipu_buffer_t type,
				uint32_t pixel_fmt,
				uint16_t width, uint16_t height,
				uint32_t stride,
				uint32_t u, uint32_t v,
				uint32_t vertical_offset, uint32_t horizontal_offset)
{
	int ret = 0;
	unsigned long lock_flags;
	uint32_t dma_chan = channel_2_dma(channel, type);

	if (dma_chan == IDMA_CHAN_INVALID)
		return -EINVAL;

	spin_lock_irqsave(&ipu_lock, lock_flags);

	if ((__raw_readl(IPU_CHA_BUF0_RDY(dma_chan)) & idma_mask(dma_chan)) ||
		(__raw_readl(IPU_CHA_BUF0_RDY(dma_chan)) & idma_mask(dma_chan)))
		ret = -EACCES;
	else
		_ipu_ch_offset_update(dma_chan, pixel_fmt, width, height, stride,
				      u, v, 0, vertical_offset, horizontal_offset);

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
	return ret;
}
EXPORT_SYMBOL(ipu_update_channel_offset);


/*!
 * This function is called to set a channel's buffer as ready.
 *
 * @param       channel         Input parameter for the logical channel ID.
 *
 * @param       type            Input parameter which buffer to initialize.
 *
 * @param       bufNum          Input parameter for which buffer number set to
 *                              ready state.
 *
 * @return      Returns 0 on success or negative error code on fail
 */
int32_t ipu_select_buffer(ipu_channel_t channel, ipu_buffer_t type,
			  uint32_t bufNum)
{
	uint32_t dma_chan = channel_2_dma(channel, type);
	uint32_t reg;

	if (dma_chan == IDMA_CHAN_INVALID)
		return -EINVAL;

	if (bufNum == 0) {
		/*Mark buffer 0 as ready. */
		reg = __raw_readl(IPU_CHA_BUF0_RDY(dma_chan));
		__raw_writel(idma_mask(dma_chan) | reg,
			     IPU_CHA_BUF0_RDY(dma_chan));
	} else {
		/*Mark buffer 1 as ready. */
		reg = __raw_readl(IPU_CHA_BUF1_RDY(dma_chan));
		__raw_writel(idma_mask(dma_chan) | reg,
			     IPU_CHA_BUF1_RDY(dma_chan));
	}
	return 0;
}
EXPORT_SYMBOL(ipu_select_buffer);

/*!
 * This function is called to set a channel's buffer as ready.
 *
 * @param       bufNum          Input parameter for which buffer number set to
 *                              ready state.
 *
 * @return      Returns 0 on success or negative error code on fail
 */
int32_t ipu_select_multi_vdi_buffer(uint32_t bufNum)
{

	uint32_t dma_chan = channel_2_dma(MEM_VDI_PRP_VF_MEM, IPU_INPUT_BUFFER);
	uint32_t mask_bit =
		idma_mask(channel_2_dma(MEM_VDI_PRP_VF_MEM_P, IPU_INPUT_BUFFER))|
		idma_mask(dma_chan)|
		idma_mask(channel_2_dma(MEM_VDI_PRP_VF_MEM_N, IPU_INPUT_BUFFER));
	uint32_t reg;

	if (bufNum == 0) {
		/*Mark buffer 0 as ready. */
		reg = __raw_readl(IPU_CHA_BUF0_RDY(dma_chan));
		__raw_writel(mask_bit | reg, IPU_CHA_BUF0_RDY(dma_chan));
	} else {
		/*Mark buffer 1 as ready. */
		reg = __raw_readl(IPU_CHA_BUF1_RDY(dma_chan));
		__raw_writel(mask_bit | reg, IPU_CHA_BUF1_RDY(dma_chan));
	}
	return 0;
}
EXPORT_SYMBOL(ipu_select_multi_vdi_buffer);

#define NA	-1
static int proc_dest_sel[] = {
	0, 1, 1, 3, 5, 5, 4, 7, 8, 9, 10, 11, 12, 14, 15, 16,
	0, 1, 1, 5, 5, 5, 5, 5, 7, 8, 9, 10, 11, 12, 14, 31 };
static int proc_src_sel[] = { 0, 6, 7, 6, 7, 8, 5, NA, NA, NA,
  NA, NA, NA, NA, NA,  1,  2,  3,  4,  7,  8, NA, 8, NA };
static int disp_src_sel[] = { 0, 6, 7, 8, 3, 4, 5, NA, NA, NA,
  NA, NA, NA, NA, NA,  1, NA,  2, NA,  3,  4,  4,  4,  4 };


/*!
 * This function links 2 channels together for automatic frame
 * synchronization. The output of the source channel is linked to the input of
 * the destination channel.
 *
 * @param       src_ch          Input parameter for the logical channel ID of
 *                              the source channel.
 *
 * @param       dest_ch         Input parameter for the logical channel ID of
 *                              the destination channel.
 *
 * @return      This function returns 0 on success or negative error code on
 *              fail.
 */
int32_t ipu_link_channels(ipu_channel_t src_ch, ipu_channel_t dest_ch)
{
	int retval = 0;
	unsigned long lock_flags;
	uint32_t fs_proc_flow1;
	uint32_t fs_proc_flow2;
	uint32_t fs_proc_flow3;
	uint32_t fs_disp_flow1;

	spin_lock_irqsave(&ipu_lock, lock_flags);

	fs_proc_flow1 = __raw_readl(IPU_FS_PROC_FLOW1);
	fs_proc_flow2 = __raw_readl(IPU_FS_PROC_FLOW2);
	fs_proc_flow3 = __raw_readl(IPU_FS_PROC_FLOW3);
	fs_disp_flow1 = __raw_readl(IPU_FS_DISP_FLOW1);

	switch (src_ch) {
	case CSI_MEM0:
		fs_proc_flow3 &= ~FS_SMFC0_DEST_SEL_MASK;
		fs_proc_flow3 |=
			proc_dest_sel[IPU_CHAN_ID(dest_ch)] <<
			FS_SMFC0_DEST_SEL_OFFSET;
		break;
	case CSI_MEM1:
		fs_proc_flow3 &= ~FS_SMFC1_DEST_SEL_MASK;
		fs_proc_flow3 |=
			proc_dest_sel[IPU_CHAN_ID(dest_ch)] <<
			FS_SMFC1_DEST_SEL_OFFSET;
		break;
	case CSI_MEM2:
		fs_proc_flow3 &= ~FS_SMFC2_DEST_SEL_MASK;
		fs_proc_flow3 |=
			proc_dest_sel[IPU_CHAN_ID(dest_ch)] <<
			FS_SMFC2_DEST_SEL_OFFSET;
		break;
	case CSI_MEM3:
		fs_proc_flow3 &= ~FS_SMFC3_DEST_SEL_MASK;
		fs_proc_flow3 |=
			proc_dest_sel[IPU_CHAN_ID(dest_ch)] <<
			FS_SMFC3_DEST_SEL_OFFSET;
		break;
	case CSI_PRP_ENC_MEM:
		fs_proc_flow2 &= ~FS_PRPENC_DEST_SEL_MASK;
		fs_proc_flow2 |=
			proc_dest_sel[IPU_CHAN_ID(dest_ch)] <<
			FS_PRPENC_DEST_SEL_OFFSET;
		break;
	case CSI_PRP_VF_MEM:
		fs_proc_flow2 &= ~FS_PRPVF_DEST_SEL_MASK;
		fs_proc_flow2 |=
			proc_dest_sel[IPU_CHAN_ID(dest_ch)] <<
			FS_PRPVF_DEST_SEL_OFFSET;
		break;
	case MEM_PP_MEM:
		fs_proc_flow2 &= ~FS_PP_DEST_SEL_MASK;
		fs_proc_flow2 |=
		    proc_dest_sel[IPU_CHAN_ID(dest_ch)] <<
		    FS_PP_DEST_SEL_OFFSET;
		break;
	case MEM_ROT_PP_MEM:
		fs_proc_flow2 &= ~FS_PP_ROT_DEST_SEL_MASK;
		fs_proc_flow2 |=
		    proc_dest_sel[IPU_CHAN_ID(dest_ch)] <<
		    FS_PP_ROT_DEST_SEL_OFFSET;
		break;
	case MEM_PRP_ENC_MEM:
		fs_proc_flow2 &= ~FS_PRPENC_DEST_SEL_MASK;
		fs_proc_flow2 |=
		    proc_dest_sel[IPU_CHAN_ID(dest_ch)] <<
		    FS_PRPENC_DEST_SEL_OFFSET;
		break;
	case MEM_ROT_ENC_MEM:
		fs_proc_flow2 &= ~FS_PRPENC_ROT_DEST_SEL_MASK;
		fs_proc_flow2 |=
		    proc_dest_sel[IPU_CHAN_ID(dest_ch)] <<
		    FS_PRPENC_ROT_DEST_SEL_OFFSET;
		break;
	case MEM_PRP_VF_MEM:
		fs_proc_flow2 &= ~FS_PRPVF_DEST_SEL_MASK;
		fs_proc_flow2 |=
		    proc_dest_sel[IPU_CHAN_ID(dest_ch)] <<
		    FS_PRPVF_DEST_SEL_OFFSET;
		break;
	case MEM_VDI_PRP_VF_MEM:
		fs_proc_flow2 &= ~FS_PRPVF_DEST_SEL_MASK;
		fs_proc_flow2 |=
		    proc_dest_sel[IPU_CHAN_ID(dest_ch)] <<
		    FS_PRPVF_DEST_SEL_OFFSET;
		break;
	case MEM_ROT_VF_MEM:
		fs_proc_flow2 &= ~FS_PRPVF_ROT_DEST_SEL_MASK;
		fs_proc_flow2 |=
		    proc_dest_sel[IPU_CHAN_ID(dest_ch)] <<
		    FS_PRPVF_ROT_DEST_SEL_OFFSET;
		break;
	default:
		retval = -EINVAL;
		goto err;
	}

	switch (dest_ch) {
	case MEM_PP_MEM:
		fs_proc_flow1 &= ~FS_PP_SRC_SEL_MASK;
		fs_proc_flow1 |=
		    proc_src_sel[IPU_CHAN_ID(src_ch)] << FS_PP_SRC_SEL_OFFSET;
		break;
	case MEM_ROT_PP_MEM:
		fs_proc_flow1 &= ~FS_PP_ROT_SRC_SEL_MASK;
		fs_proc_flow1 |=
		    proc_src_sel[IPU_CHAN_ID(src_ch)] <<
		    FS_PP_ROT_SRC_SEL_OFFSET;
		break;
	case MEM_PRP_ENC_MEM:
		fs_proc_flow1 &= ~FS_PRP_SRC_SEL_MASK;
		fs_proc_flow1 |=
		    proc_src_sel[IPU_CHAN_ID(src_ch)] << FS_PRP_SRC_SEL_OFFSET;
		break;
	case MEM_ROT_ENC_MEM:
		fs_proc_flow1 &= ~FS_PRPENC_ROT_SRC_SEL_MASK;
		fs_proc_flow1 |=
		    proc_src_sel[IPU_CHAN_ID(src_ch)] <<
		    FS_PRPENC_ROT_SRC_SEL_OFFSET;
		break;
	case MEM_PRP_VF_MEM:
		fs_proc_flow1 &= ~FS_PRP_SRC_SEL_MASK;
		fs_proc_flow1 |=
		    proc_src_sel[IPU_CHAN_ID(src_ch)] << FS_PRP_SRC_SEL_OFFSET;
		break;
	case MEM_VDI_PRP_VF_MEM:
		fs_proc_flow1 &= ~FS_PRP_SRC_SEL_MASK;
		fs_proc_flow1 |=
		    proc_src_sel[IPU_CHAN_ID(src_ch)] << FS_PRP_SRC_SEL_OFFSET;
		break;
	case MEM_ROT_VF_MEM:
		fs_proc_flow1 &= ~FS_PRPVF_ROT_SRC_SEL_MASK;
		fs_proc_flow1 |=
		    proc_src_sel[IPU_CHAN_ID(src_ch)] <<
		    FS_PRPVF_ROT_SRC_SEL_OFFSET;
		break;
	case MEM_DC_SYNC:
		fs_disp_flow1 &= ~FS_DC1_SRC_SEL_MASK;
		fs_disp_flow1 |=
		    disp_src_sel[IPU_CHAN_ID(src_ch)] << FS_DC1_SRC_SEL_OFFSET;
		break;
	case MEM_BG_SYNC:
		fs_disp_flow1 &= ~FS_DP_SYNC0_SRC_SEL_MASK;
		fs_disp_flow1 |=
		    disp_src_sel[IPU_CHAN_ID(src_ch)] <<
		    FS_DP_SYNC0_SRC_SEL_OFFSET;
		break;
	case MEM_FG_SYNC:
		fs_disp_flow1 &= ~FS_DP_SYNC1_SRC_SEL_MASK;
		fs_disp_flow1 |=
		    disp_src_sel[IPU_CHAN_ID(src_ch)] <<
		    FS_DP_SYNC1_SRC_SEL_OFFSET;
		break;
	case MEM_DC_ASYNC:
		fs_disp_flow1 &= ~FS_DC2_SRC_SEL_MASK;
		fs_disp_flow1 |=
		    disp_src_sel[IPU_CHAN_ID(src_ch)] << FS_DC2_SRC_SEL_OFFSET;
		break;
	case MEM_BG_ASYNC0:
		fs_disp_flow1 &= ~FS_DP_ASYNC0_SRC_SEL_MASK;
		fs_disp_flow1 |=
		    disp_src_sel[IPU_CHAN_ID(src_ch)] <<
		    FS_DP_ASYNC0_SRC_SEL_OFFSET;
		break;
	case MEM_FG_ASYNC0:
		fs_disp_flow1 &= ~FS_DP_ASYNC1_SRC_SEL_MASK;
		fs_disp_flow1 |=
		    disp_src_sel[IPU_CHAN_ID(src_ch)] <<
		    FS_DP_ASYNC1_SRC_SEL_OFFSET;
		break;
	default:
		retval = -EINVAL;
		goto err;
	}

	__raw_writel(fs_proc_flow1, IPU_FS_PROC_FLOW1);
	__raw_writel(fs_proc_flow2, IPU_FS_PROC_FLOW2);
	__raw_writel(fs_proc_flow3, IPU_FS_PROC_FLOW3);
	__raw_writel(fs_disp_flow1, IPU_FS_DISP_FLOW1);

err:
	spin_unlock_irqrestore(&ipu_lock, lock_flags);
	return retval;
}
EXPORT_SYMBOL(ipu_link_channels);

/*!
 * This function unlinks 2 channels and disables automatic frame
 * synchronization.
 *
 * @param       src_ch          Input parameter for the logical channel ID of
 *                              the source channel.
 *
 * @param       dest_ch         Input parameter for the logical channel ID of
 *                              the destination channel.
 *
 * @return      This function returns 0 on success or negative error code on
 *              fail.
 */
int32_t ipu_unlink_channels(ipu_channel_t src_ch, ipu_channel_t dest_ch)
{
	int retval = 0;
	unsigned long lock_flags;
	uint32_t fs_proc_flow1;
	uint32_t fs_proc_flow2;
	uint32_t fs_proc_flow3;
	uint32_t fs_disp_flow1;

	spin_lock_irqsave(&ipu_lock, lock_flags);

	fs_proc_flow1 = __raw_readl(IPU_FS_PROC_FLOW1);
	fs_proc_flow2 = __raw_readl(IPU_FS_PROC_FLOW2);
	fs_proc_flow3 = __raw_readl(IPU_FS_PROC_FLOW3);
	fs_disp_flow1 = __raw_readl(IPU_FS_DISP_FLOW1);

	switch (src_ch) {
	case CSI_MEM0:
		fs_proc_flow3 &= ~FS_SMFC0_DEST_SEL_MASK;
		break;
	case CSI_MEM1:
		fs_proc_flow3 &= ~FS_SMFC1_DEST_SEL_MASK;
		break;
	case CSI_MEM2:
		fs_proc_flow3 &= ~FS_SMFC2_DEST_SEL_MASK;
		break;
	case CSI_MEM3:
		fs_proc_flow3 &= ~FS_SMFC3_DEST_SEL_MASK;
		break;
	case CSI_PRP_ENC_MEM:
		fs_proc_flow2 &= ~FS_PRPENC_DEST_SEL_MASK;
		break;
	case CSI_PRP_VF_MEM:
		fs_proc_flow2 &= ~FS_PRPVF_DEST_SEL_MASK;
		break;
	case MEM_PP_MEM:
		fs_proc_flow2 &= ~FS_PP_DEST_SEL_MASK;
		break;
	case MEM_ROT_PP_MEM:
		fs_proc_flow2 &= ~FS_PP_ROT_DEST_SEL_MASK;
		break;
	case MEM_PRP_ENC_MEM:
		fs_proc_flow2 &= ~FS_PRPENC_DEST_SEL_MASK;
		break;
	case MEM_ROT_ENC_MEM:
		fs_proc_flow2 &= ~FS_PRPENC_ROT_DEST_SEL_MASK;
		break;
	case MEM_PRP_VF_MEM:
		fs_proc_flow2 &= ~FS_PRPVF_DEST_SEL_MASK;
		break;
	case MEM_VDI_PRP_VF_MEM:
		fs_proc_flow2 &= ~FS_PRPVF_DEST_SEL_MASK;
		break;
	case MEM_ROT_VF_MEM:
		fs_proc_flow2 &= ~FS_PRPVF_ROT_DEST_SEL_MASK;
		break;
	default:
		retval = -EINVAL;
		goto err;
	}

	switch (dest_ch) {
	case MEM_PP_MEM:
		fs_proc_flow1 &= ~FS_PP_SRC_SEL_MASK;
		break;
	case MEM_ROT_PP_MEM:
		fs_proc_flow1 &= ~FS_PP_ROT_SRC_SEL_MASK;
		break;
	case MEM_PRP_ENC_MEM:
		fs_proc_flow1 &= ~FS_PRP_SRC_SEL_MASK;
		break;
	case MEM_ROT_ENC_MEM:
		fs_proc_flow1 &= ~FS_PRPENC_ROT_SRC_SEL_MASK;
		break;
	case MEM_PRP_VF_MEM:
		fs_proc_flow1 &= ~FS_PRP_SRC_SEL_MASK;
		break;
	case MEM_VDI_PRP_VF_MEM:
		fs_proc_flow1 &= ~FS_PRP_SRC_SEL_MASK;
		break;
	case MEM_ROT_VF_MEM:
		fs_proc_flow1 &= ~FS_PRPVF_ROT_SRC_SEL_MASK;
		break;
	case MEM_DC_SYNC:
		fs_disp_flow1 &= ~FS_DC1_SRC_SEL_MASK;
		break;
	case MEM_BG_SYNC:
		fs_disp_flow1 &= ~FS_DP_SYNC0_SRC_SEL_MASK;
		break;
	case MEM_FG_SYNC:
		fs_disp_flow1 &= ~FS_DP_SYNC1_SRC_SEL_MASK;
		break;
	case MEM_DC_ASYNC:
		fs_disp_flow1 &= ~FS_DC2_SRC_SEL_MASK;
		break;
	case MEM_BG_ASYNC0:
		fs_disp_flow1 &= ~FS_DP_ASYNC0_SRC_SEL_MASK;
		break;
	case MEM_FG_ASYNC0:
		fs_disp_flow1 &= ~FS_DP_ASYNC1_SRC_SEL_MASK;
		break;
	default:
		retval = -EINVAL;
		goto err;
	}

	__raw_writel(fs_proc_flow1, IPU_FS_PROC_FLOW1);
	__raw_writel(fs_proc_flow2, IPU_FS_PROC_FLOW2);
	__raw_writel(fs_proc_flow3, IPU_FS_PROC_FLOW3);
	__raw_writel(fs_disp_flow1, IPU_FS_DISP_FLOW1);

err:
	spin_unlock_irqrestore(&ipu_lock, lock_flags);
	return retval;
}
EXPORT_SYMBOL(ipu_unlink_channels);

/*!
 * This function check whether a logical channel was enabled.
 *
 * @param       channel         Input parameter for the logical channel ID.
 *
 * @return      This function returns 1 while request channel is enabled or
 *              0 for not enabled.
 */
int32_t ipu_is_channel_busy(ipu_channel_t channel)
{
	uint32_t reg;
	uint32_t in_dma;
	uint32_t out_dma;

	out_dma = channel_2_dma(channel, IPU_OUTPUT_BUFFER);
	in_dma = channel_2_dma(channel, IPU_VIDEO_IN_BUFFER);

	reg = __raw_readl(IDMAC_CHA_EN(in_dma));
	if (reg & idma_mask(in_dma))
		return 1;
	reg = __raw_readl(IDMAC_CHA_EN(out_dma));
	if (reg & idma_mask(out_dma))
		return 1;
	return 0;
}
EXPORT_SYMBOL(ipu_is_channel_busy);

/*!
 * This function enables a logical channel.
 *
 * @param       channel         Input parameter for the logical channel ID.
 *
 * @return      This function returns 0 on success or negative error code on
 *              fail.
 */
int32_t ipu_enable_channel(ipu_channel_t channel)
{
	uint32_t reg;
	unsigned long lock_flags;
	uint32_t ipu_conf;
	uint32_t in_dma;
	uint32_t out_dma;
	uint32_t sec_dma;
	uint32_t thrd_dma;

	if (g_channel_enable_mask & (1L << IPU_CHAN_ID(channel))) {
		dev_err(g_ipu_dev, "Warning: channel already enabled %d\n",
			IPU_CHAN_ID(channel));
	}

	/* Get input and output dma channels */
	out_dma = channel_2_dma(channel, IPU_OUTPUT_BUFFER);
	in_dma = channel_2_dma(channel, IPU_VIDEO_IN_BUFFER);

	spin_lock_irqsave(&ipu_lock, lock_flags);

	ipu_conf = __raw_readl(IPU_CONF);
	if (ipu_di_use_count[0] > 0) {
		ipu_conf |= IPU_CONF_DI0_EN;
	}
	if (ipu_di_use_count[1] > 0) {
		ipu_conf |= IPU_CONF_DI1_EN;
	}
	if (ipu_dp_use_count > 0)
		ipu_conf |= IPU_CONF_DP_EN;
	if (ipu_dc_use_count > 0)
		ipu_conf |= IPU_CONF_DC_EN;
	if (ipu_dmfc_use_count > 0)
		ipu_conf |= IPU_CONF_DMFC_EN;
	if (ipu_ic_use_count > 0)
		ipu_conf |= IPU_CONF_IC_EN;
	if (ipu_vdi_use_count > 0) {
		ipu_conf |= IPU_CONF_ISP_EN;
		ipu_conf |= IPU_CONF_VDI_EN;
		ipu_conf |= IPU_CONF_IC_INPUT;
	}
	if (ipu_rot_use_count > 0)
		ipu_conf |= IPU_CONF_ROT_EN;
	if (ipu_smfc_use_count > 0)
		ipu_conf |= IPU_CONF_SMFC_EN;
	__raw_writel(ipu_conf, IPU_CONF);

	if (idma_is_valid(in_dma)) {
		reg = __raw_readl(IDMAC_CHA_EN(in_dma));
		__raw_writel(reg | idma_mask(in_dma), IDMAC_CHA_EN(in_dma));
	}
	if (idma_is_valid(out_dma)) {
		reg = __raw_readl(IDMAC_CHA_EN(out_dma));
		__raw_writel(reg | idma_mask(out_dma), IDMAC_CHA_EN(out_dma));
	}

	if ((g_sec_chan_en[IPU_CHAN_ID(channel)]) &&
		((channel == MEM_PP_MEM) || (channel == MEM_PRP_VF_MEM) ||
		 (channel == MEM_VDI_PRP_VF_MEM))) {
		sec_dma = channel_2_dma(channel, IPU_GRAPH_IN_BUFFER);
		reg = __raw_readl(IDMAC_CHA_EN(sec_dma));
		__raw_writel(reg | idma_mask(sec_dma), IDMAC_CHA_EN(sec_dma));
	}
	if ((g_thrd_chan_en[IPU_CHAN_ID(channel)]) &&
		((channel == MEM_PP_MEM) || (channel == MEM_PRP_VF_MEM))) {
		thrd_dma = channel_2_dma(channel, IPU_ALPHA_IN_BUFFER);
		reg = __raw_readl(IDMAC_CHA_EN(thrd_dma));
		__raw_writel(reg | idma_mask(thrd_dma), IDMAC_CHA_EN(thrd_dma));

		sec_dma = channel_2_dma(channel, IPU_GRAPH_IN_BUFFER);
		reg = __raw_readl(IDMAC_SEP_ALPHA);
		__raw_writel(reg | idma_mask(sec_dma), IDMAC_SEP_ALPHA);
	} else if ((g_thrd_chan_en[IPU_CHAN_ID(channel)]) &&
		   ((channel == MEM_BG_SYNC) || (channel == MEM_FG_SYNC))) {
		thrd_dma = channel_2_dma(channel, IPU_ALPHA_IN_BUFFER);
		reg = __raw_readl(IDMAC_CHA_EN(thrd_dma));
		__raw_writel(reg | idma_mask(thrd_dma), IDMAC_CHA_EN(thrd_dma));
		reg = __raw_readl(IDMAC_SEP_ALPHA);
		__raw_writel(reg | idma_mask(in_dma), IDMAC_SEP_ALPHA);
	}

	if ((channel == MEM_DC_SYNC) || (channel == MEM_BG_SYNC) ||
	    (channel == MEM_FG_SYNC)) {
		reg = __raw_readl(IDMAC_WM_EN(in_dma));
		__raw_writel(reg | idma_mask(in_dma), IDMAC_WM_EN(in_dma));

		_ipu_dp_dc_enable(channel);
	}

	if (_ipu_is_ic_chan(in_dma) || _ipu_is_ic_chan(out_dma) ||
		_ipu_is_irt_chan(in_dma) || _ipu_is_irt_chan(out_dma))
		_ipu_ic_enable_task(channel);

	g_channel_enable_mask |= 1L << IPU_CHAN_ID(channel);

	spin_unlock_irqrestore(&ipu_lock, lock_flags);

	return 0;
}
EXPORT_SYMBOL(ipu_enable_channel);

/*!
 * This function check buffer ready for a logical channel.
 *
 * @param       channel         Input parameter for the logical channel ID.
 *
 * @param       type            Input parameter which buffer to clear.
 *
 * @param       bufNum          Input parameter for which buffer number clear
 * 				ready state.
 *
 */
int32_t ipu_check_buffer_busy(ipu_channel_t channel, ipu_buffer_t type,
		uint32_t bufNum)
{
	uint32_t dma_chan = channel_2_dma(channel, type);
	uint32_t reg;

	if (dma_chan == IDMA_CHAN_INVALID)
		return -EINVAL;

	if (bufNum == 0)
		reg = __raw_readl(IPU_CHA_BUF0_RDY(dma_chan));
	else
		reg = __raw_readl(IPU_CHA_BUF1_RDY(dma_chan));

	if (reg & idma_mask(dma_chan))
		return 1;
	else
		return 0;
}
EXPORT_SYMBOL(ipu_check_buffer_busy);

/*!
 * This function clear buffer ready for a logical channel.
 *
 * @param       channel         Input parameter for the logical channel ID.
 *
 * @param       type            Input parameter which buffer to clear.
 *
 * @param       bufNum          Input parameter for which buffer number clear
 * 				ready state.
 *
 */
void ipu_clear_buffer_ready(ipu_channel_t channel, ipu_buffer_t type,
		uint32_t bufNum)
{
	unsigned long lock_flags;
	uint32_t dma_ch = channel_2_dma(channel, type);

	if (!idma_is_valid(dma_ch))
		return;

	spin_lock_irqsave(&ipu_lock, lock_flags);

	__raw_writel(0xF0000000, IPU_GPR); /* write one to clear */
	if (bufNum == 0) {
		if (idma_is_set(IPU_CHA_BUF0_RDY, dma_ch)) {
			__raw_writel(idma_mask(dma_ch),
					IPU_CHA_BUF0_RDY(dma_ch));
		}
	} else {
		if (idma_is_set(IPU_CHA_BUF1_RDY, dma_ch)) {
			__raw_writel(idma_mask(dma_ch),
					IPU_CHA_BUF1_RDY(dma_ch));
		}
	}
	__raw_writel(0x0, IPU_GPR); /* write one to set */
	spin_unlock_irqrestore(&ipu_lock, lock_flags);
}
EXPORT_SYMBOL(ipu_clear_buffer_ready);

static irqreturn_t disable_chan_irq_handler(int irq, void *dev_id)
{
	struct completion *comp = dev_id;

	complete(comp);
	return IRQ_HANDLED;
}

/*!
 * This function disables a logical channel.
 *
 * @param       channel         Input parameter for the logical channel ID.
 *
 * @param       wait_for_stop   Flag to set whether to wait for channel end
 *                              of frame or return immediately.
 *
 * @return      This function returns 0 on success or negative error code on
 *              fail.
 */
int32_t ipu_disable_channel(ipu_channel_t channel, bool wait_for_stop)
{
	uint32_t reg;
	unsigned long lock_flags;
	uint32_t in_dma;
	uint32_t out_dma;
	uint32_t sec_dma = NO_DMA;
	uint32_t thrd_dma = NO_DMA;

	if ((g_channel_enable_mask & (1L << IPU_CHAN_ID(channel))) == 0) {
		dev_err(g_ipu_dev, "Channel already disabled %d\n",
			IPU_CHAN_ID(channel));
		return 0;
	}

	/* Get input and output dma channels */
	out_dma = channel_2_dma(channel, IPU_OUTPUT_BUFFER);
	in_dma = channel_2_dma(channel, IPU_VIDEO_IN_BUFFER);

	if ((idma_is_valid(in_dma) &&
		!idma_is_set(IDMAC_CHA_EN, in_dma))
		&& (idma_is_valid(out_dma) &&
		!idma_is_set(IDMAC_CHA_EN, out_dma)))
		return -EINVAL;

	if (g_sec_chan_en[IPU_CHAN_ID(channel)])
		sec_dma = channel_2_dma(channel, IPU_GRAPH_IN_BUFFER);
	if (g_thrd_chan_en[IPU_CHAN_ID(channel)]) {
		sec_dma = channel_2_dma(channel, IPU_GRAPH_IN_BUFFER);
		thrd_dma = channel_2_dma(channel, IPU_ALPHA_IN_BUFFER);
	}

	if ((channel == MEM_BG_SYNC) || (channel == MEM_FG_SYNC) ||
	    (channel == MEM_DC_SYNC)) {
		if (channel == MEM_FG_SYNC)
			ipu_disp_set_window_pos(channel, 0, 0);

		_ipu_dp_dc_disable(channel, false);

		/*
		 * wait for BG channel EOF then disable FG-IDMAC,
		 * it avoid FG NFB4EOF error.
		 */
		if (channel == MEM_FG_SYNC) {
			int timeout = 50;

			__raw_writel(IPUIRQ_2_MASK(IPU_IRQ_BG_SYNC_EOF),
					IPUIRQ_2_STATREG(IPU_IRQ_BG_SYNC_EOF));
			while ((__raw_readl(IPUIRQ_2_STATREG(IPU_IRQ_BG_SYNC_EOF)) &
					IPUIRQ_2_MASK(IPU_IRQ_BG_SYNC_EOF)) == 0) {
				msleep(10);
				timeout -= 10;
				if (timeout <= 0) {
					dev_err(g_ipu_dev, "warning: wait for bg sync eof timeout\n");
					break;
				}
			}
		}
	} else if (wait_for_stop) {
		while (idma_is_set(IDMAC_CHA_BUSY, in_dma) ||
		       idma_is_set(IDMAC_CHA_BUSY, out_dma) ||
			(g_sec_chan_en[IPU_CHAN_ID(channel)] &&
			idma_is_set(IDMAC_CHA_BUSY, sec_dma)) ||
			(g_thrd_chan_en[IPU_CHAN_ID(channel)] &&
			idma_is_set(IDMAC_CHA_BUSY, thrd_dma))) {
			uint32_t ret, irq = 0xffffffff;
			DECLARE_COMPLETION_ONSTACK(disable_comp);

			if (idma_is_set(IDMAC_CHA_BUSY, out_dma))
				irq = out_dma;
			if (g_sec_chan_en[IPU_CHAN_ID(channel)] &&
				idma_is_set(IDMAC_CHA_BUSY, sec_dma))
				irq = sec_dma;
			if (g_thrd_chan_en[IPU_CHAN_ID(channel)] &&
				idma_is_set(IDMAC_CHA_BUSY, thrd_dma))
				irq = thrd_dma;
			if (idma_is_set(IDMAC_CHA_BUSY, in_dma))
				irq = in_dma;

			if (irq == 0xffffffff) {
				dev_err(g_ipu_dev, "warning: no channel busy, break\n");
				break;
			}
			ret = ipu_request_irq(irq, disable_chan_irq_handler, 0, NULL, &disable_comp);
			if (ret < 0) {
				dev_err(g_ipu_dev, "irq %d in use\n", irq);
				break;
			} else {
				ret = wait_for_completion_timeout(&disable_comp, msecs_to_jiffies(200));
				ipu_free_irq(irq, &disable_comp);
				if (ret == 0) {
					ipu_dump_registers();
					dev_err(g_ipu_dev, "warning: disable ipu dma channel %d during its busy state\n", irq);
					break;
				}
			}
		}
	}

	spin_lock_irqsave(&ipu_lock, lock_flags);

	if ((channel == MEM_BG_SYNC) || (channel == MEM_FG_SYNC) ||
	    (channel == MEM_DC_SYNC)) {
		reg = __raw_readl(IDMAC_WM_EN(in_dma));
		__raw_writel(reg & ~idma_mask(in_dma), IDMAC_WM_EN(in_dma));
	}

	/* Disable IC task */
	if (_ipu_is_ic_chan(in_dma) || _ipu_is_ic_chan(out_dma) ||
		_ipu_is_irt_chan(in_dma) || _ipu_is_irt_chan(out_dma))
		_ipu_ic_disable_task(channel);

	/* Disable DMA channel(s) */
	if (idma_is_valid(in_dma)) {
		reg = __raw_readl(IDMAC_CHA_EN(in_dma));
		__raw_writel(reg & ~idma_mask(in_dma), IDMAC_CHA_EN(in_dma));
		__raw_writel(idma_mask(in_dma), IPU_CHA_CUR_BUF(in_dma));
	}
	if (idma_is_valid(out_dma)) {
		reg = __raw_readl(IDMAC_CHA_EN(out_dma));
		__raw_writel(reg & ~idma_mask(out_dma), IDMAC_CHA_EN(out_dma));
		__raw_writel(idma_mask(out_dma), IPU_CHA_CUR_BUF(out_dma));
	}
	if (g_sec_chan_en[IPU_CHAN_ID(channel)] && idma_is_valid(sec_dma)) {
		reg = __raw_readl(IDMAC_CHA_EN(sec_dma));
		__raw_writel(reg & ~idma_mask(sec_dma), IDMAC_CHA_EN(sec_dma));
		__raw_writel(idma_mask(sec_dma), IPU_CHA_CUR_BUF(sec_dma));
	}
	if (g_thrd_chan_en[IPU_CHAN_ID(channel)] && idma_is_valid(thrd_dma)) {
		reg = __raw_readl(IDMAC_CHA_EN(thrd_dma));
		__raw_writel(reg & ~idma_mask(thrd_dma), IDMAC_CHA_EN(thrd_dma));
		if (channel == MEM_BG_SYNC || channel == MEM_FG_SYNC) {
			reg = __raw_readl(IDMAC_SEP_ALPHA);
			__raw_writel(reg & ~idma_mask(in_dma), IDMAC_SEP_ALPHA);
		} else {
			reg = __raw_readl(IDMAC_SEP_ALPHA);
			__raw_writel(reg & ~idma_mask(sec_dma), IDMAC_SEP_ALPHA);
		}
		__raw_writel(idma_mask(thrd_dma), IPU_CHA_CUR_BUF(thrd_dma));
	}

	g_channel_enable_mask &= ~(1L << IPU_CHAN_ID(channel));

	/* Set channel buffers NOT to be ready */
	if (idma_is_valid(in_dma)) {
		ipu_clear_buffer_ready(channel, IPU_VIDEO_IN_BUFFER, 0);
		ipu_clear_buffer_ready(channel, IPU_VIDEO_IN_BUFFER, 1);
	}
	if (idma_is_valid(out_dma)) {
		ipu_clear_buffer_ready(channel, IPU_OUTPUT_BUFFER, 0);
		ipu_clear_buffer_ready(channel, IPU_OUTPUT_BUFFER, 1);
	}
	if (g_sec_chan_en[IPU_CHAN_ID(channel)] && idma_is_valid(sec_dma)) {
		ipu_clear_buffer_ready(channel, IPU_GRAPH_IN_BUFFER, 0);
		ipu_clear_buffer_ready(channel, IPU_GRAPH_IN_BUFFER, 1);
	}
	if (g_thrd_chan_en[IPU_CHAN_ID(channel)] && idma_is_valid(thrd_dma)) {
		ipu_clear_buffer_ready(channel, IPU_ALPHA_IN_BUFFER, 0);
		ipu_clear_buffer_ready(channel, IPU_ALPHA_IN_BUFFER, 1);
	}

	spin_unlock_irqrestore(&ipu_lock, lock_flags);

	return 0;
}
EXPORT_SYMBOL(ipu_disable_channel);

/*!
 * This function enables CSI.
 *
 * @param       csi	csi num 0 or 1
 *
 * @return      This function returns 0 on success or negative error code on
 *              fail.
 */
int32_t ipu_enable_csi(uint32_t csi)
{
	uint32_t reg;
	unsigned long lock_flags;

	if (csi > 1) {
		dev_err(g_ipu_dev, "Wrong csi num_%d\n", csi);
		return -EINVAL;
	}

	spin_lock_irqsave(&ipu_lock, lock_flags);
	ipu_csi_use_count[csi]++;

	if (ipu_csi_use_count[csi] == 1) {
		reg = __raw_readl(IPU_CONF);
		if (csi == 0)
			__raw_writel(reg | IPU_CONF_CSI0_EN, IPU_CONF);
		else
			__raw_writel(reg | IPU_CONF_CSI1_EN, IPU_CONF);
	}
	spin_unlock_irqrestore(&ipu_lock, lock_flags);
	return 0;
}
EXPORT_SYMBOL(ipu_enable_csi);

/*!
 * This function disables CSI.
 *
 * @param       csi	csi num 0 or 1
 *
 * @return      This function returns 0 on success or negative error code on
 *              fail.
 */
int32_t ipu_disable_csi(uint32_t csi)
{
	uint32_t reg;
	unsigned long lock_flags;

	if (csi > 1) {
		dev_err(g_ipu_dev, "Wrong csi num_%d\n", csi);
		return -EINVAL;
	}

	spin_lock_irqsave(&ipu_lock, lock_flags);
	ipu_csi_use_count[csi]--;

	if (ipu_csi_use_count[csi] == 0) {
		reg = __raw_readl(IPU_CONF);
		if (csi == 0)
			__raw_writel(reg & ~IPU_CONF_CSI0_EN, IPU_CONF);
		else
			__raw_writel(reg & ~IPU_CONF_CSI1_EN, IPU_CONF);
	}
	spin_unlock_irqrestore(&ipu_lock, lock_flags);
	return 0;
}
EXPORT_SYMBOL(ipu_disable_csi);

static irqreturn_t ipu_irq_handler(int irq, void *desc)
{
	int i;
	uint32_t line;
	irqreturn_t result = IRQ_NONE;
	uint32_t int_stat;
	const int err_reg[] = { 5, 6, 9, 10, 0 };
	const int int_reg[] = { 1, 2, 3, 4, 11, 12, 13, 14, 15, 0 };

	for (i = 0;; i++) {
		if (err_reg[i] == 0)
			break;
		int_stat = __raw_readl(IPU_INT_STAT(err_reg[i]));
		int_stat &= __raw_readl(IPU_INT_CTRL(err_reg[i]));
		if (int_stat) {
			__raw_writel(int_stat, IPU_INT_STAT(err_reg[i]));
			dev_err(g_ipu_dev,
				"IPU Error - IPU_INT_STAT_%d = 0x%08X\n",
				err_reg[i], int_stat);
			/* Disable interrupts so we only get error once */
			int_stat =
			    __raw_readl(IPU_INT_CTRL(err_reg[i])) & ~int_stat;
			__raw_writel(int_stat, IPU_INT_CTRL(err_reg[i]));
		}
	}

	for (i = 0;; i++) {
		if (int_reg[i] == 0)
			break;
		int_stat = __raw_readl(IPU_INT_STAT(int_reg[i]));
		int_stat &= __raw_readl(IPU_INT_CTRL(int_reg[i]));
		__raw_writel(int_stat, IPU_INT_STAT(int_reg[i]));
		while ((line = ffs(int_stat)) != 0) {
			line--;
			int_stat &= ~(1UL << line);
			line += (int_reg[i] - 1) * 32;
			result |=
			    ipu_irq_list[line].handler(line,
						       ipu_irq_list[line].
						       dev_id);
		}
	}

	return result;
}

/*!
 * This function enables the interrupt for the specified interrupt line.
 * The interrupt lines are defined in \b ipu_irq_line enum.
 *
 * @param       irq             Interrupt line to enable interrupt for.
 *
 */
void ipu_enable_irq(uint32_t irq)
{
	uint32_t reg;
	unsigned long lock_flags;

	if (!g_ipu_clk_enabled)
		clk_enable(g_ipu_clk);

	spin_lock_irqsave(&ipu_lock, lock_flags);

	reg = __raw_readl(IPUIRQ_2_CTRLREG(irq));
	reg |= IPUIRQ_2_MASK(irq);
	__raw_writel(reg, IPUIRQ_2_CTRLREG(irq));

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
	if (!g_ipu_clk_enabled)
		clk_disable(g_ipu_clk);
}
EXPORT_SYMBOL(ipu_enable_irq);

/*!
 * This function disables the interrupt for the specified interrupt line.
 * The interrupt lines are defined in \b ipu_irq_line enum.
 *
 * @param       irq             Interrupt line to disable interrupt for.
 *
 */
void ipu_disable_irq(uint32_t irq)
{
	uint32_t reg;
	unsigned long lock_flags;

	if (!g_ipu_clk_enabled)
		clk_enable(g_ipu_clk);
	spin_lock_irqsave(&ipu_lock, lock_flags);

	reg = __raw_readl(IPUIRQ_2_CTRLREG(irq));
	reg &= ~IPUIRQ_2_MASK(irq);
	__raw_writel(reg, IPUIRQ_2_CTRLREG(irq));

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
	if (!g_ipu_clk_enabled)
		clk_disable(g_ipu_clk);
}
EXPORT_SYMBOL(ipu_disable_irq);

/*!
 * This function clears the interrupt for the specified interrupt line.
 * The interrupt lines are defined in \b ipu_irq_line enum.
 *
 * @param       irq             Interrupt line to clear interrupt for.
 *
 */
void ipu_clear_irq(uint32_t irq)
{
	if (!g_ipu_clk_enabled)
		clk_enable(g_ipu_clk);

	__raw_writel(IPUIRQ_2_MASK(irq), IPUIRQ_2_STATREG(irq));

	if (!g_ipu_clk_enabled)
		clk_disable(g_ipu_clk);
}
EXPORT_SYMBOL(ipu_clear_irq);

/*!
 * This function returns the current interrupt status for the specified
 * interrupt line. The interrupt lines are defined in \b ipu_irq_line enum.
 *
 * @param       irq             Interrupt line to get status for.
 *
 * @return      Returns true if the interrupt is pending/asserted or false if
 *              the interrupt is not pending.
 */
bool ipu_get_irq_status(uint32_t irq)
{
	uint32_t reg;

	if (!g_ipu_clk_enabled)
		clk_enable(g_ipu_clk);

	reg = __raw_readl(IPUIRQ_2_STATREG(irq));

	if (!g_ipu_clk_enabled)
		clk_disable(g_ipu_clk);

	if (reg & IPUIRQ_2_MASK(irq))
		return true;
	else
		return false;
}
EXPORT_SYMBOL(ipu_get_irq_status);

/*!
 * This function registers an interrupt handler function for the specified
 * interrupt line. The interrupt lines are defined in \b ipu_irq_line enum.
 *
 * @param       irq             Interrupt line to get status for.
 *
 * @param       handler         Input parameter for address of the handler
 *                              function.
 *
 * @param       irq_flags       Flags for interrupt mode. Currently not used.
 *
 * @param       devname         Input parameter for string name of driver
 *                              registering the handler.
 *
 * @param       dev_id          Input parameter for pointer of data to be
 *                              passed to the handler.
 *
 * @return      This function returns 0 on success or negative error code on
 *              fail.
 */
int ipu_request_irq(uint32_t irq,
		    irqreturn_t(*handler) (int, void *),
		    uint32_t irq_flags, const char *devname, void *dev_id)
{
	unsigned long lock_flags;

	BUG_ON(irq >= IPU_IRQ_COUNT);

	spin_lock_irqsave(&ipu_lock, lock_flags);

	if (ipu_irq_list[irq].handler != NULL) {
		dev_err(g_ipu_dev,
			"handler already installed on irq %d\n", irq);
		spin_unlock_irqrestore(&ipu_lock, lock_flags);
		return -EINVAL;
	}

	ipu_irq_list[irq].handler = handler;
	ipu_irq_list[irq].flags = irq_flags;
	ipu_irq_list[irq].dev_id = dev_id;
	ipu_irq_list[irq].name = devname;

	spin_unlock_irqrestore(&ipu_lock, lock_flags);

	ipu_enable_irq(irq);	/* enable the interrupt */

	return 0;
}
EXPORT_SYMBOL(ipu_request_irq);

/*!
 * This function unregisters an interrupt handler for the specified interrupt
 * line. The interrupt lines are defined in \b ipu_irq_line enum.
 *
 * @param       irq             Interrupt line to get status for.
 *
 * @param       dev_id          Input parameter for pointer of data to be passed
 *                              to the handler. This must match value passed to
 *                              ipu_request_irq().
 *
 */
void ipu_free_irq(uint32_t irq, void *dev_id)
{
	ipu_disable_irq(irq);	/* disable the interrupt */

	if (ipu_irq_list[irq].dev_id == dev_id)
		ipu_irq_list[irq].handler = NULL;
}
EXPORT_SYMBOL(ipu_free_irq);

uint32_t ipu_get_cur_buffer_idx(ipu_channel_t channel, ipu_buffer_t type)
{
	uint32_t reg, dma_chan;

	dma_chan = channel_2_dma(channel, type);
	if (!idma_is_valid(dma_chan))
		return -EINVAL;

	reg = __raw_readl(IPU_CHA_CUR_BUF(dma_chan/32));
	if (reg & idma_mask(dma_chan))
		return 1;
	else
		return 0;
}
EXPORT_SYMBOL(ipu_get_cur_buffer_idx);

uint32_t _ipu_channel_status(ipu_channel_t channel)
{
	uint32_t stat = 0;
	uint32_t task_stat_reg = __raw_readl(IPU_PROC_TASK_STAT);

	switch (channel) {
	case MEM_PRP_VF_MEM:
		stat = (task_stat_reg & TSTAT_VF_MASK) >> TSTAT_VF_OFFSET;
		break;
	case MEM_VDI_PRP_VF_MEM:
		stat = (task_stat_reg & TSTAT_VF_MASK) >> TSTAT_VF_OFFSET;
		break;
	case MEM_ROT_VF_MEM:
		stat =
		    (task_stat_reg & TSTAT_VF_ROT_MASK) >> TSTAT_VF_ROT_OFFSET;
		break;
	case MEM_PRP_ENC_MEM:
		stat = (task_stat_reg & TSTAT_ENC_MASK) >> TSTAT_ENC_OFFSET;
		break;
	case MEM_ROT_ENC_MEM:
		stat =
		    (task_stat_reg & TSTAT_ENC_ROT_MASK) >>
		    TSTAT_ENC_ROT_OFFSET;
		break;
	case MEM_PP_MEM:
		stat = (task_stat_reg & TSTAT_PP_MASK) >> TSTAT_PP_OFFSET;
		break;
	case MEM_ROT_PP_MEM:
		stat =
		    (task_stat_reg & TSTAT_PP_ROT_MASK) >> TSTAT_PP_ROT_OFFSET;
		break;

	default:
		stat = TASK_STAT_IDLE;
		break;
	}
	return stat;
}

int32_t ipu_swap_channel(ipu_channel_t from_ch, ipu_channel_t to_ch)
{
	uint32_t reg;
	unsigned long lock_flags;

	int from_dma = channel_2_dma(from_ch, IPU_INPUT_BUFFER);
	int to_dma = channel_2_dma(to_ch, IPU_INPUT_BUFFER);

	/* enable target channel */
	spin_lock_irqsave(&ipu_lock, lock_flags);

	reg = __raw_readl(IDMAC_CHA_EN(to_dma));
	__raw_writel(reg | idma_mask(to_dma), IDMAC_CHA_EN(to_dma));

	g_channel_enable_mask |= 1L << IPU_CHAN_ID(to_ch);

	spin_unlock_irqrestore(&ipu_lock, lock_flags);

	/* switch dp dc */
	_ipu_dp_dc_disable(from_ch, true);

	/* disable source channel */
	spin_lock_irqsave(&ipu_lock, lock_flags);

	reg = __raw_readl(IDMAC_CHA_EN(from_dma));
	__raw_writel(reg & ~idma_mask(from_dma), IDMAC_CHA_EN(from_dma));
	__raw_writel(idma_mask(from_dma), IPU_CHA_CUR_BUF(from_dma));

	g_channel_enable_mask &= ~(1L << IPU_CHAN_ID(from_ch));

	spin_unlock_irqrestore(&ipu_lock, lock_flags);

	return 0;
}
EXPORT_SYMBOL(ipu_swap_channel);

uint32_t bytes_per_pixel(uint32_t fmt)
{
	switch (fmt) {
	case IPU_PIX_FMT_GENERIC:	/*generic data */
	case IPU_PIX_FMT_RGB332:
	case IPU_PIX_FMT_YUV420P:
	case IPU_PIX_FMT_YUV422P:
		return 1;
		break;
	case IPU_PIX_FMT_RGB565:
	case IPU_PIX_FMT_YUYV:
	case IPU_PIX_FMT_UYVY:
		return 2;
		break;
	case IPU_PIX_FMT_BGR24:
	case IPU_PIX_FMT_RGB24:
		return 3;
		break;
	case IPU_PIX_FMT_GENERIC_32:	/*generic data */
	case IPU_PIX_FMT_BGR32:
	case IPU_PIX_FMT_BGRA32:
	case IPU_PIX_FMT_RGB32:
	case IPU_PIX_FMT_RGBA32:
	case IPU_PIX_FMT_ABGR32:
		return 4;
		break;
	default:
		return 1;
		break;
	}
	return 0;
}
EXPORT_SYMBOL(bytes_per_pixel);

ipu_color_space_t format_to_colorspace(uint32_t fmt)
{
	switch (fmt) {
	case IPU_PIX_FMT_RGB666:
	case IPU_PIX_FMT_RGB565:
	case IPU_PIX_FMT_BGR24:
	case IPU_PIX_FMT_RGB24:
	case IPU_PIX_FMT_GBR24:
	case IPU_PIX_FMT_BGR32:
	case IPU_PIX_FMT_BGRA32:
	case IPU_PIX_FMT_RGB32:
	case IPU_PIX_FMT_RGBA32:
	case IPU_PIX_FMT_ABGR32:
	case IPU_PIX_FMT_LVDS666:
	case IPU_PIX_FMT_LVDS888:
		return RGB;
		break;

	default:
		return YCbCr;
		break;
	}
	return RGB;
}

bool ipu_pixel_format_has_alpha(uint32_t fmt)
{
	switch (fmt) {
	case IPU_PIX_FMT_RGBA32:
	case IPU_PIX_FMT_BGRA32:
	case IPU_PIX_FMT_ABGR32:
		return true;
		break;
	default:
		return false;
		break;
	}
	return false;
}

void ipu_set_csc_coefficients(ipu_channel_t channel, int32_t param[][3])
{
	_ipu_dp_set_csc_coefficients(channel, param);
}
EXPORT_SYMBOL(ipu_set_csc_coefficients);

static int ipu_suspend(struct platform_device *pdev, pm_message_t state)
{
	if (g_ipu_clk_enabled) {
		/* save and disable enabled channels*/
		idma_enable_reg[0] = __raw_readl(IDMAC_CHA_EN(0));
		idma_enable_reg[1] = __raw_readl(IDMAC_CHA_EN(32));
		while ((__raw_readl(IDMAC_CHA_BUSY(0)) & idma_enable_reg[0])
			|| (__raw_readl(IDMAC_CHA_BUSY(32)) &
				idma_enable_reg[1])) {
			/* disable channel not busy already */
			uint32_t chan_should_disable, timeout = 1000, time = 0;

			chan_should_disable =
				__raw_readl(IDMAC_CHA_BUSY(0))
					^ idma_enable_reg[0];
			__raw_writel((~chan_should_disable) &
					idma_enable_reg[0], IDMAC_CHA_EN(0));
			chan_should_disable =
				__raw_readl(IDMAC_CHA_BUSY(1))
					^ idma_enable_reg[1];
			__raw_writel((~chan_should_disable) &
					idma_enable_reg[1], IDMAC_CHA_EN(32));
			msleep(2);
			time += 2;
			if (time >= timeout)
				return -1;
		}
		__raw_writel(0, IDMAC_CHA_EN(0));
		__raw_writel(0, IDMAC_CHA_EN(32));

		/* save double buffer select regs */
		ipu_cha_db_mode_reg[0] = __raw_readl(IPU_CHA_DB_MODE_SEL(0));
		ipu_cha_db_mode_reg[1] = __raw_readl(IPU_CHA_DB_MODE_SEL(32));
		ipu_cha_db_mode_reg[2] =
			__raw_readl(IPU_ALT_CHA_DB_MODE_SEL(0));
		ipu_cha_db_mode_reg[3] =
			__raw_readl(IPU_ALT_CHA_DB_MODE_SEL(32));

		/* save current buffer regs */
		ipu_cha_cur_buf_reg[0] = __raw_readl(IPU_CHA_CUR_BUF(0));
		ipu_cha_cur_buf_reg[1] = __raw_readl(IPU_CHA_CUR_BUF(32));
		ipu_cha_cur_buf_reg[2] = __raw_readl(IPU_ALT_CUR_BUF0);
		ipu_cha_cur_buf_reg[3] = __raw_readl(IPU_ALT_CUR_BUF1);

		/* save sub-modules status and disable all */
		ic_conf_reg = __raw_readl(IC_CONF);
		__raw_writel(0, IC_CONF);
		ipu_conf_reg = __raw_readl(IPU_CONF);
		__raw_writel(0, IPU_CONF);

		/* save buf ready regs */
		buf_ready_reg[0] = __raw_readl(IPU_CHA_BUF0_RDY(0));
		buf_ready_reg[1] = __raw_readl(IPU_CHA_BUF0_RDY(32));
		buf_ready_reg[2] = __raw_readl(IPU_CHA_BUF1_RDY(0));
		buf_ready_reg[3] = __raw_readl(IPU_CHA_BUF1_RDY(32));
		buf_ready_reg[4] = __raw_readl(IPU_ALT_CHA_BUF0_RDY(0));
		buf_ready_reg[5] = __raw_readl(IPU_ALT_CHA_BUF0_RDY(32));
		buf_ready_reg[6] = __raw_readl(IPU_ALT_CHA_BUF1_RDY(0));
		buf_ready_reg[7] = __raw_readl(IPU_ALT_CHA_BUF1_RDY(32));
	}

	mxc_pg_enable(pdev);

	return 0;
}

static int ipu_resume(struct platform_device *pdev)
{
	mxc_pg_disable(pdev);

	if (g_ipu_clk_enabled) {

		/* restore buf ready regs */
		__raw_writel(buf_ready_reg[0], IPU_CHA_BUF0_RDY(0));
		__raw_writel(buf_ready_reg[1], IPU_CHA_BUF0_RDY(32));
		__raw_writel(buf_ready_reg[2], IPU_CHA_BUF1_RDY(0));
		__raw_writel(buf_ready_reg[3], IPU_CHA_BUF1_RDY(32));
		__raw_writel(buf_ready_reg[4], IPU_ALT_CHA_BUF0_RDY(0));
		__raw_writel(buf_ready_reg[5], IPU_ALT_CHA_BUF0_RDY(32));
		__raw_writel(buf_ready_reg[6], IPU_ALT_CHA_BUF1_RDY(0));
		__raw_writel(buf_ready_reg[7], IPU_ALT_CHA_BUF1_RDY(32));

		/* re-enable sub-modules*/
		__raw_writel(ipu_conf_reg, IPU_CONF);
		__raw_writel(ic_conf_reg, IC_CONF);

		/* restore double buffer select regs */
		__raw_writel(ipu_cha_db_mode_reg[0], IPU_CHA_DB_MODE_SEL(0));
		__raw_writel(ipu_cha_db_mode_reg[1], IPU_CHA_DB_MODE_SEL(32));
		__raw_writel(ipu_cha_db_mode_reg[2],
				IPU_ALT_CHA_DB_MODE_SEL(0));
		__raw_writel(ipu_cha_db_mode_reg[3],
				IPU_ALT_CHA_DB_MODE_SEL(32));

		/* restore current buffer select regs */
		__raw_writel(~(ipu_cha_cur_buf_reg[0]), IPU_CHA_CUR_BUF(0));
		__raw_writel(~(ipu_cha_cur_buf_reg[1]), IPU_CHA_CUR_BUF(32));
		__raw_writel(~(ipu_cha_cur_buf_reg[2]), IPU_ALT_CUR_BUF0);
		__raw_writel(~(ipu_cha_cur_buf_reg[3]), IPU_ALT_CUR_BUF1);

		/* restart idma channel*/
		__raw_writel(idma_enable_reg[0], IDMAC_CHA_EN(0));
		__raw_writel(idma_enable_reg[1], IDMAC_CHA_EN(32));
	} else {
		clk_enable(g_ipu_clk);
		_ipu_dmfc_init(dmfc_type_setup, 1);
		_ipu_init_dc_mappings();

		/* Set sync refresh channels as high priority */
		__raw_writel(0x18800000L, IDMAC_CHA_PRI(0));
		clk_disable(g_ipu_clk);
	}

	return 0;
}

/*!
 * This structure contains pointers to the power management callback functions.
 */
static struct platform_driver mxcipu_driver = {
	.driver = {
		   .name = "mxc_ipu",
		   },
	.probe = ipu_probe,
	.remove = ipu_remove,
	.suspend = ipu_suspend,
	.resume = ipu_resume,
};

int32_t __init ipu_gen_init(void)
{
	int32_t ret;

	ret = platform_driver_register(&mxcipu_driver);
	return 0;
}

subsys_initcall(ipu_gen_init);

static void __exit ipu_gen_uninit(void)
{
	platform_driver_unregister(&mxcipu_driver);
}

module_exit(ipu_gen_uninit);
