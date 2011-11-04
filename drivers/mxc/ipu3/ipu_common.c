/*
 * Copyright 2005-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/clk.h>
#include <mach/clock.h>
#include <mach/hardware.h>
#include <mach/ipu-v3.h>
#include <mach/devices-common.h>
#include <asm/cacheflush.h>
#include <linux/delay.h>

#include "ipu_prv.h"
#include "ipu_regs.h"
#include "ipu_param_mem.h"

static struct ipu_soc ipu_array[MXC_IPU_MAX_NUM];
int g_ipu_hw_rev;

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

static inline int _ipu_is_trb_chan(uint32_t dma_chan)
{
	return (((dma_chan == 8) || (dma_chan == 9) ||
		 (dma_chan == 10) || (dma_chan == 13) ||
		 (dma_chan == 21) || (dma_chan == 23) ||
		 (dma_chan == 27) || (dma_chan == 28)) &&
		(g_ipu_hw_rev >= 2));
}

#define idma_is_valid(ch)	(ch != NO_DMA)
#define idma_mask(ch)		(idma_is_valid(ch) ? (1UL << (ch & 0x1F)) : 0)
#define idma_is_set(ipu, reg, dma)	(ipu_idmac_read(ipu, reg(dma)) & idma_mask(dma))
#define tri_cur_buf_mask(ch)	(idma_mask(ch*2) * 3)
#define tri_cur_buf_shift(ch)	(ffs(idma_mask(ch*2)) - 1)

static int ipu_reset(struct ipu_soc *ipu)
{
	int timeout = 1000;

	ipu_cm_write(ipu, 0x807FFFFF, IPU_MEM_RST);

	while (ipu_cm_read(ipu, IPU_MEM_RST) & 0x80000000) {
		if (!timeout--)
			return -ETIME;
		msleep(1);
	}

	return 0;
}

static int __devinit ipu_clk_setup_enable(struct ipu_soc *ipu,
		struct platform_device *pdev)
{
	struct imx_ipuv3_platform_data *plat_data = pdev->dev.platform_data;
	char ipu_clk[] = "ipu1_clk";
	char di0_clk[] = "ipu1_di0_clk";
	char di1_clk[] = "ipu1_di1_clk";

	ipu_clk[3] += pdev->id;
	di0_clk[3] += pdev->id;
	di1_clk[3] += pdev->id;

	ipu->ipu_clk = clk_get(ipu->dev, ipu_clk);
	if (IS_ERR(ipu->ipu_clk)) {
		dev_err(ipu->dev, "clk_get failed");
		return PTR_ERR(ipu->ipu_clk);
	}
	dev_dbg(ipu->dev, "ipu_clk = %lu\n", clk_get_rate(ipu->ipu_clk));

	ipu->pixel_clk[0] = ipu_pixel_clk[0];
	ipu->pixel_clk[1] = ipu_pixel_clk[1];

	ipu_lookups[pdev->id][0].clk = &ipu->pixel_clk[0];
	ipu_lookups[pdev->id][1].clk = &ipu->pixel_clk[1];
	ipu_lookups[pdev->id][0].dev_id = dev_name(ipu->dev);
	ipu_lookups[pdev->id][1].dev_id = dev_name(ipu->dev);
	clkdev_add(&ipu_lookups[pdev->id][0]);
	clkdev_add(&ipu_lookups[pdev->id][1]);

	clk_debug_register(&ipu->pixel_clk[0]);
	clk_debug_register(&ipu->pixel_clk[1]);

	clk_enable(ipu->ipu_clk);

	clk_set_parent(&ipu->pixel_clk[0], ipu->ipu_clk);
	clk_set_parent(&ipu->pixel_clk[1], ipu->ipu_clk);

	ipu->di_clk[0] = clk_get(ipu->dev, di0_clk);
	ipu->di_clk[1] = clk_get(ipu->dev, di1_clk);

	ipu->csi_clk[0] = clk_get(ipu->dev, plat_data->csi_clk[0]);
	ipu->csi_clk[1] = clk_get(ipu->dev, plat_data->csi_clk[1]);

	return 0;
}

#if 0
static void ipu_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	struct ipu_soc *ipu = irq_desc_get_handler_data(desc);
	const int int_reg[] = { 1, 2, 3, 4, 11, 12, 13, 14, 15, 0 };
	u32 status;
	int i, line;

	for (i = 0;; i++) {
		if (int_reg[i] == 0)
			break;

		status = ipu_cm_read(ipu, IPU_INT_STAT(int_reg[i]));
		status &= ipu_cm_read(ipu, IPU_INT_CTRL(int_reg[i]));

		while ((line = ffs(status))) {
			line--;
			status &= ~(1UL << line);
			line += ipu->irq_start + (int_reg[i] - 1) * 32;
			generic_handle_irq(line);
		}

	}
}

static void ipu_err_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	struct ipu_soc *ipu = irq_desc_get_handler_data(desc);
	const int int_reg[] = { 5, 6, 9, 10, 0 };
	u32 status;
	int i, line;

	for (i = 0;; i++) {
		if (int_reg[i] == 0)
			break;

		status = ipu_cm_read(ipu, IPU_INT_STAT(int_reg[i]));
		status &= ipu_cm_read(ipu, IPU_INT_CTRL(int_reg[i]));

		while ((line = ffs(status))) {
			line--;
			status &= ~(1UL << line);
			line += ipu->irq_start + (int_reg[i] - 1) * 32;
			generic_handle_irq(line);
		}

	}
}

static void ipu_ack_irq(struct irq_data *d)
{
	struct ipu_soc *ipu = irq_data_get_irq_chip_data(d);
	unsigned int irq = d->irq - ipu->irq_start;
	unsigned long flags;

	spin_lock_irqsave(&ipu->ipu_lock, flags);
	ipu_cm_write(ipu, 1 << (irq % 32), IPU_INT_STAT(irq / 32 + 1));
	spin_unlock_irqrestore(&ipu->ipu_lock, flags);
}

static void ipu_unmask_irq(struct irq_data *d)
{
	struct ipu_soc *ipu = irq_data_get_irq_chip_data(d);
	unsigned int irq = d->irq - ipu->irq_start;
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&ipu->ipu_lock, flags);
	reg = ipu_cm_read(ipu, IPU_INT_CTRL(irq / 32 + 1));
	reg |= 1 << (irq % 32);
	ipu_cm_write(ipu, reg, IPU_INT_CTRL(irq / 32 + 1));
	spin_unlock_irqrestore(&ipu->ipu_lock, flags);
}

static void ipu_mask_irq(struct irq_data *d)
{
	struct ipu_soc *ipu = irq_data_get_irq_chip_data(d);
	unsigned int irq = d->irq - ipu->irq_start;
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&ipu->ipu_lock, flags);
	reg = ipu_cm_read(ipu, IPU_INT_CTRL(irq / 32 + 1));
	reg &= ~(1 << (irq % 32));
	ipu_cm_write(ipu, reg, IPU_INT_CTRL(irq / 32 + 1));
	spin_unlock_irqrestore(&ipu->ipu_lock, flags);
}

static struct irq_chip ipu_irq_chip = {
	.name = "IPU",
	.irq_ack = ipu_ack_irq,
	.irq_mask = ipu_mask_irq,
	.irq_unmask = ipu_unmask_irq,
};

static void __devinit ipu_irq_setup(struct ipu_soc *ipu)
{
	int i;

	for (i = ipu->irq_start; i < ipu->irq_start + MX5_IPU_IRQS; i++) {
		irq_set_chip_and_handler(i, &ipu_irq_chip, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
		irq_set_chip_data(i, ipu);
	}

	irq_set_chained_handler(ipu->irq_sync, ipu_irq_handler);
	irq_set_handler_data(ipu->irq_sync, ipu);
	irq_set_chained_handler(ipu->irq_err, ipu_err_irq_handler);
	irq_set_handler_data(ipu->irq_err, ipu);
}

int ipu_request_irq(struct ipu_soc *ipu, unsigned int irq,
		irq_handler_t handler, unsigned long flags,
		const char *name, void *dev)
{
	return request_irq(ipu->irq_start + irq, handler, flags, name, dev);
}
EXPORT_SYMBOL_GPL(ipu_request_irq);

void ipu_enable_irq(struct ipu_soc *ipu, unsigned int irq)
{
	return enable_irq(ipu->irq_start + irq);
}
EXPORT_SYMBOL_GPL(ipu_disable_irq);

void ipu_disable_irq(struct ipu_soc *ipu, unsigned int irq)
{
	return disable_irq(ipu->irq_start + irq);
}
EXPORT_SYMBOL_GPL(ipu_disable_irq);

void ipu_free_irq(struct ipu_soc *ipu, unsigned int irq, void *dev_id)
{
	free_irq(ipu->irq_start + irq, dev_id);
}
EXPORT_SYMBOL_GPL(ipu_free_irq);

static irqreturn_t ipu_completion_handler(int irq, void *dev)
{
	struct completion *completion = dev;

	complete(completion);
	return IRQ_HANDLED;
}

int ipu_wait_for_interrupt(struct ipu_soc *ipu, int interrupt, int timeout_ms)
{
	DECLARE_COMPLETION_ONSTACK(completion);
	int ret;

	ret = ipu_request_irq(ipu, interrupt, ipu_completion_handler,
			0, NULL, &completion);
	if (ret) {
		dev_err(ipu->dev,
			"ipu request irq %d fail\n", interrupt);
		return ret;
	}

	ret = wait_for_completion_timeout(&completion,
			msecs_to_jiffies(timeout_ms));

	ipu_free_irq(ipu, interrupt, &completion);

	return ret > 0 ? 0 : -ETIMEDOUT;
}
EXPORT_SYMBOL_GPL(ipu_wait_for_interrupt);
#endif

struct ipu_soc *ipu_get_soc(int id)
{
	if (id >= MXC_IPU_MAX_NUM)
		return ERR_PTR(-ENODEV);
	else if (!ipu_array[id].online)
		return ERR_PTR(-ENODEV);
	else
		return &(ipu_array[id]);
}
EXPORT_SYMBOL_GPL(ipu_get_soc);

void _ipu_lock(struct ipu_soc *ipu)
{
	/*TODO:remove in_irq() condition after v4l2 driver rewrite*/
	if (!in_irq() && !in_softirq())
		mutex_lock(&ipu->mutex_lock);
}

void _ipu_unlock(struct ipu_soc *ipu)
{
	/*TODO:remove in_irq() condition after v4l2 driver rewrite*/
	if (!in_irq() && !in_softirq())
		mutex_unlock(&ipu->mutex_lock);
}

void _ipu_get(struct ipu_soc *ipu)
{
	if (atomic_inc_return(&ipu->ipu_use_count) == 1)
		clk_enable(ipu->ipu_clk);
}

void _ipu_put(struct ipu_soc *ipu)
{
	if (atomic_dec_return(&ipu->ipu_use_count) == 0)
		clk_disable(ipu->ipu_clk);
}

/*!
 * This function is called by the driver framework to initialize the IPU
 * hardware.
 *
 * @param	dev	The device structure for the IPU passed in by the
 *			driver framework.
 *
 * @return      Returns 0 on success or negative error code on error
 */
static int __devinit ipu_probe(struct platform_device *pdev)
{
	struct imx_ipuv3_platform_data *plat_data = pdev->dev.platform_data;
	struct ipu_soc *ipu;
	struct resource *res;
	unsigned long ipu_base;
	int ret = 0;

	if (pdev->id >= MXC_IPU_MAX_NUM)
		return -ENODEV;

	ipu = &ipu_array[pdev->id];
	memset(ipu, 0, sizeof(struct ipu_soc));

	spin_lock_init(&ipu->spin_lock);
	mutex_init(&ipu->mutex_lock);
	atomic_set(&ipu->ipu_use_count, 0);

	g_ipu_hw_rev = plat_data->rev;

	ipu->dev = &pdev->dev;

	if (plat_data->init)
		plat_data->init(pdev->id);

	ipu->irq_sync = platform_get_irq(pdev, 0);
	ipu->irq_err = platform_get_irq(pdev, 1);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!res || ipu->irq_sync < 0 || ipu->irq_err < 0) {
		ret = -ENODEV;
		goto failed_get_res;
	}

	if (request_irq(ipu->irq_sync, ipu_irq_handler, 0, pdev->name, ipu) != 0) {
		dev_err(ipu->dev, "request SYNC interrupt failed\n");
		ret = -EBUSY;
		goto failed_req_irq_sync;
	}
	/* Some platforms have 2 IPU interrupts */
	if (ipu->irq_err >= 0) {
		if (request_irq
		    (ipu->irq_err, ipu_irq_handler, 0, pdev->name, ipu) != 0) {
			dev_err(ipu->dev, "request ERR interrupt failed\n");
			ret = -EBUSY;
			goto failed_req_irq_err;
		}
	}

	ipu_base = res->start;
	/* base fixup */
	if (g_ipu_hw_rev == 4)	/* IPUv3H */
		ipu_base += IPUV3H_REG_BASE;
	else if (g_ipu_hw_rev == 3)	/* IPUv3M */
		ipu_base += IPUV3M_REG_BASE;
	else			/* IPUv3D, v3E, v3EX */
		ipu_base += IPUV3DEX_REG_BASE;

	ipu->cm_reg = ioremap(ipu_base + IPU_CM_REG_BASE, PAGE_SIZE);
	ipu->ic_reg = ioremap(ipu_base + IPU_IC_REG_BASE, PAGE_SIZE);
	ipu->idmac_reg = ioremap(ipu_base + IPU_IDMAC_REG_BASE, PAGE_SIZE);
	/* DP Registers are accessed thru the SRM */
	ipu->dp_reg = ioremap(ipu_base + IPU_SRM_REG_BASE, PAGE_SIZE);
	ipu->dc_reg = ioremap(ipu_base + IPU_DC_REG_BASE, PAGE_SIZE);
	ipu->dmfc_reg = ioremap(ipu_base + IPU_DMFC_REG_BASE, PAGE_SIZE);
	ipu->di_reg[0] = ioremap(ipu_base + IPU_DI0_REG_BASE, PAGE_SIZE);
	ipu->di_reg[1] = ioremap(ipu_base + IPU_DI1_REG_BASE, PAGE_SIZE);
	ipu->smfc_reg = ioremap(ipu_base + IPU_SMFC_REG_BASE, PAGE_SIZE);
	ipu->csi_reg[0] = ioremap(ipu_base + IPU_CSI0_REG_BASE, PAGE_SIZE);
	ipu->csi_reg[1] = ioremap(ipu_base + IPU_CSI1_REG_BASE, PAGE_SIZE);
	ipu->cpmem_base = ioremap(ipu_base + IPU_CPMEM_REG_BASE, SZ_128K);
	ipu->tpmem_base = ioremap(ipu_base + IPU_TPM_REG_BASE, SZ_64K);
	ipu->dc_tmpl_reg = ioremap(ipu_base + IPU_DC_TMPL_REG_BASE, SZ_128K);
	ipu->vdi_reg = ioremap(ipu_base + IPU_VDI_REG_BASE, PAGE_SIZE);
	ipu->disp_base[1] = ioremap(ipu_base + IPU_DISP1_BASE, SZ_4K);

	if (!ipu->cm_reg || !ipu->ic_reg || !ipu->idmac_reg ||
		!ipu->dp_reg || !ipu->dc_reg || !ipu->dmfc_reg ||
		!ipu->di_reg[0] || !ipu->di_reg[1] || !ipu->smfc_reg ||
		!ipu->csi_reg[0] || !ipu->csi_reg[1] || !ipu->cpmem_base ||
		!ipu->tpmem_base || !ipu->dc_tmpl_reg || !ipu->disp_base[1]
		|| !ipu->vdi_reg) {
		ret = -ENOMEM;
		goto failed_ioremap;
	}

	dev_dbg(ipu->dev, "IPU CM Regs = %p\n", ipu->cm_reg);
	dev_dbg(ipu->dev, "IPU IC Regs = %p\n", ipu->ic_reg);
	dev_dbg(ipu->dev, "IPU IDMAC Regs = %p\n", ipu->idmac_reg);
	dev_dbg(ipu->dev, "IPU DP Regs = %p\n", ipu->dp_reg);
	dev_dbg(ipu->dev, "IPU DC Regs = %p\n", ipu->dc_reg);
	dev_dbg(ipu->dev, "IPU DMFC Regs = %p\n", ipu->dmfc_reg);
	dev_dbg(ipu->dev, "IPU DI0 Regs = %p\n", ipu->di_reg[0]);
	dev_dbg(ipu->dev, "IPU DI1 Regs = %p\n", ipu->di_reg[1]);
	dev_dbg(ipu->dev, "IPU SMFC Regs = %p\n", ipu->smfc_reg);
	dev_dbg(ipu->dev, "IPU CSI0 Regs = %p\n", ipu->csi_reg[0]);
	dev_dbg(ipu->dev, "IPU CSI1 Regs = %p\n", ipu->csi_reg[1]);
	dev_dbg(ipu->dev, "IPU CPMem = %p\n", ipu->cpmem_base);
	dev_dbg(ipu->dev, "IPU TPMem = %p\n", ipu->tpmem_base);
	dev_dbg(ipu->dev, "IPU DC Template Mem = %p\n", ipu->dc_tmpl_reg);
	dev_dbg(ipu->dev, "IPU Display Region 1 Mem = %p\n", ipu->disp_base[1]);
	dev_dbg(ipu->dev, "IPU VDI Regs = %p\n", ipu->vdi_reg);

	ret = ipu_clk_setup_enable(ipu, pdev);
	if (ret < 0) {
		dev_err(ipu->dev, "ipu clk setup failed\n");
		goto failed_clk_setup;
	}

	platform_set_drvdata(pdev, ipu);

	ipu_reset(ipu);

	ipu_disp_init(ipu);

	/* Set sync refresh channels and CSI->mem channel as high priority */
	ipu_idmac_write(ipu, 0x18800001L, IDMAC_CHA_PRI(0));

	/* Set MCU_T to divide MCU access window into 2 */
	ipu_cm_write(ipu, 0x00400000L | (IPU_MCU_T_DEFAULT << 18), IPU_DISP_GEN);

	clk_disable(ipu->ipu_clk);

	register_ipu_device(ipu, pdev->id);

	ipu->online = true;

	return ret;

failed_clk_setup:
	iounmap(ipu->cm_reg);
	iounmap(ipu->ic_reg);
	iounmap(ipu->idmac_reg);
	iounmap(ipu->dc_reg);
	iounmap(ipu->dp_reg);
	iounmap(ipu->dmfc_reg);
	iounmap(ipu->di_reg[0]);
	iounmap(ipu->di_reg[1]);
	iounmap(ipu->smfc_reg);
	iounmap(ipu->csi_reg[0]);
	iounmap(ipu->csi_reg[1]);
	iounmap(ipu->cpmem_base);
	iounmap(ipu->tpmem_base);
	iounmap(ipu->dc_tmpl_reg);
	iounmap(ipu->disp_base[1]);
	iounmap(ipu->vdi_reg);
failed_ioremap:
	if (ipu->irq_sync)
		free_irq(ipu->irq_err, ipu);
failed_req_irq_err:
	free_irq(ipu->irq_sync, ipu);
failed_req_irq_sync:
failed_get_res:
	return ret;
}

int __devexit ipu_remove(struct platform_device *pdev)
{
	struct ipu_soc *ipu = platform_get_drvdata(pdev);

	unregister_ipu_device(ipu, pdev->id);

	if (ipu->irq_sync)
		free_irq(ipu->irq_sync, ipu);
	if (ipu->irq_err)
		free_irq(ipu->irq_err, ipu);

	clk_put(ipu->ipu_clk);

	iounmap(ipu->cm_reg);
	iounmap(ipu->ic_reg);
	iounmap(ipu->idmac_reg);
	iounmap(ipu->dc_reg);
	iounmap(ipu->dp_reg);
	iounmap(ipu->dmfc_reg);
	iounmap(ipu->di_reg[0]);
	iounmap(ipu->di_reg[1]);
	iounmap(ipu->smfc_reg);
	iounmap(ipu->csi_reg[0]);
	iounmap(ipu->csi_reg[1]);
	iounmap(ipu->cpmem_base);
	iounmap(ipu->tpmem_base);
	iounmap(ipu->dc_tmpl_reg);
	iounmap(ipu->disp_base[1]);
	iounmap(ipu->vdi_reg);

	return 0;
}

void ipu_dump_registers(struct ipu_soc *ipu)
{
	dev_dbg(ipu->dev, "IPU_CONF = \t0x%08X\n", ipu_cm_read(ipu, IPU_CONF));
	dev_dbg(ipu->dev, "IDMAC_CONF = \t0x%08X\n", ipu_idmac_read(ipu, IDMAC_CONF));
	dev_dbg(ipu->dev, "IDMAC_CHA_EN1 = \t0x%08X\n",
	       ipu_idmac_read(ipu, IDMAC_CHA_EN(0)));
	dev_dbg(ipu->dev, "IDMAC_CHA_EN2 = \t0x%08X\n",
	       ipu_idmac_read(ipu, IDMAC_CHA_EN(32)));
	dev_dbg(ipu->dev, "IDMAC_CHA_PRI1 = \t0x%08X\n",
	       ipu_idmac_read(ipu, IDMAC_CHA_PRI(0)));
	dev_dbg(ipu->dev, "IDMAC_CHA_PRI2 = \t0x%08X\n",
	       ipu_idmac_read(ipu, IDMAC_CHA_PRI(32)));
	dev_dbg(ipu->dev, "IDMAC_BAND_EN1 = \t0x%08X\n",
	       ipu_idmac_read(ipu, IDMAC_BAND_EN(0)));
	dev_dbg(ipu->dev, "IDMAC_BAND_EN2 = \t0x%08X\n",
	       ipu_idmac_read(ipu, IDMAC_BAND_EN(32)));
	dev_dbg(ipu->dev, "IPU_CHA_DB_MODE_SEL0 = \t0x%08X\n",
	       ipu_cm_read(ipu, IPU_CHA_DB_MODE_SEL(0)));
	dev_dbg(ipu->dev, "IPU_CHA_DB_MODE_SEL1 = \t0x%08X\n",
	       ipu_cm_read(ipu, IPU_CHA_DB_MODE_SEL(32)));
	if (g_ipu_hw_rev >= 2) {
		dev_dbg(ipu->dev, "IPU_CHA_TRB_MODE_SEL0 = \t0x%08X\n",
		       ipu_cm_read(ipu, IPU_CHA_TRB_MODE_SEL(0)));
		dev_dbg(ipu->dev, "IPU_CHA_TRB_MODE_SEL1 = \t0x%08X\n",
		       ipu_cm_read(ipu, IPU_CHA_TRB_MODE_SEL(32)));
	}
	dev_dbg(ipu->dev, "DMFC_WR_CHAN = \t0x%08X\n",
	       ipu_dmfc_read(ipu, DMFC_WR_CHAN));
	dev_dbg(ipu->dev, "DMFC_WR_CHAN_DEF = \t0x%08X\n",
	       ipu_dmfc_read(ipu, DMFC_WR_CHAN_DEF));
	dev_dbg(ipu->dev, "DMFC_DP_CHAN = \t0x%08X\n",
	       ipu_dmfc_read(ipu, DMFC_DP_CHAN));
	dev_dbg(ipu->dev, "DMFC_DP_CHAN_DEF = \t0x%08X\n",
	       ipu_dmfc_read(ipu, DMFC_DP_CHAN_DEF));
	dev_dbg(ipu->dev, "DMFC_IC_CTRL = \t0x%08X\n",
	       ipu_dmfc_read(ipu, DMFC_IC_CTRL));
	dev_dbg(ipu->dev, "IPU_FS_PROC_FLOW1 = \t0x%08X\n",
	       ipu_cm_read(ipu, IPU_FS_PROC_FLOW1));
	dev_dbg(ipu->dev, "IPU_FS_PROC_FLOW2 = \t0x%08X\n",
	       ipu_cm_read(ipu, IPU_FS_PROC_FLOW2));
	dev_dbg(ipu->dev, "IPU_FS_PROC_FLOW3 = \t0x%08X\n",
	       ipu_cm_read(ipu, IPU_FS_PROC_FLOW3));
	dev_dbg(ipu->dev, "IPU_FS_DISP_FLOW1 = \t0x%08X\n",
	       ipu_cm_read(ipu, IPU_FS_DISP_FLOW1));
}

/*!
 * This function is called to initialize a logical IPU channel.
 *
 * @param	ipu	ipu handler
 * @param       channel Input parameter for the logical channel ID to init.
 *
 * @param       params  Input parameter containing union of channel
 *                      initialization parameters.
 *
 * @return      Returns 0 on success or negative error code on fail
 */
int32_t ipu_init_channel(struct ipu_soc *ipu, ipu_channel_t channel, ipu_channel_params_t *params)
{
	int ret = 0;
	uint32_t ipu_conf;
	uint32_t reg;

	dev_dbg(ipu->dev, "init channel = %d\n", IPU_CHAN_ID(channel));

	_ipu_get(ipu);

	_ipu_lock(ipu);

	if (ipu->channel_init_mask & (1L << IPU_CHAN_ID(channel))) {
		dev_warn(ipu->dev, "Warning: channel already initialized %d\n",
			IPU_CHAN_ID(channel));
	}

	ipu_conf = ipu_cm_read(ipu, IPU_CONF);

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
			ipu->chan_is_interlaced[channel_2_dma(channel,
				IPU_OUTPUT_BUFFER)] = true;
		else
			ipu->chan_is_interlaced[channel_2_dma(channel,
				IPU_OUTPUT_BUFFER)] = false;

		ipu->smfc_use_count++;
		ipu->csi_channel[params->csi_mem.csi] = channel;

		/*SMFC setting*/
		if (params->csi_mem.mipi_en) {
			ipu_conf |= (1 << (IPU_CONF_CSI0_DATA_SOURCE_OFFSET +
				params->csi_mem.csi));
			_ipu_smfc_init(ipu, channel, params->csi_mem.mipi_vc,
				params->csi_mem.csi);
			_ipu_csi_set_mipi_di(ipu, params->csi_mem.mipi_vc,
				params->csi_mem.mipi_id, params->csi_mem.csi);
		} else {
			ipu_conf &= ~(1 << (IPU_CONF_CSI0_DATA_SOURCE_OFFSET +
				params->csi_mem.csi));
			_ipu_smfc_init(ipu, channel, 0, params->csi_mem.csi);
		}

		/*CSI data (include compander) dest*/
		_ipu_csi_init(ipu, channel, params->csi_mem.csi);
		break;
	case CSI_PRP_ENC_MEM:
		if (params->csi_prp_enc_mem.csi > 1) {
			ret = -EINVAL;
			goto err;
		}
		if (ipu->using_ic_dirct_ch == MEM_VDI_PRP_VF_MEM) {
			ret = -EINVAL;
			goto err;
		}
		ipu->using_ic_dirct_ch = CSI_PRP_ENC_MEM;

		ipu->ic_use_count++;
		ipu->csi_channel[params->csi_prp_enc_mem.csi] = channel;

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
		reg = ipu_cm_read(ipu, IPU_FS_PROC_FLOW1);
		ipu_cm_write(ipu, reg & ~FS_ENC_IN_VALID, IPU_FS_PROC_FLOW1);

		/*CSI data (include compander) dest*/
		_ipu_csi_init(ipu, channel, params->csi_prp_enc_mem.csi);
		_ipu_ic_init_prpenc(ipu, params, true);
		break;
	case CSI_PRP_VF_MEM:
		if (params->csi_prp_vf_mem.csi > 1) {
			ret = -EINVAL;
			goto err;
		}
		if (ipu->using_ic_dirct_ch == MEM_VDI_PRP_VF_MEM) {
			ret = -EINVAL;
			goto err;
		}
		ipu->using_ic_dirct_ch = CSI_PRP_VF_MEM;

		ipu->ic_use_count++;
		ipu->csi_channel[params->csi_prp_vf_mem.csi] = channel;

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
		reg = ipu_cm_read(ipu, IPU_FS_PROC_FLOW1);
		ipu_cm_write(ipu, reg & ~FS_VF_IN_VALID, IPU_FS_PROC_FLOW1);

		/*CSI data (include compander) dest*/
		_ipu_csi_init(ipu, channel, params->csi_prp_vf_mem.csi);
		_ipu_ic_init_prpvf(ipu, params, true);
		break;
	case MEM_PRP_VF_MEM:
		ipu->ic_use_count++;
		reg = ipu_cm_read(ipu, IPU_FS_PROC_FLOW1);
		ipu_cm_write(ipu, reg | FS_VF_IN_VALID, IPU_FS_PROC_FLOW1);

		if (params->mem_prp_vf_mem.graphics_combine_en)
			ipu->sec_chan_en[IPU_CHAN_ID(channel)] = true;
		if (params->mem_prp_vf_mem.alpha_chan_en)
			ipu->thrd_chan_en[IPU_CHAN_ID(channel)] = true;

		_ipu_ic_init_prpvf(ipu, params, false);
		break;
	case MEM_VDI_PRP_VF_MEM:
		if ((ipu->using_ic_dirct_ch == CSI_PRP_VF_MEM) ||
		     (ipu->using_ic_dirct_ch == CSI_PRP_ENC_MEM)) {
			ret = -EINVAL;
			goto err;
		}
		ipu->using_ic_dirct_ch = MEM_VDI_PRP_VF_MEM;
		ipu->ic_use_count++;
		ipu->vdi_use_count++;
		reg = ipu_cm_read(ipu, IPU_FS_PROC_FLOW1);
		reg &= ~FS_VDI_SRC_SEL_MASK;
		ipu_cm_write(ipu, reg , IPU_FS_PROC_FLOW1);

		if (params->mem_prp_vf_mem.graphics_combine_en)
			ipu->sec_chan_en[IPU_CHAN_ID(channel)] = true;
		_ipu_ic_init_prpvf(ipu, params, false);
		_ipu_vdi_init(ipu, channel, params);
		break;
	case MEM_VDI_PRP_VF_MEM_P:
		_ipu_vdi_init(ipu, channel, params);
		break;
	case MEM_VDI_PRP_VF_MEM_N:
		_ipu_vdi_init(ipu, channel, params);
		break;
	case MEM_ROT_VF_MEM:
		ipu->ic_use_count++;
		ipu->rot_use_count++;
		_ipu_ic_init_rotate_vf(ipu, params);
		break;
	case MEM_PRP_ENC_MEM:
		ipu->ic_use_count++;
		reg = ipu_cm_read(ipu, IPU_FS_PROC_FLOW1);
		ipu_cm_write(ipu, reg | FS_ENC_IN_VALID, IPU_FS_PROC_FLOW1);
		_ipu_ic_init_prpenc(ipu, params, false);
		break;
	case MEM_ROT_ENC_MEM:
		ipu->ic_use_count++;
		ipu->rot_use_count++;
		_ipu_ic_init_rotate_enc(ipu, params);
		break;
	case MEM_PP_MEM:
		if (params->mem_pp_mem.graphics_combine_en)
			ipu->sec_chan_en[IPU_CHAN_ID(channel)] = true;
		if (params->mem_pp_mem.alpha_chan_en)
			ipu->thrd_chan_en[IPU_CHAN_ID(channel)] = true;
		_ipu_ic_init_pp(ipu, params);
		ipu->ic_use_count++;
		break;
	case MEM_ROT_PP_MEM:
		_ipu_ic_init_rotate_pp(ipu, params);
		ipu->ic_use_count++;
		ipu->rot_use_count++;
		break;
	case MEM_DC_SYNC:
		if (params->mem_dc_sync.di > 1) {
			ret = -EINVAL;
			goto err;
		}

		ipu->dc_di_assignment[1] = params->mem_dc_sync.di;
		_ipu_dc_init(ipu, 1, params->mem_dc_sync.di,
			     params->mem_dc_sync.interlaced,
			     params->mem_dc_sync.out_pixel_fmt);
		ipu->di_use_count[params->mem_dc_sync.di]++;
		ipu->dc_use_count++;
		ipu->dmfc_use_count++;
		break;
	case MEM_BG_SYNC:
		if (params->mem_dp_bg_sync.di > 1) {
			ret = -EINVAL;
			goto err;
		}

		if (params->mem_dp_bg_sync.alpha_chan_en)
			ipu->thrd_chan_en[IPU_CHAN_ID(channel)] = true;

		ipu->dc_di_assignment[5] = params->mem_dp_bg_sync.di;
		_ipu_dp_init(ipu, channel, params->mem_dp_bg_sync.in_pixel_fmt,
			     params->mem_dp_bg_sync.out_pixel_fmt);
		_ipu_dc_init(ipu, 5, params->mem_dp_bg_sync.di,
			     params->mem_dp_bg_sync.interlaced,
			     params->mem_dp_bg_sync.out_pixel_fmt);
		ipu->di_use_count[params->mem_dp_bg_sync.di]++;
		ipu->dc_use_count++;
		ipu->dp_use_count++;
		ipu->dmfc_use_count++;
		break;
	case MEM_FG_SYNC:
		_ipu_dp_init(ipu, channel, params->mem_dp_fg_sync.in_pixel_fmt,
			     params->mem_dp_fg_sync.out_pixel_fmt);

		if (params->mem_dp_fg_sync.alpha_chan_en)
			ipu->thrd_chan_en[IPU_CHAN_ID(channel)] = true;

		ipu->dc_use_count++;
		ipu->dp_use_count++;
		ipu->dmfc_use_count++;
		break;
	case DIRECT_ASYNC0:
		if (params->direct_async.di > 1) {
			ret = -EINVAL;
			goto err;
		}

		ipu->dc_di_assignment[8] = params->direct_async.di;
		_ipu_dc_init(ipu, 8, params->direct_async.di, false, IPU_PIX_FMT_GENERIC);
		ipu->di_use_count[params->direct_async.di]++;
		ipu->dc_use_count++;
		break;
	case DIRECT_ASYNC1:
		if (params->direct_async.di > 1) {
			ret = -EINVAL;
			goto err;
		}

		ipu->dc_di_assignment[9] = params->direct_async.di;
		_ipu_dc_init(ipu, 9, params->direct_async.di, false, IPU_PIX_FMT_GENERIC);
		ipu->di_use_count[params->direct_async.di]++;
		ipu->dc_use_count++;
		break;
	default:
		dev_err(ipu->dev, "Missing channel initialization\n");
		break;
	}

	ipu->channel_init_mask |= 1L << IPU_CHAN_ID(channel);

	ipu_cm_write(ipu, ipu_conf, IPU_CONF);

err:
	_ipu_unlock(ipu);
	return ret;
}
EXPORT_SYMBOL(ipu_init_channel);

/*!
 * This function is called to uninitialize a logical IPU channel.
 *
 * @param	ipu	ipu handler
 * @param       channel Input parameter for the logical channel ID to uninit.
 */
void ipu_uninit_channel(struct ipu_soc *ipu, ipu_channel_t channel)
{
	uint32_t reg;
	uint32_t in_dma, out_dma = 0;
	uint32_t ipu_conf;

	_ipu_lock(ipu);

	if ((ipu->channel_init_mask & (1L << IPU_CHAN_ID(channel))) == 0) {
		dev_err(ipu->dev, "Channel already uninitialized %d\n",
			IPU_CHAN_ID(channel));
		_ipu_unlock(ipu);
		return;
	}

	/* Make sure channel is disabled */
	/* Get input and output dma channels */
	in_dma = channel_2_dma(channel, IPU_VIDEO_IN_BUFFER);
	out_dma = channel_2_dma(channel, IPU_OUTPUT_BUFFER);

	if (idma_is_set(ipu, IDMAC_CHA_EN, in_dma) ||
	    idma_is_set(ipu, IDMAC_CHA_EN, out_dma)) {
		dev_err(ipu->dev,
			"Channel %d is not disabled, disable first\n",
			IPU_CHAN_ID(channel));
		_ipu_unlock(ipu);
		return;
	}

	ipu_conf = ipu_cm_read(ipu, IPU_CONF);

	/* Reset the double buffer */
	reg = ipu_cm_read(ipu, IPU_CHA_DB_MODE_SEL(in_dma));
	ipu_cm_write(ipu, reg & ~idma_mask(in_dma), IPU_CHA_DB_MODE_SEL(in_dma));
	reg = ipu_cm_read(ipu, IPU_CHA_DB_MODE_SEL(out_dma));
	ipu_cm_write(ipu, reg & ~idma_mask(out_dma), IPU_CHA_DB_MODE_SEL(out_dma));

	/* Reset the triple buffer */
	reg = ipu_cm_read(ipu, IPU_CHA_TRB_MODE_SEL(in_dma));
	ipu_cm_write(ipu, reg & ~idma_mask(in_dma), IPU_CHA_TRB_MODE_SEL(in_dma));
	reg = ipu_cm_read(ipu, IPU_CHA_TRB_MODE_SEL(out_dma));
	ipu_cm_write(ipu, reg & ~idma_mask(out_dma), IPU_CHA_TRB_MODE_SEL(out_dma));

	if (_ipu_is_ic_chan(in_dma) || _ipu_is_dp_graphic_chan(in_dma)) {
		ipu->sec_chan_en[IPU_CHAN_ID(channel)] = false;
		ipu->thrd_chan_en[IPU_CHAN_ID(channel)] = false;
	}

	switch (channel) {
	case CSI_MEM0:
	case CSI_MEM1:
	case CSI_MEM2:
	case CSI_MEM3:
		ipu->smfc_use_count--;
		if (ipu->csi_channel[0] == channel) {
			ipu->csi_channel[0] = CHAN_NONE;
		} else if (ipu->csi_channel[1] == channel) {
			ipu->csi_channel[1] = CHAN_NONE;
		}
		break;
	case CSI_PRP_ENC_MEM:
		ipu->ic_use_count--;
		if (ipu->using_ic_dirct_ch == CSI_PRP_ENC_MEM)
			ipu->using_ic_dirct_ch = 0;
		_ipu_ic_uninit_prpenc(ipu);
		if (ipu->csi_channel[0] == channel) {
			ipu->csi_channel[0] = CHAN_NONE;
		} else if (ipu->csi_channel[1] == channel) {
			ipu->csi_channel[1] = CHAN_NONE;
		}
		break;
	case CSI_PRP_VF_MEM:
		ipu->ic_use_count--;
		if (ipu->using_ic_dirct_ch == CSI_PRP_VF_MEM)
			ipu->using_ic_dirct_ch = 0;
		_ipu_ic_uninit_prpvf(ipu);
		if (ipu->csi_channel[0] == channel) {
			ipu->csi_channel[0] = CHAN_NONE;
		} else if (ipu->csi_channel[1] == channel) {
			ipu->csi_channel[1] = CHAN_NONE;
		}
		break;
	case MEM_PRP_VF_MEM:
		ipu->ic_use_count--;
		_ipu_ic_uninit_prpvf(ipu);
		reg = ipu_cm_read(ipu, IPU_FS_PROC_FLOW1);
		ipu_cm_write(ipu, reg & ~FS_VF_IN_VALID, IPU_FS_PROC_FLOW1);
		break;
	case MEM_VDI_PRP_VF_MEM:
		ipu->ic_use_count--;
		ipu->vdi_use_count--;
		if (ipu->using_ic_dirct_ch == MEM_VDI_PRP_VF_MEM)
			ipu->using_ic_dirct_ch = 0;
		_ipu_ic_uninit_prpvf(ipu);
		_ipu_vdi_uninit(ipu);
		reg = ipu_cm_read(ipu, IPU_FS_PROC_FLOW1);
		ipu_cm_write(ipu, reg & ~FS_VF_IN_VALID, IPU_FS_PROC_FLOW1);
		break;
	case MEM_VDI_PRP_VF_MEM_P:
	case MEM_VDI_PRP_VF_MEM_N:
		break;
	case MEM_ROT_VF_MEM:
		ipu->rot_use_count--;
		ipu->ic_use_count--;
		_ipu_ic_uninit_rotate_vf(ipu);
		break;
	case MEM_PRP_ENC_MEM:
		ipu->ic_use_count--;
		_ipu_ic_uninit_prpenc(ipu);
		reg = ipu_cm_read(ipu, IPU_FS_PROC_FLOW1);
		ipu_cm_write(ipu, reg & ~FS_ENC_IN_VALID, IPU_FS_PROC_FLOW1);
		break;
	case MEM_ROT_ENC_MEM:
		ipu->rot_use_count--;
		ipu->ic_use_count--;
		_ipu_ic_uninit_rotate_enc(ipu);
		break;
	case MEM_PP_MEM:
		ipu->ic_use_count--;
		_ipu_ic_uninit_pp(ipu);
		break;
	case MEM_ROT_PP_MEM:
		ipu->rot_use_count--;
		ipu->ic_use_count--;
		_ipu_ic_uninit_rotate_pp(ipu);
		break;
	case MEM_DC_SYNC:
		_ipu_dc_uninit(ipu, 1);
		ipu->di_use_count[ipu->dc_di_assignment[1]]--;
		ipu->dc_use_count--;
		ipu->dmfc_use_count--;
		break;
	case MEM_BG_SYNC:
		_ipu_dp_uninit(ipu, channel);
		_ipu_dc_uninit(ipu, 5);
		ipu->di_use_count[ipu->dc_di_assignment[5]]--;
		ipu->dc_use_count--;
		ipu->dp_use_count--;
		ipu->dmfc_use_count--;
		break;
	case MEM_FG_SYNC:
		_ipu_dp_uninit(ipu, channel);
		ipu->dc_use_count--;
		ipu->dp_use_count--;
		ipu->dmfc_use_count--;
		break;
	case DIRECT_ASYNC0:
		_ipu_dc_uninit(ipu, 8);
		ipu->di_use_count[ipu->dc_di_assignment[8]]--;
		ipu->dc_use_count--;
		break;
	case DIRECT_ASYNC1:
		_ipu_dc_uninit(ipu, 9);
		ipu->di_use_count[ipu->dc_di_assignment[9]]--;
		ipu->dc_use_count--;
		break;
	default:
		break;
	}

	if (ipu->ic_use_count == 0)
		ipu_conf &= ~IPU_CONF_IC_EN;
	if (ipu->vdi_use_count == 0) {
		ipu_conf &= ~IPU_CONF_ISP_EN;
		ipu_conf &= ~IPU_CONF_VDI_EN;
		ipu_conf &= ~IPU_CONF_IC_INPUT;
	}
	if (ipu->rot_use_count == 0)
		ipu_conf &= ~IPU_CONF_ROT_EN;
	if (ipu->dc_use_count == 0)
		ipu_conf &= ~IPU_CONF_DC_EN;
	if (ipu->dp_use_count == 0)
		ipu_conf &= ~IPU_CONF_DP_EN;
	if (ipu->dmfc_use_count == 0)
		ipu_conf &= ~IPU_CONF_DMFC_EN;
	if (ipu->di_use_count[0] == 0) {
		ipu_conf &= ~IPU_CONF_DI0_EN;
	}
	if (ipu->di_use_count[1] == 0) {
		ipu_conf &= ~IPU_CONF_DI1_EN;
	}
	if (ipu->smfc_use_count == 0)
		ipu_conf &= ~IPU_CONF_SMFC_EN;

	ipu_cm_write(ipu, ipu_conf, IPU_CONF);

	ipu->channel_init_mask &= ~(1L << IPU_CHAN_ID(channel));

	_ipu_unlock(ipu);

	_ipu_put(ipu);

	WARN_ON(ipu->ic_use_count < 0);
	WARN_ON(ipu->vdi_use_count < 0);
	WARN_ON(ipu->rot_use_count < 0);
	WARN_ON(ipu->dc_use_count < 0);
	WARN_ON(ipu->dp_use_count < 0);
	WARN_ON(ipu->dmfc_use_count < 0);
	WARN_ON(ipu->smfc_use_count < 0);
}
EXPORT_SYMBOL(ipu_uninit_channel);

/*!
 * This function is called to initialize buffer(s) for logical IPU channel.
 *
 * @param	ipu		ipu handler
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
 * @param       phyaddr_2       Input parameter buffer 2 physical address.
 *                              Setting this to a value other than NULL enables
 *                              triple buffering mode, phyaddr_1 should not be
 *                              NULL then.
 *
 * @param       u		private u offset for additional cropping,
 *				zero if not used.
 *
 * @param       v		private v offset for additional cropping,
 *				zero if not used.
 *
 * @return      Returns 0 on success or negative error code on fail
 */
int32_t ipu_init_channel_buffer(struct ipu_soc *ipu, ipu_channel_t channel,
				ipu_buffer_t type,
				uint32_t pixel_fmt,
				uint16_t width, uint16_t height,
				uint32_t stride,
				ipu_rotate_mode_t rot_mode,
				dma_addr_t phyaddr_0, dma_addr_t phyaddr_1,
				dma_addr_t phyaddr_2,
				uint32_t u, uint32_t v)
{
	uint32_t reg;
	uint32_t dma_chan;
	uint32_t burst_size;

	dma_chan = channel_2_dma(channel, type);
	if (!idma_is_valid(dma_chan))
		return -EINVAL;

	if (stride < width * bytes_per_pixel(pixel_fmt))
		stride = width * bytes_per_pixel(pixel_fmt);

	if (stride % 4) {
		dev_err(ipu->dev,
			"Stride not 32-bit aligned, stride = %d\n", stride);
		return -EINVAL;
	}
	/* IC & IRT channels' width must be multiple of 8 pixels */
	if ((_ipu_is_ic_chan(dma_chan) || _ipu_is_irt_chan(dma_chan))
		&& (width % 8)) {
		dev_err(ipu->dev, "Width must be 8 pixel multiple\n");
		return -EINVAL;
	}

	/* IPUv3EX and IPUv3M support triple buffer */
	if ((!_ipu_is_trb_chan(dma_chan)) && phyaddr_2) {
		dev_err(ipu->dev, "Chan%d doesn't support triple buffer "
				   "mode\n", dma_chan);
		return -EINVAL;
	}
	if (!phyaddr_1 && phyaddr_2) {
		dev_err(ipu->dev, "Chan%d's buf1 physical addr is NULL for "
				   "triple buffer mode\n", dma_chan);
		return -EINVAL;
	}

	_ipu_lock(ipu);

	/* Build parameter memory data for DMA channel */
	_ipu_ch_param_init(ipu, dma_chan, pixel_fmt, width, height, stride, u, v, 0,
			   phyaddr_0, phyaddr_1, phyaddr_2);

	/* Set correlative channel parameter of local alpha channel */
	if ((_ipu_is_ic_graphic_chan(dma_chan) ||
	     _ipu_is_dp_graphic_chan(dma_chan)) &&
	    (ipu->thrd_chan_en[IPU_CHAN_ID(channel)] == true)) {
		_ipu_ch_param_set_alpha_use_separate_channel(ipu, dma_chan, true);
		_ipu_ch_param_set_alpha_buffer_memory(ipu, dma_chan);
		_ipu_ch_param_set_alpha_condition_read(ipu, dma_chan);
		/* fix alpha width as 8 and burst size as 16*/
		_ipu_ch_params_set_alpha_width(ipu, dma_chan, 8);
		_ipu_ch_param_set_burst_size(ipu, dma_chan, 16);
	} else if (_ipu_is_ic_graphic_chan(dma_chan) &&
		   ipu_pixel_format_has_alpha(pixel_fmt))
		_ipu_ch_param_set_alpha_use_separate_channel(ipu, dma_chan, false);

	if (rot_mode)
		_ipu_ch_param_set_rotation(ipu, dma_chan, rot_mode);

	/* IC and ROT channels have restriction of 8 or 16 pix burst length */
	if (_ipu_is_ic_chan(dma_chan)) {
		if ((width % 16) == 0)
			_ipu_ch_param_set_burst_size(ipu, dma_chan, 16);
		else
			_ipu_ch_param_set_burst_size(ipu, dma_chan, 8);
	} else if (_ipu_is_irt_chan(dma_chan)) {
		_ipu_ch_param_set_burst_size(ipu, dma_chan, 8);
		_ipu_ch_param_set_block_mode(ipu, dma_chan);
	} else if (_ipu_is_dmfc_chan(dma_chan)) {
		burst_size = _ipu_ch_param_get_burst_size(ipu, dma_chan);
		_ipu_dmfc_set_wait4eot(ipu, dma_chan, width);
		_ipu_dmfc_set_burst_size(ipu, dma_chan, burst_size);
	}

	if (_ipu_disp_chan_is_interlaced(ipu, channel) ||
		ipu->chan_is_interlaced[dma_chan])
		_ipu_ch_param_set_interlaced_scan(ipu, dma_chan);

	if (_ipu_is_ic_chan(dma_chan) || _ipu_is_irt_chan(dma_chan)) {
		burst_size = _ipu_ch_param_get_burst_size(ipu, dma_chan);
		_ipu_ic_idma_init(ipu, dma_chan, width, height, burst_size,
			rot_mode);
	} else if (_ipu_is_smfc_chan(dma_chan)) {
		burst_size = _ipu_ch_param_get_burst_size(ipu, dma_chan);
		if ((pixel_fmt == IPU_PIX_FMT_GENERIC) &&
			((_ipu_ch_param_get_bpp(ipu, dma_chan) == 5) ||
			(_ipu_ch_param_get_bpp(ipu, dma_chan) == 3)))
			burst_size = burst_size >> 4;
		else
			burst_size = burst_size >> 2;
		_ipu_smfc_set_burst_size(ipu, channel, burst_size-1);
	}

	/* AXI-id */
	if (idma_is_set(ipu, IDMAC_CHA_PRI, dma_chan)) {
		unsigned reg = IDMAC_CH_LOCK_EN_1;
		uint32_t value = 0;
		if (cpu_is_mx53() || cpu_is_mx6q()) {
			_ipu_ch_param_set_axi_id(ipu, dma_chan, 0);
			switch (dma_chan) {
			case 5:
				value = 0x3;
				break;
			case 11:
				value = 0x3 << 2;
				break;
			case 12:
				value = 0x3 << 4;
				break;
			case 14:
				value = 0x3 << 6;
				break;
			case 15:
				value = 0x3 << 8;
				break;
			case 20:
				value = 0x3 << 10;
				break;
			case 21:
				value = 0x3 << 12;
				break;
			case 22:
				value = 0x3 << 14;
				break;
			case 23:
				value = 0x3 << 16;
				break;
			case 27:
				value = 0x3 << 18;
				break;
			case 28:
				value = 0x3 << 20;
				break;
			case 45:
				reg = IDMAC_CH_LOCK_EN_2;
				value = 0x3 << 0;
				break;
			case 46:
				reg = IDMAC_CH_LOCK_EN_2;
				value = 0x3 << 2;
				break;
			case 47:
				reg = IDMAC_CH_LOCK_EN_2;
				value = 0x3 << 4;
				break;
			case 48:
				reg = IDMAC_CH_LOCK_EN_2;
				value = 0x3 << 6;
				break;
			case 49:
				reg = IDMAC_CH_LOCK_EN_2;
				value = 0x3 << 8;
				break;
			case 50:
				reg = IDMAC_CH_LOCK_EN_2;
				value = 0x3 << 10;
				break;
			default:
				break;
			}
			value |= ipu_idmac_read(ipu, reg);
			ipu_idmac_write(ipu, value, reg);
		} else
			_ipu_ch_param_set_axi_id(ipu, dma_chan, 1);
	} else {
		if (cpu_is_mx6q())
			_ipu_ch_param_set_axi_id(ipu, dma_chan, 1);
	}

	_ipu_ch_param_dump(ipu, dma_chan);

	if (phyaddr_2 && g_ipu_hw_rev >= 2) {
		reg = ipu_cm_read(ipu, IPU_CHA_DB_MODE_SEL(dma_chan));
		reg &= ~idma_mask(dma_chan);
		ipu_cm_write(ipu, reg, IPU_CHA_DB_MODE_SEL(dma_chan));

		reg = ipu_cm_read(ipu, IPU_CHA_TRB_MODE_SEL(dma_chan));
		reg |= idma_mask(dma_chan);
		ipu_cm_write(ipu, reg, IPU_CHA_TRB_MODE_SEL(dma_chan));

		/* Set IDMAC third buffer's cpmem number */
		/* See __ipu_ch_get_third_buf_cpmem_num() for mapping */
		ipu_idmac_write(ipu, 0x00444047L, IDMAC_SUB_ADDR_4);
		ipu_idmac_write(ipu, 0x46004241L, IDMAC_SUB_ADDR_3);
		ipu_idmac_write(ipu, 0x00000045L, IDMAC_SUB_ADDR_1);

		/* Reset to buffer 0 */
		ipu_cm_write(ipu, tri_cur_buf_mask(dma_chan),
				IPU_CHA_TRIPLE_CUR_BUF(dma_chan));
	} else {
		reg = ipu_cm_read(ipu, IPU_CHA_TRB_MODE_SEL(dma_chan));
		reg &= ~idma_mask(dma_chan);
		ipu_cm_write(ipu, reg, IPU_CHA_TRB_MODE_SEL(dma_chan));

		reg = ipu_cm_read(ipu, IPU_CHA_DB_MODE_SEL(dma_chan));
		if (phyaddr_1)
			reg |= idma_mask(dma_chan);
		else
			reg &= ~idma_mask(dma_chan);
		ipu_cm_write(ipu, reg, IPU_CHA_DB_MODE_SEL(dma_chan));

		/* Reset to buffer 0 */
		ipu_cm_write(ipu, idma_mask(dma_chan),
				IPU_CHA_CUR_BUF(dma_chan));

	}

	_ipu_unlock(ipu);

	return 0;
}
EXPORT_SYMBOL(ipu_init_channel_buffer);

/*!
 * This function is called to update the physical address of a buffer for
 * a logical IPU channel.
 *
 * @param	ipu		ipu handler
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
int32_t ipu_update_channel_buffer(struct ipu_soc *ipu, ipu_channel_t channel,
				ipu_buffer_t type, uint32_t bufNum, dma_addr_t phyaddr)
{
	uint32_t reg;
	int ret = 0;
	uint32_t dma_chan = channel_2_dma(channel, type);

	if (dma_chan == IDMA_CHAN_INVALID)
		return -EINVAL;

	_ipu_lock(ipu);

	if (bufNum == 0)
		reg = ipu_cm_read(ipu, IPU_CHA_BUF0_RDY(dma_chan));
	else if (bufNum == 1)
		reg = ipu_cm_read(ipu, IPU_CHA_BUF1_RDY(dma_chan));
	else
		reg = ipu_cm_read(ipu, IPU_CHA_BUF2_RDY(dma_chan));

	if ((reg & idma_mask(dma_chan)) == 0)
		_ipu_ch_param_set_buffer(ipu, dma_chan, bufNum, phyaddr);
	else
		ret = -EACCES;

	_ipu_unlock(ipu);

	return ret;
}
EXPORT_SYMBOL(ipu_update_channel_buffer);


/*!
 * This function is called to initialize a buffer for logical IPU channel.
 *
 * @param	ipu		ipu handler
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

int32_t ipu_update_channel_offset(struct ipu_soc *ipu,
				ipu_channel_t channel, ipu_buffer_t type,
				uint32_t pixel_fmt,
				uint16_t width, uint16_t height,
				uint32_t stride,
				uint32_t u, uint32_t v,
				uint32_t vertical_offset, uint32_t horizontal_offset)
{
	int ret = 0;
	uint32_t dma_chan = channel_2_dma(channel, type);

	if (dma_chan == IDMA_CHAN_INVALID)
		return -EINVAL;

	_ipu_lock(ipu);

	if ((ipu_cm_read(ipu, IPU_CHA_BUF0_RDY(dma_chan)) & idma_mask(dma_chan)) ||
	    (ipu_cm_read(ipu, IPU_CHA_BUF1_RDY(dma_chan)) & idma_mask(dma_chan)) ||
	    ((ipu_cm_read(ipu, IPU_CHA_BUF2_RDY(dma_chan)) & idma_mask(dma_chan)) &&
	     (ipu_cm_read(ipu, IPU_CHA_TRB_MODE_SEL(dma_chan)) & idma_mask(dma_chan)) &&
	     _ipu_is_trb_chan(dma_chan)))
		ret = -EACCES;
	else
		_ipu_ch_offset_update(ipu, dma_chan, pixel_fmt, width, height, stride,
				      u, v, 0, vertical_offset, horizontal_offset);

	_ipu_unlock(ipu);
	return ret;
}
EXPORT_SYMBOL(ipu_update_channel_offset);


/*!
 * This function is called to set a channel's buffer as ready.
 *
 * @param	ipu		ipu handler
 * @param       channel         Input parameter for the logical channel ID.
 *
 * @param       type            Input parameter which buffer to initialize.
 *
 * @param       bufNum          Input parameter for which buffer number set to
 *                              ready state.
 *
 * @return      Returns 0 on success or negative error code on fail
 */
int32_t ipu_select_buffer(struct ipu_soc *ipu, ipu_channel_t channel,
			ipu_buffer_t type, uint32_t bufNum)
{
	uint32_t dma_chan = channel_2_dma(channel, type);

	if (dma_chan == IDMA_CHAN_INVALID)
		return -EINVAL;

	/* Mark buffer to be ready. */
	_ipu_lock(ipu);
	if (bufNum == 0)
		ipu_cm_write(ipu, idma_mask(dma_chan),
			     IPU_CHA_BUF0_RDY(dma_chan));
	else if (bufNum == 1)
		ipu_cm_write(ipu, idma_mask(dma_chan),
			     IPU_CHA_BUF1_RDY(dma_chan));
	else
		ipu_cm_write(ipu, idma_mask(dma_chan),
			     IPU_CHA_BUF2_RDY(dma_chan));
	_ipu_unlock(ipu);
	return 0;
}
EXPORT_SYMBOL(ipu_select_buffer);

/*!
 * This function is called to set a channel's buffer as ready.
 *
 * @param	ipu		ipu handler
 * @param       bufNum          Input parameter for which buffer number set to
 *                              ready state.
 *
 * @return      Returns 0 on success or negative error code on fail
 */
int32_t ipu_select_multi_vdi_buffer(struct ipu_soc *ipu, uint32_t bufNum)
{

	uint32_t dma_chan = channel_2_dma(MEM_VDI_PRP_VF_MEM, IPU_INPUT_BUFFER);
	uint32_t mask_bit =
		idma_mask(channel_2_dma(MEM_VDI_PRP_VF_MEM_P, IPU_INPUT_BUFFER))|
		idma_mask(dma_chan)|
		idma_mask(channel_2_dma(MEM_VDI_PRP_VF_MEM_N, IPU_INPUT_BUFFER));

	/* Mark buffers to be ready. */
	_ipu_lock(ipu);
	if (bufNum == 0)
		ipu_cm_write(ipu, mask_bit, IPU_CHA_BUF0_RDY(dma_chan));
	else
		ipu_cm_write(ipu, mask_bit, IPU_CHA_BUF1_RDY(dma_chan));
	_ipu_unlock(ipu);
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
 * @param	ipu		ipu handler
 * @param       src_ch          Input parameter for the logical channel ID of
 *                              the source channel.
 *
 * @param       dest_ch         Input parameter for the logical channel ID of
 *                              the destination channel.
 *
 * @return      This function returns 0 on success or negative error code on
 *              fail.
 */
int32_t ipu_link_channels(struct ipu_soc *ipu, ipu_channel_t src_ch, ipu_channel_t dest_ch)
{
	int retval = 0;
	uint32_t fs_proc_flow1;
	uint32_t fs_proc_flow2;
	uint32_t fs_proc_flow3;
	uint32_t fs_disp_flow1;

	_ipu_lock(ipu);

	fs_proc_flow1 = ipu_cm_read(ipu, IPU_FS_PROC_FLOW1);
	fs_proc_flow2 = ipu_cm_read(ipu, IPU_FS_PROC_FLOW2);
	fs_proc_flow3 = ipu_cm_read(ipu, IPU_FS_PROC_FLOW3);
	fs_disp_flow1 = ipu_cm_read(ipu, IPU_FS_DISP_FLOW1);

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

	ipu_cm_write(ipu, fs_proc_flow1, IPU_FS_PROC_FLOW1);
	ipu_cm_write(ipu, fs_proc_flow2, IPU_FS_PROC_FLOW2);
	ipu_cm_write(ipu, fs_proc_flow3, IPU_FS_PROC_FLOW3);
	ipu_cm_write(ipu, fs_disp_flow1, IPU_FS_DISP_FLOW1);

err:
	_ipu_unlock(ipu);
	return retval;
}
EXPORT_SYMBOL(ipu_link_channels);

/*!
 * This function unlinks 2 channels and disables automatic frame
 * synchronization.
 *
 * @param	ipu		ipu handler
 * @param       src_ch          Input parameter for the logical channel ID of
 *                              the source channel.
 *
 * @param       dest_ch         Input parameter for the logical channel ID of
 *                              the destination channel.
 *
 * @return      This function returns 0 on success or negative error code on
 *              fail.
 */
int32_t ipu_unlink_channels(struct ipu_soc *ipu, ipu_channel_t src_ch, ipu_channel_t dest_ch)
{
	int retval = 0;
	uint32_t fs_proc_flow1;
	uint32_t fs_proc_flow2;
	uint32_t fs_proc_flow3;
	uint32_t fs_disp_flow1;

	_ipu_lock(ipu);

	fs_proc_flow1 = ipu_cm_read(ipu, IPU_FS_PROC_FLOW1);
	fs_proc_flow2 = ipu_cm_read(ipu, IPU_FS_PROC_FLOW2);
	fs_proc_flow3 = ipu_cm_read(ipu, IPU_FS_PROC_FLOW3);
	fs_disp_flow1 = ipu_cm_read(ipu, IPU_FS_DISP_FLOW1);

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

	ipu_cm_write(ipu, fs_proc_flow1, IPU_FS_PROC_FLOW1);
	ipu_cm_write(ipu, fs_proc_flow2, IPU_FS_PROC_FLOW2);
	ipu_cm_write(ipu, fs_proc_flow3, IPU_FS_PROC_FLOW3);
	ipu_cm_write(ipu, fs_disp_flow1, IPU_FS_DISP_FLOW1);

err:
	_ipu_unlock(ipu);
	return retval;
}
EXPORT_SYMBOL(ipu_unlink_channels);

/*!
 * This function check whether a logical channel was enabled.
 *
 * @param	ipu		ipu handler
 * @param       channel         Input parameter for the logical channel ID.
 *
 * @return      This function returns 1 while request channel is enabled or
 *              0 for not enabled.
 */
int32_t ipu_is_channel_busy(struct ipu_soc *ipu, ipu_channel_t channel)
{
	uint32_t reg;
	uint32_t in_dma;
	uint32_t out_dma;

	out_dma = channel_2_dma(channel, IPU_OUTPUT_BUFFER);
	in_dma = channel_2_dma(channel, IPU_VIDEO_IN_BUFFER);

	reg = ipu_idmac_read(ipu, IDMAC_CHA_EN(in_dma));
	if (reg & idma_mask(in_dma))
		return 1;
	reg = ipu_idmac_read(ipu, IDMAC_CHA_EN(out_dma));
	if (reg & idma_mask(out_dma))
		return 1;
	return 0;
}
EXPORT_SYMBOL(ipu_is_channel_busy);

/*!
 * This function enables a logical channel.
 *
 * @param	ipu		ipu handler
 * @param       channel         Input parameter for the logical channel ID.
 *
 * @return      This function returns 0 on success or negative error code on
 *              fail.
 */
int32_t ipu_enable_channel(struct ipu_soc *ipu, ipu_channel_t channel)
{
	uint32_t reg;
	uint32_t ipu_conf;
	uint32_t in_dma;
	uint32_t out_dma;
	uint32_t sec_dma;
	uint32_t thrd_dma;

	_ipu_lock(ipu);

	if (ipu->channel_enable_mask & (1L << IPU_CHAN_ID(channel))) {
		dev_err(ipu->dev, "Warning: channel already enabled %d\n",
			IPU_CHAN_ID(channel));
		_ipu_unlock(ipu);
		return -EACCES;
	}

	/* Get input and output dma channels */
	out_dma = channel_2_dma(channel, IPU_OUTPUT_BUFFER);
	in_dma = channel_2_dma(channel, IPU_VIDEO_IN_BUFFER);

	ipu_conf = ipu_cm_read(ipu, IPU_CONF);
	if (ipu->di_use_count[0] > 0) {
		ipu_conf |= IPU_CONF_DI0_EN;
	}
	if (ipu->di_use_count[1] > 0) {
		ipu_conf |= IPU_CONF_DI1_EN;
	}
	if (ipu->dp_use_count > 0)
		ipu_conf |= IPU_CONF_DP_EN;
	if (ipu->dc_use_count > 0)
		ipu_conf |= IPU_CONF_DC_EN;
	if (ipu->dmfc_use_count > 0)
		ipu_conf |= IPU_CONF_DMFC_EN;
	if (ipu->ic_use_count > 0)
		ipu_conf |= IPU_CONF_IC_EN;
	if (ipu->vdi_use_count > 0) {
		ipu_conf |= IPU_CONF_ISP_EN;
		ipu_conf |= IPU_CONF_VDI_EN;
		ipu_conf |= IPU_CONF_IC_INPUT;
	}
	if (ipu->rot_use_count > 0)
		ipu_conf |= IPU_CONF_ROT_EN;
	if (ipu->smfc_use_count > 0)
		ipu_conf |= IPU_CONF_SMFC_EN;
	ipu_cm_write(ipu, ipu_conf, IPU_CONF);

	if (idma_is_valid(in_dma)) {
		reg = ipu_idmac_read(ipu, IDMAC_CHA_EN(in_dma));
		ipu_idmac_write(ipu, reg | idma_mask(in_dma), IDMAC_CHA_EN(in_dma));
	}
	if (idma_is_valid(out_dma)) {
		reg = ipu_idmac_read(ipu, IDMAC_CHA_EN(out_dma));
		ipu_idmac_write(ipu, reg | idma_mask(out_dma), IDMAC_CHA_EN(out_dma));
	}

	if ((ipu->sec_chan_en[IPU_CHAN_ID(channel)]) &&
		((channel == MEM_PP_MEM) || (channel == MEM_PRP_VF_MEM) ||
		 (channel == MEM_VDI_PRP_VF_MEM))) {
		sec_dma = channel_2_dma(channel, IPU_GRAPH_IN_BUFFER);
		reg = ipu_idmac_read(ipu, IDMAC_CHA_EN(sec_dma));
		ipu_idmac_write(ipu, reg | idma_mask(sec_dma), IDMAC_CHA_EN(sec_dma));
	}
	if ((ipu->thrd_chan_en[IPU_CHAN_ID(channel)]) &&
		((channel == MEM_PP_MEM) || (channel == MEM_PRP_VF_MEM))) {
		thrd_dma = channel_2_dma(channel, IPU_ALPHA_IN_BUFFER);
		reg = ipu_idmac_read(ipu, IDMAC_CHA_EN(thrd_dma));
		ipu_idmac_write(ipu, reg | idma_mask(thrd_dma), IDMAC_CHA_EN(thrd_dma));

		sec_dma = channel_2_dma(channel, IPU_GRAPH_IN_BUFFER);
		reg = ipu_idmac_read(ipu, IDMAC_SEP_ALPHA);
		ipu_idmac_write(ipu, reg | idma_mask(sec_dma), IDMAC_SEP_ALPHA);
	} else if ((ipu->thrd_chan_en[IPU_CHAN_ID(channel)]) &&
		   ((channel == MEM_BG_SYNC) || (channel == MEM_FG_SYNC))) {
		thrd_dma = channel_2_dma(channel, IPU_ALPHA_IN_BUFFER);
		reg = ipu_idmac_read(ipu, IDMAC_CHA_EN(thrd_dma));
		ipu_idmac_write(ipu, reg | idma_mask(thrd_dma), IDMAC_CHA_EN(thrd_dma));
		reg = ipu_idmac_read(ipu, IDMAC_SEP_ALPHA);
		ipu_idmac_write(ipu, reg | idma_mask(in_dma), IDMAC_SEP_ALPHA);
	}

	if ((channel == MEM_DC_SYNC) || (channel == MEM_BG_SYNC) ||
	    (channel == MEM_FG_SYNC)) {
		reg = ipu_idmac_read(ipu, IDMAC_WM_EN(in_dma));
		ipu_idmac_write(ipu, reg | idma_mask(in_dma), IDMAC_WM_EN(in_dma));

		_ipu_dp_dc_enable(ipu, channel);
	}

	if (_ipu_is_ic_chan(in_dma) || _ipu_is_ic_chan(out_dma) ||
		_ipu_is_irt_chan(in_dma) || _ipu_is_irt_chan(out_dma))
		_ipu_ic_enable_task(ipu, channel);

	ipu->channel_enable_mask |= 1L << IPU_CHAN_ID(channel);

	_ipu_unlock(ipu);

	return 0;
}
EXPORT_SYMBOL(ipu_enable_channel);

/*!
 * This function check buffer ready for a logical channel.
 *
 * @param	ipu		ipu handler
 * @param       channel         Input parameter for the logical channel ID.
 *
 * @param       type            Input parameter which buffer to clear.
 *
 * @param       bufNum          Input parameter for which buffer number clear
 * 				ready state.
 *
 */
int32_t ipu_check_buffer_ready(struct ipu_soc *ipu, ipu_channel_t channel, ipu_buffer_t type,
		uint32_t bufNum)
{
	uint32_t dma_chan = channel_2_dma(channel, type);
	uint32_t reg;

	if (dma_chan == IDMA_CHAN_INVALID)
		return -EINVAL;

	if (bufNum == 0)
		reg = ipu_cm_read(ipu, IPU_CHA_BUF0_RDY(dma_chan));
	else if (bufNum == 1)
		reg = ipu_cm_read(ipu, IPU_CHA_BUF1_RDY(dma_chan));
	else
		reg = ipu_cm_read(ipu, IPU_CHA_BUF2_RDY(dma_chan));

	if (reg & idma_mask(dma_chan))
		return 1;
	else
		return 0;
}
EXPORT_SYMBOL(ipu_check_buffer_ready);

/*!
 * This function clear buffer ready for a logical channel.
 *
 * @param	ipu		ipu handler
 * @param       channel         Input parameter for the logical channel ID.
 *
 * @param       type            Input parameter which buffer to clear.
 *
 * @param       bufNum          Input parameter for which buffer number clear
 * 				ready state.
 *
 */
void _ipu_clear_buffer_ready(struct ipu_soc *ipu, ipu_channel_t channel, ipu_buffer_t type,
		uint32_t bufNum)
{
	uint32_t dma_ch = channel_2_dma(channel, type);

	if (!idma_is_valid(dma_ch))
		return;

	ipu_cm_write(ipu, 0xF0300000, IPU_GPR); /* write one to clear */
	if (bufNum == 0)
		ipu_cm_write(ipu, idma_mask(dma_ch),
				IPU_CHA_BUF0_RDY(dma_ch));
	else if (bufNum == 1)
		ipu_cm_write(ipu, idma_mask(dma_ch),
				IPU_CHA_BUF1_RDY(dma_ch));
	else
		ipu_cm_write(ipu, idma_mask(dma_ch),
				IPU_CHA_BUF2_RDY(dma_ch));
	ipu_cm_write(ipu, 0x0, IPU_GPR); /* write one to set */
}

void ipu_clear_buffer_ready(struct ipu_soc *ipu, ipu_channel_t channel, ipu_buffer_t type,
		uint32_t bufNum)
{
	_ipu_lock(ipu);
	_ipu_clear_buffer_ready(ipu, channel, type, bufNum);
	_ipu_unlock(ipu);
}
EXPORT_SYMBOL(ipu_clear_buffer_ready);

/*!
 * This function disables a logical channel.
 *
 * @param	ipu		ipu handler
 * @param       channel         Input parameter for the logical channel ID.
 *
 * @param       wait_for_stop   Flag to set whether to wait for channel end
 *                              of frame or return immediately.
 *
 * @return      This function returns 0 on success or negative error code on
 *              fail.
 */
int32_t ipu_disable_channel(struct ipu_soc *ipu, ipu_channel_t channel, bool wait_for_stop)
{
	uint32_t reg;
	uint32_t in_dma;
	uint32_t out_dma;
	uint32_t sec_dma = NO_DMA;
	uint32_t thrd_dma = NO_DMA;
	uint16_t fg_pos_x, fg_pos_y;

	_ipu_lock(ipu);

	if ((ipu->channel_enable_mask & (1L << IPU_CHAN_ID(channel))) == 0) {
		dev_err(ipu->dev, "Channel already disabled %d\n",
			IPU_CHAN_ID(channel));
		_ipu_unlock(ipu);
		return -EACCES;
	}

	/* Get input and output dma channels */
	out_dma = channel_2_dma(channel, IPU_OUTPUT_BUFFER);
	in_dma = channel_2_dma(channel, IPU_VIDEO_IN_BUFFER);

	if ((idma_is_valid(in_dma) &&
		!idma_is_set(ipu, IDMAC_CHA_EN, in_dma))
		&& (idma_is_valid(out_dma) &&
		!idma_is_set(ipu, IDMAC_CHA_EN, out_dma))) {
		_ipu_unlock(ipu);
		return -EINVAL;
	}

	if (ipu->sec_chan_en[IPU_CHAN_ID(channel)])
		sec_dma = channel_2_dma(channel, IPU_GRAPH_IN_BUFFER);
	if (ipu->thrd_chan_en[IPU_CHAN_ID(channel)]) {
		sec_dma = channel_2_dma(channel, IPU_GRAPH_IN_BUFFER);
		thrd_dma = channel_2_dma(channel, IPU_ALPHA_IN_BUFFER);
	}

	if ((channel == MEM_BG_SYNC) || (channel == MEM_FG_SYNC) ||
	    (channel == MEM_DC_SYNC)) {
		if (channel == MEM_FG_SYNC) {
			_ipu_disp_get_window_pos(ipu, channel, &fg_pos_x, &fg_pos_y);
			_ipu_disp_set_window_pos(ipu, channel, 0, 0);
		}

		_ipu_dp_dc_disable(ipu, channel, false);

		/*
		 * wait for BG channel EOF then disable FG-IDMAC,
		 * it avoid FG NFB4EOF error.
		 */
		if ((channel == MEM_FG_SYNC) && (ipu_is_channel_busy(ipu, MEM_BG_SYNC))) {
			int timeout = 50;

			ipu_cm_write(ipu, IPUIRQ_2_MASK(IPU_IRQ_BG_SYNC_EOF),
					IPUIRQ_2_STATREG(IPU_IRQ_BG_SYNC_EOF));
			while ((ipu_cm_read(ipu, IPUIRQ_2_STATREG(IPU_IRQ_BG_SYNC_EOF)) &
						IPUIRQ_2_MASK(IPU_IRQ_BG_SYNC_EOF)) == 0) {
				msleep(10);
				timeout -= 10;
				if (timeout <= 0) {
					dev_err(ipu->dev, "warning: wait for bg sync eof timeout\n");
					break;
				}
			}
		}
	} else if (wait_for_stop) {
		while (idma_is_set(ipu, IDMAC_CHA_BUSY, in_dma) ||
		       idma_is_set(ipu, IDMAC_CHA_BUSY, out_dma) ||
			(ipu->sec_chan_en[IPU_CHAN_ID(channel)] &&
			idma_is_set(ipu, IDMAC_CHA_BUSY, sec_dma)) ||
			(ipu->thrd_chan_en[IPU_CHAN_ID(channel)] &&
			idma_is_set(ipu, IDMAC_CHA_BUSY, thrd_dma))) {
			uint32_t irq = 0xffffffff;
			int timeout = 50;

			if (idma_is_set(ipu, IDMAC_CHA_BUSY, out_dma))
				irq = out_dma;
			if (ipu->sec_chan_en[IPU_CHAN_ID(channel)] &&
				idma_is_set(ipu, IDMAC_CHA_BUSY, sec_dma))
				irq = sec_dma;
			if (ipu->thrd_chan_en[IPU_CHAN_ID(channel)] &&
				idma_is_set(ipu, IDMAC_CHA_BUSY, thrd_dma))
				irq = thrd_dma;
			if (idma_is_set(ipu, IDMAC_CHA_BUSY, in_dma))
				irq = in_dma;

			if (irq == 0xffffffff) {
				dev_dbg(ipu->dev, "warning: no channel busy, break\n");
				break;
			}

			ipu_cm_write(ipu, IPUIRQ_2_MASK(irq),
					IPUIRQ_2_STATREG(irq));

			dev_dbg(ipu->dev, "warning: channel %d busy, need wait\n", irq);

			while (((ipu_cm_read(ipu, IPUIRQ_2_STATREG(irq))
				& IPUIRQ_2_MASK(irq)) == 0) &&
				(idma_is_set(ipu, IDMAC_CHA_BUSY, irq))) {
				msleep(10);
				timeout -= 10;
				if (timeout <= 0) {
					ipu_dump_registers(ipu);
					dev_err(ipu->dev, "warning: disable ipu dma channel %d during its busy state\n", irq);
					break;
				}
			}

		}
	}

	if ((channel == MEM_BG_SYNC) || (channel == MEM_FG_SYNC) ||
	    (channel == MEM_DC_SYNC)) {
		reg = ipu_idmac_read(ipu, IDMAC_WM_EN(in_dma));
		ipu_idmac_write(ipu, reg & ~idma_mask(in_dma), IDMAC_WM_EN(in_dma));
	}

	/* Disable IC task */
	if (_ipu_is_ic_chan(in_dma) || _ipu_is_ic_chan(out_dma) ||
		_ipu_is_irt_chan(in_dma) || _ipu_is_irt_chan(out_dma))
		_ipu_ic_disable_task(ipu, channel);

	/* Disable DMA channel(s) */
	if (idma_is_valid(in_dma)) {
		reg = ipu_idmac_read(ipu, IDMAC_CHA_EN(in_dma));
		ipu_idmac_write(ipu, reg & ~idma_mask(in_dma), IDMAC_CHA_EN(in_dma));
		ipu_cm_write(ipu, idma_mask(in_dma), IPU_CHA_CUR_BUF(in_dma));
		ipu_cm_write(ipu, tri_cur_buf_mask(in_dma),
					IPU_CHA_TRIPLE_CUR_BUF(in_dma));
	}
	if (idma_is_valid(out_dma)) {
		reg = ipu_idmac_read(ipu, IDMAC_CHA_EN(out_dma));
		ipu_idmac_write(ipu, reg & ~idma_mask(out_dma), IDMAC_CHA_EN(out_dma));
		ipu_cm_write(ipu, idma_mask(out_dma), IPU_CHA_CUR_BUF(out_dma));
		ipu_cm_write(ipu, tri_cur_buf_mask(out_dma),
					IPU_CHA_TRIPLE_CUR_BUF(out_dma));
	}
	if (ipu->sec_chan_en[IPU_CHAN_ID(channel)] && idma_is_valid(sec_dma)) {
		reg = ipu_idmac_read(ipu, IDMAC_CHA_EN(sec_dma));
		ipu_idmac_write(ipu, reg & ~idma_mask(sec_dma), IDMAC_CHA_EN(sec_dma));
		ipu_cm_write(ipu, idma_mask(sec_dma), IPU_CHA_CUR_BUF(sec_dma));
	}
	if (ipu->thrd_chan_en[IPU_CHAN_ID(channel)] && idma_is_valid(thrd_dma)) {
		reg = ipu_idmac_read(ipu, IDMAC_CHA_EN(thrd_dma));
		ipu_idmac_write(ipu, reg & ~idma_mask(thrd_dma), IDMAC_CHA_EN(thrd_dma));
		if (channel == MEM_BG_SYNC || channel == MEM_FG_SYNC) {
			reg = ipu_idmac_read(ipu, IDMAC_SEP_ALPHA);
			ipu_idmac_write(ipu, reg & ~idma_mask(in_dma), IDMAC_SEP_ALPHA);
		} else {
			reg = ipu_idmac_read(ipu, IDMAC_SEP_ALPHA);
			ipu_idmac_write(ipu, reg & ~idma_mask(sec_dma), IDMAC_SEP_ALPHA);
		}
		ipu_cm_write(ipu, idma_mask(thrd_dma), IPU_CHA_CUR_BUF(thrd_dma));
	}

	if (channel == MEM_FG_SYNC)
		_ipu_disp_set_window_pos(ipu, channel, fg_pos_x, fg_pos_y);

	/* Set channel buffers NOT to be ready */
	if (idma_is_valid(in_dma)) {
		_ipu_clear_buffer_ready(ipu, channel, IPU_VIDEO_IN_BUFFER, 0);
		_ipu_clear_buffer_ready(ipu, channel, IPU_VIDEO_IN_BUFFER, 1);
		_ipu_clear_buffer_ready(ipu, channel, IPU_VIDEO_IN_BUFFER, 2);
	}
	if (idma_is_valid(out_dma)) {
		_ipu_clear_buffer_ready(ipu, channel, IPU_OUTPUT_BUFFER, 0);
		_ipu_clear_buffer_ready(ipu, channel, IPU_OUTPUT_BUFFER, 1);
	}
	if (ipu->sec_chan_en[IPU_CHAN_ID(channel)] && idma_is_valid(sec_dma)) {
		_ipu_clear_buffer_ready(ipu, channel, IPU_GRAPH_IN_BUFFER, 0);
		_ipu_clear_buffer_ready(ipu, channel, IPU_GRAPH_IN_BUFFER, 1);
	}
	if (ipu->thrd_chan_en[IPU_CHAN_ID(channel)] && idma_is_valid(thrd_dma)) {
		_ipu_clear_buffer_ready(ipu, channel, IPU_ALPHA_IN_BUFFER, 0);
		_ipu_clear_buffer_ready(ipu, channel, IPU_ALPHA_IN_BUFFER, 1);
	}

	ipu->channel_enable_mask &= ~(1L << IPU_CHAN_ID(channel));

	_ipu_unlock(ipu);

	return 0;
}
EXPORT_SYMBOL(ipu_disable_channel);

/*!
 * This function enables CSI.
 *
 * @param	ipu		ipu handler
 * @param       csi	csi num 0 or 1
 *
 * @return      This function returns 0 on success or negative error code on
 *              fail.
 */
int32_t ipu_enable_csi(struct ipu_soc *ipu, uint32_t csi)
{
	uint32_t reg;

	if (csi > 1) {
		dev_err(ipu->dev, "Wrong csi num_%d\n", csi);
		return -EINVAL;
	}

	_ipu_lock(ipu);
	ipu->csi_use_count[csi]++;

	if (ipu->csi_use_count[csi] == 1) {
		reg = ipu_cm_read(ipu, IPU_CONF);
		if (csi == 0)
			ipu_cm_write(ipu, reg | IPU_CONF_CSI0_EN, IPU_CONF);
		else
			ipu_cm_write(ipu, reg | IPU_CONF_CSI1_EN, IPU_CONF);
	}
	_ipu_unlock(ipu);
	return 0;
}
EXPORT_SYMBOL(ipu_enable_csi);

/*!
 * This function disables CSI.
 *
 * @param	ipu		ipu handler
 * @param       csi	csi num 0 or 1
 *
 * @return      This function returns 0 on success or negative error code on
 *              fail.
 */
int32_t ipu_disable_csi(struct ipu_soc *ipu, uint32_t csi)
{
	uint32_t reg;

	if (csi > 1) {
		dev_err(ipu->dev, "Wrong csi num_%d\n", csi);
		return -EINVAL;
	}

	_ipu_lock(ipu);
	ipu->csi_use_count[csi]--;
	if (ipu->csi_use_count[csi] == 0) {
		reg = ipu_cm_read(ipu, IPU_CONF);
		if (csi == 0)
			ipu_cm_write(ipu, reg & ~IPU_CONF_CSI0_EN, IPU_CONF);
		else
			ipu_cm_write(ipu, reg & ~IPU_CONF_CSI1_EN, IPU_CONF);
	}
	_ipu_unlock(ipu);
	return 0;
}
EXPORT_SYMBOL(ipu_disable_csi);

static irqreturn_t ipu_irq_handler(int irq, void *desc)
{
	struct ipu_soc *ipu = desc;
	int i;
	uint32_t line;
	irqreturn_t result = IRQ_NONE;
	uint32_t int_stat;
	const int err_reg[] = { 5, 6, 9, 10, 0 };
	const int int_reg[] = { 1, 2, 3, 4, 11, 12, 13, 14, 15, 0 };
	unsigned long lock_flags;

	for (i = 0;; i++) {
		if (err_reg[i] == 0)
			break;

		spin_lock_irqsave(&ipu->spin_lock, lock_flags);

		int_stat = ipu_cm_read(ipu, IPU_INT_STAT(err_reg[i]));
		int_stat &= ipu_cm_read(ipu, IPU_INT_CTRL(err_reg[i]));
		if (int_stat) {
			ipu_cm_write(ipu, int_stat, IPU_INT_STAT(err_reg[i]));
			dev_err(ipu->dev,
				"IPU Error - IPU_INT_STAT_%d = 0x%08X\n",
				err_reg[i], int_stat);
			/* Disable interrupts so we only get error once */
			int_stat =
			    ipu_cm_read(ipu, IPU_INT_CTRL(err_reg[i])) & ~int_stat;
			ipu_cm_write(ipu, int_stat, IPU_INT_CTRL(err_reg[i]));
		}

		spin_unlock_irqrestore(&ipu->spin_lock, lock_flags);
	}

	for (i = 0;; i++) {
		if (int_reg[i] == 0)
			break;
		spin_lock_irqsave(&ipu->spin_lock, lock_flags);
		int_stat = ipu_cm_read(ipu, IPU_INT_STAT(int_reg[i]));
		int_stat &= ipu_cm_read(ipu, IPU_INT_CTRL(int_reg[i]));
		ipu_cm_write(ipu, int_stat, IPU_INT_STAT(int_reg[i]));
		spin_unlock_irqrestore(&ipu->spin_lock, lock_flags);
		while ((line = ffs(int_stat)) != 0) {
			line--;
			int_stat &= ~(1UL << line);
			line += (int_reg[i] - 1) * 32;
			result |=
			    ipu->irq_list[line].handler(line,
						       ipu->irq_list[line].
						       dev_id);
		}
	}

	return result;
}

/*!
 * This function enables the interrupt for the specified interrupt line.
 * The interrupt lines are defined in \b ipu_irq_line enum.
 *
 * @param	ipu		ipu handler
 * @param       irq             Interrupt line to enable interrupt for.
 *
 */
void ipu_enable_irq(struct ipu_soc *ipu, uint32_t irq)
{
	uint32_t reg;
	unsigned long lock_flags;

	_ipu_get(ipu);

	spin_lock_irqsave(&ipu->spin_lock, lock_flags);

	reg = ipu_cm_read(ipu, IPUIRQ_2_CTRLREG(irq));
	reg |= IPUIRQ_2_MASK(irq);
	ipu_cm_write(ipu, reg, IPUIRQ_2_CTRLREG(irq));

	spin_unlock_irqrestore(&ipu->spin_lock, lock_flags);

	_ipu_put(ipu);
}
EXPORT_SYMBOL(ipu_enable_irq);

/*!
 * This function disables the interrupt for the specified interrupt line.
 * The interrupt lines are defined in \b ipu_irq_line enum.
 *
 * @param	ipu		ipu handler
 * @param       irq             Interrupt line to disable interrupt for.
 *
 */
void ipu_disable_irq(struct ipu_soc *ipu, uint32_t irq)
{
	uint32_t reg;
	unsigned long lock_flags;

	_ipu_get(ipu);

	spin_lock_irqsave(&ipu->spin_lock, lock_flags);

	reg = ipu_cm_read(ipu, IPUIRQ_2_CTRLREG(irq));
	reg &= ~IPUIRQ_2_MASK(irq);
	ipu_cm_write(ipu, reg, IPUIRQ_2_CTRLREG(irq));

	spin_unlock_irqrestore(&ipu->spin_lock, lock_flags);

	_ipu_put(ipu);
}
EXPORT_SYMBOL(ipu_disable_irq);

/*!
 * This function clears the interrupt for the specified interrupt line.
 * The interrupt lines are defined in \b ipu_irq_line enum.
 *
 * @param	ipu		ipu handler
 * @param       irq             Interrupt line to clear interrupt for.
 *
 */
void ipu_clear_irq(struct ipu_soc *ipu, uint32_t irq)
{
	unsigned long lock_flags;

	_ipu_get(ipu);

	spin_lock_irqsave(&ipu->spin_lock, lock_flags);

	ipu_cm_write(ipu, IPUIRQ_2_MASK(irq), IPUIRQ_2_STATREG(irq));

	spin_unlock_irqrestore(&ipu->spin_lock, lock_flags);

	_ipu_put(ipu);
}
EXPORT_SYMBOL(ipu_clear_irq);

/*!
 * This function returns the current interrupt status for the specified
 * interrupt line. The interrupt lines are defined in \b ipu_irq_line enum.
 *
 * @param	ipu		ipu handler
 * @param       irq             Interrupt line to get status for.
 *
 * @return      Returns true if the interrupt is pending/asserted or false if
 *              the interrupt is not pending.
 */
bool ipu_get_irq_status(struct ipu_soc *ipu, uint32_t irq)
{
	uint32_t reg;

	_ipu_get(ipu);

	reg = ipu_cm_read(ipu, IPUIRQ_2_STATREG(irq));

	_ipu_put(ipu);

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
 * @param	ipu		ipu handler
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
int ipu_request_irq(struct ipu_soc *ipu, uint32_t irq,
		    irqreturn_t(*handler) (int, void *),
		    uint32_t irq_flags, const char *devname, void *dev_id)
{
	unsigned long lock_flags;

	BUG_ON(irq >= IPU_IRQ_COUNT);

	_ipu_get(ipu);

	spin_lock_irqsave(&ipu->spin_lock, lock_flags);

	if (ipu->irq_list[irq].handler != NULL) {
		dev_err(ipu->dev,
			"handler already installed on irq %d\n", irq);
		spin_unlock_irqrestore(&ipu->spin_lock, lock_flags);
		return -EINVAL;
	}

	ipu->irq_list[irq].handler = handler;
	ipu->irq_list[irq].flags = irq_flags;
	ipu->irq_list[irq].dev_id = dev_id;
	ipu->irq_list[irq].name = devname;

	/* clear irq stat for previous use */
	ipu_cm_write(ipu, IPUIRQ_2_MASK(irq), IPUIRQ_2_STATREG(irq));

	spin_unlock_irqrestore(&ipu->spin_lock, lock_flags);

	_ipu_put(ipu);

	ipu_enable_irq(ipu, irq);	/* enable the interrupt */

	return 0;
}
EXPORT_SYMBOL(ipu_request_irq);

/*!
 * This function unregisters an interrupt handler for the specified interrupt
 * line. The interrupt lines are defined in \b ipu_irq_line enum.
 *
 * @param	ipu		ipu handler
 * @param       irq             Interrupt line to get status for.
 *
 * @param       dev_id          Input parameter for pointer of data to be passed
 *                              to the handler. This must match value passed to
 *                              ipu_request_irq().
 *
 */
void ipu_free_irq(struct ipu_soc *ipu, uint32_t irq, void *dev_id)
{
	unsigned long lock_flags;

	ipu_disable_irq(ipu, irq);	/* disable the interrupt */

	spin_lock_irqsave(&ipu->spin_lock, lock_flags);
	if (ipu->irq_list[irq].dev_id == dev_id)
		ipu->irq_list[irq].handler = NULL;
	spin_unlock_irqrestore(&ipu->spin_lock, lock_flags);
}
EXPORT_SYMBOL(ipu_free_irq);

uint32_t ipu_get_cur_buffer_idx(struct ipu_soc *ipu, ipu_channel_t channel, ipu_buffer_t type)
{
	uint32_t reg, dma_chan;

	dma_chan = channel_2_dma(channel, type);
	if (!idma_is_valid(dma_chan))
		return -EINVAL;

	reg = ipu_cm_read(ipu, IPU_CHA_TRB_MODE_SEL(dma_chan));
	if ((reg & idma_mask(dma_chan)) && _ipu_is_trb_chan(dma_chan)) {
		reg = ipu_cm_read(ipu, IPU_CHA_TRIPLE_CUR_BUF(dma_chan));
		return (reg & tri_cur_buf_mask(dma_chan)) >>
				tri_cur_buf_shift(dma_chan);
	} else {
		reg = ipu_cm_read(ipu, IPU_CHA_CUR_BUF(dma_chan));
		if (reg & idma_mask(dma_chan))
			return 1;
		else
			return 0;
	}
}
EXPORT_SYMBOL(ipu_get_cur_buffer_idx);

uint32_t _ipu_channel_status(struct ipu_soc *ipu, ipu_channel_t channel)
{
	uint32_t stat = 0;
	uint32_t task_stat_reg = ipu_cm_read(ipu, IPU_PROC_TASK_STAT);

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

int32_t ipu_swap_channel(struct ipu_soc *ipu, ipu_channel_t from_ch, ipu_channel_t to_ch)
{
	uint32_t reg;

	int from_dma = channel_2_dma(from_ch, IPU_INPUT_BUFFER);
	int to_dma = channel_2_dma(to_ch, IPU_INPUT_BUFFER);

	_ipu_lock(ipu);

	/* enable target channel */
	reg = ipu_idmac_read(ipu, IDMAC_CHA_EN(to_dma));
	ipu_idmac_write(ipu, reg | idma_mask(to_dma), IDMAC_CHA_EN(to_dma));

	ipu->channel_enable_mask |= 1L << IPU_CHAN_ID(to_ch);

	/* switch dp dc */
	_ipu_dp_dc_disable(ipu, from_ch, true);

	/* disable source channel */
	reg = ipu_idmac_read(ipu, IDMAC_CHA_EN(from_dma));
	ipu_idmac_write(ipu, reg & ~idma_mask(from_dma), IDMAC_CHA_EN(from_dma));
	ipu_cm_write(ipu, idma_mask(from_dma), IPU_CHA_CUR_BUF(from_dma));
	ipu_cm_write(ipu, tri_cur_buf_mask(from_dma),
				IPU_CHA_TRIPLE_CUR_BUF(from_dma));

	ipu->channel_enable_mask &= ~(1L << IPU_CHAN_ID(from_ch));

	_ipu_clear_buffer_ready(ipu, from_ch, IPU_VIDEO_IN_BUFFER, 0);
	_ipu_clear_buffer_ready(ipu, from_ch, IPU_VIDEO_IN_BUFFER, 1);
	_ipu_clear_buffer_ready(ipu, from_ch, IPU_VIDEO_IN_BUFFER, 2);

	_ipu_unlock(ipu);

	return 0;
}
EXPORT_SYMBOL(ipu_swap_channel);

uint32_t bytes_per_pixel(uint32_t fmt)
{
	switch (fmt) {
	case IPU_PIX_FMT_GENERIC:	/*generic data */
	case IPU_PIX_FMT_RGB332:
	case IPU_PIX_FMT_YUV420P:
	case IPU_PIX_FMT_YVU420P:
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

static int ipu_suspend_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct imx_ipuv3_platform_data *plat_data = pdev->dev.platform_data;
	struct ipu_soc *ipu = platform_get_drvdata(pdev);

	if (atomic_read(&ipu->ipu_use_count)) {
		/* save and disable enabled channels*/
		ipu->idma_enable_reg[0] = ipu_idmac_read(ipu, IDMAC_CHA_EN(0));
		ipu->idma_enable_reg[1] = ipu_idmac_read(ipu, IDMAC_CHA_EN(32));
		while ((ipu_idmac_read(ipu, IDMAC_CHA_BUSY(0))
			& ipu->idma_enable_reg[0])
			|| (ipu_idmac_read(ipu, IDMAC_CHA_BUSY(32))
			& ipu->idma_enable_reg[1])) {
			/* disable channel not busy already */
			uint32_t chan_should_disable, timeout = 1000, time = 0;

			chan_should_disable =
				ipu_idmac_read(ipu, IDMAC_CHA_BUSY(0))
					^ ipu->idma_enable_reg[0];
			ipu_idmac_write(ipu, (~chan_should_disable) &
					ipu->idma_enable_reg[0], IDMAC_CHA_EN(0));
			chan_should_disable =
				ipu_idmac_read(ipu, IDMAC_CHA_BUSY(1))
					^ ipu->idma_enable_reg[1];
			ipu_idmac_write(ipu, (~chan_should_disable) &
					ipu->idma_enable_reg[1], IDMAC_CHA_EN(32));
			msleep(2);
			time += 2;
			if (time >= timeout)
				return -1;
		}
		ipu_idmac_write(ipu, 0, IDMAC_CHA_EN(0));
		ipu_idmac_write(ipu, 0, IDMAC_CHA_EN(32));

		/* save double buffer select regs */
		ipu->cha_db_mode_reg[0] = ipu_cm_read(ipu, IPU_CHA_DB_MODE_SEL(0));
		ipu->cha_db_mode_reg[1] = ipu_cm_read(ipu, IPU_CHA_DB_MODE_SEL(32));
		ipu->cha_db_mode_reg[2] =
			ipu_cm_read(ipu, IPU_ALT_CHA_DB_MODE_SEL(0));
		ipu->cha_db_mode_reg[3] =
			ipu_cm_read(ipu, IPU_ALT_CHA_DB_MODE_SEL(32));

		/* save triple buffer select regs */
		ipu->cha_trb_mode_reg[0] = ipu_cm_read(ipu, IPU_CHA_TRB_MODE_SEL(0));
		ipu->cha_trb_mode_reg[1] = ipu_cm_read(ipu, IPU_CHA_TRB_MODE_SEL(32));

		/* save idamc sub addr regs */
		ipu->idma_sub_addr_reg[0] = ipu_idmac_read(ipu, IDMAC_SUB_ADDR_0);
		ipu->idma_sub_addr_reg[1] = ipu_idmac_read(ipu, IDMAC_SUB_ADDR_1);
		ipu->idma_sub_addr_reg[2] = ipu_idmac_read(ipu, IDMAC_SUB_ADDR_2);
		ipu->idma_sub_addr_reg[3] = ipu_idmac_read(ipu, IDMAC_SUB_ADDR_3);
		ipu->idma_sub_addr_reg[4] = ipu_idmac_read(ipu, IDMAC_SUB_ADDR_4);

		/* save sub-modules status and disable all */
		ipu->ic_conf_reg = ipu_ic_read(ipu, IC_CONF);
		ipu_ic_write(ipu, 0, IC_CONF);
		ipu->ipu_conf_reg = ipu_cm_read(ipu, IPU_CONF);
		ipu_cm_write(ipu, 0, IPU_CONF);

		/* save buf ready regs */
		ipu->buf_ready_reg[0] = ipu_cm_read(ipu, IPU_CHA_BUF0_RDY(0));
		ipu->buf_ready_reg[1] = ipu_cm_read(ipu, IPU_CHA_BUF0_RDY(32));
		ipu->buf_ready_reg[2] = ipu_cm_read(ipu, IPU_CHA_BUF1_RDY(0));
		ipu->buf_ready_reg[3] = ipu_cm_read(ipu, IPU_CHA_BUF1_RDY(32));
		ipu->buf_ready_reg[4] = ipu_cm_read(ipu, IPU_ALT_CHA_BUF0_RDY(0));
		ipu->buf_ready_reg[5] = ipu_cm_read(ipu, IPU_ALT_CHA_BUF0_RDY(32));
		ipu->buf_ready_reg[6] = ipu_cm_read(ipu, IPU_ALT_CHA_BUF1_RDY(0));
		ipu->buf_ready_reg[7] = ipu_cm_read(ipu, IPU_ALT_CHA_BUF1_RDY(32));
		ipu->buf_ready_reg[8] = ipu_cm_read(ipu, IPU_CHA_BUF2_RDY(0));
		ipu->buf_ready_reg[9] = ipu_cm_read(ipu, IPU_CHA_BUF2_RDY(32));

		clk_disable(ipu->ipu_clk);
	}

	if (plat_data->pg)
		plat_data->pg(1);

	return 0;
}

static int ipu_resume_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct imx_ipuv3_platform_data *plat_data = pdev->dev.platform_data;
	struct ipu_soc *ipu = platform_get_drvdata(pdev);

	if (plat_data->pg)
		plat_data->pg(0);

	if (atomic_read(&ipu->ipu_use_count)) {
		clk_enable(ipu->ipu_clk);

		/* restore buf ready regs */
		ipu_cm_write(ipu, ipu->buf_ready_reg[0], IPU_CHA_BUF0_RDY(0));
		ipu_cm_write(ipu, ipu->buf_ready_reg[1], IPU_CHA_BUF0_RDY(32));
		ipu_cm_write(ipu, ipu->buf_ready_reg[2], IPU_CHA_BUF1_RDY(0));
		ipu_cm_write(ipu, ipu->buf_ready_reg[3], IPU_CHA_BUF1_RDY(32));
		ipu_cm_write(ipu, ipu->buf_ready_reg[4], IPU_ALT_CHA_BUF0_RDY(0));
		ipu_cm_write(ipu, ipu->buf_ready_reg[5], IPU_ALT_CHA_BUF0_RDY(32));
		ipu_cm_write(ipu, ipu->buf_ready_reg[6], IPU_ALT_CHA_BUF1_RDY(0));
		ipu_cm_write(ipu, ipu->buf_ready_reg[7], IPU_ALT_CHA_BUF1_RDY(32));
		ipu_cm_write(ipu, ipu->buf_ready_reg[8], IPU_CHA_BUF2_RDY(0));
		ipu_cm_write(ipu, ipu->buf_ready_reg[9], IPU_CHA_BUF2_RDY(32));

		/* re-enable sub-modules*/
		ipu_cm_write(ipu, ipu->ipu_conf_reg, IPU_CONF);
		ipu_ic_write(ipu, ipu->ic_conf_reg, IC_CONF);

		/* restore double buffer select regs */
		ipu_cm_write(ipu, ipu->cha_db_mode_reg[0], IPU_CHA_DB_MODE_SEL(0));
		ipu_cm_write(ipu, ipu->cha_db_mode_reg[1], IPU_CHA_DB_MODE_SEL(32));
		ipu_cm_write(ipu, ipu->cha_db_mode_reg[2],
				IPU_ALT_CHA_DB_MODE_SEL(0));
		ipu_cm_write(ipu, ipu->cha_db_mode_reg[3],
				IPU_ALT_CHA_DB_MODE_SEL(32));

		/* restore triple buffer select regs */
		ipu_cm_write(ipu, ipu->cha_trb_mode_reg[0], IPU_CHA_TRB_MODE_SEL(0));
		ipu_cm_write(ipu, ipu->cha_trb_mode_reg[1], IPU_CHA_TRB_MODE_SEL(32));

		/* restore idamc sub addr regs */
		ipu_idmac_write(ipu, ipu->idma_sub_addr_reg[0], IDMAC_SUB_ADDR_0);
		ipu_idmac_write(ipu, ipu->idma_sub_addr_reg[1], IDMAC_SUB_ADDR_1);
		ipu_idmac_write(ipu, ipu->idma_sub_addr_reg[2], IDMAC_SUB_ADDR_2);
		ipu_idmac_write(ipu, ipu->idma_sub_addr_reg[3], IDMAC_SUB_ADDR_3);
		ipu_idmac_write(ipu, ipu->idma_sub_addr_reg[4], IDMAC_SUB_ADDR_4);

		/* restart idma channel*/
		ipu_idmac_write(ipu, ipu->idma_enable_reg[0], IDMAC_CHA_EN(0));
		ipu_idmac_write(ipu, ipu->idma_enable_reg[1], IDMAC_CHA_EN(32));
	} else {
		_ipu_get(ipu);
		_ipu_dmfc_init(ipu, dmfc_type_setup, 1);
		_ipu_init_dc_mappings(ipu);
		/* Set sync refresh channels as high priority */
		ipu_idmac_write(ipu, 0x18800001L, IDMAC_CHA_PRI(0));
		_ipu_put(ipu);
	}

	return 0;
}

static const struct dev_pm_ops mxcipu_pm_ops = {
	.suspend_noirq = ipu_suspend_noirq,
	.resume_noirq = ipu_resume_noirq,
};

/*!
 * This structure contains pointers to the power management callback functions.
 */
static struct platform_driver mxcipu_driver = {
	.driver = {
		   .name = "imx-ipuv3",
		   .pm = &mxcipu_pm_ops,
		   },
	.probe = ipu_probe,
	.remove = ipu_remove,
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
