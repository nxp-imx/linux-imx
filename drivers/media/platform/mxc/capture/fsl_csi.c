/*
 * Copyright 2009-2014 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file fsl_csi.c, this file is derived from mx27_csi.c
 *
 * @brief mx25 CMOS Sensor interface functions
 *
 * @ingroup CSI
 */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/sched.h>

#include "mxc_v4l2_capture.h"
#include "fsl_csi.h"

#define CSI_MAX_NUM	2
struct csi_soc csi_array[CSI_MAX_NUM], *csi;

static csi_irq_callback_t g_callback;
static void *g_callback_data;

static irqreturn_t csi_irq_handler(int irq, void *data)
{
	cam_data *cam = (cam_data *) data;
	struct csi_soc *csi = &csi_array[cam->csi];
	unsigned long status = __raw_readl(csi->regbase + CSI_CSISR);
	u32 cr3, cr18;

	__raw_writel(status, csi->regbase + CSI_CSISR);

	if (status & BIT_HRESP_ERR_INT)
		pr_warning("Hresponse error is detected.\n");

	if (status & BIT_ADDR_CH_ERR_INT) {
		/* Disable csi  */
		cr18 = __raw_readl(csi->regbase + CSI_CSICR18);
		cr18 &= ~BIT_CSI_ENABLE;
		__raw_writel(cr18, csi->regbase + CSI_CSICR18);

		/* DMA reflash */
		cr3 = __raw_readl(csi->regbase + CSI_CSICR3);
		cr3 |= BIT_DMA_REFLASH_RFF;
		__raw_writel(cr3, csi->regbase + CSI_CSICR3);

		/* Ensable csi  */
		cr18 |= BIT_CSI_ENABLE;
		__raw_writel(cr18, csi->regbase + CSI_CSICR18);

		pr_debug("base address switching Change Err.\n");
	}

	if ((status & BIT_DMA_TSF_DONE_FB1) &&
		(status & BIT_DMA_TSF_DONE_FB2)) {
		/* For both FB1 and FB2 interrupter bits set case,
		 * CSI DMA is work in one of FB1 and FB2 buffer,
		 * but software can not know the state.
		 * Skip it to avoid base address updated
		 * when csi work in field0 and field1 will write to
		 * new base address.
		 * PDM TKT230775 */
		pr_debug("Skip two frames\n");
	} else if (status & BIT_DMA_TSF_DONE_FB1) {
		if (cam->capture_on) {
			spin_lock(&cam->queue_int_lock);
			cam->ping_pong_csi = 1;
			spin_unlock(&cam->queue_int_lock);
			cam->enc_callback(0, cam);
		} else {
			cam->still_counter++;
			wake_up_interruptible(&cam->still_queue);
		}
	} else if (status & BIT_DMA_TSF_DONE_FB2) {
		if (cam->capture_on) {
			spin_lock(&cam->queue_int_lock);
			cam->ping_pong_csi = 2;
			spin_unlock(&cam->queue_int_lock);
			cam->enc_callback(0, cam);
		} else {
			cam->still_counter++;
			wake_up_interruptible(&cam->still_queue);
		}
	}

	if (g_callback)
		g_callback(g_callback_data, status);

	pr_debug("CSI status = 0x%08lX\n", status);

	return IRQ_HANDLED;
}

static void csihw_reset_frame_count(struct csi_soc *csi)
{
	__raw_writel(__raw_readl(csi->regbase + CSI_CSICR3) | BIT_FRMCNT_RST,
			csi->regbase + CSI_CSICR3);
}

static void csihw_reset(struct csi_soc *csi)
{
	csihw_reset_frame_count(csi);
	__raw_writel(CSICR1_RESET_VAL, csi->regbase + CSI_CSICR1);
	__raw_writel(CSICR2_RESET_VAL, csi->regbase + CSI_CSICR2);
	__raw_writel(CSICR3_RESET_VAL, csi->regbase + CSI_CSICR3);
}

static void csisw_reset(struct csi_soc *csi)
{
	int cr1, cr3, cr18, isr;

	/* Disable csi */
	cr18 = __raw_readl(csi->regbase + CSI_CSICR18);
	cr18 &= ~BIT_CSI_ENABLE;
	__raw_writel(cr18, csi->regbase + CSI_CSICR18);

	/* Clear RX FIFO */
	cr1 = __raw_readl(csi->regbase + CSI_CSICR1);
	__raw_writel(cr1 & ~BIT_FCC, csi->regbase + CSI_CSICR1);
	cr1 = __raw_readl(csi->regbase + CSI_CSICR1);
	__raw_writel(cr1 | BIT_CLR_RXFIFO, csi->regbase + CSI_CSICR1);

	/* DMA reflash */
	cr3 = __raw_readl(csi->regbase + CSI_CSICR3);
	cr3 |= BIT_DMA_REFLASH_RFF | BIT_FRMCNT_RST;
	__raw_writel(cr3, csi->regbase + CSI_CSICR3);

	msleep(2);

	cr1 = __raw_readl(csi->regbase + CSI_CSICR1);
	__raw_writel(cr1 | BIT_FCC, csi->regbase + CSI_CSICR1);

	isr = __raw_readl(csi->regbase + CSI_CSISR);
	__raw_writel(isr, csi->regbase + CSI_CSISR);

	/* Ensable csi  */
	cr18 |= BIT_CSI_ENABLE;
	__raw_writel(cr18, csi->regbase + CSI_CSICR18);
}

/*!
 * csi_init_interface
 *    Init csi interface
 */
static void csi_init_interface(struct csi_soc *csi)
{
	unsigned int val = 0;
	unsigned int imag_para;

	val |= BIT_SOF_POL;
	val |= BIT_REDGE;
	val |= BIT_GCLK_MODE;
	val |= BIT_HSYNC_POL;
	val |= BIT_FCC;
	val |= 1 << SHIFT_MCLKDIV;
	val |= BIT_MCLKEN;
	__raw_writel(val, csi->regbase + CSI_CSICR1);

	imag_para = (640 << 16) | 960;
	__raw_writel(imag_para, csi->regbase + CSI_CSIIMAG_PARA);

	val = 0x1010;
	val |= BIT_DMA_REFLASH_RFF;
	__raw_writel(val, csi->regbase + CSI_CSICR3);
}

void csi_format_swap16(cam_data *cam, bool enable)
{
	struct csi_soc *csi = &csi_array[cam->csi];
	unsigned int val;

	val = __raw_readl(csi->regbase + CSI_CSICR1);
	if (enable) {
		val |= BIT_PACK_DIR;
		val |= BIT_SWAP16_EN;
	} else {
		val &= ~BIT_PACK_DIR;
		val &= ~BIT_SWAP16_EN;
	}

	__raw_writel(val, csi->regbase + CSI_CSICR1);
}
EXPORT_SYMBOL(csi_format_swap16);

/*!
 * csi_read_mclk_flag
 *
 * @return  gcsi_mclk_source
 */
int csi_read_mclk_flag(void)
{
	return 0;
}
EXPORT_SYMBOL(csi_read_mclk_flag);

void csi_start_callback(void *data)
{
	cam_data *cam = (cam_data *) data;

	if (request_irq(csi_array[cam->csi].irq_nr, csi_irq_handler, 0, "csi",
			cam) < 0)
		pr_debug("CSI error: irq request fail\n");

}
EXPORT_SYMBOL(csi_start_callback);

void csi_stop_callback(void *data)
{
	cam_data *cam = (cam_data *) data;

	free_irq(csi_array[cam->csi].irq_nr, cam);
}
EXPORT_SYMBOL(csi_stop_callback);

void csi_enable_int(cam_data *cam, int arg)
{
	struct csi_soc *csi = &csi_array[cam->csi];
	unsigned long cr1 = __raw_readl(csi->regbase + CSI_CSICR1);

	cr1 |= BIT_SOF_INTEN;
	if (arg == 1) {
		/* still capture needs DMA intterrupt */
		cr1 |= BIT_FB1_DMA_DONE_INTEN;
		cr1 |= BIT_FB2_DMA_DONE_INTEN;
	}
	__raw_writel(cr1, csi->regbase + CSI_CSICR1);
}
EXPORT_SYMBOL(csi_enable_int);

void csi_disable_int(cam_data *cam)
{
	struct csi_soc *csi = &csi_array[cam->csi];
	unsigned long cr1 = __raw_readl(csi->regbase + CSI_CSICR1);

	cr1 &= ~BIT_SOF_INTEN;
	cr1 &= ~BIT_FB1_DMA_DONE_INTEN;
	cr1 &= ~BIT_FB2_DMA_DONE_INTEN;
	__raw_writel(cr1, csi->regbase + CSI_CSICR1);
}
EXPORT_SYMBOL(csi_disable_int);

void csi_enable(cam_data *cam, int arg)
{
	struct csi_soc *csi = &csi_array[cam->csi];
	unsigned long cr = __raw_readl(csi->regbase + CSI_CSICR18);

	if (arg == 1) {
		csisw_reset(csi);
		cr |= BIT_CSI_ENABLE;
	} else
		cr &= ~BIT_CSI_ENABLE;
	__raw_writel(cr, csi->regbase + CSI_CSICR18);
}
EXPORT_SYMBOL(csi_enable);

void csi_buf_stride_set(cam_data *cam, u32 stride)
{
	struct csi_soc *csi = &csi_array[cam->csi];

	__raw_writel(stride, csi->regbase + CSI_CSIFBUF_PARA);
}
EXPORT_SYMBOL(csi_buf_stride_set);

void csi_deinterlace_enable(cam_data *cam, bool enable)
{
	struct csi_soc *csi = &csi_array[cam->csi];
	unsigned long cr18 = __raw_readl(csi->regbase + CSI_CSICR18);

	if (enable == true)
		cr18 |= BIT_DEINTERLACE_EN;
	else
		cr18 &= ~BIT_DEINTERLACE_EN;

	__raw_writel(cr18, csi->regbase + CSI_CSICR18);
}
EXPORT_SYMBOL(csi_deinterlace_enable);

void csi_deinterlace_mode(cam_data *cam, int mode)
{
	struct csi_soc *csi = &csi_array[cam->csi];
	unsigned long cr18 = __raw_readl(csi->regbase + CSI_CSICR18);

	if (mode == V4L2_STD_NTSC)
		cr18 |= BIT_NTSC_EN;
	else
		cr18 &= ~BIT_NTSC_EN;

	__raw_writel(cr18, csi->regbase + CSI_CSICR18);
}
EXPORT_SYMBOL(csi_deinterlace_mode);

void csi_tvdec_enable(cam_data *cam, bool enable)
{
	struct csi_soc *csi = &csi_array[cam->csi];
	unsigned long cr18 = __raw_readl(csi->regbase + CSI_CSICR18);
	unsigned long cr1 = __raw_readl(csi->regbase + CSI_CSICR1);

	if (enable == true) {
		cr18 |= (BIT_TVDECODER_IN_EN |
				BIT_BASEADDR_SWITCH_EN |
				BIT_BASEADDR_SWITCH_SEL |
				BIT_BASEADDR_CHG_ERR_EN);
		cr1 |= BIT_CCIR_MODE;
		cr1 &= ~(BIT_SOF_POL | BIT_REDGE);
	} else {
		cr18 &= ~(BIT_TVDECODER_IN_EN |
				BIT_BASEADDR_SWITCH_EN |
				BIT_BASEADDR_SWITCH_SEL |
				BIT_BASEADDR_CHG_ERR_EN);
		cr1 &= ~BIT_CCIR_MODE;
		cr1 |= BIT_SOF_POL | BIT_REDGE;
	}

	__raw_writel(cr18, csi->regbase + CSI_CSICR18);
	__raw_writel(cr1, csi->regbase + CSI_CSICR1);
}
EXPORT_SYMBOL(csi_tvdec_enable);

void csi_set_32bit_imagpara(cam_data *cam, int width, int height)
{
	struct csi_soc *csi = &csi_array[cam->csi];
	int imag_para = 0;
	unsigned long cr3 = __raw_readl(csi->regbase + CSI_CSICR3);

	imag_para = (width << 16) | height;
	__raw_writel(imag_para, csi->regbase + CSI_CSIIMAG_PARA);


	/* reflash the embeded DMA controller */
	__raw_writel(cr3 | BIT_DMA_REFLASH_RFF, csi->regbase + CSI_CSICR3);
}
EXPORT_SYMBOL(csi_set_32bit_imagpara);

void csi_set_16bit_imagpara(cam_data *cam, int width, int height)
{
	struct csi_soc *csi = &csi_array[cam->csi];
	int imag_para = 0;
	unsigned long cr3 = __raw_readl(csi->regbase + CSI_CSICR3);

	imag_para = (width << 16) | (height * 2);
	__raw_writel(imag_para, csi->regbase + CSI_CSIIMAG_PARA);

	/* reflash the embeded DMA controller */
	__raw_writel(cr3 | BIT_DMA_REFLASH_RFF, csi->regbase + CSI_CSICR3);
}
EXPORT_SYMBOL(csi_set_16bit_imagpara);

void csi_set_12bit_imagpara(cam_data *cam, int width, int height)
{
	struct csi_soc *csi = &csi_array[cam->csi];
	int imag_para = 0;
	unsigned long cr3 = __raw_readl(csi->regbase + CSI_CSICR3);

	imag_para = (width << 16) | (height * 3 / 2);
	__raw_writel(imag_para, csi->regbase + CSI_CSIIMAG_PARA);

	/* reflash the embeded DMA controller */
	__raw_writel(cr3 | BIT_DMA_REFLASH_RFF, csi->regbase + CSI_CSICR3);
}
EXPORT_SYMBOL(csi_set_12bit_imagpara);

void csi_dmareq_rff_enable(struct csi_soc *csi)
{
	unsigned long cr3 = __raw_readl(csi->regbase + CSI_CSICR3);
	unsigned long cr2 = __raw_readl(csi->regbase + CSI_CSICR2);

	/* Burst Type of DMA Transfer from RxFIFO. INCR16 */
	cr2 |= 0xC0000000;

	cr3 |= BIT_DMA_REQ_EN_RFF;
	cr3 |= BIT_HRESP_ERR_EN;
	cr3 &= ~BIT_RXFF_LEVEL;
	cr3 |= 0x2 << 4;

	__raw_writel(cr3, csi->regbase + CSI_CSICR3);
	__raw_writel(cr2, csi->regbase + CSI_CSICR2);
}
EXPORT_SYMBOL(csi_dmareq_rff_enable);

void csi_dmareq_rff_disable(struct csi_soc *csi)
{
	unsigned long cr3 = __raw_readl(csi->regbase + CSI_CSICR3);

	cr3 &= ~BIT_DMA_REQ_EN_RFF;
	cr3 &= ~BIT_HRESP_ERR_EN;
	__raw_writel(cr3, csi->regbase + CSI_CSICR3);
}
EXPORT_SYMBOL(csi_dmareq_rff_disable);

struct csi_soc *csi_get_soc(int id)
{
	if (id >= CSI_MAX_NUM)
		return ERR_PTR(-ENODEV);
	else if (!csi_array[id].online)
		return ERR_PTR(-ENODEV);
	else
		return &(csi_array[id]);
}
EXPORT_SYMBOL_GPL(csi_get_soc);

static const struct of_device_id fsl_csi_dt_ids[] = {
	{ .compatible = "fsl,imx6sl-csi", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fsl_csi_dt_ids);

static int csi_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;
	int id;

	id = of_alias_get_id(pdev->dev.of_node, "csi");
	if (id < 0) {
		dev_dbg(&pdev->dev, "can not get alias id\n");
		return id;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "No csi irq found.\n");
		ret = -ENODEV;
		goto err;
	}

	csi = &csi_array[id];
	csi->irq_nr = res->start;
	csi->online = false;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "No csi base address found.\n");
		ret = -ENODEV;
		goto err;
	}
	csi->regbase = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!csi->regbase) {
		dev_err(&pdev->dev, "ioremap failed with csi base\n");
		ret = -ENOMEM;
		goto err;
	}

	csi->disp_axi_clk = devm_clk_get(&pdev->dev, "disp-axi");
	if (IS_ERR(csi->disp_axi_clk)) {
		dev_err(&pdev->dev, "get csi clock failed\n");
		return PTR_ERR(csi->disp_axi_clk);
	}
	csi->csi_clk = devm_clk_get(&pdev->dev, "csi_mclk");
	if (IS_ERR(csi->csi_clk)) {
		dev_err(&pdev->dev, "get csi mclk failed\n");
		return PTR_ERR(csi->csi_clk);
	}

	csi->dcic_clk = devm_clk_get(&pdev->dev, "dcic");
	if (IS_ERR(csi->dcic_clk)) {
		dev_err(&pdev->dev, "get dcic clk failed\n");
		return PTR_ERR(csi->dcic_clk);
	}

	csi->disp_reg = devm_regulator_get(&pdev->dev, "disp");
	if (IS_ERR(csi->disp_reg)) {
		dev_dbg(&pdev->dev, "display regulator is not ready\n");
		csi->disp_reg = NULL;
	}

	platform_set_drvdata(pdev, csi);

	if (csi->disp_reg)
		ret = regulator_enable(csi->disp_reg);

	clk_prepare_enable(csi->disp_axi_clk);
	clk_prepare_enable(csi->dcic_clk);
	clk_prepare_enable(csi->csi_clk);

	csihw_reset(csi);
	csi_init_interface(csi);
	csi_dmareq_rff_disable(csi);

	csi->online = true;
err:
	return ret;
}

static int csi_remove(struct platform_device *pdev)
{
	struct csi_soc *csi = platform_get_drvdata(pdev);

	csi->online = false;

	clk_disable_unprepare(csi->csi_clk);
	clk_disable_unprepare(csi->dcic_clk);
	clk_disable_unprepare(csi->disp_axi_clk);

	if (csi->disp_reg)
		regulator_disable(csi->disp_reg);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int csi_suspend(struct device *dev)
{
	struct csi_soc *csi = dev_get_drvdata(dev);

	csi->online = false;

	return 0;
}

static int csi_resume(struct device *dev)
{
	struct csi_soc *csi = dev_get_drvdata(dev);

	csihw_reset(csi);
	csi_init_interface(csi);
	csi_dmareq_rff_disable(csi);

	csi->online = true;

	return 0;
}
#else
#define	csi_suspend	NULL
#define	csi_resume	NULL
#endif

static const struct dev_pm_ops csi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(csi_suspend, csi_resume)
};

static struct platform_driver csi_driver = {
	.driver = {
		   .name = "fsl_csi",
		   .of_match_table = of_match_ptr(fsl_csi_dt_ids),
		   .pm = &csi_pm_ops,
		   },
	.probe = csi_probe,
	.remove = csi_remove,
};

module_platform_driver(csi_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("fsl CSI driver");
MODULE_LICENSE("GPL");
