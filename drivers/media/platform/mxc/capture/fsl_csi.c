/*
 * Copyright 2009-2013 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/sched.h>

#include "mxc_v4l2_capture.h"
#include "fsl_csi.h"

void __iomem *csi_regbase;
static int irq_nr;
static csi_irq_callback_t g_callback;
static void *g_callback_data;

static irqreturn_t csi_irq_handler(int irq, void *data)
{
	cam_data *cam = (cam_data *) data;
	unsigned long status = __raw_readl(CSI_CSISR);

	__raw_writel(status, CSI_CSISR);

	if (status & BIT_HRESP_ERR_INT)
		pr_warning("Hresponse error is detected.\n");

	if (status & BIT_DMA_TSF_DONE_FB1) {
		if (cam->capture_on) {
			spin_lock(&cam->queue_int_lock);
			cam->ping_pong_csi = 1;
			spin_unlock(&cam->queue_int_lock);
			cam->enc_callback(0, cam);
		} else {
			cam->still_counter++;
			wake_up_interruptible(&cam->still_queue);
		}
	}

	if (status & BIT_DMA_TSF_DONE_FB2) {
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

static void csihw_reset_frame_count(void)
{
	__raw_writel(__raw_readl(CSI_CSICR3) | BIT_FRMCNT_RST, CSI_CSICR3);
}

static void csihw_reset(void)
{
	csihw_reset_frame_count();
	__raw_writel(CSICR1_RESET_VAL, CSI_CSICR1);
	__raw_writel(CSICR2_RESET_VAL, CSI_CSICR2);
	__raw_writel(CSICR3_RESET_VAL, CSI_CSICR3);
}

/*!
 * csi_init_interface
 *    Init csi interface
 */
void csi_init_interface(void)
{
	unsigned int val = 0;
	unsigned int imag_para;

	val |= BIT_SOF_POL;
	val |= BIT_REDGE;
	val |= BIT_GCLK_MODE;
	val |= BIT_HSYNC_POL;
	val |= BIT_PACK_DIR;
	val |= BIT_FCC;
	val |= BIT_SWAP16_EN;
	val |= 1 << SHIFT_MCLKDIV;
	val |= BIT_MCLKEN;
	__raw_writel(val, CSI_CSICR1);

	imag_para = (640 << 16) | 960;
	__raw_writel(imag_para, CSI_CSIIMAG_PARA);

	val = 0x1010;
	val |= BIT_DMA_REFLASH_RFF;
	__raw_writel(val, CSI_CSICR3);
}
EXPORT_SYMBOL(csi_init_interface);

void csi_init_format(int fmt)
{
	unsigned int val;

	val = __raw_readl(CSI_CSICR1);
	if (fmt == V4L2_PIX_FMT_YUYV) {
		val &= ~BIT_PACK_DIR;
		val &= ~BIT_SWAP16_EN;
	} else if (fmt == V4L2_PIX_FMT_UYVY) {
		val |= BIT_PACK_DIR;
		val |= BIT_SWAP16_EN;
	} else
		pr_warning("unsupported format, old format remains.\n");

	__raw_writel(val, CSI_CSICR1);
}
EXPORT_SYMBOL(csi_init_format);

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

	if (request_irq(irq_nr, csi_irq_handler, 0, "csi", cam) < 0)
		pr_debug("CSI error: irq request fail\n");

}
EXPORT_SYMBOL(csi_start_callback);

void csi_stop_callback(void *data)
{
	cam_data *cam = (cam_data *) data;

	free_irq(irq_nr, cam);
}
EXPORT_SYMBOL(csi_stop_callback);

void csi_enable_int(int arg)
{
	unsigned long cr1 = __raw_readl(CSI_CSICR1);

	cr1 |= BIT_SOF_INTEN;
	if (arg == 1) {
		/* still capture needs DMA intterrupt */
		cr1 |= BIT_FB1_DMA_DONE_INTEN;
		cr1 |= BIT_FB2_DMA_DONE_INTEN;
	}
	__raw_writel(cr1, CSI_CSICR1);
}
EXPORT_SYMBOL(csi_enable_int);

void csi_disable_int(void)
{
	unsigned long cr1 = __raw_readl(CSI_CSICR1);

	cr1 &= ~BIT_SOF_INTEN;
	cr1 &= ~BIT_FB1_DMA_DONE_INTEN;
	cr1 &= ~BIT_FB2_DMA_DONE_INTEN;
	__raw_writel(cr1, CSI_CSICR1);
}
EXPORT_SYMBOL(csi_disable_int);

void csi_set_16bit_imagpara(int width, int height)
{
	int imag_para = 0;
	unsigned long cr3 = __raw_readl(CSI_CSICR3);

	imag_para = (width << 16) | (height * 2);
	__raw_writel(imag_para, CSI_CSIIMAG_PARA);

	/* reflash the embeded DMA controller */
	__raw_writel(cr3 | BIT_DMA_REFLASH_RFF, CSI_CSICR3);
}
EXPORT_SYMBOL(csi_set_16bit_imagpara);

void csi_set_12bit_imagpara(int width, int height)
{
	int imag_para = 0;
	unsigned long cr3 = __raw_readl(CSI_CSICR3);

	imag_para = (width << 16) | (height * 3 / 2);
	__raw_writel(imag_para, CSI_CSIIMAG_PARA);

	/* reflash the embeded DMA controller */
	__raw_writel(cr3 | BIT_DMA_REFLASH_RFF, CSI_CSICR3);
}
EXPORT_SYMBOL(csi_set_12bit_imagpara);

void csi_dmareq_rff_enable(void)
{
	unsigned long cr3 = __raw_readl(CSI_CSICR3);

	cr3 |= BIT_DMA_REQ_EN_RFF;
	cr3 |= BIT_HRESP_ERR_EN;
	__raw_writel(cr3, CSI_CSICR3);
}

void csi_dmareq_rff_disable(void)
{
	unsigned long cr3 = __raw_readl(CSI_CSICR3);

	cr3 &= ~BIT_DMA_REQ_EN_RFF;
	cr3 &= ~BIT_HRESP_ERR_EN;
	__raw_writel(cr3, CSI_CSICR3);
}

static const struct of_device_id fsl_csi_dt_ids[] = {
	{ .compatible = "fsl,imx6sl-csi", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fsl_csi_dt_ids);

static int csi_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "No csi irq found.\n");
		ret = -ENODEV;
		goto err;
	}
	irq_nr = res->start;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "No csi base address found.\n");
		ret = -ENODEV;
		goto err;
	}
	csi_regbase = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!csi_regbase) {
		dev_err(&pdev->dev, "ioremap failed with csi base\n");
		ret = -ENOMEM;
		goto err;
	}

	csihw_reset();
	csi_init_interface();
	csi_dmareq_rff_disable();

err:
	return ret;
}

static int csi_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver csi_driver = {
	.driver = {
		   .name = "fsl_csi",
		   .of_match_table = of_match_ptr(fsl_csi_dt_ids),
		   },
	.probe = csi_probe,
	.remove = csi_remove,
};

module_platform_driver(csi_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("fsl CSI driver");
MODULE_LICENSE("GPL");
