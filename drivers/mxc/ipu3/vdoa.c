/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <linux/types.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/ipu.h>
#include <linux/iram_alloc.h>

#include "vdoa.h"
/* 6band(3field* double buffer) * (width*2) * bandline(8)
	= 6x1024x2x8 = 96k or 72k(1.5byte) */
#define MAX_VDOA_IRAM_SIZE	(1024*96)
#define VDOA_IRAM_SIZE		(1024*72)

#define VDOAC_BAND_HEIGHT_32LINES	(32)
#define VDOAC_BAND_HEIGHT_16LINES	(16)
#define VDOAC_BAND_HEIGHT_8LINES	(8)
#define VDOAC_THREE_FRAMES		(0x1 << 2)
#define VDOAC_SYNC_BAND_MODE		(0x1 << 3)
#define VDOAC_SCAN_ORDER_INTERLACED	(0x1 << 4)
#define VDOAC_PFS_YUYV			(0x1 << 5)
#define VDOAC_IPU_SEL_1			(0x1 << 6)
#define VDOAFP_FH_MASK			(0x1FFF)
#define VDOAFP_FH_SHIFT			(16)
#define VDOAFP_FW_MASK			(0x3FFF)
#define VDOAFP_FW_SHIFT			(0)
#define VDOASL_VSLY_MASK		(0x3FFF)
#define VDOASL_VSLY_SHIFT		(16)
#define VDOASL_ISLY_MASK		(0x7FFF)
#define VDOASL_ISLY_SHIFT		(0)
#define VDOASRR_START_XFER		(0x2)
#define VDOASRR_SWRST			(0x1)
#define VDOAIEIST_TRANSFER_ERR		(0x2)
#define VDOAIEIST_TRANSFER_END		(0x1)

#define	VDOAC		(0x0)	/* Control Register */
#define	VDOASRR		(0x4)	/* Start and Reset Register */
#define	VDOAIE		(0x8)	/* Interrupt Enable Register */
#define	VDOAIST		(0xc)	/* Interrupt Status Register */
#define	VDOAFP		(0x10)	/* Frame Parameters Register */
#define	VDOAIEBA00	(0x14)	/* External Buffer n Frame m Address Register */
#define	VDOAIEBA01	(0x18)	/* External Buffer n Frame m Address Register */
#define	VDOAIEBA02	(0x1c)	/* External Buffer n Frame m Address Register */
#define	VDOAIEBA10	(0x20)	/* External Buffer n Frame m Address Register */
#define	VDOAIEBA11	(0x24)	/* External Buffer n Frame m Address Register */
#define	VDOAIEBA12	(0x28)	/* External Buffer n Frame m Address Register */
#define	VDOASL		(0x2c)	/* IPU Stride Line Register */
#define	VDOAIUBO	(0x30)	/* IPU Chroma Buffer Offset Register */
#define	VDOAVEBA0	(0x34)	/* External Buffer m Address Register */
#define	VDOAVEBA1	(0x38)	/* External Buffer m Address Register */
#define	VDOAVEBA2	(0x3c)	/* External Buffer m Address Register */
#define	VDOAVUBO	(0x40)	/* VPU Chroma Buffer Offset */
#define	VDOASR		(0x44)	/* Status Register */
#define	VDOATD		(0x48)	/* Test Debug Register */


enum {
	VDOA_INIT	= 0x1,
	VDOA_GET	= 0x2,
	VDOA_SETUP	= 0x4,
	VDOA_GET_OBUF	= 0x8,
	VDOA_START	= 0x10,
	VDOA_INIRQ	= 0x20,
	VDOA_STOP	= 0x40,
	VDOA_PUT	= VDOA_INIT,
};

enum {
	VDOA_NULL	= 0,
	VDOA_FRAME	= 1,
	VDOA_PREV_FIELD	= 2,
	VDOA_CURR_FIELD	= 3,
	VDOA_NEXT_FIELD	= 4,
};

#define CHECK_STATE(expect, retcode)					\
do {									\
	if (!((expect) & vdoa->state)) {				\
		dev_err(vdoa->dev, "ERR: %s state:0x%x, expect:0x%x.\n",\
				__func__, vdoa->state, (expect));	\
		retcode;						\
	}								\
} while (0)

#define CHECK_NULL_PTR(ptr)						\
do {									\
	pr_debug("vdoa_ptr:0x%p in %s state:0x%x.\n",			\
			vdoa, __func__, vdoa->state);			\
	if (NULL == (ptr)) {						\
		pr_err("ERR vdoa: %s state:0x%x null ptr.\n",		\
				__func__, vdoa->state);			\
	}								\
} while (0)

struct vdoa_info {
	int		state;
	struct device	*dev;
	struct clk	*clk;
	void __iomem	*reg_base;
	void __iomem	*iram_base;
	unsigned long	iram_paddr;
	int		irq;
	int		field;
	struct completion comp;
};

static struct vdoa_info *g_vdoa;
static unsigned long iram_size;
static DEFINE_MUTEX(vdoa_lock);

static inline void vdoa_read_register(struct vdoa_info *vdoa,
				u32 reg, u32 *val)
{
	*val = ioread32(vdoa->reg_base + reg);
	dev_dbg(vdoa->dev, "read_reg:0x%02x, val:0x%08x.\n", reg, *val);
}

static inline void vdoa_write_register(struct vdoa_info *vdoa,
				u32 reg, u32 val)
{
	iowrite32(val, vdoa->reg_base + reg);
	dev_dbg(vdoa->dev, "\t\twrite_reg:0x%02x, val:0x%08x.\n", reg, val);
}

static void dump_registers(struct vdoa_info *vdoa)
{
	int i;
	u32 data;

	for (i = VDOAC; i < VDOATD; i += 4)
		vdoa_read_register(vdoa, i, &data);
}

int vdoa_setup(vdoa_handle_t handle, struct vdoa_params *params)
{
	int	band_size;
	int	total_band_size = 0;
	int	ipu_stride;
	u32	data;
	struct vdoa_info *vdoa = (struct vdoa_info *)handle;

	CHECK_NULL_PTR(vdoa);
	CHECK_STATE(VDOA_GET | VDOA_GET_OBUF | VDOA_STOP, return -EINVAL);
	if (VDOA_GET == vdoa->state) {
		dev_dbg(vdoa->dev, "w:%d, h:%d.\n",
			 params->width, params->height);
		data = (params->band_lines == VDOAC_BAND_HEIGHT_32LINES) ? 2 :
			((params->band_lines == VDOAC_BAND_HEIGHT_16LINES) ?
				 1 : 0);
		data |= params->scan_order ? VDOAC_SCAN_ORDER_INTERLACED : 0;
		data |= params->band_mode ? VDOAC_SYNC_BAND_MODE : 0;
		data |= params->pfs ? VDOAC_PFS_YUYV : 0;
		data |= params->ipu_num ? VDOAC_IPU_SEL_1 : 0;
		vdoa_write_register(vdoa, VDOAC, data);

		data = ((params->width & VDOAFP_FW_MASK) << VDOAFP_FW_SHIFT) |
			((params->height & VDOAFP_FH_MASK) << VDOAFP_FH_SHIFT);
		vdoa_write_register(vdoa, VDOAFP, data);

		ipu_stride = params->pfs ? params->width << 1 : params->width;
		data = ((params->vpu_stride & VDOASL_VSLY_MASK) <<
							VDOASL_VSLY_SHIFT) |
			((ipu_stride & VDOASL_ISLY_MASK) << VDOASL_ISLY_SHIFT);
		vdoa_write_register(vdoa, VDOASL, data);

		dev_dbg(vdoa->dev, "band_mode:%d, band_line:%d, base:0x%lx.\n",
		params->band_mode, params->band_lines, vdoa->iram_paddr);
	}
	/*
	 * band size	= (luma_per_line + chroma_per_line) * bandLines
	 *		= width * (3/2 or 2) * bandLines
	 * double buffer mode used.
	 */
	if (params->pfs)
		band_size = (params->width << 1) * params->band_lines;
	else
		band_size = ((params->width * 3) >> 1) *
						params->band_lines;
	if (params->interlaced) {
		total_band_size = 6 * band_size; /* 3 frames*double buffer */
		if (iram_size < total_band_size) {
			dev_err(vdoa->dev, "iram_size:0x%lx is smaller than "
				"request:0x%x!\n", iram_size, total_band_size);
			return -EINVAL;
		}
		if (params->vfield_buf.prev_veba) {
			if (params->band_mode) {
				vdoa_write_register(vdoa, VDOAIEBA00,
							vdoa->iram_paddr);
				vdoa_write_register(vdoa, VDOAIEBA10,
						 vdoa->iram_paddr + band_size);
			} else
				vdoa_write_register(vdoa, VDOAIEBA00,
							params->ieba0);
			vdoa_write_register(vdoa, VDOAVEBA0,
					params->vfield_buf.prev_veba);
			vdoa->field = VDOA_PREV_FIELD;
		}
		if (params->vfield_buf.cur_veba) {
			if (params->band_mode) {
				vdoa_write_register(vdoa, VDOAIEBA01,
					 vdoa->iram_paddr + band_size * 2);
				vdoa_write_register(vdoa, VDOAIEBA11,
					 vdoa->iram_paddr + band_size * 3);
			} else
				vdoa_write_register(vdoa, VDOAIEBA01,
							params->ieba1);
			vdoa_write_register(vdoa, VDOAVEBA1,
					params->vfield_buf.cur_veba);
			vdoa->field = VDOA_CURR_FIELD;
		}
		if (params->vfield_buf.next_veba) {
			if (params->band_mode) {
				vdoa_write_register(vdoa, VDOAIEBA02,
					 vdoa->iram_paddr + band_size * 4);
				vdoa_write_register(vdoa, VDOAIEBA12,
					 vdoa->iram_paddr + band_size * 5);
			} else
				vdoa_write_register(vdoa, VDOAIEBA02,
							params->ieba2);
			vdoa_write_register(vdoa, VDOAVEBA2,
					params->vfield_buf.next_veba);
			vdoa->field = VDOA_NEXT_FIELD;
			vdoa_read_register(vdoa, VDOAC, &data);
			data |= VDOAC_THREE_FRAMES;
			vdoa_write_register(vdoa, VDOAC, data);
		}

		if (!params->pfs)
			vdoa_write_register(vdoa, VDOAIUBO,
				 params->width * params->band_lines);
		vdoa_write_register(vdoa, VDOAVUBO,
				 params->vfield_buf.vubo);
		dev_dbg(vdoa->dev, "total band_size:0x%x.\n", band_size*6);
	} else if (params->band_mode) {
		/* used for progressive frame resize on PrP channel */
		BUG(); /* currently not support */
		/* progressvie frame: band mode */
		vdoa_write_register(vdoa, VDOAIEBA00, vdoa->iram_paddr);
		vdoa_write_register(vdoa, VDOAIEBA10,
					 vdoa->iram_paddr + band_size);
		if (!params->pfs)
			vdoa_write_register(vdoa, VDOAIUBO,
					params->width * params->band_lines);
		dev_dbg(vdoa->dev, "total band_size:0x%x\n", band_size*2);
	} else {
		/* progressive frame: mem->mem, non-band mode */
		vdoa->field = VDOA_FRAME;
		vdoa_write_register(vdoa, VDOAVEBA0, params->vframe_buf.veba);
		vdoa_write_register(vdoa, VDOAVUBO, params->vframe_buf.vubo);
		vdoa_write_register(vdoa, VDOAIEBA00, params->ieba0);
		if (!params->pfs)
			/* note: iubo is relative value, based on ieba0 */
			vdoa_write_register(vdoa, VDOAIUBO,
					params->width * params->height);
	}
	vdoa->state = VDOA_SETUP;
	return 0;
}

void vdoa_get_output_buf(vdoa_handle_t handle, struct vdoa_ipu_buf *buf)
{
	u32	data;
	struct vdoa_info *vdoa = (struct vdoa_info *)handle;

	CHECK_NULL_PTR(vdoa);
	CHECK_STATE(VDOA_SETUP, return);
	vdoa->state = VDOA_GET_OBUF;
	memset(buf, 0, sizeof(*buf));

	vdoa_read_register(vdoa, VDOAC, &data);
	switch (vdoa->field) {
	case VDOA_FRAME:
	case VDOA_PREV_FIELD:
		vdoa_read_register(vdoa, VDOAIEBA00, &buf->ieba0);
		if (data & VDOAC_SYNC_BAND_MODE)
			vdoa_read_register(vdoa, VDOAIEBA10, &buf->ieba1);
		break;
	case VDOA_CURR_FIELD:
		vdoa_read_register(vdoa, VDOAIEBA01, &buf->ieba0);
		vdoa_read_register(vdoa, VDOAIEBA11, &buf->ieba1);
		break;
	case VDOA_NEXT_FIELD:
		vdoa_read_register(vdoa, VDOAIEBA02, &buf->ieba0);
		vdoa_read_register(vdoa, VDOAIEBA12, &buf->ieba1);
		break;
	default:
		BUG();
		break;
	}
	if (!(data & VDOAC_PFS_YUYV))
		vdoa_read_register(vdoa, VDOAIUBO, &buf->iubo);
}

int vdoa_start(vdoa_handle_t handle, int timeout_ms)
{
	int ret;
	struct vdoa_info *vdoa = (struct vdoa_info *)handle;

	CHECK_NULL_PTR(vdoa);
	CHECK_STATE(VDOA_GET_OBUF, return -EINVAL);
	vdoa->state = VDOA_START;
	init_completion(&vdoa->comp);
	vdoa_write_register(vdoa, VDOAIST,
			VDOAIEIST_TRANSFER_ERR | VDOAIEIST_TRANSFER_END);
	vdoa_write_register(vdoa, VDOAIE,
			VDOAIEIST_TRANSFER_ERR | VDOAIEIST_TRANSFER_END);

	enable_irq(vdoa->irq);
	vdoa_write_register(vdoa, VDOASRR, VDOASRR_START_XFER);
	dump_registers(vdoa);

	ret = wait_for_completion_timeout(&vdoa->comp,
			msecs_to_jiffies(timeout_ms));

	return ret > 0 ? 0 : -ETIMEDOUT;
}

void vdoa_stop(vdoa_handle_t handle)
{
	struct vdoa_info *vdoa = (struct vdoa_info *)handle;

	CHECK_NULL_PTR(vdoa);
	CHECK_STATE(VDOA_GET | VDOA_START | VDOA_INIRQ, return);
	vdoa->state = VDOA_STOP;

	disable_irq(vdoa->irq);

	vdoa_write_register(vdoa, VDOASRR, VDOASRR_SWRST);
}

void vdoa_get_handle(vdoa_handle_t *handle)
{
	struct vdoa_info *vdoa = g_vdoa;

	CHECK_NULL_PTR(handle);
	*handle = (vdoa_handle_t *)NULL;
	CHECK_STATE(VDOA_INIT, return);
	mutex_lock(&vdoa_lock);
	clk_enable(vdoa->clk);
	vdoa->state = VDOA_GET;
	vdoa->field = VDOA_NULL;
	vdoa_write_register(vdoa, VDOASRR, VDOASRR_SWRST);

	*handle = (vdoa_handle_t *)vdoa;
}

void vdoa_put_handle(vdoa_handle_t *handle)
{
	struct vdoa_info *vdoa = (struct vdoa_info *)(*handle);

	CHECK_NULL_PTR(vdoa);
	CHECK_STATE(VDOA_STOP, return);
	if (vdoa != g_vdoa)
		BUG();

	clk_disable(vdoa->clk);
	vdoa->state = VDOA_PUT;
	*handle = (vdoa_handle_t *)NULL;
	mutex_unlock(&vdoa_lock);
}

static irqreturn_t vdoa_irq_handler(int irq, void *data)
{
	u32 status, mask, val;
	struct vdoa_info *vdoa = data;

	CHECK_NULL_PTR(vdoa);
	CHECK_STATE(VDOA_START, return IRQ_HANDLED);
	vdoa->state = VDOA_INIRQ;
	vdoa_read_register(vdoa, VDOAIST, &status);
	vdoa_read_register(vdoa, VDOAIE, &mask);
	val = status & mask;
	vdoa_write_register(vdoa, VDOAIST, val);
	if (VDOAIEIST_TRANSFER_ERR & val)
		dev_err(vdoa->dev, "vdoa Transfer err irq!\n");
	if (VDOAIEIST_TRANSFER_END & val)
		dev_dbg(vdoa->dev, "vdoa Transfer end irq!\n");
	if (0 == val) {
		dev_err(vdoa->dev, "vdoa unknown irq!\n");
		BUG();
	}

	complete(&vdoa->comp);
	return IRQ_HANDLED;
}

/* IRAM Size in Kbytes, example:vdoa_iram_size=64, 64KBytes */
static int __init vdoa_iram_size_setup(char *options)
{
	int ret;

	ret = strict_strtoul(options, 0, &iram_size);
	if (ret)
		iram_size = 0;
	else
		iram_size *= SZ_1K;

	return 1;
}
__setup("vdoa_iram_size=", vdoa_iram_size_setup);

static int vdoa_probe(struct platform_device *pdev)
{
	int ret;
	struct vdoa_info *vdoa;
	struct resource *res;
	struct resource *res_irq;
	struct device	*dev;
	char   clk[] = "vdoa";

	vdoa = kzalloc(sizeof(struct vdoa_info), GFP_KERNEL);
	if (!vdoa) {
		ret = -ENOMEM;
		goto alloc_failed;
	}
	vdoa->dev = dev	= &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "get IORESOURCE_MEM error\n");
		ret = -ENODEV;
		goto res_mem_failed;
	}

	res = request_mem_region(res->start, resource_size(res), pdev->name);
	if (!res) {
		dev_err(dev, "request mem region error\n");
		ret = -EBUSY;
		goto req_mem_region;
	}
	vdoa->reg_base = ioremap(res->start, resource_size(res));
	if (!vdoa->reg_base) {
		dev_err(dev, "map vdoa registers error\n");
		ret = -EIO;
		goto err_ioremap;
	}

	res_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res_irq) {
		dev_err(dev, "failed to get irq resource\n");
		ret = -ENODEV;
		goto err_get_irq;
	}
	vdoa->irq = res_irq->start;
	ret = request_irq(vdoa->irq, vdoa_irq_handler, 0, "vdoa", vdoa);
	if (ret) {
		dev_err(dev, "request vdoa interrupt failed\n");
		ret = -EBUSY;
		goto err_req_irq;
	}
	disable_irq(vdoa->irq);

	vdoa->clk = clk_get(dev, clk);
	if (IS_ERR(vdoa->clk)) {
		dev_err(dev, "failed to get vdoa_clk\n");
		ret = PTR_ERR(vdoa->clk);
		goto err_clk;
	}
	if ((iram_size == 0) || (iram_size > MAX_VDOA_IRAM_SIZE))
		iram_size = VDOA_IRAM_SIZE;
	vdoa->iram_base = iram_alloc(iram_size, &vdoa->iram_paddr);
	if (!vdoa->iram_base) {
		dev_err(dev, "failed to get iram memory:0x%lx\n", iram_size);
		ret = -ENOMEM;
		goto err_iram_alloc;
	}
	dev_dbg(dev, "iram_base:0x%p,iram_paddr:0x%lx,size:0x%lx\n",
		 vdoa->iram_base, vdoa->iram_paddr, iram_size);

	vdoa->state = VDOA_INIT;
	dev_set_drvdata(dev, vdoa);
	g_vdoa = vdoa;
	dev_info(dev, "i.MX Video Data Order Adapter(VDOA) driver probed\n");
	return 0;

err_iram_alloc:
	clk_put(vdoa->clk);
err_clk:
err_req_irq:
err_get_irq:
	iounmap(vdoa->reg_base);
err_ioremap:
	release_mem_region(res->start, resource_size(res));
req_mem_region:
res_mem_failed:
	kfree(vdoa);
alloc_failed:
	return ret;
}

static int __devexit vdoa_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;
	struct vdoa_info *vdoa = dev_get_drvdata(&pdev->dev);

	clk_put(vdoa->clk);
	clk_disable(vdoa->clk);
	iram_free(vdoa->iram_paddr, iram_size);
	iounmap(vdoa->reg_base);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "get IORESOURCE_MEM error\n");
		ret = -ENODEV;
		goto res_mem_failed;
	}
	release_mem_region(res->start, resource_size(res));
	kfree(vdoa);
	dev_set_drvdata(&pdev->dev, NULL);

res_mem_failed:
	return ret;
}

static struct platform_driver vdoa_driver = {
	.driver = {
		   .name = "mxc_vdoa",
	},
	.probe = vdoa_probe,
	.remove = __devexit_p(vdoa_remove),
};

static int __init vdoa_init(void)
{
	int err;

	err = platform_driver_register(&vdoa_driver);
	if (err) {
		pr_err("vdoa_driver register failed\n");
		return -ENODEV;
	}
	return 0;
}

static void __exit vdoa_cleanup(void)
{
	platform_driver_unregister(&vdoa_driver);
}

module_init(vdoa_init);
module_exit(vdoa_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("i.MX Video Data Order Adapter(VDOA) driver");
MODULE_LICENSE("GPL");
