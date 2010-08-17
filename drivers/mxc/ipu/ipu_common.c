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
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/ipu.h>
#include <linux/fsl_devices.h>

#include "ipu_prv.h"
#include "ipu_regs.h"
#include "ipu_param_mem.h"

/*
 * This type definition is used to define a node in the GPIO interrupt queue for
 * registered interrupts for GPIO pins. Each node contains the GPIO signal number
 * associated with the ISR and the actual ISR function pointer.
 */
struct ipu_irq_node {
	irqreturn_t(*handler) (int, void *);	/*!< the ISR */
	const char *name;	/*!< device associated with the interrupt */
	void *dev_id;		/*!< some unique information for the ISR */
	__u32 flags;		/*!< not used */
};

/* Globals */
struct clk *g_ipu_clk;
struct clk *g_ipu_csi_clk;
static struct clk *dfm_clk;
int g_ipu_irq[2];
int g_ipu_hw_rev;
bool g_sec_chan_en[21];
uint32_t g_channel_init_mask;
DEFINE_SPINLOCK(ipu_lock);
struct device *g_ipu_dev;

static struct ipu_irq_node ipu_irq_list[IPU_IRQ_COUNT];
static const char driver_name[] = "mxc_ipu";

static uint32_t g_ipu_config;
static uint32_t g_channel_init_mask_backup;
static bool g_csi_used;

/* Static functions */
static irqreturn_t ipu_irq_handler(int irq, void *desc);
static void _ipu_pf_init(ipu_channel_params_t *params);
static void _ipu_pf_uninit(void);

static inline uint32_t channel_2_dma(ipu_channel_t ch, ipu_buffer_t type)
{
	return ((type == IPU_INPUT_BUFFER) ? ((uint32_t) ch & 0xFF) :
		((type == IPU_OUTPUT_BUFFER) ? (((uint32_t) ch >> 8) & 0xFF)
		 : (((uint32_t) ch >> 16) & 0xFF)));
};

static inline uint32_t DMAParamAddr(uint32_t dma_ch)
{
	return 0x10000 | (dma_ch << 4);
};

/*!
 * This function is called by the driver framework to initialize the IPU
 * hardware.
 *
 * @param       dev       The device structure for the IPU passed in by the framework.
 *
 * @return      This function returns 0 on success or negative error code on error
 */
static
int ipu_probe(struct platform_device *pdev)
{
	struct mxc_ipu_config *ipu_conf = pdev->dev.platform_data;

	spin_lock_init(&ipu_lock);

	g_ipu_dev = &pdev->dev;
	g_ipu_hw_rev = ipu_conf->rev;

	/* Register IPU interrupts */
	g_ipu_irq[0] = platform_get_irq(pdev, 0);
	if (g_ipu_irq[0] < 0)
		return -EINVAL;

	if (request_irq(g_ipu_irq[0], ipu_irq_handler, 0, driver_name, 0) != 0) {
		dev_err(g_ipu_dev, "request SYNC interrupt failed\n");
		return -EBUSY;
	}
	/* Some platforms have 2 IPU interrupts */
	g_ipu_irq[1] = platform_get_irq(pdev, 1);
	if (g_ipu_irq[1] >= 0) {
		if (request_irq
		    (g_ipu_irq[1], ipu_irq_handler, 0, driver_name, 0) != 0) {
			dev_err(g_ipu_dev, "request ERR interrupt failed\n");
			return -EBUSY;
		}
	}

	/* Enable IPU and CSI clocks */
	/* Get IPU clock freq */
	g_ipu_clk = clk_get(&pdev->dev, "ipu_clk");
	dev_dbg(g_ipu_dev, "ipu_clk = %lu\n", clk_get_rate(g_ipu_clk));

	g_ipu_csi_clk = clk_get(&pdev->dev, "csi_clk");

	dfm_clk = clk_get(NULL, "dfm_clk");

	clk_enable(g_ipu_clk);

	/* resetting the CONF register of the IPU */
	__raw_writel(0x00000000, IPU_CONF);

	__raw_writel(0x00100010L, DI_HSP_CLK_PER);

	/* Set SDC refresh channels as high priority */
	__raw_writel(0x0000C000L, IDMAC_CHA_PRI);

	/* Set to max back to back burst requests */
	__raw_writel(0x00000000L, IDMAC_CONF);

	register_ipu_device();

	return 0;
}

/*!
 * This function is called to initialize a logical IPU channel.
 *
 * @param       channel Input parameter for the logical channel ID to initalize.
 *
 * @param       params  Input parameter containing union of channel initialization
 *                      parameters.
 *
 * @return      This function returns 0 on success or negative error code on fail
 */
int32_t ipu_init_channel(ipu_channel_t channel, ipu_channel_params_t *params)
{
	uint32_t ipu_conf;
	uint32_t reg;
	unsigned long lock_flags;

	dev_dbg(g_ipu_dev, "init channel = %d\n", IPU_CHAN_ID(channel));

	if ((channel != MEM_SDC_BG) && (channel != MEM_SDC_FG) &&
	    (channel != MEM_ROT_ENC_MEM) && (channel != MEM_ROT_VF_MEM) &&
	    (channel != MEM_ROT_PP_MEM) && (channel != CSI_MEM)
	    && (params == NULL)) {
		return -EINVAL;
	}

	spin_lock_irqsave(&ipu_lock, lock_flags);

	ipu_conf = __raw_readl(IPU_CONF);
	if (ipu_conf == 0) {
		clk_enable(g_ipu_clk);
	}

	if (g_channel_init_mask & (1L << IPU_CHAN_ID(channel))) {
		dev_err(g_ipu_dev, "Warning: channel already initialized %d\n",
			IPU_CHAN_ID(channel));
	}

	switch (channel) {
	case CSI_PRP_VF_MEM:
		reg = __raw_readl(IPU_FS_PROC_FLOW);
		__raw_writel(reg & ~FS_VF_IN_VALID, IPU_FS_PROC_FLOW);

		if (params->mem_prp_vf_mem.graphics_combine_en)
			g_sec_chan_en[IPU_CHAN_ID(channel)] = true;

		_ipu_ic_init_prpvf(params, true);
		break;
	case CSI_PRP_VF_ADC:
		reg = __raw_readl(IPU_FS_PROC_FLOW);
		__raw_writel(reg | (FS_DEST_ADC << FS_PRPVF_DEST_SEL_OFFSET),
			     IPU_FS_PROC_FLOW);

		_ipu_adc_init_channel(CSI_PRP_VF_ADC,
				      params->csi_prp_vf_adc.disp,
				      WriteTemplateNonSeq,
				      params->csi_prp_vf_adc.out_left,
				      params->csi_prp_vf_adc.out_top);

		_ipu_ic_init_prpvf(params, true);
		break;
	case MEM_PRP_VF_MEM:
		reg = __raw_readl(IPU_FS_PROC_FLOW);
		__raw_writel(reg | FS_VF_IN_VALID, IPU_FS_PROC_FLOW);

		if (params->mem_prp_vf_mem.graphics_combine_en)
			g_sec_chan_en[IPU_CHAN_ID(channel)] = true;

		_ipu_ic_init_prpvf(params, false);
		break;
	case MEM_ROT_VF_MEM:
		_ipu_ic_init_rotate_vf(params);
		break;
	case CSI_PRP_ENC_MEM:
		reg = __raw_readl(IPU_FS_PROC_FLOW);
		__raw_writel(reg & ~FS_ENC_IN_VALID, IPU_FS_PROC_FLOW);
		_ipu_ic_init_prpenc(params, true);
		break;
	case MEM_PRP_ENC_MEM:
		reg = __raw_readl(IPU_FS_PROC_FLOW);
		__raw_writel(reg | FS_ENC_IN_VALID, IPU_FS_PROC_FLOW);
		_ipu_ic_init_prpenc(params, false);
		break;
	case MEM_ROT_ENC_MEM:
		_ipu_ic_init_rotate_enc(params);
		break;
	case MEM_PP_ADC:
		reg = __raw_readl(IPU_FS_PROC_FLOW);
		__raw_writel(reg | (FS_DEST_ADC << FS_PP_DEST_SEL_OFFSET),
			     IPU_FS_PROC_FLOW);

		_ipu_adc_init_channel(MEM_PP_ADC, params->mem_pp_adc.disp,
				      WriteTemplateNonSeq,
				      params->mem_pp_adc.out_left,
				      params->mem_pp_adc.out_top);

		if (params->mem_pp_adc.graphics_combine_en)
			g_sec_chan_en[IPU_CHAN_ID(channel)] = true;

		_ipu_ic_init_pp(params);
		break;
	case MEM_PP_MEM:
		if (params->mem_pp_mem.graphics_combine_en)
			g_sec_chan_en[IPU_CHAN_ID(channel)] = true;

		_ipu_ic_init_pp(params);
		break;
	case MEM_ROT_PP_MEM:
		_ipu_ic_init_rotate_pp(params);
		break;
	case CSI_MEM:
		_ipu_ic_init_csi(params);
		break;

	case MEM_PF_Y_MEM:
	case MEM_PF_U_MEM:
	case MEM_PF_V_MEM:
		/* Enable PF block */
		_ipu_pf_init(params);
		break;

	case MEM_SDC_BG:
		break;
	case MEM_SDC_FG:
		break;
	case ADC_SYS1:
		_ipu_adc_init_channel(ADC_SYS1, params->adc_sys1.disp,
				      params->adc_sys1.ch_mode,
				      params->adc_sys1.out_left,
				      params->adc_sys1.out_top);
		break;
	case ADC_SYS2:
		_ipu_adc_init_channel(ADC_SYS2, params->adc_sys2.disp,
				      params->adc_sys2.ch_mode,
				      params->adc_sys2.out_left,
				      params->adc_sys2.out_top);
		break;
	default:
		dev_err(g_ipu_dev, "Missing channel initialization\n");
		spin_unlock_irqrestore(&ipu_lock, lock_flags);
		return -EINVAL;
	}

	/* Enable IPU sub module */
	g_channel_init_mask |= 1L << IPU_CHAN_ID(channel);

	if (g_channel_init_mask & 0x00000066L) {	/*CSI */
		ipu_conf |= IPU_CONF_CSI_EN;
		if (cpu_is_mx31() || cpu_is_mx32()) {
			g_csi_used = true;
		}
	}
	if (g_channel_init_mask & 0x00001FFFL) {	/*IC */
		ipu_conf |= IPU_CONF_IC_EN;
	}
	if (g_channel_init_mask & 0x00000A10L) {	/*ROT */
		ipu_conf |= IPU_CONF_ROT_EN;
	}
	if (g_channel_init_mask & 0x0001C000L) {	/*SDC */
		ipu_conf |= IPU_CONF_SDC_EN | IPU_CONF_DI_EN;
	}
	if (g_channel_init_mask & 0x00061140L) {	/*ADC */
		ipu_conf |= IPU_CONF_ADC_EN | IPU_CONF_DI_EN;
	}
	if (g_channel_init_mask & 0x00380000L) {	/*PF */
		ipu_conf |= IPU_CONF_PF_EN;
	}
	__raw_writel(ipu_conf, IPU_CONF);

	spin_unlock_irqrestore(&ipu_lock, lock_flags);

	return 0;
}

/*!
 * This function is called to uninitialize a logical IPU channel.
 *
 * @param       channel Input parameter for the logical channel ID to uninitalize.
 */
void ipu_uninit_channel(ipu_channel_t channel)
{
	unsigned long lock_flags;
	uint32_t reg;
	uint32_t dma, mask = 0;
	uint32_t ipu_conf;

	spin_lock_irqsave(&ipu_lock, lock_flags);

	if ((g_channel_init_mask & (1L << IPU_CHAN_ID(channel))) == 0) {
		dev_err(g_ipu_dev, "Channel already uninitialized %d\n",
			IPU_CHAN_ID(channel));
		spin_unlock_irqrestore(&ipu_lock, lock_flags);
		return;
	}

	/* Make sure channel is disabled */
	/* Get input and output dma channels */
	dma = channel_2_dma(channel, IPU_OUTPUT_BUFFER);
	if (dma != IDMA_CHAN_INVALID)
		mask |= 1UL << dma;
	dma = channel_2_dma(channel, IPU_INPUT_BUFFER);
	if (dma != IDMA_CHAN_INVALID)
		mask |= 1UL << dma;
	/* Get secondary input dma channel */
	if (g_sec_chan_en[IPU_CHAN_ID(channel)]) {
		dma = channel_2_dma(channel, IPU_SEC_INPUT_BUFFER);
		if (dma != IDMA_CHAN_INVALID) {
			mask |= 1UL << dma;
		}
	}
	if (mask & __raw_readl(IDMAC_CHA_EN)) {
		dev_err(g_ipu_dev,
			"Channel %d is not disabled, disable first\n",
			IPU_CHAN_ID(channel));
		spin_unlock_irqrestore(&ipu_lock, lock_flags);
		return;
	}

	/* Reset the double buffer */
	reg = __raw_readl(IPU_CHA_DB_MODE_SEL);
	__raw_writel(reg & ~mask, IPU_CHA_DB_MODE_SEL);

	g_sec_chan_en[IPU_CHAN_ID(channel)] = false;

	switch (channel) {
	case CSI_MEM:
		_ipu_ic_uninit_csi();
		break;
	case CSI_PRP_VF_ADC:
		reg = __raw_readl(IPU_FS_PROC_FLOW);
		__raw_writel(reg & ~FS_PRPVF_DEST_SEL_MASK, IPU_FS_PROC_FLOW);

		_ipu_adc_uninit_channel(CSI_PRP_VF_ADC);

		/* Fall thru */
	case CSI_PRP_VF_MEM:
	case MEM_PRP_VF_MEM:
		_ipu_ic_uninit_prpvf();
		break;
	case MEM_PRP_VF_ADC:
		break;
	case MEM_ROT_VF_MEM:
		_ipu_ic_uninit_rotate_vf();
		break;
	case CSI_PRP_ENC_MEM:
	case MEM_PRP_ENC_MEM:
		_ipu_ic_uninit_prpenc();
		break;
	case MEM_ROT_ENC_MEM:
		_ipu_ic_uninit_rotate_enc();
		break;
	case MEM_PP_ADC:
		reg = __raw_readl(IPU_FS_PROC_FLOW);
		__raw_writel(reg & ~FS_PP_DEST_SEL_MASK, IPU_FS_PROC_FLOW);

		_ipu_adc_uninit_channel(MEM_PP_ADC);

		/* Fall thru */
	case MEM_PP_MEM:
		_ipu_ic_uninit_pp();
		break;
	case MEM_ROT_PP_MEM:
		_ipu_ic_uninit_rotate_pp();
		break;

	case MEM_PF_Y_MEM:
		_ipu_pf_uninit();
		break;
	case MEM_PF_U_MEM:
	case MEM_PF_V_MEM:
		break;

	case MEM_SDC_BG:
		break;
	case MEM_SDC_FG:
		break;
	case ADC_SYS1:
		_ipu_adc_uninit_channel(ADC_SYS1);
		break;
	case ADC_SYS2:
		_ipu_adc_uninit_channel(ADC_SYS2);
		break;
	case MEM_SDC_MASK:
	case CHAN_NONE:
		break;
	}

	g_channel_init_mask &= ~(1L << IPU_CHAN_ID(channel));

	ipu_conf = __raw_readl(IPU_CONF);
	if ((g_channel_init_mask & 0x00000066L) == 0) {	/*CSI */
		ipu_conf &= ~IPU_CONF_CSI_EN;
	}
	if ((g_channel_init_mask & 0x00001FFFL) == 0) {	/*IC */
		ipu_conf &= ~IPU_CONF_IC_EN;
	}
	if ((g_channel_init_mask & 0x00000A10L) == 0) {	/*ROT */
		ipu_conf &= ~IPU_CONF_ROT_EN;
	}
	if ((g_channel_init_mask & 0x0001C000L) == 0) {	/*SDC */
		ipu_conf &= ~IPU_CONF_SDC_EN;
	}
	if ((g_channel_init_mask & 0x00061140L) == 0) {	/*ADC */
		ipu_conf &= ~IPU_CONF_ADC_EN;
	}
	if ((g_channel_init_mask & 0x0007D140L) == 0) {	/*DI */
		ipu_conf &= ~IPU_CONF_DI_EN;
	}
	if ((g_channel_init_mask & 0x00380000L) == 0) {	/*PF */
		ipu_conf &= ~IPU_CONF_PF_EN;
	}
	__raw_writel(ipu_conf, IPU_CONF);
	if (ipu_conf == 0) {
		clk_disable(g_ipu_clk);
	}

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
}

/*!
 * This function is called to initialize a buffer for logical IPU channel.
 *
 * @param       channel         Input parameter for the logical channel ID.
 *
 * @param       type            Input parameter which buffer to initialize.
 *
 * @param       pixel_fmt       Input parameter for pixel format of buffer. Pixel
 *                              format is a FOURCC ASCII code.
 *
 * @param       width           Input parameter for width of buffer in pixels.
 *
 * @param       height          Input parameter for height of buffer in pixels.
 *
 * @param       stride          Input parameter for stride length of buffer
 *                              in pixels.
 *
 * @param       rot_mode        Input parameter for rotation setting of buffer.
 *                              A rotation setting other than \b IPU_ROTATE_VERT_FLIP
 *                              should only be used for input buffers of rotation
 *                              channels.
 *
 * @param       phyaddr_0       Input parameter buffer 0 physical address.
 *
 * @param       phyaddr_1       Input parameter buffer 1 physical address.
 *                              Setting this to a value other than NULL enables
 *                              double buffering mode.
 *
 * @param       u		       	private u offset for additional cropping,
 *								zero if not used.
 *
 * @param       v		       	private v offset for additional cropping,
 *								zero if not used.
 *
 * @return      This function returns 0 on success or negative error code on fail
 */
int32_t ipu_init_channel_buffer(ipu_channel_t channel, ipu_buffer_t type,
				uint32_t pixel_fmt,
				uint16_t width, uint16_t height,
				uint32_t stride,
				ipu_rotate_mode_t rot_mode,
				dma_addr_t phyaddr_0, dma_addr_t phyaddr_1,
				uint32_t u, uint32_t v)
{
	uint32_t params[10];
	unsigned long lock_flags;
	uint32_t reg;
	uint32_t dma_chan;

	dma_chan = channel_2_dma(channel, type);

	if (stride < width * bytes_per_pixel(pixel_fmt))
		stride = width * bytes_per_pixel(pixel_fmt);

	if (dma_chan == IDMA_CHAN_INVALID)
		return -EINVAL;

	if (stride % 4) {
		dev_err(g_ipu_dev,
			"Stride must be 32-bit aligned, stride = %d\n", stride);
		return -EINVAL;
	}
	/* IC channels' width must be multiple of 8 pixels     */
	if ((dma_chan <= 13) && (width % 8)) {
		dev_err(g_ipu_dev, "width must be 8 pixel multiple\n");
		return -EINVAL;
	}
	/* Build parameter memory data for DMA channel */
	_ipu_ch_param_set_size(params, pixel_fmt, width, height, stride, u, v);
	_ipu_ch_param_set_buffer(params, phyaddr_0, phyaddr_1);
	_ipu_ch_param_set_rotation(params, rot_mode);
	/* Some channels (rotation) have restriction on burst length */
	if ((dma_chan == 10) || (dma_chan == 11) || (dma_chan == 13)) {
		_ipu_ch_param_set_burst_size(params, 8);
	} else if (dma_chan == 24) {	/* PF QP channel */
		_ipu_ch_param_set_burst_size(params, 4);
	} else if (dma_chan == 25) {	/* PF H264 BS channel */
		_ipu_ch_param_set_burst_size(params, 16);
	} else if (((dma_chan == 14) || (dma_chan == 15)) &&
		   pixel_fmt == IPU_PIX_FMT_RGB565) {
		_ipu_ch_param_set_burst_size(params, 16);
	}

	spin_lock_irqsave(&ipu_lock, lock_flags);

	_ipu_write_param_mem(DMAParamAddr(dma_chan), params, 10);

	reg = __raw_readl(IPU_CHA_DB_MODE_SEL);
	if (phyaddr_1) {
		reg |= 1UL << dma_chan;
	} else {
		reg &= ~(1UL << dma_chan);
	}
	__raw_writel(reg, IPU_CHA_DB_MODE_SEL);

	/* Reset to buffer 0 */
	__raw_writel(1UL << dma_chan, IPU_CHA_CUR_BUF);

	spin_unlock_irqrestore(&ipu_lock, lock_flags);

	return 0;
}

/*!
 * This function is called to update the physical address of a buffer for
 * a logical IPU channel.
 *
 * @param       channel         Input parameter for the logical channel ID.
 *
 * @param       type            Input parameter which buffer to initialize.
 *
 * @param       bufNum          Input parameter for which buffer number to update.
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
	unsigned long lock_flags;
	uint32_t dma_chan = channel_2_dma(channel, type);

	if (dma_chan == IDMA_CHAN_INVALID)
		return -EINVAL;

	spin_lock_irqsave(&ipu_lock, lock_flags);

	if (bufNum == 0) {
		reg = __raw_readl(IPU_CHA_BUF0_RDY);
		if (reg & (1UL << dma_chan)) {
			spin_unlock_irqrestore(&ipu_lock, lock_flags);
			return -EACCES;
		}
		__raw_writel(DMAParamAddr(dma_chan) + 0x0008UL, IPU_IMA_ADDR);
		__raw_writel(phyaddr, IPU_IMA_DATA);
	} else {
		reg = __raw_readl(IPU_CHA_BUF1_RDY);
		if (reg & (1UL << dma_chan)) {
			spin_unlock_irqrestore(&ipu_lock, lock_flags);
			return -EACCES;
		}
		__raw_writel(DMAParamAddr(dma_chan) + 0x0009UL, IPU_IMA_ADDR);
		__raw_writel(phyaddr, IPU_IMA_DATA);
	}

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
	dev_dbg(g_ipu_dev, "IPU: update IDMA ch %d buf %d = 0x%08X\n",
		dma_chan, bufNum, phyaddr);
	return 0;
}

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
 * @param       u				predefined private u offset for additional cropping,
 *								zero if not used.
 *
 * @param       v				predefined private v offset for additional cropping,
 *								zero if not used.
 *
 * @param		vertical_offset vertical offset for Y coordinate
 * 								in the existed frame
 *
 *
 * @param		horizontal_offset horizontal offset for X coordinate
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
	uint32_t reg;
	int ret = 0;
	unsigned long lock_flags;
	uint32_t dma_chan = channel_2_dma(channel, type);

	if (dma_chan == IDMA_CHAN_INVALID)
		return -EINVAL;

	spin_lock_irqsave(&ipu_lock, lock_flags);

	ret = -EACCES;
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
 * @return      This function returns 0 on success or negative error code on fail
 */
int32_t ipu_select_buffer(ipu_channel_t channel, ipu_buffer_t type,
			  uint32_t bufNum)
{
	uint32_t dma_chan = channel_2_dma(channel, type);

	if (dma_chan == IDMA_CHAN_INVALID)
		return -EINVAL;

	if (bufNum == 0) {
		/*Mark buffer 0 as ready. */
		__raw_writel(1UL << dma_chan, IPU_CHA_BUF0_RDY);
	} else {
		/*Mark buffer 1 as ready. */
		__raw_writel(1UL << dma_chan, IPU_CHA_BUF1_RDY);
	}
	return 0;
}

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
		reg = __raw_readl(IPU_CHA_BUF0_RDY);
	else
		reg = __raw_readl(IPU_CHA_BUF1_RDY);

	if (reg & (1UL << dma_chan))
		return 1;
	else
		return 0;
}
EXPORT_SYMBOL(ipu_check_buffer_busy);

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
	unsigned long lock_flags;
	uint32_t out_dma;
	uint32_t in_dma;
	bool isProc;
	uint32_t value;
	uint32_t mask;
	uint32_t offset;
	uint32_t fs_proc_flow;
	uint32_t fs_disp_flow;

	spin_lock_irqsave(&ipu_lock, lock_flags);

	fs_proc_flow = __raw_readl(IPU_FS_PROC_FLOW);
	fs_disp_flow = __raw_readl(IPU_FS_DISP_FLOW);

	out_dma = (1UL << channel_2_dma(src_ch, IPU_OUTPUT_BUFFER));
	in_dma = (1UL << channel_2_dma(dest_ch, IPU_INPUT_BUFFER));

	/* PROCESS THE OUTPUT DMA CH */
	switch (out_dma) {
		/*VF-> */
	case IDMA_IC_1:
		pr_debug("Link VF->");
		isProc = true;
		mask = FS_PRPVF_DEST_SEL_MASK;
		offset = FS_PRPVF_DEST_SEL_OFFSET;
		value = (in_dma == IDMA_IC_11) ? FS_DEST_ROT :	/*->VF_ROT */
		    (in_dma == IDMA_ADC_SYS1_WR) ? FS_DEST_ADC1 :	/* ->ADC1 */
		    (in_dma == IDMA_ADC_SYS2_WR) ? FS_DEST_ADC2 :	/* ->ADC2 */
		    (in_dma == IDMA_SDC_BG) ? FS_DEST_SDC_BG :	/*->SDC_BG */
		    (in_dma == IDMA_SDC_FG) ? FS_DEST_SDC_FG :	/*->SDC_FG */
		    (in_dma == IDMA_ADC_SYS1_WR) ? FS_DEST_ADC1 :	/*->ADC1 */
		    /* ->ADCDirect */
		    0;
		break;

		/*VF_ROT-> */
	case IDMA_IC_9:
		pr_debug("Link VF_ROT->");
		isProc = true;
		mask = FS_PRPVF_ROT_DEST_SEL_MASK;
		offset = FS_PRPVF_ROT_DEST_SEL_OFFSET;
		value = (in_dma == IDMA_ADC_SYS1_WR) ? FS_DEST_ADC1 :	/*->ADC1 */
		    (in_dma == IDMA_ADC_SYS2_WR) ? FS_DEST_ADC2 :	/* ->ADC2 */
		    (in_dma == IDMA_SDC_BG) ? FS_DEST_SDC_BG :	/*->SDC_BG */
		    (in_dma == IDMA_SDC_FG) ? FS_DEST_SDC_FG :	/*->SDC_FG */
		    0;
		break;

		/*ENC-> */
	case IDMA_IC_0:
		pr_debug("Link ENC->");
		isProc = true;
		mask = 0;
		offset = 0;
		value = (in_dma == IDMA_IC_10) ? FS_PRPENC_DEST_SEL :	/*->ENC_ROT	*/
		    0;
		break;

		/*PP-> */
	case IDMA_IC_2:
		pr_debug("Link PP->");
		isProc = true;
		mask = FS_PP_DEST_SEL_MASK;
		offset = FS_PP_DEST_SEL_OFFSET;
		value = (in_dma == IDMA_IC_13) ? FS_DEST_ROT :	/*->PP_ROT */
		    (in_dma == IDMA_ADC_SYS1_WR) ? FS_DEST_ADC1 :	/* ->ADC1 */
		    (in_dma == IDMA_ADC_SYS2_WR) ? FS_DEST_ADC2 :	/* ->ADC2 */
		    (in_dma == IDMA_SDC_BG) ? FS_DEST_SDC_BG :	/*->SDC_BG */
		    (in_dma == IDMA_SDC_FG) ? FS_DEST_SDC_FG :	/*->SDC_FG */
		    /* ->ADCDirect */
		    0;
		break;

		/*PP_ROT-> */
	case IDMA_IC_12:
		pr_debug("Link PP_ROT->");
		isProc = true;
		mask = FS_PP_ROT_DEST_SEL_MASK;
		offset = FS_PP_ROT_DEST_SEL_OFFSET;
		value = (in_dma == IDMA_IC_5) ? FS_DEST_ROT :	/*->PP */
		    (in_dma == IDMA_ADC_SYS1_WR) ? FS_DEST_ADC1 :	/* ->ADC1 */
		    (in_dma == IDMA_ADC_SYS2_WR) ? FS_DEST_ADC2 :	/* ->ADC2 */
		    (in_dma == IDMA_SDC_BG) ? FS_DEST_SDC_BG :	/*->SDC_BG */
		    (in_dma == IDMA_SDC_FG) ? FS_DEST_SDC_FG :	/*->SDC_FG */
		    0;
		break;

		/*PF-> */
	case IDMA_PF_Y_OUT:
	case IDMA_PF_U_OUT:
	case IDMA_PF_V_OUT:
		pr_debug("Link PF->");
		isProc = true;
		mask = FS_PF_DEST_SEL_MASK;
		offset = FS_PF_DEST_SEL_OFFSET;
		value = (in_dma == IDMA_IC_5) ? FS_PF_DEST_PP :
		    (in_dma == IDMA_IC_13) ? FS_PF_DEST_ROT : 0;
		break;

		/* Invalid Chainings: ENC_ROT-> */
	default:
		pr_debug("Link Invalid->");
		value = 0;
		break;

	}

	if (value) {
		if (isProc) {
			fs_proc_flow &= ~mask;
			fs_proc_flow |= (value << offset);
		} else {
			fs_disp_flow &= ~mask;
			fs_disp_flow |= (value << offset);
		}
	} else {
		dev_err(g_ipu_dev, "Invalid channel chaining %d -> %d\n",
			out_dma, in_dma);
		return -EINVAL;
	}

	/* PROCESS THE INPUT DMA CH */
	switch (in_dma) {
		/* ->VF_ROT */
	case IDMA_IC_11:
		pr_debug("VF_ROT\n");
		isProc = true;
		mask = 0;
		offset = 0;
		value = (out_dma == IDMA_IC_1) ? FS_PRPVF_ROT_SRC_SEL :	/*VF-> */
		    0;
		break;

		/* ->ENC_ROT */
	case IDMA_IC_10:
		pr_debug("ENC_ROT\n");
		isProc = true;
		mask = 0;
		offset = 0;
		value = (out_dma == IDMA_IC_0) ? FS_PRPENC_ROT_SRC_SEL :	/*ENC-> */
		    0;
		break;

		/* ->PP */
	case IDMA_IC_5:
		pr_debug("PP\n");
		isProc = true;
		mask = FS_PP_SRC_SEL_MASK;
		offset = FS_PP_SRC_SEL_OFFSET;
		value = (out_dma == IDMA_PF_Y_OUT) ? FS_PP_SRC_PF :	/*PF-> */
		    (out_dma == IDMA_PF_U_OUT) ? FS_PP_SRC_PF :	/*PF-> */
		    (out_dma == IDMA_PF_V_OUT) ? FS_PP_SRC_PF :	/*PF-> */
		    (out_dma == IDMA_IC_12) ? FS_PP_SRC_ROT :	/*PP_ROT-> */
		    0;
		break;

		/* ->PP_ROT */
	case IDMA_IC_13:
		pr_debug("PP_ROT\n");
		isProc = true;
		mask = FS_PP_ROT_SRC_SEL_MASK;
		offset = FS_PP_ROT_SRC_SEL_OFFSET;
		value = (out_dma == IDMA_PF_Y_OUT) ? FS_PP_SRC_PF :	/*PF-> */
		    (out_dma == IDMA_PF_U_OUT) ? FS_PP_SRC_PF :	/*PF-> */
		    (out_dma == IDMA_PF_V_OUT) ? FS_PP_SRC_PF :	/*PF-> */
		    (out_dma == IDMA_IC_2) ? FS_ROT_SRC_PP :	/*PP-> */
		    0;
		break;

		/* ->SDC_BG */
	case IDMA_SDC_BG:
		pr_debug("SDC_BG\n");
		isProc = false;
		mask = FS_SDC_BG_SRC_SEL_MASK;
		offset = FS_SDC_BG_SRC_SEL_OFFSET;
		value = (out_dma == IDMA_IC_9) ? FS_SRC_ROT_VF :	/*VF_ROT-> */
		    (out_dma == IDMA_IC_12) ? FS_SRC_ROT_PP :	/*PP_ROT-> */
		    (out_dma == IDMA_IC_1) ? FS_SRC_VF :	/*VF-> */
		    (out_dma == IDMA_IC_2) ? FS_SRC_PP :	/*PP-> */
		    0;
		break;

		/* ->SDC_FG */
	case IDMA_SDC_FG:
		pr_debug("SDC_FG\n");
		isProc = false;
		mask = FS_SDC_FG_SRC_SEL_MASK;
		offset = FS_SDC_FG_SRC_SEL_OFFSET;
		value = (out_dma == IDMA_IC_9) ? FS_SRC_ROT_VF :	/*VF_ROT-> */
		    (out_dma == IDMA_IC_12) ? FS_SRC_ROT_PP :	/*PP_ROT-> */
		    (out_dma == IDMA_IC_1) ? FS_SRC_VF :	/*VF-> */
		    (out_dma == IDMA_IC_2) ? FS_SRC_PP :	/*PP-> */
		    0;
		break;

		/* ->ADC1 */
	case IDMA_ADC_SYS1_WR:
		pr_debug("ADC_SYS1\n");
		isProc = false;
		mask = FS_ADC1_SRC_SEL_MASK;
		offset = FS_ADC1_SRC_SEL_OFFSET;
		value = (out_dma == IDMA_IC_9) ? FS_SRC_ROT_VF :	/*VF_ROT-> */
		    (out_dma == IDMA_IC_12) ? FS_SRC_ROT_PP :	/*PP_ROT-> */
		    (out_dma == IDMA_IC_1) ? FS_SRC_VF :	/*VF-> */
		    (out_dma == IDMA_IC_2) ? FS_SRC_PP :	/*PP-> */
		    0;
		break;

		/* ->ADC2 */
	case IDMA_ADC_SYS2_WR:
		pr_debug("ADC_SYS2\n");
		isProc = false;
		mask = FS_ADC2_SRC_SEL_MASK;
		offset = FS_ADC2_SRC_SEL_OFFSET;
		value = (out_dma == IDMA_IC_9) ? FS_SRC_ROT_VF :	/*VF_ROT-> */
		    (out_dma == IDMA_IC_12) ? FS_SRC_ROT_PP :	/*PP_ROT-> */
		    (out_dma == IDMA_IC_1) ? FS_SRC_VF :	/*VF-> */
		    (out_dma == IDMA_IC_2) ? FS_SRC_PP :	/*PP-> */
		    0;
		break;

		/*Invalid chains: */
		/* ->ENC, ->VF, ->PF, ->VF_COMBINE, ->PP_COMBINE */
	default:
		pr_debug("Invalid\n");
		value = 0;
		break;

	}

	if (value) {
		if (isProc) {
			fs_proc_flow &= ~mask;
			fs_proc_flow |= (value << offset);
		} else {
			fs_disp_flow &= ~mask;
			fs_disp_flow |= (value << offset);
		}
	} else {
		dev_err(g_ipu_dev, "Invalid channel chaining %d -> %d\n",
			out_dma, in_dma);
		return -EINVAL;
	}

	__raw_writel(fs_proc_flow, IPU_FS_PROC_FLOW);
	__raw_writel(fs_disp_flow, IPU_FS_DISP_FLOW);

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
	return 0;
}

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
	unsigned long lock_flags;
	uint32_t out_dma;
	uint32_t in_dma;
	uint32_t fs_proc_flow;
	uint32_t fs_disp_flow;

	spin_lock_irqsave(&ipu_lock, lock_flags);

	fs_proc_flow = __raw_readl(IPU_FS_PROC_FLOW);
	fs_disp_flow = __raw_readl(IPU_FS_DISP_FLOW);

	out_dma = (1UL << channel_2_dma(src_ch, IPU_OUTPUT_BUFFER));
	in_dma = (1UL << channel_2_dma(dest_ch, IPU_INPUT_BUFFER));

	/*clear the src_ch's output destination */
	switch (out_dma) {
		/*VF-> */
	case IDMA_IC_1:
		pr_debug("Unlink VF->");
		fs_proc_flow &= ~FS_PRPVF_DEST_SEL_MASK;
		break;

		/*VF_ROT-> */
	case IDMA_IC_9:
		pr_debug("Unlink VF_Rot->");
		fs_proc_flow &= ~FS_PRPVF_ROT_DEST_SEL_MASK;
		break;

		/*ENC-> */
	case IDMA_IC_0:
		pr_debug("Unlink ENC->");
		fs_proc_flow &= ~FS_PRPENC_DEST_SEL;
		break;

		/*PP-> */
	case IDMA_IC_2:
		pr_debug("Unlink PP->");
		fs_proc_flow &= ~FS_PP_DEST_SEL_MASK;
		break;

		/*PP_ROT-> */
	case IDMA_IC_12:
		pr_debug("Unlink PP_ROT->");
		fs_proc_flow &= ~FS_PP_ROT_DEST_SEL_MASK;
		break;

		/*PF-> */
	case IDMA_PF_Y_OUT:
	case IDMA_PF_U_OUT:
	case IDMA_PF_V_OUT:
		pr_debug("Unlink PF->");
		fs_proc_flow &= ~FS_PF_DEST_SEL_MASK;
		break;

	default:		/*ENC_ROT->     */
		pr_debug("Unlink Invalid->");
		break;
	}

	/*clear the dest_ch's input source */
	switch (in_dma) {
	/*->VF_ROT*/
	case IDMA_IC_11:
		pr_debug("VF_ROT\n");
		fs_proc_flow &= ~FS_PRPVF_ROT_SRC_SEL;
		break;

	/*->Enc_ROT*/
	case IDMA_IC_10:
		pr_debug("ENC_ROT\n");
		fs_proc_flow &= ~FS_PRPENC_ROT_SRC_SEL;
		break;

	/*->PP*/
	case IDMA_IC_5:
		pr_debug("PP\n");
		fs_proc_flow &= ~FS_PP_SRC_SEL_MASK;
		break;

	/*->PP_ROT*/
	case IDMA_IC_13:
		pr_debug("PP_ROT\n");
		fs_proc_flow &= ~FS_PP_ROT_SRC_SEL_MASK;
		break;

	/*->SDC_FG*/
	case IDMA_SDC_FG:
		pr_debug("SDC_FG\n");
		fs_disp_flow &= ~FS_SDC_FG_SRC_SEL_MASK;
		break;

	/*->SDC_BG*/
	case IDMA_SDC_BG:
		pr_debug("SDC_BG\n");
		fs_disp_flow &= ~FS_SDC_BG_SRC_SEL_MASK;
		break;

	/*->ADC1*/
	case IDMA_ADC_SYS1_WR:
		pr_debug("ADC_SYS1\n");
		fs_disp_flow &= ~FS_ADC1_SRC_SEL_MASK;
		break;

	/*->ADC2*/
	case IDMA_ADC_SYS2_WR:
		pr_debug("ADC_SYS2\n");
		fs_disp_flow &= ~FS_ADC2_SRC_SEL_MASK;
		break;

	default:		/*->VF, ->ENC, ->VF_COMBINE, ->PP_COMBINE, ->PF*/
		pr_debug("Invalid\n");
		break;
	}

	__raw_writel(fs_proc_flow, IPU_FS_PROC_FLOW);
	__raw_writel(fs_disp_flow, IPU_FS_DISP_FLOW);

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
	return 0;
}

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
	in_dma = channel_2_dma(channel, IPU_INPUT_BUFFER);

	reg = __raw_readl(IDMAC_CHA_EN);

	if (reg & ((1UL << out_dma) | (1UL << in_dma)))
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
	uint32_t in_dma;
	uint32_t sec_dma;
	uint32_t out_dma;
	uint32_t chan_mask = 0;

	spin_lock_irqsave(&ipu_lock, lock_flags);

	reg = __raw_readl(IDMAC_CHA_EN);

	/* Get input and output dma channels */
	out_dma = channel_2_dma(channel, IPU_OUTPUT_BUFFER);
	if (out_dma != IDMA_CHAN_INVALID)
		reg |= 1UL << out_dma;
	in_dma = channel_2_dma(channel, IPU_INPUT_BUFFER);
	if (in_dma != IDMA_CHAN_INVALID)
		reg |= 1UL << in_dma;

	/* Get secondary input dma channel */
	if (g_sec_chan_en[IPU_CHAN_ID(channel)]) {
		sec_dma = channel_2_dma(channel, IPU_SEC_INPUT_BUFFER);
		if (sec_dma != IDMA_CHAN_INVALID) {
			reg |= 1UL << sec_dma;
		}
	}

	__raw_writel(reg | chan_mask, IDMAC_CHA_EN);

	if (IPU_CHAN_ID(channel) <= IPU_CHAN_ID(MEM_PP_ADC)) {
		_ipu_ic_enable_task(channel);
	} else if (channel == MEM_SDC_BG) {
		dev_dbg(g_ipu_dev, "Initializing SDC BG\n");
		_ipu_sdc_bg_init(NULL);
	} else if (channel == MEM_SDC_FG) {
		dev_dbg(g_ipu_dev, "Initializing SDC FG\n");
		_ipu_sdc_fg_init(NULL);
	}

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
	return 0;
}

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
	/*TODO*/
}
EXPORT_SYMBOL(ipu_clear_buffer_ready);

uint32_t ipu_get_cur_buffer_idx(ipu_channel_t channel, ipu_buffer_t type)
{
	uint32_t reg, dma_chan;

	dma_chan = channel_2_dma(channel, type);

	reg = __raw_readl(IPU_CHA_CUR_BUF);
	if (reg & (1UL << dma_chan))
		return 1;
	else
		return 0;
}
EXPORT_SYMBOL(ipu_get_cur_buffer_idx);

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
	uint32_t sec_dma;
	uint32_t in_dma;
	uint32_t out_dma;
	uint32_t chan_mask = 0;
	uint32_t timeout;
	uint32_t eof_intr;
	uint32_t enabled;

	/* Get input and output dma channels */
	out_dma = channel_2_dma(channel, IPU_OUTPUT_BUFFER);
	if (out_dma != IDMA_CHAN_INVALID)
		chan_mask = 1UL << out_dma;
	in_dma = channel_2_dma(channel, IPU_INPUT_BUFFER);
	if (in_dma != IDMA_CHAN_INVALID)
		chan_mask |= 1UL << in_dma;
	sec_dma = channel_2_dma(channel, IPU_SEC_INPUT_BUFFER);
	if (sec_dma != IDMA_CHAN_INVALID)
		chan_mask |= 1UL << sec_dma;

	if (wait_for_stop && channel != MEM_SDC_FG && channel != MEM_SDC_BG) {
		timeout = 40;
		while ((__raw_readl(IDMAC_CHA_BUSY) & chan_mask) ||
		       (_ipu_channel_status(channel) == TASK_STAT_ACTIVE)) {
			timeout--;
			msleep(10);
			if (timeout == 0) {
				printk
				    (KERN_INFO
				     "MXC IPU: Warning - timeout waiting for channel to stop,\n"
				     "\tbuf0_rdy = 0x%08X, buf1_rdy = 0x%08X\n"
				     "\tbusy = 0x%08X, tstat = 0x%08X\n\tmask = 0x%08X\n",
				     __raw_readl(IPU_CHA_BUF0_RDY),
				     __raw_readl(IPU_CHA_BUF1_RDY),
				     __raw_readl(IDMAC_CHA_BUSY),
				     __raw_readl(IPU_TASKS_STAT), chan_mask);
				break;
			}
		}
		dev_dbg(g_ipu_dev, "timeout = %d * 10ms\n", 40 - timeout);
	}
	/* SDC BG and FG must be disabled before DMA is disabled */
	if ((channel == MEM_SDC_BG) || (channel == MEM_SDC_FG)) {

		if (channel == MEM_SDC_BG)
			eof_intr = IPU_IRQ_SDC_BG_EOF;
		else
			eof_intr = IPU_IRQ_SDC_FG_EOF;

		/* Wait for any buffer flips to finsh */
		timeout = 4;
		while (timeout &&
		       ((__raw_readl(IPU_CHA_BUF0_RDY) & chan_mask) ||
			(__raw_readl(IPU_CHA_BUF1_RDY) & chan_mask))) {
			msleep(10);
			timeout--;
		}

		spin_lock_irqsave(&ipu_lock, lock_flags);
		ipu_clear_irq(eof_intr);
		if (channel == MEM_SDC_BG)
			enabled = _ipu_sdc_bg_uninit();
		else
			enabled = _ipu_sdc_fg_uninit();
		spin_unlock_irqrestore(&ipu_lock, lock_flags);

		if (enabled && wait_for_stop) {
			timeout = 5;
		} else {
			timeout = 0;
		}
		while (timeout && !ipu_get_irq_status(eof_intr)) {
			msleep(5);
			timeout--;
		}
	}

	spin_lock_irqsave(&ipu_lock, lock_flags);

	/* Disable IC task */
	if (IPU_CHAN_ID(channel) <= IPU_CHAN_ID(MEM_PP_ADC)) {
		_ipu_ic_disable_task(channel);
	}

	/* Disable DMA channel(s) */
	reg = __raw_readl(IDMAC_CHA_EN);
	__raw_writel(reg & ~chan_mask, IDMAC_CHA_EN);

	/* Clear DMA related interrupts */
	__raw_writel(chan_mask, IPU_INT_STAT_1);
	__raw_writel(chan_mask, IPU_INT_STAT_2);
	__raw_writel(chan_mask, IPU_INT_STAT_4);

	spin_unlock_irqrestore(&ipu_lock, lock_flags);

	return 0;
}

int32_t ipu_enable_csi(uint32_t csi)
{
	return 0;
}


int32_t ipu_disable_csi(uint32_t csi)
{
	return 0;
}

static
irqreturn_t ipu_irq_handler(int irq, void *desc)
{
	uint32_t line_base = 0;
	uint32_t line;
	irqreturn_t result = IRQ_NONE;
	uint32_t int_stat;

	int_stat = __raw_readl(IPU_INT_STAT_1);
	int_stat &= __raw_readl(IPU_INT_CTRL_1);
	__raw_writel(int_stat, IPU_INT_STAT_1);
	while ((line = ffs(int_stat)) != 0) {
		int_stat &= ~(1UL << (line - 1));
		line += line_base - 1;
		result |=
		    ipu_irq_list[line].handler(line, ipu_irq_list[line].dev_id);
	}

	line_base = 32;
	int_stat = __raw_readl(IPU_INT_STAT_2);
	int_stat &= __raw_readl(IPU_INT_CTRL_2);
	__raw_writel(int_stat, IPU_INT_STAT_2);
	while ((line = ffs(int_stat)) != 0) {
		int_stat &= ~(1UL << (line - 1));
		line += line_base - 1;
		result |=
		    ipu_irq_list[line].handler(line, ipu_irq_list[line].dev_id);
	}

	line_base = 64;
	int_stat = __raw_readl(IPU_INT_STAT_3);
	int_stat &= __raw_readl(IPU_INT_CTRL_3);
	__raw_writel(int_stat, IPU_INT_STAT_3);
	while ((line = ffs(int_stat)) != 0) {
		int_stat &= ~(1UL << (line - 1));
		line += line_base - 1;
		result |=
		    ipu_irq_list[line].handler(line, ipu_irq_list[line].dev_id);
	}

	line_base = 96;
	int_stat = __raw_readl(IPU_INT_STAT_4);
	int_stat &= __raw_readl(IPU_INT_CTRL_4);
	__raw_writel(int_stat, IPU_INT_STAT_4);
	while ((line = ffs(int_stat)) != 0) {
		int_stat &= ~(1UL << (line - 1));
		line += line_base - 1;
		result |=
		    ipu_irq_list[line].handler(line, ipu_irq_list[line].dev_id);
	}

	line_base = 128;
	int_stat = __raw_readl(IPU_INT_STAT_5);
	int_stat &= __raw_readl(IPU_INT_CTRL_5);
	__raw_writel(int_stat, IPU_INT_STAT_5);
	while ((line = ffs(int_stat)) != 0) {
		int_stat &= ~(1UL << (line - 1));
		line += line_base - 1;
		result |=
		    ipu_irq_list[line].handler(line, ipu_irq_list[line].dev_id);
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

	spin_lock_irqsave(&ipu_lock, lock_flags);

	reg = __raw_readl(IPUIRQ_2_CTRLREG(irq));
	reg |= IPUIRQ_2_MASK(irq);
	__raw_writel(reg, IPUIRQ_2_CTRLREG(irq));

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
}

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

	spin_lock_irqsave(&ipu_lock, lock_flags);

	reg = __raw_readl(IPUIRQ_2_CTRLREG(irq));
	reg &= ~IPUIRQ_2_MASK(irq);
	__raw_writel(reg, IPUIRQ_2_CTRLREG(irq));

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
}

/*!
 * This function clears the interrupt for the specified interrupt line.
 * The interrupt lines are defined in \b ipu_irq_line enum.
 *
 * @param       irq             Interrupt line to clear interrupt for.
 *
 */
void ipu_clear_irq(uint32_t irq)
{
	__raw_writel(IPUIRQ_2_MASK(irq), IPUIRQ_2_STATREG(irq));
}

/*!
 * This function returns the current interrupt status for the specified interrupt
 * line. The interrupt lines are defined in \b ipu_irq_line enum.
 *
 * @param       irq             Interrupt line to get status for.
 *
 * @return      Returns true if the interrupt is pending/asserted or false if
 *              the interrupt is not pending.
 */
bool ipu_get_irq_status(uint32_t irq)
{
	uint32_t reg = __raw_readl(IPUIRQ_2_STATREG(irq));

	if (reg & IPUIRQ_2_MASK(irq))
		return true;
	else
		return false;
}

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
 * @param       dev_id          Input parameter for pointer of data to be passed
 *                              to the handler.
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
			"ipu_request_irq - handler already installed on irq %d\n",
			irq);
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

	if (ipu_irq_list[irq].dev_id == dev_id) {
		ipu_irq_list[irq].handler = NULL;
	}
}

/*!
 * This function sets the post-filter pause row for h.264 mode.
 *
 * @param       pause_row       The last row to process before pausing.
 *
 * @return      This function returns 0 on success or negative error code on
 *              fail.
 *
 */
int32_t ipu_pf_set_pause_row(uint32_t pause_row)
{
	int32_t retval = 0;
	uint32_t timeout = 5;
	unsigned long lock_flags;
	uint32_t reg;

	reg = __raw_readl(IPU_TASKS_STAT);
	while ((reg & TSTAT_PF_MASK) && ((reg & TSTAT_PF_H264_PAUSE) == 0)) {
		timeout--;
		msleep(5);
		if (timeout == 0) {
			dev_err(g_ipu_dev, "PF Timeout - tstat = 0x%08X\n",
				__raw_readl(IPU_TASKS_STAT));
			retval = -ETIMEDOUT;
			goto err0;
		}
	}

	spin_lock_irqsave(&ipu_lock, lock_flags);

	reg = __raw_readl(PF_CONF);

	/* Set the pause row */
	if (pause_row) {
		reg &= ~PF_CONF_PAUSE_ROW_MASK;
		reg |= PF_CONF_PAUSE_EN | pause_row << PF_CONF_PAUSE_ROW_SHIFT;
	} else {
		reg &= ~(PF_CONF_PAUSE_EN | PF_CONF_PAUSE_ROW_MASK);
	}
	__raw_writel(reg, PF_CONF);

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
      err0:
	return retval;
}

/* Private functions */
void _ipu_write_param_mem(uint32_t addr, uint32_t *data, uint32_t numWords)
{
	for (; numWords > 0; numWords--) {
		dev_dbg(g_ipu_dev,
			"write param mem - addr = 0x%08X, data = 0x%08X\n",
			addr, *data);
		__raw_writel(addr, IPU_IMA_ADDR);
		__raw_writel(*data++, IPU_IMA_DATA);
		addr++;
		if ((addr & 0x7) == 5) {
			addr &= ~0x7;	/* set to word 0 */
			addr += 8;	/* increment to next row */
		}
	}
}

static void _ipu_pf_init(ipu_channel_params_t *params)
{
	uint32_t reg;

	/*Setup the type of filtering required */
	switch (params->mem_pf_mem.operation) {
	case PF_MPEG4_DEBLOCK:
	case PF_MPEG4_DERING:
	case PF_MPEG4_DEBLOCK_DERING:
		g_sec_chan_en[IPU_CHAN_ID(MEM_PF_Y_MEM)] = true;
		g_sec_chan_en[IPU_CHAN_ID(MEM_PF_U_MEM)] = false;
		break;
	case PF_H264_DEBLOCK:
		g_sec_chan_en[IPU_CHAN_ID(MEM_PF_Y_MEM)] = true;
		g_sec_chan_en[IPU_CHAN_ID(MEM_PF_U_MEM)] = true;
		break;
	default:
		g_sec_chan_en[IPU_CHAN_ID(MEM_PF_Y_MEM)] = false;
		g_sec_chan_en[IPU_CHAN_ID(MEM_PF_U_MEM)] = false;
		return;
		break;
	}
	reg = params->mem_pf_mem.operation;
	__raw_writel(reg, PF_CONF);
}

static void _ipu_pf_uninit(void)
{
	__raw_writel(0x0L, PF_CONF);
	g_sec_chan_en[IPU_CHAN_ID(MEM_PF_Y_MEM)] = false;
	g_sec_chan_en[IPU_CHAN_ID(MEM_PF_U_MEM)] = false;
}

uint32_t _ipu_channel_status(ipu_channel_t channel)
{
	uint32_t stat = 0;
	uint32_t task_stat_reg = __raw_readl(IPU_TASKS_STAT);

	switch (channel) {
	case CSI_MEM:
		stat =
		    (task_stat_reg & TSTAT_CSI2MEM_MASK) >>
		    TSTAT_CSI2MEM_OFFSET;
		break;
	case CSI_PRP_VF_ADC:
	case CSI_PRP_VF_MEM:
	case MEM_PRP_VF_ADC:
	case MEM_PRP_VF_MEM:
		stat = (task_stat_reg & TSTAT_VF_MASK) >> TSTAT_VF_OFFSET;
		break;
	case MEM_ROT_VF_MEM:
		stat =
		    (task_stat_reg & TSTAT_VF_ROT_MASK) >> TSTAT_VF_ROT_OFFSET;
		break;
	case CSI_PRP_ENC_MEM:
	case MEM_PRP_ENC_MEM:
		stat = (task_stat_reg & TSTAT_ENC_MASK) >> TSTAT_ENC_OFFSET;
		break;
	case MEM_ROT_ENC_MEM:
		stat =
		    (task_stat_reg & TSTAT_ENC_ROT_MASK) >>
		    TSTAT_ENC_ROT_OFFSET;
		break;
	case MEM_PP_ADC:
	case MEM_PP_MEM:
		stat = (task_stat_reg & TSTAT_PP_MASK) >> TSTAT_PP_OFFSET;
		break;
	case MEM_ROT_PP_MEM:
		stat =
		    (task_stat_reg & TSTAT_PP_ROT_MASK) >> TSTAT_PP_ROT_OFFSET;
		break;

	case MEM_PF_Y_MEM:
	case MEM_PF_U_MEM:
	case MEM_PF_V_MEM:
		stat = (task_stat_reg & TSTAT_PF_MASK) >> TSTAT_PF_OFFSET;
		break;
	case MEM_SDC_BG:
		break;
	case MEM_SDC_FG:
		break;
	case ADC_SYS1:
		stat =
		    (task_stat_reg & TSTAT_ADCSYS1_MASK) >>
		    TSTAT_ADCSYS1_OFFSET;
		break;
	case ADC_SYS2:
		stat =
		    (task_stat_reg & TSTAT_ADCSYS2_MASK) >>
		    TSTAT_ADCSYS2_OFFSET;
		break;
	case MEM_SDC_MASK:
	default:
		stat = TASK_STAT_IDLE;
		break;
	}
	return stat;
}

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
	case IPU_PIX_FMT_RGB32:
	case IPU_PIX_FMT_ABGR32:
		return 4;
		break;
	default:
		return 1;
		break;
	}
	return 0;
}

void ipu_set_csc_coefficients(ipu_channel_t channel, int32_t param[][3])
{
	/* TODO */
}
EXPORT_SYMBOL(ipu_set_csc_coefficients);

ipu_color_space_t format_to_colorspace(uint32_t fmt)
{
	switch (fmt) {
	case IPU_PIX_FMT_RGB565:
	case IPU_PIX_FMT_BGR24:
	case IPU_PIX_FMT_RGB24:
	case IPU_PIX_FMT_BGR32:
	case IPU_PIX_FMT_RGB32:
		return RGB;
		break;

	default:
		return YCbCr;
		break;
	}
	return RGB;

}

static u32 saved_disp3_time_conf;
static bool in_lpdr_mode;
static struct clk *default_ipu_parent_clk;

int ipu_lowpwr_display_enable(void)
{
	unsigned long rate, div;
	struct clk *parent_clk = g_ipu_clk;

	if (in_lpdr_mode || IS_ERR(dfm_clk)) {
		return -EINVAL;
	}

	if (g_channel_init_mask != (1L << IPU_CHAN_ID(MEM_SDC_BG))) {
		dev_err(g_ipu_dev, "LPDR mode requires only SDC BG active.\n");
		return -EINVAL;
	}

	default_ipu_parent_clk = clk_get_parent(g_ipu_clk);
	in_lpdr_mode = true;

	/* Calculate current pixel clock rate */
	rate = clk_get_rate(g_ipu_clk) * 16;
	saved_disp3_time_conf = __raw_readl(DI_DISP3_TIME_CONF);
	div = saved_disp3_time_conf & 0xFFF;
	rate /= div;
	rate *= 4;		/* min hsp clk is 4x pixel clk */

	/* Initialize DFM clock */
	rate = clk_round_rate(dfm_clk, rate);
	clk_set_rate(dfm_clk, rate);
	clk_enable(dfm_clk);

	/* Wait for next VSYNC */
	__raw_writel(IPUIRQ_2_MASK(IPU_IRQ_SDC_DISP3_VSYNC),
		     IPUIRQ_2_STATREG(IPU_IRQ_SDC_DISP3_VSYNC));
	while ((__raw_readl(IPUIRQ_2_STATREG(IPU_IRQ_SDC_DISP3_VSYNC)) &
		IPUIRQ_2_MASK(IPU_IRQ_SDC_DISP3_VSYNC)) == 0)
		msleep_interruptible(1);

	/* Set display clock divider to divide by 4 */
	__raw_writel(((0x8) << 22) | 0x40, DI_DISP3_TIME_CONF);

	clk_set_parent(parent_clk, dfm_clk);

	return 0;
}

int ipu_lowpwr_display_disable(void)
{
	struct clk *parent_clk = g_ipu_clk;

	if (!in_lpdr_mode || IS_ERR(dfm_clk)) {
		return -EINVAL;
	}

	if (g_channel_init_mask != (1L << IPU_CHAN_ID(MEM_SDC_BG))) {
		dev_err(g_ipu_dev, "LPDR mode requires only SDC BG active.\n");
		return -EINVAL;
	}

	in_lpdr_mode = false;

	/* Wait for next VSYNC */
	__raw_writel(IPUIRQ_2_MASK(IPU_IRQ_SDC_DISP3_VSYNC),
		     IPUIRQ_2_STATREG(IPU_IRQ_SDC_DISP3_VSYNC));
	while ((__raw_readl(IPUIRQ_2_STATREG(IPU_IRQ_SDC_DISP3_VSYNC)) &
		IPUIRQ_2_MASK(IPU_IRQ_SDC_DISP3_VSYNC)) == 0)
		msleep_interruptible(1);

	__raw_writel(saved_disp3_time_conf, DI_DISP3_TIME_CONF);
	clk_set_parent(parent_clk, default_ipu_parent_clk);
	clk_disable(dfm_clk);

	return 0;
}

static int ipu_suspend(struct platform_device *pdev, pm_message_t state)
{
	if (cpu_is_mx31() || cpu_is_mx32()) {
		/* work-around for i.Mx31 SR mode after camera related test */
		if (g_csi_used) {
			g_ipu_config = __raw_readl(IPU_CONF);
			clk_enable(g_ipu_csi_clk);
			__raw_writel(0x51, IPU_CONF);
			g_channel_init_mask_backup = g_channel_init_mask;
			g_channel_init_mask |= 2;
		}
	} else if (cpu_is_mx35()) {
		g_ipu_config = __raw_readl(IPU_CONF);
		/* Disable the clock of display otherwise the backlight cannot
		 * be close after camera/tvin related test */
		__raw_writel(g_ipu_config & 0xfbf, IPU_CONF);
	}

	return 0;
}

static int ipu_resume(struct platform_device *pdev)
{
	if (cpu_is_mx31() || cpu_is_mx32()) {
		/* work-around for i.Mx31 SR mode after camera related test */
		if (g_csi_used) {
			__raw_writel(g_ipu_config, IPU_CONF);
			clk_disable(g_ipu_csi_clk);
			g_channel_init_mask = g_channel_init_mask_backup;
		}
	} else if (cpu_is_mx35()) {
		__raw_writel(g_ipu_config, IPU_CONF);
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
	if (g_ipu_irq[0])
		free_irq(g_ipu_irq[0], 0);
	if (g_ipu_irq[1])
		free_irq(g_ipu_irq[1], 0);

	platform_driver_unregister(&mxcipu_driver);
}

module_exit(ipu_gen_uninit);

EXPORT_SYMBOL(ipu_init_channel);
EXPORT_SYMBOL(ipu_uninit_channel);
EXPORT_SYMBOL(ipu_init_channel_buffer);
EXPORT_SYMBOL(ipu_unlink_channels);
EXPORT_SYMBOL(ipu_update_channel_buffer);
EXPORT_SYMBOL(ipu_select_buffer);
EXPORT_SYMBOL(ipu_link_channels);
EXPORT_SYMBOL(ipu_enable_channel);
EXPORT_SYMBOL(ipu_disable_channel);
EXPORT_SYMBOL(ipu_enable_csi);
EXPORT_SYMBOL(ipu_disable_csi);
EXPORT_SYMBOL(ipu_enable_irq);
EXPORT_SYMBOL(ipu_disable_irq);
EXPORT_SYMBOL(ipu_clear_irq);
EXPORT_SYMBOL(ipu_get_irq_status);
EXPORT_SYMBOL(ipu_request_irq);
EXPORT_SYMBOL(ipu_free_irq);
EXPORT_SYMBOL(ipu_pf_set_pause_row);
EXPORT_SYMBOL(bytes_per_pixel);
