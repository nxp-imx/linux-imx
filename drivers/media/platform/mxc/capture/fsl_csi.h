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
 * @file fsl_csi.h
 *
 * @brief mx25 CMOS Sensor interface functions
 *
 * @ingroup CSI
 */

#ifndef MX25_CSI_H
#define MX25_CSI_H

#include <linux/io.h>

/* reset values */
#define CSICR1_RESET_VAL	0x40000800
#define CSICR2_RESET_VAL	0x0
#define CSICR3_RESET_VAL	0x0

/* csi control reg 1 */
#define BIT_SWAP16_EN		(0x1 << 31)
#define BIT_EXT_VSYNC		(0x1 << 30)
#define BIT_EOF_INT_EN		(0x1 << 29)
#define BIT_PRP_IF_EN		(0x1 << 28)
#define BIT_CCIR_MODE		(0x1 << 27)
#define BIT_COF_INT_EN		(0x1 << 26)
#define BIT_SF_OR_INTEN		(0x1 << 25)
#define BIT_RF_OR_INTEN		(0x1 << 24)
#define BIT_SFF_DMA_DONE_INTEN  (0x1 << 22)
#define BIT_STATFF_INTEN	(0x1 << 21)
#define BIT_FB2_DMA_DONE_INTEN  (0x1 << 20)
#define BIT_FB1_DMA_DONE_INTEN  (0x1 << 19)
#define BIT_RXFF_INTEN		(0x1 << 18)
#define BIT_SOF_POL		(0x1 << 17)
#define BIT_SOF_INTEN		(0x1 << 16)
#define BIT_MCLKDIV		(0xF << 12)
#define BIT_HSYNC_POL		(0x1 << 11)
#define BIT_CCIR_EN		(0x1 << 10)
#define BIT_MCLKEN		(0x1 << 9)
#define BIT_FCC			(0x1 << 8)
#define BIT_PACK_DIR		(0x1 << 7)
#define BIT_CLR_STATFIFO	(0x1 << 6)
#define BIT_CLR_RXFIFO		(0x1 << 5)
#define BIT_GCLK_MODE		(0x1 << 4)
#define BIT_INV_DATA		(0x1 << 3)
#define BIT_INV_PCLK		(0x1 << 2)
#define BIT_REDGE		(0x1 << 1)
#define BIT_PIXEL_BIT		(0x1 << 0)

#define SHIFT_MCLKDIV		12

/* control reg 3 */
#define BIT_FRMCNT		(0xFFFF << 16)
#define BIT_FRMCNT_RST		(0x1 << 15)
#define BIT_DMA_REFLASH_RFF	(0x1 << 14)
#define BIT_DMA_REFLASH_SFF	(0x1 << 13)
#define BIT_DMA_REQ_EN_RFF	(0x1 << 12)
#define BIT_DMA_REQ_EN_SFF	(0x1 << 11)
#define BIT_STATFF_LEVEL	(0x7 << 8)
#define BIT_HRESP_ERR_EN	(0x1 << 7)
#define BIT_RXFF_LEVEL		(0x7 << 4)
#define BIT_TWO_8BIT_SENSOR	(0x1 << 3)
#define BIT_ZERO_PACK_EN	(0x1 << 2)
#define BIT_ECC_INT_EN		(0x1 << 1)
#define BIT_ECC_AUTO_EN		(0x1 << 0)

#define SHIFT_FRMCNT		16

/* csi status reg */
#define BIT_SFF_OR_INT		(0x1 << 25)
#define BIT_RFF_OR_INT		(0x1 << 24)
#define BIT_DMA_TSF_DONE_SFF	(0x1 << 22)
#define BIT_STATFF_INT		(0x1 << 21)
#define BIT_DMA_TSF_DONE_FB2	(0x1 << 20)
#define BIT_DMA_TSF_DONE_FB1	(0x1 << 19)
#define BIT_RXFF_INT		(0x1 << 18)
#define BIT_EOF_INT		(0x1 << 17)
#define BIT_SOF_INT		(0x1 << 16)
#define BIT_F2_INT		(0x1 << 15)
#define BIT_F1_INT		(0x1 << 14)
#define BIT_COF_INT		(0x1 << 13)
#define BIT_HRESP_ERR_INT	(0x1 << 7)
#define BIT_ECC_INT		(0x1 << 1)
#define BIT_DRDY		(0x1 << 0)

/* csi control reg 18 */
#define BIT_CSI_ENABLE			(0x1 << 31)
#define BIT_BASEADDR_SWITCH_SEL	(0x1 << 5)
#define BIT_BASEADDR_SWITCH_EN	(0x1 << 4)
#define BIT_PARALLEL24_EN		(0x1 << 3)
#define BIT_DEINTERLACE_EN		(0x1 << 2)
#define BIT_TVDECODER_IN_EN		(0x1 << 1)
#define BIT_NTSC_EN				(0x1 << 0)

#define CSI_MCLK_VF		1
#define CSI_MCLK_ENC		2
#define CSI_MCLK_RAW		4
#define CSI_MCLK_I2C		8
#endif

extern void __iomem *csi_regbase;
#define CSI_CSICR1		(csi_regbase)
#define CSI_CSICR2		(csi_regbase + 0x4)
#define CSI_CSICR3		(csi_regbase + 0x8)
#define CSI_STATFIFO		(csi_regbase + 0xC)
#define CSI_CSIRXFIFO		(csi_regbase + 0x10)
#define CSI_CSIRXCNT		(csi_regbase + 0x14)
#define CSI_CSISR		(csi_regbase + 0x18)

#define CSI_CSIDBG		(csi_regbase + 0x1C)
#define CSI_CSIDMASA_STATFIFO	(csi_regbase + 0x20)
#define CSI_CSIDMATS_STATFIFO	(csi_regbase + 0x24)
#define CSI_CSIDMASA_FB1	(csi_regbase + 0x28)
#define CSI_CSIDMASA_FB2	(csi_regbase + 0x2C)
#define CSI_CSIFBUF_PARA	(csi_regbase + 0x30)
#define CSI_CSIIMAG_PARA	(csi_regbase + 0x34)

#define CSI_CSICR18		(csi_regbase + 0x48)
#define CSI_CSICR19		(csi_regbase + 0x4c)

static inline void csi_clear_status(unsigned long status)
{
	__raw_writel(status, CSI_CSISR);
}

struct csi_signal_cfg_t {
	unsigned data_width:3;
	unsigned clk_mode:2;
	unsigned ext_vsync:1;
	unsigned Vsync_pol:1;
	unsigned Hsync_pol:1;
	unsigned pixclk_pol:1;
	unsigned data_pol:1;
	unsigned sens_clksrc:1;
};

struct csi_config_t {
	/* control reg 1 */
	unsigned int swap16_en:1;
	unsigned int ext_vsync:1;
	unsigned int eof_int_en:1;
	unsigned int prp_if_en:1;
	unsigned int ccir_mode:1;
	unsigned int cof_int_en:1;
	unsigned int sf_or_inten:1;
	unsigned int rf_or_inten:1;
	unsigned int sff_dma_done_inten:1;
	unsigned int statff_inten:1;
	unsigned int fb2_dma_done_inten:1;
	unsigned int fb1_dma_done_inten:1;
	unsigned int rxff_inten:1;
	unsigned int sof_pol:1;
	unsigned int sof_inten:1;
	unsigned int mclkdiv:4;
	unsigned int hsync_pol:1;
	unsigned int ccir_en:1;
	unsigned int mclken:1;
	unsigned int fcc:1;
	unsigned int pack_dir:1;
	unsigned int gclk_mode:1;
	unsigned int inv_data:1;
	unsigned int inv_pclk:1;
	unsigned int redge:1;
	unsigned int pixel_bit:1;

	/* control reg 3 */
	unsigned int frmcnt:16;
	unsigned int frame_reset:1;
	unsigned int dma_reflash_rff:1;
	unsigned int dma_reflash_sff:1;
	unsigned int dma_req_en_rff:1;
	unsigned int dma_req_en_sff:1;
	unsigned int statff_level:3;
	unsigned int hresp_err_en:1;
	unsigned int rxff_level:3;
	unsigned int two_8bit_sensor:1;
	unsigned int zero_pack_en:1;
	unsigned int ecc_int_en:1;
	unsigned int ecc_auto_en:1;
	/* fifo counter */
	unsigned int rxcnt;
};

typedef void (*csi_irq_callback_t) (void *data, unsigned long status);

void csi_init_interface(void);
void csi_set_32bit_imagpara(int width, int height);
void csi_set_16bit_imagpara(int width, int height);
void csi_set_12bit_imagpara(int width, int height);
void csi_format_swap16(bool enable);
int csi_read_mclk_flag(void);
void csi_start_callback(void *data);
void csi_stop_callback(void *data);
void csi_enable_int(int arg);
void csi_buf_stride_set(u32 stride);
void csi_deinterlace_mode(int mode);
void csi_deinterlace_enable(bool enable);
void csi_tvdec_enable(bool enable);
void csi_enable(int arg);
void csi_disable_int(void);
void csi_clk_enable(void);
void csi_clk_disable(void);
void csi_dmareq_rff_enable(void);
void csi_dmareq_rff_disable(void);
