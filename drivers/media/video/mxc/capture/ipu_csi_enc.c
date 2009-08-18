/*
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file csi_enc.c
 *
 * @brief CSI Use case for video capture
 *
 * @ingroup IPU
 */

#include <linux/dma-mapping.h>
#include <linux/ipu.h>
#include "mxc_v4l2_capture.h"
#include "ipu_prp_sw.h"

#ifdef CAMERA_DBG
	#define CAMERA_TRACE(x) (printk)x
#else
	#define CAMERA_TRACE(x)
#endif

/*
 * Function definitions
 */

/*!
 * csi ENC callback function.
 *
 * @param irq       int irq line
 * @param dev_id    void * device id
 *
 * @return status   IRQ_HANDLED for handled
 */
static irqreturn_t csi_enc_callback(int irq, void *dev_id)
{
	cam_data *cam = (cam_data *) dev_id;

	if (cam->enc_callback == NULL)
		return IRQ_HANDLED;

	cam->enc_callback(irq, dev_id);
	return IRQ_HANDLED;
}

/*!
 * CSI ENC enable channel setup function
 *
 * @param cam       struct cam_data * mxc capture instance
 *
 * @return  status
 */
static int csi_enc_setup(cam_data *cam)
{
	ipu_channel_params_t params;
	u32 pixel_fmt;
	int err = 0;
	dma_addr_t dummy = 0xdeadbeaf;

	CAMERA_TRACE("In csi_enc_setup\n");
	if (!cam) {
		printk(KERN_ERR "cam private is NULL\n");
		return -ENXIO;
	}

	memset(&params, 0, sizeof(ipu_channel_params_t));
	params.csi_mem.csi = cam->csi;

	if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420)
		pixel_fmt = IPU_PIX_FMT_YUV420P;
	else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV422P)
		pixel_fmt = IPU_PIX_FMT_YUV422P;
	else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_UYVY)
		pixel_fmt = IPU_PIX_FMT_UYVY;
	else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_NV12)
		pixel_fmt = IPU_PIX_FMT_NV12;
	else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_BGR24)
		pixel_fmt = IPU_PIX_FMT_BGR24;
	else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24)
		pixel_fmt = IPU_PIX_FMT_RGB24;
	else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB565)
		pixel_fmt = IPU_PIX_FMT_RGB565;
	else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_BGR32)
		pixel_fmt = IPU_PIX_FMT_BGR32;
	else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB32)
		pixel_fmt = IPU_PIX_FMT_RGB32;
	else {
		printk(KERN_ERR "format not supported\n");
		return -EINVAL;
	}

	ipu_csi_enable_mclk_if(CSI_MCLK_ENC, cam->csi, true, true);

	err = ipu_init_channel(CSI_MEM, &params);
	if (err != 0) {
		printk(KERN_ERR "ipu_init_channel %d\n", err);
		return err;
	}

	err = ipu_init_channel_buffer(CSI_MEM, IPU_OUTPUT_BUFFER,
				      pixel_fmt, cam->v2f.fmt.pix.width,
				      cam->v2f.fmt.pix.height,
				      cam->v2f.fmt.pix.width, IPU_ROTATE_NONE,
				      dummy, dummy,
				      cam->offset.u_offset,
				      cam->offset.v_offset);
	if (err != 0) {
		printk(KERN_ERR "CSI_MEM output buffer\n");
		return err;
	}
	err = ipu_enable_channel(CSI_MEM);
	if (err < 0) {
		printk(KERN_ERR "ipu_enable_channel CSI_MEM\n");
		return err;
	}

	return err;
}

/*!
 * function to update physical buffer address for encorder IDMA channel
 *
 * @param eba         physical buffer address for encorder IDMA channel
 * @param buffer_num  int buffer 0 or buffer 1
 *
 * @return  status
 */
static int csi_enc_eba_update(dma_addr_t eba, int *buffer_num)
{
	int err = 0;

	pr_debug("eba %x\n", eba);
	err = ipu_update_channel_buffer(CSI_MEM, IPU_OUTPUT_BUFFER,
					*buffer_num, eba);
	if (err != 0) {
		printk(KERN_ERR "err %d buffer_num %d\n", err, *buffer_num);
		return err;
	}

	ipu_select_buffer(CSI_MEM, IPU_OUTPUT_BUFFER, *buffer_num);

	*buffer_num = (*buffer_num == 0) ? 1 : 0;

	return 0;
}

/*!
 * Enable encoder task
 * @param private       struct cam_data * mxc capture instance
 *
 * @return  status
 */
static int csi_enc_enabling_tasks(void *private)
{
	cam_data *cam = (cam_data *) private;
	int err = 0;
	CAMERA_TRACE("IPU:In csi_enc_enabling_tasks\n");

	err = ipu_request_irq(IPU_IRQ_CSI0_OUT_EOF,
			      csi_enc_callback, 0, "Mxc Camera", cam);
	if (err != 0) {
		printk(KERN_ERR "Error registering rot irq\n");
		return err;
	}

	err = csi_enc_setup(cam);
	if (err != 0) {
		printk(KERN_ERR "csi_enc_setup %d\n", err);
		return err;
	}

	return err;
}

/*!
 * Disable encoder task
 * @param private       struct cam_data * mxc capture instance
 *
 * @return  int
 */
static int csi_enc_disabling_tasks(void *private)
{
	cam_data *cam = (cam_data *) private;
	int err = 0;

	ipu_free_irq(IPU_IRQ_CSI0_OUT_EOF, cam);

	err = ipu_disable_channel(CSI_MEM, true);

	ipu_uninit_channel(CSI_MEM);

	ipu_csi_enable_mclk_if(CSI_MCLK_ENC, cam->csi, false, false);

	return err;
}

/*!
 * function to select CSI ENC as the working path
 *
 * @param private       struct cam_data * mxc capture instance
 *
 * @return  int
 */
int csi_enc_select(void *private)
{
	cam_data *cam = (cam_data *) private;
	int err = 0;

	if (cam) {
		cam->enc_update_eba = csi_enc_eba_update;
		cam->enc_enable = csi_enc_enabling_tasks;
		cam->enc_disable = csi_enc_disabling_tasks;
	} else {
		err = -EIO;
	}

	return err;
}

/*!
 * function to de-select CSI ENC as the working path
 *
 * @param private       struct cam_data * mxc capture instance
 *
 * @return  int
 */
int csi_enc_deselect(void *private)
{
	cam_data *cam = (cam_data *) private;
	int err = 0;

	if (cam) {
		cam->enc_update_eba = NULL;
		cam->enc_enable = NULL;
		cam->enc_disable = NULL;
	}

	return err;
}

/*!
 * Init the Encorder channels
 *
 * @return  Error code indicating success or failure
 */
__init int csi_enc_init(void)
{
	return 0;
}

/*!
 * Deinit the Encorder channels
 *
 */
void __exit csi_enc_exit(void)
{
}

module_init(csi_enc_init);
module_exit(csi_enc_exit);

EXPORT_SYMBOL(csi_enc_select);
EXPORT_SYMBOL(csi_enc_deselect);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("CSI ENC Driver");
MODULE_LICENSE("GPL");
