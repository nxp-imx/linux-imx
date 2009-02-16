/*
 * Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file ipu_prp_vf_sdc.c
 *
 * @brief IPU Use case for PRP-VF
 *
 * @ingroup IPU
 */

#include <linux/dma-mapping.h>
#include <linux/ipu.h>
#include "mxc_v4l2_capture.h"
#include "ipu_prp_sw.h"

/*
 * Function definitions
 */

/*!
 * prpvf_start - start the vf task
 *
 * @param private    cam_data * mxc v4l2 main structure
 *
 */
static int prpvf_start(void *private)
{
	cam_data *cam = (cam_data *) private;
	ipu_channel_params_t vf, params;
	u32 format = IPU_PIX_FMT_RGB565;
	u32 size = 2;
	int err = 0;

	if (!cam) {
		printk(KERN_ERR "private is NULL\n");
		return -EIO;
	}

	if (cam->overlay_active == true) {
		pr_debug("already started.\n");
		return 0;
	}

	memset(&vf, 0, sizeof(ipu_channel_params_t));
	ipu_csi_get_window_size(&vf.csi_prp_vf_mem.in_width,
				&vf.csi_prp_vf_mem.in_height, cam->csi);
	vf.csi_prp_vf_mem.in_pixel_fmt = IPU_PIX_FMT_UYVY;
	vf.csi_prp_vf_mem.out_width = cam->win.w.width;
	vf.csi_prp_vf_mem.out_height = cam->win.w.height;
	vf.csi_prp_vf_mem.csi = cam->csi;
	if (cam->rotation >= IPU_ROTATE_90_RIGHT) {
		vf.csi_prp_vf_mem.out_width = cam->win.w.height;
		vf.csi_prp_vf_mem.out_height = cam->win.w.width;
	}
	vf.csi_prp_vf_mem.out_pixel_fmt = format;
	size = cam->win.w.width * cam->win.w.height * size;

	err = ipu_init_channel(CSI_PRP_VF_MEM, &vf);
	if (err != 0)
		goto out_4;

	ipu_csi_enable_mclk_if(CSI_MCLK_VF, cam->csi, true, true);

	if (cam->vf_bufs_vaddr[0]) {
		dma_free_coherent(0, cam->vf_bufs_size[0],
				  cam->vf_bufs_vaddr[0],
				  (dma_addr_t) cam->vf_bufs[0]);
	}
	if (cam->vf_bufs_vaddr[1]) {
		dma_free_coherent(0, cam->vf_bufs_size[1],
				  cam->vf_bufs_vaddr[1],
				  (dma_addr_t) cam->vf_bufs[1]);
	}
	cam->vf_bufs_size[0] = PAGE_ALIGN(size);
	cam->vf_bufs_vaddr[0] = (void *)dma_alloc_coherent(0,
							   cam->vf_bufs_size[0],
							   (dma_addr_t *) &
							   cam->vf_bufs[0],
							   GFP_DMA |
							   GFP_KERNEL);
	if (cam->vf_bufs_vaddr[0] == NULL) {
		printk(KERN_ERR "Error to allocate vf buffer\n");
		err = -ENOMEM;
		goto out_3;
	}
	cam->vf_bufs_size[1] = PAGE_ALIGN(size);
	cam->vf_bufs_vaddr[1] = (void *)dma_alloc_coherent(0,
							   cam->vf_bufs_size[1],
							   (dma_addr_t *) &
							   cam->vf_bufs[1],
							   GFP_DMA |
							   GFP_KERNEL);
	if (cam->vf_bufs_vaddr[1] == NULL) {
		printk(KERN_ERR "Error to allocate vf buffer\n");
		err = -ENOMEM;
		goto out_3;
	}
	pr_debug("vf_bufs %x %x\n", cam->vf_bufs[0], cam->vf_bufs[1]);

	if (cam->rotation >= IPU_ROTATE_90_RIGHT) {
		err = ipu_init_channel_buffer(CSI_PRP_VF_MEM, IPU_OUTPUT_BUFFER,
					      format,
					      vf.csi_prp_vf_mem.out_width,
					      vf.csi_prp_vf_mem.out_height,
					      vf.csi_prp_vf_mem.out_width,
					      IPU_ROTATE_NONE, cam->vf_bufs[0],
					      cam->vf_bufs[1], 0, 0);
		if (err != 0) {
			goto out_3;
		}

		if (cam->rot_vf_bufs[0]) {
			dma_free_coherent(0, cam->rot_vf_buf_size[0],
					  cam->rot_vf_bufs_vaddr[0],
					  (dma_addr_t) cam->rot_vf_bufs[0]);
			cam->rot_vf_bufs_vaddr[0] = NULL;
			cam->rot_vf_bufs[0] = 0;
		}
		if (cam->rot_vf_bufs[1]) {
			dma_free_coherent(0, cam->rot_vf_buf_size[1],
					  cam->rot_vf_bufs_vaddr[1],
					  (dma_addr_t) cam->rot_vf_bufs[1]);
			cam->rot_vf_bufs_vaddr[1] = NULL;
			cam->rot_vf_bufs[1] = 0;
		}
		cam->rot_vf_buf_size[0] = PAGE_ALIGN(size);
		cam->rot_vf_bufs_vaddr[0] = (void *)dma_alloc_coherent(0,
								       cam->
								       rot_vf_buf_size
								       [0],
								       &cam->
								       rot_vf_bufs
								       [0],
								       GFP_DMA |
								       GFP_KERNEL);
		if (cam->rot_vf_bufs_vaddr[0] == NULL) {
			printk(KERN_ERR "alloc rot_vf_bufs.\n");
			err = -ENOMEM;
			goto out_3;
		}
		cam->rot_vf_buf_size[1] = PAGE_ALIGN(size);
		cam->rot_vf_bufs_vaddr[1] = (void *)dma_alloc_coherent(0,
								       cam->
								       rot_vf_buf_size
								       [0],
								       &cam->
								       rot_vf_bufs
								       [1],
								       GFP_DMA |
								       GFP_KERNEL);
		if (cam->rot_vf_bufs_vaddr[1] == NULL) {
			printk(KERN_ERR "alloc rot_vf_bufs.\n");
			err = -ENOMEM;
			goto out_3;
		}
		pr_debug("rot_vf_bufs %x %x\n", cam->rot_vf_bufs[0],
			 cam->rot_vf_bufs[1]);

		err = ipu_init_channel(MEM_ROT_VF_MEM, NULL);
		if (err != 0) {
			printk(KERN_ERR "Error MEM_ROT_VF_MEM channel\n");
			goto out_3;
		}

		err = ipu_init_channel_buffer(MEM_ROT_VF_MEM, IPU_INPUT_BUFFER,
					      format,
					      vf.csi_prp_vf_mem.out_width,
					      vf.csi_prp_vf_mem.out_height,
					      vf.csi_prp_vf_mem.out_width,
					      cam->rotation, cam->vf_bufs[0],
					      cam->vf_bufs[1], 0, 0);
		if (err != 0) {
			printk(KERN_ERR "Error MEM_ROT_VF_MEM input buffer\n");
			goto out_3;
		}

		err = ipu_init_channel_buffer(MEM_ROT_VF_MEM, IPU_OUTPUT_BUFFER,
					      format,
					      vf.csi_prp_vf_mem.out_height,
					      vf.csi_prp_vf_mem.out_width,
					      vf.csi_prp_vf_mem.out_height,
					      IPU_ROTATE_NONE,
					      cam->rot_vf_bufs[0],
					      cam->rot_vf_bufs[1], 0, 0);
		if (err != 0) {
			printk(KERN_ERR "Error MEM_ROT_VF_MEM output buffer\n");
			goto out_2;
		}

		err = ipu_link_channels(CSI_PRP_VF_MEM, MEM_ROT_VF_MEM);
		if (err < 0) {
			printk(KERN_ERR
			       "Error link CSI_PRP_VF_MEM-MEM_ROT_VF_MEM\n");
			goto out_2;
		}

		memset(&params, 0, sizeof(ipu_channel_params_t));
		params.mem_dp_fg_sync.in_pixel_fmt = format;
		params.mem_dp_fg_sync.out_pixel_fmt = IPU_PIX_FMT_RGB24;
		err = ipu_init_channel(MEM_FG_SYNC, &params);
		if (err != 0)
			goto out_2;

		ipu_disp_set_window_pos(MEM_FG_SYNC, cam->win.w.left,
					cam->win.w.top);

		err = ipu_init_channel_buffer(MEM_FG_SYNC, IPU_INPUT_BUFFER,
					      format,
					      vf.csi_prp_vf_mem.out_height,
					      vf.csi_prp_vf_mem.out_width,
					      vf.csi_prp_vf_mem.out_height,
					      IPU_ROTATE_NONE,
					      cam->rot_vf_bufs[0],
					      cam->rot_vf_bufs[1], 0, 0);
		if (err != 0) {
			printk(KERN_ERR "Error initializing SDC FG buffer\n");
			goto out_2;
		}

		err = ipu_link_channels(MEM_ROT_VF_MEM, MEM_FG_SYNC);
		if (err < 0) {
			printk(KERN_ERR
			       "Error link MEM_ROT_VF_MEM-MEM_FG_SYNC\n");
			goto out_1;
		}

		ipu_enable_channel(CSI_PRP_VF_MEM);
		ipu_enable_channel(MEM_ROT_VF_MEM);
		ipu_enable_channel(MEM_FG_SYNC);

		ipu_select_buffer(CSI_PRP_VF_MEM, IPU_OUTPUT_BUFFER, 0);
		ipu_select_buffer(CSI_PRP_VF_MEM, IPU_OUTPUT_BUFFER, 1);
		ipu_select_buffer(MEM_ROT_VF_MEM, IPU_OUTPUT_BUFFER, 0);
		ipu_select_buffer(MEM_ROT_VF_MEM, IPU_OUTPUT_BUFFER, 1);
	} else {
		err = ipu_init_channel_buffer(CSI_PRP_VF_MEM, IPU_OUTPUT_BUFFER,
					      format, cam->win.w.width,
					      cam->win.w.height,
					      cam->win.w.width, cam->rotation,
					      cam->vf_bufs[0], cam->vf_bufs[1],
					      0, 0);
		if (err != 0) {
			printk(KERN_ERR "Error initializing CSI_PRP_VF_MEM\n");
			goto out_4;
		}
		memset(&params, 0, sizeof(ipu_channel_params_t));
		params.mem_dp_fg_sync.in_pixel_fmt = format;
		params.mem_dp_fg_sync.out_pixel_fmt = IPU_PIX_FMT_RGB24;
		err = ipu_init_channel(MEM_FG_SYNC, &params);
		if (err != 0)
			goto out_3;

		ipu_disp_set_window_pos(MEM_FG_SYNC, cam->win.w.left,
					cam->win.w.top);
		err = ipu_init_channel_buffer(MEM_FG_SYNC,
					      IPU_INPUT_BUFFER, format,
					      cam->win.w.width,
					      cam->win.w.height,
					      cam->win.w.width, IPU_ROTATE_NONE,
					      cam->vf_bufs[0], cam->vf_bufs[1],
					      0, 0);
		if (err != 0) {
			printk(KERN_ERR "Error initializing SDC FG buffer\n");
			goto out_1;
		}

		err = ipu_link_channels(CSI_PRP_VF_MEM, MEM_FG_SYNC);
		if (err < 0) {
			printk(KERN_ERR "Error linking ipu channels\n");
			goto out_1;
		}

		ipu_enable_channel(CSI_PRP_VF_MEM);
		ipu_enable_channel(MEM_FG_SYNC);

		ipu_select_buffer(CSI_PRP_VF_MEM, IPU_OUTPUT_BUFFER, 0);
		ipu_select_buffer(CSI_PRP_VF_MEM, IPU_OUTPUT_BUFFER, 1);
	}

	cam->overlay_active = true;
	return err;

      out_1:
	ipu_uninit_channel(MEM_FG_SYNC);
      out_2:
	if (cam->rotation >= IPU_ROTATE_90_RIGHT) {
		ipu_uninit_channel(MEM_ROT_VF_MEM);
	}
      out_3:
	ipu_uninit_channel(CSI_PRP_VF_MEM);
      out_4:
	if (cam->vf_bufs_vaddr[0]) {
		dma_free_coherent(0, cam->vf_bufs_size[0],
				  cam->vf_bufs_vaddr[0],
				  (dma_addr_t) cam->vf_bufs[0]);
		cam->vf_bufs_vaddr[0] = NULL;
		cam->vf_bufs[0] = 0;
	}
	if (cam->vf_bufs_vaddr[1]) {
		dma_free_coherent(0, cam->vf_bufs_size[1],
				  cam->vf_bufs_vaddr[1],
				  (dma_addr_t) cam->vf_bufs[1]);
		cam->vf_bufs_vaddr[1] = NULL;
		cam->vf_bufs[1] = 0;
	}
	if (cam->rot_vf_bufs_vaddr[0]) {
		dma_free_coherent(0, cam->rot_vf_buf_size[0],
				  cam->rot_vf_bufs_vaddr[0],
				  (dma_addr_t) cam->rot_vf_bufs[0]);
		cam->rot_vf_bufs_vaddr[0] = NULL;
		cam->rot_vf_bufs[0] = 0;
	}
	if (cam->rot_vf_bufs_vaddr[1]) {
		dma_free_coherent(0, cam->rot_vf_buf_size[1],
				  cam->rot_vf_bufs_vaddr[1],
				  (dma_addr_t) cam->rot_vf_bufs[1]);
		cam->rot_vf_bufs_vaddr[1] = NULL;
		cam->rot_vf_bufs[1] = 0;
	}
	return err;
}

/*!
 * prpvf_stop - stop the vf task
 *
 * @param private    cam_data * mxc v4l2 main structure
 *
 */
static int prpvf_stop(void *private)
{
	cam_data *cam = (cam_data *) private;
	int err = 0;

	if (cam->overlay_active == false)
		return 0;

	ipu_disp_set_window_pos(MEM_FG_SYNC, 0, 0);

	if (cam->rotation >= IPU_ROTATE_90_RIGHT) {
		ipu_unlink_channels(CSI_PRP_VF_MEM, MEM_ROT_VF_MEM);
		ipu_unlink_channels(MEM_ROT_VF_MEM, MEM_FG_SYNC);
	} else {
		ipu_unlink_channels(CSI_PRP_VF_MEM, MEM_FG_SYNC);
	}

	ipu_disable_channel(MEM_FG_SYNC, true);
	ipu_disable_channel(CSI_PRP_VF_MEM, true);

	if (cam->rotation >= IPU_ROTATE_90_RIGHT) {
		ipu_disable_channel(MEM_ROT_VF_MEM, true);
		ipu_uninit_channel(MEM_ROT_VF_MEM);
	}
	ipu_uninit_channel(MEM_FG_SYNC);
	ipu_uninit_channel(CSI_PRP_VF_MEM);

	ipu_csi_enable_mclk_if(CSI_MCLK_VF, cam->csi, false, false);

	if (cam->vf_bufs_vaddr[0]) {
		dma_free_coherent(0, cam->vf_bufs_size[0],
				  cam->vf_bufs_vaddr[0],
				  (dma_addr_t) cam->vf_bufs[0]);
		cam->vf_bufs_vaddr[0] = NULL;
		cam->vf_bufs[0] = 0;
	}
	if (cam->vf_bufs_vaddr[1]) {
		dma_free_coherent(0, cam->vf_bufs_size[1],
				  cam->vf_bufs_vaddr[1],
				  (dma_addr_t) cam->vf_bufs[1]);
		cam->vf_bufs_vaddr[1] = NULL;
		cam->vf_bufs[1] = 0;
	}
	if (cam->rot_vf_bufs_vaddr[0]) {
		dma_free_coherent(0, cam->rot_vf_buf_size[0],
				  cam->rot_vf_bufs_vaddr[0],
				  (dma_addr_t) cam->rot_vf_bufs[0]);
		cam->rot_vf_bufs_vaddr[0] = NULL;
		cam->rot_vf_bufs[0] = 0;
	}
	if (cam->rot_vf_bufs_vaddr[1]) {
		dma_free_coherent(0, cam->rot_vf_buf_size[1],
				  cam->rot_vf_bufs_vaddr[1],
				  (dma_addr_t) cam->rot_vf_bufs[1]);
		cam->rot_vf_bufs_vaddr[1] = NULL;
		cam->rot_vf_bufs[1] = 0;
	}

	cam->overlay_active = false;
	return err;
}

/*!
 * function to select PRP-VF as the working path
 *
 * @param private    cam_data * mxc v4l2 main structure
 *
 * @return  status
 */
int prp_vf_sdc_select(void *private)
{
	cam_data *cam;
	int err = 0;
	if (private) {
		cam = (cam_data *) private;
		cam->vf_start_sdc = prpvf_start;
		cam->vf_stop_sdc = prpvf_stop;
		cam->overlay_active = false;
	} else
		err = -EIO;

	return err;
}

/*!
 * function to de-select PRP-VF as the working path
 *
 * @param private    cam_data * mxc v4l2 main structure
 *
 * @return  int
 */
int prp_vf_sdc_deselect(void *private)
{
	cam_data *cam;
	int err = 0;
	err = prpvf_stop(private);

	if (private) {
		cam = (cam_data *) private;
		cam->vf_start_sdc = NULL;
		cam->vf_stop_sdc = NULL;
	}
	return err;
}

/*!
 * Init viewfinder task.
 *
 * @return  Error code indicating success or failure
 */
__init int prp_vf_sdc_init(void)
{
	return 0;
}

/*!
 * Deinit viewfinder task.
 *
 * @return  Error code indicating success or failure
 */
void __exit prp_vf_sdc_exit(void)
{
}

module_init(prp_vf_sdc_init);
module_exit(prp_vf_sdc_exit);

EXPORT_SYMBOL(prp_vf_sdc_select);
EXPORT_SYMBOL(prp_vf_sdc_deselect);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("IPU PRP VF SDC Driver");
MODULE_LICENSE("GPL");
