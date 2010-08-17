/*
 * Copyright 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file ipu_prp_vf_adc.c
 *
 * @brief IPU Use case for PRP-VF
 *
 * @ingroup IPU
 */

#include "mxc_v4l2_capture.h"
#include "ipu_prp_sw.h"
#include <mach/mxcfb.h>
#include <mach/ipu.h>
#include <linux/dma-mapping.h>

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
	ipu_channel_params_t vf;
	ipu_channel_params_t params;
	u32 format = IPU_PIX_FMT_RGB565;
	u32 size = 2;
	int err = 0;

	if (!cam) {
		printk(KERN_ERR "prpvf_start private is NULL\n");
		return -ENXIO;
	}

	if (cam->overlay_active == true) {
		printk(KERN_ERR "prpvf_start already start.\n");
		return 0;
	}

	mxcfb_set_refresh_mode(cam->overlay_fb, MXCFB_REFRESH_OFF, 0);

	memset(&vf, 0, sizeof(ipu_channel_params_t));
	ipu_csi_get_window_size(&vf.csi_prp_vf_adc.in_width,
				&vf.csi_prp_vf_adc.in_height);
	vf.csi_prp_vf_adc.in_pixel_fmt = IPU_PIX_FMT_UYVY;
	vf.csi_prp_vf_adc.out_width = cam->win.w.width;
	vf.csi_prp_vf_adc.out_height = cam->win.w.height;
	vf.csi_prp_vf_adc.graphics_combine_en = 0;
	vf.csi_prp_vf_adc.out_left = cam->win.w.left;

	/* hope to be removed when those offset taken cared by adc driver. */
#ifdef CONFIG_FB_MXC_EPSON_QVGA_PANEL
	vf.csi_prp_vf_adc.out_left += 12;
#endif
#ifdef CONFIG_FB_MXC_EPSON_PANEL
	vf.csi_prp_vf_adc.out_left += 2;
#endif

	vf.csi_prp_vf_adc.out_top = cam->win.w.top;

	if (cam->vf_rotation >= IPU_ROTATE_90_RIGHT) {
		vf.csi_prp_vf_adc.out_width = cam->win.w.height;
		vf.csi_prp_vf_adc.out_height = cam->win.w.width;

		size = cam->win.w.width * cam->win.w.height * size;
		vf.csi_prp_vf_adc.out_pixel_fmt = format;
		err = ipu_init_channel(CSI_PRP_VF_MEM, &vf);
		if (err != 0)
			return err;

		ipu_csi_enable_mclk_if(CSI_MCLK_VF, cam->csi, true, true);

		if (cam->vf_bufs_vaddr[0]) {
			dma_free_coherent(0, cam->vf_bufs_size[0],
					  cam->vf_bufs_vaddr[0],
					  cam->vf_bufs[0]);
		}
		if (cam->vf_bufs_vaddr[1]) {
			dma_free_coherent(0, cam->vf_bufs_size[1],
					  cam->vf_bufs_vaddr[1],
					  cam->vf_bufs[1]);
		}
		cam->vf_bufs_size[0] = size;
		cam->vf_bufs_vaddr[0] = (void *)dma_alloc_coherent(0,
								   cam->
								   vf_bufs_size
								   [0],
								   &cam->
								   vf_bufs[0],
								   GFP_DMA |
								   GFP_KERNEL);
		if (cam->vf_bufs_vaddr[0] == NULL) {
			printk(KERN_ERR
			       "prpvf_start: Error to allocate vf buffer\n");
			err = -ENOMEM;
			goto out_3;
		}
		cam->vf_bufs_size[1] = size;
		cam->vf_bufs_vaddr[1] = (void *)dma_alloc_coherent(0,
								   cam->
								   vf_bufs_size
								   [1],
								   &cam->
								   vf_bufs[1],
								   GFP_DMA |
								   GFP_KERNEL);
		if (cam->vf_bufs_vaddr[1] == NULL) {
			printk(KERN_ERR
			       "prpvf_start: Error to allocate vf buffer\n");
			err = -ENOMEM;
			goto out_3;
		}

		err = ipu_init_channel_buffer(CSI_PRP_VF_MEM, IPU_OUTPUT_BUFFER,
					      format,
					      vf.csi_prp_vf_mem.out_width,
					      vf.csi_prp_vf_mem.out_height,
					      vf.csi_prp_vf_mem.out_width,
					      IPU_ROTATE_NONE,
					      cam->vf_bufs[0], cam->vf_bufs[1],
					      0, 0);
		if (err != 0)
			goto out_3;

		if (cam->rot_vf_bufs[0]) {
			dma_free_coherent(0, cam->rot_vf_buf_size[0],
					  cam->rot_vf_bufs_vaddr[0],
					  cam->rot_vf_bufs[0]);
		}
		if (cam->rot_vf_bufs[1]) {
			dma_free_coherent(0, cam->rot_vf_buf_size[1],
					  cam->rot_vf_bufs_vaddr[1],
					  cam->rot_vf_bufs[1]);
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
			printk(KERN_ERR
			       "prpvf_start: Error to allocate rot_vf_bufs\n");
			err = -ENOMEM;
			goto out_3;
		}
		cam->rot_vf_buf_size[1] = PAGE_ALIGN(size);
		cam->rot_vf_bufs_vaddr[1] = (void *)dma_alloc_coherent(0,
								       cam->
								       rot_vf_buf_size
								       [1],
								       &cam->
								       rot_vf_bufs
								       [1],
								       GFP_DMA |
								       GFP_KERNEL);
		if (cam->rot_vf_bufs_vaddr[1] == NULL) {
			printk(KERN_ERR
			       "prpvf_start: Error to allocate rot_vf_bufs\n");
			err = -ENOMEM;
			goto out_3;
		}
		err = ipu_init_channel(MEM_ROT_VF_MEM, NULL);
		if (err != 0) {
			printk(KERN_ERR "prpvf_start :Error "
			       "MEM_ROT_VF_MEM channel\n");
			goto out_3;
		}

		err = ipu_init_channel_buffer(MEM_ROT_VF_MEM, IPU_INPUT_BUFFER,
					      format,
					      vf.csi_prp_vf_mem.out_width,
					      vf.csi_prp_vf_mem.out_height,
					      vf.csi_prp_vf_mem.out_width,
					      cam->vf_rotation, cam->vf_bufs[0],
					      cam->vf_bufs[1], 0, 0);
		if (err != 0) {
			printk(KERN_ERR "prpvf_start: Error "
			       "MEM_ROT_VF_MEM input buffer\n");
			goto out_2;
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
			printk(KERN_ERR "prpvf_start: Error "
			       "MEM_ROT_VF_MEM output buffer\n");
			goto out_2;
		}

		err = ipu_link_channels(CSI_PRP_VF_MEM, MEM_ROT_VF_MEM);
		if (err < 0) {
			printk(KERN_ERR "prpvf_start: Error "
			       "linking CSI_PRP_VF_MEM-MEM_ROT_VF_MEM\n");
			goto out_2;
		}

		ipu_disable_channel(ADC_SYS2, false);
		ipu_uninit_channel(ADC_SYS2);

		params.adc_sys2.disp = DISP0;
		params.adc_sys2.ch_mode = WriteTemplateNonSeq;
		params.adc_sys2.out_left = cam->win.w.left;
		/* going to be removed when those offset taken cared by adc driver. */
#ifdef CONFIG_FB_MXC_EPSON_QVGA_PANEL
		params.adc_sys2.out_left += 12;
#endif
#ifdef CONFIG_FB_MXC_EPSON_PANEL
		params.adc_sys2.out_left += 2;
#endif
		params.adc_sys2.out_top = cam->win.w.top;
		err = ipu_init_channel(ADC_SYS2, &params);
		if (err != 0) {
			printk(KERN_ERR
			       "prpvf_start: Error initializing ADC SYS1\n");
			goto out_2;
		}

		err = ipu_init_channel_buffer(ADC_SYS2, IPU_INPUT_BUFFER,
					      format,
					      vf.csi_prp_vf_mem.out_height,
					      vf.csi_prp_vf_mem.out_width,
					      vf.csi_prp_vf_mem.out_height,
					      IPU_ROTATE_NONE,
					      cam->rot_vf_bufs[0],
					      cam->rot_vf_bufs[1], 0, 0);
		if (err != 0) {
			printk(KERN_ERR "Error initializing ADC SYS1 buffer\n");
			goto out_1;
		}

		err = ipu_link_channels(MEM_ROT_VF_MEM, ADC_SYS2);
		if (err < 0) {
			printk(KERN_ERR
			       "Error linking MEM_ROT_VF_MEM-ADC_SYS2\n");
			goto out_1;
		}

		ipu_enable_channel(CSI_PRP_VF_MEM);
		ipu_enable_channel(MEM_ROT_VF_MEM);
		ipu_enable_channel(ADC_SYS2);

		ipu_select_buffer(CSI_PRP_VF_MEM, IPU_OUTPUT_BUFFER, 0);
		ipu_select_buffer(CSI_PRP_VF_MEM, IPU_OUTPUT_BUFFER, 1);
		ipu_select_buffer(MEM_ROT_VF_MEM, IPU_OUTPUT_BUFFER, 0);
		ipu_select_buffer(MEM_ROT_VF_MEM, IPU_OUTPUT_BUFFER, 1);
	}
#ifndef CONFIG_MXC_IPU_PRP_VF_SDC
	else if (cam->vf_rotation == IPU_ROTATE_NONE) {
		vf.csi_prp_vf_adc.out_pixel_fmt = IPU_PIX_FMT_BGR32;
		err = ipu_init_channel(CSI_PRP_VF_ADC, &vf);
		if (err != 0) {
			printk(KERN_ERR "prpvf_start: Error "
			       "initializing CSI_PRP_VF_ADC\n");
			return err;
		}
		ipu_csi_enable_mclk_if(CSI_MCLK_VF, cam->csi, true, true);
		err = ipu_init_channel_buffer(CSI_PRP_VF_ADC, IPU_OUTPUT_BUFFER,
					      format, cam->win.w.width,
					      cam->win.w.height,
					      cam->win.w.width, IPU_ROTATE_NONE,
					      0, 0, 0, 0);
		if (err != 0) {
			printk(KERN_ERR "prpvf_start: Error "
			       "initializing CSI_PRP_VF_MEM\n");
			return err;
		}
		ipu_enable_channel(CSI_PRP_VF_ADC);
	}
#endif
	else {
		size = cam->win.w.width * cam->win.w.height * size;
		vf.csi_prp_vf_adc.out_pixel_fmt = format;
		err = ipu_init_channel(CSI_PRP_VF_MEM, &vf);
		if (err != 0) {
			printk(KERN_ERR "prpvf_start: Error "
			       "initializing CSI_PRP_VF_MEM\n");
			return err;
		}

		ipu_csi_enable_mclk_if(CSI_MCLK_VF, cam->csi, true, true);

		if (cam->vf_bufs[0]) {
			dma_free_coherent(0, cam->vf_bufs_size[0],
					  cam->vf_bufs_vaddr[0],
					  cam->vf_bufs[0]);
		}
		if (cam->vf_bufs[1]) {
			dma_free_coherent(0, cam->vf_bufs_size[1],
					  cam->vf_bufs_vaddr[1],
					  cam->vf_bufs[1]);
		}
		cam->vf_bufs_size[0] = PAGE_ALIGN(size);
		cam->vf_bufs_vaddr[0] = (void *)dma_alloc_coherent(0,
								   cam->
								   vf_bufs_size
								   [0],
								   &cam->
								   vf_bufs[0],
								   GFP_DMA |
								   GFP_KERNEL);
		if (cam->vf_bufs_vaddr[0] == NULL) {
			printk(KERN_ERR
			       "prpvf_start: Error to allocate vf_bufs\n");
			err = -ENOMEM;
			goto out_3;
		}
		cam->vf_bufs_size[1] = PAGE_ALIGN(size);
		cam->vf_bufs_vaddr[1] = (void *)dma_alloc_coherent(0,
								   cam->
								   vf_bufs_size
								   [1],
								   &cam->
								   vf_bufs[1],
								   GFP_DMA |
								   GFP_KERNEL);
		if (cam->vf_bufs_vaddr[1] == NULL) {
			printk(KERN_ERR
			       "prpvf_start: Error to allocate vf_bufs\n");
			err = -ENOMEM;
			goto out_3;
		}
		err = ipu_init_channel_buffer(CSI_PRP_VF_MEM, IPU_OUTPUT_BUFFER,
					      format,
					      vf.csi_prp_vf_mem.out_width,
					      vf.csi_prp_vf_mem.out_height,
					      vf.csi_prp_vf_mem.out_width,
					      cam->vf_rotation,
					      cam->vf_bufs[0], cam->vf_bufs[1],
					      0, 0);
		if (err != 0) {
			printk(KERN_ERR "prpvf_start: Error "
			       "initializing CSI_PRP_VF_MEM\n");
			goto out_3;
		}

		ipu_disable_channel(ADC_SYS2, false);
		ipu_uninit_channel(ADC_SYS2);

		params.adc_sys2.disp = DISP0;
		params.adc_sys2.ch_mode = WriteTemplateNonSeq;
		params.adc_sys2.out_left = cam->win.w.left;
	/* going to be removed when those offset taken cared by adc driver.*/
#ifdef CONFIG_FB_MXC_EPSON_QVGA_PANEL
		params.adc_sys2.out_left += 12;
#endif
#ifdef CONFIG_FB_MXC_EPSON_PANEL
		params.adc_sys2.out_left += 2;
#endif
		params.adc_sys2.out_top = cam->win.w.top;
		err = ipu_init_channel(ADC_SYS2, &params);
		if (err != 0) {
			printk(KERN_ERR "prpvf_start: Error "
			       "initializing ADC_SYS2\n");
			goto out_3;
		}

		err = ipu_init_channel_buffer(ADC_SYS2, IPU_INPUT_BUFFER,
					      format,
					      vf.csi_prp_vf_mem.out_width,
					      vf.csi_prp_vf_mem.out_height,
					      vf.csi_prp_vf_mem.out_width,
					      IPU_ROTATE_NONE, cam->vf_bufs[0],
					      cam->vf_bufs[1], 0, 0);
		if (err != 0) {
			printk(KERN_ERR "prpvf_start: Error "
			       "initializing ADC SYS1 buffer\n");
			goto out_1;
		}

		err = ipu_link_channels(CSI_PRP_VF_MEM, ADC_SYS2);
		if (err < 0) {
			printk(KERN_ERR "prpvf_start: Error "
			       "linking MEM_ROT_VF_MEM-ADC_SYS2\n");
			goto out_1;
		}

		ipu_enable_channel(CSI_PRP_VF_MEM);
		ipu_enable_channel(ADC_SYS2);

		ipu_select_buffer(CSI_PRP_VF_MEM, IPU_OUTPUT_BUFFER, 0);
		ipu_select_buffer(CSI_PRP_VF_MEM, IPU_OUTPUT_BUFFER, 1);
	}

	cam->overlay_active = true;
	return err;

      out_1:
	ipu_uninit_channel(ADC_SYS2);
      out_2:
	if (cam->vf_rotation >= IPU_ROTATE_90_RIGHT) {
		ipu_uninit_channel(MEM_ROT_VF_MEM);
	}
      out_3:
	ipu_uninit_channel(CSI_PRP_VF_MEM);
	if (cam->rot_vf_bufs_vaddr[0]) {
		dma_free_coherent(0, cam->rot_vf_buf_size[0],
				  cam->rot_vf_bufs_vaddr[0],
				  cam->rot_vf_bufs[0]);
		cam->rot_vf_bufs_vaddr[0] = NULL;
		cam->rot_vf_bufs[0] = 0;
	}
	if (cam->rot_vf_bufs_vaddr[1]) {
		dma_free_coherent(0, cam->rot_vf_buf_size[1],
				  cam->rot_vf_bufs_vaddr[1],
				  cam->rot_vf_bufs[1]);
		cam->rot_vf_bufs_vaddr[1] = NULL;
		cam->rot_vf_bufs[1] = 0;
	}
	if (cam->vf_bufs_vaddr[0]) {
		dma_free_coherent(0, cam->vf_bufs_size[0],
				  cam->vf_bufs_vaddr[0], cam->vf_bufs[0]);
		cam->vf_bufs_vaddr[0] = NULL;
		cam->vf_bufs[0] = 0;
	}
	if (cam->vf_bufs_vaddr[1]) {
		dma_free_coherent(0, cam->vf_bufs_size[1],
				  cam->vf_bufs_vaddr[1], cam->vf_bufs[1]);
		cam->vf_bufs_vaddr[1] = NULL;
		cam->vf_bufs[1] = 0;
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

	if (cam->vf_rotation >= IPU_ROTATE_90_RIGHT) {
		ipu_unlink_channels(CSI_PRP_VF_MEM, MEM_ROT_VF_MEM);
		ipu_unlink_channels(MEM_ROT_VF_MEM, ADC_SYS2);

		ipu_disable_channel(CSI_PRP_VF_MEM, true);
		ipu_disable_channel(MEM_ROT_VF_MEM, true);
		ipu_disable_channel(ADC_SYS2, true);

		ipu_uninit_channel(CSI_PRP_VF_MEM);
		ipu_uninit_channel(MEM_ROT_VF_MEM);
		ipu_uninit_channel(ADC_SYS2);

		ipu_csi_enable_mclk_if(CSI_MCLK_VF, cam->csi, false, false);
	}
#ifndef CONFIG_MXC_IPU_PRP_VF_SDC
	else if (cam->vf_rotation == IPU_ROTATE_NONE) {
		ipu_disable_channel(CSI_PRP_VF_ADC, false);
		ipu_uninit_channel(CSI_PRP_VF_ADC);
		ipu_csi_enable_mclk_if(CSI_MCLK_VF, cam->csi, false, false);
	}
#endif
	else {
		ipu_unlink_channels(CSI_PRP_VF_MEM, ADC_SYS2);

		ipu_disable_channel(CSI_PRP_VF_MEM, true);
		ipu_disable_channel(ADC_SYS2, true);

		ipu_uninit_channel(CSI_PRP_VF_MEM);
		ipu_uninit_channel(ADC_SYS2);

		ipu_csi_enable_mclk_if(CSI_MCLK_VF, cam->csi, false, false);
	}

	if (cam->vf_bufs_vaddr[0]) {
		dma_free_coherent(0, cam->vf_bufs_size[0],
				  cam->vf_bufs_vaddr[0], cam->vf_bufs[0]);
		cam->vf_bufs_vaddr[0] = NULL;
		cam->vf_bufs[0] = 0;
	}
	if (cam->vf_bufs_vaddr[1]) {
		dma_free_coherent(0, cam->vf_bufs_size[1],
				  cam->vf_bufs_vaddr[1], cam->vf_bufs[1]);
		cam->vf_bufs_vaddr[1] = NULL;
		cam->vf_bufs[1] = 0;
	}
	if (cam->rot_vf_bufs_vaddr[0]) {
		dma_free_coherent(0, cam->rot_vf_buf_size[0],
				  cam->rot_vf_bufs_vaddr[0],
				  cam->rot_vf_bufs[0]);
		cam->rot_vf_bufs_vaddr[0] = NULL;
		cam->rot_vf_bufs[0] = 0;
	}
	if (cam->rot_vf_bufs_vaddr[1]) {
		dma_free_coherent(0, cam->rot_vf_buf_size[1],
				  cam->rot_vf_bufs_vaddr[1],
				  cam->rot_vf_bufs[1]);
		cam->rot_vf_bufs_vaddr[1] = NULL;
		cam->rot_vf_bufs[1] = 0;
	}

	cam->overlay_active = false;

	mxcfb_set_refresh_mode(cam->overlay_fb, MXCFB_REFRESH_PARTIAL, 0);
	return err;
}

/*!
 * function to select PRP-VF as the working path
 *
 * @param private    cam_data * mxc v4l2 main structure
 *
 * @return  status
 */
int prp_vf_adc_select(void *private)
{
	cam_data *cam;
	if (private) {
		cam = (cam_data *) private;
		cam->vf_start_adc = prpvf_start;
		cam->vf_stop_adc = prpvf_stop;
		cam->overlay_active = false;
	} else {
		return -EIO;
	}
	return 0;
}

/*!
 * function to de-select PRP-VF as the working path
 *
 * @param private    cam_data * mxc v4l2 main structure
 *
 * @return  status
 */
int prp_vf_adc_deselect(void *private)
{
	cam_data *cam;
	int err = 0;
	err = prpvf_stop(private);

	if (private) {
		cam = (cam_data *) private;
		cam->vf_start_adc = NULL;
		cam->vf_stop_adc = NULL;
	}
	return err;
}

/*!
 * Init viewfinder task.
 *
 * @return  Error code indicating success or failure
 */
__init int prp_vf_adc_init(void)
{
	return 0;
}

/*!
 * Deinit viewfinder task.
 *
 * @return  Error code indicating success or failure
 */
void __exit prp_vf_adc_exit(void)
{
}

module_init(prp_vf_adc_init);
module_exit(prp_vf_adc_exit);

EXPORT_SYMBOL(prp_vf_adc_select);
EXPORT_SYMBOL(prp_vf_adc_deselect);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("IPU PRP VF ADC Driver");
MODULE_LICENSE("GPL");
