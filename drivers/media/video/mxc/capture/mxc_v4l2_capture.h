/*
 * Copyright 2004-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @defgroup MXC_V4L2_CAPTURE MXC V4L2 Video Capture Driver
 */
/*!
 * @file mxc_v4l2_capture.h
 *
 * @brief mxc V4L2 capture device API  Header file
 *
 * It include all the defines for frame operations, also three structure defines
 * use case ops structure, common v4l2 driver structure and frame structure.
 *
 * @ingroup MXC_V4L2_CAPTURE
 */
#ifndef __MXC_V4L2_CAPTURE_H__
#define __MXC_V4L2_CAPTURE_H__

#include <asm/uaccess.h>
#include <linux/list.h>
#include <linux/smp_lock.h>
#include <linux/ipu.h>
#include <linux/mxc_v4l2.h>

#include <media/v4l2-dev.h>

#define FRAME_NUM 10

/*!
 * v4l2 frame structure.
 */
struct mxc_v4l_frame {
	u32 paddress;
	void *vaddress;
	int count;
	int width;
	int height;

	struct v4l2_buffer buffer;
	struct list_head queue;
	int index;
	int ipu_buf_num;
};

/* Only for old version.  Will go away soon. */
typedef struct {
	u8 clk_mode;
	u8 ext_vsync;
	u8 Vsync_pol;
	u8 Hsync_pol;
	u8 pixclk_pol;
	u8 data_pol;
	u8 data_width;
	u8 pack_tight;
	u8 force_eof;
	u8 data_en_pol;
	u16 width;
	u16 height;
	u32 pixel_fmt;
	u32 mclk;
	u16 active_width;
	u16 active_height;
} sensor_interface;

/* Sensor control function */
/* Only for old version.  Will go away soon. */
struct camera_sensor {
	void (*set_color) (int bright, int saturation, int red, int green,
			   int blue);
	void (*get_color) (int *bright, int *saturation, int *red, int *green,
			   int *blue);
	void (*set_ae_mode) (int ae_mode);
	void (*get_ae_mode) (int *ae_mode);
	sensor_interface *(*config) (int *frame_rate, int high_quality);
	sensor_interface *(*reset) (void);
	void (*get_std) (v4l2_std_id *std);
	void (*set_std) (v4l2_std_id std);
	unsigned int csi;
};

/*!
 * common v4l2 driver structure.
 */
typedef struct _cam_data {
	struct video_device *video_dev;
	int device_type;

	/* semaphore guard against SMP multithreading */
	struct semaphore busy_lock;

	int open_count;

	/* params lock for this camera */
	struct semaphore param_lock;

	/* Encoder */
	struct list_head ready_q;
	struct list_head done_q;
	struct list_head working_q;
	int ping_pong_csi;
	spinlock_t queue_int_lock;
	spinlock_t dqueue_int_lock;
	struct mxc_v4l_frame frame[FRAME_NUM];
	struct mxc_v4l_frame dummy_frame;
	int skip_frame;
	wait_queue_head_t enc_queue;
	int enc_counter;
	dma_addr_t rot_enc_bufs[2];
	void *rot_enc_bufs_vaddr[2];
	int rot_enc_buf_size[2];
	enum v4l2_buf_type type;

	/* still image capture */
	wait_queue_head_t still_queue;
	int still_counter;
	dma_addr_t still_buf[2];
	void *still_buf_vaddr;

	/* overlay */
	struct v4l2_window win;
	struct v4l2_framebuffer v4l2_fb;
	dma_addr_t vf_bufs[2];
	void *vf_bufs_vaddr[2];
	int vf_bufs_size[2];
	dma_addr_t rot_vf_bufs[2];
	void *rot_vf_bufs_vaddr[2];
	int rot_vf_buf_size[2];
	bool overlay_active;
	int output;
	struct fb_info *overlay_fb;
	int fb_origin_std;

	/* v4l2 format */
	struct v4l2_format v2f;
	int rotation;	/* for IPUv1 and IPUv3, this means encoder rotation */
	int vf_rotation; /* viewfinder rotation only for IPUv1 and IPUv3 */
	struct v4l2_mxc_offset offset;

	/* V4l2 control bit */
	int bright;
	int hue;
	int contrast;
	int saturation;
	int red;
	int green;
	int blue;
	int ae_mode;

	/* standard */
	struct v4l2_streamparm streamparm;
	struct v4l2_standard standard;
	bool standard_autodetect;

	/* crop */
	struct v4l2_rect crop_bounds;
	struct v4l2_rect crop_defrect;
	struct v4l2_rect crop_current;

	int (*enc_update_eba) (dma_addr_t eba, int *bufferNum);
	int (*enc_enable) (void *private);
	int (*enc_disable) (void *private);
	int (*enc_enable_csi) (void *private);
	int (*enc_disable_csi) (void *private);
	void (*enc_callback) (u32 mask, void *dev);
	int (*vf_start_adc) (void *private);
	int (*vf_stop_adc) (void *private);
	int (*vf_start_sdc) (void *private);
	int (*vf_stop_sdc) (void *private);
	int (*vf_enable_csi) (void *private);
	int (*vf_disable_csi) (void *private);
	int (*csi_start) (void *private);
	int (*csi_stop) (void *private);

	/* misc status flag */
	bool overlay_on;
	bool capture_on;
	int overlay_pid;
	int capture_pid;
	bool low_power;
	wait_queue_head_t power_queue;
	unsigned int csi;
	int current_input;

	/* camera sensor interface */
	struct camera_sensor *cam_sensor; 	/* old version */
	struct v4l2_int_device *sensor;
} cam_data;

#if defined(CONFIG_MXC_IPU_V1) || defined(CONFIG_VIDEO_MXC_EMMA_CAMERA) \
			       || defined(CONFIG_VIDEO_MXC_CSI_CAMERA_MODULE) \
			       || defined(CONFIG_VIDEO_MXC_CSI_CAMERA)
void set_mclk_rate(uint32_t *p_mclk_freq);
#else
void set_mclk_rate(uint32_t *p_mclk_freq, uint32_t csi);
#endif
#endif				/* __MXC_V4L2_CAPTURE_H__ */
