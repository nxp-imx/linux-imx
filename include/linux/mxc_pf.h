/*
 * Copyright 2005-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU Lesser General
 * Public License.  You may obtain a copy of the GNU Lesser General
 * Public License Version 2.1 or later at the following locations:
 *
 * http://www.opensource.org/licenses/lgpl-license.html
 * http://www.gnu.org/copyleft/lgpl.html
 */

/*!
 * @defgroup MXC_PF MPEG4/H.264 Post Filter Driver
 */
/*!
 * @file arch-mxc/mxc_pf.h
 *
 * @brief MXC IPU MPEG4/H.264 Post-filtering driver
 *
 * User-level API for IPU Hardware MPEG4/H.264 Post-filtering.
 *
 * @ingroup MXC_PF
 */
#ifndef __INCLUDED_MXC_PF_H__
#define __INCLUDED_MXC_PF_H__

#define PF_MAX_BUFFER_CNT       17

#define PF_WAIT_Y	0x0001
#define PF_WAIT_U	0x0002
#define PF_WAIT_V	0x0004
#define PF_WAIT_ALL	(PF_WAIT_Y|PF_WAIT_U|PF_WAIT_V)

/*!
 * Structure for Post Filter initialization parameters.
 */
typedef struct {
	uint16_t pf_mode;	/*!< Post filter operation mode */
	uint16_t width;		/*!< Width of frame in pixels */
	uint16_t height;	/*!< Height of frame in pixels */
	uint16_t stride;	/*!< Stride of Y plane in pixels. Stride for U and V planes is half Y stride */
	uint32_t qp_size;
	unsigned long qp_paddr;
} pf_init_params;

/*!
 * Structure for Post Filter buffer request parameters.
 */
typedef struct {
	int count;		/*!< Number of buffers requested */
	__u32 req_size;
} pf_reqbufs_params;

/*!
 * Structure for Post Filter buffer request parameters.
 */
typedef struct {
	int index;
	int size;		/*!< Size of buffer allocated */
	__u32 offset;		/*!< Buffer offset in driver memory. Set by QUERYBUF */
	__u32 y_offset;		/*!< Optional starting relative offset of Y data
				   from beginning of buffer. Set to 0 to use default
				   calculated based on height and stride */
	__u32 u_offset;		/*!< Optional starting relative offset of U data
				   from beginning of buffer. Set to 0 to use default
				   calculated based on height and stride */
	__u32 v_offset;		/*!< Optional starting relative offset of V data
				   from beginning of buffer. Set to 0 to use default
				   calculated based on height and stride */
} pf_buf;

/*!
 * Structure for Post Filter start parameters.
 */
typedef struct {
	pf_buf in;		/*!< Input buffer address and offsets */
	pf_buf out;		/*!< Output buffer address and offsets */
	int qp_buf;
	int wait;
	uint32_t h264_pause_row;	/*!< Row to pause at for H.264 mode. 0 to disable pause */
} pf_start_params;

/*! @name User Client Ioctl Interface */
/*! @{ */

/*!
 * IOCTL to Initialize the Post Filter.
 */
#define PF_IOCTL_INIT           _IOW('F', 0x0, pf_init_params)

/*!
 * IOCTL to Uninitialize the Post Filter.
 */
#define PF_IOCTL_UNINIT         _IO('F', 0x1)

/*!
 * IOCTL to set the buffer mode and allocate buffers if driver allocated.
 */
#define PF_IOCTL_REQBUFS        _IOWR('F', 0x2, pf_reqbufs_params)

/*!
 * IOCTL to set the buffer mode and allocate buffers if driver allocated.
 */
#define PF_IOCTL_QUERYBUF       _IOR('F', 0x2, pf_buf)

/*!
 * IOCTL to start post filtering on a frame of data. This ioctl may block until
 * processing is done or return immediately.
 */
#define PF_IOCTL_START          _IOWR('F', 0x3, pf_start_params)

/*!
 * IOCTL to resume post-filtering after an intra frame pause in H.264 mode.
 */
#define PF_IOCTL_RESUME         _IOW('F', 0x4, int)

/*!
 * IOCTL to wait for post-filtering to complete.
 */
#define PF_IOCTL_WAIT           _IOW('F', 0x5, int)
/*! @} */

#endif				/* __INCLUDED_MXC_PF_H__ */
